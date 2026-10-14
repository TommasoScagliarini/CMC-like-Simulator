"""Render seven prescribed diagnostics from one explicitly selected finalized run.

Consumer only: no simulator, policy, inference, integration or run-file writes.
Figures 01--06 reuse root plot/plotter.py; figure 07 compares raw bundle IK,
healthy dataset and actual motion. All artifacts are saved under root plot/.
Use the same script separately for AB06 and AB07, with their explicit inputs.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import fields
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[3]
sys.dont_write_bytecode = True


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def project_path(raw: str | Path) -> Path:
    path = Path(raw)
    return (path if path.is_absolute() else REPO / path).resolve()


def relative(path: Path) -> str:
    try:
        return path.relative_to(REPO).as_posix()
    except ValueError:
        return str(path)


def inventory(directory: Path) -> dict:
    return {relative(path): {"size_bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in sorted(directory.rglob("*")) if path.is_file()}


def load_plotter():
    spec = importlib.util.spec_from_file_location("paired_prescribed_plotter", REPO / "plot/plotter.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_storage(path: Path, plotter):
    """Strict whitespace-delimited reader, also for raw IK .mot files."""
    np = plotter.np
    with path.open(encoding="utf-8-sig") as stream:
        metadata = {}
        for line in stream:
            if line.strip().lower() == "endheader":
                break
            if "=" in line:
                key, value = line.split("=", 1)
                metadata[key.strip().lower()] = value.strip()
        else:
            raise ValueError(f"Missing endheader: {path}")
        labels = next(line for line in stream if line.strip()).split()
        values = np.loadtxt(stream, ndmin=2)
    if not labels or labels[0].lower() != "time" or len(labels) != len(set(labels)):
        raise ValueError(f"Invalid storage labels: {path}")
    if values.shape[0] < 2 or values.shape[1] != len(labels):
        raise ValueError(f"Invalid storage dimensions: {path}")
    if not np.all(np.isfinite(values)) or not np.all(np.diff(values[:, 0]) > 0):
        raise ValueError(f"Nonfinite data or nonmonotone time: {path}")
    for key, expected in (("nrows", values.shape[0]), ("ncolumns", values.shape[1])):
        if key in metadata and int(metadata[key]) != expected:
            raise ValueError(f"Header {key} disagrees with data: {path}")
    return plotter.StoTable(path, values[:, 0], labels[1:], values[:, 1:],
                            metadata.get("indegrees", "no").lower() == "yes")


def selected_cycles(path: Path, window: tuple[float, float]) -> tuple[list, list, list]:
    """Preserve original rows/timestamps; require exact recorded support."""
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        if names is None or not {"side", "cycle_start", "cycle_end"}.issubset(names):
            raise ValueError("Events CSV must include side,cycle_start,cycle_end")
        rows = list(reader)
    kept, excluded = [], []
    for row in rows:
        start, end = float(row["cycle_start"]), float(row["cycle_end"])
        if row["side"].strip().lower() not in {"left", "right", "ankle", "knee", "all"}:
            raise ValueError(f"Unsupported event side: {row['side']}")
        if not all(math.isfinite(t) for t in (start, end)) or end <= start:
            raise ValueError(f"Invalid cycle: {row}")
        if start >= window[0] and end <= window[1]:
            kept.append(row)
        else:
            excluded.append({"row": row, "reason": "outside_common_recorded_support"})
    return names, kept, excluded


def figure7(args, plotter, tables: dict, out_dir: Path, missing, outer: str, inner: str) -> dict:
    """Raw-IK diagnostic, intentionally distinct from the served reference in 06."""
    np, plt = plotter.np, plotter.plt
    actual, ik, healthy = tables["actual"], tables["IK"], tables["healthy"]
    time = actual.time
    for name, table in tables.items():
        if table.time[0] > time[0] or table.time[-1] < time[-1]:
            raise ValueError(f"{name} does not cover the actual motion recording")
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    metrics = {}

    def series(table, coordinate):
        raw = table.series([coordinate])
        if raw is None:
            return None
        factor = 1.0 if table.in_degrees else 180.0 / np.pi
        sign = -1.0 if coordinate in {"pros_knee_angle", "knee_angle_r"} else 1.0
        return np.interp(time, raw[0], raw[1]) * factor * sign

    for column, (label, prosthetic, biological) in enumerate([
        ("Knee", "pros_knee_angle", "knee_angle_r"),
        ("Ankle", "pros_ankle_angle", "ankle_angle_r"),
    ]):
        axes[0, column].set_title(label)
        signals = {}
        for row, coordinate in ((0, prosthetic), (1, biological)):
            for name, color, style, legend in [
                ("healthy", "0.5", ":", f"{args.subject} healthy dataset remap"),
                ("IK", "tab:orange", "--", "Raw prosthetic-model IK"),
                ("actual", "tab:blue", "-", "Actual simulated motion"),
            ]:
                values = series(tables[name], coordinate)
                signals[name, coordinate] = values
                if values is None:
                    missing.add(f"figure 7 / {coordinate} / {name}: missing coordinate")
                    plotter.annotate_missing(axes[row, column], f"{name}: {coordinate}")
                else:
                    axes[row, column].plot(time, values, color=color, linestyle=style, label=legend)
        actual_q, ik_q, healthy_q = (signals[n, prosthetic] for n in ("actual", "IK", "healthy"))
        joint_metrics = {}
        if actual_q is not None and ik_q is not None:
            error = actual_q - ik_q
            axes[2, column].plot(time, error, color="tab:blue", label="Actual − raw IK")
            joint_metrics["actual_vs_raw_ik_rmse_deg"] = float(np.sqrt(np.mean(error ** 2)))
            joint_metrics["actual_vs_raw_ik_max_abs_deg"] = float(np.max(np.abs(error)))
        if ik_q is not None and healthy_q is not None:
            delta = ik_q - healthy_q
            axes[2, column].plot(time, delta, color="tab:orange", label="Raw IK − healthy left")
            joint_metrics["raw_ik_vs_healthy_rmse_deg"] = float(np.sqrt(np.mean(delta ** 2)))
        metrics[prosthetic] = joint_metrics
        actual_right, ik_right = signals["actual", biological], signals["IK", biological]
        if actual_right is not None and ik_right is not None:
            error = actual_right - ik_right
            metrics[biological] = {"actual_vs_raw_ik_rmse_deg": float(np.sqrt(np.mean(error ** 2))),
                                    "actual_vs_raw_ik_max_abs_deg": float(np.max(np.abs(error)))}
        axes[2, column].axhline(0, linewidth=0.8, color="black", alpha=0.4)
        axes[2, column].set_xlabel("Time [s]")
    for row, label in enumerate(["Prosthetic left angle [deg]", "Biological right angle [deg]", "Difference [deg]"]):
        axes[row, 0].set_ylabel(label)
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8)
    top = plotter.apply_figure_header(fig, "Healthy Dataset, Raw Prosthetic-model IK and Actual Motion", outer, inner)
    fig.tight_layout(rect=(0, 0, 1, top))
    plotter.save_figure(fig, out_dir, "07_prescribed_healthy_ik_actual.png")
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", required=True, choices=["AB06", "AB07"])
    for name in ("run-dir", "receipt", "results-dir", "setup", "model", "reference",
                 "healthy-kinematics", "healthy-forces", "events", "out-root"):
        parser.add_argument(f"--{name}", required=True, type=project_path)
    parser.add_argument("--prefix", default="sim_output")
    parser.add_argument("--gait-side", choices=["left", "right"], default="left")
    parser.add_argument("--events-description", default="Measured-GRF reference cycles")
    parser.add_argument("--check-only", action="store_true", help="Validate inputs without writing any artifacts")
    parser.add_argument("--allow-finalized-failed", action="store_true",
                        help="Explicitly admit the terminal FSM sample-boundary error only with a bound numerical audit proving the full physical recording")
    parser.add_argument("--numerical-audit", type=project_path,
                        help="Required with --allow-finalized-failed; finalized numerical_verification.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.out_root.is_relative_to(REPO / "plot") or args.out_root == REPO / "plot":
        raise ValueError("Use a dedicated --out-root subdirectory under repository root plot/")
    if args.out_root.is_relative_to(args.run_dir) or not args.results_dir.is_relative_to(args.run_dir):
        raise ValueError("Results must belong to the explicit run; plots must be outside it")
    receipt = json.loads(args.receipt.read_text(encoding="utf-8"))
    if receipt.get("subject") != args.subject:
        raise ValueError("Receipt must attest the explicitly requested subject")
    failed_audit = None
    if receipt.get("status") == "completed":
        if receipt.get("completed_requested_duration") is not True:
            raise ValueError("Requested prescribed duration is not attested complete")
    elif receipt.get("status") == "failed" and args.allow_finalized_failed:
        if not args.numerical_audit or not receipt.get("finished_utc"):
            raise ValueError("Finalized failed rendering requires the numerical audit and finished receipt")
        if "Policy boundaries must span a positive integer number of samples" not in receipt.get("error", ""):
            raise ValueError("Failure is outside the explicitly admitted terminal adapter case")
        failed_audit = json.loads(args.numerical_audit.read_text(encoding="utf-8"))
        if (failed_audit.get("subject") != args.subject
                or project_path(failed_audit["run"]) != args.run_dir
                or failed_audit["receipt"]["sha256"] != sha256(args.receipt)):
            raise ValueError("Numerical audit is not bound to the exact failed run receipt")
        checks = ("clocks_at_requested_end", "recorded_numerical_data_available", "all_sto_finite",
                  "all_sto_same_time_grid", "all_sto_cover_requested_episode",
                  "all_sto_match_contiguous_recorded_prefix", "all_sto_dt_matches_captured_config",
                  "source_fingerprints_all_match_receipt")
        if any(failed_audit["summary"].get(key) is not True for key in checks):
            raise ValueError("Numerical audit does not attest the full coherent physical recording")
    else:
        raise ValueError("Completed receipt required unless the explicit audited finalized-failed option applies")
    episode_start = float(receipt["actual_episode_start_s"])
    episode_end = float(receipt["actual_episode_end_s"])
    duration = episode_end - episode_start
    if not all(math.isfinite(t) for t in (episode_start, episode_end)) or abs(duration - 9.0) > 1e-7:
        raise ValueError("Paired receipt must attest the actual nine-second episode window")
    physical_end = float(receipt["final_runner_time_s"])
    physical_duration = physical_end - episode_start
    for key in ("final_time_s", "final_runner_time_s", "final_state_time_s"):
        clock = float(receipt[key])
        if not math.isfinite(clock) or abs(clock - episode_end) > 1e-7:
            raise ValueError(f"Final physical/logical clock does not attest nine seconds: {key}")
    effective = receipt["effective_simulator_config"]
    plotter = load_plotter()
    required = {f.name for f in fields(plotter.SimulatorConfig) if f.init}
    if set(effective) != required:
        raise ValueError(f"Incomplete/noncurrent config snapshot: missing={sorted(required-set(effective))}, extra={sorted(set(effective)-required)}")
    cfg = plotter.SimulatorConfig(**effective)
    resolved = plotter.resolve_simulator_paths(cfg)
    if resolved.model_path.resolve() != args.model or resolved.kinematics_path.resolve() != args.reference:
        raise ValueError("Explicit model/IK disagree with the effective run configuration")
    setup = plotter.read_setup_xml(args.setup)
    if setup.model_file.resolve() != args.model or setup.kinematics_file.resolve() != args.reference:
        raise ValueError("Explicit setup selects a different model/IK")
    if cfg.output_prefix != args.prefix or project_path(cfg.output_dir) != args.results_dir:
        raise ValueError("Results path/prefix disagree with the effective run configuration")
    if cfg.pros_coords != ["pros_knee_angle", "pros_ankle_angle"]:
        raise ValueError("Canonical plot adapter requires the preserved SEA coordinate topology")

    missing = plotter.MissingReport()
    table_keys = {"sea_torques": "sea_torques", "sea_controls": "sea_controls", "states": "states",
                  "kinematics_reference": "kinematics_reference", "reserve_torques": "reserve_torques",
                  "sea_states": "sea_states", "sea_diagnostics": "sea_diagnostics", "power": "power"}
    tables = {key: read_storage(args.results_dir / f"{args.prefix}_{suffix}.sto", plotter)
              if (args.results_dir / f"{args.prefix}_{suffix}.sto").is_file() else None
              for key, suffix in table_keys.items()}
    actual = read_storage(args.results_dir / f"{args.prefix}_kinematics.sto", plotter)
    healthy_kin = read_storage(args.healthy_kinematics, plotter)
    healthy_force = read_storage(args.healthy_forces, plotter)
    raw_ik = read_storage(args.reference, plotter)
    healthy = plotter.HealthyData(args.healthy_kinematics.parent, healthy_kin, healthy_force,
                                  ["Healthy velocity: finite differences; power: torque × derived velocity"])
    supports = [table for table in tables.values() if table is not None] + [actual, healthy_kin, healthy_force]
    window = max(float(t.time[0]) for t in supports), min(float(t.time[-1]) for t in supports)
    if window[1] <= window[0]:
        raise ValueError("Plot channels have no common recorded interval")
    names, cycles, excluded = selected_cycles(args.events, window)
    by_side = {side: sum(row["side"].strip().lower() == side for row in cycles)
               for side in ("left", "right", "ankle", "knee", "all")}
    if args.check_only:
        print(json.dumps({"status": "INPUTS_VALID", "subject": args.subject,
                          "receipt_status": receipt["status"], "allow_finalized_failed_used": failed_audit is not None,
                          "common_recorded_range_s": window, "cycles_by_side": by_side,
                          "excluded_cycles": len(excluded)}, indent=2, allow_nan=False))
        return 0

    before = inventory(args.run_dir)
    out_dir = plotter.next_output_dir(args.out_root)
    event_copy = out_dir / "reference_cycles_in_recorded_window.csv"
    with event_copy.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(cycles)
    events = plotter.load_events(event_copy, missing)
    # The recorded served reference includes the runtime segment wrapper and
    # governor. Raw/filtered IK cannot replace a missing served reference in 06.
    recorded_reference = plotter.recorded_reference_angle_series

    def supported_recorded_reference(table, time, coord, side):
        if table is None or table.series([f"{coord}_q_ref"]) is None:
            missing.add(f"figure 6 / {side} / served reference: recorded channel unavailable")
            return None
        if table.time[0] > time[0] or table.time[-1] < time[-1]:
            missing.add(f"figure 6 / {side} / served reference: recorded channel does not cover actual states")
            return None
        return recorded_reference(table, time, coord, side)

    plotter.recorded_reference_angle_series = supported_recorded_reference
    outer = plotter.outer_loop_subtitle(cfg)
    inner_params = plotter.load_sea_params(cfg)
    inner = plotter.inner_loop_subtitle(inner_params, cfg)
    run_label = "FAILED terminal adapter" if failed_audit is not None else "completed"
    prefix = (f"{args.subject} paired prescribed | IK-driven HS-aligned | requested {duration:g} s; "
              f"physical recorded {physical_duration:g} s | {run_label}")
    original_save = plotter.save_figure

    def labelled_save(fig, destination, filename):
        title = fig._suptitle.get_text() if fig._suptitle is not None else ""
        fig.suptitle(f"{prefix}\n{title}", fontsize=12, y=0.995)
        header_texts = [text for text in fig.texts if text is not fig._suptitle and text.get_position()[1] > 0.9]
        for text, y in zip(header_texts, (0.943, 0.921)):
            text.set_position((text.get_position()[0], y))
        for ax in fig.axes:
            if filename.startswith(("03_", "04_")):
                if ax.get_xlabel() == "gait cycle [%]":
                    ax.set_xlabel("Measured-GRF reference phase [%]")
                for line in ax.lines:
                    label = line.get_label()
                    if label.lower().startswith("healthy"):
                        line.set_label(label.replace("healthy", f"{args.subject} healthy dataset", 1))
                if ax.get_legend_handles_labels()[0]:
                    ax.legend(loc="best", fontsize=8)
            if filename.startswith("06_"):
                for line in ax.lines:
                    label = line.get_label()
                    line.set_label(label.replace("served kinematic ref", "recorded served IK-driven reference"))
                if ax.get_legend_handles_labels()[0]:
                    ax.legend(loc="best", fontsize=8)
        fig.tight_layout(rect=(0, 0, 1, 0.865))
        original_save(fig, destination, filename)

    plotter.save_figure = labelled_save
    plotter.plot_figure_1(tables, out_dir, missing, outer, inner)
    plotter.plot_figure_2(tables, out_dir, missing, outer, inner)
    plotter.plot_figure_3(tables, events, args.gait_side, healthy, out_dir, missing, outer, inner)
    plotter.plot_figure_4(tables, events, args.gait_side, healthy, out_dir, missing, outer, inner)
    plotter.plot_figure_5(tables, plotter.load_sea_f_opt(cfg, missing), out_dir, missing, outer, inner)
    plotter.plot_figure_6(tables, None, out_dir, missing, outer, inner)
    metrics = figure7(args, plotter, {"actual": actual, "IK": raw_ik, "healthy": healthy_kin},
                      out_dir, missing, outer, inner)
    missing.write(out_dir / "missing_channels.txt")
    after = inventory(args.run_dir)
    if before != after:
        changed = sorted(set(before) ^ set(after) | {p for p in before.keys() & after.keys() if before[p] != after[p]})
        raise ValueError(f"Run artifacts changed during rendering: {changed}")
    pngs = sorted(out_dir.glob("*.png"))
    if len(pngs) != 7:
        raise ValueError(f"Expected seven canonical PNGs, got {len(pngs)}")
    png_info = {}
    for png in pngs:
        pixels = plotter.plt.imread(png)
        png_info[png.name] = {"size_bytes": png.stat().st_size, "sha256": sha256(png),
                              "width": int(pixels.shape[1]), "height": int(pixels.shape[0])}
    inputs = [args.receipt, args.setup, args.model, args.reference, args.healthy_kinematics,
              args.healthy_forces, args.events, actual.path]
    if failed_audit is not None:
        inputs.append(args.numerical_audit)
    inputs += [t.path for t in tables.values() if t is not None]
    sources = [Path(__file__), REPO / "plot/plotter.py", REPO / "config.py", REPO / "output.py",
               REPO / "kinematics_interpolator.py", REPO / "path_resolver.py", REPO / "setup_io.py"]
    provenance = {
        "status": "RENDERED_PENDING_VISUAL_REVIEW", "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Seven prescribed diagnostics only; rendering is not a biomechanical qualification",
        "subject": args.subject, "run": relative(args.run_dir), "folder": relative(out_dir),
        "receipt_status": receipt["status"],
        "completed_requested_duration_from_receipt": receipt.get("completed_requested_duration", False),
        "terminal_adapter_error_from_receipt": receipt.get("error"), "receipt_steps": receipt["steps"],
        "allow_finalized_failed_used": failed_audit is not None,
        "failed_recording_numerical_audit": ({"path": relative(args.numerical_audit),
                                               "sha256": sha256(args.numerical_audit),
                                               "scope": "Full recorded physics and final clocks; failed receipt and missing terminal adapter trace remain unchanged"}
                                              if failed_audit is not None else None),
        "command": [sys.executable, "-B", relative(Path(__file__)), *sys.argv[1:]],
        "png_count": 7, "files": png_info, "missing_channels": missing.items,
        "effective_config_source": relative(args.receipt) + "/effective_simulator_config",
        "model_inner_loop_parameters": inner_params,
        "recorded_time_range_s": [float(actual.time[0]), float(actual.time[-1])],
        "actual_episode_time_range_s": [episode_start, episode_end], "actual_episode_duration_s": duration,
        "recorded_physical_duration_s_from_runner_clock": physical_duration,
        "final_clocks_from_receipt_s": {key: receipt[key] for key in ("final_time_s", "final_runner_time_s", "final_state_time_s")},
        "recorded_rows": len(actual.time), "common_recorded_range_s": window,
        "events": {"source": relative(args.events), "description": args.events_description,
                   "plot_only_csv": relative(event_copy), "cycles_by_side": by_side,
                   "used_cycles": cycles, "excluded_rows": excluded, "timestamp_changes": False,
                   "selection": "Entire original cycle must lie inside all recorded and healthy channels",
                   "same_actual_healthy_boundaries": True,
                   "actor_fsm_counts_are_separate": True, "new_events_fabricated": False},
        "figure6_semantics": "Actual vs recorded served IK-driven reference; missing/partial reference is annotated without raw/filtered IK fallback",
        "figure7_semantics": "Healthy remap, raw bundle IK and actual motion; knee flexion=-native, ankle=native, degrees",
        "figure7_metrics": metrics, "inputs_sha256": {relative(p): sha256(p) for p in inputs},
        "source_scripts_sha256": {relative(p): sha256(p) for p in sources},
        "run_files_unchanged_after_plotting": {"file_count": len(before), "files": before, "passed": True},
        "latest_rollout_discovery_used": False, "last_setup_discovery_used": False,
        "figure6_prescribed_reference_fallback_used": False,
        "runtime_code_changes": [], "new_integrations_for_plotting": 0,
        "visual_review": {p.name: "pending" for p in pngs},
    }
    (out_dir / "plot_rendering_manifest.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": provenance["status"], "folder": relative(out_dir), "png_count": 7,
                      "cycles_by_side": by_side, "missing_channels": missing.items,
                      "run_files_unchanged": len(before)}, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
