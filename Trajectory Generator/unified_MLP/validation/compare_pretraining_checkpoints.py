"""Compare deterministic unified rollout traces; Python standard library only.

From the repository root (the same command works on macOS and Windows)::

    python "Trajectory Generator/unified_MLP/validation/compare_pretraining_checkpoints.py"

Defaults select the six 2026-09-14 pilot rollouts and write comparison.json,
metrics.csv, aligned_samples.csv, and comparison.md under unified_MLP/runs/
validation/pretraining_v1_pen1_compare_3_5_2026-09-14. Use --help for overrides.
Missing files, missing required metrics, nonfinite measured values, mismatched
time grids or evaluation settings fail explicitly; no missing samples are filled.
Unrelated inactive reward diagnostics may be nonfinite and are not consumed.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


UNIFIED_DIR = Path(__file__).resolve().parents[1]
STARTS = ("early", "center", "late")
JOINTS = {"knee": "pros_knee_angle", "ankle": "pros_ankle_angle"}
TIME_TOL = 1e-7
RAD_TO_DEG = 180.0 / math.pi


def finite(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid numeric {field}: {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"Nonfinite {field}: {value!r}")
    return result


def value(row: dict, section: str, key: str) -> float:
    try:
        return finite(row[section][key], f"step {row.get('step')} {section}.{key}")
    except KeyError as exc:
        raise ValueError(f"Missing step {row.get('step')} {section}.{key}") from exc


def load_rollout(directory: Path) -> dict:
    summary_path = directory / "rollout_summary.json"
    trace_path = directory / "rollout_policy_trace.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    if summary.get("ok") is not True or not isinstance(rows, list) or not rows:
        raise ValueError(f"Incomplete or unsuccessful rollout: {directory}")
    if summary.get("action_selection") != "deterministic":
        raise ValueError(f"Expected deterministic action selection: {directory}")
    if summary.get("steps") != len(rows):
        raise ValueError(f"Summary/trace step count mismatch: {directory}")
    for i, row in enumerate(rows):
        if row.get("step") != i + 1:
            raise ValueError(f"Nonsequential step at row {i}: {directory}")
        times = row.get("policy_segment_times", [])
        if len(times) < 2:
            raise ValueError(f"Missing policy segment timestamps at step {i + 1}")
        pre = finite(times[0], "policy segment start")
        end = finite(row["time"], "poststep time")
        if abs(end - finite(times[-1], "policy segment end")) > TIME_TOL:
            raise ValueError(f"Poststep/segment timestamp mismatch at step {i + 1}")
        if end <= pre or (i and abs(pre - rows[i - 1]["time"]) > TIME_TOL):
            raise ValueError(f"Nonpositive or discontinuous sample interval at step {i + 1}")
        row["_pre_time"] = pre
        row["_dt"] = end - pre
        for joint in JOINTS.values():
            for key in (joint, joint + "_vel", joint + "_served_ref", joint + "_served_ref_vel"):
                value(row, "prosthetic_state", key)
            value(row, "imitation_target_q", joint)
            value(row, "imitation_target_qdot", joint)
    return {
        "directory": str(directory.resolve()), "summary": summary, "rows": rows,
        "source_hashes": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (summary_path, trace_path)},
    }


def common_rows(first: dict, second: dict) -> tuple[list, list]:
    a, b = first["rows"], second["rows"]
    if abs(a[0]["_pre_time"] - b[0]["_pre_time"]) > TIME_TOL:
        raise ValueError("Paired rollouts have different initial simulator times")
    end = min(a[-1]["time"], b[-1]["time"])
    a = [row for row in a if row["time"] <= end + TIME_TOL]
    b = [row for row in b if row["time"] <= end + TIME_TOL]
    if len(a) != len(b) or any(abs(x["time"] - y["time"]) > TIME_TOL for x, y in zip(a, b)):
        raise ValueError("Paired rollouts use different time grids; resampling is not implicit")
    for x, y in zip(a, b):
        for section in ("imitation_target_q", "imitation_target_qdot"):
            for joint in JOINTS.values():
                if abs(value(x, section, joint) - value(y, section, joint)) > 1e-6:
                    raise ValueError("Paired left prescribed targets differ at the same timestamp")
    return a, b


def validate_settings(first: dict, second: dict) -> None:
    keys = (
        "action_selection", "action_seed", "action_mode", "setup_xml_path",
        "episode_start_offset_s", "imitation_initialize_to_target", "actor_feature_names",
        "grf_mode", "online_grf_profile_file", "online_grf_detector_profile_file",
        "binary_phase_detector_profile_file", "phase_fsm_input_mode",
        "phase_sensor_on_threshold_n", "phase_sensor_off_threshold_n", "phase_sensor_dwell_s",
        "detector_sample_dt_s", "event_contract_id", "binary_phase_fsm_mode",
        "binary_phase_event_contract_id", "gait_clock_enable", "prescribed_grf_disabled_sides",
        "online_grf_applied_sides", "grf_penetration_penalty_threshold_m",
        "grf_penetration_termination_m", "reward_config",
    )
    for key in keys:
        if key not in first["summary"] or key not in second["summary"]:
            raise ValueError(f"Missing evaluation setting: {key}")
        if first["summary"][key] != second["summary"][key]:
            raise ValueError(f"Paired evaluation setting differs: {key}")


def weighted_mean(values: list[float], weights: list[float]) -> float:
    return math.fsum(v * w for v, w in zip(values, weights)) / math.fsum(weights)


def error_metrics(errors: list[float], weights: list[float], unit: str) -> dict:
    rmse = math.sqrt(weighted_mean([e * e for e in errors], weights))
    return {
        f"rmse_{unit}": rmse, f"mae_{unit}": weighted_mean([abs(e) for e in errors], weights),
        f"max_abs_{unit}": max(abs(e) for e in errors),
        f"bias_{unit}": weighted_mean(errors, weights),
        f"rmse_{'deg' if unit == 'rad' else 'deg_s'}": rmse * RAD_TO_DEG,
    }


def event_counters_at_window_end(rollout: dict, rows: list) -> dict[str, int]:
    """Validate observed counters, including the transition into this window.

    v3 can revoke a young heel strike after a bounced contact, so its HS counter
    is a net FSM count, not a monotonic tally of accepted transitions. Every
    observed decrease must be covered by contemporaneous cancellation evidence.
    The other counters remain monotonic. Legacy traces may omit cancellation
    telemetry entirely, but an unexplained decrease still fails explicitly.
    """
    previous_index = rows[0]["step"] - 2
    previous = rollout["rows"][previous_index] if previous_index >= 0 else None
    observed = ([previous] if previous else []) + rows
    cancellation_present = ["hs_cancelled_count" in row["phase_fsm"] for row in observed]
    if any(cancellation_present) and not all(cancellation_present):
        raise ValueError("Partially missing phase_fsm.hs_cancelled_count telemetry")
    keys = ["valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count"]
    if all(cancellation_present):
        keys.append("hs_cancelled_count")
    counts = {}
    for key in keys:
        values = [value(row, "phase_fsm", key) for row in observed]
        if any(number < 0 or not number.is_integer() for number in values):
            raise ValueError(f"Expected nonnegative integer episode event counter: {key}")
        counts[key] = [int(number) for number in values]
    for key, values in counts.items():
        for index, (before, after) in enumerate(zip(values, values[1:])):
            if after >= before:
                continue
            cancelled = counts.get("hs_cancelled_count")
            if (key == "valid_hs_count" and cancelled is not None
                    and before - after <= cancelled[index + 1] - cancelled[index]):
                continue
            raise ValueError(f"Nonmonotonic episode event counter: {key}")
    return {key: values[-1] for key, values in counts.items()}


def analyze_window(rollout: dict, rows: list, planned_duration: float, full: bool) -> dict:
    if not rows:
        raise ValueError("An evaluation window cannot be empty")
    summary = rollout["summary"]
    dt = [row["_dt"] for row in rows]
    duration = math.fsum(dt)
    result = {
        "samples": len(rows), "start_time_s": rows[0]["_pre_time"],
        "last_sample_time_s": rows[-1]["time"], "duration_s": duration,
        "planned_duration_s": planned_duration, "planned_coverage_fraction": duration / planned_duration,
        "episode_coverage_fraction": duration / math.fsum(row["_dt"] for row in rollout["rows"]),
        "episode_sample_coverage_fraction": len(rows) / len(rollout["rows"]),
        "tracking": {}, "sea": {},
    }
    for short, joint in JOINTS.items():
        target_q = [value(row, "imitation_target_q", joint) for row in rows]
        target_v = [value(row, "imitation_target_qdot", joint) for row in rows]
        tracking = {"prescribed_excursion_deg": (max(target_q) - min(target_q)) * RAD_TO_DEG}
        for source, suffix in (("actual", ""), ("served", "_served_ref")):
            q = [value(row, "prosthetic_state", joint + suffix) for row in rows]
            v = [value(row, "prosthetic_state", joint + suffix + "_vel") for row in rows]
            tracking[source + "_vs_prescribed"] = {
                "position": error_metrics([a - b for a, b in zip(q, target_q)], dt, "rad"),
                "velocity": error_metrics([a - b for a, b in zip(v, target_v)], dt, "rad_s"),
                "excursion_deg": (max(q) - min(q)) * RAD_TO_DEG,
            }
        tracking["actual_vs_served"] = {
            "position": error_metrics([
                value(row, "prosthetic_state", joint) - value(row, "prosthetic_state", joint + "_served_ref")
                for row in rows], dt, "rad"),
            "velocity": error_metrics([
                value(row, "prosthetic_state", joint + "_vel") - value(row, "prosthetic_state", joint + "_served_ref_vel")
                for row in rows], dt, "rad_s"),
        }
        result["tracking"][short] = tracking
        prefix = joint + "_sea_"
        counts = [value(row, "reward_terms", prefix + "sample_count") for row in rows]
        if any(n <= 0 for n in counts):
            raise ValueError(f"Missing internal SEA samples for {joint}")
        count = math.fsum(counts)
        sea = {"internal_samples": count}
        for metric, sum_suffix, unit in (
            ("tau_spring", "sum_squares_nm2", "nm"),
            ("torque_error", "sum_squares_nm2", "nm"),
            ("motor_speed", "sum_squares_rad2_s2", "rad_s"),
            ("motor_accel", "sum_squares_rad2_s4", "rad_s2"),
            ("motor_power", "sum_squares_w2", "w"),
        ):
            squares = [value(row, "reward_terms", prefix + metric + "_" + sum_suffix) for row in rows]
            if any(s < 0 for s in squares):
                raise ValueError(f"Negative SEA sum of squares for {joint}/{metric}")
            sea[metric + "_rms_" + unit] = math.sqrt(math.fsum(squares) / count)
            sea[metric + "_abs_max_" + unit] = max(
                value(row, "reward_terms", prefix + metric + "_abs_max_" + unit) for row in rows)
        sea["tau_input_abs_max_nm"] = max(value(row, "reward_terms", prefix + "tau_input_abs_max_nm") for row in rows)
        sea["tau_input_saturation_fraction"] = math.fsum(
            value(row, "reward_terms", prefix + "tau_input_saturation_count") for row in rows) / count
        for name in ("tau_input_plugin_fallback_count", "motor_accel_plugin_fallback_count"):
            sea[name] = math.fsum(value(row, "reward_terms", prefix + name) for row in rows)
        result["sea"][short] = sea
    phases = [value(row, "actor_observation_before", "online_left_cycle_duration_s") for row in rows]
    available = [p > 0 for p in phases]
    first_available = next((row["_pre_time"] - rows[0]["_pre_time"] for row, yes in zip(rows, available) if yes), None)
    result["phase"] = {
        "online_period_available_samples": sum(available),
        "online_period_available_fraction": weighted_mean([float(x) for x in available], dt),
        "first_online_period_available_elapsed_s": first_available,
        "online_sincos_unique_pairs": len({(
            value(row, "actor_observation_before", "online_left_gait_phase_sin"),
            value(row, "actor_observation_before", "online_left_gait_phase_cos")) for row in rows}),
        "prescribed_clock_zero_samples": sum(
            value(row, "actor_observation_before", "gait_phase_sin") == 0
            and value(row, "actor_observation_before", "gait_phase_cos") == 0 for row in rows),
    }
    result["phase"].update(event_counters_at_window_end(rollout, rows))
    penetration = [value(row, "reward_terms", "grf_penetration_m") for row in rows]
    threshold = finite(summary["grf_penetration_penalty_threshold_m"], "penetration threshold")
    result["safety"] = {
        "penetration_max_mm": max(penetration) * 1000.0,
        "penetration_threshold_mm": threshold * 1000.0,
        "penetration_above_threshold_policy_samples": sum(p > threshold for p in penetration),
        "penetration_above_threshold_policy_fraction": weighted_mean([float(p > threshold) for p in penetration], dt),
        "penetration_above_threshold_policy_interval_s": math.fsum(w for p, w in zip(penetration, dt) if p > threshold),
        "normalized_sea_u_abs_max": max(value(row, "reward_terms", "u_abs_max") for row in rows),
        "policy_action_clip_fraction_mean": weighted_mean([value(row, "reward_terms", "policy_action_clip_fraction") for row in rows], dt),
        "clipped_policy_steps": sum(value(row, "reward_terms", "policy_action_clip_fraction") > 0 for row in rows),
        "reserve_norm_max_nm": max(value(row, "reward_terms", "reserve_norm_nm") for row in rows),
    }
    result["reward"] = {
        "episode_return": finite(summary["episode_return"], "episode return") if full else None,
        "mean": finite(summary["reward_mean"], "episode reward mean") if full else None,
        "source": "rollout_summary" if full else "unavailable: scalar step reward is not stored in trace",
    }
    result["termination"] = {
        "end_reason": summary["end_reason"] if full else "common_prefix_window",
        "terminated": bool(summary["terminated"]) if full else None,
        "truncated": bool(summary["truncated"]) if full else None,
        "reached_planned_duration": duration >= planned_duration - TIME_TOL,
    }
    return result


def flatten(mapping: dict, prefix: str = "") -> dict:
    result = {}
    for key, item in mapping.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            result.update(flatten(item, name))
        else:
            result[name] = item
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def markdown_report(report: dict) -> str:
    first, second = report["iterations"]
    lines = [
        f"# Confronto deterministico pretraining: checkpoint {first} e {second}", "",
        "Le metriche descrivono questi sei rollout, senza stimare variabilità fra seed o parità con la baseline.",
        "Nessun punteggio aggregato o vincitore viene ricavato da episodi di durata differente.", "",
        "| Start | Checkpoint | Durata / prevista [s] | Fine | HS / TO / cicli validi | Fase online disponibile | Penetrazione max [mm] | Return |",
        "|---|---:|---:|---|---:|---:|---:|---:|",
    ]
    for case in report["cases"]:
        m = case["full_episode"]; phase = m["phase"]
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} | {m['duration_s']:.2f} / {m['planned_duration_s']:.2f} | {m['termination']['end_reason']} | {phase['valid_hs_count']} / {phase['valid_to_count']} / {phase['valid_cycle_count']} | {phase['online_period_available_fraction']:.1%} | {m['safety']['penetration_max_mm']:.2f} | {m['reward']['episode_return']:.3f} |")
    lines += ["", "## Tracking su finestre temporali comuni", "",
              "Actual, reference served e prescribed sinistro sono confrontati al medesimo timestamp post-step, senza shift o riallineamento di fase.",
              "RMSE ponderato per la durata del passo. Le finestre iniziano al reset e terminano al primo termine della coppia; la copertura resta esplicita.", "",
              "| Start | CP | Finestra [s] | Copertura prevista | Knee actual / served [°] | Ankle actual / served [°] | Knee vel. actual / served [°/s] | Ankle vel. actual / served [°/s] |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for case in report["cases"]:
        m = case["common_window"]
        cells = []
        for measure, unit in (("position", "deg"), ("velocity", "deg_s")):
            for joint in ("knee", "ankle"):
                t = m["tracking"][joint]
                cells.append(" / ".join(f"{t[source + '_vs_prescribed'][measure]['rmse_' + unit]:.2f}" for source in ("actual", "served")))
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} | {m['duration_s']:.2f} | {m['planned_coverage_fraction']:.1%} | " + " | ".join(cells) + " |")
    lines += ["", "## Inseguimento della reference servita", "",
              "RMSE actual–served sulle stesse finestre comuni. Misura quanto il giunto segue il riferimento dopo slew limiter e filtro C2.", "",
              "| Start | CP | Ginocchio [°] | Caviglia [°] |",
              "|---|---:|---:|---:|"]
    for case in report["cases"]:
        tracking = case["common_window"]["tracking"]
        knee = tracking["knee"]["actual_vs_served"]["position"]["rmse_deg"]
        ankle = tracking["ankle"]["actual_vs_served"]["position"]["rmse_deg"]
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} | {knee:.3f} | {ankle:.3f} |")
    lines += ["", "## Interpretazione e riproduzione", ""]
    lines += ["- " + item for item in report["limitations"]]
    lines += ["", "`comparison.json` e `metrics.csv` includono metriche complete e comuni, SEA e clipping. `aligned_samples.csv` conserva i segnali post-step utilizzati e gli ingressi di fase pre-step con timestamp distinto.", "",
              "Dalla root, su macOS o Windows:", "", "```console",
              'python "Trajectory Generator/unified_MLP/validation/compare_pretraining_checkpoints.py"', "```", ""]
    return "\n".join(lines)


def compare(args: argparse.Namespace) -> dict:
    rollouts = {}
    for start in STARTS:
        for iteration in args.iterations:
            name = args.directory_pattern.format(iteration=iteration, start=start, date=args.date)
            directory = args.rollout_root / name
            rollouts[(start, iteration)] = load_rollout(directory)
    report = {
        "schema_version": 1, "iterations": args.iterations, "date": args.date,
        "planned_duration_s": args.planned_duration_s, "cases": [],
        "limitations": [
            "Una traiettoria deterministica per checkpoint/start: non è una stima statistica della qualità o della convergenza.",
            "Una RMSE minore in un episodio più corto non dimostra un miglioramento: usare anche finestra comune, durata e cicli validi. Nessun campione viene imputato dopo una terminazione.",
            "La fase disponibile indica periodo online > 0 negli ingressi dell'actor pre-step; non misura accuratezza della fase né osservabilità completa. I target e il tracking sono post-step.",
            "HS/TO/cicli sono eventi accettati dalla FSM corrente. Zero cicli impedisce una conclusione sulla qualità del ciclo; nessuna normalizzazione sul ciclo è applicata.",
            "Il superamento della penetrazione è campionato tramite la diagnostica per passo policy: l'intervallo associato non equivale al tempo fisico esatto sopra soglia né al massimo continuo del contatto.",
            "Le metriche SEA RMS aggregano le somme dei quadrati e i conteggi dei campioni interni; la saturazione usa i conteggi interni, non la media semplice dei massimi.",
            "La reward scalare per passo non è presente nelle trace: return e media provengono solo dal summary completo; la reward della finestra comune non è ricostruita.",
            "Le misure richieste mancanti o non finite interrompono l'analisi. Diagnostiche di obiettivi inattivi non utilizzate possono essere non finite senza contaminare queste metriche.",
        ],
    }
    flat_rows, samples = [], []
    for start in STARTS:
        first, second = [rollouts[(start, iteration)] for iteration in args.iterations]
        validate_settings(first, second)
        common = common_rows(first, second)
        for iteration, rollout, common_subset in zip(args.iterations, (first, second), common):
            case = {
                "start": start, "checkpoint_iteration": iteration,
                "checkpoint": rollout["summary"]["checkpoint"],
                "episode_start_offset_s": rollout["summary"]["episode_start_offset_s"],
                "source_directory": rollout["directory"], "source_hashes": rollout["source_hashes"],
                "full_episode": analyze_window(rollout, rollout["rows"], args.planned_duration_s, True),
                "common_window": analyze_window(rollout, common_subset, args.planned_duration_s, False),
            }
            report["cases"].append(case)
            for window in ("full_episode", "common_window"):
                flat_rows.append({"start": start, "checkpoint_iteration": iteration, "window": window, **flatten(case[window])})
            for row in rollout["rows"]:
                sample = {
                    "start": start, "checkpoint_iteration": iteration, "step": row["step"],
                    "post_time_s": row["time"], "pre_observation_time_s": row["_pre_time"],
                    "elapsed_post_s": row["time"] - rollout["rows"][0]["_pre_time"],
                    "in_common_window": row["step"] <= len(common_subset),
                    "penetration_m": value(row, "reward_terms", "grf_penetration_m"),
                    "pre_online_period_s": value(row, "actor_observation_before", "online_left_cycle_duration_s"),
                    "post_fsm_state": row["phase_fsm"]["state_name"],
                }
                for short, joint in JOINTS.items():
                    for source, suffix in (("actual", ""), ("served", "_served_ref")):
                        sample[f"{short}_{source}_rad"] = value(row, "prosthetic_state", joint + suffix)
                        sample[f"{short}_{source}_rad_s"] = value(row, "prosthetic_state", joint + suffix + "_vel")
                    sample[f"{short}_prescribed_rad"] = value(row, "imitation_target_q", joint)
                    sample[f"{short}_prescribed_rad_s"] = value(row, "imitation_target_qdot", joint)
                samples.append(sample)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_csv(output / "metrics.csv", flat_rows)
    write_csv(output / "aligned_samples.csv", samples)
    (output / "comparison.md").write_text(markdown_report(report), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--date", default="2026-09-14")
    parser.add_argument("--iterations", nargs=2, type=int, default=[3, 5])
    parser.add_argument("--rollout-root", type=Path, default=UNIFIED_DIR / "runs" / "rollout")
    parser.add_argument("--directory-pattern", default="pretraining_v1_pen1_iter{iteration}_{start}_{date}")
    parser.add_argument("--output-dir", type=Path, default=UNIFIED_DIR / "runs" / "validation" / "pretraining_v1_pen1_compare_3_5_2026-09-14")
    parser.add_argument("--planned-duration-s", type=float, default=5.0)
    args = parser.parse_args()
    if len(set(args.iterations)) != 2 or not math.isfinite(args.planned_duration_s) or args.planned_duration_s <= 0:
        parser.error("Use two distinct checkpoint iterations and a finite positive planned duration")
    result = compare(args)
    print(json.dumps({"ok": True, "cases": len(result["cases"]), "output_dir": str(args.output_dir.resolve())}))


if __name__ == "__main__":
    main()
