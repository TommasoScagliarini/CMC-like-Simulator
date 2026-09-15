"""Evaluate immutable V1 imitation checkpoints at logical iteration 50 or 100.

Analysis/selection uses the standard library; complete reports also use matplotlib.
Without --execute-rollouts this command only analyzes existing rollout files, or
writes an explicit pending evaluation plan. It never launches training. Example::

    python "Trajectory Generator/unified_MLP/validation/evaluate_training_milestone.py" \
        --run-dir RUN --milestone 50 --execute-rollouts

Each unique candidate receives three deterministic seed-123, 500-step rollouts.
The highest training return is a selection rule, never a qualification verdict.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

try:
    from . import compare_pretraining_checkpoints as analysis
except ImportError:
    import compare_pretraining_checkpoints as analysis


UNIFIED_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = UNIFIED_DIR.parents[1]
START_OFFSETS = dict(zip(analysis.STARTS, (1.756870983805102, 1.956870983805102, 2.156870983805102)))
PLANNED_DURATION = 5.0
POLICY_DT = 0.01
SEED = 123
MAX_STEPS = 500
EVENT_KEYS = ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count")
REPORT_REQUEST_FILENAME = "post_evaluation_report_request.json"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data) -> None:
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def select_candidates(run_dir: Path, milestone: int) -> dict:
    """Freeze successful history through the milestone; ties choose earliest.

    JSONL is authoritative when present, including during a resumed process.
    Duplicate logical iterations or a missing milestone fail rather than silently
    evaluating a different model. Mutable rl_module_best/last are never selected.
    """
    history_path = run_dir / "train_iterations.jsonl"
    if history_path.exists():
        history = [json.loads(line) for line in history_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    else:
        history_path = run_dir / "summary.json"
        history = read_json(history_path)["history"]
    selected_history = {}
    for row in history:
        iteration = int(row["iteration"])
        if iteration > milestone:
            continue
        if iteration in selected_history:
            raise ValueError(f"Duplicate logical iteration {iteration} in {history_path}")
        selected_history[iteration] = row
    if milestone not in selected_history:
        raise ValueError(f"Logical milestone {milestone} has not completed")
    finite_history = []
    for iteration, row in sorted(selected_history.items()):
        score = row.get("episode_return_mean")
        if score is not None and math.isfinite(float(score)):
            finite_history.append({"iteration": iteration, "episode_return_mean": float(score)})
    if not finite_history:
        raise ValueError("No finite training episode_return_mean through milestone")
    best = max(finite_history, key=lambda row: (row["episode_return_mean"], -row["iteration"]))
    roles = {"last": milestone, "best_training_return": best["iteration"]}
    candidates = []
    for iteration in sorted(set(roles.values())):
        artifact = run_dir / f"milestone_iteration_{iteration:06d}"
        checkpoint = artifact / "rl_module_last"
        metadata_path = artifact / "rl_module_last_meta.json"
        metadata = read_json(metadata_path)
        if int(metadata["logical_iteration"]) != iteration:
            raise ValueError(f"Checkpoint metadata iteration mismatch: {artifact}")
        state = read_json(checkpoint / "unified_training_state.json")
        if int(state["last_completed_iteration"]) != iteration or state["reward_mode"] != "imitation":
            raise ValueError(f"Expected completed imitation state at iteration {iteration}: {artifact}")
        contract = read_json(checkpoint / "unified_contract.json")
        if len(contract["actor_feature_names"]) != 35:
            raise ValueError(f"Expected V1 35-feature actor: {artifact}")
        candidates.append({
            "iteration": iteration, "roles": [role for role, index in roles.items() if index == iteration],
            "checkpoint": str(checkpoint.resolve()),
            "module_state_sha256": sha256(checkpoint / "module_state.pkl"),
            "contract_sha256": sha256(checkpoint / "unified_contract.json"),
            "metadata_sha256": sha256(metadata_path),
            "training_episode_return_mean": next((row["episode_return_mean"] for row in finite_history if row["iteration"] == iteration), None),
        })
    return {"milestone": milestone, "selection_rule": "maximum finite episode_return_mean through milestone; earliest iteration wins exact ties",
            "best_is_automatically_qualified": False, "history_source": str(history_path.resolve()),
            "history_through_milestone": finite_history, "roles": roles, "candidates": candidates}


def rollout_command(python: str, candidate: dict, start: str, output: Path, config: Path) -> list[str]:
    return [python, str(UNIFIED_DIR / "rollout_eval.py"), "--checkpoint", candidate["checkpoint"],
            "--config", str(config), "--output-dir", str(output),
            "--episode-start-offset-s", repr(START_OFFSETS[start]), "--episode-duration", "5.0",
            "--segment-duration", "0.01", "--max-steps", str(MAX_STEPS), "--seed", str(SEED),
            "--action-selection", "deterministic", "--record-policy-trace", "--record-outputs", "--no-progress"]


def validate_rollout(rollout: dict, candidate: dict, start: str) -> None:
    summary, rows = rollout["summary"], rollout["rows"]
    if summary["action_seed"] != SEED or len(rows) > MAX_STEPS:
        raise ValueError("Rollout seed or maximum step budget differs")
    if abs(summary["episode_start_offset_s"] - START_OFFSETS[start]) > analysis.TIME_TOL:
        raise ValueError(f"Rollout start differs: {start}")
    if summary.get("record_policy_trace") is not True or summary.get("record_outputs") is not True:
        raise ValueError("Milestone evaluation requires policy trace and simulator outputs")
    if summary.get("gait_clock_enable") is not False or len(summary["actor_feature_names"]) != 35:
        raise ValueError("Expected 35 actor features with prescribed gait clock disabled")
    if summary["reward_config"].get("reward_mode") != "imitation":
        raise ValueError("Expected imitation reward")
    if any(abs(row["_dt"] - POLICY_DT) > analysis.TIME_TOL for row in rows):
        raise ValueError("Expected 10 ms policy intervals")
    actual_checkpoint = Path(summary["checkpoint"])
    if not actual_checkpoint.is_dir():
        # An evaluation can be moved together with its run between supported OSes.
        # Its original absolute path is then unavailable; the immutable iteration
        # plus the sidecar identity must establish provenance at the caller.
        raise ValueError(f"Rollout checkpoint unavailable for content verification: {actual_checkpoint}")
    if sha256(actual_checkpoint / "module_state.pkl") != candidate["module_state_sha256"]:
        raise ValueError("Rollout checkpoint weights differ from selected immutable artifact")
    if sha256(Path(candidate["checkpoint"]) / "module_state.pkl") != candidate["module_state_sha256"]:
        raise ValueError("Selected immutable checkpoint changed during evaluation")
    if summary.get("n_actor", 35) != 35:
        raise ValueError("Unexpected actor width")
    config_path = Path(rollout["directory"]) / "training_cfg.resolved.yaml"
    rollout["source_hashes"][config_path.name] = sha256(config_path)


def runtime_config_lines(path: Path) -> list[str]:
    """Compare emitted flat YAML runtime blocks without a YAML dependency.

    Training iterations and the sampler's list of reset choices do not change a
    deterministic evaluation at an explicitly selected reset. All other emitted
    simulation, GRF and reward fields must match exactly. Unknown fields within
    these blocks remain included (fail-closed for future runtime additions).
    """
    selected, section, skip_child = [], None, False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line.startswith(" "):
            section = line.split(":", 1)[0]
            skip_child = False
        elif line.startswith("  ") and not line.startswith("   ") and not line.lstrip().startswith("-"):
            child = line.strip().split(":", 1)[0]
            skip_child = section == "simulation" and child in ("iterations", "episode_start_offset_choices_s")
        if section in ("simulation", "grf", "reward") and not skip_child:
            selected.append(line.rstrip())
    if not selected:
        raise ValueError(f"Missing runtime config blocks: {path}")
    return selected


def validate_pair(first: dict, second: dict) -> None:
    analysis.validate_settings(first, second)
    configs = [runtime_config_lines(Path(rollout["directory"]) / "training_cfg.resolved.yaml") for rollout in (first, second)]
    if configs[0] != configs[1]:
        raise ValueError("Paired resolved simulation/GRF/reward settings differ")


def window_metrics(rollout: dict, rows: list, *, full: bool = False) -> dict | None:
    """Analyze a fixed window; signed counter changes exclude its prefix.

    V3 can cancel a previously credited heel strike. Its net HS change may be
    negative in a subwindow; cancellation counts remain a separate diagnostic.
    """
    if not rows:
        return None
    result = analysis.analyze_window(rollout, rows, PLANNED_DURATION, full)
    previous_index = rows[0]["step"] - 2
    previous = rollout["rows"][previous_index] if previous_index >= 0 else None
    keys = EVENT_KEYS + (("hs_cancelled_count",) if "hs_cancelled_count" in result["phase"] else ())
    for key in keys:
        result["phase"][key + "_cumulative_at_end"] = result["phase"][key]
        result["phase"][key] -= int(analysis.value(previous, "phase_fsm", key)) if previous else 0
    result["phase"]["event_count_semantics"] = (
        "Signed change in FSM counters over the window. valid_hs_count is net of "
        "v3 bounce cancellations, including cancellation of an HS before the window; "
        "hs_cancelled_count is reported separately when present in the trace."
    )
    penetrations = [analysis.value(row, "reward_terms", "grf_penetration_m") for row in rows]
    duration = result["duration_s"]
    result["safety"]["penetration_thresholds_mm"] = {}
    for threshold_mm in (20, 25, 28):
        over = [row for row, p in zip(rows, penetrations) if p > threshold_mm / 1000.0]
        interval = math.fsum(row["_dt"] for row in over)
        result["safety"]["penetration_thresholds_mm"][str(threshold_mm)] = {
            "policy_samples_above": len(over), "policy_interval_s_above": interval,
            "policy_fraction_above": interval / duration,
        }
    origin = rollout["rows"][0]["_pre_time"]
    result["elapsed_start_s"] = rows[0]["_pre_time"] - origin
    result["elapsed_end_s"] = rows[-1]["time"] - origin
    if not full:
        result["termination"]["end_reason"] = "observed_window"
    return result


def describe_case(rollout: dict, iteration: int, start: str, roles: list[str]) -> dict:
    rows = rollout["rows"]
    first_available = next((i for i, row in enumerate(rows) if analysis.value(row, "actor_observation_before", "online_left_cycle_duration_s") > 0), None)
    split = first_available if first_available is not None else len(rows)
    return {"start": start, "checkpoint_iteration": iteration, "roles": roles,
            "source_directory": rollout["directory"], "source_hashes": rollout["source_hashes"],
            "full_episode": window_metrics(rollout, rows, full=True),
            "startup": window_metrics(rollout, rows[:split]),
            "online_period_regime": window_metrics(rollout, rows[split:]),
            "regime_definition": "from first pre-step online period > 0; availability does not establish a stable gait",
            "first_online_period_available_elapsed_s": (rows[split]["_pre_time"] - rows[0]["_pre_time"]) if split < len(rows) else None}


def quality_gate(cases: list[dict]) -> dict:
    """Conservative necessary screen. Tracking remains per-joint descriptive data."""
    failures = []
    by_start = {case["start"]: case for case in cases}
    if len(cases) != len(START_OFFSETS) or set(by_start) != set(START_OFFSETS):
        failures.append("missing_or_duplicate_start")
    for start, case in by_start.items():
        m = case["full_episode"]
        checks = {
            "duration_5s": m["duration_s"] >= PLANNED_DURATION - analysis.TIME_TOL,
            "two_valid_cycles": m["phase"]["valid_cycle_count"] >= 2,
            "online_period_observed": case["first_online_period_available_elapsed_s"] is not None,
            "no_invalid_events": m["phase"]["invalid_event_count"] == 0,
            "prescribed_clock_zero": m["phase"]["prescribed_clock_zero_samples"] == m["samples"],
            "normal_episode_completion": m["termination"]["end_reason"] == "episode_time_limit" and not m["termination"]["terminated"],
            "penetration_below_28mm": m["safety"]["penetration_max_mm"] < 28.0,
            "normalized_sea_control_bounded": m["safety"]["normalized_sea_u_abs_max"] <= 1.0 + 1e-6,
            "motor_torque_bounded": all(sea["tau_input_abs_max_nm"] <= 500.0 + 1e-6 for sea in m["sea"].values()),
        }
        failures.extend(f"{start}:{name}" for name, passed in checks.items() if not passed)
    return {"pass": not failures, "failures": failures,
            "scope": "necessary deterministic feasibility screen; not tracking qualification, statistical robustness or baseline parity",
            "criteria": "all three starts: 5 s, >=2 valid cycles, online period, zero invalid events, disabled prescribed clock, normal completion, penetration <28 mm, |u|<=1, motor |tau_input|<=500 Nm",
            "tracking_thresholds": "none imposed; knee and ankle position/velocity errors reported separately on complete and common-prefix windows"}


def historical_reference(run_dir: Path, evaluated: dict) -> tuple[dict, list[dict]]:
    """Reuse CP5 only with matching weights and per-start evaluation settings."""
    results, exclusions = {}, []
    checkpoint = run_dir / "milestone_iteration_000005" / "rl_module_last"
    if not (checkpoint / "module_state.pkl").is_file():
        return results, [{"reason": "immutable CP5 weights unavailable"}]
    candidate = {"checkpoint": str(checkpoint), "module_state_sha256": sha256(checkpoint / "module_state.pkl")}
    for start in START_OFFSETS:
        anchor = next((r for (iteration, label), r in evaluated.items() if label == start), None)
        if anchor is None:
            continue
        for path in sorted((UNIFIED_DIR / "runs" / "rollout").glob(f"pretraining_v1_pen1_iter5_{start}_*")):
            try:
                rollout = analysis.load_rollout(path)
                validate_rollout(rollout, candidate, start)
                validate_pair(anchor, rollout)
                analysis.common_rows(anchor, rollout)
                results[(5, start)] = rollout
                break
            except (OSError, ValueError, KeyError) as exc:
                exclusions.append({"directory": str(path), "reason": str(exc)})
        if (5, start) not in results:
            exclusions.append({"start": start, "reason": "no compatible historical CP5 rollout"})
    return results, exclusions


def previous_milestone_reference(run_dir: Path, output: Path, evaluated: dict) -> tuple[dict, list[dict]]:
    """Reuse both candidates already evaluated at 50, preserving source hashes."""
    references, exclusions = {}, []
    search_roots = {UNIFIED_DIR / "runs" / "validation", output.parent}
    reports = sorted({path for root in search_roots for path in root.rglob("milestone_evaluation.json") if path.parent != output})
    matches = []
    for path in reports:
        report = read_json(path)
        if report.get("selection", {}).get("milestone") != 50 or report.get("status") != "complete":
            continue
        if Path(report.get("plan", {}).get("run_dir", "")).resolve() == run_dir.resolve():
            matches.append((path, report))
    if not matches:
        raise ValueError("Milestone 100 evaluation requires a completed milestone 50 evaluation for this run")
    # Do not silently pick a different candidate set from duplicate evaluation reports.
    expected = matches[0][1]["selection"]["roles"]
    if any(report["selection"]["roles"] != expected for _, report in matches):
        raise ValueError("Conflicting milestone 50 candidate selections")
    path, report = matches[0]
    for candidate in report["selection"]["candidates"]:
        iteration = candidate["iteration"]
        for start in START_OFFSETS:
            if (iteration, start) in evaluated:
                evaluated[(iteration, start)].setdefault("_reference_roles", []).append("reference_milestone50")
                continue
            case = next(c for c in report["cases"] if c["checkpoint_iteration"] == iteration and c["start"] == start)
            directory = Path(case["source_directory"])
            for filename, digest in case["source_hashes"].items():
                if sha256(directory / filename) != digest:
                    raise ValueError(f"Milestone 50 reference changed: {directory / filename}")
            rollout = analysis.load_rollout(directory)
            validate_rollout(rollout, candidate, start)
            anchor = next(r for (i, label), r in evaluated.items() if label == start)
            validate_pair(anchor, rollout)
            analysis.common_rows(anchor, rollout)
            rollout["_reference_roles"] = ["reference_milestone50"]
            rollout["_reference_report"] = str(path)
            references[(iteration, start)] = rollout
    return references, exclusions


def plot_tracking(output_dir: Path, rollouts: dict, milestone: int) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    files = []
    for start in START_OFFSETS:
        selected = sorted((iteration, rollout) for (iteration, label), rollout in rollouts.items() if label == start)
        fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
        fig.suptitle(f"Milestone {milestone} · {start} · deterministic seed {SEED}")
        reference = max((rollout for _, rollout in selected), key=lambda r: len(r["rows"]))
        origin = reference["rows"][0]["_pre_time"]
        common_end = min(r["rows"][-1]["time"] for _, r in selected) - origin
        for joint_index, (short, joint) in enumerate(analysis.JOINTS.items()):
            for column, (measure, target_section, suffix, unit) in enumerate((
                ("Position", "imitation_target_q", "", "deg"),
                ("Velocity", "imitation_target_qdot", "_vel", "deg/s"),
            )):
                ax = axes[joint_index, column]
                times = [row["time"] - origin for row in reference["rows"]]
                ax.plot(times, [analysis.value(row, target_section, joint) * analysis.RAD_TO_DEG for row in reference["rows"]], color="black", lw=1.5, label="prescribed left")
                for index, (iteration, rollout) in enumerate(selected):
                    rows = rollout["rows"]
                    times = [row["time"] - origin for row in rows]
                    color = f"C{index}"
                    for source, state_suffix, style in (("actual", suffix, "-"), ("served", "_served_ref" + suffix, "--")):
                        ax.plot(times, [analysis.value(row, "prosthetic_state", joint + state_suffix) * analysis.RAD_TO_DEG for row in rows], color=color, ls=style, lw=1.2, label=f"CP{iteration} {source}")
                    if times[-1] < PLANNED_DURATION - analysis.TIME_TOL:
                        ax.axvline(times[-1], color=color, ls=":", alpha=0.5)
                ax.axvspan(0, common_end, color="grey", alpha=0.07)
                ax.set(title=f"{short.capitalize()} · {measure}", ylabel=unit, xlim=(0, PLANNED_DURATION))
                ax.grid(alpha=0.15)
                if joint_index == 1:
                    ax.set_xlabel("Elapsed from reset [s]")
        axes[0, 0].legend(fontsize=8, ncol=2)
        details = [f"CP{i}: {r['rows'][-1]['time'] - origin:.2f}s, {r['summary']['end_reason']}, {r['summary']['phase_valid_cycle_count']} cycles" for i, r in selected]
        fig.text(0.04, 0.025, " | ".join(details), fontsize=8)
        fig.text(0.04, 0.005, "Shading: common observed prefix. Post-step alignment; no extrapolation, phase shift or survivor pooling.", fontsize=8)
        fig.tight_layout(rect=(0, 0.06, 1, 0.94))
        for suffix in ("png", "pdf"):
            path = output_dir / f"tracking_{start}.{suffix}"
            fig.savefig(path, dpi=160, facecolor="white")
            files.append(path.name)
        plt.close(fig)
    return files


def markdown_report(report: dict) -> str:
    lines = [f"# Valutazione milestone {report['selection']['milestone']}", "",
             "Best indica il massimo return del training entro la milestone (parità: iterazione più precoce). Non implica qualificazione.", "",
             "| Start | CP | Ruolo | Durata [s] | Fine | HS/TO/cicli | Primo periodo online [s] | Pen max [mm] |",
             "|---|---:|---|---:|---|---:|---:|---:|"]
    for case in report["cases"]:
        m = case["full_episode"]; p = m["phase"]
        first = case["first_online_period_available_elapsed_s"]
        lines.append(f"| {case['start']} | {case['checkpoint_iteration']} | {', '.join(case['roles'])} | {m['duration_s']:.2f} | {m['termination']['end_reason']} | {p['valid_hs_count']}/{p['valid_to_count']}/{p['valid_cycle_count']} | {first if first is not None else 'non disponibile'} | {m['safety']['penetration_max_mm']:.2f} |")
    lines += ["", "## Screening conservativo", ""]
    for iteration, gate in report["quality_gates"].items():
        lines.append(f"- CP{iteration}: {'superato' if gate['pass'] else 'non superato'}. " + (", ".join(gate["failures"]) or gate["scope"]))
    lines += ["", "Questo screening richiede tutti e tre gli start a 5 s, almeno due cicli validi e i vincoli documentati nel JSON. Non assegna una soglia clinica o biomeccanica al tracking.", "",
              "## Tracking sulle finestre comuni", "",
              "RMSE ponderata per durata, separata per giunto e start. Ogni coppia termina al primo stop osservato; nessun campione viene inventato dopo la terminazione.", "",
              "| Start | Coppia | CP | Finestra [s] | Ginocchio actual/served [°] | Caviglia actual/served [°] | Ginocchio vel. actual/served [°/s] | Caviglia vel. actual/served [°/s] |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for pair in report["common_prefix_comparisons"]:
        for iteration, m in pair["metrics"].items():
            cells = []
            for measure, unit in (("position", "deg"), ("velocity", "deg_s")):
                for joint in analysis.JOINTS:
                    t = m["tracking"][joint]
                    cells.append(" / ".join(f"{t[source + '_vs_prescribed'][measure]['rmse_' + unit]:.2f}" for source in ("actual", "served")))
            lines.append(f"| {pair['start']} | {pair['iterations']} | {iteration} | {m['duration_s']:.2f} | " + " | ".join(cells) + " |")
    lines += ["", "## Startup, regime e sicurezza", "",
              "Startup termina prima della prima osservazione pre-step con periodo online >0; il regime successivo indica disponibilità del periodo, senza assumere stabilità del passo. Se il periodo non arriva, startup comprende tutto l'episodio e regime è nullo.", "",
              "Il JSON e metrics.csv includono tutte le finestre, actual–served, velocità, escursioni, SEA RMS su campioni interni, saturazione e soglie di penetrazione 20/25/28 mm. Le sottofinestre riportano le variazioni dei contatori FSM. HS è il conteggio netto: la FSM v3 può annullare un appoggio per rimbalzo; una sottofinestra può quindi avere variazione HS negativa se annulla un appoggio precedente. Le cancellazioni sono riportate separatamente in hs_cancelled_count quando presenti nel trace. Un calo HS è ammesso soltanto se coperto da cancellazioni registrate; gli altri contatori restano monotoni.", "",
              "Gli intervalli sopra soglia sono campionati per passo policy e non misurano la durata continua esatta del contatto. Un solo seed deterministico non stima robustezza o parità con la baseline. Non si confrontano return di finestre incomplete né si aggregano gli start sopravvissuti.", ""]
    if report["historical_reference_exclusions"]:
        lines += ["Riferimenti storici esclusi:", ""] + ["- " + json.dumps(item, ensure_ascii=False) for item in report["historical_reference_exclusions"]]
    lines += ["", "Figure:", ""] + [f"- [{name}]({name})" for name in report["figures"] if name.endswith(".png")]
    return "\n".join(lines) + "\n"


def evaluate(args: argparse.Namespace) -> dict:
    run_dir = args.run_dir.resolve()
    output = (args.output_dir or UNIFIED_DIR / "runs" / "validation" / f"{run_dir.name}_milestone_{args.milestone}").resolve()
    selection = select_candidates(run_dir, args.milestone)
    output.mkdir(parents=True, exist_ok=True)
    source_config = run_dir / "training_cfg.resolved.yaml"
    config = output / "training_cfg.resolved.yaml"
    if not config.exists():
        shutil.copyfile(source_config, config)
    # The copied evaluation config is intentionally retained after training resumes.
    # Provenance below freezes it; inference never silently adopts later edits.
    plan = {"schema_version": 1, "run_dir": str(run_dir), "selection": selection,
            "evaluation_config": str(config), "evaluation_config_sha256": sha256(config),
            "planned_duration_s": PLANNED_DURATION, "seed": SEED, "max_steps": MAX_STEPS,
            "starts": START_OFFSETS, "commands": []}
    evaluated, pending = {}, []
    # rollout_eval deliberately confines output to its own category. A digest
    # prevents two custom validation paths with the same basename from colliding.
    output_id = hashlib.sha256(str(output).encode()).hexdigest()[:10]
    rollout_root = UNIFIED_DIR / "runs" / "rollout" / f"{output.name}_{output_id}"
    plan["rollout_root"] = str(rollout_root)

    def collect(candidate: dict, start: str, reference_role: str | None = None) -> None:
        directory = rollout_root / f"iteration_{candidate['iteration']:06d}_{start}"
        command = rollout_command(args.python, candidate, start, directory, config)
        identity = {"iteration": candidate["iteration"], "module_state_sha256": candidate["module_state_sha256"],
                    "config_sha256": plan["evaluation_config_sha256"], "start": start,
                    "offset_s": START_OFFSETS[start], "seed": SEED, "max_steps": MAX_STEPS,
                    "planned_duration_s": PLANNED_DURATION, "policy_dt_s": POLICY_DT}
        plan["commands"].append({"iteration": candidate["iteration"], "start": start, "argv": command, "identity": identity})
        identity_path = directory / "evaluation_identity.json"
        if identity_path.exists() and read_json(identity_path) != identity:
            raise ValueError(f"Existing evaluation identity differs: {directory}")
        summary_path = directory / "rollout_summary.json"
        if summary_path.exists() and not identity_path.exists():
            raise ValueError(f"Existing rollout lacks evaluation identity: {directory}")
        if not summary_path.exists() and args.execute_rollouts:
            directory.mkdir(parents=True, exist_ok=True)
            write_json(identity_path, identity)
            write_json(output / "evaluation_plan.json", plan)
            print(f"Evaluating CP{candidate['iteration']} / {start}", flush=True)
            with (directory / "evaluation_console.log").open("a", encoding="utf-8") as log:
                result = subprocess.run(command, cwd=REPO_DIR, stdout=log, stderr=subprocess.STDOUT, check=False)
            if result.returncode:
                raise RuntimeError(f"Rollout failed ({result.returncode}); see {directory / 'evaluation_console.log'}")
        if not summary_path.exists():
            pending.append({"iteration": candidate["iteration"], "start": start, "directory": str(directory)})
            return
        rollout = analysis.load_rollout(directory)
        validate_rollout(rollout, candidate, start)
        if reference_role:
            rollout["_reference_roles"] = [reference_role]
        evaluated[(candidate["iteration"], start)] = rollout

    for candidate in selection["candidates"]:
        for start in START_OFFSETS:
            collect(candidate, start)
    plan["pending_rollouts"] = pending
    write_json(output / "evaluation_plan.json", plan)
    if pending:
        return {"ok": False, "status": "pending_rollouts", "output_dir": str(output), "pending_rollouts": pending,
                "instruction": "Run again with --execute-rollouts to execute the recorded commands."}
    reference, exclusions = historical_reference(run_dir, evaluated)
    for key, rollout in reference.items():
        evaluated.setdefault(key, rollout)
    # A missing or incompatible historical trace must not erase the CP5 baseline.
    # Explicit execution permission also covers these reference reruns.
    missing_cp5 = [start for start in START_OFFSETS if (5, start) not in evaluated]
    if missing_cp5:
        cp5 = next(c for c in select_candidates(run_dir, 5)["candidates"] if c["iteration"] == 5)
        for start in missing_cp5:
            collect(cp5, start, "reference_CP5_recomputed")
    plan["pending_rollouts"] = pending
    write_json(output / "evaluation_plan.json", plan)
    if pending:
        return {"ok": False, "status": "pending_rollouts", "output_dir": str(output), "pending_rollouts": pending,
                "instruction": "Historical CP5 was incompatible or absent; --execute-rollouts recomputes the missing references."}
    if args.milestone == 100:
        previous, previous_exclusions = previous_milestone_reference(run_dir, output, evaluated)
        evaluated.update(previous)
        exclusions.extend(previous_exclusions)
    report = {"schema_version": 1, "status": "complete", "selection": selection, "plan": plan,
              "cases": [], "common_prefix_comparisons": [], "quality_gates": {}, "historical_reference_exclusions": exclusions}
    flat_rows = []
    for (iteration, start), rollout in sorted(evaluated.items()):
        candidate = next((c for c in selection["candidates"] if c["iteration"] == iteration), None)
        roles = list(candidate["roles"]) if candidate else ([] if rollout.get("_reference_roles") else ["historical_CP5"])
        roles.extend(rollout.get("_reference_roles", []))
        case = describe_case(rollout, iteration, start, roles)
        if rollout.get("_reference_report"):
            case["reference_report"] = rollout["_reference_report"]
        report["cases"].append(case)
        for window in ("full_episode", "startup", "online_period_regime"):
            if case[window] is not None:
                flat_rows.append({"iteration": iteration, "start": start, "window": window, **analysis.flatten(case[window])})
    for start in START_OFFSETS:
        indices = sorted(iteration for iteration, label in evaluated if label == start)
        for first, second in itertools.combinations(indices, 2):
            a, b = evaluated[(first, start)], evaluated[(second, start)]
            validate_pair(a, b)
            rows_a, rows_b = analysis.common_rows(a, b)
            pair = {"start": start, "iterations": [first, second], "metrics": {
                str(first): window_metrics(a, rows_a), str(second): window_metrics(b, rows_b)}}
            report["common_prefix_comparisons"].append(pair)
            for iteration, metrics in pair["metrics"].items():
                flat_rows.append({"iteration": iteration, "start": start, "window": f"common_CP{first}_CP{second}", **analysis.flatten(metrics)})
    for iteration in sorted({case["checkpoint_iteration"] for case in report["cases"]}):
        report["quality_gates"][str(iteration)] = quality_gate([case for case in report["cases"] if case["checkpoint_iteration"] == iteration])
    report["figures"] = plot_tracking(output, evaluated, args.milestone)
    write_json(output / "milestone_evaluation.json", report)
    analysis.write_csv(output / "metrics.csv", flat_rows)
    (output / "milestone_evaluation.md").write_text(markdown_report(report), encoding="utf-8")
    return {"ok": True, "status": "complete", "output_dir": str(output), "cases": len(report["cases"]), "quality_gates": report["quality_gates"]}


def create_requested_report(run_dir: Path, evaluation_dir: Path, milestone: int) -> dict | None:
    """Honor an explicit request after evaluation and before this child exits.

    The long-running controller launches this evaluator in a fresh process, so
    adding a report request requires no interruption of the active training.
    """
    run_dir = run_dir.resolve()
    request_path = run_dir / REPORT_REQUEST_FILENAME
    if not request_path.exists():
        return None
    request = read_json(request_path)
    if request.get("schema_version") != 1:
        raise ValueError("Unsupported post-evaluation report request")
    if milestone not in request.get("milestones", []):
        return None
    run_dir.relative_to((UNIFIED_DIR / "runs" / "training").resolve())
    if Path(request["run_dir"]).resolve() != run_dir:
        raise ValueError("Report request belongs to a different training run")
    plan_dir = Path(request["plan_dir"]).resolve()
    plan_dir.relative_to((UNIFIED_DIR / "runs" / "plans").resolve())
    try:
        from .create_milestone_report import create_report
    except ImportError:
        from create_milestone_report import create_report
    receipt = create_report(evaluation_dir.resolve(), plan_dir)
    request["completed_reports"] = {**request.get("completed_reports", {}), str(milestone): receipt}
    temporary = request_path.with_suffix(".json.tmp")
    write_json(temporary, request)
    temporary.replace(request_path)
    return receipt


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--milestone", type=int, choices=(50, 100), required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--python", default=sys.executable, help="Interpreter with the existing OpenSim/RLlib environment")
    parser.add_argument("--execute-rollouts", action="store_true", help="Explicitly launch missing rollouts, sequentially")
    args = parser.parse_args()
    result = evaluate(args)
    if result.get("ok") is True and result.get("status") == "complete":
        receipt = create_requested_report(args.run_dir, Path(result["output_dir"]), args.milestone)
        if receipt is not None:
            result["create_report"] = receipt
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
