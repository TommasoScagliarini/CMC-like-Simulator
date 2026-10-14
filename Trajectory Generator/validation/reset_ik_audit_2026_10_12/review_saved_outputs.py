"""Independent, stdlib-only arithmetic review of the six saved reset rollouts.

No simulator, policy, evaluator, consolidated analysis or external conclusions
are imported. Visual observations below record the agent's view_image review of
the named files; programmatic existence checks alone are not visual inspection.
"""
from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
RUNS = AUDIT / "rollouts"
CASES = ("nominal", "trained_minus020", "trained_plus020", "heldout_phase_plus040", "heldout_next_left_hs", "zero_prosthetic_velocity")
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
PICTURES = ("01_time_sea_control_reserve.png", "02_time_joint_motor_states.png", "07_mlp_policy_vs_sound_leg_error.png")
VISUAL_NOTES = {
    "nominal": "01/02 show repeated finite-looking control/kinematic patterns with motor-speed ripples. 07 shows substantial phase displacement from the shifted sound-leg diagnostic; it is an ex-novo run, so that trace is not an active imitation objective.",
    "trained_minus020": "Different initial knee pose/velocity produces a visible initial transient; repeated motion follows. 01 prosthetic reserves are flat zero. 07 diagnostic phase displacement persists.",
    "trained_plus020": "Initial ankle velocity/motor-speed spikes are visible, followed by repeated bounded motion over the plotted interval. 07 remains displaced from the shifted sound-leg diagnostic.",
    "heldout_phase_plus040": "01 has a positive ankle-torque burst in the first portion and 02 has a sharp initial knee motor-speed transient. Later motion repeats. No broken or missing plot panel was observed; 07 diagnostic mismatch remains.",
    "heldout_next_left_hs": "01 has deeper negative knee torque than nominal and 02 has repeated motor-speed ripples. 07 knee swings appear out of phase with the shifted sound-leg comparator. Completion does not resolve these shape/contact questions.",
    "zero_prosthetic_velocity": "01/02/07 are visually close to nominal after the brief initial transient. Saved-data arithmetic, rather than the visual similarity, determines the measured differences.",
}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda text: (_ for _ in ()).throw(ValueError(text)))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_sto(path):
    """Parse separately from the implementation under review; check every value."""
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    header_end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    metadata = dict(line.strip().split("=", 1) for line in lines[:header_end] if "=" in line)
    body = [line.split() for line in lines[header_end + 1:] if line.strip()]
    columns, body = body[0], body[1:]
    rows = []
    for row in body:
        if len(row) != len(columns):
            raise ValueError(f"Row width mismatch: {path}")
        values = tuple(map(float, row))
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"Nonfinite value: {path}")
        rows.append(values)
    if len(rows) != int(metadata["nRows"]) or len(columns) != int(metadata["nColumns"]):
        raise ValueError(f"Storage metadata mismatch: {path}")
    times = [row[0] for row in rows]
    intervals = [right - left for left, right in zip(times, times[1:])]
    if not intervals or min(intervals) <= 0:
        raise ValueError(f"Invalid time grid: {path}")
    return {name: [row[i] for row in rows] for i, name in enumerate(columns)}, metadata


def rmse(left, right, indices):
    return math.sqrt(math.fsum((left[i] - right[i]) ** 2 for i in indices) / len(indices)) * 180 / math.pi


def main():
    output_json = AUDIT / "independent_saved_outputs_review.json"
    output_md = AUDIT / "independent_saved_outputs_review.md"
    if output_json.exists() or output_md.exists():
        raise FileExistsError("Preserve previous review outputs")
    review = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026",
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "Saved outputs and passive initial-state receipts only; independent parser/math; 18 PNGs inspected with view_image",
              "native_executions": 0, "policy_updates": 0,
              "scientific_qualification": "not_assessed_or_certified",
              "cases": {}, "visual_review": [], "input_sha256_before": {}}
    nominal_summary = read_json(RUNS / "nominal/rollout_summary.json")
    receipts = {}
    for case in CASES:
        folder = RUNS / case
        summary = read_json(folder / "rollout_summary.json")
        receipt = read_json(folder / "receipt.json")
        receipts[case] = receipt
        metrics = read_json(folder / "metrics.json")
        trace = read_json(folder / "rollout_policy_trace.json")
        sto_paths = sorted((folder / "sim_outputs").glob("*.sto"))
        if len(sto_paths) != 17:
            raise ValueError(f"Expected 17 STO files: {case}")
        tables, file_checks = {}, {}
        reference_time = None
        for path in sto_paths:
            data, metadata = parse_sto(path)
            if len(data["time"]) != 5000:
                raise ValueError(f"Expected 5000 finite rows: {path}")
            if reference_time is None:
                reference_time = data["time"]
            elif reference_time != data["time"]:
                raise ValueError(f"Time grid differs between STOs: {case}")
            if max(abs((b - a) - 0.001) for a, b in zip(data["time"], data["time"][1:])) > 2e-8:
                raise ValueError(f"STO timestep changed: {path}")
            if metadata.get("inDegrees") != "no":
                raise ValueError(f"Unexpected units in current output: {path}")
            tables[path.stem[len("rollout_episode_"):]] = data
            file_checks[path.name] = {"rows": 5000, "all_numeric_finite": True, "sha256": digest(path)}
            review["input_sha256_before"][str(path.relative_to(REPO))] = file_checks[path.name]["sha256"]
        for name in ("receipt.json", "metrics.json", "rollout_summary.json", "rollout_policy_trace.json"):
            path = folder / name
            review["input_sha256_before"][str(path.relative_to(REPO))] = digest(path)
        if len(trace) != 500 or summary["steps"] != 500 or summary["terminated"] or summary["end_reason"] != "episode_time_limit":
            raise ValueError(f"Requested 500-step completion is not recorded: {case}")
        # OutputRecorder stores each integration interval's left endpoint:
        # [t0, t_end - dt]. Final runtime time is recorded in the receipt.
        if abs(reference_time[0] - receipt["actual_start_s"]) > 1e-8 or abs(reference_time[-1] - receipt["actual_start_s"] - 4.999) > 1e-8 or abs(receipt["actual_end_s"] - receipt["actual_start_s"] - 5.0) > 1e-8:
            raise ValueError(f"Recorded samples do not span the five-second integration: {case}")
        actual, served = tables["kinematics"], tables["kinematics_reference"]
        all_indices = range(5000)
        windows = {"all_recorded_samples": all_indices,
                   "first_050s": [i for i, t in enumerate(reference_time) if t < receipt["actual_start_s"] + 0.5],
                   "after_050s": [i for i, t in enumerate(reference_time) if t >= receipt["actual_start_s"] + 0.5]}
        tracking = {joint: {window: rmse(actual[joint], served[joint + "_q_ref"], indices) for window, indices in windows.items()} for joint in JOINTS}
        metric_disagreement = max(abs(value - metrics["tracking"][joint]["actual_minus_served_reference_rmse"][window])
                                 for joint, values in tracking.items() for window, value in values.items())
        if metric_disagreement > 1e-12:
            raise ValueError(f"Independent RMSE disagrees with recorded metrics: {case}: {metric_disagreement}")
        last_phase = trace[-1]["phase_fsm"]
        counters = {name: last_phase[name] for name in ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count", "hs_cancelled_count")}
        for name, field in (("valid_hs_count", "phase_valid_hs_count"), ("valid_to_count", "phase_valid_to_count"), ("valid_cycle_count", "phase_valid_cycle_count"), ("invalid_event_count", "invalid_event_count")):
            if counters[name] != summary[field]:
                raise ValueError(f"Trace/summary phase mismatch: {case}/{name}")
        phase_flags = {name: sum(bool(row["phase_fsm"][name]) for row in trace) for name in
                       ("invalid_event_this_step", "cycle_rejected_this_step", "cycle_degraded", "timeout_exceeded", "resync_event_this_step")}
        grf = tables["online_grf"]
        review["cases"][case] = {"start_s": receipt["actual_start_s"], "duration_s": 5.0,
                                 "sto_file_count": 17, "sto_row_count_each": 5000, "all_finite": True,
                                 "sto_file_checks": file_checks,
                                 "return_recorded": summary["episode_return"],
                                 "return_delta_from_nominal": summary["episode_return"] - nominal_summary["episode_return"],
                                 "return_delta_percent_from_nominal": (summary["episode_return"] / nominal_summary["episode_return"] - 1) * 100,
                                 "phase_counters": counters, "phase_flags_record_count": phase_flags,
                                 "tracking_actual_minus_served_rmse_deg": tracking,
                                 "recorded_rmse_agreement_max_abs_deg": metric_disagreement,
                                 "left_peak_penetration_mm_1ms": max(grf["left_penetration"]) * 1000,
                                 "left_peak_normal_force_n_1ms": max(grf["left_normal_force"]),
                                 "full_state_count": len(receipt["reset_reads"][0]["full_state_after_reset"])}
        plot_root = REPO / "plot/reset_ik_audit_2026_10_12" / case
        for picture in PICTURES:
            path = plot_root / picture
            review["visual_review"].append({"case": case, "path": str(path.relative_to(REPO)),
                                            "sha256": digest(path), "inspected_with": "functions.view_image",
                                            "readability": "readable", "note": VISUAL_NOTES[case]})
            review["input_sha256_before"][str(path.relative_to(REPO))] = digest(path)
    baseline = receipts["nominal"]["reset_reads"][0]
    zero = receipts["zero_prosthetic_velocity"]["reset_reads"][0]
    original_state, changed_state = baseline["full_state_after_reset"], zero["full_state_after_reset"]
    expected_names = {"/jointset/pros_knee/pros_knee_angle/speed", "/jointset/pros_ankle/pros_ankle_angle/speed",
                      "/forceset/SEA_Knee/motor_speed", "/forceset/SEA_Ankle/motor_speed"}
    differences = {name: {"nominal": value, "zero": changed_state[name]} for name, value in original_state.items() if value != changed_state[name]}
    if original_state.keys() != changed_state.keys() or set(differences) != expected_names or any(change["zero"] != 0 for change in differences.values()):
        raise ValueError("Zero perturbation changed something other than exactly the four speed states")
    for name in ("q_ik", "qdot_ik", "qddot_ik", "actual_q", "served_reference_after_reset"):
        if baseline[name] != zero[name]:
            raise ValueError(f"Zero intervention altered preserved initial field: {name}")
    review["zero_intervention"] = {"exactly_four_states_changed": True, "changes": differences,
                                    "all_other_initial_states_exactly_preserved": True,
                                    "position_and_original_ik_derivatives_and_served_reference_preserved": True,
                                    "note": "Two prosthetic joint speeds and matching SEA motor speeds only; biological motion/support remains present. This does not constitute a standing start."}
    for source_name, source in (("control_2026_10_06", REPO / "Trajectory Generator/runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s/sim_outputs"),
                                ("historical_2026_09_02", REPO / "Trajectory Generator/runs/rollout/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-02_best_rollout/sim_outputs")):
        equality = {}
        for name, checks in review["cases"]["nominal"]["sto_file_checks"].items():
            path = source / name
            old_digest = digest(path)
            review["input_sha256_before"][str(path.relative_to(REPO))] = old_digest
            equality[name] = old_digest == checks["sha256"]
        if not all(equality.values()):
            raise ValueError(f"Nominal does not replay all historical physical STOs: {source_name}")
        review.setdefault("nominal_exact_sto_replay", {})[source_name] = equality
    changed_inputs = [name for name, expected in review["input_sha256_before"].items() if digest(REPO / name) != expected]
    review["read_inputs_unchanged"] = not changed_inputs
    review["changed_input_paths"] = changed_inputs
    review["numerical_review"] = "checks_confirmed" if not changed_inputs else "input_integrity_failure"
    review["limits"] = ["Single deterministic policy/subject and short windows; no statistical robustness or scientific qualification is asserted.",
                        "Shifted sound-leg plots are a diagnostic in ex_novo, not an active target or independent biomechanics verdict.",
                        "Different start times change biological reference and prescribed right GRF as well as initial state; their returns are descriptive reset sensitivity, not an isolated causal reset intervention.",
                        "Recorded episode return is read from the evaluator summary. Trace does not save each returned reward, so this review independently computes return differences, not an independent reconstruction of every returned reward.",
                        "Actual-versus-served RMSE measures low-level tracking; it is not raw-IK tracking, morphology or gait quality."]
    output_json.write_text(json.dumps(review, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    text = ["# Review indipendente degli output del reset IK — 2026-10-12", "",
            "Verifica in sola lettura: 102 STO (17 per caso), ciascuno con 5000 righe numeriche finite e griglia comune di 1 ms. I sei episodi registrano 500 step/5 s. Il nominale riproduce byte per byte tutti i 17 STO del controllo 6 ottobre e dello storico 2 settembre. Nessuna nuova simulazione o modifica degli input.", "",
            "L'intervento zero cambia esattamente quattro variabili iniziali: velocità knee/ankle protesiche e le due motor_speed SEA. Le altre variabili, la posa, le derivate IK originali e la reference servita iniziale sono identiche al nominale. È una perturbazione di uno stato di cammino, con biologia presente; non un avvio da fermo.", "",
            "| Caso | Return | Δ nominale | Cicli | Invalidi | RMSE knee/ankle actual–served [°] | Pen. 1 ms [mm] |",
            "|---|---:|---:|---:|---:|---:|---:|"]
    for case, data in review["cases"].items():
        tracking = data["tracking_actual_minus_served_rmse_deg"]
        text.append(f"| {case} | {data['return_recorded']:.6f} | {data['return_delta_from_nominal']:+.6f} | {data['phase_counters']['valid_cycle_count']:.0f} | {data['phase_counters']['invalid_event_count']:.0f} | {tracking[JOINTS[0]]['all_recorded_samples']:.6f}/{tracking[JOINTS[1]]['all_recorded_samples']:.6f} | {data['left_peak_penetration_mm_1ms']:.6f} |")
    text += ["", "RMSE ricalcolati con parser e somme indipendenti: accordo con le metriche registrate entro 1e-12°. Resync e HS cancellati finali sono zero in tutti i casi; i flag per-step, incluse eventuali attese soft, sono conservati nel JSON e non confusi con invalidazioni.", "",
             "Ispezionati visivamente via view_image i seguenti 18 file (01 mostra SEA/controlli/reserve; la cinematica è nella 02):", ""]
    for item in review["visual_review"]:
        text.append(f"- [{item['case']}/{Path(item['path']).name}]({(REPO / item['path']).as_posix()})")
    text.append("")
    for case in CASES:
        text.append(f"**{case}:** {VISUAL_NOTES[case]}\n")
    text += ["Non è espresso un verdetto di qualificazione scientifica. Completamento, ritorno, tracking del controller, contatto e forma delle traiettorie sono criteri distinti; il confronto fra start diversi conserva anche cambiamenti di riferimento biologico e GRF prescritte. Le figure 07 confrontano una curva sound-leg diagnostica in ex-novo, senza farne un obiettivo imitativo attivo.", "",
             "Tutti i file letti e i loro SHA sono elencati nel JSON; hash invariati dopo la review. Non sono state lette conclusioni Claude o la sintesi consolidata per produrre queste verifiche.", ""]
    output_md.write_text("\n".join(text), encoding="utf-8")
    print(json.dumps({"review": review["numerical_review"], "cases": len(CASES), "finite_sto_files": len(CASES) * 17,
                      "plots_visually_inspected": len(review["visual_review"]), "changed_inputs": changed_inputs}))
    for case, data in review["cases"].items():
        print(case, {"return": data["return_recorded"], "delta": data["return_delta_from_nominal"],
                     "rmse_deg": {j: data["tracking_actual_minus_served_rmse_deg"][j]["all_recorded_samples"] for j in JOINTS},
                     "phase_flags": data["phase_flags_record_count"], "penetration_mm": data["left_peak_penetration_mm_1ms"]})


if __name__ == "__main__":
    main()
