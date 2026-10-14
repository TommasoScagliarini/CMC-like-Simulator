"""Consolidate frozen reset experiments without changing their inputs."""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import frozen_reset_tests as test

AUDIT = Path(__file__).resolve().parent


def snapshot_delta(a, b):
    return max(abs(float(a[k]) - float(b[k])) for k in a)


def main():
    plan = test.read_json(AUDIT / "frozen_manifest.json")
    test.verify_inputs(plan)
    cases = []
    for case in plan["cases"]:
        folder = AUDIT / "rollouts" / case["id"]
        receipt = test.read_json(folder / "receipt.json")
        if receipt["status"] != "completed" or not receipt["frozen_inputs_unchanged"]:
            raise RuntimeError(f"Incomplete or mutated experiment: {case['id']}")
        metrics = test.read_json(folder / "metrics.json")
        summary = metrics["summary"]
        charts = test.read_json(folder / "plot_receipt.json")
        missing = (charts["missing_channels_report"] or "").strip()
        if len(charts["figures"]) != 7 or missing not in ("", "No missing channels."):
            raise RuntimeError("Missing canonical plots/channels")
        for path, expected in charts["figures"].items():
            if test.sha256(test.resolve(path)) != expected:
                raise RuntimeError("Changed plot")
        record = {
            "id": case["id"], "kind": case["kind"], "offset_s": case["offset_s"],
            "actual_start_s": receipt["actual_start_s"],
            "physical_duration_s": receipt["actual_end_s"] - receipt["actual_start_s"],
            "steps": summary["steps"], "end_reason": summary["end_reason"],
            "completed_requested_duration": receipt["completed_requested_duration"],
            "episode_return": summary["episode_return"],
            "valid_hs": summary["phase_valid_hs_count"],
            "valid_to": summary["phase_valid_to_count"],
            "valid_cycles": summary["phase_valid_cycle_count"],
            "invalid_events": summary["invalid_event_count"],
            "policy_clipped_steps": summary["action_clipped_steps"],
            "max_penetration_mm": 1000 * metrics["left_grf"]["max_penetration_m"],
            "tracking": {coord: metrics["tracking"][coord] for coord in test.JOINTS},
            "left_grf": metrics["left_grf"],
            "sea": metrics["sea"],
            "reserve_max_abs_by_coordinate": metrics["reserve_max_abs_by_coordinate"],
            "morphology_settled_samples": summary["morphology_settled_samples"],
            "morphology_discarded_samples": summary["morphology_discarded_samples"],
            "sto_count": len(metrics["outputs"]),
            "sto_rows_per_file": sorted(set(v["rows"] for v in metrics["outputs"].values())),
            "plot_count": len(charts["figures"]),
            "reset_q": receipt["reset_reads"][0]["actual_q"],
            "reset_qdot": receipt["reset_reads"][0]["actual_qdot"],
            "elapsed_wall_s": receipt["elapsed_wall_s"],
        }
        cases.append(record)

    current = AUDIT / "rollouts/nominal/sim_outputs"
    historical = test.HISTORICAL / "sim_outputs"
    comparisons = []
    for path in sorted(current.glob("*.sto")):
        old = historical / path.name
        a, _ = test.storage(path)
        b, _ = test.storage(old)
        if list(a) != list(b) or any(len(a[k]) != len(b[k]) for k in a):
            raise RuntimeError(f"Changed baseline STO schema: {path.name}")
        max_error = max(abs(v - w) for key in a for v, w in zip(a[key], b[key]))
        comparisons.append({"file": path.name, "bytes_identical": path.read_bytes() == old.read_bytes(),
                            "numerical_max_abs_delta": max_error})

    probe = test.read_json(AUDIT / "rollouts/passive_reset_probe/reset_probe.json")
    records = {r["case"]["id"]: r for r in probe["records"]}
    nominal, zero = records["nominal"], records["zero_prosthetic_velocity"]
    nstate, zstate = nominal["state"], zero["state"]
    changed = {name: [nstate["full_state_after_reset"][name], value]
               for name, value in zstate["full_state_after_reset"].items()
               if nstate["full_state_after_reset"][name] != value}
    allowed = ("pros_knee_angle/speed", "pros_ankle_angle/speed", "SEA_Knee/motor_speed", "SEA_Ankle/motor_speed")
    if len(changed) != 4 or any(not any(name.endswith(tail) for tail in allowed) for name in changed):
        raise RuntimeError(f"Unexpected zero-speed intervention: {changed}")

    delta = {}
    for suffix in ("kinematics", "kinematics_reference", "sea_torques", "reserve_torques", "online_grf"):
        name = f"rollout_episode_{suffix}.sto"
        a, _ = test.storage(AUDIT / "rollouts/nominal/sim_outputs" / name)
        b, _ = test.storage(AUDIT / "rollouts/zero_prosthetic_velocity/sim_outputs" / name)
        if a["time"] != b["time"]:
            raise RuntimeError("Zero-speed comparison changed timeline")
        times = a["time"]
        first = times[0]
        windows = {"all": list(range(len(times))),
                   "first_050s": [i for i,t in enumerate(times) if t < first + .5],
                   "after_050s": [i for i,t in enumerate(times) if t >= first + .5]}
        delta[suffix] = {key: {win: {"rms_raw_units": test.rms([a[key][i] - b[key][i] for i in ids]),
                                             "max_abs_raw_units": max(abs(a[key][i] - b[key][i]) for i in ids)}
                             for win, ids in windows.items()} for key in a if key != "time"}

    result = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "policy_updates": 0, "cases": cases,
              "nominal_historical_replay": comparisons,
              "nominal_historical_replay_all_numeric_exact": all(v["numerical_max_abs_delta"] == 0 for v in comparisons),
              "passive_reset": {"state_seed_invariance": probe["fixed_start_state_identical_across_seeds"],
                                "actor_seed_invariance": probe["fixed_start_actor_observation_identical_across_seeds"],
                                "ik_position_max_error": max(r["state"]["position_max_error_rad_or_m"] for r in probe["records"]),
                                "applied_velocity_max_error": max(r["state"]["velocity_max_error_rad_s_or_m_s"] for r in probe["records"]),
                                "zero_velocity_changed_states_only": changed,
                                "zero_vs_nominal_position_delta": snapshot_delta(nstate["actual_q"], zstate["actual_q"]),
                                "zero_vs_nominal_served_ref_q_delta": snapshot_delta(nstate["served_reference_after_reset"]["q"], zstate["served_reference_after_reset"]["q"]),
                                "zero_vs_nominal_served_ref_qdot_delta": snapshot_delta(nstate["served_reference_after_reset"]["qdot"], zstate["served_reference_after_reset"]["qdot"])},
              "zero_vs_nominal_same_time_grid": delta,
              "input_integrity_verified": True,
              "limitations": ["One frozen checkpoint, one AB06 condition, deterministic inference; no PPO updates.",
                              "Changing start changes biological IK/GRF window and FSM startup, not only pose.",
                              "Zero prosthetic speeds at walking pose are not static equilibrium or autonomous gait onset.",
                              "Return includes reward coverage/startup; completion and valid cycles do not establish physical qualification.",
                              "Native tests attest this macOS arm64 host; Windows native verification remains open."]}
    test.write_json(AUDIT / "consolidated_results.json", result)
    seal = {"sealed_utc": datetime.now(timezone.utc).isoformat(),
            "comparison_with_claude_not_yet_requested": True,
            "files": {test.relative(p): test.sha256(p) for p in [AUDIT/"codex_source_audit.json", AUDIT/"consolidated_results.json", AUDIT/"PROTOCOL.md"]}}
    test.write_json(AUDIT / "codex_results_seal.json", seal)
    print(json.dumps({"cases": [{k:c[k] for k in ["id","steps","episode_return","valid_cycles","max_penetration_mm"]} for c in cases],
                      "nominal_historical_replay_all_numeric_exact": result["nominal_historical_replay_all_numeric_exact"]}, indent=2))


if __name__ == "__main__":
    main()
