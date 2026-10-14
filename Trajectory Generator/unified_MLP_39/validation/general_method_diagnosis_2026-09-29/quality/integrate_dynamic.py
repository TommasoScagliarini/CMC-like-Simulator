"""Analyze finalized physical sequence cases, including clean early failures.

No simulation or polling. A missing diagnostic receipt yields status=pending
and no comparison. Full-episode rankings require 500 complete segments ending
for episode_time_limit. Failed candidates get matched-prefix analyses only.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
import analyze as q
import numpy as np

HERE, REPO = q.HERE, q.REPO


def load_case(name, spec):
    folder = REPO / spec["path"]
    receipt = folder / "diagnostic_receipt.json"
    if not receipt.exists():
        return {"status": "pending_or_missing_receipt", "source": spec["path"]}, None
    check = q.read_json(receipt)
    summary = q.read_json(folder / "rollout_summary.json")
    trace = q.read_json(folder / "rollout_policy_trace.json")
    assert len(trace) == summary["steps"] == check["steps"], name
    assert check["checkpoint_unchanged"], name
    original_cfg = q.RewardConfig.from_mapping(summary["reward_config"])
    reproduced = sum(q.compute_reward(r["reward_terms"], original_cfg, reference=r["policy_segment_values"])[0] for r in trace)
    error = abs(reproduced - summary["episode_return"])
    assert error < 1e-7, (name, error)
    complete = [i for i, r in enumerate(trace) if r.get("segment_completed", True)]
    # In hard failures the final partial row may repeat the previous timestamp.
    # Keep it for original return accounting, never treat it as a new q sample.
    assert complete == list(range(len(complete))), "Only a terminal partial segment is supported"
    full = len(trace) == len(complete) == 500 and summary["end_reason"] == "episode_time_limit"
    status = {"status": "full_episode" if full else "early_or_nonstandard_end",
              "source": spec["path"], "target": spec["target"], "role": spec["role"],
              "steps": len(trace), "complete_segments": len(complete), "end_reason": summary["end_reason"],
              "original_return_reconstruction_error": error, "eligible_full_return_comparison": full,
              "terminal_partial_rows": len(trace) - len(complete), "receipt": check}
    tables = {key: q.read_sto(folder / "sim_outputs" / f"rollout_episode_{key}.sto") for key in ("sea_torques", "online_grf")}
    return status, {"trace": trace, "summary": summary, "tables": tables}


def curve_metrics(time, value, phase_time, phase, window):
    mask = (time >= window[0] - 1e-9) & (time <= window[1] + 1e-9)
    x = value[mask]
    scale = float(np.percentile(x, 95) - np.percentile(x, 5))
    duration = float(time[mask][-1] - time[mask][0])
    low, high = q.filter_split(time, value, 10)
    _, high20 = q.filter_split(time, value, 20)
    result = {"rms_nm": q.rms(x), "min_nm": float(x.min()), "max_nm": float(x.max()),
              "raw_tv_normalized_per_s": q.ratio(np.abs(np.diff(x)).sum(), scale * duration),
              "lp10_tv_normalized_per_s": q.ratio(np.abs(np.diff(low[mask])).sum(), scale * duration),
              "hf20_residual_rms_nm": q.rms(high20[mask]),
              "hf20_residual_normalized": q.ratio(q.rms(high20[mask]), np.std(x))}
    try:
        profile = q.profile(time, low, phase_time, phase, window=window)
    except AssertionError as exc:
        if str(exc) != "No complete prescribed cycle in analysis window":
            raise
        result["complete_cycles"] = 0
        result["phase_profile"] = None
    else:
        result["complete_cycles"] = len(profile["intervals_s"])
        result["phase_profile"] = {"intervals_s": profile["intervals_s"], "mean_nm": profile["mean"].tolist(),
                                   "tv_over_range": q.ratio(np.abs(np.diff(profile["mean"])).sum(), np.ptp(profile["mean"]))}
    return result, low


def compare_pair(group, baseline, candidate, statuses, cfg):
    names = [f"{group}_baseline", f"{group}_sequence"]
    count = min(statuses[n]["complete_segments"] for n in names)
    if count == 0:
        return {"target": group, "comparison": "no_completed_common_prefix",
                "prefix_complete_segments": 0, "full_return_comparison_allowed": False,
                "shape_available": False, "data": {},
                "interpretation": "No physically completed segment; no prefix quality or return comparison is possible."}
    pair = {names[0]: baseline, names[1]: candidate}
    traces = {n: d["trace"][:count] for n, d in pair.items()}
    time = np.array([r["time"] for r in traces[names[0]]])
    assert np.all(np.diff(time) > 0)
    np.testing.assert_allclose(time, [r["time"] for r in traces[names[1]]], atol=1e-8, rtol=0)
    phases = {}
    for joint in q.JOINTS:
        coord = f"pros_{joint}_angle"
        phases[joint] = np.array([r["reward_terms"][f"{coord}_imitation_target_phase"] % 1 for r in traces[names[0]]])
        np.testing.assert_allclose(phases[joint], [r["reward_terms"][f"{coord}_imitation_target_phase"] % 1 for r in traces[names[1]]], atol=1e-8, rtol=0)
        for key in ("imitation_target_q", "imitation_target_qdot"):
            np.testing.assert_allclose([r[key][coord] for r in traces[names[0]]], [r[key][coord] for r in traces[names[1]]], atol=1e-8, rtol=0)
    full = all(statuses[n]["eligible_full_return_comparison"] for n in names)
    common_end = min(time[-1], *(d["tables"]["sea_torques"]["time"][-1] for d in pair.values()))
    # Same input time support for both offline zero-phase filters. For a partial
    # episode exclude a 150ms trailing margin; do not borrow baseline's future.
    window = (q.WINDOW[0], min(q.WINDOW[1], common_end - (0 if full else .15)))
    result = {"target": group, "comparison": "full_500_same_target" if full else "matched_completed_prefix_only",
              "prefix_complete_segments": count, "prefix_times_s": [float(time[0]), float(time[-1])],
              "full_return_comparison_allowed": full, "shape_window_s": window,
              "terminal_partial_rows_excluded_from_prefix": statuses[names[1]]["terminal_partial_rows"],
              "data": {}, "same_target_q_qdot_phase_time_verified": True}
    usable = window[1] - window[0] >= .2
    result["shape_available"] = usable
    for name, data in pair.items():
        rows = traces[name]
        budget = q.aggregate_reward(q.reward_rows(rows, cfg))
        metrics = {"common_contract_prefix_reward": budget, "kinematics": {}, "torque": {}}
        qmask = (time >= window[0] - 1e-9) & (time <= window[1] + 1e-9)
        if usable:
            for joint in q.JOINTS:
                coord = f"pros_{joint}_angle"
                target = np.degrees([r["imitation_target_q"][coord] for r in rows])
                actual = np.degrees([r["prosthetic_state"][coord] for r in rows])
                served = np.degrees([r["prosthetic_state"][coord + "_served_ref"] for r in rows])
                qm = {"actual_target_deg": q.matching(actual[qmask], target[qmask]),
                      "served_target_deg": q.matching(served[qmask], target[qmask]),
                      "actual_served_deg": q.matching(actual[qmask], served[qmask])}
                try:
                    a = q.profile(time, actual, time, phases[joint], window=window)
                    ref = q.profile(time, target, time, phases[joint], window=window)
                except AssertionError as exc:
                    if str(exc) != "No complete prescribed cycle in analysis window":
                        raise
                    qm["phase_mean_actual_target_deg"] = None
                else:
                    qm["phase_mean_actual_target_deg"] = q.matching(a["mean"], ref["mean"])
                metrics["kinematics"][joint] = qm
                tab = data["tables"]["sea_torques"]
                support = tab["time"] <= common_end + 1e-9
                tm, _ = curve_metrics(tab["time"][support], tab[f"SEA_{joint.title()}_tau_spring"][support], time, phases[joint], window)
                metrics["torque"][joint] = tm
            grf = data["tables"]["online_grf"]
            gm = (grf["time"] >= window[0] - 1e-9) & (grf["time"] <= window[1] + 1e-9)
            metrics["max_recorded_penetration_mm_in_shape_window"] = float(1000 * grf["left_penetration"][gm].max())
        result["data"][name] = metrics
    result["observed_prefix_reward_preference"] = max(names, key=lambda n: result["data"][n]["common_contract_prefix_reward"]["sum"]["reward"])
    result["interpretation"] = ("Observed full-episode ranking, not an optimum or learned-policy result." if full else
                                "Prefix ranking excludes the terminal cost and missing future; it is not episode success, robustness, or a full-return ranking.")
    return result


def main():
    specs = q.read_json(HERE / "dynamic_sources.json")
    statuses, cases = {}, {}
    for name, spec in specs.items():
        statuses[name], cases[name] = load_case(name, spec)
    q.save(HERE / "dynamic_integration_status.json", statuses)
    if any(d is None for d in cases.values()):
        print("No integration yet: at least one finalized diagnostic receipt is missing.")
        return
    # Derive the contract from an immutable physical baseline input, rather
    # than metrics.json, which the subsequent full-candidate report rewrites.
    cfg = q.RewardConfig.from_mapping(cases["LEFT_baseline"]["summary"]["reward_config"])
    assert cfg.reward_mode == "imitation" and cfg.grf_penetration_weight == 2
    pairs = {g: compare_pair(g, cases[f"{g}_baseline"], cases[f"{g}_sequence"], statuses, cfg) for g in ("LEFT", "RIGHT")}
    full_sources = {n: s for n, s in specs.items() if s["role"] == "sequence" and statuses[n]["eligible_full_return_comparison"]}
    q.save(HERE / "dynamic_full_sources.json", full_sources)
    q.save(HERE / "dynamic_pair_metrics.json", {"cases": statuses, "common_reward_config": asdict(cfg), "pairs": pairs,
          "limitations": ["Open-loop optimized sequences have full-trajectory information; they are not causal learned policies.",
                          "One subject and one fixed start; identical method LEFT/RIGHT is not validation across subjects.",
                          "Clean early physical failure remains failure even if its common prefix tracks well.",
                          "Terminal partial states are omitted from shape/prefix metrics, retained in original return verification."]})
    q.read(Path(__file__))
    q.read(HERE / "analyze.py")
    q.read(q.PACKAGE / "reward_function.py")
    assert all(hashlib.sha256((REPO / name).read_bytes()).hexdigest() == digest for name, digest in q.HASHES.items())
    q.save(HERE / "dynamic_integration_input_hashes.json", q.HASHES)
    q.save(HERE / "dynamic_integration_verification.json", {"passed": True, "receipts_present": True, "sources_unchanged": True,
           "original_returns_verified": True, "new_simulations": 0, "new_training": 0,
           "full_sequences": list(full_sources), "partial_sequences": [n for n, s in specs.items() if s["role"] == "sequence" and n not in full_sources]})
    # Reuse the established report only for full candidates. Partial outputs
    # remain exclusively in matched-prefix files, never in 500-step rankings.
    if full_sources:
        with (HERE / "execution_with_dynamic.log").open("w", encoding="utf-8") as output:
            subprocess.run([sys.executable, str(HERE / "analyze.py"), "--extra-sources", str(HERE / "dynamic_full_sources.json")], stdout=output, stderr=subprocess.STDOUT, check=True)
    print(json.dumps({"status": statuses, "pairs": pairs}, indent=2))


if __name__ == "__main__":
    main()
