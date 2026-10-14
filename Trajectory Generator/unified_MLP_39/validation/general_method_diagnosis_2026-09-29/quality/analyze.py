"""Read-only trace comparison: kinematic shape, torque shape, common reward.

Run with Python from any cwd. No simulator, training, or production config edits.
Paths are repository relative; optional --extra-sources is a JSON mapping
name -> {"path": "repo/relative/path", "target": "LEFT" | "RIGHT"}.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import io
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").is_file())
TG = REPO / "Trajectory Generator"
PACKAGE = TG / "unified_MLP_39"
PLOTS = REPO / "plot/09_29_2026_general_method_diagnosis/quality"
for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[variable] = "1"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
sys.path.insert(0, str(TG))
sys.dont_write_bytecode = True

import numpy as np
from scipy.signal import butter, sosfiltfilt
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from unified_MLP_39.reward_function import RewardConfig, compute_reward

JOINTS = ("knee", "ankle")
WINDOW = (13.15, 17.95)
GRID = np.linspace(0, 1, 201)
HASHES: dict[str, str] = {}
COLORS = {"best80LEFT": "#d95f02", "ORACLE_LEFT": "#7570b3", "RIGHT60": "#1b9e77", "V26": "#242424", "ORACLE_RIGHT": "#1f78b4", "LEFT_sequence": "#00887a", "RIGHT_sequence": "#bc5090"}


def read(path):
    raw = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(raw).hexdigest()
    return raw.decode("utf-8")


def read_json(path):
    return json.loads(read(path))


def read_sto(path):
    text = read(path).split("endheader", 1)[1].strip()
    columns = text.splitlines()[0].split()
    array = np.loadtxt(io.StringIO(text), skiprows=1, ndmin=2)
    assert array.shape[1] == len(columns) and np.isfinite(array).all()
    result = dict(zip(columns, array.T))
    assert np.all(np.diff(result["time"]) > 0)
    return result


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def rms(a):
    return float(np.sqrt(np.mean(np.square(a))))


def ratio(a, b):
    return float(a / b) if abs(b) > 1e-12 else None


def corr(a, b):
    return float(np.corrcoef(a, b)[0, 1]) if min(np.std(a), np.std(b)) > 1e-12 else None


def standardized(a):
    return (a - np.mean(a)) / max(float(np.std(a)), 1e-12)


def matching(a, target):
    """No lag optimization or time warping: morphology at prescribed phase."""
    return {"rmse": rms(a - target), "bias": float(np.mean(a - target)),
            "correlation": corr(a, target),
            "centered_unit_rms_shape_error": rms(standardized(a) - standardized(target)),
            "range": float(np.ptp(a)), "target_range": float(np.ptp(target)),
            "range_ratio": ratio(np.ptp(a), np.ptp(target)),
            "normalized_rmse_target_range": ratio(rms(a - target), np.ptp(target))}


def profile(time, signal, trace_time, phase, window=WINDOW):
    """Average only complete cycles; interpolate each cycle before averaging."""
    unwrapped = np.unwrap(phase * (2 * np.pi)) / (2 * np.pi)
    assert np.all(np.diff(unwrapped) > 0)
    low = max(window[0], time[0], trace_time[0])
    high = min(window[1], time[-1], trace_time[-1])
    bounds = np.interp([low, high], trace_time, unwrapped)
    cycles, intervals = [], []
    for cycle in range(int(np.ceil(bounds[0])), int(np.floor(bounds[1]))):
        query_time = np.interp(cycle + GRID, unwrapped, trace_time)
        assert query_time[0] >= low - 1e-9 and query_time[-1] <= high + 1e-9
        cycles.append(np.interp(query_time, time, signal))
        intervals.append([float(query_time[0]), float(query_time[-1])])
    assert cycles, "No complete prescribed cycle in analysis window"
    array = np.array(cycles)
    return {"mean": np.mean(array, axis=0), "std": np.std(array, axis=0),
            "cycles": array, "intervals_s": intervals}


def filter_split(time, values, cutoff):
    dt = float(np.median(np.diff(time)))
    np.testing.assert_allclose(np.diff(time), dt, atol=1e-7, rtol=0)
    low = sosfiltfilt(butter(4, cutoff, fs=1 / dt, output="sos"), values)
    return low, values - low


def torque_metrics(time, values, trace_time, phase):
    mask = (time >= WINDOW[0] - 1e-9) & (time <= WINDOW[1] + 1e-9)
    x, t = values[mask], time[mask]
    robust_range = float(np.percentile(x, 95) - np.percentile(x, 5))
    variation = float(np.abs(np.diff(x)).sum())
    out = {"rms_nm": rms(x), "std_nm": float(np.std(x)), "range_nm": float(np.ptp(x)),
           "robust_p95_p05_range_nm": robust_range, "min_nm": float(np.min(x)),
           "max_nm": float(np.max(x)), "positive_fraction": float(np.mean(x > 0)),
           "total_variation_nm": variation,
           "total_variation_normalized_per_s": ratio(variation, robust_range * (t[-1] - t[0])),
           "rate_rms_nm_s": rms(np.diff(x) / np.diff(t)), "cutoffs_hz": {}}
    profiles = {}
    for cutoff in (10, 20):
        low, high = filter_split(time, values, cutoff)
        p = profile(time, low, trace_time, phase)
        centered = x - np.mean(x)
        out["cutoffs_hz"][str(cutoff)] = {
            "lowpass_rms_nm": rms(low[mask]), "residual_hf_rms_nm": rms(high[mask]),
            "residual_hf_rms_normalized_dynamic_rms": ratio(rms(high[mask]), rms(centered)),
            "residual_hf_energy_over_dynamic_energy": ratio(rms(high[mask]) ** 2, rms(centered) ** 2),
            "lowpass_total_variation_normalized_per_s": ratio(np.abs(np.diff(low[mask])).sum(), robust_range * (t[-1] - t[0])),
            "phase_mean_total_variation_over_range": ratio(np.abs(np.diff(p["mean"])).sum(), np.ptp(p["mean"])),
            "cycle_shape_variability_normalized": ratio(rms(p["std"]), np.std(p["mean"])),
            "complete_cycle_intervals_s": p["intervals_s"],
        }
        profiles[str(cutoff)] = {**p, "filtered": low}
    return out, profiles


def reward_rows(trace, cfg):
    """Recorded target-specific primitive errors + one common scalar contract.

    Rebuild the weighted imitation aggregates: compute_reward alone does not
    rebuild these aggregates if coordinate weights were changed in a config.
    Contact tolerances must match because their losses are already recorded.
    """
    records = []
    for row in trace:
        terms = dict(row["reward_terms"])
        for key in ("grf_ankle_moment_flip_tau_tol_nm", "grf_ankle_moment_flip_force_threshold_n"):
            np.testing.assert_allclose(terms[key], getattr(cfg, key), atol=1e-12)
        losses = {}
        for kind, loss_key, middle in (("actual", "sound_imitation_loss", ""), ("served", "served_imitation_loss", "served_")):
            total = 0.0
            for joint in JOINTS:
                for derivative in ("position", "velocity"):
                    value = terms[f"pros_{joint}_angle_{middle}imitation_{derivative}_loss"]
                    weighted = getattr(cfg, f"imitation_{joint}_{derivative}_weight") * value
                    losses[f"{kind}_{joint}_{derivative}_weighted_loss"] = weighted
                    total += weighted
            terms[loss_key] = total
        reward, comp = compute_reward(terms, cfg, reference=row["policy_segment_values"])
        score_a = cfg.blend_imitation * comp["imitation_score"]
        score_s = cfg.blend_served_imitation * comp["served_imitation_score"]
        score_track = cfg.blend_imitation_tracking * comp["tracking_score"]
        before = score_a + score_s + score_track - comp["penalty"]
        record = {"time": row["time"], "phase": terms["pros_knee_angle_imitation_target_phase"] % 1,
                  "fsm_state": row["phase_fsm"]["state_name"], "reward": reward,
                  "actual_score": score_a, "served_score": score_s, "tracking_score": score_track,
                  "base_penalty": comp["penalty"], "command_rate_cost": cfg.command_rate_weight * terms["command_rate_loss"],
                  "other_base_cost": comp["penalty"] - cfg.command_rate_weight * terms["command_rate_loss"],
                  "clip_adjustment": comp["reward_base"] - before,
                  "postclip_cost": comp["reward_base"] - reward,
                  "penetration_cost": comp["grf_penetration_term"],
                  "moment_flip_cost": comp["grf_ankle_moment_flip_term"],
                  "lower_clipped": int(before < cfg.clip_low), **losses}
        np.testing.assert_allclose(score_a + score_s + score_track - record["base_penalty"] + record["clip_adjustment"] - record["postclip_cost"], reward, atol=1e-12)
        records.append(record)
    return records


def aggregate_reward(records):
    keys = [k for k in records[0] if k not in ("time", "phase", "fsm_state")]
    return {"steps": len(records), "sum": {k: float(sum(r[k] for r in records)) for k in keys},
            "mean": {k: float(np.mean([r[k] for r in records])) for k in keys}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extra-sources", type=Path)
    args = parser.parse_args()
    manifest_path = PACKAGE / "validation/left_reward_audit_2026-09-24/sources.json"
    old_sources = read_json(manifest_path)
    read(PACKAGE / "validation/best80_torque_audit_2026-09-24/analyze.py")
    sources = {"best80LEFT": {"path": str((PACKAGE / "runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic").relative_to(REPO)), "target": "LEFT"},
               **{n: {"path": old_sources[n], "target": "LEFT" if n == "ORACLE_LEFT" else "RIGHT"} for n in ("ORACLE_LEFT", "RIGHT60", "V26", "ORACLE_RIGHT")}}
    if args.extra_sources:
        extra = read_json(args.extra_sources.resolve())
        assert not (extra.keys() & sources.keys()), "Extra sources cannot replace historical sources"
        sources.update(extra)
    assert all(s["target"] in ("LEFT", "RIGHT") for s in sources.values())
    common_cfg = RewardConfig.from_mapping(read_json(REPO / sources["best80LEFT"]["path"] / "rollout_summary.json")["reward_config"])
    assert common_cfg.reward_mode == "imitation" and common_cfg.blend_imitation_tracking == 0
    datasets, signals, all_records, profile_rows, summary_rows = {}, {}, {}, [], []
    reconstruction_errors, primitive_errors, sto_trace_errors = [], [], []
    for name, spec in sources.items():
        folder = REPO / spec["path"]
        summary = read_json(folder / "rollout_summary.json")
        trace = read_json(folder / "rollout_policy_trace.json")
        time = np.array([r["time"] for r in trace])
        phase = np.array([r["reward_terms"]["pros_knee_angle_imitation_target_phase"] % 1 for r in trace])
        ankle_phase = np.array([r["reward_terms"]["pros_ankle_angle_imitation_target_phase"] % 1 for r in trace])
        joint_phases = {"knee": phase, "ankle": ankle_phase}
        phase_offset = (ankle_phase - phase + .5) % 1 - .5
        assert np.all(np.diff(time) > 0)
        mask = (time >= WINDOW[0] - 1e-9) & (time <= WINDOW[1] + 1e-9)
        original_cfg = RewardConfig.from_mapping(summary["reward_config"])
        original_return = sum(compute_reward(r["reward_terms"], original_cfg, reference=r["policy_segment_values"])[0] for r in trace)
        error = abs(original_return - summary["episode_return"])
        assert error < 1e-7, (name, error)
        reconstruction_errors.append(error)
        records = reward_rows(trace, common_cfg)
        all_records[name] = records
        tables = {key: read_sto(folder / "sim_outputs" / f"rollout_episode_{key}.sto") for key in ("sea_torques", "kinematics", "kinematics_reference", "online_grf")}
        torque_time = tables["sea_torques"]["time"]
        kin, torque, qp, tp = {}, {}, {}, {}
        qsignals = {}
        for joint in JOINTS:
            coord = f"pros_{joint}_angle"
            target = np.array([r["imitation_target_q"][coord] for r in trace])
            target_v = np.array([r["imitation_target_qdot"][coord] for r in trace])
            q = np.array([r["prosthetic_state"][coord] for r in trace])
            served = np.array([r["prosthetic_state"][coord + "_served_ref"] for r in trace])
            v = np.array([r["prosthetic_state"][coord + "_vel"] for r in trace])
            served_v = np.array([r["prosthetic_state"][coord + "_served_ref_vel"] for r in trace])
            for kind, actual, desired, suffix, unit in (("actual_position", q, target, "imitation_position_loss", "position_range_rad"), ("actual_velocity", v, target_v, "imitation_velocity_loss", "velocity_range_rad_s"), ("served_position", served, target, "served_imitation_position_loss", "position_range_rad"), ("served_velocity", served_v, target_v, "served_imitation_velocity_loss", "velocity_range_rad_s")):
                scale = np.array([r["reward_terms"][f"{coord}_imitation_{unit}"] for r in trace])
                expected = np.array([r["reward_terms"][f"{coord}_{suffix}"] for r in trace])
                e = float(np.max(np.abs(((desired - actual) / scale) ** 2 - expected)))
                primitive_errors.append(e)
                assert e < 2e-6, (name, kind, e)
            for array, tab, key in ((q, tables["kinematics"], coord), (served, tables["kinematics_reference"], coord + "_q_ref")):
                # STO recorder omits the terminal endpoint; do not validate an
                # extrapolated/clamped sample against the terminal trace state.
                supported = (time >= tab["time"][0] - 1e-9) & (time <= tab["time"][-1] + 1e-9)
                e = float(np.max(np.abs(np.interp(time[supported], tab["time"], tab[key]) - array[supported])))
                sto_trace_errors.append(e)
                assert e < 2e-6, (name, key, e)
            kin[joint] = {"actual_vs_target_deg": matching(np.degrees(q[mask]), np.degrees(target[mask])),
                          "served_vs_target_deg": matching(np.degrees(served[mask]), np.degrees(target[mask])),
                          "actual_vs_served_deg": matching(np.degrees(q[mask]), np.degrees(served[mask])),
                          "actual_velocity_vs_target_rad_s": matching(v[mask], target_v[mask]),
                          "served_velocity_vs_target_rad_s": matching(served_v[mask], target_v[mask])}
            qp[joint] = {k: profile(time, np.degrees(a), time, joint_phases[joint]) for k, a in (("target", target), ("served", served), ("actual", q))}
            kin[joint]["phase_mean_actual_vs_target_deg"] = matching(qp[joint]["actual"]["mean"], qp[joint]["target"]["mean"])
            kin[joint]["phase_mean_served_vs_target_deg"] = matching(qp[joint]["served"]["mean"], qp[joint]["target"]["mean"])
            tau = tables["sea_torques"][f"SEA_{joint.title()}_tau_spring"]
            torque[joint], tp[joint] = torque_metrics(torque_time, tau, time, joint_phases[joint])
            tm = torque[joint]
            tm["absolute_peak_nm"] = max(abs(tm["min_nm"]), abs(tm["max_nm"]))
            tm["canonical_plot_min_nm"] = -tm["max_nm"] if joint == "knee" else tm["min_nm"]
            tm["canonical_plot_max_nm"] = -tm["min_nm"] if joint == "knee" else tm["max_nm"]
            qsignals[joint] = {"target": target, "target_velocity": target_v, "actual": q, "served": served}
            for index, fraction in enumerate(GRID):
                profile_rows.append({"dataset": name, "target_family": spec["target"], "joint": joint, "prescribed_phase": fraction,
                                     **{f"q_{k}_mean_deg": float(vv["mean"][index]) for k, vv in qp[joint].items()},
                                     "q_actual_between_cycle_sd_deg": float(qp[joint]["actual"]["std"][index]),
                                     "tau_lowpass10_mean_nm": float(tp[joint]["10"]["mean"][index]),
                                     "tau_lowpass10_between_cycle_sd_nm": float(tp[joint]["10"]["std"][index])})
            summary_rows.append({"dataset": name, "target_family": spec["target"], "joint": joint,
                                 "actual_target_rmse_deg": kin[joint]["actual_vs_target_deg"]["rmse"],
                                 "served_target_rmse_deg": kin[joint]["served_vs_target_deg"]["rmse"],
                                 "actual_served_rmse_deg": kin[joint]["actual_vs_served_deg"]["rmse"],
                                 "actual_target_correlation": kin[joint]["actual_vs_target_deg"]["correlation"],
                                 "phase_shape_error": kin[joint]["phase_mean_actual_vs_target_deg"]["centered_unit_rms_shape_error"],
                                 "phase_range_ratio": kin[joint]["phase_mean_actual_vs_target_deg"]["range_ratio"],
                                 "torque_hf20_rms_nm": torque[joint]["cutoffs_hz"]["20"]["residual_hf_rms_nm"],
                                 "torque_hf20_normalized": torque[joint]["cutoffs_hz"]["20"]["residual_hf_rms_normalized_dynamic_rms"],
                                 "torque_lowpass10_phase_tv_over_range": torque[joint]["cutoffs_hz"]["10"]["phase_mean_total_variation_over_range"],
                                 "torque_lowpass10_cycle_shape_variability": torque[joint]["cutoffs_hz"]["10"]["cycle_shape_variability_normalized"],
                                 "torque_tv_normalized_per_s": torque[joint]["total_variation_normalized_per_s"]})
        phase_budget = []
        for lower in np.arange(0, 1, .1):
            subset = [r for r in records if lower <= r["phase"] < lower + .1 - 1e-14]
            if subset:
                phase_budget.append({"phase_start": float(lower), **aggregate_reward(subset)})
        grf = tables["online_grf"]
        gm = (grf["time"] >= WINDOW[0] - 1e-9) & (grf["time"] <= WINDOW[1] + 1e-9)
        fsm = {state: aggregate_reward([r for r in records if r["fsm_state"] == state]) for state in sorted({r["fsm_state"] for r in records})}
        datasets[name] = {"source": spec["path"], "target_family": spec["target"], "analysis_window_s": WINDOW,
                          "ankle_minus_knee_prescribed_phase_offset": {"min": float(phase_offset.min()), "max": float(phase_offset.max())},
                          "original_return_reconstruction_error": error, "kinematics": kin, "torque": torque,
                          "reward_common_contract": aggregate_reward(records), "reward_by_prescribed_phase": phase_budget,
                          "reward_by_physical_fsm_state": fsm,
                          "original_reward_config": summary["reward_config"],
                          "max_left_penetration_mm_window": float(1000 * grf["left_penetration"][gm].max()),
                          "physical_fsm_version": trace[-1]["phase_fsm"].get("fsm_behaviour_version", "unspecified"),
                          "final_fsm_counts": {k: trace[-1]["phase_fsm"].get(k) for k in ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count")}}
        signals[name] = {"trace": trace, "time": time, "phase": phase, "joint_phases": joint_phases, "q": qsignals, "qp": qp, "tp": tp, "tables": tables}
        write_csv(HERE / f"reward_steps_{name}.csv", records)
    comparisons = {}
    for group in ("LEFT", "RIGHT"):
        names = [n for n, s in sources.items() if s["target"] == group]
        reference = signals[names[0]]
        for name in names[1:]:
            s = signals[name]
            np.testing.assert_allclose(s["time"], reference["time"], atol=1e-8, rtol=0)
            np.testing.assert_allclose(s["phase"], reference["phase"], atol=1e-8, rtol=0)
            for joint in JOINTS:
                np.testing.assert_allclose(s["joint_phases"][joint], reference["joint_phases"][joint], atol=1e-8, rtol=0)
                np.testing.assert_allclose(s["q"][joint]["target"], reference["q"][joint]["target"], atol=1e-8, rtol=0)
                np.testing.assert_allclose(s["q"][joint]["target_velocity"], reference["q"][joint]["target_velocity"], atol=1e-8, rtol=0)
        # Rankings are only within one target and are observed trace rankings.
        comparisons[group] = {"target_and_times_identical": True, "datasets": names,
                              "observed_common_reward_order": sorted(names, key=lambda n: datasets[n]["reward_common_contract"]["sum"]["reward"], reverse=True),
                              "observed_kinematic_rmse_order": sorted(names, key=lambda n: sum(datasets[n]["kinematics"][j]["actual_vs_target_deg"]["rmse"] for j in JOINTS))}
    for name, metrics in datasets.items():
        metrics["v26_torque_shape_descriptive_only"] = {}
        for joint in JOINTS:
            a = signals[name]["tp"][joint]["10"]["mean"]
            b = signals["V26"]["tp"][joint]["10"]["mean"]
            metrics["v26_torque_shape_descriptive_only"][joint] = matching(a, b)
        if metrics["target_family"] == "RIGHT":
            comparisons_same_target = {}
            for joint in JOINTS:
                t = signals["V26"]["tables"]["sea_torques"]["time"]
                own_t = signals[name]["tables"]["sea_torques"]["time"]
                own = np.interp(t, own_t, signals[name]["tp"][joint]["10"]["filtered"])
                baseline = signals["V26"]["tp"][joint]["10"]["filtered"]
                mask = (t >= WINDOW[0] - 1e-9) & (t <= WINDOW[1] + 1e-9)
                intervals = signals["V26"]["tp"][joint]["10"]["intervals_s"]
                cycle_mask = np.zeros(len(t), dtype=bool)
                for start, end in intervals:
                    cycle_mask |= (t >= start) & (t <= end)
                comparisons_same_target[joint] = {
                    "same_time_window_lowpass10_nm": matching(own[mask], baseline[mask]),
                    "same_complete_cycles_lowpass10_nm": matching(own[cycle_mask], baseline[cycle_mask]),
                    "same_complete_cycles_intervals_s": intervals,
                    "two_cycle_phase_resampled_lowpass10_nm": matching(signals[name]["tp"][joint]["10"]["cycles"].ravel(), signals["V26"]["tp"][joint]["10"]["cycles"].ravel()),
                }
            metrics["v26_same_target_torque_comparison_no_time_shift"] = comparisons_same_target
    verification = self_checks()
    verification.update({"original_return_max_error": max(reconstruction_errors), "imitation_primitive_max_abs_error": max(primitive_errors),
                         "sto_trace_max_error_rad": max(sto_trace_errors), "same_target_reference_arrays_verified": True,
                         "same_target_velocity_arrays_verified": True, "per_joint_prescribed_phases_match_within_target": True,
                         "new_simulations": 0, "new_training": 0, "datasets": len(datasets)})
    limits = ["One deterministic trace per controller, one subject/model, short fixed window; no population or robustness claim.",
              "Complete prescribed cycles of each joint define phase averages; RIGHT ankle has a +0.013 phase offset from knee. Reward bins use knee prescribed phase. Physical FSM groups are separate and never realign the reference.",
              "V26 torque profiles are descriptive shape benchmarks, never torque targets for LEFT.",
              "Common reward uses recorded target-specific losses and best80 pen2 scalar configuration; comparisons only within identical targets.",
              "Offline trace ranking is not a proof of reward optimality, trainability, causal reward effects, or a generalizable learning improvement.",
              "Historical oracles use a 10 ms preview and are not learned policies; V26 has a different observation/FSM history.",
              "Filtering is zero-phase offline analysis only; HF is x minus lowpass, not an orthogonal spectral projection.",
              "Shape error is centered and scale-normalized; report amplitude/range, RMSE, and penetration separately."]
    save(HERE / "sources.json", sources)
    save(HERE / "metrics.json", {"common_reward_config": asdict(common_cfg), "datasets": datasets, "within_target_comparisons": comparisons,
         "sign_convention": "Raw OpenSim STO/trace signs in metrics and ordinary diagnostic plots. Canonical plot convention multiplies knee q and torque by -1; ankle unchanged. LEFT_focused_comparison follows canonical convention.", "limitations": limits})
    write_csv(HERE / "summary.csv", summary_rows)
    write_csv(HERE / "phase_profiles.csv", profile_rows)
    plot(signals, datasets, sources)
    for path in (Path(__file__), PACKAGE / "reward_function.py", PACKAGE / "tuning_terms.py", TG / "osim_trj_cmc_like.py"):
        read(path)
    assert all(hashlib.sha256((REPO / path).read_bytes()).hexdigest() == digest for path, digest in HASHES.items())
    verification["source_hashes_unchanged"] = True
    save(HERE / "input_hashes.json", HASHES)
    save(HERE / "verification.json", verification)
    print(json.dumps({"summary": summary_rows, "within_target_comparisons": comparisons,
                      "common_rewards_by_target": {g: {n: datasets[n]["reward_common_contract"]["sum"] for n in v["datasets"]} for g, v in comparisons.items()}, "verification": verification}, indent=2))


def self_checks():
    t = np.arange(0, 6, .001)
    slow, fast = np.sin(2 * np.pi * t), .2 * np.sin(2 * np.pi * 60 * t)
    low, high = filter_split(t, slow + fast, 10)
    interior = (t > .5) & (t < 5.5)
    assert rms(low[interior] - slow[interior]) < .001
    assert abs(rms(high[interior]) - .2 / np.sqrt(2)) < .001
    shape = matching(3 * slow + 5, slow)
    assert shape["centered_unit_rms_shape_error"] < 1e-12
    assert abs(shape["range_ratio"] - 3) < 1e-12
    p = profile(t, slow, t, (t + .2) % 1, window=(.1, 5.9))
    assert len(p["intervals_s"]) == 5
    assert rms(p["mean"] - np.sin(2 * np.pi * (GRID - .2))) < 1e-6
    return {"passed": True, "synthetic_filter_split": True, "shape_affine_invariance": True, "complete_cycle_phase_alignment": True}


def plot(signals, metrics, sources):
    PLOTS.mkdir(parents=True, exist_ok=True)
    groups = {g: [n for n, s in sources.items() if s["target"] == g] for g in ("LEFT", "RIGHT")}
    for group, names in groups.items():
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained", sharex=True)
        for j, joint in enumerate(JOINTS):
            for col, kind in enumerate(("actual", "served")):
                ax = axes[j, col]
                ax.plot(100 * GRID, signals[names[0]]["qp"][joint]["target"]["mean"], "k--", linewidth=2, label=f"Prescribed {group}")
                for name in names:
                    p = signals[name]["qp"][joint][kind]
                    ax.plot(100 * GRID, p["mean"], label=name, color=COLORS.get(name))
                    ax.fill_between(100 * GRID, p["mean"] - p["std"], p["mean"] + p["std"], alpha=.07, color=COLORS.get(name))
                ax.set(title=f"{joint.title()} — {kind}", ylabel="Angle [deg, OpenSim sign]", xlabel=f"Prescribed {group} phase [%]")
                ax.legend(fontsize=7); ax.grid(alpha=.25)
        fig.suptitle(f"Kinematic shape, target {group}: complete prescribed cycles only\nShading = between-cycle SD; no lag fitting or FSM realignment")
        fig.savefig(PLOTS / f"kinematic_shape_{group}.png", dpi=170); plt.close(fig)
    names = list(sources)
    fig, axes = plt.subplots(len(names), 2, figsize=(14, 2.6 * len(names)), sharex=True, layout="constrained", squeeze=False)
    for row, name in enumerate(names):
        data = signals[name]
        table = data["tables"]["sea_torques"]
        for col, joint in enumerate(JOINTS):
            ax = axes[row, col]
            ax.plot(table["time"], table[f"SEA_{joint.title()}_tau_spring"], color=COLORS.get(name), alpha=.5, linewidth=.6, label="Raw spring torque")
            ax.plot(table["time"], data["tp"][joint]["10"]["filtered"], color="black", linewidth=1.1, label="Offline LP 10 Hz")
            ax.set(title=f"{name} / {sources[name]['target']} — {joint}", ylabel="Torque [Nm, OpenSim sign]", xlim=WINDOW)
            ax.grid(alpha=.25); ax.legend(fontsize=7)
    axes[-1, 0].set_xlabel("Simulation time [s]"); axes[-1, 1].set_xlabel("Simulation time [s]")
    fig.suptitle("Torque shape and oscillations — raw torque + offline low-pass; axes retain physical amplitude")
    fig.savefig(PLOTS / "torque_time_shapes.png", dpi=170); plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained", sharex=True)
    for col, joint in enumerate(JOINTS):
        for name in names:
            mean = signals[name]["tp"][joint]["10"]["mean"]
            axes[0, col].plot(100 * GRID, mean, label=f"{name} ({sources[name]['target']})", color=COLORS.get(name))
            axes[1, col].plot(100 * GRID, standardized(mean), label=f"{name} ({sources[name]['target']})", color=COLORS.get(name))
        axes[0, col].set(title=joint.title(), ylabel="LP 10 Hz torque [Nm]")
        axes[1, col].set(ylabel="Centered / own RMS", xlabel="Own prescribed reference phase [%]")
    for ax in axes.flat:
        ax.legend(fontsize=7); ax.grid(alpha=.25)
    fig.suptitle("Torque cycle morphology — each reference has its own prescribed phase\nV26 is a descriptive benchmark, not a LEFT torque target")
    fig.savefig(PLOTS / "torque_phase_shapes_descriptive.png", dpi=170); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), layout="constrained", sharex=True)
    for ax, joint in zip(axes, JOINTS):
        for name in groups["RIGHT"]:
            t = signals[name]["tables"]["sea_torques"]["time"]
            ax.plot(t, signals[name]["tp"][joint]["10"]["filtered"], label=name, color=COLORS.get(name))
        for start, end in signals["V26"]["tp"][joint]["10"]["intervals_s"]:
            ax.axvspan(start, end, color="gray", alpha=.07)
            ax.axvline(start, color="gray", linewidth=.6, linestyle="--")
        ax.set(ylabel=f"{joint} LP 10 Hz [Nm]", xlim=WINDOW); ax.legend(); ax.grid(alpha=.25)
    axes[-1].set_xlabel("Same simulation time [s]; shaded complete RIGHT prescribed cycles")
    fig.suptitle("RIGHT and V26: identical target q, qdot, phase per joint and timestamps verified\nNo optimized shift; ankle phase is knee phase + 0.013 for all RIGHT runs")
    fig.savefig(PLOTS / "torque_RIGHT_V26_same_time.png", dpi=170); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout="constrained")
    for ax, (group, names) in zip(axes, groups.items()):
        totals = [metrics[n]["reward_common_contract"]["sum"] for n in names]
        ax.bar(names, [r["actual_score"] for r in totals], label="Actual imitation score")
        ax.bar(names, [r["served_score"] for r in totals], bottom=[r["actual_score"] for r in totals], label="Served imitation score")
        bottom = np.zeros(len(names))
        for key, label in (("command_rate_cost", "Command cost"), ("other_base_cost", "Other base costs"), ("penetration_cost", "Penetration"), ("moment_flip_cost", "GRF moment")):
            values = np.array([-r[key] for r in totals])
            ax.bar(names, values, bottom=bottom, label=label); bottom += values
        ax.plot(names, [r["reward"] for r in totals], "ko", label="Total incl. clipping / remaining costs")
        ax.axhline(0, color="black", linewidth=.6); ax.set(title=f"Same target {group} only", ylabel="Episode points / 500 steps")
        ax.tick_params(axis="x", rotation=20); ax.grid(axis="y", alpha=.25)
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, fontsize=7, loc="outside lower center", ncol=4)
    fig.suptitle("One common reward configuration (best80 pen2), recorded target-specific errors\nPanels are separate tasks; no LEFT/RIGHT return ranking")
    fig.savefig(PLOTS / "common_reward_within_target.png", dpi=170); plt.close(fig)
    if "LEFT_sequence" in signals:
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
        for col, joint in enumerate(JOINTS):
            sign = -1 if joint == "knee" else 1
            ax = axes[0, col]
            target = signals["best80LEFT"]["qp"][joint]["target"]["mean"]
            ax.plot(100 * GRID, sign * target, "k--", linewidth=2, label="Prescribed LEFT target")
            for name, label in (("best80LEFT", "best80 LEFT"), ("LEFT_sequence", "Anticipated LEFT sequence")):
                profile = signals[name]["qp"][joint]["actual"]
                mean = sign * profile["mean"]
                ax.plot(100 * GRID, mean, color=COLORS[name], linewidth=1.8, label=label)
                ax.fill_between(100 * GRID, mean - profile["std"], mean + profile["std"], color=COLORS[name], alpha=.09)
            ax.set(title=f"{joint.title()}: actual joint kinematics", ylabel="Angle [deg]", xlabel="LEFT prescribed phase [%]")
            ax = axes[1, col]
            for name, label in (("best80LEFT", "best80 LEFT"), ("LEFT_sequence", "Anticipated LEFT sequence"), ("V26", "V26 / own RIGHT phase (descriptive)")):
                mean = signals[name]["tp"][joint]["10"]["mean"]
                ax.plot(100 * GRID, sign * mean, color=COLORS[name], linewidth=1.8, linestyle="--" if name == "V26" else "-", label=label)
            ax.set(title=f"{joint.title()}: spring torque, offline LP 10 Hz", ylabel="Torque [Nm]", xlabel="Own prescribed phase [%]; V26 uses RIGHT")
        for ax in axes.flat:
            ax.legend(fontsize=8); ax.grid(alpha=.25)
        fig.suptitle("LEFT: improved kinematic imitation and smoother torques; V26 remains a descriptive torque benchmark\nTwo complete cycles; shading = between-cycle SD, not uncertainty. Canonical knee sign; no fitted time shift.", fontsize=11)
        fig.savefig(PLOTS / "LEFT_focused_comparison.png", dpi=180); plt.close(fig)


if __name__ == "__main__":
    main()
