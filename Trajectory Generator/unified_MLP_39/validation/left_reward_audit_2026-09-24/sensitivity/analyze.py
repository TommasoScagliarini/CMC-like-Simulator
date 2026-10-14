"""Offline frozen-trace reward sensitivities. Never runs an environment/policy.

Run with envCMC-rllib Python; output stays beside this script. Every variant
changes one reward family; outcomes/trajectories/terminations remain recorded.
"""
from __future__ import annotations

import os
for _key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[_key] = "1"
import csv
import hashlib
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
TG = REPO / "Trajectory Generator"
sys.path.insert(0, str(TG))
from unified_MLP_39.reward_function import compute_reward, reward_config_from_mapping

ROLLOUT = TG / "unified_MLP_39/runs/rollout"
DATASETS = {
    "left40": ROLLOUT / "MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "left72": ROLLOUT / "MLP39_completion_2026-09-22_best072/deterministic",
    "oracle_left": ROLLOUT / "causal_difference_2026-09-21_oracle_left",
    "right60": ROLLOUT / "MLP39_right_v26_60iter_2026-09-23_last60",
}
for _noise in ("1", "0.5"):
    for _seed in range(123, 128):
        DATASETS[f"left72_noise{_noise}_seed{_seed}"] = ROLLOUT / f"MLP39_completion_2026-09-22_best072/noise_{_noise}_seed{_seed}"

VARIANTS = {
    "baseline": {},
    "command_rate_half": {"command_rate_weight": 0.025},
    "command_rate_zero_diagnostic": {"command_rate_weight": 0.0},
    "actual_served_blend_50_50": {"blend_served_imitation": 0.5, "blend_imitation": 0.5},
    "ankle_position_weight_double": {"imitation_ankle_position_weight": 4.0},
    "served_sharpness_half": {"served_imitation_weight": 4.0},
    "base_clip_low_minus_one": {"clip_low": -1.0},
    "ankle_flip_weight_double": {"grf_ankle_moment_flip_weight": 0.2},
    "penetration_weight_double": {"grf_penetration_weight": 2.0},
}
CMD_TERMS = ("segment_delta_loss", "qdot_ref_loss", "qddot_ref_loss", "jerk_ref_loss", "reference_governor_loss", "u_rate_loss")


def variant_terms(row, cfg, variant):
    terms = dict(row["reward_terms"])
    if variant == "ankle_position_weight_double":
        delta = cfg.imitation_ankle_position_weight - float(terms["pros_ankle_angle_imitation_position_weight"])
        terms["sound_imitation_loss"] += delta * terms["pros_ankle_angle_imitation_position_loss"]
        terms["served_imitation_loss"] += delta * terms["pros_ankle_angle_served_imitation_position_loss"]
    return terms


def main():
    hashes = {}
    results = {}
    reward_arrays = {}
    csv_rows = []
    for name, path in DATASETS.items():
        files = [path / "rollout_summary.json", path / "rollout_policy_trace.json"]
        for file in files:
            hashes[str(file.relative_to(REPO))] = hashlib.sha256(file.read_bytes()).hexdigest()
        summary = json.loads(files[0].read_text())
        trace = json.loads(files[1].read_text())
        cfg = reward_config_from_mapping(summary["reward_config"])
        assert cfg.reward_mode == "imitation"
        baseline_pairs = [compute_reward(r["reward_terms"], cfg, reference=r["policy_segment_values"]) for r in trace]
        baseline = np.asarray([item[0] for item in baseline_pairs])
        baseline_preclip = np.asarray([cfg.blend_served_imitation*c["served_imitation_score"]+cfg.blend_imitation*c["imitation_score"]-c["penalty"] for _,c in baseline_pairs])
        baseline_clip_interior = (baseline_preclip>cfg.clip_low)&(baseline_preclip<cfg.clip_high)
        discrepancy = float(baseline.sum() - summary["episode_return"])
        assert abs(discrepancy) < 1e-8, (name, discrepancy)
        terms = {k: np.asarray([r["reward_terms"].get(k, 0.0) for r in trace], dtype=float) for k in set().union(*(r["reward_terms"].keys() for r in trace))}
        variants = {}
        reward_arrays[name] = {}
        for variant, changes in VARIANTS.items():
            vcfg = replace(cfg, **changes)
            recomputed = [compute_reward(variant_terms(row, vcfg, variant), vcfg, reference=row["policy_segment_values"]) for row in trace]
            reward = np.asarray([item[0] for item in recomputed])
            comps = [item[1] for item in recomputed]
            base = np.asarray([c["reward_base"] for c in comps])
            unclip = np.asarray([vcfg.blend_served_imitation * c["served_imitation_score"] + vcfg.blend_imitation * c["imitation_score"] - c["penalty"] for c in comps])
            # Discounted complete trace is descriptive; it is not the PPO GAE
            # advantage, which additionally uses learned values/bootstrapping.
            discounts = np.power(0.99, np.arange(len(reward)))
            result = {
                "return_on_frozen_trajectory": float(reward.sum()),
                "delta_return": float((reward-baseline).sum()),
                "discounted_reward_sum_gamma099": float(np.dot(discounts, reward)),
                "steps": len(trace),
                "steps_base_clip_low": int(np.count_nonzero(unclip < vcfg.clip_low)),
                "steps_base_clip_high": int(np.count_nonzero(unclip > vcfg.clip_high)),
                "base_clip_lift_sum": float(np.sum(base-unclip)),
                "mean_reward": float(reward.mean()),
                "reward_min": float(reward.min()),
                "changed_steps": int(np.count_nonzero(np.abs(reward-baseline)>1e-12)),
            }
            variants[variant] = result
            reward_arrays[name][variant] = reward
            csv_rows.append({"dataset":name,"variant":variant,**result})
        cmd = {k: {"weighted_sum": float((cfg.command_rate_weight/6*terms[k]).sum()), "mean_raw_loss": float(terms[k].mean()), "near_cap25_steps":int(np.count_nonzero(terms[k]>=25-1e-10))} for k in CMD_TERMS}
        err = float(np.max(np.abs(sum(terms[k] for k in CMD_TERMS)/6-terms["command_rate_loss"])))
        assert err < 1e-12
        # Configured defaults shared by these aligned runs: reconstruct, rather
        # than assuming that the recorded aggregate is temporal action change.
        raw_delta = np.asarray([np.diff(r["policy_segment_values"],axis=0)[0] for r in trace])
        normalized_delta = raw_delta / np.asarray([0.05,0.04])
        reconstructed_delta = np.mean(np.minimum(normalized_delta**2,25.0),axis=1)
        delta_error = float(np.max(np.abs(reconstructed_delta-terms["segment_delta_loss"])))
        assert delta_error < 1e-10
        delta_caps = {
            "knee_steps_at_cap":int(np.count_nonzero(np.abs(normalized_delta[:,0])>=5)),
            "ankle_steps_at_cap":int(np.count_nonzero(np.abs(normalized_delta[:,1])>=5)),
            "both_joints_at_cap":int(np.count_nonzero(np.all(np.abs(normalized_delta)>=5,axis=1))),
            "knee_cap_threshold_rad":0.25,"ankle_cap_threshold_rad":0.20,
            "reconstruction_max_error":delta_error,
            "definition":"One-knot policy endpoint minus current served/filtered continuity anchor, not change between consecutive raw policy endpoints.",
        }
        # Conditional sensitivity of the scalar only: move one served position
        # 10% toward its target while freezing all physical and command losses.
        local = {}
        for coord in ("pros_knee_angle", "pros_ankle_angle"):
            change = []
            for row, old in zip(trace, baseline):
                t = dict(row["reward_terms"])
                pos = float(t[f"{coord}_served_imitation_position_loss"])
                weight = float(t[f"{coord}_imitation_position_weight"])
                t["served_imitation_loss"] -= 0.19 * weight * pos
                change.append(compute_reward(t,cfg,reference=row["policy_segment_values"])[0]-old)
            phase = terms[f"{coord}_imitation_target_phase"]
            gains = np.asarray(change)
            per_phase = []
            for bin_index in range(10):
                mask = (phase>=bin_index/10)&(phase<(bin_index+1)/10)
                n = int(np.count_nonzero(mask))
                if not n:
                    continue
                per_phase.append({
                    "phase_start":bin_index/10,"phase_end":(bin_index+1)/10,"samples":n,
                    "mean_reward_gain_10pct_served_position_error_correction":float(gains[mask].mean()),
                    "mean_weighted_served_position_loss":float((terms[f"{coord}_served_imitation_position_loss"]*terms[f"{coord}_imitation_position_weight"])[mask].mean()),
                    "mean_reward_served_loss_sensitivity_abs":float(np.mean(baseline_clip_interior[mask]*cfg.blend_served_imitation*cfg.served_imitation_weight/(1+cfg.served_imitation_weight*terms["served_imitation_loss"][mask])**2)),
                    "mean_reward_actual_loss_sensitivity_abs":float(np.mean(baseline_clip_interior[mask]*cfg.blend_imitation*cfg.imitation_weight/(1+cfg.imitation_weight*terms["sound_imitation_loss"][mask])**2)),
                })
            local[coord] = {"sum_reward_gain_frozen_other_terms":float(np.sum(change)), "mean_reward_gain":float(np.mean(change)), "zero_gain_steps":int(np.count_nonzero(np.abs(change)<1e-12)), "per_phase":per_phase}
        results[name] = {
            "source":str(path.relative_to(REPO)),
            "end_reason":summary.get("end_reason"),
            "recorded_return":summary["episode_return"],
            "reward_reconstruction_sum_error":discrepancy,
            "command_aggregation_max_error":err,
            "variants":variants,
            "command_rate_components":cmd,
            "segment_delta_caps":delta_caps,
            "local_served_position_error_reduction_10percent":local,
        }
        if summary.get("end_reason") in {"grf_penetration", "grf_penetration_hard"}:
            last = trace[-1]
            rt = last["reward_terms"]
            reward, components = compute_reward(rt,cfg,reference=last["policy_segment_values"])
            terminal = {
                "step":last["step"],"recorded_time":last["time"],
                "time_duplicate_with_previous":bool(len(trace)>1 and last["time"]==trace[-2]["time"]),
                "reward":reward,"reward_base":components["reward_base"],
                "safety_term":components["safety_term"],
                "penetration_term":components["grf_penetration_term"],
                "ankle_flip_term":components["grf_ankle_moment_flip_term"],
                "oob_term":components["oob_term"],
                "sound_imitation_loss":rt["sound_imitation_loss"],
                "served_imitation_loss":rt["served_imitation_loss"],
                "recorded_penetration_m":rt["grf_penetration_m"],
            }
            if summary["end_reason"] == "grf_penetration_hard":
                scale=summary["grf_penetration_termination_m"]-summary["grf_penetration_penalty_threshold_m"]
                penalty_at_guard=cfg.grf_penetration_weight*((0.030-summary["grf_penetration_penalty_threshold_m"])/scale)**2
                terminal.update({
                    "penetration_penalty_at_30mm_guard_boundary":penalty_at_guard,
                    "reward_with_boundary_penetration_only_added_frozen_other_terms":reward-penalty_at_guard,
                    "boundary_calculation_caveat":"Scalar boundary calculation using 30 mm guard in root config.py; not a recovered physical sample or a re-simulated terminal transition. Contact moment is unknown on the aborted step.",
                })
            results[name]["terminal_reward_audit"]=terminal
    for name,data in results.items():
        if name.startswith("left72_noise") and "terminal_reward_audit" in data:
            stop=data["variants"]["baseline"]["steps"]-1
            start=max(0,stop-10)
            data["ten_completed_steps_before_failure_vs_nominal_same_steps"]={
                "first_step":start+1,"last_step":stop,
                "caveat":"Same step indexes from fixed-start rollouts, recorded trajectories differ. Excludes terminal/partial step; frozen scalar ranking only.",
                "variants":{
                    variant:{
                        "failed_rollout_window_reward_sum":float(values[start:stop].sum()),
                        "deterministic_window_reward_sum":float(reward_arrays["left72"][variant][start:stop].sum()),
                        "failed_minus_deterministic":float(values[start:stop].sum()-reward_arrays["left72"][variant][start:stop].sum()),
                    } for variant,values in reward_arrays[name].items()
                },
            }
    math_info = {
        "gamma":0.99,"lambda":0.9,"dt_s":0.01,
        "reward_discount_half_life_s":0.01*math.log(0.5)/math.log(0.99),
        "gae_residual_geometric_half_life_s":0.01*math.log(0.5)/math.log(0.99*0.9),
        "reward_discount_100ms":0.99**10,
        "reward_discount_500ms":0.99**50,
        "reward_discount_1s":0.99**100,
        "reward_discount_1p6s":0.99**160,
        "caveat":"GAE includes learned value bootstrapping: its residual geometric half-life is not a hard temporal horizon. PPO optimizes advantages, not recorded undiscounted episode totals.",
    }
    output = {"contract":"Frozen trajectories; only scalar rewards rescored. No policy training, new simulation, or prediction of changed physical trajectories.","variants":VARIANTS,"temporal_discount":math_info,"datasets":results}
    (HERE / "metrics.json").write_text(json.dumps(output,indent=2)+"\n")
    with (HERE/"variants.csv").open("w",newline="") as handle:
        writer=csv.DictWriter(handle,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    hashes[str(Path(__file__).relative_to(REPO))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    hashes[str((TG/"unified_MLP_39/reward_function.py").relative_to(REPO))]=hashlib.sha256((TG/"unified_MLP_39/reward_function.py").read_bytes()).hexdigest()
    for source in (REPO/"config.py", TG/"osim_trj_cmc_like.py", TG/"unified_MLP_39/train_ppo_mlp.py", TG/"unified_MLP_39/configs/temporary_v26_aligned_left_phase_fsmv3_2026-09-21.yaml"):
        hashes[str(source.relative_to(REPO))]=hashlib.sha256(source.read_bytes()).hexdigest()
    (HERE/"input_hashes.json").write_text(json.dumps(hashes,indent=2)+"\n")
    print(json.dumps({name:{variant:round(value["delta_return"],3) for variant,value in data["variants"].items()} for name,data in results.items()},indent=2))


if __name__ == "__main__":
    main()
