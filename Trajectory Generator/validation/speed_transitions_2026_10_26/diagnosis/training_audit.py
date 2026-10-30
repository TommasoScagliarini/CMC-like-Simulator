"""Read-only audit of the documented best47 task; does not train or simulate."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASELINE = ROOT / "Trajectory Generator/baseline_MLP"
sys.path.insert(0, str(BASELINE))
import _bootstrap

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: F401,E402
import yaml  # noqa: E402
from reward_function import RewardConfig, RewardShapingWrapper  # noqa: E402


def main():
    run = ROOT / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01"
    summary_file = run / "summary.json"
    preset_file = BASELINE / "training_exnovo_cfg.yaml"
    meta_file = run / "checkpoint_best_meta.json"
    summary = json.loads(summary_file.read_text(encoding="utf-8"))
    preset = yaml.safe_load(preset_file.read_text(encoding="utf-8"))
    meta = json.loads(meta_file.read_text(encoding="utf-8"))
    actual_reward = summary["reward_config"]
    reward_differences = {
        key: {"current": value, "training": actual_reward.get(key)}
        for key, value in preset["reward"].items()
        if value != actual_reward.get(key)
    }
    # The documented source move is the only difference; no numerical mismatch.
    assert set(reward_differences) == {"morphology_profile"}
    history = summary["history"]
    starts = sorted({name for row in history for name in row["per_start_metrics"]})
    lengths = [start["episode_length_mean"] for row in history
               for start in row["per_start_metrics"].values()
               if start.get("episode_length_mean") is not None]
    trace_file = HERE.parent / "stop_restart_aligned/rollout_policy_trace.json"
    trace = json.loads(trace_file.read_text(encoding="utf-8"))
    feature_names = list(trace[0]["actor_observation_before"])
    assert len(feature_names) == 35
    fixed_contact_file = HERE / "fsm_diagnosis.json"
    fixed_contact = json.loads(fixed_contact_file.read_text(encoding="utf-8"))
    payloads = fixed_contact["cases"]["constant_loaded_baseline_timeout_10s"]["diagnostic_payloads"]
    reward_cfg = RewardConfig.from_mapping(actual_reward)
    reward_probe = {}
    for elapsed, payload in payloads.items():
        # This production method only reads reward_config and the FSM payload.
        terms = RewardShapingWrapper._phase_regular_terms(
            SimpleNamespace(reward_config=reward_cfg), {"phase_fsm": payload}
        )
        reward_probe[elapsed] = {
            "stance_elapsed_s": terms["phase_stance_elapsed_s"],
            "timeout_loss": terms["phase_timeout_loss"],
            "timeout_penalty_subtracted_after_clip": reward_cfg.phase_timeout_penalty_weight * terms["phase_timeout_loss"],
        }
    result = {
        "scope": "Stored best47 training summary + current equivalent preset + actual rollout actor schema; no claim to reconstruct removed historical datasets.",
        "logical_best_iteration": meta["logical_iteration"],
        "successful_updates_in_run": len(history),
        "setup_xml_at_training": summary["setup_xml_path"],
        "resume_from": summary["resume_from"],
        "sampled_start_conditions_all_updates": starts,
        "episode_steps_mean_range": [min(lengths), max(lengths)],
        "current_simulation_config": preset["simulation"],
        "reward_mode": actual_reward["reward_mode"],
        "reward_differences_current_vs_training": reward_differences,
        "walking_objective_parameters": {key: actual_reward[key] for key in (
            "blend_phase_regular", "blend_phase_event_progress", "phase_period_nominal_s",
            "phase_period_soft_margin_s", "phase_stance_timeout_s", "phase_stance_hard_timeout_s",
            "phase_swing_hard_timeout_s", "phase_cycle_complete_bonus", "phase_min_cycle_knee_excursion_rad",
            "morphology_knee_weight", "morphology_ankle_weight")},
        "inactive_imitation_blend_explanation": "reward_function.compute_reward selects imitation terms only when reward_mode == imitation; stored best47 mode is ex_novo.",
        "fixed_loaded_contact_reward_probe": reward_probe,
        "reward_probe_scope": "Production phase loss only, not total reward; hard timeout extended in sensor probe solely to inspect sustained stance. Soft penalty retains actual training values.",
        "actor_feature_names": feature_names,
        "direct_stop_or_belt_speed_input": False,
        "absent_intent_input_caveat": "Indirect changes in ipsilateral mechanics can reach the actor; missing explicit intent alone is not a mathematical proof that stopping cannot emerge.",
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (summary_file, preset_file, meta_file, trace_file, fixed_contact_file,
                                       BASELINE / "reward_function.py", Path(__file__))},
    }
    output = HERE / "training_audit.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "logical_best_iteration", "successful_updates_in_run", "sampled_start_conditions_all_updates",
        "episode_steps_mean_range", "reward_mode", "reward_differences_current_vs_training")}, indent=2))


if __name__ == "__main__":
    main()
