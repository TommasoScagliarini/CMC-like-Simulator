"""Diagnostic target-following rollout; no policy learning or runtime source edits.

Both arms use the current 39D checkpoint's simulator settings and FSM v3.
The sole intervention is to command the left-phase or historical right-shifted
target directly as the next absolute endpoint. The existing governor and SEA
controllers remain active. This is a filtered oracle, not perfect kinematic
replay, and is explicitly labelled as such in the output artifacts.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
sys.path.insert(0, str(TG))

import numpy as np
from unified_MLP_39 import rollout_eval as evaluator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=("left", "right"), required=True)
    opts, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0], *remaining]
    args = evaluator.parse_args()
    assert args.action_selection == "deterministic"
    evaluator._load_inference_stack()
    from osim_trj_cmc_like import GaitPhaseClock, PhaseBasedImitationTarget, CMCLikeProsthesisTrajectoryEnv
    original_factory = evaluator.env_factory.make_cmc_env
    original_policy = evaluator._policy_action
    holder = {}
    references = []

    def factory(config):
        env = original_factory(config)
        base = env.unwrapped
        holder["base"] = base
        assert base.env_cfg.binary_phase_actor_fsm_version == "v3"
        if opts.source == "right":
            clock = GaitPhaseClock(base._load_sound_heel_strikes("right"),
                                   phase_offset=float(base.env_cfg.gait_clock_phase_offset))
            target = PhaseBasedImitationTarget(base.base_kin, clock,
                base.env_cfg.imitation_sound_coords, base.env_cfg.imitation_phase_shifts,
                fallback_phase_shift=base.env_cfg.imitation_phase_shift,
                phase_samples=base.env_cfg.imitation_phase_samples,
                time_window=(base.cfg.t_start, base.cfg.t_end))
            assert target.available
            base.imitation_target = target.get
            base._imitation_position_ranges_rad, base._imitation_velocity_ranges_rad_s = (
                CMCLikeProsthesisTrajectoryEnv._build_imitation_error_ranges(base))
        reference_path = (TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout/rollout_policy_trace.json"
                          if opts.source == "right" else PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/rollout_policy_trace.json")
        reference = json.loads(reference_path.read_text())
        for row in reference:
            q, qd, _ = base.imitation_target(row["time"])
            for joint in base.cfg.pros_coords:
                np.testing.assert_allclose(q[joint], row["imitation_target_q"][joint], atol=1e-10)
                np.testing.assert_allclose(qd[joint], row["imitation_target_qdot"][joint], atol=1e-10)
        holder["target_verified_against_recorded_trace"] = True
        return env

    def oracle_action(module, obs, shape, **kwargs):
        base = holder["base"]
        target_time = min(base.t + base.env_cfg.segment_duration, base._episode_end)
        q, qd, phases = base.imitation_target(target_time)
        values = []
        for joint in base.cfg.pros_coords:
            low, high = base.env_cfg.absolute_bounds_rad[joint]
            values.append(2 * (q[joint] - low) / (high - low) - 1)
        action = np.asarray(values, dtype=np.float32).reshape(shape)
        assert np.all(np.abs(action) <= 1), "Oracle target outside absolute action limits"
        references.append({"time_s": target_time, "q": q, "qdot": qd, "phases": phases,
                           "action": action.tolist()})
        return action

    evaluator.env_factory.make_cmc_env = factory
    evaluator._policy_action = oracle_action
    try:
        summary = evaluator.run(args)
    finally:
        evaluator.env_factory.make_cmc_env = original_factory
        evaluator._policy_action = original_policy
    folder = evaluator._resolve_output_dir(args.output_dir, "rollout_eval")
    intervention = {"diagnostic_only": True, "controller": "direct target endpoint through original governor",
                    "learned_policy_used": False, "source": opts.source,
                    "fsm": "v3", "reference_trace_match": holder["target_verified_against_recorded_trace"],
                    "caveat": "Right arm reward uses right target but observation/contract metadata remain from the 39D scaffold; no NN acts on these observations."}
    summary["diagnostic_intervention"] = intervention
    (folder / "rollout_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (folder / "oracle_reference.json").write_text(json.dumps(references, indent=2) + "\n")
    (folder / "intervention.json").write_text(json.dumps(intervention, indent=2) + "\n")
    print(json.dumps({k: summary.get(k) for k in ("ok", "steps", "end_reason", "episode_return", "grf_penetration_max_m")}, indent=2))


if __name__ == "__main__":
    main()
