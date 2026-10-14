"""Bounded OpenSim smoke of contact transport, with no learner or Ray runtime.

Nominal before/after uses the saved pre-fix step() on fresh 39D environments.
Hard guards use real measured contact and a lowered test-only guard/soft limit;
the production 20/28/30 mm thresholds are never written to a configuration.
"""
from __future__ import annotations

import ast
import copy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import sys
import time

for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[name] = "1"
HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from unified_MLP_39 import train_ppo_mlp as train


def main():
    started = time.monotonic()
    train._load_training_stack()
    from unified_MLP_39 import env_factory
    import osim_trj_cmc_like as adapter
    import numpy as np

    config_path = PACKAGE / "configs/temporary_v26_aligned_left_phase_fsmv3_2026-09-21.yaml"
    sys.argv = [str(Path(train.__file__)), "--config", str(config_path), "--iterations", "1"]
    args = train.parse_args()
    config = train.build_config(args, dict(args._cfg_reward))
    env = env_factory.make_cmc_env(dict(config.env_config))
    base = env.unwrapped
    original_step = adapter.CMCLikeProsthesisTrajectoryEnv.step
    saved_source = (HERE / "before/osim_trj_cmc_like.py").read_text(encoding="utf-8")
    tree = ast.parse(saved_source)
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == "CMCLikeProsthesisTrajectoryEnv")
    step = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "step")
    namespace = dict(vars(adapter))
    exec(compile(ast.Module(body=[step], type_ignores=[]), "<saved-before-step>", "exec"), namespace)
    before_step = namespace["step"]
    action = np.zeros(env.action_space.shape, dtype=np.float32)
    production_env_config = base.env_cfg
    production_hard_limit = base.runner._cfg.online_grf_max_penetration_m
    production_reward_config = env.reward_config
    results = {
        "training_started": False, "ray_runtime_started": False,
        "configuration": str(config_path.relative_to(REPO)),
        "n_actor": base.n_actor, "n_obs": base.n_obs,
        "fsm_version": base.env_cfg.binary_phase_actor_fsm_version,
        "method": "Real OpenSim, fixed zero actions; saved pre-fix step versus current step. Hard path uses actual measured contact with test-only lower limits, not synthetic penetration.",
        "production_thresholds_m": {"soft": base.env_cfg.grf_penetration_penalty_threshold_m,
                                    "ordinary": base.env_cfg.grf_penetration_termination_m,
                                    "hard": production_hard_limit},
    }
    try:
        nominal = []
        for arm_index, method in enumerate((before_step, original_step)):
            if arm_index:
                env.close()
                env = env_factory.make_cmc_env(dict(config.env_config))
                base = env.unwrapped
            adapter.CMCLikeProsthesisTrajectoryEnv.step = method
            env.reset(seed=123)
            rows = []
            for _ in range(5):
                obs, reward, terminated, truncated, info = env.step(action)
                assert not terminated and not truncated
                terms = {key: value for key, value in info["reward_terms"].items()
                         if key not in ("segment_completed", "grf_hard_contact_available")}
                rows.append({"obs": obs.tolist(), "reward": reward, "terms": terms,
                             "phase_fsm": copy.deepcopy(info["phase_fsm"]),
                             "contact": copy.deepcopy(base._online_grf)})
                if method is original_step:
                    assert info["segment_completed"] is True
                    assert "terminal_contact" not in info
            nominal.append(rows)
        if nominal[0] != nominal[1]:
            (HERE / "nominal_mismatch.json").write_text(json.dumps(nominal, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        assert nominal[0] == nominal[1], "Normal dynamics/reward changed in before/after smoke"
        results["nominal_before_after"] = {
            "steps_per_arm": 5, "exact_equality": True,
            "checked": ["observation", "reward", "reward_terms_existing", "phase_fsm", "primary_contact"],
            "sha256": hashlib.sha256(json.dumps(nominal[1], sort_keys=True).encode()).hexdigest(),
        }
        print("Nominal before/after: 5 steps exactly equal", flush=True)

        terminals = []
        for weight in (1.0, 2.0):
            env.close()
            env = env_factory.make_cmc_env(dict(config.env_config))
            base = env.unwrapped
            env.reward_config = replace(production_reward_config, grf_penetration_weight=weight)
            base.env_cfg = production_env_config
            base.runner._cfg.online_grf_max_penetration_m = production_hard_limit
            env.reset(seed=123)
            for _ in range(2):
                _, _, terminated, truncated, info = env.step(action)
                assert not terminated and not truncated
            boundary, completed = base.t, base._step_index
            # These local limits force a reachable REAL contact through the
            # exact production guard. Positive shaping is checked at both weights.
            base.env_cfg = replace(production_env_config, grf_penetration_penalty_threshold_m=0.0)
            primary_sample = base.runner._sample_online_grf
            guard_after = boundary

            def delayed_guard(state, t):
                # Let the actual integrator advance first, so the measured
                # terminal time must differ from the last complete boundary.
                base.runner._cfg.online_grf_max_penetration_m = (
                    1e-6 if t > guard_after + 1e-10 else production_hard_limit
                )
                return primary_sample(state, t)

            base.runner._sample_online_grf = delayed_guard
            for probe_step in range(100):
                boundary, completed = base.t, base._step_index
                obs, reward, terminated, truncated, info = env.step(action)
                if terminated or truncated:
                    break
            base.runner._sample_online_grf = primary_sample
            assert terminated and not truncated
            assert info["end_reason"] == "grf_penetration_hard"
            assert info["segment_completed"] is False
            assert base.t == boundary and base._step_index == completed
            contact = info["terminal_contact"]
            terms, components = info["reward_terms"], info["reward_components"]
            assert contact["available"] is True
            assert contact["time_s"] == base.runner.state.getTime()
            assert contact["time_s"] > boundary + 1e-10
            assert contact["penetration_m"] > contact["limit_m"]
            assert contact["penetration_m"] == contact["online_grf"][contact["side"]]["penetration"]
            assert contact["penetration_m"] == terms["grf_penetration_m"]
            assert terms["grf_penetration_loss"] > 0
            assert terms["grf_ankle_moment_flip_available"] == 1
            assert components["safety_term"] == production_reward_config.safety_weight * terms["safety_loss"]
            assert components["grf_penetration_term"] == weight * terms["grf_penetration_loss"]
            terminals.append({"weight": weight, "reward": reward,
                              "probe_steps": probe_step + 1,
                              "logical_time_s": info["time"],
                              "runner_time_s": base.runner.state.getTime(),
                              "completed_segments": completed,
                              "terminal_contact": contact,
                              "terms": terms, "components": components})
            print(f"Hard guard weight={weight}: actual={contact['penetration_m']:.9f}m, loss={terms['grf_penetration_loss']:.9f}", flush=True)
        assert terminals[0]["terminal_contact"] == terminals[1]["terminal_contact"]
        assert terminals[0]["terms"] == terminals[1]["terms"]
        difference = terminals[0]["reward"] - terminals[1]["reward"]
        expected = terminals[0]["terms"]["grf_penetration_loss"]
        assert abs(difference - expected) < 1e-12
        results["hard_weights"] = terminals
        results["weight_difference"] = {"measured": difference, "expected": expected, "pass": True}

        # A fresh episode must carry no terminal sample from the failed one.
        base.env_cfg = production_env_config
        base.runner._cfg.online_grf_max_penetration_m = production_hard_limit
        env.reward_config = production_reward_config
        _, reset_info = env.reset(seed=123)
        assert "terminal_contact" not in reset_info
        for _ in range(3):
            _, _, terminated, truncated, info = env.step(action)
            assert not terminated and not truncated
            assert info["segment_completed"] and "terminal_contact" not in info
            assert info["reward_terms"]["grf_hard_contact_available"] == 0
        results["reset_after_hard"] = {"steps": 3, "survives": True, "no_stale_contact": True}
    finally:
        adapter.CMCLikeProsthesisTrajectoryEnv.step = original_step
        base.env_cfg = production_env_config
        base.runner._cfg.online_grf_max_penetration_m = production_hard_limit
        env.reward_config = production_reward_config
        env.close()
    results["elapsed_s"] = time.monotonic() - started
    results["status"] = "PASS"
    (HERE / "live_smoke.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: results[key] for key in ("status", "elapsed_s", "weight_difference", "reset_after_hard")}, indent=2))


if __name__ == "__main__":
    main()
