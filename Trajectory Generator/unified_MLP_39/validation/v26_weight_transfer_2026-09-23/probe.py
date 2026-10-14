"""Isolated, inference-only V26 -> unified39 RIGHT diagnostic; never train.

The ordinary left-target checkpoint/rollout validators remain unchanged.
This export has its own explicit diagnostic contract and is intentionally not
loadable as a production left-target unified checkpoint.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
sys.path.insert(0, str(TG))
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(key, "1")

SOURCE_RUN = TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
SOURCE = SOURCE_RUN / "rl_module_best"
HISTORICAL = TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout"
CURRENT = PACKAGE / "runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
EXPORT = HERE / "converted_module"
ROLLOUT = PACKAGE / "runs/rollout/V26_best87_in_unified39_right_fsmv3_2026-09-23"
RIGHT_FEATURES = ("prescribed_right_shifted_knee_q_scaled", "prescribed_right_shifted_knee_qdot_scaled",
                  "prescribed_right_shifted_ankle_q_scaled", "prescribed_right_shifted_ankle_qdot_scaled")
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
PERM = [0, 1, *range(6, 39), 2, 3, 4, 5]  # new column -> old column


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write(path, value):
    from unified_MLP_39.rollout_eval import _jsonable
    Path(path).write_text(json.dumps(_jsonable(value), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stack():
    from unified_MLP_39 import rollout_eval as ev
    ev._load_inference_stack()
    ev.torch.set_num_threads(1)
    return ev


def feature_contract():
    old = read(HISTORICAL / "rollout_summary.json")["observation_feature_names"]
    actor = read(SOURCE_RUN / "actor_feature_manifest.json")["actor_feature_names"]
    base = read(CURRENT / "rl_module_best/unified_contract.json")
    new = base["observation_feature_names"][:]
    new[35:39] = RIGHT_FEATURES
    assert old[:39] == actor and len(old) == len(new) == 88
    assert new[:35] == [old[i] for i in PERM[:35]] and new[39:] == old[39:]
    return {
        "schema_version": "diagnostic_v26_right_transplant_v1", "diagnostic_only": True,
        "training_allowed": False, "teacher_source": "prescribed_right_phase_template",
        "clock": "sin/cos of prescribed RIGHT GRF phase; NOT shifted knee target phase",
        "clock_phase_offset": 0.0, "phase_shifts": dict(zip(JOINTS, (.465, .452))),
        "target_velocity_scale": 1.0, "prescribed_availability_probability": 1.0,
        "fsm": "v3", "source_checkpoint": str(SOURCE),
        "source_state_sha256": sha(SOURCE / "module_state.pkl"),
        "source_actor_feature_names": actor, "actor_feature_names": new[:39],
        "observation_feature_names": new, "new_to_old_actor_permutation": PERM,
        "critic": "absent in source inference export; not transferred or used",
        "action_low": [-1., -1.], "action_high": [1., 1.],
    }


def prepare():
    ev = stack()
    np, torch = ev.np, ev.torch
    from unified_MLP_39.asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
    contract = feature_contract()
    source = ev.RLModule.from_checkpoint(SOURCE)
    config = dict(source.model_config, n_actor=39, n_full=88, freeze_actor=False, freeze_logstd=False)
    converted = AsymmetricActorCriticTorchRLModule(
        observation_space=source.observation_space, action_space=source.action_space,
        inference_only=True, model_config=config)
    old = source.state_dict()
    assert all(k.startswith(("pi.", "pi_encoder.")) for k in old), "Expected actor-only export"
    state = converted.state_dict()
    assert set(state) - set(old) == {"_unified_training_phase", "_unified_freeze_logstd"}
    for key, value in old.items():
        state[key] = value[:, PERM].clone() if key in ("pi_encoder.0.weight", "pi.0.0.weight") else value.clone()
    converted.load_state_dict(state, strict=True)
    assert converted.get_training_phase_state()["phase"] == "imitation"
    source.eval()
    converted.eval()
    trace = read(HISTORICAL / "rollout_policy_trace.json")
    x = np.array([[r["actor_observation_before"][k] for k in contract["source_actor_feature_names"]] for r in trace], dtype=np.float32)
    rng = np.random.default_rng(123)
    synthetic = rng.normal(size=(4096, 39)).astype(np.float32) * np.maximum(np.std(x, axis=0), .1) + np.mean(x, axis=0)
    audit = {"source_state_sha256": contract["source_state_sha256"], "tests": {}}
    for name, inputs in (("historical_500_observations", x), ("synthetic_4096_observations", synthetic)):
        with torch.no_grad():
            a = source.forward_inference({"obs": torch.from_numpy(inputs)})["action_dist_inputs"].numpy()
            b = converted.forward_inference({"obs": torch.from_numpy(inputs[:, PERM].copy())})["action_dist_inputs"].numpy()
        np.testing.assert_allclose(a, b, atol=5e-6, rtol=5e-6)
        audit["tests"][name] = {"samples": len(inputs), "mean_max_abs_error": float(abs(a[:, :2]-b[:, :2]).max()),
                                  "logstd_max_abs_error": float(abs(a[:, 2:]-b[:, 2:]).max())}
    recorded = np.asarray([r["raw_policy_action"] for r in trace])
    with torch.no_grad():
        replay = source.forward_inference({"obs": torch.from_numpy(x)})["action_dist_inputs"].numpy()[:, :2]
        wrong = converted.forward_inference({"obs": torch.from_numpy(x)})["action_dist_inputs"].numpy()[:, :2]
    np.testing.assert_allclose(replay, recorded, atol=5e-6, rtol=5e-6)
    audit["historical_action_replay_max_abs_error"] = float(abs(replay-recorded).max())
    audit["negative_control_without_observation_permutation_max_error"] = float(abs(wrong-replay).max())
    assert audit["negative_control_without_observation_permutation_max_error"] > .01
    if EXPORT.exists():
        raise FileExistsError(f"Refusing to overwrite diagnostic export: {EXPORT}")
    converted.save_to_path(EXPORT)
    write(EXPORT / "diagnostic_contract.json", contract)
    audit["export_state_sha256"] = sha(EXPORT / "module_state.pkl")
    restored = ev.RLModule.from_checkpoint(EXPORT)
    with torch.no_grad():
        out = restored.forward_inference({"obs": torch.from_numpy(x[:, PERM].copy())})["action_dist_inputs"].numpy()[:, :2]
    np.testing.assert_allclose(out, recorded, atol=5e-6, rtol=5e-6)
    audit["export_reload_action_max_abs_error"] = float(abs(out-recorded).max())
    assert sha(SOURCE / "module_state.pkl") == contract["source_state_sha256"]
    write(HERE / "offline_equivalence.json", audit)
    print(json.dumps(audit, indent=2), flush=True)


def right_env_class():
    from unified_MLP_39.env import UnifiedProsthesisEnv
    from unified_MLP_39.target_availability import extend_observation
    from osim_trj_cmc_like import CMCLikeProsthesisTrajectoryEnv as Base, GaitPhaseClock, PhaseBasedImitationTarget

    class RightDiagnosticEnv(UnifiedProsthesisEnv):
        @property
        def actor_feature_names(self):
            return super().actor_feature_names[:35] + RIGHT_FEATURES

        @property
        def observation_feature_names(self):
            names = super().observation_feature_names
            return names[:35] + RIGHT_FEATURES + names[39:]

        def _build_imitation_error_ranges(self):
            self._right_clock = GaitPhaseClock(self._load_sound_heel_strikes("right"), phase_offset=0.)
            self._right_target = PhaseBasedImitationTarget(
                self.base_kin, self._right_clock, self.env_cfg.imitation_sound_coords,
                dict(zip(JOINTS, (.465, .452))), fallback_phase_shift=.5, phase_samples=200,
                time_window=(self.cfg.t_start, self.cfg.t_end))
            assert self._right_target.available
            return Base._build_imitation_error_ranges(self)

        def imitation_target(self, t):
            return self._right_target.get(t)

        def _imitation_target_summary(self):
            return dict(self._right_target.summary(), source="prescribed_right_phase_template",
                        position_ranges_rad=self._imitation_position_ranges_rad,
                        velocity_ranges_rad_s=self._imitation_velocity_ranges_rad_s)

        def _gait_clock_summary(self):
            return {"available": True, "side": "right", "phase_offset": 0., "phase": self._right_clock.phase(self.t)}

        def _imitation_target_payload(self):
            return dict(Base._imitation_target_payload(self), source="prescribed_right_phase_template")

        def _get_observation(self):
            base, _ = Base._get_observation(self)
            assert self._n_actor == 35
            q, qd, _ = self.imitation_target(self.t)
            obs = extend_observation(base, [q[JOINTS[0]], qd[JOINTS[0]], q[JOINTS[1]], qd[JOINTS[1]]],
                                     self._target_availability.visible, phase=self._right_clock.phase(self.t),
                                     target_velocity_scale=1.)
            return obs, dict(zip(self.observation_feature_names, map(float, obs)))
    return RightDiagnosticEnv


def make_env(folder):
    import yaml
    from unified_MLP_39 import env_factory
    cfg = yaml.safe_load((CURRENT / "training_cfg.resolved.yaml").read_text())
    raw = dict(cfg["simulation"], **cfg["grf"], **cfg["target_availability"])
    renames = {"setup_xml": "setup_xml_path", "pros_ref_governor": "enable_pros_ref_governor",
               "pros_ref_cutoff_hz": "pros_ref_lpf_cutoff_hz", "online_grf_profile": "online_grf_profile_file",
               "online_grf_detector_profile": "online_grf_detector_profile_file",
               "binary_phase_detector_profile": "binary_phase_detector_profile_file",
               "online_grf_observation": "include_online_grf_observation",
               "online_grf_applied_side": "online_grf_applied_sides",
               "disable_prescribed_grf_side": "prescribed_grf_disabled_sides"}
    for old, new in renames.items():
        raw[new] = raw.pop(old)
    for label, field in (("target_slew_rate_limit_rad_s", "target_slew_rate_limit_rad_s"),
                         ("ref_velocity_limit_rad_s", "pros_ref_velocity_limit_rad_s"),
                         ("ref_acceleration_limit_rad_s2", "pros_ref_acceleration_limit_rad_s2"),
                         ("ref_jerk_limit_rad_s3", "pros_ref_jerk_limit_rad_s3")):
        raw[field] = {j: raw.pop(f"pros_{short}_{label}") for j, short in zip(JOINTS, ("knee", "ankle"))}
    raw.update(critic_privileged_observation=True, reward=cfg["reward"], record_outputs=True,
               save_outputs_on_close=True, fail_fast=True, output_dir=str(folder / "sim_outputs"),
               output_prefix="rollout_episode")
    assert raw["binary_phase_actor_fsm_version"] == "v3"
    assert raw["target_availability_probability"] == raw["target_velocity_scale"] == 1.
    original = env_factory.UnifiedProsthesisEnv
    try:
        # Only select an explicitly named diagnostic subclass during construction;
        # all factory validation and normal wrapping still run unchanged.
        env_factory.UnifiedProsthesisEnv = right_env_class()
        env = env_factory.make_cmc_env(raw)
    finally:
        env_factory.UnifiedProsthesisEnv = original
    write(folder / "diagnostic_env_config.json", raw)
    write(folder / "effective_cmc_env_config.json", vars(env.unwrapped.env_cfg))
    (folder / "training_cfg.resolved.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    return env, raw


def rollout():
    ev = stack()
    np, torch = ev.np, ev.torch
    from baseline_MLP import process_watchdog as watchdog
    ROLLOUT.mkdir(parents=True, exist_ok=True)
    if (ROLLOUT / "rollout_summary.json").exists():
        raise FileExistsError("Refusing to overwrite an evaluated rollout")
    heartbeat = ROLLOUT / "watchdog_state.json"
    watchdog.write_heartbeat(heartbeat, "load diagnostic export and build environment", timeout_s=600)
    contract = read(EXPORT / "diagnostic_contract.json")
    assert contract == feature_contract()
    audit = read(HERE / "offline_equivalence.json")
    assert sha(EXPORT / "module_state.pkl") == audit["export_state_sha256"]
    module = ev.RLModule.from_checkpoint(EXPORT)
    original_module = ev.RLModule.from_checkpoint(SOURCE)
    module.eval()
    original_module.eval()
    torch.manual_seed(123)
    np.random.seed(123)
    env, raw_cfg = make_env(ROLLOUT)
    base = env.unwrapped
    assert list(base.actor_feature_names) == contract["actor_feature_names"]
    assert list(base.observation_feature_names) == contract["observation_feature_names"]
    ev._validate_module_observation_contract(module, base.actor_feature_names, base.observation_feature_names)
    np.testing.assert_array_equal(env.action_space.low, contract["action_low"])
    np.testing.assert_array_equal(env.action_space.high, contract["action_high"])
    historical = read(HISTORICAL / "rollout_policy_trace.json")
    target_error = clock_error = 0.
    for row in historical:
        q, qd, _ = base.imitation_target(row["time"])
        for j in JOINTS:
            for actual, expected in ((q[j], row["imitation_target_q"][j]), (qd[j], row["imitation_target_qdot"][j])):
                target_error = max(target_error, abs(actual-expected))
                np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=0)
        t_before = row["time"] - .01
        phase = base._right_clock.phase(t_before)
        expected = [row["actor_observation_before"][k] for k in ("gait_phase_sin", "gait_phase_cos")]
        actual = [np.sin(2*np.pi*phase), np.cos(2*np.pi*phase)]
        clock_error = max(clock_error, float(np.max(np.abs(np.array(actual)-expected))))
        np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=0)
    validation = {"historical_target_max_abs_error": target_error, "historical_clock_max_abs_error": clock_error,
                  "fsm": "v3", "source_immutable": True, "live_policy_logit_max_abs_error": 0.}
    write(ROLLOUT / "runtime_validation.json", validation)
    trace, rewards = [], []
    terminated = truncated = False
    info = {}
    try:
        obs, reset_info = env.reset(seed=123)
        write(ROLLOUT / "rollout_reset_diagnostics.json", reset_info.get("reset_diagnostics", {}))
        first = np.asarray(obs[:39])
        old_first = np.array([historical[0]["actor_observation_before"][k] for k in contract["source_actor_feature_names"]])[PERM]
        validation["initial_actor_observation_differences"] = {
            k: {"current": float(a), "historical": float(b), "delta": float(a-b)}
            for k, a, b in zip(contract["actor_feature_names"], first, old_first) if abs(a-b) > 1e-6}
        for step in range(1, 501):
            watchdog.write_heartbeat(heartbeat, f"simulator step {step}", progress=step-1, timeout_s=180)
            before = np.asarray(obs, dtype=np.float32).copy()
            assert base._target_availability.visible
            # Live numerical gate: historical network on inverse-permuted CURRENT
            # observations must agree with the converted network at every step.
            old_obs = before.copy()
            old_obs[np.asarray(PERM)] = before[:39]
            with torch.no_grad():
                a = original_module.forward_inference({"obs": torch.from_numpy(old_obs[None])})["action_dist_inputs"].numpy()
                b = module.forward_inference({"obs": torch.from_numpy(before[None])})["action_dist_inputs"].numpy()
            np.testing.assert_allclose(a, b, atol=5e-6, rtol=5e-6)
            validation["live_policy_logit_max_abs_error"] = max(validation["live_policy_logit_max_abs_error"], float(abs(a-b).max()))
            action, mean, std, noise = ev._policy_action_with_diagnostics(module, before, env.action_space.shape)
            obs, reward, terminated, truncated, info = env.step(action)
            rewards.append(float(reward))
            target, observation = info["imitation_target"], info["observation"]
            row = {"step": step, "time": float(info["time"]), "raw_policy_action": action,
                   "policy_action_mean": mean, "policy_action_std": std, "exploration_action_noise": noise,
                   "applied_policy_action": np.clip(action, env.action_space.low, env.action_space.high),
                   "actor_observation_before": dict(zip(base.actor_feature_names, map(float, before[:39]))),
                   "actor_observation_vector_before": before[:39], "full_observation_vector_before": before,
                   "imitation_target_q": target["q"], "imitation_target_qdot": target["qdot"],
                   "prosthetic_state": {k: float(observation[k]) for k in observation
                                          if any(k == j or k.startswith(j + "_") for j in JOINTS)},
                   "reward_terms": info["reward_terms"]}
            for key, default in (("target_availability", {}), ("policy_segment_times", []), ("policy_segment_values", []),
                                 ("policy_segment_derivatives", []), ("reference_governor_diagnostics", {}),
                                 ("phase_fsm", {}), ("legacy_online_events", []), ("online_events", []),
                                 ("morphology_completed_segments", []), ("morphology_ledger_diagnostics", {})):
                row[key] = info.get(key, default)
            row["detector_sensors"] = info.get("online_grf_detector", {}).get("sensors", {})
            trace.append(row)
            if step % 25 == 0 or terminated or truncated:
                write(ROLLOUT / "progress.json", {"steps": step, "return": sum(rewards), "time": info["time"],
                                                  "end_reason": info.get("end_reason")})
                print(f"Step {step}/500; return={sum(rewards):.6f}", flush=True)
            if terminated or truncated:
                break
    finally:
        env.close()
        write(ROLLOUT / "rollout_policy_trace.json", trace)
        write(ROLLOUT / "runtime_validation.json", validation)
    assert sha(SOURCE / "module_state.pkl") == contract["source_state_sha256"]
    summary = {"ok": True, "checkpoint": str(EXPORT), "action_mode": "absolute", "action_selection": "deterministic",
               "action_seed": 123, "setup_xml_path": raw_cfg["setup_xml_path"], "episode_start_offset_s": 1.,
               "steps": len(trace), "episode_return": sum(rewards), "end_reason": info.get("end_reason"),
               "terminated": bool(terminated), "truncated": bool(truncated), "n_actor": 39, "n_observation": 88,
               "actor_feature_names": list(base.actor_feature_names), "observation_feature_names": list(base.observation_feature_names),
               "action_shape": [2], "record_outputs": True, "record_policy_trace": True,
               "reward_config": ev.reward_function.RewardConfig.from_mapping(raw_cfg["reward"]).to_dict(),
               "grf_penetration_max_m": max(r["reward_terms"]["grf_penetration_m"] for r in trace),
               "diagnostic_intervention": contract}
    for key in ("phase_valid_hs_count", "phase_valid_to_count", "phase_valid_cycle_count", "invalid_event_count"):
        summary[key] = max(int(r["reward_terms"].get(key, 0) or 0) for r in trace)
    write(ROLLOUT / "rollout_summary.json", summary)
    watchdog.write_heartbeat(heartbeat, "complete", progress=len(trace))
    print(json.dumps({k: summary[k] for k in ("steps", "episode_return", "end_reason", "grf_penetration_max_m")}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "rollout"))
    parser.add_argument("--worker", action="store_true")
    args = parser.parse_args()
    if args.command == "prepare":
        prepare()
    elif args.worker:
        rollout()
    else:
        from baseline_MLP import process_watchdog
        ROLLOUT.mkdir(parents=True, exist_ok=True)
        result = process_watchdog.supervise_process(
            [sys.executable, str(Path(__file__).resolve()), "rollout", "--worker"],
            heartbeat_file=ROLLOUT / "watchdog_state.json", summary_file=ROLLOUT / "watchdog_summary.json",
            startup_timeout_s=600., stall_timeout_s=240., run_timeout_s=2400., label="V26 weights in unified39 RIGHT")
        raise SystemExit(result["returncode"] if result["timeout_reason"] is None else 124)


if __name__ == "__main__":
    main()
