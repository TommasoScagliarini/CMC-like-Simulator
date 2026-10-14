"""Frozen V26 best47: coherent random prosthetic reset, biological IK unchanged.

--prepare is stdlib-only. --run and --bootstrap-probe execute isolated native
workers after review. No production source, plugin or checkpoint is edited.
Random candidates are retained in order; static rejections are not replaced.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import random
import sys
import time
import traceback
from dataclasses import asdict
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
PREVIOUS = REPO / "Trajectory Generator/validation/reset_ik_audit_2026_10_12/frozen_reset_tests.py"
EVALUATOR = REPO / "Trajectory Generator/baseline_MLP/rollout_eval.py"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
DOMAIN = {JOINTS[0]: (-1.20, -0.03), JOINTS[1]: (-0.55, 0.55)}


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def common():
    return import_file("preserved_reset_audit_helpers", PREVIOUS)


class CoherentBootstrapKinematics:
    """Seed the existing governor and preserve its original policy dynamics.

    Before the first policy segment, only prosthetic reference values become
    q=random, v=a=0. Biological values always come from the original base IK.
    Once segments arrive, the original set_segment and served spline are used.
    An out-of-segment query fails closed, never returns prosthetic IK fallback.
    """
    def __init__(self, delegate, pose, numpy):
        self.delegate = delegate
        self.pose = tuple(float(pose[name]) for name in JOINTS)
        self.np = numpy
        self.set_segment_count = 0
        self.first_segment = None
        self.clear_segment()

    def __getattr__(self, name):
        return getattr(self.delegate, name)

    def clear_segment(self):
        self.delegate.clear_segment()
        self.delegate._filt_q = self.np.asarray(self.pose, dtype=float).copy()
        self.delegate._filt_v = self.np.zeros(2, dtype=float)
        self.delegate._filt_a = self.np.zeros(2, dtype=float)
        self.delegate._last_anchor = self.np.asarray(self.pose, dtype=float).copy()
        self.set_segment_count = 0
        self.first_segment = None

    def get(self, t):
        biological = tuple(dict(values) for values in self.delegate._base.get(t))
        if self.delegate._segment_spline is None:
            prosthetic = (self.pose, (0.0, 0.0), (0.0, 0.0))
        else:
            t0, t1 = self.delegate._segment_t0, self.delegate._segment_t1
            if t < t0 - 1e-10 or t > t1 + 1e-10:
                raise RuntimeError(f"No policy segment covers reference time {t}; prosthetic IK fallback prohibited")
            served_time = min(max(float(t), float(t0)), float(t1))
            spline = self.delegate._segment_spline
            prosthetic = (spline(served_time), spline.derivative(1)(served_time), spline.derivative(2)(served_time))
        for order, values in enumerate(prosthetic):
            for index, name in enumerate(JOINTS):
                biological[order][name] = float(values[index])
        return biological

    def current_target(self, t):
        return self.np.asarray([self.get(t)[0][name] for name in JOINTS], dtype=float)

    def set_segment(self, times, values, derivatives=None):
        before = {"q": self.delegate._filt_q.tolist(), "v": self.delegate._filt_v.tolist(), "a": self.delegate._filt_a.tolist()}
        self.delegate.set_segment(times, values, derivatives)
        self.set_segment_count += 1
        if self.first_segment is None:
            q, v, a = self.get(float(times[0]))
            self.first_segment = {"raw_anchor_q": self.np.asarray(values[0]).tolist(),
                                  "raw_anchor_v": self.np.asarray(derivatives[0]).tolist() if derivatives is not None else None,
                                  "filter_state_before": before,
                                  "served_initial_q": {name: q[name] for name in JOINTS},
                                  "served_initial_v": {name: v[name] for name in JOINTS},
                                  "served_initial_a": {name: a[name] for name in JOINTS}}
            if any(abs(q[name] - self.pose[i]) > 1e-10 or abs(v[name]) > 1e-10 or abs(a[name]) > 1e-10 for i, name in enumerate(JOINTS)):
                raise RuntimeError("First served policy segment lost the coherent random initial condition")


def validate_pose(pose):
    if set(pose) != set(JOINTS):
        raise ValueError("Pose must contain exactly the two prosthetic coordinates")
    for name, value in pose.items():
        value = float(value)
        low, high = DOMAIN[name]
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"Pose outside preregistered domain: {name}={value}")


def prepare(count, sampling_seed, duration, candidate_count=400):
    if count < 1 or candidate_count < count:
        raise ValueError("Candidate count must be positive")
    helper = common()
    plan = helper.make_plan(duration)
    rng = random.Random(sampling_seed)
    plan["cases"] = [dict(id="coherent_nominal", kind="coherent_ik_prosthetic_walking_reset", pose=None)]
    for index in range(candidate_count):
        pose = {name: rng.uniform(*DOMAIN[name]) for name in JOINTS}
        validate_pose(pose)
        plan["cases"].append(dict(id=f"random_{index + 1:03d}", kind="coherent_random_prosthetic_walking_reset", pose=pose))
    plan["episode_start_offset_s"] = helper.NOMINAL_OFFSET
    plan["random_pose_sampling"] = {"distribution": "independent uniform knee/ankle rectangle", "seed": sampling_seed,
                                    "candidate_count": candidate_count, "selected_random_count": count, "domain_rad": DOMAIN,
                                    "selection": "First eligible candidates in fixed order; all preflight candidates and rejection reasons preserved",
                                    "rejections": "No candidate redraws, adaptive bounds or outcome-based selection"}
    plan["static_gate"] = {"max_left_primary_penetration_m": 0.027, "force_norm_guard_bw": 5.0, "air_clearance_cap": None}
    plan["strict_screening"] = {"duration_s": 5.0, "minimum_valid_cycles": 2, "first_valid_hs_deadline_s": 3.0,
                                "maximum_invalid_events": 0, "after_050s_actual_served_rmse_deg_per_coordinate": 2.0,
                                "scientific_qualification": "These operational criteria do not certify general biomechanics or subject transfer"}
    plan["reference_contract"].update(random_bootstrap="q random; joint/rotor v0; rotor angle=q; controller PI0; governor q=random,v=a=0; biological IK/time and both GRF paths preserved; no target or fake HS initialization")
    for path in (Path(__file__), AUDIT / "bootstrap_probe.py", AUDIT / "test_bootstrap.py"):
        if path.exists():
            plan["protected_sha256"][helper.relative(path)] = helper.sha256(path)
    return plan


def static_contact(base, evaluator):
    from online_grf import read_online_grf
    ctx, state = base.ctx, base.runner.state
    primary = read_online_grf(ctx.model, state, ctx.online_grf_force_paths, ctx.online_grf_force_sides)
    applied = list(base.cfg.online_grf_applied_sides)
    forces = [evaluator.np.asarray(primary["sides"][side]["force"], dtype=float) for side in applied]
    peak_force = max([float(evaluator.np.linalg.norm(force)) for force in forces] + [float(evaluator.np.linalg.norm(evaluator.np.sum(forces, axis=0)))])
    force_limit = float(base.cfg.online_grf_max_force_bw) * float(ctx.model.getTotalMass(state)) * 9.80665
    penetration = max(float(primary["sides"][side]["penetration"]) for side in applied)
    sv = ctx.model.getStateVariableValues(state)
    pelvis_height = float(sv.get(ctx.q_sv_idx["pelvis_ty"]))
    reasons = []
    if not math.isfinite(peak_force) or not math.isfinite(penetration):
        reasons.append("nonfinite_primary_contact")
    if peak_force > force_limit:
        reasons.append("existing_native_force_guard_exceeded_at_reset")
    if penetration > float(base.cfg.online_grf_max_penetration_m):
        reasons.append("existing_native_hard_penetration_guard_exceeded_at_reset")
    if penetration > 0.027:
        reasons.append("preregistered_static_penetration_margin_27mm_exceeded")
    if pelvis_height < float(base.env_cfg.pelvis_min_height):
        reasons.append("existing_pelvis_height_guard_exceeded_at_reset")
    return {"primary_grf": evaluator._jsonable(primary), "binary_detector_baseline": evaluator._jsonable(base.runner.last_step_info.get("binary_phase_sensor_baseline")),
            "peak_applied_force_norm_n": peak_force, "force_guard_n": force_limit,
            "max_applied_penetration_m": penetration,
            "static_penetration_gate_m": 0.027, "ordinary_penetration_guard_m": float(base.env_cfg.grf_penetration_termination_m),
            "native_hard_penetration_guard_m": float(base.cfg.online_grf_max_penetration_m),
            "pelvis_height_m": pelvis_height, "eligible_to_integrate": not reasons, "rejection_reasons": reasons,
            "note": "Static geometry/contact eligibility only; no stability or scientific qualification claim."}


class StaticResetRejected(RuntimeError):
    pass


def exception_category(exc):
    text = str(exc).lower()
    if isinstance(exc, FloatingPointError) and "online grf exceeds" in text:
        return "physical_force_guard_censored"
    if type(exc).__name__ == "SegmentWallClockTimeout":
        return "segment_wall_clock_timeout_censored"
    if type(exc).__name__ == "OnlineGRFPenetrationLimitExceeded":
        return "physical_guard_censored"
    if isinstance(exc, FloatingPointError):
        return "numerical_exception_censored"
    return "harness_or_native_failure"


def preserve_censored_summary(output, receipt, rows, evaluator, args, exc):
    """Preserve only actual completed transitions; never invent terminal reward."""
    helper = common()
    if (output / "rollout_summary.json").exists():
        return
    base = receipt.get("initial_state", {})
    phase = rows[-1].get("phase_fsm", {}) if rows else {}
    summary = {"ok": True, "summary_kind": "harness_censored_last_completed_transitions",
               "checkpoint": args.checkpoint, "setup_xml_path": args.setup_xml,
               "episode_start_offset_s": args.episode_start_offset_s,
               "action_mode": args.action_mode, "action_selection": "deterministic", "action_shape": [1, 2],
               "steps": len(rows), "episode_return": math.fsum(row["returned_reward"] for row in rows),
               "terminated": False, "truncated": True, "end_reason": exception_category(exc),
               "failure_exception": f"{type(exc).__name__}: {exc}",
               "last_completed_transition_time_s": rows[-1]["time"] if rows else base.get("time_s"),
               "reward_config": receipt.get("effective_env_config", {}).get("reward", getattr(args, "_cfg_reward", {})),
               "phase_valid_hs_count": int(phase.get("valid_hs_count", 0)),
               "phase_valid_to_count": int(phase.get("valid_to_count", 0)),
               "phase_valid_cycle_count": int(phase.get("valid_cycle_count", 0)),
               "invalid_event_count": int(phase.get("invalid_event_count", 0)),
               "actor_feature_names": receipt.get("actor_feature_names", []),
               "observation_feature_names": receipt.get("observation_feature_names", []),
               "warning": "Interrupted transition excluded: no returned reward or completed sensor batch exists. Physical state after the exception is not a completed policy boundary."}
    helper.write_json(output / "rollout_summary.json", summary)
    helper.write_json(output / "rollout_policy_trace.json", rows)
    receipt["censored_summary_created"] = True


def install_bootstrap(env, case, receipt, evaluator):
    base = env.unwrapped
    adapter = None
    resolved_pose = case["pose"]
    if case["kind"] == "coherent_ik_prosthetic_walking_reset":
        q0, _, _ = base.base_kin.get(base.cfg.t_start + base.env_cfg.episode_start_offset_s)
        resolved_pose = {name: float(q0[name]) for name in JOINTS}
    if resolved_pose is not None:
        validate_pose(resolved_pose)
        if base.env_cfg.imitation_initialize_to_target or base.env_cfg.rebuild_model_on_reset:
            raise RuntimeError("This diagnostic requires original ex-novo reset without rebuilding or target initialization")
        if base.kin._ref_model != "butterworth3_jerk_limited" or not base.kin._ref_lpf_enable:
            raise RuntimeError("Frozen governor contract changed")
        adapter = CoherentBootstrapKinematics(base.kin, resolved_pose, evaluator.np)
        base.kin = adapter
        base.runner._kin = adapter
    original_reset = env.reset

    def reset(*args, **kwargs):
        observation, info = original_reset(*args, **kwargs)
        ctx, state = base.ctx, base.runner.state
        values = ctx.model.getStateVariableValues(state)
        q = {name: float(values.get(index)) for name, index in ctx.q_sv_idx.items()}
        v = {name: float(values.get(index)) for name, index in ctx.qdot_sv_idx.items()}
        iq, iv, ia = base.base_kin.get(base.t)
        served_q, served_v, served_a = base.kin.get(base.t)
        initial = {"time_s": float(base.t), "actual_q": q, "actual_qdot": v,
                   "biological_ik_q": {name: value for name, value in iq.items() if name not in JOINTS},
                   "biological_ik_qdot": {name: value for name, value in iv.items() if name not in JOINTS},
                   "original_prosthetic_ik": {name: {"q": iq[name], "v": iv[name], "a": ia[name]} for name in JOINTS},
                   "served_prosthetic_reference": {name: {"q": served_q[name], "v": served_v[name], "a": served_a[name]} for name in JOINTS},
                   "previous_endpoint": base._last_policy_endpoint.tolist(),
                   "outer_pi_integral": dict(base.runner._prosthesis_ctrl._integral_error),
                   "cascade_pi_integral": dict(base.runner._prosthesis_ctrl._cascade_integral_error),
                   "sea_motor_angles": {name: float(values.get(index)) for name, index in ctx.sea_motor_angle_sv_idx.items()},
                   "sea_motor_speeds": {name: float(values.get(index)) for name, index in ctx.sea_motor_speed_sv_idx.items()},
                   "sea_auxiliary_states": {sea: {name: float(values.get(index)) for name, index in state_indices.items() if name not in ("motor_angle", "motor_speed")}
                                            for sea, state_indices in ctx.sea_state_sv_idx.items()},
                   "full_state": common().capture_state(base), "reset_info": evaluator._jsonable(info),
                   "observation": evaluator._jsonable(observation)}
        receipt["initial_state"] = evaluator._jsonable(initial)
        receipt["resolved_prosthetic_pose"] = resolved_pose
        # Preserve evidence before any bootstrap assertion or contact gate.
        common().write_json(Path(receipt["output_dir"]) / "receipt.json", receipt)
        expected_time = float(base.cfg.t_start) + float(base.env_cfg.episode_start_offset_s)
        if abs(float(base.t) - expected_time) > 1e-9:
            raise RuntimeError(f"Reset time was clamped/changed: actual={base.t}, expected={expected_time}")
        for name in initial["biological_ik_q"]:
            if abs(q[name] - iq[name]) > 1e-10 or abs(v[name] - iv[name]) > 1e-10:
                raise RuntimeError(f"Biological IK initial state changed: {name}")
        if adapter is not None:
            for index, (name, sea) in enumerate(zip(JOINTS, (base.cfg.sea_knee_name, base.cfg.sea_ankle_name))):
                target = resolved_pose[name]
                checks = [q[name] - target, v[name], initial["sea_motor_angles"][sea] - target,
                          initial["sea_motor_speeds"][sea], served_q[name] - target, served_v[name], served_a[name],
                          base._last_policy_endpoint[index] - target]
                if any(abs(value) > 1e-10 for value in checks):
                    raise RuntimeError(f"Incoherent prosthetic bootstrap: {name}: {checks}")
            if any(value != 0 for value in (*initial["outer_pi_integral"].values(), *initial["cascade_pi_integral"].values())):
                raise RuntimeError("Python controller PI memory did not reset")
            if any(value != 0 for states in initial["sea_auxiliary_states"].values() for value in states.values()):
                raise RuntimeError("SEA auxiliary state did not reset to zero")
        for field in ("valid_hs_count", "valid_to_count", "valid_cycle_count"):
            if float(info["phase_fsm"].get(field, 0)) != 0:
                raise RuntimeError("Reset invented an accepted gait event")
        receipt["static_contact_gate"] = static_contact(base, evaluator)
        common().write_json(Path(receipt["output_dir"]) / "receipt.json", receipt)
        if not receipt["static_contact_gate"]["eligible_to_integrate"]:
            raise StaticResetRejected(";".join(receipt["static_contact_gate"]["rejection_reasons"]))
        return observation, info

    env.reset = reset
    return adapter


def render(output):
    helper = common()
    os.environ.setdefault("MPLBACKEND", "Agg")
    plotter = import_file("random_reset_canonical_plotter", REPO / "plot/plotter.py")
    context = plotter.valid_mlp_rollout_context(output)
    if context is None:
        return {"rendered": False, "reason": "complete_plot_inputs_unavailable"}
    folder = REPO / "plot/random_pose_reset_2026_10_13" / output.name
    folder.mkdir(parents=True, exist_ok=False)
    old_next, old_discovery, old_argv = plotter.next_output_dir, plotter.latest_mlp_rollout_context, sys.argv
    plotter.next_output_dir = lambda root: folder
    plotter.latest_mlp_rollout_context = lambda: context
    summary = helper.read_json(output / "rollout_summary.json")
    sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir), "--prefix", "rollout_episode", "--setup", summary["setup_xml_path"], "--gait-side", "left", "--out-root", str(folder.parent)]
    try:
        result = plotter.main()
    finally:
        plotter.next_output_dir, plotter.latest_mlp_rollout_context, sys.argv = old_next, old_discovery, old_argv
    pngs = sorted(folder.glob("[0-9][0-9]_*.png"))
    receipt = {"rendered": result == 0 and len(pngs) == 7, "figures": {helper.relative(path): helper.sha256(path) for path in pngs}}
    helper.write_json(output / "plot_receipt.json", receipt)
    if not receipt["rendered"]:
        raise RuntimeError("Seven canonical plots incomplete")
    return receipt


def apply_wrapped_action(env, raw_action, wrapper_type):
    """Use the original FlattenClipAction mapping before direct probe planning."""
    wrapper = env
    while not isinstance(wrapper, wrapper_type):
        inner = getattr(wrapper, "env", None)
        if inner is None or inner is wrapper:
            raise RuntimeError("Original FlattenClipAction missing from probe wrapper stack")
        wrapper = inner
    return wrapper.action(raw_action)


def postprocess_saved(output, receipt, helper):
    """Plot/I/O failures never reclassify a completed or censored physical run."""
    try:
        helper.analyze(output)
        receipt["postprocessing"] = render(output)
    except Exception as exc:
        receipt["postprocessing_error"] = f"{type(exc).__name__}: {exc}"
        receipt["postprocessing"] = {"rendered": False, "error_type": type(exc).__name__, "error": str(exc)}


def validated_selection_cases(plan, selection, selection_dir, helper):
    """Reconstruct all eligibility/order from the 401 hashed native receipts."""
    expected_paths = {helper.relative(selection_dir / case["id"] / "receipt.json") for case in plan["cases"]}
    hashes = selection["preflight_receipt_hashes"]
    if set(hashes) != expected_paths:
        raise RuntimeError("Preflight receipt set does not cover the complete frozen candidate pool")
    reconstructed = []
    for case in plan["cases"]:
        path = selection_dir / case["id"] / "receipt.json"
        key = helper.relative(path)
        if helper.sha256(path) != hashes[key]:
            raise RuntimeError("Preflight geometry receipt changed before rollout selection")
        receipt = helper.read_json(path)
        if receipt.get("case") != case:
            raise RuntimeError("Preflight receipt case differs from the frozen candidate")
        status, gate = receipt.get("status"), receipt.get("static_contact_gate")
        if status not in {"static_eligible", "static_rejected"} or not isinstance(gate, dict):
            raise RuntimeError("Preflight receipt has no completed static eligibility verdict")
        if gate.get("eligible_to_integrate") != (status == "static_eligible"):
            raise RuntimeError("Preflight receipt status disagrees with its contact gate")
        reconstructed.append({"id": case["id"], "status": status, "gate": gate})
    if reconstructed != selection["preflight_outcomes"]:
        raise RuntimeError("Selection outcomes differ from the hashed native receipt case/status/gate/order")
    eligible = [item["id"] for item in reconstructed if item["status"] == "static_eligible"]
    expected_ids = ["coherent_nominal"] + [name for name in eligible if name != "coherent_nominal"][:plan["random_pose_sampling"]["selected_random_count"]]
    if "coherent_nominal" not in eligible or len(expected_ids) != plan["random_pose_sampling"]["selected_random_count"] + 1:
        raise RuntimeError("Preflight does not contain the required eligible control and random count")
    declared = {case["id"]: case for case in plan["cases"]}
    expected_cases = [declared[name] for name in expected_ids]
    if selection["cases"] != expected_cases:
        raise RuntimeError("Selection does not follow the frozen first-eligible receipt order")
    return expected_cases


def execute(plan, case, output, probe_only=False):
    helper = common()
    helper.verify_inputs(plan)
    helper.prepare_worker_output(output)
    evaluator = import_file("random_reset_original_evaluator", EVALUATOR)
    receipt = {"schema_version": 1, "instruction_check_token": helper.TOKEN, "case": case,
               "status": "started", "output_dir": str(output), "policy_updates": 0,
               "probe_only": probe_only, "integration_steps": 0, "source_hashes": plan["protected_sha256"]}
    old_argv = sys.argv
    sys.argv = [str(EVALUATOR), "--checkpoint", str(helper.resolve(plan["checkpoint"])), "--setup-xml", str(helper.resolve(plan["setup_xml"])),
                "--episode-duration", str(plan["duration_s"]), "--segment-duration", "0.01", "--policy-knots", "1",
                "--episode-start-offset-s", str(plan["episode_start_offset_s"]), "--max-steps", str(math.ceil(plan["duration_s"] / 0.01)),
                "--action-mode", "absolute", "--action-selection", "deterministic", "--seed", "123", "--record-outputs", "--record-policy-trace", "--no-progress", "--output-dir", str(output)]
    try:
        args = evaluator.parse_args()
    finally:
        sys.argv = old_argv
    original_loader = evaluator._load_inference_stack
    original_factory = None
    environments, adapters = [], []
    valid_rows = []
    closed_environments = set()

    def loader():
        nonlocal original_factory
        original_loader()
        original_factory = evaluator.env_factory.make_cmc_env

        def factory(config=None):
            env = original_factory(config)
            base = env.unwrapped
            expected = helper.relocated_historical_receipt()
            for field in ("actor_feature_names", "observation_feature_names"):
                if list(getattr(base, field)) != expected[field]:
                    raise RuntimeError("Frozen observation names/order changed")
            if not base.cfg.use_muscles_in_so or base.cfg.grf_mode != "online_sensor" or list(base.cfg.online_grf_applied_sides) != ["left"]:
                raise RuntimeError("Biological/GRF support contract changed")
            receipt["effective_env_config"] = evaluator._jsonable(asdict(base.env_cfg))
            receipt["effective_simulator_config"] = evaluator._jsonable(asdict(base.cfg))
            receipt["actor_feature_names"] = list(base.actor_feature_names)
            receipt["observation_feature_names"] = list(base.observation_feature_names)
            environments.append(env)
            adapters.append(install_bootstrap(env, case, receipt, evaluator))
            original_step = env.step
            original_close = env.close

            def close():
                if id(env) not in closed_environments:
                    try:
                        original_close()
                    finally:
                        closed_environments.add(id(env))

            env.close = close

            def step(action):
                before = float(base.t)
                prior = (valid_rows[-1]["next_observation"] if valid_rows else receipt["initial_state"]["observation"])
                result = original_step(action)
                receipt["integration_steps"] += 1
                _, reward, terminated, truncated, info = result
                observation_values = info.get("observation", {})
                target = info.get("imitation_target", {})
                valid_row = {"step": len(valid_rows) + 1, "time": float(info["time"]),
                             "raw_policy_action": evaluator._jsonable(action),
                             "applied_policy_action": evaluator._jsonable(evaluator.np.clip(action, -1, 1)),
                             "actor_observation_before": {name: float(prior[index]) for index, name in enumerate(base.actor_feature_names)},
                             "actor_observation_vector_before": list(prior[:base.n_actor]),
                             "next_observation": evaluator._jsonable(result[0]),
                             "policy_segment_times": evaluator._jsonable(info.get("policy_segment_times", [])),
                             "policy_segment_values": evaluator._jsonable(info.get("policy_segment_values", [])),
                             "policy_segment_derivatives": evaluator._jsonable(info.get("policy_segment_derivatives", [])),
                             "imitation_target_q": target.get("q", {}), "imitation_target_qdot": target.get("qdot", {}),
                             "prosthetic_state": {name: value for name, value in observation_values.items() if name.startswith("pros_")},
                             "phase_fsm": evaluator._jsonable(info.get("phase_fsm", {})),
                             "reference_governor_diagnostics": evaluator._jsonable(info.get("reference_governor_diagnostics", {})),
                             "reward_terms": evaluator._jsonable(info.get("reward_terms", {})), "returned_reward": float(reward)}
                valid_rows.append(evaluator._jsonable(valid_row))
                with (output / "last_valid_trace.jsonl").open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(valid_rows[-1], allow_nan=False) + "\n")
                with (output / "returned_rewards.jsonl").open("a", encoding="utf-8") as stream:
                    row = {"step": receipt["integration_steps"], "time_before_s": before,
                           "time_after_s": float(base.t), "returned_reward": float(reward),
                           "terminated": bool(terminated), "truncated": bool(truncated),
                           "end_reason": info.get("end_reason"), "failure": info.get("failure"),
                           "terminal_contact": evaluator._jsonable(info.get("terminal_contact"))}
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
                return result

            env.step = step
            return env

        evaluator.env_factory.make_cmc_env = factory

    evaluator._load_inference_stack = loader
    started = time.monotonic()
    try:
        if probe_only:
            evaluator.process_watchdog.write_heartbeat(output / "watchdog_state.json", "bootstrap probe imports and environment build", timeout_s=600)
            loader()
            config = dict(helper.relocated_historical_receipt()["requested_env_config"])
            config.update(output_dir=str(output / "sim_outputs"), record_outputs=False, save_outputs_on_close=False,
                          episode_start_offset_s=plan["episode_start_offset_s"], episode_duration=plan["duration_s"])
            env = evaluator.env_factory.make_cmc_env(config)
            obs, info = env.reset(seed=123)
            module = evaluator.RLModule.from_checkpoint(helper.resolve(plan["checkpoint"]))
            action = evaluator._deterministic_action(module, obs, env.action_space.shape)
            applied_action = apply_wrapped_action(env, action, evaluator.env_factory.FlattenClipAction)
            base = env.unwrapped
            times, values, derivatives = base._action_to_segment(applied_action, base.t + 0.01)
            base.kin.set_segment(times, values, derivatives)
            receipt["first_policy_action"] = evaluator._jsonable(action)
            receipt["first_raw_policy_action"] = evaluator._jsonable(action)
            receipt["first_applied_policy_action"] = evaluator._jsonable(applied_action)
            receipt["first_planned_segment"] = {"times": times.tolist(), "values": values.tolist(), "derivatives": derivatives.tolist()}
            receipt["first_governor_segment"] = adapters[0].first_segment if adapters[0] is not None else None
            receipt["status"] = "bootstrap_probe_complete_without_integration"
            env.close()
        else:
            summary = evaluator.run(args)
            base = environments[0].unwrapped
            receipt.update(status="rollout_complete", actual_start_s=float(base._episode_start), actual_end_s=float(base.t),
                           completed_requested_duration=bool(summary["end_reason"] == "episode_time_limit" and not summary["terminated"] and abs(base.t - base._episode_end) < 1e-8),
                           first_governor_segment=adapters[0].first_segment if adapters[0] is not None else None)
            helper.write_json(output / "receipt.json", receipt)
            postprocess_saved(output, receipt, helper)
        return_code = 0
    except StaticResetRejected as exc:
        receipt.update(status="static_reset_rejected", rejection=str(exc))
        return_code = 0
    except Exception as exc:
        receipt.update(status=exception_category(exc), error=f"{type(exc).__name__}: {exc}",
                       completed_requested_duration=False,
                       actual_start_s=receipt.get("initial_state", {}).get("time_s"),
                       actual_end_s=valid_rows[-1]["time"] if valid_rows else receipt.get("initial_state", {}).get("time_s"))
        if not probe_only:
            preserve_censored_summary(output, receipt, valid_rows, evaluator, args, exc)
            helper.write_json(output / "receipt.json", receipt)
            postprocess_saved(output, receipt, helper)
        traceback.print_exc()
        return_code = 0 if exception_category(exc) != "harness_or_native_failure" else 1
    finally:
        receipt["elapsed_wall_s"] = time.monotonic() - started
        if adapters:
            receipt["first_governor_segment"] = adapters[0].first_segment if adapters[0] is not None else None
        helper.verify_inputs(plan)
        receipt["frozen_inputs_unchanged"] = True
        helper.write_json(output / "receipt.json", receipt)
        evaluator._load_inference_stack = original_loader
        if original_factory is not None:
            evaluator.env_factory.make_cmc_env = original_factory
        for env in environments:
            try:
                env.close()
            except Exception:
                pass
    print(json.dumps({"case": case["id"], "status": receipt["status"], "integration_steps": receipt["integration_steps"]}))
    return return_code


def static_preflight(plan, destination):
    """Screen the frozen 400-candidate order without advancing physical time."""
    helper = common()
    helper.verify_inputs(plan)
    if destination.exists():
        raise FileExistsError(f"Preserve prior preflight: {destination}")
    destination.mkdir(parents=True)
    evaluator = import_file("random_reset_static_preflight_evaluator", EVALUATOR)
    evaluator._load_inference_stack()
    config = dict(helper.relocated_historical_receipt()["requested_env_config"])
    config.update(output_dir=str(destination / "unused_outputs"), record_outputs=False, save_outputs_on_close=False,
                  episode_start_offset_s=plan["episode_start_offset_s"], episode_duration=plan["duration_s"])
    env = evaluator.env_factory.make_cmc_env(config)
    base = env.unwrapped
    original_reset, original_kin = env.reset, base.kin
    outcomes, selected = [], []
    requested = plan["random_pose_sampling"]["selected_random_count"]
    try:
        for case in plan["cases"]:
            output = destination / case["id"]
            output.mkdir()
            receipt = {"schema_version": 1, "instruction_check_token": helper.TOKEN, "case": case,
                       "status": "preflight_started", "output_dir": str(output), "integration_steps": 0,
                       "policy_loaded": False, "policy_updates": 0}
            env.reset, base.kin, base.runner._kin = original_reset, original_kin, original_kin
            install_bootstrap(env, case, receipt, evaluator)
            try:
                env.reset(seed=123)
                receipt["status"] = "static_eligible"
                if case["id"] == "coherent_nominal" or len([item for item in selected if item["id"] != "coherent_nominal"]) < requested:
                    selected.append(case)
            except StaticResetRejected as exc:
                receipt.update(status="static_rejected", rejection=str(exc))
            except Exception as exc:
                receipt.update(status="preflight_harness_or_native_failure", error=f"{type(exc).__name__}: {exc}")
                helper.write_json(output / "receipt.json", receipt)
                raise
            helper.write_json(output / "receipt.json", receipt)
            outcomes.append({"id": case["id"], "status": receipt["status"], "gate": receipt.get("static_contact_gate")})
        helper.verify_inputs(plan)
        eligible_control = any(item["id"] == "coherent_nominal" for item in selected)
        ready = eligible_control and len(selected) == requested + 1
        selection = {"schema_version": 1, "cases": selected, "ready_for_preregistered_rollouts": ready,
                     "selection_rule": f"coherent nominal control plus first {requested} eligible random candidates in frozen order",
                     "integration_steps": 0, "candidate_count": plan["random_pose_sampling"]["candidate_count"],
                     "frozen_source_hashes": plan["protected_sha256"],
                     "preflight_receipt_hashes": {helper.relative(destination / case["id"] / "receipt.json"): helper.sha256(destination / case["id"] / "receipt.json") for case in plan["cases"]},
                     "preflight_outcomes": outcomes, "frozen_inputs_unchanged": True}
        helper.write_json(destination / "selected_cases.json", selection)
        print(json.dumps({"preflight_complete": True, "candidate_count": len(outcomes) - 1,
                          "selected_random_count": len(selected) - int(eligible_control), "ready": ready,
                          "selection": str(destination / "selected_cases.json")}))
        return 0 if ready else 2
    finally:
        env.reset, base.kin, base.runner._kin = original_reset, original_kin, original_kin
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--run", action="store_true")
    action.add_argument("--bootstrap-probe", action="store_true")
    action.add_argument("--static-preflight", action="store_true")
    action.add_argument("--worker", metavar="CASE")
    parser.add_argument("--probe-worker", action="store_true")
    parser.add_argument("--count", type=int, default=20, help="Required first eligible random candidates")
    parser.add_argument("--candidate-count", type=int, default=400)
    parser.add_argument("--sampling-seed", type=int, default=20261013)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--manifest", type=Path, default=AUDIT / "frozen_manifest.json")
    parser.add_argument("--output-root", type=Path, default=AUDIT / "rollouts")
    parser.add_argument("--selection", type=Path, default=AUDIT / "preflight/selected_cases.json")
    parser.add_argument("--selection-sha256", help=argparse.SUPPRESS)
    parser.add_argument("--cases", nargs="+")
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()
    helper = common()
    if not (args.run or args.worker or args.bootstrap_probe or args.static_preflight):
        plan = prepare(args.count, args.sampling_seed, args.duration_s, args.candidate_count)
        if args.prepare:
            if args.manifest.exists():
                raise FileExistsError("Preserve existing manifest; select another --manifest")
            helper.write_json(args.manifest, plan)
        print(json.dumps({"case_count_including_control": len(plan["cases"]), "first_cases": plan["cases"][:4],
                          "sampling": plan["random_pose_sampling"], "static_gate": plan["static_gate"],
                          "missing_required_files": plan["missing_required_files"], "native_execution_started": False}, indent=2))
        return 0
    plan = helper.read_json(args.manifest)
    helper.verify_inputs(plan)
    root = args.output_root.resolve()
    if args.static_preflight:
        return static_preflight(plan, AUDIT / "preflight")
    if args.worker:
        if args.selection_sha256 and helper.sha256(args.selection) != args.selection_sha256:
            raise RuntimeError("Frozen preflight selection changed before worker execution")
        case = next(case for case in plan["cases"] if case["id"] == args.worker)
        return execute(plan, case, root / case["id"], args.probe_worker)
    available = plan["cases"]
    if args.run:
        selection = helper.read_json(args.selection)
        if not selection["ready_for_preregistered_rollouts"] or selection["frozen_source_hashes"] != plan["protected_sha256"]:
            raise RuntimeError("Preflight selection missing/incomplete or source identity changed")
        available = validated_selection_cases(plan, selection, args.selection.resolve().parent, helper)
    cases = [case for case in available if args.cases is None or case["id"] in args.cases]
    if args.cases and set(args.cases) != {case["id"] for case in cases}:
        raise ValueError("Unknown case id")
    root.mkdir(parents=True, exist_ok=True)
    evaluator = import_file("random_reset_original_supervisor", EVALUATOR)
    executions = []
    batch_suffix = hashlib.sha256("\n".join(case["id"] for case in cases).encode("utf-8")).hexdigest()[:16]
    batch_path = root / f"batch_execution_{'probe' if args.bootstrap_probe else 'run'}_{batch_suffix}.json"
    for case in cases:
        output = root / case["id"]
        if output.exists():
            raise FileExistsError(f"Preserve previous case output: {output}")
        argv = [args.python, "-B", "-u", str(Path(__file__).resolve()), "--worker", case["id"], "--manifest", str(args.manifest.resolve()), "--output-root", str(root)]
        if args.bootstrap_probe:
            argv.append("--probe-worker")
        elif args.run:
            argv.extend(["--selection", str(args.selection.resolve()), "--selection-sha256", helper.sha256(args.selection)])
        log = root / (case["id"] + ".log")
        launcher = root / (case["id"] + "_supervisor.py")
        launcher.write_text("import subprocess, sys\nfrom pathlib import Path\nwith Path(sys.argv[1]).open('wb') as log:\n    raise SystemExit(subprocess.call(sys.argv[2:], stdout=log, stderr=subprocess.STDOUT))\n", encoding="utf-8")
        result = evaluator.process_watchdog.supervise_process([args.python, "-B", str(launcher), str(log), *argv],
            heartbeat_file=output / "watchdog_state.json", summary_file=root / (case["id"] + "_watchdog.json"),
            startup_timeout_s=600, stall_timeout_s=180, run_timeout_s=1800,
            label="Coherent random prosthetic reset " + case["id"])
        helper.verify_inputs(plan)
        helper.write_json(root / (case["id"] + "_execution.json"), {"argv": argv, "watchdog": result})
        executions.append({"case": case["id"], "argv": argv, "watchdog": result})
        helper.write_json(batch_path, {"cases": executions, "policy_updates": 0,
                                                         "continue_after_single_case_failure": True})
    return 1 if any(item["watchdog"]["returncode"] != 0 or item["watchdog"]["timeout_reason"] for item in executions) else 0


if __name__ == "__main__":
    raise SystemExit(main())
