"""Isolated, training-free TNT/prescribed dynamic comparison.

The caller must first qualify the frozen kinematic controller bundle. This
adapter instantiates SimulationRunner directly: no Gym environment, network,
reward, gait clock, old binary detector or prosthetic phase FSM is constructed.
The primary GRF components remain part of the physical plant. Their existing
numerical hard guards can abort the diagnostic, never tune the generator.
Run with the repository OpenSim environment; no operational source is changed.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
import time
import traceback
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
STUDY = HERE.parent
TG = HERE.parents[2]
REPO = TG.parent
DEFAULT_RECEIPT = TG / "validation/AB06/v26c_best47_control_2026_10_06/runtime_receipt.json"
PROTECTED = [REPO / p for p in (
    "config.py", "model_loader.py", "simulation_runner.py", "prosthesis_controller.py",
    "kinematics_interpolator.py", "outer_loop.py", "static_optimization.py",
    "inverse_dynamics.py", "output.py", "Trajectory Generator/osim_trj_cmc_like.py",
)]


def local_path(raw):
    value = str(raw).replace("\\", "/")
    for anchor in ("/Trajectory Generator/", "/models/", "/plugins/", "/validation/",
                   "/online_grf_profiles/"):
        if anchor in value:
            return REPO / (anchor[1:] + value.split(anchor, 1)[1])
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def fingerprint(path):
    path = Path(path).resolve()
    try:
        label = path.relative_to(REPO).as_posix()
    except ValueError:
        label = str(path)
    return {"path": label, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def serial(value):
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if hasattr(value, "tolist"):
        return serial(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, data):
    path.write_text(json.dumps(serial(data), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def heading(bundle):
    values = bundle.get("angle_definition", {}).get("forward_ground", bundle.get("forward_ground"))
    if values is None:
        raise ValueError("Bundle must declare angle_definition.forward_ground or forward_ground")
    import numpy as np
    forward = np.asarray(values, dtype=float)
    if forward.shape != (3,) or not np.isfinite(forward).all() or abs(forward[1]) > 1e-8:
        raise ValueError("Bundle heading must be a finite horizontal three-vector")
    if abs(float(np.linalg.norm(forward)) - 1) > 1e-6:
        raise ValueError("Bundle heading must be normalized")
    return forward


def prepare(args):
    source = json.loads(args.receipt.read_text(encoding="utf-8"))
    bundle = json.loads(args.bundle.read_text(encoding="utf-8"))
    heading(bundle)
    if bundle.get("joint_names") != ["pros_knee_angle", "pros_ankle_angle"] or bundle.get("joint_units") != "degree":
        raise ValueError("Bundle must specify knee/ankle absolute references in degrees")
    definition = bundle["angle_definition"]
    if definition.get("body") != "transfemur" or definition.get("body_longitudinal_axis") != [0, -1, 0]:
        raise ValueError("Bundle thigh body/axis differs from the runtime measurement")
    raw = dict(source["effective_simulator_config"])
    path_keys = ("model_bundle_dir", "model_file", "kinematics_file", "external_loads_xml",
                 "reserve_actuators_xml", "online_grf_profile_file")
    for key in path_keys:
        if raw.get(key):
            raw[key] = str(local_path(raw[key]))
    # Remove only measurement/detection components, retaining primary contact.
    raw.update(online_grf_detector_profile_file="", binary_phase_detector_profile_file="",
               save_gait_events=False, output_dir=str(args.outdir / "sim_outputs"),
               output_prefix="tnt_diagnostic", t_start=args.start, t_end=args.start + args.duration)
    if raw["grf_mode"] != "online_sensor" or raw["online_grf_applied_sides"] != ["left"]:
        raise ValueError("Expected the frozen left-online/right-prescribed physical plant")
    if raw["integration_scheme"] != "rk4_bypass" or raw["sea_forward_mode"] != "plugin":
        raise ValueError("Expected the validated RK4/plugin SEA physical path")
    env = source["effective_env_config"]
    if env["pros_ref_model"] != "butterworth3_jerk_limited":
        raise ValueError("Diagnostic requires the frozen C2 governor")
    if not math.isclose(env["segment_duration"], .01, abs_tol=1e-10):
        raise ValueError("Diagnostic requires 100 Hz generation")
    input_bindings = {}
    for kind, key in (("model", "model_file"), ("prescribed_ik", "kinematics_file")):
        observed = fingerprint(raw[key])
        if observed["sha256"] != bundle["inputs"][kind]["sha256"]:
            raise ValueError(f"The runtime {kind} does not match the frozen generator calibration")
        input_bindings[kind] = observed
    for key in ("reserve_actuators_xml", "external_loads_xml", "online_grf_profile_file"):
        if raw.get(key):
            input_bindings[key] = fingerprint(raw[key])
    controller_sources = [STUDY / "kinematic_generator.py",
                          STUDY.parent / "tnt_preliminary_2026_10_13/tnt_replay.py"]
    input_bindings["generator_dependencies"] = [fingerprint(p) for p in controller_sources]
    receipt = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026", "status": "prepared",
        "mode": args.mode, "requested_window_s": [args.start, args.start + args.duration],
        "source_receipt": fingerprint(args.receipt), "bundle": fingerprint(args.bundle),
        "adapter": fingerprint(Path(__file__)),
        "input_bindings": input_bindings,
        "protected_before": [fingerprint(p) for p in PROTECTED],
        "effective_simulator_config": raw,
        "inherited_reference_and_safety_config": {k: env[k] for k in (
            "enable_pros_ref_lpf", "pros_ref_model", "pros_ref_lpf_cutoff_hz",
            "pros_ref_lpf_zeta", "enable_pros_ref_governor", "pros_ref_velocity_limit_rad_s",
            "pros_ref_acceleration_limit_rad_s2", "pros_ref_jerk_limit_rad_s3",
            "target_slew_rate_limit_rad_s", "absolute_bounds_rad", "truncation_bounds_rad",
            "pelvis_min_height", "grf_penetration_termination_m", "step_wall_timeout_s")},
        "input_contract": "Generator receives only current time and actual transfemur angle; no force/contact/event input",
        "baseline_contract": "Current prescribed prosthetic q sampled at 100 Hz, with identical endpoint bounds, target slew and governor",
        "initialization": "Physical reset from IK; generator starts from current measured thigh and nominal stance, no prescribed sensor prehistory; governor anchored at actual reset q/qdot",
        "state_semantics": "TNT stance is an internal estimator mode, not an observed heel strike or an assertion of knee extension/unloading. This tests TNT plus fitted full-cycle curves and a stance timer, not the paper's original knee Fourier trajectory.",
        "timing": "Target computed at time t, held as the C2 governor endpoint command over [t,t+10ms]; no future measured thigh or target",
        "plant_guards": {"force_limit_bw": raw["online_grf_max_force_bw"],
                         "substep_penetration_limit_m": raw["online_grf_max_penetration_m"],
                         "policy_step_penetration_limit_m": env["grf_penetration_termination_m"],
                         "force_use": "Simulator integrity abort only; no controller, calibration, reward, parameter selection or training"},
        "limitations": ["Short isolated diagnostic; no proof of generalization or clinical safety",
                        "Biological side tracks prescribed motion through existing muscle/reserve dynamics",
                        "No GRF-derived targets, gait events or phase FSM are constructed by this adapter",
                        "The established physical contact model retains its original calibration provenance"],
    }
    return receipt, bundle


def execute(args, receipt, bundle):
    for directory in (REPO, TG, TG / "baseline_MLP", STUDY):
        sys.path.insert(0, str(directory))
    import win_runtime  # noqa: F401: preserves Windows OpenSim/runtime load order
    import numpy as np
    import opensim
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator
    from model_loader import setup_model
    from osim_trj_cmc_like import ProstheticSegmentKinematics
    from simulation_runner import SimulationRunner

    cfg = SimulatorConfig(**receipt["effective_simulator_config"])
    params = receipt["inherited_reference_and_safety_config"]
    ctx = setup_model(cfg)
    if ctx.online_grf_detector_force_paths or ctx.binary_phase_detector_profile is not None:
        raise AssertionError("Old phase detectors must be absent")
    base = KinematicsInterpolator(cfg)
    kin = ProstheticSegmentKinematics(
        base, cfg.pros_coords, ref_lpf_enable=params["enable_pros_ref_lpf"],
        ref_model=params["pros_ref_model"], ref_lpf_cutoff_hz=params["pros_ref_lpf_cutoff_hz"],
        ref_lpf_zeta=params["pros_ref_lpf_zeta"], ref_governor_enable=params["enable_pros_ref_governor"],
        ref_velocity_limits=[params["pros_ref_velocity_limit_rad_s"][n] for n in cfg.pros_coords],
        ref_acceleration_limits=[params["pros_ref_acceleration_limit_rad_s2"][n] for n in cfg.pros_coords],
        ref_jerk_limits=[params["pros_ref_jerk_limit_rad_s3"][n] for n in cfg.pros_coords],
        control_dt=cfg.dt,
    )
    runner = SimulationRunner(cfg, ctx, kin)
    # The runner normally constructs an event logger from the primary force.
    # It has no dynamical role and is disabled even for diagnostic events here.
    runner._online_event_detector = None
    runner.reset_to_time(args.start)
    runner.reset_outputs()
    thigh = ctx.model.getBodySet().get("transfemur")
    forward = heading(bundle)

    def measured_thigh():
        ctx.model.realizePosition(runner.state)
        v = thigh.expressVectorInGround(runner.state, opensim.Vec3(0, -1, 0))
        axis = np.array([v.get(i) for i in range(3)], dtype=float)
        if not np.isfinite(axis).all() or (axis @ forward)**2 + axis[1]**2 < 1e-10:
            raise FloatingPointError("Invalid or undefined sagittal thigh projection")
        return float(np.rad2deg(np.arctan2(axis @ forward, -axis[1])))

    coordinates = [ctx.model.getCoordinateSet().get(n) for n in cfg.pros_coords]
    initial_q = np.array([c.getValue(runner.state) for c in coordinates])
    initial_v = np.array([c.getSpeedValue(runner.state) for c in coordinates])
    controller = None
    if args.mode == "tnt":
        module_path = STUDY / "kinematic_generator.py"
        module = load_module("tnt_diagnostic_kinematic_generator", module_path)
        controller = module.KinematicTNTController(bundle, args.start, measured_thigh())
        receipt["controller_source"] = fingerprint(module_path)
    receipt.update(status="running", effective_simulator_config=asdict(cfg),
                   initial_q_rad=initial_q, initial_qdot_rad_s=initial_v,
                   initial_thigh_deg=measured_thigh(), generated_samples=0)
    write_json(args.outdir / "receipt.json", receipt)
    last_endpoint = initial_q.copy()
    slew = np.array([params["target_slew_rate_limit_rad_s"][n] for n in cfg.pros_coords])
    bounds = np.array([params["absolute_bounds_rad"][n] for n in cfg.pros_coords])
    trace, previous_print = [], time.monotonic()
    pending_row = None
    wall_start = time.monotonic()
    code = 1
    try:
        with (args.outdir / "generator_trace.jsonl").open("x", encoding="utf-8") as stream:
            while runner.current_time < cfg.t_end - 1e-10:
                t = float(runner.current_time)
                target_t = min(t + .01, cfg.t_end)
                theta = measured_thigh()
                prescribed_q, _, _ = base.get(t)
                if controller is None:
                    raw = np.array([prescribed_q[n] for n in cfg.pros_coords])
                    diagnostics = {"source": "prescribed_current_q"}
                else:
                    raw = np.deg2rad(np.asarray(controller.step(t, theta), dtype=float))
                    diagnostics = dict(controller.last)
                if raw.shape != (2,) or not np.isfinite(raw).all():
                    raise FloatingPointError("Generator produced a nonfinite or invalid target")
                clipped = np.clip(raw, bounds[:, 0], bounds[:, 1])
                endpoint = last_endpoint + np.clip(clipped - last_endpoint,
                                                   -slew * (target_t - t), slew * (target_t - t))
                q_anchor, v_anchor, _ = kin.get(t)
                q0 = np.array([q_anchor[n] for n in cfg.pros_coords])
                v0 = np.array([v_anchor[n] for n in cfg.pros_coords])
                kin.set_segment(np.array([t, target_t]), np.stack([q0, endpoint]),
                                np.stack([v0, (endpoint - q0) / (target_t - t)]))
                row = {"time": t, "thigh_actual_deg": theta, "generator": diagnostics,
                       "raw_target_rad": raw, "bounded_target_rad": clipped,
                       "slew_limited_target_rad": endpoint, "served_at_start_rad": q0,
                       "prescribed_same_time_q_rad": {n: prescribed_q[n] for n in cfg.pros_coords},
                       "actual_at_start_rad": [c.getValue(runner.state) for c in coordinates],
                       "governor": kin.reference_governor_diagnostics}
                pending_row = row
                last_endpoint = endpoint
                info = runner.step_until(target_t, record=True,
                                         wall_timeout_s=params["step_wall_timeout_s"])
                row["completed_until_s"] = float(runner.current_time)
                row["actual_at_end_rad"] = [c.getValue(runner.state) for c in coordinates]
                row["served_at_end_rad"] = [kin.get(target_t)[0][n] for n in cfg.pros_coords]
                row["plant_diagnostics"] = {k: info.get(k) for k in ("online_grf", "sea_segment_diagnostics", "so_diagnostics")}
                trace.append(serial(row))
                pending_row = None
                stream.write(json.dumps(serial(row), allow_nan=False) + "\n")
                values = ctx.model.getStateVariableValues(runner.state)
                actual = np.array([values.get(i) for i in range(values.size())], dtype=float)
                if not np.isfinite(actual).all():
                    raise FloatingPointError("Nonfinite physical state")
                pelvis_y = float(values.get(ctx.q_sv_idx["pelvis_ty"]))
                if pelvis_y < params["pelvis_min_height"]:
                    raise FloatingPointError(f"Geometric fall guard: pelvis height {pelvis_y}")
                for n in cfg.pros_coords:
                    low, high = params["truncation_bounds_rad"][n]
                    value = float(values.get(ctx.q_sv_idx[n]))
                    if not low <= value <= high:
                        raise FloatingPointError(f"Joint divergence guard: {n}={value}")
                for side in cfg.online_grf_applied_sides:
                    penetration = float(info["online_grf"][side]["penetration"])
                    if penetration > params["grf_penetration_termination_m"]:
                        raise FloatingPointError(f"Geometric penetration guard: {side}={penetration}")
                if time.monotonic() - previous_print > 20:
                    stream.flush()
                    print(f"mode={args.mode} t={runner.current_time:.4f}/{cfg.t_end:.4f} samples={len(trace)}", flush=True)
                    previous_print = time.monotonic()
            receipt["status"] = "completed"
            code = 0
    except Exception as error:
        receipt.update(status="aborted", error=f"{type(error).__name__}: {error}",
                       traceback=traceback.format_exc())
        if pending_row is not None:
            receipt["attempted_incomplete_step"] = serial(pending_row)
        if getattr(error, "online_grf", None) is not None:
            receipt["aborted_substep_contact"] = error.online_grf
    finally:
        try:
            runner.save_results()
        except Exception as save_error:
            receipt["save_error"] = f"{type(save_error).__name__}: {save_error}"
            code = 1
        receipt.update(final_time_s=float(runner.current_time), generated_samples=len(trace),
                       elapsed_wall_time_s=time.monotonic() - wall_start,
                       protected_sources_unchanged=receipt["protected_before"] == [fingerprint(p) for p in PROTECTED])
        dependencies = receipt["input_bindings"]["generator_dependencies"]
        receipt["generator_dependencies_unchanged"] = dependencies == [fingerprint(local_path(x["path"])) for x in dependencies]
        if not receipt["generator_dependencies_unchanged"]:
            code = 1
        if not receipt["protected_sources_unchanged"]:
            receipt["source_integrity_error"] = "An operational source changed during diagnostic"
            code = 1
        write_json(args.outdir / "generator_trace.json", trace)
        write_json(args.outdir / "receipt.json", receipt)
        print(json.dumps({"status": receipt["status"], "final_time_s": receipt["final_time_s"],
                          "elapsed_wall_time_s": receipt["elapsed_wall_time_s"],
                          "receipt": str(args.outdir / "receipt.json")}), flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("prescribed", "tnt"))
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--receipt", type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument("--start", required=True, type=float)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--outdir", required=True, type=Path)
    parser.add_argument("--run", action="store_true", help="Execute; omitted prepares receipt only")
    args = parser.parse_args()
    if not np_finite_positive_duration(args.start, args.duration):
        parser.error("Start must be finite and duration positive; maximum isolated diagnostic is 10 seconds")
    args.bundle, args.receipt, args.outdir = [p.resolve() for p in (args.bundle, args.receipt, args.outdir)]
    if args.outdir.exists() and any(args.outdir.iterdir()):
        parser.error("Output directory must be new or empty")
    args.outdir.mkdir(parents=True, exist_ok=True)
    receipt, bundle = prepare(args)
    write_json(args.outdir / "receipt.json", receipt)
    if not args.run:
        return 0
    try:
        return execute(args, receipt, bundle)
    except Exception as error:
        receipt.update(status="setup_or_save_failed", error=f"{type(error).__name__}: {error}",
                       traceback=traceback.format_exc())
        write_json(args.outdir / "receipt.json", receipt)
        raise


def np_finite_positive_duration(start, duration):
    return math.isfinite(start) and math.isfinite(duration) and 0 < duration <= 10


if __name__ == "__main__":
    raise SystemExit(main())
