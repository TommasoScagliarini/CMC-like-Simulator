"""Single AB07 prescribed diagnostic in the frozen V26 contact/FSM runtime.

No checkpoint is loaded. The raw diagnostic action comes from the AB07 IK;
the V26 reference governor and the existing Python/plugin controllers remain
active. Run only after the bundle and ordinary prescribed simulation are ready.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import traceback
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET


REPO = Path(__file__).resolve().parents[3]
TG = REPO / "Trajectory Generator"
SOURCE_CONFIG = (
    TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
    / "training_cfg.resolved.yaml"
)
DEFAULT_SETUP = (
    "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml"
)
FROZEN_PROFILES = {
    "online_grf_profile_file": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
        "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0",
    ),
    "online_grf_detector_profile_file": (
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
        "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e",
    ),
    "binary_phase_detector_profile_file": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json",
        "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2",
    ),
}


def _path(raw: str | Path) -> Path:
    value = Path(str(raw).replace("\\", "/"))
    return value.resolve() if value.is_absolute() else (REPO / value).resolve()


def _fingerprint(raw: str | Path) -> dict[str, str]:
    path = _path(raw)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def build_env_kwargs(
    source: dict[str, Any], *, setup: str, output_dir: str,
    stage: str, duration: float = 9.01,
) -> dict[str, Any]:
    """Mirror train_ppo_mlp/env_factory physical settings without RL wrappers."""
    if stage not in {"sensor_only", "left_applied"}:
        raise ValueError("stage must be sensor_only or left_applied")
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    sim, grf, reward = source["simulation"], source["grf"], source["reward"]
    kwargs = {
        "setup_xml_path": str(_path(setup)),
        "segment_duration": sim["segment_duration"],
        "episode_duration": duration,
        "episode_start_offset_s": 0.0,
        "policy_knots": sim["policy_knots"],
        "action_mode": "raw",
        "random_init": sim["random_init"],
        "rebuild_model_on_reset": False,
        "fail_fast": True,
        "step_wall_timeout_s": sim["step_wall_timeout_s"],
        "record_outputs": True,
        "save_outputs_on_close": True,
        "output_dir": str(_path(output_dir) / "sim_outputs"),
        "output_prefix": "sim_output",
        "enable_pros_ref_governor": sim["pros_ref_governor"],
        "pros_ref_model": sim["pros_ref_model"],
        "pros_ref_lpf_cutoff_hz": sim["pros_ref_cutoff_hz"],
        "critic_privileged_observation": source["model"]["asymmetric_actor_critic"],
        "include_imitation_target_observation": reward["reward_mode"] == "imitation",
        "grf_mode": grf["grf_mode"],
        "online_grf_profile_file": grf["online_grf_profile"],
        "online_grf_detector_profile_file": grf["online_grf_detector_profile"],
        "binary_phase_detector_profile_file": grf["binary_phase_detector_profile"],
        "include_online_grf_observation": grf["online_grf_observation"],
        "online_grf_applied_sides": grf["online_grf_applied_side"] if stage == "left_applied" else [],
        "prescribed_grf_disabled_sides": grf["disable_prescribed_grf_side"],
        "binary_phase_actor_fsm_version": "v3",
        "phase_landing_force_full_credit_bw": reward["contact_load_target_bw"],
        "grf_ankle_moment_flip_tau_tol_nm": reward["grf_ankle_moment_flip_tau_tol_nm"],
        "grf_ankle_moment_flip_force_threshold_n": reward["grf_ankle_moment_flip_force_threshold_n"],
    }
    for key in (
        "gait_clock_enable", "actor_cyclic_phase_only",
        "include_reference_state_observation", "include_controller_state_observation",
        "include_controller_diagnostic_observation", "deployable_minimal_observation",
        "imitation_initialize_to_target", "reward_reference_range_floor",
        "reward_reference_velocity_range_floor", "grf_penetration_penalty_threshold_m",
        "grf_penetration_termination_m",
    ):
        kwargs[key] = sim[key]
    for key in (
        "phase_fsm_input_mode", "phase_sensor_on_threshold_n", "phase_sensor_off_threshold_n",
        "phase_sensor_dwell_s", "detector_sample_dt_s", "event_contract_id",
        "binary_phase_fsm_mode", "binary_phase_debounce_s",
        "binary_phase_invalid_event_policy", "binary_phase_event_contract_id",
    ):
        kwargs[key] = grf[key]
    for key in (
        "phase_min_stance_duration_s", "phase_min_swing_duration_s",
        "phase_landing_window_start_s", "phase_landing_window_end_s",
        "phase_stance_hard_timeout_s", "phase_swing_hard_timeout_s",
        "phase_min_stance_contact_fraction", "phase_min_stance_load_bw_s",
        "phase_min_cycle_knee_excursion_rad", "phase_hs_event_credit",
        "phase_to_event_credit", "phase_cycle_complete_bonus", "phase_failure_extra_penalty",
    ):
        kwargs[key] = reward[key]
    kwargs["target_slew_rate_limit_rad_s"] = {
        f"pros_{joint}_angle": sim[f"pros_{joint}_target_slew_rate_limit_rad_s"]
        for joint in ("knee", "ankle")
    }
    for kind, unit in (("velocity", "rad_s"), ("acceleration", "rad_s2"), ("jerk", "rad_s3")):
        kwargs[f"pros_ref_{kind}_limit_{unit}"] = {
            f"pros_{joint}_angle": sim[f"pros_{joint}_ref_{kind}_limit_{unit}"]
            for joint in ("knee", "ankle")
        }
    for kind in ("position", "velocity"):
        kwargs[f"imitation_{kind}_weights"] = {
            f"pros_{joint}_angle": reward[f"imitation_{joint}_{kind}_weight"]
            for joint in ("knee", "ankle")
        }
    # Existing lineage contracts remain authoritative; this script cannot tune them.
    if (kwargs["grf_mode"], kwargs["binary_phase_fsm_mode"],
        kwargs["binary_phase_event_contract_id"]) != (
        "online_sensor", "binary_active", "binary_point_v25+heel_qualified_fsm_v2"
    ):
        raise ValueError("source config is not the frozen V26 active lineage")
    if list(grf["online_grf_applied_side"]) != ["left"] or grf["disable_prescribed_grf_side"]:
        raise ValueError("source config must retain V26 left hybrid GRF")
    for key, (expected_path, _) in FROZEN_PROFILES.items():
        if _path(kwargs[key]) != _path(expected_path):
            raise ValueError(f"frozen profile path differs: {key}")
    if kwargs["random_init"] or kwargs["imitation_initialize_to_target"]:
        raise ValueError("source must retain prescribed deterministic reset")
    return kwargs


def _json_value(value: Any) -> Any:
    """Convert NumPy diagnostics while emitting strict JSON (no NaN tokens)."""
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(v) for v in value]
    if hasattr(value, "tolist"):
        return _json_value(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_json_value(value), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def prepare_receipt(setup: str, kwargs: dict[str, Any], stage: str) -> dict[str, Any]:
    # These modules are file/config utilities and do not import OpenSim.
    sys.path.insert(0, str(REPO))
    from setup_io import read_setup_xml

    parsed = read_setup_xml(setup)
    if parsed.t_end - parsed.t_start + 1e-9 < kwargs["episode_duration"]:
        raise ValueError("setup window is shorter than the requested diagnostic")
    profiles = {}
    for key, (path, expected_sha) in FROZEN_PROFILES.items():
        observed = _fingerprint(path)
        if observed["sha256"] != expected_sha:
            raise ValueError(f"frozen profile hash differs: {key}")
        profiles[key] = observed
    source_files = {
        "source_config": _fingerprint(SOURCE_CONFIG),
        "simulator_config": _fingerprint(REPO / "config.py"),
        "environment": _fingerprint(TG / "osim_trj_cmc_like.py"),
        "action_generator": _fingerprint(TG / "prescribed_action.py"),
        "setup": _fingerprint(setup), "model": _fingerprint(parsed.model_file),
        "ik": _fingerprint(parsed.kinematics_file),
        "reserve_actuators": _fingerprint(parsed.reserve_actuators_xml),
    }
    if parsed.external_loads_xml is not None:
        source_files["external_loads_xml"] = _fingerprint(parsed.external_loads_xml)
        loads = ET.parse(parsed.external_loads_xml).getroot()
        datafile = loads.find(".//datafile")
        if datafile is not None and datafile.text:
            raw = Path(datafile.text.strip().replace("\\", "/"))
            grf_file = raw if raw.is_absolute() else parsed.external_loads_xml.parent / raw
            source_files["prescribed_grf"] = _fingerprint(grf_file)
    return {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "subject": "AB07", "stage": stage,
        "action_source": "AB07 setup IK via prescribed_action; no learned policy/checkpoint",
        "profiles_status": "shared frozen V26 candidates; AB07 qualification pending",
        "qualification_verdict": "requires review of contact, FSM, tracking, SEA and reserve diagnostics",
        "profile_retuning": False,
        "source_files": source_files, "profile_association": profiles,
        "env_kwargs": kwargs,
        "diagnostic_overrides": {
            "setup": setup, "episode_duration_s": kwargs["episode_duration"],
            "episode_start_offset_s": 0.0, "action_mode": "raw",
            "binary_phase_actor_fsm_version": "v3 (current runtime default)",
            "record_outputs": True,
            "online_grf_applied_sides": kwargs["online_grf_applied_sides"],
        },
        "limitations": [
            "One prescribed session does not establish multi-start or policy qualification.",
            "AB06 ground, belt velocity and residual-wrench fits are unchanged candidates.",
            "Ankle-moment-flip diagnostics depend on the runtime ankle-center geometry implementation.",
        ],
        "requested_time_range": [parsed.t_start, parsed.t_start + kwargs["episode_duration"]],
        "status": "prepared",
    }


def run_probe(receipt: dict[str, Any], output: Path, seed: int) -> int:
    """This function is the only place that imports/starts the native runtime."""
    for entry in (str(REPO), str(TG), str(TG / "baseline_MLP")):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    env = None
    info: dict[str, Any] = {}
    steps = 0
    code = 1
    receipt["seed"] = seed
    try:
        import win_runtime  # noqa: F401: Windows torch/OpenSim load-order shim
        from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
        from prescribed_action import prescribed_action

        env = CMCLikeProsthesisTrajectoryEnv(CMCEnvConfig(**receipt["env_kwargs"]))
        receipt["effective_simulator_config"] = asdict(env.cfg)
        receipt["effective_env_config"] = asdict(env.env_cfg)
        _, reset = env.reset(seed=seed)
        receipt["reset"] = reset
        _write_json(output / "receipt.json", receipt)
        with (output / "trace.jsonl").open("w", encoding="utf-8") as trace:
            while True:
                _, _, terminated, truncated, info = env.step(prescribed_action(env))
                steps += 1
                row = {key: value for key, value in info.items() if key not in {"so_solver_audit_entries", "log"}}
                trace.write(json.dumps(_json_value(row), allow_nan=False) + "\n")
                if steps % 100 == 0 or terminated or truncated:
                    trace.flush()
                    print(f"step={steps} t={env.t:.6f} end={info.get('end_reason')}", flush=True)
                if terminated or truncated:
                    break
        final_time = float(info.get("time", env.t))
        full = final_time >= receipt["requested_time_range"][1] - 1e-7
        completed = full and info.get("end_reason") in {"dataset_end", "episode_time_limit"} and not info.get("failure")
        receipt.update(
            status="completed" if completed else "interrupted",
            completed_requested_duration=completed, steps=steps, final_time_s=final_time,
            final_info=info,
        )
        code = 0 if completed else 1
    except Exception as exc:
        receipt.update(status="failed", steps=steps, error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
    finally:
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                receipt["close_error"] = f"{type(exc).__name__}: {exc}"
                code = 1
        _write_json(output / "receipt.json", receipt)
    return code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--output-dir", required=True, help="New dedicated session directory.")
    parser.add_argument("--stage", choices=("sensor_only", "left_applied"), default="left_applied")
    parser.add_argument("--duration", type=float, default=9.01)
    parser.add_argument("--seed", type=int, default=123)
    args = parser.parse_args(argv)
    import yaml

    source = yaml.safe_load(SOURCE_CONFIG.read_text(encoding="utf-8"))
    kwargs = build_env_kwargs(source, setup=args.setup, output_dir=args.output_dir, stage=args.stage, duration=args.duration)
    receipt = prepare_receipt(args.setup, kwargs, args.stage)
    output = _path(args.output_dir)
    if output.exists() and any(output.iterdir()):
        parser.error(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    _write_json(output / "receipt.json", receipt)
    return run_probe(receipt, output, args.seed)


if __name__ == "__main__":
    raise SystemExit(main())
