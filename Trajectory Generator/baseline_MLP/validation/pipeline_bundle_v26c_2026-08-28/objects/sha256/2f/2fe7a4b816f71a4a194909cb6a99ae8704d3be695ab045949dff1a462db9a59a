"""Validate the onlineGRF contract used by training and inference."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
import win_runtime  # noqa: E402,F401
import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import env_factory  # noqa: E402
import numpy as np  # noqa: E402
from process_watchdog import write_heartbeat  # noqa: E402

_DEFAULT_SETUP_XML = (
    r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
)
_DEFAULT_ACCEPTANCE_REPORT = (
    "results/online_grf_acceptance_physical_basis_10mm_balanced_500ms.json"
)
_ONLINE_FEATURE_SUFFIXES = (
    "normal_grf_bw",
    "in_contact",
    "heel_strike",
    "toe_off",
    "gait_phase",
    "cycle_duration_s",
)


def _repo_path(value: str | Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = _bootstrap.REPO_ROOT / path
    return path.resolve()


def _acceptance_gate(
    mode: str,
    report_path: Path,
    requested_profile: Path,
) -> dict[str, Any]:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    accepted_sensor_profile_raw = report.get("sources", {}).get("sensor_input_profile")
    accepted_sensor_profile = (
        _repo_path(accepted_sensor_profile_raw)
        if accepted_sensor_profile_raw
        else None
    )
    profile_matches = accepted_sensor_profile == requested_profile
    criteria = report.get("criteria", [])
    sensor_criteria = [
        item
        for item in criteria
        if str(item.get("name", "")).startswith(("plugin.", "sensor."))
    ]
    sensor_failed = [item for item in sensor_criteria if not item.get("pass")]
    if mode == "online_sensor":
        passed = bool(sensor_criteria) and not sensor_failed and profile_matches
        reason = (
            "requested profile matches the physically plausible sensor report"
            if passed
            else "profile mismatch or plugin/sensor holdout criterion failed"
        )
    else:
        passed = report.get("verdict") == "PASS" and profile_matches
        reason = (
            "full active physical acceptance passed"
            if passed
            else "full active physical acceptance failed"
        )
    return {
        "pass": passed,
        "reason": reason,
        "acceptance_report": str(report_path),
        "acceptance_verdict": report.get("verdict"),
        "requested_profile": str(requested_profile),
        "accepted_sensor_profile": (
            str(accepted_sensor_profile) if accepted_sensor_profile else None
        ),
        "profile_matches": profile_matches,
        "sensor_criteria_count": len(sensor_criteria),
        "sensor_failed": sensor_failed,
    }


def _assert_finite_array(name: str, value: Any) -> None:
    array = np.asarray(value)
    if not np.all(np.isfinite(array)):
        raise RuntimeError(f"{name} contains non-finite values")


def _validate_online_payload(info: dict[str, Any], mode: str) -> None:
    if info.get("grf_mode") != mode:
        raise RuntimeError(
            f"GRF mode mismatch: expected {mode!r}, got {info.get('grf_mode')!r}"
        )
    gait = info.get("online_gait")
    if not isinstance(gait, dict) or not gait.get("available"):
        raise RuntimeError("online_gait payload is unavailable")
    for side in ("left", "right"):
        side_info = gait.get("sides", {}).get(side)
        if not isinstance(side_info, dict):
            raise RuntimeError(f"Missing online_gait payload for {side}")
        for key in ("normal_force_n", "normal_force_bw", "gait_phase", "cycle_duration_s"):
            value = float(side_info[key])
            if not math.isfinite(value):
                raise RuntimeError(f"Non-finite online_gait {side}.{key}")
        if float(side_info["normal_force_n"]) < -1e-9:
            raise RuntimeError(f"Negative online normal force for {side}")


def _run_probe(
    *,
    label: str,
    action_kind: str,
    args: argparse.Namespace,
    heartbeat_path: Path,
) -> dict[str, Any]:
    write_heartbeat(
        heartbeat_path,
        f"{label}: build environment",
        progress=0,
        timeout_s=args.phase_timeout_s,
    )
    env = env_factory.make_cmc_env(
        {
            "setup_xml_path": args.setup_xml,
            "segment_duration": args.segment_duration,
            "episode_duration": args.episode_duration,
            "policy_knots": args.policy_knots,
            "action_mode": "delta",
            "max_delta_rad": args.max_delta_rad,
            "random_init": False,
            "fail_fast": True,
            "record_outputs": False,
            "grf_mode": args.grf_mode,
            "online_grf_profile_file": args.online_grf_profile,
            "include_online_grf_observation": True,
        }
    )
    rng = np.random.default_rng(args.seed)
    step_durations: list[float] = []
    event_counts = {"heel_strike": 0, "toe_off": 0}
    terminated = truncated = False
    end_reason = None
    try:
        write_heartbeat(
            heartbeat_path,
            f"{label}: reset",
            progress=0,
            timeout_s=args.phase_timeout_s,
        )
        obs, info = env.reset(seed=args.seed)
        _assert_finite_array(f"{label} reset observation", obs)
        _validate_online_payload(info, args.grf_mode)
        names = tuple(info.get("observation_feature_names", ()))
        missing_features = [
            f"online_{side}_{suffix}"
            for side in ("left", "right")
            for suffix in _ONLINE_FEATURE_SUFFIXES
            if f"online_{side}_{suffix}" not in names
        ]
        if missing_features:
            raise RuntimeError(f"Missing online observation features: {missing_features}")

        steps = 0
        for step_index in range(args.max_steps):
            if action_kind == "random":
                action = rng.uniform(env.action_space.low, env.action_space.high).astype(
                    np.float32
                )
            else:
                action = np.zeros(env.action_space.shape, dtype=np.float32)
            _assert_finite_array(f"{label} action", action)
            write_heartbeat(
                heartbeat_path,
                f"{label}: simulator step {step_index + 1}",
                progress=step_index,
                timeout_s=args.phase_timeout_s,
            )
            started = time.perf_counter()
            obs, reward, terminated, truncated, info = env.step(action)
            step_durations.append(time.perf_counter() - started)
            steps += 1
            _assert_finite_array(f"{label} observation step {steps}", obs)
            if not math.isfinite(float(reward)):
                raise RuntimeError(f"{label} reward is non-finite at step {steps}")
            _validate_online_payload(info, args.grf_mode)
            for event in info.get("online_events", []):
                event_name = str(event.get("event", ""))
                if event_name in event_counts:
                    event_counts[event_name] += 1
            write_heartbeat(
                heartbeat_path,
                f"{label}: completed simulator step {steps}",
                progress=steps,
                timeout_s=args.phase_timeout_s,
            )
            end_reason = info.get("end_reason")
            if terminated or truncated:
                break
    finally:
        write_heartbeat(
            heartbeat_path,
            f"{label}: close",
            progress=len(step_durations),
            timeout_s=args.phase_timeout_s,
        )
        env.close()

    return {
        "label": label,
        "action_kind": action_kind,
        "steps": len(step_durations),
        "step_wall_time_s": step_durations,
        "step_wall_time_max_s": max(step_durations, default=None),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": end_reason,
        "event_counts": event_counts,
        "observation_feature_count": len(names),
        "online_observation_features": [
            name for name in names if name.startswith("online_")
        ],
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    report_path = _repo_path(args.report)
    heartbeat_path = _repo_path(args.heartbeat_file)
    acceptance_path = _repo_path(args.acceptance_report)
    requested_profile = _repo_path(args.online_grf_profile)
    write_heartbeat(
        heartbeat_path,
        "acceptance gate",
        progress=0,
        timeout_s=args.phase_timeout_s,
    )
    gate = _acceptance_gate(args.grf_mode, acceptance_path, requested_profile)
    if not gate["pass"]:
        report = {
            "verdict": "BLOCKED",
            "reason": gate["reason"],
            "grf_mode": args.grf_mode,
            "online_grf_profile_file": str(_repo_path(args.online_grf_profile)),
            "gate": gate,
            "probes": [],
        }
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        write_heartbeat(heartbeat_path, "blocked by acceptance gate", progress=0)
        return report

    probes = [
        _run_probe(
            label="training contract",
            action_kind="random",
            args=args,
            heartbeat_path=heartbeat_path,
        ),
        _run_probe(
            label="inference contract",
            action_kind="zero",
            args=args,
            heartbeat_path=heartbeat_path,
        ),
    ]
    report = {
        "verdict": "PASS",
        "grf_mode": args.grf_mode,
        "setup_xml_path": str(_repo_path(args.setup_xml)),
        "online_grf_profile_file": str(_repo_path(args.online_grf_profile)),
        "gate": gate,
        "probes": probes,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    write_heartbeat(heartbeat_path, "complete", progress=sum(p["steps"] for p in probes))
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-xml", default=_DEFAULT_SETUP_XML)
    parser.add_argument(
        "--grf-mode",
        choices=("online_sensor", "online"),
        default=env_factory.DEFAULT_NETWORK_GRF_MODE,
    )
    parser.add_argument(
        "--online-grf-profile",
        default=env_factory.DEFAULT_NETWORK_ONLINE_GRF_PROFILE,
    )
    parser.add_argument("--acceptance-report", default=_DEFAULT_ACCEPTANCE_REPORT)
    parser.add_argument(
        "--report",
        default="results/online_grf_training_inference_validation.json",
    )
    parser.add_argument(
        "--heartbeat-file",
        default="results/online_grf_training_inference_watchdog.json",
    )
    parser.add_argument("--segment-duration", type=float, default=0.02)
    parser.add_argument("--episode-duration", type=float, default=0.08)
    parser.add_argument("--policy-knots", type=int, default=3)
    parser.add_argument("--max-delta-rad", type=float, default=0.35)
    parser.add_argument("--max-steps", type=int, default=4)
    parser.add_argument("--phase-timeout-s", type=float, default=120.0)
    parser.add_argument("--seed", type=int, default=123)
    return parser.parse_args()


def main() -> int:
    report = run(parse_args())
    print(json.dumps(report, indent=2))
    return 0 if report["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
