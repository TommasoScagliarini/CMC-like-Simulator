"""Bit-exact A/B comparator and condition-matched C gate for H0/V25.

The module is intentionally independent from OpenSim and RLlib.  It consumes
only strict JSON artifacts emitted by ``run_h0_v25_abc_preflight.py`` and fails
closed on a missing field, a non-finite number, a shortened trace, or an
unregistered diagnostic exclusion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence


EXPECTED_STEPS = 500
EXPECTED_RAW_SAMPLES = 5000
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
PENETRATION_LIMIT_M = 0.025
NUMERICAL_ABS_TOLERANCE = 1.0e-6
NUMERICAL_REL_TOLERANCE = 1.0e-9
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEA_SIGNALS = (
    "torque_error_nm",
    "tau_spring_nm",
    "tau_spring_rate_nm_s",
    "motor_speed_rad_s",
    "motor_accel_rad_s2",
    "motor_power_w",
)


class H0V25GateError(RuntimeError):
    """Raised when an artifact cannot be classified safely."""


def _reject_constant(token: str) -> None:
    raise ValueError(f"non-finite JSON token {token!r}")


def strict_json_load(path: str | Path) -> Any:
    resolved = Path(path).expanduser().resolve()
    try:
        value = json.loads(
            resolved.read_text(encoding="utf-8"),
            parse_constant=_reject_constant,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise H0V25GateError(f"cannot read strict JSON {resolved}: {exc}") from exc
    canonical_json_bytes(value)
    return value


def canonical_json_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise H0V25GateError(f"value is not finite strict JSON: {exc}") from exc


def payload_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def write_json_exclusive(path: str | Path, payload: Any) -> Path:
    target = Path(path).expanduser().resolve()
    canonical_json_bytes(payload)
    target.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(target):
        raise H0V25GateError(f"refusing to clobber: {target}")
    fd, temporary_raw = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError as exc:
            raise H0V25GateError(f"refusing to clobber: {target}") from exc
        temporary.unlink()
    finally:
        if temporary.exists():
            temporary.unlink()
    return target


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise H0V25GateError(f"{label} must be an object")
    return dict(value)


def _sequence(value: Any, label: str) -> list[Any]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise H0V25GateError(f"{label} must be an array")
    return list(value)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise H0V25GateError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise H0V25GateError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise H0V25GateError(f"{label} must be finite")
    return result


def _integer(value: Any, label: str) -> int:
    number = _finite(value, label)
    result = int(round(number))
    if abs(number - result) > 1e-12:
        raise H0V25GateError(f"{label} must be integral")
    return result


def _binary_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    """Apply the sole A/B exclusion allowed by the frozen protocol."""

    return {
        str(key): item
        for key, item in value.items()
        if not str(key).startswith("binary_phase_")
    }


def _first_mismatch(left: Any, right: Any, path: str = "$") -> str | None:
    if type(left) is not type(right):
        return f"{path}: type {type(left).__name__} != {type(right).__name__}"
    if isinstance(left, Mapping):
        left_keys = list(left.keys())
        right_keys = list(right.keys())
        if left_keys != right_keys:
            return f"{path}: keys {left_keys!r} != {right_keys!r}"
        for key in left_keys:
            mismatch = _first_mismatch(left[key], right[key], f"{path}.{key}")
            if mismatch is not None:
                return mismatch
        return None
    if isinstance(left, list):
        if len(left) != len(right):
            return f"{path}: length {len(left)} != {len(right)}"
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            mismatch = _first_mismatch(
                left_item, right_item, f"{path}[{index}]"
            )
            if mismatch is not None:
                return mismatch
        return None
    if left != right:
        return f"{path}: {left!r} != {right!r}"
    return None


def common_rollout_checks(summary: Mapping[str, Any]) -> list[dict[str, Any]]:
    data = _mapping(summary, "summary")
    checks: list[tuple[str, bool, Any, Any]] = []

    def add(name: str, actual: Any, expected: Any, ok: bool) -> None:
        checks.append((name, bool(ok), actual, expected))

    steps = _integer(data.get("steps"), "steps")
    add("steps", steps, EXPECTED_STEPS, steps == EXPECTED_STEPS)
    add(
        "end_reason",
        data.get("end_reason"),
        "episode_time_limit",
        data.get("end_reason") == "episode_time_limit",
    )
    add("terminated", data.get("terminated"), False, data.get("terminated") is False)
    add("truncated", data.get("truncated"), True, data.get("truncated") is True)
    add(
        "phase_valid_cycle_count",
        data.get("phase_valid_cycle_count"),
        ">=2",
        _integer(data.get("phase_valid_cycle_count"), "phase_valid_cycle_count")
        >= 2,
    )
    penetration = _finite(
        data.get("grf_penetration_max_m"), "grf_penetration_max_m"
    )
    add(
        "grf_penetration_max_m",
        penetration,
        f"<{PENETRATION_LIMIT_M}",
        0.0 <= penetration < PENETRATION_LIMIT_M,
    )
    add(
        "action_clipped_values",
        data.get("action_clipped_values"),
        0,
        _integer(data.get("action_clipped_values"), "action_clipped_values") == 0,
    )
    add(
        "timeout_count",
        data.get("timeout_count"),
        0,
        _integer(data.get("timeout_count"), "timeout_count") == 0,
    )
    add(
        "safety_stop_count",
        data.get("safety_stop_count"),
        0,
        _integer(data.get("safety_stop_count"), "safety_stop_count") == 0,
    )
    add(
        "fallback_count",
        data.get("fallback_count"),
        0,
        _integer(data.get("fallback_count"), "fallback_count") == 0,
    )
    add(
        "hard_invalid_count",
        data.get("hard_invalid_count"),
        0,
        _integer(data.get("hard_invalid_count"), "hard_invalid_count") == 0,
    )
    add(
        "nonfinite_count",
        data.get("nonfinite_count"),
        0,
        _integer(data.get("nonfinite_count"), "nonfinite_count") == 0,
    )
    add("n_actor", data.get("n_actor"), EXPECTED_ACTOR_FEATURES, data.get("n_actor") == EXPECTED_ACTOR_FEATURES)
    add("n_observation", data.get("n_observation"), EXPECTED_FULL_FEATURES, data.get("n_observation") == EXPECTED_FULL_FEATURES)
    add("observation_dtype", data.get("observation_dtype"), "float32", data.get("observation_dtype") == "float32")
    add("morphology_weight", data.get("morphology_weight"), 0.0, _finite(data.get("morphology_weight"), "morphology_weight") == 0.0)
    for metric_name in ("reserve_norm_nm", "residual_norm_nm"):
        metric = _mapping(
            _mapping(data.get("episode_metrics"), "episode_metrics").get(metric_name),
            f"episode_metrics.{metric_name}",
        )
        add(
            f"episode_metrics.{metric_name}.sample_count",
            metric.get("sample_count"),
            EXPECTED_STEPS,
            _integer(metric.get("sample_count"), f"{metric_name}.sample_count")
            == EXPECTED_STEPS,
        )
        _finite(metric.get("rms"), f"{metric_name}.rms")
        _finite(metric.get("abs_max"), f"{metric_name}.abs_max")
    sea = _mapping(data.get("sea_episode_metrics"), "sea_episode_metrics")
    for joint in JOINTS:
        joint_data = _mapping(sea.get(joint), f"sea_episode_metrics.{joint}")
        for signal in SEA_SIGNALS:
            metric = _mapping(joint_data.get(signal), f"{joint}.{signal}")
            count = _integer(metric.get("sample_count"), f"{joint}.{signal}.sample_count")
            add(
                f"{joint}.{signal}.sample_count",
                count,
                ">0",
                count > 0,
            )
            _finite(metric.get("rms"), f"{joint}.{signal}.rms")
            _finite(metric.get("abs_max"), f"{joint}.{signal}.abs_max")
        saturation = _mapping(
            joint_data.get("tau_input_saturated"),
            f"{joint}.tau_input_saturated",
        )
        denominator = _integer(
            saturation.get("sample_count"),
            f"{joint}.tau_input_saturated.sample_count",
        )
        count = _integer(
            saturation.get("count"), f"{joint}.tau_input_saturated.count"
        )
        fraction = _finite(
            saturation.get("fraction"),
            f"{joint}.tau_input_saturated.fraction",
        )
        add(
            f"{joint}.tau_input_saturated.denominator",
            denominator,
            ">0",
            denominator > 0,
        )
        add(
            f"{joint}.tau_input_saturated.consistency",
            fraction,
            "count/sample_count",
            denominator > 0 and fraction == count / denominator,
        )
    return [
        {"name": name, "status": "PASS" if ok else "FAIL", "actual": actual, "expected": expected}
        for name, ok, actual, expected in checks
    ]


def compare_ab(
    *,
    a_trace: Sequence[Mapping[str, Any]],
    b_trace: Sequence[Mapping[str, Any]],
    a_summary: Mapping[str, Any],
    b_summary: Mapping[str, Any],
    a_journal: Mapping[str, Any],
    b_journal: Mapping[str, Any],
) -> dict[str, Any]:
    a_rows = _sequence(a_trace, "A trace")
    b_rows = _sequence(b_trace, "B trace")
    a_common = common_rollout_checks(a_summary)
    b_common = common_rollout_checks(b_summary)
    checks: list[dict[str, Any]] = [
        {
            "name": "trace_lengths_exact",
            "status": "PASS" if len(a_rows) == len(b_rows) == EXPECTED_STEPS else "FAIL",
            "actual": [len(a_rows), len(b_rows)],
            "expected": [EXPECTED_STEPS, EXPECTED_STEPS],
        }
    ]
    trace_mismatch = None
    if len(a_rows) == len(b_rows) == EXPECTED_STEPS:
        for index, (a_row, b_row) in enumerate(zip(a_rows, b_rows)):
            projected_a = _binary_projection(_mapping(a_row, f"A trace[{index}]"))
            projected_b = _binary_projection(_mapping(b_row, f"B trace[{index}]"))
            mismatch = _first_mismatch(projected_a, projected_b, f"$[{index}]")
            if mismatch is not None:
                trace_mismatch = mismatch
                break
    checks.append(
        {
            "name": "projected_trace_bit_exact",
            "status": "PASS" if trace_mismatch is None and len(a_rows) == len(b_rows) == EXPECTED_STEPS else "FAIL",
            "actual": trace_mismatch,
            "expected": None,
        }
    )
    projected_a_summary = _binary_projection(_mapping(a_summary, "A summary"))
    projected_b_summary = _binary_projection(_mapping(b_summary, "B summary"))
    summary_mismatch = _first_mismatch(
        projected_a_summary, projected_b_summary, "$.summary"
    )
    checks.append(
        {
            "name": "projected_summary_bit_exact",
            "status": "PASS" if summary_mismatch is None else "FAIL",
            "actual": summary_mismatch,
            "expected": None,
        }
    )
    a_journal_data = _mapping(a_journal, "A V25 journal")
    b_journal_data = _mapping(b_journal, "B V25 journal")
    a_samples = _sequence(a_journal_data.get("samples"), "A V25 samples")
    b_samples = _sequence(b_journal_data.get("samples"), "B V25 samples")
    journal_mismatch = _first_mismatch(a_journal_data, b_journal_data, "$.v25")
    checks.extend(
        [
            {
                "name": "v25_sample_count",
                "status": "PASS" if len(a_samples) == len(b_samples) == EXPECTED_RAW_SAMPLES else "FAIL",
                "actual": [len(a_samples), len(b_samples)],
                "expected": [EXPECTED_RAW_SAMPLES, EXPECTED_RAW_SAMPLES],
            },
            {
                "name": "v25_raw_journal_bit_exact",
                "status": "PASS" if journal_mismatch is None else "FAIL",
                "actual": journal_mismatch,
                "expected": None,
            },
        ]
    )
    passed = (
        all(item["status"] == "PASS" for item in checks)
        and all(item["status"] == "PASS" for item in a_common)
        and all(item["status"] == "PASS" for item in b_common)
    )
    return {
        "status": "PASS_AB_SHADOW_NONINTERFERENCE" if passed else "ERROR_SHADOW_NONINTERFERENCE",
        "passed": passed,
        "checks": checks,
        "a_common_checks": a_common,
        "b_common_checks": b_common,
        "a_projected_trace_sha256": payload_sha256(
            [_binary_projection(_mapping(row, "A trace row")) for row in a_rows]
        ),
        "b_projected_trace_sha256": payload_sha256(
            [_binary_projection(_mapping(row, "B trace row")) for row in b_rows]
        ),
        "a_v25_journal_sha256": payload_sha256(a_journal_data),
        "b_v25_journal_sha256": payload_sha256(b_journal_data),
    }


def _nonregression_cap(reference: float) -> float:
    return reference + max(
        NUMERICAL_ABS_TOLERANCE,
        NUMERICAL_REL_TOLERANCE * abs(reference),
    )


def gate_c(
    *,
    a_summary: Mapping[str, Any],
    c_summary: Mapping[str, Any],
) -> dict[str, Any]:
    a_data = _mapping(a_summary, "A summary")
    c_data = _mapping(c_summary, "C summary")
    common = common_rollout_checks(c_data)
    checks: list[dict[str, Any]] = []

    def compare_metric(name: str, a_value: Any, c_value: Any) -> None:
        reference = _finite(a_value, f"A {name}")
        candidate = _finite(c_value, f"C {name}")
        cap = _nonregression_cap(reference)
        checks.append(
            {
                "name": name,
                "status": "PASS" if candidate <= cap else "FAIL",
                "actual": candidate,
                "expected": f"<= {cap}",
                "reference": reference,
            }
        )

    a_episode = _mapping(a_data.get("episode_metrics"), "A episode_metrics")
    c_episode = _mapping(c_data.get("episode_metrics"), "C episode_metrics")
    for metric_name in ("reserve_norm_nm", "residual_norm_nm"):
        a_metric = _mapping(a_episode.get(metric_name), f"A {metric_name}")
        c_metric = _mapping(c_episode.get(metric_name), f"C {metric_name}")
        for aggregation in ("rms", "abs_max"):
            compare_metric(
                f"{metric_name}.{aggregation}",
                a_metric.get(aggregation),
                c_metric.get(aggregation),
            )
    a_sea = _mapping(a_data.get("sea_episode_metrics"), "A sea_episode_metrics")
    c_sea = _mapping(c_data.get("sea_episode_metrics"), "C sea_episode_metrics")
    for joint in JOINTS:
        a_joint = _mapping(a_sea.get(joint), f"A {joint}")
        c_joint = _mapping(c_sea.get(joint), f"C {joint}")
        for signal in SEA_SIGNALS:
            a_metric = _mapping(a_joint.get(signal), f"A {joint}.{signal}")
            c_metric = _mapping(c_joint.get(signal), f"C {joint}.{signal}")
            for aggregation in ("rms", "abs_max"):
                compare_metric(
                    f"{joint}.{signal}.{aggregation}",
                    a_metric.get(aggregation),
                    c_metric.get(aggregation),
                )
        a_sat = _mapping(a_joint.get("tau_input_saturated"), f"A {joint}.sat")
        c_sat = _mapping(c_joint.get("tau_input_saturated"), f"C {joint}.sat")
        a_denominator = _integer(a_sat.get("sample_count"), f"A {joint}.sat denominator")
        c_denominator = _integer(c_sat.get("sample_count"), f"C {joint}.sat denominator")
        for aggregation in ("count", "fraction"):
            reference = _finite(a_sat.get(aggregation), f"A {joint}.sat.{aggregation}")
            candidate = _finite(c_sat.get(aggregation), f"C {joint}.sat.{aggregation}")
            checks.append(
                {
                    "name": f"{joint}.tau_input_saturated.{aggregation}",
                    "status": "PASS" if candidate <= reference else "FAIL",
                    "actual": candidate,
                    "expected": f"<= {reference}",
                    "reference": reference,
                }
            )
        checks.append(
            {
                "name": f"{joint}.tau_input_saturated.same_denominator",
                "status": "PASS" if a_denominator == c_denominator else "FAIL",
                "actual": [a_denominator, c_denominator],
                "expected": "equal finite denominators",
            }
        )
    event_gate = _mapping(c_data.get("binary_phase_event_gate"), "binary_phase_event_gate")
    for field, expected in (
        ("passed", True),
        ("sample_count", EXPECTED_RAW_SAMPLES),
        ("hard_invalid_count", 0),
        ("fallback_count", 0),
        ("duplicate_event_count", 0),
        ("out_of_order_event_count", 0),
        ("left_non_v25_source_count", 0),
    ):
        actual = event_gate.get(field)
        checks.append(
            {
                "name": f"binary_phase_event_gate.{field}",
                "status": "PASS" if actual == expected else "FAIL",
                "actual": actual,
                "expected": expected,
            }
        )
    passed = all(item["status"] == "PASS" for item in common + checks)
    return {
        "status": "PASS_H0_V25_COMPATIBLE_CONDITION" if passed else "FAIL_H0_V25_COMPATIBILITY",
        "passed": passed,
        "common_checks": common,
        "nonregression_and_event_checks": checks,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--compare-ab", action="store_true")
    mode.add_argument("--gate-c", action="store_true")
    parser.add_argument("--a-dir", required=True)
    parser.add_argument("--b-dir")
    parser.add_argument("--c-dir")
    parser.add_argument("--output", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    a_dir = Path(args.a_dir).expanduser().resolve()
    if args.compare_ab:
        if not args.b_dir:
            raise SystemExit("--b-dir is required with --compare-ab")
        b_dir = Path(args.b_dir).expanduser().resolve()
        result = compare_ab(
            a_trace=strict_json_load(a_dir / "trace.json"),
            b_trace=strict_json_load(b_dir / "trace.json"),
            a_summary=strict_json_load(a_dir / "summary.json"),
            b_summary=strict_json_load(b_dir / "summary.json"),
            a_journal=strict_json_load(a_dir / "v25_raw_journal.json"),
            b_journal=strict_json_load(b_dir / "v25_raw_journal.json"),
        )
    else:
        if not args.c_dir:
            raise SystemExit("--c-dir is required with --gate-c")
        c_dir = Path(args.c_dir).expanduser().resolve()
        result = gate_c(
            a_summary=strict_json_load(a_dir / "summary.json"),
            c_summary=strict_json_load(c_dir / "summary.json"),
        )
    write_json_exclusive(args.output, result)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
