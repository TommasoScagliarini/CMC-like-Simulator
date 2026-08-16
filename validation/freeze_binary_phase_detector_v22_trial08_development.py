"""Freeze the authorized V22 development replay on already-open trial 08.

V22 does not reopen or reinterpret the terminal V21 one-shot ERROR.  It keeps
the V21 finalist geometry, V20 FSM, scorer, event oracle, and numerical gates
unchanged.  The only protocol correction is that the oracle's native global
GRF lattice need not equal the detector/IK replay lattice; every scoreable view
and event must instead be covered causally by the detector trace.

Trial 08 is explicitly OPEN DEVELOPMENT data in this cycle.  A PASS is not an
independent validation and cannot promote runtime, training, H0 execution, or
positive morphology-reward PPO.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for import_root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))


# Importing Python is code execution.  Verify these frozen helper modules before
# the first import, then verify them again as ordinary manifest sources below.
_PREIMPORT_PINNED = {
    "validation/build_canonical_grf_event_oracle.py": (
        "246c7cb326c209fe5bf732e3c5b2d3a9125b33d3cb3208378f7ded0bd7c40a89"
    ),
    "validation/freeze_binary_phase_detector_v21_trial08.py": (
        "62b5f2720e91535d1e4f3aa7f8daa0df5bcd54efc0786a74bcaa94a191f56765"
    ),
    "validation/validate_binary_phase_detector_v21_trial08_one_shot.py": (
        "d63127387db8edb2d0c75e96854f5d614ad006ccf830466142ed0f29bc47d990"
    ),
}
for _relative, _expected in _PREIMPORT_PINNED.items():
    _dependency = REPO_ROOT / _relative
    _digest = hashlib.sha256(_dependency.read_bytes()).hexdigest()
    if _digest != _expected:
        raise RuntimeError(
            f"refusing to import drifted frozen dependency: {_relative}"
        )

import build_canonical_grf_event_oracle as canonical_oracle  # noqa: E402
import freeze_binary_phase_detector_v21_trial08 as v21_freeze  # noqa: E402
import validate_binary_phase_detector_v21_trial08_one_shot as v21_gate  # noqa: E402


SCHEMA_VERSION = 22
PROTOCOL_ID = "AB06_BINARY_POINT_V22_TRIAL08_OPEN_DEVELOPMENT_COVERAGE_FIX"
CANDIDATE_ID = v21_freeze.CANDIDATE_ID
TRIAL_ID = "08"
TRIAL_ROLE = "OPEN_DEVELOPMENT_REPLAY_NOT_HOLDOUT_NOT_INDEPENDENT"

FREEZE_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v22_trial08_development_freeze_lock.json"
)
EXECUTION_LEDGER_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v22_trial08_development_execution_ledger.json"
)
OUTPUT_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v22_development_runs/"
    "2026-08-04_trial08_oracle_coverage_fix"
)

TRACE_START_S = 10.69
TRACE_END_S = 154.89
TRACE_SAMPLE_COUNT = 144201
SAMPLE_DT_S = 0.001
LEFT_SCORE_MARGIN_S = 0.080
RIGHT_OBSERVATION_MARGIN_S = 0.060

ORACLE_START_S = 10.678
ORACLE_END_S = 154.900
ORACLE_SAMPLE_COUNT = 144223
ORACLE_SHA256 = "aa0cf5a2b044bcf5faecf012e8eac5a3693a48459b9dfabc1317536288021f16"
ORACLE_CORE_SHA256 = (
    "1e596953892a64cbda8c1026b582f792c8f19b872c376292fe1e4eb26c71768c"
)
ORACLE_PATH = (
    VALIDATION_ROOT
    / "canonical_event_oracles/2026-08-03_v17_development/"
    "trial_08_canonical_event_ledger.json"
)

EXPECTED_VIEWS = (
    {
        "view_id": "plateau_01",
        "interval_s": [13.512, 45.696],
        "counts": {"complete_cycles": 25, "heel_strike": 26, "toe_off": 25},
        "view_sha256": "40132c625f983717ff0eb8d252d680f58b0d2c14f2536c751cb2aa342d631a8b",
    },
    {
        "view_id": "plateau_02",
        "interval_s": [48.363, 80.708],
        "counts": {"complete_cycles": 33, "heel_strike": 34, "toe_off": 33},
        "view_sha256": "7e5a6489841338b196a932acfe4505e12b3ef37b8eaaecd934b0db6c548c9c6a",
    },
    {
        "view_id": "plateau_03",
        "interval_s": [82.042, 115.723],
        "counts": {"complete_cycles": 38, "heel_strike": 39, "toe_off": 38},
        "view_sha256": "5064827e1c0fb8b4ef61a80d8a8bbb3b1b7af94175a4c51fda361619a510f3a1",
    },
    {
        "view_id": "plateau_04",
        "interval_s": [118.390, 150.738],
        "counts": {"complete_cycles": 30, "heel_strike": 31, "toe_off": 30},
        "view_sha256": "8e76e338f9c240b9a6ac140ab3e1c24417338fbbeb34253d800efcf24a499b0d",
    },
)

PREVIOUS_TERMINAL = {
    "freeze_lock": {
        "path": "validation/binary_phase_detector_v21_trial08_freeze_lock.json",
        "sha256": "68774248090a071221eecdd1ca771d9f598c36b3f89d2f7497ede2cdd4cc2964",
    },
    "execution_ledger": {
        "path": "validation/binary_phase_detector_v21_trial08_execution_ledger.json",
        "sha256": "37ce1f009e4f99904063ccb8d997e6bc2a8ad62d105a2ffe676164cc6eafebde",
    },
    "access_receipt": {
        "path": (
            "validation/binary_phase_detector_v21_holdout_runs/"
            "2026-08-04_trial08_one_shot/trial08_access_receipt.json"
        ),
        "sha256": "37ce1f009e4f99904063ccb8d997e6bc2a8ad62d105a2ffe676164cc6eafebde",
    },
    "failure": {
        "path": (
            "validation/binary_phase_detector_v21_holdout_runs/"
            "2026-08-04_trial08_one_shot/failure.json"
        ),
        "sha256": "fba0255fd35e25ffc53a09af64221aa7d9868129a442ed0ee915784b4f2db5f4",
    },
}

PINNED_SOURCES = {
    "validation/freeze_binary_phase_detector_v21_trial08.py": (
        "62b5f2720e91535d1e4f3aa7f8daa0df5bcd54efc0786a74bcaa94a191f56765"
    ),
    "validation/validate_binary_phase_detector_v21_trial08_one_shot.py": (
        "d63127387db8edb2d0c75e96854f5d614ad006ccf830466142ed0f29bc47d990"
    ),
    "validation/test_binary_phase_detector_v21_trial08_one_shot.py": (
        "1b89cbeec9e40d0775d5e761fe8a99fd970962f2a79446a7d6a95d9bc8a116f5"
    ),
    "validation/sweep_binary_phase_detector_v21_geometry.py": (
        "32aeb6dadba42000607e7d1e7a2480d16574b032fb0d59ad2cd5fd6558147a47"
    ),
    "validation/validate_binary_phase_fsm_v20_development.py": (
        "1ce0b82a4e9db3b2d90b4dc132de798f8eb452e784fe2492aa76ccbe10e8e431"
    ),
    "Trajectory Generator/binary_phase_fsm.py": (
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1"
    ),
    "binary_phase_detector.py": (
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6"
    ),
    "validation/build_canonical_grf_event_oracle.py": (
        "246c7cb326c209fe5bf732e3c5b2d3a9125b33d3cb3208378f7ded0bd7c40a89"
    ),
    "kinematics_interpolator.py": (
        "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede"
    ),
    "config.py": (
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3"
    ),
    "model_loader.py": (
        "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76"
    ),
}

DYNAMIC_SOURCES = (
    "validation/freeze_binary_phase_detector_v22_trial08_development.py",
    "validation/validate_binary_phase_detector_v22_trial08_development.py",
    "validation/test_binary_phase_detector_v22_trial08_development.py",
)

POST_PASS_SCOPE = {
    "development_candidate_freeze_allowed": True,
    "h0_integration_implementation_allowed": True,
    "h0_execution_allowed": False,
    "development_candidate_h0_ready_allowed": False,
    "independent_validation_claim_allowed": False,
    "protected_trial_access_allowed": False,
    "runtime_promotion_allowed": False,
    "training_promotion_allowed": False,
    "positive_morphology_reward_ppo_allowed": False,
    "next_required_work": [
        "freeze_development_candidate_bundle",
        "attest_marker_to_runtime_model_geometry_compatibility",
        "implement_dormant_v21_h0_routing",
    ],
}


class V22FreezeError(RuntimeError):
    """Raised when the corrected development contract cannot be frozen."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise V22FreezeError(f"source is missing: {path}")
    return {
        "path": path.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(path),
        "size_bytes": int(path.stat().st_size),
    }


def _strict_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    return v21_gate._strict_json(path, expected_sha256=expected_sha256)


def validate_oracle_trace_coverage(
    core: Mapping[str, Any],
    *,
    trace_start_s: float = TRACE_START_S,
    trace_end_s: float = TRACE_END_S,
    trace_sample_count: int = TRACE_SAMPLE_COUNT,
    sample_dt_s: float = SAMPLE_DT_S,
) -> dict[str, Any]:
    """Prove causal scoreability without equating the two global lattices."""

    trace_start = float(trace_start_s)
    trace_end = float(trace_end_s)
    dt = float(sample_dt_s)
    if not all(math.isfinite(value) for value in (trace_start, trace_end, dt)):
        raise V22FreezeError("trace coverage parameters must be finite")
    if trace_start >= trace_end or dt <= 0.0:
        raise V22FreezeError("trace coverage interval is invalid")
    oracle_grid = core.get("time_grid")
    if not isinstance(oracle_grid, Mapping):
        raise V22FreezeError("oracle time grid is missing")
    oracle_start = float(oracle_grid.get("start_s", math.nan))
    oracle_end = float(oracle_grid.get("end_s", math.nan))
    oracle_count = int(oracle_grid.get("sample_count", -1))
    trace_count = int(trace_sample_count)
    try:
        decimal_dt = Decimal(str(dt))
        decimal_oracle_start = Decimal(str(oracle_start))
        decimal_oracle_end = Decimal(str(oracle_end))
        decimal_trace_start = Decimal(str(trace_start))
        decimal_trace_end = Decimal(str(trace_end))
        oracle_span_samples = (
            decimal_oracle_end - decimal_oracle_start
        ) / decimal_dt
        trace_span_samples = (
            decimal_trace_end - decimal_trace_start
        ) / decimal_dt
        left_decimal = (
            decimal_trace_start - decimal_oracle_start
        ) / decimal_dt
        right_decimal = (
            decimal_oracle_end - decimal_trace_end
        ) / decimal_dt
    except (InvalidOperation, ZeroDivisionError) as exc:
        raise V22FreezeError("oracle/trace decimal lattice is invalid") from exc
    oracle_span_integral = (
        oracle_span_samples == oracle_span_samples.to_integral_value()
    )
    trace_span_integral = trace_span_samples == trace_span_samples.to_integral_value()
    left_integral = left_decimal == left_decimal.to_integral_value()
    right_integral = right_decimal == right_decimal.to_integral_value()
    expected_oracle_count = (
        int(oracle_span_samples) + 1 if oracle_span_integral else -1
    )
    expected_trace_count = (
        int(trace_span_samples) + 1 if trace_span_integral else -1
    )
    left_samples = int(left_decimal) if left_integral else -1
    right_samples = int(right_decimal) if right_integral else -1
    subgrid_checks = {
        "finite": all(
            math.isfinite(value)
            for value in (oracle_start, oracle_end, trace_start, trace_end, dt)
        ),
        "oracle_count_identity": oracle_count == expected_oracle_count,
        "trace_count_identity": trace_count == expected_trace_count,
        "frozen_trace_count": trace_count == TRACE_SAMPLE_COUNT,
        "oracle_contains_trace": oracle_start <= trace_start <= trace_end <= oracle_end,
        "oracle_span_decimal_aligned": oracle_span_integral,
        "trace_span_decimal_aligned": trace_span_integral,
        "left_margin_decimal_aligned": left_integral,
        "right_margin_decimal_aligned": right_integral,
        "margins_nonnegative": left_samples >= 0 and right_samples >= 0,
        "contiguous_count_identity": trace_count
        == oracle_count - left_samples - right_samples,
        "trial08_left_margin_exact": left_samples == 12,
        "trial08_right_margin_exact": right_samples == 10,
        "trace_start_is_oracle_index": abs(
            oracle_start + left_samples * dt - trace_start
        )
        <= 1e-12,
        "trace_end_is_oracle_index": abs(
            oracle_start + (oracle_count - right_samples - 1) * dt - trace_end
        )
        <= 1e-12,
    }
    if not all(subgrid_checks.values()):
        raise V22FreezeError(f"oracle/trace subgrid contract failed: {subgrid_checks}")
    views = core.get("views")
    if not isinstance(views, list) or len(views) != len(EXPECTED_VIEWS):
        raise V22FreezeError("oracle must expose the four frozen views")
    records: list[dict[str, Any]] = []
    for view, expected in zip(views, EXPECTED_VIEWS):
        if not isinstance(view, Mapping):
            raise V22FreezeError("oracle view must be an object")
        observed = {
            "view_id": view.get("view_id"),
            "interval_s": view.get("interval_s"),
            "counts": view.get("counts"),
            "view_sha256": view.get("view_sha256"),
        }
        if observed != expected:
            raise V22FreezeError(f"oracle view drifted: {observed}")
        start, end = (float(value) for value in view["interval_s"])
        coverage_checks = {
            "left_scoring_margin": trace_start
            <= start - LEFT_SCORE_MARGIN_S + 1e-12,
            "right_observation_margin": trace_end
            >= end + RIGHT_OBSERVATION_MARGIN_S - 1e-12,
            "view_start_on_detector_lattice": abs(
                (start - trace_start) / dt - round((start - trace_start) / dt)
            )
            <= 1e-9,
            "view_end_on_detector_lattice": abs(
                (end - trace_start) / dt - round((end - trace_start) / dt)
            )
            <= 1e-9,
            "inclusive_view_sample_count": int(round((end - start) / dt)) + 1
            > 1,
        }
        event_count = 0
        prior_event_time = -math.inf
        for event in view.get("scoreable_events", []):
            event_time = float(event.get("event_time_s", math.nan))
            event_name = str(event.get("event", ""))
            confirmed_tolerance = 0.050 if event_name == "heel_strike" else 0.080
            delivered_tolerance = 0.060 if event_name == "heel_strike" else 0.090
            oracle_sample_index = int(event.get("sample_index", -1))
            trace_index_float = (event_time - trace_start) / dt
            trace_index = int(round(trace_index_float))
            event_checks = {
                "finite": math.isfinite(event_time),
                "known_event": event_name in {"heel_strike", "toe_off"},
                "ordered": event_time >= prior_event_time - 1e-12,
                "covered": trace_start - 1e-12
                <= event_time
                <= trace_end + 1e-12,
                "on_detector_lattice": abs(
                    trace_index_float - trace_index
                )
                <= 1e-9,
                "oracle_to_trace_index_exact": trace_index
                == oracle_sample_index - left_samples,
                "absolute_timestamp_reconstructed": abs(
                    trace_start + trace_index * dt - event_time
                )
                <= 1e-12,
                "full_left_matching_window_observable": trace_start
                <= event_time - confirmed_tolerance - 0.005 - dt + 1e-12,
                "full_right_delivery_window_observable": trace_end
                >= event_time + delivered_tolerance - 1e-12,
                "view_local_left_window_observable": start
                <= event_time - confirmed_tolerance - 0.005 - dt + 1e-12,
                "view_local_right_window_observable": end
                >= event_time + delivered_tolerance - 1e-12,
            }
            if event.get("event") == "heel_strike":
                persistence = float(
                    event.get("persistence_confirmed_time_s", math.nan)
                )
                event_checks["oracle_persistence_covered"] = (
                    math.isfinite(persistence)
                    and event_time <= persistence <= trace_end + 1e-12
                )
            if not all(event_checks.values()):
                raise V22FreezeError(
                    f"scoreable event is not covered by detector trace: {event_checks}"
                )
            event_count += 1
            prior_event_time = event_time
        closing_hs = max(
            float(event["event_time_s"])
            for event in view.get("scoreable_events", [])
            if event.get("event") == "heel_strike"
        )
        coverage_checks["closing_hs_observation_margin"] = (
            end >= closing_hs + RIGHT_OBSERVATION_MARGIN_S - 1e-12
        )
        if event_count != int(view["counts"]["heel_strike"]) + int(
            view["counts"]["toe_off"]
        ):
            raise V22FreezeError("scoreable event count drifted")
        if not all(coverage_checks.values()):
            raise V22FreezeError(
                f"detector trace does not cover {view['view_id']}: {coverage_checks}"
            )
        records.append(
            {
                "view_id": str(view["view_id"]),
                "interval_s": [start, end],
                "scoreable_event_count": event_count,
                "checks": coverage_checks,
                "pass": True,
            }
        )
    return {
        "pass": True,
        "oracle_native_grid": dict(oracle_grid),
        "detector_trace_grid": {
            "start_s": trace_start,
            "end_s": trace_end,
            "sample_count": trace_count,
            "sample_dt_s": dt,
        },
        "global_grid_equality_required": False,
        "exact_contiguous_subgrid": {
            "left_margin_samples": left_samples,
            "right_margin_samples": right_samples,
            "checks": subgrid_checks,
        },
        "consumer_rethresholds_grf": False,
        "views": records,
    }


def _verify_previous_terminal() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for label, expected in PREVIOUS_TERMINAL.items():
        path = REPO_ROOT / str(expected["path"])
        record = _source_record(path)
        if record["sha256"] != expected["sha256"]:
            raise V22FreezeError(f"previous terminal artifact drifted: {label}")
        records[label] = record
    ledger = _strict_json(
        REPO_ROOT / PREVIOUS_TERMINAL["execution_ledger"]["path"],
        PREVIOUS_TERMINAL["execution_ledger"]["sha256"],
    )
    receipt_path = REPO_ROOT / PREVIOUS_TERMINAL["access_receipt"]["path"]
    if (REPO_ROOT / PREVIOUS_TERMINAL["execution_ledger"]["path"]).read_bytes() != (
        receipt_path.read_bytes()
    ):
        raise V22FreezeError("previous ledger and receipt are not byte-identical")
    failure = _strict_json(
        REPO_ROOT / PREVIOUS_TERMINAL["failure"]["path"],
        PREVIOUS_TERMINAL["failure"]["sha256"],
    )
    checks = {
        "ledger_consumed": ledger.get("stage_consumed") is True,
        "ledger_no_retry": ledger.get("rerun_allowed") is False,
        "failure_status": failure.get("status")
        == "ERROR_INTERNAL_V21_TRIAL08_CONSUMED",
        "failure_consumed": failure.get("stage_consumed") is True,
        "failure_no_retry": failure.get("rerun_allowed") is False,
        "no_detector_trace_sampled": failure.get("execution_audit", {}).get("stage")
        == "trial08_replay_inputs_verified",
        "protected_unopened": failure.get("protected_trials_opened") == [],
        "reserve_unopened": failure.get("reserve_trials_opened") == [],
        "error_is_grid_overconstraint": "'grid_start': False"
        in str(failure.get("exception", ""))
        and "'grid_end': False" in str(failure.get("exception", ""))
        and "'grid_count': False" in str(failure.get("exception", "")),
    }
    old_dir = receipt_path.parent
    checks["no_old_success_artifacts"] = not any(
        os.path.lexists(os.fspath(old_dir / name))
        for name in (
            "manifest.json",
            "trial08_decision_lock.json",
            "packed_binary_trace.json",
        )
    )
    if not all(checks.values()):
        raise V22FreezeError(f"previous V21 terminal state drifted: {checks}")
    return {"checks": checks, "artifacts": records}


def _verify_oracle() -> dict[str, Any]:
    ledger = _strict_json(ORACLE_PATH, ORACLE_SHA256)
    canonical_oracle.validate_ledger(ledger)
    core = ledger["scientific_core"]
    views = core.get("views")
    view_integrity = v21_gate._validate_oracle_views(
        core,
        views,
        minimum_cycles=int(v21_freeze.FROZEN_GATE["minimum_complete_cycles_per_view"]),
    )
    checks = {
        "trial": core.get("trial_id") == TRIAL_ID,
        "core_sha": ledger.get("scientific_core_sha256") == ORACLE_CORE_SHA256,
        "grid_start": float(core["time_grid"]["start_s"]) == ORACLE_START_S,
        "grid_end": float(core["time_grid"]["end_s"]) == ORACLE_END_S,
        "grid_count": int(core["time_grid"]["sample_count"])
        == ORACLE_SAMPLE_COUNT,
        "sample_dt": float(core["sample_dt_s"]) == SAMPLE_DT_S,
        "threshold": float(core["threshold_n"]) == 20.0,
        "persistence": float(core["min_contact_duration_s"]) == 0.050,
        "min_cycle": float(core["min_cycle_duration_s"]) == 0.30,
        "view_integrity": view_integrity["pass"],
    }
    if not all(checks.values()):
        raise V22FreezeError(f"oracle contract drifted: {checks}")
    coverage = validate_oracle_trace_coverage(core)
    return {
        "checks": checks,
        "coverage": coverage,
        "source": _source_record(ORACLE_PATH),
        "scientific_core_sha256": ORACLE_CORE_SHA256,
    }


def _verify_sources() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative, expected_sha in PINNED_SOURCES.items():
        record = _source_record(REPO_ROOT / relative)
        if record["sha256"] != expected_sha:
            raise V22FreezeError(f"pinned source drifted: {relative}")
        records[relative] = record
    for relative in DYNAMIC_SOURCES:
        records[relative] = _source_record(REPO_ROOT / relative)
    return records


def build_freeze_payload(*, check_destinations: bool = False) -> dict[str, Any]:
    if check_destinations:
        if any(
            os.path.lexists(os.fspath(path))
            for path in (FREEZE_PATH, EXECUTION_LEDGER_PATH, OUTPUT_DIR)
        ):
            raise V22FreezeError("V22 freeze or execution destination is occupied")
    previous = _verify_previous_terminal()
    oracle = _verify_oracle()
    sources = _verify_sources()
    v21_profile = _source_record(v21_freeze.PROFILE_PATH)
    if v21_profile["sha256"] != v21_freeze.PROFILE_SHA256:
        raise V22FreezeError("V21 finalist profile drifted")
    declarations = json.loads(
        json.dumps(v21_freeze.TRIAL08_DECLARATIONS, allow_nan=False)
    )
    payload = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "freeze_date": "2026-08-04",
        "status": "V22_OPEN_DEVELOPMENT_FROZEN_BEFORE_DETECTOR_REPLAY",
        "authorization": {
            "kind": "explicit_user_authorized_new_development_cycle_after_v21_error",
            "date": "2026-08-04",
            "request": "procedi con la soluzione proposta",
            "previous_error_reclassified": False,
            "previous_error_artifacts_mutable": False,
        },
        "candidate": {
            "candidate_id": CANDIDATE_ID,
            "profile": v21_profile,
            "geometry": json.loads(
                json.dumps(v21_freeze.CANDIDATE_GEOMETRY, allow_nan=False)
            ),
            "geometry_sha256": v21_freeze.canonical_sha256(
                v21_freeze.CANDIDATE_GEOMETRY
            ),
            "fsm": sources["Trajectory Generator/binary_phase_fsm.py"],
            "fsm_current_implementation_contract_id": (
                "binary_point_v19+functional_contact_fsm_v1_shadow"
            ),
            "target_bundle_contract_id_after_h0_integration": (
                "primary_grf_split_v1+binary_point_v21+functional_contact_fsm_v1"
            ),
            "sample_dt_s": SAMPLE_DT_S,
            "debounce_s": 0.005,
            "policy_step_s": 0.010,
            "candidate_count": 1,
        },
        "trial": {
            "trial_id": TRIAL_ID,
            "role": TRIAL_ROLE,
            "scientifically_virgin": False,
            "holdout": False,
            "independent_validation": False,
            "already_open_before_v22": True,
            "trial08_previously_opened": True,
            "claim_limit": (
                "development robustness evidence for frozen V21 geometry and "
                "V20 FSM on trials 02/04/08; not independent validation"
            ),
            "trace_interval_s": [TRACE_START_S, TRACE_END_S],
            "trace_sample_count": TRACE_SAMPLE_COUNT,
            "declared_inputs": declarations,
        },
        "correction": {
            "changed_invariant_only": (
                "replace global oracle-grid equality with causal coverage of all "
                "scoreable views by the detector trace"
            ),
            "oracle_native_grid": {
                "start_s": ORACLE_START_S,
                "end_s": ORACLE_END_S,
                "sample_count": ORACLE_SAMPLE_COUNT,
                "sample_dt_s": SAMPLE_DT_S,
            },
            "detector_trace_grid": {
                "start_s": TRACE_START_S,
                "end_s": TRACE_END_S,
                "sample_count": TRACE_SAMPLE_COUNT,
                "sample_dt_s": SAMPLE_DT_S,
            },
            "global_grid_equality_required": False,
            "view_coverage_required": True,
            "left_score_margin_s": LEFT_SCORE_MARGIN_S,
            "right_observation_margin_s": RIGHT_OBSERVATION_MARGIN_S,
            "geometry_changed": False,
            "fsm_changed": False,
            "scorer_or_numerical_gate_changed": False,
            "oracle_reconstructed_or_rethresholded": False,
        },
        "gate": json.loads(
            json.dumps(v21_freeze.FROZEN_GATE, allow_nan=False)
        ),
        "oracle": oracle,
        "previous_v21_terminal_error": previous,
        "opening_contract": {
            "global_execution_ledger": EXECUTION_LEDGER_PATH.relative_to(
                REPO_ROOT
            ).as_posix(),
            "output_dir": OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
            "ledger_and_receipt_before_detector_replay": True,
            "single_execution_in_this_development_cycle": True,
            "retry_after_terminal_result_allowed": False,
            "geometry_fsm_or_gate_retuning_allowed": False,
        },
        "post_pass_scope": json.loads(
            json.dumps(POST_PASS_SCOPE, allow_nan=False)
        ),
        "data_governance": {
            "opened_development_trial": [TRIAL_ID],
            "protected_trials_opened": [],
            "protected_trials_remaining_closed": ["05", "06"],
            "reserve_trials_opened": [],
            "reserve_trials_remaining_closed": ["03", "07"],
            "raw_prescribed_grf_required": False,
            "external_loads_required": False,
        },
        "sources": sources,
        "non_actions": {
            "primary_grf_modification_allowed": False,
            "cpp_or_contact_modification_allowed": False,
            "sea_semantics_modification_allowed": False,
            "h0_execution_allowed": False,
            "training_allowed": False,
            "protected_or_reserve_access_allowed": False,
        },
    }
    json.dumps(payload, allow_nan=False)
    return payload


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--freeze", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.freeze:
            payload = build_freeze_payload(check_destinations=True)
            v21_gate._write_json_exclusive(FREEZE_PATH, payload)
            existing_verified = False
        elif FREEZE_PATH.is_file():
            payload = build_freeze_payload(check_destinations=False)
            existing = _strict_json(FREEZE_PATH, sha256_file(FREEZE_PATH))
            if existing != payload:
                raise V22FreezeError("existing V22 freeze differs from expected")
            existing_verified = True
        else:
            payload = build_freeze_payload(check_destinations=True)
            existing_verified = False
    except Exception as exc:
        print(
            f"V22 development freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(
        json.dumps(
            {
                "status": payload["status"],
                "candidate_id": CANDIDATE_ID,
                "trial_role": TRIAL_ROLE,
                "coverage_pass": payload["oracle"]["coverage"]["pass"],
                "existing_freeze_verified": existing_verified,
                "freeze_path": FREEZE_PATH.relative_to(REPO_ROOT).as_posix()
                if args.freeze
                else None,
            },
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
