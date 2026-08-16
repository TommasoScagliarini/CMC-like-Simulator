"""Freeze the explicitly authorized V23 open-development replay on trial 08.

V23 preserves the V21 finalist geometry, V20 FSM, canonical event oracle,
scorer, and numerical gates.  It does not reopen or reinterpret the terminal
V21 or V22 cycles.  Its only protocol correction is to keep descriptive facts
separate from boolean assertions when binding the oracle to the detector trace.

In particular, ``global_grid_equality_required = False`` is a frozen fact, not
an assertion that must evaluate to true.  Every entry in ``assertions`` is a
positive invariant and is therefore safe to aggregate with ``all(...)``.

Trial 08 remains already-open development data.  A V23 PASS is neither an
independent validation nor authorization for H0 execution, protected data,
runtime promotion, training, corridor activation, or PPO.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for import_root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))


# Importing Python executes code.  Verify the immutable V22 helper first; V22
# performs its own pre-import verification of the V21/oracle dependencies.
_V22_FREEZE_SOURCE = (
    VALIDATION_ROOT / "freeze_binary_phase_detector_v22_trial08_development.py"
)
_V22_FREEZE_SOURCE_SHA256 = (
    "fa7b16c692186b25bd6929c096e5d9df9a24979082d457c3516a9f1e7a5223fc"
)
if hashlib.sha256(_V22_FREEZE_SOURCE.read_bytes()).hexdigest() != (
    _V22_FREEZE_SOURCE_SHA256
):
    raise RuntimeError("refusing to import drifted frozen V22 coverage helper")

import freeze_binary_phase_detector_v22_trial08_development as v22_freeze  # noqa: E402


v21_freeze = v22_freeze.v21_freeze
v21_gate = v22_freeze.v21_gate
canonical_oracle = v22_freeze.canonical_oracle

SCHEMA_VERSION = 23
PROTOCOL_ID = "AB06_BINARY_POINT_V23_TRIAL08_OPEN_DEVELOPMENT_ASSERTIONS_FACTS_FIX"
CANDIDATE_ID = v21_freeze.CANDIDATE_ID
TRIAL_ID = "08"
TRIAL_ROLE = "OPEN_DEVELOPMENT_REPLAY_NOT_HOLDOUT_NOT_INDEPENDENT"

FREEZE_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v23_trial08_development_freeze_lock.json"
)
EXECUTION_LEDGER_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v23_trial08_development_execution_ledger.json"
)
OUTPUT_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v23_development_runs/"
    "2026-08-04_trial08_assertions_facts_fix"
)

TRACE_START_S = v22_freeze.TRACE_START_S
TRACE_END_S = v22_freeze.TRACE_END_S
TRACE_SAMPLE_COUNT = v22_freeze.TRACE_SAMPLE_COUNT
SAMPLE_DT_S = v22_freeze.SAMPLE_DT_S
ORACLE_START_S = v22_freeze.ORACLE_START_S
ORACLE_END_S = v22_freeze.ORACLE_END_S
ORACLE_SAMPLE_COUNT = v22_freeze.ORACLE_SAMPLE_COUNT
ORACLE_SHA256 = v22_freeze.ORACLE_SHA256
ORACLE_CORE_SHA256 = v22_freeze.ORACLE_CORE_SHA256
ORACLE_PATH = v22_freeze.ORACLE_PATH
EXPECTED_VIEWS = v22_freeze.EXPECTED_VIEWS


V21_TERMINAL = copy.deepcopy(v22_freeze.PREVIOUS_TERMINAL)
V22_TERMINAL = {
    "freeze_lock": {
        "path": "validation/binary_phase_detector_v22_trial08_development_freeze_lock.json",
        "sha256": "1a909f6a48aa5fece3521144e6e0829199d7c7d3831f6d355bdf176608926077",
    },
    "execution_ledger": {
        "path": "validation/binary_phase_detector_v22_trial08_development_execution_ledger.json",
        "sha256": "abf074400dc2fb112e9375ce26d89eb55eacea16d348789c73f876cb1004a917",
    },
    "access_receipt": {
        "path": (
            "validation/binary_phase_detector_v22_development_runs/"
            "2026-08-04_trial08_oracle_coverage_fix/"
            "trial08_development_access_receipt.json"
        ),
        "sha256": "abf074400dc2fb112e9375ce26d89eb55eacea16d348789c73f876cb1004a917",
    },
    "failure": {
        "path": (
            "validation/binary_phase_detector_v22_development_runs/"
            "2026-08-04_trial08_oracle_coverage_fix/failure.json"
        ),
        "sha256": "19bd2edfd0a9ffca113308fc84845c838caf54fc4a21b645ec2e351fd4ea8898",
    },
}


PINNED_SOURCES = {
    **v22_freeze.PINNED_SOURCES,
    "validation/freeze_binary_phase_detector_v22_trial08_development.py": (
        _V22_FREEZE_SOURCE_SHA256
    ),
    "validation/validate_binary_phase_detector_v22_trial08_development.py": (
        "dc93ba69b03d1c8ed970440d5bc4342fce005aaa43c09e5f6c3d0970641f3d77"
    ),
    "validation/test_binary_phase_detector_v22_trial08_development.py": (
        "5f720da9b52f5bc6bc4772bafa9c245dcc01cbbf27be125301133b7a63af639d"
    ),
}

DYNAMIC_SOURCES = (
    "validation/freeze_binary_phase_detector_v23_trial08_development.py",
    "validation/validate_binary_phase_detector_v23_trial08_development.py",
    "validation/test_binary_phase_detector_v23_trial08_development.py",
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
    "corridor_activation_allowed": False,
    "positive_morphology_reward_ppo_allowed": False,
    "next_required_work": [
        "freeze_development_candidate_bundle",
        "attest_marker_to_runtime_model_geometry_compatibility",
        "implement_dormant_v21_h0_routing",
    ],
}


class V23FreezeError(RuntimeError):
    """Raised when the V23 development contract cannot be frozen."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V23FreezeError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _strict_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    return v21_gate._strict_json(path, expected_sha256=expected_sha256)


def _load_frozen_v22_payload() -> dict[str, Any]:
    expected = V22_TERMINAL["freeze_lock"]
    return _strict_json(
        REPO_ROOT / str(expected["path"]),
        str(expected["sha256"]),
    )


# This snapshot is loaded from the hash-pinned terminal V22 freeze once and
# serialized immediately.  The immutable string prevents a caller from
# mutating the reference coverage through an imported module global.
_FROZEN_V22_PAYLOAD = _load_frozen_v22_payload()
_FROZEN_V22_COVERAGE_JSON = json.dumps(
    _FROZEN_V22_PAYLOAD["oracle"]["coverage"],
    sort_keys=True,
    separators=(",", ":"),
    allow_nan=False,
)
# Public audit snapshot.  The binding helper intentionally decodes its own copy
# from the immutable JSON string, so mutation of this convenience value cannot
# alter the scientific gate.
FROZEN_V22_COVERAGE = json.loads(_FROZEN_V22_COVERAGE_JSON)


def validate_oracle_binding(
    core: Mapping[str, Any],
    ledger: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a pure oracle/trace binding with assertions separate from facts.

    No filesystem access occurs here and neither input is modified.  The V22
    coverage algorithm and its frozen expected result are reused verbatim.  A
    caller may aggregate *only* ``result["assertions"].values()``.
    """

    if not isinstance(core, Mapping) or not isinstance(ledger, Mapping):
        raise V23FreezeError("oracle core and ledger must be mappings")
    try:
        core_before = json.dumps(
            core, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        ledger_before = json.dumps(
            ledger, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
    except (TypeError, ValueError) as exc:
        raise V23FreezeError(f"oracle binding input is not strict JSON: {exc}") from exc

    # Both helpers are frozen sources and operate read-only on the supplied
    # mappings.  Coverage is returned as a fresh object.
    canonical_oracle.validate_ledger(ledger)
    views = core.get("views")
    view_integrity = v21_gate._validate_oracle_views(
        core,
        views,
        minimum_cycles=int(v21_freeze.FROZEN_GATE["minimum_complete_cycles_per_view"]),
    )
    coverage = v22_freeze.validate_oracle_trace_coverage(core)
    frozen_v22_coverage = json.loads(_FROZEN_V22_COVERAGE_JSON)

    core_after = json.dumps(
        core, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    ledger_after = json.dumps(
        ledger, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    time_grid = core.get("time_grid")
    if not isinstance(time_grid, Mapping):
        raise V23FreezeError("oracle time_grid is missing")

    assertions = {
        "inputs_unchanged": (
            core_after == core_before and ledger_after == ledger_before
        ),
        "ledger_embeds_supplied_core": ledger.get("scientific_core") == core,
        "trial_exact": core.get("trial_id") == TRIAL_ID,
        "event_contract_exact": core.get("event_contract_id")
        == canonical_oracle.EVENT_CONTRACT_ID,
        "scientific_core_sha_exact": ledger.get("scientific_core_sha256")
        == ORACLE_CORE_SHA256,
        "sample_dt_exact": float(core.get("sample_dt_s", math.nan))
        == SAMPLE_DT_S,
        "threshold_exact": float(core.get("threshold_n", math.nan)) == 20.0,
        "contact_persistence_exact": float(
            core.get("min_contact_duration_s", math.nan)
        )
        == 0.050,
        "minimum_cycle_exact": float(
            core.get("min_cycle_duration_s", math.nan)
        )
        == 0.30,
        "oracle_grid_start_exact": float(time_grid.get("start_s", math.nan))
        == ORACLE_START_S,
        "oracle_grid_end_exact": float(time_grid.get("end_s", math.nan))
        == ORACLE_END_S,
        "oracle_grid_count_exact": int(time_grid.get("sample_count", -1))
        == ORACLE_SAMPLE_COUNT,
        "four_frozen_views_present": isinstance(views, list)
        and len(views) == len(EXPECTED_VIEWS),
        "view_integrity_pass": view_integrity.get("pass") is True,
        "v22_coverage_pass": coverage.get("pass") is True,
        "v22_coverage_exactly_frozen": coverage == frozen_v22_coverage,
    }
    if not all(isinstance(value, bool) for value in assertions.values()):
        raise V23FreezeError("oracle assertions must contain booleans only")

    # These are descriptions of the frozen scientific protocol.  Some are
    # intentionally false and must never be folded into the assertion gate.
    facts = {
        "global_grid_equality_required": False,
        "global_grids_equal": (
            ORACLE_START_S == TRACE_START_S
            and ORACLE_END_S == TRACE_END_S
            and ORACLE_SAMPLE_COUNT == TRACE_SAMPLE_COUNT
        ),
        "oracle_reconstructed": False,
        "consumer_rethresholds_grf": False,
        "trial_is_scientifically_virgin": False,
        "trial_is_independent_holdout": False,
        "oracle_native_grid": copy.deepcopy(coverage["oracle_native_grid"]),
        "detector_trace_grid": copy.deepcopy(coverage["detector_trace_grid"]),
        "oracle_prefix_samples_outside_trace": int(
            coverage["exact_contiguous_subgrid"]["left_margin_samples"]
        ),
        "oracle_suffix_samples_outside_trace": int(
            coverage["exact_contiguous_subgrid"]["right_margin_samples"]
        ),
    }
    return {
        "assertions": assertions,
        "facts": facts,
        "assertions_pass": all(assertions.values()),
        "coverage": copy.deepcopy(coverage),
        "view_integrity": copy.deepcopy(view_integrity),
    }


def _verify_terminal(
    artifacts: Mapping[str, Mapping[str, str]],
    *,
    generation: str,
) -> dict[str, Any]:
    records: dict[str, Any] = {}
    for label, expected in artifacts.items():
        path = REPO_ROOT / str(expected["path"])
        record = _source_record(path)
        if record["sha256"] != expected["sha256"]:
            raise V23FreezeError(
                f"previous {generation} terminal artifact drifted: {label}"
            )
        records[label] = record

    ledger_path = REPO_ROOT / str(artifacts["execution_ledger"]["path"])
    receipt_path = REPO_ROOT / str(artifacts["access_receipt"]["path"])
    if ledger_path.read_bytes() != receipt_path.read_bytes():
        raise V23FreezeError(
            f"previous {generation} ledger and receipt are not byte-identical"
        )
    ledger = _strict_json(
        ledger_path, str(artifacts["execution_ledger"]["sha256"])
    )
    failure = _strict_json(
        REPO_ROOT / str(artifacts["failure"]["path"]),
        str(artifacts["failure"]["sha256"]),
    )

    if generation == "V21":
        checks = {
            "ledger_consumed": ledger.get("stage_consumed") is True,
            "ledger_no_retry": ledger.get("rerun_allowed") is False,
            "failure_status_exact": failure.get("status")
            == "ERROR_INTERNAL_V21_TRIAL08_CONSUMED",
            "failure_consumed": failure.get("stage_consumed") is True,
            "failure_no_retry": failure.get("rerun_allowed") is False,
            "detector_not_sampled": failure.get("execution_audit", {}).get("stage")
            == "trial08_replay_inputs_verified",
            "protected_unopened": failure.get("protected_trials_opened") == [],
            "reserve_unopened": failure.get("reserve_trials_opened") == [],
        }
        success_names = (
            "manifest.json",
            "trial08_decision_lock.json",
            "packed_binary_trace.json",
        )
    elif generation == "V22":
        exception = str(failure.get("exception", ""))
        checks = {
            "ledger_consumed": ledger.get("cycle_consumed") is True,
            "ledger_no_retry": ledger.get("rerun_allowed") is False,
            "ledger_replay_not_started": ledger.get("detector_replay_started")
            is False,
            "failure_status_exact": failure.get("status")
            == "ERROR_V22_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CONSUMED",
            "failure_consumed": failure.get("cycle_consumed") is True,
            "failure_no_retry": failure.get("rerun_allowed") is False,
            "detector_not_sampled": failure.get("execution_audit", {}).get("stage")
            == "development_replay_inputs_verified",
            "boolean_partition_error_exact": (
                "'global_grid_equality_required': False" in exception
                and "'coverage': True" in exception
                and "'coverage_exactly_frozen': True" in exception
            ),
            "protected_unopened": failure.get("protected_trials_opened") == [],
            "reserve_unopened": failure.get("reserve_trials_opened") == [],
            "h0_not_executed": failure.get("h0_executed") is False,
            "runtime_not_promoted": failure.get("runtime_promoted") is False,
            "training_not_promoted": failure.get("training_promoted") is False,
        }
        success_names = (
            "manifest.json",
            "trial08_development_decision_lock.json",
            "packed_binary_trace.json",
        )
    else:  # pragma: no cover - internal programming guard
        raise V23FreezeError(f"unknown terminal generation: {generation}")

    old_output_dir = receipt_path.parent
    checks["no_old_success_artifacts"] = not any(
        os.path.lexists(os.fspath(old_output_dir / name))
        for name in success_names
    )
    if not all(checks.values()):
        raise V23FreezeError(
            f"previous {generation} terminal state drifted: {checks}"
        )
    return {"checks": checks, "artifacts": records}


def _verify_oracle() -> dict[str, Any]:
    ledger = _strict_json(ORACLE_PATH, ORACLE_SHA256)
    core = ledger.get("scientific_core")
    if not isinstance(core, Mapping):
        raise V23FreezeError("oracle scientific_core is missing")
    binding = validate_oracle_binding(core, ledger)
    if not all(binding["assertions"].values()):
        raise V23FreezeError(
            f"oracle binding assertions failed: {binding['assertions']}"
        )
    if binding["assertions_pass"] is not True:
        raise V23FreezeError("oracle assertion aggregate drifted")
    return {
        "source": _source_record(ORACLE_PATH),
        "scientific_core_sha256": ORACLE_CORE_SHA256,
        "binding": binding,
    }


def _verify_sources() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative, expected_sha in PINNED_SOURCES.items():
        record = _source_record(REPO_ROOT / relative)
        if record["sha256"] != expected_sha:
            raise V23FreezeError(f"pinned source drifted: {relative}")
        records[relative] = record
    for relative in DYNAMIC_SOURCES:
        records[relative] = _source_record(REPO_ROOT / relative)
    return records


def build_freeze_payload(*, check_destinations: bool = False) -> dict[str, Any]:
    if check_destinations and any(
        os.path.lexists(os.fspath(path))
        for path in (FREEZE_PATH, EXECUTION_LEDGER_PATH, OUTPUT_DIR)
    ):
        raise V23FreezeError("V23 freeze or execution destination is occupied")

    previous_v21 = _verify_terminal(V21_TERMINAL, generation="V21")
    previous_v22 = _verify_terminal(V22_TERMINAL, generation="V22")
    oracle = _verify_oracle()
    sources = _verify_sources()
    profile = _source_record(v21_freeze.PROFILE_PATH)
    if profile["sha256"] != v21_freeze.PROFILE_SHA256:
        raise V23FreezeError("V21 finalist profile drifted")
    declarations = json.loads(
        json.dumps(v21_freeze.TRIAL08_DECLARATIONS, allow_nan=False)
    )

    payload = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "freeze_date": "2026-08-04",
        "status": "V23_OPEN_DEVELOPMENT_FROZEN_BEFORE_DETECTOR_REPLAY",
        "authorization": {
            "kind": "explicit_user_authorized_v23_after_terminal_v22_error",
            "date": "2026-08-04",
            "request": "autorizzo V23",
            "previous_v21_error_reclassified": False,
            "previous_v22_error_reclassified": False,
            "previous_terminal_artifacts_mutable": False,
        },
        "candidate": {
            "candidate_id": CANDIDATE_ID,
            "profile": profile,
            "geometry": copy.deepcopy(v21_freeze.CANDIDATE_GEOMETRY),
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
            "already_open_before_v23": True,
            "claim_limit": (
                "development robustness evidence for the frozen V21 geometry "
                "and V20 FSM on trials 02/04/08; not independent validation"
            ),
            "trace_interval_s": [TRACE_START_S, TRACE_END_S],
            "trace_sample_count": TRACE_SAMPLE_COUNT,
            "declared_inputs": declarations,
        },
        "correction": {
            "changed_invariant_only": (
                "separate positive oracle-binding assertions from descriptive "
                "facts before boolean aggregation"
            ),
            "assertion_aggregate_path": "oracle.binding.assertions",
            "facts_are_never_aggregated": True,
            "global_grid_equality_required_is_fact": True,
            "global_grid_equality_required": False,
            "v22_coverage_algorithm_changed": False,
            "v22_frozen_coverage_changed": False,
            "geometry_changed": False,
            "fsm_changed": False,
            "scorer_or_numerical_gate_changed": False,
            "oracle_reconstructed_or_rethresholded": False,
        },
        "gate": copy.deepcopy(v21_freeze.FROZEN_GATE),
        "oracle": oracle,
        "previous_v21_terminal_error": previous_v21,
        "previous_v22_terminal_error": previous_v22,
        "opening_contract": {
            "global_execution_ledger": EXECUTION_LEDGER_PATH.relative_to(
                REPO_ROOT
            ).as_posix(),
            "output_dir": OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
            "ledger_and_receipt_before_detector_replay": True,
            "single_execution_in_this_development_cycle": True,
            "retry_after_terminal_result_allowed": False,
            "geometry_fsm_or_gate_retuning_allowed": False,
            "terminal_decision_last": True,
            "all_destinations_no_clobber_lexists": True,
        },
        "post_pass_scope": copy.deepcopy(POST_PASS_SCOPE),
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
            "protected_or_reserve_access_allowed": False,
            "runtime_promotion_allowed": False,
            "training_allowed": False,
            "corridor_activation_allowed": False,
            "positive_morphology_reward_ppo_allowed": False,
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
                raise V23FreezeError("existing V23 freeze differs from expected")
            existing_verified = True
        else:
            payload = build_freeze_payload(check_destinations=True)
            existing_verified = False
    except Exception as exc:
        print(
            f"V23 development freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2

    print(
        json.dumps(
            {
                "status": payload["status"],
                "candidate_id": CANDIDATE_ID,
                "trial_role": TRIAL_ROLE,
                "oracle_assertions_pass": payload["oracle"]["binding"][
                    "assertions_pass"
                ],
                "global_grid_equality_required": payload["oracle"]["binding"][
                    "facts"
                ]["global_grid_equality_required"],
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
