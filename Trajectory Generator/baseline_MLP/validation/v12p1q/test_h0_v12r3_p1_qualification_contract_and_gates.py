"""Pure tests for the independent V12R3-P1 qualification contract and gates."""

from __future__ import annotations

import copy
import importlib
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12P1Q_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_primary_split_v6_qualification_contract as v6  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import h0_v12r3_p1_qualification_gates as gates  # noqa: E402


_SHA = "a" * 64


def _artifact(path: str | PurePosixPath, *, sha256: str = _SHA) -> dict[str, Any]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": sha256,
        "size_bytes": 1,
    }


def _source_h0_tree() -> dict[str, Any]:
    return {
        **contract.SOURCE_H0_MODULE,
        "file_count": 1,
        "files": [
            {
                "path": "module_state.pkl",
                "sha256": _SHA,
                "size_bytes": 1,
            }
        ],
    }


def _metric(sample_count: int, *, rms: float, abs_max: float) -> dict[str, Any]:
    return {"sample_count": sample_count, "rms": rms, "abs_max": abs_max}


def _valid_prerequisite_payload() -> dict[str, Any]:
    rows = [
        {
            "name": requirement["name"],
            "status": requirement["required_status"],
            "passed": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "candidate_module_tree_sha256": contract.P1_CANDIDATE_MODULE["tree_sha256"],
            "artifact": _artifact(requirement["path"]),
        }
        for requirement in contract.prerequisite_requirements()
    ]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREREQUISITE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "prerequisites": rows,
        "hashes_bound_after_salvage_terminal_pass": True,
        "salvage_stage_order": list(contract.SALVAGE_STAGE_IDS),
        "salvage_completed_stages": list(contract.SALVAGE_STAGE_IDS),
        "salvage_rollout_count": 6,
        "salvage_passing_rollout_count": 6,
        "salvage_failed_rollout_count": 0,
        "salvage_terminal_ledger_passed": True,
        "qualification_root_preexisting": False,
        "noise_root_preexisting": False,
        "qualification_rollouts_opened": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "runtime_promoted": False,
    }


def _valid_rollout(role: str, case_id: str) -> dict[str, Any]:
    expected = contract.canonical_rollout(role, case_id)
    root = PurePosixPath(expected["destination"])
    actor_module = (
        copy.deepcopy(contract.P1_CANDIDATE_MODULE)
        if role == contract.CANDIDATE_ROLE
        else _source_h0_tree()
    )
    sea = {}
    for joint in contract.JOINTS:
        joint_metrics = {
            signal: _metric(
                contract.SEA_EXPECTED_SAMPLE_COUNTS[signal], rms=1.0, abs_max=2.0
            )
            for signal in contract.SEA_SIGNALS
        }
        joint_metrics["tau_input_saturated"] = {
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "count": 0,
            "fraction": 0.0,
        }
        sea[joint] = joint_metrics
    payload = {
        **expected,
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "actor_module": actor_module,
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_DTYPE,
        "action_shape": list(contract.EXPECTED_ACTION_SHAPE),
        "action_dtype": contract.EXPECTED_DTYPE,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "actor_query_count": contract.EXPECTED_STEPS,
        "steps": contract.EXPECTED_STEPS,
        "trace_step_count": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "binary_phase_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": contract.MINIMUM_VALID_CYCLES,
        "grf_penetration_max_m": contract.PENETRATION_LIMIT_M - 0.001,
        "legacy_event_integrity_passed": True,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "episode_metrics": {
            "reserve_norm_nm": _metric(contract.EXPECTED_STEPS, rms=10.0, abs_max=20.0),
            "residual_norm_nm": _metric(
                contract.EXPECTED_STEPS, rms=1.0e-7, abs_max=2.0e-7
            ),
        },
        "sea_episode_metrics": sea,
        "random_noise_draw_count": (
            contract.EXPECTED_STEPS
            if expected["action_selection"] == "stochastic"
            else 0
        ),
        "single_noise_application_count": contract.EXPECTED_STEPS,
        "noise_tape": _artifact(expected["noise_tape"]),
        "noise_tape_array_sha256": _SHA,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "worker_claim": _artifact(
            contract.worker_claim_path(f"rollout__{role}__{case_id}")
        ),
        "prerequisite_gate_passed": True,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
    }
    payload.update({field: 0 for field in contract.ZERO_REQUIRED_COUNTS})
    return payload


def _valid_pair_bindings() -> list[dict[str, Any]]:
    return [
        {
            "case_id": case_id,
            "passed": True,
            "pair_gate": _artifact(contract.pair_gate_path(case_id)),
            "baseline_receipt": _artifact(
                contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id)
            ),
            "candidate_receipt": _artifact(
                contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id)
            ),
        }
        for case_id in contract.CASE_IDS
    ]


def _valid_aggregate_payload() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "prerequisite_gate_passed": True,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": _valid_pair_bindings(),
        "pair_count": 6,
        "passing_pair_count": 6,
        "failed_pair_count": 0,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(contract.worker_claim_path("finalize_qualification")),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "compensation_or_averaging_used": False,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _signature(case: dict[str, Any]) -> tuple[Any, ...]:
    return (
        case["case_id"],
        case["action_selection"],
        case["episode_start_offset_s"],
        case["action_seed"],
        case["runtime_seed"],
        case["sigma"],
        PurePosixPath(case["noise_tape"]).name,
    )


def test_contract_identity_and_exact_p1_v26_routing() -> None:
    assert contract.SCHEMA_VERSION == 125
    assert contract.REVISION == "2026-08-09"
    assert contract.AUTHORITY_TEXT == "esegui i punti 1-6"
    assert contract.P1_CANDIDATE_ID == (
        "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY:" "p1:ff34e153ae0ac9b6"
    )
    assert contract.P1_CANDIDATE_MODULE["tree_sha256"] == (
        "ff34e153ae0ac9b6f7b8d7d92766e47eecf087285020ac5332d9bd41170ac3ed"
    )
    candidate = contract.role_contract(contract.CANDIDATE_ROLE)
    assert candidate["event_contract_id"] == ("binary_point_v25+heel_qualified_fsm_v2")
    assert candidate["target_contract_id"] == (
        "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
    )
    assert candidate["binary_phase_fsm_mode"] == "binary_active"
    assert candidate["actor_module"] == contract.P1_CANDIDATE_MODULE
    assert candidate["morphology_weight"] == 0.0


def test_baseline_is_original_h0_historical_analog_with_legacy_events() -> None:
    baseline = contract.role_contract(contract.BASELINE_ROLE)
    assert baseline["actor_id"] == "original_h0"
    assert baseline["actor_input_view"] == "historical_analog"
    assert baseline["observation_semantics"] == "counterfactual_analog"
    assert baseline["phase_fsm_input_mode"] == "legacy_events"
    assert baseline["event_contract_id"] == "legacy_events"
    assert baseline["binary_phase_fsm_mode"] == "disabled"
    assert baseline["actor_module"] == contract.SOURCE_H0_MODULE


def test_holdouts_are_the_six_unmaterialized_v6_conditions() -> None:
    observed = tuple(_signature(case) for case in contract.canonical_cases())
    inherited = tuple(_signature(case) for case in v6.canonical_cases())
    assert observed == inherited
    assert len(observed) == 6
    assert observed[:2] == (
        (
            "deterministic_offset_minus_0p30",
            "deterministic",
            contract.CANONICAL_OFFSET_S - 0.30,
            None,
            129,
            0.0,
            "deterministic_all_zero.npz",
        ),
        (
            "deterministic_offset_plus_0p30",
            "deterministic",
            contract.CANONICAL_OFFSET_S + 0.30,
            None,
            129,
            0.0,
            "deterministic_all_zero.npz",
        ),
    )
    assert [row[3:6] for row in observed[2:]] == [
        (seed, seed, 0.005) for seed in (130, 131, 132, 133)
    ]
    assert contract.HOLDOUT_PROVENANCE["noise_tapes_materialized"] is False
    assert contract.HOLDOUT_PROVENANCE["qualification_rollouts_opened"] is False


def test_matrix_is_baseline_first_then_candidate_and_has_no_compensation() -> None:
    assert [(row["role"], row["case_id"]) for row in contract.ROLLOUT_MATRIX] == [
        (role, case_id)
        for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
        for case_id in contract.CASE_IDS
    ]
    assert len(contract.ROLLOUT_MATRIX) == 12
    assert len(contract.ROLLOUT_STAGE_IDS) == 12
    assert contract.STAGE_IDS[-1] == "finalize_qualification"


def test_v6_tolerances_and_strict_absolute_gates_are_preserved() -> None:
    assert contract.RESERVE_TOLERANCES == v6.RESERVE_TOLERANCES
    assert contract.SEA_TOLERANCES == v6.SEA_TOLERANCES
    assert len(contract.SEA_TOLERANCES) == 24
    assert contract.EXPECTED_STEPS == 500
    assert contract.EXPECTED_CONTROL_WINDOWS == 5_000
    assert contract.EXPECTED_RAW_SENSOR_SAMPLES == 5_000
    assert contract.MINIMUM_VALID_CYCLES == 2
    assert contract.PENETRATION_LIMIT_M == 0.025


def test_future_salvage_requirements_defer_hashes_and_use_terminal_runner_status() -> (
    None
):
    requirements = contract.prerequisite_requirements()
    assert [row["name"] for row in requirements] == [
        "salvage_protocol_freeze",
        "salvage_execution_lock",
        "salvage_final_development_receipt",
        "salvage_terminal_pass_ledger",
    ]
    assert all(
        set(row) == {"name", "path", "required_status", "candidate_identity_required"}
        for row in requirements
    )
    assert all(row["candidate_identity_required"] is True for row in requirements)

    salvage_root = (
        REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation" / "v12p1s"
    )
    if str(salvage_root) not in sys.path:
        sys.path.insert(0, str(salvage_root))
    salvage_contract = importlib.import_module("h0_v12r3_p1_salvage_contract")
    salvage_runner = importlib.import_module("run_h0_v12r3_p1_salvage_development")
    assert requirements[0]["required_status"] == (
        salvage_contract.PROTOCOL_FREEZE_PASS_STATUS
    )
    assert requirements[1]["required_status"] == (
        salvage_contract.EXECUTION_LOCK_PASS_STATUS
    )
    assert requirements[2]["required_status"] == (
        salvage_contract.FINAL_DEVELOPMENT_PASS_STATUS
    )
    assert requirements[3]["required_status"] == salvage_runner.LEDGER_PASS_STATUS
    assert contract.SALVAGE_PIPELINE_PASS_STATUS == (
        "PASS_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
    )


def test_design_freeze_and_future_protocol_freeze_are_separate() -> None:
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.PROTOCOL_FREEZE_PATH
    assert contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS == (
        "PASS_H0_V12R3_P1_QUALIFICATION_DESIGN_FREEZE"
    )
    assert list(contract.DESIGN_SOURCE_RELATIVE_PATHS) == [
        "v6_qualification_contract",
        "v6_qualification_gates",
        "v6_noise_preparer",
        "qualification_contract",
        "qualification_gates",
        "qualification_design_freezer",
        "qualification_contract_and_gates_tests",
        "qualification_design_freezer_tests",
    ]


def test_declared_paths_are_isolated_portable_and_unique() -> None:
    paths = contract.declared_mutation_paths()
    root = f"{contract.VALIDATION_ROOT.as_posix()}/"
    values = [path.as_posix() for path in paths.values()]
    assert paths["qualification_design_freeze"] == (
        contract.QUALIFICATION_DESIGN_FREEZE_PATH
    )
    assert all(
        not path.is_absolute() and ".." not in path.parts for path in paths.values()
    )
    assert all(value.startswith(root) for value in values)
    assert len(values) == len(set(values))
    assert max(map(len, values)) < 220


@pytest.mark.parametrize("role", contract.ROLE_ORDER)
@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_common_gate_passes_all_twelve_canonical_rollouts(
    role: str, case_id: str
) -> None:
    result = gates.common_rollout_gate(
        _valid_rollout(role, case_id), role=role, case_id=case_id
    )
    assert result["passed"] is True
    assert result["status"] == contract.ROLLOUT_PASS_STATUS


def test_prerequisite_gate_requires_all_four_same_candidate_terminal_records() -> None:
    payload = _valid_prerequisite_payload()
    result = gates.prerequisite_gate(payload)
    assert result["passed"] is True
    assert result["qualification_execution_authorized"] is False

    for mutate in (
        lambda value: value["prerequisites"].pop(),
        lambda value: value["prerequisites"][3].update(status="old_status"),
        lambda value: value["prerequisites"][2].update(candidate_id="other"),
        lambda value: value["prerequisites"][1]["artifact"].update(sha256="bad"),
        lambda value: value.update(salvage_passing_rollout_count=5),
        lambda value: value.update(salvage_terminal_ledger_passed=False),
        lambda value: value.update(qualification_root_preexisting=True),
        lambda value: value.update(actor_updates=1),
        lambda value: value.update(retry_authorized=True),
    ):
        drifted = copy.deepcopy(payload)
        mutate(drifted)
        assert gates.prerequisite_gate(drifted)["passed"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("steps", 499),
        ("trace_step_count", 499),
        ("control_window_count", 4_999),
        ("raw_sensor_sample_count", 4_999),
        ("binary_phase_sensor_sample_count", 4_999),
        ("phase_valid_cycle_count", 1),
        ("grf_penetration_max_m", 0.025),
        ("grf_penetration_max_m", math.nan),
    ],
)
def test_common_gate_rejects_every_physical_boundary(field: str, value: Any) -> None:
    case_id = contract.CASE_IDS[0]
    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload[field] = value
    result = gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["physical"] is False


@pytest.mark.parametrize("field", contract.ZERO_REQUIRED_COUNTS)
def test_common_gate_rejects_each_nonzero_invalid_fallback_or_update(
    field: str,
) -> None:
    case_id = contract.CASE_IDS[0]
    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload[field] = 1
    result = gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["zero_invalids_fallbacks_updates"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("teacher_enabled", True),
        ("teacher_loaded_during_rollout", True),
        ("blending_enabled", True),
        ("safety_latch_enabled", True),
        ("actor_query_count", 499),
    ],
)
def test_common_gate_rejects_teacher_blend_latch_or_missing_actor_query(
    field: str, value: Any
) -> None:
    case_id = contract.CASE_IDS[0]
    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload[field] = value
    result = gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["pure_autonomy"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("passed", False),
        ("sample_count", 4_999),
        ("duplicate_event_count", 1),
        ("out_of_order_event_count", 1),
        ("left_non_v26_source_count", 1),
        ("fallback_count", 1),
        ("hard_invalid_count", 1),
    ],
)
def test_candidate_gate_requires_exact_binary_v26_event_integrity(
    field: str, value: Any
) -> None:
    case_id = contract.CASE_IDS[0]
    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload["binary_phase_event_gate"][field] = value
    result = gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["role_event_integrity"] is False


def test_role_routing_is_fail_closed_for_baseline_and_candidate_drift() -> None:
    case_id = contract.CASE_IDS[0]
    baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
    baseline["legacy_event_integrity_passed"] = False
    assert not gates.common_rollout_gate(
        baseline, role=contract.BASELINE_ROLE, case_id=case_id
    )["passed"]

    candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    candidate["target_contract_id"] = "binary_point_v25+functional_contact_fsm_v1"
    result = gates.common_rollout_gate(
        candidate, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["role_routing_exact"] is False


def test_common_gate_rejects_bad_metric_samples_nonfinite_and_saturation() -> None:
    case_id = contract.CASE_IDS[0]
    mutations = []

    bad_samples = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    bad_samples["episode_metrics"]["reserve_norm_nm"]["sample_count"] = 499
    mutations.append((bad_samples, "diagnostic_metrics"))

    nonfinite = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    nonfinite["sea_episode_metrics"][contract.JOINTS[0]][contract.SEA_SIGNALS[0]][
        "rms"
    ] = math.inf
    mutations.append((nonfinite, "diagnostic_metrics"))

    saturated = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    saturated["sea_episode_metrics"][contract.JOINTS[0]]["tau_input_saturated"].update(
        count=1, fraction=0.0002
    )
    mutations.append((saturated, "sea_saturation_zero"))

    for payload, check in mutations:
        result = gates.common_rollout_gate(
            payload, role=contract.CANDIDATE_ROLE, case_id=case_id
        )
        assert result["passed"] is False
        assert result["checks"][check] is False


def test_common_gate_rejects_noise_or_persistence_drift() -> None:
    case_id = contract.CASE_IDS[2]
    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload["random_noise_draw_count"] = 499
    assert not gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )["passed"]

    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload["noise_tape"]["path"] = "wrong.npz"
    assert not gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )["passed"]

    payload = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    payload["trace"]["path"] = "wrong.json"
    result = gates.common_rollout_gate(
        payload, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert result["passed"] is False
    assert result["checks"]["persisted_before_gate"] is False


@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_each_condition_matched_pair_passes_without_compensation(case_id: str) -> None:
    result = gates.condition_matched_gate(
        _valid_rollout(contract.BASELINE_ROLE, case_id),
        _valid_rollout(contract.CANDIDATE_ROLE, case_id),
        case_id=case_id,
    )
    assert result["passed"] is True
    assert result["status"] == contract.PAIR_PASS_STATUS
    assert result["compensation_or_averaging_used"] is False
    assert len(result["noninferiority_rows"]) == 28


def test_pair_gate_rejects_condition_mismatch() -> None:
    case_id = contract.CASE_IDS[0]
    baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
    candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    candidate["noise_tape_array_sha256"] = "b" * 64
    result = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
    assert result["passed"] is False
    assert result["checks"]["condition_exact"] is False


def test_reserve_noninferiority_accepts_exact_cap_and_rejects_above() -> None:
    case_id = contract.CASE_IDS[0]
    baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
    candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
    baseline["episode_metrics"]["reserve_norm_nm"].update(rms=1.0, abs_max=20.0)
    candidate["episode_metrics"]["reserve_norm_nm"].update(rms=6.0, abs_max=20.0)
    assert gates.condition_matched_gate(baseline, candidate, case_id=case_id)["passed"]

    candidate["episode_metrics"]["reserve_norm_nm"]["rms"] = 6.000001
    result = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
    assert result["passed"] is False
    failed = [
        row
        for row in result["noninferiority_rows"]
        if row["metric"] == "reserve_norm_nm.rms"
    ]
    assert len(failed) == 1 and failed[0]["passed"] is False


def test_each_sea_signal_aggregation_uses_exact_v6_tolerance() -> None:
    case_id = contract.CASE_IDS[0]
    for metric_name, absolute, relative in contract.SEA_TOLERANCES:
        joint, signal, aggregation = metric_name.split(".")
        baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
        candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)
        baseline_metric = baseline["sea_episode_metrics"][joint][signal]
        candidate_metric = candidate["sea_episode_metrics"][joint][signal]
        if aggregation == "rms":
            baseline_metric.update(rms=1.0e-8, abs_max=1.0e-4)
            candidate_metric.update(rms=1.01e-6, abs_max=1.0e-4)
        else:
            baseline_metric.update(rms=0.0, abs_max=1.0e-8)
            candidate_metric.update(rms=0.0, abs_max=1.01e-6)
        passing = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
        assert passing["passed"] is True
        row = next(
            row
            for row in passing["noninferiority_rows"]
            if row["metric"] == metric_name
        )
        assert row["absolute_tolerance"] == absolute == 1.0e-6
        assert row["relative_tolerance"] == relative == 0.05
        assert row["comparison_cap"] == math.nextafter(row["cap"], math.inf)

        candidate_metric[aggregation] += 1.0e-9
        failing = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
        assert failing["passed"] is False


def test_aggregate_requires_ordered_six_of_six_without_compensation() -> None:
    payload = _valid_aggregate_payload()
    result = gates.aggregate_qualification_gate(payload)
    assert result["passed"] is True
    assert result["independent_qualification_passed"] is True
    assert result["candidate_promoted"] is False

    mutations = []
    missing = copy.deepcopy(payload)
    missing["pair_bindings"].pop()
    mutations.append(missing)
    reordered = copy.deepcopy(payload)
    reordered["pair_bindings"][0], reordered["pair_bindings"][1] = (
        reordered["pair_bindings"][1],
        reordered["pair_bindings"][0],
    )
    mutations.append(reordered)
    one_failed = copy.deepcopy(payload)
    one_failed["pair_bindings"][0]["passed"] = False
    mutations.append(one_failed)
    wrong_count = copy.deepcopy(payload)
    wrong_count["passing_pair_count"] = 5
    mutations.append(wrong_count)
    compensated = copy.deepcopy(payload)
    compensated["compensation_or_averaging_used"] = True
    mutations.append(compensated)
    retried = copy.deepcopy(payload)
    retried["retry_authorized"] = True
    mutations.append(retried)
    promoted = copy.deepcopy(payload)
    promoted["runtime_promoted"] = True
    mutations.append(promoted)
    morphology = copy.deepcopy(payload)
    morphology["positive_morphology_enabled"] = True
    mutations.append(morphology)
    for drifted in mutations:
        assert gates.aggregate_qualification_gate(drifted)["passed"] is False


@pytest.mark.parametrize("malformed", [None, {}, [], "bad", 1, math.nan])
def test_all_gates_are_total_and_fail_closed_on_malformed_input(malformed: Any) -> None:
    case_id = contract.CASE_IDS[0]
    assert gates.prerequisite_gate(malformed)["passed"] is False
    assert (
        gates.common_rollout_gate(
            malformed, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is False
    )
    assert (
        gates.condition_matched_gate(malformed, malformed, case_id=case_id)["passed"]
        is False
    )
    assert gates.aggregate_qualification_gate(malformed)["passed"] is False
