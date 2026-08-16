from __future__ import annotations

import copy
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Callable

import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
PARENT_VALIDATION = LOCAL_ROOT.parent
R3_ROOT = PARENT_VALIDATION / "v12r3"
for _root in (LOCAL_ROOT, PARENT_VALIDATION, R3_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_primary_split_v12r3_autonomy_recovery_contract as r3  # noqa: E402
import h0_v12r3_p1_salvage_contract as contract  # noqa: E402


_SHA = "ab" * 32


def _artifact(path: str | PurePosixPath) -> dict[str, object]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": _SHA,
        "size_bytes": 1,
    }


def _valid_qualification_payload() -> dict[str, Any]:
    return {
        "schema_version": contract.QUALIFICATION_DESIGN_SCHEMA_VERSION,
        "status": contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.QUALIFICATION_DESIGN_PROTOCOL_ID,
        "freeze_kind": contract.QUALIFICATION_DESIGN_FREEZE_KIND,
        "publication_destination": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
        "checks": {
            "schema_125": True,
            "salvage_run_root_absent": True,
        },
        "zero_design_activity": {
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "rollout_executions": 0,
        },
        "zero_design_activity_gate": {"passed": True},
        "qualification_access": {
            "status": contract.QUALIFICATION_DESIGN_LOCKED_ACCESS_STATUS,
            "qualification_design_frozen": True,
            "qualification_protocol_freeze": None,
            "qualification_execution_lock": None,
            "future_prerequisite_hashes": None,
            "hash_binding_deferred_until_salvage_terminal_pass": True,
            "required_salvage_rollout_count": 6,
            "required_salvage_passing_rollout_count": 6,
            "required_salvage_failed_rollout_count": 0,
            "noise_materialization_authorized": False,
            "qualification_execution_authorized": False,
            "runtime_promotion_authorized": False,
        },
        "qualification_protocol_freeze": None,
        "qualification_execution_lock": None,
        "noise_manifest": None,
        "salvage_artifact_hashes": None,
        "runtime_promoted": False,
    }


def _qualification_binding() -> dict[str, Any]:
    payload = _valid_qualification_payload()
    gate = contract.qualification_design_freeze_gate(payload)
    assert gate["passed"] is True
    return {
        "qualification_design_freeze": _artifact(
            contract.QUALIFICATION_DESIGN_FREEZE_PATH
        ),
        "qualification_design_freeze_gate": gate,
    }


def _valid_selection() -> dict[str, Any]:
    return {
        **_qualification_binding(),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_SELECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "r3_selected_artifacts": copy.deepcopy(contract.R3_SELECTED_ARTIFACTS),
        "r3_terminal_run_tree": copy.deepcopy(contract.R3_TERMINAL_RUN_TREE),
        "r3_terminal_semantics": copy.deepcopy(contract.R3_TERMINAL_SEMANTICS),
        "selection_policy": copy.deepcopy(contract.CANDIDATE_SELECTION_POLICY),
        "unique_eligible_candidate": True,
        "p1_fit_gate_passed": True,
        "p1_probe_integrity_passed": True,
        "p1_probe_autonomy_passed": True,
        "v12r3_reopened": False,
        "v12r3_completed": False,
        "runtime_promoted": False,
        "label_p1_used_for_candidate_selection": False,
        "p2_artifacts_used": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "actor_fit_executions": 0,
        "offline_teacher_label_calls": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
    }


def _valid_rollout(case_id: str) -> dict[str, Any]:
    case = contract.canonical_development_case(case_id)
    root = PurePosixPath(case["destination"])
    result: dict[str, Any] = {
        **_qualification_binding(),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "candidate_selection_gate_passed": True,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "behavior": contract.BEHAVIOR,
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_DTYPE,
        "morphology_weight": 0.0,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "latch_active_at_episode_end": False,
        "candidate_mean_query_count": 500,
        "steps": 500,
        "trace_step_count": 500,
        "control_window_count": 5_000,
        "raw_sensor_sample_count": 5_000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024999,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": 5_000,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "binary_event_prefix_integrity": {
            "passed": True,
            "sample_count": 5_000,
            "raw_sensor_sample_count": 5_000,
            "control_window_count": 5_000,
            "expected_sample_count": 5_000,
        },
        "episode_metrics": {
            "reserve_norm_nm": {
                "sample_count": 500,
                "rms": 210.0,
                "abs_max": 573.0,
            },
            "residual_norm_nm": {
                "sample_count": 500,
                "rms": 1.0e-10,
                "abs_max": 1.0e-8,
            },
        },
        "sea_episode_metrics": {
            joint: {
                **{
                    signal: {
                        "sample_count": contract.SEA_EXPECTED_SAMPLE_COUNTS[signal],
                        "rms": 1.0,
                        "abs_max": 2.0,
                    }
                    for signal in contract.SEA_SIGNALS
                },
                "tau_input_saturated": {
                    "sample_count": 5_000,
                    "count": 0,
                    "fraction": 0.0,
                },
            }
            for joint in contract.JOINTS
        },
        "random_noise_draw_count": (
            500 if case["action_selection"] == "stochastic" else 0
        ),
        "single_noise_application_count": 500,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "logstd_byte_exact": True,
        "disabled_clock_column_indices": list(contract.DISABLED_CLOCK_COLUMN_INDICES),
        "disabled_clock_columns_bit_zero": True,
        "execution_authority": copy.deepcopy(contract.DEVELOPMENT_EXECUTION_AUTHORITY),
        "rollout_executions": 1,
        "environment_reset_calls": 1,
        "environment_step_calls": contract.EXPECTED_STEPS,
        "candidate_mean_queries": contract.EXPECTED_STEPS,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "worker_claim": _artifact(
            contract.worker_claim_path(f"development__{case_id}")
        ),
        "p2_artifacts_opened": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "development_only": True,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
    }
    result.update({name: 0 for name in contract._ZERO_ROLLOUT_COUNTERS})
    return result


def _valid_final() -> dict[str, Any]:
    bindings = []
    for case in contract.DEVELOPMENT_CASES:
        root = PurePosixPath(case["destination"])
        bindings.append(
            {
                "case_id": case["case_id"],
                "passed": True,
                "receipt": _artifact(root / "receipt.json"),
                "gate": _artifact(root / "gate.json"),
                "summary": _artifact(root / "summary.json"),
            }
        )
    return {
        **_qualification_binding(),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "finalize_development",
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "candidate_selection_gate_passed": True,
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(contract.worker_claim_path("finalize_development")),
        "rollout_bindings": bindings,
        "rollout_count": 6,
        "passing_rollout_count": 6,
        "failed_rollout_count": 0,
        "case_order": list(contract.CASE_IDS),
        "all_cases_required": True,
        "compensation_authorized": False,
        "best_k_authorized": False,
        "case_drop_authorized": False,
        "execution_authority": copy.deepcopy(contract.DEVELOPMENT_EXECUTION_AUTHORITY),
        "fit_executions": 0,
        "actor_fit_executions": 0,
        "offline_teacher_label_calls": 0,
        "environment_reset_calls_outside_rollouts": 0,
        "environment_step_calls_outside_rollouts": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "rollout_environment_reset_calls": 6,
        "rollout_environment_step_calls": 6 * contract.EXPECTED_STEPS,
        "candidate_mean_queries": 6 * contract.EXPECTED_STEPS,
        "p2_artifacts_opened": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "qualification_executed": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
    }


def test_identity_authority_candidate_and_terminal_tree_are_exact() -> None:
    assert contract.SCHEMA_VERSION == 124
    assert contract.REVISION == "2026-08-09"
    assert contract.AUTHORITY_TEXT == "esegui i punti 1-6"
    assert contract.AUTHORITY["actor_fit_authorized"] is False
    assert contract.AUTHORITY["offline_teacher_labeling_authorized"] is False
    assert contract.AUTHORITY["p2_reuse_authorized"] is False
    assert contract.P1_CANDIDATE_ID.endswith(":p1:ff34e153ae0ac9b6")
    assert contract.P1_CANDIDATE_MODULE["tree_sha256"] == (
        "ff34e153ae0ac9b6f7b8d7d92766e47eecf087285020ac5332d9bd41170ac3ed"
    )
    assert contract.R3_TERMINAL_RUN_TREE == {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r3/h0_v12r3_run_20260809"
        ),
        "tree_sha256": (
            "1248381ad63d990c59b9aaff812385b606bdbf1516814c497f32a53a0d1ee50e"
        ),
        "file_count": 2_819,
        "total_size_bytes": 79_261_095,
    }
    assert contract.DISABLED_CLOCK_COLUMN_INDICES == (0, 1)


def test_qualification_design_contract_and_semantics_are_fail_closed() -> None:
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix().endswith(
        "v12p1q/h0_v12r3_p1_qualification_design_freeze.json"
    )
    payload = _valid_qualification_payload()
    assert contract.qualification_design_freeze_gate(payload)["passed"] is True

    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value.__setitem__("schema_version", 124),
        lambda value: value.__setitem__("status", "FAIL"),
        lambda value: value.__setitem__("passed", False),
        lambda value: value.__setitem__("protocol_id", "other"),
        lambda value: value.__setitem__("freeze_kind", "other"),
        lambda value: value["checks"].__setitem__("salvage_run_root_absent", False),
        lambda value: value["zero_design_activity"].__setitem__(
            "rollout_executions", 1
        ),
        lambda value: value["qualification_access"].__setitem__("status", "UNLOCKED"),
        lambda value: value["qualification_access"].__setitem__(
            "qualification_execution_authorized", True
        ),
    )
    for mutate in mutations:
        broken = copy.deepcopy(payload)
        mutate(broken)
        assert contract.qualification_design_freeze_gate(broken)["passed"] is False


def test_exact_eleven_selected_records_exclude_label_and_p2() -> None:
    records = contract.R3_SELECTED_ARTIFACTS
    assert len(records) == 11
    assert set(records) == {
        "r3_protocol_freeze",
        "r3_design_audit",
        "r3_execution_lock",
        "r3_pipeline_claim",
        "r3_pipeline_ledger",
        "r3_fit_p1_receipt",
        "r3_fit_p1_gate",
        "r3_fit_p1_summary",
        "r3_probe_p1_receipt",
        "r3_probe_p1_gate",
        "r3_probe_p1_summary",
    }
    assert all(
        set(record) == {"path", "sha256", "size_bytes"} for record in records.values()
    )
    assert all("/label/" not in record["path"] for record in records.values())
    assert all("/fit/p2/" not in record["path"] for record in records.values())
    assert contract.CANDIDATE_SELECTION_POLICY["p2_is_lineage_evidence_only"] is True
    assert contract.CANDIDATE_SELECTION_POLICY["p2_reuse_authorized"] is False


def test_six_cases_are_condition_identical_to_r3_final_cases() -> None:
    assert contract.CASE_IDS == r3.FINAL_CASE_IDS
    assert len(contract.DEVELOPMENT_CASES) == 6
    fields = (
        "case_id",
        "action_selection",
        "episode_start_offset_s",
        "action_seed",
        "runtime_seed",
        "sigma",
    )
    for salvage, source in zip(contract.DEVELOPMENT_CASES, r3.FINAL_CASES, strict=True):
        assert {name: salvage[name] for name in fields} == {
            name: source[name] for name in fields
        }
        assert salvage["destination"].startswith(contract.DEVELOPMENT_ROOT.as_posix())
        assert "v12r3" not in salvage["destination"]
    assert len(contract.STAGE_IDS) == 7
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert not any("fit" in stage or "label" in stage for stage in contract.STAGE_IDS)


def test_paths_and_mutation_manifest_are_isolated_complete_and_unique() -> None:
    assert "v12p1s" in contract.RUN_ROOT.as_posix()
    assert contract.RUN_ROOT != r3.RUN_ROOT
    paths = contract.declared_mutation_paths()
    assert len(paths) == len(set(paths.values()))
    assert paths["protocol_freeze"] == contract.PROTOCOL_FREEZE_PATH
    assert paths["execution_lock"] == contract.EXECUTION_LOCK_PATH
    assert paths["pipeline_claim"] == contract.PIPELINE_CLAIM_PATH
    assert paths["pipeline_ledger"] == contract.PIPELINE_LEDGER_PATH
    for name, path in paths.items():
        assert (
            path == contract.PROTOCOL_FREEZE_PATH
            or path == contract.EXECUTION_LOCK_PATH
            or path == contract.RUN_ROOT
            or contract.RUN_ROOT in path.parents
        ), name
        assert "v12r3/h0_v12r3_run" not in path.as_posix()
    for case_id in contract.CASE_IDS:
        prefix = f"development_{case_id}"
        for suffix in (
            "steps_root",
            "run_start",
            "trace",
            "partial_summary",
            "summary",
            "gate",
            "receipt",
            "failure",
        ):
            assert f"{prefix}_{suffix}" in paths


def test_candidate_selection_passes_only_on_complete_exact_lineage() -> None:
    payload = _valid_selection()
    gate = contract.candidate_selection_gate(payload)
    assert gate["passed"] is True
    assert gate["candidate_selected_for_development"] is True
    assert gate["candidate_promoted"] is False

    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value.__setitem__("candidate_id", "other"),
        lambda value: value["candidate_module"].__setitem__("tree_sha256", "00" * 32),
        lambda value: value["r3_selected_artifacts"].pop("r3_probe_p1_gate"),
        lambda value: value["r3_terminal_run_tree"].__setitem__("file_count", 2_818),
        lambda value: value["r3_terminal_semantics"].__setitem__(
            "attempted_stage", "probe_p1"
        ),
        lambda value: value["selection_policy"].__setitem__(
            "eligible_candidates", ["p0", "p1"]
        ),
        lambda value: value.__setitem__("p1_probe_autonomy_passed", False),
        lambda value: value.__setitem__("label_p1_used_for_candidate_selection", True),
        lambda value: value.__setitem__("p2_artifacts_used", ["fit/p2"]),
        lambda value: value.__setitem__("actor_fit_executions", 1),
        lambda value: value.__setitem__("retry_authorized", True),
    )
    for mutate in mutations:
        broken = copy.deepcopy(payload)
        mutate(broken)
        assert contract.candidate_selection_gate(broken)["passed"] is False


@pytest.mark.parametrize("malformed", [None, [], "payload", 1, {}, {"passed": True}])
def test_candidate_selection_gate_is_total_and_fail_closed(malformed: Any) -> None:
    assert contract.candidate_selection_gate(malformed)["passed"] is False


@pytest.mark.parametrize("case_id", contract.CASE_IDS)
def test_each_exact_v26_development_rollout_passes(case_id: str) -> None:
    gate = contract.development_rollout_gate(_valid_rollout(case_id))
    assert gate["passed"] is True
    assert gate["case_id"] == case_id
    assert gate["candidate_promoted"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("steps", 499),
        ("trace_step_count", 499),
        ("control_window_count", 4_999),
        ("raw_sensor_sample_count", 4_999),
        ("phase_valid_cycle_count", 1),
        ("grf_penetration_max_m", 0.025),
        ("grf_penetration_max_m", float("nan")),
    ],
)
def test_rollout_physical_bounds_are_strict(field: str, value: Any) -> None:
    summary = _valid_rollout(contract.CASE_IDS[0])
    summary[field] = value
    assert contract.development_rollout_gate(summary)["passed"] is False


@pytest.mark.parametrize("counter", contract._ZERO_ROLLOUT_COUNTERS)
def test_each_forbidden_rollout_counter_fails_closed(counter: str) -> None:
    summary = _valid_rollout(contract.CASE_IDS[0])
    summary[counter] = 1
    gate = contract.development_rollout_gate(summary)
    assert gate["passed"] is False
    assert gate["checks"]["zero_invalids_fallbacks_updates"] is False


def test_rollout_rejects_v26_autonomy_provenance_and_p2_drift() -> None:
    base = _valid_rollout(contract.CASE_IDS[0])
    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value.__setitem__("binary_phase_fsm_mode", "disabled"),
        lambda value: value.__setitem__("event_contract_id", "legacy_events"),
        lambda value: value.__setitem__("morphology_weight", 1.0),
        lambda value: value.__setitem__("teacher_enabled", True),
        lambda value: value.__setitem__("blending_enabled", True),
        lambda value: value.__setitem__("safety_latch_enabled", True),
        lambda value: value["binary_phase_event_gate"].__setitem__(
            "left_non_v26_source_count", 1
        ),
        lambda value: value["binary_event_prefix_integrity"].__setitem__(
            "passed", False
        ),
        lambda value: value["trace"].__setitem__("path", "other/trace.json"),
        lambda value: value.__setitem__("p2_module_loaded", True),
        lambda value: value.__setitem__("candidate_fit_stage", "p2"),
    )
    for mutate in mutations:
        broken = copy.deepcopy(base)
        mutate(broken)
        assert contract.development_rollout_gate(broken)["passed"] is False


@pytest.mark.parametrize(
    ("field", "value", "failed_check"),
    [
        ("pipeline_id", "other", "pipeline_stage_exact"),
        ("stage_id", "development__other", "pipeline_stage_exact"),
        ("fit_executions", 1, "zero_invalids_fallbacks_updates"),
        ("logstd_byte_exact", False, "normalization_invariants"),
        ("disabled_clock_column_indices", [1, 0], "normalization_invariants"),
        ("latch_active_at_episode_end", True, "pure_autonomy"),
        ("rollout_executions", 0, "producer_activity_exact"),
        ("environment_reset_calls", 0, "producer_activity_exact"),
        ("environment_step_calls", 499, "producer_activity_exact"),
        ("candidate_mean_queries", 499, "producer_activity_exact"),
    ],
)
def test_rollout_rejects_each_producer_owned_contract_mutation(
    field: str, value: Any, failed_check: str
) -> None:
    summary = _valid_rollout(contract.CASE_IDS[0])
    summary[field] = value
    gate = contract.development_rollout_gate(summary)
    assert gate["passed"] is False
    assert gate["checks"][failed_check] is False


def test_rollout_rejects_authority_or_qualification_binding_mutation() -> None:
    authority = _valid_rollout(contract.CASE_IDS[0])
    authority["execution_authority"]["fit_authorized"] = True
    gate = contract.development_rollout_gate(authority)
    assert gate["checks"]["zero_authority_exact"] is False

    qualification = _valid_rollout(contract.CASE_IDS[0])
    qualification["qualification_design_freeze"]["sha256"] = "00" * 32
    qualification["qualification_design_freeze"]["path"] = "other.json"
    gate = contract.development_rollout_gate(qualification)
    assert gate["checks"]["qualification_design_bound"] is False


def test_rollout_requires_finite_diagnostics_and_zero_sea_saturation() -> None:
    base = _valid_rollout(contract.CASE_IDS[0])
    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value["episode_metrics"]["reserve_norm_nm"].__setitem__(
            "sample_count", 499
        ),
        lambda value: value["episode_metrics"]["residual_norm_nm"].__setitem__(
            "rms", float("nan")
        ),
        lambda value: value["sea_episode_metrics"]["pros_knee_angle"][
            "motor_power_w"
        ].__setitem__("abs_max", float("inf")),
        lambda value: value["sea_episode_metrics"]["pros_ankle_angle"][
            "tau_spring_rate_nm_s"
        ].__setitem__("sample_count", 5_000),
        lambda value: value["sea_episode_metrics"]["pros_knee_angle"][
            "tau_input_saturated"
        ].__setitem__("count", 1),
        lambda value: value["sea_episode_metrics"]["pros_ankle_angle"][
            "tau_input_saturated"
        ].__setitem__("sample_count", 4_999),
        lambda value: value["sea_episode_metrics"]["pros_ankle_angle"][
            "tau_input_saturated"
        ].__setitem__("fraction", float("nan")),
    )
    for mutate in mutations:
        broken = copy.deepcopy(base)
        mutate(broken)
        gate = contract.development_rollout_gate(broken)
        assert gate["passed"] is False
        assert not all(
            gate["checks"][name]
            for name in (
                "episode_metrics_finite_exact",
                "sea_metrics_finite_exact",
                "sea_saturation_zero",
            )
        )


@pytest.mark.parametrize("malformed", [None, [], "summary", 1, {}, {"steps": 500}])
def test_rollout_gate_is_total_and_fail_closed(malformed: Any) -> None:
    assert contract.development_rollout_gate(malformed)["passed"] is False


def test_final_development_requires_six_of_six_without_promotion() -> None:
    summary = _valid_final()
    gate = contract.final_development_gate(summary)
    assert gate["passed"] is True
    assert gate["qualification_eligible"] is True
    assert gate["candidate_promoted"] is False
    assert gate["runtime_promoted"] is False
    assert gate["next_stage"] == "WAIT_INDEPENDENT_QUALIFICATION_PROTOCOL"


def test_final_development_has_no_averaging_compensation_or_p2_escape() -> None:
    base = _valid_final()
    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value["rollout_bindings"].pop(),
        lambda value: value["rollout_bindings"][0].__setitem__("passed", False),
        lambda value: value["rollout_bindings"].__setitem__(
            0, copy.deepcopy(value["rollout_bindings"][1])
        ),
        lambda value: value.__setitem__("passing_rollout_count", 5),
        lambda value: value.__setitem__("failed_rollout_count", 1),
        lambda value: value.__setitem__("actor_fit_executions", 1),
        lambda value: value.__setitem__("offline_teacher_label_calls", 1),
        lambda value: value.__setitem__("p2_corpus_loaded", True),
        lambda value: value.__setitem__("qualification_executed", True),
        lambda value: value.__setitem__("checkpoint_zero_created", True),
        lambda value: value.__setitem__("retry_authorized", True),
    )
    for mutate in mutations:
        broken = copy.deepcopy(base)
        mutate(broken)
        assert contract.final_development_gate(broken)["passed"] is False


def test_final_gate_rejects_producer_activity_authority_and_provenance_drift() -> None:
    mutations: tuple[Callable[[dict[str, Any]], None], ...] = (
        lambda value: value.__setitem__("pipeline_id", "other"),
        lambda value: value.__setitem__("stage_id", "other"),
        lambda value: value.__setitem__("fit_executions", 1),
        lambda value: value.__setitem__("teacher_queries", 1),
        lambda value: value.__setitem__("rollout_environment_reset_calls", 5),
        lambda value: value.__setitem__("rollout_environment_step_calls", 2_999),
        lambda value: value.__setitem__("candidate_mean_queries", 2_999),
        lambda value: value.__setitem__("compensation_authorized", True),
        lambda value: value["execution_authority"].__setitem__(
            "teacher_authorized", True
        ),
        lambda value: value["qualification_design_freeze_gate"].__setitem__(
            "passed", False
        ),
    )
    for mutate in mutations:
        broken = _valid_final()
        mutate(broken)
        assert contract.final_development_gate(broken)["passed"] is False


@pytest.mark.parametrize("malformed", [None, [], "summary", 1, {}, {"passed": True}])
def test_final_development_gate_is_total_and_fail_closed(malformed: Any) -> None:
    assert contract.final_development_gate(malformed)["passed"] is False


def test_stage_and_artifact_helpers_are_exact_and_fail_closed() -> None:
    for index, stage_id in enumerate(contract.STAGE_IDS, start=1):
        descriptor = contract.stage_descriptor(stage_id)
        assert descriptor["kind"] in {"development", "finalize_development"}
        assert contract.worker_claim_path(stage_id).name.startswith(f"{index:02d}_")
        assert contract.stage_receipt_path(stage_id).name == "receipt.json"
    with pytest.raises(ValueError):
        contract.stage_descriptor("fit_p2")
    with pytest.raises(ValueError):
        contract.worker_claim_path("label_p1")
    with pytest.raises(ValueError):
        contract.canonical_development_case("unknown")

    path = contract.RUN_ROOT / "artifact.json"
    assert contract.artifact_record_matches(_artifact(path), path) is True
    assert contract.artifact_record_matches(None, path) is False
    malformed = _artifact(path)
    malformed["sha256"] = "not-a-hash"
    assert contract.artifact_record_matches(malformed, path) is False
