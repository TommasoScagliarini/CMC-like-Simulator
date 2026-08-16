"""Temporary-root tests for the source-only V12R5-Q3 runtime."""

from __future__ import annotations

import copy
import os
import sys
from collections.abc import Callable
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


RUNTIME_ROOT = Path(__file__).resolve().parent
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

import freeze_h0_v12r5_q3_qualification_protocol as protocol  # noqa: E402
import h0_v12r5_q3_artifacts as artifacts  # noqa: E402
import h0_v12r5_q3_qualification_gates as gates  # noqa: E402
import h0_v12r5_q3_runtime_contract as contract  # noqa: E402
import prepare_h0_v12r5_q3_qualification_noise_tapes as noise  # noqa: E402
import run_h0_v12r5_q3_qualification as runner  # noqa: E402
import verify_h0_v12r5_q3_prerequisites as prerequisites  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_binding() -> Any:
    contract.clear_candidate_binding_for_tests()
    yield
    contract.clear_candidate_binding_for_tests()


def _candidate() -> tuple[str, dict[str, Any]]:
    files = [{"path": "module_state.pkl", "sha256": "b" * 64, "size_bytes": 123}]
    tree_sha256 = contract._tree_digest(files)
    module = {
        "path": contract.R5_CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": tree_sha256,
        "file_count": 1,
        "files": files,
    }
    return f"AB06_H0_V12R5_CASE_BALANCED:{tree_sha256}", module


def _selection() -> dict[str, Any]:
    return {
        "rule": contract.R5_CANDIDATE_SELECTION_RULE,
        "module_path": contract.R5_CANDIDATE_MODULE_PATH.as_posix(),
        "candidate_id": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
        "candidate_tree_sha256": "DEFERRED_UNTIL_CASE_BALANCED_FIT",
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    noise.forensic.write_json_exclusive(path, payload)


def _temp_r5_prerequisites(
    tmp_path: Path,
) -> tuple[
    dict[str, Path],
    dict[str, dict[str, Any]],
    dict[str, dict[str, Any]],
]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    candidate_id, candidate_module = _candidate()
    requirements = contract.prerequisite_requirements()
    names = [row["name"] for row in requirements]
    paths = {
        name: tmp_path / f"{index:02d}_{name}.json" for index, name in enumerate(names)
    }

    protocol_payload = {
        "status": requirements[0]["required_status"],
        "passed": True,
        "candidate_selection": _selection(),
    }
    _write_json(paths[names[0]], protocol_payload)
    protocol_record = artifacts.record(
        paths[names[0]], logical_path=paths[names[0]].absolute().as_posix()
    )

    lock_payload = {
        "status": requirements[1]["required_status"],
        "passed": True,
        "candidate_selection": _selection(),
    }
    _write_json(paths[names[1]], lock_payload)
    lock_record = artifacts.record(
        paths[names[1]], logical_path=paths[names[1]].absolute().as_posix()
    )

    candidate_payload = {
        "status": requirements[2]["required_status"],
        "passed": True,
        "candidate_selection_rule": contract.R5_CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
    }
    _write_json(paths[names[2]], candidate_payload)
    candidate_record = artifacts.record(
        paths[names[2]], logical_path=paths[names[2]].absolute().as_posix()
    )

    final_payload = {
        "status": requirements[3]["required_status"],
        "passed": True,
        "candidate_selection_rule": contract.R5_CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
        "candidate_freeze": candidate_record,
        "runtime_promoted": False,
    }
    _write_json(paths[names[3]], final_payload)
    final_record = artifacts.record(
        paths[names[3]], logical_path=paths[names[3]].absolute().as_posix()
    )

    ledger_payload = {
        "status": requirements[4]["required_status"],
        "passed": True,
        "terminal": True,
        "candidate_selection_rule": contract.R5_CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
        "candidate_freeze": candidate_record,
        "final_development_receipt": final_record,
        "protocol_freeze": protocol_record,
        "execution_lock": lock_record,
        "q3_paths_opened": [],
        "runtime_promoted": False,
    }
    _write_json(paths[names[4]], ledger_payload)

    payloads = {
        names[0]: protocol_payload,
        names[1]: lock_payload,
        names[2]: candidate_payload,
        names[3]: final_payload,
        names[4]: ledger_payload,
    }
    records = {
        name: artifacts.record(
            paths[name], logical_path=paths[name].absolute().as_posix()
        )
        for name in names
    }
    return paths, payloads, records


def _bundle(
    payloads: dict[str, dict[str, Any]],
    calls: list[str] | None = None,
) -> prerequisites.R5VerifierBundle:
    names = [row["name"] for row in contract.prerequisite_requirements()]

    def callback(name: str) -> Callable[[], dict[str, Any]]:
        def run() -> dict[str, Any]:
            if calls is not None:
                calls.append(name)
            return copy.deepcopy(payloads[name])

        return run

    return prerequisites.R5VerifierBundle(*(callback(name) for name in names))


def _canonical_gate_snapshot(gate: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(gate)
    requirements = contract.prerequisite_requirements()
    for row, requirement in zip(result["prerequisites"], requirements, strict=True):
        row["artifact"]["path"] = requirement["path"]
    return result


def _artifact(path: str | PurePosixPath) -> dict[str, Any]:
    return {
        "path": PurePosixPath(path).as_posix(),
        "sha256": "a" * 64,
        "size_bytes": 1,
    }


def _source_tree() -> dict[str, Any]:
    return {
        **contract.SOURCE_H0_MODULE,
        "file_count": 1,
        "files": [{"path": "x", "sha256": "a" * 64, "size_bytes": 1}],
    }


def _metric(sample_count: int, value: float = 1.0) -> dict[str, Any]:
    return {"sample_count": sample_count, "rms": value, "abs_max": value}


def _valid_rollout(role: str, case_id: str) -> dict[str, Any]:
    expected = contract.canonical_rollout(role, case_id)
    root = PurePosixPath(expected["destination"])
    sea = {
        joint: {
            **{
                signal: _metric(contract.SEA_EXPECTED_SAMPLE_COUNTS[signal])
                for signal in contract.SEA_SIGNALS
            },
            "tau_input_saturated": {
                "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
                "count": 0,
                "fraction": 0.0,
            },
        }
        for joint in contract.JOINTS
    }
    result = {
        **copy.deepcopy(expected),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "actor_module": (
            copy.deepcopy(contract.R5_CANDIDATE_MODULE)
            if role == contract.CANDIDATE_ROLE
            else _source_tree()
        ),
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
        "grf_penetration_max_m": 0.01,
        "episode_metrics": {
            "reserve_norm_nm": _metric(contract.EXPECTED_STEPS),
            "residual_norm_nm": _metric(contract.EXPECTED_STEPS, 1.0e-7),
        },
        "sea_episode_metrics": sea,
        "random_noise_draw_count": (
            contract.EXPECTED_STEPS
            if expected["action_selection"] == "stochastic"
            else 0
        ),
        "single_noise_application_count": contract.EXPECTED_STEPS,
        "noise_tape": _artifact(expected["noise_tape"]),
        "noise_tape_array_sha256": contract.EXPECTED_TAPE_ARRAY_SHA256[
            PurePosixPath(expected["noise_tape"]).name
        ],
        "protocol_freeze": _artifact(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _artifact(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(
            contract.worker_claim_path(f"rollout__{role}__{case_id}")
        ),
        "run_start": _artifact(root / "run_start.json"),
        "trace": _artifact(root / "trace.json"),
        "partial_summary": _artifact(root / "partial_summary.json"),
        "prerequisite_gate_passed": True,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        **{name: 0 for name in contract.ZERO_REQUIRED_COUNTS},
    }
    if role == contract.BASELINE_ROLE:
        result["legacy_event_integrity_passed"] = True
    else:
        result["binary_phase_event_gate"] = {
            "passed": True,
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        }
    return result


def _valid_aggregate() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.R5_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.R5_CANDIDATE_MODULE),
        "prerequisite_gate_passed": True,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": [
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
        ],
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
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "compensation_or_averaging_used": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def test_contract_binds_only_the_exact_digest_and_candidate_id(tmp_path: Path) -> None:
    del tmp_path
    candidate_id, module = _candidate()

    with pytest.raises(contract.Q3CandidateBindingError, match="deferred"):
        contract.current_candidate_binding()
    assert contract.bind_candidate(candidate_id, module) == {
        "candidate_id": candidate_id,
        "candidate_module": module,
    }
    assert contract.role_contract(contract.CANDIDATE_ROLE)["actor_module"] == module

    drift = copy.deepcopy(module)
    drift["files"][0]["size_bytes"] += 1
    with pytest.raises(contract.Q3CandidateBindingError, match="digest"):
        contract.validate_candidate_binding(candidate_id, drift)


def test_all_five_official_verifier_results_are_called_and_cross_bound(
    tmp_path: Path,
) -> None:
    paths, payloads, _records = _temp_r5_prerequisites(tmp_path)
    calls: list[str] = []

    result = prerequisites.load_and_verify_r5_prerequisites(
        verifier_bundle=_bundle(payloads, calls),
        prerequisite_paths=paths,
        require_q3_unopened=False,
    )

    names = [row["name"] for row in contract.prerequisite_requirements()]
    assert calls == names
    assert result["gate"]["passed"] is True
    assert result["gate"]["official_verifier_count"] == 5
    assert gates.r5_prerequisite_gate(result["gate"])["passed"] is True
    assert contract.current_candidate_binding()["candidate_id"] == _candidate()[0]


def test_preflight_explicitly_rejects_terminal_fail_without_q3_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    paths, payloads, _records = _temp_r5_prerequisites(tmp_path / "r5")
    names = list(payloads)
    calls: list[str] = []

    def value(name: str) -> Callable[[], dict[str, Any]]:
        def callback() -> dict[str, Any]:
            calls.append(name)
            if name in {names[2], names[3]}:
                raise FileNotFoundError("R5 stopped before candidate freeze")
            result = copy.deepcopy(payloads[name])
            if name == names[4]:
                result["status"] = "FAIL_H0_V12R5_CASE_BALANCED_PIPELINE_TERMINAL"
                result["passed"] = False
                result["attempted_stage"] = "fit_case_balanced_candidate"
            return result

        return callback

    bundle = prerequisites.R5VerifierBundle(*(value(name) for name in names))
    q3_output = tmp_path / "q3-output"
    run_preflight = prerequisites.load_and_verify_r5_prerequisites
    monkeypatch.setattr(
        noise,
        "verified_design_binding",
        lambda: {
            "record": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
            "payload": {"design_snapshot": {}},
        },
    )
    monkeypatch.setattr(
        noise.prerequisites_module,
        "load_and_verify_r5_prerequisites",
        lambda: run_preflight(
            verifier_bundle=bundle,
            prerequisite_paths=paths,
            require_q3_unopened=False,
        ),
    )

    with pytest.raises(
        prerequisites.V12R5Q3PrerequisiteError, match="R5_TERMINAL_NOT_PASS"
    ):
        noise.prepare(
            output_root=q3_output,
            enforce_canonical_destination=False,
        )

    assert calls == names
    assert not q3_output.exists()
    with pytest.raises(contract.Q3CandidateBindingError, match="deferred"):
        contract.current_candidate_binding()


def test_prerequisite_cross_binding_rejects_candidate_or_chain_drift(
    tmp_path: Path,
) -> None:
    _paths, payloads, records = _temp_r5_prerequisites(tmp_path)
    names = list(payloads)
    payloads[names[-1]]["candidate_id"] = "wrong"

    gate = prerequisites.validate_verified_payloads(
        payloads, records, enforce_canonical_record_paths=False
    )

    assert gate["passed"] is False
    assert gate["checks"]["terminal_candidate_binding_exact"] is False


def test_reference_tapes_match_preregistered_hashes_without_writes(
    tmp_path: Path,
) -> None:
    output = tmp_path / "not-created"

    tapes = noise.build_tapes()

    assert {
        name: noise.array_sha256(row["standard_normal"]) for name, row in tapes.items()
    } == contract.EXPECTED_TAPE_ARRAY_SHA256
    assert not output.exists()


def test_prepare_and_verify_tapes_only_in_tmp_and_no_clobber(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _paths, payloads, records = _temp_r5_prerequisites(tmp_path / "r5")
    gate = prerequisites.validate_verified_payloads(
        payloads, records, enforce_canonical_record_paths=False
    )
    gate = _canonical_gate_snapshot(gate)
    snapshot = {"gate": gate, "records": records}
    monkeypatch.setattr(
        noise,
        "verified_design_binding",
        lambda: {
            "record": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
            "payload": {
                "design_snapshot": {
                    "holdout_cases": list(contract.HOLDOUT_CASES),
                    "rollout_matrix": list(contract.ROLLOUT_MATRIX),
                }
            },
        },
    )
    output = tmp_path / "q3-noise"

    manifest = noise.prepare(
        output_root=output,
        prerequisites=snapshot,
        enforce_canonical_destination=False,
    )

    assert manifest["passed"] is True
    assert manifest["update_activity"] == {
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert (
        noise.verify_manifest(noise_root=output, enforce_canonical_destination=False)
        == manifest
    )
    with pytest.raises(noise.V12R5Q3QualificationNoiseError, match="clobber"):
        noise.prepare(
            output_root=output,
            prerequisites=snapshot,
            enforce_canonical_destination=False,
        )


def test_independent_common_pair_and_aggregate_gates_pass_exact_shapes(
    tmp_path: Path,
) -> None:
    del tmp_path
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    case_id = contract.CASE_IDS[0]
    baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
    candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)

    assert (
        gates.common_rollout_gate(
            baseline, role=contract.BASELINE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.common_rollout_gate(
            candidate, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is True
    )
    assert (
        gates.condition_matched_gate(baseline, candidate, case_id=case_id)["passed"]
        is True
    )
    assert gates.aggregate_qualification_gate(_valid_aggregate())["passed"] is True


def test_gates_reject_detector_morphology_noninferiority_and_update_drift(
    tmp_path: Path,
) -> None:
    del tmp_path
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    case_id = contract.CASE_IDS[0]
    baseline = _valid_rollout(contract.BASELINE_ROLE, case_id)
    candidate = _valid_rollout(contract.CANDIDATE_ROLE, case_id)

    detector_drift = copy.deepcopy(candidate)
    detector_drift["binary_phase_event_gate"]["fallback_count"] = 1
    assert (
        gates.common_rollout_gate(
            detector_drift, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is False
    )

    morphology_drift = copy.deepcopy(candidate)
    morphology_drift["morphology_weight"] = 0.0025
    assert (
        gates.common_rollout_gate(
            morphology_drift, role=contract.CANDIDATE_ROLE, case_id=case_id
        )["passed"]
        is False
    )

    noninferior_drift = copy.deepcopy(candidate)
    noninferior_drift["episode_metrics"]["reserve_norm_nm"]["rms"] = 100.0
    noninferior_drift["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 100.0
    assert (
        gates.condition_matched_gate(baseline, noninferior_drift, case_id=case_id)[
            "passed"
        ]
        is False
    )

    aggregate = _valid_aggregate()
    aggregate["actor_updates"] = 1
    aggregate["update_activity"]["actor_updates"] = 1
    assert gates.aggregate_qualification_gate(aggregate)["passed"] is False


def test_protocol_assembly_binds_matrix_sources_and_zero_activity(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    del tmp_path
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    gate = {
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": module,
    }
    record = {"path": "x", "sha256": "a" * 64, "size_bytes": 1}
    monkeypatch.setattr(
        noise,
        "verified_design_binding",
        lambda: {
            "record": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
            "payload": {
                "design_snapshot": {
                    "holdout_cases": list(contract.HOLDOUT_CASES),
                    "rollout_matrix": list(contract.ROLLOUT_MATRIX),
                }
            },
        },
    )
    monkeypatch.setattr(
        protocol,
        "_prerequisite_snapshot",
        lambda: {
            "gate": gate,
            "pure_gate": {"passed": True},
            "records": {"r5": record},
        },
    )
    monkeypatch.setattr(
        protocol,
        "_noise_gate",
        lambda: {
            "passed": True,
            "manifest_record": record,
            "tapes": {},
        },
    )
    monkeypatch.setattr(
        protocol,
        "_source_gate",
        lambda: {"passed": True, "checks": {"closed": True}, "records": {}},
    )
    monkeypatch.setattr(
        protocol,
        "_input_gate",
        lambda binding=None: {
            "passed": True,
            "checks": {"bound": True},
            "records": {"candidate_module": module},
        },
    )

    payload = protocol._assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "qualification_run_root_absent": True,
            "noise_manifest_present": True,
        }
    )

    assert payload["passed"] is True
    assert payload["selected_candidate_id"] == candidate_id
    assert payload["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert all(value == 0 for value in payload["zero_freeze_activity"].values())


def test_runner_is_q1_independent_baseline_first_and_zero_update(
    tmp_path: Path,
) -> None:
    del tmp_path
    runner_source = Path(runner.__file__).read_text(encoding="utf-8")
    gates_source = Path(gates.__file__).read_text(encoding="utf-8")

    assert "run_h0_v12r3_p1_qualification" not in runner_source
    assert "_Q1_ENGINE" not in gates_source
    assert list(contract.STAGE_IDS[:6]) == [
        f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert list(contract.STAGE_IDS[6:12]) == [
        f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS
    ]
    assert contract.STAGE_IDS[-1] == "finalize_qualification"
    assert runner.EXPECTED_TERMINAL_ACTIVITY["actor_updates"] == 0
    assert runner.EXPECTED_TERMINAL_ACTIVITY["critic_updates"] == 0
    assert runner.EXPECTED_TERMINAL_ACTIVITY["ppo_updates"] == 0
    assert runner.NEXT_STAGE_AFTER_PASS == "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"


def test_runtime_source_gate_is_closed_and_binds_official_verifiers(
    tmp_path: Path,
) -> None:
    del tmp_path

    source_gate = protocol._source_gate()

    assert source_gate["passed"] is True
    assert len(source_gate["records"]) == 22
    assert source_gate["records"]["q1_artifact_predicate_source"]["path"].endswith(
        "v12p1q/h0_v12r3_p1_qualification_gates.py"
    )
    assert source_gate["records"]["r5_official_protocol_verifier"]["path"].endswith(
        "v12r5/freeze_h0_v12r5_case_balanced.py"
    )
    assert source_gate["records"]["r5_official_runtime_verifier"]["path"].endswith(
        "v12r5/run_h0_v12r5_case_balanced.py"
    )


def test_terminal_ledger_payload_stops_and_never_creates_checkpoint_zero(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    candidate_id, module = _candidate()
    contract.bind_candidate(candidate_id, module)
    protocol_path = tmp_path / "protocol.json"
    lock_path = tmp_path / "lock.json"
    claim_path = tmp_path / "claim.json"
    for path in (protocol_path, lock_path, claim_path):
        _write_json(path, {"path": path.name})
    monkeypatch.setattr(runner, "PROTOCOL_FREEZE_PATH", protocol_path)
    monkeypatch.setattr(runner, "LOCK_PATH", lock_path)
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", claim_path)
    for name in runner._ACTIVITY:
        runner._ACTIVITY[name] = 0

    failed = runner._ledger_payload(
        passed=False,
        attempted_stage=None,
        completed_stages=[],
        error=RuntimeError("fit lineage unavailable"),
    )

    assert failed["next_stage"] == "STOP_TERMINAL"
    assert failed["checkpoint_zero_created"] is False
    assert failed["positive_morphology_enabled"] is False
    assert failed["update_activity"] == {
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def test_no_q3_canonical_runtime_outputs_are_created(tmp_path: Path) -> None:
    del tmp_path
    for path in (
        contract.NOISE_ROOT,
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.RUN_ROOT,
    ):
        assert not os.path.lexists(artifacts.resolve_relative(path))
