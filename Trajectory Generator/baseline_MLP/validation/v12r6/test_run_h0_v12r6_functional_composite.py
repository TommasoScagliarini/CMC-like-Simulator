"""Mocked execution tests for the V12R6 functional-composite runner."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

import numpy as np
import pytest


V12R6_ROOT = Path(__file__).resolve().parent
if os.fspath(V12R6_ROOT) not in sys.path:
    sys.path.insert(0, os.fspath(V12R6_ROOT))

import run_h0_v12r6_functional_composite as runner  # noqa: E402


TREE_SHA256 = "a" * 64


def _publish(path: Path, payload: Any) -> Path:
    return runner.forensic.write_json_exclusive(path, payload)


def _sandbox(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Path]:
    root = tmp_path.resolve()
    monkeypatch.setattr(runner, "REPO_ROOT", root)
    relative = {
        "protocol": PurePosixPath("state/protocol_freeze.json"),
        "lock": PurePosixPath("state/execution_lock.json"),
        "run": PurePosixPath("run/h0_v12r6"),
        "claim": PurePosixPath("run/h0_v12r6/pipeline_claim.json"),
        "ledger": PurePosixPath("run/h0_v12r6/pipeline_ledger.json"),
        "attestation": PurePosixPath(
            "run/h0_v12r6/locked_input_attestation_receipt.json"
        ),
        "candidate_root": PurePosixPath("run/h0_v12r6/candidate"),
        "candidate_module": PurePosixPath("run/h0_v12r6/candidate/rl_module_composite"),
        "synthesis_summary": PurePosixPath(
            "run/h0_v12r6/candidate/composite_synthesis_summary.json"
        ),
        "synthesis_gate": PurePosixPath(
            "run/h0_v12r6/candidate/composite_synthesis_gate.json"
        ),
        "synthesis_receipt": PurePosixPath(
            "run/h0_v12r6/candidate/composite_synthesis_receipt.json"
        ),
        "candidate_freeze": PurePosixPath("run/h0_v12r6/candidate_freeze_receipt.json"),
        "final": PurePosixPath("run/h0_v12r6/final_development_receipt.json"),
    }
    contract = runner.contract
    monkeypatch.setattr(contract, "PROTOCOL_FREEZE_PATH", relative["protocol"])
    monkeypatch.setattr(contract, "EXECUTION_LOCK_PATH", relative["lock"])
    monkeypatch.setattr(contract, "RUN_ROOT", relative["run"])
    monkeypatch.setattr(contract, "PIPELINE_CLAIM_PATH", relative["claim"])
    monkeypatch.setattr(contract, "PIPELINE_LEDGER_PATH", relative["ledger"])
    monkeypatch.setattr(contract, "ATTESTATION_RECEIPT_PATH", relative["attestation"])
    monkeypatch.setattr(contract, "CANDIDATE_ROOT", relative["candidate_root"])
    monkeypatch.setattr(
        contract,
        "CANDIDATE_MODULE_PATH",
        relative["candidate_module"],
    )
    monkeypatch.setattr(
        contract,
        "SYNTHESIS_SUMMARY_PATH",
        relative["synthesis_summary"],
    )
    monkeypatch.setattr(contract, "SYNTHESIS_GATE_PATH", relative["synthesis_gate"])
    monkeypatch.setattr(
        contract,
        "SYNTHESIS_RECEIPT_PATH",
        relative["synthesis_receipt"],
    )
    monkeypatch.setattr(
        contract,
        "CANDIDATE_FREEZE_PATH",
        relative["candidate_freeze"],
    )
    monkeypatch.setattr(contract, "FINAL_DEVELOPMENT_RECEIPT_PATH", relative["final"])
    development_paths = {
        case_id: PurePosixPath(f"run/h0_v12r6/development/{case_id}")
        for case_id in contract.DEVELOPMENT_CASE_IDS
    }
    monkeypatch.setattr(contract, "DEVELOPMENT_ROOT", relative["run"] / "development")
    monkeypatch.setattr(contract, "DEVELOPMENT_PATHS", development_paths)

    absolute = {name: root.joinpath(*path.parts) for name, path in relative.items()}
    monkeypatch.setattr(runner, "RUN_ROOT", absolute["run"])
    monkeypatch.setattr(runner, "PIPELINE_CLAIM_PATH", absolute["claim"])
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", absolute["ledger"])
    monkeypatch.setattr(runner, "ATTESTATION_RECEIPT_PATH", absolute["attestation"])
    monkeypatch.setattr(runner, "CANDIDATE_ROOT", absolute["candidate_root"])
    monkeypatch.setattr(runner, "CANDIDATE_MODULE_PATH", absolute["candidate_module"])
    monkeypatch.setattr(
        runner,
        "SYNTHESIS_SUMMARY_PATH",
        absolute["synthesis_summary"],
    )
    monkeypatch.setattr(runner, "SYNTHESIS_GATE_PATH", absolute["synthesis_gate"])
    monkeypatch.setattr(
        runner,
        "SYNTHESIS_RECEIPT_PATH",
        absolute["synthesis_receipt"],
    )
    monkeypatch.setattr(
        runner,
        "CANDIDATE_FREEZE_PATH",
        absolute["candidate_freeze"],
    )
    monkeypatch.setattr(runner, "FINAL_DEVELOPMENT_RECEIPT_PATH", absolute["final"])

    _publish(absolute["protocol"], {"passed": True})
    _publish(absolute["lock"], {"passed": True})
    return {"root": root, **absolute}


def _valid_offline() -> dict[str, Any]:
    names = {
        "global",
        "p2_subset",
        "nominal_r4_pass",
        "nominal_r4_student_exposed",
        *(f"case::{case_id}" for case_id in runner.contract.CASE_IDS),
    }
    return {
        "passed": True,
        "failed_checks": [],
        "metrics": {
            name: {
                "rmse": 0.001,
                "max_abs_error": 0.01,
                "reset_max_abs_error": 0.001,
            }
            for name in names
        },
        "critical_window_metrics": {"rmse": 0.001, "max_abs_error": 0.01},
        "p2_critical_window_metrics": {"rmse": 0.002, "max_abs_error": 0.02},
    }


def _valid_verification() -> dict[str, Any]:
    return {
        "passed": True,
        "max_abs_error_before_save": 1.0e-6,
        "max_abs_error_after_reload": 1.0e-6,
        "logstd_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "actor_feature_manifest_valid": True,
        "warm_start_target_512_compatible": True,
        "actor_digest": "b" * 64,
    }


def _candidate_tree() -> dict[str, Any]:
    return {
        "path": runner.contract.CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": TREE_SHA256,
        "file_count": 3,
        "files": [],
    }


def _stage_receipt_path(stage_id: str) -> Path:
    descriptor = runner.contract.stage_descriptor(stage_id)
    if descriptor["kind"] == "attestation":
        return runner.ATTESTATION_RECEIPT_PATH
    if descriptor["kind"] == "synthesis":
        return runner.SYNTHESIS_RECEIPT_PATH
    if descriptor["kind"] == "candidate_freeze":
        return runner.CANDIDATE_FREEZE_PATH
    if descriptor["kind"] == "development":
        case_id = descriptor["case"]["case_id"]
        return runner.resolve_relative(runner.contract.DEVELOPMENT_PATHS[case_id]) / (
            "receipt.json"
        )
    return runner.FINAL_DEVELOPMENT_RECEIPT_PATH


def test_offline_evaluation_normalizes_all_locked_slices(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = _sandbox(tmp_path, monkeypatch)
    corpus_path = state["root"] / "inputs/corpus.npz"
    trace_path = state["root"] / "inputs/r4_trace.json"
    p2_path = state["root"] / "inputs/p2"
    candidate_path = state["root"] / "inputs/candidate"
    p2_path.mkdir(parents=True)
    candidate_path.mkdir(parents=True)

    row_count = 9232
    observations = np.zeros((row_count, 35), dtype=np.float32)
    targets = np.zeros((row_count, 2), dtype=np.float32)
    reset_mask = np.ones(row_count, dtype=np.bool_)
    case_ids = np.asarray(
        [runner.contract.CASE_IDS[index % 6] for index in range(row_count)]
    )
    tranche_ids = np.asarray(
        [runner.contract.CRITICAL_WINDOW["tranche_id"]] * row_count
    )
    step_indices = np.asarray(
        [(index % 500) + 1 for index in range(row_count)],
        dtype=np.int64,
    )
    feature_names = np.asarray([f"actor_{index}" for index in range(35)])
    corpus_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        corpus_path,
        observations=observations,
        actions=targets,
        reset_mask=reset_mask,
        case_ids=case_ids,
        tranche_ids=tranche_ids,
        step_indices=step_indices,
        actor_feature_names=feature_names,
    )
    _publish(
        trace_path,
        [
            {
                "step": step,
                "effective_alpha": 0.5,
                "safety_latch_active": False,
            }
            for step in range(1, 501)
        ],
    )
    monkeypatch.setattr(
        runner.contract,
        "R5_CORPUS_ARTIFACT",
        {"path": corpus_path.relative_to(state["root"]).as_posix()},
    )
    monkeypatch.setattr(
        runner.contract,
        "R4_NOMINAL_TRACE_ARTIFACT",
        {"path": trace_path.relative_to(state["root"]).as_posix()},
    )
    monkeypatch.setattr(
        runner.contract,
        "P2_MODULE_PATH",
        PurePosixPath(p2_path.relative_to(state["root"]).as_posix()),
    )

    class FakeModule:
        def __init__(self, label: str) -> None:
            self.label = label

        def eval(self) -> None:
            return None

    from ray.rllib.core.rl_module.rl_module import RLModule

    monkeypatch.setattr(
        RLModule,
        "from_checkpoint",
        classmethod(lambda _cls, path: FakeModule(Path(path).name)),
    )

    def logits(module: FakeModule, values: np.ndarray) -> np.ndarray:
        mean = (
            np.zeros((len(values), 2), dtype=np.float32)
            if module.label == candidate_path.name
            else np.full((len(values), 2), 0.002, dtype=np.float32)
        )
        return np.concatenate(
            (mean, np.full_like(mean, np.log(np.float32(0.005)))),
            axis=1,
        )

    monkeypatch.setattr(runner.composite, "_logits", logits)
    result = runner._offline_evaluation(candidate_path)

    assert result["passed"] is True
    assert result["failed_checks"] == []
    assert result["rows"] == 9232
    assert result["actor_feature_names"] == feature_names.tolist()
    assert set(result["metrics"]) == {
        "global",
        "p2_subset",
        "nominal_r4_pass",
        "nominal_r4_student_exposed",
        *(f"case::{case_id}" for case_id in runner.contract.CASE_IDS),
    }
    assert result["metrics"]["global"] == {
        "rmse": 0.0,
        "max_abs_error": 0.0,
        "reset_max_abs_error": 0.0,
    }
    assert result["critical_window_metrics"] == {
        "rmse": 0.0,
        "max_abs_error": 0.0,
    }
    assert result["p2_critical_window_metrics"]["rmse"] == pytest.approx(0.002)
    assert (
        runner._triplet_passes(
            {"rmse": False, "max_abs_error": 0.0, "reset_max_abs_error": 0.0}
        )
        is False
    )
    with pytest.raises(runner.V12R6ExecutionError, match="empty"):
        runner._metric_triplet(
            targets,
            targets,
            reset_mask,
            np.zeros(row_count, dtype=np.bool_),
            np=np,
        )


def test_functional_verification_normalizes_three_reload_phases(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = _sandbox(tmp_path, monkeypatch)
    module = state["candidate_module"]
    module.mkdir(parents=True)
    digest = "c" * 64
    state_sha = "d" * 64
    _publish(
        module / runner.composite.ACTOR_FEATURE_MANIFEST,
        {
            "status": "H0_V12R6_COMPOSITE_ACTOR_FEATURE_CONTRACT",
            "actor_feature_count": 35,
            "fcnet_hiddens": [512, 512],
            "actor_digest": digest,
            "module_state_sha256": state_sha,
        },
    )

    def phase(error: float) -> dict[str, Any]:
        direct = {
            "mean_max_abs_error": error,
            "composite_logstd_byte_exact": True,
        }
        return {
            "passed": True,
            "corpus_direct_equivalence": copy.deepcopy(direct),
            "deterministic_direct_equivalence": copy.deepcopy(direct),
            "topology": {
                "actor_digest": digest,
                "checks": {"disabled_clock_columns_positive_zero": True},
            },
            "clock_invariance": {"output_byte_exact_under_clock_perturbation": True},
        }

    manifest = {
        "actor_digest": digest,
        "module_state_sha256": state_sha,
        "before_save": phase(0.5e-6),
        "after_reload": phase(0.8e-6),
        "final_reload": phase(1.0e-6),
        "warm_start_and_checkpoint_zero_512": {
            "passed": True,
            "required_target_fcnet_hiddens": [512, 512],
            "warm_start_transplant_actor_exact": True,
            "checkpoint_zero_standard_actor_transplant_exact": True,
            "fresh_critic_preserved_exact": True,
            "forward_surface_byte_exact": True,
        },
    }
    result = runner._functional_verification(manifest)
    assert result["passed"] is True
    assert result["max_abs_error_before_save"] == 0.5e-6
    assert result["max_abs_error_after_reload"] == 1.0e-6
    assert result["save_reload_exact"] is True
    assert result["warm_start_target_512_compatible"] is True

    malformed = copy.deepcopy(manifest)
    malformed["after_reload"]["clock_invariance"][
        "output_byte_exact_under_clock_perturbation"
    ] = 1
    assert runner._functional_verification(malformed)["passed"] is False


def test_synthesis_stage_is_once_normalized_and_no_clobber(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = _sandbox(tmp_path, monkeypatch)
    state["run"].mkdir(parents=True)
    tree = _candidate_tree()
    manifest = {"mock_manifest": True}

    def build(**kwargs: Any) -> dict[str, Any]:
        output = Path(kwargs["output_path"])
        output.mkdir()
        _publish(output / runner.composite.BUILD_MANIFEST, {"built": True})
        return manifest

    monkeypatch.setattr(runner.composite, "build_verify_save_composite", build)
    monkeypatch.setattr(runner, "_tree", lambda _path: copy.deepcopy(tree))
    monkeypatch.setattr(
        runner,
        "_offline_evaluation",
        lambda _path: _valid_offline(),
    )
    monkeypatch.setattr(
        runner,
        "_functional_verification",
        lambda value: _valid_verification() if value == manifest else {},
    )
    activity = runner._zero_activity()
    receipt = runner._run_synthesis(activity)

    assert receipt["passed"] is True
    assert receipt["status"] == runner.contract.SYNTHESIS_PASS_STATUS
    assert receipt["candidate_id"] == runner.contract.candidate_id(TREE_SHA256)
    assert activity["actor_synthesis_stage_calls_attempted"] == 1
    assert activity["actor_synthesis_executions_confirmed"] == 1
    summary = runner._mapping(state["synthesis_summary"])
    gate = runner._mapping(state["synthesis_gate"])
    assert gate["passed"] is True
    assert summary["actor_updates"] == 0
    assert type(summary["actor_updates"]) is int
    assert summary["environment_reset_calls"] == 0
    assert summary["offline_evaluation"] == _valid_offline()

    before = copy.deepcopy(activity)
    with pytest.raises(runner.V12R6ExecutionError, match="already exists"):
        runner._run_synthesis(activity)
    assert activity == before


def test_development_delegates_to_mocked_physical_surface_only(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = _sandbox(tmp_path, monkeypatch)
    state["run"].mkdir(parents=True)
    _publish(state["claim"], {"claimed": True})
    state["candidate_module"].mkdir(parents=True)
    (state["candidate_module"] / "module_state.pkl").write_bytes(b"mock")
    tree = _candidate_tree()
    identity = runner.contract.candidate_id(TREE_SHA256)
    _publish(
        state["candidate_freeze"],
        {
            "status": runner.contract.CANDIDATE_FREEZE_PASS_STATUS,
            "passed": True,
            "candidate_id": identity,
            "candidate_module": tree,
        },
    )
    monkeypatch.setattr(runner, "_tree", lambda _path: copy.deepcopy(tree))
    monkeypatch.setattr(runner, "_assert_qualification_closed", lambda: {})
    audit = {"passed": True, "row_count": 500}
    captured: dict[str, Any] = {}

    def fake_run_case(**kwargs: Any) -> dict[str, Any]:
        captured.update(kwargs)
        kwargs["activity_callback"]("environment_reset_calls", 1)
        kwargs["activity_callback"]("environment_step_calls", 500)
        kwargs["activity_callback"]("raw_sensor_sample_count", 5000)
        destination = Path(kwargs["destination"])
        destination.mkdir(parents=True)
        trace = [{"step": step} for step in range(1, 501)]
        summary = {"pure_policy_trace_audit": copy.deepcopy(audit)}
        _publish(destination / "trace.json", trace)
        _publish(destination / "summary.json", summary)
        return {"trace": trace, "summary": summary}

    monkeypatch.setattr(runner.physical, "run_case", fake_run_case)
    monkeypatch.setattr(
        runner.contract,
        "pure_policy_trace_audit",
        lambda trace, *, case_id: copy.deepcopy(audit)
        if len(trace) == 500 and case_id == runner.contract.DEVELOPMENT_CASE_IDS[0]
        else {"passed": False},
    )
    monkeypatch.setattr(
        runner.contract,
        "development_gate",
        lambda summary, *, case_id, trace: {
            "status": runner.contract.DEVELOPMENT_PASS_STATUS,
            "passed": summary.get("pure_policy_trace_audit") == audit
            and len(trace) == 500
            and case_id == runner.contract.DEVELOPMENT_CASE_IDS[0],
        },
    )
    activity = runner._zero_activity()
    case_id = runner.contract.DEVELOPMENT_CASE_IDS[0]
    receipt = runner._run_development(case_id, activity)

    assert receipt["passed"] is True
    assert receipt["candidate_id"] == identity
    assert activity["environment_reset_calls"] == 1
    assert activity["environment_step_calls"] == 500
    assert activity["raw_sensor_sample_count"] == 5000
    assert captured["module_path"] == state["candidate_module"]
    assert captured["config"].artifact_root == state["root"]
    assert captured["start_metadata"]["candidate_freeze"] == runner._record(
        state["candidate_freeze"]
    )
    assert captured["summary_metadata"]["target_contract_id"] == (
        runner.contract.TARGET_CONTRACT_ID
    )
    destination = Path(captured["destination"])
    assert (destination / "gate.json").is_file()
    assert (destination / "receipt.json").is_file()
    assert not any(
        name.startswith("teacher")
        for name in captured
        if name not in {"start_metadata", "summary_metadata"}
    )


def test_terminal_pass_and_fail_ledgers_are_type_strict(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    pass_state = _sandbox(tmp_path / "pass", monkeypatch)
    pass_state["run"].mkdir(parents=True)
    _publish(pass_state["claim"], {"claimed": True})
    completed: list[dict[str, Any]] = []
    for stage_id in runner.contract.STAGE_IDS:
        path = _stage_receipt_path(stage_id)
        _publish(path, {"stage_id": stage_id, "passed": True})
        completed.append({"stage_id": stage_id, "receipt": runner._record(path)})
    payload = runner._terminal_payload(
        passed=True,
        completed=completed,
        attempted_stage=None,
        activity=runner._zero_activity(),
        error=None,
    )
    _publish(pass_state["ledger"], payload)
    assert runner.verify_terminal_ledger() == payload
    with pytest.raises(runner.forensic.ForensicRolloutError, match="clobber"):
        runner._write(pass_state["ledger"], payload)

    fail_root = tmp_path / "fail"
    fail_state = _sandbox(fail_root, monkeypatch)
    fail_state["run"].mkdir(parents=True)
    _publish(fail_state["claim"], {"claimed": True})
    failed_completed: list[dict[str, Any]] = []
    for stage_id in runner.contract.STAGE_IDS[:2]:
        path = _stage_receipt_path(stage_id)
        _publish(path, {"stage_id": stage_id, "passed": True})
        failed_completed.append({"stage_id": stage_id, "receipt": runner._record(path)})
    failed = runner._terminal_payload(
        passed=False,
        completed=failed_completed,
        attempted_stage=runner.contract.STAGE_IDS[2],
        activity=runner._zero_activity(),
        error=RuntimeError("mock terminal failure"),
    )
    _publish(fail_state["ledger"], failed)
    assert runner.verify_terminal_ledger() == failed

    bool_path = fail_state["run"] / "pipeline_ledger_bool.json"
    malformed = copy.deepcopy(failed)
    malformed["passed"] = 1
    _publish(bool_path, malformed)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", bool_path)
    with pytest.raises(runner.V12R6ExecutionError, match="identity"):
        runner.verify_terminal_ledger()

    count_path = fail_state["run"] / "pipeline_ledger_count.json"
    malformed = copy.deepcopy(failed)
    malformed["completed_stages"] = malformed["completed_stages"][:1]
    malformed["completed_stage_count"] = True
    _publish(count_path, malformed)
    monkeypatch.setattr(runner, "PIPELINE_LEDGER_PATH", count_path)
    with pytest.raises(runner.V12R6ExecutionError, match="prefix"):
        runner.verify_terminal_ledger()


def test_execute_uses_exact_stage_order_and_never_resumes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ordered_state = _sandbox(tmp_path / "ordered", monkeypatch)
    ordered_state["run"].parent.mkdir(parents=True, exist_ok=True)
    monkeypatch.setattr(runner.freezer, "verify_protocol_freeze", lambda: {})
    monkeypatch.setattr(
        runner.freezer,
        "verify_execution_lock",
        lambda **_kwargs: {},
    )
    observed: list[str] = []

    def attest() -> dict[str, Any]:
        stage_id = runner.contract.STAGE_IDS[0]
        observed.append(stage_id)
        return {"stage_id": stage_id, "passed": True}

    def synthesize(_activity: dict[str, int]) -> dict[str, Any]:
        stage_id = runner.contract.STAGE_IDS[1]
        observed.append(stage_id)
        payload = {"stage_id": stage_id, "passed": True}
        runner._write(runner.SYNTHESIS_RECEIPT_PATH, payload)
        return payload

    def freeze() -> dict[str, Any]:
        stage_id = runner.contract.STAGE_IDS[2]
        observed.append(stage_id)
        payload = {"stage_id": stage_id, "passed": True}
        runner._write(runner.CANDIDATE_FREEZE_PATH, payload)
        return payload

    def development(case_id: str, _activity: dict[str, int]) -> dict[str, Any]:
        stage_id = f"development__{case_id}"
        observed.append(stage_id)
        path = runner.resolve_relative(runner.contract.DEVELOPMENT_PATHS[case_id]) / (
            "receipt.json"
        )
        payload = {"stage_id": stage_id, "passed": True}
        runner._write(path, payload)
        return payload

    def finalize(_activity: Mapping[str, int]) -> dict[str, Any]:
        stage_id = runner.contract.STAGE_IDS[-1]
        observed.append(stage_id)
        payload = {"stage_id": stage_id, "passed": True}
        runner._write(runner.FINAL_DEVELOPMENT_RECEIPT_PATH, payload)
        return payload

    monkeypatch.setattr(runner, "_attest_terminal_inputs", attest)
    monkeypatch.setattr(runner, "_run_synthesis", synthesize)
    monkeypatch.setattr(runner, "_run_candidate_freeze", freeze)
    monkeypatch.setattr(runner, "_run_development", development)
    monkeypatch.setattr(runner, "_run_finalize_development", finalize)
    result = runner.execute()

    assert result["passed"] is True
    assert observed == list(runner.contract.STAGE_IDS)
    assert result["completed_stage_count"] == len(runner.contract.STAGE_IDS)
    observed.clear()
    assert runner.execute() == result
    assert observed == []

    occupied_state = _sandbox(tmp_path / "occupied", monkeypatch)
    occupied_state["run"].mkdir(parents=True)
    with pytest.raises(runner.V12R6ExecutionError, match="resume is forbidden"):
        runner.execute()
    assert not occupied_state["ledger"].exists()


@pytest.mark.parametrize(
    "value",
    ("", "../escape", "a/../b", "/absolute", "a//b", "a\\..\\escape"),
)
def test_resolver_rejects_noncanonical_cross_platform_paths(value: str) -> None:
    with pytest.raises(runner.V12R6ExecutionError, match="non-canonical"):
        runner.resolve_relative(value)


def test_activity_updates_reject_bool_and_unknown_fields() -> None:
    activity = runner._zero_activity()
    with pytest.raises(runner.V12R6ExecutionError, match="invalid activity"):
        runner._increment(activity, "environment_step_calls", True)
    with pytest.raises(runner.V12R6ExecutionError, match="invalid activity"):
        runner._increment(activity, "not_a_counter", 1)


def test_record_and_tree_normalize_contained_absolute_paths_only(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = _sandbox(tmp_path, monkeypatch)
    artifact = state["root"] / "artifacts/evidence.json"
    _publish(artifact, {"evidence": True})
    relative_artifact = artifact.relative_to(state["root"]).as_posix()
    assert runner._record(artifact) == runner._record(relative_artifact)

    tree = state["root"] / "artifacts/module"
    tree.mkdir()
    (tree / "module_state.pkl").write_bytes(b"state")
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", state["root"])
    relative_tree = PurePosixPath(tree.relative_to(state["root"]).as_posix())
    assert runner._tree(tree) == runner._tree(relative_tree)

    escaped = state["root"].parent / "outside-v12r6-artifact"
    with pytest.raises(runner.V12R6ExecutionError, match="escaped"):
        runner._record(escaped)
    with pytest.raises(runner.V12R6ExecutionError, match="escaped"):
        runner._tree(escaped)
