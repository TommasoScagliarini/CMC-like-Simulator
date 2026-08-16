from __future__ import annotations

import ast
import copy
import hashlib
import math
from pathlib import Path
from typing import Any, Callable

import numpy as np
import pytest

import h0_v12r5_case_balanced_contract as contract
import h0_v12r5_case_balanced_fitter as fitter


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(fitter.forensic.canonical_json_bytes(payload))


def _synthetic_fit_stage(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> dict[str, Any]:
    """Publish only small verifier fixtures below ``tmp_path``."""

    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    for name, path in contract.Q3_UNOPENED_PATHS.items():
        destination = fitter._resolve(path)
        if name.endswith("root"):
            destination.mkdir(parents=True, exist_ok=True)
        else:
            _write_json(destination, {"synthetic": True})

    root = fitter._resolve(contract.FIT_ROOT)
    root.mkdir(parents=True, exist_ok=True)
    module_root = fitter._resolve(contract.CANDIDATE_MODULE_PATH)
    module_root.mkdir(parents=True, exist_ok=True)
    module_bytes = b"x"
    module_file = module_root / "weights.bin"
    module_file.write_bytes(module_bytes)
    file_sha256 = hashlib.sha256(module_bytes).hexdigest()
    tree_digest = hashlib.sha256()
    tree_digest.update(b"weights.bin\0")
    tree_digest.update(file_sha256.encode("ascii"))
    tree_digest.update(b"\0")
    tree_digest.update(str(len(module_bytes)).encode("ascii"))
    tree_digest.update(b"\n")
    module = {
        "path": fitter._logical_path(contract.CANDIDATE_MODULE_PATH),
        "tree_sha256": tree_digest.hexdigest(),
        "file_count": 1,
        "files": [
            {
                "path": "weights.bin",
                "sha256": file_sha256,
                "size_bytes": len(module_bytes),
            }
        ],
    }

    def synthetic_tree_record(path: Any) -> dict[str, Any]:
        assert fitter._resolve(path) == module_root
        return copy.deepcopy(module)

    monkeypatch.setattr(fitter, "tree_record", synthetic_tree_record)
    worker_claim_path = fitter._resolve(contract.WORKER_CLAIMS_ROOT) / (
        "fit_case_balanced_candidate.json"
    )
    source_paths = (
        fitter._resolve(contract.PIPELINE_CLAIM_PATH),
        worker_claim_path,
        fitter._resolve(contract.PROTOCOL_FREEZE_PATH),
        fitter._resolve(contract.EXECUTION_LOCK_PATH),
    )
    for index, path in enumerate(source_paths):
        _write_json(path, {"synthetic_source": index})
    (root / "corpus.npz").write_bytes(b"x")
    _write_json(root / "adaptation_report.json", {"synthetic": "report"})
    _write_json(root / "adaptation_history.json", [])

    metric = {"rmse": 0.0, "max_abs_error": 0.0, "reset_max_abs_error": 0.0}
    critical_metric = {"rmse": 0.0, "max_abs_error": 0.0}
    candidate_id = contract.candidate_id(module["tree_sha256"])
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit": copy.deepcopy(contract.FIT),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_p2": False,
        "fit_counts": contract.expected_corpus_counts(),
        "sample_count": contract.CORPUS_ROWS,
        "episode_count": contract.CORPUS_EPISODE_COUNT,
        "reset_row_count": contract.CORPUS_RESET_ROWS,
        "corpus_components": ["p2_corpus", "r4_nominal_pass_labels_only"],
        "failed_plus_prefix_rows_loaded": 0,
        "weighting": copy.deepcopy(contract.WEIGHTING),
        "case_mass": {case_id: 1000.0 for case_id in contract.CASE_IDS},
        "weight_hashes": {
            "p2_max_abs_error_sha256": contract.WEIGHTING[
                "expected_p2_max_abs_error_sha256"
            ],
            "hardness_sha256": contract.WEIGHTING["expected_hardness_sha256"],
            "source_risk_sha256": contract.WEIGHTING["expected_source_risk_sha256"],
            "normalized_weights_sha256": contract.WEIGHTING[
                "expected_normalized_weights_sha256"
            ],
        },
        "normalized_total_sample_mass": float(contract.NORMALIZED_TOTAL_MASS),
        "metrics": copy.deepcopy(metric),
        "p2_subset_metrics": copy.deepcopy(metric),
        "nominal_r4_pass_metrics": copy.deepcopy(metric),
        "nominal_r4_student_exposed_metrics": copy.deepcopy(metric),
        "per_case_metrics": {
            case_id: copy.deepcopy(metric) for case_id in contract.CASE_IDS
        },
        "critical_window": copy.deepcopy(contract.CRITICAL_WINDOW),
        "critical_window_metrics": copy.deepcopy(critical_metric),
        "critical_window_p2_baseline_metrics": copy.deepcopy(critical_metric),
        "critical_window_p2_baseline_recomputed": True,
        "critical_window_p2_module_tree": copy.deepcopy(contract.P2_MODULE_TREE),
        "worst_row": {
            "absolute_error": 0.0,
            "action_dimension": 0,
            "case_id": contract.CASE_IDS[0],
            "step_index": 1,
            "tranche_id": "synthetic",
        },
        "adamw_epochs_run": 3000,
        "lbfgs_max_iter": 600,
        "lbfgs_max_eval": 1200,
        "lbfgs_closure_calls": 1,
        "deterministic_algorithms_enabled": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": True,
        "critic_present": False,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "hard_polish_used": False,
        "fallback_used": False,
        "sweep_used": False,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "candidate_module": copy.deepcopy(module),
        "candidate_id": candidate_id,
        "corpus": fitter._record(root / "corpus.npz"),
        "adaptation_report": fitter._record(root / "adaptation_report.json"),
        "adaptation_history": fitter._record(root / "adaptation_history.json"),
        "pipeline_claim": fitter._record(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": fitter._record(worker_claim_path),
        "protocol_freeze": fitter._record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": fitter._record(contract.EXECUTION_LOCK_PATH),
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    state = {
        "root": root,
        "module": module,
        "summary": summary,
    }
    _republish_bound_fit(state)
    return state


def _republish_bound_fit(
    state: dict[str, Any],
    *,
    summary_mutation: Callable[[dict[str, Any]], None] | None = None,
    gate_mutation: Callable[[dict[str, Any]], None] | None = None,
    receipt_mutation: Callable[[dict[str, Any]], None] | None = None,
) -> None:
    root = state["root"]
    summary = copy.deepcopy(state["summary"])
    if summary_mutation is not None:
        summary_mutation(summary)
    _write_json(root / "summary.json", summary)
    gate = contract.fit_gate(summary)
    if gate_mutation is not None:
        gate_mutation(gate)
    _write_json(root / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": summary["candidate_id"],
        "candidate_module": copy.deepcopy(summary["candidate_module"]),
        "summary": fitter._record(root / "summary.json"),
        "gate": fitter._record(root / "gate.json"),
        "corpus": copy.deepcopy(summary["corpus"]),
        "adaptation_report": copy.deepcopy(summary["adaptation_report"]),
        "adaptation_history": copy.deepcopy(summary["adaptation_history"]),
        "pipeline_claim": copy.deepcopy(summary["pipeline_claim"]),
        "worker_claim": copy.deepcopy(summary["worker_claim"]),
        "actor_updates": summary["actor_updates"],
        "critic_updates": summary["critic_updates"],
        "ppo_updates": summary["ppo_updates"],
    }
    if receipt_mutation is not None:
        receipt_mutation(receipt)
    _write_json(root / "receipt.json", receipt)
    state.update(summary=summary, gate=gate, receipt=receipt)


def test_adamw_schedule_boundaries() -> None:
    assert fitter.adamw_learning_rate(1) == 3.0e-4
    assert fitter.adamw_learning_rate(1500) == 3.0e-4
    assert fitter.adamw_learning_rate(1501) == 1.0e-4
    assert fitter.adamw_learning_rate(2500) == 1.0e-4
    assert fitter.adamw_learning_rate(2501) == 3.0e-5
    assert fitter.adamw_learning_rate(3000) == 3.0e-5
    for value in (0, 3001, True):
        with pytest.raises(fitter.V12R5CaseBalancedFitError):
            fitter.adamw_learning_rate(value)


def test_locked_input_attestation_includes_forensic_exclusion_and_runtime_closure() -> (
    None
):
    attestation = fitter._attest_locked_inputs()
    assert all(attestation["semantics"].values())
    assert attestation["r4_nominal_pass"] == contract.R4_NOMINAL_REUSABLE_ARTIFACTS
    assert attestation["r4_plus_failure_forensic_only"] == (
        contract.R4_PLUS_FAILURE_EVIDENCE
    )
    assert attestation["external_runtime_sources"] == (
        contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
    )
    assert len(attestation["external_runtime_sources"]) == 64
    assert len(attestation["production_source_closure"]) == 68


def test_p2_and_nominal_pieces_are_exact_and_nominal_mask_has_255_rows() -> None:
    p2 = fitter._load_p2_piece()
    nominal, exposed = fitter._load_nominal_piece()
    assert p2["observations"].shape == (contract.P2_CORPUS_ROWS, 35)
    assert p2["actions"].shape == (contract.P2_CORPUS_ROWS, 2)
    assert int(np.count_nonzero(p2["reset_mask"])) == 18
    assert nominal["observations"].shape == (contract.NOMINAL_PASS_ROWS, 35)
    assert nominal["actions"].shape == (contract.NOMINAL_PASS_ROWS, 2)
    assert int(np.count_nonzero(nominal["reset_mask"])) == 1
    assert int(np.count_nonzero(exposed)) == 255
    assert bool(exposed[0]) is True
    assert int(np.count_nonzero(nominal["reset_mask"][exposed])) == 1


def test_fixed_hardness_then_case_balance_formula() -> None:
    case_ids = np.asarray(contract.CASE_IDS, dtype="U64")
    source_risk = np.asarray([1.0, 100.0, 2.0, 1.0, 1.0, 1.0])
    targets = np.zeros((6, 2), dtype=np.float32)
    predictions = np.asarray(
        [[0.0, 0.0], [0.03, 0.0], [0.06, 0.0], [0.12, 0.0], [0.0, 0.0], [0.0, 0.0]],
        dtype=np.float32,
    )
    weights, audit = fitter.compute_case_balanced_weights(
        source_risk=source_risk,
        p2_predictions=predictions,
        targets=targets,
        case_ids=case_ids,
    )
    assert np.array_equal(weights, np.repeat(np.float64(1000.0), 6))
    assert all(
        math.isclose(value, 1000.0, rel_tol=0.0, abs_tol=1e-12)
        for value in audit["case_mass"].values()
    )
    assert audit["normalized_total_sample_mass"] == 6000.0
    assert audit["weighting"] == contract.WEIGHTING


def test_fitter_has_no_environment_or_new_collection_surface() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    keywords = {
        keyword.arg: ast.literal_eval(keyword.value)
        for call in calls
        for keyword in call.keywords
        if keyword.arg in {"max_iter", "max_eval"}
        and isinstance(keyword.value, ast.Constant)
    }
    assert keywords == {"max_iter": 600, "max_eval": 1200}
    assert "make_cmc_env" not in source
    assert ".reset(" not in source
    assert "env.step(" not in source
    assert "_load_collection_piece" not in source
    assert "COLLECTION_CASE_IDS" not in source
    assert 'failed_plus_prefix_rows_loaded": 0' in source
    assert 'hard_polish_used": True' not in source
    assert 'fallback_used": True' not in source


def test_failed_plus_label_npz_is_never_passed_to_npz_reader() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    read_calls = [
        ast.get_source_segment(source, node) or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_read_npz"
    ]
    assert len(read_calls) == 2
    assert any("P2_CORPUS_ARTIFACT" in call for call in read_calls)
    assert any("R4_NOMINAL_REUSABLE_ARTIFACTS" in call for call in read_calls)
    assert all("R4_PLUS_FAILURE_EVIDENCE" not in call for call in read_calls)


def test_historical_fit_verifier_is_invariant_after_every_q3_path_opens(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    state = _synthetic_fit_stage(monkeypatch, tmp_path)
    assert all(
        fitter.os.path.lexists(fitter._resolve(path))
        for path in contract.Q3_UNOPENED_PATHS.values()
    )
    assert fitter._qualification_unopened() is False
    assert fitter.verify_fit_stage() == state["receipt"]


@pytest.mark.parametrize("drift", ["missing", "extra"])
def test_fit_receipt_schema_is_exact(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, drift: str
) -> None:
    state = _synthetic_fit_stage(monkeypatch, tmp_path)
    receipt = copy.deepcopy(state["receipt"])
    if drift == "missing":
        receipt.pop("ppo_updates")
    else:
        receipt["unexpected"] = False
    _write_json(state["root"] / "receipt.json", receipt)
    with pytest.raises(
        fitter.V12R5CaseBalancedFitError,
        match="receipt/binding drifted",
    ):
        fitter.verify_fit_stage()


@pytest.mark.parametrize(
    "drift",
    [
        "receipt_bool_counter",
        "summary_bool_counter",
        "gate_bool_check",
        "tree_bool_size",
        "artifact_bool_size",
        "artifact_extra_field",
        "summary_extra_field",
    ],
)
def test_fit_verifier_rejects_deep_schema_and_python_numeric_coercions(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, drift: str
) -> None:
    state = _synthetic_fit_stage(monkeypatch, tmp_path)

    def mutate_summary(summary: dict[str, Any]) -> None:
        if drift == "summary_bool_counter":
            summary["actor_updates"] = True
        elif drift == "tree_bool_size":
            summary["candidate_module"]["files"][0]["size_bytes"] = True
        elif drift == "artifact_bool_size":
            summary["corpus"]["size_bytes"] = True
        elif drift == "artifact_extra_field":
            summary["corpus"]["unexpected"] = "not-canonical"
        elif drift == "summary_extra_field":
            summary["unexpected"] = False

    def mutate_gate(gate: dict[str, Any]) -> None:
        if drift == "gate_bool_check":
            gate["checks"]["one_fit"] = 1

    def mutate_receipt(receipt: dict[str, Any]) -> None:
        if drift == "receipt_bool_counter":
            receipt["critic_updates"] = False

    _republish_bound_fit(
        state,
        summary_mutation=mutate_summary,
        gate_mutation=mutate_gate,
        receipt_mutation=mutate_receipt,
    )
    assert state["gate"]["passed"] is True
    with pytest.raises(
        fitter.V12R5CaseBalancedFitError,
        match="receipt/binding drifted",
    ):
        fitter.verify_fit_stage()


def test_fit_verifier_recomputes_every_receipt_artifact_record(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    state = _synthetic_fit_stage(monkeypatch, tmp_path)
    (state["root"] / "adaptation_report.json").write_bytes(b"changed")
    with pytest.raises(
        fitter.V12R5CaseBalancedFitError,
        match="receipt/binding drifted",
    ):
        fitter.verify_fit_stage()
