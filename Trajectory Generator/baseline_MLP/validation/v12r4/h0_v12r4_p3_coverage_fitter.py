"""Sole fresh-H0 weighted fitter for the V12R4 P3 candidate.

The fitter consumes the byte-bound V12R3 P2 corpus and exactly four new
shielded same-state teacher-label episodes.  It never opens an environment and
never continues from the P2 module: P2 is collection-only.  The one numerical
design is full-batch AdamW(3000) followed by deterministic LBFGS(600/1200).
"""

from __future__ import annotations

import copy
import math
import os
import sys
from pathlib import Path, PurePath
from typing import Any, Callable, Mapping

import numpy as np


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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import h0_primary_split_v11_weighted_fit as v11  # noqa: E402
import h0_primary_split_v12r3_recovery_weighted_fitter as v12r3_fit  # noqa: E402
import h0_v12r4_p3_coverage_contract as contract  # noqa: E402
from freeze_h0_v12r4_p3_coverage import tree_record  # noqa: E402


class V12R4CoverageFitError(RuntimeError):
    """Raised when the fixed R4 corpus or fit contract drifts."""


RecoveryFitCorpus = v12r3_fit.RecoveryFitCorpus
FrozenNormalization = v11.FrozenNormalization
InMemoryFitResult = v11.InMemoryFitResult
CLOCK_COLUMNS = (0, 1)
DETERMINISTIC_TORCH_THREADS = v11.DETERMINISTIC_TORCH_THREADS

_CORPUS_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
    "episode_ids",
    "raw_sample_weights",
    "normalized_sample_weights",
    "training_indices",
}
_LABEL_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
}


def _resolve(path: str | PurePath | Path) -> Path:
    candidate = Path(path)
    return (candidate if candidate.is_absolute() else REPO_ROOT / candidate).resolve()


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    try:
        value = forensic.strict_json_load(resolved)
    except Exception as exc:
        raise V12R4CoverageFitError(f"invalid strict JSON object: {resolved}") from exc
    if not isinstance(value, Mapping):
        raise V12R4CoverageFitError(f"expected JSON object: {resolved}")
    return dict(value)


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)


def _artifact_matches(value: Any, path: str | PurePath | Path) -> bool:
    return isinstance(value, Mapping) and dict(value) == _record(path)


def array_sha256(value: Any) -> str:
    return v10s_fit.array_sha256(value)


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _read_npz(path: str | PurePath | Path, keys: set[str]) -> dict[str, np.ndarray]:
    resolved = _resolve(path)
    try:
        with np.load(resolved, allow_pickle=False) as archive:
            if set(archive.files) != keys:
                raise V12R4CoverageFitError(
                    f"NPZ keys drifted for {resolved}: {sorted(archive.files)}"
                )
            return {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R4CoverageFitError:
        raise
    except Exception as exc:
        raise V12R4CoverageFitError(f"cannot read exact NPZ: {resolved}") from exc


def _q2_unopened() -> bool:
    return all(
        not os.path.lexists(_resolve(path))
        for path in contract.Q2_UNOPENED_PATHS.values()
    )


def _load_p2_piece() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if _record(contract.P2_CORPUS_ARTIFACT["path"]) != contract.P2_CORPUS_ARTIFACT:
        raise V12R4CoverageFitError("P2 corpus hash/size drifted")
    arrays = _read_npz(contract.P2_CORPUS_ARTIFACT["path"], _CORPUS_KEYS)
    rows = contract.P2_CORPUS_ROWS
    if (
        arrays["observations"].shape != (rows, contract.EXPECTED_ACTOR_FEATURES)
        or arrays["observations"].dtype != np.dtype(np.float32)
        or arrays["actions"].shape != (rows, contract.EXPECTED_ACTION_DIM)
        or arrays["actions"].dtype != np.dtype(np.float32)
        or arrays["reset_mask"].shape != (rows,)
        or arrays["reset_mask"].dtype != np.dtype(np.bool_)
        or int(np.count_nonzero(arrays["reset_mask"])) != contract.P2_RESET_ROWS
        or len(set(arrays["episode_ids"].astype(str).tolist()))
        != contract.P2_EPISODE_COUNT
        or not np.array_equal(arrays["training_indices"], np.arange(rows))
        or not np.all(np.isfinite(arrays["observations"]))
        or not np.all(np.isfinite(arrays["actions"]))
        or not np.all(np.isfinite(arrays["normalized_sample_weights"]))
        or math.fsum(float(value) for value in arrays["normalized_sample_weights"])
        != contract.P2_EPISODE_COUNT * contract.EPISODE_TARGET_MASS
    ):
        raise V12R4CoverageFitError("P2 corpus semantic contract drifted")
    return arrays, {
        "role": "IMMUTABLE_P2_CORPUS_NOT_P2_CHECKPOINT_CONTINUATION",
        "artifact": dict(contract.P2_CORPUS_ARTIFACT),
        "sample_count": rows,
        "episode_count": contract.P2_EPISODE_COUNT,
    }


def _load_collection_piece(
    case_id: str, pipeline_claim: Mapping[str, Any]
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    root = _resolve(contract.COLLECTION_PATHS[case_id])
    receipt_path = root / "receipt.json"
    receipt = _mapping(receipt_path)
    summary = _mapping(root / "summary.json")
    gate = _mapping(root / "gate.json")
    labels_path = root / "labels.npz"
    label_record = _record(labels_path)
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.COLLECTION_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("case_id") != case_id
        or receipt.get("sample_count") != contract.EXPECTED_STEPS
        or receipt.get("label_corpus") != label_record
        or receipt.get("pipeline_claim") != pipeline_claim
        or gate != contract.collection_gate(summary, case_id=case_id)
        or gate.get("passed") is not True
        or summary.get("candidate_module") != contract.P2_MODULE_TREE
        or summary.get("candidate_role") != "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE"
    ):
        raise V12R4CoverageFitError(f"collection receipt/gate drifted: {case_id}")
    arrays = _read_npz(labels_path, _LABEL_KEYS)
    rows = contract.EXPECTED_STEPS
    reset = arrays["reset_mask"]
    if (
        arrays["observations"].shape != (rows, contract.EXPECTED_ACTOR_FEATURES)
        or arrays["observations"].dtype != np.dtype(np.float32)
        or arrays["actions"].shape != (rows, contract.EXPECTED_ACTION_DIM)
        or arrays["actions"].dtype != np.dtype(np.float32)
        or reset.shape != (rows,)
        or reset.dtype != np.dtype(np.bool_)
        or int(np.count_nonzero(reset)) != 1
        or not bool(reset[0])
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or set(arrays["case_ids"].astype(str).tolist()) != {case_id}
        or set(arrays["tranche_ids"].astype(str).tolist()) != {"v12r4_p3_coverage"}
        or set(arrays["origins"].astype(str).tolist())
        != {"V12R4_SHIELDED_SAME_STATE_TEACHER_LABEL"}
        or not np.all(np.isfinite(arrays["observations"]))
        or not np.all(np.isfinite(arrays["actions"]))
        or np.any(np.abs(arrays["actions"]) > 1.0)
    ):
        raise V12R4CoverageFitError(f"collection arrays drifted: {case_id}")
    raw, normalized = v12r3_fit._shielded_episode_weights(reset)
    episode_ids = np.repeat(
        np.asarray([f"v12r4_coverage:p3:{case_id}"], dtype="U192"), rows
    )
    piece = {
        **arrays,
        "episode_ids": episode_ids,
        "raw_sample_weights": raw,
        "normalized_sample_weights": normalized,
    }
    return piece, {
        "case_id": case_id,
        "receipt": _record(receipt_path),
        "label_corpus": label_record,
        "sample_count": rows,
        "passed": True,
    }


def load_p3_corpus(pipeline_claim: Mapping[str, Any]) -> RecoveryFitCorpus:
    """Load the exact 10,732-row P2+coverage corpus in frozen order."""

    p2, p2_binding = _load_p2_piece()
    pieces: list[dict[str, np.ndarray]] = [p2]
    bindings: list[dict[str, Any]] = []
    for case_id in contract.COLLECTION_CASE_IDS:
        piece, binding = _load_collection_piece(case_id, pipeline_claim)
        pieces.append(piece)
        bindings.append(binding)
    row_names = (
        "observations",
        "actions",
        "reset_mask",
        "case_ids",
        "step_indices",
        "tranche_ids",
        "origins",
        "episode_ids",
        "raw_sample_weights",
        "normalized_sample_weights",
    )
    combined = {
        name: np.ascontiguousarray(np.concatenate([piece[name] for piece in pieces]))
        for name in row_names
    }
    features = np.ascontiguousarray(p2["actor_feature_names"], dtype="U64")
    if any(
        not _bytes_equal(piece["actor_feature_names"], features) for piece in pieces[1:]
    ):
        raise V12R4CoverageFitError("collection actor feature names drifted")
    identities = list(
        zip(
            combined["tranche_ids"].astype(str).tolist(),
            combined["case_ids"].astype(str).tolist(),
            combined["step_indices"].tolist(),
        )
    )
    counts = contract.expected_corpus_counts()
    episode_count = len(set(combined["episode_ids"].astype(str).tolist()))
    if (
        len(combined["observations"]) != contract.P3_CORPUS_ROWS
        or int(np.count_nonzero(combined["reset_mask"])) != contract.P3_RESET_ROWS
        or episode_count != contract.P3_EPISODE_COUNT
        or len(identities) != len(set(identities))
        or math.fsum(float(value) for value in combined["normalized_sample_weights"])
        != contract.P3_NORMALIZED_TOTAL_MASS
        or not all(
            np.all(np.isfinite(combined[name]))
            for name in ("observations", "actions", "normalized_sample_weights")
        )
    ):
        raise V12R4CoverageFitError("assembled P3 corpus contract drifted")
    audit = {
        **counts,
        "all_finite": True,
        "duplicate_sample_count": 0,
        "component_order_exact": True,
        "q2_unopened": _q2_unopened(),
    }
    if not audit["q2_unopened"]:
        raise V12R4CoverageFitError("Q2 execution output opened during R4")
    return RecoveryFitCorpus(
        observations=combined["observations"],
        actions=combined["actions"],
        reset_mask=combined["reset_mask"],
        actor_feature_names=features,
        case_ids=combined["case_ids"],
        step_indices=combined["step_indices"],
        tranche_ids=combined["tranche_ids"],
        origins=combined["origins"],
        episode_ids=combined["episode_ids"],
        raw_sample_weights=combined["raw_sample_weights"],
        normalized_sample_weights=combined["normalized_sample_weights"],
        source_records={"p2": p2_binding, "collections": bindings},
        probe_label_bindings=(),
        collection_bindings=tuple(bindings),
        audit=audit,
    )


def adamw_learning_rate(epoch: int) -> float:
    if not isinstance(epoch, int) or isinstance(epoch, bool) or not 1 <= epoch <= 3000:
        raise V12R4CoverageFitError(f"AdamW epoch outside frozen schedule: {epoch!r}")
    if epoch <= 1500:
        return 3.0e-4
    if epoch <= 2500:
        return 1.0e-4
    return 3.0e-5


def fit_p3_full_mean_in_memory(
    *,
    source_state: Mapping[str, Any],
    observations: Any,
    targets: Any,
    reset_mask: Any,
    sample_weights: Any,
    normalization: FrozenNormalization,
    activity_callback: Callable[[str, int], None] | None = None,
) -> InMemoryFitResult:
    """Run the sole fixed R4 numerical design, with no fallback or polish."""

    import torch

    try:
        v11.validate_source_h0_state(source_state)
    except Exception as exc:
        raise V12R4CoverageFitError("source H0 state validation failed") from exc
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    weights_np = np.ascontiguousarray(sample_weights, dtype=np.float64)
    if (
        raw.shape != (contract.P3_CORPUS_ROWS, contract.EXPECTED_ACTOR_FEATURES)
        or labels.shape != (contract.P3_CORPUS_ROWS, contract.EXPECTED_ACTION_DIM)
        or reset.shape != (contract.P3_CORPUS_ROWS,)
        or weights_np.shape != (contract.P3_CORPUS_ROWS,)
        or not np.all(np.isfinite(raw))
        or not np.all(np.isfinite(labels))
        or not np.all(np.isfinite(weights_np))
        or np.any(weights_np <= 0.0)
    ):
        raise V12R4CoverageFitError("weighted P3 arrays are malformed")
    normalized = v11.normalized_observations(raw, normalization)
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(DETERMINISTIC_TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(20260807)
        model = v11._new_normalized_model(source_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        weights = torch.as_tensor(weights_np, dtype=torch.float64)
        weight_sum = torch.sum(weights)
        history: list[dict[str, Any]] = []
        optimizer = torch.optim.AdamW(
            model.parameters(), lr=3.0e-4, weight_decay=1.0e-7
        )
        for epoch in range(1, 3001):
            rate = adamw_learning_rate(epoch)
            for group in optimizer.param_groups:
                group["lr"] = rate
            optimizer.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            loss = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(loss):
                raise V12R4CoverageFitError(f"non-finite AdamW loss at {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            optimizer.step()
            if activity_callback is not None:
                activity_callback("adamw_epochs_completed", 1)
            if epoch in {1, 250, 500, 1000, 1500, 2000, 2500, 3000}:
                v11._milestone(history, stage="adamw", index=epoch, loss=loss, lr=rate)

        lbfgs = torch.optim.LBFGS(
            model.parameters(),
            lr=0.7,
            max_iter=600,
            max_eval=1200,
            tolerance_grad=1.0e-10,
            tolerance_change=1.0e-12,
            history_size=50,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        last_lbfgs_loss: Any = None

        def closure() -> Any:
            nonlocal closure_calls, last_lbfgs_loss
            lbfgs.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            value = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(value):
                raise V12R4CoverageFitError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            if activity_callback is not None:
                activity_callback("lbfgs_closure_calls", 1)
            last_lbfgs_loss = value
            if closure_calls in {1, 50, 100, 200, 300, 400, 600, 800, 1200}:
                v11._milestone(
                    history,
                    stage="lbfgs_closure",
                    index=closure_calls,
                    loss=value,
                    lr=0.7,
                )
            return value

        lbfgs.step(closure)
        if last_lbfgs_loss is None:
            raise V12R4CoverageFitError("LBFGS never evaluated the objective")
        v11._milestone(
            history,
            stage="lbfgs_final",
            index=closure_calls,
            loss=last_lbfgs_loss,
            lr=0.7,
        )
        candidate_state, fold_audit = v11._fold_normalization_into_state(
            model, source_state, normalization
        )
        with torch.no_grad():
            normalized_prediction = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        runtime_logits = v11._state_logits(candidate_state, raw)
        runtime_prediction = np.ascontiguousarray(
            runtime_logits[:, :2], dtype=np.float32
        )
        normalization_audit = {
            **fold_audit,
            **v11.fold_equivalence_audit(normalized_prediction, runtime_prediction),
            "normalization": normalization.record(),
        }
        preservation = v11.full_mean_update_audit(source_state, candidate_state)
        source_logits = v11._state_logits(source_state, raw)
        preservation = {
            **preservation,
            "logstd_outputs_bit_exact": source_logits[:, 2:].tobytes()
            == runtime_logits[:, 2:].tobytes(),
        }
        preservation["passed"] = bool(
            preservation["passed"] and preservation["logstd_outputs_bit_exact"]
        )
        if not preservation["passed"]:
            raise V12R4CoverageFitError("mean-only preservation audit failed")
        metrics = v11.prediction_metrics(runtime_prediction, labels, reset)
        optimizer_audit = {
            "fit_contract_id": contract.FIT_CONTRACT_ID,
            "seed": 20260807,
            "full_batch": True,
            "sample_count": len(raw),
            "explicit_sample_weights": True,
            "sample_weight_dtype": "float64",
            "sample_weights_sha256": array_sha256(weights_np),
            "normalized_total_sample_mass": float(
                math.fsum(float(value) for value in weights_np)
            ),
            "adamw_epochs": 3000,
            "adamw_schedule": copy.deepcopy(
                contract.FIT["adamw"]["learning_rate_schedule"]
            ),
            "adamw_weight_decay": 1.0e-7,
            "gradient_clip_norm": 10.0,
            "lbfgs_lr": 0.7,
            "lbfgs_max_iter": 600,
            "lbfgs_max_eval": 1200,
            "lbfgs_tolerance_grad": 1.0e-10,
            "lbfgs_tolerance_change": 1.0e-12,
            "lbfgs_history_size": 50,
            "lbfgs_line_search": "strong_wolfe",
            "lbfgs_closure_calls": closure_calls,
            "hard_polish": False,
            "fallback": False,
            "sweep": False,
            "torch_threads": DETERMINISTIC_TORCH_THREADS,
            "deterministic_algorithms_enabled": True,
        }
        return InMemoryFitResult(
            candidate_state=candidate_state,
            predictions=runtime_prediction,
            metrics=metrics,
            normalization=normalization,
            normalization_audit=normalization_audit,
            preservation_audit=preservation,
            history=tuple(history),
            optimizer_audit=optimizer_audit,
        )
    except V12R4CoverageFitError:
        raise
    except Exception as exc:
        raise V12R4CoverageFitError("fixed R4 weighted fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _load_source_module_and_state() -> tuple[Any, dict[str, Any]]:
    """Load H0 through an absolute checkpoint path (required by pyarrow)."""

    from ray.rllib.core.rl_module.rl_module import RLModule

    source = _resolve(contract.SOURCE_H0_MODULE_PATH)
    module = RLModule.from_checkpoint(source.resolve())
    state = v11._clone_state(module.get_state())
    v11.validate_source_h0_state(state)
    return module, state


def _metric_slice(
    predictions: np.ndarray, corpus: RecoveryFitCorpus, selection: np.ndarray
) -> dict[str, float]:
    return v11.prediction_metrics(
        predictions[selection],
        corpus.actions[selection],
        corpus.reset_mask[selection],
    )


def run_fit_stage(
    *,
    pipeline_claim_path: str | PurePath | Path,
    worker_claim_path: str | PurePath | Path,
    protocol_freeze_path: str | PurePath | Path,
    execution_lock_path: str | PurePath | Path,
    activity_callback: Callable[[str, int], None] | None = None,
) -> dict[str, Any]:
    """Execute and exclusively persist the sole P3 fit."""

    destination = _resolve(contract.FIT_ROOT)
    if os.path.lexists(destination):
        raise V12R4CoverageFitError("P3 fit destination exists/no-clobber")
    pipeline_claim = _record(pipeline_claim_path)
    worker_claim = _record(worker_claim_path)
    protocol_freeze = _record(protocol_freeze_path)
    execution_lock = _record(execution_lock_path)
    if not _q2_unopened():
        raise V12R4CoverageFitError("Q2 execution output already opened")
    corpus = load_p3_corpus(pipeline_claim)
    source_before = tree_record(contract.SOURCE_H0_MODULE_PATH)
    if source_before.get("tree_sha256") != contract.SOURCE_H0_TREE_SHA256:
        raise V12R4CoverageFitError("source H0 tree drifted")
    source_module, source_state = _load_source_module_and_state()
    normalization = v12r3_fit.frozen_base_normalization(corpus.observations[:3000])
    result = fit_p3_full_mean_in_memory(
        source_state=source_state,
        observations=corpus.observations,
        targets=corpus.actions,
        reset_mask=corpus.reset_mask,
        sample_weights=corpus.normalized_sample_weights,
        normalization=normalization,
        activity_callback=activity_callback,
    )
    if source_before != tree_record(contract.SOURCE_H0_MODULE_PATH):
        raise V12R4CoverageFitError("source H0 changed during P3 fit")
    destination.mkdir(parents=True, exist_ok=False)
    v10s_fit._write_npz_exclusive(destination / "corpus.npz", corpus.arrays())
    save_reload = v11._save_candidate_exact(
        source_module=source_module,
        candidate_state=result.candidate_state,
        destination=_resolve(contract.P3_MODULE_PATH),
    )
    module_record = tree_record(contract.P3_MODULE_PATH)
    p2_selection = np.arange(contract.P2_CORPUS_ROWS, dtype=np.int64)
    p2_metrics = _metric_slice(result.predictions, corpus, p2_selection)
    new_metrics: dict[str, dict[str, float]] = {}
    for case_id in contract.COLLECTION_CASE_IDS:
        selected = np.flatnonzero(corpus.case_ids.astype(str) == case_id)
        selected = selected[selected >= contract.P2_CORPUS_ROWS]
        if len(selected) != contract.EXPECTED_STEPS:
            raise V12R4CoverageFitError(f"P3 case slice drifted: {case_id}")
        new_metrics[case_id] = _metric_slice(result.predictions, corpus, selected)
    absolute_error = np.abs(
        result.predictions.astype(np.float64) - corpus.actions.astype(np.float64)
    )
    flat_index = int(np.argmax(absolute_error))
    row_index, action_dimension = np.unravel_index(flat_index, absolute_error.shape)
    worst_row = {
        "absolute_error": float(absolute_error[row_index, action_dimension]),
        "action_dimension": int(action_dimension),
        "case_id": str(corpus.case_ids[row_index]),
        "step_index": int(corpus.step_indices[row_index]),
        "tranche_id": str(corpus.tranche_ids[row_index]),
    }
    history_path = destination / "adaptation_history.json"
    report_path = destination / "adaptation_report.json"
    summary_path = destination / "summary.json"
    gate_path = destination / "gate.json"
    receipt_path = destination / "receipt.json"
    forensic.write_json_exclusive(history_path, list(result.history))
    report = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R4_P3_ADAPTATION_REPORT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "corpus_exact": True,
        "training_samples": contract.P3_CORPUS_ROWS,
        "validation_samples": 0,
        "metrics": dict(result.metrics),
        "p2_subset_metrics": p2_metrics,
        "new_collection_case_metrics": new_metrics,
        "worst_row": worst_row,
        "optimizer_audit": dict(result.optimizer_audit),
        "normalization_audit": dict(result.normalization_audit),
        "preservation_audit": dict(result.preservation_audit),
        "save_reload": save_reload,
        "module_reload_exact": save_reload.get("exact") is True,
        "source_records": copy.deepcopy(corpus.source_records),
        "collection_bindings": [dict(item) for item in corpus.collection_bindings],
    }
    forensic.write_json_exclusive(report_path, report)
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit": copy.deepcopy(contract.FIT),
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_p2": False,
        "fit_counts": contract.expected_corpus_counts(),
        "sample_count": contract.P3_CORPUS_ROWS,
        "episode_count": contract.P3_EPISODE_COUNT,
        "reset_row_count": contract.P3_RESET_ROWS,
        "normalized_total_sample_mass": contract.P3_NORMALIZED_TOTAL_MASS,
        "metrics": dict(result.metrics),
        "p2_subset_metrics": p2_metrics,
        "new_collection_case_metrics": new_metrics,
        "worst_row": worst_row,
        "adamw_epochs_run": 3000,
        "lbfgs_max_iter": 600,
        "lbfgs_max_eval": 1200,
        "lbfgs_closure_calls": result.optimizer_audit["lbfgs_closure_calls"],
        "deterministic_algorithms_enabled": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": result.preservation_audit.get(
            "logstd_parameter_rows_bit_exact"
        )
        is True,
        "critic_present": False,
        "disabled_clock_columns_bit_zero": result.preservation_audit.get(
            "disabled_clock_columns_bit_zero"
        )
        is True,
        "save_reload_exact": save_reload.get("exact") is True,
        "hard_polish_used": False,
        "fallback_used": False,
        "sweep_used": False,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module": module_record,
        "candidate_id": contract.candidate_id(module_record["tree_sha256"]),
        "corpus": _record(destination / "corpus.npz"),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
        "q2_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.fit_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R4CoverageFitError("P3 offline fit gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": summary["candidate_id"],
        "candidate_module": module_record,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "corpus": _record(destination / "corpus.npz"),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(receipt_path, receipt)
    return verify_fit_stage()


def verify_fit_stage() -> dict[str, Any]:
    root = _resolve(contract.FIT_ROOT)
    receipt = _mapping(root / "receipt.json")
    summary = _mapping(root / "summary.json")
    gate = _mapping(root / "gate.json")
    module = tree_record(contract.P3_MODULE_PATH)
    expected_id = contract.candidate_id(module["tree_sha256"])
    if (
        gate != contract.fit_gate(summary)
        or gate.get("passed") is not True
        or receipt.get("status") != contract.FIT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or receipt.get("candidate_id") != expected_id
        or receipt.get("candidate_module") != module
        or summary.get("candidate_id") != expected_id
        or summary.get("candidate_module") != module
        or receipt.get("summary") != _record(root / "summary.json")
        or receipt.get("gate") != _record(root / "gate.json")
    ):
        raise V12R4CoverageFitError("P3 fit receipt/binding drifted")
    return receipt


__all__ = [
    "V12R4CoverageFitError",
    "adamw_learning_rate",
    "fit_p3_full_mean_in_memory",
    "load_p3_corpus",
    "run_fit_stage",
    "verify_fit_stage",
]
