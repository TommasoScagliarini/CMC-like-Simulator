"""Execute the preregistered one-round V6 residual/DAgger pipeline.

Only ``--execute`` may claim a fresh pipeline.  It launches every material
stage in a separate worker process with an ephemeral supervisor token and a
no-clobber, content-addressed worker claim.  A direct worker invocation cannot
create its own authority.  The pipeline is development-only: it performs no
critic or PPO update and never accesses protected or reserve trials.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v6_teacher_replay_contract as teacher_contract  # noqa: E402
import h0_primary_split_v6_residual_dagger_contract as contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_primary_grf_split_v1_adaptation as v1  # noqa: E402
import run_h0_primary_grf_split_v6_teacher_replay as teacher_collector  # noqa: E402
import run_h0_v25_abc_preflight as legacy  # noqa: E402
from primary_grf_split_adaptation import (  # noqa: E402
    array_sha256,
    build_paired_views,
    prediction_metrics,
)
from primary_split_v25_residual import (  # noqa: E402
    PrimarySplitV25ResidualTorchRLModule,
    base_state_keys,
    build_v25_residual_module_from_checkpoint,
)


WORKER_TIMEOUT_S = 3600.0


class V6ResidualDaggerError(RuntimeError):
    """Raised on any provenance, one-shot, fit, rollout, or gate failure."""


def resolve_relative(path: str | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else str(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V6ResidualDaggerError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
PIPELINE_CLAIM = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER = resolve_relative(contract.PIPELINE_LEDGER_PATH)
WORKER_CLAIMS_ROOT = resolve_relative(contract.WORKER_CLAIMS_ROOT)
SOURCE_H0_MODULE = resolve_relative(contract.SOURCE_H0_MODULE_PATH)
P0_MODULE = resolve_relative(contract.P0_MODULE_PATH)
P1_MODULE = resolve_relative(contract.P1_MODULE_PATH)
CANDIDATE_FREEZE = resolve_relative(contract.CANDIDATE_FREEZE_PATH)
DEVELOPMENT_RECEIPT = resolve_relative(contract.DEVELOPMENT_RECEIPT_PATH)


def _mapping(path: str | Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V6ResidualDaggerError(f"expected strict JSON object: {path}")
    return dict(value)


def _sequence(path: str | Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V6ResidualDaggerError(f"expected strict JSON array: {path}")
    return list(value)


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(path, artifact_root=REPO_ROOT)


def _record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and dict(record) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).expanduser().resolve()
    if not root.is_dir():
        raise V6ResidualDaggerError(f"artifact tree is missing: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise V6ResidualDaggerError(f"artifact tree is empty: {root}")
    rows: list[dict[str, Any]] = []
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha = forensic.sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _tree_record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and dict(record) == _tree_record(path)


def _write_npz_exclusive(path: str | Path, **arrays: Any) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise V6ResidualDaggerError(f"refusing to clobber: {destination}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            import numpy as np

            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claim = os.open(str(destination), flags, 0o600)
        except FileExistsError as exc:
            raise V6ResidualDaggerError(f"refusing to clobber: {destination}") from exc
        else:
            os.close(claim)
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return destination


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V6ResidualDaggerError("supervisor token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PIPELINE_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "execution_token_sha256": token_sha256,
        "authority": dict(contract.AUTHORITY),
        "retry_authorized": False,
        "dagger_rounds_authorized": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_pipeline_claim(token: str) -> dict[str, Any]:
    observed = _mapping(PIPELINE_CLAIM)
    expected = _claim_payload(_token_sha256(token))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V6ResidualDaggerError("pipeline claim/token drifted")
    return observed


def _claim_path(stage_id: str) -> Path:
    return resolve_relative(contract.worker_claim_path(stage_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return resolve_relative(contract.stage_receipt_path(stage_id))


def _worker_claim_payload(
    *,
    stage_id: str,
    token_sha256: str,
    previous_receipts: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.WORKER_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "stage_index": contract.STAGE_IDS.index(stage_id),
        "execution_token_sha256": token_sha256,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "previous_receipts": [dict(value) for value in previous_receipts],
        "retry_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_stage_receipt(stage_id: str) -> dict[str, Any]:
    receipt_path = _stage_receipt_path(stage_id)
    receipt = _mapping(receipt_path)
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != stage_id
        or receipt.get("passed") is not True
        or receipt.get("retry_authorized") is not False
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("pipeline_claim") != _record(PIPELINE_CLAIM)
        or receipt.get("worker_claim") != _record(_claim_path(stage_id))
    ):
        raise V6ResidualDaggerError(f"stage receipt drifted: {stage_id}")
    return receipt


def verify_fit_receipt(stage: str) -> dict[str, Any]:
    """Verify one fit receipt against its module, summary, and pure gate."""

    if stage not in {"p0", "p1"}:
        raise V6ResidualDaggerError(f"unknown fit receipt stage: {stage!r}")
    stage_id = f"fit_{stage}"
    receipt = verify_stage_receipt(stage_id)
    root = resolve_relative(contract.P0_ROOT if stage == "p0" else contract.P1_ROOT)
    module_path = P0_MODULE if stage == "p0" else P1_MODULE
    summary_path = root / "summary.json"
    gate_path = root / "gate.json"
    summary = _mapping(summary_path)
    gate = _mapping(gate_path)
    expected_gate = contract.fit_gate(summary, stage=stage)
    expected_gate["save_reload_byte_exact"] = summary.get("save_reload_byte_exact")
    expected_gate["passed"] = bool(
        expected_gate["passed"] and summary.get("save_reload_byte_exact") is True
    )
    if not expected_gate["passed"]:
        expected_gate["status"] = (
            contract.P0_FAIL_STATUS if stage == "p0" else contract.P1_FAIL_STATUS
        )
    if (
        receipt.get("status") != expected_gate["status"]
        or receipt.get("fit_stage") != stage
        or receipt.get("promotable") is not (stage == "p1")
        or not _tree_record_matches(receipt.get("module"), module_path)
        or not _record_matches(receipt.get("summary"), summary_path)
        or not _record_matches(receipt.get("gate"), gate_path)
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V6ResidualDaggerError(f"fit receipt closure drifted: {stage}")
    return receipt


def verify_dagger_receipt(case_id: str) -> dict[str, Any]:
    stage_id = f"collect_dagger__{case_id}"
    contract.canonical_case(case_id, dagger=True)
    receipt = verify_stage_receipt(stage_id)
    destination = resolve_relative(contract.DAGGER_ROOT / case_id)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    persisted = writer.finalized_artifact_records()
    summary = _mapping(writer.summary_path)
    gate = _mapping(writer.gate_path)
    expected_gate = contract.dagger_gate(summary)
    expected_gate["persisted_before_gate"] = persisted
    if (
        receipt.get("status") != contract.DAGGER_PASS_STATUS
        or receipt.get("case_id") != case_id
        or receipt.get("dagger_round") != 1
        or receipt.get("sample_count") != summary.get("sample_count")
        or receipt.get("completion_mode") != summary.get("completion_mode")
        or receipt.get("artifacts") != writer.artifact_records()
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V6ResidualDaggerError(f"DAgger receipt closure drifted: {case_id}")
    return receipt


def verify_development_rollout_receipt(case_id: str) -> dict[str, Any]:
    stage_id = f"develop__{case_id}"
    contract.canonical_case(case_id)
    receipt = verify_stage_receipt(stage_id)
    destination = resolve_relative(contract.DEVELOPMENT_ROOT / case_id)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    persisted = writer.finalized_artifact_records()
    summary = _mapping(writer.summary_path)
    gate = _mapping(writer.gate_path)
    expected_gate = contract.development_rollout_gate(summary)
    expected_gate["persisted_before_gate"] = persisted
    if (
        receipt.get("status") != contract.DEVELOPMENT_ROLLOUT_PASS_STATUS
        or receipt.get("case_id") != case_id
        or receipt.get("candidate_id") != summary.get("candidate_id")
        or receipt.get("artifacts") != writer.artifact_records()
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V6ResidualDaggerError(f"development receipt closure drifted: {case_id}")
    return receipt


def verify_worker_claim(stage_id: str, token: str) -> dict[str, Any]:
    """Reject direct, out-of-order, repeated, and post-ledger workers."""

    if stage_id not in contract.STAGE_IDS:
        raise V6ResidualDaggerError(f"unknown worker stage: {stage_id!r}")
    if os.path.lexists(PIPELINE_LEDGER):
        raise V6ResidualDaggerError("terminal pipeline ledger already exists")
    execution = verify_pipeline_claim(token)
    index = contract.STAGE_IDS.index(stage_id)
    previous = [
        {
            "stage_id": prior,
            "receipt": _record(_stage_receipt_path(prior)),
        }
        for prior in contract.STAGE_IDS[:index]
        if verify_stage_receipt(prior)
    ]
    expected = _worker_claim_payload(
        stage_id=stage_id,
        token_sha256=execution["execution_token_sha256"],
        previous_receipts=previous,
    )
    observed = _mapping(_claim_path(stage_id))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V6ResidualDaggerError(f"worker claim/order drifted: {stage_id}")
    if os.path.lexists(_stage_receipt_path(stage_id)):
        raise V6ResidualDaggerError(f"worker stage already completed: {stage_id}")
    for later_stage in contract.STAGE_IDS[index + 1 :]:
        if os.path.lexists(_stage_receipt_path(later_stage)):
            raise V6ResidualDaggerError(
                f"later stage already exists before {stage_id}: {later_stage}"
            )
    return observed


def verify_teacher_replay() -> dict[str, Any]:
    ledger_path = resolve_relative(contract.TEACHER_REPLAY_LEDGER_PATH)
    ledger = _mapping(ledger_path)
    if (
        ledger.get("status") != teacher_contract.PROTOCOL_PASS_STATUS
        or ledger.get("passed") is not True
        or ledger.get("completed_cases") != list(teacher_contract.CASE_IDS)
        or ledger.get("candidate_created") is not False
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
    ):
        raise V6ResidualDaggerError("teacher replay ledger is not a terminal PASS")
    for case_id in teacher_contract.CASE_IDS:
        teacher_collector.verify_case_receipt(case_id)
    return ledger


def _float32_array(value: Any, *, shape: tuple[int, ...], label: str, np: Any):
    array = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise V6ResidualDaggerError(f"{label} is not finite float32 {shape}")
    return array


def load_teacher_corpus(*, np: Any) -> dict[str, Any]:
    """Load exactly the six already-gated V25 teacher-action traces."""

    verify_teacher_replay()
    observations: list[Any] = []
    targets: list[Any] = []
    reset_mask: list[bool] = []
    origins: list[str] = []
    for case_id in teacher_contract.CASE_IDS:
        trace_path = resolve_relative(
            contract.TEACHER_REPLAY_ROOT / case_id / "trace.json"
        )
        rows = _sequence(trace_path)
        if len(rows) != contract.EXPECTED_STEPS:
            raise V6ResidualDaggerError(f"teacher trace length drifted: {case_id}")
        for index, raw in enumerate(rows):
            if not isinstance(raw, Mapping) or raw.get("step") != index + 1:
                raise V6ResidualDaggerError(
                    f"teacher trace order drifted: {case_id}/{index + 1}"
                )
            observation = _float32_array(
                raw.get("v25_observation"),
                shape=(contract.EXPECTED_ACTOR_FEATURES,),
                label=f"{case_id}/{index + 1} V25 observation",
                np=np,
            )
            target = _float32_array(
                raw.get("queried_teacher_mean"),
                shape=(contract.EXPECTED_ACTION_DIM,),
                label=f"{case_id}/{index + 1} teacher mean",
                np=np,
            )
            if np.any(np.abs(target) > 1.0):
                raise V6ResidualDaggerError("teacher mean exceeds action bounds")
            observations.append(observation)
            targets.append(target)
            reset_mask.append(index == 0)
            origins.append(f"teacher:{case_id}:{index + 1}")
    observation_array = np.ascontiguousarray(observations, dtype=np.float32)
    target_array = np.ascontiguousarray(targets, dtype=np.float32)
    reset_array = np.asarray(reset_mask, dtype=np.bool_)
    if (
        observation_array.shape != (6 * contract.EXPECTED_STEPS, 35)
        or target_array.shape != (6 * contract.EXPECTED_STEPS, 2)
        or np.count_nonzero(reset_array) != 6
    ):
        raise V6ResidualDaggerError("teacher corpus aggregate drifted")
    return {
        "observations": observation_array,
        "targets": target_array,
        "reset_mask": reset_array,
        "origins": np.asarray(origins, dtype="U96"),
    }


def load_dagger_corpus(*, np: Any) -> dict[str, Any]:
    observations: list[Any] = []
    targets: list[Any] = []
    reset_mask: list[bool] = []
    origins: list[str] = []
    for case_id in contract.DAGGER_CASE_IDS:
        receipt = verify_dagger_receipt(case_id)
        if receipt.get("status") != contract.DAGGER_PASS_STATUS:
            raise V6ResidualDaggerError(f"DAgger receipt failed: {case_id}")
        trace_path = resolve_relative(contract.DAGGER_ROOT / case_id / "trace.json")
        rows = _sequence(trace_path)
        if not rows or len(rows) > contract.EXPECTED_STEPS:
            raise V6ResidualDaggerError(f"DAgger trace length drifted: {case_id}")
        for index, raw in enumerate(rows):
            if not isinstance(raw, Mapping) or raw.get("step") != index + 1:
                raise V6ResidualDaggerError(
                    f"DAgger trace order drifted: {case_id}/{index + 1}"
                )
            observations.append(
                _float32_array(
                    raw.get("v25_observation"),
                    shape=(35,),
                    label=f"{case_id}/{index + 1} DAgger observation",
                    np=np,
                )
            )
            target = _float32_array(
                raw.get("counterfactual_teacher_mean"),
                shape=(2,),
                label=f"{case_id}/{index + 1} DAgger teacher",
                np=np,
            )
            if np.any(np.abs(target) > 1.0):
                raise V6ResidualDaggerError("DAgger teacher exceeds action bounds")
            targets.append(target)
            reset_mask.append(index == 0)
            origins.append(f"dagger:{case_id}:{index + 1}")
    result = {
        "observations": np.ascontiguousarray(observations, dtype=np.float32),
        "targets": np.ascontiguousarray(targets, dtype=np.float32),
        "reset_mask": np.asarray(reset_mask, dtype=np.bool_),
        "origins": np.asarray(origins, dtype="U96"),
    }
    if (
        result["observations"].ndim != 2
        or result["observations"].shape[1:] != (35,)
        or result["targets"].shape != (len(result["observations"]), 2)
        or len(result["observations"])
        < len(contract.DAGGER_CASE_IDS) * contract.MIN_DAGGER_SAMPLES_PER_CASE
        or len(result["observations"])
        > len(contract.DAGGER_CASE_IDS) * contract.EXPECTED_STEPS
        or np.count_nonzero(result["reset_mask"]) != len(contract.DAGGER_CASE_IDS)
    ):
        raise V6ResidualDaggerError("combined DAgger corpus drifted")
    return result


def _normalization(observations: Any, *, np: Any) -> tuple[Any, Any]:
    selected = np.asarray(
        observations[:, list(contract.RESIDUAL_INPUT_INDICES)], dtype=np.float32
    )
    mean = np.ascontiguousarray(
        selected.mean(axis=0, dtype=np.float64), dtype=np.float32
    )
    raw_std = selected.std(axis=0, dtype=np.float64)
    std = np.ascontiguousarray(
        np.maximum(raw_std, contract.NORMALIZATION_STD_FLOOR), dtype=np.float32
    )
    if (
        mean.shape != (33,)
        or std.shape != (33,)
        or not np.all(np.isfinite(mean))
        or not np.all(np.isfinite(std))
        or np.any(std < contract.NORMALIZATION_STD_FLOOR)
    ):
        raise V6ResidualDaggerError("P0 normalization is malformed")
    return mean, std


def _state_exact(left: Mapping[str, Any], right: Mapping[str, Any], *, np: Any) -> bool:
    if set(left) != set(right):
        return False
    for key in left:
        a = np.asarray(left[key])
        b = np.asarray(right[key])
        if a.dtype != b.dtype or a.shape != b.shape or a.tobytes() != b.tobytes():
            return False
    return True


def _fit_stage(stage: str) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.columns import Columns
    from ray.rllib.core.rl_module.rl_module import RLModule

    if stage == "p0":
        output_root = resolve_relative(contract.P0_ROOT)
        module_path = P0_MODULE
        source_checkpoint = SOURCE_H0_MODULE
        teacher = load_teacher_corpus(np=np)
        corpus = teacher
        dagger_count = 0
        mean, std = _normalization(corpus["observations"], np=np)
        module = build_v25_residual_module_from_checkpoint(
            source_checkpoint,
            input_mean=mean.tolist(),
            input_std=std.tolist(),
            residual_limits=contract.RESIDUAL_LIMITS,
            init_seed=contract.FIT_SEED,
        )
        fit_spec = contract.P0_FIT
    elif stage == "p1":
        output_root = resolve_relative(contract.P1_ROOT)
        module_path = P1_MODULE
        source_checkpoint = P0_MODULE
        verify_fit_receipt("p0")
        teacher = load_teacher_corpus(np=np)
        dagger = load_dagger_corpus(np=np)
        corpus = {
            key: np.concatenate((teacher[key], dagger[key]), axis=0)
            for key in ("observations", "targets", "reset_mask", "origins")
        }
        dagger_count = len(dagger["observations"])
        loaded = RLModule.from_checkpoint(source_checkpoint)
        if not isinstance(loaded, PrimarySplitV25ResidualTorchRLModule):
            raise V6ResidualDaggerError("P0 checkpoint is not the residual module")
        module = loaded
        mean = np.asarray(
            module.primary_split_v25_residual_input_mean.detach().cpu(),
            dtype=np.float32,
        )
        std = np.asarray(
            module.primary_split_v25_residual_input_std.detach().cpu(),
            dtype=np.float32,
        )
        expected_mean, expected_std = _normalization(teacher["observations"], np=np)
        if (
            mean.tobytes() != expected_mean.tobytes()
            or std.tobytes() != expected_std.tobytes()
        ):
            raise V6ResidualDaggerError("P1 normalization drifted from frozen P0")
        fit_spec = contract.P1_FIT
    else:
        raise V6ResidualDaggerError(f"unknown fit stage: {stage}")

    if os.path.lexists(output_root) or os.path.lexists(module_path):
        raise V6ResidualDaggerError(f"fit output is already occupied: {stage}")
    output_root.mkdir(parents=True, exist_ok=False)
    observations = np.ascontiguousarray(corpus["observations"], dtype=np.float32)
    targets = np.ascontiguousarray(corpus["targets"], dtype=np.float32)
    reset_mask = np.asarray(corpus["reset_mask"], dtype=np.bool_)
    weights = np.ones(len(observations), dtype=np.float32)
    weights[reset_mask] = np.float32(contract.RESET_ROW_WEIGHT)
    if (
        observations.shape != (len(targets), 35)
        or targets.shape != (len(observations), 2)
        or not np.all(np.isfinite(observations))
        or not np.all(np.isfinite(targets))
        or not np.all(np.isfinite(weights))
    ):
        raise V6ResidualDaggerError("fit corpus is malformed")

    source_h0 = RLModule.from_checkpoint(SOURCE_H0_MODULE)
    source_state = source_h0.get_state()
    base_before = {
        key: np.asarray(module.get_state()[key]).copy()
        for key in base_state_keys(module)
    }
    if not _state_exact(source_state, base_before, np=np):
        raise V6ResidualDaggerError("residual module base is not byte-exact H0")

    dataset_path = _write_npz_exclusive(
        output_root / "corpus.npz",
        observations=observations,
        targets=targets,
        reset_mask=reset_mask,
        sample_weights=weights,
        origins=corpus["origins"],
        residual_input_mean=mean,
        residual_input_std=std,
    )
    torch.manual_seed(contract.FIT_SEED)
    parameters = module.prepare_residual_fit()
    optimizer = torch.optim.AdamW(
        parameters,
        lr=contract.learning_rate(stage, 1),
        weight_decay=contract.WEIGHT_DECAY,
    )
    x = torch.as_tensor(observations, dtype=torch.float32)
    y = torch.as_tensor(targets, dtype=torch.float32)
    w = torch.as_tensor(weights, dtype=torch.float32)
    initial_loss: float | None = None
    final_loss: float | None = None
    for epoch in range(1, int(fit_spec["epochs"]) + 1):
        lr = contract.learning_rate(stage, epoch)
        for group in optimizer.param_groups:
            group["lr"] = lr
        optimizer.zero_grad(set_to_none=True)
        prediction = module._policy_logits({Columns.OBS: x})[:, :2]
        row_loss = torch.mean(torch.square(prediction - y), dim=1)
        loss = torch.sum(w * row_loss) / torch.sum(w)
        if not bool(torch.isfinite(loss)):
            raise V6ResidualDaggerError(f"non-finite {stage} loss at epoch {epoch}")
        loss.backward()
        norm = torch.nn.utils.clip_grad_norm_(parameters, contract.GRAD_CLIP_NORM)
        if not bool(torch.isfinite(norm)):
            raise V6ResidualDaggerError(
                f"non-finite {stage} gradient norm at epoch {epoch}"
            )
        optimizer.step()
        scalar = float(loss.detach().cpu())
        if initial_loss is None:
            initial_loss = scalar
        final_loss = scalar
        if epoch == 1 or epoch % 500 == 0 or epoch == fit_spec["epochs"]:
            print(
                f"[V6 residual/{stage}] {epoch:4d}/{fit_spec['epochs']} "
                f"lr={lr:.1e} loss={scalar:.9g}",
                flush=True,
            )

    module.eval()
    with torch.no_grad():
        logits = module._policy_logits({Columns.OBS: x}).detach().cpu().numpy()
        source_logits = (
            source_h0._policy_logits({Columns.OBS: x}).detach().cpu().numpy()
        )
    predictions = np.ascontiguousarray(logits[:, :2], dtype=np.float32)
    metrics = prediction_metrics(predictions, targets)
    reset_error = np.abs(
        predictions[reset_mask].astype(np.float64)
        - targets[reset_mask].astype(np.float64)
    )
    metrics["reset_max_abs_error"] = float(np.max(reset_error, initial=0.0))
    after = module.get_state()
    base_after = {key: np.asarray(after[key]).copy() for key in base_state_keys(module)}
    base_exact = _state_exact(source_state, base_after, np=np)
    logstd_exact = (
        logits[:, 2:].dtype == source_logits[:, 2:].dtype
        and logits[:, 2:].shape == source_logits[:, 2:].shape
        and logits[:, 2:].tobytes() == source_logits[:, 2:].tobytes()
    )
    if os.path.lexists(module_path):
        raise V6ResidualDaggerError(f"module destination appeared: {module_path}")
    module.save_to_path(module_path)
    reloaded = RLModule.from_checkpoint(module_path)
    reloaded_state = reloaded.get_state()
    save_reload_exact = _state_exact(after, reloaded_state, np=np)
    all_finite = bool(
        np.all(np.isfinite(predictions))
        and all(np.all(np.isfinite(np.asarray(value))) for value in after.values())
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": f"H0_PRIMARY_SPLIT_V6_{stage.upper()}_FIT_COMPLETE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit": dict(fit_spec),
        "sample_count": len(observations),
        "teacher_sample_count": len(teacher["observations"]),
        "dagger_sample_count": dagger_count,
        "reset_row_count": int(np.count_nonzero(reset_mask)),
        "metrics": metrics,
        "initial_weighted_loss": initial_loss,
        "final_weighted_loss": final_loss,
        "all_finite": all_finite,
        "base_h0_byte_exact": base_exact,
        "critic_byte_exact": base_exact,
        "logstd_byte_exact": logstd_exact,
        "save_reload_byte_exact": save_reload_exact,
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
        "residual_input_mean": mean.tolist(),
        "residual_input_std": std.tolist(),
        "normalization_frozen": True,
        "optimizer_steps": int(fit_spec["epochs"]),
        "promotable": bool(fit_spec["promotable"]),
        "corpus": _record(dataset_path),
        "corpus_observations_sha256": array_sha256(observations),
        "corpus_targets_sha256": array_sha256(targets),
        "module": _tree_record(module_path),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    summary_path = forensic.write_json_exclusive(output_root / "summary.json", summary)
    gate = contract.fit_gate(summary, stage=stage)
    gate["save_reload_byte_exact"] = save_reload_exact
    gate["passed"] = bool(gate["passed"] and save_reload_exact)
    if not gate["passed"]:
        gate["status"] = (
            contract.P0_FAIL_STATUS if stage == "p0" else contract.P1_FAIL_STATUS
        )
    gate_path = forensic.write_json_exclusive(output_root / "gate.json", gate)
    stage_id = f"fit_{stage}"
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "fit_stage": stage,
        "promotable": bool(fit_spec["promotable"]),
        "module": _tree_record(module_path),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(
        resolve_relative(contract.stage_receipt_path(stage_id)), receipt
    )
    if not gate["passed"]:
        raise V6ResidualDaggerError(f"{stage} offline fit gate failed")
    return {**receipt, "receipt": _record(receipt_path)}


def _freeze_p1() -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    p1_receipt = verify_fit_receipt("p1")
    if p1_receipt.get("status") != contract.P1_PASS_STATUS:
        raise V6ResidualDaggerError("P1 is not an offline PASS")
    if os.path.lexists(CANDIDATE_FREEZE):
        raise V6ResidualDaggerError("candidate freeze is already occupied")
    module_record = _tree_record(P1_MODULE)
    candidate_id = f"H0_PRIMARY_SPLIT_V6_P1_{module_record['tree_sha256'][:16]}"
    module = RLModule.from_checkpoint(P1_MODULE)
    if not isinstance(module, PrimarySplitV25ResidualTorchRLModule):
        raise V6ResidualDaggerError("P1 module class drifted")
    mean = module.primary_split_v25_residual_input_mean.detach().cpu().tolist()
    std = module.primary_split_v25_residual_input_std.detach().cpu().tolist()
    limits = module.primary_split_v25_residual_limits.detach().cpu().tolist()
    if (
        len(mean) != 33
        or len(std) != 33
        or any(not math.isfinite(float(value)) for value in (*mean, *std, *limits))
        or any(float(value) < contract.NORMALIZATION_STD_FLOOR for value in std)
        or any(
            abs(float(a) - float(b)) > 1.0e-7
            for a, b in zip(limits, contract.RESIDUAL_LIMITS)
        )
    ):
        raise V6ResidualDaggerError("serialized P1 residual contract drifted")
    stage_id = "freeze_p1"
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_id": candidate_id,
        "candidate_module": module_record,
        "p1_fit_receipt": _record(resolve_relative(contract.P1_RECEIPT_PATH)),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "architecture": {
            "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
            "residual_input_count": contract.RESIDUAL_INPUT_COUNT,
            "hidden_dims": list(contract.RESIDUAL_HIDDEN_DIMS),
            "action_dim": contract.EXPECTED_ACTION_DIM,
            "limits": [float(value) for value in limits],
            "init_seed": contract.FIT_SEED,
        },
        "model_config": {
            "primary_split_v25_residual_input_mean": [float(value) for value in mean],
            "primary_split_v25_residual_input_std": [float(value) for value in std],
            "primary_split_v25_residual_limits": [float(value) for value in limits],
            "primary_split_v25_residual_init_seed": contract.FIT_SEED,
            "primary_split_v25_residual_reset_bypass": False,
        },
        "fit": {"p0": dict(contract.P0_FIT), "p1": dict(contract.P1_FIT)},
        "dagger_rounds": 1,
        "dagger_case_ids": list(contract.DAGGER_CASE_IDS),
        "p0_promotable": False,
        "p1_unique_final_candidate": True,
        "base_h0_byte_exact": True,
        "critic_byte_exact": True,
        "logstd_byte_exact": True,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 2,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    path = forensic.write_json_exclusive(CANDIDATE_FREEZE, receipt)
    return {**receipt, "receipt": _record(path)}


def _frozen_innovations(case_id: str, *, np: Any) -> Any:
    """Recover the already-opened V5 standard normals without fresh RNG."""

    case = contract.canonical_case(case_id)
    rows = _sequence(resolve_relative(case["teacher_trace"]))
    if len(rows) != contract.EXPECTED_STEPS:
        raise V6ResidualDaggerError(f"condition trace length drifted: {case_id}")
    values = np.zeros((contract.EXPECTED_STEPS, 2), dtype=np.float32)
    if case["action_selection"] == "deterministic":
        return values
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise V6ResidualDaggerError("condition trace row is malformed")
        raw = _float32_array(
            row.get("frozen_raw_action"),
            shape=(2,),
            label=f"{case_id}/{index + 1} frozen action",
            np=np,
        )
        mean = _float32_array(
            row.get("frozen_teacher_mean"),
            shape=(2,),
            label=f"{case_id}/{index + 1} frozen mean",
            np=np,
        )
        std = _float32_array(
            row.get("teacher_std"),
            shape=(2,),
            label=f"{case_id}/{index + 1} frozen std",
            np=np,
        )
        if not np.allclose(std, contract.EXPECTED_SIGMA, rtol=0.0, atol=1.0e-8):
            raise V6ResidualDaggerError("frozen condition logstd drifted")
        values[index] = np.asarray((raw - mean) / std, dtype=np.float32)
    if not np.all(np.isfinite(values)):
        raise V6ResidualDaggerError("derived frozen innovations are non-finite")
    return values


def _candidate_freeze() -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    receipt = _mapping(CANDIDATE_FREEZE)
    module_record = _tree_record(P1_MODULE)
    expected_id = f"H0_PRIMARY_SPLIT_V6_P1_{module_record['tree_sha256'][:16]}"
    architecture = receipt.get("architecture")
    model_config = receipt.get("model_config")
    module = RLModule.from_checkpoint(P1_MODULE)
    if not isinstance(module, PrimarySplitV25ResidualTorchRLModule):
        raise V6ResidualDaggerError("frozen candidate module class drifted")
    serialized_mean = (
        module.primary_split_v25_residual_input_mean.detach().cpu().tolist()
    )
    serialized_std = module.primary_split_v25_residual_input_std.detach().cpu().tolist()
    serialized_limits = module.primary_split_v25_residual_limits.detach().cpu().tolist()
    if (
        receipt.get("status") != contract.CANDIDATE_FREEZE_STATUS
        or receipt.get("passed") is not True
        or receipt.get("candidate_id") != expected_id
        or receipt.get("candidate_module") != module_record
        or not _record_matches(
            receipt.get("p1_fit_receipt"), resolve_relative(contract.P1_RECEIPT_PATH)
        )
        or receipt.get("source_h0") != _tree_record(SOURCE_H0_MODULE)
        or receipt.get("target_contract_id") != contract.TARGET_CONTRACT_ID
        or architecture
        != {
            "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
            "residual_input_count": contract.RESIDUAL_INPUT_COUNT,
            "hidden_dims": list(contract.RESIDUAL_HIDDEN_DIMS),
            "action_dim": contract.EXPECTED_ACTION_DIM,
            "limits": serialized_limits,
            "init_seed": contract.FIT_SEED,
        }
        or model_config
        != {
            "primary_split_v25_residual_input_mean": serialized_mean,
            "primary_split_v25_residual_input_std": serialized_std,
            "primary_split_v25_residual_limits": serialized_limits,
            "primary_split_v25_residual_init_seed": contract.FIT_SEED,
            "primary_split_v25_residual_reset_bypass": False,
        }
        or receipt.get("fit")
        != {"p0": dict(contract.P0_FIT), "p1": dict(contract.P1_FIT)}
        or receipt.get("dagger_rounds") != 1
        or receipt.get("dagger_case_ids") != list(contract.DAGGER_CASE_IDS)
        or receipt.get("p0_promotable") is not False
        or receipt.get("p1_unique_final_candidate") is not True
        or receipt.get("base_h0_byte_exact") is not True
        or receipt.get("critic_byte_exact") is not True
        or receipt.get("logstd_byte_exact") is not True
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise V6ResidualDaggerError("candidate freeze receipt drifted")
    return receipt


def _is_fsm_event_rejection(error: BaseException) -> bool:
    """Narrowly classify the sole non-full DAgger completion mode."""

    if not isinstance(error, ValueError):
        return False
    prefix = "Actor FSM rejected a V20 active event: "
    message = str(error)
    if not message.startswith(prefix):
        return False
    try:
        payload = json.loads(message[len(prefix) :])
    except (json.JSONDecodeError, TypeError, ValueError):
        return False
    if not isinstance(payload, Mapping) or set(payload) != {
        "invalid_event_type",
        "state_name",
        "adapted_events",
    }:
        return False
    if not isinstance(payload["invalid_event_type"], str) or not payload[
        "invalid_event_type"
    ]:
        return False
    if not isinstance(payload["state_name"], str) or not payload["state_name"]:
        return False
    events = payload["adapted_events"]
    if not isinstance(events, list) or not events:
        return False
    required = {
        "side",
        "event",
        "source",
        "event_contract_id",
        "event_time_s",
        "confirmed_time_s",
        "delivered_time_s",
    }
    for event in events:
        if not isinstance(event, Mapping) or set(event) != required:
            return False
        if (
            event["side"] != "left"
            or event["event"] not in {"heel_strike", "toe_off"}
            or event["source"] != "v25_fsm_v20"
            or event["event_contract_id"] != contract.EVENT_CONTRACT_ID
        ):
            return False
        times = (
            event["event_time_s"],
            event["confirmed_time_s"],
            event["delivered_time_s"],
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in times
        ):
            return False
        if not float(times[0]) <= float(times[1]) <= float(times[2]):
            return False
    return True


def _legacy_shadow_fsm(base: Any) -> Any:
    from prosthetic_phase_fsm import ProstheticPhaseFSM

    config = dataclasses.replace(base._phase_fsm.config, event_source="legacy_events")
    shadow = ProstheticPhaseFSM(config)
    if shadow.observation() != {
        "phase_fsm_wait_hs": 1.0,
        "phase_fsm_stance_after_hs": 0.0,
        "phase_fsm_swing_after_to": 0.0,
        "phase_expected_hs": 1.0,
        "phase_expected_to": 0.0,
        "phase_stance_elapsed_norm": 0.0,
        "phase_swing_elapsed_norm": 0.0,
        "phase_cycle_progress_credit": 0.0,
    }:
        raise V6ResidualDaggerError("legacy counterfactual shadow reset drifted")
    return shadow


def _validate_runtime_layout(
    *, module: Any, env: Any, observation: Any, rollout_eval: Any, np: Any
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
    full_names = tuple(str(name) for name in env.unwrapped.observation_feature_names)
    rollout_eval._validate_module_observation_contract(module, actor_names, full_names)
    if (
        np.asarray(observation).shape != (contract.EXPECTED_FULL_FEATURES,)
        or np.asarray(observation).dtype != np.dtype(contract.EXPECTED_DTYPE)
        or actor_names != teacher_contract.EXPECTED_ACTOR_FEATURE_NAMES
        or full_names != teacher_contract.EXPECTED_OBSERVATION_FEATURE_NAMES
        or tuple(env.action_space.shape) != (contract.EXPECTED_ACTION_DIM,)
    ):
        raise V6ResidualDaggerError("runtime layout drifted from 35/84 float32")
    return actor_names, full_names


def _policy_values(
    *,
    module: Any,
    observation: Any,
    action_selection: str,
    innovation: Any,
    env: Any,
    rollout_eval: Any,
    np: Any,
) -> tuple[Any, Any, Any]:
    raw, mean, std, _ = legacy._policy_values(
        module=module,
        obs=observation,
        action_shape=tuple(env.action_space.shape),
        standard_normal=(innovation if action_selection == "stochastic" else None),
        stochastic=action_selection == "stochastic",
        rollout_eval=rollout_eval,
    )
    raw = _float32_array(raw, shape=(2,), label="raw policy action", np=np)
    mean = _float32_array(mean, shape=(2,), label="policy mean", np=np)
    std = _float32_array(std, shape=(2,), label="policy std", np=np)
    if not np.allclose(std, contract.EXPECTED_SIGMA, rtol=0.0, atol=1.0e-8):
        raise V6ResidualDaggerError("policy logstd drifted from H0")
    return raw, mean, std


def _dagger_start_payload(*, case: Mapping[str, Any], stage_id: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DAGGER_COLLECTION_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "dagger_round": 1,
        "case": dict(case),
        "behavior": "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "teacher": "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
        "p0_module": _tree_record(P0_MODULE),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "second_dagger_round_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _collect_dagger(case_id: str) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    stage_id = f"collect_dagger__{case_id}"
    case = contract.canonical_case(case_id, dagger=True)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V6ResidualDaggerError(f"DAgger destination already exists: {case_id}")
    p0_receipt = verify_fit_receipt("p0")
    if (
        p0_receipt.get("status") != contract.P0_PASS_STATUS
        or p0_receipt.get("promotable") is not False
    ):
        raise V6ResidualDaggerError("P0 is not a gated non-promotable interim")
    rollout_eval, _np, _torch, _RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    if _np is not np or _torch is not torch:
        raise V6ResidualDaggerError("inference stack identity drifted")
    candidate = RLModule.from_checkpoint(P0_MODULE)
    teacher = RLModule.from_checkpoint(SOURCE_H0_MODULE)
    if not isinstance(candidate, PrimarySplitV25ResidualTorchRLModule):
        raise V6ResidualDaggerError("P0 runtime module class drifted")
    candidate.eval()
    teacher.eval()
    innovations = _frozen_innovations(case_id, np=np)
    env_config = teacher_collector.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(_dagger_start_payload(case=case, stage_id=stage_id))
    rows: list[dict[str, Any]] = []
    selected_before_teacher_count = 0
    served_dependency_count = 0
    clipped_values = 0
    nonfinite_count = 0
    completion_mode: str | None = None
    rejection: dict[str, Any] | None = None
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    phase_valid_cycles = 0
    penetration_max = 0.0
    raw_sample_count = 0
    control_window_count = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    so_solver_unaccepted_count = 0
    sea_plugin_fallback_count = 0
    routing_failure_count = 0
    try:
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, _full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        rollout_eval._validate_module_observation_contract(
            teacher, actor_names, tuple(env.unwrapped.observation_feature_names)
        )
        if (
            reset_info.get("binary_phase_fsm_executed") is not True
            or reset_info.get("binary_phase_fsm_mode") != "binary_active"
            or reset_info.get("binary_phase_event_contract_id")
            != contract.EVENT_CONTRACT_ID
        ):
            raise V6ResidualDaggerError("V25 binary-active reset routing drifted")
        shadow_fsm = _legacy_shadow_fsm(env.unwrapped)
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V6ResidualDaggerError("body weight is malformed")
        current_info: Mapping[str, Any] = dict(reset_info)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            candidate_input = observation.copy()
            raw_action, candidate_mean, candidate_std = _policy_values(
                module=candidate,
                observation=candidate_input,
                action_selection=str(case["action_selection"]),
                innovation=innovations[index],
                env=env,
                rollout_eval=rollout_eval,
                np=np,
            )
            selected_before_teacher_count += 1
            paired = build_paired_views(
                observation,
                actor_names,
                current_info,
                body_weight_n=body_weight_n,
                reset_boundary=index == 0,
                teacher_phase_observation=shadow_fsm.observation(),
            )
            if candidate_input[:35].tobytes(order="C") != paired.student.tobytes(
                order="C"
            ):
                served_dependency_count += 1
                raise V6ResidualDaggerError(
                    "candidate action input differs from causal V25 student view"
                )
            teacher_full = observation.copy()
            teacher_full[:35] = paired.teacher
            _teacher_raw, teacher_mean, teacher_std = _policy_values(
                module=teacher,
                observation=teacher_full,
                action_selection="deterministic",
                innovation=np.zeros(2, dtype=np.float32),
                env=env,
                rollout_eval=rollout_eval,
                np=np,
            )
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            clipped_values += int(np.count_nonzero(applied != raw_action))
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "case_id": case_id,
                "v25_observation": paired.student.tolist(),
                "counterfactual_teacher_observation": paired.teacher.tolist(),
                "counterfactual_teacher_mean": teacher_mean.tolist(),
                "counterfactual_teacher_std": teacher_std.tolist(),
                "candidate_mean": candidate_mean.tolist(),
                "candidate_std": candidate_std.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "standard_normal": (
                    innovations[index].tolist()
                    if case["action_selection"] == "stochastic"
                    else None
                ),
                "primary_load_bw": paired.primary_load_bw,
                "detector_load_bw": paired.detector_load_bw,
                "candidate_selected_before_teacher": True,
                "transition_outcome": "PENDING_AT_PERSISTENCE",
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            try:
                observation_after, reward, terminated, truncated, info = env.step(
                    raw_action
                )
            except Exception as exc:
                if not _is_fsm_event_rejection(exc):
                    raise
                completion_mode = "fsm_event_rejected"
                rejection = {
                    "step": step,
                    "type": type(exc).__name__,
                    "message": str(exc),
                }
                break
            observation = np.ascontiguousarray(observation_after, dtype=np.float32)
            if observation.shape != (84,) or not np.all(np.isfinite(observation)):
                nonfinite_count += 1
                raise V6ResidualDaggerError("DAgger next observation is malformed")
            if not isinstance(info, Mapping):
                raise V6ResidualDaggerError("DAgger runtime info is malformed")
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != 10
            ):
                raise V6ResidualDaggerError(
                    f"DAgger step {step} lacks ten V25 samples"
                )
            raw_sample_count += len(samples)
            if (
                info.get("binary_phase_fsm_mode") != "binary_active"
                or info.get("binary_phase_event_contract_id")
                != contract.EVENT_CONTRACT_ID
                or info.get("online_grf_applied_sides") != ["left"]
            ):
                routing_failure_count += 1
                raise V6ResidualDaggerError("DAgger routing drifted")
            reward_terms = info.get("reward_terms")
            phase = info.get("phase_fsm")
            if not isinstance(reward_terms, Mapping) or not isinstance(phase, Mapping):
                raise V6ResidualDaggerError("DAgger diagnostics are incomplete")
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise V6ResidualDaggerError("DAgger penetration is malformed")
            penetration_max = max(penetration_max, penetration)
            phase_valid_cycles = int(float(phase.get("valid_cycle_count", 0)))
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            try:
                classified = so_recovery.classify_policy_step(
                    info.get("so_solver_audit_entries"),
                    policy_id=contract.SO_POLICY_ID,
                )
            except so_recovery.SORecoveryContractError as exc:
                raise V6ResidualDaggerError(str(exc)) from exc
            counters = classified["counters"]
            control_window_count += int(counters["control_window_count"])
            so_solver_unaccepted_count += int(
                counters["unaccepted_hard_so_fallback_count"]
                + counters["unaccepted_bounded_ls_count"]
            )
            sea_plugin_fallback_count += teacher_collector._sea_fallback_count(
                info.get("sea_segment_diagnostics")
            )
            hard_invalid_count += int("failure" in info)
            finite = bool(
                np.all(np.isfinite(candidate_input))
                and np.all(np.isfinite(observation))
                and np.all(np.isfinite(raw_action))
                and np.all(np.isfinite(candidate_mean))
                and np.all(np.isfinite(candidate_std))
                and np.all(np.isfinite(teacher_mean))
                and np.all(np.isfinite(teacher_std))
                and math.isfinite(float(reward))
            )
            nonfinite_count += int(not finite)
            if not finite:
                raise V6ResidualDaggerError("DAgger step contains non-finite values")
            v1._update_shadow_fsm(
                shadow_fsm,
                info=info,
                body_weight_n=body_weight_n,
            )
            current_info = dict(info)
            if terminated or truncated:
                completion_mode = (
                    "episode_time_limit"
                    if info.get("end_reason") == "episode_time_limit"
                    and truncated
                    and not terminated
                    else "runtime_terminal"
                )
                break
        if completion_mode is None and len(rows) == contract.EXPECTED_STEPS:
            completion_mode = "episode_time_limit"
    finally:
        env.close()

    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DAGGER_PERSISTED_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "dagger_round": 1,
        "dagger_rounds_completed": 1,
        "behavior": "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "teacher": "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
        "completion_mode": completion_mode,
        "fsm_event_rejection": rejection,
        "sample_count": len(rows),
        "persisted_sample_count": writer.last_completed_step,
        "candidate_selected_before_teacher_count": selected_before_teacher_count,
        "served_action_teacher_dependency_count": served_dependency_count,
        "action_clipped_values": clipped_values,
        "nonfinite_count": nonfinite_count,
        "control_window_count": control_window_count,
        "v25_raw_sensor_sample_count": raw_sample_count,
        "timeout_count": timeout_count,
        "invalid_event_count": invalid_event_count,
        "hard_invalid_count": hard_invalid_count,
        "so_solver_unaccepted_count": so_solver_unaccepted_count,
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "routing_failure_count": routing_failure_count,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "phase_valid_cycle_count": phase_valid_cycles,
        "grf_penetration_max_m": penetration_max,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "p0_promotable": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DAGGER_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "sample_count": len(rows),
        "completion_mode": completion_mode,
        "gate_evaluated": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.dagger_gate(summary)
        gate["persisted_before_gate"] = persisted
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        raise V6ResidualDaggerError(f"DAgger collection gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DAGGER_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "dagger_round": 1,
        "sample_count": len(rows),
        "completion_mode": completion_mode,
        "artifacts": writer.artifact_records(),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    path = forensic.write_json_exclusive(writer.run_directory / "receipt.json", receipt)
    return {**receipt, "receipt": _record(path)}


def _development_start_payload(
    *, case: Mapping[str, Any], stage_id: str, freeze: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DEVELOPMENT_ROLLOUT_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case": dict(case),
        "candidate_id": freeze["candidate_id"],
        "candidate_module": freeze["candidate_module"],
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "behavior": "P1_CLOSED_LOOP_V25_BINARY_ACTIVE",
        "noise": "FROZEN_V5_DEVELOPMENT_CONDITION_INNOVATIONS",
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _develop(case_id: str) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    stage_id = f"develop__{case_id}"
    case = contract.canonical_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V6ResidualDaggerError(
            f"development destination already exists: {case_id}"
        )
    freeze = _candidate_freeze()
    rollout_eval, _np, _torch, _RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    if _np is not np or _torch is not torch:
        raise V6ResidualDaggerError("inference stack identity drifted")
    module = RLModule.from_checkpoint(P1_MODULE)
    if not isinstance(module, PrimarySplitV25ResidualTorchRLModule):
        raise V6ResidualDaggerError("P1 runtime module class drifted")
    module.eval()
    innovations = _frozen_innovations(case_id, np=np)
    env_config = teacher_collector.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        _development_start_payload(case=case, stage_id=stage_id, freeze=freeze)
    )
    rows: list[dict[str, Any]] = []
    raw_sample_count = 0
    control_window_count = 0
    fallback_count = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    nonfinite_count = 0
    action_clipped_values = 0
    sea_plugin_fallback_count = 0
    so_solver_unaccepted_count = 0
    penetrations: list[float] = []
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    try:
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=module,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        if (
            reset_info.get("binary_phase_fsm_executed") is not True
            or reset_info.get("binary_phase_fsm_mode") != "binary_active"
            or reset_info.get("binary_phase_event_contract_id")
            != contract.EVENT_CONTRACT_ID
        ):
            raise V6ResidualDaggerError("development V25 reset routing drifted")
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            obs_before = observation.copy()
            raw_action, mean, std = _policy_values(
                module=module,
                observation=obs_before,
                action_selection=str(case["action_selection"]),
                innovation=innovations[index],
                env=env,
                rollout_eval=rollout_eval,
                np=np,
            )
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            action_clipped_values += int(np.count_nonzero(raw_action != applied))
            observation_after, reward, terminated, truncated, info = env.step(
                raw_action
            )
            observation = np.ascontiguousarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise V6ResidualDaggerError("development info is malformed")
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != 10
            ):
                raise V6ResidualDaggerError(
                    f"development step {step} lacks ten V25 samples"
                )
            raw_sample_count += len(samples)
            if (
                info.get("binary_phase_fsm_mode") != "binary_active"
                or info.get("binary_phase_event_contract_id")
                != contract.EVENT_CONTRACT_ID
                or info.get("online_grf_applied_sides") != ["left"]
            ):
                raise V6ResidualDaggerError("development routing drifted")
            reward_terms = info.get("reward_terms")
            phase = info.get("phase_fsm")
            if not isinstance(reward_terms, Mapping) or not isinstance(phase, Mapping):
                raise V6ResidualDaggerError("development diagnostics are incomplete")
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise V6ResidualDaggerError("development penetration is malformed")
            penetrations.append(penetration)
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            try:
                classified = so_recovery.classify_policy_step(
                    info.get("so_solver_audit_entries"),
                    policy_id=contract.SO_POLICY_ID,
                )
            except so_recovery.SORecoveryContractError as exc:
                raise V6ResidualDaggerError(str(exc)) from exc
            counters = classified["counters"]
            control_window_count += int(counters["control_window_count"])
            step_unaccepted = int(
                counters["unaccepted_hard_so_fallback_count"]
                + counters["unaccepted_bounded_ls_count"]
            )
            so_solver_unaccepted_count += step_unaccepted
            fallback_count += step_unaccepted
            sea_plugin_fallback_count += teacher_collector._sea_fallback_count(
                info.get("sea_segment_diagnostics")
            )
            hard_invalid_count += int("failure" in info)
            finite = bool(
                np.all(np.isfinite(obs_before))
                and np.all(np.isfinite(observation))
                and np.all(np.isfinite(raw_action))
                and np.all(np.isfinite(mean))
                and np.all(np.isfinite(std))
                and math.isfinite(float(reward))
            )
            nonfinite_count += int(not finite)
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "case_id": case_id,
                "observation_before": obs_before[:35].tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": (
                    innovations[index].tolist()
                    if case["action_selection"] == "stochastic"
                    else None
                ),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
                "grf_penetration_m": penetration,
                "phase_fsm": legacy._jsonable(phase),
                "binary_phase_fsm": legacy._jsonable(info.get("binary_phase_fsm")),
                "binary_phase_active_adapter": legacy._jsonable(
                    info.get("binary_phase_active_adapter")
                ),
                "so_recovery_counters": legacy._jsonable(counters),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            if step == 1 or step % 25 == 0:
                print(
                    f"[V6 development/{case_id}] {step:3d}/500",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DEVELOPMENT_PERSISTED_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "candidate_id": freeze["candidate_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "steps": len(rows),
        "control_window_count": control_window_count,
        "v25_raw_sensor_sample_count": raw_sample_count,
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "action_clipped_values": action_clipped_values,
        "fallback_count": fallback_count,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "so_solver_unaccepted_count": so_solver_unaccepted_count,
        "hard_invalid_count": hard_invalid_count,
        "invalid_event_count": invalid_event_count,
        "nonfinite_count": nonfinite_count,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": contract.EXPECTED_DTYPE,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_DEVELOPMENT_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "candidate_id": freeze["candidate_id"],
        "steps": len(rows),
        "end_reason": summary["end_reason"],
        "gate_evaluated": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.development_rollout_gate(summary)
        gate["persisted_before_gate"] = persisted
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        raise V6ResidualDaggerError(f"development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_ROLLOUT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": freeze["candidate_id"],
        "artifacts": writer.artifact_records(),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    path = forensic.write_json_exclusive(writer.run_directory / "receipt.json", receipt)
    return {**receipt, "receipt": _record(path)}


def _finalize_development() -> dict[str, Any]:
    stage_id = "finalize_development"
    freeze = _candidate_freeze()
    case_receipts: list[dict[str, Any]] = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        receipt = verify_development_rollout_receipt(case_id)
        if (
            receipt.get("status") != contract.DEVELOPMENT_ROLLOUT_PASS_STATUS
            or receipt.get("candidate_id") != freeze["candidate_id"]
        ):
            raise V6ResidualDaggerError(
                f"development candidate/status mismatch: {case_id}"
            )
        case_receipts.append(
            {
                "case_id": case_id,
                "receipt": _record(
                    resolve_relative(
                        contract.DEVELOPMENT_ROOT / case_id / "receipt.json"
                    )
                ),
            }
        )
    if os.path.lexists(DEVELOPMENT_RECEIPT):
        raise V6ResidualDaggerError("development aggregate receipt already exists")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_id": freeze["candidate_id"],
        "candidate_freeze": _record(CANDIDATE_FREEZE),
        "case_ids": list(contract.DEVELOPMENT_CASE_IDS),
        "case_receipts": case_receipts,
        "same_six_cases_as_teacher_replay": True,
        "development_only": True,
        "qualification_eligible": True,
        "retry_authorized": False,
        "dagger_rounds": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
    }
    path = forensic.write_json_exclusive(DEVELOPMENT_RECEIPT, receipt)
    return {**receipt, "receipt": _record(path)}


def run_worker(*, stage_id: str, supervisor_token: str) -> dict[str, Any]:
    """Run one claimed worker stage; never infer or create worker authority."""

    verify_worker_claim(stage_id, supervisor_token)
    try:
        if stage_id == "fit_p0":
            return _fit_stage("p0")
        if stage_id.startswith("collect_dagger__"):
            return _collect_dagger(stage_id.removeprefix("collect_dagger__"))
        if stage_id == "fit_p1":
            return _fit_stage("p1")
        if stage_id == "freeze_p1":
            return _freeze_p1()
        if stage_id.startswith("develop__"):
            return _develop(stage_id.removeprefix("develop__"))
        if stage_id == "finalize_development":
            return _finalize_development()
    except Exception as exc:
        # Rollout workers own a forensic writer and publish their local failure
        # whenever possible.  Non-rollout stages still get a one-shot stage
        # failure receipt next to the claimed output, never a reusable retry.
        failure_path = _stage_receipt_path(stage_id).with_name("failure.json")
        if not os.path.lexists(failure_path):
            try:
                forensic.write_json_exclusive(
                    failure_path,
                    {
                        "schema_version": contract.SCHEMA_VERSION,
                        "status": contract.PIPELINE_FAIL_STATUS,
                        "passed": False,
                        "protocol_id": contract.PROTOCOL_ID,
                        "stage_id": stage_id,
                        "error": {
                            "type": type(exc).__name__,
                            "message": str(exc),
                        },
                        "pipeline_claim": _record(PIPELINE_CLAIM),
                        "worker_claim": _record(_claim_path(stage_id)),
                        "retry_authorized": False,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                    },
                )
            except Exception:
                pass
        raise
    raise V6ResidualDaggerError(f"unhandled worker stage: {stage_id}")


def _worker_command(stage_id: str, token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--stage",
        stage_id,
        "--supervisor-token",
        token,
    ]


def _preexecution_absence() -> None:
    if os.path.lexists(PIPELINE_CLAIM) or os.path.lexists(PIPELINE_LEDGER):
        raise V6ResidualDaggerError("V6 residual pipeline was already claimed")
    protected_outputs = (
        resolve_relative(contract.ADAPTATION_ROOT),
        resolve_relative(contract.DAGGER_ROOT),
        resolve_relative(contract.DEVELOPMENT_ROOT),
        resolve_relative(contract.WORKER_CLAIMS_ROOT),
    )
    occupied = [str(path) for path in protected_outputs if os.path.lexists(path)]
    if occupied:
        raise V6ResidualDaggerError(
            "V6 pipeline output is already occupied: " + ", ".join(occupied)
        )
    verify_teacher_replay()


def execute() -> dict[str, Any]:
    """Claim and execute the complete no-retry pipeline sequentially."""

    _preexecution_absence()
    token = secrets.token_urlsafe(32)
    token_sha256 = _token_sha256(token)
    forensic.write_json_exclusive(PIPELINE_CLAIM, _claim_payload(token_sha256))
    started = time.time()
    completed_stages: list[str] = []
    completed_receipts: list[dict[str, Any]] = []
    error: str | None = None
    passed = False
    try:
        for stage_id in contract.STAGE_IDS:
            previous = [
                {
                    "stage_id": prior,
                    "receipt": _record(_stage_receipt_path(prior)),
                }
                for prior in completed_stages
            ]
            forensic.write_json_exclusive(
                _claim_path(stage_id),
                _worker_claim_payload(
                    stage_id=stage_id,
                    token_sha256=token_sha256,
                    previous_receipts=previous,
                ),
            )
            completed = subprocess.run(
                _worker_command(stage_id, token),
                cwd=REPO_ROOT,
                timeout=WORKER_TIMEOUT_S,
                check=False,
            )
            if completed.returncode != 0:
                raise V6ResidualDaggerError(
                    f"worker {stage_id} exited {completed.returncode}"
                )
            verify_stage_receipt(stage_id)
            completed_stages.append(stage_id)
            completed_receipts.append(
                {
                    "stage_id": stage_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
        development = _mapping(DEVELOPMENT_RECEIPT)
        freeze = _candidate_freeze()
        if (
            development.get("status") != contract.DEVELOPMENT_PASS_STATUS
            or development.get("passed") is not True
            or development.get("candidate_id") != freeze["candidate_id"]
        ):
            raise V6ResidualDaggerError("terminal development closure drifted")
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"

    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_PASS_STATUS if passed else contract.PIPELINE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "expected_stages": list(contract.STAGE_IDS),
        "completed_stages": completed_stages,
        "completed_receipts": completed_receipts,
        "error": error,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "candidate_freeze": _record(CANDIDATE_FREEZE) if passed else None,
        "development_receipt": _record(DEVELOPMENT_RECEIPT) if passed else None,
        "dagger_rounds": 1 if "fit_p1" in completed_stages else 0,
        "retry_authorized": False,
        "second_dagger_round_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "PREPARE_FRESH_V6_QUALIFICATION_NOISE_TAPES"
            if passed
            else "STOP_WITHOUT_RETRY_RETUNING_OR_SECOND_DAGGER_ROUND"
        ),
    }
    forensic.write_json_exclusive(PIPELINE_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise V6ResidualDaggerError(error or contract.PIPELINE_FAIL_STATUS)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--stage", choices=contract.STAGE_IDS)
    parser.add_argument("--supervisor-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            if args.stage is not None or args.supervisor_token is not None:
                raise V6ResidualDaggerError(
                    "supervisor execution does not accept worker arguments"
                )
            execute()
            return 0
        if args.stage is None or args.supervisor_token is None:
            raise V6ResidualDaggerError(
                "worker requires --stage and --supervisor-token"
            )
        run_worker(stage_id=args.stage, supervisor_token=args.supervisor_token)
        return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
