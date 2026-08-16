"""Run the fresh, terminal H0 primary-split V6 qualification fail-closed.

Importing this module is source-only: it imports neither OpenSim nor the
inference stack and performs no filesystem access.  ``--execute`` is the sole
supervisor authority.  It first verifies an immutable P1 candidate freeze, a
development PASS for the same P1, and the separately materialized fresh-noise
manifest.  It then publishes a hashed supervisor claim and launches twelve
one-shot workers in the fixed order: all six original-H0 baselines followed by
all six V25-active candidate rollouts.

Every rollout journals each completed step and persists trace/partial summary/
summary before its common gate.  Every candidate is then compared with its
condition-matched baseline using the preregistered SEA/reserve tolerances.  Any
failure after the supervisor claim is terminal and consumes the qualification;
there is no resume, retry, rescue, retuning, or update path in this driver.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))

import compare_h0_primary_split_v6_qualification as gates  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v6_qualification_contract as contract  # noqa: E402
import h0_primary_split_v6_residual_dagger_contract as p1_contract  # noqa: E402


RUN_ROOT = REPO_ROOT / contract.RUN_ROOT
EXECUTION_CLAIM = RUN_ROOT / "execution_claim.json"
EXECUTION_LEDGER = RUN_ROOT / "execution_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"
CANDIDATE_FREEZE = REPO_ROOT / p1_contract.CANDIDATE_FREEZE_PATH
DEVELOPMENT_GATE = REPO_ROOT / p1_contract.DEVELOPMENT_RECEIPT_PATH
NOISE_MANIFEST = REPO_ROOT / contract.NOISE_MANIFEST_PATH
SOURCE_H0_CONFIG = (
    VALIDATION_ROOT
    / "critic_warmup"
    / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
    / "training_cfg.resolved.yaml"
)
ANALOG_PROFILE = (
    REPO_ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
V25_PROFILE = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
WORKER_TIMEOUT_S = 3600.0
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V6_QUALIFICATION_WORKER_CLAIMED"
UNGATED_STATUS = "H0_PRIMARY_SPLIT_V6_QUALIFICATION_PERSISTED_UNGATED"
PERSISTED_STATUS = "H0_PRIMARY_SPLIT_V6_QUALIFICATION_PERSISTED_BEFORE_GATE"
_MODULE_REQUIRED_FILES = {
    "module_state.pkl",
    "class_and_ctor_args.pkl",
    "metadata.json",
}
_ROLES = (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
MATRIX = tuple((role, case_id) for role in _ROLES for case_id in contract.CASE_IDS)


class QualificationExecutionError(RuntimeError):
    """Raised on every provenance, ordering, runtime, persistence, or gate error."""


def _mapping(path: str | Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise QualificationExecutionError(f"expected strict JSON object: {path}")
    return dict(value)


def _canonical(value: Any) -> bytes:
    return forensic.canonical_json_bytes(value)


def _resolve_repository_path(raw: str, label: str, *, directory: bool = False) -> Path:
    if not isinstance(raw, str) or not raw:
        raise QualificationExecutionError(f"{label} path is malformed")
    pure = PurePosixPath(raw)
    if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise QualificationExecutionError(f"{label} path is not repository-relative")
    path = REPO_ROOT.joinpath(*pure.parts).resolve()
    try:
        path.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise QualificationExecutionError(f"{label} escaped the repository") from exc
    if directory and not path.is_dir():
        raise QualificationExecutionError(f"{label} directory is missing: {path}")
    if not directory and not path.is_file():
        raise QualificationExecutionError(f"{label} file is missing: {path}")
    return path


def _portable_path(path: str | Path) -> str:
    resolved = Path(path).expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return str(resolved)


def _record(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise QualificationExecutionError(f"artifact is missing: {source}")
    return {
        "path": _portable_path(source),
        "sha256": forensic.sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def _record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and dict(record) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).expanduser().resolve()
    if not root.is_dir():
        raise QualificationExecutionError(f"artifact tree is missing: {root}")
    try:
        root_path = root.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise QualificationExecutionError("artifact tree escaped the repository") from exc
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise QualificationExecutionError(f"artifact tree is empty: {root}")
    rows: list[dict[str, Any]] = []
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root_path,
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _tree_path(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "tree_sha256",
        "file_count",
        "files",
    }:
        raise QualificationExecutionError(f"{label} tree record is malformed")
    path = _resolve_repository_path(str(record["path"]), label, directory=True)
    observed = _tree_record(path)
    if dict(record) != observed:
        raise QualificationExecutionError(f"{label} tree integrity mismatch")
    names = {str(row["path"]) for row in observed["files"]}
    if not _MODULE_REQUIRED_FILES.issubset(names):
        raise QualificationExecutionError(f"{label} lacks the RLModule core files")
    return path


def _array_sha256(array: Any, np: Any) -> str:
    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise QualificationExecutionError("supervisor token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _case(case_id: str) -> dict[str, Any]:
    rows = [row for row in contract.canonical_cases() if row["case_id"] == case_id]
    if len(rows) != 1:
        raise QualificationExecutionError(f"unknown qualification case {case_id!r}")
    return rows[0]


def _matrix_index(role: str, case_id: str) -> int:
    try:
        return MATRIX.index((role, case_id))
    except ValueError as exc:
        raise QualificationExecutionError(f"unknown worker {role}/{case_id}") from exc


def _worker_claim_path(role: str, case_id: str) -> Path:
    index = _matrix_index(role, case_id)
    return WORKER_CLAIMS_ROOT / f"{index + 1:02d}_{role}_{case_id}.json"


def _rollout_root(role: str, case_id: str) -> Path:
    return REPO_ROOT / contract.rollout_destination(role, case_id)


def _receipt_path(role: str, case_id: str) -> Path:
    return _rollout_root(role, case_id) / "receipt.json"


def _gate_path(case_id: str) -> Path:
    return REPO_ROOT / contract.gate_destination(case_id)


def _validate_prerequisites() -> dict[str, Any]:
    freeze = _mapping(CANDIDATE_FREEZE)
    development = _mapping(DEVELOPMENT_GATE)
    candidate_id = freeze.get("candidate_id")
    if (
        freeze.get("schema_version") != contract.SCHEMA_VERSION
        or freeze.get("status") != contract.CANDIDATE_FREEZE_REQUIRED_STATUS
        or freeze.get("passed") is not True
        or freeze.get("protocol_id") != p1_contract.PROTOCOL_ID
        or freeze.get("stage_id") != "freeze_p1"
        or not isinstance(candidate_id, str)
        or not candidate_id.strip()
        or freeze.get("target_contract_id") != p1_contract.TARGET_CONTRACT_ID
        or freeze.get("fit")
        != {"p0": dict(p1_contract.P0_FIT), "p1": dict(p1_contract.P1_FIT)}
        or freeze.get("dagger_rounds") != 1
        or freeze.get("dagger_case_ids") != list(p1_contract.DAGGER_CASE_IDS)
        or freeze.get("p0_promotable") is not False
        or freeze.get("p1_unique_final_candidate") is not True
        or freeze.get("base_h0_byte_exact") is not True
        or freeze.get("critic_byte_exact") is not True
        or freeze.get("logstd_byte_exact") is not True
        or freeze.get("retry_authorized") is not False
        or freeze.get("actor_updates") != 2
        or freeze.get("critic_updates") != 0
        or freeze.get("ppo_updates") != 0
        or freeze.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError("P1 candidate-freeze prerequisite drifted")
    candidate_module = _tree_path(freeze.get("candidate_module"), "P1 candidate")
    source_h0 = _tree_path(freeze.get("source_h0"), "source H0")
    expected_candidate_id = (
        "H0_PRIMARY_SPLIT_V6_P1_"
        f"{freeze['candidate_module']['tree_sha256'][:16]}"
    )
    if candidate_id != expected_candidate_id:
        raise QualificationExecutionError("P1 candidate id/tree digest mismatch")
    if (
        development.get("schema_version") != contract.SCHEMA_VERSION
        or development.get("status") != contract.DEVELOPMENT_PASS_REQUIRED_STATUS
        or development.get("passed") is not True
        or development.get("protocol_id") != p1_contract.PROTOCOL_ID
        or development.get("stage_id") != "finalize_development"
        or development.get("candidate_id") != candidate_id
        or development.get("candidate_freeze") != _record(CANDIDATE_FREEZE)
        or development.get("case_ids") != list(p1_contract.DEVELOPMENT_CASE_IDS)
        or development.get("same_six_cases_as_teacher_replay") is not True
        or development.get("development_only") is not True
        or development.get("qualification_eligible") is not True
        or development.get("retry_authorized") is not False
        or development.get("dagger_rounds") != 1
        or development.get("actor_updates") != 0
        or development.get("critic_updates") != 0
        or development.get("ppo_updates") != 0
        or development.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError("P1 development prerequisite drifted")
    for path, label in (
        (SOURCE_H0_CONFIG, "source H0 config"),
        (ANALOG_PROFILE, "analog profile"),
        (V25_PROFILE, "V25 profile"),
    ):
        if not path.is_file():
            raise QualificationExecutionError(f"{label} is missing: {path}")
    return {
        "candidate_id": candidate_id,
        "candidate_module_path": candidate_module,
        "source_h0_path": source_h0,
        "candidate_freeze": freeze,
        "development_gate": development,
    }


def _validate_noise_manifest(candidate_id: str) -> dict[str, Any]:
    """Verify the manifest, every tape file, and every decoded array."""

    import numpy as np

    manifest = _mapping(NOISE_MANIFEST)
    if (
        manifest.get("schema_version") != contract.SCHEMA_VERSION
        or manifest.get("revision") != contract.REVISION
        or manifest.get("status") != contract.NOISE_TAPES_STATUS
        or manifest.get("protocol_id") != contract.PROTOCOL_ID
        or manifest.get("candidate_id") != candidate_id
        or manifest.get("generator")
        != "numpy.random.default_rng(seed).standard_normal"
        or manifest.get("standard_normal_pre_scaling") is not True
        or manifest.get("shape")
        != [contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE]
        or manifest.get("dtype") != contract.EXPECTED_DTYPE
        or manifest.get("qualification_execution_authorized_by_manifest") is not False
        or manifest.get("retry_authorized") is not False
        or manifest.get("actor_updates") != 0
        or manifest.get("critic_updates") != 0
        or manifest.get("ppo_updates") != 0
        or manifest.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError("qualification noise manifest drifted")
    prerequisites = manifest.get("prerequisites")
    if not isinstance(prerequisites, Mapping) or set(prerequisites) != {
        "candidate_freeze",
        "development_gate",
    }:
        raise QualificationExecutionError("noise prerequisite closure is malformed")
    for name, status, passed, path in (
        (
            "candidate_freeze",
            contract.CANDIDATE_FREEZE_REQUIRED_STATUS,
            True,
            CANDIDATE_FREEZE,
        ),
        (
            "development_gate",
            contract.DEVELOPMENT_PASS_REQUIRED_STATUS,
            True,
            DEVELOPMENT_GATE,
        ),
    ):
        row = prerequisites.get(name)
        if (
            not isinstance(row, Mapping)
            or row.get("status") != status
            or row.get("passed") is not passed
            or row.get("candidate_id") != candidate_id
            or row.get("artifact") != _record(path)
        ):
            raise QualificationExecutionError(f"noise prerequisite {name} drifted")

    tapes = manifest.get("tapes")
    cases = manifest.get("cases")
    expected_names = {
        "deterministic_all_zero.npz",
        *(f"stochastic_seed_{seed}_standard_normal.npz" for seed in contract.STOCHASTIC_SEEDS),
    }
    if not isinstance(tapes, Mapping) or set(tapes) != expected_names:
        raise QualificationExecutionError("noise tape set drifted")
    tape_arrays: dict[str, dict[str, Any]] = {}
    for filename in sorted(expected_names):
        row = tapes[filename]
        path = NOISE_MANIFEST.parent / filename
        expected_seed = None if filename.startswith("deterministic") else int(
            filename.split("seed_", 1)[1].split("_", 1)[0]
        )
        if (
            not isinstance(row, Mapping)
            or row.get("seed") != expected_seed
            or row.get("shape")
            != [contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE]
            or row.get("dtype") != contract.EXPECTED_DTYPE
            or not isinstance(row.get("array_sha256"), str)
            or row.get("artifact") != {
                "path": filename,
                "sha256": forensic.sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        ):
            raise QualificationExecutionError(f"noise tape record drifted: {filename}")
        with np.load(path, allow_pickle=False) as archive:
            expected_archive_keys = {"standard_normal"} | (
                set() if expected_seed is None else {"seed"}
            )
            if set(archive.files) != expected_archive_keys:
                raise QualificationExecutionError(f"noise archive schema drifted: {filename}")
            array = np.ascontiguousarray(archive["standard_normal"])
            if expected_seed is not None:
                seed_array = np.asarray(archive["seed"])
                if seed_array.dtype != np.int64 or seed_array.tolist() != [expected_seed]:
                    raise QualificationExecutionError(f"noise seed drifted: {filename}")
        if (
            array.shape != (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)
            or array.dtype != np.float32
            or not np.all(np.isfinite(array))
            or _array_sha256(array, np) != row["array_sha256"]
            or (expected_seed is None and np.count_nonzero(array) != 0)
        ):
            raise QualificationExecutionError(f"noise array drifted: {filename}")
        tape_arrays[filename] = {
            "path": path,
            "file_sha256": row["artifact"]["sha256"],
            "array_sha256": row["array_sha256"],
        }

    if not isinstance(cases, Sequence) or isinstance(cases, (str, bytes)):
        raise QualificationExecutionError("noise manifest cases are malformed")
    expected_cases = []
    case_tapes: dict[str, dict[str, Any]] = {}
    for case in contract.canonical_cases():
        filename = Path(str(case["noise_tape"])).name
        row = dict(case)
        row["noise_tape_array_sha256"] = tape_arrays[filename]["array_sha256"]
        expected_cases.append(row)
        case_tapes[str(case["case_id"])] = tape_arrays[filename]
    if list(cases) != expected_cases:
        raise QualificationExecutionError("noise case matrix drifted")
    return {"manifest": manifest, "case_tapes": case_tapes}


def _source_records() -> dict[str, Any]:
    return {
        "runner": _record(Path(__file__).resolve()),
        "comparator": _record(Path(gates.__file__).resolve()),
        "qualification_contract": _record(Path(contract.__file__).resolve()),
        "p1_contract": _record(Path(p1_contract.__file__).resolve()),
        "forensic_writer": _record(Path(forensic.__file__).resolve()),
    }


def _execution_claim_payload(token_sha256: str) -> dict[str, Any]:
    prerequisites = _validate_prerequisites()
    candidate_id = str(prerequisites["candidate_id"])
    noise = _validate_noise_manifest(candidate_id)
    roles = []
    for role in _ROLES:
        row = contract.role_contract(role)
        if role == contract.CANDIDATE_ROLE:
            row["actor_id"] = candidate_id
        roles.append(row)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "revision": contract.REVISION,
        "status": contract.UNLOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "execution_token_sha256": token_sha256,
        "execution_order": [
            {"index": index, "role": role, "case_id": case_id}
            for index, (role, case_id) in enumerate(MATRIX)
        ],
        "canonical_cases": [dict(case) for case in contract.canonical_cases()],
        "role_contracts": roles,
        "tolerances": contract.tolerance_rows(),
        "prerequisites": {
            "candidate_freeze": _record(CANDIDATE_FREEZE),
            "development_gate": _record(DEVELOPMENT_GATE),
            "noise_manifest": _record(NOISE_MANIFEST),
        },
        "inputs": {
            "candidate_module": _tree_record(prerequisites["candidate_module_path"]),
            "source_h0_module": _tree_record(prerequisites["source_h0_path"]),
            "source_h0_config": _record(SOURCE_H0_CONFIG),
            "analog_profile": _record(ANALOG_PROFILE),
            "v25_profile": _record(V25_PROFILE),
            "noise_tapes": {
                case_id: {
                    "artifact": _record(row["path"]),
                    "array_sha256": row["array_sha256"],
                }
                for case_id, row in noise["case_tapes"].items()
            },
        },
        "sources": _source_records(),
        "persist_before_gate": ["trace.json", "partial_summary.json", "summary.json"],
        "baseline_first": True,
        "supervisor_only_workers": True,
        "source_contract_authority": dict(contract.AUTHORITY),
        "execution_authority": {
            "explicit_execute_invocation": True,
            "same_candidate_freeze_and_development_pass_verified": True,
            "fresh_noise_manifest_verified": True,
            "source_contract_alone_authorizes_execution": False,
            "noise_manifest_alone_authorizes_execution": False,
            "one_terminal_matrix_authorized": True,
            "retry_or_resume_authorized": False,
            "actor_updates_authorized": False,
            "critic_updates_authorized": False,
            "ppo_updates_authorized": False,
            "protected_trial_access_authorized": False,
        },
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_execution_claim(token: str) -> dict[str, Any]:
    if os.path.lexists(EXECUTION_LEDGER):
        raise QualificationExecutionError("terminal qualification ledger already exists")
    observed = _mapping(EXECUTION_CLAIM)
    expected = _execution_claim_payload(_token_sha256(token))
    if _canonical(observed) != _canonical(expected):
        raise QualificationExecutionError("execution claim/token/input closure drifted")
    return observed


def _worker_claim_payload(
    *, role: str, case_id: str, token_sha256: str
) -> dict[str, Any]:
    index = _matrix_index(role, case_id)
    previous_receipts = [
        {
            "role": prior_role,
            "case_id": prior_case,
            "receipt": _record(_receipt_path(prior_role, prior_case)),
        }
        for prior_role, prior_case in MATRIX[:index]
    ]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": WORKER_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": _mapping(EXECUTION_CLAIM)["candidate_id"],
        "worker_index": index,
        "role": role,
        "case_id": case_id,
        "execution_token_sha256": token_sha256,
        "execution_claim": _record(EXECUTION_CLAIM),
        "previous_receipts": previous_receipts,
        "destination": contract.rollout_destination(role, case_id).as_posix(),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_worker_claim(role: str, case_id: str, token: str) -> dict[str, Any]:
    execution = verify_execution_claim(token)
    index = _matrix_index(role, case_id)
    observed = _mapping(_worker_claim_path(role, case_id))
    expected = _worker_claim_payload(
        role=role,
        case_id=case_id,
        token_sha256=str(execution["execution_token_sha256"]),
    )
    if _canonical(observed) != _canonical(expected):
        raise QualificationExecutionError(f"worker claim/order drifted: {role}/{case_id}")
    if os.path.lexists(_rollout_root(role, case_id)):
        raise QualificationExecutionError(f"rollout destination already consumed: {role}/{case_id}")
    for prior_role, prior_case in MATRIX[:index]:
        verify_rollout_receipt(prior_role, prior_case)
        if prior_role == contract.CANDIDATE_ROLE:
            pair = _mapping(_gate_path(prior_case))
            if (
                pair.get("status") != contract.CASE_PASS_STATUS
                or pair.get("passed") is not True
                or pair.get("candidate_id") != execution["candidate_id"]
            ):
                raise QualificationExecutionError(f"prior pair gate failed: {prior_case}")
    for later_role, later_case in MATRIX[index + 1 :]:
        if os.path.lexists(_worker_claim_path(later_role, later_case)) or os.path.lexists(
            _rollout_root(later_role, later_case)
        ):
            raise QualificationExecutionError("later qualification stage already exists")
    if role == contract.CANDIDATE_ROLE:
        for baseline_case in contract.CASE_IDS:
            verify_rollout_receipt(contract.BASELINE_ROLE, baseline_case)
    return observed


def _input_file(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise QualificationExecutionError(f"{label} record is malformed")
    path = _resolve_repository_path(str(record["path"]), label)
    if dict(record) != _record(path):
        raise QualificationExecutionError(f"{label} integrity mismatch")
    return path


def _load_noise_tape(
    execution: Mapping[str, Any], case_id: str, *, np: Any
) -> tuple[Any, str, str]:
    inputs = execution.get("inputs")
    if not isinstance(inputs, Mapping):
        raise QualificationExecutionError("execution inputs are malformed")
    tapes = inputs.get("noise_tapes")
    if not isinstance(tapes, Mapping) or set(tapes) != set(contract.CASE_IDS):
        raise QualificationExecutionError("execution noise closure drifted")
    record = tapes.get(case_id)
    if not isinstance(record, Mapping) or set(record) != {"artifact", "array_sha256"}:
        raise QualificationExecutionError(f"noise claim is malformed: {case_id}")
    path = _input_file(record["artifact"], f"noise tape {case_id}")
    with np.load(path, allow_pickle=False) as archive:
        if "standard_normal" not in archive.files:
            raise QualificationExecutionError("noise tape lacks standard_normal")
        array = np.ascontiguousarray(archive["standard_normal"])
    if (
        array.shape != (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)
        or array.dtype != np.float32
        or not np.all(np.isfinite(array))
        or _array_sha256(array, np) != record["array_sha256"]
    ):
        raise QualificationExecutionError(f"noise tape array drifted: {case_id}")
    case = _case(case_id)
    if case["action_selection"] == "deterministic" and np.count_nonzero(array):
        raise QualificationExecutionError("deterministic qualification tape is non-zero")
    return array, str(record["artifact"]["sha256"]), str(record["array_sha256"])


def _build_env_config(
    *, role: str, case: Mapping[str, Any], execution: Mapping[str, Any], legacy: Any
) -> dict[str, Any]:
    inputs = execution["inputs"]
    config = _input_file(inputs["source_h0_config"], "source H0 config")
    analog = _input_file(inputs["analog_profile"], "analog profile")
    v25 = _input_file(inputs["v25_profile"], "V25 profile")
    condition = {
        "id": case["case_id"],
        "action_selection": case["action_selection"],
        "offset_s": case["episode_start_offset_s"],
        "seed": case["runtime_seed"],
    }
    previous = (legacy.H0_CONFIG, legacy.ANALOG_PROFILE, legacy.V25_PROFILE)
    try:
        legacy.H0_CONFIG = config
        legacy.ANALOG_PROFILE = analog
        legacy.V25_PROFILE = v25
        result = legacy.build_env_config(
            case_id="A" if role == contract.BASELINE_ROLE else "C",
            condition=condition,
        )
    finally:
        legacy.H0_CONFIG, legacy.ANALOG_PROFILE, legacy.V25_PROFILE = previous
    expected_binary = (
        "disabled" if role == contract.BASELINE_ROLE else "binary_active"
    )
    expected_binary_event = (
        legacy.V25_SHADOW_CONTRACT
        if role == contract.BASELINE_ROLE
        else contract.CANDIDATE_EVENT_TARGET_ID
    )
    if (
        result.get("binary_phase_fsm_mode") != expected_binary
        or result.get("binary_phase_event_contract_id") != expected_binary_event
        or result.get("phase_fsm_input_mode") != "legacy_events"
        or result.get("event_contract_id") != "legacy_events_v1"
        or result.get("online_grf_applied_sides") != ["left"]
        or result.get("reward", {}).get("morphology_weight") != contract.MORPHOLOGY_WEIGHT
        or result.get("detector_sample_dt_s") != 0.001
        or result.get("segment_duration") != 0.010
        or result.get("episode_duration") != 5.0
    ):
        raise QualificationExecutionError(f"{role} environment routing drifted")
    return result


def _sea_fallback_count(payload: Any) -> int:
    if not isinstance(payload, Mapping) or not isinstance(payload.get("joints"), Mapping):
        raise QualificationExecutionError("SEA diagnostics are malformed")
    total = 0
    for joint in contract.JOINTS:
        row = payload["joints"].get(joint)
        if not isinstance(row, Mapping):
            raise QualificationExecutionError(f"SEA diagnostics missing {joint}")
        for field in (
            "tau_input_plugin_fallback_count",
            "motor_accel_plugin_fallback_count",
        ):
            value = row.get(field)
            if type(value) is not int or value < 0:
                raise QualificationExecutionError(f"SEA fallback malformed: {joint}.{field}")
            total += value
    return total


def _runtime_routing_ok(
    info: Mapping[str, Any], *, role: str, require_samples: bool
) -> bool:
    expected_mode = "disabled" if role == contract.BASELINE_ROLE else "binary_active"
    expected_executed = role == contract.CANDIDATE_ROLE
    expected_binary_event = (
        "binary_point_v25+functional_contact_fsm_v1_shadow"
        if role == contract.BASELINE_ROLE
        else contract.CANDIDATE_EVENT_TARGET_ID
    )
    samples = info.get("binary_phase_sensor_samples")
    sample_ok = (
        isinstance(samples, Sequence)
        and not isinstance(samples, (str, bytes, bytearray))
        and (len(samples) == 10 if require_samples else len(samples) == 0)
    )
    return bool(
        info.get("phase_fsm_input_mode") == "legacy_events"
        and info.get("event_contract_id") == "legacy_events_v1"
        and info.get("binary_phase_fsm_mode") == expected_mode
        and info.get("binary_phase_fsm_executed") is expected_executed
        and info.get("binary_phase_event_contract_id") == expected_binary_event
        and info.get("online_grf_applied_sides") == ["left"]
        and sample_ok
    )


def _solver_totals(v3: Any) -> dict[str, int]:
    return {key: 0 for key in v3.SO_RECOVERY_COUNTER_KEYS}


def _start_payload(
    *, role: str, case: Mapping[str, Any], execution: Mapping[str, Any]
) -> dict[str, Any]:
    role_spec = contract.role_contract(role)
    if role == contract.CANDIDATE_ROLE:
        role_spec["actor_id"] = execution["candidate_id"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V6_QUALIFICATION_ROLLOUT_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": execution["candidate_id"],
        "role": role,
        "case": dict(case),
        "role_contract": role_spec,
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(_worker_claim_path(role, str(case["case_id"]))),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _execute_rollout(
    *,
    role: str,
    case: Mapping[str, Any],
    execution: Mapping[str, Any],
    writer: forensic.ForensicRolloutWriter,
) -> dict[str, Any]:
    """Execute one claimed rollout; all heavyweight imports remain local."""

    import numpy as np

    import h0_v3_so_recovery_contract as so_recovery
    import primary_grf_split_adaptation as split_contract
    import run_h0_primary_grf_split_v1_adaptation as v1
    import run_h0_primary_grf_split_v3_semantic_replay as v3
    import run_h0_v25_abc_preflight as legacy

    if role == contract.CANDIDATE_ROLE:
        # Register the frozen residual RLModule class before checkpoint loading.
        import primary_split_v25_residual  # noqa: F401

    rollout_eval, stack_np, torch, RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    if stack_np is not np:
        raise QualificationExecutionError("inference NumPy identity drifted")
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    module_record = execution["inputs"][
        "source_h0_module" if role == contract.BASELINE_ROLE else "candidate_module"
    ]
    module_path = _tree_path(module_record, f"{role} module")
    module = RLModule.from_checkpoint(module_path)
    if hasattr(module, "eval"):
        module.eval()
    env_config = _build_env_config(
        role=role, case=case, execution=execution, legacy=legacy
    )
    env = env_factory.make_cmc_env(env_config)
    tape, tape_file_sha, tape_array_sha = _load_noise_tape(
        execution, str(case["case_id"]), np=np
    )
    action_shape = tuple(int(value) for value in env.action_space.shape)
    if action_shape != contract.EXPECTED_ACTION_SHAPE:
        raise QualificationExecutionError("action shape drifted")

    reserve = legacy._empty_accumulator()
    residual = legacy._empty_accumulator()
    sea = legacy._sea_accumulators()
    solver_totals = _solver_totals(v3)
    rows: list[dict[str, Any]] = []
    penetrations: list[float] = []
    binary_sample_count = 0
    clipping_values = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    nonfinite_count = 0
    sea_plugin_fallback_count = 0
    policy_step_terminal_fallback_count = 0
    runtime_contract_mismatch_count = 0
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        if not isinstance(reset_info, Mapping):
            raise QualificationExecutionError("reset info is malformed")
        base = env.unwrapped
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        full_names = tuple(str(name) for name in base.observation_feature_names)
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.float32
            or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
            or len(full_names) != contract.EXPECTED_FULL_FEATURES
            or not np.all(np.isfinite(observation))
        ):
            raise QualificationExecutionError("35/84 float32 reset contract drifted")
        if not _runtime_routing_ok(reset_info, role=role, require_samples=False):
            raise QualificationExecutionError(f"{role} reset routing drifted")
        shadow_fsm = (
            copy.deepcopy(base._phase_fsm)
            if role == contract.BASELINE_ROLE
            else None
        )
        body_weight_n = float(base._body_weight_n)
        current_info = dict(reset_info)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            if role == contract.BASELINE_ROLE:
                paired = split_contract.build_paired_views(
                    observation_before,
                    actor_names,
                    current_info,
                    body_weight_n=body_weight_n,
                    reset_boundary=index == 0,
                    teacher_phase_observation=shadow_fsm.observation(),
                )
                policy_input = v1._teacher_full_observation(
                    observation_before, paired, np
                )
                actor_observation = np.ascontiguousarray(
                    paired.teacher, dtype=np.float32
                )
            else:
                policy_input = observation_before.copy()
                actor_observation = policy_input[: contract.EXPECTED_ACTOR_FEATURES]
            _raw, mean, std = v1._policy(
                module, policy_input, action_shape, rollout_eval
            )
            mean = np.ascontiguousarray(mean, dtype=np.float32).reshape(action_shape)
            std = np.ascontiguousarray(std, dtype=np.float32).reshape(action_shape)
            if (
                not np.all(np.isfinite(mean))
                or not np.all(np.isfinite(std))
                or not np.allclose(
                    std, contract.STOCHASTIC_SIGMA, rtol=0.0, atol=1.0e-8
                )
            ):
                raise QualificationExecutionError("actor mean/logstd contract drifted")
            raw_action = (
                mean + std * tape[index]
                if case["action_selection"] == "stochastic"
                else mean.copy()
            )
            raw_action = np.ascontiguousarray(raw_action, dtype=np.float32)
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            clipping_values += int(np.count_nonzero(raw_action != applied))
            observation_after, reward, terminated, truncated, info = env.step(raw_action)
            observation = np.ascontiguousarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise QualificationExecutionError("environment info is malformed")
            if not _runtime_routing_ok(info, role=role, require_samples=True):
                runtime_contract_mismatch_count += 1
                raise QualificationExecutionError(f"{role} step routing drifted")
            samples = info["binary_phase_sensor_samples"]
            binary_sample_count += len(samples)
            reward_terms = info.get("reward_terms")
            phase = info.get("phase_fsm")
            if not isinstance(reward_terms, Mapping) or not isinstance(phase, Mapping):
                raise QualificationExecutionError("rollout diagnostics are incomplete")
            legacy._accumulate_scalar(reserve, reward_terms.get("reserve_norm_nm"))
            legacy._accumulate_scalar(residual, reward_terms.get("residual_norm_nm"))
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise QualificationExecutionError("penetration is malformed")
            penetrations.append(penetration)
            sea_payload = info.get("sea_segment_diagnostics")
            legacy._accumulate_sea(sea, sea_payload)
            sea_plugin_fallback_count += _sea_fallback_count(sea_payload)
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or type(so.get("solver_fallback_used")) is not bool:
                raise QualificationExecutionError("SO terminal diagnostic is missing")
            terminal_fallback = bool(so["solver_fallback_used"])
            policy_step_terminal_fallback_count += int(terminal_fallback)
            v3._validate_so_solver_audit_entries(
                info.get("so_solver_audit_entries"),
                step_index=step,
                selected_fallback=terminal_fallback,
            )
            recovery = so_recovery.classify_policy_step(
                info.get("so_solver_audit_entries"), policy_id=contract.SO_POLICY_ID
            )
            counters = recovery["counters"]
            for key in v3.SO_RECOVERY_COUNTER_KEYS:
                value = counters.get(key)
                if type(value) is not int or value < 0:
                    raise QualificationExecutionError(f"SO counter drifted: {key}")
                solver_totals[key] += value
            hard_invalid_count += int("failure" in info)
            finite = bool(
                np.all(np.isfinite(observation_before))
                and np.all(np.isfinite(observation))
                and np.all(np.isfinite(actor_observation))
                and np.all(np.isfinite(raw_action))
                and math.isfinite(float(reward))
                and math.isfinite(float(info.get("time")))
            )
            nonfinite_count += int(not finite)
            if not finite:
                raise QualificationExecutionError("non-finite rollout value")
            if role == contract.BASELINE_ROLE:
                v1._update_shadow_fsm(
                    shadow_fsm, info=info, body_weight_n=body_weight_n
                )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "candidate_id": execution["candidate_id"],
                "role": role,
                "case_id": case["case_id"],
                "time_s": float(info["time"]),
                "actor_input_view": (
                    "historical_analog"
                    if role == contract.BASELINE_ROLE
                    else "primary_split"
                ),
                "actor_observation": actor_observation.tolist(),
                "mean_action": mean.tolist(),
                "standard_normal": tape[index].tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "reward": float(reward),
                "grf_penetration_m": penetration,
                "phase_valid_cycle_count": int(
                    float(phase.get("valid_cycle_count", 0.0))
                ),
                "phase_invalid_event_count": int(
                    float(phase.get("invalid_event_count", 0.0))
                ),
                "so_recovery_counters": legacy._jsonable(counters),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            current_info = dict(info)
            if step == 1 or step % 25 == 0 or step == contract.EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V6 qualification/{role}/{case['case_id']}] "
                    f"{step:3d}/500 elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = legacy._finalize_sea(sea)
    aggregate_sea_fallback = sum(
        int(sea[joint]["fallback_count"]) for joint in contract.JOINTS
    )
    if aggregate_sea_fallback != sea_plugin_fallback_count:
        raise QualificationExecutionError("SEA fallback aggregation mismatch")
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    unaccepted_fallbacks = (
        solver_totals["unaccepted_hard_so_fallback_count"]
        + solver_totals["unaccepted_bounded_ls_count"]
        + sea_plugin_fallback_count
    )
    role_spec = contract.role_contract(role)
    actor_id = (
        contract.BASELINE_ACTOR_ID
        if role == contract.BASELINE_ROLE
        else execution["candidate_id"]
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": UNGATED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": execution["candidate_id"],
        "role": role,
        "actor_id": actor_id,
        "contract_id": role_spec["contract_id"],
        "actor_input_view": role_spec["actor_input_view"],
        "observation_semantics": role_spec["observation_semantics"],
        "primary_load_contract_id": role_spec["primary_load_contract_id"],
        "phase_fsm_input_mode": role_spec["phase_fsm_input_mode"],
        "event_contract_id": role_spec["event_contract_id"],
        "binary_phase_fsm_mode": role_spec["binary_phase_fsm_mode"],
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": float(case["episode_start_offset_s"]),
        "action_seed": case["action_seed"],
        "runtime_seed": runtime_seed,
        "sigma": float(case["sigma"]),
        "noise_tape_file_sha256": tape_file_sha,
        "noise_tape_array_sha256": tape_array_sha,
        "steps": len(rows),
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0.0))),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "binary_phase_sensor_sample_count": binary_sample_count,
        "action_clipped_values": clipping_values,
        "fallback_count": unaccepted_fallbacks,
        "raw_so_fallback_count": solver_totals["bounded_ls_invocation_count"],
        "policy_step_terminal_so_fallback_count": policy_step_terminal_fallback_count,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "invalid_event_count": invalid_event_count,
        "nonfinite_count": nonfinite_count,
        "runtime_contract_mismatch_count": runtime_contract_mismatch_count,
        "so_policy_id": contract.SO_POLICY_ID,
        "so_solver_control_window_count": solver_totals["control_window_count"],
        "so_solver_bounded_ls_invocation_count": solver_totals["bounded_ls_invocation_count"],
        "so_solver_verified_bounded_ls_count": solver_totals["verified_bounded_ls_count"],
        "so_solver_verified_status0_max_iter_count": solver_totals[
            "verified_status0_max_iter_count"
        ],
        "so_solver_unaccepted_hard_fallback_count": solver_totals[
            "unaccepted_hard_so_fallback_count"
        ],
        "so_solver_unaccepted_bounded_ls_count": solver_totals[
            "unaccepted_bounded_ls_count"
        ],
        "so_solver_hard_fallback_count": solver_totals["hard_so_fallback_count"],
        "so_solver_reuse_previous_count": solver_totals["reuse_previous_count"],
        "so_solver_bounded_ls_unsuccessful_count": solver_totals[
            "bounded_ls_unsuccessful_count"
        ],
        "so_solver_bounds_violation_count": solver_totals["bounds_violation_count"],
        "so_solver_nonfinite_count": solver_totals["nonfinite_solver_count"],
        "so_solver_selected_infeasible_count": solver_totals[
            "selected_infeasible_count"
        ],
        "so_solver_selected_solution_mismatch_count": solver_totals[
            "selected_solution_mismatch_count"
        ],
        "so_solver_residual_contract_mismatch_count": solver_totals[
            "residual_contract_mismatch_count"
        ],
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": contract.EXPECTED_DTYPE,
        "action_shape": list(action_shape),
        "action_dtype": contract.EXPECTED_DTYPE,
        "online_grf_applied_sides": ["left"],
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "episode_metrics": {
            "reserve_norm_nm": legacy._finalize_accumulator(reserve),
            "residual_norm_nm": legacy._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PERSISTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": execution["candidate_id"],
        "role": role,
        "case_id": case["case_id"],
        "steps": len(rows),
        "end_reason": summary["end_reason"],
        "gate_evaluated": False,
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(records: dict[str, Any]) -> dict[str, Any]:
        gate = gates.common_rollout_gate(
            summary,
            role=role,
            case_id=str(case["case_id"]),
            candidate_id=str(execution["candidate_id"]),
        )
        gate["persisted_before_gate"] = records
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": execution["candidate_id"],
        "role": role,
        "case_id": case["case_id"],
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(_worker_claim_path(role, str(case["case_id"]))),
        "persisted_before_gate": persisted,
        "artifacts": writer.artifact_records(),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(writer.run_directory / "receipt.json", receipt)
    if gate.get("passed") is not True:
        raise QualificationExecutionError(f"common rollout gate failed: {role}/{case['case_id']}")
    return {**receipt, "receipt": _record(receipt_path)}


def run_worker(*, role: str, case_id: str, supervisor_token: str) -> dict[str, Any]:
    """Execute one supervisor-claimed rollout and never infer authority."""

    verify_worker_claim(role, case_id, supervisor_token)
    execution = _mapping(EXECUTION_CLAIM)
    case = _case(case_id)
    writer = forensic.ForensicRolloutWriter(
        _rollout_root(role, case_id), artifact_root=REPO_ROOT
    )
    try:
        writer.start(_start_payload(role=role, case=case, execution=execution))
        return _execute_rollout(
            role=role, case=case, execution=execution, writer=writer
        )
    except Exception as exc:
        if writer.run_start_path.is_file() and not writer.failure_path.exists():
            try:
                writer.publish_failure(
                    end_reason="qualification_worker_failed_terminal_no_retry",
                    error=exc,
                    status=contract.FAIL_STATUS,
                    details={
                        "protocol_id": contract.PROTOCOL_ID,
                        "candidate_id": execution["candidate_id"],
                        "role": role,
                        "case_id": case_id,
                        "retry_authorized": False,
                        "actor_updates": 0,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                    },
                )
            except Exception:
                pass
        raise


def _verify_claim_content() -> dict[str, Any]:
    observed = _mapping(EXECUTION_CLAIM)
    token_hash = observed.get("execution_token_sha256")
    if (
        not isinstance(token_hash, str)
        or len(token_hash) != 64
        or any(character not in "0123456789abcdef" for character in token_hash)
    ):
        raise QualificationExecutionError("execution claim token hash is malformed")
    expected = _execution_claim_payload(token_hash)
    if _canonical(observed) != _canonical(expected):
        raise QualificationExecutionError("execution claim content drifted")
    return observed


def verify_rollout_receipt(role: str, case_id: str) -> dict[str, Any]:
    """Verify a completed rollout, its journal, and its pure common gate."""

    execution = _verify_claim_content()
    receipt_path = _receipt_path(role, case_id)
    receipt = _mapping(receipt_path)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "candidate_id",
        "role",
        "case_id",
        "execution_claim",
        "worker_claim",
        "persisted_before_gate",
        "artifacts",
        "retry_authorized",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if (
        set(receipt) != expected_keys
        or receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.ROLLOUT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("candidate_id") != execution["candidate_id"]
        or receipt.get("role") != role
        or receipt.get("case_id") != case_id
        or receipt.get("execution_claim") != _record(EXECUTION_CLAIM)
        or receipt.get("worker_claim") != _record(_worker_claim_path(role, case_id))
        or receipt.get("retry_authorized") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError(f"rollout receipt drifted: {role}/{case_id}")
    writer = forensic.ForensicRolloutWriter(
        _rollout_root(role, case_id), artifact_root=REPO_ROOT
    )
    if writer.failure_path.exists() or writer.last_completed_step != contract.EXPECTED_STEPS:
        raise QualificationExecutionError(f"rollout journal is incomplete: {role}/{case_id}")
    persisted = writer.finalized_artifact_records()
    if receipt.get("persisted_before_gate") != persisted:
        raise QualificationExecutionError("persist-before-gate records drifted")
    summary = _mapping(writer.summary_path)
    expected_gate = gates.common_rollout_gate(
        summary,
        role=role,
        case_id=case_id,
        candidate_id=str(execution["candidate_id"]),
    )
    expected_gate["persisted_before_gate"] = persisted
    observed_gate = _mapping(writer.gate_path)
    if _canonical(observed_gate) != _canonical(expected_gate):
        raise QualificationExecutionError(f"common gate drifted: {role}/{case_id}")
    if receipt.get("artifacts") != writer.artifact_records():
        raise QualificationExecutionError(f"rollout artifacts drifted: {role}/{case_id}")
    return receipt


def _publish_worker_claim(role: str, case_id: str, token_hash: str) -> Path:
    index = _matrix_index(role, case_id)
    if os.path.lexists(_rollout_root(role, case_id)):
        raise QualificationExecutionError(f"rollout destination occupied: {role}/{case_id}")
    for prior_role, prior_case in MATRIX[:index]:
        verify_rollout_receipt(prior_role, prior_case)
        if prior_role == contract.CANDIDATE_ROLE:
            gate = _mapping(_gate_path(prior_case))
            if gate.get("passed") is not True:
                raise QualificationExecutionError(f"prior pair failed: {prior_case}")
    path = _worker_claim_path(role, case_id)
    return forensic.write_json_exclusive(
        path,
        _worker_claim_payload(
            role=role,
            case_id=case_id,
            token_sha256=token_hash,
        ),
    )


def _worker_command(role: str, case_id: str, token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--role",
        role,
        "--case",
        case_id,
        "--supervisor-token",
        token,
    ]


def _run_worker_process(role: str, case_id: str, token: str) -> None:
    completed = subprocess.run(
        _worker_command(role, case_id, token),
        cwd=REPO_ROOT,
        timeout=WORKER_TIMEOUT_S,
        check=False,
    )
    if completed.returncode != 0:
        raise QualificationExecutionError(
            f"worker {role}/{case_id} exited {completed.returncode}"
        )


def _publish_pair_gate(case_id: str, candidate_id: str) -> dict[str, Any]:
    baseline_receipt = verify_rollout_receipt(contract.BASELINE_ROLE, case_id)
    candidate_receipt = verify_rollout_receipt(contract.CANDIDATE_ROLE, case_id)
    baseline_path = _rollout_root(contract.BASELINE_ROLE, case_id) / "summary.json"
    candidate_path = _rollout_root(contract.CANDIDATE_ROLE, case_id) / "summary.json"
    gate = gates.condition_matched_gate(
        _mapping(baseline_path),
        _mapping(candidate_path),
        case_id=case_id,
        candidate_id=candidate_id,
    )
    gate["provenance"] = {
        "baseline_receipt": _record(_receipt_path(contract.BASELINE_ROLE, case_id)),
        "candidate_receipt": _record(_receipt_path(contract.CANDIDATE_ROLE, case_id)),
        "baseline_summary": _record(baseline_path),
        "candidate_summary": _record(candidate_path),
        "baseline_status": baseline_receipt["status"],
        "candidate_status": candidate_receipt["status"],
    }
    forensic.write_json_exclusive(_gate_path(case_id), gate)
    if gate.get("passed") is not True:
        raise QualificationExecutionError(f"condition-matched gate failed: {case_id}")
    return gate


def _completed_count(role: str) -> int:
    return sum(int(_receipt_path(role, case_id).is_file()) for case_id in contract.CASE_IDS)


def _terminal_payload(
    *,
    started_unix_s: float,
    passed: bool,
    error: str | None,
) -> dict[str, Any]:
    execution = _mapping(EXECUTION_CLAIM)
    gate_records = {
        case_id: _record(_gate_path(case_id))
        for case_id in contract.CASE_IDS
        if _gate_path(case_id).is_file()
    }
    rollout_records: list[dict[str, Any]] = []
    for role, case_id in MATRIX:
        path = _receipt_path(role, case_id)
        if path.is_file():
            rollout_records.append(
                {"role": role, "case_id": case_id, "receipt": _record(path)}
            )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "revision": contract.REVISION,
        "status": contract.PASS_STATUS if passed else contract.FAIL_STATUS,
        "passed": passed,
        "error": error,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": execution["candidate_id"],
        "started_unix_s": started_unix_s,
        "completed_unix_s": time.time(),
        "execution_claim": _record(EXECUTION_CLAIM),
        "rollout_receipts": rollout_records,
        "case_gates": gate_records,
        "baseline_rollouts_completed": _completed_count(contract.BASELINE_ROLE),
        "candidate_rollouts_completed": _completed_count(contract.CANDIDATE_ROLE),
        "terminal": True,
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "TRAINER_ZERO_UPDATE_PORT"
            if passed
            else "STOP_TERMINAL_WITHOUT_RETRY_RESCUE_OR_RETUNING"
        ),
    }


def execute() -> dict[str, Any]:
    """Claim and execute the exact fresh matrix once, or stop terminally."""

    if os.path.lexists(RUN_ROOT):
        raise QualificationExecutionError(
            f"qualification root already claimed/no retry: {RUN_ROOT}"
        )
    token = secrets.token_urlsafe(32)
    token_hash = _token_sha256(token)
    # Validate every prerequisite and decoded tape before consuming the fresh
    # qualification destination.
    claim = _execution_claim_payload(token_hash)
    try:
        RUN_ROOT.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise QualificationExecutionError("qualification root was claimed concurrently") from exc
    forensic.write_json_exclusive(EXECUTION_CLAIM, claim)
    started = time.time()
    passed = False
    error: str | None = None
    try:
        for role, case_id in MATRIX:
            _publish_worker_claim(role, case_id, token_hash)
            _run_worker_process(role, case_id, token)
            verify_rollout_receipt(role, case_id)
            if role == contract.CANDIDATE_ROLE:
                _publish_pair_gate(case_id, str(claim["candidate_id"]))
        for case_id in contract.CASE_IDS:
            gate = _mapping(_gate_path(case_id))
            if (
                gate.get("status") != contract.CASE_PASS_STATUS
                or gate.get("passed") is not True
                or gate.get("candidate_id") != claim["candidate_id"]
            ):
                raise QualificationExecutionError(f"terminal pair gate drifted: {case_id}")
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = _terminal_payload(started_unix_s=started, passed=passed, error=error)
    forensic.write_json_exclusive(EXECUTION_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise QualificationExecutionError(error or contract.FAIL_STATUS)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--role", choices=_ROLES)
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--supervisor-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            if any(
                value is not None
                for value in (args.role, args.case, args.supervisor_token)
            ):
                raise QualificationExecutionError(
                    "supervisor mode does not accept worker arguments"
                )
            result = execute()
        else:
            if args.role is None or args.case is None or args.supervisor_token is None:
                raise QualificationExecutionError(
                    "worker role/case/supervisor-token are required"
                )
            result = run_worker(
                role=args.role,
                case_id=args.case,
                supervisor_token=args.supervisor_token,
            )
    except Exception as exc:
        print(
            "H0 V6 qualification failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
