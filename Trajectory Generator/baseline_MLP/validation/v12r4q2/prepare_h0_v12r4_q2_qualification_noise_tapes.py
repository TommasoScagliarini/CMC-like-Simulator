"""Prepare the five Q2 tapes only after exact terminal-PASS V12R4 binding.

Import, candidate validation, and ``build_tapes`` never write.  ``prepare`` is
explicit and no-clobber; the canonical root is not materialized by this source
implementation task.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

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
ROOT_VALIDATION = REPO_ROOT / "validation"
Q2_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, Q2_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r4_q2_qualification_design as design_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r4_q2_qualification_gates as gates  # noqa: E402
import h0_v12r4_q2_runtime_contract as contract  # noqa: E402


class V12R4Q2QualificationNoiseError(RuntimeError):
    """Raised before any unsafe or noncanonical Q2 tape operation."""


DESIGN_FREEZE_RECORD = {
    "path": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
    "sha256": "d92fac765bd192f43ef2e0420a8529e8bb21860a3f47420ab5948863fb53eaf5",
    "size_bytes": 32_166,
}
NOISE_TAPES_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_NOISE_TAPES"
NOISE_TAPES_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_NOISE_TAPES"
TAPE_SHAPE = (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)


def _raw_relative(value: str | os.PathLike[str] | PurePosixPath) -> str:
    return value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = _raw_relative(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R4Q2QualificationNoiseError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attributes = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attributes & reparse)


def _assert_no_link_components(path: Path) -> None:
    cursor = path.absolute()
    while True:
        if os.path.lexists(cursor) and _is_link_or_reparse(cursor):
            raise V12R4Q2QualificationNoiseError(
                f"path contains link/reparse component: {cursor}"
            )
        if cursor == cursor.parent:
            break
        cursor = cursor.parent


def _portable_path(path: Path) -> str:
    absolute = path.absolute()
    try:
        return absolute.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(absolute)


def _record(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R4Q2QualificationNoiseError(
            f"artifact is not a safe regular file: {source}"
        )
    return {
        "path": _portable_path(source),
        "sha256": forensic.sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def _mapping(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    try:
        payload = forensic.strict_json_load(source)
    except Exception as exc:
        raise V12R4Q2QualificationNoiseError(
            f"cannot read strict canonical JSON: {source}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R4Q2QualificationNoiseError(f"expected JSON object: {source}")
    result = dict(payload)
    if source.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R4Q2QualificationNoiseError(
            f"JSON artifact is not byte-canonical: {source}"
        )
    return result


def array_sha256(array: Any) -> str:
    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def build_tapes() -> dict[str, dict[str, Any]]:
    """Regenerate the five preregistered arrays in memory."""

    tapes: dict[str, dict[str, Any]] = {
        "deterministic_all_zero.npz": {
            "seed": None,
            "standard_normal": np.zeros(TAPE_SHAPE, dtype=np.float32),
        }
    }
    for seed in contract.STOCHASTIC_SEEDS:
        tapes[f"stochastic_seed_{seed}_standard_normal.npz"] = {
            "seed": seed,
            "standard_normal": np.random.default_rng(seed)
            .standard_normal(TAPE_SHAPE)
            .astype(np.float32),
        }
    observed = {
        name: array_sha256(definition["standard_normal"])
        for name, definition in tapes.items()
    }
    if observed != design_freezer.contract.EXPECTED_TAPE_ARRAY_SHA256:
        raise V12R4Q2QualificationNoiseError("reference tape ABI/hash drifted")
    if not all(
        definition["standard_normal"].shape == TAPE_SHAPE
        and definition["standard_normal"].dtype == np.float32
        and definition["standard_normal"].flags.c_contiguous
        and np.all(np.isfinite(definition["standard_normal"]))
        for definition in tapes.values()
    ):
        raise V12R4Q2QualificationNoiseError("reference tape array contract drifted")
    return tapes


def _verified_design_binding() -> dict[str, Any]:
    path = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if _record(path) != DESIGN_FREEZE_RECORD:
        raise V12R4Q2QualificationNoiseError("Q2 design freeze hash/size drifted")
    try:
        payload = design_freezer.verify_design_freeze()
    except Exception as exc:
        raise V12R4Q2QualificationNoiseError(
            "Q2 design freeze verification failed"
        ) from exc
    snapshot = payload.get("design_snapshot")
    if (
        payload.get("status")
        != design_freezer.contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("candidate_binding_state") != "DEFERRED"
        or payload.get("candidate_id") is not None
        or not isinstance(snapshot, Mapping)
        or snapshot.get("holdout_cases") != list(contract.HOLDOUT_CASES)
        or snapshot.get("rollout_matrix") != list(contract.ROLLOUT_MATRIX)
    ):
        raise V12R4Q2QualificationNoiseError("Q2 design semantics drifted")
    return {"record": copy.deepcopy(DESIGN_FREEZE_RECORD), "payload": payload}


def _artifact_record_valid(value: Any) -> bool:
    return bool(
        isinstance(value, Mapping)
        and set(value) == {"path", "sha256", "size_bytes"}
        and isinstance(value.get("path"), str)
        and value["path"]
        and isinstance(value.get("sha256"), str)
        and len(value["sha256"]) == 64
        and all(character in "0123456789abcdef" for character in value["sha256"])
        and type(value.get("size_bytes")) is int
        and value["size_bytes"] > 0
    )


def validate_prerequisite_payloads(
    payloads: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Any]],
    *,
    enforce_canonical_record_paths: bool = True,
) -> dict[str, Any]:
    """Resolve one exact candidate from the five ordered R4 artifacts."""

    requirements = contract.prerequisite_requirements()
    names = [row["name"] for row in requirements]
    checks: dict[str, bool] = {
        "exact_five_names": list(payloads) == names and list(records) == names,
    }
    if not checks["exact_five_names"]:
        return {
            "passed": False,
            "checks": checks,
            "candidate_id": None,
            "candidate_module": None,
            "rows": [],
        }

    protocol, lock, candidate, final, ledger = (payloads[name] for name in names)
    candidate_id = candidate.get("candidate_id")
    candidate_module = candidate.get("candidate_module")
    try:
        binding = contract.validate_candidate_binding(candidate_id, candidate_module)
    except (TypeError, ValueError):
        binding = None

    rows: list[dict[str, Any]] = []
    for index, (requirement, name) in enumerate(zip(requirements, names, strict=True)):
        payload = payloads[name]
        record = records[name]
        record_ok = _artifact_record_valid(record) and (
            not enforce_canonical_record_paths
            or record.get("path") == requirement["path"]
        )
        checks[f"{name}_status_record"] = bool(
            payload.get("status") == requirement["required_status"]
            and payload.get("passed") is True
            and record_ok
        )
        rows.append(
            {
                "name": name,
                "status": payload.get("status"),
                "passed": payload.get("passed"),
                "candidate_selection_rule": (
                    payload.get("candidate_selection", {}).get("rule")
                    if index < 2
                    else None
                ),
                "candidate_module_path": (
                    payload.get("candidate_selection", {}).get("module_path")
                    if index < 2
                    else None
                ),
                "candidate_id": None if index < 2 else payload.get("candidate_id"),
                "candidate_module_tree_sha256": (
                    None
                    if index < 2
                    or not isinstance(payload.get("candidate_module"), Mapping)
                    else payload["candidate_module"].get("tree_sha256")
                ),
                "artifact": copy.deepcopy(record),
            }
        )

    selection = {
        "rule": contract.R4_CANDIDATE_SELECTION_RULE,
        "module_path": contract.R4_CANDIDATE_MODULE_PATH.as_posix(),
        "candidate_id": "DEFERRED_UNTIL_FIT_P3",
        "candidate_tree_sha256": "DEFERRED_UNTIL_FIT_P3",
    }
    design = copy.deepcopy(DESIGN_FREEZE_RECORD)
    checks.update(
        {
            "pre_fit_selection_rule_exact": protocol.get("candidate_selection")
            == selection
            and lock.get("candidate_selection") == selection,
            "protocol_and_lock_bind_design": protocol.get("q2_design_freeze") == design
            and lock.get("q2_design_freeze") == design,
            "lock_binds_protocol": lock.get("protocol_freeze") == records[names[0]],
            "terminal_candidate_exact": binding is not None
            and final.get("candidate_id") == candidate_id
            and final.get("candidate_module") == candidate_module
            and ledger.get("candidate_id") == candidate_id
            and ledger.get("candidate_module") == candidate_module,
            "candidate_and_final_chain": final.get("candidate_freeze")
            == records[names[2]]
            and ledger.get("candidate_freeze") == records[names[2]]
            and ledger.get("final_development_receipt") == records[names[3]],
            "terminal_chain": ledger.get("terminal") is True
            and ledger.get("protocol_freeze") == records[names[0]]
            and ledger.get("execution_lock") == records[names[1]]
            and ledger.get("q2_paths_opened") == [],
            "no_retry_or_promotion": final.get("retry_authorized") is False
            and final.get("resume_authorized") is False
            and final.get("runtime_promoted") is False
            and ledger.get("runtime_promoted") is False,
        }
    )
    normalized_rows = []
    for requirement, row in zip(requirements, rows, strict=True):
        normalized = copy.deepcopy(row)
        normalized["artifact"]["path"] = requirement["path"]
        normalized_rows.append(normalized)
    frozen = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREREQUISITE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": copy.deepcopy(candidate_module),
        "prerequisites": normalized_rows,
        "r4_terminal_passed": True,
        "candidate_identity_unique_count": 1,
        "q2_protocol_preexisting": False,
        "q2_execution_lock_preexisting": False,
        "q2_noise_root_preexisting": False,
        "q2_run_root_preexisting": False,
        "qualification_rollouts_opened": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }
    pure_gate = gates.r4_prerequisite_gate(frozen)
    checks["pure_prerequisite_gate_pass"] = pure_gate.get("passed") is True
    passed = all(checks.values())
    if passed and binding is not None:
        contract.bind_candidate(binding["candidate_id"], binding["candidate_module"])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PREREQUISITE_PASS_STATUS
            if passed
            else contract.PREREQUISITE_FAIL_STATUS
        ),
        "passed": passed,
        "candidate_id": candidate_id if passed else None,
        "candidate_module": copy.deepcopy(candidate_module) if passed else None,
        "checks": checks,
        "rows": rows,
        "frozen_gate": pure_gate,
    }


def _canonical_prerequisite_paths() -> dict[str, Path]:
    return {
        row["name"]: resolve_relative(row["path"])
        for row in contract.prerequisite_requirements()
    }


def load_and_validate_prerequisites(
    *,
    prerequisite_paths: Mapping[str, Path] | None = None,
    require_qualification_unopened: bool = True,
) -> dict[str, Any]:
    if require_qualification_unopened:
        occupied = [
            path
            for path in (
                resolve_relative(contract.PROTOCOL_FREEZE_PATH),
                resolve_relative(contract.EXECUTION_LOCK_PATH),
                resolve_relative(contract.NOISE_ROOT),
                resolve_relative(contract.RUN_ROOT),
            )
            if os.path.lexists(path)
        ]
        if occupied:
            raise V12R4Q2QualificationNoiseError(
                f"Q2 must be unopened before R4 binding: {occupied}"
            )
    paths = dict(
        _canonical_prerequisite_paths()
        if prerequisite_paths is None
        else prerequisite_paths
    )
    names = [row["name"] for row in contract.prerequisite_requirements()]
    if (
        list(paths) != names
        or len({Path(path).absolute() for path in paths.values()}) != 5
    ):
        raise V12R4Q2QualificationNoiseError(
            "R4 prerequisites must be five distinct ordered artifacts"
        )
    payloads = {name: _mapping(paths[name]) for name in names}
    records = {name: _record(paths[name]) for name in names}
    gate = validate_prerequisite_payloads(
        payloads,
        records,
        enforce_canonical_record_paths=prerequisite_paths is None,
    )
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if value is not True]
        raise V12R4Q2QualificationNoiseError(
            f"R4 terminal prerequisites are not eligible: {failed}"
        )
    return {"payloads": payloads, "records": records, "gate": gate}


def _write_npz_exclusive(path: Path, *, array: Any, seed: int | None) -> None:
    _assert_no_link_components(path)
    if os.path.lexists(path):
        raise V12R4Q2QualificationNoiseError(f"refusing to clobber: {path}")
    with path.open("xb") as stream:
        arrays: dict[str, Any] = {"standard_normal": array}
        if seed is not None:
            arrays["seed"] = np.asarray([seed], dtype=np.int64)
        np.savez(stream, **arrays)
        stream.flush()
        os.fsync(stream.fileno())


def _read_tape(path: Path, *, seed: int | None) -> Any:
    _assert_no_link_components(path)
    try:
        with np.load(path, allow_pickle=False) as archive:
            expected = (
                {"standard_normal"} if seed is None else {"standard_normal", "seed"}
            )
            if set(archive.files) != expected:
                raise V12R4Q2QualificationNoiseError(f"tape keys drifted: {path}")
            if seed is not None and archive["seed"].tolist() != [seed]:
                raise V12R4Q2QualificationNoiseError(f"tape seed drifted: {path}")
            return np.ascontiguousarray(archive["standard_normal"], dtype=np.float32)
    except V12R4Q2QualificationNoiseError:
        raise
    except Exception as exc:
        raise V12R4Q2QualificationNoiseError(f"cannot read tape: {path}") from exc


def _manifest_payload(
    *, root: Path, prerequisites: Mapping[str, Any]
) -> dict[str, Any]:
    tapes = build_tapes()
    tape_rows: dict[str, Any] = {}
    for filename, definition in tapes.items():
        path = root / filename
        tape_rows[filename] = {
            "seed": definition["seed"],
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "array_sha256": array_sha256(definition["standard_normal"]),
            "artifact": _record(path),
        }
    binding = prerequisites["gate"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": NOISE_TAPES_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "candidate_module_tree_sha256": binding["candidate_module"]["tree_sha256"],
        "qualification_design_freeze": copy.deepcopy(DESIGN_FREEZE_RECORD),
        "r4_prerequisite_gate": copy.deepcopy(binding),
        "r4_prerequisite_records": copy.deepcopy(prerequisites["records"]),
        "root": _portable_path(root),
        "array_key": "standard_normal",
        "shape": list(TAPE_SHAPE),
        "dtype": "float32",
        "c_contiguous": True,
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "hash_scheme": "sha256(dtype_ascii+compact_json_shape+contiguous_c_bytes)",
        "tapes": tape_rows,
        "case_order": list(contract.CASE_IDS),
        "cases": [
            {
                **contract.canonical_case(case_id),
                "noise_tape_array_sha256": tape_rows[
                    PurePosixPath(contract.canonical_case(case_id)["noise_tape"]).name
                ]["array_sha256"],
            }
            for case_id in contract.CASE_IDS
        ],
        "same_tape_required_for_condition_pair": True,
        "qualification_rollouts_opened": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def prepare(
    *,
    output_root: str | os.PathLike[str] | None = None,
    prerequisite_paths: Mapping[str, Path] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Materialize an explicitly requested root, atomically per file."""

    root = (
        resolve_relative(contract.NOISE_ROOT)
        if output_root is None
        else Path(output_root).absolute()
    )
    canonical = resolve_relative(contract.NOISE_ROOT)
    if enforce_canonical_destination and root != canonical:
        raise V12R4Q2QualificationNoiseError(f"non-canonical noise destination: {root}")
    _assert_no_link_components(root)
    if os.path.lexists(root):
        raise V12R4Q2QualificationNoiseError(f"refusing to clobber: {root}")
    _verified_design_binding()
    prerequisites = load_and_validate_prerequisites(
        prerequisite_paths=prerequisite_paths,
        require_qualification_unopened=output_root is None,
    )
    definitions = build_tapes()
    root.mkdir(parents=True, exist_ok=False)
    for filename, definition in definitions.items():
        _write_npz_exclusive(
            root / filename,
            array=definition["standard_normal"],
            seed=definition["seed"],
        )
    payload = _manifest_payload(root=root, prerequisites=prerequisites)
    forensic.write_json_exclusive(root / "manifest.json", payload)
    return verify_manifest(
        noise_root=root,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def verify_manifest(
    *,
    noise_root: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    root = (
        resolve_relative(contract.NOISE_ROOT)
        if noise_root is None
        else Path(noise_root).absolute()
    )
    canonical = resolve_relative(contract.NOISE_ROOT)
    if enforce_canonical_destination and root != canonical:
        raise V12R4Q2QualificationNoiseError(f"non-canonical noise root: {root}")
    payload = _mapping(root / "manifest.json")
    expected_files = {
        "manifest.json",
        *design_freezer.contract.EXPECTED_TAPE_ARRAY_SHA256,
    }
    if not root.is_dir() or {path.name for path in root.iterdir()} != expected_files:
        raise V12R4Q2QualificationNoiseError("noise root file set drifted")
    if (
        payload.get("status") != NOISE_TAPES_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("qualification_design_freeze") != DESIGN_FREEZE_RECORD
        or payload.get("case_order") != list(contract.CASE_IDS)
    ):
        raise V12R4Q2QualificationNoiseError("noise manifest semantics drifted")
    for (
        filename,
        expected_hash,
    ) in design_freezer.contract.EXPECTED_TAPE_ARRAY_SHA256.items():
        row = payload.get("tapes", {}).get(filename)
        if not isinstance(row, Mapping):
            raise V12R4Q2QualificationNoiseError(f"missing tape row: {filename}")
        array = _read_tape(root / filename, seed=row.get("seed"))
        if (
            array.shape != TAPE_SHAPE
            or array.dtype != np.float32
            or not array.flags.c_contiguous
            or array_sha256(array) != expected_hash
            or row.get("array_sha256") != expected_hash
            or row.get("artifact") != _record(root / filename)
        ):
            raise V12R4Q2QualificationNoiseError(f"tape closure drifted: {filename}")
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    action.add_argument("--build-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare:
        payload = prepare()
    elif args.verify:
        payload = verify_manifest()
    else:
        payload = {
            "status": "PASS_H0_V12R4_Q2_REFERENCE_TAPES_BUILD_ONLY",
            "passed": len(build_tapes()) == 5,
        }
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "DESIGN_FREEZE_RECORD",
    "NOISE_TAPES_PASS_STATUS",
    "V12R4Q2QualificationNoiseError",
    "array_sha256",
    "build_tapes",
    "load_and_validate_prerequisites",
    "prepare",
    "validate_prerequisite_payloads",
    "verify_manifest",
]
