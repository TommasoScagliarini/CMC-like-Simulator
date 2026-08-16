"""Materialize the frozen V12R3-P1 qualification innovations once.

Importing this module performs no filesystem write, random draw, environment
construction, or model loading.  Materialization is fail-closed until the
canonical qualification design and all four V12P1S terminal prerequisites are
present, PASS, content-bound, and identify the exact same P1 candidate.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
import tempfile
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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12P1Q_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r3_p1_qualification_design as design_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import h0_v12r3_p1_qualification_gates as gates  # noqa: E402


class V12R3P1QualificationNoiseError(RuntimeError):
    """Raised before any unsafe or non-preregistered tape operation."""


NOISE_TAPES_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_NOISE_TAPES"
TAPE_SHAPE = (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)
DEFAULT_OUTPUT_ROOT = REPO_ROOT.joinpath(*contract.NOISE_ROOT.parts)
DESIGN_FREEZE_RECORD = {
    "path": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
    "sha256": "c64928cb85df0e1f5d53c5f2e6eba52172b7c831ccf712a0113522bcff4ef686",
    "size_bytes": 21_303,
}


def _raw_relative(value: str | os.PathLike[str] | PurePosixPath) -> str:
    return value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve a normalized repository-relative path without following aliases."""

    raw = _raw_relative(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R3P1QualificationNoiseError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    result = REPO_ROOT.joinpath(*pure.parts)
    try:
        result.relative_to(REPO_ROOT)
    except ValueError as exc:  # pragma: no cover - defensive on exotic hosts.
        raise V12R3P1QualificationNoiseError(
            f"repository path escaped: {raw!r}"
        ) from exc
    return result


def _is_link_or_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attribute = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attribute & reparse)


def _assert_no_link_components(path: Path) -> None:
    absolute = path.absolute()
    existing: list[Path] = []
    cursor = absolute
    while True:
        if os.path.lexists(cursor):
            existing.append(cursor)
        if cursor == cursor.parent:
            break
        cursor = cursor.parent
    for component in reversed(existing):
        if _is_link_or_reparse(component):
            raise V12R3P1QualificationNoiseError(
                f"path contains link/reparse component: {component}"
            )


def _portable_path(path: Path) -> str:
    source = path.absolute()
    try:
        return source.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(source)


def _record(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R3P1QualificationNoiseError(
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
        raise V12R3P1QualificationNoiseError(
            f"cannot read strict JSON: {source}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R3P1QualificationNoiseError(f"expected JSON object: {source}")
    result = dict(payload)
    if source.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R3P1QualificationNoiseError(
            f"JSON artifact is not byte-canonical: {source}"
        )
    return result


def array_sha256(array: Any) -> str:
    """Hash dtype, shape, and contiguous values using the frozen V6 scheme."""

    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def build_tapes() -> dict[str, dict[str, Any]]:
    """Construct the five exact float32 V6 arrays in memory."""

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
    for filename, definition in tapes.items():
        array = definition["standard_normal"]
        if (
            array.shape != TAPE_SHAPE
            or array.dtype != np.float32
            or not array.flags.c_contiguous
            or not np.all(np.isfinite(array))
        ):
            raise V12R3P1QualificationNoiseError(
                f"generated tape contract drifted: {filename}"
            )
    return tapes


def _verified_design_binding() -> dict[str, Any]:
    path = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if _record(path) != DESIGN_FREEZE_RECORD:
        raise V12R3P1QualificationNoiseError(
            "canonical qualification design freeze hash/size drifted"
        )
    try:
        payload = design_freezer.verify_design_freeze()
    except Exception as exc:
        raise V12R3P1QualificationNoiseError(
            "canonical qualification design freeze verification failed"
        ) from exc
    if (
        payload.get("status") != contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("design_snapshot", {}).get("candidate_id")
        != contract.P1_CANDIDATE_ID
        or payload.get("design_snapshot", {}).get("candidate_module")
        != contract.P1_CANDIDATE_MODULE
        or payload.get("design_snapshot", {}).get("holdout_cases")
        != list(contract.HOLDOUT_CASES)
        or payload.get("design_snapshot", {}).get("rollout_matrix")
        != list(contract.ROLLOUT_MATRIX)
    ):
        raise V12R3P1QualificationNoiseError(
            "canonical qualification design semantics drifted"
        )
    return {"record": copy.deepcopy(DESIGN_FREEZE_RECORD), "payload": payload}


def _candidate_fields(name: str, payload: Mapping[str, Any]) -> tuple[Any, Any]:
    if name == "salvage_protocol_freeze":
        return payload.get("selected_candidate_id"), payload.get("selected_candidate")
    if name in {"salvage_execution_lock", "salvage_terminal_pass_ledger"}:
        return payload.get("candidate_id"), payload.get("candidate_module")
    if name == "salvage_final_development_receipt":
        return payload.get("candidate_id"), None
    raise V12R3P1QualificationNoiseError(f"unknown prerequisite: {name}")


def validate_prerequisite_payloads(
    payloads: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Any]],
    *,
    enforce_canonical_record_paths: bool = True,
) -> dict[str, Any]:
    """Validate the exact four terminal P1S records without reading source files."""

    if type(enforce_canonical_record_paths) is not bool:
        raise V12R3P1QualificationNoiseError(
            "enforce_canonical_record_paths must be bool"
        )

    requirements = contract.prerequisite_requirements()
    expected_names = [str(row["name"]) for row in requirements]
    checks: dict[str, bool] = {
        "exact_four_names": list(payloads) == expected_names
        and list(records) == expected_names
    }

    def artifact_record_valid(value: Any) -> bool:
        return bool(
            isinstance(value, Mapping)
            and set(value) == {"path", "sha256", "size_bytes"}
            and isinstance(value.get("path"), str)
            and bool(value["path"])
            and isinstance(value.get("sha256"), str)
            and len(value["sha256"]) == 64
            and all(character in "0123456789abcdef" for character in value["sha256"])
            and type(value.get("size_bytes")) is int
            and value["size_bytes"] > 0
        )

    rows: list[dict[str, Any]] = []
    for requirement in requirements:
        name = str(requirement["name"])
        payload = payloads.get(name)
        record = records.get(name)
        candidate_id, candidate_module = (
            _candidate_fields(name, payload)
            if isinstance(payload, Mapping)
            else (None, None)
        )
        exact_identity = candidate_id == contract.P1_CANDIDATE_ID and (
            candidate_module is None or candidate_module == contract.P1_CANDIDATE_MODULE
        )
        record_ok = artifact_record_valid(record) and (
            not enforce_canonical_record_paths
            or record.get("path") == requirement["path"]
        )
        passed = bool(
            isinstance(payload, Mapping)
            and payload.get("status") == requirement["required_status"]
            and payload.get("passed") is True
            and exact_identity
            and record_ok
        )
        checks[f"{name}_terminal_exact"] = passed
        rows.append(
            {
                "name": name,
                "status": (
                    payload.get("status") if isinstance(payload, Mapping) else None
                ),
                "passed": (
                    payload.get("passed") if isinstance(payload, Mapping) else None
                ),
                "candidate_id": candidate_id,
                "candidate_module_tree_sha256": contract.P1_CANDIDATE_MODULE[
                    "tree_sha256"
                ],
                "artifact": copy.deepcopy(record),
            }
        )

    protocol = payloads.get("salvage_protocol_freeze", {})
    lock = payloads.get("salvage_execution_lock", {})
    final = payloads.get("salvage_final_development_receipt", {})
    ledger = payloads.get("salvage_terminal_pass_ledger", {})
    design = copy.deepcopy(DESIGN_FREEZE_RECORD)
    protocol_record = records.get("salvage_protocol_freeze")
    lock_record = records.get("salvage_execution_lock")
    final_record = records.get("salvage_final_development_receipt")
    completed_receipts = ledger.get("completed_receipts")
    completed_receipts_exact = bool(
        isinstance(completed_receipts, list)
        and len(completed_receipts) == len(contract.SALVAGE_STAGE_IDS)
        and all(
            isinstance(row, Mapping)
            and set(row) == {"stage_id", "receipt"}
            and row.get("stage_id") == stage_id
            and artifact_record_valid(row.get("receipt"))
            for row, stage_id in zip(
                completed_receipts, contract.SALVAGE_STAGE_IDS, strict=True
            )
        )
        and completed_receipts[-1].get("receipt") == final_record
    )
    checks.update(
        {
            "same_design_freeze_bound": all(
                isinstance(payload, Mapping)
                and payload.get("qualification_design_freeze") == design
                for payload in (protocol, lock, final, ledger)
            ),
            "lock_binds_protocol": isinstance(lock, Mapping)
            and lock.get("protocol_freeze") == protocol_record,
            "ledger_binds_protocol_and_lock": isinstance(ledger, Mapping)
            and ledger.get("protocol_freeze") == protocol_record
            and ledger.get("execution_lock") == lock_record,
            "final_six_of_six": isinstance(final, Mapping)
            and final.get("case_count") == 6
            and final.get("stage_id") == contract.SALVAGE_STAGE_IDS[-1]
            and isinstance(final.get("case_receipts"), list)
            and len(final["case_receipts"]) == 6
            and all(artifact_record_valid(row) for row in final["case_receipts"])
            and final.get("retry_authorized") is False
            and final.get("actor_updates") == 0
            and final.get("critic_updates") == 0
            and final.get("ppo_updates") == 0,
            "terminal_ledger_six_of_six": isinstance(ledger, Mapping)
            and ledger.get("terminal") is True
            and ledger.get("stage_order") == list(contract.SALVAGE_STAGE_IDS)
            and ledger.get("attempted_stage") is None
            and ledger.get("completed_stages") == list(contract.SALVAGE_STAGE_IDS)
            and ledger.get("failed_stage_receipt") is None
            and ledger.get("aggregate_requires_6_of_6") is True
            and ledger.get("compensation_authorized") is False
            and ledger.get("retry_authorized") is False
            and ledger.get("resume_authorized") is False
            and ledger.get("runtime_promoted") is False
            and completed_receipts_exact,
            "zero_updates_across_prerequisites": all(
                isinstance(payload, Mapping)
                and payload.get("actor_updates", 0) == 0
                and payload.get("critic_updates", 0) == 0
                and payload.get("ppo_updates", 0) == 0
                for payload in (protocol, lock, final, ledger)
            ),
            "no_runtime_promotion_across_prerequisites": isinstance(protocol, Mapping)
            and protocol.get("runtime_promoted") is False
            and isinstance(lock.get("authority"), Mapping)
            and lock["authority"].get("runtime_promotion_authorized") is False
            and isinstance(final.get("execution_authority"), Mapping)
            and final["execution_authority"].get("runtime_promotion_authorized")
            is False
            and ledger.get("runtime_promoted") is False,
        }
    )
    frozen_rows = []
    for requirement, row in zip(requirements, rows, strict=True):
        artifact = copy.deepcopy(row["artifact"])
        if isinstance(artifact, Mapping):
            artifact = dict(artifact)
            # Non-canonical prerequisite paths are accepted only by explicit
            # test overrides.  The frozen pure gate always evaluates the
            # canonical artifact identity that production publication binds.
            artifact["path"] = str(requirement["path"])
        frozen_rows.append({**copy.deepcopy(row), "artifact": artifact})
    frozen_payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREREQUISITE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "prerequisites": frozen_rows,
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
    frozen_gate = gates.prerequisite_gate(frozen_payload)
    checks["frozen_prerequisite_gate_pass"] = (
        frozen_gate.get("status") == contract.PREREQUISITE_PASS_STATUS
        and frozen_gate.get("passed") is True
        and frozen_gate.get("qualification_execution_authorized") is False
        and frozen_gate.get("runtime_promoted") is False
    )
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PREREQUISITE_PASS_STATUS
            if passed
            else contract.PREREQUISITE_FAIL_STATUS
        ),
        "passed": passed,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "checks": checks,
        "rows": rows,
        "frozen_gate": frozen_gate,
    }


def _canonical_prerequisite_paths() -> dict[str, Path]:
    return {
        str(row["name"]): resolve_relative(str(row["path"]))
        for row in contract.prerequisite_requirements()
    }


def load_and_validate_prerequisites(
    *,
    prerequisite_paths: Mapping[str, Path] | None = None,
    require_qualification_unopened: bool = True,
) -> dict[str, Any]:
    if type(require_qualification_unopened) is not bool:
        raise V12R3P1QualificationNoiseError(
            "require_qualification_unopened must be bool"
        )
    if require_qualification_unopened:
        occupied = [
            path
            for path in (
                resolve_relative(contract.NOISE_ROOT),
                resolve_relative(contract.RUN_ROOT),
                resolve_relative(contract.PROTOCOL_FREEZE_PATH),
                resolve_relative(contract.EXECUTION_LOCK_PATH),
            )
            if os.path.lexists(path)
        ]
        if occupied:
            raise V12R3P1QualificationNoiseError(
                "qualification must be unopened before prerequisite binding: "
                f"{[_portable_path(path) for path in occupied]}"
            )
    paths = dict(
        _canonical_prerequisite_paths()
        if prerequisite_paths is None
        else prerequisite_paths
    )
    names = [str(row["name"]) for row in contract.prerequisite_requirements()]
    if (
        list(paths) != names
        or len({Path(path).absolute() for path in paths.values()}) != 4
    ):
        raise V12R3P1QualificationNoiseError(
            "prerequisite paths must be four distinct ordered artifacts"
        )
    terminal_name = "salvage_terminal_pass_ledger"
    terminal = _mapping(paths[terminal_name])
    terminal_requirement = next(
        row
        for row in contract.prerequisite_requirements()
        if row["name"] == terminal_name
    )
    if (
        terminal.get("status") != terminal_requirement["required_status"]
        or terminal.get("passed") is not True
        or terminal.get("terminal") is not True
    ):
        raise V12R3P1QualificationNoiseError(
            "V12P1S terminal prerequisites are not eligible: "
            f"ledger status={terminal.get('status')!r}, "
            f"passed={terminal.get('passed')!r}"
        )
    payloads = {
        name: terminal if name == terminal_name else _mapping(paths[name])
        for name in names
    }
    records = {name: _record(paths[name]) for name in names}
    gate = validate_prerequisite_payloads(
        payloads,
        records,
        enforce_canonical_record_paths=prerequisite_paths is None,
    )
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if value is not True]
        raise V12R3P1QualificationNoiseError(
            f"V12P1S terminal prerequisites are not eligible: {failed}"
        )
    return {"payloads": payloads, "records": records, "gate": gate}


def _write_npz_exclusive(path: Path, *, array: Any, seed: int | None) -> None:
    _assert_no_link_components(path)
    if os.path.lexists(path):
        raise V12R3P1QualificationNoiseError(f"refusing to clobber: {path}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            arrays: dict[str, Any] = {"standard_normal": array}
            if seed is not None:
                arrays["seed"] = np.asarray([seed], dtype=np.int64)
            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claim = os.open(path, flags, 0o600)
        except FileExistsError as exc:
            raise V12R3P1QualificationNoiseError(
                f"refusing to clobber: {path}"
            ) from exc
        else:
            os.close(claim)
        os.replace(temporary, path)
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _tape_record(path: Path) -> dict[str, Any]:
    record = _record(path)
    record["path"] = path.name
    return record


def _manifest_payload(
    *,
    prerequisites: Mapping[str, Any],
    design: Mapping[str, Any],
    tape_records: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    cases = []
    for case in contract.canonical_cases():
        row = copy.deepcopy(case)
        filename = PurePosixPath(str(row["noise_tape"])).name
        row["noise_tape_array_sha256"] = tape_records[filename]["array_sha256"]
        cases.append(row)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "revision": contract.REVISION,
        "status": NOISE_TAPES_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module_tree_sha256": contract.P1_CANDIDATE_MODULE["tree_sha256"],
        "qualification_design_freeze": copy.deepcopy(design["record"]),
        "prerequisite_gate": copy.deepcopy(prerequisites["gate"]),
        "prerequisites": copy.deepcopy(prerequisites["records"]),
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "standard_normal_pre_scaling": True,
        "shape": list(TAPE_SHAPE),
        "dtype": "float32",
        "case_order": list(contract.CASE_IDS),
        "cases": cases,
        "tapes": copy.deepcopy(tape_records),
        "same_tape_required_for_condition_pair": True,
        "qualification_execution_authorized_by_manifest": False,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }


def _read_tape(path: Path, *, seed: int | None) -> Any:
    _assert_no_link_components(path)
    if not path.is_file() or _is_link_or_reparse(path):
        raise V12R3P1QualificationNoiseError(f"tape is missing/unsafe: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            expected_files = {"standard_normal"} | (
                {"seed"} if seed is not None else set()
            )
            if set(archive.files) != expected_files:
                raise V12R3P1QualificationNoiseError(
                    f"tape archive schema drifted: {path.name}"
                )
            array = np.ascontiguousarray(archive["standard_normal"])
            if seed is not None:
                seed_array = np.asarray(archive["seed"])
                if seed_array.dtype != np.int64 or seed_array.tolist() != [seed]:
                    raise V12R3P1QualificationNoiseError(
                        f"tape seed drifted: {path.name}"
                    )
    except V12R3P1QualificationNoiseError:
        raise
    except Exception as exc:
        raise V12R3P1QualificationNoiseError(f"cannot read tape: {path.name}") from exc
    return array


def verify_manifest(
    *,
    output_root: str | os.PathLike[str] | Path = DEFAULT_OUTPUT_ROOT,
    prerequisite_paths: Mapping[str, Path] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    root = Path(output_root).absolute()
    canonical = DEFAULT_OUTPUT_ROOT.absolute()
    if enforce_canonical_destination and root != canonical:
        raise V12R3P1QualificationNoiseError(f"non-canonical noise root: {root}")
    _assert_no_link_components(root)
    design = _verified_design_binding()
    prerequisites = load_and_validate_prerequisites(
        prerequisite_paths=prerequisite_paths,
        require_qualification_unopened=False,
    )
    definitions = build_tapes()
    records: dict[str, dict[str, Any]] = {}
    for filename, definition in definitions.items():
        array = _read_tape(root / filename, seed=definition["seed"])
        expected = definition["standard_normal"]
        if (
            array.shape != TAPE_SHAPE
            or array.dtype != np.float32
            or not array.flags.c_contiguous
            or not np.array_equal(array, expected)
            or array_sha256(array) != array_sha256(expected)
        ):
            raise V12R3P1QualificationNoiseError(
                f"tape bytes/values drifted: {filename}"
            )
        records[filename] = {
            "seed": definition["seed"],
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "array_sha256": array_sha256(array),
            "artifact": _tape_record(root / filename),
        }
    expected_manifest = _manifest_payload(
        prerequisites=prerequisites,
        design=design,
        tape_records=records,
    )
    manifest_path = root / "manifest.json"
    observed = _mapping(manifest_path)
    if (
        observed != expected_manifest
        or manifest_path.read_bytes()
        != forensic.canonical_json_bytes(expected_manifest)
    ):
        raise V12R3P1QualificationNoiseError("noise manifest drifted")
    return observed


def prepare(
    *,
    output_root: str | os.PathLike[str] | Path = DEFAULT_OUTPUT_ROOT,
    prerequisite_paths: Mapping[str, Path] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Publish all five tapes and their manifest once after terminal P1S PASS."""

    root = Path(output_root).absolute()
    canonical = DEFAULT_OUTPUT_ROOT.absolute()
    if enforce_canonical_destination and root != canonical:
        raise V12R3P1QualificationNoiseError(f"non-canonical noise root: {root}")
    _assert_no_link_components(root)
    if os.path.lexists(root):
        raise V12R3P1QualificationNoiseError(f"refusing to clobber: {root}")
    design = _verified_design_binding()
    prerequisites = load_and_validate_prerequisites(
        prerequisite_paths=prerequisite_paths
    )
    definitions = build_tapes()
    root.parent.mkdir(parents=True, exist_ok=True)
    try:
        root.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise V12R3P1QualificationNoiseError(f"refusing to clobber: {root}") from exc
    records: dict[str, dict[str, Any]] = {}
    for filename, definition in definitions.items():
        path = root / filename
        _write_npz_exclusive(
            path,
            array=definition["standard_normal"],
            seed=definition["seed"],
        )
        records[filename] = {
            "seed": definition["seed"],
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "array_sha256": array_sha256(definition["standard_normal"]),
            "artifact": _tape_record(path),
        }
    manifest = _manifest_payload(
        prerequisites=prerequisites,
        design=design,
        tape_records=records,
    )
    forensic.write_json_exclusive(root / "manifest.json", manifest)
    return verify_manifest(
        output_root=root,
        prerequisite_paths=prerequisite_paths,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = prepare() if args.prepare else verify_manifest()
    print(f"{result['status']}: {contract.NOISE_MANIFEST_PATH.as_posix()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "DESIGN_FREEZE_RECORD",
    "NOISE_TAPES_PASS_STATUS",
    "TAPE_SHAPE",
    "V12R3P1QualificationNoiseError",
    "array_sha256",
    "build_tapes",
    "load_and_validate_prerequisites",
    "prepare",
    "validate_prerequisite_payloads",
    "verify_manifest",
]
