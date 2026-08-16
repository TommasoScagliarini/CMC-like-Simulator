"""Materialize the preregistered V6 qualification noise tapes fail-closed.

The canonical destination is deliberately absent when this source is added.
Materialization requires explicit, strict-JSON candidate-freeze and development
PASS receipts for the same candidate.  Every destination is no-clobber and the
manifest is published last.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

try:  # Package import in tests; direct import when invoked as a script.
    from validation import h0_primary_split_v6_qualification_contract as contract
except ImportError:  # pragma: no cover
    import h0_primary_split_v6_qualification_contract as contract


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / contract.NOISE_ROOT
TAPE_SHAPE = (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)


class QualificationNoiseTapeError(RuntimeError):
    """Raised before overwrite or on malformed qualification provenance."""


def _reject_json_constant(token: str) -> None:
    raise QualificationNoiseTapeError(f"non-finite JSON constant: {token}")


def _unique_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise QualificationNoiseTapeError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _strict_json_object(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    try:
        with source.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=_reject_json_constant,
                object_pairs_hook=_unique_object,
            )
    except QualificationNoiseTapeError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise QualificationNoiseTapeError(
            f"cannot read strict JSON prerequisite {source}: {exc}"
        ) from exc
    if not isinstance(value, Mapping):
        raise QualificationNoiseTapeError(
            f"prerequisite must be a JSON object: {source}"
        )
    _canonical_json_bytes(value)
    return dict(value)


def _canonical_json_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise QualificationNoiseTapeError(
            f"value is not finite strict JSON: {exc}"
        ) from exc


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def array_sha256(array: Any) -> str:
    """Hash dtype, shape, and contiguous values using the established scheme."""

    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def _portable_source_path(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def _source_record(
    path: str | Path, *, portable_path: str | None = None
) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    return {
        "path": portable_path or _portable_source_path(resolved),
        "sha256": _sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _publish_temporary_exclusive(temporary: Path, target: Path) -> None:
    try:
        os.link(temporary, target)
    except FileExistsError as exc:
        raise QualificationNoiseTapeError(f"refusing to clobber: {target}") from exc
    except OSError as exc:
        raise QualificationNoiseTapeError(
            f"cannot publish no-clobber artifact {target}: {exc}"
        ) from exc
    temporary.unlink()


def _write_npz_exclusive(path: Path, **arrays: Any) -> None:
    if os.path.lexists(path):
        raise QualificationNoiseTapeError(f"refusing to clobber: {path}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _write_json_exclusive(path: Path, payload: Any) -> None:
    _canonical_json_bytes(payload)
    if os.path.lexists(path):
        raise QualificationNoiseTapeError(f"refusing to clobber: {path}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _validate_prerequisites(
    *,
    candidate_freeze_receipt: str | Path,
    development_gate_receipt: str | Path,
) -> tuple[str, dict[str, Any]]:
    paths = {
        "candidate_freeze": Path(candidate_freeze_receipt).expanduser().resolve(),
        "development_gate": Path(development_gate_receipt).expanduser().resolve(),
    }
    if paths["candidate_freeze"] == paths["development_gate"]:
        raise QualificationNoiseTapeError(
            "candidate freeze and development gate must be distinct artifacts"
        )

    records: dict[str, Any] = {}
    candidate_id: str | None = None
    for requirement in contract.prerequisite_requirements():
        name = str(requirement["name"])
        path = paths[name]
        payload = _strict_json_object(path)
        required_status = requirement["required_status"]
        if payload.get("status") != required_status:
            raise QualificationNoiseTapeError(
                f"{name}.status must be {required_status!r}"
            )
        required_passed = requirement["required_passed"]
        if required_passed is not None and payload.get("passed") is not required_passed:
            raise QualificationNoiseTapeError(
                f"{name}.passed must be {required_passed!r}"
            )
        observed_id = payload.get("candidate_id")
        if not isinstance(observed_id, str) or not observed_id.strip():
            raise QualificationNoiseTapeError(
                f"{name}.candidate_id must be a non-empty string"
            )
        if candidate_id is None:
            candidate_id = observed_id
        elif observed_id != candidate_id:
            raise QualificationNoiseTapeError(
                "candidate freeze and development PASS identify different candidates"
            )
        records[name] = {
            "status": required_status,
            "passed": payload.get("passed"),
            "candidate_id": observed_id,
            "artifact": _source_record(path),
        }

    if candidate_id is None:  # Defensive: the frozen contract requires two rows.
        raise QualificationNoiseTapeError("no candidate identity was established")
    return candidate_id, records


def _build_tapes() -> dict[str, dict[str, Any]]:
    tapes: dict[str, dict[str, Any]] = {
        "deterministic_all_zero.npz": {
            "seed": None,
            "standard_normal": np.zeros(TAPE_SHAPE, dtype=np.float32),
        }
    }
    for seed in contract.STOCHASTIC_SEEDS:
        tapes[f"stochastic_seed_{seed}_standard_normal.npz"] = {
            "seed": int(seed),
            "standard_normal": np.random.default_rng(seed)
            .standard_normal(TAPE_SHAPE)
            .astype(np.float32),
        }

    for filename, record in tapes.items():
        array = record["standard_normal"]
        if array.shape != TAPE_SHAPE or array.dtype != np.float32:
            raise QualificationNoiseTapeError(
                f"generated tape contract drifted: {filename}"
            )
        if not np.all(np.isfinite(array)):
            raise QualificationNoiseTapeError(
                f"generated tape is non-finite: {filename}"
            )
    return tapes


def prepare(
    *,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    candidate_freeze_receipt: str | Path,
    development_gate_receipt: str | Path,
) -> dict[str, Any]:
    """Create all five tapes and the manifest after both prerequisite gates."""

    candidate_id, prerequisites = _validate_prerequisites(
        candidate_freeze_receipt=candidate_freeze_receipt,
        development_gate_receipt=development_gate_receipt,
    )
    tapes = _build_tapes()
    destination = Path(output_root).expanduser().resolve()
    if os.path.lexists(destination):
        raise QualificationNoiseTapeError(f"refusing to clobber: {destination}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        destination.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise QualificationNoiseTapeError(
            f"refusing to clobber: {destination}"
        ) from exc

    tape_records: dict[str, dict[str, Any]] = {}
    for filename, definition in tapes.items():
        path = destination / filename
        array = definition["standard_normal"]
        seed = definition["seed"]
        archive_arrays: dict[str, Any] = {"standard_normal": array}
        if seed is not None:
            archive_arrays["seed"] = np.asarray([seed], dtype=np.int64)
        _write_npz_exclusive(path, **archive_arrays)
        tape_records[filename] = {
            "seed": seed,
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "array_sha256": array_sha256(array),
            "artifact": _source_record(path, portable_path=filename),
        }

    cases: list[dict[str, Any]] = []
    for case in contract.canonical_cases():
        case_record = dict(case)
        filename = Path(case_record["noise_tape"]).name
        case_record["noise_tape_array_sha256"] = tape_records[filename]["array_sha256"]
        cases.append(case_record)

    manifest: dict[str, Any] = {
        "schema_version": contract.SCHEMA_VERSION,
        "revision": contract.REVISION,
        "status": contract.NOISE_TAPES_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "standard_normal_pre_scaling": True,
        "shape": list(TAPE_SHAPE),
        "dtype": "float32",
        "cases": cases,
        "tapes": tape_records,
        "prerequisites": prerequisites,
        "qualification_execution_authorized_by_manifest": False,
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "manifest.json", manifest)
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Materialize fresh V6 qualification tapes after candidate freeze "
            "and a development PASS."
        )
    )
    parser.add_argument(
        "--candidate-freeze-receipt",
        required=True,
        help="Strict-JSON immutable candidate-freeze receipt.",
    )
    parser.add_argument(
        "--development-gate-receipt",
        required=True,
        help="Strict-JSON condition-matched development PASS gate.",
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="No-clobber destination (defaults to the canonical V6 tape root).",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    manifest = prepare(
        output_root=args.output_root,
        candidate_freeze_receipt=args.candidate_freeze_receipt,
        development_gate_receipt=args.development_gate_receipt,
    )
    print(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
