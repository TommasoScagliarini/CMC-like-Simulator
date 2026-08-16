"""Build the five deterministic Q3 noise tapes after the Q3 lock is live.

Array generation is pure and deterministic.  Publication is explicit,
exclusive, and candidate-bound.  Merely importing or running ``--build-only``
does not create any file.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np


RUNTIME_ROOT = Path(__file__).resolve().parent
Q3_ROOT = RUNTIME_ROOT.parent
for _root in (Q3_ROOT, RUNTIME_ROOT, Q3_ROOT.parents[3] / "validation"):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r7_q3_artifacts as artifacts  # noqa: E402
import h0_v12r7_q3_prerequisites as prerequisites  # noqa: E402
import h0_v12r7_q3_qualification_contract as contract  # noqa: E402
import freeze_h0_v12r7_q3_qualification_protocol as freezer  # noqa: E402


class V12R7Q3NoiseError(RuntimeError):
    """Raised before an invalid or premature tape operation."""


NOISE_TAPES_PASS_STATUS = "PASS_H0_V12R7_Q3_QUALIFICATION_NOISE_TAPES"
TAPE_SHAPE = (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)


def array_sha256(array: Any) -> str:
    value = np.ascontiguousarray(np.asarray(array))
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(json.dumps(list(value.shape), separators=(",", ":")).encode("ascii"))
    digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def build_tapes() -> dict[str, dict[str, Any]]:
    """Regenerate all five preregistered arrays without writing."""

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
    hashes = {
        filename: array_sha256(row["standard_normal"])
        for filename, row in tapes.items()
    }
    if hashes != contract.EXPECTED_TAPE_ARRAY_SHA256:
        raise V12R7Q3NoiseError("Q3 tape ABI/hash drifted")
    if not all(
        row["standard_normal"].shape == TAPE_SHAPE
        and row["standard_normal"].dtype == np.float32
        and row["standard_normal"].flags.c_contiguous
        and np.all(np.isfinite(row["standard_normal"]))
        for row in tapes.values()
    ):
        raise V12R7Q3NoiseError("Q3 tape array contract drifted")
    return tapes


def _write_npz_exclusive(path: Path, *, array: Any, seed: int | None) -> None:
    if os.path.lexists(path):
        raise V12R7Q3NoiseError(f"refusing to clobber: {path}")
    with path.open("xb") as stream:
        values: dict[str, Any] = {"standard_normal": array}
        if seed is not None:
            values["seed"] = np.asarray([seed], dtype=np.int64)
        np.savez(stream, **values)
        stream.flush()
        os.fsync(stream.fileno())


def _read_tape(path: Path, *, seed: int | None) -> Any:
    try:
        with np.load(path, allow_pickle=False) as archive:
            expected = (
                {"standard_normal"} if seed is None else {"standard_normal", "seed"}
            )
            if set(archive.files) != expected:
                raise V12R7Q3NoiseError(f"tape keys drifted: {path}")
            if seed is not None:
                stored_seed = archive["seed"]
                if (
                    stored_seed.dtype != np.int64
                    or stored_seed.shape != (1,)
                    or stored_seed.tolist() != [seed]
                ):
                    raise V12R7Q3NoiseError(f"tape seed drifted: {path}")
            stored = archive["standard_normal"]
            if stored.dtype != np.float32 or stored.shape != TAPE_SHAPE:
                raise V12R7Q3NoiseError(f"tape array ABI drifted: {path}")
            return np.ascontiguousarray(stored)
    except V12R7Q3NoiseError:
        raise
    except BaseException as exc:
        raise V12R7Q3NoiseError(f"cannot read tape: {path}") from exc


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except BaseException as exc:
        raise V12R7Q3NoiseError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R7Q3NoiseError(f"expected JSON object: {path}")
    result = dict(value)
    if path.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R7Q3NoiseError(f"JSON is not canonical: {path}")
    return result


def _validated_snapshot(provided: Mapping[str, Any] | None) -> dict[str, Any]:
    snapshot = (
        freezer.live_r7_prerequisite_snapshot(require_q3_unopened=False)
        if provided is None
        else copy.deepcopy(dict(provided))
    )
    gate = snapshot.get("gate")
    if not isinstance(gate, Mapping) or gate.get("passed") is not True:
        raise V12R7Q3NoiseError("R7 prerequisite snapshot is not terminal PASS")
    try:
        prerequisites.bind_candidate(
            gate.get("candidate_id"), gate.get("candidate_module")
        )
    except BaseException as exc:
        raise V12R7Q3NoiseError("R7 candidate snapshot is invalid") from exc
    return snapshot


def _manifest_payload(
    *,
    root: Path,
    snapshot: Mapping[str, Any],
    protocol_record: Mapping[str, Any],
    lock_record: Mapping[str, Any],
) -> dict[str, Any]:
    definitions = build_tapes()
    rows: dict[str, Any] = {}
    for filename, definition in definitions.items():
        rows[filename] = {
            "seed": definition["seed"],
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "c_contiguous": True,
            "array_sha256": array_sha256(definition["standard_normal"]),
            "artifact": forensic.artifact_record(
                root / filename, artifact_root=root.parent
            ),
        }
    gate = snapshot["gate"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": NOISE_TAPES_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": gate["candidate_id"],
        "candidate_module": copy.deepcopy(gate["candidate_module"]),
        "protocol_freeze": copy.deepcopy(dict(protocol_record)),
        "execution_lock": copy.deepcopy(dict(lock_record)),
        "r7_prerequisite_gate": copy.deepcopy(dict(gate)),
        "root": (
            contract.NOISE_ROOT.as_posix()
            if root == freezer.resolve_relative(contract.NOISE_ROOT)
            else root.as_posix()
        ),
        "shape": list(TAPE_SHAPE),
        "dtype": "float32",
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "hash_scheme": "sha256(dtype_ascii+compact_json_shape+contiguous_c_bytes)",
        "tapes": rows,
        "case_order": list(contract.CASE_IDS),
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
    prerequisite_snapshot: Mapping[str, Any] | None = None,
    protocol_record: Mapping[str, Any] | None = None,
    lock_record: Mapping[str, Any] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Write a no-clobber candidate-bound tape set and strict manifest."""

    if enforce_canonical_destination and contract.HISTORICAL_TERMINAL_FAILURE is True:
        raise V12R7Q3NoiseError(
            "V12R7 terminal FAIL permanently closes canonical Q3 tape publication"
        )
    canonical = freezer.resolve_relative(contract.NOISE_ROOT)
    root = canonical if output_root is None else Path(output_root).absolute()
    if enforce_canonical_destination and root != canonical:
        raise V12R7Q3NoiseError(f"non-canonical noise destination: {root}")
    if os.path.lexists(root):
        raise V12R7Q3NoiseError(f"refusing to clobber: {root}")
    snapshot = _validated_snapshot(prerequisite_snapshot)
    if protocol_record is None or lock_record is None:
        freezer.verify_execution_lock()
        protocol_record = freezer._record(contract.PROTOCOL_FREEZE_PATH)
        lock_record = freezer._record(contract.EXECUTION_LOCK_PATH)
    if (
        not (
            artifacts.artifact_record_matches(
                protocol_record, contract.PROTOCOL_FREEZE_PATH
            )
            and artifacts.artifact_record_matches(
                lock_record, contract.EXECUTION_LOCK_PATH
            )
        )
        and enforce_canonical_destination
    ):
        raise V12R7Q3NoiseError("protocol/lock records are not canonical")
    definitions = build_tapes()
    root.mkdir(parents=True, exist_ok=False)
    for filename, definition in definitions.items():
        _write_npz_exclusive(
            root / filename,
            array=definition["standard_normal"],
            seed=definition["seed"],
        )
    payload = _manifest_payload(
        root=root,
        snapshot=snapshot,
        protocol_record=protocol_record,
        lock_record=lock_record,
    )
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
    canonical = freezer.resolve_relative(contract.NOISE_ROOT)
    root = canonical if noise_root is None else Path(noise_root).absolute()
    if enforce_canonical_destination and root != canonical:
        raise V12R7Q3NoiseError(f"non-canonical noise root: {root}")
    payload = _strict_mapping(root / "manifest.json")
    expected_files = {"manifest.json", *contract.EXPECTED_TAPE_ARRAY_SHA256}
    if (
        not root.is_dir()
        or root.is_symlink()
        or {item.name for item in root.iterdir()} != expected_files
    ):
        raise V12R7Q3NoiseError("noise root file set drifted")
    binding = prerequisites.validate_candidate_tree(
        payload.get("candidate_id"), payload.get("candidate_module")
    )
    if (
        payload.get("schema_version") != contract.SCHEMA_VERSION
        or payload.get("status") != NOISE_TAPES_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("protocol_id") != contract.PROTOCOL_ID
        or payload.get("candidate_id") != binding["candidate_id"]
        or payload.get("shape") != list(TAPE_SHAPE)
        or payload.get("dtype") != "float32"
        or payload.get("same_tape_required_for_condition_pair") is not True
        or payload.get("case_order") != list(contract.CASE_IDS)
        or type(payload.get("qualification_rollouts_opened")) is not int
        or payload.get("qualification_rollouts_opened") != 0
        or type(payload.get("actor_updates")) is not int
        or payload.get("actor_updates") != 0
        or type(payload.get("critic_updates")) is not int
        or payload.get("critic_updates") != 0
        or type(payload.get("ppo_updates")) is not int
        or payload.get("ppo_updates") != 0
        or payload.get("runtime_promoted") is not False
    ):
        raise V12R7Q3NoiseError("noise manifest semantics drifted")
    if enforce_canonical_destination and (
        payload.get("root") != contract.NOISE_ROOT.as_posix()
        or payload.get("protocol_freeze")
        != freezer._record(contract.PROTOCOL_FREEZE_PATH)
        or payload.get("execution_lock")
        != freezer._record(contract.EXECUTION_LOCK_PATH)
    ):
        raise V12R7Q3NoiseError("canonical noise lineage drifted")
    tapes = payload.get("tapes")
    if not isinstance(tapes, Mapping) or set(tapes) != set(
        contract.EXPECTED_TAPE_ARRAY_SHA256
    ):
        raise V12R7Q3NoiseError("noise manifest rows drifted")
    definitions = build_tapes()
    for filename, expected_hash in contract.EXPECTED_TAPE_ARRAY_SHA256.items():
        row = tapes[filename]
        if not isinstance(row, Mapping):
            raise V12R7Q3NoiseError(f"noise row malformed: {filename}")
        expected_seed = definitions[filename]["seed"]
        array = _read_tape(root / filename, seed=expected_seed)
        artifact = forensic.artifact_record(root / filename, artifact_root=root.parent)
        if (
            array.shape != TAPE_SHAPE
            or array.dtype != np.float32
            or not array.flags.c_contiguous
            or array_sha256(array) != expected_hash
            or row.get("array_sha256") != expected_hash
            or row.get("seed") != expected_seed
            or row.get("artifact") != artifact
        ):
            raise V12R7Q3NoiseError(f"noise tape closure drifted: {filename}")
    prerequisites.bind_candidate(binding["candidate_id"], binding["candidate_module"])
    return payload


def load_case_tape(case_id: str) -> tuple[Any, dict[str, Any], str]:
    """Load one canonical verified tape for the physical runtime."""

    manifest = verify_manifest()
    case = contract.canonical_case(case_id)
    path = freezer.resolve_relative(case["noise_tape"])
    filename = path.name
    row = manifest["tapes"][filename]
    array = _read_tape(path, seed=row["seed"])
    return array, freezer._record(path), row["array_sha256"]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--build-only", action="store_true")
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.build_only:
        result = {
            "status": "PASS_H0_V12R7_Q3_TAPES_BUILD_ONLY",
            "passed": len(build_tapes()) == 5,
        }
    elif args.prepare:
        result = prepare()
    else:
        result = verify_manifest()
    print(result["status"])
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "NOISE_TAPES_PASS_STATUS",
    "V12R7Q3NoiseError",
    "array_sha256",
    "build_tapes",
    "load_case_tape",
    "prepare",
    "verify_manifest",
]
