"""Materialize Q3 tapes only after the five official R5 verifiers pass.

Array construction is in-memory and deterministic.  Publication is explicit,
no-clobber, and supports a noncanonical temporary root only for tests.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np

import h0_v12r5_q3_artifacts as artifacts
import h0_v12r5_q3_qualification_gates as gates
import h0_v12r5_q3_runtime_contract as contract
import verify_h0_v12r5_q3_prerequisites as prerequisites_module


Q3_ROOT = Path(__file__).resolve().parent.parent
ROOT_VALIDATION = artifacts.REPO_ROOT / "validation"
for _root in (Q3_ROOT, ROOT_VALIDATION):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r5_q3_qualification_design as design_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402


class V12R5Q3QualificationNoiseError(RuntimeError):
    """Raised before a noncanonical or ineligible Q3 tape operation."""


DESIGN_FREEZE_RECORD = {
    "path": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
    "sha256": "f2764f2cf16abfc168255056fdf0c1407d97c65100bb3965b866f38db084e56d",
    "size_bytes": 68_465,
}
NOISE_TAPES_PASS_STATUS = "PASS_H0_V12R5_Q3_QUALIFICATION_NOISE_TAPES"
NOISE_TAPES_FAIL_STATUS = "FAIL_H0_V12R5_Q3_QUALIFICATION_NOISE_TAPES"
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
    observed = {
        filename: array_sha256(row["standard_normal"])
        for filename, row in tapes.items()
    }
    if observed != contract.EXPECTED_TAPE_ARRAY_SHA256:
        raise V12R5Q3QualificationNoiseError("Q3 tape ABI/hash drifted")
    if not all(
        row["standard_normal"].shape == TAPE_SHAPE
        and row["standard_normal"].dtype == np.float32
        and row["standard_normal"].flags.c_contiguous
        and np.all(np.isfinite(row["standard_normal"]))
        for row in tapes.values()
    ):
        raise V12R5Q3QualificationNoiseError("Q3 tape array contract drifted")
    return tapes


def verified_design_binding() -> dict[str, Any]:
    path = artifacts.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    if artifacts.record(path) != DESIGN_FREEZE_RECORD:
        raise V12R5Q3QualificationNoiseError("Q3 design freeze hash/size drifted")
    try:
        payload = design_freezer.verify_design_freeze()
    except BaseException as exc:
        raise V12R5Q3QualificationNoiseError(
            "Q3 design freeze official verification failed"
        ) from exc
    snapshot = payload.get("design_snapshot")
    if (
        payload.get("status") != contract.design.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("candidate_binding_state") != "DEFERRED"
        or payload.get("candidate_id") is not None
        or not isinstance(snapshot, Mapping)
        or snapshot.get("holdout_cases") != list(contract.HOLDOUT_CASES)
        or snapshot.get("rollout_matrix") != list(contract.ROLLOUT_MATRIX)
    ):
        raise V12R5Q3QualificationNoiseError("Q3 design semantics drifted")
    return {"record": copy.deepcopy(DESIGN_FREEZE_RECORD), "payload": payload}


def _write_npz_exclusive(path: Path, *, array: Any, seed: int | None) -> None:
    artifacts.assert_no_link_components(path)
    if os.path.lexists(path):
        raise V12R5Q3QualificationNoiseError(f"refusing to clobber: {path}")
    with path.open("xb") as stream:
        arrays: dict[str, Any] = {"standard_normal": array}
        if seed is not None:
            arrays["seed"] = np.asarray([seed], dtype=np.int64)
        np.savez(stream, **arrays)
        stream.flush()
        os.fsync(stream.fileno())


def _read_tape(path: Path, *, seed: int | None) -> Any:
    artifacts.assert_no_link_components(path)
    try:
        with np.load(path, allow_pickle=False) as archive:
            expected = (
                {"standard_normal"} if seed is None else {"standard_normal", "seed"}
            )
            if set(archive.files) != expected:
                raise V12R5Q3QualificationNoiseError(f"tape keys drifted: {path}")
            if seed is not None and archive["seed"].tolist() != [seed]:
                raise V12R5Q3QualificationNoiseError(f"tape seed drifted: {path}")
            return np.ascontiguousarray(archive["standard_normal"], dtype=np.float32)
    except V12R5Q3QualificationNoiseError:
        raise
    except BaseException as exc:
        raise V12R5Q3QualificationNoiseError(f"cannot read tape: {path}") from exc


def _strict_mapping(path: Path) -> dict[str, Any]:
    artifacts.assert_no_link_components(path)
    try:
        payload = forensic.strict_json_load(path)
    except BaseException as exc:
        raise V12R5Q3QualificationNoiseError(
            f"cannot read strict JSON: {path}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R5Q3QualificationNoiseError(f"expected JSON object: {path}")
    result = dict(payload)
    if path.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R5Q3QualificationNoiseError(f"JSON is not canonical: {path}")
    return result


def _validated_prerequisites(
    provided: Mapping[str, Any] | None,
) -> dict[str, Any]:
    snapshot = (
        prerequisites_module.load_and_verify_r5_prerequisites()
        if provided is None
        else copy.deepcopy(dict(provided))
    )
    gate = snapshot.get("gate")
    if not isinstance(gate, Mapping):
        raise V12R5Q3QualificationNoiseError("R5 prerequisite snapshot is malformed")
    pure = gates.r5_prerequisite_gate(gate)
    if pure.get("passed") is not True:
        raise V12R5Q3QualificationNoiseError("R5 prerequisite pure gate failed")
    contract.bind_candidate(gate.get("candidate_id"), gate.get("candidate_module"))
    result = copy.deepcopy(snapshot)
    result["pure_gate"] = pure
    return result


def _manifest_payload(
    *, root: Path, prerequisites: Mapping[str, Any]
) -> dict[str, Any]:
    definitions = build_tapes()
    rows: dict[str, Any] = {}
    for filename, definition in definitions.items():
        path = root / filename
        rows[filename] = {
            "seed": definition["seed"],
            "shape": list(TAPE_SHAPE),
            "dtype": "float32",
            "c_contiguous": True,
            "array_sha256": array_sha256(definition["standard_normal"]),
            "artifact": artifacts.record(path),
        }
    gate = prerequisites["gate"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": NOISE_TAPES_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": gate["candidate_id"],
        "candidate_module": copy.deepcopy(gate["candidate_module"]),
        "candidate_module_tree_sha256": gate["candidate_module"]["tree_sha256"],
        "qualification_design_freeze": copy.deepcopy(DESIGN_FREEZE_RECORD),
        "r5_prerequisite_gate": copy.deepcopy(gate),
        "r5_prerequisite_records": copy.deepcopy(prerequisites.get("records", {})),
        "pure_prerequisite_gate": copy.deepcopy(prerequisites["pure_gate"]),
        "root": artifacts.portable_path(root),
        "array_key": contract.TAPE_ABI["array_key"],
        "shape": list(TAPE_SHAPE),
        "dtype": "float32",
        "c_contiguous": True,
        "generator": contract.TAPE_ABI["generator"],
        "hash_scheme": contract.TAPE_ABI["hash_scheme"],
        "tapes": rows,
        "case_order": list(contract.CASE_IDS),
        "cases": [
            {
                **contract.canonical_case(case_id),
                "noise_tape_array_sha256": rows[
                    PurePosixPath(contract.canonical_case(case_id)["noise_tape"]).name
                ]["array_sha256"],
            }
            for case_id in contract.CASE_IDS
        ],
        "same_tape_required_for_condition_pair": True,
        "qualification_rollouts_opened": 0,
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "runtime_promoted": False,
    }


def prepare(
    *,
    output_root: str | os.PathLike[str] | None = None,
    prerequisites: Mapping[str, Any] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Write one no-clobber set of five tapes and its canonical manifest."""

    root = (
        artifacts.resolve_relative(contract.NOISE_ROOT)
        if output_root is None
        else Path(output_root).absolute()
    )
    canonical = artifacts.resolve_relative(contract.NOISE_ROOT)
    if enforce_canonical_destination and root != canonical:
        raise V12R5Q3QualificationNoiseError(f"non-canonical noise destination: {root}")
    artifacts.assert_no_link_components(root)
    if os.path.lexists(root):
        raise V12R5Q3QualificationNoiseError(f"refusing to clobber: {root}")
    verified_design_binding()
    prerequisite_snapshot = _validated_prerequisites(prerequisites)
    definitions = build_tapes()
    root.mkdir(parents=True, exist_ok=False)
    for filename, definition in definitions.items():
        _write_npz_exclusive(
            root / filename,
            array=definition["standard_normal"],
            seed=definition["seed"],
        )
    payload = _manifest_payload(root=root, prerequisites=prerequisite_snapshot)
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
        artifacts.resolve_relative(contract.NOISE_ROOT)
        if noise_root is None
        else Path(noise_root).absolute()
    )
    canonical = artifacts.resolve_relative(contract.NOISE_ROOT)
    if enforce_canonical_destination and root != canonical:
        raise V12R5Q3QualificationNoiseError(f"non-canonical noise root: {root}")
    payload = _strict_mapping(root / "manifest.json")
    expected_files = {"manifest.json", *contract.EXPECTED_TAPE_ARRAY_SHA256}
    if (
        not root.is_dir()
        or artifacts.is_link_or_reparse(root)
        or {path.name for path in root.iterdir()} != expected_files
    ):
        raise V12R5Q3QualificationNoiseError("noise root file set drifted")
    binding = contract.validate_candidate_binding(
        payload.get("candidate_id"), payload.get("candidate_module")
    )
    if (
        payload.get("status") != NOISE_TAPES_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("qualification_design_freeze") != DESIGN_FREEZE_RECORD
        or payload.get("case_order") != list(contract.CASE_IDS)
        or payload.get("candidate_id") != binding["candidate_id"]
        or payload.get("candidate_module") != binding["candidate_module"]
        or payload.get("update_activity")
        != {"actor_updates": 0, "critic_updates": 0, "ppo_updates": 0}
    ):
        raise V12R5Q3QualificationNoiseError("noise manifest semantics drifted")
    for filename, expected_hash in contract.EXPECTED_TAPE_ARRAY_SHA256.items():
        row = _mapping(payload.get("tapes")).get(filename)
        if not isinstance(row, Mapping):
            raise V12R5Q3QualificationNoiseError(f"missing tape row: {filename}")
        array = _read_tape(root / filename, seed=row.get("seed"))
        if (
            array.shape != TAPE_SHAPE
            or array.dtype != np.float32
            or not array.flags.c_contiguous
            or array_sha256(array) != expected_hash
            or row.get("array_sha256") != expected_hash
            or row.get("artifact") != artifacts.record(root / filename)
        ):
            raise V12R5Q3QualificationNoiseError(f"tape closure drifted: {filename}")
    contract.bind_candidate(binding["candidate_id"], binding["candidate_module"])
    return payload


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


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
            "status": "PASS_H0_V12R5_Q3_REFERENCE_TAPES_BUILD_ONLY",
            "passed": len(build_tapes()) == 5,
        }
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "DESIGN_FREEZE_RECORD",
    "NOISE_TAPES_PASS_STATUS",
    "V12R5Q3QualificationNoiseError",
    "array_sha256",
    "build_tapes",
    "prepare",
    "verified_design_binding",
    "verify_manifest",
]
