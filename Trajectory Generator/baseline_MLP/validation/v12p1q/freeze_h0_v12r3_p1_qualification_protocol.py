"""Freeze the executable V12R3-P1 independent qualification protocol.

The frozen design is an immutable pre-development input.  This later protocol
freeze may be built only after all four V12P1S artifacts are terminal PASS and
the five deterministic V6 tapes have been materialized and verified.  Import
and build are read-only; publication is explicit, atomic, and no-clobber.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
import stat
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


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
V12R3_ROOT = LOCAL_VALIDATION / "v12r3"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    ROOT_VALIDATION,
    LOCAL_VALIDATION,
    V12R3_ROOT,
    V12P1Q_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import prepare_h0_v12r3_p1_qualification_noise_tapes as noise  # noqa: E402


class V12R3P1QualificationProtocolFreezeError(RuntimeError):
    """Raised whenever the executable qualification closure is incomplete."""


PROTOCOL_FREEZE_PATH = REPO_ROOT.joinpath(*contract.PROTOCOL_FREEZE_PATH.parts)
EXECUTION_LOCK_PATH = REPO_ROOT.joinpath(*contract.EXECUTION_LOCK_PATH.parts)
RUN_ROOT = REPO_ROOT.joinpath(*contract.RUN_ROOT.parts)
NOISE_MANIFEST_PATH = REPO_ROOT.joinpath(*contract.NOISE_MANIFEST_PATH.parts)

RUNTIME_SOURCE_RELATIVE_PATHS = {
    "noise_preparer": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "prepare_h0_v12r3_p1_qualification_noise_tapes.py"
    ),
    "protocol_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "freeze_h0_v12r3_p1_qualification_protocol.py"
    ),
    "execution_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "run_h0_v12r3_p1_qualification.py"
    ),
    "noise_preparer_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "test_prepare_h0_v12r3_p1_qualification_noise_tapes.py"
    ),
    "protocol_freezer_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "test_freeze_h0_v12r3_p1_qualification_protocol.py"
    ),
    "execution_runner_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "test_run_h0_v12r3_p1_qualification.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v6_role_runtime": "validation/run_h0_primary_split_v6_qualification.py",
    "v26_environment_source": "validation/run_h0_primary_split_v9_causal_teacher.py",
    "v12r3_physical_runtime": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
}

SOURCE_H0_CONFIG_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
    "training_cfg.resolved.yaml"
)
ANALOG_PROFILE_PATH = PurePosixPath(
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
BASELINE_SHADOW_V25_PROFILE_PATH = PurePosixPath(
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)

ZERO_FREEZE_ACTIVITY = {
    "environment_imports": 0,
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "rollout_executions": 0,
    "actor_fit_executions": 0,
    "offline_teacher_label_calls": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}


def _raw_relative(value: str | os.PathLike[str] | PurePosixPath) -> str:
    return value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = _raw_relative(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R3P1QualificationProtocolFreezeError(
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
    attribute = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(
        getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0) and attribute & reparse
    )


def _assert_no_link_components(path: Path) -> None:
    cursor = path.absolute()
    while True:
        if os.path.lexists(cursor) and _is_link_or_reparse(cursor):
            raise V12R3P1QualificationProtocolFreezeError(
                f"path contains link/reparse component: {cursor}"
            )
        if cursor == cursor.parent:
            break
        cursor = cursor.parent


def _record(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R3P1QualificationProtocolFreezeError(
            f"artifact is not a safe regular file: {source}"
        )
    try:
        relative = source.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            f"artifact escaped repository: {source}"
        ) from exc
    return {
        "path": relative,
        "sha256": forensic.sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def _mapping(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    try:
        payload = forensic.strict_json_load(source)
    except Exception as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            f"cannot read strict JSON: {source}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R3P1QualificationProtocolFreezeError(f"expected JSON object: {source}")
    result = dict(payload)
    if source.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R3P1QualificationProtocolFreezeError(
            f"JSON artifact is not byte-canonical: {source}"
        )
    return result


def _tree_record(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    root = Path(path).absolute()
    _assert_no_link_components(root)
    if not root.is_dir() or _is_link_or_reparse(root):
        raise V12R3P1QualificationProtocolFreezeError(
            f"artifact tree is missing/unsafe: {root}"
        )
    entries = sorted(root.rglob("*"), key=lambda item: item.as_posix())
    if any(_is_link_or_reparse(item) for item in entries):
        raise V12R3P1QualificationProtocolFreezeError(
            f"artifact tree contains link/reparse: {root}"
        )
    files = [item for item in entries if item.is_file()]
    if not files:
        raise V12R3P1QualificationProtocolFreezeError(f"artifact tree is empty: {root}")
    digest = hashlib.sha256()
    rows = []
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
    try:
        portable = root.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            f"artifact tree escaped repository: {root}"
        ) from exc
    return {
        "path": portable,
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _runtime_source_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(path))
        for name, path in RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }


def _source_gate() -> dict[str, Any]:
    records = _runtime_source_records()
    checks = {
        "closed_ten_source_list": list(records) == list(RUNTIME_SOURCE_RELATIVE_PATHS)
        and len(records) == 10,
        "paths_exact_and_unique": [row["path"] for row in records.values()]
        == list(RUNTIME_SOURCE_RELATIVE_PATHS.values())
        and len({row["path"] for row in records.values()}) == 10,
        "all_hashes_and_sizes_valid": all(
            isinstance(row.get("sha256"), str)
            and len(row["sha256"]) == 64
            and type(row.get("size_bytes")) is int
            and row["size_bytes"] > 0
            for row in records.values()
        ),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


def _input_gate() -> dict[str, Any]:
    candidate = _tree_record(resolve_relative(contract.P1_CANDIDATE_MODULE["path"]))
    source_h0 = _tree_record(resolve_relative(contract.SOURCE_H0_MODULE["path"]))
    source_h0_config = _record(resolve_relative(SOURCE_H0_CONFIG_PATH))
    analog_profile = _record(resolve_relative(ANALOG_PROFILE_PATH))
    baseline_shadow = _record(resolve_relative(BASELINE_SHADOW_V25_PROFILE_PATH))
    checks = {
        "candidate_p1_tree_exact": candidate == contract.P1_CANDIDATE_MODULE,
        "source_h0_tree_exact": source_h0.get("path")
        == contract.SOURCE_H0_MODULE["path"]
        and source_h0.get("tree_sha256") == contract.SOURCE_H0_MODULE["tree_sha256"],
        "source_h0_config_bound": source_h0_config["size_bytes"] > 0,
        "historical_analog_profile_bound": analog_profile["size_bytes"] > 0,
        "baseline_shadow_profile_bound": baseline_shadow["size_bytes"] > 0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "records": {
            "candidate_module": candidate,
            "source_h0_module": source_h0,
            "source_h0_config": source_h0_config,
            "historical_analog_profile": analog_profile,
            "baseline_shadow_v25_profile": baseline_shadow,
        },
    }


def _noise_gate() -> dict[str, Any]:
    try:
        manifest = noise.verify_manifest()
    except Exception as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            "canonical qualification noise closure is not verified"
        ) from exc
    tape_records: dict[str, Any] = {}
    for filename, definition in manifest["tapes"].items():
        path = resolve_relative(contract.NOISE_ROOT / filename)
        observed = _record(path)
        expected = copy.deepcopy(definition["artifact"])
        expected["path"] = (contract.NOISE_ROOT / filename).as_posix()
        if observed != expected:
            raise V12R3P1QualificationProtocolFreezeError(
                f"noise artifact closure drifted: {filename}"
            )
        tape_records[filename] = {
            "artifact": observed,
            "array_sha256": definition["array_sha256"],
            "seed": definition["seed"],
        }
    checks = {
        "manifest_pass": manifest.get("status") == noise.NOISE_TAPES_PASS_STATUS
        and manifest.get("passed") is True,
        "same_candidate": manifest.get("candidate_id") == contract.P1_CANDIDATE_ID
        and manifest.get("candidate_module_tree_sha256")
        == contract.P1_CANDIDATE_MODULE["tree_sha256"],
        "five_tapes": set(tape_records)
        == {
            "deterministic_all_zero.npz",
            *{
                f"stochastic_seed_{seed}_standard_normal.npz"
                for seed in contract.STOCHASTIC_SEEDS
            },
        },
        "six_cases": manifest.get("case_order") == list(contract.CASE_IDS)
        and len(manifest.get("cases", [])) == 6,
        "same_tape_pair_required": manifest.get("same_tape_required_for_condition_pair")
        is True,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "manifest": manifest,
        "manifest_record": _record(NOISE_MANIFEST_PATH),
        "tapes": tape_records,
    }


def _prerequisite_gate() -> dict[str, Any]:
    try:
        binding = noise.load_and_validate_prerequisites(
            require_qualification_unopened=False
        )
    except Exception as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            "V12P1S terminal prerequisite closure is not eligible"
        ) from exc
    return binding


def _occupancy_snapshot() -> dict[str, bool]:
    return {
        "protocol_freeze_unoccupied": not os.path.lexists(PROTOCOL_FREEZE_PATH),
        "execution_lock_absent": not os.path.lexists(EXECUTION_LOCK_PATH),
        "qualification_run_root_absent": not os.path.lexists(RUN_ROOT),
        "noise_manifest_present": os.path.isfile(NOISE_MANIFEST_PATH)
        and not _is_link_or_reparse(NOISE_MANIFEST_PATH),
    }


def _assemble_protocol_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    expected_occupancy = {
        "protocol_freeze_unoccupied",
        "execution_lock_absent",
        "qualification_run_root_absent",
        "noise_manifest_present",
    }
    if set(occupancy) != expected_occupancy or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12R3P1QualificationProtocolFreezeError("occupancy schema drifted")
    design = noise._verified_design_binding()
    prerequisites = _prerequisite_gate()
    noise_closure = _noise_gate()
    source = _source_gate()
    inputs = _input_gate()
    frozen_snapshot = design["payload"]["design_snapshot"]
    checks = {
        "schema_125": contract.SCHEMA_VERSION == 125,
        "design_freeze_canonical_exact": design["record"] == noise.DESIGN_FREEZE_RECORD,
        "four_salvage_prerequisites_terminal_pass": prerequisites["gate"].get("passed")
        is True,
        "same_exact_p1": prerequisites["gate"].get("candidate_id")
        == contract.P1_CANDIDATE_ID
        and prerequisites["gate"].get("candidate_module")
        == contract.P1_CANDIDATE_MODULE,
        "noise_closure_verified": noise_closure["passed"] is True,
        "runtime_source_closure_bound": source["passed"] is True,
        "runtime_input_closure_bound": inputs["passed"] is True,
        "six_holdouts_and_matrix_unchanged": frozen_snapshot.get("holdout_cases")
        == list(contract.HOLDOUT_CASES)
        and frozen_snapshot.get("rollout_matrix") == list(contract.ROLLOUT_MATRIX),
        "baseline_first_then_candidate": [
            (row["role"], row["case_id"]) for row in contract.ROLLOUT_MATRIX
        ]
        == [
            (role, case_id)
            for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
            for case_id in contract.CASE_IDS
        ],
        "zero_freeze_activity": all(
            type(value) is int and value == 0 for value in ZERO_FREEZE_ACTIVITY.values()
        ),
        **dict(occupancy),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_FREEZE_PASS_STATUS
            if passed
            else contract.PROTOCOL_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "freeze_kind": "POST_SALVAGE_INDEPENDENT_QUALIFICATION_PROTOCOL",
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "qualification_design_freeze": copy.deepcopy(design["record"]),
        "qualification_design_snapshot": copy.deepcopy(frozen_snapshot),
        "selected_candidate_id": contract.P1_CANDIDATE_ID,
        "selected_candidate": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "salvage_prerequisite_gate": copy.deepcopy(prerequisites["gate"]),
        "salvage_prerequisite_records": copy.deepcopy(prerequisites["records"]),
        "salvage_terminal_payload_bindings": {
            name: {
                "status": payload.get("status"),
                "passed": payload.get("passed"),
                "candidate_id": (
                    payload.get("selected_candidate_id")
                    if name == "salvage_protocol_freeze"
                    else payload.get("candidate_id")
                ),
            }
            for name, payload in prerequisites["payloads"].items()
        },
        "noise_manifest": copy.deepcopy(noise_closure["manifest_record"]),
        "noise_tapes": copy.deepcopy(noise_closure["tapes"]),
        "runtime_sources": copy.deepcopy(source["records"]),
        "runtime_source_gate": {
            "passed": source["passed"],
            "checks": source["checks"],
        },
        "runtime_inputs": copy.deepcopy(inputs["records"]),
        "runtime_input_gate": {
            "passed": inputs["passed"],
            "checks": inputs["checks"],
        },
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "stage_order": list(contract.STAGE_IDS),
        "pairwise_tolerances": {
            "reserve": [list(row) for row in contract.RESERVE_TOLERANCES],
            "sea": [list(row) for row in contract.SEA_TOLERANCES],
        },
        "zero_freeze_activity": copy.deepcopy(ZERO_FREEZE_ACTIVITY),
        "execution_lock": None,
        "pipeline_claim": None,
        "runtime_promoted": False,
        "next_stage": "PREPARE_NO_CLOBBER_QUALIFICATION_EXECUTION_LOCK",
    }


def build_protocol_freeze() -> dict[str, Any]:
    """Build the post-salvage executable closure in memory."""

    return _assemble_protocol_freeze(_occupancy_snapshot())


def publish_protocol_freeze(
    *,
    output_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    destination = (
        PROTOCOL_FREEZE_PATH if output_path is None else Path(output_path).absolute()
    )
    if enforce_canonical_destination and destination != PROTOCOL_FREEZE_PATH:
        raise V12R3P1QualificationProtocolFreezeError(
            f"non-canonical protocol freeze destination: {destination}"
        )
    _assert_no_link_components(destination)
    if os.path.lexists(destination):
        raise V12R3P1QualificationProtocolFreezeError(
            f"refusing to clobber: {destination}"
        )
    payload = build_protocol_freeze()
    if payload.get("passed") is not True:
        failed = [
            name for name, value in payload["checks"].items() if value is not True
        ]
        raise V12R3P1QualificationProtocolFreezeError(
            f"protocol freeze checks failed: {failed}"
        )
    try:
        forensic.write_json_exclusive(destination, payload)
    except Exception as exc:
        raise V12R3P1QualificationProtocolFreezeError(
            f"protocol freeze publication failed: {destination}"
        ) from exc
    return verify_protocol_freeze(
        input_path=destination,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def verify_protocol_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    destination = (
        PROTOCOL_FREEZE_PATH if input_path is None else Path(input_path).absolute()
    )
    if enforce_canonical_destination and destination != PROTOCOL_FREEZE_PATH:
        raise V12R3P1QualificationProtocolFreezeError(
            f"non-canonical protocol freeze: {destination}"
        )
    if not destination.is_file() or _is_link_or_reparse(destination):
        raise V12R3P1QualificationProtocolFreezeError(
            f"protocol freeze is not a safe regular file: {destination}"
        )
    observed = _mapping(destination)
    expected = _assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "qualification_run_root_absent": True,
            "noise_manifest_present": True,
        }
    )
    if (
        observed != expected
        or destination.read_bytes() != forensic.canonical_json_bytes(expected)
    ):
        raise V12R3P1QualificationProtocolFreezeError("protocol freeze drifted")
    return observed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--publish", action="store_true")
    action.add_argument("--verify", action="store_true")
    action.add_argument("--build-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.publish:
        payload = publish_protocol_freeze()
    elif args.verify:
        payload = verify_protocol_freeze()
    else:
        payload = build_protocol_freeze()
    print(f"{payload['status']}: {contract.PROTOCOL_FREEZE_PATH.as_posix()}")
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "RUNTIME_SOURCE_RELATIVE_PATHS",
    "V12R3P1QualificationProtocolFreezeError",
    "ZERO_FREEZE_ACTIVITY",
    "build_protocol_freeze",
    "publish_protocol_freeze",
    "verify_protocol_freeze",
]
