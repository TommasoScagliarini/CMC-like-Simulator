"""Freeze the executable Q3 closure after R5 PASS and tape verification."""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

import h0_v12r5_q3_artifacts as artifacts
import h0_v12r5_q3_qualification_gates as gates
import h0_v12r5_q3_runtime_contract as contract
import prepare_h0_v12r5_q3_qualification_noise_tapes as noise
import verify_h0_v12r5_q3_prerequisites as prerequisites_module


class V12R5Q3QualificationProtocolFreezeError(RuntimeError):
    """Raised whenever the Q3 executable closure is incomplete."""


PROTOCOL_FREEZE_PATH = artifacts.resolve_relative(contract.PROTOCOL_FREEZE_PATH)
EXECUTION_LOCK_PATH = artifacts.resolve_relative(contract.EXECUTION_LOCK_PATH)
RUN_ROOT = artifacts.resolve_relative(contract.RUN_ROOT)
NOISE_MANIFEST_PATH = artifacts.resolve_relative(contract.NOISE_MANIFEST_PATH)

RUNTIME_SOURCE_RELATIVE_PATHS = {
    "runtime_package": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/__init__.py"
    ),
    "artifact_helpers": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_artifacts.py"
    ),
    "runtime_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_runtime_contract.py"
    ),
    "r5_prerequisite_verifiers": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "verify_h0_v12r5_q3_prerequisites.py"
    ),
    "qualification_gates": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_qualification_gates.py"
    ),
    "noise_preparer": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "prepare_h0_v12r5_q3_qualification_noise_tapes.py"
    ),
    "protocol_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "freeze_h0_v12r5_q3_qualification_protocol.py"
    ),
    "physical_rollout": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_physical_rollout.py"
    ),
    "execution_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "run_h0_v12r5_q3_qualification.py"
    ),
    "runtime_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "test_h0_v12r5_q3_runtime.py"
    ),
    "q1_artifact_predicate_source": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "h0_v12r3_p1_qualification_gates.py"
    ),
    "r5_official_protocol_verifier": (
        "Trajectory Generator/baseline_MLP/validation/v12r5/"
        "freeze_h0_v12r5_case_balanced.py"
    ),
    "r5_official_runtime_verifier": (
        "Trajectory Generator/baseline_MLP/validation/v12r5/"
        "run_h0_v12r5_case_balanced.py"
    ),
    "r5_official_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12r5/"
        "h0_v12r5_case_balanced_contract.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v6_role_runtime": "validation/run_h0_primary_split_v6_qualification.py",
    "v26_environment_source": "validation/run_h0_primary_split_v9_causal_teacher.py",
    "v12r3_physical_runtime": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "primary_split_pairing": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "primary_split_v1_runtime": (
        "validation/run_h0_primary_grf_split_v1_adaptation.py"
    ),
    "primary_split_v3_runtime": (
        "validation/run_h0_primary_grf_split_v3_semantic_replay.py"
    ),
    "so_recovery_contract": "validation/h0_v3_so_recovery_contract.py",
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


def _tree_record(path: Path) -> dict[str, Any]:
    root = path.absolute()
    artifacts.assert_no_link_components(root)
    if not root.is_dir() or artifacts.is_link_or_reparse(root):
        raise V12R5Q3QualificationProtocolFreezeError(
            f"artifact tree is missing/unsafe: {root}"
        )
    entries = sorted(root.rglob("*"), key=lambda item: item.as_posix())
    if any(artifacts.is_link_or_reparse(item) for item in entries):
        raise V12R5Q3QualificationProtocolFreezeError(
            f"artifact tree contains link/reparse: {root}"
        )
    files = [item for item in entries if item.is_file()]
    rows: list[dict[str, Any]] = []
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        item_record = artifacts.record(item)
        row = {
            "path": relative,
            "sha256": item_record["sha256"],
            "size_bytes": item_record["size_bytes"],
        }
        rows.append(row)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(row["sha256"].encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    if not rows:
        raise V12R5Q3QualificationProtocolFreezeError("artifact tree is empty")
    return {
        "path": artifacts.portable_path(root),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _source_gate() -> dict[str, Any]:
    records = {
        name: artifacts.record(artifacts.resolve_relative(path))
        for name, path in RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }
    paths = [row["path"] for row in records.values()]
    checks = {
        "closed_twenty_two_source_list": len(records) == 22
        and list(records) == list(RUNTIME_SOURCE_RELATIVE_PATHS),
        "paths_exact_unique": paths == list(RUNTIME_SOURCE_RELATIVE_PATHS.values())
        and len(set(paths)) == 22,
        "hashes_sizes_valid": all(
            isinstance(row.get("sha256"), str)
            and len(row["sha256"]) == 64
            and type(row.get("size_bytes")) is int
            and row["size_bytes"] > 0
            for row in records.values()
        ),
        "q1_reuse_limited_to_artifact_predicate": set(records)
        & {"q1_runner", "q1_rollout_gates", "q1_pair_gates", "q1_aggregate_gate"}
        == set(),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


def _input_gate(binding: Mapping[str, Any] | None = None) -> dict[str, Any]:
    selected = (
        contract.current_candidate_binding() if binding is None else dict(binding)
    )
    candidate = _tree_record(
        artifacts.resolve_relative(selected["candidate_module"]["path"])
    )
    source_h0 = _tree_record(
        artifacts.resolve_relative(contract.SOURCE_H0_MODULE["path"])
    )
    records = {
        "candidate_module": candidate,
        "source_h0_module": source_h0,
        "source_h0_config": artifacts.record(
            artifacts.resolve_relative(SOURCE_H0_CONFIG_PATH)
        ),
        "historical_analog_profile": artifacts.record(
            artifacts.resolve_relative(ANALOG_PROFILE_PATH)
        ),
        "baseline_shadow_v25_profile": artifacts.record(
            artifacts.resolve_relative(BASELINE_SHADOW_V25_PROFILE_PATH)
        ),
    }
    checks = {
        "candidate_tree_exact": candidate == selected["candidate_module"],
        "source_h0_exact": source_h0.get("path") == contract.SOURCE_H0_MODULE["path"]
        and source_h0.get("tree_sha256") == contract.SOURCE_H0_MODULE["tree_sha256"],
        "supporting_inputs_bound": all(
            records[name]["size_bytes"] > 0
            for name in (
                "source_h0_config",
                "historical_analog_profile",
                "baseline_shadow_v25_profile",
            )
        ),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": records}


def _prerequisite_snapshot() -> dict[str, Any]:
    try:
        snapshot = prerequisites_module.load_and_verify_r5_prerequisites(
            require_q3_unopened=False
        )
    except BaseException as exc:
        raise V12R5Q3QualificationProtocolFreezeError(
            "official R5 prerequisite closure is not eligible"
        ) from exc
    pure_gate = gates.r5_prerequisite_gate(snapshot["gate"])
    if pure_gate.get("passed") is not True:
        raise V12R5Q3QualificationProtocolFreezeError(
            "independent Q3 prerequisite gate failed"
        )
    result = copy.deepcopy(snapshot)
    result["pure_gate"] = pure_gate
    return result


def _noise_gate() -> dict[str, Any]:
    try:
        manifest = noise.verify_manifest()
    except BaseException as exc:
        raise V12R5Q3QualificationProtocolFreezeError(
            "canonical Q3 noise closure is not verified"
        ) from exc
    binding = contract.current_candidate_binding()
    tapes = {
        filename: {
            "artifact": artifacts.record(
                artifacts.resolve_relative(contract.NOISE_ROOT / filename)
            ),
            "array_sha256": row["array_sha256"],
            "seed": row["seed"],
        }
        for filename, row in manifest["tapes"].items()
    }
    checks = {
        "manifest_pass": manifest.get("status") == noise.NOISE_TAPES_PASS_STATUS
        and manifest.get("passed") is True,
        "candidate_exact": manifest.get("candidate_id") == binding["candidate_id"]
        and manifest.get("candidate_module") == binding["candidate_module"],
        "five_tapes": len(tapes) == 5,
        "six_cases": manifest.get("case_order") == list(contract.CASE_IDS),
        "pair_tape_shared": manifest.get("same_tape_required_for_condition_pair")
        is True,
        "zero_update_accounting": manifest.get("update_activity")
        == {"actor_updates": 0, "critic_updates": 0, "ppo_updates": 0},
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "manifest": manifest,
        "manifest_record": artifacts.record(NOISE_MANIFEST_PATH),
        "tapes": tapes,
    }


def _occupancy_snapshot() -> dict[str, bool]:
    return {
        "protocol_freeze_unoccupied": not os.path.lexists(PROTOCOL_FREEZE_PATH),
        "execution_lock_absent": not os.path.lexists(EXECUTION_LOCK_PATH),
        "qualification_run_root_absent": not os.path.lexists(RUN_ROOT),
        "noise_manifest_present": NOISE_MANIFEST_PATH.is_file()
        and not artifacts.is_link_or_reparse(NOISE_MANIFEST_PATH),
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
        raise V12R5Q3QualificationProtocolFreezeError("occupancy schema drifted")
    design = noise.verified_design_binding()
    prerequisites = _prerequisite_snapshot()
    binding = prerequisites["gate"]
    contract.bind_candidate(binding["candidate_id"], binding["candidate_module"])
    noise_closure = _noise_gate()
    sources = _source_gate()
    inputs = _input_gate(binding)
    snapshot = design["payload"]["design_snapshot"]
    checks = {
        "schema_140": contract.SCHEMA_VERSION == 140,
        "design_freeze_exact": design["record"] == noise.DESIGN_FREEZE_RECORD,
        "five_r5_official_verifiers_pass": binding.get("passed") is True
        and prerequisites["pure_gate"].get("passed") is True,
        "same_exact_r5_candidate": binding.get("candidate_id")
        == contract.R5_CANDIDATE_ID
        and binding.get("candidate_module") == contract.R5_CANDIDATE_MODULE,
        "noise_closure_verified": noise_closure["passed"] is True,
        "runtime_source_closure_bound": sources["passed"] is True,
        "runtime_input_closure_bound": inputs["passed"] is True,
        "six_holdouts_matrix_unchanged": snapshot.get("holdout_cases")
        == list(contract.HOLDOUT_CASES)
        and snapshot.get("rollout_matrix") == list(contract.ROLLOUT_MATRIX),
        "baseline_six_then_candidate_six": [
            (row["role"], row["case_id"]) for row in contract.ROLLOUT_MATRIX
        ]
        == [
            (role, case_id)
            for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
            for case_id in contract.CASE_IDS
        ],
        "zero_freeze_activity": all(
            value == 0 for value in ZERO_FREEZE_ACTIVITY.values()
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
        "freeze_kind": "POST_R5_TERMINAL_PASS_INDEPENDENT_Q3_PROTOCOL",
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "qualification_design_freeze": copy.deepcopy(design["record"]),
        "qualification_design_snapshot": copy.deepcopy(snapshot),
        "selected_candidate_id": binding["candidate_id"],
        "selected_candidate": copy.deepcopy(binding["candidate_module"]),
        "r5_prerequisite_gate": copy.deepcopy(binding),
        "r5_pure_prerequisite_gate": copy.deepcopy(prerequisites["pure_gate"]),
        "r5_prerequisite_records": copy.deepcopy(prerequisites["records"]),
        "noise_manifest": copy.deepcopy(noise_closure["manifest_record"]),
        "noise_tapes": copy.deepcopy(noise_closure["tapes"]),
        "runtime_sources": copy.deepcopy(sources["records"]),
        "runtime_source_gate": {
            "passed": sources["passed"],
            "checks": sources["checks"],
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
        "checkpoint_zero_created": False,
        "next_stage": "PREPARE_NO_CLOBBER_Q3_EXECUTION_LOCK",
    }


def build_protocol_freeze() -> dict[str, Any]:
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
        raise V12R5Q3QualificationProtocolFreezeError(
            f"non-canonical protocol freeze destination: {destination}"
        )
    if os.path.lexists(destination):
        raise V12R5Q3QualificationProtocolFreezeError(
            f"refusing to clobber: {destination}"
        )
    payload = build_protocol_freeze()
    if payload.get("passed") is not True:
        raise V12R5Q3QualificationProtocolFreezeError("protocol freeze checks failed")
    import h0_forensic_rollout as forensic

    forensic.write_json_exclusive(destination, payload)
    return verify_protocol_freeze(
        input_path=destination,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def verify_protocol_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    import h0_forensic_rollout as forensic

    destination = (
        PROTOCOL_FREEZE_PATH if input_path is None else Path(input_path).absolute()
    )
    if enforce_canonical_destination and destination != PROTOCOL_FREEZE_PATH:
        raise V12R5Q3QualificationProtocolFreezeError(
            f"non-canonical protocol freeze: {destination}"
        )
    try:
        observed = forensic.strict_json_load(destination)
    except BaseException as exc:
        raise V12R5Q3QualificationProtocolFreezeError(
            "protocol freeze is not strict JSON"
        ) from exc
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
        raise V12R5Q3QualificationProtocolFreezeError("protocol freeze drifted")
    return dict(observed)


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
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "RUNTIME_SOURCE_RELATIVE_PATHS",
    "V12R5Q3QualificationProtocolFreezeError",
    "build_protocol_freeze",
    "publish_protocol_freeze",
    "verify_protocol_freeze",
]
