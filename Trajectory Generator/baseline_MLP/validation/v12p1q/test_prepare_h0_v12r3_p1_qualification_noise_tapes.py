"""Tests for the prospective no-clobber V12P1Q tape preparer."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest


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

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import prepare_h0_v12r3_p1_qualification_noise_tapes as noise  # noqa: E402


def _record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.absolute()),
        "sha256": forensic.sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _write(path: Path, payload: dict[str, Any]) -> None:
    forensic.write_json_exclusive(path, payload)


def _fake_prerequisites(tmp_path: Path) -> dict[str, Path]:
    root = tmp_path / "salvage"
    root.mkdir()
    design = copy.deepcopy(noise.DESIGN_FREEZE_RECORD)
    authority = {"runtime_promotion_authorized": False}

    protocol_path = root / "protocol.json"
    _write(
        protocol_path,
        {
            "status": contract.SALVAGE_PROTOCOL_FREEZE_PASS_STATUS,
            "passed": True,
            "selected_candidate_id": contract.P1_CANDIDATE_ID,
            "selected_candidate": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
            "qualification_design_freeze": design,
            "runtime_promoted": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )
    protocol_record = _record(protocol_path)

    lock_path = root / "lock.json"
    _write(
        lock_path,
        {
            "status": contract.SALVAGE_EXECUTION_LOCK_PASS_STATUS,
            "passed": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
            "qualification_design_freeze": design,
            "protocol_freeze": protocol_record,
            "authority": authority,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )
    lock_record = _record(lock_path)

    final_path = root / "final_receipt.json"
    _write(
        final_path,
        {
            "status": contract.SALVAGE_FINAL_DEVELOPMENT_PASS_STATUS,
            "passed": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "qualification_design_freeze": design,
            "stage_id": contract.SALVAGE_STAGE_IDS[-1],
            "case_count": 6,
            "case_receipts": [
                {
                    "path": f"development/case_{index}/receipt.json",
                    "sha256": f"{index + 1:064x}",
                    "size_bytes": 1,
                }
                for index in range(6)
            ],
            "execution_authority": authority,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )
    final_record = _record(final_path)

    ledger_path = root / "ledger.json"
    completed = [
        {
            "stage_id": stage,
            "receipt": {"path": stage, "sha256": "a" * 64, "size_bytes": 1},
        }
        for stage in contract.SALVAGE_STAGE_IDS[:-1]
    ]
    completed.append(
        {
            "stage_id": contract.SALVAGE_STAGE_IDS[-1],
            "receipt": final_record,
        }
    )
    _write(
        ledger_path,
        {
            "status": contract.SALVAGE_PIPELINE_PASS_STATUS,
            "passed": True,
            "terminal": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
            "qualification_design_freeze": design,
            "protocol_freeze": protocol_record,
            "execution_lock": lock_record,
            "stage_order": list(contract.SALVAGE_STAGE_IDS),
            "attempted_stage": None,
            "completed_stages": list(contract.SALVAGE_STAGE_IDS),
            "completed_receipts": completed,
            "failed_stage_receipt": None,
            "aggregate_requires_6_of_6": True,
            "compensation_authorized": False,
            "retry_authorized": False,
            "resume_authorized": False,
            "runtime_promoted": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )
    return {
        "salvage_protocol_freeze": protocol_path,
        "salvage_execution_lock": lock_path,
        "salvage_final_development_receipt": final_path,
        "salvage_terminal_pass_ledger": ledger_path,
    }


def test_design_binding_is_the_published_immutable_freeze() -> None:
    binding = noise._verified_design_binding()
    assert binding["record"] == {
        "path": contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix(),
        "sha256": "c64928cb85df0e1f5d53c5f2e6eba52172b7c831ccf712a0113522bcff4ef686",
        "size_bytes": 21_303,
    }
    assert binding["payload"]["passed"] is True
    assert binding["payload"]["design_snapshot"]["holdout_cases"] == list(
        contract.HOLDOUT_CASES
    )


def test_exact_float32_tapes_are_reproducible_and_source_frozen() -> None:
    expected_hashes = {
        "deterministic_all_zero.npz": (
            "ab89c5ecd7d818ab19f726cffc9ce431f5889448c7a79f84927f7153e546782c"
        ),
        "stochastic_seed_130_standard_normal.npz": (
            "067a5fb858ba1a5365856367eb1de793d954c9cbc07fa1aaaed34520a08657fa"
        ),
        "stochastic_seed_131_standard_normal.npz": (
            "00ea968882002a0881451cc2d80a365b2f6a3ccbfd698fcbe3768ca572abae67"
        ),
        "stochastic_seed_132_standard_normal.npz": (
            "9905eddb8074676cfe1ac2feeee152d114e9532b8cee9844daad016fa4930b61"
        ),
        "stochastic_seed_133_standard_normal.npz": (
            "d25e7515c993742e24da93f01313348991ea35871f98dbf07cd767367ec44438"
        ),
    }
    first = noise.build_tapes()
    second = noise.build_tapes()
    assert list(first) == list(expected_hashes)
    for name, expected_hash in expected_hashes.items():
        array = first[name]["standard_normal"]
        assert array.shape == (500, 2)
        assert array.dtype == np.float32
        assert array.flags.c_contiguous
        assert array.tobytes() == second[name]["standard_normal"].tobytes()
        assert noise.array_sha256(array) == expected_hash
    assert np.count_nonzero(first["deterministic_all_zero.npz"]["standard_normal"]) == 0


def test_four_prerequisites_must_be_terminal_same_p1_and_six_of_six(
    tmp_path: Path,
) -> None:
    paths = _fake_prerequisites(tmp_path)
    binding = noise.load_and_validate_prerequisites(prerequisite_paths=paths)
    assert binding["gate"]["passed"] is True
    assert binding["gate"]["frozen_gate"]["passed"] is True
    assert binding["gate"]["checks"]["frozen_prerequisite_gate_pass"] is True
    assert binding["gate"]["candidate_id"] == contract.P1_CANDIDATE_ID
    assert [row["name"] for row in binding["gate"]["rows"]] == list(paths)

    ledger = forensic.strict_json_load(paths["salvage_terminal_pass_ledger"])
    ledger["completed_stages"] = ledger["completed_stages"][:-1]
    paths["salvage_terminal_pass_ledger"].write_bytes(
        forensic.canonical_json_bytes(ledger)
    )
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="not eligible"):
        noise.load_and_validate_prerequisites(prerequisite_paths=paths)


def test_prerequisite_binding_refuses_an_opened_qualification(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    paths = _fake_prerequisites(tmp_path)
    occupied_run = tmp_path / "qualification-already-open"
    occupied_run.mkdir()
    original_resolve = noise.resolve_relative

    def redirected(value: Any) -> Path:
        if value == contract.RUN_ROOT:
            return occupied_run
        return original_resolve(value)

    monkeypatch.setattr(noise, "resolve_relative", redirected)
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="must be unopened"):
        noise.load_and_validate_prerequisites(prerequisite_paths=paths)


def test_prepare_and_verify_temp_root_is_no_clobber_and_pair_shared(
    tmp_path: Path,
) -> None:
    paths = _fake_prerequisites(tmp_path)
    output = tmp_path / "noise"
    manifest = noise.prepare(
        output_root=output,
        prerequisite_paths=paths,
        enforce_canonical_destination=False,
    )
    assert manifest["status"] == noise.NOISE_TAPES_PASS_STATUS
    assert manifest["passed"] is True
    assert manifest["candidate_id"] == contract.P1_CANDIDATE_ID
    assert manifest["case_order"] == list(contract.CASE_IDS)
    assert list(manifest["tapes"]) == [
        "deterministic_all_zero.npz",
        "stochastic_seed_130_standard_normal.npz",
        "stochastic_seed_131_standard_normal.npz",
        "stochastic_seed_132_standard_normal.npz",
        "stochastic_seed_133_standard_normal.npz",
    ]
    assert manifest["cases"][0]["noise_tape_array_sha256"] == (
        manifest["cases"][1]["noise_tape_array_sha256"]
    )
    assert (
        noise.verify_manifest(
            output_root=output,
            prerequisite_paths=paths,
            enforce_canonical_destination=False,
        )
        == manifest
    )
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="clobber"):
        noise.prepare(
            output_root=output,
            prerequisite_paths=paths,
            enforce_canonical_destination=False,
        )
    assert not noise.DEFAULT_OUTPUT_ROOT.exists()


def test_bad_prerequisite_stops_before_noise_root_creation(tmp_path: Path) -> None:
    paths = _fake_prerequisites(tmp_path)
    ledger_path = paths["salvage_terminal_pass_ledger"]
    ledger = forensic.strict_json_load(ledger_path)
    ledger["status"] = "FAIL"
    ledger_path.write_bytes(forensic.canonical_json_bytes(ledger))
    paths["salvage_final_development_receipt"].unlink()
    output = tmp_path / "must_not_exist"
    with pytest.raises(
        noise.V12R3P1QualificationNoiseError,
        match="not eligible: ledger status='FAIL'",
    ):
        noise.prepare(
            output_root=output,
            prerequisite_paths=paths,
            enforce_canonical_destination=False,
        )
    assert not output.exists()


def test_tape_or_manifest_drift_fails_verification(tmp_path: Path) -> None:
    paths = _fake_prerequisites(tmp_path)
    output = tmp_path / "noise"
    noise.prepare(
        output_root=output,
        prerequisite_paths=paths,
        enforce_canonical_destination=False,
    )
    target = output / "stochastic_seed_130_standard_normal.npz"
    target.write_bytes(b"not-an-npz")
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="cannot read tape"):
        noise.verify_manifest(
            output_root=output,
            prerequisite_paths=paths,
            enforce_canonical_destination=False,
        )


def test_noncanonical_root_requires_explicit_override(tmp_path: Path) -> None:
    paths = _fake_prerequisites(tmp_path)
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="non-canonical"):
        noise.prepare(output_root=tmp_path / "noise", prerequisite_paths=paths)


def test_symlink_or_windows_reparse_component_fails_closed(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation unavailable")
    with pytest.raises(noise.V12R3P1QualificationNoiseError, match="link/reparse"):
        noise._assert_no_link_components(link / "tape.npz")


def test_import_and_build_do_not_materialize_canonical_noise() -> None:
    before = os.path.lexists(noise.DEFAULT_OUTPUT_ROOT)
    noise.build_tapes()
    assert before is False
    assert not os.path.lexists(noise.DEFAULT_OUTPUT_ROOT)
