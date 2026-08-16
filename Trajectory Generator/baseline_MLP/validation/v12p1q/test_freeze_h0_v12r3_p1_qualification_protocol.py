"""Tests for the prospective post-salvage V12P1Q protocol freezer."""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path
from typing import Any

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

import freeze_h0_v12r3_p1_qualification_protocol as freeze  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import prepare_h0_v12r3_p1_qualification_noise_tapes as noise  # noqa: E402


def _artifact(path: str) -> dict[str, Any]:
    return {"path": path, "sha256": "a" * 64, "size_bytes": 1}


def _fake_prerequisite_binding() -> dict[str, Any]:
    names = [row["name"] for row in contract.FUTURE_PREREQUISITE_REQUIREMENTS]
    records = {
        name: _artifact(requirement["path"])
        for name, requirement in zip(
            names, contract.FUTURE_PREREQUISITE_REQUIREMENTS, strict=True
        )
    }
    payloads = {
        name: {
            "status": requirement["required_status"],
            "passed": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
        }
        for name, requirement in zip(
            names, contract.FUTURE_PREREQUISITE_REQUIREMENTS, strict=True
        )
    }
    payloads["salvage_protocol_freeze"][
        "selected_candidate_id"
    ] = contract.P1_CANDIDATE_ID
    return {
        "gate": {
            "passed": True,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        },
        "records": records,
        "payloads": payloads,
    }


def _fake_noise_gate() -> dict[str, Any]:
    tapes = {
        "deterministic_all_zero.npz": {
            "artifact": _artifact(
                f"{contract.NOISE_ROOT.as_posix()}/deterministic_all_zero.npz"
            ),
            "array_sha256": "b" * 64,
            "seed": None,
        },
        **{
            f"stochastic_seed_{seed}_standard_normal.npz": {
                "artifact": _artifact(
                    f"{contract.NOISE_ROOT.as_posix()}/"
                    f"stochastic_seed_{seed}_standard_normal.npz"
                ),
                "array_sha256": f"{seed:064x}"[-64:],
                "seed": seed,
            }
            for seed in contract.STOCHASTIC_SEEDS
        },
    }
    return {
        "passed": True,
        "checks": {"all": True},
        "manifest": {
            "status": noise.NOISE_TAPES_PASS_STATUS,
            "passed": True,
            "case_order": list(contract.CASE_IDS),
        },
        "manifest_record": _artifact(contract.NOISE_MANIFEST_PATH.as_posix()),
        "tapes": tapes,
    }


def _fake_source_gate() -> dict[str, Any]:
    records = {
        name: _artifact(path)
        for name, path in freeze.RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }
    return {"passed": True, "checks": {"all": True}, "records": records}


def _fake_input_gate() -> dict[str, Any]:
    return {
        "passed": True,
        "checks": {"all": True},
        "records": {
            "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
            "source_h0_module": {
                **contract.SOURCE_H0_MODULE,
                "file_count": 1,
                "files": [{"path": "module_state.pkl"}],
            },
            "source_h0_config": _artifact(freeze.SOURCE_H0_CONFIG_PATH.as_posix()),
            "historical_analog_profile": _artifact(
                freeze.ANALOG_PROFILE_PATH.as_posix()
            ),
            "baseline_shadow_v25_profile": _artifact(
                freeze.BASELINE_SHADOW_V25_PROFILE_PATH.as_posix()
            ),
        },
    }


def _patch_complete_closure(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(freeze, "_prerequisite_gate", _fake_prerequisite_binding)
    monkeypatch.setattr(freeze, "_noise_gate", _fake_noise_gate)
    monkeypatch.setattr(freeze, "_source_gate", _fake_source_gate)
    monkeypatch.setattr(freeze, "_input_gate", _fake_input_gate)
    monkeypatch.setattr(
        freeze,
        "_occupancy_snapshot",
        lambda: {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "qualification_run_root_absent": True,
            "noise_manifest_present": True,
        },
    )


def test_build_is_blocked_while_salvage_prerequisites_are_absent() -> None:
    assert not freeze.PROTOCOL_FREEZE_PATH.exists()
    with pytest.raises(
        freeze.V12R3P1QualificationProtocolFreezeError,
        match="prerequisite",
    ):
        freeze.build_protocol_freeze()
    assert not freeze.PROTOCOL_FREEZE_PATH.exists()


def test_complete_closure_builds_exact_post_salvage_protocol(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_complete_closure(monkeypatch)
    payload = freeze.build_protocol_freeze()
    assert payload["schema_version"] == 125
    assert payload["status"] == contract.PROTOCOL_FREEZE_PASS_STATUS
    assert payload["passed"] is True
    assert payload["freeze_kind"] == ("POST_SALVAGE_INDEPENDENT_QUALIFICATION_PROTOCOL")
    assert payload["qualification_design_freeze"] == noise.DESIGN_FREEZE_RECORD
    assert payload["selected_candidate_id"] == contract.P1_CANDIDATE_ID
    assert payload["selected_candidate"] == contract.P1_CANDIDATE_MODULE
    assert len(payload["salvage_prerequisite_records"]) == 4
    assert len(payload["noise_tapes"]) == 5
    assert len(payload["runtime_sources"]) == 10
    assert payload["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert payload["stage_order"][-1] == "finalize_qualification"
    assert all(value == 0 for value in payload["zero_freeze_activity"].values())
    assert payload["execution_lock"] is None
    assert payload["pipeline_claim"] is None


def test_runtime_source_list_is_closed_and_excludes_moving_p1s_sources() -> None:
    assert list(freeze.RUNTIME_SOURCE_RELATIVE_PATHS) == [
        "noise_preparer",
        "protocol_freezer",
        "execution_runner",
        "noise_preparer_tests",
        "protocol_freezer_tests",
        "execution_runner_tests",
        "forensic_writer",
        "v6_role_runtime",
        "v26_environment_source",
        "v12r3_physical_runtime",
    ]
    assert len(set(freeze.RUNTIME_SOURCE_RELATIVE_PATHS.values())) == 10
    assert not any(
        "/v12p1s/" in path for path in freeze.RUNTIME_SOURCE_RELATIVE_PATHS.values()
    )
    source = freeze._source_gate()
    assert source["passed"] is True
    assert list(source["records"]) == list(freeze.RUNTIME_SOURCE_RELATIVE_PATHS)


def test_current_h0_p1_and_runtime_input_trees_are_bound() -> None:
    inputs = freeze._input_gate()
    assert inputs["passed"] is True
    assert inputs["records"]["candidate_module"] == contract.P1_CANDIDATE_MODULE
    assert inputs["records"]["source_h0_module"]["tree_sha256"] == (
        contract.SOURCE_H0_MODULE["tree_sha256"]
    )
    assert inputs["records"]["source_h0_config"]["size_bytes"] > 0
    assert inputs["records"]["historical_analog_profile"]["size_bytes"] > 0
    assert inputs["records"]["baseline_shadow_v25_profile"]["size_bytes"] > 0


def test_design_or_candidate_drift_fails_assembly(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_complete_closure(monkeypatch)
    bad = _fake_prerequisite_binding()
    bad["gate"]["candidate_id"] = "other"
    monkeypatch.setattr(freeze, "_prerequisite_gate", lambda: bad)
    payload = freeze.build_protocol_freeze()
    assert payload["passed"] is False
    assert payload["checks"]["same_exact_p1"] is False


def test_temp_publish_verify_is_canonical_no_clobber(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_complete_closure(monkeypatch)
    destination = tmp_path / "protocol.json"
    published = freeze.publish_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    assert published["passed"] is True
    assert (
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
        == published
    )
    with pytest.raises(freeze.V12R3P1QualificationProtocolFreezeError, match="clobber"):
        freeze.publish_protocol_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert not freeze.PROTOCOL_FREEZE_PATH.exists()


def test_noncanonical_publish_requires_explicit_test_override(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_complete_closure(monkeypatch)
    destination = tmp_path / "not-canonical.json"
    with pytest.raises(
        freeze.V12R3P1QualificationProtocolFreezeError, match="non-canonical"
    ):
        freeze.publish_protocol_freeze(output_path=destination)
    assert not destination.exists()


def test_verify_rejects_byte_drift(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_complete_closure(monkeypatch)
    destination = tmp_path / "protocol.json"
    freeze.publish_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    destination.write_bytes(b"{}\n")
    with pytest.raises(freeze.V12R3P1QualificationProtocolFreezeError, match="drifted"):
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )


def test_no_canonical_protocol_noise_lock_or_run_is_published() -> None:
    assert not os.path.lexists(freeze.PROTOCOL_FREEZE_PATH)
    assert not os.path.lexists(freeze.EXECUTION_LOCK_PATH)
    assert not os.path.lexists(freeze.RUN_ROOT)
    assert not os.path.lexists(freeze.NOISE_MANIFEST_PATH)
