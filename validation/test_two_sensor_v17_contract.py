"""Static tests for the V17 profile, primary lock, and hybrid routing."""

from __future__ import annotations

import ast
import copy
import sys
from pathlib import Path

import pytest

VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import validate_two_sensor_v17_contract as subject  # noqa: E402


def test_repository_v17_contract_and_primary_routing_pass() -> None:
    result = subject.validate_all()
    assert result["status"] == "PASS"
    assert result["event_contract_id"] == subject.EVENT_CONTRACT_ID
    assert result["v17_profile"]["sensor_count"] == 2
    assert result["v17_profile"]["geometry_exact_v13"] is True
    assert result["primary_core"]["claim"] == "hybrid_left_online_right_prescribed_only"
    assert result["routing"]["detector_applies_force_false"] is True
    assert result["routing"]["right_prescribed_external_force_present"] is True
    assert result["data_access"]["protected_sources_exposed"] == []


def test_v17_contract_exposes_only_development_source_paths() -> None:
    contract = subject._strict_object(subject.DEFAULT_V17_CONTRACT, "contract")
    assert set(contract["development_sources"]) == {"02", "04", "08"}
    assert contract["data_split"]["validation_one_shot_closed"] == ["05"]
    assert contract["data_split"]["sealed_one_shot_closed"] == ["06"]
    assert contract["data_split"]["unallocated_reserve_closed"] == ["03", "07"]
    assert contract["decision_contract"]["protected_open_allowed_by_this_contract"] is False


def test_profile_geometry_is_exactly_v13_and_not_promoted() -> None:
    contract = subject._strict_object(subject.DEFAULT_V17_CONTRACT, "contract")
    result = subject.validate_v17_profile(contract)
    assert result["geometry_exact_v13"] is True
    assert result["not_promoted"] is True


def test_primary_lock_does_not_hash_python_or_yaml_into_scientific_core() -> None:
    lock = subject._strict_object(subject.DEFAULT_PRIMARY_LOCK, "lock")
    paths = {record["path"] for record in lock["scientific_core"].values()}
    assert "model_loader.py" not in paths
    assert "online_grf.py" not in paths
    assert not any(path.endswith((".yaml", ".yml")) for path in paths)
    assert "COP representation or sentinel at zero load" in lock["scientific_exclusions"]
    assert "technical left-only primary-profile loader option" in lock["scientific_exclusions"]


def test_tampered_primary_hash_fails_closed() -> None:
    lock = subject._strict_object(subject.DEFAULT_PRIMARY_LOCK, "lock")
    tampered = copy.deepcopy(lock)
    tampered["scientific_core"]["primary_profile"]["sha256"] = "0" * 64
    with pytest.raises(subject.V17ContractError, match="hash mismatch"):
        subject.validate_primary_core_lock(tampered)


def test_loader_ast_contains_no_dynamic_detector_force_flag() -> None:
    lock = subject._strict_object(subject.DEFAULT_PRIMARY_LOCK, "lock")
    loader = subject._repo_path(
        lock["routing_contract"]["loader"], label="routing.loader"
    )
    parsed = ast.parse(loader.read_text(encoding="utf-8"))
    result = subject.validate_loader_routing(loader)
    assert parsed is not None
    assert result["detector_applies_force_false"] is True
    assert result["primary_per_side_application"] is True
