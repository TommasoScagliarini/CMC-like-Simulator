from __future__ import annotations

import ast
from pathlib import Path

import pytest

import freeze_h0_v12r4_p3_coverage as freezer
import h0_v12r4_p3_coverage_contract as contract


def test_resolver_rejects_noncanonical_paths() -> None:
    for value in ("../escape", "/absolute", "a/../b", ""):
        with pytest.raises(freezer.V12R4ProtocolFreezeError):
            freezer.resolve_relative(value)


def test_immutable_p2_and_terminal_records_are_current() -> None:
    assert all(freezer._records_exact().values())
    assert all(freezer._terminal_semantics().values())
    assert all(freezer._p2_semantics().values())


def test_q2_design_is_allowed_present_not_part_of_runtime_absence() -> None:
    absence = freezer._output_absence()
    assert "q2_design_freeze" not in absence
    assert set(
        name.removeprefix("q2_") for name in absence if name.startswith("q2_")
    ) == set(contract.Q2_UNOPENED_PATHS)
    semantics = freezer._q2_design_semantics()
    assert all(semantics.values())
    assert (
        freezer.artifact_record(contract.Q2_DESIGN_FREEZE_PATH)
        == contract.Q2_DESIGN_FREEZE_ARTIFACT
    )


def test_freezer_source_has_no_rllib_or_opensim_import() -> None:
    source = Path(freezer.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert not any(name.startswith("ray") for name in imported)
    assert not any(name.startswith("opensim") for name in imported)


def test_source_closure_includes_cross_platform_attributes() -> None:
    source = Path(freezer.__file__).read_text(encoding="utf-8")
    assert '".gitattributes"' in source
    attributes = Path(freezer.REVISION_ROOT / ".gitattributes").read_text(
        encoding="utf-8"
    )
    assert "*.py text eol=lf" in attributes
    assert "*.json text eol=lf" in attributes
    assert "*.npz binary" in attributes
    assert "*.pkl binary" in attributes
