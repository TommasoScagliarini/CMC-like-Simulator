"""Regression tests for the full-tree R9/Q3 candidate ABI.

The R8 terminal failure was caused by comparing a complete runtime tree with a
three-field projection.  These tests bind the Q3 validator to a real five-file
tree and make the lossy projection an explicit negative case.
"""

from __future__ import annotations

import copy
import hashlib
import sys
from pathlib import Path
from typing import Any

import pytest


Q3_ROOT = Path(__file__).resolve().parent
REPO_ROOT = Q3_ROOT.parents[3]
if str(Q3_ROOT) not in sys.path:
    sys.path.insert(0, str(Q3_ROOT))

import h0_v12r9_q3_artifacts as artifacts  # noqa: E402
import h0_v12r9_q3_qualification_contract as contract  # noqa: E402


def _real_tree_record(path: Path, *, repository_path: str) -> dict[str, Any]:
    assert path.is_dir() and not path.is_symlink()
    rows: list[dict[str, Any]] = []
    for child in sorted(path.iterdir(), key=lambda item: item.name):
        assert child.is_file() and not child.is_symlink()
        payload = child.read_bytes()
        rows.append(
            {
                "path": child.name,
                "sha256": hashlib.sha256(payload).hexdigest(),
                "size_bytes": len(payload),
            }
        )
    return {
        "path": repository_path,
        "tree_sha256": artifacts.tree_digest(rows),
        "file_count": len(rows),
        "files": rows,
    }


def test_locked_full_r6_tree_matches_the_real_five_file_tree() -> None:
    expected = copy.deepcopy(contract.r9.FULL_R6_CANDIDATE_TREE)
    observed = _real_tree_record(
        REPO_ROOT.joinpath(*Path(expected["path"]).parts),
        repository_path=expected["path"],
    )
    assert observed == expected
    assert set(observed) == {"path", "tree_sha256", "file_count", "files"}
    assert observed["file_count"] == len(observed["files"]) == 5


def test_q3_tree_validator_requires_full_tree_not_r8_projection() -> None:
    real = copy.deepcopy(contract.r9.FULL_R6_CANDIDATE_TREE)
    expected_files = tuple(row["path"] for row in real["files"])
    assert (
        artifacts.validate_exact_tree(
            real,
            expected_path=real["path"],
            expected_files=expected_files,
        )
        == real
    )

    projection = {key: real[key] for key in ("path", "tree_sha256", "file_count")}
    with pytest.raises(
        artifacts.V12R9Q3ArtifactError,
        match="schema or root path",
    ):
        artifacts.validate_exact_tree(
            projection,
            expected_path=real["path"],
            expected_files=expected_files,
        )


def test_r9_contract_itself_rejects_the_historical_lossy_projection() -> None:
    real = contract.r9.FULL_R6_CANDIDATE_TREE
    projection = {key: real[key] for key in ("path", "tree_sha256", "file_count")}
    assert contract.r9.LOCKED_INPUTS["r6_candidate"] == real
    assert projection == contract.r9.v12r8.LOCKED_INPUTS["r6_candidate"]
    assert real != projection
