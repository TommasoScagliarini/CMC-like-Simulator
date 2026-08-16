"""Portable fail-closed artifact predicates for the V12R7-Q3 scaffold.

The helpers are deliberately independent of historical qualification gates.
They perform no filesystem access and accept only canonical repository-relative
POSIX paths.  In particular, Windows separators, drive-qualified paths,
traversal, booleans masquerading as integers, and malformed SHA-256 values are
rejected.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping, Sequence
from pathlib import PurePosixPath
from typing import Any


class V12R7Q3ArtifactError(ValueError):
    """Raised when an artifact description is not canonical and immutable."""


def is_sha256(value: Any) -> bool:
    """Return whether *value* is one lowercase hexadecimal SHA-256 digest."""

    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def canonical_relative_path(value: Any) -> bool:
    """Accept one normalized repository-relative POSIX path only."""

    if (
        not isinstance(value, str)
        or not value
        or value == "."
        or "\\" in value
        or "\x00" in value
        or ":" in value
    ):
        return False
    path = PurePosixPath(value)
    return (
        not path.is_absolute()
        and value == path.as_posix()
        and all(part not in {"", ".", ".."} for part in path.parts)
    )


def artifact_record_matches(value: Any, expected_path: str | PurePosixPath) -> bool:
    """Validate a strict three-field regular-file record without touching disk."""

    record = dict(value) if isinstance(value, Mapping) else {}
    expected = (
        expected_path.as_posix()
        if isinstance(expected_path, PurePosixPath)
        else expected_path
    )
    return (
        canonical_relative_path(expected)
        and set(record) == {"path", "sha256", "size_bytes"}
        and record.get("path") == expected
        and is_sha256(record.get("sha256"))
        and type(record.get("size_bytes")) is int
        and record["size_bytes"] > 0
    )


def tree_digest(rows: Sequence[Mapping[str, Any]]) -> str:
    """Compute the canonical tree digest used by the R6/R7 actor artifacts."""

    digest = hashlib.sha256()
    for row in rows:
        digest.update(str(row["path"]).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(row["sha256"]).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def validate_exact_tree(
    value: Any,
    *,
    expected_path: str | PurePosixPath,
    expected_files: Sequence[str],
) -> dict[str, Any]:
    """Return a normalized exact tree or raise :class:`V12R7Q3ArtifactError`."""

    tree = dict(value) if isinstance(value, Mapping) else {}
    path = (
        expected_path.as_posix()
        if isinstance(expected_path, PurePosixPath)
        else expected_path
    )
    rows = tree.get("files")
    expected_names = tuple(sorted(expected_files))
    if (
        not canonical_relative_path(path)
        or set(tree) != {"path", "tree_sha256", "file_count", "files"}
        or tree.get("path") != path
        or not is_sha256(tree.get("tree_sha256"))
        or type(tree.get("file_count")) is not int
        or tree.get("file_count") != len(expected_names)
        or not isinstance(rows, list)
        or len(rows) != len(expected_names)
    ):
        raise V12R7Q3ArtifactError("artifact tree schema or root path is invalid")

    normalized: list[dict[str, Any]] = []
    for row in rows:
        current = dict(row) if isinstance(row, Mapping) else {}
        if (
            set(current) != {"path", "sha256", "size_bytes"}
            or not canonical_relative_path(current.get("path"))
            or PurePosixPath(current["path"]).parent != PurePosixPath(".")
            or not is_sha256(current.get("sha256"))
            or type(current.get("size_bytes")) is not int
            or current["size_bytes"] <= 0
        ):
            raise V12R7Q3ArtifactError("artifact tree contains a malformed file")
        normalized.append(current)

    names = tuple(row["path"] for row in normalized)
    if names != expected_names or len(set(names)) != len(names):
        raise V12R7Q3ArtifactError("artifact tree file manifest is not exact")
    if tree_digest(normalized) != tree["tree_sha256"]:
        raise V12R7Q3ArtifactError("artifact tree digest does not bind its files")
    return {
        "path": path,
        "tree_sha256": tree["tree_sha256"],
        "file_count": len(normalized),
        "files": [dict(row) for row in normalized],
    }


__all__ = [
    "V12R7Q3ArtifactError",
    "artifact_record_matches",
    "canonical_relative_path",
    "is_sha256",
    "tree_digest",
    "validate_exact_tree",
]
