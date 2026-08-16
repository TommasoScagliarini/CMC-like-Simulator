"""Strict artifact helpers; only the generic Q1 record predicate is reused."""

from __future__ import annotations

import importlib.util
import os
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any


def discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = discover_repo_root(Path(__file__))
Q1_GATES_PATH = (
    REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "validation"
    / "v12p1q"
    / "h0_v12r3_p1_qualification_gates.py"
)
Q1_ROOT = Q1_GATES_PATH.parent
if str(Q1_ROOT) not in sys.path:
    sys.path.insert(0, str(Q1_ROOT))


def _load_q1_artifact_predicate() -> Any:
    spec = importlib.util.spec_from_file_location(
        "_v12r5q3_q1_artifact_predicate_only", Q1_GATES_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("frozen Q1 artifact predicate could not be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.artifact_record_matches


_Q1_ARTIFACT_RECORD_MATCHES = _load_q1_artifact_predicate()


class Q3ArtifactError(RuntimeError):
    """Raised for unsafe, noncanonical, or drifting artifacts."""


def artifact_record_matches(value: Any, expected_path: str | PurePosixPath) -> bool:
    """Call the one generic Q1 helper permitted by the Q3 design."""

    return bool(_Q1_ARTIFACT_RECORD_MATCHES(value, expected_path))


def is_link_or_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attributes = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attributes & reparse)


def assert_no_link_components(path: Path) -> None:
    cursor = path.absolute()
    while True:
        if os.path.lexists(cursor) and is_link_or_reparse(cursor):
            raise Q3ArtifactError(f"path contains link/reparse component: {cursor}")
        if cursor == cursor.parent:
            break
        cursor = cursor.parent


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise Q3ArtifactError(f"non-canonical repository-relative path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def portable_path(path: Path) -> str:
    absolute = path.absolute()
    try:
        return absolute.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(absolute)


def record(
    path: str | os.PathLike[str] | Path, *, logical_path: str | None = None
) -> dict[str, Any]:
    import hashlib

    source = Path(path).absolute()
    assert_no_link_components(source)
    if not source.is_file() or is_link_or_reparse(source):
        raise Q3ArtifactError(f"artifact is not a safe regular file: {source}")
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": portable_path(source) if logical_path is None else logical_path,
        "sha256": digest.hexdigest(),
        "size_bytes": source.stat().st_size,
    }


__all__ = [
    "Q3ArtifactError",
    "REPO_ROOT",
    "artifact_record_matches",
    "assert_no_link_components",
    "discover_repo_root",
    "is_link_or_reparse",
    "portable_path",
    "record",
    "resolve_relative",
]
