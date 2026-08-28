"""Consumed-input ledger, strict serialisation and no-replace artefact publishing for F0.6 / F0.8.

CONTRACT (fail-closed):

  discovery   Code may read files to ENUMERATE the consumed-input set (registry description,
              receipt-level hashing, config read to name a profile). Discovery data is never
              used for computation: every computational input is re-read through the ledger.
  PRE         ``InputLedger.declare(paths)`` hashes every declared input (SHA-256) BEFORE any
              computational read. On POSIX every path is opened component by component with
              O_NOFOLLOW (openat chain, leaf O_NONBLOCK + fstat regular): a SYMLINK anywhere in
              the chain, a missing component or a non-regular leaf fails closed. The set must
              include every data input, the scripts/helpers that compute the artefact, the
              actor feature manifest and (as provenance) the interpreter.
  byte-bound  ``InputLedger.read_bytes(path)`` re-opens the same no-follow chain at EVERY call
  reads       (it is not a cache), reads the bytes, hashes exactly those bytes, compares with the
              PRE digest and hands the same bytes to the parser (``read_text`` / ``read_json`` /
              ``read_yaml`` / ``read_sto`` / ``read_pickle``): every read is byte-bound to the PRE
              digest and re-verified POST. An undeclared path, a symlink (leaf or ancestor), or
              a digest different from PRE fails closed. There is no "note + direct read".
  CONTENT-    The ledger is CONTENT-ADDRESSED: it binds BYTES, not inodes or path identity. A
  ADDRESSED   plain file or plain ancestor directory replaced by another plain one carrying the
              SAME bytes is accepted by design and is NOT detected; only symlinks (POSIX
              no-follow chain) and byte changes are refused. The Windows / non-dir_fd fallback
              is lstat-then-open: compatible, but without the atomic ancestor guarantee.
  POST        ``assert_unchanged`` re-opens and re-hashes the declared set with the same chain
              checks (after loading, after computing, immediately before the staged verification
              that precedes the publish rename); ``assert_all_read`` requires every declared
              input to have been consumed.
  publish     ``publish_artifact`` validates the stamp as a plain basename, checks the whole
              ancestor chain for symlinks BEFORE mkdir, builds in an exclusive staging sibling
              ``.staging_<stamp>_<pid>``, refuses ANY entry (file, directory, special node) named
              ``artifact_manifest_*`` and any non-regular node in the tree, writes the single
              expected sidecar ``artifact_manifest_<stamp>.json`` LAST (SHA-256 of every other
              file), re-checks the inputs (``before_publish``), verifies the staged tree
              DEEP-EXACT and BYTE-EXACT against the sidecar just sealed IMMEDIATELY before the
              rename, renames with a NO-REPLACE primitive (Darwin renamex_np RENAME_EXCL, Linux
              renameat2 RENAME_NOREPLACE, Windows MoveFileEx without replace; fail-closed
              elsewhere) and verifies the published tree again against the same sealed sidecar.
              A failed staged verification RAISES and leaves no final directory; a failed
              post-rename verification moves WHATEVER occupies the authoritative stamp
              (directory, file, symlink - also broken -, special node; the entry itself, never
              followed) to a unique ``.quarantine_*`` sibling (no-replace), re-checks lexically
              (lexists) that the stamp is free and RAISES ``ArtifactVerificationError``; if the
              quarantine itself fails the error says so and never claims the stamp is free.
              Nothing staged is ever serialised or returned; the sidecar digest is returned so
              later audits can pin it.

SCOPE OF THE GUARANTEE: DATA inputs are byte-bound (PRE digest, every computational read
re-opened and checked against PRE, POST re-hash). Python SOURCE (F0 scripts, helpers,
warm_start.py) is imported by the
interpreter BEFORE the runtime ledger exists: the recorded source hashes attest the on-disk
provenance of the code at run time, NOT the bytes already compiled into the running process.
No claim of code-execution TOCTOU freedom is made.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import errno
import hashlib
import json
import math
import os
import pickle
import platform
import re
import shutil
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Callable

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402

ARTIFACT_MANIFEST_PREFIX = "artifact_manifest_"
STAGING_MARKER = ".staging_"
MANIFEST_SCHEMA_VERSION = 3
STAMP_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")
SIDECAR_RE = re.compile(r"artifact_manifest_([A-Za-z0-9][A-Za-z0-9._-]{0,63})\.json")
HEX64 = set("0123456789abcdef")
REQUIRED_PROVENANCE = ("kind", "source_analysis", "script_sha256", "git_head", "inputs_unchanged", "interpreter", "consumed_inputs")
CHUNK = 1 << 20


class StagingError(RuntimeError):
    """The publishing protocol cannot proceed fail-closed (paths, chains, stamps, sidecar, schema, platform primitive)."""


class ArtifactVerificationError(StagingError):
    """A sealed (or staged) artefact failed verification: digests, sizes, sidecar binding, unlisted or missing files."""


class InputsChangedError(RuntimeError):
    """A consumed input changed between snapshots, was read undeclared, or was never read."""


class SerializationError(RuntimeError):
    """A payload cannot be serialised/parsed strictly (NaN/Inf, staging paths, unknown types)."""


# --- strict JSON --------------------------------------------------------------------------------


def _json_default(value: Any) -> Any:
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return value.as_posix()
    raise TypeError(f"not JSON serialisable: {type(value).__name__}")


def assert_no_staging_paths(obj: Any, path: str = "") -> None:
    if isinstance(obj, dict):
        for k, v in obj.items():
            assert_no_staging_paths(k, f"{path}.{k}")
            assert_no_staging_paths(v, f"{path}.{k}")
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_staging_paths(v, f"{path}[{i}]")
    elif isinstance(obj, (str, Path)) and STAGING_MARKER in str(obj):
        raise SerializationError(f"staging path serialised at {path or '<root>'}: {obj}")


def dumps_strict(payload: Any) -> str:
    assert_no_staging_paths(payload)
    try:
        return json.dumps(payload, indent=2, allow_nan=False, default=_json_default)
    except (ValueError, TypeError) as exc:
        raise SerializationError(f"strict JSON serialisation failed: {exc}") from exc


def _no_const(name: str) -> Any:
    raise SerializationError(f"non-finite constant in JSON: {name}")


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise SerializationError(f"JSON float overflow/non-finite literal: {text}")
    return value


def _no_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise SerializationError(f"duplicate JSON object key: {key!r}")
        out[key] = value
    return out


def loads_strict(text: str | bytes) -> Any:
    """Strict JSON: no NaN/Infinity constants, no float overflow (1e999), no duplicate object keys at any level,
    every number finite (assert_finite_tree), no staging paths."""
    if isinstance(text, bytes):
        text = text.decode("utf-8")
    try:
        payload = json.loads(text, parse_constant=_no_const, parse_float=_finite_float, object_pairs_hook=_no_duplicate_keys)
    except SerializationError:
        raise
    except ValueError as exc:
        raise SerializationError(f"strict JSON parse failed: {exc}") from exc
    assert_finite_tree(payload, "json")
    assert_no_staging_paths(payload)
    return payload


def write_json_strict(path: Path, payload: Any) -> Path:
    path = Path(path)
    text = dumps_strict(payload)
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing artefact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def assert_finite_tree(obj: Any, label: str = "") -> None:
    """Every number in a nested structure must be finite (NaN / +-Inf refused); None, str, bool allowed."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            assert_finite_tree(v, f"{label}.{k}" if label else str(k))
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_finite_tree(v, f"{label}[{i}]")
    elif isinstance(obj, np.ndarray):
        if obj.dtype.kind in "fc" and not np.all(np.isfinite(obj)):
            raise SerializationError(f"non-finite values in array {label}")
    elif obj is None or isinstance(obj, (bool, str, np.bool_, int, np.integer)):
        return
    elif isinstance(obj, (float, np.floating)):
        if not math.isfinite(float(obj)):
            raise SerializationError(f"non-finite number at {label}: {obj!r}")
    else:
        raise SerializationError(f"unsupported value type at {label}: {type(obj).__name__}")


# --- symlink-free chains ------------------------------------------------------------------------


def assert_regular_chain(path: Path, *, kind: str = "file") -> Path:
    """lstat every component of an ABSOLUTE path from the filesystem root: no symlink anywhere; the leaf must
    be a regular file (kind='file') or a directory (kind='dir'). Missing components fail closed."""
    p = Path(path)
    if not p.is_absolute():
        raise StagingError(f"absolute path required: {p}")
    current = Path(p.anchor)
    for part in p.parts[1:]:
        current = current / part
        try:
            st = os.lstat(current)
        except OSError as exc:
            raise StagingError(f"path chain broken at {current}: {exc.strerror}") from exc
        if stat.S_ISLNK(st.st_mode):
            raise StagingError(f"symlink in the path chain: {current}")
    leaf = os.lstat(p)
    if kind == "file" and not stat.S_ISREG(leaf.st_mode):
        raise StagingError(f"not a regular file: {p}")
    if kind == "dir" and not stat.S_ISDIR(leaf.st_mode):
        raise StagingError(f"not a directory: {p}")
    return p


def _open_regular_nofollow(path: Path) -> int:
    """Open a regular file through an O_NOFOLLOW openat chain (POSIX): every component is opened relative to
    its parent descriptor with O_NOFOLLOW, so a SYMLINK anywhere in the chain (or one planted between the
    check and the open) is refused by the kernel; the leaf is opened O_NONBLOCK and must fstat as a regular
    file (FIFOs/devices refused without blocking). The check is content-addressed: a plain directory or file
    replaced by another plain one with the same bytes is NOT distinguished (no inode/path identity claim).
    Windows/non-dir_fd fallback: lstat walk then open (O_NOFOLLOW when available) - compatible, but without
    the atomic ancestor guarantee of the openat chain."""
    p = Path(path)
    if not p.is_absolute():
        raise StagingError(f"absolute path required: {p}")
    flags_dir = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    # O_NONBLOCK: opening a FIFO/special file must not block; regular-file reads are unaffected and fstat rejects non-regular leaves
    flags_file = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_BINARY", 0) | getattr(os, "O_NONBLOCK", 0)
    if os.name == "posix" and os.open in os.supports_dir_fd:
        parts = p.parts[1:]
        try:
            fd = os.open(p.anchor, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        except OSError as exc:
            raise StagingError(f"cannot open filesystem root {p.anchor}: {exc.strerror}") from exc
        try:
            for part in parts[:-1]:
                try:
                    nfd = os.open(part, flags_dir, dir_fd=fd)
                except OSError as exc:
                    raise StagingError(f"path chain refused at {part!r} of {p}: {exc.strerror} (symlink, missing or not a directory)") from exc
                os.close(fd)
                fd = nfd
            try:
                lfd = os.open(parts[-1], flags_file, dir_fd=fd)
            except OSError as exc:
                raise StagingError(f"leaf refused for {p}: {exc.strerror} (symlink or missing)") from exc
        finally:
            os.close(fd)
    else:
        assert_regular_chain(p, kind="file")
        try:
            lfd = os.open(str(p), flags_file)
        except OSError as exc:
            raise StagingError(f"cannot open {p}: {exc.strerror}") from exc
    try:
        if not stat.S_ISREG(os.fstat(lfd).st_mode):
            raise StagingError(f"not a regular file: {p}")
    except BaseException:
        os.close(lfd)
        raise
    return lfd


def read_regular_nofollow(path: Path) -> bytes:
    fd = _open_regular_nofollow(path)
    try:
        chunks = []
        while True:
            block = os.read(fd, CHUNK)
            if not block:
                break
            chunks.append(block)
        return b"".join(chunks)
    finally:
        os.close(fd)


def sha256_regular_nofollow(path: Path) -> tuple[str, int]:
    """(sha256, size) of a regular file opened through the no-follow chain (streamed)."""
    fd = _open_regular_nofollow(path)
    try:
        digest = hashlib.sha256()
        size = 0
        while True:
            block = os.read(fd, CHUNK)
            if not block:
                break
            digest.update(block)
            size += len(block)
        return digest.hexdigest(), size
    finally:
        os.close(fd)


def _walk_entries(root: Path) -> list[tuple[Path, os.stat_result]]:
    """Every entry of a tree (directories and files, never following symlinks) with its lstat, sorted by path."""
    out: list[tuple[Path, os.stat_result]] = []
    for dirpath, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
        for name in (*dirnames, *filenames):
            p = Path(dirpath) / name
            out.append((p, os.lstat(p)))
    return sorted(out, key=lambda e: str(e[0]))


# --- paths ------------------------------------------------------------------------------------


def validate_stamp(stamp: Any) -> str:
    """A stamp is a plain basename: no separators, no traversal, no staging marker, no sidecar prefix."""
    if not isinstance(stamp, str) or not STAMP_RE.fullmatch(stamp) or stamp in (".", "..") or "/" in stamp or "\\" in stamp:
        raise StagingError(f"invalid artefact stamp (basename expected): {stamp!r}")
    if STAGING_MARKER in stamp or stamp.startswith(ARTIFACT_MANIFEST_PREFIX):
        raise StagingError(f"reserved marker in artefact stamp: {stamp!r}")
    return stamp


def sidecar_name(stamp: str) -> str:
    return f"{ARTIFACT_MANIFEST_PREFIX}{validate_stamp(stamp)}.json"


def assert_no_symlink_components(path: Path, anchor: Path, label: str) -> Path:
    """The anchor's own chain must be symlink-free (realpath == normpath), ``path`` must stay inside it
    without traversal, and every EXISTING component below the anchor must not be a symlink."""
    path, anchor = Path(path), Path(anchor)
    if not anchor.is_absolute():
        raise StagingError(f"{label}: anchor must be absolute: {anchor}")
    if any(part == ".." for part in str(path).replace("\\", "/").split("/")) or any(part == ".." for part in str(anchor).replace("\\", "/").split("/")):
        raise StagingError(f"{label}: lexical traversal ('..') refused before normalisation: {path}")
    anchor_norm = os.path.normpath(str(anchor))
    if os.path.normcase(os.path.realpath(anchor_norm)) != os.path.normcase(anchor_norm):
        raise StagingError(f"{label}: symlink in the anchor chain: {anchor} -> {os.path.realpath(anchor_norm)}")
    norm = Path(os.path.normpath(str(path)))
    try:
        rel = norm.relative_to(Path(anchor_norm))
    except ValueError as exc:
        raise StagingError(f"{label}: path escapes the approved anchor {anchor}: {path}") from exc
    if any(p in (".", "..") for p in rel.parts):
        raise StagingError(f"{label}: traversal in {path}")
    current = Path(anchor_norm)
    for part in rel.parts:
        current = current / part
        if current.is_symlink():
            raise StagingError(f"{label}: symlink component {current}")
    return norm


def resolve_under_roots(value: str, roots: tuple[Path, ...], label: str) -> Path:
    """Resolve a config/profile path under the approved roots only (no escape, no traversal, no symlink ancestor, regular file)."""
    raw = str(value)
    text = raw.replace("\\", "/")
    if any(part == ".." for part in text.split("/")):
        raise StagingError(f"{label}: lexical traversal ('..') refused before normalisation: {value!r}")
    candidates = [Path(text)] if os.path.isabs(text) else [Path(root) / text for root in roots]
    for cand in candidates:
        norm = Path(os.path.normpath(str(cand)))
        anchors = [Path(r) for r in roots if str(norm).startswith(os.path.normpath(str(r)) + os.sep)]
        if not anchors:
            continue
        if norm.is_file() or norm.is_symlink():
            assert_no_symlink_components(norm, anchors[0], label)
            assert_regular_chain(norm, kind="file")
            return norm
    raise StagingError(f"{label}: {value!r} not found under the approved roots {[str(r) for r in roots]} (or escapes them)")


# --- consumed-input ledger ------------------------------------------------------------------------


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def snapshot_files(paths: list[Path]) -> dict[str, str]:
    """SHA-256 of every path through the no-follow chain (a symlink anywhere, a missing or non-regular node fails closed)."""
    out: dict[str, str] = {}
    for p in paths:
        out[C.rel(p)] = sha256_regular_nofollow(Path(p))[0]
    return out


def assert_unchanged(before: dict[str, str], after: dict[str, str], label: str) -> None:
    if set(before) != set(after):
        raise InputsChangedError(f"{label}: input set changed: {sorted(set(before) ^ set(after))[:10]}")
    changed = [k for k in before if before[k] != after[k]]
    if changed:
        raise InputsChangedError(f"{label}: {len(changed)} consumed input(s) changed: {changed[:10]}")


def parse_sto(text: str) -> tuple[list[str], np.ndarray]:
    """OpenSim .sto table from text (same grammar as f0_replay_analysis.read_sto, bytes supplied by the caller)."""
    lines = text.splitlines()
    try:
        idx = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    except StopIteration as exc:
        raise ValueError("malformed .sto table: no endheader") from exc
    names = lines[idx + 1].split()
    rows = [[float(x) for x in line.split()] for line in lines[idx + 2:] if line.strip()]
    data = np.asarray(rows, dtype=np.float64)
    if data.ndim != 2 or data.shape[1] != len(names):
        raise ValueError("malformed .sto table: column count mismatch")
    return names, data


class DirectReader:
    """Reader WITHOUT declaration (no-follow chain, the bytes of each call handed to the parser; no cache).
    Used by unit tests, discovery and read-only checks; the artefact wrappers use ``InputLedger``."""

    def read_bytes(self, path: Path) -> bytes:
        return read_regular_nofollow(Path(path))

    def read_text(self, path: Path, *, errors: str = "strict") -> str:
        return self.read_bytes(path).decode("utf-8", errors=errors)

    def read_json(self, path: Path) -> Any:
        return loads_strict(self.read_text(path))

    def read_yaml(self, path: Path) -> dict[str, Any]:
        import yaml  # noqa: PLC0415

        data = yaml.safe_load(self.read_text(path))
        return data or {}

    def read_sto(self, path: Path) -> tuple[list[str], np.ndarray]:
        return parse_sto(self.read_text(path, errors="replace"))

    def read_pickle(self, path: Path) -> Any:
        return pickle.loads(self.read_bytes(path))


DIRECT = DirectReader()


class InputLedger(DirectReader):
    """Declared consumed inputs hashed BEFORE any computational read; every read goes through
    ``read_bytes`` (no-follow chain re-opened at each call - not a cache -, digest of the bytes
    actually returned compared with PRE, re-verified POST)."""

    def __init__(self) -> None:
        self.declared: dict[str, str] = {}
        self.paths: dict[str, Path] = {}
        self.read: dict[str, str] = {}

    def declare(self, paths: list[Path]) -> dict[str, str]:
        if self.declared:
            raise InputsChangedError("ledger already declared (PRE snapshot is taken once)")
        uniq: dict[str, Path] = {}
        for p in paths:
            p = Path(p)
            if not p.is_absolute():
                raise StagingError(f"declared input must be absolute: {p}")
            uniq[C.rel(p)] = p
        self.paths = uniq
        self.declared = snapshot_files(list(uniq.values()))
        return dict(self.declared)

    def read_bytes(self, path: Path) -> bytes:
        rel = C.rel(path)
        if rel not in self.declared:
            raise InputsChangedError(f"undeclared input read during computation: {rel}")
        data = read_regular_nofollow(self.paths[rel])
        sha = sha256_bytes(data)
        if sha != self.declared[rel]:
            raise InputsChangedError(f"consumed input changed since the PRE snapshot: {rel}")
        self.read[rel] = sha
        return data

    def digest(self, path: Path) -> str:
        rel = C.rel(path)
        if rel not in self.declared:
            raise InputsChangedError(f"undeclared input: {rel}")
        return self.declared[rel]

    def assert_unchanged(self, label: str) -> dict[str, str]:
        after = snapshot_files(list(self.paths.values()))
        assert_unchanged(self.declared, after, label)
        return after

    def assert_all_read(self) -> None:
        unread = sorted(k for k in self.declared if k not in self.read)
        if unread:
            raise InputsChangedError(f"declared inputs never read during computation (set mismatch): {unread[:10]}")


def interpreter_provenance() -> dict[str, Any]:
    """In-process interpreter/runtime provenance of the generating process."""
    versions = {}
    for name in ("numpy", "matplotlib", "yaml"):
        try:
            versions[name] = __import__(name).__version__
        except Exception as exc:  # noqa: BLE001
            versions[name] = f"unavailable: {type(exc).__name__}"
    return {"executable": sys.executable, "executable_realpath": os.path.realpath(sys.executable), "python_version": sys.version.split()[0], "platform": platform.platform(), "machine": platform.machine(), "packages": versions, "note": "in-process provenance of the generating interpreter"}


# --- no-replace rename -------------------------------------------------------------------------------


def rename_noreplace(src: Path, dst: Path) -> None:
    """Atomic directory rename that NEVER replaces an existing destination (also an empty directory).

    Darwin: renamex_np(RENAME_EXCL); Linux: renameat2(RENAME_NOREPLACE) (glibc symbol or raw
    syscall); Windows: os.rename = MoveFileExW without MOVEFILE_REPLACE_EXISTING. Any other
    platform, or a kernel/filesystem refusing the flag, fails closed (StagingError) - there is
    no fallback to a replacing rename."""
    src_b, dst_b = os.fsencode(str(src)), os.fsencode(str(dst))
    if sys.platform == "darwin":
        libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
        fn = getattr(libc, "renamex_np", None)
        if fn is None:
            raise StagingError("no-replace rename unavailable: renamex_np missing")
        fn.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        fn.restype = ctypes.c_int
        rc = fn(src_b, dst_b, 4)  # RENAME_EXCL
        if rc != 0:
            err = ctypes.get_errno()
            if err == errno.EEXIST:
                raise FileExistsError(errno.EEXIST, f"destination exists (no-replace rename refused): {dst}")
            raise OSError(err, f"renamex_np failed: {os.strerror(err)}", str(src), None, str(dst))
        return
    if sys.platform.startswith("linux"):
        libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
        at_fdcwd, noreplace = -100, 1
        fn = getattr(libc, "renameat2", None)
        if fn is not None:
            fn.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
            fn.restype = ctypes.c_int
            rc = fn(at_fdcwd, src_b, at_fdcwd, dst_b, noreplace)
        else:
            nr = {"x86_64": 316, "aarch64": 276, "i686": 353, "i386": 353, "armv7l": 382}.get(platform.machine())
            if nr is None:
                raise StagingError(f"no-replace rename unavailable: renameat2 syscall number unknown for {platform.machine()}")
            syscall = libc.syscall
            syscall.restype = ctypes.c_long
            rc = syscall(ctypes.c_long(nr), ctypes.c_int(at_fdcwd), ctypes.c_char_p(src_b), ctypes.c_int(at_fdcwd), ctypes.c_char_p(dst_b), ctypes.c_uint(noreplace))
        if rc != 0:
            err = ctypes.get_errno()
            if err == errno.EEXIST:
                raise FileExistsError(errno.EEXIST, f"destination exists (no-replace rename refused): {dst}")
            if err in (errno.EINVAL, errno.ENOSYS, errno.ENOTSUP, getattr(errno, "EOPNOTSUPP", errno.ENOTSUP)):
                raise StagingError(f"no-replace rename unsupported by this kernel/filesystem (errno {err}); fail-closed")
            raise OSError(err, f"renameat2 failed: {os.strerror(err)}", str(src), None, str(dst))
        return
    if sys.platform == "win32":
        try:
            os.rename(str(src), str(dst))  # MoveFileExW without MOVEFILE_REPLACE_EXISTING: existing dst -> error
        except FileExistsError:
            raise
        except OSError as exc:
            if getattr(exc, "winerror", None) in (80, 183):
                raise FileExistsError(errno.EEXIST, f"destination exists (no-replace rename refused): {dst}") from exc
            raise
        return
    raise StagingError(f"no-replace rename unavailable on platform {sys.platform}; fail-closed")


# --- sidecar ------------------------------------------------------------------------------------------


def _safe_relative(rel: Any) -> str:
    if not isinstance(rel, str) or not rel or "\\" in rel or rel.startswith("/") or PurePosixPath(rel).is_absolute():
        raise StagingError(f"invalid artefact path entry: {rel!r}")
    parts = PurePosixPath(rel).parts
    if any(p in (".", "..") for p in parts) or STAGING_MARKER in rel or any(p.startswith(ARTIFACT_MANIFEST_PREFIX) for p in parts):
        raise StagingError(f"path traversal, staging marker or sidecar-like component in artefact path entry: {rel!r}")
    return rel


def _is_sha256(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and set(value) <= HEX64


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _tree_files(root: Path, *, sidecar: Path | None) -> list[Path]:
    """Regular files of a tree after refusing symlinks, special nodes and ANY sidecar-like entry (file, directory,
    special) other than the one expected sidecar."""
    files: list[Path] = []
    for p, st in _walk_entries(root):
        if stat.S_ISLNK(st.st_mode):
            raise StagingError(f"symlink inside the artefact tree: {p}")
        if p.name.startswith(ARTIFACT_MANIFEST_PREFIX) and p != sidecar:
            raise StagingError(f"sidecar-like entry inside the artefact tree (only the expected sidecar may exist): {p.relative_to(root).as_posix()}")
        if stat.S_ISDIR(st.st_mode):
            continue
        if not stat.S_ISREG(st.st_mode):
            raise StagingError(f"special node inside the artefact tree: {p}")
        files.append(p)
    return files


def write_artifact_manifest(staging: Path, stamp: str, *, provenance: dict[str, Any]) -> dict[str, Any]:
    """Sidecar written LAST: SHA-256 of every other regular file of the tree (relative paths, strict JSON).
    Returns the sealed manifest (deep-exact expectation), its bytes digest and path."""
    staging = Path(staging)
    name = sidecar_name(stamp)
    missing = [k for k in REQUIRED_PROVENANCE if k not in provenance]
    if missing:
        raise StagingError(f"artifact provenance lacks {missing}")
    files = []
    seen: set[str] = set()
    for p in _tree_files(staging, sidecar=None):
        rel = _safe_relative(p.relative_to(staging).as_posix())
        if rel in seen:
            raise StagingError(f"duplicate artefact path: {rel}")
        seen.add(rel)
        sha, size = sha256_regular_nofollow(p)
        files.append({"path": rel, "sha256": sha, "bytes": size})
    if not files:
        raise StagingError("no output produced: refusing to seal an empty artefact")
    payload = {"schema_version": MANIFEST_SCHEMA_VERSION, "stamp": stamp, "sidecar": name, "generated_at_utc": C.utc_now(), "file_count": len(files), "total_bytes": sum(f["bytes"] for f in files), "files": files, "self_excluded": True, "note": "hashes of every other file of this directory (relative paths); this sidecar is the only excluded entry and is not self-hashed", **provenance}
    text = dumps_strict(payload)
    path = staging / name
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing sidecar: {path}")
    data = text.encode("utf-8")
    path.write_bytes(data)
    return {"path": path, "manifest": loads_strict(text), "sha256": sha256_bytes(data), "bytes": len(data)}


def validate_manifest_schema(manifest: Any, *, expected_stamp: str | None = None) -> None:
    """Strict schema/type validation of an artifact manifest (NaN/Inf and staging markers already rejected by loads_strict)."""
    if not isinstance(manifest, dict):
        raise StagingError("artifact manifest is not a mapping")
    if manifest.get("schema_version") != MANIFEST_SCHEMA_VERSION or manifest.get("self_excluded") is not True:
        raise StagingError("artifact manifest schema_version/self_excluded invalid")
    stamp = manifest.get("stamp")
    try:
        validate_stamp(stamp)
    except StagingError as exc:
        raise StagingError(f"artifact manifest stamp invalid: {exc}") from exc
    if expected_stamp is not None and stamp != expected_stamp:
        raise StagingError(f"artifact manifest stamp {stamp!r} != expected {expected_stamp!r}")
    if manifest.get("sidecar") != sidecar_name(stamp) or not isinstance(manifest.get("generated_at_utc"), str):
        raise StagingError("artifact manifest sidecar name / generated_at_utc invalid")
    missing = [k for k in REQUIRED_PROVENANCE if k not in manifest]
    if missing:
        raise StagingError(f"artifact manifest lacks provenance {missing}")
    src = manifest.get("source_analysis")
    if not (isinstance(manifest.get("kind"), str) and manifest["kind"]) or not _is_sha256(manifest.get("script_sha256")) or not isinstance(src, dict) or not _is_sha256(src.get("sha256")) or not isinstance(src.get("path"), str) or manifest.get("inputs_unchanged") is not True or not isinstance(manifest.get("interpreter"), dict) or not isinstance(manifest.get("git_head"), str) or not _is_int(manifest.get("consumed_inputs")) or manifest["consumed_inputs"] < 0:
        raise StagingError("artifact manifest provenance types invalid")
    files = manifest.get("files")
    if not isinstance(files, list) or not files or not _is_int(manifest.get("file_count")) or manifest["file_count"] != len(files):
        raise StagingError("artifact manifest file_count/files invalid")
    total = 0
    seen: set[str] = set()
    for f in files:
        if not isinstance(f, dict) or not _is_sha256(f.get("sha256")) or not _is_int(f.get("bytes")) or f["bytes"] < 0:
            raise StagingError(f"artifact manifest file entry invalid: {f}")
        rel = _safe_relative(f.get("path"))
        if rel in seen:
            raise StagingError(f"duplicate path in the artifact manifest: {rel}")
        seen.add(rel)
        total += f["bytes"]
    if manifest.get("total_bytes") != total:
        raise StagingError(f"artifact manifest total_bytes {manifest.get('total_bytes')} != {total}")


def verify_artifact_dir(final_dir: Path, *, anchor: Path | None = None, expected_stamp: str | None = None, strict: bool = False, expected_manifest: dict[str, Any] | None = None, expected_sidecar_sha256: str | None = None) -> dict[str, Any]:
    """Re-verify a sealed (or staged) artefact tree against its single sidecar (read-only): symlink-free chain,
    no sidecar-like or special entries, schema, digests, sizes, duplicates, traversal, unlisted files; when an
    expectation is given the sidecar must be BYTE-EXACT (digest) and DEEP-EXACT (content) to the sealed one.
    ``strict`` raises when not ok."""
    final_dir = Path(final_dir)
    anchor = Path(anchor) if anchor is not None else final_dir.parent
    if final_dir.is_symlink() or not final_dir.is_dir():
        raise StagingError(f"artefact directory missing or symlink: {final_dir}")
    assert_no_symlink_components(final_dir, anchor, "artefact dir")
    assert_regular_chain(final_dir, kind="dir")
    if expected_stamp is not None:
        sidecar = final_dir / sidecar_name(expected_stamp)
    else:
        found = [p for p in final_dir.iterdir() if not p.is_symlink() and p.is_file() and SIDECAR_RE.fullmatch(p.name)]
        if len(found) != 1:
            raise StagingError(f"expected exactly one regular sidecar artifact_manifest_<stamp>.json at the top level of {final_dir}, found {len(found)}")
        sidecar = found[0]
    tree = _tree_files(final_dir, sidecar=sidecar)  # refuses symlinks, special nodes, any other sidecar-like entry
    if sidecar not in tree:
        raise StagingError(f"expected sidecar missing or not a regular file: {sidecar.name}")
    raw = read_regular_nofollow(sidecar)
    sidecar_sha = sha256_bytes(raw)
    if expected_sidecar_sha256 is not None and sidecar_sha != expected_sidecar_sha256:
        raise ArtifactVerificationError(f"sidecar bytes differ from the sealed sidecar (provenance substitution): {sidecar.name}")
    manifest = loads_strict(raw)
    stamp = SIDECAR_RE.fullmatch(sidecar.name).group(1)
    validate_manifest_schema(manifest, expected_stamp=stamp)
    if expected_manifest is not None and manifest != expected_manifest:
        raise ArtifactVerificationError(f"sidecar content differs from the sealed manifest (deep-exact check failed): {sidecar.name}")
    mismatches: list[str] = []
    listed: set[str] = set()
    for f in manifest["files"]:
        rel = f["path"]
        listed.add(rel)
        p = final_dir / rel
        try:
            assert_no_symlink_components(p, final_dir, "artefact file")
            sha, size = sha256_regular_nofollow(p)
        except StagingError:
            mismatches.append(rel)
            continue
        if size != f["bytes"] or sha != f["sha256"]:
            mismatches.append(rel)
    extra = sorted(p.relative_to(final_dir).as_posix() for p in tree if p != sidecar and p.relative_to(final_dir).as_posix() not in listed)
    result = {"sidecar": sidecar.name, "sidecar_sha256": sidecar_sha, "stamp": stamp, "files": len(manifest["files"]), "total_bytes": manifest["total_bytes"], "mismatches": sorted(mismatches), "unlisted": extra, "ok": not mismatches and not extra}
    if strict and not result["ok"]:
        raise ArtifactVerificationError(f"artefact verification failed for {final_dir.name}: mismatches={result['mismatches'][:10]} unlisted={result['unlisted'][:10]}")
    return result


# --- publish ------------------------------------------------------------------------------------------


QUARANTINE_MARKER = ".quarantine_"


def quarantine_entry(entry: Path) -> Path:
    """Move WHATEVER occupies ``entry`` (directory, regular file, symlink - also broken -, FIFO/special node) to a
    unique quarantine sibling with the no-replace primitive. rename(2) acts on the directory entry itself and never
    follows a symlink source, so a planted link is moved, not its target. Never clobbers."""
    entry = Path(entry)
    if not os.path.lexists(entry):
        raise StagingError(f"nothing to quarantine at {entry}")
    for n in range(1000):
        target = entry.parent / f"{QUARANTINE_MARKER}{entry.name}_{os.getpid()}_{n}"
        if os.path.lexists(target):
            continue
        try:
            rename_noreplace(entry, target)
        except FileExistsError:
            continue
        if os.path.lexists(entry):
            raise StagingError(f"entry still present after quarantine rename: {entry}")
        return target
    raise StagingError(f"could not find a free quarantine name for {entry}")


quarantine_dir = quarantine_entry  # backward-compatible alias


def publish_artifact(final_dir: Path, *, anchor: Path, stamp: str, build: Callable[[Path], dict[str, Any]], before_publish: Callable[[], Any] | None = None, after_seal: Callable[[Path], Any] | None = None, after_publish: Callable[[Path], Any] | None = None) -> dict[str, Any]:
    """Build in an exclusive staging sibling, seal with the single sidecar, re-check inputs (``before_publish``),
    verify the staged tree deep/byte-exact against the sealed sidecar immediately before the NO-REPLACE rename,
    verify the published tree again; an invalid published tree is quarantined (no-replace) and the call raises.
    Returns only final-directory facts (never a staging path) incl. the sidecar digest; raises on any failure.
    ``after_seal`` / ``after_publish`` are adversarial-test hooks (after the sidecar is written / after the rename)."""
    stamp = validate_stamp(stamp)
    final_dir, anchor = Path(final_dir), Path(anchor)
    if final_dir.name != stamp:
        raise StagingError(f"final directory basename {final_dir.name!r} must equal the stamp {stamp!r}")
    parent = final_dir.parent
    assert_no_symlink_components(parent, anchor, "output parent (before mkdir)")
    parent.mkdir(parents=True, exist_ok=True)
    assert_no_symlink_components(final_dir, anchor, "output dir (after mkdir)")
    assert_regular_chain(parent, kind="dir")
    if final_dir.is_symlink() or final_dir.exists():
        raise FileExistsError(f"refusing to reuse existing output directory or symlink: {final_dir}")
    staging = parent / f"{STAGING_MARKER}{stamp}_{os.getpid()}"
    if staging.is_symlink() or staging.exists():
        raise FileExistsError(f"staging directory already exists (another process?): {staging}")
    staging.mkdir()  # exclusive: raises if it appeared meanwhile
    published = False
    try:
        provenance = build(staging)
        if not isinstance(provenance, dict):
            raise StagingError("build must return the provenance mapping of the sidecar")
        sealed = write_artifact_manifest(staging, stamp, provenance=provenance)
        if after_seal is not None:
            after_seal(staging)
        if before_publish is not None:
            before_publish()  # inputs re-checked BEFORE the last staged verification
        # staged tree verified deep-exact + byte-exact against the sealed sidecar IMMEDIATELY before the rename
        staged = verify_artifact_dir(staging, anchor=parent, expected_stamp=stamp, strict=True, expected_manifest=sealed["manifest"], expected_sidecar_sha256=sealed["sha256"])
        if final_dir.is_symlink() or final_dir.exists():
            raise FileExistsError(f"output directory appeared during generation: {final_dir}")
        rename_noreplace(staging, final_dir)
        published = True
    except BaseException:
        if not published and staging.exists() and not staging.is_symlink():
            shutil.rmtree(staging, ignore_errors=True)
        raise
    try:
        if after_publish is not None:
            after_publish(final_dir)
        final = verify_artifact_dir(final_dir, anchor=anchor, expected_stamp=stamp, strict=True, expected_manifest=sealed["manifest"], expected_sidecar_sha256=sealed["sha256"])
    except BaseException as exc:
        # whatever now occupies the authoritative stamp (directory, file, symlink - also broken -, special node) is
        # moved away without following it; the stamp is declared free ONLY after a lexical (lexists) re-check
        quarantined: Path | None = None
        try:
            if os.path.lexists(final_dir):
                quarantined = quarantine_entry(final_dir)
            if os.path.lexists(final_dir):
                raise StagingError(f"authoritative stamp still occupied after quarantine: {final_dir}")
        except BaseException as qexc:
            raise ArtifactVerificationError(f"published tree failed verification AND the quarantine failed: the authoritative stamp {stamp!r} may still be occupied ({final_dir}); verification error: {exc}; quarantine error: {qexc}") from exc
        where = f"moved to {quarantined.name}" if quarantined is not None else "no entry found under the stamp"
        raise ArtifactVerificationError(f"published tree failed verification; the entry under the authoritative stamp {stamp!r} was {where} and the stamp is now free (lexists re-checked): {exc}") from exc
    result = {"final_dir": C.rel(final_dir), "stamp": stamp, "artifact": final, "sidecar_sha256": sealed["sha256"], "staged_files": staged["files"], "staged_total_bytes": staged["total_bytes"]}
    assert_no_staging_paths(result)
    return result
