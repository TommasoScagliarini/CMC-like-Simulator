"""Crash-safe forensic artifact writer for H0 rollout protocols.

The helper is deliberately independent from OpenSim, Torch, RLlib, and any
specific H0 lineage.  A rollout driver can publish its start receipt, one
strict-JSON artifact per completed policy step, and the final aggregate
artifacts without ever replacing an existing result.

The lifecycle is intentionally one-way::

    run_start.json
      -> steps/000001.json, ...
      -> trace.json, partial_summary.json, summary.json
      -> gate.json

At any point after ``run_start.json`` a terminal ``failure.json`` can be
published.  The failure receipt records the last contiguous completed step
and content-addressed records for every artifact that had already become
visible.  Aggregate artifacts must exist and agree with the per-step journal
before a gate callback can run.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


SCHEMA_VERSION = 1
FAIL_STATUS = "FAIL_H0_FORENSIC_ROLLOUT"
_STEP_NAME = re.compile(r"^(?P<step>[0-9]{6})\.json$")


class ForensicRolloutError(RuntimeError):
    """Raised when a forensic artifact cannot be published or verified."""


def _reject_constant(token: str) -> None:
    raise ForensicRolloutError(f"non-finite JSON constant: {token}")


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ForensicRolloutError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _finite_json_tree(value: Any) -> bool:
    if value is None or isinstance(value, (bool, int, str)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, Mapping):
        return all(
            isinstance(key, str) and _finite_json_tree(child)
            for key, child in value.items()
        )
    if isinstance(value, (list, tuple)):
        return all(_finite_json_tree(child) for child in value)
    return False


def canonical_json_bytes(payload: Any) -> bytes:
    """Encode one finite JSON value using the canonical forensic format."""

    if not _finite_json_tree(payload):
        raise ForensicRolloutError("payload is not a finite strict-JSON value")
    try:
        return (
            json.dumps(
                payload,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ForensicRolloutError(f"payload is not strict JSON: {exc}") from exc


def strict_json_load(path: str | Path) -> Any:
    """Load JSON while rejecting duplicate keys and non-finite constants."""

    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(
            source.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_constant,
        )
    except ForensicRolloutError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ForensicRolloutError(f"cannot read strict JSON {source}: {exc}") from exc
    canonical_json_bytes(payload)
    return payload


def sha256_file(path: str | Path) -> str:
    """Return the SHA-256 digest of an existing regular file."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ForensicRolloutError(f"artifact is not a regular file: {source}")
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_record(
    path: str | Path,
    *,
    artifact_root: str | Path,
) -> dict[str, Any]:
    """Create a relative path/size/hash record for one published artifact."""

    source = Path(path).expanduser().resolve()
    root = Path(artifact_root).expanduser().resolve()
    try:
        relative = source.relative_to(root)
    except ValueError as exc:
        raise ForensicRolloutError(
            f"artifact escaped the declared root: {source}"
        ) from exc
    if not source.is_file():
        raise ForensicRolloutError(f"artifact is not a regular file: {source}")
    return {
        "path": relative.as_posix(),
        "sha256": sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def _fsync_directory(path: Path) -> None:
    """Best-effort directory fsync (not supported by every platform)."""

    try:
        descriptor = os.open(str(path), os.O_RDONLY)
    except OSError:  # pragma: no cover - platform/filesystem dependent.
        return
    try:
        os.fsync(descriptor)
    except OSError:  # pragma: no cover - platform/filesystem dependent.
        pass
    finally:
        os.close(descriptor)


def _publish_temporary_exclusive(temporary: Path, destination: Path) -> None:
    """Claim ``destination`` without races, then atomically replace the claim."""

    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(str(destination), flags, 0o600)
    except FileExistsError as exc:
        raise ForensicRolloutError(f"refusing to clobber: {destination}") from exc
    else:
        os.close(descriptor)
    # If replace fails, the exclusive zero-byte claim remains.  That is
    # deliberately fail-closed: a later process cannot silently reuse the
    # same artifact identity after an interrupted publication.
    os.replace(str(temporary), str(destination))
    _fsync_directory(destination.parent)


def write_json_exclusive(path: str | Path, payload: Any) -> Path:
    """Publish strict JSON with fsync, atomic replace, and no-clobber."""

    destination = Path(path).expanduser().resolve()
    encoded = canonical_json_bytes(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise ForensicRolloutError(f"refusing to clobber: {destination}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".tmp",
        dir=str(destination.parent),
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, destination)
        return destination
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ForensicRolloutError(f"{label} must be a JSON object")
    result = dict(value)
    canonical_json_bytes(result)
    return result


class ForensicRolloutWriter:
    """One-way, crash-forensic writer for a single rollout directory."""

    def __init__(
        self,
        run_directory: str | Path,
        *,
        artifact_root: str | Path | None = None,
    ) -> None:
        self.run_directory = Path(run_directory).expanduser().resolve()
        default_root = self.run_directory.parent
        self.artifact_root = (
            Path(default_root if artifact_root is None else artifact_root)
            .expanduser()
            .resolve()
        )
        try:
            self.run_directory.relative_to(self.artifact_root)
        except ValueError as exc:
            raise ForensicRolloutError(
                "run directory must be inside artifact_root"
            ) from exc
        self._append_step_cache: int | None = None

    @property
    def run_start_path(self) -> Path:
        return self.run_directory / "run_start.json"

    @property
    def steps_directory(self) -> Path:
        return self.run_directory / "steps"

    @property
    def trace_path(self) -> Path:
        return self.run_directory / "trace.json"

    @property
    def partial_summary_path(self) -> Path:
        return self.run_directory / "partial_summary.json"

    @property
    def summary_path(self) -> Path:
        return self.run_directory / "summary.json"

    @property
    def gate_path(self) -> Path:
        return self.run_directory / "gate.json"

    @property
    def failure_path(self) -> Path:
        return self.run_directory / "failure.json"

    def _record(self, path: Path) -> dict[str, Any]:
        return artifact_record(path, artifact_root=self.artifact_root)

    def _require_started(self) -> dict[str, Any]:
        if not self.run_start_path.is_file():
            raise ForensicRolloutError("run_start.json has not been published")
        value = strict_json_load(self.run_start_path)
        return _mapping(value, "run_start.json")

    def _require_open(self) -> None:
        if os.path.lexists(self.failure_path):
            raise ForensicRolloutError("rollout already has a failure receipt")

    def _step_paths(self) -> list[Path]:
        if not self.steps_directory.exists():
            return []
        if not self.steps_directory.is_dir():
            raise ForensicRolloutError("steps path is not a directory")
        indexed: list[tuple[int, Path]] = []
        for path in self.steps_directory.iterdir():
            # Interrupted temporary files never count as completed steps.
            if path.name.startswith(".") and path.name.endswith(".tmp"):
                continue
            match = _STEP_NAME.fullmatch(path.name)
            if match is None or not path.is_file():
                raise ForensicRolloutError(
                    f"unexpected entry in steps directory: {path.name}"
                )
            indexed.append((int(match.group("step")), path))
        indexed.sort()
        expected = list(range(1, len(indexed) + 1))
        observed = [step for step, _path in indexed]
        if observed != expected:
            raise ForensicRolloutError(f"step journal is not contiguous: {observed}")
        paths: list[Path] = []
        for step, path in indexed:
            row = _mapping(strict_json_load(path), path.name)
            if row.get("step") != step:
                raise ForensicRolloutError(
                    f"{path.name} embeds step={row.get('step')!r}"
                )
            paths.append(path)
        return paths

    @property
    def last_completed_step(self) -> int:
        """Return the largest contiguous, strict, durably published step."""

        return len(self._step_paths())

    def _last_step_for_append(self) -> int:
        # A newly created or recovered writer scans the durable journal once.
        # Subsequent appends remain O(1); finalization and failure publication
        # independently rescan and validate the complete visible prefix.
        if self._append_step_cache is None:
            self._append_step_cache = self.last_completed_step
        return self._append_step_cache

    def start(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Publish ``run_start.json`` exactly once."""

        value = _mapping(payload, "run start payload")
        if self.run_directory.exists() and any(self.run_directory.iterdir()):
            raise ForensicRolloutError(
                f"rollout directory is already occupied: {self.run_directory}"
            )
        path = write_json_exclusive(self.run_start_path, value)
        return self._record(path)

    def write_step(
        self,
        step: int,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Publish the next contiguous ``steps/NNNNNN.json`` artifact."""

        self._require_started()
        self._require_open()
        if any(
            os.path.lexists(path)
            for path in (
                self.trace_path,
                self.partial_summary_path,
                self.summary_path,
                self.gate_path,
            )
        ):
            raise ForensicRolloutError(
                "cannot append a step after finalization has started"
            )
        if type(step) is not int or step < 1 or step > 999999:
            raise ForensicRolloutError("step must be an integer in [1, 999999]")
        expected = self._last_step_for_append() + 1
        if step != expected:
            raise ForensicRolloutError(
                f"next step must be {expected}, requested {step}"
            )
        row = _mapping(payload, f"step {step} payload")
        if "step" in row and row["step"] != step:
            raise ForensicRolloutError(
                f"step payload embeds {row['step']!r}, expected {step}"
            )
        row["step"] = step
        path = write_json_exclusive(
            self.steps_directory / f"{step:06d}.json",
            row,
        )
        self._append_step_cache = step
        return self._record(path)

    def _step_rows(self) -> list[dict[str, Any]]:
        return [
            _mapping(strict_json_load(path), path.name) for path in self._step_paths()
        ]

    def finalize_before_gate(
        self,
        *,
        partial_summary: Mapping[str, Any],
        summary: Mapping[str, Any],
        trace: Sequence[Mapping[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Publish trace and both summaries before any gate can execute.

        By default ``trace.json`` is assembled from the immutable step journal.
        A caller-supplied trace is accepted only when its canonical JSON is
        exactly equal to that journal, preventing a lossy final aggregate.
        All three values are validated before the first artifact is published.
        """

        self._require_started()
        self._require_open()
        if os.path.lexists(self.gate_path):
            raise ForensicRolloutError("gate exists before finalization")
        targets = (
            self.trace_path,
            self.partial_summary_path,
            self.summary_path,
        )
        if any(os.path.lexists(path) for path in targets):
            raise ForensicRolloutError(
                "final aggregate publication already started/no retry"
            )
        rows = self._step_rows()
        if not rows:
            raise ForensicRolloutError("cannot finalize a rollout with zero steps")
        trace_value: list[dict[str, Any]]
        if trace is None:
            trace_value = rows
        else:
            trace_value = [
                _mapping(row, f"trace row {index}")
                for index, row in enumerate(trace, start=1)
            ]
            if canonical_json_bytes(trace_value) != canonical_json_bytes(rows):
                raise ForensicRolloutError(
                    "trace does not exactly match the per-step journal"
                )
        partial_value = _mapping(partial_summary, "partial summary")
        summary_value = _mapping(summary, "summary")
        # Validate every payload before allowing a partially published final
        # aggregate due to a deterministic schema/non-finite error.
        for value in (trace_value, partial_value, summary_value):
            canonical_json_bytes(value)
        published = {
            "trace": self._record(write_json_exclusive(self.trace_path, trace_value)),
            "partial_summary": self._record(
                write_json_exclusive(
                    self.partial_summary_path,
                    partial_value,
                )
            ),
            "summary": self._record(
                write_json_exclusive(self.summary_path, summary_value)
            ),
        }
        return published

    def finalized_artifact_records(self) -> dict[str, Any]:
        """Verify final aggregates and return their content records."""

        self._require_started()
        paths = {
            "trace": self.trace_path,
            "partial_summary": self.partial_summary_path,
            "summary": self.summary_path,
        }
        missing = [name for name, path in paths.items() if not path.is_file()]
        if missing:
            raise ForensicRolloutError(
                f"rollout is not ready for a gate; missing {missing}"
            )
        trace = strict_json_load(self.trace_path)
        rows = self._step_rows()
        if canonical_json_bytes(trace) != canonical_json_bytes(rows):
            raise ForensicRolloutError("trace no longer matches the per-step journal")
        _mapping(strict_json_load(self.partial_summary_path), "partial summary")
        _mapping(strict_json_load(self.summary_path), "summary")
        return {name: self._record(path) for name, path in paths.items()}

    def publish_gate(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Publish a gate only after all forensic aggregates are durable."""

        self._require_open()
        self.finalized_artifact_records()
        value = _mapping(payload, "gate payload")
        path = write_json_exclusive(self.gate_path, value)
        return self._record(path)

    def run_gate(
        self,
        gate: Callable[[dict[str, Any]], Mapping[str, Any]],
    ) -> dict[str, Any]:
        """Run ``gate`` after finalization and publish its strict result."""

        self._require_open()
        artifacts_before = self.finalized_artifact_records()
        result = gate(dict(artifacts_before))
        artifacts_after = self.finalized_artifact_records()
        if canonical_json_bytes(artifacts_after) != canonical_json_bytes(
            artifacts_before
        ):
            raise ForensicRolloutError(
                "gate callback changed a finalized forensic artifact"
            )
        return self.publish_gate(_mapping(result, "gate result"))

    def artifact_records(self) -> dict[str, Any]:
        """Return records for every currently published non-failure artifact."""

        self._require_started()
        result: dict[str, Any] = {"run_start": self._record(self.run_start_path)}
        result["steps"] = [self._record(path) for path in self._step_paths()]
        optional = {
            "trace": self.trace_path,
            "partial_summary": self.partial_summary_path,
            "summary": self.summary_path,
            "gate": self.gate_path,
        }
        for name, path in optional.items():
            if os.path.lexists(path):
                # A zero-byte exclusive claim left by an interrupted replace is
                # not a valid published artifact and must fail closed here.
                strict_json_load(path)
                result[name] = self._record(path)
        return result

    @staticmethod
    def _error_payload(
        error: BaseException | Mapping[str, Any] | str,
    ) -> dict[str, Any]:
        if isinstance(error, BaseException):
            value = {
                "type": type(error).__name__,
                "message": str(error),
            }
        elif isinstance(error, str):
            value = {"type": "RuntimeError", "message": error}
        else:
            value = _mapping(error, "failure error")
        return _mapping(value, "failure error")

    def publish_failure(
        self,
        *,
        end_reason: str,
        error: BaseException | Mapping[str, Any] | str,
        status: str = FAIL_STATUS,
        details: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Publish one terminal failure receipt for the visible prefix."""

        self._require_started()
        if os.path.lexists(self.failure_path):
            raise ForensicRolloutError("failure receipt already exists/no retry")
        if not isinstance(end_reason, str) or not end_reason.strip():
            raise ForensicRolloutError("end_reason must be a non-empty string")
        if not isinstance(status, str) or not status.strip():
            raise ForensicRolloutError("failure status must be a non-empty string")
        details_value = (
            None if details is None else _mapping(details, "failure details")
        )
        receipt = {
            "schema_version": SCHEMA_VERSION,
            "status": status,
            "passed": False,
            "last_completed_step": self.last_completed_step,
            "end_reason": end_reason,
            "error": self._error_payload(error),
            "artifacts": self.artifact_records(),
            "details": details_value,
        }
        path = write_json_exclusive(self.failure_path, receipt)
        return self._record(path)


__all__ = [
    "FAIL_STATUS",
    "SCHEMA_VERSION",
    "ForensicRolloutError",
    "ForensicRolloutWriter",
    "artifact_record",
    "canonical_json_bytes",
    "sha256_file",
    "strict_json_load",
    "write_json_exclusive",
]
