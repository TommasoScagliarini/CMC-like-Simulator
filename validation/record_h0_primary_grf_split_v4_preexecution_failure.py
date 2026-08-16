"""Record the terminal pre-execution integrity failure of H0 V4.

The V4 execution lock is immutable.  After it was published, the runner was
hardened and its preflight receipt was deliberately removed as stale.  The
normal V4 verifier therefore stopped before creating the attempt root.  This
recorder preserves that exact fail-closed outcome without altering V4.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
LOCK = REPO_ROOT / "validation/h0_primary_grf_split_v4_execution_lock.json"
RUNNER = REPO_ROOT / "validation/run_h0_primary_grf_split_v4_full_mean.py"
DESTINATION = (
    REPO_ROOT
    / "validation/h0_primary_grf_split_v4_preexecution_failure.json"
)


class V4PreexecutionFailureRecordError(RuntimeError):
    """Raised if the observed V4 failure is not the exact known boundary."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _record(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise V4PreexecutionFailureRecordError(f"missing regular file: {path}")
    return {
        "path": path.resolve().relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
    }


def _strict_mapping(path: Path) -> dict[str, Any]:
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(
            ValueError(f"non-finite JSON token: {token}")
        ),
        object_pairs_hook=reject_duplicates,
    )
    if not isinstance(value, Mapping):
        raise V4PreexecutionFailureRecordError("V4 lock is not a JSON object")
    return dict(value)


def _require_finite(value: Any) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise V4PreexecutionFailureRecordError("non-finite receipt value")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise V4PreexecutionFailureRecordError("invalid receipt key")
            _require_finite(child)
        return
    if isinstance(value, list):
        for child in value:
            _require_finite(child)
        return
    raise V4PreexecutionFailureRecordError("non-JSON receipt value")


def build() -> dict[str, Any]:
    if os.path.lexists(DESTINATION):
        raise V4PreexecutionFailureRecordError(
            f"refusing to clobber: {DESTINATION}"
        )
    lock = _strict_mapping(LOCK)
    if (
        lock.get("status") != "H0_PRIMARY_GRF_SPLIT_V4_EXECUTION_FROZEN"
        or lock.get("protocol_id") != "AB06_H0_PRIMARY_GRF_SPLIT_V4_FULL_MEAN"
    ):
        raise V4PreexecutionFailureRecordError("unexpected V4 lock identity")
    locked_runner = lock.get("sources", {}).get("v4", {}).get("runner")
    locked_preflight = lock.get("inputs", {}).get("preflight_receipt")
    if not isinstance(locked_runner, Mapping) or not isinstance(
        locked_preflight, Mapping
    ):
        raise V4PreexecutionFailureRecordError("V4 lock closure is malformed")
    current_runner = _record(RUNNER)
    preflight_path = REPO_ROOT / str(locked_preflight.get("path", ""))
    run_root = REPO_ROOT / str(lock.get("run_root", ""))
    if (
        current_runner["sha256"] == locked_runner.get("sha256")
        or current_runner["size_bytes"] == locked_runner.get("size_bytes")
        or preflight_path.exists()
        or run_root.exists()
    ):
        raise V4PreexecutionFailureRecordError(
            "observed state is not the known pre-execution integrity failure"
        )
    payload = {
        "schema_version": 1,
        "status": "FAIL_H0_PRIMARY_SPLIT_V4_PREEXECUTION_INTEGRITY",
        "passed": False,
        "protocol_id": lock["protocol_id"],
        "failure_stage": "verify_lock_before_attempt_claim",
        "failure": "post_lock_runner_drift_and_stale_preflight_removal",
        "execution_error": "V4FreezeError: V4 preflight source drifted: runner",
        "execution_lock": _record(LOCK),
        "locked_runner": dict(locked_runner),
        "observed_runner": current_runner,
        "locked_preflight_receipt": dict(locked_preflight),
        "observed_preflight_receipt_exists": False,
        "run_root_exists": False,
        "attempt_claim_created": False,
        "v4_execution_started": False,
        "seed125_semantic_accessed": False,
        "actor_update_candidates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "v4_retry_allowed": False,
        "next_stage": "NEW_PROTOCOL_REQUIRED_NO_V4_RETRY",
    }
    _require_finite(payload)
    return payload


def publish() -> dict[str, Any]:
    payload = build()
    encoded = json.dumps(
        payload,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8") + b"\n"
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    descriptor = os.open(str(DESTINATION), flags, 0o600)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        raise
    return payload


if __name__ == "__main__":
    print(json.dumps(publish(), indent=2, sort_keys=True, allow_nan=False))
