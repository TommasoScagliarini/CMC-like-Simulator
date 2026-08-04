#!/usr/bin/env python3
"""Fail-closed post-run audit for hidden RLlib EnvRunner restarts.

The PPO supervisor records child-process restarts, but RLlib may independently
restart a failed ``SingleAgentEnvRunner`` inside one otherwise successful
``algo.train()`` call.  This audit combines the run summary, any persisted
``fault_tolerance`` counters, and the Ray driver log scoped to the worker PID.

It is intentionally post-run only: a missing/incomplete summary or an
unidentifiable driver log is a failed audit, not an implicit clean result.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = 1
DEFAULT_OUTPUT_NAME = "env_runner_restart_audit.json"
_MAX_FINDINGS = 100

_RESTART_COUNTER_KEYS = {
    "num_remote_worker_restarts",
    "num_remote_env_runner_restarts",
    "num_env_runner_restarts",
}
_HEALTHY_COUNTER_KEYS = {
    "num_healthy_workers",
    "num_healthy_env_runners",
}

_POSITIVE_RESTART_RE = re.compile(r"\bnum_restarts\s*[:=]\s*([1-9]\d*)\b", re.I)
_ZERO_RESTART_RE = re.compile(r"\bnum_restarts\s*[:=]\s*0\b", re.I)
_RAY_ACTOR_ERROR_RE = re.compile(r"\bRayActorError\b", re.I)
_ENV_RUNNER_FAILURE_RE = re.compile(
    r"\b(?:SingleAgentEnvRunner|EnvRunner)\b.*\b(?:failed|failure|died|dead|"
    r"restarting|restarted|unexpected(?:ly)?\s+exit(?:ed)?)\b",
    re.I,
)
_RESTART_ENV_RUNNER_RE = re.compile(
    r"\b(?:restart(?:ing|ed)?|reconstruct(?:ing|ed)?)\b.*"
    r"\b(?:SingleAgentEnvRunner|EnvRunner)\b",
    re.I,
)
_UNEXPECTED_WORKER_EXIT_RE = re.compile(
    r"\bworker\b.*\b(?:died unexpectedly|unexpectedly exited|"
    r"unexpected exit|crashed)\b",
    re.I,
)
_DEAD_ACTOR_STATE_RE = re.compile(r"\bstate\s*:\s*DEAD\b", re.I)
_INTENDED_RAY_SHUTDOWN_RE = re.compile(
    r"exit_type=INTENDED_USER_EXIT|Shutdown by ray\.shutdown\(\)", re.I
)
_ENV_RUNNER_LIFECYCLE_RE = re.compile(
    r"\bSingleAgentEnvRunner\b|\bnum_restarts\s*[:=]", re.I
)
_KNOWN_BENIGN_MAX_RESTART_WARNING_RE = re.compile(
    r"constructor arguments.*max_restarts\s*>\s*0.*restart will fail", re.I
)


def _read_json_dict(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}
    return value if isinstance(value, dict) else {}


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _empty_list_or_missing(value: Any) -> bool:
    return value is None or (isinstance(value, list) and not value)


def _walk_mapping(
    value: Any,
    *,
    source: str,
    path: tuple[str, ...] = (),
) -> Iterable[dict[str, Any]]:
    if isinstance(value, Mapping):
        for key, nested in value.items():
            key_text = str(key)
            nested_path = (*path, key_text)
            if key_text in _RESTART_COUNTER_KEYS | _HEALTHY_COUNTER_KEYS:
                numeric = _finite_number(nested)
                yield {
                    "source": source,
                    "json_path": "/" + "/".join(nested_path),
                    "key": key_text,
                    "value": numeric,
                    "raw_value": nested if numeric is None else None,
                }
            yield from _walk_mapping(
                nested,
                source=source,
                path=nested_path,
            )
    elif isinstance(value, list):
        for index, nested in enumerate(value):
            yield from _walk_mapping(
                nested,
                source=source,
                path=(*path, str(index)),
            )


def _load_iteration_rows(path: Path) -> tuple[list[dict[str, Any]], list[int]]:
    rows: list[dict[str, Any]] = []
    invalid_lines: list[int] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return rows, invalid_lines
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            invalid_lines.append(line_number)
            continue
        if isinstance(row, dict):
            rows.append(row)
        else:
            invalid_lines.append(line_number)
    return rows, invalid_lines


def _expected_env_runners(summary: Mapping[str, Any], run_dir: Path) -> int | None:
    direct = _finite_number(summary.get("num_env_runners"))
    if direct is not None and direct >= 0 and direct.is_integer():
        return int(direct)

    snapshot = run_dir / "training_cfg.resolved.yaml"
    try:
        lines = snapshot.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    in_parallelism = False
    for line in lines:
        if line and not line.startswith((" ", "\t")):
            in_parallelism = line.strip() == "parallelism:"
            continue
        if not in_parallelism:
            continue
        match = re.match(r"\s*num_env_runners\s*:\s*(\d+)\s*$", line)
        if match:
            return int(match.group(1))
    return None


def _resolve_driver_pid(
    run_dir: Path,
    summary: Mapping[str, Any],
    explicit_pid: int | None,
) -> tuple[int | None, str | None]:
    if explicit_pid is not None:
        return int(explicit_pid), "cli"

    watchdog = _read_json_dict(run_dir / "watchdog_state.json")
    candidates = (
        (watchdog.get("pid"), "watchdog_state.json"),
        (summary.get("worker_pid"), "summary.worker_pid"),
        (summary.get("pid"), "summary.pid"),
    )
    for raw, source in candidates:
        try:
            pid = int(raw)
        except (TypeError, ValueError):
            continue
        if pid > 0:
            return pid, source
    return None, None


def _driver_logs_for_pid(
    ray_log_dir: Path,
    pid: int | None,
) -> tuple[list[Path], str]:
    if not ray_log_dir.is_dir():
        return [], "missing_ray_log_dir"
    all_driver_logs = sorted(ray_log_dir.glob("python-core-driver-*.log"))
    if pid is not None:
        suffix = f"_{pid}.log"
        return [path for path in all_driver_logs if path.name.endswith(suffix)], "pid"
    if len(all_driver_logs) == 1:
        return all_driver_logs, "single_driver_fallback"
    return [], "ambiguous_without_pid"


def _classify_log_line(line: str) -> list[str]:
    kinds: list[str] = []
    if _RAY_ACTOR_ERROR_RE.search(line):
        kinds.append("ray_actor_error")
    if _POSITIVE_RESTART_RE.search(line):
        kinds.append("positive_actor_restart_count")
    if _UNEXPECTED_WORKER_EXIT_RE.search(line):
        kinds.append("unexpected_worker_exit")

    benign_max_restart_warning = bool(
        _KNOWN_BENIGN_MAX_RESTART_WARNING_RE.search(line)
    )
    if not benign_max_restart_warning:
        if _ENV_RUNNER_FAILURE_RE.search(line):
            kinds.append("env_runner_failure_or_restart")
        if _RESTART_ENV_RUNNER_RE.search(line):
            kinds.append("env_runner_restart_message")
    return list(dict.fromkeys(kinds))


def _scan_driver_logs(paths: Iterable[Path]) -> dict[str, Any]:
    findings: list[dict[str, Any]] = []
    scanned_files: list[dict[str, Any]] = []
    lifecycle_lines = 0
    zero_restart_lines = 0
    total_lines = 0
    read_errors: list[dict[str, str]] = []
    terminal_dead_actor_notifications: list[dict[str, Any]] = []

    for path in paths:
        file_lines = 0
        dead_actor_lines: list[dict[str, Any]] = []
        intended_shutdown_lines: list[int] = []
        try:
            stream = path.open("r", encoding="utf-8", errors="replace")
        except OSError as exc:
            read_errors.append({"path": str(path), "error": str(exc)})
            continue
        with stream:
            for line_number, raw_line in enumerate(stream, 1):
                file_lines += 1
                total_lines += 1
                line = raw_line.rstrip("\r\n")
                if _ENV_RUNNER_LIFECYCLE_RE.search(line):
                    lifecycle_lines += 1
                if _ZERO_RESTART_RE.search(line):
                    zero_restart_lines += 1
                if _DEAD_ACTOR_STATE_RE.search(line):
                    dead_actor_lines.append(
                        {
                            "path": str(path),
                            "line_number": line_number,
                            "line": line,
                        }
                    )
                if _INTENDED_RAY_SHUTDOWN_RE.search(line):
                    intended_shutdown_lines.append(line_number)
                for kind in _classify_log_line(line):
                    if len(findings) < _MAX_FINDINGS:
                        findings.append(
                            {
                                "kind": kind,
                                "path": str(path),
                                "line_number": line_number,
                                "line": line,
                            }
                        )
        # Ray normally reports every EnvRunner actor as DEAD immediately before
        # ``ray.shutdown()``.  Treat only that compact terminal block as benign;
        # an earlier/unclassified DEAD notification remains fail-closed evidence.
        shutdown_line = max(intended_shutdown_lines, default=None)
        for item in dead_actor_lines:
            distance = (
                shutdown_line - item["line_number"]
                if shutdown_line is not None
                else None
            )
            if (
                distance is not None
                and 0 <= distance <= 256
                and _ZERO_RESTART_RE.search(item["line"])
            ):
                terminal_dead_actor_notifications.append(
                    {**item, "lines_before_intended_shutdown": distance}
                )
                continue
            if len(findings) < _MAX_FINDINGS:
                findings.append(
                    {
                        "kind": "dead_actor_outside_terminal_shutdown",
                        **item,
                        "lines_before_intended_shutdown": distance,
                    }
                )
        scanned_files.append({"path": str(path), "lines": file_lines})

    return {
        "scanned_files": scanned_files,
        "total_lines": total_lines,
        "env_runner_lifecycle_lines": lifecycle_lines,
        "zero_restart_lines": zero_restart_lines,
        "read_errors": read_errors,
        "terminal_dead_actor_notifications": terminal_dead_actor_notifications,
        "finding_limit": _MAX_FINDINGS,
        "findings": findings,
        "finding_count": len(findings),
    }


def _atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        temp_path.write_text(
            json.dumps(payload, indent=2, sort_keys=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temp_path, path)
    finally:
        try:
            temp_path.unlink(missing_ok=True)
        except OSError:
            pass


def audit_training_restarts(
    run_dir: str | Path,
    ray_log_dir: str | Path,
    *,
    driver_pid: int | None = None,
) -> dict[str, Any]:
    run_dir = Path(run_dir).expanduser().resolve()
    ray_log_dir = Path(ray_log_dir).expanduser().resolve()
    summary_path = run_dir / "summary.json"
    history_path = run_dir / "train_iterations.jsonl"
    summary = _read_json_dict(summary_path)
    rows, invalid_history_lines = _load_iteration_rows(history_path)
    expected_workers = _expected_env_runners(summary, run_dir)

    metric_observations = list(
        _walk_mapping(summary, source=str(summary_path))
    )
    for row_index, row in enumerate(rows):
        metric_observations.extend(
            _walk_mapping(
                row,
                source=f"{history_path}#row={row_index}",
            )
        )
    restart_metrics = [
        item for item in metric_observations if item["key"] in _RESTART_COUNTER_KEYS
    ]
    healthy_metrics = [
        item for item in metric_observations if item["key"] in _HEALTHY_COUNTER_KEYS
    ]
    malformed_metrics = [
        item for item in metric_observations if item["value"] is None
    ]
    invalid_counter_metrics = [
        item
        for item in metric_observations
        if item["value"] is not None
        and (item["value"] < 0.0 or not float(item["value"]).is_integer())
    ]
    positive_restart_metrics = [
        item
        for item in restart_metrics
        if item["value"] is not None and item["value"] > 0.0
    ]
    unhealthy_metrics = [
        item
        for item in healthy_metrics
        if expected_workers is not None
        and item["value"] is not None
        and item["value"] != float(expected_workers)
    ]

    resolved_pid, pid_source = _resolve_driver_pid(
        run_dir,
        summary,
        driver_pid,
    )
    driver_logs, log_resolution = _driver_logs_for_pid(ray_log_dir, resolved_pid)
    log_audit = _scan_driver_logs(driver_logs)

    skipped_iterations = summary.get("skipped_iterations")
    crash_restarts = summary.get("crash_restarts")
    checks = {
        "run_directory_exists": run_dir.is_dir(),
        "completed_summary_available": bool(summary),
        "training_completed_ok": (
            summary.get("ok") is True
            and summary.get("stop_reason") == "completed"
        ),
        "no_supervisor_restarts": (
            _finite_number(summary.get("restart_count", 0)) == 0.0
            and _finite_number(summary.get("crash_restart_count", 0)) == 0.0
            and _empty_list_or_missing(crash_restarts)
        ),
        "no_skipped_iterations": _empty_list_or_missing(skipped_iterations),
        "iteration_history_available": bool(rows),
        "iteration_history_parseable": not invalid_history_lines,
        "expected_env_runner_count_resolved": expected_workers is not None,
        "driver_pid_resolved": resolved_pid is not None,
        "scoped_ray_driver_log_available": bool(driver_logs),
        "ray_driver_log_readable": not log_audit["read_errors"],
        "ray_driver_has_env_runner_lifecycle_evidence": (
            log_audit["env_runner_lifecycle_lines"] > 0
        ),
        "no_ray_restart_or_failure_evidence": not log_audit["findings"],
        "fault_tolerance_metrics_well_formed_if_present": (
            not malformed_metrics and not invalid_counter_metrics
        ),
        "fault_tolerance_restart_count_zero_if_present": (
            not positive_restart_metrics
        ),
        "healthy_env_runner_count_matches_if_present": not unhealthy_metrics,
    }
    failed_checks = [name for name, passed in checks.items() if not passed]

    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "ok": not failed_checks,
        "status": "PASS" if not failed_checks else "FAIL",
        "run_dir": str(run_dir),
        "summary_path": str(summary_path),
        "history_path": str(history_path),
        "ray_log_dir": str(ray_log_dir),
        "contract": {
            "post_run_only": True,
            "require_completed_summary": True,
            "require_scoped_driver_log": True,
            "require_no_supervisor_restart_or_skip": True,
            "require_no_ray_restart_evidence": True,
            "fault_tolerance_metrics_optional_but_fail_if_nonzero": True,
            "expected_num_env_runners": expected_workers,
        },
        "summary": {
            "available": bool(summary),
            "ok": summary.get("ok"),
            "stop_reason": summary.get("stop_reason"),
            "iterations_completed": summary.get("iterations_completed"),
            "restart_count": summary.get("restart_count", 0),
            "crash_restart_count": summary.get("crash_restart_count", 0),
            "crash_restarts": crash_restarts,
            "skipped_iterations": skipped_iterations,
        },
        "iteration_history": {
            "rows": len(rows),
            "invalid_line_numbers": invalid_history_lines,
        },
        "fault_tolerance_metrics": {
            "attested": bool(restart_metrics or healthy_metrics),
            "observations": metric_observations,
            "positive_restart_observations": positive_restart_metrics,
            "unhealthy_worker_observations": unhealthy_metrics,
            "malformed_observations": malformed_metrics,
            "invalid_counter_observations": invalid_counter_metrics,
        },
        "ray_driver_logs": {
            "driver_pid": resolved_pid,
            "driver_pid_source": pid_source,
            "resolution": log_resolution,
            **log_audit,
        },
        "checks": checks,
        "failed_checks": failed_checks,
    }


def _default_ray_log_dir() -> Path:
    return Path(tempfile.gettempdir()) / "ray" / "session_latest" / "logs"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument(
        "--ray-log-dir",
        default=str(_default_ray_log_dir()),
        help="Ray session logs directory (default: platform temp/ray/session_latest/logs).",
    )
    parser.add_argument(
        "--driver-pid",
        type=int,
        default=None,
        help="Optional worker PID override when watchdog_state.json is unavailable.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help=f"Output JSON (default: <run-dir>/{DEFAULT_OUTPUT_NAME}).",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else run_dir / DEFAULT_OUTPUT_NAME
    )
    try:
        report = audit_training_restarts(
            run_dir,
            args.ray_log_dir,
            driver_pid=args.driver_pid,
        )
    except BaseException as exc:  # fail closed while still emitting an artifact
        report = {
            "schema_version": SCHEMA_VERSION,
            "generated_at": datetime.now()
            .astimezone()
            .isoformat(timespec="seconds"),
            "ok": False,
            "status": "FAIL",
            "run_dir": str(run_dir),
            "ray_log_dir": str(Path(args.ray_log_dir).expanduser()),
            "error": f"{type(exc).__name__}: {exc}",
            "failed_checks": ["audit_completed_without_exception"],
        }
    _atomic_write_json(output, report)
    print(json.dumps(report, indent=2))
    return 0 if report.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
