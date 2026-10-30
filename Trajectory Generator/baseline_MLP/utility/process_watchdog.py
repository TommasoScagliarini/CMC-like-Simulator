"""Cross-platform heartbeat watchdog for simulator and inference processes."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Sequence


class _WindowsProcessJob:
    """Windows Job Object that terminates a supervised process tree."""

    def __init__(self, process: subprocess.Popen) -> None:
        self._handle = None
        if os.name != "nt":
            return

        import ctypes
        from ctypes import wintypes

        class IO_COUNTERS(ctypes.Structure):
            _fields_ = [
                ("ReadOperationCount", ctypes.c_ulonglong),
                ("WriteOperationCount", ctypes.c_ulonglong),
                ("OtherOperationCount", ctypes.c_ulonglong),
                ("ReadTransferCount", ctypes.c_ulonglong),
                ("WriteTransferCount", ctypes.c_ulonglong),
                ("OtherTransferCount", ctypes.c_ulonglong),
            ]

        class JOBOBJECT_BASIC_LIMIT_INFORMATION(ctypes.Structure):
            _fields_ = [
                ("PerProcessUserTimeLimit", ctypes.c_longlong),
                ("PerJobUserTimeLimit", ctypes.c_longlong),
                ("LimitFlags", wintypes.DWORD),
                ("MinimumWorkingSetSize", ctypes.c_size_t),
                ("MaximumWorkingSetSize", ctypes.c_size_t),
                ("ActiveProcessLimit", wintypes.DWORD),
                ("Affinity", ctypes.c_size_t),
                ("PriorityClass", wintypes.DWORD),
                ("SchedulingClass", wintypes.DWORD),
            ]

        class JOBOBJECT_EXTENDED_LIMIT_INFORMATION(ctypes.Structure):
            _fields_ = [
                ("BasicLimitInformation", JOBOBJECT_BASIC_LIMIT_INFORMATION),
                ("IoInfo", IO_COUNTERS),
                ("ProcessMemoryLimit", ctypes.c_size_t),
                ("JobMemoryLimit", ctypes.c_size_t),
                ("PeakProcessMemoryUsed", ctypes.c_size_t),
                ("PeakJobMemoryUsed", ctypes.c_size_t),
            ]

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateJobObjectW.restype = wintypes.HANDLE
        kernel32.SetInformationJobObject.restype = wintypes.BOOL
        kernel32.AssignProcessToJobObject.restype = wintypes.BOOL
        kernel32.TerminateJobObject.restype = wintypes.BOOL
        kernel32.CloseHandle.restype = wintypes.BOOL

        handle = kernel32.CreateJobObjectW(None, None)
        if not handle:
            raise OSError(ctypes.get_last_error(), "CreateJobObjectW failed")

        info = JOBOBJECT_EXTENDED_LIMIT_INFORMATION()
        info.BasicLimitInformation.LimitFlags = 0x00002000  # KILL_ON_JOB_CLOSE
        if not kernel32.SetInformationJobObject(
            handle,
            9,
            ctypes.byref(info),
            ctypes.sizeof(info),
        ):
            error = ctypes.get_last_error()
            kernel32.CloseHandle(handle)
            raise OSError(error, "SetInformationJobObject failed")
        if not kernel32.AssignProcessToJobObject(
            handle,
            wintypes.HANDLE(int(process._handle)),
        ):
            error = ctypes.get_last_error()
            kernel32.CloseHandle(handle)
            raise OSError(error, "AssignProcessToJobObject failed")

        self._kernel32 = kernel32
        self._handle = handle

    def terminate(self, exit_code: int = 124) -> None:
        if self._handle is not None:
            self._kernel32.TerminateJobObject(self._handle, exit_code)

    def close(self) -> None:
        if self._handle is not None:
            self._kernel32.CloseHandle(self._handle)
            self._handle = None


def atomic_write_json(
    path: str | Path,
    payload: dict[str, Any],
    *,
    retries: int = 6,
    delay_s: float = 0.05,
) -> bool:
    """Best-effort atomic JSON write that tolerates the Windows replace race.

    On Windows ``os.replace`` raises ``PermissionError`` (WinError 5) when the
    destination is momentarily held open by another process — exactly the case
    here, where the external supervisor reads the heartbeat while the supervised
    process replaces it. The supervisor's read is brief, so a short retry almost
    always wins. This never raises: a missed advisory heartbeat is harmless (the
    supervisor's stall timeout tolerates the occasional gap), whereas a crash in
    the hot step loop is not. Returns True if the file was updated.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temp_path = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    try:
        temp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except OSError:
        return False
    for attempt in range(max(1, retries)):
        try:
            os.replace(temp_path, target)
            return True
        except PermissionError:
            if attempt < retries - 1:
                time.sleep(delay_s)
                continue
        except OSError:
            break
    try:
        temp_path.unlink(missing_ok=True)
    except OSError:
        pass
    return False


def write_heartbeat(
    path: str | Path,
    phase: str,
    *,
    progress: int | float | None = None,
    timeout_s: float = 0.0,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Atomically publish process progress for an external supervisor."""
    now = time.time()
    payload: dict[str, Any] = {
        "phase": str(phase),
        "progress": progress,
        "timeout_s": float(timeout_s),
        "started_at_unix_s": now,
        "updated_at_unix_s": now,
        "pid": os.getpid(),
    }
    if extra:
        payload.update(extra)
    atomic_write_json(path, payload)
    return payload


def read_heartbeat(path: str | Path) -> dict[str, Any] | None:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return value if isinstance(value, dict) else None


def _terminate_process_tree(
    process: subprocess.Popen,
    windows_job: _WindowsProcessJob | None,
) -> None:
    if windows_job is not None:
        windows_job.terminate()
        try:
            process.wait(timeout=10)
            return
        except subprocess.TimeoutExpired:
            pass
    if process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(
            [
                r"C:\Windows\System32\taskkill.exe",
                "/PID",
                str(process.pid),
                "/T",
                "/F",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=30,
            check=False,
        )
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
    else:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                process.kill()


def supervised_child_env() -> dict[str, str]:
    """Make a directly invoked Windows Conda interpreter resolve its DLLs."""
    env = os.environ.copy()
    if os.name != "nt":
        return env
    prefix = Path(sys.prefix)
    candidates = [
        prefix,
        prefix / "Library" / "mingw-w64" / "bin",
        prefix / "Library" / "usr" / "bin",
        prefix / "Library" / "bin",
        prefix / "Scripts",
        prefix / "bin",
    ]
    existing = env.get("PATH", "").split(os.pathsep)
    if prefix.parent.name.lower() == "envs":
        conda_root = prefix.parents[1]
        root_norm = os.path.normcase(os.path.abspath(conda_root))
        prefix_norm = os.path.normcase(os.path.abspath(prefix))
        existing = [
            item
            for item in existing
            if not (
                os.path.normcase(os.path.abspath(item)).startswith(
                    root_norm + os.sep
                )
                and not os.path.normcase(os.path.abspath(item)).startswith(
                    prefix_norm + os.sep
                )
                and Path(item).name.lower() != "condabin"
            )
        ]
    existing_lower = {item.lower() for item in existing}
    prepend = [
        str(path)
        for path in candidates
        if path.is_dir() and str(path).lower() not in existing_lower
    ]
    env["PATH"] = os.pathsep.join(prepend + existing)
    env["CONDA_PREFIX"] = str(prefix)
    return env


def supervise_process(
    command: Sequence[str],
    *,
    heartbeat_file: str | Path,
    summary_file: str | Path | None = None,
    startup_timeout_s: float = 60.0,
    stall_timeout_s: float = 120.0,
    run_timeout_s: float = 0.0,
    poll_s: float = 0.25,
    label: str = "process",
) -> dict[str, Any]:
    """Run a command and terminate its tree if heartbeat progress stalls."""
    heartbeat_path = Path(heartbeat_file).resolve()
    heartbeat_path.parent.mkdir(parents=True, exist_ok=True)
    heartbeat_path.unlink(missing_ok=True)

    creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
    child_env = supervised_child_env()
    child_env.setdefault("CMC_SIM_HEARTBEAT_FILE", str(heartbeat_path))
    process = subprocess.Popen(
        list(command),
        env=child_env,
        creationflags=creationflags,
        start_new_session=(os.name != "nt"),
    )
    windows_job = None
    if os.name == "nt":
        try:
            windows_job = _WindowsProcessJob(process)
        except OSError as exc:
            print(f"[watchdog] Windows Job Object unavailable: {exc}", flush=True)

    started_at = time.time()
    timeout_reason: str | None = None
    timeout_message: str | None = None
    last_state: dict[str, Any] | None = None
    try:
        while process.poll() is None:
            now = time.time()
            state = read_heartbeat(heartbeat_path)
            if state is not None:
                last_state = state

            if run_timeout_s > 0.0 and now - started_at > run_timeout_s:
                timeout_reason = "run_timeout"
                timeout_message = f"{label} exceeded {run_timeout_s:g} s total."
                break
            if last_state is None:
                if startup_timeout_s > 0.0 and now - started_at > startup_timeout_s:
                    timeout_reason = "startup_timeout"
                    timeout_message = (
                        f"{label} did not publish a heartbeat within "
                        f"{startup_timeout_s:g} s."
                    )
                    break
            else:
                phase = str(last_state.get("phase", "unknown"))
                updated_at = float(
                    last_state.get(
                        "updated_at_unix_s",
                        last_state.get("started_at_unix_s", now),
                    )
                )
                if stall_timeout_s > 0.0 and now - updated_at > stall_timeout_s:
                    timeout_reason = "stall_timeout"
                    timeout_message = (
                        f"{label} heartbeat stalled in {phase!r} for more than "
                        f"{stall_timeout_s:g} s."
                    )
                    break
                phase_timeout_s = float(last_state.get("timeout_s", 0.0) or 0.0)
                phase_started_at = float(
                    last_state.get("started_at_unix_s", updated_at)
                )
                if (
                    phase_timeout_s > 0.0
                    and now - phase_started_at > phase_timeout_s
                ):
                    timeout_reason = "phase_timeout"
                    timeout_message = (
                        f"{label} phase {phase!r} exceeded "
                        f"{phase_timeout_s:g} s."
                    )
                    break
            time.sleep(max(0.05, poll_s))
    except KeyboardInterrupt:
        timeout_reason = "user_interrupt"
        timeout_message = f"{label} interrupted by user."

    if timeout_reason is not None:
        print(json.dumps({"watchdog_timeout": timeout_message}), flush=True)
        _terminate_process_tree(process, windows_job)

    returncode = process.poll()
    if returncode is None:
        returncode = process.wait()
    final_state = read_heartbeat(heartbeat_path)
    if final_state is not None:
        last_state = final_state
    if windows_job is not None:
        windows_job.close()

    summary = {
        "ok": timeout_reason is None and returncode == 0,
        "label": label,
        "command": list(command),
        "returncode": int(returncode),
        "timeout_reason": timeout_reason,
        "timeout_message": timeout_message,
        "elapsed_wall_time_s": time.time() - started_at,
        "heartbeat_file": str(heartbeat_path),
        "last_heartbeat": last_state,
    }
    if summary_file is not None:
        summary_path = Path(summary_file)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def run_self_test(output_dir: str | Path, stall_timeout_s: float = 1.5) -> dict:
    """Prove that a deliberately stalled child is detected and terminated."""
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    heartbeat = out / "watchdog_state.json"
    child_summary = out / "watchdog_child_summary.json"
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--self-test-worker",
        "--heartbeat-file",
        str(heartbeat),
        "--self-test-worker-sleep-s",
        str(max(5.0, stall_timeout_s * 4.0)),
    ]
    child = supervise_process(
        command,
        heartbeat_file=heartbeat,
        summary_file=child_summary,
        startup_timeout_s=5.0,
        stall_timeout_s=stall_timeout_s,
        run_timeout_s=max(10.0, stall_timeout_s * 6.0),
        label="watchdog self-test",
    )
    result = {
        "ok": child.get("timeout_reason") == "stall_timeout",
        "expected_timeout_reason": "stall_timeout",
        "child": child,
    }
    (out / "watchdog_self_test.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--heartbeat-file")
    parser.add_argument("--summary-file")
    parser.add_argument("--startup-timeout-s", type=float, default=60.0)
    parser.add_argument("--stall-timeout-s", type=float, default=120.0)
    parser.add_argument("--run-timeout-s", type=float, default=0.0)
    parser.add_argument("--poll-s", type=float, default=0.25)
    parser.add_argument("--label", default="process")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--output-dir", default="results/watchdog_self_test")
    parser.add_argument("--self-test-worker", action="store_true")
    parser.add_argument("--self-test-worker-sleep-s", type=float, default=10.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test_worker:
        if not args.heartbeat_file:
            raise ValueError("--self-test-worker requires --heartbeat-file")
        write_heartbeat(args.heartbeat_file, "intentional_stall", progress=1)
        time.sleep(args.self_test_worker_sleep_s)
        return 0
    if args.self_test:
        result = run_self_test(args.output_dir, args.stall_timeout_s)
        print(json.dumps(result, indent=2))
        return 0 if result["ok"] else 1
    if not args.heartbeat_file or not args.command:
        raise ValueError("Supervision requires --heartbeat-file and a command")
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    result = supervise_process(
        command,
        heartbeat_file=args.heartbeat_file,
        summary_file=args.summary_file,
        startup_timeout_s=args.startup_timeout_s,
        stall_timeout_s=args.stall_timeout_s,
        run_timeout_s=args.run_timeout_s,
        poll_s=args.poll_s,
        label=args.label,
    )
    print(json.dumps(result, indent=2))
    if result["timeout_reason"] is not None:
        return 124 if result["timeout_reason"] != "user_interrupt" else 130
    return int(result["returncode"])


if __name__ == "__main__":
    raise SystemExit(main())
