"""PPO (MLP) training entrypoint for the CMC-like prosthetic trajectory env.

Single-agent Ray RLlib PPO on the **new API stack** (RLModule + Learner). The
policy learns to emit prosthetic trajectory segments; the biological side stays
muscle-driven via Static Optimization inside the unchanged simulator.

Staging:
  * Phase A (correctness): ``--num-env-runners 0`` runs everything in this
    process (no Ray workers, no DLL/CWD surprises).
  * Phase B (parallelism): ``--num-env-runners N`` spawns EnvRunner workers; the
    Windows torch/OpenSim shim is applied in each worker via a setup hook.

Run from the repository root, e.g.:
  python "Trajectory Generator\\baseline_MLP\\train_ppo_mlp.py" --setup-xml ...
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import argparse  # noqa: E402
import faulthandler  # noqa: E402
import json  # noqa: E402
import logging  # noqa: E402
import math  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
import warnings  # noqa: E402
from contextlib import contextmanager  # noqa: E402
from datetime import datetime  # noqa: E402
from typing import Any  # noqa: E402

import progress_display  # noqa: E402
import training_config  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
_DEFAULT_GRF_MODE = "online_sensor"
_DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)
_BASELINE_DIR = str(Path(__file__).resolve().parent)
_SCRIPT_PATH = Path(__file__).resolve()
_TRAJ_GEN_DIR = _SCRIPT_PATH.parents[1]   # .../Trajectory Generator
_RUNS_ROOT = _TRAJ_GEN_DIR / "runs"
_TRAINING_RUNS_ROOT = _RUNS_ROOT / "training"
_WATCHDOG_FILENAME = "watchdog_state.json"
_SUPERVISOR_STATE_FILENAME = "supervisor_state.json"


def _strip_runs_prefix(path: Path, category: str) -> Path:
    parts = path.parts
    if parts and parts[0].lower() == "runs":
        parts = parts[1:]
    if parts and parts[0].lower() == category.lower():
        parts = parts[1:]
    return Path(*parts) if parts else Path()


def _under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _resolve_category_output_dir(output_dir, default_stem, category_root, category):
    if output_dir:
        path = Path(output_dir)
        if path.is_absolute():
            resolved = path.resolve()
            if _under(resolved, _RUNS_ROOT) and not _under(resolved, category_root):
                rel = _strip_runs_prefix(resolved.relative_to(_RUNS_ROOT), category)
                return (category_root / rel).resolve()
            return resolved
        rel = _strip_runs_prefix(path, category)
        path = category_root / rel
    else:
        path = category_root / f"{default_stem}_{datetime.now():%Y%m%d_%H%M%S}"
    return path.resolve()


def _strategy_name_from_reward_mode(reward_mode: str | None) -> str:
    if str(reward_mode or "").lower() == "imitation":
        return "imitation"
    return "ExNovo"


def _sanitize_name_suffix(value: str | None) -> str:
    if value is None:
        return ""
    suffix = str(value).strip()
    if not suffix:
        return ""
    invalid = '<>:"/\\|?*'
    suffix = "".join(
        "_" if char in invalid or ord(char) < 32 else char for char in suffix
    ).strip(" .")
    if not suffix:
        return ""
    return suffix if suffix.startswith("_") else f"_{suffix}"


def _unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    for index in range(2, 1000):
        candidate = Path(f"{path}_{index:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not find a free output directory name for {path}")


def _load_reward_json_for_run_name(spec: str | None) -> dict[str, Any]:
    if not spec:
        return {}
    path = Path(spec)
    text = path.read_text(encoding="utf-8") if path.exists() else spec
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("reward override must be a JSON object of RewardConfig fields")
    return data


def _resolved_reward_mode_for_run_name(
    args: argparse.Namespace,
    cfg_reward: dict[str, Any],
) -> str:
    reward = dict(cfg_reward or {})
    reward.update(_load_reward_json_for_run_name(args.reward_json))
    if args.reward_mode is not None:
        reward["reward_mode"] = args.reward_mode
    return str(reward.get("reward_mode", "ex_novo"))


def _default_training_output_dir(
    reward_mode: str | None,
    name_suffix: str | None = None,
    *,
    now: datetime | None = None,
) -> Path:
    current = now or datetime.now()
    strategy = _strategy_name_from_reward_mode(reward_mode)
    suffix = _sanitize_name_suffix(name_suffix)
    folder = f"MLP_{strategy}_training_{current:%m-%d-%Y}{suffix}"
    return _unique_path((_TRAINING_RUNS_ROOT / folder).resolve())


def _resolve_output_dir(output_dir, default_stem):
    """Resolve ``--output-dir`` so artifacts always land under
    ``Trajectory Generator/runs/training`` regardless of the current working
    directory.

    A relative path is taken relative to the training runs directory. Historical
    ``runs\\foo`` arguments are treated as ``runs\\training\\foo``; absolute paths
    outside ``Trajectory Generator/runs`` are honored as-is. The returned path is
    absolute (pyarrow's ``from_uri`` rejects relative paths) and CWD-independent
    so the supervisor and the worker child always agree on the same directory.
    """
    return _resolve_category_output_dir(
        output_dir, default_stem, _TRAINING_RUNS_ROOT, "training"
    )
_TRAINING_STACK_LOADED = False


def _load_training_stack() -> None:
    """Import Torch/Ray/OpenSim only inside the supervised worker process."""
    global _TRAINING_STACK_LOADED
    global DefaultModelConfig, PPOConfig, env_factory, ray, reward_function, tb_logging

    if _TRAINING_STACK_LOADED:
        return

    # The Windows shim must import torch before either Ray or OpenSim.
    import win_runtime  # noqa: F401
    import _bootstrap

    _bootstrap.ensure_sim_paths()

    import ray as _ray
    from ray.rllib.algorithms.ppo import PPOConfig as _PPOConfig
    from ray.rllib.core.rl_module.default_model_config import (
        DefaultModelConfig as _DefaultModelConfig,
    )

    import env_factory as _env_factory
    import reward_function as _reward_function
    import tb_logging as _tb_logging

    ray = _ray
    PPOConfig = _PPOConfig
    DefaultModelConfig = _DefaultModelConfig
    env_factory = _env_factory
    reward_function = _reward_function
    tb_logging = _tb_logging
    _TRAINING_STACK_LOADED = True


class _WindowsProcessJob:
    """Windows Job Object that kills the supervised process tree on request."""

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
            9,  # JobObjectExtendedLimitInformation
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


def _write_watchdog_state(output_dir: Path, phase: str, timeout_s: float) -> None:
    """Publish the current blocking phase for the external process watchdog.

    Best-effort: ``os.replace`` can raise ``PermissionError`` (WinError 5) when
    the supervisor reads the file at the instant we replace it. The supervisor's
    read is brief, so a short retry almost always wins; a missed advisory update
    is harmless, a crash here is not.
    """
    path = output_dir / _WATCHDOG_FILENAME
    now = time.time()
    payload = {
        "phase": phase,
        "timeout_s": float(timeout_s),
        "started_at_unix_s": now,
        "updated_at_unix_s": now,
        "pid": os.getpid(),
    }
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        temp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except OSError:
        return
    for attempt in range(6):
        try:
            os.replace(temp_path, path)
            return
        except PermissionError:
            if attempt < 5:
                time.sleep(0.05)
                continue
        except OSError:
            break
    try:
        temp_path.unlink(missing_ok=True)
    except OSError:
        pass


def _fmt_metric(value: Any) -> str:
    """Compact human string for an optional numeric metric."""
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return f"{float(value):.4g}"
    return "n/a"


class _TrainingMonitor:
    """In-process phase self-timeout + live progress for the training child.

    One daemon thread drives two responsibilities:

    * **Self-timeout (the hard fix for a stalled iteration).** Each phase carries
      a wall-clock budget. If a phase (a training iteration, sampling, a
      checkpoint export, startup, ...) overruns it, the monitor dumps every
      thread stack, writes a timeout ``summary.json`` and hard-exits the process.
      Because the check runs in a *separate Python thread*, it fires even when the
      main thread is blocked in a C-level ``ray.wait`` (which releases the GIL),
      so the iteration timeout is enforced without relying on the external
      file-watching supervisor.
    * **Live progress.** Renders an in-place bar with overall percent, iteration
      counter, elapsed and ETA, so the terminal is never silent during the long
      ``algo.train()`` calls.

    The watchdog state file is still written on every phase change, so the
    external supervisor keeps working as a redundant outer layer.
    """

    def __init__(
        self,
        output_dir: Path,
        total_iterations: int,
        *,
        initial_iteration: int = 0,
        resume_from: str | None = None,
        self_timeout: bool = True,
        progress: bool = True,
    ) -> None:
        self._output_dir = Path(output_dir)
        self._total = int(total_iterations)
        self._self_timeout = bool(self_timeout)
        self._lock = threading.Lock()
        self._phase = "starting"
        self._deadline: float | None = None
        self._timeout_s = 0.0
        self._iterations_completed = int(initial_iteration)
        self._iterations_completed_this_process = 0
        self._current_iteration: int | None = None
        self._resume_from = resume_from
        self._fired = False
        self._stop = threading.Event()
        self._progress = (
            progress_display.LiveProgress(total=self._total, label="train")
            if progress
            else None
        )
        self._thread = threading.Thread(
            target=self._loop, name="train-monitor", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def set_phase(self, phase: str, timeout_s: float) -> None:
        timeout_s = float(timeout_s)
        with self._lock:
            self._phase = phase
            self._timeout_s = timeout_s
            self._deadline = (
                time.monotonic() + timeout_s if timeout_s > 0.0 else None
            )
        _write_watchdog_state(self._output_dir, phase, timeout_s)
        if self._progress is not None:
            self._progress.update(phase=phase, phase_reset=True)

    def set_iteration(self, iteration: int) -> None:
        with self._lock:
            self._current_iteration = int(iteration)

    def set_initial_iteration(self, iteration: int) -> None:
        with self._lock:
            self._iterations_completed = int(iteration)
        if self._progress is not None:
            self._progress.update(completed=int(iteration), eta_reset=True)

    def on_iteration_complete(
        self,
        iteration: int,
        ret: Any,
        length: Any,
        steps: Any,
        iter_seconds: float,
    ) -> None:
        with self._lock:
            self._iterations_completed = int(iteration)
            self._iterations_completed_this_process += 1
        if self._progress is not None:
            self._progress.update(
                completed=int(iteration), suffix=f"ret {_fmt_metric(ret)}"
            )
            self._progress.log(
                f"[iter {iteration}/{self._total}] return={_fmt_metric(ret)} "
                f"len={_fmt_metric(length)} steps={_fmt_metric(steps)} "
                f"time={progress_display.format_hms(iter_seconds)}"
            )

    def log_event(self, message: str) -> None:
        """Emit a permanent line above the live bar (e.g. an iteration skip)."""
        if self._progress is not None:
            self._progress.log(message)
        else:
            print(message, flush=True)

    def _loop(self) -> None:
        while not self._stop.wait(0.5):
            with self._lock:
                deadline = self._deadline
                phase = self._phase
                timeout_s = self._timeout_s
                fired = self._fired
            if (
                self._self_timeout
                and not fired
                and deadline is not None
                and time.monotonic() > deadline
            ):
                self._fire(phase, timeout_s)
                return
            if self._progress is not None:
                self._progress.render()

    def _fire(self, phase: str, timeout_s: float) -> None:
        with self._lock:
            self._fired = True
            current_iteration = self._current_iteration
            iterations_completed = self._iterations_completed
            completed_this_process = self._iterations_completed_this_process
        iteration_timeout = phase.startswith("algo.train iteration ")
        stop_reason = "iteration_timeout" if iteration_timeout else "phase_timeout"
        message = (
            f"{phase} exceeded the {timeout_s:g} s wall-clock timeout "
            "(child self-guard)."
        )
        try:
            dump_path = self._output_dir / "faulthandler.log"
            with dump_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    f"\n=== {datetime.now().isoformat()} CHILD SELF-TIMEOUT "
                    f"phase={phase!r} after {timeout_s:g}s ===\n"
                )
                faulthandler.dump_traceback(file=stream, all_threads=True)
        except Exception:
            pass
        try:
            summary_path = self._output_dir / "summary.json"
            existing: dict = {}
            try:
                existing = json.loads(summary_path.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                pass
            existing.update(
                {
                    "ok": False,
                    "stop_reason": stop_reason,
                    "stop_message": f"Training STOPPED - {message}",
                    "timed_out": True,
                    "interrupted": False,
                    "timed_out_phase": phase,
                    "timed_out_iteration": (
                        current_iteration if iteration_timeout else None
                    ),
                    "next_iteration": (
                        current_iteration + 1
                        if iteration_timeout and current_iteration is not None
                        else None
                    ),
                    "error": message,
                    "iterations_completed": iterations_completed,
                    "iterations_completed_this_process": completed_this_process,
                    "resume_from": self._resume_from,
                    "output_dir": str(self._output_dir),
                    "checkpoint_last": str(self._output_dir / "checkpoint_last"),
                    "checkpoint_best": str(self._output_dir / "checkpoint_best"),
                    "rl_module_last": str(self._output_dir / "rl_module_last"),
                    "rl_module_best": str(self._output_dir / "rl_module_best"),
                }
            )
            summary_path.write_text(
                json.dumps(existing, indent=2), encoding="utf-8"
            )
        except Exception:
            pass
        try:
            print(
                json.dumps(
                    {
                        "training_timeout": message,
                        "stop_reason": stop_reason,
                        "timed_out_iteration": current_iteration,
                    }
                ),
                flush=True,
            )
        except Exception:
            pass
        os._exit(124)

    def stop(self, final_message: str | None = None) -> None:
        self._stop.set()
        if self._progress is not None:
            self._progress.finish(final_message)


@contextmanager
def _fault_dump_guard(output_dir: Path, phase: str, timeout_s: float):
    """Dump all Python thread stacks before a blocking phase times out."""
    if timeout_s <= 0.0:
        yield
        return

    dump_path = output_dir / "faulthandler.log"
    delay_s = max(5.0, min(30.0, timeout_s * 0.5))
    with dump_path.open("a", encoding="utf-8", buffering=1) as stream:
        stream.write(
            f"\n=== {datetime.now().isoformat()} phase={phase!r} "
            f"pre-timeout stack dump after {delay_s:g}s ===\n"
        )
        faulthandler.enable(file=stream, all_threads=True)
        faulthandler.dump_traceback_later(delay_s, repeat=True, file=stream)
        try:
            yield
        finally:
            faulthandler.cancel_dump_traceback_later()
            faulthandler.disable()


def _worker_setup(baseline_dir: str = _BASELINE_DIR) -> None:
    """Ray worker process setup hook: apply path bootstrap + Windows shim.

    Picklable-by-value (defined in __main__ when run as a script); the absolute
    baseline dir is captured as a default argument so workers need no CWD.
    """
    import sys as _sys
    import warnings as _warnings

    # Keep the worker quiet (Static Optimization QP fallback RuntimeWarning,
    # deprecation/Future warnings) so it never reaches the driver console; the
    # driver disables log forwarding too unless --verbose-workers is set.
    _warnings.filterwarnings("ignore")

    if baseline_dir not in _sys.path:
        _sys.path.insert(0, baseline_dir)
    import _bootstrap as _bs

    _bs.ensure_sim_paths()
    import win_runtime as _wr  # noqa: F401


def _get_metric(result: dict, key: str):
    """Read an env-runner metric across RLlib result layouts."""
    er = result.get("env_runners")
    if isinstance(er, dict) and key in er:
        return er[key]
    return result.get(key)


def _finite(value) -> bool:
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _find_numeric_metric(node: Any, suffix: str) -> float | None:
    """Find the first finite scalar whose slash-path ends with ``suffix``."""
    if not isinstance(node, dict):
        return None
    for key, value in node.items():
        if str(key).endswith(suffix) and _finite(value):
            return float(value)
        found = _find_numeric_metric(value, suffix)
        if found is not None:
            return found
    return None


def _collect_numeric_metrics(node: Any, prefix: str = "") -> dict[str, float]:
    """Flatten finite numeric values from an RLlib result subtree."""
    if not isinstance(node, dict):
        return {}
    metrics: dict[str, float] = {}
    for key, value in node.items():
        path = f"{prefix}/{key}" if prefix else str(key)
        if isinstance(value, dict):
            metrics.update(_collect_numeric_metrics(value, path))
        elif _finite(value):
            metrics[path] = float(value)
    return metrics


def _find_flat_metric(metrics: dict[str, float], *suffixes: str) -> float | None:
    for suffix in suffixes:
        for key, value in metrics.items():
            if key.endswith(suffix):
                return value
    return None


def build_config(
    args: argparse.Namespace, reward_overrides: dict | None = None
) -> PPOConfig:
    env_factory.register_cmc_env()

    env_config = {
        "setup_xml_path": args.setup_xml,
        "segment_duration": args.segment_duration,
        "episode_duration": args.episode_duration,
        "policy_knots": args.policy_knots,
        "action_mode": args.action_mode,
        "max_delta_rad": args.max_delta_rad,
        "enable_pros_ref_governor": args.pros_ref_governor,
        "pros_ref_model": args.pros_ref_model,
        "pros_ref_lpf_cutoff_hz": args.pros_ref_cutoff_hz,
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": args.pros_knee_ref_velocity_limit_rad_s,
            "pros_ankle_angle": args.pros_ankle_ref_velocity_limit_rad_s,
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": args.pros_knee_ref_acceleration_limit_rad_s2,
            "pros_ankle_angle": args.pros_ankle_ref_acceleration_limit_rad_s2,
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": args.pros_knee_ref_jerk_limit_rad_s3,
            "pros_ankle_angle": args.pros_ankle_ref_jerk_limit_rad_s3,
        },
        "actor_cyclic_phase_only": args.actor_cyclic_phase_only,
        "include_reference_state_observation": args.include_reference_state_observation,
        "imitation_initialize_to_target": args.imitation_initialize_to_target,
        "reward_reference_range_floor": args.reward_reference_range_floor,
        "reward_reference_velocity_range_floor": (
            args.reward_reference_velocity_range_floor
        ),
        "random_init": args.random_init,
        "episode_start_offset_s": args.episode_start_offset_s,
        "rebuild_model_on_reset": False,
        "record_outputs": False,
        "fail_fast": True,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "include_online_grf_observation": args.online_grf_observation,
        "critic_privileged_observation": args.asymmetric_actor_critic,
        "prescribed_grf_disabled_sides": args.disable_prescribed_grf_side,
        "online_grf_applied_sides": args.online_grf_applied_side,
        "step_wall_timeout_s": args.step_wall_timeout_s,
    }
    # The reward seen by the agent is shaped by reward_function.py (single source
    # of truth); these overrides reach RewardShapingWrapper via make_cmc_env.
    if reward_overrides:
        env_config["reward"] = reward_overrides

    training_kwargs = {
        "train_batch_size": args.train_batch_size,
        "minibatch_size": args.minibatch_size,
        "num_epochs": args.num_epochs,
        "lr": args.lr,
        "gamma": args.gamma,
        "lambda_": args.lam,
        "clip_param": args.clip_param,
    }
    if args.vf_clip_param is not None:
        training_kwargs["vf_clip_param"] = args.vf_clip_param
    if args.vf_loss_coeff is not None:
        training_kwargs["vf_loss_coeff"] = args.vf_loss_coeff

    config = (
        PPOConfig()
        .environment(env_factory.ENV_NAME, env_config=env_config)
        .framework("torch")
        .callbacks(tb_logging.RewardComponentsCallback)
        .env_runners(
            num_env_runners=args.num_env_runners,
            num_envs_per_env_runner=1,
            rollout_fragment_length="auto",
            sample_timeout_s=args.sample_timeout_s,
        )
        .fault_tolerance(
            # Keep transient single-worker failures recoverable. A full
            # algo.train() stall is handled one level above by restarting the
            # entire child process from checkpoint_last.
            restart_failed_env_runners=True,
            ignore_env_runner_failures=True,
        )
        .learners(num_learners=0)  # learner in the main process (CPU)
        .resources(num_cpus_for_main_process=1, num_gpus=0)
        .training(**training_kwargs)
        .reporting(
            min_sample_timesteps_per_iteration=args.train_batch_size,
            metrics_num_episodes_for_smoothing=min(
                100,
                max(1, args.train_batch_size),
            ),
        )
        .debugging(seed=args.seed)
    )

    # Uniform-width MLP: fcnet_hiddens = [dim] * num (see training_cfg.yaml model:).
    # Backward-compat: an explicit (deprecated) --fcnet-hiddens list wins.
    if getattr(args, "fcnet_hiddens", None):
        hiddens = [int(x) for x in args.fcnet_hiddens]
    else:
        hiddens = [int(args.dim_hidden_layers)] * int(args.num_hidden_layers)
    if args.asymmetric_actor_critic:
        # Asymmetric actor-critic: the env emits the FULL [actor | privileged]
        # observation; the custom RLModule routes obs[:n_actor] to the policy and
        # the full vector to the value head. n_actor/n_full come from a one-off
        # probe env (single source of truth for the feature layout).
        from ray.rllib.core.rl_module.rl_module import RLModuleSpec
        from asymmetric_rl_module import AsymmetricActorCriticTorchRLModule
        from osim_trj_cmc_like import CMCLikeProsthesisTrajectoryEnv

        probe = CMCLikeProsthesisTrajectoryEnv(
            env_factory.build_env_config(env_config)
        )
        try:
            n_actor = int(probe.n_actor)
            n_full = int(probe.n_obs)
        finally:
            probe.close()
        config = config.rl_module(
            rl_module_spec=RLModuleSpec(
                module_class=AsymmetricActorCriticTorchRLModule,
                model_config={
                    "n_actor": n_actor,
                    "n_full": n_full,
                    "fcnet_hiddens": hiddens,
                    "fcnet_activation": args.fcnet_activation,
                },
            )
        )
    else:
        config = config.rl_module(
            model_config=DefaultModelConfig(
                fcnet_hiddens=hiddens,
                fcnet_activation=args.fcnet_activation,
            )
        )
    return config


def _dump_all_stacks(output_dir: Path, header: str) -> None:
    """Append a single all-threads stack dump to faulthandler.log (best-effort)."""
    try:
        with (output_dir / "faulthandler.log").open("a", encoding="utf-8") as stream:
            stream.write(f"\n=== {datetime.now().isoformat()} {header} ===\n")
            faulthandler.dump_traceback(file=stream, all_threads=True)
    except Exception:
        pass


def _load_iteration_history(path: Path) -> list[dict[str, Any]]:
    """Load the latest valid row for each logical iteration from JSONL."""
    latest: dict[int, dict[str, Any]] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return []
    for line in lines:
        try:
            row = json.loads(line)
            iteration = int(row["iteration"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            continue
        if isinstance(row, dict):
            latest[iteration] = row
    return [latest[key] for key in sorted(latest)]


def _merge_iteration_history(
    history: list[dict[str, Any]],
    row: dict[str, Any],
) -> list[dict[str, Any]]:
    """Upsert one logical iteration and return history sorted without duplicates."""
    latest: dict[int, dict[str, Any]] = {}
    for candidate in [*history, row]:
        try:
            iteration = int(candidate["iteration"])
        except (KeyError, TypeError, ValueError):
            continue
        latest[iteration] = candidate
    return [latest[key] for key in sorted(latest)]


def _write_iteration_history(path: Path, history: list[dict[str, Any]]) -> None:
    """Atomically write canonical one-row-per-logical-iteration JSONL history."""
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temp_path.write_text(
        "".join(json.dumps(row) + "\n" for row in history),
        encoding="utf-8",
    )
    os.replace(temp_path, path)


def _resolve_resume_path(value: str | None) -> Path | None:
    """Resolve ``--resume-from`` accepting both path conventions.

    Relative paths are tried against the CWD first (``Trajectory
    Generator\\runs\\...`` from the simulator root), then against the Trajectory
    Generator directory, then against ``Trajectory Generator\\runs\\training``.
    Thus the historical ``runs\\foo\\checkpoint_last`` form still resolves after
    training artifacts moved under ``runs\\training``. Missing paths fall back to
    the CWD candidate so the child's existing not-found error stays meaningful.
    """
    if not value:
        return None
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    traj_candidate = (_TRAJ_GEN_DIR / path).resolve()
    if traj_candidate.exists():
        return traj_candidate
    training_candidate = (
        _TRAINING_RUNS_ROOT / _strip_runs_prefix(path, "training")
    ).resolve()
    if training_candidate.exists():
        return training_candidate
    return cwd_candidate


def _read_json_dict(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}
    return value if isinstance(value, dict) else {}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def run(args: argparse.Namespace) -> dict:
    output_dir = _resolve_output_dir(args.output_dir, "baseline_mlp")
    output_dir.mkdir(parents=True, exist_ok=True)

    ray_num_cpus = args.ray_num_cpus
    if ray_num_cpus is None:
        ray_num_cpus = max(1, args.num_env_runners + 1)

    resume_path = _resolve_resume_path(args.resume_from)
    reward_overrides: dict = {}
    algo = None
    tb_writer = None
    history = _load_iteration_history(output_dir / "train_iterations.jsonl")
    best_return = max(
        (
            float(row["episode_return_mean"])
            for row in history
            if _finite(row.get("episode_return_mean"))
        ),
        default=-float("inf"),
    )
    interrupted = False
    error: str | None = None
    stop_reason = "completed"
    stop_message: str | None = None
    restored_training_iteration = 0
    restored_logical_iteration = 0
    iteration_start = max(1, int(args.iteration_start or 0))
    iterations_completed_this_process = 0
    start_time = time.perf_counter()

    monitor = _TrainingMonitor(
        output_dir,
        args.iterations,
        initial_iteration=iteration_start - 1,
        resume_from=str(resume_path) if resume_path is not None else None,
        self_timeout=args.child_self_timeout,
        progress=args.progress,
    )
    monitor.start()

    try:
        monitor.set_phase("training stack imports", args.startup_timeout_s)
        with _fault_dump_guard(
            output_dir, "training stack imports", args.startup_timeout_s
        ):
            _load_training_stack()

        # Reward precedence (weak -> strong): training_cfg.yaml `reward:` section,
        # then --reward-json, then --reward-mode. None of --reward-mode leaves the
        # json/yaml setting (default "ex_novo"). All shaped by reward_function.py.
        reward_overrides = dict(getattr(args, "_cfg_reward", None) or {})
        json_overrides = reward_function.load_reward_overrides(args.reward_json)
        if json_overrides:
            reward_overrides.update(json_overrides)
        if args.reward_mode is not None:
            reward_overrides["reward_mode"] = args.reward_mode
        # Snapshot the fully-resolved config (YAML + CLI overrides) so rollout_eval
        # can reconstruct the exact env/model/reward from the checkpoint's run dir.
        try:
            training_config.dump_resolved(
                args,
                reward_overrides,
                output_dir / training_config.RESOLVED_CONFIG_NAME,
            )
        except OSError as exc:
            training_config._warn(f"could not write resolved config snapshot: {exc}")
        # Clean console by default: do NOT forward each worker's stdout/stderr to
        # the driver (that per-worker spam — Static Optimization QP fallback,
        # deprecation/Future warnings, muscle init — is what interrupts the live
        # progress bar and makes it reprint many times) and silence Python
        # warnings. Worker errors still surface as driver exceptions and Ray's own
        # log files. Use --verbose-workers to restore full worker logging.
        quiet = not args.verbose_workers
        env_vars = {"KMP_DUPLICATE_LIB_OK": "TRUE"}
        if quiet:
            env_vars["PYTHONWARNINGS"] = "ignore"
            warnings.filterwarnings("ignore")
        runtime_env = {"env_vars": env_vars}
        if args.num_env_runners > 0:
            runtime_env["worker_process_setup_hook"] = _worker_setup

        monitor.set_phase("ray.init", args.startup_timeout_s)
        with _fault_dump_guard(output_dir, "ray.init", args.startup_timeout_s):
            ray.init(
                include_dashboard=False,
                ignore_reinit_error=True,
                runtime_env=runtime_env,
                log_to_driver=args.verbose_workers,
                logging_level=logging.ERROR if quiet else logging.INFO,
                num_cpus=ray_num_cpus,
                num_gpus=0,
            )

        # Standalone Algorithm.build_algo() otherwise writes under ~/ray_results.
        # Keep every RLlib artifact within this run's explicitly writable root.
        from ray.tune.trainable import trainable as ray_trainable

        rllib_log_root = output_dir / "rllib"
        rllib_log_root.mkdir(parents=True, exist_ok=True)
        ray_trainable.DEFAULT_STORAGE_PATH = str(rllib_log_root)

        config = build_config(args, reward_overrides)
        monitor.set_phase("PPOConfig.build_algo", args.startup_timeout_s)
        with _fault_dump_guard(
            output_dir, "PPOConfig.build_algo", args.startup_timeout_s
        ):
            algo = config.build_algo()

        if resume_path is not None:
            if not resume_path.is_dir():
                raise FileNotFoundError(
                    f"Resume checkpoint directory not found: {resume_path}"
                )
            monitor.set_phase(
                f"restore checkpoint {resume_path.name}",
                args.startup_timeout_s,
            )
            with _fault_dump_guard(
                output_dir,
                f"restore checkpoint {resume_path.name}",
                args.startup_timeout_s,
            ):
                algo.restore_from_path(resume_path)
            restored_training_iteration = int(getattr(algo, "iteration", 0) or 0)
            checkpoint_meta = _read_json_dict(
                resume_path.parent / f"{resume_path.name}_meta.json"
            )
            restored_logical_iteration = int(
                checkpoint_meta.get("logical_iteration")
                or restored_training_iteration
            )
            if args.iteration_start <= 0:
                iteration_start = restored_logical_iteration + 1
            monitor.set_initial_iteration(iteration_start - 1)
            monitor.log_event(
                f"Resumed checkpoint {resume_path} at RLlib iteration "
                f"{restored_training_iteration}, logical iteration "
                f"{restored_logical_iteration}; next logical iteration "
                f"{iteration_start}/{args.iterations}."
            )

        if args.tensorboard:
            tb_writer = tb_logging.make_tb_writer(output_dir / "tensorboard")

        for iteration in range(iteration_start, args.iterations + 1):
            # algo.train() is intentionally blocking in the main thread. If its
            # monotonic deadline expires, the monitor writes an
            # iteration_timeout summary and hard-exits this child. The supervisor
            # then starts a fresh process from checkpoint_last.
            monitor.set_iteration(iteration)
            monitor.set_phase(
                f"algo.train iteration {iteration}",
                args.iteration_timeout_s,
            )
            iter_t0 = time.perf_counter()
            with _fault_dump_guard(
                output_dir,
                f"algo.train iteration {iteration}",
                args.iteration_timeout_s,
            ):
                result = algo.train()
            iter_seconds = time.perf_counter() - iter_t0
            ret = _get_metric(result, "episode_return_mean")
            length = _get_metric(result, "episode_len_mean")
            learner_metrics = _collect_numeric_metrics(
                result.get("learners"), "learners"
            )
            env_metrics = _collect_numeric_metrics(
                result.get("env_runners"), "env_runners"
            )
            row = {
                "iteration": iteration,
                "episode_return_mean": float(ret) if _finite(ret) else None,
                "episode_len_mean": float(length) if _finite(length) else None,
                "num_env_steps_sampled_lifetime": _get_metric(
                    result, "num_env_steps_sampled_lifetime"
                ),
                "policy_loss": _find_flat_metric(
                    learner_metrics, "/policy_loss", "/policy_loss_mean"
                ),
                "vf_loss": _find_flat_metric(
                    learner_metrics, "/vf_loss", "/vf_loss_unclipped"
                ),
                "entropy": _find_flat_metric(
                    learner_metrics, "/entropy", "/entropy_mean"
                ),
                "learner_metrics": learner_metrics,
                "termination_metrics": {
                    key: value
                    for key, value in env_metrics.items()
                    if "/episode_end/" in key
                    or key.endswith("/reward_loss/terminated")
                    or key.endswith("/reward_loss/truncated")
                },
            }
            history = _merge_iteration_history(history, row)
            iterations_completed_this_process += 1
            # Full per-iteration metrics go to a .jsonl log (keeps the terminal
            # clean for the live progress bar); the monitor prints a concise
            # permanent line above the bar. Rewrite the small canonical history
            # atomically so a retried logical iteration replaces its prior row
            # instead of leaving duplicates.
            try:
                _write_iteration_history(
                    output_dir / "train_iterations.jsonl",
                    history,
                )
            except Exception as exc:
                print(f"[log] {exc}", flush=True)
            monitor.on_iteration_complete(
                iteration,
                row["episode_return_mean"],
                row["episode_len_mean"],
                row["num_env_steps_sampled_lifetime"],
                iter_seconds,
            )

            if tb_writer is not None:
                steps = row["num_env_steps_sampled_lifetime"]
                x = int(steps) if isinstance(steps, (int, float)) and steps else iteration
                tb_logging.log_result_scalars(tb_writer, result, x)

            if iteration % args.checkpoint_every == 0 or iteration == args.iterations:
                monitor.set_phase(
                    f"checkpoint_last iteration {iteration}",
                    args.checkpoint_timeout_s,
                )
                _save_checkpoint_pair(algo, output_dir, "last", iteration)

            if _finite(ret) and float(ret) > best_return:
                best_return = float(ret)
                monitor.set_phase(
                    f"checkpoint_best iteration {iteration}",
                    args.checkpoint_timeout_s,
                )
                _save_checkpoint_pair(algo, output_dir, "best", iteration)
    except KeyboardInterrupt:
        interrupted = True
        stop_reason = "user_interrupt"
        stop_message = "Training INTERRUPTED by user."
        error = "KeyboardInterrupt"
    except BaseException as exc:  # noqa: BLE001 — distinguish real errors here
        stop_reason = "error"
        stop_message = (
            f"Training STOPPED — ERROR ({type(exc).__name__}): {exc} "
            "See the traceback above and faulthandler.log."
        )
        error = traceback.format_exc()
        _dump_all_stacks(output_dir, f"FATAL ERROR {type(exc).__name__}")
        print(error, flush=True)

    # Replace the (possibly already expired) last-iteration deadline with a fresh
    # finalizing window so the self-guard cannot hard-exit while we write the
    # summary; cleanup phases below re-arm their own deadlines.
    monitor.set_phase("finalizing", max(args.cleanup_timeout_s, 60.0))

    if stop_reason == "completed" and stop_message is None:
        stop_message = (
            f"Training COMPLETE — logical target {args.iterations}, "
            f"{len(history)} successful iteration(s), best return "
            f"{_fmt_metric(best_return if math.isfinite(best_return) else None)}."
        )

    successful_iterations = [
        int(row["iteration"])
        for row in history
        if isinstance(row, dict) and isinstance(row.get("iteration"), int)
    ]
    summary = {
        "ok": stop_reason == "completed",
        "stop_reason": stop_reason,
        "stop_message": stop_message,
        "interrupted": interrupted,
        "timed_out": False,
        "error": error,
        "elapsed_wall_time_s": time.perf_counter() - start_time,
        "output_dir": str(output_dir),
        "rllib_log_root": str(output_dir / "rllib"),
        "setup_xml_path": args.setup_xml,
        "iterations_run": len(history),
        "iterations_completed": max(successful_iterations, default=iteration_start - 1),
        "iterations_completed_this_process": iterations_completed_this_process,
        "iteration_start": iteration_start,
        "next_iteration": args.iterations + 1 if stop_reason == "completed" else None,
        "resume_from": str(resume_path) if resume_path is not None else None,
        "restored_training_iteration": restored_training_iteration,
        "restored_logical_iteration": restored_logical_iteration,
        "best_episode_return_mean": best_return if math.isfinite(best_return) else None,
        "env_name": (
            env_factory.ENV_NAME if _TRAINING_STACK_LOADED else None
        ),
        "num_env_runners": args.num_env_runners,
        "ray_num_cpus": ray_num_cpus,
        "iteration_timeout_s": args.iteration_timeout_s,
        "max_consecutive_skips": args.max_consecutive_skips,
        "max_consecutive_crash_restarts": args.max_consecutive_crash_restarts,
        "startup_timeout_s": args.startup_timeout_s,
        "checkpoint_timeout_s": args.checkpoint_timeout_s,
        "cleanup_timeout_s": args.cleanup_timeout_s,
        "sample_timeout_s": args.sample_timeout_s,
        "step_wall_timeout_s": args.step_wall_timeout_s,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "online_grf_observation": bool(args.online_grf_observation),
        "prescribed_grf_disabled_sides": list(args.disable_prescribed_grf_side),
        "online_grf_applied_sides": list(args.online_grf_applied_side),
        "reward_config": (
            reward_function.RewardConfig.from_mapping(reward_overrides).to_dict()
            if _TRAINING_STACK_LOADED
            else reward_overrides
        ),
        "tensorboard_dir": (
            str(output_dir / "tensorboard") if args.tensorboard else None
        ),
        "history": history,
        "checkpoint_last": str(output_dir / "checkpoint_last"),
        "checkpoint_best": str(output_dir / "checkpoint_best"),
        "rl_module_last": str(output_dir / "rl_module_last"),
        "rl_module_best": str(output_dir / "rl_module_best"),
    }
    _write_json(output_dir / "summary.json", summary)

    if tb_writer is not None:
        try:
            tb_writer.flush()
            tb_writer.close()
        except Exception:
            pass

    # The external supervisor enforces cleanup_timeout_s on these calls.
    if algo is not None:
        try:
            monitor.set_phase("algo.stop", args.cleanup_timeout_s)
            algo.stop()
        except Exception as exc:
            print(f"[cleanup] {exc}", flush=True)
    if _TRAINING_STACK_LOADED and ray.is_initialized():
        try:
            monitor.set_phase("ray.shutdown", args.cleanup_timeout_s)
            ray.shutdown()
        except Exception as exc:
            print(f"[cleanup] {exc}", flush=True)

    monitor.set_phase("complete", 0.0)
    monitor.stop(
        final_message=(
            stop_message
            or f"Training finished: {len(history)}/{args.iterations} iterations, "
            f"elapsed {progress_display.format_hms(summary['elapsed_wall_time_s'])}."
        )
    )
    return summary


def _save_module(algo, path: Path) -> None:
    """Export the inference RLModule (default_policy) to a stable directory."""
    module = algo.get_module("default_policy")
    module.save_to_path(Path(path).resolve())  # absolute: pyarrow from_uri


def _save_checkpoint_pair(
    algo,
    output_dir: Path,
    label: str,
    logical_iteration: int,
) -> None:
    algo.save_to_path(output_dir / f"checkpoint_{label}")
    _save_module(algo, output_dir / f"rl_module_{label}")
    _write_json(
        output_dir / f"checkpoint_{label}_meta.json",
        {
            "logical_iteration": int(logical_iteration),
            "rllib_training_iteration": int(getattr(algo, "iteration", 0) or 0),
            "checkpoint": str(output_dir / f"checkpoint_{label}"),
        },
    )


def _terminate_process_tree(
    process: subprocess.Popen,
    windows_job: _WindowsProcessJob | None = None,
) -> None:
    """Terminate only the supervised training process and its descendants."""
    if windows_job is not None:
        windows_job.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        return
    if os.name != "nt":
        try:
            os.killpg(process.pid, signal.SIGTERM)
            if process.poll() is None:
                process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except Exception:
                if process.poll() is None:
                    process.kill()
        return
    if process.poll() is not None:
        return
    result = subprocess.run(
        [r"C:\Windows\System32\taskkill.exe", "/PID", str(process.pid), "/T", "/F"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=30,
        check=False,
    )
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
    if result.returncode != 0 and process.poll() is None:
        raise RuntimeError(
            f"Unable to terminate training process tree PID {process.pid}."
        )


def _read_watchdog_state(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return value if isinstance(value, dict) else None


def _file_mtime_ns(path: Path) -> int | None:
    """Return a file timestamp suitable for detecting a child-written summary."""
    try:
        return path.stat().st_mtime_ns
    except OSError:
        return None


def _last_checkpoint(output_dir: Path) -> tuple[Path | None, int | None]:
    """Return checkpoint_last only when its directory and metadata agree."""
    checkpoint_path = output_dir / "checkpoint_last"
    checkpoint_meta = _read_json_dict(output_dir / "checkpoint_last_meta.json")
    if not checkpoint_path.is_dir() or not checkpoint_meta:
        return None, None
    try:
        logical_iteration = int(checkpoint_meta["logical_iteration"])
    except (KeyError, TypeError, ValueError):
        return None, None
    return checkpoint_path.resolve(), logical_iteration


def _checkpoint_logical_iteration(checkpoint_path: Path | None) -> int | None:
    """Read the logical iteration associated with an arbitrary checkpoint."""
    if checkpoint_path is None or not checkpoint_path.is_dir():
        return None
    checkpoint_meta = _read_json_dict(
        checkpoint_path.parent / f"{checkpoint_path.name}_meta.json"
    )
    try:
        return int(checkpoint_meta["logical_iteration"])
    except (KeyError, TypeError, ValueError):
        return None


def _prefer_output_checkpoint_on_resume(
    output_dir: Path,
    requested_resume: Path | None,
    iteration_start: int,
) -> tuple[Path | None, int, str | None]:
    """Avoid rolling a manually resumed run back behind its own latest progress.

    A common manual-restart command keeps pointing at the original seed
    checkpoint while reusing the fine-tuning output directory. Once that output
    directory contains a newer ``checkpoint_last``, restoring the seed again
    would repeat already-completed logical iterations and append duplicate
    history rows. Prefer the newer output checkpoint unless the caller supplied
    an explicit logical ``iteration_start``.
    """
    if requested_resume is None or iteration_start > 0:
        return requested_resume, iteration_start, None

    output_checkpoint, output_iteration = _last_checkpoint(output_dir)
    requested_iteration = _checkpoint_logical_iteration(requested_resume)
    if (
        output_checkpoint is None
        or output_iteration is None
        or requested_iteration is None
        or output_iteration <= requested_iteration
    ):
        return requested_resume, iteration_start, None

    message = (
        f"Output checkpoint_last is newer than requested resume checkpoint "
        f"({output_iteration} > {requested_iteration}); continuing from "
        f"{output_checkpoint} at logical iteration {output_iteration + 1} "
        "to avoid rollback and duplicate iterations."
    )
    return output_checkpoint, output_iteration + 1, message


def _watchdog_iteration(state: dict[str, Any] | None) -> int | None:
    """Extract the active logical iteration from a child heartbeat."""
    if not state:
        return None
    phase = str(state.get("phase", ""))
    prefix = "algo.train iteration "
    if not phase.startswith(prefix):
        return None
    try:
        return int(phase[len(prefix) :])
    except ValueError:
        return None


def _supervised_child_env() -> dict[str, str]:
    """Ensure a directly-invoked Windows Conda interpreter can resolve its DLLs."""
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
    prepend = [
        str(path)
        for path in candidates
        if path.is_dir() and str(path).lower() not in {item.lower() for item in existing}
    ]
    env["PATH"] = os.pathsep.join(prepend + existing)
    env["CONDA_PREFIX"] = str(prefix)
    return env


def _worker_child_args(
    output_dir: Path,
    resume_from: Path | None,
    iteration_start: int,
) -> list[str]:
    child_args = [value for value in sys.argv[1:] if value != "--worker-process"]
    child_args.extend(["--output-dir", str(output_dir), "--worker-process"])
    if resume_from is not None:
        child_args.extend(["--resume-from", str(resume_from)])
    if iteration_start > 0:
        child_args.extend(["--iteration-start", str(iteration_start)])
    return child_args


def run_supervised(args: argparse.Namespace) -> int:
    """Restart timed-out or crashed training children until completion."""
    output_dir = _resolve_output_dir(args.output_dir, "baseline_mlp")
    output_dir.mkdir(parents=True, exist_ok=True)
    watchdog_path = output_dir / _WATCHDOG_FILENAME
    summary_path = output_dir / "summary.json"
    supervisor_state_path = output_dir / _SUPERVISOR_STATE_FILENAME
    resume_from = _resolve_resume_path(args.resume_from)
    iteration_start = max(0, int(args.iteration_start or 0))
    resume_from, iteration_start, resume_upgrade_message = (
        _prefer_output_checkpoint_on_resume(
            output_dir,
            resume_from,
            iteration_start,
        )
    )
    if resume_upgrade_message is not None:
        print(f"[supervisor] {resume_upgrade_message}", flush=True)
    skipped_iterations: list[int] = []
    consecutive_skips = 0
    restart_count = 0
    crash_restart_count = 0
    consecutive_crash_restarts = 0
    last_crash_checkpoint_iteration: int | None = None
    crash_restarts: list[dict[str, Any]] = []

    if resume_from is None and iteration_start == 0:
        for filename in (
            _WATCHDOG_FILENAME,
            _SUPERVISOR_STATE_FILENAME,
            "summary.json",
            "faulthandler.log",
            "train_iterations.jsonl",
            "checkpoint_last_meta.json",
            "checkpoint_best_meta.json",
        ):
            try:
                (output_dir / filename).unlink(missing_ok=True)
            except OSError:
                pass

    while True:
        watchdog_path.unlink(missing_ok=True)
        summary_mtime_before = _file_mtime_ns(summary_path)
        process = subprocess.Popen(
            [
                sys.executable,
                str(_SCRIPT_PATH),
                *_worker_child_args(output_dir, resume_from, iteration_start),
            ],
            env=_supervised_child_env(),
            creationflags=(
                subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
            ),
            start_new_session=(os.name != "nt"),
        )
        windows_job = None
        if os.name == "nt":
            try:
                windows_job = _WindowsProcessJob(process)
            except OSError as exc:
                print(f"[watchdog] Windows Job Object unavailable: {exc}", flush=True)

        child_started = time.time()
        timeout_error: str | None = None
        timed_out_phase: str | None = None
        try:
            while process.poll() is None:
                state = _read_watchdog_state(watchdog_path)
                now = time.time()
                if (
                    state is None
                    and args.startup_timeout_s > 0.0
                    and now - child_started > args.startup_timeout_s
                ):
                    timed_out_phase = "initial_heartbeat"
                    timeout_error = (
                        "Training worker did not publish its initial heartbeat "
                        f"within {args.startup_timeout_s:g} s."
                    )
                    break
                if state is not None:
                    phase = str(state.get("phase", "unknown"))
                    timeout_s = float(state.get("timeout_s", 0.0) or 0.0)
                    started_at = float(state.get("started_at_unix_s", now))
                    # The child owns algo.train() deadlines with time.monotonic(),
                    # so suspend/sleep cannot create a false iteration timeout.
                    if (
                        not phase.startswith("algo.train iteration ")
                        and timeout_s > 0.0
                        and now - started_at > timeout_s
                    ):
                        timed_out_phase = phase
                        timeout_error = (
                            f"{phase} exceeded the {timeout_s:g} s wall-clock timeout."
                        )
                        break
                time.sleep(0.5)
        except KeyboardInterrupt:
            timed_out_phase = "user_interrupt"
            timeout_error = "Training interrupted by user."

        if timeout_error is not None:
            if timed_out_phase == "user_interrupt":
                stop_reason = "user_interrupt"
            elif timed_out_phase == "initial_heartbeat":
                stop_reason = "startup_timeout"
            else:
                stop_reason = "phase_timeout"
            stop_message = (
                f"Training STOPPED by supervisor - {stop_reason}: {timeout_error}"
            )
            print(f"[supervisor] {stop_message}", flush=True)
            _terminate_process_tree(process, windows_job)
            if windows_job is not None:
                windows_job.close()
            summary = _read_json_dict(summary_path)
            summary.update(
                {
                    "ok": False,
                    "stop_reason": stop_reason,
                    "stop_message": stop_message,
                    "timed_out": timed_out_phase != "user_interrupt",
                    "interrupted": timed_out_phase == "user_interrupt",
                    "error": timeout_error,
                    "timed_out_phase": timed_out_phase,
                    "output_dir": str(output_dir),
                    "skipped_iterations": skipped_iterations,
                    "restart_count": restart_count,
                    "crash_restart_count": crash_restart_count,
                    "crash_restarts": crash_restarts,
                }
            )
            _write_json(summary_path, summary)
            _write_json(
                supervisor_state_path,
                {
                    "status": stop_reason,
                    "restart_count": restart_count,
                    "skipped_iterations": skipped_iterations,
                    "consecutive_skips": consecutive_skips,
                    "crash_restart_count": crash_restart_count,
                    "consecutive_crash_restarts": consecutive_crash_restarts,
                },
            )
            return 124 if timed_out_phase != "user_interrupt" else 130

        child_code = int(process.returncode or 0)
        summary_fresh = _file_mtime_ns(summary_path) != summary_mtime_before
        summary = _read_json_dict(summary_path) if summary_fresh else {}
        stop_reason = str(summary.get("stop_reason", "error"))

        if stop_reason == "iteration_timeout":
            # The hard-exited child may have left Ray descendants. Tear down the
            # entire process group/job before starting a fresh interpreter.
            _terminate_process_tree(process, windows_job)
            if windows_job is not None:
                windows_job.close()

            timed_out_iteration = int(summary.get("timed_out_iteration") or 0)
            completed_this_process = int(
                summary.get("iterations_completed_this_process") or 0
            )
            consecutive_skips = (
                1 if completed_this_process > 0 else consecutive_skips + 1
            )
            if timed_out_iteration > 0:
                skipped_iterations.append(timed_out_iteration)
            restart_count += 1

            checkpoint_meta = _read_json_dict(
                output_dir / "checkpoint_last_meta.json"
            )
            checkpoint_path = output_dir / "checkpoint_last"
            if checkpoint_meta and checkpoint_path.is_dir():
                resume_from = checkpoint_path.resolve()
            iteration_start = int(
                summary.get("next_iteration")
                or (timed_out_iteration + 1 if timed_out_iteration > 0 else 1)
            )

            event = (
                f"Iteration {timed_out_iteration} SKIPPED - restarting fresh "
                f"process from {resume_from or 'initial state'} at logical "
                f"iteration {iteration_start} "
                f"({consecutive_skips}/{args.max_consecutive_skips} consecutive)."
            )
            print(f"[supervisor] {event}", flush=True)
            if consecutive_skips >= args.max_consecutive_skips:
                stop_message = (
                    f"Training ABORTED - {consecutive_skips} consecutive "
                    "iteration timeouts."
                )
                summary.update(
                    {
                        "ok": False,
                        "stop_reason": "aborted_consecutive_skips",
                        "stop_message": stop_message,
                        "skipped_iterations": skipped_iterations,
                        "consecutive_skips": consecutive_skips,
                        "restart_count": restart_count,
                        "crash_restart_count": crash_restart_count,
                        "crash_restarts": crash_restarts,
                    }
                )
                _write_json(summary_path, summary)
                _write_json(
                    supervisor_state_path,
                    {
                        "status": "aborted_consecutive_skips",
                        "restart_count": restart_count,
                        "skipped_iterations": skipped_iterations,
                        "consecutive_skips": consecutive_skips,
                        "crash_restart_count": crash_restart_count,
                        "consecutive_crash_restarts": consecutive_crash_restarts,
                    },
                )
                print(f"[supervisor] {stop_message}", flush=True)
                return 124

            _write_json(
                supervisor_state_path,
                {
                    "status": "restarting",
                    "restart_count": restart_count,
                    "skipped_iterations": skipped_iterations,
                    "consecutive_skips": consecutive_skips,
                    "crash_restart_count": crash_restart_count,
                    "consecutive_crash_restarts": consecutive_crash_restarts,
                    "resume_from": str(resume_from) if resume_from else None,
                    "next_iteration": iteration_start,
                },
            )
            continue

        recoverable_child_crash = (
            not summary_fresh
            or stop_reason == "error"
            or (
                child_code != 0
                and stop_reason not in {"completed", "user_interrupt"}
            )
        )
        if recoverable_child_crash:
            # Native failures (for example a Ray access violation) commonly kill
            # the child before it can refresh summary.json. Always kill lingering
            # Ray descendants, then retry the failed logical iteration from the
            # latest complete checkpoint. Explicit Python errors are retried too,
            # but the no-progress limit prevents deterministic failures looping.
            child_state = _read_watchdog_state(watchdog_path)
            child_phase = str((child_state or {}).get("phase", "unknown"))
            crashed_iteration = _watchdog_iteration(child_state)
            child_stop_reason = stop_reason if summary_fresh else "unreported_exit"
            checkpoint_path, checkpoint_iteration = _last_checkpoint(output_dir)

            _terminate_process_tree(process, windows_job)
            if windows_job is not None:
                windows_job.close()

            if checkpoint_iteration == last_crash_checkpoint_iteration:
                consecutive_crash_restarts += 1
            else:
                consecutive_crash_restarts = 1
            last_crash_checkpoint_iteration = checkpoint_iteration

            crash_event = {
                "exit_code": child_code,
                "child_stop_reason": child_stop_reason,
                "summary_fresh": summary_fresh,
                "phase": child_phase,
                "crashed_iteration": crashed_iteration,
                "checkpoint_iteration": checkpoint_iteration,
                "consecutive_crash_restarts": consecutive_crash_restarts,
            }
            crash_restarts.append(crash_event)

            if checkpoint_path is None or checkpoint_iteration is None:
                stop_message = (
                    "Training ABORTED - child crashed without a valid "
                    "checkpoint_last to resume."
                )
                summary.update(
                    {
                        "ok": False,
                        "stop_reason": "unrecoverable_child_crash",
                        "stop_message": stop_message,
                        "child_exit_code": child_code,
                        "child_stop_reason": child_stop_reason,
                        "crash_restarts": crash_restarts,
                        "crash_restart_count": crash_restart_count,
                        "restart_count": restart_count,
                    }
                )
                _write_json(summary_path, summary)
                _write_json(
                    supervisor_state_path,
                    {
                        "status": "unrecoverable_child_crash",
                        "restart_count": restart_count,
                        "crash_restart_count": crash_restart_count,
                        "consecutive_crash_restarts": consecutive_crash_restarts,
                        "last_crash": crash_event,
                    },
                )
                print(f"[supervisor] {stop_message}", flush=True)
                return max(1, child_code)

            if consecutive_crash_restarts > args.max_consecutive_crash_restarts:
                stop_message = (
                    "Training ABORTED - exceeded "
                    f"{args.max_consecutive_crash_restarts} consecutive child "
                    f"crash restart(s) without checkpoint progress."
                )
                summary.update(
                    {
                        "ok": False,
                        "stop_reason": "aborted_consecutive_crashes",
                        "stop_message": stop_message,
                        "child_exit_code": child_code,
                        "child_stop_reason": child_stop_reason,
                        "crash_restarts": crash_restarts,
                        "crash_restart_count": crash_restart_count,
                        "restart_count": restart_count,
                    }
                )
                _write_json(summary_path, summary)
                _write_json(
                    supervisor_state_path,
                    {
                        "status": "aborted_consecutive_crashes",
                        "restart_count": restart_count,
                        "crash_restart_count": crash_restart_count,
                        "consecutive_crash_restarts": consecutive_crash_restarts,
                        "last_crash": crash_event,
                    },
                )
                print(f"[supervisor] {stop_message}", flush=True)
                return max(1, child_code)

            resume_from = checkpoint_path
            iteration_start = checkpoint_iteration + 1
            restart_count += 1
            crash_restart_count += 1
            event = (
                f"Child crash in {child_phase!r} (exit {child_code}, "
                f"summary={child_stop_reason}) - restarting from checkpoint "
                f"iteration {checkpoint_iteration}; retry logical iteration "
                f"{iteration_start} ({consecutive_crash_restarts}/"
                f"{args.max_consecutive_crash_restarts} without progress)."
            )
            print(f"[supervisor] {event}", flush=True)
            _write_json(
                supervisor_state_path,
                {
                    "status": "restarting_after_child_crash",
                    "restart_count": restart_count,
                    "crash_restart_count": crash_restart_count,
                    "consecutive_crash_restarts": consecutive_crash_restarts,
                    "resume_from": str(resume_from),
                    "next_iteration": iteration_start,
                    "last_crash": crash_event,
                },
            )
            time.sleep(5.0)
            continue

        if child_code == 124:
            _terminate_process_tree(process, windows_job)
        if windows_job is not None:
            windows_job.close()
        summary.update(
            {
                "skipped_iterations": skipped_iterations,
                "consecutive_skips": consecutive_skips,
                "restart_count": restart_count,
                "crash_restart_count": crash_restart_count,
                "crash_restarts": crash_restarts,
                "supervisor_resume_from": (
                    str(resume_from) if resume_from is not None else None
                ),
            }
        )
        if stop_reason == "completed" and (
            skipped_iterations or crash_restart_count
        ):
            summary["stop_message"] = (
                f"Training COMPLETE - logical target {args.iterations}, "
                f"{len(skipped_iterations)} skipped iteration(s), "
                f"{crash_restart_count} crash restart(s), "
                f"{restart_count} total restart(s)."
            )
        _write_json(summary_path, summary)
        _write_json(
            supervisor_state_path,
            {
                "status": stop_reason,
                "restart_count": restart_count,
                "skipped_iterations": skipped_iterations,
                "consecutive_skips": consecutive_skips,
                "crash_restart_count": crash_restart_count,
                "consecutive_crash_restarts": consecutive_crash_restarts,
                "resume_from": str(resume_from) if resume_from else None,
            },
        )
        return max(1, child_code) if stop_reason != "completed" else 0


def _run_watchdog_probe(args: argparse.Namespace) -> None:
    """Intentionally stall so the parent supervisor can prove it terminates us."""
    output_dir = _resolve_output_dir(args.output_dir, "baseline_mlp_probe")
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_watchdog_state(
        output_dir,
        "intentional watchdog probe stall",
        args.watchdog_probe_timeout_s,
    )
    time.sleep(max(30.0, args.watchdog_probe_timeout_s * 10.0))


def parse_args() -> argparse.Namespace:
    # First pass: read only --config so training_cfg.yaml can seed the defaults of
    # every argument below (an explicit CLI flag still overrides the YAML value).
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=str(training_config.DEFAULT_CONFIG_PATH))
    pre_args, _ = pre.parse_known_args()
    cfg = training_config.load(training_config.resolve_config_path(pre_args.config))
    cfg_defaults, cfg_reward = training_config.to_argparse_defaults(cfg)

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--config",
        default=str(training_config.DEFAULT_CONFIG_PATH),
        help="YAML config (training_cfg.yaml) read as DEFAULTS for the parameters "
        "below; an explicit CLI flag still overrides it. Use a different file "
        "(e.g. training_cfg.mac.yaml) for other setups.",
    )
    p.add_argument("--setup-xml", default=_DEFAULT_SETUP_XML)
    p.add_argument("--output-dir")
    p.add_argument(
        "--name",
        default=None,
        help="Optional suffix for the auto-generated training output directory "
        "(e.g. --name _example). Ignored when --output-dir is provided.",
    )
    p.add_argument("--segment-duration", type=float, default=0.01)
    p.add_argument("--episode-duration", type=float, default=0.5)
    p.add_argument(
        "--episode-start-offset-s",
        type=float,
        default=0.0,
        help="Deterministic offset from setup t_start when --random-init is off.",
    )
    p.add_argument("--policy-knots", type=int, default=3)
    p.add_argument(
        "--action-mode",
        choices=["absolute", "delta", "raw"],
        default="absolute",
        help=(
            "Policy output semantics. 'absolute' (default): the policy emits an "
            "ABSOLUTE prosthetic trajectory over the env's absolute_bounds_rad "
            "(ex-novo generation). 'delta': offset from the prescribed IK "
            "(imitative). 'raw': raw radians."
        ),
    )
    p.add_argument("--max-delta-rad", type=float, default=0.35)
    p.add_argument(
        "--pros-ref-governor",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Bound the served prosthetic reference velocity/acceleration.",
    )
    p.add_argument(
        "--pros-ref-model",
        choices=("second_order", "butterworth3_jerk_limited"),
        default="second_order",
    )
    p.add_argument("--pros-ref-cutoff-hz", type=float, default=6.0)
    p.add_argument("--pros-knee-ref-velocity-limit-rad-s", type=float, default=6.0)
    p.add_argument("--pros-ankle-ref-velocity-limit-rad-s", type=float, default=3.5)
    p.add_argument(
        "--pros-knee-ref-acceleration-limit-rad-s2", type=float, default=60.0
    )
    p.add_argument(
        "--pros-ankle-ref-acceleration-limit-rad-s2", type=float, default=55.0
    )
    p.add_argument("--pros-knee-ref-jerk-limit-rad-s3", type=float, default=3000.0)
    p.add_argument("--pros-ankle-ref-jerk-limit-rad-s3", type=float, default=2750.0)
    p.add_argument(
        "--actor-cyclic-phase-only",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    p.add_argument(
        "--include-reference-state-observation",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    p.add_argument(
        "--imitation-initialize-to-target",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Initialize served reference directly at the anti-phase target. "
        "Disable for a physically continuous C2 transition from the reset pose.",
    )
    p.add_argument(
        "--reward-reference-range-floor",
        type=float,
        default=0.05,
        help="Minimum q range used to normalize reward tracking/reference losses.",
    )
    p.add_argument(
        "--reward-reference-velocity-range-floor",
        type=float,
        default=0.1,
        help="Minimum qdot range used to normalize reward tracking losses.",
    )
    p.add_argument("--iterations", type=int, default=1)
    p.add_argument("--num-env-runners", type=int, default=0)
    p.add_argument(
        "--ray-num-cpus",
        type=int,
        default=None,
        help="CPUs exposed to Ray. Default: num_env_runners + 1, preventing Ray "
        "from pre-spawning one idle worker per host CPU.",
    )
    p.add_argument(
        "--iteration-timeout-s",
        type=float,
        default=900.0,
        help="Per-iteration wall-clock budget for each algo.train() call, measured "
        "with a monotonic clock (immune to machine sleep/suspend). On overrun the "
        "child exits and the supervisor starts a fresh process from checkpoint_last "
        "at the next logical iteration. Set this generously; <=0 disables.",
    )
    p.add_argument(
        "--max-consecutive-skips",
        type=int,
        default=5,
        help="Abort the run after this many consecutive skipped iterations "
        "(a likely systemic divergence). The counter resets on any good iteration.",
    )
    p.add_argument(
        "--max-consecutive-crash-restarts",
        type=int,
        default=5,
        help="Maximum child crash restarts allowed without checkpoint progress. "
        "Native/unreported crashes retry the failed logical iteration from "
        "checkpoint_last; the next crash aborts the run.",
    )
    p.add_argument(
        "--startup-timeout-s",
        type=float,
        default=300.0,
        help="Wall-clock timeout for PPOConfig.build_algo().",
    )
    p.add_argument(
        "--checkpoint-timeout-s",
        type=float,
        default=120.0,
        help="Wall-clock timeout for each checkpoint export.",
    )
    p.add_argument(
        "--cleanup-timeout-s",
        type=float,
        default=30.0,
        help="Wall-clock timeout for algo.stop()/ray.shutdown().",
    )
    p.add_argument(
        "--sample-timeout-s",
        type=float,
        default=120.0,
        help="RLlib EnvRunner sampling timeout.",
    )
    p.add_argument(
        "--step-wall-timeout-s",
        type=float,
        default=30.0,
        help="Per-env-step wall-clock budget [s]. A degenerate, pathologically "
        "slow simulation segment is truncated gracefully (end_reason="
        "'step_wall_timeout') so one slow worker cannot stall synchronous "
        "sampling. A healthy segment runs in well under a second; 0 disables.",
    )
    p.add_argument(
        "--child-self-timeout",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In-process self-guard that hard-exits the training child when a "
        "phase (iteration/sampling/checkpoint) overruns its timeout, even if the "
        "main thread is blocked in a C-level ray.wait (default: on).",
    )
    p.add_argument(
        "--progress",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Live in-place progress bar with percent, iteration counter, "
        "elapsed and ETA (default: on). Use --no-progress for plain logging.",
    )
    p.add_argument(
        "--verbose-workers",
        action="store_true",
        help="Forward every Ray worker's stdout/stderr and Python warnings to "
        "the driver console (default: off). Off keeps a clean log with a single "
        "live progress line; turn on only to debug worker-side issues.",
    )
    p.add_argument("--train-batch-size", type=int, default=2048)
    p.add_argument("--minibatch-size", type=int, default=128)
    p.add_argument("--num-epochs", type=int, default=10)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--gamma", type=float, default=0.95)
    p.add_argument("--lam", type=float, default=0.9)
    p.add_argument("--clip-param", type=float, default=0.2)
    p.add_argument(
        "--vf-clip-param",
        type=float,
        default=None,
        help="RLlib PPO value-function clipping parameter. None keeps RLlib default.",
    )
    p.add_argument(
        "--vf-loss-coeff",
        type=float,
        default=None,
        help="RLlib PPO value-function loss coefficient. None keeps RLlib default.",
    )
    p.add_argument(
        "--num-hidden-layers",
        type=int,
        default=2,
        help="Number of hidden layers (uniform width) of the actor/critic MLP.",
    )
    p.add_argument(
        "--dim-hidden-layers",
        type=int,
        default=256,
        help="Neurons per hidden layer -> fcnet_hiddens = [dim] * num.",
    )
    p.add_argument(
        "--fcnet-activation",
        default="tanh",
        help="MLP activation: tanh | relu | elu | swish | silu.",
    )
    p.add_argument(
        # Deprecated: superseded by --num-hidden-layers/--dim-hidden-layers. Kept
        # (hidden) so older commands and in-flight supervisor restarts keep working;
        # an explicit list still wins over the num/dim form in build_config.
        "--fcnet-hiddens",
        type=int,
        nargs="+",
        default=None,
        help=argparse.SUPPRESS,
    )
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--checkpoint-every", type=int, default=1)
    p.add_argument(
        "--resume-from",
        default=None,
        help="Restore the full RLlib Algorithm state from this checkpoint directory "
        "before continuing. With no internal iteration override, training resumes "
        "at checkpoint training_iteration + 1.",
    )
    p.add_argument("--random-init", action="store_true")
    p.add_argument(
        "--grf-mode",
        choices=("prescribed", "online_sensor", "online"),
        default=_DEFAULT_GRF_MODE,
        help="GRF mode used during training (default: online_sensor).",
    )
    p.add_argument(
        "--online-grf-profile",
        default=_DEFAULT_ONLINE_GRF_PROFILE,
        help="onlineGRF profile JSON used by online_sensor/online.",
    )
    p.add_argument(
        "--online-grf-observation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Expose normalized online GRF, gait events, and heel-strike phase "
        "to the policy observation (default: on). Use --no-online-grf-observation "
        "with legacy checkpoints.",
    )
    p.add_argument(
        "--asymmetric-actor-critic",
        "--critic-privileged-observation",  # deprecated alias
        dest="asymmetric_actor_critic",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Asymmetric actor-critic: env emits the FULL [actor | privileged] "
        "observation Box(n_full) instead of the realistic actor-only Box(n_actor). "
        "The policy reads obs[:n_actor]; the value head reads the full vector "
        "(custom RLModule). Default off = realistic obs on DefaultModelConfig. "
        "MUST match between training and rollout.",
    )
    p.add_argument(
        "--disable-prescribed-grf-side",
        nargs="*",
        choices=("left", "right"),
        default=[],
        help="Diagnostic opt-in: keep prescribed GRF as an oracle but remove the "
        "selected side(s)' ExternalForce from model dynamics. Space-separated, "
        "e.g. --disable-prescribed-grf-side left right.",
    )
    p.add_argument(
        "--online-grf-applied-side",
        nargs="*",
        choices=("left", "right"),
        default=[],
        help="Hybrid GRF: APPLY the online contact (not just sense it) on the "
        "given side(s) so the prosthetic ankle/knee work against a real ground "
        "reaction; prescribed is auto-disabled there. Space-separated; use 'left' "
        "(prosthetic) with --grf-mode online_sensor.",
    )
    p.add_argument(
        "--reward-json",
        default=None,
        help="Reward overrides: a JSON file path or an inline JSON object of "
        "reward_function.RewardConfig fields (e.g. '{\"blend_reference\": 0.3}'). "
        "Defaults reproduce the env's original reward.",
    )
    p.add_argument(
        "--reward-mode",
        choices=("ex_novo", "imitation"),
        default=None,
        help="Reward objective: 'ex_novo' (default) or 'imitation' (prosthetic "
        "joints mirror the sound leg anti-phase, for imitation pre-training). "
        "Overrides reward_mode in --reward-json; MUST match between train and "
        "rollout.",
    )
    p.add_argument(
        "--tensorboard",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write TensorBoard logs to <output_dir>/tensorboard (default: on). "
        "Use --no-tensorboard to disable.",
    )
    p.add_argument("--worker-process", action="store_true", help=argparse.SUPPRESS)
    p.add_argument(
        "--iteration-start",
        type=int,
        default=0,
        help=argparse.SUPPRESS,
    )
    p.add_argument("--watchdog-probe", action="store_true", help=argparse.SUPPRESS)
    p.add_argument(
        "--watchdog-probe-timeout-s",
        type=float,
        default=1.5,
        help=argparse.SUPPRESS,
    )

    # Seed argparse defaults from the YAML, then parse (explicit CLI flags win).
    valid_dests = {action.dest for action in p._actions}
    for dest in sorted(set(cfg_defaults) - valid_dests):
        training_config._warn(f"config maps to unknown argument '{dest}' (ignored)")
    p.set_defaults(**{k: v for k, v in cfg_defaults.items() if k in valid_dests})
    args = p.parse_args()
    args._cfg_reward = cfg_reward
    if args.output_dir is None:
        reward_mode = _resolved_reward_mode_for_run_name(args, cfg_reward)
        args.output_dir = str(_default_training_output_dir(reward_mode, args.name))
    return args


def main() -> None:
    args = parse_args()
    if args.checkpoint_every <= 0:
        raise SystemExit("--checkpoint-every must be >= 1")
    if args.max_consecutive_skips <= 0:
        raise SystemExit("--max-consecutive-skips must be >= 1")
    if args.max_consecutive_crash_restarts <= 0:
        raise SystemExit("--max-consecutive-crash-restarts must be >= 1")
    if not args.worker_process:
        raise SystemExit(run_supervised(args))
    if args.watchdog_probe:
        _run_watchdog_probe(args)
        return
    summary = run(args)
    print(json.dumps(summary, indent=2))
    if not summary.get("ok", False):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
