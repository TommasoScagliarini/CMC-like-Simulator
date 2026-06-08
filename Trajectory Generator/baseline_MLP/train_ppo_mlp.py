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
import math  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from contextlib import contextmanager  # noqa: E402
from datetime import datetime  # noqa: E402
from typing import Any  # noqa: E402

_DEFAULT_SETUP_XML = r"models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml"
_DEFAULT_GRF_MODE = "online_sensor"
_DEFAULT_ONLINE_GRF_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)
_BASELINE_DIR = str(Path(__file__).resolve().parent)
_SCRIPT_PATH = Path(__file__).resolve()
_WATCHDOG_FILENAME = "watchdog_state.json"
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
    """Publish the current blocking phase for the external process watchdog."""
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
    temp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(temp_path, path)


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
        "action_mode": "delta",
        "max_delta_rad": args.max_delta_rad,
        "random_init": args.random_init,
        "rebuild_model_on_reset": False,
        "record_outputs": False,
        "fail_fast": True,
        "grf_mode": args.grf_mode,
        "online_grf_profile_file": args.online_grf_profile,
        "include_online_grf_observation": args.online_grf_observation,
        "prescribed_grf_disabled_sides": args.disable_prescribed_grf_side,
        "online_grf_applied_sides": args.online_grf_applied_side,
    }
    # The reward seen by the agent is shaped by reward_function.py (single source
    # of truth); these overrides reach RewardShapingWrapper via make_cmc_env.
    if reward_overrides:
        env_config["reward"] = reward_overrides

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
        .learners(num_learners=0)  # learner in the main process (CPU)
        .resources(num_cpus_for_main_process=1, num_gpus=0)
        .training(
            train_batch_size=args.train_batch_size,
            minibatch_size=args.minibatch_size,
            num_epochs=args.num_epochs,
            lr=args.lr,
            gamma=args.gamma,
            lambda_=args.lam,
            clip_param=args.clip_param,
        )
        .reporting(
            min_sample_timesteps_per_iteration=args.train_batch_size,
            metrics_num_episodes_for_smoothing=min(
                100,
                max(1, args.train_batch_size),
            ),
        )
        .rl_module(
            model_config=DefaultModelConfig(
                fcnet_hiddens=list(args.fcnet_hiddens),
                fcnet_activation="tanh",
            )
        )
        .debugging(seed=args.seed)
    )
    return config


def run(args: argparse.Namespace) -> dict:
    output_dir = Path(
        args.output_dir
        or (Path("runs") / f"baseline_mlp_{datetime.now():%Y%m%d_%H%M%S}")
    ).resolve()  # absolute: pyarrow's from_uri rejects relative paths
    output_dir.mkdir(parents=True, exist_ok=True)

    ray_num_cpus = args.ray_num_cpus
    if ray_num_cpus is None:
        ray_num_cpus = max(1, args.num_env_runners + 1)

    reward_overrides: dict = {}
    algo = None
    tb_writer = None
    history: list[dict] = []
    best_return = -float("inf")
    interrupted = False
    timed_out = False
    error: str | None = None
    start_time = time.perf_counter()

    try:
        _write_watchdog_state(
            output_dir, "training stack imports", args.startup_timeout_s
        )
        with _fault_dump_guard(
            output_dir, "training stack imports", args.startup_timeout_s
        ):
            _load_training_stack()

        reward_overrides = reward_function.load_reward_overrides(args.reward_json)
        runtime_env = {"env_vars": {"KMP_DUPLICATE_LIB_OK": "TRUE"}}
        if args.num_env_runners > 0:
            runtime_env["worker_process_setup_hook"] = _worker_setup

        _write_watchdog_state(output_dir, "ray.init", args.startup_timeout_s)
        with _fault_dump_guard(output_dir, "ray.init", args.startup_timeout_s):
            ray.init(
                include_dashboard=False,
                ignore_reinit_error=True,
                runtime_env=runtime_env,
                log_to_driver=True,
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
        _write_watchdog_state(output_dir, "PPOConfig.build_algo", args.startup_timeout_s)
        with _fault_dump_guard(
            output_dir, "PPOConfig.build_algo", args.startup_timeout_s
        ):
            algo = config.build_algo()

        if args.tensorboard:
            tb_writer = tb_logging.make_tb_writer(output_dir / "tensorboard")

        for iteration in range(1, args.iterations + 1):
            elapsed = time.perf_counter() - start_time
            iteration_timeout_s = args.iteration_timeout_s
            if args.run_timeout_s > 0.0:
                remaining = args.run_timeout_s - elapsed
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"Training run exceeded the {args.run_timeout_s:g} s "
                        "total wall-clock timeout."
                    )
                if iteration_timeout_s <= 0.0:
                    iteration_timeout_s = remaining
                else:
                    iteration_timeout_s = min(iteration_timeout_s, remaining)
            _write_watchdog_state(
                output_dir,
                f"algo.train iteration {iteration}",
                iteration_timeout_s,
            )
            with _fault_dump_guard(
                output_dir,
                f"algo.train iteration {iteration}",
                iteration_timeout_s,
            ):
                result = algo.train()
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
            history.append(row)
            print(
                json.dumps(
                    {
                        "train_iter": {
                            key: value
                            for key, value in row.items()
                            if key not in ("learner_metrics", "termination_metrics")
                        },
                        "termination_metrics": row["termination_metrics"],
                    }
                ),
                flush=True,
            )

            if tb_writer is not None:
                steps = row["num_env_steps_sampled_lifetime"]
                x = int(steps) if isinstance(steps, (int, float)) and steps else iteration
                tb_logging.log_result_scalars(tb_writer, result, x)

            if iteration % args.checkpoint_every == 0 or iteration == args.iterations:
                _write_watchdog_state(
                    output_dir,
                    f"checkpoint_last iteration {iteration}",
                    args.checkpoint_timeout_s,
                )
                _save_checkpoint_pair(algo, output_dir, "last")

            if _finite(ret) and float(ret) > best_return:
                best_return = float(ret)
                _write_watchdog_state(
                    output_dir,
                    f"checkpoint_best iteration {iteration}",
                    args.checkpoint_timeout_s,
                )
                _save_checkpoint_pair(algo, output_dir, "best")
    except KeyboardInterrupt:
        interrupted = True
        error = "KeyboardInterrupt"
    except TimeoutError as exc:
        timed_out = True
        error = str(exc)
        print(json.dumps({"training_timeout": error}), flush=True)
    except BaseException:
        error = traceback.format_exc()
        print(error, flush=True)

    summary = {
        "ok": error is None,
        "interrupted": interrupted,
        "timed_out": timed_out,
        "error": error,
        "elapsed_wall_time_s": time.perf_counter() - start_time,
        "output_dir": str(output_dir),
        "rllib_log_root": str(output_dir / "rllib"),
        "setup_xml_path": args.setup_xml,
        "iterations_run": len(history),
        "best_episode_return_mean": best_return if math.isfinite(best_return) else None,
        "env_name": (
            env_factory.ENV_NAME if _TRAINING_STACK_LOADED else None
        ),
        "num_env_runners": args.num_env_runners,
        "ray_num_cpus": ray_num_cpus,
        "iteration_timeout_s": args.iteration_timeout_s,
        "run_timeout_s": args.run_timeout_s,
        "startup_timeout_s": args.startup_timeout_s,
        "checkpoint_timeout_s": args.checkpoint_timeout_s,
        "cleanup_timeout_s": args.cleanup_timeout_s,
        "sample_timeout_s": args.sample_timeout_s,
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
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )

    if tb_writer is not None:
        try:
            tb_writer.flush()
            tb_writer.close()
        except Exception:
            pass

    # The external supervisor enforces cleanup_timeout_s on these calls.
    if algo is not None and not timed_out:
        try:
            _write_watchdog_state(output_dir, "algo.stop", args.cleanup_timeout_s)
            algo.stop()
        except Exception as exc:
            print(f"[cleanup] {exc}", flush=True)
    if _TRAINING_STACK_LOADED and ray.is_initialized():
        try:
            _write_watchdog_state(output_dir, "ray.shutdown", args.cleanup_timeout_s)
            ray.shutdown()
        except Exception as exc:
            print(f"[cleanup] {exc}", flush=True)

    _write_watchdog_state(output_dir, "complete", 0.0)
    if error is not None:
        raise RuntimeError(error)
    return summary


def _save_module(algo, path: Path) -> None:
    """Export the inference RLModule (default_policy) to a stable directory."""
    module = algo.get_module("default_policy")
    module.save_to_path(Path(path).resolve())  # absolute: pyarrow from_uri


def _save_checkpoint_pair(algo, output_dir: Path, label: str) -> None:
    algo.save_to_path(output_dir / f"checkpoint_{label}")
    _save_module(algo, output_dir / f"rl_module_{label}")


def _terminate_process_tree(
    process: subprocess.Popen,
    windows_job: _WindowsProcessJob | None = None,
) -> None:
    """Terminate only the supervised training process and its descendants."""
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
    else:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                process.kill()


def _read_watchdog_state(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return value if isinstance(value, dict) else None


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


def run_supervised(args: argparse.Namespace) -> int:
    """Run training in a child process and enforce hard per-phase timeouts."""
    output_dir = Path(
        args.output_dir
        or (Path("runs") / f"baseline_mlp_{datetime.now():%Y%m%d_%H%M%S}")
    ).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    watchdog_path = output_dir / _WATCHDOG_FILENAME
    watchdog_path.unlink(missing_ok=True)

    child_args = list(sys.argv[1:])
    if args.output_dir is None:
        child_args.extend(["--output-dir", str(output_dir)])
    child_args.append("--worker-process")
    creationflags = (
        subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
    )
    process = subprocess.Popen(
        [sys.executable, str(_SCRIPT_PATH), *child_args],
        env=_supervised_child_env(),
        creationflags=creationflags,
        start_new_session=(os.name != "nt"),
    )
    windows_job = None
    if os.name == "nt":
        try:
            windows_job = _WindowsProcessJob(process)
        except OSError as exc:
            print(f"[watchdog] Windows Job Object unavailable: {exc}", flush=True)

    run_started = time.time()
    timeout_error: str | None = None
    timed_out_phase: str | None = None
    try:
        while process.poll() is None:
            state = _read_watchdog_state(watchdog_path)
            now = time.time()
            if args.run_timeout_s > 0.0 and now - run_started > args.run_timeout_s:
                timed_out_phase = "total_run"
                timeout_error = (
                    f"Training run exceeded the {args.run_timeout_s:g} s total "
                    "wall-clock timeout."
                )
                break
            if (
                state is None
                and args.startup_timeout_s > 0.0
                and now - run_started > args.startup_timeout_s
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
                if timeout_s > 0.0 and now - started_at > timeout_s:
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
        print(json.dumps({"training_timeout": timeout_error}), flush=True)
        _terminate_process_tree(process, windows_job)
        summary_path = output_dir / "summary.json"
        existing = {}
        try:
            existing = json.loads(summary_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            pass
        existing.update(
            {
                "ok": False,
                "timed_out": timed_out_phase != "user_interrupt",
                "interrupted": timed_out_phase == "user_interrupt",
                "error": timeout_error,
                "timed_out_phase": timed_out_phase,
                "output_dir": str(output_dir),
            }
        )
        summary_path.write_text(json.dumps(existing, indent=2), encoding="utf-8")
        if windows_job is not None:
            windows_job.close()
        return 124 if timed_out_phase != "user_interrupt" else 130

    if windows_job is not None:
        windows_job.close()
    return int(process.returncode or 0)


def _run_watchdog_probe(args: argparse.Namespace) -> None:
    """Intentionally stall so the parent supervisor can prove it terminates us."""
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_watchdog_state(
        output_dir,
        "intentional watchdog probe stall",
        args.watchdog_probe_timeout_s,
    )
    time.sleep(max(30.0, args.watchdog_probe_timeout_s * 10.0))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--setup-xml", default=_DEFAULT_SETUP_XML)
    p.add_argument("--output-dir")
    p.add_argument("--segment-duration", type=float, default=0.01)
    p.add_argument("--episode-duration", type=float, default=0.5)
    p.add_argument("--policy-knots", type=int, default=3)
    p.add_argument("--max-delta-rad", type=float, default=0.35)
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
        help="Hard wall-clock timeout for each algo.train() call; <=0 disables.",
    )
    p.add_argument(
        "--run-timeout-s",
        type=float,
        default=7200.0,
        help="Total training wall-clock timeout; <=0 disables.",
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
    p.add_argument("--train-batch-size", type=int, default=2048)
    p.add_argument("--minibatch-size", type=int, default=128)
    p.add_argument("--num-epochs", type=int, default=10)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--lam", type=float, default=0.95)
    p.add_argument("--clip-param", type=float, default=0.2)
    p.add_argument("--fcnet-hiddens", type=int, nargs="+", default=[256, 256])
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--checkpoint-every", type=int, default=1)
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
        "--disable-prescribed-grf-side",
        action="append",
        choices=("left", "right"),
        default=[],
        help="Diagnostic opt-in: keep prescribed GRF as an oracle but remove "
        "the selected side's ExternalForce from model dynamics. Repeat for "
        "multiple sides.",
    )
    p.add_argument(
        "--online-grf-applied-side",
        action="append",
        choices=("left", "right"),
        default=[],
        help="Hybrid GRF: APPLY the online contact (not just sense it) on the "
        "given side so the prosthetic ankle/knee work against a real ground "
        "reaction; prescribed is auto-disabled there. Use 'left' (prosthetic) "
        "with --grf-mode online_sensor. Repeat for multiple sides.",
    )
    p.add_argument(
        "--reward-json",
        default=None,
        help="Reward overrides: a JSON file path or an inline JSON object of "
        "reward_function.RewardConfig fields (e.g. '{\"blend_reference\": 0.3}'). "
        "Defaults reproduce the env's original reward.",
    )
    p.add_argument(
        "--tensorboard",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write TensorBoard logs to <output_dir>/tensorboard (default: on). "
        "Use --no-tensorboard to disable.",
    )
    p.add_argument("--worker-process", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--watchdog-probe", action="store_true", help=argparse.SUPPRESS)
    p.add_argument(
        "--watchdog-probe-timeout-s",
        type=float,
        default=1.5,
        help=argparse.SUPPRESS,
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if not args.worker_process:
        raise SystemExit(run_supervised(args))
    if args.watchdog_probe:
        _run_watchdog_probe(args)
        return
    summary = run(args)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
