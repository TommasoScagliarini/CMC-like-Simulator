"""Prepare-only pilot manifests and an explicit, resumable bounded launcher.

Preparation is pure file generation. No simulator, RLlib, Torch or training code
is imported until a subprocess is explicitly launched by ``run``.
"""
from __future__ import annotations

import copy
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
TUNING_FIELDS = frozenset({"ankle_positive_torque_weight", "ankle_positive_torque_scale_nm",
                         "knee_torque_rate_weight", "knee_torque_rate_scale_nm_s",
                         "knee_kinematic_jerk_weight", "ankle_kinematic_jerk_weight", "kinematic_jerk_scale_rad_s3"})


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def resolve_path(root: Path, value) -> Path:
    raw = os.fspath(value)
    if os.name != "nt":
        raw = raw.replace("\\", "/")
    path = Path(raw).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def verify_hashes(root: Path, hashes: dict) -> None:
    for value, digest in hashes.items():
        path = resolve_path(root, value)
        if not path.is_file() or sha256(path) != digest:
            raise ValueError(f"Prepared pilot input changed or is missing: {path}")


def checkpoint_iteration(path: Path) -> int:
    """Require a published complete full checkpoint, never infer from its name."""
    required = ("algorithm_state.pkl", "rllib_checkpoint.json", "training_cfg.resolved.yaml",
                "unified_contract.json", "unified_training_state.json", "unified_learner_state.json",
                "learner_group/learner/state.pkl",
                "learner_group/learner/rl_module/default_policy/module_state.pkl")
    if any(not (path / name).is_file() for name in required):
        raise ValueError(f"Incomplete full checkpoint: {path}")
    state = read_json(path / "unified_training_state.json")
    metadata = read_json(path.parent / f"{path.name}_meta.json")
    logical = int(state["last_completed_iteration"])
    if int(metadata["logical_iteration"]) != logical:
        raise ValueError(f"Checkpoint metadata and state disagree: {path}")
    return logical


def latest_complete_checkpoint(run_dir: Path):
    found = []
    # Milestones are atomically published; ignore hidden staging directories.
    candidates = [run_dir / "checkpoint_last", *run_dir.glob("milestone_iteration_*/checkpoint_last")]
    for path in candidates:
        try:
            found.append((checkpoint_iteration(path), path))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return max(found, key=lambda row: (row[0], "milestone_iteration_" in str(row[1]))) if found else None


def prepare(config: dict, analysis: dict, output: Path) -> dict:
    import yaml
    root = Path(config.get("_repo_root", REPO)).resolve()
    output = Path(output).resolve()
    manifest_path = output / "pilot_manifest.json"
    if manifest_path.exists():
        raise FileExistsError(f"Prepared pilot already exists: {manifest_path}")
    source = resolve_path(root, config["source_checkpoint"])
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError("Pilot output must be disjoint from the source checkpoint")
    iteration = checkpoint_iteration(source)
    source_config_path = resolve_path(root, config.get("source_config", source / "training_cfg.resolved.yaml"))
    source_config = yaml.safe_load(source_config_path.read_text(encoding="utf-8"))
    opts = config.get("pilot", {})
    candidate_role = str(opts.get("candidate", "ankle_10"))
    if candidate_role not in {"ankle_5", "ankle_10", "ankle_20"}:
        raise ValueError("Pilot candidate must be one calibrated ankle family intensity")
    candidate = analysis["candidates"][candidate_role]["overrides"]
    if set(candidate) - TUNING_FIELDS:
        raise ValueError(f"Pilot fork contains unrelated reward fields: {set(candidate) - TUNING_FIELDS}")
    if not candidate or float(candidate.get("ankle_positive_torque_weight", 0)) <= 0:
        raise ValueError(f"{candidate_role} must supply a strictly positive ankle reward weight")
    if any(float(value) != 0 for key, value in candidate.items() if key.endswith("_weight") and key != "ankle_positive_torque_weight"):
        raise ValueError("The first pilot candidate must change ankle cost only")
    if any(float(value) <= 0 for key, value in candidate.items() if "_scale_" in key):
        raise ValueError("Tuning normalization scales must be positive")
    if any(not math.isfinite(float(value)) for value in candidate.values()):
        raise ValueError("Candidate reward overrides must be finite")
    if int(opts.get("training_seed", source_config.get("model", {}).get("seed", 123))) != int(source_config.get("model", {}).get("seed", 123)):
        raise ValueError("Pilot training seed must match the full checkpoint source")
    critic = int(opts.get("critic_updates", 1))
    ppo = int(opts.get("ppo_updates", 3))
    batch = int(opts.get("batch_size", 4096))
    budget = float(opts.get("budget_minutes", 120)) * 60
    if critic != 1 or ppo != 3 or batch != 4096 or not 0 < budget <= 7200:
        raise ValueError("V1 pilot requires 1 critic + 3 PPO, batch4096 and budget in (0,120] minutes")
    if source_config.get("reward", {}).get("reward_mode") != "imitation":
        raise ValueError("Pilot requires an imitation source")
    if float(source_config["simulation"]["episode_duration"]) != 5.0:
        raise ValueError("Pilot requires the unchanged 5s source episode")
    if int(source_config["ppo"]["train_batch_size"]) != batch:
        raise ValueError("Changing source train batch size is outside the pilot protocol")
    targets = source_config.get("target_availability", {})
    if targets.get("target_reference") != "left" or float(targets.get("target_availability_probability", 1)) != 1 or targets.get("target_curriculum_probabilities"):
        raise ValueError("Pilot requires fully visible LEFT prescribed targets")
    from .references import load_references
    reused = load_references(config, root, source, source_config, analysis, output)
    hashes = dict(analysis.get("input_hashes", {}))
    if reused:
        hashes.update(reused["input_hashes"])
    verify_hashes(root, hashes)
    files = list(source.rglob("*")) + [source.parent / f"{source.name}_meta.json", source_config_path]
    for input_key in ("_config_path", "_analysis_path", "descriptive_comparison_file"):
        if config.get(input_key):
            artifact = resolve_path(root, config[input_key])
            if not artifact.is_file():
                raise FileNotFoundError(artifact)
            files.append(artifact)
    # Freeze executable source, without hashing mutable run outputs or caches.
    files += list(root.glob("*.py")) + list(PACKAGE.glob("*.py"))
    files += list((PACKAGE / "reward_tuning").glob("*.py"))
    files += list((PACKAGE.parent / "baseline_MLP").glob("*.py"))
    files += [PACKAGE / "validation/render_canonical_rollout.py", root / "plot/plotter.py"]
    setup = source_config.get("simulation", {}).get("setup_xml")
    if setup:
        setup_path = resolve_path(root, setup)
        if not setup_path.is_file():
            raise FileNotFoundError(setup_path)
        files.extend(p for p in setup_path.parent.rglob("*") if p.suffix.lower() in {".xml", ".osim", ".mot", ".sto"})
    for field, value in source_config.get("grf", {}).items():
        if value and field.endswith("_profile"):
            asset = resolve_path(root, value)
            if not asset.is_file():
                raise FileNotFoundError(asset)
            files.append(asset)
    files.extend(p for p in (root / "plugins").glob("*") if p.suffix.lower() in {".dll", ".dylib", ".so"})
    for path in files:
        if path.is_file():
            key = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
            hashes[key] = sha256(path)
    output.mkdir(parents=True, exist_ok=True)
    python = str(opts.get("python", config.get("python", sys.executable)))
    name = output.parent.name if output.name == "pilot" else output.name
    if not name or name in {".", ".."}:
        raise ValueError("A named pilot output directory is required")
    branches = copy.deepcopy(reused["branches"]) if reused else {}
    new_branches = [(candidate_role, candidate)] if reused else [("control", {}), (candidate_role, candidate)]
    for role, overrides in new_branches:
        run_dir = PACKAGE / "runs/training" / f"{name}_{role}"
        if run_dir.exists():
            raise FileExistsError(f"Training output already exists: {run_dir}")
        branch = copy.deepcopy(source_config)
        branch.setdefault("simulation", {})["iterations"] = iteration + critic + ppo
        branch.setdefault("supervision", {}).update(checkpoint_every=1, retain_iteration_checkpoints=True)
        branch.setdefault("reward", {}).update(overrides)
        branch.setdefault("run", {}).update(resume_from=str(source), output_dir=str(run_dir),
            resume_tuning_reward_fork=True, imitation_critic_rewarm_iterations=critic,
            resume_penetration_weight_override=False)
        branch_path = output / f"{role}.yaml"
        if branch_path.exists():
            raise FileExistsError(f"Branch configuration already exists: {branch_path}")
        branch_path.write_text(yaml.safe_dump(branch, sort_keys=False), encoding="utf-8")
        branches[role] = {"config": str(branch_path), "output_dir": str(run_dir),
            "source_checkpoint": str(source), "target_iteration": iteration + critic + ppo,
            "command": [python, str(PACKAGE / "train_ppo_mlp.py"), "--config", str(branch_path)]}
        hashes[str(branch_path)] = sha256(branch_path)
    evaluation = config.get("evaluation", {})
    seeds = list(evaluation.get("seeds", evaluation.get("noise_seeds", opts.get("noise_seeds", [123, 125, 127]))))
    if seeds != [123, 125, 127]:
        raise ValueError("V1 evaluation protocol requires seeds 123/125/127")
    noise_scale = float(evaluation.get("noise_scale", 1.0))
    if not math.isfinite(noise_scale) or noise_scale <= 0:
        raise ValueError("Noise scale must be finite and positive")
    cases = copy.deepcopy(reused["cases"]) if reused else []
    sigma_path = Path(reused["sigma_path"]) if reused else output / "sigma_best80.json"
    for role in ([candidate_role] if reused else ("best80", "control", candidate_role)):
        checkpoint = source if role == "best80" else Path(branches[role]["output_dir"]) / f"milestone_iteration_{iteration + critic + ppo:06d}" / "checkpoint_last"
        for scenario, seed in [("nominal", 123), *((f"noise_{s}", s) for s in seeds)]:
            cases.append({"role": role, "scenario": scenario, "noise_seed": seed,
                "environment_seed": int(evaluation.get("environment_seed", opts.get("environment_seed", 123))),
                "noise_scale": noise_scale, "maximum_steps": 500,
                "checkpoint": str(checkpoint), "sigma_path": str(sigma_path),
                "freeze_sigma": role == "best80" and scenario == "nominal",
                "output_dir": str(PACKAGE / "runs/rollout" / f"{name}_{role}_{scenario}")})
    for case in cases:
        if Path(case["output_dir"]).exists() and not (reused and case["role"] in {"best80", "control"}):
            raise FileExistsError(f"Evaluation output already exists: {case['output_dir']}")
    manifest = {"schema": "reward_tuning_pilot_v1", "repo_root": str(root), "output_dir": str(output),
        "source_checkpoint": str(source), "source_iteration": iteration, "source_config": source_config,
        "input_hashes": hashes, "branches": branches, "evaluation_cases": cases,
        "candidate_role": candidate_role, "reference_reuse": reused["provenance"] if reused else None,
        "budget_seconds": budget, "python": python,
        "critic_updates": critic, "ppo_updates": ppo, "batch_size": batch,
        "evaluation_parallelism": min(4, max(1, int(opts.get("parallel_evaluations", 4)))), "per_process_numeric_threads": 1,
        "sigma_protocol": "Freeze best80 nominal sigma per decision; reuse identical schedule and Gaussian streams for every policy",
        "training_seed": source_config.get("model", {}).get("seed"),
        "training_pairing": "Same seed and checkpoint; worker sampling trajectories need not be identical",
        "cost_estimate": {"training_updates": len(new_branches) * (critic + ppo), "training_transitions": len(new_branches) * (critic + ppo) * batch,
                          "evaluation_episodes": 4 if reused else len(cases),
                          "max_evaluation_transitions": (4 if reused else len(cases)) * 500,
                          "reused_training_updates": critic + ppo if reused else 0,
                          "reused_evaluation_episodes": 8 if reused else 0,
                          "wall_minutes_cap": budget / 60, "wall_estimate": "No throughput assumption; cap includes training, evaluation and plots"},
        "status": "prepared_only", "training_started": False}
    write_json(manifest_path, manifest)
    state = {"schema": manifest["schema"], "manifest_sha256": sha256(manifest_path),
        "status": "prepared", "elapsed_seconds": 0.0, "stages": {}, "evaluation": {}}
    if reused:
        receipt = {"status": "completed", "reused_from": str(opts["reuse_reference_experiment"]), "attempts": 0}
        state["stages"]["train_control"] = dict(receipt)
        for case in reused["cases"]:
            role, scenario = case["role"], case["scenario"]
            state["evaluation"].setdefault(role, {})[scenario] = case["output_dir"]
            state["stages"][f"eval_{role}_{scenario}"] = dict(receipt)
        state["sigma_sha256"] = sha256(sigma_path)
    write_json(output / "pilot_state.json", state)
    return manifest


def _kill_tree(process) -> None:
    """Kill descendants as well as parent, including subprocess-created sessions."""
    try:
        import psutil
        children = psutil.Process(process.pid).children(recursive=True)
        for child in reversed(children):
            try:
                child.terminate()
            except psutil.Error:
                pass
    except (ImportError, ProcessLookupError):
        children = []
    except Exception:
        children = []
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15, check=False)
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
    for child in children:
        try:
            if child.is_running():
                child.kill()
        except Exception:
            pass
    if os.name != "nt":
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def execute_command(command: list[str], log_path: Path, *, cwd: Path, remaining_seconds: float, heartbeat=None) -> dict:
    """Shell-free bounded subprocess; injectable in launcher tests."""
    env = os.environ.copy()
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    env["MPLBACKEND"] = "Agg"
    options = {"start_new_session": True} if os.name != "nt" else {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
    started = time.monotonic()
    progress_path = None
    if "--case-json" in command:
        case = read_json(Path(command[command.index("--case-json") + 1]))
        progress_path = Path(case["output_dir"]) / "watchdog_state.json"
    with log_path.open("ab") as log:
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, **options)
        try:
            while process.poll() is None:
                elapsed = time.monotonic() - started
                if heartbeat:
                    heartbeat(elapsed, process.pid)
                if progress_path:
                    now = time.time()
                    try:
                        progress = read_json(progress_path)
                    except (OSError, ValueError):
                        progress = None
                    timed_out = (not progress and elapsed > 600)
                    if progress:
                        phase_timeout = float(progress.get("timeout_s", 0))
                        updated = float(progress.get("updated_at_unix_s", now))
                        phase_start = float(progress.get("started_at_unix_s", updated))
                        timed_out = now - updated > 240 or (phase_timeout > 0 and now-phase_start > phase_timeout)
                    if timed_out:
                        _kill_tree(process)
                        return {"returncode": process.returncode, "elapsed_seconds": time.monotonic()-started, "reason": "watchdog_timeout"}
                if elapsed >= remaining_seconds:
                    _kill_tree(process)
                    return {"returncode": process.returncode, "elapsed_seconds": time.monotonic()-started, "reason": "budget_exhausted"}
                time.sleep(min(0.2, max(0.001, remaining_seconds-elapsed)))
        except BaseException:
            _kill_tree(process)
            raise
    return {"returncode": process.returncode, "elapsed_seconds": time.monotonic()-started,
            "reason": "completed" if process.returncode == 0 else "failed"}


def run(config_or_output, output: Path | None = None) -> dict:
    """Execute an already frozen manifest; never extend its cumulative budget."""
    output = Path(config_or_output if output is None else output).resolve()
    manifest_path, state_path = output / "pilot_manifest.json", output / "pilot_state.json"
    manifest, state = read_json(manifest_path), read_json(state_path)
    if state["manifest_sha256"] != sha256(manifest_path):
        raise ValueError("Prepared manifest changed; create a separate pilot")
    root = Path(manifest["repo_root"])
    verify_hashes(root, manifest["input_hashes"])
    lock = output / "pilot.lock"
    try:
        handle = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        try:
            pid = int(lock.read_text(encoding="utf-8"))
            os.kill(pid, 0)
        except ProcessLookupError:
            for record in state.get("stages", {}).values():
                if record.get("status") == "running" and record.get("pid"):
                    try:
                        os.kill(int(record["pid"]), 0)
                    except ProcessLookupError:
                        pass
                    else:
                        raise RuntimeError("Previous launcher exited but a recorded child is still alive; refusing overlapping work")
            lock.unlink()
            handle = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        else:
            raise RuntimeError("Pilot already has an active launcher; do not run it twice")
    with os.fdopen(handle, "w") as stream:
        stream.write(str(os.getpid()))
    session_start = time.monotonic()
    previous = float(state["elapsed_seconds"])
    if state.get("active_since_unix"):
        # Conservative crash accounting includes time since the last persisted
        # heartbeat. It cannot grant extra wall budget after an abrupt exit.
        previous += max(0, time.time() - float(state["active_since_unix"]))
    budget = float(manifest["budget_seconds"])
    old_sigterm = None
    if threading.current_thread() is threading.main_thread():
        old_sigterm = signal.getsignal(signal.SIGTERM)
        def interrupt_signal(_signum, _frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupt_signal)
    state_lock = threading.RLock()
    stop_event = threading.Event()
    def save():
        with state_lock:
            state["elapsed_seconds"] = previous + time.monotonic() - session_start
            state["active_since_unix"] = time.time()
            write_json(state_path, state)
    def heartbeat(record, pid):
        if stop_event.is_set():
            raise InterruptedError("Pilot interrupted")
        with state_lock:
            record.update(pid=pid)
            save()
    def stage(name, command, *, validate=None):
        if stop_event.is_set():
            return False
        prior = state["stages"].get(name, {})
        if prior.get("status") == "completed":
            if validate:
                validate()
            return True
        save()
        remaining = budget - float(state["elapsed_seconds"])
        if remaining <= 0:
            state["status"] = "budget_exhausted"
            return False
        attempts = int(prior.get("attempts", 0)) + 1
        verify_hashes(root, manifest["input_hashes"])
        # Hash validation is itself charged to the same global deadline.
        remaining = budget - (previous + time.monotonic() - session_start)
        if remaining <= 0:
            with state_lock:
                state["status"] = "budget_exhausted"
            return False
        record = {"status": "running", "attempts": attempts, "command": command}
        with state_lock:
            state["stages"][name] = record
            save()
        result = execute_command(command, output / f"{name}.log", cwd=root, remaining_seconds=remaining,
                                 heartbeat=lambda _elapsed, pid: heartbeat(record, pid))
        with state_lock:
            record.update(result, status=result["reason"])
        if result["reason"] == "completed" and validate:
            validate()
        if result["reason"] != "completed":
            with state_lock:
                state["status"] = result["reason"]
                save()
            return False
        save()
        return True
    try:
        state["status"] = "running"
        save()
        for role, branch in manifest["branches"].items():
            destination = Path(branch["output_dir"])
            latest = latest_complete_checkpoint(destination)
            command = list(branch["command"])
            if latest:
                if latest[0] == branch["target_iteration"]:
                    prior = state["stages"].get(f"train_{role}", {})
                    state["stages"][f"train_{role}"] = {**prior, "status": "completed"}
                command += ["--resume-from", str(latest[1])]
                state.setdefault("latest_checkpoints", {})[role] = {"iteration": latest[0], "path": str(latest[1])}
            def check_training(branch=branch):
                latest = latest_complete_checkpoint(Path(branch["output_dir"]))
                if latest is None or latest[0] != branch["target_iteration"]:
                    raise RuntimeError(f"Training exited without target checkpoint: {branch['output_dir']}")
                trained = read_json(latest[1] / "unified_training_state.json")
                if (trained.get("imitation_critic_rewarm_remaining") != 0
                        or trained.get("imitation_critic_rewarm_completed_iterations") != manifest["critic_updates"]
                        or trained.get("last_completed_phase") != "imitation"):
                    raise RuntimeError("Target checkpoint does not prove completed critic rewarm and return to imitation")
                # Logical iteration alone cannot prove that no update was skipped.
                expected_phases = ["imitation_critic_rewarm"] * manifest["critic_updates"] + ["imitation"] * manifest["ppo_updates"]
                for offset, phase in enumerate(expected_phases, start=1):
                    logical = manifest["source_iteration"] + offset
                    saved = Path(branch["output_dir"]) / f"milestone_iteration_{logical:06d}" / "checkpoint_last"
                    if checkpoint_iteration(saved) != logical:
                        raise RuntimeError(f"Missing successful pilot update {logical}")
                    if read_json(saved / "unified_training_state.json").get("last_completed_phase") != phase:
                        raise RuntimeError(f"Pilot update {logical} did not complete expected phase {phase}")
            if not stage(f"train_{role}", command, validate=check_training):
                return state
        def run_case(index, original_case):
            case = dict(original_case)
            key = f"eval_{case['role']}_{case['scenario']}"
            existing = state["evaluation"].get(case["role"], {}).get(case["scenario"])
            if existing:
                case["output_dir"] = existing
            prior = state["stages"].get(key, {})
            # Preserve failed attempts; completed artifacts are verified/skipped.
            if Path(case["output_dir"]).exists() and prior.get("status") != "completed":
                complete_path = Path(case["output_dir"]) / "evaluation_complete.json"
                if complete_path.is_file():
                    with state_lock:
                        state["stages"][key] = {**prior, "status": "completed"}
                else:
                    attempt = int(prior.get("attempts", 0)) + 1
                    case["output_dir"] = original_case["output_dir"] + f"_attempt{attempt:02d}"
            with state_lock:
                state["evaluation"].setdefault(case["role"], {})[case["scenario"]] = case["output_dir"]
            case_path = output / f"case_{index:02d}.json"
            write_json(case_path, case)
            def check_eval(case=case):
                folder = Path(case["output_dir"])
                receipt = read_json(folder / "evaluation_complete.json")
                hashes = receipt.get("artifact_hashes", {})
                required = {"rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json"}
                if not required.issubset(hashes):
                    raise RuntimeError(f"Evaluation receipt is incomplete: {folder}")
                verify_hashes(folder, hashes)
                noise_receipt = read_json(folder / "paired_noise_receipt.json")
                if noise_receipt.get("sigma_sha256") != sha256(Path(case["sigma_path"])):
                    raise ValueError(f"Evaluation sigma receipt disagrees: {folder}")
            command = [manifest["python"], str(PACKAGE / "reward_tuning/evaluation.py"), "--case-json", str(case_path)]
            if not stage(key, command, validate=check_eval):
                return False
            summary = read_json(Path(case["output_dir"]) / "rollout_summary.json")
            if summary.get("completed_full_episode") and not (manifest.get("reference_reuse") and case["role"] in {"best80", "control"}):
                command = [manifest["python"], str(PACKAGE / "validation/render_canonical_rollout.py"),
                           "--rollout", case["output_dir"]]
                if not stage(f"plots_{case['role']}_{case['scenario']}", command):
                    return False
            return True
        sigma_path = Path(manifest["evaluation_cases"][0]["sigma_path"])
        if state.get("sigma_sha256") and (not sigma_path.is_file() or sha256(sigma_path) != state["sigma_sha256"]):
            raise ValueError("Frozen baseline sigma changed after evaluation")
        # Freeze sigma before dispatching noisy cases; preserve a single baseline
        # policy observation sequence for every paired evaluation.
        if not run_case(0, manifest["evaluation_cases"][0]):
            return state
        state["sigma_sha256"] = sha256(sigma_path)
        save()
        executor = ThreadPoolExecutor(max_workers=manifest["evaluation_parallelism"])
        futures = [executor.submit(run_case, index, case) for index, case in
                   enumerate(manifest["evaluation_cases"][1:], start=1)]
        all_ok = True
        try:
            for future in as_completed(futures):
                all_ok = bool(future.result()) and all_ok
        except BaseException:
            stop_event.set()
            for future in futures:
                future.cancel()
            raise
        finally:
            executor.shutdown(wait=True, cancel_futures=stop_event.is_set())
        if not all_ok:
            if state["elapsed_seconds"] >= budget:
                state["status"] = "budget_exhausted"
            elif state["status"] == "running":
                state["status"] = "failed"
            return state
        verify_hashes(root, manifest["input_hashes"])
        state["status"] = "completed"
        return state
    except KeyboardInterrupt:
        stop_event.set()
        state["status"] = "interrupted"
        return state
    except Exception as exc:
        state["status"] = "failed"
        state["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        save()
        state.pop("active_since_unix", None)
        for role, branch in manifest["branches"].items():
            latest = latest_complete_checkpoint(Path(branch["output_dir"]))
            if latest:
                state.setdefault("latest_checkpoints", {})[role] = {"iteration": latest[0], "path": str(latest[1])}
        write_json(state_path, state)
        lock.unlink(missing_ok=True)
        if old_sigterm is not None:
            signal.signal(signal.SIGTERM, old_sigterm)
