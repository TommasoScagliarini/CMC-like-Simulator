"""One production PPO iteration with observational learner instrumentation.

All runtime patches live in this process. No production source or source
checkpoint is changed. Run ``prepare`` before ``train`` to freeze the inputs.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from datetime import datetime, timezone

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
SOURCE = PACKAGE / "runs/training/MLP39_LEFT72_penetration_weight2_10iter_2026-09-24/checkpoint_best"
OUTPUT = PACKAGE / "runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29"
MANIFEST = HERE / "manifest.json"
CONFIG = HERE / "audit.yaml"


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare():
    import yaml
    if MANIFEST.exists() or OUTPUT.exists():
        raise FileExistsError("Audit already prepared or output exists; preserve it")
    config = yaml.safe_load((SOURCE / "training_cfg.resolved.yaml").read_text(encoding="utf-8"))
    state = json.loads((SOURCE / "unified_training_state.json").read_text())
    source_iteration = int(state["last_completed_iteration"])
    if source_iteration != 80 or state["reward_mode"] != "imitation":
        raise ValueError("Expected complete best80 imitation checkpoint")
    branch = copy.deepcopy(config)
    branch["simulation"]["iterations"] = source_iteration + 1
    branch["run"] = {
        "resume_from": str(SOURCE), "output_dir": str(OUTPUT),
        "imitation_critic_rewarm_iterations": 0,
        "resume_tuning_reward_fork": False,
        "resume_penetration_weight_override": False,
    }
    # Execution guards and logging do not change PPO or the physical task.
    branch["supervision"].update(checkpoint_every=1, retain_iteration_checkpoints=True)
    branch["logging"].update(progress=False)
    for section in ("reward", "model", "ppo", "parallelism", "grf", "target_availability"):
        if branch[section] != config[section]:
            raise AssertionError(f"Unexpected scientific configuration change: {section}")
    CONFIG.write_text(yaml.safe_dump(branch, sort_keys=False), encoding="utf-8")
    files = [p for p in SOURCE.rglob("*") if p.is_file()]
    files += [SOURCE.parent / "checkpoint_best_meta.json", CONFIG]
    files += list(REPO.glob("*.py")) + list(PACKAGE.glob("*.py"))
    files += list((PACKAGE.parent / "baseline_MLP").glob("*.py"))
    files += [HERE / "run_audit.py", HERE / "learner_probe.py"]
    setup = REPO / config["simulation"]["setup_xml"].replace("\\", "/")
    files += [p for p in setup.parent.rglob("*") if p.suffix.lower() in {".xml", ".osim", ".mot", ".sto"}]
    files += [p for p in (REPO / "plugins").glob("*") if p.suffix.lower() in {".dll", ".dylib", ".so"}]
    for name, value in config["grf"].items():
        if name.endswith("_profile") and value:
            files.append(REPO / value.replace("\\", "/"))
    hashes = {str(p.relative_to(REPO)): sha256(p) for p in sorted(set(files))}
    write_json(MANIFEST, {
        "schema": "rl_learning_audit_v1", "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_checkpoint": str(SOURCE), "source_iteration": source_iteration,
        "output": str(OUTPUT), "configuration": str(CONFIG), "input_hashes": hashes,
        "training_updates": 1, "train_batch_size": 4096, "minibatch_size": 512,
        "num_epochs": 10, "critic_rewarm": False, "teacher_labels": False,
        "same_reward_architecture_governor": True,
        "collection": "New on-policy sampling from restored weights; historical simulator and RNG state are unavailable",
        "evaluation": {"nominal": True, "noise_seeds": [123, 125, 127],
                       "sigma": "Each evaluated policy uses its own current sigma; pair standard Gaussian streams"},
        "limits": "One local PPO iteration and one training seed; no claim about global learnability or other models",
    })
    print(json.dumps({"prepared": str(MANIFEST), "frozen_inputs": len(hashes)}))


def verify_manifest():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    changed = [name for name, digest in manifest["input_hashes"].items()
               if not (REPO / name).is_file() or sha256(REPO / name) != digest]
    if changed:
        raise RuntimeError(f"Frozen input changed: {changed}")
    return manifest


def train():
    manifest = verify_manifest()
    if OUTPUT.exists():
        raise FileExistsError("Refuse to overwrite or silently repeat this one-update audit")
    OUTPUT.mkdir(parents=True)
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[key] = "1"
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
    project_pythonpath = os.pathsep.join((str(PACKAGE.parent), str(REPO)))
    if os.environ.get("PYTHONPATH"):
        project_pythonpath += os.pathsep + os.environ["PYTHONPATH"]
    os.environ["PYTHONPATH"] = project_pythonpath
    sys.path.insert(0, str(PACKAGE.parent))
    sys.path.insert(0, str(HERE))
    from unified_MLP_39 import train_ppo_mlp as driver, training_runtime
    from unified_MLP_39 import learner_state_resume, reward_resume
    import learner_probe
    # A dedicated local Ray cluster cannot attach to the separate fresh run.
    driver._load_training_stack()
    original_ray_init = driver.ray.init
    # macOS's default per-user temp path can exceed Unix socket path limits.
    ray_temp = tempfile.mkdtemp(prefix="cmc_pa_", dir="/tmp" if os.name != "nt" else None)

    def isolated_ray_init(*args, **kwargs):
        kwargs.update(address="local", _temp_dir=ray_temp)
        kwargs.setdefault("runtime_env", {}).setdefault("env_vars", {})["PYTHONPATH"] = project_pythonpath
        return original_ray_init(*args, **kwargs)

    driver.ray.init = isolated_ray_init
    original_build_config = driver.build_config

    def build_config(args, reward_overrides=None):
        config = original_build_config(args, reward_overrides)
        names = ("normalize_actions", "clip_actions", "batch_mode", "rollout_fragment_length",
                 "train_batch_size", "minibatch_size", "num_epochs", "gamma", "lambda_",
                 "clip_param", "kl_coeff", "kl_target", "vf_clip_param", "vf_loss_coeff",
                 "shuffle_batch_per_epoch", "num_env_runners", "seed")
        write_json(OUTPUT / "effective_rllib_configuration.json",
                   {name: getattr(config, name, None) for name in names})
        return config

    driver.build_config = build_config
    original_before = training_runtime.before_iteration
    original_after = training_runtime.after_iteration
    calls = {"before": 0, "after": 0}

    def before(algo, args, owner):
        calls["before"] += 1
        if calls["before"] != 1:
            raise RuntimeError("Audit permits only one PPO iteration")
        result = original_before(algo, args, owner)
        if result["phase"] != "imitation":
            raise RuntimeError("Unexpected training phase")
        live_rewards = reward_resume.audit_live_rewards(algo, args, algo._unified_training_state["reward_config"])
        auxiliary = driver._learner_call_results(algo, learner_state_resume.capture_on_learner)
        write_json(OUTPUT / "pre_update_audit.json", {"phase": result, "live_rewards": live_rewards,
                   "learner_auxiliary": auxiliary, "ray_temp": ray_temp,
                   "source_input_hash_count": len(manifest["input_hashes"])})
        driver._learner_call_results(algo, learner_probe.attach_probe, directory=str(OUTPUT / "ppo_audit"))
        return result

    def after(algo, iteration, phase_before, args, owner):
        calls["after"] += 1
        driver._learner_call_results(algo, learner_probe.detach_probe)
        return original_after(algo, iteration, phase_before, args, owner)

    training_runtime.before_iteration = before
    training_runtime.after_iteration = after
    sys.argv = [str(Path(driver.__file__)), "--config", str(CONFIG), "--worker-process"]
    try:
        driver.main()
        if calls != {"before": 1, "after": 1}:
            raise RuntimeError(f"Audit iteration counts incorrect: {calls}")
        verify_manifest()
        write_json(OUTPUT / "audit_completed.json", {"completed": True, "calls": calls,
                   "input_hashes_unchanged": True, "ray_temp": ray_temp})
    finally:
        training_runtime.before_iteration = original_before
        training_runtime.after_iteration = original_after
        driver.ray.init = original_ray_init
        driver.build_config = original_build_config


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "train", "verify"))
    args = parser.parse_args()
    {"prepare": prepare, "train": train, "verify": verify_manifest}[args.command]()
