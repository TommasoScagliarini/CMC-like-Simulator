"""Materialize the isolated 39D package only when phase 5 explicitly invokes it.

This module has no import-time side effects and never launches training. The
source package is read-only. Every textual patch fails closed on source drift.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

BUNDLE = Path(__file__).resolve().parent
SOURCE = BUNDLE.parents[1]
PACKAGE = "unified_MLP_39"


def _replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError(f"variant39 source drift: expected {count} matches for {old[:100]!r}")
    return text.replace(old, new)


def _transform(name, text):
    text = text.replace("unified_MLP", PACKAGE)
    if name == "actor_transition.py":
        text = text.replace('"unified_mlp_v1"', '"unified_mlp39_v1"')
        text = _replace(text, "import numpy as np", "import numpy as np\nfrom unified_MLP_39.target_availability import TARGET_CONTRACT, TARGET_FEATURES")
        text = _replace(text, '    "observation_feature_names", "action_low", "action_high",', '    "observation_feature_names", "action_low", "action_high", "target_channels",')
        text = _replace(text, '        "schema_version": SCHEMA_VERSION,', '        "schema_version": SCHEMA_VERSION,\n        "target_channels": copy.deepcopy(TARGET_CONTRACT),')
        text = _replace(text, '    if len(actor) != 35 or full[:35] != actor:', '    if len(actor) != 39 or len(full) != 88 or full[:39] != actor:')
        text = _replace(text, '    low, high = (np.asarray(contract[k], dtype=float)', '    if actor[35:39] != list(TARGET_FEATURES) or contract["target_channels"] != TARGET_CONTRACT:\n        raise ValueError("variant39 target order/normalization/mask contract mismatch")\n    low, high = (np.asarray(contract[k], dtype=float)')
        text = text.replace("(256, 35)", "(256, 39)").replace("requires 35 actor features", "requires 39 actor features and 88 total features")
    elif name == "env_factory.py":
        text = _replace(text, 'ENV_NAME = "cmc_unified_traj_env"', 'ENV_NAME = "cmc_unified39_traj_env"')
        text = _replace(text, '    env = UnifiedProsthesisEnv(cfg)', '''    env = UnifiedProsthesisEnv(
        cfg, availability_probability=raw.get("target_availability_probability", 0.0),
        mask_seed=raw.get("target_mask_seed", 123), reward_mode=reward_cfg.reward_mode,
    )''')
    elif name == "training_config.py":
        text = _replace(text, 'SECTION_MAP: dict[str, dict[str, Callable[[Any], Any]]] = {', '''SECTION_MAP: dict[str, dict[str, Callable[[Any], Any]]] = {
    "target_availability": {
        "target_availability_probability": float,
        "target_curriculum_probabilities": _list_float,
        "target_curriculum_block_iterations": int,
        "target_mask_seed": int,
    },''')
    elif name in ("train_ppo_mlp.py", "rollout_eval.py"):
        text = _replace(text, '    valid_dests = {action.dest for action in p._actions}', '''    from unified_MLP_39.target_availability import add_cli_arguments
    add_cli_arguments(p)
    valid_dests = {action.dest for action in p._actions}''')
        text = _replace(text, '    env_config = {\n        "setup_xml_path": args.setup_xml,', '''    from unified_MLP_39.target_availability import env_options
    env_config = {
        **env_options(args, evaluation=''' + str(name == "rollout_eval.py") + '''),
        "setup_xml_path": args.setup_xml,''')
        if name == "rollout_eval.py":
            text = _replace(text, '            policy_mean = None\n            action_noise = None', '            policy_mean = None\n            policy_std = None\n            action_noise = None')
            text = _replace(text, '                        "exploration_action_noise": (', '''                        "policy_action_std": (
                            np.asarray(policy_std, dtype=float).tolist()
                            if policy_std is not None else None
                        ),
                        "target_availability": dict(info.get("target_availability", {})),
                        "exploration_action_noise": (''')
    elif name == "asymmetric_rl_module.py":
        text = _replace(text, '    def _actor_in(self, batch: Dict[str, Any]) -> "torch.Tensor":\n        obs = batch[Columns.OBS]', '''    def _masked_observation(self, batch):
        obs = batch[Columns.OBS]
        # Persisted phase buffer is shared by learner, sampler and checkpoint.
        # Defense in depth: ex-novo/warmup cannot reopen target channels even
        # if the caller bypasses the collector and supplies unmasked values.
        if int(self._unified_training_phase.item()) != 0:
            obs = obs.clone()
            obs[..., 35:39] = 0.0
        return obs

    def _actor_in(self, batch: Dict[str, Any]) -> "torch.Tensor":
        obs = self._masked_observation(batch)''')
        text = _replace(text, '        return batch[Columns.OBS][..., : self._n_full]', '        return self._masked_observation(batch)[..., : self._n_full]')
    elif name == "training_runtime.py":
        text = text.replace('"unified_mlp_v1"', '"unified_mlp39_v1"')
        text = _replace(text, "from unified_MLP_39 import actor_transition", "from unified_MLP_39 import actor_transition, target_availability")
        text = _replace(text, '    write_json(Path(output_dir) / CONTRACT_FILE, contract)', '''    target_availability.initialize_runtime(algo, args, resume_path)
    write_json(Path(output_dir) / CONTRACT_FILE, contract)''')
        text = _replace(text, 'def before_iteration(algo, args, driver):\n    state = algo._unified_training_state', '''def before_iteration(algo, args, driver):
    availability_audit = target_availability.before_iteration(algo, args)
    state = algo._unified_training_state''')
        text = _replace(text, '        "phase": phase,\n        "actor_before":', '        "phase": phase,\n        "target_availability": availability_audit,\n        "actor_before":')
        text = _replace(text, '    return audit', '    audit["target_availability_batch"] = target_availability.after_iteration(algo, iteration, args)\n    return audit')
    return text


def materialize_variant(destination=None, *, source=None):
    source = Path(source or SOURCE).resolve()
    destination = Path(destination or source.parent / PACKAGE).resolve()
    if destination.name != PACKAGE:
        raise ValueError(f"destination must be named {PACKAGE} (checkpoint import namespace)")
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite existing variant package: {destination}")
    if source == destination or source in destination.parents:
        raise ValueError("variant destination must be outside the preserved V1 package")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".variant39-build-", dir=destination.parent))
    hashes = {}
    try:
        # Runtime Python modules and configs only. Runs/checkpoints, tests,
        # caches, V1 documentation and this deferred bundle are never copied.
        files = sorted(source.glob("*.py")) + sorted((source / "configs").glob("*.yaml"))
        for path in files:
            relative = path.relative_to(source)
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            raw = path.read_bytes()
            hashes[str(relative)] = hashlib.sha256(raw).hexdigest()
            text = _transform(path.name, raw.decode("utf-8"))
            target.write_text(text, encoding="utf-8")
        (staging / "env.py").rename(staging / "env_v1.py")
        for payload, target in (("env39.py", "env.py"), ("target_availability.py", "target_availability.py")):
            shutil.copyfile(BUNDLE / payload, staging / target)
        receipt = {
            "schema": "deferred_feasibility39_materialization_v1", "namespace": PACKAGE,
            "source": str(source), "source_sha256": hashes,
            "payload_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(BUNDLE.glob("*.py"))},
            "generated_sha256": {str(p.relative_to(staging)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(staging.rglob("*")) if p.is_file()},
            "training_launched": False,
            "resume_limitation": "PPO/optimizer/phase/curriculum/mask RNG restored; physical episode restarts",
        }
        (staging / "variant39_build_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        staging.rename(destination)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=None)
    args = parser.parse_args()
    print(materialize_variant(args.destination))


if __name__ == "__main__":
    main()
