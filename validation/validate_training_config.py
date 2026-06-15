"""Smoke test for the training_cfg.yaml loader + train/rollout argparse wiring.

Light-weight: imports only the entrypoint modules (no OpenSim/Ray stack is loaded
at import or parse time) and exercises the config loader, default seeding, CLI
override precedence, the deprecated alias, nargs GRF sides, and the resolved
snapshot round-trip used by rollout auto-match.
"""
import sys
import tempfile
from pathlib import Path

_BASELINE = Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(_BASELINE))

import training_config  # noqa: E402
import train_ppo_mlp  # noqa: E402
import rollout_eval  # noqa: E402


def _parse(module, argv):
    saved = sys.argv
    sys.argv = ["prog", *argv]
    try:
        return module.parse_args()
    finally:
        sys.argv = saved


failures = []


def check(name, cond):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}")
    if not cond:
        failures.append(name)


# 1. Loader: default YAML -> flat defaults + reward.
cfg = training_config.load(training_config.DEFAULT_CONFIG_PATH)
flat, reward = training_config.to_argparse_defaults(cfg)
check("reward section == {reward_mode: ex_novo}", reward == {"reward_mode": "ex_novo"})
check("flat has num/dim hidden layers", flat.get("num_hidden_layers") == 2 and flat.get("dim_hidden_layers") == 256)
check("flat lr is float 0.0001", isinstance(flat.get("lr"), float) and abs(flat["lr"] - 1e-4) < 1e-12)
check("flat grf applied side == ['left']", flat.get("online_grf_applied_side") == ["left"])
check("no action_mode in shipped yaml flat", "action_mode" not in flat)

# 2. Train parse_args: YAML seeds defaults.
a = _parse(train_ppo_mlp, [])
check("train default num_hidden_layers=2 (yaml)", a.num_hidden_layers == 2)
check("train default dim_hidden_layers=256 (yaml)", a.dim_hidden_layers == 256)
check("train default train_batch_size=4096 (yaml)", a.train_batch_size == 4096)
check("train default iterations=40 (yaml)", a.iterations == 40)
check("train default grf_mode online_sensor (yaml)", a.grf_mode == "online_sensor")
check("train default asymmetric False (yaml)", a.asymmetric_actor_critic is False)
check("train _cfg_reward == {reward_mode: ex_novo}", a._cfg_reward == {"reward_mode": "ex_novo"})
hiddens = [int(a.dim_hidden_layers)] * int(a.num_hidden_layers)
check("train hiddens expand to [256, 256]", hiddens == [256, 256])

# 3. CLI override wins over YAML.
b = _parse(train_ppo_mlp, ["--train-batch-size", "99", "--dim-hidden-layers", "128", "--lr", "5e-4"])
check("CLI override train_batch_size=99", b.train_batch_size == 99)
check("CLI override dim_hidden_layers=128", b.dim_hidden_layers == 128)
check("CLI override lr=5e-4", abs(b.lr - 5e-4) < 1e-12)

# 4. Asymmetric flag + deprecated alias.
c = _parse(train_ppo_mlp, ["--asymmetric-actor-critic"])
check("new flag sets asymmetric True", c.asymmetric_actor_critic is True)
d = _parse(train_ppo_mlp, ["--critic-privileged-observation"])
check("deprecated alias sets asymmetric True", d.asymmetric_actor_critic is True)

# 5. GRF side nargs override (replace, not append).
e = _parse(train_ppo_mlp, ["--online-grf-applied-side", "left", "right"])
check("nargs GRF sides == ['left','right']", e.online_grf_applied_side == ["left", "right"])

# 5b. Deprecated --fcnet-hiddens alias still parses (protects older commands and
# in-flight supervisor restarts from an argparse 'unrecognized arguments' crash).
f2 = _parse(train_ppo_mlp, ["--fcnet-hiddens", "256", "256"])
check("deprecated --fcnet-hiddens parses to [256,256]", getattr(f2, "fcnet_hiddens", None) == [256, 256])

# 6. Rollout parse_args: built-in defaults when no snapshot (--no-auto-config).
r = _parse(rollout_eval, ["--checkpoint", "DUMMY_DOES_NOT_EXIST", "--no-auto-config"])
check("rollout action_mode default absolute", r.action_mode == "absolute")
check("rollout asymmetric default False", r.asymmetric_actor_critic is False)
check("rollout has _cfg_reward attr", hasattr(r, "_cfg_reward"))

# 7. Snapshot round-trip (dump_resolved -> load_resolved_for_checkpoint).
with tempfile.TemporaryDirectory() as tmp:
    run_dir = Path(tmp) / "run"
    run_dir.mkdir()
    snap_path = run_dir / training_config.RESOLVED_CONFIG_NAME
    training_config.dump_resolved(a, {"reward_mode": "imitation"}, snap_path)
    check("snapshot file written", snap_path.is_file())
    loaded = training_config.load_resolved_for_checkpoint(run_dir / "rl_module_best")
    check("snapshot loaded from checkpoint dir", isinstance(loaded, dict))
    sflat, sreward = training_config.to_argparse_defaults(loaded)
    check("snapshot carries action_mode (diagnostic round-trip)", sflat.get("action_mode") == "absolute")
    check("snapshot reward_mode imitation", sreward.get("reward_mode") == "imitation")
    check("snapshot omits run-identity output_dir", "output_dir" not in sflat)

print()
if failures:
    print(f"FAILURES ({len(failures)}): {failures}")
    sys.exit(1)
print("ALL SMOKE CHECKS PASSED")
