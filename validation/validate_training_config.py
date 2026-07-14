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
check("default reward_mode imitation", reward.get("reward_mode") == "imitation")
check("default reward carries penetration weight", reward.get("grf_penetration_weight") == 0.5)
check("flat has num/dim hidden layers", flat.get("num_hidden_layers") == 2 and flat.get("dim_hidden_layers") == 256)
check("flat lr is float 0.0001", isinstance(flat.get("lr"), float) and abs(flat["lr"] - 1e-4) < 1e-12)
check("flat grf applied side == ['left']", flat.get("online_grf_applied_side") == ["left"])
check("no action_mode in shipped yaml flat", "action_mode" not in flat)

# 1b. Explicit ex-novo YAML -> task-based reward defaults.
exnovo_cfg = training_config.load(_BASELINE / "training_exnovo_cfg.yaml")
exnovo_flat, exnovo_reward = training_config.to_argparse_defaults(exnovo_cfg)
check("exnovo reward_mode ex_novo", exnovo_reward.get("reward_mode") == "ex_novo")
check("exnovo asymmetric actor-critic on", exnovo_flat.get("asymmetric_actor_critic") is True)
check("exnovo gait clock disabled", exnovo_flat.get("gait_clock_enable") is False)
check("exnovo deployable minimal observation off", exnovo_flat.get("deployable_minimal_observation") is False)
check("exnovo reference state observation on", exnovo_flat.get("include_reference_state_observation") is True)
check("exnovo deployable controller state is actor-visible", exnovo_flat.get("include_controller_state_observation") is True)
check("exnovo derived controller diagnostics are critic-only", exnovo_flat.get("include_controller_diagnostic_observation") is False)
check("exnovo task contact load enabled", exnovo_reward.get("blend_contact_load", 0.0) > 0.0)
check(
    "exnovo contact support ledger enabled",
    exnovo_reward.get("blend_contact_support_to") == 0.30
    and exnovo_reward.get("contact_support_failure_clawback_weight") == 1.0,
)
check(
    "exnovo penetration thresholds match frozen full-episode baseline",
    exnovo_flat.get("grf_penetration_penalty_threshold_m") == 0.015
    and exnovo_flat.get("grf_penetration_termination_m") == 0.025,
)
check("exnovo task phase regularity enabled", exnovo_reward.get("blend_phase_regular", 0.0) > 0.0)
check("exnovo command tracking reward disabled", exnovo_reward.get("blend_tracking") == 0.0)
check("exnovo online-GRF slip reward disabled", exnovo_reward.get("grf_slip_weight") == 0.0)
check(
    "exnovo penetration reward is weak-only",
    exnovo_reward.get("grf_penetration_weight") == 0.05,
)
check("exnovo FSM progress reward enabled", exnovo_reward.get("blend_phase_event_progress", 0.0) > 0.0)
check("exnovo landing window reward enabled", exnovo_reward.get("blend_landing_window_contact", 0.0) > 0.0)
check("exnovo FSM min durations configured", exnovo_reward.get("phase_min_stance_duration_s") == 0.30 and exnovo_reward.get("phase_min_swing_duration_s") == 0.25)
check("exnovo FSM landing window configured", exnovo_reward.get("phase_landing_window_start_s") == 0.35 and exnovo_reward.get("phase_landing_window_end_s") == 0.85)
check("exnovo FSM ledger credits configured", exnovo_reward.get("phase_hs_event_credit") == 0.10 and exnovo_reward.get("phase_to_event_credit") == 0.20 and exnovo_reward.get("phase_cycle_complete_bonus") == 0.70)
check("exnovo FSM clawback enabled", exnovo_reward.get("phase_failure_extra_penalty") == 0.05 and exnovo_reward.get("phase_clawback_penalty_weight") == 1.00)
check("exnovo FSM anti-fake-cycle gates configured", exnovo_reward.get("phase_min_stance_contact_fraction") == 0.20 and exnovo_reward.get("phase_min_stance_load_bw_s") == 0.04 and exnovo_reward.get("phase_min_cycle_knee_excursion_rad") == 0.12)
check("exnovo phase timeout post-clip penalty enabled", exnovo_reward.get("phase_timeout_penalty_weight") == 0.50)
check("exnovo hard stance/swing timeouts configured", exnovo_reward.get("phase_stance_hard_timeout_s") == 2.20 and exnovo_reward.get("phase_swing_hard_timeout_s") == 1.10)
check("exnovo action clip penalty enabled", exnovo_reward.get("policy_action_clip_weight") == 0.25)
check("exnovo tightened knee range configured", exnovo_reward.get("oob_q_min") == [-1.25, -0.60] and exnovo_reward.get("prosthetic_joint_q_min") == [-1.30, -0.60] and exnovo_reward.get("prosthetic_joint_range_weight") == 2.0)
check("exnovo reserve/pelvis diagnostic weights zero", exnovo_reward.get("reserve_residual_weight") == 0.0 and exnovo_reward.get("pelvis_height_weight") == 0.0)
check(
    "exnovo morphology profile configured",
    exnovo_reward.get("morphology_profile")
    == "morphology_profiles/ab06_prosthetic_mean_std_corridor.json",
)
check(
    "exnovo morphology diagnostic weight zero",
    exnovo_reward.get("morphology_weight") == 0.0,
)
check(
    "exnovo morphology K configured",
    exnovo_reward.get("morphology_std_multiplier_knee") == 1.6
    and exnovo_reward.get("morphology_std_multiplier_ankle") == 0.6,
)
check(
    "exnovo morphology margins configured",
    exnovo_reward.get("morphology_margin_knee_deg") == 7.5
    and exnovo_reward.get("morphology_margin_ankle_deg") == 7.5,
)

# 2. Train parse_args: YAML seeds defaults.
a = _parse(train_ppo_mlp, [])
check("train default num_hidden_layers=2 (yaml)", a.num_hidden_layers == 2)
check("train default dim_hidden_layers=256 (yaml)", a.dim_hidden_layers == 256)
check("train default train_batch_size=4096 (yaml)", a.train_batch_size == 4096)
check("train default iterations=40 (yaml)", a.iterations == 40)
check("train default grf_mode online_sensor (yaml)", a.grf_mode == "online_sensor")
check("train default asymmetric True (yaml)", a.asymmetric_actor_critic is True)
check("train _cfg_reward reward_mode imitation", a._cfg_reward.get("reward_mode") == "imitation")
hiddens = [int(a.dim_hidden_layers)] * int(a.num_hidden_layers)
check("train hiddens expand to [256, 256]", hiddens == [256, 256])
check("train default start choices empty", a.episode_start_offset_choices_s == [])
check("train default KL target 0.01", a.kl_target == 0.01)

# 2b. Train parse_args with ex-novo config: explicit config seeds ex-novo defaults.
a_ex = _parse(train_ppo_mlp, ["--config", str(_BASELINE / "training_exnovo_cfg.yaml")])
check("train exnovo config reward_mode ex_novo", a_ex._cfg_reward.get("reward_mode") == "ex_novo")
check("train exnovo config asymmetric True", a_ex.asymmetric_actor_critic is True)
check("train exnovo config gait clock disabled", a_ex.gait_clock_enable is False)
check("train exnovo config deployable minimal obs off", a_ex.deployable_minimal_observation is False)
check("train exnovo config reference state obs on", a_ex.include_reference_state_observation is True)
check("train exnovo config controller state actor-visible", a_ex.include_controller_state_observation is True)
check("train exnovo config controller diagnostics critic-only", a_ex.include_controller_diagnostic_observation is False)
check("train exnovo config profile correct magnitude", "grf_correct_magnitude" in a_ex.online_grf_profile)
check("train exnovo config detector HS/TO profile", "grf_detector_HS-TO" in a_ex.online_grf_detector_profile)
check(
    "train exnovo config carries penetration thresholds",
    a_ex.grf_penetration_penalty_threshold_m == 0.015
    and a_ex.grf_penetration_termination_m == 0.025,
)

# 3. CLI override wins over YAML.
b = _parse(train_ppo_mlp, ["--train-batch-size", "99", "--dim-hidden-layers", "128", "--lr", "5e-4"])
check("CLI override train_batch_size=99", b.train_batch_size == 99)
check("CLI override dim_hidden_layers=128", b.dim_hidden_layers == 128)
check("CLI override lr=5e-4", abs(b.lr - 5e-4) < 1e-12)

b_start = _parse(
    train_ppo_mlp,
    [
        "--episode-start-offset-choices-s",
        "1.75",
        "1.95",
        "2.15",
        "--kl-coeff",
        "1.0",
        "--kl-target",
        "0.005",
    ],
)
check(
    "CLI multi-start offsets preserve order",
    b_start.episode_start_offset_choices_s == [1.75, 1.95, 2.15],
)
check("CLI KL controls parse", b_start.kl_coeff == 1.0 and b_start.kl_target == 0.005)

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

# 5c. A custom warm-start source must not inherit the official source config.
f3 = _parse(
    train_ppo_mlp,
    ["--warm-start", "--warm-start-source", "CUSTOM_RL_MODULE"],
)
check("custom warm-start keeps adjacent config resolution", f3.warm_start_source_config is None)

# 6. Rollout parse_args: built-in defaults when no snapshot (--no-auto-config).
# A dummy checkpoint is OK for argparse only, but provide output_dir so rollout
# does not try to resolve the nonexistent checkpoint just to derive a run name.
r = _parse(
    rollout_eval,
    [
        "--checkpoint",
        "DUMMY_DOES_NOT_EXIST",
        "--no-auto-config",
        "--output-dir",
        str(Path(tempfile.gettempdir()) / "dummy_rollout_parse"),
    ],
)
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

    training_config.dump_resolved(
        b_start, {"reward_mode": "ex_novo"}, snap_path
    )
    multistart_loaded = training_config.load(snap_path)
    multistart_flat, _ = training_config.to_argparse_defaults(multistart_loaded)
    check(
        "snapshot round-trips multi-start and KL controls",
        multistart_flat.get("episode_start_offset_choices_s")
        == [1.75, 1.95, 2.15]
        and multistart_flat.get("kl_coeff") == 1.0
        and multistart_flat.get("kl_target") == 0.005,
    )

print()
if failures:
    print(f"FAILURES ({len(failures)}): {failures}")
    sys.exit(1)
print("ALL SMOKE CHECKS PASSED")
