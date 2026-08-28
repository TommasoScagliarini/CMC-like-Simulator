"""V26C J1 - prescribed-teacher collection under the CURRENT EGRF/FSM v3 runtime.

Replays the July BASE stage semantics on the current runtime. It does NOT fit, does NOT promote,
does NOT touch production, and constructs NO environment until an explicit execution token is
supplied.

WHY A NEW COLLECTOR EXISTS AT ALL
    `target_domain_imitation.build_target_env_config` forwards none of the twelve detector/FSM v3
    keys and no morphology or corridor block. Calling it - even while reading the pinned config -
    would silently build a NON-v3 teacher. This module therefore assembles the FULL env_config
    itself, semantically equivalent to `rollout_eval.run` / `train_ppo_mlp.build_config`, and
    refuses to start if a single required field is missing or mismatched.

TWO CONTRACT IDS, NEVER ONE
    `event_contract_id` (legacy_events_v1) and `binary_phase_event_contract_id`
    (binary_point_v25+heel_qualified_fsm_v2) are DISTINCT env keys with DIFFERENT values. Both
    appear side by side in the production builders. They must never be merged or swapped.

THE ONLY CONFIG -> ENV RENAME
    config `binary_phase_detector_profile` -> env `binary_phase_detector_profile_file`.
    `binary_phase_invalid_event_policy` and both contract ids already carry their final names.

PREFLIGHT IS INERT
    `--preflight` performs ZERO environment construction, reset or step. The heavy imports live
    inside `collect()` only, and the test proves the inertness with a sentinel.

Parent, exclusively: runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter.
July artefacts are methodological evidence only, never operational parents, datasets or labels.
LOTO / LOCO / B1R1 / B1R2 remain future, non-operational TODOs.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
TG = REPO / "Trajectory Generator"
BASELINE = TG / "baseline_MLP"


class J1Error(RuntimeError):
    pass


STAGE = "V26C-J1-TEACHER-COLLECT"
RECEIPT_NAME = "v26c_j1_collection_receipt.json"

# ------------------------------------------------------------------ pinned runtime config -------
PINNED_CONFIG = (TG / "runs" / "training" / "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter"
                 / "training_cfg.resolved.yaml")
PIN_RUNTIME_CONFIG_SHA = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"

# ------------------------------------------------------------------ exclusive operative parent --
PARENT_DIR = (TG / "runs" / "training"
              / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
PIN_PARENT_STATE_SHA = "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
FORBIDDEN_PARENT_MARKERS = ("controller_memory_ablation", "target_domain_", "2026-07-",
                            "markov35", "zero_iter_port")

# ------------------------------------------------------------------ observation contract --------
ACTOR_WIDTH = 35
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
MASKED_COLUMNS = tuple(sorted(CLOCK_COLUMNS + CONTROLLER_COLUMNS))

# ------------------------------------------------------------------ v3 contract, fail-closed ----
# The twelve v3 env keys, each mapped to the CONFIG key it must equal. The expected value is
# DERIVED from the immutable pinned YAML, never hardcoded, so "present and non-empty" can never
# stand in for "exactly right". `binary_phase_detector_profile_file` is the sole rename and its
# expectation is the resolved absolute path.
V3_ENV_TO_CONFIG: dict[str, str] = {
    "binary_phase_detector_profile_file": "binary_phase_detector_profile",   # the sole rename
    "phase_fsm_input_mode": "phase_fsm_input_mode",
    "phase_sensor_on_threshold_n": "phase_sensor_on_threshold_n",
    "phase_sensor_off_threshold_n": "phase_sensor_off_threshold_n",
    "phase_sensor_dwell_s": "phase_sensor_dwell_s",
    "detector_sample_dt_s": "detector_sample_dt_s",
    "event_contract_id": "event_contract_id",
    "binary_phase_fsm_mode": "binary_phase_fsm_mode",
    "binary_phase_invalid_event_policy": "binary_phase_invalid_event_policy",
    "binary_phase_actor_fsm_version": "binary_phase_actor_fsm_version",
    "binary_phase_debounce_s": "binary_phase_debounce_s",
    "binary_phase_event_contract_id": "binary_phase_event_contract_id",
}
V3_PATH_VALUED = ("binary_phase_detector_profile_file",)
# an independent literal cross-check of the values the architect pinned; the YAML remains the
# source of truth and any disagreement is a hard failure.
V3_LITERAL_CROSSCHECK: dict[str, Any] = {
    "phase_fsm_input_mode": "legacy_events",
    "phase_sensor_on_threshold_n": 5.0,
    "phase_sensor_off_threshold_n": 2.0,
    "phase_sensor_dwell_s": 0.03,
    "detector_sample_dt_s": 0.001,
    "event_contract_id": "legacy_events_v1",
    "binary_phase_fsm_mode": "binary_active",
    "binary_phase_invalid_event_policy": "reject_continue",
    "binary_phase_actor_fsm_version": "v3",
    "binary_phase_debounce_s": 0.005,
    "binary_phase_event_contract_id": "binary_point_v25+heel_qualified_fsm_v2",
}
CONFIG_TO_ENV_RENAME = {"binary_phase_detector_profile": "binary_phase_detector_profile_file"}

# Core env fields assembled from the config, each mapped to (section, config key). Verified
# value-for-value so a corrupted field cannot pass.
CORE_ENV_TO_CONFIG: dict[str, tuple[str, str]] = {
    "segment_duration": ("simulation", "segment_duration"),
    "episode_duration": ("simulation", "episode_duration"),
    "episode_start_offset_s": ("simulation", "episode_start_offset_s"),
    "policy_knots": ("simulation", "policy_knots"),
    "action_mode": ("simulation", "action_mode"),
    "max_delta_rad": ("simulation", "max_delta_rad"),
    "pros_ref_model": ("simulation", "pros_ref_model"),
    "pros_ref_lpf_cutoff_hz": ("simulation", "pros_ref_cutoff_hz"),
    "enable_pros_ref_governor": ("simulation", "pros_ref_governor"),
    "gait_clock_enable": ("simulation", "gait_clock_enable"),
    "actor_cyclic_phase_only": ("simulation", "actor_cyclic_phase_only"),
    "include_reference_state_observation": ("simulation", "include_reference_state_observation"),
    "include_controller_state_observation": ("simulation", "include_controller_state_observation"),
    "include_controller_diagnostic_observation":
        ("simulation", "include_controller_diagnostic_observation"),
    "deployable_minimal_observation": ("simulation", "deployable_minimal_observation"),
    "imitation_initialize_to_target": ("simulation", "imitation_initialize_to_target"),
    "reward_reference_range_floor": ("simulation", "reward_reference_range_floor"),
    "reward_reference_velocity_range_floor":
        ("simulation", "reward_reference_velocity_range_floor"),
    "step_wall_timeout_s": ("simulation", "step_wall_timeout_s"),
    "grf_penetration_penalty_threshold_m": ("simulation", "grf_penetration_penalty_threshold_m"),
    "grf_penetration_termination_m": ("simulation", "grf_penetration_termination_m"),
    "grf_mode": ("grf", "grf_mode"),
    "include_online_grf_observation": ("grf", "online_grf_observation"),
}
# fields whose value is fixed by this protocol rather than read from the config
PROTOCOL_FIXED_ENV: dict[str, Any] = {
    "random_init": False,
    "rebuild_model_on_reset": False,      # as in training and in the historical collector
    "fail_fast": True,
    "critic_privileged_observation": True,
}

# start, guards and timeouts: an independent literal cross-check on top of the YAML-derived
# expectation, so a silent config drift cannot pass unnoticed.
RUNTIME_LITERAL_CROSSCHECK: dict[str, Any] = {
    "episode_start_offset_s": 1.956870983805102,
    "episode_duration": 5.0,
    "segment_duration": 0.01,
    "policy_knots": 1,
    "action_mode": "absolute",
    "grf_mode": "online_sensor",
    "grf_penetration_penalty_threshold_m": 0.020,
    "grf_penetration_termination_m": 0.028,
    "step_wall_timeout_s": 60.0,
    "gait_clock_enable": False,
    "actor_cyclic_phase_only": True,
    "include_controller_state_observation": True,
}

# The reward block is verified in FULL against the pinned mapping, key by key and value by value.
# Spot checks are insufficient: a corrupted key outside the sample would pass unnoticed.
EXPECTED_REWARD_KEY_COUNT = 125
MORPHOLOGY_KEY_PREFIX = "morphology"

# ------------------------------------------------------------------ July collection semantics ---
# Recovered verbatim from target_domain_imitation_no_controller_memory_2026-07-13. NOT defaults.
COLLECTION_SEMANTICS: dict[str, Any] = {
    "seed": 123,
    "teacher_lookahead_s": 0.0,
    "action_noise_sigma": [0.0, 0.0],
    "action_noise_hold_steps": 1,
    "action_noise_hold_duration_s": 0.01,
}
EXPECTED_STEPS = 500
LABEL_SOURCE = ("prescribed_teacher_action on states visited by that same prescribed teacher; "
                "no other label source is permitted")

# ------------------------------------------------------------------ J1 binding gate --------------
# COMMON gate only: integrity, runtime, safety, contract. Kinematic quality is NOT a J1 gate.
J1_GATE: dict[str, Any] = {
    "steps_required": 500,
    "end_reason": "episode_time_limit",
    "valid_cycles_min": 2,
    "phase_timeout_stance_max": 0,
    "phase_timeout_swing_max": 0,
    "morphology_causal_contract_failure_max": 0,
    "hs_cancelled_count_max": 0,
    "resync_count_max": 1,
    "max_penetration_m_max": 0.020,
}
J1_GATE_EXCLUDES_KINEMATICS = ("ankle_min_rad", "ankle_amplitude_min_rad",
                               "knee_amplitude_min_rad", "knee_strictly_flexed")
DIAGNOSTIC_NOT_BINDING = ("action_clipped_steps",)

OUT_ROOT = HERE / "j1_runs"


def _sha_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ================================================================ config ==========================

def load_pinned_config() -> dict[str, Any]:
    if not PINNED_CONFIG.is_file():
        raise J1Error(f"the pinned runtime config is missing: {PINNED_CONFIG}")
    got = _sha_file(PINNED_CONFIG)
    if got != PIN_RUNTIME_CONFIG_SHA:
        raise J1Error(f"the pinned runtime config changed: {got} != {PIN_RUNTIME_CONFIG_SHA}")
    import yaml
    cfg = yaml.safe_load(PINNED_CONFIG.read_text())
    for section in ("simulation", "grf", "reward"):
        if section not in cfg:
            raise J1Error(f"the pinned config has no '{section}' section")
    return cfg


def _profile_path(value: str) -> str:
    """Profile, model and detector paths in the config are relative to the SIMULATOR ROOT.

    Verified against the repository layout: `models/`, `online_grf_profiles/` and
    `validation/binary_phase_detector_v25_geometry_runs/` all live at the root, not under
    baseline_MLP. The resolved path's existence is asserted by verify_env_config.
    """
    p = Path(str(value))
    return str(p if p.is_absolute() else (REPO / p))


def build_full_env_config(cfg: Mapping[str, Any], *, output_dir: Path | None = None
                          ) -> dict[str, Any]:
    """The FULL env_config, semantically equivalent to rollout_eval.run / train_ppo_mlp.

    Assembled here on purpose: the historical build_target_env_config forwards none of the twelve
    v3 keys and no morphology block, so reusing it would silently produce a non-v3 teacher.
    """
    sim, grf, reward = cfg["simulation"], cfg["grf"], cfg["reward"]
    env: dict[str, Any] = {
        # --- episode and integration -------------------------------------------------------
        "setup_xml_path": _profile_path(sim["setup_xml"]),
        "segment_duration": float(sim["segment_duration"]),
        "episode_duration": float(sim["episode_duration"]),
        "episode_start_offset_s": float(sim["episode_start_offset_s"]),
        "policy_knots": int(sim["policy_knots"]),
        "action_mode": str(sim["action_mode"]),
        "max_delta_rad": float(sim["max_delta_rad"]),
        # --- target slew and reference governor ---------------------------------------------
        "target_slew_rate_limit_rad_s": {
            "pros_knee_angle": float(sim["pros_knee_target_slew_rate_limit_rad_s"]),
            "pros_ankle_angle": float(sim["pros_ankle_target_slew_rate_limit_rad_s"]),
        },
        "enable_pros_ref_governor": bool(sim["pros_ref_governor"]),
        "pros_ref_model": str(sim["pros_ref_model"]),
        "pros_ref_lpf_cutoff_hz": float(sim["pros_ref_cutoff_hz"]),
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": float(sim["pros_knee_ref_velocity_limit_rad_s"]),
            "pros_ankle_angle": float(sim["pros_ankle_ref_velocity_limit_rad_s"]),
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": float(sim["pros_knee_ref_acceleration_limit_rad_s2"]),
            "pros_ankle_angle": float(sim["pros_ankle_ref_acceleration_limit_rad_s2"]),
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": float(sim["pros_knee_ref_jerk_limit_rad_s3"]),
            "pros_ankle_angle": float(sim["pros_ankle_ref_jerk_limit_rad_s3"]),
        },
        # --- observation contract -----------------------------------------------------------
        "gait_clock_enable": bool(sim["gait_clock_enable"]),
        "actor_cyclic_phase_only": bool(sim["actor_cyclic_phase_only"]),
        "include_reference_state_observation": bool(sim["include_reference_state_observation"]),
        "include_controller_state_observation": bool(sim["include_controller_state_observation"]),
        "include_controller_diagnostic_observation":
            bool(sim["include_controller_diagnostic_observation"]),
        "deployable_minimal_observation": bool(sim["deployable_minimal_observation"]),
        "imitation_initialize_to_target": bool(sim["imitation_initialize_to_target"]),
        "reward_reference_range_floor": float(sim["reward_reference_range_floor"]),
        "reward_reference_velocity_range_floor":
            float(sim["reward_reference_velocity_range_floor"]),
        "random_init": False,
        "rebuild_model_on_reset": False,
        "fail_fast": True,
        # --- GRF and the DETECTOR / FSM v3 contract -----------------------------------------
        "grf_mode": str(grf["grf_mode"]),
        "online_grf_profile_file": _profile_path(grf["online_grf_profile"]),
        "online_grf_detector_profile_file": _profile_path(grf["online_grf_detector_profile"]),
        # the ONLY config -> env rename
        "binary_phase_detector_profile_file": _profile_path(grf["binary_phase_detector_profile"]),
        "phase_fsm_input_mode": str(grf["phase_fsm_input_mode"]),
        "phase_sensor_on_threshold_n": float(grf["phase_sensor_on_threshold_n"]),
        "phase_sensor_off_threshold_n": float(grf["phase_sensor_off_threshold_n"]),
        "phase_sensor_dwell_s": float(grf["phase_sensor_dwell_s"]),
        "detector_sample_dt_s": float(grf["detector_sample_dt_s"]),
        # TWO DISTINCT contract ids - never merged, never swapped
        "event_contract_id": str(grf["event_contract_id"]),
        "binary_phase_event_contract_id": str(grf["binary_phase_event_contract_id"]),
        "binary_phase_fsm_mode": str(grf["binary_phase_fsm_mode"]),
        "binary_phase_invalid_event_policy": str(grf["binary_phase_invalid_event_policy"]),
        "binary_phase_actor_fsm_version": str(grf["binary_phase_actor_fsm_version"]),
        "binary_phase_debounce_s": float(grf["binary_phase_debounce_s"]),
        "include_online_grf_observation": bool(grf["online_grf_observation"]),
        "online_grf_applied_sides": list(grf["online_grf_applied_side"]),
        "prescribed_grf_disabled_sides": list(grf["disable_prescribed_grf_side"]),
        "critic_privileged_observation": True,
        # --- guards and timeouts -------------------------------------------------------------
        "step_wall_timeout_s": float(sim["step_wall_timeout_s"]),
        "grf_penetration_penalty_threshold_m": float(sim["grf_penetration_penalty_threshold_m"]),
        "grf_penetration_termination_m": float(sim["grf_penetration_termination_m"]),
        # --- the FULL reward block, morphology and corridor included -------------------------
        "reward": dict(reward),
        # --- recording ------------------------------------------------------------------------
        "record_outputs": bool(output_dir is not None),
        "save_outputs_on_close": bool(output_dir is not None),
    }
    if output_dir is not None:
        env["output_dir"] = str(Path(output_dir) / "sim_outputs")
        env["output_prefix"] = "j1_teacher_episode"
    return env


# ================================================================ fail-closed verification ========

def _equal(a: Any, b: Any) -> bool:
    """Exact for everything except floats, which compare with a numeric tolerance. Booleans are
    never treated as numbers, so True must not satisfy an expectation of 1."""
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-12)
    if isinstance(a, Mapping) and isinstance(b, Mapping):
        return set(a) == set(b) and all(_equal(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_equal(x, y) for x, y in zip(a, b))
    return a == b


def canonical_expectations(cfg: Mapping[str, Any]) -> dict[str, Any]:
    """The expected env_config, derived FROM the pinned YAML. Never hardcoded."""
    grf, sim = cfg["grf"], cfg["simulation"]
    exp: dict[str, Any] = {}
    for env_key, cfg_key in V3_ENV_TO_CONFIG.items():
        raw = grf[cfg_key]
        exp[env_key] = _profile_path(raw) if env_key in V3_PATH_VALUED else raw
    for env_key, (section, cfg_key) in CORE_ENV_TO_CONFIG.items():
        exp[env_key] = cfg[section][cfg_key]
    exp.update(PROTOCOL_FIXED_ENV)
    exp["setup_xml_path"] = _profile_path(sim["setup_xml"])
    exp["online_grf_profile_file"] = _profile_path(grf["online_grf_profile"])
    exp["online_grf_detector_profile_file"] = _profile_path(grf["online_grf_detector_profile"])
    exp["online_grf_applied_sides"] = list(grf["online_grf_applied_side"])
    exp["prescribed_grf_disabled_sides"] = list(grf["disable_prescribed_grf_side"])
    return exp


def verify_env_config(env: Mapping[str, Any], cfg: Mapping[str, Any] | None = None
                      ) -> dict[str, Any]:
    """Fail-closed EXACT verification against the pinned YAML. Presence is never enough."""
    cfg = cfg if cfg is not None else load_pinned_config()
    expected = canonical_expectations(cfg)
    missing: list[str] = []
    mismatched: list[dict[str, Any]] = []

    for key, want in expected.items():
        if key not in env:
            missing.append(key)
        elif not _equal(env[key], want):
            mismatched.append({"field": key, "expected": want, "observed": env[key]})

    # the v3 values and the start/guards/timeouts must ALSO match their literal cross-checks
    for label, table in (("v3", V3_LITERAL_CROSSCHECK),
                         ("runtime", RUNTIME_LITERAL_CROSSCHECK)):
        for key, want in table.items():
            if key not in env:
                missing.append(f"{key} ({label} cross-check)")
            elif not _equal(env[key], want):
                mismatched.append({"field": f"{key} ({label} cross-check)", "expected": want,
                                   "observed": env[key]})

    # the FULL reward block, key by key and value by value
    reward, want_reward = env.get("reward"), cfg["reward"]
    if not isinstance(reward, Mapping):
        missing.append("reward (block absent or not a mapping)")
    else:
        extra = sorted(set(reward) - set(want_reward))
        absent = sorted(set(want_reward) - set(reward))
        if extra or absent:
            mismatched.append({"field": "reward (key set)", "expected": f"exactly the "
                               f"{len(want_reward)} pinned keys",
                               "observed": f"extra={extra} absent={absent}"})
        for k in sorted(set(reward) & set(want_reward)):
            if not _equal(reward[k], want_reward[k]):
                mismatched.append({"field": f"reward.{k}", "expected": want_reward[k],
                                   "observed": reward[k]})
        if len(want_reward) != EXPECTED_REWARD_KEY_COUNT:
            mismatched.append({"field": "reward (pinned block size)",
                               "expected": EXPECTED_REWARD_KEY_COUNT,
                               "observed": len(want_reward)})

    # the two contract ids must remain distinct and unswapped
    a, b = env.get("event_contract_id"), env.get("binary_phase_event_contract_id")
    if a is not None and b is not None:
        if a == b:
            mismatched.append({"field": "contract ids", "expected": "two DISTINCT values",
                               "observed": f"both are {a!r}"})
        if (a == V3_LITERAL_CROSSCHECK["binary_phase_event_contract_id"]
                and b == V3_LITERAL_CROSSCHECK["event_contract_id"]):
            mismatched.append({"field": "contract ids", "expected": "unswapped",
                               "observed": "event_contract_id and binary_phase_event_contract_id "
                                           "are SWAPPED"})
    # every path-valued field must exist on disk
    for key in ("binary_phase_detector_profile_file", "online_grf_detector_profile_file",
                "online_grf_profile_file", "setup_xml_path"):
        value = env.get(key)
        if value is not None and not Path(str(value)).is_file():
            mismatched.append({"field": key, "expected": "an existing file",
                               "observed": str(value)})
    if missing or mismatched:
        raise J1Error(f"env_config FAILS the v3 contract. missing={missing} "
                      f"mismatched={mismatched}")
    morph = sorted(k for k in reward if k.startswith(MORPHOLOGY_KEY_PREFIX))
    return {"verified_fields": len(expected),
            "v3_fields": len(V3_ENV_TO_CONFIG),
            "core_fields": len(CORE_ENV_TO_CONFIG),
            "protocol_fixed_fields": len(PROTOCOL_FIXED_ENV),
            "reward_block_keys": len(reward),
            "reward_verified": "every key and value equals the pinned mapping",
            "morphology_keys_verified": len(morph),
            "contract_ids": {"event_contract_id": a, "binary_phase_event_contract_id": b,
                             "distinct": a != b},
            "config_to_env_rename_applied": dict(CONFIG_TO_ENV_RENAME),
            "status": "PASS"}


def verify_collection_semantics(sem: Mapping[str, Any]) -> dict[str, Any]:
    bad = {k: {"expected": v, "observed": sem.get(k, "<absent>")}
           for k, v in COLLECTION_SEMANTICS.items() if sem.get(k, "<absent>") != v}
    if bad:
        raise J1Error(f"the collection semantics do not match the pinned July values: {bad}")
    return {"pinned": dict(COLLECTION_SEMANTICS),
            "label_source": LABEL_SOURCE,
            "status": "PASS",
            "note": "July artefact values, not defaults"}


def verify_parent() -> dict[str, Any]:
    state = PARENT_DIR / "module_state.pkl"
    if not state.is_file():
        raise J1Error(f"the operative parent is missing: {state}")
    got = _sha_file(state)
    if got != PIN_PARENT_STATE_SHA:
        raise J1Error(f"the operative parent changed: {got} != {PIN_PARENT_STATE_SHA}")
    text = str(PARENT_DIR)
    for marker in FORBIDDEN_PARENT_MARKERS:
        if marker in text:
            raise J1Error(f"the parent path contains the forbidden marker {marker!r}: July "
                          "artefacts are methodological evidence only, never operative parents")
    return {"path": str(PARENT_DIR.relative_to(REPO)), "module_state_sha256": got,
            "role": "the EXCLUSIVE operative parent",
            "july_status": "methodological evidence only; no July checkpoint, dataset or label is "
                           "used as an operative input"}


FORBIDDEN_CALL_SITES = ("build_target_env_config", "_teacher.main(", ".main(")
ALLOWED_TEACHER_REUSE = "_teacher.prescribed_teacher_action("


def verify_no_historical_builder() -> dict[str, Any]:
    """Importing the July module for `prescribed_teacher_action` is REQUIRED read-only reuse.
    Calling its env builder or its `main` is forbidden. The distinction is enforced on call
    sites, not on the import, because the module carries both."""
    import io
    import tokenize
    src = Path(__file__).read_text()
    # Token-level scan. NAME tokens exclude every string literal and comment by construction, so
    # the prose in this module's own docstring and error messages cannot trigger a false positive.
    toks = [t for t in tokenize.generate_tokens(io.StringIO(src).readline)
            if t.type in (tokenize.NAME, tokenize.OP)]
    names = {t.string for t in toks if t.type == tokenize.NAME}
    offenders: list[str] = []
    if "build_target_env_config" in names:
        offenders.append("build_target_env_config")
    # an attribute call `.main(` is the pattern OP '.' NAME 'main' OP '('
    for i in range(len(toks) - 2):
        if (toks[i].type == tokenize.OP and toks[i].string == "."
                and toks[i + 1].type == tokenize.NAME and toks[i + 1].string == "main"
                and toks[i + 2].type == tokenize.OP and toks[i + 2].string == "("):
            offenders.append(".main(")
            break
    if offenders:
        raise J1Error(f"forbidden call sites present: {sorted(set(offenders))}. The historical env "
                      "builder forwards none of the twelve v3 keys and no morphology block, and "
                      "the July module's own entry point runs its own protocol")
    if "prescribed_teacher_action" not in names:
        raise J1Error("the collector must reuse prescribed_teacher_action read-only; no other "
                      "label source is permitted")
    return {"forbidden_call_sites_absent": list(FORBIDDEN_CALL_SITES),
            "allowed_read_only_reuse": ALLOWED_TEACHER_REUSE,
            "reason": "the historical env builder would silently produce a non-v3 teacher; its "
                      "prescribed_teacher_action, by contrast, is the required label source"}


# ================================================================ preflight (INERT) ===============

def preflight() -> dict[str, Any]:
    """Fail-closed, and provably inert: NO environment is constructed, reset or stepped here."""
    parent = verify_parent()
    no_hist = verify_no_historical_builder()
    cfg = load_pinned_config()
    env = build_full_env_config(cfg)
    contract = verify_env_config(env)
    semantics = verify_collection_semantics(COLLECTION_SEMANTICS)
    # No-clobber is PER LEAF, never "the root must stay empty forever": otherwise a single
    # completed or partial attempt would block every future preflight.
    blockers: list[str] = []
    siblings = sorted(p.name for p in OUT_ROOT.iterdir()) if OUT_ROOT.is_dir() else []
    return {
        "verdict": "GO" if not blockers else "BLOCKED",
        "stage": STAGE,
        "blockers": blockers,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False,
                  "note": "the env factory is imported inside collect() only"},
        "pinned_config": {"path": str(PINNED_CONFIG.relative_to(REPO)),
                          "sha256": PIN_RUNTIME_CONFIG_SHA},
        "parent": parent,
        "historical_builder": no_hist,
        "env_contract": contract,
        "collection_semantics": semantics,
        "expected_steps": EXPECTED_STEPS,
        "j1_gate": dict(J1_GATE),
        "j1_gate_excludes_kinematics": {
            "excluded": list(J1_GATE_EXCLUDES_KINEMATICS),
            "why": "kinematic quality qualifies the gait an ACTOR produces. J1 accepts or rejects "
                   "a DATASET; the kinematic gate binds J3 only",
        },
        "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
        "actor_contract": {"width": ACTOR_WIDTH, "masked_columns": list(MASKED_COLUMNS),
                           "note": "the same single 35D actor; no standalone 25D artefact, no "
                                   "widening; contralateral absence is intentional"},
        "env_config_sha256": _sha_obj(env),
        "no_clobber": {"scope": "PER LEAF run directory, not the root",
                       "root": str(OUT_ROOT.relative_to(REPO)),
                       "existing_sibling_runs": siblings,
                       "rule": "prior sibling runs never block the preflight; collect() refuses "
                               "only if its own fresh leaf already exists"},
        "execution_requires": f"--authorized-stage {STAGE}",
        "future_todo_non_operational": ["LOTO (B1)", "LOCO (B1R1)", "B1R2", "B1R2-A", "B1R2-B"],
        "generated_at_utc": _utc(),
    }


# ================================================================ collection ======================

def collect(*, authorized_stage: str | None, out_dir: Path | None = None,
            progress: bool = True) -> dict[str, Any]:
    """The real collection. Requires the explicit execution token. NOT run by --preflight."""
    if authorized_stage != STAGE:
        raise J1Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    pre = preflight()
    if pre["blockers"]:
        raise J1Error(f"preflight BLOCKED: {pre['blockers']}")

    # The leaf is only NAMED here. It is created after the config validates and the heavy imports
    # succeed, so a failure at those stages leaves no directory behind and poisons no future
    # preflight. Only this leaf is subject to no-clobber; sibling runs are irrelevant.
    out = Path(out_dir) if out_dir is not None else (OUT_ROOT / f"j1_{_utc().replace(':', '')}")
    if out.exists():
        raise J1Error(f"no-clobber: the leaf {out} already exists; choose a fresh --out-dir")

    cfg = load_pinned_config()
    env_config = build_full_env_config(cfg, output_dir=out)
    verify_env_config(env_config, cfg)

    # heavy imports live HERE so that preflight stays inert
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import numpy as np
    import env_factory
    import exploration_noise
    import target_domain_imitation as _teacher   # read-only reuse of prescribed_teacher_action

    out.mkdir(parents=True, exist_ok=False)
    # SimulationRunner.__init__ calls os.makedirs(cfg.output_dir), so make_cmc_env can populate
    # out/sim_outputs and only then fail. If construction raises, remove the leaf this call
    # created - and ONLY that leaf, never a sibling and never the root.
    try:
        env = env_factory.make_cmc_env(env_config)
    except BaseException:
        _remove_own_leaf(out)
        raise
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=int(COLLECTION_SEMANTICS["seed"]))
        action_dim = int(np.prod(env.action_space.shape))
        sigma = exploration_noise.broadcast_sigma(
            list(COLLECTION_SEMANTICS["action_noise_sigma"]), action_dim
        ).reshape(env.action_space.shape)
        noise = exploration_noise.HeldStandardNormal(
            np.random.default_rng(int(COLLECTION_SEMANTICS["seed"])),
            env.action_space.shape,
            int(COLLECTION_SEMANTICS["action_noise_hold_steps"]),
        )
        expected = int(math.ceil(float(base.env_cfg.episode_duration)
                                 / float(base.env_cfg.segment_duration)))
        if expected != EXPECTED_STEPS:
            raise J1Error(f"the pinned config yields {expected} steps, expected {EXPECTED_STEPS}")

        feature_names = tuple(str(n) for n in getattr(base, "actor_feature_names", ()))
        if len(feature_names) != ACTOR_WIDTH:
            raise J1Error(f"the actor feature manifest has {len(feature_names)} names, "
                          f"expected {ACTOR_WIDTH}")
        if int(getattr(base, "n_actor", -1)) != ACTOR_WIDTH:
            raise J1Error(f"base.n_actor is {getattr(base, 'n_actor', None)}, "
                          f"expected {ACTOR_WIDTH}")
        observations: list[Any] = []
        actions: list[Any] = []
        executed_actions: list[Any] = []
        action_noises: list[Any] = []
        times: list[float] = []
        trace: list[dict[str, Any]] = []
        end_reason = "unterminated"
        clipped = 0
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        for step in range(1, expected + 1):
            actor_obs = np.asarray(obs, dtype=np.float32).reshape(-1)[: base.n_actor]
            target_t = min(float(base.t) + float(base.env_cfg.segment_duration),
                           float(base._episode_end))
            teacher_action = _teacher.prescribed_teacher_action(
                base, target_t,
                lookahead_s=float(COLLECTION_SEMANTICS["teacher_lookahead_s"]),
            ).astype(np.float32)
            noise_vec = (noise.next() * sigma).astype(np.float32).reshape(-1)
            executed = (teacher_action + noise_vec).astype(np.float32)
            raw = np.asarray(executed, dtype=np.float64).reshape(-1)
            if bool(np.any(raw < low - 1e-12) or np.any(raw > high + 1e-12)):
                clipped += 1
            observations.append(actor_obs.copy())
            actions.append(teacher_action.copy())
            executed_actions.append(executed.copy())
            action_noises.append(noise_vec.copy())
            times.append(float(base.t))
            t_before = _finite(base.t, f"step {step}: time_before")
            obs, reward, terminated, truncated, info = env.step(executed)
            # validate BEFORE the row is appended: a non-finite value must never enter the trace
            if "time" not in info:
                raise J1Error(f"step {step}: info exposes no 'time'; refusing to record a row "
                              "with an unknown timestamp")
            t_after = _finite(info["time"], f"step {step}: info.time")
            reward_value = _finite(reward, f"step {step}: reward")
            row: dict[str, Any] = {
                "step": step, "time_before": t_before,
                "time_after": t_after,
                "reward": reward_value,
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "teacher_action": teacher_action.astype(float).tolist(),
                "executed_action": executed.astype(float).tolist(),
                "action_noise": noise_vec.astype(float).tolist(),
                # STRUCTURED, recursively converted: nested mappings stay mappings
                "reward_terms": _jsonable(info.get("reward_terms", {}), "reward_terms"),
                FSM_KEY: _jsonable(info.get(FSM_KEY), FSM_KEY),
                "prosthetic_state": _prosthetic_state(actor_obs, feature_names),
            }
            for extra in ("observation", "morphology_causal_diagnostics",
                          "morphology_ledger_diagnostics", "online_grf", "online_grf_detector",
                          "observer_raw_sensor_journal"):
                if extra in info:
                    row[extra] = _jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: _jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            trace.append(row)
            if terminated or truncated:
                break
        # Both paths - early termination AND the full-length run - resolve the end_reason from the
        # last recorded row. Nothing is assumed and no success is invented.
        end_reason = _resolve_end_reason(trace)
        obs_arr = np.asarray(observations, dtype=np.float32)
        act_arr = np.asarray(actions, dtype=np.float32)
        exe_arr = np.asarray(executed_actions, dtype=np.float32)
        noi_arr = np.asarray(action_noises, dtype=np.float32)
        tim_arr = np.asarray(times, dtype=np.float64)
        # --- July-usable, auditable dataset schema, asserted before it is written --------------
        if obs_arr.shape != (len(trace), ACTOR_WIDTH):
            raise J1Error(f"observations have shape {obs_arr.shape}, expected "
                          f"{(len(trace), ACTOR_WIDTH)}")
        action_dim_expected = int(np.prod(env.action_space.shape))
        for name, arr in (("actions", act_arr), ("executed_actions", exe_arr),
                          ("action_noises", noi_arr)):
            if arr.shape != (len(trace), action_dim_expected):
                raise J1Error(f"{name} have shape {arr.shape}, expected "
                              f"{(len(trace), action_dim_expected)}")
        if tim_arr.shape != (len(trace),):
            raise J1Error(f"times have shape {tim_arr.shape}, expected {(len(trace),)}")
        for name, arr in (("observations", obs_arr), ("actions", act_arr),
                          ("executed_actions", exe_arr), ("action_noises", noi_arr),
                          ("times", tim_arr)):
            if not bool(np.all(np.isfinite(arr))):
                raise J1Error(f"{name} contain non-finite values")
        realized_noise_rms = [float(v) for v in np.sqrt(np.mean(noi_arr ** 2, axis=0))]
        if list(COLLECTION_SEMANTICS["action_noise_sigma"]) == [0.0, 0.0]:
            if float(np.max(np.abs(noi_arr))) != 0.0:
                raise J1Error("sigma is [0.0, 0.0] but the realised action noise is not exactly "
                              f"zero: max |noise| = {float(np.max(np.abs(noi_arr)))}")
            if not bool(np.array_equal(exe_arr, act_arr)):
                raise J1Error("with zero sigma the executed actions must equal the teacher "
                              "actions bit for bit")
        summary = _summarise(trace, end_reason, clipped, realized_noise_rms)
        gate = _evaluate_gate(summary)
        receipt = {
            "schema": "v26c_j1_collection.1", "stage": STAGE,
            "verdict": "PASS" if gate["pass"] else "FAIL",
            "pinned_config": pre["pinned_config"], "parent": pre["parent"],
            "env_contract": pre["env_contract"], "env_config_sha256": _sha_obj(env_config),
            "collection_semantics": pre["collection_semantics"],
            "label_source": LABEL_SOURCE,
            "runtime_identity": {"fsm_behaviour_version": EXPECTED_FSM_BEHAVIOUR_VERSION,
                                 "event_source": EXPECTED_EVENT_SOURCE,
                                 "observed_versions": summary["fsm_behaviour_versions"],
                                 "observed_event_sources": summary["event_sources"],
                                 "note": "verified on EVERY row; not a new gate, a check that the "
                                         "collected runtime is the preregistered one"},
            "realized_noise_rms": summary["realized_noise_rms"],
            "summary": summary, "j1_gate": dict(J1_GATE), "gate": gate,
            "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
            "actor_contract": pre["actor_contract"],
            "future_todo_non_operational": pre["future_todo_non_operational"],
            "generated_at_utc": _utc(),
        }
        np.savez_compressed(out / "teacher_dataset.npz",
                            observations=obs_arr, actions=act_arr,
                            executed_actions=exe_arr, action_noises=noi_arr,
                            times=tim_arr,
                            actor_feature_names=np.asarray(feature_names, dtype=str))
        # allow_nan=False: a non-finite value must abort the write, never be serialised as NaN
        (out / "teacher_trace.json").write_text(
            json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
        receipt["outputs_sha256"] = {p.name: _sha_file(p)
                                     for p in sorted(out.iterdir()) if p.is_file()}
        (out / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
        if progress:
            print(json.dumps({"verdict": receipt["verdict"], "steps": summary["steps"],
                              "end_reason": end_reason}, indent=2))
        return receipt
    finally:
        try:
            env.close()
        except Exception:
            pass


# --- PRODUCTION FIELD NAMES, verified against the runtime, not inherited from July --------------
# reward_terms carries the phase_* prefixed names; the raw counters live in the NESTED info
# ["phase_fsm"] mapping. An earlier revision read July's unprefixed names and would have reported
# zero cycles on every row.
RT_VALID_CYCLE = "phase_valid_cycle_count"
RT_VALID_HS = "phase_valid_hs_count"
RT_VALID_TO = "phase_valid_to_count"
RT_TIMEOUT_EXCEEDED = "phase_timeout_exceeded"
RT_TIMEOUT_SIDE = "phase_timeout_side"
RT_MORPH_FAILED = "morphology_causal_failed_closed"
RT_PENETRATION = "grf_penetration_m"
FSM_KEY = "phase_fsm"
FSM_REQUIRED = ("resync_count", "hs_cancelled_count", "valid_cycle_count", "valid_hs_count",
                "valid_to_count", "timeout_exceeded", "timeout_side", "fsm_behaviour_version",
                "event_source")
TIMEOUT_SIDE_STANCE, TIMEOUT_SIDE_SWING = 1.0, 2.0
END_REASON_TIMEOUT_PREFIX = "phase_timeout:"
END_REASON_MORPH_FAILURE = "morphology_causal_contract_failure"
END_REASON_FULL_LENGTH = "episode_time_limit"

# reward_terms fields REQUIRED on every row. None of them may be defaulted: a silent 0.0 would
# turn missing telemetry into an apparent pass.
RT_REQUIRED = (RT_VALID_CYCLE, RT_VALID_HS, RT_VALID_TO, RT_TIMEOUT_EXCEEDED, RT_TIMEOUT_SIDE,
               RT_MORPH_FAILED, RT_PENETRATION)

# The collected runtime must BE the preregistered one. This is not a new gate: it verifies that
# what was recorded came from the pinned current/V26 runtime.
EXPECTED_FSM_BEHAVIOUR_VERSION = "v3"
EXPECTED_EVENT_SOURCE = "binary_active_v26"


PROSTHETIC_STATE_FEATURES = ("pros_knee_angle", "pros_knee_angle_vel",
                             "pros_ankle_angle", "pros_ankle_angle_vel")


def _jsonable(value: Any, field: str) -> Any:
    """Recursive, finite JSON conversion that PRESERVES structure.

    An earlier revision stringified every nested mapping, which destroyed phase_fsm, the
    observation and the morphology diagnostics and made the claimed trace false.
    """
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, (int, float)):
        out = float(value)
        if not math.isfinite(out):
            raise J1Error(f"{field} is not finite: {value!r}")
        return int(value) if isinstance(value, int) else out
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v, f"{field}.{k}") for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v, f"{field}[{i}]") for i, v in enumerate(value)]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist(), field)
    return str(value)


def _prosthetic_state(actor_obs: Any, feature_names: tuple[str, ...]) -> dict[str, float]:
    """Knee and ankle q and qdot, extracted BY NAME from the actor feature manifest."""
    missing = [n for n in PROSTHETIC_STATE_FEATURES if n not in feature_names]
    if missing:
        raise J1Error(f"the actor feature manifest lacks {missing}; the kinematics required for a "
                      "later audit cannot be recorded")
    out: dict[str, float] = {}
    for name in PROSTHETIC_STATE_FEATURES:
        out[name] = _finite(actor_obs[feature_names.index(name)], f"prosthetic_state.{name}")
    return out


def _finite(value: Any, field: str) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise J1Error(f"{field} is not numeric: {value!r}") from exc
    if not math.isfinite(out):
        raise J1Error(f"{field} is not finite: {value!r}")
    return out


def _counter(value: Any, field: str) -> int:
    out = _finite(value, field)
    if abs(out - round(out)) > 1e-9:
        raise J1Error(f"{field} must be an integer counter, got {value!r}")
    return int(round(out))


def _remove_own_leaf(leaf: Path) -> None:
    """Delete ONLY the leaf this call created, never a sibling and never the root.

    Guarded three ways: the path must be a directory, its parent must be exactly OUT_ROOT or the
    caller-supplied leaf's own parent, and it must never be OUT_ROOT itself. Any doubt and the
    directory is left in place - an orphan directory is a far smaller problem than deleting the
    wrong one.
    """
    import shutil
    leaf = Path(leaf)
    if not leaf.is_dir():
        return
    if leaf.resolve() == OUT_ROOT.resolve() or leaf.resolve() == HERE.resolve():
        raise J1Error(f"refusing to remove {leaf}: it is the run root, not a leaf")
    if leaf.resolve().parent != OUT_ROOT.resolve():
        # a caller-supplied --out-dir outside the standard root is never auto-removed
        return
    shutil.rmtree(leaf, ignore_errors=True)


def _resolve_end_reason(trace: list[dict[str, Any]]) -> str:
    """The end_reason comes from the LAST row's recorded value, never invented.

    An earlier revision read `trace[-1]["info"]`, a key the rows do not carry, and defaulted the
    full-length path to `episode_time_limit`. That both raised KeyError and would have fabricated
    a success. If the runtime did not expose an end_reason, this fails closed.
    """
    if not trace:
        raise J1Error("the trace is empty; no end_reason can be resolved")
    value = str(trace[-1].get("end_reason", "") or "").strip()
    if not value:
        raise J1Error("the final step exposed no end_reason; refusing to assume "
                      f"{END_REASON_FULL_LENGTH!r} or any other success")
    return value


def _summarise(trace: list[dict[str, Any]], end_reason: str, clipped: int,
               realized_noise_rms: list[float] | None = None) -> dict[str, Any]:
    """Fail-closed aggregation over PRODUCTION field names.

    Cumulative counters are aggregated with max, never with the last row: under the v3
    cancellation contract `valid_hs_count` can DECREASE, so reading the final row alone would
    understate what actually happened.
    """
    if not trace:
        raise J1Error("the trace is empty; nothing can be summarised")
    cycles = hs = to = resync = cancelled = 0
    timeout_stance = timeout_swing = 0
    morph_failed = 0
    penetration = 0.0
    versions: set[str] = set()
    sources: set[str] = set()
    for row in trace:
        step = row.get("step")
        rt = row.get("reward_terms")
        if not isinstance(rt, Mapping):
            raise J1Error(f"step {step}: reward_terms is absent or not a mapping")
        fsm = row.get(FSM_KEY)
        if not isinstance(fsm, Mapping):
            raise J1Error(f"step {step}: info['{FSM_KEY}'] is absent or not a mapping; the v3 "
                          "counters cannot be read and must never default to zero")
        for name in FSM_REQUIRED:
            if name not in fsm:
                raise J1Error(f"step {step}: {FSM_KEY}.{name} is absent; refusing to default it")
        for name in RT_REQUIRED:
            if name not in rt:
                raise J1Error(f"step {step}: reward_terms.{name} is absent; refusing to default "
                              "it, because a silent 0.0 would turn missing telemetry into a pass")
            _finite(rt[name], f"reward_terms.{name}")
        # the collected runtime must BE the preregistered one
        version = str(fsm["fsm_behaviour_version"])
        source = str(fsm["event_source"])
        if version != EXPECTED_FSM_BEHAVIOUR_VERSION:
            raise J1Error(f"step {step}: {FSM_KEY}.fsm_behaviour_version is {version!r}, expected "
                          f"{EXPECTED_FSM_BEHAVIOUR_VERSION!r}; the row did not come from the "
                          "preregistered runtime")
        if source != EXPECTED_EVENT_SOURCE:
            raise J1Error(f"step {step}: {FSM_KEY}.event_source is {source!r}, expected "
                          f"{EXPECTED_EVENT_SOURCE!r}; the row did not come from the "
                          "preregistered runtime")
        # cumulative counters: MAX, because v3 cancellation can decrease valid_hs_count
        cycles = max(cycles, _counter(fsm["valid_cycle_count"], f"{FSM_KEY}.valid_cycle_count"))
        hs = max(hs, _counter(fsm["valid_hs_count"], f"{FSM_KEY}.valid_hs_count"))
        to = max(to, _counter(fsm["valid_to_count"], f"{FSM_KEY}.valid_to_count"))
        resync = max(resync, _counter(fsm["resync_count"], f"{FSM_KEY}.resync_count"))
        cancelled = max(cancelled,
                        _counter(fsm["hs_cancelled_count"], f"{FSM_KEY}.hs_cancelled_count"))
        versions.add(str(fsm["fsm_behaviour_version"]))
        sources.add(str(fsm["event_source"]))
        # the FSM timeout evidence must agree with the reward_terms exposure, with no default
        for rt_name, fsm_name in ((RT_TIMEOUT_EXCEEDED, "timeout_exceeded"),
                                  (RT_TIMEOUT_SIDE, "timeout_side")):
            if not math.isclose(_finite(rt[rt_name], f"reward_terms.{rt_name}"),
                                _finite(fsm[fsm_name], f"{FSM_KEY}.{fsm_name}"),
                                rel_tol=0.0, abs_tol=1e-9):
                raise J1Error(f"step {step}: reward_terms.{rt_name} disagrees with "
                              f"{FSM_KEY}.{fsm_name}")
        # timeouts: reward_terms exposure plus the side code
        if _finite(rt[RT_TIMEOUT_EXCEEDED], f"reward_terms.{RT_TIMEOUT_EXCEEDED}") > 0.0:
            side = _finite(rt[RT_TIMEOUT_SIDE], f"reward_terms.{RT_TIMEOUT_SIDE}")
            if math.isclose(side, TIMEOUT_SIDE_STANCE):
                timeout_stance += 1
            elif math.isclose(side, TIMEOUT_SIDE_SWING):
                timeout_swing += 1
            else:
                raise J1Error(f"step {step}: a timeout was flagged with an unknown side {side!r}")
        if _finite(rt[RT_MORPH_FAILED], f"reward_terms.{RT_MORPH_FAILED}") > 0.0:
            morph_failed += 1
        penetration = max(penetration,
                          _finite(rt[RT_PENETRATION], f"reward_terms.{RT_PENETRATION}"))
        # the FSM counters must agree with the reward_terms exposure; the check is MANDATORY
        for rt_name, fsm_name in ((RT_VALID_CYCLE, "valid_cycle_count"),
                                  (RT_VALID_HS, "valid_hs_count"),
                                  (RT_VALID_TO, "valid_to_count")):
            if not math.isclose(_finite(rt[rt_name], f"reward_terms.{rt_name}"),
                                _finite(fsm[fsm_name], f"{FSM_KEY}.{fsm_name}"),
                                rel_tol=0.0, abs_tol=1e-9):
                raise J1Error(f"step {step}: reward_terms.{rt_name} disagrees with "
                              f"{FSM_KEY}.{fsm_name}")
    # the end_reason also carries timeout and morphology evidence
    if str(end_reason).startswith(END_REASON_TIMEOUT_PREFIX):
        side_name = str(end_reason).split(":", 1)[1]
        if side_name == "stance":
            timeout_stance = max(timeout_stance, 1)
        elif side_name == "swing":
            timeout_swing = max(timeout_swing, 1)
    if str(end_reason) == END_REASON_MORPH_FAILURE:
        morph_failed = max(morph_failed, 1)
    return {
        "steps": len(trace),
        "end_reason": str(end_reason),
        "max_penetration_m": penetration,
        "valid_cycle_count": cycles,
        "valid_hs_count": hs,
        "valid_to_count": to,
        "phase_timeout_stance": timeout_stance,
        "phase_timeout_swing": timeout_swing,
        "morphology_causal_contract_failure": morph_failed,
        "hs_cancelled_count": cancelled,
        "resync_count": resync,
        "action_clipped_steps": int(clipped),
        "fsm_behaviour_versions": sorted(versions),
        "event_sources": sorted(sources),
        "episode_return": _finite(sum(_finite(r["reward"], "reward") for r in trace),
                                  "episode_return"),
        "realized_noise_rms": ([float(v) for v in realized_noise_rms]
                               if realized_noise_rms is not None else None),
        "action_noise_sigma": list(COLLECTION_SEMANTICS["action_noise_sigma"]),
        "aggregation": "cumulative counters aggregated with MAX, never the last row: under the v3 "
                       "cancellation contract valid_hs_count can decrease",
    }


def _evaluate_gate(s: Mapping[str, Any]) -> dict[str, Any]:
    checks = {
        "steps": s["steps"] == J1_GATE["steps_required"],
        "end_reason": s["end_reason"] == J1_GATE["end_reason"],
        "valid_cycles": s["valid_cycle_count"] >= J1_GATE["valid_cycles_min"],
        "phase_timeout_stance": s["phase_timeout_stance"] <= J1_GATE["phase_timeout_stance_max"],
        "phase_timeout_swing": s["phase_timeout_swing"] <= J1_GATE["phase_timeout_swing_max"],
        "morphology_causal_contract_failure":
            s["morphology_causal_contract_failure"]
            <= J1_GATE["morphology_causal_contract_failure_max"],
        "hs_cancelled_count": s["hs_cancelled_count"] <= J1_GATE["hs_cancelled_count_max"],
        "resync_count": s["resync_count"] <= J1_GATE["resync_count_max"],
        "max_penetration_m": s["max_penetration_m"] <= J1_GATE["max_penetration_m_max"],
    }
    return {"checks": checks, "failed": sorted(k for k, v in checks.items() if not v),
            "pass": all(checks.values()),
            "kinematic_quality_applies": False}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J1 prescribed-teacher collection")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    r = collect(authorized_stage=a.authorized_stage,
                out_dir=(Path(a.out_dir) if a.out_dir else None),
                progress=not a.no_progress)
    return 0 if r["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
