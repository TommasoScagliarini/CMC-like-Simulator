"""V26C J10 - the multistart prescribed-teacher collection, on the August V26 lineage.

WHAT IT IS
    Exactly TWO prescribed-teacher rollouts of 500 steps, at the two NON-NOMINAL episode start
    offsets, in the frozen order B then C:

        B   1.756870983805102 s   (-0.20 s)
        C   2.156870983805102 s   (+0.20 s)

    seed 123, sigma exactly zero, teacher lookahead 0.0. The nominal start is NOT re-collected:
    it already exists as the J1 leaf, unchanged.

WHY
    J9R1 failed on the -0.20 s start with 0 cycles and the FSM stuck in WAIT_HS for 500/500 steps.
    The measured cause is a support gap: across all 16713 rows of the J7 training set the FSM
    one-hot never takes the value (1, 0, 0), i.e. phase_fsm_wait_hs is constant zero. This stage
    collects the missing states. It COLLECTS ONLY - it fits nothing.

THE LABEL SOURCE
    target_domain_imitation.prescribed_teacher_action, reused READ-ONLY. That is a PRESCRIBED
    reference action, not a policy sample: no checkpoint is queried for the action, so no RLModule
    is loaded, no Ray cluster is started and no EnvRunner exists. The J8 STUDENT is never used,
    neither as a policy nor as a label source.

THE PARENT
    The August V26 imitative actor, module_state 0ba56eb7..., byte-identical to the parent the
    nominal J1 collection used. It defines the OBSERVATION CONTRACT - width, feature names, mask -
    and is never the action source. July is METHODOLOGY ONLY.

THE GATES
    Per cell and binding: 500 steps, episode_time_limit, >= 2 valid cycles, zero phase timeouts,
    zero morphology causal failures, zero cancelled HS, <= 1 resync, ZERO action clipping, the
    unchanged J3 kinematic gate, and penetration binding-pass under the single contract.
    Collection-wide and binding: at least one observation row with phase_fsm_wait_hs == 1.

PROVENANCE
    Every path recorded in the receipt is LEAF-RELATIVE and therefore valid after the commit; no
    receipt field ever contains a staging path. valid_cycle_count is NOT listed among the
    non-binding diagnostics, because valid_cycles is binding.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1            # hardened env builder, summariser, common gate, parent
import v26c_j3_closed_loop as J3        # the UNCHANGED kinematic gate and telemetry invariant
import v26c_penetration_contract as PC  # the ONLY penetration authority

REPO = J1.REPO
BASELINE = J3.BASELINE


class J10Error(RuntimeError):
    pass


STAGE = "V26C_J10_MULTISTART_TEACHER"
PREREG = HERE / "v26c_j10_prereg_multistart_teacher.json"
PIN_PREREG = "79b1b573eb45831e7333c3dbc539f76a7e9555986b38e73ea83a91590f8241d7"

# ------------------------------------------------------------------ the operative parent --------
PARENT_DIR = J1.PARENT_DIR
PIN_PARENT_STATE_SHA = "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
PIN_RUNTIME_CONFIG_SHA = J1.PIN_RUNTIME_CONFIG_SHA
PIN_CONTRACT = "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461"
# EXCLUDED as a label source and as a policy. Named so the refusal is explicit, never implicit.
J8_STUDENT_DIR = HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1" / "rl_module"

# ------------------------------------------------------------------ collection semantics --------
ROLLOUT_SEED = 123
TEACHER_LOOKAHEAD_S = 0.0
SIGMA = (0.0, 0.0)
NOISE_HOLD_STEPS = 1
EXPECTED_STEPS = 500
ACTOR_WIDTH = J1.ACTOR_WIDTH
LABEL_SOURCE = "target_domain_imitation.prescribed_teacher_action, read-only"
ALLOWED_TEACHER_REUSE = "_teacher.prescribed_teacher_action("
FORBIDDEN_CALL_SITES = ("build_target_env_config", "_teacher.main(", ".main(")

# ------------------------------------------------------------------ the frozen matrix -----------
OFFSET_NOMINAL = 1.956870983805102
OFFSET_MINUS = 1.756870983805102
OFFSET_PLUS = 2.156870983805102
OFFSET_UNIT = "seconds"
FROZEN_OFFSETS = (OFFSET_MINUS, OFFSET_PLUS)
MATRIX: tuple[dict[str, Any], ...] = (
    {"id": "B", "offset_s": OFFSET_MINUS, "label": "-0.20 s"},
    {"id": "C", "offset_s": OFFSET_PLUS, "label": "+0.20 s"},
)

RESET_TIME_TOLERANCE_S = 1e-9
RESET_SEMANTICS = (
    "osim_trj_cmc_like._initialise_episode: "
    "max_start = max(cfg.t_start, cfg.t_end - episode_duration); "
    "requested = cfg.t_start + max(0.0, episode_start_offset_s); "
    "episode_start = min(requested, max_start); self.t = episode_start")

# ------------------------------------------------------------------ the gates -------------------
# J1's common criteria MINUS its 0.020 penetration bar: the sole binding penetration criterion is
# the contract's 0.028 band. Action clipping is BINDING here, unlike J1.
J10_COMMON_GATE: dict[str, Any] = {k: v for k, v in J1.J1_GATE.items()
                                   if k != "max_penetration_m_max"}
J10_COMMON_GATE["action_clipped_steps_max"] = 0
J10_KINEMATIC_GATE: dict[str, Any] = dict(J3.J3_KINEMATIC_GATE)      # UNCHANGED, same as J9R1
COVERAGE_FEATURE = "phase_fsm_wait_hs"
COVERAGE_MIN_ROWS = 1
# valid_cycle_count is DELIBERATELY absent: valid_cycles is binding, so listing its measure here
# would be self-contradictory. action_clipped_steps is absent for the same reason.
DIAGNOSTIC_NOT_BINDING = ("episode_return", "realized_noise_rms", "max_reserve_norm_nm",
                          "mean_reserve_norm_nm", "slew_limited_steps")

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j10_runs", "j10_multistart_teacher_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
RECEIPT_NAME = "v26c_j10_multistart_teacher_receipt.json"
PREFLIGHT_SENTINEL = HERE / "_j10_preflight_sentinel_never_created"
OUTPUT_ROOT_OVERRIDE: Path | None = None

FORBIDDEN_HERE = ("fit", "optimizer step", "weight update", "critic", "PPO", "promotion",
                  "deployability", "a student rollout", "the J8 student as a label source",
                  "any policy as the action source", "a second parent", "widening",
                  "contralateral features", "a standalone 25D actor", "LOTO", "LOCO", "B1R1",
                  "B1R2", "behavioural fail-fast", "an autonomous retry",
                  "a local penetration threshold",
                  "mutating any env field but episode_start_offset_s",
                  "starting a Ray cluster", "starting Ray workers", "EnvRunners",
                  "a staging path in the receipt")


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _sha_array(a: np.ndarray) -> str:
    arr = np.ascontiguousarray(a)
    h = hashlib.sha256()
    h.update(str(arr.dtype).encode())
    h.update(str(arr.shape).encode())
    h.update(arr.tobytes())
    return h.hexdigest()


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


def _leaf_rel(p: Path, root: Path) -> str:
    """LEAF-RELATIVE, so the string stays valid after the staging is renamed onto the leaf.

    This is the J9R1 provenance defect corrected: that receipt recorded absolute staging paths
    which ceased to exist the moment the commit renamed the directory.
    """
    return str(Path(p).relative_to(root)).replace(os.sep, "/")


# ================================================================ inputs =========================

def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J10Error("the J10 preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J10Error(f"the J10 preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage_proposed") != STAGE:
        raise J10Error(f"the preregistration proposes {data.get('stage_proposed')!r}, not {STAGE}")
    local: dict[str, str] = {}
    for rel, pin in data["pinned_artefacts_sha256"].items():
        got = _sha_file(HERE / rel)
        if got != pin:
            raise J10Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        local[rel] = got
    repo: dict[str, str] = {}
    for rel, pin in data["pinned_repo_artefacts_sha256"].items():
        got = _sha_file(REPO / rel)
        if got != pin:
            raise J10Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        repo[rel] = got
    declared = data["matrix"]["cell_list"]
    if [c["id"] for c in declared] != [c["id"] for c in MATRIX]:
        raise J10Error("the preregistered matrix order and the runner's disagree")
    for want, got in zip(MATRIX, declared):
        if float(got["episode_start_offset_s"]) != float(want["offset_s"]):
            raise J10Error(f"cell {want['id']} offset differs from the preregistration")
    if int(data["matrix"]["seed"]) != ROLLOUT_SEED \
            or int(data["matrix"]["steps_per_cell"]) != EXPECTED_STEPS \
            or int(data["matrix"]["cells"]) != len(MATRIX):
        raise J10Error("the preregistration and the runner disagree on seed, steps or cell count")
    if data["matrix"]["behavioural_fail_fast"] is not False:
        raise J10Error("the preregistration must declare behavioural_fail_fast false")
    if float(data["label_source"]["lookahead_s"]) != TEACHER_LOOKAHEAD_S \
            or [float(v) for v in data["label_source"]["sigma"]] != list(SIGMA):
        raise J10Error("the preregistration and the runner disagree on lookahead or sigma")
    if data["coverage_gate"]["binding"] is not True \
            or data["coverage_gate"]["scope"].split(",")[0] != "the COLLECTION AS A WHOLE":
        raise J10Error("the coverage gate must be declared binding and collection-wide")
    # the two provenance corrections, asserted rather than assumed
    if "valid_cycle_count" in data["diagnostics_not_binding"] \
            or "action_clipped_steps" in data["diagnostics_not_binding"]:
        raise J10Error("a binding measure is listed among the non-binding diagnostics")
    return {"file": _rel(PREREG), "sha256": digest,
            "manifest_entries": len(local) + len(repo),
            "pinned_artefacts_sha256": local, "pinned_repo_artefacts_sha256": repo}


def verify_parent() -> dict[str, Any]:
    """The August V26 imitative actor: J1's own check, plus an explicit J8-student refusal."""
    parent = J1.verify_parent()
    if parent["module_state_sha256"] != PIN_PARENT_STATE_SHA:
        raise J10Error(f"the parent is {parent['module_state_sha256']}, not "
                       f"{PIN_PARENT_STATE_SHA}")
    if str(J8_STUDENT_DIR.resolve()) == str(PARENT_DIR.resolve()):
        raise J10Error("the J8 student is not a permitted parent for this stage")
    return {**parent,
            "is_the_only_parent": True,
            "role": "defines the OBSERVATION CONTRACT; it is NOT the action source",
            "same_parent_as_j1": PIN_PARENT_STATE_SHA == J1.PIN_PARENT_STATE_SHA,
            "j8_student_excluded": {
                "path": _rel(J8_STUDENT_DIR),
                "used_as_parent": False, "used_as_policy": False, "used_as_label_source": False,
                "why": "J10 collects PRESCRIBED teacher labels; a student label would defeat the "
                       "purpose of covering states the student has never seen"}}


def runtime_feature_names(base: Any) -> tuple[str, ...]:
    """The 35 RUNTIME actor feature names, taken from the LIVE environment, exactly as J1 does.

    NOT from the parent's actor_feature_manifest.json. That manifest declares 39 names - the
    June-lineage slice, which includes four healthy_*_imitation_target[_vel] features that the
    runtime actor block does not carry. The runtime width is 35 and the 35 are an order-preserving
    subsequence of the 39. J1 reads getattr(base, "actor_feature_names") for the same reason
    (v26c_j1_collect.py:621). Reading the manifest here would fail on the width check and, worse,
    would silently mis-align a column index if the length check were ever relaxed.
    """
    names = tuple(str(n) for n in getattr(base, "actor_feature_names", ()))
    if len(names) != ACTOR_WIDTH:
        raise J10Error(f"the runtime actor exposes {len(names)} feature names, expected "
                       f"{ACTOR_WIDTH}")
    if COVERAGE_FEATURE not in names:
        raise J10Error(f"the coverage feature {COVERAGE_FEATURE!r} is absent from the runtime "
                       "actor feature names")
    return names


def parent_manifest_width() -> dict[str, Any]:
    """Record the 39-vs-35 provenance fact rather than trip over it."""
    manifest = json.loads((PARENT_DIR / "actor_feature_manifest.json").read_text())
    declared = [str(n) for n in manifest["actor_feature_names"]]
    return {"parent_manifest_feature_count": len(declared),
            "runtime_actor_width": ACTOR_WIDTH,
            "coverage_feature_in_parent_manifest": COVERAGE_FEATURE in declared,
            "note": "the parent manifest declares the 39-wide June-lineage slice; the runtime "
                    "actor block is 35 wide and is an order-preserving subsequence of it. The "
                    "collection uses the RUNTIME names from the live environment, never these.",
            "parent_is_provenance_only": "the parent is pinned by hash for lineage and is never "
                                         "loaded: no observation is ever multiplied by its "
                                         "weights in this stage, so the width difference cannot "
                                         "affect a single recorded value"}


def coverage_index(names: Sequence[str]) -> int:
    """Resolved BY NAME, never by a hard-coded index."""
    return int(list(names).index(COVERAGE_FEATURE))


# ================================================================ reset arithmetic ===============

def expected_reset_time(t_start: float, t_end: float, episode_duration: float | None,
                        offset_s: float) -> dict[str, Any]:
    """The production episode-start arithmetic. Offsets are RELATIVE to cfg.t_start."""
    max_start = float(t_end)
    if episode_duration is not None:
        max_start = float(t_end) - float(episode_duration)
    max_start = max(float(t_start), max_start)
    requested = float(t_start) + max(0.0, float(offset_s))
    start = min(requested, max_start)
    return {"cfg_t_start_s": float(t_start), "cfg_t_end_s": float(t_end),
            "episode_duration_s": None if episode_duration is None else float(episode_duration),
            "episode_start_offset_s": float(offset_s),
            "requested_unclamped_time_s": requested, "max_start_s": max_start,
            "expected_reset_time_s": start, "clamped": bool(requested > max_start),
            "semantics": RESET_SEMANTICS}


# ================================================================ env configuration ==============

def base_env_config(output_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """The pinned config through the HARDENED J1 builder, with a REAL output root.

    A real root is what gives record_outputs and save_outputs_on_close, and therefore the 19
    sim_outputs per cell. Passing None is the J9 defect and is refused.
    """
    if output_root is None:
        raise J10Error("base_env_config requires a REAL output root; None disables recording")
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_root)
    J1.verify_env_config(env, cfg)
    if float(env["episode_start_offset_s"]) != OFFSET_NOMINAL:
        raise J10Error(f"the pinned config's offset is {env['episode_start_offset_s']}, expected "
                       f"the nominal {OFFSET_NOMINAL}")
    for flag in ("record_outputs", "save_outputs_on_close"):
        if env.get(flag) is not True:
            raise J10Error(f"{flag} is {env.get(flag)!r}, expected exactly True")
    if not env.get("output_prefix") or not env.get("output_dir"):
        raise J10Error("the base env config carries no output_prefix or no output_dir")
    return env, cfg


def cell_env_config(base: Mapping[str, Any], offset_s: float,
                    output_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Mutate ONLY episode_start_offset_s and output_dir, and PROVE nothing else moved."""
    if float(offset_s) not in [float(v) for v in FROZEN_OFFSETS]:
        raise J10Error(f"{offset_s} is not one of the two frozen offsets {FROZEN_OFFSETS}")
    env = dict(base)
    env["episode_start_offset_s"] = float(offset_s)
    env["output_dir"] = str(output_dir)
    allowed = {"episode_start_offset_s", "output_dir"}
    differing = sorted(k for k in set(base) | set(env)
                       if base.get(k, "<absent>") != env.get(k, "<absent>"))
    forbidden = sorted(set(differing) - allowed)
    if forbidden:
        raise J10Error(f"the cell config mutates {forbidden}; only the offset and output_dir may "
                       "differ from the verified base")
    added = sorted(set(env) - set(base))
    removed = sorted(set(base) - set(env))
    if removed or sorted(set(added) - {"output_dir"}):
        raise J10Error(f"the cell config adds {added} and removes {removed}")
    # STABLE KEYS: every key that is not one of the two declared-mutable ones must be equal.
    stable_keys = set(base) - allowed
    differing_stable = sorted(k for k in stable_keys if base[k] != env.get(k, "<absent>"))
    if differing_stable:
        raise J10Error(f"these stable fields differ from the verified base: {differing_stable}")
    return env, {"mutated_keys": differing, "allowed_superset": sorted(allowed),
                 "mutated_is_subset_of_allowed": True,
                 "stable_keys_count": len(stable_keys), "stable_keys_all_identical": True,
                 "episode_start_offset_s": float(offset_s), "unit": OFFSET_UNIT,
                 "delta_from_nominal_s": float(offset_s) - OFFSET_NOMINAL,
                 "base_verified_by_j1_before_mutation": True,
                 "second_j1_verify_on_mutated_config": False,
                 "recording_inherited_from_base": True}


# ================================================================ the runtime stack ==============

class _Stack:
    """The heavy runtime, injected. Production builds it in collect(); tests inject a fake."""

    def __init__(self, *, name: str, operational: bool, make_env: Any, teacher_action: Any,
                 held_normal: Any) -> None:
        self.name = name
        self.operational = bool(operational)
        self.make_env = make_env
        self.teacher_action = teacher_action
        self.held_normal = held_normal


def production_stack() -> _Stack:
    """env_factory, exploration_noise and the prescribed teacher. NO RLlib, NO Ray, NO policy."""
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import env_factory
    import exploration_noise
    import target_domain_imitation as _teacher   # read-only reuse of prescribed_teacher_action
    if not hasattr(_teacher, "prescribed_teacher_action"):
        raise J10Error("the teacher module exposes no prescribed_teacher_action")
    return _Stack(name="production", operational=True,
                  make_env=env_factory.make_cmc_env,
                  teacher_action=_teacher.prescribed_teacher_action,
                  held_normal=exploration_noise.HeldStandardNormal)


# ================================================================ gate ===========================

def evaluate_cell_gate(summary: Mapping[str, Any], knee: np.ndarray, ankle: np.ndarray,
                       penetration: Mapping[str, Any]) -> dict[str, Any]:
    """The per-cell BINDING gate. Telemetry coherence is evaluated separately."""
    checks = {
        "steps": summary["steps"] == J10_COMMON_GATE["steps_required"],
        "end_reason": summary["end_reason"] == J10_COMMON_GATE["end_reason"],
        "valid_cycles": summary["valid_cycle_count"] >= J10_COMMON_GATE["valid_cycles_min"],
        "phase_timeout_stance":
            summary["phase_timeout_stance"] <= J10_COMMON_GATE["phase_timeout_stance_max"],
        "phase_timeout_swing":
            summary["phase_timeout_swing"] <= J10_COMMON_GATE["phase_timeout_swing_max"],
        "morphology_causal_contract_failure":
            summary["morphology_causal_contract_failure"]
            <= J10_COMMON_GATE["morphology_causal_contract_failure_max"],
        "hs_cancelled_count":
            summary["hs_cancelled_count"] <= J10_COMMON_GATE["hs_cancelled_count_max"],
        "resync_count": summary["resync_count"] <= J10_COMMON_GATE["resync_count_max"],
        "action_clipped_steps":
            summary["action_clipped_steps"] <= J10_COMMON_GATE["action_clipped_steps_max"],
        "penetration_hard_binding": bool(penetration["binding_pass"]),
    }
    kin = J3.kinematic_quality(np.asarray(knee, dtype=np.float64),
                               np.asarray(ankle, dtype=np.float64))
    for name, block in kin.items():
        checks[f"kinematic_{name}"] = bool(block["pass"])
    failed = sorted(k for k, v in checks.items() if not v)
    return {
        "criteria": {"common": dict(J10_COMMON_GATE),
                     "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                           for k, v in J10_KINEMATIC_GATE.items()},
                     "penetration": "the contract's 0.028 binding band, the SOLE binding "
                                    "penetration criterion"},
        "kinematic_quality": kin,
        "ankle_min_direction": "ankle_min <= -0.03 rad; more negative PASSES, exactly -0.03 PASSES",
        "action_clipping_is_binding": True,
        "checks": checks, "failed": failed, "pass": not failed,
        "telemetry_integrity_evaluated_separately": True,
    }


def evaluate_coverage(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """The COLLECTION-WIDE coverage gate: at least one row in the missing FSM state."""
    per_cell = {r["id"]: int(r["wait_hs_rows"]) for r in rows}
    total = int(sum(per_cell.values()))
    return {
        "feature": COVERAGE_FEATURE,
        "resolved_by": "name, from the parent's actor feature manifest",
        "criterion": f"at least {COVERAGE_MIN_ROWS} row with {COVERAGE_FEATURE} == 1, "
                     "across the collection as a whole",
        "scope": "collection-wide, NOT per cell",
        "per_cell_rows": per_cell, "total_rows": total,
        "rows_required": COVERAGE_MIN_ROWS,
        "pass": bool(total >= COVERAGE_MIN_ROWS),
        "binding": True,
        "why": "the J7 training set holds phase_fsm_wait_hs constant at zero across all 16713 "
               "rows; a collection without the missing state would not close the gap",
        "no_invented_thresholds": "no minimum count beyond one, no fraction and no per-cell "
                                  "requirement is asserted",
    }


def penetration_report(series: Sequence[float], contract: Mapping[str, Any],
                       label: str) -> dict[str, Any]:
    ev = PC.evaluate_series(series, contract, label=label)
    return {**ev, "contract_sha256": contract["sha256"],
            "evaluated_by": "v26c_penetration_contract.evaluate_series",
            "soft_and_july_are_diagnostic": True, "sole_binding": "above_hard_binding"}


# ================================================================ destination guards =============

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J10Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J10Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J10Error("--collect requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J10Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J10Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J10Error(f"the authorised leaf already exists; this stage is no-clobber and "
                       f"single-execution: {leaf}")
    return leaf


# ================================================================ one cell =======================

def run_cell(cell: Mapping[str, Any], *, stack: _Stack, base_env: Mapping[str, Any],
             cfg: Mapping[str, Any], contract: Mapping[str, Any], staging: Path,
             progress: bool = True) -> dict[str, Any]:
    """ONE prescribed-teacher rollout. Builds, resets, steps and CLOSES exactly one environment."""
    cid = str(cell["id"])
    sim_out = staging / f"j10_cell_{cid}_sim_outputs"
    env_config, mutation = cell_env_config(base_env, float(cell["offset_s"]), sim_out)

    np.random.seed(ROLLOUT_SEED)
    env = stack.make_env(env_config)
    env_closed = False
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=ROLLOUT_SEED)

        sim_cfg = getattr(base, "cfg", None)
        if sim_cfg is None or not hasattr(sim_cfg, "t_start") or not hasattr(sim_cfg, "t_end"):
            raise J10Error(f"cell {cid}: the environment exposes no cfg.t_start / cfg.t_end")
        env_cfg_live = getattr(base, "env_cfg", None)
        if env_cfg_live is None or not hasattr(env_cfg_live, "episode_duration"):
            raise J10Error(f"cell {cid}: the environment exposes no env_cfg.episode_duration")
        live_duration = getattr(env_cfg_live, "episode_duration")
        declared = env_config.get("episode_duration")
        if (live_duration is None) != (declared is None):
            raise J10Error(f"cell {cid}: live and declared episode_duration disagree on presence")
        if live_duration is not None and abs(float(live_duration) - float(declared)) > 1e-12:
            raise J10Error(f"cell {cid}: live episode_duration {live_duration} differs from the "
                           f"declared {declared} by more than 1e-12")
        reset_expect = expected_reset_time(float(sim_cfg.t_start), float(sim_cfg.t_end),
                                           None if live_duration is None else float(live_duration),
                                           float(cell["offset_s"]))
        reset_time = float(J1._finite(base.t, f"cell {cid}: reset time"))
        reset_error = abs(reset_time - reset_expect["expected_reset_time_s"])
        if reset_error > RESET_TIME_TOLERANCE_S:
            raise J10Error(f"cell {cid}: reset time {reset_time} differs from the expected "
                           f"{reset_expect['expected_reset_time_s']} by {reset_error} s")

        # the RUNTIME feature names, from the live env - never the 39-wide parent manifest
        names = runtime_feature_names(base)
        cov_idx = coverage_index(names)
        n_actor = int(getattr(base, "n_actor", ACTOR_WIDTH))
        if n_actor != ACTOR_WIDTH:
            raise J10Error(f"cell {cid}: the env exposes n_actor {n_actor}, expected {ACTOR_WIDTH}")
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)
        sigma = np.asarray(SIGMA, dtype=np.float32).reshape(action_shape)
        noise = stack.held_normal(np.random.default_rng(ROLLOUT_SEED), action_shape,
                                  NOISE_HOLD_STEPS)

        observations: list[np.ndarray] = []
        actions: list[np.ndarray] = []
        executed_actions: list[np.ndarray] = []
        action_noises: list[np.ndarray] = []
        times: list[float] = []
        trace: list[dict[str, Any]] = []
        knee: list[float] = []
        ankle: list[float] = []
        pen: list[float] = []
        clipped = 0
        wait_hs_rows = 0

        for step in range(1, EXPECTED_STEPS + 1):
            actor_obs = np.asarray(obs, dtype=np.float32).reshape(-1)[:ACTOR_WIDTH]
            if actor_obs.size != ACTOR_WIDTH:
                raise J10Error(f"cell {cid} step {step}: the actor observation is "
                               f"{actor_obs.size}D, expected {ACTOR_WIDTH}")
            if float(actor_obs[cov_idx]) == 1.0:
                wait_hs_rows += 1
            target_t = min(float(base.t) + float(base.env_cfg.segment_duration),
                           float(base._episode_end))
            teacher_action = np.asarray(
                stack.teacher_action(base, target_t, lookahead_s=TEACHER_LOOKAHEAD_S),
                dtype=np.float32).reshape(action_shape)
            noise_vec = (np.asarray(noise.next(), dtype=np.float32).reshape(action_shape)
                         * sigma).astype(np.float32)
            if float(np.max(np.abs(noise_vec))) != 0.0:
                raise J10Error(f"cell {cid} step {step}: sigma is zero, so the noise must be "
                               f"exactly zero; got {noise_vec.tolist()}")
            executed = (teacher_action + noise_vec).astype(np.float32)
            raw = np.asarray(executed, dtype=np.float64).reshape(-1)
            if bool(np.any(raw < low - 1e-12) or np.any(raw > high + 1e-12)):
                clipped += 1

            observations.append(actor_obs.copy())
            actions.append(teacher_action.reshape(-1).copy())
            executed_actions.append(executed.reshape(-1).copy())
            action_noises.append(noise_vec.reshape(-1).copy())
            times.append(float(base.t))
            pros = J1._prosthetic_state(actor_obs, tuple(names))
            knee.append(pros["pros_knee_angle"])
            ankle.append(pros["pros_ankle_angle"])
            t_before = J1._finite(base.t, f"cell {cid} step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(executed)

            if "time" not in info:
                raise J10Error(f"cell {cid} step {step}: info exposes no 'time'")
            terms = J1._jsonable(info.get("reward_terms", {}), "reward_terms")
            if "grf_penetration_m" not in terms:
                raise J10Error(f"cell {cid} step {step}: reward_terms carries no "
                               "grf_penetration_m")
            pen.append(float(terms["grf_penetration_m"]))
            row: dict[str, Any] = {
                "step": step, "cell": cid,
                "time_before": t_before,
                "time_after": J1._finite(info["time"], f"cell {cid} step {step}: info.time"),
                "reward": J1._finite(reward, f"cell {cid} step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "prescribed_teacher_action": teacher_action.reshape(-1).astype(float).tolist(),
                "executed_action": executed.reshape(-1).astype(float).tolist(),
                "action_noise": noise_vec.reshape(-1).astype(float).tolist(),
                "stepped_with": "executed_action (== prescribed teacher at sigma 0)",
                "label_source": LABEL_SOURCE,
                "reward_terms": terms,
                J1.FSM_KEY: J1._jsonable(info.get(J1.FSM_KEY), J1.FSM_KEY),
                "prosthetic_state": pros,
            }
            for extra in ("morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                          "online_grf", "online_grf_detector"):
                if extra in info:
                    row[extra] = J1._jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: J1._jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", J1.FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            trace.append(row)
            if progress and (step % 50 == 0 or step == 1):
                print(json.dumps({"cell": cid, "step": step, "reward": row["reward"]}), flush=True)
            if terminated or truncated:
                break
    except BaseException:
        try:
            env.close()
            env_closed = True
        except Exception:                                       # pragma: no cover
            env_closed = False
        raise
    else:
        try:
            env.close()
        except Exception as exc:
            raise J10Error(f"cell {cid}: the environment failed to close cleanly, so the cell is "
                           f"technically invalid and nothing is committed: {exc!r}") from exc
        env_closed = True

    end_reason = J1._resolve_end_reason(trace)
    obs_arr = np.asarray(observations, dtype=np.float32)
    act_arr = np.asarray(actions, dtype=np.float32)
    exe_arr = np.asarray(executed_actions, dtype=np.float32)
    noi_arr = np.asarray(action_noises, dtype=np.float32)
    tim_arr = np.asarray(times, dtype=np.float64)
    knee_arr = np.asarray(knee, dtype=np.float64)
    ankle_arr = np.asarray(ankle, dtype=np.float64)
    pen_arr = np.asarray(pen, dtype=np.float64)
    summary = J1._summarise(trace, end_reason, clipped, [0.0, 0.0])
    if list(summary["action_noise_sigma"]) != [0.0, 0.0]:
        raise J10Error("the summariser reports a non-zero sigma; this stage injects none")
    penetration = penetration_report(pen_arr, contract, f"J10 cell {cid}")
    gate = evaluate_cell_gate(summary, knee_arr, ankle_arr, penetration)
    integrity = J3.telemetry_integrity(summary)
    verdict = J3.overall_verdict(gate, integrity)

    if not sim_out.is_dir() or not any(sim_out.iterdir()):
        raise J10Error(f"cell {cid}: production sim_outputs are missing")

    ds = staging / f"j10_cell_{cid}_teacher_dataset.npz"
    np.savez_compressed(ds, observations=obs_arr, actions=act_arr,
                        executed_actions=exe_arr, action_noises=noi_arr, times=tim_arr,
                        actor_feature_names=np.asarray(names, dtype=str))
    tr = staging / f"j10_cell_{cid}_trace.json"
    tr.write_text(json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
    kn = staging / f"j10_cell_{cid}_kinematics.npz"
    np.savez_compressed(kn, knee_rad=knee_arr, ankle_rad=ankle_arr,
                        actor_feature_names=np.asarray(names, dtype=str))
    pn = staging / f"j10_cell_{cid}_penetration.npz"
    np.savez_compressed(pn, penetration_m=pen_arr)

    return {
        "id": cid, "label": str(cell["label"]),
        "episode_start_offset_s": float(cell["offset_s"]), "offset_unit": OFFSET_UNIT,
        "seed": ROLLOUT_SEED,
        "env_mutation": mutation, "env_config_sha256": _sha_obj(env_config),
        "reset_check": {**reset_expect, "actual_reset_time_s": reset_time,
                        "reset_time_error_s": reset_error,
                        "tolerance_s": RESET_TIME_TOLERANCE_S,
                        "checked_immediately_after_reset": True,
                        "compared_against": "cfg.t_start + offset, clamped to max_start"},
        "label_semantics": {"source": LABEL_SOURCE, "lookahead_s": TEACHER_LOOKAHEAD_S,
                            "sigma": list(SIGMA), "noise_hold_steps": NOISE_HOLD_STEPS,
                            "executed_equals_teacher": bool(np.array_equal(act_arr, exe_arr)),
                            "max_abs_noise": float(np.max(np.abs(noi_arr))),
                            "policy_queried": False, "j8_student_used": False},
        "summary": summary,
        "kinematics": {"steps": int(knee_arr.size),
                       "knee_min_rad": float(knee_arr.min()), "knee_max_rad": float(knee_arr.max()),
                       "ankle_min_rad": float(ankle_arr.min()),
                       "ankle_max_rad": float(ankle_arr.max())},
        "penetration": penetration,
        "coverage": {"feature": COVERAGE_FEATURE, "wait_hs_rows": int(wait_hs_rows),
                     "rows": int(obs_arr.shape[0])},
        "wait_hs_rows": int(wait_hs_rows),
        "gate": gate, "telemetry_integrity": integrity, "verdict": verdict,
        "behavioural_pass": bool(gate["pass"]), "telemetry_valid": bool(integrity["pass"]),
        "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
        "env_closed": env_closed,
        "dataset_schema": {"observations": list(obs_arr.shape), "actions": list(act_arr.shape),
                           "executed_actions": list(exe_arr.shape),
                           "action_noises": list(noi_arr.shape), "times": list(tim_arr.shape),
                           "actor_feature_names": len(names)},
        "content_hashes": {"observations": _sha_array(obs_arr), "actions": _sha_array(act_arr),
                           "executed_actions": _sha_array(exe_arr),
                           "action_noises": _sha_array(noi_arr), "times": _sha_array(tim_arr),
                           "knee_rad": _sha_array(knee_arr), "ankle_rad": _sha_array(ankle_arr),
                           "penetration_m": _sha_array(pen_arr)},
        # LEAF-RELATIVE paths only: valid after the commit renames the staging onto the leaf
        "artefacts": {"teacher_dataset": _leaf_rel(ds, staging),
                      "trace": _leaf_rel(tr, staging),
                      "kinematics": _leaf_rel(kn, staging),
                      "penetration": _leaf_rel(pn, staging),
                      "sim_outputs": _leaf_rel(sim_out, staging),
                      "sim_outputs_file_count": len(list(sim_out.iterdir()))},
        "paths_are_leaf_relative": True,
    }


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: no heavy import, no environment, no write."""
    heavy_before = sorted(m for m in ("torch", "ray", "opensim", "env_factory",
                                      "target_domain_imitation", "gymnasium")
                          if m in sys.modules)
    prereg = verify_prereg()
    parent = verify_parent()
    widths = parent_manifest_width()
    contract = PC.load_contract()
    if contract["sha256"] != PIN_CONTRACT:
        raise J10Error(f"the penetration contract changed: {contract['sha256']}")
    sentinel_before = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                       "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    env, cfg = base_env_config(PREFLIGHT_SENTINEL)
    sentinel_after = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                      "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    planned = []
    for cell in MATRIX:
        _, mutation = cell_env_config(env, float(cell["offset_s"]),
                                      HERE / "PREFLIGHT_NOT_A_PATH")
        planned.append({"id": cell["id"], "label": cell["label"], "seed": ROLLOUT_SEED,
                        "episode_start_offset_s": float(cell["offset_s"]),
                        "offset_unit": OFFSET_UNIT,
                        "delta_from_nominal_s": mutation["delta_from_nominal_s"],
                        "mutated_keys": mutation["mutated_keys"],
                        "steps": EXPECTED_STEPS, "sigma": list(SIGMA),
                        "lookahead_s": TEACHER_LOOKAHEAD_S,
                        "expected_reset_time_s": expected_reset_time(
                            11.99, 30.0, float(env["episode_duration"]),
                            float(cell["offset_s"]))["expected_reset_time_s"]})
    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J10 lock is already held or was left behind: {lock_path}")
    heavy_after = sorted(m for m in ("torch", "ray", "opensim", "env_factory",
                                     "target_domain_imitation", "gymnasium")
                         if m in sys.modules)
    introduced = sorted(set(heavy_after) - set(heavy_before))
    if introduced:
        blockers.append(f"the preflight introduced heavy modules: {introduced}")
    if sentinel_after != sentinel_before or sentinel_after["root"]:
        blockers.append(f"the preflight created the sentinel output root: {sentinel_after}")
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "policy_loaded": False,
                  "fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                  "student_rollout": False,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False,
                  "heavy_modules_before": heavy_before, "heavy_modules_after": heavy_after,
                  "heavy_modules_introduced_by_preflight": introduced},
        "preregistration": prereg,
        "parent": parent,
        "label_source": {"source": LABEL_SOURCE, "lookahead_s": TEACHER_LOOKAHEAD_S,
                         "sigma": list(SIGMA), "noise_hold_steps": NOISE_HOLD_STEPS,
                         "is_a_policy": False, "j8_student_used": False,
                         "allowed_reuse": ALLOWED_TEACHER_REUSE,
                         "forbidden_call_sites": list(FORBIDDEN_CALL_SITES)},
        "recording_instrumentation": {
            "record_outputs": env["record_outputs"],
            "save_outputs_on_close": env["save_outputs_on_close"],
            "output_prefix": env["output_prefix"],
            "verified_fail_closed_before_any_env": True,
            "expected_sim_outputs_per_cell": 19},
        "preflight_sentinel": {"root": _rel(PREFLIGHT_SENTINEL),
                               "existed_before": sentinel_before, "exists_after": sentinel_after,
                               "created_by_the_preflight": False, "measured_not_assumed": True},
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config + verify_env_config",
                    "environments": "one at a time, sequential, closed fail-closed",
                    "rllib_checkpoint_loader": False,
                    "ray_cluster_started": False, "ray_workers_started": False,
                    "env_runners": False,
                    "note": "no policy is queried, so no RLlib module is loaded at all"},
        "penetration_authority": {
            "contract": contract["path"], "contract_sha256": contract["sha256"],
            "bands_m": {"soft_diagnostic": contract["soft_m"],
                        "july_legacy": contract["july_legacy_m"],
                        "hard_binding": contract["hard_m"]},
            "semantics": {"above_soft_iff": "> 0.020 (diagnostic)",
                          "july_legacy_breach_iff": ">= 0.025 (diagnostic)",
                          "binding_pass_iff": "<= 0.028, so exactly 0.028 PASSES"},
            "local_thresholds_in_this_module": 0},
        "planned_matrix": planned,
        "gate_specification": {
            "binding_common": dict(J10_COMMON_GATE),
            "binding_kinematic": {k: (list(v) if isinstance(v, tuple) else v)
                                  for k, v in J10_KINEMATIC_GATE.items()},
            "binding_penetration": "the contract's 0.028 band",
            "binding_coverage": {"feature": COVERAGE_FEATURE, "rows_required": COVERAGE_MIN_ROWS,
                                 "scope": "collection-wide"},
            "action_clipping_is_binding": True,
            "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
            "valid_cycle_count_not_listed_as_diagnostic": True,
            "telemetry_integrity": "SEPARATE fail-closed technical invariant"},
        "coverage_context": {
            "j7_gap": "phase_fsm_wait_hs is constant zero across all 16713 J7 rows; the FSM "
                      "one-hot never takes (1, 0, 0)",
            "feature": COVERAGE_FEATURE,
            "resolved_by_name_from_the_live_env": True,
            "not_resolved_from_the_parent_manifest": True,
            "actor_width": widths},
        "reset_check": {"tolerance_s": RESET_TIME_TOLERANCE_S, "semantics": RESET_SEMANTICS,
                        "offset_is_relative_to_t_start": True,
                        "never_compared_against_the_offset_itself": True},
        "would_write": {"leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
                        "leaf_exists": bool(leaf.exists()),
                        "per_cell": ["j10_cell_<ID>_teacher_dataset.npz",
                                     "j10_cell_<ID>_trace.json",
                                     "j10_cell_<ID>_kinematics.npz",
                                     "j10_cell_<ID>_penetration.npz",
                                     "j10_cell_<ID>_sim_outputs/"],
                        "aggregate": RECEIPT_NAME,
                        "staging": staging.name, "lock": lock_path.name,
                        "paths_recorded_leaf_relative": True,
                        "protocol": "exclusive sibling lock, staging, verify, re-check, atomic "
                                    "rename",
                        "outcome_fail_still_commits": True},
        "outcome_policy": {"deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False, "fit_authorized": False,
                           "critic_authorized": False, "ppo_authorized_to_start": False,
                           "single_execution": True, "no_autonomous_retry": True},
        "deferred_todo": json.loads(PREREG.read_text())["deferred_todo"],
        "requires_to_collect": {"flag": "--collect", "stage_token": STAGE,
                                "out": "must equal the authorised leaf exactly"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ the collection =================

def collect(out_arg: str | None, stage_token: str | None, *, stack: _Stack | None = None,
            progress: bool = True) -> dict[str, Any]:
    """Both cells, in the frozen order, with NO behavioural fail-fast."""
    if OUTPUT_ROOT_OVERRIDE is not None and stack is None:
        raise J10Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. It is permitted "
                       f"only for synthetic, isolated tests.")
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    pre = preflight()
    if pre["blockers"]:
        raise J10Error(f"preflight BLOCKED: {pre['blockers']}")

    contract = PC.load_contract()
    parent_before = verify_parent()

    injected = stack is not None
    stack = stack if stack is not None else production_stack()

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J10Error(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    staging_created: Path | None = None
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise J10Error(f"the J10 lock already exists: {lock_path}. This stage fails closed "
                           f"and removes no lock it does not own.") from exc
        lock_owned = lock_path
        try:
            os.write(fd, json.dumps({"stage": STAGE, "pid": os.getpid(),
                                     "leaf": str(leaf)}).encode("utf-8"))
        finally:
            os.close(fd)
        staging.mkdir()
        staging_created = staging

        base_env, cfg = base_env_config(staging)
        for flag in ("record_outputs", "save_outputs_on_close"):
            if base_env.get(flag) is not True:
                raise J10Error(f"the base about to be used has {flag}={base_env.get(flag)!r}; "
                               "refusing before constructing any environment")

        cells: list[dict[str, Any]] = []
        for cell in MATRIX:
            # NO behavioural fail-fast: both cells run. A technical exception still fails closed.
            cells.append(run_cell(cell, stack=stack, base_env=base_env, cfg=cfg,
                                  contract=contract, staging=staging, progress=progress))

        parent_after = verify_parent()
        if parent_after["module_state_sha256"] != parent_before["module_state_sha256"]:
            raise J10Error("the operative parent changed during the collection")

        coverage = evaluate_coverage(cells)
        behavioural = sum(1 for c in cells if c["behavioural_pass"])
        valid = sum(1 for c in cells if c["telemetry_valid"])
        aggregate_pass = (behavioural == len(MATRIX) and valid == len(MATRIX)
                          and bool(coverage["pass"]))
        verdict = ("PASS" if aggregate_pass
                   else ("INVALID" if valid != len(MATRIX) else "FAIL"))

        receipt = {
            "schema": "v26c_j10_multistart_teacher_receipt.1", "stage": STAGE,
            "verdict": verdict, "aggregate_pass": aggregate_pass,
            "cells_behavioural_pass": behavioural, "cells_telemetry_valid": valid,
            "cells_total": len(MATRIX),
            "aggregate_rule": "PASS iff 2/2 behavioural PASS, 2/2 telemetry-valid AND the "
                              "collection-wide coverage gate is satisfied",
            "coverage": coverage,
            "stack": {"name": stack.name, "operational": stack.operational, "injected": injected,
                      "note": ("an INJECTED stack is a test double: this receipt is NOT "
                               "operational evidence" if injected
                               else "the production env_factory / teacher stack")},
            "preregistration": pre["preregistration"],
            "parent_before": parent_before, "parent_after": parent_after,
            "parent_unchanged": True,
            "label_source": pre["label_source"],
            "recording_instrumentation": pre["recording_instrumentation"],
            "runtime": pre["runtime"],
            "penetration_authority": pre["penetration_authority"],
            "gate_specification": pre["gate_specification"],
            "reset_check": pre["reset_check"],
            "coverage_context": pre["coverage_context"],
            "cells": cells,
            "provenance": {
                "paths_are_leaf_relative": True,
                "no_staging_path_recorded": True,
                "why": "the J9R1 receipt recorded absolute staging paths that ceased to exist at "
                       "the commit; every path here is relative to the committed leaf",
                "valid_cycle_count_listed_as_diagnostic": False,
                "why_not": "valid_cycles is a BINDING check; listing its measure among the "
                           "non-binding diagnostics would be self-contradictory"},
            "inert": {"fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                      "parent_edited": False, "student_used": False, "policy_queried": False,
                      "rllib_checkpoint_loader": False, "ray_cluster_started": False,
                      "ray_workers_started": False, "env_runners": False},
            "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                        "fit_authorized": False, "single_execution": True,
                        "no_autonomous_retry": True,
                        "note": "this stage produces a dataset and authorises nothing"},
            "forbidden_here": list(FORBIDDEN_HERE),
        }
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False, default=str) + "\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J10Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None
    except BaseException:
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            shutil.rmtree(staging_created, ignore_errors=True)
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass
            lock_owned = None
        if parent_created is not None:
            try:
                parent_created.rmdir()
            except OSError:
                pass
        raise
    finally:
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass

    return {"verdict": verdict, "stage": STAGE, "leaf": _rel(leaf),
            "aggregate_pass": aggregate_pass,
            "coverage": {"total_wait_hs_rows": coverage["total_rows"],
                         "pass": coverage["pass"]},
            "cells": [{"id": c["id"], "offset_s": c["episode_start_offset_s"],
                       "verdict": c["verdict"], "failed": c["gate"]["failed"],
                       "wait_hs_rows": c["wait_hs_rows"],
                       "max_penetration_m": c["penetration"]["max_penetration_m"]}
                      for c in cells],
            "receipt_sha256": _sha_file(leaf / RECEIPT_NAME),
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "fit_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J10 multistart prescribed-teacher collection")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.collect:
        r = collect(a.out, a.authorized_stage, progress=not a.no_progress)
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "PASS" else 1
    if a.out is not None:
        raise J10Error("--out is meaningless without --collect; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
