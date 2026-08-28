"""V26C J10R1 - the multistart prescribed-teacher collection, on the August V26 lineage.

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


class J10R1Error(RuntimeError):
    pass


STAGE = "V26C_J10R1_MULTISTART_TEACHER"
PREREG = HERE / "v26c_j10r1_prereg_multistart_teacher.json"
PIN_PREREG = "72fcd6b9971ac2d3a2d8dda68af8d1aec9aba366c0a2a9ca679317d26fa12874"
# The J10 bundle the architect REJECTED before execution. It is immutable evidence of what
# was proposed and refused, so this stage refuses to run if any of it changed.
PIN_REJECTED_J10_REPORT_REL = "reports/user/2026-08-27_v26c_j10_multistart_teacher_readiness.md"

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
# EXACTLY this many regular files per cell. A non-empty directory is NOT enough:
# 18 would mean a writer failed silently, 20 would mean something else wrote here.
EXPECTED_SIM_OUTPUT_FILES = 19
# valid_cycle_count is DELIBERATELY absent: valid_cycles is binding, so listing its measure here
# would be self-contradictory. action_clipped_steps is absent for the same reason.
DIAGNOSTIC_NOT_BINDING = ("episode_return", "realized_noise_rms", "max_reserve_norm_nm",
                          "mean_reserve_norm_nm", "slew_limited_steps")

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j10r1_runs", "j10r1_multistart_teacher_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
RECEIPT_NAME = "v26c_j10r1_multistart_teacher_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
TECHNICAL_INVALID_NAME = "TECHNICAL_INVALID"
PREFLIGHT_SENTINEL = HERE / "_j10r1_preflight_sentinel_never_created"
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
    try:
        rel = Path(p).relative_to(root)
    except ValueError:
        raise J10R1Error(f"{p} is not under the staging root {root}; it cannot be recorded as a "
                         f"leaf-relative path") from None
    # relative_to is purely LEXICAL: a root reached through a ".." segment can yield an escaping
    # result. Catch it here, before it is written into the receipt and spends two rollouts.
    if ".." in rel.parts:
        raise J10R1Error(f"the relative path for {p} under {root} escapes the leaf: {rel}")
    return str(rel).replace(os.sep, "/")


# ================================================================ inputs =========================

def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J10R1Error("the J10R1 preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J10R1Error(f"the J10R1 preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage_proposed") != STAGE:
        raise J10R1Error(f"the preregistration proposes {data.get('stage_proposed')!r}, not {STAGE}")
    local: dict[str, str] = {}
    for rel, pin in data["pinned_artefacts_sha256"].items():
        got = _sha_file(HERE / rel)
        if got != pin:
            raise J10R1Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        local[rel] = got
    repo: dict[str, str] = {}
    for rel, pin in data["pinned_repo_artefacts_sha256"].items():
        got = _sha_file(REPO / rel)
        if got != pin:
            raise J10R1Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        repo[rel] = got
    declared = data["matrix"]["cell_list"]
    if [c["id"] for c in declared] != [c["id"] for c in MATRIX]:
        raise J10R1Error("the preregistered matrix order and the runner's disagree")
    for want, got in zip(MATRIX, declared):
        if float(got["episode_start_offset_s"]) != float(want["offset_s"]):
            raise J10R1Error(f"cell {want['id']} offset differs from the preregistration")
    if int(data["matrix"]["seed"]) != ROLLOUT_SEED \
            or int(data["matrix"]["steps_per_cell"]) != EXPECTED_STEPS \
            or int(data["matrix"]["cells"]) != len(MATRIX):
        raise J10R1Error("the preregistration and the runner disagree on seed, steps or cell count")
    if data["matrix"]["behavioural_fail_fast"] is not False:
        raise J10R1Error("the preregistration must declare behavioural_fail_fast false")
    if float(data["label_source"]["lookahead_s"]) != TEACHER_LOOKAHEAD_S \
            or [float(v) for v in data["label_source"]["sigma"]] != list(SIGMA):
        raise J10R1Error("the preregistration and the runner disagree on lookahead or sigma")
    if data["coverage_gate"]["binding"] is not True \
            or data["coverage_gate"]["scope"].split(",")[0] != "the COLLECTION AS A WHOLE":
        raise J10R1Error("the coverage gate must be declared binding and collection-wide")
    # the rejected J10 bundle is immutable evidence: hash it, do not merely cite it
    rejected_here: dict[str, str] = {}
    rejected_repo: dict[str, str] = {}
    for rel, pin in data["rejected_j10_bundle_sha256"].items():
        q = HERE / rel
        if not q.is_file():
            raise J10R1Error(f"the rejected J10 artefact {rel} is missing; it is immutable "
                             f"evidence and must be preserved byte-identical")
        got = _sha_file(q)
        if got != pin:
            raise J10R1Error(f"the rejected J10 artefact {rel} was MODIFIED: {got} != {pin}. "
                             f"The rejected bundle is preserved byte-identical and never edited.")
        rejected_here[rel] = got
    for rel, pin in data["supersedes_rejected_j10"]["rejected_readiness_report_sha256"].items():
        q = REPO / rel
        if not q.is_file():
            raise J10R1Error(f"the rejected J10 readiness report {rel} is missing")
        got = _sha_file(q)
        if got != pin:
            raise J10R1Error(f"the rejected J10 readiness report was MODIFIED: {got} != {pin}")
        if rel != PIN_REJECTED_J10_REPORT_REL:
            raise J10R1Error(f"the rejected readiness report is recorded as {rel}, not the pinned "
                             f"{PIN_REJECTED_J10_REPORT_REL}")
        rejected_repo[rel] = got
    if data["supersedes_rejected_j10"]["rejected_bundle_sha256"] \
            != data["rejected_j10_bundle_sha256"]:
        raise J10R1Error("the rejection record and the enforced pin table disagree about the "
                         "rejected J10 bundle; two copies that can drift are not a record")
    if data["supersedes_rejected_j10"]["rejected_stage"] == STAGE:
        raise J10R1Error("the successor must not carry the rejected stage token")

    # the two provenance corrections, asserted rather than assumed
    if "valid_cycle_count" in data["diagnostics_not_binding"] \
            or "action_clipped_steps" in data["diagnostics_not_binding"]:
        raise J10R1Error("a binding measure is listed among the non-binding diagnostics")
    return {"file": _rel(PREREG), "sha256": digest,
            "manifest_entries": len(local) + len(repo),
            "pinned_artefacts_sha256": local, "pinned_repo_artefacts_sha256": repo,
            "supersedes": data["supersedes_rejected_j10"]["rejected_stage"],
            "rejected_j10_bundle_verified_byte_identical": {
                "resolved_against_the_validation_dir": rejected_here,
                "resolved_against_the_repository_root": rejected_repo,
                "why_split": "the two sets live under different roots; merging them would leave "
                             "an auditor unable to re-resolve either"},
            "rejected_j10_never_executed": True}


def verify_parent() -> dict[str, Any]:
    """The August V26 imitative actor: J1's own check, plus an explicit J8-student refusal."""
    parent = J1.verify_parent()
    if parent["module_state_sha256"] != PIN_PARENT_STATE_SHA:
        raise J10R1Error(f"the parent is {parent['module_state_sha256']}, not "
                       f"{PIN_PARENT_STATE_SHA}")
    if str(J8_STUDENT_DIR.resolve()) == str(PARENT_DIR.resolve()):
        raise J10R1Error("the J8 student is not a permitted parent for this stage")
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
        raise J10R1Error(f"the runtime actor exposes {len(names)} feature names, expected "
                       f"{ACTOR_WIDTH}")
    if COVERAGE_FEATURE not in names:
        raise J10R1Error(f"the coverage feature {COVERAGE_FEATURE!r} is absent from the runtime "
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
        raise J10R1Error("base_env_config requires a REAL output root; None disables recording")
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_root)
    J1.verify_env_config(env, cfg)
    if float(env["episode_start_offset_s"]) != OFFSET_NOMINAL:
        raise J10R1Error(f"the pinned config's offset is {env['episode_start_offset_s']}, expected "
                       f"the nominal {OFFSET_NOMINAL}")
    for flag in ("record_outputs", "save_outputs_on_close"):
        if env.get(flag) is not True:
            raise J10R1Error(f"{flag} is {env.get(flag)!r}, expected exactly True")
    if not env.get("output_prefix") or not env.get("output_dir"):
        raise J10R1Error("the base env config carries no output_prefix or no output_dir")
    return env, cfg


def cell_env_config(base: Mapping[str, Any], offset_s: float,
                    output_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Mutate ONLY episode_start_offset_s and output_dir, and PROVE nothing else moved."""
    if float(offset_s) not in [float(v) for v in FROZEN_OFFSETS]:
        raise J10R1Error(f"{offset_s} is not one of the two frozen offsets {FROZEN_OFFSETS}")
    env = dict(base)
    env["episode_start_offset_s"] = float(offset_s)
    env["output_dir"] = str(output_dir)
    allowed = {"episode_start_offset_s", "output_dir"}
    differing = sorted(k for k in set(base) | set(env)
                       if base.get(k, "<absent>") != env.get(k, "<absent>"))
    forbidden = sorted(set(differing) - allowed)
    if forbidden:
        raise J10R1Error(f"the cell config mutates {forbidden}; only the offset and output_dir may "
                       "differ from the verified base")
    added = sorted(set(env) - set(base))
    removed = sorted(set(base) - set(env))
    if removed or sorted(set(added) - {"output_dir"}):
        raise J10R1Error(f"the cell config adds {added} and removes {removed}")
    # STABLE KEYS: every key that is not one of the two declared-mutable ones must be equal.
    stable_keys = set(base) - allowed
    differing_stable = sorted(k for k in stable_keys if base[k] != env.get(k, "<absent>"))
    if differing_stable:
        raise J10R1Error(f"these stable fields differ from the verified base: {differing_stable}")
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
        raise J10R1Error("the teacher module exposes no prescribed_teacher_action")
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
        "resolved_by": "feature names of the LIVE RUNTIME ENVIRONMENT "
                       "(base.actor_feature_names), resolved by name at run time",
        "not_resolved_from": "the parent's actor_feature_manifest.json, which declares the "
                             "39-wide June-lineage slice and is NEVER used for this index",
        "runtime_width": ACTOR_WIDTH,
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
            raise J10R1Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J10R1Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J10R1Error("--collect requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J10R1Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J10R1Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J10R1Error(f"the authorised leaf already exists; this stage is no-clobber and "
                       f"single-execution: {leaf}")
    return leaf


# ================================================================ one cell =======================

def run_cell(cell: Mapping[str, Any], *, stack: _Stack, base_env: Mapping[str, Any],
             cfg: Mapping[str, Any], contract: Mapping[str, Any], staging: Path,
             progress: bool = True) -> dict[str, Any]:
    """ONE prescribed-teacher rollout. Builds, resets, steps and CLOSES exactly one environment."""
    cid = str(cell["id"])
    sim_out = staging / f"j10r1_cell_{cid}_sim_outputs"
    env_config, mutation = cell_env_config(base_env, float(cell["offset_s"]), sim_out)

    np.random.seed(ROLLOUT_SEED)
    env = stack.make_env(env_config)
    env_closed = False
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=ROLLOUT_SEED)

        sim_cfg = getattr(base, "cfg", None)
        if sim_cfg is None or not hasattr(sim_cfg, "t_start") or not hasattr(sim_cfg, "t_end"):
            raise J10R1Error(f"cell {cid}: the environment exposes no cfg.t_start / cfg.t_end")
        env_cfg_live = getattr(base, "env_cfg", None)
        if env_cfg_live is None or not hasattr(env_cfg_live, "episode_duration"):
            raise J10R1Error(f"cell {cid}: the environment exposes no env_cfg.episode_duration")
        live_duration = getattr(env_cfg_live, "episode_duration")
        declared = env_config.get("episode_duration")
        if (live_duration is None) != (declared is None):
            raise J10R1Error(f"cell {cid}: live and declared episode_duration disagree on presence")
        if live_duration is not None and abs(float(live_duration) - float(declared)) > 1e-12:
            raise J10R1Error(f"cell {cid}: live episode_duration {live_duration} differs from the "
                           f"declared {declared} by more than 1e-12")
        reset_expect = expected_reset_time(float(sim_cfg.t_start), float(sim_cfg.t_end),
                                           None if live_duration is None else float(live_duration),
                                           float(cell["offset_s"]))
        reset_time = float(J1._finite(base.t, f"cell {cid}: reset time"))
        reset_error = abs(reset_time - reset_expect["expected_reset_time_s"])
        if reset_error > RESET_TIME_TOLERANCE_S:
            raise J10R1Error(f"cell {cid}: reset time {reset_time} differs from the expected "
                           f"{reset_expect['expected_reset_time_s']} by {reset_error} s")

        # the RUNTIME feature names, from the live env - never the 39-wide parent manifest
        names = runtime_feature_names(base)
        cov_idx = coverage_index(names)
        n_actor = int(getattr(base, "n_actor", ACTOR_WIDTH))
        if n_actor != ACTOR_WIDTH:
            raise J10R1Error(f"cell {cid}: the env exposes n_actor {n_actor}, expected {ACTOR_WIDTH}")
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
                raise J10R1Error(f"cell {cid} step {step}: the actor observation is "
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
                raise J10R1Error(f"cell {cid} step {step}: sigma is zero, so the noise must be "
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
                raise J10R1Error(f"cell {cid} step {step}: info exposes no 'time'")
            terms = J1._jsonable(info.get("reward_terms", {}), "reward_terms")
            if "grf_penetration_m" not in terms:
                raise J10R1Error(f"cell {cid} step {step}: reward_terms carries no "
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
            raise J10R1Error(f"cell {cid}: the environment failed to close cleanly, so the cell is "
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
        raise J10R1Error("the summariser reports a non-zero sigma; this stage injects none")
    penetration = penetration_report(pen_arr, contract, f"J10 cell {cid}")
    gate = evaluate_cell_gate(summary, knee_arr, ankle_arr, penetration)
    integrity = J3.telemetry_integrity(summary)
    verdict = J3.overall_verdict(gate, integrity)

    # (2) EXACTLY 19 REGULAR FILES. Counted after close(), because close() is what writes them.
    if not sim_out.is_dir():
        raise J10R1Error(f"cell {cid}: the sim_outputs directory is missing: {sim_out}")
    entries = sorted(sim_out.iterdir())
    sim_files = sorted((q for q in entries if q.is_file() and not q.is_symlink()),
                       key=lambda q: q.name)
    non_files = sorted(q.name for q in entries if not (q.is_file() and not q.is_symlink()))
    if non_files:
        raise J10R1Error(f"cell {cid}: sim_outputs holds non-regular entries {non_files}")
    if len(sim_files) != EXPECTED_SIM_OUTPUT_FILES:
        raise J10R1Error(
            f"cell {cid}: sim_outputs holds {len(sim_files)} regular files, expected EXACTLY "
            f"{EXPECTED_SIM_OUTPUT_FILES}. Fewer means a writer failed silently; more means "
            f"something else wrote into the directory. Names: "
            f"{[q.name for q in sim_files]}")

    ds = staging / f"j10r1_cell_{cid}_teacher_dataset.npz"
    np.savez_compressed(ds, observations=obs_arr, actions=act_arr,
                        executed_actions=exe_arr, action_noises=noi_arr, times=tim_arr,
                        actor_feature_names=np.asarray(names, dtype=str))
    tr = staging / f"j10r1_cell_{cid}_trace.json"
    tr.write_text(json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
    kn = staging / f"j10r1_cell_{cid}_kinematics.npz"
    np.savez_compressed(kn, knee_rad=knee_arr, ankle_rad=ankle_arr,
                        actor_feature_names=np.asarray(names, dtype=str))
    pn = staging / f"j10r1_cell_{cid}_penetration.npz"
    np.savez_compressed(pn, penetration_m=pen_arr)

    # (3) SHA-256 of every artefact this cell produced, keyed by LEAF-RELATIVE path, computed
    # BEFORE the receipt is written so the receipt can carry them.
    artefact_sha: dict[str, str] = {}
    for q in (ds, tr, kn, pn):
        artefact_sha[_leaf_rel(q, staging)] = _sha_file(q)
    sim_output_sha: dict[str, str] = {}
    for q in sim_files:
        sim_output_sha[_leaf_rel(q, staging)] = _sha_file(q)
    if len(sim_output_sha) != EXPECTED_SIM_OUTPUT_FILES:
        raise J10R1Error(f"cell {cid}: hashed {len(sim_output_sha)} sim_outputs, expected "
                         f"{EXPECTED_SIM_OUTPUT_FILES}")
    dataset_sha = artefact_sha[_leaf_rel(ds, staging)]

    return {
        "id": cid, "label": str(cell["label"]),
        "episode_start_offset_s": float(cell["offset_s"]), "offset_unit": OFFSET_UNIT,
        "seed": ROLLOUT_SEED,
        "env_mutation": mutation,
        "env_config_sha256_excluding_output_dir": _sha_obj(
            {k: v for k, v in env_config.items() if k != "output_dir"}),
        "env_config_hash_excludes": ["output_dir"],
        "why_output_dir_is_excluded": (
            "output_dir is the STAGING path, which is deliberately never recorded and ceases to "
            "exist at the rename. Hashing it would make the digest unreproducible by "
            "construction. Excluding it makes the digest verifiable, and the directory it named "
            "is recorded separately as a leaf-relative path."),
        "output_dir_leaf_relative": _leaf_rel(Path(env_config["output_dir"]), staging),
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
        "artefact_sha256": artefact_sha,
        "sim_outputs_sha256": sim_output_sha,
        "sim_outputs_file_names": [q.name for q in sim_files],
        "sim_outputs_regular_file_count": len(sim_files),
        "sim_outputs_expected_count": EXPECTED_SIM_OUTPUT_FILES,
        "sim_outputs_count_is_exact": True,
        "teacher_dataset_sha256": dataset_sha,
        "teacher_dataset_sha256_is_binding_for_j11": (
            "the future J11 fit must verify THIS hash before consuming the dataset"),
        "artefacts": {"teacher_dataset": _leaf_rel(ds, staging),
                      "trace": _leaf_rel(tr, staging),
                      "kinematics": _leaf_rel(kn, staging),
                      "penetration": _leaf_rel(pn, staging),
                      "sim_outputs": _leaf_rel(sim_out, staging),
                      "sim_outputs_file_count": len(sim_files)},
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
        raise J10R1Error(f"the penetration contract changed: {contract['sha256']}")
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
                        "expected_reset_time_s": None,
                        "why_not_computed_here": (
                            "cfg.t_start and cfg.t_end come from the model setup XML and exist "
                            "only once an environment has been constructed, which this preflight "
                            "is forbidden to do. Publishing a number from hard-coded values would "
                            "be false precision: it would agree with the run only by luck, and "
                            "would diverge silently the moment episode_duration clamped the "
                            "start. The authoritative check is made at run time in run_cell, "
                            "against the LIVE cfg.t_start and cfg.t_end, to 1e-9 s."),
                        "arithmetic": RESET_SEMANTICS,
                        "offset_is_relative_to_t_start": True})
    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J10R1 lock is already held or was left behind: {lock_path}")
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
            "expected_sim_outputs_per_cell": EXPECTED_SIM_OUTPUT_FILES,
            "count_is_exact_not_a_lower_bound": True},
        "preflight_sentinel": {"root": _rel(PREFLIGHT_SENTINEL),
                               "existed_before": sentinel_before, "exists_after": sentinel_after,
                               "created_by_the_preflight": False, "measured_not_assumed": True},
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256_excluding_output_dir": _sha_obj(
                        {k: v for k, v in env.items() if k != "output_dir"}),
                    "env_config_hash_excludes": ["output_dir"],
                    "comparable_with_each_cell": (
                        "this is the BASE config, hashed the same way as each cell's. Because the "
                        "only permitted mutations are episode_start_offset_s and output_dir, a "
                        "cell digest may differ from this one ONLY through the offset."),
                    "built_against": "the inert preflight sentinel root, which is never created",
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
                        "per_cell": ["j10r1_cell_<ID>_teacher_dataset.npz",
                                     "j10r1_cell_<ID>_trace.json",
                                     "j10r1_cell_<ID>_kinematics.npz",
                                     "j10r1_cell_<ID>_penetration.npz",
                                     "j10r1_cell_<ID>_sim_outputs/"],
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

# ================================================ (4) post-commit verification ==================

def _resolve_inside(leaf: Path, rel: str) -> Path:
    """Resolve a leaf-relative path and PROVE it stays inside the committed leaf.

    A recorded path that escapes the leaf (``..``, an absolute string, a symlink pointing out)
    is an integrity failure, not a lookup miss.
    """
    if not rel or Path(rel).is_absolute() or ".." in Path(rel).parts:
        raise J10R1Error(f"recorded path is not leaf-relative: {rel!r}")
    q = leaf / rel
    if q.is_symlink():
        raise J10R1Error(f"recorded path is a symlink, which is never committed here: {rel}")
    root = leaf.resolve()
    try:
        q.resolve().relative_to(root)
    except ValueError:
        raise J10R1Error(f"recorded path escapes the committed leaf: {rel}") from None
    return q


def verify_committed_leaf(leaf: Path, cells: list[dict[str, Any]] | None = None, *,
                          expected_receipt_sha: str | None = None) -> dict[str, Any]:
    """Re-resolve EVERY recorded leaf-relative path in the COMMITTED leaf and recompute EVERY hash.

    This runs after ``os.rename``, because only then do the recorded paths mean what they claim.
    It is not a formality: a path that does not resolve, or a hash that does not reproduce, means
    the committed content is not the content that was measured, and the leaf is TECHNICALLY
    INVALID whatever the behavioural verdict said.
    """
    # Read the cell blocks back out of the COMMITTED receipt rather than trusting the in-memory
    # copy. What must be verified is what the receipt actually RECORDS: if serialisation dropped
    # or altered a path or a hash, verifying against memory would not notice.
    receipt = leaf / RECEIPT_NAME
    verified_against = "the committed receipt"
    # ONE read. The digest that is reported, the digest that is compared and the bytes that are
    # parsed must all be the same bytes: hashing and re-reading separately would let a receipt
    # altered in between be certified by a digest taken before the change.
    receipt_bytes = receipt.read_bytes() if receipt.is_file() else None
    receipt_sha = hashlib.sha256(receipt_bytes).hexdigest() if receipt_bytes is not None else None
    receipt_matches_staging: bool | None = None
    if expected_receipt_sha is not None:
        # The receipt is the file the verifier TRUSTS, so it is also the file that most needs
        # checking: a corrupted receipt would otherwise validate itself.
        receipt_matches_staging = receipt_sha == expected_receipt_sha
    if receipt_bytes is not None:
        cells = json.loads(receipt_bytes.decode("utf-8"))["cells"]
    elif cells is None:
        raise J10R1Error(f"the committed receipt is missing and no cells were supplied: {receipt}")
    else:
        verified_against = "the in-memory cells (the committed receipt is MISSING)"

    # A receipt whose cell list is empty or short would otherwise verify nothing and be declared
    # valid. The matrix is the contract: every cell must be present, in the frozen order.
    if [c.get("id") for c in cells] != [m["id"] for m in MATRIX]:
        raise J10R1Error(
            f"the committed receipt records cells {[c.get('id') for c in cells]}, but the frozen "
            f"matrix is {[m['id'] for m in MATRIX]}. A receipt that lost a cell cannot be "
            f"verified into validity.")

    checked_files = 0
    checked_dirs = 0
    mismatches: list[dict[str, str]] = []
    missing: list[str] = []
    recomputed: dict[str, dict[str, str]] = {}

    for c in cells:
        cid = c["id"]
        per_cell: dict[str, str] = {}
        expected = dict(c["artefact_sha256"])
        expected.update(c["sim_outputs_sha256"])
        if len(c["sim_outputs_sha256"]) != EXPECTED_SIM_OUTPUT_FILES:
            raise J10R1Error(f"cell {cid}: the receipt carries "
                             f"{len(c['sim_outputs_sha256'])} sim_output hashes, expected "
                             f"{EXPECTED_SIM_OUTPUT_FILES}")

        # every artefact path the cell block advertises must also be hashed
        for key in ("teacher_dataset", "trace", "kinematics", "penetration"):
            rel = c["artefacts"][key]
            if rel not in c["artefact_sha256"]:
                raise J10R1Error(f"cell {cid}: artefact {key} is advertised at {rel} but carries "
                                 f"no hash")

        for rel, want in sorted(expected.items()):
            q = _resolve_inside(leaf, rel)
            if not q.is_file():
                missing.append(rel)
                continue
            got = _sha_file(q)
            per_cell[rel] = got
            checked_files += 1
            if got != want:
                mismatches.append({"path": rel, "expected": want, "recomputed": got})

        # the sim_outputs DIRECTORY must still hold exactly the 19 regular files, by name
        sd = _resolve_inside(leaf, c["artefacts"]["sim_outputs"])
        if not sd.is_dir():
            missing.append(c["artefacts"]["sim_outputs"])
        else:
            checked_dirs += 1
            listing = sorted(sd.iterdir())          # ONE listing, classified twice
            present = sorted(q.name for q in listing if q.is_file() and not q.is_symlink())
            extra = sorted(q.name for q in listing if not (q.is_file() and not q.is_symlink()))
            if extra:
                mismatches.append({"path": c["artefacts"]["sim_outputs"],
                                   "expected": "regular files only",
                                   "recomputed": f"non-regular entries {extra}"})
            if len(present) != EXPECTED_SIM_OUTPUT_FILES:
                mismatches.append({"path": c["artefacts"]["sim_outputs"],
                                   "expected": f"{EXPECTED_SIM_OUTPUT_FILES} regular files",
                                   "recomputed": f"{len(present)} regular files"})
            if present != sorted(c["sim_outputs_file_names"]):
                mismatches.append({"path": c["artefacts"]["sim_outputs"],
                                   "expected": "the recorded file names",
                                   "recomputed": f"{present}"})
        recomputed[cid] = per_cell

    if receipt_bytes is None:
        missing.append(RECEIPT_NAME)
    if receipt_matches_staging is False:
        mismatches.append({"path": RECEIPT_NAME, "expected": str(expected_receipt_sha),
                           "recomputed": str(receipt_sha)})
    ok = not mismatches and not missing
    return {
        "schema": "v26c_j10r1_commit_verification.1",
        "stage": STAGE,
        "when": "AFTER os.rename, against the COMMITTED leaf",
        "verified_against": verified_against,
        "cells_verified": len(cells),
        "cells_expected": len(MATRIX),
        "receipt_matches_staging_bytes": receipt_matches_staging,
        "pass": ok,
        "files_checked": checked_files,
        "directories_checked": checked_dirs,
        "expected_files_per_cell": EXPECTED_SIM_OUTPUT_FILES + 4,
        "paths_missing": missing,
        "hash_mismatches": mismatches,
        "recomputed_sha256": recomputed,
        # measured here because a receipt can never carry its own hash, and taken from the SAME
        # bytes that were parsed and compared above
        "receipt_sha256": receipt_sha,
        "meaning": ("the leaf is VALID EVIDENCE if and only if this file exists and pass is true. "
                    "Its absence, or pass false, marks the leaf TECHNICALLY INVALID: the committed "
                    "content is not the content that was measured, so nothing in it may be "
                    "consumed, promoted or cited."),
        "on_failure": ("preserved fail-closed and NOT promoted. No deletion, no retry, no repair: "
                       "a corrupted commit is evidence about the commit."),
    }


def collect(out_arg: str | None, stage_token: str | None, *, stack: _Stack | None = None,
            progress: bool = True) -> dict[str, Any]:
    """Both cells, in the frozen order, with NO behavioural fail-fast."""
    if OUTPUT_ROOT_OVERRIDE is not None and stack is None:
        raise J10R1Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. It is permitted "
                       f"only for synthetic, isolated tests.")
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    pre = preflight()
    if pre["blockers"]:
        raise J10R1Error(f"preflight BLOCKED: {pre['blockers']}")

    contract = PC.load_contract()
    parent_before = verify_parent()

    injected = stack is not None
    stack = stack if stack is not None else production_stack()

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J10R1Error(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    staging_created: Path | None = None
    commit_verified = False
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise J10R1Error(f"the J10R1 lock already exists: {lock_path}. This stage fails closed "
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
                raise J10R1Error(f"the base about to be used has {flag}={base_env.get(flag)!r}; "
                               "refusing before constructing any environment")

        cells: list[dict[str, Any]] = []
        for cell in MATRIX:
            # NO behavioural fail-fast: both cells run. A technical exception still fails closed.
            cells.append(run_cell(cell, stack=stack, base_env=base_env, cfg=cfg,
                                  contract=contract, staging=staging, progress=progress))

        parent_after = verify_parent()
        if parent_after["module_state_sha256"] != parent_before["module_state_sha256"]:
            raise J10R1Error("the operative parent changed during the collection")

        coverage = evaluate_coverage(cells)
        behavioural = sum(1 for c in cells if c["behavioural_pass"])
        valid = sum(1 for c in cells if c["telemetry_valid"])
        aggregate_pass = (behavioural == len(MATRIX) and valid == len(MATRIX)
                          and bool(coverage["pass"]))
        verdict = ("PASS" if aggregate_pass
                   else ("INVALID" if valid != len(MATRIX) else "FAIL"))

        receipt = {
            "schema": "v26c_j10r1_multistart_teacher_receipt.1", "stage": STAGE,
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
            "commit_verification": {
                "file": COMMIT_VERIFICATION_NAME,
                "state_when_this_receipt_was_written": "PENDING",
                "why_pending": ("this receipt is written into the staging directory, BEFORE the "
                                "rename. The recorded leaf-relative paths cannot be resolved, and "
                                "their hashes cannot be re-verified, until the leaf exists."),
                "validity_rule": ("this leaf is VALID EVIDENCE if and only if "
                                  f"{COMMIT_VERIFICATION_NAME} exists beside this receipt and "
                                  "declares pass true. If it is absent, or declares pass false, "
                                  "the leaf is TECHNICALLY INVALID and nothing in it may be "
                                  "consumed, promoted or cited - whatever verdict this receipt "
                                  "carries."),
                "marker_on_failure": TECHNICAL_INVALID_NAME},
            "provenance": {
                "artefact_paths_are_leaf_relative": True,
                "which_paths": ("every path this run PRODUCED - the four per-cell artefacts, the "
                                "sim_outputs directory and its 19 files - is relative to the "
                                "committed leaf and resolves inside it"),
                "repo_relative_fields": [
                    "preregistration.file", "preregistration.pinned_repo_artefacts_sha256 keys",
                    "parent_before.path", "parent_after.path",
                    "penetration_authority.contract"],
                "why_those_differ": ("those name INPUTS that live in the repository, not outputs "
                                     "of this run. They are repository-relative and must be "
                                     "resolved against the repository root, never against the "
                                     "leaf."),
                "no_staging_path_recorded": True,
                "why": "the J9R1 receipt recorded absolute staging paths that ceased to exist at "
                       "the commit; no path here is a staging path",
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
        # measured on the STAGING receipt, so the post-commit check can prove the committed
        # receipt is byte-identical to the one that was written
        staging_receipt_sha = _sha_file(staging / RECEIPT_NAME)

        # THE LEAF IS BORN INVALID. The marker is written into the staging directory BEFORE the
        # rename, so it travels with the commit. There is therefore NO instant at which a leaf
        # exists without an explicit statement that it is unverified - not even if the process is
        # killed between the rename and the verification. The marker is removed LAST, only after
        # the verification has passed.
        (staging / TECHNICAL_INVALID_NAME).write_text(
            "TECHNICALLY INVALID - UNVERIFIED\n"
            f"stage: {STAGE}\n"
            f"see: {COMMIT_VERIFICATION_NAME}\n"
            "This marker is written BEFORE the commit and removed only after the post-commit "
            "verification has passed. While it is present the leaf is NOT valid evidence: it is "
            "either unverified or verified-and-failed. Nothing in it may be consumed, promoted "
            "or cited.\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J10R1Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        # The marker must have travelled with the rename. If it did not, this leaf is not the
        # directory this run built, whatever else is inside it.
        marker = leaf / TECHNICAL_INVALID_NAME
        if not marker.is_file():
            raise J10R1Error(
                f"the committed leaf does not carry the pre-commit invalidity marker "
                f"{TECHNICAL_INVALID_NAME}: {leaf} is not the directory this run staged")

        # (4) POST-COMMIT VERIFICATION. The rename is atomic, but atomicity says nothing about
        # whether the committed bytes are the bytes that were measured. Verify now. The leaf is
        # already marked invalid, so a failure here - or a crash here - can never leave an
        # apparently-valid leaf behind.
        try:
            verification = verify_committed_leaf(leaf, cells,
                                                 expected_receipt_sha=staging_receipt_sha)
        except Exception as exc:                         # the verifier itself failed to complete
            # BaseException is deliberately NOT caught: a Ctrl-C or a SystemExit must propagate
            # with the leaf left marked invalid, not be recorded as a verification verdict.
            verification = {"schema": "v26c_j10r1_commit_verification.1", "stage": STAGE,
                            "pass": False, "verifier_error": f"{type(exc).__name__}: {exc}",
                            "meaning": "verification could not complete; the leaf is TECHNICALLY "
                                       "INVALID and is preserved unpromoted"}
        try:
            (leaf / COMMIT_VERIFICATION_NAME).write_text(
                json.dumps(verification, indent=2, ensure_ascii=False, allow_nan=False,
                           default=str) + "\n", encoding="utf-8")
        except OSError as exc:
            # The very fault class this verification exists to detect. The leaf stays marked.
            raise J10R1Error(
                f"the verification could not be recorded in {leaf}: {type(exc).__name__}: {exc}. "
                f"The leaf remains marked {TECHNICAL_INVALID_NAME} and is preserved "
                f"fail-closed.") from exc
        if not verification.get("pass"):
            try:
                marker.write_text(
                    "TECHNICALLY INVALID - VERIFICATION FAILED\n"
                    f"stage: {STAGE}\n"
                    f"see: {COMMIT_VERIFICATION_NAME}\n"
                    "The committed content did not reproduce the paths and hashes recorded in "
                    "the receipt. This leaf is preserved as evidence about the commit. It is NOT "
                    "a dataset, it is NOT promotable and nothing in it may be consumed.\n",
                    encoding="utf-8")
            except OSError:
                pass                                     # the birth marker already says invalid
            raise J10R1Error(
                f"POST-COMMIT VERIFICATION FAILED for {leaf}: "
                f"{len(verification.get('paths_missing') or [])} recorded paths did not resolve, "
                f"{len(verification.get('hash_mismatches') or [])} hashes did not reproduce. "
                f"The leaf is marked {TECHNICAL_INVALID_NAME} and preserved fail-closed.")
        # PASSED. Only now is the birth marker removed, and it is the LAST write of the commit.
        try:
            marker.unlink()
        except OSError as exc:
            raise J10R1Error(
                f"the verification passed but the invalidity marker could not be removed from "
                f"{leaf}: {type(exc).__name__}: {exc}. The leaf therefore still reads as "
                f"TECHNICALLY INVALID, which is the fail-closed outcome.") from exc
        commit_verified = True
    except BaseException:
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            try:
                shutil.rmtree(staging_created)
            except OSError as _rm:                       # Windows: an open handle blocks removal
                print(f"WARNING: the staging directory could not be removed: "
                      f"{staging_created}: {type(_rm).__name__}: {_rm}. It will BLOCK the next "
                      f"run and must be removed by hand; this stage never repairs itself.",
                      file=sys.stderr)
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
            "commit_verification": {
                "file": COMMIT_VERIFICATION_NAME, "pass": commit_verified,
                "files_checked": verification.get("files_checked"),
                "directories_checked": verification.get("directories_checked"),
                "rule": "the leaf is valid evidence only while this is true"},
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "fit_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J10R1 multistart prescribed-teacher collection")
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
        raise J10R1Error("--out is meaningless without --collect; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
