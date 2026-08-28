"""V26B stage B1 - BASE phase fit of the masked 35D student. TOOLING ONLY.

Supervised IK imitation of the B0 actor while the controller/Markov channel stays hard-masked.
This module builds the dataset, the fail-closed split, the masked training protocol and the
gates.  It performs no fit, no training, no rollout and no collection: `run_b1` refuses.

The superseded 25D modules are NEVER imported: every threshold that branch declared is
re-declared here so the quarantine holds by construction.

Dataset (no new collection is needed): the three PINNED V26 anchor trajectories
A_ISO39CLK_V3 - nominal, -0.20 s and +0.20 s - 500 rows each, paired with the prescribed
AB06 u_IK labels from the pinned privileged caches by EXACT time lookup.  These are V26
lineage; S0D, S1A and every REV4*/V2_* artifact is forbidden as parent or data.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_b0_masked35 as B0  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class B1Error(RuntimeError):
    pass


STAGE = "V26B-B1-BASE-FIT-MASKED35"
OUT_DIR = VA.OUT_ROOT / "candidates" / "B1_BASE35_MASKED"
RECEIPT_NAME = "v26b_b1_receipt.json"
EXPECTED_ROWS_PER_TRAJECTORY = 500
EXPECTED_TRAJECTORIES = 3

# --- preregistered closed-loop gates (re-declared here; the 25D branch is not imported) ---------
REFERENCE = {"knee": {"min": -1.0165, "max": -0.1640, "amplitude": 0.8525},
             "ankle": {"min": -0.1552, "max": +0.4002, "amplitude": 0.5553,
                       "negative_row_fraction": 0.1940}}
ANKLE_MIN_RAD = -0.03
ANKLE_AMPLITUDE_MIN_RAD = 0.30
KNEE_AMPLITUDE_MIN_RAD = 0.60
COMPLETION_STEPS = 500
MIN_VALID_CYCLES = 2
PENETRATION_MAX_M = 0.020
MAX_RESYNC = 1
CRITICAL_COUNTERS = ("phase_timeout_stance", "phase_timeout_swing",
                     "morphology_causal_contract_failure", "hs_cancelled_count")
GATE_MOTIVATION = {
    "ankle_min_rad": ("the prosthetic ankle must genuinely plantarflex, not merely approach zero. "
                      "-0.03 rad is 19.3% of the reference plantarflexion depth 0.1552 rad. A "
                      "minimum of -0.0099 rad DOES NOT satisfy it: -0.0099 > -0.03, and it is only "
                      "33% of the threshold in magnitude"),
    "ankle_amplitude_min_rad": "54.0% of the reference ankle amplitude 0.5553 rad",
    "knee_amplitude_min_rad": "70.4% of the reference knee amplitude 0.8525 rad",
    "penetration": "the CURRENT v3 soft guard 0.020 m, unchanged. No guard is relaxed anywhere",
}
SIGMA_VERIFICATION = {
    "status": "OPEN - sigma 0.005 is NOT assumed",
    "rule": "before ANY stochastic recovery is collected for the Markov phase, the exploration "
            "scale of the base actor must be MEASURED by comparing the empirical per-joint action "
            "noise RMS of stochastic rollouts against the nominal rollout of the SAME actor",
    "precedent": "on 2026-07-13 traces labelled sigma003 carried an empirical noise of about 0.03 "
                 "and were discarded; the corrected trace measured 0.005103 and 0.004554",
}


# ------------------------------------------------------------------ dataset ---------------------

def build_dataset() -> dict[str, Any]:
    """(masked obs35, u_IK) pairs from the three pinned V26 anchors. No collection is performed."""
    rep = R.verify_anchor_pins()
    if not rep.get("all_match"):
        raise B1Error("pinned deterministic anchors do not match their content digests")
    obs_blocks: list[np.ndarray] = []
    act_blocks: list[np.ndarray] = []
    traj: list[int] = []
    per_trajectory: dict[str, Any] = {}
    for tid, (start, spec) in enumerate(sorted(R.ANCHORS.items())):
        job = Path(spec["job_dir"])
        B0.assert_not_forbidden(job, f"anchor {start}")
        tr = DS.trajectory_from_job(job, expected_width=B0.ACTOR_WIDTH)
        obs = np.asarray(tr["obs35"], dtype=np.float32)
        t = np.asarray(tr["t_pre"], dtype=np.float64)
        if obs.shape != (EXPECTED_ROWS_PER_TRAJECTORY, B0.ACTOR_WIDTH):
            raise B1Error(f"anchor {start}: obs {obs.shape}, expected "
                          f"({EXPECTED_ROWS_PER_TRAJECTORY}, {B0.ACTOR_WIDTH})")
        cache = L.load_cache(R.OUT_CACHE, start)
        idx = cache.lookup(t)                       # exact float equality or it raises
        u = np.asarray(cache.ik_action, dtype=np.float32)[idx]
        if u.shape != (obs.shape[0], R.ACTION_DIM):
            raise B1Error(f"anchor {start}: labels {u.shape}")
        if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(u))):
            raise B1Error(f"anchor {start}: non-finite data")
        if not np.all(np.abs(u) <= 1.0 + 1e-6):
            raise B1Error(f"anchor {start}: labels outside [-1, 1]")
        obs_blocks.append(obs); act_blocks.append(u); traj.extend([tid] * obs.shape[0])
        per_trajectory[start] = {"trajectory_id": tid, "rows": int(obs.shape[0]),
                                 "job_dir": C.rel(job),
                                 "adapter_trace_sha256": spec["adapter_trace_sha256"],
                                 "cache_digest": cache.digest()}
    obs = np.concatenate(obs_blocks, axis=0)
    act = np.concatenate(act_blocks, axis=0)
    if obs.shape[0] != EXPECTED_TRAJECTORIES * EXPECTED_ROWS_PER_TRAJECTORY:
        raise B1Error(f"dataset has {obs.shape[0]} rows, expected "
                      f"{EXPECTED_TRAJECTORIES * EXPECTED_ROWS_PER_TRAJECTORY}")
    masked = B0.apply_input_mask(obs)
    if np.array_equal(masked, obs):
        raise B1Error("masking changed nothing: the anchors carry no controller-state signal")
    return {"observations_masked": masked, "observations_raw": obs, "actions": act,
            "trajectory_id": np.asarray(traj, dtype=np.int64),
            "report": {"rows": int(obs.shape[0]), "trajectories": EXPECTED_TRAJECTORIES,
                       "per_trajectory": per_trajectory,
                       "label_rule": "prescribed AB06 u_IK at the anchor's own times, exact float "
                                     "lookup on the pinned privileged cache; no interpolation",
                       "collection_performed": False,
                       "provenance": "three pinned V26 anchor trajectories (A_ISO39CLK_V3); no "
                                     "S0D, S1A, REV4* or V2_* row"}}


def trajectory_split(traj: Sequence[int]) -> dict[str, Any]:
    """Fail-closed leave-one-trajectory-out. Never random: whole trajectories are held out, so no
    temporal neighbour of a held-out row can appear in training."""
    ids = np.asarray(traj).reshape(-1)
    uniq = np.unique(ids)
    if uniq.size < 2:
        raise B1Error("a trajectory split needs at least two trajectories")
    folds = []
    for u in uniq:
        hold = np.where(ids == u)[0]
        keep = np.where(ids != u)[0]
        if hold.size == 0 or keep.size == 0:
            raise B1Error(f"degenerate fold for trajectory {int(u)}")
        if np.intersect1d(hold, keep).size:
            raise B1Error("train and holdout overlap")
        folds.append({"holdout_trajectory": int(u), "holdout_rows": int(hold.size),
                      "train_rows": int(keep.size)})
    return {"mode": "leave_one_trajectory_out", "trajectories": int(uniq.size), "folds": folds,
            "rationale": "consecutive steps of one trajectory are strongly dependent; a random "
                         "split leaks the neighbours of every held-out row into training"}


# ------------------------------------------------------------------ masked protocol -------------

def masked_training_protocol() -> dict[str, Any]:
    """The exact sequence B1 must follow at every optimizer step. Declared, not executed."""
    return {"per_step": [
                "1. mask the input batch: obs[:, masked_columns] = 0",
                "2. forward and backward",
                "3. zero the gradient on the masked and clock first-layer columns (defensive: with "
                "masked inputs that gradient is already identically zero)",
                "4. optimizer step",
                "5. restore: first-layer masked and clock columns set back to exactly zero",
                "6. assert_no_masked_update(before, after): bit-identical AND still exactly zero",
                "7. restore the frozen log-std head",
            ],
            "frozen": ["log-std head", "critic (never touched)", "the ten masked first-layer columns",
                        "the two clock first-layer columns"],
            "no_ppo": True,
            "why_two_guarantees": "the input mask makes the gradient on those columns identically "
                                  "zero, so the restore is an independent second guard; a leak in "
                                  "either mechanism is detected by step 6"}


def offline_gates(init_state: Mapping[str, Any], fitted_state: Mapping[str, Any],
                  obs_masked: np.ndarray, actions: np.ndarray) -> dict[str, Any]:
    """Binding offline gates for B1. IK metrics are reported without an invented threshold."""
    integrity = {
        "ten_keys": tuple(fitted_state.keys()) == RF.EXPECTED_KEY_ORDER,
        "width_35": int(np.asarray(fitted_state["pi.0.0.weight"]).shape[1]) == B0.ACTOR_WIDTH,
        "no_critic": True,
    }
    try:
        B0.assert_masked_columns_zero(fitted_state, "B1 fitted")
        integrity["masked_columns_zero"] = True
    except B0.B0Error:
        integrity["masked_columns_zero"] = False
    try:
        B0.assert_clock_columns_zero(fitted_state, "B1 fitted")
        integrity["clock_columns_zero"] = True
    except B0.B0Error:
        integrity["clock_columns_zero"] = False
    integrity["logstd_byte_identical_to_init"] = bool(
        np.array_equal(np.asarray(fitted_state["pi.1.weight"])[R.ACTION_DIM:],
                       np.asarray(init_state["pi.1.weight"])[R.ACTION_DIM:])
        and np.array_equal(np.asarray(fitted_state["pi.1.bias"])[R.ACTION_DIM:],
                           np.asarray(init_state["pi.1.bias"])[R.ACTION_DIM:]))
    try:
        B0.assert_no_masked_update(init_state, fitted_state)
        integrity["no_masked_update"] = True
    except B0.B0Error:
        integrity["no_masked_update"] = False
    m0 = RF.numpy_mean(dict(init_state), obs_masked)
    m1 = RF.numpy_mean(dict(fitted_state), obs_masked)
    before = float(np.sqrt(np.mean((m0 - actions) ** 2)))
    after = float(np.sqrt(np.mean((m1 - actions) ** 2)))
    equiv = B0.functional_equivalence_25(fitted_state, obs_masked)
    binding = {
        "integrity_invariants": {"binding": True, **integrity, "pass": bool(all(integrity.values()))},
        "functional_equivalence_25": {"binding": True, **equiv, "pass": bool(equiv["bit_identical"])},
        "fit_convergence": {"binding": True, "rmse_before": before, "rmse_after": after,
                            "pass": bool(after < before)},
    }
    per_joint = {jn: {"rmse": float(np.sqrt(np.mean((m1[:, j] - actions[:, j]) ** 2))),
                      "max_abs": float(np.max(np.abs(m1[:, j] - actions[:, j])))}
                 for j, jn in ((0, "knee"), (1, "ankle"))}
    return {"binding": binding, "failed": [k for k, v in binding.items() if not v["pass"]],
            "all_binding_pass": all(v["pass"] for v in binding.values()),
            "ik_metrics_reported_without_threshold": {"aggregate_rmse": after, "per_joint": per_joint}}


def kinematic_gates(knee_q: Sequence[float], ankle_q: Sequence[float]) -> dict[str, Any]:
    k = np.asarray(knee_q, dtype=float); a = np.asarray(ankle_q, dtype=float)
    if k.size == 0 or a.size == 0:
        raise B1Error("empty kinematic series")
    g = {"ankle_plantarflexion": {"observed_min_rad": float(a.min()), "threshold_rad": ANKLE_MIN_RAD,
                                  "rule": "observed_min <= -0.03", "pass": bool(a.min() <= ANKLE_MIN_RAD)},
         "ankle_amplitude": {"observed_rad": float(a.max() - a.min()),
                             "threshold_rad": ANKLE_AMPLITUDE_MIN_RAD,
                             "pass": bool((a.max() - a.min()) >= ANKLE_AMPLITUDE_MIN_RAD)},
         "knee_amplitude": {"observed_rad": float(k.max() - k.min()),
                            "threshold_rad": KNEE_AMPLITUDE_MIN_RAD,
                            "pass": bool((k.max() - k.min()) >= KNEE_AMPLITUDE_MIN_RAD)},
         "knee_stays_flexed": {"fraction_negative": float((k < 0.0).mean()), "pass": bool(np.all(k < 0.0))},
         "within_bounds": {"knee_outside": int(np.sum((k < RO.KNEE_BOUNDS[0]) | (k > RO.KNEE_BOUNDS[1]))),
                           "ankle_outside": int(np.sum((a < RO.ANKLE_BOUNDS[0]) | (a > RO.ANKLE_BOUNDS[1]))),
                           "pass": bool(np.all((k >= RO.KNEE_BOUNDS[0]) & (k <= RO.KNEE_BOUNDS[1]))
                                        and np.all((a >= RO.ANKLE_BOUNDS[0]) & (a <= RO.ANKLE_BOUNDS[1])))}}
    failed = [n for n, v in g.items() if not v["pass"]]
    return {"gates": g, "failed": failed, "all_pass": not failed,
            "reference": REFERENCE, "motivation": GATE_MOTIVATION}


def declared_closed_loop_gates() -> dict[str, Any]:
    return {"completion": {"steps": COMPLETION_STEPS, "end_reason": "episode_time_limit"},
            "valid_cycle_count_min": MIN_VALID_CYCLES,
            "critical_counters_zero": list(CRITICAL_COUNTERS),
            "resync_count_max": MAX_RESYNC,
            "penetration_max_m": PENETRATION_MAX_M,
            "kinematic_quality": {"ankle_min_rad": ANKLE_MIN_RAD,
                                  "ankle_amplitude_min_rad": ANKLE_AMPLITUDE_MIN_RAD,
                                  "knee_amplitude_min_rad": KNEE_AMPLITUDE_MIN_RAD,
                                  "knee_strictly_flexed": True, "within_bounds": True},
            "motivation": GATE_MOTIVATION,
            "no_guard_relaxed": "every threshold is the current v3 value or stricter; FSM v3, the "
                                "morphology corridor, the contact model, the reward and every "
                                "safety guard are untouched"}


def markov_phase_declared() -> dict[str, Any]:
    """Declared only. Nothing here is designed in detail: the architect preregisters it later."""
    return {"trigger": "ONLY after the base gates pass",
            "operation": "remove the input mask; the ten first-layer columns are still exactly "
                          "zero, so the transition is bit-exact by construction",
            "dataset": ["nominal anchors of the base actor, labelled with the actor's OWN means",
                         "same-actor phase-consistent recoveries",
                         "multistart, only if separately authorised"],
            "frozen": ["log-std", "critic"],
            "whole_mean_network_adaptation": "NOT decided now. It is preregistered at that stage",
            "interpolation": "not part of this declaration",
            "sigma": SIGMA_VERIFICATION}


def preflight() -> dict[str, Any]:
    if any(m in sys.modules for m in B0.SUPERSEDED_MODULES):
        raise B1Error("a superseded 25D module is imported; the quarantine forbids it")
    b0 = B0.preflight()
    data = build_dataset()
    split = trajectory_split(data["trajectory_id"])
    state, _ = B0.build_b0_state()
    equiv = B0.functional_equivalence_25(state, data["observations_raw"])
    m0 = RF.numpy_mean(dict(state), data["observations_masked"])
    rmse0 = float(np.sqrt(np.mean((m0 - data["actions"]) ** 2)))
    blockers: list[str] = []
    if OUT_DIR.exists():
        blockers.append(f"no-clobber: {C.rel(OUT_DIR)} already exists")
    return {"verdict": ("GO" if not blockers else "BLOCKED"), "stage": STAGE, "blockers": blockers,
            "b0": {"verdict": b0["verdict"], "actor_digest": b0["transplant"]["b0_actor_digest"],
                   "functional_equivalence_25": b0["functional_equivalence_25"]["bit_identical"],
                   "bit_exact_unmask_transition": b0["bit_exact_unmask_transition"]["bit_identical"]},
            "dataset": data["report"],
            "split": split,
            "functional_equivalence_25_on_full_dataset": equiv,
            "initial_rmse_of_b0_on_the_dataset_diagnostic": rmse0,
            "masked_training_protocol": masked_training_protocol(),
            "declared_closed_loop_gates": declared_closed_loop_gates(),
            "markov_phase_declared": markov_phase_declared(),
            "sigma": SIGMA_VERIFICATION}


def run_b1(*, authorized_stage: str | None) -> dict[str, Any]:
    raise B1Error(f"B1 is TOOLING ONLY in this stage: no fit, training, rollout or collection is "
                  f"authorised (requested token {authorized_stage!r})")


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1: masked 35D base fit (tooling and preflight only)")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    a = p.parse_args(argv)
    if a.authorized_stage is not None:
        run_b1(authorized_stage=a.authorized_stage)
    print(json.dumps(preflight(), indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
