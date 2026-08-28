"""V26B stage A1 - IK imitation of the 25D intermediate actor. TOOLING ONLY.

This module builds the A1 dataset, the fail-closed split, the offline gates and the
DECLARED closed-loop gates.  It performs no fit, no training, no rollout and no
collection: `run_a1` refuses to run without an authorization token that is not yet issued.

Parent chain (exclusive):  August V26 39D 5bbc6cbd...  ->  A0 25D transplant  ->  A1.
S0D, S1A and every REV4* artifact are DIAGNOSTIC EVIDENCE ONLY: they may be read for
calibration tables, never used as init, anchor or training data.  No July artifact.

A1a (BLOCKING PREREQUISITE, separately authorised): a prescribed-teacher nominal rollout
under the 25D contract, producing teacher_dataset.npz + teacher_summary.json through the
production entry point target_domain_imitation.collect_teacher_dataset.  A1 fails closed
while that artifact is absent - the existing corpus is S0D-visited and is forbidden here.

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

import v26b_a0_transplant25 as A0  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_rev4b_dagger as B4  # noqa: E402   (frozen: teacher IK view, DIAGNOSTIC use only here)
import v26b_r0a_rollout as RO  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402


class A1Error(RuntimeError):
    pass


STAGE = "V26B-A1-IK-IMITATION"
A1A_STAGE = "V26B-A1A-TEACHER-COLLECTION-25D"
TEACHER_DIR = VA.OUT_ROOT / "datasets" / "A1A_TEACHER_25D"
TEACHER_NPZ = TEACHER_DIR / "teacher_dataset.npz"
TEACHER_SUMMARY = TEACHER_DIR / "teacher_summary.json"
OUT_DIR = VA.OUT_ROOT / "candidates" / "A1_IK25_INTERMEDIATE"
RECEIPT_NAME = "v26b_a1_receipt.json"

# --- fail-closed split -------------------------------------------------------------------------
SPLIT_FOLDS = 5
SPLIT_EMBARGO_STEPS = 10
VALIDATION_FRACTION = 0.20

# --- preregistered closed-loop gates, motivated on the AB06 prosthetic IK reference -------------
# Reference measured on the frozen 500-row AB06 corpus (diagnostic read of the label cache):
#   knee   min -1.0165  max -0.1640  amplitude 0.8525   (always flexed, i.e. negative)
#   ankle  min -0.1552  max +0.4002  amplitude 0.5553   (19.40% of rows plantarflexed)
REFERENCE = {"knee": {"min": -1.0165, "max": -0.1640, "amplitude": 0.8525},
             "ankle": {"min": -0.1552, "max": +0.4002, "amplitude": 0.5553,
                       "negative_row_fraction": 0.1940}}

ANKLE_MIN_RAD = -0.03          # <= this value. 19.3% of the reference plantarflexion depth.
ANKLE_AMPLITUDE_MIN_RAD = 0.30  # >= this value. 54.0% of the reference ankle amplitude.
KNEE_AMPLITUDE_MIN_RAD = 0.60   # >= this value. 70.4% of the reference knee amplitude.
COMPLETION_STEPS = 500
MIN_VALID_CYCLES = 2
PENETRATION_MAX_M = 0.020       # the current v3 soft guard, unchanged
MAX_RESYNC = 1

GATE_MOTIVATION = {
    "ankle_min_rad": ("the prosthetic ankle must genuinely plantarflex, not merely approach zero. "
                      "-0.03 rad is 19.3% of the reference plantarflexion depth 0.1552 rad, far "
                      "above any numerical noise. A minimum of -0.0099 rad does NOT satisfy it: "
                      "-0.0099 > -0.03, and it is only 33% of the threshold in magnitude"),
    "ankle_amplitude_min_rad": ("54.0% of the reference ankle amplitude 0.5553 rad: enough to "
                                "exclude a joint that merely oscillates around a fixed offset"),
    "knee_amplitude_min_rad": ("70.4% of the reference knee amplitude 0.8525 rad: a walking knee "
                               "sweeps most of the reference range; a collapsed knee does not"),
    "completion": "500/500 with end_reason episode_time_limit, the same bar as every prior stage",
    "valid_cycles": "at least 2, the promotion bar already in force; 1 cycle proved insufficient",
    "penetration": "the CURRENT v3 soft guard 0.020 m, unchanged. No guard is relaxed anywhere",
    "critical_counters": "phase_timeout stance and swing, morphology causal contract failure and "
                         "hs_cancelled must all be zero; resync at most 1",
}

SIGMA_VERIFICATION = {
    "status": "OPEN - sigma 0.005 is NOT assumed",
    "rule": "before ANY stochastic recovery is collected for A3, the exploration scale of the A1 "
            "actor must be MEASURED: record one nominal deterministic rollout and stochastic "
            "rollouts of the SAME actor, then compare the empirical per-joint action noise RMS "
            "against the serialised placeholder",
    "precedent": "on 2026-07-13 traces labelled sigma003 were found to carry an empirical noise of "
                 "about 0.03 and were discarded; the corrected trace measured 0.005103 / 0.004554",
    "consequence": "a mismatch between the serialised placeholder and the measured noise invalidates "
                   "the recovery dataset before it is built",
}


# ============================================================ dataset and split ==================

def require_a1a() -> dict[str, Any]:
    """Fail closed until the A1a teacher collection under the 25D contract exists."""
    if not TEACHER_NPZ.is_file() or not TEACHER_SUMMARY.is_file():
        raise A1Error(
            f"A1a is missing: {C.rel(TEACHER_DIR)} must contain teacher_dataset.npz and "
            f"teacher_summary.json produced under the 25D contract by "
            f"target_domain_imitation.collect_teacher_dataset (stage {A1A_STAGE}, separately "
            "authorised). The existing 500-row corpus is S0D-visited and is FORBIDDEN as A1 data")
    summary = json.loads(TEACHER_SUMMARY.read_text(encoding="utf-8"))
    if not bool(summary.get("gate_pass")):
        raise A1Error("the A1a teacher run did not pass its own production gate")
    with np.load(TEACHER_NPZ) as z:
        data = {k: np.asarray(z[k]).copy() for k in z.files}
    return {"summary": summary, "data": data}


def assert_dataset_contract_25(data: Mapping[str, np.ndarray]) -> dict[str, Any]:
    obs = np.asarray(data["observations"]); act = np.asarray(data["actions"])
    names = [str(x) for x in np.asarray(data["actor_feature_names"], dtype=str)]
    ct = A0.contract_25d()
    if names != list(ct["names25"]):
        raise A1Error("the teacher dataset feature names are not the pinned 25D contract")
    if obs.ndim != 2 or obs.shape[1] != A0.N_ACTOR_25:
        raise A1Error(f"observations must be (N,25), got {obs.shape}")
    if act.shape != (obs.shape[0], R.ACTION_DIM):
        raise A1Error(f"actions must be (N,2), got {act.shape}")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise A1Error("non-finite values in the A1 dataset")
    if not np.all(np.abs(act) <= 1.0 + 1e-6):
        raise A1Error("prescribed actions outside the normalised [-1, 1] range")
    if not np.all(obs[:, list(A0.CLOCK_COLUMNS)] == obs[0, list(A0.CLOCK_COLUMNS)]):
        raise A1Error("the disabled clock columns are not constant in the dataset")
    return {"rows": int(obs.shape[0]), "width": int(obs.shape[1]),
            "feature_names": names, "actions_within_bounds": True}


def blocked_temporal_split(n: int, folds: int = SPLIT_FOLDS,
                           embargo: int = SPLIT_EMBARGO_STEPS) -> dict[str, Any]:
    """Contiguous blocked split with a symmetric embargo. NEVER random: consecutive steps of one
    trajectory are strongly dependent, and a random split leaks the neighbours of every held-out
    row into training."""
    if n < folds * (2 * embargo + 2):
        raise A1Error(f"{n} rows cannot support {folds} folds with an embargo of {embargo}")
    edges = np.linspace(0, n, folds + 1).astype(int)
    out = []
    for f in range(folds):
        lo, hi = int(edges[f]), int(edges[f + 1])
        hold = np.arange(lo, hi)
        keep = np.concatenate([np.arange(0, max(0, lo - embargo)),
                               np.arange(min(n, hi + embargo), n)])
        if keep.size == 0 or hold.size == 0:
            raise A1Error(f"fold {f} degenerate")
        if np.intersect1d(hold, keep).size:
            raise A1Error(f"fold {f}: embargo failed, train and holdout overlap")
        out.append({"fold": f, "holdout": [int(lo), int(hi)], "train_rows": int(keep.size),
                    "holdout_rows": int(hold.size)})
    return {"mode": "blocked_temporal_with_embargo", "folds": folds, "embargo_steps": embargo,
            "rows": n, "per_fold": out,
            "rationale": "consecutive steps are strongly dependent; a random split leaks "
                         "neighbouring rows into training and inflates the offline score"}


def trajectory_split(traj_ids: Sequence[int]) -> dict[str, Any]:
    ids = np.asarray(traj_ids)
    uniq = np.unique(ids)
    if uniq.size < 2:
        raise A1Error("a trajectory split needs at least two trajectories")
    return {"mode": "by_trajectory", "trajectories": int(uniq.size),
            "per_trajectory_rows": {int(u): int((ids == u).sum()) for u in uniq},
            "rationale": "whole trajectories are held out, so no temporal neighbour leaks"}


def build_split(data: Mapping[str, np.ndarray]) -> dict[str, Any]:
    """Fail-closed choice: by trajectory when more than one exists, otherwise blocked temporal."""
    n = int(np.asarray(data["observations"]).shape[0])
    ids = data.get("trajectory_id")
    if ids is not None and np.unique(np.asarray(ids)).size >= 2:
        return trajectory_split(np.asarray(ids).reshape(-1))
    return blocked_temporal_split(n)


# ============================================================ gates =============================

def offline_gates(init_state: Mapping[str, Any], fitted_state: Mapping[str, Any],
                  obs: np.ndarray, actions: np.ndarray) -> dict[str, Any]:
    """Binding offline gates. Reported values only; no invented threshold on the IK metrics."""
    import f2r_refit as RF
    integrity = {
        "ten_keys": tuple(fitted_state.keys()) == RF.EXPECTED_KEY_ORDER,
        "width_25": int(np.asarray(fitted_state["pi.0.0.weight"]).shape[1]) == A0.N_ACTOR_25,
        "clock_columns_zero": bool(np.all(np.asarray(fitted_state["pi.0.0.weight"])[:, list(A0.CLOCK_COLUMNS)] == 0.0)),
        "logstd_byte_identical_to_init": bool(
            np.array_equal(np.asarray(fitted_state["pi.1.weight"])[R.ACTION_DIM:],
                           np.asarray(init_state["pi.1.weight"])[R.ACTION_DIM:])
            and np.array_equal(np.asarray(fitted_state["pi.1.bias"])[R.ACTION_DIM:],
                               np.asarray(init_state["pi.1.bias"])[R.ACTION_DIM:])),
        "no_critic": True,
    }
    m0 = RF.numpy_mean(init_state, obs.astype(np.float32))
    m1 = RF.numpy_mean(fitted_state, obs.astype(np.float32))
    before = float(np.sqrt(np.mean((m0 - actions) ** 2)))
    after = float(np.sqrt(np.mean((m1 - actions) ** 2)))
    per_joint = {jn: {"rmse": float(np.sqrt(np.mean((m1[:, j] - actions[:, j]) ** 2))),
                      "max_abs": float(np.max(np.abs(m1[:, j] - actions[:, j])))}
                 for j, jn in ((0, "knee"), (1, "ankle"))}
    binding = {"integrity_invariants": {"binding": True, **integrity, "pass": bool(all(integrity.values()))},
               "fit_convergence": {"binding": True, "rmse_before": before, "rmse_after": after,
                                   "pass": bool(after < before)}}
    return {"binding": binding, "failed": [k for k, v in binding.items() if not v["pass"]],
            "all_binding_pass": all(v["pass"] for v in binding.values()),
            "ik_metrics_reported_without_threshold": {"aggregate_rmse": after, "per_joint": per_joint}}


def kinematic_gates(knee_q: Sequence[float], ankle_q: Sequence[float]) -> dict[str, Any]:
    """Preregistered biological kinematic quality, motivated on the AB06 reference."""
    k = np.asarray(knee_q, dtype=float); a = np.asarray(ankle_q, dtype=float)
    if k.size == 0 or a.size == 0:
        raise A1Error("empty kinematic series")
    k_amp = float(k.max() - k.min()); a_amp = float(a.max() - a.min())
    g = {
        "ankle_plantarflexion": {"observed_min_rad": float(a.min()), "threshold_rad": ANKLE_MIN_RAD,
                                 "rule": "observed_min <= -0.03",
                                 "pass": bool(a.min() <= ANKLE_MIN_RAD)},
        "ankle_amplitude": {"observed_rad": a_amp, "threshold_rad": ANKLE_AMPLITUDE_MIN_RAD,
                            "pass": bool(a_amp >= ANKLE_AMPLITUDE_MIN_RAD)},
        "knee_amplitude": {"observed_rad": k_amp, "threshold_rad": KNEE_AMPLITUDE_MIN_RAD,
                           "pass": bool(k_amp >= KNEE_AMPLITUDE_MIN_RAD)},
        "knee_stays_flexed": {"fraction_negative": float((k < 0.0).mean()),
                              "pass": bool(np.all(k < 0.0))},
        "within_bounds": {"knee_outside": int(np.sum((k < RO.KNEE_BOUNDS[0]) | (k > RO.KNEE_BOUNDS[1]))),
                          "ankle_outside": int(np.sum((a < RO.ANKLE_BOUNDS[0]) | (a > RO.ANKLE_BOUNDS[1]))),
                          "pass": bool(np.all((k >= RO.KNEE_BOUNDS[0]) & (k <= RO.KNEE_BOUNDS[1]))
                                       and np.all((a >= RO.ANKLE_BOUNDS[0]) & (a <= RO.ANKLE_BOUNDS[1])))},
    }
    failed = [n for n, v in g.items() if not v["pass"]]
    return {"gates": g, "failed": failed, "all_pass": not failed,
            "reference": REFERENCE, "motivation": GATE_MOTIVATION}


def declared_closed_loop_gates() -> dict[str, Any]:
    """The A1 rollout gates, declared now and frozen by the prereg. The rollout itself is a later,
    separately authorised stage; nothing here launches it."""
    return {"completion": {"steps": COMPLETION_STEPS, "end_reason": "episode_time_limit"},
            "valid_cycle_count_min": MIN_VALID_CYCLES,
            "critical_counters_zero": ["phase_timeout_stance", "phase_timeout_swing",
                                       "morphology_causal_contract_failure", "hs_cancelled_count"],
            "resync_count_max": MAX_RESYNC,
            "penetration_max_m": PENETRATION_MAX_M,
            "kinematic_quality": {"ankle_min_rad": ANKLE_MIN_RAD,
                                  "ankle_amplitude_min_rad": ANKLE_AMPLITUDE_MIN_RAD,
                                  "knee_amplitude_min_rad": KNEE_AMPLITUDE_MIN_RAD,
                                  "knee_strictly_flexed": True,
                                  "within_bounds": True},
            "motivation": GATE_MOTIVATION,
            "no_guard_relaxed": "every threshold is the current v3 value or stricter; nothing in "
                                "FSM v3, the morphology corridor, the contact model, the reward or "
                                "any safety guard is modified"}


def calibration_against_existing_actors() -> dict[str, Any]:
    """DIAGNOSTIC ONLY: how the preregistered kinematic gates score the existing 35D actors.
    These artifacts are never init, anchor or data of this branch; they are read to show the
    thresholds discriminate."""
    jobs = {"S0D": VA.OUT_ROOT / "rollouts" / "s0d_nominal_det" / "S0D_35D__v3_canonical__nominal__det",
            "S1A": VA.OUT_ROOT / "rollouts" / "s1a_nominal_det" / "S1A_35D__v3_canonical__nominal__det",
            "REV4E": VA.OUT_ROOT / "rollouts" / "rev4e_nominal_det" / "REV4E_35D__v3_canonical__nominal__det"}
    out: dict[str, Any] = {"note": "diagnostic evidence only; never data of this branch"}
    for tag, job in jobs.items():
        if not job.is_dir():
            continue
        o = np.asarray(DS.trajectory_from_job(job, expected_width=R.ENV_ACTOR_WIDTH)["obs35"], float)
        res = kinematic_gates(o[:, RO.IDX_KNEE_Q], o[:, RO.IDX_ANKLE_Q])
        out[tag] = {"ankle_min": res["gates"]["ankle_plantarflexion"]["observed_min_rad"],
                    "ankle_amplitude": res["gates"]["ankle_amplitude"]["observed_rad"],
                    "knee_amplitude": res["gates"]["knee_amplitude"]["observed_rad"],
                    "failed_gates": res["failed"], "would_pass": res["all_pass"]}
    return out


# ============================================================ preflight =========================

def preflight() -> dict[str, Any]:
    """No-write, fail-closed. Reports what exists and what blocks A1."""
    a0 = A0.preflight()
    blockers: list[str] = []
    teacher: dict[str, Any] = {}
    try:
        got = require_a1a()
        teacher = {"present": True, "contract": assert_dataset_contract_25(got["data"]),
                   "split": build_split(got["data"]),
                   "summary_gate_pass": bool(got["summary"].get("gate_pass"))}
    except A1Error as exc:
        teacher = {"present": False, "reason": str(exc)}
        blockers.append(f"A1a missing: {A1A_STAGE}")
    if OUT_DIR.exists():
        blockers.append(f"no-clobber: {C.rel(OUT_DIR)} already exists")
    split_demo = blocked_temporal_split(500)
    return {"verdict": ("GO" if not blockers else "BLOCKED"), "stage": STAGE, "blockers": blockers,
            "a0_verdict": a0["verdict"], "a0_new_actor_digest": a0["transplant"]["new_actor_digest"],
            "contract_25d_sha256": a0["contract"]["contract_yaml_sha256"],
            "a1a_prerequisite": {"stage": A1A_STAGE, "expected_dir": C.rel(TEACHER_DIR),
                                 "producer": "target_domain_imitation.collect_teacher_dataset "
                                             "under the 25D contract",
                                 "why": "the existing 500-row corpus is S0D-visited and is "
                                        "forbidden as data of this branch"},
            "teacher_dataset": teacher,
            "split_policy_demonstration_on_500_rows": split_demo,
            "declared_closed_loop_gates": declared_closed_loop_gates(),
            "kinematic_gate_calibration": calibration_against_existing_actors(),
            "sigma_verification": SIGMA_VERIFICATION}


def run_a1(*, authorized_stage: str | None) -> dict[str, Any]:
    """Refuses to run: A1 is tooling-only until the architect issues the token."""
    raise A1Error(f"A1 is TOOLING ONLY in this stage. No fit, training, rollout or collection is "
                  f"authorised (requested token {authorized_stage!r}); A1a must be produced first")


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B A1: 25D IK imitation (tooling and preflight only)")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    a = p.parse_args(argv)
    if a.authorized_stage is not None:
        run_a1(authorized_stage=a.authorized_stage)
    print(json.dumps(preflight(), indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
