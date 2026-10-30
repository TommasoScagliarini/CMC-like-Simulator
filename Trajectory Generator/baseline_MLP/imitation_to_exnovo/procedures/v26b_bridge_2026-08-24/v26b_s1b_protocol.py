"""V26B rev3s — S1B anchored bridge: PROTOCOL / CORPUS BUILDER / GATE DEFINITIONS (validation only).

Token V26B-S1B-ANCHORED-PROTOCOL.  This module builds and verifies the S1B corpus, proves the
source==init property of the S0D nominal trace, reproduces the analytic predictions frozen in
rev3s and defines the offline hierarchy and the (separately tokened) closed-loop gate.

It NEVER fits, NEVER launches an episode and NEVER collects data: the corresponding entry
points exist only as fail-closed guards.  S1A is never init, source or anchor.

Classification: SUPERVISED ANCHORED BRIDGE, not DAgger (no new on-policy collection, no teacher
relabelling of newly visited states).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1a_rollout as SA  # noqa: E402      (rev3r lineage; S1A is quarantined, never used as source)
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402          (frozen task view / split, unmodified)
import v26b_s0d_rollout as SR  # noqa: E402      (S0D trace job, read-only)
import v26b_r0a_rollout as RO  # noqa: E402      (July discrete-feature rule)
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class S1BError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1B-ANCHORED-PROTOCOL"
AMENDMENT_REV3S = HERE / "v26b_amendment_rev3s_s1b_anchored_protocol.json"
PIN_AMENDMENT_REV3S = "89e8c227eecb2a80350307d6c0315e0f90e42b37ecae1d75fea30f9da2e067a0"
FUTURE_FIT_TOKEN = "V26B-S1B-FIT"
FUTURE_ROLLOUT_TOKEN = "V26B-S1B-NOMINAL-ROLLOUT"
FUTURE_COLLECTION_TOKEN = "V26B-S0D-ALTSTART-COLLECTION"

PIN_S0D_ACTOR = FIT.PIN_S0D_ACTOR
PIN_S0D_ROLLOUT_RECEIPT = "cbec1a671b7cdf2980881ec4ca33f69534e66d94cbbfa86c7b9cd9c1a39412b7"
FORBIDDEN_SOURCES = {"S1A": A.P0_ACTOR_DIGEST}
SOURCE_EQ_INIT_TOL = 1e-6
OUT_DRY = VA.OUT_ROOT / "s1b_protocol"


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV3S)
    if got != PIN_AMENDMENT_REV3S:
        raise S1BError(f"rev3s sha {got} != pinned")
    return json.loads(AMENDMENT_REV3S.read_text(encoding="utf-8"))


# --- lineage ---------------------------------------------------------------------------------------

def verify_lineage_s1b() -> dict[str, Any]:
    lin = SA.verify_lineage_rollout()      # rev3l..rev3r + candidate/production pins, all immutable
    lin["amendment_rev3s"] = C.sha256_file(AMENDMENT_REV3S)
    if lin["amendment_rev3s"] != PIN_AMENDMENT_REV3S:
        raise S1BError("rev3s sha != pinned")
    s1a_receipt = json.loads((SA.JOB_DIR / SA.RECEIPT_NAME).read_text(encoding="utf-8"))
    if s1a_receipt["status"] != "NOT_ELIGIBLE_FAIL":
        raise S1BError("the S1A rollout receipt no longer records NOT_ELIGIBLE_FAIL")
    lin["s1a_status"] = s1a_receipt["status"]
    lin["s1a_quarantined"] = "S1A is never init, source or anchor of S1B"
    return lin


# --- source == init ----------------------------------------------------------------------------------

def verify_source_equals_init() -> dict[str, Any]:
    """The available complete S0D trace is that actor's OWN nominal deterministic 500-step trace."""
    receipt_path = SR.JOB_DIR / SR.RECEIPT_NAME
    got = C.sha256_file(receipt_path)
    if got != PIN_S0D_ROLLOUT_RECEIPT:
        raise S1BError(f"S0D rollout receipt sha {got} != pinned")
    rec = json.loads(receipt_path.read_text(encoding="utf-8"))
    if rec["status"] != "COMPLETE" or not rec["analysis"]["completion"]["complete_500_time_limit"]:
        raise S1BError("the S0D trace is not a complete 500/500 episode_time_limit run")
    cmd = rec["command"]
    ckpt = cmd[cmd.index("--checkpoint") + 1]
    if Path(ckpt).resolve() != SR.S0D_MODULE.resolve():
        raise S1BError(f"the trace was produced by {ckpt}, not by the pinned S0D module")
    if cmd[cmd.index("--action-selection") + 1] != "deterministic" or cmd[cmd.index("--seed") + 1] != str(R.DET_SEED):
        raise S1BError("the trace is not the deterministic seed-123 nominal run")
    traj = DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    if traj["trace_sha256"] != rec["analysis"]["trace_sha256"]:
        raise S1BError("S0D trace digest != receipt")
    obs = np.asarray(traj["obs35"], dtype=np.float32)
    recorded = np.asarray(traj["b_raw_action"], dtype=np.float64)
    if obs.shape[0] != 500:
        raise S1BError(f"trace rows {obs.shape[0]} != 500")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SR.S0D_MODULE).items()}
    digest = W.actor_state_digest(init)
    if digest != PIN_S0D_ACTOR:
        raise S1BError(f"init digest {digest} != pinned S0D")
    if digest in FORBIDDEN_SOURCES.values():
        raise S1BError("forbidden source actor")
    fwd = RF.numpy_mean(init, obs)
    dev = float(np.max(np.abs(fwd - recorded)))
    if not dev <= SOURCE_EQ_INIT_TOL:
        raise S1BError(f"source==init NOT proven: max deviation {dev:.3e} > {SOURCE_EQ_INIT_TOL:.0e}")
    return {"rollout_receipt_sha256": got, "trace_sha256": traj["trace_sha256"], "rows": int(obs.shape[0]),
            "completion": rec["analysis"]["completion"], "checkpoint": C.rel(Path(ckpt)),
            "init_actor_digest": digest,
            "numpy_forward_vs_recorded": {"max_abs": dev, "rmse": float(np.sqrt(np.mean((fwd - recorded) ** 2))),
                                          "tolerance": SOURCE_EQ_INIT_TOL,
                                          "july_homolog": "13/07 source==init proven at float32 rmse 1.19e-07"},
            "verdict": "the recorded raw actions ARE this actor's deterministic mean on those states: anchors are source==init"}


# --- corpus ------------------------------------------------------------------------------------------

def build_corpus() -> dict[str, Any]:
    """Anchor role (u_S0D) and task role (u_IK) on the SAME frozen 380-row training mask;
    the frozen holdout 201-300 stays unseen by BOTH roles."""
    view = FIT.build_s1_task()          # frozen split, fail-closed leakage checks inside
    traj = DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    own = np.asarray(traj["b_raw_action"], dtype=np.float64)
    if not np.array_equal(own, view["u_own"]):
        raise S1BError("the trace actions differ from the frozen task view's u_own")
    tr, ho, em = view["train"], view["hold"], view["embargo"]
    step = np.arange(1, 501)
    for lo, hi in FIT.TRAIN_STEPS_EXCLUDED:
        if np.any((step[tr] >= lo) & (step[tr] <= hi)):
            raise S1BError(f"LEAKAGE: training rows inside [{lo},{hi}]")
    if int(tr.sum()) != 380 or int(ho.sum()) != 100 or int(em.sum()) != 20:
        raise S1BError("frozen split is not 380/100/20")
    obs = view["obs"]
    anchor = {"rows": int(tr.sum()), "labels": "u_S0D own recorded actions",
              "obs_sha256": hashlib.sha256(obs[tr].tobytes()).hexdigest(),
              "labels_sha256": hashlib.sha256(own[tr].tobytes()).hexdigest()}
    task = {"rows": int(tr.sum()), "labels": "AB06 u_IK same-time",
            "obs_sha256": anchor["obs_sha256"],
            "labels_sha256": hashlib.sha256(view["u_ik"][tr].tobytes()).hexdigest()}
    if anchor["labels_sha256"] == task["labels_sha256"]:
        raise S1BError("anchor and task labels are identical: the corpus would be degenerate")
    strata = discrete_strata(obs[tr])
    return {"view": view, "own": own, "anchor": anchor, "task": task, "strata": strata,
            "identical_inputs": True,
            "holdout": {"steps": [201, 300], "rows": int(ho.sum()), "unseen_by_both_roles": True,
                        "obs_sha256": hashlib.sha256(obs[ho].tobytes()).hexdigest()},
            "embargo_rows": int(em.sum())}


def discrete_strata(obs_train: np.ndarray) -> dict[str, Any]:
    """Preservation strata from the July discrete observation components (FSM one-hot, contact,
    heel-strike / toe-off flags): every stratum present in the anchor rows is measured separately."""
    idx = RO.discrete_feature_indices(R.FEATURE_NAMES_35)
    out = {}
    for i in idx:
        name = R.FEATURE_NAMES_35[i]
        m = obs_train[:, i] > 0.5
        out[name] = {"index": int(i), "rows": int(m.sum()), "present": bool(m.any())}
    return {"rule": "July discrete observation components; stratum = rows where the flag is active",
            "features": out, "absent_strata": [k for k, v in out.items() if not v["present"]]}


def gap_stats(corpus: Mapping[str, Any]) -> dict[str, Any]:
    view = corpus["view"]
    g = view["u_ik"] - corpus["own"]
    per = lambda f: [float(f(g[:, 0])), float(f(g[:, 1]))]  # noqa: E731
    tr, ho = view["train"], view["hold"]
    return {"on_500_rows": {"mean_abs": per(lambda x: np.abs(x).mean()), "rms": per(lambda x: np.sqrt(np.mean(x ** 2))),
                            "p90_abs": per(lambda x: np.quantile(np.abs(x), .9)), "max_abs": per(lambda x: np.abs(x).max())},
            "on_train_rows": {"mean_abs": [float(np.abs(g[tr, j]).mean()) for j in (0, 1)],
                              "rms": [float(np.sqrt(np.mean(g[tr, j] ** 2))) for j in (0, 1)],
                              "max_abs": [float(np.abs(g[tr, j]).max()) for j in (0, 1)]},
            "s0d_baseline_vs_uIK_on_holdout_from_recorded_actions": [float(np.sqrt(np.mean(g[ho, j] ** 2))) for j in (0, 1)],
            "baseline_definition": {
                "binding_for_the_T_gate": _amendment()["analytic_predictions_from_measured_gap"]["s0d_baseline_vs_uIK_on_frozen_holdout"],
                "computed_from": "the numpy FORWARD of S0D (the definition used by rev3o/rev3q/rev3r); it is the frozen baseline",
                "recorded_actions_variant": "the anchors use the RECORDED actions (the July construction); the two definitions agree "
                                             "to ~1.1e-08 on this holdout, consistent with the source==init deviation of 2.7e-07. "
                                             "The T gate MUST use the frozen forward-based baseline, never a recomputed one"},
            "joints": ["knee", "ankle"]}


# --- analytic predictions -------------------------------------------------------------------------------

def predict(r: float, gap: Mapping[str, Any]) -> dict[str, Any]:
    """Per-row minimiser of lambda_a*(pi-u_S0D)^2 + lambda_t*(pi-u_IK)^2 with identical inputs."""
    f = 1.0 / (1.0 + float(r))
    g5 = gap["on_500_rows"]
    return {"anchor_target_ratio_r": float(r),
            "predicted_train_drift_mean_abs": [round(g5["mean_abs"][j] * f, 6) for j in (0, 1)],
            "predicted_train_drift_rms": [round(g5["rms"][j] * f, 6) for j in (0, 1)],
            "predicted_train_drift_max_abs": [round(g5["max_abs"][j] * f, 6) for j in (0, 1)],
            "predicted_target_improvement_fraction_train": round(f, 6)}


def verify_predictions(gap: Mapping[str, Any]) -> dict[str, Any]:
    """The predictions frozen in rev3s must be reproducible from the measured corpus."""
    am = _amendment()
    out = []
    for c in am["candidate_budget_finite_frozen"]["candidates"]:
        p = predict(c["anchor_target_ratio_r"], gap)
        for k in ("predicted_train_drift_mean_abs", "predicted_train_drift_rms",
                  "predicted_train_drift_max_abs", "predicted_target_improvement_fraction_train"):
            a, b = c[k], p[k]
            ok = (abs(a - b) <= 5e-6) if isinstance(a, float) else all(abs(x - y) <= 5e-6 for x, y in zip(a, b))
            if not ok:
                raise S1BError(f"candidate {c['id']}: frozen {k} {a} != recomputed {b}")
        out.append({"id": c["id"], **p, "epochs": c["epochs"]})
    return {"candidates": out, "model": am["analytic_predictions_from_measured_gap"]["model"],
            "feasible_band": am["analytic_predictions_from_measured_gap"]["feasible_band_derivation"]}


# --- offline hierarchy (pure; evaluated by the FUTURE fit stage) --------------------------------------------

def offline_hierarchy(metrics: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed hierarchy I -> P -> T -> D with short circuit.  Thresholds come from rev3s and
    are never recomputed here.  No level may declare walking."""
    am = _amendment()["offline_selection_hierarchy_fail_closed"]
    levels: dict[str, Any] = {}
    order = am["order"]
    verdict = "PASS"; failed = None
    for level in order:
        if verdict != "PASS":
            levels[level] = {"result": "not_evaluated"}
            continue
        if level == "I_integrity":
            need = {k: v for k, v in am["I_integrity"].items() if isinstance(v, bool)}
            got = metrics.get("integrity", {})
            bad = [k for k, v in need.items() if bool(got.get(k)) is not v]
            t = float(got.get("T1_T2_max", 1.0))
            if t > am["I_integrity"]["T1_T2_scaling_max"]:
                bad.append("T1_T2_scaling_max")
            ok = not bad
            levels[level] = {"result": "pass" if ok else "fail", "violations": bad}
        elif level == "P_preservation":
            p = am["P_preservation_on_anchors"]; got = metrics.get("preservation", {})
            bad = []
            for key, bound in (("mean_abs", p["mean_abs_max_per_joint"]), ("rms", p["rms_max_per_joint"]),
                               ("max_abs", p["max_abs_max_per_joint"])):
                vals = got.get(key) or []
                if any(float(v) > bound for v in vals):
                    bad.append(f"{key}>{bound}")
            for name, vals in (got.get("per_stratum_mean_abs") or {}).items():
                if any(float(v) > p["per_discrete_stratum_mean_abs_max"] for v in vals):
                    bad.append(f"stratum:{name}")
            levels[level] = {"result": "pass" if not bad else "fail", "violations": bad}
        elif level == "T_target_improvement":
            t = am["T_target_improvement"]; got = metrics.get("target", {})
            base = t["baseline_S0D"]; obs = got.get("holdout_rmse") or []
            impr = [(base[j] - float(obs[j])) / base[j] if j < len(obs) else -1.0 for j in (0, 1)]
            bad = [f"joint{j}:{impr[j]:.4f}" for j in (0, 1) if impr[j] < t["min_relative_improvement_per_joint"]]
            levels[level] = {"result": "pass" if not bad else "fail", "relative_improvement": impr, "violations": bad}
        elif level == "D_drift":
            d = am["D_drift"]; got = metrics.get("drift", {})
            bad = []
            if float(got.get("parameter_shift_sq", 1e9)) > d["parameter_shift_sq_max"]:
                bad.append("parameter_shift_sq")
            if any(float(v) > d["action_drift_mean_abs_max_per_joint_on_500_rows"] for v in (got.get("action_mean_abs") or [1e9])):
                bad.append("action_drift_mean_abs")
            levels[level] = {"result": "pass" if not bad else "fail", "violations": bad}
        else:
            raise S1BError(f"unknown level {level}")
        if levels[level]["result"] == "fail":
            verdict = "FAIL"; failed = level
    return {"levels": levels, "verdict": verdict, "failed_level": failed,
            "allowed_to_be_tested_closed_loop": bool(verdict == "PASS"),
            "no_walking_claim": am["no_walking_claim"]}


def closed_loop_order(offline: Mapping[str, Mapping[str, Any]]) -> list[str]:
    """Frozen evaluation order restricted to the offline survivors (most preserving first)."""
    frozen = _amendment()["closed_loop_gate_preregistered_separate_token"]["order"]
    seq = [t.strip(" ,") for t in frozen.split("FROZEN order")[1].split("restricted")[0].split(",")]
    return [c for c in seq if offline.get(c, {}).get("verdict") == "PASS"]


# --- guards (this stage executes none of these) --------------------------------------------------------------

def run_fit(*_, **kw):
    raise S1BError(f"the S1B fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted "
                   f"(rev3s authorises protocol/tooling/tests/dry-run only); got {kw.get('authorized_stage')!r}")


def run_rollout(*_, **kw):
    raise S1BError(f"the S1B closed-loop evaluation requires --authorized-stage {FUTURE_ROLLOUT_TOKEN}, NOT granted; "
                   f"got {kw.get('authorized_stage')!r}")


def run_collection(*_, **kw):
    raise S1BError(f"alt-start collection requires --authorized-stage {FUTURE_COLLECTION_TOKEN}, NOT granted; "
                   f"got {kw.get('authorized_stage')!r}")


# --- dry run --------------------------------------------------------------------------------------------------

def dry_run(*, authorized_stage: str | None, out_dir: Path = OUT_DRY) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1BError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_s1b()
    prov = verify_source_equals_init()
    corpus = build_corpus()
    gap = gap_stats(corpus)
    am = _amendment()
    frozen_gap = am["analytic_predictions_from_measured_gap"]["measured_gap_on_the_500_trace_rows"]
    for key, got in (("mean_abs", gap["on_500_rows"]["mean_abs"]), ("max_abs", gap["on_500_rows"]["max_abs"])):
        if any(abs(a - b) > 1e-6 for a, b in zip(frozen_gap[key], got)):
            raise S1BError(f"measured gap {key} {got} != frozen {frozen_gap[key]}")
    preds = verify_predictions(gap)
    receipt = {"schema": "v26b_s1b_protocol.1", "authorized_stage": AUTHORIZED_STAGE,
               "amendment_rev3s": PIN_AMENDMENT_REV3S,
               "classification": am["classification"], "lineage": lineage,
               "source_equals_init": prov,
               "corpus": {"anchor": corpus["anchor"], "task": corpus["task"], "holdout": corpus["holdout"],
                          "embargo_rows": corpus["embargo_rows"], "strata": corpus["strata"],
                          "identical_inputs_note": am["structural_finding_blocker"]["identical_inputs_two_labels"]},
               "gap_stats": gap, "analytic_predictions": preds,
               "candidate_budget": am["candidate_budget_finite_frozen"],
               "offline_hierarchy_spec": am["offline_selection_hierarchy_fail_closed"],
               "closed_loop_spec": am["closed_loop_gate_preregistered_separate_token"],
               "blockers": {"structural": am["structural_finding_blocker"],
                            "vs_july": am["source_equals_init_proof"]["limits_vs_13_07"],
                            "vs_S1A": am["comparison_with_S1A_failure"]},
               "sigma_rule": am["sigma_rule"], "iterative_roadmap": am["iterative_roadmap_if_S1B_passes"],
               "executed_in_this_stage": {"fit": False, "rollout": False, "collection": False, "export": False},
               "code_digests": {"v26b_s1b_protocol.py": C.sha256_file(Path(__file__).resolve()),
                                "test_v26b_s1b_protocol.py": C.sha256_file(HERE / "test_v26b_s1b_protocol.py") if (HERE / "test_v26b_s1b_protocol.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    path = R.unique_artifact_path(out_dir, f"v26b_s1b_protocol_dryrun_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "receipt": receipt}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3s S1B protocol (validation only; no fit/rollout/collection)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.dry_run:
        verify_lineage_s1b()
        print(json.dumps({"mode": "lineage-only", "ok": True}, indent=2))
        return 0
    out = dry_run(authorized_stage=args.authorized_stage)
    r = out["receipt"]
    print(json.dumps({"receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"],
                      "source_equals_init_max_dev": r["source_equals_init"]["numpy_forward_vs_recorded"]["max_abs"],
                      "anchor_rows": r["corpus"]["anchor"]["rows"], "task_rows": r["corpus"]["task"]["rows"],
                      "holdout_rows": r["corpus"]["holdout"]["rows"],
                      "candidates": [c["id"] for c in r["analytic_predictions"]["candidates"]],
                      "executed": r["executed_in_this_stage"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
