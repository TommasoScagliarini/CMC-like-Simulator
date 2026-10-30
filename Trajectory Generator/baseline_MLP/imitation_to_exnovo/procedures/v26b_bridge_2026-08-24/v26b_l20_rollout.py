"""V26B rev4a — SINGLE deterministic nominal rollout of the L20 candidate (token V26B-L20-NOMINAL-ROLLOUT).

Same frozen harness/config as the S0D, S1A and A2 rollouts; the candidate is strictly READ-ONLY
(digests verified before and after, flags never edited).  The seven binding closed-loop gates are
unchanged.  A PASS marks ONLY CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT.

L20's recorded offline state (OFFLINE_FAILED_QUARANTINED on max_abs) is PRESERVED: rev4a demotes
max_abs to informational only for admitting it to this single diagnostic test, and rewrites nothing.

Exactly one launch, no retry, no fit, no promotion, no sigma choice.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1c2z_fit as Z  # noqa: E402          (rev3z lineage + candidate paths, unmodified)
import v26b_s1b_rollout as SB  # noqa: E402       (rev3u gate semantics + A2 homolog, unmodified)
import v26b_s1a_rollout as SA  # noqa: E402       (S1A homolog)
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class L20RolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-L20-NOMINAL-ROLLOUT"
AMENDMENT_REV4A = HERE / "v26b_amendment_rev4a_l20_nominal_rollout.json"
PIN_AMENDMENT_REV4A = "014b922a44168c5853bd9e1a9549fae914cf676f6c9f15fc549526014449db6e"
PROPOSAL = HERE / "v26b_PROPOSAL_ONLY_rev4a_l20_rollout.json"
PIN_PROPOSAL = "8ce44caf78db542e4ef1249f76439e00377c0c0a86aa2cfaee286d1d14d8e2ce"

CANDIDATE_ID = "L20"
CANDIDATE_DIR = Z.out_dir_for(CANDIDATE_ID)
CANDIDATE_MODULE = CANDIDATE_DIR / "rl_module"
PIN_L20_ACTOR = "71d21f309ccf1df7bf8aac60cbf8d1c4322586a6f0fe959f8f522624ee23db55"
PIN_L20_RECEIPT = "03802eadd3fc6371a1f959116b44694d5736ced59ff056e6e4162aae36ce71a9"

JOB_DIR = VA.OUT_ROOT / "rollouts" / "l20_nominal_det" / "S1C2Z_L20_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_l20_rollout_receipt.json"
LOG_NAME = "l20_nominal_det_rollout.log"
PENETRATION_MAX_M = SB.PENETRATION_MAX_M
PYTHON_EXE = SB.PYTHON_EXE
PASS_STATUS = "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT"
FAIL_STATUS = "CLOSED_LOOP_FAIL_QUARANTINED"


def candidate_state() -> dict[str, Any]:
    return {"module_files_sha256": {p.name: C.sha256_file(p) for p in sorted(CANDIDATE_MODULE.iterdir())},
            "fit_receipt_sha256": C.sha256_file(CANDIDATE_DIR / Z.RECEIPT_NAME)}


def verify_candidate(where: str) -> dict[str, Any]:
    st = candidate_state()
    pinned = json.loads(AMENDMENT_REV4A.read_text(encoding="utf-8"))["candidate_under_test"]["files_sha256"]
    if st["module_files_sha256"] != pinned:
        raise L20RolloutError(f"{where}: L20 module files changed {st['module_files_sha256']}")
    if st["fit_receipt_sha256"] != PIN_L20_RECEIPT:
        raise L20RolloutError(f"{where}: L20 fit receipt changed {st['fit_receipt_sha256']}")
    rec = json.loads((CANDIDATE_DIR / Z.RECEIPT_NAME).read_text(encoding="utf-8"))
    if rec["offline_state"] != Z.STATE_FAIL or rec["binding_offline_hierarchy"]["levels"]["P_preservation"]["violations"] != ["max_abs>0.25"]:
        raise L20RolloutError(f"{where}: the recorded offline state of L20 was rewritten - it must stay preserved")
    man = json.loads((CANDIDATE_MODULE / "actor_feature_manifest.json").read_text(encoding="utf-8"))
    if man.get("deployable") is not False or man.get("sigma_unresolved") is not True:
        raise L20RolloutError(f"{where}: candidate flags altered")
    A.assert_no_deployable_marking(man, f"{where}.manifest")
    return st


def verify_lineage() -> dict[str, Any]:
    lin = Z.verify_lineage()
    for path, pin, key in ((AMENDMENT_REV4A, PIN_AMENDMENT_REV4A, "amendment_rev4a"),
                           (PROPOSAL, PIN_PROPOSAL, "proposal_preserved_non_authoritative")):
        got = C.sha256_file(path)
        if got != pin:
            raise L20RolloutError(f"{key} sha {got} != pinned")
        lin[key] = got
    agg = json.loads((VA.OUT_ROOT / "candidates" / "v26b_s1c2z_fit_aggregate_20260824_213123.json").read_text(encoding="utf-8"))
    if agg["offline_survivors"] or agg["state_by_candidate"][CANDIDATE_ID] != Z.STATE_FAIL:
        raise L20RolloutError("the rev3z aggregate no longer records L20 as a failed candidate")
    lin["rev3z_aggregate_states_preserved"] = agg["state_by_candidate"]
    lin["candidate_before"] = verify_candidate("pre-run")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    digest = W.actor_state_digest(W.load_module_state(CANDIDATE_MODULE))
    if digest != PIN_L20_ACTOR:
        raise L20RolloutError(f"L20 actor digest {digest} != pinned")
    lin["l20_actor_digest"] = digest
    prod = json.loads(AMENDMENT_REV4A.read_text(encoding="utf-8"))["harness_equivalence"]
    for key, path in (("rollout_eval_sha256", F1.ROLLOUT_EVAL), ("v3_canonical_yaml_sha256", F1.RUNTIME_CONFIG),
                      ("morphology_corridor_sha256", R.CORRIDOR_PROFILE["path"])):
        if C.sha256_file(Path(path)) != prod[key]:
            raise L20RolloutError(f"production pin {key} changed")
    lin["production_pins_verified"] = True
    lin["harness_equivalence_vs_a2"] = harness_equivalence()
    return lin


def rollout_command(python_exe: str = PYTHON_EXE) -> list[str]:
    return [
        python_exe, str(F1.ROLLOUT_EVAL),
        "--checkpoint", str(CANDIDATE_MODULE),
        "--no-auto-config",
        "--config", str(F1.RUNTIME_CONFIG),
        "--episode-start-offset-s", repr(float(R.EXACT_STARTS["nominal"])),
        "--action-selection", "deterministic",
        "--seed", str(R.DET_SEED),
        "--output-dir", str(JOB_DIR),
        "--record-outputs", "--record-policy-trace",
        *[str(a) for a in F1.JOB_TIMEOUT_ARGS],
        *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]],
    ]


def harness_equivalence(python_exe: str = PYTHON_EXE) -> dict[str, Any]:
    """The command must differ from the frozen A2 command ONLY in --checkpoint and --output-dir."""
    mine = rollout_command(python_exe)
    theirs = json.loads(SB.AMENDMENT_REV3U.read_text(encoding="utf-8"))["harness_equivalence"]["command_resolved"]
    if len(mine) != len(theirs):
        raise L20RolloutError("command length differs from the frozen A2 command")
    diffs = [i for i in range(len(mine)) if mine[i] != theirs[i]]
    allowed = {mine.index("--checkpoint") + 1, mine.index("--output-dir") + 1}
    bad = [i for i in diffs if i not in allowed]
    if bad:
        raise L20RolloutError(f"harness NOT equivalent to the A2 rollout: unexpected differences at {bad}")
    return {"reference": "frozen rev3u A2 command", "differs_only_in": ["--checkpoint", "--output-dir"],
            "diff_positions": diffs, "equivalent": True}


def eligibility_gates(analysis: Mapping[str, Any], rowscan: Mapping[str, Any]) -> dict[str, Any]:
    g = SB.eligibility_gates(analysis, rowscan)
    g["meaning"] = ("closed-loop eligibility of L20. A PASS marks ONLY "
                    f"{PASS_STATUS}: it is NOT a promotion, the candidate stays NON-DEPLOYABLE, "
                    "and it does NOT resolve sigma")
    return g


def closed_loop_comparison(job_dir: Path) -> dict[str, Any]:
    """DIAGNOSTIC ONLY: L20 versus S0D, A2, S1A and the prosthetic IK target on the visited states."""
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(traj["obs35"], dtype=np.float32)
    act = np.asarray(traj["b_raw_action"], dtype=np.float64)
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    s0d = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SR.S0D_MODULE).items()}
    s0d_on = RF.numpy_mean(s0d, obs)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(t_pre)
    u_ik = cc.ik_action[idx].astype(np.float64)
    tgt = cc.targets[idx]
    q_ik = SC.decode_action(u_ik)
    knee = obs[:, RO.IDX_KNEE_Q].astype(np.float64); ankle = obs[:, RO.IDX_ANKLE_Q].astype(np.float64)
    q_cmd = SC.decode_action(act)

    def jstats(x, bounds):
        return {"min": float(x.min()), "max": float(x.max()), "mean": float(x.mean()), "range": float(np.ptp(x)),
                "frac_negative": float(np.mean(x < 0.0)), "outside_bounds_steps": int(np.sum((x < bounds[0]) | (x > bounds[1])))}

    def vs(x, y):
        sx, sy = float(x.std()), float(y.std())
        deg = bool(np.all(y >= 0.0) or np.all(y <= 0.0))
        return {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                "amplitude_ratio": (sx / sy if sy > 0 else None),
                "sign_agreement": (None if deg else float(np.mean(np.sign(x) == np.sign(y)))),
                "sign_agreement_verdict": ("VOID_degenerate_reference" if deg else "valid")}

    neg = q_ik[:, 1] < 0.0
    s0d_rec = json.loads((SR.JOB_DIR / SR.RECEIPT_NAME).read_text(encoding="utf-8"))["analysis"]
    a2_rec = json.loads((SB.JOB_DIR / SB.RECEIPT_NAME).read_text(encoding="utf-8"))
    s1a_rec = json.loads((SA.JOB_DIR / SA.RECEIPT_NAME).read_text(encoding="utf-8"))
    per = lambda d: [float(np.abs(d[:, j]).mean()) for j in (0, 1)]  # noqa: E731
    return {
        "scope": "CLOSED-LOOP metrics on the states L20 actually visited. Diagnostic only, never a gate; not comparable term by term with the offline fit metrics",
        "kinematics_L20": {"knee_q": jstats(knee, RO.KNEE_BOUNDS), "ankle_q": jstats(ankle, RO.ANKLE_BOUNDS)},
        "vs_prosthetic_IK_target": {"knee_q": vs(knee, q_ik[:, 0]), "ankle_q": vs(ankle, q_ik[:, 1]),
                                    "role": "BINDING quality reference in the rev3v convention (diagnostic here)"},
        "vs_healthy_symmetry_diagnostic": {"knee_q": vs(knee, tgt[:, 0]), "ankle_q": vs(ankle, tgt[:, 2]),
                                           "role": "symmetry diagnostic only; sign agreement is VOID when the reference is sign-degenerate"},
        "actions_on_visited_states": {
            "L20_vs_S0D_same_states": {"mean_abs": per(act - s0d_on),
                                       "max_abs": [float(np.abs((act - s0d_on)[:, j]).max()) for j in (0, 1)]},
            "L20_vs_uIK_same_times": {"mean_abs": per(act - u_ik),
                                      "rmse": [float(np.sqrt(np.mean((act[:, j] - u_ik[:, j]) ** 2))) for j in (0, 1)]},
            "S0D_vs_uIK_on_L20_states": {"mean_abs": per(s0d_on - u_ik)},
        },
        "negative_window_sign": {"rows_with_negative_IK_target": int(neg.sum()),
                                 "fraction_positive_ankle_command_in_those_rows": (float(np.mean(q_cmd[neg, 1] > 0.0)) if neg.any() else None),
                                 "note": "on the states L20 visited, so the window membership follows its own trace"},
        "homologs": {
            "S0D": {"steps_end": [s0d_rec["completion"]["steps"], s0d_rec["completion"]["end_reason"]],
                    "valid_cycles": s0d_rec["counters"]["valid_cycle_count"],
                    "penetration_max_m": s0d_rec["trajectory_quality"]["penetration_m"]["max"]},
            "A2": {"steps_end": [a2_rec["analysis"]["completion"]["steps"], a2_rec["analysis"]["completion"]["end_reason"]],
                   "valid_cycles": a2_rec["analysis"]["counters"]["valid_cycle_count"],
                   "penetration_max_m": a2_rec["analysis"]["trajectory_quality"]["penetration_m"]["max"],
                   "status": a2_rec["status"]},
            "S1A": {"steps_end": [s1a_rec["analysis"]["completion"]["steps"], s1a_rec["analysis"]["completion"]["end_reason"]],
                    "valid_cycles": s1a_rec["analysis"]["counters"]["valid_cycle_count"],
                    "penetration_max_m": s1a_rec["analysis"]["trajectory_quality"]["penetration_m"]["max"]},
        },
        "offline_reference_block": {
            "source": "L20 fit receipt (frozen S0D-visited rows)",
            "action_drift_vs_S0D": [0.0649, 0.0815], "A2_action_drift_vs_S0D": [0.0657, 0.0455],
            "S1A_action_drift_vs_S0D": [0.384, 0.289],
            "note": "offline block, context only; never mixed with the closed-loop block above"},
    }


def run_rollout(*, authorized_stage: str | None, python_exe: str = PYTHON_EXE, job_dir: Path = JOB_DIR) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise L20RolloutError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    job_dir = Path(job_dir)
    if job_dir.exists():
        raise L20RolloutError(f"no-clobber: {job_dir} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    job_dir.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_l20_rollout.1", "authorized_stage": AUTHORIZED_STAGE, "amendment_rev4a": PIN_AMENDMENT_REV4A,
        "candidate": {"id": CANDIDATE_ID, "module": C.rel(CANDIDATE_MODULE), "actor_digest": PIN_L20_ACTOR,
                      "fit_receipt_sha256": PIN_L20_RECEIPT, "deployable": False, "sigma_unresolved": True,
                      "recorded_offline_state": Z.STATE_FAIL,
                      "offline_state_preserved": "the recorded FAIL on max_abs is NOT rewritten; rev4a demoted max_abs to informational only to admit L20 to this single test",
                      "immutability": "verified before and after; never edited"},
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
        "log": C.rel(log_path), "lineage": lineage,
        "sigma": "UNRESOLVED: deterministic mean only; this stage does NOT resolve sigma",
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise L20RolloutError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(job_dir)
        rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        receipt["analysis"] = analysis
        receipt["fsm_counters_rowscan"] = rowscan
        receipt["eligibility"] = eligibility_gates(analysis, rowscan)
        receipt["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                                  "b3_phase_window": PA.b3_late_stance(rows),
                                  "closed_loop_comparison": closed_loop_comparison(job_dir)}
        ok = receipt["eligibility"]["all_pass"]
        status = PASS_STATUS if ok else FAIL_STATUS
        if ok:
            receipt["promotion"] = {"promoted": False, "status": PASS_STATUS,
                                    "note": "eligibility only; the candidate stays NON-DEPLOYABLE; the architect decides afterwards"}
        else:
            receipt["quarantine"] = {"candidate": CANDIDATE_ID, "quarantined": True, "may_not_be_source": True,
                                     "policy": "no relaunch, no tuning, no promotion",
                                     "forward_rule_rev4a": "the next architectural candidate remains a July-faithful DAgger on the S1A prefix (392 steps)"}
    finally:
        receipt["candidate_after"] = verify_candidate("post-run")
        receipt["status"] = status
        receipt["anchors_not_built"] = True
        receipt["inherited_prose_allowlisted"] = SB.assert_no_deployable_marking_local(receipt, "receipt")
        R.write_json_exclusive(job_dir / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev4a single L20 nominal det rollout (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default=PYTHON_EXE)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "job_dir_exists": JOB_DIR.exists()}, indent=2))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "completion": receipt.get("analysis", {}).get("completion"),
                      "gates": receipt.get("eligibility", {}).get("gates"),
                      "failed": receipt.get("eligibility", {}).get("failed"),
                      "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == PASS_STATUS else 3


if __name__ == "__main__":
    sys.exit(main())
