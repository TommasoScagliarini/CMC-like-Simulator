"""V26B rev3u — SINGLE deterministic-mean nominal rollout of the offline survivor A2.

Token V26B-S1B-NOMINAL-ROLLOUT.  Same frozen harness/config as the S0D and S1A rollouts; the
candidate is strictly READ-ONLY (digests verified before and after).  Seven binding gates decide
closed-loop eligibility; a PASS marks ONLY CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT.

The A2 vs S0D vs AB06-u_IK comparison on the resulting trace is DIAGNOSTIC and can never change a
gate; offline metrics (frozen S0D-visited rows) and closed-loop metrics (A2's own visited states)
are reported in separate blocks and are not comparable term by term.

Exactly one launch, no retry, no sigma sampling, no promotion.
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

import v26b_s1b_fit as F  # noqa: E402           (rev3t lineage + candidate paths)
import v26b_s1a_rollout as SA  # noqa: E402      (rev3r gate semantics + S1A homolog, unmodified)
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_fit as FIT  # noqa: E402
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


class S1BRolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1B-NOMINAL-ROLLOUT"
AMENDMENT_REV3U = HERE / "v26b_amendment_rev3u_s1b_a2_nominal_rollout.json"
PIN_AMENDMENT_REV3U = "a3b17a883fb43006f7a3d49b9a1a1e77334f0974e959241af3a496f62259a011"
CANDIDATE_ID = "A2"
CANDIDATE_DIR = F.out_dir_for(CANDIDATE_ID)
CANDIDATE_MODULE = CANDIDATE_DIR / "rl_module"
PIN_A2_ACTOR = "cde2c8e6356f833d9e1108b542286c2c9265f8f69bb4c0a894c36136c5d0916c"
PIN_A2_RECEIPT = "5430115d327ca9c0acd4ed992b7026bf4a940555544c8b81414fb72dfab78616"
PIN_AGGREGATE = "0539475cda88030e0af059b394974938f64a03b80490134295ca0ed486ae47e6"
AGGREGATE = VA.OUT_ROOT / "candidates" / "v26b_s1b_fit_aggregate_20260824_201458.json"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "s1b_a2_nominal_det" / "S1B_A2_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_s1b_rollout_receipt.json"
LOG_NAME = "s1b_a2_nominal_det_rollout.log"
PENETRATION_MAX_M = SA.PENETRATION_MAX_M          # 0.020 by reference
PYTHON_EXE = SA.PYTHON_EXE
PASS_STATUS = "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT"

# Documentary prose inherited VERBATIM from frozen upstream tools. It uses the word descriptively and
# marks nothing; it is allow-listed one string at a time, never by pattern, so any other occurrence
# still fails closed.  (Source: v26b_r0a_rollout.analyse_rollout, a pinned tool we must not modify.)
INHERITED_PROSE_ALLOWLIST = (
    "the deployable 35D manifest carries no *_saturated feature (those were 39-slice diagnostics); "
    "the July name filter matches 8 features here",
)


def assert_no_deployable_marking_local(obj: Any, where: str = "receipt", seen: list | None = None) -> list:
    """Same intent as the rev3q scanner (no deployable MARKING or alias), but tolerant of the exact
    documentary strings inherited from frozen tools, which are recorded rather than silently accepted."""
    seen = [] if seen is None else seen
    if isinstance(obj, dict):
        for k, v in obj.items():
            if str(k).lower() == "deployable" and v is not False:
                raise S1BRolloutError(f"{where}: key 'deployable' must be False, got {v!r}")
            assert_no_deployable_marking_local(v, f"{where}.{k}", seen)
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_deployable_marking_local(v, f"{where}[{i}]", seen)
    elif isinstance(obj, str):
        norm = obj.lower().replace("-", "_")
        if "deployable" in norm.replace("non_deployable", "").replace("nondeployable", ""):
            if obj in INHERITED_PROSE_ALLOWLIST:
                if obj not in seen:
                    seen.append(obj)
            else:
                raise S1BRolloutError(f"{where}: forbidden deployable marking/alias in {obj!r}")
    return seen


def candidate_state() -> dict[str, Any]:
    return {"module_files_sha256": {p.name: C.sha256_file(p) for p in sorted(CANDIDATE_MODULE.iterdir())},
            "receipt_sha256": C.sha256_file(CANDIDATE_DIR / F.RECEIPT_NAME)}


def verify_candidate(where: str) -> dict[str, Any]:
    st = candidate_state()
    pinned = json.loads(AMENDMENT_REV3U.read_text(encoding="utf-8"))["candidate_under_test"]["files_sha256"]
    if st["module_files_sha256"] != pinned:
        raise S1BRolloutError(f"{where}: A2 module files changed {st['module_files_sha256']}")
    if st["receipt_sha256"] != PIN_A2_RECEIPT:
        raise S1BRolloutError(f"{where}: A2 fit receipt changed {st['receipt_sha256']}")
    rec = json.loads((CANDIDATE_DIR / F.RECEIPT_NAME).read_text(encoding="utf-8"))
    man = json.loads((CANDIDATE_MODULE / "actor_feature_manifest.json").read_text(encoding="utf-8"))
    if rec["offline_hierarchy"]["verdict"] != "PASS" or rec["quarantined"] is not False:
        raise S1BRolloutError(f"{where}: A2 is not the offline survivor")
    for k, v in A.MANDATORY_FLAGS.items():
        if rec.get(k) != v or man.get(k) != v:
            raise S1BRolloutError(f"{where}: candidate flag {k} altered")
    A.assert_no_deployable_marking(man, f"{where}.manifest")
    return st


def verify_lineage_rollout() -> dict[str, Any]:
    lin = F.verify_lineage_fit()
    got = C.sha256_file(AMENDMENT_REV3U)
    if got != PIN_AMENDMENT_REV3U:
        raise S1BRolloutError(f"rev3u sha {got} != pinned")
    lin["amendment_rev3u"] = got
    got = C.sha256_file(AGGREGATE)
    if got != PIN_AGGREGATE:
        raise S1BRolloutError(f"S1B fit aggregate sha {got} != pinned")
    agg = json.loads(AGGREGATE.read_text(encoding="utf-8"))
    if agg["offline_survivors_in_frozen_closed_loop_order"] != [CANDIDATE_ID]:
        raise S1BRolloutError(f"the aggregate does not record {CANDIDATE_ID} as the sole survivor")
    lin["s1b_fit_aggregate"] = got
    lin["offline_survivors"] = agg["offline_survivors_in_frozen_closed_loop_order"]
    lin["candidate_before"] = verify_candidate("pre-run")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    digest = W.actor_state_digest(W.load_module_state(CANDIDATE_MODULE))
    if digest != PIN_A2_ACTOR:
        raise S1BRolloutError(f"A2 actor digest {digest} != pinned")
    lin["a2_actor_digest"] = digest
    prod = json.loads(AMENDMENT_REV3U.read_text(encoding="utf-8"))["harness_equivalence"]
    for key, path in (("rollout_eval.py", F1.ROLLOUT_EVAL), ("v3_canonical_resolved_yaml", F1.RUNTIME_CONFIG),
                      ("morphology_corridor_profile", R.CORRIDOR_PROFILE["path"])):
        if C.sha256_file(Path(path)) != prod[key]["sha256"]:
            raise S1BRolloutError(f"production pin {key} changed")
    lin["production_pins_verified"] = True
    lin["harness_equivalence_vs_s1a"] = harness_equivalence()
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
    """The command must differ from the frozen S1A one ONLY in --checkpoint and --output-dir."""
    mine = rollout_command(python_exe)
    theirs = json.loads(SA.AMENDMENT_REV3R.read_text(encoding="utf-8"))["command_resolved"]
    if len(mine) != len(theirs):
        raise S1BRolloutError("command length differs from the frozen S1A command")
    diffs = [(i, theirs[i], mine[i]) for i in range(len(mine)) if mine[i] != theirs[i]]
    allowed = {mine.index("--checkpoint") + 1, mine.index("--output-dir") + 1}
    bad = [d for d in diffs if d[0] not in allowed]
    if bad:
        raise S1BRolloutError(f"harness NOT equivalent to the S1A rollout: unexpected differences {bad}")
    return {"differs_only_in": ["--checkpoint", "--output-dir"], "diff_positions": [d[0] for d in diffs],
            "reference": "frozen rev3r S1A command", "equivalent": True}


# --- gates ---------------------------------------------------------------------------------------------

def eligibility_gates(analysis: Mapping[str, Any], rowscan: Mapping[str, Any]) -> dict[str, Any]:
    """The seven binding rev3u gates (identical semantics to rev3r), computed whole-trace."""
    g = SA.eligibility_gates(analysis, rowscan)
    g["meaning"] = ("closed-loop eligibility of A2. A PASS marks ONLY "
                    f"{PASS_STATUS}: it is NOT a promotion, the candidate stays NON-DEPLOYABLE, "
                    "and it does NOT resolve sigma")
    return g


def closed_loop_comparison(analysis: Mapping[str, Any], job_dir: Path) -> dict[str, Any]:
    """DIAGNOSTIC ONLY. A2 vs S0D vs AB06 u_IK on the states A2 actually visited."""
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(traj["obs35"], dtype=np.float32)
    a2_act = np.asarray(traj["b_raw_action"], dtype=np.float64)
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    s0d = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(SR.S0D_MODULE).items()}
    s0d_on_a2 = RF.numpy_mean(s0d, obs)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    u_ik = cc.ik_action[cc.lookup(t_pre)].astype(np.float64)
    tgt = cc.targets[cc.lookup(t_pre)]
    knee = obs[:, RO.IDX_KNEE_Q].astype(np.float64); ankle = obs[:, RO.IDX_ANKLE_Q].astype(np.float64)

    def jstats(x, bounds):
        return {"min": float(x.min()), "max": float(x.max()), "mean": float(x.mean()),
                "range": float(np.ptp(x)), "frac_negative": float(np.mean(x < 0.0)),
                "outside_bounds_steps": int(np.sum((x < bounds[0]) | (x > bounds[1]))), "bounds": list(bounds)}

    def vs_prescribed(x, y):
        sx, sy = float(x.std()), float(y.std())
        return {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                "sign_agreement_fraction": float(np.mean(np.sign(x) == np.sign(y))),
                "amplitude_ratio": (sx / sy if sy > 0 else None), "mean_offset": float(np.mean(x - y))}

    s0d_rec = json.loads((SR.JOB_DIR / SR.RECEIPT_NAME).read_text(encoding="utf-8"))["analysis"]
    s1a_rec = json.loads((SA.JOB_DIR / SA.RECEIPT_NAME).read_text(encoding="utf-8"))
    per = lambda d: [float(np.abs(d[:, j]).mean()) for j in (0, 1)]  # noqa: E731
    return {
        "scope": "CLOSED-LOOP metrics on the states A2 actually visited. NOT comparable term by term with the "
                 "OFFLINE fit metrics, which were computed on the frozen S0D-visited rows. Diagnostic only, never a gate",
        "kinematics_A2": {"knee_q": jstats(knee, RO.KNEE_BOUNDS), "ankle_q": jstats(ankle, RO.ANKLE_BOUNDS)},
        "vs_prescribed_targets": {"knee_q": vs_prescribed(knee, tgt[:, 0]), "ankle_q": vs_prescribed(ankle, tgt[:, 2]),
                                  "note": "prosthetic joint vs healthy prescribed target on the grid"},
        "actions_on_visited_states": {
            "A2_vs_S0D_same_states": {"mean_abs": per(a2_act - s0d_on_a2),
                                      "max_abs": [float(np.abs((a2_act - s0d_on_a2)[:, j]).max()) for j in (0, 1)],
                                      "meaning": "how far A2 acts from what S0D would do on the states A2 itself reached"},
            "A2_vs_uIK_same_times": {"mean_abs": per(a2_act - u_ik),
                                     "rmse": [float(np.sqrt(np.mean((a2_act[:, j] - u_ik[:, j]) ** 2))) for j in (0, 1)]},
            "S0D_vs_uIK_on_A2_states": {"mean_abs": per(s0d_on_a2 - u_ik),
                                        "rmse": [float(np.sqrt(np.mean((s0d_on_a2[:, j] - u_ik[:, j]) ** 2))) for j in (0, 1)],
                                        "meaning": "the reference gap re-measured on A2's own visited states"},
        },
        "homologs": {
            "S0D": {"steps_end": [s0d_rec["completion"]["steps"], s0d_rec["completion"]["end_reason"]],
                    "valid_cycles": s0d_rec["counters"]["valid_cycle_count"],
                    "penetration_max_m": s0d_rec["trajectory_quality"]["penetration_m"]["max"],
                    "knee_q": s0d_rec["trajectory_quality"]["knee_q"], "ankle_q": s0d_rec["trajectory_quality"]["ankle_q"],
                    "rmse_vs_prescribed": s0d_rec["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"]},
            "S1A": {"steps_end": [s1a_rec["analysis"]["completion"]["steps"], s1a_rec["analysis"]["completion"]["end_reason"]],
                    "valid_cycles": s1a_rec["analysis"]["counters"]["valid_cycle_count"],
                    "penetration_max_m": s1a_rec["analysis"]["trajectory_quality"]["penetration_m"]["max"],
                    "prescribed": s1a_rec["diagnostics"]["prescribed_comparison"]},
            "july": {"11_07_BC_clone": "68/500, 1/0/0", "11_07_DAgger_r2": "356/500, 2/2/1",
                     "13_07_markov35": "500/500 x3 starts, 2 cycles (guards 15/25 mm vs v3 20/28 mm)"},
        },
        "offline_reference_block": {
            "source": "A2 fit receipt (frozen S0D-visited rows)",
            "holdout_rmse_vs_uIK": [0.5080219305410626, 0.39396737899584355],
            "s0d_baseline_holdout": [0.6214394597731364, 0.4569210122991978],
            "anchor_preservation_mean_abs": [0.059229, 0.044862],
            "note": "offline block, reported for context only; never mixed with the closed-loop block above"},
    }


# --- run -------------------------------------------------------------------------------------------------

def run_rollout(*, authorized_stage: str | None, python_exe: str = PYTHON_EXE, job_dir: Path = JOB_DIR) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1BRolloutError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_rollout()
    job_dir = Path(job_dir)
    if job_dir.exists():
        raise S1BRolloutError(f"no-clobber: {job_dir} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    frozen = json.loads(AMENDMENT_REV3U.read_text(encoding="utf-8"))["harness_equivalence"]["command_resolved"]
    if job_dir == JOB_DIR and cmd != frozen:
        raise S1BRolloutError("command differs from the one frozen in rev3u")
    job_dir.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_s1b_rollout.1", "authorized_stage": AUTHORIZED_STAGE, "amendment_rev3u": PIN_AMENDMENT_REV3U,
        "candidate": {"id": CANDIDATE_ID, "module": C.rel(CANDIDATE_MODULE), "actor_digest": PIN_A2_ACTOR,
                      "fit_receipt_sha256": PIN_A2_RECEIPT, **A.MANDATORY_FLAGS, "quarantined": False,
                      "immutability": "verified before and after; never edited"},
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
        "log": C.rel(log_path), "lineage": lineage,
        "sigma": "UNRESOLVED: deterministic mean only; this stage does NOT resolve sigma",
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise S1BRolloutError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(job_dir)
        rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        receipt["analysis"] = analysis
        receipt["fsm_counters_rowscan"] = rowscan
        receipt["eligibility"] = eligibility_gates(analysis, rowscan)
        receipt["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                                  "b3_phase_window": PA.b3_late_stance(rows),
                                  "closed_loop_comparison": closed_loop_comparison(analysis, job_dir)}
        ok = receipt["eligibility"]["all_pass"]
        status = PASS_STATUS if ok else "CLOSED_LOOP_FAIL_QUARANTINED"
        if not ok:
            receipt["quarantine"] = {"candidate": CANDIDATE_ID, "quarantined": True, "may_not_be_source": True,
                                     "policy": "no relaunch, no tuning, no promotion; causal diagnosis in the report"}
        else:
            receipt["promotion"] = {"promoted": False, "status": PASS_STATUS,
                                    "note": "eligibility only; the candidate stays NON-DEPLOYABLE, no anchor construction, architect audit required"}
    finally:
        receipt["candidate_after"] = verify_candidate("post-run")
        receipt["status"] = status
        receipt["anchors_not_built"] = True
        receipt["inherited_prose_allowlisted"] = assert_no_deployable_marking_local(receipt, "receipt")
        R.write_json_exclusive(job_dir / RECEIPT_NAME, receipt)
    return receipt


def finalize_receipt(*, authorized_stage: str | None, job_dir: Path = JOB_DIR,
                     incident: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Write the receipt of the SINGLE rollout already executed, from its preserved artifacts.
    This launches NOTHING: it exists because a tooling exception aborted the receipt write after the
    subprocess had completed.  It refuses if the artifacts are missing or a receipt already exists."""
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1BRolloutError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    job_dir = Path(job_dir)
    for name in ("rollout_policy_trace.json", "rollout_summary.json"):
        if not (job_dir / name).is_file():
            raise S1BRolloutError(f"cannot finalize: {name} missing (no rollout was executed)")
    if (job_dir / RECEIPT_NAME).exists():
        raise S1BRolloutError("a receipt already exists: nothing to finalize")
    lineage = verify_lineage_rollout()
    cmd = rollout_command()
    log_path = VA.OUT_LOGS / LOG_NAME
    receipt: dict[str, Any] = {
        "schema": "v26b_s1b_rollout.1", "authorized_stage": AUTHORIZED_STAGE, "amendment_rev3u": PIN_AMENDMENT_REV3U,
        "candidate": {"id": CANDIDATE_ID, "module": C.rel(CANDIDATE_MODULE), "actor_digest": PIN_A2_ACTOR,
                      "fit_receipt_sha256": PIN_A2_RECEIPT, **A.MANDATORY_FLAGS, "quarantined": False,
                      "immutability": "verified before and after; never edited"},
        "command": cmd, "returncode": 0,
        "returncode_provenance": ("proven by the executed code path: run_rollout raises unless proc.returncode == 0, "
                                   "and execution proceeded past that check into the trace analysis"),
        "log": C.rel(log_path), "log_sha256": C.sha256_file(log_path), "lineage": lineage,
        "sigma": "UNRESOLVED: deterministic mean only; this stage does NOT resolve sigma",
        "receipt_finalized_after_tooling_incident": dict(incident or {}),
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        analysis = RO.analyse_rollout(job_dir)
        rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        receipt["analysis"] = analysis
        receipt["fsm_counters_rowscan"] = rowscan
        receipt["eligibility"] = eligibility_gates(analysis, rowscan)
        receipt["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                                  "b3_phase_window": PA.b3_late_stance(rows),
                                  "closed_loop_comparison": closed_loop_comparison(analysis, job_dir)}
        ok = receipt["eligibility"]["all_pass"]
        status = PASS_STATUS if ok else "CLOSED_LOOP_FAIL_QUARANTINED"
        if not ok:
            receipt["quarantine"] = {"candidate": CANDIDATE_ID, "quarantined": True, "may_not_be_source": True,
                                     "policy": "no relaunch, no tuning, no promotion; causal diagnosis in the report"}
        else:
            receipt["promotion"] = {"promoted": False, "status": PASS_STATUS,
                                    "note": "eligibility only; the candidate stays NON-DEPLOYABLE, no anchor construction, architect audit required"}
    finally:
        receipt["candidate_after"] = verify_candidate("post-run")
        receipt["status"] = status
        receipt["anchors_not_built"] = True
        receipt["inherited_prose_allowlisted"] = assert_no_deployable_marking_local(receipt, "receipt")
        R.write_json_exclusive(job_dir / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3u single A2 nominal det rollout (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default=PYTHON_EXE)
    parser.add_argument("--finalize-receipt", action="store_true",
                        help="write the receipt of the single already-executed rollout; launches nothing")
    args = parser.parse_args(argv)
    if args.finalize_receipt:
        rec = finalize_receipt(authorized_stage=args.authorized_stage,
                               incident={"what": "the receipt write of the single executed rollout aborted inside the finally block",
                                          "cause": "the rev3q NON-DEPLOYABLE-marking scanner rejected documentary prose inherited verbatim from the frozen tool v26b_r0a_rollout.analyse_rollout",
                                          "resolution": "a local scanner with an explicit verbatim allow-list was added in this module; the frozen tool was NOT modified",
                                          "rollout_relaunched": False})
        print(json.dumps({"status": rec["status"], "completion": rec["analysis"]["completion"],
                          "gates": rec["eligibility"]["gates"], "failed": rec["eligibility"]["failed"],
                          "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
        return 0 if rec["status"] == PASS_STATUS else 3
    if not args.execute:
        verify_lineage_rollout()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "job_dir_exists": JOB_DIR.exists(),
                          "command": rollout_command(args.python)}, indent=2, default=str))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "completion": receipt.get("analysis", {}).get("completion"),
                      "gates": receipt.get("eligibility", {}).get("gates"), "failed": receipt.get("eligibility", {}).get("failed"),
                      "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == PASS_STATUS else 3


if __name__ == "__main__":
    sys.exit(main())
