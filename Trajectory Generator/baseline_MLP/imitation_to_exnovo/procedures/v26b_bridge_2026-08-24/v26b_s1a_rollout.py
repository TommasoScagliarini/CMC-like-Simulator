"""V26B rev3r — SINGLE deterministic nominal rollout of the immutable S1A candidate.

Token V26B-S1A-NOMINAL-ROLLOUT.  Exact rev3c/e/j command shape; the candidate is treated as
strictly READ-ONLY (digests verified before AND after the run, flags never edited).  The seven
binding gates decide ELIGIBILITY of S1A as a future source==init for a July-style anchor
collection stage - they are NOT a deployable promotion and nothing is built here.

Exactly one launch, no retry, no tuning, no promotion.
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

import v26b_s1a_bc as A  # noqa: E402            (rev3q lineage + candidate constants)
import v26b_r0a_rollout as RO  # noqa: E402      (analyse_rollout, corrected_counters)
import v26b_r1_rollout as R1  # noqa: E402       (action_stats, counter_rowscan)
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402   (correct B3 phase-window)
import v26b_s0d_rollout as SR  # noqa: E402      (S0D homolog, read-only)
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402


class S1ARolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1A-NOMINAL-ROLLOUT"
AMENDMENT_REV3R = HERE / "v26b_amendment_rev3r_s1a_nominal_rollout.json"
PIN_AMENDMENT_REV3R = "b20306919d884529bbabbb37b29da3d67a66a1bf1c2b7d2884b583f2a426556c"

CANDIDATE_DIR = A.OUT_S1A
CANDIDATE_MODULE = CANDIDATE_DIR / "rl_module"
PIN_CANDIDATE_RECEIPT = "0d2aa071458c5ea855c1f33d7fcfb7689bc0560ea08b38321bada7d404eafe5a"
PIN_CANDIDATE_ACTOR = A.P0_ACTOR_DIGEST
PIN_CANDIDATE_FILES = {
    "actor_feature_manifest.json": "8c3418769adf4cfa8c117e5f1aa94a0e66a674ac16e540f02fc14be332544ffd",
    "class_and_ctor_args.pkl": "c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7",
    "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "module_state.pkl": "345539298a9cf369c98f24bc8ebc3e7ea06a112f980e83e4e0607bd7e1264c30",
}

JOB_DIR = VA.OUT_ROOT / "rollouts" / "s1a_nominal_det" / "S1A_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_s1a_rollout_receipt.json"
LOG_NAME = "s1a_nominal_det_rollout.log"
PENETRATION_MAX_M = 0.020
PYTHON_EXE = "/opt/anaconda3/envs/envCMC-rllib/bin/python"


# --- lineage / candidate immutability ---------------------------------------------------------------

def candidate_state() -> dict[str, Any]:
    files = {p.name: C.sha256_file(p) for p in sorted(CANDIDATE_MODULE.iterdir())}
    return {"module_files_sha256": files, "receipt_sha256": C.sha256_file(CANDIDATE_DIR / A.RECEIPT_NAME)}


def verify_candidate(where: str) -> dict[str, Any]:
    st = candidate_state()
    if st["module_files_sha256"] != PIN_CANDIDATE_FILES:
        raise S1ARolloutError(f"{where}: candidate module files changed {st['module_files_sha256']}")
    if st["receipt_sha256"] != PIN_CANDIDATE_RECEIPT:
        raise S1ARolloutError(f"{where}: candidate receipt changed {st['receipt_sha256']}")
    rec = json.loads((CANDIDATE_DIR / A.RECEIPT_NAME).read_text(encoding="utf-8"))
    man = json.loads((CANDIDATE_MODULE / "actor_feature_manifest.json").read_text(encoding="utf-8"))
    for k, v in A.MANDATORY_FLAGS.items():
        if rec.get(k) != v or man.get(k) != v:
            raise S1ARolloutError(f"{where}: candidate flag {k} altered (receipt {rec.get(k)!r}, manifest {man.get(k)!r})")
    A.assert_no_deployable_marking(man, f"{where}.manifest")
    return st


def verify_lineage_rollout() -> dict[str, Any]:
    lin = A.verify_lineage_s1a()        # rev3l -> rev3q, prior REJECTED artifacts, additivity, no student/
    got = C.sha256_file(AMENDMENT_REV3R)
    if got != PIN_AMENDMENT_REV3R:
        raise S1ARolloutError(f"rev3r sha {got} != pinned")
    lin["amendment_rev3r"] = got
    lin["candidate_before"] = verify_candidate("pre-run")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    digest = W.actor_state_digest(W.load_module_state(CANDIDATE_MODULE))
    if digest != PIN_CANDIDATE_ACTOR:
        raise S1ARolloutError(f"candidate actor digest {digest} != pinned {PIN_CANDIDATE_ACTOR}")
    lin["candidate_actor_digest"] = digest
    prod = json.loads(AMENDMENT_REV3R.read_text(encoding="utf-8"))["production_pins_resolved"]
    for key, path in (("rollout_eval.py", F1.ROLLOUT_EVAL), ("v3_canonical_resolved_yaml", F1.RUNTIME_CONFIG),
                      ("morphology_corridor_profile", R.CORRIDOR_PROFILE["path"])):
        got = C.sha256_file(Path(path))
        if got != prod[key]["sha256"]:
            raise S1ARolloutError(f"production pin {key} changed: {got} != {prod[key]['sha256']}")
    if C.sha256_file(Path(F1.ROLLOUT_EVAL)) != R.ROLLOUT_EVAL_SHA256_PINNED or C.sha256_file(Path(F1.RUNTIME_CONFIG)) != F1.RUNTIME_CONFIG_SHA256:
        raise S1ARolloutError("chain-level production pins changed")
    lin["production_pins_verified"] = True
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


# --- gates ------------------------------------------------------------------------------------------

def eligibility_gates(analysis: Mapping[str, Any], rowscan: Mapping[str, Any]) -> dict[str, Any]:
    """The seven binding rev3r gates.  Entry gate to anchor collection, never a promotion."""
    comp = analysis["completion"]
    corr = analysis["counters"]["corrected_per_row"]
    mcf = corr["morphology_causal_contract_failure"]
    pen_max = float(analysis["trajectory_quality"]["penetration_m"]["max"])
    g: dict[str, Any] = {}
    g["completion_500_time_limit"] = {"steps": int(comp["steps"]), "end_reason": str(comp["end_reason"]),
                                      "pass": bool(int(comp["steps"]) == 500 and str(comp["end_reason"]) == "episode_time_limit")}
    g["phase_timeout_zero"] = {"stance": int(corr["phase_timeout_stance"]), "swing": int(corr["phase_timeout_swing"]),
                               "source": "all rows, reward_terms.phase_timeout_exceeded + phase_timeout_side",
                               "pass": bool(corr["phase_timeout_stance"] == 0 and corr["phase_timeout_swing"] == 0)}
    g["morphology_causal_contract_failure"] = {"rows_positive": int(mcf["rows_positive"]), "max": float(mcf["max"]),
                                               "failure": bool(mcf["failure"]),
                                               "source": "per row from reward_terms.morphology_causal_failed_closed; never inferred from end_reason",
                                               "pass": bool(mcf["rows_positive"] == 0 and float(mcf["max"]) == 0.0 and mcf["failure"] is False)}
    hs = rowscan["hs_cancelled_count"]; rs = rowscan["resync_count"]; vc = rowscan["valid_cycle_count"]
    g["hs_cancelled_zero"] = {"max_over_rows": float(hs["max_over_rows"]), "final": float(hs["final"]),
                              "pass": bool(hs["max_over_rows"] == 0.0 and hs["final"] == 0.0)}
    g["resync_at_most_one"] = {"max_over_rows": float(rs["max_over_rows"]), "final": float(rs["final"]),
                               "pass": bool(rs["max_over_rows"] <= 1.0 and rs["final"] <= 1.0)}
    g["valid_cycle_at_least_one"] = {"max_over_rows": float(vc["max_over_rows"]), "final": float(vc["final"]),
                                     "pass": bool(vc["final"] >= 1.0)}
    g["penetration"] = {"max_m": pen_max, "limit_m": PENETRATION_MAX_M,
                        "no_grf_penetration_termination": bool(str(comp["end_reason"]) != "grf_penetration"),
                        "pass": bool(pen_max <= PENETRATION_MAX_M and str(comp["end_reason"]) != "grf_penetration")}
    failed = [k for k, v in g.items() if not v["pass"]]
    return {"gates": g, "failed": failed, "all_pass": bool(not failed),
            "meaning": "ELIGIBILITY as a future source==init for a July-style anchor collection stage; NOT a deployable promotion"}


def prescribed_comparison(analysis: Mapping[str, Any], job_dir: Path) -> dict[str, Any]:
    """RMSE / correlation / sign agreement / shape vs the prescribed targets on the grid (diagnostic)."""
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = traj["obs35"].astype(np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    tgt = cc.targets[cc.lookup(np.asarray(traj["t_pre"], dtype=np.float64))]
    out: dict[str, Any] = {}
    for name, col, tcol in (("knee_q", RO.IDX_KNEE_Q, 0), ("ankle_q", RO.IDX_ANKLE_Q, 2)):
        x = obs[:, col]; y = tgt[:, tcol]
        sx, sy = float(x.std()), float(y.std())
        out[name] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                     "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                     "sign_agreement_fraction": float(np.mean(np.sign(x) == np.sign(y))),
                     "shape": {"rollout": {"std": sx, "ptp": float(np.ptp(x))}, "prescribed": {"std": sy, "ptp": float(np.ptp(y))},
                               "amplitude_ratio": (sx / sy if sy > 0 else None)},
                     "mean_offset": float(np.mean(x - y))}
    out["note"] = "prosthetic joint vs healthy prescribed target on the grid; DIAGNOSTIC only, never a gate"
    return out


def compare_homologs(analysis: Mapping[str, Any]) -> dict[str, Any]:
    """S0D (the only chain actor with closed-loop evidence) and the July homologs, where homologous."""
    s0d = json.loads((SR.JOB_DIR / SR.RECEIPT_NAME).read_text(encoding="utf-8"))["analysis"]
    return {
        "s1a": {"steps_end": [analysis["completion"]["steps"], analysis["completion"]["end_reason"]],
                "penetration_max_m": analysis["trajectory_quality"]["penetration_m"]["max"],
                "valid_cycles": analysis["counters"]["valid_cycle_count"]},
        "s0d": {"steps_end": [s0d["completion"]["steps"], s0d["completion"]["end_reason"]],
                "penetration_max_m": s0d["trajectory_quality"]["penetration_m"]["max"],
                "valid_cycles": s0d["counters"]["valid_cycle_count"],
                "note": "the only chain actor with closed-loop evidence (500/500 nominal under v3)"},
        "chain_towards_uIK": {"r0a": "493/500 joint_divergence", "r1": "242/500 grf_penetration", "r2i": "197/500 grf_penetration"},
        "july_homologs": {"11_07_BC_clone": "68/500, 1/0/0, grf_penetration 25.164 mm (covariate shift)",
                          "11_07_DAgger_r2": "356/500, 2/2/1 (best July BC-stage actor; full episode FAIL)",
                          "13_07_markov35": "3/3 det starts 500 steps, 2 cycles (init that ALREADY walked; nominal shift RMS 0.004175)",
                          "note": "July guards were 15/25 mm vs v3 20/28 mm: comparable only qualitatively"},
    }


# --- run ---------------------------------------------------------------------------------------------

def run_rollout(*, authorized_stage: str | None, python_exe: str = PYTHON_EXE,
                job_dir: Path = JOB_DIR) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S1ARolloutError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_rollout()
    job_dir = Path(job_dir)
    if job_dir.exists():
        raise S1ARolloutError(f"no-clobber: {job_dir} exists (the rollout runs EXACTLY once; no retry)")
    cmd = rollout_command(python_exe)
    recorded = json.loads(AMENDMENT_REV3R.read_text(encoding="utf-8"))["command_resolved"]
    if job_dir == JOB_DIR and cmd != recorded:
        raise S1ARolloutError("command differs from the one frozen in rev3r")
    job_dir.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_s1a_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
        "amendment_rev3r": PIN_AMENDMENT_REV3R,
        "purpose": "ELIGIBILITY of S1A as a future source==init; NOT a deployable promotion",
        "candidate": {"module": C.rel(CANDIDATE_MODULE), "actor_digest": PIN_CANDIDATE_ACTOR,
                      "files_sha256": PIN_CANDIDATE_FILES, "receipt_sha256": PIN_CANDIDATE_RECEIPT,
                      **A.MANDATORY_FLAGS,
                      "immutability": "verified before and after the run; never edited"},
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
        "log": C.rel(log_path), "lineage": lineage,
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise S1ARolloutError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(job_dir)
        rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        receipt["analysis"] = analysis
        receipt["fsm_counters_rowscan"] = rowscan
        receipt["eligibility"] = eligibility_gates(analysis, rowscan)
        receipt["diagnostics"] = {
            "action_stats_whole_trace": R1.action_stats(rows),
            "b3_phase_window": PA.b3_late_stance(rows),
            "prescribed_comparison": prescribed_comparison(analysis, job_dir),
            "homologs": compare_homologs(analysis),
            "note": "diagnostics are recorded and never relaxed; none of them is a gate",
        }
        status = "ELIGIBLE" if receipt["eligibility"]["all_pass"] else "NOT_ELIGIBLE_FAIL"
        if not receipt["eligibility"]["all_pass"]:
            receipt["stop"] = ("binding eligibility gates FAILED: everything preserved, cause diagnosed, "
                               "NO retry, NO tuning, NO promotion, no anchor construction")
    finally:
        receipt["candidate_after"] = verify_candidate("post-run")
        receipt["status"] = status
        receipt["anchors_not_built"] = True
        R.write_json_exclusive(job_dir / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3r single S1A nominal det rollout (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default=PYTHON_EXE)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_rollout()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "job_dir_exists": JOB_DIR.exists(),
                          "command": rollout_command(args.python)}, indent=2, default=str))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"],
                      "completion": receipt.get("analysis", {}).get("completion"),
                      "eligibility": receipt.get("eligibility", {}).get("gates"),
                      "failed": receipt.get("eligibility", {}).get("failed"),
                      "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == "ELIGIBLE" else 3


if __name__ == "__main__":
    sys.exit(main())
