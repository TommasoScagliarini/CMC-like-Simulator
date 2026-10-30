"""V26B rev3e — SINGLE deterministic nominal rollout of V2_DAGGER_R1.

Reuses the validated rev3c/rev3d rollout tooling READ-ONLY (analysis functions
imported from ``v26b_r0a_rollout``, corrected whole-trace counters included);
adds whole-trace action statistics and the explicit R0a comparison.  Exactly one
job, no retry, no stochastic rollout, no operational sigma, no DAgger follow-up.
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
VALIDATION_DIR = HERE.parent
for _entry in (str(VALIDATION_DIR / "f0_freeze_2026-08-22"), str(VALIDATION_DIR / "f1_ablation_2026-08-23"), str(VALIDATION_DIR / "f2r_bridge_2026-08-23"), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_dagger_r1 as D1  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_student as VS  # noqa: E402


class R1RolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R1-ROLLOUT"
AMENDMENT_REV3E = HERE / "v26b_amendment_rev3e_r1_nominal_rollout.json"
PIN_AMENDMENT_REV3E = "8ac5af69f7914d2a8de19b7da73cd19351876ff3e561660c1b42f51a35b7c04c"
R1_MODULE = D1.OUT_R1 / "rl_module"
PIN_R1_RECEIPT = "f409e7880e5c8be3c17d81c00b1ddc48c4c4e6c25058b0b55e452aaebb346c42"
PIN_R1_ACTOR_DIGEST = "c7bcee1c1165625d0c574a709a9444a98ea95edea8fcddd5d81f59dfa2e3d31e"
PIN_R1_MODULE_FILES = {
    "actor_feature_manifest.json": "002ebd265fe58dac00dde88f0524465a21b5a20502c6c027c0231a1f9507f646",
    "class_and_ctor_args.pkl": "c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7",
    "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "module_state.pkl": "12447f48b2458086462fd7dd3e8ca37ccd77db815599bf78cded4672a17f2cf7",
}
PIN_R1_DATASET_NPZ = "ce309b40e716342bc00a4fe6e8b835815c93419d36be310271f8f4e2ab7250de"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "r1_nominal_det" / "DAGGER_R1_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_r1_rollout_receipt.json"


def verify_lineage_r1_rollout() -> dict[str, Any]:
    lin = D1.verify_lineage_r1()  # rev3..rev3d chain + R0a artefacts + caches + config
    got = C.sha256_file(AMENDMENT_REV3E)
    if got != PIN_AMENDMENT_REV3E:
        raise R1RolloutError(f"rev3e amendment sha {got} != pinned")
    lin["amendment_rev3e"] = {"path": C.rel(AMENDMENT_REV3E), "sha256": got}
    got = C.sha256_file(D1.OUT_R1 / D1.RECEIPT_NAME)
    if got != PIN_R1_RECEIPT:
        raise R1RolloutError(f"R1 receipt sha {got} != pinned")
    for name, pin in PIN_R1_MODULE_FILES.items():
        disk = C.sha256_file(R1_MODULE / name)
        if disk != pin:
            raise R1RolloutError(f"R1 module file {name}: {disk} != pinned {pin}")
    # actor digest + 35D manifest verified BEFORE launch (rev3e preflight)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    state = W.load_module_state(R1_MODULE)
    digest = W.actor_state_digest(state)
    if digest != PIN_R1_ACTOR_DIGEST:
        raise R1RolloutError(f"R1 actor digest {digest} != pinned {PIN_R1_ACTOR_DIGEST}")
    manifest = json.loads((R1_MODULE / "actor_feature_manifest.json").read_text(encoding="utf-8"))
    names35, _, manifest_shas = VS.pinned_names()
    if manifest.get("actor_feature_names") != names35 or manifest.get("actor_feature_count") != 35 or manifest.get("actor_digest") != PIN_R1_ACTOR_DIGEST or manifest.get("manifest35_sha256") != manifest_shas["manifest35_sha256"]:
        raise R1RolloutError("R1 module 35D manifest does not match the pinned contract")
    ds = json.loads((D1.OUT_R1 / D1.RECEIPT_NAME).read_text(encoding="utf-8"))["dataset"]["files"]["npz"]["sha256"]
    if ds != PIN_R1_DATASET_NPZ:
        raise R1RolloutError("R1 dataset npz pin mismatch in the R1 receipt")
    lin["r1_receipt_sha256"] = PIN_R1_RECEIPT
    lin["r1_actor_digest"] = digest
    lin["r1_manifest_verified"] = True
    return lin


def rollout_command(python_exe: str) -> list[str]:
    return [
        python_exe, str(F1.ROLLOUT_EVAL),
        "--checkpoint", str(R1_MODULE),
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


def action_stats(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Whole-trace raw + applied action statistics (never last-row-only)."""
    raw = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float64)
    app = np.asarray([r["applied_policy_action"] for r in rows], dtype=np.float64)
    if raw.shape != app.shape or raw.shape[1] != 2 or not (np.all(np.isfinite(raw)) and np.all(np.isfinite(app))):
        raise R1RolloutError("malformed raw/applied actions")
    n = raw.shape[0]
    tail = slice(max(0, n - 20), n)
    def per_joint(x):
        return {"max_abs": [float(np.max(np.abs(x[:, j]))) for j in range(2)], "mean": [float(np.mean(x[:, j])) for j in range(2)]}
    return {
        "rows": n,
        "raw": per_joint(raw),
        "applied": per_joint(app),
        "saturation_fraction_rows_absraw_gt_1": [float(np.mean(np.abs(raw[:, j]) > 1.0)) for j in range(2)],
        "rows_raw_neq_applied": int(np.sum(np.any(raw != app, axis=1))),
        "final_20_steps_mean_raw": [float(np.mean(raw[tail, j])) for j in range(2)],
        "joints": ["knee", "ankle"],
    }


def counter_rowscan(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Cumulative FSM counters scanned over ALL rows (max AND final; never last-row-only)."""
    out = {}
    for key in ("hs_cancelled_count", "resync_count", "invalid_event_count", "valid_hs_count", "valid_to_count", "valid_cycle_count"):
        vals = []
        for i, r in enumerate(rows):
            pf = r.get("phase_fsm")
            if not isinstance(pf, Mapping) or key not in pf:
                raise R1RolloutError(f"row {i + 1}: phase_fsm without {key!r}")
            vals.append(float(pf[key]))
        out[key] = {"max_over_rows": float(max(vals)), "final": float(vals[-1])}
    return out


def compare_with_r0a(analysis: Mapping[str, Any], act: Mapping[str, Any]) -> dict[str, Any]:
    r0a_receipt = json.loads((RO.JOB_DIR / RO.RECEIPT_NAME).read_text(encoding="utf-8"))
    r0a_rows = json.loads((RO.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    r0a_act = action_stats(r0a_rows)
    a0 = r0a_receipt["analysis"]
    return {
        "r0a": {
            "steps_end": [a0["completion"]["steps"], a0["completion"]["end_reason"]],
            "knee_q": a0["trajectory_quality"]["knee_q"], "ankle_q": a0["trajectory_quality"]["ankle_q"],
            "rmse_vs_prescribed": a0["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"],
            "penetration_max_m": a0["trajectory_quality"]["penetration_m"]["max"],
            "clipping": a0["july_report_level_gates"]["clipping"]["action_clipped_steps"],
            "return": a0["july_report_level_gates"]["return"]["episode_return"],
            "final_20_steps_mean_raw": r0a_act["final_20_steps_mean_raw"],
            "saturation_fraction": r0a_act["saturation_fraction_rows_absraw_gt_1"],
            "counters": "clean (invalid 0, resync 0, hs_cancelled 0; corrected causal 0, timeouts 0/0)",
        },
        "r1": {
            "steps_end": [analysis["completion"]["steps"], analysis["completion"]["end_reason"]],
            "knee_q": analysis["trajectory_quality"]["knee_q"], "ankle_q": analysis["trajectory_quality"]["ankle_q"],
            "rmse_vs_prescribed": analysis["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"],
            "penetration_max_m": analysis["trajectory_quality"]["penetration_m"]["max"],
            "clipping": analysis["july_report_level_gates"]["clipping"]["action_clipped_steps"],
            "return": analysis["july_report_level_gates"]["return"]["episode_return"],
            "final_20_steps_mean_raw": act["final_20_steps_mean_raw"],
            "saturation_fraction": act["saturation_fraction_rows_absraw_gt_1"],
        },
        "july_homologous_only": {
            "note": "only truly homologous metrics: July-11 BC clone completed 68/500 and DAgger r2 356/500 (rollout_eval, seed 123, deterministic, 15/25 mm guards); the v3 runtime uses 20/28 mm guards and the v3 FSM, so penetration/counters are NOT directly comparable — steps-to-termination under a deterministic nominal rollout is the only homologous scalar",
            "july_bc_steps": 68, "july_dagger_r2_steps": 356,
        },
        "no_quality_conclusion": "NO biological-quality conclusion is drawn from the offline gate alone; closed-loop behaviour is judged by the architect on this receipt",
    }


def run_r1_rollout(*, authorized_stage: str | None, python_exe: str = "/opt/anaconda3/envs/envCMC-rllib/bin/python") -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise R1RolloutError(f"the R1 rollout requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_r1_rollout()
    if JOB_DIR.exists():
        raise R1RolloutError(f"no-clobber: job dir exists: {JOB_DIR} (exactly once; no retry)")
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / "r1_nominal_det_rollout.log"
    cmd = rollout_command(python_exe)
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_r1_rollout.1",
        "authorized_stage": AUTHORIZED_STAGE,
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3), "log": C.rel(log_path),
        "lineage": lineage,
        "pins": {"amendment_rev3e": PIN_AMENDMENT_REV3E, "r1_receipt": PIN_R1_RECEIPT, "r1_actor_digest": PIN_R1_ACTOR_DIGEST, "r1_dataset_npz": PIN_R1_DATASET_NPZ, "runtime_config_sha256": F1.RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": R.ROLLOUT_EVAL_SHA256_PINNED},
        "sigma": "no operational sigma exists or is used; log-std head = frozen serialisation placeholder (value 0.005 UNDECIDED; V4 decides)",
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise R1RolloutError(f"rollout returncode {proc.returncode}: fail-closed STOP, no retry (log: {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)  # validated rev3c/rev3d analysis incl. corrected whole-trace counters
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        act = action_stats(rows)
        receipt["analysis"] = analysis
        receipt["action_stats_whole_trace"] = act
        receipt["fsm_counters_rowscan"] = counter_rowscan(rows)
        receipt["comparison"] = compare_with_r0a(analysis, act)
        status = "COMPLETE" if analysis["completion"]["complete_500_time_limit"] else "ENDED_EARLY"
        if status == "ENDED_EARLY":
            receipt["stop"] = "early end: everything preserved, diagnosis in analysis/comparison; NO automatic DAgger"
    finally:
        receipt["status"] = status
        R.write_json_exclusive(JOB_DIR / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3e: single deterministic nominal rollout of V2_DAGGER_R1 (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default="/opt/anaconda3/envs/envCMC-rllib/bin/python")
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_r1_rollout()
        print(json.dumps({"mode": "dry (no rollout)", "lineage_ok": True, "command": rollout_command(args.python), "job_dir_exists": JOB_DIR.exists()}, indent=2, default=str))
        return 0
    receipt = run_r1_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "returncode": receipt["returncode"], "completion": receipt.get("analysis", {}).get("completion"), "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == "COMPLETE" else 3


if __name__ == "__main__":
    sys.exit(main())
