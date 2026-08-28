"""V26B rev3j — SINGLE deterministic nominal rollout of V2_R2I (sole candidate; not a PPO baseline).

Command/invariants/gates reused EXACTLY from the executed rev3c/rev3e pattern
(R0a/R1 rollouts); analysis functions imported READ-ONLY from the validated
modules; additions per rev3j: ankle negative-tract diagnostics, prescribed
reference temporal stats, three-way R0a/R1/R2I comparison.  Preregistered
binding gate: completion 500/500 ``episode_time_limit``; early end = FAIL,
diagnosed, NO relaunch.  Diagnostics recorded, never relaxed.
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

import v26b_anchors as VA  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2i as R2I  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402


class R2IRolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R2I-ROLLOUT"
AMENDMENT_REV3J = HERE / "v26b_amendment_rev3j_r2i_nominal_rollout.json"
PIN_AMENDMENT_REV3J = "723474d3692db2c1199c1e535b05f0e217a37103ef076048198eaa8ea5d9e82b"
R2I_MODULE = R2I.OUT_R2I / "rl_module"
PIN_R2I_RECEIPT = "07fdce69fd85b21519c71c73e4ec79347f0f06cfc162c19591310936eb33a2bd"
PIN_R2I_ACTOR_DIGEST = "f6579a7fdf27dc6af32d4cc6a2b9e2dbb182fc3c7cb888d83ed1a50de4490ae2"
PIN_R2I_MODULE_FILES = {
    "actor_feature_manifest.json": "2192948b40dab91dc8c5b4c99c28bbb0ebae74d241c268f5149cad80eb3bec8e",
    "class_and_ctor_args.pkl": "c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7",
    "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "module_state.pkl": "61bfb835e63629bc75899a185195e830b0b815e07d4eabec0af88e6172818635",
}
JOB_DIR = VA.OUT_ROOT / "rollouts" / "r2i_nominal_det" / "R2I_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_r2i_rollout_receipt.json"
B3_DIAGNOSTIC_NOTE = "protocol B3 reference 'ankle min <= -0.03 in late stance' recorded as DIAGNOSTIC only (rollout-blocking gate is completion 500/500 episode_time_limit, per rev3c/rev3e)"


def verify_lineage_rollout_r2i() -> dict[str, Any]:
    lin = R2I.verify_lineage_r2i()  # full chain incl. rev3i, R2G rejected-evidence, shared tooling
    got = C.sha256_file(AMENDMENT_REV3J)
    if got != PIN_AMENDMENT_REV3J:
        raise R2IRolloutError(f"rev3j amendment sha {got} != pinned")
    lin["amendment_rev3j"] = {"path": C.rel(AMENDMENT_REV3J), "sha256": got}
    got = C.sha256_file(R2I.OUT_R2I / R2I.RECEIPT_NAME)
    if got != PIN_R2I_RECEIPT:
        raise R2IRolloutError(f"R2I receipt sha {got} != pinned")
    for name, pin in PIN_R2I_MODULE_FILES.items():
        disk = C.sha256_file(R2I_MODULE / name)
        if disk != pin:
            raise R2IRolloutError(f"R2I module file {name}: {disk} != pinned {pin}")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    digest = W.actor_state_digest(W.load_module_state(R2I_MODULE))
    if digest != PIN_R2I_ACTOR_DIGEST:
        raise R2IRolloutError(f"R2I actor digest {digest} != pinned")
    manifest = json.loads((R2I_MODULE / "actor_feature_manifest.json").read_text(encoding="utf-8"))
    names35, _, manifest_shas = VS.pinned_names()
    if manifest.get("actor_feature_names") != names35 or manifest.get("actor_digest") != PIN_R2I_ACTOR_DIGEST or manifest.get("manifest35_sha256") != manifest_shas["manifest35_sha256"]:
        raise R2IRolloutError("R2I 35D manifest does not match the pinned contract")
    am = json.loads(AMENDMENT_REV3J.read_text(encoding="utf-8"))
    for label, pin in am["parents_immutable"]["rollout_tooling_reused"].items():
        disk = C.sha256_file(HERE / label)
        if disk != pin:
            raise R2IRolloutError(f"{label} changed after the rev3j amendment")
    prod = am["parents_immutable"]["production_configuration_pins"]
    if C.sha256_file(Path(F1.ROLLOUT_EVAL)) != prod["rollout_eval.py"] or C.sha256_file(Path(F1.RUNTIME_CONFIG)) != prod["v3_canonical_resolved_yaml"] or C.sha256_file(Path(R.CORRIDOR_PROFILE["path"])) != prod["morphology_corridor_profile"]:
        raise R2IRolloutError("production configuration pin mismatch (rollout_eval / v3 yaml / corridor)")
    lin["r2i_receipt_sha256"] = PIN_R2I_RECEIPT
    lin["r2i_actor_digest"] = digest
    lin["production_pins_verified"] = True
    return lin


def rollout_command(python_exe: str) -> list[str]:
    return [
        python_exe, str(F1.ROLLOUT_EVAL),
        "--checkpoint", str(R2I_MODULE),
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


def ankle_negative_tract(rows: Sequence[Mapping[str, Any]], obs: np.ndarray) -> dict[str, Any]:
    """rev3j diagnostic: presence/amplitude of the negative ankle tract + reference temporal stats."""
    ankle = obs[:, RO.IDX_ANKLE_Q].astype(np.float64)
    knee = obs[:, RO.IDX_KNEE_Q].astype(np.float64)
    neg = ankle < 0.0
    runs = []
    start = None
    for i, v in enumerate(neg):
        if v and start is None:
            start = i
        elif not v and start is not None:
            runs.append((start + 1, i, float(ankle[start:i].min()))); start = None
    if start is not None:
        runs.append((start + 1, len(neg), float(ankle[start:].min())))
    # per-cycle late-stance min (between each valid TO-1... simplified: min within each inter-HS segment when cycles exist)
    hs_steps = [r["step"] for r in rows if (r.get("phase_fsm") or {}).get("state_name") == "STANCE_AFTER_HS" and (rows[r["step"] - 2].get("phase_fsm") or {}).get("state_name") != "STANCE_AFTER_HS"] if len(rows) > 1 else []
    return {
        "ankle_min_overall": float(ankle.min()), "ankle_max_overall": float(ankle.max()),
        "negative_fraction_steps": float(np.mean(neg)), "negative_runs_start_end_min": runs[:12], "n_negative_runs": len(runs),
        "b3_reference_diagnostic": {"ankle_min_vs_-0.03": float(ankle.min()), "meets": bool(ankle.min() <= -0.03), "note": B3_DIAGNOSTIC_NOTE},
        "knee_temporal": {"p10": float(np.percentile(knee, 10)), "p50": float(np.percentile(knee, 50)), "p90": float(np.percentile(knee, 90))},
        "ankle_temporal": {"p10": float(np.percentile(ankle, 10)), "p50": float(np.percentile(ankle, 50)), "p90": float(np.percentile(ankle, 90))},
        "hs_entry_steps": hs_steps[:10],
    }


def reference_stats(t_pre: np.ndarray) -> dict[str, Any]:
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(np.asarray(t_pre, dtype=np.float64))
    tg = cc.targets[idx]  # [healthy knee q, knee qdot, ankle q, ankle qdot]
    import v26b_r0a as A
    return {"prescribed_knee_q": {"min": float(tg[:, 0].min()), "max": float(tg[:, 0].max()), "mean": float(tg[:, 0].mean())},
            "prescribed_ankle_q": {"min": float(tg[:, 2].min()), "max": float(tg[:, 2].max()), "mean": float(tg[:, 2].mean()), "negative_fraction": float(np.mean(tg[:, 2] < 0))},
            "rows": int(tg.shape[0]), "cache_digest_full": A.IK_CACHE_DIGESTS_FULL["nominal"]}


def compare_three_way(analysis: Mapping[str, Any], act: Mapping[str, Any]) -> dict[str, Any]:
    out = {"r2i": {"steps_end": [analysis["completion"]["steps"], analysis["completion"]["end_reason"]],
                   "knee_q": analysis["trajectory_quality"]["knee_q"], "ankle_q": analysis["trajectory_quality"]["ankle_q"],
                   "rmse_vs_prescribed": analysis["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"],
                   "penetration_max_m": analysis["trajectory_quality"]["penetration_m"]["max"],
                   "clipping": analysis["july_report_level_gates"]["clipping"]["action_clipped_steps"],
                   "return": analysis["july_report_level_gates"]["return"]["episode_return"],
                   "saturation_fraction": act["saturation_fraction_rows_absraw_gt_1"], "final_20_steps_mean_raw": act["final_20_steps_mean_raw"]}}
    for tag, job in (("r0a", RO.JOB_DIR), ("r1", R1.JOB_DIR)):
        rec = json.loads((job / (RO.RECEIPT_NAME if tag == "r0a" else R1.RECEIPT_NAME)).read_text(encoding="utf-8"))
        a = rec["analysis"]
        rows = json.loads((job / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        st = R1.action_stats(rows)
        out[tag] = {"steps_end": [a["completion"]["steps"], a["completion"]["end_reason"]], "knee_q": a["trajectory_quality"]["knee_q"], "ankle_q": a["trajectory_quality"]["ankle_q"], "rmse_vs_prescribed": a["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"], "penetration_max_m": a["trajectory_quality"]["penetration_m"]["max"], "clipping": a["july_report_level_gates"]["clipping"]["action_clipped_steps"], "return": a["july_report_level_gates"]["return"]["episode_return"], "saturation_fraction": st["saturation_fraction_rows_absraw_gt_1"], "final_20_steps_mean_raw": st["final_20_steps_mean_raw"]}
    return out


def run_rollout(*, authorized_stage: str | None, python_exe: str = "/opt/anaconda3/envs/envCMC-rllib/bin/python") -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise R2IRolloutError(f"the R2I rollout requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_rollout_r2i()
    if JOB_DIR.exists():
        raise R2IRolloutError(f"no-clobber: {JOB_DIR} exists (exactly once; no relaunch)")
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / "r2i_nominal_det_rollout.log"
    cmd = rollout_command(python_exe)
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_r2i_rollout.1",
        "authorized_stage": AUTHORIZED_STAGE,
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3), "log": C.rel(log_path),
        "lineage": lineage,
        "pins": {"amendment_rev3j": PIN_AMENDMENT_REV3J, "r2i_receipt": PIN_R2I_RECEIPT, "r2i_actor_digest": PIN_R2I_ACTOR_DIGEST, "r2i_module_files": PIN_R2I_MODULE_FILES, "rollout_eval": R.ROLLOUT_EVAL_SHA256_PINNED, "v3_config": F1.RUNTIME_CONFIG_SHA256, "corridor": R.CORRIDOR_PROFILE["sha256"]},
        "gate_definition": {"preregistered_binding": "completion 500/500 episode_time_limit (rev3c/rev3e); early end = FAIL, diagnosed, no relaunch", "everything_else": "diagnostics recorded, never relaxed"},
        "sigma": "no operational sigma; frozen placeholder (V4 decides)",
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise R2IRolloutError(f"rollout returncode {proc.returncode}: fail-closed STOP (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)  # validated whole-trace analysis incl. corrected counters
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        obs = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32)
        traj = DS.trajectory_from_job(JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
        act = R1.action_stats(rows)
        receipt["analysis"] = analysis
        receipt["action_stats_whole_trace"] = act
        receipt["fsm_counters_rowscan"] = R1.counter_rowscan(rows)
        receipt["ankle_negative_tract_diagnostic"] = ankle_negative_tract(rows, obs)
        receipt["prescribed_reference_stats"] = reference_stats(traj["t_pre"])
        receipt["comparison_three_way"] = compare_three_way(analysis, act)
        gate_pass = bool(analysis["completion"]["complete_500_time_limit"])
        receipt["preregistered_gate"] = {"complete_500_time_limit": gate_pass, "pass": gate_pass}
        status = "COMPLETE" if gate_pass else "ENDED_EARLY_FAIL"
        if not gate_pass:
            receipt["stop"] = "preregistered gate FAIL (early end): everything preserved, diagnosis recorded, NO relaunch"
    finally:
        receipt["status"] = status
        R.write_json_exclusive(JOB_DIR / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3j: single deterministic nominal rollout of V2_R2I (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default="/opt/anaconda3/envs/envCMC-rllib/bin/python")
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_rollout_r2i()
        print(json.dumps({"mode": "dry (no rollout)", "lineage_ok": True, "command": rollout_command(args.python), "job_dir_exists": JOB_DIR.exists()}, indent=2, default=str))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "returncode": receipt["returncode"], "completion": receipt.get("analysis", {}).get("completion"), "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == "COMPLETE" else 3


if __name__ == "__main__":
    sys.exit(main())
