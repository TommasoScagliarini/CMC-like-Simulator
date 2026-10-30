"""V26B — SINGLE deterministic nominal rollout of the S0D distilled checkpoint (token-gated).

Exact rev3c/e/j command/config; primary gate 500/500 episode_time_limit; whole-trace
corrected counters; B3 ONLY in the phase window [0.55, 0.80] (not_evaluable without
valid phase/cycle); no sigma, no 3-start, no fit/IK/PPO/critic; no automatic IK on fail.
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
import v26b_r2i_rollout as RR  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402  (correct B3 phase-window function)
import v26b_s0d as S  # noqa: E402
import v26b_s0d_fit as F  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f2r_common as R  # noqa: E402


class S0DRolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S0D-NOMINAL-ROLLOUT"
S0D_MODULE = F.OUT_S0D / "rl_module"
PIN_FIT_RECEIPT = "1b2dbab43bd945ca8afb1c9651c8de969e30813a539c99623e06ff116ed9e859"
PIN_MODULE_STATE = "cda6d893138444908b4fcc908dc0045bd0df317ef0f6bbc72f70e84629e14597"
PIN_ACTOR_DIGEST = "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a"
PIN_TEST_ADDENDUM = "177126d084fe9d3b367760cd6f995aa4c7c90da752cfb9fa43f1b17b5de386b7"
TEST_ADDENDUM = C.REPO / "reports" / "user" / "2026-08-24_v26b_s0d_fit_test_addendum.md"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "s0d_nominal_det" / "S0D_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_s0d_rollout_receipt.json"


def verify_lineage() -> dict[str, Any]:
    lin = F.verify_lineage_fit()  # rev3k + pregate tool/receipt + V1
    for path, pin, key in ((F.OUT_S0D / F.RECEIPT_NAME, PIN_FIT_RECEIPT, "s0d_fit_receipt"),
                           (S0D_MODULE / "module_state.pkl", PIN_MODULE_STATE, "s0d_module_state"),
                           (TEST_ADDENDUM, PIN_TEST_ADDENDUM, "test_addendum")):
        got = C.sha256_file(Path(path))
        if got != pin:
            raise S0DRolloutError(f"{key} sha {got} != pinned {pin}")
        lin[key] = got
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    digest = W.actor_state_digest(W.load_module_state(S0D_MODULE))
    if digest != PIN_ACTOR_DIGEST:
        raise S0DRolloutError(f"actor digest {digest} != pinned {PIN_ACTOR_DIGEST}")
    if C.sha256_file(Path(F1.ROLLOUT_EVAL)) != R.ROLLOUT_EVAL_SHA256_PINNED or C.sha256_file(Path(F1.RUNTIME_CONFIG)) != F1.RUNTIME_CONFIG_SHA256:
        raise S0DRolloutError("production pins changed (rollout_eval / v3 config)")
    lin["s0d_actor_digest"] = digest
    return lin


def rollout_command(python_exe: str) -> list[str]:
    return [
        python_exe, str(F1.ROLLOUT_EVAL),
        "--checkpoint", str(S0D_MODULE),
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


def compare_chain(analysis: Mapping[str, Any]) -> dict[str, Any]:
    out = {"s0d": {"steps_end": [analysis["completion"]["steps"], analysis["completion"]["end_reason"]],
                   "rmse_vs_prescribed": analysis["trajectory_quality"]["rmse_vs_prescribed_targets_diagnostic"],
                   "penetration_max_m": analysis["trajectory_quality"]["penetration_m"]["max"]}}
    for tag, job, rn in (("r0a", RO.JOB_DIR, RO.RECEIPT_NAME), ("r1", R1.JOB_DIR, R1.RECEIPT_NAME), ("r2i", RR.JOB_DIR, RR.RECEIPT_NAME)):
        a = json.loads((job / rn).read_text(encoding="utf-8"))["analysis"]
        out[tag] = {"steps_end": [a["completion"]["steps"], a["completion"]["end_reason"]]}
    return out


def run_rollout(*, authorized_stage: str | None, python_exe: str = "/opt/anaconda3/envs/envCMC-rllib/bin/python") -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise S0DRolloutError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    if JOB_DIR.exists():
        raise S0DRolloutError(f"no-clobber: {JOB_DIR} exists (exactly once)")
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / "s0d_nominal_det_rollout.log"
    cmd = rollout_command(python_exe)
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    receipt: dict[str, Any] = {
        "schema": "v26b_s0d_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
        "command": cmd, "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3), "log": C.rel(log_path),
        "lineage": lineage,
        "pins": {"fit_receipt": PIN_FIT_RECEIPT, "module_state": PIN_MODULE_STATE, "actor_digest": PIN_ACTOR_DIGEST, "test_addendum": PIN_TEST_ADDENDUM, "rollout_eval": R.ROLLOUT_EVAL_SHA256_PINNED, "v3_config": F1.RUNTIME_CONFIG_SHA256},
        "gate_definition": {"primary_binding": "500/500 with end_reason episode_time_limit", "diagnostics": "recorded, never relaxed; B3 phase-window [0.55,0.80] only (not_evaluable without valid phase/cycle)"},
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise S0DRolloutError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)  # whole-trace corrected counters etc.
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        receipt["analysis"] = analysis
        receipt["action_stats_whole_trace"] = R1.action_stats(rows)
        receipt["fsm_counters_rowscan"] = R1.counter_rowscan(rows)
        receipt["b3_phase_window"] = PA.b3_late_stance(rows)  # correct-by-construction (window only)
        receipt["comparison_chain"] = compare_chain(analysis)
        gate = bool(analysis["completion"]["complete_500_time_limit"])
        receipt["primary_gate"] = {"complete_500_time_limit": gate, "pass": gate}
        status = "COMPLETE" if gate else "ENDED_EARLY_FAIL"
        if not gate:
            receipt["stop"] = "primary gate FAIL: preserved, diagnosed, NO relaunch, NO automatic IK"
    finally:
        receipt["status"] = status
        R.write_json_exclusive(JOB_DIR / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B S0D single nominal det rollout (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default="/opt/anaconda3/envs/envCMC-rllib/bin/python")
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "command": rollout_command(args.python)}, indent=2, default=str))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "completion": receipt.get("analysis", {}).get("completion"), "receipt_sha256": C.sha256_file(JOB_DIR / RECEIPT_NAME)}, indent=2, default=str))
    return 0 if receipt["status"] == "COMPLETE" else 3


if __name__ == "__main__":
    sys.exit(main())
