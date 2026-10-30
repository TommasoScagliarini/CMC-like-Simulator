"""V26B rev3c — deterministic NOMINAL rollout of the R0a actor (exactly once) + DAgger manifest.

Per amendment ``rev3c_r0a_nominal_rollout`` (additive, pinned): ONE deterministic
nominal rollout of the R0a 35D actor under the pinned v3 canonical runtime
(config ``a870cc38…`` with detector V26 / FSM v3 / event-anchored morphology
corridor already configured — NOTHING changed).  July harness semantics:
``rollout_eval.py`` CLI, seed 123, deterministic action = Gaussian mean, no
exploration; the V26B chain passes the pinned config explicitly
(``--no-auto-config``).  Fail-closed: no-clobber, no retry; if the rollout does
not start or ends badly -> STOP + diagnosis, NO DAgger.  If it completes
(500/500 ``episode_time_limit``): the DAgger PLAN/DATASET MANIFEST is written
(manifest ONLY — no refit, no aggregation, no further rollout).
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
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_r0a as A  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class RolloutError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R0A-ROLLOUT"
AMENDMENT_REV3C = HERE / "v26b_amendment_rev3c_r0a_nominal_rollout.json"
PIN_AMENDMENT_REV3C = "723c8981263fc74d2c6e66033619c6a4d44aa4392c607a524934dbc7e403df23"
R0A_MODULE = A.OUT_R0A / "rl_module"
PIN_R0A_RECEIPT = "ff92d2fc8e730d4a9ad61c3b0f4be64741b3988196d37040cb1841260e278040"
PIN_R0A_ACTOR_DIGEST = "8567071e1185d6719290255cb1cb2f062d235a929d39b5b4767d1b613ee38959"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "r0a_nominal_det" / "R0A_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_r0a_rollout_receipt.json"
V3_PEN_SOFT_M, V3_PEN_HARD_M = 0.020, 0.028
KNEE_BOUNDS, ANKLE_BOUNDS = (-1.5, 0.0), (-0.7, 0.7)
IDX_KNEE_Q, IDX_ANKLE_Q = 2, 4  # pros_knee_angle, pros_ankle_angle in the pinned 35D manifest


def verify_lineage_rollout() -> dict[str, Any]:
    lin = A.verify_lineage_rev3b()
    got = C.sha256_file(AMENDMENT_REV3C)
    if got != PIN_AMENDMENT_REV3C:
        raise RolloutError(f"rev3c amendment sha {got} != pinned {PIN_AMENDMENT_REV3C}")
    lin["amendment_rev3c"] = {"path": C.rel(AMENDMENT_REV3C), "sha256": got}
    receipt_path = A.OUT_R0A / A.RECEIPT_NAME
    got = C.sha256_file(receipt_path)
    if got != PIN_R0A_RECEIPT:
        raise RolloutError(f"R0a receipt sha {got} != pinned {PIN_R0A_RECEIPT}")
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    for name, sha in receipt["output_files_sha256"].items():
        disk = C.sha256_file(R0A_MODULE / name)
        if disk != sha:
            raise RolloutError(f"R0a module file {name} changed: {disk} != receipt {sha}")
    if receipt["fit"]["new_actor_digest"] != PIN_R0A_ACTOR_DIGEST:
        raise RolloutError("R0a receipt actor digest != pinned")
    lin["r0a_receipt_sha256"] = PIN_R0A_RECEIPT
    lin["r0a_module_files_sha256"] = dict(receipt["output_files_sha256"])
    A.verify_ik_caches_full_productive()  # full 64-hex cache pins (rev3b rule)
    cfg_sha = C.sha256_file(Path(F1.RUNTIME_CONFIG))
    if cfg_sha != F1.RUNTIME_CONFIG_SHA256:
        raise RolloutError("v3 canonical config digest changed")
    ev_sha = C.sha256_file(Path(F1.ROLLOUT_EVAL))
    if ev_sha != R.ROLLOUT_EVAL_SHA256_PINNED:
        raise RolloutError("rollout_eval.py digest changed")
    lin["runtime_config_sha256"] = cfg_sha
    lin["rollout_eval_sha256"] = ev_sha
    return lin


def rollout_command(python_exe: str) -> list[str]:
    return [
        python_exe, str(F1.ROLLOUT_EVAL),
        "--checkpoint", str(R0A_MODULE),
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


# --- July-faithful discrete-mismatch rule ---------------------------------------------------------

def discrete_feature_indices(names: Sequence[str]) -> list[int]:
    """Exactly the July rule (target_domain_noise_adaptation._discrete_feature_indices, lines 61-72)."""
    return [i for i, n in enumerate(names) if str(n).endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated")) or str(n).startswith(("phase_fsm_", "phase_expected_"))]


def first_discrete_mismatch(reference_obs: np.ndarray, rollout_obs: np.ndarray, names: Sequence[str]) -> dict[str, Any]:
    """July semantics (truncate_before_discrete_mismatch, target_domain_noise_adaptation.py:75-101):
    per fixed-start step index, first row where ANY discrete component differs; that row and all
    later rows are label-invalid (kept prefix = first_mismatch_step - 1)."""
    idx = discrete_feature_indices(names)
    if not idx:
        raise RolloutError("no discrete features found in the 35D manifest")
    ref = np.asarray(reference_obs, dtype=np.float32)[:, idx]
    cur = np.asarray(rollout_obs, dtype=np.float32)[:, idx]
    limit = min(ref.shape[0], cur.shape[0])
    first = None
    for i in range(limit):
        if np.any(ref[i] != cur[i]):
            first = i + 1  # 1-based, July convention
            break
    kept = (first - 1) if first is not None else limit
    return {
        "rule": "July truncate_before_discrete_mismatch: exact inequality on the discrete observation components per fixed-start step",
        "discrete_feature_names": [names[i] for i in idx],
        "discrete_feature_indices": idx,
        "note_35d": "the deployable 35D manifest carries no *_saturated feature (those were 39-slice diagnostics); the July name filter matches 8 features here",
        "reference": "pinned det nominal V26 anchor trace (grid-aligned prescribed-cycle proxy; July's reference was the source actor's OWN nominal det trace, unavailable for R0a's first-ever rollout — declared in rev3c, audit-gated)",
        "compared_steps": int(limit),
        "first_mismatch_step": first,
        "label_valid_prefix_rows": int(kept),
    }


# --- corrected per-row counters (architect audit order, 2026-08-24) -------------------------------

def corrected_counters(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Architect-mandated computation over ALL rows (never inferred from end_reason / last row):

    * ``morphology_causal_contract_failure`` from ``reward_terms.morphology_causal_failed_closed``
      (rows-positive count + max), plus the causal diagnostics counters;
    * ``phase_timeout_stance`` / ``phase_timeout_swing`` counted over all rows from
      ``reward_terms.phase_timeout_exceeded`` + ``phase_timeout_side`` (1 = stance, 2 = swing)."""
    n = len(rows)
    if n == 0:
        raise RolloutError("empty trace")
    fc, te, side = [], [], []
    diag_keys = ("morphology_causal_cancelled_transition_count", "morphology_causal_dropped_pending_count", "morphology_causal_dropped_wait_hs_count", "morphology_causal_timeout_transition_count", "morphology_causal_terminal_flushed", "morphology_causal_delay_s")
    diags: dict[str, list[float]] = {k: [] for k in diag_keys}
    for i, r in enumerate(rows):
        rt = r.get("reward_terms")
        if not isinstance(rt, Mapping):
            raise RolloutError(f"row {i + 1}: no reward_terms")
        for key, dest in (("morphology_causal_failed_closed", fc), ("phase_timeout_exceeded", te), ("phase_timeout_side", side)):
            if key not in rt:
                raise RolloutError(f"row {i + 1}: reward_terms without {key!r}")
            dest.append(float(rt[key]))
        for k in diag_keys:
            if k in rt:
                diags[k].append(float(rt[k]))
    fc_a = np.asarray(fc); te_a = np.asarray(te); side_a = np.asarray(side)
    exceeded = te_a > 0.0
    stance = int(np.sum(exceeded & (side_a == 1.0)))
    swing = int(np.sum(exceeded & (side_a == 2.0)))
    other = int(np.sum(exceeded)) - stance - swing
    return {
        "rule": "architect audit 2026-08-24: computed over ALL rows from reward_terms, never inferred from end_reason or the last row only",
        "rows": n,
        "morphology_causal_contract_failure": {"rows_positive": int(np.sum(fc_a > 0.0)), "max": float(fc_a.max()), "failure": bool(np.any(fc_a > 0.0))},
        "morphology_causal_diagnostics_last_and_max": {k: {"last": (v[-1] if v else None), "max": (max(v) if v else None), "rows_present": len(v)} for k, v in diags.items()},
        "phase_timeout_stance": stance,
        "phase_timeout_swing": swing,
        "phase_timeout_exceeded_rows": int(np.sum(exceeded)),
        "phase_timeout_other_side_rows": other,
        "phase_timeout_side_coding": "1 = stance, 2 = swing (0 = none)",
    }


def write_audit_addendum(job_dir: Path) -> dict[str, Any]:
    """Content-addressed audit addendum next to an ALREADY-WRITTEN receipt (which is preserved,
    never rewritten): corrected counters recomputed from the immutable trace."""
    job_dir = Path(job_dir)
    receipt_path = job_dir / RECEIPT_NAME
    if receipt_path.is_symlink() or not receipt_path.is_file():
        raise RolloutError(f"receipt missing: {receipt_path}")
    receipt_sha = C.sha256_file(receipt_path)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    trace_path = job_dir / "rollout_policy_trace.json"
    trace_sha = C.sha256_file(trace_path)
    recorded = (receipt.get("analysis") or {}).get("trace_sha256")
    if recorded is not None and recorded != trace_sha:
        raise RolloutError(f"trace digest {trace_sha} != receipt-recorded {recorded}")
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    corrected = corrected_counters(rows)
    old_counters = (receipt.get("analysis") or {}).get("counters") or {}
    addendum = {
        "schema": "v26b_r0a_rollout_audit_addendum.1",
        "order": "architect audit 2026-08-24: morphology_causal_contract_failure from reward_terms.morphology_causal_failed_closed rows (never the end_reason string); phase_timeout_stance/swing counted over ALL rows from phase_timeout_exceeded + phase_timeout_side (1=stance, 2=swing)",
        "parent_receipt": {"path": C.rel(receipt_path), "sha256": receipt_sha, "preserved": True},
        "trace": {"path": C.rel(trace_path), "sha256": trace_sha},
        "superseded_fields_in_parent": {
            "analysis.counters.morphology.causal_contract_failure": old_counters.get("morphology", {}).get("causal_contract_failure"),
            "analysis.counters.phase_timeout": old_counters.get("phase_timeout"),
            "note": "the parent receipt values above were end_reason-inferred / last-row-only; the corrected values below are authoritative",
        },
        "corrected": corrected,
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "generated_at_utc": C.utc_now(),
    }
    path = job_dir / "v26b_r0a_rollout_receipt_audit_addendum.json"
    if path.exists():
        raise RolloutError(f"no-clobber: {path} exists")
    R.write_json_exclusive(path, addendum)
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "corrected": corrected}


# --- receipt building -----------------------------------------------------------------------------

def analyse_rollout(job_dir: Path) -> dict[str, Any]:
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    rows = json.loads((job_dir / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    summary = json.loads((job_dir / "rollout_summary.json").read_text(encoding="utf-8"))
    if int(traj["seed"]) != R.DET_SEED or str(traj["action_selection"]) != "deterministic" or traj["stochastic"]:
        raise RolloutError("rollout is not the deterministic seed-123 job defined by rev3c")
    if float(traj["episode_start_offset_s"]) != float(R.EXACT_STARTS["nominal"]):
        raise RolloutError("rollout start offset != exact nominal")
    obs = traj["obs35"].astype(np.float32)
    steps = int(traj["steps"]); end = str(traj["end_reason"])
    fsm = rows[-1].get("phase_fsm") or {}
    for key in ("timeout_exceeded", "hs_cancelled_count", "resync_count", "invalid_event_count", "valid_cycle_count", "valid_hs_count", "valid_to_count", "timeout_side", "state_name"):
        if key not in fsm:
            raise RolloutError(f"trace last row phase_fsm without {key!r}")
    events = []
    for r in rows:
        pf = r.get("phase_fsm") or {}
        if pf.get("invalid_event_this_step"):
            events.append({"step": r["step"], "kind": "invalid", "type": pf.get("invalid_event_type")})
        if pf.get("resync_event_this_step"):
            events.append({"step": r["step"], "kind": "resync"})
    pen = np.asarray([float((r.get("reward_terms") or {}).get("grf_penetration_m", np.nan)) for r in rows])
    res = np.asarray([float((r.get("reward_terms") or {}).get("reserve_norm_nm", np.nan)) for r in rows])
    if not (np.all(np.isfinite(pen)) and np.all(np.isfinite(res))):
        raise RolloutError("non-finite penetration/reserve terms in the trace")
    # trajectory quality (sign/range/bounds) + diagnostic RMSE vs prescribed targets on the grid
    knee = obs[:, IDX_KNEE_Q].astype(np.float64); ankle = obs[:, IDX_ANKLE_Q].astype(np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(np.asarray(traj["t_pre"], dtype=np.float64))
    tgt = cc.targets[idx]  # [healthy knee q, knee qdot, ankle q, ankle qdot]
    def joint_stats(x: np.ndarray, bounds: tuple[float, float]) -> dict[str, Any]:
        return {"min": float(x.min()), "max": float(x.max()), "mean": float(x.mean()), "frac_negative": float(np.mean(x < 0.0)), "outside_bounds_steps": int(np.sum((x < bounds[0]) | (x > bounds[1]))), "bounds": list(bounds)}
    ref_obs = DS.trajectory_from_job(Path(R.ANCHORS["nominal"]["job_dir"]), expected_width=R.ENV_ACTOR_WIDTH)["obs35"].astype(np.float32)
    mismatch = first_discrete_mismatch(ref_obs, obs, R.FEATURE_NAMES_35)
    july_gates = {
        "full_episode_500": {"steps": steps, "end_reason": end, "pass": bool(steps == 500 and end == "episode_time_limit")},
        "at_least_one_valid_cycle": {"valid_cycle_count": int(fsm["valid_cycle_count"]), "pass": bool(int(fsm["valid_cycle_count"]) >= 1)},
        "penetration_v3_guards": {"max_m": float(pen.max()), "soft_m": V3_PEN_SOFT_M, "hard_m": V3_PEN_HARD_M, "below_soft": bool(pen.max() <= V3_PEN_SOFT_M), "below_hard": bool(pen.max() <= V3_PEN_HARD_M)},
        "clipping": {"action_clipped_steps": int(summary["action_clipped_steps"]), "zero": bool(int(summary["action_clipped_steps"]) == 0)},
        "return": {"episode_return": float(summary["episode_return"]), "positive": bool(float(summary["episode_return"]) > 0.0)},
        "note": "July report-level actor gates recorded under the v3 20/28 mm guards (July guards were 15/25 mm); recorded, not a rollout-blocking gate at this stage",
    }
    return {
        "completion": {"steps": steps, "rows": int(obs.shape[0]), "end_reason": end, "complete_500_time_limit": bool(steps == 500 and end == "episode_time_limit")},
        "counters": {
            "phase_timeout_last_row_fsm_context_only": {"timeout_exceeded": bool(fsm["timeout_exceeded"]), "timeout_side": fsm["timeout_side"]},
            "morphology": {"settled_segments": summary.get("morphology_settled_segments"), "discarded_segments": summary.get("morphology_discarded_segments"), "settled_samples": summary.get("morphology_settled_samples"), "discarded_samples": summary.get("morphology_discarded_samples")},
            "corrected_per_row": corrected_counters(rows),
            "resync_count": int(fsm["resync_count"]), "hs_cancelled_count": int(fsm["hs_cancelled_count"]), "invalid_event_count": int(fsm["invalid_event_count"]),
            "valid_hs_count": int(fsm["valid_hs_count"]), "valid_to_count": int(fsm["valid_to_count"]), "valid_cycle_count": int(fsm["valid_cycle_count"]),
            "final_state_name": fsm["state_name"], "events": events,
        },
        "first_discrete_fsm_mismatch": mismatch,
        "trajectory_quality": {
            "knee_q": joint_stats(knee, KNEE_BOUNDS), "ankle_q": joint_stats(ankle, ANKLE_BOUNDS),
            "rmse_vs_prescribed_targets_diagnostic": {"knee_q": float(np.sqrt(np.mean((knee - tgt[:, 0]) ** 2))), "ankle_q": float(np.sqrt(np.mean((ankle - tgt[:, 2]) ** 2))), "note": "prosthetic joint vs healthy prescribed target on the grid; diagnostic only"},
            "penetration_m": {"max": float(pen.max()), "mean": float(pen.mean())},
            "reserve_norm_nm": {"max": float(res.max()), "mean": float(res.mean())},
        },
        "july_report_level_gates": july_gates,
        "trace_sha256": traj["trace_sha256"], "summary_sha256": traj["summary_sha256"], "reset_sha256": traj["reset_sha256"],
    }


# --- DAgger plan/manifest (ONLY if the rollout completed; NO refit) -------------------------------

def write_dagger_manifest(job_dir: Path, analysis: Mapping[str, Any], lineage: Mapping[str, Any]) -> dict[str, Any]:
    if not analysis["completion"]["complete_500_time_limit"]:
        raise RolloutError("DAgger manifest is only prepared for a complete 500/500 episode_time_limit rollout")
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = traj["obs35"].astype(np.float32)
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(t_pre)
    u_ik = cc.ik_action[idx]
    kept = int(analysis["first_discrete_fsm_mismatch"]["label_valid_prefix_rows"])
    uniq = len({obs[i].tobytes() for i in range(obs.shape[0])})
    manifest = {
        "schema": "v26b_dagger_plan.1",
        "status": "PLAN/MANIFEST ONLY - the DAgger refit, aggregation and any further rollout are NOT executed and NOT authorised",
        "on_policy_rows": {"source_trace_sha256": traj["trace_sha256"], "rows": int(obs.shape[0]), "bitwise_unique_rows": uniq, "start": "nominal", "seed": R.DET_SEED, "actor_digest": PIN_R0A_ACTOR_DIGEST},
        "labels": {"rule": "same-step u_IK on the fixed-start grid (July semantics behind the 'same-state' shorthand: aggregate_dagger_traces labels teacher_actions[step-1], target_domain_imitation.py:640-676; here u_IK(t_pre) from the FULL-pinned nominal cache, exact float grid lookup)", "cache_digest": A.IK_CACHE_DIGESTS_FULL["nominal"], "u_ik_finite_in_bounds": bool(np.all(np.isfinite(u_ik)) and np.all(np.abs(u_ik) <= 1.0))},
        "truncation": {**dict(analysis["first_discrete_fsm_mismatch"]), "rows_labelled_valid": kept, "rows_discarded": int(obs.shape[0]) - kept},
        "counts": {"candidate_rows_before_truncation": int(obs.shape[0]), "rows_after_truncation": kept},
        "provenance": {"amendments": {"rev3b": A.PIN_AMENDMENT_REV3B, "rev3c": PIN_AMENDMENT_REV3C}, "r0a_receipt": PIN_R0A_RECEIPT, "rollout_receipt": "sibling " + RECEIPT_NAME, "lineage": {k: v for k, v in lineage.items() if isinstance(v, (str, dict))}},
        "next_after_architect_audit_only": "aggregation with the R0a BC dataset and the mono-role refit (July round-1 analog), then preservation w.r.t. the adapted actor (source == init) per rev3b next-stage text",
        "generated_at_utc": C.utc_now(),
    }
    path = job_dir / "v26b_dagger_plan_manifest.json"
    if path.exists():
        raise RolloutError(f"no-clobber: {path} exists")
    R.write_json_exclusive(path, manifest)
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "rows_after_truncation": kept}


# --- runner ---------------------------------------------------------------------------------------

def run_rollout(*, authorized_stage: str | None, python_exe: str = sys.executable) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise RolloutError(f"the R0a rollout requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_rollout()
    if JOB_DIR.exists():
        raise RolloutError(f"no-clobber: job dir exists: {JOB_DIR} (the rollout runs EXACTLY once; no retry)")
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / "r0a_nominal_det_rollout.log"
    cmd = rollout_command(python_exe)
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    duration = round(time.time() - t0, 3)
    receipt: dict[str, Any] = {
        "schema": "v26b_r0a_rollout.1",
        "authorized_stage": AUTHORIZED_STAGE,
        "command": cmd, "returncode": int(proc.returncode), "duration_s": duration, "log": C.rel(log_path),
        "lineage": lineage,
        "pins": {"amendment_rev3c": PIN_AMENDMENT_REV3C, "r0a_receipt": PIN_R0A_RECEIPT, "r0a_actor_digest": PIN_R0A_ACTOR_DIGEST, "runtime_config_sha256": F1.RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": R.ROLLOUT_EVAL_SHA256_PINNED},
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
    }
    status = "FAILED"
    dagger = None
    try:
        if proc.returncode != 0:
            raise RolloutError(f"rollout returncode {proc.returncode}: fail-closed STOP, no retry, NO DAgger (log: {log_path})")
        analysis = analyse_rollout(JOB_DIR)
        receipt["analysis"] = analysis
        if analysis["completion"]["complete_500_time_limit"]:
            status = "COMPLETE"
            dagger = write_dagger_manifest(JOB_DIR, analysis, lineage)
            receipt["dagger_plan_manifest"] = dagger
        else:
            status = "ENDED_EARLY"
            receipt["dagger_plan_manifest"] = None
            receipt["stop"] = "rollout ended early: fail-closed STOP, diagnosis in analysis, NO DAgger manifest"
    finally:
        receipt["status"] = status
        R.write_json_exclusive(JOB_DIR / RECEIPT_NAME, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3c: deterministic nominal rollout of R0a (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default="/opt/anaconda3/envs/envCMC-rllib/bin/python")
    args = parser.parse_args(argv)
    if not args.execute:
        lineage = verify_lineage_rollout()
        print(json.dumps({"mode": "dry (no rollout)", "lineage_ok": True, "command": rollout_command(args.python), "job_dir_exists": JOB_DIR.exists()}, indent=2, default=str))
        return 0
    receipt = run_rollout(authorized_stage=args.authorized_stage, python_exe=args.python)
    print(json.dumps({"status": receipt["status"], "returncode": receipt["returncode"], "duration_s": receipt["duration_s"], "completion": receipt.get("analysis", {}).get("completion"), "dagger_plan_manifest": receipt.get("dagger_plan_manifest")}, indent=2, default=str))
    return 0 if receipt["status"] == "COMPLETE" else 3


if __name__ == "__main__":
    sys.exit(main())
