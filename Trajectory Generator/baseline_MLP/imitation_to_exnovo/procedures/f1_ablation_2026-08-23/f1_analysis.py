"""F1 analyzer: per-job metrics, contrasts and preregistered gates G1-G5.

Per-job metrics of F1 jobs are computed with the F0 library
(``f0_matrix_analysis.job_metrics`` & co., same definitions as the F0 gate:
time-domain shape, FSM counters, penetration, reserve, actions) through the
registry-free route (explicit provenance dict); metrics of the reused F0
control jobs are **read from the pinned F0 analysis JSON** (never
recomputed).  Every job is reduced to a compact record (``compact_record``)
and the gates are pure functions of compact records + receipts + refit
report, so they are unit-tested on synthetic inputs (S1) and applied
unchanged to the real outputs (S2, stage 4).

Thresholds come exclusively from ``f1_protocol.json`` (SHA recorded).
Outputs (no-clobber): ``metrics/f1_analysis_<stamp>.json/.md`` and
``gate/f1_gate_<stamp>.json``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
for entry in (str(HERE),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f0_matrix_analysis as A  # noqa: E402  (F0 library)
import f0_rollout_matrix as M  # noqa: E402  (output_digest only)
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402

RECEIPT_FILE = "f1_receipt.json"
COMPACT_KEYS = ("job_id", "candidate", "role", "family", "comparison_class", "start", "action_selection", "seed", "repeat", "source", "steps", "episode_return", "end_reason", "horizon_completed", "valid_cycles", "valid_hs", "valid_to", "invalid_events", "knee_rom", "knee_min", "knee_max", "ankle_rom", "ankle_min", "ankle_max", "ankle_negative_fraction", "penetration_max_m", "reserve_norm_max_nm", "action_abs_max", "action_clipped_steps", "resync_count", "hs_cancelled_count", "summary_sha256", "trace_sha256")


class AnalysisError(RuntimeError):
    pass


# --- compact records --------------------------------------------------------------------------


def compact_from_f0_analysis(entry: dict[str, Any], *, role: str, comparison_class: str) -> dict[str, Any]:
    m = entry["metrics"]
    ka = m["kinematics_actual"]
    ev = m["actor_fsm_events_summary"]
    fc = m.get("fsm_counters", {})
    return {
        "job_id": entry["job_id"], "candidate": entry["candidate"], "role": role, "family": entry["family"], "comparison_class": comparison_class,
        "start": entry["start"], "action_selection": entry["action_selection"], "seed": int(entry["seed"]), "repeat": int(entry.get("repeat", 1)), "source": "f0_analysis",
        "steps": int(m["steps"]), "episode_return": float(m["episode_return"]), "end_reason": str(m["end_reason"]), "horizon_completed": bool(m["horizon"]["horizon_completed"]),
        "valid_cycles": int(ev["phase_valid_cycle_count"]), "valid_hs": int(ev["phase_valid_hs_count"]), "valid_to": int(ev["phase_valid_to_count"]), "invalid_events": int(ev["invalid_event_count"]),
        "knee_rom": float(ka["knee"]["rom"]), "knee_min": float(ka["knee"]["min"]), "knee_max": float(ka["knee"]["max"]),
        "ankle_rom": float(ka["ankle"]["rom"]), "ankle_min": float(ka["ankle"]["min"]), "ankle_max": float(ka["ankle"]["max"]), "ankle_negative_fraction": float(ka["ankle"]["negative_fraction"]),
        "penetration_max_m": float(m["penetration"]["series_max_m"]), "reserve_norm_max_nm": float(m["reserve"]["series_norm_max_nm"]),
        "action_abs_max": float(m["actions"]["applied_abs_max"]), "action_clipped_steps": int(m["actions"]["clipped_steps_summary"]),
        "resync_count": fc.get("resync_count", {}).get("value"), "hs_cancelled_count": fc.get("hs_cancelled_count", {}).get("value"),
        "summary_sha256": entry.get("receipt", {}).get("summary_sha256"), "trace_sha256": entry.get("receipt", {}).get("trace_sha256"),
    }


def compact_from_metrics(receipt: dict[str, Any], m: dict[str, Any], digest: dict[str, Any]) -> dict[str, Any]:
    ka = m["kinematics_actual"]
    ev = m["actor_fsm_events_summary"]
    fc = m.get("fsm_counters", {})
    return {
        "job_id": receipt["job_id"], "candidate": receipt["candidate"], "role": receipt["role"], "family": receipt["family"], "comparison_class": receipt["comparison_class"],
        "start": receipt["start"], "action_selection": receipt["action_selection"], "seed": int(receipt["seed"]), "repeat": int(receipt["repeat"]), "source": "f1",
        "steps": int(m["steps"]), "episode_return": float(m["episode_return"]), "end_reason": str(m["end_reason"]), "horizon_completed": bool(m["horizon"]["horizon_completed"]),
        "valid_cycles": int(ev["phase_valid_cycle_count"]), "valid_hs": int(ev["phase_valid_hs_count"]), "valid_to": int(ev["phase_valid_to_count"]), "invalid_events": int(ev["invalid_event_count"]),
        "knee_rom": float(ka["knee"]["rom"]), "knee_min": float(ka["knee"]["min"]), "knee_max": float(ka["knee"]["max"]),
        "ankle_rom": float(ka["ankle"]["rom"]), "ankle_min": float(ka["ankle"]["min"]), "ankle_max": float(ka["ankle"]["max"]), "ankle_negative_fraction": float(ka["ankle"]["negative_fraction"]),
        "penetration_max_m": float(m["penetration"]["series_max_m"]), "reserve_norm_max_nm": float(m["reserve"]["series_norm_max_nm"]),
        "action_abs_max": float(m["actions"]["applied_abs_max"]), "action_clipped_steps": int(m["actions"]["clipped_steps_summary"]),
        "resync_count": fc.get("resync_count", {}).get("value"), "hs_cancelled_count": fc.get("hs_cancelled_count", {}).get("value"),
        "summary_sha256": digest.get("summary_sha256"), "trace_sha256": digest.get("trace_sha256"),
    }


def f1_job_metrics(out_dir: Path, receipt: dict[str, Any], *, names35: list[str]) -> dict[str, Any]:
    """Registry-free route through the F0 analyser for one F1 job directory."""
    summary = C.read_json(out_dir / M.SUMMARY_FILE)
    cfg_path = C.REPO / receipt["config"]
    if C.sha256_file(cfg_path) != receipt["config_sha256"]:
        raise AnalysisError("pinned config digest changed")
    cfg = C.load_yaml(cfg_path)
    rollout_eval = C.REPO / "Trajectory Generator" / "baseline_MLP" / "rollout_eval.py"
    if C.sha256_file(rollout_eval) != receipt["rollout_eval_sha256"]:
        raise AnalysisError("rollout_eval.py digest != receipt")
    defaults = A.rollout_eval_argparse_defaults(rollout_eval)
    cli = A.cli_values(receipt["command"], A.ROLLOUT_EVAL_CLI_FLAGS)
    expected = A.expected_runtime(cfg, defaults, cli, None)
    expected["expected_observation_width"] = F1.FULL_OBS_WIDTH_35
    expected["runtime"] = receipt["runtime"]
    horizon = A.expected_horizon(cfg, defaults, cli)
    problems = A.summary_evidence_matrix(summary, out_dir / M.SUMMARY_FILE, expected=expected, candidate=None)
    if problems.get("valid") is False:
        raise AnalysisError(f"summary evidence invalid: {problems}")
    evidence = A.collect_evidence(out_dir, summary, expected, candidate=None)
    if not evidence.get("_complete"):
        raise AnalysisError(f"incomplete evidence: {evidence.get('_missing_or_invalid')}")
    provenance = {"config": receipt["config"], "config_sha256": receipt["config_sha256"], "rollout_eval": C.rel(rollout_eval), "rollout_eval_sha256": receipt["rollout_eval_sha256"], "candidate_width": F1.ENV_ACTOR_WIDTH, "actor_feature_manifest": {"path": receipt["env_manifest"], "sha256": receipt["env_manifest_sha256"], "actor_feature_names": list(names35)}, "expected_runtime": expected, "horizon": horizon}
    m = A.job_metrics(out_dir, summary, receipt, receipt["runtime"], provenance)
    A.assert_finite_metrics(m)
    return m


# --- integrity helpers ----------------------------------------------------------------------------


def teacher_forward_check(trace_path: Path, adapter_trace_path: Path, teacher_arrays: dict[str, np.ndarray], spec: OA.InsertionSpec, *, tol: float = 1e-5) -> dict[str, Any]:
    rows = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    side = json.loads(Path(adapter_trace_path).read_text(encoding="utf-8"))
    if len(rows) != len(side):
        raise AnalysisError(f"trace/side-car length mismatch {len(rows)} vs {len(side)}")
    obs35 = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32)
    targets = np.asarray([s["targets"] for s in side], dtype=np.float32)
    obs39 = OA.insert_targets(obs35, targets, spec)
    logits = DS.actor_logits_numpy(teacher_arrays, obs39.astype(np.float64))
    means = [r.get("policy_action_mean") for r in rows]
    recorded = np.asarray(means if all(isinstance(m, list) for m in means) else [r["raw_policy_action"] for r in rows], dtype=np.float64)
    diff = np.abs(logits[:, :F1.ACTION_DIM] - recorded)
    return {"rows": int(len(rows)), "max_abs_error": float(diff.max()) if diff.size else 0.0, "tolerance": tol, "ok": bool(diff.size and diff.max() <= tol)}


def t_pre_sidecar_check(trace_path: Path, reset_path: Path, adapter_trace_path: Path) -> dict[str, Any]:
    rows = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    reset = json.loads(Path(reset_path).read_text(encoding="utf-8"))
    side = json.loads(Path(adapter_trace_path).read_text(encoding="utf-8"))
    expected = OA.t_pre_from_trace(float(reset["time"]), [float(r["time"]) for r in rows])
    got = [float(s["t_pre"]) for s in side]
    return {"rows": len(rows), "ok": bool(len(got) == len(expected) and all(a == b for a, b in zip(got, expected)))}


def crn_step1_check(b_trace_path: Path, c_trace_path: Path, b_arrays: dict[str, np.ndarray], *, sigma_c: float = F1.SIGMA_CONSTANT, tol: float = 1e-5) -> dict[str, Any]:
    """Common random numbers: at step 1 both runs see the same obs; the
    standard-normal draw z = noise / sigma must coincide."""
    b = json.loads(Path(b_trace_path).read_text(encoding="utf-8"))[0]
    c = json.loads(Path(c_trace_path).read_text(encoding="utf-8"))[0]
    if b["actor_observation_vector_before"] != c["actor_observation_vector_before"]:
        return {"ok": False, "reason": "step-1 observations differ"}
    obs = np.asarray(b["actor_observation_vector_before"], dtype=np.float32).astype(np.float64)[None, :]
    logits = DS.actor_logits_numpy(b_arrays, obs)[0]
    sigma_b = np.exp(logits[F1.ACTION_DIM:])
    z_b = np.asarray(b["exploration_action_noise"], dtype=np.float64) / sigma_b
    z_c = np.asarray(c["exploration_action_noise"], dtype=np.float64) / float(sigma_c)
    diff = float(np.max(np.abs(z_b - z_c)))
    return {"ok": bool(diff <= tol), "max_abs_z_diff": diff, "z_b": z_b.tolist(), "z_c": z_c.tolist(), "sigma_b_step1": sigma_b.tolist(), "tolerance": tol}


# --- adapter cross-check payload (f1_adapter_crosscheck.py, schema 2) ---------------------------

CROSSCHECK_SCHEMA_VERSION = 2
CROSSCHECK_RESULT_KEYS = ("schema_version", "observation_dtype", "rows", "cells", "recorded_representable_in_obs_dtype", "exact_rows_runtime_dtype", "exact_cells_runtime_dtype", "exact_runtime_dtype", "max_abs_diff_runtime_dtype", "float64_post_step_rows", "exact_cells_float64_post_step", "exact_float64_post_step", "max_abs_diff_float64_post_step", "exact", "f0_job_dir", "trace_sha256")
CROSSCHECK_FILE_PREFIX = "f1_adapter_crosscheck_"


def _is_hex64(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def _pos_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _zero_float(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) and float(value) == 0.0


def validate_crosscheck_payload(payload: Any, *, required_starts: Sequence[str] = F1.STARTS, expected_trace_sha: dict[str, str] | None = None) -> dict[str, Any]:
    """Fail-closed validation of the FULL payload written by f1_adapter_crosscheck.py
    (schema 2: top-level metadata + ``results`` keyed by start).  Never raises on
    malformed input; returns ``{"ok", "problems", "per_start", "all_exact", ...}``.
    ``ok`` requires, with NO tolerance: mapping with ``schema_version == 2``, a
    floating ``observation_dtype`` shared by every result, ``results`` mapping with
    every required start and no unknown start, each record well formed with
    ``recorded_representable_in_obs_dtype``, ``exact_runtime_dtype`` and
    ``exact_float64_post_step`` all True, ``exact_rows_runtime_dtype == rows > 0``,
    ``exact_cells_runtime_dtype == cells == exact_cells_float64_post_step``, both
    ``max_abs_diff_*`` exactly 0.0, hex trace SHA equal to the pinned F0 ctrl39
    trace when given, ``exact`` True, and ``all_exact`` True and consistent."""
    problems: list[str] = []
    per_start: dict[str, Any] = {}
    if not isinstance(payload, dict):
        return {"ok": False, "problems": [f"payload is {type(payload).__name__}, expected mapping"], "per_start": per_start, "all_exact": None}
    if payload.get("schema_version") != CROSSCHECK_SCHEMA_VERSION:
        problems.append(f"schema_version {payload.get('schema_version')!r} != {CROSSCHECK_SCHEMA_VERSION}")
    dtype = payload.get("observation_dtype")
    try:
        dtype_ok = isinstance(dtype, str) and np.dtype(dtype).kind == "f"
    except TypeError:
        dtype_ok = False
    if not dtype_ok:
        problems.append(f"observation_dtype {dtype!r} is not a floating numpy dtype name")
    results = payload.get("results")
    if not isinstance(results, dict):
        problems.append(f"results is {type(results).__name__}, expected mapping keyed by start")
        results = {}
    if not isinstance(payload.get("reconstructor"), dict):
        problems.append("reconstructor provenance missing or not a mapping")
    if not isinstance(payload.get("env_build"), dict) or payload.get("env_build", {}).get("steps") != 0:
        problems.append("env_build missing or steps != 0 (the cross-check must not step the simulation)")
    if payload.get("built_under_start") not in required_starts:
        problems.append(f"built_under_start {payload.get('built_under_start')!r} not a required start")
    unknown = sorted(set(results) - set(required_starts))
    if unknown:
        problems.append(f"unknown starts in results: {unknown}")
    for start in required_starts:
        rec = results.get(start)
        if not isinstance(rec, dict):
            problems.append(f"missing or malformed result for start {start!r}")
            per_start[start] = None
            continue
        missing = [k for k in CROSSCHECK_RESULT_KEYS if k not in rec]
        if missing:
            problems.append(f"{start}: missing keys {missing}")
        if rec.get("schema_version") != CROSSCHECK_SCHEMA_VERSION:
            problems.append(f"{start}: result schema_version {rec.get('schema_version')!r} != {CROSSCHECK_SCHEMA_VERSION}")
        if rec.get("observation_dtype") != dtype:
            problems.append(f"{start}: observation_dtype {rec.get('observation_dtype')!r} != payload {dtype!r}")
        rows, cells = rec.get("rows"), rec.get("cells")
        if not _pos_int(rows) or not _pos_int(cells):
            problems.append(f"{start}: rows/cells must be positive ints (got {rows!r}/{cells!r})")
        if rec.get("recorded_representable_in_obs_dtype") is not True:
            problems.append(f"{start}: recorded values not representable in the observation dtype")
        if rec.get("exact_rows_runtime_dtype") != rows or rec.get("exact_cells_runtime_dtype") != cells:
            problems.append(f"{start}: runtime-dtype exact rows/cells {rec.get('exact_rows_runtime_dtype')!r}/{rec.get('exact_cells_runtime_dtype')!r} != {rows!r}/{cells!r}")
        if rec.get("float64_post_step_rows") != rows or rec.get("exact_cells_float64_post_step") != cells:
            problems.append(f"{start}: float64 post-step rows/cells {rec.get('float64_post_step_rows')!r}/{rec.get('exact_cells_float64_post_step')!r} != {rows!r}/{cells!r}")
        for flag in ("exact_runtime_dtype", "exact_float64_post_step", "exact"):
            if rec.get(flag) is not True:
                problems.append(f"{start}: {flag} is {rec.get(flag)!r}, expected True")
        for key in ("max_abs_diff_runtime_dtype", "max_abs_diff_float64_post_step"):
            if not _zero_float(rec.get(key)):
                problems.append(f"{start}: {key} must be finite and exactly 0.0 (got {rec.get(key)!r})")
        sha = rec.get("trace_sha256")
        if not _is_hex64(sha):
            problems.append(f"{start}: trace_sha256 not a 64-hex digest")
        elif expected_trace_sha is not None and expected_trace_sha.get(start) != sha:
            problems.append(f"{start}: trace_sha256 {sha[:12]} != pinned F0 ctrl39 trace {str(expected_trace_sha.get(start))[:12]}")
        if not isinstance(rec.get("f0_job_dir"), str) or not rec.get("f0_job_dir"):
            problems.append(f"{start}: f0_job_dir missing")
        per_start[start] = {k: rec.get(k) for k in CROSSCHECK_RESULT_KEYS}
    all_exact = payload.get("all_exact")
    per_flags = [isinstance(results.get(s), dict) and results[s].get("exact") is True for s in required_starts]
    if all_exact is not True:
        problems.append(f"all_exact is {all_exact!r}, expected True")
    elif not all(per_flags):
        problems.append("all_exact True but a per-start exact flag is not True (inconsistent payload)")
    return {"ok": not problems, "problems": problems, "per_start": per_start, "all_exact": all_exact, "built_under_start": payload.get("built_under_start"), "schema_version": payload.get("schema_version"), "observation_dtype": dtype, "stamp": payload.get("stamp")}


def latest_crosscheck_file(metrics_dir: Path) -> Path | None:
    """Newest stamped cross-check payload (schema 2 files only; the legacy
    fixed-name v1 file is never considered)."""
    files = sorted(p for p in Path(metrics_dir).glob(f"{CROSSCHECK_FILE_PREFIX}*.json") if p.is_file() and not p.is_symlink())
    return files[-1] if files else None


# --- gates (pure) -----------------------------------------------------------------------------------


def _by(records: Sequence[dict[str, Any]], **where: Any) -> list[dict[str, Any]]:
    out = []
    for r in records:
        if all(r.get(k) == v for k, v in where.items()):
            out.append(r)
    return out


def _one(records: Sequence[dict[str, Any]], **where: Any) -> dict[str, Any] | None:
    found = _by(records, **where)
    if len(found) > 1:
        raise AnalysisError(f"ambiguous record selection {where}")
    return found[0] if found else None


def gate_g1(records: Sequence[dict[str, Any]], receipts: Sequence[dict[str, Any]], extras: dict[str, Any], protocol: dict[str, Any]) -> dict[str, Any]:
    """extras: f0_b_det_trace_sha {start: sha}, adapter_checks {job_id: {...}}, teacher_checks {job_id: {...}},
    sidecar_tpre_checks {job_id: {...}}, derived_modules {name: {...invariants}}, crn {(start,seed): {...}},
    dataset_receipt, refit_report, reconstruction_config_equal (bool), f0_crosscheck = FULL payload of
    f1_adapter_crosscheck.py (validated by validate_crosscheck_payload), f0_ctrl39_trace_sha {start: sha}."""
    checks: dict[str, dict[str, Any]] = {}
    gate_receipts = [r for r in receipts if r.get("gate_relevant", True)]
    bad = [r["job_id"] for r in gate_receipts if not (r.get("status") == "ok" and r.get("returncode") == 0 and r.get("summary_ok") is True and r.get("trace_present") is True and r.get("source_closure_unchanged") is True and r.get("job_inputs_unchanged") is True and r.get("closure_manifest_unchanged") is True)]
    checks["receipts_ok"] = {"ok": not bad, "failed_jobs": bad, "count": len(gate_receipts)}
    seeds = sorted({int(r["seed"]) for r in receipts})
    checks["seeds"] = {"ok": all(s in protocol["seeds"]["development"] for s in seeds), "used": seeds}
    bc = {}
    for s in F1.STARTS:
        b = _one(records, candidate="B", action_selection="deterministic", start=s, repeat=1)
        c = _one(records, candidate="C", action_selection="deterministic", start=s, repeat=1)
        bc[s] = {"b": b and b.get("trace_sha256"), "c": c and c.get("trace_sha256"), "equal": bool(b and c and b.get("trace_sha256") and b["trace_sha256"] == c["trace_sha256"])}
    checks["c_det_equals_b_det"] = {"ok": all(v["equal"] for v in bc.values()), "per_start": bc}
    f0b = extras.get("f0_b_det_trace_sha", {})
    xp = {}
    for s in F1.STARTS:
        b = _one(records, candidate="B", action_selection="deterministic", start=s, repeat=1)
        xp[s] = {"f1": b and b.get("trace_sha256"), "f0": f0b.get(s), "equal": bool(b and f0b.get(s) and b.get("trace_sha256") == f0b.get(s))}
    checks["b_det_equals_f0"] = {"ok": all(v["equal"] for v in xp.values()), "per_start": xp}
    pt = _one(records, candidate="PASSTHROUGH_B")
    bn = _one(records, candidate="B", action_selection="deterministic", start="nominal", repeat=1)
    checks["passthrough_inert"] = {"ok": bool(pt and bn and pt.get("trace_sha256") and pt["trace_sha256"] == bn.get("trace_sha256")), "passthrough": pt and pt.get("trace_sha256"), "b_nominal": bn and bn.get("trace_sha256")}
    ad = extras.get("adapter_checks", {})
    aiso_ids = [r["job_id"] for r in receipts if r.get("driver") == "f1_rollout_aiso" and r.get("gate_relevant", True)]
    checks["adapter_assertions"] = {"ok": bool(aiso_ids) and all(ad.get(j, {}).get("all_steps_asserted") is True and ad.get(j, {}).get("steps_match_rollout") is True for j in aiso_ids), "jobs": {j: ad.get(j) for j in aiso_ids}}
    tp = extras.get("sidecar_tpre_checks", {})
    checks["sidecar_t_pre"] = {"ok": bool(aiso_ids) and all(tp.get(j, {}).get("ok") is True for j in aiso_ids), "jobs": {j: tp.get(j) for j in aiso_ids}}
    checks["reconstruction_config_equal"] = {"ok": extras.get("reconstruction_config_equal") is True}
    xc = validate_crosscheck_payload(extras.get("f0_crosscheck"), expected_trace_sha=extras.get("f0_ctrl39_trace_sha"))
    checks["adapter_vs_f0_ctrl39"] = {"ok": xc["ok"] is True, "problems": xc["problems"], "per_start": xc["per_start"], "all_exact": xc["all_exact"], "built_under_start": xc.get("built_under_start"), "observation_dtype": xc.get("observation_dtype"), "stamp": xc.get("stamp"), "file": extras.get("f0_crosscheck_file")}
    tc = extras.get("teacher_checks", {})
    det_aiso = [r["job_id"] for r in receipts if r.get("candidate") == "A_ISO39_V3" and r.get("action_selection") == "deterministic"]
    checks["teacher_forward"] = {"ok": bool(det_aiso) and all(tc.get(j, {}).get("ok") is True for j in det_aiso), "jobs": {j: tc.get(j) for j in det_aiso}}
    dm = extras.get("derived_modules", {})
    checks["derived_modules"] = {"ok": all(dm.get(n, {}).get("ok") is True for n in ("C", "A_ISO39_V3_S005", "D")), "modules": dm}
    crn = extras.get("crn", {})
    rep = _one(records, candidate="B", action_selection="stochastic", start="nominal", seed=123, repeat=2)
    rep1 = _one(records, candidate="B", action_selection="stochastic", start="nominal", seed=123, repeat=1)
    checks["crn"] = {"ok": bool(crn) and all(v.get("ok") is True for v in crn.values()) and bool(rep and rep1 and rep.get("trace_sha256") == rep1.get("trace_sha256")), "pairs": crn, "b_stoch_rep2_bit_exact": bool(rep and rep1 and rep.get("trace_sha256") == rep1.get("trace_sha256"))}
    ds = extras.get("dataset_receipt") or {}
    checks["dataset"] = {"ok": bool(ds) and ds.get("train_seeds") == protocol["dataset"]["train_seeds"] and ds.get("val_seeds") == protocol["dataset"]["val_seeds"] and not (set(ds.get("trajectories_train", [])) & set(ds.get("trajectories_val", []))) and ds.get("include_det") is False and ds.get("rows_train", 0) > 0 and ds.get("rows_val", 0) > 0}
    rr = extras.get("refit_report") or {}
    checks["refit_budget"] = {"ok": bool(rr) and rr.get("epochs_run") == protocol["refit"]["budget"]["epochs"] and rr.get("selection") == "fixed_final_epoch" and rr.get("dagger_rounds") == 0 and rr.get("new_collection_after_fit") is False and rr.get("budget") == protocol["refit"]["budget"]}
    return {"gate": "G1_integrity", "pass": all(v["ok"] for v in checks.values()), "checks": checks}


def gate_g2(refit_report: dict[str, Any] | None, protocol: dict[str, Any]) -> dict[str, Any]:
    if not refit_report:
        return {"gate": "G2_fidelity", "pass": False, "status": "NOT_EVALUABLE", "reason": "refit report missing"}
    init = refit_report["init_rmse_vs_teacher"]
    fit = refit_report["fit_rmse_vs_teacher"]
    primary = fit["val"] <= 0.5 * init["val"]
    guard = fit["val"] <= 1.5 * fit["train"]
    secondary = fit["val"] <= 0.02
    return {"gate": "G2_fidelity", "pass": bool(primary and guard), "rmse_C_val": init["val"], "rmse_D_val": fit["val"], "rmse_D_train": fit["train"], "primary_rule": protocol["gates"]["G2_fidelity"]["primary"]["rule"], "primary": bool(primary), "overfit_guard": bool(guard), "secondary_informational_le_0_02": bool(secondary), "relative_reduction": float(1.0 - fit["val"] / init["val"]) if init["val"] > 0 else None}


def aiso_success(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    det = [_one(records, candidate="A_ISO39_V3", action_selection="deterministic", start=s) for s in F1.STARTS]
    if any(r is None for r in det):
        return {"ok": False, "reason": "missing A_iso det job", "present": [r is not None for r in det]}
    cycles = sum(r["valid_cycles"] for r in det)
    return {
        "ok": bool(cycles >= 4 and sum(r["valid_cycles"] >= 1 for r in det) >= 2 and sum(r["horizon_completed"] for r in det) >= 2 and sum(r["knee_rom"] >= 0.6 for r in det) >= 2),
        "sum_cycles": cycles, "starts_with_cycle": sum(r["valid_cycles"] >= 1 for r in det), "starts_horizon": sum(r["horizon_completed"] for r in det), "starts_knee_rom_ge_0_6": sum(r["knee_rom"] >= 0.6 for r in det),
        "per_start": {r["start"]: {k: r[k] for k in ("valid_cycles", "knee_rom", "horizon_completed", "end_reason", "reserve_norm_max_nm")} for r in det},
    }


def gate_g3(records: Sequence[dict[str, Any]], protocol: dict[str, Any]) -> dict[str, Any]:
    pred = aiso_success(records)
    out: dict[str, Any] = {"gate": "G3_closed_loop", "aiso_success_predicate": pred}
    if not pred["ok"]:
        out.update({"pass": False, "status": "NOT_EVALUABLE_AISO_FAILED", "interpretation": protocol["gates"]["G3_closed_loop"]["aiso_success_predicate"]["if_false"]})
        return out
    frac = float(protocol["gates"]["G3_closed_loop"]["D_vs_C_det"]["fraction_of_aiso"])
    per_start = {}
    for s in F1.STARTS:
        a = _one(records, candidate="A_ISO39_V3", action_selection="deterministic", start=s)
        c = _one(records, candidate="C", action_selection="deterministic", start=s, repeat=1)
        d = _one(records, candidate="D", action_selection="deterministic", start=s)
        if a is None or c is None or d is None:
            out.update({"pass": False, "status": "NOT_EVALUABLE", "reason": f"missing det job at {s}"})
            return out
        rom_target = c["knee_rom"] + frac * (a["knee_rom"] - c["knee_rom"])
        per_start[s] = {"cycles": {"A_iso": a["valid_cycles"], "C": c["valid_cycles"], "D": d["valid_cycles"]}, "knee_rom": {"A_iso": a["knee_rom"], "C": c["knee_rom"], "D": d["knee_rom"], "target": rom_target, "ok": d["knee_rom"] >= rom_target}, "horizon_D": d["horizon_completed"], "reserve": {"A_iso": a["reserve_norm_max_nm"], "C": c["reserve_norm_max_nm"], "D": d["reserve_norm_max_nm"], "ok": d["reserve_norm_max_nm"] <= 1.15 * max(c["reserve_norm_max_nm"], a["reserve_norm_max_nm"])}, "end_reason_D": d["end_reason"]}
    sum_a = sum(v["cycles"]["A_iso"] for v in per_start.values())
    sum_c = sum(v["cycles"]["C"] for v in per_start.values())
    sum_d = sum(v["cycles"]["D"] for v in per_start.values())
    cycles_ok = sum_d >= max(1, math.ceil(frac * sum_a)) and sum_d > sum_c
    rom_ok = sum(v["knee_rom"]["ok"] for v in per_start.values()) >= 2
    horizon_ok = sum(v["horizon_D"] for v in per_start.values()) >= 2
    safety_ok = all(v["reserve"]["ok"] for v in per_start.values()) and sum(v["end_reason_D"] != "grf_penetration" for v in per_start.values()) >= 2
    det = {"cycles_ok": cycles_ok, "sum_cycles": {"A_iso": sum_a, "C": sum_c, "D": sum_d, "required_D": max(1, math.ceil(frac * sum_a))}, "knee_rom_ok": rom_ok, "horizon_ok": horizon_ok, "safety_ok": safety_ok, "per_start": per_start}
    stoch_pairs = []
    for s in F1.STARTS:
        for k in F1.DEVELOPMENT_SEEDS:
            c = _one(records, candidate="C", action_selection="stochastic", start=s, seed=k)
            d = _one(records, candidate="D", action_selection="stochastic", start=s, seed=k)
            if c is None or d is None:
                continue
            stoch_pairs.append({"start": s, "seed": k, "horizon_C": c["horizon_completed"], "horizon_D": d["horizon_completed"], "cycles_C": c["valid_cycles"], "cycles_D": d["valid_cycles"]})
    if len(stoch_pairs) == 9:
        hf_c = sum(p["horizon_C"] for p in stoch_pairs) / 9.0
        hf_d = sum(p["horizon_D"] for p in stoch_pairs) / 9.0
        cyc_c = sum(p["cycles_C"] for p in stoch_pairs)
        cyc_d = sum(p["cycles_D"] for p in stoch_pairs)
        stoch = {"evaluable": True, "horizon_fraction": {"C": hf_c, "D": hf_d}, "sum_cycles": {"C": cyc_c, "D": cyc_d}, "ok": bool(hf_d >= hf_c and cyc_d >= cyc_c), "pairs": stoch_pairs}
    else:
        stoch = {"evaluable": False, "pairs_found": len(stoch_pairs), "ok": False}
    out.update({"D_vs_C_det": det, "D_vs_C_stoch": stoch, "pass": bool(cycles_ok and rom_ok and horizon_ok and safety_ok and stoch["ok"]), "status": "EVALUATED"})
    return out


def gate_g4(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    pairs = []
    for s in F1.STARTS:
        for k in F1.DEVELOPMENT_SEEDS:
            b = _one(records, candidate="B", action_selection="stochastic", start=s, seed=k, repeat=1)
            c = _one(records, candidate="C", action_selection="stochastic", start=s, seed=k)
            if b is None or c is None:
                continue
            pairs.append({"start": s, "seed": k, **{f"{k2}_B": b[k2] for k2 in ("steps", "valid_cycles", "episode_return", "action_clipped_steps", "penetration_max_m", "reserve_norm_max_nm", "horizon_completed")}, **{f"{k2}_C": c[k2] for k2 in ("steps", "valid_cycles", "episode_return", "action_clipped_steps", "penetration_max_m", "reserve_norm_max_nm", "horizon_completed")}})
    def mean(key: str) -> float | None:
        vals = [float(p[key]) for p in pairs]
        return float(np.mean(vals)) if vals else None
    return {"gate": "G4_sigma_contrast", "type": "diagnostic", "pairs": pairs, "paired_mean_delta_C_minus_B": {k: (mean(f"{k}_C") - mean(f"{k}_B")) if pairs else None for k in ("steps", "valid_cycles", "episode_return", "action_clipped_steps", "penetration_max_m", "reserve_norm_max_nm")}, "pairs_found": len(pairs)}


def gate_g5(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    rows = [r for r in records if r["role"] in ("A_native", "E", "A_iso", "A_iso_clock_diagnostic", "B", "C", "D") and r["action_selection"] == "deterministic"]
    return {"gate": "G5_controls", "type": "diagnostic", "non_isometric_roles": ["A_native", "E"], "rows": [{k: r.get(k) for k in ("role", "candidate", "start", "valid_cycles", "knee_rom", "ankle_min", "ankle_negative_fraction", "horizon_completed", "end_reason", "reserve_norm_max_nm", "penetration_max_m", "source")} for r in rows]}


def decide(g1: dict[str, Any], g2: dict[str, Any], g3: dict[str, Any]) -> dict[str, Any]:
    if g3.get("status") == "NOT_EVALUABLE_AISO_FAILED":
        verdict = "F1_NOT_EVALUABLE_AISO_FAILED_REDESIGN_F2"
    elif g1["pass"] and g2["pass"] and g3["pass"]:
        verdict = "F1_PASS"
    else:
        verdict = "F1_FAIL"
    return {"verdict": verdict, "G1": g1["pass"], "G2": g2["pass"], "G3": g3.get("pass"), "G3_status": g3.get("status")}


# --- real-data driver (stage 4) ---------------------------------------------------------------------


def load_f0_records() -> tuple[list[dict[str, Any]], dict[str, str], dict[str, str]]:
    if C.sha256_file(F1.F0_ANALYSIS_JSON) != F1.F0_ANALYSIS_SHA256:
        raise AnalysisError("pinned F0 analysis digest mismatch")
    payload = C.read_json(F1.F0_ANALYSIS_JSON)
    records, f0_b_det, f0_ctrl39 = [], {}, {}
    for entry in payload["jobs"]:
        if not entry.get("executed") or entry.get("verdict") != "PASS_ANALYSED":
            continue
        cand, fam = entry["candidate"], entry["family"]
        if cand == "V26_39D" and fam == "ctrl39":
            records.append(compact_from_f0_analysis(entry, role="A_native", comparison_class="compatibility_control_39D"))
            f0_ctrl39[entry["start"]] = entry.get("receipt", {}).get("trace_sha256")
        elif cand == "JUL_H0" and fam in ("det", "stoch") and entry["runtime"] == "v3_canonical":
            records.append(compact_from_f0_analysis(entry, role="E", comparison_class="historical_control"))
        elif cand == "B0820_H0" and fam == "det" and entry["runtime"] == "v3_canonical" and entry.get("repeat", 1) == 1:
            f0_b_det[entry["start"]] = entry.get("receipt", {}).get("trace_sha256")
    return records, f0_b_det, f0_ctrl39


def analyse_real(manifest_root: Path) -> dict[str, Any]:
    protocol = F1.load_protocol()
    names35, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39)
    receipts, records, extras = [], [], {"adapter_checks": {}, "teacher_checks": {}, "sidecar_tpre_checks": {}, "crn": {}, "derived_modules": {}, "f0_crosscheck": {}}
    roll = manifest_root / "rollouts"
    for rec_path in sorted(roll.glob("*/*/f1_receipt.json")):
        rec = C.read_json(rec_path)
        receipts.append(rec)
        out_dir = rec_path.parent
        if rec.get("status") != "ok":
            continue
        m = f1_job_metrics(out_dir, rec, names35=names35)
        records.append(compact_from_metrics(rec, m, M.output_digest(out_dir)))
        if rec.get("driver") == "f1_rollout_aiso":
            side = C.read_json(out_dir / "f1_adapter_summary.json")
            extras["adapter_checks"][rec["job_id"]] = {k: side.get(k) for k in ("all_steps_asserted", "steps_match_rollout", "steps", "projection_assert_count")}
            extras["sidecar_tpre_checks"][rec["job_id"]] = t_pre_sidecar_check(out_dir / M.TRACE_FILE, out_dir / "rollout_reset_diagnostics.json", out_dir / "f1_adapter_trace.json")
    import warm_start as W
    v26 = DS.load_actor_arrays(W.load_module_state(F1.CANDIDATES["A_ISO39_V3"]["module"]), expected_width=F1.MODULE_WIDTH_39)
    b_arrays = DS.load_actor_arrays(W.load_module_state(F1.CANDIDATES["B"]["module"]), expected_width=F1.ENV_ACTOR_WIDTH)
    for rec in receipts:
        if rec.get("candidate") == "A_ISO39_V3" and rec.get("status") == "ok":
            d = C.REPO / rec["output_dir"]
            extras["teacher_checks"][rec["job_id"]] = teacher_forward_check(d / M.TRACE_FILE, d / "f1_adapter_trace.json", v26, spec)
    for s in F1.STARTS:
        for k in F1.DEVELOPMENT_SEEDS:
            b = roll / "b_stoch_native" / f"B__{F1.TARGET_RUNTIME}__{s}__stoch_seed{k}" / M.TRACE_FILE
            c = roll / "c_stoch_s005" / f"C__{F1.TARGET_RUNTIME}__{s}__stoch_seed{k}" / M.TRACE_FILE
            if b.is_file() and c.is_file():
                extras["crn"][f"{s}_seed{k}"] = crn_step1_check(b, c, b_arrays)
    import f1_matrix as MX
    for name, mod in MX.DERIVED_MODULE_DIRS.items():
        if (mod / "module_state.pkl").is_file():
            st = W.load_module_state(mod)
            ok, head = SV.constant_sigma_invariants(st, sigma=F1.SIGMA_CONSTANT)
            extras["derived_modules"][name] = {"ok": ok, **head}
    ds_rec = manifest_root / "datasets" / f"f1_dataset_{MX.DATASET_STAMP}.json"
    extras["dataset_receipt"] = C.read_json(ds_rec) if ds_rec.is_file() else None
    rr = MX.DERIVED_MODULE_DIRS["D"].parent / "f1_refit_report.json"
    extras["refit_report"] = C.read_json(rr) if rr.is_file() else None
    extras["reconstruction_config_equal"] = F1.reconstruction_config_equality()["equal"]
    xc = latest_crosscheck_file(manifest_root / "metrics")
    extras["f0_crosscheck"] = C.read_json(xc) if xc is not None else None  # full schema-2 payload; validated fail-closed in gate_g1
    extras["f0_crosscheck_file"] = {"path": C.rel(xc), "sha256": C.sha256_file(xc)} if xc is not None else None
    f0_records, f0_b_det, f0_ctrl39 = load_f0_records()
    extras["f0_b_det_trace_sha"] = f0_b_det
    extras["f0_ctrl39_trace_sha"] = f0_ctrl39
    all_records = records + f0_records
    g1 = gate_g1(records, receipts, extras, protocol)
    g2 = gate_g2(extras["refit_report"], protocol)
    g3 = gate_g3(records, protocol)
    g4 = gate_g4(records)
    g5 = gate_g5(all_records)
    return {"schema_version": 1, "protocol": F1.protocol_digests(), "generated_at_utc": C.utc_now(), "git": C.git_snapshot(), "records": all_records, "receipts_count": len(receipts), "extras": {k: v for k, v in extras.items() if k != "refit_report"}, "refit_report_summary": {k: extras["refit_report"].get(k) for k in ("init_rmse_vs_teacher", "fit_rmse_vs_teacher", "epochs_run", "optimizer_steps")} if extras["refit_report"] else None, "gates": {"G1": g1, "G2": g2, "G3": g3, "G4": g4, "G5": g5}, "decision": decide(g1, g2, g3)}


def write_outputs(payload: dict[str, Any], stamp: str, *, metrics_dir: Path, gate_dir: Path) -> dict[str, str]:
    metrics_dir.mkdir(parents=True, exist_ok=True)
    gate_dir.mkdir(parents=True, exist_ok=True)
    jpath = metrics_dir / f"f1_analysis_{stamp}.json"
    C.write_json(jpath, payload)
    md = [f"# F1 analysis {stamp}", "", f"Decision: **{payload['decision']['verdict']}** (G1 {payload['decision']['G1']}, G2 {payload['decision']['G2']}, G3 {payload['decision']['G3']} / {payload['decision']['G3_status']})", ""]
    md.append(C.md_table(["role", "candidate", "start", "sel", "seed", "steps", "return", "end", "cycles", "knee ROM", "ankle min", "ankle neg frac", "horizon", "reserve max", "pen max mm", "source"], [[r["role"], r["candidate"], r["start"], r["action_selection"][:3], r["seed"], r["steps"], r["episode_return"], r["end_reason"], r["valid_cycles"], r["knee_rom"], r["ankle_min"], r["ankle_negative_fraction"], r["horizon_completed"], r["reserve_norm_max_nm"], r["penetration_max_m"] * 1000.0, r["source"]] for r in payload["records"]]))
    mpath = metrics_dir / f"f1_analysis_{stamp}.md"
    C.write_text(mpath, "\n".join(md) + "\n")
    gpath = gate_dir / f"f1_gate_{stamp}.json"
    C.write_json(gpath, {"decision": payload["decision"], "gates": payload["gates"], "protocol": payload["protocol"], "analysis": C.rel(jpath), "analysis_sha256": C.sha256_file(jpath)})
    return {"analysis": C.rel(jpath), "analysis_sha256": C.sha256_file(jpath), "markdown": C.rel(mpath), "gate": C.rel(gpath), "gate_sha256": C.sha256_file(gpath)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="F1 analysis and gates (stage 4).")
    parser.add_argument("--manifest-root", required=True)
    args = parser.parse_args(argv)
    root = Path(args.manifest_root)
    if str(F1.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(F1.BASELINE_DIR))
    payload = analyse_real(root)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    print(json.dumps(write_outputs(payload, stamp, metrics_dir=root / "metrics", gate_dir=root / "gate"), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
