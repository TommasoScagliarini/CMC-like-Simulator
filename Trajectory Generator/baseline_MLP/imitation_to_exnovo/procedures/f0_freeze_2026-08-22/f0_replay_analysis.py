"""F0 replay analysis: executed ``replay`` jobs versus their recorded references.

For every job of the ``replay`` family (historical configuration replayed on
the current code) this module:
  1. re-verifies the output directory fail-closed through the driver's own
     ``verify_existing``/``preflight_all`` (receipt fields, status ok,
     returncode 0, summary.ok, trace present, SHA-256 of summary and trace
     recomputed from disk);
  2. records the EVIDENCE on both sides (new output and recorded reference):
     presence, SHA-256, parseability, expected columns, row count consistent
     with the summary step count (10 samples per 0.01 s policy step at 1 kHz),
     finiteness, trace schema (actions and observation vectors on every step);
  3. compares the summary field by field, every expected time series
     (prosthetic kinematics, normalized SEA controls ``u_sea`` from
     ``rollout_episode_sea_controls.sto`` - NOT served kinematics, see
     output.py:461-462 - SEA torques, left online GRF, prosthetic reserve
     torques) on the same time grid, and the policy trace step by step (raw
     actions and actor observation vectors);
  4. emits a verdict that claims full identity ONLY when the complete
     expected evidence set is available, verified and identical on both
     sides; otherwise it names the identical/divergent/missing subsets.

Verdicts:
  FAIL_NO_RECEIPT / FAIL_VERIFICATION          - receipt missing or fail-closed check failed
  FAIL_OUTPUT_INCOMPLETE                       - new output evidence missing/invalid/non-finite
  PASS_EXECUTED_NO_REFERENCE                   - complete new evidence, job has no recorded reference
  PASS_FULL_IDENTICAL_TO_REFERENCE             - complete evidence on both sides, everything identical
  PASS_EXECUTED_DIVERGENT_FROM_REFERENCE       - complete evidence on both sides, at least one difference
  INCOMPLETE_REFERENCE_EVIDENCE_IDENTICAL_ON_SUBSET[...]
  INCOMPLETE_REFERENCE_EVIDENCE_DIVERGENT_ON_SUBSET[...] - reference evidence incomplete; subsets named
``analysis_complete`` is true only for the PASS_FULL_IDENTICAL / PASS_EXECUTED_
DIVERGENT / PASS_EXECUTED_NO_REFERENCE outcomes.

Trace classification: an observation width mismatch or missing vectors is an
environment/schema divergence, never identity and never a policy-side
finding. Writes ``metrics/replay_analysis_<stamp>.json/.md`` (no-clobber).
"""

from __future__ import annotations

import glob
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

TOL = 1e-9
SAMPLES_PER_STEP = 10  # 1 kHz physical sampling, 0.01 s policy step
ROWS_TOLERANCE = 1
SUMMARY_FIELDS = (
    "steps", "episode_return", "end_reason", "phase_valid_hs_count", "phase_valid_to_count",
    "phase_valid_cycle_count", "invalid_event_count", "grf_penetration_max_m", "reserve_norm_max_nm",
    "action_abs_max", "action_clipped_steps", "exploration_noise_realized_rms", "exploration_std_mean",
    "morphology_settled_segments", "morphology_discarded_samples", "n_actor",
)
# Explicit expected evidence: file -> columns that must be present and are compared.
EXPECTED_STO: dict[str, tuple[str, ...]] = {
    "rollout_episode_kinematics.sto": ("pros_knee_angle", "pros_ankle_angle"),
    "rollout_episode_sea_controls.sto": ("pros_knee_angle", "pros_ankle_angle"),
    "rollout_episode_sea_torques.sto": ("SEA_Knee_tau_spring", "SEA_Ankle_tau_spring", "SEA_Knee_tau_motor", "SEA_Ankle_tau_motor"),
    "rollout_episode_online_grf.sto": ("left_normal_force", "left_penetration", "left_in_contact"),
    "rollout_episode_reserve_torques.sto": ("pros_knee_angle_reserve_torque", "pros_ankle_angle_reserve_torque"),
}
EXPECTED_EVIDENCE: tuple[str, ...] = ("summary", *EXPECTED_STO.keys(), "trace")
assert EXPECTED_STO, "expected series set must be explicit and non-empty"
SCHEMA_DIVERGENCE = "environment/schema divergence: observation width mismatch or missing vectors (never identity, never policy-side)"


# --- low-level readers -------------------------------------------------------


def read_sto(path: Path) -> tuple[list[str], np.ndarray]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    idx = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    names = lines[idx + 1].split()
    rows = [[float(x) for x in line.split()] for line in lines[idx + 2:] if line.strip()]
    data = np.asarray(rows, dtype=np.float64)
    if data.ndim != 2 or data.shape[1] != len(names):
        raise ValueError(f"malformed .sto table: {path}")
    return names, data


def column(names: list[str], data: np.ndarray, name: str) -> np.ndarray | None:
    if name not in names:
        return None
    return data[:, names.index(name)]


def _sha(path: Path) -> str | None:
    return C.sha256_file(path) if path.is_file() else None


# --- evidence ------------------------------------------------------------------


def sto_evidence(path: Path, columns: tuple[str, ...], steps: int | None) -> dict[str, Any]:
    rec: dict[str, Any] = {"path": C.rel(path), "present": path.is_file(), "sha256": _sha(path), "expected_columns": list(columns)}
    if not rec["present"]:
        rec.update({"parse_ok": False, "columns_present": [], "columns_missing": list(columns), "rows": None, "rows_expected": None, "rows_consistent": False, "time_present": False, "finite": False, "valid": False})
        return rec
    try:
        names, data = read_sto(path)
        rec["parse_ok"] = True
    except Exception as exc:  # noqa: BLE001 - any parse failure invalidates the evidence
        rec.update({"parse_ok": False, "parse_error": f"{type(exc).__name__}: {exc}", "columns_present": [], "columns_missing": list(columns), "rows": None, "rows_expected": None, "rows_consistent": False, "time_present": False, "finite": False, "valid": False})
        return rec
    rec["columns_present"] = [c for c in columns if c in names]
    rec["columns_missing"] = [c for c in columns if c not in names]
    rec["time_present"] = "time" in names
    rec["rows"] = int(data.shape[0])
    rec["rows_expected"] = (int(steps) * SAMPLES_PER_STEP) if steps is not None else None
    rec["rows_consistent"] = rec["rows_expected"] is not None and abs(rec["rows"] - rec["rows_expected"]) <= ROWS_TOLERANCE
    cols = [column(names, data, c) for c in rec["columns_present"]]
    rec["nonfinite_count"] = int(sum(int(np.count_nonzero(~np.isfinite(c))) for c in cols)) + (int(np.count_nonzero(~np.isfinite(column(names, data, "time")))) if rec["time_present"] else 0)
    rec["finite"] = rec["nonfinite_count"] == 0
    rec["valid"] = bool(rec["parse_ok"] and not rec["columns_missing"] and rec["time_present"] and rec["rows_consistent"] and rec["finite"])
    return rec


def trace_evidence(path: Path, steps: int | None) -> dict[str, Any]:
    rec: dict[str, Any] = {"path": C.rel(path), "present": path.is_file(), "sha256": _sha(path)}
    if not rec["present"]:
        rec.update({"parse_ok": False, "steps": None, "steps_consistent": False, "actions_complete": False, "observation_vectors_complete": False, "observation_widths": [], "finite": False, "valid": False})
        return rec
    try:
        rows = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(rows, list):
            raise ValueError("trace is not a list")
        rec["parse_ok"] = True
    except Exception as exc:  # noqa: BLE001
        rec.update({"parse_ok": False, "parse_error": f"{type(exc).__name__}: {exc}", "steps": None, "steps_consistent": False, "actions_complete": False, "observation_vectors_complete": False, "observation_widths": [], "finite": False, "valid": False})
        return rec
    rec["steps"] = len(rows)
    rec["steps_consistent"] = steps is not None and len(rows) == int(steps)
    actions = [r.get("raw_policy_action") if isinstance(r, dict) else None for r in rows]
    obs = [r.get("actor_observation_vector_before") if isinstance(r, dict) else None for r in rows]
    rec["actions_complete"] = bool(rows) and all(isinstance(a, list) and len(a) > 0 for a in actions)
    rec["observation_vectors_complete"] = bool(rows) and all(isinstance(o, list) and len(o) > 0 for o in obs)
    rec["observation_widths"] = sorted({len(o) for o in obs if isinstance(o, list)})
    finite = True
    for a in actions:
        if isinstance(a, list) and not all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in a):
            finite = False
    for o in obs:
        if isinstance(o, list) and not all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in o):
            finite = False
    rec["finite"] = finite
    rec["valid"] = bool(rec["steps_consistent"] and rec["actions_complete"] and rec["observation_vectors_complete"] and len(rec["observation_widths"]) == 1 and finite)
    return rec


def summary_evidence(summary: dict[str, Any] | None, path: Path | None) -> dict[str, Any]:
    rec: dict[str, Any] = {"path": C.rel(path) if path else None, "present": summary is not None, "sha256": _sha(path) if path else None}
    if summary is None:
        rec.update({"finite": False, "valid": False})
        return rec
    nonfinite = [k for k in SUMMARY_FIELDS if isinstance(summary.get(k), float) and not math.isfinite(summary[k])]
    rec["nonfinite_fields"] = nonfinite
    rec["finite"] = not nonfinite
    rec["steps"] = summary.get("steps")
    rec["ok"] = summary.get("ok")
    rec["valid"] = bool(summary.get("ok") is True and not nonfinite and isinstance(summary.get("steps"), int))
    return rec


def collect_evidence(out_dir: Path | None, summary: dict[str, Any] | None, summary_path: Path | None) -> dict[str, Any]:
    steps = summary.get("steps") if isinstance(summary, dict) else None
    ev: dict[str, Any] = {"summary": summary_evidence(summary, summary_path)}
    if out_dir is None:
        for fname in EXPECTED_STO:
            ev[fname] = {"present": False, "valid": False, "sha256": None, "path": None}
        ev["trace"] = {"present": False, "valid": False, "sha256": None, "path": None}
    else:
        for fname, cols in EXPECTED_STO.items():
            ev[fname] = sto_evidence(out_dir / "sim_outputs" / fname, cols, steps)
        ev["trace"] = trace_evidence(out_dir / M.TRACE_FILE, steps)
    ev["_complete"] = all(ev[item].get("valid") is True for item in EXPECTED_EVIDENCE)
    ev["_missing_or_invalid"] = [item for item in EXPECTED_EVIDENCE if ev[item].get("valid") is not True]
    return ev


# --- comparisons ---------------------------------------------------------------


def compare_series(new: np.ndarray | None, ref: np.ndarray | None, time_new: np.ndarray | None, time_ref: np.ndarray | None) -> dict[str, Any]:
    """Same-length requirement, shared time grid, NaN/Inf detection, and a
    separation between ``overlap_identical`` (identical on the common prefix)
    and ``full_identical`` (same length, same grid, finite, identical everywhere)."""
    if new is None or ref is None:
        return {"available": False, "overlap_identical": False, "full_identical": False}
    rows_new, rows_ref = int(len(new)), int(len(ref))
    n = min(rows_new, rows_ref)
    nonfinite_new = int(np.count_nonzero(~np.isfinite(new)))
    nonfinite_ref = int(np.count_nonzero(~np.isfinite(ref)))
    finite = nonfinite_new == 0 and nonfinite_ref == 0
    same_length = rows_new == rows_ref
    if time_new is None or time_ref is None or len(time_new) < n or len(time_ref) < n:
        time_grid_match = False
    else:
        time_grid_match = bool(n > 0 and np.all(np.abs(np.asarray(time_new[:n]) - np.asarray(time_ref[:n])) <= TOL))
    out: dict[str, Any] = {
        "available": n > 0,
        "rows_new": rows_new,
        "rows_ref": rows_ref,
        "same_length": same_length,
        "rows_compared": n,
        "time_grid_match_on_overlap": time_grid_match,
        "nonfinite_new": nonfinite_new,
        "nonfinite_ref": nonfinite_ref,
        "finite": finite,
    }
    if n == 0:
        out.update({"overlap_identical": False, "full_identical": False})
        return out
    diff = np.abs(new[:n] - ref[:n])
    with np.errstate(invalid="ignore"):
        exceeds = diff > TOL
    exceeds = exceeds | ~np.isfinite(diff)
    first = int(np.argmax(exceeds)) if bool(np.any(exceeds)) else None
    overlap_identical = finite and time_grid_match and first is None
    out.update(
        {
            "rms_diff": float(np.sqrt(np.nanmean(np.square(new[:n] - ref[:n])))) if finite else None,
            "max_abs_diff": float(np.nanmax(diff)) if finite else None,
            "first_divergence_index": first,
            "first_divergence_time_s": (float(time_new[first]) if (first is not None and time_new is not None and first < len(time_new)) else None),
            "overlap_identical": bool(overlap_identical),
            "full_identical": bool(overlap_identical and same_length),
        }
    )
    return out


def compare_summaries(new: dict[str, Any], ref: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    all_equal = True
    nonfinite = False
    for field in SUMMARY_FIELDS:
        a, b = new.get(field), ref.get(field)
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
            if not (math.isfinite(float(a)) and math.isfinite(float(b))):
                nonfinite = True
                equal, delta = False, None
            else:
                equal = math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=TOL)
                delta = float(a) - float(b)
        elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
            try:
                equal = all(math.isclose(float(x), float(y), rel_tol=0.0, abs_tol=TOL) for x, y in zip(a, b))
                delta = [float(x) - float(y) for x, y in zip(a, b)]
            except (TypeError, ValueError):
                equal, delta = a == b, None
        else:
            equal, delta = a == b, None
        if field in ref or field in new:
            all_equal = all_equal and equal
        out[field] = {"new": a, "reference": b, "equal": equal, "delta": delta}
    out["_all_equal"] = bool(all_equal and not nonfinite)
    out["_nonfinite"] = nonfinite
    return out


def compare_traces(new_rows: list[dict[str, Any]], ref_rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Step-by-step comparison of raw actions and actor observation vectors.

    Width mismatch or missing vectors/actions on any compared step are an
    environment/schema divergence (never identity, never policy-side)."""
    steps_new, steps_ref = len(new_rows), len(ref_rows)
    n = min(steps_new, steps_ref)
    out: dict[str, Any] = {"available": n > 0, "steps_new": steps_new, "steps_ref": steps_ref, "same_length": steps_new == steps_ref, "steps_compared": n}
    if n == 0:
        out.update({"schema_divergence": True, "interpretation": SCHEMA_DIVERGENCE + " (no overlapping steps)", "overlap_identical": False, "full_identical": False})
        return out
    acts_new = [r.get("raw_policy_action") for r in new_rows[:n]]
    acts_ref = [r.get("raw_policy_action") for r in ref_rows[:n]]
    obs_new = [r.get("actor_observation_vector_before") for r in new_rows[:n]]
    obs_ref = [r.get("actor_observation_vector_before") for r in ref_rows[:n]]
    missing = [i for i in range(n) if not (isinstance(acts_new[i], list) and isinstance(acts_ref[i], list) and isinstance(obs_new[i], list) and isinstance(obs_ref[i], list) and acts_new[i] and acts_ref[i] and obs_new[i] and obs_ref[i])]
    width_mismatch = [i for i in range(n) if i not in missing and (len(obs_new[i]) != len(obs_ref[i]) or len(acts_new[i]) != len(acts_ref[i]))]
    out["missing_vector_steps"] = missing[:20]
    out["width_mismatch_steps"] = width_mismatch[:20]
    out["observation_widths_new"] = sorted({len(o) for o in obs_new if isinstance(o, list)})
    out["observation_widths_ref"] = sorted({len(o) for o in obs_ref if isinstance(o, list)})
    if missing or width_mismatch:
        out.update({"schema_divergence": True, "observation_width_match": not width_mismatch, "interpretation": SCHEMA_DIVERGENCE, "overlap_identical": False, "full_identical": False})
        return out
    out["schema_divergence"] = False
    out["observation_width_match"] = True
    act_new = np.asarray(acts_new, dtype=np.float64)
    act_ref = np.asarray(acts_ref, dtype=np.float64)
    ob_new = np.asarray(obs_new, dtype=np.float64)
    ob_ref = np.asarray(obs_ref, dtype=np.float64)
    finite = bool(np.all(np.isfinite(act_new)) and np.all(np.isfinite(act_ref)) and np.all(np.isfinite(ob_new)) and np.all(np.isfinite(ob_ref)))
    out["finite"] = finite
    act_diff = np.max(np.abs(act_new - act_ref), axis=1)
    obs_diff = np.max(np.abs(ob_new - ob_ref), axis=1)
    first_act = int(np.argmax(act_diff > TOL)) if bool(np.any(act_diff > TOL)) else None
    first_obs = int(np.argmax(obs_diff > TOL)) if bool(np.any(obs_diff > TOL)) else None
    names = new_rows[0].get("actor_observation_before") if isinstance(new_rows[0].get("actor_observation_before"), dict) else None
    divergent_features = None
    if first_obs is not None and names:
        keys = list(names.keys())
        divergent_features = [keys[j] for j in range(min(len(keys), ob_new.shape[1])) if abs(ob_new[first_obs, j] - ob_ref[first_obs, j]) > TOL][:12]
    overlap_identical = finite and first_act is None and first_obs is None
    if not finite:
        interpretation = "non-finite values in actions/observations: comparison invalid"
    elif overlap_identical:
        interpretation = "identical actions and observations over the compared steps"
    elif first_obs is not None and (first_act is None or first_obs <= first_act):
        interpretation = "observations diverge at or before the first action divergence: the difference enters through the environment/runtime (detector, FSM, served reference, reward-side observations), not through the policy weights"
    else:
        interpretation = "actions diverge before any observation difference (same schema): policy-side divergence in the inference path - investigate"
    out.update(
        {
            "action_rms_diff": float(np.sqrt(np.mean(np.square(act_new - act_ref)))) if finite else None,
            "action_max_abs_diff": float(np.max(act_diff)) if finite else None,
            "first_action_divergence_step_index": first_act,
            "first_observation_divergence_step_index": first_obs,
            "divergent_observation_features_at_first_step": divergent_features,
            "interpretation": interpretation,
            "overlap_identical": bool(overlap_identical),
            "full_identical": bool(overlap_identical and steps_new == steps_ref),
        }
    )
    return out


# --- job-level analysis ----------------------------------------------------------


def analyse_outputs(new_dir: Path, ref_dir: Path | None, new_summary: dict[str, Any], ref_summary: dict[str, Any] | None, ref_summary_path: Path | None) -> dict[str, Any]:
    """Evidence-gated comparison of one executed output against its reference."""
    ev_new = collect_evidence(new_dir, new_summary, new_dir / M.SUMMARY_FILE)
    result: dict[str, Any] = {"evidence_new": ev_new, "expected_evidence": list(EXPECTED_EVIDENCE)}
    if not ev_new["_complete"]:
        result.update({"analysis_complete": False, "verdict": "FAIL_OUTPUT_INCOMPLETE", "verdict_reason": f"new output evidence missing or invalid: {ev_new['_missing_or_invalid']}"})
        return result
    if ref_dir is None or ref_summary is None:
        result.update({"analysis_complete": True, "verdict": "PASS_EXECUTED_NO_REFERENCE", "verdict_reason": "complete new evidence; no recorded reference for this job"})
        return result
    ev_ref = collect_evidence(ref_dir, ref_summary, ref_summary_path)
    ev_ref["_auxiliary_verification"] = "content-consistency only (presence, parse, expected columns, rows vs summary steps, finiteness, SHA-256 recorded now); no prior digest of the auxiliary reference files exists"
    result["evidence_ref"] = ev_ref
    comparisons: dict[str, Any] = {"summary": compare_summaries(new_summary, ref_summary)}
    identical: list[str] = []
    divergent: list[str] = []
    missing_ref: list[str] = []
    if ev_ref["summary"]["valid"]:
        (identical if comparisons["summary"]["_all_equal"] else divergent).append("summary")
    else:
        missing_ref.append("summary")
    for fname, cols in EXPECTED_STO.items():
        if not ev_ref[fname]["valid"]:
            missing_ref.append(fname)
            continue
        n_names, n_data = read_sto(new_dir / "sim_outputs" / fname)
        r_names, r_data = read_sto(ref_dir / "sim_outputs" / fname)
        t_new, t_ref = column(n_names, n_data, "time"), column(r_names, r_data, "time")
        per_col = {col: compare_series(column(n_names, n_data, col), column(r_names, r_data, col), t_new, t_ref) for col in cols}
        comparisons[fname] = per_col
        (identical if all(per_col[c]["full_identical"] for c in cols) else divergent).append(fname)
    if ev_ref["trace"]["valid"]:
        new_rows = json.loads((new_dir / M.TRACE_FILE).read_text(encoding="utf-8"))
        ref_rows = json.loads((ref_dir / M.TRACE_FILE).read_text(encoding="utf-8"))
        comparisons["trace"] = compare_traces(new_rows, ref_rows)
        (identical if comparisons["trace"]["full_identical"] else divergent).append("trace")
    else:
        missing_ref.append("trace")
    result["comparisons"] = comparisons
    result["identical_subset"] = identical
    result["divergent_subset"] = divergent
    result["missing_reference_evidence"] = missing_ref
    if ev_ref["_complete"] and not missing_ref:
        result["analysis_complete"] = True
        if not divergent and set(identical) == set(EXPECTED_EVIDENCE):
            result["verdict"] = "PASS_FULL_IDENTICAL_TO_REFERENCE"
            result["verdict_reason"] = "complete evidence on both sides; summary, all expected series and the trace are identical"
        else:
            result["verdict"] = "PASS_EXECUTED_DIVERGENT_FROM_REFERENCE"
            result["verdict_reason"] = f"complete evidence on both sides; divergent: {divergent}"
    else:
        result["analysis_complete"] = False
        tag = "IDENTICAL_ON_SUBSET" if not divergent else "DIVERGENT_ON_SUBSET"
        result["verdict"] = f"INCOMPLETE_REFERENCE_EVIDENCE_{tag}[{'+'.join(identical) or 'none'}]"
        result["verdict_reason"] = f"reference evidence missing or invalid: {missing_ref}; identical on {identical}; divergent on {divergent}"
    return result


def analyse_job(rec: dict[str, Any]) -> dict[str, Any]:
    out_dir = C.REPO / rec["output_dir"]
    receipt_path = out_dir / M.RECEIPT_FILE
    receipt = C.read_json(receipt_path) if receipt_path.is_file() else None
    result: dict[str, Any] = {
        "job_id": rec["job_id"],
        "candidate": rec["candidate"],
        "runtime": rec["runtime"],
        "start": rec["start"],
        "action_selection": rec["action_selection"],
        "seed": rec["seed"],
        "comparison_class": rec["comparison_class"],
        "output_dir": rec["output_dir"],
        "receipt_present": receipt is not None,
        "receipt_status": (receipt or {}).get("status"),
        "receipt_returncode": (receipt or {}).get("returncode"),
        "receipt_duration_s": (receipt or {}).get("duration_s"),
        "receipt_summary_ok": (receipt or {}).get("summary_ok"),
        "receipt_trace_present": (receipt or {}).get("trace_present"),
        "receipt_summary_sha256": (receipt or {}).get("summary_sha256"),
        "receipt_trace_sha256": (receipt or {}).get("trace_sha256"),
        "historical_reference_summary": rec.get("historical_reference_summary"),
        "historical_reference_summary_sha256": rec.get("historical_reference_summary_sha256"),
    }
    if receipt is None:
        result.update({"analysis_complete": False, "verdict": "FAIL_NO_RECEIPT"})
        return result
    try:
        M.verify_existing(rec, out_dir)
        result["independent_verification"] = "PASS"
    except RuntimeError as exc:
        result.update({"independent_verification": f"FAIL: {str(exc)[:600]}", "analysis_complete": False, "verdict": "FAIL_VERIFICATION"})
        return result
    new_summary = C.read_json(out_dir / M.SUMMARY_FILE)
    result["new_summary"] = {k: new_summary.get(k) for k in SUMMARY_FIELDS}
    ref_rel = rec.get("historical_reference_summary")
    ref_path = (C.REPO / ref_rel) if ref_rel else None
    ref_summary = C.read_json(ref_path) if (ref_path is not None and ref_path.is_file()) else None
    if ref_path is not None:
        result["reference_recorded_at"] = C.iso_mtime(ref_path) if ref_path.is_file() else None
        result["reference_checkpoint_field"] = (ref_summary or {}).get("checkpoint")
    result.update(analyse_outputs(out_dir, ref_path.parent if ref_path is not None else None, new_summary, ref_summary, ref_path))
    return result


def main() -> int:
    C.ensure_out_dirs()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_json = C.OUT_METRICS / f"replay_analysis_{stamp}.json"
    out_md = C.OUT_METRICS / f"replay_analysis_{stamp}.md"
    interpreter = C.select_python()
    python_exe = interpreter["selected"]
    all_described = [M.describe_job(j, python_exe, None) for j in M.build_jobs()]
    preflight = M.preflight_all(all_described)  # raises fail-closed if any existing output is invalid
    replay = [d for d in all_described if d["family"] == "replay"]
    manifests = sorted(glob.glob(str(C.OUT_ROLLOUTS / "rollout_matrix_manifest_execute_*.json")))
    statuses = sorted(glob.glob(str(C.OUT_ROLLOUTS / "rollout_matrix_status_*.json")))
    results = [analyse_job(d) for d in replay]
    verdict_counts: dict[str, int] = {}
    for r in results:
        verdict_counts[r["verdict"]] = verdict_counts.get(r["verdict"], 0) + 1
    payload = {
        "schema_version": 2,
        "revision": C.F0_REV,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "interpreter": {k: interpreter.get(k) for k in ("selected", "source")},
        "execute_manifests": [C.rel(m) for m in manifests],
        "status_files": [C.rel(s) for s in statuses],
        "preflight_independent": preflight,
        "tolerance_abs": TOL,
        "samples_per_step": SAMPLES_PER_STEP,
        "expected_evidence": list(EXPECTED_EVIDENCE),
        "expected_series": {k: list(v) for k, v in EXPECTED_STO.items()},
        "verdict_counts": verdict_counts,
        "technical_execution_ok": all(r.get("receipt_status") == "ok" and r.get("independent_verification") == "PASS" for r in results) and len(results) == 9,
        "all_analyses_complete": all(r.get("analysis_complete") is True for r in results),
        "note": "replay = historical resolved yaml + historical weights on current HEAD code; a divergence from the recorded reference measures code changes since the reference date and is expected, not a failure; full identity is claimed only with complete evidence on both sides. The technical execution gate (9/9 ok) is distinct from the behavioural gate.",
        "jobs": results,
    }
    C.write_json(out_json, payload)

    def f(v: Any, nd: int = 6) -> str:
        return f"{v:.{nd}g}" if isinstance(v, float) else str(v)

    def sc(r: dict[str, Any], field: str, key: str) -> Any:
        return (((r.get("comparisons") or {}).get("summary") or {}).get(field) or {}).get(key)

    def ser(r: dict[str, Any], fname: str, col: str, key: str) -> Any:
        return (((r.get("comparisons") or {}).get(fname) or {}).get(col) or {}).get(key)

    lines = [
        f"# F0 — Analisi dei job replay rispetto ai riferimenti registrati — revisione {C.F0_REV} — stamp {stamp}",
        "",
        f"Generato: {payload['generated_at_utc']} — git HEAD `{payload['git']['head'][:12]}`. Manifest di esecuzione: {', '.join(payload['execute_manifests']) or '-'}. Preflight indipendente: {len(preflight['verified_identical'])} output verificati, {len(preflight['invalid'])} invalidi. Esecuzione tecnica 9/9 ok: **{payload['technical_execution_ok']}**; analisi complete su tutti i job: **{payload['all_analyses_complete']}**.",
        "",
        payload["note"],
        "",
        f"Evidenza attesa per l'identita' piena: {', '.join(EXPECTED_EVIDENCE)}; serie confrontate: " + "; ".join(f"{k}: {', '.join(v)}" for k, v in EXPECTED_STO.items()) + ".",
        "",
        "## Esito per job",
        "",
        C.md_table(
            ["job", "verifica", "rc", "durata [s]", "verdetto", "analisi completa", "identici", "divergenti", "evidenza ref mancante", "steps new/ref", "return new", "return ref", "Δ return", "fine new/ref", "cicli new/ref"],
            [[
                r["job_id"], r.get("independent_verification", "-"), r.get("receipt_returncode"), r.get("receipt_duration_s"), r["verdict"], r.get("analysis_complete"),
                "+".join(r.get("identical_subset") or []) or "-", "+".join(r.get("divergent_subset") or []) or "-", "+".join(r.get("missing_reference_evidence") or []) or "-",
                f"{(r.get('new_summary') or {}).get('steps')}/{sc(r, 'steps', 'reference')}",
                f((r.get("new_summary") or {}).get("episode_return")), f(sc(r, "episode_return", "reference")), f(sc(r, "episode_return", "delta")),
                f"{(r.get('new_summary') or {}).get('end_reason')}/{sc(r, 'end_reason', 'reference')}",
                f"{(r.get('new_summary') or {}).get('phase_valid_cycle_count')}/{sc(r, 'phase_valid_cycle_count', 'reference')}",
            ] for r in results],
        ),
        "",
        "## Prime divergenze delle serie (tempo [s] / RMS / max) e localizzazione da trace",
        "",
        C.md_table(
            ["job", "knee q", "ankle q", "SEA control u knee (normalizzato, non q servito)", "SEA knee tau_spring", "GRF left normal", "grid/len ok", "trace: 1° step div. oss. / az.", "feature divergenti", "interpretazione"],
            [[
                r["job_id"],
                *[
                    (f"{f(ser(r, fn, c, 'first_divergence_time_s'))} / {f(ser(r, fn, c, 'rms_diff'))} / {f(ser(r, fn, c, 'max_abs_diff'))}" if ser(r, fn, c, "available") else "n/a")
                    for fn, c in (("rollout_episode_kinematics.sto", "pros_knee_angle"), ("rollout_episode_kinematics.sto", "pros_ankle_angle"), ("rollout_episode_sea_controls.sto", "pros_knee_angle"), ("rollout_episode_sea_torques.sto", "SEA_Knee_tau_spring"), ("rollout_episode_online_grf.sto", "left_normal_force"))
                ],
                f"{ser(r, 'rollout_episode_kinematics.sto', 'pros_knee_angle', 'time_grid_match_on_overlap')}/{ser(r, 'rollout_episode_kinematics.sto', 'pros_knee_angle', 'same_length')}",
                (f"{((r.get('comparisons') or {}).get('trace') or {}).get('first_observation_divergence_step_index')} / {((r.get('comparisons') or {}).get('trace') or {}).get('first_action_divergence_step_index')}" if ((r.get("comparisons") or {}).get("trace") or {}).get("available") else "n/a"),
                ((r.get("comparisons") or {}).get("trace") or {}).get("divergent_observation_features_at_first_step"),
                ((r.get("comparisons") or {}).get("trace") or {}).get("interpretation", "-"),
            ] for r in results],
        ),
        "",
        f"Verdetti: {verdict_counts}.",
        "",
    ]
    C.write_text(out_md, "\n".join(lines))
    print(f"[replay-analysis] written {out_json} / {out_md}; verdicts={verdict_counts}; technical_ok={payload['technical_execution_ok']} complete={payload['all_analyses_complete']}; preflight verified={len(preflight['verified_identical'])} invalid={len(preflight['invalid'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
