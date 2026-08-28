"""F0 replay analysis: executed ``replay`` jobs versus their recorded references.

For every job of the ``replay`` family (historical configuration replayed on
the current code) this script:
  1. re-verifies the output directory fail-closed through the driver's own
     ``preflight_all`` (receipt fields, status ok, returncode 0, summary.ok,
     trace present, SHA-256 of summary and trace recomputed from disk);
  2. compares the new ``rollout_summary.json`` with the historical reference
     summary field by field (return, steps, end reason, events, cycles,
     penetration, reserve, action bounds, clipping, exploration RMS);
  3. compares the prosthetic kinematics, SEA torques and left normal GRF
     recorded in ``sim_outputs/*.sto`` with the reference ones (RMS, max
     absolute difference, first divergence time);
  4. when both policy traces exist, compares raw actions and actor
     observation vectors step by step (first divergence step) to localise
     whether the divergence enters through the observations (detector /
     FSM / reward runtime) or through the policy itself.

Writes ``metrics/replay_analysis_<stamp>.json/.md`` (no-clobber). Static
except for the interpreter probe needed to rebuild the job descriptions with
the same normalised command as the receipts.
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
SUMMARY_FIELDS = (
    "steps", "episode_return", "end_reason", "phase_valid_hs_count", "phase_valid_to_count",
    "phase_valid_cycle_count", "invalid_event_count", "grf_penetration_max_m", "reserve_norm_max_nm",
    "action_abs_max", "action_clipped_steps", "exploration_noise_realized_rms", "exploration_std_mean",
    "morphology_settled_segments", "morphology_discarded_samples", "n_actor",
)


def read_sto(path: Path) -> tuple[list[str], np.ndarray]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    idx = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    names = lines[idx + 1].split()
    rows = [[float(x) for x in line.split()] for line in lines[idx + 2:] if line.strip()]
    return names, np.asarray(rows, dtype=np.float64)


def column(names: list[str], data: np.ndarray, name: str) -> np.ndarray | None:
    if name not in names:
        return None
    return data[:, names.index(name)]


def compare_series(new: np.ndarray | None, ref: np.ndarray | None, time_new: np.ndarray | None) -> dict[str, Any]:
    if new is None or ref is None:
        return {"available": False}
    n = int(min(len(new), len(ref)))
    if n == 0:
        return {"available": False}
    diff = np.abs(new[:n] - ref[:n])
    first = int(np.argmax(diff > TOL)) if bool(np.any(diff > TOL)) else None
    return {
        "available": True,
        "rows_compared": n,
        "rows_new": int(len(new)),
        "rows_ref": int(len(ref)),
        "rms_diff": float(np.sqrt(np.mean(np.square(new[:n] - ref[:n])))),
        "max_abs_diff": float(np.max(diff)),
        "identical_within_tol": first is None,
        "first_divergence_index": first,
        "first_divergence_time_s": (float(time_new[first]) if (first is not None and time_new is not None and first < len(time_new)) else None),
    }


def compare_summaries(new: dict[str, Any], ref: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    all_equal = True
    for field in SUMMARY_FIELDS:
        a, b = new.get(field), ref.get(field)
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
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
    out["_all_equal"] = all_equal
    return out


def compare_traces(new_path: Path, ref_path: Path) -> dict[str, Any]:
    new = json.loads(new_path.read_text(encoding="utf-8"))
    ref = json.loads(ref_path.read_text(encoding="utf-8"))
    n = min(len(new), len(ref))
    if n == 0:
        return {"available": False}
    act_new = np.asarray([r["raw_policy_action"] for r in new[:n]], dtype=np.float64)
    act_ref = np.asarray([r["raw_policy_action"] for r in ref[:n]], dtype=np.float64)
    obs_new = [np.asarray(r.get("actor_observation_vector_before") or [], dtype=np.float64) for r in new[:n]]
    obs_ref = [np.asarray(r.get("actor_observation_vector_before") or [], dtype=np.float64) for r in ref[:n]]
    act_diff = np.max(np.abs(act_new - act_ref), axis=1)
    first_act = int(np.argmax(act_diff > TOL)) if bool(np.any(act_diff > TOL)) else None
    first_obs = None
    obs_width_match = all(len(a) == len(b) for a, b in zip(obs_new, obs_ref))
    if obs_width_match:
        for i, (a, b) in enumerate(zip(obs_new, obs_ref)):
            if a.size and b.size and bool(np.any(np.abs(a - b) > TOL)):
                first_obs = i
                break
    names_new = new[0].get("actor_observation_before") or {}
    divergent_obs_features = None
    if first_obs is not None and isinstance(names_new, dict):
        keys = list(names_new.keys())
        a, b = obs_new[first_obs], obs_ref[first_obs]
        divergent_obs_features = [keys[j] for j in range(min(len(keys), a.size, b.size)) if abs(a[j] - b[j]) > TOL][:12]
    return {
        "available": True,
        "steps_compared": n,
        "steps_new": len(new),
        "steps_ref": len(ref),
        "action_rms_diff": float(np.sqrt(np.mean(np.square(act_new - act_ref)))),
        "action_max_abs_diff": float(np.max(act_diff)),
        "first_action_divergence_step_index": first_act,
        "first_observation_divergence_step_index": first_obs,
        "observation_width_match": obs_width_match,
        "divergent_observation_features_at_first_step": divergent_obs_features,
        "interpretation": (
            "identical actions and observations over the compared steps"
            if first_act is None and first_obs is None
            else (
                "observations diverge at or before the first action divergence: the difference enters through the environment/runtime (detector, FSM, served reference, reward-side observations), not through the policy weights"
                if first_obs is not None and (first_act is None or first_obs <= first_act)
                else "actions diverge before any observation difference: unexpected policy-side divergence (inference path) - investigate"
            )
        ),
    }


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
        result["verdict"] = "FAIL_NO_RECEIPT"
        return result
    try:
        M.verify_existing(rec, out_dir)
        result["independent_verification"] = "PASS"
    except RuntimeError as exc:
        result["independent_verification"] = f"FAIL: {str(exc)[:600]}"
        result["verdict"] = "FAIL_VERIFICATION"
        return result
    new_summary = C.read_json(out_dir / M.SUMMARY_FILE)
    result["new_summary"] = {k: new_summary.get(k) for k in SUMMARY_FIELDS}
    ref_rel = rec.get("historical_reference_summary")
    if not ref_rel:
        result["verdict"] = "PASS_EXECUTED_NO_REFERENCE"
        return result
    ref_path = C.REPO / ref_rel
    ref_summary = C.read_json(ref_path)
    result["reference_recorded_at"] = C.iso_mtime(ref_path)
    result["reference_checkpoint_field"] = ref_summary.get("checkpoint")
    result["summary_comparison"] = compare_summaries(new_summary, ref_summary)
    ref_dir = ref_path.parent
    kin_new = out_dir / "sim_outputs" / "rollout_episode_kinematics.sto"
    kin_ref = ref_dir / "sim_outputs" / "rollout_episode_kinematics.sto"
    series: dict[str, Any] = {}
    if kin_new.is_file() and kin_ref.is_file():
        n_names, n_data = read_sto(kin_new)
        r_names, r_data = read_sto(kin_ref)
        t_new = column(n_names, n_data, "time")
        for joint in ("pros_knee_angle", "pros_ankle_angle"):
            series[joint] = compare_series(column(n_names, n_data, joint), column(r_names, r_data, joint), t_new)
    for fname, cols in (("rollout_episode_sea_torques.sto", ("SEA_Knee_tau_spring", "SEA_Ankle_tau_spring")), ("rollout_episode_online_grf.sto", ("left_normal_force", "left_penetration")), ("rollout_episode_sea_controls.sto", ("pros_knee_angle", "pros_ankle_angle"))):
        p_new, p_ref = out_dir / "sim_outputs" / fname, ref_dir / "sim_outputs" / fname
        if p_new.is_file() and p_ref.is_file():
            n_names, n_data = read_sto(p_new)
            r_names, r_data = read_sto(p_ref)
            t_new = column(n_names, n_data, "time")
            for col in cols:
                series[f"{fname}:{col}"] = compare_series(column(n_names, n_data, col), column(r_names, r_data, col), t_new)
    result["series_comparison"] = series
    tr_new, tr_ref = out_dir / M.TRACE_FILE, ref_dir / M.TRACE_FILE
    result["trace_comparison"] = compare_traces(tr_new, tr_ref) if (tr_new.is_file() and tr_ref.is_file()) else {"available": False, "reason": "reference trace missing" if not tr_ref.is_file() else "new trace missing"}
    kin_identical = all(s.get("identical_within_tol") for k, s in series.items() if s.get("available") and k in ("pros_knee_angle", "pros_ankle_angle"))
    if result["summary_comparison"]["_all_equal"] and kin_identical and series:
        result["verdict"] = "PASS_IDENTICAL_TO_REFERENCE"
    else:
        result["verdict"] = "PASS_EXECUTED_DIVERGENT_FROM_REFERENCE"
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
        "schema_version": 1,
        "revision": C.F0_REV,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "interpreter": {k: interpreter.get(k) for k in ("selected", "source")},
        "execute_manifests": [C.rel(m) for m in manifests],
        "status_files": [C.rel(s) for s in statuses],
        "preflight_independent": preflight,
        "tolerance_abs": TOL,
        "verdict_counts": verdict_counts,
        "all_executed_ok": all(r["verdict"].startswith("PASS") for r in results) and len(results) == 9,
        "note": "replay = historical resolved yaml + historical weights on current HEAD code; a divergence from the recorded reference measures code changes since the reference date and is expected, not a failure; identity is reported when it holds.",
        "jobs": results,
    }
    C.write_json(out_json, payload)

    def f(v: Any, nd: int = 6) -> str:
        if isinstance(v, float):
            return f"{v:.{nd}g}"
        return str(v)

    lines = [
        f"# F0 — Analisi dei job replay rispetto ai riferimenti registrati — revisione {C.F0_REV} — stamp {stamp}",
        "",
        f"Generato: {payload['generated_at_utc']} — git HEAD `{payload['git']['head'][:12]}`. Manifest di esecuzione: {', '.join(payload['execute_manifests']) or '-'}. Preflight indipendente: {len(preflight['verified_identical'])} output verificati, {len(preflight['invalid'])} invalidi.",
        "",
        payload["note"],
        "",
        "## Esito per job",
        "",
        C.md_table(
            ["job", "verifica", "rc", "durata [s]", "verdetto", "steps new/ref", "return new", "return ref", "Δ return", "fine new/ref", "cicli new/ref", "HS/TO new", "HS/TO ref", "penetr. new/ref [mm]", "reserve new/ref [Nm]", "clip new/ref"],
            [[
                r["job_id"], r.get("independent_verification", "-"), r.get("receipt_returncode"), r.get("receipt_duration_s"), r["verdict"],
                f"{(r.get('new_summary') or {}).get('steps')}/{((r.get('summary_comparison') or {}).get('steps') or {}).get('reference')}",
                f((r.get("new_summary") or {}).get("episode_return")), f(((r.get("summary_comparison") or {}).get("episode_return") or {}).get("reference")), f(((r.get("summary_comparison") or {}).get("episode_return") or {}).get("delta")),
                f"{(r.get('new_summary') or {}).get('end_reason')}/{((r.get('summary_comparison') or {}).get('end_reason') or {}).get('reference')}",
                f"{(r.get('new_summary') or {}).get('phase_valid_cycle_count')}/{((r.get('summary_comparison') or {}).get('phase_valid_cycle_count') or {}).get('reference')}",
                f"{(r.get('new_summary') or {}).get('phase_valid_hs_count')}/{(r.get('new_summary') or {}).get('phase_valid_to_count')}",
                f"{((r.get('summary_comparison') or {}).get('phase_valid_hs_count') or {}).get('reference')}/{((r.get('summary_comparison') or {}).get('phase_valid_to_count') or {}).get('reference')}",
                f"{f(1000 * ((r.get('new_summary') or {}).get('grf_penetration_max_m') or 0), 4)}/{f(1000 * (((r.get('summary_comparison') or {}).get('grf_penetration_max_m') or {}).get('reference') or 0), 4)}",
                f"{f((r.get('new_summary') or {}).get('reserve_norm_max_nm'), 5)}/{f(((r.get('summary_comparison') or {}).get('reserve_norm_max_nm') or {}).get('reference'), 5)}",
                f"{(r.get('new_summary') or {}).get('action_clipped_steps')}/{((r.get('summary_comparison') or {}).get('action_clipped_steps') or {}).get('reference')}",
            ] for r in results],
        ),
        "",
        "## Divergenza delle serie temporali (prime divergenze e RMS) e localizzazione da trace",
        "",
        C.md_table(
            ["job", "knee: 1a div. [s] / RMS [rad] / max", "ankle: 1a div. [s] / RMS [rad] / max", "served knee RMS", "SEA knee tau RMS [Nm]", "GRF left normal RMS [N]", "trace: 1° step div. oss. / az.", "feature divergenti al 1° step", "interpretazione"],
            [[
                r["job_id"],
                *[
                    f"{f(s.get('first_divergence_time_s'))} / {f(s.get('rms_diff'))} / {f(s.get('max_abs_diff'))}" if s.get("available") else "n/a"
                    for s in ((r.get("series_comparison") or {}).get("pros_knee_angle", {}), (r.get("series_comparison") or {}).get("pros_ankle_angle", {}))
                ],
                f(((r.get("series_comparison") or {}).get("rollout_episode_sea_controls.sto:pros_knee_angle") or {}).get("rms_diff")),
                f(((r.get("series_comparison") or {}).get("rollout_episode_sea_torques.sto:SEA_Knee_tau_spring") or {}).get("rms_diff")),
                f(((r.get("series_comparison") or {}).get("rollout_episode_online_grf.sto:left_normal_force") or {}).get("rms_diff")),
                (f"{(r.get('trace_comparison') or {}).get('first_observation_divergence_step_index')} / {(r.get('trace_comparison') or {}).get('first_action_divergence_step_index')}" if (r.get("trace_comparison") or {}).get("available") else "n/a: " + str((r.get("trace_comparison") or {}).get("reason"))),
                (r.get("trace_comparison") or {}).get("divergent_observation_features_at_first_step"),
                (r.get("trace_comparison") or {}).get("interpretation", "-"),
            ] for r in results],
        ),
        "",
        f"Verdetti: {verdict_counts}. Tutti eseguiti con esito ok: **{payload['all_executed_ok']}**.",
        "",
    ]
    C.write_text(out_md, "\n".join(lines))
    print(f"[replay-analysis] written {out_json} / {out_md}; verdicts={verdict_counts}; preflight verified={len(preflight['verified_identical'])} invalid={len(preflight['invalid'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
