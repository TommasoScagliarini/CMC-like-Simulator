"""Static self-test of f0_replay_analysis (no simulation, no rollouts).

Synthetic outputs are written in a ``tempfile`` directory created per run
and removed at the end; no real F0 artefact or fixed directory is touched. Covered MEDIUM findings:
  (1) compare_series: same-length requirement, time-grid check, NaN/Inf
      detection, overlap-identical vs full-identical separation;
  (2) full identity only with the complete explicit evidence set (summary,
      five .sto series, trace); a single SEA column difference or a missing
      reference trace can never yield PASS_FULL_IDENTICAL_TO_REFERENCE;
      no vacuous all();
  (3) compare_traces: width mismatch / missing vectors -> environment/schema
      divergence, never identity, never policy-side;
  (4) missing new .sto, missing column, inconsistent rows, non-finite
      values -> FAIL_OUTPUT_INCOMPLETE; missing/invalid reference evidence ->
      INCOMPLETE_REFERENCE_EVIDENCE with SHA/presence/columns recorded.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402
import f0_replay_analysis as A  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

PASSED = 0


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def write_sto(path: Path, names: list[str], data: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["synthetic", f"nRows={data.shape[0]}", f"nColumns={data.shape[1]}", "endheader", "\t".join(names)]
    lines += ["\t".join(f"{v:.12g}" for v in row) for row in data]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_output(root: Path, steps: int, *, seed: int = 0, obs_width: int = 35, time_shift: float = 0.0, mutate=None) -> Path:
    """Build a complete synthetic output directory (summary, five .sto, trace)."""
    rng = np.random.default_rng(seed)
    rows = steps * A.SAMPLES_PER_STEP
    t = 13.0 + np.arange(rows) * 0.001 + time_shift
    series = {}
    for fname, cols in A.EXPECTED_STO.items():
        data = np.column_stack([t] + [np.sin(0.01 * np.arange(rows) * (k + 1)) + 0.1 * k for k in range(len(cols))])
        series[fname] = (["time", *cols], data)
    trace = []
    for i in range(steps):
        obs = (rng.standard_normal(obs_width) * 0 + np.arange(obs_width) * 0.01 + i * 0.001).tolist()
        trace.append({"step": i + 1, "time": float(13.0 + i * 0.01), "raw_policy_action": [0.5 + 0.001 * i, -0.2], "actor_observation_vector_before": obs, "actor_observation_before": {f"f{j}": obs[j] for j in range(obs_width)}})
    summary = {"ok": True, "steps": steps, "episode_return": 1.5, "end_reason": "episode_time_limit", "phase_valid_hs_count": 1, "phase_valid_to_count": 1, "phase_valid_cycle_count": 0, "invalid_event_count": 0, "grf_penetration_max_m": 0.01, "reserve_norm_max_nm": 100.0, "action_abs_max": 0.6, "action_clipped_steps": 0, "n_actor": obs_width}
    if mutate:
        mutate(series, trace, summary)
    root.mkdir(parents=True, exist_ok=True)
    for fname, (names, data) in series.items():
        write_sto(root / "sim_outputs" / fname, names, data)
    (root / M.TRACE_FILE).write_text(json.dumps(trace), encoding="utf-8")
    (root / M.SUMMARY_FILE).write_text(json.dumps(summary), encoding="utf-8")
    return root


def analyse(new_dir: Path, ref_dir: Path | None) -> dict:
    new_summary = json.loads((new_dir / M.SUMMARY_FILE).read_text(encoding="utf-8"))
    ref_summary = json.loads((ref_dir / M.SUMMARY_FILE).read_text(encoding="utf-8")) if ref_dir is not None and (ref_dir / M.SUMMARY_FILE).is_file() else None
    return A.analyse_outputs(new_dir, ref_dir, new_summary, ref_summary, (ref_dir / M.SUMMARY_FILE) if ref_dir is not None else None)


def main() -> int:
    base = "/private/tmp" if (Path("/private/tmp").is_dir() and os.access("/private/tmp", os.W_OK)) else tempfile.gettempdir()
    tmp = Path(tempfile.mkdtemp(prefix="f0_replay_selftest_", dir=base))
    try:
        ref = make_output(tmp / "ref", 3, seed=1)

        # ---------- (1) compare_series -------------------------------------------
        t = np.arange(30) * 0.001
        x = np.sin(t * 40)
        r = A.compare_series(x, x.copy(), t, t.copy())
        assert r["full_identical"] and r["overlap_identical"] and r["same_length"] and r["time_grid_match_on_overlap"] and r["finite"]
        ok("(1) identical same-length series on the same grid -> full_identical")
        r = A.compare_series(np.concatenate([x, x[:5]]), x, np.arange(35) * 0.001, t)
        assert r["overlap_identical"] and not r["full_identical"] and not r["same_length"] and r["rows_compared"] == 30
        ok("(1) longer series with identical overlap -> overlap_identical but NOT full_identical")
        r = A.compare_series(x, x.copy(), t + 0.0005, t)
        assert not r["time_grid_match_on_overlap"] and not r["overlap_identical"] and not r["full_identical"]
        ok("(1) time-grid mismatch -> no identity even with equal values")
        y = x.copy()
        y[7] = math.nan
        r = A.compare_series(y, x, t, t)
        assert r["nonfinite_new"] == 1 and not r["finite"] and not r["overlap_identical"] and not r["full_identical"] and r["rms_diff"] is None
        y[7] = math.inf
        r = A.compare_series(x, y, t, t)
        assert r["nonfinite_ref"] == 1 and not r["finite"] and not r["full_identical"]
        ok("(1) NaN/Inf on either side detected and identity denied")
        r = A.compare_series(x + 1e-6, x, t, t)
        assert not r["overlap_identical"] and r["first_divergence_index"] == 0 and abs(r["max_abs_diff"] - 1e-6) < 1e-12
        ok("(1) numeric difference above tolerance -> first divergence index 0")

        # ---------- (2) full identity only with complete evidence -----------------
        new = make_output(tmp / "new_identical", 3, seed=1)
        res = analyse(new, ref)
        assert res["verdict"] == "PASS_FULL_IDENTICAL_TO_REFERENCE" and res["analysis_complete"] is True
        assert set(res["identical_subset"]) == set(A.EXPECTED_EVIDENCE) and not res["divergent_subset"] and not res["missing_reference_evidence"]
        assert len(A.EXPECTED_STO) == 5 and "trace" in A.EXPECTED_EVIDENCE and "summary" in A.EXPECTED_EVIDENCE
        ok("(2) complete identical evidence (summary + 5 series + trace) -> PASS_FULL_IDENTICAL_TO_REFERENCE")

        def bump_sea(series, trace, summary):
            names, data = series["rollout_episode_sea_torques.sto"]
            data[:, names.index("SEA_Ankle_tau_motor")] += 1e-3

        new = make_output(tmp / "new_sea_diff", 3, seed=1, mutate=bump_sea)
        res = analyse(new, ref)
        assert res["verdict"] == "PASS_EXECUTED_DIVERGENT_FROM_REFERENCE" and res["analysis_complete"] is True
        assert "rollout_episode_sea_torques.sto" in res["divergent_subset"] and "rollout_episode_kinematics.sto" in res["identical_subset"] and "summary" in res["identical_subset"]
        ok("(2) identical summary+kinematics but one SEA column differs -> DIVERGENT, not identical")

        ref_no_trace = make_output(tmp / "ref_no_trace", 3, seed=1)
        (ref_no_trace / M.TRACE_FILE).unlink()
        new = make_output(tmp / "new_for_no_trace", 3, seed=1)
        res = analyse(new, ref_no_trace)
        assert res["verdict"].startswith("INCOMPLETE_REFERENCE_EVIDENCE_IDENTICAL_ON_SUBSET") and res["analysis_complete"] is False
        assert res["missing_reference_evidence"] == ["trace"] and "trace" not in res["identical_subset"] and res["evidence_ref"]["trace"]["present"] is False
        ok("(2) reference trace missing -> INCOMPLETE (identical on named subset), never PASS_FULL_IDENTICAL")

        def bump_kin(series, trace, summary):
            names, data = series["rollout_episode_kinematics.sto"]
            data[:, names.index("pros_knee_angle")] += 1e-4

        new = make_output(tmp / "new_for_no_trace_div", 3, seed=1, mutate=bump_kin)
        res = analyse(new, ref_no_trace)
        assert res["verdict"].startswith("INCOMPLETE_REFERENCE_EVIDENCE_DIVERGENT_ON_SUBSET") and res["analysis_complete"] is False
        ok("(2) reference trace missing + kinematics differ -> INCOMPLETE ... DIVERGENT_ON_SUBSET")

        # ---------- (3) compare_traces classification ------------------------------
        tr_ref = json.loads((ref / M.TRACE_FILE).read_text(encoding="utf-8"))
        tr_new = json.loads((ref / M.TRACE_FILE).read_text(encoding="utf-8"))
        r = A.compare_traces(tr_new, tr_ref)
        assert r["full_identical"] and not r["schema_divergence"]
        ok("(3) identical traces -> full_identical")
        wide = make_output(tmp / "new_wide", 3, seed=1, obs_width=39)
        tr_wide = json.loads((wide / M.TRACE_FILE).read_text(encoding="utf-8"))
        r = A.compare_traces(tr_wide, tr_ref)
        assert r["schema_divergence"] and not r["full_identical"] and not r["overlap_identical"] and "schema" in r["interpretation"] and "policy-side" not in r["interpretation"].split("never")[0]
        assert r["observation_widths_new"] == [39] and r["observation_widths_ref"] == [35]
        ok("(3) observation width mismatch -> environment/schema divergence, not identity, not policy-side")
        tr_missing = json.loads(json.dumps(tr_ref))
        tr_missing[1].pop("actor_observation_vector_before")
        r = A.compare_traces(tr_missing, tr_ref)
        assert r["schema_divergence"] and r["missing_vector_steps"] == [1] and not r["full_identical"]
        ok("(3) missing observation vector on one step -> schema divergence")
        tr_act = json.loads(json.dumps(tr_ref))
        tr_act[1]["raw_policy_action"][0] += 0.01
        r = A.compare_traces(tr_act, tr_ref)
        assert not r["schema_divergence"] and r["first_action_divergence_step_index"] == 1 and r["first_observation_divergence_step_index"] is None and "policy-side" in r["interpretation"]
        ok("(3) same schema, identical observations, action differs -> policy-side only in this legitimate case")
        tr_obs = json.loads(json.dumps(tr_ref))
        tr_obs[1]["actor_observation_vector_before"][3] += 0.5
        tr_obs[2]["raw_policy_action"][1] += 0.02
        r = A.compare_traces(tr_obs, tr_ref)
        assert r["first_observation_divergence_step_index"] == 1 and r["first_action_divergence_step_index"] == 2 and "environment/runtime" in r["interpretation"] and r["divergent_observation_features_at_first_step"] == ["f3"]
        ok("(3) observation diverges before action -> environment/runtime divergence with feature named")
        res = analyse(wide, ref)
        assert res["verdict"] == "PASS_EXECUTED_DIVERGENT_FROM_REFERENCE" and res["comparisons"]["trace"]["schema_divergence"] and "trace" in res["divergent_subset"]
        ok("(3) job-level: width mismatch recorded as divergent trace (schema), never identical")

        # ---------- (4) missing/invalid evidence -----------------------------------
        new = make_output(tmp / "new_missing_sto", 3, seed=1)
        (new / "sim_outputs" / "rollout_episode_sea_torques.sto").unlink()
        res = analyse(new, ref)
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["analysis_complete"] is False
        assert res["evidence_new"]["rollout_episode_sea_torques.sto"]["present"] is False and "rollout_episode_sea_torques.sto" in res["evidence_new"]["_missing_or_invalid"]
        ok("(4) new .sto missing -> FAIL_OUTPUT_INCOMPLETE, presence recorded")

        def drop_col(series, trace, summary):
            names, data = series["rollout_episode_online_grf.sto"]
            j = names.index("left_penetration")
            names.pop(j)
            series["rollout_episode_online_grf.sto"] = (names, np.delete(data, j, axis=1))

        new = make_output(tmp / "new_missing_col", 3, seed=1, mutate=drop_col)
        res = analyse(new, ref)
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence_new"]["rollout_episode_online_grf.sto"]["columns_missing"] == ["left_penetration"]
        ok("(4) new .sto missing an expected column -> FAIL_OUTPUT_INCOMPLETE, column recorded")

        def bad_rows(series, trace, summary):
            names, data = series["rollout_episode_kinematics.sto"]
            series["rollout_episode_kinematics.sto"] = (names, data[:-5])

        new = make_output(tmp / "new_bad_rows", 3, seed=1, mutate=bad_rows)
        res = analyse(new, ref)
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence_new"]["rollout_episode_kinematics.sto"]["rows_consistent"] is False
        ok("(4) rows inconsistent with summary steps -> FAIL_OUTPUT_INCOMPLETE")

        def nan_rows(series, trace, summary):
            names, data = series["rollout_episode_reserve_torques.sto"]
            data[4, names.index("pros_knee_angle_reserve_torque")] = math.nan

        new = make_output(tmp / "new_nan", 3, seed=1, mutate=nan_rows)
        res = analyse(new, ref)
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence_new"]["rollout_episode_reserve_torques.sto"]["finite"] is False
        ok("(4) non-finite value in a new series -> FAIL_OUTPUT_INCOMPLETE")

        new = make_output(tmp / "new_no_trace", 3, seed=1)
        (new / M.TRACE_FILE).unlink()
        res = analyse(new, ref)
        assert res["verdict"] == "FAIL_OUTPUT_INCOMPLETE" and res["evidence_new"]["trace"]["present"] is False
        ok("(4) new trace missing -> FAIL_OUTPUT_INCOMPLETE")

        ref_missing = make_output(tmp / "ref_missing_sto", 3, seed=1)
        (ref_missing / "sim_outputs" / "rollout_episode_reserve_torques.sto").unlink()
        new = make_output(tmp / "new_vs_ref_missing", 3, seed=1)
        res = analyse(new, ref_missing)
        assert res["verdict"].startswith("INCOMPLETE_REFERENCE_EVIDENCE") and res["analysis_complete"] is False
        assert res["missing_reference_evidence"] == ["rollout_episode_reserve_torques.sto"]
        assert res["evidence_ref"]["rollout_episode_reserve_torques.sto"]["present"] is False and res["evidence_ref"]["rollout_episode_kinematics.sto"]["sha256"] and res["evidence_ref"]["rollout_episode_kinematics.sto"]["columns_present"] == ["pros_knee_angle", "pros_ankle_angle"]
        ok("(4) reference .sto missing -> INCOMPLETE_REFERENCE_EVIDENCE; SHA/presence/columns recorded for the rest")

        def ref_nan(series, trace, summary):
            names, data = series["rollout_episode_online_grf.sto"]
            data[2, names.index("left_normal_force")] = math.inf

        ref_bad = make_output(tmp / "ref_nonfinite", 3, seed=1, mutate=ref_nan)
        new = make_output(tmp / "new_vs_ref_bad", 3, seed=1)
        res = analyse(new, ref_bad)
        assert res["verdict"].startswith("INCOMPLETE_REFERENCE_EVIDENCE") and res["evidence_ref"]["rollout_episode_online_grf.sto"]["finite"] is False
        ok("(4) non-finite value in reference series -> INCOMPLETE_REFERENCE_EVIDENCE")

        new = make_output(tmp / "new_no_ref", 3, seed=1)
        res = analyse(new, None)
        assert res["verdict"] == "PASS_EXECUTED_NO_REFERENCE" and res["analysis_complete"] is True
        ok("job without reference and complete evidence -> PASS_EXECUTED_NO_REFERENCE")

        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
