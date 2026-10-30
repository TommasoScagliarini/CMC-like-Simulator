"""Self-test of f0_overlays (temp-only by default; optional explicit --real smoke checks).

Synthetic (fixture of test_f0_matrix_analysis extended with the overlay keys): accepted-cycle
extraction (HS->TO->HS cycle_valid=1 -> 1 cycle / 101 points; cycle_valid=0 or cancelled HS ->
insufficient_cycles), step-wise counters (no interpolation), event table, corridor points only
on settled samples with config/profile provenance, corridor-without-profile fail-closed, missing
SEA state file, CSV/PNG generation, GROUP PHASE RULE (mixed group with one zero-cycle member ->
no phase PNG and status NOT_PHASE_ALIGNABLE_ZERO_VALID_CYCLES with the member listed; all-cycle
group -> phase PNG), no-clobber, atomic staging with artifact manifest written last.
The default path never reads repository rollout outputs. ``--real`` adds a read-only smoke test
against the pinned final analysis and frozen F0 rollouts.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_artifacts as ART  # noqa: E402
import f0_common as C  # noqa: E402
import f0_overlays as O  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402
import test_f0_matrix_analysis as T  # noqa: E402

PASSED = 0


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def expect(fn, exc_types, label: str, needle: str | None = None):
    try:
        fn()
    except exc_types as exc:
        if needle and needle not in str(exc):
            raise AssertionError(f"{label}: error does not mention {needle!r}: {exc}")
        ok(f"{label} -> {type(exc).__name__}")
        return exc
    raise AssertionError(f"{label}: expected {exc_types}")


@contextmanager
def synthetic_output_dir(final: Path):
    """Minimal temp-only harness; the production no-replace publisher is tested in test_f0_actor_drift."""
    if final.exists() or final.is_symlink():
        raise FileExistsError(final)
    final.parent.mkdir(parents=True, exist_ok=True)
    staging = final.parent / f".synthetic_{final.name}"
    staging.mkdir()
    try:
        yield staging
        staging.rename(final)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def overlay_keys(cycle: bool = True, cancel: bool = False, corridor: bool = True):
    def mut(series, trace, summary):
        rows = len(trace) * T.A.SAMPLES_PER_STEP
        t = np.arange(rows) * 0.001
        t0 = float(t[0])
        series[O.SEA_STATES_FILE] = (["time", "SEA_Knee_motor_angle", "SEA_Knee_motor_speed"], np.column_stack([t, 0.1 * np.sin(t), 0.1 * np.cos(t)]))
        for i, r in enumerate(trace):
            r["time"] = t0 + (i + 1) * 0.01
            r["policy_action_mean"] = None
            r["exploration_action_noise"] = None
            r["policy_segment_times"] = [r["time"], r["time"] + 0.01]
            r["policy_segment_values"] = [[0.1, 0.2], [0.11 + 0.001 * i, 0.21]]
            r["reward_terms"] = {"morphology_available": 1.0 if corridor else 0.0, "morphology_settled_this_step": 1.0 if (corridor and i % 3 == 0) else 0.0, "morphology_phase": (i % 100) / 100.0, "fsm_morphology_phase": (i % 100) / 100.0, "morphology_knee_value_rad": 0.3, "morphology_knee_min_rad": 0.1, "morphology_knee_max_rad": 0.5, "morphology_ankle_value_rad": 0.0, "morphology_ankle_min_rad": -0.2, "morphology_ankle_max_rad": 0.2, "morphology_knee_loss": 0.0, "morphology_ankle_loss": 0.0, "morphology_phase_mode_id": 3.0, "morphology_phase_source_id": 1.0}
            r["phase_fsm"].update({"accepted_transitions_this_step": [], "sensor_events_this_step": [], "invalid_event_this_step": 0.0, "invalid_event_type": "", "invalid_event_loss": 0.0, "resync_event_this_step": 0.0, "cycle_rejected_this_step": 0.0, "cycle_reject_reason": "", "pending_cycle_credit": 0.0, "cycle_progress_credit": 0.0, "expected_next_event": "heel_strike"})
        hs1, to1, hs2 = t0 + 0.20, t0 + 0.90, t0 + 1.50
        trace[20]["phase_fsm"]["accepted_transitions_this_step"] = [{"event": "heel_strike", "event_time_s": hs1, "cycle_valid": -1.0, "segment_valid": 1.0, "cycle_reject_reason": ""}]
        if cancel:
            trace[30]["phase_fsm"]["accepted_transitions_this_step"] = [{"event": "heel_strike_cancelled", "event_time_s": hs1, "cycle_valid": -1.0, "cycle_reject_reason": "hs_bounce_cancelled"}]
        else:
            trace[90]["phase_fsm"]["accepted_transitions_this_step"] = [{"event": "toe_off", "event_time_s": to1, "segment_start_time_s": hs1, "cycle_valid": -1.0, "segment_valid": 1.0, "cycle_reject_reason": ""}]
            trace[150]["phase_fsm"]["accepted_transitions_this_step"] = [{"event": "heel_strike", "event_time_s": hs2, "cycle_valid": 1.0 if cycle else 0.0, "segment_valid": 1.0, "cycle_reject_reason": "" if cycle else "stance_too_short"}]
        if cancel or not cycle:
            summary["phase_valid_cycle_count"] = 0
            for r in trace:
                r["phase_fsm"]["valid_cycle_count"] = 0
        trace[100]["phase_fsm"]["invalid_event_this_step"] = 1.0
        trace[100]["phase_fsm"]["invalid_event_type"] = "to_too_early_after_hs"
        trace[120]["phase_fsm"]["resync_event_this_step"] = 1.0
    return mut


def describe(out: Path, job_id: str, family: str, cls: str, cand: str, start: str, mode: str, seed: int, runtime: str = "v3_canonical") -> dict:
    return {"job_id": job_id, "family": family, "comparison_class": cls, "candidate": cand, "runtime": runtime, "start": start, "action_selection": mode, "seed": seed, "repeat": 1, "output_dir": C.rel(out), "config": ART.DIRECT.read_json(out / M.RECEIPT_FILE)["config"], "historical_reference_summary": None}


def analysis_job(out: Path, rec: dict) -> dict:
    receipt = ART.DIRECT.read_json(out / M.RECEIPT_FILE)
    evidence = {}
    for name, path in {
        "summary": out / M.SUMMARY_FILE,
        "trace": out / M.TRACE_FILE,
        **{f: out / "sim_outputs" / f for f in O.A.EXPECTED_STO},
    }.items():
        evidence[name] = {"path": C.rel(path), "present": True, "valid": True, "sha256": C.sha256_file(path)}
    return {**rec, "receipt": receipt, "evidence": evidence, "provenance": {"config": rec["config"], "config_sha256": receipt["config_sha256"]}}


def main(*, real: bool = False) -> int:
    base = T.tmp_base()
    tmp = Path(tempfile.mkdtemp(prefix="f0_overlay_selftest_", dir=base))
    try:
        fx = T.Fixture(tmp)
        steps = 200
        synthetic_actor_names = [f"synthetic_actor_f{i}" for i in range(35)]
        def seal(out_dir: Path) -> Path:
            receipt_path = out_dir / M.RECEIPT_FILE
            receipt = ART.DIRECT.read_json(receipt_path)
            receipt["summary_sha256"] = C.sha256_file(out_dir / M.SUMMARY_FILE)
            receipt["trace_sha256"] = C.sha256_file(out_dir / M.TRACE_FILE)
            receipt["config"] = str((C.REPO / receipt["config"]).resolve())
            receipt_path.unlink()
            ART.write_json_strict(receipt_path, receipt)
            return out_dir

        verify = lambda rec, out_dir: ART.DIRECT.read_json(out_dir / M.RECEIPT_FILE)  # noqa: E731
        load = lambda rec: O.load_job(rec, verify=verify, alpha=0.62, roots=(tmp,))  # noqa: E731
        profile = tmp / "profile.json"
        profile.write_text(json.dumps({"version": 2, "phase_grid": np.linspace(0.0, 1.0, 11).tolist(), "coordinates": {"pros_knee_angle": {"mean_rad": [0.3] * 11, "std_rad": [0.05] * 11}, "pros_ankle_angle": {"mean_rad": [0.0] * 11, "std_rad": [0.05] * 11}}, "metadata": {"phase_parameterization": "event_warped_hs_to_to_to_hs_v1"}}), encoding="utf-8")
        cfg_prof = tmp / "cfg_prof.yaml"
        cfg_prof.write_text(fx.cfg.read_text(encoding="utf-8").replace("reward:\n", f"reward:\n  morphology_profile: {profile}\n  morphology_weight: 0.0025\n"), encoding="utf-8")
        h0 = seal(fx.make_output("H0", steps, candidate="B0820_H0", config=cfg_prof, actor_names=synthetic_actor_names, mutate=overlay_keys(cycle=True)))
        h0b = seal(fx.make_output("H0B", steps, candidate="B0820_V2_BEST", config=cfg_prof, actor_names=synthetic_actor_names, knee_amp=0.35, mutate=overlay_keys(cycle=True)))
        best = seal(fx.make_output("BEST", steps, candidate="B0820_V3_BEST", config=cfg_prof, actor_names=synthetic_actor_names, knee_amp=0.4, mutate=overlay_keys(cycle=False)))
        canc = seal(fx.make_output("CANC", steps, candidate="B0820_V3_LAST", config=cfg_prof, actor_names=synthetic_actor_names, mutate=overlay_keys(cancel=True)))
        d_h0 = load(describe(h0, "B0820_H0__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_H0", "nominal", "deterministic", 123))
        d_h0b = load(describe(h0b, "B0820_V2_BEST__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_V2_BEST", "nominal", "deterministic", 123))
        d_best = load(describe(best, "B0820_V3_BEST__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_V3_BEST", "nominal", "deterministic", 123))
        d_canc = load(describe(canc, "B0820_V3_LAST__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_V3_LAST", "nominal", "deterministic", 123))
        assert d_h0["cycles"]["valid_cycle_count"] == 1 and d_h0["cycles"]["phase_aligned"] == "accepted_fsm_cycles" and abs(d_h0["cycles"]["cycles"][0]["duration_s"] - 1.3) < 1e-9 and len(d_h0["phase"]["grid"]) == O.PHASE_POINTS and d_h0["phase"]["interpolation"] == O.PHASE_INTERPOLATION
        assert d_best["cycles"]["valid_cycle_count"] == 0 and d_best["cycles"]["phase_aligned"] == "insufficient_cycles" and d_best["phase"] is None and d_best["cycles"]["rejected_cycles"][0]["reason"] == "stance_too_short"
        assert d_canc["cycles"]["valid_cycle_count"] == 0 and any(e["kind"] == "fsm_hs_cancelled" for e in d_canc["events"])
        ok("accepted cycles: HS->TO->HS cycle_valid=1 -> 1 cycle (101 points, linear interpolation declared); cycle_valid=0 -> insufficient_cycles with reason; cancelled HS never closes a cycle")

        bound = seal(fx.make_output("BOUND", steps, candidate="B0820_H0", config=cfg_prof, actor_names=synthetic_actor_names, mutate=overlay_keys(cycle=True)))
        bound_rec = describe(bound, "BOUND", "det", "isometric_comparable", "B0820_H0", "nominal", "deterministic", 123)
        bound_analysis = analysis_job(bound, bound_rec)
        assert O.load_job(bound_rec, analysis_job=bound_analysis, verify=None, alpha=0.62, roots=(tmp,))["analysis_binding"]["mode"] == "pinned_analysis_exact_evidence"
        kin = bound / "sim_outputs" / O.A.KINEMATICS_FILE
        original_kin = kin.read_text(encoding="utf-8")
        lines = original_kin.splitlines()
        data_i = lines.index("endheader") + 2
        fields = lines[data_i].split("\t")
        fields[1] = str(float(fields[1]) + 0.001)
        lines[data_i] = "\t".join(fields)
        kin.write_text("\n".join(lines) + "\n", encoding="utf-8")
        expect(lambda: O.load_job(bound_rec, analysis_job=bound_analysis, verify=None, alpha=0.62, roots=(tmp,)), O.OverlayError, "analysis binding refuses structurally valid mutated STO", "pinned-analysis evidence")
        kin.write_text(original_kin, encoding="utf-8")
        ok("pinned-analysis binding: exact receipt projection plus summary/trace/analysed-STO digests; overlay-only SEA state explicitly classified")

        def event_rows(events):
            return [{"step": i + 1, "phase_fsm": {"accepted_transitions_this_step": [event]}} for i, event in enumerate(events)]

        hs0 = {"event": "heel_strike", "event_time_s": 1.0, "cycle_valid": -1, "segment_valid": 1}
        to = {"event": "toe_off", "event_time_s": 1.6, "segment_start_time_s": 1.0, "cycle_valid": -1, "segment_valid": 1}
        hs1 = {"event": "heel_strike", "event_time_s": 2.0, "segment_start_time_s": 1.6, "cycle_valid": 1, "segment_valid": 1}
        direct = O.cycles_from_trace(event_rows([hs0, to, hs1]), summary_cycle_count=1, trace_final_count=1)
        assert direct["valid_cycle_count"] == 1
        expect(lambda: O.cycles_from_trace(event_rows([to, hs1])), O.OverlayError, "cycle refuses TO+HS without opening HS", "required accepted HS -> TO chain")
        bad_to = {**to, "segment_valid": 0}
        expect(lambda: O.cycles_from_trace(event_rows([hs0, bad_to, hs1])), O.OverlayError, "cycle refuses segment-invalid TO", "required accepted HS -> TO chain")
        missing_start = {k: v for k, v in to.items() if k != "segment_start_time_s"}
        expect(lambda: O.cycles_from_trace(event_rows([hs0, missing_start, hs1])), O.OverlayError, "cycle refuses TO without segment_start", "required accepted HS -> TO chain")
        to2 = {**to, "event_time_s": 1.8}
        expect(lambda: O.cycles_from_trace(event_rows([hs0, to, to2, hs1])), O.OverlayError, "cycle refuses repeated TO before closing HS", "required accepted HS -> TO chain")
        degraded = O.cycles_for_overlay(event_rows([hs0, to, {"event": "heel_strike_cancelled", "event_time_s": 1.9}, hs1]), summary_cycle_count=1, trace_final_count=1)
        assert degraded["phase_aligned"] == "invalid_fsm_cycle_contract" and degraded["valid_cycle_count"] == 0 and "required accepted HS -> TO chain" in degraded["phase_contract_error"]
        expect(lambda: O.cycles_from_trace(event_rows([hs0, to, hs1]), summary_cycle_count=0, trace_final_count=1), O.OverlayError, "cycle count must match summary", "summary")
        expect(lambda: O.cycles_from_trace(event_rows([{**hs0, "event_time_s": float("nan")}])) , O.OverlayError, "cycle refuses non-finite event time", "finite")
        expect(lambda: O.cycles_from_trace(event_rows([{**hs0, "segment_valid": 0.5}])), O.OverlayError, "cycle refuses fractional FSM flag", "invalid FSM flag")
        ok("cycle parser fail-closed: opening HS required, TO segment_valid==1 with matching start, finite ordered events and exact summary/final counters; inconsistent runtime cycle disables phase products but preserves time-domain eligibility")
        st = d_h0["steps"]
        assert {"fsm_accepted_heel_strike", "fsm_accepted_toe_off", "fsm_rejected:to_too_early_after_hs", "fsm_resync"} <= {e["kind"] for e in d_h0["events"]}
        assert len(st) == steps and st[0]["fsm_valid_hs_count"] == 1 and st[1]["fsm_valid_hs_count"] == 2 and isinstance(st[1]["fsm_valid_hs_count"], int) and st[5]["causal_total_resolved_sample_count"] == 12 and st[0]["policy_segment_end_knee_rad_diagnostic"] == 0.11 and st[0]["mean_knee"] is None
        cor = d_h0["corridor"]
        assert cor["settled_points"] == math.ceil(steps / 3) and cor["evaluated_steps"] == steps and cor["profile_sha256"] == C.sha256_file(profile) and cor["config_sha256"] == C.sha256_file(cfg_prof) and "evaluated by the reward" in cor["semantics"] and cor["profile_path"]
        assert d_h0["sea_state_columns"] == ["SEA_Knee_motor_angle", "SEA_Knee_motor_speed"] and len(d_h0["khz"]["t_rel_s"]) == steps * T.A.SAMPLES_PER_STEP
        ok("step table step-wise (ints, no interpolation), events table, corridor points only on settled samples with config/profile path+SHA provenance")
        cfg_noprof = tmp / "cfg_noprof.yaml"
        cfg_noprof.write_text(fx.cfg.read_text(encoding="utf-8").replace("reward:\n", "reward:\n  morphology_weight: 0.0\n"), encoding="utf-8")
        bad = seal(fx.make_output("NOPROF", steps, candidate="B0820_H0", config=cfg_noprof, actor_names=synthetic_actor_names, mutate=overlay_keys(corridor=True)))
        expect(lambda: load(describe(bad, "B0820_H0__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_H0", "nominal", "deterministic", 123)), O.OverlayError, "corridor reported without pinned profile", "no morphology_profile")
        noc = seal(fx.make_output("NOCORR", steps, candidate="B0820_H0", config=cfg_noprof, actor_names=synthetic_actor_names, mutate=overlay_keys(corridor=False)))
        assert "NOT evaluated" in load(describe(noc, "B0820_H0__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_H0", "nominal", "deterministic", 123))["corridor"]["semantics"]

        def drop_sea(series, trace, summary):
            overlay_keys()(series, trace, summary)
            series.pop(O.SEA_STATES_FILE)

        nosea = seal(fx.make_output("NOSEA", steps, candidate="B0820_H0", config=cfg_prof, actor_names=synthetic_actor_names, mutate=drop_sea))
        expect(lambda: load(describe(nosea, "B0820_H0__v3_canonical__nominal__det", "det", "isometric_comparable", "B0820_H0", "nominal", "deterministic", 123)), (O.OverlayError, ART.StagingError, OSError), "missing SEA states file")
        ok("corridor consistency fail-closed; runtime without evaluation labelled; missing SEA states file refused")
        # GROUP PHASE RULE + rendering inside an atomic staging dir
        final = tmp / "ov" / "stamp1"
        with synthetic_output_dir(final) as staging:
            files = O.write_job_csvs(staging, d_h0)
            assert files["phase"]["rows"] == O.PHASE_POINTS and files["steps"]["rows"] == steps and files["khz"]["rows"] == steps * T.A.SAMPLES_PER_STEP and files["phase"]["interpolation"] == O.PHASE_INTERPOLATION
            assert O.write_job_csvs(staging, d_best)["phase"] == {"path": None, "status": "insufficient_cycles"}
            degraded_data = {**d_best, "job": {**d_best["job"], "job_id": "DEGRADED"}, "cycles": degraded, "phase": None}
            degraded_csv = O.write_job_csvs(staging, degraded_data)["phase"]
            assert degraded_csv == {"path": None, "status": "invalid_fsm_cycle_contract", "phase_contract_error": degraded["phase_contract_error"]}
            expect(lambda: O.write_job_csvs(staging, d_h0), FileExistsError, "CSV no-clobber")
            mixed = O.render_group(staging, "G_mixed", [(d_h0, "reference"), (d_best, "member"), (d_canc, "member")], "mixed group")
            assert mixed["group_phase_status"] == O.PHASE_NONE == "NOT_PHASE_ALIGNABLE_ZERO_VALID_CYCLES" and mixed["zero_cycle_members"] == ["B0820_V3_BEST__v3_canonical__nominal__det", "B0820_V3_LAST__v3_canonical__nominal__det"] and [o["kind"] for o in mixed["outputs"]] == ["time_aligned"] and not (staging / "png" / "G_mixed__phase.png").exists()
            full = O.render_group(staging, "G_full", [(d_h0, "reference"), (d_h0b, "member")], "all-cycle group")
            assert full["group_phase_status"] == O.PHASE_ALL and full["zero_cycle_members"] == [] and [o["kind"] for o in full["outputs"]] == ["time_aligned", "phase_aligned"] and (staging / "png" / "G_full__phase.png").stat().st_size > 1000
            g2 = O.render_group(staging, "G2_declared_time_only", [(d_h0, "reference"), (d_h0b, "control")], "G2", phase_allowed=False)
            assert g2["group_phase_status"] == O.PHASE_DECLARED_TIME_ONLY and [o["kind"] for o in g2["outputs"]] == ["time_aligned"] and not (staging / "png" / "G2_declared_time_only__phase.png").exists()
            expect(lambda: O.render_group(staging, "G_full", [(d_h0, "reference")], "again"), FileExistsError, "PNG no-clobber")
            band = O.stochastic_band([d_h0, d_best], "#000", "band")
            assert band["common_samples"] == steps * T.A.SAMPLES_PER_STEP and np.all(band["knee_measured_rad_min"] <= band["knee_measured_rad_max"]) and "common span" in band["definition"]
            ART.assert_no_staging_paths({"files": files, "mixed": mixed, "full": full, "g2": g2})
            assert files["khz"]["path"] == "series/B0820_H0__v3_canonical__nominal__det__1khz.csv" and mixed["outputs"][0]["path"] == "png/G_mixed.png"
            ART.write_artifact_manifest(staging, "stamp1", provenance={"kind": "test", "source_analysis": {"path": "synthetic.json", "sha256": "0" * 64}, "script_sha256": C.sha256_file(Path(__file__)), "git_head": "synthetic", "inputs_unchanged": True, "interpreter": {}, "consumed_inputs": 0})
        art = ART.verify_artifact_dir(final)
        assert art["ok"] and art["files"] >= 7 and final.is_dir() and not any(p.name.startswith(".staging") for p in (tmp / "ov").iterdir())
        expect(lambda: synthetic_output_dir(final).__enter__(), FileExistsError, "overlay dir no-clobber")
        ok("group phase rule: mixed/zero-cycle -> no phase PNG; all-cycle -> phase PNG; G2 stays time-only by declaration; degraded FSM contract status/error propagated to phase CSV record; CSV/PNG no-clobber; artefact-relative paths only; atomic staging sealed")
        expect(lambda: ART.loads_strict('{"a":1,"a":2}'), ART.SerializationError, "strict JSON refuses duplicate keys", "duplicate")
        expect(lambda: O._write_csv(tmp / "nonfinite.csv", ["x"], [[float("inf")]]), O.OverlayError, "CSV refuses non-finite numbers", "non-finite")
        ok("strict JSON duplicate-key and CSV non-finite gates")
        if real:
            analysis, ref = O.load_final_analysis()
            assert ref["sha256"] == O.FINAL_ANALYSIS_SHA256
            groups = O.group_specs(M.canonical_index())
            assert len(groups) == 3 * 2 + 6 + 2 + 9
            rows_h0 = ART.DIRECT.read_json(C.OUT_ROLLOUTS / "det" / "B0820_H0__v3_canonical__nominal__det" / M.TRACE_FILE)
            assert O.cycles_from_trace(rows_h0)["phase_aligned"] == "insufficient_cycles"
            cj = O.cycles_from_trace(ART.DIRECT.read_json(C.OUT_ROLLOUTS / "det" / "JUL_H0__v3_canonical__minus020__det" / M.TRACE_FILE))
            assert cj["valid_cycle_count"] >= 1 and all(c["duration_s"] > 0.3 for c in cj["cycles"])
            ok(f"explicit --real smoke: final analysis pinned; {len(groups)} groups; B0820_H0 zero-cycle; JUL_H0 has {cj['valid_cycle_count']} accepted cycles")
        assert str(tmp).startswith(base) and not str(tmp).startswith(str(C.REPO))
        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", action="store_true", help="add read-only smoke checks against frozen repository outputs")
    raise SystemExit(main(real=parser.parse_args().real))
