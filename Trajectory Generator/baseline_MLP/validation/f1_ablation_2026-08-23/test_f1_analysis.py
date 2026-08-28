"""Self-test of f1_analysis: gates on synthetic compact records + the real
extraction path on one existing (immutable) F0 job directory, compared with
the pinned F0 analysis JSON.  No rollout; F0 outputs are only read."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402
import f1_analysis as AN  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def rec(candidate: str, role: str, start: str, sel: str, seed: int, *, cycles: int, rom: float, horizon: bool = True, reserve: float = 800.0, end: str = "episode_time_limit", repeat: int = 1, trace: str | None = None, steps: int = 500, family: str = "x", cls: str = "isometric_comparable") -> dict:
    return {"job_id": f"{candidate}__v3_canonical__{start}__{'det' if sel == 'deterministic' else 'stoch_seed' + str(seed)}{'' if repeat == 1 else '_rep' + str(repeat)}", "candidate": candidate, "role": role, "family": family, "comparison_class": cls, "start": start, "action_selection": sel, "seed": seed, "repeat": repeat, "source": "f1", "steps": steps, "episode_return": 0.0, "end_reason": end, "horizon_completed": horizon, "valid_cycles": cycles, "valid_hs": cycles + 1, "valid_to": cycles, "invalid_events": 0, "knee_rom": rom, "knee_min": -1.0, "knee_max": -1.0 + rom, "ankle_rom": 0.4, "ankle_min": 0.0, "ankle_max": 0.4, "ankle_negative_fraction": 0.0, "penetration_max_m": 0.015, "reserve_norm_max_nm": reserve, "action_abs_max": 1.0, "action_clipped_steps": 3, "resync_count": 0, "hs_cancelled_count": 0, "summary_sha256": "s" * 64, "trace_sha256": trace or (candidate + start + sel + str(seed) + str(repeat)).ljust(64, "0")}


def synthetic_world(*, d_good: bool = True, aiso_ok: bool = True) -> tuple[list[dict], list[dict], dict]:
    records: list[dict] = []
    a_cycles = {"minus020": 3, "nominal": 2, "plus020": 2} if aiso_ok else {"minus020": 1, "nominal": 0, "plus020": 0}
    a_rom = {"minus020": 1.0, "nominal": 0.95, "plus020": 0.9} if aiso_ok else {"minus020": 0.4, "nominal": 0.3, "plus020": 0.3}
    for s in F1.STARTS:
        records.append(rec("A_ISO39_V3", "A_iso", s, "deterministic", 123, cycles=a_cycles[s], rom=a_rom[s], reserve=900.0, cls="isometric_privileged_39D"))
        records.append(rec("B", "B", s, "deterministic", 123, cycles=0, rom=0.3, trace=("b" + s).ljust(64, "0")))
        records.append(rec("C", "C", s, "deterministic", 123, cycles=0, rom=0.3, trace=("b" + s).ljust(64, "0")))
        d_c = {"minus020": 2, "nominal": 1, "plus020": 1}[s] if d_good else 0
        records.append(rec("D", "D", s, "deterministic", 123, cycles=d_c, rom=0.75 if d_good else 0.32, reserve=850.0))
        for k in F1.DEVELOPMENT_SEEDS:
            records.append(rec("B", "B", s, "stochastic", k, cycles=0, rom=0.3, horizon=(k != 125), steps=500 if k != 125 else 358))
            records.append(rec("C", "C", s, "stochastic", k, cycles=0, rom=0.3))
            records.append(rec("D", "D", s, "stochastic", k, cycles=1 if d_good else 0, rom=0.7 if d_good else 0.3))
            records.append(rec("A_ISO39_V3_S005", "A_iso_sigma005", s, "stochastic", k, cycles=2, rom=0.9, cls="isometric_privileged_39D"))
    records.append(rec("B", "B", "nominal", "stochastic", 123, cycles=0, rom=0.3, repeat=2, trace=("Bnominalstochastic1231").ljust(64, "0")))
    records.append(rec("PASSTHROUGH_B", "integrity_passthrough", "nominal", "deterministic", 123, cycles=0, rom=0.3, trace=("bnominal").ljust(64, "0"), cls="integrity_control"))
    receipts = [{"job_id": r["job_id"], "candidate": r["candidate"], "driver": "f1_rollout_aiso" if r["candidate"].startswith("A_ISO") or r["candidate"] == "PASSTHROUGH_B" else "rollout_eval", "action_selection": r["action_selection"], "seed": r["seed"], "status": "ok", "returncode": 0, "summary_ok": True, "trace_present": True, "source_closure_unchanged": True, "job_inputs_unchanged": True, "closure_manifest_unchanged": True, "gate_relevant": True} for r in records]
    aiso_ids = [r["job_id"] for r in receipts if r["driver"] == "f1_rollout_aiso"]
    extras = {
        "f0_b_det_trace_sha": {s: ("b" + s).ljust(64, "0") for s in F1.STARTS},
        "adapter_checks": {j: {"all_steps_asserted": True, "steps_match_rollout": True, "steps": 500, "projection_assert_count": 500} for j in aiso_ids},
        "sidecar_tpre_checks": {j: {"ok": True} for j in aiso_ids},
        "teacher_checks": {j: {"ok": True, "max_abs_error": 1e-7} for j in aiso_ids if "A_ISO39_V3__" in j and j.endswith("det")},
        "derived_modules": {n: {"ok": True} for n in ("C", "A_ISO39_V3_S005", "D")},
        "crn": {f"{s}_seed{k}": {"ok": True} for s in F1.STARTS for k in F1.DEVELOPMENT_SEEDS},
        "dataset_receipt": {"train_seeds": [123, 124], "val_seeds": [125], "trajectories_train": [0, 1, 3, 4, 6, 7], "trajectories_val": [2, 5, 8], "include_det": False, "rows_train": 3000, "rows_val": 1500},
        "refit_report": {"epochs_run": 300, "selection": "fixed_final_epoch", "dagger_rounds": 0, "new_collection_after_fit": False, "budget": F1.load_protocol()["refit"]["budget"], "init_rmse_vs_teacher": {"train": 0.10, "val": 0.11}, "fit_rmse_vs_teacher": {"train": 0.03, "val": 0.04}},
        "reconstruction_config_equal": True,
        "f0_crosscheck": real_crosscheck_payload(),
        "f0_ctrl39_trace_sha": {s: hashlib.sha256(("ctrl39" + s).encode()).hexdigest() for s in F1.STARTS},
    }
    return records, receipts, extras


def crosscheck_result(start: str) -> dict:
    return {"schema_version": 2, "observation_dtype": "float32", "rows": 500, "cells": 2000, "recorded_representable_in_obs_dtype": True, "exact_rows_runtime_dtype": 500, "exact_cells_runtime_dtype": 2000, "exact_runtime_dtype": True, "max_abs_diff_runtime_dtype": 0.0, "float64_post_step_rows": 500, "exact_cells_float64_post_step": 2000, "exact_float64_post_step": True, "max_abs_diff_float64_post_step": 0.0, "max_abs_diff_float64_vs_recorded_informational": 2.37e-07, "exact": True, "f0_job_dir": f"Trajectory Generator/runs/rollout/validation/f0_freeze_runs/2026-08-22_F0_freeze_inventory_baseline_r3/rollouts/ctrl39/V26_39D__v26_imitation_native__{start}__det", "trace_sha256": hashlib.sha256(("ctrl39" + start).encode()).hexdigest()}


def real_crosscheck_payload() -> dict:
    """Exact top-level form written by f1_adapter_crosscheck.run_crosscheck (schema 2:
    metadata + results keyed by start); the r1 gate iterated .values() of this mapping."""
    return {
        "schema_version": 2,
        "stamp": "20260823_150000",
        "built_under_start": "nominal",
        "env_build": {"steps": 0, "n_actor": 35, "n_observation": 84, "gait_clock_enable": False, "checkpoint": "x", "config": "y", "config_sha256": "0" * 64},
        "observation_dtype": "float32",
        "dtype_rule": "...",
        "projection_contract": {"insert_index": 2, "insert_count": 4},
        "reconstructor": {"builder": "f1_rollout_aiso.build_reconstructor_from_env", "clock_available": True, "clock_n_cycles": 124},
        "results": {s: crosscheck_result(s) for s in F1.STARTS},
        "all_exact": True,
        "tool_sha256": "0" * 64,
        "generated_at_utc": "2026-08-23T00:00:00+00:00",
        "git": {"head": "04c0c470da8192c6fd8496b753fa061cac7dc270", "status_porcelain": []},
    }


def v1_crosscheck_payload() -> dict:
    """The superseded schema-1 form (float64-vs-float32 comparison): must be rejected."""
    return {"schema_version": 1, "built_under_start": "nominal", "reconstructor": {}, "results": {s: {"rows": 500, "max_abs_diff": 0.0, "exact_rows": 500, "exact": True, "f0_job_dir": "x", "trace_sha256": hashlib.sha256(("ctrl39" + s).encode()).hexdigest()} for s in F1.STARTS}, "all_exact": True, "generated_at_utc": "", "git": {}}


def main() -> int:
    protocol = F1.load_protocol()
    records, receipts, extras = synthetic_world()
    g1 = AN.gate_g1(records, receipts, extras, protocol)
    check(g1["pass"], f"G1 passes on a consistent synthetic world: {[k for k, v in g1['checks'].items() if not v['ok']]}")
    # G1 negatives
    bad = copy.deepcopy(extras)
    bad["adapter_checks"][next(iter(bad["adapter_checks"]))]["all_steps_asserted"] = False
    check(not AN.gate_g1(records, receipts, bad, protocol)["checks"]["adapter_assertions"]["ok"], "G1 fails when an A_iso step was not asserted")
    rec2 = copy.deepcopy(records)
    for r in rec2:
        if r["candidate"] == "C" and r["action_selection"] == "deterministic" and r["start"] == "nominal":
            r["trace_sha256"] = "z" * 64
    check(not AN.gate_g1(rec2, receipts, extras, protocol)["checks"]["c_det_equals_b_det"]["ok"], "G1 fails when C-det != B-det")
    rcp = copy.deepcopy(receipts)
    rcp[0]["status"] = "failed"
    check(not AN.gate_g1(records, rcp, extras, protocol)["checks"]["receipts_ok"]["ok"], "G1 fails on a failed receipt")
    rcp2 = copy.deepcopy(receipts)
    rcp2[3]["seed"] = 126
    check(not AN.gate_g1(records, rcp2, extras, protocol)["checks"]["seeds"]["ok"], "G1 fails on a sealed seed")
    bad2 = copy.deepcopy(extras)
    bad2["refit_report"]["epochs_run"] = 299
    check(not AN.gate_g1(records, receipts, bad2, protocol)["checks"]["refit_budget"]["ok"], "G1 fails on a budget deviation")
    bad3 = copy.deepcopy(extras)
    bad3["dataset_receipt"]["trajectories_val"] = [0]
    check(not AN.gate_g1(records, receipts, bad3, protocol)["checks"]["dataset"]["ok"], "G1 fails on split leakage")
    # --- adapter cross-check payload: real full form (metadata + results) must validate; malformed forms fail closed
    xc_ok = g1["checks"]["adapter_vs_f0_ctrl39"]
    check(xc_ok["ok"] is True and xc_ok["problems"] == [] and set(xc_ok["per_start"]) == set(F1.STARTS) and xc_ok["all_exact"] is True and xc_ok["built_under_start"] == "nominal" and xc_ok["observation_dtype"] == "float32" and xc_ok["stamp"] == "20260823_150000", "G1 accepts the real full schema-2 cross-check payload (top-level metadata + results)")
    v_ok = AN.validate_crosscheck_payload(real_crosscheck_payload(), expected_trace_sha=extras["f0_ctrl39_trace_sha"])
    check(v_ok["ok"] and v_ok["per_start"]["plus020"]["exact_rows_runtime_dtype"] == 500 and v_ok["per_start"]["plus020"]["exact_cells_float64_post_step"] == 2000, "validator: real payload ok")
    v1 = AN.validate_crosscheck_payload(v1_crosscheck_payload(), expected_trace_sha=extras["f0_ctrl39_trace_sha"])
    check(v1["ok"] is False and any("schema_version" in pr for pr in v1["problems"]), "validator rejects the superseded schema-1 payload")
    bad4 = copy.deepcopy(extras)
    bad4["f0_crosscheck"]["results"]["nominal"]["exact"] = False
    bad4["f0_crosscheck"]["results"]["nominal"]["exact_runtime_dtype"] = False
    bad4["f0_crosscheck"]["results"]["nominal"]["exact_rows_runtime_dtype"] = 499
    bad4["f0_crosscheck"]["results"]["nominal"]["exact_cells_runtime_dtype"] = 1999
    bad4["f0_crosscheck"]["results"]["nominal"]["max_abs_diff_runtime_dtype"] = 2.4e-07
    r4 = AN.gate_g1(records, receipts, bad4, protocol)["checks"]["adapter_vs_f0_ctrl39"]
    check(not r4["ok"] and any("nominal" in pr for pr in r4["problems"]), "G1 fails when the reconstruction differs from the F0 ctrl39 trace")
    bad_missing = copy.deepcopy(extras)
    del bad_missing["f0_crosscheck"]["results"]["plus020"]
    r5 = AN.gate_g1(records, receipts, bad_missing, protocol)["checks"]["adapter_vs_f0_ctrl39"]
    check(not r5["ok"] and any("plus020" in pr for pr in r5["problems"]) and r5["per_start"]["plus020"] is None, "G1 fails when a required start is missing from results")
    for label, mutate in (
        ("payload None (tool never run)", lambda d: None),
        ("payload is a list", lambda d: [d]),
        ("results is a list", lambda d: {**d, "results": list(d["results"].values())}),
        ("results flattened at top level (old assumption)", lambda d: {**{k: v for k, v in d.items() if k != "results"}, **d["results"]}),
        ("schema_version wrong", lambda d: {**d, "schema_version": 3}),
        ("all_exact False", lambda d: {**d, "all_exact": False}),
        ("all_exact True but a start not exact (inconsistent)", lambda d: {**d, "results": {**d["results"], "minus020": {**d["results"]["minus020"], "exact": False}}}),
        ("result record is not a mapping", lambda d: {**d, "results": {**d["results"], "nominal": "exact"}}),
        ("unknown start in results", lambda d: {**d, "results": {**d["results"], "plus040": d["results"]["nominal"]}}),
        ("rows zero", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "rows": 0, "exact_rows_runtime_dtype": 0, "float64_post_step_rows": 0}}}),
        ("max_abs_diff_runtime_dtype NaN", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "max_abs_diff_runtime_dtype": float("nan")}}}),
        ("max_abs_diff_float64_post_step 1e-12 (no tolerance)", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "max_abs_diff_float64_post_step": 1e-12}}}),
        ("runtime-dtype exact but float64 post-step not exact", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "exact_float64_post_step": False, "exact_cells_float64_post_step": 1999}}}),
        ("float64 post-step exact but runtime dtype not exact", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "exact_runtime_dtype": False, "exact_cells_runtime_dtype": 1999, "exact_rows_runtime_dtype": 499}}}),
        ("recorded not representable in the observation dtype", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "recorded_representable_in_obs_dtype": False}}}),
        ("observation dtype mismatch between payload and a result", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "observation_dtype": "float64"}}}),
        ("observation dtype not floating", lambda d: {**d, "observation_dtype": "int32", "results": {s: {**r, "observation_dtype": "int32"} for s, r in d["results"].items()}}),
        ("env build stepped the simulation", lambda d: {**d, "env_build": {**d["env_build"], "steps": 1}}),
        ("per-start result schema_version 1", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "schema_version": 1}}}),
        ("trace sha not pinned", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "trace_sha256": "f" * 64}}}),
        ("trace sha malformed", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "trace_sha256": "abc"}}}),
        ("reconstructor missing", lambda d: {k: v for k, v in d.items() if k != "reconstructor"}),
        ("built_under_start unknown", lambda d: {**d, "built_under_start": "plus040"}),
        ("bool rows", lambda d: {**d, "results": {**d["results"], "nominal": {**d["results"]["nominal"], "rows": True, "exact_rows_runtime_dtype": True, "float64_post_step_rows": True}}}),
    ):
        payload = mutate(copy.deepcopy(real_crosscheck_payload()))
        v = AN.validate_crosscheck_payload(payload, expected_trace_sha=extras["f0_ctrl39_trace_sha"])
        check(v["ok"] is False and v["problems"], f"validator fails closed: {label}")
        ex = copy.deepcopy(extras)
        ex["f0_crosscheck"] = payload
        gg = AN.gate_g1(records, receipts, ex, protocol)
        check(gg["checks"]["adapter_vs_f0_ctrl39"]["ok"] is False and gg["pass"] is False, f"G1 fails closed without raising: {label}")
    v_nosha = AN.validate_crosscheck_payload(real_crosscheck_payload())
    check(v_nosha["ok"], "validator without pinned SHAs accepts a well-formed payload")
    bad5 = copy.deepcopy(extras)
    bad5["crn"]["nominal_seed123"]["ok"] = False
    check(not AN.gate_g1(records, receipts, bad5, protocol)["checks"]["crn"]["ok"], "G1 fails on CRN mismatch")
    # G2
    g2 = AN.gate_g2(extras["refit_report"], protocol)
    check(g2["pass"] and g2["primary"] and g2["overfit_guard"] and abs(g2["relative_reduction"] - (1 - 0.04 / 0.11)) < 1e-12, "G2 passes (val 0.04 <= 0.5*0.11, val <= 1.5*train)")
    check(not AN.gate_g2({**extras["refit_report"], "fit_rmse_vs_teacher": {"train": 0.03, "val": 0.06}}, protocol)["pass"], "G2 fails when val RMSE > 0.5 x C")
    check(not AN.gate_g2({**extras["refit_report"], "fit_rmse_vs_teacher": {"train": 0.02, "val": 0.05}}, protocol)["pass"], "G2 fails on the overfit guard")
    check(AN.gate_g2(None, protocol)["status"] == "NOT_EVALUABLE", "G2 not evaluable without the refit report")
    # G3
    g3 = AN.gate_g3(records, protocol)
    check(g3["aiso_success_predicate"]["ok"] and g3["status"] == "EVALUATED" and g3["pass"], f"G3 passes on the good synthetic world: {g3['D_vs_C_det']['cycles_ok'], g3['D_vs_C_det']['knee_rom_ok'], g3['D_vs_C_det']['horizon_ok'], g3['D_vs_C_det']['safety_ok'], g3['D_vs_C_stoch']['ok']}")
    check(g3["D_vs_C_det"]["sum_cycles"] == {"A_iso": 7, "C": 0, "D": 4, "required_D": 4}, "G3 cycle arithmetic (ceil(0.5*7)=4)")
    check(g3["D_vs_C_det"]["per_start"]["nominal"]["knee_rom"]["target"] == 0.3 + 0.5 * (0.95 - 0.3), "G3 ROM target = C + 0.5*(A_iso - C)")
    bad_d, r_bad, _ = synthetic_world(d_good=False)
    g3b = AN.gate_g3(bad_d, protocol)
    check(g3b["status"] == "EVALUATED" and not g3b["pass"] and not g3b["D_vs_C_det"]["cycles_ok"], "G3 fails when D does not recover")
    no_a, _, _ = synthetic_world(aiso_ok=False)
    g3c = AN.gate_g3(no_a, protocol)
    check(g3c["status"] == "NOT_EVALUABLE_AISO_FAILED" and g3c["pass"] is False and "F2" in g3c["interpretation"], "G3 not evaluable when A_iso fails (redesign F2, no drop attribution)")
    safety = copy.deepcopy(records)
    for r in safety:
        if r["candidate"] == "D" and r["action_selection"] == "deterministic":
            r["reserve_norm_max_nm"] = 2000.0
    check(not AN.gate_g3(safety, protocol)["D_vs_C_det"]["safety_ok"], "G3 safety guard on reserve")
    stoch_reg = copy.deepcopy(records)
    for r in stoch_reg:
        if r["candidate"] == "D" and r["action_selection"] == "stochastic":
            r["horizon_completed"] = False
    check(not AN.gate_g3(stoch_reg, protocol)["D_vs_C_stoch"]["ok"], "G3 stochastic non-regression")
    # G4 / G5 / decision
    g4 = AN.gate_g4(records)
    check(g4["pairs_found"] == 9 and g4["paired_mean_delta_C_minus_B"]["steps"] > 0, "G4 pairs by start x seed (rep2 excluded)")
    g5 = AN.gate_g5(records + [AN.compact_from_f0_analysis(e, role="A_native", comparison_class="compatibility_control_39D") for e in json.loads(F1.F0_ANALYSIS_JSON.read_text())["jobs"] if e["candidate"] == "V26_39D" and e["executed"]])
    check(sum(1 for r in g5["rows"] if r["role"] == "A_native") == 3 and any(r["source"] == "f0_analysis" for r in g5["rows"]), "G5 lists the F0 A_native controls")
    check(AN.decide(g1, g2, g3)["verdict"] == "F1_PASS" and AN.decide(g1, g2, g3b)["verdict"] == "F1_FAIL" and AN.decide(g1, g2, g3c)["verdict"] == "F1_NOT_EVALUABLE_AISO_FAILED_REDESIGN_F2", "decision rule")
    # F0 records from the pinned analysis
    f0_records, f0_b_det, f0_ctrl39 = AN.load_f0_records()
    check(sum(1 for r in f0_records if r["role"] == "A_native") == 3 and sum(1 for r in f0_records if r["role"] == "E") == 12 and set(f0_b_det) == set(F1.STARTS), "F0 reuse records: 3 A_native, 12 E, 3 B det trace SHAs")
    check(set(f0_ctrl39) == set(F1.STARTS) and all(isinstance(v, str) and len(v) == 64 for v in f0_ctrl39.values()), "pinned F0 ctrl39 trace SHAs for the 3 starts")
    an_rec = {r["start"]: r for r in f0_records if r["role"] == "A_native"}
    check(all(an_rec[s]["trace_sha256"] == f0_ctrl39[s] for s in F1.STARTS), "ctrl39 trace SHAs consistent with the A_native records")
    an = [r for r in f0_records if r["role"] == "A_native" and r["start"] == "nominal"][0]
    check(an["valid_cycles"] == 2 and abs(an["knee_rom"] - 0.997) < 0.001 and an["ankle_negative_fraction"] == 0.0, "A_native nominal record matches the F0 report (2 cycles, ROM 0.997)")
    # real extraction path on one immutable F0 job dir vs the pinned F0 analysis entry
    f0 = json.loads(F1.F0_ANALYSIS_JSON.read_text())
    entry = next(e for e in f0["jobs"] if e["job_id"] == "B0820_H0__v3_canonical__nominal__det")
    job_dir = C.REPO / entry["output_dir"]
    f0_receipt = C.read_json(job_dir / "f0_receipt.json")
    names35, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    pseudo = {"job_id": entry["job_id"], "candidate": "B", "role": "B", "family": "b_det", "comparison_class": "isometric_comparable", "start": "nominal", "action_selection": "deterministic", "seed": 123, "repeat": 1, "runtime": "v3_canonical", "config": f0_receipt["config"], "config_sha256": f0_receipt["config_sha256"], "rollout_eval_sha256": f0_receipt["rollout_eval_sha256"], "command": f0_receipt["command"], "env_manifest": C.rel(C.ACTOR_MANIFEST_35), "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256}
    m = AN.f1_job_metrics(job_dir, pseudo, names35=names35)
    cr = AN.compact_from_metrics(pseudo, m, M.output_digest(job_dir))
    ref = AN.compact_from_f0_analysis(entry, role="B", comparison_class="isometric_comparable")
    for k in ("steps", "episode_return", "end_reason", "horizon_completed", "valid_cycles", "valid_hs", "valid_to", "invalid_events", "knee_rom", "knee_min", "knee_max", "ankle_rom", "ankle_min", "ankle_max", "ankle_negative_fraction", "penetration_max_m", "reserve_norm_max_nm", "action_abs_max", "action_clipped_steps", "resync_count", "hs_cancelled_count", "trace_sha256"):
        check(cr[k] == ref[k], f"F1 extraction == pinned F0 analysis for {k} ({cr[k]} vs {ref[k]})")
    check(cr["valid_cycles"] == 0 and abs(cr["knee_rom"] - 0.336) < 0.001 and cr["steps"] == 500, "B0820_H0 nominal: 0 cycles, ROM 0.336, 500 steps (F0 report)")
    # integrity helpers on synthetic traces
    tmp = Path(tempfile.mkdtemp(prefix="f1_an_"))
    rows = [{"step": k + 1, "time": 12.0 + 0.01 * k, "actor_observation_vector_before": [0.0, 1.0] + [0.1 * k] * 33, "raw_policy_action": [0.0, 0.0], "policy_action_mean": None, "exploration_action_noise": None} for k in range(4)]
    (tmp / "trace.json").write_text(json.dumps(rows))
    (tmp / "reset.json").write_text(json.dumps({"time": 11.99}))
    side = [{"step_index": k + 1, "t_pre": (11.99 if k == 0 else 12.0 + 0.01 * (k - 1)), "targets": [0.1, 0.2, 0.3, 0.4]} for k in range(4)]
    (tmp / "side.json").write_text(json.dumps(side))
    check(AN.t_pre_sidecar_check(tmp / "trace.json", tmp / "reset.json", tmp / "side.json")["ok"], "side-car t_pre rule verified")
    side[2]["t_pre"] += 1e-12
    (tmp / "side2.json").write_text(json.dumps(side))
    check(not AN.t_pre_sidecar_check(tmp / "trace.json", tmp / "reset.json", tmp / "side2.json")["ok"], "side-car t_pre mismatch detected")
    from test_f1_sigma_variant import synthetic_state
    import f1_dataset as DS

    t39 = DS.load_actor_arrays(synthetic_state(39, seed=5), expected_width=39)
    spec = OA.derive_insertion(names35, OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)[0])
    obs39 = OA.insert_targets(np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32), np.asarray([s["targets"] for s in side], dtype=np.float32), spec)
    mu = DS.actor_logits_numpy(t39, obs39.astype(np.float64))[:, :2]
    for k, r in enumerate(rows):
        r["raw_policy_action"] = mu[k].tolist()
    (tmp / "trace_t.json").write_text(json.dumps(rows))
    (tmp / "side.json").write_text(json.dumps([{**s, "t_pre": 0.0} for s in side]))
    tc = AN.teacher_forward_check(tmp / "trace_t.json", tmp / "side.json", t39, spec)
    check(tc["ok"] and tc["max_abs_error"] < 1e-6, "teacher forward check reproduces recorded actions")
    # CRN check on synthetic traces
    b35 = DS.load_actor_arrays(synthetic_state(35, seed=9), expected_width=35)
    obs1 = [0.0, 1.0] + [0.05] * 33
    lg = DS.actor_logits_numpy(b35, np.asarray([obs1], dtype=np.float32).astype(np.float64))[0]
    z = np.array([0.7, -1.2])
    nb = (np.exp(lg[2:]) * z).tolist()
    nc = (0.005 * z).tolist()
    (tmp / "b.json").write_text(json.dumps([{"actor_observation_vector_before": obs1, "exploration_action_noise": nb}]))
    (tmp / "c.json").write_text(json.dumps([{"actor_observation_vector_before": obs1, "exploration_action_noise": nc}]))
    crn = AN.crn_step1_check(tmp / "b.json", tmp / "c.json", b35)
    check(crn["ok"] and crn["max_abs_z_diff"] < 1e-9, "CRN step-1 check recovers identical z")
    (tmp / "c2.json").write_text(json.dumps([{"actor_observation_vector_before": obs1, "exploration_action_noise": (0.005 * np.array([0.7, 1.2])).tolist()}]))
    check(not AN.crn_step1_check(tmp / "b.json", tmp / "c2.json", b35)["ok"], "CRN mismatch detected")
    # outputs
    payload = {"decision": AN.decide(g1, g2, g3), "gates": {"G1": g1, "G2": g2, "G3": g3, "G4": g4, "G5": g5}, "protocol": F1.protocol_digests(), "records": records}
    out = AN.write_outputs(payload, "TEST", metrics_dir=tmp / "metrics", gate_dir=tmp / "gate")
    check((C.REPO / out["analysis"]).is_file() and (C.REPO / out["gate"]).is_file() and (C.REPO / out["markdown"]).is_file(), "analysis/gate/markdown written")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
