"""Self-test for the V26B anchor-collection tooling (V0: synthetic + read-only real checks).

Runs NO rollout, NO fit, NO gate.  Synthetic job directories live in a portable
tempdir; the only real artefacts touched are read-only (pins, manifests, the
pinned V26 teacher and the 3 existing det anchors) plus the authorised
no-clobber dry-run manifests exercised by ``test_v26b_dryrun`` (kept)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_anchors as V  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0


def check(cond: bool, what: str) -> None:
    global CHECKS
    assert cond, what
    CHECKS += 1


def expect(fn, exc, what: str):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {what}")


ERR = V.V26BContractError


# --- synthetic fabrication ------------------------------------------------------------------------

def synth_ctx() -> dict:
    rng = np.random.default_rng(2026)
    h = 16
    arrays = {
        "pi.0.0.weight": rng.normal(0, 0.3, (h, R.MODULE_WIDTH_39)),
        "pi.0.0.bias": rng.normal(0, 0.1, (h,)),
        "pi.0.2.weight": rng.normal(0, 0.3, (h, h)),
        "pi.0.2.bias": rng.normal(0, 0.1, (h,)),
        "pi.1.weight": rng.normal(0, 0.3, (4, h)),
        "pi.1.bias": rng.normal(0, 0.1, (4,)),
    }
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)
    return {"arrays": arrays, "spec": spec, "digest": "synthetic-teacher-digest", "module_state_sha256": "synthetic", "module": "synthetic"}


def fabricate_job(job_dir: Path, ctx: dict, *, seed: int, start: str, sigma: float, steps: int = 500,
                  stochastic: bool = True, end_reason: str = "episode_time_limit",
                  fsm: dict | None = None, obs_from: np.ndarray | None = None,
                  sidecar_rng_seed: int | None = None, mean_tamper: float = 0.0,
                  receipt: bool = True) -> dict:
    """Write a complete synthetic A_iso6clk job dir; returns {obs, t_pre, u_mean, raw}."""
    job_dir.mkdir(parents=True, exist_ok=False)
    n = int(steps)
    reset_time = float(R.EXACT_STARTS[start])
    times = [reset_time + 0.01 * (k + 1) for k in range(n)]
    t_pre = np.asarray(OA.t_pre_from_trace(reset_time, times), dtype=np.float64)
    rng = np.random.default_rng([seed, int(round(sigma * 1e4)), abs(hash(start)) % (2**31)])
    obs = rng.normal(0, 0.5, (n, R.ENV_ACTOR_WIDTH)).astype(np.float32) if obs_from is None else obs_from.astype(np.float32).copy()
    obs[:, 0] = np.float32(0.0)
    obs[:, 1] = np.float32(1.0)
    srng = rng if sidecar_rng_seed is None else np.random.default_rng(sidecar_rng_seed)
    targets = srng.normal(0, 0.3, (n, 4))
    phi = 2.0 * np.pi * 0.5 * t_pre
    clock = np.stack([np.sin(phi), np.cos(phi)], axis=1)
    adapter_rows = [{"step_index": i + 1, "t_pre": float(t_pre[i]), "targets": [float(v) for v in targets[i]], "clock_sin_cos_inserted": [float(clock[i, 0]), float(clock[i, 1])]} for i in range(n)]
    adapter_summary = {"adapter_mode": "aiso6clk", "mode": "aiso6clk", "all_steps_asserted": True, "steps_match_rollout": True, "steps": n,
                       "insertion": {"insert_index": 2, "insert_count": 4, "inserted_names": list(OA.TARGET_FEATURE_NAMES),
                                     "manifest35_sha256": C.ACTOR_MANIFEST_35_SHA256, "manifest39_sha256": C.ACTOR_MANIFEST_39_SHA256}}
    (job_dir / "f1_adapter_trace.json").write_text(json.dumps(adapter_rows), encoding="utf-8")
    (job_dir / "f1_adapter_summary.json").write_text(json.dumps(adapter_summary), encoding="utf-8")
    lab = V._labeller_for_job(ctx, job_dir)
    u_mean = np.asarray(lab.label(obs, t_pre)["actions"], dtype=np.float64)
    noise = rng.normal(0, sigma, (n, 2)) if stochastic else np.zeros((n, 2))
    raw = u_mean + noise
    fsm_row = dict(fsm or {"timeout_exceeded": False, "hs_cancelled_count": 0, "resync_count": 0, "invalid_event_count": 0, "valid_cycle_count": 2})
    trace_rows = []
    for i in range(n):
        row = {"step": i + 1, "time": times[i], "actor_observation_vector_before": [float(v) for v in obs[i]],
               "raw_policy_action": [float(v) for v in raw[i]],
               "policy_action_mean": ([float(u_mean[i, 0] + mean_tamper), float(u_mean[i, 1])] if stochastic else None),
               "phase_fsm": fsm_row}
        trace_rows.append(row)
    (job_dir / "rollout_policy_trace.json").write_text(json.dumps(trace_rows), encoding="utf-8")
    (job_dir / "rollout_summary.json").write_text(json.dumps({"n_actor": R.ENV_ACTOR_WIDTH, "steps": n, "action_seed": int(seed),
                                                              "action_selection": "stochastic" if stochastic else "deterministic",
                                                              "episode_start_offset_s": reset_time, "end_reason": end_reason}), encoding="utf-8")
    (job_dir / "rollout_reset_diagnostics.json").write_text(json.dumps({"time": reset_time}), encoding="utf-8")
    if receipt:
        rec = {"schema": V.RECEIPT_SCHEMA, "job_id": job_dir.name, "sigma": float(sigma), "seed": int(seed), "start": str(start),
               "returncode": 0, "pins": {"runtime_config_sha256": F1.RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": R.ROLLOUT_EVAL_SHA256_PINNED},
               "outputs": {name: C.sha256_file(job_dir / name) for name in V.TRACE_OUTPUT_FILES}}
        (job_dir / V.RECEIPT_FILE).write_text(json.dumps(rec), encoding="utf-8")
    return {"obs": obs, "t_pre": t_pre, "u_mean": u_mean, "raw": raw}


def spec_for(job_dir: Path, *, seed: int, start: str, sigma: float) -> dict:
    return {"job_id": job_dir.name, "sigma": float(sigma), "sigma_tag": V.sigma_tag(sigma), "seed": int(seed), "start": str(start), "dir": job_dir}


def main() -> int:
    tmp = R.portable_tempdir("v26b_selftest_")
    try:
        ctx = synth_ctx()

        # --- protocol binding ---------------------------------------------------------------
        proto = V.load_protocol()
        check(proto["protocol_id"] == "V26B-bridge-rev3", "protocol id rev3")
        check(proto["anchors"]["coverage_target_mandatory_no_waiver"]["min_bitwise_unique_rows"] == 16000, "protocol coverage 16000")
        bad = tmp / "bad_protocol.json"
        mutated = json.loads(V.PROTOCOL_JSON.read_text())
        mutated["anchors"]["coverage_target_mandatory_no_waiver"]["min_bitwise_unique_rows"] = 1500
        bad.write_text(json.dumps(mutated))
        orig_proto = V.PROTOCOL_JSON
        try:
            V.PROTOCOL_JSON = bad
            expect(V.load_protocol, ERR, "protocol coverage weakened -> tooling refuses")
            mutated["anchors"]["coverage_target_mandatory_no_waiver"]["min_bitwise_unique_rows"] = 16000
            mutated["anchors"]["anchor_only_seeds_preregistered"]["seeds"] = [1000, 1001, 1002, 125]
            bad.write_text(json.dumps(mutated))
            expect(V.load_protocol, ERR, "protocol anchor seeds altered -> refuses")
            mutated["anchors"]["anchor_only_seeds_preregistered"]["seeds"] = [1000, 1001, 1002, 1003]
            mutated["protocol_id"] = "V26B-bridge-rev2"
            bad.write_text(json.dumps(mutated))
            expect(V.load_protocol, ERR, "rev2 protocol -> tooling only runs under rev3")
        finally:
            V.PROTOCOL_JSON = orig_proto
        check(V.load_protocol()["protocol_id"] == "V26B-bridge-rev3", "binding restored")

        # --- seed governance ----------------------------------------------------------------
        for s in (123, 124, 125, 126, 127, 128):
            e = expect(lambda s=s: V.assert_anchor_seed(s, context="anchor_collection_stoch"), ERR, f"reserved seed {s} refused for anchor collection")
            check("reserved" in str(e), f"refusal for {s} names the reserved role")
        for s in V.ANCHOR_SEEDS:
            check(V.assert_anchor_seed(s, context="anchor_collection_stoch") == s, f"anchor seed {s} accepted")
        expect(lambda: V.assert_anchor_seed(999, context="anchor_collection_stoch"), ERR, "non-preregistered seed refused (no automatic seeds)")
        expect(lambda: V.assert_anchor_seed(1004, context="anchor_collection_stoch"), ERR, "1004 not preregistered")
        for ctx_name in ("dagger", "gate", "holdout", "sealed"):
            expect(lambda c=ctx_name: V.assert_anchor_seed(1000, context=c), ERR, f"anchor seed refused in {ctx_name} context")
            check(V.assert_anchor_seed(123, context=ctx_name) == 123, f"F2R seeds pass through {ctx_name} (governed by F2R rules)")
        expect(lambda: V.assert_anchor_seed(1000, context="unknown"), ERR, "unknown context refused")
        expect(lambda: V.assert_anchor_seed(True, context="anchor_collection_stoch"), ERR, "bool seed refused")
        expect(lambda: R.assert_collection_seed(1000), Exception, "F2R collection rules untouched: 1000 is not a collection seed")

        # --- planner ------------------------------------------------------------------------
        specs = V.job_specs()
        check(len(specs) == 36, "36 planned jobs")
        combos = {(s["sigma"], s["start"], s["seed"]) for s in specs}
        check(len(combos) == 36 and {c[0] for c in combos} == set(V.SIGMA_GRID) and {c[2] for c in combos} == set(V.ANCHOR_SEEDS), "3 sigmas x 3 starts x 4 seeds")
        check(len({s["job_id"] for s in specs}) == 36 and len({str(s["dir"]) for s in specs}) == 36, "unique ids and dirs")
        check(all(s["enters_dataset"] is True and s["action_selection"] == "stochastic" and s["adapter_mode"] == "aiso6clk" for s in specs), "all jobs stochastic aiso6clk dataset-bound")
        cmds = [V.job_command(s, "/usr/bin/python3") for s in specs]
        for s, cmd in zip(specs, cmds):
            check(cmd[-2:] == ["--f1-adapter", "aiso6clk"] and "--action-selection" in cmd and cmd[cmd.index("--action-selection") + 1] == "stochastic", f"{s['job_id']}: aiso6clk stochastic command")
            check(cmd[cmd.index("--seed") + 1] == str(s["seed"]) and cmd[cmd.index("--episode-start-offset-s") + 1] == repr(float(R.EXACT_STARTS[s["start"]])), f"{s['job_id']}: exact seed/start")
            check(cmd[cmd.index("--config") + 1] == str(F1.RUNTIME_CONFIG) and str(V.OUT_DERIVED) in cmd[cmd.index("--checkpoint") + 1], f"{s['job_id']}: pinned config + derived module")
            check(not any(tok in (str(x) for x in cmd) for tok in ("123", "124", "125", "126", "127", "128")), f"{s['job_id']}: no reserved seed token in the command")
        modules = V.sigma_module_plan()
        check(len(modules) == 3 and [m["sigma"] for m in modules] == list(V.SIGMA_GRID), "3 sigma modules on the frozen grid")
        check(all(m["expected_source_actor_digest"] == R.TEACHER["actor_digest"] for m in modules), "sigma modules pinned to the V26 source digest")
        check(V.sigma_tag(0.005) == "s0005", "sigma tag")
        expect(lambda: V.sigma_tag(0.02), ERR, "off-grid sigma refused")
        check(V.EXPECTED_STOCH_TRACES == 36 and V.MIN_UNIQUE_ROWS == 16000 and V.MIN_COMPLETE_TRACES_PER_SIGMA == 3, "frozen coverage constants")

        # --- synthetic happy path: 36 complete traces, no det ------------------------------
        sroot = tmp / "stoch"
        fab_specs, fabs = [], {}
        for s in V.job_specs():
            d = sroot / s["dir"].name
            fabs[d.name] = fabricate_job(d, ctx, seed=s["seed"], start=s["start"], sigma=s["sigma"])
            fab_specs.append(spec_for(d, seed=s["seed"], start=s["start"], sigma=s["sigma"]))
        arrays, report = V.collect_anchor_rows(fab_specs, ctx, include_det=False)
        check(report["pass"] is True and report["unique_rows"] == 36 * 500 and report["raw_rows"] == 36 * 500, "36x500 unique rows >= 16000 -> PASS")
        check(all(n == 12 for n in report["complete_traces_by_sigma"].values()), "12 complete traces per sigma")
        check(arrays["obs35"].shape == (18000, 35) and arrays["targets"].shape == (18000, 2) and arrays["targets"].dtype == np.float32, "collected array shapes/dtype")
        check(report["coverage_policy"]["waiver"].startswith("NONE"), "report states: no waiver")
        one = fab_specs[0]
        fb = fabs[one["job_id"]]
        sel = arrays["job_id"] == one["job_id"]
        check(int(np.sum(sel)) == 500, "per-trace rows present")
        got = arrays["targets"][sel]
        check(np.array_equal(got, fb["u_mean"].astype(np.float32)), "targets == deterministic V26 mean bitwise")
        check(float(np.max(np.abs(fb["raw"] - fb["u_mean"]))) > 1e-4 and not np.allclose(got.astype(np.float64), fb["raw"]), "targets are NOT the sampled raw actions")

        # --- dedup + conflicts (small scale; CoverageStop still carries the counts) --------
        droot = tmp / "dup"
        a = fabricate_job(droot / "A", ctx, seed=1000, start="nominal", sigma=0.005)
        fabricate_job(droot / "B", ctx, seed=1000, start="nominal", sigma=0.005)  # identical fabrication -> bit-identical rows
        e = expect(lambda: V.collect_anchor_rows([spec_for(droot / "A", seed=1000, start="nominal", sigma=0.005), spec_for(droot / "B", seed=1000, start="nominal", sigma=0.005)], ctx, include_det=False), V.CoverageStop, "500 unique < 16000 -> STOP")
        check(e.report["bitwise_duplicates_removed"] == 500 and e.report["unique_rows"] == 500, "bitwise dedup removed the duplicated trace")
        check("architect" in str(e) and "seed" in str(e).lower(), "STOP message: architect decision, no automatic seeds")
        fabricate_job(droot / "Cnf", ctx, seed=1001, start="nominal", sigma=0.005, obs_from=a["obs"], sidecar_rng_seed=77)  # same obs, different privileged grid -> different mean
        expect(lambda: V.collect_anchor_rows([spec_for(droot / "A", seed=1000, start="nominal", sigma=0.005), spec_for(droot / "Cnf", seed=1001, start="nominal", sigma=0.005)], ctx, include_det=False), ERR, "duplicate obs with conflicting targets -> fail-closed")

        # --- quality exclusions (counted, zero rows) ----------------------------------------
        qroot = tmp / "quality"
        fabricate_job(qroot / "short", ctx, seed=1000, start="nominal", sigma=0.0025, steps=300)
        e = expect(lambda: V.collect_anchor_rows([spec_for(qroot / "short", seed=1000, start="nominal", sigma=0.0025)], ctx, include_det=False), V.CoverageStop, "incomplete trace contributes no rows")
        check(e.report["unique_rows"] == 0 and e.report["traces"][0]["complete"] is False and e.report["traces"][0]["steps"] == 300, "300-step trace excluded and counted")
        fabricate_job(qroot / "pen", ctx, seed=1001, start="nominal", sigma=0.0025, end_reason="grf_penetration_hard_limit")
        e = expect(lambda: V.collect_anchor_rows([spec_for(qroot / "pen", seed=1001, start="nominal", sigma=0.0025)], ctx, include_det=False), V.CoverageStop, "500-step penetration-terminated trace still not complete")
        check(e.report["traces"][0]["complete"] is False and e.report["traces"][0]["end_reason"] == "grf_penetration_hard_limit", "end_reason recorded")
        fabricate_job(qroot / "tmo", ctx, seed=1002, start="nominal", sigma=0.0025, fsm={"timeout_exceeded": True, "hs_cancelled_count": 0, "resync_count": 0, "invalid_event_count": 0, "valid_cycle_count": 0})
        e = expect(lambda: V.collect_anchor_rows([spec_for(qroot / "tmo", seed=1002, start="nominal", sigma=0.0025)], ctx, include_det=False), V.CoverageStop, "fsm timeout -> contract failure -> excluded")
        check(e.report["traces"][0]["contract_failure"] is True, "contract failure flagged")
        fabricate_job(qroot / "morph", ctx, seed=1003, start="nominal", sigma=0.0025, end_reason="morphology_causal_contract_failure")
        e = expect(lambda: V.collect_anchor_rows([spec_for(qroot / "morph", seed=1003, start="nominal", sigma=0.0025)], ctx, include_det=False), V.CoverageStop, "morphology contract end_reason -> excluded")
        check(e.report["traces"][0]["contract_failure"] is True, "morphology failure flagged as contract failure")
        # per-sigma completeness: 12 complete for two sigmas, only 2 for s001 -> named in the STOP
        thin = [sp for sp in fab_specs if sp["sigma_tag"] != "s001" or (sp["start"] == "nominal" and sp["seed"] in (1000, 1001))]
        e = expect(lambda: V.collect_anchor_rows(thin, ctx, include_det=False), V.CoverageStop, "a sigma below 3 complete traces fails")
        check("s001" in str(e) and e.report["complete_traces_by_sigma"]["s001"] == 2, "STOP names the under-covered sigma (2 < 3)")

        # --- hard errors --------------------------------------------------------------------
        hroot = tmp / "hard"
        fabricate_job(hroot / "norec", ctx, seed=1000, start="nominal", sigma=0.01, receipt=False)
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "norec", seed=1000, start="nominal", sigma=0.01)], ctx, include_det=False), ERR, "missing receipt -> fail-closed")
        fabricate_job(hroot / "badpin", ctx, seed=1001, start="nominal", sigma=0.01)
        rec = json.loads((hroot / "badpin" / V.RECEIPT_FILE).read_text()); rec["pins"]["runtime_config_sha256"] = "0" * 64
        (hroot / "badpin" / V.RECEIPT_FILE).write_text(json.dumps(rec))
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "badpin", seed=1001, start="nominal", sigma=0.01)], ctx, include_det=False), ERR, "receipt with wrong runtime pin -> fail-closed")
        fabricate_job(hroot / "tamper", ctx, seed=1002, start="nominal", sigma=0.01)
        tr = hroot / "tamper" / "rollout_summary.json"
        tr.write_text(tr.read_text() + " ")
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "tamper", seed=1002, start="nominal", sigma=0.01)], ctx, include_det=False), ERR, "output modified after the receipt -> digest mismatch")
        fabricate_job(hroot / "wseed", ctx, seed=1003, start="nominal", sigma=0.01)
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "wseed", seed=1001, start="nominal", sigma=0.01)], ctx, include_det=False), ERR, "recorded action_seed != declared -> fail-closed")
        fabricate_job(hroot / "det", ctx, seed=1000, start="minus020", sigma=0.01, stochastic=False)
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "det", seed=1000, start="minus020", sigma=0.01)], ctx, include_det=False), ERR, "deterministic trace under a stochastic spec -> fail-closed")
        fabricate_job(hroot / "mean", ctx, seed=1001, start="minus020", sigma=0.01, mean_tamper=1e-3)
        e = expect(lambda: V.collect_anchor_rows([spec_for(hroot / "mean", seed=1001, start="minus020", sigma=0.01)], ctx, include_det=False), ERR, "recorded mean != offline V26 mean -> teacher/runtime mismatch")
        check("teacher/runtime" in str(e), "mean-mismatch error names the cause")
        fabricate_job(hroot / "amode", ctx, seed=1002, start="minus020", sigma=0.01)
        asum = hroot / "amode" / "f1_adapter_summary.json"
        payload = json.loads(asum.read_text()); payload["adapter_mode"] = "aiso4"; payload["mode"] = "aiso4"
        asum.write_text(json.dumps(payload))
        rec = json.loads((hroot / "amode" / V.RECEIPT_FILE).read_text()); rec["outputs"]["f1_adapter_summary.json"] = C.sha256_file(asum)
        (hroot / "amode" / V.RECEIPT_FILE).write_text(json.dumps(rec))
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "amode", seed=1002, start="minus020", sigma=0.01)], ctx, include_det=False), Exception, "aiso4 sidecar -> privileged cache refused")
        expect(lambda: V.collect_anchor_rows([spec_for(hroot / "missing", seed=1003, start="minus020", sigma=0.01)], ctx, include_det=False), ERR, "missing job dir -> fail-closed")
        e = expect(lambda: V.collect_anchor_rows([{**spec_for(hroot / "det", seed=125, start="minus020", sigma=0.01)}], ctx, include_det=False), ERR, "reserved seed inside a spec -> refused before any read")
        check("reserved" in str(e), "spec-level seed guard active")

        # --- real read-only: preflight, teacher, existing det anchors ------------------------
        flight = V.preflight()
        check(flight["det_anchors_all_match"] is True and flight["protocol_id"] == "V26B-bridge-rev3", "real preflight passes")
        check(flight["driver_inputs"]["rollout_eval_sha256"] == R.ROLLOUT_EVAL_SHA256_PINNED, "rollout_eval pinned")
        check(flight["driver_inputs"]["runtime_config"]["sha256"] == F1.RUNTIME_CONFIG_SHA256, "runtime config pinned")
        rctx = V.teacher_context()
        check(rctx["digest"] == R.TEACHER["actor_digest"] and rctx["module_state_sha256"] == R.TEACHER["module_state_sha256"], "real V26 teacher digests match the pins")
        det_rows, det_reports = V.det_anchor_rows(rctx)
        check(sum(r["obs35"].shape[0] for r in det_rows) == 1500 and len(det_reports) == 3, "existing det anchors: 1500 rows")
        check(all(rep["complete"] and rep["mean_match_max_abs_dev"] <= 1e-5 for rep in det_reports), "det anchors: recorded action == offline V26 mean <= 1e-5")
        e = expect(lambda: V.collect_anchor_rows([], rctx, include_det=True), V.CoverageStop, "det-only coverage (1500) < 16000 -> STOP, architect decision")
        check(e.report["det_anchor_rows"] == 1500 and e.report["unique_rows"] <= 1500 and e.report["pass"] is False, "det-only report: 1500 rows, below target")
        check(all(n == 0 for n in e.report["complete_traces_by_sigma"].values()), "det-only: zero stochastic traces per sigma")

        # --- sigma-module materialization smoke (isolated tempdir, real V26 source, NO rollout)
        # regression for the blocking V0 bug: run_collection pre-created plan["out_dir"] while
        # f1_sigma_variant refuses an existing output_dir -> the first derivation always failed.
        # The helper must NOT pre-create the dir: deriving from a nonexistent dir must succeed.
        sm = tmp / "smoke_derived"
        plans = [{**pl, "out_dir": sm / pl["name"], "module": sm / pl["name"] / "rl_module"} for pl in V.sigma_module_plan()]
        reports = V.derive_sigma_modules(plans)
        check(len(reports) == 3 and [r["sigma"] for r in reports] == list(V.SIGMA_GRID), "3 modules materialised on the frozen grid")
        for pl, rep in zip(plans, reports):
            check(rep["mode"] == "materialize" and rep["source_actor_digest"] == R.TEACHER["actor_digest"], f"{pl['name']}: materialised from the pinned V26 source")
            check((Path(pl["module"]) / "module_state.pkl").is_file(), f"{pl['name']}: rl_module/module_state.pkl on disk")
            receipt_name = "f1_sigma_variant_receipt.json" if pl["sigma"] == 0.005 else V.V26B_SIGMA_RECEIPT
            check((Path(pl["out_dir"]) / "derivation_report.json").is_file() and (Path(pl["out_dir"]) / receipt_name).is_file(), f"{pl['name']}: derivation report + receipt written")
            check(rep["derived_head"]["logstd_bias_exact"] is True, f"{pl['name']}: log-std bias exact for its sigma")
        # equivalence of the v26b mirror with the unmodified F1 tool at sigma=0.005 (bit-identical)
        eq = tmp / "smoke_equiv"
        rep_f1 = V.SV.derive_constant_sigma_module(Path(V.R.TEACHER["module"]), eq / "f1", sigma=0.005, materialize=True, expected_source_actor_digest=R.TEACHER["actor_digest"])
        rep_mi = V._derive_constant_sigma_v26b(Path(V.R.TEACHER["module"]), eq / "mirror", sigma=0.005, expected_source_actor_digest=R.TEACHER["actor_digest"])
        check(rep_f1["derived_actor_digest"] == rep_mi["derived_actor_digest"], "mirror at 0.005: same derived actor digest as the F1 tool")
        import f0_common as _C
        check(_C.sha256_file(eq / "f1" / "rl_module" / "module_state.pkl") == _C.sha256_file(eq / "mirror" / "rl_module" / "module_state.pkl"), "mirror at 0.005: bit-identical module_state.pkl")
        check(rep_mi["derived_head"]["entropy_within_tolerance_v26b"] is True and abs(rep_mi["derived_head"]["entropy_expected_nat_v26b"] - (-7.758758)) < 1e-4, "mirror entropy expectation reproduces the F1 constant at 0.005")
        # partial-failure lifecycle: pre-existing dir for the SECOND module -> fail-fast after the first
        sm2 = tmp / "smoke_partial"
        plans2 = [{**pl, "out_dir": sm2 / pl["name"], "module": sm2 / pl["name"] / "rl_module"} for pl in V.sigma_module_plan()]
        Path(plans2[1]["out_dir"]).mkdir(parents=True)
        e = expect(lambda: V.derive_sigma_modules(plans2), ERR, "existing module dir -> refused before touching the F1 tool")
        check("no-clobber" in str(e) and "architect" in str(e) and str(plans2[1]["out_dir"]) in str(e), "diagnostic names the offending dir, no automatic retry")
        check((Path(plans2[0]["module"]) / "module_state.pkl").is_file() and (Path(plans2[0]["out_dir"]) / "derivation_report.json").is_file(), "module derived before the failure stays on disk untouched")
        check(not Path(plans2[2]["out_dir"]).exists(), "fail-fast: no module derived after the failure")
        check(not any(Path(plans2[1]["out_dir"]).iterdir()), "pre-existing dir left exactly as found")
        e = expect(lambda: V.derive_sigma_modules(plans2), ERR, "re-invocation after partial failure -> stops on the FIRST existing dir, no silent retry")
        check(str(plans2[0]["out_dir"]) in str(e), "re-invocation refusal names the first already-derived module")

        # --- collection guard (never runs in V0) --------------------------------------------
        e = expect(lambda: V.run_collection(python_exe=sys.executable, workers=1, authorized_stage=None), ERR, "collection without the stage token refused")
        check("V26B-ANCHORS" in str(e), "refusal names the required stage token")
        expect(lambda: V.run_collection(python_exe=sys.executable, workers=1, authorized_stage="S1"), ERR, "wrong stage token refused")
        check(not V.OUT_ROLLOUTS.exists() and not V.OUT_DERIVED.exists() and not V.OUT_COVERAGE.exists(), "refused collection created no rollout/module/coverage artefacts")

        print(f"SELFTEST PASS ({CHECKS} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
