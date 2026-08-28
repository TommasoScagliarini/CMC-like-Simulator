"""Self-test of f1_matrix: job matrix, command binding, dry-run manifest in a
temporary root, run_job end-to-end with a fake subprocess (temp only; no real
rollout is launched; real F0/F1 roots are never written)."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f0_closure as CL  # noqa: E402
import f1_matrix as MX  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect(fn, exc, label):
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


def redirect_roots(tmp: Path) -> None:
    F1.OUT_ROOT = tmp / "root"
    F1.OUT_MANIFEST = F1.OUT_ROOT / "manifest"
    F1.OUT_ROLLOUTS = F1.OUT_ROOT / "rollouts"
    F1.OUT_DERIVED = F1.OUT_ROOT / "derived_modules"
    F1.OUT_DATASETS = F1.OUT_ROOT / "datasets"
    F1.OUT_REFIT = F1.OUT_ROOT / "refit"
    F1.OUT_METRICS = F1.OUT_ROOT / "metrics"
    F1.OUT_GATE = F1.OUT_ROOT / "gate"
    F1.OUT_LOGS = F1.OUT_ROOT / "logs"
    MX.DERIVED_MODULE_DIRS = {"C": F1.OUT_DERIVED / "B_sigma0005" / "rl_module", "A_ISO39_V3_S005": F1.OUT_DERIVED / "V26_39D_sigma0005" / "rl_module", "D": F1.OUT_REFIT / "D_refit_v1" / "rl_module_refit"}


def fake_subprocess_factory(marker: dict, *, tamper_sidecar: bool = False, fail: bool = False):
    def fake_run(command, cwd=None, stdout=None, stderr=None, text=None, check=None):
        marker["launched"] = marker.get("launched", 0) + 1
        out = Path(command[command.index("--output-dir") + 1])
        out.mkdir(parents=True, exist_ok=True)
        steps = 5
        if not fail:
            (out / "rollout_summary.json").write_text(json.dumps({"ok": True, "steps": steps, "episode_return": 1.0, "end_reason": "episode_time_limit", "n_actor": 35, "action_seed": int(command[command.index("--seed") + 1]), "action_selection": command[command.index("--action-selection") + 1], "episode_start_offset_s": float(command[command.index("--episode-start-offset-s") + 1])}), encoding="utf-8")
            (out / "rollout_policy_trace.json").write_text(json.dumps([{"step": k + 1, "time": 12.0 + 0.01 * k} for k in range(steps)]), encoding="utf-8")
            if "--f1-adapter" in command:
                mode = command[command.index("--f1-adapter") + 1]
                (out / "f1_adapter_summary.json").write_text(json.dumps({"adapter_mode": mode, "steps": steps, "projection_assert_count": steps if not tamper_sidecar else steps - 1, "all_steps_asserted": not tamper_sidecar, "steps_match_rollout": True}), encoding="utf-8")
                (out / "f1_adapter_trace.json").write_text(json.dumps([{"step_index": k + 1, "t_pre": 11.99 + 0.01 * k, "targets": [0, 0, 0, 0]} for k in range(steps)]), encoding="utf-8")
        if stdout is not None:
            stdout.write("fake rollout\n")
        return SimpleNamespace(returncode=1 if fail else 0)
    return fake_run


def symlink_free_tmp(prefix: str) -> Path:
    """Temp dir without symlinked components (the F0 closure validator refuses
    symlinks; macOS /var -> /private/var): prefer the session scratchpad."""
    import os

    for base in (os.environ.get("CMC_F1_TEST_TMP"), "/private/tmp", tempfile.gettempdir()):
        if not base:
            continue
        root = Path(base)
        if root.is_dir() and os.path.realpath(root) == str(root):
            return Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
    return Path(os.path.realpath(tempfile.mkdtemp(prefix=prefix)))


def main() -> int:
    tmp = symlink_free_tmp("f1_matrix_")
    redirect_roots(tmp)
    jobs = MX.build_jobs()
    check(len(jobs) == 53, "53 jobs")
    fam = {f: sum(1 for j in jobs if j["family"] == f) for f in F1.FAMILIES}
    check(fam == {"aiso_det": 3, "aiso_stoch_s005": 9, "b_det": 3, "b_stoch_native": 10, "c_det": 3, "c_stoch_s005": 9, "d_det": 3, "d_stoch_s005": 9, "passthrough": 1, "aiso_clk_diag": 3}, f"family counts {fam}")
    check({j["stage"] for j in jobs} == {1, 3} and sum(1 for j in jobs if j["stage"] == 3) == 12, "stages 1 and 3")
    check(all(j["seed"] in F1.DEVELOPMENT_SEEDS for j in jobs), "only development seeds")
    check(len({j["job_id"] for j in jobs}) == 53, "unique ids")
    check(all(j["driver"] == "f1_rollout_aiso" for j in jobs if j["candidate"].startswith("A_ISO") or j["candidate"] == "PASSTHROUGH_B") and all(j["driver"] == "rollout_eval" for j in jobs if j["candidate"] in ("B", "C", "D")), "drivers per candidate")
    check(all(not j["gate_relevant"] for j in jobs if j["candidate"] == "A_ISO39CLK_V3"), "clock diagnostic not gate relevant")
    idx = MX.canonical_index()
    b_nom = idx["B__v3_canonical__nominal__det"]
    a_nom = idx["A_ISO39_V3__v3_canonical__nominal__det"]
    cmd_b = MX.job_command(b_nom, MX.output_dir_for(b_nom), "PY")
    cmd_a = MX.job_command(a_nom, MX.output_dir_for(a_nom), "PY")
    check(cmd_b[1] == str(F1.ROLLOUT_EVAL) and "--f1-adapter" not in cmd_b and "--no-auto-config" in cmd_b and cmd_b[cmd_b.index("--config") + 1] == str(F1.RUNTIME_CONFIG), "rollout_eval command")
    check(cmd_a[1] == str(F1.AISO_DRIVER) and cmd_a[-2:] == ["--f1-adapter", "aiso4"] and cmd_a[cmd_a.index("--checkpoint") + 1] == str(F1.CANDIDATES["A_ISO39_V3"]["module"]), "A_iso command")
    check(cmd_a[cmd_a.index("--episode-start-offset-s") + 1] == repr(F1.EXACT_STARTS["nominal"]) == "1.956870983805102", "exact start repr")
    # describe: frozen vs pending
    py = sys.executable
    rb = MX.describe_job(b_nom, py)
    check(rb["module_pending"] is False and len(rb["module_state_sha256"]) == 64 and rb["job_inputs_count"] >= 10 and rb["schema_version"] == "f1.1", "described frozen B job with inputs")
    check(rb["config_sha256"] == F1.RUNTIME_CONFIG_SHA256 and rb["driver_sha256"] == C.sha256_file(F1.ROLLOUT_EVAL) == rb["rollout_eval_sha256"], "pinned config / driver digests")
    rc = MX.describe_job(idx["C__v3_canonical__nominal__det"], py)
    check(rc["module_pending"] is True and rc["module_state_sha256"] is None and rc["job_inputs"] is None and "stage 1" in rc["module_pending_reason"], "pending derived module flagged")
    rd = MX.describe_job(idx["D__v3_canonical__nominal__det"], py)
    check(rd["module_pending"] and "stage 2" in rd["module_pending_reason"], "pending refit module flagged")
    ra = MX.describe_job(a_nom, py)
    check(ra["driver"] == "f1_rollout_aiso" and ra["driver_sha256"] == C.sha256_file(F1.AISO_DRIVER) and ra["module_width"] == 39 and ra["env_width"] == 35, "A_iso description")
    # command binding
    check(MX.command_binding_problems(rb) == {}, "genuine description binds")
    mut = dict(rb)
    mut["command"] = list(rb["command"])
    mut["command"][0] = "/not/python"
    check("command[0]" in MX.command_binding_problems(mut) or "command_normalized" in MX.command_binding_problems(mut), "mutated interpreter rejected")
    mut2 = dict(rb)
    mut2["command"] = [t for t in rb["command"] if t != "--no-auto-config"]
    mut2["command_normalized"] = MX_norm = __import__("f0_rollout_matrix").normalize_command(mut2["command"])
    p2 = MX.command_binding_problems(mut2)
    check("--no-auto-config" in p2 or "command_argv" in p2, "missing --no-auto-config rejected")
    mut3 = dict(rb)
    mut3["driver_sha256"] = "0" * 64
    check("driver_sha256" in MX.command_binding_problems(mut3), "driver sha mismatch rejected")
    mut4 = dict(rb)
    mut4["output_dir"] = rb["output_dir"] + "_x"
    check("output_dir" in MX.command_binding_problems(mut4), "output dir mismatch rejected")
    mut5 = dict(ra)
    mut5["command"] = ra["command"][:-2] + ["--f1-adapter", "aiso6clk"]
    mut5["command_normalized"] = __import__("f0_rollout_matrix").normalize_command(mut5["command"])
    check("--f1-adapter" in MX.command_binding_problems(mut5) or "command_argv" in MX.command_binding_problems(mut5), "adapter mode mismatch rejected")
    check("job_id" in MX.command_binding_problems({**rb, "job_id": "bogus"}), "unknown job id rejected")
    # dry-run manifest in the temp root
    described = [MX.describe_job(s, py) for s in jobs]
    index = MX.write_dry_run_manifest(described, stamp="TEST", python_record={"selected": py, "source": "test"})
    mpath = C.REPO / index["manifest"]
    manifest = json.loads(mpath.read_text(encoding="utf-8"))
    check(manifest["dry_run"] is True and manifest["job_count"] == 53 and not manifest["seeds"]["sealed_seed_found"], "dry-run manifest written")
    check(manifest["reconstruction_config_equality"]["equal"] is True, "v26/v3 reconstruction keys equal")
    check(manifest["insertion_spec"]["insert_index"] == 2 and manifest["insertion_spec"]["insert_count"] == 4, "insertion spec in manifest")
    check(manifest["reuse_f0"]["f0_analysis_pin_ok"] is True and manifest["reuse_f0"]["jobs"]["A_NATIVE_39D"]["count"] == 3 and manifest["reuse_f0"]["jobs"]["A_NATIVE_39D"]["all_ok"], "F0 ctrl39 reuse pins verified")
    check(manifest["reuse_f0"]["jobs"]["E"]["count"] == 12 and manifest["reuse_f0"]["jobs"]["B_F0_REFERENCE"]["count"] == 12 and manifest["reuse_f0"]["jobs"]["E"]["all_ok"], "F0 JUL_H0 / B0820_H0 reuse pins verified")
    check(manifest["dirty_user_files"]["training_exnovo_cfg_unchanged_since_F0"] is True, "dirty user config unchanged since F0")
    check(len(manifest["jobs_pending_module"]) == 33, f"pending modules: {len(manifest['jobs_pending_module'])} (C 12 + A_ISO_S005 9 + D 12)")
    check(all("<" not in t for cmds in manifest["stage_commands"].values() for cmd in cmds for t in cmd), "stage commands have no placeholders")
    check((C.REPO / index["markdown"]).is_file() and (F1.OUT_MANIFEST / "f1_job_matrix_dry_run_TEST.index.json").is_file(), "markdown + index written")
    expect(lambda: MX.write_dry_run_manifest(described, stamp="TEST", python_record={"selected": py}), FileExistsError, "dry-run manifest no-clobber")
    # run_job end-to-end with a fake subprocess
    orch = tuple(str(F1.F0_DIR / p) for p in CL.ORCHESTRATION_FILES) + tuple(str(HERE / p) for p in MX.F1_SCRIPTS if (HERE / p).is_file())
    snapshot_fn = lambda: CL.closure_snapshot(py, orchestration_files=orch)  # noqa: E731
    snapshot = snapshot_fn()
    selected = [rb, ra]
    bindings = {r["job_id"]: r["job_inputs_digest"] for r in selected}
    F1.ensure_out_dirs()
    written = CL.write_closure_manifest(F1.OUT_ROLLOUTS / "source_closure_manifest_TEST.json", snapshot, job_inputs=bindings)
    closure = {**written, "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"], "launch_snapshot": snapshot, "job_inputs": bindings, "snapshot_fn": snapshot_fn}
    log = F1.OUT_ROLLOUTS / "driver_log.jsonl"
    marker: dict = {}
    res_b = MX.run_job(dict(rb), log_path=log, closure=closure, subprocess_run=fake_subprocess_factory(marker))
    check(res_b["status"] == "ok" and res_b["returncode"] == 0 and marker["launched"] == 1 and res_b["summary_ok"] and res_b["closure_manifest_unchanged"], "B job ok with fake rollout")
    check(CL.verify_closure_fields(res_b) == {}, "F1 receipt verifies with the unchanged F0 closure validator")
    out_b = MX.output_dir_for(b_nom)
    check((out_b / MX.RECEIPT_FILE).is_file() and (out_b / MX.STDOUT_LOG).is_file(), "receipt + stdout log written")
    res_b2 = MX.run_job(dict(rb), log_path=log, closure=closure, subprocess_run=fake_subprocess_factory(marker))
    check(res_b2["status"] == "skipped_verified_identical" and marker["launched"] == 1, "second run skipped after verification, no relaunch")
    res_a = MX.run_job(dict(ra), log_path=log, closure=closure, subprocess_run=fake_subprocess_factory(marker))
    check(res_a["status"] == "ok" and res_a["adapter_ok"] and res_a["adapter_mode_recorded"] == "aiso4" and len(res_a["adapter_trace_sha256"]) == 64, "A_iso job ok with side-cars")
    # tampered trace -> preflight fails closed
    trace = out_b / "rollout_policy_trace.json"
    trace.write_text(trace.read_text() + " ", encoding="utf-8")
    expect(lambda: MX.preflight_all([rb]), MX.MatrixError, "tampered trace refused by preflight")
    expect(lambda: MX.run_job(dict(rb), log_path=log, closure=closure, subprocess_run=fake_subprocess_factory(marker)), MX.MatrixError, "run_job refuses a tampered existing output")
    check(marker["launched"] == 2, "no relaunch on tampered output")
    # adapter side-car invalid -> failed
    idx_a2 = idx["A_ISO39_V3__v3_canonical__plus020__det"]
    ra2 = MX.describe_job(idx_a2, py)
    closure2 = {**closure, "job_inputs": {**bindings, ra2["job_id"]: ra2["job_inputs_digest"]}}
    w2 = CL.write_closure_manifest(F1.OUT_ROLLOUTS / "source_closure_manifest_TEST2.json", snapshot, job_inputs=closure2["job_inputs"])
    closure2.update(w2)
    res_a2 = MX.run_job(dict(ra2), log_path=log, closure=closure2, subprocess_run=fake_subprocess_factory(marker, tamper_sidecar=True))
    check(res_a2["status"] == "failed" and res_a2["status_reason"] == "adapter side-car invalid", "side-car assertion shortfall => failed")
    # failing rollout
    idx_b2 = idx["B__v3_canonical__plus020__det"]
    rb2 = MX.describe_job(idx_b2, py)
    closure3 = {**closure, "job_inputs": {**bindings, rb2["job_id"]: rb2["job_inputs_digest"]}}
    closure3.update(CL.write_closure_manifest(F1.OUT_ROLLOUTS / "source_closure_manifest_TEST3.json", snapshot, job_inputs=closure3["job_inputs"]))
    res_b3 = MX.run_job(dict(rb2), log_path=log, closure=closure3, subprocess_run=fake_subprocess_factory(marker, fail=True))
    check(res_b3["status"] == "failed" and res_b3["returncode"] == 1, "rc 1 => failed")
    # pending module refused; mutated receipt refused before launch
    expect(lambda: MX.run_job(dict(rc), log_path=log, closure=closure), MX.MatrixError, "pending module refused")
    expect(lambda: MX.run_job(mut3, log_path=log, closure=closure), MX.MatrixError, "mutated receipt refused before launch")
    check(marker["launched"] == 4, "exactly four fake launches overall (B, A_iso, A_iso tampered side-car, failing B)")
    # the real roots were never touched by this test
    real_root = C.RUNS_ROLLOUT / "validation" / "f1_ablation_runs" / f"{F1.F1_TAG}_{F1.F1_REV}"
    check(not (real_root / "rollouts" / "b_det").exists(), "real F1 rollout root untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
