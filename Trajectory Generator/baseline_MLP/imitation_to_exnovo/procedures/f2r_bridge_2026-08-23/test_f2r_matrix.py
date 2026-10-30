"""Self-test of f2r_matrix: job specs, seed policy, command binding, receipt closure fields,
dry-run manifest in a temp output root (no-clobber), execution refusals (pending module)."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f0_closure as CL  # noqa: E402
import f2r_matrix as MX  # noqa: E402
HERE_T = HERE

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


def snapshot_tree(root: Path) -> dict:
    """Complete inventory of a real artefact tree, used to prove that the test never creates, removes
    or alters anything real — whatever the root legitimately contains (P0/T1 outputs of authorised S1
    stages).  Keys are POSIX paths relative to ``root``: every directory (including EMPTY ones, so a
    stray ``ensure_out_dirs`` aimed at the real root is detected) -> ("dir",); every regular file ->
    (size, SHA-256); every symlink -> ("symlink", target) without following it; the root itself ->
    "." : ("dir",).  Empty mapping when the root does not exist (distinct from an existing empty root)."""
    import hashlib
    if not root.exists():
        return {}
    out = {".": ("dir",)}
    for p in sorted(root.rglob("*")):
        rel = p.relative_to(root).as_posix()
        if p.is_symlink():
            out[rel] = ("symlink", os.readlink(p))
        elif p.is_dir():
            out[rel] = ("dir",)
        elif p.is_file():
            out[rel] = (p.stat().st_size, hashlib.sha256(p.read_bytes()).hexdigest())
    return out


def main() -> int:
    tmp = R.portable_tempdir("f2r_matrix_")
    # redirect every output dir of f2r_common into the temp root (the real OUT_ROOT is untouched)
    real_out_root = R.OUT_ROOT
    real_snapshot = snapshot_tree(real_out_root)  # pre-test inventory of the REAL root (may be populated by S1 stages)
    R.OUT_ROOT = tmp / "out"
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(real_out_root))
    # --- snapshot helper self-test: directories (empty too), files, symlinks, root sentinel
    st_root = tmp / "snap"; (st_root / "a" / "empty").mkdir(parents=True); (st_root / "a" / "f.txt").write_text("x")
    try:
        os.symlink(st_root / "a" / "f.txt", st_root / "link"); has_link = True
    except (OSError, NotImplementedError):
        has_link = False
    snap = snapshot_tree(st_root)
    check(snap["."] == ("dir",) and snap["a"] == ("dir",) and snap["a/empty"] == ("dir",) and snap["a/f.txt"][0] == 1 and len(snap["a/f.txt"][1]) == 64 and (not has_link or snap["link"][0] == "symlink") and snapshot_tree(tmp / "missing") == {}, "snapshot_tree lists dirs (empty included), files with size+SHA, symlinks unfollowed, root sentinel; missing root -> {}")
    (st_root / "b").mkdir()
    check(snapshot_tree(st_root) != snap and set(snapshot_tree(st_root)) - set(snap) == {"b"}, "snapshot_tree detects the creation of an empty directory")
    (st_root / "b").rmdir(); check(snapshot_tree(st_root) == snap, "snapshot_tree back to the original after removing it")
    # --- specs
    jobs = MX.build_jobs()
    rounds = MX.phase_rounds()
    check(rounds == {"T1": 1, "T2": 4, "T3": 2, "T1R": 1}, f"rounds from protocol: {rounds}")
    n_legacy = 3 + (4 + 2) * (3 + 6 + 3)
    n_expected = n_legacy + 3  # + T1R corrective commissioning (3 det)
    check(len(jobs) == n_expected and rounds["T1R"] == 1, f"job count {len(jobs)} == {n_expected} (75 legacy + 3 T1R)")
    # --- non-regression of the 75 legacy specs and commands (fingerprints pinned BEFORE the T1R change)
    import hashlib as _hl
    legacy = jobs[:n_legacy]
    check(all(j["phase"] != "T1R" for j in legacy) and _hl.sha256(json.dumps(legacy, sort_keys=True, default=str).encode()).hexdigest() == "86e27ef8ed2080f52720539cee36a0dca2d33f41085992e93206783438169c71", "the 75 legacy job specs are byte-identical to the pre-T1R fingerprint (same ids, seeds, purposes, stages, meaning)")
    norm_cmds = [[tok.replace(str(R.OUT_ROOT), "<OUT_ROOT>") for tok in MX.job_command(j, MX.output_dir_for(j), "PY")] for j in legacy]
    check(_hl.sha256(json.dumps(norm_cmds, default=str).encode()).hexdigest() == "72bc9c57af8813f16149481ab31ce34bf179cc376982e6be2130607323af4438", "the 75 legacy commands (root-normalised) are identical to the pre-T1R fingerprint")
    t1r = jobs[n_legacy:]
    check([j["job_id"] for j in t1r] == [f"STUDENT_T1R_r1__v3_canonical__{s}__det" for s in R.STARTS] and all(j["purpose"] == "det" and j["seed"] == 123 and j["phase"] == "T1R" and j["round"] == 1 and j["candidate"] == "STUDENT_T1R_r1" and j["stage"] == "T1R_round_1" and j["enters_dataset"] is False and j["generates_labels"] is False and j["gate_role"] == "commissioning gate measurement A/B/C only; no aggregation" for j in t1r), "T1R: exactly 3 det seed-123 GATE-ONLY jobs (enters_dataset false, generates_labels false) appended after the legacy ones")
    check(MX.student_module_dir("T1R", 1) == R.OUT_REFIT / "T1R" / "round_1" / "rl_module_student" and all(str(MX.student_module_dir("T1R", 1)) in MX.job_command(j, MX.output_dir_for(j), "PY") for j in t1r), "T1R jobs bind the T1R student module path")
    expect(lambda: MX._spec("T1R", 1, "stoch", "nominal", 124), MX.MatrixError, "T1R never has stochastic jobs (spec refused by the matrix contract)")
    expect(lambda: MX._spec("T1R", 1, "validation_gate", "nominal", 125), MX.MatrixError, "T1R never has seed-125 validation jobs")
    check(all(j["seed"] not in R.SEALED_SEEDS for j in jobs), "no sealed seed in the matrix")
    check(all(j["seed"] == 125 for j in jobs if j["purpose"] == "validation_gate") and all(j["seed"] != 125 for j in jobs if j["purpose"] != "validation_gate"), "seed 125 only and always in validation_gate jobs")
    check(all(not j["enters_dataset"] for j in jobs if j["purpose"] == "validation_gate") and all(j["enters_dataset"] for j in jobs if j["purpose"] != "validation_gate" and j["phase"] != "T1R") and all(not j["enters_dataset"] for j in jobs if j["phase"] == "T1R"), "validation gate jobs never enter a dataset; legacy det/stoch jobs do; T1R gate-only jobs never do")
    check(all(j["seed"] == 123 for j in jobs if j["purpose"] == "det"), "det jobs use seed 123")
    t1 = [j for j in jobs if j["phase"] == "T1"]
    check(len(t1) == 3 and all(j["purpose"] == "det" for j in t1), "T1 commissioning: 3 det only")
    check(all(j["runtime"] == "v3_canonical" and j["driver"] == "rollout_eval" for j in jobs), "runtime/driver")
    expect(lambda: MX._spec("T2", 1, "stoch", "nominal", 125), R.F2RContractError, "stoch with seed 125 refused")
    expect(lambda: MX._spec("T2", 1, "validation_gate", "nominal", 124), R.F2RContractError, "validation gate with seed 124 refused")
    expect(lambda: MX._spec("T2", 1, "det", "nominal", 126), R.F2RContractError, "sealed seed refused")
    expect(lambda: MX._spec("T2", 1, "det", "nominal", 124), MX.MatrixError, "det with seed 124 refused")
    # --- describe (module pending, receipt closure fields)
    py = "/usr/bin/env-python-fake"
    recs = [MX.describe_job(j, py) for j in jobs]
    check(all(r["module_pending"] and r["module_state_sha256"] is None and r["job_inputs"] is None for r in recs), "all student modules pending in S0")
    check(all(r["schema_version"] == "f2r.1" and r["protocol_id"] == R.PROTOCOL_ID for r in recs), "receipt schema/protocol id")
    check(all(r["command"][0] == py and r["command"][1] == str(R.ROLLOUT_EVAL) and r["command"].count("--no-auto-config") == 1 for r in recs), "command shape")
    check(all("--record-policy-trace" in r["command"] and r["command"][r["command"].index("--config") + 1] == str(R.RUNTIME_CONFIG) and "--binary-phase-actor-fsm-version" not in r["command"] for r in recs), "v3_canonical: pinned resolved yaml (FSM v3 inside), no extra flags (as F0/F1)")
    check(all(r["command"][r["command"].index("--seed") + 1] == str(r["seed"]) and r["command"][r["command"].index("--action-selection") + 1] == r["action_selection"] for r in recs), "seed/selection bound in command")
    check(all(r["init_primary"]["actor_digest"] == R.INIT_PRIMARY["actor_digest"] for r in recs), "init primary pinned in receipts")
    check(all(MX.command_binding_problems(r) == {} for r in recs), "command binding clean for all described jobs")
    rec = dict(recs[0])
    bad = dict(rec); bad["seed"] = 125
    check("seed" in MX.command_binding_problems(bad), "binding detects seed drift")
    bad = dict(rec); bad["command"] = rec["command"][:-1]
    check("command_argv" in MX.command_binding_problems(bad), "binding detects command drift")
    bad = dict(rec); bad["output_dir"] = "elsewhere"
    check("output_dir" in MX.command_binding_problems(bad), "binding detects output dir drift")
    # closure fields used by F0 verify_closure_fields must exist on a completed receipt shape (verified via the F0 validator on a synthetic failure receipt)
    # --- execution refusals
    expect(lambda: MX.run_job(dict(rec), log_path=tmp / "log.jsonl"), MX.MatrixError, "run_job refuses pending student module")
    bad = dict(rec); bad["seed"] = 127
    expect(lambda: MX.run_job(bad, log_path=tmp / "log.jsonl"), MX.MatrixError, "run_job refuses sealed seed before anything else")
    check(not (tmp / "log.jsonl").exists() and not R.OUT_ROLLOUTS.exists(), "refusals create no output")
    # --- dry-run manifest (temp root), no-clobber
    idx = MX.write_dry_run_manifest(recs, stamp="TEST1", python_record={"selected": py})
    man = C.read_json(C.REPO / idx["manifest"])
    check(man["dry_run"] and man["stage"] == "S0" and man["p0_executed"] is False and man["rollouts_executed"] == 0 and man["refit_executed"] is False, "manifest S0 flags")
    check(man["actors"]["all_match"] and man["anchors"]["all_match"] and man["p0_jul_jobs"]["all_match"] and man["job_count"] == n_expected and not man["seeds"]["sealed_seed_found"], "manifest pins (actors, anchors, P0 JUL_H0 jobs) and counts")
    check(all(man["p0_jul_jobs"][s]["files"][k]["sha256"] == R.P0_JUL_PINS[s][f"{k}_sha256"] for s in R.STARTS for k in R.P0_JUL_FILES) and man["p0_jul_jobs"]["f0_analysis"]["ok"], "manifest records the verified P0 input digests (4 files x 3 starts) and the F0 analysis chain")
    check(man["immutability"]["f0_analysis"]["ok"] and man["immutability"]["f1_scripts_pins_vs_r3_manifest"]["ok"] is True, "F0 analysis and F1 r3 pins verified in manifest")
    check(len(man["existing_output_dirs"]) == 0 and len(man["jobs_pending_module"]) == n_expected, "no existing outputs; all modules pending")
    check(all(isinstance(c, list) and all(isinstance(t, str) and "<" not in t for t in c) for cmds in man["pipeline_commands"].values() for c in cmds), "pipeline commands exact (no placeholders)")
    check("P0_offline_NOT_EXECUTED_IN_S0" in man["pipeline_commands"], "P0 command listed but marked not executed")
    check(sorted(p.name for p in R.OUT_ROOT.iterdir()) == ["manifest"] and sorted(p.name for p in R.OUT_MANIFEST.iterdir()) == ["f2r_s0_dry_run_TEST1.index.json", "f2r_s0_dry_run_TEST1.json", "f2r_s0_dry_run_TEST1.md"], "dry-run content: OUT_ROOT holds only manifest/ with exactly the 3 artefacts")
    check(idx["materialised_files"] == ["f2r_s0_dry_run_TEST1.json", "f2r_s0_dry_run_TEST1.md", "f2r_s0_dry_run_TEST1.index.json"] and man["previous_manifests_superseded_by_this"] == [], "index lists the 3 materialised files; first manifest supersedes nothing")
    on_disk_index = json.loads((R.OUT_MANIFEST / "f2r_s0_dry_run_TEST1.index.json").read_text(encoding="utf-8"))
    check(on_disk_index == idx and on_disk_index["materialised_files"] == ["f2r_s0_dry_run_TEST1.json", "f2r_s0_dry_run_TEST1.md", "f2r_s0_dry_run_TEST1.index.json"], "index re-read from disk == returned index, with exactly the three names (built before the atomic fill)")
    check(all((R.OUT_MANIFEST / n).is_file() for n in on_disk_index["materialised_files"]) and on_disk_index["manifest_sha256"] == C.sha256_file(R.OUT_MANIFEST / "f2r_s0_dry_run_TEST1.json"), "the three named files exist and the index SHA matches the manifest on disk")
    idx_b = MX.write_dry_run_manifest(recs, stamp="TEST1", python_record={"selected": py})
    check(idx_b["materialised_files"] == ["f2r_s0_dry_run_TEST1_01.json", "f2r_s0_dry_run_TEST1_01.md", "f2r_s0_dry_run_TEST1_01.index.json"], "same stamp again -> _01 suffix (exclusive reservation; nothing reused/overwritten)")
    man_b = C.read_json(C.REPO / idx_b["manifest"])
    check(man_b["previous_manifests_superseded_by_this"] == ["f2r_s0_dry_run_TEST1.index.json"] and json.loads((R.OUT_MANIFEST / "f2r_s0_dry_run_TEST1.json").read_text())["stamp"] == "TEST1", "second manifest records the first as superseded; first untouched")
    check(sorted(p.name for p in R.OUT_ROOT.iterdir()) == ["manifest"] and len(list(R.OUT_MANIFEST.iterdir())) == 6 and not list(R.OUT_MANIFEST.glob("*.part-*")), "after two dry-runs: still only manifest/, 6 files, no temp parts")
    check(man["pipeline_commands"]["S1_privileged_cache_build_env_zero_steps"] == [[py, str(HERE / "f2r_labeller.py"), "build-cache", "--with-ik", "--authorized-stage", "S1", "--out-dir", str(R.OUT_CACHE)]], "S1 cache command is exactly build-cache --with-ik --authorized-stage S1")
    check(all(cmd[-2:] != ["--authorized-stage", "S0"] and "--authorized-stage" in cmd and cmd[cmd.index("--authorized-stage") + 1] == "S1" for cmds in man["pipeline_commands"].values() for cmd in cmds), "every pipeline command carries --authorized-stage S1")
    check(man["pipeline_commands"]["T1R_corrective_commissioning"] == [[py, str(HERE / "f2r_dagger.py"), "refit-t1r", "--authorized-stage", "S1", "--out-dir", str(R.OUT_REFIT / "T1R" / "round_1")], [py, str(HERE / "f2r_matrix.py"), "--execute", "--phase", "T1R", "--round", "1", "--workers", "2", "--authorized-stage", "S1"], [py, str(HERE / "f2r_gates.py"), "--phase", "T1R", "--round", "1", "--authorized-stage", "S1", "--out-dir", str(R.OUT_GATE)]], "T1R pipeline commands exact (refit-t1r -> 3 det rollouts -> gate)")
    check(man["actors"]["init_primary"]["digests_distinct"] and man["actors"]["digest_semantics"]["actor_digest"].startswith("warm_start.actor_state_digest") and man["actors"]["init_primary"]["actor_digest"]["pinned"] != man["actors"]["init_primary"]["module_state_sha256"]["pinned"], "manifest distinguishes actor digest (parameters) from module_state.pkl SHA-256 (file)")
    check(man["stage"] == "S0" and man["p0_executed"] is False and man["refit_executed"] is False and man["rollouts_executed"] == 0 and man["dry_run_launches_jobs"] is False and idx_b["materialisation_audit"]["added"] == sorted(f"manifest/{n}" for n in idx_b["materialised_files"]), "empty root: stage S0, nothing executed; audit lists exactly the 3 added files")
    # populated root (as after authorised S1 stages): the dry-run must add ONLY its three files and report the detected state truthfully
    pop = tmp / "populated"; R.OUT_ROOT = pop
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(tmp / "out"))
    (R.OUT_P0).mkdir(parents=True); (R.OUT_P0 / "p0_result_FAKE.json").write_text("{}"); (R.OUT_ROLLOUTS / "T1" / "round_1" / "empty_dir").mkdir(parents=True); (R.OUT_GATE).mkdir(); (R.OUT_GATE / "gate_T1_round_1_FAKE.json").write_text("{}")
    pre_inv = MX.output_root_inventory(R.OUT_ROOT)
    idx_p = MX.write_dry_run_manifest(recs, stamp="POP", python_record={"selected": py})
    post_inv = MX.output_root_inventory(R.OUT_ROOT)
    check(set(post_inv) - set(pre_inv) == {"manifest", "manifest/f2r_s0_dry_run_POP.json", "manifest/f2r_s0_dry_run_POP.md", "manifest/f2r_s0_dry_run_POP.index.json"} and all(post_inv[k] == pre_inv[k] for k in pre_inv), "populated root: exactly manifest/ + 3 files added, every pre-existing entry (dirs incl. empty, files) unchanged")
    man_p = C.read_json(C.REPO / idx_p["manifest"])
    check(man_p["stage"] == "S1" and man_p["p0_executed"] is True and man_p["refit_executed"] is False and man_p["rollouts_executed"] == 0 and man_p["executed_artefacts_detected"]["gate_results"] == [C.rel(R.OUT_GATE / "gate_T1_round_1_FAKE.json")], "populated root: detected state reported truthfully (stage S1, p0 executed, gate listed, no ok rollout)")
    md_p = (C.REPO / idx_p["markdown"]).read_text(encoding="utf-8")
    check("detected stage S1" in md_p.splitlines()[0] and "all pending" not in md_p and "P0 results 1" in md_p and "gates 1" in md_p, "populated root: markdown heading reports detected stage S1 and the executed artefacts")
    import io as _io, contextlib as _ctx
    buf_p = _io.StringIO()
    with _ctx.redirect_stdout(buf_p):
        rc_p = MX.main([])
    out_p = json.loads(buf_p.getvalue())
    check(rc_p == 0 and out_p["stage"] == "S1" and out_p["mode"] == "dry_run", "populated root: CLI stdout reports the detected stage S1 (not S0)")
    # same CLI on the empty temp root must say S0
    empty_root = tmp / "empty_root"; R.OUT_ROOT = empty_root
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(pop))
    buf_e = _io.StringIO()
    with _ctx.redirect_stdout(buf_e):
        rc_e = MX.main([])
    check(rc_e == 0 and json.loads(buf_e.getvalue())["stage"] == "S0" and sorted(p.name for p in empty_root.iterdir()) == ["manifest"], "empty root: CLI stdout reports stage S0 and only manifest/ is materialised")
    R.OUT_ROOT = pop
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(empty_root))
    # a foreign write during the dry-run (simulated by a patched filler) must make the audit fail closed
    real_fill = R._atomic_fill
    def leaky_fill(path, data):
        (R.OUT_MANIFEST / "stray.txt").write_text("x")
        return real_fill(path, data)
    R._atomic_fill = leaky_fill
    try:
        expect(lambda: MX.write_dry_run_manifest(recs, stamp="LEAK", python_record={"selected": py}), MX.MatrixError, "foreign file appearing during the dry-run -> materialisation audit fails closed")
    finally:
        R._atomic_fill = real_fill
    R.OUT_ROOT = tmp / "out"
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(pop))
    # structural violation detection in manifest writer
    tampered = [dict(r) for r in recs]; tampered[5]["seed"] = 126
    expect(lambda: MX.write_dry_run_manifest(tampered, stamp="TEST2", python_record={"selected": py}), MX.MatrixError, "manifest writer refuses a sealed seed")
    tampered = [dict(r) for r in recs]; v = next(r for r in tampered if r["purpose"] == "validation_gate"); v["enters_dataset"] = True
    expect(lambda: MX.write_dry_run_manifest(tampered, stamp="TEST3", python_record={"selected": py}), MX.MatrixError, "manifest writer refuses validation job entering a dataset")
    check(not R.OUT_ROLLOUTS.exists() and not R.OUT_REFIT.exists() and not R.OUT_P0.exists(), "dry-run materialises only manifest/ (no rollouts/refit/p0 dirs)")
    # --- other S1 CLIs refuse in S0 (no env/fit/aggregation/rollout)
    import f2r_dagger as DG, f2r_gates as GT, f2r_labeller as LB, f2r_observability as OB
    expect(lambda: DG.main(["refit", "--variant", "T2", "--round", "1", "--out-dir", str(tmp / "x")]), SystemExit, "dagger refit refused in S0")
    expect(lambda: DG.main(["aggregate", "--variant", "T2", "--round", "1", "--out-dir", str(tmp / "x")]), SystemExit, "dagger aggregate refused in S0")
    expect(lambda: GT.main(["--phase", "T2", "--round", "1", "--out-dir", str(tmp / "x")]), SystemExit, "gates evaluation refused in S0")
    expect(lambda: LB.main(["build-cache", "--with-ik", "--out-dir", str(tmp / "x")]), SystemExit, "cache build with env refused in S0")
    expect(lambda: OB.main(["--real", "--out-dir", str(tmp / "x")]), SystemExit, "P0 real refused in S0")
    check(not (tmp / "x").exists(), "refused CLIs created nothing")
    # --- CLI dry-run in the temp root: no interpreter probe, only read-only git subprocesses, only manifest/ materialised
    import subprocess
    spied: list[list[str]] = []
    real_popen = subprocess.Popen

    class SpyPopen(real_popen):  # type: ignore[misc]
        def __init__(self, args, *a, **kw):
            spied.append([str(x) for x in (args if isinstance(args, (list, tuple)) else [args])])
            super().__init__(args, *a, **kw)

    subprocess.Popen = SpyPopen
    try:
        before = sorted(p.name for p in R.OUT_MANIFEST.iterdir())
        rc = MX.main([])
    finally:
        subprocess.Popen = real_popen
    after = sorted(p.name for p in R.OUT_MANIFEST.iterdir())
    new_files = sorted(set(after) - set(before))
    check(rc == 0 and len(new_files) == 3 and sorted(p.name for p in R.OUT_ROOT.iterdir()) == ["manifest"], "CLI dry-run: exactly 3 new files under manifest/, nothing else under the output root")
    check(spied and all(argv[0] == "git" for argv in spied), f"CLI dry-run spawned only git metadata queries ({len(spied)} calls), no interpreter probe, no job")
    cli_man = C.read_json(R.OUT_MANIFEST / [n for n in new_files if n.endswith(".json") and not n.endswith(".index.json")][0])
    check(cli_man["interpreter"]["selected"] == sys.executable and cli_man["interpreter"]["source"].startswith("sys.executable") and all(r["python"] == sys.executable for r in cli_man["jobs"]), "dry-run interpreter = sys.executable of the driver (no probe), bound in every job command")
    check(cli_man["materialisation_policy"].startswith("S0 dry-run writes exactly these three files") and cli_man["process_policy"].startswith("dry-run launches no job"), "materialisation/process policies recorded")
    cli_md = (R.OUT_MANIFEST / [n for n in new_files if n.endswith(".md")][0]).read_text(encoding="utf-8")
    check(cli_md.splitlines()[0].startswith("# F2R dry-run") and "detected stage S0" in cli_md.splitlines()[0] and "all pending" not in cli_md and f"student module pending for {len(cli_man['jobs'])}" in cli_md and "materialised outputs for 0" in cli_md, "empty root: markdown heading reports detected stage S0 and truthful pending/materialised counts")
    # --- CLI: --execute refused in S0
    expect(lambda: MX.main(["--execute", "--phase", "T1", "--round", "1"]), SystemExit, "--execute refused in S0")
    expect(lambda: MX.main(["--execute", "--phase", "T1", "--round", "1", "--authorized-stage", "S1"]), SystemExit, "--execute with the S1 flag still refuses while the student module is pending (no launch)")
    check(not R.OUT_ROLLOUTS.exists() and not R.OUT_REFIT.exists(), "refused --execute created no rollout/refit dir")
    # --- run_job end-to-end with a fake subprocess (temp root; synthetic student module = copy of JUL_H0)
    import shutil
    from types import SimpleNamespace
    real_py = C.select_python()["selected"]
    student = MX.student_module_dir("T1", 1)
    shutil.copytree(Path(R.INIT_PRIMARY["module"]), student)
    t1 = [j for j in MX.build_jobs() if j["phase"] == "T1"]
    recs_t1 = [MX.describe_job(j, real_py) for j in t1]
    check(all(not r["module_pending"] and r["job_inputs_digest"] for r in recs_t1), "T1 jobs described with a materialised student module (job inputs bound)")
    orch = tuple(str(R.F0_DIR / p) for p in CL.ORCHESTRATION_FILES) + tuple(str(HERE / p) for p in MX.F2R_SCRIPTS if (HERE / p).is_file())
    snapshot_fn = lambda: CL.closure_snapshot(real_py, orchestration_files=orch)  # noqa: E731
    snapshot = snapshot_fn()
    bindings = {r["job_id"]: r["job_inputs_digest"] for r in recs_t1}
    R.ensure_out_dirs()
    written = CL.write_closure_manifest(R.OUT_ROLLOUTS / "source_closure_manifest_TEST.json", snapshot, job_inputs=bindings)
    closure = {**written, "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"], "launch_snapshot": snapshot, "job_inputs": bindings, "snapshot_fn": snapshot_fn}
    marker: dict = {}

    def fake_factory(fail: bool = False):
        def fake_run(command, cwd=None, stdout=None, stderr=None, text=None, check=None):
            marker["launched"] = marker.get("launched", 0) + 1
            out = Path(command[command.index("--output-dir") + 1]); out.mkdir(parents=True, exist_ok=True)
            if not fail:
                (out / "rollout_summary.json").write_text(json.dumps({"ok": True, "steps": 5, "episode_return": 1.0, "end_reason": "episode_time_limit", "n_actor": 35, "action_seed": int(command[command.index("--seed") + 1]), "action_selection": command[command.index("--action-selection") + 1], "episode_start_offset_s": float(command[command.index("--episode-start-offset-s") + 1])}), encoding="utf-8")
                (out / "rollout_policy_trace.json").write_text(json.dumps([{"step": k + 1, "time": 12.0 + 0.01 * k} for k in range(5)]), encoding="utf-8")
            if stdout is not None:
                stdout.write("fake rollout\n")
            return SimpleNamespace(returncode=1 if fail else 0)
        return fake_run

    log = R.OUT_ROLLOUTS / "driver_log.jsonl"
    r0 = recs_t1[0]
    res = MX.run_job(dict(r0), log_path=log, closure=closure, subprocess_run=fake_factory())
    check(res["status"] == "ok" and res["returncode"] == 0 and marker["launched"] == 1 and res["summary_ok"] and res["closure_manifest_unchanged"] and res["job_inputs_unchanged"], "T1 det job ok with the fake rollout (closure pre/post equal)")
    check(CL.verify_closure_fields(res) == {}, "f2r.1 receipt verifies with the unchanged F0 closure validator")
    out0 = C.REPO / r0["output_dir"]
    check((out0 / MX.RECEIPT_FILE).is_file() and (out0 / MX.STDOUT_LOG).is_file() and json.loads((out0 / MX.RECEIPT_FILE).read_text())["command"][0] == real_py, "receipt + stdout log written; interpreter bound")
    res2 = MX.run_job(dict(r0), log_path=log, closure=closure, subprocess_run=fake_factory())
    check(res2["status"] == "skipped_verified_identical" and marker["launched"] == 1, "second run skipped after verification, no relaunch")
    (out0 / "rollout_policy_trace.json").write_text((out0 / "rollout_policy_trace.json").read_text() + " ", encoding="utf-8")
    expect(lambda: MX.preflight_all([r0]), MX.MatrixError, "tampered trace refused by preflight")
    expect(lambda: MX.run_job(dict(r0), log_path=log, closure=closure, subprocess_run=fake_factory()), MX.MatrixError, "run_job refuses a tampered existing output (no relaunch)")
    r1 = recs_t1[1]
    res3 = MX.run_job(dict(r1), log_path=log, closure=closure, subprocess_run=fake_factory(fail=True))
    check(res3["status"] == "failed" and res3["returncode"] == 1 and marker["launched"] == 2, "rc 1 => failed")
    mut = dict(recs_t1[2]); mut["seed"] = 124
    expect(lambda: MX.run_job(mut, log_path=log, closure=closure, subprocess_run=fake_factory()), MX.MatrixError, "mutated receipt (seed) refused before launch")
    check(marker["launched"] == 2, "exactly two fake launches overall")
    # ================= amendment "preflight scoping" (architect decision 2026-08-23) =================
    # A valid legacy output produced under a PREVIOUS tooling revision / git head must not block the launch of
    # another selection; the same output IS refused when it is itself selected (no silent reuse); archival
    # integrity problems are refused for every output, selected or not.
    r_ok = recs_t1[2]  # T1 plus020: no existing output yet in this temp root -> create a valid one with the fake runner
    res_ok = MX.run_job(dict(r_ok), log_path=log, closure=closure, subprocess_run=fake_factory())
    check(res_ok["status"] == "ok" and marker["launched"] == 3, "a third valid legacy output created (fake run)")
    out_ok = C.REPO / r_ok["output_dir"]; rpath = out_ok / MX.RECEIPT_FILE
    hist = json.loads(rpath.read_text(encoding="utf-8"))
    hist["f2r_driver_inputs_digest"] = "1d7d202b" + "0" * 56; hist["git_head"] = "0123456789abcdef0123456789abcdef01234567"  # historical revision / head
    rpath.write_text(json.dumps(hist, indent=2), encoding="utf-8")  # simulates a receipt written by a previous tooling revision (synthetic only)
    t1r_ids = {j["job_id"] for j in MX.build_jobs() if j["phase"] == "T1R"}
    described_tmp = [r for r in recs_t1 if r["job_id"] == r_ok["job_id"]] + [MX.describe_job(j, real_py) for j in MX.build_jobs() if j["phase"] == "T1R"]
    pf = MX.preflight_all(described_tmp, selected_ids=t1r_ids)
    check(pf["invalid"] == [] and pf["verified_identical"] == [] and pf["verified_integrity_only"] == [r_ok["job_id"]] and len(pf["tooling_revision_differs"]) == 1 and pf["tooling_revision_differs"][0]["job_id"] == r_ok["job_id"] and pf["tooling_revision_differs"][0]["integrity"] == "verified" and set(pf["tooling_revision_differs"][0]) >= {"f2r_driver_inputs_digest", "git_head"}, "legacy output valid under a historical driver digest + git head, T1R selected -> preflight PASS with an explicit informational revision mismatch")
    check(json.loads(rpath.read_text(encoding="utf-8")) == hist, "the legacy receipt is untouched by the preflight")
    expect(lambda: MX.preflight_all(described_tmp, selected_ids={r_ok["job_id"]}), MX.MatrixError, "the same legacy output, when SELECTED, is refused (stale driver digest / git head: no silent reuse)")
    expect(lambda: MX.preflight_all(described_tmp), MX.MatrixError, "selected_ids=None keeps the strict legacy behaviour (every output a relaunch candidate)")
    never = lambda *a, **k: (_ for _ in ()).throw(AssertionError("runner must not be called when the preflight fails"))  # noqa: E731
    expect(lambda: MX.execute(described_tmp, [r for r in described_tmp if r["job_id"] == r_ok["job_id"]], workers=1, log_path=log, closure=closure, runner=never), MX.MatrixError, "execute with the stale legacy job selected fails BEFORE the runner")
    expect(lambda: MX.run_job(dict(r_ok), log_path=log, closure=closure, subprocess_run=fake_factory()), MX.MatrixError, "run_job on the stale legacy output is refused before launch (verify_existing strict for relaunch candidates)")
    check(marker["launched"] == 3, "no relaunch happened")
    # archival-integrity problems of a NON-selected output are still refused, each in isolation
    import copy
    def with_receipt(mut):
        h2 = copy.deepcopy(hist); mut(h2); rpath.write_text(json.dumps(h2, indent=2), encoding="utf-8")
    trace = out_ok / "rollout_policy_trace.json"; trace_bytes = trace.read_bytes()
    trace.write_bytes(trace_bytes + b" ")
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output with a tampered trace -> FAIL")
    trace.write_bytes(trace_bytes)
    with_receipt(lambda h: h.__setitem__("job_inputs_digest_pre", "f" * 64))
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output with a wrong job_inputs digest -> FAIL")
    with_receipt(lambda h: h.__setitem__("closure_manifest_sha256", "e" * 64))
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output with an invalid F0 closure field -> FAIL")
    with_receipt(lambda h: h.__setitem__("command", h["command"][:-1]))
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output with a broken command binding -> FAIL")
    with_receipt(lambda h: h.__setitem__("status", "failed"))
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output with status != ok -> FAIL")
    with_receipt(lambda h: h.__setitem__("summary_sha256", "d" * 64))
    expect(lambda: MX.preflight_all(described_tmp, selected_ids=t1r_ids), MX.MatrixError, "non-selected legacy output whose stored summary digest differs from the disk -> FAIL")
    rpath.write_text(json.dumps(hist, indent=2), encoding="utf-8")
    check(MX.preflight_all(described_tmp, selected_ids=t1r_ids)["invalid"] == [], "restored legacy receipt -> PASS again")
    # selected collision: an existing output dir of a SELECTED T1R job without a receipt -> FAIL before the runner
    t1r_out = C.REPO / described_tmp[1]["output_dir"]; t1r_out.mkdir(parents=True)
    expect(lambda: MX.execute(described_tmp, [described_tmp[1]], workers=1, log_path=log, closure=closure, runner=never), MX.MatrixError, "selected T1R job with an invalid existing output (collision without receipt) -> FAIL before the runner")
    t1r_out.rmdir()
    # the runner / closure / post-check path of the selected jobs is unchanged: a fresh selected job still runs the full closure-bound run_job
    st_t1r = MX.student_module_dir("T1R", 1); shutil.copytree(Path(R.INIT_PRIMARY["module"]), st_t1r)
    d_t1r = [MX.describe_job(j, real_py) for j in MX.build_jobs() if j["phase"] == "T1R"]
    closure_t1r = {**closure, "job_inputs": {**bindings, **{d["job_id"]: d["job_inputs_digest"] for d in d_t1r}}}
    closure_t1r.update(CL.write_closure_manifest(R.OUT_ROLLOUTS / "source_closure_manifest_T1R_TEST.json", snapshot, job_inputs=closure_t1r["job_inputs"]))
    res_t1r = MX.run_job(dict(d_t1r[0]), log_path=log, closure=closure_t1r, subprocess_run=fake_factory())
    check(res_t1r["status"] == "ok" and res_t1r["closure_matches_launch_manifest_pre"] and res_t1r["source_closure_unchanged"] and res_t1r["job_inputs_unchanged"] and res_t1r["closure_manifest_unchanged"] and CL.verify_closure_fields(res_t1r) == {} and res_t1r["enters_dataset"] is False and res_t1r["purpose"] == "det" and marker["launched"] == 4, "selected T1R job: full closure-bound run_job (pre/post closure, job inputs, manifest) unchanged and fail-closed; receipt gate-only")
    # executed_artefacts lists launch sets, including an aborted one (transparency of orchestration attempts)
    (R.OUT_ROLLOUTS / "f2r_job_matrix_status_ABORT.json").write_text(json.dumps({"tag": "ABORT", "phase": "T1R", "round": 1, "status": "aborted", "error": "synthetic", "job_count": 3}), encoding="utf-8")
    ls = MX.executed_artefacts()["launch_sets"]
    check(any(x["tag"] == "ABORT" and x["status"] == "aborted" for x in ls), "executed_artefacts lists launch sets with their status (aborted attempt visible)")
    check(snapshot_tree(real_out_root) == real_snapshot, f"real output root byte-identical to the pre-test snapshot after the fake launches ({len(real_snapshot)} real files untouched)")
    # --- launch set reserved as a whole before any job (synthetic: no subprocess, no env, no rollout)
    import functools, hashlib
    from types import SimpleNamespace as _NS
    fp = {"role": "rollout_interpreter", "requested_executable": real_py, "executable_realpath": os.path.realpath(real_py), "fingerprint_sha256": hashlib.sha256(b"synthetic-fingerprint").hexdigest(), "python_version": "synthetic", "modules": {}}

    def synthetic_snapshot() -> dict:
        runtime = CL.digest_table([C.REPO / p for p in CL.RUNTIME_CORE_FILES], label="runtime core")
        plugins = CL.native_plugin_table(CL.NATIVE_PLUGIN_DIR)
        assets = CL.digest_table(list(CL.DATA_ASSET_FILES), label="data assets")
        orch_t = CL.digest_table([Path(p) for p in orch], label="orchestration")
        return {"closure_schema_version": CL.CLOSURE_SCHEMA_VERSION, "computed_at_utc": "synthetic", "runtime_source_closure_digest": CL.table_digest(runtime, plugins, assets), "runtime_core": runtime, "native_plugins": plugins, "data_assets": assets, "orchestration_digest": CL.table_digest(orch_t), "orchestration": orch_t, "environment_fingerprint": fp, "git": {"head": "synthetic"}, "closure_declared_not_import_traced": True, "provenance_limitation": CL.PROVENANCE_LIMITATION, "irrecoverable_limitations": list(CL.IRRECOVERABLE_LIMITATIONS)}

    launch_dir = tmp / "launch_root"
    R.OUT_ROOT = launch_dir
    for name in ("OUT_MANIFEST", "OUT_ROLLOUTS", "OUT_DATASETS", "OUT_CACHE", "OUT_REFIT", "OUT_P0", "OUT_METRICS", "OUT_GATE", "OUT_LOGS"):
        setattr(R, name, R.OUT_ROOT / getattr(R, name).relative_to(tmp / "out"))
    shutil.copytree(Path(R.INIT_PRIMARY["module"]), MX.student_module_dir("T1", 1))
    recs_l = [MX.describe_job(j, real_py) for j in t1]  # git calls happen here, before the spied launch
    spied2: list = []

    class SpyPopen2(real_popen):  # type: ignore[misc]
        def __init__(self, args, *a, **kw):
            spied2.append(args)
            super().__init__(args, *a, **kw)

    fake_git = lambda: {"head": "synthetic", "dirty": []}  # noqa: E731
    ok_runner = functools.partial(MX.run_job, subprocess_run=fake_factory())
    subprocess.Popen = SpyPopen2
    try:
        res1 = MX.launch_round(recs_l, recs_l, phase="T1", round_index=1, stamp="STAMP", python_record={"selected": real_py}, workers=1, snapshot_fn=synthetic_snapshot, runner=ok_runner, git_snapshot_fn=fake_git)
        res2 = MX.launch_round(recs_l, recs_l, phase="T1", round_index=1, stamp="STAMP", python_record={"selected": real_py}, workers=1, snapshot_fn=synthetic_snapshot, runner=ok_runner, git_snapshot_fn=fake_git)
        (R.OUT_ROLLOUTS / "f2r_job_matrix_status_STAMP_02.json").write_text("{}", encoding="utf-8")  # partial collision on one name only
        res3 = MX.launch_round(recs_l, recs_l, phase="T1", round_index=1, stamp="STAMP", python_record={"selected": real_py}, workers=1, snapshot_fn=synthetic_snapshot, runner=ok_runner, git_snapshot_fn=fake_git)

        def boom(*a, **k):
            raise RuntimeError("synthetic execution failure")

        try:
            MX.launch_round(recs_l, recs_l, phase="T1", round_index=1, stamp="STAMP", python_record={"selected": real_py}, workers=1, snapshot_fn=synthetic_snapshot, runner=boom, git_snapshot_fn=fake_git)
            raise AssertionError("expected the aborted launch to re-raise")
        except RuntimeError:
            pass
    finally:
        subprocess.Popen = real_popen
    check(spied2 == [], "launch-set logic spawned no subprocess at all (synthetic snapshot/git/runner)")
    names = lambda tag: {k: v.format(tag=tag) for k, v in MX.LAUNCH_TEMPLATES.items()}  # noqa: E731
    check(res1["tag"] == "STAMP" and res1["paths"] == {k: C.rel(R.OUT_ROLLOUTS / n) for k, n in names("STAMP").items()}, "first launch: tag = stamp, the three reserved names share it")
    check(res1["status"]["ok"] == 3 and res1["status"]["failed"] == 0 and res1["status"]["job_count"] == 3, "first launch: 3 fake jobs ok")
    first_sha = {k: C.sha256_file(R.OUT_ROLLOUTS / n) for k, n in names("STAMP").items()}
    clo = json.loads((R.OUT_ROLLOUTS / names("STAMP")["closure"]).read_text(encoding="utf-8"))
    check(clo["closure_schema_version"] == CL.CLOSURE_SCHEMA_VERSION and clo["job_inputs"] == {r["job_id"]: r["job_inputs_digest"] for r in recs_l} and clo["runtime_source_closure_digest"] == synthetic_snapshot()["runtime_source_closure_digest"], "closure manifest payload: schema 2, job bindings, digests (F0-equivalent payload)")
    check(all(CL.verify_manifest_file(C.rel(R.OUT_ROLLOUTS / names("STAMP")["closure"]), first_sha["closure"], reference=synthetic_snapshot(), job_id=r["job_id"], job_inputs_digest=r["job_inputs_digest"]) == {} for r in recs_l), "unchanged f0_closure.verify_manifest_file accepts the F2R-written closure manifest for every job")
    ex = json.loads((R.OUT_ROLLOUTS / names("STAMP")["execute"]).read_text(encoding="utf-8"))
    check(ex["tag"] == "STAMP" and ex["launch_set"] == names("STAMP") and ex["closure_manifest"]["closure_manifest_sha256"] == first_sha["closure"] and ex["git"] == {"head": "synthetic", "dirty": []}, "launch manifest written before the jobs, bound to the closure manifest of the same set")
    rec0 = json.loads((C.REPO / recs_l[0]["output_dir"] / MX.RECEIPT_FILE).read_text(encoding="utf-8"))
    check(rec0["closure_manifest"] == C.rel(R.OUT_ROLLOUTS / names("STAMP")["closure"]) and rec0["closure_manifest_sha256"] == first_sha["closure"] and rec0["status"] == "ok" and CL.verify_closure_fields(rec0) == {}, "receipts point to the reserved closure manifest; F0 closure validator accepts them")
    check(res2["tag"] == "STAMP_01" and res2["status"]["skipped_verified_identical"] == 3 and res2["status"]["ok"] == 0, "same-second relaunch: tag STAMP_01 for all three names; jobs skipped verified-identical")
    check({k: C.sha256_file(R.OUT_ROLLOUTS / n) for k, n in names("STAMP").items()} == first_sha, "the first launch set is untouched by the second launch")
    check(all((R.OUT_ROLLOUTS / n).is_file() and (R.OUT_ROLLOUTS / n).stat().st_size > 2 for n in names("STAMP_01").values()), "second set: all three files present and filled")
    check(res3["tag"] == "STAMP_03" and not (R.OUT_ROLLOUTS / names("STAMP_02")["closure"]).exists() and not (R.OUT_ROLLOUTS / names("STAMP_02")["execute"]).exists() and (R.OUT_ROLLOUTS / names("STAMP_02")["status"]).read_text() == "{}", "partial collision on STAMP_02 (status only): whole tag skipped, no orphan closure/execute, foreign file untouched")
    aborted = json.loads((R.OUT_ROLLOUTS / names("STAMP_04")["status"]).read_text(encoding="utf-8"))
    check(aborted["status"] == "aborted" and "synthetic execution failure" in aborted["error"] and (R.OUT_ROLLOUTS / names("STAMP_04")["closure"]).stat().st_size > 2 and (R.OUT_ROLLOUTS / names("STAMP_04")["execute"]).stat().st_size > 2, "aborted execution: status filled with an aborted payload (no empty reservation), closure + launch already on disk")
    launch_files = sorted(p.name for p in R.OUT_ROLLOUTS.iterdir() if p.is_file())
    expected_files = sorted([*names("STAMP").values(), *names("STAMP_01").values(), names("STAMP_02")["status"], *names("STAMP_03").values(), *names("STAMP_04").values(), MX.DRIVER_LOG])
    check(launch_files == expected_files and not any(".part-" in n for n in launch_files), "exact launch files on disk (4 sets + foreign status + append-only driver log), no temp parts")
    check(sorted(p.name for p in R.OUT_ROOT.iterdir()) == ["refit", "rollouts"], "launch root holds only refit/ (student module) and rollouts/")
    check(snapshot_tree(real_out_root) == real_snapshot and R.OUT_ROOT != real_out_root and str(R.OUT_ROOT).startswith(str(tmp)), f"real output root byte-identical to the pre-test snapshot at the end ({len(real_snapshot)} files); all writes went to the temp root")
    # ================= REAL, read-only: the real root (populated by audited S1 stages) with the T1R selection
    R.OUT_ROOT = real_out_root
    for name, sub in {"OUT_MANIFEST": "manifest", "OUT_ROLLOUTS": "rollouts", "OUT_DATASETS": "datasets", "OUT_CACHE": "privileged_cache", "OUT_REFIT": "refit", "OUT_P0": "p0", "OUT_METRICS": "metrics", "OUT_GATE": "gate", "OUT_LOGS": "logs"}.items():
        setattr(R, name, real_out_root / sub)
    snap_real = snapshot_tree(real_out_root)
    if (real_out_root / "rollouts" / "T1" / "round_1").is_dir():
        real_jobs = MX.build_jobs(); real_described = [MX.describe_job(j, real_py) for j in real_jobs]
        t1r_real_ids = {j["job_id"] for j in real_jobs if j["phase"] == "T1R"}
        pf_real = MX.preflight_all(real_described, selected_ids=t1r_real_ids)
        t1_ids = sorted(j["job_id"] for j in real_jobs if j["phase"] == "T1")
        check(pf_real["invalid"] == [] and sorted(pf_real["verified_integrity_only"]) == t1_ids and sorted(d["job_id"] for d in pf_real["tooling_revision_differs"]) == t1_ids and all(d["integrity"] == "verified" and "f2r_driver_inputs_digest" in d for d in pf_real["tooling_revision_differs"]), f"REAL root: T1R selection -> preflight PASS; the 3 T1 outputs are integrity-verified with an explicit revision mismatch ({pf_real['tooling_revision_differs'][0]['f2r_driver_inputs_digest']['stored'][:8]}.. -> {pf_real['tooling_revision_differs'][0]['f2r_driver_inputs_digest']['fresh'][:8]}..)")
        expect(lambda: MX.preflight_all(real_described, selected_ids=set(t1_ids)), MX.MatrixError, "REAL root: selecting the T1 jobs themselves is refused (stale driver revision, no silent reuse)")
        check(snapshot_tree(real_out_root) == snap_real, "REAL root byte-identical after the read-only preflight (no receipt rewritten, nothing created)")
    else:
        check(True, "real T1 outputs not present on this machine: real preflight regression skipped")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
