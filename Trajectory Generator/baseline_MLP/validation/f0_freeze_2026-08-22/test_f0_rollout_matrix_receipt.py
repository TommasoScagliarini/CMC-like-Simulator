"""Self-test of the fail-closed receipt verification, the schema-4/schema-5
branches, the contemporaneous-closure protocol (runtime closure + per-job
scientific inputs + closure manifest on disk), the CANONICAL identity / output
path / RAW-argv binding and the ctrl39 actor contract of f0_rollout_matrix.

Runs without the simulation stack and launches NO rollout: the only subprocess
is a synthetic rollout SCRIPT reached through a SYNTHETIC ROOT patched in this
process only (C.REPO, C.OUT_ROLLOUTS, C.ROLLOUT_EVAL and one candidate module /
one runtime config redirected to a tempfile tree, restored afterwards), so
that output directories are canonical (OUT_ROLLOUTS/family/job_id[__tag]) and
the launched argv is exactly what job_command emits. Side effects of the fake
rollout are requested through environment variables, never through argv.
Temporary trees live under /private/tmp (tempfile) and are removed at the end;
the real F0 output root and the recorded summaries are only READ.

Covered:
  ctrl39     - flag carried only by v26_imitation_native; contract on recorded
               feature names and env source;
  canonical  - job_id -> canonical spec index (unique); seed 123->124 or family
               det->stoch with argv/normalized updated -> rejected; every
               identity field must equal the canonical spec (type + value);
  paths      - retry_tag allowlist validated in describe_job, in the validator
               and in main() before ensure_out_dirs; output_dir must be exactly
               the canonical relative POSIX path (absolute, traversal,
               backslash, wrong family/job, inconsistent retry -> rejected;
               run_job raises BEFORE any filesystem access: no directory, no
               marker, no subprocess);
  raw argv   - whole RAW argv == job_command(canonical spec, canonical dir,
               python) token per token: FSM v3->v2, removed diagnostic flag,
               timeout, removed --record-*/--no-progress, byte-identical script
               copy, extra/swapped tokens; command[1] with '.'/'..' or a symlink
               bridge (the auditor's normpath-equal PoC) rejected before the
               subprocess; fingerprint and SHA checks kept;
  legacy v4  - schema-4 receipts accepted through the legacy branch; the 28
               real receipts pass preflight unchanged; 58/58 descriptions bound;
  v5         - closure fields, manifest on disk (pre/post), job inputs, drift
               before/during the job, closure mismatches, symlink in module dir;
  kept       - digests, provenance fields, status/returncode, atomic preflight,
               closure handed to the runner, normalize_command (descriptive).
"""

from __future__ import annotations

import contextlib
import json
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_closure as CL  # noqa: E402
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

PASSED = 0
H1, H2, H3, H4 = "1" * 64, "2" * 64, "3" * 64, "4" * 64
DIAG = list(C.CONTROLLER_DIAGNOSTIC_FEATURES)
REAL_REPO = C.REPO
REAL_OUT_ROLLOUTS = C.OUT_ROLLOUTS
REAL_ROLLOUT_EVAL = C.ROLLOUT_EVAL
V26_HIST_SUMMARY = C.RUNS_ROLLOUT / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout" / "rollout_summary.json"
FAKE_ROLLOUT_SCRIPT = """import json, os, pathlib, sys
a = sys.argv
d = pathlib.Path(a[a.index('--output-dir') + 1])
(d / 'rollout_summary.json').write_text(json.dumps({'ok': True, 'steps': 1}))
(d / 'rollout_policy_trace.json').write_text('[{"step": 1}]')
if os.environ.get('F0_FAKE_MUTATE'):
    open(os.environ['F0_FAKE_MUTATE'], 'ab').write(b'drift')
if os.environ.get('F0_FAKE_TAMPER_MANIFEST'):
    open(os.environ['F0_FAKE_TAMPER_MANIFEST'], 'ab').write(b'\\n')
if os.environ.get('F0_FAKE_DELETE_MANIFEST'):
    pathlib.Path(os.environ['F0_FAKE_DELETE_MANIFEST']).unlink()
if os.environ.get('F0_FAKE_MARKER'):
    pathlib.Path(os.environ['F0_FAKE_MARKER']).write_text('ran')
"""
EVIL_SCRIPT = "import pathlib, sys\npathlib.Path(__file__).with_name('EVIL_MARKER').write_text('evil ran')\n"
FAKE_ENV_KEYS = {"mutate": "F0_FAKE_MUTATE", "tamper_manifest": "F0_FAKE_TAMPER_MANIFEST", "delete_manifest": "F0_FAKE_DELETE_MANIFEST", "marker": "F0_FAKE_MARKER"}


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def expect_error(fn, label: str, needle: str | None = None, exc_types=(RuntimeError,)) -> None:
    try:
        fn()
    except exc_types as exc:
        if needle and needle not in str(exc):
            raise AssertionError(f"{label}: error raised but does not mention {needle!r}: {exc}")
        ok(f"{label} -> {type(exc).__name__} ({str(exc)[:80]}...)")
        return
    raise AssertionError(f"{label}: expected {exc_types}, none raised")


def write_outputs(out_dir: Path, summary: dict, trace: object = None) -> dict:
    (out_dir / M.SUMMARY_FILE).write_text(json.dumps(summary), encoding="utf-8")
    (out_dir / M.TRACE_FILE).write_text(json.dumps(trace if trace is not None else [{"step": 1}]), encoding="utf-8")
    return M.output_digest(out_dir)


def write_receipt(out_dir: Path, fresh: dict, **overrides) -> None:
    stored = dict(fresh)
    stored.update(overrides)
    (out_dir / M.RECEIPT_FILE).write_text(json.dumps(stored, default=str), encoding="utf-8")


def fake_env(version: str = "3.12.0", executable: str | None = None) -> dict:
    exe = executable or sys.executable
    stable = {"executable_realpath": os.path.realpath(exe), "python_version": version, "platform": "fake-os", "machine": "arm64", "modules": {"torch": "2.0", "ray": "2.0", "opensim": "4.5", "numpy": "2.0", "yaml": "6.0"}}
    return {"role": "rollout_interpreter", "requested_executable": exe, "executable": exe, **stable, "fingerprint_sha256": CL.canonical_sha256(stable)}


def fake_snapshot(*, plugin_sha: str = H2, orch_sha: str = H3, env_version: str = "3.12.0", core_sha: str = H1, executable: str | None = None) -> dict:
    runtime_core = [{"path": "main.py", "sha256": core_sha, "bytes": 10}]
    native = [{"path": "plugins/libX.dylib", "sha256": plugin_sha, "bytes": 5, "referenced_by_config": True, "loaded_in_process_proven": False}]
    assets = [{"path": "models/x.xml", "sha256": H4, "bytes": 3}]
    orch = [{"path": "f0_common.py", "sha256": orch_sha, "bytes": 7}]
    return {
        "runtime_source_closure_digest": CL.table_digest(runtime_core, native, assets), "runtime_core": runtime_core, "native_plugins": native, "data_assets": assets,
        "orchestration_digest": CL.table_digest(orch), "orchestration": orch, "environment_fingerprint": fake_env(env_version, executable), "git": {"head": "h"},
    }


class Closure:
    def __init__(self, root: Path, snapshot: dict, bindings: dict[str, str], name: str = "source_closure_manifest_test.json"):
        self.snapshot = snapshot
        self.bindings = dict(bindings)
        root.mkdir(parents=True, exist_ok=True)
        written = CL.write_closure_manifest(root / name, snapshot, job_inputs=bindings)
        self.rel = written["closure_manifest"]
        self.sha = written["closure_manifest_sha256"]

    def for_run(self, snapshot_fn) -> dict:
        return {"closure_manifest": self.rel, "closure_manifest_sha256": self.sha, "runtime_source_closure_digest": self.snapshot["runtime_source_closure_digest"], "launch_snapshot": self.snapshot, "job_inputs": self.bindings, "snapshot_fn": snapshot_fn}

    def fields(self, fresh: dict, **overrides) -> dict:
        base = {
            "runtime_source_closure_digest_pre": self.snapshot["runtime_source_closure_digest"], "runtime_source_closure_digest_post": self.snapshot["runtime_source_closure_digest"],
            "orchestration_digest": self.snapshot["orchestration_digest"],
            **{s: self.snapshot[s] for s in CL.SNAPSHOT_SECTIONS},
            "job_inputs": fresh["job_inputs"], "job_inputs_digest_pre": fresh["job_inputs_digest"], "job_inputs_digest_post": fresh["job_inputs_digest"], "job_inputs_unchanged": True,
            "closure_manifest": self.rel, "closure_manifest_sha256": self.sha, "source_closure_unchanged": True,
            "closure_manifest_verified_pre_launch": {"ok": True, "problems": {}}, "closure_manifest_verified_post": {"ok": True, "problems": {}}, "closure_manifest_unchanged": True,
            "provenance_class": CL.PROVENANCE_CLASS_CONTEMPORANEOUS, "provenance_limitation": CL.PROVENANCE_LIMITATION,
        }
        base.update(overrides)
        return base


def make_v5(out_dir: Path, fresh: dict, closure: Closure, **overrides) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    digest = write_outputs(out_dir, {"ok": True, "steps": 500})
    fields = dict(status="ok", returncode=0, summary_sha256=digest["summary_sha256"], trace_sha256=digest["trace_sha256"], **closure.fields(fresh))
    fields.update(overrides)
    write_receipt(out_dir, fresh, **fields)


def make_v4(out_dir: Path, fresh: dict, **overrides) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    digest = write_outputs(out_dir, {"ok": True, "steps": 500})
    legacy = {k: v for k, v in fresh.items() if k not in ("provenance_class", "provenance_limitation", "job_inputs", "job_inputs_digest", "job_inputs_count")}
    legacy["schema_version"] = 4
    fields = dict(status="ok", returncode=0, summary_sha256=digest["summary_sha256"], trace_sha256=digest["trace_sha256"])
    fields.update(overrides)
    write_receipt(out_dir, legacy, **fields)


def make_bundle(root: Path) -> dict:
    data = root / "bundle"
    data.mkdir(parents=True)
    files = {"model": data / "model.osim", "ik": data / "ik.mot", "grf": data / "grf.mot", "actuators": data / "actuators.xml"}
    for p in files.values():
        p.write_text(f"content of {p.name}\n", encoding="utf-8")
    ext = data / "ExternalForces.xml"
    ext.write_text("<OpenSimDocument><ExternalLoads><datafile>grf.mot</datafile></ExternalLoads></OpenSimDocument>", encoding="utf-8")
    setup = data / "setup.xml"
    setup.write_text(f"<OpenSimDocument><CMC_Simulator_Setup><model_file>{files['model']}</model_file><kinematics_file>{files['ik']}</kinematics_file><external_loads_xml>{ext}</external_loads_xml><reserve_actuators_xml>{files['actuators']}</reserve_actuators_xml></CMC_Simulator_Setup></OpenSimDocument>", encoding="utf-8")
    cfg = root / "training_cfg.resolved.yaml"
    cfg.write_text(f"simulation:\n  setup_xml: {setup}\n  episode_duration: 5.0\n  segment_duration: 0.01\n", encoding="utf-8")
    module = root / "rl_module_test"
    module.mkdir()
    for name in ("module_state.pkl", "class_and_ctor_args.pkl", "metadata.json"):
        (module / name).write_bytes(b"bytes of " + name.encode())
    return {"config": cfg, "module": module, "setup": setup, "ext": ext, **files}


@contextlib.contextmanager
def synthetic_root(root: Path, candidate: str, runtime: str, bundle: dict, fake_script: Path):
    """Synthetic repository root for this process only: canonical output dirs under root/rollouts,
    fake rollout script, one candidate module and one runtime config redirected to temp files."""
    saved = (C.REPO, C.OUT_ROLLOUTS, C.ROLLOUT_EVAL, C.CANDIDATES[candidate]["module"], C.RUNTIMES[runtime]["config"])
    C.REPO = root
    C.OUT_ROLLOUTS = root / "rollouts"
    C.ROLLOUT_EVAL = fake_script
    C.CANDIDATES[candidate]["module"] = bundle["module"]
    C.RUNTIMES[runtime]["config"] = bundle["config"]
    try:
        yield
    finally:
        C.REPO, C.OUT_ROLLOUTS, C.ROLLOUT_EVAL, C.CANDIDATES[candidate]["module"], C.RUNTIMES[runtime]["config"] = saved


@contextlib.contextmanager
def fake_effects(**effects):
    saved = {FAKE_ENV_KEYS[k]: os.environ.get(FAKE_ENV_KEYS[k]) for k in effects}
    for k, v in effects.items():
        os.environ[FAKE_ENV_KEYS[k]] = str(v)
    try:
        yield
    finally:
        for key, old in saved.items():
            if old is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old


def main() -> int:
    base = "/private/tmp" if (Path("/private/tmp").is_dir() and os.access("/private/tmp", os.W_OK)) else tempfile.gettempdir()
    tmp_root = Path(tempfile.mkdtemp(prefix="f0_receipt_selftest_", dir=base))
    try:
        jobs = M.build_jobs()
        index = M.canonical_index()
        assert len(index) == 58 and all(M.job_id(s) in index and index[M.job_id(s)] == s for s in jobs)
        ok("canonical index: 58 unique job_ids -> specs (collision-free)")

        # --- ctrl39: command flag and actor contract (read-only, no OpenSim) ---------------------
        for spec in jobs:
            cmd = M.job_command(spec, tmp_root / "o", "py")
            has_flag = C.NO_CONTROLLER_DIAGNOSTIC_FLAG in cmd
            assert has_flag == (spec[2] == "v26_imitation_native"), (M.job_id(spec), cmd)
            if has_flag:
                assert cmd[cmd.index("--binary-phase-actor-fsm-version") + 1] == "v3" and cmd.count(C.NO_CONTROLLER_DIAGNOSTIC_FLAG) == 1
        assert sum(1 for s in jobs if s[2] == "v26_imitation_native") == 3
        ok("ctrl39: the 3 v26_imitation_native commands carry --no-include-controller-diagnostic-observation + FSM v3; the other 55 jobs do not")
        m39 = C.load_actor_feature_manifest("V26_39D")
        m35 = C.load_actor_feature_manifest("B0820_H0")
        assert m39["sha256"] == C.ACTOR_MANIFEST_39_SHA256 and m39["actor_feature_count"] == 39 and m35["sha256"] == C.ACTOR_MANIFEST_35_SHA256 and m35["actor_feature_count"] == 35
        h43 = C.read_json(V26_HIST_SUMMARY)
        s35 = C.read_json(REAL_OUT_ROLLOUTS / "det" / "B0820_H0__v3_canonical__nominal__det" / M.SUMMARY_FILE)
        a43, o88 = h43["actor_feature_names"], h43["observation_feature_names"]
        a35, o84 = s35["actor_feature_names"], s35["observation_feature_names"]
        assert h43["include_controller_diagnostic_observation"] is True and s35["include_controller_diagnostic_observation"] is False
        assert a43 == m39["actor_feature_names"] + DIAG and o88[:43] == a43 and len(o88) == 88
        suffix45 = o88[43:]
        assert a35 == m35["actor_feature_names"] and o84 == a35 + DIAG + suffix45 and len(o84) == 84
        assert m39["actor_feature_names"] + DIAG + suffix45 == o88 and not any(n in DIAG for n in m39["actor_feature_names"])
        assert m35["actor_feature_names"][:2] + [n for n in m39["actor_feature_names"] if n.startswith("healthy_")] + m35["actor_feature_names"][2:] == m39["actor_feature_names"]
        src = (REAL_REPO / "Trajectory Generator" / "osim_trj_cmc_like.py").read_text(encoding="utf-8")
        assert re.search(r"include_controller_diagnostic_observation\s*\)\s*:\s*actor\.update\(controller_diagnostics\)\s*else:\s*priv\.update\(controller_diagnostics\)", src)
        ok("contract (recorded names + env source): 43D actor = 39 manifest + 4 trailing diagnostics; with the flag the expected actor prefix is exactly the 39 manifest names, the 4 diagnostics head the privileged suffix and the full 88 observation keeps its order (35D analogue: 84 = 35 + 4 + 45)")

        # --- retry tag validation before any filesystem access --------------------------------------
        for bad in ("../evil", "a/b", "a\\b", ".", "..", "", ".hidden", "x" * 65, "tag with space", 5):
            expect_error(lambda b=bad: M.validate_retry_tag(b), f"retry tag {bad!r}", "invalid --retry-tag")
        assert M.validate_retry_tag(None) is None and M.validate_retry_tag("r2") == "r2" and M.validate_retry_tag("A.b-c_9") == "A.b-c_9"
        expect_error(lambda: M.describe_job(jobs[0], sys.executable, "../evil"), "describe_job with traversal retry tag", "invalid --retry-tag")
        calls: list = []
        saved_ensure, saved_argv = C.ensure_out_dirs, sys.argv
        C.ensure_out_dirs = lambda: calls.append("ensure_out_dirs")
        sys.argv = ["f0_rollout_matrix.py", "--retry-tag", "../evil"]
        try:
            expect_error(lambda: M.main(), "main() with traversal retry tag", "invalid --retry-tag")
        finally:
            C.ensure_out_dirs, sys.argv = saved_ensure, saved_argv
        assert calls == [], "ensure_out_dirs must not run before the retry tag is validated"
        ok("retry tag: allowlist [A-Za-z0-9][A-Za-z0-9._-]{0,63} enforced in validate_retry_tag, describe_job and main() before ensure_out_dirs")

        # --- canonical identity (real root, read-only): seed / family / other fields with argv + normalized updated -------
        det_spec = next(j for j in jobs if j[0] == "det" and j[1] == "B0820_H0" and j[3] == "nominal")
        det = M.describe_job(det_spec, sys.executable, None)
        assert det["seed"] == 123 and det["family"] == "det" and M.command_binding_problems(det) == {}

        def identity_case(**changes) -> dict:
            r = dict(det)
            r.update(changes)
            if "seed" in changes:
                r["command"] = [str(changes["seed"]) if (i > 0 and r["command"][i - 1] == "--seed") else t for i, t in enumerate(r["command"])]
            if "action_selection" in changes:
                r["command"] = [changes["action_selection"] if (i > 0 and r["command"][i - 1] == "--action-selection") else t for i, t in enumerate(r["command"])]
            r["command_normalized"] = M.normalize_command(r["command"])
            return M.command_binding_problems(r)

        p = identity_case(seed=124)
        assert "spec" in p and "seed" in p["spec"] and "argv" not in p, p
        p = identity_case(family="stoch")
        assert "spec" in p and "family" in p["spec"], p
        p = identity_case(family="stoch", action_selection="stochastic")
        assert "spec" in p and {"family", "action_selection"} <= set(p["spec"]), p
        for field, value in (("seed", True), ("repeat", 2), ("candidate", "B0820_V3_BEST"), ("runtime", "v2_b0820"), ("start", "plus020"), ("historical_reference_summary", None), ("job_id", "NOT_A_JOB")):
            p = identity_case(**{field: value})
            assert "spec" in p and field in p["spec"], (field, p)
        for bad_id in (None, "", 5):
            assert "spec" in identity_case(job_id=bad_id)
        ok("canonical identity: det seed 123->124 and family det->stoch (argv + normalized updated) rejected; every identity field (candidate, runtime, start, action_selection, seed incl. bool, repeat, reference, job_id) bound to the canonical spec")

        # --- output_dir / retry_tag canonical path (auditor PoC; validator level, no filesystem) --------
        canon = det["output_dir"]
        assert canon == C.rel(C.OUT_ROLLOUTS / "det" / det["job_id"]) and canon.endswith(f"rollouts/det/{det['job_id']}") and ".." not in canon.split("/")
        other_job = next(j for j in jobs if j[0] == "det" and M.job_id(j) != det["job_id"])

        def path_case(new_out: object, tag: object = None) -> dict:
            r = dict(det, output_dir=new_out, retry_tag=tag)
            if isinstance(new_out, str):
                target = str(C.REPO / new_out) if not os.path.isabs(new_out) else new_out
                r["command"] = [target if (i > 0 and r["command"][i - 1] == "--output-dir") else t for i, t in enumerate(r["command"])]
                r["command_normalized"] = M.normalize_command(r["command"])
            return M.command_binding_problems(r)

        path_cases = {
            "absolute": str(tmp_root / "abs_out"),
            "traversal ../../../../private/tmp": "../../../../private/tmp/f0_evil_out",
            "backslash": canon.replace("/", "\\"),
            "wrong family": canon.replace("/det/" + det["job_id"], "/stoch/" + det["job_id"]),
            "other job": canon.replace(det["job_id"], M.job_id(other_job)),
            "dot component": canon.replace("/det/", "/./det/"),
            "trailing slash": canon + "/",
            "leading slash": "/" + canon,
            "not a string": 5,
        }
        for label, bad_out in path_cases.items():
            p = path_case(bad_out)
            assert "output_dir" in p, (label, p)
        assert "output_dir" in path_case(canon, tag="r2")  # tag declared but dir without suffix
        assert "output_dir" in path_case(canon + "__r2", tag=None)  # suffix without tag
        p = path_case(canon + "__r2", tag="r2")
        assert "output_dir" not in p and "spec" not in p and "retry_tag" not in p, p  # consistent tag accepted (argv differs only by the dir)
        for bad_tag in ("../x", "a/b", "", ".", 7):
            assert "retry_tag" in path_case(canon, tag=bad_tag), bad_tag
        ok("output_dir: absolute, traversal, backslash, wrong family, other job, '.' component, trailing/leading slash, non-string, retry inconsistent (tag without suffix / suffix without tag / invalid tag) -> rejected; consistent canonical tag accepted")

        # --- ctrl39 RAW argv (real root, validator level): FSM v3 -> v2 and diagnostic flag removal -------------
        c39_spec = next(j for j in jobs if j[0] == "ctrl39")
        c39 = M.describe_job(c39_spec, sys.executable, None)
        assert M.command_binding_problems(c39) == {}
        ccmd = list(c39["command"])
        i_fsm = ccmd.index("--binary-phase-actor-fsm-version") + 1
        for label, new_cmd in (("FSM v3 -> v2", ccmd[:i_fsm] + ["v2"] + ccmd[i_fsm + 1:]), ("--no-include-controller-diagnostic-observation removed", [t for t in ccmd if t != C.NO_CONTROLLER_DIAGNOSTIC_FLAG])):
            p = M.command_binding_problems(dict(c39, command=new_cmd, command_normalized=M.normalize_command(new_cmd)))
            assert "argv" in p, (label, p)
        ok("ctrl39 RAW argv: FSM v3 -> v2 and removed --no-include-controller-diagnostic-observation rejected even with command_normalized recomputed")

        bundle = make_bundle(tmp_root / "inputs")
        scripts = tmp_root / "scripts"
        scripts.mkdir()
        fake_script = scripts / "fake_rollout_eval.py"
        fake_script.write_text(FAKE_ROLLOUT_SCRIPT, encoding="utf-8")
        spec_job = jobs[1]  # JUL_H0 / july_legacy / nominal / det (historical reference read-only)
        with synthetic_root(tmp_root, spec_job[1], spec_job[2], bundle, fake_script):
            # --- fixtures under the synthetic root (canonical dirs, argv == job_command) ----------------
            fresh = M.describe_job(jobs[0], sys.executable, None)
            assert fresh["schema_version"] == 5 and fresh["output_dir"] == f"rollouts/replay/{fresh['job_id']}" and fresh["command"][1] == str(fake_script) and fresh["config"] == C.rel(bundle["config"])
            assert M.command_binding_problems(fresh) == {} and M.command_binding_problems(fresh, fingerprint=fake_env(), require_fingerprint=True) == {}
            assert M.command_binding_problems(fresh, require_fingerprint=True) == {"environment_fingerprint": {"existing": None, "expected": "interpreter fingerprint bound to command[0]"}}
            ok("command binding of a genuine description: bound (with and without fingerprint); fingerprint required for schema 5")
            out_dir = C.REPO / fresh["output_dir"]
            fresh_n = M.describe_job(next(j for j in jobs if j[7] is None), sys.executable, None)
            out_n = C.REPO / fresh_n["output_dir"]
            fresh_b = M.describe_job(jobs[2], sys.executable, None)
            out_b = C.REPO / fresh_b["output_dir"]
            assert M.command_binding_problems(fresh_n) == {} and M.command_binding_problems(fresh_b) == {}
            snap = fake_snapshot()
            closure = Closure(tmp_root / "launch", snap, {d["job_id"]: d["job_inputs_digest"] for d in (fresh, fresh_n, fresh_b)})

            # --- accept: v5 and legacy v4 ---------------------------------------------
            make_v5(out_dir, fresh, closure)
            old = M.verify_existing(fresh, out_dir)
            assert old["job_id"] == fresh["job_id"] and M.verification_branch(old) == M.BRANCH_CONTEMPORANEOUS_V5
            ok("schema-5 receipt with consistent closure -> accepted")
            make_v4(out_dir, fresh)
            old = M.verify_existing(fresh, out_dir)
            assert M.verification_branch(old) == M.BRANCH_LEGACY_V4 and "runtime_source_closure_digest_pre" not in old
            ok("schema-4 receipt without closure fields -> accepted through legacy_schema_v4_class_B")

            # --- reject: v5 closure ---------------------------------------------------
            for field in M.CLOSURE_RECEIPT_FIELDS:
                make_v5(out_dir, fresh, closure)
                stored = json.loads((out_dir / M.RECEIPT_FILE).read_text(encoding="utf-8"))
                stored.pop(field)
                (out_dir / M.RECEIPT_FILE).write_text(json.dumps(stored, default=str), encoding="utf-8")
                expect_error(lambda: M.verify_existing(fresh, out_dir), f"v5 missing {field}", "closure")
            other = fake_snapshot(plugin_sha=H1)
            for label, overrides, needle in (
                ("pre != post", {"runtime_source_closure_digest_post": H2}, "pre_vs_post"),
                ("source_closure_unchanged False", {"source_closure_unchanged": False}, "source_closure_unchanged"),
                ("job_inputs_unchanged False", {"job_inputs_unchanged": False}, "job_inputs_unchanged"),
                ("closure manifest SHA mismatch", {"closure_manifest_sha256": H2}, "closure_manifest_sha256"),
                ("closure manifest missing", {"closure_manifest": C.rel(tmp_root / "absent.json")}, "closure_manifest"),
                ("native plugin table differs from manifest", {"native_plugins": other["native_plugins"], "runtime_source_closure_digest_pre": other["runtime_source_closure_digest"], "runtime_source_closure_digest_post": other["runtime_source_closure_digest"]}, "closure_manifest"),
                ("native plugin table altered, digests kept", {"native_plugins": other["native_plugins"]}, "runtime_source_closure_digest_recomputed"),
                ("runtime_core table altered", {"runtime_core": fake_snapshot(core_sha=H2)["runtime_core"]}, "runtime_source_closure_digest_recomputed"),
                ("data_assets table altered", {"data_assets": [{"path": "models/y.xml", "sha256": H1, "bytes": 1}]}, "runtime_source_closure_digest_recomputed"),
                ("orchestration table differs from manifest", {"orchestration": fake_snapshot(orch_sha=H1)["orchestration"], "orchestration_digest": fake_snapshot(orch_sha=H1)["orchestration_digest"]}, "closure_manifest_orchestration_digest"),
                ("orchestration table altered, digest kept", {"orchestration": fake_snapshot(orch_sha=H1)["orchestration"]}, "orchestration_digest_recomputed"),
                ("environment fingerprint differs from manifest", {"environment_fingerprint": fake_env("3.11.0")}, "closure_manifest_section_environment_fingerprint"),
                ("environment fingerprint of the driver", {"environment_fingerprint": dict(fake_env(), role="driver_current_interpreter")}, "environment_fingerprint"),
                ("job inputs table altered", {"job_inputs": [{"path": "x.yaml", "sha256": H1, "bytes": 1}]}, "job_inputs_digest_recomputed"),
                ("job inputs digest not bound in the manifest", {"job_inputs_digest_pre": H1, "job_inputs_digest_post": H1, "job_inputs": [{"path": "x.yaml", "sha256": H1, "bytes": 1}]}, "job_inputs"),
                ("malformed digests", {"runtime_source_closure_digest_pre": "zz", "runtime_source_closure_digest_post": "zz"}, "64-hex"),
                ("empty native plugin table", {"native_plugins": []}, "native_plugins"),
                ("provenance_class A claimed", {"provenance_class": "A"}, "provenance_class"),
                ("unsupported schema 6", {"schema_version": 6}, "unsupported receipt schema_version"),
                ("pre-launch manifest flag not ok", {"closure_manifest_verified_pre_launch": {"ok": False, "problems": {"x": 1}}}, "closure_manifest_verified_pre_launch"),
                ("post manifest flag not ok", {"closure_manifest_verified_post": {"ok": False, "problems": {}}}, "closure_manifest_verified_post"),
                ("closure_manifest_unchanged False", {"closure_manifest_unchanged": False}, "closure_manifest_unchanged"),
            ):
                make_v5(out_dir, fresh, closure, **overrides)
                expect_error(lambda: M.verify_existing(fresh, out_dir), f"v5 {label}", needle)
            make_v5(out_dir, fresh, closure)
            stored = json.loads((out_dir / M.RECEIPT_FILE).read_text(encoding="utf-8"))
            stored.pop("schema_version")
            (out_dir / M.RECEIPT_FILE).write_text(json.dumps(stored, default=str), encoding="utf-8")
            expect_error(lambda: M.verify_existing(fresh, out_dir), "schema_version missing", "unsupported receipt schema_version")
            bad_schema = Closure(tmp_root / "launch_bad_schema", snap, {fresh["job_id"]: fresh["job_inputs_digest"]})
            payload = C.read_json(C.REPO / bad_schema.rel)
            payload["closure_schema_version"] = 99
            (C.REPO / bad_schema.rel).write_text(json.dumps(payload), encoding="utf-8")
            make_v5(out_dir, fresh, closure, closure_manifest=bad_schema.rel, closure_manifest_sha256=C.sha256_file(C.REPO / bad_schema.rel))
            expect_error(lambda: M.verify_existing(fresh, out_dir), "v5 manifest with wrong closure_schema_version (SHA consistent)", "closure_manifest_schema")
            make_v5(out_dir, fresh, closure)
            expect_error(lambda: M.verify_existing(dict(fresh, job_inputs_digest=H1), out_dir), "current job inputs differ from what the job consumed", "job_inputs_digest")

            # --- command binding: explicit checks ----------------------------------------------------
            cmd = list(fresh["command"])

            def with_cmd(new_cmd: list, **extra) -> dict:
                return dict(command=new_cmd, command_normalized=M.normalize_command(new_cmd), **extra)

            def bound(**overrides) -> dict:
                stored = dict(fresh, **closure.fields(fresh), status="ok", returncode=0)
                stored.update(overrides)
                return M.command_binding_problems(stored, require_fingerprint=True)

            proof = {"command": ["/definitely/not/python"] + cmd[1:], "python": "/also/not/python"}
            assert {"command_normalized", "command[0]", "environment_fingerprint_executable"} <= set(bound(**proof))
            make_v5(out_dir, fresh, closure, **proof)
            expect_error(lambda: M.verify_existing(fresh, out_dir), "reviewer's proof: command[0] + python replaced, normalized/fingerprint untouched", "command_binding")
            cases = {
                "python field mutated alone": ({"python": "/also/not/python"}, "command[0]"),
                "command[0] + normalized mutated, python untouched": (with_cmd(["/definitely/not/python"] + cmd[1:]), "command[0]"),
                "command[0] + normalized + python mutated consistently, fingerprint untouched": (with_cmd(["/definitely/not/python"] + cmd[1:], python="/definitely/not/python"), "environment_fingerprint_executable"),
                "fingerprint realpath discordant": ({"environment_fingerprint": fake_env(executable="/definitely/not/python")}, "environment_fingerprint_executable"),
                "fingerprint role driver": ({"environment_fingerprint": dict(fake_env(), role="driver_current_interpreter")}, "environment_fingerprint_role"),
                "--checkpoint missing": (with_cmd([t for i, t in enumerate(cmd) if not (t == "--checkpoint" or (i > 0 and cmd[i - 1] == "--checkpoint"))]), "--checkpoint"),
                "--config duplicated": (with_cmd(cmd + ["--config", cmd[cmd.index("--config") + 1]]), "--config"),
                "--output-dir misaligned": (with_cmd([str(tmp_root / "elsewhere") if (i > 0 and cmd[i - 1] == "--output-dir") else t for i, t in enumerate(cmd)]), "--output-dir"),
                "--checkpoint misaligned": (with_cmd([str(tmp_root) if (i > 0 and cmd[i - 1] == "--checkpoint") else t for i, t in enumerate(cmd)]), "--checkpoint"),
                "--checkpoint without value": (with_cmd([t for i, t in enumerate(cmd) if not (t == "--checkpoint" or (i > 0 and cmd[i - 1] == "--checkpoint"))] + ["--checkpoint"]), "--checkpoint"),
                "--checkpoint=value ambiguous token": (with_cmd(cmd[: cmd.index("--checkpoint")] + [f"--checkpoint={cmd[cmd.index('--checkpoint') + 1]}"] + cmd[cmd.index("--checkpoint") + 2:]), "--checkpoint"),
                "--no-auto-config missing": (with_cmd([t for t in cmd if t != "--no-auto-config"]), "--no-auto-config"),
                "--seed discordant": (with_cmd(["124" if (i > 0 and cmd[i - 1] == "--seed") else t for i, t in enumerate(cmd)]), "--seed"),
                "--episode-start-offset-s discordant": (with_cmd(["1.9" if (i > 0 and cmd[i - 1] == "--episode-start-offset-s") else t for i, t in enumerate(cmd)]), "--episode-start-offset-s"),
                "script token replaced (SHA != rollout_eval_sha256)": (with_cmd([cmd[0], str(REAL_REPO / "config.py")] + cmd[2:]), "command[1]_sha256"),
                "script token missing file": (with_cmd([cmd[0], str(tmp_root / "absent.py")] + cmd[2:]), "command[1]"),
                "cwd not the repository": ({"cwd": str(REAL_REPO / "x")}, "cwd"),
                "command not a list": ({"command": "python rollout_eval.py"}, "command"),
                "command empty": (with_cmd([]), "command"),
                "command with empty token": (with_cmd(cmd + [""]), "command"),
            }
            for label, (overrides, key) in cases.items():
                problems = bound(**overrides)
                assert key in problems, (label, key, problems)
                make_v5(out_dir, fresh, closure, **overrides)
                expect_error(lambda: M.verify_existing(fresh, out_dir), f"command binding: {label}", "command_binding")
            ok("command binding rejects python/command[0] mutations, discordant fingerprint, flag/script/cwd/command malformations")

            # --- run_job refuses BEFORE any filesystem access ------------------------------------------
            log = tmp_root / "log.jsonl"

            def job(tag: str) -> dict:
                rec = M.describe_job(spec_job, sys.executable, tag)
                assert rec["output_dir"] == f"rollouts/replay/{rec['job_id']}__{tag}" and rec["command"][1] == str(fake_script)
                assert M.command_binding_problems(rec, fingerprint=fake_env(), require_fingerprint=True) == {}, M.command_binding_problems(rec, fingerprint=fake_env(), require_fingerprint=True)
                return rec

            def bind(j: dict, launch_cl: Closure) -> dict:
                return launch_cl.for_run(lambda: fake_snapshot()) | {"job_inputs": {j["job_id"]: j["job_inputs_digest"]}}

            stable = job("run_stable")
            launch = Closure(tmp_root / "launch_run", snap, {stable["job_id"]: stable["job_inputs_digest"]})
            evil_out = tmp_root / "EVIL_OUT"
            for label, mutate in (
                ("output_dir absolute (token rewritten)", lambda r: (r.update(output_dir=str(evil_out)), r.update(command=[str(evil_out) if (i > 0 and r["command"][i - 1] == "--output-dir") else t for i, t in enumerate(r["command"])]), r.update(command_normalized=M.normalize_command(r["command"])))),
                ("output_dir traversal (token rewritten)", lambda r: (r.update(output_dir="../../../../private/tmp/f0_evil_out_x"), r.update(command=[str(C.REPO / "../../../../private/tmp/f0_evil_out_x") if (i > 0 and r["command"][i - 1] == "--output-dir") else t for i, t in enumerate(r["command"])]), r.update(command_normalized=M.normalize_command(r["command"])))),
                ("output_dir backslash", lambda r: r.update(output_dir=r["output_dir"].replace("/", "\\"))),
                ("output_dir wrong family", lambda r: r.update(output_dir=r["output_dir"].replace("rollouts/replay/", "rollouts/det/"))),
                ("retry_tag inconsistent", lambda r: r.update(retry_tag="other")),
                ("retry_tag traversal", lambda r: r.update(retry_tag="../evil")),
                ("seed 124 (argv + normalized updated)", lambda r: (r.update(seed=124), r.update(command=["124" if (i > 0 and r["command"][i - 1] == "--seed") else t for i, t in enumerate(r["command"])]), r.update(command_normalized=M.normalize_command(r["command"])))),
                ("family stoch", lambda r: r.update(family="stoch")),
            ):
                j = job("run_refuse")
                mutate(j)
                marker = tmp_root / "run_refuse.marker"
                with fake_effects(marker=marker):
                    expect_error(lambda: M.run_job(dict(j), log_path=log, closure=bind(j, launch)), f"run_job refuses ({label}) before any filesystem access", "not bound to the canonical matrix")
                assert not marker.exists() and not evil_out.exists() and not (tmp_root / "rollouts" / "replay" / f"{j['job_id']}__run_refuse").exists() and not (tmp_root / "rollouts" / "det").exists() and not log.exists(), label
            assert not Path("/private/tmp/f0_evil_out_x").exists()
            ok("run_job: invalid output_dir (absolute / traversal / backslash / wrong family), inconsistent or traversal retry_tag, non-canonical seed/family -> RuntimeError before any filesystem access (no directory, no marker, no subprocess, no log)")

            # --- RAW argv: mutations + symlink bridge PoC (normpath-equal) ----------------------------------
            evil_dir = scripts / "evil"
            (evil_dir / "subdir").mkdir(parents=True)
            (evil_dir / "fake_rollout_eval.py").write_text(EVIL_SCRIPT, encoding="utf-8")
            (scripts / "bridge").symlink_to(evil_dir / "subdir", target_is_directory=True)
            bridged = str(scripts / "bridge" / ".." / "fake_rollout_eval.py")
            assert os.path.normpath(bridged) == str(fake_script) and os.path.realpath(bridged) == str((evil_dir / "fake_rollout_eval.py").resolve())
            j = job("run_bridge")
            j["command"] = [j["command"][0], bridged] + j["command"][2:]
            j["command_normalized"] = M.normalize_command(j["command"])
            p = M.command_binding_problems(j, fingerprint=fake_env(), require_fingerprint=True)
            assert "argv" in p and "command[1]" in p and "command[1]_components" in p and "command[1]_realpath" in p and "command_normalized" not in p, p
            expect_error(lambda: M.run_job(dict(j), log_path=log, closure=bind(j, launch)), "symlink bridge/../script PoC (normpath-equal, realpath evil)", "not bound to the canonical matrix")
            assert not (evil_dir / "EVIL_MARKER").exists() and not (tmp_root / "rollouts" / "replay" / f"{j['job_id']}__run_bridge").exists()
            twin = scripts / "twin_fake_rollout_eval.py"
            twin.write_bytes(fake_script.read_bytes())
            i_to = stable["command"].index("--run-timeout-s") + 1
            argv_cases = {
                "--run-timeout-s 7200 -> 7201": stable["command"][:i_to] + ["7201"] + stable["command"][i_to + 1:],
                "--record-policy-trace removed": [t for t in stable["command"] if t != "--record-policy-trace"],
                "--record-outputs removed": [t for t in stable["command"] if t != "--record-outputs"],
                "--no-progress removed": [t for t in stable["command"] if t != "--no-progress"],
                "command[1] byte-identical twin": [stable["command"][0], str(twin)] + stable["command"][2:],
                "extra token appended": stable["command"] + ["--extra"],
                "two flag groups swapped": stable["command"][:2] + stable["command"][4:6] + stable["command"][2:4] + stable["command"][6:],
                "script with '..' component (resolves to the same file)": [stable["command"][0], str(scripts / ".." / "scripts" / "fake_rollout_eval.py")] + stable["command"][2:],
            }
            for label, new_cmd in argv_cases.items():
                j = job("run_argv")
                j["command"], j["command_normalized"] = new_cmd, M.normalize_command(new_cmd)
                p = M.command_binding_problems(j, fingerprint=fake_env(), require_fingerprint=True)
                assert "argv" in p, (label, p)
                if "twin" in label:
                    assert "command[1]_sha256" not in p and "command[1]_realpath" in p, p
                if "'..'" in label:
                    assert "command[1]_components" in p, p
                marker = tmp_root / "run_argv.marker"
                with fake_effects(marker=marker):
                    expect_error(lambda: M.run_job(dict(j), log_path=log, closure=bind(j, launch)), f"run_job refuses raw argv mutation ({label})", "not bound to the canonical matrix")
                assert not marker.exists() and not (tmp_root / "rollouts" / "replay" / f"{j['job_id']}__run_argv").exists(), label
            ok("RAW argv: symlink bridge/../script PoC rejected (argv, command[1], components, realpath) with no evil marker and no directory; timeout, --record-*/--no-progress, byte-identical twin (SHA equal, realpath differs), '..' component, extra/swapped tokens -> rejected, subprocess never started")

            # --- run_job happy path and closure/inputs/manifest protocol ------------------------------------
            marker = tmp_root / "run_stable.marker"
            with fake_effects(marker=marker):
                rec = M.run_job(dict(stable), log_path=log, closure=launch.for_run(lambda: fake_snapshot()))
            assert rec["status"] == "ok" and marker.exists() and rec["source_closure_unchanged"] is True and rec["job_inputs_unchanged"] is True and rec["schema_version"] == 5
            assert rec["command_binding_pre_launch"] == {"ok": True, "problems": {}} and rec["closure_manifest_verified_pre_launch"]["ok"] is True and rec["closure_manifest_verified_post"]["ok"] is True and rec["closure_manifest_unchanged"] is True and len(rec["job_inputs"]) == 12
            M.verify_existing(job("run_stable"), C.REPO / stable["output_dir"])
            ok("run_job with argv == job_command(canonical spec): stable closure + inputs + manifest -> status ok (marker written), 12 bound inputs, receipt verifiable")
            for label, path in (("config", bundle["config"]), ("module_state", bundle["module"] / "module_state.pkl"), ("metadata", bundle["module"] / "metadata.json"), ("model .osim", bundle["model"]), ("GRF .mot", bundle["grf"]), ("actuators xml", bundle["actuators"])):
                tag = "pre_" + label.replace(" ", "_").replace(".", "")
                j = job(tag)
                original = path.read_bytes()
                path.write_bytes(original + (b"\n# tampered\n" if label == "config" else b"tampered"))
                marker = tmp_root / (tag + ".marker")
                try:
                    with fake_effects(marker=marker):
                        rec = M.run_job(dict(j), log_path=log, closure=bind(j, launch))
                finally:
                    path.write_bytes(original)
                assert rec["status"] == "failed" and rec["returncode"] is None and "not launched" in rec["status_reason"] and "job inputs differ" in rec["status_reason"], (label, rec["status_reason"])
                assert not marker.exists() and not (C.REPO / j["output_dir"] / "f0_driver_stdout.log").exists()
            j = job("pre_config_corrupt")
            original = bundle["config"].read_bytes()
            bundle["config"].write_bytes(original + b"tampered")
            try:
                rec = M.run_job(dict(j), log_path=log, closure=bind(j, launch))
            finally:
                bundle["config"].write_bytes(original)
            assert rec["status"] == "failed" and rec["returncode"] is None and "could not be resolved" in rec["status_reason"]
            ok("run_job pre-launch drift of config / module files / model / GRF / actuators -> not launched (corrupted YAML fails closed)")
            for label, path in (("module_state", bundle["module"] / "module_state.pkl"), ("model .osim", bundle["model"]), ("config", bundle["config"])):
                tag = "post_" + label.replace(" ", "_").replace(".", "")
                j = job(tag)
                original = path.read_bytes()
                try:
                    with fake_effects(mutate=path):
                        rec = M.run_job(dict(j), log_path=log, closure=bind(j, launch))
                finally:
                    path.write_bytes(original)
                assert rec["status"] == "failed" and rec["returncode"] == 0 and rec["job_inputs_unchanged"] is False and "job inputs changed" in rec["status_reason"], (label, rec["status_reason"])
                expect_error(lambda: M.verify_existing(job(tag), C.REPO / j["output_dir"]), f"post-drift ({label}) receipt rejected", "status")
            ok("run_job drift during the job -> failed despite rc 0; receipts rejected by verification")
            seq = iter([fake_snapshot(), fake_snapshot(plugin_sha=H1)])
            j = job("plugin_drift")
            rec = M.run_job(dict(j), log_path=log, closure=launch.for_run(lambda: next(seq)) | {"job_inputs": {j["job_id"]: j["job_inputs_digest"]}})
            assert rec["status"] == "failed" and rec["source_closure_unchanged"] is False and rec["closure_sections_post_equal_pre"]["native_plugins"] is False
            for label, snapshot in (("native_plugin", fake_snapshot(plugin_sha=H1)), ("orchestration", fake_snapshot(orch_sha=H1)), ("environment", fake_snapshot(env_version="3.11.0")), ("runtime_core", fake_snapshot(core_sha=H2))):
                j = job("pre_mismatch_" + label)
                rec = M.run_job(dict(j), log_path=log, closure=launch.for_run(lambda s=snapshot: s) | {"job_inputs": {j["job_id"]: j["job_inputs_digest"]}})
                assert rec["status"] == "failed" and rec["returncode"] is None and "differs from the launch manifest" in rec["status_reason"], (label, rec["status_reason"])
            for label, drift in (("environment_fingerprint", fake_snapshot(env_version="3.11.0")), ("orchestration", fake_snapshot(orch_sha=H1))):
                seq = iter([fake_snapshot(), drift])
                j = job("post_drift_" + label)
                rec = M.run_job(dict(j), log_path=log, closure=launch.for_run(lambda: next(seq)) | {"job_inputs": {j["job_id"]: j["job_inputs_digest"]}})
                assert rec["status"] == "failed" and rec["closure_sections_post_equal_pre"][label] is False

            def broken():
                raise CL.ClosureError("synthetic missing closure file")

            rec = M.run_job(job("pre_error"), log_path=log, closure=launch.for_run(broken))
            assert rec["status"] == "failed" and rec["returncode"] is None and "missing closure file" in rec["closure_error"]
            ok("run_job closure: section mismatch before launch -> not launched; drift during the job -> failed; closure error -> not launched")
            j = job("symlink_module")
            link_file = bundle["module"] / "extra_link.pkl"
            link_file.symlink_to(bundle["module"] / "module_state.pkl")
            try:
                rec = M.run_job(dict(j), log_path=log, closure=bind(j, launch))
            finally:
                link_file.unlink()
            assert rec["status"] == "failed" and rec["returncode"] is None and "job inputs could not be resolved" in rec["status_reason"] and "symlink" in rec["closure_error"]
            expect_error(lambda: M.run_job(job("noclosure"), log_path=log, closure=None), "run_job without closure", "requires a contemporaneous closure")
            ok("run_job: symlink inside the module directory -> not launched; no closure -> RuntimeError")

            # closure manifest ON DISK before/after the subprocess
            def fresh_closure(name: str, job_rec: dict) -> Closure:
                return Closure(tmp_root / name, snap, {job_rec["job_id"]: job_rec["job_inputs_digest"]})

            def assert_not_launched(rec: dict, marker: Path, j: dict, needle: str, label: str) -> None:
                assert rec["status"] == "failed" and rec["returncode"] is None and "closure manifest on disk missing or tampered before launch" in rec["status_reason"], (label, rec["status_reason"])
                assert rec["closure_manifest_verified_pre_launch"]["ok"] is False and needle in json.dumps(rec["closure_manifest_verified_pre_launch"]["problems"]), (label, rec["closure_manifest_verified_pre_launch"])
                assert rec["closure_manifest_unchanged"] is False and not marker.exists() and not (C.REPO / j["output_dir"] / "f0_driver_stdout.log").exists(), label

            for tag, prepare, needle in (
                ("m_pre_delete", lambda cl: (C.REPO / cl.rel).unlink(), "closure_manifest_file"),
                ("m_pre_tamper", lambda cl: (C.REPO / cl.rel).open("ab").write(b"\n"), "closure_manifest_sha256"),
            ):
                j = job(tag)
                cl = fresh_closure("launch_" + tag, j)
                prepare(cl)
                marker = tmp_root / (tag + ".marker")
                with fake_effects(marker=marker):
                    assert_not_launched(M.run_job(dict(j), log_path=log, closure=cl.for_run(lambda: fake_snapshot())), marker, j, needle, tag)
            j = job("m_pre_unbound")
            cl = Closure(tmp_root / "launch_m_pre_unbound", snap, {"another_job": j["job_inputs_digest"]})
            cl.bindings = {j["job_id"]: j["job_inputs_digest"]}
            marker = tmp_root / "m_pre_unbound.marker"
            with fake_effects(marker=marker):
                assert_not_launched(M.run_job(dict(j), log_path=log, closure=cl.for_run(lambda: fake_snapshot())), marker, j, "closure_manifest_job_inputs_binding", "pre: binding absent")
            j = job("m_pre_symlink")
            cl = fresh_closure("launch_m_pre_symlink", j)
            real_path = C.REPO / cl.rel
            copy_path = real_path.with_name("copy.json")
            copy_path.write_bytes(real_path.read_bytes())
            real_path.unlink()
            real_path.symlink_to(copy_path)
            marker = tmp_root / "m_pre_symlink.marker"
            with fake_effects(marker=marker):
                assert_not_launched(M.run_job(dict(j), log_path=log, closure=cl.for_run(lambda: fake_snapshot())), marker, j, "symlink", "pre: manifest symlink")
            j = job("m_pre_content")
            cl = fresh_closure("launch_m_pre_content", j)
            payload = C.read_json(C.REPO / cl.rel)
            payload["native_plugins"][0]["sha256"] = H1
            (C.REPO / cl.rel).write_text(json.dumps(payload), encoding="utf-8")
            cl.sha = C.sha256_file(C.REPO / cl.rel)
            marker = tmp_root / "m_pre_content.marker"
            with fake_effects(marker=marker):
                assert_not_launched(M.run_job(dict(j), log_path=log, closure=cl.for_run(lambda: fake_snapshot())), marker, j, "closure_manifest_runtime_digest", "pre: table rewritten")
            ok("closure manifest on disk before launch: deleted, tampered, binding absent, symlink, table rewritten -> not launched (marker and log absent)")
            for tag, effect, needle in (("m_post_tamper", "tamper_manifest", "closure_manifest_sha256"), ("m_post_delete", "delete_manifest", "closure_manifest_file")):
                j = job(tag)
                cl = fresh_closure("launch_" + tag, j)
                marker = tmp_root / (tag + ".marker")
                with fake_effects(marker=marker, **{effect: C.REPO / cl.rel}):
                    rec = M.run_job(dict(j), log_path=log, closure=cl.for_run(lambda: fake_snapshot()))
                assert rec["status"] == "failed" and rec["returncode"] == 0 and marker.exists() and rec["closure_manifest_verified_pre_launch"]["ok"] is True and rec["closure_manifest_verified_post"]["ok"] is False and rec["closure_manifest_unchanged"] is False and needle in json.dumps(rec["closure_manifest_verified_post"]["problems"]), (tag, rec["status_reason"])
                expect_error(lambda: M.verify_existing(job(tag), C.REPO / j["output_dir"]), f"{tag} receipt rejected by verification", "status")
            ok("closure manifest on disk after the subprocess: tampered or deleted during the job -> failed despite rc 0, receipts rejected")

            # --- preflight atomic + closure handed to the runner ----------------------------------
            make_v5(out_dir, fresh, closure)
            make_v4(out_b, fresh_b)
            report = M.preflight_all([fresh, fresh_b])
            assert report["invalid"] == {} and len(report["verified_identical"]) == 2 and report["verified_by_branch"] == {M.BRANCH_LEGACY_V4: 1, M.BRANCH_CONTEMPORANEOUS_V5: 1}
            write_outputs(out_b, {"ok": False})
            invoked: list = []

            def fake_runner(rec, *, log_path, closure=None):
                invoked.append((rec["job_id"], closure))
                return dict(rec, status="ok")

            launch_dict = closure.for_run(lambda: snap)
            expect_error(lambda: M.execute_matrix([fresh, fresh_b], [fresh, fresh_b], workers=2, log_path=tmp_root / "log2.jsonl", runner=fake_runner, closure=launch_dict), "execute_matrix with one invalid existing output", "preflight failed")
            assert invoked == [] and not (tmp_root / "log2.jsonl").exists()
            make_v4(out_b, fresh_b)
            outcome = M.execute_matrix([fresh, fresh_b], [fresh, fresh_b], workers=2, log_path=tmp_root / "log2.jsonl", runner=fake_runner, closure=launch_dict)
            assert sorted(i[0] for i in invoked) == sorted([fresh["job_id"], fresh_b["job_id"]]) and all(i[1] is launch_dict for i in invoked) and len(outcome["results"]) == 2
            make_v5(out_n, fresh_n, closure)
            M.verify_existing(fresh_n, out_n)
            make_v5(out_n, fresh_n, closure, historical_reference_summary_sha256="f" * 64)
            expect_error(lambda: M.verify_existing(fresh_n, out_n), "job without reference but receipt carries a digest", "historical_reference_summary_sha256")
            for label, overrides in (("status failed", {"status": "failed"}), ("returncode 1", {"returncode": 1}), ("summary sha None", {"summary_sha256": None, "trace_sha256": None})):
                make_v5(out_dir, fresh, closure, **overrides)
                expect_error(lambda: M.verify_existing(fresh, out_dir), label, None)
            for field in M.RECEIPT_VERIFY_FIELDS:
                bad_value = "DIVERGENT" if not isinstance(fresh[field], list) else fresh[field] + ["--extra"]
                make_v5(out_dir, fresh, closure, **{field: bad_value})
                expect_error(lambda: M.verify_existing(fresh, out_dir), f"divergent {field}", None)
            good = M.output_digest(out_dir)
            assert M.status_from(0, good) == "ok" and M.status_from(1, good) == "failed" and "summary_fields" in good and "action_selection" not in good
            ok("preflight atomic, closure handed to the runner, reference digest, status/returncode/digest/provenance rejections, summary-derived identity fields never overwrite the receipt")
        assert C.REPO == REAL_REPO and C.OUT_ROLLOUTS == REAL_OUT_ROLLOUTS and C.ROLLOUT_EVAL == REAL_ROLLOUT_EVAL and C.CANDIDATES[spec_job[1]]["module"] != bundle["module"] and C.RUNTIMES[spec_job[2]]["config"] != bundle["config"]
        ok("synthetic root restored (real REPO / OUT_ROLLOUTS / rollout_eval / module / config back in place)")

        # --- the 58 real descriptions and the 28 real schema-4 receipts (read-only) ------------------
        interpreter = C.select_python()
        real = [M.describe_job(j, interpreter["selected"], None) for j in jobs]
        assert len(real) == 58 and all(M.command_binding_problems(d) == {} for d in real)
        real_report = M.preflight_all(real)
        existing = sum(1 for d in real if d["output_dir_exists"])
        assert real_report["checked_existing"] == existing == len(real_report["verified_identical"]) and real_report["invalid"] == {}, real_report["invalid"]
        branches = real_report["verified_by_branch"]
        assert branches[M.BRANCH_LEGACY_V4] == 28 and branches[M.BRANCH_LEGACY_V4] + branches[M.BRANCH_CONTEMPORANEOUS_V5] == existing, branches
        stored_real = [C.read_json(C.REPO / d["output_dir"] / M.RECEIPT_FILE) for d in real if d["output_dir_exists"]]
        assert len(stored_real) == existing and sum(1 for r in stored_real if r["schema_version"] == 4) == 28 and all(r["schema_version"] in (4, 5) and M.command_binding_problems(r, require_fingerprint=(r["schema_version"] == 5)) == {} for r in stored_real)
        ok(f"58/58 real descriptions bound to the canonical matrix (RAW argv, canonical dirs); {existing}/{existing} stored receipts accepted (28 schema 4 legacy + {branches[M.BRANCH_CONTEMPORANEOUS_V5]} schema 5 contemporaneous, full binding incl. fingerprint for schema 5); nothing launched")
        sample = [os.path.join(os.path.abspath(os.sep), "a", "b", "..", "c"), "--flag", "x"]
        assert M.normalize_command(sample) == [os.path.normpath(sample[0]), "--flag", "x"]
        ok("normalize_command == os.path.normpath on absolute tokens (descriptive integrity field only)")
        assert str(tmp_root).startswith(base) and not str(tmp_root).startswith(str(REAL_REPO))
        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        for key in FAKE_ENV_KEYS.values():
            os.environ.pop(key, None)
        C.REPO, C.OUT_ROLLOUTS, C.ROLLOUT_EVAL = REAL_REPO, REAL_OUT_ROLLOUTS, REAL_ROLLOUT_EVAL
        shutil.rmtree(tmp_root, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
