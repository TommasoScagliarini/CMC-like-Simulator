"""Fail-closed tests for the V26C J7 materialisation layer.

The suite proves the preflight writes nothing at all, that every guard fails closed, and that a
real materialisation - executed ONLY into a temporary root through the declared override - lands
exactly two files, round-trips, and never touches the authorised leaf.

No fit, no environment, no rollout, no critic, no PPO. No existing artefact is modified.
"""
from __future__ import annotations
import ast, builtins, io, json, os, shutil, sys, tempfile, tokenize, types
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j7_materialize as M  # noqa: E402
import v26c_j7_markov_dataset as J7B  # noqa: E402

CHECKS = 0


def check(c, w):
    global CHECKS
    assert c, w
    CHECKS += 1


def expect(fn, exc, w):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def snapshot(root: Path) -> dict[str, str]:
    """Every file under root, by hash, ignoring bytecode caches."""
    out: dict[str, str] = {}
    for p in sorted(root.rglob("*")):
        if "__pycache__" in p.parts or not p.is_file():
            continue
        out[str(p.relative_to(root))] = M._sha_file(p)
    return out


def main() -> int:
    src = (HERE / "v26c_j7_materialize.py").read_text()
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    auth_json = json.loads(M.AUTHORIZATION.read_text())

    # ---------------------------------------------------------------- the authorisation ---------
    check(auth_json["kind"] == "ADDITIVE, IMMUTABLE, FORMAL AUTHORISATION"
          and auth_json["stage_authorised"] == "V26C-J7-MATERIALIZE" == M.STAGE,
          "the authorisation is additive, immutable and names the materialisation stage")
    check(M._sha_file(M.AUTHORIZATION) == M.PIN_AUTHORIZATION
          == "a64cf7348705528cb84638143ca00f481b9c8ef432146a4989b09ad09a62383f",
          "and is pinned by exact hash")
    check(all(isinstance(v, str) and len(v) == 64 for v in auth_json["frozen_inputs"].values()),
          "frozen_inputs is a pure path -> sha256 map, with no prose mixed in")
    frozen = auth_json["frozen_inputs"]
    check(frozen["v26c_j7_prereg_markov_dataset.json"]
          == "bea97f268d2080968954cd694c7244cc3dc9d1f40ea4ae4ec79e9726fa9edf96"
          and frozen["v26c_j7_markov_dataset.py"]
          == "b3cc2b88997ccb7c23964123d70ac3fb524adf4fdf3e8882d26ce20ff87d1461"
          and frozen["test_v26c_j7_markov_dataset.py"]
          == "aca18c3f73f91d8be9fc3621a5e33b97d400ee7ef3d3b9c825db0db2bbd32924",
          "it pins the three frozen J7 artefacts at the architect's hashes")
    check(all(M._sha_file(HERE / rel) == sha for rel, sha in frozen.items()),
          "MEASURED: all three verify on disk right now")
    check(dict(M.FROZEN) == frozen,
          "and the materializer's own constants agree with the authorisation")
    d = auth_json["destination"]
    check(d["relative_leaf"] == "j7_runs/j7_markov_dataset_v26c_2026-08-26_r1" == M.RELATIVE_LEAF
          and d["is_the_only_destination"] is True,
          "the single destination is the authorised relative leaf")
    check(d["must_not_exist_before"] is True and d["must_not_be_a_symlink"] is True
          and d["no_clobber"] is True, "which must not pre-exist, must not be a symlink, no-clobber")
    check(d["write_protocol"].startswith("atomic commit, no-clobber under exclusive materializer "
                                         "lock"),
          "the protocol is stated as an atomic commit under an exclusive materializer lock")
    lk = d["lock"]
    check(lk["file"] == ".lock_j7_markov_dataset_v26c_2026-08-26_r1" == M.LOCK_NAME
          and "O_CREAT | O_EXCL" in lk["acquisition"],
          "the lock is a sibling file taken with O_CREAT|O_EXCL")
    check(lk["serialises"] == "materializer instances, and only those"
          and "ignores the lock" in lk["does_not_guarantee"]
          and "empty directory" in lk["does_not_guarantee"].lower(),
          "and the authorisation states the LIMIT: os.rename replaces an empty directory, so "
          "nothing is claimed against a writer that ignores the lock")
    check(lk["if_it_pre_exists"].startswith("FAIL CLOSED")
          and "removes no lock it does not own" in lk["if_it_pre_exists"].lower(),
          "a pre-existing lock is a fail-closed stop, never an autonomous removal")
    cl = d["cleanup_on_failure"]
    check("ONLY on the exact staging directory this run created" in cl["staging"]
          and "NON-RECURSIVE rmdir" in cl["parent"]
          and "only if it is empty" in cl["parent"],
          "cleanup: rmtree on the staging alone, at most a non-recursive rmdir of the parent")
    check("no recursive removal of j7_runs" in cl["never"]
          and "appeared concurrently survives" in cl["never"],
          "and concurrent content is guaranteed to survive")
    bs = auth_json["bootstrap"]
    check("BEFORE it is imported or executed" in bs["rule"]
          and "spec_from_file_location" in bs["loading"]
          and "Never an import by name" in bs["loading"],
          "the authorisation requires authentication BEFORE import/exec, and loading by path")
    check(bs["preloaded_module"].startswith("FAIL CLOSED")
          and "even when its __file__ and hash match" in bs["preloaded_module"]
          and "There is no reuse branch" in bs["preloaded_module"],
          "and it requires a fail-closed refusal of ANY pre-loaded module of that name")
    o = auth_json["authorised_outputs"]
    check(o["dataset"] == M.DATASET_NAME == "v26c_j7_markov_recovery_dataset.npz"
          and o["receipt"] == M.RECEIPT_NAME == "v26c_j7_markov_dataset_receipt.json"
          and o["count"] == 2, "and the only outputs are the Markov NPZ and the receipt JSON")
    dc = auth_json["dataset_contract"]
    check(tuple(dc["keys"]) == M.DATASET_KEYS
          == ("observations", "actions", "actor_feature_names")
          and dc["no_other_key"] is True, "the NPZ carries exactly three keys")
    check(tuple(dc["observations_shape"]) == (16713, 35) == M.EXPECTED_OBS_SHAPE
          and tuple(dc["actions_shape"]) == (16713, 2) == M.EXPECTED_ACT_SHAPE
          and dc["dtype"] == "float32" and dc["actor_feature_names_count"] == 35,
          "16713 x 35 and 16713 x 2, float32, 35 feature names")
    check(dc["content_hash_observations"] == M.EXPECTED_OBS_SHA
          == "a59fb89b43a372162661ec8cf131b5abbf1e9ce32b16f98518af9010a4cf587a"
          and dc["content_hash_actions"] == M.EXPECTED_ACT_SHA
          == "6e920c2f890dbdc2cfacf6a2c6842b34c6f33e24499d0d8716bd6347624bde4f",
          "pinned to the audited content hashes")
    check(dc["allow_pickle"] is False and auth_json["receipt_contract"]["allow_nan"] is False,
          "no pickle in the dataset, no NaN in the receipt")
    for forbidden in ("fit, adapt or train anything", "run any rollout",
                      "touch the critic or run PPO",
                      "modify FSM, morphology, reward, SEA or the C++ plugin",
                      "modify the J2 parent or any existing artefact"):
        check(forbidden in auth_json["this_stage_does_not"],
              f"the authorisation forbids: {forbidden}")
    check(auth_json["outcome_policy"] == {"deployable": False, "promotion": "NONE",
                                          "next_stage_authorized": False, "single_execution": True,
                                          "no_autonomous_retry": "on failure the evidence is "
                                                                 "preserved and the run stops"},
          "it confers no deployability, no promotion and no next stage")

    # ---------------------------------------------------------------- imported builder ----------
    a = M.verify_authorization()
    check(Path(J7B.__file__).resolve() == M.BUILDER.resolve()
          and M._sha_file(Path(J7B.__file__)) == M.FROZEN["v26c_j7_markov_dataset.py"],
          "the imported builder IS the frozen file, at the frozen hash")
    check(a["imported_builder"].endswith("v26c_j7_markov_dataset.py")
          and a["relative_leaf"] == M.RELATIVE_LEAF,
          "and verify_authorization reports it")
    check(J7B.PIN_PREREG == M.FROZEN["v26c_j7_prereg_markov_dataset.json"],
          "the frozen builder pins the frozen preregistration")
    check("assemble" in ids and "def assemble" not in src,
          "the materializer CALLS assemble() and does not redefine it")
    check(J7B is M.J7B,
          "the builder this suite imports IS the one the materializer authenticated")

    # ------------------------------------------------------- AUTHENTICATE, THEN EXECUTE ---------
    # STATIC: the builder never enters through an import statement, and inside the loader the
    # hash comparison and its raise both precede exec_module.
    tree = ast.parse(src)
    imported_names = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported_names |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check("v26c_j7_markov_dataset" not in imported_names,
          f"STATIC: the builder is never reached by an import statement ({sorted(imported_names)})")
    loader = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "load_verified_builder")
    sha_lines = [n.lineno for n in ast.walk(loader) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == "_sha_file"]
    raise_lines = [n.lineno for n in ast.walk(loader) if isinstance(n, ast.Raise)
                   and "changed" in ast.dump(n)]
    exec_lines = [n.lineno for n in ast.walk(loader) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute) and n.func.attr == "exec_module"]
    check(sha_lines and raise_lines and exec_lines, "the loader hashes, refuses and execs")
    check(min(sha_lines) < min(raise_lines) < min(exec_lines),
          f"STATIC ORDER: hash (line {min(sha_lines)}) -> refuse (line {min(raise_lines)}) -> "
          f"exec (line {min(exec_lines)})")
    check(M.BOOTSTRAP_TRACE[0].startswith("verify:v26c_j7_markov_dataset:"),
          "DYNAMIC: the very first bootstrap step was the verification")
    check(M.BOOTSTRAP_TRACE[0].endswith(M.FROZEN["v26c_j7_markov_dataset.py"]),
          "against the frozen hash")
    execs = [i for i, s in enumerate(M.BOOTSTRAP_TRACE) if s.startswith("exec:")]
    verifies = [i for i, s in enumerate(M.BOOTSTRAP_TRACE) if s.startswith("verify:")]
    check(not execs or min(verifies) < min(execs),
          f"and every execution followed a verification ({M.BOOTSTRAP_TRACE})")
    check(a["bootstrap"]["authenticated_before_exec"] is True
          and a["bootstrap"]["trace"] == M.BOOTSTRAP_TRACE,
          "verify_authorization reports the bootstrap order it observed")
    check(a["bootstrap"]["reused_a_preloaded_module"] is False
          and "FAIL CLOSED" in a["bootstrap"]["preloaded_module_policy"],
          "and declares that it reused no pre-loaded module")
    check(M.BOOTSTRAP_TRACE[1].startswith(f"exec:{M.BUILDER_MODULE_NAME}:")
          and not any(s.startswith("reuse") for s in M.BOOTSTRAP_TRACE),
          "the clean-process bootstrap is exactly verify then exec, with no reuse step")

    # DYNAMIC PROOF: a probe module whose mere execution leaves a mark. With a wrong hash the
    # mark must NOT appear - that is the difference between checking before and checking after.
    probe_dir = Path(tempfile.mkdtemp(prefix="v26c_j7_bootstrap_probe_"))
    try:
        mark = probe_dir / "executed.txt"
        probe = probe_dir / "probe_builder.py"
        probe.write_text("from pathlib import Path\n"
                         f"Path({str(mark)!r}).write_text('executed')\n", encoding="utf-8")
        probe_sha = M._sha_file(probe)
        before_trace = len(M.BOOTSTRAP_TRACE)
        e = expect(lambda: M.load_verified_builder(probe, "0" * 64, "v26c_probe_rejected"),
                   M.MaterializeError, "a builder whose hash does not match must be refused")
        check("Refusing to import or execute it" in str(e), "and the refusal says so explicitly")
        check(not mark.exists(),
              "PROVEN: the rejected module was NEVER executed - authentication precedes exec")
        check("v26c_probe_rejected" not in sys.modules,
              "and it was never registered in sys.modules")
        new = M.BOOTSTRAP_TRACE[before_trace:]
        check(len(new) == 1 and new[0].startswith("verify:v26c_probe_rejected:"),
              f"the trace records the verification and no exec ({new})")
        # the same probe with its true hash DOES execute: the detector is valid, not inert
        M.load_verified_builder(probe, probe_sha, "v26c_probe_accepted")
        check(mark.is_file() and mark.read_text() == "executed",
              "with the correct hash the probe executes: the detector really detects execution")
        # ---- FAIL CLOSED on ANY pre-loaded module of that name. No reuse branch exists. ----
        # (i) a namesake bound to a DIFFERENT file
        ghost = types.ModuleType("v26c_probe_namesake")
        ghost.__file__ = str(probe_dir / "somewhere_else.py")
        sys.modules["v26c_probe_namesake"] = ghost
        try:
            mark.unlink()
            before_trace = len(M.BOOTSTRAP_TRACE)
            e = expect(lambda: M.load_verified_builder(probe, probe_sha, "v26c_probe_namesake"),
                       M.MaterializeError, "a namesake from another path must be refused")
            check("already present in sys.modules" in str(e)
                  and "somewhere_else.py" in str(e),
                  "the refusal names where the pre-loaded module actually came from")
            check(not mark.exists(),
                  "PROVEN: the loader executed nothing on the namesake path")
            new = M.BOOTSTRAP_TRACE[before_trace:]
            check(all(not s.startswith("exec:") for s in new)
                  and any(s.startswith("refuse-preloaded:v26c_probe_namesake:") for s in new),
                  f"and the trace records a refusal, never an exec ({new})")
            check(sys.modules["v26c_probe_namesake"] is ghost,
                  "the pre-loaded module was left exactly as it was found")
        finally:
            sys.modules.pop("v26c_probe_namesake", None)

        # (ii) the SAME EXACT FILE, already loaded, with a matching hash: still refused
        M.load_verified_builder(probe, probe_sha, "v26c_probe_twice")
        check(mark.is_file(), "the first load of a clean name executes normally")
        try:
            mark.unlink()
            before_trace = len(M.BOOTSTRAP_TRACE)
            e = expect(lambda: M.load_verified_builder(probe, probe_sha, "v26c_probe_twice"),
                       M.MaterializeError,
                       "the same file, already loaded, must be refused too")
            check("including one whose path and hash match" in str(e)
                  and "identity is not provenance" in str(e),
                  "the refusal states WHY a matching path and hash are not enough")
            check(not mark.exists(),
                  "PROVEN: no second execution - the loader refused before exec_module")
            new = M.BOOTSTRAP_TRACE[before_trace:]
            check(all(not s.startswith("exec:") for s in new),
                  f"and the trace shows no exec at all ({new})")
        finally:
            sys.modules.pop("v26c_probe_twice", None)

        # (iii) the real builder's own name, already loaded by this process's bootstrap
        e = expect(lambda: M.load_verified_builder(M.BUILDER,
                                                   M.FROZEN["v26c_j7_markov_dataset.py"],
                                                   M.BUILDER_MODULE_NAME),
                   M.MaterializeError,
                   "even the frozen builder itself is refused once it is already loaded")
        check("already present in sys.modules" in str(e),
              "so a second bootstrap in a dirty process cannot silently reuse it")
        check("reuse:" not in src and "return existing" not in src,
              "STATIC: there is no reuse branch in the loader at all")
    finally:
        shutil.rmtree(probe_dir, ignore_errors=True)

    # ---------------------------------------------------------------- preflight is read-only ----
    before = snapshot(HERE)
    pre = M.preflight()
    check(pre["verdict"] == "GO" and pre["blockers"] == [] and pre["read_only"] is True,
          "the preflight is GO and declares itself read-only")
    check(snapshot(HERE) == before,
          "MEASURED: not one byte under the validation root changed during the preflight")
    check(not M.authorized_leaf().exists()
          and not (M.authorized_leaf().parent / M.STAGING_NAME).exists(),
          "no leaf and no staging directory exist after it")
    check(pre["inert"] == {"leaf_created": False, "staging_created": False,
                           "dataset_written": False, "receipt_written": False,
                           "rename_performed": False, "fit_executed": False,
                           "environment_constructed": False, "rollout_executed": False,
                           "critic_touched": False, "ppo_updates": 0,
                           "note": "the dataset is assembled in memory, audited and discarded"},
          "and it declares every write and every heavy operation absent")
    check(pre["payload_audit"]["content_hashes"] == {"observations": M.EXPECTED_OBS_SHA,
                                                     "actions": M.EXPECTED_ACT_SHA}
          and pre["payload_audit"]["content_hashes_match_audited"] is True,
          "the in-memory payload reproduces the audited content hashes")
    check(pre["would_write"]["files"] == [M.DATASET_NAME, M.RECEIPT_NAME]
          and pre["would_write"]["relative_leaf"] == M.RELATIVE_LEAF
          and pre["would_write"]["output_root_override"] is None,
          "it names what it WOULD write, at the authorised leaf, with no override active")

    # DYNAMIC: every write primitive is armed to explode during a second preflight
    tripped: list[str] = []
    real = {"rename": os.rename, "replace": os.replace, "savez": np.savez,
            "savez_compressed": np.savez_compressed, "mkdir": Path.mkdir,
            "write_text": Path.write_text, "write_bytes": Path.write_bytes,
            "open": builtins.open, "rmtree": shutil.rmtree}

    def boom(tag):
        def f(*args, **kwargs):  # noqa: ANN001
            tripped.append(tag)
            raise AssertionError(f"the preflight attempted {tag}")
        return f

    def guarded_open(file, mode="r", *args, **kwargs):  # noqa: ANN001
        if any(m in str(mode) for m in ("w", "a", "x", "+")):
            tripped.append(f"open({mode})")
            raise AssertionError(f"the preflight opened {file} for writing")
        return real["open"](file, mode, *args, **kwargs)

    os.rename, os.replace = boom("os.rename"), boom("os.replace")
    np.savez, np.savez_compressed = boom("np.savez"), boom("np.savez_compressed")
    Path.mkdir, Path.write_text = boom("Path.mkdir"), boom("Path.write_text")
    Path.write_bytes, shutil.rmtree = boom("Path.write_bytes"), boom("shutil.rmtree")
    builtins.open = guarded_open
    banned = {"torch", "ray", "env_factory", "opensim", "rollout_eval", "gymnasium"}
    real_import = builtins.__import__

    def guarded_import(name, *ar, **kw):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(f"import {name}")
            raise AssertionError(f"the preflight imported {name}")
        return real_import(name, *ar, **kw)

    builtins.__import__ = guarded_import
    try:
        pre2 = M.preflight()
    finally:
        os.rename, os.replace = real["rename"], real["replace"]
        np.savez, np.savez_compressed = real["savez"], real["savez_compressed"]
        Path.mkdir, Path.write_text = real["mkdir"], real["write_text"]
        Path.write_bytes, shutil.rmtree = real["write_bytes"], real["rmtree"]
        builtins.open, builtins.__import__ = real["open"], real_import
    check(tripped == [], f"PROVEN: the preflight called no write primitive at all ({tripped})")
    check(pre2["payload_audit"]["content_hashes"] == pre["payload_audit"]["content_hashes"],
          "and two preflights audit identical content: the payload is deterministic")
    e = expect(lambda: M.main(["--out", "/tmp/anything"]), M.MaterializeError,
               "--out without --materialize must be refused")
    check("meaningless without --materialize" in str(e), "with a reason")

    # ---------------------------------------------------------------- fail-closed guards --------
    leaf = M.authorized_leaf()
    e = expect(lambda: M.materialize(str(leaf), None), M.MaterializeError,
               "a missing stage token must be refused")
    check("--authorized-stage must be exactly" in str(e), "naming the exact token required")
    e = expect(lambda: M.materialize(str(leaf), "V26C-J7-MARKOV-DATASET"), M.MaterializeError,
               "the J7 build token must not authorise a materialisation")
    check("V26C-J7-MATERIALIZE" in str(e), "the two stage tokens are distinct")
    for bad in ("v26c-j7-materialize", "V26C-J7-MATERIALIZE ", "", "MATERIALIZE"):
        expect(lambda b=bad: M.materialize(str(leaf), b), M.MaterializeError,
               f"the stage token {bad!r} must be refused")
    e = expect(lambda: M.materialize(None, M.STAGE), M.MaterializeError,
               "--materialize without --out must be refused")
    check("requires --out" in str(e), "and say so")
    for bad_out in (str(HERE), str(HERE / "j7_runs"), str(HERE / "j7_runs" / "other_leaf"),
                    str(HERE / "elsewhere" / "j7_markov_dataset_v26c_2026-08-26_r1")):
        e = expect(lambda b=bad_out: M.materialize(b, M.STAGE), M.MaterializeError,
                   f"--out {bad_out} is not the authorised leaf")
        check("not the authorised leaf" in str(e), f"refused: {Path(bad_out).name}")
    check(snapshot(HERE) == before and not leaf.exists(),
          "MEASURED: every refusal above wrote nothing")

    # tampering with any pinned hash must fail closed
    saved = dict(M.FROZEN)
    try:
        M.FROZEN["v26c_j7_markov_dataset.py"] = "0" * 64
        e = expect(M.verify_authorization, M.MaterializeError, "a builder hash mismatch")
        check("changed" in str(e), "a changed builder is caught")
    finally:
        M.FROZEN.clear()
        M.FROZEN.update(saved)
    saved_pin = M.PIN_AUTHORIZATION
    try:
        M.PIN_AUTHORIZATION = "0" * 64
        e = expect(M.verify_authorization, M.MaterializeError, "an authorisation hash mismatch")
        check("the authorisation changed" in str(e), "a changed authorisation is caught")
    finally:
        M.PIN_AUTHORIZATION = saved_pin
    saved_sha = M.EXPECTED_OBS_SHA
    try:
        M.EXPECTED_OBS_SHA = "0" * 64
        e = expect(M.build_payload, M.MaterializeError, "a content-hash mismatch")
        check("not the audited" in str(e), "a dataset that is not the audited one is caught")
    finally:
        M.EXPECTED_OBS_SHA = saved_sha
    saved_builder = M.BUILDER
    try:
        M.BUILDER = HERE / "v26c_j7_materialize.py"
        e = expect(M.verify_authorization, M.MaterializeError, "a builder identity mismatch")
        check("not the frozen" in str(e), "an imported module that is not the frozen builder")
    finally:
        M.BUILDER = saved_builder
    check(M.verify_authorization()["sha256"] == M.PIN_AUTHORIZATION,
          "and every constant is restored afterwards")

    # ---------------------------------------------------------------- a REAL materialisation ----
    # Executed only into a temporary root, through the declared override. The authorised leaf is
    # never created by this suite.
    saved_root = M.OUTPUT_ROOT_OVERRIDE
    tmp = Path(tempfile.mkdtemp(prefix="v26c_j7_materialize_test_"))
    try:
        M.OUTPUT_ROOT_OVERRIDE = tmp
        target = M.authorized_leaf()
        check(target == tmp / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1",
              "the override redirects the ROOT only; the relative leaf is unchanged")

        lock = target.parent / M.LOCK_NAME

        # (a) clean failure: the run created the parent, the lock and the staging, and nothing
        # else appeared. All three go, because all three are its own.
        np.savez = boom("np.savez")
        try:
            expect(lambda: M.materialize(str(target), M.STAGE), AssertionError,
                   "a failed write must propagate")
        finally:
            np.savez = real["savez"]
        check(not target.exists(), "the leaf was NOT created by the failed run")
        check(not (target.parent / M.STAGING_NAME).exists(),
              "and the staging directory it created was removed")
        check(not lock.exists(), "and it released its own lock")
        check(not (tmp / "j7_runs").exists(),
              "including the parent it created, which was empty: cleanup removes only what the "
              "run made")

        # (b) CONCURRENT CONTENT: the run creates the parent, something else drops a file into it
        # mid-write, then the write fails. The sentinel and the parent MUST survive.
        sentinel = tmp / "j7_runs" / "concurrent_sentinel.txt"

        def savez_then_concurrent_write(path, **kw):  # noqa: ANN001
            Path(path).parent.parent.joinpath("concurrent_sentinel.txt").write_text(
                "written by another run", encoding="utf-8")
            raise AssertionError("forced failure after the parent, lock and staging exist")

        np.savez = savez_then_concurrent_write
        try:
            expect(lambda: M.materialize(str(target), M.STAGE), AssertionError,
                   "the forced failure must propagate")
        finally:
            np.savez = real["savez"]
        check(sentinel.is_file() and sentinel.read_text() == "written by another run",
              "PROVEN: the concurrent sentinel SURVIVED the failed run's cleanup")
        check((tmp / "j7_runs").is_dir(),
              "and so did the parent the run created: rmdir is non-recursive and refused to "
              "delete a non-empty directory")
        check(not (target.parent / M.STAGING_NAME).exists(),
              "only the exact staging directory was removed recursively")
        check(not lock.exists(), "the run released its own lock")
        check(not target.exists(), "and the leaf was never created")

        # (c) the LOCK is exclusive and fail-closed. A lock this run does not own is never
        # removed, whether it looks live or stale.
        live = json.dumps({"stage": M.STAGE, "pid": 999999, "leaf": str(target)})
        lock.write_text(live, encoding="utf-8")
        e = expect(lambda: M.materialize(str(target), M.STAGE), M.MaterializeError,
                   "a held lock must stop a second instance")
        check("lock already exists" in str(e) and "removes no lock it does not own" in str(e),
              "the refusal names the lock and refuses to remove it")
        check(lock.read_text() == live,
              "MEASURED: the foreign lock was neither removed nor rewritten")
        check(not (target.parent / M.STAGING_NAME).exists() and not target.exists(),
              "and the blocked instance staged nothing")
        stale = "garbage left by an interrupted run"
        lock.write_text(stale, encoding="utf-8")
        e = expect(lambda: M.materialize(str(target), M.STAGE), M.MaterializeError,
                   "a stale lock must stop the run too")
        check(lock.read_text() == stale,
              "a STALE lock is treated identically: no heuristic, no autonomous removal")
        pf = M.preflight()
        check(pf["verdict"] == "BLOCKED"
              and any("lock" in b for b in pf["blockers"]),
              f"and the read-only preflight reports it as a blocker ({pf['blockers']})")
        lock.unlink()                      # the operator's manual action, never the tool's
        check(M.preflight()["verdict"] == "GO", "once removed by hand, the preflight is GO again")

        res = M.materialize(str(target), M.STAGE)
        check(res["verdict"] == "MATERIALIZED" and res["clobbered_nothing"] is True
              and res["staging_removed"] is True, "the materialisation reports success")
        check(res["authoritative"] is False,
              "and declares itself NON-authoritative: the root was overridden")
        check(sorted(p.name for p in target.iterdir()) == sorted([M.DATASET_NAME, M.RECEIPT_NAME]),
              "the leaf holds exactly the two authorised files and nothing else")
        check(not (target.parent / M.STAGING_NAME).exists(), "no staging directory survives")
        check(not lock.exists() and res["lock_released"] is True,
              "and the lock was released after the commit")
        check(res["commit"] == "atomic commit, no-clobber under exclusive materializer lock"
              and "ignores the lock" in res["commit_limit"],
              "the result describes the commit AND its limit, claiming no more than it can")
        check(sentinel.is_file(),
              "the concurrent sentinel is still there: the successful run touched nothing of its "
              "own accord")

        # round trip from the bytes on disk
        with np.load(target / M.DATASET_NAME, allow_pickle=False) as arch:
            keys = tuple(sorted(arch.files))
            obs = np.asarray(arch["observations"])
            act = np.asarray(arch["actions"])
            names = [str(n) for n in np.asarray(arch["actor_feature_names"]).tolist()]
        check(keys == tuple(sorted(M.DATASET_KEYS)),
              f"the NPZ holds exactly observations, actions, actor_feature_names ({keys})")
        check(obs.shape == (16713, 35) and act.shape == (16713, 2), "16713 x 35 and 16713 x 2")
        check(obs.dtype == np.float32 and act.dtype == np.float32, "both float32")
        check(len(names) == 35 and names == list(J7B.actor_feature_names()),
              "and the 35 feature names are the parent manifest's")
        check(J7B._sha_array(obs) == M.EXPECTED_OBS_SHA
              and J7B._sha_array(act) == M.EXPECTED_ACT_SHA,
              "ROUND TRIP: the bytes on disk reproduce the audited content hashes exactly")
        check(res["files"][M.DATASET_NAME] == M._sha_file(target / M.DATASET_NAME),
              "the reported dataset hash is the file's actual hash")

        # the receipt
        rc = json.loads((target / M.RECEIPT_NAME).read_text())
        check(rc["stage"] == M.STAGE and rc["schema"] == "v26c_j7_materialize_receipt.1",
              "the receipt names the stage")
        check(rc["authorization"]["sha256"] == M.PIN_AUTHORIZATION
              and rc["authorization"]["frozen_inputs"] == dict(M.FROZEN),
              "and records the authorisation and the three frozen input pins")
        check(rc["preregistration"]["sha256"] == J7B.PIN_PREREG
              and rc["preregistration"]["manifest_entries"] == 26,
              "and the preregistration with its 26 pins")
        check(rc["output"]["dataset_sha256"] == M._sha_file(target / M.DATASET_NAME),
              "the receipt carries the SHA256 of the dataset FILE")
        check(rc["output"]["write_protocol"]
              == "atomic commit, no-clobber under exclusive materializer lock",
              "the receipt states the protocol correctly")
        rl = rc["output"]["lock"]
        check(rl["file"] == M.LOCK_NAME and "O_CREAT | O_EXCL" in rl["acquisition"]
              and rl["serialises"] == "materializer instances, and only those",
              "and describes the lock it used")
        check("ignores the lock" in rl["does_not_guarantee"]
              and "EMPTY directory" in rl["does_not_guarantee"],
              "and does NOT claim a guarantee against writers that ignore it")
        rcl = rc["output"]["cleanup_scope"]
        check("the exact staging directory this run created" in rcl["staging"]
              and "non-recursive rmdir" in rcl["parent"]
              and "concurrent content survives" in rcl["never"],
              "and records the cleanup scope it actually applies")
        check(rc["authorization"]["bootstrap"]["authenticated_before_exec"] is True
              and rc["authorization"]["bootstrap"]["trace"][0].startswith("verify:"),
              "the receipt records that the builder was authenticated before execution")
        check(rc["content_hashes"] == {"observations": M.EXPECTED_OBS_SHA,
                                       "actions": M.EXPECTED_ACT_SHA},
              "and the ARRAY content hashes")
        check(rc["schema_of_dataset"]["observations"] == [16713, 35]
              and rc["composition"]["total_rows"] == 16713
              and rc["composition"]["nominal_rows"] == 16000
              and rc["composition"]["recovery_rows"] == 713,
              "and the schema and the composition")
        check(rc["ratios"]["recovery_over_aggregate"] == 713 / 16713 == 0.04266140130437384,
              "with the exact aggregate ratio")
        check(set(rc["penetration_per_seed"]) == {"123", "124", "125"},
              "a penetration summary per seed")
        for seed in ("123", "124", "125"):
            p = rc["penetration_per_seed"][seed]["penetration"]
            check(p["evaluated_by"] == "v26c_penetration_contract.evaluate_series"
                  and p["binding_verdict"] == "PASS" and p["counts"]["above_hard_binding"] == 0
                  and p["bit_identical_to_trace"] is True,
                  f"seed {seed}: from the contract evaluator, binding PASS, series bit-identical")
        check(rc["teacher_observations_read"] is False
              and rc["teacher_usage"]["observations_read"] is False,
              "and the confirmation that the teacher's observations were never read")
        check(rc["inert"] == {"fit_executed": False, "environment_constructed": False,
                              "rollout_executed": False, "critic_touched": False,
                              "ppo_updates": 0, "dataset_recomputed_or_altered": False,
                              "note": "the arrays written are exactly the ones assemble() "
                                      "returned"},
              "the receipt declares no fit, no environment, no rollout, no critic, no PPO")
        check(rc["output"]["authoritative"] is False
              and rc["output"]["output_root_override"] == str(tmp),
              "and stamps itself non-authoritative, naming the overridden root")
        json.dumps(rc, allow_nan=False)
        check(True, "the receipt is strictly JSON: no NaN, no Infinity")

        # no-clobber: a second run must refuse and change nothing
        after_first = snapshot(target)
        e = expect(lambda: M.materialize(str(target), M.STAGE), M.MaterializeError,
                   "a second materialisation must be refused")
        check("no-clobber and single-execution" in str(e), "because the leaf already exists")
        check(snapshot(target) == after_first, "MEASURED: the existing leaf was left untouched")
        check(not lock.exists(),
              "and the refused run left no lock behind: it never got as far as taking one")

        # a symlinked destination is refused (skipped where the OS forbids making one)
        link_root = Path(tempfile.mkdtemp(prefix="v26c_j7_materialize_link_"))
        try:
            M.OUTPUT_ROOT_OVERRIDE = link_root
            (link_root / "j7_runs").mkdir()
            try:
                (link_root / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1").symlink_to(target)
                made = True
            except (OSError, NotImplementedError):       # pragma: no cover - Windows w/o privilege
                made = False
            if made:
                e = expect(lambda: M.materialize(str(M.authorized_leaf()), M.STAGE),
                           M.MaterializeError, "a symlinked leaf must be refused")
                check("symlink" in str(e).lower(), "the refusal names the symlink")
            # a symlinked PARENT, pointing at an empty directory so nothing else can fire first
            link2 = Path(tempfile.mkdtemp(prefix="v26c_j7_materialize_link2_"))
            empty = Path(tempfile.mkdtemp(prefix="v26c_j7_materialize_empty_"))
            M.OUTPUT_ROOT_OVERRIDE = link2
            try:
                (link2 / "j7_runs").symlink_to(empty, target_is_directory=True)
                made2 = True
            except (OSError, NotImplementedError):       # pragma: no cover
                made2 = False
            if made2:
                check(not M.authorized_leaf().exists(),
                      "the leaf behind the symlinked parent does not exist, so no-clobber cannot "
                      "fire")
                e = expect(lambda: M.materialize(str(M.authorized_leaf()), M.STAGE),
                           M.MaterializeError, "a symlinked parent must be refused")
                check("symlinked path component" in str(e),
                      "a symlinked path COMPONENT is refused on its own")
                check(not any(empty.iterdir()),
                      "and nothing was written through it")
            shutil.rmtree(link2, ignore_errors=True)
            shutil.rmtree(empty, ignore_errors=True)
        finally:
            shutil.rmtree(link_root, ignore_errors=True)
    finally:
        M.OUTPUT_ROOT_OVERRIDE = saved_root
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------------------------------------------------------- the real leaf is untouched -
    check(M.OUTPUT_ROOT_OVERRIDE is None, "the override is restored to None")
    check(M.authorized_leaf() == HERE / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1",
          "and the authorised leaf resolves under the validation root again")
    check(not M.authorized_leaf().exists() and not (HERE / "j7_runs").exists(),
          "MEASURED: this suite never created the authorised leaf, nor its parent")
    check(snapshot(HERE) == before,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")

    # ---------------------------------------------------------------- static guarantees ---------
    for name in ("adapt_actor", "PPOConfig", "train_ppo", "optimizer", "Adam", "backward",
                 "critic", "make_cmc_env", "forward_exploration", "forward_inference",
                 "torch", "ray", "opensim", "gymnasium"):
        check(name not in ids, f"the materializer never references {name}")
    check("np.savez" in src and "savez_compressed" not in src,
          "it writes an uncompressed NPZ, deterministically")
    check("allow_pickle=False" in src, "and reads it back with pickling disabled")
    check("allow_nan=False" in src, "the receipt is written with allow_nan=False")
    check("os.rename" in src and "shutil.rmtree" in src,
          "it renames atomically and cleans up explicitly")
    check(src.count("shutil.rmtree") == 1,
          "there is exactly ONE recursive removal in the whole module")
    mat = next(n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == "materialize")
    rmtrees = [n for n in ast.walk(mat) if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
    check(len(rmtrees) == 1 and isinstance(rmtrees[0].args[0], ast.Name)
          and rmtrees[0].args[0].id == "staging_created",
          "and its only argument is staging_created - never the parent, never j7_runs")
    rmdirs = [n for n in ast.walk(mat) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "rmdir"]
    check(len(rmdirs) == 1 and isinstance(rmdirs[0].func.value, ast.Name)
          and rmdirs[0].func.value.id == "parent_created" and not rmdirs[0].args,
          "the parent gets a bare, non-recursive rmdir and nothing more")
    check("O_CREAT | os.O_EXCL" in src, "the lock is taken with O_CREAT|O_EXCL")
    check("parent_created" in ids and "staging_created" in ids and "lock_owned" in ids,
          "cleanup tracks the parent, the staging and the lock separately, by ownership")
    check("created.append" not in src and "reversed(created)" not in src,
          "the old undifferentiated 'created' list is gone")
    check(src.count("OUTPUT_ROOT_OVERRIDE") >= 5 and "RELATIVE_LEAF_PARTS" in src,
          "the override touches the root only; the relative leaf is a constant")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
