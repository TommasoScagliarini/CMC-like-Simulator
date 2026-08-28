"""Static test suite for V26C J20 R2 - the corrective restore audit.

Launches no child, starts no Ray, builds no Algorithm, constructs no
environment and trains nothing. It holds the R2 package to seven properties:

  * the R2 runner is a MECHANICAL COPY of the R1 runner: exactly six functions
    differ - child_environment (new), launch_once, go_pin_targets,
    expected_pin_hashes, check_entry_evidence, run_execution - and every other
    function is byte-identical, so R1's verified behaviour carries over;
  * the single correction is correct: the ABSOLUTE baseline directory is FIRST
    on PYTHONPATH, any pre-existing value is preserved verbatim, os.pathsep is
    used, and os.environ itself is never mutated;
  * that environment really reaches subprocess.Popen;
  * the REAL smoke test rev5 requires: train_ppo_mlp._worker_setup is
    serialised with Ray's own cloudpickle and deserialised in two fresh
    interpreters - without PYTHONPATH it must fail PRECISELY with
    ModuleNotFoundError naming train_ppo_mlp, the very error the 115 R1 workers
    raised, and with child_environment() it must yield a callable whose module
    and name are the expected ones. ray.init is never called, no daemon is
    started and the hook is never invoked. find_spec is kept as a cheap
    preliminary;
  * R1 is untouched: its leaf, runner, suite and GO all re-hash to their pins,
    and it still reads FAIL_CLOSED 4/13 under TECHNICAL_INVALID;
  * the leaf is new, distinct from R1's, born invalid, and there is exactly one
    launch with no retry path;
  * the thirteen gates and the zero-iteration semantics are unmoved.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_restore_audit_r2.py
"""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

sys.dont_write_bytecode = True

import v26c_j20_restore_audit as R1  # noqa: E402
import v26c_j20_restore_audit_r2 as R  # noqa: E402

R1_PATH = HERE / R.R1_RUNNER_NAME
R2_PATH = HERE / R.RUNNER_NAME
R1_SOURCE = R1_PATH.read_text(encoding="utf-8")
R2_SOURCE = R2_PATH.read_text(encoding="utf-8")
R2_TREE = ast.parse(R2_SOURCE)

REV4 = json.loads((HERE / R.PREREG_REV4_NAME).read_text(encoding="utf-8"))
REV5 = json.loads((HERE / R.PREREG_REV5_NAME).read_text(encoding="utf-8"))

R1_LEAF = HERE / R.R1_LEAF_REL
LEAF = HERE / R.LEAF_ROOT / R.LEAF_NAME

# The six functions that may differ: one new plus five modified. Nothing else.
DECLARED_NEW = {"child_environment"}
DECLARED_CHANGED = {"launch_once", "go_pin_targets", "expected_pin_hashes",
                    "check_entry_evidence", "run_execution"}

CHECKS: list = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def function_map(source: str) -> dict:
    tree = ast.parse(source)
    return {node.name: ast.get_source_segment(source, node)
            for node in tree.body if isinstance(node, ast.FunctionDef)}


def function_named(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


# ------------------------------------------------- A. the mechanical copy

def test_mechanical_copy() -> None:
    left, right = function_map(R1_SOURCE), function_map(R2_SOURCE)
    new = set(right) - set(left)
    removed = set(left) - set(right)
    changed = {k for k in set(left) & set(right) if left[k] != right[k]}

    check("A01 no function was removed from the R1 runner", not removed,
          str(sorted(removed)))
    check("A02 exactly the declared new function was added",
          new == DECLARED_NEW, str(sorted(new)))
    check("A03 exactly the declared functions changed",
          changed == DECLARED_CHANGED, str(sorted(changed)))
    shared = set(left) & set(right)
    identical = shared - changed
    check("A04 every other function is byte-identical to R1's",
          len(identical) == len(shared) - len(DECLARED_CHANGED),
          "%d identical" % len(identical))
    # The exact counts, asserted rather than described, so the report cannot
    # drift from the code. R1 has 25 functions; R2 has those 25 plus one new.
    check("A04a the R1 runner defines 25 functions", len(left) == 25,
          str(len(left)))
    check("A04b the R2 runner defines 26 functions", len(right) == 26,
          str(len(right)))
    check("A04c 25 functions are shared", len(shared) == 25, str(len(shared)))
    check("A04d 20 of them are byte-identical", len(identical) == 20,
          str(len(identical)))
    check("A04e 5 of them are modified", len(changed) == 5, str(len(changed)))
    check("A04f 1 is new", len(new) == 1, str(len(new)))
    check("A04g so 6 functions differ in total, and 20 do not",
          len(changed) + len(new) == 6 and len(identical) == 20)
    check("A05 evaluate_gates is byte-identical, so the thirteen gates cannot "
          "have moved", left.get("evaluate_gates") == right.get("evaluate_gates"))
    check("A06 restore_command is byte-identical, so the four operations and "
          "the delegated argv cannot have moved",
          left.get("restore_command") == right.get("restore_command"))
    check("A07 hermetic_restore is byte-identical",
          left.get("hermetic_restore") == right.get("hermetic_restore"))
    check("A08 verify_commit is byte-identical",
          left.get("verify_commit") == right.get("verify_commit"))
    check("A09 validate_go is byte-identical, so the APPROVED-only rule is "
          "unchanged", left.get("validate_go") == right.get("validate_go"))

    check("A10 the R1 runner still hashes to its pin",
          sha256_file(R1_PATH) == R.PIN_R1_RUNNER,
          "%s vs %s" % (sha256_file(R1_PATH), R.PIN_R1_RUNNER))
    check("A11 the stage identifiers differ, so neither GO can start the other",
          R.STAGE == "V26C_J20_RESTORE_AUDIT_R2" and R1.STAGE != R.STAGE)
    check("A12 the leaf names differ",
          R.LEAF_NAME == "j20_restore_audit_v26c_2026-08-27_r2"
          and R.LEAF_NAME != R1.LEAF_NAME)
    check("A13 the gate count is still thirteen",
          len(R.evaluate_gates({}, {}, {}, {})["gates"]) == 13)
    check("A14 the wrapper is the very one R1 used, byte-identical",
          sha256_file(HERE / R.CHILD_NAME) == sha256_file(HERE / R1.CHILD_NAME)
          and R.CHILD_NAME == R1.CHILD_NAME)


# --------------------------------------- B. the single correction, in detail

def test_child_environment() -> None:
    saved = os.environ.get("PYTHONPATH")
    try:
        os.environ.pop("PYTHONPATH", None)
        before = dict(os.environ)
        env = R.child_environment()
        first = env["PYTHONPATH"].split(os.pathsep)[0]

        check("B01 PYTHONPATH is set", bool(env.get("PYTHONPATH")))
        check("B02 the first entry is the baseline directory",
              first == str(R.BASELINE), first)
        check("B03 that entry is ABSOLUTE", os.path.isabs(first))
        check("B04 it is the directory that actually holds train_ppo_mlp.py",
              (pathlib.Path(first) / "train_ppo_mlp.py").is_file())
        check("B05 with no pre-existing value, no separator is appended",
              env["PYTHONPATH"] == str(R.BASELINE),
              repr(env["PYTHONPATH"]))
        check("B06 os.environ is NOT mutated", dict(os.environ) == before)
        check("B07 the returned mapping is a copy, not os.environ itself",
              env is not os.environ)
        check("B08 PYTHONDONTWRITEBYTECODE is still set",
              env.get("PYTHONDONTWRITEBYTECODE") == "1")

        os.environ["PYTHONPATH"] = "/already/here"
        env = R.child_environment()
        check("B09 a pre-existing value is preserved VERBATIM after the "
              "separator",
              env["PYTHONPATH"]
              == str(R.BASELINE) + os.pathsep + "/already/here",
              repr(env["PYTHONPATH"]))
        check("B10 the baseline still comes FIRST, so it wins",
              env["PYTHONPATH"].split(os.pathsep)[0] == str(R.BASELINE))
        check("B11 the pre-existing value is still present, unaltered",
              env["PYTHONPATH"].split(os.pathsep)[1:] == ["/already/here"])

        multi = os.pathsep.join(["/a", "/b", "/c"])
        os.environ["PYTHONPATH"] = multi
        env = R.child_environment()
        check("B12 a multi-entry pre-existing value survives in order",
              env["PYTHONPATH"].split(os.pathsep)[1:] == ["/a", "/b", "/c"])
        check("B13 nothing is deduplicated away or reordered",
              env["PYTHONPATH"] == str(R.BASELINE) + os.pathsep + multi)
    finally:
        if saved is None:
            os.environ.pop("PYTHONPATH", None)
        else:
            os.environ["PYTHONPATH"] = saved

    node = function_named(R2_TREE, "child_environment")
    body = ast.get_source_segment(R2_SOURCE, node) or ""
    check("B14 the separator is os.pathsep, not a hard-coded character",
          "os.pathsep" in body)
    check("B15 no hard-coded ':' or ';' separator is used",
          '":"' not in body and "';'" not in body and '";"' not in body)
    check("B16 the environment is built from a COPY of os.environ",
          "dict(os.environ)" in body)
    check("B17 child_environment writes no file",
          not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                  and n.func.attr in ("write_bytes", "write_text", "mkdir")
                  for n in ast.walk(node)))

    # Fail-closed behaviour: a baseline that cannot work must raise here, not
    # produce 115 dead workers later.
    original = R.BASELINE
    try:
        R.BASELINE = pathlib.Path("relative/not/absolute")
        try:
            R.child_environment()
            raised = False
        except R.RestoreAuditError:
            raised = True
        check("B18 a relative baseline is refused", raised)

        R.BASELINE = pathlib.Path("/tmp")
        try:
            R.child_environment()
            raised = False
        except R.RestoreAuditError:
            raised = True
        check("B19 a baseline without train_ppo_mlp.py is refused", raised)
    finally:
        R.BASELINE = original
    check("B20 the baseline constant was restored", R.BASELINE == original)


def test_environment_reaches_popen() -> None:
    node = function_named(R2_TREE, "launch_once")
    source = ast.get_source_segment(R2_SOURCE, node) or ""
    check("B21 launch_once obtains its environment from child_environment()",
          any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id == "child_environment" for n in ast.walk(node)))
    check("B22 launch_once no longer builds the environment inline",
          "dict(os.environ)" not in source)

    popen = [n for n in ast.walk(node)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "Popen"]
    check("B23 launch_once holds exactly one Popen", len(popen) == 1)
    keywords = {k.arg: k.value for k in popen[0].keywords} if popen else {}
    check("B24 Popen is given env=", "env" in keywords)
    check("B25 that env is the environment child_environment() returned",
          isinstance(keywords.get("env"), ast.Name)
          and keywords["env"].id == "environment")
    check("B26 Popen is given cwd=baseline",
          "cwd" in keywords and "BASELINE" in ast.dump(keywords["cwd"]))
    check("B27 the receipt records the PYTHONPATH actually used",
          '"pythonpath": environment.get("PYTHONPATH")' in source)

    whole = [n for n in ast.walk(R2_TREE)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "Popen"]
    check("B28 exactly one Popen in the entire runner", len(whole) == 1)
    run = function_named(R2_TREE, "run_execution")
    check("B29 run_execution calls launch_once exactly once",
          sum(1 for n in ast.walk(run) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Name)
              and n.func.id == "launch_once") == 1)
    check("B30 no loop in the runner ever reaches launch_once",
          not any(isinstance(n, (ast.For, ast.While)) and any(
              isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
              and c.func.id == "launch_once" for c in ast.walk(n))
              for n in ast.walk(R2_TREE)))

    calls = {n.func.attr for n in ast.walk(R2_TREE)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    for forbidden in ("train", "sample", "restore_from_path", "save_to_path"):
        check("B31 the runner never calls %s" % forbidden,
              forbidden not in calls)


# ------------------------------------------------------- C. the smoke test

RAY_PROCESS_MARKERS = ("raylet", "gcs_server", "ray::", "plasma_store",
                       "ray_client_server")


def _load_psutil():
    """psutil, from wherever it is available, or None.

    Ray vendors its own copy, so a machine without a top-level psutil can still
    resolve ray.thirdparty_files.psutil.
    """
    try:
        import psutil
        return psutil
    except Exception:                                       # noqa: BLE001
        pass
    try:
        from ray.thirdparty_files import psutil             # noqa: F401
        return psutil
    except Exception:                                       # noqa: BLE001
        return None


def _ray_count_via_psutil():
    """Count with psutil, or None if psutil cannot enumerate here.

    IMPORTING psutil is not the same as being ALLOWED to enumerate: under the
    macOS sandbox process_iter() raises PermissionError [Errno 1] while
    iterating, which an import-only guard would let escape. The whole walk is
    therefore inside the guard, so an enumeration that is refused degrades to
    the platform fallback instead of killing the suite.
    """
    psutil = _load_psutil()
    if psutil is None:
        return None
    total = 0
    try:
        for process in psutil.process_iter(["name", "cmdline"]):
            try:
                info = process.info
                blob = ((info.get("name") or "") + " "
                        + " ".join(info.get("cmdline") or [])).lower()
            except Exception:                               # noqa: BLE001
                continue                    # one unreadable process, not fatal
            if any(marker in blob for marker in RAY_PROCESS_MARKERS):
                total += 1
    except Exception:                                       # noqa: BLE001
        return None                         # enumeration refused: not measured
    return total


def _ray_count_via_platform():
    """Count by asking the platform's own process listing, or None."""
    command = ["wmic", "process", "get", "commandline"] if os.name == "nt" \
        else ["ps", "-A", "-o", "command="]
    try:
        listing = subprocess.run(command, capture_output=True, text=True,
                                 timeout=60)
    except Exception:                                       # noqa: BLE001
        return None
    if listing.returncode != 0:
        return None
    return sum(1 for line in listing.stdout.splitlines()
               if any(marker in line.lower() for marker in RAY_PROCESS_MARKERS))


def ray_process_count():
    """How many Ray-related processes exist right now, or None if unmeasurable.

    Never returns 0 as a way of saying "I could not look": an unmeasurable
    count returns None and the caller FAILS the check rather than passing
    silently. Both routes are tried before giving up.
    """
    count = _ray_count_via_psutil()
    if count is not None:
        return count
    return _ray_count_via_platform()


def test_hook_serialisation_smoke() -> None:
    """The REAL smoke: serialise the actual Ray setup hook, deserialise it.

    rev5 replaced rev4's find_spec-only check because find_spec proves a module
    is FINDABLE, not that the operation which actually failed - cloudpickle
    deserialising a by-reference payload - now succeeds.

    So: import train_ppo_mlp here, serialise train_ppo_mlp._worker_setup with
    Ray's own cloudpickle, and deserialise that exact payload in two fresh
    interpreters whose working directory is the validation directory. Without
    PYTHONPATH it must fail PRECISELY with ModuleNotFoundError naming
    train_ppo_mlp; with child_environment() it must yield a callable whose
    __module__ and __name__ are the expected ones.

    ray.init is never called, no daemon is started, and the deserialised
    function is NEVER invoked - only inspected.
    """
    import tempfile

    ray_before = ray_process_count()

    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import ray                                              # noqa: E402
    import ray.cloudpickle as ray_cloudpickle               # noqa: E402
    import train_ppo_mlp                                    # noqa: E402

    check("C10 importing train_ppo_mlp does not initialise Ray",
          not ray.is_initialized())
    check("C11 the hook the trainer registers exists",
          callable(getattr(train_ppo_mlp, "_worker_setup", None)))
    check("C12 the hook resolves to the imported module, so cloudpickle will "
          "serialise it BY REFERENCE",
          train_ppo_mlp._worker_setup.__module__ == "train_ppo_mlp")

    payload = ray_cloudpickle.dumps(train_ppo_mlp._worker_setup)
    check("C13 the payload names train_ppo_mlp, which is what by-reference "
          "looks like", b"train_ppo_mlp" in payload)
    check("C14 the payload is small, as a by-reference pickle is",
          len(payload) < 1024, "%d bytes" % len(payload))

    loader = (
        "import pathlib, sys\n"
        "import ray.cloudpickle as rcp\n"
        "raw = pathlib.Path(sys.argv[1]).read_bytes()\n"
        "try:\n"
        "    fn = rcp.loads(raw)\n"
        "except BaseException as exc:\n"
        "    print('LOADFAIL', type(exc).__name__, str(exc)[:200])\n"
        "    raise SystemExit(0)\n"
        "print('LOADOK', getattr(fn, '__module__', None),\n"
        "      getattr(fn, '__name__', None), callable(fn))\n"
    )

    with tempfile.TemporaryDirectory() as tmp:
        blob = pathlib.Path(tmp) / "worker_setup_hook.pkl"
        blob.write_bytes(payload)
        script = pathlib.Path(tmp) / "deserialise_only.py"
        script.write_text(loader, encoding="utf-8")

        without = dict(os.environ)
        without.pop("PYTHONPATH", None)
        without["PYTHONDONTWRITEBYTECODE"] = "1"
        negative = subprocess.run([R.INTERPRETER, str(script), str(blob)],
                                  cwd=str(HERE), env=without,
                                  capture_output=True, text=True)
        out = negative.stdout.strip()
        check("C15 NEGATIVE: without PYTHONPATH the hook does not deserialise",
              out.startswith("LOADFAIL"), out or negative.stderr[-200:])
        check("C16 NEGATIVE: it fails PRECISELY with ModuleNotFoundError",
              "ModuleNotFoundError" in out, out)
        check("C17 NEGATIVE: and precisely on train_ppo_mlp",
              "No module named 'train_ppo_mlp'" in out, out)
        check("C18 NEGATIVE: this is the very error the 115 R1 workers raised",
              "ModuleNotFoundError" in out and "train_ppo_mlp" in out)

        positive = subprocess.run([R.INTERPRETER, str(script), str(blob)],
                                  cwd=str(HERE), env=R.child_environment(),
                                  capture_output=True, text=True)
        out = positive.stdout.strip()
        check("C19 POSITIVE: with child_environment() the hook deserialises",
              out.startswith("LOADOK"), out or positive.stderr[-200:])
        fields = out.split()
        check("C20 POSITIVE: the module is train_ppo_mlp",
              len(fields) > 1 and fields[1] == "train_ppo_mlp", out)
        check("C21 POSITIVE: the name is _worker_setup",
              len(fields) > 2 and fields[2] == "_worker_setup", out)
        check("C22 POSITIVE: the result is callable",
              len(fields) > 3 and fields[3] == "True", out)
        check("C23 the two runs differed ONLY by the environment",
              negative.stdout.strip() != positive.stdout.strip())

    check("C24 the deserialised hook was never invoked",
          "fn(" not in loader and "fn()" not in loader)
    check("C25 the loader never calls ray.init", "ray.init" not in loader)
    # On the AST, not on a substring: the only textual occurrence of the call
    # would be the literal inside this very check, which could never pass.
    def calls_ray_init(source: str) -> bool:
        return any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                   and n.func.attr == "init"
                   and isinstance(n.func.value, ast.Name)
                   and n.func.value.id == "ray"
                   for n in ast.walk(ast.parse(source)))

    check("C26 the suite never calls ray.init",
          not calls_ray_init(pathlib.Path(__file__).read_text(encoding="utf-8")))
    check("C26b the R2 runner never calls ray.init",
          not calls_ray_init(R2_SOURCE))
    check("C26c the wrapper never calls ray.init",
          not calls_ray_init((HERE / R.CHILD_NAME).read_text(encoding="utf-8")))
    check("C26d the check is not vacuous: it detects a real ray.init call",
          calls_ray_init("import ray\nray.init()\n"))
    check("C27 Ray is still not initialised in this process",
          not ray.is_initialized())
    ray_after = ray_process_count()
    check("C28a the Ray process count is measurable, so C28 is not vacuous",
          ray_before is not None and ray_after is not None,
          "before=%r after=%r" % (ray_before, ray_after))

    # The exact failure an independent review hit: psutil IMPORTS but
    # process_iter raises PermissionError [Errno 1] under the macOS sandbox.
    # An import-only guard let that escape and the fallback was never reached.
    class _RefusingPsutil:
        @staticmethod
        def process_iter(_attrs=None):
            raise PermissionError(1, "Operation not permitted")

    saved_loader = globals()["_load_psutil"]
    try:
        globals()["_load_psutil"] = lambda: _RefusingPsutil
        refused = _ray_count_via_psutil()
        check("C28b a psutil that refuses to enumerate degrades to None "
              "instead of raising", refused is None, repr(refused))
        fell_back = ray_process_count()
        check("C28c and ray_process_count still measures, via the platform "
              "fallback", fell_back is not None, repr(fell_back))

        # And if BOTH routes fail, the count must be None so C28a fails closed.
        saved_platform = globals()["_ray_count_via_platform"]
        try:
            globals()["_ray_count_via_platform"] = lambda: None
            check("C28d with both routes unavailable the count is None, so the "
                  "check fails rather than passing silently",
                  ray_process_count() is None)
        finally:
            globals()["_ray_count_via_platform"] = saved_platform
    finally:
        globals()["_load_psutil"] = saved_loader
    check("C28e the real loader was restored",
          globals()["_load_psutil"] is saved_loader)
    check("C28 no Ray daemon was started by the smoke test",
          ray_before is not None and ray_after == ray_before,
          "%r before, %r after" % (ray_before, ray_after))
    check("C29 the smoke test created no R2 leaf", not LEAF.exists())
    check("C30 the smoke test left the R1 leaf untouched",
          all(sha256_file(R1_LEAF / n) == d
              for n, d in R.PIN_R1_LEAF.items()))


def test_import_smoke() -> None:
    """The cheap preliminary rev5 keeps: is train_ppo_mlp even FINDABLE?

    Retained from rev4 as a first-line check. It is necessary but not
    sufficient, which is why test_hook_serialisation_smoke exists.
    """
    probe = ("import importlib.util, sys;"
             "print('FOUND' if importlib.util.find_spec('train_ppo_mlp') "
             "else 'MISSING')")

    without = dict(os.environ)
    without.pop("PYTHONPATH", None)
    without["PYTHONDONTWRITEBYTECODE"] = "1"
    before = subprocess.run([R.INTERPRETER, "-c", probe], cwd=str(HERE),
                            env=without, capture_output=True, text=True)
    check("C01 without the correction a fresh interpreter cannot find "
          "train_ppo_mlp",
          before.stdout.strip() == "MISSING",
          before.stdout.strip() + before.stderr[-160:])

    after = subprocess.run([R.INTERPRETER, "-c", probe], cwd=str(HERE),
                           env=R.child_environment(), capture_output=True,
                           text=True)
    check("C02 with the correction the same interpreter finds it",
          after.stdout.strip() == "FOUND",
          after.stdout.strip() + after.stderr[-160:])

    check("C03 the two probes differed only by the environment",
          before.stdout.strip() != after.stdout.strip())

    origin = ("import importlib.util;"
              "print(importlib.util.find_spec('train_ppo_mlp').origin)")
    located = subprocess.run([R.INTERPRETER, "-c", origin], cwd=str(HERE),
                             env=R.child_environment(), capture_output=True,
                             text=True)
    check("C04 the module it finds is the pinned production trainer",
          located.stdout.strip() == str(BASELINE / "train_ppo_mlp.py"),
          located.stdout.strip())

    check("C05 the smoke test starts no Ray",
          "ray" not in probe and "ray" not in origin)
    check("C06 rev4 documented the original find_spec check",
          "smoke_test" in REV4)
    check("C07 rev5 says why find_spec alone was insufficient",
          "does not exercise the operation that actually failed"
          in REV5["correction_3_the_smoke_test_is_strengthened"][
              "why_that_was_insufficient"])
    check("C08 rev5 requires the precise ModuleNotFoundError in the negative",
          "PRECISELY with ModuleNotFoundError"
          in REV5["correction_3_the_smoke_test_is_strengthened"][
              "what_is_required_now"]["step_3_negative"])
    check("C09 rev5 forbids invoking the deserialised hook",
          "NEVER invoked"
          in REV5["correction_3_the_smoke_test_is_strengthened"][
              "what_is_required_now"]["step_5_never_call_it"])


# --------------------------------------------------- D. R1 is untouched

def test_r1_preserved() -> None:
    check("D01 the R1 leaf still exists", R1_LEAF.is_dir())
    bad = [n for n, d in R.PIN_R1_LEAF.items()
           if not (R1_LEAF / n).is_file() or sha256_file(R1_LEAF / n) != d]
    check("D02 every pinned R1 leaf file re-hashes to its pin", not bad,
          str(bad))
    check("D03 the R1 leaf still carries TECHNICAL_INVALID",
          (R1_LEAF / R.INVALID_MARKER).is_file())
    check("D04 the R1 leaf does NOT carry RESTORE_AUDIT_PASSED",
          not (R1_LEAF / R.PASSED_MARKER).exists())

    result = json.loads(
        (R1_LEAF / "v26c_j20_restore_audit_result.json").read_text("utf-8"))
    check("D05 R1 still reads FAIL_CLOSED", result["verdict"] == R.VERDICT_FAILED)
    check("D06 R1 still reads 4 of 13",
          result["gates_passed"] == 4 and result["gates_total"] == 13)
    check("D07 R1 recorded the 124 self-timeout",
          result["child"]["returncode"] == 124)
    check("D08 R1 recorded a single launch and no retry",
          result["child"]["launched_once"] is True
          and result["child"]["retried"] is False)
    check("D09 R1's four passing gates were R8, R10, R11 and R13",
          sorted(k.split("_")[0] for k, ok in result["gates"].items() if ok)
          == ["R10", "R11", "R13", "R8"])

    check("D10 the R1 suite still hashes to its pin",
          sha256_file(HERE / R.R1_TEST_NAME) == R.PIN_R1_TEST)
    check("D11 the R1 architect GO still hashes to its pin",
          sha256_file(HERE / R.R1_GO_NAME) == R.PIN_R1_GO)

    targets = R.go_pin_targets()
    for name in R.PIN_R1_LEAF:
        check("D12 the R1 leaf file %s is inside the pin map" % name,
              (R.R1_LEAF_REL + "/" + name) in targets)
    check("D13 R11 therefore covers R1 without R11 being edited",
          function_map(R1_SOURCE)["evaluate_gates"]
          == function_map(R2_SOURCE)["evaluate_gates"])

    # The invariant, on the AST: every filesystem write in run_execution has
    # the R2 leaf as its destination, and none is built from the R1 leaf.
    # (run_execution does mention R1_LEAF_REL - it records it in the receipt -
    # so a substring test would be wrong here.)
    node = function_named(R2_TREE, "run_execution")
    writes = [n for n in ast.walk(node)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr in ("write_bytes", "write_text", "mkdir",
                                  "unlink", "rmdir", "rename", "replace")]
    targets = [ast.get_source_segment(R2_SOURCE, n.func.value) or ""
               for n in writes]
    check("D14 every write in run_execution targets the R2 leaf",
          targets and all("leaf" in t for t in targets), str(targets))
    check("D14b no write in run_execution is built from the R1 leaf",
          not any("R1_LEAF" in t for t in targets), str(targets))
    check("D14c the R1 leaf appears in run_execution only as a recorded value",
          all("R1_LEAF" not in t for t in targets))

    entry = R.check_entry_evidence()
    check("D15 the entry evidence passes", entry["ok"],
          "; ".join(entry["problems"]))
    check("D16 it reads R1 as FAIL_CLOSED 4/13",
          entry["found"]["r1_verdict"] == R.VERDICT_FAILED
          and entry["found"]["r1_gates"] == "4/13")
    check("D17 it reads the R1 child returncode 124",
          entry["found"]["r1_child_returncode"] == 124)
    check("D18 the warm-up leaf is still AWAITING_RESTORE_AUDIT 12/12",
          entry["found"]["warmup_verdict"] == "AWAITING_RESTORE_AUDIT"
          and entry["found"]["warmup_gates"] == "12/12")


# ------------------------------------------------------- E. the new leaf

def test_new_leaf() -> None:
    check("E01 the R2 destination is absent", not LEAF.exists())
    check("E02 the R2 destination is not the R1 leaf",
          LEAF != R1_LEAF and LEAF.name.endswith("_r2"))
    run = ast.get_source_segment(R2_SOURCE,
                                 function_named(R2_TREE, "run_execution")) or ""
    check("E03 an existing destination is refused",
          "refusing to clobber an existing leaf" in run)
    check("E04 the invalid marker is written before the child",
          run.index("INVALID_MARKER") < run.index("launch_once"))
    check("E05 the leaf is asserted clean before the child",
          run.index("the destination is not clean") < run.index("launch_once"))
    check("E06 promotion stays NONE", '"promotion": "NONE"' in R2_SOURCE)
    check("E07 training_ready stays false",
          '"training_ready": False' in R2_SOURCE)
    check("E08 next_stage_authorized stays false",
          '"next_stage_authorized": False' in R2_SOURCE)


# ------------------------------------------------- F. preregistration and GO

def test_prereg_and_go() -> None:
    path = HERE / R.PREREG_REV4_NAME
    actual = sha256_file(path)
    check("F01 rev4 hashes to its pin", actual == R.PIN_PREREG_REV4,
          "%s vs %s" % (actual, R.PIN_PREREG_REV4))
    check("F02 rev4 declares contains_no_self_hash",
          REV4.get("contains_no_self_hash") is True)
    check("F03 rev4 really contains no self hash",
          actual not in path.read_text(encoding="utf-8"))
    check("F04 rev4 names the R2 stage", REV4["stage"] == R.STAGE)
    for label, pin in (("base", R.PIN_PREREG), ("rev1", R.PIN_PREREG_REV1),
                       ("rev2", R.PIN_PREREG_REV2), ("rev3", R.PIN_PREREG_REV3)):
        check("F05 rev4 cites %s by exact hash" % label,
              REV4["cites"]["%s_sha256" % label] == pin)
        check("F06 %s is still at its sealed digest" % label,
              sha256_file(HERE / {"base": R.PREREG_NAME,
                                  "rev1": R.PREREG_REV1_NAME,
                                  "rev2": R.PREREG_REV2_NAME,
                                  "rev3": R.PREREG_REV3_NAME}[label]) == pin)
    # rev4 declared the chain as it stood WHEN REV4 WAS SEALED. rev5 has since
    # superseded it, so rev4's chain must be the runner's chain minus rev5 -
    # not equal to it. rev4 is not wrong; it is simply no longer the head.
    check("F07 rev4 declares the chain that was current when it was sealed",
          REV4["precedence"].replace(" ", "")
          == ">".join(R.PREREG_PRECEDENCE[1:]),
          "%r vs %r" % (REV4["precedence"], R.PREREG_PRECEDENCE[1:]))
    check("F07b rev5 is the head of the chain the runner implements",
          R.PREREG_PRECEDENCE[0] == "rev5"
          and REV5["precedence"].replace(" ", "")
          == ">".join(R.PREREG_PRECEDENCE))
    check("F08 rev4 records the 115 observed ModuleNotFoundError",
          REV4["observed_cause"]["occurrences_in_the_r1_child_log"] == 115)
    check("F09 rev4 records zero occurrences in the successful warm-up",
          REV4["observed_cause"]["occurrences_in_the_successful_warm_up_log"]
          == 0)
    check("F10 rev4 records the 124 returncode and the build_algo phase",
          REV4["observed_cause"]["child_returncode"] == 124
          and REV4["observed_cause"]["watchdog_phase"] == "PPOConfig.build_algo")
    check("F11 rev4 records that PYTHONPATH was unset in R1",
          REV4["observed_cause"]["pythonpath_in_the_r1_child_environment"]
          is None)
    check("F12 rev4 declares a SINGLE correction",
          "the_single_correction" in REV4
          and "PYTHONPATH" in REV4["the_single_correction"]["what"])
    check("F13 rev4 forbids an automatic retry",
          "not a retry" in REV4["no_automatic_retry"]["rule"].lower())
    check("F14 rev4 preserves R1 as immutable evidence",
          "PRESERVED AS IMMUTABLE EVIDENCE"
          in REV4["the_r1_attempt"]["status"])
    check("F15 rev4 adds no gate",
          "No gate is added" in REV4["additive"])
    check("F16 rev4 declares the two mechanical argv consequences",
          set(REV4["changes_that_follow_mechanically_from_the_new_leaf"])
          >= {"--output-dir", "--audit-evidence"})

    path5 = HERE / R.PREREG_REV5_NAME
    actual5 = sha256_file(path5)
    check("F24 rev5 hashes to its pin", actual5 == R.PIN_PREREG_REV5,
          "%s vs %s" % (actual5, R.PIN_PREREG_REV5))
    check("F25 rev5 declares contains_no_self_hash",
          REV5.get("contains_no_self_hash") is True)
    check("F26 rev5 really contains no self hash",
          actual5 not in path5.read_text(encoding="utf-8"))
    check("F27 rev5 names the R2 stage", REV5["stage"] == R.STAGE)
    check("F28 rev5 cites and pins rev4 by exact hash",
          REV5["cites"]["rev4_sha256"] == R.PIN_PREREG_REV4)
    for label, pin in (("base", R.PIN_PREREG), ("rev1", R.PIN_PREREG_REV1),
                       ("rev2", R.PIN_PREREG_REV2), ("rev3", R.PIN_PREREG_REV3),
                       ("rev4", R.PIN_PREREG_REV4)):
        check("F29 rev5 cites %s by exact hash" % label,
              REV5["cites"]["%s_sha256" % label] == pin)
    check("F30 rev4 is still at its sealed digest",
          sha256_file(HERE / R.PREREG_REV4_NAME) == R.PIN_PREREG_REV4)
    check("F31 rev5 declares the precedence the runner implements",
          REV5["precedence"].replace(" ", "") == ">".join(R.PREREG_PRECEDENCE))
    check("F32 the runner puts rev5 first in precedence",
          R.PREREG_PRECEDENCE[0] == "rev5")
    check("F33 rev5 names R2 the second corrective attempt",
          "SECOND CORRECTIVE ATTEMPT"
          in REV5["correction_1_what_R2_honestly_is"]["the_name"])
    check("F34 rev5 requires BOTH user and architect authorisation",
          "user authorisation" in
          REV5["correction_1_what_R2_honestly_is"]["authorisation_required"]
          and "APPROVED" in
          REV5["correction_1_what_R2_honestly_is"]["authorisation_required"])
    check("F35 rev5 keeps R2 non-automatic",
          any("NEVER automatic" in v for v in
              REV5["correction_1_what_R2_honestly_is"]["what_remains_true_from_rev4"]))
    check("F36 rev5 separates direct evidence from causal inference",
          "DIRECT_EVIDENCE" in REV5["correction_2_evidence_versus_inference"]
          and "CAUSAL_INFERENCE"
          in REV5["correction_2_evidence_versus_inference"])
    check("F37 the direct evidence records the 115 occurrences",
          any("115 occurrences" in item for item in
              REV5["correction_2_evidence_versus_inference"]["DIRECT_EVIDENCE"]["items"]))
    check("F38 the direct evidence records the traceback location",
          any("setup_hook.py" in item for item in
              REV5["correction_2_evidence_versus_inference"]["DIRECT_EVIDENCE"]["items"]))
    check("F39 the direct evidence records the absent PYTHONPATH",
          any("no PYTHONPATH" in item for item in
              REV5["correction_2_evidence_versus_inference"]["DIRECT_EVIDENCE"]["items"]))
    check("F40 the direct evidence records the phase timeout",
          any("phase_timeout" in item for item in
              REV5["correction_2_evidence_versus_inference"]["DIRECT_EVIDENCE"]["items"]))
    check("F41 the remaining hypothesis is still marked NOT VERIFIED",
          "NOT VERIFIED" in
          REV5["correction_2_evidence_versus_inference"]["STILL_A_HYPOTHESIS"]["status"])
    check("F42 rev5 leaves the 600 s timeout alone",
          any("600 s startup timeout: NOT raised" in item
              for item in REV5["explicitly_unchanged"]))
    check("F43 rev5 annotates the two historical R1 suite checks",
          "ANNOTATED, NOT CORRECTED"
          in REV5["note_on_the_two_historical_R1_suite_checks"]["decision"])
    check("F44 rev5 adds no gate", "No gate is added" in REV5["additive"])

    draft = HERE / "v26c_j20_restore_audit_r2_go_DRAFT.json"
    if draft.is_file():
        payload = json.loads(draft.read_text(encoding="utf-8"))
        verdict = R.validate_go(payload)
        check("F17 the DRAFT GO is refused", not verdict["valid"])
        # rev5: the DRAFT must be inert on BOTH counts.
        check("F18 the DRAFT declares status DRAFT",
              payload.get("status") == "DRAFT")
        check("F18b the DRAFT declares authorises_execution false",
              payload.get("authorises_execution") is False)
        check("F18c it is refused on both counts, and on nothing else",
              len(verdict["problems"]) == 2
              and any("status" in p for p in verdict["problems"])
              and any("authorises_execution" in p for p in verdict["problems"]),
              str(verdict["problems"]))
        approved = copy.deepcopy(payload)
        approved["status"] = "APPROVED"
        approved["authorises_execution"] = True
        check("F19 the same payload with APPROVED would validate, so every pin "
              "is right", R.validate_go(approved)["valid"],
              "; ".join(R.validate_go(approved)["problems"][:3]))
        check("F20 the DRAFT pins every required label",
              verdict["pin_labels_supplied"] == verdict["pin_labels_required"])
        r1_stage = copy.deepcopy(approved)
        r1_stage["stage"] = R1.STAGE
        check("F21 an R1-stage GO cannot authorise R2",
              not R.validate_go(r1_stage)["valid"])
        for flag in (False, None, "true", 1):
            bad = copy.deepcopy(approved)
            bad["authorises_execution"] = flag
            check("F21b authorises_execution %r is refused" % (flag,),
                  not R.validate_go(bad)["valid"])
        for status in (None, "", "approved", "ok", True, 1):
            bad = copy.deepcopy(approved)
            bad["status"] = status
            check("F22 the status %r is refused" % (status,),
                  not R.validate_go(bad)["valid"])
        bad = copy.deepcopy(approved)
        del bad["status"]
        check("F23 a missing status is refused", not R.validate_go(bad)["valid"])


# ------------------------------------------------- G. gates still non-vacuous

def good_inputs() -> tuple:
    checkpoint = str(HERE / R.SOURCE_LEAF_REL / R.CHECKPOINT_REL)
    summary = {
        "iterations_run": 0, "iterations_completed_this_process": 0,
        "iterations_completed": 1, "history": [],
        "restored_training_iteration": 1, "restored_logical_iteration": 1,
        "iteration_start": 2, "next_iteration": 2,
        "resume_from": checkpoint, "initialization_mode": "resume_from",
        "warm_start_raw_transplant_applied_this_process": False,
        "actor_freeze_audit": [{"stage": "before_training",
                                "actor_digest": R.EXPECTED_ACTOR_DIGEST}],
        "critic_state_audit": [{"stage": "before_training",
                                "critic_digest": R.EXPECTED_CRITIC_DIGEST,
                                "critic_keys": list(R.CRITIC_KEYS)}],
        "optimizer_lr_audit": [{"stage": "after_restore", "learners": []}],
        "stop_reason": "completed", "interrupted": False, "timed_out": False,
    }
    artefacts = {
        "summary": summary, "iteration_rows": [],
        "evidence": {
            "kind": R.EVIDENCE_KIND, "stage_marker": R.EVIDENCE_STAGE_MARKER,
            "source_state_sha256": R.EXPECTED_SOURCE_STATE_SHA,
            "exact": True, "differences": [], "difference_count": 0,
            "problems": [], "normalised_digest_source": "a" * 64,
            "normalised_digest_live": "a" * 64,
            "normalised_digests_match": True,
            "param_groups": {"exact": True, "differences": [],
                             "digests_match": True},
            "top_level_keys_source": ["param_groups", "state"],
            "top_level_keys_live": ["param_groups", "state"],
            "state_indices_source": [6, 7, 8, 9, 10, 11],
            "state_indices_live": [6, 7, 8, 9, 10, 11],
            "learner_count": 1, "optimizer_names": ["default_optimizer"],
        },
        "forbidden_present": [], "supervisor_state_present": False,
    }
    hermetic = {
        "ok": True, "all_keys_byte_identical": True, "state_keys": 16,
        "strict_load_missing": [], "strict_load_unexpected": [],
        "actor_digest": R.EXPECTED_ACTOR_DIGEST,
        "critic_digest": R.EXPECTED_CRITIC_DIGEST,
        "logstd_weight_rows_zero": True, "logstd_bias_exact": True,
        "sigma_exact": True, "optimizer_loaded": True,
        "all_moments_byte_identical": True,
        "adam_indices": list(R.EXPECTED_CRITIC_PARAM_INDICES),
        "adam_steps": [81.0] * 6,
        "critic_indices_are_the_critic": True,
        "actor_indices_are_not_the_critic": True,
    }
    integrity = {"source_tree_unchanged": True, "pins_unchanged": True}
    child = {"returncode": 0, "launched_once": True, "retried": False}
    return child, artefacts, hermetic, integrity


def test_gates_unmoved() -> None:
    child, artefacts, hermetic, integrity = good_inputs()
    gates = R.evaluate_gates(child, artefacts, hermetic, integrity)["gates"]
    check("G01 a correct R2 run would pass all thirteen", all(gates.values()),
          str(sorted(n for n, ok in gates.items() if not ok)))
    check("G02 R2 grades identically to R1 on the same inputs",
          R1.evaluate_gates(child, artefacts, hermetic, integrity)["gates"]
          == gates)

    def flips(label, gate, mutate):
        c, a, h, i = good_inputs()
        mutate(c, a, h, i)
        after = R.evaluate_gates(c, a, h, i)["gates"]
        check("G03 %s" % label, after[gate] is False, "%s did not flip" % gate)

    # A representative flip per gate. The exhaustive 55-perturbation sweep lives
    # in the R1 suite and carries over because evaluate_gates is byte-identical.
    flips("an iteration row fails R1", "R1_zero_new_iterations",
          lambda c, a, h, i: a.__setitem__("iteration_rows", ["{}"]))
    flips("a non-empty history fails R2", "R2_empty_history",
          lambda c, a, h, i: a["summary"].__setitem__("history", [{"i": 1}]))
    flips("restored iteration 0 fails R3", "R3_restored_iteration_is_one",
          lambda c, a, h, i: a["summary"].__setitem__(
              "restored_training_iteration", 0))
    flips("iteration_start 1 fails R4", "R4_next_and_start_are_two",
          lambda c, a, h, i: a["summary"].__setitem__("iteration_start", 1))
    flips("warm_start_raw mode fails R5", "R5_the_restore_path_was_taken",
          lambda c, a, h, i: a["summary"].__setitem__("initialization_mode",
                                                      "warm_start_raw"))
    flips("a wrong actor digest fails R6", "R6_restored_actor_digest",
          lambda c, a, h, i: a["summary"]["actor_freeze_audit"][0].__setitem__(
              "actor_digest", "0" * 64))
    flips("a wrong critic digest fails R7", "R7_restored_critic_digest",
          lambda c, a, h, i: a["summary"]["critic_state_audit"][0].__setitem__(
              "critic_digest", "0" * 64))
    flips("a non-identical module fails R8", "R8_hermetic_full_module_restore",
          lambda c, a, h, i: h.__setitem__("all_keys_byte_identical", False))
    flips("missing live evidence fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a.__setitem__("evidence", None))
    flips("a falsified difference count fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a["evidence"].__setitem__("differences", ["x"]))
    flips("a checkpoint in the leaf fails R10", "R10_nothing_was_trained",
          lambda c, a, h, i: a.__setitem__("forbidden_present",
                                           ["checkpoint_last"]))
    flips("a changed pin fails R11", "R11_sources_unchanged",
          lambda c, a, h, i: i.__setitem__("pins_unchanged", False))
    flips("returncode 124 fails R12", "R12_single_clean_process",
          lambda c, a, h, i: c.__setitem__("returncode", 124))
    flips("non-identical moments fail R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("all_moments_byte_identical", False))


# ------------------------------------------------------------ H. preflight

def test_preflight() -> None:
    module_imports = set()
    for node in R2_TREE.body:
        if isinstance(node, ast.Import):
            module_imports.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            module_imports.add(node.module.split(".")[0])
    for heavy in ("torch", "ray", "gymnasium"):
        check("H01 the runner imports no %s at module level" % heavy,
              heavy not in module_imports)

    report = R.preflight(verbose=False)
    check("H02 preflight passes", report["ok"], "; ".join(report["problems"]))
    check("H03 preflight reports thirteen gates", report["gates"] == 13)
    check("H04 preflight creates no leaf", not LEAF.exists())
    command = R.restore_command(str(LEAF))
    check("H05 the three token counts are unchanged from R1",
          command["derived_token_count"] == R.EXPECTED_DERIVED_TOKENS == 20
          and command["delegated_argv_count"] == R.EXPECTED_DELEGATED_ARGV == 18
          and command["child_argv_count"] == R.EXPECTED_CHILD_ARGV == 25)
    check("H06 the output directory is the R2 leaf",
          command["derived_tokens"][
              command["derived_tokens"].index("--output-dir") + 1] == str(LEAF))
    check("H07 the evidence path is inside the R2 leaf",
          command["child_argv"][5] == str(LEAF / R.EVIDENCE_NAME))
    check("H08 the resume path is still the warm-up checkpoint",
          command["derived_tokens"][-1]
          == str(HERE / R.SOURCE_LEAF_REL / R.CHECKPOINT_REL))
    check("H09 --iteration-start is still not passed",
          "--iteration-start" not in command["child_argv"])
    check("H10 the config is still the critic-only one, unmodified",
          str(HERE / "v26c_j20_warmup_critic_only_cfg.yaml")
          in command["derived_tokens"])

    r1_command = R1.restore_command(str(LEAF))
    check("H11 apart from the leaf, the delegated argv is identical to R1's",
          list(command["delegated_argv"]) == list(r1_command["delegated_argv"]))


def main() -> int:
    test_mechanical_copy()
    test_child_environment()
    test_environment_reaches_popen()
    test_import_smoke()
    test_hook_serialisation_smoke()
    test_r1_preserved()
    test_new_leaf()
    test_prereg_and_go()
    test_gates_unmoved()
    test_preflight()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
