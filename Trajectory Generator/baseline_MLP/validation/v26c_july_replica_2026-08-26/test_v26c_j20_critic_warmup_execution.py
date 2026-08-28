"""Static test suite for V26C J20 - the critic-only warm-up execution stage.

Launches no child, runs no warm-up, starts no Ray, builds no environment and
performs no restore. It holds the stage to six properties:

  * the runner is STRUCTURALLY incapable of training, sampling, restoring or
    starting a second child - asserted by walking its own AST;
  * the critic-only config differs from the J20 config in EXACTLY one semantic
    key, and that key is the declared one;
  * the command has ONE source - sealed_command() - with exactly one token
    substituted, verified token by token;
  * the GO is fail-closed: a DRAFT cannot start anything, and missing, extra or
    stale pins are refused;
  * the leaf is born invalid, is asserted clean before the child, and can only
    reach RESTORE_AUDIT_PENDING through a passing audit;
  * the verdict can never be a final PASS.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_critic_warmup_execution.py
"""

from __future__ import annotations

import ast
import hashlib
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

sys.dont_write_bytecode = True

import yaml  # noqa: E402

import v26c_j20_critic_warmup_execution as X  # noqa: E402

RUNNER_PATH = HERE / X.RUNNER_NAME
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
RUNNER_TREE = ast.parse(RUNNER_SOURCE)
PREREG = json.loads((HERE / X.PREREG_NAME).read_text(encoding="utf-8"))

CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def called_names(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        target = node.func
        if isinstance(target, ast.Attribute):
            names.add(target.attr)
        elif isinstance(target, ast.Name):
            names.add(target.id)
    return names


def imported_modules(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


# ------------------------------------------ the runner cannot train or restore

def test_structurally_inert() -> None:
    calls = called_names(RUNNER_TREE)
    for forbidden in X.FORBIDDEN_CALLS:
        check("A01 the runner never calls %s()" % forbidden,
              forbidden not in calls, "found in call position")
    imports = imported_modules(RUNNER_TREE)
    for forbidden in X.FORBIDDEN_IMPORTS:
        check("A02 the runner never imports %s" % forbidden,
              forbidden not in imports and not any(
                  name.startswith(forbidden + ".") for name in imports))
    check("A02b the only in-tree module it imports is the readiness runner",
          "v26c_j20_critic_warmup_readiness" in RUNNER_SOURCE
          and "v26c_j20_k1r1_gradient_amendment" not in imports)

    # EXACTLY ONE child launch, and no loop around it.
    popens = [n for n in ast.walk(RUNNER_TREE)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == "Popen"]
    check("A03 there is exactly one Popen in the whole file", len(popens) == 1,
          str(len(popens)))
    check("A04 subprocess.run is never used",
          "subprocess.run" not in RUNNER_SOURCE)
    launch = next((n for n in ast.walk(RUNNER_TREE)
                   if isinstance(n, (ast.FunctionDef,)) and n.name == "launch_once"),
                  None)
    check("A05 the Popen lives in launch_once", launch is not None
          and any(isinstance(i, ast.Call) and isinstance(i.func, ast.Attribute)
                  and i.func.attr == "Popen" for i in ast.walk(launch)))
    check("A06 launch_once contains no loop",
          launch is not None and not any(
              isinstance(i, (ast.For, ast.While, ast.AsyncFor))
              for i in ast.walk(launch)))
    callers = [n.name for n in ast.walk(RUNNER_TREE)
               if isinstance(n, ast.FunctionDef)
               and any(isinstance(i, ast.Call) and isinstance(i.func, ast.Name)
                       and i.func.id == "launch_once" for i in ast.walk(n))]
    check("A07 launch_once is called from exactly one place", len(callers) == 1,
          str(callers))
    # The real invariant is structural: the single launch is not wrapped in an
    # exception handler that could re-enter it, and no function calls it twice.
    run_exec = next(n for n in ast.walk(RUNNER_TREE)
                    if isinstance(n, ast.FunctionDef) and n.name == "run_execution")
    handlers_over_launch = [
        h for h in ast.walk(run_exec) if isinstance(h, ast.Try)
        and any(isinstance(i, ast.Call) and isinstance(i.func, ast.Name)
                and i.func.id == "launch_once" for i in ast.walk(h))]
    check("A08 the single launch is not inside a try/except that could re-enter it",
          handlers_over_launch == [], str(len(handlers_over_launch)))
    launch_calls = [n for n in ast.walk(RUNNER_TREE)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                    and n.func.id == "launch_once"]
    check("A08b launch_once is invoked from exactly one call site",
          len(launch_calls) == 1, str(len(launch_calls)))

    for name in ("run_execution", "launch_once"):
        body = RUNNER_SOURCE[RUNNER_SOURCE.index("def %s" % name):]
        body = body[:body.index("\ndef ")]
        check("A09 %s has no while loop" % name, "while " not in body)

    probe = RUNNER_SOURCE[RUNNER_SOURCE.index("def run_execution"):]
    probe = probe[:probe.index("\ndef ")]
    check("A10 run_execution refuses without a valid GO", 'if not go["valid"]' in probe)
    check("A11 run_execution refuses without a passing preflight",
          'if not report["ok"]' in probe)
    check("A12 run_execution refuses to clobber an existing leaf",
          "refusing to clobber an existing leaf" in probe)
    check("A13 run_execution never calls restore_from_path",
          "restore_from_path" not in calls
          and "restore_from_path(" not in RUNNER_SOURCE)
    check("A13b the result declares the restore not performed and not performable",
          '"performed": False' in probe and '"performable_here": False' in probe)


def test_preflight_is_pure() -> None:
    before = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                    for p in HERE.rglob("*") if p.is_file())
    completed = subprocess.run(
        [sys.executable, str(RUNNER_PATH), "--preflight-only"],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    after = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                   for p in HERE.rglob("*") if p.is_file())
    check("B01 --preflight-only exits 0", completed.returncode == 0,
          completed.stderr[-400:])
    check("B02 --preflight-only writes nothing", before == after,
          str(set(after) ^ set(before)))
    check("B03 --preflight-only reports READY", "READY" in completed.stdout,
          completed.stdout[-400:])

    dry = subprocess.run(
        [sys.executable, str(RUNNER_PATH), "--dry-run"],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    after_dry = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                       for p in HERE.rglob("*") if p.is_file())
    check("B04 --dry-run exits 0 and writes nothing",
          dry.returncode == 0 and after_dry == before)
    check("B05 --dry-run says the verdict cannot be a final PASS",
          X.VERDICT_AWAITING in dry.stdout and "separate stage" in dry.stdout)

    probe = subprocess.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, %r); sys.dont_write_bytecode = True;\n"
         "import v26c_j20_critic_warmup_execution as X;\n"
         "X.preflight(verbose=False);\n"
         "print('torch' in sys.modules, 'ray' in sys.modules)" % str(HERE)],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    check("B06 the preflight imports neither torch nor ray",
          probe.stdout.strip() == "False False",
          probe.stdout.strip() + probe.stderr[-300:])


def test_preflight_passes() -> None:
    report = X.preflight(verbose=False)
    check("B07 the preflight verdict is READY", report["ok"],
          "; ".join(report["problems"]))
    check("B08 every pin matches",
          report["pins_matching"] == report["pins_checked"]
          and report["pins_checked"] == 41,
          "%d/%d" % (report["pins_matching"], report["pins_checked"]))
    check("B09 the destination is absent",
          report["destination"]["leaf_exists"] is False)
    check("B10 j20_runs holds only the K1 and K1R1 leaves",
          report["destination"]["j20_runs_contents"]
          == sorted([X.K1_LEAF_REL.split("/")[-1],
                     X.K1R1_LEAF_REL.split("/")[-1]]))
    check("B11 the preflight declares it performs no restore",
          report["it_performs_no_restore"] is True)
    check("B12 the best available verdict is AWAITING_RESTORE_AUDIT",
          report["best_available_verdict"] == X.VERDICT_AWAITING)


# ----------------------------------------------------- the config amendment

def test_config_amendment() -> None:
    base = yaml.safe_load((HERE / X.J20_CONFIG_NAME).read_text(encoding="utf-8"))
    amended = yaml.safe_load((HERE / X.CONFIG_NAME).read_text(encoding="utf-8"))
    diff = []
    for section in sorted(set(base) | set(amended)):
        left, right = base.get(section) or {}, amended.get(section) or {}
        for key in sorted(set(left) | set(right)):
            if left.get(key) != right.get(key):
                diff.append((section, key, left.get(key), right.get(key)))

    check("C01 there is EXACTLY one semantic difference", len(diff) == 1, str(diff))
    check("C02 it is supervision.max_minibatch_mean_kl_loss",
          diff and diff[0][0] == "supervision"
          and diff[0][1] == "max_minibatch_mean_kl_loss", str(diff))
    check("C03 it goes from 0.01 to null",
          diff and diff[0][2] == 0.01 and diff[0][3] is None, str(diff))
    for section in X.CONFIG_SECTIONS_MUST_BE_IDENTICAL:
        check("C04 section %s is identical between the two configs" % section,
              base.get(section) == amended.get(section))
    check("C05 the amended config disables the guard",
          amended["supervision"]["max_minibatch_mean_kl_loss"] is None)
    check("C06 exact_start_sampling stays false",
          amended["parallelism"]["exact_start_sampling"] is False)
    check("C07 the July mechanics survive the amendment",
          amended["ppo"]["train_batch_size"] == 4096
          and amended["ppo"]["num_epochs"] == 10
          and amended["ppo"]["lr"] == 1e-04
          and amended["parallelism"]["num_env_runners"] == 13
          and amended["simulation"]["iterations"] == 1)
    check("C08 the freeze contract survives the amendment",
          amended["model"]["freeze_actor"] is True
          and amended["model"]["freeze_logstd"] is True
          and amended["model"]["asymmetric_actor_critic"] is True)
    check("C09 the reward section is untouched, all 125 keys",
          len(amended["reward"]) == 125
          and amended["reward"] == base["reward"])

    # The J20 config must be BYTE-unchanged: this stage may not overwrite it.
    check("C10 the J20 config is byte-unchanged",
          sha256_file(HERE / X.J20_CONFIG_NAME) == X.PIN_J20_CONFIG)
    check("C11 the amended config matches its pin",
          sha256_file(HERE / X.CONFIG_NAME) == X.PIN_CONFIG)

    report = X.check_config_amendment()
    check("C12 the runner's own check agrees", report["ok"]
          and report["difference_count"] == 1, str(report["problems"]))
    check("C13 the runner records why the guard must be off",
          "kl_update" in report["why_the_guard_must_be_off"]
          and "exact-start" in report["why_the_guard_must_be_off"])
    check("C14 the runner records what replaces it",
          "byte-identical" in report["what_replaces_it"]
          and "1e-09" in report["what_replaces_it"])

    # The blocker is real: prove the guard would fire on the stock metric set.
    sys.path.insert(0, str(BASELINE))
    import train_ppo_mlp as trainer

    stock = {"learners/default_policy/mean_kl_loss": 0.0,
             "learners/default_policy/vf_loss": 0.34,
             "learners/default_policy/vf_explained_var": 0.6}
    extracted = trainer._kl_update_metrics(stock)
    check("C15 the stock learner emits no kl_update metrics",
          all(v is None for v in extracted.values()), str(extracted))
    try:
        trainer._enforce_kl_update_guard(
            stock, max_minibatch_mean_kl_loss=0.01, logical_iteration=1)
        fired = False
    except RuntimeError:
        fired = True
    check("C16 with the guard at 0.01 the child WOULD abort", fired,
          "this is why the amendment exists")
    check("C17 with the guard at null it does not fire",
          trainer._enforce_kl_update_guard(
              stock, max_minibatch_mean_kl_loss=None, logical_iteration=1) is None)


# ---------------------------------------------------------------- the command

def test_command() -> None:
    leaf = str(HERE / X.LEAF_ROOT / X.LEAF_NAME)
    command = X.execution_command(leaf)
    sealed = command["sealed_tokens"]
    tokens = command["tokens"]
    check("D01 the command has one source: sealed_command()",
          "sealed_command" in command["source"])
    check("D02 exactly one token differs from the sealed command",
          sum(1 for a, b in zip(sealed, tokens) if a != b) == 1
          and len(sealed) == len(tokens))
    check("D03 the differing token is the --config value",
          tokens[tokens.index("--config") + 1] == str(HERE / X.CONFIG_NAME)
          and sealed[sealed.index("--config") + 1] == str(HERE / X.J20_CONFIG_NAME))
    check("D04 the command is one-shot: --worker-process",
          "--worker-process" in tokens)
    check("D05 exactly one iteration",
          tokens[tokens.index("--iterations") + 1] == "1")
    check("D06 no --resume-from", "--resume-from" not in tokens)
    check("D07 the output dir is the canonical leaf, absolute",
          tokens[tokens.index("--output-dir") + 1] == leaf
          and pathlib.Path(leaf).is_absolute())
    check("D08 the output dir is outside Trajectory Generator/runs",
          "Trajectory Generator/runs" not in leaf,
          "otherwise _resolve_category_output_dir would relocate it")
    check("D09 both freezes are explicit",
          "--freeze-actor" in tokens and "--freeze-logstd" in tokens)
    check("D10 the overlay is passed explicitly, not auto-discovered",
          tokens[tokens.index("--warm-start-raw-source-feature-manifest") + 1]
          == str(HERE / X.OVERLAY_NAME))
    check("D11 the registry is not touched", "--no-update-history" in tokens)
    check("D12 the milestone is retained, for gate G9",
          "--retain-iteration-checkpoints" in tokens)
    check("D13 the token count matches the preregistration",
          len(tokens) == PREREG["the_command"]["token_count"] == 23)

    # It must REFUSE to substitute if the sealed command ever stops carrying the
    # J20 config at that position.
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("def execution_command"):]
    source = source[:source.index("\n# ---")]
    check("D14 the substitution verifies the token it replaces",
          "the sealed command's config is" in source)
    check("D15 the substitution refuses if any other token moved",
          "the substitution touched" in source)
    check("D16 the readiness runner's bytes are verified before import",
          "the pinned readiness runner changed" in RUNNER_SOURCE)


# ---------------------------------------------------------------------- the GO

def valid_go(status: str | None = None) -> dict:
    payload = {
        "kind": "ARCHITECT GO - SINGLE EXECUTION",
        "stage": X.GO_REQUIRED_STAGE,
        "authorises_execution": True,
        "pinned_artefacts_sha256": {
            label: sha256_file(path)
            for label, path in X.go_pin_targets().items()
        },
    }
    if status is not None:
        payload["status"] = status
    return payload


def test_go_validation() -> None:
    check("E01 a well-formed GO validates", X.validate_go(valid_go())["valid"])
    check("E02 a DRAFT GO is REFUSED",
          not X.validate_go(valid_go("DRAFT"))["valid"],
          "a draft must not be able to start anything")
    check("E03 a PROPOSED GO is REFUSED",
          not X.validate_go(valid_go("PROPOSED"))["valid"])

    bad = valid_go(); bad["stage"] = X.READINESS_STAGE
    check("E04 a GO for the readiness stage is refused here",
          not X.validate_go(bad)["valid"])
    bad = valid_go(); bad["authorises_execution"] = "yes"
    check("E05 authorises_execution must be exactly true",
          not X.validate_go(bad)["valid"])
    for forbidden in ("authorises_restore", "authorises_retry", "authorises_ppo",
                      "authorises_ex_novo", "authorises_promotion",
                      "authorises_rewriting_k1"):
        bad = valid_go(); bad[forbidden] = True
        check("E06 a warm-up GO may not set %s" % forbidden,
              not X.validate_go(bad)["valid"])
    bad = valid_go(); bad["pinned_artefacts_sha256"].pop(X.PREREG_NAME)
    check("E07 a missing pin is refused", not X.validate_go(bad)["valid"])
    bad = valid_go(); bad["pinned_artefacts_sha256"][X.PREREG_NAME] = "0" * 64
    check("E08 a stale pin is refused", not X.validate_go(bad)["valid"])
    bad = valid_go(); bad["pinned_artefacts_sha256"]["some_other_file.py"] = "0" * 64
    check("E09 a pin outside the closed map is refused",
          not X.validate_go(bad)["valid"])
    check("E10 an absent GO file is refused, not defaulted",
          not X.load_go(str(HERE / "no_such_go.json"))["valid"])

    targets = X.go_pin_targets()
    check("E11 the closed map has 41 labels", len(targets) == 41, str(len(targets)))
    check("E12 it pins this stage's three artefacts",
          {X.PREREG_NAME, X.RUNNER_NAME, X.TEST_NAME} <= set(targets))
    check("E13 it pins BOTH configs and the overlay and the deriver",
          {X.CONFIG_NAME, X.J20_CONFIG_NAME, X.OVERLAY_NAME,
           X.DERIVER_NAME} <= set(targets))
    check("E14 it pins the readiness runner, prereg and GO",
          {X.READINESS_RUNNER_NAME, X.READINESS_PREREG_NAME,
           X.READINESS_GO_NAME} <= set(targets))
    check("E15 it pins the K1R1 prereg, runner and GO",
          {X.K1R1_PREREG_NAME, X.K1R1_RUNNER_NAME, X.K1R1_GO_NAME} <= set(targets))
    check("E16 it pins the four production modules the CHILD executes",
          set(X.BASELINE_MODULE_LABELS) <= set(targets))
    check("E17 it pins the telemetry suite and the pinned runtime config",
          {X.TELEMETRY_TEST_LABEL, X.RUNTIME_CONFIG_LABEL} <= set(targets))
    check("E18 it pins all seven J19A leaf files",
          {X.J19A_LEAF_REL + "/" + n for n in X.PIN_J19A_LEAF} <= set(targets))
    check("E19 it pins the J19B and J19C qualification evidence",
          set(X.PIN_J19B) <= set(targets) and set(X.PIN_J19C) <= set(targets))
    check("E20 it pins all six K1 and all five K1R1 leaf files",
          {X.K1_LEAF_REL + "/" + n for n in X.PIN_K1_LEAF} <= set(targets)
          and {X.K1R1_LEAF_REL + "/" + n for n in X.PIN_K1R1_LEAF} <= set(targets))
    check("E21 no report is pinned",
          not any(l.endswith(".md") or "reports/" in l for l in targets))
    check("E22 labels outside this directory are resolved by the map",
          not (HERE / X.TELEMETRY_TEST_LABEL).exists()
          and targets[X.TELEMETRY_TEST_LABEL].is_file())
    body = RUNNER_SOURCE[RUNNER_SOURCE.index("def validate_go"):]
    body = body[:body.index("def load_go")]
    check("E23 validate_go resolves paths from the map, never from the payload",
          "targets = go_pin_targets()" in body and "path = targets[label]" in body)


# ------------------------------------------------ the leaf and the verdict

def test_leaf_discipline() -> None:
    body = RUNNER_SOURCE[RUNNER_SOURCE.index("def run_execution"):]
    body = body[:body.index("\ndef ")]
    check("F01 the marker is written FIRST, before the child",
          body.index("INVALID_MARKER") < body.index("launch_once"))
    check("F02 the leaf is asserted clean before the child",
          "the destination is not clean" in body
          and body.index("the destination is not clean") < body.index("launch_once"))
    check("F03 the clean-leaf assertion names the real hazard",
          "inherited train_iterations.jsonl or checkpoint" in body)
    check("F04 input hashes are taken before AND after the child",
          body.count("sha256_file(HERE / J19A_LEAF_REL / n)") == 2)

    commit = RUNNER_SOURCE[RUNNER_SOURCE.index("def verify_commit"):]
    commit = commit[:commit.index("\n# ---")]
    # The early return is the gate: nothing after it can run unless the audit
    # passed. Check the ORDER of the real statements, not a fragile substring.
    guard = commit.index("if problems or not immediate_ok:")
    pending_write = commit.index("(leaf / PENDING_MARKER).write_bytes(")
    unlink = commit.index("(leaf / INVALID_MARKER).unlink()")
    check("F05 the pending marker is written only after the pass-guard",
          guard < pending_write,
          "guard at %d, pending write at %d" % (guard, pending_write))
    check("F05b the guard returns early, so a failure never reaches the swap",
          "return verification" in commit[guard:pending_write])
    check("F06 the invalid marker is removed LAST", unlink > pending_write)
    check("F07 the J19A, K1 and K1R1 leaves are re-hashed at commit time",
          "PIN_J19A_LEAF" in commit and "PIN_K1_LEAF" in commit
          and "PIN_K1R1_LEAF" in commit)
    check("F08 a commit problem keeps the invalid marker",
          '"marker": PENDING_MARKER if (not problems and immediate_ok)' in commit)


def test_verdict_can_never_be_pass() -> None:
    check("G01 the runner defines only two verdicts",
          X.VERDICT_AWAITING == "AWAITING_RESTORE_AUDIT"
          and X.VERDICT_FAILED == "FAIL_CLOSED")
    # "PASS" appears legitimately in check_entry_evidence, where J19B's and
    # J19C's OWN verdicts are read. What matters is that THIS stage never
    # assigns it: the verdict comes from the two constants and nothing else.
    verdict_block = RUNNER_SOURCE[RUNNER_SOURCE.index("    verdict = VERDICT_AWAITING"):]
    verdict_block = verdict_block[:verdict_block.index("    result = {")]
    check("G02 this stage never assigns the verdict PASS",
          '"PASS"' not in verdict_block and "'PASS'" not in verdict_block
          and verdict_block.count("VERDICT_") == 2, verdict_block.strip())
    entry_block = RUNNER_SOURCE[RUNNER_SOURCE.index("def check_entry_evidence"):]
    entry_block = entry_block[:entry_block.index("\ndef ")]
    check("G02b the only PASS strings read OTHER stages' verdicts",
          RUNNER_SOURCE.count('"PASS"') == entry_block.count('"PASS"'),
          "J19B and J19C are read, never restated")
    body = RUNNER_SOURCE[RUNNER_SOURCE.index("    immediate_ok = bool("):]
    body = body[:body.index("    result = {")]
    check("G03 the verdict is AWAITING or FAILED, nothing else",
          "VERDICT_AWAITING if immediate_ok else VERDICT_FAILED" in body)
    check("G04 immediate_ok requires every gate AND every leaf unchanged",
          "all(graded[\"gates\"].values())" in body
          and 'leaf_hashes["j19a_unchanged"]' in body
          and 'leaf_hashes["k1_unchanged"]' in body
          and 'leaf_hashes["k1r1_unchanged"]' in body)
    check("G05 the result explains why it is not PASS",
          "why_not_PASS" in RUNNER_SOURCE
          and "SEPARATE stage" in RUNNER_SOURCE)
    check("G06 nothing is promoted and nothing is training-ready",
          '"promotion": "NONE"' in RUNNER_SOURCE
          and '"training_ready": False' in RUNNER_SOURCE
          and '"next_stage_authorized": False' in RUNNER_SOURCE)
    check("G07 the restore is declared not performed and not performable",
          '"performed": False' in RUNNER_SOURCE
          and '"performable_here": False' in RUNNER_SOURCE)


# --------------------------------------------------------------- the gates

def test_gate_definitions() -> None:
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("    gates = {"):]
    source = source[:source.index("    measurements = {")]
    ids = [line.split('"')[1].split("_")[0]
           for line in source.splitlines() if line.strip().startswith('"G')]
    check("H01 twelve gates, G1 through G12",
          ids == ["G%d" % n for n in range(1, 13)], str(ids))
    check("H02 G2 pins the sampled steps at exactly 4096",
          "EXPECTED_SAMPLED_STEPS" in source and X.EXPECTED_SAMPLED_STEPS == 4096.0)
    # Scope the check to G2's own entry: G11 legitimately uses the same constant.
    g2 = source[source.index('"G2_'):]
    g2 = g2[:g2.index('"G3_')]
    check("H02b G2 checks BOTH the lifetime and the per-iteration delta",
          "lifetime is not None" in g2 and "this_iteration is not None" in g2
          and g2.count("EXPECTED_SAMPLED_STEPS") == 2,
          "a gate reading only the lifetime would not notice if the two ever "
          "stopped coinciding")
    check("H02c the per-iteration key is the one the trainer really emits",
          X.METRIC_STEPS_THIS_ITERATION
          == "learners/__all_modules__/learner_connector_sum_episodes_length_in")
    check("H03 G3 requires finiteness and NO threshold",
          "finite(vf_loss) and finite(vf_ev)" in source
          and "vf_ev >" not in source and "vf_ev <" not in source)
    check("H04 G4 bounds the KL at 1e-09", "KL_TOLERANCE" in source
          and X.KL_TOLERANCE == 1e-09)
    check("H05 G5 requires the pinned actor digest at every audit entry",
          "PIN_J19A_ACTOR_DIGEST" in source)
    check("H06 G6 checks the log-std rows and sigma separately",
          "logstd_rows_byte_identical" in source and "SIGMA" in source)
    check("H07 G7 requires the critic digest to change",
          "digests[0] != digests[-1]" in source)
    check("H08 G8 requires the file hashes to differ",
          "file_hashes_differ" in source)
    check("H09 G9 is the STRUCTURAL half only",
          "G9_structural" in source)
    check("H09b G9 requires the EXACT Adam index set, not merely six entries",
          X.EXPECTED_ADAM_STATE_INDICES == (6, 7, 8, 9, 10, 11))
    audit = RUNNER_SOURCE[RUNNER_SOURCE.index("def audit_checkpoint"):]
    audit = audit[:audit.index("\ndef ")]
    check("H09c audit_checkpoint compares the index tuple",
          "indices != EXPECTED_ADAM_STATE_INDICES" in audit
          and "only_the_critic_has_adam_moments" in audit
          and 'report["only_the_critic_has_adam_moments"] = \\\n        indices == EXPECTED_ADAM_STATE_INDICES'
          in audit)
    check("H09d it explains why a count alone is not enough",
          "the count alone cannot tell" in audit)
    check("H10 G10 reads the absence of supervisor_state.json",
          "supervisor_state.json" in source)
    check("H11 G11 requires completeness, not a bound on incidents",
          "health_present" in source and "missing" in source
          and "phase_timeout_stance_rows" not in source)
    check("H12 G12 requires the J19A leaf unchanged", "j19a_unchanged" in source)

    measurements = RUNNER_SOURCE[RUNNER_SOURCE.index("    measurements = {"):]
    measurements = measurements[:measurements.index("    return {\"gates\"")]
    check("H12b vf_loss falls back to the literal learner_metrics key",
          "vf_loss = metrics.get(METRIC_VF_LOSS)" in RUNNER_SOURCE
          and X.METRIC_VF_LOSS == "learners/default_policy/vf_loss")
    check("H12c the source actually used is recorded",
          '"vf_loss_source"' in RUNNER_SOURCE)
    check("H12d vf_explained_var is only ever read from learner_metrics",
          "vf_ev = metrics.get(METRIC_VF_EV)" in RUNNER_SOURCE
          and 'row.get("vf_explained_var")' not in RUNNER_SOURCE,
          "the trainer never hoists it to a row key")
    check("H13 the incident counters are RECORDED, not bounded",
          "recorded_not_bounded" in measurements
          and "training_health" in measurements)
    for field in ("phase_timeout_stance_rows",
                  "morphology_causal_contract_failure_rows",
                  "resync_event_rows", "resync_count_max",
                  "hs_cancelled_count_max"):
        check("H14 %s is in the recorded-not-bounded set" % field,
              field in X.TRAINING_HEALTH_INCIDENT_FIELDS)


def test_checkpoint_layout_is_read_not_assumed() -> None:
    """The layout constants must match a REAL committed July checkpoint."""
    july = (HERE.parents[3] / "validation" / "critic_warmup"
            / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
            / X.CHECKPOINT_DIR)
    check("I01 the July ground-truth checkpoint is present", july.is_dir(), str(july))
    if not july.is_dir():
        return
    check("I02 the optimizer path exists in the real checkpoint",
          (july / X.OPTIMIZER_REL).is_file(), X.OPTIMIZER_REL)
    check("I03 the learner module path exists in the real checkpoint",
          (july / X.LEARNER_MODULE_REL).is_file(), X.LEARNER_MODULE_REL)

    import pickle

    for decoy in X.EMPTY_DECOY_PATHS:
        path = july / decoy
        empty = path.is_file() and len(pickle.loads(path.read_bytes())) == 0
        check("I04 the decoy %s really is an empty dict" % decoy, empty,
              "an auditor that opened this would wrongly conclude it was missing")

    optimizer = pickle.loads((july / X.OPTIMIZER_REL).read_bytes())
    check("I05 the optimizer top-level keys match the constant",
          tuple(sorted(optimizer)) == X.OPTIMIZER_TOP_KEYS, str(sorted(optimizer)))
    entry = optimizer["optimizer"][X.OPTIMIZER_ENTRY]["state"]
    check("I06 the param group holds twelve parameters",
          len(entry["param_groups"][0]["params"]) == X.EXPECTED_PARAM_GROUP_SIZE)
    check("I07 exactly six parameters carry Adam moments: the critic tensors",
          len(entry["state"]) == X.EXPECTED_ADAM_STATE_ENTRIES
          and sorted(entry["state"]) == [6, 7, 8, 9, 10, 11],
          "a frozen actor takes no gradient, so it can never acquire a moment")

    check("I07b the Adam indices in the real artefact are exactly the expected set",
          tuple(sorted(int(i) for i in entry["state"]))
          == X.EXPECTED_ADAM_STATE_INDICES)

    # The per-iteration key must exist in a real row, and must be the DELTA.
    rows = [json.loads(line) for line in
            (july.parent / "train_iterations.jsonl").read_text().splitlines()
            if line.strip()]
    metrics = rows[0]["learner_metrics"]
    check("I07c the per-iteration key exists in a real July row",
          X.METRIC_STEPS_THIS_ITERATION in metrics,
          X.METRIC_STEPS_THIS_ITERATION)
    check("I07d in that row it equals the lifetime, as it must on iteration one",
          metrics.get(X.METRIC_STEPS_THIS_ITERATION)
          == rows[0]["num_env_steps_sampled_lifetime"] == 4096.0)

    august = (HERE.parents[3] / "validation" / "critic_warmup"
              / "2026-08-20_B0820_native_v26_frozen_actor_iter5"
              / "train_iterations.jsonl")
    if august.is_file():
        arows = [json.loads(l) for l in august.read_text().splitlines() if l.strip()]
        deltas = [r["learner_metrics"].get(X.METRIC_STEPS_THIS_ITERATION)
                  for r in arows]
        lifetimes = [r["num_env_steps_sampled_lifetime"] for r in arows]
        check("I07e over five real iterations the key is the DELTA, not the "
              "lifetime",
              len(set(deltas)) == 1 and len(set(lifetimes)) == 5,
              "deltas %s vs lifetimes %s" % (deltas, lifetimes))

    module = pickle.loads((july / X.LEARNER_MODULE_REL).read_bytes())
    check("I08 the learner module state holds sixteen keys", len(module) == 16)
    check("I09 it carries both towers",
          all(k in module for k in X.ACTOR_KEYS)
          and all(k in module for k in X.CRITIC_KEYS))


# ------------------------------------------------------- the preregistration

def test_prereg() -> None:
    check("J01 the preregistration is sealed to a real hash",
          len(X.PIN_PREREG) == 64
          and all(c in "0123456789abcdef" for c in X.PIN_PREREG), X.PIN_PREREG)
    check("J02 the sealed hash is the preregistration's actual hash",
          X.PIN_PREREG == sha256_file(HERE / X.PREREG_NAME))
    check("J03 the check is UNCONDITIONAL", "SEALED_BEFORE_ANY_GO" not in RUNNER_SOURCE)
    check("J04 the preregistration contains no self-hash",
          X.PIN_PREREG not in (HERE / X.PREREG_NAME).read_text(encoding="utf-8")
          and PREREG["contains_no_self_hash"] is True)
    check("J05 it declares the stage and what it executes",
          PREREG["stage"] == X.STAGE and "sealed warm-up command" in PREREG["executes"])
    check("J06 it states NOT EXECUTED", "NOT EXECUTED" in PREREG["status"])

    amendment = PREREG["the_config_amendment"]
    check("J07 the prereg names the one difference exactly",
          amendment["the_only_difference"] == {
              "section": "supervision", "key": "max_minibatch_mean_kl_loss",
              "from": 0.01, "to": None})
    check("J08 it records the section correction",
          "not under `training:`" in amendment["note_on_the_section"])
    check("J09 it records that the blocker was measured, not predicted",
          "PURE FUNCTION" in amendment["why_this_amendment_is_necessary"]["measured_not_predicted"])
    check("J10 it records the July precedent",
          "no such guard" in amendment["why_this_amendment_is_necessary"]["july_precedent"])
    check("J11 it declares the J20 config not modified",
          amendment["derived_from_sha256"] == X.PIN_J20_CONFIG
          and any("read, never written" in item
                  for item in PREREG["invariants"]["not_modified"]))

    check("J12 twelve gates are preregistered",
          len(PREREG["the_immediate_gates"]) == 12)
    check("J13 the gate ids are G1..G12",
          [g["id"] for g in PREREG["the_immediate_gates"]]
          == ["G%d" % n for n in range(1, 13)])
    check("J14 G9 is declared structural-half-only",
          "STRUCTURAL HALF ONLY" in PREREG["the_immediate_gates"][8]["criterion"])
    check("J15 the prereg forbids a final PASS",
          "AWAITING_RESTORE_AUDIT, not PASS"
          in PREREG["the_verdict_cannot_be_PASS"]["rule"])
    check("J16 the prereg records the 4096 semantics",
          "NOT a number of environments" in PREREG["the_immediate_gates"][1]["note"])
    check("J17 the telemetry is declared recorded, not bounded",
          "the values are MEASUREMENTS"
          in PREREG["telemetry_is_recorded_not_bounded"]["rule"])
    check("J18 the five incident fields match the runner",
          tuple(PREREG["telemetry_is_recorded_not_bounded"]
                ["recorded_without_any_behavioural_threshold"])
          == X.TRAINING_HEALTH_INCIDENT_FIELDS)
    check("J19 the prereg's pin count matches the map",
          PREREG["go_requirements"]["pin_count"] == len(X.go_pin_targets()) == 41)
    check("J20 the prereg's forbidden lists match the runner",
          tuple(PREREG["execution_discipline"]["structurally_forbidden_calls"])
          == X.FORBIDDEN_CALLS
          and tuple(PREREG["execution_discipline"]["structurally_forbidden_imports"])
          == X.FORBIDDEN_IMPORTS)
    check("J21 the prereg names the checkpoint decoys",
          "decoys" in PREREG["the_immediate_gates"][8]["note"])
    check("J22 the prereg records the six-Adam-entry reading of the freeze",
          "only six carry Adam moments"
          in PREREG["the_six_adam_entries_are_an_independent_reading_of_the_freeze"])
    review = PREREG["architect_review_2026_08_27"]
    check("J22b the prereg records the G2 correction",
          "per-iteration delta" in review["1_G2_checks_both_figures"])
    check("J22c the prereg records the G9 index-set correction",
          "[6,7,8,9,10,11]" in review["2_G9_checks_the_index_set"])
    check("J22d the prereg records the vf_loss fallback",
          "fallback" in review["3_vf_loss_has_a_fallback"].lower()
          or "otherwise the" in review["3_vf_loss_has_a_fallback"])
    check("J22e G2's criterion names both figures",
          "BOTH figures" in PREREG["the_immediate_gates"][1]["criterion"]
          and X.METRIC_STEPS_THIS_ITERATION.split("/")[-1]
          in PREREG["the_immediate_gates"][1]["criterion"])
    check("J22f G9's criterion names the exact index set",
          "[6, 7, 8, 9, 10, 11]" in PREREG["the_immediate_gates"][8]["criterion"])
    check("J22g G3's criterion names the fallback",
          "falls back" in PREREG["the_immediate_gates"][2]["criterion"])
    check("J23 the prereg refuses a DRAFT GO",
          "DRAFT" in PREREG["go_requirements"]["a_DRAFT_or_PROPOSED_go_is_refused"])


def test_destination_absent() -> None:
    leaf = HERE / X.LEAF_ROOT / X.LEAF_NAME
    check("K01 no warm-up leaf exists", not leaf.exists(), str(leaf))
    check("K02 no staging or lock directory for this stage",
          not any(p.name.startswith((".staging", ".lock"))
                  for p in (HERE / X.LEAF_ROOT).iterdir()))
    check("K03 j20_runs holds exactly the two audited leaves",
          sorted(p.name for p in (HERE / X.LEAF_ROOT).iterdir())
          == sorted([X.K1_LEAF_REL.split("/")[-1],
                     X.K1R1_LEAF_REL.split("/")[-1]]))
    check("K04 no TECHNICAL_INVALID or RESTORE_AUDIT_PENDING anywhere",
          not list((HERE / X.LEAF_ROOT).rglob(X.INVALID_MARKER))
          and not list((HERE / X.LEAF_ROOT).rglob(X.PENDING_MARKER)))


def main() -> int:
    test_structurally_inert()
    test_preflight_is_pure()
    test_preflight_passes()
    test_config_amendment()
    test_command()
    test_go_validation()
    test_leaf_discipline()
    test_verdict_can_never_be_pass()
    test_gate_definitions()
    test_checkpoint_layout_is_read_not_assumed()
    test_prereg()
    test_destination_absent()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
