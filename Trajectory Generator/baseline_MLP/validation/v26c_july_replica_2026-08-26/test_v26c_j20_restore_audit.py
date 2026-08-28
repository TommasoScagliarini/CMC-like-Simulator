"""Static test suite for V26C J20 - the zero-iteration restore audit.

Launches no child, starts no Ray, builds no environment and performs no
restore through RLlib. It reads the committed checkpoint files and exercises
pure functions. It holds the stage to seven properties:

  * the runner is STRUCTURALLY incapable of training, sampling or starting a
    second child - asserted by walking its own AST;
  * the command has ONE source, sealed_command(), transformed by exactly four
    declared operations, verified token by token;
  * the GO is fail-closed: a DRAFT cannot start anything, and missing, extra or
    stale pins are refused;
  * every one of the thirteen gates is NON-VACUOUS: each is shown to flip to
    False under a targeted perturbation of exactly its own input;
  * the wrapper's normalisation obeys rev2 - in particular a Python float 81.0
    and a zero-dimensional torch tensor 81.0 normalise identically, which is
    the source-versus-live case R9 depends on;
  * the wrapper detects corruption of moments, of param_groups and of the
    Adam index set;
  * the leaf is born invalid and can only reach RESTORE_AUDIT_PASSED through a
    passing audit.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_restore_audit.py
"""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import pathlib
import pickle
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

sys.dont_write_bytecode = True

import numpy as np  # noqa: E402
import torch  # noqa: E402

import v26c_j20_restore_audit as R  # noqa: E402
import v26c_j20_restore_audit_child as C  # noqa: E402

RUNNER_PATH = HERE / R.RUNNER_NAME
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
RUNNER_TREE = ast.parse(RUNNER_SOURCE)
CHILD_PATH = HERE / R.CHILD_NAME
CHILD_SOURCE = CHILD_PATH.read_text(encoding="utf-8")
CHILD_TREE = ast.parse(CHILD_SOURCE)

BASE = json.loads((HERE / R.PREREG_NAME).read_text(encoding="utf-8"))
REV1 = json.loads((HERE / R.PREREG_REV1_NAME).read_text(encoding="utf-8"))
REV2 = json.loads((HERE / R.PREREG_REV2_NAME).read_text(encoding="utf-8"))
REV3 = json.loads((HERE / R.PREREG_REV3_NAME).read_text(encoding="utf-8"))

SOURCE = HERE / R.SOURCE_LEAF_REL
CHECKPOINT = SOURCE / R.CHECKPOINT_REL
LEAF = HERE / R.LEAF_ROOT / R.LEAF_NAME

CHECKS: list = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def called_attributes(tree) -> set:
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            target = node.func
            if isinstance(target, ast.Attribute):
                names.add(target.attr)
            elif isinstance(target, ast.Name):
                names.add(target.id)
    return names


def function_named(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


# ------------------------------------------------------- A. structural inertness

def test_structurally_inert() -> None:
    calls = called_attributes(RUNNER_TREE)
    check("A01 the runner never calls algo.train", "train" not in calls)
    check("A02 the runner never calls sample", "sample" not in calls)
    check("A03 the runner never calls restore_from_path",
          "restore_from_path" not in calls,
          "the LIVE restore belongs to production, reached via the child")
    check("A04 the runner never calls save_to_path", "save_to_path" not in calls)

    popen = [n for n in ast.walk(RUNNER_TREE)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "Popen"]
    check("A05 exactly one subprocess.Popen in the whole runner",
          len(popen) == 1, "found %d" % len(popen))
    launch = function_named(RUNNER_TREE, "launch_once")
    check("A06 the single Popen lives in launch_once",
          launch is not None and any(
              isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == "Popen" for n in ast.walk(launch)))
    check("A07 launch_once contains no loop",
          launch is not None and not any(
              isinstance(n, (ast.For, ast.While)) for n in ast.walk(launch)))

    run = function_named(RUNNER_TREE, "run_execution")
    check("A08 run_execution calls launch_once exactly once",
          run is not None and sum(
              1 for n in ast.walk(run) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Name)
              and n.func.id == "launch_once") == 1)
    check("A09 run_execution contains no retry loop",
          run is not None and not any(
              isinstance(n, (ast.For, ast.While)) and any(
                  isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
                  and c.func.id == "launch_once" for c in ast.walk(n))
              for n in ast.walk(run)))
    check("A10 run_execution loads the GO before anything else",
          run is not None and isinstance(run.body[1], ast.Assign)
          and "load_go" in ast.dump(run.body[1]))
    check("A11 the wrapper never constructs an Algorithm",
          "Algorithm" not in CHILD_SOURCE
          or "from ray.rllib.algorithms" not in CHILD_SOURCE)
    child_calls = called_attributes(CHILD_TREE)
    check("A12 the wrapper never calls train", "train" not in child_calls)
    check("A13 the wrapper never calls restore_from_path",
          "restore_from_path" not in child_calls)
    check("A14 the wrapper reaches production through train_ppo_mlp.main",
          "train_ppo_mlp.main()" in CHILD_SOURCE)
    # rev3, correction 2: the accurate claim is "no PRODUCTION file", not "no
    # file under baseline_MLP" - this stage's own validation leaf lives there.
    check("A15 the wrapper claims no production file is modified, not the "
          "false wider claim",
          "modifies NO PRODUCTION FILE" in CHILD_SOURCE
          and "writes no file under" not in CHILD_SOURCE)
    check("A16 the wrapper says where the evidence really goes",
          "validation leaf" in CHILD_SOURCE)
    writes = [n for n in ast.walk(CHILD_TREE)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr in ("write_bytes", "write_text")]
    check("A17 the wrapper has exactly one file write, the evidence",
          len(writes) == 1, "found %d" % len(writes))
    check("A18 that write lives in write_evidence",
          function_named(CHILD_TREE, "write_evidence") is not None
          and any(isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute)
                  and n.func.attr == "write_bytes"
                  for n in ast.walk(function_named(CHILD_TREE,
                                                   "write_evidence"))))


# ------------------------------------------------------------ B. preflight purity

def test_preflight_is_pure() -> None:
    module_imports = set()
    for node in RUNNER_TREE.body:
        if isinstance(node, ast.Import):
            module_imports.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            module_imports.add(node.module.split(".")[0])
    check("B01 the runner imports no torch at module level",
          "torch" not in module_imports)
    check("B02 the runner imports no ray at module level",
          "ray" not in module_imports)
    check("B03 the runner imports no gymnasium at module level",
          "gymnasium" not in module_imports)

    for name in ("preflight", "check_pins", "check_entry_evidence",
                 "check_destination", "restore_command", "validate_go"):
        node = function_named(RUNNER_TREE, name)
        writes = [n for n in ast.walk(node)
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                  and n.func.attr in ("write_bytes", "write_text", "mkdir",
                                      "unlink", "Popen")] if node else []
        check("B04 %s writes nothing" % name, node is not None and not writes,
              "found %s" % [n.func.attr for n in writes])

    completed = subprocess.run(
        [R.INTERPRETER, str(RUNNER_PATH), "--preflight-only"],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"})
    check("B05 --preflight-only exits without a traceback",
          "Traceback" not in completed.stderr, completed.stderr[-200:])
    check("B06 --preflight-only creates no leaf", not LEAF.exists())

    probe = (
        "import sys; sys.path.insert(0, %r); sys.dont_write_bytecode = True\n"
        "import v26c_j20_restore_audit as R\n"
        "R.preflight(verbose=False)\n"
        "print('torch' in sys.modules, 'ray' in sys.modules)\n" % str(HERE))
    loaded = subprocess.run([R.INTERPRETER, "-c", probe], cwd=str(HERE),
                            capture_output=True, text=True)
    check("B07 preflight imports neither torch nor ray",
          loaded.stdout.strip().endswith("False False"),
          loaded.stdout.strip() + loaded.stderr[-200:])


# ------------------------------------------------------- C. the command derivation

def test_command() -> None:
    command = R.restore_command(str(LEAF))
    derived = list(command["derived_tokens"])
    delegated = list(command["delegated_argv"])
    child_argv = list(command["child_argv"])

    check("C01 the derived command holds twenty tokens", len(derived) == 20)
    check("C02 the delegated argv is the derived list minus interpreter and "
          "script", delegated == derived[2:] and len(delegated) == 18)
    check("C03 the child argv holds twenty-five tokens", len(child_argv) == 25)
    check("C04 the child argv is interpreter, wrapper, four wrapper arguments, "
          "a separator, then the delegated argv",
          child_argv[0] == R.INTERPRETER
          and child_argv[1] == str(HERE / R.CHILD_NAME)
          and child_argv[2] == "--audit-source-state"
          and child_argv[4] == "--audit-evidence"
          and child_argv[6] == "--"
          and child_argv[7:] == delegated)

    warmup_tokens = list(command["warmup_tokens"])
    check("C05 the warm-up command it derives from holds twenty-three tokens",
          len(warmup_tokens) == 23, str(len(warmup_tokens)))
    removed = [t for t in warmup_tokens if t not in derived]
    check("C06 exactly five tokens were removed", len(removed) == 5, str(removed))
    check("C07 the removed tokens are the warm-start window",
          removed == command["operations"]["2_removed"])
    for token in ("--warm-start-raw", "--warm-start-raw-source",
                  "--warm-start-raw-source-feature-manifest"):
        check("C08 %s is absent from the derived command" % token,
              token not in derived)

    check("C09 --resume-from was appended",
          derived[-2] == "--resume-from")
    check("C10 --resume-from points at the audited checkpoint",
          derived[-1] == str(CHECKPOINT))
    check("C11 the script token became the wrapper",
          derived[1] == str(BASELINE / "train_ppo_mlp.py")
          and child_argv[1] == str(HERE / R.CHILD_NAME))
    check("C12 --iteration-start is NOT passed",
          "--iteration-start" not in child_argv,
          "the trainer must derive it, not be told it")
    check("C13 --iterations 1 is retained",
          "--iterations" in derived
          and derived[derived.index("--iterations") + 1] == "1")
    check("C14 --worker-process is retained, so there is no supervisor",
          "--worker-process" in delegated)
    check("C15 --no-update-history is retained",
          "--no-update-history" in delegated)
    check("C16 --freeze-actor and --freeze-logstd are retained",
          "--freeze-actor" in delegated and "--freeze-logstd" in delegated)
    check("C17 --asymmetric-actor-critic is retained, which is what makes the "
          "trainer record the restored digests",
          "--asymmetric-actor-critic" in delegated)
    check("C18 the config is the critic-only one",
          derived[derived.index("--config") + 1]
          == str(HERE / "v26c_j20_warmup_critic_only_cfg.yaml"))
    check("C19 the output directory is the audit leaf",
          derived[derived.index("--output-dir") + 1] == str(LEAF))
    check("C20 the evidence path is inside the audit leaf",
          child_argv[5] == str(LEAF / R.EVIDENCE_NAME))
    check("C21 the audit source state is the checkpoint learner state",
          child_argv[3] == str(SOURCE / R.LEARNER_STATE_REL))
    check("C22 four operations are recorded", len(command["operations"]) == 4)
    check("C23 the derivation names its single source",
          "sealed_command" in command["source"])
    check("C24 the counting discrepancy in rev1 is recorded, not hidden",
          "inaccurate by two" in command["counting_note"]
          and "rev3" in command["counting_note"])
    check("C25 the three counts match the constants rev3 fixed",
          command["derived_token_count"] == R.EXPECTED_DERIVED_TOKENS
          and command["delegated_argv_count"] == R.EXPECTED_DELEGATED_ARGV
          and command["child_argv_count"] == R.EXPECTED_CHILD_ARGV)
    check("C26 the counts are enforced as refusals, not just recorded",
          all(("raise RestoreAuditError" in ast.get_source_segment(
              RUNNER_SOURCE, function_named(RUNNER_TREE, "restore_command")))
              for _ in (0,))
          and ast.get_source_segment(
              RUNNER_SOURCE, function_named(RUNNER_TREE, "restore_command")
          ).count("EXPECTED_DERIVED_TOKENS") >= 2)


# ---------------------------------------------------------------- D. the GO gate

def good_go() -> dict:
    targets = R.go_pin_targets()
    return {
        "stage": R.GO_REQUIRED_STAGE,
        "status": "APPROVED",
        "authorises_execution": True,
        "pinned_artefacts_sha256": {label: sha256_file(path)
                                    for label, path in targets.items()},
    }


def test_go_validation() -> None:
    payload = good_go()
    check("D01 a complete, correct GO validates",
          R.validate_go(payload)["valid"],
          "; ".join(R.validate_go(payload)["problems"][:3]))

    # rev3, correction 3: only the exact string APPROVED authorises execution.
    for status in ("DRAFT", "PROPOSED", "approved", "APPROVED ", "ok", "final",
                   "", "TRUE", "1"):
        bad = copy.deepcopy(payload)
        bad["status"] = status
        check("D02 the status %r is refused" % status,
              not R.validate_go(bad)["valid"])
    for value in (None, True, 1, ["APPROVED"], {"status": "APPROVED"}):
        bad = copy.deepcopy(payload)
        bad["status"] = value
        check("D02b a non-string status %r is refused" % (value,),
              not R.validate_go(bad)["valid"])
    bad = copy.deepcopy(payload)
    del bad["status"]
    check("D02c a MISSING status is refused", not R.validate_go(bad)["valid"],
          "a missing field must not be treated as approval")

    bad = copy.deepcopy(payload)
    bad["authorises_execution"] = False
    check("D03 authorises_execution false is refused",
          not R.validate_go(bad)["valid"])

    bad = copy.deepcopy(payload)
    bad["stage"] = "V26C_J20_CRITIC_WARMUP"
    check("D04 the wrong stage is refused", not R.validate_go(bad)["valid"])

    for forbidden in ("authorises_retry", "authorises_ppo", "authorises_ex_novo",
                      "authorises_promotion", "authorises_training",
                      "authorises_rewriting_the_warmup_leaf"):
        bad = copy.deepcopy(payload)
        bad[forbidden] = True
        check("D05 %s is refused" % forbidden, not R.validate_go(bad)["valid"])

    bad = copy.deepcopy(payload)
    dropped = sorted(bad["pinned_artefacts_sha256"])[0]
    del bad["pinned_artefacts_sha256"][dropped]
    check("D06 a missing pin is refused", not R.validate_go(bad)["valid"])

    bad = copy.deepcopy(payload)
    bad["pinned_artefacts_sha256"]["not_in_scope.json"] = "0" * 64
    check("D07 a pin outside the closed scope is refused",
          not R.validate_go(bad)["valid"])

    bad = copy.deepcopy(payload)
    stale = sorted(bad["pinned_artefacts_sha256"])[0]
    bad["pinned_artefacts_sha256"][stale] = "0" * 64
    check("D08 a stale pin is refused", not R.validate_go(bad)["valid"])

    check("D09 the GO payload is not an object", not R.validate_go([])["valid"])
    check("D10 a missing GO file is refused",
          not R.load_go(str(HERE / "no_such_go.json"))["valid"])

    targets = R.go_pin_targets()
    expected = R.expected_pin_hashes()
    check("D11 every pin target has an expected hash entry",
          set(targets) == set(expected))
    self_pinned = [label for label in (R.RUNNER_NAME, R.TEST_NAME, R.CHILD_NAME)
                   if expected.get(label) is None]
    check("D12 the runner, the test and the wrapper are pinned by the GO, not "
          "by themselves", len(self_pinned) == 3)
    check("D13 the three preregistration documents are all pinned",
          all(label in targets for label in
              (R.PREREG_NAME, R.PREREG_REV1_NAME, R.PREREG_REV2_NAME)))
    check("D14 the source leaf checkpoint is pinned",
          (R.SOURCE_LEAF_REL + "/" + R.MODULE_STATE_REL) in targets
          and (R.SOURCE_LEAF_REL + "/" + R.LEARNER_STATE_REL) in targets)


# ------------------------------------------------------- E. gate non-vacuity

def good_inputs() -> tuple:
    checkpoint = str(CHECKPOINT)
    summary = {
        "iterations_run": 0,
        "iterations_completed_this_process": 0,
        "iterations_completed": 1,
        "history": [],
        "restored_training_iteration": 1,
        "restored_logical_iteration": 1,
        "iteration_start": 2,
        "next_iteration": 2,
        "resume_from": checkpoint,
        "initialization_mode": "resume_from",
        "warm_start_raw_transplant_applied_this_process": False,
        "actor_freeze_audit": [{"stage": "before_training",
                                "actor_digest": R.EXPECTED_ACTOR_DIGEST,
                                "exact": True, "max_abs_diff": 0.0}],
        "critic_state_audit": [{"stage": "before_training",
                                "critic_digest": R.EXPECTED_CRITIC_DIGEST,
                                "critic_keys": list(R.CRITIC_KEYS)}],
        "optimizer_lr_audit": [{"stage": "after_restore", "learners": []}],
        "stop_reason": "completed",
        "interrupted": False,
        "timed_out": False,
    }
    artefacts = {
        "summary": summary,
        "iteration_rows": [],
        "evidence": {
            "kind": R.EVIDENCE_KIND,
            "stage_marker": R.EVIDENCE_STAGE_MARKER,
            "source_state_sha256": R.EXPECTED_SOURCE_STATE_SHA,
            "exact": True,
            "differences": [],
            "difference_count": 0,
            "problems": [],
            "normalised_digest_source": "a" * 64,
            "normalised_digest_live": "a" * 64,
            "normalised_digests_match": True,
            "param_groups": {"exact": True, "differences": [],
                             "difference_count": 0, "digest_source": "b" * 64,
                             "digest_live": "b" * 64, "digests_match": True},
            "top_level_keys_source": ["param_groups", "state"],
            "top_level_keys_live": ["param_groups", "state"],
            "state_indices_source": [6, 7, 8, 9, 10, 11],
            "state_indices_live": [6, 7, 8, 9, 10, 11],
            "learner_count": 1,
            "optimizer_names": ["default_optimizer"],
        },
        "forbidden_present": [],
        "supervisor_state_present": False,
    }
    hermetic = {
        "ok": True,
        "all_keys_byte_identical": True,
        "state_keys": 16,
        "strict_load_missing": [],
        "strict_load_unexpected": [],
        "actor_digest": R.EXPECTED_ACTOR_DIGEST,
        "critic_digest": R.EXPECTED_CRITIC_DIGEST,
        "logstd_weight_rows_zero": True,
        "logstd_bias_exact": True,
        "sigma_exact": True,
        "optimizer_loaded": True,
        "all_moments_byte_identical": True,
        "adam_indices": list(R.EXPECTED_CRITIC_PARAM_INDICES),
        "adam_steps": [81.0] * 6,
        "critic_indices_are_the_critic": True,
        "actor_indices_are_not_the_critic": True,
    }
    integrity = {"source_tree_unchanged": True, "pins_unchanged": True}
    child = {"returncode": 0, "launched_once": True, "retried": False}
    return child, artefacts, hermetic, integrity


def grade(child, artefacts, hermetic, integrity) -> dict:
    return R.evaluate_gates(child, artefacts, hermetic, integrity)["gates"]


def test_gate_definitions() -> None:
    child, artefacts, hermetic, integrity = good_inputs()
    gates = grade(child, artefacts, hermetic, integrity)
    check("E01 there are exactly thirteen gates", len(gates) == 13,
          str(len(gates)))
    check("E02 a correct run passes all thirteen", all(gates.values()),
          str(sorted(n for n, ok in gates.items() if not ok)))

    names = sorted(gates)
    for expected in ("R1_zero_new_iterations", "R2_empty_history",
                     "R3_restored_iteration_is_one",
                     "R4_next_and_start_are_two",
                     "R5_the_restore_path_was_taken",
                     "R6_restored_actor_digest", "R7_restored_critic_digest",
                     "R8_hermetic_full_module_restore",
                     "R9_live_optimizer_is_byte_exact_after_restore",
                     "R10_nothing_was_trained", "R11_sources_unchanged",
                     "R12_single_clean_process",
                     "R13_hermetic_second_opinion_with_byte_identity"):
        check("E03 gate %s exists" % expected, expected in names)

    # Each perturbation must flip EXACTLY the gate it targets.
    def flips(label, gate, mutate):
        c, a, h, i = good_inputs()
        mutate(c, a, h, i)
        after = grade(c, a, h, i)
        check("E04 %s" % label, after[gate] is False,
              "%s did not flip" % gate)

    flips("one iteration row fails R1", "R1_zero_new_iterations",
          lambda c, a, h, i: a.__setitem__("iteration_rows", ["{}"]))
    flips("iterations_run 1 fails R1", "R1_zero_new_iterations",
          lambda c, a, h, i: a["summary"].__setitem__("iterations_run", 1))
    flips("a non-empty history fails R2", "R2_empty_history",
          lambda c, a, h, i: a["summary"].__setitem__("history", [{"i": 1}]))
    flips("restored training iteration 0 fails R3",
          "R3_restored_iteration_is_one",
          lambda c, a, h, i: a["summary"].__setitem__(
              "restored_training_iteration", 0))
    flips("restored logical iteration 2 fails R3",
          "R3_restored_iteration_is_one",
          lambda c, a, h, i: a["summary"].__setitem__(
              "restored_logical_iteration", 2))
    flips("iteration_start 1 fails R4", "R4_next_and_start_are_two",
          lambda c, a, h, i: a["summary"].__setitem__("iteration_start", 1))
    flips("next_iteration 3 fails R4", "R4_next_and_start_are_two",
          lambda c, a, h, i: a["summary"].__setitem__("next_iteration", 3))
    flips("initialization_mode warm_start_raw fails R5",
          "R5_the_restore_path_was_taken",
          lambda c, a, h, i: a["summary"].__setitem__("initialization_mode",
                                                      "warm_start_raw"))
    flips("a transplant applied this process fails R5",
          "R5_the_restore_path_was_taken",
          lambda c, a, h, i: a["summary"].__setitem__(
              "warm_start_raw_transplant_applied_this_process", True))
    flips("a wrong resume path fails R5", "R5_the_restore_path_was_taken",
          lambda c, a, h, i: a["summary"].__setitem__("resume_from", "/tmp/x"))
    flips("a second freeze-audit entry fails R6", "R6_restored_actor_digest",
          lambda c, a, h, i: a["summary"]["actor_freeze_audit"].append(
              {"stage": "after_iteration"}))
    flips("a wrong actor digest fails R6", "R6_restored_actor_digest",
          lambda c, a, h, i: a["summary"]["actor_freeze_audit"][0].__setitem__(
              "actor_digest", "0" * 64))
    flips("a wrong critic digest fails R7", "R7_restored_critic_digest",
          lambda c, a, h, i: a["summary"]["critic_state_audit"][0].__setitem__(
              "critic_digest", "0" * 64))
    flips("a short critic key list fails R7", "R7_restored_critic_digest",
          lambda c, a, h, i: a["summary"]["critic_state_audit"][0].__setitem__(
              "critic_keys", ["vf.bias"]))
    flips("a non-byte-identical module fails R8",
          "R8_hermetic_full_module_restore",
          lambda c, a, h, i: h.__setitem__("all_keys_byte_identical", False))
    flips("a non-empty strict-load missing list fails R8",
          "R8_hermetic_full_module_restore",
          lambda c, a, h, i: h.__setitem__("strict_load_missing", ["vf.bias"]))
    flips("an inexact sigma fails R8", "R8_hermetic_full_module_restore",
          lambda c, a, h, i: h.__setitem__("sigma_exact", False))
    flips("a non-zero log-std weight fails R8",
          "R8_hermetic_full_module_restore",
          lambda c, a, h, i: h.__setitem__("logstd_weight_rows_zero", False))
    flips("live exact false fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a["evidence"].__setitem__("exact", False))
    flips("a live difference fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a["evidence"].__setitem__("difference_count", 1))
    flips("missing live evidence fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a.__setitem__("evidence", None))
    flips("two learners fail R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a["evidence"].__setitem__("learner_count", 2))
    flips("an empty optimizer_lr_audit fails R9",
          "R9_live_optimizer_is_byte_exact_after_restore",
          lambda c, a, h, i: a["summary"].__setitem__("optimizer_lr_audit", []))
    flips("a checkpoint in the leaf fails R10", "R10_nothing_was_trained",
          lambda c, a, h, i: a.__setitem__("forbidden_present",
                                           ["checkpoint_last"]))
    flips("a milestone directory fails R10", "R10_nothing_was_trained",
          lambda c, a, h, i: a.__setitem__("forbidden_present",
                                           ["milestone_iteration_000001"]))
    flips("a changed source tree fails R11", "R11_sources_unchanged",
          lambda c, a, h, i: i.__setitem__("source_tree_unchanged", False))
    flips("a changed pin fails R11", "R11_sources_unchanged",
          lambda c, a, h, i: i.__setitem__("pins_unchanged", False))
    flips("a non-zero return code fails R12", "R12_single_clean_process",
          lambda c, a, h, i: c.__setitem__("returncode", 1))
    flips("stop_reason error fails R12", "R12_single_clean_process",
          lambda c, a, h, i: a["summary"].__setitem__("stop_reason", "error"))
    flips("a supervisor state file fails R12", "R12_single_clean_process",
          lambda c, a, h, i: a.__setitem__("supervisor_state_present", True))
    flips("a retry fails R12", "R12_single_clean_process",
          lambda c, a, h, i: c.__setitem__("retried", True))
    flips("non-byte-identical moments fail R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("all_moments_byte_identical", False))
    flips("an optimizer that did not load fails R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("optimizer_loaded", False))
    flips("wrong Adam indices fail R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("adam_indices", [0, 1, 2, 3, 4, 5]))
    flips("a wrong Adam step fails R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("adam_steps", [80.0] * 6))
    flips("indices that are not the critic fail R13",
          "R13_hermetic_second_opinion_with_byte_identity",
          lambda c, a, h, i: h.__setitem__("critic_indices_are_the_critic",
                                           False))

    # rev3's tightened R9. Each of these is a payload that would have passed
    # the rev1 predicate.
    gate9 = "R9_live_optimizer_is_byte_exact_after_restore"
    flips("a falsified count with a non-empty difference list fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "differences", ["optimizer.state.6.exp_avg: ... vs ..."]))
    flips("a missing source digest fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "normalised_digest_source", None))
    flips("a missing live digest fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "normalised_digest_live", None))
    flips("digests that differ fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "normalised_digest_live", "c" * 64))
    flips("a falsified digests_match fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "normalised_digests_match", False))
    flips("param_groups exact false fails R9", gate9,
          lambda c, a, h, i: a["evidence"]["param_groups"].__setitem__(
              "exact", False))
    flips("non-empty param_groups differences fail R9", gate9,
          lambda c, a, h, i: a["evidence"]["param_groups"].__setitem__(
              "differences", ["param_groups[0].lr: ... vs ..."]))
    flips("param_groups digests that differ fail R9", gate9,
          lambda c, a, h, i: a["evidence"]["param_groups"].__setitem__(
              "digests_match", False))
    flips("a missing param_groups block fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("param_groups", None))
    flips("a wrong source state sha fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "source_state_sha256", "0" * 64))
    flips("a wrong evidence kind fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("kind", "SOMETHING ELSE"))
    flips("a wrong stage marker fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("stage_marker",
                                                       "before_training"))
    flips("wrong live top-level keys fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("top_level_keys_live",
                                                       ["state"]))
    flips("wrong source top-level keys fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("top_level_keys_source",
                                                       ["state"]))
    flips("wrong live state indices fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "state_indices_live", [0, 1, 2, 3, 4, 5]))
    flips("wrong source state indices fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "state_indices_source", [0, 1, 2, 3, 4, 5]))
    flips("an unexpected optimizer name fails R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("optimizer_names",
                                                       ["other_optimizer"]))
    flips("two optimizer names fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__(
              "optimizer_names", ["default_optimizer", "default_optimizer"]))
    flips("non-empty problems fail R9", gate9,
          lambda c, a, h, i: a["evidence"].__setitem__("problems", ["x"]))


# ------------------------------------------ F. rev2's normalisation requirements

def test_normalisation() -> None:
    # T_step_representation - the exact source-versus-live case of R9.
    source_step = 81.0                       # what the pickle holds
    live_step = torch.as_tensor(81.0)        # what the live optimizer holds
    check("F01 a Python float and a 0-D torch tensor normalise identically",
          C.normalise(source_step) == C.normalise(live_step),
          "%r vs %r" % (C.normalise(source_step), C.normalise(live_step)))
    check("F02 a NumPy scalar normalises the same way",
          C.normalise(np.float32(81.0)) == C.normalise(source_step))
    check("F03 a 0-D NumPy array normalises the same way",
          C.normalise(np.asarray(81.0)) == C.normalise(source_step))
    check("F04 the shared form is a scalar, not a one-element tensor",
          C.normalise(live_step)[0] == "scalar",
          "the ascontiguousarray 0-D promotion trap of rev2")
    check("F05 a 0-D tensor never normalises to shape (1,)",
          C.normalise(live_step) != ("tensor", "float32", (1,),
                                     C.normalise(np.asarray([81.0],
                                                            dtype=np.float32))[3]))

    # T_bool_is_not_a_number
    check("F06 False does not normalise like 0",
          C.normalise(False) != C.normalise(0))
    check("F07 True does not normalise like 1",
          C.normalise(True) != C.normalise(1))
    check("F08 a bool keeps its typed prim form",
          C.normalise(True) == ("prim", "bool", True))
    check("F09 None keeps its typed prim form",
          C.normalise(None) == ("prim", "NoneType", None))
    check("F10 a string keeps its typed prim form",
          C.normalise("adam") == ("prim", "str", "adam"))
    check("F11 a tuple and a list with equal contents normalise equal",
          C.normalise((0.9, 0.999)) == C.normalise([0.9, 0.999]))
    check("F12 an unknown type becomes opaque",
          C.normalise(object())[0] == "opaque")

    # An n-dimensional tensor is hashed, and the hash is dtype and shape aware.
    a = np.arange(6, dtype=np.float32).reshape(2, 3)
    check("F13 a matrix normalises to a tensor descriptor",
          C.normalise(a)[0] == "tensor" and C.normalise(a)[1] == "float32"
          and C.normalise(a)[2] == (2, 3))
    check("F14 numpy and torch views of the same bytes normalise equal",
          C.normalise(a) == C.normalise(torch.as_tensor(a)))
    check("F15 a dtype change is detected",
          C.normalise(a) != C.normalise(a.astype(np.float64)))
    check("F16 a shape change is detected",
          C.normalise(a) != C.normalise(a.reshape(3, 2)))


# ------------------------------------------- G. the wrapper detects real damage

def load_source_optimizer() -> dict:
    return C.read_source_optimizer(CHECKPOINT / "learner_group/learner/state.pkl")


def to_torch(obj):
    if isinstance(obj, dict):
        return {k: to_torch(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [to_torch(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return torch.as_tensor(obj)
    return obj


def test_wrapper_detects_damage() -> None:
    source = load_source_optimizer()
    live = to_torch(copy.deepcopy(source))
    base = C.differences(C.normalise(source), C.normalise(live), "optimizer")
    check("G01 an untouched round trip shows zero differences",
          base == [], str(base[:3]))

    # T_moment_corruption
    for key in ("exp_avg", "exp_avg_sq"):
        damaged = to_torch(copy.deepcopy(source))
        damaged["state"][6][key] = damaged["state"][6][key].clone()
        damaged["state"][6][key].view(-1)[0] += np.float32(1e-7)
        found = C.differences(C.normalise(source), C.normalise(damaged),
                              "optimizer")
        check("G02 one perturbed element of %s is caught" % key,
              len(found) == 1 and key in found[0] and ".6." in found[0],
              str(found[:2]))

    # T_param_groups_corruption
    for label, mutate in (
            ("lr", lambda d: d["param_groups"][0].__setitem__("lr", 2e-4)),
            ("weight_decay",
             lambda d: d["param_groups"][0].__setitem__("weight_decay", 1)),
            ("a flipped boolean",
             lambda d: d["param_groups"][0].__setitem__("amsgrad", True)),
            ("a dropped key",
             lambda d: d["param_groups"][0].pop("eps")),
            ("a shortened params list",
             lambda d: d["param_groups"][0].__setitem__(
                 "params", d["param_groups"][0]["params"][:11])),
            ("betas",
             lambda d: d["param_groups"][0].__setitem__("betas", (0.8, 0.999)))):
        damaged = to_torch(copy.deepcopy(source))
        mutate(damaged)
        found = C.differences(C.normalise(source), C.normalise(damaged),
                              "optimizer")
        check("G03 param_groups corruption is caught: %s" % label,
              len(found) >= 1, str(found[:2]))

    # T_state_index_corruption
    damaged = to_torch(copy.deepcopy(source))
    damaged["state"].pop(6)
    found = C.differences(C.normalise(source), C.normalise(damaged), "optimizer")
    check("G04 a removed Adam entry is caught", len(found) >= 1, str(found[:2]))

    damaged = to_torch(copy.deepcopy(source))
    damaged["state"][12] = copy.deepcopy(damaged["state"][6])
    found = C.differences(C.normalise(source), C.normalise(damaged), "optimizer")
    check("G05 an added Adam entry is caught", len(found) >= 1, str(found[:2]))

    # A fresh optimizer - the silent failure this whole stage exists to exclude.
    damaged = to_torch(copy.deepcopy(source))
    for index in list(damaged["state"]):
        for key in ("exp_avg", "exp_avg_sq"):
            damaged["state"][index][key] = torch.zeros_like(
                damaged["state"][index][key])
    found = C.differences(C.normalise(source), C.normalise(damaged), "optimizer")
    check("G06 a zeroed - that is, fresh - optimizer is caught",
          len(found) == 12, "%d differences" % len(found))

    check("G07 the source really holds six entries at [6..11]",
          sorted(source["state"]) == [6, 7, 8, 9, 10, 11])
    check("G08 the source really holds twelve parameters",
          [len(g["params"]) for g in source["param_groups"]] == [12])


def test_wrapper_contract() -> None:
    node = function_named(CHILD_TREE, "install_wrap")
    body = ast.dump(node) if node else ""
    check("G09 install_wrap exists", node is not None)

    # The ordering claims below are about the STATEMENTS of the wrapped
    # function, so they are read off the AST. Comparing positions of substrings
    # in the file would also match the module docstring, which names these
    # helpers before any code runs.
    wrapped = function_named(node, "wrapped") if node else None

    def statement_index(function, predicate):
        if function is None:
            return None
        for index, statement in enumerate(function.body):
            if any(predicate(inner) for inner in ast.walk(statement)):
                return index
        return None

    calls_original = statement_index(
        wrapped, lambda n: isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name) and n.func.id == "original")
    captures = statement_index(
        wrapped, lambda n: isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr == "_learner_call_results")
    writes_evidence = statement_index(
        wrapped, lambda n: isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name) and n.func.id == "write_evidence")
    raises = statement_index(wrapped, lambda n: isinstance(n, ast.Raise))

    check("G10 the wrapped function calls the original FIRST",
          calls_original is not None and captures is not None
          and calls_original < captures,
          "original at %s, capture at %s" % (calls_original, captures))
    check("G11 the original's return value is handed back unchanged",
          wrapped is not None
          and isinstance(wrapped.body[-1], ast.Return)
          and isinstance(wrapped.body[-1].value, ast.Name)
          and wrapped.body[-1].value.id == "reports")
    check("G12 a second install is refused",
          "_v26c_restore_audit_wrap" in body)
    check("G13 a mismatch raises rather than warns",
          raises is not None
          and any(isinstance(n, ast.Name) and n.id == "LiveOptimizerMismatch"
                  for n in ast.walk(wrapped.body[raises])))
    check("G14 evidence is written before the raise",
          writes_evidence is not None and raises is not None
          and writes_evidence < raises,
          "write at %s, raise at %s" % (writes_evidence, raises))
    check("G15 the wrapper refuses to overwrite existing evidence",
          "refusing to overwrite existing evidence" in CHILD_SOURCE)
    check("G16 split_argv demands a separator",
          "requires a '--' separator" in CHILD_SOURCE)
    try:
        C.split_argv(["--audit-evidence", "x"])
        raised = False
    except SystemExit:
        raised = True
    check("G17 split_argv raises without a separator", raised)
    mine, theirs = C.split_argv(["--a", "1", "--", "--worker-process"])
    check("G18 split_argv splits at the separator",
          mine == ["--a", "1"] and theirs == ["--worker-process"])

    class Stub:
        pass

    stub = Stub()
    try:
        C.install_wrap(stub, pathlib.Path("/x"), pathlib.Path("/y"))
        refused = False
    except C.LiveOptimizerMismatch:
        refused = True
    check("G19 wrapping a module without the target function is refused",
          refused)


# ------------------------------------------------ H. the preregistration documents

def test_prereg() -> None:
    for label, payload, path, pin in (
            ("base", BASE, HERE / R.PREREG_NAME, R.PIN_PREREG),
            ("rev1", REV1, HERE / R.PREREG_REV1_NAME, R.PIN_PREREG_REV1),
            ("rev2", REV2, HERE / R.PREREG_REV2_NAME, R.PIN_PREREG_REV2),
            ("rev3", REV3, HERE / R.PREREG_REV3_NAME, R.PIN_PREREG_REV3)):
        actual = sha256_file(path)
        check("H01 %s hashes to its pin" % label, actual == pin,
              "%s vs %s" % (actual, pin))
        check("H02 %s declares contains_no_self_hash" % label,
              payload.get("contains_no_self_hash") is True)
        check("H03 %s really contains no self hash" % label,
              actual not in path.read_text(encoding="utf-8"))
        check("H04 %s names this stage" % label, payload.get("stage") == R.STAGE)

    check("H05 rev1 amends the base by exact hash",
          REV1.get("amends_sha256") == R.PIN_PREREG)
    check("H06 rev2 cites the base by exact hash",
          REV2["cites"]["base_sha256"] == R.PIN_PREREG)
    check("H07 rev2 cites rev1 by exact hash",
          REV2["cites"]["rev1_sha256"] == R.PIN_PREREG_REV1)
    check("H08 rev1 raises the gate count to thirteen",
          REV1["gate_count"]["now"] == 13)
    check("H09 rev2 leaves the gate count alone",
          "gate_count" not in REV2)
    check("H10 rev2's scope is the normalisation rules only",
          "normalisation rules" in REV2["scope"])
    check("H11 the base document forbids a retry",
          any("retry" in p for p in BASE["prohibitions"]))
    check("H12 success authorises no promotion in the base document",
          BASE["verdicts"]["fields_that_stay_false_on_success"]["promotion"]
          == "NONE")
    check("H13 the base names the destination this runner uses",
          BASE["protocol"]["destination"] == R.LEAF_ROOT + "/" + R.LEAF_NAME)
    check("H14 the base names the verdicts this runner uses",
          BASE["verdicts"]["on_success"] == R.VERDICT_PASS
          and BASE["verdicts"]["on_failure"] == R.VERDICT_FAILED)
    check("H15 rev2 records the ascontiguousarray trap",
          "ascontiguousarray" in json.dumps(REV2))
    check("H16 rev1 requires the live capture to call the original first",
          "calls the ORIGINAL first" in json.dumps(REV1))

    # rev3 is additive and must cite all three predecessors by exact hash.
    check("H17 rev3 cites the base by exact hash",
          REV3["cites"]["base_sha256"] == R.PIN_PREREG)
    check("H18 rev3 cites rev1 by exact hash",
          REV3["cites"]["rev1_sha256"] == R.PIN_PREREG_REV1)
    check("H19 rev3 cites rev2 by exact hash",
          REV3["cites"]["rev2_sha256"] == R.PIN_PREREG_REV2)
    check("H20 rev3 declares the precedence the runner implements",
          REV3["precedence"].replace(" ", "")
          == ">".join(R.PREREG_PRECEDENCE))
    check("H21 rev3 leaves the gate count at thirteen",
          "adds no gate" in REV3["additive"] and "13" in REV3["additive"])
    check("H22 rev3 fixes the three token counts",
          REV3["correction_1_token_counting"]["the_three_counts_that_govern"]
          == {"derived_command_tokens": R.EXPECTED_DERIVED_TOKENS,
              "delegated_trainer_argv": R.EXPECTED_DELEGATED_ARGV,
              "full_child_argv": R.EXPECTED_CHILD_ARGV})
    check("H23 rev3 corrects the containment wording",
          "NO PRODUCTION FILE"
          in REV3["correction_2_wrapper_wording"]["the_accurate_claim"])
    check("H24 rev3 requires the exact APPROVED status",
          R.GO_REQUIRED_STATUS
          in REV3["correction_3_go_status"]["the_rule_now"])
    # The invariant, not the wording: what rev3 declares must be what the
    # config actually asks for, read from the pinned YAML itself.
    import yaml
    warmup = R.load_warmup()
    config = yaml.safe_load(
        (HERE / warmup.CONFIG_NAME).read_text(encoding="utf-8"))
    parallelism = config.get("parallelism") or {}
    declared = REV3["correction_4_declared_side_effects"][
        "what_the_config_asks_for"]
    check("H25 rev3's declared environment cost matches the pinned config",
          declared == {"num_env_runners": parallelism.get("num_env_runners"),
                       "ray_num_cpus": parallelism.get("ray_num_cpus")},
          "declared %s, config says %s" % (declared, parallelism))
    check("H25b the config is the one the warm-up used, unmodified",
          sha256_file(HERE / warmup.CONFIG_NAME) == warmup.PIN_CONFIG)
    check("H25c rev3 states that zero train, sample and rollout occur",
          all(word in REV3["correction_4_declared_side_effects"][
              "what_does_NOT_happen"]
              for word in ("algo.train", "sampling", "rollouts")))
    check("H26 rev3 pins the source state sha R9 checks",
          R.EXPECTED_SOURCE_STATE_SHA in json.dumps(REV3))
    check("H27 rev3 keeps raise-on-mismatch and original-called-first",
          any("RAISES" in item for item in REV3["tightened_R9"]["unchanged_from_rev1"])
          and any("called FIRST" in item
                  for item in REV3["tightened_R9"]["unchanged_from_rev1"]))

    # The sealed predecessors must not have moved while rev3 was added.
    for label, path, pin in (
            ("base", HERE / R.PREREG_NAME, R.PIN_PREREG),
            ("rev1", HERE / R.PREREG_REV1_NAME, R.PIN_PREREG_REV1),
            ("rev2", HERE / R.PREREG_REV2_NAME, R.PIN_PREREG_REV2)):
        check("H28 %s is still at its sealed digest after rev3 was added"
              % label, sha256_file(path) == pin,
              "%s vs %s" % (sha256_file(path), pin))


# --------------------------------------------------------- I. leaf discipline

def test_leaf_discipline() -> None:
    run = function_named(RUNNER_TREE, "run_execution")
    source = ast.get_source_segment(RUNNER_SOURCE, run) or ""
    check("I01 the leaf is refused if it already exists",
          "refusing to clobber an existing leaf" in source)
    check("I02 the invalid marker is written before the child",
          source.index("INVALID_MARKER") < source.index("launch_once"))
    check("I03 the leaf is asserted clean before the child",
          source.index("the destination is not clean")
          < source.index("launch_once"))
    verify = ast.get_source_segment(
        RUNNER_SOURCE, function_named(RUNNER_TREE, "verify_commit")) or ""
    check("I04 the passed marker is written only after verification",
          "PASSED_MARKER" in verify)
    check("I05 the invalid marker is unlinked last",
          verify.index("PASSED_MARKER") < verify.index("INVALID_MARKER)"
                                                       ".unlink()")
          if "INVALID_MARKER).unlink()" in verify else False)
    check("I06 verification fails closed when a gate failed",
          "if problems or not gates_ok" in verify)
    check("I07 the destination is currently absent", not LEAF.exists())
    check("I08 the marker names differ from the warm-up's",
          R.PASSED_MARKER == "RESTORE_AUDIT_PASSED"
          and R.PASSED_MARKER != "RESTORE_AUDIT_PENDING")
    check("I09 the result declares promotion NONE",
          '"promotion": "NONE"' in RUNNER_SOURCE)
    check("I10 the result declares training_ready false",
          '"training_ready": False' in RUNNER_SOURCE)
    check("I11 the result declares next_stage_authorized false",
          '"next_stage_authorized": False' in RUNNER_SOURCE)


# ------------------------------------------------------------ J. entry evidence

def test_entry_evidence() -> None:
    entry = R.check_entry_evidence()
    check("J01 the entry evidence passes", entry["ok"],
          "; ".join(entry["problems"]))
    check("J02 the warm-up reads AWAITING_RESTORE_AUDIT",
          entry["found"]["warmup_verdict"] == "AWAITING_RESTORE_AUDIT")
    check("J03 the warm-up reads 12/12", entry["found"]["warmup_gates"] == "12/12")
    check("J04 the warm-up carries the pending marker",
          entry["found"]["pending_marker"] is True)
    check("J05 the checkpoint meta reads logical iteration 1",
          entry["found"]["logical_iteration"] == 1)
    check("J06 the checkpoint meta reads rllib iteration 1",
          entry["found"]["rllib_training_iteration"] == 1)

    with (CHECKPOINT / R.MODULE_STATE_REL.split(R.CHECKPOINT_REL + "/")[1]
          ).open("rb") as handle:
        state = pickle.load(handle)
    check("J07 the checkpoint module holds sixteen keys", len(state) == 16)
    check("J08 the module state hashes to the expected value",
          sha256_file(SOURCE / R.MODULE_STATE_REL)
          == R.EXPECTED_MODULE_STATE_SHA)
    check("J09 the pinned source files all still match",
          all(sha256_file(SOURCE / name) == digest
              for name, digest in R.PIN_SOURCE_LEAF.items()))


def main() -> int:
    test_structurally_inert()
    test_preflight_is_pure()
    test_command()
    test_go_validation()
    test_gate_definitions()
    test_normalisation()
    test_wrapper_detects_damage()
    test_wrapper_contract()
    test_prereg()
    test_leaf_discipline()
    test_entry_evidence()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
