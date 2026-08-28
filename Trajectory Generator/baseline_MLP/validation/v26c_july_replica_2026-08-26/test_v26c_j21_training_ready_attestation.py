"""Hermetic test suite for V26C J21 - the training-ready evidence attestation.

Hermetic in the strict sense: it imports the standard library and the aggregator
under test, and nothing else. No torch, no ray, no numpy, no gymnasium, no
OpenSim, no train_ppo_mlp. It starts no Ray, builds no Algorithm, constructs no
environment, launches no subprocess, samples nothing, rolls out nothing and
trains nothing. It creates no leaf and writes no file anywhere.

It holds the J21 package, as rev1 corrects it, to eleven properties:

  * the aggregator is standard-library-only and cannot construct runtime or
    train - asserted statically over its AST, and again through its own T14;
  * every one of the EIGHTEEN gates is NON-VACUOUS: for each, a single mutation
    of the committed evidence flips it to FAIL;
  * rev1's corrected T15 holds: J19A's actor promotion is recorded as TRUE and
    compatible, no document asserts that no upstream promotion occurred, and
    J21 claims only the first CURRENT V26C checkpoint-level attestation after
    J20 rather than the first ever;
  * the August lineage is MEASURED byte for byte across five links, with a
    mutation per link, and the JULY_FAITHFUL label cannot cause a false
    failure;
  * worker sync is proved from the J20 transplant report - 14 runners, learner
    actor and saved initial actor - with weights_seq_no demoted to supporting
    telemetry that gates nothing;
  * leaf file sets and committed maps are exact, tested on SYNTHETIC data so
    that no test ever creates, deletes or renames a file under a real leaf;
  * a MISSING field is a failure and never a silently-defaulted one, which is
    the property that separates a real aggregation from a hopeful one;
  * the mandatory final semantics hold on PASS and on FAIL alike, and
    deployable is false in both;
  * promotion on PASS is exactly TRAINING_INPUT_ONLY and never anything wider;
  * the readiness scope is explicit that the pilot protocol is NOT sealed, and
    the aggregator invents neither a pilot config nor a launch command;
  * the warm-up leaf and its RESTORE_AUDIT_PENDING marker are byte-immutable,
    and G9's closure is read from the R3 receipt;
  * the DRAFT GO is inert and validate_go refuses it, while an APPROVED GO for
    another stage is refused too.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j21_training_ready_attestation.py
"""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

sys.dont_write_bytecode = True

import v26c_j21_training_ready_attestation as A                  # noqa: E402

RUNNER_PATH = HERE / A.RUNNER_NAME
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
RUNNER_TREE = ast.parse(RUNNER_SOURCE)
PREREG = json.loads((HERE / A.PREREG_NAME).read_text(encoding="utf-8"))
REV1 = json.loads((HERE / A.PREREG_REV1_NAME).read_text(encoding="utf-8"))
# The base and rev1 DRAFTs are HISTORICAL preparation artefacts: never edited,
# never deleted, and superseded. The rev2 DRAFT is the current inert template.
HISTORICAL_DRAFT_GO = HERE / "v26c_j21_training_ready_go_DRAFT.json"
REV1_DRAFT_GO = HERE / "v26c_j21_training_ready_go_DRAFT_rev1.json"
DRAFT_GO = HERE / "v26c_j21_training_ready_go_DRAFT_rev2.json"
LEAF = HERE / A.LEAF_ROOT / A.LEAF_NAME
WARMUP_LEAF = HERE / A.WARMUP_LEAF_REL

CHECKS: list = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(condition), detail))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# The committed evidence, read once and deep-copied per mutation so that no
# test can perturb another and nothing on disk is ever touched.
EVIDENCE = A.evidence()
PINS = A.all_pins()


def fresh():
    return copy.deepcopy(EVIDENCE)


def problems_of(function, *args) -> list:
    """Run a gate body against a mutated evidence set and collect its problems."""
    found: list = []
    function(*(args + (found,)))
    return found


# --------------------------------------------- A. the aggregator is read-only

def test_aggregator_is_read_only() -> None:
    imported = set()
    for node in ast.walk(RUNNER_TREE):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    check("A01 the aggregator imports only the standard library",
          not (imported & set(A.FORBIDDEN_IMPORTS)),
          str(sorted(imported & set(A.FORBIDDEN_IMPORTS))))
    check("A02 the imports are exactly the declared standard-library set",
          imported <= {"argparse", "ast", "hashlib", "json", "pathlib", "sys",
                       "datetime", "__future__"}, str(sorted(imported)))
    called = set()
    for node in ast.walk(RUNNER_TREE):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute):
                called.add(func.attr)
            elif isinstance(func, ast.Name):
                called.add(func.id)
    check("A03 the aggregator makes no forbidden call",
          not (called & set(A.FORBIDDEN_CALL_NAMES)),
          str(sorted(called & set(A.FORBIDDEN_CALL_NAMES))))
    for banned in ("train_ppo_mlp", "ray.init", "build_algo",
                   "restore_from_path", "subprocess", "Popen", "os.system"):
        check("A04 the aggregator source never mentions %r in code" % banned,
              banned not in _code_only(RUNNER_SOURCE))

    audit = A.gate_t14_self_audit()
    check("A05 its own T14 self-audit passes", audit["problems"] == [],
          str(audit["problems"]))
    for flag in ("constructs_runtime", "starts_ray", "constructs_environment",
                 "launches_subprocess", "trains"):
        check("A06 T14 records %s false" % flag, audit[flag] is False)
    check("A07 T14 records standard_library_only true",
          audit["standard_library_only"] is True)
    check("A08 T14 is not vacuous: it would catch a forbidden import",
          "ray" in A.FORBIDDEN_IMPORTS and "torch" in A.FORBIDDEN_IMPORTS
          and "train_ppo_mlp" in A.FORBIDDEN_IMPORTS
          and "subprocess" in A.FORBIDDEN_IMPORTS)


def _code_only(source: str) -> str:
    """The source with every docstring and string literal emptied."""

    class Blank(ast.NodeTransformer):
        def visit_Constant(self, node):                      # noqa: N802
            if isinstance(node.value, str):
                return ast.copy_location(ast.Constant(value=""), node)
            return node

    tree = Blank().visit(ast.parse(source))
    ast.fix_missing_locations(tree)
    return ast.unparse(tree)


# ------------------------------------------------ B. the gates are non-vacuous

def test_gates_pass_on_the_real_evidence() -> None:
    graded = A.evaluate()
    check("B01 all EIGHTEEN gates pass on the committed evidence",
          graded["passed"] == 18 and graded["total"] == 18
          and graded["failed"] == [],
          str(graded["failed"]))
    check("B02 every pinned artefact matches",
          graded["pin_report"]["matching"] == graded["pin_report"]["checked"],
          "%d/%d" % (graded["pin_report"]["matching"],
                     graded["pin_report"]["checked"]))
    check("B03 the pin set is the R3 GO's 92 plus the additional plus both "
          "preregistrations",
          graded["pin_report"]["checked"]
          == A.R3_GO_PIN_COUNT + len(A.PIN_ADDITIONAL) + 2,
          str(graded["pin_report"]["checked"]))
    check("B03a the additional set grew by the nine lineage artefacts",
          len(A.PIN_ADDITIONAL) == 45, str(len(A.PIN_ADDITIONAL)))


def test_each_gate_flips() -> None:
    # T1 lineage
    check("B04 T1 no longer scans labels, so an extra pin label containing "
          "the word July does NOT fail it - only measured bytes decide",
          not problems_of(A.gate_t1_lineage, fresh(),
                          dict(PINS, **{"j0_july_leaf/x.json": "0" * 64})))
    check("B05 T1 flips when the J19A parent is not the J18 leaf",
          problems_of(A.gate_t1_lineage, _mutate(
              lambda e: e["j19a_receipt"]["inputs"].__setitem__(
                  "j18_leaf", "july_runs/whatever")), PINS))
    check("B06 T1 flips when the 35D actor manifest is not pinned",
          problems_of(A.gate_t1_lineage, fresh(),
                      {k: v for k, v in PINS.items()
                       if "actor_feature_manifest" not in k}))

    # T2 J19A
    for label, mutate in (
        ("a non-PASS verdict",
         lambda e: e["j19a_receipt"].__setitem__("verdict", "FAIL")),
        ("eligibility not ok",
         lambda e: e["j19a_result"]["eligibility"].__setitem__("ok", False)),
        ("a binding criterion that did not pass",
         lambda e: e["j19a_result"]["eligibility"]["binding"][0].__setitem__(
             "passed", False)),
        ("a missing binding criterion",
         lambda e: e["j19a_result"]["eligibility"]["binding"].pop()),
        ("35 exact fields instead of 36",
         lambda e: e["j19a_receipt"].__setitem__("exact_fields_compared", 35)),
        ("a non-finite count",
         lambda e: e["j19a_receipt"].__setitem__("non_finite_count", 1)),
        ("a rollout having been performed",
         lambda e: e["j19a_receipt"].__setitem__("rollout_performed", True)),
        ("a diagnostic criterion promoted to binding",
         lambda e: e["j19a_result"]["eligibility"][
             "diagnostic_only"][0].__setitem__("binding", True)),
    ):
        check("B07 T2 flips on %s" % label,
              problems_of(A.gate_t2_j19a, _mutate(mutate)))

    # T3 J19B and T4 J19C
    for gate, receipt_key, ids, total, tag in (
            ("T3", "j19b", A.J19B_CELLS, 6, "B08"),
            ("T4", "j19c", A.J19C_CELLS, 3, "B09")):
        for label, mutate in (
            ("a non-PASS verdict",
             lambda e, k=receipt_key: e[k].__setitem__("verdict", "FAIL")),
            ("aggregate_pass false",
             lambda e, k=receipt_key: e[k].__setitem__("aggregate_pass", False)),
            ("the actor having changed",
             lambda e, k=receipt_key: e[k].__setitem__("actor_unchanged",
                                                       False)),
            ("one cell short",
             lambda e, k=receipt_key: e[k]["cells"].pop()),
            ("a cell that did not behaviourally pass",
             lambda e, k=receipt_key: e[k]["cells"][0].__setitem__(
                 "behavioural_pass", False)),
            ("a cell whose telemetry is invalid",
             lambda e, k=receipt_key: e[k]["cells"][0].__setitem__(
                 "telemetry_valid", False)),
            ("a wrong behavioural count",
             lambda e, k=receipt_key: e[k].__setitem__(
                 "cells_behavioural_pass", 1)),
        ):
            mutated = _mutate(mutate)
            check("%s %s flips on %s" % (tag, gate, label),
                  problems_of(A.gate_j19_cells, mutated[receipt_key], ids,
                              total, gate))

    # T5 penetration
    for label, mutate in (
        ("a sample above the 28 mm hard band",
         lambda e: e["j19b"]["cells"][0]["penetration"]["counts"].__setitem__(
             "above_hard_binding", 1)),
        ("the hard-binding flag raised",
         lambda e: e["j19b"]["cells"][2]["penetration"]["flags"].__setitem__(
             "above_hard_binding", True)),
        ("binding_pass false",
         lambda e: e["j19c"]["cells"][1]["penetration"].__setitem__(
             "binding_pass", False)),
        ("a moved hard band",
         lambda e: e["contract"]["bands"]["hard_binding"].__setitem__(
             "value_m", 0.05)),
        ("the hard band declared non-binding",
         lambda e: e["contract"]["bands"]["hard_binding"].__setitem__(
             "binding", False)),
        ("the soft band declared binding",
         lambda e: e["contract"]["bands"]["soft_diagnostic"].__setitem__(
             "binding", True)),
        ("a different penetration authority",
         lambda e: e["j19b"]["penetration_authority"].__setitem__(
             "contract_sha256", "0" * 64)),
        ("a third cell entering the July band",
         lambda e: e["j19b"]["cells"][0]["penetration"]["counts"].__setitem__(
             "at_or_above_july_legacy", 7)),
    ):
        check("B10 T5 flips on %s" % label,
              problems_of(A.gate_t5_penetration, _mutate(mutate)))

    # T6 counters
    for counter in A.ZERO_CELL_COUNTERS:
        check("B11 T6 flips when cell counter %s is non-zero" % counter,
              problems_of(A.gate_t6_counters, _mutate(
                  lambda e, c=counter: e["j19b"]["cells"][0][
                      "telemetry"].__setitem__(c, 1))))
        check("B12 T6 flips when cell counter %s is absent" % counter,
              problems_of(A.gate_t6_counters, _mutate(
                  lambda e, c=counter: e["j19c"]["cells"][0][
                      "telemetry"].pop(c))))
    for field in A.ZERO_HEALTH_FIELDS:
        check("B13 T6 flips when warm-up health %s is non-zero" % field,
              problems_of(A.gate_t6_counters, _mutate(
                  lambda e, f=field: e["warmup_result"]["measurements"][
                      "training_health"].__setitem__(f, 1.0))))

    # T7 K1R1
    for label, mutate in (
        ("a non-PASS verdict",
         lambda e: e["k1r1"].__setitem__("verdict", "FAIL")),
        ("11 of 12 checks",
         lambda e: e["k1r1"].__setitem__("amended_checks_passed", 11)),
        ("a promotion claim",
         lambda e: e["k1r1"].__setitem__("promotion", "SOMETHING")),
    ):
        check("B14 T7 flips on %s" % label,
              problems_of(A.gate_t7_k1r1, _mutate(mutate)))

    # T8 warm-up
    for label, mutate in (
        ("a wrong verdict",
         lambda e: e["warmup_result"].__setitem__("verdict", "PASS")),
        ("11 of 12 gates",
         lambda e: e["warmup_result"].__setitem__("gates_passed", 11)),
        ("a failed named gate",
         lambda e: e["warmup_result"]["gates"].__setitem__(
             "G5_actor_bit_exact_at_every_audit", False)),
        ("a missing named gate",
         lambda e: e["warmup_result"]["gates"].pop("G7_critic_digest_changed")),
        ("an actor that is not byte-identical",
         lambda e: e["warmup_result"]["actor"].__setitem__(
             "actor_all_byte_identical", False)),
        ("a wrong actor digest",
         lambda e: e["warmup_result"]["actor"].__setitem__("actor_digest",
                                                           "0" * 64)),
        ("one actor key not byte-identical",
         lambda e: e["warmup_result"]["actor"][
             "actor_per_key_byte_identical"].__setitem__("pi.1.weight", False)),
        ("a log-std that moved",
         lambda e: e["warmup_result"]["actor"].__setitem__(
             "logstd_rows_byte_identical", False)),
        ("a changed sigma",
         lambda e: e["warmup_result"]["actor"].__setitem__("sigma",
                                                           [0.01, 0.01])),
        ("a critic that did NOT change",
         lambda e: e["warmup_result"]["measurements"].__setitem__(
             "critic_digest_before",
             e["warmup_result"]["measurements"]["critic_digest_after"])),
        ("2048 sampled steps instead of 4096",
         lambda e: e["warmup_result"]["measurements"].__setitem__(
             "num_env_steps_sampled_lifetime", 2048.0)),
        ("a non-zero mean KL",
         lambda e: e["warmup_result"]["measurements"].__setitem__(
             "mean_kl_loss", 0.1)),
        ("two committed iteration rows",
         lambda e: e["warmup_rows"].append(e["warmup_rows"][0])),
        ("a checkpoint that is not ok",
         lambda e: e["warmup_result"]["checkpoint"].__setitem__("ok", False)),
        ("15 module keys",
         lambda e: e["warmup_result"]["checkpoint"].__setitem__(
             "learner_module_key_count", 15)),
        ("shifted Adam indices",
         lambda e: e["warmup_result"]["checkpoint"].__setitem__(
             "optimizer_adam_state_indices", [0, 1, 2, 3, 4, 5])),
        ("moments outside the critic",
         lambda e: e["warmup_result"]["checkpoint"].__setitem__(
             "only_the_critic_has_adam_moments", False)),
    ):
        check("B15 T8 flips on %s" % label,
              problems_of(A.gate_t8_warmup, _mutate(mutate)))

    # T9 R3
    for label, mutate in (
        ("a non-PASS verdict",
         lambda e: e["r3_result"].__setitem__("verdict", "FAIL_CLOSED")),
        ("12 of 13 gates",
         lambda e: e["r3_result"].__setitem__("gates_passed", 12)),
        ("a failed gate listed",
         lambda e: e["r3_result"].__setitem__("gates_failed", ["R9"])),
        ("a gate that was not evaluated",
         lambda e: e["r3_result"].__setitem__("gates_not_evaluated", ["R12"])),
        ("an incomplete live restore",
         lambda e: e["r3_result"]["live_restore"].__setitem__("completed",
                                                              False)),
        ("g9 not closed",
         lambda e: e["r3_receipt"].__setitem__("g9_closed", False)),
        ("a non-zero child returncode",
         lambda e: e["r3_receipt"].__setitem__("child_returncode", 1)),
        ("sources having changed during R3",
         lambda e: e["r3_receipt"].__setitem__("source_tree_unchanged", False)),
        ("a failed commit verification",
         lambda e: e["r3_commit"].__setitem__("ok", False)),
        ("a commit verification problem",
         lambda e: e["r3_commit"].__setitem__("problems", ["x"])),
    ):
        check("B16 T9 flips on %s" % label,
              problems_of(A.gate_t9_r3, _mutate(mutate)))

    # T10 live optimizer, including rev7's learning-rate gate
    for label, mutate in (
        ("a non-exact verdict",
         lambda e: e["r3_evidence"].__setitem__("exact", False)),
        ("a canonical difference",
         lambda e: e["r3_evidence"]["canonical"].__setitem__(
             "difference_count", 1)),
        ("canonical digests that do not match",
         lambda e: e["r3_evidence"]["canonical"].__setitem__("digests_match",
                                                             False)),
        ("a ninth accepted equivalence",
         lambda e: e["r3_evidence"].__setitem__(
             "accepted_equivalences_are_exactly_the_expected_eight", False)),
        ("an unexplained canonical path",
         lambda e: e["r3_evidence"].__setitem__("unexplained_canonical_paths",
                                                ["x"])),
        ("moments that do not match",
         lambda e: e["r3_evidence"].__setitem__("moment_digests_match", False)),
        ("a moment moved by canonicalisation",
         lambda e: e["r3_evidence"].__setitem__(
             "moments_unchanged_by_canonicalisation_live", False)),
        ("an unverified conversion call site",
         lambda e: e["r3_evidence"]["conversion"].__setitem__(
             "call_site_verified", False)),
        ("a broken fixed point",
         lambda e: e["r3_evidence"].__setitem__("conversion_fixed_point_source",
                                                False)),
        ("a learning rate that did not match",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "matches", False)),
        ("an ungated learning-rate observation",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "gated", False)),
        ("a missing learning-rate observation",
         lambda e: e["r3_evidence"].pop("learning_rate_observation")),
        ("a null canonical source lr",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "canonical_source_lr", None)),
        ("disagreeing canonical lr fields",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "canonical_live_lr_before_reapply", "('t', 'float32', (), 'zz')")),
        ("a missing pre-reapply lr",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "live_lr_before_reapply", None)),
        ("a wrong canonical lr dtype",
         lambda e: e["r3_evidence"]["learning_rate_observation"].__setitem__(
             "canonical_dtype", "float64")),
        ("a wrong source state sha",
         lambda e: e["r3_evidence"].__setitem__("source_state_sha256",
                                                "0" * 64)),
    ):
        check("B17 T10 flips on %s" % label,
              problems_of(A.gate_t10_live_optimizer, _mutate(mutate)))

    # T11 warm-up leaf immutability
    check("B18 T11 flips when the pending marker is not pinned",
          problems_of(A.gate_t11_warmup_leaf_immutable,
                      {k: v for k, v in PINS.items()
                       if not k.endswith("RESTORE_AUDIT_PENDING")}))
    check("B19 T11 flips when the pending marker's pin is wrong",
          problems_of(A.gate_t11_warmup_leaf_immutable,
                      dict(PINS, **{A.WARMUP_LEAF_REL
                                    + "/RESTORE_AUDIT_PENDING": "0" * 64})))

    # T12 checkpoint
    check("B20 T12 flips when a checkpoint file is unpinned",
          problems_of(A.gate_t12_checkpoint, fresh(),
                      {k: v for k, v in PINS.items()
                       if "checkpoint_last/rllib_checkpoint.json" not in k}))
    check("B21 T12 flips when the meta does not read iteration 1",
          problems_of(A.gate_t12_checkpoint, _mutate(
              lambda e: e["warmup_meta"].__setitem__("logical_iteration", 2)),
              PINS))

    # T15 upstream never promoted THIS checkpoint or authorised a launch
    for label, mutate in (
        ("an upstream promotion claim",
         lambda e: e["r3_result"].__setitem__("promotion", "SOMETHING")),
        ("an upstream training_ready claim",
         lambda e: e["warmup_result"].__setitem__("training_ready", True)),
        ("an upstream next-stage authorisation",
         lambda e: e["r3_receipt"].__setitem__("next_stage_authorized", True)),
        ("an upstream deployable claim",
         lambda e: e["j19b"]["outcome"].__setitem__("deployable", True)),
        ("an upstream stage already promoting to TRAINING_INPUT_ONLY",
         lambda e: e["r3_result"].__setitem__("promotion",
                                              A.PROMOTION_PASS)),
        ("a J19 outcome already promoting to TRAINING_INPUT_ONLY",
         lambda e: e["j19c"]["outcome"].__setitem__("promotion",
                                                    A.PROMOTION_PASS)),
        ("J19A NOT having promoted its actor",
         lambda e: e["j19a_result"].__setitem__("actor_promoted", False)),
        ("a missing J19A actor promotion field",
         lambda e: e["j19a_result"].pop("actor_promoted")),
        ("a J19B PPO update",
         lambda e: e["j19b"]["inert"].__setitem__("ppo_updates", 1)),
        ("a J19C continuation note",
         lambda e: e["j19c"]["outcome"].__setitem__(
             "note", "continue downstream")),
        ("a K1R1 PPO update",
         lambda e: e["k1r1"]["inert"].__setitem__("ppo_updates", 1)),
        ("a K1R1 algo train call",
         lambda e: e["k1r1"]["inert"].__setitem__(
             "algo_train_called", True)),
        ("a K1R1 warmup execution",
         lambda e: e["k1r1"]["inert"].__setitem__(
             "warmup_executed", True)),
        ("a warm-up result authorising PPO/ex-novo",
         lambda e: e["warmup_result"]["inert"].__setitem__(
             "ppo_ex_novo", True)),
        ("a warm-up receipt authorising PPO/ex-novo",
         lambda e: e["warmup_receipt"]["inert"].__setitem__(
             "ppo_ex_novo", True)),
        ("an R3 result authorising PPO/ex-novo",
         lambda e: e["r3_result"]["inert"].__setitem__(
             "ppo_ex_novo", True)),
        ("an R3 receipt authorising PPO/ex-novo",
         lambda e: e["r3_receipt"]["inert"].__setitem__(
             "ppo_ex_novo", True)),
        ("R3 no longer denying PPO",
         lambda e: e["r3_result"]["what_this_still_does_not_authorise"].remove(
             "PPO")),
        ("R3 no longer denying an ex-novo run",
         lambda e: e["r3_result"]["what_this_still_does_not_authorise"].remove(
             "an ex-novo run")),
        ("R3 no longer denying promotion",
         lambda e: e["r3_result"]["what_this_still_does_not_authorise"].remove(
             "any promotion")),
        ("R3 no longer denying training readiness",
         lambda e: e["r3_result"]["what_this_still_does_not_authorise"].remove(
             "training readiness")),
        ("R3 no longer denying a subsequent stage",
         lambda e: e["r3_result"]["what_this_still_does_not_authorise"].remove(
             "any subsequent stage")),
    ):
        check("B22 T15 flips on %s" % label,
              problems_of(A.gate_t15_no_upstream_checkpoint_readiness,
                          _mutate(mutate)))

    # T16 worker sync, every field the architect named
    for label, mutate in (
        ("weights not synced before the first sample",
         lambda e: e["transplant_report"]["integration_validation"].__setitem__(
             "weights_synced_before_first_sample", False)),
        ("13 env runners checked instead of 14",
         lambda e: e["transplant_report"]["integration_validation"].__setitem__(
             "env_runner_count_checked", 13)),
        ("env runner actors not exact",
         lambda e: e["transplant_report"]["integration_validation"].__setitem__(
             "env_runner_actors_exact", False)),
        ("one env runner holding a different actor",
         lambda e: e["transplant_report"]["integration_validation"][
             "env_runner_actor_digests"].__setitem__(3, "0" * 64)),
        ("only 13 env runner digests",
         lambda e: e["transplant_report"]["integration_validation"][
             "env_runner_actor_digests"].pop()),
        ("a 15th env runner digest",
         lambda e: e["transplant_report"]["integration_validation"][
             "env_runner_actor_digests"].append(A.EXPECTED_ACTOR_DIGEST)),
        ("the digest list replaced by a scalar",
         lambda e: e["transplant_report"]["integration_validation"].__setitem__(
             "env_runner_actor_digests", A.EXPECTED_ACTOR_DIGEST)),
        ("a missing integration_validation block",
         lambda e: e["transplant_report"].pop("integration_validation")),
    ):
        check("B23 T16 flips on %s" % label,
              problems_of(A.gate_t16_worker_sync, _mutate(mutate)))
    for block in ("learner_actor", "saved_initial_actor"):
        for field, bad in (("exact", False), ("max_abs_diff", 1e-9),
                           ("expected_digest", "0" * 64),
                           ("actual_digest", "0" * 64),
                           ("missing_keys", ["pi.1.weight"])):
            check("B24 T16 flips when %s.%s is wrong" % (block, field),
                  problems_of(A.gate_t16_worker_sync, _mutate(
                      lambda e, b=block, f=field, v=bad:
                      e["transplant_report"]["integration_validation"][
                          b].__setitem__(f, v))))
        check("B25 T16 flips when the %s block is missing" % block,
              problems_of(A.gate_t16_worker_sync, _mutate(
                  lambda e, b=block: e["transplant_report"][
                      "integration_validation"].pop(b))))
    check("B26 weights_seq_no is NO LONGER binding in T8",
          not problems_of(A.gate_t8_warmup, _mutate(
              lambda e: e["warmup_rows"][0]["learner_metrics"].__setitem__(
                  "learners/default_policy/weights_seq_no", 7.0))),
          "T8 must not fail on a changed weights_seq_no under rev1")
    check("B27 ... and it is still RECORDED as supporting telemetry",
          A.gate_t8_warmup(fresh(), [])[
              "worker_weights_seq_no_supporting_telemetry_only"] == 1.0)

    # T17 measured lineage, one mutation per link
    for label, mutate in (
        ("a different August source module",
         lambda e: e["j2_receipt"]["derivation"]["parent"].__setitem__(
             "module_state_sha256", "0" * 64)),
        ("a different August source manifest",
         lambda e: e["j2_receipt"]["derivation"]["parent"].__setitem__(
             "manifest39_sha256", "0" * 64)),
        ("a wrong August source width",
         lambda e: e["j2_receipt"]["derivation"]["parent"].__setitem__(
             "width", 35)),
        ("a wrong August source path",
         lambda e: e["j2_receipt"]["derivation"]["parent"].__setitem__(
             "path", "Trajectory Generator/runs/training/somewhere_else")),
        ("a different J2 output module",
         lambda e: e["j2_receipt"]["output_files_sha256"].__setitem__(
             "module_state.pkl", "0" * 64)),
        ("a different J2 35D manifest",
         lambda e: e["j2_receipt"]["output_files_sha256"].__setitem__(
             "actor_feature_manifest.json", "0" * 64)),
        ("a J8 parent that is not J2",
         lambda e: e["j8_receipt"]["inputs"]["parent"].__setitem__(
             "module_state_sha256", "0" * 64)),
        ("a different J8 output module",
         lambda e: e["j8_receipt"]["output"]["files"].__setitem__(
             "rl_module/module_state.pkl", "0" * 64)),
        ("a J18 parent that is not J8",
         lambda e: e["j18_receipt"]["inputs"]["parent"].__setitem__(
             "actual_byte_sha256", "0" * 64)),
        ("a wrong J8 actor digest at J18",
         lambda e: e["j18_receipt"]["inputs"]["parent"].__setitem__(
             "actor_digest", "0" * 64)),
        ("a J19A parent that is not J8",
         lambda e: e["j19a_receipt"]["inputs"].__setitem__("parent_j8_sha256",
                                                           "0" * 64)),
        ("a J19A grandparent that is not J2",
         lambda e: e["j19a_receipt"]["inputs"].__setitem__(
             "parent_of_j8_sha256", "0" * 64)),
        ("a J19A manifest whose parent is not J8",
         lambda e: e["j19a_manifest"]["derived_from"].__setitem__("parent",
                                                                  "July")),
        ("a J19A manifest source actor that is not the J8 actor",
         lambda e: e["j19a_manifest"].__setitem__("source_actor_digest",
                                                  "0" * 64)),
        ("a J19A output actor that is not d4a13ff7",
         lambda e: e["j19a_manifest"].__setitem__("actor_digest", "0" * 64)),
        ("a J19A output module that is not 8153dc97",
         lambda e: e["j19a_manifest"].__setitem__("module_state_sha256",
                                                  "0" * 64)),
        ("a warm-up source actor that is not J19A's",
         lambda e: e["transplant_report"].__setitem__("source_actor_digest",
                                                      "0" * 64)),
        ("a warm-up target actor that changed",
         lambda e: e["transplant_report"].__setitem__(
             "target_actor_digest_after", "0" * 64)),
        ("a warm-up manifest that validated a different actor",
         lambda e: e["transplant_report"]["source_actor_feature_manifest"
                                          ].__setitem__(
             "validated_actor_digest", "0" * 64)),
    ):
        check("B28 T17 flips on %s" % label,
              problems_of(A.gate_t17_measured_lineage, _mutate(mutate)))

    # T1 July semantics, refined
    for label, mutate in (
        ("J8 no longer declaring July's parent unreplicated",
         lambda e: e["j8_receipt"]["july_faithfulness"]["does_not_replicate"
                                                        ].remove("July's parent")),
        ("J8 no longer declaring July's actor unreplicated",
         lambda e: e["j8_receipt"]["july_faithfulness"]["does_not_replicate"
                                                        ].remove("July's actor")),
        ("J8 no longer declaring July's dataset unreplicated",
         lambda e: e["j8_receipt"]["july_faithfulness"][
             "does_not_replicate"].remove("July's dataset")),
        ("a missing does_not_replicate list",
         lambda e: e["j8_receipt"]["july_faithfulness"].pop(
             "does_not_replicate")),
        ("a J8 operational lineage that does not name the August V26 J2 parent",
         lambda e: e["j8_receipt"]["july_faithfulness"].__setitem__(
             "operational_lineage", "July's parent and July's dataset")),
        ("a student that is not 35D",
         lambda e: e["j19a_manifest"].__setitem__("observation_width", 39)),
    ):
        check("B29 T1 flips on %s" % label,
              problems_of(A.gate_t1_lineage, _mutate(mutate), PINS))
    check("B30 T1 does NOT fail merely because a label contains the word July",
          not problems_of(A.gate_t1_lineage, fresh(), PINS),
          "the containing directory is v26c_july_replica_2026-08-26 and J8 is "
          "labelled July-faithful; neither may cause a failure")


def _mutate(mutate):
    evidence = fresh()
    mutate(evidence)
    return evidence


# ------------------------- B2. T18 leaf integrity, tested HERMETICALLY
#
# check_leaf_set and check_committed_map take their inputs as ARGUMENTS, so
# every case below is exercised on synthetic data. No test creates, deletes,
# renames or touches a single file under any real leaf.

def test_leaf_integrity_hermetically() -> None:
    expected = ["a.json", "b/c.npz", "receipt.json"]

    found: list = []
    result = A.check_leaf_set("synthetic", list(expected), expected, found)
    check("B31 an exact file set passes", found == [] and result["exact"] is True)

    found = []
    A.check_leaf_set("synthetic", expected + ["stowaway.txt"], expected, found)
    check("B32 an UNEXPECTED file fails, and is named",
          any("unexpected files" in p and "stowaway.txt" in p for p in found),
          str(found))

    found = []
    A.check_leaf_set("synthetic", expected[:-1], expected, found)
    check("B33 a MISSING file fails, and is named",
          any("missing" in p and "receipt.json" in p for p in found), str(found))

    found = []
    A.check_leaf_set("synthetic", [], expected, found)
    check("B34 an empty leaf fails", len(found) >= 2)

    # committed maps
    found = []
    A.check_committed_map("synthetic", {"committed_files_sha256":
                                        {"x": "0" * 64}}, 1, found)
    check("B35 a correct committed map passes", found == [])

    found = []
    A.check_committed_map("synthetic", {}, 1, found)
    check("B36 a MISSING committed map fails",
          any("no committed_files_sha256 map" in p for p in found), str(found))

    found = []
    A.check_committed_map("synthetic", {"committed_files_sha256": {}}, 1, found)
    check("B37 an EMPTY committed map fails, rather than verifying vacuously",
          any("EMPTY" in p for p in found), str(found))

    found = []
    A.check_committed_map("synthetic", {"committed_files_sha256": []}, 1, found)
    check("B38 a committed map that is not a dict fails",
          any("not a map" in p for p in found), str(found))

    found = []
    A.check_committed_map("synthetic",
                          {"committed_files_sha256": {"x": "0" * 64}}, 132,
                          found)
    check("B39 a committed map of the WRONG SIZE fails",
          any("expected 132" in p for p in found), str(found))

    found = []
    A.check_committed_map("synthetic", "not a receipt", 1, found)
    check("B40 a receipt that is not an object fails", found != [])

    # and the real leaves, at their exact expected sizes
    graded = A.evaluate()
    integrity = graded["findings"][
        "T18_leaf_file_sets_and_committed_maps_are_exact"]
    for label, count in A.EXPECTED_LEAF_FILE_COUNTS.items():
        check("B41 the real %s leaf holds exactly %d files" % (label, count),
              integrity[label]["files"] == count
              and integrity[label]["exact"] is True,
              str(integrity[label]))
    for label, size in A.EXPECTED_COMMITTED_MAP_SIZES.items():
        check("B42 the real %s committed map holds exactly %d entries"
              % (label, size),
              integrity[label]["committed_map_entries"] == size)
    check("B43 the checkpoint gate still requires exactly 24 files",
          A.EXPECTED_CHECKPOINT_FILES == 24
          and graded["findings"][
              "T12_checkpoint_files_are_pinned_and_present"]["files"] == 24)


# ----------------------------------- B3. rev1's corrected claims and semantics

def test_rev1_semantics() -> None:
    graded = A.evaluate()
    t15 = graded["findings"]["T15_no_upstream_checkpoint_readiness_or_launch"]

    check("B44 J19A's actor promotion is recorded as TRUE, not denied",
          t15["compatible_and_distinct"]["j19a_actor_promoted"] is True)
    check("B45 it is labelled STAGE-LOCAL ACTOR PROMOTION",
          t15["compatible_and_distinct"]["scope"]
          == "STAGE-LOCAL ACTOR PROMOTION")
    check("B46 it is explained as compatible and distinct, not an exception",
          "Different object, different claim."
          in t15["compatible_and_distinct"]["why_it_does_not_conflict"])
    check("B47 it is REQUIRED, not merely tolerated",
          "would fail this gate"
          in t15["compatible_and_distinct"]["it_is_required_not_merely_tolerated"])
    check("B48 the gate description is about THIS checkpoint, not all promotions",
          any("THIS J20 checkpoint" in clause for clause in t15["gated"]))
    check("B49 J21 claims only the first CURRENT V26C checkpoint-level "
          "attestation after J20",
          "first CURRENT V26C checkpoint-level" in t15["what_j21_may_claim"]
          and "after J20" in t15["what_j21_may_claim"])
    check("B50 J21 explicitly refuses the 'first ever' claim",
          "first training_ready in the project's history"
          in t15["what_j21_may_NOT_claim"])

    # No document in the package may ASSERT the global claim. Quoting the
    # base's wrong wording in order to correct it is the opposite of asserting
    # it, so the two fields whose whole purpose is to quote it are excluded by
    # name - and their presence is separately REQUIRED by B53.
    quoted_for_correction = {
        REV1["correction_1_T15_was_factually_wrong"]["what_the_base_said"],
        REV1["correction_1_T15_was_factually_wrong"]["the_second_error"],
    }

    def asserted_text(text: str) -> str:
        for quotation in quoted_for_correction:
            text = text.replace(quotation, "")
            text = text.replace(json.dumps(quotation)[1:-1], "")
        return text.lower()

    for name, text in (
        ("the aggregator", _code_only(RUNNER_SOURCE)),
        ("rev1", asserted_text(json.dumps(REV1))),
        ("the current rev2 DRAFT", DRAFT_GO.read_text(encoding="utf-8")),
        ("the base preregistration's successor semantics",
         json.dumps(A.evaluate()["findings"])),
    ):
        lowered = text.lower()
        check("B51 %s never ASSERTS being the FIRST AND ONLY place readiness "
              "may be true" % name,
              "first and only place" not in lowered)
        check("B52 %s never ASSERTS that no upstream promotion occurred" % name,
              "no upstream stage ever claimed" not in lowered
              and "ever claimed a promotion" not in lowered)
    check("B53 rev1 states plainly that the base T15 was factually wrong",
          "factually_wrong" in json.dumps(REV1)
          and "J19A legitimately records actor_promoted true"
          in REV1["correction_1_T15_was_factually_wrong"]["why_that_is_wrong"])

    # rev1's exact J19C values, not rounded approximations.
    exact = REV1["correction_6_exact_J19C_values"][
        "the_exact_values_from_the_receipt"]
    cells = graded["findings"][
        "T5_penetration_contract_and_no_hard_breach"]["cells"]
    for cid in ("G", "H", "I"):
        check("B54 rev1's J19C %s value is the receipt's exact value" % cid,
              exact[cid] == cells[cid]["max_penetration_m"],
              "%r vs %r" % (exact[cid], cells[cid]["max_penetration_m"]))
    check("B55 H is the full-precision value, not 0.025684",
          exact["H"] == 0.025684338426320914 and exact["H"] != 0.025684)
    check("B56 I is the full-precision value, not 0.024763",
          exact["I"] == 0.024762677044730928 and exact["I"] != 0.024763)
    for cid, value in REV1["correction_6_exact_J19C_values"][
            "the_J19B_values_were_already_exact"].items():
        check("B57 rev1's J19B %s value matches the receipt" % cid,
              value == cells[cid]["max_penetration_m"])

    # rev1's measured lineage, reproduced in the findings.
    links = graded["findings"]["T17_august_lineage_measured_byte_for_byte"][
        "links"]
    check("B58 the August source is the measured parent of J2",
          links["august_to_j2"]["parent"] == A.SHA_AUGUST_MODULE
          and links["august_to_j2"]["child"] == A.SHA_J2_MODULE)
    check("B59 the width transition 39 -> 35 is recorded",
          links["august_to_j2"]["width"] == "39 -> 35")
    check("B60 J2 is the measured parent of J8",
          links["j2_to_j8"]["child"] == A.SHA_J8_MODULE)
    check("B61 J8 is the measured parent of J18, with its actor digest",
          links["j8_to_j18"]["parent_actor_digest"] == A.DIGEST_J8_ACTOR)
    check("B62 J18 records zero persisted actors, and that is stated",
          "persisted zero actors" in links["j8_to_j18"]["note"])
    check("B63 J19A's output actor is the one inside the checkpoint",
          links["j18_j8_to_j19a"]["child_actor_digest"]
          == A.EXPECTED_ACTOR_DIGEST)
    check("B64 the warm-up transplanted exactly that actor, unchanged",
          links["j19a_to_warmup"]["source_actor_digest"]
          == links["j19a_to_warmup"]["target_actor_digest"]
          == A.EXPECTED_ACTOR_DIGEST)
    check("B65 the lineage is declared MEASURED, not asserted",
          graded["findings"]["T17_august_lineage_measured_byte_for_byte"][
              "measured_not_declared"] is True)

    # rev1's refined July semantics.
    t1 = graded["findings"][
        "T1_lineage_is_august_operational_july_informational"]
    check("B66 July is forbidden as parent or training input, itemised",
          sorted(t1["july_forbidden_as_operational_parent_or_training_input"])
          == sorted(A.JULY_FORBIDDEN_AS_INPUT))
    check("B67 July-derived constants, scaling code and the 25 mm band are "
          "allowed as disclosed informational sources",
          any("methodological constants" in s
              for s in t1["july_allowed_as_disclosed_informational_sources"])
          and any("25 mm" in s
                  for s in t1["july_allowed_as_disclosed_informational_sources"])
          and any("scaling code" in s
                  for s in t1["july_allowed_as_disclosed_informational_sources"]))
    check("B68 the JULY_FAITHFUL label is explicitly not a lineage",
          "label is not a lineage" in t1["label_is_not_lineage"].lower()
          or "What decides is the parent bytes" in t1["label_is_not_lineage"])
    check("B69 J8's own receipt refutes July as parent, actor, dataset, output",
          all(item in t1["j8_does_not_replicate_from_july"]
              for item in A.JULY_MUST_NOT_REPLICATE))
    check("B70 and names the August V26 J2 parent as its operational lineage",
          "August V26" in t1["j8_operational_lineage"])

    # rev1's worker-sync proof.
    t16 = graded["findings"]["T16_worker_sync_proved_by_the_transplant_report"]
    check("B71 the sync is proved from the transplant report, 14 runners",
          t16["weights_synced_before_first_sample"] is True
          and t16["env_runner_count_checked"] == 14
          and t16["env_runner_actor_digests_count"] == 14
          and t16["env_runner_actors_exact"] is True)
    check("B72 all fourteen runner digests are the expected actor",
          t16["env_runner_actor_digests_all_expected"] is True)
    for block in ("learner_actor", "saved_initial_actor"):
        check("B73 %s is exact with max_abs_diff 0 and the right digests"
              % block,
              t16[block]["exact"] is True
              and t16[block]["max_abs_diff"] == 0.0
              and t16[block]["expected_digest"] == A.EXPECTED_ACTOR_DIGEST
              and t16[block]["actual_digest"] == A.EXPECTED_ACTOR_DIGEST)
    check("B74 weights_seq_no survives only as supporting telemetry",
          t16["weights_seq_no_supporting_telemetry"] == 1.0
          and "gates nothing" in t16["weights_seq_no_is_not_the_proof"])
    lower_source = RUNNER_SOURCE.lower()
    check("B75 the active runner says eighteen, never fifteen",
          "eighteen" in lower_source and "fifteen" not in lower_source)
    check("B76 the active runner never makes the false global claim that no "
          "training occurred",
          "no training has occurred" not in lower_source)
    check("B77 the active success marker scopes zero training to J21 and the "
          "downstream actor-updating run while recording the one critic warm-up",
          "j21 executed zero training iterations" in lower_source
          and "critic-only warm-up had already completed exactly" in lower_source)
    check("B78 the active pin documentation names all 45 additional pins",
          "92 plus 45 additional" in RUNNER_SOURCE)


# ------------------------------------------- C. a missing field is a failure

def test_missing_field_is_a_failure() -> None:
    problems: list = []
    value = A.need({"a": {"b": 1}}, "a.b", "C", problems)
    check("C01 a present field is returned", value == 1 and problems == [])
    problems = []
    value = A.need({"a": {}}, "a.b", "C", problems)
    check("C02 a MISSING field yields a problem and never a default",
          value is None and len(problems) == 1 and "missing field" in problems[0])
    problems = []
    A.need({}, "a.b.c", "C", problems)
    check("C03 a missing intermediate is also a problem", len(problems) == 1)
    problems = []
    value = A.need({"a": {"b": False}}, "a.b", "C", problems)
    check("C04 a present FALSE is not confused with an absent field",
          value is False and problems == [])
    problems = []
    value = A.need({"a": {"b": 0}}, "a.b", "C", problems)
    check("C05 a present ZERO is not confused with an absent field",
          value == 0 and problems == [])

    # And the property end to end: strip a field the gate needs.
    check("C06 a gate fails when its evidence file loses a required field",
          problems_of(A.gate_t9_r3, _mutate(
              lambda e: e["r3_result"].pop("gates_not_evaluated"))))
    check("C07 a gate fails when a whole evidence sub-object is removed",
          problems_of(A.gate_t8_warmup, _mutate(
              lambda e: e["warmup_result"].pop("actor"))))


# ------------------------------------------- D. the mandatory final semantics

def test_final_semantics() -> None:
    on_pass = A.final_fields(True)
    on_fail = A.final_fields(False)
    check("D01 training_ready is true ONLY on a full pass",
          on_pass["training_ready"] is True
          and on_fail["training_ready"] is False)
    check("D02 promotion on pass is exactly TRAINING_INPUT_ONLY",
          on_pass["promotion"] == "TRAINING_INPUT_ONLY"
          == A.PROMOTION_PASS)
    check("D03 promotion on fail is NONE", on_fail["promotion"] == "NONE")
    for field in ("training_started", "sampling", "rollout", "ray_started",
                  "environment_constructed", "launch_authorized",
                  "next_stage_authorized", "ppo_authorized",
                  "ex_novo_authorized", "deployable"):
        check("D04 %s is false on PASS" % field, on_pass[field] is False)
        check("D05 %s is false on FAIL" % field, on_fail[field] is False)
    check("D06 training_iterations_run is 0 on PASS and on FAIL",
          on_pass["training_iterations_run"] == 0
          and on_fail["training_iterations_run"] == 0)
    check("D07 deployable is NEVER true, in either outcome",
          on_pass["deployable"] is False and on_fail["deployable"] is False)
    check("D08 the promotion string is never 'deployable' in any form",
          "deploy" not in A.PROMOTION_PASS.lower()
          and "deploy" not in A.PROMOTION_FAIL.lower())
    check("D09 the two outcomes differ ONLY in training_ready and promotion",
          {k for k in on_pass if on_pass[k] != on_fail[k]}
          == {"training_ready", "promotion"})
    check("D10 the base preregistration declares the same PASS fields",
          PREREG["promotion_semantics"]["mandatory_final_fields_on_PASS"]
          == on_pass)
    check("D11 rev1 declares the same FAIL fields, written out in full",
          REV1["correction_2_training_counters_are_scoped_and_typed"][
              "mandatory_final_fields_on_FAIL"] == on_fail)

    # rev1: counts are NUMBERS, never booleans.
    facts = A.training_history_facts()
    for field in ("training_iterations_run",):
        for outcome, name in ((on_pass, "PASS"), (on_fail, "FAIL")):
            value = outcome[field]
            check("D12 %s is an int on %s, not a bool" % (field, name),
                  isinstance(value, int) and not isinstance(value, bool)
                  and value == 0, "%r (%s)" % (value, type(value).__name__))
    for field in ("critic_warmup_iterations_completed",
                  "checkpoint_logical_iteration", "next_logical_iteration",
                  "j21_execution_training_iterations"):
        value = facts[field]
        check("D13 %s is an int, not a bool" % field,
              isinstance(value, int) and not isinstance(value, bool),
              "%r (%s)" % (value, type(value).__name__))
    check("D14 the truthful warm-up counts are recorded",
          facts["critic_warmup_iterations_completed"] == 1
          and facts["checkpoint_logical_iteration"] == 1
          and facts["next_logical_iteration"] == 2
          and facts["j21_execution_training_iterations"] == 0
          and facts["downstream_actor_training_started"] is False)
    check("D15 downstream_actor_training_started is a bool",
          isinstance(facts["downstream_actor_training_started"], bool))
    check("D16 they agree with the committed warm-up and checkpoint evidence",
          EVIDENCE["warmup_result"]["measurements"][
              "iterations_completed_this_process"]
          == facts["critic_warmup_iterations_completed"]
          and EVIDENCE["warmup_meta"]["logical_iteration"]
          == facts["checkpoint_logical_iteration"]
          and EVIDENCE["r3_result"]["measurements"]["next_iteration"]
          == facts["next_logical_iteration"])

    # rev1: the false flags are SCOPED, not global denials.
    scopes = A.field_scopes()
    check("D17 training_started is scoped to downstream pilot training and J21",
          "downstream post-J20" in scopes["training_started"]
          and "NOT the J20 critic warm-up" in scopes["training_started"])
    check("D18 training_iterations_run carries the same scope",
          "NOT the J20 critic warm-up" in scopes["training_iterations_run"])
    check("D19 environment_constructed is scoped to J21 itself",
          "J21 ITSELF" in scopes["environment_constructed"]
          and "Prior stages did" in scopes["environment_constructed"])
    for field in ("ray_started", "sampling", "rollout"):
        check("D20 %s is scoped to J21 itself" % field,
              "J21 itself" in scopes[field])
    check("D21 deployable is unscoped and always false",
          "always false" in scopes["deployable"])


# -------------------------------------------------- E. the readiness scope

def test_readiness_scope() -> None:
    scope = A.readiness_scope()
    check("E01 the scope names the checkpoint under attestation",
          scope["object"] == A.CHECKPOINT_REL)
    check("E02 it identifies it by hash, not by path alone",
          scope["identified_by"]["module_state_sha256"]
          == A.EXPECTED_MODULE_STATE_SHA
          and scope["identified_by"]["learner_state_sha256"]
          == A.EXPECTED_LEARNER_STATE_SHA)
    check("E03 the pilot protocol is explicitly NOT sealed",
          scope["pilot_protocol_sealed"] is False)
    check("E04 it says so in words a reader cannot miss",
          "NO pilot configuration" in scope["pilot_protocol_note"]
          and "invents neither" in scope["pilot_protocol_note"])
    check("E05 it explains why the attestation is still meaningful",
          "closed list of properties" in scope["why_it_is_still_meaningful"])
    check("E06 it disclaims the pilot config, the launch command and "
          "deployability",
          any("launch command" in item for item in scope["does_not_assert"])
          and any("deployable" in item for item in scope["does_not_assert"])
          and any("configuration" in item for item in scope["does_not_assert"]))
    check("E07 the aggregator invents no pilot config and no launch command",
          "pilot_cfg" not in RUNNER_SOURCE
          and "--iterations" not in RUNNER_SOURCE
          and "pilot_command" not in RUNNER_SOURCE)
    check("E08 the preregistration carries the same scope section",
          PREREG["readiness_scope"]["the_pilot_protocol_is_NOT_SEALED"][
              "status"].startswith("AT THE TIME"))


# ------------------------------- F. the warm-up leaf is byte-immutable

def test_warmup_leaf_immutable() -> None:
    marker = WARMUP_LEAF / "RESTORE_AUDIT_PENDING"
    check("F01 the warm-up leaf still carries RESTORE_AUDIT_PENDING",
          marker.is_file())
    check("F02 the marker matches its pin",
          sha256_file(marker)
          == PINS[A.WARMUP_LEAF_REL + "/RESTORE_AUDIT_PENDING"])
    for forbidden in ("RESTORE_AUDIT_PASSED", "TECHNICAL_INVALID",
                      A.ATTESTED_MARKER):
        check("F03 the warm-up leaf does not carry %s" % forbidden,
              not (WARMUP_LEAF / forbidden).exists())
    check("F04 G9 closure is read from the R3 receipt, not from this leaf",
          EVIDENCE["r3_receipt"]["g9_closed"] is True)
    unrooted = _writes_not_rooted_at_leaf(RUNNER_TREE)
    check("F05 EVERY write the aggregator makes is rooted at its own leaf, so "
          "it cannot reach the warm-up leaf or any other",
          not unrooted, str(unrooted))
    writes = [n for n in ast.walk(RUNNER_TREE)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr in ("write_bytes", "write_text", "mkdir",
                                  "unlink", "rmdir", "replace", "rename")]
    check("F06 the aggregator has exactly the writes the protocol allows",
          len(writes) == 6, "%d write-like calls" % len(writes))
    check("F07 none of them is reachable outside run_execution/verify_commit",
          _writes_only_in(RUNNER_TREE, {"run_execution", "verify_commit"}))


WRITE_CALLS = ("write_bytes", "write_text", "mkdir", "unlink", "rmdir",
               "replace", "rename")


def _root_name(node):
    """The leftmost Name of a path expression such as (leaf / X / Y)."""
    while True:
        if isinstance(node, ast.BinOp):
            node = node.left
        elif isinstance(node, ast.Call):
            node = node.func
        elif isinstance(node, ast.Attribute):
            node = node.value
        elif isinstance(node, ast.Name):
            return node.id
        else:
            return None


def _writes_not_rooted_at_leaf(tree) -> list:
    """Every write-like call whose target is not derived from `leaf`.

    A write rooted anywhere else could reach the warm-up leaf, a prior leaf or
    a production file. There must be none.
    """
    offenders = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr in WRITE_CALLS:
            root = _root_name(node.func.value)
            if root != "leaf":
                offenders.append("%s on a path rooted at %r (line %d)"
                                 % (node.func.attr, root, node.lineno))
    return offenders


def _writes_only_in(tree, allowed: set) -> bool:
    for node in ast.walk(tree):
        if not (isinstance(node, ast.FunctionDef) and node.name not in allowed):
            continue
        for inner in ast.walk(node):
            if isinstance(inner, ast.Call) \
                    and isinstance(inner.func, ast.Attribute) \
                    and inner.func.attr in ("write_bytes", "write_text",
                                            "mkdir", "unlink"):
                return False
    return True


# ---------------------------------------------------- G. preflight and the GO

def test_preflight_and_go() -> None:
    report = A.preflight(verbose=False)
    check("G01 preflight passes", report["ok"], "; ".join(report["problems"]))
    check("G02 preflight reports eighteen gates and eighteen passes",
          report["gates"] == 18 and report["gates_passed"] == 18)
    check("G03 preflight creates NO leaf", not LEAF.exists())
    check("G04 the destination is absent", report["destination_exists"] is False)
    check("G05 preflight reports the final fields it would write",
          report["final_fields_if_it_passed"] == A.final_fields(True))
    check("G06 the preregistration hashes to the pin the aggregator embeds",
          sha256_file(HERE / A.PREREG_NAME) == A.PIN_PREREG)
    check("G07 the preregistration is for this stage and carries no self hash",
          PREREG["stage"] == A.STAGE
          and PREREG["contains_no_self_hash"] is True)
    check("G08 the base preregistration declared fifteen gates",
          PREREG["gates"]["count"] == 15)
    check("G08a rev1 moves the count to eighteen",
          REV1["gate_count"]["base"] == 15 and REV1["gate_count"]["rev1"] == 18
          and sorted(REV1["gate_count"]["added"]) == ["T16", "T17", "T18"])
    check("G09 the aggregator evaluates exactly eighteen gates",
          len(A.evaluate()["gates"]) == 18)
    check("G09a rev1 hashes to the pin the aggregator embeds",
          sha256_file(HERE / A.PREREG_REV1_NAME) == A.PIN_PREREG_REV1)
    check("G09b rev1 cites the base at the pinned hash and takes precedence",
          REV1["cites"]["base_sha256"] == A.PIN_PREREG
          and REV1["precedence"] == "rev1 > base"
          and A.PREREG_PRECEDENCE == ("rev1", "base"))
    check("G09c the base preregistration is sealed and unmodified",
          sha256_file(HERE / A.PREREG_NAME) == A.PIN_PREREG)
    check("G09d the original DRAFT is preserved unedited",
          HISTORICAL_DRAFT_GO.is_file()
          and sha256_file(HISTORICAL_DRAFT_GO)
          == "b2b4f6014e2fdd43dc9ca0714f24311ee0ba67d3d20ec79488531182cd10716c")
    historical = json.loads(HISTORICAL_DRAFT_GO.read_text(encoding="utf-8"))
    check("G09e the original DRAFT is still doubly inert",
          historical.get("status") == "DRAFT"
          and historical.get("authorises_execution") is False)
    check("G09f it is now refused on staleness as well as on status",
          A.validate_go(historical)["valid"] is False
          and any("stale" in p or "no pin for" in p
                  for p in A.validate_go(historical)["problems"]))
    check("G09g the rev1 DRAFT is also preserved unedited",
          REV1_DRAFT_GO.is_file()
          and sha256_file(REV1_DRAFT_GO)
          == "2286c51025508c38fe51735f4830a75c5eca5542fcf0e6bfcc8a78d7e1aca909")
    check("G09h the three DRAFTs are distinct files",
          len({DRAFT_GO, REV1_DRAFT_GO, HISTORICAL_DRAFT_GO}) == 3)

    check("G10 a DRAFT GO exists", DRAFT_GO.is_file())
    payload = json.loads(DRAFT_GO.read_text(encoding="utf-8"))
    check("G11 the DRAFT is doubly inert",
          payload.get("status") == "DRAFT"
          and payload.get("authorises_execution") is False)
    check("G11a the current DRAFT consistently names all eighteen gates",
          payload.get("gates") == 18
          and "conclusion_if_all_eighteen_pass" in payload
          and "conclusion_if_all_fifteen_pass" not in payload
          and "fifteen" not in DRAFT_GO.read_text(encoding="utf-8").lower())
    verdict = A.validate_go(payload)
    check("G12 validate_go refuses the DRAFT", verdict["valid"] is False)
    check("G13 it refuses on BOTH status and authorises_execution",
          any("status is" in p for p in verdict["problems"])
          and any("authorises_execution" in p for p in verdict["problems"]))
    check("G14 the DRAFT pins exactly the labels the aggregator requires",
          sorted(payload.get("pinned_artefacts_sha256") or {})
          == A.go_required_labels(),
          "%d vs %d" % (len(payload.get("pinned_artefacts_sha256") or {}),
                        len(A.go_required_labels())))
    stale = [label for label, digest
             in (payload.get("pinned_artefacts_sha256") or {}).items()
             if A.resolve(label).is_file()
             and sha256_file(A.resolve(label)) != digest]
    check("G15 every DRAFT pin is current", not stale, str(stale))
    for flag in ("authorises_retry", "authorises_ppo", "authorises_ex_novo",
                 "authorises_promotion", "authorises_training",
                 "authorises_launch", "authorises_deployment"):
        check("G16 the DRAFT never sets %s" % flag,
              payload.get(flag) is not True)
    check("G17 an APPROVED GO for another stage is refused",
          A.validate_go(dict(payload, status="APPROVED",
                             authorises_execution=True,
                             stage="V26C_J20_RESTORE_AUDIT_R3"))["valid"]
          is False)
    check("G18 only the exact string APPROVED authorises",
          A.validate_go(dict(payload, status="approved",
                             authorises_execution=True))["valid"] is False)
    check("G19 a GO that authorises training is refused",
          A.validate_go(dict(payload, status="APPROVED",
                             authorises_execution=True,
                             authorises_training=True))["valid"] is False)
    check("G20 a GO that authorises a launch is refused",
          A.validate_go(dict(payload, status="APPROVED",
                             authorises_execution=True,
                             authorises_launch=True))["valid"] is False)
    approved = dict(payload, status="APPROVED", authorises_execution=True)
    check("G21 a correctly scoped APPROVED evidence-aggregation GO is valid",
          A.validate_go(approved)["valid"] is True,
          str(A.validate_go(approved)["problems"]))
    for label, mutate in (
        ("missing user authorisation", lambda p: p.pop("user_authorisation")),
        ("non-explicit user authorisation",
         lambda p: p["user_authorisation"].__setitem__("explicit", False)),
        ("training launch authorised",
         lambda p: p["user_authorisation"].__setitem__(
             "training_launch_authorised", True)),
        ("wrong authorisation scope",
         lambda p: p["user_authorisation"].__setitem__("scope", "training")),
        ("altered attestation quote",
         lambda p: p["user_authorisation"].__setitem__(
             "attestation_quote_verbatim", "arriva training ready")),
        ("altered stop quote",
         lambda p: p["user_authorisation"].__setitem__(
             "stop_quote_verbatim", "lancia il training")),
    ):
        candidate = copy.deepcopy(approved)
        mutate(candidate)
        check("G22 the GO refuses %s" % label,
              A.validate_go(candidate)["valid"] is False)


# --------------------------------------- H. the disclosed accepted diagnostics

def test_accepted_diagnostics_are_disclosed() -> None:
    graded = A.evaluate()
    found = graded["findings"]["T5_penetration_contract_and_no_hard_breach"]
    accepted = found["accepted_diagnostics"]
    check("H01 all nine cells are recorded as above the 20 mm soft band",
          sorted(accepted["cells_above_soft_20mm"])
          == sorted(A.J19B_CELLS + A.J19C_CELLS),
          str(accepted["cells_above_soft_20mm"]))
    check("H02 exactly F and H are recorded at or above the 25 mm July band",
          sorted(accepted["cells_at_or_above_july_25mm"]) == ["F", "H"],
          str(accepted["cells_at_or_above_july_25mm"]))
    check("H03 no cell is above the 28 mm hard band",
          accepted["cells_above_hard_28mm"] == [])
    check("H04 the soft and July bands are recorded as diagnostic only",
          accepted["soft_and_july_are_diagnostic_only"] is True
          and accepted["sole_binding_threshold_m"] == 0.028)
    check("H05 the hard band is the only one that can fail the stage",
          found["no_hard_breach"] is True)
    for cid in ("F", "H"):
        cell = found["cells"][cid]
        check("H06 cell %s records 4 samples at or above 25 mm of 500" % cid,
              cell["at_or_above_july_legacy"] == 4 and cell["samples"] == 500,
              str(cell))
    check("H07 the preregistration discloses the same two cells BEFORE any run",
          set(PREREG["accepted_diagnostics_disclosed"][
              "two_cells_exceed_the_25mm_july_legacy_band"]["fact"].split())
          & {"F", "H"} == {"F", "H"})
    check("H08 the preregistration discloses that every cell exceeds 20 mm",
          "all nine cells report above_soft_diagnostic true"
          in PREREG["accepted_diagnostics_disclosed"][
              "every_cell_exceeds_the_soft_20mm_band"]["fact"])
    check("H09 J19A's two diagnostic-only criteria are recorded, not gated",
          set(graded["findings"][
              "T2_j19a_reproducibility_and_offline_eligibility"][
                  "diagnostic_only_recorded_not_gated"])
          == set(A.J19A_DIAGNOSTIC_IDS))


def main() -> int:
    test_aggregator_is_read_only()
    test_gates_pass_on_the_real_evidence()
    test_each_gate_flips()
    test_leaf_integrity_hermetically()
    test_rev1_semantics()
    test_missing_field_is_a_failure()
    test_final_semantics()
    test_readiness_scope()
    test_warmup_leaf_immutable()
    test_preflight_and_go()
    test_accepted_diagnostics_are_disclosed()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
