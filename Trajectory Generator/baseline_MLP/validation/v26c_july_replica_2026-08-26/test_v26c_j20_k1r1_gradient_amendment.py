"""Static test suite for V26C J20-K1R1 - non-degenerate gradient amendment.

Executes no probe, no warm-up, no training, no rollout, and builds no
environment. It holds the amendment to five properties:

  * it is STRUCTURALLY unable to train, sample, build an environment, start Ray
    or promote a checkpoint - asserted by walking its own AST;
  * it cannot run unless the K1 evidence is EXACTLY what it was written for:
    FAIL_CLOSED 28/29, P24 the sole failure, commit verification OK;
  * the stimulus is exactly the preregistered construction and satisfies every
    declared property, including that it is NOT the degenerate zero batch;
  * the composite gate cannot report 29/29 by any route other than the immutable
    twenty-eight plus a genuinely passing amended P24, and it never rewrites K1;
  * the amended P24 is STRICTLY STRONGER than the check it replaces.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_k1r1_gradient_amendment.py
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

import numpy as np  # noqa: E402

import v26c_j20_k1r1_gradient_amendment as A  # noqa: E402

RUNNER_PATH = HERE / "v26c_j20_k1r1_gradient_amendment.py"
RUNNER_SOURCE = RUNNER_PATH.read_text(encoding="utf-8")
RUNNER_TREE = ast.parse(RUNNER_SOURCE)
PREREG = json.loads((HERE / A.PREREG_NAME).read_text(encoding="utf-8"))

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


# ------------------------------------------- the amendment cannot train at all

def test_structurally_inert() -> None:
    calls = called_names(RUNNER_TREE)
    for forbidden in A.FORBIDDEN_CALLS:
        check("A01 the runner never calls %s()" % forbidden,
              forbidden not in calls, "found in call position")
    imports = imported_modules(RUNNER_TREE)
    for forbidden in A.FORBIDDEN_IMPORTS:
        check("A02 the runner never imports %s" % forbidden,
              forbidden not in imports and not any(
                  name.startswith(forbidden + ".") for name in imports))
    check("A02b the runner imports only the pinned production module",
          "asymmetric_rl_module" in imports
          and not any(name in imports for name in
                      ("warm_start", "env_factory", "rollout_eval",
                       "train_ppo_mlp", "v26c_j1_collect")),
          str(sorted(imports)))

    backward_functions = set()
    for node in ast.walk(RUNNER_TREE):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for inner in ast.walk(node):
                if (isinstance(inner, ast.Call)
                        and isinstance(inner.func, ast.Attribute)
                        and inner.func.attr == "backward"):
                    backward_functions.add(node.name)
    check("A03 the only backward pass is inside measure_gradients",
          backward_functions == {"measure_gradients"}, str(sorted(backward_functions)))
    check("A04 there is exactly one backward call in the whole file",
          sum(1 for node in ast.walk(RUNNER_TREE)
              if isinstance(node, ast.Call)
              and isinstance(node.func, ast.Attribute)
              and node.func.attr == "backward") == 1)
    check("A05 no optimizer is constructed",
          not any(name in calls for name in ("Adam", "SGD", "AdamW", "RMSprop")))
    optim_attribute = any(
        isinstance(node, ast.Attribute) and node.attr == "optim"
        and isinstance(node.value, ast.Name) and node.value.id == "torch"
        for node in ast.walk(RUNNER_TREE))
    check("A06 torch.optim is never accessed as an attribute either",
          not optim_attribute)

    probe_source = RUNNER_SOURCE[RUNNER_SOURCE.index("def run_probe"):]
    probe_source = probe_source[:probe_source.index("\n# ---")]
    check("A07 run_probe refuses to start without a valid GO",
          'if not go["valid"]' in probe_source)
    check("A08 run_probe refuses to start without a passing preflight",
          'if not report["ok"]' in probe_source)
    check("A09 run_probe refuses to start unless verify_original passes",
          "verify_original()" in probe_source)

    # No RNG anywhere: the stimulus must be a property of this file alone.
    check("A10 the stimulus uses no random number generator",
          not any(name in calls for name in
                  ("rand", "randn", "random", "default_rng", "manual_seed",
                   "seed", "uniform", "normal")),
          "the stimulus must be algebraic, not sampled")


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
          completed.stdout[-300:])

    dry = subprocess.run(
        [sys.executable, str(RUNNER_PATH), "--dry-run"],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    after_dry = sorted((str(p.relative_to(HERE)), p.stat().st_size)
                       for p in HERE.rglob("*") if p.is_file())
    check("B04 --dry-run exits 0 and writes nothing",
          dry.returncode == 0 and after_dry == before)
    check("B05 --dry-run states that K1 is not retried",
          "NOT retried" in dry.stdout, dry.stdout[-300:])

    probe = subprocess.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, %r); sys.dont_write_bytecode = True;\n"
         "import v26c_j20_k1r1_gradient_amendment as A;\n"
         "A.preflight(verbose=False);\n"
         "print('torch' in sys.modules, 'ray' in sys.modules)" % str(HERE)],
        cwd=str(HERE), capture_output=True, text=True,
        env={"PYTHONDONTWRITEBYTECODE": "1", "PATH": "/usr/bin:/bin"})
    check("B06 the preflight imports neither torch nor ray",
          probe.stdout.strip() == "False False",
          probe.stdout.strip() + probe.stderr[-300:])


def test_preflight_passes() -> None:
    report = A.preflight(verbose=False)
    check("B07 the preflight verdict is READY", report["ok"],
          "; ".join(report["problems"]))
    check("B08 every pin matches",
          report["pins_matching"] == report["pins_checked"]
          and report["pins_checked"] >= 12,
          "%d/%d" % (report["pins_matching"], report["pins_checked"]))
    check("B09 the destination leaf does not exist yet",
          report["destination"]["ok"])
    check("B10 the K1 leaf is present and is not the destination",
          report["destination"]["k1_leaf_present"]
          and A.LEAF_NAME != A.K1_LEAF_REL.split("/")[-1])
    check("B11 the preflight declares it does not retry or rewrite K1",
          report["it_does_not_retry_k1"] and report["it_does_not_rewrite_the_k1_result"])


# ---------------------------------------------------------------- the stimulus

def test_stimulus() -> None:
    report = A.stimulus_report(np)
    matrix = report["matrix"]
    check("C01 the stimulus satisfies every declared property",
          report["ok"], str({k: v for k, v in report["checks"].items() if not v}))
    check("C02 shape is exactly 8 x 84 float32",
          matrix.shape == (8, 84) and str(matrix.dtype) == "float32")
    check("C03 every element is strictly non-zero",
          bool(np.all(matrix != 0.0)))
    check("C04 every element lies within [-1, 1]",
          bool(np.all(np.abs(matrix) <= 1.0)),
          "min %r max %r" % (report["min"], report["max"]))
    check("C05 all eight rows are distinct",
          len({matrix[i].tobytes() for i in range(8)}) == 8)
    check("C06 no column is constant: every column has eight distinct values",
          report["min_distinct_values_per_column"] == 8,
          str(report["min_distinct_values_per_column"]))
    check("C07 every row has 84 distinct values",
          min(len(np.unique(matrix[i])) for i in range(8)) == 84)
    check("C08 the digest matches the pinned one",
          report["digest_float32_c_order"] == A.PIN_STIMULUS_DIGEST,
          report["digest_float32_c_order"])
    check("C09 the pinned range and abs-min are the measured ones",
          report["min"] == A.PIN_STIMULUS_MIN
          and report["max"] == A.PIN_STIMULUS_MAX
          and report["abs_min"] == A.PIN_STIMULUS_ABS_MIN)
    # The range must be BINDING at run time, not merely asserted here.
    check("C09a the range is a runtime check inside stimulus_report",
          {"matches_the_pinned_minimum", "matches_the_pinned_maximum",
           "matches_the_pinned_absolute_minimum"} <= set(report["checks"]),
          str(sorted(report["checks"])))
    check("C09b stimulus_report publishes eleven properties",
          len(report["checks"]) == 11, str(len(report["checks"])))
    for pin in ("PIN_STIMULUS_MIN", "PIN_STIMULUS_MAX", "PIN_STIMULUS_ABS_MIN"):
        original = getattr(A, pin)
        try:
            setattr(A, pin, original + 1.0)
            perturbed = A.stimulus_report(np)
        finally:
            setattr(A, pin, original)
        check("C09c stimulus['ok'] fails when %s differs" % pin,
              not perturbed["ok"],
              "a drifted range must block the stage, not just a test")
    check("C10 the signs are balanced, so no trivial one-sided structure",
          report["positive_elements"] == report["negative_elements"] == 336)

    # DETERMINISM: recomputing must give byte-identical bytes, twice.
    again = A.stimulus_matrix(np)
    third = A.stimulus_matrix(np)
    check("C11 the construction is deterministic across recomputation",
          np.ascontiguousarray(again).tobytes(order="C")
          == np.ascontiguousarray(matrix).tobytes(order="C")
          == np.ascontiguousarray(third).tobytes(order="C"))

    # It must be the OPPOSITE of K1's stimulus in the one way that matters.
    check("C12 it is not the degenerate all-zero batch K1 used",
          not bool(np.all(matrix == 0.0)) and bool(np.all(matrix != 0.0)))

    # An independent re-derivation of the formula, written out here, must agree.
    independent = np.empty((8, 84), dtype=np.float32)
    for i in range(8):
        for j in range(84):
            p, q = i + 1, j + 1
            num = (p * 37 + q * 53 + p * q) % 191 + 1
            independent[i, j] = np.float32((-1.0 if (p + q) % 2 else 1.0)
                                           * num / 192.0)
    check("C13 an independent re-derivation of the formula agrees byte for byte",
          np.ascontiguousarray(independent).tobytes(order="C")
          == np.ascontiguousarray(matrix).tobytes(order="C"))

    check("C14 the row weights are eight distinct positive values",
          len(set(A.ROW_WEIGHTS)) == 8 and all(w > 0 for w in A.ROW_WEIGHTS)
          and len(A.ROW_WEIGHTS) == 8, str(A.ROW_WEIGHTS))

    # The property list must be enforced, not merely reported.
    check("C15 the stage refuses to proceed on a failed stimulus property",
          "stimulus property failed" in RUNNER_SOURCE)


# ------------------------------------------------- the immutable K1 evidence

def test_original_evidence() -> None:
    original = A.verify_original()
    check("D01 the K1 record is readable and exactly as expected",
          original["ok"], "; ".join(original["problems"]))
    check("D02 the K1 verdict is FAIL_CLOSED",
          original["verdict"] == "FAIL_CLOSED")
    check("D03 the K1 tally is 28 of 29",
          original["checks_passed"] == 28 and original["checks_total"] == 29)
    check("D04 P24 is its sole failure",
          original["failed_checks"] == [A.K1_SOLE_FAILED_CHECK])
    check("D05 the other twenty-eight are the ones this amendment composes with",
          len(original["passing_checks"]) == 28
          and tuple(original["passing_checks"]) == tuple(sorted(A.K1_PASSING_CHECKS)))
    check("D06 the K1 commit verification passed",
          original["commit_verification_ok"])
    check("D07 the K1 leaf carries no TECHNICAL_INVALID marker",
          not (HERE / A.K1_LEAF_REL / "TECHNICAL_INVALID").exists())
    for name, expected in sorted(A.PIN_K1_LEAF.items()):
        check("D08 K1 leaf pin matches: %s" % name,
              sha256_file(HERE / A.K1_LEAF_REL / name) == expected)
    check("D09 the K1 leaf holds exactly six files",
          sum(1 for p in (HERE / A.K1_LEAF_REL).rglob("*") if p.is_file()) == 6)
    check("D10 the K1 runner is pinned at the exact bytes that produced it",
          sha256_file(HERE / A.K1_RUNNER_NAME) == A.PIN_K1_RUNNER)
    check("D11 the K1 preregistration and GO are pinned too",
          sha256_file(HERE / A.K1_PREREG_NAME) == A.PIN_K1_PREREG
          and sha256_file(HERE / A.K1_GO_NAME) == A.PIN_K1_GO)
    check("D12 the J19A module is pinned",
          sha256_file(HERE / A.J19A_MODULE_REL / "module_state.pkl")
          == A.PIN_J19A_MODULE_STATE)
    for name, expected in sorted(A.BASELINE_PINS.items()):
        check("D13 the executed production module is pinned: %s" % name,
              sha256_file(BASELINE / name) == expected)

    # verify_original must REFUSE anything that is not the exact evidence.
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("def verify_original"):]
    source = source[:source.index("\n# ---")]
    for phrase in ("K1_EXPECTED_VERDICT", "K1_EXPECTED_TOTAL",
                   "K1_EXPECTED_PASSED", "K1_SOLE_FAILED_CHECK",
                   "K1_PASSING_CHECKS", "commit verification did not pass",
                   "TECHNICAL_INVALID"):
        check("D14 verify_original enforces %s" % phrase, phrase in source)


# ------------------------------------------------------------ the composite

def test_composite_semantics() -> None:
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("    amended_p24 = bool("):]
    source = source[:source.index("    result = {")]
    check("E01 the amended P24 requires all six critic gradients",
          "A06_all_six_critic_tensors_have_strictly_positive_gradient_norm" in source)
    check("E02 the amended P24 requires vf_encoder.0.weight specifically",
          "A07_vf_encoder_0_weight_specifically_has_gradient" in source)
    check("E03 the amended P24 requires NO actor gradient",
          "A08_no_actor_parameter_received_gradient" in source)
    check("E04 the composite requires the original record to be intact",
          'original["ok"]' in source)
    check("E05 the composite requires every amended check to pass",
          "all(amended.values())" in source)
    check("E06 the composite total stays 29",
          "K1_EXPECTED_TOTAL" in source)
    check("E07 the composite counts 28 immutable plus the amended one",
          "28 + (1 if amended_p24 else 0)" in source)
    check("E08 the composite declares it does not rewrite K1",
          "the_k1_result_is_NOT_rewritten" in source
          and "the_k1_leaf_still_reads_fail_closed" in source)

    # The strictness that closes the original defect.
    grade = RUNNER_SOURCE[RUNNER_SOURCE.index("    critic_all_positive = all("):]
    grade = grade[:grade.index("    unchanged = {")]
    check("E09 a gradient must be finite AND strictly positive, not merely present",
          "value > 0.0" in grade and "isfinite" in grade and "is not None" in grade,
          "K1's failure was a present-but-zero gradient; requiring presence "
          "would re-admit the defect")
    check("E10 the actor must have NO gradient, or exactly zero",
          "value is None or value == 0.0" in grade)

    # It must never write into the K1 leaf.
    check("E11 the runner writes only into its own leaf",
          A.LEAF_NAME in RUNNER_SOURCE
          and 'final = leaf_root / LEAF_NAME' in RUNNER_SOURCE)
    check("E12 the K1 leaf path is only ever read",
          "K1_LEAF_REL" in RUNNER_SOURCE
          and "write_bytes" not in RUNNER_SOURCE[
              RUNNER_SOURCE.index("def verify_original"):
              RUNNER_SOURCE.index("def check_pins")])
    check("E13 the destination differs from the K1 leaf",
          A.LEAF_NAME not in A.K1_LEAF_REL)


def test_exact_load_and_freeze_contract() -> None:
    """A04 must bind on byte identity of the load AND on a live freeze contract."""
    body = RUNNER_SOURCE[RUNNER_SOURCE.index("    loaded_vs_saved = {"):]
    body = body[:body.index("    measurement = measure_gradients(")]
    check("J01 every loaded tensor is compared to the saved pickle",
          "bytes_identical(before[key], saved[key])" in body)
    check("J02 the key SETS are compared, not just the count",
          '"same_key_set": sorted(before) == sorted(saved)' in body)
    check("J03 the per-key map and the count are recorded",
          '"per_key_byte_identical"' in body and '"identical_count"' in body)
    check("J04 the comparison method is dtype + shape + C-order bytes",
          '"method": "same dtype, same shape, same C-order bytes, per key"' in body)
    check("J05 the freeze flags are read off the LIVE module",
          '"_freeze_actor": getattr(module, "_freeze_actor", None)' in body
          and '"_freeze_logstd": getattr(module, "_freeze_logstd", None)' in body)
    check("J06 the live contract also checks inference_only and the geometry",
          '"inference_only": getattr(module, "inference_only", None)' in body
          and '"_n_actor"' in body and '"_n_full"' in body
          and '"_action_dim"' in body)
    check("J07 the live contract requires both value-tower children to exist",
          '"has_vf": hasattr(module, "vf")' in body
          and '"has_vf_encoder": hasattr(module, "vf_encoder")' in body)
    check("J08 the contract requires freeze_actor and freeze_logstd True",
          'freeze_contract["_freeze_actor"] is True' in body
          and 'freeze_contract["_freeze_logstd"] is True' in body)

    a04 = RUNNER_SOURCE[RUNNER_SOURCE.index('"A04_'):]
    a04 = a04[:a04.index('"A05_')]
    check("J09 A04 requires an empty missing and unexpected key set",
          "tuple(missing) == ()" in a04 and "tuple(unexpected) == ()" in a04)
    check("J10 A04 requires sixteen tensors on both sides",
          "len(saved) == 16" in a04
          and 'load_exactness["tensor_count_live"] == 16' in a04)
    check("J11 A04 requires the same key set and all sixteen byte-identical",
          'load_exactness["same_key_set"]' in a04
          and 'load_exactness["all_identical"]' in a04
          and 'load_exactness["identical_count"] == 16' in a04)
    check("J12 A04 binds on the live freeze contract",
          "freeze_contract_ok" in a04)
    check("J13 the evidence reaches the result",
          '"load_exactness": load_exactness' in RUNNER_SOURCE
          and '"freeze_contract": freeze_contract' in RUNNER_SOURCE)
    # The rationale sits in the comment just above the comparison, so it is
    # outside the slice used for the checks above; widen the window.
    rationale = RUNNER_SOURCE[RUNNER_SOURCE.index("    missing, unexpected = module.load_state_dict("):]
    rationale = rationale[:rationale.index("    freeze_contract = {")]
    check("J14 strict=True alone is documented as insufficient",
          "does NOT prove the" in rationale and "signed zero" in rationale,
          "the reason for the extra comparison must be on the record")


def test_k1_immutability_is_derived_three_times() -> None:
    """Before the measurement, after the backward, and after the commit."""
    body = RUNNER_SOURCE[RUNNER_SOURCE.index("    original_after = verify_original()"):]
    body = body[:body.index("    norms = measurement")]
    check("K01 verify_original is repeated AFTER the backward",
          "original_after = verify_original()" in body)
    check("K02 the two hash sets are compared to each other",
          'original_after["hashes"] == original["hashes"]' in body)
    check("K03 and both are compared to the pinned values",
          'original_after["hashes"] == dict(PIN_K1_LEAF)' in body)

    a01 = RUNNER_SOURCE[RUNNER_SOURCE.index('"A01_'):]
    a01 = a01[:a01.index('"A02_')]
    check("K04 A01 binds on BOTH verifications and on their agreement",
          'original["ok"]' in a01 and 'original_after["ok"]' in a01
          and "k1_unchanged_across_measurement" in a01)

    composite = RUNNER_SOURCE[RUNNER_SOURCE.index("    composite_ok = bool("):]
    composite = composite[:composite.index("    composite = {")]
    check("K05 the composite fails closed on either verification",
          'original["ok"]' in composite and 'original_after["ok"]' in composite
          and "k1_unchanged_across_measurement" in composite)

    commit = RUNNER_SOURCE[RUNNER_SOURCE.index("    k1_post_commit = {}"):]
    commit = commit[:commit.index("    verification = {")]
    check("K06 all six K1 files are re-hashed a THIRD time after the commit",
          "for name, expected in sorted(PIN_K1_LEAF.items())" in commit)
    check("K07 a post-commit mismatch appends a problem, so the marker stays",
          "problems.append" in commit and "changed: expected" in commit)
    marker = RUNNER_SOURCE[RUNNER_SOURCE.index("    if problems:\n        raise K1R1Error"):]
    check("K08 problems leave TECHNICAL_INVALID in place",
          "the invalid marker is" in marker[:400])

    check("K09 the unchanged flag is DERIVED, never written as a constant true",
          '"k1_leaf_unchanged": True' not in RUNNER_SOURCE
          and "k1_leaf_unchanged_across_measurement" in RUNNER_SOURCE)
    check("K10 before, after and post-commit evidence all reach the artefacts",
          '"hashes_before_measurement"' in RUNNER_SOURCE
          and '"hashes_after_measurement"' in RUNNER_SOURCE
          and '"k1_leaf_re_hashed_post_commit"' in RUNNER_SOURCE)
    check("K11 the result explains that the flag is derived",
          "unchanged_is_DERIVED_not_declared" in RUNNER_SOURCE)


def test_precise_language() -> None:
    """The evidentiary path differs; the runner must not claim otherwise."""
    check("L01 the runner no longer claims nothing else differs",
          "Nothing else moves" not in RUNNER_SOURCE
          and '"nothing_else_differs"' not in RUNNER_SOURCE)
    check("L02 it names W1 and W2 as the only SCIENTIFIC differences",
          "ONLY SCIENTIFIC differences" in RUNNER_SOURCE)
    check("L03 it states the evidentiary path differs intentionally",
          "EVIDENTIARY PATH differs intentionally" in RUNNER_SOURCE
          or "the_EVIDENTIARY_PATH_differs_intentionally" in RUNNER_SOURCE)
    check("L04 it says explicitly it is NOT operationally identical to K1",
          "NOT operationally identical" in RUNNER_SOURCE)
    check("L05 it says what IS identical: the thing measured",
          "identical in what is measured" in RUNNER_SOURCE
          or "identical in what is MEASURED" in RUNNER_SOURCE)

    whitelist = PREREG["authorised_differences_from_k1_closed_whitelist"]
    check("L06 the preregistration uses the same precise formulation",
          "ONLY SCIENTIFIC differences" in whitelist["rule"]
          and "the_EVIDENTIARY_PATH_differs_intentionally" in whitelist)
    check("L07 the preregistration drops the old absolute claim",
          "nothing_else_differs" not in whitelist)
    check("L08 the preregistration says NOT operationally identical",
          "NOT operationally identical"
          in whitelist["the_EVIDENTIARY_PATH_differs_intentionally"])


def test_byte_and_immutability_checks() -> None:
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("    unchanged = {"):]
    source = source[:source.index("    amended = {")]
    check("F01 all sixteen tensors are compared before vs after the backward",
          "for key in before" in source)
    check("F02 the ten actor tensors are compared against J19A on disk",
          "on_disk" in source and "ACTOR_KEYS" in source)
    check("F03 the canonical actor digest is recomputed after the backward",
          "actor_digest(after)" in source)
    check("F04 the log-std rows are compared byte for byte",
          "LOGSTD_ROWS" in source and "bytes_identical" in source)
    # AST, not a substring hunt: "array_equal" legitimately appears in prose
    # that explains why it is NOT used. What matters is that it is never CALLED.
    check("F05 array_equal is never called: byte identity is dtype + shape + "
          "C-order bytes",
          "array_equal" not in called_names(RUNNER_TREE)
          and 'tobytes(order="C")' in RUNNER_SOURCE)
    check("F05b the only equality primitive is bytes_identical",
          "bytes_identical" in called_names(RUNNER_TREE))
    check("F06 the canonical digest transcription reproduces J19A's",
          A.PIN_J19A_ACTOR_DIGEST
          == "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96")

    # The transcribed digest must actually reproduce J19A's, computed here.
    import pickle

    state = pickle.loads((HERE / A.J19A_MODULE_REL
                          / "module_state.pkl").read_bytes())
    check("F07 the transcribed actor_digest reproduces J19A from its bytes",
          A.actor_digest({k: np.asarray(v) for k, v in state.items()})
          == A.PIN_J19A_ACTOR_DIGEST)

    # bytes_identical must be strict.
    a32 = np.zeros((2, 2), dtype=np.float32)
    a64 = np.zeros((2, 2), dtype=np.float64)
    check("F08 bytes_identical rejects a dtype change", not A.bytes_identical(a32, a64))
    signed = np.array([[-0.0, 0.0]], dtype=np.float32)
    unsigned = np.array([[0.0, 0.0]], dtype=np.float32)
    check("F09 bytes_identical rejects signed zero, which array_equal accepts",
          not A.bytes_identical(signed, unsigned)
          and bool(np.array_equal(signed, unsigned)))
    check("F10 bytes_identical rejects a shape change",
          not A.bytes_identical(a32, np.zeros((4,), dtype=np.float32)))


# ---------------------------------------------------------------------- the GO

def valid_go() -> dict:
    return {
        "kind": "ARCHITECT GO - SINGLE EXECUTION",
        "stage": A.GO_REQUIRED_STAGE,
        "authorises_execution": True,
        "pinned_artefacts_sha256": {
            label: sha256_file(path)
            for label, path in A.go_pin_targets().items()
        },
    }


def test_go_validation() -> None:
    check("G01 a well-formed GO validates", A.validate_go(valid_go())["valid"])

    bad = valid_go()
    bad["stage"] = "V26C_J20_CRITIC_WARMUP_READINESS"
    check("G02 a GO for the ORIGINAL K1 stage is refused here",
          not A.validate_go(bad)["valid"])

    bad = valid_go()
    bad["authorises_execution"] = "yes"
    check("G03 authorises_execution must be exactly true",
          not A.validate_go(bad)["valid"])

    for forbidden in ("authorises_warmup", "authorises_training",
                      "authorises_rollout", "authorises_ppo",
                      "authorises_k1_retry", "authorises_rewriting_k1"):
        bad = valid_go()
        bad[forbidden] = True
        check("G04 a K1R1 GO may not set %s" % forbidden,
              not A.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"].pop(A.PREREG_NAME)
    check("G05 a GO missing a required pin is refused",
          not A.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"][A.PREREG_NAME] = "0" * 64
    check("G06 a GO with a stale pin is refused",
          not A.validate_go(bad)["valid"])

    bad = valid_go()
    bad["pinned_artefacts_sha256"]["v26c_j20_warmup_cfg.yaml"] = "0" * 64
    check("G07 a GO pinning something outside the scope is refused",
          not A.validate_go(bad)["valid"])

    check("G08 an absent GO file is refused, not defaulted",
          not A.load_go(str(HERE / "no_such_go.json"))["valid"])
    targets = A.go_pin_targets()
    check("G09 the pin map is closed and has exactly fourteen labels",
          len(targets) == 14 and tuple(sorted(targets)) == A.GO_REQUIRED_PINS,
          str(len(targets)))
    check("G10 it pins this stage's own three artefacts",
          {A.PREREG_NAME, A.RUNNER_NAME, A.TEST_NAME} <= set(targets))
    check("G11 it pins ALL SIX immutable K1 leaf files",
          {A.K1_LEAF_REL + "/" + name for name in A.PIN_K1_LEAF} <= set(targets),
          str(sorted(A.PIN_K1_LEAF)))
    check("G12 it pins the K1 runner, preregistration and GO",
          {A.K1_RUNNER_NAME, A.K1_PREREG_NAME, A.K1_GO_NAME} <= set(targets))
    check("G13 it pins the J19A module state",
          (A.J19A_MODULE_REL + "/module_state.pkl") in targets)
    check("G14 it pins the one production module the stage imports",
          A.BASELINE_MODULE_LABEL in targets
          and targets[A.BASELINE_MODULE_LABEL]
          == BASELINE / "asymmetric_rl_module.py")
    for label, path in sorted(targets.items()):
        check("G15 the pin target exists: %s" % label, path.is_file(), str(path))

    # The path must come from the MAP, never from the payload. A label that is
    # not a path relative to HERE proves it: resolving it naively would fail.
    check("G16 a pinned label need not be a path relative to this directory",
          not (HERE / A.BASELINE_MODULE_LABEL).exists()
          and targets[A.BASELINE_MODULE_LABEL].is_file(),
          "the baseline module is resolved from an internal constant")
    source = RUNNER_SOURCE[RUNNER_SOURCE.index("def validate_go"):]
    source = source[:source.index("def load_go")]
    check("G17 validate_go resolves paths from the closed map, not the payload",
          "targets = go_pin_targets()" in source
          and "path = targets[label]" in source
          and "root / " not in source)
    check("G18 validate_go reports how many labels were required and supplied",
          A.validate_go(valid_go())["pin_labels_required"] == 14
          and A.validate_go(valid_go())["pin_labels_supplied"] == 14)


# ------------------------------------------------------- preregistration

def test_prereg() -> None:
    check("H01 the preregistration is sealed to a real hash",
          len(A.PIN_PREREG) == 64
          and all(c in "0123456789abcdef" for c in A.PIN_PREREG), A.PIN_PREREG)
    check("H02 the sealed hash is the preregistration's actual hash",
          A.PIN_PREREG == sha256_file(HERE / A.PREREG_NAME))
    check("H03 the check is UNCONDITIONAL - no placeholder branch",
          "SEALED_BEFORE_ANY_GO" not in RUNNER_SOURCE
          and "PENDING" not in RUNNER_SOURCE)
    check("H04 the preregistration contains no self-hash",
          A.PIN_PREREG not in (HERE / A.PREREG_NAME).read_text(encoding="utf-8")
          and PREREG["contains_no_self_hash"] is True)
    check("H05 it declares the stage and what it amends",
          PREREG["stage"] == A.STAGE and PREREG["amends"] == A.AMENDS_STAGE)
    check("H06 it states the status as not executed",
          "NOT EXECUTED" in PREREG["status"])

    whitelist = PREREG["authorised_differences_from_k1_closed_whitelist"]
    check("H07 exactly two differences from K1 are authorised",
          sorted(k for k in whitelist if k.startswith("W"))
          == ["W1_stimulus", "W2_row_weights"], str(sorted(whitelist)))
    check("H08 the stimulus construction in the prereg matches the runner",
          whitelist["W1_stimulus"]["construction"]
          == A.stimulus_report(np)["construction"])
    check("H09 the pinned stimulus digest matches the runner constant",
          whitelist["W1_stimulus"]["pinned_digest_float32_c_order"]
          == A.PIN_STIMULUS_DIGEST)
    check("H10 the prereg row weights match the runner",
          tuple(whitelist["W2_row_weights"]["k1r1"]) == A.ROW_WEIGHTS)
    check("H11 the prereg records that no RNG is used",
          "no_rng_no_seed" in whitelist["W1_stimulus"])

    entry = PREREG["entry_conditions_that_must_hold_before_it_can_run"]
    check("H12 the prereg pins the same K1 leaf hashes as the runner",
          entry["pinned_k1_leaf_sha256"] == dict(A.PIN_K1_LEAF))
    check("H13 the prereg pins the K1 runner hash the architect named",
          entry["pinned_k1_runner_sha256"] == A.PIN_K1_RUNNER
          == "d33e0709ca4682ef857e07a85c7015d2d857aba2b04f454afb3bc1aa0c57858b")
    check("H14 the prereg requires FAIL_CLOSED 28/29 with P24 as sole failure",
          entry["k1_verdict_required"] == "FAIL_CLOSED"
          and entry["k1_checks_required"] == "28 of 29"
          and entry["k1_sole_failed_check_required"] == A.K1_SOLE_FAILED_CHECK)

    forbid = PREREG["what_this_amendment_must_not_do"]
    for key in ("no_retry_of_k1", "no_overwrite", "no_rewriting_the_verdict",
                "no_new_transplant", "no_fresh_critic", "no_weakening_of_P24"):
        check("H15 the prereg forbids: %s" % key, key in forbid)

    composite = PREREG["composite_gate"]
    check("H16 the composite gate is preregistered as 29/29 or nothing",
          "29/29" in composite["rule"]
          and composite["the_k1_result_is_NOT_rewritten"] is True)
    check("H17 no threshold is invented",
          composite["thresholds_invented_in_this_stage"] == 0)
    check("H18 the prereg explains why strictly-positive and not merely-present",
          "present-but-zero" in composite["why_strictly_positive_and_not_merely_present"])
    check("H19 the prereg lists exactly twelve amended checks",
          len(PREREG["the_twelve_amended_checks"]) == 12,
          str(len(PREREG["the_twelve_amended_checks"])))
    check("H20 the amended check ids are A01..A12 in order",
          [entry.split()[0] for entry in PREREG["the_twelve_amended_checks"]]
          == ["A%02d" % n for n in range(1, 13)])

    discipline = PREREG["execution_discipline"]
    check("H21 the prereg's forbidden call list matches the runner's",
          tuple(discipline["structurally_forbidden_calls"]) == A.FORBIDDEN_CALLS)
    check("H22 the prereg's forbidden import list matches the runner's",
          tuple(discipline["structurally_forbidden_imports"]) == A.FORBIDDEN_IMPORTS)
    check("H23 the prereg's GO pin list matches the runner's",
          tuple(discipline["go_must_pin"]) == A.GO_REQUIRED_PINS)
    check("H24 the prereg's leaf name matches the runner's",
          discipline["output"].startswith(A.LEAF_ROOT + "/" + A.LEAF_NAME))
    check("H25 the prereg declares the warm-up still unauthorised",
          "own separate architect GO" in
          PREREG["what_this_stage_cannot_establish"]["the_warmup_remains_unauthorised"])
    check("H26 the prereg leaves the K1 artefacts in the not-modified list",
          any("K1 leaf" in item for item in PREREG["invariants"]["not_modified"]))
    check("H27 the module under test is the saved K1 state, not a new transplant",
          PREREG["the_module_under_test"]["sha256"]
          == A.PIN_K1_LEAF["probe_full_module_state.pkl"]
          and PREREG["the_module_under_test"]["tensors"] == 16)
    check("H28 the module is loaded with both freeze flags on",
          PREREG["the_module_under_test"]["geometry"]["freeze_actor"] is True
          and PREREG["the_module_under_test"]["geometry"]["freeze_logstd"] is True
          and PREREG["the_module_under_test"]["geometry"]["inference_only"] is False
          and PREREG["the_module_under_test"]["geometry"]["n_actor"] == 35
          and PREREG["the_module_under_test"]["geometry"]["n_full"] == 84)


def test_destination_absent() -> None:
    leaf = HERE / A.LEAF_ROOT / A.LEAF_NAME
    check("I01 no K1R1 result leaf exists", not leaf.exists(), str(leaf))
    check("I02 no staging or lock directory exists for this stage",
          not any(p.name.startswith((".staging", ".lock"))
                  for p in (HERE / A.LEAF_ROOT).iterdir()))
    check("I03 j20_runs holds only the K1 leaf",
          sorted(p.name for p in (HERE / A.LEAF_ROOT).iterdir())
          == [A.K1_LEAF_REL.split("/")[-1]],
          str(sorted(p.name for p in (HERE / A.LEAF_ROOT).iterdir())))


def main() -> int:
    test_structurally_inert()
    test_preflight_is_pure()
    test_preflight_passes()
    test_stimulus()
    test_original_evidence()
    test_composite_semantics()
    test_byte_and_immutability_checks()
    test_exact_load_and_freeze_contract()
    test_k1_immutability_is_derived_three_times()
    test_precise_language()
    test_go_validation()
    test_prereg()
    test_destination_absent()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
