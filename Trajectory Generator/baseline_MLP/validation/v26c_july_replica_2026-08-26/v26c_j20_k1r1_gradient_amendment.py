"""V26C J20-K1R1 - non-degenerate gradient amendment to the K1 probe.

The K1 probe returned FAIL_CLOSED 28/29. Its sole failure, P24, asserted that all
six unique critic tensors receive gradient; ``vf_encoder.0.weight`` did not. The
cause was the probe's own stimulus, not the pipeline: K1 fed an all-zero
observation batch, and for a linear layer y = Wx + b the first-layer weight
gradient is dL/dW = (dL/dy) x^T, which is IDENTICALLY ZERO when x = 0 whatever
the freeze does. The bias gradient is unaffected, which is exactly the pattern
K1 recorded - in BOTH arms only the first-layer WEIGHT was missing.

This amendment measures ONLY that one missing quantity, under a stimulus that is
not degenerate, and composes its verdict with the ORIGINAL evidence.

What it deliberately does NOT do:

  * it does not retry K1, does not overwrite its leaf, and does not rewrite its
    result to PASS. The K1 leaf stays FAIL_CLOSED 28/29 forever - that is what
    happened, and an amendment that edited the record would be a forgery;
  * it does not transplant an actor again and does not build a fresh critic. It
    loads the EXACT 16-tensor module state K1 committed, so it measures the same
    module K1 measured and not a look-alike;
  * it does not relax P24. The amended check is strictly stronger: finite AND
    strictly non-zero gradient norm on all six critic tensors, including the one
    that failed.

W1 and W2 are the ONLY SCIENTIFIC differences in the gradient computation:

  W1  the observation batch: a deterministic algebraic 8x84 matrix instead of
      zeros;
  W2  deterministic per-row weights on the value term, so eight rows cannot
      cancel each other symmetrically.

The EVIDENTIARY PATH differs intentionally, and that is not the same claim. K1
built a module and transplanted an actor into it; K1R1 reconstructs a module and
loads the complete post-K1 sixteen-tensor state BYTE FOR BYTE instead of
re-running the transplant. That is deliberate - re-running the transplant would
re-measure evidence K1 already produced under a GO - and it means this stage is
NOT operationally identical to K1. What it is identical in is the thing being
measured: same class, same geometry, same freeze flags, same forward path, same
single backward, same byte-identity re-verification afterwards. Every tensor the
gradient is computed from is proven byte-identical to the one K1 committed.

Usage:
    python v26c_j20_k1r1_gradient_amendment.py --preflight-only
    python v26c_j20_k1r1_gradient_amendment.py --dry-run
    python v26c_j20_k1r1_gradient_amendment.py --probe --go-file <architect GO json>

``--preflight-only`` and ``--dry-run`` write nothing, create no directory and
never import torch.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys

sys.dont_write_bytecode = True

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent

STAGE = "V26C_J20_K1R1_NONDEGENERATE_GRADIENT_AMENDMENT"
AMENDS_STAGE = "V26C_J20_CRITIC_WARMUP_READINESS"

PREREG_NAME = "v26c_j20_k1r1_prereg_nondegenerate_gradient_amendment.json"
PIN_PREREG = "d770477cf31988d0f464b157a258b42fa16a1bd8de03c90d24dc8767fb9fbac5"

# --------------------------------------------------------------------------
# The IMMUTABLE original. Read, never edited, never re-run.
# --------------------------------------------------------------------------
K1_LEAF_REL = "j20_runs/j20_critic_warmup_readiness_v26c_2026-08-27_r1"
PIN_K1_LEAF = {
    "commit_verification.json":
        "06b60b92af40b852cdbff77551421189c2235f8b64165d972f7e4784e6eeb063",
    "probe_actor_transplant_report.json":
        "73bb4c78f9e3910d552308250bf6dda29028bf6182a9760d0965db3c45ff0f80",
    "probe_full_module_state.pkl":
        "b5aef9b22a60ce664e8d5fc9415621448b24cd90fe7c2a27fc732f5b275edcd2",
    "probe_module_manifest.json":
        "eab14deedfe88449bac9ab37f5f56959d3213a3d0830ce7f22b71bb3aedfceaa",
    "v26c_j20_critic_warmup_readiness_receipt.json":
        "8d2a4ba9a8e286194eb239665b0d1d71179cf52160b4bed93a4a99cddb18d128",
    "v26c_j20_probe_result.json":
        "51b5f999f22fbd937b754159fc7a44f2fc3efa72463ea137154829b9d71d2452",
}
K1_RESULT_NAME = "v26c_j20_probe_result.json"
K1_RECEIPT_NAME = "v26c_j20_critic_warmup_readiness_receipt.json"
K1_COMMIT_NAME = "commit_verification.json"
K1_MODULE_NAME = "probe_full_module_state.pkl"

# The original runner and its GO. Not executed here; pinned so the amendment is
# anchored to the exact bytes that produced the evidence it composes with.
PIN_K1_RUNNER = "d33e0709ca4682ef857e07a85c7015d2d857aba2b04f454afb3bc1aa0c57858b"
K1_RUNNER_NAME = "v26c_j20_critic_warmup_readiness.py"
PIN_K1_PREREG = "f4ec06d77641a460e26df554f4c5c0f10ebceacb4534bbd0368773294706130c"
K1_PREREG_NAME = "v26c_j20_prereg_critic_warmup_readiness.json"
PIN_K1_GO = "aca75c7c12585b5b1ee50c52ebabf3030facf8c83e4ece668e70e421cd3f44f0"
K1_GO_NAME = "v26c_j20_architect_go_k1.json"

# The expected shape of the original verdict. The amendment refuses to run
# against anything else: composing with evidence that is not the evidence it was
# written for would be meaningless.
K1_EXPECTED_VERDICT = "FAIL_CLOSED"
K1_EXPECTED_TOTAL = 29
K1_EXPECTED_PASSED = 28
K1_SOLE_FAILED_CHECK = "P24_frozen_module_still_gives_the_critic_gradient"
K1_PASSING_CHECKS = (
    "P01_module_class_is_the_pinned_one",
    "P02_inference_only_false_builds_sixteen_keys",
    "P03_critic_keys_present",
    "P04_critic_shapes_exact",
    "P05_real_resolver_accepted_the_explicit_overlay",
    "P06_real_resolver_validated_the_actor_digest",
    "P07_target_contract_comes_from_the_live_env_and_matches_the_source",
    "P08_real_transplant_copied_the_controller_block_and_zeroed_only_the_clock",
    "P09_real_transplant_reports_the_source_is_actor_only",
    "P10_real_transplant_left_the_critic_untouched",
    "P11_critic_is_the_TARGET_init_not_imported_from_the_source",
    "P12_production_load_is_strict_and_complete",
    "P13_all_ten_actor_tensors_byte_identical",
    "P14_actor_digest_reproduces_j19a",
    "P15_warm_start_own_comparator_calls_the_actor_exact",
    "P16_encoder_aliases_still_byte_identical",
    "P17_clock_columns_are_already_exactly_zero",
    "P18_controller_memory_25_35_survives_alive",
    "P19_logstd_rows_are_byte_identical",
    "P20_logstd_is_state_independent",
    "P21_sigma_is_exactly_the_qualified_value",
    "P22_freeze_flags_live_on_the_module",
    "P23_frozen_module_gives_the_actor_no_gradient",
    "P25_control_proves_the_probe_can_see_an_unfrozen_actor",
    "P26_actor_bytes_survive_the_gradient_probe",
    "P27_full_module_hash_differs_from_j19a",
    "P28_j19a_leaf_is_byte_unchanged",
    "P29_bare_load_diagnostic_shows_the_silent_fresh_critic",
)

# --------------------------------------------------------------------------
# The actor this whole chain protects. Read only, for the final comparison.
# --------------------------------------------------------------------------
J19A_LEAF_REL = "j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1"
J19A_MODULE_REL = J19A_LEAF_REL + "/rl_module"
PIN_J19A_MODULE_STATE = \
    "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530"
PIN_J19A_ACTOR_DIGEST = \
    "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"

# The only production code this amendment executes.
BASELINE_PINS = {
    "asymmetric_rl_module.py":
        "5084786c8e6312de2d37744bf327b907ed52ff92cc6e3686b36bd1bde6d21a0f",
}

# --------------------------------------------------------------------------
# The module contract, inherited unchanged from K1.
# --------------------------------------------------------------------------
MODULE_CLASS = "AsymmetricActorCriticTorchRLModule"
ACTOR_WIDTH = 35
FULL_WIDTH = 84
ACTION_DIM = 2
HIDDENS = (256, 256)
ACTIVATION = "tanh"

ACTOR_KEYS = (
    "pi.0.0.bias", "pi.0.0.weight", "pi.0.2.bias", "pi.0.2.weight",
    "pi.1.bias", "pi.1.weight",
    "pi_encoder.0.bias", "pi_encoder.0.weight",
    "pi_encoder.2.bias", "pi_encoder.2.weight",
)
CRITIC_KEYS = (
    "vf.bias", "vf.weight",
    "vf_encoder.0.bias", "vf_encoder.0.weight",
    "vf_encoder.2.bias", "vf_encoder.2.weight",
)
# named_parameters() de-duplicates shared tensors, so pi.0.* never appears: it IS
# pi_encoder.*. These six are the actor's unique parameters, and the six critic
# keys above are already unique.
ACTOR_UNIQUE_PARAMETERS = (
    "pi_encoder.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias",
    "pi.1.weight", "pi.1.bias",
)
WARM_START_ACTOR_KEYS = (
    "pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
    "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
)
LOGSTD_ROWS = slice(2, 4)

# --------------------------------------------------------------------------
# W1 - THE STIMULUS.
#
# Fully specified here, algebraic, no RNG, no floating-point accumulation. For
# the 1-based row p and column q:
#
#     num   = (p*STIMULUS_A + q*STIMULUS_B + p*q) mod STIMULUS_MOD + 1
#     sign  = -1 if (p + q) is odd else +1
#     value = float32(sign * num / STIMULUS_DEN)
#
# num lies in 1..STIMULUS_MOD, so no element is zero, and |value| lies in
# [1/192, 191/192], strictly inside [-1, 1]. The four structural properties the
# preregistration requires - no zero, bounded, rows distinct, no constant column
# - are RE-DERIVED at run time and the stage refuses to proceed if any fails.
# --------------------------------------------------------------------------
STIMULUS_ROWS = 8
STIMULUS_COLS = FULL_WIDTH
STIMULUS_A = 37
STIMULUS_B = 53
STIMULUS_MOD = 191
STIMULUS_DEN = 192.0
PIN_STIMULUS_DIGEST = \
    "4f643e9cf7d46fb95044cf87ad09342ba56c2e22ca7cda4b72bd57e2a537fb35"
PIN_STIMULUS_MIN = -0.9947916865348816
PIN_STIMULUS_MAX = 0.9947916865348816
PIN_STIMULUS_ABS_MIN = 0.0052083334885537624

# W2 - deterministic per-row weights on the value term. Eight distinct positive
# integers, so the eight rows cannot cancel one another symmetrically in the
# first-layer weight gradient.
ROW_WEIGHTS = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)

# The amended check is STRICTLY STRONGER than K1's P24: finite AND strictly
# positive gradient norm, on all six critic tensors.
GRADIENT_NORM_MUST_BE_STRICTLY_POSITIVE = True

# --------------------------------------------------------------------------
# The only destination.
# --------------------------------------------------------------------------
LEAF_ROOT = "j20_runs"
LEAF_NAME = "j20_k1r1_nondegenerate_gradient_v26c_2026-08-27_r1"
RECEIPT_NAME = "v26c_j20_k1r1_gradient_amendment_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
INVALID_MARKER = "TECHNICAL_INVALID"

GO_REQUIRED_STAGE = STAGE
RUNNER_NAME = "v26c_j20_k1r1_gradient_amendment.py"
TEST_NAME = "test_v26c_j20_k1r1_gradient_amendment.py"
# asymmetric_rl_module.py lives OUTSIDE this directory, so its label cannot be a
# path relative to HERE. That is the point of the map below.
BASELINE_MODULE_LABEL = "baseline_MLP/asymmetric_rl_module.py"


def go_pin_targets() -> dict:
    """The CLOSED label -> path map a GO may pin.

    Paths are resolved HERE, from constants, and never taken from the GO
    payload. A GO that could name its own paths could point a pin at a file that
    is not the one this stage will actually read, which would make the pin
    decorative. The payload therefore carries labels and hashes only.

    Every input and every executable this stage semantically uses is in this
    map: its own three artefacts, all six immutable K1 leaf files, the K1
    runner, preregistration and GO, the J19A module, and the single production
    module it imports.
    """
    targets = {
        PREREG_NAME: HERE / PREREG_NAME,
        RUNNER_NAME: HERE / RUNNER_NAME,
        TEST_NAME: HERE / TEST_NAME,
        K1_RUNNER_NAME: HERE / K1_RUNNER_NAME,
        K1_PREREG_NAME: HERE / K1_PREREG_NAME,
        K1_GO_NAME: HERE / K1_GO_NAME,
        J19A_MODULE_REL + "/module_state.pkl":
            HERE / J19A_MODULE_REL / "module_state.pkl",
        BASELINE_MODULE_LABEL: BASELINE / "asymmetric_rl_module.py",
    }
    for name in PIN_K1_LEAF:
        targets[K1_LEAF_REL + "/" + name] = HERE / K1_LEAF_REL / name
    return targets


GO_REQUIRED_PINS = tuple(sorted(go_pin_targets()))

# Calls this file must never make, asserted by the test suite against this
# file's own AST. The probe must be structurally unable to train, sample, build
# an environment, start Ray or promote a checkpoint.
FORBIDDEN_CALLS = (
    "train", "build_algo", "PPOConfig", "init", "make_cmc_env", "make_env",
    "save_to_path", "restore_from_path", "from_checkpoint", "sync_weights",
    "reset", "sample", "step", "transplant_actor_state",
)
FORBIDDEN_IMPORTS = (
    "torch.optim", "env_factory", "rollout_eval", "train_ppo_mlp", "ray.tune",
)


class K1R1Error(RuntimeError):
    """The amendment refused to proceed."""


# ------------------------------------------------------------------ helpers --

def sha256_file(path: pathlib.Path) -> str:
    """SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def encode_json(payload) -> bytes:
    """Deterministic JSON bytes; non-finite values are refused."""
    return json.dumps(payload, indent=2, sort_keys=True,
                      allow_nan=False).encode("utf-8")


def tensor_digest(array) -> str:
    """The project's canonical tensor digest: dtype, shape, C-order bytes."""
    import numpy as np

    contiguous = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(str(contiguous.dtype).encode("ascii"))
    digest.update(repr(tuple(int(d) for d in contiguous.shape)).encode("ascii"))
    digest.update(contiguous.tobytes(order="C"))
    return digest.hexdigest()


def actor_digest(state) -> str:
    """warm_start.actor_state_digest, transcribed. Must reproduce d4a13ff7..."""
    digest = hashlib.sha256()
    for key in sorted(WARM_START_ACTOR_KEYS):
        if key not in state:
            raise K1R1Error("actor state is missing %s" % key)
        digest.update(key.encode("utf-8"))
        digest.update(tensor_digest(state[key]).encode("ascii"))
    return digest.hexdigest()


def bytes_identical(left, right) -> bool:
    """Same dtype, same shape, same C-order bytes. NOT numpy.array_equal."""
    import numpy as np

    a = np.ascontiguousarray(left)
    b = np.ascontiguousarray(right)
    return bool(a.dtype == b.dtype and a.shape == b.shape
                and a.tobytes(order="C") == b.tobytes(order="C"))


# ------------------------------------------------------------- the stimulus --

def stimulus_matrix(numpy_module):
    """W1. The deterministic non-degenerate observation batch.

    Pure integer arithmetic then one exact division, so the matrix is a property
    of this source file and of nothing else - no RNG, no seed, no platform float
    accumulation. It is re-derived at run time and its digest is compared to the
    pinned one, so a silent change to the construction cannot pass unnoticed.
    """
    matrix = numpy_module.empty((STIMULUS_ROWS, STIMULUS_COLS),
                                dtype=numpy_module.float32)
    for row in range(STIMULUS_ROWS):
        p = row + 1
        for column in range(STIMULUS_COLS):
            q = column + 1
            magnitude = (p * STIMULUS_A + q * STIMULUS_B + p * q) % STIMULUS_MOD + 1
            sign = -1.0 if ((p + q) % 2) else 1.0
            matrix[row, column] = numpy_module.float32(
                sign * magnitude / STIMULUS_DEN)
    return matrix


def stimulus_report(numpy_module) -> dict:
    """Derive the stimulus and MEASURE the properties the prereg requires.

    The properties are re-derived, never assumed. A construction that stopped
    satisfying one of them would be caught here rather than silently producing a
    weaker experiment than the one that was preregistered.
    """
    matrix = stimulus_matrix(numpy_module)
    rows = {matrix[i].tobytes() for i in range(matrix.shape[0])}
    per_column_unique = [int(len(numpy_module.unique(matrix[:, j])))
                         for j in range(matrix.shape[1])]
    digest = hashlib.sha256(
        numpy_module.ascontiguousarray(matrix).tobytes(order="C")).hexdigest()
    observed_min = float(matrix.min())
    observed_max = float(matrix.max())
    observed_abs_min = float(numpy_module.abs(matrix).min())
    checks = {
        "shape_is_8x84": matrix.shape == (STIMULUS_ROWS, STIMULUS_COLS),
        "dtype_is_float32": str(matrix.dtype) == "float32",
        "every_element_is_nonzero": bool(numpy_module.all(matrix != 0.0)),
        "every_element_is_within_minus_one_and_one":
            bool(numpy_module.all(numpy_module.abs(matrix) <= 1.0)),
        "all_eight_rows_are_distinct": len(rows) == matrix.shape[0],
        "no_column_is_constant": min(per_column_unique) >= 2,
        "matches_the_pinned_digest": digest == PIN_STIMULUS_DIGEST,
        "is_not_the_degenerate_zero_batch":
            bool(numpy_module.any(matrix != 0.0)),
        # The pinned range is BINDING here, not merely asserted by an external
        # test. A construction that drifted while still hashing differently
        # would already fail the digest; a construction whose range moved while
        # the digest was re-pinned would not, and these three close that door.
        "matches_the_pinned_minimum": observed_min == PIN_STIMULUS_MIN,
        "matches_the_pinned_maximum": observed_max == PIN_STIMULUS_MAX,
        "matches_the_pinned_absolute_minimum":
            observed_abs_min == PIN_STIMULUS_ABS_MIN,
    }
    return {
        "matrix": matrix,
        "ok": all(checks.values()),
        "checks": checks,
        "digest_float32_c_order": digest,
        "min": observed_min,
        "max": observed_max,
        "abs_min": observed_abs_min,
        "pinned_min": PIN_STIMULUS_MIN,
        "pinned_max": PIN_STIMULUS_MAX,
        "pinned_abs_min": PIN_STIMULUS_ABS_MIN,
        "min_distinct_values_per_column": int(min(per_column_unique)),
        "positive_elements": int((matrix > 0.0).sum()),
        "negative_elements": int((matrix < 0.0).sum()),
        "construction": (
            "value[p-1][q-1] = sign * ((p*%d + q*%d + p*q) mod %d + 1) / %s, "
            "with p and q 1-based and sign = -1 when (p+q) is odd"
            % (STIMULUS_A, STIMULUS_B, STIMULUS_MOD, STIMULUS_DEN)),
    }


# ------------------------------------------------------- the original record --

def verify_original() -> dict:
    """The K1 evidence must be EXACTLY what this amendment was written for.

    Not merely present: FAIL_CLOSED, 28 of 29, the sole failure P24, the other
    twenty-eight all true, and a commit verification that passed. Composing with
    anything else would be composing with a different experiment.
    """
    leaf = HERE / K1_LEAF_REL
    problems = []
    hashes = {}
    for name, expected in sorted(PIN_K1_LEAF.items()):
        path = leaf / name
        if not path.is_file():
            problems.append("the K1 leaf is missing %s" % name)
            continue
        actual = sha256_file(path)
        hashes[name] = actual
        if actual != expected:
            problems.append("K1 leaf file %s hashes %s, expected %s"
                            % (name, actual, expected))
    if problems:
        return {"ok": False, "problems": problems, "hashes": hashes}

    result = json.loads((leaf / K1_RESULT_NAME).read_text(encoding="utf-8"))
    receipt = json.loads((leaf / K1_RECEIPT_NAME).read_text(encoding="utf-8"))
    commit = json.loads((leaf / K1_COMMIT_NAME).read_text(encoding="utf-8"))

    checks = result.get("checks") or {}
    failed = sorted(name for name, value in checks.items() if not value)
    passed = sorted(name for name, value in checks.items() if value)

    if result.get("verdict") != K1_EXPECTED_VERDICT:
        problems.append("the K1 verdict is %r, expected %r"
                        % (result.get("verdict"), K1_EXPECTED_VERDICT))
    if int(result.get("checks_total", -1)) != K1_EXPECTED_TOTAL:
        problems.append("K1 recorded %r checks, expected %d"
                        % (result.get("checks_total"), K1_EXPECTED_TOTAL))
    if int(result.get("checks_passed", -1)) != K1_EXPECTED_PASSED:
        problems.append("K1 passed %r checks, expected %d"
                        % (result.get("checks_passed"), K1_EXPECTED_PASSED))
    if failed != [K1_SOLE_FAILED_CHECK]:
        problems.append("K1's failed checks are %s, expected exactly [%s]"
                        % (failed, K1_SOLE_FAILED_CHECK))
    if tuple(passed) != tuple(sorted(K1_PASSING_CHECKS)):
        problems.append("K1's passing checks are not the twenty-eight this "
                        "amendment composes with")
    if commit.get("ok") is not True or commit.get("problems"):
        problems.append("the K1 commit verification did not pass")
    if receipt.get("verdict") != K1_EXPECTED_VERDICT:
        problems.append("the K1 receipt verdict is %r" % receipt.get("verdict"))
    if sorted(receipt.get("failed_checks") or []) != [K1_SOLE_FAILED_CHECK]:
        problems.append("the K1 receipt names different failed checks")
    if (leaf / INVALID_MARKER).exists():
        problems.append("the K1 leaf still carries its TECHNICAL_INVALID marker")
    inert = result.get("inert") or {}
    for flag in ("algo_train_called", "optimizer_constructed",
                 "optimizer_step_taken", "ray_cluster_started",
                 "environment_constructed", "environment_stepped",
                 "rollout_performed", "warmup_executed"):
        if inert.get(flag) is not False:
            problems.append("K1's inert flag %s is not false" % flag)

    return {
        "ok": not problems,
        "problems": problems,
        "hashes": hashes,
        "verdict": result.get("verdict"),
        "checks_total": result.get("checks_total"),
        "checks_passed": result.get("checks_passed"),
        "failed_checks": failed,
        "passing_checks": passed,
        "commit_verification_ok": bool(commit.get("ok")),
        "k1_degenerate_stimulus_recorded": True,
    }


# ------------------------------------------------------------------ preflight --

def check_pins() -> list[dict]:
    """Every pinned artefact, re-hashed. Pure: reads files, writes nothing."""
    results = []

    def record(label: str, path: pathlib.Path, expected: str | None) -> None:
        if not path.is_file():
            results.append({"artefact": label, "ok": False, "why": "missing",
                            "expected": expected, "actual": None})
            return
        actual = sha256_file(path)
        results.append({
            "artefact": label,
            "ok": expected is None or actual == expected,
            "why": "present" if expected is None else
                   ("matches" if actual == expected else "HASH MISMATCH"),
            "expected": expected,
            "actual": actual,
        })

    record(PREREG_NAME, HERE / PREREG_NAME, PIN_PREREG)
    for name, expected in sorted(PIN_K1_LEAF.items()):
        record(K1_LEAF_REL + "/" + name, HERE / K1_LEAF_REL / name, expected)
    record(K1_RUNNER_NAME, HERE / K1_RUNNER_NAME, PIN_K1_RUNNER)
    record(K1_PREREG_NAME, HERE / K1_PREREG_NAME, PIN_K1_PREREG)
    record(K1_GO_NAME, HERE / K1_GO_NAME, PIN_K1_GO)
    record(J19A_MODULE_REL + "/module_state.pkl",
           HERE / J19A_MODULE_REL / "module_state.pkl", PIN_J19A_MODULE_STATE)
    for name, expected in sorted(BASELINE_PINS.items()):
        record("baseline_MLP/" + name, BASELINE / name, expected)
    return results


def check_no_leaf() -> dict:
    """The destination must not exist yet, and K1's must be untouched."""
    leaf = HERE / LEAF_ROOT / LEAF_NAME
    return {"ok": not leaf.exists(),
            "leaf": LEAF_ROOT + "/" + LEAF_NAME,
            "leaf_exists": leaf.exists(),
            "k1_leaf_present": (HERE / K1_LEAF_REL).is_dir(),
            "j20_runs_contents": sorted(
                p.name for p in (HERE / LEAF_ROOT).iterdir())
            if (HERE / LEAF_ROOT).is_dir() else []}


def preflight(verbose: bool = True) -> dict:
    """Fail-closed readiness check. No torch, no ray, no env, no write."""
    import numpy

    pins = check_pins()
    original = verify_original()
    stimulus = stimulus_report(numpy)
    destination = check_no_leaf()

    problems = [entry["artefact"] + ": " + entry["why"]
                for entry in pins if not entry["ok"]]
    problems += original["problems"]
    if not stimulus["ok"]:
        problems += ["stimulus property failed: %s" % name
                     for name, value in stimulus["checks"].items() if not value]
    if not destination["ok"]:
        problems.append("the destination leaf already exists: " + destination["leaf"])

    report = {
        "stage": STAGE,
        "amends": AMENDS_STAGE,
        "ok": not problems,
        "problems": problems,
        "pins": pins,
        "pins_checked": len(pins),
        "pins_matching": sum(1 for entry in pins if entry["ok"]),
        "original": {key: value for key, value in original.items()
                     if key != "hashes"},
        "stimulus": {key: value for key, value in stimulus.items()
                     if key != "matrix"},
        "destination": destination,
        "it_does_not_retry_k1": True,
        "it_does_not_rewrite_the_k1_result": True,
    }
    if verbose:
        print("stage              %s" % STAGE)
        print("amends             %s" % AMENDS_STAGE)
        print("pins               %d/%d match"
              % (report["pins_matching"], report["pins_checked"]))
        print("original K1        %s %s/%s, sole failure %s"
              % (original.get("verdict"), original.get("checks_passed"),
                 original.get("checks_total"),
                 (original.get("failed_checks") or ["?"])[0]))
        print("K1 commit verified %s" % original.get("commit_verification_ok"))
        print("stimulus           %s (%d/%d properties)"
              % ("OK" if stimulus["ok"] else "FAILED",
                 sum(1 for v in stimulus["checks"].values() if v),
                 len(stimulus["checks"])))
        print("stimulus digest    %s" % stimulus["digest_float32_c_order"])
        print("stimulus range     [%.16f, %.16f], |min| %.16f"
              % (stimulus["min"], stimulus["max"], stimulus["abs_min"]))
        print("destination        %s (%s)"
              % (destination["leaf"], "absent" if destination["ok"] else "PRESENT"))
        print("verdict            %s" % ("READY" if report["ok"] else "BLOCKED"))
        for problem in problems:
            print("  problem: %s" % problem, file=sys.stderr)
    return report


# ---------------------------------------------------------------------- the GO --

def validate_go(payload, root: pathlib.Path = HERE) -> dict:
    """Validate an architect GO. It authorises this amendment and nothing else."""
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"],
                "pins": {}}
    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append("stage is %r, expected %r"
                        % (payload.get("stage"), GO_REQUIRED_STAGE))
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")
    for forbidden in ("authorises_warmup", "authorises_training",
                      "authorises_rollout", "authorises_ppo",
                      "authorises_k1_retry", "authorises_rewriting_k1"):
        if payload.get(forbidden) is True:
            problems.append("a K1R1 GO must never set %s" % forbidden)
    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict):
        problems.append("pinned_artefacts_sha256 is missing or is not an object")
        pins = {}
    targets = go_pin_targets()
    for required in sorted(targets):
        if required not in pins:
            problems.append("no pin for %s" % required)
    for label in sorted(pins):
        if label not in targets:
            problems.append("pin for %s is outside the authorised scope" % label)
            continue
        # The path comes from the closed map, never from the payload.
        path = targets[label]
        if not path.is_file():
            problems.append("pinned artefact %s does not exist at %s"
                            % (label, path))
            continue
        actual = sha256_file(path)
        if actual != pins[label]:
            problems.append("pinned hash for %s is stale: GO says %s, the file is %s"
                            % (label, pins[label], actual))
    return {"valid": not problems, "problems": problems, "pins": dict(pins),
            "pin_labels_required": len(targets),
            "pin_labels_supplied": len(pins)}


def load_go(go_file: str) -> dict:
    """Read and validate a GO file."""
    path = pathlib.Path(go_file)
    if not path.exists():
        return {"valid": False, "pins": {},
                "problems": ["the GO file %s does not exist" % go_file]}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError) as error:
        return {"valid": False, "pins": {},
                "problems": ["the GO file is unreadable: %s" % error]}
    return validate_go(payload)


# ------------------------------------------------------------ the measurement --

def build_module(torch_module, gym_module, module_cls):
    """The SAME class at the SAME geometry, with BOTH freeze flags on.

    Its freshly-initialised weights are irrelevant: every one of the sixteen
    tensors is overwritten by the state K1 committed. The construction exists
    only to obtain a module object of the right class and shape.
    """
    import numpy as np

    return module_cls(
        observation_space=gym_module.spaces.Box(
            -np.inf, np.inf, (FULL_WIDTH,), np.float32),
        action_space=gym_module.spaces.Box(-1.0, 1.0, (ACTION_DIM,), np.float32),
        inference_only=False,
        learner_only=False,
        catalog_class=None,
        model_config={
            "vf_share_layers": False,
            "n_actor": ACTOR_WIDTH,
            "n_full": FULL_WIDTH,
            "fcnet_hiddens": list(HIDDENS),
            "fcnet_activation": ACTIVATION,
            "freeze_logstd": True,
            "freeze_actor": True,
        },
    )


def measure_gradients(torch_module, numpy_module, module, stimulus) -> dict:
    """ONE backward over the non-degenerate stimulus. No optimizer, no step.

    The forward path is K1's: the policy logits from _forward_train plus the
    value head. W2 weights the value term per row so eight rows cannot cancel
    symmetrically in the first-layer weight gradient - the very quantity K1
    could not observe.
    """
    from ray.rllib.core.columns import Columns

    batch = {Columns.OBS: torch_module.as_tensor(stimulus)}
    weights = torch_module.as_tensor(
        numpy_module.asarray(ROW_WEIGHTS, dtype=numpy_module.float32))
    for parameter in module.parameters():
        parameter.grad = None

    outputs = module._forward_train(batch)
    logits = outputs[Columns.ACTION_DIST_INPUTS]
    values = module.compute_values(batch)
    loss = logits.sum() + (weights * values).sum()
    loss.backward()

    norms = {}
    for name, parameter in module.named_parameters():
        grad = parameter.grad
        if grad is None:
            norms[name] = None
            continue
        norms[name] = float(grad.detach().norm().item())
    return {
        "gradient_norms": norms,
        "unique_parameters": sorted(norms),
        "loss_is_finite": bool(numpy_module.isfinite(float(loss.item()))),
        "no_optimizer_was_constructed": True,
        "no_optimizer_step_was_taken": True,
        "forward_path": "_forward_train logits plus compute_values, exactly K1's",
    }


def run_probe(go_file: str) -> dict:
    """K1R1. Load K1's exact module, measure once, compose, commit."""
    go = load_go(go_file)
    if not go["valid"]:
        raise K1R1Error("refusing to run the amendment: the architect GO is "
                        "absent or invalid. %s" % "; ".join(go["problems"]))
    report = preflight(verbose=False)
    if not report["ok"]:
        raise K1R1Error("refusing to run the amendment: preflight did not pass: "
                        "%s" % "; ".join(report["problems"]))

    import pickle

    import numpy as np

    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import gymnasium
    import torch
    from asymmetric_rl_module import AsymmetricActorCriticTorchRLModule as module_cls

    if module_cls.__name__ != MODULE_CLASS:
        raise K1R1Error("the imported class is %s, expected %s"
                        % (module_cls.__name__, MODULE_CLASS))

    original = verify_original()
    stimulus = stimulus_report(np)
    matrix = stimulus["matrix"]

    # THE EXACT MODULE K1 MEASURED. Not a re-transplant, not a fresh critic.
    saved_path = HERE / K1_LEAF_REL / K1_MODULE_NAME
    saved = {k: np.asarray(v)
             for k, v in pickle.loads(saved_path.read_bytes()).items()}
    module = build_module(torch, gymnasium, module_cls)
    missing, unexpected = module.load_state_dict(
        {k: torch.as_tensor(v) for k, v in saved.items()}, strict=True)

    before = {k: np.array(v.detach().cpu().numpy(), copy=True)
              for k, v in module.state_dict().items()}

    # strict=True proves no key was missing or unexpected. It does NOT prove the
    # VALUES that landed are the ones on disk: a silent dtype promotion or a
    # copy that lost a signed zero would pass a strict load. So every one of the
    # sixteen tensors is compared, key set first, then dtype, shape and C-order
    # bytes.
    loaded_vs_saved = {key: bytes_identical(before[key], saved[key])
                       for key in sorted(set(before) & set(saved))}
    load_exactness = {
        "same_key_set": sorted(before) == sorted(saved),
        "keys_live": sorted(before),
        "keys_saved": sorted(saved),
        "tensor_count_live": len(before),
        "tensor_count_saved": len(saved),
        "per_key_byte_identical": loaded_vs_saved,
        "identical_count": sum(1 for v in loaded_vs_saved.values() if v),
        "all_identical": bool(loaded_vs_saved) and all(loaded_vs_saved.values()),
        "missing_keys": sorted(str(k) for k in missing),
        "unexpected_keys": sorted(str(k) for k in unexpected),
        "method": "same dtype, same shape, same C-order bytes, per key",
    }

    # The freeze contract must be LIVE on the object that will be differentiated,
    # not merely requested in the constructor call.
    freeze_contract = {
        "_freeze_actor": getattr(module, "_freeze_actor", None),
        "_freeze_logstd": getattr(module, "_freeze_logstd", None),
        "inference_only": getattr(module, "inference_only", None),
        "_n_actor": getattr(module, "_n_actor", None),
        "_n_full": getattr(module, "_n_full", None),
        "_action_dim": getattr(module, "_action_dim", None),
        "has_vf": hasattr(module, "vf"),
        "has_vf_encoder": hasattr(module, "vf_encoder"),
    }
    freeze_contract_ok = bool(
        freeze_contract["_freeze_actor"] is True
        and freeze_contract["_freeze_logstd"] is True
        and freeze_contract["inference_only"] is False
        and freeze_contract["_n_actor"] == ACTOR_WIDTH
        and freeze_contract["_n_full"] == FULL_WIDTH
        and freeze_contract["_action_dim"] == ACTION_DIM
        and freeze_contract["has_vf"] and freeze_contract["has_vf_encoder"])
    freeze_contract["contract_holds"] = freeze_contract_ok
    measurement = measure_gradients(torch, np, module, matrix)
    after = {k: np.array(v.detach().cpu().numpy(), copy=True)
             for k, v in module.state_dict().items()}

    # The K1 record is re-verified AFTER the backward and BEFORE anything is
    # written. Verifying it only once, at the start, would leave the interval in
    # which this stage actually runs unobserved - and that interval is precisely
    # where an accident could touch the evidence being composed with.
    original_after = verify_original()
    k1_unchanged_across_measurement = bool(
        original_after["ok"]
        and original_after["hashes"] == original["hashes"]
        and original_after["hashes"] == dict(PIN_K1_LEAF))

    norms = measurement["gradient_norms"]
    critic_norms = {key: norms.get(key) for key in CRITIC_KEYS}
    actor_norms = {key: norms.get(key) for key in ACTOR_UNIQUE_PARAMETERS}
    critic_all_positive = all(
        value is not None and np.isfinite(value) and value > 0.0
        for value in critic_norms.values())
    actor_all_absent = all(
        value is None or value == 0.0 for value in actor_norms.values())

    unchanged = {key: bytes_identical(before[key], after[key]) for key in before}
    on_disk = {k: np.asarray(v) for k, v in pickle.loads(
        (HERE / J19A_MODULE_REL / "module_state.pkl").read_bytes()).items()}
    actor_identical = {key: bytes_identical(after[key], on_disk[key])
                       for key in ACTOR_KEYS}
    digest_after = actor_digest(after)

    head_weight = np.asarray(after["pi.1.weight"])
    head_bias = np.asarray(after["pi.1.bias"])
    logstd_unchanged = bool(
        bytes_identical(head_weight[LOGSTD_ROWS],
                        np.asarray(on_disk["pi.1.weight"])[LOGSTD_ROWS])
        and bytes_identical(head_bias[LOGSTD_ROWS],
                            np.asarray(on_disk["pi.1.bias"])[LOGSTD_ROWS]))

    amended = {
        "A01_original_k1_is_exactly_fail_closed_28_of_29_before_AND_after":
            bool(original["ok"] and original_after["ok"]
                 and k1_unchanged_across_measurement),
        "A02_stimulus_satisfies_every_preregistered_property": bool(stimulus["ok"]),
        "A03_stimulus_matches_the_pinned_digest":
            stimulus["digest_float32_c_order"] == PIN_STIMULUS_DIGEST,
        "A04_module_loaded_exactly_from_the_k1_leaf_with_the_freeze_contract_live":
            tuple(missing) == () and tuple(unexpected) == ()
            and len(saved) == 16 and load_exactness["tensor_count_live"] == 16
            and load_exactness["same_key_set"]
            and load_exactness["all_identical"]
            and load_exactness["identical_count"] == 16
            and freeze_contract_ok,
        "A05_loss_is_finite": bool(measurement["loss_is_finite"]),
        "A06_all_six_critic_tensors_have_strictly_positive_gradient_norm":
            bool(critic_all_positive),
        "A07_vf_encoder_0_weight_specifically_has_gradient":
            bool(critic_norms.get("vf_encoder.0.weight") is not None
                 and np.isfinite(critic_norms["vf_encoder.0.weight"])
                 and critic_norms["vf_encoder.0.weight"] > 0.0),
        "A08_no_actor_parameter_received_gradient": bool(actor_all_absent),
        "A09_logstd_rows_unchanged": logstd_unchanged,
        "A10_all_sixteen_tensors_byte_identical_before_and_after":
            all(unchanged.values()),
        "A11_ten_actor_tensors_still_byte_identical_to_j19a":
            all(actor_identical.values()),
        "A12_actor_digest_still_reproduces_j19a":
            digest_after == PIN_J19A_ACTOR_DIGEST,
    }

    # THE COMPOSITE. The original 28 are read from the immutable record and are
    # NOT re-measured; the amended P24 is measured here. The K1 result stays
    # FAIL_CLOSED on disk and this composition never edits it.
    amended_p24 = bool(amended["A06_all_six_critic_tensors_have_strictly_positive_gradient_norm"]
                       and amended["A07_vf_encoder_0_weight_specifically_has_gradient"]
                       and amended["A08_no_actor_parameter_received_gradient"])
    composite_ok = bool(original["ok"] and original_after["ok"]
                        and k1_unchanged_across_measurement
                        and all(amended.values()) and amended_p24)
    composite = {
        "rule": "PASS 29/29 if and only if the immutable K1 record is exactly "
                "FAIL_CLOSED 28/29 with P24 as its sole failure, its other "
                "twenty-eight checks are intact, and the amended P24 passes here",
        "original_passing_checks": len(original["passing_checks"]),
        "original_sole_failure": K1_SOLE_FAILED_CHECK,
        "amended_p24_passes": amended_p24,
        "composite_passed": 28 + (1 if amended_p24 else 0),
        "composite_total": K1_EXPECTED_TOTAL,
        "verdict": "PASS" if composite_ok else "FAIL_CLOSED",
        "the_k1_result_is_NOT_rewritten": True,
        "the_k1_leaf_still_reads_fail_closed": True,
        "why_composition_and_not_a_retry": "K1's twenty-eight passing checks are "
            "evidence that was produced once, under a GO, and re-running them "
            "would replace evidence rather than add to it. The amendment "
            "measures ONLY the quantity K1 could not observe.",
    }

    result = {
        "kind": "K1R1 AMENDMENT RESULT",
        "stage": STAGE,
        "amends": AMENDS_STAGE,
        "verdict": composite["verdict"],
        "amended_checks": amended,
        "amended_checks_passed": sum(1 for v in amended.values() if v),
        "amended_checks_total": len(amended),
        "composite": composite,
        "authorised_differences_from_k1": {
            "W1_stimulus": {
                "k1": "an all-zero 8x84 batch",
                "k1r1": stimulus["construction"],
                "why": "for y = Wx + b the first-layer weight gradient is "
                       "dL/dW = (dL/dy) x^T, identically zero when x = 0, "
                       "whatever the freeze does. K1 could not observe "
                       "vf_encoder.0.weight for that reason and no other.",
                "digest_float32_c_order": stimulus["digest_float32_c_order"],
                "range": [stimulus["min"], stimulus["max"]],
                "abs_min": stimulus["abs_min"],
                "properties": stimulus["checks"],
                "positive_elements": stimulus["positive_elements"],
                "negative_elements": stimulus["negative_elements"],
            },
            "W2_row_weights": {
                "k1": "the value term summed with unit weights",
                "k1r1": list(ROW_WEIGHTS),
                "why": "eight distinct positive weights, so the rows cannot "
                       "cancel one another symmetrically in the first-layer "
                       "weight gradient",
            },
            "these_are_the_only_SCIENTIFIC_differences": "W1 and W2 are the "
                "only differences in the gradient computation itself: same "
                "class, same geometry, same freeze flags, same forward path, "
                "same single backward, same byte re-verification.",
            "the_EVIDENTIARY_PATH_differs_intentionally": "K1 built a module and "
                "transplanted an actor into it. K1R1 reconstructs a module and "
                "loads the complete post-K1 sixteen-tensor state byte for byte "
                "instead of re-running the transplant, because re-running it "
                "would re-measure evidence K1 already produced under a GO. This "
                "stage is therefore NOT operationally identical to K1; it is "
                "identical in what is measured, and every tensor the gradient is "
                "computed from is proven byte-identical to the one K1 committed.",
        },
        "gradient_norms": {
            "critic": critic_norms,
            "actor_unique_parameters": actor_norms,
            "all": norms,
        },
        "load_exactness": load_exactness,
        "freeze_contract": freeze_contract,
        "byte_identity": {
            "all_sixteen_before_vs_after": unchanged,
            "ten_actor_tensors_vs_j19a": actor_identical,
            "actor_digest_after": digest_after,
            "actor_digest_expected": PIN_J19A_ACTOR_DIGEST,
            "method": "same dtype, same shape, same C-order bytes. Deliberately "
                      "NOT numpy.array_equal, which is numeric.",
        },
        "original_k1_record": {
            "leaf": K1_LEAF_REL,
            "verdict": original["verdict"],
            "checks_passed": original["checks_passed"],
            "checks_total": original["checks_total"],
            "failed_checks": original["failed_checks"],
            "commit_verification_ok": original["commit_verification_ok"],
            "hashes_before_measurement": original["hashes"],
            "hashes_after_measurement": original_after["hashes"],
            "verified_ok_before_measurement": bool(original["ok"]),
            "verified_ok_after_measurement": bool(original_after["ok"]),
            "unchanged_across_measurement": k1_unchanged_across_measurement,
            "unchanged_is_DERIVED_not_declared":
                "this flag is the conjunction of two independent re-hashes of "
                "all six K1 files, before and after the backward, both compared "
                "to the pinned values. A third re-hash runs after the commit and "
                "is recorded in commit_verification.json.",
            "post_commit_re_hash": "recorded in commit_verification.json",
            "it_is_read_never_edited_never_re_run": True,
        },
        "inert": {
            "k1_retried": False,
            "k1_result_rewritten": False,
            "algo_train_called": False,
            "optimizer_constructed": False,
            "optimizer_step_taken": False,
            "ray_cluster_started": False,
            "environment_constructed": False,
            "environment_stepped": False,
            "rollout_performed": False,
            "actor_edited": False,
            "actor_transplanted": False,
            "critic_replaced": False,
            "warmup_executed": False,
            "checkpoint_promoted": False,
            "ppo_updates": 0,
        },
        "promotion": "NONE",
        "next_stage_authorized": False,
    }

    import io

    buffer = io.BytesIO()
    np.save(buffer, np.ascontiguousarray(matrix), allow_pickle=False)

    files = {}
    files["v26c_j20_k1r1_result.json"] = encode_json(result)
    files["k1r1_stimulus.npy"] = buffer.getvalue()
    files["k1r1_stimulus_manifest.json"] = encode_json({
        "what_this_is": "the exact float32 8x84 stimulus this amendment used, "
                        "saved so the measurement can be reproduced from the "
                        "leaf without re-deriving it from source",
        "construction": stimulus["construction"],
        "digest_float32_c_order": stimulus["digest_float32_c_order"],
        "shape": [STIMULUS_ROWS, STIMULUS_COLS],
        "dtype": "float32",
        "min": stimulus["min"],
        "max": stimulus["max"],
        "abs_min": stimulus["abs_min"],
        "row_weights": list(ROW_WEIGHTS),
        "properties": stimulus["checks"],
        "stage": STAGE,
    })

    receipt = {
        "kind": "EXECUTION RECEIPT",
        "stage": STAGE,
        "amends": AMENDS_STAGE,
        "verdict": result["verdict"],
        "amended_checks_passed": result["amended_checks_passed"],
        "amended_checks_total": result["amended_checks_total"],
        "failed_amended_checks": sorted(k for k, v in amended.items() if not v),
        "composite_passed": composite["composite_passed"],
        "composite_total": composite["composite_total"],
        "inputs": {
            "prereg_sha256": sha256_file(HERE / PREREG_NAME),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "k1_leaf_sha256": dict(sorted(PIN_K1_LEAF.items())),
            "k1_runner_sha256": PIN_K1_RUNNER,
            "k1_go_sha256": PIN_K1_GO,
            "j19a_module_state_sha256": PIN_J19A_MODULE_STATE,
            "j19a_actor_digest": PIN_J19A_ACTOR_DIGEST,
            "baseline_pins_sha256": dict(sorted(BASELINE_PINS.items())),
            "stimulus_digest": stimulus["digest_float32_c_order"],
            "architect_go_pins": go["pins"],
        },
        "k1_leaf_unchanged_across_measurement": k1_unchanged_across_measurement,
        "k1_leaf_verified_before_and_after_the_backward": bool(
            original["ok"] and original_after["ok"]),
        "k1_leaf_post_commit_re_hash": "recorded in commit_verification.json",
        "k1_result_still_reads_fail_closed":
            original_after.get("verdict") == K1_EXPECTED_VERDICT,
        "inert": result["inert"],
        "promotion": "NONE",
        "next_stage_authorized": False,
        "artefacts_sha256": {name: hashlib.sha256(payload).hexdigest()
                             for name, payload in sorted(files.items())},
    }
    files[RECEIPT_NAME] = encode_json(receipt)
    committed = commit_leaf(files)
    return {"leaf": committed["leaf"], "verdict": receipt["verdict"],
            "verification": committed["verification"],
            "composite": "%d/%d" % (composite["composite_passed"],
                                    composite["composite_total"])}


# ----------------------------------------------------------------- the commit --

def commit_leaf(files: dict) -> dict:
    """Write the leaf atomically, born invalid, verified after commit."""
    import os

    leaf_root = HERE / LEAF_ROOT
    final = leaf_root / LEAF_NAME
    if final.exists():
        raise K1R1Error("refusing to clobber an existing leaf: %s" % final)

    digest = hashlib.sha256()
    for name in sorted(files):
        digest.update(name.encode("utf-8"))
        digest.update(hashlib.sha256(files[name]).hexdigest().encode("ascii"))
    aggregate = digest.hexdigest()

    staging = leaf_root / (".staging_%s" % aggregate[:16])
    if staging.exists():
        raise K1R1Error("refusing to reuse an existing staging directory: %s"
                        % staging)
    staging.mkdir(parents=True)
    (staging / INVALID_MARKER).write_bytes(
        b"born invalid; removed only after post-commit verification succeeds\n")
    for name in sorted(files):
        target = staging / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(files[name])
    os.rename(str(staging), str(final))

    problems = []
    committed = json.loads((final / RECEIPT_NAME).read_text(encoding="utf-8"))
    for name, expected in sorted(committed["artefacts_sha256"].items()):
        path = final / name
        if not path.exists():
            problems.append("committed receipt names a missing file: %s" % name)
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append("committed %s hashes %s, receipt says %s"
                            % (name, actual, expected))
        if name in files and hashlib.sha256(files[name]).hexdigest() != actual:
            problems.append("committed %s differs from the staged bytes" % name)

    # The K1 leaf is re-hashed a THIRD time, after the commit. If this stage had
    # touched the evidence it composes with - at any point, including while
    # writing its own leaf - the marker below stays in place and the leaf is
    # born invalid forever.
    k1_post_commit = {}
    for name, expected in sorted(PIN_K1_LEAF.items()):
        path = HERE / K1_LEAF_REL / name
        actual = sha256_file(path) if path.is_file() else None
        k1_post_commit[name] = actual
        if actual != expected:
            problems.append("the K1 leaf file %s changed: expected %s, found %s"
                            % (name, expected, actual))
    k1_unchanged_post_commit = k1_post_commit == dict(PIN_K1_LEAF)

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(final.relative_to(HERE)),
        "aggregate_digest": aggregate,
        "files_verified": len(committed["artefacts_sha256"]),
        "k1_leaf_re_hashed_post_commit": k1_post_commit,
        "k1_leaf_unchanged_post_commit": k1_unchanged_post_commit,
        "problems": problems,
        "ok": not problems,
        "method": "every leaf-relative path is re-resolved from the COMMITTED "
                  "receipt, re-hashed, and compared both to the receipt and to "
                  "the staged bytes; and all six K1 leaf files are re-hashed "
                  "against their pinned values a third time",
    }
    (final / COMMIT_VERIFICATION_NAME).write_bytes(encode_json(verification))
    if problems:
        raise K1R1Error("post-commit verification FAILED; the invalid marker is "
                        "left in place: %s" % problems)
    (final / INVALID_MARKER).unlink()
    return {"leaf": str(final.relative_to(HERE)), "verification": verification}


# ------------------------------------------------------------------------ CLI --

def build_parser() -> argparse.ArgumentParser:
    """Command-line surface. Preflight is the default and writes nothing."""
    parser = argparse.ArgumentParser(
        description="V26C J20-K1R1 - non-degenerate gradient amendment")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--probe", action="store_true")
    parser.add_argument("--go-file", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Return 0 on success, 1 on failure. Writes nothing unless --probe."""
    args = build_parser().parse_args(argv)
    if args.probe:
        if not args.go_file:
            print("--probe requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_probe(args.go_file)
        except (K1R1Error, RuntimeError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"],
                          "composite": outcome["composite"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = preflight(verbose=True)
    if args.dry_run:
        print("\nplan")
        print("  load the EXACT sixteen-tensor module state K1 committed, push")
        print("  ONE backward over the non-degenerate stimulus, and require all")
        print("  six critic tensors - vf_encoder.0.weight included - to carry a")
        print("  finite, strictly positive gradient norm while no actor")
        print("  parameter carries any.")
        print("\n  K1 is NOT retried. Its leaf stays FAIL_CLOSED 28/29 and this")
        print("  stage never edits it. The composite reads 29/29 only if the")
        print("  immutable twenty-eight are intact AND the amended P24 passes.")
        print("\nthis stage authorises no warm-up, no PPO and no promotion.")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
