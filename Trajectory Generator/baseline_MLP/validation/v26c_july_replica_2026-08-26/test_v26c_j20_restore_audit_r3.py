"""Static and unit test suite for V26C J20 R3 - the canonicalised restore audit.

Launches no child, calls no ray.init, builds no Algorithm, constructs no
environment, samples nothing, rolls out nothing and trains nothing. It imports
``ray.rllib.utils.torch_utils`` for one reason only - to exercise the very
conversion function the audit canonicalises through - and never starts a Ray
runtime.

It holds the R3 package to nine properties:

  * the R3 runner is a MECHANICAL COPY of the R2 runner: exactly seven functions
    differ - live_restore_completed (new), preflight, go_pin_targets,
    expected_pin_hashes, check_entry_evidence, evaluate_gates and run_execution
    - and every other function is byte-identical, so R2's verified behaviour
    carries over. restore_command, hermetic_restore, verify_commit, validate_go,
    child_environment and launch_once are among the identical ones;
  * the eight differences R2 committed are ACCEPTED EQUIVALENCES and nothing
    else is: each is asserted one at a time, on the REAL checkpoint values, and
    each is shown to differ raw and agree canonically;
  * every corruption class rev6 lists still fails: a boolean flip, a boolean
    against a numeric zero, a beta/eps/lr beyond float32, a dropped key, an
    added key, a shortened params list, a reordered params list, a changed state
    index set, and a changed moment value, dtype or shape;
  * the moments stay BYTE-EXACT through canonicalisation, and a moment whose
    dtype the conversion would silently downcast is caught by the two
    moments_unchanged flags rather than slipping through the canonical walk;
  * rev7's learning-rate gate holds: the value the RESTORE produced, before the
    production reapply overwrites it, is compared canonically and ENFORCED. The
    masking scenario is reproduced in full - a live optimizer whose lr was
    rewritten to the configured value, so that the canonical walk sees nothing,
    still fails when the pre-reapply value was wrong;
  * there is no tolerance: no epsilon, no isclose, no approximate comparison
    anywhere in the child, asserted statically over its source;
  * the conversion the audit canonicalises through is the one RLlib takes, and
    it is a fixed point;
  * R1 and R2 are untouched: their leaves, runners, suites and GOs all re-hash
    to their pins, and both still read FAIL_CLOSED under TECHNICAL_INVALID;
  * the leaf is new and distinct from both, born invalid, with exactly one
    launch and no retry path;
  * the thirteen gates are unmoved and non-vacuous, and the five downstream of
    R9 are evaluated only after a completed live restore - without ever being
    allowed to pass by not being evaluated.

Run:
    PYTHONDONTWRITEBYTECODE=1 python test_v26c_j20_restore_audit_r3.py
"""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import pathlib
import pickle
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

sys.dont_write_bytecode = True

import numpy as np                                              # noqa: E402
import torch                                                    # noqa: E402

import v26c_j20_restore_audit_r2 as R2                          # noqa: E402
import v26c_j20_restore_audit_r3 as R                           # noqa: E402
import v26c_j20_restore_audit_r3_child as W                     # noqa: E402

R2_PATH = HERE / R.R2_RUNNER_NAME
R3_PATH = HERE / R.RUNNER_NAME
R2_SOURCE = R2_PATH.read_text(encoding="utf-8")
R3_SOURCE = R3_PATH.read_text(encoding="utf-8")
R3_TREE = ast.parse(R3_SOURCE)
CHILD_PATH = HERE / R.CHILD_NAME
CHILD_SOURCE = CHILD_PATH.read_text(encoding="utf-8")
CHILD_TREE = ast.parse(CHILD_SOURCE)

REV5 = json.loads((HERE / R.PREREG_REV5_NAME).read_text(encoding="utf-8"))
REV6 = json.loads((HERE / R.PREREG_REV6_NAME).read_text(encoding="utf-8"))
REV7 = json.loads((HERE / R.PREREG_REV7_NAME).read_text(encoding="utf-8"))

R1_LEAF = HERE / R.R1_LEAF_REL
R2_LEAF = HERE / R.R2_LEAF_REL
LEAF = HERE / R.LEAF_ROOT / R.LEAF_NAME
# The rev6 DRAFT is a HISTORICAL preparation artefact: never edited, never
# deleted, and superseded. The rev7 DRAFT is the current one.
HISTORICAL_DRAFT_GO = HERE / "v26c_j20_restore_audit_r3_go_DRAFT.json"
DRAFT_GO = HERE / "v26c_j20_restore_audit_r3_go_DRAFT_rev7.json"
SOURCE_STATE_PATH = HERE / R.SOURCE_LEAF_REL / R.LEARNER_STATE_REL
# The value production itself committed in R2: float32(1e-4).
R2_LR_BEFORE = 9.999999747378752e-05

# Seven functions may differ: one new plus six modified. Nothing else.
DECLARED_NEW = {"live_restore_completed"}
DECLARED_CHANGED = {"preflight", "go_pin_targets", "expected_pin_hashes",
                    "check_entry_evidence", "evaluate_gates", "run_execution"}

DEVICE = "cpu"

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


class _BlankStrings(ast.NodeTransformer):
    """Replace every string literal with '' so only CODE is left."""

    def visit_Constant(self, node):                          # noqa: N802
        if isinstance(node.value, str):
            return ast.copy_location(ast.Constant(value=""), node)
        return node


def code_only(source: str) -> str:
    """The source with every docstring and string literal emptied.

    A textual scan over raw source cannot tell a prose mention of ray.init in a
    docstring from a call to it. Blanking the strings first makes the scan mean
    what it says.
    """
    tree = _BlankStrings().visit(ast.parse(source))
    ast.fix_missing_locations(tree)
    return ast.unparse(tree)


def calls_attribute(tree, owner: str, attribute: str) -> bool:
    """Is there a call of the form <owner>.<attribute>(...) anywhere?"""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (isinstance(func, ast.Attribute) and func.attr == attribute
                and isinstance(func.value, ast.Name) and func.value.id == owner):
            return True
    return False


# --------------------------------------------------- the real checkpoint state

def real_source_state() -> dict:
    """The checkpoint's optimizer state_dict, read off the pinned pickle."""
    path = HERE / R.SOURCE_LEAF_REL / R.LEARNER_STATE_REL
    with path.open("rb") as handle:
        payload = pickle.load(handle)
    return payload["optimizer"]["default_policy_default_optimizer"]["state"]


def simulated_live_state(source: dict) -> dict:
    """What the live optimizer holds after the restore, reconstructed.

    Two steps, both taken from production and neither invented here:

      1. RLlib converts the checkpoint with convert_to_torch_tensor before
         handing it to load_state_dict, and torch copies the saved
         hyperparameters into param_groups verbatim. So every param_groups leaf
         on the live side is the CONVERTED leaf.
      2. train_ppo_mlp's _set_optimizer_learning_rate_on_learner then writes
         float(learning_rate) back into param_groups lr, which is why lr is a
         Python float on the live side and is NOT among R2's differences.

    Step 2 matters: without it the reconstruction would show NINE raw
    differences and would not be the state R2 measured.
    """
    live = W.rllib_convert(source, DEVICE)
    live["param_groups"][0]["lr"] = float(1e-4)
    return live


def r2_committed_differences() -> list:
    """The eight difference strings R2 committed, read from its own leaf."""
    evidence = json.loads((R2_LEAF / "live_optimizer_audit.json"
                           ).read_text(encoding="utf-8"))
    return list(evidence["differences"])


def raw_differences(left, right, root="optimizer") -> list:
    return W.as_strings(W.differences(W.normalise(left), W.normalise(right),
                                      root))


def canonical_differences(left, right, root="optimizer") -> list:
    return W.as_strings(W.differences(W.canonicalise_converted(left, DEVICE),
                                      W.canonicalise_converted(right, DEVICE),
                                      root))


SOURCE_STATE = real_source_state()
LIVE_STATE = simulated_live_state(SOURCE_STATE)


# ------------------------------------------------- A. the mechanical copy

def test_mechanical_copy() -> None:
    left, right = function_map(R2_SOURCE), function_map(R3_SOURCE)
    new = set(right) - set(left)
    removed = set(left) - set(right)
    changed = {k for k in set(left) & set(right) if left[k] != right[k]}

    check("A01 no function was removed from the R2 runner", not removed,
          str(sorted(removed)))
    check("A02 exactly the declared new function was added",
          new == DECLARED_NEW, str(sorted(new)))
    check("A03 exactly the declared functions changed",
          changed == DECLARED_CHANGED, str(sorted(changed)))
    shared = set(left) & set(right)
    identical = shared - changed
    check("A04a the R2 runner defines 26 functions", len(left) == 26,
          str(len(left)))
    check("A04b the R3 runner defines 27 functions", len(right) == 27,
          str(len(right)))
    check("A04c 26 functions are shared", len(shared) == 26, str(len(shared)))
    check("A04d 20 of them are byte-identical", len(identical) == 20,
          str(len(identical)))
    check("A04e 6 of them are modified", len(changed) == 6, str(len(changed)))
    check("A04f 1 is new", len(new) == 1, str(len(new)))
    check("A04g so 7 functions differ in total, and 20 do not",
          len(changed) + len(new) == 7 and len(identical) == 20)

    for name, why in (
        ("restore_command", "the four operations and the delegated argv"),
        ("hermetic_restore", "R8 and R13, the hermetic second opinion"),
        ("verify_commit", "the post-commit protocol and the marker swap"),
        ("validate_go", "the APPROVED-only GO rule"),
        ("child_environment", "rev4's single environment correction"),
        ("launch_once", "the single Popen with no retry"),
        ("check_pins", "the pin re-hashing"),
        ("check_destination", "the no-clobber rule"),
        ("read_child_artefacts", "what is read back out of the leaf"),
    ):
        check("A05 %s is byte-identical, so %s cannot have moved" % (name, why),
              left.get(name) == right.get(name))

    check("A06 the R2 runner still hashes to its pin",
          sha256_file(R2_PATH) == R.PIN_R2_RUNNER,
          "%s vs %s" % (sha256_file(R2_PATH), R.PIN_R2_RUNNER))
    check("A07 the stage identifiers differ, so no GO can start another stage",
          R.STAGE == "V26C_J20_RESTORE_AUDIT_R3" and R2.STAGE != R.STAGE
          and R.GO_REQUIRED_STAGE == R.STAGE)
    check("A08 the leaf names differ from R1's and R2's",
          R.LEAF_NAME == "j20_restore_audit_v26c_2026-08-28_r3"
          and R.LEAF_NAME != R2.LEAF_NAME and R.LEAF_NAME != R2.R1_LEAF_REL)
    check("A09 the gate count is still thirteen",
          len(R.evaluate_gates({}, {}, {}, {})["gates"]) == 13)
    check("A10 R3 uses its OWN wrapper, not the frozen one",
          R.CHILD_NAME == "v26c_j20_restore_audit_r3_child.py"
          and R.CHILD_NAME != R.FROZEN_CHILD_NAME)
    check("A11 the frozen wrapper is untouched and still hashes to its pin",
          sha256_file(HERE / R.FROZEN_CHILD_NAME) == R.PIN_FROZEN_CHILD)
    check("A12 the frozen wrapper and the R3 wrapper are different files",
          sha256_file(HERE / R.FROZEN_CHILD_NAME) != sha256_file(CHILD_PATH))
    check("A13 the precedence records rev7 first",
          R.PREREG_PRECEDENCE
          == ("rev7", "rev6", "rev5", "rev4", "rev3", "rev2", "rev1", "base"))
    check("A14 rev7 pins the R3 wrapper by hash in the runner, not only in "
          "the GO",
          R.expected_pin_hashes()[R.CHILD_NAME] == R.PIN_R3_CHILD
          and sha256_file(CHILD_PATH) == R.PIN_R3_CHILD,
          sha256_file(CHILD_PATH))


# ------------------------------------- B. the eight accepted equivalences

def test_accepted_equivalences() -> None:
    committed = r2_committed_differences()
    check("B01 R2's committed evidence holds exactly eight differences",
          len(committed) == 8, str(len(committed)))

    raw = raw_differences(SOURCE_STATE, LIVE_STATE)
    check("B02 the reconstructed live state reproduces R2's eight strings "
          "exactly", sorted(raw) == sorted(committed),
          "reconstructed=%s" % sorted(raw))

    # The strongest check in this suite. The reconstruction is not merely
    # difference-compatible with R2: its two RAW whole-tree digests are the two
    # R2 committed, byte for byte. So the state compared below IS the state R2
    # measured, and "the mismatch was representational" stops being an argument
    # about R2 and becomes a measurement on R2's own numbers.
    committed_evidence = json.loads((R2_LEAF / "live_optimizer_audit.json"
                                     ).read_text(encoding="utf-8"))
    check("B02a the reconstruction reproduces R2's committed RAW source digest",
          W.canonical_digest(W.normalise(SOURCE_STATE))
          == committed_evidence["normalised_digest_source"],
          W.canonical_digest(W.normalise(SOURCE_STATE)))
    check("B02b the reconstruction reproduces R2's committed RAW live digest",
          W.canonical_digest(W.normalise(LIVE_STATE))
          == committed_evidence["normalised_digest_live"],
          W.canonical_digest(W.normalise(LIVE_STATE)))
    check("B02c R2 committed those two as NOT matching",
          committed_evidence["normalised_digests_match"] is False
          and committed_evidence["exact"] is False)

    canonical = canonical_differences(SOURCE_STATE, LIVE_STATE)
    check("B03 canonically the same two states are identical",
          canonical == [], str(canonical))

    src_canon = W.canonicalise_converted(SOURCE_STATE, DEVICE)
    live_canon = W.canonicalise_converted(LIVE_STATE, DEVICE)
    check("B04 the canonical whole-tree digests coincide",
          W.canonical_digest(src_canon) == W.canonical_digest(live_canon),
          W.canonical_digest(src_canon))
    check("B05 the RAW whole-tree digests do NOT coincide, which is what made "
          "R2 fail",
          W.canonical_digest(W.normalise(SOURCE_STATE))
          != W.canonical_digest(W.normalise(LIVE_STATE)))
    check("B05a R2's committed moment digests already agreed, 12 of 12",
          committed_evidence["moment_digests_source"]
          == committed_evidence["moment_digests_live"])

    # Each of the eight, one at a time, with the real values on both sides.
    raw_paths = {text.split(":")[0].replace("optimizer.", "", 1)
                 for text in raw}
    for path in R.EXPECTED_ACCEPTED_EQUIVALENCES:
        node_source = W.node_at(src_canon, path)
        node_live = W.node_at(live_canon, path)
        check("B06 %s differs raw" % path, path in raw_paths)
        check("B07 %s agrees canonically" % path,
              node_source is not None and node_source == node_live,
              "%r vs %r" % (node_source, node_live))

    check("B08 the accepted set is exactly the eight rev6 closes it at",
          sorted(raw_paths) == sorted(R.EXPECTED_ACCEPTED_EQUIVALENCES),
          str(sorted(raw_paths)))
    check("B09 rev6 lists the same eight",
          sorted(REV6["correction_5_the_pass_must_be_evidential"]
                 ["the_eight_accepted_equivalences"])
          == sorted(R.EXPECTED_ACCEPTED_EQUIVALENCES))
    check("B10 the wrapper closes the same eight",
          sorted(W.ACCEPTED_EQUIVALENCE_PATHS)
          == sorted(R.EXPECTED_ACCEPTED_EQUIVALENCES))
    check("B11 the wrapper's R2 reference strings are R2's committed ones",
          sorted(W.R2_RAW_DIFFERENCE_STRINGS) == sorted(committed))

    # The five booleans keep their dtype; the three floats keep float32.
    for path in ("param_groups[0].amsgrad", "param_groups[0].capturable",
                 "param_groups[0].decoupled_weight_decay",
                 "param_groups[0].differentiable",
                 "param_groups[0].maximize"):
        node = W.node_at(src_canon, path)
        check("B12 %s canonicalises to a bool tensor, not a number" % path,
              node is not None and node[0] == "t" and node[1] == "bool",
              repr(node))
    for path in ("param_groups[0].betas[0]", "param_groups[0].betas[1]",
                 "param_groups[0].eps", "param_groups[0].lr"):
        node = W.node_at(src_canon, path)
        check("B13 %s canonicalises to float32" % path,
              node is not None and node[0] == "t" and node[1] == "float32",
              repr(node))
    check("B14 weight_decay stays an integer type, so it can never equal a "
          "bool", W.node_at(src_canon, "param_groups[0].weight_decay")[1]
          not in ("bool",))
    for path in ("param_groups[0].foreach", "param_groups[0].fused"):
        check("B15 %s survives canonicalisation as None" % path,
              W.node_at(src_canon, path) == ("none",)
              and W.node_at(live_canon, path) == ("none",))


# ------------------------------------------------ C. the corruption classes

def mutated(mutate):
    """A copy of the reconstructed live state with one thing changed."""
    live = copy.deepcopy(LIVE_STATE)
    mutate(live)
    return live


def flips(label, mutate, expect_path_fragment=None) -> None:
    found = canonical_differences(SOURCE_STATE, mutated(mutate))
    ok = bool(found)
    if ok and expect_path_fragment is not None:
        ok = any(expect_path_fragment in text for text in found)
    check("C %s" % label, ok, str(found[:2]))


def test_corruption_classes() -> None:
    flips("a boolean flip is caught",
          lambda s: s["param_groups"][0].__setitem__("amsgrad",
                                                    torch.tensor(True)),
          "amsgrad")
    flips("a second boolean flip is caught",
          lambda s: s["param_groups"][0].__setitem__("maximize",
                                                     torch.tensor(True)),
          "maximize")
    flips("a bool replaced by the numeric zero is caught, by dtype",
          lambda s: s["param_groups"][0].__setitem__("amsgrad", 0),
          "amsgrad")
    flips("a bool replaced by the numeric one is caught, by dtype",
          lambda s: s["param_groups"][0].__setitem__("maximize", 1),
          "maximize")
    # ... and the dtype really is what separates them.
    false_node = W.canonicalise(W.rllib_convert(False, DEVICE))
    zero_node = W.canonicalise(W.rllib_convert(0, DEVICE))
    check("C a bool and a numeric zero differ by dtype, not by value",
          false_node[1] == "bool" and zero_node[1] == "int64"
          and false_node != zero_node,
          "%r vs %r" % (false_node, zero_node))

    flips("a beta beyond float32 is caught",
          lambda s: s["param_groups"][0].__setitem__(
              "betas", (torch.tensor(0.91), torch.tensor(0.999))),
          "betas[0]")
    flips("an eps beyond float32 is caught",
          lambda s: s["param_groups"][0].__setitem__("eps",
                                                     torch.tensor(1e-07)),
          "eps")
    flips("an lr beyond float32 is caught",
          lambda s: s["param_groups"][0].__setitem__("lr", 1e-3), "lr")
    # The float32 boundary is a property of RLlib's conversion, not a tolerance.
    tiny = canonical_differences(
        SOURCE_STATE, mutated(lambda s: s["param_groups"][0].__setitem__(
            "eps", 1e-08 + 1e-24)))
    check("C a change smaller than float32 does not survive the conversion "
          "itself, so it is not a difference the restore could have preserved",
          tiny == [],
          "%s ; float32(1e-08+1e-24)==float32(1e-08) is %s"
          % (tiny, np.float32(1e-08 + 1e-24) == np.float32(1e-08)))

    flips("a dropped param_groups key is caught",
          lambda s: s["param_groups"][0].pop("capturable"), "key sets differ")
    flips("an added param_groups key is caught",
          lambda s: s["param_groups"][0].__setitem__("novel_flag", False),
          "key sets differ")
    flips("a shortened params list is caught",
          lambda s: s["param_groups"][0].__setitem__(
              "params", s["param_groups"][0]["params"][:-1]), "length")
    flips("a reordered params list is caught",
          lambda s: s["param_groups"][0].__setitem__(
              "params", list(reversed(s["param_groups"][0]["params"]))),
          "params[0]")
    flips("a dropped Adam state index is caught",
          lambda s: s["state"].pop(11), "key sets differ")
    flips("an added Adam state index is caught",
          lambda s: s["state"].__setitem__(12, s["state"][11]),
          "key sets differ")
    flips("a None that became a value is caught",
          lambda s: s["param_groups"][0].__setitem__("foreach", 0), "foreach")

    def perturb_one(state):
        moment = state["state"][6]["exp_avg"].clone()
        flat = moment.reshape(-1)
        flat[0] = flat[0] + torch.tensor(1.0, dtype=flat.dtype)
        state["state"][6]["exp_avg"] = moment
    flips("a single perturbed moment element is caught, by index and key",
          perturb_one, "state.6.exp_avg")
    flips("a reshaped moment is caught",
          lambda s: s["state"][7].__setitem__(
              "exp_avg", s["state"][7]["exp_avg"].reshape(1, -1)),
          "state.7.exp_avg")
    flips("a changed step is caught",
          lambda s: s["state"][8].__setitem__("step", torch.tensor(80.0)),
          "state.8.step")
    flips("a dropped moment key is caught",
          lambda s: s["state"][9].pop("exp_avg_sq"), "key sets differ")


def test_moments_are_byte_exact() -> None:
    raw = W.moment_digest_map(SOURCE_STATE, W.normalise)
    canonical = W.moment_digest_map(W.rllib_convert(SOURCE_STATE, DEVICE),
                                    W.canonicalise)
    check("C moment digests: six indices on both sides",
          sorted(raw) == sorted(canonical) == ["10", "11", "6", "7", "8", "9"],
          str(sorted(canonical)))
    check("C canonicalisation leaves every moment byte-exact",
          W.moment_identity(raw) == W.moment_identity(canonical))
    for index in sorted(raw):
        for key in ("exp_avg", "exp_avg_sq"):
            check("C %s.%s keeps dtype, shape and bytes" % (index, key),
                  list(raw[index][key][1:]) == list(canonical[index][key][1:]),
                  "%r vs %r" % (raw[index][key], canonical[index][key]))
    live_canonical = W.moment_digest_map(W.rllib_convert(LIVE_STATE, DEVICE),
                                         W.canonicalise)
    check("C the checkpoint and the live moments agree, 12 of 12",
          canonical == live_canonical)

    # A float64 moment is downcast by the conversion, so the canonical walk
    # alone cannot see it. The two moments_unchanged flags are what catch it.
    widened = copy.deepcopy(SOURCE_STATE)
    widened["state"][6]["exp_avg"] = \
        np.asarray(widened["state"][6]["exp_avg"], dtype=np.float64)
    widened_raw = W.moment_digest_map(widened, W.normalise)
    widened_canonical = W.moment_digest_map(W.rllib_convert(widened, DEVICE),
                                            W.canonicalise)
    check("C a float64 moment survives the canonical walk, as expected",
          canonical_differences(SOURCE_STATE, widened) == [])
    check("C ... and is caught by moments_unchanged_by_canonicalisation",
          W.moment_identity(widened_raw) != W.moment_identity(widened_canonical),
          "raw %s canonical %s"
          % (W.moment_identity(widened_raw)["6"]["exp_avg"][0],
             W.moment_identity(widened_canonical)["6"]["exp_avg"][0]))
    check("C ... which is true for the real state and false for the widened one",
          W.moment_identity(raw) == W.moment_identity(canonical)
          and W.moment_identity(widened_raw)
          != W.moment_identity(widened_canonical))


# ------------------------------------------ D. no tolerance, and the fixed point

def test_no_tolerance() -> None:
    # The scan runs over CODE ONLY: every docstring and string literal is
    # blanked first, so a prose mention of ray.init cannot be mistaken for a
    # call to it, and the reverse cannot hide one either.
    code = code_only(CHILD_SOURCE)
    lowered = code.lower()
    for banned in ("isclose", "allclose", "rel_tol", "abs_tol", "atol=",
                   "rtol=", "approx", "fabs", ".round(", "epsilon",
                   "tolerance"):
        check("D01 the wrapper's code contains no %r" % banned,
              banned not in lowered)
    check("D01b the wrapper imports no math module",
          "import math" not in lowered)
    check("D02 the wrapper never calls ray.init",
          not calls_attribute(CHILD_TREE, "ray", "init")
          and "ray.init" not in code)
    check("D03 the wrapper never constructs an Algorithm",
          "build_algo" not in code and "PPOConfig" not in code
          and "restore_from_path" not in code)
    check("D04 the wrapper imports nothing heavy at module level",
          not {a.name.split(".")[0]
               for node in CHILD_TREE.body if isinstance(node, ast.Import)
               for a in node.names} & {"torch", "ray", "numpy", "gymnasium"})
    check("D05 the canonical leaf carries dtype, shape and a sha256, and "
          "nothing numeric",
          W.canonicalise(torch.tensor([1.0, 2.0]))[:3]
          == ("t", "float32", (2,)))
    check("D06 two float32 tensors that differ in the last bit differ "
          "canonically",
          W.canonicalise(torch.tensor(1.0))
          != W.canonicalise(torch.tensor(np.nextafter(np.float32(1.0),
                                                      np.float32(2.0)))))


# ------------------------------------- D2. rev7: the learning-rate gate

def lr_reports(before):
    """A production _reapply_optimizer_learning_rate report list."""
    return [[{"optimizer_name": "default_optimizer", "optimizer_type": "Adam",
              "before": before, "requested": 1e-4, "after": 1e-4}]]


def enforce(reports, exact=True):
    """Run the wrapper's own enforcement on a synthetic verdict."""
    verdict = {"exact": exact, "problems": [], "device": DEVICE,
               "canonical": {"differences": [], "difference_count": 0}}
    W.enforce_learning_rate_observation(verdict, SOURCE_STATE_PATH, reports,
                                        1e-4)
    return verdict


def test_learning_rate_gate() -> None:
    checkpoint_lr = (SOURCE_STATE["param_groups"][0] or {}).get("lr")
    check("D20 the checkpoint's lr is the Python float 1e-4",
          checkpoint_lr == 1e-4 and isinstance(checkpoint_lr, float),
          repr(checkpoint_lr))

    # 1. The real, matching pre-reapply report passes.
    good = enforce(lr_reports(R2_LR_BEFORE))
    observation = good["learning_rate_observation"]
    check("D21 the pre-reapply value R2 actually committed matches the "
          "checkpoint canonically", observation["matches"] is True,
          repr(observation))
    check("D22 the verdict stays exact and no problem is appended",
          good["exact"] is True and good["problems"] == [])
    check("D23 the canonical fields are populated and coherent",
          isinstance(observation["canonical_source_lr"], str)
          and observation["canonical_source_lr"].startswith("('t'")
          and observation["canonical_live_lr_before_reapply"]
          == observation["canonical_source_lr"]
          and observation["canonical_dtype"] == "float32"
          and observation["gated"] is True
          and observation["live_lr_before_reapply"] == R2_LR_BEFORE)
    check("D24 R2's OWN committed production_lr_reports satisfy the gate",
          enforce(json.loads((R2_LEAF / "live_optimizer_audit.json").read_text(
              encoding="utf-8"))["production_lr_reports"]
          )["learning_rate_observation"]["matches"] is True)

    # 2. A mismatching pre-reapply lr makes the child verdict non-exact.
    for label, before in (("5e-4, five times too large", 5e-4),
                          ("1e-3, the next decade", 1e-3),
                          ("9e-5, close but beyond float32", 9e-5),
                          ("0.0, a fresh optimizer", 0.0)):
        bad = enforce(lr_reports(before))
        check("D25 a restored lr of %s fails closed" % label,
              bad["learning_rate_observation"]["matches"] is False
              and bad["exact"] is False
              and any("learning rate" in p for p in bad["problems"]),
              repr(bad["problems"][:1]))

    # 3. A missing or foreign report fails closed rather than passing vacuously.
    for label, reports in (
        ("no reports at all", []),
        ("an empty learner report", [[]]),
        ("a report for another optimizer",
         [[{"optimizer_name": "other", "before": R2_LR_BEFORE}]]),
        ("a report with no before field",
         [[{"optimizer_name": "default_optimizer", "after": 1e-4}]]),
    ):
        blank = enforce(reports)
        check("D26 %s fails closed" % label,
              blank["learning_rate_observation"]["matches"] is False
              and blank["exact"] is False
              and blank["learning_rate_observation"][
                  "canonical_live_lr_before_reapply"] is None)

    # 4. THE MASKING SCENARIO, reproduced. The production reapply rewrites
    #    param_groups lr to the configured value, so the canonical walk is
    #    perfectly clean even though the restore loaded 5e-4. Under rev6 this
    #    passed 13 of 13. It must not now.
    masked = copy.deepcopy(LIVE_STATE)
    masked["param_groups"][0]["lr"] = float(1e-4)        # what production writes
    check("D27 with the overwrite in place the canonical walk sees NOTHING",
          canonical_differences(SOURCE_STATE, masked) == [],
          str(canonical_differences(SOURCE_STATE, masked)))
    hidden = enforce(lr_reports(5e-4))
    check("D28 ... and the overwrite still cannot hide the wrong restored lr",
          hidden["exact"] is False
          and hidden["learning_rate_observation"]["matches"] is False)
    check("D29 the problem names the learning rate and both values",
          any("learning rate" in p and "0.0005" in p
              for p in hidden["problems"]), repr(hidden["problems"][:1]))

    # 5. The enforcement runs BEFORE the evidence is written and before the
    #    raise, on the wrapper's real path.
    wrapped_source = None
    for node in ast.walk(CHILD_TREE):
        if isinstance(node, ast.FunctionDef) and node.name == "wrapped":
            wrapped_source = ast.get_source_segment(CHILD_SOURCE, node)
    check("D30 the wrapper enforces the lr BEFORE writing the evidence",
          wrapped_source.index("enforce_learning_rate_observation")
          < wrapped_source.index("write_evidence"))
    check("D31 ... and before the raise",
          wrapped_source.index("enforce_learning_rate_observation")
          < wrapped_source.index("raise"))
    check("D32 the wrapper no longer assigns the observation without gating it",
          "verdict[\"learning_rate_observation\"] =" not in wrapped_source)
    check("D33 the enforcement appears in the CODE exactly twice - its "
          "definition and its single call from the wrap - so there is no "
          "second, ungated path",
          code_only(CHILD_SOURCE).count("enforce_learning_rate_observation")
          == 2,
          str(code_only(CHILD_SOURCE).count(
              "enforce_learning_rate_observation")))

    # 6. End to end at the RUNNER: the evidence the corrected child would emit
    #    for the masking scenario must fail R9 and must make the audit
    #    unpassable. This is the check that would have caught the rev6 defect.
    child, artefacts, hermetic, integrity = good_inputs()
    emitted = enforce(lr_reports(5e-4))
    artefacts["evidence"]["learning_rate_observation"] = \
        emitted["learning_rate_observation"]
    artefacts["evidence"]["exact"] = emitted["exact"]
    artefacts["evidence"]["problems"] = emitted["problems"]
    graded = R.evaluate_gates(child, artefacts, hermetic, integrity)
    gate9 = "R9_live_optimizer_is_byte_exact_after_restore"
    check("D34 the masking scenario fails R9 at the runner",
          graded["gates"][gate9] is False)
    check("D35 ... and the audit cannot reach thirteen of thirteen",
          graded["passed"] < graded["total"], str(graded["passed"]))
    check("D36 ... while the canonical walk itself reported nothing, which is "
          "exactly why the lr had to be gated separately",
          artefacts["evidence"]["canonical"]["differences"] == []
          and artefacts["evidence"]["canonical"]["difference_count"] == 0)

    # 7. The rev6 package would have PASSED this same input. Recorded so the
    #    correction is demonstrated, not asserted.
    rev6_style = copy.deepcopy(artefacts["evidence"])
    rev6_style["exact"] = True
    rev6_style["problems"] = []
    c2, a2, h2, i2 = good_inputs()
    a2["evidence"] = rev6_style
    check("D37 with the lr clause satisfied only by rev6's weaker evidence - "
          "exact true, no problem - R9 still refuses, because rev7 requires "
          "matches itself",
          R.evaluate_gates(c2, a2, h2, i2)["gates"][gate9] is False)


def test_conversion_provenance() -> None:
    provenance = W.rllib_conversion_provenance()
    check("D07 the call site inside _set_optimizer_state is the one the audit "
          "canonicalises through", provenance["call_site_verified"] is True,
          provenance["required_call_site"])
    check("D08 convert_to_torch_tensor still takes a device",
          provenance["device_parameter_present"] is True,
          str(provenance["convert_to_torch_tensor_parameters"]))
    check("D09 the float32 downcast clause is present",
          provenance["downcast_clause_present"] is True)
    check("D10 the None passthrough clause is present",
          provenance["none_passthrough_clause_present"] is True)
    check("D11 the provenance records the library it read",
          bool(provenance["ray_version"])
          and provenance["torch_utils_file"].endswith("torch_utils.py")
          and len(provenance["torch_utils_file_sha256"]) == 64)
    check("D12 the conversion is a fixed point on the checkpoint",
          W.is_fixed_point(SOURCE_STATE, DEVICE) is True)
    check("D13 the conversion is a fixed point on the live state",
          W.is_fixed_point(LIVE_STATE, DEVICE) is True)
    check("D14 None survives the conversion unchanged",
          W.rllib_convert(None, DEVICE) is None
          and W.canonicalise(None) == ("none",))
    check("D15 a None never equals a value canonically",
          W.canonicalise(None) != W.canonicalise(W.rllib_convert(0, DEVICE)))
    check("D16 a 0-d tensor is NOT promoted to shape (1,)",
          W.canonicalise(torch.tensor(81.0))[2] == ()
          and W.canonicalise(torch.tensor([81.0]))[2] == (1,))
    check("D17 a 0-d tensor and a 1-element tensor therefore differ",
          W.canonicalise(torch.tensor(81.0))
          != W.canonicalise(torch.tensor([81.0])))
    check("D18 an int key and a str key can never collide",
          W.canonical_key(6) != W.canonical_key("6"))
    check("D19 an unrecognised type fails closed as opaque",
          W.canonicalise(object())[0] == "opaque")


# ------------------------------------------------- E. R1 and R2 are untouched

def test_predecessors_preserved() -> None:
    for label, path, pin in (
        ("the R1 runner", HERE / R.R1_RUNNER_NAME, R.PIN_R1_RUNNER),
        ("the R1 suite", HERE / R.R1_TEST_NAME, R.PIN_R1_TEST),
        ("the R1 GO", HERE / R.R1_GO_NAME, R.PIN_R1_GO),
        ("the R2 runner", HERE / R.R2_RUNNER_NAME, R.PIN_R2_RUNNER),
        ("the R2 suite", HERE / R.R2_TEST_NAME, R.PIN_R2_TEST),
        ("the R2 GO", HERE / R.R2_GO_NAME, R.PIN_R2_GO),
        ("the frozen wrapper", HERE / R.FROZEN_CHILD_NAME, R.PIN_FROZEN_CHILD),
    ):
        check("E01 %s still hashes to its pin" % label,
              path.is_file() and sha256_file(path) == pin,
              sha256_file(path) if path.is_file() else "missing")

    for leaf_rel, pins in ((R.R1_LEAF_REL, R.PIN_R1_LEAF),
                           (R.R2_LEAF_REL, R.PIN_R2_LEAF)):
        for name, pin in sorted(pins.items()):
            path = HERE / leaf_rel / name
            check("E02 %s/%s is unchanged" % (leaf_rel.split("/")[-1], name),
                  path.is_file() and sha256_file(path) == pin)

    for leaf, gates in ((R1_LEAF, 4), (R2_LEAF, 7)):
        result = json.loads((leaf / "v26c_j20_restore_audit_result.json"
                             ).read_text(encoding="utf-8"))
        check("E03 %s still reads FAIL_CLOSED %d/13" % (leaf.name, gates),
              result.get("verdict") == R.VERDICT_FAILED
              and result.get("gates_passed") == gates
              and result.get("gates_total") == 13,
              "%s %s/%s" % (result.get("verdict"), result.get("gates_passed"),
                            result.get("gates_total")))
        check("E04 %s still carries TECHNICAL_INVALID and not the pass marker"
              % leaf.name,
              (leaf / R.INVALID_MARKER).is_file()
              and not (leaf / R.PASSED_MARKER).exists())
        check("E05 %s promotes nothing" % leaf.name,
              result.get("promotion") == "NONE"
              and result.get("training_ready") is False
              and result.get("next_stage_authorized") is False)

    warmup = HERE / R.SOURCE_LEAF_REL
    check("E06 the warm-up leaf still carries RESTORE_AUDIT_PENDING",
          (warmup / "RESTORE_AUDIT_PENDING").is_file()
          and not (warmup / "TECHNICAL_INVALID").exists())
    check("E07 the R3 leaf does not exist yet, so nothing has been executed",
          not LEAF.exists())


# ------------------------------------------------------------- F. the new leaf

def test_new_leaf_and_single_launch() -> None:
    check("F01 the destination is new and distinct from R1's and R2's",
          str(LEAF) not in (str(R1_LEAF), str(R2_LEAF)))
    check("F02 the destination is absent", not LEAF.exists())

    run = function_named(R3_TREE, "run_execution")
    source = ast.get_source_segment(R3_SOURCE, run)
    check("F03 run_execution refuses to clobber an existing leaf",
          "refusing to clobber an existing leaf" in source)
    check("F04 the leaf is born invalid, marker written first",
          source.index("INVALID_MARKER") < source.index("launch_once"))
    check("F05 the marker is asserted to be the only resident before launch",
          "the destination is not clean" in source
          and source.index("is not clean") < source.index("launch_once"))
    check("F06 launch_once is called exactly once in the whole runner",
          R3_SOURCE.count("launch_once(") == 2)      # the def plus one call
    check("F07 there is exactly one subprocess.Popen in the runner",
          R3_SOURCE.count("subprocess.Popen") == 1)
    check("F08 no retry token appears anywhere in the runner",
          '"retried": False' in R3_SOURCE
          and "retried=True" not in R3_SOURCE)
    for banned in ("while ", "for attempt", "range(3)", "max_retries"):
        check("F09 run_execution contains no %r loop" % banned,
              banned not in source)
    check("F10 the child argv names the R3 wrapper and the R3 leaf",
          str(HERE / R.CHILD_NAME)
          in R.restore_command(str(LEAF))["child_argv"]
          and str(LEAF / R.EVIDENCE_NAME)
          in R.restore_command(str(LEAF))["child_argv"])


# ------------------------------------------- G. preregistration and the GO

def test_prereg_and_go() -> None:
    check("G01 rev6 is an additive amendment for stage R3",
          REV6["stage"] == R.STAGE and REV6["revision"] == "rev6"
          and REV6["kind"] == "IMMUTABLE PREREGISTRATION AMENDMENT")
    check("G02 rev6 carries no self hash", REV6["contains_no_self_hash"] is True)
    check("G03 rev6 pins the base and all five predecessors with the hashes "
          "the runner enforces",
          REV6["cites"]["base_sha256"] == R.PIN_PREREG
          and REV6["cites"]["rev1_sha256"] == R.PIN_PREREG_REV1
          and REV6["cites"]["rev2_sha256"] == R.PIN_PREREG_REV2
          and REV6["cites"]["rev3_sha256"] == R.PIN_PREREG_REV3
          and REV6["cites"]["rev4_sha256"] == R.PIN_PREREG_REV4
          and REV6["cites"]["rev5_sha256"] == R.PIN_PREREG_REV5)
    check("G04 rev6 hashes to the pin the runner embeds",
          sha256_file(HERE / R.PREREG_REV6_NAME) == R.PIN_PREREG_REV6)
    for name, pin in ((R.PREREG_NAME, R.PIN_PREREG),
                      (R.PREREG_REV1_NAME, R.PIN_PREREG_REV1),
                      (R.PREREG_REV2_NAME, R.PIN_PREREG_REV2),
                      (R.PREREG_REV3_NAME, R.PIN_PREREG_REV3),
                      (R.PREREG_REV4_NAME, R.PIN_PREREG_REV4),
                      (R.PREREG_REV5_NAME, R.PIN_PREREG_REV5),
                      (R.PREREG_REV6_NAME, R.PIN_PREREG_REV6)):
        check("G05 %s is sealed and unmodified" % name,
              sha256_file(HERE / name) == pin)

    # rev7: the amendment that closes the learning-rate hole.
    check("G05a rev7 is an additive amendment for stage R3",
          REV7["stage"] == R.STAGE and REV7["revision"] == "rev7"
          and REV7["kind"] == "IMMUTABLE PREREGISTRATION AMENDMENT"
          and REV7["contains_no_self_hash"] is True)
    check("G05b rev7 hashes to the pin the runner embeds",
          sha256_file(HERE / R.PREREG_REV7_NAME) == R.PIN_PREREG_REV7)
    check("G05c rev7 cites rev6 at the hash the runner enforces",
          REV7["cites"]["rev6_sha256"] == R.PIN_PREREG_REV6)
    check("G05d rev7 records the precedence the runner uses",
          REV7["precedence"] == " > ".join(R.PREREG_PRECEDENCE))
    check("G05e rev7 pins the corrected wrapper hash",
          REV7["correction_4_the_child_hash_is_pinned_here"]["the_new_hash"]
          == R.PIN_R3_CHILD == sha256_file(CHILD_PATH))
    check("G05f rev7 preserves rev6 and the original DRAFT at their hashes",
          REV7["correction_5_historical_artefacts_are_preserved"]["rev6"][
              "sha256"] == sha256_file(HERE / R.PREREG_REV6_NAME)
          and REV7["correction_5_historical_artefacts_are_preserved"][
              "the_original_draft_go"]["sha256"]
          == sha256_file(HISTORICAL_DRAFT_GO))
    check("G05g rev7 keeps the gate count at thirteen and adds no gate",
          "the count stays 13" in REV7["additive"])
    check("G05h rev7 records that a passing R3 still leaves training_ready "
          "false and defers the aggregation attestation",
          REV7["note_on_training_readiness"]["status"].startswith("RECORDED"))

    # The rev6 DRAFT survives untouched and is now provably superseded.
    check("G05i the original DRAFT still exists and is unedited",
          HISTORICAL_DRAFT_GO.is_file()
          and sha256_file(HISTORICAL_DRAFT_GO)
          == "dce5a521fb21a5e192d341aff0fbc977da87cf71b8a3b0e81bc522399fe2d908")
    historical = json.loads(HISTORICAL_DRAFT_GO.read_text(encoding="utf-8"))
    check("G05j the original DRAFT is still doubly inert",
          historical.get("status") == "DRAFT"
          and historical.get("authorises_execution") is False)
    historical_verdict = R.validate_go(historical)
    check("G05k it is refused, and now on staleness as well as on status",
          historical_verdict["valid"] is False
          and any("stale" in p for p in historical_verdict["problems"]),
          str(historical_verdict["problems"][:3]))
    check("G05l the two DRAFTs are different files",
          DRAFT_GO != HISTORICAL_DRAFT_GO
          and sha256_file(DRAFT_GO) != sha256_file(HISTORICAL_DRAFT_GO))
    check("G06 rev6 forbids a retry and a second execution",
          any("no automatic retry" in text for text in REV6["prohibitions"])
          and any("without BOTH explicit user authorisation" in text
                  for text in REV6["prohibitions"]))
    check("G07 rev6 forbids writing into the R1 and R2 leaves",
          any("no write of any kind into the R1 leaf or the R2 leaf" in text
              for text in REV6["prohibitions"]))
    check("G08 rev6 forbids touching the frozen wrapper",
          any("frozen wrapper" in text for text in REV6["prohibitions"]))
    check("G09 rev6 forbids any tolerance",
          any("no tolerance" in text for text in REV6["prohibitions"]))
    check("G10 rev6 keeps success authorising nothing",
          REV6["verdicts"]["on_success_still_false"]["promotion"] == "NONE"
          and REV6["verdicts"]["on_success_still_false"]["training_ready"]
          is False
          and REV6["verdicts"]["on_success_still_false"][
              "next_stage_authorized"] is False)

    check("G11 the current rev7 DRAFT GO exists", DRAFT_GO.is_file())
    payload = json.loads(DRAFT_GO.read_text(encoding="utf-8"))
    check("G12 the DRAFT is doubly inert",
          payload.get("status") == "DRAFT"
          and payload.get("authorises_execution") is False)
    verdict = R.validate_go(payload)
    check("G13 validate_go refuses the DRAFT", verdict["valid"] is False)
    check("G14 it refuses it on BOTH counts, status and authorises_execution",
          any("status is" in p for p in verdict["problems"])
          and any("authorises_execution" in p for p in verdict["problems"]),
          str(verdict["problems"][:4]))
    check("G15 the DRAFT pins every artefact the runner requires and nothing "
          "else",
          sorted(payload.get("pinned_artefacts_sha256") or {})
          == sorted(R.go_pin_targets()),
          "%d pinned vs %d required"
          % (len(payload.get("pinned_artefacts_sha256") or {}),
             len(R.go_pin_targets())))
    stale = [label for label, digest
             in (payload.get("pinned_artefacts_sha256") or {}).items()
             if (R.go_pin_targets()[label].is_file()
                 and sha256_file(R.go_pin_targets()[label]) != digest)]
    check("G16 every DRAFT pin is current", not stale, str(stale))
    for flag in ("authorises_retry", "authorises_ppo", "authorises_ex_novo",
                 "authorises_promotion", "authorises_training",
                 "authorises_rewriting_the_warmup_leaf"):
        check("G17 the DRAFT never sets %s" % flag,
              payload.get(flag) is not True)
    check("G18 an APPROVED GO for another stage is still refused",
          R.validate_go(dict(payload, status="APPROVED",
                             authorises_execution=True,
                             stage="V26C_J20_RESTORE_AUDIT_R2"))["valid"]
          is False)
    check("G19 only the exact string APPROVED authorises",
          R.validate_go(dict(payload, status="approved",
                             authorises_execution=True))["valid"] is False
          and R.validate_go(dict(payload, status="ok",
                                 authorises_execution=True))["valid"] is False)


# ------------------------------------- H. the gates, and the downstream rule

def good_evidence() -> dict:
    return {
        "kind": R.EVIDENCE_KIND, "stage_marker": R.EVIDENCE_STAGE_MARKER,
        "source_state_sha256": R.EXPECTED_SOURCE_STATE_SHA,
        "exact": True, "problems": [],
        "raw": {"reproduces_r2_eight": True, "difference_count": 8,
                "differences": list(W.R2_RAW_DIFFERENCE_STRINGS)},
        "canonical": {
            "exact": True, "differences": [], "difference_count": 0,
            "digest_source": "a" * 64, "digest_live": "a" * 64,
            "digests_match": True,
            "param_groups": {"exact": True, "differences": [],
                             "digests_match": True},
        },
        "accepted_equivalence_paths": sorted(R.EXPECTED_ACCEPTED_EQUIVALENCES),
        "accepted_equivalences_are_exactly_the_expected_eight": True,
        "unexplained_canonical_paths": [],
        "moment_digests_source": {"6": {"exp_avg": ["t", "float32", [2], "b"]}},
        "moment_digests_live": {"6": {"exp_avg": ["t", "float32", [2], "b"]}},
        "moment_digests_match": True,
        "moments_unchanged_by_canonicalisation_source": True,
        "moments_unchanged_by_canonicalisation_live": True,
        "conversion": {"call_site_verified": True,
                       "device_parameter_present": True,
                       "ray_version": "2.55.1"},
        "conversion_fixed_point_source": True,
        "conversion_fixed_point_live": True,
        "top_level_keys_source": ["param_groups", "state"],
        "top_level_keys_live": ["param_groups", "state"],
        "state_indices_source": [6, 7, 8, 9, 10, 11],
        "state_indices_live": [6, 7, 8, 9, 10, 11],
        "param_group_sizes_live": [12],
        "learner_count": 1, "optimizer_names": ["default_optimizer"],
        # rev7: produced by the wrapper's own enforcement on the REAL
        # checkpoint, not hand-written, so the fixture cannot drift from what
        # the child actually emits.
        "learning_rate_observation": enforce(
            lr_reports(R2_LR_BEFORE))["learning_rate_observation"],
    }


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
    artefacts = {"summary": summary, "iteration_rows": [],
                 "evidence": good_evidence(), "forbidden_present": [],
                 "supervisor_state_present": False}
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


def test_gates_non_vacuous() -> None:
    child, artefacts, hermetic, integrity = good_inputs()
    graded = R.evaluate_gates(child, artefacts, hermetic, integrity)
    check("H01 a correct R3 run passes all thirteen",
          all(graded["gates"].values()) and graded["passed"] == 13,
          str(sorted(n for n, ok in graded["gates"].items() if not ok)))
    check("H02 all thirteen were evaluated", graded["not_evaluated"] == [])
    check("H03 the gate names are unmoved from R2's",
          sorted(graded["gates"])
          == sorted(R2.evaluate_gates(child, artefacts, hermetic,
                                      integrity)["gates"]))

    def flip(label, gate, mutate):
        c, a, h, i = good_inputs()
        mutate(c, a, h, i)
        after = R.evaluate_gates(c, a, h, i)
        check("H04 %s" % label, after["gates"][gate] is False,
              "%s did not flip" % gate)

    flip("an iteration row fails R1", "R1_zero_new_iterations",
         lambda c, a, h, i: a.__setitem__("iteration_rows", ["{}"]))
    flip("a non-empty history fails R2", "R2_empty_history",
         lambda c, a, h, i: a["summary"].__setitem__("history", [{"i": 1}]))
    flip("restored iteration 0 fails R3", "R3_restored_iteration_is_one",
         lambda c, a, h, i: a["summary"].__setitem__(
             "restored_training_iteration", 0))
    flip("iteration_start 1 fails R4", "R4_next_and_start_are_two",
         lambda c, a, h, i: a["summary"].__setitem__("iteration_start", 1))
    flip("warm_start_raw mode fails R5", "R5_the_restore_path_was_taken",
         lambda c, a, h, i: a["summary"].__setitem__("initialization_mode",
                                                     "warm_start_raw"))
    flip("a wrong actor digest fails R6", "R6_restored_actor_digest",
         lambda c, a, h, i: a["summary"]["actor_freeze_audit"][0].__setitem__(
             "actor_digest", "0" * 64))
    flip("a wrong critic digest fails R7", "R7_restored_critic_digest",
         lambda c, a, h, i: a["summary"]["critic_state_audit"][0].__setitem__(
             "critic_digest", "0" * 64))
    flip("a non-identical module fails R8", "R8_hermetic_full_module_restore",
         lambda c, a, h, i: h.__setitem__("all_keys_byte_identical", False))
    flip("a checkpoint in the leaf fails R10", "R10_nothing_was_trained",
         lambda c, a, h, i: a.__setitem__("forbidden_present",
                                          ["checkpoint_last"]))
    flip("a changed pin fails R11", "R11_sources_unchanged",
         lambda c, a, h, i: i.__setitem__("pins_unchanged", False))
    flip("returncode 124 fails R12", "R12_single_clean_process",
         lambda c, a, h, i: c.__setitem__("returncode", 124))
    flip("non-identical moments fail R13",
         "R13_hermetic_second_opinion_with_byte_identity",
         lambda c, a, h, i: h.__setitem__("all_moments_byte_identical", False))

    gate9 = "R9_live_optimizer_is_byte_exact_after_restore"
    for label, mutate in (
        ("missing evidence",
         lambda e: None),
        ("a canonical difference",
         lambda e: e["canonical"].__setitem__("differences", ["x"])),
        ("a non-zero canonical count",
         lambda e: e["canonical"].__setitem__("difference_count", 1)),
        ("disagreeing canonical digests",
         lambda e: e["canonical"].__setitem__("digest_live", "b" * 64)),
        ("a param_groups difference",
         lambda e: e["canonical"]["param_groups"].__setitem__("differences",
                                                              ["x"])),
        ("raw no longer reproducing R2's eight",
         lambda e: e["raw"].__setitem__("reproduces_r2_eight", False)),
        ("a ninth accepted equivalence",
         lambda e: e.__setitem__(
             "accepted_equivalence_paths",
             sorted(R.EXPECTED_ACCEPTED_EQUIVALENCES) + ["param_groups[0].x"])),
        ("an unexplained canonical path",
         lambda e: e.__setitem__("unexplained_canonical_paths", ["state.6"])),
        ("disagreeing moments",
         lambda e: e.__setitem__("moment_digests_live",
                                 {"6": {"exp_avg": ["t", "float32", [2], "c"]}})),
        ("a moment moved by canonicalisation on the source side",
         lambda e: e.__setitem__(
             "moments_unchanged_by_canonicalisation_source", False)),
        ("a moment moved by canonicalisation on the live side",
         lambda e: e.__setitem__(
             "moments_unchanged_by_canonicalisation_live", False)),
        ("an unverified call site",
         lambda e: e["conversion"].__setitem__("call_site_verified", False)),
        ("a broken fixed point on the source side",
         lambda e: e.__setitem__("conversion_fixed_point_source", False)),
        ("a broken fixed point on the live side",
         lambda e: e.__setitem__("conversion_fixed_point_live", False)),
        ("a wrong source state sha",
         lambda e: e.__setitem__("source_state_sha256", "0" * 64)),
        ("a wrong evidence kind",
         lambda e: e.__setitem__("kind", "LIVE OPTIMIZER RESTORE AUDIT")),
        ("a wrong state index set",
         lambda e: e.__setitem__("state_indices_live", [6, 7, 8, 9, 10])),
        ("a wrong param group size",
         lambda e: e.__setitem__("param_group_sizes_live", [11])),
        ("two learners",
         lambda e: e.__setitem__("learner_count", 2)),
        ("a wrong optimizer name",
         lambda e: e.__setitem__("optimizer_names", ["other"])),
        ("a reported problem",
         lambda e: e.__setitem__("problems", ["something"])),
        # rev7's clause, mutated seven ways.
        ("a learning rate that did not match",
         lambda e: e["learning_rate_observation"].__setitem__("matches", False)),
        ("a missing learning-rate observation",
         lambda e: e.__setitem__("learning_rate_observation", None)),
        ("an observation that is not a dict",
         lambda e: e.__setitem__("learning_rate_observation", "matches")),
        ("a null canonical source lr",
         lambda e: e["learning_rate_observation"].__setitem__(
             "canonical_source_lr", None)),
        ("the string 'None' instead of a canonical node",
         lambda e: e["learning_rate_observation"].update(
             {"canonical_source_lr": "None",
              "canonical_live_lr_before_reapply": "None"})),
        ("disagreeing canonical lr fields",
         lambda e: e["learning_rate_observation"].__setitem__(
             "canonical_live_lr_before_reapply", "('t', 'float32', (), 'zz')")),
        ("a missing pre-reapply value",
         lambda e: e["learning_rate_observation"].__setitem__(
             "live_lr_before_reapply", None)),
        ("a wrong canonical dtype",
         lambda e: e["learning_rate_observation"].__setitem__(
             "canonical_dtype", "float64")),
        ("an observation the wrapper did not gate",
         lambda e: e["learning_rate_observation"].__setitem__("gated", False)),
    ):
        c, a, h, i = good_inputs()
        if label == "missing evidence":
            a["evidence"] = None
        else:
            mutate(a["evidence"])
        after = R.evaluate_gates(c, a, h, i)
        check("H05 R9 fails on %s" % label, after["gates"][gate9] is False)


def test_downstream_discipline() -> None:
    # 1. The live restore never completed: the five are NOT evaluated, do not
    #    count as passed, and the verdict cannot be a pass.
    child, artefacts, hermetic, integrity = good_inputs()
    artefacts["evidence"]["exact"] = False
    artefacts["evidence"]["canonical"]["differences"] = ["x"]
    artefacts["summary"]["optimizer_lr_audit"] = []
    artefacts["summary"]["restored_training_iteration"] = 0
    artefacts["summary"]["restored_logical_iteration"] = 0
    artefacts["summary"]["iteration_start"] = 1
    artefacts["summary"]["next_iteration"] = None
    artefacts["summary"]["actor_freeze_audit"] = []
    artefacts["summary"]["critic_state_audit"] = []
    artefacts["summary"]["stop_reason"] = "error"
    child["returncode"] = 1
    graded = R.evaluate_gates(child, artefacts, hermetic, integrity)

    check("I01 the live restore is reported as not completed",
          graded["live_restore"]["completed"] is False)
    check("I02 exactly the five gates downstream of R9 are not evaluated",
          graded["not_evaluated"] == sorted(R.DOWNSTREAM_OF_R9),
          str(graded["not_evaluated"]))
    check("I03 each one carries the reason",
          all(graded["not_evaluated_reason"][name] == R.NOT_EVALUATED_REASON
              for name in R.DOWNSTREAM_OF_R9))
    check("I04 a gate that was not evaluated is NOT passed",
          all(graded["gates"][name] is False for name in R.DOWNSTREAM_OF_R9))
    check("I05 it does not count towards the passed total",
          graded["passed"] == sum(1 for name, ok in graded["gates"].items()
                                  if ok and graded["evaluated"][name])
          and graded["passed"] <= 8)
    check("I06 R9 is still reported as a genuine failure",
          "R9_live_optimizer_is_byte_exact_after_restore" in graded["failed"])
    check("I07 the five are NOT listed among the failures",
          not set(graded["failed"]) & set(R.DOWNSTREAM_OF_R9),
          str(graded["failed"]))
    check("I08 the audit therefore cannot pass",
          not (not graded["failed"] and not graded["not_evaluated"]
               and graded["passed"] == graded["total"]))

    # 2. This is exactly what R2's committed run looked like.
    r2_summary = json.loads((R2_LEAF / "summary.json").read_text("utf-8"))
    r2_evidence = json.loads((R2_LEAF / "live_optimizer_audit.json"
                              ).read_text("utf-8"))
    restore = R.live_restore_completed(
        {"returncode": 1},
        {"summary": r2_summary, "evidence": r2_evidence})
    check("I09 replayed against R2's own artefacts the restore reads as not "
          "completed", restore["completed"] is False)
    check("I10 ... because the wrapper raised and production never recorded "
          "the after_restore reapply",
          restore["wrapper_did_not_raise"] is False
          and restore["production_recorded_the_reapply"] is False
          and restore["evidence_written"] is True)

    # 3. With the restore complete, the five ARE evaluated and still fail on a
    #    genuine perturbation - the discipline never hides a real failure.
    for gate, mutate in (
        ("R3_restored_iteration_is_one",
         lambda a: a["summary"].__setitem__("restored_logical_iteration", 5)),
        ("R4_next_and_start_are_two",
         lambda a: a["summary"].__setitem__("next_iteration", 3)),
        ("R6_restored_actor_digest",
         lambda a: a["summary"].__setitem__("actor_freeze_audit", [])),
        ("R7_restored_critic_digest",
         lambda a: a["summary"].__setitem__("critic_state_audit", [])),
        ("R12_single_clean_process",
         lambda a: a["summary"].__setitem__("timed_out", True)),
    ):
        c, a, h, i = good_inputs()
        mutate(a)
        after = R.evaluate_gates(c, a, h, i)
        check("I11 with a completed restore, %s is evaluated and fails" % gate,
              after["live_restore"]["completed"] is True
              and after["evaluated"][gate] is True
              and after["gates"][gate] is False
              and gate in after["failed"])


# ------------------------------------------------------------ J. preflight

def test_preflight() -> None:
    module_imports = set()
    for node in R3_TREE.body:
        if isinstance(node, ast.Import):
            module_imports.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            module_imports.add(node.module.split(".")[0])
    for heavy in ("torch", "ray", "gymnasium", "numpy"):
        check("J01 the runner imports no %s at module level" % heavy,
              heavy not in module_imports)

    report = R.preflight(verbose=False)
    check("J02 preflight passes", report["ok"], "; ".join(report["problems"]))
    check("J03 preflight reports thirteen gates", report["gates"] == 13)
    check("J04 preflight creates no leaf", not LEAF.exists())
    check("J05 preflight pins both predecessor leaves",
          any(item["artefact"].startswith(R.R1_LEAF_REL) for item in
              report["pins"])
          and any(item["artefact"].startswith(R.R2_LEAF_REL) for item in
                  report["pins"]))
    check("J06 every pin matches",
          report["pins_matching"] == report["pins_checked"],
          "%d/%d" % (report["pins_matching"], report["pins_checked"]))
    check("J07 the entry evidence records R1 4/13 and R2 7/13",
          report["entry_evidence"]["found"]["r1_gates"] == "4/13"
          and report["entry_evidence"]["found"]["r2_gates"] == "7/13")
    check("J08 the entry evidence confirms R2's moments agreed",
          report["entry_evidence"]["found"]["r2_moments_agreed"] is True)
    check("J09 the entry evidence records R2's eight difference paths",
          report["entry_evidence"]["found"]["r2_difference_count"] == 8)

    command = R.restore_command(str(LEAF))
    check("J10 the three token counts are unchanged from R2's",
          command["derived_token_count"] == R.EXPECTED_DERIVED_TOKENS == 20
          and command["delegated_argv_count"] == R.EXPECTED_DELEGATED_ARGV == 18
          and command["child_argv_count"] == R.EXPECTED_CHILD_ARGV == 25)
    check("J11 the output directory is the R3 leaf",
          command["derived_tokens"][
              command["derived_tokens"].index("--output-dir") + 1] == str(LEAF))
    check("J12 the evidence path is inside the R3 leaf",
          command["child_argv"][5] == str(LEAF / R.EVIDENCE_NAME))
    check("J13 the resume path is still the warm-up checkpoint",
          command["derived_tokens"][-1]
          == str(HERE / R.SOURCE_LEAF_REL / R.CHECKPOINT_REL))
    check("J14 --iteration-start is still not passed",
          "--iteration-start" not in command["child_argv"])
    check("J15 the config is still the critic-only one, unmodified",
          str(HERE / "v26c_j20_warmup_critic_only_cfg.yaml")
          in command["derived_tokens"])
    check("J16 --iterations is still 1, which makes range(2, 2) empty",
          command["delegated_argv"][
              list(command["delegated_argv"]).index("--iterations") + 1] == "1")

    r2_command = R2.restore_command(str(LEAF))
    check("J17 apart from the leaf, the delegated argv is identical to R2's",
          list(command["delegated_argv"]) == list(r2_command["delegated_argv"]))
    check("J18 the derived command - which still names train_ppo_mlp.py - is "
          "identical to R2's",
          list(command["derived_tokens"]) == list(r2_command["derived_tokens"]))
    check("J19 the ONLY difference in the full child argv is the script token, "
          "operation 4",
          [(a, b) for a, b in zip(command["child_argv"],
                                  r2_command["child_argv"]) if a != b]
          == [(str(HERE / R.CHILD_NAME), str(HERE / R.FROZEN_CHILD_NAME))],
          str([(a, b) for a, b in zip(command["child_argv"],
                                      r2_command["child_argv"]) if a != b]))
    check("J20 operation 4 is recorded as substituting the R3 wrapper",
          command["operations"]["4_substituted_script"]["to"]
          == str(HERE / R.CHILD_NAME)
          and command["operations"]["4_substituted_script"]["from"]
          == str(BASELINE / "train_ppo_mlp.py"))


# --------------------------------------------------- K. the wrapper contract

def test_wrapper_contract() -> None:
    wrapped = None
    for node in ast.walk(CHILD_TREE):
        if isinstance(node, ast.FunctionDef) and node.name == "wrapped":
            wrapped = ast.get_source_segment(CHILD_SOURCE, node)
    check("K01 the wrapper defines the replacement function", wrapped is not None)
    check("K02 the ORIGINAL is called first and its value is returned untouched",
          wrapped.index("original(algo, learning_rate)")
          < wrapped.index("_learner_call_results")
          and "return reports" in wrapped)
    check("K03 the capture uses the production traversal",
          "_learner_call_results" in wrapped
          and "get_optimizers_for_module" in CHILD_SOURCE)
    check("K04 the evidence is written BEFORE the raise",
          wrapped.index("write_evidence") < wrapped.index("raise"))
    check("K05 a mismatch raises and is never downgraded to a warning",
          "raise LiveOptimizerMismatch" in wrapped
          and "warn" not in wrapped.lower())
    check("K06 the wrapper refuses to install itself twice",
          "the wrap is already installed" in CHILD_SOURCE)
    check("K07 the wrapper hands over to the REAL entry point and never "
          "reaches for the restore itself",
          "train_ppo_mlp.main()" in code_only(CHILD_SOURCE)
          and "restore_from_path" not in code_only(CHILD_SOURCE))
    check("K08 the wrapper refuses to overwrite existing evidence",
          "refusing to overwrite existing evidence" in CHILD_SOURCE)
    check("K09 the wrapper writes exactly one file",
          CHILD_SOURCE.count("write_bytes(") == 1)
    check("K10 the evidence kind matches what the runner requires",
          W.EVIDENCE_KIND == R.EVIDENCE_KIND
          and W.STAGE_MARKER == R.EVIDENCE_STAGE_MARKER)
    check("K11 the wrapper and the runner agree on the state indices and the "
          "optimizer name",
          tuple(W.EXPECTED_STATE_INDICES) == R.EXPECTED_CRITIC_PARAM_INDICES
          and W.EXPECTED_OPTIMIZER_NAME == R.EXPECTED_OPTIMIZER_NAME
          and W.EXPECTED_PARAM_COUNT == R.EXPECTED_PARAM_COUNT)
    check("K12 the wrapper modifies exactly one module attribute",
          CHILD_SOURCE.count("train_ppo_mlp._reapply_optimizer_learning_rate =")
          == 1)


def main() -> int:
    test_mechanical_copy()
    test_accepted_equivalences()
    test_corruption_classes()
    test_moments_are_byte_exact()
    test_no_tolerance()
    test_learning_rate_gate()
    test_conversion_provenance()
    test_predecessors_preserved()
    test_new_leaf_and_single_launch()
    test_prereg_and_go()
    test_gates_non_vacuous()
    test_downstream_discipline()
    test_preflight()
    test_wrapper_contract()

    failed = [(n, d) for n, ok, d in CHECKS if not ok]
    for name, detail in failed:
        print("FAIL  %s  %s" % (name, detail))
    print("%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
