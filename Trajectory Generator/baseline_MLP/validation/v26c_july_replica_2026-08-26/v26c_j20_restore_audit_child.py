"""V26C J20 restore audit - the validation-only child.

This file exists for ONE reason: gate R9 of
``v26c_j20_prereg_restore_audit_rev1.json`` requires the optimizer to be read
from the LIVE learner that ``algo.restore_from_path`` created, at the instant
the restore returns. That instant is inside the training child process and
cannot be observed from outside it.

What this file does
-------------------
It imports ``train_ppo_mlp`` and rebinds ONE module attribute,
``_reapply_optimizer_learning_rate``, in this process only. The replacement:

  1. calls the ORIGINAL first and keeps its return value untouched,
  2. immediately captures the live optimizer state from the learner, using the
     production traversal (``_learner_call_results`` ->
     ``learner_group.foreach_learner`` -> ``get_optimizers_for_module``),
  3. normalises it and the source ``learner/state.pkl`` to one canonical form,
  4. compares them recursively and exactly - keys, primitives, param_groups and
     the BYTES of every moment tensor,
  5. writes canonical evidence, and
  6. RAISES on any mismatch, so the production run fails.

What this file does NOT do
--------------------------
It does not emulate the restore. After installing the wrap it sets ``sys.argv``
and calls ``train_ppo_mlp.main()``, which reaches ``run()`` and the real
``algo.restore_from_path``. It constructs no Algorithm of its own and
reimplements no RLlib code path.

It modifies NO PRODUCTION FILE. The rebinding is an in-memory attribute of the
imported module and lives and dies with this process. The only file it writes is
the audit evidence, and that goes into the validation leaf of this stage - which
physically sits under ``baseline_MLP/validation/``, so "writes nothing under
baseline_MLP" would be false. The accurate claim is the one made here: no
production module, config or artefact is written.

Why the original is called first: the architect requires it. The original
reapplies the configured learning rate to ``param_groups``. Both the checkpoint
and this stage use 1e-4, so the write is numerically a no-op; had they differed
the ``lr`` comparison below would fail and say so, which is the point.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import pickle
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent

# The learner state the restore is audited against, and where the evidence goes.
# Both are set by main() from the command line; the capture closure reads them.
OPTIMIZER_ENTRY = "default_policy_default_optimizer"
MODULE_ID = "default_policy"
EXPECTED_STATE_INDICES = (6, 7, 8, 9, 10, 11)
EXPECTED_PARAM_COUNT = 12
# Not a new assumption: the warm-up under audit already recorded this name in
# its committed train_iterations.jsonl, under optimizer_learning_rates ->
# {"optimizer_name": "default_optimizer", "optimizer_type": "Adam", ...}.
EXPECTED_OPTIMIZER_NAME = "default_optimizer"
MOMENT_KEYS = ("exp_avg", "exp_avg_sq")
EVIDENCE_KIND = "LIVE OPTIMIZER RESTORE AUDIT"
STAGE_MARKER = "after_restore"


class LiveOptimizerMismatch(RuntimeError):
    """Raised when the live optimizer does not match the checkpoint exactly."""


# ------------------------------------------------------------- normalisation --

def normalise(value):
    """Map NumPy, torch and Python values onto ONE canonical form.

    The source is a pickle of NumPy arrays and Python scalars; the live
    optimizer holds torch tensors. Comparing the raw objects would fail on
    representation instead of content, so both sides come through here first
    and only then are compared with ``==``.

    The rules are fixed by v26c_j20_prereg_restore_audit_rev1.json. An
    unrecognised type becomes ('opaque', type name), which can never compare
    equal to a real value on the other side, so a surprise fails closed rather
    than passing silently.
    """
    import numpy as np

    # bool BEFORE int: bool is a subclass of int and must not be folded into it.
    if isinstance(value, bool):
        return ("prim", "bool", value)
    if value is None:
        return ("prim", "NoneType", None)
    if isinstance(value, str):
        return ("prim", "str", value)
    if isinstance(value, (int, float)):
        # Same branch as a zero-dimensional tensor, and that is the point: the
        # checkpoint stores Adam's step as a Python float while the live
        # optimizer holds it as a 0-d tensor. Both are the same fact, so both
        # normalise to a scalar and are compared by value with ==.
        return ("scalar", value)

    base = None
    dtype = None
    if hasattr(value, "detach") and hasattr(value, "cpu"):        # torch.Tensor
        detached = value.detach().cpu()
        base = detached.numpy()
        dtype = str(detached.dtype).replace("torch.", "")
    elif isinstance(value, np.ndarray):
        base = value
        dtype = str(value.dtype)
    elif isinstance(value, np.generic):                           # NumPy scalar
        base = np.asarray(value)
        dtype = str(base.dtype)

    if base is not None:
        # ndim is read BEFORE ascontiguousarray, which promotes a 0-d array to
        # shape (1,) and would otherwise hide every scalar in the tensor branch.
        if base.ndim == 0:
            # A zero-dimensional float32 tensor holding 81.0 and a Python float
            # 81.0 are the same fact recorded two ways. Both become a scalar.
            return ("scalar", base.item())
        array = np.ascontiguousarray(base)
        return ("tensor", dtype, tuple(int(d) for d in array.shape),
                hashlib.sha256(array.tobytes(order="C")).hexdigest())

    if isinstance(value, (list, tuple)):
        # betas survives the round trip as either a tuple or a list.
        return ("seq", [normalise(item) for item in value])
    if isinstance(value, dict):
        return ("map", [[normalise(key), normalise(value[key])]
                        for key in sorted(value, key=repr)])
    return ("opaque", type(value).__name__)


def canonical_digest(normalised) -> str:
    """A stable sha256 over a whole normalised tree.

    repr() of the normalised structure is deterministic: it contains only
    tuples, lists, strings, bools, None and numbers, and Python's float repr is
    the shortest round-tripping form. Two trees with the same digest are the
    same tree, so the source and the live optimizer can be compared with one
    number in addition to the element-by-element walk.
    """
    return hashlib.sha256(repr(normalised).encode("utf-8")).hexdigest()


def differences(left, right, path: str = "") -> list:
    """Every place two normalised trees disagree, as readable paths.

    Recursive rather than a single ``==`` so a failure names the key that broke
    instead of only reporting that something did.
    """
    where = path or "<root>"
    if type(left) is not type(right):
        return ["%s: kind %s vs %s" % (where, type(left).__name__,
                                       type(right).__name__)]
    if isinstance(left, tuple) and left and isinstance(left[0], str):
        kind_left, kind_right = left[0], right[0]
        if kind_left != kind_right:
            return ["%s: %s vs %s" % (where, kind_left, kind_right)]
        if kind_left in ("prim", "scalar", "tensor"):
            return [] if left == right else [
                "%s: %r vs %r" % (where, left[1:], right[1:])]
        if kind_left == "seq":
            items_left, items_right = left[1], right[1]
            if len(items_left) != len(items_right):
                return ["%s: length %d vs %d"
                        % (where, len(items_left), len(items_right))]
            found = []
            for index, (a, b) in enumerate(zip(items_left, items_right)):
                found += differences(a, b, "%s[%d]" % (where, index))
            return found
        if kind_left == "map":
            keys_left = [entry[0] for entry in left[1]]
            keys_right = [entry[0] for entry in right[1]]
            if keys_left != keys_right:
                only_left = [k for k in keys_left if k not in keys_right]
                only_right = [k for k in keys_right if k not in keys_left]
                return ["%s: key sets differ; only source %r, only live %r"
                        % (where, only_left, only_right)]
            found = []
            for (key, a), (_, b) in zip(left[1], right[1]):
                label = key[-1] if isinstance(key, tuple) else key
                found += differences(a, b, "%s.%s" % (where, label))
            return found
        return ["%s: opaque value %r vs %r" % (where, left, right)]
    return [] if left == right else ["%s: %r vs %r" % (where, left, right)]


# ----------------------------------------------------------------- the capture --

def capture_live_optimizer(learner) -> list:
    """Read the live optimizers of the module, normalised.

    Runs on the learner. It returns plain data - no tensors - so the same code
    is correct whether ``foreach_learner`` executes locally or on a Ray actor.
    The traversal is the production one: ``get_optimizers_for_module``.
    """
    captured = []
    for name, optimizer in learner.get_optimizers_for_module(MODULE_ID):
        state = optimizer.state_dict()
        captured.append({
            "optimizer_name": str(name),
            "optimizer_type": type(optimizer).__name__,
            "normalised": normalise(state),
            "normalised_param_groups": normalise(state.get("param_groups", [])),
            "top_level_keys": sorted(state),
            "state_indices": sorted(int(k) for k in state.get("state", {})),
            "param_group_sizes": [len(group.get("params", []))
                                  for group in state.get("param_groups", [])],
            "moment_digests": {
                str(index): {
                    key: normalise(state["state"][index][key])
                    for key in MOMENT_KEYS if key in state["state"][index]
                }
                for index in sorted(state.get("state", {}))
            },
        })
    if not captured:
        raise LiveOptimizerMismatch(
            "no optimizer is registered for %s on the live learner" % MODULE_ID)
    return captured


def read_source_optimizer(source_state: pathlib.Path) -> dict:
    """The checkpoint's optimizer state_dict, straight off the pickle."""
    with source_state.open("rb") as handle:
        payload = pickle.load(handle)
    entry = (payload.get("optimizer") or {}).get(OPTIMIZER_ENTRY)
    if not isinstance(entry, dict) or "state" not in entry:
        raise LiveOptimizerMismatch(
            "%s holds no %s entry with a state_dict" % (source_state,
                                                        OPTIMIZER_ENTRY))
    return entry["state"]


def moment_digest_map(state: dict) -> dict:
    """Per-index sha256 of exp_avg and exp_avg_sq, for the evidence file."""
    return {
        str(index): {key: normalise(state["state"][index][key])
                     for key in MOMENT_KEYS if key in state["state"][index]}
        for index in sorted(state.get("state", {}))
    }


def compare(source_state: pathlib.Path, captured: list) -> dict:
    """Compare the live optimizer against the checkpoint. Exact, recursive.

    The evidence this returns is what rev1 promised and rev3 makes explicit: a
    whole-tree digest for each side that must coincide, a param_groups
    comparison of its own, and the top-level keys and Adam index set of both
    sides - not only a difference count.
    """
    source = read_source_optimizer(source_state)
    normalised_source = normalise(source)
    normalised_source_pg = normalise(source.get("param_groups", []))

    problems: list = []
    if len(captured) != 1:
        problems.append("expected exactly one learner, captured %d"
                        % len(captured))
    live_entries = [entry for learner in captured for entry in learner] \
        if captured and isinstance(captured[0], list) else list(captured)
    if len(live_entries) != 1:
        problems.append("expected exactly one optimizer, captured %d"
                        % len(live_entries))

    live = live_entries[0] if live_entries else None
    found = differences(normalised_source, live["normalised"],
                        "optimizer") if live else ["no live optimizer captured"]
    pg_found = differences(normalised_source_pg,
                           live["normalised_param_groups"],
                           "param_groups") if live else \
        ["no live optimizer captured"]

    source_indices = sorted(int(k) for k in source.get("state", {}))
    source_top = sorted(source)

    if live is not None:
        if sorted(live["top_level_keys"]) != ["param_groups", "state"]:
            problems.append("live top-level keys are %s, expected "
                            "['param_groups', 'state']"
                            % live["top_level_keys"])
        if source_top != ["param_groups", "state"]:
            problems.append("source top-level keys are %s, expected "
                            "['param_groups', 'state']" % source_top)
        if tuple(live["state_indices"]) != EXPECTED_STATE_INDICES:
            problems.append("live Adam state indices are %s, expected %s"
                            % (live["state_indices"],
                               list(EXPECTED_STATE_INDICES)))
        if tuple(source_indices) != EXPECTED_STATE_INDICES:
            problems.append("source Adam state indices are %s, expected %s"
                            % (source_indices, list(EXPECTED_STATE_INDICES)))
        if live["param_group_sizes"] != [EXPECTED_PARAM_COUNT]:
            problems.append("live param_groups hold %s parameters, expected [%d]"
                            % (live["param_group_sizes"], EXPECTED_PARAM_COUNT))
        if live.get("optimizer_name") != EXPECTED_OPTIMIZER_NAME:
            problems.append("the live optimizer is named %r, expected %r"
                            % (live.get("optimizer_name"),
                               EXPECTED_OPTIMIZER_NAME))

    digest_source = canonical_digest(normalised_source)
    digest_live = canonical_digest(live["normalised"]) if live else None
    pg_digest_source = canonical_digest(normalised_source_pg)
    pg_digest_live = canonical_digest(live["normalised_param_groups"]) \
        if live else None
    if digest_live is not None and digest_live != digest_source:
        problems.append("the normalised whole-tree digests differ: source %s, "
                        "live %s" % (digest_source, digest_live))

    exact = not found and not problems
    return {
        "kind": EVIDENCE_KIND,
        "stage_marker": STAGE_MARKER,
        "exact": exact,
        "differences": found,
        "problems": problems,
        "difference_count": len(found),
        "learner_count": len(captured),
        "optimizer_names": [entry.get("optimizer_name") for entry in live_entries],
        "optimizer_types": [entry.get("optimizer_type") for entry in live_entries],
        "expected_optimizer_name": EXPECTED_OPTIMIZER_NAME,
        "source_state_pkl": str(source_state),
        "source_state_sha256": hashlib.sha256(
            source_state.read_bytes()).hexdigest(),
        "normalised_digest_source": digest_source,
        "normalised_digest_live": digest_live,
        "normalised_digests_match": digest_live == digest_source,
        "param_groups": {
            "exact": not pg_found,
            "differences": pg_found,
            "difference_count": len(pg_found),
            "digest_source": pg_digest_source,
            "digest_live": pg_digest_live,
            "digests_match": pg_digest_live == pg_digest_source,
        },
        "top_level_keys_source": source_top,
        "top_level_keys_live": live["top_level_keys"] if live else [],
        "state_indices_source": source_indices,
        "state_indices_live": live["state_indices"] if live else [],
        "expected_state_indices": list(EXPECTED_STATE_INDICES),
        "expected_param_count": EXPECTED_PARAM_COUNT,
        "param_group_sizes_live": live["param_group_sizes"] if live else [],
        "moment_digests_source": moment_digest_map(source),
        "moment_digests_live": live["moment_digests"] if live else {},
        "method": "the original _reapply_optimizer_learning_rate was called "
                  "first; the live optimizer was then read through "
                  "get_optimizers_for_module and normalised on the learner; "
                  "both sides were compared recursively and exactly, with "
                  "moment tensors compared by dtype, shape and sha256 of their "
                  "C-order bytes, param_groups compared separately, and the "
                  "whole normalised tree reduced to one digest per side",
    }


def write_evidence(path: pathlib.Path, payload: dict) -> None:
    """Canonical JSON. Written on mismatch too, so evidence survives failure."""
    path.write_bytes(json.dumps(payload, indent=2, sort_keys=True,
                                allow_nan=False).encode("utf-8"))


# -------------------------------------------------------------------- the wrap --

def install_wrap(train_ppo_mlp, source_state: pathlib.Path,
                 evidence: pathlib.Path) -> None:
    """Rebind _reapply_optimizer_learning_rate, in THIS process only."""
    original = getattr(train_ppo_mlp, "_reapply_optimizer_learning_rate", None)
    if original is None:
        raise LiveOptimizerMismatch(
            "train_ppo_mlp has no _reapply_optimizer_learning_rate to wrap; "
            "the production module is not the one this audit was written for")
    if getattr(original, "_v26c_restore_audit_wrap", False):
        raise LiveOptimizerMismatch("the wrap is already installed")

    def wrapped(algo, learning_rate):
        reports = original(algo, learning_rate)          # production, untouched
        captured = train_ppo_mlp._learner_call_results(algo,
                                                       capture_live_optimizer)
        verdict = compare(source_state, list(captured))
        verdict["reapplied_learning_rate"] = float(learning_rate)
        verdict["production_lr_reports"] = reports
        write_evidence(evidence, verdict)
        if not verdict["exact"]:
            raise LiveOptimizerMismatch(
                "the live optimizer does not match the checkpoint exactly: "
                "%d difference(s), %d problem(s). First: %s"
                % (verdict["difference_count"], len(verdict["problems"]),
                   (verdict["differences"] or verdict["problems"] or ["-"])[0]))
        return reports

    wrapped._v26c_restore_audit_wrap = True
    train_ppo_mlp._reapply_optimizer_learning_rate = wrapped


# -------------------------------------------------------------------- the CLI --

def split_argv(argv: list) -> tuple:
    """Our own arguments, then '--', then everything train_ppo_mlp gets."""
    if "--" not in argv:
        raise SystemExit("the wrapper requires a '--' separator before the "
                         "train_ppo_mlp arguments")
    index = argv.index("--")
    return argv[:index], argv[index + 1:]


def main(argv: list | None = None) -> int:
    """Install the wrap, then hand over to the real trainer."""
    mine, theirs = split_argv(list(sys.argv[1:] if argv is None else argv))
    parser = argparse.ArgumentParser(
        description="V26C J20 restore audit - validation-only child wrapper")
    parser.add_argument("--audit-source-state", required=True)
    parser.add_argument("--audit-evidence", required=True)
    args = parser.parse_args(mine)

    source_state = pathlib.Path(args.audit_source_state).resolve()
    evidence = pathlib.Path(args.audit_evidence).resolve()
    if not source_state.is_file():
        raise SystemExit("the source learner state does not exist: %s"
                         % source_state)
    if evidence.exists():
        raise SystemExit("refusing to overwrite existing evidence: %s"
                         % evidence)
    if not theirs:
        raise SystemExit("no train_ppo_mlp arguments after '--'")

    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import train_ppo_mlp

    install_wrap(train_ppo_mlp, source_state, evidence)

    # Hand over to the REAL entry point. parse_args() reads sys.argv, so the
    # argv we were given is installed verbatim; nothing here re-parses or
    # re-interprets a single training flag.
    sys.argv = [str(BASELINE / "train_ppo_mlp.py")] + list(theirs)
    try:
        train_ppo_mlp.main()
    except SystemExit as exit_request:
        return int(exit_request.code or 0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
