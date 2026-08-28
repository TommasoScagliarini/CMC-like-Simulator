"""V26C J20 R3 restore audit - the validation-only child, canonicalised.

A NEW file. ``v26c_j20_restore_audit_child.py`` - the wrapper R1 and R2 both
used - is pinned by two GOs and named by two receipts, so it is part of their
evidence. It is not modified, not copied over and not deleted; R3 re-hashes it
to 38a57b25da27666a99891511666d63cac07534aa7602f20469048012f536caf4 and leaves
it alone.

Why a third attempt exists
--------------------------
R2 reached ``algo.restore_from_path`` for the first time and gate R9 reported
eight differences, all inside ``param_groups[0]``: five Python booleans against
zero-dimensional tensors, and betas[0], betas[1] and eps against their float32
images. Every moment tensor agreed byte for byte, the state indices agreed, the
top-level keys agreed.

Those eight are not a finding about the restore. They are what happens when the
checkpoint is read BEFORE RLlib's conversion and the live optimizer is read
AFTER it. In the installed ray 2.55.1::

    # ray/rllib/core/learner/torch/torch_learner.py
    self._named_optimizers[name].load_state_dict(
        convert_to_torch_tensor(state_dict["state"], device=self._device))

and ``convert_to_torch_tensor`` maps EVERY leaf: ``None`` passes through, an
existing tensor is kept, a numpy array is wrapped, and anything else becomes
``torch.from_numpy(np.asarray(item))``; then every floating tensor that is not
float16 is downcast with ``tensor.float()``, and the result is moved to the
device. So a Python ``False`` becomes a 0-d ``bool`` tensor and a Python ``0.9``
becomes float32 ``0.8999999761581421``, necessarily and by design.

What this file does differently
-------------------------------
It compares the two sides under the SAME post-conversion semantics:

  1. it calls the ORIGINAL ``_reapply_optimizer_learning_rate`` first and keeps
     its return value untouched - unchanged from rev1;
  2. it captures the live optimizer through the production traversal at the
     instant the restore returns - unchanged from rev1;
  3. it records the RAW provenance of both sides under rev2's normalisation,
     unchanged, so R2's eight differences are reproduced rather than hidden;
  4. it then passes BOTH sides through RLlib's own
     ``convert_to_torch_tensor`` - imported, never reimplemented - and reduces
     each result to a canonical tree in which every leaf carries its dtype, its
     shape and the sha256 of its C-order bytes;
  5. it compares those trees exactly. There is no tolerance, no epsilon and no
     approximate comparison anywhere in this file;
  6. it measures the learning rate the restore produced, BEFORE the production
     reapply overwrites it, and gates on it - see
     ``enforce_learning_rate_observation``;
  7. it writes the raw provenance, the canonical digests, the differences, the
     closed list of accepted equivalences and the conversion's own provenance;
  8. it RAISES on any canonical difference or any recorded problem, so the
     production run fails.

What rev7 corrects
------------------
Under rev6 the learning-rate observation was computed AFTER ``compare`` had
already fixed ``exact``, and neither this file nor the runner's R9 predicate
required it to match. Because ``_reapply_optimizer_learning_rate`` overwrites
``param_groups`` lr with the stage's configured value before the capture, a
restore that had loaded the WRONG learning rate produced a perfectly clean
canonical walk - both sides read 1e-4 - and passed. rev6's own text lists a
changed lr among the corruption classes that must be caught, so that was a hole
in the contract, not merely in the code. rev7 moves the observation in front of
the verdict and makes a mismatch an explicit problem that sets ``exact`` false.

What this file does NOT do
--------------------------
It does not emulate the restore. After installing the wrap it sets ``sys.argv``
and calls ``train_ppo_mlp.main()``, which reaches ``run()`` and the real
``algo.restore_from_path``. It constructs no Algorithm of its own, calls
``ray.init`` itself never, and reimplements no RLlib code path.

It modifies NO PRODUCTION FILE. The rebinding is an in-memory attribute of the
imported module and lives and dies with this process. The only file it writes is
the audit evidence, which goes into the validation leaf of this stage - and that
leaf physically sits under ``baseline_MLP/validation/``, so the accurate claim
is the narrow one rev3 required: no production module, config or artefact is
written.

Governed by v26c_j20_prereg_restore_audit_rev7.json, which amends rev6 > rev5 >
rev4 > rev3 > rev2 > rev1 > base.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import pathlib
import pickle
import sys

HERE = pathlib.Path(__file__).resolve().parent
BASELINE = HERE.parent.parent

OPTIMIZER_ENTRY = "default_policy_default_optimizer"
MODULE_ID = "default_policy"
EXPECTED_STATE_INDICES = (6, 7, 8, 9, 10, 11)
EXPECTED_PARAM_COUNT = 12
EXPECTED_OPTIMIZER_NAME = "default_optimizer"
MOMENT_KEYS = ("exp_avg", "exp_avg_sq")
EVIDENCE_KIND = "LIVE OPTIMIZER RESTORE AUDIT, CANONICALISED"
STAGE_MARKER = "after_restore"

# The literal that must still be in TorchLearner._set_optimizer_state. If RLlib
# ever stops taking this path, canonicalising through it would be canonicalising
# through a fiction, so the audit refuses instead.
REQUIRED_CALL_SITE = \
    'convert_to_torch_tensor(state_dict["state"], device=self._device)'

# The CLOSED list of paths rev6 accepts as representational. It is exactly the
# eight R2 measured. A ninth is a change in the object under audit, not a
# convenience, and is recorded as a problem.
ACCEPTED_EQUIVALENCE_PATHS = (
    "param_groups[0].amsgrad",
    "param_groups[0].betas[0]",
    "param_groups[0].betas[1]",
    "param_groups[0].capturable",
    "param_groups[0].decoupled_weight_decay",
    "param_groups[0].differentiable",
    "param_groups[0].eps",
    "param_groups[0].maximize",
)
# The eight strings R2's evidence holds, verbatim. raw.differences must
# reproduce them, which is what makes R3 a superset of R2 rather than a
# replacement of it.
R2_RAW_DIFFERENCE_STRINGS = (
    "optimizer.param_groups[0].amsgrad: prim vs scalar",
    "optimizer.param_groups[0].betas[0]: (0.9,) vs (0.8999999761581421,)",
    "optimizer.param_groups[0].betas[1]: (0.999,) vs (0.9990000128746033,)",
    "optimizer.param_groups[0].capturable: prim vs scalar",
    "optimizer.param_groups[0].decoupled_weight_decay: prim vs scalar",
    "optimizer.param_groups[0].differentiable: prim vs scalar",
    "optimizer.param_groups[0].eps: (1e-08,) vs (9.99999993922529e-09,)",
    "optimizer.param_groups[0].maximize: prim vs scalar",
)

WHOLE_TREE_ROOT = "optimizer"
PARAM_GROUPS_ROOT = "param_groups"

# Node kinds that hold a value rather than children. Two vocabularies live here:
# rev2's raw one (prim, scalar, tensor) and rev6's canonical one (none, str, t).
# One walker serves both, so a difference is reported the same way in each.
LEAF_KINDS = ("prim", "scalar", "tensor", "none", "str", "t")


class LiveOptimizerMismatch(RuntimeError):
    """Raised when the live optimizer does not match the checkpoint."""


# --------------------------------------------- rev2's normalisation, verbatim --

def normalise(value):
    """rev2's canonical form. Reproduced here so R3 can record RAW provenance.

    This is the function whose output produced R2's eight differences. It is
    kept BYTE-FOR-BYTE equivalent to the frozen wrapper's, deliberately: R3 has
    to be able to show the same raw picture R2 saw, otherwise 'the mismatch was
    representational' would be an assertion instead of a measurement.

    The rules are rev2's, in rev2's order: bool before int; None; str; every
    numeric scalar and every zero-dimensional array or tensor to ('scalar',
    value); rank >= 1 to a dtype/shape/bytes triple; sequences; mappings; and
    anything unrecognised to ('opaque', type name), which can never equal a real
    value on the other side.
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
            return ("scalar", base.item())
        array = np.ascontiguousarray(base)
        return ("tensor", dtype, tuple(int(d) for d in array.shape),
                hashlib.sha256(array.tobytes(order="C")).hexdigest())

    if isinstance(value, (list, tuple)):
        return ("seq", [normalise(item) for item in value])
    if isinstance(value, dict):
        return ("map", [[normalise(key), normalise(value[key])]
                        for key in sorted(value, key=repr)])
    return ("opaque", type(value).__name__)


# ------------------------------------------------- rev6's canonicalisation --

def rllib_conversion_provenance() -> dict:
    """Everything about the conversion this audit canonicalises through.

    Recorded rather than assumed, and checked rather than trusted: if the call
    site inside ``_set_optimizer_state`` no longer holds the literal we
    canonicalise through, or if ``convert_to_torch_tensor`` no longer takes a
    device, the audit fails closed instead of comparing against a path RLlib no
    longer takes.
    """
    import numpy
    import ray
    import torch
    from ray.rllib.core.learner.torch.torch_learner import TorchLearner
    from ray.rllib.utils.torch_utils import convert_to_torch_tensor

    convert_source = inspect.getsource(convert_to_torch_tensor)
    setter_source = inspect.getsource(TorchLearner._set_optimizer_state)
    torch_utils_file = pathlib.Path(inspect.getsourcefile(
        convert_to_torch_tensor)).resolve()
    learner_file = pathlib.Path(inspect.getsourcefile(
        TorchLearner._set_optimizer_state)).resolve()
    parameters = list(inspect.signature(convert_to_torch_tensor)
                      .parameters.keys())
    return {
        "ray_version": str(ray.__version__),
        "torch_version": str(torch.__version__),
        "numpy_version": str(numpy.__version__),
        "torch_utils_file": str(torch_utils_file),
        "torch_utils_file_sha256": hashlib.sha256(
            torch_utils_file.read_bytes()).hexdigest(),
        "torch_learner_file": str(learner_file),
        "torch_learner_file_sha256": hashlib.sha256(
            learner_file.read_bytes()).hexdigest(),
        "convert_to_torch_tensor_source_sha256": hashlib.sha256(
            convert_source.encode("utf-8")).hexdigest(),
        "set_optimizer_state_source_sha256": hashlib.sha256(
            setter_source.encode("utf-8")).hexdigest(),
        "convert_to_torch_tensor_parameters": parameters,
        "required_call_site": REQUIRED_CALL_SITE,
        "call_site_verified": REQUIRED_CALL_SITE in setter_source,
        "device_parameter_present": "device" in parameters,
        "downcast_clause_present":
            "tensor.is_floating_point()" in convert_source
            and "tensor.float()" in convert_source,
        "none_passthrough_clause_present":
            "if item is None:" in convert_source,
    }


def rllib_convert(payload, device):
    """RLlib's own conversion, imported. Never a reimplementation of it."""
    from ray.rllib.utils.torch_utils import convert_to_torch_tensor

    return convert_to_torch_tensor(payload, device=device)


def canonical_key(key):
    """A dict key, typed, so an int key can never collide with a str key."""
    if isinstance(key, bool):                # before int, as everywhere here
        return ("key", "bool", key)
    if key is None or isinstance(key, (int, float, str)):
        return ("key", type(key).__name__, key)
    return ("key", type(key).__name__, repr(key))


def canonicalise(value):
    """The post-conversion canonical form of rev6.

    Every leaf is a tensor by the time it gets here, because that is what
    ``convert_to_torch_tensor`` produces from everything except ``None``. A
    tensor becomes ('t', dtype, shape, sha256 of its C-order bytes) whatever its
    rank, INCLUDING rank zero - which is the whole point. rev2 had to collapse
    0-d tensors to bare scalars so that the checkpoint's Python float step could
    equal the live 0-d tensor step; after canonicalisation both sides hold a 0-d
    tensor, so the collapse is no longer needed and the dtype it used to discard
    is exactly what tells a bool apart from a numeric zero.

    Nothing here compares numbers. Identity is dtype, shape and bytes.
    """
    if value is None:
        return ("none",)
    if isinstance(value, str):
        return ("str", value)

    detach = getattr(value, "detach", None)
    if detach is not None and hasattr(value, "cpu") and hasattr(value, "shape"):
        detached = value.detach().cpu().contiguous()
        shape = tuple(int(d) for d in detached.shape)   # read from the tensor,
        dtype = str(detached.dtype).replace("torch.", "")  # never from a
        payload = detached.numpy().tobytes(order="C")   # promoted array
        return ("t", dtype, shape,
                hashlib.sha256(payload).hexdigest())

    if isinstance(value, (list, tuple)):
        return ("seq", [canonicalise(item) for item in value])
    if isinstance(value, dict):
        return ("map", [[canonical_key(key), canonicalise(value[key])]
                        for key in sorted(value, key=repr)])
    return ("opaque", type(value).__name__)


def canonicalise_converted(payload, device):
    """Convert with RLlib's function, then canonicalise. One step, one meaning."""
    return canonicalise(rllib_convert(payload, device))


def is_fixed_point(payload, device) -> bool:
    """convert(convert(x)) must canonicalise to the same tree as convert(x).

    If it does not, then 'both sides under the same semantics' is not a
    well-defined statement and the audit has no business passing.
    """
    once = rllib_convert(payload, device)
    twice = rllib_convert(once, device)
    return canonicalise(once) == canonicalise(twice)


def canonical_digest(tree) -> str:
    """A stable sha256 over a whole normalised or canonical tree.

    repr() of either tree is deterministic: both contain only tuples, lists,
    strings, bools, None and numbers, and Python's float repr is the shortest
    round-tripping form. Two trees with the same digest are the same tree.
    """
    return hashlib.sha256(repr(tree).encode("utf-8")).hexdigest()


# ----------------------------------------------------------- the comparison --

def differences(left, right, path: str = "") -> list:
    """Every place two trees disagree, as records with a readable path.

    Serves rev2's raw vocabulary and rev6's canonical one alike; the leaf kinds
    of both are listed in LEAF_KINDS. Recursive rather than a single ``==`` so a
    failure names the key that broke instead of only reporting that something
    did. Each record carries the path and the same detail text the frozen
    wrapper produced, so the raw list can be rendered byte-for-byte as R2's.
    """
    where = path or "<root>"
    if type(left) is not type(right):
        return [{"path": where,
                 "detail": "kind %s vs %s" % (type(left).__name__,
                                              type(right).__name__),
                 "source": repr(left), "live": repr(right)}]
    if isinstance(left, tuple) and left and isinstance(left[0], str):
        kind_left, kind_right = left[0], right[0]
        if kind_left != kind_right:
            return [{"path": where,
                     "detail": "%s vs %s" % (kind_left, kind_right),
                     "source": repr(left), "live": repr(right)}]
        if kind_left in LEAF_KINDS:
            if left == right:
                return []
            return [{"path": where,
                     "detail": "%r vs %r" % (left[1:], right[1:]),
                     "source": repr(left), "live": repr(right)}]
        if kind_left == "seq":
            items_left, items_right = left[1], right[1]
            if len(items_left) != len(items_right):
                return [{"path": where,
                         "detail": "length %d vs %d" % (len(items_left),
                                                        len(items_right)),
                         "source": repr(len(items_left)),
                         "live": repr(len(items_right))}]
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
                return [{"path": where,
                         "detail": "key sets differ; only source %r, only live "
                                   "%r" % (only_left, only_right),
                         "source": repr(only_left), "live": repr(only_right)}]
            found = []
            for (key, a), (_, b) in zip(left[1], right[1]):
                label = key[-1] if isinstance(key, tuple) else key
                found += differences(a, b, "%s.%s" % (where, label))
            return found
        return [{"path": where, "detail": "opaque value",
                 "source": repr(left), "live": repr(right)}]
    if left == right:
        return []
    return [{"path": where, "detail": "%r vs %r" % (left, right),
             "source": repr(left), "live": repr(right)}]


def as_strings(records) -> list:
    """The frozen wrapper's exact rendering: '<path>: <detail>'."""
    return ["%s: %s" % (record["path"], record["detail"]) for record in records]


def strip_root(path: str) -> str:
    """'optimizer.param_groups[0].eps' -> 'param_groups[0].eps'."""
    prefix = WHOLE_TREE_ROOT + "."
    return path[len(prefix):] if path.startswith(prefix) else path


# --------------------------------------------------------------- the capture --

def capture_live_optimizer(learner) -> list:
    """Read the live optimizers of the module, raw AND canonical.

    Runs on the learner and returns plain data only - strings, numbers, lists
    and dicts, never a tensor - so the same code is correct whether
    ``foreach_learner`` executes locally or on a Ray actor. The traversal is the
    production one, ``get_optimizers_for_module``, unchanged from rev1.

    The device is the learner's OWN ``_device``, so the canonical form is
    produced on the device the restore actually used rather than on a device
    this audit chose.
    """
    device = str(getattr(learner, "_device", "cpu"))
    captured = []
    for name, optimizer in learner.get_optimizers_for_module(MODULE_ID):
        state = optimizer.state_dict()
        raw = normalise(state)
        canonical = canonicalise_converted(state, device)
        converted = rllib_convert(state, device)
        captured.append({
            "optimizer_name": str(name),
            "optimizer_type": type(optimizer).__name__,
            "device": device,
            "normalised": raw,
            "normalised_param_groups": normalise(state.get("param_groups", [])),
            "canonical": canonical,
            "canonical_param_groups": canonicalise_converted(
                state.get("param_groups", []), device),
            "fixed_point": is_fixed_point(state, device),
            "top_level_keys": sorted(state),
            "state_indices": sorted(int(k) for k in state.get("state", {})),
            "param_group_sizes": [len(group.get("params", []))
                                  for group in state.get("param_groups", [])],
            "raw_moment_digests": moment_digest_map(state, normalise),
            "canonical_moment_digests": moment_digest_map(converted,
                                                          canonicalise),
            "raw_leaf_values": leaf_values(state),
        })
    if not captured:
        raise LiveOptimizerMismatch(
            "no optimizer is registered for %s on the live learner" % MODULE_ID)
    return captured


def read_source_optimizer(source_state: pathlib.Path) -> dict:
    """The checkpoint's optimizer state_dict, straight off the pickle.

    This object is precisely the argument RLlib hands to
    ``convert_to_torch_tensor``: ``_set_optimizer_state`` iterates the learner
    state's ``optimizer`` mapping and converts ``entry['state']``.
    """
    with source_state.open("rb") as handle:
        payload = pickle.load(handle)
    entry = (payload.get("optimizer") or {}).get(OPTIMIZER_ENTRY)
    if not isinstance(entry, dict) or "state" not in entry:
        raise LiveOptimizerMismatch(
            "%s holds no %s entry with a state_dict" % (source_state,
                                                        OPTIMIZER_ENTRY))
    return entry["state"]


def moment_digest_map(state, reducer) -> dict:
    """Per-index digest of exp_avg and exp_avg_sq under a given reducer."""
    entries = state.get("state", {}) if isinstance(state, dict) else {}
    return {
        str(index): {key: reducer(entries[index][key])
                     for key in MOMENT_KEYS if key in entries[index]}
        for index in sorted(entries, key=lambda k: int(k))
    }


def moment_identity(digests: dict) -> dict:
    """dtype/shape/sha triples only, so raw and canonical can be compared.

    A raw moment reads ('tensor', dtype, shape, sha) and a canonical one reads
    ('t', dtype, shape, sha). Dropping the tag makes the two directly
    comparable, which is how 'canonicalisation left the moments alone' becomes a
    measurement rather than a claim.
    """
    return {index: {key: list(node[1:]) for key, node in keys.items()}
            for index, keys in digests.items()}


def leaf_values(payload, path: str = "") -> dict:
    """Path -> (type name, repr) for every leaf, as read BEFORE conversion.

    This is the raw provenance rev6 requires: a future reader can see what the
    two sides actually held without re-running anything.
    """
    where = path or WHOLE_TREE_ROOT
    out = {}
    if isinstance(payload, dict):
        for key in sorted(payload, key=repr):
            out.update(leaf_values(payload[key], "%s.%s" % (where, key)))
        return out
    if isinstance(payload, (list, tuple)):
        for index, item in enumerate(payload):
            out.update(leaf_values(item, "%s[%d]" % (where, index)))
        return out
    text = repr(payload)
    if len(text) > 120:                       # a moment tensor, not a scalar
        text = text[:117] + "..."
    out[where] = [type(payload).__name__, text]
    return out


# -------------------------------------------------------------- the verdict --

def compare(source_state: pathlib.Path, captured: list) -> dict:
    """Compare the live optimizer against the checkpoint, canonically.

    Two comparisons are performed and both are recorded. The RAW one is rev2's,
    unchanged, and is expected to reproduce R2's eight differences. The
    CANONICAL one passes both sides through RLlib's own conversion first and is
    the one gate R9 is decided on. A path that differs raw and agrees
    canonically is an ACCEPTED EQUIVALENCE, and the set of them is closed: it
    must be exactly the eight rev6 lists.
    """
    problems: list = []
    if len(captured) != 1:
        problems.append("expected exactly one learner, captured %d"
                        % len(captured))
    live_entries = [entry for learner in captured for entry in learner] \
        if captured and isinstance(captured[0], list) else list(captured)
    if len(live_entries) != 1:
        problems.append("expected exactly one optimizer, captured %d"
                        % len(live_entries))
    if not live_entries:
        raise LiveOptimizerMismatch("no live optimizer was captured")
    live = live_entries[0]
    device = live.get("device") or "cpu"

    provenance = rllib_conversion_provenance()
    if not provenance["call_site_verified"]:
        problems.append(
            "TorchLearner._set_optimizer_state no longer contains %r, so "
            "canonicalising through convert_to_torch_tensor would compare "
            "against a path RLlib does not take" % REQUIRED_CALL_SITE)
    if not provenance["device_parameter_present"]:
        problems.append("convert_to_torch_tensor no longer takes a device "
                        "parameter; its parameters are %s"
                        % provenance["convert_to_torch_tensor_parameters"])

    source = read_source_optimizer(source_state)
    raw_source = normalise(source)
    raw_source_pg = normalise(source.get("param_groups", []))
    canonical_source = canonicalise_converted(source, device)
    canonical_source_pg = canonicalise_converted(
        source.get("param_groups", []), device)
    converted_source = rllib_convert(source, device)
    fixed_point_source = is_fixed_point(source, device)
    if not fixed_point_source:
        problems.append("the conversion is not a fixed point on the checkpoint "
                        "side, so 'both sides under the same semantics' is not "
                        "well defined")
    if not live.get("fixed_point"):
        problems.append("the conversion is not a fixed point on the live side, "
                        "so 'both sides under the same semantics' is not well "
                        "defined")

    raw_found = differences(raw_source, live["normalised"], WHOLE_TREE_ROOT)
    raw_pg_found = differences(raw_source_pg, live["normalised_param_groups"],
                               PARAM_GROUPS_ROOT)
    canonical_found = differences(canonical_source, live["canonical"],
                                  WHOLE_TREE_ROOT)
    canonical_pg_found = differences(canonical_source_pg,
                                     live["canonical_param_groups"],
                                     PARAM_GROUPS_ROOT)

    raw_paths = {strip_root(record["path"]) for record in raw_found}
    canonical_paths = {strip_root(record["path"]) for record in canonical_found}
    source_leaves = leaf_values(source)
    live_leaves = live.get("raw_leaf_values") or {}
    accepted = []
    for record in raw_found:
        path = strip_root(record["path"])
        if path in canonical_paths:
            continue
        accepted.append({
            "path": path,
            "raw_detail": record["detail"],
            "raw_source_leaf": source_leaves.get(WHOLE_TREE_ROOT + "." + path),
            "raw_live_leaf": live_leaves.get(WHOLE_TREE_ROOT + "." + path),
            "canonical_node_source": repr(node_at(canonical_source, path)),
            "canonical_node_live": repr(node_at(live["canonical"], path)),
        })
    accepted_paths = sorted(item["path"] for item in accepted)
    if accepted_paths != sorted(ACCEPTED_EQUIVALENCE_PATHS):
        problems.append(
            "the accepted equivalences are %s, and rev6 closes that list at %s"
            % (accepted_paths, sorted(ACCEPTED_EQUIVALENCE_PATHS)))
    unexplained = sorted(canonical_paths - raw_paths)
    if unexplained:
        problems.append("these paths agree raw and differ canonically, which "
                        "cannot be representational: %s" % unexplained)

    raw_strings = as_strings(raw_found)
    if sorted(raw_strings) != sorted(R2_RAW_DIFFERENCE_STRINGS):
        problems.append(
            "the raw differences no longer reproduce R2's eight: found %s"
            % raw_strings)

    source_indices = sorted(int(k) for k in source.get("state", {}))
    source_top = sorted(source)
    if sorted(live["top_level_keys"]) != ["param_groups", "state"]:
        problems.append("live top-level keys are %s, expected "
                        "['param_groups', 'state']" % live["top_level_keys"])
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
                        % (live.get("optimizer_name"), EXPECTED_OPTIMIZER_NAME))

    raw_moments_source = moment_digest_map(source, normalise)
    canonical_moments_source = moment_digest_map(converted_source, canonicalise)
    canonical_moments_live = live["canonical_moment_digests"]
    moments_match = canonical_moments_source == canonical_moments_live
    if not moments_match:
        problems.append("the canonical moment digests differ between the "
                        "checkpoint and the live optimizer")
    source_moments_untouched = \
        moment_identity(raw_moments_source) == \
        moment_identity(canonical_moments_source)
    live_moments_untouched = \
        moment_identity(live["raw_moment_digests"]) == \
        moment_identity(canonical_moments_live)
    if not source_moments_untouched or not live_moments_untouched:
        problems.append("canonicalisation altered a moment tensor, which means "
                        "it overreached: source untouched %s, live untouched %s"
                        % (source_moments_untouched, live_moments_untouched))

    digests = {
        "raw": {
            "digest_source": canonical_digest(raw_source),
            "digest_live": canonical_digest(live["normalised"]),
            "param_groups_digest_source": canonical_digest(raw_source_pg),
            "param_groups_digest_live": canonical_digest(
                live["normalised_param_groups"]),
        },
        "canonical": {
            "digest_source": canonical_digest(canonical_source),
            "digest_live": canonical_digest(live["canonical"]),
            "param_groups_digest_source": canonical_digest(canonical_source_pg),
            "param_groups_digest_live": canonical_digest(
                live["canonical_param_groups"]),
        },
    }
    canonical_match = digests["canonical"]["digest_source"] == \
        digests["canonical"]["digest_live"]
    if not canonical_match and not canonical_found:
        problems.append("the canonical digests differ although the recursive "
                        "walk found nothing, which cannot both be true")
    if canonical_found and canonical_match:
        problems.append("the canonical walk found differences although the "
                        "digests agree, which cannot both be true")

    exact = (not canonical_found) and (not canonical_pg_found) \
        and canonical_match and not problems
    return {
        "kind": EVIDENCE_KIND,
        "stage_marker": STAGE_MARKER,
        "exact": exact,
        "problems": problems,
        "device": device,
        "conversion": provenance,
        "conversion_fixed_point_source": fixed_point_source,
        "conversion_fixed_point_live": bool(live.get("fixed_point")),
        "raw": {
            "semantics": "v26c_j20_prereg_restore_audit_rev2.json, unchanged",
            "exact": not raw_found,
            "differences": raw_strings,
            "difference_count": len(raw_found),
            "difference_records": raw_found,
            "digest_source": digests["raw"]["digest_source"],
            "digest_live": digests["raw"]["digest_live"],
            "digests_match": digests["raw"]["digest_source"]
            == digests["raw"]["digest_live"],
            "reproduces_r2_eight":
                sorted(raw_strings) == sorted(R2_RAW_DIFFERENCE_STRINGS),
            "param_groups": {
                "exact": not raw_pg_found,
                "differences": as_strings(raw_pg_found),
                "difference_count": len(raw_pg_found),
                "digest_source": digests["raw"]["param_groups_digest_source"],
                "digest_live": digests["raw"]["param_groups_digest_live"],
                "digests_match":
                    digests["raw"]["param_groups_digest_source"]
                    == digests["raw"]["param_groups_digest_live"],
            },
        },
        "canonical": {
            "semantics": "both sides through "
                         "ray.rllib.utils.torch_utils.convert_to_torch_tensor, "
                         "imported from the installed library, then reduced to "
                         "dtype, shape and the sha256 of the C-order bytes of "
                         "every leaf. No tolerance of any kind.",
            "exact": not canonical_found,
            "differences": as_strings(canonical_found),
            "difference_count": len(canonical_found),
            "difference_records": canonical_found,
            "digest_source": digests["canonical"]["digest_source"],
            "digest_live": digests["canonical"]["digest_live"],
            "digests_match": canonical_match,
            "param_groups": {
                "exact": not canonical_pg_found,
                "differences": as_strings(canonical_pg_found),
                "difference_count": len(canonical_pg_found),
                "digest_source":
                    digests["canonical"]["param_groups_digest_source"],
                "digest_live":
                    digests["canonical"]["param_groups_digest_live"],
                "digests_match":
                    digests["canonical"]["param_groups_digest_source"]
                    == digests["canonical"]["param_groups_digest_live"],
            },
        },
        "accepted_equivalences": accepted,
        "accepted_equivalence_paths": accepted_paths,
        "expected_accepted_equivalence_paths":
            sorted(ACCEPTED_EQUIVALENCE_PATHS),
        "accepted_equivalences_are_exactly_the_expected_eight":
            accepted_paths == sorted(ACCEPTED_EQUIVALENCE_PATHS),
        "unexplained_canonical_paths": unexplained,
        "moment_digests_source": canonical_moments_source,
        "moment_digests_live": canonical_moments_live,
        "moment_digests_match": moments_match,
        "raw_moment_digests_source": raw_moments_source,
        "raw_moment_digests_live": live["raw_moment_digests"],
        "note_on_moment_dtype": "a moment stored as float64 would be downcast "
                                "to float32 by the conversion, so the canonical "
                                "walk alone could not see it. That is why the "
                                "two moments_unchanged_by_canonicalisation "
                                "flags exist: they compare each side's RAW "
                                "dtype/shape/bytes against its canonical ones "
                                "and fail closed if the conversion moved a "
                                "moment at all.",
        "moments_unchanged_by_canonicalisation_source": source_moments_untouched,
        "moments_unchanged_by_canonicalisation_live": live_moments_untouched,
        "raw_leaf_values_source": source_leaves,
        "raw_leaf_values_live": live_leaves,
        "learner_count": len(captured),
        "optimizer_names": [entry.get("optimizer_name")
                            for entry in live_entries],
        "optimizer_types": [entry.get("optimizer_type")
                            for entry in live_entries],
        "expected_optimizer_name": EXPECTED_OPTIMIZER_NAME,
        "source_state_pkl": str(source_state),
        "source_state_sha256": hashlib.sha256(
            source_state.read_bytes()).hexdigest(),
        "top_level_keys_source": source_top,
        "top_level_keys_live": live["top_level_keys"],
        "state_indices_source": source_indices,
        "state_indices_live": live["state_indices"],
        "expected_state_indices": list(EXPECTED_STATE_INDICES),
        "expected_param_count": EXPECTED_PARAM_COUNT,
        "param_group_sizes_live": live["param_group_sizes"],
        "method":
            "the original _reapply_optimizer_learning_rate was called first; "
            "the live optimizer was then read through get_optimizers_for_module "
            "on the learner; both sides were recorded RAW under rev2's "
            "normalisation and then passed through RLlib's own "
            "convert_to_torch_tensor, the same call TorchLearner."
            "_set_optimizer_state makes, and compared recursively and exactly "
            "on dtype, shape and the sha256 of the C-order bytes of every leaf; "
            "the conversion was checked to be a fixed point on both sides; the "
            "paths that differ raw and agree canonically were recorded as a "
            "closed list of accepted equivalences with both raw values",
    }


def node_at(tree, path: str):
    """Follow a canonical path such as 'param_groups[0].eps' into a tree."""
    node = tree
    token = ""
    index = 0
    while index < len(path):
        character = path[index]
        if character == ".":
            node = descend_key(node, token)
            token = ""
        elif character == "[":
            node = descend_key(node, token) if token else node
            token = ""
            close = path.index("]", index)
            node = descend_index(node, int(path[index + 1:close]))
            index = close
        else:
            token += character
        index += 1
    return descend_key(node, token) if token else node


def descend_key(node, label):
    """One step into a ('map', ...) node by the printed form of its key."""
    if not label:
        return node
    if not (isinstance(node, tuple) and node and node[0] == "map"):
        return None
    for key, value in node[1]:
        if str(key[-1] if isinstance(key, tuple) else key) == label:
            return value
    return None


def descend_index(node, position):
    """One step into a ('seq', ...) node."""
    if not (isinstance(node, tuple) and node and node[0] == "seq"):
        return None
    items = node[1]
    return items[position] if 0 <= position < len(items) else None


def learning_rate_observation(source_state: pathlib.Path, reports, device,
                              learning_rate=None) -> dict:
    """The pre-overwrite learning rate, measured instead of merely declared.

    rev1 requires the original ``_reapply_optimizer_learning_rate`` to run
    before the capture, which overwrites ``param_groups`` lr with the stage's
    configured value and therefore removes lr from the comparison inside the
    main tree. rev1 and rev2 both declared that openly.

    R3 does not change that ordering. It reads what production itself reports as
    the value BEFORE the overwrite - the ``before`` field of
    ``_set_optimizer_learning_rate_on_learner`` - and compares it against the
    checkpoint's lr under the SAME canonical semantics as everything else.

    rev7 makes this a GATE rather than a note. Under rev6 the observation was
    computed after ``compare`` had already fixed ``exact``, and neither the
    wrapper nor the runner required ``matches``. A restore that produced the
    wrong learning rate was therefore erased by the production overwrite and
    could still pass: the canonical walk compares the checkpoint's lr against
    the value the stage reapplied, so both sides read 1e-4 whatever the restore
    actually loaded. This field is the ONLY place the restored value is visible,
    so it has to be enforced here or the changed-lr corruption class is not
    covered at all.

    Every failure mode is fail-closed: a missing report, a missing checkpoint
    lr, a value that will not canonicalise, or a canonical mismatch all yield
    ``matches`` False.
    """
    source = read_source_optimizer(source_state)
    groups = source.get("param_groups") or [{}]
    source_lr = groups[0].get("lr")
    before = None
    reports_seen = 0
    for learner_reports in (reports or []):
        for report in (learner_reports or []):
            reports_seen += 1
            if report.get("optimizer_name") == EXPECTED_OPTIMIZER_NAME:
                before = report.get("before")
    canonical_source = canonicalise_converted(source_lr, device) \
        if source_lr is not None else None
    canonical_before = canonicalise_converted(before, device) \
        if before is not None else None
    matches = (canonical_source is not None and canonical_before is not None
               and canonical_source == canonical_before)
    return {
        "source_lr_raw": repr(source_lr),
        "source_lr_type": type(source_lr).__name__,
        "live_lr_before_reapply": before,
        "live_lr_before_reapply_type": type(before).__name__,
        "production_reports_seen": reports_seen,
        "reapplied_learning_rate": None if learning_rate is None
        else float(learning_rate),
        # Null, not the string 'None', when absent: a runner predicate that
        # tests for a populated canonical field must be able to tell the
        # difference between a value and its absence.
        "canonical_source_lr": repr(canonical_source)
        if canonical_source is not None else None,
        "canonical_live_lr_before_reapply": repr(canonical_before)
        if canonical_before is not None else None,
        "canonical_dtype": canonical_source[1]
        if canonical_source is not None else None,
        "matches": bool(matches),
        "gated": True,
        "note": "the comparison of param_groups lr inside the main tree is "
                "between the checkpoint and the value the stage reapplied, "
                "because rev1 requires the original to run first. This field "
                "records the restored value BEFORE that overwrite, and rev7 "
                "requires it to match: it is the only witness of the restored "
                "learning rate.",
    }


def enforce_learning_rate_observation(verdict: dict,
                                      source_state: pathlib.Path,
                                      reports, learning_rate) -> dict:
    """Compute the lr observation, attach it, and FAIL CLOSED on a mismatch.

    Separated from the wrap so it can be exercised directly by the test suite,
    without a learner and without an Algorithm. It mutates the verdict in place
    and returns it, so the caller's write-then-raise path is unchanged: the
    evidence is still written before anything is raised.
    """
    device = verdict.get("device") or "cpu"
    observation = learning_rate_observation(source_state, reports, device,
                                            learning_rate)
    verdict["learning_rate_observation"] = observation
    if observation.get("matches") is not True:
        verdict.setdefault("problems", []).append(
            "the learning rate the restore produced does not match the "
            "checkpoint under the canonical semantics: checkpoint %s, live "
            "before the production reapply %r (%s). The reapply overwrites "
            "param_groups lr before this capture, so the main tree cannot see "
            "this and rev7 gates it here."
            % (observation.get("canonical_source_lr"),
               observation.get("live_lr_before_reapply"),
               observation.get("canonical_live_lr_before_reapply")))
        verdict["exact"] = False
    return verdict


def write_evidence(path: pathlib.Path, payload: dict) -> None:
    """Canonical JSON. Written on mismatch too, so evidence survives failure."""
    path.write_bytes(json.dumps(payload, indent=2, sort_keys=True,
                                allow_nan=False, default=str).encode("utf-8"))


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
        # rev7: computed and GATED before the evidence is written, so a wrong
        # restored learning rate cannot be erased by the production overwrite.
        enforce_learning_rate_observation(verdict, source_state, reports,
                                          learning_rate)
        write_evidence(evidence, verdict)
        if not verdict["exact"]:
            first = (verdict["canonical"]["differences"]
                     or verdict["problems"] or ["-"])[0]
            raise LiveOptimizerMismatch(
                "the live optimizer does not match the checkpoint under "
                "RLlib's own conversion semantics: %d canonical difference(s), "
                "%d problem(s). First: %s"
                % (verdict["canonical"]["difference_count"],
                   len(verdict["problems"]), first))
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
        description="V26C J20 R3 restore audit - validation-only child wrapper")
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
