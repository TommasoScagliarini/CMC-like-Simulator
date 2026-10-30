"""V26C J18 - J8 B-only constrained update.

Readiness artefact. Building the dataset and validating it against the pinned
manifest requires only numpy; ``torch`` is imported lazily inside the fit and is
therefore never loaded by ``--preflight-only`` or ``--dry-run``. The
execution-path tests do load it.

The parent actor is bound by the ACTUAL BYTES of its ``module_state.pkl``. The
J8 leaf sidecar ``actor_feature_manifest.json`` is stale - it is byte-identical
to J2's and declares J2's module hash - and is never read as an identity source.
See ``v26c_j18_provenance_overlay_j8_2026-08-27.json``.

Usage:
    python v26c_j18_b_only_update.py --preflight-only
    python v26c_j18_b_only_update.py --dry-run
    python v26c_j18_b_only_update.py --execute --go-file <architect GO json>

``--preflight-only`` and ``--dry-run`` write nothing and create no directory.
``--execute`` refuses to start without a valid architect GO file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import pickle
import sys

import numpy as np

# --------------------------------------------------------------------------
# Immutable locations. Everything is resolved relative to this file so the
# module behaves identically on Windows x86 and macOS arm64.
# --------------------------------------------------------------------------

VALIDATION_ROOT = pathlib.Path(__file__).resolve().parent

PARENT_MODULE_PATH = (
    "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1/rl_module/module_state.pkl"
)
PARENT_MODULE_SHA256 = (
    "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82"
)
PARENT_STALE_MANIFEST_PATH = (
    "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1/rl_module/actor_feature_manifest.json"
)
J2_MODULE_SHA256 = (
    "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
)

J9R1_DIR = "j9r1_runs/j9r1_closed_loop_v26c_2026-08-26_r1"
J10R1_TEACHER_PATH = (
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/"
    "j10r1_cell_B_teacher_dataset.npz"
)
J7_DATASET_PATH = (
    "j7_runs/j7_markov_dataset_v26c_2026-08-26_r1/"
    "v26c_j7_markov_recovery_dataset.npz"
)

MANIFEST_PATH = "v26c_j18_dataset_manifest.json"
PREREG_PATH = "v26c_j18_prereg_b_only_constrained_update.json"
ADDENDUM_PATH = "v26c_j18_prereg_addendum_v2_minibatch_and_scaling_2026-08-27.json"
ADDENDUM_V3_PATH = "v26c_j18_prereg_addendum_v3_scale_absorption_2026-08-27.json"
PROVENANCE_OVERLAY_PATH = "v26c_j18_provenance_overlay_j8_2026-08-27.json"

LEAF_ROOT = "j18_runs"
LEAF_NAME = "j18_b_only_update_v26c_2026-08-27_r1"
RECEIPT_NAME = "v26c_j18_b_only_update_receipt.json"
INVALID_MARKER = "TECHNICAL_INVALID"

# (W*s)@(o/s) and W@o are different float32 operation sequences, so bit-identity
# is not claimed. The measured deviation is 5.96e-08 in numpy and 1.19e-07 in
# torch, the latter being exactly the float32 epsilon.
RAW_VS_SCALED_TOLERANCE = 1e-06

TARGET_CELL = "B"
ANCHOR_CELLS = ("A", "C", "D", "E", "F")
BLOCK_PRIORITY = ("A", "B", "C", "D")

J7_NOMINAL_ROWS = 16000

EXPECTED_PREFIX_LENGTH = 15
EXPECTED_FIRST_MISMATCH_INDEX = 15
EXPECTED_FIRST_MISMATCH_COLUMN = 11
EXPECTED_BLOCK_ROWS = {"A": 500, "B": 14, "C": 2497, "D": 1210}
EXPECTED_TOTAL_ROWS = 4221

OBSERVATION_WIDTH = 35
ACTION_WIDTH = 2
ACTION_SCALE_RAD = 0.35
CLOCK_COLUMNS = (0, 1)

DIRECT_KEYS = (
    "pi.0.0.weight",
    "pi.0.0.bias",
    "pi.0.2.weight",
    "pi.0.2.bias",
    "pi.1.weight",
    "pi.1.bias",
)
ENCODER_ALIASES = {
    "pi_encoder.0.weight": "pi.0.0.weight",
    "pi_encoder.0.bias": "pi.0.0.bias",
    "pi_encoder.2.weight": "pi.0.2.weight",
    "pi_encoder.2.bias": "pi.0.2.bias",
}
CRITIC_KEY_MARKERS = ("vf", "value", "critic")

# ---------------------------------------------------------------------------
# Locally transcribed constants and predicates.
#
# J18 imports NO prior-stage executable module at run time. Depending on J8's,
# J11's or J14's code would make this runner's behaviour hostage to files that
# a J18 GO does not pin, so every borrowed rule is transcribed here and pinned
# by test against the values it must reproduce.
# ---------------------------------------------------------------------------

# warm_start.actor_state_digest, transcribed. Reproduces J8 = 6a879714... and
# J2 = 59d54240..., both asserted by the test suite.
WARM_START_ACTOR_KEYS = (
    "pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
    "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
    "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias",
)

# The registered July physical scales, in the registered order:
# (1, 1, 4, 60, 1, 1, 1, 3.5, 55, 1).
JULY_CONTROLLER_FEATURE_SCALES = (
    ("pros_knee_angle_previous_endpoint", 1.0),
    ("pros_knee_angle_served_ref", 1.0),
    ("pros_knee_angle_served_ref_vel", 4.0),
    ("pros_knee_angle_served_ref_accel", 60.0),
    ("pros_knee_angle_sea_u", 1.0),
    ("pros_ankle_angle_previous_endpoint", 1.0),
    ("pros_ankle_angle_served_ref", 1.0),
    ("pros_ankle_angle_served_ref_vel", 3.5),
    ("pros_ankle_angle_served_ref_accel", 55.0),
    ("pros_ankle_angle_sea_u", 1.0),
)

# J14's discrete-feature predicate, transcribed. On the current 35 names it must
# resolve to exactly [11, 12, 13, 17, 18, 19, 20, 21].
DISCRETE_NAME_SUFFIXES = ("_in_contact", "_heel_strike", "_toe_off", "_saturated")
DISCRETE_NAME_PREFIXES = ("phase_fsm_", "phase_expected_")

# The head emits four values: [mean_knee, mean_ankle, logstd_knee, logstd_ankle].
# There is NO separate log-std key. Freezing the log-std therefore means holding
# rows 2:4 of pi.1.weight and pi.1.bias, not excluding a named parameter.
HEAD_KEYS = ("pi.1.weight", "pi.1.bias")
MEAN_ROWS = slice(0, 2)
LOGSTD_ROWS = slice(2, 4)

COMPUTED_LABEL_TOLERANCE = 1e-06

# --------------------------------------------------------------------------
# Frozen fit protocol. Outside the grid; not tunable at run time.
# --------------------------------------------------------------------------

FIT_SEED = 123
FIT_EPOCHS = 200
FIT_BATCH_SIZE = 128
SEALED_SEEDS = (126, 127, 128)

GRID_LAMBDA = (1.0, 3.0, 10.0, 30.0)
GRID_BETA = (1.0, 5.0)
GRID_LEARNING_RATE = (1e-05, 5e-05)
WEIGHT_A = 1.0

# --------------------------------------------------------------------------
# Gates. Every threshold is derived from a measured baseline; see the
# preregistration for the derivation of each one.
# --------------------------------------------------------------------------

# Measured at candidate == parent, at full double precision. Every threshold
# below is DERIVED from these two numbers rather than written down separately,
# so a rounded literal can never drift away from the measurement it claims.
BASELINE_MSE_A = 0.004108013186414973
BASELINE_MSE_B = 0.1909311190718203

# Smallest maximum perturbation among the stochastic cells J8 actually passed.
SMALLEST_TOLERATED_PERTURBATION = 0.014557

GATE_MAX_DRIFT = 0.005
GATE_BIAS_DRIFT = SMALLEST_TOLERATED_PERTURBATION / 10.0
GATE_MSE_B_CEILING = BASELINE_MSE_B / 2.0
GATE_MSE_A_CEILING = BASELINE_MSE_A


# --------------------------------------------------------------------------
# Hashing helpers
# --------------------------------------------------------------------------


def sha256_file(path):
    """SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_array(array):
    """SHA-256 of a C-contiguous array's raw bytes."""
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def row_key(row):
    """Bit-exact identity of one observation row."""
    return bytes(np.ascontiguousarray(row))


# --------------------------------------------------------------------------
# Parent actor
# --------------------------------------------------------------------------


def load_parent_state(root=VALIDATION_ROOT):
    """Load J8's module state, binding to the actual bytes rather than the
    stale leaf manifest."""
    path = root / PARENT_MODULE_PATH
    actual = sha256_file(path)
    if actual != PARENT_MODULE_SHA256:
        raise RuntimeError(
            "parent module bytes do not match the pinned hash: "
            "expected %s, found %s" % (PARENT_MODULE_SHA256, actual)
        )
    with open(path, "rb") as handle:
        return pickle.load(handle)


def forward_mean(state, observations):
    """J8's mean head: two tanh layers then a linear head, first two columns.

    Computed in float64 and returned in float64; callers cast as needed.
    """
    x = np.asarray(observations, dtype=np.float64)
    for weight_key, bias_key in (
        ("pi.0.0.weight", "pi.0.0.bias"),
        ("pi.0.2.weight", "pi.0.2.bias"),
    ):
        weight = np.asarray(state[weight_key], dtype=np.float64)
        bias = np.asarray(state[bias_key], dtype=np.float64)
        x = np.tanh(x @ weight.T + bias)
    weight = np.asarray(state["pi.1.weight"], dtype=np.float64)
    bias = np.asarray(state["pi.1.bias"], dtype=np.float64)
    return (x @ weight.T + bias)[:, :ACTION_WIDTH]


def logstd_raw_slices(state):
    """The frozen log-std parameters, with NO dtype conversion.

    They are NOT a separate key. The output head has four rows and the last two
    carry the log-std, so the frozen quantity is a SLICE of pi.1.weight and
    pi.1.bias. A name-based search would find nothing and would make any
    equality gate built on it vacuously true.

    Nothing is cast here: G7 compares raw bytes, and a cast would erase exactly
    the differences it exists to catch.
    """
    return {key: np.asarray(state[key])[LOGSTD_ROWS] for key in HEAD_KEYS}


def logstd_tensors(state):
    """The frozen log-std parameters as float64, for arithmetic and reporting.

    NOT for equality: use ``bytes_identical`` on ``logstd_raw_slices`` instead.
    """
    return {
        key: np.asarray(state[key], dtype=np.float64)[LOGSTD_ROWS]
        for key in HEAD_KEYS
    }


def bytes_identical(left, right):
    """True only when dtype, shape and C-order bytes all match.

    ``np.array_equal`` is not byte identity: it compares numerically, so it sees
    float32 and float64 as equal and, more sharply, treats -0.0 as equal to
    +0.0. A log-std whose sign bit had flipped would slip through. This does
    not.
    """
    a = np.ascontiguousarray(left)
    b = np.ascontiguousarray(right)
    return bool(
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def logstd_sigma(state):
    """The standard deviation implied by the frozen log-std bias."""
    return np.exp(np.asarray(state["pi.1.bias"], dtype=np.float64)[LOGSTD_ROWS])


def logstd_is_state_independent(state):
    """True when the log-std rows of the head weight are exactly zero, so the
    log-std cannot depend on the observation."""
    rows = np.asarray(state["pi.1.weight"], dtype=np.float64)[LOGSTD_ROWS]
    return bool(np.all(rows == 0.0))


def critic_keys(state):
    """Any value-function key. J8 has none and none is ever created."""
    return tuple(
        sorted(
            k
            for k in state
            if any(marker in k.lower() for marker in CRITIC_KEY_MARKERS)
        )
    )


# --------------------------------------------------------------------------
# Raw input loading
# --------------------------------------------------------------------------


def load_trace(cell, root=VALIDATION_ROOT):
    """Load one J9R1 cell trace."""
    path = root / J9R1_DIR / ("j9_cell_%s_trace.json" % cell)
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def trace_observations(trace):
    """The 35D observation the actor saw BEFORE each step."""
    return np.array(
        [row["actor_observation_vector_before"] for row in trace],
        dtype=np.float32,
    )


def trace_policy_means(trace):
    """The mean action the actor emitted at each step."""
    return np.array([row["policy_mean"] for row in trace], dtype=np.float32)


def load_teacher(root=VALIDATION_ROOT):
    """The J10R1 cell-B teacher dataset."""
    with np.load(root / J10R1_TEACHER_PATH, allow_pickle=False) as bundle:
        return {
            "observations": bundle["observations"].astype(np.float32),
            "actions": bundle["actions"].astype(np.float32),
            "times": bundle["times"].astype(np.float64),
            "feature_names": [str(name) for name in bundle["actor_feature_names"]],
        }


def load_j7_support(root=VALIDATION_ROOT):
    """The J7 Markov recovery dataset."""
    with np.load(root / J7_DATASET_PATH, allow_pickle=False) as bundle:
        return bundle["observations"].astype(np.float32)


# --------------------------------------------------------------------------
# On-policy prefix
# --------------------------------------------------------------------------


def is_discrete_feature(name):
    """J14's discrete-feature predicate, transcribed locally.

    A feature is discrete when its name ends with one of the recorded suffixes
    or starts with one of the recorded prefixes. Pure and total.
    """
    text = str(name)
    return text.endswith(DISCRETE_NAME_SUFFIXES) or text.startswith(DISCRETE_NAME_PREFIXES)


def discrete_indices(feature_names):
    """The discrete-mismatch operator. No J14 import; the predicate is local."""
    return [i for i, name in enumerate(feature_names) if is_discrete_feature(name)]


def derive_prefix(on_policy_obs, teacher_obs, discrete_cols):
    """Derive the on-policy prefix rather than assume it.

    The prefix is the maximal contiguous run of leading indices at which the two
    trajectories agree on EVERY discrete feature. Returns the derived facts; the
    caller asserts them against the manifest.
    """
    agree = np.all(
        on_policy_obs[:, discrete_cols] == teacher_obs[:, discrete_cols], axis=1
    )
    disagreeing = np.flatnonzero(~agree)
    if disagreeing.size == 0:
        first_mismatch = int(agree.size)
        column = None
    else:
        first_mismatch = int(disagreeing[0])
        row_diff = (
            on_policy_obs[first_mismatch, discrete_cols]
            != teacher_obs[first_mismatch, discrete_cols]
        )
        column = int(np.asarray(discrete_cols)[np.flatnonzero(row_diff)[0]])
    return {
        "prefix_length": first_mismatch,
        "first_mismatch_index": first_mismatch,
        "first_mismatch_column": column,
        "prefix_is_contiguous": bool(np.all(agree[:first_mismatch])),
        "rows_excluded_post_mismatch": int(agree.size - first_mismatch),
        "total_mismatching_rows": int(disagreeing.size),
    }


# --------------------------------------------------------------------------
# Block construction
# --------------------------------------------------------------------------


def build_blocks(root=VALIDATION_ROOT, state=None):
    """Build the four disjoint loss blocks under the architect's priority order.

    Global disjunction with semantic priority A > B > C > D. An observation
    already claimed by a higher-priority block, or already seen earlier within
    the same block, is dropped; first occurrence in priority order wins. Every
    overlap and every target conflict is measured, not assumed.
    """
    if state is None:
        state = load_parent_state(root)

    teacher = load_teacher(root)
    target_trace = load_trace(TARGET_CELL, root)
    on_policy_obs = trace_observations(target_trace)

    discrete_cols = discrete_indices(teacher["feature_names"])
    prefix = derive_prefix(on_policy_obs, teacher["observations"], discrete_cols)
    prefix_length = prefix["prefix_length"]

    raw = {}
    raw["A"] = (teacher["observations"], teacher["actions"])
    raw["B"] = (
        on_policy_obs[:prefix_length],
        teacher["actions"][:prefix_length],
    )

    anchor_obs = []
    anchor_labels = []
    anchor_cell_of = []
    for cell in ANCHOR_CELLS:
        trace = load_trace(cell, root)
        anchor_obs.append(trace_observations(trace))
        anchor_labels.append(trace_policy_means(trace))
        anchor_cell_of.extend([cell] * len(trace))
    raw["C"] = (np.concatenate(anchor_obs), np.concatenate(anchor_labels))

    support = np.unique(load_j7_support(root), axis=0)
    raw["D"] = (support, forward_mean(state, support).astype(np.float32))

    overlaps = measure_overlaps(raw)

    claimed = set()
    blocks = {}
    kept_indices = {}
    for name in BLOCK_PRIORITY:
        observations, labels = raw[name]
        keep = []
        for index, row in enumerate(observations):
            key = row_key(row)
            if key in claimed:
                continue
            claimed.add(key)
            keep.append(index)
        kept_indices[name] = keep
        blocks[name] = (observations[keep], labels[keep])

    cells_after = {cell: 0 for cell in ANCHOR_CELLS}
    for index in kept_indices["C"]:
        cells_after[anchor_cell_of[index]] += 1

    return {
        "blocks": blocks,
        "raw_rows": {name: int(len(raw[name][0])) for name in BLOCK_PRIORITY},
        "kept_indices": kept_indices,
        "prefix": prefix,
        "discrete_indices": discrete_cols,
        "discrete_names": [teacher["feature_names"][i] for i in discrete_cols],
        "overlaps": overlaps,
        "anchor_rows_per_cell_after": cells_after,
        "teacher": teacher,
        "on_policy_observations": on_policy_obs,
        "state": state,
    }


def measure_overlaps(raw):
    """Overlap matrix and target-conflict measurement over the RAW blocks."""
    keys = {name: [row_key(r) for r in raw[name][0]] for name in BLOCK_PRIORITY}
    sets = {name: set(v) for name, v in keys.items()}

    matrix = {}
    for i, left in enumerate(BLOCK_PRIORITY):
        for right in BLOCK_PRIORITY[i + 1 :]:
            matrix["%s_%s" % (left, right)] = len(sets[left] & sets[right])

    conflicts = []
    for i, left in enumerate(BLOCK_PRIORITY):
        for right in BLOCK_PRIORITY[i + 1 :]:
            if not sets[left] & sets[right]:
                continue
            index_right = {}
            for index, key in enumerate(keys[right]):
                index_right.setdefault(key, index)
            worst = 0.0
            rows = 0
            conflicting = 0
            for index, key in enumerate(keys[left]):
                if key not in index_right:
                    continue
                rows += 1
                delta = float(
                    np.abs(
                        raw[left][1][index].astype(np.float64)
                        - raw[right][1][index_right[key]].astype(np.float64)
                    ).max()
                )
                worst = max(worst, delta)
                if delta > 0.0:
                    conflicting += 1
            conflicts.append(
                {
                    "overlap": "%s_%s" % (left, right),
                    "rows": rows,
                    "max_abs_target_difference": worst,
                    "conflicting_rows": conflicting,
                }
            )

    internal = {}
    for name in BLOCK_PRIORITY:
        internal[name] = int(len(keys[name]) - len(sets[name]))

    return {
        "pairwise_intersection": matrix,
        "internal_duplicates": internal,
        "distinct_observations": {n: len(sets[n]) for n in BLOCK_PRIORITY},
        "target_conflicts": conflicts,
        "total_conflicting_rows": int(
            sum(entry["conflicting_rows"] for entry in conflicts)
        ),
    }


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------


def is_finite_recursive(value):
    """True when every scalar reachable from ``value`` is finite.

    Walks dicts, sequences and arrays, so a NaN nested inside a per-action
    metric is caught as surely as one sitting in a weight tensor. Strings,
    booleans and None carry no float to check and are finite by vacuity.
    """
    if isinstance(value, dict):
        return all(is_finite_recursive(item) for item in value.values())
    if isinstance(value, (list, tuple, set, frozenset)):
        return all(is_finite_recursive(item) for item in value)
    if value is None or isinstance(value, (str, bytes, bool, np.bool_)):
        return True
    if isinstance(value, (int, np.integer)):
        return True
    if isinstance(value, (float, np.floating, complex, np.complexfloating)):
        return bool(np.isfinite(value))
    array = np.asarray(value)
    if array.dtype.kind in "fc":
        return bool(np.all(np.isfinite(array)))
    if array.dtype.kind == "O":
        return all(is_finite_recursive(item) for item in array.ravel().tolist())
    return True


def sanitise_non_finite(value, path="$"):
    """Replace every non-finite scalar with a typed marker, recording where.

    Fail-closed choice, made explicitly: a candidate carrying a non-finite
    diagnostic FAILS G11 and can therefore never be selected, but its record is
    still written, with each non-finite value replaced by
    ``{"non_finite": <kind>, "path": <path>}``. Aborting the whole run instead
    would destroy the evidence of the other fifteen candidates, which the
    fail-closed rule requires to be recorded. Nothing non-finite ever reaches
    JSON, and ``encode_json``'s ``allow_nan=False`` remains the last barrier.
    """
    findings = []

    def walk(node, here):
        if isinstance(node, dict):
            return {key: walk(item, "%s.%s" % (here, key)) for key, item in node.items()}
        if isinstance(node, (list, tuple)):
            return [walk(item, "%s[%d]" % (here, i)) for i, item in enumerate(node)]
        if isinstance(node, np.ndarray):
            return walk(node.tolist(), here)
        if isinstance(node, (bool, np.bool_)) or node is None:
            return node
        if isinstance(node, (float, np.floating)) and not np.isfinite(node):
            kind = ("NaN" if np.isnan(node)
                    else ("Infinity" if node > 0 else "-Infinity"))
            findings.append({"path": here, "kind": kind})
            return {"non_finite": kind, "path": here}
        if isinstance(node, np.floating):
            return float(node)
        if isinstance(node, np.integer):
            return int(node)
        return node

    return walk(value, path), findings


def build_candidate_record(candidate, metrics, gates, diagnostics, state=None):
    """Assemble one candidate's record so it serialises whatever it contains.

    The verdict is taken from the RAW gates, before any sanitisation touches
    them: it must come from what was measured, never from its serialisable
    rendering.

    Metrics, diagnostics AND gates are then sanitised. Gates matter here as much
    as the other two: ``evaluate_gates`` copies the measured value into
    ``gate["measured"]``, so a non-finite metric lands there too, and leaving
    gates unsanitised would abort the entire run at serialisation instead of
    recording this candidate as FAIL and continuing with the other fifteen -
    exactly what the fail-closed rule forbids.
    """
    passed_all = all(gate["passed"] for gate in gates)

    safe_metrics, metric_findings = sanitise_non_finite(metrics, "$.metrics")
    safe_diagnostics, diagnostic_findings = sanitise_non_finite(
        diagnostics, "$.diagnostics"
    )
    safe_gates, gate_findings = sanitise_non_finite(gates, "$.gates")
    findings = metric_findings + diagnostic_findings + gate_findings

    record = {
        "candidate": candidate,
        "metrics": safe_metrics,
        "gates": safe_gates,
        "passed_all_gates": passed_all,
        "non_finite_findings": findings,
        "non_finite_count": len(findings),
    }
    record.update(safe_diagnostics)
    if state is not None:
        record["state"] = state
    return record


def block_metrics(predictions, labels):
    """Per-block and per-action error, normalised and physical."""
    error = np.asarray(predictions, dtype=np.float64) - np.asarray(
        labels, dtype=np.float64
    )
    metrics = {
        "n": int(error.shape[0]),
        "mse": float((error ** 2).mean()),
        "rmse": float(np.sqrt((error ** 2).mean())),
        "max_abs": float(np.abs(error).max()),
    }
    for index, action in enumerate(("knee", "ankle")):
        column = error[:, index]
        metrics[action] = {
            "rmse": float(np.sqrt((column ** 2).mean())),
            "mae": float(np.abs(column).mean()),
            "max_abs": float(np.abs(column).max()),
            "signed_mean_shift": float(column.mean()),
            "rmse_rad": float(np.sqrt((column ** 2).mean()) * ACTION_SCALE_RAD),
            "signed_mean_shift_rad": float(column.mean() * ACTION_SCALE_RAD),
        }
    return metrics


def evaluate_all_blocks(state, blocks):
    """Every block's metrics for a given actor state."""
    return {
        name: block_metrics(forward_mean(state, observations), labels)
        for name, (observations, labels) in blocks.items()
    }


# --------------------------------------------------------------------------
# Grid and gates
# --------------------------------------------------------------------------


def enumerate_grid():
    """The finite preregistered grid, in its preregistered enumeration order."""
    candidates = []
    for lam in GRID_LAMBDA:
        for beta in GRID_BETA:
            for learning_rate in GRID_LEARNING_RATE:
                candidates.append(
                    {
                        "candidate_index": len(candidates),
                        "preservation_weight_lambda": lam,
                        "on_policy_weight_beta": beta,
                        "learning_rate": learning_rate,
                        "w_A": WEIGHT_A,
                        "w_B": beta,
                        "w_C": lam,
                        "w_D": lam,
                    }
                )
    return candidates


def composite_objective(metrics, candidate):
    """L = w_A*MSE_A + w_B*MSE_B + w_C*MSE_C + w_D*MSE_D."""
    return sum(
        candidate["w_%s" % name] * metrics[name]["mse"] for name in BLOCK_PRIORITY
    )


def evaluate_gates(state, metrics, parent=None, reported_diagnostics=None):
    """Every hard gate. Returns one record per gate, with the measured value.

    ``reported_diagnostics`` carries every OTHER numeric field the candidate
    record will publish - best_objective, the per-epoch history, the scale
    absorption report - so that G11's "every reported metric" covers them too
    rather than stopping at the A-D block metrics.
    """
    if parent is None:
        parent = load_parent_state()
    drift_c = metrics["C"]
    drift_d = metrics["D"]

    # G11 covers every tensor AND every reported metric, as registered: the
    # state, the A-D block metrics, and every other numeric field the candidate
    # record publishes. A NaN buried in a per-epoch history entry or in the
    # scale-absorption report must fail the gate, not slip past it because only
    # the state and the block metrics were inspected.
    finite = (
        is_finite_recursive(state)
        and is_finite_recursive(metrics)
        and is_finite_recursive(reported_diagnostics if reported_diagnostics else {})
    )
    parent_logstd = logstd_raw_slices(parent)
    candidate_logstd = logstd_raw_slices(state)
    logstd_identical = all(
        bytes_identical(parent_logstd[key], candidate_logstd[key])
        for key in HEAD_KEYS
    ) and logstd_is_state_independent(state)
    clock = np.asarray(state["pi.0.0.weight"], dtype=np.float64)[:, list(CLOCK_COLUMNS)]
    same_keys = set(parent) == set(state) and all(
        np.shape(parent[key]) == np.shape(state[key]) for key in parent
    )

    gates = [
        ("G1", "preservation_max_drift_C", drift_c["max_abs"], GATE_MAX_DRIFT, "<="),
        ("G2", "preservation_max_drift_D", drift_d["max_abs"], GATE_MAX_DRIFT, "<="),
        (
            "G3",
            "preservation_bias_C",
            max(abs(drift_c[a]["signed_mean_shift"]) for a in ("knee", "ankle")),
            GATE_BIAS_DRIFT,
            "<=",
        ),
        (
            "G4",
            "preservation_bias_D",
            max(abs(drift_d[a]["signed_mean_shift"]) for a in ("knee", "ankle")),
            GATE_BIAS_DRIFT,
            "<=",
        ),
        ("G5", "on_policy_improvement_B", metrics["B"]["mse"], GATE_MSE_B_CEILING, "<="),
        ("G6", "teacher_non_regression_A", metrics["A"]["mse"], GATE_MSE_A_CEILING, "<="),
        ("G7", "logstd_byte_identical", float(logstd_identical), 1.0, "=="),
        ("G8", "no_critic", float(len(critic_keys(state))), 0.0, "=="),
        ("G9", "clock_columns_zero", float(np.abs(clock).max()), 0.0, "=="),
        ("G10", "key_set_identical", float(same_keys), 1.0, "=="),
        ("G11", "finiteness", float(finite), 1.0, "=="),
    ]

    records = []
    for gate_id, name, measured, threshold, relation in gates:
        passed = measured <= threshold if relation == "<=" else measured == threshold
        records.append(
            {
                "id": gate_id,
                "name": name,
                "measured": float(measured),
                "threshold": float(threshold),
                "relation": relation,
                "passed": bool(passed),
            }
        )
    return records


def rank_survivors(survivors):
    """The preregistered total order over candidates that passed every gate."""
    return sorted(
        survivors,
        key=lambda entry: (
            entry["metrics"]["B"]["mse"],
            entry["metrics"]["A"]["mse"],
            max(entry["metrics"]["C"]["max_abs"], entry["metrics"]["D"]["max_abs"]),
            entry["candidate"]["candidate_index"],
        ),
    )


# --------------------------------------------------------------------------
# Preflight
# --------------------------------------------------------------------------


def load_manifest(root=VALIDATION_ROOT):
    """The pinned dataset manifest."""
    with open(root / MANIFEST_PATH, "r", encoding="utf-8") as handle:
        return json.load(handle)


def run_preflight(root=VALIDATION_ROOT, verbose=True):
    """Rebuild the dataset and check it against the pinned manifest.

    Trains nothing, writes nothing, creates no directory and never imports
    torch. Returns a report whose ``ok`` field is the verdict.
    """
    manifest = load_manifest(root)
    checks = []

    def record(name, passed, detail=""):
        checks.append({"check": name, "passed": bool(passed), "detail": str(detail)})

    for relative, expected in manifest["inputs_sha256"].items():
        actual = sha256_file(root / relative)
        record("input_hash:%s" % relative, actual == expected, actual)

    parent_actual = sha256_file(root / PARENT_MODULE_PATH)
    record(
        "parent_module_actual_bytes",
        parent_actual == PARENT_MODULE_SHA256,
        parent_actual,
    )

    stale = json.loads((root / PARENT_STALE_MANIFEST_PATH).read_text(encoding="utf-8"))
    declared = stale.get("module_state_sha256")
    record(
        "parent_leaf_manifest_is_stale_as_documented",
        declared == J2_MODULE_SHA256 and declared != PARENT_MODULE_SHA256,
        "declared=%s actual=%s" % (declared, parent_actual),
    )

    built = build_blocks(root)
    blocks = built["blocks"]
    prefix = built["prefix"]

    record(
        "prefix_derived_not_hardcoded",
        prefix["prefix_length"] == EXPECTED_PREFIX_LENGTH,
        prefix["prefix_length"],
    )
    record(
        "first_mismatch_index",
        prefix["first_mismatch_index"] == EXPECTED_FIRST_MISMATCH_INDEX,
        prefix["first_mismatch_index"],
    )
    record(
        "first_mismatch_column",
        prefix["first_mismatch_column"] == EXPECTED_FIRST_MISMATCH_COLUMN,
        prefix["first_mismatch_column"],
    )
    record("prefix_is_contiguous", prefix["prefix_is_contiguous"], True)
    record(
        "no_post_mismatch_rows_in_block_B",
        len(blocks["B"][0]) <= prefix["prefix_length"],
        len(blocks["B"][0]),
    )

    for name in BLOCK_PRIORITY:
        record(
            "block_rows:%s" % name,
            len(blocks[name][0]) == EXPECTED_BLOCK_ROWS[name],
            len(blocks[name][0]),
        )

    total = sum(len(blocks[n][0]) for n in BLOCK_PRIORITY)
    record("total_rows", total == EXPECTED_TOTAL_ROWS, total)

    union = set()
    for name in BLOCK_PRIORITY:
        union |= {row_key(row) for row in blocks[name][0]}
    record("blocks_are_disjoint", len(union) == total, len(union))

    record(
        "target_conflicts_are_zero",
        built["overlaps"]["total_conflicting_rows"] == 0,
        built["overlaps"]["total_conflicting_rows"],
    )

    manifest_blocks = {entry["id"]: entry for entry in manifest["block_composition"]["blocks"]}
    for name in BLOCK_PRIORITY:
        observations, labels = blocks[name]
        entry = manifest_blocks[name]
        record(
            "observations_hash:%s" % name,
            sha256_array(observations) == entry["observations_sha256"],
            sha256_array(observations),
        )
        if entry["labels_read_from_file_not_recomputed"]:
            record(
                "labels_hash:%s" % name,
                sha256_array(labels) == entry["labels_sha256"],
                sha256_array(labels),
            )
        else:
            recomputed = forward_mean(built["state"], observations).astype(np.float32)
            worst = float(np.abs(recomputed.astype(np.float64) - labels.astype(np.float64)).max())
            record(
                "labels_tolerance:%s" % name,
                worst <= COMPUTED_LABEL_TOLERANCE,
                worst,
            )

    for name in BLOCK_PRIORITY:
        expected = manifest["block_composition"]["counts_before_and_after"][name]
        record(
            "counts_before:%s" % name,
            built["raw_rows"][name] == expected["before"],
            built["raw_rows"][name],
        )

    metrics = evaluate_all_blocks(built["state"], blocks)
    record(
        "baseline_MSE_A_matches_manifest",
        abs(metrics["A"]["mse"] - BASELINE_MSE_A) < 1e-09,
        metrics["A"]["mse"],
    )
    record(
        "baseline_MSE_B_matches_manifest",
        abs(metrics["B"]["mse"] - BASELINE_MSE_B) < 1e-09,
        metrics["B"]["mse"],
    )

    parent_state = built["state"]
    record("parent_has_no_critic_key", len(critic_keys(parent_state)) == 0, "")
    clock = np.asarray(parent_state["pi.0.0.weight"], dtype=np.float64)[
        :, list(CLOCK_COLUMNS)
    ]
    record("parent_clock_columns_zero", float(np.abs(clock).max()) == 0.0, "")
    record(
        "observation_width",
        np.asarray(parent_state["pi.0.0.weight"]).shape[1] == OBSERVATION_WIDTH,
        np.asarray(parent_state["pi.0.0.weight"]).shape[1],
    )

    grid = enumerate_grid()
    record("grid_is_finite", len(grid) == 16, len(grid))
    record(
        "grid_indices_unique",
        len({entry["candidate_index"] for entry in grid}) == len(grid),
        "",
    )

    report = {
        "stage": "V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE",
        "mode": "PREFLIGHT",
        "trained_anything": False,
        "wrote_anything": False,
        "created_any_directory": False,
        "checks": checks,
        "passed": sum(1 for c in checks if c["passed"]),
        "total": len(checks),
        "ok": all(c["passed"] for c in checks),
        "prefix": prefix,
        "overlaps": built["overlaps"],
        "baseline_metrics": metrics,
        "grid": grid,
    }

    if verbose:
        for check in checks:
            flag = "PASS" if check["passed"] else "FAIL"
            print("[%s] %-52s %s" % (flag, check["check"], check["detail"]))
        print(
            "\npreflight %s: %d/%d checks"
            % ("PASS" if report["ok"] else "FAIL", report["passed"], report["total"])
        )
        print("trained nothing, wrote nothing, created no directory")

    return report


# --------------------------------------------------------------------------
# July-compatible scaling.
#
# The committed actor consumes RAW observations; training runs on obs/scales.
# J11 and J15R1 initialised straight from their parent's RAW weights and fed
# scaled inputs, which is function-preserving ONLY when the parent's weights on
# the scaled columns are zero. That was true of their parent J2. It is NOT true
# of J8, whose controller block is live: the naive route moves the output by
# 1.077e-02 before a single gradient, about 74x the G1 drift budget. J18
# therefore converts the parent into the scaled domain first.
# --------------------------------------------------------------------------


def july_scale_vector(feature_names):
    """The July physical scale per feature, from the local registered mapping.

    No J8 import. Every name in the mapping MUST be present; a missing name is a
    hard failure, never a silent 1.0, because a silently unscaled controller
    feature would change what the network is trained on without any signal.
    """
    names = [str(name) for name in feature_names]
    scales = np.ones(len(names), dtype=np.float32)
    missing = [name for name, _ in JULY_CONTROLLER_FEATURE_SCALES if name not in names]
    if missing:
        raise RuntimeError(
            "the observation is missing registered controller features, so the "
            "July scaling cannot be applied: %s" % missing
        )
    for name, value in JULY_CONTROLLER_FEATURE_SCALES:
        scales[names.index(name)] = np.float32(value)
    return scales


def scaled_from_raw(weight, scales):
    """Raw-domain first-layer weight -> scaled-domain, column-wise."""
    out = np.array(weight, dtype=np.float32, copy=True)
    for column, scale in enumerate(np.asarray(scales, dtype=np.float32)):
        if float(scale) != 1.0:
            out[:, column] = (out[:, column] * scale).astype(np.float32)
    return out


def raw_from_scaled(weight, scales):
    """Scaled-domain first-layer weight -> raw-domain, column-wise."""
    out = np.array(weight, dtype=np.float32, copy=True)
    for column, scale in enumerate(np.asarray(scales, dtype=np.float32)):
        if float(scale) != 1.0:
            out[:, column] = (out[:, column] / scale).astype(np.float32)
    return out


def torch_forward_mean(state, x, torch_module):
    """The mean head evaluated by the torch float32 kernel, for any state."""
    torch = torch_module
    from torch.nn import functional

    def tensor(value):
        return torch.as_tensor(np.asarray(value, dtype=np.float32))

    h = torch.tanh(
        functional.linear(tensor(x), tensor(state["pi.0.0.weight"]),
                          tensor(state["pi.0.0.bias"]))
    )
    h = torch.tanh(
        functional.linear(h, tensor(state["pi.0.2.weight"]), tensor(state["pi.0.2.bias"]))
    )
    return functional.linear(
        h, tensor(state["pi.1.weight"]), tensor(state["pi.1.bias"])
    )[:, MEAN_ROWS]


def weight_round_trip_report(scaled_weight, scales):
    """Diagnostic only: how far (W/s)*s lands from W.

    This is NOT a contract. For arbitrary float32 values produced by Adam,
    (x/s)*s is frequently not bit-identical - measurably so at s = 3.5, 55 and
    60 - so requiring byte identity here would abort sound runs. What is
    required is the FUNCTIONAL equivalence measured separately.
    """
    restored = scaled_from_raw(raw_from_scaled(scaled_weight, scales), scales)
    difference = np.abs(
        restored.astype(np.float64) - np.asarray(scaled_weight, dtype=np.float64)
    )
    changed = int((difference > 0.0).sum())
    total = int(difference.size)
    return {
        "max_abs_delta": float(difference.max()),
        "entries_not_bit_identical": changed,
        "entries_total": total,
        "fraction_not_bit_identical": (changed / total) if total else 0.0,
        "is_a_diagnostic_not_a_gate": True,
    }


def scaled_domain_state(parent, scales):
    """A copy of the parent whose first layer lives in the scaled domain."""
    state = {key: np.array(parent[key], dtype=np.float32, copy=True) for key in DIRECT_KEYS}
    state["pi.0.0.weight"] = scaled_from_raw(parent["pi.0.0.weight"], scales)
    return state


# --------------------------------------------------------------------------
# The flat disjoint union and the registered minibatch estimator
# --------------------------------------------------------------------------


def build_flat_dataset(built):
    """Concatenate the four disjoint blocks into one row-indexed dataset."""
    blocks = built["blocks"]
    observations = np.concatenate([blocks[b][0] for b in BLOCK_PRIORITY]).astype(np.float32)
    labels = np.concatenate([blocks[b][1] for b in BLOCK_PRIORITY]).astype(np.float32)
    block_index = np.concatenate(
        [np.full(len(blocks[b][0]), i, dtype=np.int64)
         for i, b in enumerate(BLOCK_PRIORITY)]
    )
    sizes = {b: int(len(blocks[b][0])) for b in BLOCK_PRIORITY}
    return {
        "observations": observations,
        "labels": labels,
        "block_index": block_index,
        "block_sizes": sizes,
        "n": int(len(observations)),
    }


def row_coefficients(candidate, flat):
    """coef_i = w_block(i) / N_block(i), the per-row weight of the estimator."""
    coefficients = np.zeros(flat["n"], dtype=np.float64)
    for position, name in enumerate(BLOCK_PRIORITY):
        mask = flat["block_index"] == position
        coefficients[mask] = candidate["w_%s" % name] / float(flat["block_sizes"][name])
    return coefficients


def epoch_batches(rng, n, batch=FIT_BATCH_SIZE):
    """One permutation per epoch, consecutive chunks, final short chunk KEPT."""
    permutation = rng.permutation(n)
    for start in range(0, n, batch):
        yield permutation[start:start + batch]


def expected_batches_per_epoch(n=EXPECTED_TOTAL_ROWS, batch=FIT_BATCH_SIZE):
    """Chunk count for one epoch, counting the final short chunk."""
    return -(-n // batch)


def minibatch_loss(predictions, targets, coefficients, n, torch_module):
    """L_M = (N / |M|) * sum_{i in M} coef_i * e_i, with e_i averaged over
    the two action dimensions.

    FOR A FIXED theta, summing one epoch's chunk losses weighted by |M|/N gives
    exactly the registered objective sum_b w_b * MSE_b; and for an M drawn
    uniformly and independently of theta, L_M is an unbiased estimator of it.

    Neither statement is claimed for the actual training loop. Training uses
    random reshuffling, so after the first minibatch theta depends on the rows
    already visited and the next chunk comes from the remainder; chunk and theta
    are not independent, and theta is not fixed across an epoch. What the loop
    does guarantee is exact per-row coefficients - each block's influence is set
    by its registered weight, never by how many of its rows land in a batch -
    and exact once-per-epoch coverage. A plain per-batch row mean would give
    neither.
    """
    per_row = ((predictions - targets) ** 2).mean(dim=1)
    scale = float(n) / float(per_row.shape[0])
    return torch_module.mul(coefficients * per_row, scale).sum()


# --------------------------------------------------------------------------
# Actor assembly
# --------------------------------------------------------------------------


def tensor_digest(value):
    """One tensor's digest: dtype, then shape, then C-ordered bytes."""
    array = np.ascontiguousarray(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(d) for d in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def actor_digest(state):
    """The project's canonical actor digest, transcribed locally.

    An exact transcription of ``warm_start.actor_state_digest``, with no import
    of J11 or any other prior-stage module: a J18 run must not depend on code a
    J18 GO does not pin. The transcription is held honest by tests that assert
    it reproduces J8 = 6a879714... and J2 = 59d54240... from their artefacts.
    """
    digest = hashlib.sha256()
    for key in sorted(WARM_START_ACTOR_KEYS):
        if key not in state:
            raise RuntimeError("actor state is missing %s" % key)
        digest.update(key.encode("utf-8"))
        digest.update(tensor_digest(state[key]).encode("ascii"))
    return digest.hexdigest()


def assemble_actor_state(direct, parent):
    """The ten-key actor: six trained tensors plus four bit-identical aliases."""
    state = {key: np.array(direct[key], dtype=np.float32, copy=True) for key in DIRECT_KEYS}
    for alias, source in ENCODER_ALIASES.items():
        state[alias] = state[source].copy()
    missing = set(parent) - set(state)
    if missing:
        raise RuntimeError("assembled actor is missing parent keys: %s" % sorted(missing))
    extra = set(state) - set(parent)
    if extra:
        raise RuntimeError("assembled actor has keys the parent lacks: %s" % sorted(extra))
    return state


# --------------------------------------------------------------------------
# Architect GO
# --------------------------------------------------------------------------

GO_REQUIRED_STAGE = "V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE"
GO_REQUIRED_PINS = (
    PREREG_PATH,
    ADDENDUM_PATH,
    ADDENDUM_V3_PATH,
    MANIFEST_PATH,
    PROVENANCE_OVERLAY_PATH,
    "v26c_j18_b_only_update.py",
    "test_v26c_j18_b_only_update.py",
)


def validate_go(payload, root=VALIDATION_ROOT):
    """Validate an architect GO payload. Pure: takes a dict, touches no GO file.

    A GO authorises execution only if it names this stage, carries an explicit
    execution authorisation, and pins the exact current hash of all SEVEN
    artefacts: the v1 preregistration, the v2 and v3 addenda, the dataset
    manifest, the provenance overlay, this runner and its test suite. Any
    missing pin, any extra scope and any hash mismatch is a refusal, never a
    warning.
    """
    problems = []
    if not isinstance(payload, dict):
        return {"valid": False, "problems": ["the GO payload is not an object"], "pins": {}}

    if payload.get("stage") != GO_REQUIRED_STAGE:
        problems.append(
            "stage is %r, expected %r" % (payload.get("stage"), GO_REQUIRED_STAGE)
        )
    if payload.get("authorises_execution") is not True:
        problems.append("authorises_execution is not exactly true")

    pins = payload.get("pinned_artefacts_sha256")
    if not isinstance(pins, dict):
        problems.append("pinned_artefacts_sha256 is missing or is not an object")
        pins = {}

    for required in GO_REQUIRED_PINS:
        if required not in pins:
            problems.append("no pin for %s" % required)

    for name in sorted(pins):
        if name not in GO_REQUIRED_PINS:
            problems.append("pin for %s is outside the authorised scope" % name)
            continue
        path = root / name
        if not path.exists():
            problems.append("pinned artefact %s does not exist" % name)
            continue
        actual = sha256_file(path)
        if actual != pins[name]:
            problems.append(
                "pinned hash for %s is stale: GO says %s, the file is %s"
                % (name, pins[name], actual)
            )

    return {"valid": not problems, "problems": problems, "pins": dict(pins)}


def load_go(go_file):
    """Read a GO file from disk and validate it."""
    path = pathlib.Path(go_file)
    if not path.exists():
        return {"valid": False, "problems": ["the GO file %s does not exist" % go_file],
                "pins": {}}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (ValueError, OSError) as error:
        return {"valid": False, "problems": ["the GO file is unreadable: %s" % error],
                "pins": {}}
    return validate_go(payload)


# --------------------------------------------------------------------------
# One candidate
# --------------------------------------------------------------------------


def run_candidate(candidate, flat, parent, scales, torch_module, progress=False):
    """Train one candidate. Returns the RAW actor and its history.

    Every candidate starts from pristine parent bytes with a fresh Adam, a
    fresh torch seed and a fresh permutation generator, so nothing carries over
    from the candidate before it.
    """
    torch = torch_module
    from torch.nn import functional

    torch.manual_seed(FIT_SEED)
    np.random.seed(FIT_SEED)

    start_state = scaled_domain_state(parent, scales)
    params = {
        key: torch.nn.Parameter(torch.as_tensor(np.array(start_state[key], copy=True)))
        for key in DIRECT_KEYS
    }
    frozen_logstd_weight = torch.as_tensor(
        np.array(parent["pi.1.weight"], dtype=np.float32)[LOGSTD_ROWS].copy()
    )
    frozen_logstd_bias = torch.as_tensor(
        np.array(parent["pi.1.bias"], dtype=np.float32)[LOGSTD_ROWS].copy()
    )

    def forward(x):
        h = torch.tanh(functional.linear(x, params["pi.0.0.weight"], params["pi.0.0.bias"]))
        h = torch.tanh(functional.linear(h, params["pi.0.2.weight"], params["pi.0.2.bias"]))
        return functional.linear(h, params["pi.1.weight"], params["pi.1.bias"])

    def project():
        """The two standing invariants, re-imposed after EVERY optimizer step."""
        with torch.no_grad():
            params["pi.1.weight"][LOGSTD_ROWS].copy_(frozen_logstd_weight)
            params["pi.1.bias"][LOGSTD_ROWS].copy_(frozen_logstd_bias)
            params["pi.0.0.weight"][:, list(CLOCK_COLUMNS)].zero_()

    project()

    observations = flat["observations"]
    scaled_inputs = torch.as_tensor((observations / scales).astype(np.float32))
    targets = torch.as_tensor(flat["labels"])
    coefficients = torch.as_tensor(row_coefficients(candidate, flat))
    n = flat["n"]

    # The registered seed order puts the Generator BEFORE the optimizer. Adam
    # consumes no NumPy randomness today, so the order is not observable in the
    # current torch, but the code follows the registered contract rather than
    # relying on that remaining true.
    rng = np.random.default_rng(FIT_SEED)

    optimiser = torch.optim.Adam(
        list(params.values()), lr=float(candidate["learning_rate"])
    )

    def composite_now():
        with torch.no_grad():
            predictions = forward(scaled_inputs)[:, MEAN_ROWS]
            per_row = ((predictions - targets) ** 2).mean(dim=1).numpy().astype(np.float64)
        total = 0.0
        for position, name in enumerate(BLOCK_PRIORITY):
            mask = flat["block_index"] == position
            total += candidate["w_%s" % name] * float(per_row[mask].mean())
        return total

    best = {
        "epoch": 0,
        "objective": composite_now(),
        "state": {k: v.detach().clone() for k, v in params.items()},
    }
    history = []
    steps = 0
    for epoch in range(1, FIT_EPOCHS + 1):
        batches = 0
        losses = []
        for index in epoch_batches(rng, n):
            selection = torch.as_tensor(index)
            logits = forward(scaled_inputs[selection])
            loss = minibatch_loss(
                logits[:, MEAN_ROWS], targets[selection], coefficients[selection], n, torch
            )
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            optimiser.step()
            project()
            losses.append(float(loss.item()))
            batches += 1
            steps += 1
        objective = composite_now()
        history.append(
            {
                "epoch": epoch,
                "batches": batches,
                "train_loss_mean": float(np.mean(losses)),
                "composite_objective": objective,
            }
        )
        # strict improvement only, so ties resolve to the EARLIEST epoch
        if objective < best["objective"]:
            best = {
                "epoch": epoch,
                "objective": objective,
                "state": {k: v.detach().clone() for k, v in params.items()},
            }
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(
                json.dumps(
                    {
                        "candidate": candidate["candidate_index"],
                        "epoch": epoch,
                        "objective": objective,
                    }
                ),
                flush=True,
            )

    with torch.no_grad():
        for key in DIRECT_KEYS:
            params[key].copy_(best["state"][key])
        project()

    scaled_weight = params["pi.0.0.weight"].detach().numpy().astype(np.float32)
    raw_weight = raw_from_scaled(scaled_weight, scales)

    direct = {key: params[key].detach().numpy().astype(np.float32) for key in DIRECT_KEYS}
    direct["pi.0.0.weight"] = raw_weight
    direct["pi.0.0.weight"][:, list(CLOCK_COLUMNS)] = 0.0

    # Absorbing the scales must preserve the FUNCTION. That is the contract, and
    # under addendum v3 it is measured over ALL rows in BOTH kernels, each side
    # evaluated like for like.
    #
    # It is NOT a bit-identical weight round trip. For arbitrary float32 values
    # produced by Adam, (x/s)*s is frequently not bit-identical - measurably so
    # at s = 3.5, 55 and 60 - so requiring byte identity would abort sound runs.
    # The delta is recorded as a diagnostic instead. The parent-side claim
    # (W*s)/s == W is a different direction, is exact, and is checked separately
    # in verify_raw_scaled_equivalence.
    scaled_state = {key: np.array(direct[key], copy=True) for key in DIRECT_KEYS}
    scaled_state["pi.0.0.weight"] = scaled_weight.copy()
    scaled_state["pi.0.0.weight"][:, list(CLOCK_COLUMNS)] = 0.0
    scaled_observations = (observations / scales).astype(np.float32)

    numpy_error = float(
        np.abs(
            forward_mean(scaled_state, scaled_observations)
            - forward_mean(direct, observations)
        ).max()
    )
    with torch.no_grad():
        torch_error = float(
            np.abs(
                torch_forward_mean(scaled_state, scaled_observations, torch).numpy()
                .astype(np.float64)
                - torch_forward_mean(direct, observations, torch).numpy().astype(np.float64)
            ).max()
        )
    absorption = {
        "rows": int(len(observations)),
        "numpy_max_abs_diff": numpy_error,
        "torch_max_abs_diff": torch_error,
        "tolerance": RAW_VS_SCALED_TOLERANCE,
        "weight_round_trip": weight_round_trip_report(scaled_weight, scales),
        "byte_identity_is_not_required": True,
    }
    absorption["ok"] = bool(
        numpy_error <= RAW_VS_SCALED_TOLERANCE and torch_error <= RAW_VS_SCALED_TOLERANCE
    )
    if not absorption["ok"]:
        raise RuntimeError(
            "absorbing the scales changed candidate %d's function beyond the "
            "registered tolerance %.1e: numpy %.3e, torch %.3e"
            % (candidate["candidate_index"], RAW_VS_SCALED_TOLERANCE,
               numpy_error, torch_error)
        )

    return {
        "candidate": candidate,
        "state": assemble_actor_state(direct, parent),
        "history": history,
        "best_epoch": int(best["epoch"]),
        "best_objective": float(best["objective"]),
        "optimizer_steps": int(steps),
        "batches_per_epoch": int(history[0]["batches"]) if history else 0,
        "scale_absorption": absorption,
    }


# --------------------------------------------------------------------------
# Preconditions that must hold before any optimizer step
# --------------------------------------------------------------------------


def verify_raw_scaled_equivalence(parent, observations, scales, torch_module=None):
    """The scaled-domain parameters must reproduce the parent's raw output."""
    raw = forward_mean(parent, observations)
    scaled_state = dict(parent)
    scaled_state = {k: np.array(parent[k], dtype=np.float32, copy=True) for k in parent}
    scaled_state["pi.0.0.weight"] = scaled_from_raw(parent["pi.0.0.weight"], scales)
    scaled_inputs = (np.asarray(observations, dtype=np.float32) / scales).astype(np.float32)
    scaled = forward_mean(scaled_state, scaled_inputs)
    numpy_diff = float(np.abs(raw - scaled).max())

    report = {
        "numpy_max_abs_diff": numpy_diff,
        "tolerance": RAW_VS_SCALED_TOLERANCE,
        "numpy_within_tolerance": numpy_diff <= RAW_VS_SCALED_TOLERANCE,
        "bit_identical_is_not_claimed": True,
        "why": "(W*s)@(o/s) and W@o are different float32 operation sequences and "
               "need not agree bit for bit. J11 and J15R1 could claim bit-identity "
               "only because their multiply and divide were no-ops on zero weights.",
    }

    round_trip = raw_from_scaled(scaled_state["pi.0.0.weight"], scales)
    report["weight_round_trip_bit_identical"] = bool(
        np.array_equal(
            round_trip, np.array(parent["pi.0.0.weight"], dtype=np.float32)
        )
    )

    if torch_module is not None:
        torch = torch_module
        with torch.no_grad():
            torch_raw = torch_forward_mean(parent, observations, torch).numpy()
            torch_scaled = torch_forward_mean(scaled_state, scaled_inputs, torch).numpy()
        torch_diff = float(
            np.abs(torch_raw.astype(np.float64) - torch_scaled.astype(np.float64)).max()
        )
        report["torch_max_abs_diff"] = torch_diff
        report["torch_within_tolerance"] = torch_diff <= RAW_VS_SCALED_TOLERANCE
        report["torch_version"] = str(torch.__version__)

    report["ok"] = bool(
        report["numpy_within_tolerance"]
        and report["weight_round_trip_bit_identical"]
        and report.get("torch_within_tolerance", True)
    )
    return report


# --------------------------------------------------------------------------
# Leaf: content-addressed, no-clobber, born invalid
# --------------------------------------------------------------------------


def aggregate_digest(files):
    """One digest over a mapping of leaf-relative name -> bytes."""
    digest = hashlib.sha256()
    for name in sorted(files):
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(hashlib.sha256(files[name]).hexdigest().encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def commit_leaf(files, root=VALIDATION_ROOT, leaf_name=LEAF_NAME):
    """Write a leaf atomically, born invalid, verified after commit.

    ``files`` maps leaf-relative names to bytes. The invalid marker is the FIRST
    write into staging and its removal is the LAST write after verification, so
    a leaf is never valid-looking while unverified.
    """
    import os

    leaf_root = root / LEAF_ROOT
    final = leaf_root / leaf_name
    if final.exists():
        raise RuntimeError(
            "refusing to clobber an existing leaf: %s. Execution is single-shot."
            % final
        )

    digest = aggregate_digest(files)
    staging = leaf_root / (".staging_%s" % digest[:16])
    if staging.exists():
        raise RuntimeError("refusing to reuse an existing staging directory: %s" % staging)
    staging.mkdir(parents=True)

    marker = staging / INVALID_MARKER
    marker.write_bytes(
        b"born invalid; removed only after post-commit verification succeeds\n"
    )

    for name in sorted(files):
        target = staging / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(files[name])

    os.rename(str(staging), str(final))

    problems = []
    receipt_name = RECEIPT_NAME
    with open(final / receipt_name, "r", encoding="utf-8") as handle:
        committed = json.load(handle)
    for name, expected in sorted(committed["artefacts_sha256"].items()):
        path = final / name
        if not path.exists():
            problems.append("committed receipt names a missing file: %s" % name)
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append("committed %s hashes %s, receipt says %s" % (name, actual, expected))
        staged = hashlib.sha256(files[name]).hexdigest() if name in files else None
        if staged is not None and staged != actual:
            problems.append("committed %s differs from the staged bytes" % name)

    verification = {
        "kind": "POST-COMMIT VERIFICATION",
        "leaf": str(final.relative_to(root)),
        "aggregate_digest": digest,
        "files_verified": len(committed["artefacts_sha256"]),
        "problems": problems,
        "ok": not problems,
        "method": "every leaf-relative path is re-resolved from the COMMITTED receipt, "
                  "re-hashed, and compared both to the receipt and to the staged bytes",
    }
    (final / "commit_verification.json").write_bytes(
        json.dumps(verification, indent=2, sort_keys=True).encode("utf-8")
    )
    if problems:
        raise RuntimeError(
            "post-commit verification FAILED; the invalid marker is left in place: %s"
            % problems
        )
    (final / INVALID_MARKER).unlink()
    return {"leaf": str(final.relative_to(root)), "verification": verification}


def encode_json(payload):
    """Deterministic JSON bytes.

    ``allow_nan=False`` is a second fail-closed barrier behind G11: even if a
    non-finite value reached serialisation, it would raise here rather than be
    written as the non-standard NaN/Infinity literals that json emits by
    default and that a strict reader would then reject.
    """
    return json.dumps(
        payload, indent=2, sort_keys=True, allow_nan=False
    ).encode("utf-8")


def strip_state(entry):
    """A candidate record without its tensors, for serialisation."""
    return {k: v for k, v in entry.items() if k != "state"}


def build_selection(results, ranked, selected):
    """The selection record. Pure, so it can be tested without writing a leaf."""
    survivors = [entry["candidate"]["candidate_index"] for entry in ranked]
    return {
        "kind": "SELECTION UNDER THE PREREGISTERED TOTAL ORDER",
        "stage": GO_REQUIRED_STAGE,
        "ranking_keys": [
            "MSE_B ascending", "MSE_A ascending",
            "max_abs_drift over C union D ascending", "candidate_index ascending",
        ],
        "candidates_evaluated": len(results),
        "survivors_in_rank_order": survivors,
        "survivor_count": len(survivors),
        "selected_candidate_index": (
            selected["candidate"]["candidate_index"] if selected else None
        ),
        "actors_persisted": 1 if selected else 0,
        "sixteen_actors_are_never_persisted": True,
        "fail_closed": selected is None,
        "fail_closed_meaning": (
            "no candidate passed every hard gate, so NO actor is written. No "
            "threshold is relaxed, no axis value is added and no candidate is "
            "re-run. The phase returns to the architect."
            if selected is None else "not triggered; one survivor was selected"
        ),
        "gate_failure_tally": {
            gate["id"]: sum(
                1 for entry in results
                for g in entry["gates"] if g["id"] == gate["id"] and not g["passed"]
            )
            for gate in results[0]["gates"]
        } if results else {},
    }


def finalise(root, built, flat, parent, scales, equivalence, go, results, ranked,
             selected, expected_batches, expected_steps):
    """Assemble every leaf file and commit it.

    ALL sixteen candidates contribute metrics and gate outcomes. AT MOST ONE
    actor is persisted: the top-ranked survivor. With no survivor the leaf is
    written with no actor at all.
    """
    files = {}

    files["v26c_j18_candidate_metrics.json"] = encode_json(
        {
            "kind": "ALL-CANDIDATE METRICS AND GATE OUTCOMES",
            "stage": GO_REQUIRED_STAGE,
            "candidates_run": len(results),
            "every_candidate_is_recorded_whether_it_passed_or_not": True,
            "records": [strip_state(entry) for entry in results],
        }
    )

    files["history.json"] = encode_json(
        {
            "per_candidate_epoch_history": {
                str(entry["candidate"]["candidate_index"]): entry["history"]
                for entry in results
            }
        }
    )

    selection = build_selection(results, ranked, selected)
    files["v26c_j18_selection.json"] = encode_json(selection)

    if selected is not None:
        state = selected["state"]
        module_bytes = pickle.dumps(state, protocol=4)
        files["rl_module/module_state.pkl"] = module_bytes

        source = root / PARENT_MODULE_PATH.replace(
            "module_state.pkl", "class_and_ctor_args.pkl"
        )
        files["rl_module/class_and_ctor_args.pkl"] = source.read_bytes()

        digest = actor_digest(state)
        files["rl_module/actor_feature_manifest.json"] = encode_json(
            {
                "actor_label": "J18_B_ONLY_CONSTRAINED_UPDATE",
                "stage": GO_REQUIRED_STAGE,
                "status": "candidate; offline gates passed; closed-loop requalification NOT run",
                "module_state_sha256": hashlib.sha256(module_bytes).hexdigest(),
                "actor_digest": digest,
                "source_actor_digest": actor_digest(parent),
                "derived_from": {
                    "parent": "J8",
                    "parent_module_state_sha256": PARENT_MODULE_SHA256,
                    "parent_identity_source": "the parent's ACTUAL BYTES, never its leaf "
                                              "sidecar manifest, which is stale; see "
                                              + PROVENANCE_OVERLAY_PATH,
                },
                "observation_width": OBSERVATION_WIDTH,
                "feature_names": list(built["teacher"]["feature_names"]),
                "input_convention": "RAW observations, unscaled, in the feature order above",
                "physical_scaling": "the July physical scales are absorbed into pi.0.0.weight "
                                    "at save; the parent was converted INTO the scaled domain "
                                    "before training because its controller block is live",
                "clock_columns": list(CLOCK_COLUMNS),
                "clock_columns_exactly_zero": True,
                "logstd": "rows 2:4 of pi.1.weight and pi.1.bias, byte-identical to J8",
                "truthful_about_its_own_module": True,
            }
        )

        files["rl_module/metadata.json"] = encode_json(
            {
                "stage": GO_REQUIRED_STAGE,
                "candidate_index": selected["candidate"]["candidate_index"],
                "hyperparameters": selected["candidate"],
                "seed": FIT_SEED,
                "epochs": FIT_EPOCHS,
                "batch_size": FIT_BATCH_SIZE,
                "batches_per_epoch": expected_batches,
                "optimizer_steps": selected["optimizer_steps"],
                "best_epoch": selected["best_epoch"],
                "best_objective": selected["best_objective"],
                "no_early_stopping": True,
                "no_validation_split": True,
            }
        )

    receipt = {
        "kind": "EXECUTION RECEIPT",
        "stage": GO_REQUIRED_STAGE,
        "verdict": "PASS" if selected is not None else "FAIL_CLOSED_NO_SURVIVOR",
        "inputs": {
            "parent": {
                "path": PARENT_MODULE_PATH,
                "actual_byte_sha256": PARENT_MODULE_SHA256,
                "actor_digest": actor_digest(parent),
                "identity_source": "actual bytes; the leaf sidecar manifest is stale",
            },
            "dataset_manifest_sha256": sha256_file(root / MANIFEST_PATH),
            "prereg_sha256": sha256_file(root / PREREG_PATH),
            "prereg_addendum_v2_sha256": sha256_file(root / ADDENDUM_PATH),
            "prereg_addendum_v3_sha256": sha256_file(root / ADDENDUM_V3_PATH),
            "provenance_overlay_sha256": sha256_file(root / PROVENANCE_OVERLAY_PATH),
            "runner_sha256": sha256_file(pathlib.Path(__file__).resolve()),
            "architect_go_pins": go["pins"],
        },
        "dataset": {
            "rows": flat["n"],
            "rows_by_block": flat["block_sizes"],
            "blocks_are_disjoint": True,
            "priority_order": list(BLOCK_PRIORITY),
        },
        "protocol": {
            "candidates": len(results),
            "seed": FIT_SEED,
            "epochs": FIT_EPOCHS,
            "batch_size": FIT_BATCH_SIZE,
            "batches_per_epoch": expected_batches,
            "optimizer_steps_per_candidate": expected_steps,
            "estimator": "L_M = (N/|M|) * sum_{i in M} w_block(i)/N_block(i) * e_i",
            "rng": "np.random.default_rng(%d), reset identically per candidate" % FIT_SEED,
            "final_short_chunk_kept": True,
            "no_early_stopping": True,
            "no_retry": True,
        },
        "raw_vs_scaled_equivalence": equivalence,
        "selection": selection,
        "artefacts_sha256": {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in sorted(files.items())
        },
    }
    files[RECEIPT_NAME] = encode_json(receipt)

    committed = commit_leaf(files, root=root)
    return {
        "leaf": committed["leaf"],
        "verification": committed["verification"],
        "verdict": receipt["verdict"],
        "selection": selection,
    }


# --------------------------------------------------------------------------
# Fit. Requires a valid architect GO; refuses otherwise.
# --------------------------------------------------------------------------


def run_fit(go_file, root=VALIDATION_ROOT, progress=False):
    """Execute the preregistered grid under a valid architect GO.

    Runs all sixteen candidates, records every candidate's metrics and every
    gate outcome, and persists AT MOST ONE actor: the top-ranked survivor. If
    no candidate survives, the leaf is written with no actor at all.
    """
    go = load_go(go_file)
    if not go["valid"]:
        raise RuntimeError(
            "refusing to execute: the architect GO is absent or invalid. %s"
            % "; ".join(go["problems"])
        )

    preflight = run_preflight(root, verbose=False)
    if not preflight["ok"]:
        raise RuntimeError("refusing to execute: preflight did not pass")

    built = build_blocks(root)
    flat = build_flat_dataset(built)
    parent = built["state"]
    scales = july_scale_vector(built["teacher"]["feature_names"])

    import torch

    equivalence = verify_raw_scaled_equivalence(
        parent, flat["observations"], scales, torch_module=torch
    )
    if not equivalence["ok"]:
        raise RuntimeError(
            "refusing to execute: raw/scaled equivalence failed before the "
            "optimizer: %s" % equivalence
        )

    expected_batches = expected_batches_per_epoch(flat["n"])
    expected_steps = expected_batches * FIT_EPOCHS

    results = []
    for candidate in enumerate_grid():
        trained = run_candidate(candidate, flat, parent, scales, torch, progress=progress)
        if trained["optimizer_steps"] != expected_steps:
            raise RuntimeError(
                "candidate %d took %d optimizer steps, the contract is %d"
                % (candidate["candidate_index"], trained["optimizer_steps"], expected_steps)
            )
        metrics = evaluate_all_blocks(trained["state"], built["blocks"])

        # Every OTHER numeric field this record will publish, assembled BEFORE
        # the gate runs and reused verbatim in the record, so G11 inspects
        # exactly what will be serialised - no reported number sits outside it.
        diagnostics = {
            "best_epoch": trained["best_epoch"],
            "best_objective": trained["best_objective"],
            "optimizer_steps": trained["optimizer_steps"],
            "batches_per_epoch": trained["batches_per_epoch"],
            "scale_absorption": trained["scale_absorption"],
            "history": trained["history"],
        }
        gates = evaluate_gates(
            trained["state"], metrics, parent=parent, reported_diagnostics=diagnostics
        )

        results.append(
            build_candidate_record(
                candidate, metrics, gates, diagnostics, state=trained["state"]
            )
        )

    survivors = [entry for entry in results if entry["passed_all_gates"]]
    ranked = rank_survivors(survivors)
    selected = ranked[0] if ranked else None

    return finalise(
        root, built, flat, parent, scales, equivalence, go, results, ranked, selected,
        expected_batches, expected_steps,
    )


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def build_parser():
    """Command-line surface. Preflight is the default."""
    parser = argparse.ArgumentParser(
        description="V26C J18 - J8 B-only constrained update"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--preflight-only",
        action="store_true",
        help="validate the manifest without training and without writing",
    )
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="alias of --preflight-only; also prints the candidate grid",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="run the grid; requires --go-file",
    )
    parser.add_argument("--go-file", default=None, help="architect GO json")
    parser.add_argument("--progress", action="store_true", help="print epoch progress")
    return parser


def main(argv=None):
    """Return 0 on success, 1 on failure. Writes nothing unless --execute."""
    args = build_parser().parse_args(argv)

    if args.execute:
        if not args.go_file:
            print("--execute requires --go-file", file=sys.stderr)
            return 1
        try:
            outcome = run_fit(args.go_file, progress=args.progress)
        except RuntimeError as error:
            print(str(error), file=sys.stderr)
            return 1
        print(json.dumps({"leaf": outcome["leaf"], "verdict": outcome["verdict"]}, indent=2))
        return 0 if outcome["verification"]["ok"] else 1

    report = run_preflight(verbose=True)
    if args.dry_run:
        print("\ncandidate grid, in preregistered enumeration order:")
        for entry in report["grid"]:
            print(
                "  %2d  lambda=%-5s beta=%-4s lr=%s"
                % (
                    entry["candidate_index"],
                    entry["preservation_weight_lambda"],
                    entry["on_policy_weight_beta"],
                    entry["learning_rate"],
                )
            )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
