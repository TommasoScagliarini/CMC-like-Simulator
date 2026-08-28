"""V26C J8 - the recovery fit: one actor-only supervised update of the FULL mean network.

WHAT IT DOES
    Starts from the J2 parent, trains the entire mean network on the MATERIALISED J7 Markov
    recovery dataset, and writes one fitted actor. It fits nothing else.

WHAT "JULY-FAITHFUL" MEANS HERE, PRECISELY
    J8 replicates the ALGORITHM, the SCALING, the HYPERPARAMETERS, and July's DOCUMENTED SEEDING
    PROTOCOL together with its Generator / split / shuffle ORDER. It does NOT replicate July's
    dataset, parent, actor or output, and it is not a reproduction of that run. The operational
    lineage is the AUGUST V26 J2 parent and the CURRENT J7 dataset of 16713 rows. July's
    multistart training block is absent and deferred.

    It makes NO claim about the deterministic-algorithms backend or the thread count. Those were
    never recorded for the July run and the code snapshot closest to it contains neither call, so
    J8 does not force them and does not claim to replicate them. Their observed values are
    reported as diagnostics at fit time.

WHAT IT NEVER DOES
    No critic, no PPO, no environment, no rollout, no closed-loop claim, no promotion, no
    deployability. It never rebuilds the J7 dataset and never imports the J7 builder: it reads
    the materialised bytes. It never imports v26c_j4_recovery either - J4 is a reading reference
    for the shape of the loop, nothing more.

THE ACTOR
    35D, one of them, no widening and no contralateral feature.
    trainable   the six DIRECT parameters pi.0.0.{weight,bias}, pi.0.2.{weight,bias},
                pi.1.{weight,bias}
    aliases     pi_encoder.* are ALIASES, never optimised, written at the end as bit-identical
                copies and verified as such
    logstd      output rows [2:] of pi.1.* are restored from the parent after EVERY step and
                verified bit-identical to the parent at the end
    clock       columns [0, 1] of the first layer are zeroed after EVERY step and again before
                saving, in both aliases
    controller  columns 25..34 are NOT masked and must end with a strictly positive norm

THE SCALING
    July's MARKOV_CONTROLLER_FEATURE_SCALES, extracted by AST from the PINNED source and compared
    against the transcription here. The fit runs on obs / scale; after the best state is restored
    the scale is ABSORBED into the first layer, so the saved module consumes RAW observations.

THE WRITE
    Atomic commit, no-clobber under an exclusive fit lock, into exactly one leaf. On failure the
    run removes only the staging directory and lock it created, and stops: no autonomous retry.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]


class J8Error(RuntimeError):
    pass


STAGE = "V26C-J8-RECOVERY-FIT"
PREREG = HERE / "v26c_j8_prereg_recovery_fit.json"
PIN_PREREG = "8f661320348f6b6f034bbbe7b2c04ca85efc10cac48375349261ce2dbcba737b"

# ------------------------------------------------------------------ inputs ----------------------
J7_LEAF = HERE / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1"
DATASET = J7_LEAF / "v26c_j7_markov_recovery_dataset.npz"
PIN_DATASET = "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
J7_RECEIPT = J7_LEAF / "v26c_j7_markov_dataset_receipt.json"
PIN_J7_RECEIPT = "39cbed8072f0126b84ba40a12d1268991a128e5af3794d6fb74b246c2d509ca2"
PIN_OBS_CONTENT = "a59fb89b43a372162661ec8cf131b5abbf1e9ce32b16f98518af9010a4cf587a"
PIN_ACT_CONTENT = "6e920c2f890dbdc2cfacf6a2c6842b34c6f33e24499d0d8716bd6347624bde4f"

PARENT_MODULE_DIR = HERE / "j2_runs" / "j2_base_v26c_2026-08-26_r1" / "rl_module"
PIN_PARENT_STATE = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
SIDECARS = {
    "class_and_ctor_args.pkl": "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
}

# The July scaling authority. Pinned and compared, never imported: importing it pulls torch and
# the preflight must stay free of it.
JULY_SCALES_SOURCE = REPO / "Trajectory Generator" / "baseline_MLP" / \
    "target_domain_markov_adaptation.py"
PIN_JULY_SCALES_SOURCE = "7518f1d4bdaa31505d8335934ce3478ebc03dc1cb1f31b508b5ec4a229cba7e0"
JULY_SCALES_SYMBOL = "MARKOV_CONTROLLER_FEATURE_SCALES"
MARKOV_CONTROLLER_FEATURE_SCALES: dict[str, float] = {
    "pros_knee_angle_previous_endpoint": 1.0,
    "pros_knee_angle_served_ref": 1.0,
    "pros_knee_angle_served_ref_vel": 4.0,
    "pros_knee_angle_served_ref_accel": 60.0,
    "pros_knee_angle_sea_u": 1.0,
    "pros_ankle_angle_previous_endpoint": 1.0,
    "pros_ankle_angle_served_ref": 1.0,
    "pros_ankle_angle_served_ref_vel": 3.5,
    "pros_ankle_angle_served_ref_accel": 55.0,
    "pros_ankle_angle_sea_u": 1.0,
}

# EVIDENCE / READING REFERENCE ONLY. Never imported, never a parent, never data.
J4_REFERENCE = HERE / "v26c_j4_recovery.py"
PIN_J4_REFERENCE = "24dceef98242cc9e02f63d0442e8361bae32994c0829859b1747f0e7091a6841"

# ------------------------------------------------------------------ composition -----------------
ACTOR_WIDTH = 35
ACTION_DIM = 2
CLOCK_COLUMNS = (0, 1)
TOTAL_ROWS = 16713
NOMINAL_UNIQUE = 500
NOMINAL_REPEAT = 32
NOMINAL_ROWS = NOMINAL_UNIQUE * NOMINAL_REPEAT
RECOVERY_ROWS = TOTAL_ROWS - NOMINAL_ROWS
DIRECT_KEYS = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias",
               "pi.1.weight", "pi.1.bias")
ALIAS_PAIRS = (("pi_encoder.0.weight", "pi.0.0.weight"), ("pi_encoder.0.bias", "pi.0.0.bias"),
               ("pi_encoder.2.weight", "pi.0.2.weight"), ("pi_encoder.2.bias", "pi.0.2.bias"))
STATE_KEYS = tuple(a for a, _ in ALIAS_PAIRS) + DIRECT_KEYS

# ------------------------------------------------------------------ hyperparameters -------------
JULY_HP: dict[str, Any] = {
    "seed": 123,
    "epochs": 400,
    "batch_size": 128,
    "learning_rate": 5e-05,
    "validation_fraction": 0.2,
    "patience": 60,
    "clip_weight": 1.0,
    "logstd_weight": 0.0,
    "anchor_weight": 0.01,
    "trainable_first_layer_features": None,
}
BEST_EPSILON = 1e-9

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j8_runs", "j8_recovery_fit_v26c_2026-08-26_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
MODULE_DIRNAME = "rl_module"
STATE_NAME = "module_state.pkl"
RECEIPT_NAME = "v26c_j8_recovery_fit_receipt.json"
HISTORY_NAME = "history.json"

# The ROOT of the destination, and nothing else, may be redirected. Production leaves this None.
# A test sets it to a temporary directory; the receipt then declares itself non-authoritative.
OUTPUT_ROOT_OVERRIDE: Path | None = None

FORBIDDEN_HERE = ("critic", "PPO", "environment construction", "rollout", "closed-loop evaluation",
                  "promotion", "deployability", "rebuilding the J7 dataset",
                  "importing the J7 builder", "importing v26c_j4_recovery",
                  "a second parent", "widening", "contralateral features", "LOTO", "LOCO",
                  "multistart training data", "an autonomous retry", "an invented threshold")


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_array(a: np.ndarray) -> str:
    arr = np.ascontiguousarray(a)
    h = hashlib.sha256()
    h.update(str(arr.dtype).encode())
    h.update(str(arr.shape).encode())
    h.update(arr.tobytes())
    return h.hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, allow_nan=False,
                                     default=str).encode("utf-8")).hexdigest()


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


# ================================================================ inputs =========================

def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J8Error("the J8 preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J8Error(f"the J8 preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage_authorised") != STAGE:
        raise J8Error(f"the preregistration names {data.get('stage_authorised')!r}, not {STAGE}")
    local: dict[str, str] = {}
    for rel, pin in data["pinned_artefacts_sha256"].items():
        target = HERE / rel
        if not target.is_file():
            raise J8Error(f"the preregistration pins {rel}, which is missing")
        got = _sha_file(target)
        if got != pin:
            raise J8Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        local[rel] = got
    repo: dict[str, str] = {}
    for rel, pin in data["pinned_repo_artefacts_sha256"].items():
        target = REPO / rel
        if not target.is_file():
            raise J8Error(f"the preregistration pins {rel}, which is missing")
        got = _sha_file(target)
        if got != pin:
            raise J8Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        repo[rel] = got
    hp = data["hyperparameters"]
    for key, value in JULY_HP.items():
        if hp.get(key) != value:
            raise J8Error(f"the preregistration and the runner disagree on {key}: "
                          f"{hp.get(key)!r} != {value!r}")
    if data["future_leaf"]["relative_leaf"] != RELATIVE_LEAF:
        raise J8Error("the preregistration and the runner disagree on the destination")
    return {"file": _rel(PREREG), "sha256": digest,
            "manifest_entries": len(local) + len(repo),
            "pinned_artefacts_sha256": local, "pinned_repo_artefacts_sha256": repo}


def july_scales_from_source() -> dict[str, float]:
    """Extract the scale literal from the PINNED July source by AST. Never imports it."""
    digest = _sha_file(JULY_SCALES_SOURCE)
    if digest != PIN_JULY_SCALES_SOURCE:
        raise J8Error(f"the July scaling source changed: {digest} != {PIN_JULY_SCALES_SOURCE}")
    tree = ast.parse(JULY_SCALES_SOURCE.read_text(encoding="utf-8"))
    found: dict[str, float] | None = None
    for node in tree.body:
        named = False
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            named = node.target.id == JULY_SCALES_SYMBOL
        elif isinstance(node, ast.Assign):
            named = any(isinstance(t, ast.Name) and t.id == JULY_SCALES_SYMBOL
                        for t in node.targets)
        if named and node.value is not None:
            found = {str(k): float(v) for k, v in ast.literal_eval(node.value).items()}
            break
    if found is None:
        raise J8Error(f"{JULY_SCALES_SYMBOL} is not defined in {JULY_SCALES_SOURCE}")
    if found != MARKOV_CONTROLLER_FEATURE_SCALES:
        raise J8Error(f"the transcribed scales differ from the pinned source: {found} != "
                      f"{MARKOV_CONTROLLER_FEATURE_SCALES}")
    return found


def actor_feature_names() -> tuple[str, ...]:
    manifest = json.loads((PARENT_MODULE_DIR / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J8Error(f"the manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


def controller_indices(names: Sequence[str]) -> tuple[int, ...]:
    """Resolved by FEATURE NAME, never positionally."""
    idx = []
    for name in MARKOV_CONTROLLER_FEATURE_SCALES:
        if name not in names:
            raise J8Error(f"the scaled feature {name} is absent from the actor manifest")
        idx.append(int(list(names).index(name)))
    return tuple(sorted(idx))


def scale_vector(names: Sequence[str]) -> np.ndarray:
    scales = np.ones(len(names), dtype=np.float32)
    for name, value in MARKOV_CONTROLLER_FEATURE_SCALES.items():
        scales[list(names).index(name)] = np.float32(value)
    return scales


def load_dataset() -> dict[str, Any]:
    """Read the MATERIALISED J7 bytes. The builder is never imported and nothing is rebuilt."""
    digest = _sha_file(DATASET)
    if digest != PIN_DATASET:
        raise J8Error(f"the J7 dataset changed: {digest} != {PIN_DATASET}")
    receipt_digest = _sha_file(J7_RECEIPT)
    if receipt_digest != PIN_J7_RECEIPT:
        raise J8Error(f"the J7 receipt changed: {receipt_digest} != {PIN_J7_RECEIPT}")
    with np.load(DATASET, allow_pickle=False) as archive:
        keys = tuple(sorted(archive.files))
        if keys != ("actions", "actor_feature_names", "observations"):
            raise J8Error(f"the dataset holds {keys}, not the three authorised keys")
        obs = np.asarray(archive["observations"])
        act = np.asarray(archive["actions"])
        names = tuple(str(n) for n in np.asarray(archive["actor_feature_names"]).tolist())
    if obs.shape != (TOTAL_ROWS, ACTOR_WIDTH) or act.shape != (TOTAL_ROWS, ACTION_DIM):
        raise J8Error(f"the dataset is {obs.shape}/{act.shape}, expected "
                      f"{(TOTAL_ROWS, ACTOR_WIDTH)}/{(TOTAL_ROWS, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J8Error(f"the dataset is {obs.dtype}/{act.dtype}, expected float32")
    if _sha_array(obs) != PIN_OBS_CONTENT or _sha_array(act) != PIN_ACT_CONTENT:
        raise J8Error("the dataset arrays do not reproduce the audited content hashes")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J8Error("the dataset holds non-finite values")
    if names != actor_feature_names():
        raise J8Error("the dataset schema differs from the parent's actor manifest")
    # the declared layout: 32 bit-identical repeats, then the recovery rows
    for r in range(NOMINAL_REPEAT):
        block = slice(r * NOMINAL_UNIQUE, (r + 1) * NOMINAL_UNIQUE)
        if not (np.array_equal(obs[block], obs[:NOMINAL_UNIQUE])
                and np.array_equal(act[block], act[:NOMINAL_UNIQUE])):
            raise J8Error(f"nominal repeat {r} is not bit-identical to the first block")
    clock = list(CLOCK_COLUMNS)
    if float(np.max(np.abs(obs[:, clock]))) != 0.0:
        raise J8Error("the dataset's clock columns are not exactly zero")
    return {"observations": obs, "actions": act, "names": names, "sha256": digest,
            "receipt_sha256": receipt_digest,
            "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act)}}


def load_parent_state() -> dict[str, np.ndarray]:
    """Fresh from disk, every time. No cached, adapted or intermediate state is ever a parent."""
    path = PARENT_MODULE_DIR / STATE_NAME
    digest = _sha_file(path)
    if digest != PIN_PARENT_STATE:
        raise J8Error(f"the J2 parent state changed: {digest} != {PIN_PARENT_STATE}")
    for name, pin in SIDECARS.items():
        got = _sha_file(PARENT_MODULE_DIR / name)
        if got != pin:
            raise J8Error(f"the parent sidecar {name} changed: {got} != {pin}")
    with path.open("rb") as fh:
        state = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}
    if tuple(sorted(state)) != tuple(sorted(STATE_KEYS)):
        raise J8Error(f"the parent holds {tuple(sorted(state))}, expected {sorted(STATE_KEYS)}")
    if any("critic" in k or k.startswith("vf") for k in state):
        raise J8Error("the parent carries a critic key; this stage excludes the critic entirely")
    return state


def _numpy_forward(state: Mapping[str, Any], x: np.ndarray) -> np.ndarray:
    """The mean network in float32 numpy. Torch-free, so the preflight can use it."""
    h = np.tanh(np.asarray(x, np.float32) @ np.asarray(state["pi.0.0.weight"]).T
                + np.asarray(state["pi.0.0.bias"]))
    h = np.tanh(h @ np.asarray(state["pi.0.2.weight"]).T + np.asarray(state["pi.0.2.bias"]))
    return h @ np.asarray(state["pi.1.weight"]).T + np.asarray(state["pi.1.bias"])


def parent_preconditions(parent: Mapping[str, np.ndarray], names: Sequence[str],
                         observations: np.ndarray) -> dict[str, Any]:
    """BINDING, before the optimizer exists. A violation aborts and no leaf is created.

    Two conditions, and the second is what makes the inverse-scale absorption legitimate:

      1. every one of the ten controller columns 25:35 of the parent's first actor layer is
         EXACTLY 0.0;
      2. with the parent's weights used directly, the mean network gives the SAME output on raw
         observations as on physically scaled ones, with max_abs_diff EXACTLY 0.0.

    (2) follows from (1): the only columns whose scale differs from 1.0 lie inside the bit-zero
    block, so a zero weight times either input contributes the same signed zero and every term of
    the accumulation is bit-identical. It is MEASURED here, never assumed.
    """
    controller = list(controller_indices(names))
    W = np.asarray(parent["pi.0.0.weight"])
    per_column = {}
    for c in controller:
        col = W[:, c]
        per_column[names[c]] = {"index": int(c), "norm": float(np.linalg.norm(col)),
                                "max_abs": float(np.abs(col).max()),
                                "bit_zero": bool(np.all(col == 0.0))}
    all_zero = bool(np.all(W[:, controller] == 0.0))
    if not all_zero:
        offenders = sorted(n for n, v in per_column.items() if not v["bit_zero"])
        raise J8Error(f"the parent's controller columns are not bit-zero: {offenders}. The "
                      f"inverse-scale absorption is only valid when they are, so this aborts "
                      f"before the optimizer exists.")

    scales = scale_vector(names)
    scaled_columns = [int(i) for i in range(len(names)) if float(scales[i]) != 1.0]
    raw_out = _numpy_forward(parent, np.asarray(observations, np.float32))
    scaled_out = _numpy_forward(parent, (np.asarray(observations, np.float32) / scales
                                         ).astype(np.float32))
    max_abs_diff = float(np.max(np.abs(raw_out.astype(np.float64)
                                       - scaled_out.astype(np.float64))))
    bit_identical = bool(np.array_equal(raw_out, scaled_out))
    if max_abs_diff != 0.0 or not bit_identical:
        raise J8Error(f"the parent is not equivalent on raw and physically scaled inputs: "
                      f"max_abs_diff {max_abs_diff}, bit_identical {bit_identical}. No tolerance "
                      f"is assumed here; exact equivalence is what justifies absorbing the scale "
                      f"at save time.")
    return {
        "controller_columns_bit_zero": {
            "all_bit_zero": all_zero, "columns": per_column,
            "binding": True,
            "why": "the inverse-scale absorption is only valid on a parent whose scaled columns "
                   "start at exactly zero"},
        "raw_vs_scaled_equivalence": {
            "max_abs_diff": max_abs_diff, "bit_identical": bit_identical,
            "kernel": "numpy float32", "tolerance": "NONE - exactly 0.0 is required",
            "measured_not_assumed": True,
            "scaled_columns": scaled_columns,
            "scaled_columns_inside_bit_zero_block": all(c in controller for c in scaled_columns),
            "justifies": "absorbing the scale into pi.0.0.weight at save time: at initialisation "
                         "the raw-input and scaled-input representations of the parent coincide "
                         "exactly, so the absorption changes no behaviour it should not"},
    }


def build_split(rows: int) -> dict[str, Any]:
    """July's split, transcribed from target_domain_imitation._resolve_adaptation_split.

        indices          = rng.permutation(sample_count)
        validation_count = max(1, int(round(len(indices) * validation_fraction)))
        resolved_validation = np.sort(indices[:validation_count])
        resolved_training   = np.asarray(indices[validation_count:], dtype=int)

    ONLY the validation half is sorted. The training half KEEPS the order the initial
    permutation gave it, and that order is not cosmetic: every epoch shuffle is
    rng.permutation(train_idx), which permutes positions, so a sorted train_idx would
    produce a different batch sequence in every epoch. Sorting it would silently break
    July fidelity.

    dtype is np.int64 explicitly. July writes dtype=int, which is int64 on macOS/Linux; the
    explicit width keeps Windows identical instead of platform-dependent.
    """
    rng = np.random.default_rng(int(JULY_HP["seed"]))
    indices = rng.permutation(rows)
    n_val = max(1, int(round(len(indices) * float(JULY_HP["validation_fraction"]))))
    val_idx = np.sort(indices[:n_val]).astype(np.int64)              # membership, sorted
    train_idx = np.asarray(indices[n_val:], dtype=np.int64)          # ORDER PRESERVED
    if not len(train_idx):
        raise J8Error("the validation split left no training samples")
    return {"rng": rng, "val_idx": val_idx, "train_idx": train_idx,
            "n_val": int(val_idx.size), "n_train": int(train_idx.size),
            # membership digests, sorted on both sides: comparable to July's _split_index_digest,
            # which commits to membership and not to caller ordering
            "digest": _sha_obj({"val": sorted(int(v) for v in val_idx),
                                "train": sorted(int(v) for v in train_idx)}),
            "val_membership_sha256": _sha_array(np.sort(val_idx)),
            "train_membership_sha256": _sha_array(np.sort(train_idx)),
            # ORDER-SENSITIVE hashes: these are what actually determine the epoch batching
            "val_sha256": _sha_array(val_idx), "train_sha256": _sha_array(train_idx),
            "train_order_preserved": True,
            "train_is_sorted": bool(np.array_equal(train_idx, np.sort(train_idx))),
            "validation_sorted": True,
            "dtype": str(train_idx.dtype),
            "rule": "initial permutation first; validation membership sorted; training order "
                    "preserved; the SAME Generator then produces the epoch shuffles",
            "row_level": True, "group_split": False}


# ================================================================ destination guards ==============

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J8Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J8Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J8Error("--fit requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J8Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J8Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J8Error(f"the authorised leaf already exists; this stage is no-clobber and "
                      f"single-execution: {leaf}")
    return leaf


# ================================================================ the fit ========================

def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((np.asarray(a, dtype=np.float64)
                                  - np.asarray(b, dtype=np.float64)) ** 2)))


def run_fit(*, progress: bool = False) -> dict[str, Any]:
    """Train the full mean network IN MEMORY and audit the result. Writes nothing."""
    prereg = verify_prereg()
    scales_checked = july_scales_from_source()
    data = load_dataset()
    parent = load_parent_state()
    names = data["names"]
    obs, act = data["observations"], data["actions"]
    clock = list(CLOCK_COLUMNS)
    controller = list(controller_indices(names))
    scales = scale_vector(names)

    # BINDING preconditions. Pure numpy, so they run before torch is anywhere near the process.
    preconditions = parent_preconditions(parent, names, obs)

    # ---- July's executive order, transcribed from target_domain_imitation.adapt_actor ----------
    # (1) import torch, in the --fit path only
    # (2) torch.manual_seed(seed)
    # (3) np.random.seed(seed)
    # (4) deterministic initialisation from the parent: tensors, parameters, initial forward.
    #     Nothing here draws from any RNG.
    # (5) the single np.default_rng(seed), inside build_split, which draws the split FIRST and
    #     then produces every epoch shuffle
    # (6) the optimizer
    #
    # NOTE ON WHAT IS *NOT* HERE: no torch.use_deterministic_algorithms and no
    # torch.set_num_threads. Neither call exists anywhere in the repository snapshot closest to
    # the July run (bdbf99c1, 2026-07-14), the run's own adaptation_report.json records no
    # determinism setting at all, and in today's source use_deterministic_algorithms sits only in
    # the fixed_final_epoch branch, which is mutually exclusive with that run's early stopping.
    # J8 therefore does not force them, and claims no replication of them. Their OBSERVED values
    # are recorded below as diagnostics.
    import torch                                        # (1) heavy import: the FIT only
    from torch.nn import functional
    torch.manual_seed(int(JULY_HP["seed"]))             # (2)
    np.random.seed(int(JULY_HP["seed"]))                # (3) July's legacy global seed, once

    # OBSERVED, never forced: whatever the environment happens to provide.
    torch_backend = {
        "are_deterministic_algorithms_enabled":
            bool(torch.are_deterministic_algorithms_enabled()),
        "get_num_threads": int(torch.get_num_threads()),
        "forced_by_j8": False,
        "binding": False,
        "why_not_forced": "the July run recorded neither setting and the code snapshot closest "
                          "to it contains neither call; forcing them would be an invention, not "
                          "a replication",
        "observed_at": "fit time; the preflight is torch-free and cannot observe them",
    }

    # (4) deterministic initialisation from the parent. No RNG is consulted in this block.
    X = torch.as_tensor(obs / scales)
    Y = torch.as_tensor(act)
    params = {k: torch.nn.Parameter(torch.as_tensor(np.array(parent[k]))) for k in DIRECT_KEYS}
    anchor = {k: params[k].detach().clone() for k in DIRECT_KEYS}
    logstd_w = params["pi.1.weight"].detach()[ACTION_DIM:].clone()
    logstd_b = params["pi.1.bias"].detach()[ACTION_DIM:].clone()

    def forward(x: Any) -> Any:
        h = torch.tanh(functional.linear(x, params["pi.0.0.weight"], params["pi.0.0.bias"]))
        h = torch.tanh(functional.linear(h, params["pi.0.2.weight"], params["pi.0.2.bias"]))
        return functional.linear(h, params["pi.1.weight"], params["pi.1.bias"])

    def project() -> None:
        """The two standing invariants, re-imposed after EVERY optimizer step."""
        with torch.no_grad():
            params["pi.1.weight"][ACTION_DIM:].copy_(logstd_w)
            params["pi.1.bias"][ACTION_DIM:].copy_(logstd_b)
            params["pi.0.0.weight"][:, clock].zero_()

    project()
    with torch.no_grad():
        source_logstd = forward(X)[:, ACTION_DIM:].clone()
        before = forward(torch.as_tensor(obs))[:, :ACTION_DIM].numpy()
        # the same equivalence, re-measured in the kernel that actually trains
        torch_scaled = forward(X)[:, :ACTION_DIM].numpy()
    torch_diff = float(np.max(np.abs(before.astype(np.float64)
                                     - torch_scaled.astype(np.float64))))
    if torch_diff != 0.0 or not np.array_equal(before, torch_scaled):
        raise J8Error(f"the torch kernel does not reproduce the raw/scaled equivalence: "
                      f"max_abs_diff {torch_diff}. No tolerance is assumed.")
    preconditions["raw_vs_scaled_equivalence"]["torch_max_abs_diff"] = torch_diff
    preconditions["raw_vs_scaled_equivalence"]["torch_bit_identical"] = True
    preconditions["raw_vs_scaled_equivalence"]["kernels_checked"] = ["numpy float32", "torch"]

    # (5) the single Generator, AFTER the deterministic initialisation and BEFORE the optimizer,
    #     exactly where July creates it. It draws the split first and the epoch shuffles after.
    split = build_split(len(obs))
    rng = split["rng"]

    optimiser = torch.optim.Adam(list(params.values()),        # (6)
                                 lr=float(JULY_HP["learning_rate"]))
    best = {"epoch": 0, "val": float("inf"),
            "state": {k: v.detach().clone() for k, v in params.items()}}
    stale = 0
    history: list[dict[str, float]] = []
    batch = int(JULY_HP["batch_size"])
    stopped_early = False
    for epoch in range(1, int(JULY_HP["epochs"]) + 1):
        shuffled = rng.permutation(split["train_idx"])
        losses: list[float] = []
        for start in range(0, len(shuffled), batch):
            idx = shuffled[start:start + batch]
            logits = forward(X[idx])
            means = logits[:, :ACTION_DIM]
            mean_loss = functional.mse_loss(means, Y[idx])
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = functional.mse_loss(logits[:, ACTION_DIM:], source_logstd[idx])
            anchor_loss = torch.stack([(params[k] - anchor[k]).square().mean()
                                       for k in DIRECT_KEYS]).mean()
            loss = (mean_loss
                    + float(JULY_HP["clip_weight"]) * clip_loss
                    + float(JULY_HP["logstd_weight"]) * logstd_loss
                    + float(JULY_HP["anchor_weight"]) * anchor_loss)
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            optimiser.step()
            project()
            losses.append(float(loss.item()))
        with torch.no_grad():
            val = float(functional.mse_loss(forward(X[split["val_idx"]])[:, :ACTION_DIM],
                                            Y[split["val_idx"]]).item())
        history.append({"epoch": float(epoch), "train_loss": float(np.mean(losses)),
                        "validation_mse": val})
        if val < best["val"] - BEST_EPSILON:
            best = {"epoch": epoch, "val": val,
                    "state": {k: v.detach().clone() for k, v in params.items()}}
            stale = 0
        else:
            stale += 1
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps({"epoch": epoch, "train": history[-1]["train_loss"], "val": val}),
                  flush=True)
        if stale >= int(JULY_HP["patience"]):
            stopped_early = True
            break

    with torch.no_grad():
        for k in DIRECT_KEYS:
            params[k].copy_(best["state"][k])
        project()
        # absorb the physical scales: the saved actor consumes RAW observations
        for name, value in MARKOV_CONTROLLER_FEATURE_SCALES.items():
            params["pi.0.0.weight"][:, list(names).index(name)].div_(float(value))
        params["pi.0.0.weight"][:, clock].zero_()
        after = forward(torch.as_tensor(obs))[:, :ACTION_DIM].numpy()

    final = {k: params[k].detach().numpy().astype(np.float32) for k in DIRECT_KEYS}
    for alias, direct in ALIAS_PAIRS:
        final[alias] = final[direct].copy()

    return {"final": final, "parent": parent, "prereg": prereg, "data": data, "split": split,
            "history": history, "best_epoch": int(best["epoch"]),
            "best_validation_mse": float(best["val"]), "stopped_early": bool(stopped_early),
            "epochs_run": len(history), "before": before, "after": after,
            "names": names, "clock": clock, "controller": controller,
            "scales_verified": scales_checked, "preconditions": preconditions,
            "torch_backend_observed": torch_backend,
            "seed_order": ["import torch", "torch.manual_seed", "np.random.seed",
                           "deterministic init from parent (no RNG)",
                           "np.default_rng in build_split", "optimizer"],
            "numpy_legacy_seed": {"call": "np.random.seed", "value": int(JULY_HP["seed"]),
                                  "count": 1, "is_a_generator": False,
                                  "position": "after torch.manual_seed, before the Generator, "
                                              "as in July's adapt_actor",
                                  "note": "July's legacy global seed, carried for protocol "
                                          "fidelity; it is not a second Generator"}}


# ================================================================ audit and gate =================

def audit(fitted: Mapping[str, Any]) -> dict[str, Any]:
    """Every invariant and every metric, computed BEFORE a byte is written."""
    final = fitted["final"]
    parent = fitted["parent"]
    obs = fitted["data"]["observations"]
    act = fitted["data"]["actions"]
    before, after = fitted["before"], fitted["after"]
    clock, controller = fitted["clock"], fitted["controller"]
    names = fitted["names"]

    # ---- INTEGRITY: a violation here means the artefact is unsound -------------------------
    integrity: dict[str, bool] = {}
    integrity["keys_and_shapes_match_parent"] = (
        tuple(sorted(final)) == tuple(sorted(parent))
        and all(final[k].shape == parent[k].shape for k in parent)
        and all(final[k].dtype == np.float32 for k in final))
    integrity["all_parameters_finite"] = all(bool(np.all(np.isfinite(v))) for v in final.values())
    integrity["clock_bit_zero"] = all(
        float(np.max(np.abs(final[a][:, clock]))) == 0.0
        for a in ("pi.0.0.weight", "pi_encoder.0.weight"))
    integrity["aliases_bit_identical"] = all(np.array_equal(final[a], final[d])
                                             for a, d in ALIAS_PAIRS)
    integrity["logstd_bit_identical_to_parent"] = (
        np.array_equal(final["pi.1.weight"][ACTION_DIM:], parent["pi.1.weight"][ACTION_DIM:])
        and np.array_equal(final["pi.1.bias"][ACTION_DIM:], parent["pi.1.bias"][ACTION_DIM:]))
    integrity["no_critic_key"] = not any("critic" in k or k.startswith("vf") for k in final)
    integrity["inputs_unchanged"] = (_sha_file(DATASET) == PIN_DATASET
                                     and _sha_file(PARENT_MODULE_DIR / STATE_NAME)
                                     == PIN_PARENT_STATE
                                     and all(_sha_file(PARENT_MODULE_DIR / n) == p
                                             for n, p in SIDECARS.items()))

    # ---- OUTCOME: recorded even when it fails, because the evidence is worth keeping --------
    nominal_states = slice(0, NOMINAL_UNIQUE)
    recovery = slice(NOMINAL_ROWS, TOTAL_ROWS)
    recovery_before = _rmse(before[recovery], act[recovery])
    recovery_after = _rmse(after[recovery], act[recovery])
    norms = {names[c]: float(np.linalg.norm(final["pi.0.0.weight"][:, c])) for c in controller}
    outcome: dict[str, bool] = {
        "recovery_rmse_decreases": bool(recovery_after < recovery_before),
        "controller_columns_nonzero": all(v > 0.0 for v in norms.values()),
    }

    shift = after[nominal_states].astype(np.float64) - before[nominal_states].astype(np.float64)

    def oob(block: slice) -> dict[str, Any]:
        m = np.abs(after[block])
        return {"rows": int(m.shape[0]), "count": int(np.sum(m > 1.0)),
                "fraction": float(np.mean(m > 1.0)), "max_abs": float(m.max())}

    diagnostics = {
        "recovery_rmse": {"before": recovery_before, "after": recovery_after,
                          "delta": recovery_after - recovery_before,
                          "rows": int(RECOVERY_ROWS)},
        "nominal_rmse": {
            "before": _rmse(before[nominal_states], act[nominal_states]),
            "after": _rmse(after[nominal_states], act[nominal_states]),
            "note": "the nominal labels ARE the parent's own means, so 'before' is ~0 by "
                    "construction and only the numpy/torch kernel difference separates it from "
                    "exactly 0. This is a diagnostic, never a gate."},
        "nominal_shift_vs_parent": {
            "rms": float(np.sqrt(np.mean(shift ** 2))),
            "max_abs": float(np.max(np.abs(shift))),
            "per_action_rms": [float(v) for v in np.sqrt(np.mean(shift ** 2, axis=0))]},
        "clipping_out_of_bounds": {"aggregate": oob(slice(0, TOTAL_ROWS)),
                                   "nominal_block": oob(slice(0, NOMINAL_ROWS)),
                                   "recovery_block": oob(recovery),
                                   "convention": "|mean| > 1.0"},
        "best_validation_mse": fitted["best_validation_mse"],
        "controller_column_norms": norms,
        "binding": False,
        "no_closed_loop_claim": "nothing here was evaluated in closed loop",
    }

    # the best epoch must be recomputable from the history under the stated rule
    replay_best, replay_val = 0, float("inf")
    for record in fitted["history"]:
        if float(record["validation_mse"]) < replay_val - BEST_EPSILON:
            replay_best, replay_val = int(record["epoch"]), float(record["validation_mse"])
    reconstructible = (replay_best == fitted["best_epoch"]
                       and abs(replay_val - fitted["best_validation_mse"]) <= 0.0)
    integrity["best_state_reconstructible_from_history"] = bool(reconstructible)

    metrics_finite = all(np.isfinite(v) for v in
                         [recovery_before, recovery_after, fitted["best_validation_mse"]]
                         + [float(r["train_loss"]) for r in fitted["history"]]
                         + [float(r["validation_mse"]) for r in fitted["history"]])
    integrity["all_metrics_finite"] = bool(metrics_finite)

    return {"integrity": integrity, "outcome": outcome, "diagnostics": diagnostics,
            "selection": {"best_epoch": fitted["best_epoch"],
                          "best_validation_mse": fitted["best_validation_mse"],
                          "rule": f"validation_mse < best - {BEST_EPSILON}",
                          "epochs_run": fitted["epochs_run"],
                          "stopped_early": fitted["stopped_early"],
                          "patience": int(JULY_HP["patience"]),
                          "recomputed_best_epoch": replay_best,
                          "reconstructible": bool(reconstructible)},
            "verdict": "PASS" if all(integrity.values()) and all(outcome.values()) else "FAIL",
            "failed": sorted([k for k, v in integrity.items() if not v]
                             + [k for k, v in outcome.items() if not v])}


# ================================================================ preflight (READ-ONLY) ==========

def preflight() -> dict[str, Any]:
    """Verify every input and every guard, and write NOTHING. No torch, no optimizer, no fit."""
    torch_before = "torch" in sys.modules
    prereg = verify_prereg()
    scales = july_scales_from_source()
    data = load_dataset()
    parent = load_parent_state()
    names = data["names"]
    controller = list(controller_indices(names))
    split = build_split(len(data["observations"]))
    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME

    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a fit lock is already held or was left behind: {lock_path}. This stage "
                        f"removes no lock it does not own.")
    # torch_imported is REPORTED, not a blocker: whether some other caller in this process has
    # already imported torch says nothing about whether this preflight wrote or trained anything.

    obs = data["observations"]
    ctrl_norms = {names[c]: float(np.linalg.norm(obs[:, c])) for c in controller}
    # the two binding preconditions, exercised read-only exactly as the fit will exercise them
    preconditions = parent_preconditions(parent, names, obs)

    torch_after = "torch" in sys.modules
    imported_here = bool(torch_after and not torch_before)
    if imported_here:
        blockers.append("the preflight introduced torch into the process")
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"optimizer_constructed": False, "fit_executed": False,
                  "environment_constructed": False, "rollout_executed": False,
                  "critic_touched": False, "ppo_updates": 0,
                  "torch_imported": torch_after,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "module_written": False, "receipt_written": False,
                  "note": "inputs are verified, the preconditions are measured and the split is "
                          "drawn in memory; nothing else"},
        "july_faithfulness": {
            "replicates": ["the algorithm", "the physical scaling", "the hyperparameters",
                           "the documented seeding protocol",
                           "the Generator / split / shuffle order"],
            "makes_no_claim_about": ["the deterministic-algorithms backend", "the thread count"],
            "does_not_replicate": ["July's dataset", "July's parent", "July's actor",
                                   "July's output"],
            "is_not": "a reproduction of the July run",
            "operational_lineage": "the August V26 J2 parent and the current J7 dataset, 16713 "
                                   "rows",
            "july_multistart_training_block": "ABSENT / DEFERRED"},
        "torch_provenance": {
            "torch_present_before": torch_before, "torch_present_after": torch_after,
            "torch_imported_by_preflight": imported_here,
            "requirement": "torch_imported_by_preflight must be false",
            "pre_existing_is_observation_only": "a torch already loaded by some other caller is "
                                                "recorded, never treated as a blocker",
            "backend_values": "are_deterministic_algorithms_enabled and get_num_threads will be "
                              "OBSERVED at fit time and recorded in the receipt as non-binding "
                              "diagnostics. This preflight is torch-free, does not observe them, "
                              "and assumes nothing about them.",
            "forced_by_j8": False},
        "parent_preconditions": preconditions,
        "preregistration": prereg,
        "inputs": {
            "dataset": {"file": _rel(DATASET), "sha256": data["sha256"],
                        "content_hashes": data["content_hashes"],
                        "observations_shape": list(obs.shape),
                        "actions_shape": list(data["actions"].shape),
                        "dtype": "float32/float32",
                        "nominal_unique": NOMINAL_UNIQUE, "nominal_repeat": NOMINAL_REPEAT,
                        "nominal_rows": NOMINAL_ROWS, "recovery_rows": RECOVERY_ROWS,
                        "repeats_bit_identical": True, "clock_exactly_zero_in_data": True,
                        "loaded_directly": True, "rebuilt": False, "builder_imported": False},
            "j7_receipt": {"file": _rel(J7_RECEIPT), "sha256": data["receipt_sha256"]},
            "parent": {"module": _rel(PARENT_MODULE_DIR), "module_state_sha256": PIN_PARENT_STATE,
                       "sidecars_sha256": dict(SIDECARS), "keys": sorted(parent),
                       "loaded_fresh": True, "critic_present": False,
                       "is_the_only_parent": True},
            "j4_reference": {"file": _rel(J4_REFERENCE), "sha256": PIN_J4_REFERENCE,
                             "role": "reading reference for the loop only; never imported"},
        },
        "physical_scaling": {
            "source": _rel(JULY_SCALES_SOURCE), "source_sha256": PIN_JULY_SCALES_SOURCE,
            "symbol": JULY_SCALES_SYMBOL, "extracted_by": "AST, without importing the module",
            "matches_transcription": True, "scales": scales,
            "application": "fit on obs/scale, then absorb into pi.0.0.weight before saving",
            "saved_module_consumes": "RAW observations"},
        "columns": {
            "clock": list(CLOCK_COLUMNS), "clock_zero_in_data": True,
            "clock_zero_in_parent_weights": bool(
                np.all(parent["pi.0.0.weight"][:, list(CLOCK_COLUMNS)] == 0.0)),
            "controller": controller, "controller_masked": False,
            "controller_resolved_by_name": True,
            "controller_data_column_norms": ctrl_norms,
            "controller_data_non_degenerate": all(v > 0.0 for v in ctrl_norms.values())},
        "trainability": {
            "direct_parameters": list(DIRECT_KEYS),
            "aliases_not_parameters": [a for a, _ in ALIAS_PAIRS],
            "aliases_bit_identical_in_parent": all(
                np.array_equal(parent[a], parent[d]) for a, d in ALIAS_PAIRS),
            "logstd_rows": f"[{ACTION_DIM}:]",
            "logstd_frozen": True,
            "logstd_parent_weight_is_zero": bool(
                np.all(parent["pi.1.weight"][ACTION_DIM:] == 0.0)),
            "logstd_parent_bias": [float(v) for v in parent["pi.1.bias"][ACTION_DIM:]],
            "critic": "excluded"},
        "hyperparameters": dict(JULY_HP),
        "determinism": {
            "replicated": "July's DOCUMENTED seeding protocol and its Generator / split / "
                          "shuffle order",
            "not_replicated": "the deterministic-algorithms backend and the thread count. The "
                              "July run recorded neither, and the snapshot closest to it "
                              "(bdbf99c1, 2026-07-14) contains neither call. J8 does not force "
                              "them and claims no replication of them.",
            "torch_use_deterministic_algorithms": "NOT SET by J8",
            "torch_num_threads": "NOT SET by J8",
            "torch_backend_observed": "observed at FIT time only; the preflight is torch-free and "
                                      "does not assume these values",
            "executive_order": ["import torch", "torch.manual_seed", "np.random.seed",
                                "deterministic init from parent (no RNG)",
                                "np.default_rng in build_split", "optimizer"],
            "executive_order_source": "target_domain_imitation.adapt_actor",
            "torch_manual_seed": int(JULY_HP["seed"]),
            "numpy_legacy_global_seed": {
                "call": "np.random.seed", "value": int(JULY_HP["seed"]), "count_in_fit_path": 1,
                "is_a_generator": False,
                "where": "the --fit path only, immediately before the single default_rng",
                "set_by_this_preflight": False,
                "note": "July's legacy global seed, carried for protocol fidelity"},
            "generators": {"call": "np.random.default_rng", "value": int(JULY_HP["seed"]),
                           "count": 1,
                           "note": "the ONLY Generator; the legacy global seed above is not one"},
            "split_first_then_epoch_shuffles": True},
        "split": {"row_level": True, "group_split": False,
                  "n_train": split["n_train"], "n_val": split["n_val"],
                  "validation_fraction": float(JULY_HP["validation_fraction"]),
                  "digest": split["digest"], "train_sha256": split["train_sha256"],
                  "val_sha256": split["val_sha256"],
                  "train_membership_sha256": split["train_membership_sha256"],
                  "val_membership_sha256": split["val_membership_sha256"],
                  "ordering": split["rule"],
                  "validation_sorted": split["validation_sorted"],
                  "train_order_preserved": split["train_order_preserved"],
                  "train_is_sorted": split["train_is_sorted"],
                  "dtype": split["dtype"],
                  "ordering_source": "target_domain_imitation._resolve_adaptation_split",
                  "why_order_matters": "each epoch shuffles with rng.permutation(train_idx), "
                                       "which permutes POSITIONS; sorting the training half "
                                       "would change every epoch's batch sequence",
                  "known_property": "the nominal block is 32 identical repeats, so the same "
                                    "nominal state appears in both partitions. Inherited from "
                                    "July, recorded, NOT corrected: the validation MSE is a "
                                    "diagnostic, never a generalisation estimate."},
        "would_write": {
            "leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF, "leaf_exists": bool(leaf.exists()),
            "files": [f"{MODULE_DIRNAME}/{STATE_NAME}"]
                     + [f"{MODULE_DIRNAME}/{n}" for n in sorted(SIDECARS)]
                     + [RECEIPT_NAME, HISTORY_NAME],
            "staging": staging.name, "lock": lock_path.name,
            "protocol": "acquire the exclusive sibling lock, stage, verify, re-check the leaf, "
                        "then commit by atomic rename",
            "protocol_limit": "the lock serialises fit instances only; os.rename on POSIX would "
                              "replace an empty directory, so nothing is claimed against a writer "
                              "that ignores the lock",
            "output_root_override": None if OUTPUT_ROOT_OVERRIDE is None
            else str(OUTPUT_ROOT_OVERRIDE)},
        "gate_specification": {
            "binding_preconditions": ["parent_controller_columns_bit_zero",
                                      "raw_vs_scaled_equivalence_exactly_zero"],
            "binding_integrity": ["keys_and_shapes_match_parent", "all_parameters_finite",
                                  "all_metrics_finite", "clock_bit_zero", "aliases_bit_identical",
                                  "logstd_bit_identical_to_parent", "no_critic_key",
                                  "inputs_unchanged",
                                  "best_state_reconstructible_from_history"],
            "binding_outcome": ["recovery_rmse_decreases", "controller_columns_nonzero"],
            "sidecars_byte_identical": "verified on the staged bytes before the commit",
            "diagnostic_nonbinding": ["nominal_rmse", "nominal_shift_vs_parent",
                                      "clipping_out_of_bounds", "best_validation_mse"],
            "enforcement": "a precondition or integrity violation aborts before any byte is "
                           "written and the leaf is never created; an outcome violation writes "
                           "the leaf atomically with verdict FAIL and exits 1, so the evidence of "
                           "the single attempt survives. Neither path retries.",
            "no_invented_thresholds": "the only comparisons are 'strictly lower than before' and "
                                      "'strictly greater than zero'"},
        "outcome_policy": {"deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False, "closed_loop": False,
                           "single_execution": True, "no_autonomous_retry": True},
        "deferred_todo": json.loads(PREREG.read_text())["deferred_todo"],
        "requires_to_fit": {"flag": "--fit", "stage_token": STAGE,
                            "out": "must equal the authorised leaf exactly"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ receipt and commit =============

def build_receipt(fitted: Mapping[str, Any], report: Mapping[str, Any], leaf: Path,
                  files: Mapping[str, str]) -> dict[str, Any]:
    split = fitted["split"]
    overridden = OUTPUT_ROOT_OVERRIDE is not None
    return {
        "schema": "v26c_j8_recovery_fit_receipt.1",
        "stage": STAGE,
        "verdict": report["verdict"],
        "failed_checks": report["failed"],
        "preregistration": fitted["prereg"],
        "inputs": {
            "dataset": {"file": _rel(DATASET), "sha256": fitted["data"]["sha256"],
                        "content_hashes": fitted["data"]["content_hashes"],
                        "loaded_directly": True, "rebuilt": False, "builder_imported": False},
            "j7_receipt": {"file": _rel(J7_RECEIPT), "sha256": fitted["data"]["receipt_sha256"]},
            "parent": {"module": _rel(PARENT_MODULE_DIR),
                       "module_state_sha256": PIN_PARENT_STATE,
                       "sidecars_sha256": dict(SIDECARS), "loaded_fresh": True},
            "july_scaling_source": {"file": _rel(JULY_SCALES_SOURCE),
                                    "sha256": PIN_JULY_SCALES_SOURCE,
                                    "scales": fitted["scales_verified"],
                                    "extracted_by": "AST, without importing the module"},
            "j4_reference": {"file": _rel(J4_REFERENCE), "sha256": PIN_J4_REFERENCE,
                             "imported": False},
        },
        "july_faithfulness": {
            "replicates": ["the algorithm", "the physical scaling", "the hyperparameters",
                           "the documented seeding protocol",
                           "the Generator / split / shuffle order"],
            "makes_no_claim_about": ["the deterministic-algorithms backend", "the thread count"],
            "does_not_replicate": ["July's dataset", "July's parent", "July's actor",
                                   "July's output"],
            "is_not": "a reproduction of the July run; no number here is comparable to a July "
                      "number as if it were the same experiment",
            "operational_lineage": "the August V26 J2 parent and the current J7 dataset, 16713 "
                                   "rows",
            "july_multistart_training_block": "ABSENT / DEFERRED"},
        "hyperparameters": dict(JULY_HP),
        "parent_preconditions": fitted["preconditions"],
        "determinism": {"replicated": "July's DOCUMENTED seeding protocol and its Generator / "
                                      "split / shuffle order",
                        "not_replicated": "the deterministic-algorithms backend and the thread "
                                          "count: never recorded for the July run, absent from "
                                          "the snapshot closest to it, so not forced and not "
                                          "claimed",
                        "torch_use_deterministic_algorithms": "NOT SET by J8",
                        "torch_num_threads": "NOT SET by J8",
                        "torch_backend_observed": fitted["torch_backend_observed"],
                        "executive_order": fitted["seed_order"],
                        "executive_order_source": "target_domain_imitation.adapt_actor",
                        "torch_manual_seed": int(JULY_HP["seed"]),
                        "numpy_legacy_global_seed": fitted["numpy_legacy_seed"],
                        "generators": {"call": "np.random.default_rng",
                                       "value": int(JULY_HP["seed"]), "count": 1,
                                       "note": "the ONLY Generator; the legacy global seed is "
                                               "not one"}},
        "split": {"row_level": True, "group_split": False,
                  "validation_fraction": float(JULY_HP["validation_fraction"]),
                  "n_train": split["n_train"], "n_val": split["n_val"],
                  "digest": split["digest"], "train_sha256": split["train_sha256"],
                  "val_sha256": split["val_sha256"],
                  "train_membership_sha256": split["train_membership_sha256"],
                  "val_membership_sha256": split["val_membership_sha256"],
                  "ordering": split["rule"],
                  "ordering_source": "target_domain_imitation._resolve_adaptation_split",
                  "validation_sorted": split["validation_sorted"],
                  "train_order_preserved": split["train_order_preserved"],
                  "train_is_sorted": split["train_is_sorted"],
                  "dtype": split["dtype"],
                  "indices_are_in_execution_order": "train_indices below are recorded in the "
                                                    "order the fit consumed them, NOT sorted",
                  "train_indices": [int(v) for v in split["train_idx"]],
                  "validation_indices": [int(v) for v in split["val_idx"]]},
        "history": fitted["history"],
        "selection": report["selection"],
        "gate": {"binding_integrity": report["integrity"], "binding_outcome": report["outcome"],
                 "verdict": report["verdict"], "failed": report["failed"],
                 "no_invented_thresholds": True},
        "diagnostics_nonbinding": report["diagnostics"],
        "output": {"leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF, "files": dict(files),
                   "write_protocol": "atomic commit, no-clobber under exclusive fit lock",
                   "lock": {"file": LOCK_NAME,
                            "acquisition": "os.open with O_CREAT | O_EXCL | O_WRONLY",
                            "serialises": "fit instances, and only those",
                            "does_not_guarantee": "nothing is claimed against a writer that "
                                                  "ignores the lock; os.rename on POSIX silently "
                                                  "replaces an existing EMPTY directory",
                            "if_it_pre_exists": "fail closed, removing no lock this run does not "
                                                "own"},
                   "cleanup_scope": {"staging": "recursive removal of the exact staging directory "
                                                "this run created",
                                     "lock": "unlinked only if this run created it",
                                     "parent": "non-recursive rmdir, only if this run created it "
                                               "and only if empty",
                                     "never": "no recursive removal of j8_runs; concurrent "
                                              "content survives"},
                   "sidecars_byte_identical_to_parent": True,
                   "output_root_override": None if not overridden else str(OUTPUT_ROOT_OVERRIDE),
                   "authoritative": not overridden},
        "inert": {"critic_touched": False, "ppo_updates": 0, "environment_constructed": False,
                  "rollout_executed": False, "closed_loop_evaluated": False,
                  "dataset_rebuilt": False},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "closed_loop": False, "single_execution": True, "no_autonomous_retry": True,
                    "note": "an offline fit qualifies nothing. Only a fresh deterministic "
                            "closed-loop revalidation could speak to behaviour, and this stage "
                            "runs none."},
        "deferred_todo": json.loads(PREREG.read_text())["deferred_todo"],
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def commit(fitted: Mapping[str, Any], report: Mapping[str, Any], leaf: Path) -> dict[str, Any]:
    """Write the audited result atomically. Separated from fit() so the write path is testable
    on synthetic input without fitting the real dataset."""
    # INTEGRITY first: an unsound artefact is never written at all.
    broken = sorted(k for k, v in report["integrity"].items() if not v)
    if broken:
        raise J8Error(f"integrity invariants violated, refusing to write anything: {broken}")

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J8Error(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    staging_created: Path | None = None
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise J8Error(
                f"the fit lock already exists: {lock_path}. Another instance holds it, or a "
                f"previous run left it behind. This stage fails closed and removes no lock it "
                f"does not own - inspect it and remove it by hand.") from exc
        lock_owned = lock_path
        try:
            os.write(fd, json.dumps({"stage": STAGE, "pid": os.getpid(),
                                     "leaf": str(leaf)}).encode("utf-8"))
        finally:
            os.close(fd)

        staging.mkdir()
        staging_created = staging
        module_dir = staging / MODULE_DIRNAME
        module_dir.mkdir()
        with (module_dir / STATE_NAME).open("wb") as fh:
            pickle.dump({k: fitted["final"][k] for k in sorted(fitted["final"])}, fh,
                        protocol=4)
        for name in SIDECARS:
            shutil.copyfile(PARENT_MODULE_DIR / name, module_dir / name)
        for name, pin in SIDECARS.items():
            if _sha_file(module_dir / name) != pin:
                raise J8Error(f"the staged sidecar {name} is not byte-identical to the parent's")
        (staging / HISTORY_NAME).write_text(
            json.dumps(fitted["history"], indent=2, allow_nan=False), encoding="utf-8")

        # round-trip the state before committing
        with (module_dir / STATE_NAME).open("rb") as fh:
            reloaded = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}
        if tuple(sorted(reloaded)) != tuple(sorted(fitted["final"])) \
                or not all(np.array_equal(reloaded[k], fitted["final"][k]) for k in reloaded):
            raise J8Error("the staged module state does not round-trip")

        files = {f"{MODULE_DIRNAME}/{STATE_NAME}": _sha_file(module_dir / STATE_NAME)}
        for name in sorted(SIDECARS):
            files[f"{MODULE_DIRNAME}/{name}"] = _sha_file(module_dir / name)
        files[HISTORY_NAME] = _sha_file(staging / HISTORY_NAME)
        receipt = build_receipt(fitted, report, leaf, files)
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, allow_nan=False, default=str), encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J8Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None
    except BaseException:
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            shutil.rmtree(staging_created, ignore_errors=True)
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass
            lock_owned = None
        if parent_created is not None:
            try:
                parent_created.rmdir()
            except OSError:
                pass
        raise
    finally:
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass

    return {"verdict": report["verdict"], "stage": STAGE, "leaf": _rel(leaf),
            "failed_checks": report["failed"],
            "files": {**files, RECEIPT_NAME: _sha_file(leaf / RECEIPT_NAME)},
            "selection": report["selection"],
            "recovery_rmse": report["diagnostics"]["recovery_rmse"],
            "gate": {"binding_integrity": report["integrity"],
                     "binding_outcome": report["outcome"]},
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "closed_loop": False}}


def fit(out_arg: str | None, stage_token: str | None, *, progress: bool = False) -> dict[str, Any]:
    """The one authorised execution: validate, train, audit, commit."""
    # The root override exists for synthetic, isolated tests only, and always stamps
    # authoritative=false. The AUTHORISED fit refuses to run while it is set - a blocker, not a
    # warning: an authoritative artefact must land in the authorised leaf or nowhere.
    if OUTPUT_ROOT_OVERRIDE is not None:
        raise J8Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. It is permitted "
                      f"only for synthetic, isolated tests and always marks the result "
                      f"non-authoritative; the authorised fit refuses to run with it set.")
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    fitted = run_fit(progress=progress)
    report = audit(fitted)
    return commit(fitted, report, leaf)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J8 recovery fit")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--fit", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--progress", action="store_true")
    a = p.parse_args(argv)
    if a.fit:
        r = fit(a.out, a.authorized_stage, progress=a.progress)
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "PASS" else 1
    if a.out is not None:
        raise J8Error("--out is meaningless without --fit; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
