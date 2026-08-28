#!/usr/bin/env python
"""V26C J15 - fresh 35D supervised refit from the J2 parent, post-mismatch corrected.

WHAT THIS STAGE IS
    A single supervised fit of the 35D actor's mean network, starting FRESH from the J2 parent,
    on the J11 aggregate with J14's 854 corrective rows appended:

        16713 (J7) + 8x500 (cell B) + 8x500 (cell C) + 854 (J14) = 25567

WHAT THIS STAGE IS NOT
    It is not a qualification. It builds no environment, runs no rollout, touches no critic, takes
    no optimizer step over anything but the six mean-network tensors, and makes no claim whatever
    about closed-loop behaviour. Passing this gate says the fit is INTERNALLY sound, nothing more.

HYPERPARAMETERS ARE NOT TRANSCRIBED
    JULY_HP below IS the object J11 used - the same dict inside the pinned, frozen J8 module. A
    test asserts identity with J11's. "Identical to J11" is therefore mechanical, not a claim.

RUN_FIT IS NOT REWRITTEN
    run_fit is J11's function, spliced verbatim, with the exception of the error class name. A
    test recomputes the diff against J11's source and refuses anything outside a declared
    whitelist, so drift cannot creep in unnoticed.
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
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# The FROZEN J11 runner, imported as a LIBRARY of audited, stage-neutral helpers. Importing it
# performs no I/O and pulls no torch: its only `import torch` sits inside its own run_fit. It is
# pinned by hash below and its own stage functions are NEVER executed here.
import v26c_j11_multistart_fit as J11  # noqa: E402

J8 = J11.J8


class J15Error(RuntimeError):
    """Every refusal in this stage. Never caught to continue; only to record and re-raise."""


STAGE = "V26C_J15_FRESH_35D_POST_MISMATCH_REFIT"

PREREG = HERE / "v26c_j15_prereg_fresh_refit.json"
PIN_PREREG = "49c748b3a20925a0c270768aa6fddcd3adee474eb1de1cfaa885f849719d1ad2"

# ---------------------------------------------------------------- pinned library modules --------
J11_MODULE = Path(J11.__file__).resolve()
PIN_J11_MODULE = "2b4ac9f496f7a412d40fec0eb5a0d9b69ee22ac634b8e7ed020ca5480cf26242"
J8_MODULE = Path(J8.__file__).resolve()
PIN_J8_MODULE = J11.PIN_J8_MODULE
ASYMMETRIC_RL_MODULE = (REPO / "Trajectory Generator" / "baseline_MLP"
                        / "asymmetric_rl_module.py")
PIN_ASYMMETRIC_RL_MODULE = "5084786c8e6312de2d37744bf327b907ed52ff92cc6e3686b36bd1bde6d21a0f"
WARM_START_SOURCE = J11.WARM_START_SOURCE
PIN_WARM_START = J11.PIN_WARM_START

# ---------------------------------------------------------------- pinned data sources -----------
J7_DATASET = J11.J7_DATASET
J7_RECEIPT = J11.J7_RECEIPT
J10R1_LEAF = J11.J10R1_LEAF
J10R1_RECEIPT = J11.J10R1_RECEIPT
J10R1_COMMIT_VERIFICATION = J11.J10R1_COMMIT_VERIFICATION
CELL_DATASETS = dict(J11.CELL_DATASETS)

J14_LEAF = HERE / "j14_runs" / "j14_dagger_dataset_v26c_2026-08-27_r1"
J14_DATASET = J14_LEAF / "v26c_j14_dagger_increment.npz"
PIN_J14_DATASET = "54b2b8e86a922f554b1fa3bc379737d51000bd0b36596439babd0464e39b0d40"
J14_RECEIPT = J14_LEAF / "v26c_j14_dagger_dataset_receipt.json"
PIN_J14_RECEIPT = "e7262bf9d96e57fda6b3ecee18c58eb4f29ee48f71ff71bc5eba07a3d6abd79b"
J14_COMMIT_VERIFICATION = J14_LEAF / "commit_verification.json"
PIN_J14_COMMIT_VERIFICATION = "24ef5e2cbab291037c05906f592e60f04da64df614c50189303ccdec7ac3bd97"

# J11's own leaf: pinned as NEGATIVE evidence. This stage does not start from it.
J11_LEAF = J11.HERE / "j11_runs" / "j11_multistart_fit_v26c_2026-08-27_r1"
J11_RECEIPT = J11_LEAF / "v26c_j11_multistart_fit_receipt.json"
PIN_J11_RECEIPT = "39228c5cf00a753f1d57f07d4794ac2996401e1b40587cf1ec1e5f5e2b0ae65f"
J11_COMMIT_VERIFICATION = J11_LEAF / "commit_verification.json"
PIN_J11_COMMIT_VERIFICATION = "1d24eed05dd04c187c014181c89a0611a648e5601cacff5e1ae0221adcfb8643"
J11_STATE = J11_LEAF / "rl_module" / "module_state.pkl"
PIN_J11_STATE = "19bf8a43804774c06c24db30626138856bb07acf7002419606d8c1bb887f6b73"

# The J14 governance chain, pinned so the provenance of the 854 rows is closed end to end.
J14_GOVERNANCE = {
    "v26c_j14_prereg_dagger_dataset_rev1.json":
        "877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5",
    "v26c_j14_dagger_dataset.py":
        "fe7c4951c195224b8006655b831549b4b613cfd80a346b7174c58962263e2e0c",
    "v26c_j14_dagger_dataset_authorization_rev3.json":
        "46bbaa262fa0aeea9b3b09608196556f68ac602bdc9b265df4e344d41159dc2f",
    "v26c_j14_architect_go_2026-08-27.json":
        "483b8d558eb8e2245d645384b62d59e9931a1f9270eaed2c715c8a0bea998e3d",
}

# ---------------------------------------------------------------- the parent --------------------
PARENT_MODULE_DIR = J11.PARENT_MODULE_DIR
PIN_PARENT_STATE = J11.PIN_PARENT_STATE
SIDECARS = dict(J11.SIDECARS)
BYTE_IDENTICAL_SIDECARS = tuple(J11.BYTE_IDENTICAL_SIDECARS)
MANIFEST_NAME = J11.MANIFEST_NAME
PIN_OBSERVATION_CONTRACT_ANCESTOR = J11.PIN_OBSERVATION_CONTRACT_ANCESTOR

# ---------------------------------------------------------------- the contract ------------------
ACTOR_WIDTH = J11.ACTOR_WIDTH                       # 35
ACTION_DIM = J11.ACTION_DIM                         # 2
CLOCK_COLUMNS = J11.CLOCK_COLUMNS                   # (0, 1)
DIRECT_KEYS = J11.DIRECT_KEYS
ALIAS_PAIRS = J11.ALIAS_PAIRS
STATE_KEYS = J11.STATE_KEYS
CONTROLLER_SPAN = J11.CONTROLLER_SPAN               # (25, 35)

JULY_HP = dict(J8.JULY_HP)                          # exactly J11's construction, line for line
BEST_EPSILON = J8.BEST_EPSILON

J7_ROWS = J11.J7_ROWS                               # 16713
J7_NOMINAL_UNIQUE = J11.J7_NOMINAL_UNIQUE           # 500
J7_NOMINAL_REPEAT = J11.J7_NOMINAL_REPEAT           # 32
J7_NOMINAL_ROWS = J11.J7_NOMINAL_ROWS               # 16000
J7_RECOVERY_ROWS = J11.J7_RECOVERY_ROWS             # 713
CELL_UNIQUE = J11.CELL_UNIQUE                       # 500
CELL_REPEAT = J11.CELL_REPEAT                       # 8
CELL_ROWS = J11.CELL_ROWS                           # 4000
J11_TOTAL_ROWS = J11.TOTAL_ROWS                     # 24713

NEW_ROWS = 854
NEW_ROW_REPEAT = 1
NEW_ROWS_E = 500
NEW_ROWS_F = 354
NEW_POST_MISMATCH = 671
NEW_PRE_MISMATCH = NEW_ROWS - NEW_POST_MISMATCH     # 183
ALLOWED_SEEDS = (124, 125)
SEALED_SEEDS = (126, 127, 128)
# cell E occupies the first 500 rows of the increment, cell F the remaining 354. MEASURED
# on the committed J14 file, and re-checked in load_j14 rather than assumed.
NEW_CELL_SPANS = (("E", 0, 500), ("F", 500, 854))

TOTAL_ROWS = J11_TOTAL_ROWS + NEW_ROWS * NEW_ROW_REPEAT     # 25567

BLOCKS = (
    {"id": "j7_nominal", "start": 0, "stop": J7_NOMINAL_ROWS,
     "unique": J7_NOMINAL_UNIQUE, "repeat": J7_NOMINAL_REPEAT, "tiled": True},
    {"id": "j7_recovery", "start": J7_NOMINAL_ROWS, "stop": J7_ROWS,
     "unique": J7_RECOVERY_ROWS, "repeat": 1, "tiled": False},
    {"id": "cell_B", "start": J7_ROWS, "stop": J7_ROWS + CELL_ROWS,
     "unique": CELL_UNIQUE, "repeat": CELL_REPEAT, "tiled": True},
    {"id": "cell_C", "start": J7_ROWS + CELL_ROWS, "stop": J11_TOTAL_ROWS,
     "unique": CELL_UNIQUE, "repeat": CELL_REPEAT, "tiled": True},
    {"id": "j14_increment", "start": J11_TOTAL_ROWS, "stop": TOTAL_ROWS,
     "unique": NEW_ROWS, "repeat": NEW_ROW_REPEAT, "tiled": False},
)

REPEAT_SEMANTICS = J11.REPEAT_SEMANTICS

# max(1, int(round(25567 * 0.2))) - VERIFIED at run time against the frozen J8 splitter
EXPECTED_N_VAL = 5113
EXPECTED_N_TRAIN = TOTAL_ROWS - EXPECTED_N_VAL      # 20454

# ---------------------------------------------------------------- destination -------------------
RELATIVE_LEAF_PARTS = ("j15_runs", "j15_fresh_refit_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
MODULE_DIRNAME = "rl_module"
STATE_NAME = "module_state.pkl"
RECEIPT_NAME = "v26c_j15_fresh_refit_receipt.json"
HISTORY_NAME = "history.json"
AGGREGATE_NAME = "v26c_j15_aggregate_dataset.npz"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
TECHNICAL_INVALID_NAME = "TECHNICAL_INVALID"

# Tests redirect writes here. A REAL fit refuses to run into a redirected root, and an INJECTED
# fit refuses to write to the authoritative one. The two rules together make it impossible for a
# test double to masquerade as evidence.
OUTPUT_ROOT_OVERRIDE: Path | None = None

PREFLIGHT_SENTINEL = HERE / "_j15_preflight_sentinel_never_created"

FORBIDDEN_HERE = (
    "a rollout", "an environment", "a critic", "critic warm-up", "PPO", "promotion",
    "deployability", "a closed-loop claim", "an optimizer over anything but the six direct "
    "tensors", "reading seeds 126, 127 or 128", "a second parent", "widening",
    "contralateral features", "a standalone 25D actor", "dedup", "class balancing",
    "filtering by post_mismatch", "a repeat factor on the new rows", "an autonomous retry",
    "an invented threshold", "changing production, FSM v3, the morphology corridor, the reward, "
    "sigma, the SEA or the C++ plugin",
)

_sha_file = J11._sha_file
_sha_array = J11._sha_array
_sha_obj = J11._sha_obj
_rmse = J11._rmse
_rel = J11._rel
_leaf_rel = J11._leaf_rel
_resolve_inside = J11._resolve_inside
actor_state_digest = J11.actor_state_digest


# ================================================================ pinned inputs ==================

def pinned_sources() -> dict[str, str]:
    """Every byte this stage depends on, path -> expected SHA-256. Nothing is read unpinned."""
    out: dict[str, str] = {
        _rel(J11_MODULE): PIN_J11_MODULE,
        _rel(J8_MODULE): PIN_J8_MODULE,
        _rel(WARM_START_SOURCE): PIN_WARM_START,
        _rel(ASYMMETRIC_RL_MODULE): PIN_ASYMMETRIC_RL_MODULE,
        _rel(J7_DATASET): J11.PIN_J7_DATASET,
        _rel(J7_RECEIPT): J11.PIN_J7_RECEIPT,
        _rel(J10R1_RECEIPT): J11.PIN_J10R1_RECEIPT,
        _rel(J14_DATASET): PIN_J14_DATASET,
        _rel(J14_RECEIPT): PIN_J14_RECEIPT,
        _rel(J14_COMMIT_VERIFICATION): PIN_J14_COMMIT_VERIFICATION,
        _rel(J11_RECEIPT): PIN_J11_RECEIPT,
        _rel(J11_COMMIT_VERIFICATION): PIN_J11_COMMIT_VERIFICATION,
        _rel(J11_STATE): PIN_J11_STATE,
        _rel(PARENT_MODULE_DIR / STATE_NAME): PIN_PARENT_STATE,
    }
    for cid, (path, pin) in CELL_DATASETS.items():
        out[_rel(path)] = pin
    for name, pin in SIDECARS.items():
        out[_rel(PARENT_MODULE_DIR / name)] = pin
    for name, pin in J14_GOVERNANCE.items():
        out[_rel(HERE / name)] = pin
    return out


def verify_sources() -> dict[str, Any]:
    """Re-hash every pinned source. Fail closed on the first mismatch, naming the file."""
    pins = pinned_sources()
    checked: dict[str, str] = {}
    for rel_path, pin in sorted(pins.items()):
        path = REPO / rel_path
        if not path.is_file():
            raise J15Error(f"a pinned source is missing: {rel_path}")
        digest = _sha_file(path)
        if digest != pin:
            raise J15Error(f"a pinned source changed: {rel_path}: {digest} != {pin}")
        checked[rel_path] = digest
    return {"count": len(checked), "pinned_sources_sha256": checked,
            "j10r1_commit_verification_sha256": _sha_file(J10R1_COMMIT_VERIFICATION),
            "rule": "every byte this stage reads is pinned; the first mismatch stops the stage"}


def verify_source_leaf(leaf: Path, *, label: str, expect_verdict: str | None) -> dict[str, Any]:
    """A source leaf is usable only if it committed cleanly and was never marked invalid."""
    cv_path = leaf / COMMIT_VERIFICATION_NAME
    if not cv_path.is_file():
        raise J15Error(f"{label}: no {COMMIT_VERIFICATION_NAME} in {leaf}")
    cv = json.loads(cv_path.read_text())
    if cv.get("pass") is not True:
        raise J15Error(f"{label}: its commit verification does not pass")
    if (leaf / TECHNICAL_INVALID_NAME).exists():
        raise J15Error(f"{label}: it carries a {TECHNICAL_INVALID_NAME} marker")
    stale = [q.name for q in leaf.parent.iterdir()
             if q.name.startswith((".lock_", ".staging_"))] if leaf.parent.is_dir() else []
    if stale:
        raise J15Error(f"{label}: a stale lock or staging directory sits beside it: {stale}")
    receipts = sorted(q for q in leaf.glob("*_receipt.json"))
    verdict = None
    if receipts:
        verdict = json.loads(receipts[0].read_text()).get("verdict")
    if expect_verdict is not None and verdict != expect_verdict:
        raise J15Error(f"{label}: verdict {verdict!r}, expected {expect_verdict!r}")
    return {"path": _rel(leaf), "commit_verification_pass": True,
            "technical_invalid_marker": False, "no_stale_lock_or_staging": True,
            "verdict": verdict}


def verify_prereg() -> dict[str, Any]:
    digest = _sha_file(PREREG)
    if digest != PIN_PREREG:
        raise J15Error(f"the preregistration changed: {digest} != {PIN_PREREG}")
    prereg = json.loads(PREREG.read_text())
    if prereg.get("stage_proposed") != STAGE:
        raise J15Error(f"the prereg proposes {prereg.get('stage_proposed')!r}, not {STAGE!r}")
    if prereg.get("execution_permitted_now") is not False:
        raise J15Error("the preregistration must not permit its own execution")
    if "stage_authorised" in prereg:
        raise J15Error("a preregistration may not grant an authorisation")
    declared = prereg["hyperparameters"]["values"]
    if declared != dict(JULY_HP):
        raise J15Error(f"the prereg's hyperparameters differ from the frozen JULY_HP: "
                       f"{declared} != {dict(JULY_HP)}")
    if int(prereg["dataset"]["total_rows"]) != TOTAL_ROWS:
        raise J15Error("the prereg's total_rows differs from this runner's")
    if (int(prereg["split"]["n_val"]), int(prereg["split"]["n_train"])) != (EXPECTED_N_VAL,
                                                                           EXPECTED_N_TRAIN):
        raise J15Error("the prereg's split differs from this runner's")
    # the preregistered gate must describe THIS gate, in unambiguous fields
    g = prereg["gate"]
    if tuple(g["j11_binding_subset"]) != J11_BINDING_SUBSET:
        raise J15Error("the prereg's j11_binding_subset is not J11's exact 16-name set")
    if tuple(g["j15_additional_binding"]) != J15_ADDITIONAL_BINDING:
        raise J15Error("the prereg's j15_additional_binding is not this stage's single extra gate")
    if int(g["binding_count"]) != len(J15_BINDING_NAMES):
        raise J15Error(f"the prereg commits to {g['binding_count']} binding checks, this runner "
                       f"has {len(J15_BINDING_NAMES)}")
    if g.get("identical_to_j11") is not False:
        raise J15Error("the prereg must state identical_to_j11 = false: J15 has 17 gates, J11 16")
    if "binding_identical_to_j11" in g:
        raise J15Error("the ambiguous field binding_identical_to_j11 must not exist: its name "
                       "said 'identical' while its content held 17 names")
    return {"path": _rel(PREREG), "sha256": digest,
            "stage_proposed": prereg["stage_proposed"],
            "authorisation_status": prereg["authorisation_status"],
            "hyperparameters_match_frozen_july_hp": True,
            "gate_j11_binding_subset_count": len(g["j11_binding_subset"]),
            "gate_j15_additional_binding": list(g["j15_additional_binding"]),
            "gate_binding_count": int(g["binding_count"]),
            "gate_identical_to_j11": False,
            "deferred_todo": list(prereg["todo_propagated"])}


def verify_modules() -> dict[str, str]:
    """The library modules, re-hashed. J11 verifies J8 and warm_start; this adds J11 itself."""
    modules = dict(J11.verify_modules())
    for path, pin in ((J11_MODULE, PIN_J11_MODULE),):
        digest = _sha_file(path)
        if digest != pin:
            raise J15Error(f"{path.name} changed: {digest} != {pin}")
        modules[path.name] = digest
    modules["warm_start.py"] = J11.verify_warm_start_source()
    return modules


def verify_hyperparameters_are_j11s() -> dict[str, Any]:
    """MECHANICAL, not a claim - and stated at the precision it actually holds.

    J11 does `JULY_HP = dict(J8.JULY_HP)`: a COPY, not the same object. So does this stage, line
    for line. Object identity is therefore the WRONG test and would fail for a good reason. What
    is guaranteed, and checked here, is stronger than a transcription: both dicts are EQUAL, both
    are built from the same frozen J8 dict, and that module is pinned by hash. No hyperparameter
    is retyped anywhere in this stage.
    """
    frozen = dict(J8.JULY_HP)
    if JULY_HP != frozen:
        raise J15Error(f"JULY_HP differs from the frozen J8 source: {JULY_HP} != {frozen}")
    if J11.JULY_HP != frozen:
        raise J15Error(f"J11's JULY_HP differs from the frozen J8 source: {J11.JULY_HP} != "
                       f"{frozen}. The 'identical to J11' claim would be void.")
    if JULY_HP != J11.JULY_HP:
        raise J15Error(f"this stage and J11 disagree: {JULY_HP} != {J11.JULY_HP}")
    if BEST_EPSILON != J11.BEST_EPSILON:
        raise J15Error("BEST_EPSILON differs from J11's")
    if sorted(JULY_HP) != sorted(frozen):
        raise J15Error("the hyperparameter key set drifted from the frozen source")
    return {"equal_to_j11": True, "equal_to_frozen_j8_source": True,
            "same_python_object_as_j11": False,
            "why_not_the_same_object": "J11 itself takes a copy - dict(J8.JULY_HP) - and this "
                                       "stage reproduces that construction line for line",
            "values": dict(JULY_HP), "best_epsilon": BEST_EPSILON,
            "keys": sorted(JULY_HP),
            "j8_module_sha256": _sha_file(J8_MODULE),
            "j11_module_sha256": _sha_file(J11_MODULE),
            "transcribed": False,
            "how_this_is_known": "run-time equality against the frozen, hash-pinned J8 dict and "
                                 "against J11's own copy - no number is retyped in this stage"}


# ================================================================ the J14 increment ==============

def load_j14() -> dict[str, Any]:
    """The 854 corrective rows. Pinned, shape-checked, clock-checked and seed-checked."""
    receipt_digest = _sha_file(J14_RECEIPT)
    if receipt_digest != PIN_J14_RECEIPT:
        raise J15Error(f"the J14 receipt changed: {receipt_digest} != {PIN_J14_RECEIPT}")
    digest = _sha_file(J14_DATASET)
    if digest != PIN_J14_DATASET:
        raise J15Error(f"the J14 increment changed: {digest} != {PIN_J14_DATASET}")
    with np.load(J14_DATASET, allow_pickle=False) as archive:
        keys = tuple(sorted(archive.files))
        expect = ("actions", "actor_feature_names", "cell", "observations", "post_mismatch",
                  "seed", "step", "time_before")
        if keys != expect:
            raise J15Error(f"the J14 increment holds {keys}, not {expect}")
        z = {k: np.asarray(archive[k]) for k in archive.files}

    obs, act = z["observations"], z["actions"]
    names = tuple(str(n) for n in z["actor_feature_names"].tolist())
    if obs.shape != (NEW_ROWS, ACTOR_WIDTH) or act.shape != (NEW_ROWS, ACTION_DIM):
        raise J15Error(f"the J14 increment is {obs.shape}/{act.shape}, expected "
                       f"{(NEW_ROWS, ACTOR_WIDTH)}/{(NEW_ROWS, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J15Error(f"the J14 increment is {obs.dtype}/{act.dtype}, expected float32")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J15Error("the J14 increment holds non-finite values")
    if float(np.max(np.abs(obs[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J15Error("the J14 increment's clock columns are not exactly zero")

    seeds = sorted({int(v) for v in z["seed"]})
    if tuple(seeds) != ALLOWED_SEEDS:
        raise J15Error(f"the J14 increment carries seeds {seeds}, expected {list(ALLOWED_SEEDS)}")
    sealed = sorted(s for s in seeds if s in SEALED_SEEDS)
    if sealed:
        raise J15Error(f"the J14 increment carries SEALED seeds {sealed}; this stage fails closed")

    cells = np.array([str(c) for c in z["cell"]])
    per_cell = {c: int((cells == c).sum()) for c in sorted(set(cells.tolist()))}
    if per_cell != {"E": NEW_ROWS_E, "F": NEW_ROWS_F}:
        raise J15Error(f"the J14 increment splits {per_cell}, expected "
                       f"{{'E': {NEW_ROWS_E}, 'F': {NEW_ROWS_F}}}")
    for cid, a, b in NEW_CELL_SPANS:
        if not np.all(cells[a:b] == cid):
            raise J15Error(f"the J14 increment's cell {cid} is not the contiguous block "
                           f"[{a}:{b}] the binding gate's slices assume")
    post = np.asarray(z["post_mismatch"]).astype(bool)
    if int(post.sum()) != NEW_POST_MISMATCH:
        raise J15Error(f"the J14 increment has {int(post.sum())} post-mismatch rows, expected "
                       f"{NEW_POST_MISMATCH}")

    return {"observations": obs, "actions": act, "names": names, "sha256": digest,
            "receipt_sha256": receipt_digest, "clock_already_zero": True,
            "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act)},
            "rows": NEW_ROWS, "per_cell": per_cell,
            "seeds": seeds, "sealed_seeds_present": [],
            "post_mismatch_rows": int(post.sum()),
            "pre_mismatch_rows": int((~post).sum()),
            "post_mismatch_mask_sha256": _sha_array(post),
            "provenance_columns": ["cell", "seed", "step", "post_mismatch", "time_before"],
            "unique_rows": int(len(np.unique(obs, axis=0))),
            "not_truncated": True, "not_filtered_by_post_mismatch": True, "repeat": NEW_ROW_REPEAT}


# ================================================================ the aggregate ==================

def build_aggregate() -> dict[str, Any]:
    """Materialise the 25567-row aggregate. TILE semantics, clock projected to EXACT zero.

    The first four blocks reproduce the J11 aggregate exactly and in the same order, so J11's row
    indices survive unchanged and the two fits are directly comparable. J14's rows are APPENDED.
    """
    j7 = J11.load_j7()
    prov = J11.verify_j10r1_provenance()
    cells = {cid: J11.load_cell(cid) for cid in ("B", "C")}
    new = load_j14()

    names = j7["names"]
    for cid, cell in cells.items():
        if cell["names"] != names:
            raise J15Error(f"cell {cid} feature names differ from J7's")
    if new["names"] != names:
        raise J15Error("the J14 increment's feature names differ from J7's")
    manifest_names = J8.actor_feature_names()
    if tuple(manifest_names) != names:
        raise J15Error("the dataset schema differs from the parent's actor manifest")

    obs_parts = [j7["observations"]]
    act_parts = [j7["actions"]]
    for cid in ("B", "C"):
        obs_parts.append(np.tile(cells[cid]["observations"], (CELL_REPEAT, 1)))
        act_parts.append(np.tile(cells[cid]["actions"], (CELL_REPEAT, 1)))
    obs_parts.append(np.tile(new["observations"], (NEW_ROW_REPEAT, 1)))
    act_parts.append(np.tile(new["actions"], (NEW_ROW_REPEAT, 1)))
    obs = np.ascontiguousarray(np.concatenate(obs_parts, axis=0).astype(np.float32))
    act = np.ascontiguousarray(np.concatenate(act_parts, axis=0).astype(np.float32))

    if obs.shape != (TOTAL_ROWS, ACTOR_WIDTH) or act.shape != (TOTAL_ROWS, ACTION_DIM):
        raise J15Error(f"the aggregate is {obs.shape}/{act.shape}, expected "
                       f"{(TOTAL_ROWS, ACTOR_WIDTH)}/{(TOTAL_ROWS, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J15Error(f"the aggregate is {obs.dtype}/{act.dtype}, expected float32")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J15Error("the aggregate holds non-finite values")
    if float(np.max(np.abs(obs[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J15Error("the aggregate's clock columns are not exactly zero")

    block_report = []
    for b in BLOCKS:
        seg_o = obs[b["start"]:b["stop"]]
        seg_a = act[b["start"]:b["stop"]]
        if seg_o.shape[0] != b["stop"] - b["start"]:
            raise J15Error(f"block {b['id']} has the wrong length")
        if b["tiled"]:
            head_o = seg_o[:b["unique"]]
            head_a = seg_a[:b["unique"]]
            for r in range(b["repeat"]):
                sl = slice(r * b["unique"], (r + 1) * b["unique"])
                if not (np.array_equal(seg_o[sl], head_o) and np.array_equal(seg_a[sl], head_a)):
                    raise J15Error(f"block {b['id']} tile {r} is not bit-identical to the first")
        block_report.append({
            "id": b["id"], "start": b["start"], "stop": b["stop"],
            "rows": b["stop"] - b["start"], "unique": b["unique"], "repeat": b["repeat"],
            "tiled": b["tiled"],
            "unique_block_slice": [b["start"], b["start"] + b["unique"]] if b["tiled"]
                                  else [b["start"], b["stop"]],
            "distinct_rows_measured": int(len(np.unique(seg_o, axis=0))),
            "observations_sha256": _sha_array(seg_o), "actions_sha256": _sha_array(seg_a)})

    # the sources must survive intact inside the aggregate
    if not np.array_equal(obs[:J7_ROWS], j7["observations"]):
        raise J15Error("the J7 rows are not bit-identical inside the aggregate")
    for cid, b in (("B", BLOCKS[2]), ("C", BLOCKS[3])):
        head = obs[b["start"]:b["start"] + CELL_UNIQUE]
        if not np.array_equal(head, cells[cid]["observations"]):
            raise J15Error(f"cell {cid}'s projected rows are not bit-identical in the aggregate")
        if not np.array_equal(act[b["start"]:b["start"] + CELL_UNIQUE], cells[cid]["actions"]):
            raise J15Error(f"cell {cid}'s labels are not bit-identical in the aggregate")
    if not np.array_equal(obs[J11_TOTAL_ROWS:], new["observations"]):
        raise J15Error("the J14 rows are not bit-identical inside the aggregate")
    if not np.array_equal(act[J11_TOTAL_ROWS:], new["actions"]):
        raise J15Error("the J14 labels are not bit-identical inside the aggregate")

    # the J11 prefix must be BIT-IDENTICAL to J11's own aggregate content hash
    prefix_hashes = {"observations": _sha_array(obs[:J11_TOTAL_ROWS]),
                     "actions": _sha_array(act[:J11_TOTAL_ROWS])}

    return {
        "observations": obs, "actions": act, "names": names,
        "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act)},
        "rows": TOTAL_ROWS, "blocks": block_report,
        "repeat_semantics": REPEAT_SEMANTICS,
        "j11_prefix": {"rows": J11_TOTAL_ROWS, "content_hashes": prefix_hashes,
                       "meaning": "rows [0:24713] are the J11 aggregate, in J11's order. The "
                                  "854 new rows are APPENDED, so J11's indices are unchanged."},
        "sources": {
            "j7": {"path": _rel(J7_DATASET), "sha256": j7["sha256"],
                   "receipt_sha256": j7["receipt_sha256"], "rows": J7_ROWS,
                   "content_hashes": j7["content_hashes"], "clock_already_zero": True},
            "cell_B": {"path": _rel(CELL_DATASETS["B"][0]), "sha256": cells["B"]["sha256"],
                       "rows": CELL_UNIQUE, "repeat": CELL_REPEAT,
                       "raw_clock": cells["B"]["raw_clock"],
                       "content_hashes": cells["B"]["content_hashes"],
                       "unique_rows": cells["B"]["unique_rows"]},
            "cell_C": {"path": _rel(CELL_DATASETS["C"][0]), "sha256": cells["C"]["sha256"],
                       "rows": CELL_UNIQUE, "repeat": CELL_REPEAT,
                       "raw_clock": cells["C"]["raw_clock"],
                       "content_hashes": cells["C"]["content_hashes"],
                       "unique_rows": cells["C"]["unique_rows"]},
            "j14": {"path": _rel(J14_DATASET), "sha256": new["sha256"],
                    "receipt_sha256": new["receipt_sha256"], "rows": NEW_ROWS,
                    "repeat": NEW_ROW_REPEAT, "per_cell": new["per_cell"],
                    "seeds": new["seeds"], "post_mismatch_rows": new["post_mismatch_rows"],
                    "pre_mismatch_rows": new["pre_mismatch_rows"],
                    "content_hashes": new["content_hashes"],
                    "unique_rows": new["unique_rows"],
                    "clock_already_zero": True, "not_truncated": True}},
        "j10r1_provenance": prov,
        "arithmetic": {
            "expression": "16713 + 8x500 + 8x500 + 1x854 = 25567",
            "j7_rows": J7_ROWS, "cell_B": CELL_UNIQUE, "cell_C": CELL_UNIQUE,
            "cell_repeat": CELL_REPEAT, "new_rows": NEW_ROWS, "new_repeat": NEW_ROW_REPEAT,
            "total": TOTAL_ROWS, "verified_by_measurement": True},
        "clock_projection": {
            "columns": list(CLOCK_COLUMNS),
            "feature_names": [names[c] for c in CLOCK_COLUMNS],
            "j7_state": "already exactly zero before this stage",
            "j14_state": "already exactly zero; J14 projected it at materialisation",
            "cells_raw_state": "(sin, cos) = (0, 1), which is what the environment records with "
                               "the gait clock disabled. The cells on disk are NOT zeroed; the "
                               "projection happens here, exactly as in J11.",
            "action": "projected to EXACT zero before aggregation and before the fit",
            "expected_not_a_defect": "a raw cos of 1 is the documented disabled-clock encoding "
                                     "and is never a reason to reject a cell",
            "raw_and_projected_both_hashed": True},
        "binding_subsets": {
            "aggregate": [0, TOTAL_ROWS],
            "j7_recovery_original": [J7_NOMINAL_ROWS, J7_ROWS],
            "cell_B_unique": [BLOCKS[2]["start"], BLOCKS[2]["start"] + CELL_UNIQUE],
            "cell_C_unique": [BLOCKS[3]["start"], BLOCKS[3]["start"] + CELL_UNIQUE],
            # BINDING by the architect's correction of 2026-08-27: the approved J13 report
            # preregistered "RMSE dopo < prima su ... e sul nuovo blocco E". J14 now carries E AND
            # F, so the same preregistered rule covers the whole 854-row block. Same strict
            # inequality, no new numeric threshold.
            "j14_increment": [J11_TOTAL_ROWS, TOTAL_ROWS]},
        "diagnostic_subsets": {
            "j14_cell_E": [J11_TOTAL_ROWS + NEW_CELL_SPANS[0][1],
                           J11_TOTAL_ROWS + NEW_CELL_SPANS[0][2]],
            "j14_cell_F": [J11_TOTAL_ROWS + NEW_CELL_SPANS[1][1],
                           J11_TOTAL_ROWS + NEW_CELL_SPANS[1][2]],
            "j11_prefix": [0, J11_TOTAL_ROWS],
            "j7_nominal": [0, J7_NOMINAL_ROWS]},
        "post_mismatch_mask": np.asarray(
            np.load(J14_DATASET, allow_pickle=False)["post_mismatch"]).astype(bool),
    }


def build_split(rows: int) -> dict[str, Any]:
    """July's split, via the FROZEN J8 transcription. Verified against the declared counts."""
    split = J8.build_split(rows)
    if rows == TOTAL_ROWS and (split["n_val"], split["n_train"]) != (EXPECTED_N_VAL,
                                                                    EXPECTED_N_TRAIN):
        raise J15Error(f"the split produced {split['n_val']}/{split['n_train']}, not the "
                       f"declared {EXPECTED_N_VAL}/{EXPECTED_N_TRAIN}. The declared numbers are "
                       f"VERIFIED here, never assumed.")
    return split


def load_parent_state() -> dict[str, np.ndarray]:
    """The J2 parent, torch-free. This stage NEVER loads the J11 actor as a parent."""
    return J11.load_parent_state()


# ================================================================ the fit ========================
RUN_FIT_EXPECTED_DIFF = '--- J11.run_fit\n+++ J15.run_fit\n@@ -1,3 +1,9 @@\n def run_fit(*, progress: bool = False) -> dict[str, Any]:\n-    """Train the full mean network IN MEMORY and audit the result. Writes nothing."""\n+    """Train the full mean network IN MEMORY and audit the result. Writes nothing.\n+\n+    SPLICED VERBATIM from the frozen J11 runner. The only differences from J11.run_fit are\n+    the error class name and two PURE-READ diagnostics that record what J11 left unrecorded\n+    (the torch version and Adam\'s resolved argument set). Nothing in the mathematics moves.\n+    A test recomputes this diff and refuses anything outside RUN_FIT_EXPECTED_DIFF.\n+    """\n     modules = verify_modules()\n@@ -42,3 +48,4 @@\n                           "to it contains neither call; forcing them would be an invention",\n-        "observed_at": "fit time; the preflight is torch-free and cannot observe them"}\n+        "observed_at": "fit time; the preflight is torch-free and cannot observe them",\n+        "torch_version": str(torch.__version__)}\n \n@@ -72,3 +79,3 @@\n     if torch_diff != 0.0 or not np.array_equal(before, torch_scaled):\n-        raise J11Error(f"the torch kernel does not reproduce the raw/scaled equivalence: "\n+        raise J15Error(f"the torch kernel does not reproduce the raw/scaled equivalence: "\n                        f"max_abs_diff {torch_diff}. No tolerance is assumed.")\n@@ -84,2 +91,7 @@\n                                  lr=float(JULY_HP["learning_rate"]))\n+    # J11 passed only lr and recorded the rest NOWHERE, so its fit is not bit-reproducible from\n+    # its artefacts. Nothing is changed here - the resolved defaults are simply written down.\n+    torch_backend["adam_resolved_arguments"] = [\n+        {k: (list(v) if isinstance(v, tuple) else v) for k, v in g.items() if k != "params"}\n+        for g in optimiser.param_groups]\n     best = {"epoch": 0, "val": float("inf"),\n'
"""The ONLY differences between J11.run_fit and J15.run_fit. A test recomputes the
unified diff at run time and refuses anything that is not exactly this."""


def run_fit(*, progress: bool = False) -> dict[str, Any]:
    """Train the full mean network IN MEMORY and audit the result. Writes nothing.

    SPLICED VERBATIM from the frozen J11 runner. The only differences from J11.run_fit are
    the error class name and two PURE-READ diagnostics that record what J11 left unrecorded
    (the torch version and Adam's resolved argument set). Nothing in the mathematics moves.
    A test recomputes this diff and refuses anything outside RUN_FIT_EXPECTED_DIFF.
    """
    modules = verify_modules()
    modules["warm_start.py"] = verify_warm_start_source()
    prereg = verify_prereg()
    scales_checked = J8.july_scales_from_source()
    data = build_aggregate()
    parent = load_parent_state()
    names = data["names"]
    obs, act = data["observations"], data["actions"]
    clock = list(CLOCK_COLUMNS)
    controller = list(J8.controller_indices(names))
    scales = J8.scale_vector(names)

    # BINDING preconditions. Pure numpy, so they run before torch is anywhere near the process.
    preconditions = J8.parent_preconditions(parent, names, obs)

    # ---- July's executive order, as transcribed and validated in J8 ---------------------------
    # (1) import torch, in the --fit path only
    # (2) torch.manual_seed(seed)
    # (3) np.random.seed(seed)          <- July's legacy GLOBAL seed, once, not a Generator
    # (4) deterministic initialisation from the parent; nothing here draws from any RNG
    # (5) the single np.default_rng(seed) inside build_split: it draws the split FIRST and then
    #     produces every epoch shuffle
    # (6) the optimizer
    #
    # NOT here, deliberately: torch.use_deterministic_algorithms and torch.set_num_threads.
    # Neither exists in the repository snapshot closest to the July run, and the run's own
    # report records no determinism setting. Forcing them would be an invention, not a
    # replication. Their OBSERVED values are recorded as diagnostics.
    import torch                                        # (1)
    from torch.nn import functional
    torch.manual_seed(int(JULY_HP["seed"]))             # (2)
    np.random.seed(int(JULY_HP["seed"]))                # (3)

    torch_backend = {
        "are_deterministic_algorithms_enabled":
            bool(torch.are_deterministic_algorithms_enabled()),
        "get_num_threads": int(torch.get_num_threads()),
        "forced_by_j11": False, "binding": False,
        "why_not_forced": "the July run recorded neither setting and the code snapshot closest "
                          "to it contains neither call; forcing them would be an invention",
        "observed_at": "fit time; the preflight is torch-free and cannot observe them",
        "torch_version": str(torch.__version__)}

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
        torch_scaled = forward(X)[:, :ACTION_DIM].numpy()
    torch_diff = float(np.max(np.abs(before.astype(np.float64)
                                     - torch_scaled.astype(np.float64))))
    if torch_diff != 0.0 or not np.array_equal(before, torch_scaled):
        raise J15Error(f"the torch kernel does not reproduce the raw/scaled equivalence: "
                       f"max_abs_diff {torch_diff}. No tolerance is assumed.")
    preconditions["raw_vs_scaled_equivalence"]["torch_max_abs_diff"] = torch_diff
    preconditions["raw_vs_scaled_equivalence"]["torch_bit_identical"] = True
    preconditions["raw_vs_scaled_equivalence"]["kernels_checked"] = ["numpy float32", "torch"]

    # (5) the single Generator, AFTER the deterministic initialisation and BEFORE the optimizer
    split = build_split(len(obs))
    rng = split["rng"]

    optimiser = torch.optim.Adam(list(params.values()),        # (6)
                                 lr=float(JULY_HP["learning_rate"]))
    # J11 passed only lr and recorded the rest NOWHERE, so its fit is not bit-reproducible from
    # its artefacts. Nothing is changed here - the resolved defaults are simply written down.
    torch_backend["adam_resolved_arguments"] = [
        {k: (list(v) if isinstance(v, tuple) else v) for k, v in g.items() if k != "params"}
        for g in optimiser.param_groups]
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
        for name, value in J8.MARKOV_CONTROLLER_FEATURE_SCALES.items():
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
            "torch_backend_observed": torch_backend, "modules": modules,
            "seed_order": ["import torch", "torch.manual_seed", "np.random.seed",
                           "deterministic init from parent (no RNG)",
                           "np.default_rng in build_split", "optimizer"],
            "numpy_legacy_seed": {"call": "np.random.seed", "value": int(JULY_HP["seed"]),
                                  "count": 1, "is_a_generator": False,
                                  "position": "after torch.manual_seed, before the Generator"}}


# ================================================================ audit and gate =================

def audit(fit: dict[str, Any]) -> dict[str, Any]:
    """The 16-name J11 binding subset is PRESERVED; J15 adds ONE binding rule; the rest are
    diagnostics that bind nothing.

    This gate is NOT identical to J11's and does not say it is. j14_increment_rmse_decreases is
    added on the authority of the approved J13 report.
    """
    final, parent = fit["final"], fit["parent"]
    names, data = fit["names"], fit["data"]
    obs, act = data["observations"], data["actions"]
    before, after = fit["before"], fit["after"]
    clock, controller = fit["clock"], fit["controller"]

    integrity: dict[str, Any] = {}
    integrity["keys_and_shapes_match_parent"] = (
        tuple(sorted(final)) == tuple(sorted(parent))
        and all(np.asarray(final[k]).shape == np.asarray(parent[k]).shape for k in parent)
        and all(np.asarray(final[k]).dtype == np.float32 for k in final))
    integrity["all_parameters_finite"] = bool(
        all(np.all(np.isfinite(np.asarray(v))) for v in final.values()))
    integrity["clock_bit_zero"] = bool(all(
        np.all(np.asarray(final[a])[:, clock] == 0.0)
        for a in ("pi.0.0.weight", "pi_encoder.0.weight")))
    integrity["aliases_bit_identical"] = bool(all(
        np.array_equal(np.asarray(final[a]), np.asarray(final[d])) for a, d in ALIAS_PAIRS))
    integrity["logstd_bit_identical_to_parent"] = bool(
        np.array_equal(np.asarray(final["pi.1.weight"])[ACTION_DIM:],
                       np.asarray(parent["pi.1.weight"])[ACTION_DIM:])
        and np.array_equal(np.asarray(final["pi.1.bias"])[ACTION_DIM:],
                           np.asarray(parent["pi.1.bias"])[ACTION_DIM:]))
    integrity["no_critic_key"] = not any(
        "critic" in k or k.startswith("vf") for k in final)

    # every pinned input, re-hashed AFTER the fit
    try:
        verify_sources()
        unchanged = True
    except J15Error:
        unchanged = False
    integrity["inputs_unchanged"] = bool(unchanged)

    integrity["aggregate_reproduces_content_hashes"] = bool(
        _sha_array(obs) == data["content_hashes"]["observations"]
        and _sha_array(act) == data["content_hashes"]["actions"])

    integrity["split_counts_as_declared"] = bool(
        fit["split"]["n_val"] == EXPECTED_N_VAL and fit["split"]["n_train"] == EXPECTED_N_TRAIN)

    replay_best, replay_epoch = float("inf"), 0
    for row in fit["history"]:
        if float(row["validation_mse"]) < replay_best - BEST_EPSILON:
            replay_best, replay_epoch = float(row["validation_mse"]), int(row["epoch"])
    integrity["best_state_reconstructible_from_history"] = bool(
        replay_epoch == fit["best_epoch"]
        and abs(replay_best - fit["best_validation_mse"]) <= 0.0)

    # ---- RMSE on the FIVE binding subsets: J11's four, plus the whole J14 block --------------
    sub = data["binding_subsets"]
    rmse: dict[str, Any] = {}
    for key in ("aggregate", "j7_recovery_original", "cell_B_unique", "cell_C_unique",
                "j14_increment"):
        a, b = sub[key]
        r_before = _rmse(before[a:b], act[a:b])
        r_after = _rmse(after[a:b], act[a:b])
        rmse[key] = {"rows": b - a, "slice": [a, b],
                     "before": r_before, "after": r_after,
                     "decreased": bool(r_after < r_before),
                     "binding": True,
                     "rule": "strictly lower after than before; no margin is invented"}

    controller_norms = {names[c]: float(np.linalg.norm(np.asarray(final["pi.0.0.weight"])[:, c]))
                        for c in controller}
    controller_all_nonzero = bool(all(v > 0.0 for v in controller_norms.values()))

    metrics_finite = bool(
        all(np.isfinite([v["before"], v["after"]]).all() for v in rmse.values())
        and np.isfinite(fit["best_validation_mse"])
        and all(np.isfinite([r["train_loss"], r["validation_mse"]]).all()
                for r in fit["history"]))
    integrity["all_metrics_finite"] = metrics_finite

    # ---- diagnostics, explicitly NOT binding -------------------------------------------------
    dsub = data["diagnostic_subsets"]
    n0, n1 = dsub["j7_nominal"]
    g0, g1 = data["binding_subsets"]["j14_increment"]
    mask = np.asarray(data["post_mismatch_mask"]).astype(bool)
    post_idx = np.arange(g0, g1)[mask]
    pre_idx = np.arange(g0, g1)[~mask]

    diagnostics = {
        "nominal_rmse": {
            "before": _rmse(before[n0:n1], act[n0:n1]),
            "after": _rmse(after[n0:n1], act[n0:n1]),
            "binding": False,
            "why_not": "the nominal block is the self-anchor; it is expected to move and no "
                       "direction is preregistered for it"},
        "j14_cell_E_rmse": {
            "cell": "E", "seed": 124, "rows": dsub["j14_cell_E"][1] - dsub["j14_cell_E"][0],
            "slice": dsub["j14_cell_E"],
            "before": _rmse(before[dsub["j14_cell_E"][0]:dsub["j14_cell_E"][1]],
                            act[dsub["j14_cell_E"][0]:dsub["j14_cell_E"][1]]),
            "after": _rmse(after[dsub["j14_cell_E"][0]:dsub["j14_cell_E"][1]],
                           act[dsub["j14_cell_E"][0]:dsub["j14_cell_E"][1]]),
            "binding": False,
            "why_not": "E and F are kept SEPARATE for visibility, but the binding rule is on the "
                       "854-row block as a whole. Splitting the gate per cell would be a new "
                       "rule; reporting per cell is not."},
        "j14_cell_F_rmse": {
            "cell": "F", "seed": 125, "rows": dsub["j14_cell_F"][1] - dsub["j14_cell_F"][0],
            "slice": dsub["j14_cell_F"],
            "before": _rmse(before[dsub["j14_cell_F"][0]:dsub["j14_cell_F"][1]],
                            act[dsub["j14_cell_F"][0]:dsub["j14_cell_F"][1]]),
            "after": _rmse(after[dsub["j14_cell_F"][0]:dsub["j14_cell_F"][1]],
                           act[dsub["j14_cell_F"][0]:dsub["j14_cell_F"][1]]),
            "binding": False,
            "why_not": "as above - visibility, not a gate"},
        "j14_phase_invalid_fraction": {
            "value": float(int(mask.sum()) / len(mask)),
            "rows_flagged": int(mask.sum()), "rows_total": int(len(mask)),
            "binding": False,
            "what_it_measures": "the fraction of the new labels whose discrete FSM columns had "
                                "already diverged from the same-actor nominal reference, i.e. "
                                "J14's own post_mismatch flag. It is REPORTED, never used to "
                                "filter or weight a row.",
            "why_not": "the approved J13 report lists the phase-invalid fraction of the new "
                       "labels as a DIAGNOSTIC. It stays one."},
        "j14_post_mismatch_rmse": {
            "rows": int(len(post_idx)),
            "before": _rmse(before[post_idx], act[post_idx]),
            "after": _rmse(after[post_idx], act[post_idx]),
            "binding": False,
            "motivation": "isolates the 671 rows J13 identified as causal from the 183 "
                          "pre-mismatch rows a J7-style truncation would have kept"},
        "j14_pre_mismatch_rmse": {
            "rows": int(len(pre_idx)),
            "before": _rmse(before[pre_idx], act[pre_idx]),
            "after": _rmse(after[pre_idx], act[pre_idx]),
            "binding": False,
            "motivation": "the counterpart of the above; a truncating stage would have had only "
                          "these"},
        "j11_prefix_rmse": {
            "rows": J11_TOTAL_ROWS,
            "before": _rmse(before[:J11_TOTAL_ROWS], act[:J11_TOTAL_ROWS]),
            "after": _rmse(after[:J11_TOTAL_ROWS], act[:J11_TOTAL_ROWS]),
            "binding": False,
            "motivation": "what adding 854 rows cost on the exact rows J11 already fit. No "
                          "direction is preregistered, so it cannot be a gate."},
        "clipping_out_of_bounds_rows": {
            "value": int(np.sum(np.any(np.abs(after) > 1.0, axis=1))),
            "binding": False,
            "why_not": "the clip term is a loss weight, not a gate; the environment clips"},
        "best_validation_mse": {
            "value": float(fit["best_validation_mse"]), "binding": False,
            "why_not": "every repeated block puts the same unique row in BOTH partitions. "
                       "Inherited from July, J7 and J11: a training diagnostic, never a "
                       "generalisation estimate."},
        "epochs_run": {"value": int(fit["epochs_run"]), "binding": False},
        "stopped_early": {"value": bool(fit["stopped_early"]), "binding": False},
        "torch_backend_observed": fit["torch_backend_observed"],
    }

    binding = {
        "integrity_keys_and_shapes_match_parent": integrity["keys_and_shapes_match_parent"],
        "integrity_all_parameters_finite": integrity["all_parameters_finite"],
        "integrity_clock_bit_zero": integrity["clock_bit_zero"],
        "integrity_aliases_bit_identical": integrity["aliases_bit_identical"],
        "integrity_logstd_bit_identical_to_parent": integrity["logstd_bit_identical_to_parent"],
        "integrity_no_critic_key": integrity["no_critic_key"],
        "integrity_inputs_unchanged": integrity["inputs_unchanged"],
        "integrity_aggregate_reproduces_content_hashes":
            integrity["aggregate_reproduces_content_hashes"],
        "integrity_split_counts_as_declared": integrity["split_counts_as_declared"],
        "integrity_best_state_reconstructible_from_history":
            integrity["best_state_reconstructible_from_history"],
        "integrity_all_metrics_finite": integrity["all_metrics_finite"],
        "aggregate_rmse_decreases": rmse["aggregate"]["decreased"],
        "recovery_rmse_decreases": rmse["j7_recovery_original"]["decreased"],
        "cell_B_unique_rmse_decreases": rmse["cell_B_unique"]["decreased"],
        "cell_C_unique_rmse_decreases": rmse["cell_C_unique"]["decreased"],
        "j14_increment_rmse_decreases": rmse["j14_increment"]["decreased"],
        "controller_columns_nonzero": controller_all_nonzero,
    }
    if sorted(binding) != sorted(J15_BINDING_NAMES):
        raise J15Error("the binding set drifted from the preregistered one")
    failed = sorted(k for k, v in binding.items() if not v)
    return {"binding": binding, "failed": failed, "pass": not failed,
            "integrity": integrity, "rmse": rmse, "diagnostics": diagnostics,
            "controller_column_norms": controller_norms,
            "controller_columns_nonzero": controller_all_nonzero,
            "identical_to_j11": False,
            "relation_to_j11": {
                "j11_binding_subset": list(J11_BINDING_SUBSET),
                "j11_binding_subset_count": len(J11_BINDING_SUBSET),
                "j11_set_preserved_exactly": set(J11_BINDING_SUBSET) < set(binding),
                "names_j15_dropped_or_renamed_from_j11": sorted(
                    set(J11_BINDING_SUBSET) - set(binding)),
                "j15_additional_binding": list(J15_ADDITIONAL_BINDING),
                "j15_additional_binding_count": len(J15_ADDITIONAL_BINDING),
                "binding_count": len(binding),
                "statement": "the 16-name J11 binding subset is preserved, name for name, and "
                             "J15 adds one binding rule: j14_increment_rmse_decreases. "
                             "16 + 1 = 17. This gate is NOT identical to J11's and does not "
                             "claim to be."},
            "supervised_only": "this gate says nothing about closed-loop behaviour",
            "no_invented_thresholds": (
                "every binding numeric rule is a STRICT inequality against a measured baseline: "
                "'after < before' on FIVE RMSE subsets - aggregate, j7_recovery_original, "
                "cell_B_unique, cell_C_unique and the WHOLE 854-row J14 block - and 'norm > 0' on "
                "ten controller columns. No margin, no tolerance and no absolute target is "
                "asserted anywhere, and the fifth introduces no number that the first four did "
                "not already use. Only the J14 block AS A WHOLE binds: cells E and F separately, "
                "the post-mismatch 671 and pre-mismatch 183 splits, the phase-invalid fraction "
                "and every comparison with J11 remain DIAGNOSTIC and bind nothing.")}


# The EXACT 16 J11 binds. Preserved as its own tuple so "J11's set is preserved" is a checkable
# fact rather than a sentence: nothing here may be renamed, reordered away or dropped.
J11_BINDING_SUBSET = (
    "integrity_keys_and_shapes_match_parent", "integrity_all_parameters_finite",
    "integrity_clock_bit_zero", "integrity_aliases_bit_identical",
    "integrity_logstd_bit_identical_to_parent", "integrity_no_critic_key",
    "integrity_inputs_unchanged", "integrity_aggregate_reproduces_content_hashes",
    "integrity_split_counts_as_declared", "integrity_best_state_reconstructible_from_history",
    "integrity_all_metrics_finite", "aggregate_rmse_decreases", "recovery_rmse_decreases",
    "cell_B_unique_rmse_decreases", "cell_C_unique_rmse_decreases", "controller_columns_nonzero",
)

# The ONE gate J15 adds. Promoted from diagnostic to binding by the architect on 2026-08-27, on
# the authority of the approved J13 report, which preregistered the same strict inequality for the
# new corrective block. E and F are NOT split for this gate: the rule binds the whole 854 rows.
J15_ADDITIONAL_BINDING = ("j14_increment_rmse_decreases",)

# 17 = 16 + 1. J15's gate is NOT identical to J11's; it PRESERVES J11's and adds exactly one.
J15_BINDING_NAMES = J11_BINDING_SUBSET + J15_ADDITIONAL_BINDING
assert len(J15_BINDING_NAMES) == 17
assert set(J11_BINDING_SUBSET) < set(J15_BINDING_NAMES)
assert not set(J11_BINDING_SUBSET) & set(J15_ADDITIONAL_BINDING)


def fit_controller_norms(gate: Mapping[str, Any]) -> dict[str, float]:
    return {k: float(v) for k, v in gate["controller_column_norms"].items()}


# ================================================================ the manifest ===================

def build_manifest(fit: dict[str, Any], gate: dict[str, Any], state_sha: str) -> dict[str, Any]:
    """A NEW, TRUTHFUL manifest. Never a byte-identical copy of the parent's."""
    names = list(fit["names"])
    final = fit["final"]
    W = np.asarray(final["pi.0.0.weight"])
    return {
        "schema": "actor_feature_manifest.2",
        "generated_by": STAGE,
        "generated_at_stage": STAGE,
        "is_a_copy_of_the_parent_manifest": False,
        "why_not_a_copy": ("the parent's manifest describes the parent: its module_state_sha256 "
                           "is the parent's, its actor_label and status predate this fit, and its "
                           "controller_state_mask declares the controller block masked, which "
                           "this fit does not do."),
        "actor_feature_count": len(names),
        "actor_feature_names": names,
        "actor_width": ACTOR_WIDTH,
        "action_dim": ACTION_DIM,
        "module_state_sha256": state_sha,
        "module_state_sha256_is": "the SHA-256 of module_state.pkl BESIDE this manifest",
        "actor_digest": actor_state_digest(final),
        "actor_digest_algorithm": "warm_start.actor_state_digest over the ten sorted actor keys",
        "actor_digest_is_enforced_by": "warm_start.resolve_source_actor_features",
        "source_actor_digest": actor_state_digest(fit["parent"]),
        "lineage": {
            "operational": "August V26 imitation -> J2 35D -> J15 fresh post-mismatch refit",
            "derived_from": {"stage": "V26C_J2_BASE", "path": _rel(PARENT_MODULE_DIR),
                             "module_state_sha256": PIN_PARENT_STATE},
            "source_module_state_sha256": PIN_PARENT_STATE,
            "not_derived_from": {
                "j11": "J11 is the actor that FAILED J12. It is a SIBLING of this fit, never its "
                       "parent. Its module_state is pinned as negative evidence.",
                "j8": "a sibling, not an ancestor",
                "j4": "not an ancestor",
                "july": "methodology and evidence only; no July checkpoint is an ancestor"},
            "two_distinct_ancestors_do_not_conflate_them": {
                "weight_parent": {"what": "the module whose weights this fit started from",
                                  "stage": "V26C_J2_BASE", "sha256": PIN_PARENT_STATE},
                "observation_contract_ancestor": {
                    "what": "the 39-wide August V26 imitative actor that fixes the observation "
                            "contract under which the J10R1 teacher cells were collected. It is "
                            "NOT a weight ancestor and was never loaded by any of these stages.",
                    "sha256": PIN_OBSERVATION_CONTRACT_ANCESTOR}},
            "parent_chain_caveats": {
                "j2_leaf_has_no_commit_verification": True,
                "j2_authors_no_actor_digest": True,
                "integrity_rests_on": "the pinned SHA-256 of each J2 rl_module file"},
            "training_data": {
                "aggregate_rows": TOTAL_ROWS,
                "blocks": [{"id": b["id"], "rows": b["rows"], "unique": b["unique"],
                            "repeat": b["repeat"]} for b in fit["data"]["blocks"]],
                "aggregate_observations_sha256": fit["data"]["content_hashes"]["observations"],
                "aggregate_actions_sha256": fit["data"]["content_hashes"]["actions"],
                "j11_prefix_rows": J11_TOTAL_ROWS,
                "j14_corrective_rows": NEW_ROWS}},
        "clock_contract": {
            "columns": list(CLOCK_COLUMNS),
            "feature_names": [names[c] for c in CLOCK_COLUMNS],
            "weights_are_exactly_zero": bool(np.all(W[:, list(CLOCK_COLUMNS)] == 0.0)),
            "enforced": "hard-zeroed after every optimizer step and again at save",
            "consequence": "the clock inputs cannot influence the action, whatever the "
                           "environment records in those columns"},
        "controller_contract": {
            "columns": list(range(*CONTROLLER_SPAN)),
            "feature_names": names[CONTROLLER_SPAN[0]:CONTROLLER_SPAN[1]],
            "masked": False,
            "state": "LIVE - these columns are trained and carry non-zero weight",
            "column_l2_norms": fit_controller_norms(gate),
            "physical_scaling": "the July physical scales are absorbed into pi.0.0.weight at "
                                "save time, so this module consumes RAW observations"},
        "logstd_contract": {
            "rows": f"[{ACTION_DIM}:]",
            "bit_identical_to_parent": bool(
                gate["integrity"]["logstd_bit_identical_to_parent"]),
            "mechanism": "PROJECTION, not exclusion: the tensors are in the optimizer and their "
                         "log-std rows are overwritten with the parent's after every step. The "
                         "committed values are bit-identical to the parent's; Adam's moment "
                         "buffers for those entries are polluted and discarded."},
        "critic_contract": {
            "present": False,
            "meaning": "ABSENT, not preserved. A downstream stage instantiating this module gets "
                       "a RANDOMLY INITIALISED critic. This stage constructs none.",
            "sidecar_declares": "class_and_ctor_args.pkl is copied byte-identically from J2 and "
                                "declares freeze_logstd false and freeze_actor false. Those are "
                                "module-level flags for a consumer; this stage's freeze is "
                                "offline and does not depend on them."},
        "input_convention": "RAW observations, unscaled, in the feature order above",
        "deployable": False,
        "status": ("J15 offline refit " + ("PASS" if gate["pass"] else "FAIL")
                   + "; closed-loop qualification PENDING; not promoted; not deployable"),
        "offline_gate_pass": bool(gate["pass"]),
        "closed_loop_qualification": "PENDING - not run by this stage",
        "claims_not_made": [
            "no claim of closed-loop viability",
            "no claim that the J12 failure is fixed",
            "no claim of generalisation beyond AB06 and the starts of this trial",
            "no claim of deployability", "no promotion of any kind"],
    }


# ================================================================ destination ====================

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def validate_stage(token: str | None) -> None:
    if token != STAGE:
        raise J15Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")


def _refuse_symlink(path: Path, root: Path) -> None:
    probe = path
    while True:
        if probe.is_symlink():
            raise J15Error(f"refusing a symlinked path component: {probe}")
        if probe == root or probe.parent == probe:
            return
        probe = probe.parent


def validate_out(out_arg: str | None) -> Path:
    """The path rule J11 uses, plus an explicit absoluteness check.

    Both sides are resolved with strict=False so that /var vs /private/var and any other
    platform symlink cannot make an identical path look different.
    """
    if out_arg is None:
        raise J15Error("--fit requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if not got.is_absolute():
        raise J15Error(f"--out must be absolute, got {out_arg!r}")
    if got.is_symlink():
        raise J15Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J15Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J15Error(f"the authorised leaf already exists; this stage is no-clobber and "
                       f"single-execution: {leaf}")
    return leaf


# ================================================================ post-commit verification =======

def verify_committed_leaf(leaf: Path, *, expected_receipt_sha: str | None = None
                          ) -> dict[str, Any]:
    """Re-resolve and re-hash every path the COMMITTED receipt records. Read once, hash once."""
    receipt_path = leaf / RECEIPT_NAME
    raw = receipt_path.read_bytes()
    receipt_sha = hashlib.sha256(raw).hexdigest()
    receipt = json.loads(raw.decode("utf-8"))
    recorded = dict(receipt.get("committed_files_sha256", {}))
    missing: list[str] = []
    mismatched: list[str] = []
    recomputed: dict[str, str] = {}
    for rel_path, expected in sorted(recorded.items()):
        try:
            target = _resolve_inside(leaf, rel_path)
        except Exception:
            missing.append(rel_path)
            continue
        if not target.is_file():
            missing.append(rel_path)
            continue
        got = _sha_file(target)
        recomputed[rel_path] = got
        if got != expected:
            mismatched.append(rel_path)
    ok = (not missing and not mismatched and bool(recorded)
          and (expected_receipt_sha is None or receipt_sha == expected_receipt_sha))
    return {
        "schema": "v26c_j15_commit_verification.1", "stage": STAGE,
        "when": "AFTER os.rename, against the COMMITTED leaf",
        "verified_against": "the committed receipt",
        "pass": bool(ok),
        "files_checked": len(recomputed), "files_recorded": len(recorded),
        "paths_missing": missing, "hash_mismatches": mismatched,
        "recomputed_sha256": recomputed,
        "receipt_sha256": receipt_sha,
        "receipt_matches_staging_bytes": bool(expected_receipt_sha is None
                                              or receipt_sha == expected_receipt_sha),
        "meaning": "the leaf is VALID EVIDENCE if and only if this file exists and pass is true. "
                   "Its absence, or pass false, marks the leaf TECHNICALLY INVALID.",
        "on_failure": "preserved fail-closed and NOT promoted. No deletion, no retry, no repair.",
    }


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    """INERT. No torch, no fit, no write, no environment. Reports what WOULD happen."""
    torch_before = "torch" in sys.modules
    sentinel_before = PREFLIGHT_SENTINEL.exists()

    sources = verify_sources()
    modules = verify_modules()
    prereg = verify_prereg()
    hp = verify_hyperparameters_are_j11s()
    scales = J8.july_scales_from_source()

    source_leaves = {
        "j10r1": verify_source_leaf(J10R1_LEAF, label="the J10R1 leaf", expect_verdict=None),
        "j11": verify_source_leaf(J11_LEAF, label="the J11 leaf", expect_verdict=None),
        "j14": verify_source_leaf(J14_LEAF, label="the J14 leaf", expect_verdict="MATERIALIZED"),
    }

    data = build_aggregate()
    parent = load_parent_state()
    names = data["names"]
    preconditions = J8.parent_preconditions(parent, names, data["observations"])
    split = build_split(TOTAL_ROWS)

    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J15 lock is already held or was left behind: {lock_path}")

    torch_after = "torch" in sys.modules
    if torch_after and not torch_before:
        blockers.append("the preflight imported torch; it must stay torch-free")
    if PREFLIGHT_SENTINEL.exists() != sentinel_before or PREFLIGHT_SENTINEL.exists():
        blockers.append("the preflight created its sentinel path")

    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"fit_executed": False, "optimizer_steps": 0, "torch_imported": False,
                  "critic_touched": False, "ppo_updates": 0,
                  "environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "rollout": False,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False, "sealed_seeds_read": 0,
                  "torch_in_sys_modules_before": torch_before,
                  "torch_in_sys_modules_after": torch_after,
                  "torch_imported_by_this_preflight": bool(torch_after and not torch_before)},
        "modules": modules,
        "sources": sources,
        "source_leaves": source_leaves,
        "preregistration": prereg,
        "hyperparameters": hp,
        "parent": {"path": _rel(PARENT_MODULE_DIR), "module_state_sha256": PIN_PARENT_STATE,
                   "is_j2": True, "is_j11": False, "is_j8": False, "is_july": False,
                   "sidecars": dict(SIDECARS),
                   "state_keys": sorted(parent), "loaded_torch_free": True,
                   "leaf_has_commit_verification": False,
                   "integrity_rests_on": "the pinned SHA-256 of each rl_module file"},
        "aggregate": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names", "post_mismatch_mask")},
        "split": {"n_val": split["n_val"], "n_train": split["n_train"],
                  "declared_n_val": EXPECTED_N_VAL, "declared_n_train": EXPECTED_N_TRAIN,
                  "verified_not_assumed": True,
                  "formula": "max(1, int(round(rows * validation_fraction)))",
                  "rounding_note": "int(round(25567*0.2)) = 5113; a truncating int() would give "
                                   "5113 as well here, but the formula is kept identical to J11's",
                  "validation_sorted": True, "training_order_preserved": True,
                  "digest": split["digest"],
                  "known_limitation": "repeated blocks put the same unique row in both "
                                      "partitions; validation_mse is a training diagnostic only"},
        "preconditions": preconditions,
        "scales_verified": scales,
        "gate_matrix": gate_matrix(),
        "seeds": {"read_by_this_stage": [],
                  "present_in_the_j14_increment": list(ALLOWED_SEEDS),
                  "sealed_never_read": list(SEALED_SEEDS),
                  "sealed_seeds_read": 0},
        "would_write": {
            "leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
            "files": [AGGREGATE_NAME, RECEIPT_NAME, HISTORY_NAME, COMMIT_VERIFICATION_NAME,
                      f"{MODULE_DIRNAME}/{STATE_NAME}", f"{MODULE_DIRNAME}/{MANIFEST_NAME}"]
                     + [f"{MODULE_DIRNAME}/{n}" for n in sorted(BYTE_IDENTICAL_SIDECARS)]},
        "requires_to_fit": {"flag": "--fit", "stage_token": STAGE,
                            "out": "must equal the authorised leaf exactly",
                            "and": "an explicit architect GO record; this preflight grants none"},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "closed_loop_authorized": False, "critic_authorized": False,
                    "ppo_authorized_to_start": False},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def gate_matrix() -> dict[str, Any]:
    """The gate, declared before it runs.

    The 16-name J11 binding subset is PRESERVED, and J15 adds ONE binding rule,
    j14_increment_rmse_decreases. This gate is not J11's gate: it is J11's set plus one.
    """
    return {
        "binding": [
            {"name": n, "rule": "boolean must be true", "source": "preserved from J11"}
            for n in J15_BINDING_NAMES if n.startswith("integrity_")
        ] + [
            {"name": "aggregate_rmse_decreases", "rule": "RMSE after < RMSE before, rows "
             f"[0:{TOTAL_ROWS}]", "source": "preserved from J11", "margin": "none"},
            {"name": "recovery_rmse_decreases", "rule": "RMSE after < before, rows "
             f"[{J7_NOMINAL_ROWS}:{J7_ROWS}]", "source": "preserved from J11", "margin": "none"},
            {"name": "cell_B_unique_rmse_decreases", "rule": "RMSE after < before, 500 unique "
             "rows", "source": "preserved from J11", "margin": "none"},
            {"name": "cell_C_unique_rmse_decreases", "rule": "RMSE after < before, 500 unique "
             "rows", "source": "preserved from J11", "margin": "none"},
            {"name": "j14_increment_rmse_decreases",
             "rule": f"RMSE after < RMSE before, rows [{J11_TOTAL_ROWS}:{TOTAL_ROWS}] - the WHOLE "
                     f"854-row corrective block, cells E and F together",
             "source": "the approved J13 report, which preregistered 'RMSE dopo < prima su "
                       "aggregato, recovery 713, B unique, C unique e sul nuovo blocco E'. J14 "
                       "carries E AND F, so the same preregistered rule covers all 854 rows. "
                       "Promoted from diagnostic to BINDING by the architect on 2026-08-27.",
             "margin": "none",
             "not_a_new_threshold": "it is the same strict inequality against a measured "
                                    "baseline as the other four. No number is introduced."},
            {"name": "controller_columns_nonzero", "rule": "all ten column L2 norms > 0",
             "source": "preserved from J11", "margin": "none"},
        ],
        "binding_count": len(J15_BINDING_NAMES),
        "identical_to_j11": False,
        "relation_to_j11": "the 16-name J11 binding subset is preserved, name for name, and "
                           "J15 adds one binding rule: j14_increment_rmse_decreases. 16 + 1 = 17. "
                           "The 17th is not an invention - it is the rule the approved J13 report "
                           "already preregistered for the new corrective block, and it introduces "
                           "no numeric threshold.",
        "diagnostics_not_binding": [
            "nominal_rmse", "nominal_mean_shift", "j14_cell_E_rmse", "j14_cell_F_rmse",
            "j14_post_mismatch_rmse", "j14_pre_mismatch_rmse", "j14_phase_invalid_fraction",
            "j11_prefix_rmse", "clipping_out_of_bounds_rows", "best_validation_mse",
            "epochs_run", "stopped_early", "torch_backend_observed"],
        "what_stays_diagnostic_and_why": {
            "cells_E_and_F_separately": "kept SEPARATE for visibility, but the gate binds the "
                                        "854-row block as a whole. A per-cell gate would be a "
                                        "new rule; per-cell reporting is not.",
            "post_mismatch_671_and_pre_mismatch_183": "reported, never used to filter, weight or "
                                                      "truncate a row",
            "phase_invalid_fraction": "listed as a diagnostic by the approved J13 report; it "
                                      "stays one",
            "comparisons_with_j11": "j11_prefix_rmse has no preregistered direction, so it "
                                    "cannot be a gate"},
        "supervised_only": "passing this gate says NOTHING about closed-loop behaviour. The "
                           "J12-style regression gate is a separate, later stage.",
        "no_invented_thresholds": "every one of the FIVE binding RMSE rules is a strict "
                                  "inequality against a measured baseline, and the fifth "
                                  "introduces no number the first four did not already use. No "
                                  "absolute target appears anywhere.",
    }


# ================================================================ receipt and commit =============

def build_receipt(fit: dict[str, Any], gate: dict[str, Any], *,
                  committed: Mapping[str, str], injected: bool) -> dict[str, Any]:
    data = fit["data"]
    return {
        "schema": "v26c_j15_fresh_refit_receipt.1",
        "stage": STAGE,
        "verdict": "PASS" if gate["pass"] else "FAIL",
        "authoritative": not injected,
        "injected_fit_double": injected,
        "date": "2026-08-27",
        "preregistration": fit["prereg"],
        "modules": fit["modules"],
        "sources": verify_sources(),
        "hyperparameters": verify_hyperparameters_are_j11s(),
        "aggregate": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names", "post_mismatch_mask")},
        "split": {"n_val": fit["split"]["n_val"], "n_train": fit["split"]["n_train"],
                  "declared_n_val": EXPECTED_N_VAL, "declared_n_train": EXPECTED_N_TRAIN,
                  "digest": fit["split"]["digest"],
                  "val_membership_sha256": fit["split"]["val_membership_sha256"],
                  "train_membership_sha256": fit["split"]["train_membership_sha256"],
                  "validation_sorted": True, "training_order_preserved": True,
                  "known_limitation": "repeated blocks put the same unique row in both "
                                      "partitions; validation_mse is a training diagnostic only"},
        "preconditions": fit["preconditions"],
        "scales_verified": fit["scales_verified"],
        "training": {
            "trainable_parameters": list(DIRECT_KEYS),
            "aliases_not_optimised": [a for a, _ in ALIAS_PAIRS],
            "logstd_rows_frozen": f"[{ACTION_DIM}:]",
            "logstd_freeze_mechanism": "projection after every step, not requires_grad and not "
                                       "optimizer exclusion",
            "critic": "never constructed",
            "epochs_run": fit["epochs_run"], "best_epoch": fit["best_epoch"],
            "best_validation_mse": fit["best_validation_mse"],
            "stopped_early": fit["stopped_early"],
            "seed_order": fit["seed_order"],
            "numpy_legacy_seed": fit["numpy_legacy_seed"],
            "torch_backend_observed": fit["torch_backend_observed"],
            "optimizer_steps_derived": int(
                ((EXPECTED_N_TRAIN + int(JULY_HP["batch_size"]) - 1)
                 // int(JULY_HP["batch_size"])) * fit["epochs_run"]),
            "optimizer_steps_is_derived_not_measured": True},
        "gate": {"schema": "v26c_j15_gate.2",
                 "binding": gate["binding"], "failed": gate["failed"], "pass": gate["pass"],
                 "identical_to_j11": False,
                 "relation_to_j11": gate["relation_to_j11"],
                 "integrity": gate["integrity"], "rmse": gate["rmse"],
                 "controller_column_norms": gate["controller_column_norms"],
                 "no_invented_thresholds": gate["no_invented_thresholds"],
                 "supervised_only": gate["supervised_only"]},
        "diagnostics": gate["diagnostics"],
        "actor_digest": actor_state_digest(fit["final"]),
        "source_actor_digest": actor_state_digest(fit["parent"]),
        "actor_digest_recorded_here_and_in_the_manifest": True,
        "lineage": {
            "operational": "August V26 imitation -> J2 35D -> J15",
            "parent": {"stage": "V26C_J2_BASE", "path": _rel(PARENT_MODULE_DIR),
                       "module_state_sha256": PIN_PARENT_STATE},
            "not_j11": {"why": "J11 is the actor that FAILED J12; this is a FRESH fit",
                        "j11_module_state_sha256": PIN_J11_STATE,
                        "j11_was_loaded": False},
            "july_role": "methodology and evidence only"},
        "committed_files_sha256": dict(committed),
        "commit_verification": {"file": COMMIT_VERIFICATION_NAME,
                                "written": "after the rename, against the committed leaf"},
        "inert": {"rollout": False, "environment_constructed": False, "critic_touched": False,
                  "ppo_updates": 0, "promotion": "NONE", "sealed_seeds_read": 0},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "closed_loop_authorized": False, "fit_authorized": False,
                    "closed_loop_claim": "NONE - this stage is supervised only"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def commit(out_arg: str | None, stage_token: str | None, *, progress: bool = False,
           fit: dict[str, Any] | None = None, gate: dict[str, Any] | None = None
           ) -> dict[str, Any]:
    """Write the leaf. An INJECTED fit is a test double and is refused an authoritative root."""
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    injected = fit is not None
    if injected and OUTPUT_ROOT_OVERRIDE is None:
        raise J15Error("an injected fit is a TEST DOUBLE and may never write to the authoritative "
                       "root. Set OUTPUT_ROOT_OVERRIDE first.")
    if OUTPUT_ROOT_OVERRIDE is not None and not injected:
        raise J15Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. A real fit is "
                       f"only ever written to the authoritative root; this stage refuses to run "
                       f"a real fit into a redirected one.")

    if fit is None:
        fit = run_fit(progress=progress)
    if gate is None:
        gate = audit(fit)
    if not gate["pass"] and not gate["failed"]:
        raise J15Error("a failing gate must name what failed")

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J15Error(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    lock_owned: Path | None = None
    staging_created: Path | None = None
    commit_verified = False
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            raise J15Error(f"the J15 lock already exists: {lock_path}. This stage fails closed "
                           f"rather than remove a lock it does not own.") from None
        lock_owned = lock_path
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"stage": STAGE, "pid": os.getpid()}))

        staging.mkdir()
        staging_created = staging
        module_dir = staging / MODULE_DIRNAME
        module_dir.mkdir()

        np.savez_compressed(staging / AGGREGATE_NAME,
                            observations=fit["data"]["observations"],
                            actions=fit["data"]["actions"],
                            actor_feature_names=np.asarray(list(fit["names"]), dtype="<U34"))
        with (module_dir / STATE_NAME).open("wb") as fh:
            pickle.dump({k: np.asarray(v, dtype=np.float32) for k, v in fit["final"].items()},
                        fh, protocol=4)
        state_sha = _sha_file(module_dir / STATE_NAME)
        for name in BYTE_IDENTICAL_SIDECARS:
            shutil.copyfile(PARENT_MODULE_DIR / name, module_dir / name)
            if _sha_file(module_dir / name) != SIDECARS[name]:
                raise J15Error(f"the copied sidecar {name} does not reproduce the parent's hash")
        (module_dir / MANIFEST_NAME).write_text(
            json.dumps(build_manifest(fit, gate, state_sha), indent=2, ensure_ascii=False,
                       allow_nan=False) + "\n", encoding="utf-8")
        (staging / HISTORY_NAME).write_text(
            json.dumps(fit["history"], indent=2, allow_nan=False) + "\n", encoding="utf-8")

        with (module_dir / STATE_NAME).open("rb") as fh:
            reloaded = pickle.load(fh)
        if tuple(sorted(reloaded)) != tuple(sorted(fit["final"])) or not all(
                np.array_equal(np.asarray(reloaded[k]), np.asarray(fit["final"][k]))
                for k in fit["final"]):
            raise J15Error("the saved module state does not reload bit-identically")

        committed = {}
        for q in sorted(staging.rglob("*")):
            if q.is_file() and not q.is_symlink():
                committed[_leaf_rel(q, staging)] = _sha_file(q)

        receipt = build_receipt(fit, gate, committed=committed, injected=injected)
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False,
                       default=str) + "\n", encoding="utf-8")
        staging_receipt_sha = _sha_file(staging / RECEIPT_NAME)

        # THE LEAF IS BORN INVALID. Written into staging, so it is committed BY the rename.
        (staging / TECHNICAL_INVALID_NAME).write_text(
            "TECHNICALLY INVALID - UNVERIFIED\n"
            f"stage: {STAGE}\n"
            f"see: {COMMIT_VERIFICATION_NAME}\n"
            "This marker is written BEFORE the commit and removed only after the post-commit "
            "verification has passed. While it is present the leaf is NOT valid evidence.\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J15Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        marker = leaf / TECHNICAL_INVALID_NAME
        if not marker.is_file():
            raise J15Error(f"the committed leaf does not carry {TECHNICAL_INVALID_NAME}: "
                           f"{leaf} is not the directory this run staged")
        try:
            verification = verify_committed_leaf(leaf, expected_receipt_sha=staging_receipt_sha)
        except Exception as exc:
            verification = {"schema": "v26c_j15_commit_verification.1", "stage": STAGE,
                            "pass": False, "verifier_error": f"{type(exc).__name__}: {exc}",
                            "meaning": "verification could not complete; the leaf is TECHNICALLY "
                                       "INVALID and is preserved unpromoted"}
        (leaf / COMMIT_VERIFICATION_NAME).write_text(
            json.dumps(verification, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
        if verification.get("pass") is True:
            marker.unlink()          # the LAST write: the leaf becomes valid evidence here
            commit_verified = True

        return {"leaf": str(leaf), "relative_leaf": RELATIVE_LEAF,
                "verdict": receipt["verdict"], "gate_pass": gate["pass"],
                "failed": gate["failed"],
                "receipt_sha256": staging_receipt_sha,
                "dataset_sha256": committed[AGGREGATE_NAME],
                "module_state_sha256": state_sha,
                "actor_digest": receipt["actor_digest"],
                "commit_verification": {"file": COMMIT_VERIFICATION_NAME,
                                        "pass": verification.get("pass"),
                                        "files_checked": verification.get("files_checked")},
                "technically_invalid_marker_removed": commit_verified,
                "authoritative": not injected,
                "outcome": receipt["outcome"]}
    finally:
        if lock_owned is not None and lock_owned.exists():
            lock_owned.unlink()
        if staging_created is not None and staging_created.exists():
            shutil.rmtree(staging_created)
        if parent_created is not None and parent_created.exists() and not any(
                parent_created.iterdir()):
            parent_created.rmdir()


# ================================================================ entry point ====================

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J15 fresh 35D post-mismatch refit")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--fit", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--progress", action="store_true")
    args = p.parse_args(argv)

    if args.out is not None and not args.fit:
        raise J15Error("--out is meaningless without --fit; the preflight writes nothing")
    if args.preflight and args.fit:
        raise J15Error("--preflight and --fit are mutually exclusive")
    if args.fit:
        print(json.dumps(commit(args.out, args.authorized_stage, progress=args.progress),
                         indent=2, default=str))
        return 0
    print(json.dumps(preflight(), indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
