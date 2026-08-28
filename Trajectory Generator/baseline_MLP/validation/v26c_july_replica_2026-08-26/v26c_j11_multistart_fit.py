#!/usr/bin/env python
"""V26C J11 - MULTISTART OFFLINE FIT on the August V26 lineage.

WHAT THIS IS
    One offline supervised fit of the full mean network, starting FRESH from the J2 base actor,
    on an aggregate of 24713 rows materialised at execution time from three ALREADY-COMMITTED
    sources: the J7 Markov dataset (16713 rows) and the two J10R1 multistart teacher cells
    (500 rows each, each block repeated eight times).

WHAT THIS IS NOT
    Not PPO. Not a critic. Not a rollout. No environment is constructed, reset or stepped. The
    J8 student is neither a parent nor a label source. There is one actor, 35 wide; nothing is
    widened and no contralateral feature is added.

LINEAGE
    August V26 imitation -> J2 35D -> THIS fit. July is METHODOLOGY AND EVIDENCE ONLY: no July
    checkpoint, dataset, label or artefact is an operative parent here.

THE PARENT IS J2, NOT J8
    j2_runs/j2_base_v26c_2026-08-26_r1/rl_module, module_state 0f182ea9...
    J8 fitted the same parent on the J7 rows alone. J11 does not continue J8: it starts again
    from J2 with the larger dataset, so the two fits are siblings, not a chain.

CLOCK COLUMNS - a MEASURED fact, not an assumption
    With the gait clock disabled the environment records the raw clock pair as (sin, cos) =
    (0, 1). J7 was already projected to exact zero; the J10R1 cells carry the raw (0, 1). This
    stage projects columns 0:2 to EXACT zero before aggregating and before the fit, exactly as
    July and J7 did, and records both the RAW and the PROJECTED content hashes. A raw cos of 1
    is expected and is never a reason to reject a cell.

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
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# The FROZEN J8 runner, imported as a LIBRARY of audited, stage-neutral helpers. Importing it
# performs no I/O and pulls no torch: its only `import torch` sits inside its own run_fit.
# It is pinned by hash below and never executed.
import v26c_j8_recovery_fit as J8  # noqa: E402


class J11Error(RuntimeError):
    pass


STAGE = "V26C_J11_MULTISTART_FIT"
PREREG = HERE / "v26c_j11_prereg_multistart_fit.json"
PIN_PREREG = "49e352466cc82ea9c0a1d7bf29608d41ba4457dfb8a83f3a050a6f1cd752e472"

# ------------------------------------------------------------------ pinned modules --------------
PIN_J8_MODULE = "76047cb844c2ae720fab3e7b24d90aa563451e6b038c1d1e98098765762e3899"

# ------------------------------------------------------------------ inputs -----------------------
J7_LEAF = HERE / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1"
J7_DATASET = J7_LEAF / "v26c_j7_markov_recovery_dataset.npz"
PIN_J7_DATASET = "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
J7_RECEIPT = J7_LEAF / "v26c_j7_markov_dataset_receipt.json"
PIN_J7_RECEIPT = "39cbed8072f0126b84ba40a12d1268991a128e5af3794d6fb74b246c2d509ca2"
PIN_J7_OBS_CONTENT = "a59fb89b43a372162661ec8cf131b5abbf1e9ce32b16f98518af9010a4cf587a"
PIN_J7_ACT_CONTENT = "6e920c2f890dbdc2cfacf6a2c6842b34c6f33e24499d0d8716bd6347624bde4f"

J10R1_LEAF = HERE / "j10r1_runs" / "j10r1_multistart_teacher_v26c_2026-08-27_r1"
J10R1_RECEIPT = J10R1_LEAF / "v26c_j10r1_multistart_teacher_receipt.json"
PIN_J10R1_RECEIPT = "9c1c26f6e1aaaa96f2c92cb45f6494e889b3124dd6ca99427a11026a8d099f20"
J10R1_COMMIT_VERIFICATION = J10R1_LEAF / "commit_verification.json"
CELL_DATASETS = {
    "B": (J10R1_LEAF / "j10r1_cell_B_teacher_dataset.npz",
          "2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62"),
    "C": (J10R1_LEAF / "j10r1_cell_C_teacher_dataset.npz",
          "bd78e6ac13ab96d128f57ea5b36d058f5d80c18cfb056bcc76d2179bf1d756f0"),
}

PARENT_MODULE_DIR = J8.PARENT_MODULE_DIR
PIN_PARENT_STATE = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
# NOT a weight ancestor and never loaded here. It is the 39-wide August V26 imitative actor that
# fixed the OBSERVATION CONTRACT under which the J10R1 teacher cells were collected. Recorded
# distinctly because the two hashes are easy to conflate in a receipt.
PIN_OBSERVATION_CONTRACT_ANCESTOR = \
    "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
SIDECARS = dict(J8.SIDECARS)
# The two sidecars that are carried forward BYTE-IDENTICAL. The manifest is NOT among them:
# it is regenerated truthfully, because the parent's copy describes the parent, not this module.
BYTE_IDENTICAL_SIDECARS = ("class_and_ctor_args.pkl", "metadata.json")
MANIFEST_NAME = "actor_feature_manifest.json"

# ------------------------------------------------------------------ composition ------------------
ACTOR_WIDTH = J8.ACTOR_WIDTH                      # 35
ACTION_DIM = J8.ACTION_DIM                        # 2
CLOCK_COLUMNS = J8.CLOCK_COLUMNS                  # (0, 1)
DIRECT_KEYS = J8.DIRECT_KEYS
ALIAS_PAIRS = J8.ALIAS_PAIRS
STATE_KEYS = J8.STATE_KEYS
CONTROLLER_SPAN = (25, 35)

J7_ROWS = 16713
J7_NOMINAL_UNIQUE = 500
J7_NOMINAL_REPEAT = 32
J7_NOMINAL_ROWS = J7_NOMINAL_UNIQUE * J7_NOMINAL_REPEAT       # 16000
J7_RECOVERY_ROWS = J7_ROWS - J7_NOMINAL_ROWS                  # 713
CELL_UNIQUE = 500
CELL_REPEAT = 8
CELL_ROWS = CELL_UNIQUE * CELL_REPEAT                         # 4000
TOTAL_ROWS = J7_ROWS + 2 * CELL_ROWS                          # 24713

# The aggregate layout, stated once and enforced everywhere.
#   [0     : 16000)  J7 nominal, 500 unique states TILED 32 times
#   [16000 : 16713)  J7 recovery, 713 unique rows          <- binding RMSE subset
#   [16713 : 20713)  cell B, 500 unique rows TILED 8 times <- unique block is the first 500
#   [20713 : 24713)  cell C, 500 unique rows TILED 8 times <- unique block is the first 500
BLOCKS = (
    {"id": "j7_nominal", "start": 0, "stop": J7_NOMINAL_ROWS,
     "unique": J7_NOMINAL_UNIQUE, "repeat": J7_NOMINAL_REPEAT, "tiled": True},
    {"id": "j7_recovery", "start": J7_NOMINAL_ROWS, "stop": J7_ROWS,
     "unique": J7_RECOVERY_ROWS, "repeat": 1, "tiled": False},
    {"id": "cell_B", "start": J7_ROWS, "stop": J7_ROWS + CELL_ROWS,
     "unique": CELL_UNIQUE, "repeat": CELL_REPEAT, "tiled": True},
    {"id": "cell_C", "start": J7_ROWS + CELL_ROWS, "stop": TOTAL_ROWS,
     "unique": CELL_UNIQUE, "repeat": CELL_REPEAT, "tiled": True},
)

# TILE, not REPEAT. Measured on the committed J7 dataset: rows [0:500] == [500:1000] == ... ==
# [15500:16000], and the nominal block holds exactly 500 unique rows. np.repeat would instead
# place each row's copies consecutively and would produce a DIFFERENT dataset with the same
# shape. The cells are built the same way, so their unique block is always the first 500 rows.
REPEAT_SEMANTICS = "np.tile: the whole unique block is concatenated `repeat` times, contiguously"

# JULY EVIDENCE, recorded for methodology only. No July artefact is a parent, a dataset or a
# label here. Measured on the July run artefacts by an independent audit:
#   markov_dataset_report.json:  nominal 500 x 32 = 16000
#                                recovery 356 unique x 2 = 712
#                                each off-nominal start 500 x 8 = 4000
#                                aggregate 24712
#   order: [nominal x32] ++ [recovery x2] ++ [B x8] ++ [C x8], built with np.tile
#   (target_domain_markov_adaptation.py:174-187 and :362-390)
# August differs in ONE place and it is already preregistered in J7: the recovery block is 713
# UNIQUE rows taken once, not 356 unique taken twice. That is why the aggregate is 24713 and not
# 24712. The ORDER, the tile semantics and the repeat factor 8 are identical.
JULY_EVIDENCE = {
    "aggregate_rows": 24712,
    "nominal": {"unique": 500, "repeat": 32, "rows": 16000},
    "recovery": {"unique": 356, "repeat": 2, "rows": 712},
    "off_nominal_each": {"unique": 500, "repeat": 8, "rows": 4000},
    "order": "[nominal x32] ++ [recovery x2] ++ [B x8] ++ [C x8]",
    "tile_not_repeat": True,
    "validation_count": 4942,
    "difference_from_august": ("August's recovery block is 713 unique rows taken ONCE, from the "
                               "J7 stage, instead of 356 unique taken twice. The aggregate is "
                               "therefore 24713 rather than 24712. Order, tile semantics and the "
                               "repeat factor 8 are unchanged."),
    "repeat_factor_8_rationale": ("NOT FOUND in any July report, comment or config. The value "
                                  "survives only as data: recovery_dataset_repeat: 8 in the run "
                                  "report and the alt8 token in the run directory name. The CLI "
                                  "default was 1, so 8 was passed explicitly and deliberately. "
                                  "This stage REPLICATES an observed value; it does not claim a "
                                  "documented justification for it."),
    "status": "METHODOLOGY AND EVIDENCE ONLY - never an operational parent, dataset or label",
}

# ------------------------------------------------------------------ hyperparameters --------------
JULY_HP = dict(J8.JULY_HP)
BEST_EPSILON = J8.BEST_EPSILON
EXPECTED_N_VAL = 4943            # max(1, int(round(24713 * 0.2))) - VERIFIED at run time
EXPECTED_N_TRAIN = TOTAL_ROWS - EXPECTED_N_VAL                # 19770

# ------------------------------------------------------------------ the only destination ---------
RELATIVE_LEAF_PARTS = ("j11_runs", "j11_multistart_fit_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
MODULE_DIRNAME = "rl_module"
STATE_NAME = "module_state.pkl"
RECEIPT_NAME = "v26c_j11_multistart_fit_receipt.json"
HISTORY_NAME = "history.json"
AGGREGATE_NAME = "v26c_j11_aggregate_dataset.npz"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
TECHNICAL_INVALID_NAME = "TECHNICAL_INVALID"

OUTPUT_ROOT_OVERRIDE: Path | None = None

PREFLIGHT_SENTINEL = HERE / "_j11_preflight_sentinel_never_created"

FORBIDDEN_HERE = ("critic", "PPO", "optimizer over anything but the six direct tensors",
                  "environment construction", "rollout", "closed-loop evaluation", "promotion",
                  "deployability", "a second parent", "the J8 student as a parent",
                  "a July checkpoint as a parent", "widening", "contralateral features",
                  "a standalone 25D actor", "LOTO", "LOCO", "B1R1", "B1R2",
                  "training ex novo", "an autonomous retry", "an invented threshold",
                  "a byte-identical copy of the parent's actor_feature_manifest.json")


# ================================================================ small helpers ==================

_sha_file = J8._sha_file
_sha_array = J8._sha_array
_sha_obj = J8._sha_obj
_numpy_forward = J8._numpy_forward
_rmse = J8._rmse


# warm_start.actor_state_digest, TRANSCRIBED. Importing warm_start would pull torch and the
# preflight must stay free of it. The source is pinned, and the transcription is proven against
# the real function in the test suite.
WARM_START_SOURCE = REPO / "Trajectory Generator" / "baseline_MLP" / "warm_start.py"
PIN_WARM_START = "84706218dcc4c5cb7f97a8f3f67ef40ba9e064ba5aef25cb6559d1c8a506c34c"
WARM_START_ACTOR_KEYS = ("pi_encoder.0.weight", "pi.0.0.weight", "pi_encoder.0.bias",
                         "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.bias",
                         "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")


def _tensor_digest(value: Any) -> str:
    arr = np.ascontiguousarray(value)
    h = hashlib.sha256()
    h.update(str(arr.dtype).encode("ascii"))
    h.update(repr(tuple(int(d) for d in arr.shape)).encode("ascii"))
    h.update(arr.tobytes(order="C"))
    return h.hexdigest()


def actor_state_digest(state: Mapping[str, Any]) -> str:
    """Byte-for-byte the algorithm of warm_start.actor_state_digest, without importing it."""
    h = hashlib.sha256()
    for key in sorted(WARM_START_ACTOR_KEYS):
        if key not in state:
            raise J11Error(f"actor state is missing {key}")
        h.update(key.encode("utf-8"))
        h.update(_tensor_digest(state[key]).encode("ascii"))
    return h.hexdigest()


def verify_warm_start_source() -> str:
    digest = _sha_file(WARM_START_SOURCE)
    if digest != PIN_WARM_START:
        raise J11Error(f"warm_start.py changed: {digest} != {PIN_WARM_START}. The actor_digest "
                       f"transcription is only valid against the pinned source.")
    return digest


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


def _leaf_rel(p: Path, root: Path) -> str:
    """LEAF-RELATIVE, so the string stays valid after the staging is renamed onto the leaf."""
    try:
        rel = Path(p).relative_to(root)
    except ValueError:
        raise J11Error(f"{p} is not under the staging root {root}") from None
    if ".." in rel.parts:
        raise J11Error(f"the relative path for {p} under {root} escapes the leaf: {rel}")
    return str(rel).replace(os.sep, "/")


def _resolve_inside(leaf: Path, rel: str) -> Path:
    if not rel or Path(rel).is_absolute() or ".." in Path(rel).parts:
        raise J11Error(f"recorded path is not leaf-relative: {rel!r}")
    q = leaf / rel
    if q.is_symlink():
        raise J11Error(f"recorded path is a symlink, which is never committed here: {rel}")
    try:
        q.resolve().relative_to(leaf.resolve())
    except ValueError:
        raise J11Error(f"recorded path escapes the committed leaf: {rel}") from None
    return q


# ================================================================ inputs =========================

def verify_modules() -> dict[str, str]:
    """The frozen J8 runner is a LIBRARY here. Pin it, and prove it is the file we imported."""
    path = Path(J8.__file__).resolve()
    digest = _sha_file(path)
    if digest != PIN_J8_MODULE:
        raise J11Error(f"the J8 module changed: {digest} != {PIN_J8_MODULE}")
    if path.parent != HERE:
        raise J11Error(f"the imported J8 module is not the one beside this stage: {path}")
    return {"v26c_j8_recovery_fit.py": digest}


def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J11Error("the J11 preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J11Error(f"the J11 preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage_proposed") != STAGE:
        raise J11Error(f"the preregistration proposes {data.get('stage_proposed')!r}, not {STAGE}")
    local: dict[str, str] = {}
    for rel, pin in data["pinned_artefacts_sha256"].items():
        got = _sha_file(HERE / rel)
        if got != pin:
            raise J11Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        local[rel] = got
    repo: dict[str, str] = {}
    for rel, pin in data["pinned_repo_artefacts_sha256"].items():
        got = _sha_file(REPO / rel)
        if got != pin:
            raise J11Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        repo[rel] = got
    comp = data["aggregate_composition"]
    if int(comp["total_rows"]) != TOTAL_ROWS:
        raise J11Error(f"the preregistration declares {comp['total_rows']} rows, not {TOTAL_ROWS}")
    if [b["id"] for b in comp["blocks"]] != [b["id"] for b in BLOCKS]:
        raise J11Error("the preregistered block order and the runner's disagree")
    for want, got in zip(BLOCKS, comp["blocks"]):
        if (int(got["start"]), int(got["stop"]), int(got["unique"]), int(got["repeat"])) != \
                (want["start"], want["stop"], want["unique"], want["repeat"]):
            raise J11Error(f"block {want['id']} differs from the preregistration")
    if comp["repeat_semantics"].split(":")[0] != "np.tile":
        raise J11Error("the preregistration must declare np.tile semantics")
    hp = data["hyperparameters"]
    for k, v in JULY_HP.items():
        if k in hp and hp[k] != v:
            raise J11Error(f"the preregistration and the runner disagree on {k}")
    if data["parent"]["module_state_sha256"] != PIN_PARENT_STATE:
        raise J11Error("the preregistration names a different parent")
    if data["parent"]["is_j8"] is not False or data["parent"]["is_july"] is not False:
        raise J11Error("the parent must be declared as neither J8 nor July")
    if data["manifest_policy"]["byte_identical_copy"] is not False:
        raise J11Error("the manifest must be REGENERATED, never copied byte-for-byte")
    return {"file": _rel(PREREG), "sha256": digest,
            "manifest_entries": len(local) + len(repo),
            "pinned_artefacts_sha256": local, "pinned_repo_artefacts_sha256": repo}


def _load_npz(path: Path, pin: str, expect_keys: tuple[str, ...]) -> dict[str, Any]:
    digest = _sha_file(path)
    if digest != pin:
        raise J11Error(f"{path.name} changed: {digest} != {pin}")
    with np.load(path, allow_pickle=False) as archive:
        keys = tuple(sorted(archive.files))
        if keys != expect_keys:
            raise J11Error(f"{path.name} holds {keys}, not {expect_keys}")
        out = {k: np.asarray(archive[k]) for k in archive.files}
    out["_sha256"] = digest
    return out


def load_j7() -> dict[str, Any]:
    """The MATERIALISED J7 bytes. The builder is never imported and nothing is rebuilt."""
    receipt_digest = _sha_file(J7_RECEIPT)
    if receipt_digest != PIN_J7_RECEIPT:
        raise J11Error(f"the J7 receipt changed: {receipt_digest} != {PIN_J7_RECEIPT}")
    z = _load_npz(J7_DATASET, PIN_J7_DATASET,
                  ("actions", "actor_feature_names", "observations"))
    obs, act = z["observations"], z["actions"]
    names = tuple(str(n) for n in z["actor_feature_names"].tolist())
    if obs.shape != (J7_ROWS, ACTOR_WIDTH) or act.shape != (J7_ROWS, ACTION_DIM):
        raise J11Error(f"J7 is {obs.shape}/{act.shape}, expected "
                       f"{(J7_ROWS, ACTOR_WIDTH)}/{(J7_ROWS, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J11Error(f"J7 is {obs.dtype}/{act.dtype}, expected float32")
    if _sha_array(obs) != PIN_J7_OBS_CONTENT or _sha_array(act) != PIN_J7_ACT_CONTENT:
        raise J11Error("the J7 arrays do not reproduce the audited content hashes")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J11Error("J7 holds non-finite values")
    # the declared layout: 32 bit-identical repeats, then the recovery rows
    for r in range(J7_NOMINAL_REPEAT):
        block = slice(r * J7_NOMINAL_UNIQUE, (r + 1) * J7_NOMINAL_UNIQUE)
        if not (np.array_equal(obs[block], obs[:J7_NOMINAL_UNIQUE])
                and np.array_equal(act[block], act[:J7_NOMINAL_UNIQUE])):
            raise J11Error(f"J7 nominal repeat {r} is not bit-identical to the first block")
    if float(np.max(np.abs(obs[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J11Error("the J7 clock columns are not exactly zero")
    return {"observations": obs, "actions": act, "names": names, "sha256": z["_sha256"],
            "receipt_sha256": receipt_digest,
            "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act)},
            "clock_already_zero": True}


def load_cell(cid: str) -> dict[str, Any]:
    """One J10R1 teacher cell. The PRESCRIBED actions are the labels.

    The raw clock pair is (0, 1) here, which is what the environment records when the gait clock
    is disabled. That is EXPECTED and is never a reason to reject the cell: the projection to
    exact zero happens below, and both the raw and the projected content hashes are recorded.
    """
    path, pin = CELL_DATASETS[cid]
    z = _load_npz(path, pin, ("action_noises", "actions", "actor_feature_names",
                              "executed_actions", "observations", "times"))
    obs, act = z["observations"], z["actions"]
    ex, noise = z["executed_actions"], z["action_noises"]
    names = tuple(str(n) for n in z["actor_feature_names"].tolist())
    if obs.shape != (CELL_UNIQUE, ACTOR_WIDTH) or act.shape != (CELL_UNIQUE, ACTION_DIM):
        raise J11Error(f"cell {cid} is {obs.shape}/{act.shape}, expected "
                       f"{(CELL_UNIQUE, ACTOR_WIDTH)}/{(CELL_UNIQUE, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J11Error(f"cell {cid} is {obs.dtype}/{act.dtype}, expected float32")
    if not np.array_equal(act, ex):
        raise J11Error(f"cell {cid}: actions and executed_actions are not bit-identical")
    if float(np.max(np.abs(noise))) != 0.0:
        raise J11Error(f"cell {cid}: the action noise is not exactly zero")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J11Error(f"cell {cid} holds non-finite values")

    raw_clock = {"col_0_unique": sorted({float(v) for v in obs[:, 0]}),
                 "col_1_unique": sorted({float(v) for v in obs[:, 1]})}
    projected = obs.copy()
    projected[:, list(CLOCK_COLUMNS)] = 0.0
    if float(np.max(np.abs(projected[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J11Error(f"cell {cid}: the clock projection did not reach exact zero")
    if not np.array_equal(projected[:, 2:], obs[:, 2:]):
        raise J11Error(f"cell {cid}: the clock projection disturbed a non-clock column")
    return {"observations_raw": obs, "observations": projected, "actions": act, "names": names,
            "sha256": z["_sha256"], "times": z["times"],
            "raw_clock": raw_clock,
            "content_hashes": {"observations_raw": _sha_array(obs),
                               "observations_projected": _sha_array(projected),
                               "actions": _sha_array(act)},
            "unique_rows": int(len(np.unique(obs, axis=0)))}


def verify_j10r1_provenance() -> dict[str, Any]:
    """The cells may only be consumed from a leaf whose own commit verification PASSED."""
    got = _sha_file(J10R1_RECEIPT)
    if got != PIN_J10R1_RECEIPT:
        raise J11Error(f"the J10R1 receipt changed: {got} != {PIN_J10R1_RECEIPT}")
    if not J10R1_COMMIT_VERIFICATION.is_file():
        raise J11Error(f"the J10R1 leaf carries no {COMMIT_VERIFICATION_NAME}: it is not valid "
                       f"evidence and its datasets may not be consumed")
    if (J10R1_LEAF / TECHNICAL_INVALID_NAME).exists():
        raise J11Error("the J10R1 leaf is marked TECHNICAL_INVALID and may not be consumed")
    cv = json.loads(J10R1_COMMIT_VERIFICATION.read_text())
    if cv.get("pass") is not True:
        raise J11Error(f"the J10R1 commit verification does not pass: {cv.get('pass')!r}")
    receipt = json.loads(J10R1_RECEIPT.read_text())
    if receipt.get("verdict") != "PASS":
        raise J11Error(f"the J10R1 verdict is {receipt.get('verdict')!r}, not PASS")
    recorded = {}
    for cell in receipt["cells"]:
        rel = cell["artefacts"]["teacher_dataset"]
        recorded[cell["id"]] = cell["artefact_sha256"][rel]
    for cid, (path, pin) in CELL_DATASETS.items():
        if recorded.get(cid) != pin:
            raise J11Error(f"the J10R1 receipt records cell {cid} as {recorded.get(cid)}, "
                           f"not the pinned {pin}")
    return {"receipt_sha256": got, "commit_verification_pass": True,
            "technical_invalid_marker": False, "verdict": receipt["verdict"],
            "dataset_hashes_agree_with_the_j10r1_receipt": True,
            "binding_hash_honoured": "the J10R1 receipt declared each teacher_dataset SHA binding "
                                     "for this stage; both are verified here before consumption"}


def build_aggregate() -> dict[str, Any]:
    """Materialise the 24713-row aggregate. TILE semantics, clock projected to EXACT zero."""
    j7 = load_j7()
    prov = verify_j10r1_provenance()
    cells = {cid: load_cell(cid) for cid in ("B", "C")}

    names = j7["names"]
    for cid, cell in cells.items():
        if cell["names"] != names:
            raise J11Error(f"cell {cid} feature names differ from J7's")
    manifest_names = J8.actor_feature_names()
    if tuple(manifest_names) != names:
        raise J11Error("the dataset schema differs from the parent's actor manifest")

    obs_parts = [j7["observations"]]
    act_parts = [j7["actions"]]
    for cid in ("B", "C"):
        obs_parts.append(np.tile(cells[cid]["observations"], (CELL_REPEAT, 1)))
        act_parts.append(np.tile(cells[cid]["actions"], (CELL_REPEAT, 1)))
    obs = np.ascontiguousarray(np.concatenate(obs_parts, axis=0).astype(np.float32))
    act = np.ascontiguousarray(np.concatenate(act_parts, axis=0).astype(np.float32))

    if obs.shape != (TOTAL_ROWS, ACTOR_WIDTH) or act.shape != (TOTAL_ROWS, ACTION_DIM):
        raise J11Error(f"the aggregate is {obs.shape}/{act.shape}, expected "
                       f"{(TOTAL_ROWS, ACTOR_WIDTH)}/{(TOTAL_ROWS, ACTION_DIM)}")
    if obs.dtype != np.float32 or act.dtype != np.float32:
        raise J11Error(f"the aggregate is {obs.dtype}/{act.dtype}, expected float32")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J11Error("the aggregate holds non-finite values")
    if float(np.max(np.abs(obs[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J11Error("the aggregate's clock columns are not exactly zero")

    # every block must be exactly what it claims, measured on the assembled array
    block_report = []
    for b in BLOCKS:
        seg_o = obs[b["start"]:b["stop"]]
        seg_a = act[b["start"]:b["stop"]]
        if seg_o.shape[0] != b["stop"] - b["start"]:
            raise J11Error(f"block {b['id']} has the wrong length")
        if b["tiled"]:
            head_o = seg_o[:b["unique"]]
            head_a = seg_a[:b["unique"]]
            for r in range(b["repeat"]):
                sl = slice(r * b["unique"], (r + 1) * b["unique"])
                if not (np.array_equal(seg_o[sl], head_o) and np.array_equal(seg_a[sl], head_a)):
                    raise J11Error(f"block {b['id']} tile {r} is not bit-identical to the first")
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
        raise J11Error("the J7 rows are not bit-identical inside the aggregate")
    for cid, b in (("B", BLOCKS[2]), ("C", BLOCKS[3])):
        head = obs[b["start"]:b["start"] + CELL_UNIQUE]
        if not np.array_equal(head, cells[cid]["observations"]):
            raise J11Error(f"cell {cid}'s projected rows are not bit-identical in the aggregate")
        if not np.array_equal(act[b["start"]:b["start"] + CELL_UNIQUE], cells[cid]["actions"]):
            raise J11Error(f"cell {cid}'s labels are not bit-identical in the aggregate")

    return {
        "observations": obs, "actions": act, "names": names,
        "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act)},
        "rows": TOTAL_ROWS, "blocks": block_report,
        "repeat_semantics": REPEAT_SEMANTICS,
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
                       "unique_rows": cells["C"]["unique_rows"]}},
        "j10r1_provenance": prov,
        "clock_projection": {
            "columns": list(CLOCK_COLUMNS),
            "feature_names": [names[c] for c in CLOCK_COLUMNS],
            "j7_state": "already exactly zero before this stage",
            "cells_raw_state": "(sin, cos) = (0, 1), which is what the environment records with "
                               "the gait clock disabled",
            "action": "projected to EXACT zero before aggregation and before the fit, as July "
                      "and J7 did",
            "expected_not_a_defect": "a raw cos of 1 is the documented disabled-clock encoding "
                                     "and is never a reason to reject a cell",
            "raw_and_projected_both_hashed": True},
        "binding_subsets": {
            "aggregate": [0, TOTAL_ROWS],
            "j7_recovery_original": [J7_NOMINAL_ROWS, J7_ROWS],
            "cell_B_unique": [BLOCKS[2]["start"], BLOCKS[2]["start"] + CELL_UNIQUE],
            "cell_C_unique": [BLOCKS[3]["start"], BLOCKS[3]["start"] + CELL_UNIQUE]},
    }


def load_parent_state() -> dict[str, np.ndarray]:
    """Fresh from disk. The J8 student is never a parent; no July checkpoint is ever a parent."""
    path = PARENT_MODULE_DIR / STATE_NAME
    digest = _sha_file(path)
    if digest != PIN_PARENT_STATE:
        raise J11Error(f"the J2 parent state changed: {digest} != {PIN_PARENT_STATE}")
    for name, pin in SIDECARS.items():
        got = _sha_file(PARENT_MODULE_DIR / name)
        if got != pin:
            raise J11Error(f"the parent sidecar {name} changed: {got} != {pin}")
    with path.open("rb") as fh:
        state = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}
    if tuple(sorted(state)) != tuple(sorted(STATE_KEYS)):
        raise J11Error(f"the parent holds {tuple(sorted(state))}, expected {sorted(STATE_KEYS)}")
    if any("critic" in k or k.startswith("vf") for k in state):
        raise J11Error("the parent carries a critic key; this stage excludes the critic entirely")
    return state


def build_split(rows: int) -> dict[str, Any]:
    """July's split, via the FROZEN J8 transcription. Verified against the declared counts."""
    split = J8.build_split(rows)
    if rows == TOTAL_ROWS and (split["n_val"], split["n_train"]) != (EXPECTED_N_VAL,
                                                                    EXPECTED_N_TRAIN):
        raise J11Error(f"the split produced {split['n_val']}/{split['n_train']}, not the "
                       f"declared {EXPECTED_N_VAL}/{EXPECTED_N_TRAIN}. The declared numbers are "
                       f"VERIFIED here, never assumed.")
    return split


# ================================================================ the fit ========================

def run_fit(*, progress: bool = False) -> dict[str, Any]:
    """Train the full mean network IN MEMORY and audit the result. Writes nothing."""
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
        "observed_at": "fit time; the preflight is torch-free and cannot observe them"}

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
        raise J11Error(f"the torch kernel does not reproduce the raw/scaled equivalence: "
                       f"max_abs_diff {torch_diff}. No tolerance is assumed.")
    preconditions["raw_vs_scaled_equivalence"]["torch_max_abs_diff"] = torch_diff
    preconditions["raw_vs_scaled_equivalence"]["torch_bit_identical"] = True
    preconditions["raw_vs_scaled_equivalence"]["kernels_checked"] = ["numpy float32", "torch"]

    # (5) the single Generator, AFTER the deterministic initialisation and BEFORE the optimizer
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
    """Every binding check, measured. No invented margin: strict inequalities only."""
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

    # every pinned input, re-hashed after the fit
    unchanged = (
        _sha_file(J7_DATASET) == PIN_J7_DATASET
        and _sha_file(J7_RECEIPT) == PIN_J7_RECEIPT
        and _sha_file(J10R1_RECEIPT) == PIN_J10R1_RECEIPT
        and all(_sha_file(p) == pin for p, pin in CELL_DATASETS.values())
        and _sha_file(PARENT_MODULE_DIR / STATE_NAME) == PIN_PARENT_STATE
        and all(_sha_file(PARENT_MODULE_DIR / n) == pin for n, pin in SIDECARS.items())
        and _sha_file(Path(J8.__file__).resolve()) == PIN_J8_MODULE)
    integrity["inputs_unchanged"] = bool(unchanged)

    # the aggregate reproduces its own recorded content hashes
    integrity["aggregate_reproduces_content_hashes"] = bool(
        _sha_array(obs) == data["content_hashes"]["observations"]
        and _sha_array(act) == data["content_hashes"]["actions"])

    # the split is the declared one
    integrity["split_counts_as_declared"] = bool(
        fit["split"]["n_val"] == EXPECTED_N_VAL and fit["split"]["n_train"] == EXPECTED_N_TRAIN)

    # the best state is reconstructible from the recorded history
    replay_best, replay_epoch = float("inf"), 0
    for row in fit["history"]:
        if float(row["validation_mse"]) < replay_best - BEST_EPSILON:
            replay_best, replay_epoch = float(row["validation_mse"]), int(row["epoch"])
    integrity["best_state_reconstructible_from_history"] = bool(
        replay_epoch == fit["best_epoch"]
        and abs(replay_best - fit["best_validation_mse"]) <= 0.0)

    # ---- RMSE on the four binding subsets ----------------------------------------------------
    sub = data["binding_subsets"]
    rmse: dict[str, Any] = {}
    for key in ("aggregate", "j7_recovery_original", "cell_B_unique", "cell_C_unique"):
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
    nominal_a, nominal_b = 0, J7_NOMINAL_ROWS
    diagnostics = {
        "nominal_rmse": {
            "before": _rmse(before[nominal_a:nominal_b], act[nominal_a:nominal_b]),
            "after": _rmse(after[nominal_a:nominal_b], act[nominal_a:nominal_b]),
            "binding": False,
            "why_not": "the nominal block is the self-anchor; it is expected to move and no "
                       "direction is preregistered for it"},
        "nominal_mean_shift": {
            "value": float(np.max(np.abs(after[nominal_a:nominal_b].astype(np.float64)
                                         - before[nominal_a:nominal_b].astype(np.float64)))),
            "binding": False},
        "clipping_out_of_bounds_rows": {
            "value": int(np.sum(np.any(np.abs(after) > 1.0, axis=1))),
            "binding": False,
            "why_not": "the clip term is a loss weight, not a gate; the environment clips"},
        "best_validation_mse": {
            "value": float(fit["best_validation_mse"]), "binding": False,
            "why_not": "every block is repeated, so the same unique row appears in BOTH "
                       "partitions. Inherited from July and from J7, recorded and NOT corrected: "
                       "this is a training diagnostic, never a generalisation estimate."},
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
        "controller_columns_nonzero": controller_all_nonzero,
    }
    failed = sorted(k for k, v in binding.items() if not v)
    return {"binding": binding, "failed": failed, "pass": not failed,
            "integrity": integrity, "rmse": rmse, "diagnostics": diagnostics,
            "controller_column_norms": controller_norms,
            "controller_columns_nonzero": controller_all_nonzero,
            "no_invented_thresholds": (
                "every binding numeric rule is a STRICT inequality against a measured baseline: "
                "'after < before' on four subsets and 'norm > 0' on ten columns. No margin, no "
                "tolerance and no absolute target is asserted anywhere.")}


# ================================================================ the manifest ===================

def build_manifest(fit: dict[str, Any], gate: dict[str, Any], state_sha: str) -> dict[str, Any]:
    """A NEW, TRUTHFUL manifest. Never a byte-identical copy of the parent's.

    The parent's actor_feature_manifest.json describes the PARENT: it carries the parent's
    module_state_sha256, the label J2_BASE35_JULY_FAITHFUL, a status that predates this fit and a
    controller_state_mask declaring the controller block masked - which this fit does not do.
    J8 copied it verbatim and therefore ships a module whose manifest names a different module.
    This stage refuses that: the manifest below states what is true of the module beside it, and
    claims nothing that has not been demonstrated.
    """
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
                           "this fit does not do. A copied manifest would misdescribe this "
                           "module on every one of those fields."),
        "actor_feature_count": len(names),
        "actor_feature_names": names,
        "actor_width": ACTOR_WIDTH,
        "action_dim": ACTION_DIM,
        "module_state_sha256": state_sha,
        "module_state_sha256_is": "the SHA-256 of module_state.pkl BESIDE this manifest",
        # ENFORCED, not documentation: warm_start.resolve_source_actor_features compares this
        # against actor_state_digest(source_state) and refuses the checkpoint on a mismatch.
        # J2, J4 and J8 all omit it, which is why J8's stale manifest was never caught.
        "actor_digest": actor_state_digest(final),
        "actor_digest_algorithm": "warm_start.actor_state_digest over the ten sorted actor keys",
        "actor_digest_is_enforced_by": "warm_start.resolve_source_actor_features",
        "source_actor_digest": actor_state_digest(fit["parent"]),
        "lineage": {
            "operational": "August V26 imitation -> J2 35D -> J11 multistart offline fit",
            "derived_from": {
                "stage": "V26C_J2_BASE",
                "path": _rel(PARENT_MODULE_DIR),
                "module_state_sha256": PIN_PARENT_STATE},
            "source_module_state_sha256": PIN_PARENT_STATE,
            "two_distinct_ancestors_do_not_conflate_them": {
                "weight_parent": {
                    "what": "the module whose weights this fit started from",
                    "stage": "V26C_J2_BASE", "sha256": PIN_PARENT_STATE},
                "observation_contract_ancestor": {
                    "what": "the 39-wide August V26 imitative actor that fixes the observation "
                            "contract under which the J10R1 teacher cells were collected. It is "
                            "NOT a weight ancestor and was never loaded by any of these stages.",
                    "sha256": PIN_OBSERVATION_CONTRACT_ANCESTOR}},
            "not_derived_from": {
                "j8": "the J8 recovery fit is a SIBLING of this one, not an ancestor",
                "j4": "not an ancestor",
                "july": "July is methodology and evidence only; no July checkpoint is an "
                        "ancestor of this module"},
            "training_data": {
                "aggregate_rows": TOTAL_ROWS,
                "blocks": [{"id": b["id"], "rows": b["rows"], "unique": b["unique"],
                            "repeat": b["repeat"]} for b in fit["data"]["blocks"]],
                "aggregate_observations_sha256":
                    fit["data"]["content_hashes"]["observations"],
                "aggregate_actions_sha256": fit["data"]["content_hashes"]["actions"]}},
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
        "input_convention": "RAW observations, unscaled, in the feature order above",
        "deployable": False,
        "status": ("J11 offline fit " + ("PASS" if gate["pass"] else "FAIL")
                   + "; closed-loop qualification PENDING; not promoted; not deployable"),
        "offline_gate_pass": bool(gate["pass"]),
        "closed_loop_qualification": "PENDING - not run by this stage",
        "critic": "ABSENT - this module carries no value-function tensor and this stage "
                  "constructs none",
        "claims_not_made": [
            "no claim of closed-loop viability",
            "no claim of generalisation beyond AB06 and the three starts of this trial",
            "no claim of deployability",
            "no promotion of any kind"],
    }


def fit_controller_norms(gate: Mapping[str, Any]) -> dict[str, float]:
    return {k: float(v) for k, v in gate["controller_column_norms"].items()}


# ================================================================ destination ====================

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def validate_stage(token: str | None) -> None:
    if token != STAGE:
        raise J11Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J11Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J11Error("--fit requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J11Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J11Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J11Error(f"the authorised leaf already exists; this stage is no-clobber and "
                       f"single-execution: {leaf}")
    return leaf


# ================================================================ post-commit verification =======

def verify_committed_leaf(leaf: Path, *, expected_receipt_sha: str | None = None) -> dict[str, Any]:
    """Re-resolve EVERY recorded leaf-relative path in the COMMITTED leaf and re-hash it.

    Adopted from J10R1. J8 had no post-commit verification at all: it recorded the receipt's
    hash after the rename and compared it to nothing.
    """
    receipt = leaf / RECEIPT_NAME
    receipt_bytes = receipt.read_bytes() if receipt.is_file() else None
    receipt_sha = hashlib.sha256(receipt_bytes).hexdigest() if receipt_bytes is not None else None
    receipt_matches_staging: bool | None = None
    if expected_receipt_sha is not None:
        receipt_matches_staging = receipt_sha == expected_receipt_sha
    if receipt_bytes is None:
        return {"schema": "v26c_j11_commit_verification.1", "stage": STAGE, "pass": False,
                "paths_missing": [RECEIPT_NAME], "hash_mismatches": [],
                "meaning": "the committed receipt is missing; the leaf is TECHNICALLY INVALID"}
    recorded = json.loads(receipt_bytes.decode("utf-8"))["committed_files_sha256"]

    missing: list[str] = []
    mismatches: list[dict[str, str]] = []
    recomputed: dict[str, str] = {}
    for rel, want in sorted(recorded.items()):
        q = _resolve_inside(leaf, rel)
        if not q.is_file():
            missing.append(rel)
            continue
        got = _sha_file(q)
        recomputed[rel] = got
        if got != want:
            mismatches.append({"path": rel, "expected": want, "recomputed": got})
    if receipt_matches_staging is False:
        mismatches.append({"path": RECEIPT_NAME, "expected": str(expected_receipt_sha),
                           "recomputed": str(receipt_sha)})
    ok = not missing and not mismatches
    return {
        "schema": "v26c_j11_commit_verification.1", "stage": STAGE,
        "when": "AFTER os.rename, against the COMMITTED leaf",
        "verified_against": "the committed receipt",
        "pass": ok, "files_checked": len(recomputed), "files_recorded": len(recorded),
        "paths_missing": missing, "hash_mismatches": mismatches,
        "recomputed_sha256": recomputed,
        "receipt_sha256": receipt_sha,
        "receipt_matches_staging_bytes": receipt_matches_staging,
        "meaning": ("the leaf is VALID EVIDENCE if and only if this file exists and pass is true. "
                    "Its absence, or pass false, marks the leaf TECHNICALLY INVALID."),
        "on_failure": "preserved fail-closed and NOT promoted. No deletion, no retry, no repair.",
    }


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    """INERT. No torch, no fit, no write, no environment. Reports what WOULD happen."""
    torch_before = "torch" in sys.modules
    sentinel_before = PREFLIGHT_SENTINEL.exists()

    modules = verify_modules()
    modules["warm_start.py"] = verify_warm_start_source()
    prereg = verify_prereg()
    scales = J8.july_scales_from_source()
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
        blockers.append(f"a J11 lock is already held or was left behind: {lock_path}")

    torch_after = "torch" in sys.modules
    if torch_after and not torch_before:
        blockers.append("the preflight imported torch; it must stay torch-free")
    if PREFLIGHT_SENTINEL.exists() != sentinel_before or PREFLIGHT_SENTINEL.exists():
        blockers.append("the preflight created its sentinel path")

    # the parent's manifest, described honestly rather than trusted
    parent_manifest = json.loads((PARENT_MODULE_DIR / MANIFEST_NAME).read_text())
    manifest_audit = {
        "path": _rel(PARENT_MODULE_DIR / MANIFEST_NAME),
        "sha256": SIDECARS[MANIFEST_NAME],
        "declared_module_state_sha256": parent_manifest.get("module_state_sha256"),
        "describes_the_parent_not_this_module": True,
        "will_be_copied_byte_for_byte": False,
        "will_be_regenerated": True,
        "fields_that_would_be_false_if_copied": [
            "module_state_sha256 (names the parent's state, not the module J11 writes)",
            "actor_label (predates this fit)",
            "status (predates this fit)",
            "controller_state_mask.active (this fit trains the controller block unmasked)",
            "derived_from (points past this stage's parent)"],
        "j8_precedent": ("J8 copied this file verbatim and therefore ships a module whose "
                         "manifest names a different module. This stage does not repeat that."),
    }

    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"fit_executed": False, "optimizer_steps": 0, "torch_imported": False,
                  "critic_touched": False, "ppo_updates": 0,
                  "environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "rollout": False,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False,
                  "torch_in_sys_modules_before": torch_before,
                  "torch_in_sys_modules_after": torch_after,
                  "torch_imported_by_this_preflight": bool(torch_after and not torch_before)},
        "modules": modules,
        "preregistration": prereg,
        "parent": {"path": _rel(PARENT_MODULE_DIR), "module_state_sha256": PIN_PARENT_STATE,
                   "is_j2": True, "is_j8": False, "is_july": False,
                   "sidecars": dict(SIDECARS),
                   "state_keys": sorted(parent), "loaded_torch_free": True},
        "aggregate": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names")},
        "split": {"n_val": split["n_val"], "n_train": split["n_train"],
                  "declared_n_val": EXPECTED_N_VAL, "declared_n_train": EXPECTED_N_TRAIN,
                  "verified_not_assumed": True,
                  "formula": "max(1, int(round(rows * validation_fraction)))",
                  "rounding_note": "int(round(24713*0.2)) = 4943; a truncating int() would give "
                                   "4942, so the formula matters",
                  "validation_sorted": True, "training_order_preserved": True,
                  "digest": split["digest"]},
        "preconditions": preconditions,
        "hyperparameters": dict(JULY_HP),
        "scales_verified": scales,
        "july_evidence": JULY_EVIDENCE,
        "manifest_policy": manifest_audit,
        "would_write": {
            "leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
            "files": [AGGREGATE_NAME, RECEIPT_NAME, HISTORY_NAME, COMMIT_VERIFICATION_NAME,
                      f"{MODULE_DIRNAME}/{STATE_NAME}",
                      f"{MODULE_DIRNAME}/{MANIFEST_NAME}"]
                     + [f"{MODULE_DIRNAME}/{n}" for n in sorted(BYTE_IDENTICAL_SIDECARS)]},
        "requires_to_fit": {"flag": "--fit", "stage_token": STAGE,
                            "out": "must equal the authorised leaf exactly"},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "closed_loop_authorized": False, "critic_authorized": False,
                    "ppo_authorized_to_start": False},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ receipt and commit =============

def build_receipt(fit: dict[str, Any], gate: dict[str, Any], *,
                  committed: dict[str, str], injected: bool) -> dict[str, Any]:
    data = fit["data"]
    return {
        "schema": "v26c_j11_multistart_fit_receipt.1",
        "stage": STAGE,
        "verdict": "PASS" if gate["pass"] else "FAIL",
        "authoritative": OUTPUT_ROOT_OVERRIDE is None,
        "preregistration": fit["prereg"],
        "modules_pinned": fit["modules"],
        "parent": {"path": _rel(PARENT_MODULE_DIR), "module_state_sha256": PIN_PARENT_STATE,
                   "is_j2": True, "is_j8": False, "is_july": False,
                   "fresh_start": "this fit begins from the J2 parent, not from J8 and not from "
                                  "any July checkpoint; J8 is a sibling fit, not an ancestor"},
        "aggregate": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names")},
        "aggregate_file": AGGREGATE_NAME,
        "july_evidence": JULY_EVIDENCE,
        "observation_contract_ancestor": {
            "sha256": PIN_OBSERVATION_CONTRACT_ANCESTOR,
            "what": "the 39-wide August V26 imitative actor under which the J10R1 cells were "
                    "collected. NOT a weight ancestor; never loaded by this stage.",
            "do_not_conflate_with": "the weight parent " + PIN_PARENT_STATE},
        "actor_feature_names": list(fit["names"]),
        "hyperparameters": dict(JULY_HP),
        "determinism": {"seed_order": fit["seed_order"],
                        "numpy_legacy_seed": fit["numpy_legacy_seed"],
                        "generators": 1,
                        "torch_backend_observed": fit["torch_backend_observed"],
                        "use_deterministic_algorithms_forced": False,
                        "set_num_threads_forced": False,
                        "why_not_forced": "neither call exists in the July snapshot; forcing "
                                          "them would be an invention, not a replication"},
        "split": {"n_val": fit["split"]["n_val"], "n_train": fit["split"]["n_train"],
                  "declared_n_val": EXPECTED_N_VAL, "declared_n_train": EXPECTED_N_TRAIN,
                  "verified_not_assumed": True,
                  "digest": fit["split"]["digest"],
                  "val_membership_sha256": fit["split"]["val_membership_sha256"],
                  "train_membership_sha256": fit["split"]["train_membership_sha256"],
                  "validation_sorted": True, "training_order_preserved": True,
                  "repeated_rows_span_both_partitions": (
                      "every block is tiled, so the same unique row appears in both partitions. "
                      "Inherited from July and from J7, recorded and NOT corrected. The "
                      "validation MSE is a training diagnostic, never a generalisation estimate.")},
        "preconditions": fit["preconditions"],
        "training": {"epochs_run": fit["epochs_run"], "best_epoch": fit["best_epoch"],
                     "best_validation_mse": fit["best_validation_mse"],
                     "stopped_early": fit["stopped_early"],
                     "trainable_parameters": list(DIRECT_KEYS),
                     "aliases_not_optimised": [a for a, _ in ALIAS_PAIRS],
                     "logstd_rows_frozen": f"[{ACTION_DIM}:]"},
        "gate": {"binding": gate["binding"], "failed": gate["failed"], "pass": gate["pass"],
                 "no_invented_thresholds": gate["no_invented_thresholds"]},
        "rmse": gate["rmse"],
        "integrity": gate["integrity"],
        "controller_column_norms": gate["controller_column_norms"],
        "diagnostics_not_binding": gate["diagnostics"],
        "manifest_policy": {
            "regenerated": True, "byte_identical_copy": False,
            "byte_identical_sidecars": sorted(BYTE_IDENTICAL_SIDECARS),
            "why": "the parent's manifest describes the parent; copying it would ship a module "
                   "whose manifest names a different module, as J8's leaf does"},
        "committed_files_sha256": committed,
        "commit_verification": {
            "file": COMMIT_VERIFICATION_NAME,
            "state_when_this_receipt_was_written": "PENDING",
            "validity_rule": (f"this leaf is VALID EVIDENCE if and only if "
                              f"{COMMIT_VERIFICATION_NAME} exists beside this receipt and "
                              f"declares pass true"),
            "marker_on_failure": TECHNICAL_INVALID_NAME},
        "stack": {"injected": injected,
                  "note": "an INJECTED run is a test double: this receipt is NOT operational "
                          "evidence" if injected else "the operational stack"},
        "inert": {"critic_touched": False, "ppo_updates": 0, "environment_constructed": False,
                  "rollout": False, "student_used": False, "second_parent": False,
                  "widening": False, "contralateral_features": False},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "closed_loop_authorized": False, "critic_authorized": False,
                    "ppo_authorized_to_start": False, "single_execution": True,
                    "no_autonomous_retry": True,
                    "note": "this stage produces a fitted actor and authorises nothing"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def commit(out_arg: str | None, stage_token: str | None, *, progress: bool = False,
           fit: dict[str, Any] | None = None, gate: dict[str, Any] | None = None
           ) -> dict[str, Any]:
    """Write the leaf. An INJECTED fit is a test double and is refused an authoritative root.

    The fit is injectable for the same reason J8's is: exercising the whole write path - lock
    contention, staging cleanup, the born-invalid marker, the post-commit verification - must not
    require a real 400-epoch training run.
    """
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    injected = fit is not None
    if injected and OUTPUT_ROOT_OVERRIDE is None:
        raise J11Error("an injected fit is a TEST DOUBLE and may never write to the authoritative "
                       "root. Set OUTPUT_ROOT_OVERRIDE first.")
    if OUTPUT_ROOT_OVERRIDE is not None and not injected:
        raise J11Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. A real fit is "
                       f"only ever written to the authoritative root; this stage refuses to run "
                       f"a real fit into a redirected one.")

    if fit is None:
        fit = run_fit(progress=progress)
    if gate is None:
        gate = audit(fit)
    if not gate["pass"] and not gate["failed"]:
        raise J11Error("a failing gate must name what failed")

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J11Error(f"a stale staging directory is in the way: {staging}")

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
            raise J11Error(f"the J11 lock already exists: {lock_path}. This stage fails closed "
                           f"rather than remove a lock it does not own.") from None
        lock_owned = lock_path
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"stage": STAGE, "pid": os.getpid()}))

        staging.mkdir()
        staging_created = staging
        module_dir = staging / MODULE_DIRNAME
        module_dir.mkdir()

        # 1. the aggregate dataset, auditable
        np.savez_compressed(staging / AGGREGATE_NAME,
                            observations=fit["data"]["observations"],
                            actions=fit["data"]["actions"],
                            actor_feature_names=np.asarray(list(fit["names"]), dtype="<U34"))
        # 2. the module state
        with (module_dir / STATE_NAME).open("wb") as fh:
            pickle.dump({k: np.asarray(v, dtype=np.float32) for k, v in fit["final"].items()},
                        fh, protocol=4)
        state_sha = _sha_file(module_dir / STATE_NAME)
        # 3. the two byte-identical sidecars, re-verified against the parent pins
        for name in BYTE_IDENTICAL_SIDECARS:
            shutil.copyfile(PARENT_MODULE_DIR / name, module_dir / name)
            if _sha_file(module_dir / name) != SIDECARS[name]:
                raise J11Error(f"the copied sidecar {name} does not reproduce the parent's hash")
        # 4. the REGENERATED manifest
        (module_dir / MANIFEST_NAME).write_text(
            json.dumps(build_manifest(fit, gate, state_sha), indent=2, ensure_ascii=False,
                       allow_nan=False) + "\n", encoding="utf-8")
        # 5. history
        (staging / HISTORY_NAME).write_text(
            json.dumps(fit["history"], indent=2, allow_nan=False) + "\n", encoding="utf-8")

        # round-trip: the committed state must reload bit-identically
        with (module_dir / STATE_NAME).open("rb") as fh:
            reloaded = pickle.load(fh)
        if tuple(sorted(reloaded)) != tuple(sorted(fit["final"])) or not all(
                np.array_equal(np.asarray(reloaded[k]), np.asarray(fit["final"][k]))
                for k in fit["final"]):
            raise J11Error("the saved module state does not reload bit-identically")

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
            raise J11Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        marker = leaf / TECHNICAL_INVALID_NAME
        if not marker.is_file():
            raise J11Error(f"the committed leaf does not carry {TECHNICAL_INVALID_NAME}: "
                           f"{leaf} is not the directory this run staged")
        try:
            verification = verify_committed_leaf(
                leaf, expected_receipt_sha=staging_receipt_sha)
        except Exception as exc:
            verification = {"schema": "v26c_j11_commit_verification.1", "stage": STAGE,
                            "pass": False, "verifier_error": f"{type(exc).__name__}: {exc}",
                            "meaning": "verification could not complete; the leaf is TECHNICALLY "
                                       "INVALID and is preserved unpromoted"}
        try:
            (leaf / COMMIT_VERIFICATION_NAME).write_text(
                json.dumps(verification, indent=2, ensure_ascii=False, allow_nan=False,
                           default=str) + "\n", encoding="utf-8")
        except OSError as exc:
            raise J11Error(f"the verification could not be recorded in {leaf}: "
                           f"{type(exc).__name__}: {exc}. The leaf remains marked "
                           f"{TECHNICAL_INVALID_NAME}.") from exc
        if not verification.get("pass"):
            try:
                marker.write_text(
                    "TECHNICALLY INVALID - VERIFICATION FAILED\n"
                    f"stage: {STAGE}\nsee: {COMMIT_VERIFICATION_NAME}\n"
                    "The committed content did not reproduce the paths and hashes recorded in "
                    "the receipt. Preserved as evidence about the commit; not a module.\n",
                    encoding="utf-8")
            except OSError:
                pass
            raise J11Error(
                f"POST-COMMIT VERIFICATION FAILED for {leaf}: "
                f"{len(verification.get('paths_missing') or [])} paths did not resolve, "
                f"{len(verification.get('hash_mismatches') or [])} hashes did not reproduce.")
        try:
            marker.unlink()
        except OSError as exc:
            raise J11Error(f"the verification passed but the marker could not be removed from "
                           f"{leaf}: {type(exc).__name__}: {exc}") from exc
        commit_verified = True
    except BaseException:
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            try:
                shutil.rmtree(staging_created)
            except OSError as _rm:
                print(f"WARNING: the staging directory could not be removed: {staging_created}: "
                      f"{type(_rm).__name__}: {_rm}. It will BLOCK the next run and must be "
                      f"removed by hand; this stage never repairs itself.", file=sys.stderr)
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

    return {"verdict": receipt["verdict"], "stage": STAGE, "leaf": _rel(leaf),
            "gate_pass": gate["pass"], "failed": gate["failed"],
            "rmse": {k: {"before": v["before"], "after": v["after"], "decreased": v["decreased"]}
                     for k, v in gate["rmse"].items()},
            "best_epoch": fit["best_epoch"], "epochs_run": fit["epochs_run"],
            "receipt_sha256": _sha_file(leaf / RECEIPT_NAME),
            "module_state_sha256": _sha_file(leaf / MODULE_DIRNAME / STATE_NAME),
            "aggregate_sha256": _sha_file(leaf / AGGREGATE_NAME),
            "commit_verification": {"file": COMMIT_VERIFICATION_NAME, "pass": commit_verified,
                                    "files_checked": verification.get("files_checked")},
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "closed_loop_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J11 multistart offline fit")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--fit", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--progress", action="store_true")
    a = p.parse_args(argv)
    if a.fit:
        r = commit(a.out, a.authorized_stage, progress=a.progress)
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "PASS" else 1
    if a.out is not None:
        raise J11Error("--out is meaningless without --fit; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
