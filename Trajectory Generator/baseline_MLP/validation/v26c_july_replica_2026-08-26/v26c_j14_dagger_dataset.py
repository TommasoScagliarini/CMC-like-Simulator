#!/usr/bin/env python
"""V26C J14 - POST-MISMATCH DAgger DATASET, materialised from already-committed artefacts.

WHAT THIS IS
    One content-addressed dataset leaf holding the 854 corrective rows that the J13 diagnosis
    identified as missing: every step the J11 student actually visited in the two J12 cells that
    failed, E (seed 124, 500 rows) and F (seed 125, 354 rows), labelled with the J1 teacher at
    the SAME STEP, with the gait-clock columns projected to exact zero.

WHAT THIS IS NOT
    No rollout. No fit. No optimizer. No critic. No PPO. No environment is constructed, and torch
    is never imported - not even lazily. Nothing is promoted and nothing is deployable.

THE POINT: NO TRUNCATION
    J7's recovery block truncates at the first discrete mismatch, so it contains only states from
    BEFORE the student diverges. That is precisely why the corpus has no coverage of the prolonged
    swing in which E and F fail. This stage keeps EVERY row, including the post-mismatch region,
    and records for each row whether it falls before or after the first mismatch as a DIAGNOSTIC.
    The flag is never a filter. This is July's own unexecuted binding TODO of 2026-07-14:
    "raccogliere dati recovery realmente event-aligned e indipendenti, includendo la regione
    successiva al mismatch FSM invece di sole interpolazioni pre-evento".

SEED 125 IS NOT HELD OUT, AND CANNOT BE
    J7 already contains 11 rows from seed 125 (rows [16702:16713], bit-identical to the J6 seed-125
    probe). A seed that is already in the training corpus cannot be called held out. The genuine
    held-out reserve is seeds 126, 127, 128, which this stage NEVER reads and which no artefact
    here references.

LINEAGE
    August V26 imitation -> J2 35D -> the current pipeline. July is METHODOLOGY AND EVIDENCE ONLY.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


class J14Error(RuntimeError):
    pass


STAGE = "V26C_J14_POST_MISMATCH_DAGGER_DATASET"
# rev1. The rev0 preregistration is preserved byte-identical and is INERT: this runner no longer
# references it, and its pin table omits the three J11 sources that coverage_report actually reads.
PREREG = HERE / "v26c_j14_prereg_dagger_dataset_rev1.json"
PIN_PREREG = "877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5"
SUPERSEDED_PREREG = HERE / "v26c_j14_prereg_dagger_dataset.json"
PIN_SUPERSEDED_PREREG = "4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717"

# ------------------------------------------------------------------ sources ---------------------
J12_LEAF = HERE / "j12_runs" / "j12_closed_loop_v26c_2026-08-27_r1"
J1_LEAF = HERE / "j1_runs" / "j1_nominal_v26c_2026-08-26_r1"
J7_LEAF = HERE / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1"
J10R1_LEAF = HERE / "j10r1_runs" / "j10r1_multistart_teacher_v26c_2026-08-27_r1"
# The J11 leaf. coverage_report READS its aggregate to establish the training envelope, so it is
# a genuine input and is pinned and validated exactly like the others. The rev0 bundle omitted
# it: the file was read but never pinned. That gap is what the Codex audit refused.
J11_LEAF = HERE / "j11_runs" / "j11_multistart_fit_v26c_2026-08-27_r1"
J11_AGGREGATE = J11_LEAF / "v26c_j11_aggregate_dataset.npz"
J2_MODULE = HERE / "j2_runs" / "j2_base_v26c_2026-08-26_r1" / "rl_module"

PIN_SOURCES: dict[str, str] = {
    "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/v26c_j12_closed_loop_receipt.json":
        "2b3e20362bb54ea545db3495d2a468d33138555cccf8bd20d3e5ecc3d75caf11",
    "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/commit_verification.json":
        "a5ef34e758c50437d28dfbf4413e5a18a2e3ad7ba9d6ec4fb164431b17fcb175",
    "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/j12_cell_A_trace.json":
        "5e1b91999c2fc764a233b215e204ded00608894cc36a6dac8e6129b09d306e30",
    "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/j12_cell_E_trace.json":
        "69f031c1ca69e0a4060d52e2d6c9d107987c91c4e63bb4a021e42b81398b3995",
    "j12_runs/j12_closed_loop_v26c_2026-08-27_r1/j12_cell_F_trace.json":
        "86f16eba23c82cc94404199bea5c482a590f3e4bc96238e4d405906a6afdeb3e",
    "j1_runs/j1_nominal_v26c_2026-08-26_r1/teacher_dataset.npz":
        "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41",
    "j1_runs/j1_nominal_v26c_2026-08-26_r1/v26c_j1_collection_receipt.json":
        "f54028d58dc9bfde01ede3c2a72f7ea63b67aeead02979291b03cd468bf37cdd",
    "j7_runs/j7_markov_dataset_v26c_2026-08-26_r1/v26c_j7_markov_recovery_dataset.npz":
        "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27",
    "j7_runs/j7_markov_dataset_v26c_2026-08-26_r1/v26c_j7_markov_dataset_receipt.json":
        "39cbed8072f0126b84ba40a12d1268991a128e5af3794d6fb74b246c2d509ca2",
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/j10r1_cell_B_teacher_dataset.npz":
        "2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62",
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/j10r1_cell_C_teacher_dataset.npz":
        "bd78e6ac13ab96d128f57ea5b36d058f5d80c18cfb056bcc76d2179bf1d756f0",
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/"
    "v26c_j10r1_multistart_teacher_receipt.json":
        "9c1c26f6e1aaaa96f2c92cb45f6494e889b3124dd6ca99427a11026a8d099f20",
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/commit_verification.json":
        "59cd1563ad23713ef00ac5b2e95a334a088257af95c6727520fb895c61a8c61c",
    # the J11 leaf: its aggregate is READ by coverage_report, so all three are inputs
    "j11_runs/j11_multistart_fit_v26c_2026-08-27_r1/v26c_j11_aggregate_dataset.npz":
        "a936c580c4db19383255d3d5f7346560e0fd99dc4b139ac202858e82cca13f42",
    "j11_runs/j11_multistart_fit_v26c_2026-08-27_r1/v26c_j11_multistart_fit_receipt.json":
        "39228c5cf00a753f1d57f07d4794ac2996401e1b40587cf1ec1e5f5e2b0ae65f",
    "j11_runs/j11_multistart_fit_v26c_2026-08-27_r1/commit_verification.json":
        "1d24eed05dd04c187c014181c89a0611a648e5601cacff5e1ae0221adcfb8643",
    "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/module_state.pkl":
        "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130",
    "j2_runs/j2_base_v26c_2026-08-26_r1/rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "v26c_j1_collect.py":
        "83862a8684d1970fc7f205be43a71040ec0a655f545e7c4f4607ac8f38c6691d",
    "v26c_j7_markov_dataset.py":
        "b3cc2b88997ccb7c23964123d70ac3fb524adf4fdf3e8882d26ce20ff87d1461",
    "v26c_j12_closed_loop.py":
        "65d7b8619074c909753750f03aec751d38179c2226e50c87da62113ea5769fca",
}

# ------------------------------------------------------------------ composition ------------------
ACTOR_WIDTH = 35
ACTION_DIM = 2
CLOCK_COLUMNS = (0, 1)
OBS_KEY = "actor_observation_vector_before"

# The two FAILED J12 cells, and only those. The reference for the mismatch flag is cell A: the
# deterministic nominal rollout of the SAME actor, index-comparable because it shares the offset.
REFERENCE_CELL = "A"
SOURCE_CELLS: tuple[dict[str, Any], ...] = (
    {"cell": "E", "seed": 124, "rows": 500, "mode": "stochastic_held",
     "offset_s": 1.956870983805102, "j12_verdict": "FAIL"},
    {"cell": "F", "seed": 125, "rows": 354, "mode": "stochastic_held",
     "offset_s": 1.956870983805102, "j12_verdict": "FAIL"},
)
EXPECTED_ROWS = {"E": 500, "F": 354}
TOTAL_NEW_ROWS = 854
NEW_ROW_REPEAT = 1

# Seeds that may be read here. 126, 127 and 128 are the genuine held-out reserve and are NEVER
# opened by this stage; no artefact it touches references them.
ALLOWED_SEEDS = (124, 125)
SEALED_SEEDS = (126, 127, 128)

# The future aggregate, verified component by component from the pinned files. This stage does not
# build it; it proves the arithmetic so the future fit cannot drift.
FUTURE_J7_ROWS = 16713
FUTURE_CELL_UNIQUE = 500
FUTURE_CELL_REPEAT = 8
FUTURE_TOTAL_ROWS = 25567

RELATIVE_LEAF_PARTS = ("j14_runs", "j14_dagger_dataset_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
DATASET_NAME = "v26c_j14_dagger_increment.npz"
RECEIPT_NAME = "v26c_j14_dagger_dataset_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
TECHNICAL_INVALID_NAME = "TECHNICAL_INVALID"

PREFLIGHT_SENTINEL = HERE / "_j14_preflight_sentinel_never_created"
OUTPUT_ROOT_OVERRIDE: Path | None = None

FORBIDDEN_HERE = (
    "a rollout", "a fit", "an optimizer step", "a weight update", "a critic", "PPO",
    "environment construction", "importing torch", "promotion", "deployability",
    "truncating the corrective rows", "filtering by the mismatch flag",
    "reading seeds 126, 127 or 128", "a second parent", "widening",
    "contralateral features", "a standalone 25D actor", "dedup", "class balancing",
    "a repeat factor on the new rows", "an autonomous retry", "an invented threshold",
    "changing FSM v3, the morphology corridor, the reward, sigma, the SEA or the C++ plugin",
)


# ================================================================ helpers ========================

def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    """allow_nan=False deliberately: a NaN must raise, not hash to a silent 'NaN' token."""
    return hashlib.sha256(
        json.dumps(o, sort_keys=True, allow_nan=False, default=str).encode("utf-8")).hexdigest()


def _sha_array(a: np.ndarray) -> str:
    """v26c_j7_markov_dataset._sha_array, transcribed: dtype, shape, then C-order bytes."""
    arr = np.ascontiguousarray(a)
    h = hashlib.sha256()
    h.update(str(arr.dtype).encode())
    h.update(str(arr.shape).encode())
    h.update(arr.tobytes())
    return h.hexdigest()


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


def _leaf_rel(q: Path, root: Path) -> str:
    """LEAF-RELATIVE, so the string stays valid after the staging is renamed onto the leaf."""
    try:
        rel = Path(q).relative_to(root)
    except ValueError:
        raise J14Error(f"{q} is not under the staging root {root}") from None
    if ".." in rel.parts:
        raise J14Error(f"the relative path for {q} under {root} escapes the leaf: {rel}")
    return str(rel).replace(os.sep, "/")


def _resolve_inside(leaf: Path, rel: str) -> Path:
    if not rel or Path(rel).is_absolute() or ".." in Path(rel).parts:
        raise J14Error(f"recorded path is not leaf-relative: {rel!r}")
    q = leaf / rel
    if q.is_symlink():
        raise J14Error(f"recorded path is a symlink, which is never committed here: {rel}")
    try:
        q.resolve().relative_to(leaf.resolve())
    except ValueError:
        raise J14Error(f"recorded path escapes the committed leaf: {rel}") from None
    return q


# ================================================================ the mismatch operator ==========

def discrete_feature_indices(names: Sequence[str]) -> np.ndarray:
    """target_domain_noise_adaptation._discrete_feature_indices, transcribed via J7."""
    return np.asarray(
        [i for i, n in enumerate(names)
         if str(n).endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
         or str(n).startswith(("phase_fsm_", "phase_expected_"))], dtype=int)


def first_discrete_mismatch(reference: np.ndarray, candidate: np.ndarray,
                            names: Sequence[str]) -> dict[str, Any]:
    """July's comparison, used ONLY to LABEL rows - never to drop them.

    J7 called the same operator and then KEPT ONLY the prefix. That truncation is exactly what
    left the corpus without post-mismatch coverage. Here the identical comparison runs, the first
    mismatching step is recorded, and EVERY row is kept on both sides of it.
    """
    disc = discrete_feature_indices(names)
    if disc.size == 0:
        raise J14Error("no discrete columns were resolved; the operator would be vacuous")
    limit = min(len(reference), len(candidate))
    first = None
    columns: list[str] = []
    for i in range(limit):
        a = np.asarray(reference[i], dtype=np.float64)
        b = np.asarray(candidate[i], dtype=np.float64)
        if np.any(a[disc] != b[disc]):
            first = i + 1
            columns = [str(names[c]) for c in disc if a[c] != b[c]]
            break
    return {"first_discrete_mismatch_step": first,
            "mismatching_columns": columns,
            "compared_steps": limit,
            "discrete_columns": [int(c) for c in disc],
            "discrete_feature_names": [str(names[c]) for c in disc],
            "reference_cell": REFERENCE_CELL,
            "truncation_applied": False,
            "why_not": ("the flag is a DIAGNOSTIC. Truncating here would recreate the very "
                        "coverage gap this stage exists to close.")}


# ================================================================ inputs =========================

def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J14Error("the J14 preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J14Error(f"the J14 preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage_proposed") != STAGE:
        raise J14Error(f"the preregistration proposes {data.get('stage_proposed')!r}, not {STAGE}")
    # the superseded rev0 record must still be here, byte-identical, and must NOT be the one in use
    if data.get("revision") != "rev1":
        raise J14Error(f"the preregistration declares revision {data.get('revision')!r}, not rev1")
    if not SUPERSEDED_PREREG.is_file():
        raise J14Error("the superseded rev0 preregistration is missing; it is immutable evidence "
                       "of what was proposed and refused")
    got_old = _sha_file(SUPERSEDED_PREREG)
    if got_old != PIN_SUPERSEDED_PREREG:
        raise J14Error(f"the superseded rev0 preregistration was MODIFIED: {got_old} != "
                       f"{PIN_SUPERSEDED_PREREG}. It is preserved, never edited.")
    if data["supersedes"]["sha256"] != PIN_SUPERSEDED_PREREG:
        raise J14Error("rev1 does not record the correct hash of the record it supersedes")
    if PREREG.resolve() == SUPERSEDED_PREREG.resolve():
        raise J14Error("the runner is pointing at the superseded record")

    declared = data["source_cells"]
    if [c["cell"] for c in declared] != [c["cell"] for c in SOURCE_CELLS]:
        raise J14Error("the preregistered cell order and the runner's disagree")
    for want, got in zip(SOURCE_CELLS, declared):
        if (str(got["cell"]), int(got["seed"]), int(got["rows"])) != \
                (want["cell"], want["seed"], want["rows"]):
            raise J14Error(f"cell {want['cell']} differs from the preregistration")
    if int(data["totals"]["new_rows"]) != TOTAL_NEW_ROWS:
        raise J14Error(f"the preregistration declares {data['totals']['new_rows']} new rows, "
                       f"not {TOTAL_NEW_ROWS}")
    if int(data["totals"]["future_aggregate_rows"]) != FUTURE_TOTAL_ROWS:
        raise J14Error("the preregistration and the runner disagree on the future aggregate")
    if int(data["new_row_repeat"]) != NEW_ROW_REPEAT:
        raise J14Error("the new rows carry repeat 1; no other value is preregistered")
    if data["truncation"]["applied"] is not False:
        raise J14Error("the preregistration must declare that NO truncation is applied")
    if data["mismatch_flag"]["binding"] is not False:
        raise J14Error("the mismatch flag is a DIAGNOSTIC and must be declared non-binding")
    if sorted(int(s) for s in data["seeds"]["sealed"]) != sorted(SEALED_SEEDS):
        raise J14Error("the preregistration must seal seeds 126, 127 and 128")
    if data["seeds"]["seed_125_is_held_out"] is not False:
        raise J14Error("seed 125 is already inside J7 and must NOT be declared held out")

    pinned: dict[str, str] = {}
    for rel, pin in data["pinned_sources_sha256"].items():
        got = _sha_file(HERE / rel)
        if got != pin:
            raise J14Error(f"the pinned source {rel} changed: {got} != {pin}")
        pinned[rel] = got
    if pinned != PIN_SOURCES:
        raise J14Error("the preregistered pin table and the runner's disagree")
    return {"file": _rel(PREREG), "sha256": digest, "pinned_sources_sha256": pinned,
            "entries": len(pinned)}


def verify_sources() -> dict[str, Any]:
    """Every source pinned by hash, and every source LEAF proved still valid evidence."""
    checked: dict[str, str] = {}
    for rel, pin in PIN_SOURCES.items():
        q = HERE / rel
        if not q.is_file():
            raise J14Error(f"the pinned source is missing: {rel}")
        got = _sha_file(q)
        if got != pin:
            raise J14Error(f"the pinned source {rel} changed: {got} != {pin}")
        checked[rel] = got

    leaves = {}
    for name, leaf, receipt_name, verdict_key in (
            ("j12", J12_LEAF, "v26c_j12_closed_loop_receipt.json", "verdict"),
            ("j10r1", J10R1_LEAF, "v26c_j10r1_multistart_teacher_receipt.json", "verdict"),
            ("j11", J11_LEAF, "v26c_j11_multistart_fit_receipt.json", "verdict")):
        marker = leaf / TECHNICAL_INVALID_NAME
        if marker.exists():
            raise J14Error(f"the {name} leaf is marked {TECHNICAL_INVALID_NAME}; it is not valid "
                           f"evidence and nothing may be read from it")
        cv_path = leaf / COMMIT_VERIFICATION_NAME
        if not cv_path.is_file():
            raise J14Error(f"the {name} leaf carries no {COMMIT_VERIFICATION_NAME}")
        cv = json.loads(cv_path.read_text())
        if cv.get("pass") is not True:
            raise J14Error(f"the {name} commit verification does not pass: {cv.get('pass')!r}")
        for pattern in (".lock_*", ".staging_*"):
            stale = sorted(q.name for q in leaf.parent.glob(pattern))
            if stale:
                raise J14Error(f"the {name} run directory still holds {stale}; its commit did "
                               f"not complete cleanly")
        receipt = json.loads((leaf / receipt_name).read_text())
        leaves[name] = {"leaf": _rel(leaf), "commit_verification_pass": True,
                        "technical_invalid_marker": False, "no_stale_lock_or_staging": True,
                        "verdict": receipt.get(verdict_key)}

    # the J12 verdict is FAIL, and that is WHY this stage exists - it is not a defect to hide
    if leaves["j12"]["verdict"] != "FAIL":
        raise J14Error(f"the J12 receipt verdict is {leaves['j12']['verdict']!r}; this stage is "
                       f"built on the FAILED cells of a FAILED matrix and refuses to proceed if "
                       f"that is not what the evidence says")
    if leaves["j10r1"]["verdict"] != "PASS":
        raise J14Error("the J10R1 receipt verdict is not PASS")
    # The J11 leaf supplies the training envelope that coverage_report measures against. If its
    # own fit had not passed, the envelope would not be the envelope of a qualified actor.
    if leaves["j11"]["verdict"] != "PASS":
        raise J14Error(f"the J11 receipt verdict is {leaves['j11']['verdict']!r}, not PASS; its "
                       f"aggregate may not be used as the training envelope")

    # no artefact this stage reads may reference a sealed seed
    for rel in PIN_SOURCES:
        for sealed in SEALED_SEEDS:
            if f"seed{sealed}" in rel or f"seed_{sealed}" in rel:
                raise J14Error(f"the pinned source {rel} references sealed seed {sealed}")
    return {"pinned_sources_sha256": checked, "source_leaves": leaves,
            "sealed_seeds_never_read": list(SEALED_SEEDS)}


def actor_feature_names() -> tuple[str, ...]:
    manifest = json.loads((J2_MODULE / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J14Error(f"the manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


def load_teacher() -> dict[str, Any]:
    """J1's committed teacher dataset. Its ACTIONS are the labels; its observations are never read."""
    path = J1_LEAF / "teacher_dataset.npz"
    with np.load(path, allow_pickle=False) as z:
        actions = np.asarray(z["actions"])
        times = np.asarray(z["times"])
        names = tuple(str(n) for n in np.asarray(z["actor_feature_names"]).tolist())
    if actions.shape != (500, ACTION_DIM) or actions.dtype != np.float32:
        raise J14Error(f"the J1 actions are {actions.shape}/{actions.dtype}, expected "
                       f"{(500, ACTION_DIM)}/float32")
    if times.shape != (500,):
        raise J14Error(f"the J1 times are {times.shape}, expected (500,)")
    return {"actions": actions, "times": times, "names": names, "sha256": _sha_file(path),
            "observations_read": False}


def load_cell_rows(cell: str) -> dict[str, Any]:
    """The 35-wide actor observations and step/time columns of ONE J12 trace. Streamed lean."""
    path = J12_LEAF / f"j12_cell_{cell}_trace.json"
    rows = json.loads(path.read_text())
    obs = np.asarray([r[OBS_KEY] for r in rows], dtype=np.float32)
    steps = np.asarray([int(r["step"]) for r in rows], dtype=np.int64)
    times = np.asarray([float(r["time_before"]) for r in rows], dtype=np.float64)
    cells = sorted({str(r["cell"]) for r in rows})
    del rows
    if cells != [cell]:
        raise J14Error(f"the trace for cell {cell} reports cells {cells}")
    if obs.shape[1] != ACTOR_WIDTH:
        raise J14Error(f"cell {cell}: the observation is {obs.shape[1]}D, expected {ACTOR_WIDTH}")
    if not np.array_equal(steps, np.arange(1, len(steps) + 1, dtype=np.int64)):
        raise J14Error(f"cell {cell}: the step column is not 1..{len(steps)} contiguous")
    if not np.all(np.isfinite(obs)):
        raise J14Error(f"cell {cell}: the observations hold non-finite values")
    return {"observations": obs, "steps": steps, "times": times, "sha256": _sha_file(path)}


def build_increment() -> dict[str, Any]:
    """The 854 corrective rows. Every row kept; the mismatch flag is recorded, never applied."""
    # The clock contract, asserted before it is relied on. An empty or wrong CLOCK_COLUMNS would
    # make the projection a silent no-op and the zero-check a reduction over an empty array.
    if tuple(CLOCK_COLUMNS) != (0, 1):
        raise J14Error(f"CLOCK_COLUMNS is {tuple(CLOCK_COLUMNS)}, expected exactly (0, 1). The "
                       f"clock projection is only meaningful against the declared contract.")
    names = actor_feature_names()
    teacher = load_teacher()
    if teacher["names"] != names:
        raise J14Error("the J1 teacher dataset and the J2 manifest disagree on the feature names")

    reference = load_cell_rows(REFERENCE_CELL)
    if reference["observations"].shape[0] != 500:
        raise J14Error(f"the reference cell {REFERENCE_CELL} has "
                       f"{reference['observations'].shape[0]} rows, expected 500")

    obs_parts, act_parts = [], []
    cell_col, seed_col, step_col, post_col, time_col = [], [], [], [], []
    per_cell: list[dict[str, Any]] = []

    for spec in SOURCE_CELLS:
        cid, seed, want = spec["cell"], int(spec["seed"]), int(spec["rows"])
        if seed not in ALLOWED_SEEDS:
            raise J14Error(f"cell {cid} declares seed {seed}, which is not permitted here")
        if seed in SEALED_SEEDS:
            raise J14Error(f"cell {cid} declares SEALED seed {seed}")
        data = load_cell_rows(cid)
        raw = data["observations"]
        n = raw.shape[0]
        if n != want or n != EXPECTED_ROWS[cid]:
            raise J14Error(f"cell {cid} holds {n} rows, expected exactly {want}")

        # the time grid must BE the teacher's, row for row - this is what makes same-step legal
        if not np.array_equal(data["times"], teacher["times"][:n]):
            bad = int(np.argmax(data["times"] != teacher["times"][:n]))
            raise J14Error(f"cell {cid}: time_before is not bit-identical to the J1 grid; first "
                           f"divergence at row {bad} "
                           f"({data['times'][bad]!r} != {teacher['times'][:n][bad]!r})")
        labels = np.array(teacher["actions"][:n], dtype=np.float32, copy=True)

        # the clock, projected to EXACT zero, touching nothing else
        projected = raw.copy()
        projected[:, list(CLOCK_COLUMNS)] = 0.0
        if float(np.max(np.abs(projected[:, list(CLOCK_COLUMNS)]))) != 0.0:
            raise J14Error(f"cell {cid}: the clock projection did not reach exact zero")
        non_clock = [c for c in range(ACTOR_WIDTH) if c not in CLOCK_COLUMNS]
        if not np.array_equal(projected[:, non_clock], raw[:, non_clock]):
            raise J14Error(f"cell {cid}: the clock projection disturbed a non-clock column")

        mism = first_discrete_mismatch(reference["observations"][:n], raw, names)
        first = mism["first_discrete_mismatch_step"]
        post = (data["steps"] >= first) if first is not None else np.zeros(n, dtype=bool)
        post = np.asarray(post, dtype=bool)

        obs_parts.append(projected)
        act_parts.append(labels)
        cell_col.append(np.full(n, cid, dtype="<U1"))
        seed_col.append(np.full(n, seed, dtype=np.int64))
        step_col.append(data["steps"])
        post_col.append(post)
        time_col.append(data["times"])

        per_cell.append({
            "cell": cid, "seed": seed, "mode": spec["mode"],
            "episode_start_offset_s": float(spec["offset_s"]),
            "j12_verdict": spec["j12_verdict"], "rows": n, "repeat": NEW_ROW_REPEAT,
            "trace_sha256": data["sha256"],
            "label_source": "j1_runs/.../teacher_dataset.npz actions[step - 1], SAME STEP",
            "time_grid_bit_identical_to_j1": True,
            "mismatch": mism,
            "rows_pre_mismatch": int((~post).sum()),
            "rows_post_mismatch": int(post.sum()),
            "post_mismatch_fraction": float(post.mean()),
            "content_hashes": {
                "observations_raw": _sha_array(raw),
                "observations_projected": _sha_array(projected),
                "actions": _sha_array(labels)},
            "raw_clock": {"col_0_unique": sorted({float(v) for v in raw[:, 0]}),
                          "col_1_unique": sorted({float(v) for v in raw[:, 1]})},
            "distinct_rows": int(len(np.unique(projected, axis=0))),
        })

    obs = np.ascontiguousarray(np.concatenate(obs_parts, axis=0).astype(np.float32))
    act = np.ascontiguousarray(np.concatenate(act_parts, axis=0).astype(np.float32))
    cell_arr = np.concatenate(cell_col)
    seed_arr = np.concatenate(seed_col)
    step_arr = np.concatenate(step_col)
    post_arr = np.concatenate(post_col)
    time_arr = np.concatenate(time_col)

    if obs.shape != (TOTAL_NEW_ROWS, ACTOR_WIDTH) or act.shape != (TOTAL_NEW_ROWS, ACTION_DIM):
        raise J14Error(f"the increment is {obs.shape}/{act.shape}, expected "
                       f"{(TOTAL_NEW_ROWS, ACTOR_WIDTH)}/{(TOTAL_NEW_ROWS, ACTION_DIM)}")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J14Error("the increment holds non-finite values")
    if float(np.max(np.abs(obs[:, list(CLOCK_COLUMNS)]))) != 0.0:
        raise J14Error("the increment's clock columns are not exactly zero")
    if sorted(set(int(s) for s in seed_arr)) != sorted(ALLOWED_SEEDS):
        raise J14Error(f"the increment carries seeds {sorted(set(int(s) for s in seed_arr))}")
    if any(int(s) in SEALED_SEEDS for s in seed_arr):
        raise J14Error("a SEALED seed reached the increment")

    return {
        "observations": obs, "actions": act, "names": names,
        "cell": cell_arr, "seed": seed_arr, "step": step_arr,
        "post_mismatch": post_arr, "time_before": time_arr,
        "rows": int(obs.shape[0]), "per_cell": per_cell,
        "reference_cell": {"cell": REFERENCE_CELL, "rows": 500,
                           "sha256": reference["sha256"],
                           "why": "the deterministic nominal rollout of the SAME actor, "
                                  "index-comparable because it shares the episode start"},
        "teacher": {"path": _rel(J1_LEAF / "teacher_dataset.npz"),
                    "sha256": teacher["sha256"], "observations_read": False,
                    "mapping": "actions[step - 1], the SAME instant, no lead and no lag"},
        "content_hashes": {"observations": _sha_array(obs), "actions": _sha_array(act),
                           "cell": _sha_array(cell_arr), "seed": _sha_array(seed_arr),
                           "step": _sha_array(step_arr),
                           "post_mismatch": _sha_array(post_arr),
                           "time_before": _sha_array(time_arr)},
        "truncation": {"applied": False,
                       "julys_dagger_did_not_truncate": True,
                       "j7_did_truncate": "429/273/11 of 500, which is why the corpus has no "
                                          "post-mismatch coverage",
                       "julys_binding_todo_2026_07_14":
                           "includendo la regione successiva al mismatch FSM invece di sole "
                           "interpolazioni pre-evento"},
        "no_dedup": True, "no_balancing": True, "repeat": NEW_ROW_REPEAT,
        "what_the_flag_does_NOT_mean": {
            "post_mismatch_is_not_a_quality_label": True,
            "counterexample": ("in the same J12 matrix, cell D (seed 123, nominal) diverges at "
                              "step 12 - EARLIER than E at 95 or F at 90 - and PASSES the "
                              "behavioural gate. The step at which the discrete comparison first "
                              "differs carries no failure signal by itself."),
            "what_it_is": "a provenance marker: which side of the J7 truncation point a row "
                          "would have fallen on. Nothing downstream may filter or weight by it.",
            "measured_here": "the flag is recorded per row and used by nothing in this module"},
        "honest_accounting_of_the_854_rows": {
            "pre_mismatch_rows": int(sum(c["rows_pre_mismatch"] for c in per_cell)),
            "pre_mismatch_are_near_nominal": ("the pre-mismatch halves sit entirely inside the "
                                              "J11 training envelope and add essentially no new "
                                              "coverage. They are kept because truncation is what "
                                              "this stage exists to avoid, not because they "
                                              "carry information."),
            "post_mismatch_rows": int(sum(c["rows_post_mismatch"] for c in per_cell)),
            "where_the_value_is": "the post-mismatch rows, and above all those beyond the J11 "
                                  "envelope",
            "do_not_justify_this_stage_on_854": True},
    }


def coverage_report(obs: np.ndarray, post: np.ndarray, cell: np.ndarray,
                    names: Sequence[str]) -> dict[str, Any]:
    """How far outside the J11 training envelope these rows reach. DIAGNOSTIC, no threshold.

    The duplicate count is deliberately measured on the PROJECTED rows. On RAW rows it would
    always read zero for the wrong reason: every J11 row carries gait_phase_cos = 0 and every
    raw J12 row carries 1, so nothing could ever collide.
    """
    # read through the PINNED constant, and re-hash at read time: this file is an input, and the
    # rev0 bundle read it without pinning it at all
    rel = _leaf_rel(J11_AGGREGATE, HERE)
    got = _sha_file(J11_AGGREGATE)
    if got != PIN_SOURCES[rel]:
        raise J14Error(f"the J11 aggregate changed: {got} != {PIN_SOURCES[rel]}")
    with np.load(J11_AGGREGATE, allow_pickle=False) as z:
        train = np.asarray(z["observations"])
    i_swing = list(names).index("phase_swing_elapsed_norm")
    i_knee = list(names).index("pros_knee_angle")
    swing_max = float(train[:, i_swing].max())
    knee_min = float(train[:, i_knee].min())

    train_rows = {r.tobytes() for r in np.ascontiguousarray(train)}
    dup_vs_train = int(sum(1 for r in np.ascontiguousarray(obs) if r.tobytes() in train_rows))
    _, counts = np.unique(obs, axis=0, return_counts=True)
    dup_within = int(obs.shape[0] - len(counts))

    per_cell: dict[str, Any] = {}
    for cid in sorted(set(str(c) for c in cell)):
        m = cell == cid
        for half, sel in (("pre", m & ~post), ("post", m & post)):
            sub = obs[sel]
            if sub.size == 0:
                per_cell[f"{cid}_{half}"] = {"rows": 0}
                continue
            beyond_swing = int((sub[:, i_swing] > swing_max).sum())
            beyond_knee = int((sub[:, i_knee] < knee_min).sum())
            per_cell[f"{cid}_{half}"] = {
                "rows": int(sub.shape[0]),
                "swing_elapsed_norm_range": [float(sub[:, i_swing].min()),
                                             float(sub[:, i_swing].max())],
                "knee_range": [float(sub[:, i_knee].min()), float(sub[:, i_knee].max())],
                "rows_beyond_training_swing_max": beyond_swing,
                "rows_beyond_training_knee_min": beyond_knee,
                "rows_beyond_envelope_either": int(((sub[:, i_swing] > swing_max)
                                                    | (sub[:, i_knee] < knee_min)).sum())}
    return {
        "training_envelope": {"source": "the J11 aggregate, 24713 rows",
                              "phase_swing_elapsed_norm_max": swing_max,
                              "pros_knee_angle_min": knee_min,
                              "distinct_training_rows": int(len(counts)) and
                                                        int(len(np.unique(train, axis=0)))},
        "per_cell_half": per_cell,
        "duplicates": {"within_the_increment": dup_within,
                       "already_in_the_j11_aggregate": dup_vs_train,
                       "measured_on": "the PROJECTED rows",
                       "why_not_raw": ("on raw rows the count is always zero for the wrong "
                                       "reason: J11 stores the clock as 0, the raw traces as "
                                       "(0, 1), so no raw row can ever match"),
                       "not_removed": "this stage does not deduplicate"},
        "mixing_weight": {
            "new_rows": int(obs.shape[0]),
            "against_tiled_aggregate": f"{obs.shape[0]} / {FUTURE_TOTAL_ROWS}",
            "against_distinct_rows": ("the J11 aggregate holds far fewer DISTINCT rows than its "
                                      "row count, because it is built by np.tile; the effective "
                                      "share of the new rows is correspondingly larger"),
            "repeat_here": NEW_ROW_REPEAT,
            "no_repeat_is_inherited": ("the factor 8 used elsewhere has no documented rationale, "
                                       "so no repeat can be justified as inherited. Any future "
                                       "repeat would be a NEW number needing its own "
                                       "preregistration.")},
        "binding": False,
        "thresholds_here": 0,
    }


def verify_future_aggregate() -> dict[str, Any]:
    """Prove the future aggregate arithmetic from the FILES. Fails closed if it does not add up."""
    with np.load(J7_LEAF / "v26c_j7_markov_recovery_dataset.npz", allow_pickle=False) as z:
        j7_rows = int(np.asarray(z["observations"]).shape[0])
    cells = {}
    for cid in ("B", "C"):
        with np.load(J10R1_LEAF / f"j10r1_cell_{cid}_teacher_dataset.npz",
                     allow_pickle=False) as z:
            cells[cid] = int(np.asarray(z["observations"]).shape[0])
    if j7_rows != FUTURE_J7_ROWS:
        raise J14Error(f"J7 holds {j7_rows} rows, expected {FUTURE_J7_ROWS}")
    for cid, n in cells.items():
        if n != FUTURE_CELL_UNIQUE:
            raise J14Error(f"J10R1 cell {cid} holds {n} rows, expected {FUTURE_CELL_UNIQUE}")
    total = j7_rows + FUTURE_CELL_REPEAT * (cells["B"] + cells["C"]) + TOTAL_NEW_ROWS
    if total != FUTURE_TOTAL_ROWS:
        raise J14Error(f"the future aggregate computes to {total}, not {FUTURE_TOTAL_ROWS}")
    return {
        "j7_rows": j7_rows,
        "cell_B_unique": cells["B"], "cell_C_unique": cells["C"],
        "cell_repeat": FUTURE_CELL_REPEAT,
        "cell_rows_total": FUTURE_CELL_REPEAT * (cells["B"] + cells["C"]),
        "new_rows": TOTAL_NEW_ROWS, "new_row_repeat": NEW_ROW_REPEAT,
        "total": total, "expected_total": FUTURE_TOTAL_ROWS, "verified_from_files": True,
        "arithmetic": f"{j7_rows} + {FUTURE_CELL_REPEAT}x({cells['B']}+{cells['C']}) "
                      f"+ {TOTAL_NEW_ROWS} = {total}",
        "this_stage_does_not_build_it": ("J14 materialises the increment only. The future fit "
                                         "assembles the aggregate and must re-verify these "
                                         "counts against the pinned files."),
    }


# ================================================================ destination ====================

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J14Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J14Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J14Error("--materialize requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J14Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J14Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J14Error(f"the authorised leaf already exists; this stage is no-clobber and "
                       f"single-execution: {leaf}")
    return leaf


# ================================================================ post-commit verification =======

def verify_committed_leaf(leaf: Path, *, expected_receipt_sha: str | None = None) -> dict[str, Any]:
    """Re-resolve every recorded leaf-relative path in the COMMITTED leaf and re-hash it."""
    receipt = leaf / RECEIPT_NAME
    receipt_bytes = receipt.read_bytes() if receipt.is_file() else None
    receipt_sha = hashlib.sha256(receipt_bytes).hexdigest() if receipt_bytes is not None else None
    matches_staging: bool | None = None
    if expected_receipt_sha is not None:
        matches_staging = receipt_sha == expected_receipt_sha
    if receipt_bytes is None:
        return {"schema": "v26c_j14_commit_verification.1", "stage": STAGE, "pass": False,
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
    if matches_staging is False:
        mismatches.append({"path": RECEIPT_NAME, "expected": str(expected_receipt_sha),
                           "recomputed": str(receipt_sha)})
    ok = not missing and not mismatches
    return {
        "schema": "v26c_j14_commit_verification.1", "stage": STAGE,
        "when": "AFTER os.rename, against the COMMITTED leaf",
        "verified_against": "the committed receipt",
        "pass": ok, "files_checked": len(recomputed), "files_recorded": len(recorded),
        "paths_missing": missing, "hash_mismatches": mismatches,
        "recomputed_sha256": recomputed, "receipt_sha256": receipt_sha,
        "receipt_matches_staging_bytes": matches_staging,
        "meaning": ("the leaf is VALID EVIDENCE if and only if this file exists and pass is true. "
                    "Its absence, or pass false, marks the leaf TECHNICALLY INVALID."),
        "on_failure": "preserved fail-closed and NOT promoted. No deletion, no retry, no repair.",
    }


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    """INERT. No torch, no environment, no rollout, no fit, no write."""
    heavy_before = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                      "gymnasium") if m in sys.modules)
    sentinel_before = PREFLIGHT_SENTINEL.exists()

    prereg = verify_prereg()
    sources = verify_sources()
    data = build_increment()
    coverage = coverage_report(data["observations"], data["post_mismatch"],
                               data["cell"], data["names"])
    future = verify_future_aggregate()

    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J14 lock is already held or was left behind: {lock_path}")

    heavy_after = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                     "gymnasium") if m in sys.modules)
    introduced = sorted(set(heavy_after) - set(heavy_before))
    if introduced:
        blockers.append(f"the preflight introduced heavy modules: {introduced}")
    if PREFLIGHT_SENTINEL.exists() != sentinel_before or PREFLIGHT_SENTINEL.exists():
        blockers.append("the preflight created its sentinel path")

    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"rollout": False, "fit_executed": False, "optimizer_steps": 0,
                  "critic_touched": False, "ppo_updates": 0,
                  "environment_constructed": False, "torch_imported": False,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False,
                  "heavy_modules_before": heavy_before, "heavy_modules_after": heavy_after,
                  "heavy_modules_introduced_by_preflight": introduced,
                  "strongest_claim": ("this stage never needs torch, ray, OpenSim or an "
                                      "environment at any point - not even in the materialise "
                                      "path. It reads committed bytes and writes committed "
                                      "bytes.")},
        "preregistration": prereg,
        "sources": sources,
        "increment": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names", "cell", "seed", "step",
                                   "post_mismatch", "time_before")},
        "coverage": coverage,
        "scientific_assumption": {
            "what": ("the label is an OPEN-LOOP, TIME-INDEXED reference. For the post-mismatch "
                     "rows it pairs a badly drifted state with the action the teacher would have "
                     "taken at that instant on the nominal trajectory."),
            "why_it_is_defensible": ("this is the frozen J7 convention and it is DAgger-standard: "
                                     "the teacher says 'come back to the reference'. In a delayed "
                                     "swing that pushes the foot toward contact."),
            "why_it_is_still_an_assumption": ("the prescribed teacher is not a state-feedback "
                                              "expert. The nominal-time reference is not "
                                              "necessarily the correct recovery action from a "
                                              "state far outside the training envelope. July "
                                              "measured 76.27% phase-misaligned rows in its own "
                                              "recovery block and answered with truncation."),
            "status": "THE LARGEST SCIENTIFIC ASSUMPTION IN THIS DESIGN, stated rather than "
                      "inherited by default. It is arbitrated by the closed-loop gate, not here.",
            "no_threshold_attached": True},
        "future_aggregate": future,
        "seeds": {"read_here": list(ALLOWED_SEEDS),
                  "sealed_never_read": list(SEALED_SEEDS),
                  "seed_125_is_held_out": False,
                  "why_not": ("J7 already contains 11 rows from seed 125 at [16702:16713], "
                              "bit-identical to the J6 seed-125 probe. A seed already inside the "
                              "training corpus cannot be called held out."),
                  "genuine_reserve": "seeds 126, 127 and 128, opened by no stage so far"},
        "would_write": {"leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
                        "files": [DATASET_NAME, RECEIPT_NAME, COMMIT_VERIFICATION_NAME]},
        "requires_to_materialize": {"flag": "--materialize", "stage_token": STAGE,
                                    "out": "must equal the authorised leaf exactly"},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "fit_authorized": False, "critic_authorized": False,
                    "ppo_authorized_to_start": False},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ commit =========================

def build_receipt(pre: Mapping[str, Any], data: Mapping[str, Any], *,
                  committed: Mapping[str, str], injected: bool) -> dict[str, Any]:
    return {
        "schema": "v26c_j14_dagger_dataset_receipt.1",
        "stage": STAGE,
        "verdict": "MATERIALIZED",
        "authoritative": OUTPUT_ROOT_OVERRIDE is None,
        "preregistration": pre["preregistration"],
        "sources": pre["sources"],
        "increment": {k: v for k, v in data.items()
                      if k not in ("observations", "actions", "names", "cell", "seed", "step",
                                   "post_mismatch", "time_before")},
        "coverage": pre["coverage"],
        "scientific_assumption": pre["scientific_assumption"],
        "dataset_file": DATASET_NAME,
        "actor_feature_names": list(data["names"]),
        "arrays": {
            "observations": "(854, 35) float32, gait clock projected to EXACT zero",
            "actions": "(854, 2) float32, the J1 teacher at the SAME step",
            "cell": "(854,) <U1, 'E' or 'F'",
            "seed": "(854,) int64, 124 or 125",
            "step": "(854,) int64, 1-based within its own cell",
            "post_mismatch": "(854,) bool, DIAGNOSTIC: is this row at or after the first "
                             "discrete mismatch against cell A",
            "time_before": "(854,) float64, bit-identical to the J1 grid"},
        "future_aggregate": pre["future_aggregate"],
        "seeds": pre["seeds"],
        "lineage": {
            "operational": "August V26 imitation -> J2 35D -> the current pipeline",
            "july_role": "methodology and evidence only",
            "states_from": "the J11 student, as rolled out in the J12 qualification",
            "labels_from": "the J1 prescribed teacher, same step",
            "the_fit_that_will_use_this": "NOT part of this stage and NOT authorised by it"},
        "committed_files_sha256": dict(committed),
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
        "inert": {"rollout": False, "fit_executed": False, "optimizer_steps": 0,
                  "critic_touched": False, "ppo_updates": 0,
                  "environment_constructed": False, "torch_imported": False,
                  "sealed_seeds_read": 0},
        "outcome": {"deployable": False, "promotion": "NONE", "next_stage_authorized": False,
                    "fit_authorized": False, "critic_authorized": False,
                    "ppo_authorized_to_start": False, "single_execution": True,
                    "no_autonomous_retry": True,
                    "note": "this stage produces a dataset increment and authorises nothing"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def materialize(out_arg: str | None, stage_token: str | None, *,
                injected: bool = False) -> dict[str, Any]:
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    if OUTPUT_ROOT_OVERRIDE is not None and not injected:
        raise J14Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. An authoritative "
                       f"materialisation is only ever written to the real root.")

    pre = preflight()
    if pre["verdict"] != "GO":
        raise J14Error(f"the preflight is {pre['verdict']}: {pre['blockers']}")
    data = build_increment()

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J14Error(f"a stale staging directory is in the way: {staging}")

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
            raise J14Error(f"the J14 lock already exists: {lock_path}. This stage fails closed "
                           f"rather than remove a lock it does not own.") from None
        lock_owned = lock_path
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"stage": STAGE, "pid": os.getpid()}))

        staging.mkdir()
        staging_created = staging

        np.savez_compressed(
            staging / DATASET_NAME,
            observations=data["observations"], actions=data["actions"],
            actor_feature_names=np.asarray(list(data["names"]), dtype="<U34"),
            cell=data["cell"], seed=data["seed"], step=data["step"],
            post_mismatch=data["post_mismatch"], time_before=data["time_before"])
        # np.savez_compressed silently appends .npz to a name that lacks it. DATASET_NAME already
        # carries the suffix, but the only way to know the file landed where we think is to look.
        if not (staging / DATASET_NAME).is_file():
            raise J14Error(f"the dataset was not written at {DATASET_NAME}; np.savez may have "
                           f"renamed it. Staging holds: "
                           f"{sorted(q.name for q in staging.iterdir())}")

        committed = {}
        for q in sorted(staging.rglob("*")):
            if q.is_file() and not q.is_symlink():
                committed[_leaf_rel(q, staging)] = _sha_file(q)

        receipt = build_receipt(pre, data, committed=committed, injected=injected)
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False,
                       default=str) + "\n", encoding="utf-8")
        staging_receipt_sha = _sha_file(staging / RECEIPT_NAME)

        # THE LEAF IS BORN INVALID: written into the staging, so committed BY the rename.
        (staging / TECHNICAL_INVALID_NAME).write_text(
            "TECHNICALLY INVALID - UNVERIFIED\n"
            f"stage: {STAGE}\n"
            f"see: {COMMIT_VERIFICATION_NAME}\n"
            "This marker is written BEFORE the commit and removed only after the post-commit "
            "verification has passed. While it is present the leaf is NOT valid evidence.\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J14Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        marker = leaf / TECHNICAL_INVALID_NAME
        if not marker.is_file():
            raise J14Error(f"the committed leaf does not carry {TECHNICAL_INVALID_NAME}: "
                           f"{leaf} is not the directory this run staged")
        try:
            verification = verify_committed_leaf(leaf, expected_receipt_sha=staging_receipt_sha)
        except Exception as exc:
            verification = {"schema": "v26c_j14_commit_verification.1", "stage": STAGE,
                            "pass": False, "verifier_error": f"{type(exc).__name__}: {exc}",
                            "meaning": "verification could not complete; the leaf is TECHNICALLY "
                                       "INVALID and is preserved unpromoted"}
        try:
            (leaf / COMMIT_VERIFICATION_NAME).write_text(
                json.dumps(verification, indent=2, ensure_ascii=False, allow_nan=False,
                           default=str) + "\n", encoding="utf-8")
        except OSError as exc:
            raise J14Error(f"the verification could not be recorded in {leaf}: "
                           f"{type(exc).__name__}: {exc}. The leaf remains marked "
                           f"{TECHNICAL_INVALID_NAME}.") from exc
        if not verification.get("pass"):
            try:
                marker.write_text(
                    "TECHNICALLY INVALID - VERIFICATION FAILED\n"
                    f"stage: {STAGE}\nsee: {COMMIT_VERIFICATION_NAME}\n"
                    "The committed content did not reproduce the paths and hashes recorded in "
                    "the receipt. Preserved as evidence about the commit.\n", encoding="utf-8")
            except OSError:
                pass
            raise J14Error(
                f"POST-COMMIT VERIFICATION FAILED for {leaf}: "
                f"{len(verification.get('paths_missing') or [])} paths did not resolve, "
                f"{len(verification.get('hash_mismatches') or [])} hashes did not reproduce.")
        try:
            marker.unlink()
        except OSError as exc:
            raise J14Error(f"the verification passed but the marker could not be removed from "
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

    return {"verdict": "MATERIALIZED", "stage": STAGE, "leaf": _rel(leaf),
            "rows": data["rows"],
            "per_cell": [{"cell": c["cell"], "seed": c["seed"], "rows": c["rows"],
                          "first_discrete_mismatch_step":
                              c["mismatch"]["first_discrete_mismatch_step"],
                          "rows_post_mismatch": c["rows_post_mismatch"]}
                         for c in data["per_cell"]],
            "future_aggregate_rows": pre["future_aggregate"]["total"],
            "receipt_sha256": _sha_file(leaf / RECEIPT_NAME),
            "dataset_sha256": _sha_file(leaf / DATASET_NAME),
            "commit_verification": {"file": COMMIT_VERIFICATION_NAME, "pass": commit_verified,
                                    "files_checked": verification.get("files_checked")},
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "fit_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="V26C J14 post-mismatch DAgger dataset from committed J12 traces")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--materialize", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    a = p.parse_args(argv)
    if a.materialize:
        r = materialize(a.out, a.authorized_stage)
        print(json.dumps(r, indent=2, default=str))
        return 0
    if a.out is not None:
        raise J14Error("--out is meaningless without --materialize; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
