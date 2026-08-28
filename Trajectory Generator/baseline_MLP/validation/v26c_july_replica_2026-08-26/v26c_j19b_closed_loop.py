"""V26C J19B - CLOSED-LOOP QUALIFICATION of the J19A actor.

WHAT THIS IS
    Six declared cells, one matrix, one leaf. Three DETERMINISTIC starts (nominal, -0.20 s,
    +0.20 s) and three STOCHASTIC_HELD repeats at the nominal start with the actor's own frozen
    sigma. 500 steps per cell, run sequentially in the order A-F.

THE ACTOR IS J19A, AND ONLY J19A
    j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module.
    NOT J15R1 - J15R1 is the actor that FAILED this same matrix 0/6 in J16. NOT J11,
    NOT J8, NOT J4, NOT a July checkpoint. The J19A leaf is pinned file by file and is
    consumed only if its own commit_verification.json declares ok true with no problems
    - the J19A schema uses "ok" where J15R1's used "pass" - and it carries no invalidity
    marker: an unverified leaf is not evidence and its actor is not loadable here.

DERIVED FROM J16
    This runner is a MECHANICAL derivation of v26c_j16_closed_loop.py, produced by
    v26c_j19b_derive_from_j16.py from a closed, pinned substitution map. J16 itself is
    byte-unchanged. The scientific path is bytecode-identical: base_env_config,
    cell_env_config, expected_reset_time, unit_correction, evaluate_cell_gate,
    cell_verdict, penetration_report and production_stack are identical, and run_cell's
    instruction stream is identical with only four renamed references and two output
    strings differing. MATRIX, the gate tables, SIGMA, the timings, seeds 123-125,
    behavioural fail-fast false and the penetration contract are identical by value.

LINEAGE
    August V26 imitation -> J2 35D -> J19A -> THIS qualification. July is METHODOLOGY AND EVIDENCE
    ONLY: no July checkpoint, dataset or label is an operative input.

THE SCIENCE IS J9R1's, REUSED AND PROVEN EQUAL
    Every behavioural threshold is taken from the FROZEN v26c_j9r1_closed_loop module, which in
    turn takes them from v26c_j1_collect.J1_GATE and v26c_j3_closed_loop.J3_KINEMATIC_GATE. This
    stage INVENTS NO THRESHOLD: verify_scientific_equivalence proves, field by field against the
    pinned J9R1 source, that the matrix, the gates, sigma, the tolerances and the penetration
    bands are value-identical. What changed is the ACTOR under test, and nothing else.

PENETRATION
    > 0.020 m is a SOFT diagnostic; >= 0.025 m is the July legacy diagnostic; the SOLE binding
    bar is > 0.028 m. Exactly 0.028 PASSES. No other threshold exists in this stage.

WHAT THIS IS NOT
    No fit, no optimizer step, no weight update, no critic, no PPO. Nothing is promoted and
    nothing is called deployable. FSM, the morphology corridor, the reward, the SEA and the C++
    plugin are untouched, as are every J0-J19A artefact and the production configuration.

HARDENING CARRIED FORWARD FROM J10R1 AND J19A
    J9R1 committed under an exclusive lock and an atomic rename but performed NO post-commit
    verification. This stage adds: leaf-relative recorded paths, exactly-19 sim_outputs per cell,
    a TECHNICAL_INVALID marker written into the staging BEFORE the rename so the leaf is born
    invalid, and a post-commit re-resolution and re-hash of every recorded path against the
    COMMITTED receipt. The leaf is valid evidence if and only if commit_verification.json exists
    and declares pass true.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
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
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1            # noqa: E402  hardened env builder + summariser
import v26c_j3_closed_loop as J3        # noqa: E402  deterministic parity + kinematic gate
import v26c_penetration_contract as PC  # noqa: E402  the ONLY penetration authority
import v26c_j9r1_closed_loop as J9R1    # noqa: E402  FROZEN authority on the gates

REPO = J1.REPO if hasattr(J1, "REPO") else HERE.parents[3]
BASELINE = J3.BASELINE


class J19BError(RuntimeError):
    pass


STAGE = "V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION"
PREREG = HERE / "v26c_j19b_prereg_closed_loop_qualification.json"
PIN_PREREG = "31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8"
# the J9 originals, pinned as IMMUTABLE EVIDENCE. This stage neither edits nor executes them;
# the offset-unit correction is read from the first of them.
J9_PREREG_SHA = "e6a6888a473d5ef76b1f009037f7825995fd62b35cc51749dfe41ad7f718c44a"
J9_RUNNER_SHA = "8b6c36db2fd9bd63bde41d9903d4e1eb1d1e312b8cbf790e9bf1ea5b4875ac67"
J9_TEST_SHA = "c30a1ff40d1f3aeefdb1e02210ecf6e2a04d7749448a7592cba212e8b47d867f"
J9_AUTHORIZATION_SHA = "ff789766cb27798b659f7922ea2d88a0f49f112bb054634c6258a9dd49639597"
J9_FAILURE_RECORD_SHA = "67c97df1b302cfa98a77c673410e67e56ea0dd34b2d380cf65c34f8916645a20"

# ------------------------------------------------------------------ the ONLY actor --------------
# The J19A leaf, pinned FILE BY FILE. SEVEN files: J19A commits no aggregate dataset -
# it trains nothing - but it does commit its own result and its own
# commit_verification.json, whose schema uses "ok" where J15R1's used "pass".
J19A_LEAF = HERE / "j19a_runs" / "j19a_single_reproduction_v26c_2026-08-27_r1"
J19A_MODULE_DIR = J19A_LEAF / "rl_module"
PIN_J19A: dict[str, str] = {
    "commit_verification.json":
        "4d526dc119a10983e0186257132fbff58569d90ab0d1a793e0afae5ba2f32c61",
    "history.json":
        "0a00cb6be58945d525201f76aa425ad713addbd4946d108e02cf4bc2c4111af0",
    "rl_module/actor_feature_manifest.json":
        "2c01067e9a569354cc4099537a3a556ab50d55aa8baa1d2127324dafeee27c54",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/module_state.pkl":
        "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530",
    "v26c_j19a_result.json":
        "8982b2fc40e514bed903af0c33e0a4ab3737ebf849a44a232912a816002dc254",
    "v26c_j19a_single_reproduction_receipt.json":
        "235a117fc849bbe137dfd7ea29621390a6d1aa71aa9f9b4d95ca0e7a5dd50dad",
}
PIN_J19A_RECEIPT_NAME = "v26c_j19a_single_reproduction_receipt.json"
# The 35 feature NAMES are not in the J19A manifest. Three independent pinned
# sources carry them and MUST agree; see resolve_feature_names.
PIN_FEATURE_NAME_SOURCES: dict[str, str] = {
    "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1/rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "j15_runs/j15_fresh_refit_v26c_2026-08-27_r1/rl_module/actor_feature_manifest.json":
        "bb24bedca3f8572e370d92ff02640a3890171888215017cd2821d62245670653",
    "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1/"
    "j10r1_cell_B_teacher_dataset.npz":
        "2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62",
}
PIN_J19A_PARENT_J8 = "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82"

# The J15R1-lineage provenance constants J16 carried here - the J11 failed actor,
# the J14 corrective increment and the J15R1 nominal-drift addendum - describe a
# lineage that is NOT this actor's. Their only consumer was J16's verify_actor,
# which this derivation replaces, so they are dropped rather than left dangling
# or, worse, repointed to something they do not describe.

PIN_J19A_MODULE_STATE = PIN_J19A["rl_module/module_state.pkl"]
PIN_J19A_ACTOR_DIGEST = \
    "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"

# The frozen J9R1 runner, imported as the AUTHORITY on every behavioural threshold. Importing it
# performs no I/O and pulls no torch. It is never executed.
PIN_J9R1_MODULE = "f58b0d14b077536ced24ec3063bd2154ed1e207b42532bddc76177f1e02e4a3c"

# Lineage, kept DISTINCT because the two hashes are easy to conflate.
PIN_J2_PARENT = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
PIN_J7_DATASET = "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
PIN_J10R1_CELL_B = "2f37fc7cb101550d2fc0f8709cfdfc44ae5e9ae53003bb7903fcb590406acc62"
PIN_J10R1_CELL_C = "bd78e6ac13ab96d128f57ea5b36d058f5d80c18cfb056bcc76d2179bf1d756f0"
PIN_RUNTIME_CONFIG_SHA = J1.PIN_RUNTIME_CONFIG_SHA
PIN_CONTRACT = "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461"

# ------------------------------------------------------------------ the actor contract ----------
ACTOR_WIDTH = 35
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
WAIT_HS_FEATURE = "phase_fsm_wait_hs"
# DIAGNOSTIC ONLY. J9R1 cell B failed with the FSM held in WAIT_HS for 500/500 steps and zero
# cycles; J10R1 collected the missing rows and J19A fitted on them. Counting WAIT_HS here makes
# that failure mode VISIBLE per cell. It is NOT a gate: no threshold is attached to it, because
# this stage reuses J9R1's criteria exactly and invents none.
WAIT_HS_IS_BINDING = False

SIGMA = 0.005
SIGMA_TOLERANCE = 1e-6
NOISE_HOLD_STEPS = 1
EXPECTED_STEPS = 500
EXPECTED_SIM_OUTPUT_FILES = 19
# RLModule.from_checkpoint returns a weightless base RLModule, with NO exception, when
# asymmetric_rl_module cannot be imported. These two are what make that failure LOUD.
EXPECTED_MODULE_CLASS = "AsymmetricActorCriticTorchRLModule"
EXPECTED_STATE_KEYS = ("pi.0.0.bias", "pi.0.0.weight", "pi.0.2.bias", "pi.0.2.weight",
                       "pi.1.bias", "pi.1.weight", "pi_encoder.0.bias", "pi_encoder.0.weight",
                       "pi_encoder.2.bias", "pi_encoder.2.weight")

# ------------------------------------------------------------------ the frozen matrix -----------
OFFSET_NOMINAL = 1.956870983805102
OFFSET_MINUS = 1.756870983805102
OFFSET_PLUS = 2.156870983805102
OFFSET_UNIT = "seconds"
FROZEN_OFFSETS = (OFFSET_NOMINAL, OFFSET_MINUS, OFFSET_PLUS)
MATRIX: tuple[dict[str, Any], ...] = (
    {"id": "A", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_NOMINAL,
     "label": "nominal"},
    {"id": "B", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_MINUS,
     "label": "-0.20 s"},
    {"id": "C", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_PLUS,
     "label": "+0.20 s"},
    {"id": "D", "mode": "stochastic_held", "seed": 123, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
    {"id": "E", "mode": "stochastic_held", "seed": 124, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
    {"id": "F", "mode": "stochastic_held", "seed": 125, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
)
RESET_TIME_TOLERANCE_S = 1e-9
RESET_TIME_TOLERANCE_WHY = (
    "episode_start_offset_s is an offset RELATIVE TO cfg.t_start, never an absolute OpenSim time. "
    "The expected reset time is computed with the production semantics, so the comparison is an "
    "exact arithmetic identity and 1e-9 s is a cross-platform float-representation allowance, not "
    "a behavioural tolerance. It is checked IMMEDIATELY after reset, never from a post-step "
    "record.")
RESET_SEMANTICS = (
    "osim_trj_cmc_like._initialise_episode: "
    "max_start = max(cfg.t_start, cfg.t_end - episode_duration); "
    "requested = cfg.t_start + max(0.0, episode_start_offset_s); "
    "episode_start = min(requested, max_start); self.t = episode_start")


def expected_reset_time(t_start: float, t_end: float, episode_duration: float | None,
                        offset_s: float) -> dict[str, Any]:
    """The production episode-start arithmetic, transcribed. Offsets are RELATIVE to t_start."""
    max_start = float(t_end)
    if episode_duration is not None:
        max_start = float(t_end) - float(episode_duration)
    max_start = max(float(t_start), max_start)
    requested = float(t_start) + max(0.0, float(offset_s))
    start = min(requested, max_start)
    return {"cfg_t_start_s": float(t_start), "cfg_t_end_s": float(t_end),
            "episode_duration_s": None if episode_duration is None else float(episode_duration),
            "episode_start_offset_s": float(offset_s),
            "requested_unclamped_time_s": requested,
            "max_start_s": max_start,
            "expected_reset_time_s": start,
            "clamped": bool(requested > max_start),
            "semantics": RESET_SEMANTICS}

# ------------------------------------------------------------------ the gate --------------------
# J1's common criteria MINUS its 0.020 penetration bar: the sole binding penetration criterion is
# the contract's 0.028 band, exactly as J6 established.
J19B_COMMON_GATE: dict[str, Any] = {k: v for k, v in J1.J1_GATE.items()
                                  if k != "max_penetration_m_max"}
J19B_KINEMATIC_GATE: dict[str, Any] = dict(J3.J3_KINEMATIC_GATE)   # UNCHANGED
J19B_GATE_SOURCE = ("v26c_j1_collect.J1_GATE for every non-penetration criterion, "
                  "v26c_j3_closed_loop.J3_KINEMATIC_GATE unchanged for the kinematic quality, and "
                  "the penetration contract's 0.028 band as the SOLE binding penetration bar")
# valid_cycle_count is DELIBERATELY absent: it is a BINDING check in evaluate_cell_gate, and
# listing its measure among the non-binding diagnostics would be self-contradictory. J9R1 listed
# it; that is corrected here without touching any threshold.
DIAGNOSTIC_NOT_BINDING = ("action_clipped_steps", "episode_return",
                          "realized_noise_rms", "policy_std")

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j19b_runs", "j19b_closed_loop_v26c_2026-08-27_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
RECEIPT_NAME = "v26c_j19b_closed_loop_receipt.json"
COMMIT_VERIFICATION_NAME = "commit_verification.json"
TECHNICAL_INVALID_NAME = "TECHNICAL_INVALID"
# the INERT sentinel output root the preflight builds against. It must never be created.
PREFLIGHT_SENTINEL = HERE / "_j19b_preflight_sentinel_never_created"
OUTPUT_ROOT_OVERRIDE: Path | None = None

FORBIDDEN_HERE = ("fit", "optimizer step", "weight update", "critic", "PPO", "promotion",
                  "deployability", "editing or copying the actor", "editing a log-std head",
                  "a second actor", "widening", "contralateral features", "a standalone 25D actor",
                  "LOTO", "LOCO", "B1R1", "B1R2", "behavioural fail-fast",
                  "an autonomous retry", "a local penetration threshold",
                  # The env-mutation rule, stated so it cannot be misread. NO scientific or
                  # runtime field may move: not the FSM, not the corridor, not the reward, not
                  # the SEA, not a threshold, not a duration, not a seed-bearing config key.
                  # episode_start_offset_s is the ONLY such field the matrix varies, because the
                  # matrix IS a sweep over the three preregistered starts.
                  "mutating any scientific or runtime env field; episode_start_offset_s is the "
                  "ONLY one the matrix varies",
                  # output_dir is NOT an exception to that rule: it is not a scientific or
                  # runtime field at all. It is per-cell INSTRUMENTATION - where this cell's 19
                  # sim_outputs are written - and it is authorised precisely so each cell's
                  # recording lands in its own directory inside the staging.
                  "writing any cell's outputs outside its own authorised output_dir",
                  "starting a Ray cluster", "starting Ray workers", "EnvRunners")
ENV_MUTATION_POLICY = {
    "scientific_or_runtime_fields_mutable": ["episode_start_offset_s"],
    "why": "the matrix is a sweep over the three preregistered start offsets; nothing else about "
           "the environment is allowed to differ between cells",
    "instrumentation_fields_mutable": ["output_dir"],
    "output_dir_is_not_a_scientific_field": (
        "output_dir names where this cell's 19 sim_outputs are written. It carries no scientific "
        "or runtime meaning, changes no dynamics and enters no gate; it is per-cell "
        "instrumentation, authorised so each cell records into its own directory."),
    "everything_else": "IMMUTABLE - proved by the stable-key equality guard in cell_env_config, "
                       "which compares every other key against the verified base",
}


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _sha_array(a: np.ndarray) -> str:
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


# ================================================================ inputs =========================

def _leaf_rel(q: Path, root: Path) -> str:
    """LEAF-RELATIVE, so the string stays valid after the staging is renamed onto the leaf.

    J9R1 recorded `_rel(sim_out)`, a REPO-relative STAGING path that ceased to exist at the
    commit. That provenance defect is corrected here, as it was in J10R1 and J19A.
    """
    try:
        rel = Path(q).relative_to(root)
    except ValueError:
        raise J19BError(f"{q} is not under the staging root {root}") from None
    if ".." in rel.parts:
        raise J19BError(f"the relative path for {q} under {root} escapes the leaf: {rel}")
    return str(rel).replace(os.sep, "/")


def _resolve_inside(leaf: Path, rel: str) -> Path:
    if not rel or Path(rel).is_absolute() or ".." in Path(rel).parts:
        raise J19BError(f"recorded path is not leaf-relative: {rel!r}")
    q = leaf / rel
    if q.is_symlink():
        raise J19BError(f"recorded path is a symlink, which is never committed here: {rel}")
    try:
        q.resolve().relative_to(leaf.resolve())
    except ValueError:
        raise J19BError(f"recorded path escapes the committed leaf: {rel}") from None
    return q


def verify_committed_leaf(leaf: Path, *, expected_receipt_sha: str | None = None) -> dict[str, Any]:
    """Re-resolve EVERY recorded leaf-relative path in the COMMITTED leaf and re-hash it.

    Carried forward from J10R1 and J19A. J9R1 recorded its receipt's hash after the rename and
    compared it to nothing; the rename is atomic, but atomicity says nothing about whether the
    committed bytes are the bytes that were measured.
    """
    receipt = leaf / RECEIPT_NAME
    receipt_bytes = receipt.read_bytes() if receipt.is_file() else None
    receipt_sha = hashlib.sha256(receipt_bytes).hexdigest() if receipt_bytes is not None else None
    matches_staging: bool | None = None
    if expected_receipt_sha is not None:
        matches_staging = receipt_sha == expected_receipt_sha
    if receipt_bytes is None:
        return {"schema": "v26c_j19b_commit_verification.1", "stage": STAGE, "pass": False,
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
        "schema": "v26c_j19b_commit_verification.1", "stage": STAGE,
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


def verify_prereg() -> dict[str, Any]:
    """This stage's OWN preregistration, in the J19B schema.

    PROVENANCE ONLY. Adapted from J16 because the J19B preregistration is a
    different document with a different schema; every scientific field it
    checks - the matrix, the steps, the noise contract, the fail-fast rule - is
    checked against the SAME runner constants, which the derivation leaves
    untouched.
    """
    if not PREREG.is_file():
        raise J19BError("the J19B preregistration is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J19BError(f"the J19B preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage") != STAGE:
        raise J19BError(f"the preregistration declares {data.get('stage')!r}, not {STAGE}")

    # the matrix, field by field, against the runner's own frozen MATRIX
    declared = data["the_six_cells"]["cells"]
    if [c["id"] for c in declared] != [c["id"] for c in MATRIX]:
        raise J19BError("the preregistered matrix order and the runner's disagree")
    for want, got in zip(MATRIX, declared):
        if (str(got["mode"]), int(got["seed"]), float(got["offset_s"])) != \
                (want["mode"], want["seed"], float(want["offset_s"])):
            raise J19BError(f"cell {want['id']} differs from the preregistration")
    if sorted(int(s) for s in data["the_six_cells"]["seeds_used"]) != [123, 124, 125]:
        raise J19BError("the preregistration declares seeds other than 123, 124, 125")

    # the actor identity
    actor = data["actor_under_test"]
    if actor["module_state_sha256"] != PIN_J19A_MODULE_STATE \
            or actor["actor_digest"] != PIN_J19A_ACTOR_DIGEST:
        raise J19BError("the preregistration names a different actor than the pins")
    if actor.get("parent") != "J8":
        raise J19BError("the preregistration must name J8 as the parent")

    # the penetration bands, restated nowhere but asserted here
    bands = data["penetration_contract"]
    if bands.get("unchanged_from_j16") is not True:
        raise J19BError("the preregistration must declare the penetration contract unchanged")

    if data["execution_discipline"].get("single_execution") is not True:
        raise J19BError("the preregistration must declare a single execution")
    return {"file": _rel(PREREG), "sha256": digest,
            "stage": data["stage"], "schema": "J19B",
            "gates_are_j9r1s": True}


def verify_scientific_equivalence() -> dict[str, Any]:
    """Prove FIELD BY FIELD that this stage changed the ACTOR ONLY.

    The thresholds are not re-derived here and not re-typed here: they are read from the FROZEN
    J9R1 module, which is pinned by hash, and compared value by value. If any of them ever
    diverged, this raises before an environment is constructed. That is what 'invents no
    threshold' means operationally.
    """
    path = Path(J9R1.__file__).resolve()
    digest = _sha_file(path)
    if digest != PIN_J9R1_MODULE:
        raise J19BError(f"the J9R1 module changed: {digest} != {PIN_J9R1_MODULE}")
    if path.parent != HERE:
        raise J19BError(f"the imported J9R1 module is not the one beside this stage: {path}")

    compared: dict[str, Any] = {}

    def same(name: str, mine: Any, theirs: Any) -> None:
        a = {k: (list(v) if isinstance(v, tuple) else v) for k, v in mine.items()} \
            if isinstance(mine, Mapping) else (list(mine) if isinstance(mine, tuple) else mine)
        b = {k: (list(v) if isinstance(v, tuple) else v) for k, v in theirs.items()} \
            if isinstance(theirs, Mapping) else (list(theirs) if isinstance(theirs, tuple)
                                                 else theirs)
        if a != b:
            raise J19BError(f"{name} differs from the frozen J9R1 source: {a!r} != {b!r}")
        compared[name] = a

    same("COMMON_GATE", J19B_COMMON_GATE, J9R1.J9_COMMON_GATE)
    same("KINEMATIC_GATE", J19B_KINEMATIC_GATE, J9R1.J9_KINEMATIC_GATE)
    same("SIGMA", SIGMA, J9R1.SIGMA)
    same("SIGMA_TOLERANCE", SIGMA_TOLERANCE, J9R1.SIGMA_TOLERANCE)
    same("NOISE_HOLD_STEPS", NOISE_HOLD_STEPS, J9R1.NOISE_HOLD_STEPS)
    same("EXPECTED_STEPS", EXPECTED_STEPS, J9R1.EXPECTED_STEPS)
    same("RESET_TIME_TOLERANCE_S", RESET_TIME_TOLERANCE_S, J9R1.RESET_TIME_TOLERANCE_S)
    same("CLOCK_COLUMNS", CLOCK_COLUMNS, J9R1.CLOCK_COLUMNS)
    same("CONTROLLER_COLUMNS", CONTROLLER_COLUMNS, J9R1.CONTROLLER_COLUMNS)
    same("ACTOR_WIDTH", ACTOR_WIDTH, J9R1.ACTOR_WIDTH)
    same("FROZEN_OFFSETS", FROZEN_OFFSETS, J9R1.FROZEN_OFFSETS)

    # the matrix, cell by cell: id, mode, seed and offset must all be identical
    if len(MATRIX) != len(J9R1.MATRIX):
        raise J19BError("the matrix length differs from the frozen J9R1 source")
    cells = []
    for mine, theirs in zip(MATRIX, J9R1.MATRIX):
        for field in ("id", "mode", "seed", "offset_s"):
            if mine[field] != theirs[field]:
                raise J19BError(f"matrix cell {mine['id']}: {field} differs from J9R1 "
                               f"({mine[field]!r} != {theirs[field]!r})")
        cells.append({k: mine[k] for k in ("id", "mode", "seed", "offset_s")})

    # the gate is also still J1's and J3's at source, not merely J9R1's copy of them
    if J19B_COMMON_GATE != {k: v for k, v in J1.J1_GATE.items() if k != "max_penetration_m_max"}:
        raise J19BError("the common gate is no longer J1_GATE minus the soft penetration bar")
    if J19B_KINEMATIC_GATE != dict(J3.J3_KINEMATIC_GATE):
        raise J19BError("the kinematic gate is no longer J3_KINEMATIC_GATE")

    return {
        "source_module": _rel(path), "source_sha256": digest,
        "compared_fields": compared, "matrix": cells,
        "what_changed": "the ACTOR under test, and nothing else",
        "actor_before": "J15R1 (j15_runs/j15_fresh_refit_v26c_2026-08-27_r1), which "
                        "FAILED this same matrix 0/6 in J16",
        "actor_now": "J19A (j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1)",
        "thresholds_invented_here": 0,
        "ultimate_sources": {
            "common": "v26c_j1_collect.J1_GATE minus max_penetration_m_max",
            "kinematic": "v26c_j3_closed_loop.J3_KINEMATIC_GATE, unchanged",
            "penetration": "v26c_penetration_contract, the 0.028 band as the sole binding bar"},
    }


def resolve_feature_names() -> dict[str, Any]:
    """The 35 feature NAMES, from three independent pinned sources that must agree.

    The J19A manifest does not carry them. The J8 sidecar does, but that sidecar
    is KNOWN STALE - it is byte-identical to J2's and misstates the module hash.
    It is therefore used for the feature ORDER AND NAMES ONLY, never as evidence
    of any module hash, and its agreement with two other pinned sources - the
    J15R1 manifest that J16 itself used, and the J10R1 teacher dataset - is
    REQUIRED. One source alone would not be trusted here.
    """
    resolved: dict[str, tuple[str, ...]] = {}
    for rel, pin in PIN_FEATURE_NAME_SOURCES.items():
        path = HERE / rel
        got = _sha_file(path)
        if got != pin:
            raise J19BError(f"the pinned feature-name source {rel} changed: {got} != {pin}")
        if rel.endswith(".npz"):
            with np.load(path, allow_pickle=False) as bundle:
                names = tuple(str(n) for n in bundle["actor_feature_names"])
        else:
            names = tuple(str(n) for n in
                          json.loads(path.read_text())["actor_feature_names"])
        resolved[rel] = names

    distinct = {v for v in resolved.values()}
    if len(distinct) != 1:
        raise J19BError(f"the three pinned feature-name sources disagree: "
                        f"{ {k: len(v) for k, v in resolved.items()} }")
    names = distinct.pop()
    if len(names) != ACTOR_WIDTH:
        raise J19BError(f"the pinned sources hold {len(names)} names, not {ACTOR_WIDTH}")
    return {"actor_feature_names": names, "sources": dict(PIN_FEATURE_NAME_SOURCES),
            "sources_agree": True, "count": len(names),
            "j8_sidecar_used_for": "feature ORDER AND NAMES ONLY",
            "j8_sidecar_is_known_stale": True,
            "j8_sidecar_not_used_as_module_hash_evidence": True}


def verify_actor() -> dict[str, Any]:
    """The exact SEVEN-file J19A leaf, the 35D mask contract, and the frozen sigma.

    PROVENANCE AND INTEGRITY ONLY. Adapted from J16 because the J19A leaf has a
    different file set and its receipt, manifest and commit verification use
    different schemas. Every property J16 asserted is asserted here at its new
    address, and every threshold - ACTOR_WIDTH, CLOCK_COLUMNS, CONTROLLER_COLUMNS,
    SIGMA, SIGMA_TOLERANCE - is read from this module's own untouched constants.
    """
    if not J19A_LEAF.is_dir():
        raise J19BError(f"the J19A leaf is missing: {J19A_LEAF}")
    present = sorted(str(p.relative_to(J19A_LEAF)).replace(os.sep, "/")
                     for p in J19A_LEAF.rglob("*") if p.is_file())
    if present != sorted(PIN_J19A):
        raise J19BError(f"the J19A leaf holds {present}, expected exactly {sorted(PIN_J19A)}")
    marker = J19A_LEAF / TECHNICAL_INVALID_NAME
    if marker.exists():
        raise J19BError(f"the J19A leaf is marked {TECHNICAL_INVALID_NAME}; its actor may not "
                        f"be loaded, whatever its receipt says")
    cv_path = J19A_LEAF / COMMIT_VERIFICATION_NAME
    if not cv_path.is_file():
        raise J19BError(f"the J19A leaf carries no {COMMIT_VERIFICATION_NAME}: it is not valid "
                        f"evidence and its actor may not be loaded")
    cv = json.loads(cv_path.read_text())
    # the J19A commit-verification schema uses "ok", where J15R1's used "pass"
    if cv.get("ok") is not True or cv.get("problems"):
        raise J19BError(f"the J19A commit verification does not pass: ok={cv.get('ok')!r} "
                        f"problems={cv.get('problems')!r}")
    for stale in (J19A_LEAF.parent.glob(".lock_*"), J19A_LEAF.parent.glob(".staging_*")):
        leftovers = sorted(q.name for q in stale)
        if leftovers:
            raise J19BError(f"the J19A run directory still holds {leftovers}; the commit did "
                            f"not complete cleanly and the leaf is not trustworthy")

    checked: dict[str, str] = {}
    for rel, pin in PIN_J19A.items():
        got = _sha_file(J19A_LEAF / rel)
        if got != pin:
            raise J19BError(f"the J19A artefact {rel} changed: {got} != {pin}")
        checked[rel] = got

    with (J19A_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    inputs = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and v.ndim == 2 and v.shape[1] == ACTOR_WIDTH)
    if not inputs:
        raise J19BError(f"the J19A state holds no {ACTOR_WIDTH}D input layer")
    clock = list(CLOCK_COLUMNS)
    controller = list(CONTROLLER_COLUMNS)
    layers: dict[str, Any] = {}
    for key in inputs:
        W = state[key]
        zero = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if zero != clock:
            raise J19BError(f"the J19A input layer {key} has zero columns {zero}; expected "
                            f"exactly the clock {clock}. The controller columns must be LIVE.")
        norms = {int(c): float(np.linalg.norm(W[:, c])) for c in controller}
        dead = sorted(c for c, v in norms.items() if v <= 0.0)
        if dead:
            raise J19BError(f"the J19A input layer {key} has dead controller columns {dead}")
        layers[key] = {"zero_columns": zero, "controller_norms": norms}
    widths = sorted({int(state[k].shape[1]) for k in inputs})
    if widths != [ACTOR_WIDTH]:
        raise J19BError(f"the J19A input layers are {widths}D; expected {ACTOR_WIDTH}D")

    # sigma comes from the actor's own FROZEN log-std head; this stage never edits one
    w = np.asarray(state["pi.1.weight"])
    b = np.asarray(state["pi.1.bias"])
    dim = w.shape[0] // 2
    if not bool(np.all(w[dim:] == 0.0)):
        raise J19BError("the actor's log-std output weights are not zero, so sigma is not "
                        "constant")
    sigmas = np.exp(b[dim:].astype(np.float64))
    if not bool(np.all(np.isfinite(sigmas))) \
            or float(np.max(np.abs(sigmas - SIGMA))) > SIGMA_TOLERANCE:
        raise J19BError(f"the actor carries sigma {sigmas.tolist()}, expected {SIGMA} within "
                        f"{SIGMA_TOLERANCE}. This stage REFUSES to edit a log-std head.")

    receipt = json.loads((J19A_LEAF / PIN_J19A_RECEIPT_NAME).read_text())
    lineage = {
        "j19a_verdict": receipt["verdict"],
        "actors_persisted": int(receipt["actors_persisted"]),
        "rollout_performed": receipt["rollout_performed"],
        "reproducibility_ok": receipt["reproducibility_ok"],
        "exact_fields_ok": receipt["exact_fields_ok"],
        "eligibility_ok": receipt["eligibility_ok"],
        "parent_j8_sha256": receipt["inputs"]["parent_j8_sha256"],
    }
    if lineage["j19a_verdict"] != "PASS":
        raise J19BError(f"the J19A receipt verdict is {lineage['j19a_verdict']!r}, not PASS")
    if lineage["actors_persisted"] != 1:
        raise J19BError("the J19A receipt does not record exactly one persisted actor")
    if lineage["rollout_performed"] is not False:
        raise J19BError("the J19A receipt claims a rollout, which that stage never ran")
    for flag in ("reproducibility_ok", "exact_fields_ok", "eligibility_ok"):
        if lineage[flag] is not True:
            raise J19BError(f"the J19A receipt does not declare {flag} true")
    if lineage["parent_j8_sha256"] != PIN_J19A_PARENT_J8:
        raise J19BError("the J19A receipt records a different J8 parent")

    # the manifest must describe THIS module, not its parent
    manifest = json.loads((J19A_MODULE_DIR / "actor_feature_manifest.json").read_text())
    if manifest.get("module_state_sha256") != PIN_J19A_MODULE_STATE:
        raise J19BError(f"the J19A manifest names module {manifest.get('module_state_sha256')}, "
                        f"not the module beside it ({PIN_J19A_MODULE_STATE})")
    if manifest.get("actor_digest") != PIN_J19A_ACTOR_DIGEST:
        raise J19BError("the J19A manifest carries an unexpected actor_digest")
    if manifest.get("actor_digest") == manifest.get("source_actor_digest"):
        raise J19BError("the J19A manifest declares the actor identical to its parent")
    if int(manifest.get("observation_width", 0)) != ACTOR_WIDTH:
        raise J19BError(f"the J19A manifest does not declare {ACTOR_WIDTH} observation width")
    # the J19A manifest deliberately carries NEITHER actor_feature_names NOR deployable;
    # names come from resolve_feature_names, and no stage has demonstrated deployability
    if "actor_feature_names" in manifest or "deployable" in manifest:
        raise J19BError("the J19A manifest schema changed; this stage was derived against the "
                        "schema that carries neither actor_feature_names nor deployable")

    names = resolve_feature_names()
    return {
        "leaf": _rel(J19A_LEAF), "module": _rel(J19A_MODULE_DIR),
        "artefacts_sha256": checked, "file_count": len(checked),
        "leaf_validity": {"commit_verification_ok": True, "technical_invalid_marker": False,
                          "no_stale_lock_or_staging": True,
                          "rule": "a J19A leaf is valid evidence only while its own "
                                  "commit_verification.json declares ok true"},
        "manifest": {"module_state_sha256": manifest["module_state_sha256"],
                     "actor_digest": manifest["actor_digest"],
                     "describes_this_module": True,
                     "differs_from_parent": True},
        "mask_contract": {"width": ACTOR_WIDTH, "clock_columns": clock,
                          "controller_columns": controller, "layers": layers},
        "sigma": {"value": SIGMA, "from_actor": sigmas.tolist(),
                  "tolerance": SIGMA_TOLERANCE, "logstd_head_edited": False},
        "lineage": lineage,
        "feature_names": names,
    }


def unit_correction() -> dict[str, Any]:
    """The additive correction: the offsets are SECONDS. No numeric artefact was affected."""
    # the offset-unit correction was authored in the ORIGINAL J9 preregistration and is
    # unchanged since. It is read from that PINNED file, whose hash is checked here.
    j9 = HERE / "v26c_j9_prereg_closed_loop.json"
    got = _sha_file(j9)
    if got != J9_PREREG_SHA:
        raise J19BError(f"the original J9 preregistration changed: {got} != {J9_PREREG_SHA}")
    data = {**json.loads(j9.read_text())["offset_unit_correction"],
            "source": "v26c_j9_prereg_closed_loop.json (pinned, unedited)",
            "source_sha256": got}
    return {**data, "unit": OFFSET_UNIT, "frozen_offsets_s": list(FROZEN_OFFSETS),
            "nominal_s": OFFSET_NOMINAL,
            "delta_minus_s": OFFSET_MINUS - OFFSET_NOMINAL,
            "delta_plus_s": OFFSET_PLUS - OFFSET_NOMINAL,
            "earlier_records_edited": False}


# ================================================================ env configuration ==============

def base_env_config(output_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """The pinned config through the HARDENED J1 builder, with a REAL output root.

    THE J9R1 CORRECTION. The J1 builder makes recording conditional on this argument
    (v26c_j1_collect.py:325-330): with output_dir=None it sets record_outputs False,
    save_outputs_on_close False and omits output_dir and output_prefix entirely. J9 passed None
    and then required the recorded output, which nothing had been asked to write.

    Passing a REAL root is what J1, J3, J5 and J6 all do. The base therefore carries recording ON,
    and every cell inherits it. The flags are then verified fail-closed, so this stage can never
    again reach an environment that was told not to record.
    """
    if output_root is None:
        raise J19BError("base_env_config requires a REAL output root. Passing None is exactly the "
                      "J9 defect: it disables record_outputs and save_outputs_on_close and omits "
                      "output_dir and output_prefix.")
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_root)
    J1.verify_env_config(env, cfg)
    if float(env["episode_start_offset_s"]) != OFFSET_NOMINAL:
        raise J19BError(f"the pinned config's episode_start_offset_s is "
                      f"{env['episode_start_offset_s']}, expected the nominal {OFFSET_NOMINAL}")
    # FAIL-CLOSED on the instrumentation itself, BEFORE any environment exists
    if env.get("record_outputs") is not True:
        raise J19BError(f"record_outputs is {env.get('record_outputs')!r}, expected exactly True. "
                      "An environment that is not recording cannot satisfy the sim_outputs "
                      "requirement, and this stage refuses to discover that after a 500-step run.")
    if env.get("save_outputs_on_close") is not True:
        raise J19BError(f"save_outputs_on_close is {env.get('save_outputs_on_close')!r}, expected "
                      "exactly True. Without it osim_trj_cmc_like.close() never calls "
                      "runner.save_results().")
    if not env.get("output_prefix"):
        raise J19BError("output_prefix is absent from the base env config; the J1 builder sets it "
                      "only alongside a real output_dir")
    if not env.get("output_dir"):
        raise J19BError("output_dir is absent from the base env config")
    return env, cfg


def cell_env_config(base: Mapping[str, Any], cfg: Mapping[str, Any], offset_s: float,
                    output_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Mutate ONLY episode_start_offset_s, and PROVE that nothing else moved."""
    if float(offset_s) not in [float(v) for v in FROZEN_OFFSETS]:
        raise J19BError(f"{offset_s} is not one of the three frozen offsets {FROZEN_OFFSETS}")
    env = dict(base)
    env["episode_start_offset_s"] = float(offset_s)
    env["output_dir"] = str(output_dir)
    differing = sorted(k for k in set(base) | set(env)
                       if base.get(k, "<absent>") != env.get(k, "<absent>"))
    allowed = sorted({"episode_start_offset_s", "output_dir"})
    # a SUBSET check: the nominal cell legitimately differs only in output_dir, because its
    # offset already equals the base one. Nothing outside the allowed pair may ever move.
    forbidden = sorted(set(differing) - set(allowed))
    if forbidden:
        raise J19BError(f"the cell config mutates {forbidden}; ONLY episode_start_offset_s (and the "
                      f"per-cell output_dir) may differ from the verified base")
    if float(env["episode_start_offset_s"]) != float(offset_s):
        raise J19BError("the cell config does not carry the requested offset")
    added = sorted(set(env) - set(base))
    removed = sorted(set(base) - set(env))
    if removed or sorted(set(added) - {"output_dir"}):
        raise J19BError(f"the cell config adds {added} and removes {removed} relative to the "
                      f"verified base; only the per-cell output_dir may be added")
    # NO second J1.verify_env_config on the MUTATED config. J1 deliberately pins
    # episode_start_offset_s to the pinned config's own value, including in its RUNTIME LITERAL
    # CROSS-CHECK, so cells B and C would fail it for a purely technical reason. The contract is:
    #   (1) the base is built AND verified in full by J1 BEFORE any mutation;
    #   (2) a shallow copy changes only episode_start_offset_s and output_dir;
    #   (3) fail-closed proof by key-set and value equality of EVERY other field against that
    #       already-verified base.
    # (3) is what carries the guarantee here, and it is stronger than re-running a verifier that
    # cannot express the one difference we declared.
    #
    # THE STABLE-KEY SET, defined explicitly. The J9 form excluded only episode_start_offset_s and
    # required len(base) - 1 identical fields. With a recording-enabled base, output_dir is ALREADY
    # in the base and legitimately differs per cell, so that count would be len(base) - 2 and the
    # guard would fail every cell. The correct statement is: every key that is NOT one of the two
    # declared-mutable ones must be exactly equal.
    stable_keys = set(base) - {"episode_start_offset_s", "output_dir"}
    differing_stable = sorted(k for k in stable_keys if base[k] != env.get(k, "<absent>"))
    if differing_stable:
        raise J19BError(f"these stable fields differ from the verified base: {differing_stable}. "
                      f"Only episode_start_offset_s and output_dir may ever move.")
    return env, {"mutated_keys": differing, "allowed_superset": allowed,
                 "mutated_is_subset_of_allowed": True,
                 "stable_keys_count": len(stable_keys),
                 "stable_keys_all_identical": True,
                 "stable_key_definition": "every base key except episode_start_offset_s and "
                                          "output_dir, compared for exact equality",
                 "episode_start_offset_s": float(offset_s),
                 "base_offset_s": float(base["episode_start_offset_s"]),
                 "delta_s": float(offset_s) - float(base["episode_start_offset_s"]),
                 "unit": OFFSET_UNIT,
                 "every_other_field_identical": True,
                 "base_verified_by_j1_before_mutation": True,
                 "recording_inherited_from_base": True,
                 "second_j1_verify_on_mutated_config": False,
                 "why_no_second_verify": "J1.verify_env_config pins the nominal offset, including "
                                         "in its runtime literal cross-check, so B and C would "
                                         "fail it technically. The base is verified in full "
                                         "BEFORE the mutation and every other field is proved "
                                         "identical to it afterwards."}


# ================================================================ the runtime stack ==============

def production_stack() -> Any:
    """J3's production stack EXTENDED with the production stochastic helper, exactly as J6 does.

    Heavy imports live here only, so the preflight stays inert. J3 and J6 are never modified.
    """
    stack = J3.production_stack()
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import rollout_eval as RE
    import exploration_noise as EN
    stack.held_stochastic_action = (
        lambda module, obs, shape, unit: RE._held_stochastic_action(module, obs, shape, unit))
    stack.held_normal = EN.HeldStandardNormal
    return stack


# ================================================================ gate ===========================

def evaluate_cell_gate(summary: Mapping[str, Any], knee: np.ndarray, ankle: np.ndarray,
                       penetration: Mapping[str, Any]) -> dict[str, Any]:
    """The per-cell BINDING behavioural gate. HS/TO coherence is deliberately NOT in here."""
    checks = {
        "steps": summary["steps"] == J19B_COMMON_GATE["steps_required"],
        "end_reason": summary["end_reason"] == J19B_COMMON_GATE["end_reason"],
        "valid_cycles": summary["valid_cycle_count"] >= J19B_COMMON_GATE["valid_cycles_min"],
        "phase_timeout_stance":
            summary["phase_timeout_stance"] <= J19B_COMMON_GATE["phase_timeout_stance_max"],
        "phase_timeout_swing":
            summary["phase_timeout_swing"] <= J19B_COMMON_GATE["phase_timeout_swing_max"],
        "morphology_causal_contract_failure":
            summary["morphology_causal_contract_failure"]
            <= J19B_COMMON_GATE["morphology_causal_contract_failure_max"],
        "hs_cancelled_count":
            summary["hs_cancelled_count"] <= J19B_COMMON_GATE["hs_cancelled_count_max"],
        "resync_count": summary["resync_count"] <= J19B_COMMON_GATE["resync_count_max"],
        "penetration_hard_binding": bool(penetration["binding_pass"]),
    }
    kin = J3.kinematic_quality(np.asarray(knee, dtype=np.float64),
                               np.asarray(ankle, dtype=np.float64))
    for name, block in kin.items():
        checks[f"kinematic_{name}"] = bool(block["pass"])
    failed = sorted(k for k, v in checks.items() if not v)
    return {
        "criteria": {"common": dict(J19B_COMMON_GATE),
                     "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                           for k, v in J19B_KINEMATIC_GATE.items()},
                     "penetration": "the contract's 0.028 binding band, the SOLE binding "
                                    "penetration criterion"},
        "source": J19B_GATE_SOURCE,
        "kinematic_quality": kin,
        "ankle_min_direction": "ankle_min <= -0.03 rad; more negative PASSES, -0.0099 FAILS, "
                               "exactly -0.03 PASSES",
        "checks": checks, "failed": failed, "pass": not failed,
        "telemetry_integrity_evaluated_separately": True,
    }


def cell_verdict(gate: Mapping[str, Any], integrity: Mapping[str, Any]) -> str:
    return J3.overall_verdict(gate, integrity)


def penetration_report(series: Sequence[float], contract: Mapping[str, Any],
                       label: str) -> dict[str, Any]:
    """The ONLY penetration authority. No threshold is written down in this module."""
    ev = PC.evaluate_series(series, contract, label=label)
    return {**ev, "contract_sha256": contract["sha256"],
            "evaluated_by": "v26c_penetration_contract.evaluate_series",
            "soft_and_july_are_diagnostic": True,
            "sole_binding": "above_hard_binding"}


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: no heavy import, no environment, no write."""
    heavy_before = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                      "gymnasium") if m in sys.modules)
    prereg = verify_prereg()
    equivalence = verify_scientific_equivalence()
    actor = verify_actor()
    contract = PC.load_contract()
    if contract["sha256"] != PIN_CONTRACT:
        raise J19BError(f"the penetration contract changed: {contract['sha256']} != {PIN_CONTRACT}")
    # THE SENTINEL. The base is built against an output root that must NEVER be created: the J1
    # builder only formats path strings, and directory creation happens later inside
    # SimulationRunner.__init__, which the preflight never reaches. Existence is measured before
    # and after, so the claim is proved rather than asserted.
    sentinel_before = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                       "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    env, cfg = base_env_config(PREFLIGHT_SENTINEL)
    sentinel_after = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                      "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    planned = []
    for cell in MATRIX:
        _, mutation = cell_env_config(env, cfg, cell["offset_s"], HERE / "PREFLIGHT_NOT_A_PATH")
        planned.append({**{k: cell[k] for k in ("id", "mode", "seed", "label")},
                        "episode_start_offset_s": cell["offset_s"],
                        "offset_unit": OFFSET_UNIT,
                        "delta_from_nominal_s": mutation["delta_s"],
                        "mutated_keys": mutation["mutated_keys"],
                        "steps": EXPECTED_STEPS,
                        "sigma": SIGMA if cell["mode"] == "stochastic_held" else None,
                        "noise_hold_steps": (NOISE_HOLD_STEPS
                                             if cell["mode"] == "stochastic_held" else None)})
    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J19B lock is already held or was left behind: {lock_path}. This stage "
                        f"removes no lock it does not own.")
    heavy_after = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                     "gymnasium") if m in sys.modules)
    introduced = sorted(set(heavy_after) - set(heavy_before))
    if introduced:
        blockers.append(f"the preflight introduced heavy modules: {introduced}")
    if sentinel_after != sentinel_before:
        blockers.append(f"the preflight created the sentinel output root: {sentinel_before} -> "
                        f"{sentinel_after}. Building the env config must touch no filesystem.")
    if sentinel_after["root"] or sentinel_after["sim_outputs"]:
        blockers.append(f"the sentinel output root exists after the preflight: {sentinel_after}")
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "module_loaded": False,
                  "fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False,
                  "heavy_modules_before": heavy_before, "heavy_modules_after": heavy_after,
                  "heavy_modules_introduced_by_preflight": introduced,
                  "note": "inputs are verified and the matrix is planned in memory; nothing else"},
        "preregistration": prereg,
        "scientific_equivalence_to_j9r1": equivalence,
        "actor": actor,
        "offset_unit_correction": unit_correction(),
        "recording_instrumentation": {
            "record_outputs": env["record_outputs"],
            "save_outputs_on_close": env["save_outputs_on_close"],
            "output_prefix": env["output_prefix"],
            "base_output_dir_from_builder": env["output_dir"],
            "verified_fail_closed_before_any_env": True,
            "inherited_by_every_cell": True,
            "corrects": "the J9 defect recorded in v26c_j9_technical_failure_2026-08-26.json, inherited already corrected from J9R1",
            "parity": "the same recording mode J1, J3, J5 and J6 all used"},
        "preflight_sentinel": {
            "root": _rel(PREFLIGHT_SENTINEL),
            "existed_before": sentinel_before, "exists_after": sentinel_after,
            "created_by_the_preflight": False,
            "why_safe": "the J1 builder only formats path strings; directory creation happens "
                        "inside SimulationRunner.__init__, which the preflight never reaches",
            "measured_not_assumed": True},
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config + verify_env_config",
                    "base_offset_s": float(env["episode_start_offset_s"]),
                    "environments": "one at a time, sequential, closed cleanly and fail-closed",
                    "rllib_checkpoint_loader": True,
                    "ray_cluster_started": False, "ray_workers_started": False,
                    "env_runners": False,
                    "ray_import_note": "the RLlib checkpoint loader imports ray and torch at RUN time; importing them is not starting a cluster, a worker or an EnvRunner. The preflight imports neither.",
                    "sim_outputs_required": True},
        "penetration_authority": {
            "contract": contract["path"], "contract_sha256": contract["sha256"],
            "module_sha256": _sha_file(HERE / "v26c_penetration_contract.py"),
            "bands_m": {"soft_diagnostic": contract["soft_m"],
                        "july_legacy": contract["july_legacy_m"],
                        "hard_binding": contract["hard_m"]},
            "semantics": {"above_soft_iff": "> 0.020 (diagnostic)",
                          "july_legacy_breach_iff": ">= 0.025 (diagnostic)",
                          "binding_pass_iff": "<= 0.028, so exactly 0.028 PASSES",
                          "binding_fail_iff": "> 0.028"},
            "local_thresholds_in_this_module": 0},
        "planned_matrix": planned,
        "matrix_policy": {"cells": len(MATRIX), "steps_per_cell": EXPECTED_STEPS,
                          "order": [c["id"] for c in MATRIX],
                          "behavioural_fail_fast": False,
                          "rule": "a behavioural FAIL is preserved and the remaining cells still "
                                  "run; only a technical or integrity exception stops the matrix, "
                                  "fail-closed",
                          "aggregate_pass_iff": "6/6 behavioural PASS and 6/6 telemetry-valid"},
        "gate_specification": {
            "binding_common": dict(J19B_COMMON_GATE),
            "binding_kinematic": {k: (list(v) if isinstance(v, tuple) else v)
                                  for k, v in J19B_KINEMATIC_GATE.items()},
            "binding_penetration": "the contract's 0.028 band, sole binding penetration criterion",
            "ankle_min_direction": "ankle_min <= -0.03 rad; more negative PASSES, -0.0099 FAILS, "
                                   "exactly -0.03 PASSES",
            "telemetry_integrity": "SEPARATE fail-closed technical invariant, never behavioural",
            "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
            "source": J19B_GATE_SOURCE},
        "reset_time_check": {"tolerance_s": RESET_TIME_TOLERANCE_S,
                             "justification": RESET_TIME_TOLERANCE_WHY,
                             "semantics": RESET_SEMANTICS,
                             "offset_is_relative_to_t_start": True,
                             "compared_against": "cfg.t_start + offset, clamped to max_start",
                             "never_compared_against_the_offset_itself": True,
                             "checked_immediately_after_reset": True,
                             "recorded_fields": ["cfg_t_start_s", "cfg_t_end_s",
                                                 "episode_duration_s",
                                                 "requested_unclamped_time_s", "max_start_s",
                                                 "expected_reset_time_s",
                                                 "actual_reset_time_s", "reset_time_error_s",
                                                 "clamped"],
                             "checked_not_assumed": True},
        "multistart_disambiguation": {
            "training_data": (
                "PRESENT, and this is the difference from J9R1. The actor under test descends "
                "from J8 and was fitted on the J18 candidate-13 aggregate: four DISJOINT blocks "
                "totalling 4221 rows - the J10R1 cell-B teacher trajectory (500), the J8 "
                "on-policy pre-mismatch prefix (14), the closed-loop preservation anchors from "
                "cells A, C, D, E and F (2497), and the deduplicated J7 support (1210). "
                "J9R1's actor had none of that."),
            "which_multistart_data": (
                "the AUGUST J10R1 cells, collected with the prescribed teacher on the V26 "
                "lineage. NOT the two July teacher datasets, which remain DEFERRED and are never "
                "an operational input to any stage in this chain."),
            "closed_loop_validation": "cells B and C, and they are BINDING here",
            "they_are_different": (
                "multistart TRAINING DATA and multistart CLOSED-LOOP validation are separate "
                "things and must never be conflated. This stage validates in closed loop an "
                "actor that was trained on multistart rows; it does not train on anything."),
            "corrected_from_j9r1": (
                "J9R1 recorded that the multistart training data was absent, which was true of "
                "the J8 actor it tested. Carrying that sentence forward unchanged would have "
                "asserted something FALSE about the J19A actor.")},
        "would_write": {"leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
                        "leaf_exists": bool(leaf.exists()),
                        "per_cell": ["j19b_cell_<ID>_trace.json", "j19b_cell_<ID>_kinematics.npz",
                                     "j19b_cell_<ID>_penetration.npz",
                                     "j19b_cell_<ID>_sim_outputs/ (exactly 19 regular files)"],
                        "aggregate": RECEIPT_NAME,
                        "staging": staging.name, "lock": lock_path.name,
                        "protocol": "exclusive sibling lock, staging, verify, re-check, atomic "
                                    "rename",
                        "outcome_fail_still_commits": True,
                        "output_root_override": None if OUTPUT_ROOT_OVERRIDE is None
                        else str(OUTPUT_ROOT_OVERRIDE)},
        "outcome_policy": {"deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False, "critic": "excluded",
                           "ppo_updates": 0, "single_execution": True,
                           "no_autonomous_retry": True},
        "deferred_todo": json.loads(PREREG.read_text())["deferred_todo"],
        "env_mutation_policy": dict(ENV_MUTATION_POLICY),
        "requires_to_run": {"flag": "--run", "stage_token": STAGE,
                            "out": "must equal the authorised leaf exactly"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ destination guards =============

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J19BError(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J19BError(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J19BError("--run requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J19BError(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J19BError(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J19BError(f"the authorised leaf already exists; this stage is no-clobber and "
                      f"single-execution: {leaf}")
    return leaf


# ================================================================ one cell =======================

def run_cell(cell: Mapping[str, Any], *, stack: Any, base_env: Mapping[str, Any],
             cfg: Mapping[str, Any], contract: Mapping[str, Any], staging: Path,
             feature_names_expected: Sequence[str], progress: bool = True) -> dict[str, Any]:
    """ONE closed-loop cell. Builds, resets, steps and CLOSES exactly one environment."""
    cid = str(cell["id"])
    seed = int(cell["seed"])
    mode = str(cell["mode"])
    sim_out = staging / f"j19b_cell_{cid}_sim_outputs"
    env_config, mutation = cell_env_config(base_env, cfg, float(cell["offset_s"]), sim_out)

    stack.seed(seed)
    module = stack.load_module(J19A_MODULE_DIR)
    # RLModule.from_checkpoint FAILS SILENTLY into an empty shell. class_and_ctor_args.pkl names
    # asymmetric_rl_module; if that module is not importable, ray's checkpoint loader swallows
    # the ModuleNotFoundError, falls back to a bare RLModule and returns it with NO weights and
    # NO exception. A stage that did not check would roll out an empty network and never be
    # told. This is measured, not assumed.
    loaded_state = module.get_state() if hasattr(module, "get_state") else {}
    if not loaded_state:
        raise J19BError(
            f"cell {cid}: the loaded module carries NO state. RLModule.from_checkpoint returned "
            f"an empty shell, which is what happens when asymmetric_rl_module is not importable: "
            f"ray swallows the ModuleNotFoundError and falls back to a bare RLModule. Nothing "
            f"was rolled out.")
    module_class = type(module).__name__
    if module_class != EXPECTED_MODULE_CLASS:
        raise J19BError(f"cell {cid}: the loaded module is a {module_class}, not a "
                       f"{EXPECTED_MODULE_CLASS}; the checkpoint loader fell back to a base class")
    loaded_keys = tuple(sorted(str(k) for k in loaded_state))
    if loaded_keys != EXPECTED_STATE_KEYS:
        raise J19BError(f"cell {cid}: the loaded module holds {loaded_keys}, expected "
                       f"{EXPECTED_STATE_KEYS}")
    # and the weights it loaded must BE the pinned ones, not merely the right shape
    with (J19A_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        on_disk = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    mismatched = sorted(k for k in EXPECTED_STATE_KEYS
                        if not np.array_equal(np.asarray(loaded_state[k]), on_disk[k]))
    if mismatched:
        raise J19BError(f"cell {cid}: the loaded weights differ from the pinned module_state.pkl "
                       f"on {mismatched}")
    env = stack.make_env(env_config)
    env_closed = False
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=seed)
        contract_report = J3._verify_runtime_contract(base, module, obs,
                                                      tuple(feature_names_expected), cfg)
        feature_names = tuple(contract_report["actor_feature_names"])
        if WAIT_HS_FEATURE not in feature_names:
            raise J19BError(f"cell {cid}: the runtime actor exposes no {WAIT_HS_FEATURE}")
        wait_hs_idx = int(list(feature_names).index(WAIT_HS_FEATURE))
        wait_hs_rows = 0
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        # THE RESET CHECK, immediately after reset and never from a post-step record.
        # episode_start_offset_s is RELATIVE to cfg.t_start; the expected absolute time is
        # computed with the production semantics, clamp included.
        #
        # Every input to that arithmetic is taken from the LIVE environment and guarded. Reading
        # them off our own env_config would prove nothing about what the environment actually did.
        sim_cfg = getattr(base, "cfg", None)
        if sim_cfg is None:
            raise J19BError(f"cell {cid}: the environment exposes no .cfg; the reset check cannot "
                          "be performed and this stage refuses to proceed without it")
        for attr in ("t_start", "t_end"):
            if not hasattr(sim_cfg, attr):
                raise J19BError(f"cell {cid}: the environment's cfg exposes no {attr}; "
                              "episode_start_offset_s is relative to t_start and cannot be "
                              "checked without it")
            value = getattr(sim_cfg, attr)
            if not isinstance(value, (int, float)) or not np.isfinite(float(value)):
                raise J19BError(f"cell {cid}: cfg.{attr} is {value!r}, not a finite number")
        env_cfg_live = getattr(base, "env_cfg", None)
        if env_cfg_live is None or not hasattr(env_cfg_live, "episode_duration"):
            raise J19BError(f"cell {cid}: the environment exposes no env_cfg.episode_duration; the "
                          "clamp cannot be reproduced and this stage refuses to guess it")
        live_duration = getattr(env_cfg_live, "episode_duration")
        if live_duration is not None:
            if not isinstance(live_duration, (int, float)) \
                    or not np.isfinite(float(live_duration)):
                raise J19BError(f"cell {cid}: env_cfg.episode_duration is {live_duration!r}, not a "
                              "finite number")
            live_duration = float(live_duration)
        # the LIVE value is NORMATIVE; our own config is only cross-checked against it
        declared_duration = env_config.get("episode_duration")
        if (live_duration is None) != (declared_duration is None):
            raise J19BError(f"cell {cid}: the live episode_duration is {live_duration!r} while the "
                          f"config declares {declared_duration!r}")
        if live_duration is not None \
                and abs(live_duration - float(declared_duration)) > 1e-12:
            raise J19BError(f"cell {cid}: the live episode_duration {live_duration} and the declared "
                          f"{float(declared_duration)} differ by more than 1e-12; the environment "
                          "is not running the episode length this stage configured")
        reset_expect = expected_reset_time(
            float(sim_cfg.t_start), float(sim_cfg.t_end), live_duration,
            float(cell["offset_s"]))
        reset_time = float(J1._finite(base.t, f"cell {cid}: reset time"))
        reset_error = abs(reset_time - reset_expect["expected_reset_time_s"])
        reset_report = {**reset_expect, "actual_reset_time_s": reset_time,
                        "reset_time_error_s": reset_error,
                        "tolerance_s": RESET_TIME_TOLERANCE_S,
                        "checked_immediately_after_reset": True,
                        "compared_against": "cfg.t_start + offset, clamped to max_start - NOT the "
                                            "offset itself",
                        "duration_sources": {
                            "live_env_cfg_episode_duration": live_duration,
                            "declared_env_config_episode_duration":
                                None if declared_duration is None else float(declared_duration),
                            "normative": "live_env_cfg_episode_duration",
                            "cross_check_tolerance_s": 1e-12,
                            "agree": True,
                            "why_live_is_normative": "the clamp is computed by the environment "
                                                     "from its OWN env_cfg; reading our config "
                                                     "instead would prove nothing about what the "
                                                     "environment actually did"},
                        "sources": {"t_start": "live base.cfg.t_start",
                                    "t_end": "live base.cfg.t_end",
                                    "episode_duration": "live base.env_cfg.episode_duration",
                                    "all_guarded_fail_closed": True}}
        if reset_error > RESET_TIME_TOLERANCE_S:
            raise J19BError(
                f"cell {cid}: the reset time {reset_time} differs from the expected "
                f"{reset_expect['expected_reset_time_s']} (= t_start {reset_expect['cfg_t_start_s']}"
                f" + offset {cell['offset_s']}, clamped={reset_expect['clamped']}) by "
                f"{reset_error} s, beyond {RESET_TIME_TOLERANCE_S}")

        held = None
        if mode == "stochastic_held":
            if getattr(stack, "held_stochastic_action", None) is None:
                raise J19BError("the stack exposes no held_stochastic_action helper")
            held = stack.held_normal(np.random.default_rng(seed), env.action_space.shape,
                                     NOISE_HOLD_STEPS)

        trace: list[dict[str, Any]] = []
        knee: list[float] = []
        ankle: list[float] = []
        pen: list[float] = []
        noise_rows: list[np.ndarray] = []
        clipped = 0
        parity_steps = 0
        max_abs_noise = 0.0

        for step in range(1, EXPECTED_STEPS + 1):
            obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
            actor_obs = obs_vec[:ACTOR_WIDTH]
            if mode == "deterministic":
                raw, mean, std, path = J3.deterministic_action(module, obs_vec, action_shape,
                                                               torch_mod=stack.torch)
                dev = float(np.max(np.abs(np.asarray(raw, dtype=np.float64)
                                          - np.asarray(mean, dtype=np.float64))))
                max_abs_noise = max(max_abs_noise, dev)
                if dev != 0.0:
                    raise J19BError(f"cell {cid} step {step}: the deterministic action differs from "
                                  f"the policy mean by {dev}; noise is forbidden in this cell")
                if getattr(stack, "reference_action", None) is not None:
                    r_a, r_m, r_s, _ = stack.reference_action(module, obs_vec, action_shape)
                    if not (np.array_equal(raw, r_a) and np.array_equal(mean, r_m)
                            and np.array_equal(std, r_s)):
                        raise J19BError(f"cell {cid} step {step}: the deterministic action does not "
                                      "match rollout_eval's own helper bit for bit")
                    parity_steps += 1
                applied_noise = np.zeros(np.shape(raw), dtype=np.float32)
            else:
                unit = np.asarray(held.next(), dtype=np.float32).reshape(action_shape)
                raw, mean, std, applied_noise = stack.held_stochastic_action(
                    module, obs_vec, action_shape, unit)
                raw = np.asarray(raw, dtype=np.float32)
                mean = np.asarray(mean, dtype=np.float32)
                std = np.asarray(std, dtype=np.float32)
                applied_noise = np.asarray(applied_noise, dtype=np.float32)
                d_a, d_m, d_s, _ = J3.deterministic_action(module, obs_vec, action_shape,
                                                          torch_mod=stack.torch)
                if not np.array_equal(np.asarray(d_m, dtype=np.float32), mean):
                    raise J19BError(f"cell {cid} step {step}: forward_exploration and "
                                  "forward_inference disagree on the policy mean")
                if not np.array_equal(np.asarray(d_s, dtype=np.float32), std):
                    raise J19BError(f"cell {cid} step {step}: the two forward paths disagree on the "
                                  "std")
                if float(np.max(np.abs(std.astype(np.float64) - SIGMA))) > SIGMA_TOLERANCE:
                    raise J19BError(f"cell {cid} step {step}: the effective std is {std.tolist()}, "
                                  f"expected {SIGMA}")
                if not np.array_equal(applied_noise, (std * unit).astype(np.float32)):
                    raise J19BError(f"cell {cid} step {step}: applied noise is not std * unit_noise")
                parity_steps += 1
                noise_rows.append(np.asarray(applied_noise, dtype=np.float64).reshape(-1).copy())
                path = "rollout_eval._held_stochastic_action"

            applied = np.clip(raw, low.reshape(np.shape(raw)), high.reshape(np.shape(raw))
                              ).astype(np.float32)
            was_clipped = bool(np.any(applied != raw))
            clipped += int(was_clipped)
            if float(actor_obs[wait_hs_idx]) == 1.0:
                wait_hs_rows += 1
            pros = J1._prosthetic_state(actor_obs, feature_names)
            knee.append(pros["pros_knee_angle"])
            ankle.append(pros["pros_ankle_angle"])
            t_before = J1._finite(base.t, f"cell {cid} step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(raw)

            if "time" not in info:
                raise J19BError(f"cell {cid} step {step}: info exposes no 'time'")
            terms = J1._jsonable(info.get("reward_terms", {}), "reward_terms")
            if "grf_penetration_m" not in terms:
                raise J19BError(f"cell {cid} step {step}: reward_terms carries no "
                              "grf_penetration_m")
            pen.append(float(terms["grf_penetration_m"]))
            row: dict[str, Any] = {
                "step": step, "cell": cid,
                "time_before": t_before,
                "time_after": J1._finite(info["time"], f"cell {cid} step {step}: info.time"),
                "reward": J1._finite(reward, f"cell {cid} step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "raw_action": np.asarray(raw, dtype=float).reshape(-1).tolist(),
                "policy_mean": np.asarray(mean, dtype=float).reshape(-1).tolist(),
                "policy_std_diagnostic": np.asarray(std, dtype=float).reshape(-1).tolist(),
                "action_noise": np.asarray(applied_noise, dtype=float).reshape(-1).tolist(),
                "applied_action_diagnostic": np.asarray(applied, dtype=float).reshape(-1).tolist(),
                "action_clipped_diagnostic": was_clipped,
                "action_selection_path": path,
                "stepped_with": "raw_action",
                "reward_terms": terms,
                J1.FSM_KEY: J1._jsonable(info.get(J1.FSM_KEY), J1.FSM_KEY),
                "prosthetic_state": pros,
            }
            for extra in ("morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                          "online_grf", "online_grf_detector"):
                if extra in info:
                    row[extra] = J1._jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: J1._jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", J1.FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            trace.append(row)
            if progress and (step % 50 == 0 or step == 1):
                print(json.dumps({"cell": cid, "step": step, "reward": row["reward"]}), flush=True)
            if terminated or truncated:
                break
    except BaseException:
        # a PRIMARY exception is never masked - but the environment is still closed
        try:
            env.close()
            env_closed = True
        except Exception:                                       # pragma: no cover
            env_closed = False
        raise
    else:
        # FAIL-CLOSED: with no primary exception, a failed close is itself a technical failure.
        # A leaked environment means the evidence was produced by a process we did not shut down.
        # The original cause is CHAINED, never swallowed and never replaced.
        #
        # `Exception`, deliberately NOT `BaseException`: a RuntimeError from close becomes a
        # chained J19BError, but KeyboardInterrupt and SystemExit are NOT exceptions of the run -
        # they are the operator stopping it. Rewriting those as a stage error would misreport an
        # interrupt as a technical failure of the environment, so they propagate untouched. The
        # outer cleanup catches BaseException and therefore still removes the staging and the
        # lock on an interrupt: fail-closed either way.
        try:
            env.close()
        except Exception as exc:
            raise J19BError(f"cell {cid}: the environment failed to close cleanly, so the cell is "
                          f"technically invalid and nothing is committed: {exc!r}") from exc
        env_closed = True

    end_reason = J1._resolve_end_reason(trace)
    knee_arr = np.asarray(knee, dtype=np.float64)
    ankle_arr = np.asarray(ankle, dtype=np.float64)
    pen_arr = np.asarray(pen, dtype=np.float64)
    if noise_rows:
        rms = [float(v) for v in np.sqrt(np.mean(np.asarray(noise_rows) ** 2, axis=0))]
    else:
        rms = [0.0, 0.0]
    summary = J1._summarise(trace, end_reason, clipped, rms)
    penetration = penetration_report(pen_arr, contract, f"J19B cell {cid}")
    gate = evaluate_cell_gate(summary, knee_arr, ankle_arr, penetration)
    integrity = J3.telemetry_integrity(summary)
    verdict = cell_verdict(gate, integrity)

    # EXACTLY 19 regular files. A non-empty directory is NOT enough: 18 would mean a writer
    # failed silently and 19 was never produced, 20 would mean something else wrote here.
    # J9R1 accepted any non-empty directory; J10R1 hardened this and J19B inherits the hardening.
    if not sim_out.is_dir():
        raise J19BError(f"cell {cid}: the sim_outputs directory is missing at {sim_out}")
    entries = sorted(sim_out.iterdir())
    sim_files = sorted((q for q in entries if q.is_file() and not q.is_symlink()),
                       key=lambda q: q.name)
    non_files = sorted(q.name for q in entries if not (q.is_file() and not q.is_symlink()))
    if non_files:
        raise J19BError(f"cell {cid}: sim_outputs holds non-regular entries {non_files}")
    if len(sim_files) != EXPECTED_SIM_OUTPUT_FILES:
        raise J19BError(
            f"cell {cid}: sim_outputs holds {len(sim_files)} regular files, expected EXACTLY "
            f"{EXPECTED_SIM_OUTPUT_FILES}. Names: {[q.name for q in sim_files]}")

    trace_path = staging / f"j19b_cell_{cid}_trace.json"
    trace_path.write_text(json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
    kin_path = staging / f"j19b_cell_{cid}_kinematics.npz"
    np.savez_compressed(kin_path, knee_rad=knee_arr, ankle_rad=ankle_arr,
                        actor_feature_names=np.asarray(feature_names, dtype=str))
    pen_path = staging / f"j19b_cell_{cid}_penetration.npz"
    np.savez_compressed(pen_path, penetration_m=pen_arr)

    return {
        "id": cid, "mode": mode, "seed": seed, "label": str(cell["label"]),
        "episode_start_offset_s": float(cell["offset_s"]), "offset_unit": OFFSET_UNIT,
        "env_mutation": mutation,
        "env_config_sha256": _sha_obj(env_config),
        "reset_check": reset_report,
        "runtime_contract": contract_report,
        "module_load_check": {
            "class": module_class, "expected_class": EXPECTED_MODULE_CLASS,
            "state_keys": len(loaded_keys),
            "weights_match_pinned_module_state": True,
            "why_checked": ("ray's checkpoint loader swallows a ModuleNotFoundError and returns "
                            "a weightless base RLModule without raising; an unchecked stage "
                            "would roll out an empty network in silence")},
        "action_semantics": {
            "mode": mode, "path": path, "stepped_with": "raw_action",
            "clipping_is_diagnostic": True,
            "rollout_eval_parity_steps": parity_steps,
            "max_abs_action_minus_mean": max_abs_noise if mode == "deterministic" else None,
            "noise_hold_steps": NOISE_HOLD_STEPS if mode == "stochastic_held" else None,
            "sigma": SIGMA if mode == "stochastic_held" else None},
        "summary": summary,
        # EVERY criterion the architect asked to see, surfaced per cell rather than buried in
        # the summary. The values ARE the summary's; this block only makes them visible.
        "telemetry": {
            "phase_timeout_stance": summary["phase_timeout_stance"],
            "phase_timeout_swing": summary["phase_timeout_swing"],
            "morphology_causal_contract_failure":
                summary["morphology_causal_contract_failure"],
            "resync_count": summary["resync_count"],
            "hs_cancelled_count": summary["hs_cancelled_count"],
            "valid_cycle_count": summary["valid_cycle_count"],
            "valid_hs_count": summary["valid_hs_count"],
            "valid_to_count": summary["valid_to_count"],
            "steps": summary["steps"],
            "end_reason": summary["end_reason"]},
        "wait_hs": {
            "feature": WAIT_HS_FEATURE,
            "resolved_by": "name, from the LIVE RUNTIME actor feature names",
            "index": wait_hs_idx,
            "rows": int(wait_hs_rows),
            "fraction_of_steps": float(wait_hs_rows) / float(len(trace)) if trace else 0.0,
            "binding": WAIT_HS_IS_BINDING,
            "why_not_binding": ("this stage reuses J9R1's criteria exactly and invents none. "
                                "WAIT_HS is surfaced because J9R1 cell B failed with the FSM "
                                "held in WAIT_HS for every step and zero cycles, so the count "
                                "is the fastest read on whether that failure mode recurred - "
                                "but attaching a threshold to it would be a NEW gate, and a new "
                                "gate would have to be preregistered, not smuggled in here.")},
        "kinematics": {"steps": int(knee_arr.size),
                       "knee_min_rad": float(knee_arr.min()), "knee_max_rad": float(knee_arr.max()),
                       "ankle_min_rad": float(ankle_arr.min()),
                       "ankle_max_rad": float(ankle_arr.max())},
        "penetration": penetration,
        "gate": gate, "telemetry_integrity": integrity, "verdict": verdict,
        "behavioural_pass": bool(gate["pass"]),
        "telemetry_valid": bool(integrity["pass"]),
        "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
        "env_closed": env_closed,
        "sim_outputs": _leaf_rel(sim_out, staging),
        "sim_outputs_regular_file_count": len(sim_files),
        "sim_outputs_expected_count": EXPECTED_SIM_OUTPUT_FILES,
        "sim_outputs_count_is_exact": True,
        "sim_outputs_file_names": [q.name for q in sim_files],
        "paths_are_leaf_relative": True,
        "artefacts": {"trace": _leaf_rel(trace_path, staging),
                      "kinematics": _leaf_rel(kin_path, staging),
                      "penetration": _leaf_rel(pen_path, staging)},
        "outputs_sha256": {_leaf_rel(q, staging): _sha_file(q)
                           for q in (trace_path, kin_path, pen_path)},
        "sim_outputs_sha256": {_leaf_rel(q, staging): _sha_file(q) for q in sim_files},
        "content_hashes": {"knee_rad": _sha_array(knee_arr), "ankle_rad": _sha_array(ankle_arr),
                           "penetration_m": _sha_array(pen_arr)},
    }


# ================================================================ the matrix =====================

def run_matrix(out_arg: str | None, stage_token: str | None, *, stack: Any = None,
               progress: bool = True) -> dict[str, Any]:
    """All six cells, in the frozen order, with NO behavioural fail-fast."""
    if OUTPUT_ROOT_OVERRIDE is not None and stack is None:
        raise J19BError(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. It is permitted "
                      f"only for synthetic, isolated tests; the authorised matrix refuses it.")
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    pre = preflight()
    if pre["blockers"]:
        raise J19BError(f"preflight BLOCKED: {pre['blockers']}")

    contract = PC.load_contract()
    # GLUE: the J19A manifest carries no actor_feature_names. The 35 names are
    # resolved from three independent PINNED sources that must agree, BEFORE any
    # environment is constructed. Nothing else in this function changes.
    expected_features = tuple(resolve_feature_names()["actor_feature_names"])
    if len(expected_features) != ACTOR_WIDTH:
        raise J19BError(f"the pinned feature-name sources hold {len(expected_features)} names")
    actor_before = verify_actor()

    injected = stack is not None
    stack = stack if stack is not None else production_stack()

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J19BError(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    staging_created: Path | None = None
    commit_verified = False
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise J19BError(f"the J19B lock already exists: {lock_path}. This stage fails closed and "
                          f"removes no lock it does not own.") from exc
        lock_owned = lock_path
        try:
            os.write(fd, json.dumps({"stage": STAGE, "pid": os.getpid(),
                                     "leaf": str(leaf)}).encode("utf-8"))
        finally:
            os.close(fd)
        staging.mkdir()
        staging_created = staging

        # THE BASE IS BUILT HERE, with the STAGING as its output root, BEFORE any environment is
        # created or used. That is what gives every cell record_outputs True,
        # save_outputs_on_close True and an output_prefix - the instrumentation J9 lacked.
        base_env, cfg = base_env_config(staging)
        # DEFENCE IN DEPTH, at the call site and independent of base_env_config: the base that is
        # actually about to be used must carry recording ON. This fires BEFORE any environment is
        # constructed, so a recording-off base can never again be discovered after a 500-step run.
        for flag in ("record_outputs", "save_outputs_on_close"):
            if base_env.get(flag) is not True:
                raise J19BError(f"the base env config about to be used has {flag}="
                              f"{base_env.get(flag)!r}, expected exactly True. Refusing BEFORE "
                              "constructing any environment; this is the J9 defect and it is "
                              "caught here, not after the run.")
        if not base_env.get("output_prefix") or not base_env.get("output_dir"):
            raise J19BError("the base env config about to be used carries no output_prefix or no "
                          "output_dir; refusing before any environment is constructed")

        cells: list[dict[str, Any]] = []
        for cell in MATRIX:
            # NO behavioural fail-fast: every cell runs. A technical exception still fails closed.
            cells.append(run_cell(cell, stack=stack, base_env=base_env, cfg=cfg,
                                  contract=contract, staging=staging,
                                  feature_names_expected=expected_features, progress=progress))

        actor_after = verify_actor()
        if actor_after["artefacts_sha256"] != actor_before["artefacts_sha256"]:
            raise J19BError("the J19A actor changed during the matrix")

        behavioural = sum(1 for c in cells if c["behavioural_pass"])
        valid = sum(1 for c in cells if c["telemetry_valid"])
        aggregate_pass = behavioural == len(MATRIX) and valid == len(MATRIX)
        verdict = ("PASS" if aggregate_pass
                   else ("INVALID" if valid != len(MATRIX) else "FAIL"))

        receipt = {
            "schema": "v26c_j19b_closed_loop_receipt.1", "stage": STAGE,
            "verdict": verdict, "aggregate_pass": aggregate_pass,
            "cells_behavioural_pass": behavioural, "cells_telemetry_valid": valid,
            "cells_total": len(MATRIX),
            "aggregate_rule": "PASS iff 6/6 behavioural PASS AND 6/6 telemetry-valid",
            "stack": {"name": getattr(stack, "name", "?"),
                      "operational": bool(getattr(stack, "operational", False)),
                      "injected": injected,
                      "note": ("an INJECTED stack is a test double: this receipt is NOT "
                               "operational evidence" if injected
                               else "the production torch/RLlib/OpenSim stack")},
            "preregistration": pre["preregistration"],
            "actor_before": actor_before, "actor_after": actor_after,
            "actor_unchanged": True,
            "offset_unit_correction": pre["offset_unit_correction"],
            "scientific_equivalence_to_j9r1": pre["scientific_equivalence_to_j9r1"],
            "runtime": pre["runtime"],
            "recording_instrumentation": {
                "record_outputs": base_env["record_outputs"],
                "save_outputs_on_close": base_env["save_outputs_on_close"],
                "output_prefix": base_env["output_prefix"],
                # NOT the staging path: it ceases to exist at the commit. The committed leaf IS
                # the base output root, so the honest record is the leaf itself.
                "base_output_root": RELATIVE_LEAF,
                "base_output_root_is": "the committed leaf; each cell's sim_outputs sits inside it",
                "verified_fail_closed_before_any_env": True,
                "inherited_by_every_cell": True,
                "corrects": "the J9 defect recorded in v26c_j9_technical_failure_2026-08-26.json, inherited already corrected from J9R1",
                "supersedes_operationally_not_by_edit": {
                    "v26c_j9_prereg_closed_loop.json": J9_PREREG_SHA,
                    "v26c_j9_closed_loop.py": J9_RUNNER_SHA,
                    "test_v26c_j9_closed_loop.py": J9_TEST_SHA,
                    "v26c_j9_closed_loop_authorization.json": J9_AUTHORIZATION_SHA,
                    "v26c_j9_technical_failure_2026-08-26.json": J9_FAILURE_RECORD_SHA,
                    "note": "pinned, never edited"}},
            "env_mutation_policy": dict(ENV_MUTATION_POLICY),
            "penetration_authority": pre["penetration_authority"],
            "gate_specification": pre["gate_specification"],
            "matrix_policy": pre["matrix_policy"],
            "reset_time_check": pre["reset_time_check"],
            "multistart_disambiguation": pre["multistart_disambiguation"],
            "cells": cells,
            "inert": {"fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                      "actor_edited": False, "actor_copied": False,
                      "logstd_head_edited": False,
                      "rllib_checkpoint_loader": True,
                      "ray_cluster_started": False, "ray_workers_started": False,
                      "env_runners": False},
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "single_execution": True,
                        "no_autonomous_retry": True,
                        "note": "a closed-loop PASS here authorises nothing further"},
            "forbidden_here": list(FORBIDDEN_HERE),
            # every file this run produced, hashed by LEAF-RELATIVE path, so the post-commit
            # verification has something to re-resolve and re-check
            "committed_files_sha256": {
                _leaf_rel(q, staging): _sha_file(q)
                for q in sorted(staging.rglob("*")) if q.is_file() and not q.is_symlink()},
            "commit_verification": {
                "file": COMMIT_VERIFICATION_NAME,
                "state_when_this_receipt_was_written": "PENDING",
                "validity_rule": (f"this leaf is VALID EVIDENCE if and only if "
                                  f"{COMMIT_VERIFICATION_NAME} exists beside this receipt and "
                                  f"declares pass true"),
                "marker_on_failure": TECHNICAL_INVALID_NAME,
                "j9r1_had_none": "J9R1 recorded its receipt hash after the rename and compared "
                                 "it to nothing; this stage verifies the commit"},
        }
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False, default=str) + "\n",
            encoding="utf-8")

        staging_receipt_sha = _sha_file(staging / RECEIPT_NAME)

        # THE LEAF IS BORN INVALID. Written into the staging, so it is committed BY the rename:
        # there is no instant at which a leaf exists without saying it is unverified.
        (staging / TECHNICAL_INVALID_NAME).write_text(
            "TECHNICALLY INVALID - UNVERIFIED\n"
            f"stage: {STAGE}\n"
            f"see: {COMMIT_VERIFICATION_NAME}\n"
            "This marker is written BEFORE the commit and removed only after the post-commit "
            "verification has passed. While it is present the leaf is NOT valid evidence.\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J19BError(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None

        marker = leaf / TECHNICAL_INVALID_NAME
        if not marker.is_file():
            raise J19BError(f"the committed leaf does not carry {TECHNICAL_INVALID_NAME}: "
                           f"{leaf} is not the directory this run staged")
        try:
            verification = verify_committed_leaf(leaf, expected_receipt_sha=staging_receipt_sha)
        except Exception as exc:
            # BaseException is deliberately NOT caught: a Ctrl-C must propagate with the leaf
            # left marked invalid, not be recorded as a verification verdict.
            verification = {"schema": "v26c_j19b_commit_verification.1", "stage": STAGE,
                            "pass": False, "verifier_error": f"{type(exc).__name__}: {exc}",
                            "meaning": "verification could not complete; the leaf is TECHNICALLY "
                                       "INVALID and is preserved unpromoted"}
        try:
            (leaf / COMMIT_VERIFICATION_NAME).write_text(
                json.dumps(verification, indent=2, ensure_ascii=False, allow_nan=False,
                           default=str) + "\n", encoding="utf-8")
        except OSError as exc:
            raise J19BError(f"the verification could not be recorded in {leaf}: "
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
            raise J19BError(
                f"POST-COMMIT VERIFICATION FAILED for {leaf}: "
                f"{len(verification.get('paths_missing') or [])} paths did not resolve, "
                f"{len(verification.get('hash_mismatches') or [])} hashes did not reproduce.")
        try:
            marker.unlink()
        except OSError as exc:
            raise J19BError(f"the verification passed but the marker could not be removed from "
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

    return {"verdict": verdict, "stage": STAGE, "leaf": _rel(leaf),
            "aggregate_pass": aggregate_pass,
            "cells": [{"id": c["id"], "mode": c["mode"], "seed": c["seed"],
                       "offset_s": c["episode_start_offset_s"], "verdict": c["verdict"],
                       "failed": c["gate"]["failed"],
                       "max_penetration_m": c["penetration"]["max_penetration_m"],
                       "band": c["penetration"]["band"]} for c in cells],
            "receipt_sha256": _sha_file(leaf / RECEIPT_NAME),
            "commit_verification": {"file": COMMIT_VERIFICATION_NAME, "pass": commit_verified,
                                    "files_checked": verification.get("files_checked"),
                                    "rule": "the leaf is valid evidence only while this is true"},
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="V26C J19B closed-loop qualification of the J19A actor")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--run", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.run:
        r = run_matrix(a.out, a.authorized_stage, progress=not a.no_progress)
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "PASS" else 1
    if a.out is not None:
        raise J19BError("--out is meaningless without --run; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
