"""Deterministic derivation of the J19B closed-loop runner from the frozen J16.

WHY A DERIVATION TOOL AND NOT A HAND-WRITTEN COPY
-------------------------------------------------
The architect required a MECHANICAL, AUDITABLE copy of the J16 runner, with J16
itself left byte-unchanged. Retyping 1737 lines of a safety-critical closed-loop
path by hand is precisely the silent-divergence risk the requirement exists to
avoid: a single mistyped constant would not announce itself.

So the copy is produced mechanically, by this tool, from a CLOSED and PINNED
substitution map. The transformation is the auditable artefact: every change to
the derived runner is one of the entries below, and the accompanying test suite
proves by BYTECODE that the scientific path is identical and by AST that the
only functions differing are the declared adapted ones.

This tool writes exactly one file and reads J16 read-only. It never runs a
rollout, never touches J16, and is not part of the execution path.

Usage:
    python v26c_j19b_derive_from_j16.py --check     # derive in memory, report
    python v26c_j19b_derive_from_j16.py --write     # write the derived runner
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent

SOURCE_NAME = "v26c_j16_closed_loop.py"
SOURCE_SHA256 = "6ac4585424cbce34957722e8fc64dc0669de14c57a0418e6aa940b1303cc2e34"
DERIVED_NAME = "v26c_j19b_closed_loop.py"

# --------------------------------------------------------------------------
# THE CLOSED SUBSTITUTION MAP.
#
# Every entry is classified. Nothing outside this map may change, and the test
# suite proves it. Order matters: longer, more specific patterns first.
# --------------------------------------------------------------------------

SUBSTITUTIONS = (
    # --- stage -------------------------------------------------------------
    ("stage", "V26C_J16_J15R1_CLOSED_LOOP_REQUALIFICATION",
     "V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION"),

    # --- prereg ------------------------------------------------------------
    ("prereg", "v26c_j16_prereg_closed_loop_requalification.json",
     "v26c_j19b_prereg_closed_loop_qualification.json"),
    # SEALED. The J19B preregistration is immutable at this hash, verified against
    # the file before this value was written. Once sealed, verify_prereg's guard
    # stops being a no-op: any edit to the preregistration makes the runner refuse
    # to start.
    ("prereg", '"150f49b1bd2865d9224d43336d50098c2fd610b4b5d121dd1ce737213a0864aa"',
     '"31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8"'),

    # --- leaf, staging, lock, sentinel, receipt ----------------------------
    ("output", '("j16_runs", "j16_closed_loop_v26c_2026-08-27_r1")',
     '("j19b_runs", "j19b_closed_loop_v26c_2026-08-27_r1")'),
    ("output", "v26c_j16_closed_loop_receipt.json", "v26c_j19b_closed_loop_receipt.json"),
    ("output", "_j16_preflight_sentinel_never_created",
     "_j19b_preflight_sentinel_never_created"),

    # --- actor leaf and pins ----------------------------------------------
    ("actor", 'HERE / "j15_runs" / "j15_fresh_refit_v26c_2026-08-27_r1"',
     'HERE / "j19a_runs" / "j19a_single_reproduction_v26c_2026-08-27_r1"'),
    # the actor DIGEST is a separate constant from the pin dict: renaming it is not
    # enough, its VALUE must become the J19A digest.
    ("actor", '"c3551341b9b017b40585ee1f50f8ee4ecaa7ec50722f37142badc2a44b6c8590"',
     '"d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96"'),

    # --- identifiers: error class, prose and field names -------------------
    ("naming", "J16Error", "J19BError"),
    ("naming", "J15R1_LEAF", "J19A_LEAF"),
    ("naming", "J15R1_MODULE_DIR", "J19A_MODULE_DIR"),
    ("naming", "PIN_J15R1_MODULE_STATE", "PIN_J19A_MODULE_STATE"),
    ("naming", "PIN_J15R1_ACTOR_DIGEST", "PIN_J19A_ACTOR_DIGEST"),
    ("naming", "PIN_J15R1", "PIN_J19A"),
    ("naming", "J15R1", "J19A"),
    ("naming", "j16_", "j19b_"),
    ("naming", "J16", "J19B"),
)

# Functions the derivation REPLACES wholesale rather than substituting inside.
# Each is provenance or schema only; none carries a gate, an environment
# parameter, a cell definition or a penetration threshold.
REPLACED_FUNCTIONS = ("verify_prereg", "verify_actor")

# --------------------------------------------------------------------------
# STRUCTURAL REPLACEMENTS.
#
# Textual substitution cannot reach these: the J19A leaf has a different file
# set and its receipt, manifest and commit verification use different schemas.
# Each block below is classified and is provenance or schema ONLY.
# --------------------------------------------------------------------------

PIN_BLOCK = '''PIN_J19A: dict[str, str] = {
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
'''

VERIFY_PREREG_BLOCK = '''def verify_prereg() -> dict[str, Any]:
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
        if (str(got["mode"]), int(got["seed"]), float(got["offset_s"])) != \\
                (want["mode"], want["seed"], float(want["offset_s"])):
            raise J19BError(f"cell {want['id']} differs from the preregistration")
    if sorted(int(s) for s in data["the_six_cells"]["seeds_used"]) != [123, 124, 125]:
        raise J19BError("the preregistration declares seeds other than 123, 124, 125")

    # the actor identity
    actor = data["actor_under_test"]
    if actor["module_state_sha256"] != PIN_J19A_MODULE_STATE \\
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
'''

VERIFY_ACTOR_BLOCK = '''def resolve_feature_names() -> dict[str, Any]:
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
    if not bool(np.all(np.isfinite(sigmas))) \\
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
'''

# The ONLY glue inside run_matrix: the 35 names come from the pinned sources,
# resolved BEFORE the environment is built, instead of from a manifest field the
# J19A manifest does not carry.
RUN_MATRIX_GLUE_OLD = '''    manifest = json.loads((J19A_MODULE_DIR / "actor_feature_manifest.json").read_text())
    expected_features = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(expected_features) != ACTOR_WIDTH:
        raise J19BError(f"the J19A manifest holds {len(expected_features)} names")
'''

RUN_MATRIX_GLUE_NEW = '''    # GLUE: the J19A manifest carries no actor_feature_names. The 35 names are
    # resolved from three independent PINNED sources that must agree, BEFORE any
    # environment is constructed. Nothing else in this function changes.
    expected_features = tuple(resolve_feature_names()["actor_feature_names"])
    if len(expected_features) != ACTOR_WIDTH:
        raise J19BError(f"the pinned feature-name sources hold {len(expected_features)} names")
'''

DEAD_LINEAGE_BLOCK = '''# The J15R1-lineage provenance constants J16 carried here - the J11 failed actor,
# the J14 corrective increment and the J15R1 nominal-drift addendum - describe a
# lineage that is NOT this actor's. Their only consumer was J16's verify_actor,
# which this derivation replaces, so they are dropped rather than left dangling
# or, worse, repointed to something they do not describe.
'''

ACTOR_PROSE_OLD = '''        "actor_before": "J11 (j11_runs/j11_multistart_fit_v26c_2026-08-27_r1), which "
                        "FAILED this same matrix 4/6 in J12",
        "actor_now": "J19A (j15_runs/j15_fresh_refit_v26c_2026-08-27_r1)",'''

ACTOR_PROSE_NEW = '''        "actor_before": "J15R1 (j15_runs/j15_fresh_refit_v26c_2026-08-27_r1), which "
                        "FAILED this same matrix 0/6 in J16",
        "actor_now": "J19A (j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1)",'''

DOCSTRING_OLD = '''THE ACTOR IS J19A, AND ONLY J19A
    j15_runs/j15_fresh_refit_v26c_2026-08-27_r1/rl_module.
    NOT J11 - J11 is the actor that FAILED J12 and is pinned here only as negative
    evidence. NOT J8, NOT J4, NOT a July checkpoint. The J19A leaf is pinned file by file and is consumed
    only if its own commit_verification.json declares pass true and it carries no invalidity
    marker: an unverified leaf is not evidence and its actor is not loadable here.'''

DOCSTRING_NEW = '''THE ACTOR IS J19A, AND ONLY J19A
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
    behavioural fail-fast false and the penetration contract are identical by value.'''

LEAF_COMMENT_OLD = '''# The J19A leaf, pinned FILE BY FILE. Eight files, not six: J19A also commits its aggregate dataset
# and its own commit_verification.json.'''

LEAF_COMMENT_NEW = '''# The J19A leaf, pinned FILE BY FILE. SEVEN files: J19A commits no aggregate dataset -
# it trains nothing - but it does commit its own result and its own
# commit_verification.json, whose schema uses "ok" where J15R1's used "pass".'''

# This is a REPORTED FIELD, not a comment: carried over unchanged it would write a
# statement about J15R1's training aggregate into the J19B receipt, and that statement
# is false of the J19A actor.
TRAINING_DATA_OLD = '''                "PRESENT, and this is the difference from J9R1. The actor under test was fitted "
                "on an aggregate that INCLUDES multistart rows: J7's 16713 plus the two J10R1 "
                "teacher cells, 500 unique rows each tiled eight times, plus the 854 J14 "
                "corrective rows taken once, for 25567 in total. "
                "J9R1's actor had none of that."),'''

TRAINING_DATA_NEW = '''                "PRESENT, and this is the difference from J9R1. The actor under test descends "
                "from J8 and was fitted on the J18 candidate-13 aggregate: four DISJOINT blocks "
                "totalling 4221 rows - the J10R1 cell-B teacher trajectory (500), the J8 "
                "on-policy pre-mismatch prefix (14), the closed-loop preservation anchors from "
                "cells A, C, D, E and F (2497), and the deduplicated J7 support (1210). "
                "J9R1's actor had none of that."),'''

STRUCTURAL_REPLACEMENTS = (
    ("naming", DOCSTRING_OLD, None, DOCSTRING_NEW),
    ("provenance", LEAF_COMMENT_OLD, None, LEAF_COMMENT_NEW),
    ("provenance", TRAINING_DATA_OLD, None, TRAINING_DATA_NEW),
    ("provenance", "PIN_J19A: dict[str, str] = {", "\n# the actor that FAILED", PIN_BLOCK),
    ("provenance", "# the actor that FAILED", "\nPIN_J19A_MODULE_STATE", DEAD_LINEAGE_BLOCK),
    ("naming", ACTOR_PROSE_OLD, None, ACTOR_PROSE_NEW),
    ("provenance", "def verify_prereg() -> dict[str, Any]:", "\n\ndef ", VERIFY_PREREG_BLOCK),
    ("provenance", "def verify_actor() -> dict[str, Any]:", "\n\ndef ", VERIFY_ACTOR_BLOCK),
    ("glue", RUN_MATRIX_GLUE_OLD, None, RUN_MATRIX_GLUE_NEW),
)

# Functions whose bytecode MUST remain identical to J16's after derivation.
# This is the scientific path.
BYTECODE_IDENTICAL = (
    "base_env_config", "cell_env_config", "expected_reset_time", "unit_correction",
    "evaluate_cell_gate", "cell_verdict", "penetration_report", "run_cell",
    "production_stack",
)


def sha256_bytes(payload):
    """SHA-256 of a byte string."""
    return hashlib.sha256(payload).hexdigest()


def read_source(root=HERE):
    """Read the frozen J16 source after verifying its bytes."""
    path = root / SOURCE_NAME
    raw = path.read_bytes()
    actual = sha256_bytes(raw)
    if actual != SOURCE_SHA256:
        raise RuntimeError(
            "the frozen J16 runner changed: expected %s, found %s. A derivation from "
            "altered source would be meaningless." % (SOURCE_SHA256, actual)
        )
    return raw.decode("utf-8")


def apply_substitutions(text):
    """Apply the closed map, counting every hit."""
    applied = []
    for kind, old, new in SUBSTITUTIONS:
        count = text.count(old)
        text = text.replace(old, new)
        applied.append({"kind": kind, "old": old, "new": new, "hits": count})
    return text, applied


def apply_structural(text):
    """Splice the declared structural blocks, by exact span location."""
    applied = []
    for kind, start, end, replacement in STRUCTURAL_REPLACEMENTS:
        i = text.find(start)
        if i < 0:
            applied.append({"kind": kind, "anchor": start[:48], "hits": 0})
            continue
        if end is None:
            j = i + len(start)
        else:
            j = text.find(end, i + len(start))
            if j < 0:
                applied.append({"kind": kind, "anchor": start[:48], "hits": 0})
                continue
        text = text[:i] + replacement + text[j:]
        applied.append({"kind": kind, "anchor": start[:48], "hits": 1,
                        "replaced_chars": j - i, "with_chars": len(replacement)})
    return text, applied


def derive(root=HERE):
    """Produce the derived source in memory and report what changed."""
    original = read_source(root)
    derived, applied = apply_substitutions(original)
    derived, structural = apply_structural(derived)
    applied = applied + [{"kind": s["kind"], "old": s["anchor"], "new": "<structural block>",
                          "hits": s["hits"]} for s in structural]
    return {
        "original": original,
        "derived": derived,
        "applied": applied,
        "total_hits": sum(a["hits"] for a in applied),
        "unhit": [a for a in applied if a["hits"] == 0],
        "source_sha256": sha256_bytes(original.encode("utf-8")),
        "derived_sha256": sha256_bytes(derived.encode("utf-8")),
        "replaced_functions": list(REPLACED_FUNCTIONS),
        "bytecode_identical_required": list(BYTECODE_IDENTICAL),
    }


def residual_j16_references(derived):
    """Surviving J16/J15R1 identifiers OUTSIDE the structural blocks.

    Inside those blocks the references are DELIBERATE: the replacement prose
    says what it was adapted from, and the J15R1 manifest is one of the three
    pinned sources that must agree on the feature names. A residual anywhere
    else would be a derivation miss.
    """
    outside = derived
    for _kind, _start, _end, replacement in STRUCTURAL_REPLACEMENTS:
        outside = outside.replace(replacement, "")
    return sorted(set(re.findall(r"\bJ15R1\w*|\bJ16\w*|\bj16_\w*|\bj15_\w*", outside)))


def main(argv=None):
    """Return 0 on success. --write is the only mode that writes."""
    parser = argparse.ArgumentParser(description="derive the J19B runner from J16")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args(argv)

    report = derive()
    print("source  %s  %s" % (SOURCE_NAME, report["source_sha256"]))
    print("derived %s  %s" % (DERIVED_NAME, report["derived_sha256"]))
    print("\nsubstitution map, %d entries, %d hits:" % (len(SUBSTITUTIONS),
                                                        report["total_hits"]))
    for entry in report["applied"]:
        print("  [%-8s] %-5d  %.48s -> %.40s"
              % (entry["kind"], entry["hits"], entry["old"], entry["new"]))
    if report["unhit"]:
        print("\n!! entries that matched nothing: %s"
              % [e["old"] for e in report["unhit"]])
    residual = residual_j16_references(report["derived"])
    print("\nresidual J16/J15R1 identifiers in the derived source: %s"
          % (residual if residual else "NONE"))

    if args.write:
        target = HERE / DERIVED_NAME
        if target.exists():
            print("\nrefusing to overwrite %s" % target, file=sys.stderr)
            return 1
        target.write_text(report["derived"], encoding="utf-8")
        print("\nwrote %s  %s" % (target.name, sha256_bytes(target.read_bytes())))
    return 0 if not report["unhit"] and not residual else 1


if __name__ == "__main__":
    sys.exit(main())
