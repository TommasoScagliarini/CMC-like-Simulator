"""Deterministic derivation of the J19C held-out runner from the frozen J19B.

Same discipline as v26c_j19b_derive_from_j16.py, one stage further along: the
copy is produced mechanically from a CLOSED and PINNED substitution map, so the
transformation is the auditable artefact rather than a hand-retyped file. J19B
and J16 are both left byte-unchanged.

THE SEALED SEEDS
    126, 127 and 128 appear here for the first time in executable code, AFTER
    v26c_j19c_prereg_heldout_g_i.json declared them. Nothing is sampled, drawn
    or simulated by this tool; the seeds are written as integers into a matrix.

CORE SCIENTIFIC EQUIVALENCE
    Everything scientific stays identical to J19B. The authorised differences
    are a CLOSED WHITELIST - matrix 6 to 3, ids A-F to G-I, every mode
    stochastic_held, seeds 126/127/128, stage-specific strings and paths, and
    the aggregate rule 6/6 to 3/3. The derived runner asserts this at preflight
    and refuses anything outside it.

Usage:
    python v26c_j19c_derive_from_j19b.py --check
    python v26c_j19c_derive_from_j19b.py --write
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent

SOURCE_NAME = "v26c_j19b_closed_loop.py"
SOURCE_SHA256 = "327b8fe39edae290fd8ed806f6b34715313b8d82d7c08ba600054eb191bdcb19"
DERIVED_NAME = "v26c_j19c_heldout_g_i.py"

HELDOUT_SEEDS = (126, 127, 128)
PRIOR_SEEDS = (123, 124, 125)

SUBSTITUTIONS = (
    ("stage", "V26C_J19B_J19A_CLOSED_LOOP_QUALIFICATION", "V26C_J19C_J19A_HELDOUT_G_I"),

    ("prereg", "v26c_j19b_prereg_closed_loop_qualification.json",
     "v26c_j19c_prereg_heldout_g_i.json"),
    # SEALED, unconditionally. The J19C preregistration is immutable at this hash,
    # verified against the file before this value was written.
    ("prereg", '"31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8"',
     '"54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d"'),
    # How the policy is REPORTED is replaced at both call sites: the preflight and
    # the receipt. The pinned constant itself is never touched.
    ("policy_report", '"env_mutation_policy": dict(ENV_MUTATION_POLICY),',
     '"env_mutation_policy": env_mutation_report(),'),

    ("output", '("j19b_runs", "j19b_closed_loop_v26c_2026-08-27_r1")',
     '("j19c_runs", "j19c_heldout_g_i_v26c_2026-08-27_r1")'),
    ("output", "v26c_j19b_closed_loop_receipt.json", "v26c_j19c_heldout_g_i_receipt.json"),
    ("output", "_j19b_preflight_sentinel_never_created",
     "_j19c_preflight_sentinel_never_created"),

    ("aggregate", "PASS iff 6/6 behavioural PASS AND 6/6 telemetry-valid",
     "PASS iff 3/3 behavioural PASS AND 3/3 telemetry-valid"),
    # A SECOND, differently-worded copy of the same rule lives in the preflight
    # metadata under matrix_policy.aggregate_pass_iff. Substituting only the first
    # left J19C claiming 6/6 in its own preflight output.
    ("aggregate", '"aggregate_pass_iff": "6/6 behavioural PASS and 6/6 telemetry-valid"',
     '"aggregate_pass_iff": "3/3 behavioural PASS and 3/3 telemetry-valid"'),
    # Two docstrings describe the matrix that this stage no longer runs.
    ("naming", "sigma. 500 steps per cell, run sequentially in the order A-F.",
     "sigma. 500 steps per cell, run sequentially in the order G-I."),
    ("naming", '"""All six cells, in the frozen order, with NO behavioural fail-fast."""',
     '"""All three held-out cells, in the frozen order, with NO behavioural fail-fast."""'),

    ("naming", "J19BError", "J19CError"),
    ("naming", "j19b_", "j19c_"),
    ("naming", "J19B", "J19C"),
)

REPLACED_FUNCTIONS = ("verify_prereg", "verify_scientific_equivalence")

# --------------------------------------------------------------------------
# STRUCTURAL REPLACEMENTS
# --------------------------------------------------------------------------

MATRIX_OLD_ANCHOR = "MATRIX: tuple[dict[str, Any], ...] = ("
MATRIX_END = "\n)\n"

MATRIX_BLOCK = '''MATRIX: tuple[dict[str, Any], ...] = (
    {"id": "G", "mode": "stochastic_held", "seed": 126, "offset_s": OFFSET_NOMINAL,
     "label": "heldout nominal sigma 0.005"},
    {"id": "H", "mode": "stochastic_held", "seed": 127, "offset_s": OFFSET_NOMINAL,
     "label": "heldout nominal sigma 0.005"},
    {"id": "I", "mode": "stochastic_held", "seed": 128, "offset_s": OFFSET_NOMINAL,
     "label": "heldout nominal sigma 0.005"},
)
# The sealed held-out seeds, declared in the preregistration BEFORE this file
# existed. No prior seed may appear in the matrix.
HELDOUT_SEEDS = (126, 127, 128)
PRIOR_SEEDS = (123, 124, 125)
'''

VERIFY_PREREG_BLOCK = '''def verify_prereg() -> dict[str, Any]:
    """This stage's OWN preregistration, in the J19C schema.

    PROVENANCE ONLY. Every scientific field it checks is checked against the
    SAME runner constants, which the derivation leaves untouched.
    """
    if not PREREG.is_file():
        raise J19CError("the J19C preregistration is missing")
    digest = _sha_file(PREREG)
    # UNCONDITIONAL. There is no PENDING branch: the preregistration is sealed and
    # any divergence from its pin refuses the run.
    if digest != PIN_PREREG:
        raise J19CError(f"the J19C preregistration changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    if data.get("stage") != STAGE:
        raise J19CError(f"the preregistration declares {data.get('stage')!r}, not {STAGE}")

    declared = data["the_three_cells"]["cells"]
    if [c["id"] for c in declared] != [c["id"] for c in MATRIX]:
        raise J19CError("the preregistered matrix order and the runner's disagree")
    for want, got in zip(MATRIX, declared):
        if (str(got["mode"]), int(got["seed"]), float(got["offset_s"])) != \\
                (want["mode"], want["seed"], float(want["offset_s"])):
            raise J19CError(f"cell {want['id']} differs from the preregistration")
    if sorted(int(s) for s in data["the_three_cells"]["seeds_used"]) != sorted(HELDOUT_SEEDS):
        raise J19CError("the preregistration declares seeds other than the held-out set")
    if int(data["the_three_cells"]["steps_per_cell"]) != EXPECTED_STEPS \\
            or float(data["the_three_cells"]["sigma"]) != SIGMA \\
            or int(data["the_three_cells"]["noise_hold_steps"]) != NOISE_HOLD_STEPS:
        raise J19CError("the preregistration and the runner disagree on the noise contract")

    actor = data["actor_under_test"]
    if actor["module_state_sha256"] != PIN_J19A_MODULE_STATE \\
            or actor["actor_digest"] != PIN_J19A_ACTOR_DIGEST:
        raise J19CError("the preregistration names a different actor than the pins")

    # the entry evidence: J19B must have PASSED and its leaf must verify
    entry = data["entry_evidence_pinned"]
    leaf = HERE / entry["j19b_leaf"]
    if not leaf.is_dir():
        raise J19CError(f"the J19B leaf is missing: {leaf}")
    got_receipt = _sha_file(leaf / "v26c_j19b_closed_loop_receipt.json")
    if got_receipt != entry["j19b_receipt_sha256"]:
        raise J19CError(f"the J19B receipt changed: {got_receipt}")
    got_cv = _sha_file(leaf / "commit_verification.json")
    if got_cv != entry["j19b_commit_verification_sha256"]:
        raise J19CError(f"the J19B commit verification changed: {got_cv}")
    j19b = json.loads((leaf / "v26c_j19b_closed_loop_receipt.json").read_text())
    if j19b["verdict"] != "PASS":
        raise J19CError(f"J19B did not pass: {j19b['verdict']!r}. This phase runs only "
                        f"behind a PASSED J19B.")
    if j19b["cells_behavioural_pass"] != j19b["cells_total"] \\
            or j19b["cells_telemetry_valid"] != j19b["cells_total"]:
        raise J19CError("the J19B receipt does not record a full 6/6")
    cv = json.loads((leaf / "commit_verification.json").read_text())
    if cv.get("pass") is not True:
        raise J19CError("the J19B commit verification does not pass")

    if data["execution_discipline"].get("single_execution") is not True:
        raise J19CError("the preregistration must declare a single execution")
    return {"file": _rel(PREREG), "sha256": digest,
            "stage": data["stage"], "schema": "J19C",
            "entry_evidence": {"j19b_verdict": j19b["verdict"],
                               "j19b_cells": j19b["cells_total"],
                               "j19b_receipt_sha256": got_receipt,
                               "j19b_commit_verification_sha256": got_cv},
            "gates_are_j9r1s": True}
'''

VERIFY_EQUIV_BLOCK = '''def verify_scientific_equivalence() -> dict[str, Any]:
    """CORE SCIENTIFIC EQUIVALENCE to J19B, fail-closed, with a CLOSED WHITELIST.

    Architect decision: a cell-by-cell comparison against J9R1's A-F matrix is
    NOT forced, because J19C legitimately runs three held-out cells instead of
    six. What is asserted instead is that EVERYTHING scientific is identical to
    J19B, and that the ONLY differences are the six whitelisted ones. Anything
    else raises, and the preflight blocks.
    """
    import importlib.util as _ilu

    spec = _ilu.spec_from_file_location("_j19b_reference", HERE / "v26c_j19b_closed_loop.py")
    ref = _ilu.module_from_spec(spec)
    spec.loader.exec_module(ref)

    ref_sha = _sha_file(HERE / "v26c_j19b_closed_loop.py")
    if ref_sha != PIN_J19B_SOURCE:
        raise J19CError(f"the J19B source changed: {ref_sha} != {PIN_J19B_SOURCE}")

    def code_sig(fn: Any) -> str:
        import types as _t

        def flat(code: Any) -> str:
            out = [code.co_code, repr(code.co_varnames), code.co_argcount]
            for k in code.co_consts:
                out.append(flat(k) if isinstance(k, _t.CodeType) else repr(k))
            return repr(out)
        return hashlib.sha256(flat(fn.__code__).encode()).hexdigest()

    identical_functions = ("base_env_config", "cell_env_config", "expected_reset_time",
                           "unit_correction", "evaluate_cell_gate", "cell_verdict",
                           "penetration_report", "production_stack")
    drifted = [n for n in identical_functions
               if code_sig(getattr(ref, n)) != code_sig(globals()[n])]
    if drifted:
        raise J19CError(f"the scientific path drifted from J19B: {drifted}")

    scalars = {
        "COMMON_GATE": (ref.J19B_COMMON_GATE, J19C_COMMON_GATE),
        "KINEMATIC_GATE": (ref.J19B_KINEMATIC_GATE, J19C_KINEMATIC_GATE),
        "SIGMA": (ref.SIGMA, SIGMA),
        "SIGMA_TOLERANCE": (ref.SIGMA_TOLERANCE, SIGMA_TOLERANCE),
        "NOISE_HOLD_STEPS": (ref.NOISE_HOLD_STEPS, NOISE_HOLD_STEPS),
        "EXPECTED_STEPS": (ref.EXPECTED_STEPS, EXPECTED_STEPS),
        "RESET_TIME_TOLERANCE_S": (ref.RESET_TIME_TOLERANCE_S, RESET_TIME_TOLERANCE_S),
        "CLOCK_COLUMNS": (ref.CLOCK_COLUMNS, CLOCK_COLUMNS),
        "CONTROLLER_COLUMNS": (ref.CONTROLLER_COLUMNS, CONTROLLER_COLUMNS),
        "ACTOR_WIDTH": (ref.ACTOR_WIDTH, ACTOR_WIDTH),
        "FROZEN_OFFSETS": (ref.FROZEN_OFFSETS, FROZEN_OFFSETS),
        "OFFSET_NOMINAL": (ref.OFFSET_NOMINAL, OFFSET_NOMINAL),
        "ENV_MUTATION_POLICY": (dict(ref.ENV_MUTATION_POLICY), dict(ENV_MUTATION_POLICY)),
        "PIN_J19A": (dict(ref.PIN_J19A), dict(PIN_J19A)),
        "PIN_J19A_MODULE_STATE": (ref.PIN_J19A_MODULE_STATE, PIN_J19A_MODULE_STATE),
        "PIN_J19A_ACTOR_DIGEST": (ref.PIN_J19A_ACTOR_DIGEST, PIN_J19A_ACTOR_DIGEST),
        "PIN_FEATURE_NAME_SOURCES": (dict(ref.PIN_FEATURE_NAME_SOURCES),
                                     dict(PIN_FEATURE_NAME_SOURCES)),
    }
    mismatched = {k: v for k, v in scalars.items() if v[0] != v[1]}
    if mismatched:
        raise J19CError(f"core scientific constants differ from J19B: {sorted(mismatched)}")

    # ---- the CLOSED WHITELIST of authorised differences ---------------------
    if len(MATRIX) != 3 or len(ref.MATRIX) != 6:
        raise J19CError(f"W1 violated: {len(ref.MATRIX)} -> {len(MATRIX)}, expected 6 -> 3")
    if [c["id"] for c in MATRIX] != ["G", "H", "I"]:
        raise J19CError(f"W2 violated: ids are {[c['id'] for c in MATRIX]}")
    if any(c["mode"] != "stochastic_held" for c in MATRIX):
        raise J19CError("W3 violated: every held-out cell must be stochastic_held")
    if tuple(c["seed"] for c in MATRIX) != HELDOUT_SEEDS:
        raise J19CError(f"W4 violated: seeds are {[c['seed'] for c in MATRIX]}")
    if any(c["seed"] in PRIOR_SEEDS for c in MATRIX):
        raise J19CError("W4 violated: a prior seed appears in the held-out matrix")
    if any(float(c["offset_s"]) != float(OFFSET_NOMINAL) for c in MATRIX):
        raise J19CError("the held-out cells must all start at the frozen nominal offset")
    if AGGREGATE_RULE != "PASS iff 3/3 behavioural PASS AND 3/3 telemetry-valid":
        raise J19CError(f"W6 violated: aggregate rule is {AGGREGATE_RULE!r}")
    j19b_text = (HERE / "v26c_j19b_closed_loop.py").read_text()
    if J19B_AGGREGATE_RULE not in j19b_text:
        raise J19CError("the transcribed J19B aggregate rule does not occur in the J19B "
                        "source; the transcription is stale")
    if AGGREGATE_RULE in j19b_text:
        raise J19CError("the J19C aggregate rule already occurs in J19B; W6 is not a "
                        "real difference and the derivation is suspect")

    return {
        "kind": "CORE SCIENTIFIC EQUIVALENCE TO J19B, FAIL-CLOSED",
        "source_module": "v26c_j19b_closed_loop.py", "source_sha256": ref_sha,
        "bytecode_identical_functions": list(identical_functions),
        "constants_identical": sorted(scalars),
        "constants_compared": len(scalars),
        "authorised_differences": {
            "W1_matrix_size": {"from": len(ref.MATRIX), "to": len(MATRIX)},
            "W2_cell_ids": {"from": [c["id"] for c in ref.MATRIX],
                            "to": [c["id"] for c in MATRIX]},
            "W3_all_stochastic_held": True,
            "W4_seeds": {"from": sorted({c["seed"] for c in ref.MATRIX}),
                         "to": list(HELDOUT_SEEDS)},
            "W5_stage_specific_strings_and_paths": True,
            "W6_aggregate_rule": {"from": J19B_AGGREGATE_RULE, "to": AGGREGATE_RULE,
                                  "from_verified_present_in_j19b_source": True},
        },
        "anything_outside_the_whitelist_raises": True,
        "matrix": [{k: c[k] for k in ("id", "mode", "seed", "offset_s")} for c in MATRIX],
        "thresholds_invented_here": 0,
        "what_changed": "the held-out seeds and the matrix size, and nothing else",
        "actor_before": "J19A (j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1), "
                        "which PASSED J19B 6/6",
        "actor_now": "J19A, the same actor, at the SAME nominal start, under three "
                     "NEW NOISE REALISATIONS drawn with held-out seeds. No new start "
                     "offset is tested here.",
    }
'''

# The FORBIDDEN_HERE env-mutation entry. J19B's wording says the matrix varies
# episode_start_offset_s; in J19C the matrix does NOT vary it. Both the comment
# and the STRING are corrected. The pinned ENV_MUTATION_POLICY is NOT touched.
FORBIDDEN_ENV_OLD = (
    '                  # episode_start_offset_s is the ONLY such field the matrix varies,'
    ' because the\n'
    '                  # matrix IS a sweep over the three preregistered starts.\n'
    '                  "mutating any scientific or runtime env field;'
    ' episode_start_offset_s is the "\n'
    '                  "ONLY one the matrix varies",'
)

FORBIDDEN_ENV_NEW = (
    '                  # In J19C the matrix does NOT sweep: all three held-out cells start at\n'
    '                  # the SAME frozen nominal offset, so episode_start_offset_s does not\n'
    '                  # vary either. The only field that differs between G, H and I is\n'
    '                  # output_dir, which is instrumentation, not a scientific field.\n'
    '                  "mutating any scientific or runtime env field; in J19C NONE varies,"\n'
    '                  " episode_start_offset_s included: the only per-cell difference is"\n'
    '                  " output_dir",'
)

# A single source of truth for how the policy is REPORTED, inserted after the
# pinned constant, which is itself left byte-identical.
POLICY_ANCHOR = ('                       "which compares every other key against the'
                 ' verified base",\n}\n')

ENV_REPORT_HELPER = POLICY_ANCHOR + '''

def env_mutation_report() -> dict[str, Any]:
    """How this stage REPORTS the env-mutation policy, in two explicit sections.

    The pinned ENV_MUTATION_POLICY constant is byte-identical to J19B's and is
    reproduced verbatim under 'inherited_conservative_superset'. Its 'why' field
    describes J19B's A-F sweep and is FALSE of this stage, so it may appear ONLY
    inside that clearly labelled section, never as a J19C claim.

    'effective_j19c' states what actually happens: no scientific or runtime field
    varies between G, H and I, the start offset is the frozen nominal in all
    three, and the only per-cell difference is output_dir.
    """
    offsets = sorted({float(c["offset_s"]) for c in MATRIX})
    if offsets != [float(OFFSET_NOMINAL)]:
        raise J19CError(f"the held-out matrix varies the start offset: {offsets}")
    return {
        "inherited_conservative_superset": {
            "constant": dict(ENV_MUTATION_POLICY),
            "byte_identical_to_j19b": True,
            "why_reproduced_verbatim": (
                "the core scientific equivalence requires this constant to stay "
                "byte-identical to J19B's; rewording it would break the very property "
                "it exists to prove"),
            "its_why_field_describes_J19B_not_J19C": True,
            "it_AUTHORISES_varying_episode_start_offset_s": True,
            "this_stage_never_exercises_that_authorisation": True,
        },
        "effective_j19c": {
            "scientific_or_runtime_fields_that_actually_vary": [],
            "episode_start_offset_s_varies": False,
            "episode_start_offset_s_value_in_every_cell": OFFSET_NOMINAL,
            "distinct_offsets_across_the_matrix": offsets,
            "matrix_is_a_sweep": False,
            "only_field_differing_between_cells": "output_dir",
            "output_dir_is_instrumentation_not_science": True,
            "cells": [c["id"] for c in MATRIX],
        },
    }
'''

AGG_RULE_CONST = '''AGGREGATE_RULE = "PASS iff 3/3 behavioural PASS AND 3/3 telemetry-valid"
# J19B holds its rule as an inline literal, not as a named constant, so the core
# equivalence compares against this transcription and verifies it really occurs
# in the pinned J19B source rather than trusting the transcription.
J19B_AGGREGATE_RULE = "PASS iff 6/6 behavioural PASS AND 6/6 telemetry-valid"
PIN_J19B_SOURCE = "327b8fe39edae290fd8ed806f6b34715313b8d82d7c08ba600054eb191bdcb19"
'''

# (kind, start anchor, end anchor, consume_end, replacement).
# consume_end matters: a function block ends AT the next "\n\ndef ", which must be
# LEFT IN PLACE or the following function loses its own "def ". The matrix block
# ends at its closing paren, which must be consumed.
STRUCTURAL_REPLACEMENTS = (
    ("matrix", MATRIX_OLD_ANCHOR, MATRIX_END, True, MATRIX_BLOCK),
    ("policy_prose", FORBIDDEN_ENV_OLD, None, False, FORBIDDEN_ENV_NEW),
    ("policy_report", POLICY_ANCHOR, None, False, ENV_REPORT_HELPER),
    ("aggregate", "EXPECTED_STEPS = 500\n", None, False,
     "EXPECTED_STEPS = 500\n" + AGG_RULE_CONST),
    ("provenance", "def verify_prereg() -> dict[str, Any]:", "\n\ndef ", False,
     VERIFY_PREREG_BLOCK),
    ("provenance", "def verify_scientific_equivalence() -> dict[str, Any]:", "\n\ndef ",
     False, VERIFY_EQUIV_BLOCK),
)


def sha256_bytes(payload):
    """SHA-256 of a byte string."""
    return hashlib.sha256(payload).hexdigest()


def read_source(root=HERE):
    """Read the frozen J19B source after verifying its bytes."""
    path = root / SOURCE_NAME
    raw = path.read_bytes()
    actual = sha256_bytes(raw)
    if actual != SOURCE_SHA256:
        raise RuntimeError(
            "the frozen J19B runner changed: expected %s, found %s. A derivation from "
            "altered source would be meaningless." % (SOURCE_SHA256, actual)
        )
    return raw.decode("utf-8")


def apply_substitutions(text):
    """Apply the closed textual map, counting every hit."""
    applied = []
    for kind, old, new in SUBSTITUTIONS:
        count = text.count(old)
        text = text.replace(old, new)
        applied.append({"kind": kind, "old": old, "new": new, "hits": count})
    return text, applied


def apply_structural(text):
    """Splice the declared structural blocks, by exact span location."""
    applied = []
    for kind, start, end, consume_end, replacement in STRUCTURAL_REPLACEMENTS:
        i = text.find(start)
        if i < 0:
            applied.append({"kind": kind, "anchor": start[:48], "hits": 0})
            continue
        j = i + len(start) if end is None else text.find(end, i + len(start))
        if j < 0:
            applied.append({"kind": kind, "anchor": start[:48], "hits": 0})
            continue
        if end is not None and consume_end:
            j += len(end)
        text = text[:i] + replacement + text[j:]
        applied.append({"kind": kind, "anchor": start[:48], "hits": 1})
    return text, applied


def derive(root=HERE):
    """Produce the derived source in memory and report what changed."""
    original = read_source(root)
    derived, applied = apply_substitutions(original)
    derived, structural = apply_structural(derived)
    applied = applied + [{"kind": s["kind"], "old": s["anchor"], "new": "<structural block>",
                          "hits": s["hits"]} for s in structural]
    return {
        "original": original, "derived": derived, "applied": applied,
        "total_hits": sum(a["hits"] for a in applied),
        "unhit": [a for a in applied if a["hits"] == 0],
        "textual_entries": len(SUBSTITUTIONS),
        "structural_entries": len(STRUCTURAL_REPLACEMENTS),
        "source_sha256": sha256_bytes(original.encode("utf-8")),
        "derived_sha256": sha256_bytes(derived.encode("utf-8")),
    }


def residual_j19b_references(derived):
    """Surviving J19B identifiers OUTSIDE the structural blocks."""
    outside = derived
    for _kind, _start, _end, _consume, replacement in STRUCTURAL_REPLACEMENTS:
        outside = outside.replace(replacement, "")
    return sorted(set(re.findall(r"\bJ19B\w*|\bj19b_\w*", outside)))


def main(argv=None):
    """Return 0 on success. --write is the only mode that writes."""
    parser = argparse.ArgumentParser(description="derive the J19C runner from J19B")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args(argv)

    report = derive()
    print("source  %s  %s" % (SOURCE_NAME, report["source_sha256"]))
    print("derived %s  %s" % (DERIVED_NAME, report["derived_sha256"]))
    textual = [a for a in report["applied"] if a["new"] != "<structural block>"]
    structural = [a for a in report["applied"] if a["new"] == "<structural block>"]
    print("\ntextual: %d entries, %d hits | structural: %d entries, %d hits | total %d"
          % (len(textual), sum(a["hits"] for a in textual),
             len(structural), sum(a["hits"] for a in structural), report["total_hits"]))
    for entry in report["applied"]:
        print("  [%-9s] %-5d  %.46s" % (entry["kind"], entry["hits"], entry["old"]))
    if report["unhit"]:
        print("\n!! entries that matched nothing: %s" % [e["old"][:40] for e in report["unhit"]])
    residual = residual_j19b_references(report["derived"])
    print("\nresidual J19B identifiers outside the blocks: %s"
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
