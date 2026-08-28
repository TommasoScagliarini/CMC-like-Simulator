"""Static and inert tests for V26C J19B.

Runs NO rollout, NO fit and builds NO environment. It compiles and imports the
derived runner, re-derives it from J16 in memory to prove the copy is exactly
what the tool produces, and verifies by bytecode and AST that the scientific
path is unchanged.

Every defect the architect found in the withdrawn rebinding draft has a test
here that would catch it.
"""

from __future__ import annotations

import ast
import builtins
import dis
import hashlib
import json
import pathlib
import re
import sys
import types

sys.dont_write_bytecode = True

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import v26c_j16_closed_loop as J16          # noqa: E402  the frozen source, read-only
import v26c_j19b_closed_loop as J19B        # noqa: E402  the derived runner
import v26c_j19b_derive_from_j16 as DERIVE  # noqa: E402  the derivation tool

RUNNER_PATH = ROOT / "v26c_j19b_closed_loop.py"
DERIVE_PATH = ROOT / "v26c_j19b_derive_from_j16.py"
PREREG_PATH = ROOT / "v26c_j19b_prereg_closed_loop_qualification.json"
J16_PATH = ROOT / "v26c_j16_closed_loop.py"
PIN_J16 = "6ac4585424cbce34957722e8fc64dc0669de14c57a0418e6aa940b1303cc2e34"

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), str(detail)))


def sha_file(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def code_signature(function):
    """Bytecode identity: instruction stream, locals, arity and nested consts."""
    def flatten(code):
        out = [code.co_code, repr(code.co_varnames), code.co_argcount]
        for const in code.co_consts:
            out.append(flatten(const) if isinstance(const, types.CodeType) else repr(const))
        return repr(out)
    return hashlib.sha256(flatten(function.__code__).encode()).hexdigest()


# --------------------------------------------------------------------------
# B01-B06 - J16 is untouched and the copy is exactly the tool's output
# --------------------------------------------------------------------------


def test_j16_untouched_and_copy_is_mechanical():
    check("B01 the frozen J16 runner still matches its pin", sha_file(J16_PATH) == PIN_J16,
          sha_file(J16_PATH))
    check("B02 the derivation tool pins the same J16 hash",
          DERIVE.SOURCE_SHA256 == PIN_J16)

    report = DERIVE.derive(ROOT)
    on_disk = RUNNER_PATH.read_text(encoding="utf-8")
    check("B03 the derived runner on disk is EXACTLY what the tool produces",
          report["derived"] == on_disk,
          "in-memory %s vs on-disk %s" % (report["derived_sha256"][:16],
                                          hashlib.sha256(on_disk.encode()).hexdigest()[:16]))
    check("B04 every substitution entry matched at least once",
          not report["unhit"], [e["old"] for e in report["unhit"]])

    # The counts are part of what the preregistration ASSERTS. They went stale once
    # before, after two structural blocks were added, so they are pinned here and
    # cross-checked against the document rather than restated by hand.
    textual = [a for a in report["applied"] if a["new"] != "<structural block>"]
    structural = [a for a in report["applied"] if a["new"] == "<structural block>"]
    check("B04a the textual map holds exactly 17 entries",
          len(DERIVE.SUBSTITUTIONS) == 17 and len(textual) == 17, len(textual))
    check("B04b the textual map lands exactly 238 hits",
          sum(a["hits"] for a in textual) == 238, sum(a["hits"] for a in textual))
    check("B04c there are exactly 9 structural replacements, each hitting once",
          len(DERIVE.STRUCTURAL_REPLACEMENTS) == 9 and len(structural) == 9
          and sum(a["hits"] for a in structural) == 9,
          (len(structural), sum(a["hits"] for a in structural)))
    check("B04d the total is exactly 247 hits", report["total_hits"] == 247,
          report["total_hits"])

    prereg = json.loads(PREREG_PATH.read_text(encoding="utf-8"))
    derivation = prereg["derivation_instead_of_rebinding"]
    check("B04e the preregistration states the same 17 and 238",
          derivation["textual_substitution_map"]["entries"] == len(textual)
          and derivation["textual_substitution_map"]["hits"]
          == sum(a["hits"] for a in textual),
          derivation["textual_substitution_map"])
    check("B04f the preregistration states the same 9 and 9",
          derivation["structural_replacement_count"] == len(structural)
          and derivation["structural_replacement_hits"]
          == sum(a["hits"] for a in structural))
    check("B04g the preregistration states the same total 247",
          derivation["total_hits"] == report["total_hits"], derivation["total_hits"])
    check("B04h the preregistration lists all nine structural replacements",
          len(derivation["structural_replacements"]) == len(structural),
          len(derivation["structural_replacements"]))
    listed = " ".join(e["what"] for e in derivation["structural_replacements"])
    check("B04i and names the leaf-comment and training_data blocks explicitly",
          "Eight files" in listed and "training_data" in listed)
    check("B05 no J16 or J15R1 identifier survives outside the replacement blocks",
          not DERIVE.residual_j16_references(report["derived"]),
          DERIVE.residual_j16_references(report["derived"]))

    raised = False
    try:
        original = DERIVE.SOURCE_SHA256
        DERIVE.SOURCE_SHA256 = "0" * 64
        DERIVE.read_source(ROOT)
    except RuntimeError as error:
        raised = "would be meaningless" in str(error)
    finally:
        DERIVE.SOURCE_SHA256 = original
    check("B06 the tool REFUSES to derive from an altered J16", raised)


# --------------------------------------------------------------------------
# B07-B14 - the scientific path is unchanged
# --------------------------------------------------------------------------


def test_scientific_path_unchanged():
    identical = ("base_env_config", "cell_env_config", "expected_reset_time",
                 "unit_correction", "evaluate_cell_gate", "cell_verdict",
                 "penetration_report", "production_stack")
    mismatched = [n for n in identical
                  if code_signature(getattr(J16, n)) != code_signature(getattr(J19B, n))]
    check("B07 the eight scientific functions are BYTECODE-IDENTICAL to J16",
          not mismatched, mismatched)

    a, b = J16.run_cell.__code__, J19B.run_cell.__code__
    check("B08 run_cell's instruction stream is identical", a.co_code == b.co_code)
    names = sorted(set(a.co_names) ^ set(b.co_names))
    check("B09 run_cell's only name differences are the declared renames",
          names == ["J15R1_MODULE_DIR", "J16Error", "J19A_MODULE_DIR", "J19BError"], names)
    consts_a = [c for c in a.co_consts if not isinstance(c, types.CodeType)]
    consts_b = [c for c in b.co_consts if not isinstance(c, types.CodeType)]
    differing = [(x, y) for x, y in zip(consts_a, consts_b) if x != y]
    check("B10 run_cell's only constant differences are two output strings",
          differing == [("j16_cell_", "j19b_cell_"), ("J16 cell ", "J19B cell ")], differing)

    for name in ("MATRIX", "SIGMA", "SIGMA_TOLERANCE", "ACTOR_WIDTH", "CLOCK_COLUMNS",
                 "CONTROLLER_COLUMNS", "EXPECTED_STEPS", "NOISE_HOLD_STEPS"):
        check("B11 %s is identical by value" % name,
              getattr(J16, name) == getattr(J19B, name))
    check("B12 the gate tables are identical by value",
          J16.J16_COMMON_GATE == J19B.J19B_COMMON_GATE
          and J16.J16_KINEMATIC_GATE == J19B.J19B_KINEMATIC_GATE)
    check("B13 the six cells, seeds and modes are the frozen ones",
          [(c["id"], c["mode"], c["seed"], c["offset_s"]) for c in J19B.MATRIX]
          == [(c["id"], c["mode"], c["seed"], c["offset_s"]) for c in J16.MATRIX])
    check("B14 only seeds 123, 124 and 125 appear in the matrix",
          sorted({c["seed"] for c in J19B.MATRIX}) == [123, 124, 125])


# --------------------------------------------------------------------------
# B15-B22 - the four defects of the withdrawn rebinding draft
# --------------------------------------------------------------------------


def test_the_four_withdrawn_defects():
    check("B15 verify_prereg reads the J19B preregistration, not J16's",
          J19B.PREREG.name == "v26c_j19b_prereg_closed_loop_qualification.json",
          J19B.PREREG.name)
    check("B16 verify_prereg is NOT J16's function",
          code_signature(J16.verify_prereg) != code_signature(J19B.verify_prereg))

    check("B17 the leaf path is j19b_runs, not j16_runs",
          J19B.RELATIVE_LEAF_PARTS[0] == "j19b_runs"
          and J19B.RELATIVE_LEAF == "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1",
          J19B.RELATIVE_LEAF)
    check("B18 staging, lock and sentinel names follow the J19B leaf",
          J19B.STAGING_NAME.endswith("j19b_closed_loop_v26c_2026-08-27_r1")
          and J19B.LOCK_NAME.endswith("j19b_closed_loop_v26c_2026-08-27_r1")
          and "j19b" in J19B.PREFLIGHT_SENTINEL.name,
          (J19B.STAGING_NAME, J19B.LOCK_NAME, J19B.PREFLIGHT_SENTINEL.name))
    check("B19 the authorised leaf resolves under j19b_runs with no override",
          J19B.OUTPUT_ROOT_OVERRIDE is None
          and J19B.authorized_leaf().parts[-2:] ==
          ("j19b_runs", "j19b_closed_loop_v26c_2026-08-27_r1"),
          str(J19B.authorized_leaf()))

    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("B20 run_matrix no longer subscripts manifest['actor_feature_names']",
          'manifest["actor_feature_names"]' not in source)
    check("B21 the receipt name is the J19B one",
          J19B.RECEIPT_NAME == "v26c_j19b_closed_loop_receipt.json", J19B.RECEIPT_NAME)
    # Structural, not substring: the derived module docstring DELIBERATELY names
    # v26c_j16_closed_loop.py as what it was derived from. What must be absent is
    # any j16_ IDENTIFIER or output name in the code itself.
    tree = ast.parse(source)
    identifiers = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    identifiers |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    identifiers |= {n.name for n in ast.walk(tree)
                    if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    stale = sorted(i for i in identifiers if i.startswith("j16_") or i == "J16Error")
    check("B22a no j16_ identifier or J16Error survives in the code", not stale, stale)
    # The module docstring is excluded by NODE, not by text: it deliberately names
    # v26c_j16_closed_loop.py as the source of the derivation, and comparing against
    # ast.get_docstring would miss it because that helper normalises the text.
    docstring_node = None
    if tree.body and isinstance(tree.body[0], ast.Expr) \
            and isinstance(tree.body[0].value, ast.Constant):
        docstring_node = tree.body[0].value
    leaked = sorted({n.value for n in ast.walk(tree)
                     if isinstance(n, ast.Constant) and isinstance(n.value, str)
                     and "j16_" in n.value and n is not docstring_node})
    check("B22b no j16_ output name survives in any other string constant",
          not leaked, leaked)
    check("B22c the module docstring does name its J16 source, deliberately",
          docstring_node is not None
          and "MECHANICAL derivation of v26c_j16_closed_loop.py" in docstring_node.value)


# --------------------------------------------------------------------------
# B23-B31 - feature names, resolved before the environment, from agreeing pins
# --------------------------------------------------------------------------


def test_feature_names():
    resolved = J19B.resolve_feature_names()
    check("B23 exactly 35 names are resolved", resolved["count"] == 35, resolved["count"])
    check("B24 three pinned sources are consulted and they agree",
          len(resolved["sources"]) == 3 and resolved["sources_agree"] is True)
    check("B25 the names match those J16 itself used",
          list(resolved["actor_feature_names"])
          == json.loads((ROOT / "j15_runs/j15_fresh_refit_v26c_2026-08-27_r1"
                         / "rl_module/actor_feature_manifest.json").read_text())
          ["actor_feature_names"])
    with np.load(ROOT / "j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1"
                 / "j10r1_cell_B_teacher_dataset.npz", allow_pickle=False) as bundle:
        teacher = [str(n) for n in bundle["actor_feature_names"]]
    check("B26 and the pinned teacher dataset's order", list(resolved["actor_feature_names"])
          == teacher)
    check("B27 the J8 sidecar is used for names ONLY and declared stale",
          resolved["j8_sidecar_used_for"] == "feature ORDER AND NAMES ONLY"
          and resolved["j8_sidecar_is_known_stale"] is True
          and resolved["j8_sidecar_not_used_as_module_hash_evidence"] is True)

    original = dict(J19B.PIN_FEATURE_NAME_SOURCES)
    raised = False
    try:
        key = next(iter(J19B.PIN_FEATURE_NAME_SOURCES))
        J19B.PIN_FEATURE_NAME_SOURCES[key] = "0" * 64
        J19B.resolve_feature_names()
    except J19B.J19BError as error:
        raised = "changed" in str(error)
    finally:
        J19B.PIN_FEATURE_NAME_SOURCES.clear()
        J19B.PIN_FEATURE_NAME_SOURCES.update(original)
    check("B28 a stale feature-name source is REFUSED", raised)

    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == "run_matrix")
    resolve_line = next((n.lineno for n in ast.walk(fn) if isinstance(n, ast.Call)
                         and getattr(n.func, "id", "") == "resolve_feature_names"), None)
    stack_line = next((n.lineno for n in ast.walk(fn) if isinstance(n, ast.Call)
                       and getattr(n.func, "id", "") == "production_stack"), None)
    check("B29 the names are resolved BEFORE the environment stack is built",
          resolve_line is not None and stack_line is not None and resolve_line < stack_line,
          "resolve@%s stack@%s" % (resolve_line, stack_line))

    manifest = json.loads((ROOT / "j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1"
                           / "rl_module/actor_feature_manifest.json").read_text())
    check("B30 the J19A manifest genuinely has neither field",
          "actor_feature_names" not in manifest and "deployable" not in manifest,
          sorted(manifest))
    check("B31 verify_actor REFUSES a J19A manifest that grew either field",
          'if "actor_feature_names" in manifest or "deployable" in manifest:'
          in RUNNER_PATH.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------
# B32-B40 - the J19A actor, its leaf, and the adapted verify_actor
# --------------------------------------------------------------------------


def test_actor_and_leaf():
    check("B32 the pinned actor is the J19A module",
          J19B.PIN_J19A_MODULE_STATE
          == "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530")
    check("B33 the pinned digest is the J19A actor digest",
          J19B.PIN_J19A_ACTOR_DIGEST
          == "d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96")
    check("B34 the leaf holds exactly seven pinned files", len(J19B.PIN_J19A) == 7,
          sorted(J19B.PIN_J19A))
    for rel, pin in sorted(J19B.PIN_J19A.items()):
        check("B35 J19A artefact %s matches its pin" % rel,
              sha_file(J19B.J19A_LEAF / rel) == pin)

    report = J19B.verify_actor()
    check("B36 verify_actor accepts the real J19A leaf",
          report["file_count"] == 7 and report["manifest"]["describes_this_module"] is True)
    check("B37 it reads the commit verification under the J19A key 'ok'",
          report["leaf_validity"]["commit_verification_ok"] is True)
    check("B38 it asserts the mask contract with J16's own constants",
          report["mask_contract"]["width"] == J16.ACTOR_WIDTH
          and report["mask_contract"]["clock_columns"] == list(J16.CLOCK_COLUMNS))
    check("B39 it asserts the frozen sigma with J16's tolerance",
          report["sigma"]["value"] == J16.SIGMA
          and report["sigma"]["tolerance"] == J16.SIGMA_TOLERANCE
          and report["sigma"]["logstd_head_edited"] is False)
    check("B40 it asserts the J19A lineage: PASS, one actor, no rollout, parent J8",
          report["lineage"]["j19a_verdict"] == "PASS"
          and report["lineage"]["actors_persisted"] == 1
          and report["lineage"]["rollout_performed"] is False
          and report["lineage"]["parent_j8_sha256"].startswith("9c5b1571"))
    check("B41 the actor differs from its parent",
          report["manifest"]["differs_from_parent"] is True)
    check("B42 the dead J15R1-lineage constants were dropped",
          not any(hasattr(J19B, n) for n in
                  ("NOMINAL_DRIFT_ADDENDUM", "PIN_NOMINAL_DRIFT_ADDENDUM",
                   "PIN_J14_DATASET", "PIN_J11_FAILED_ACTOR")))

    # Two statements J16 carried that are FALSE of J19A. The first is a comment;
    # the second is a REPORTED FIELD and would have written a false claim about the
    # training aggregate into the J19B receipt.
    source = RUNNER_PATH.read_text(encoding="utf-8")
    on_disk = sorted(str(p.relative_to(J19B.J19A_LEAF)).replace("\\", "/")
                     for p in J19B.J19A_LEAF.rglob("*") if p.is_file())
    check("B43a the leaf comment says SEVEN and the leaf really holds seven files",
          "SEVEN files" in source and "Eight files" not in source and len(on_disk) == 7,
          len(on_disk))
    check("B43b it no longer claims J19A commits an aggregate dataset",
          "J19A commits no aggregate dataset" in source
          and not any(p.endswith(".npz") for p in on_disk))
    check("B43c the reported training aggregate describes J18 c13, not J15R1's",
          "4221 rows" in source and "25567 in total" not in source
          and "16713" not in source)
    check("B43d and names the four disjoint blocks with their real counts",
          all(t in source for t in ("(500)", "(14)", "(2497)", "(1210)")))


# --------------------------------------------------------------------------
# B43-B52 - nothing runs, nothing is written, nothing sealed
# --------------------------------------------------------------------------


def test_nothing_runs():
    check("B43 j19b_runs is absent", not (ROOT / "j19b_runs").exists())
    check("B44 no J19B GO exists on disk",
          not [p for p in ROOT.glob("*j19b*go*.json")],
          [p.name for p in ROOT.glob("*j19b*go*.json")])
    # SEALED. B45 previously asserted PIN_PREREG was still PENDING; the
    # preregistration is now immutable and the pin must be its REAL hash, not a
    # placeholder and not a hash copied by hand.
    real = sha_file(PREREG_PATH)
    check("B45a the preregistration pin is no longer PENDING",
          J19B.PIN_PREREG != "PENDING", J19B.PIN_PREREG)
    check("B45b the pin is the REAL hash of the preregistration on disk",
          J19B.PIN_PREREG == real, "pin %s vs file %s" % (J19B.PIN_PREREG[:16], real[:16]))
    check("B45c it is the hash the architect sealed",
          real == "31c2705a8c9501969ddc39a37db43c3a419187febcd5cc830ff3cbfce57347b8", real)
    check("B45d the derivation tool seals the same hash",
          any(new.strip('"') == real for kind, _old, new in DERIVE.SUBSTITUTIONS
              if kind == "prereg"),
          [new for kind, _o, new in DERIVE.SUBSTITUTIONS if kind == "prereg"])

    # The seal must BITE: with a pin that no longer matches, verify_prereg refuses.
    original = J19B.PIN_PREREG
    bitten = False
    try:
        J19B.PIN_PREREG = "0" * 64
        J19B.verify_prereg()
    except J19B.J19BError as error:
        bitten = "preregistration changed" in str(error)
    finally:
        J19B.PIN_PREREG = original
    check("B45e a preregistration that no longer matches the seal is REFUSED", bitten)
    check("B45f and with the real seal verify_prereg passes",
          J19B.verify_prereg()["sha256"] == real)
    check("B46 no environment or rollout module has been imported",
          not [m for m in ("torch", "ray", "opensim", "gymnasium", "env_factory",
                           "rollout_eval") if m in sys.modules],
          [m for m in ("torch", "ray", "opensim", "gymnasium", "env_factory",
                       "rollout_eval") if m in sys.modules])
    check("B47 the preflight sentinel was never created",
          not J19B.PREFLIGHT_SENTINEL.exists())

    seeds = {123, 124, 125}
    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    sealed = [n.value for n in ast.walk(tree)
              if isinstance(n, ast.Constant) and isinstance(n.value, int)
              and n.value in (126, 127, 128)]
    check("B48 no sealed seed 126, 127 or 128 appears in the runner", not sealed, sealed)
    check("B49 the runner's seeds are exactly the frozen three",
          {c["seed"] for c in J19B.MATRIX} == seeds)

    module_names = set(vars(J19B)) | set(vars(builtins))
    unresolved = []

    def walk(code, origin):
        for ins in dis.get_instructions(code):
            if ins.opname in ("LOAD_GLOBAL", "STORE_GLOBAL", "DELETE_GLOBAL") \
                    and ins.argval not in module_names:
                unresolved.append((origin, ins.argval))
        for const in code.co_consts:
            if isinstance(const, types.CodeType):
                walk(const, origin)

    for attr, value in vars(J19B).items():
        if isinstance(value, types.FunctionType) and value.__module__ == J19B.__name__:
            walk(value.__code__, attr)
    check("B50 every global name in the derived runner resolves", not unresolved, unresolved)

    check("B51 the penetration contract module is the pinned one",
          sha_file(ROOT / "v26c_penetration_contract.py")
          == "9257e9b8cdf54d9a59bfe2ee25526b283408d325a900156a69d64dbf196298dc")
    # Structural: the bands appear in PROSE, inherited verbatim from J16, which
    # itself states it writes down no threshold and delegates every evaluation to
    # the pinned contract module. What must be absent is a numeric LITERAL used as
    # a threshold in code.
    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    literals = sorted({n.value for n in ast.walk(tree)
                       if isinstance(n, ast.Constant) and isinstance(n.value, float)
                       and n.value in (0.020, 0.025, 0.028)})
    check("B52a no penetration band appears as a numeric literal in code",
          not literals, literals)
    check("B52b penetration is evaluated only by the pinned contract module",
          "PC.evaluate_series" in RUNNER_PATH.read_text(encoding="utf-8")
          and "def evaluate_series" not in RUNNER_PATH.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------
# B53-B60 - the preregistration says only what is true
# --------------------------------------------------------------------------


def test_prereg():
    raw = PREREG_PATH.read_text(encoding="utf-8")
    prereg = json.loads(raw)
    check("B53 the prereg parses and carries no self-hash",
          prereg["contains_no_self_hash"] is True
          and hashlib.sha256(raw.encode()).hexdigest() not in raw)
    check("B54 it records the withdrawal of the rebinding draft and its four defects",
          len(prereg["supersedes_an_earlier_draft_of_this_document"]
              ["the_four_confirmed_defects"]) == 4)
    check("B55 it claims bytecode equality, NOT object identity",
          prereg["scientific_path_is_unchanged"]["run_cell"]
          ["no_object_identity_is_claimed"].startswith("the two modules hold distinct"))
    check("B56 the eight bytecode-identical functions are named",
          len(prereg["scientific_path_is_unchanged"]["bytecode_identical_to_J16"]) == 8)
    check("B57 the six cells and seeds match the runner",
          [(c["id"], c["seed"]) for c in prereg["the_six_cells"]["cells"]]
          == [(c["id"], c["seed"]) for c in J19B.MATRIX]
          and prereg["the_six_cells"]["sealed_seeds_126_127_128"].startswith("OUT OF PERIMETER"))
    check("B58 the penetration bands are stated exactly, with 28 passing",
          "exactly 0.028 PASSES" in prereg["penetration_contract"]["hard_binding"]
          and prereg["penetration_contract"]["unchanged_from_j16"] is True)
    check("B59 the J8 sidecar caveat is declared",
          prereg["feature_names_resolution"]["the_j8_sidecar_caveat"]
          ["declared_explicitly"] is True)
    check("B60 the stage token matches the runner", prereg["stage"] == J19B.STAGE)
    check("B61 no threshold is invented in this stage",
          prereg["gates"]["thresholds_invented_in_this_stage"] == 0)


# --------------------------------------------------------------------------
# B62-B76 - the REAL preflight, called end to end.
#
# The suite previously exercised the runner field by field and never invoked
# preflight() itself. That is exactly how a KeyError on a prereg field read
# only inside preflight survived every check: 'deferred_todo' is the one
# prereg field the runner reads OUTSIDE verify_prereg. This calls the real
# thing and asserts it is inert.
# --------------------------------------------------------------------------


def test_real_preflight_end_to_end():
    heavy = ("torch", "ray", "opensim", "gymnasium", "env_factory", "rollout_eval")
    before_heavy = sorted(m for m in heavy if m in sys.modules)
    before_tree = {str(p.relative_to(ROOT)): (p.stat().st_size, p.stat().st_mtime_ns)
                   for p in sorted(ROOT.rglob("*")) if p.is_file()}
    sentinel_before = J19B.PREFLIGHT_SENTINEL.exists()

    report = J19B.preflight()

    after_heavy = sorted(m for m in heavy if m in sys.modules)
    after_tree = {str(p.relative_to(ROOT)): (p.stat().st_size, p.stat().st_mtime_ns)
                  for p in sorted(ROOT.rglob("*")) if p.is_file()}

    check("B62 preflight() runs to completion without raising", isinstance(report, dict))
    check("B63 its verdict is GO", report.get("verdict") == "GO", report.get("verdict"))
    check("B64 it reports no blockers", report.get("blockers") == [], report.get("blockers"))
    check("B65 it imported no heavy module", after_heavy == before_heavy == [],
          (before_heavy, after_heavy))
    check("B66 the sentinel was never created",
          not J19B.PREFLIGHT_SENTINEL.exists() and not sentinel_before)
    check("B67 it created no output leaf", not (ROOT / "j19b_runs").exists())
    check("B68 it left the tree byte-for-byte unchanged", before_tree == after_tree,
          sorted(set(after_tree) ^ set(before_tree))
          or [k for k in before_tree if before_tree[k] != after_tree.get(k)])

    check("B69 it read deferred_todo from THIS preregistration",
          report.get("deferred_todo")
          == json.loads(PREREG_PATH.read_text(encoding="utf-8"))["deferred_todo"])
    todo = report.get("deferred_todo") or []
    joined = " ".join(todo)
    for token in ("LOTO", "LOCO", "B1R1", "B1R2", "126, 127, 128", "G-I", "Epic"):
        check("B70 deferred_todo carries the frozen item %r" % token, token in joined)
    check("B71 deferred_todo opens no new perimeter",
          not any(t in joined for t in ("PPO", "critic", "collection", "new fit")), todo)

    check("B72 its outcome policy promotes nothing and authorises no next stage",
          report["outcome_policy"]["deployable"] is False
          and report["outcome_policy"]["promotion"] == "NONE"
          and report["outcome_policy"]["next_stage_authorized"] is False
          and report["outcome_policy"]["ppo_updates"] == 0)
    check("B73 it declares a single execution with no autonomous retry",
          report["outcome_policy"]["single_execution"] is True
          and report["outcome_policy"]["no_autonomous_retry"] is True)
    check("B74 it would write only the authorised J19B leaf",
          report["would_write"]["relative_leaf"] == J19B.RELATIVE_LEAF
          and report["would_write"]["output_root_override"] is None,
          report["would_write"]["relative_leaf"])
    check("B75 it requires the J19B stage token to run",
          report["requires_to_run"]["stage_token"] == J19B.STAGE)
    check("B76 every prereg field the runner reads is present",
          set(re.findall(r'PREREG\.read_text\(\)\)\["(\w+)"\]',
                         RUNNER_PATH.read_text(encoding="utf-8")))
          <= set(json.loads(PREREG_PATH.read_text(encoding="utf-8"))),
          sorted(set(re.findall(r'PREREG\.read_text\(\)\)\["(\w+)"\]',
                                RUNNER_PATH.read_text(encoding="utf-8")))))


def main():
    test_j16_untouched_and_copy_is_mechanical()
    test_scientific_path_unchanged()
    test_the_four_withdrawn_defects()
    test_feature_names()
    test_actor_and_leaf()
    test_nothing_runs()
    test_prereg()
    test_real_preflight_end_to_end()

    failed = [c for c in CHECKS if not c[1]]
    for name, passed, detail in CHECKS:
        print("[%s] %-66s %s" % ("PASS" if passed else "FAIL", name, detail))
    print("\n%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
