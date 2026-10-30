"""Static and inert tests for V26C J19C, the held-out G/H/I phase.

Runs NO rollout, NO fit and builds NO environment. It re-derives the runner from
J19B in memory, verifies the core scientific equivalence and its closed
whitelist, and proves NEGATIVELY that the runner refuses any alternative seed,
start, mode, sigma, step count or noise hold.
"""

from __future__ import annotations

import ast
import builtins
import copy
import dis
import hashlib
import json
import pathlib
import re
import sys
import types

sys.dont_write_bytecode = True

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import v26c_j19b_closed_loop as J19B          # noqa: E402  the frozen source
import v26c_j19c_heldout_g_i as J19C          # noqa: E402  the derived runner
import v26c_j19c_derive_from_j19b as DERIVE   # noqa: E402  the derivation tool

RUNNER_PATH = ROOT / "v26c_j19c_heldout_g_i.py"
PREREG_PATH = ROOT / "v26c_j19c_prereg_heldout_g_i.json"
J19B_PATH = ROOT / "v26c_j19b_closed_loop.py"
PIN_J19B = "327b8fe39edae290fd8ed806f6b34715313b8d82d7c08ba600054eb191bdcb19"
J19A_LEAF = ROOT / "j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1"
J19B_LEAF = ROOT / "j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1"

HELDOUT = (126, 127, 128)
PRIOR = (123, 124, 125)
NOMINAL = 1.956870983805102

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), str(detail)))


def sha_file(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def code_signature(function):
    def flatten(code):
        out = [code.co_code, repr(code.co_varnames), code.co_argcount]
        for const in code.co_consts:
            out.append(flatten(const) if isinstance(const, types.CodeType) else repr(const))
        return repr(out)
    return hashlib.sha256(flatten(function.__code__).encode()).hexdigest()


def with_matrix(cells):
    """Run the core equivalence against a mutated matrix, restoring it after."""
    original = J19C.MATRIX
    try:
        J19C.MATRIX = tuple(cells)
        J19C.verify_scientific_equivalence()
        return None
    except J19C.J19CError as error:
        return str(error)
    finally:
        J19C.MATRIX = original


# --------------------------------------------------------------------------
# C01-C08 - the copy is mechanical and J19B/J16 are untouched
# --------------------------------------------------------------------------


def test_derivation_is_mechanical():
    check("C01 the frozen J19B runner still matches its pin",
          sha_file(J19B_PATH) == PIN_J19B, sha_file(J19B_PATH))
    check("C02 the derivation tool pins the same J19B hash", DERIVE.SOURCE_SHA256 == PIN_J19B)
    check("C03 J16 is untouched",
          sha_file(ROOT / "v26c_j16_closed_loop.py")
          == "6ac4585424cbce34957722e8fc64dc0669de14c57a0418e6aa940b1303cc2e34")

    report = DERIVE.derive(ROOT)
    check("C04 the derived runner on disk is EXACTLY what the tool produces",
          report["derived"] == RUNNER_PATH.read_text(encoding="utf-8"))
    check("C05 every substitution entry matched at least once",
          not report["unhit"], [e["old"][:40] for e in report["unhit"]])
    check("C06 no J19B identifier survives outside the replacement blocks",
          not DERIVE.residual_j19b_references(report["derived"]),
          DERIVE.residual_j19b_references(report["derived"]))

    textual = [a for a in report["applied"] if a["new"] != "<structural block>"]
    structural = [a for a in report["applied"] if a["new"] == "<structural block>"]
    check("C07 the map counts are what the tool reports",
          len(textual) == len(DERIVE.SUBSTITUTIONS)
          and len(structural) == len(DERIVE.STRUCTURAL_REPLACEMENTS)
          and report["total_hits"] == sum(a["hits"] for a in report["applied"]),
          (len(textual), sum(a["hits"] for a in textual),
           len(structural), sum(a["hits"] for a in structural), report["total_hits"]))

    raised = False
    try:
        original = DERIVE.SOURCE_SHA256
        DERIVE.SOURCE_SHA256 = "0" * 64
        DERIVE.read_source(ROOT)
    except RuntimeError as error:
        raised = "would be meaningless" in str(error)
    finally:
        DERIVE.SOURCE_SHA256 = original
    check("C08 the tool REFUSES to derive from an altered J19B", raised)


# --------------------------------------------------------------------------
# C09-C22 - core scientific equivalence and the closed whitelist
# --------------------------------------------------------------------------


def test_core_equivalence():
    report = J19C.verify_scientific_equivalence()
    check("C09 the core equivalence passes", report["anything_outside_the_whitelist_raises"])
    check("C10 it is declared fail-closed against J19B",
          report["kind"] == "CORE SCIENTIFIC EQUIVALENCE TO J19B, FAIL-CLOSED"
          and report["source_sha256"] == PIN_J19B)

    identical = ("base_env_config", "cell_env_config", "expected_reset_time",
                 "unit_correction", "evaluate_cell_gate", "cell_verdict",
                 "penetration_report", "production_stack")
    drifted = [n for n in identical
               if code_signature(getattr(J19B, n)) != code_signature(getattr(J19C, n))]
    check("C11 the eight scientific functions are BYTECODE-IDENTICAL to J19B",
          not drifted, drifted)
    check("C12 the report names those same eight",
          report["bytecode_identical_functions"] == list(identical))

    for name, a, b in (("SIGMA", J19B.SIGMA, J19C.SIGMA),
                       ("SIGMA_TOLERANCE", J19B.SIGMA_TOLERANCE, J19C.SIGMA_TOLERANCE),
                       ("NOISE_HOLD_STEPS", J19B.NOISE_HOLD_STEPS, J19C.NOISE_HOLD_STEPS),
                       ("EXPECTED_STEPS", J19B.EXPECTED_STEPS, J19C.EXPECTED_STEPS),
                       ("OFFSET_NOMINAL", J19B.OFFSET_NOMINAL, J19C.OFFSET_NOMINAL),
                       ("FROZEN_OFFSETS", J19B.FROZEN_OFFSETS, J19C.FROZEN_OFFSETS),
                       ("ACTOR_WIDTH", J19B.ACTOR_WIDTH, J19C.ACTOR_WIDTH),
                       ("CLOCK_COLUMNS", J19B.CLOCK_COLUMNS, J19C.CLOCK_COLUMNS),
                       ("CONTROLLER_COLUMNS", J19B.CONTROLLER_COLUMNS, J19C.CONTROLLER_COLUMNS),
                       ("RESET_TIME_TOLERANCE_S", J19B.RESET_TIME_TOLERANCE_S,
                        J19C.RESET_TIME_TOLERANCE_S)):
        check("C13 %s identical to J19B" % name, a == b, (a, b))
    check("C14 sigma is 0.005, noise hold 1, steps 500, start nominal",
          J19C.SIGMA == 0.005 and J19C.NOISE_HOLD_STEPS == 1
          and J19C.EXPECTED_STEPS == 500 and J19C.OFFSET_NOMINAL == NOMINAL)
    check("C15 the gate tables are identical to J19B",
          J19B.J19B_COMMON_GATE == J19C.J19C_COMMON_GATE
          and J19B.J19B_KINEMATIC_GATE == J19C.J19C_KINEMATIC_GATE)
    check("C16 the actor pins are identical: the same J19A actor",
          J19B.PIN_J19A == J19C.PIN_J19A
          and J19B.PIN_J19A_MODULE_STATE == J19C.PIN_J19A_MODULE_STATE
          and J19B.PIN_J19A_ACTOR_DIGEST == J19C.PIN_J19A_ACTOR_DIGEST)
    check("C17 the env mutation policy is identical",
          dict(J19B.ENV_MUTATION_POLICY) == dict(J19C.ENV_MUTATION_POLICY))
    check("C18 the penetration contract module is the pinned one and unchanged",
          sha_file(ROOT / "v26c_penetration_contract.py")
          == "9257e9b8cdf54d9a59bfe2ee25526b283408d325a900156a69d64dbf196298dc")
    check("C19 seventeen core constants are compared", report["constants_compared"] == 17,
          report["constants_compared"])
    check("C20 no threshold is invented here", report["thresholds_invented_here"] == 0)

    w = report["authorised_differences"]
    check("C21 the whitelist has exactly the six authorised entries",
          sorted(w) == ["W1_matrix_size", "W2_cell_ids", "W3_all_stochastic_held",
                        "W4_seeds", "W5_stage_specific_strings_and_paths",
                        "W6_aggregate_rule"], sorted(w))
    check("C22 W1 6->3, W2 A-F->G-I, W4 123-125->126-128, W6 6/6->3/3",
          w["W1_matrix_size"] == {"from": 6, "to": 3}
          and w["W2_cell_ids"]["to"] == ["G", "H", "I"]
          and w["W4_seeds"]["to"] == list(HELDOUT)
          and "3/3" in w["W6_aggregate_rule"]["to"]
          and "6/6" in w["W6_aggregate_rule"]["from"])


# --------------------------------------------------------------------------
# C23-C34 - NEGATIVE tests: the runner refuses anything else
# --------------------------------------------------------------------------


def test_negative_alternatives_are_refused():
    base = [dict(c) for c in J19C.MATRIX]

    for label, mutate, token in (
        ("a prior seed 123", lambda m: m[0].__setitem__("seed", 123), "W4"),
        ("a prior seed 125", lambda m: m[2].__setitem__("seed", 125), "W4"),
        ("an unseen seed 999", lambda m: m[1].__setitem__("seed", 999), "W4"),
        ("a deterministic mode", lambda m: m[0].__setitem__("mode", "deterministic"), "W3"),
        ("a perturbed start -0.20 s",
         lambda m: m[1].__setitem__("offset_s", 1.756870983805102), "nominal offset"),
        ("a perturbed start +0.20 s",
         lambda m: m[2].__setitem__("offset_s", 2.156870983805102), "nominal offset"),
        ("a renamed cell", lambda m: m[0].__setitem__("id", "A"), "W2"),
    ):
        cells = copy.deepcopy(base)
        mutate(cells)
        error = with_matrix(cells)
        check("C23 the equivalence REFUSES %s" % label,
              error is not None and token in error, error)

    check("C24 it refuses a matrix of two cells", with_matrix(base[:2]) is not None)
    check("C25 it refuses a matrix of six cells",
          with_matrix(base + copy.deepcopy(base)) is not None)
    check("C26 and it accepts the preregistered matrix unchanged",
          with_matrix(base) is None)

    for label, name, value, token in (
        ("sigma", "SIGMA", 0.01, "core scientific constants"),
        ("noise hold", "NOISE_HOLD_STEPS", 2, "core scientific constants"),
        ("steps", "EXPECTED_STEPS", 400, "core scientific constants"),
        ("nominal offset", "OFFSET_NOMINAL", 2.0, "core scientific constants"),
        ("actor width", "ACTOR_WIDTH", 25, "core scientific constants"),
    ):
        original = getattr(J19C, name)
        error = None
        try:
            setattr(J19C, name, value)
            J19C.verify_scientific_equivalence()
        except J19C.J19CError as exc:
            error = str(exc)
        finally:
            setattr(J19C, name, original)
        check("C27 an altered %s is REFUSED" % label,
              error is not None and token in error, error)

    original = J19C.AGGREGATE_RULE
    error = None
    try:
        J19C.AGGREGATE_RULE = J19C.J19B_AGGREGATE_RULE
        J19C.verify_scientific_equivalence()
    except J19C.J19CError as exc:
        error = str(exc)
    finally:
        J19C.AGGREGATE_RULE = original
    check("C28 reverting the aggregate rule to J19B's 6/6 is REFUSED",
          error is not None, error)

    drifted = None
    original_fn = J19C.evaluate_cell_gate
    try:
        J19C.evaluate_cell_gate = lambda *a, **k: None
        J19C.verify_scientific_equivalence()
    except J19C.J19CError as exc:
        drifted = str(exc)
    finally:
        J19C.evaluate_cell_gate = original_fn
    check("C29 a drifted scientific function is REFUSED",
          drifted is not None and "scientific path drifted" in drifted, drifted)


# --------------------------------------------------------------------------
# C30-C38 - no CLI can change the protocol
# --------------------------------------------------------------------------


def test_no_cli_can_change_the_protocol():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    options = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "add_argument":
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                    options.add(arg.value)
    check("C30 the CLI exposes exactly the J19B option set",
          options == {"--preflight", "--run", "--authorized-stage", "--out", "--no-progress"},
          sorted(options))
    forbidden = [o for o in options
                 if any(t in o for t in ("seed", "start", "offset", "mode",
                                         "sigma", "step", "noise"))]
    check("C31 no CLI option can set seed, start, mode, sigma, steps or noise hold",
          not forbidden, forbidden)

    # MATRIX is an ANNOTATED assignment, so both ast.Assign and ast.AnnAssign
    # must be counted; looking only for Assign reports zero and hides the fact.
    for name in ("MATRIX", "SIGMA", "NOISE_HOLD_STEPS", "EXPECTED_STEPS", "OFFSET_NOMINAL"):
        assigns = [n for n in tree.body
                   if (isinstance(n, ast.Assign)
                       and any(getattr(t, "id", "") == name for t in n.targets))
                   or (isinstance(n, ast.AnnAssign)
                       and getattr(n.target, "id", "") == name)]
        check("C32 %s is a module constant assigned once" % name, len(assigns) == 1,
              len(assigns))
    check("C32b MATRIX is assigned at module level, never inside a function",
          not [n for n in ast.walk(tree)
               if isinstance(n, (ast.Assign, ast.AnnAssign))
               and n not in tree.body
               and (getattr(getattr(n, "target", None), "id", "") == "MATRIX"
                    or any(getattr(t, "id", "") == "MATRIX"
                           for t in getattr(n, "targets", [])))])

    check("C33 the runner reads no environment variable for the protocol",
          "os.environ" not in source or not re.search(
              r"os\.environ.*(seed|sigma|offset|step)", source, re.I))


# --------------------------------------------------------------------------
# C34-C44 - the preregistration, its pin, and the entry evidence
# --------------------------------------------------------------------------


def test_prereg_and_entry_evidence():
    raw = PREREG_PATH.read_text(encoding="utf-8")
    prereg = json.loads(raw)
    check("C34 the prereg parses and carries no self-hash",
          prereg["contains_no_self_hash"] is True
          and hashlib.sha256(raw.encode()).hexdigest() not in raw)
    check("C35 the stage token matches the runner", prereg["stage"] == J19C.STAGE)
    check("C36 it declares that preregistration precedes any use of the seeds",
          prereg["preregistration_precedes_any_use_of_the_seeds"]["seeds"] == list(HELDOUT))
    check("C37 the three cells match the runner's matrix exactly",
          [(c["id"], c["mode"], c["seed"], c["offset_s"])
           for c in prereg["the_three_cells"]["cells"]]
          == [(c["id"], c["mode"], c["seed"], c["offset_s"]) for c in J19C.MATRIX])
    check("C38 it declares 14 gates per cell and the 3/3 aggregate",
          prereg["gates"]["per_cell_count"] == 14
          and prereg["gates"]["aggregate_rule"] == J19C.AGGREGATE_RULE)
    check("C39 the penetration bands are stated exactly, 28 passing",
          "exactly 0.028 PASSES" in prereg["penetration_contract"]["hard_binding"])
    check("C40 the closed whitelist has six entries",
          len(prereg["core_scientific_equivalence"]["authorised_differences_closed_whitelist"])
          == 6)

    report = J19C.verify_prereg()
    check("C41 verify_prereg accepts the document", report["schema"] == "J19C")
    check("C42 it pins the J19B entry evidence: PASS on 6 cells",
          report["entry_evidence"]["j19b_verdict"] == "PASS"
          and report["entry_evidence"]["j19b_cells"] == 6)
    check("C43 the J19B receipt and commit verification match their pins",
          report["entry_evidence"]["j19b_receipt_sha256"]
          == sha_file(J19B_LEAF / "v26c_j19b_closed_loop_receipt.json")
          and report["entry_evidence"]["j19b_commit_verification_sha256"]
          == sha_file(J19B_LEAF / "commit_verification.json"))

    original = J19C.PIN_PREREG
    bitten = False
    try:
        J19C.PIN_PREREG = "0" * 64
        J19C.verify_prereg()
    except J19C.J19CError as error:
        bitten = "preregistration changed" in str(error)
    finally:
        J19C.PIN_PREREG = original
    check("C44 a preregistration that does not match its pin is REFUSED", bitten)
    # SEALED. The pin must be the REAL hash of the preregistration on disk, and
    # the guard must be UNCONDITIONAL: no branch may accept "PENDING".
    real = sha_file(PREREG_PATH)
    check("C45a the pin is no longer PENDING", J19C.PIN_PREREG != "PENDING", J19C.PIN_PREREG)
    check("C45b the pin is the REAL hash of the preregistration on disk",
          J19C.PIN_PREREG == real, "pin %s vs file %s" % (J19C.PIN_PREREG[:16], real[:16]))
    check("C45c it is the sealed hash", real ==
          "54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d", real)
    check("C45d the derivation tool seals the same hash",
          any(new.strip('"') == real for kind, _old, new in DERIVE.SUBSTITUTIONS
              if kind == "prereg"),
          [new for kind, _o, new in DERIVE.SUBSTITUTIONS if kind == "prereg"])

    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == "verify_prereg")
    guards = [n for n in ast.walk(fn) if isinstance(n, ast.If)
              and "PIN_PREREG" in ast.dump(n.test)]
    check("C45e the pin guard is a single unconditional comparison",
          len(guards) == 1 and not isinstance(guards[0].test, ast.BoolOp),
          [ast.dump(g.test)[:70] for g in guards])
    check("C45f no branch anywhere accepts PENDING as a valid pin",
          not [n for n in ast.walk(tree) if isinstance(n, ast.Compare)
               and any(isinstance(c, ast.Constant) and c.value == "PENDING"
                       for c in n.comparators)])

    # a MUTATED preregistration must be refused: mutate the file itself, not the pin
    original_bytes = PREREG_PATH.read_bytes()
    refused = None
    try:
        PREREG_PATH.write_bytes(original_bytes + b"\n")
        J19C.verify_prereg()
    except J19C.J19CError as error:
        refused = str(error)
    finally:
        PREREG_PATH.write_bytes(original_bytes)
    check("C45g a MUTATED preregistration file is REFUSED",
          refused is not None and "preregistration changed" in refused, refused)
    check("C45h and the file is restored byte-identically",
          sha_file(PREREG_PATH) == real)
    check("C45i with the real seal verify_prereg passes",
          J19C.verify_prereg()["sha256"] == real)


# --------------------------------------------------------------------------
# C46-C58 - the real preflight, end to end, and total inertia
# --------------------------------------------------------------------------


def test_real_preflight_and_inertia():
    heavy = ("torch", "ray", "opensim", "gymnasium", "env_factory", "rollout_eval")
    before_heavy = sorted(m for m in heavy if m in sys.modules)
    before_tree = {str(p.relative_to(ROOT)): (p.stat().st_size, p.stat().st_mtime_ns)
                   for p in sorted(ROOT.rglob("*")) if p.is_file()}

    report = J19C.preflight()

    after_heavy = sorted(m for m in heavy if m in sys.modules)
    after_tree = {str(p.relative_to(ROOT)): (p.stat().st_size, p.stat().st_mtime_ns)
                  for p in sorted(ROOT.rglob("*")) if p.is_file()}

    check("C46 preflight() runs to completion", isinstance(report, dict))
    check("C47 its verdict is GO", report.get("verdict") == "GO", report.get("verdict"))
    check("C48 it reports no blockers", report.get("blockers") == [], report.get("blockers"))
    check("C49 no heavy module was imported: no ray, torch, OpenSim, gym",
          after_heavy == before_heavy == [], (before_heavy, after_heavy))
    check("C50 the sentinel was never created", not J19C.PREFLIGHT_SENTINEL.exists())
    check("C51 the tree is byte-for-byte unchanged", before_tree == after_tree,
          sorted(set(after_tree) ^ set(before_tree)))
    check("C52 its own matrix policy states 3/3, not 6/6",
          "3/3" in report["matrix_policy"]["aggregate_pass_iff"]
          and "6/6" not in report["matrix_policy"]["aggregate_pass_iff"],
          report["matrix_policy"]["aggregate_pass_iff"])
    check("C53 it would write only the authorised J19C leaf",
          report["would_write"]["relative_leaf"] == J19C.RELATIVE_LEAF)
    check("C54 it promotes nothing and authorises no next stage",
          report["outcome_policy"]["deployable"] is False
          and report["outcome_policy"]["next_stage_authorized"] is False
          and report["outcome_policy"]["ppo_updates"] == 0)

    check("C55 j19c_runs does not exist", not (ROOT / "j19c_runs").exists())
    check("C56 no lock or staging exists for J19C",
          not list(ROOT.glob("j19c_runs/.lock_*")) and not list(ROOT.glob("j19c_runs/.staging_*")))
    check("C57 no J19C GO exists on disk", not list(ROOT.glob("*j19c*go*.json")),
          [p.name for p in ROOT.glob("*j19c*go*.json")])

    for label, leaf, pins in (
        ("J19A", J19A_LEAF, {
            "rl_module/module_state.pkl":
                "8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530",
            "v26c_j19a_single_reproduction_receipt.json":
                "235a117fc849bbe137dfd7ea29621390a6d1aa71aa9f9b4d95ca0e7a5dd50dad"}),
        ("J19B", J19B_LEAF, {
            "v26c_j19b_closed_loop_receipt.json":
                "789c57ca293ae7d5f578d4b3cae4e9aec599fa3edf4ce87239ad5fc8a7b1b509",
            "commit_verification.json":
                "0ded736553a55eba6ac600340750d40aa82f7ebf4689bcdd4717dc3636907776"}),
    ):
        for rel, pin in pins.items():
            check("C58 %s %s is byte-unchanged" % (label, rel),
                  sha_file(leaf / rel) == pin, sha_file(leaf / rel))

    module_names = set(vars(J19C)) | set(vars(builtins))
    unresolved = []

    def walk(code, origin):
        for ins in dis.get_instructions(code):
            if ins.opname in ("LOAD_GLOBAL", "STORE_GLOBAL", "DELETE_GLOBAL") \
                    and ins.argval not in module_names:
                unresolved.append((origin, ins.argval))
        for const in code.co_consts:
            if isinstance(const, types.CodeType):
                walk(const, origin)

    for attr, value in vars(J19C).items():
        if isinstance(value, types.FunctionType) and value.__module__ == J19C.__name__:
            walk(value.__code__, attr)
    check("C59 every global name in the derived runner resolves", not unresolved, unresolved)

    source = RUNNER_PATH.read_text(encoding="utf-8")
    own_six = [ln.strip() for ln in source.splitlines()
               if "6/6" in ln and "J19B" not in ln]
    check("C60 no 6/6 wording survives except in explicit J19B references",
          not own_six, own_six)
    check("C61 no A-F wording survives about this stage's own matrix",
          "order A-F" not in source and "All six cells" not in source)

    # The offset is FROZEN across G, H and I. ENV_MUTATION_POLICY authorises varying
    # it - it is an inherited conservative superset, byte-identical to J19B's - but
    # this stage never exercises that authorisation.
    offsets = {c["offset_s"] for c in J19C.MATRIX}
    check("C62 all three held-out cells share one frozen start offset",
          offsets == {NOMINAL}, sorted(offsets))
    check("C63 that offset is J9R1's frozen nominal, and no perturbed start is used",
          J19C.OFFSET_NOMINAL == NOMINAL
          and 1.756870983805102 not in offsets and 2.156870983805102 not in offsets)
    check("C64 the policy still AUTHORISES varying the offset: it is a superset",
          "episode_start_offset_s"
          in J19C.ENV_MUTATION_POLICY["scientific_or_runtime_fields_mutable"])
    check("C65 and it is byte-identical to J19B's, as the core equivalence requires",
          dict(J19C.ENV_MUTATION_POLICY) == dict(J19B.ENV_MUTATION_POLICY))
    check("C66 the preregistration documents that gap explicitly",
          json.loads(PREREG_PATH.read_text(encoding="utf-8"))["env_mutation_policy_note"]
          ["the_only_field_that_actually_varies_between_J19C_cells"] == "output_dir")
    check("C67 the source comment beside the policy no longer claims a sweep",
          "matrix does NOT sweep" in source
          and "matrix IS a sweep over the three preregistered starts" not in source)

    # No J19C-own prose may claim new starts; J19C tests new NOISE realisations.
    prereg_raw = PREREG_PATH.read_text(encoding="utf-8")
    for label, text in (("the runner", source), ("the preregistration", prereg_raw)):
        offenders = [ln.strip() for ln in text.splitlines()
                     if "never seen" in ln and "J19B" not in ln and "seeds" not in ln]
        check("C68 %s claims no unseen START" % label, not offenders, offenders)
    check("C69 the preregistration states the precise wording instead",
          "does NOT test new start offsets" in prereg_raw
          and "NEW NOISE REALISATIONS" in prereg_raw)

    # ------------------------------------------------------------------
    # C70-C81 - the POLICY REPORT: two labelled sections, and the inherited
    # sentence may never surface as a J19C claim.
    # ------------------------------------------------------------------
    rep = J19C.env_mutation_report()
    check("C70 the report has exactly the two labelled sections",
          sorted(rep) == ["effective_j19c", "inherited_conservative_superset"], sorted(rep))

    inherited, effective = rep["inherited_conservative_superset"], rep["effective_j19c"]
    check("C71 the inherited section reproduces the constant byte-identically",
          inherited["constant"] == dict(J19B.ENV_MUTATION_POLICY)
          and inherited["byte_identical_to_j19b"] is True)
    check("C72 it declares that its why field describes J19B, not J19C",
          inherited["its_why_field_describes_J19B_not_J19C"] is True
          and inherited["this_stage_never_exercises_that_authorisation"] is True)

    check("C73 the effective section says NO scientific or runtime field varies",
          effective["scientific_or_runtime_fields_that_actually_vary"] == []
          and effective["episode_start_offset_s_varies"] is False
          and effective["matrix_is_a_sweep"] is False)
    check("C74 it pins the frozen nominal offset in every cell",
          effective["episode_start_offset_s_value_in_every_cell"] == NOMINAL
          and effective["distinct_offsets_across_the_matrix"] == [NOMINAL])
    check("C75 it names output_dir as the only per-cell difference",
          effective["only_field_differing_between_cells"] == "output_dir"
          and effective["output_dir_is_instrumentation_not_science"] is True)

    # the historical sentence must appear ONLY inside the inherited section
    sweep = "the matrix is a sweep over the three preregistered start offsets"
    check("C76 the historical sentence appears inside the inherited section",
          sweep in json.dumps(inherited))
    check("C77 and NOWHERE in the effective section",
          sweep not in json.dumps(effective))

    # the report must BITE if the matrix ever varied the offset
    original = J19C.MATRIX
    bitten = None
    try:
        mutated = [dict(c) for c in original]
        mutated[1]["offset_s"] = 1.756870983805102
        J19C.MATRIX = tuple(mutated)
        J19C.env_mutation_report()
    except J19C.J19CError as error:
        bitten = str(error)
    finally:
        J19C.MATRIX = original
    check("C78 the report REFUSES a matrix that varies the start offset",
          bitten is not None and "varies the start offset" in bitten, bitten)

    # both call sites - preflight AND the future receipt - must use the helper
    tree = ast.parse(source)
    # The bare constant may be serialised in exactly two places, both legitimate:
    # inside the helper's labelled inherited section, and inside the core
    # equivalence comparison against J19B. Nowhere else - in particular, not in
    # the preflight or the receipt.
    def enclosing(lineno):
        best = None
        for fn in ast.walk(tree):
            if isinstance(fn, ast.FunctionDef) and fn.lineno <= lineno \
                    and lineno <= (fn.end_lineno or fn.lineno):
                if best is None or fn.lineno > best.lineno:
                    best = fn
        return best.name if best else "<module>"

    raw_uses = {enclosing(n.lineno) for n in ast.walk(tree)
                if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "dict"
                and any(getattr(a, "id", "") == "ENV_MUTATION_POLICY" for a in n.args)}
    helper_uses = [n.lineno for n in ast.walk(tree)
                   if isinstance(n, ast.Call)
                   and getattr(n.func, "id", "") == "env_mutation_report"]
    check("C79 the bare constant is serialised only in the helper and the equivalence",
          raw_uses == {"env_mutation_report", "verify_scientific_equivalence"},
          sorted(raw_uses))
    check("C79b neither the preflight nor run_matrix serialises it directly",
          not (raw_uses & {"preflight", "run_matrix"}), sorted(raw_uses))
    check("C80 both the preflight and the receipt go through the helper",
          len(helper_uses) == 2, helper_uses)

    preflight_policy = report["env_mutation_policy"]
    check("C81 the real preflight output carries the two sections, not the bare constant",
          sorted(preflight_policy) == ["effective_j19c", "inherited_conservative_superset"]
          and preflight_policy["effective_j19c"]["episode_start_offset_s_varies"] is False,
          sorted(preflight_policy))
    check("C82 the preflight never presents the sweep sentence as its own",
          sweep not in json.dumps(preflight_policy["effective_j19c"]))

    forbidden_env = [f for f in J19C.FORBIDDEN_HERE if "env field" in f]
    check("C83 FORBIDDEN_HERE no longer claims the matrix varies the offset",
          len(forbidden_env) == 1
          and "ONLY one the matrix varies" not in forbidden_env[0]
          and "in J19C NONE varies" in forbidden_env[0], forbidden_env)


def main():
    test_derivation_is_mechanical()
    test_core_equivalence()
    test_negative_alternatives_are_refused()
    test_no_cli_can_change_the_protocol()
    test_prereg_and_entry_evidence()
    test_real_preflight_and_inertia()

    failed = [c for c in CHECKS if not c[1]]
    for name, passed, detail in CHECKS:
        print("[%s] %-64s %s" % ("PASS" if passed else "FAIL", name, detail))
    print("\n%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
