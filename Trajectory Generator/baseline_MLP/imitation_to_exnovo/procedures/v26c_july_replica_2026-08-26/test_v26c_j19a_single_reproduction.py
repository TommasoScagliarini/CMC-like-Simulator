"""Static and dry-run tests for V26C J19A.

Runs NO fit and NO rollout. The only heavy work is the preflight, which is
itself inert. ``sys.dont_write_bytecode`` is set before the runner import so the
write-nothing snapshot is not defeated by ``__pycache__``.
"""

from __future__ import annotations

import ast
import builtins
import dis
import hashlib
import json
import math
import pathlib
import sys
import types

sys.dont_write_bytecode = True

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import v26c_j19a_single_reproduction as J19A  # noqa: E402

RUNNER_PATH = ROOT / "v26c_j19a_single_reproduction.py"
PREREG_PATH = ROOT / "v26c_j19a_prereg_single_reproduction.json"

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), str(detail)))


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def snapshot(root):
    out = {}
    for path in sorted(root.rglob("*")):
        try:
            stat = path.stat()
        except OSError:
            continue
        out[str(path.relative_to(root))] = (stat.st_size, stat.st_mtime_ns)
    return out


def test_namespace_closure():
    names = set(vars(J19A)) | set(vars(builtins))
    unresolved = []

    def walk(code, origin):
        for ins in dis.get_instructions(code):
            if ins.opname in ("LOAD_GLOBAL", "STORE_GLOBAL", "DELETE_GLOBAL"):
                if ins.argval not in names:
                    unresolved.append((origin, ins.argval))
        for const in code.co_consts:
            if isinstance(const, types.CodeType):
                walk(const, origin)

    for attr, value in vars(J19A).items():
        if isinstance(value, types.FunctionType) and value.__module__ == J19A.__name__:
            walk(value.__code__, attr)
    check("S01 every global name in the runner resolves", not unresolved, unresolved)


def test_preflight_is_inert():
    before = snapshot(ROOT)
    report = J19A.run_preflight(ROOT, verbose=False)
    after = snapshot(ROOT)
    check("S02 preflight leaves the tree byte-for-byte unchanged", before == after,
          sorted(set(after) ^ set(before)))
    check("S03 preflight trained nothing", report["trained_anything"] is False)
    check("S04 preflight wrote nothing", report["wrote_anything"] is False)
    check("S05 preflight created no directory", report["created_any_directory"] is False)
    check("S06 preflight performed no rollout", report["performed_any_rollout"] is False)
    check("S07 preflight passes every check", report["ok"],
          "%d/%d" % (report["passed"], report["total"]))
    check("S08 preflight never imported torch", "torch" not in sys.modules)
    return report


def test_no_rollout_reachable():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    forbidden = {"env_factory", "gymnasium", "gym", "ray", "rllib", "opensim",
                 "v26c_j1_collect", "v26c_j9r1_closed_loop", "v26c_j12_closed_loop",
                 "v26c_j16_closed_loop", "v26c_penetration_contract"}
    check("S09 the runner imports no environment or closed-loop module",
          not (imported & forbidden), sorted(imported & forbidden))
    check("S10 the runner's imports are exactly the expected set",
          imported <= {"__future__", "argparse", "hashlib", "json", "math", "pathlib",
                       "pickle", "sys", "numpy", "os", "torch",
                       "v26c_j18_b_only_update"},
          sorted(imported))
    # Structural, not substring: the runner DOES mention "rollout", but only to
    # deny it (rollout_performed: False, authorises_rollout refused). What must
    # be absent is any CALL into rollout or matrix machinery.
    called = {node.func.id for node in ast.walk(tree)
              if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    called |= {node.func.attr for node in ast.walk(tree)
               if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    machinery = {"run_matrix", "cell_env_config", "base_env_config", "evaluate_cell_gate",
                 "cell_verdict", "penetration_report", "evaluate_series", "make_env",
                 "run_cell", "rollout"}
    check("S11a no call into rollout, cell or matrix machinery",
          not (called & machinery), sorted(called & machinery))
    check("S11b no rollout or matrix identifier is defined or referenced",
          not any(t in source for t in ("run_matrix", "cell_env_config", "MATRIX",
                                        "penetration_report")),
          [t for t in ("run_matrix", "cell_env_config", "MATRIX", "penetration_report")
           if t in source])
    # A mention counts as a denial when the line, or the line that follows it,
    # asserts the absence. A guard like `if payload.get("authorises_rollout"):`
    # denies on its next line, so a single-line window would misjudge it.
    tokens = ("False", "never", "NOT", "NO ", "not ", "no rollout", "separate",
              "refus", "no_rollout", "must never")
    lines = source.splitlines()
    offenders = []
    for i, line in enumerate(lines):
        if "rollout" not in line.lower() or line.strip().startswith("#"):
            continue
        window = " ".join(lines[i:i + 2])
        if not any(t in window for t in tokens):
            offenders.append(line.strip())
    check("S11c every mention of rollout denies it", not offenders, offenders)
    check("S12 J19B is declared unreachable from this module",
          "not reachable from here" in source)


def test_j18_is_pinned_not_modified():
    actual = J19A.sha256_file(ROOT / J19A.J18_RUNNER_NAME)
    check("S13 the J18 runner still matches its pin",
          actual == J19A.J18_RUNNER_SHA256, actual)
    for name, expected in sorted(J19A.J18_LEAF_ARTEFACTS.items()):
        got = J19A.sha256_file(ROOT / J19A.J18_LEAF / name)
        check("S14 J18 leaf artefact %s unchanged" % name, got == expected, got)

    raised = False
    try:
        original = J19A.J18_RUNNER_SHA256
        J19A.J18_RUNNER_SHA256 = "0" * 64
        J19A.load_j18(ROOT)
    except RuntimeError as error:
        raised = "reproduction is meaningless" in str(error)
    finally:
        J19A.J18_RUNNER_SHA256 = original
    check("S15 load_j18 REFUSES a J18 runner whose bytes changed", raised)

    for historical in ("v26c_j16_closed_loop.py", "v26c_j12_closed_loop.py",
                       "v26c_j9r1_closed_loop.py", "v26c_penetration_contract.py"):
        check("S16 historical runner %s is not touched by this module" % historical,
              historical not in RUNNER_PATH.read_text(encoding="utf-8"))


def test_frozen_reference_is_read_not_hardcoded(report):
    source = RUNNER_PATH.read_text(encoding="utf-8")
    frozen = report["frozen_metrics"]
    check("S17 the six frozen metrics are READ from the pinned leaf, not hardcoded",
          not any(repr(v)[:12] in source for v in frozen.values()),
          [k for k, v in frozen.items() if repr(v)[:12] in source])

    record = J19A.frozen_record(ROOT)
    check("S18 the frozen record is candidate 13",
          record["candidate"]["candidate_index"] == 13)
    check("S19 its best_epoch is exactly 191", record["best_epoch"] == 191)
    check("S20 its step contract is 6600 over 33 batches",
          record["optimizer_steps"] == 6600 and record["batches_per_epoch"] == 33)


def test_reproducibility_contract(report):
    frozen = report["frozen_metrics"]
    exact = {"best_epoch": 191, "optimizer_steps": 6600, "batches_per_epoch": 33}

    same = J19A.compare_reproduction(dict(frozen), frozen, dict(exact), exact)
    check("S21 an identical result reproduces", same["ok"])
    check("S22 the comparison is math.isclose with the registered tolerances",
          "rel_tol=1e-06" in same["comparison"] and "abs_tol=1e-09" in same["comparison"],
          same["comparison"])
    check("S23 it is declared a numerical tolerance, not a performance gate",
          same["is_numerical_tolerance_not_a_performance_gate"] is True)

    # a relative change just inside and just outside the tolerance
    inside = dict(frozen); inside["MSE_B"] = frozen["MSE_B"] * (1 + 5e-07)
    outside = dict(frozen); outside["MSE_B"] = frozen["MSE_B"] * (1 + 5e-05)
    check("S24 a 5e-07 relative change still reproduces",
          J19A.compare_reproduction(inside, frozen, dict(exact), exact)["ok"])
    check("S25 a 5e-05 relative change FAILS reproduction",
          not J19A.compare_reproduction(outside, frozen, dict(exact), exact)["ok"])

    # a BETTER result must also fail: this is not a performance gate
    better = dict(frozen); better["MSE_B"] = frozen["MSE_B"] * 0.5
    check("S26 a BETTER result also fails, proving it is not a performance gate",
          not J19A.compare_reproduction(better, frozen, dict(exact), exact)["ok"])

    for field, wrong in (("best_epoch", 190), ("optimizer_steps", 6599),
                         ("batches_per_epoch", 32)):
        bad = dict(exact); bad[field] = wrong
        check("S27 an exact-field mismatch on %s FAILS" % field,
              not J19A.compare_reproduction(dict(frozen), frozen, bad, exact)["ok"])


def test_eligibility_rule(report):
    ceilings = report["j8_ceilings"]
    prereg = load_json(PREREG_PATH)
    binding = {b["id"]: b for b in prereg["offline_eligibility"]["binding"]}

    check("S28 the J8 ceilings recomputed at preflight match the preregistration",
          ceilings["ok"] is True)
    check("S29 G3's ceiling is J8's own measured bias on C",
          math.isclose(ceilings["C"]["bias"], binding["G3"]["threshold"], rel_tol=0, abs_tol=1e-15),
          ceilings["C"]["bias"])
    check("S30 G4's ceiling is J8's own measured bias on D",
          math.isclose(ceilings["D"]["bias"], binding["G4"]["threshold"], rel_tol=0, abs_tol=1e-15),
          ceilings["D"]["bias"])
    check("S31 E-C is J8's own measured RMSE on C",
          math.isclose(ceilings["C"]["rmse"], binding["E-C"]["threshold"], rel_tol=0, abs_tol=1e-15))
    check("S32 E-D is J8's own measured RMSE on D",
          math.isclose(ceilings["D"]["rmse"], binding["E-D"]["threshold"], rel_tol=0, abs_tol=1e-15))
    check("S33 0.0014557 appears nowhere in the runner",
          "0.0014557" not in RUNNER_PATH.read_text(encoding="utf-8"))

    diagnostic = {d["id"] for d in prereg["offline_eligibility"]["diagnostic_only_never_binding"]}
    check("S34 G1, G2 and POST_485 are declared diagnostic only",
          {"G1", "G2", "POST_485"} <= diagnostic, sorted(diagnostic))
    check("S35 no binding criterion is a max-drift",
          not any("max_drift" in b["name"] for b in prereg["offline_eligibility"]["binding"]))

    # the frozen candidate-13 metrics must satisfy the rule
    record = J19A.frozen_record(ROOT)
    m = record["metrics"]
    bias = lambda b: max(abs(m[b]["knee"]["signed_mean_shift"]),
                         abs(m[b]["ankle"]["signed_mean_shift"]))
    check("S36 c13's frozen bias C is inside J8's ceiling",
          bias("C") <= ceilings["C"]["bias"],
          "%.10g <= %.10g (%.3fx)" % (bias("C"), ceilings["C"]["bias"],
                                      bias("C") / ceilings["C"]["bias"]))
    check("S37 c13's frozen bias D is inside J8's ceiling",
          bias("D") <= ceilings["D"]["bias"],
          "%.10g <= %.10g (%.3fx)" % (bias("D"), ceilings["D"]["bias"],
                                      bias("D") / ceilings["D"]["bias"]))
    check("S38 c13's frozen rmse C is inside J8's ceiling",
          m["C"]["rmse"] <= ceilings["C"]["rmse"])
    check("S39 c13's frozen rmse D is inside J8's ceiling",
          m["D"]["rmse"] <= ceilings["D"]["rmse"])
    check("S40 c13's frozen max drift EXCEEDS the diagnostic reference, and that is "
          "no longer binding",
          m["C"]["max_abs"] > J19A.DIAGNOSTIC_MAX_DRIFT_REFERENCE, m["C"]["max_abs"])


def test_go_and_persistence():
    good = {"stage": J19A.GO_REQUIRED_STAGE, "authorises_execution": True,
            "pinned_artefacts_sha256": {n: J19A.sha256_file(ROOT / n)
                                        for n in J19A.GO_REQUIRED_PINS}}
    check("S41 a correctly pinned in-memory GO validates", J19A.validate_go(good)["valid"],
          J19A.validate_go(good)["problems"])
    check("S42 a GO that authorises a rollout is REFUSED",
          not J19A.validate_go(dict(good, authorises_rollout=True))["valid"])
    check("S43 a GO for another stage is refused",
          not J19A.validate_go(dict(good, stage="V26C_J19B"))["valid"])
    missing = {k: v for k, v in good["pinned_artefacts_sha256"].items()
               if k != J19A.J18_RUNNER_NAME}
    check("S44 a GO not pinning the J18 runner is refused",
          not J19A.validate_go(dict(good, pinned_artefacts_sha256=missing))["valid"])
    stale = dict(good["pinned_artefacts_sha256"]); stale[J19A.PREREG_PATH] = "0" * 64
    check("S45 a GO with a stale pin is refused",
          not J19A.validate_go(dict(good, pinned_artefacts_sha256=stale))["valid"])
    check("S46 a missing GO file is refused",
          not J19A.load_go(str(ROOT / "no_such_go.json"))["valid"])
    check("S47 no J19A GO file exists on disk",
          not [p for p in ROOT.glob("*j19a*go*.json")],
          [p.name for p in ROOT.glob("*j19a*go*.json")])

    raised = False
    try:
        J19A.run_reproduction("nonexistent-go.json")
    except RuntimeError as error:
        raised = "GO is absent or invalid" in str(error)
    check("S48 run_reproduction refuses without a valid GO", raised)
    check("S49 --execute without a GO file returns failure", J19A.main(["--execute"]) == 1)
    check("S50 no j19a leaf exists", not (ROOT / J19A.LEAF_ROOT / J19A.LEAF_NAME).exists())

    tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "finalise")
    guarded = [n for n in ast.walk(fn) if isinstance(n, ast.If) and "promote" in ast.dump(n.test)]
    check("S51 the actor is written only inside a promote guard", len(guarded) == 1, len(guarded))


def test_prereg_is_coherent():
    raw = PREREG_PATH.read_text(encoding="utf-8")
    prereg = json.loads(raw)
    check("S52 the prereg parses and carries no self-hash",
          prereg["contains_no_self_hash"] is True
          and hashlib.sha256(raw.encode()).hexdigest() not in raw)
    check("S53 all seven architect decisions are encoded",
          [d["id"] for d in prereg["architect_decisions_encoded"]]
          == ["D1", "D2", "D3", "D4", "D5", "D6", "D7"])
    check("S54 the phase separation forbids chaining fit and rollout",
          "NEVER chained" in prereg["phase_separation"]["rule"])
    check("S55 J19B is declared described-only and not implemented",
          prereg["phase_separation"]["j19b_is_not_implemented_in_this_stage"] is True)
    check("S56 the closed-loop bands are recorded exactly as the current contract",
          "28 passes" in prereg["architect_decisions_encoded"][6]["decision"]
          and "20 mm soft" in prereg["architect_decisions_encoded"][6]["decision"])
    check("S57 the prereg declares the J18-import inversion for architect ruling",
          prereg["training_code_provenance"]["declared_for_architect_ruling"] is True)
    check("S58 historical runners are declared immutable",
          any("immutable evidence" in s for s in prereg["invariants"]["not_modified"]))


# --------------------------------------------------------------------------
# S59-S80 - the three audit corrections: full exact-field comparison, G11 over
# every published diagnostic, and self-contained preflight hermeticity
# --------------------------------------------------------------------------


def test_exact_field_comparison(report):
    exact = report["exact_fields"]
    names = [f["field"] for f in exact["fields"]]

    check("S59 the exact comparison passes on the frozen inputs", exact["ok"],
          exact["failed_fields"])
    check("S60 the seed is compared exactly", "protocol.seed" in names)
    check("S61 all eight candidate/config fields are compared",
          [n for n in names if n.startswith("candidate.")] == [
              "candidate.candidate_index", "candidate.learning_rate",
              "candidate.on_policy_weight_beta", "candidate.preservation_weight_lambda",
              "candidate.w_A", "candidate.w_B", "candidate.w_C", "candidate.w_D"],
          [n for n in names if n.startswith("candidate.")])
    check("S62 all four block row counts are compared",
          sorted(n for n in names if n.startswith("rows.")) ==
          ["rows.A", "rows.B", "rows.C", "rows.D"])
    check("S63 all four observation hashes are compared",
          len([n for n in names if n.startswith("observations_sha256.")]) == 4)
    check("S64 all four label hashes are compared",
          len([n for n in names if n.startswith("labels_sha256.")]) == 4)
    check("S65 the parent bytes and digest are compared",
          "parent_j8_module_sha256" in names and "parent_j8_actor_digest" in names)
    check("S66 the pinned provenance is compared",
          len([n for n in names if n.startswith("provenance.")]) >= 7,
          [n for n in names if n.startswith("provenance.")])
    check("S67 the comparison is declared exact and fail-closed",
          "exact equality" in exact["relation"] and "fail-closed" in exact["relation"])

    # the historical cross-platform distinction must be recorded on block D
    d_label = [f for f in exact["fields"] if f["field"] == "labels_sha256.D"][0]
    check("S68 block D's computed labels carry the historical distinction",
          "note" in d_label and "CROSS-PLATFORM" in d_label["note"]
          and "1e-06" in d_label["note"], d_label.get("note", "")[:80])
    check("S69 blocks read from file carry no such note",
          all("note" not in f for f in exact["fields"]
              if f["field"] in ("labels_sha256.A", "labels_sha256.B", "labels_sha256.C")))
    check("S70 the label-hash policy is declared",
          exact["label_hash_policy"]["j18_cross_platform_tolerance"] == 1e-06)

    # the comparison must actually bite
    import v26c_j18_b_only_update as j18
    built = j18.build_blocks(ROOT)
    record = J19A.frozen_record(ROOT)
    tampered = json.loads(json.dumps(record))
    tampered["candidate"]["w_C"] = 29.0
    bitten = J19A.compare_exact_fields(j18, built, tampered, ROOT)
    check("S71 a single altered config field FAILS the exact comparison",
          not bitten["ok"] and bitten["failed_fields"] == ["candidate.w_C"],
          bitten["failed_fields"])

    check("S72 the receipt records the exact comparison, not just the preflight",
          '"exact_fields_ok": bool(safe_payload["exact_fields"]["ok"])'
          in RUNNER_PATH.read_text(encoding="utf-8"))


def test_g11_covers_every_published_diagnostic():
    import v26c_j18_b_only_update as j18
    built = j18.build_blocks(ROOT)
    parent = built["state"]
    metrics = j18.evaluate_all_blocks(parent, built["blocks"])
    ceilings, _ = J19A.j8_reference_ceilings(j18, ROOT)

    def full_payload():
        """A complete, finite g11_payload with every published family."""
        return {
            "candidate": {"candidate_index": 13, "learning_rate": 5e-05,
                          "on_policy_weight_beta": 1.0,
                          "preservation_weight_lambda": 30.0,
                          "w_A": 1.0, "w_B": 1.0, "w_C": 30.0, "w_D": 30.0},
            "training_diagnostics": {
                "best_epoch": 191, "best_objective": 0.00126,
                "optimizer_steps": 6600, "batches_per_epoch": 33,
                "scale_absorption": {"numpy_max_abs_diff": 1e-09,
                                     "torch_max_abs_diff": 1.2e-07},
                "post_485_diagnostic": {"max_abs": 0.017, "rmse": 0.0089},
                "history": [{"epoch": 1, "composite_objective": 0.1,
                             "train_loss_mean": 0.2}]},
            "j8_ceilings": {"C": {"rmse": 0.00447, "bias": 0.000899},
                            "D": {"rmse": 0.00526, "bias": 0.000726}},
            "raw_vs_scaled_equivalence": {"numpy_max_abs_diff": 3.2e-10,
                                          "torch_max_abs_diff": 1.19e-07},
            "reproducibility": {"numeric": [{"field": "MSE_B", "abs_delta": 0.0}],
                                "ok": True},
            "exact_fields": {"count": 30, "failed_fields": [], "ok": True},
        }

    raised = False
    try:
        J19A.evaluate_eligibility(j18, parent, metrics, parent, ceilings, None)
    except RuntimeError as error:
        raised = "must cover every published numeric field" in str(error)
    check("S73a evaluate_eligibility REFUSES to run without a g11_payload", raised)

    for family in J19A.G11_PAYLOAD_FAMILIES:
        truncated = {k: v for k, v in full_payload().items() if k != family}
        raised = False
        try:
            J19A.evaluate_eligibility(j18, parent, metrics, parent, ceilings, truncated)
        except RuntimeError as error:
            raised = family in str(error) and "would not cover them" in str(error)
        check("S73b a payload missing '%s' is REFUSED" % family, raised)

    ok = J19A.evaluate_eligibility(j18, parent, metrics, parent, ceilings, full_payload())
    check("S74 G11 passes on a fully finite payload",
          {g["id"]: g for g in ok["binding"]}["G11"]["passed"])

    # every family must be able to fail the gate
    spoils = [
        ("candidate", lambda p: p["candidate"].__setitem__("w_C", float("nan"))),
        ("training_diagnostics.history",
         lambda p: p["training_diagnostics"]["history"][0].__setitem__(
             "composite_objective", float("nan"))),
        ("training_diagnostics.post_485",
         lambda p: p["training_diagnostics"]["post_485_diagnostic"].__setitem__(
             "max_abs", float("inf"))),
        ("training_diagnostics.best_objective",
         lambda p: p["training_diagnostics"].__setitem__("best_objective", float("nan"))),
        ("training_diagnostics.scale_absorption",
         lambda p: p["training_diagnostics"]["scale_absorption"].__setitem__(
             "numpy_max_abs_diff", float("-inf"))),
        ("j8_ceilings", lambda p: p["j8_ceilings"]["C"].__setitem__("bias", float("nan"))),
        ("raw_vs_scaled_equivalence",
         lambda p: p["raw_vs_scaled_equivalence"].__setitem__(
             "torch_max_abs_diff", float("inf"))),
        ("reproducibility",
         lambda p: p["reproducibility"]["numeric"][0].__setitem__(
             "abs_delta", float("nan"))),
        ("exact_fields",
         lambda p: p["exact_fields"].__setitem__("count", float("nan"))),
    ]
    for label, spoil in spoils:
        payload = full_payload()
        spoil(payload)
        bad = J19A.evaluate_eligibility(j18, parent, metrics, parent, ceilings, payload)
        gate = {x["id"]: x for x in bad["binding"]}["G11"]
        check("S75 G11 FAILS on a non-finite value in %s" % label, not gate["passed"])
        check("S76 eligibility FAILS on %s, so promote cannot be true" % label,
              not bad["ok"])

    source = RUNNER_PATH.read_text(encoding="utf-8")
    check("S77 POST-485 is computed BEFORE the gate",
          source.index("post485 = post_485_diagnostic(") <
          source.index("eligibility = evaluate_eligibility("))
    tree = ast.parse(source)
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == "run_reproduction")
    call = next(n for n in ast.walk(fn) if isinstance(n, ast.Call)
                and getattr(n.func, "id", "") == "evaluate_eligibility")
    check("S78 the single g11_payload is the last argument reaching the gate",
          isinstance(call.args[-1], ast.Name) and call.args[-1].id == "g11_payload",
          ast.dump(call.args[-1])[:60])
    assigned = [n for n in ast.walk(fn) if isinstance(n, ast.Assign)
                and isinstance(n.targets[0], ast.Name)
                and n.targets[0].id == "g11_payload"]
    check("S78b g11_payload is built once, before the gate",
          len(assigned) == 1 and assigned[0].lineno < call.lineno,
          "built@%s call@%s" % (assigned[0].lineno if assigned else None, call.lineno))
    check("S79 promotion is impossible when sanitisation reports findings",
          "promote = bool(promote and not findings)" in source)
    check("S80 promote also requires the exact-field comparison",
          'promote = bool(repro["ok"] and exact["ok"] and eligibility["ok"])' in source)


def test_only_safe_values_are_published():
    """Nothing raw reaches the result; history.json comes from the safe payload."""
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == "finalise")

    check("S85 finalise sanitises the whole g11_payload in one pass",
          'j18.sanitise_non_finite(g11_payload, "$.g11_payload")' in source)
    check("S86 history.json is built from the SANITISED history, not the raw one",
          'encode_json({"epoch_history": safe_training["history"]})' in source
          and 'trained["history"]' not in source.split("def finalise")[1])
    check("S87 safe_training is derived from the sanitised payload",
          'safe_training = safe_payload["training_diagnostics"]' in source)

    # every value published in the result must be safe, derived, or constant
    result_dict = None
    for node in ast.walk(fn):
        if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Subscript)
                and isinstance(node.value, ast.Call)
                and getattr(node.value.func, "id", "") == "encode_json"
                and isinstance(node.value.args[0], ast.Dict)):
            keys = [k.value for k in node.value.args[0].keys if isinstance(k, ast.Constant)]
            if "actor_promoted" in keys:
                result_dict = node.value.args[0]
    check("S88 the result payload was located for inspection", result_dict is not None)

    # The criterion is not "which names are allowed" but "no RAW, unsanitised
    # runtime object is published". These are the names holding raw values.
    RAW = {"metrics", "trained", "g11_payload", "eligibility", "ceilings",
           "equivalence", "repro", "exact", "diagnostics", "post485",
           "safe_payload_raw"}

    def raw_names(node):
        return {n.id for n in ast.walk(node) if isinstance(n, ast.Name) and n.id in RAW}

    offenders = []
    if result_dict is not None:
        for key, value in zip(result_dict.keys, result_dict.values):
            leaked = raw_names(value)
            if leaked:
                offenders.append((getattr(key, "value", "?"), sorted(leaked)))
    check("S89 no raw unsanitised object is published in the result",
          not offenders, offenders)

    families = {getattr(k, "value", "") for k in (result_dict.keys if result_dict else [])}
    check("S90 the result publishes exactly the families G11 covers, plus metrics",
          {"candidate", "reproducibility", "exact_fields", "eligibility", "metrics",
           "diagnostics", "j8_ceilings", "raw_vs_scaled_equivalence"} <= families,
          sorted(families))

    receipt_dict = None
    for node in ast.walk(fn):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id == "receipt" and isinstance(node.value, ast.Dict):
            receipt_dict = node.value
    check("S91 the receipt was located for inspection", receipt_dict is not None)
    receipt_offenders = []
    if receipt_dict is not None:
        for key, value in zip(receipt_dict.keys, receipt_dict.values):
            leaked = raw_names(value)
            if leaked:
                receipt_offenders.append((getattr(key, "value", "?"), sorted(leaked)))
    check("S92 no raw unsanitised object reaches the receipt",
          not receipt_offenders, receipt_offenders)

    # and positively: every numeric-looking receipt field is wrapped in a
    # derivation, so no raw float can reach it even in principle
    wrapped = []
    if receipt_dict is not None:
        for key, value in zip(receipt_dict.keys, receipt_dict.values):
            name = getattr(key, "value", "")
            if name.endswith(("_ok", "_count", "_compared", "_persisted", "_failed",
                              "_non_finite")):
                fn_name = getattr(getattr(value, "func", None), "id", "")
                wrapped.append((name, fn_name or type(value).__name__))
    check("S93 every numeric or boolean receipt field is a derivation",
          all(k in ("bool", "int", "list", "len", "IfExp") for _, k in wrapped),
          wrapped)


def test_preflight_hermeticity_is_self_contained():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    flag_line = None
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Attribute)
                and node.targets[0].attr == "dont_write_bytecode"):
            flag_line = node.lineno
    check("S81 the runner sets sys.dont_write_bytecode at module level",
          flag_line is not None, flag_line)

    import_line = source.index("import v26c_j18_b_only_update")
    check("S82 the flag is set BEFORE the dynamic J18 import can occur",
          flag_line is not None
          and flag_line < source[:import_line].count("\n") + 1,
          "flag@%s import@%s" % (flag_line, source[:import_line].count("\n") + 1))
    check("S83 the flag is set at import time, not inside a function",
          any(isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
              and n.targets[0].attr == "dont_write_bytecode" for n in tree.body))
    check("S84 J19A itself has the flag active", sys.dont_write_bytecode is True)


def main():
    test_namespace_closure()
    report = test_preflight_is_inert()
    test_no_rollout_reachable()
    test_j18_is_pinned_not_modified()
    test_frozen_reference_is_read_not_hardcoded(report)
    test_reproducibility_contract(report)
    test_eligibility_rule(report)
    test_go_and_persistence()
    test_prereg_is_coherent()
    test_exact_field_comparison(report)
    test_g11_covers_every_published_diagnostic()
    test_only_safe_values_are_published()
    test_preflight_hermeticity_is_self_contained()

    failed = [c for c in CHECKS if not c[1]]
    for name, passed, detail in CHECKS:
        print("[%s] %-64s %s" % ("PASS" if passed else "FAIL", name, detail))
    print("\n%d/%d checks passed" % (len(CHECKS) - len(failed), len(CHECKS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
