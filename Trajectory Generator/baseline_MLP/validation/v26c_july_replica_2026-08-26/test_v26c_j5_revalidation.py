"""Fail-closed tests for the V26C J5 revalidation tooling.

The rollout is exercised end to end against a FAKE stack (the same doubles the J3 runner suite
uses). No physical rollout, no fit, no optimizer. Every pinned artefact is read, never written:
mutations are proven on temporary copies or by in-memory pin edits that are restored.

These tests deliberately do NOT assert that j5_runs is empty: once the architect authorises the
real rollout, the same suite must still pass and must be able to verify the produced leaf.
"""
from __future__ import annotations
import ast, builtins, io, json, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j5_revalidation as J5  # noqa: E402
import v26c_j3_closed_loop as J3  # noqa: E402
import v26c_j1_collect as J1  # noqa: E402
from test_v26c_j3_runner import FakeEnv, FakeModule, _FakeTorch  # noqa: E402

FAILCLOSED = (J5.J5Error, J3.J3Error, J1.J1Error)
CHECKS = 0


def check(c, w):
    global CHECKS
    assert c, w
    CHECKS += 1


def expect(fn, exc, w):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


NAMES = tuple(json.loads((J5.J4_MODULE_DIR / "actor_feature_manifest.json").read_text()
                         )["actor_feature_names"])


def _stack(env, module, tm, **kw):
    return J3._Stack(name="fake", operational=False, torch_mod=tm,
                     load_module=lambda p: module, make_env=lambda cfg: env, **kw)


def _run(td, env, module, tm, *, leaf):
    return J5.run(authorized_stage=J5.STAGE, out_dir=Path(td) / leaf,
                  stack=_stack(env, module, tm), progress=False)


def main() -> int:
    src = (HERE / "v26c_j5_revalidation.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    tm = _FakeTorch
    amendment = json.loads(J5.AMENDMENT.read_text())

    # ---------------------------------------------------------------- the J5 amendment ----------
    check(amendment["kind"] == "ADDITIVE, IMMUTABLE"
          and amendment["stage_authorised"] == "V26C-J5-REVALIDATION",
          "the J5 amendment is additive and names its own stage")
    check(amendment["authorised_by"].startswith("THE USER, explicitly"),
          "the authorisation is attributed to the user")
    dnr = amendment["does_not_rewrite"]
    check(dnr["j4_amendment_sha256"]
          == "fed5b81666782902ed4ab0187da457cdfbf0516dd676c049fdfe41e48d21614f"
          and J5._sha_file(HERE / dnr["j4_amendment_file"]) == dnr["j4_amendment_sha256"],
          "and the J4 amendment is untouched, pinned by its own hash")
    check(dnr["j1_verdict_unchanged"] == "FAIL" and dnr["j3_verdict_unchanged"] == "FAIL"
          and "TECHNICAL/OFFLINE" in dnr["j4_verdict_unchanged"],
          "the J1/J3 FAILs and the J4 technical PASS are declared unchanged")
    check(len(amendment["pinned_artefacts_sha256"]) == 16,
          f"it pins 16 artefacts, got {len(amendment['pinned_artefacts_sha256'])}")
    for rel, pin in amendment["pinned_artefacts_sha256"].items():
        check((HERE / rel).is_file() and J5._sha_file(HERE / rel) == pin,
              f"the amendment pins the REAL hash of {rel}")
    check(len([k for k in amendment["pinned_artefacts_sha256"]
               if k.startswith("j4_runs/j4_recovery_v26c_2026-08-26_r1/")]) == 8,
          "including all eight files of the finalised J4 leaf")
    check(amendment["operational_actor"]["hard_zero_columns"] == [0, 1]
          and amendment["operational_actor"]["trainable_controller_columns"] == list(range(25, 35))
          and amendment["operational_actor"]["controller_columns_must_be_nonzero"] is True,
          "and states the J5 mask: clock hard-zero, controller live")
    check(amendment["outcome_policy"]["on_pass"] == {"deployable": False, "promotion": "NONE",
                                                     "next_stage_authorized": False}
          and amendment["outcome_policy"]["automatic_promotion"] == "NONE, under any outcome",
          "a PASS promotes nothing")
    check(amendment["gates"]["common"] == dict(J3.J3_COMMON_GATE)
          and amendment["gates"]["hard_guard_m"] == 0.028
          and amendment["gates"]["soft_diagnostic_m"] == 0.020,
          "the amendment repeats the J3 gates verbatim")

    # ---------------------------------------------------------------- pins and lineage ----------
    actor = J5.verify_j4_actor()
    check(len(J5.PIN_J4) == 8 and actor["artefacts_sha256"] == J5.PIN_J4,
          "the eight J4 artefacts are pinned and every one matches")
    check(actor["verdict"] == "PASS" and "POST-CRASH FINALIZED" in actor["verdict_kind"],
          "the J4 receipt is the finalised technical PASS")
    check(actor["selection"]["best_epoch"] == 9 and actor["selection"]["epochs_run"] == 69,
          "and J5 reads its selection from that receipt")
    for rel in list(J5.PIN_J4):
        saved = J5.PIN_J4[rel]
        J5.PIN_J4[rel] = "0" * 64
        e = expect(J5.verify_j4_actor, J5.J5Error, f"a mutated {rel} must be refused")
        check(rel in str(e), f"and the failure names {rel}")
        J5.PIN_J4[rel] = saved
    saved_leaf = J5.J4_LEAF
    J5.J4_LEAF = HERE / "j4_runs" / "does_not_exist"
    expect(J5.verify_j4_actor, J5.J5Error, "a missing J4 leaf must be refused")
    J5.J4_LEAF = saved_leaf
    with tempfile.TemporaryDirectory() as td0:
        copy = Path(td0) / "j4_copy"
        shutil.copytree(J5.J4_LEAF, copy)
        J5.J4_LEAF = copy
        check(J5.verify_j4_actor()["artefacts_sha256"] == J5.PIN_J4,
              "a faithful COPY of the J4 leaf verifies clean")
        (copy / "_stray.txt").write_text("x", encoding="utf-8")
        e = expect(J5.verify_j4_actor, J5.J5Error, "an EXTRA file in the J4 leaf is refused")
        check("_stray.txt" in str(e), "and the failure names it")
    J5.J4_LEAF = saved_leaf
    check(J5.verify_j4_actor()["artefacts_sha256"] == J5.PIN_J4,
          "the production J4 leaf was never written to")

    lin = J5.verify_lineage()
    check(lin["j1_verdict"] == "FAIL" and lin["j3_verdict"] == "FAIL"
          and lin["j3_failed"] == ["max_penetration_m"] and lin["j4_verdict"] == "PASS",
          "J1 FAIL, J3 FAIL and the J4 PASS are all read back from their own receipts")
    check(lin["j1_max_penetration_m"] == 0.02294380435912411
          and lin["j3_max_penetration_m"] == 0.02704966381076714,
          "with their measured penetrations unchanged")
    check("neither re-scores nor retroactively promotes" in lin["statement"],
          "and J5 declares that it promotes none of them")
    a2_entries = json.loads(J5.AMENDMENT_R2.read_text())["pinned_artefacts_sha256"]
    union = {**amendment["pinned_artefacts_sha256"], **a2_entries}
    check(len(amendment["pinned_artefacts_sha256"]) == 16,
          "the J5 authorisation amendment pins 16 artefacts")
    check(lin["j5_amendment_manifest_sha256"] == union
          and lin["j5_amendment_manifest_entries"] == len(union),
          f"and BOTH manifests are verified at RUNTIME: {len(union)} distinct artefacts")
    check(set(lin["j5_amendment_manifest_sha256"]) >= set(a2_entries),
          "including every artefact the r2 amendment pins")
    check(lin["amendments_verified"] == ["j5_authorisation", "j5_r2_rerun"]
          and lin["j5_amendment_r2_sha256"] == J5.PIN_AMENDMENT_R2,
          "both amendments are hash-verified, and neither replaces the other")
    for attr in ("PIN_J3_RECEIPT", "PIN_J3_RUNNER", "PIN_J4_RUNNER", "PIN_J4_AMENDMENT",
                 "PIN_AMENDMENT"):
        saved = getattr(J5, attr)
        setattr(J5, attr, "0" * 64)
        expect(J5.verify_lineage, J5.J5Error, f"a changed {attr} must be refused")
        setattr(J5, attr, saved)
    check(J5.verify_lineage()["j5_amendment_sha256"] == J5.PIN_AMENDMENT,
          "and lineage verification passes again with the true pins")

    # ---------------------------------------------------------------- the contract --------------
    check(J5.ACTOR_WIDTH == 35 and J5.FULL_OBS_WIDTH == 84,
          "one 35D actor inside an 84D asymmetric observation")
    check(J5.EXPECTED_ZERO_COLUMNS == [0, 1]
          and J5.CONTROLLER_COLUMNS == tuple(range(25, 35)),
          "at J5 only the clock is hard-zero; the controller memory is live")
    check(J3.MASKED_COLUMNS == tuple([0, 1] + list(range(25, 35)))
          and J5.EXPECTED_ZERO_COLUMNS != list(J3.MASKED_COLUMNS),
          "which is deliberately DIFFERENT from the J2/J3 base mask")
    import pickle
    with (J5.J4_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    cols = J5.verify_actor_columns(state)
    layers = {k: v for k, v in cols.items() if isinstance(v, dict)}
    check(sorted(layers) == ["pi.0.0.weight", "pi_encoder.0.weight"]
          and J5.EXPECTED_INPUT_LAYERS == ("pi.0.0.weight", "pi_encoder.0.weight"),
          "exactly the two expected input layers are present")
    check(all(tuple(v["shape"]) == (256, 35) for v in layers.values())
          and J5.EXPECTED_INPUT_SHAPE == (256, 35),
          "both are 256x35")
    check(all(v["zero_columns"] == [0, 1] for v in layers.values()),
          "BOTH input-layer aliases carry zero clock columns")
    check(all(len(v["controller_norms"]) == 10 and min(v["controller_norms"].values()) > 0.0
              for v in layers.values()),
          "and all ten controller columns are non-zero in both aliases")
    check(cols["aliases_bit_identical"] is True, "and the two aliases are bit-identical")
    # an EXTRA 35D input layer, a wrong shape, or diverging aliases are all refused
    extra = {k: v.copy() for k, v in state.items()}
    extra["pi.9.9.weight"] = np.zeros((256, 35), dtype=np.float32)
    e = expect(lambda: J5.verify_actor_columns(extra), J5.J5Error,
               "a third 35D input layer must be refused")
    check("expected exactly" in str(e), "because the actor has exactly two")
    missing = {k: v for k, v in state.items() if k != "pi_encoder.0.weight"}
    expect(lambda: J5.verify_actor_columns(missing), J5.J5Error,
           "a missing alias must be refused too")
    reshaped = {k: v.copy() for k, v in state.items()}
    reshaped["pi.0.0.weight"] = np.zeros((128, 35), dtype=np.float32)
    reshaped["pi.0.0.weight"][:, 2:25] = 0.5
    reshaped["pi.0.0.weight"][:, 25:35] = 0.5
    e = expect(lambda: J5.verify_actor_columns(reshaped), J5.J5Error,
               "a 128x35 input layer must be refused")
    check("expected (256, 35)" in str(e), "naming the expected shape")
    diverged = {k: v.copy() for k, v in state.items()}
    diverged["pi_encoder.0.weight"] = diverged["pi_encoder.0.weight"].copy()
    diverged["pi_encoder.0.weight"][0, 5] += np.float32(0.001)
    e = expect(lambda: J5.verify_actor_columns(diverged), J5.J5Error,
               "diverging aliases must be refused")
    check("not bit-identical" in str(e), "because they are one layer, mirrored")
    zeroed = {k: v.copy() for k, v in state.items()}
    zeroed["pi.0.0.weight"][:, 25:35] = 0.0
    zeroed["pi_encoder.0.weight"][:, 25:35] = 0.0
    e = expect(lambda: J5.verify_actor_columns(zeroed), J5.J5Error,
               "a checkpoint whose controller columns are still zero must be refused")
    check("zero columns" in str(e) or "zero-norm controller" in str(e),
          "because that would be the base actor, not the recovery product")
    live_clock = {k: v.copy() for k, v in state.items()}
    live_clock["pi.0.0.weight"][:, 0] = 0.1
    live_clock["pi_encoder.0.weight"][:, 0] = 0.1
    expect(lambda: J5.verify_actor_columns(live_clock), J5.J5Error,
           "a checkpoint whose clock columns are live must be refused")
    check(J5.actor_feature_manifest() == NAMES and len(NAMES) == 35,
          "the actor feature manifest is the pinned 35-name one")

    # ---------------------------------------------------------------- gates, verbatim -----------
    check(J5.J5_COMMON_GATE == J3.J3_COMMON_GATE == J1.J1_GATE,
          "the COMMON gate is inherited verbatim from J3, which took it from J1")
    check(J5.J5_KINEMATIC_GATE == J3.J3_KINEMATIC_GATE,
          "and so is the kinematic quality")
    check(J5.TELEMETRY_INTEGRITY_INVARIANT == J3.TELEMETRY_INTEGRITY_INVARIANT
          and J5.DIAGNOSTIC_NOT_BINDING == J3.DIAGNOSTIC_NOT_BINDING == ("action_clipped_steps",),
          "as are the telemetry invariant and the clipping diagnostic")
    check(J5.J5_COMMON_GATE["max_penetration_m_max"] == 0.020
          and J5.HARD_GUARD_M == 0.028 and J5.SOFT_DIAGNOSTIC_M == 0.020,
          "0.020 m stays the gate and 0.028 m the hard guard")
    good = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
            "valid_hs_count": 3, "valid_to_count": 3, "phase_timeout_stance": 0,
            "phase_timeout_swing": 0, "morphology_causal_contract_failure": 0,
            "hs_cancelled_count": 0, "resync_count": 0, "max_penetration_m": 0.0199,
            "action_clipped_steps": 0}
    knee = np.linspace(-0.75, -0.05, 500)
    ankle = np.linspace(-0.20, 0.15, 500)
    check(J5.evaluate_gate(good, knee, ankle) == J3.evaluate_gate(good, knee, ankle),
          "evaluate_gate is J3's own, so it cannot drift")
    check(J5.evaluate_gate(good, knee, ankle)["pass"] is True
          and J5.evaluate_gate({**good, "max_penetration_m": 0.0201}, knee, ankle)["pass"]
          is False,
          "and it still binds at 0.020 m")
    check(J5.telemetry_integrity(good)["pass"] is True
          and J5.overall_verdict(J5.evaluate_gate(good, knee, ankle),
                                 J5.telemetry_integrity(good)) == "PASS",
          "telemetry integrity is evaluated separately")

    # ------------------------------------------- derived historical references ------------------
    refs = J5.historical_references(lin)
    check(refs["j3"]["max_penetration_m"] == 0.02704966381076714
          and refs["j3"]["steps_above_soft"] == 106 and refs["j3"]["rows"] == 500,
          f"the J3 reference is DERIVED from its pinned trace: {refs['j3']['max_penetration_m']}, "
          f"{refs['j3']['steps_above_soft']} steps above 20 mm")
    check(refs["j1"]["max_penetration_m"] == 0.02294380435912411
          and refs["j1"]["steps_above_soft"] == 97 and refs["j1"]["rows"] == 500,
          "and so is the J1 teacher reference")
    check(refs["j3"]["sha256"] == J5.PIN_J3_TRACE and refs["j1"]["sha256"] == J5.PIN_J1_TRACE,
          "each carries the digest of the trace it was derived from")
    check(refs["cross_checked_against"] == ["the pinned receipt", "the preregistered expectation"],
          "cross-checked against both the receipt and the preregistration")
    check("the values USED are the derived ones" in refs["no_silent_fallback"],
          "and the hard-coded numbers are only an expectation to agree with")
    # a disagreement with the preregistered expectation must abort
    for attr, bad in (("J3_STEPS_ABOVE_SOFT", 105), ("J1_STEPS_ABOVE_SOFT", 96),
                      ("J3_MAX_PENETRATION_M", 0.03), ("J1_MAX_PENETRATION_M", 0.03)):
        saved = getattr(J5, attr)
        setattr(J5, attr, bad)
        expect(lambda: J5.historical_references(lin), J5.J5Error,
               f"a derived value disagreeing with {attr} must abort")
        setattr(J5, attr, saved)
    lin_bad = {**lin, "j3_max_penetration_m": 0.9}
    expect(lambda: J5.historical_references(lin_bad), J5.J5Error,
           "a derived value disagreeing with the pinned receipt must abort")

    # the derivation itself, on small temporary traces
    def rows_of(vals, *, step_from=1, terms=True, key=J1.RT_PENETRATION):
        out = []
        for i, v in enumerate(vals, start=step_from):
            row = {"step": i}
            if terms:
                row["reward_terms"] = {} if v is None else {key: v}
            out.append(row)
        return out

    with tempfile.TemporaryDirectory() as tdr:
        base = Path(tdr)
        def write(name, obj):
            q = base / name
            q.write_text(json.dumps(obj), encoding="utf-8")
            return q, J5._sha_file(q)
        q, h = write("ok.json", rows_of([0.001] * 3 + [0.05]))
        got = J5.derive_penetration_reference(q, h, label="t", expected_rows=4)
        check(got["max_penetration_m"] == 0.05 and got["steps_above_soft"] == 1
              and got["steps_above_hard"] == 1,
              "a well-formed trace derives max and the strictly-above-20 mm count")
        q2, h2 = write("edge.json", rows_of([0.020, 0.0200000001]))
        got2 = J5.derive_penetration_reference(q2, h2, label="t", expected_rows=2)
        check(got2["steps_above_soft"] == 1,
              "the count is STRICTLY above 0.020 m: exactly 0.020 does not count")
        for label, name, obj, rows in (
                ("a wrong row count", "count.json", rows_of([0.001] * 3), 4),
                ("a non-list trace", "notlist.json", {"step": 1}, 1),
                ("a non-mapping row", "notmap.json", [1.0], 1),
                ("a non-contiguous trace", "gap.json", rows_of([0.001, 0.002], step_from=2), 2),
                ("a missing reward_terms", "noterms.json", rows_of([0.001], terms=False), 1),
                ("a missing penetration field", "nofield.json", rows_of([None]), 1),
                ("a non-finite penetration", "nan.json",
                 [{"step": 1, "reward_terms": {J1.RT_PENETRATION: float("nan")}}], 1)):
            qb = base / name
            qb.write_text(json.dumps(obj) if name != "nan.json"
                          else '[{"step": 1, "reward_terms": {"grf_penetration_m": NaN}}]',
                          encoding="utf-8")
            expect(lambda a=qb, r=rows: J5.derive_penetration_reference(
                a, J5._sha_file(a), label="t", expected_rows=r), FAILCLOSED,
                   f"{label} must be refused")
        expect(lambda: J5.derive_penetration_reference(base / "absent.json", "0" * 64, label="t"),
               J5.J5Error, "a missing trace must be refused")
        # the cache must NEVER hide a mutation: the digest is part of its key
        q3 = base / "mut.json"
        q3.write_text(json.dumps(rows_of([0.001, 0.002])), encoding="utf-8")
        h3 = J5._sha_file(q3)
        first = J5.derive_penetration_reference(q3, h3, label="t", expected_rows=2)
        check(first["max_penetration_m"] == 0.002, "the first derivation is cached")
        q3.write_text(json.dumps(rows_of([0.001, 0.09])), encoding="utf-8")
        h3b = J5._sha_file(q3)
        check(h3b != h3, "a mutation changes the digest")
        second = J5.derive_penetration_reference(q3, h3b, label="t", expected_rows=2)
        check(second["max_penetration_m"] == 0.09,
              "so the cache MISSES and the mutated content is re-derived, never served stale")
        check(J5.derive_penetration_reference(q3, h3b, label="t", expected_rows=2)
              == second, "while a repeat with the same digest is served consistently")

    # ---------------------------------------------------------------- comparisons ---------------
    pen = np.full(500, 0.0199)
    pen[100] = 0.0225
    cmp_ = J5.compare_with_history({"max_penetration_m": 0.0225}, pen, refs)
    check("EVIDENCE ONLY" in cmp_["status"] and "never a gate" in cmp_["status"],
          "comparisons are declared evidence only")
    check(cmp_["j5"]["max_penetration_m"] == 0.0225
          and cmp_["j5"]["steps_above_soft"] == 1 and cmp_["j5"]["steps_above_hard"] == 0,
          "the J5 side is measured from the per-step series")
    check(cmp_["vs_j3_base"]["j3_max_penetration_m"] == refs["j3"]["max_penetration_m"]
          and cmp_["vs_j3_base"]["j3_steps_above_soft"] == refs["j3"]["steps_above_soft"]
          and cmp_["vs_j3_base"]["delta_steps_above_soft"] == 1 - 106,
          "J5 is compared with the DERIVED J3 reference")
    check(cmp_["vs_j1_teacher"]["j1_max_penetration_m"] == refs["j1"]["max_penetration_m"]
          and cmp_["vs_j1_teacher"]["delta_steps_above_soft"] == 1 - 97,
          "and with the DERIVED J1 teacher reference")
    check(cmp_["references_derived_from_pinned_traces"]["j3"]["sha256"] == J5.PIN_J3_TRACE,
          "the receipt carries the provenance of both references")
    for broken in ({}, {"j3": refs["j3"]}, {"j3": refs["j3"], "j1": {"rows": 1}}):
        expect(lambda b=broken: J5.compare_with_history({"max_penetration_m": 0.0225}, pen, b),
               J5.J5Error, "a missing derived reference must abort, never fall back")
    expect(lambda: J5.compare_with_history({"max_penetration_m": 0.9}, pen, refs), J5.J5Error,
           "a summary disagreeing with the series must fail closed")
    expect(lambda: J5.compare_with_history({"max_penetration_m": 0.0}, np.array([]), refs),
           J5.J5Error, "an empty penetration series must fail closed")

    # ---------------------------------------------------------------- preflight is inert --------
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"env_factory", "exploration_noise", "torch", "ray", "opensim", "rollout_eval",
              "gymnasium", "gym", "target_domain_imitation"}

    def guarded(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(name)
            raise AssertionError(f"preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre = J5.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: the preflight imported no heavy stack ({tripped})")
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight verdict is GO")
    check(pre["inert"] == {"environment_constructed": False, "environment_reset": False,
                           "environment_stepped": False,
                           "note": "the env factory is imported inside run() only"},
          "and nothing was constructed, reset or stepped")
    check(pre["authorisation"]["execution_requires"]
          == f"--authorized-stage {J5.STAGE} --out-dir <fresh leaf>"
          and pre["authorisation"]["expected_leaf"] == "j5_revalidation_v26c_2026-08-26_r2",
          "execution needs the exact token and a fresh named leaf")
    auth = pre["authorisation"]
    check(auth["scope"] == "exactly one authorised r2 re-run"
          and auth["attempts_authorised"] == 1,
          "the scope names exactly one authorised r2 re-run")
    check(set(auth["amendments"]) == {"j5_authorisation", "j5_r2_rerun"}
          and auth["amendments"]["j5_authorisation"]["sha256"] == J5.PIN_AMENDMENT
          and auth["amendments"]["j5_r2_rerun"]["sha256"] == J5.PIN_AMENDMENT_R2,
          "and BOTH amendments are cited with their hashes")
    check(auth["amendments"]["j5_authorisation"]["file"].endswith(
              "v26c_j5_amendment_revalidation.json")
          and auth["amendments"]["j5_r2_rerun"]["file"].endswith(
              "v26c_j5_amendment_r2_rerun.json"),
          "each by its own file")
    check(auth["production_command_forbids"] == ["-W error"],
          "and the forbidden invocation flag is declared")
    check(pre["runtime"]["pinned_config_sha256"]
          == "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"
          and pre["runtime"]["v3_keys"] == 12,
          "the runtime is the pinned a870cc38... with its twelve v3 keys")
    check(pre["outcome_policy"]["deployable"] is False
          and pre["outcome_policy"]["promotion"] == "NONE"
          and pre["outcome_policy"]["next_stage_authorized"] is False,
          "and the outcome policy promotes nothing")

    # ---------------------------------------------------------------- refusals ------------------
    with tempfile.TemporaryDirectory() as td:
        for stage in (None, "", "V26C-J5", "V26C-J3-CLOSED-LOOP", "V26C-J4-RECOVERY"):
            expect(lambda st=stage: J5.run(authorized_stage=st, out_dir=Path(td) / "x",
                                           stack=_stack(FakeEnv(NAMES), FakeModule(tm), tm),
                                           progress=False),
                   J5.J5Error, f"the token {stage!r} must be refused")
        expect(lambda: J5.run(authorized_stage=J5.STAGE, out_dir=None,
                              stack=_stack(FakeEnv(NAMES), FakeModule(tm), tm), progress=False),
               J5.J5Error, "an implicit output directory must be refused")
        check(not (Path(td) / "x").exists(), "and a refused call creates nothing")
        expect(lambda: J5.main(["--authorized-stage", "WRONG", "--out-dir", str(Path(td) / "y")]),
               J5.J5Error, "the CLI refuses a wrong token too")

        # ------------------------------------------------------------ the happy path ------------
        env, mod = FakeEnv(NAMES), FakeModule(tm)
        r = _run(td, env, mod, tm, leaf="ok")
        leaf = Path(td) / "ok"
        check(r["verdict"] == "PASS" and r["gate_pass"] is True,
              f"a conforming fake episode revalidates: {r['gate']['failed']}")
        check(r["verdict_kind"] == "FRESH DETERMINISTIC CLOSED-LOOP REVALIDATION",
              "and the receipt says what kind of verdict it is")
        check(r["summary"]["steps"] == 500
              and r["summary"]["end_reason"] == "episode_time_limit",
              "exactly 500 steps ending on episode_time_limit")
        check(env.seed_used == 123 and 123 in _FakeTorch.seeded,
              "numpy, torch and env.reset are all seeded with 123")
        check(sorted(q.name for q in leaf.iterdir() if q.is_file())
              == ["j5_kinematics.npz", "j5_trace.json", J5.RECEIPT_NAME],
              "the fresh leaf holds trace, kinematics and receipt")
        trace = json.loads((leaf / "j5_trace.json").read_text())
        check(len(trace) == 500, "the trace holds one row per step")
        for field in ("step", "time_before", "time_after", "reward", "terminated", "truncated",
                      "end_reason", "actor_observation_vector_before", "raw_action", "policy_mean",
                      "policy_std_diagnostic", "applied_action_diagnostic",
                      "action_clipped_diagnostic", "action_selection_path", "stepped_with",
                      "reward_terms", "phase_fsm", "prosthetic_state", "info_scalars"):
            check(field in trace[0], f"every row records {field}")
        check(len(trace[0]["actor_observation_vector_before"]) == 35,
              "the recorded actor observation is the 35D vector the policy saw")
        check(trace[0]["phase_fsm"]["fsm_behaviour_version"] == "v3"
              and trace[0]["phase_fsm"]["event_source"] == "binary_active_v26",
              "and carries the v3 runtime identity")
        kin = np.load(leaf / "j5_kinematics.npz", allow_pickle=False)
        check(kin["knee_rad"].shape == (500,) and kin["ankle_rad"].shape == (500,)
              and kin["penetration_m"].shape == (500,),
              "kinematics AND the penetration series are saved")
        check(r["deterministic_semantics"]["max_abs_action_minus_mean"] == 0.0
              and r["summary"]["realized_noise_rms"] == [0.0, 0.0],
              "the action IS the mean and no noise was realised")
        check(r["deployable"] is False and r["promotion"] == "NONE"
              and r["next_stage_authorized"] is False
              and r["automatic_promotion"] == "NONE, under any outcome",
              "a PASS is not deployable, not promoted and authorises no next stage")
        check(r["lineage_preserved"]["j1_verdict"] == "FAIL"
              and r["lineage_preserved"]["j3_verdict"] == "FAIL"
              and r["lineage_preserved"]["j4_verdict"] == "PASS",
              "and the J1/J3 FAILs and J4 PASS are carried into the J5 receipt unchanged")
        check(r["inputs_sha256"]["j4_artefacts"] == J5.PIN_J4
              and r["inputs_sha256"]["j5_amendment"] == J5.PIN_AMENDMENT
              and r["inputs_sha256"]["pinned_runtime_config"] == J5.PIN_RUNTIME_CONFIG_SHA,
              "the receipt pins every input by hash")
        check(set(r["outputs_sha256"]) == {"j5_trace.json", "j5_kinematics.npz"}
              and r["outputs_sha256"]["j5_trace.json"] == J5._sha_file(leaf / "j5_trace.json"),
              "and hashes what it wrote")
        check(r["comparison"]["j5"]["max_penetration_m"] == r["summary"]["max_penetration_m"]
              and "vs_j3_base" in r["comparison"] and "vs_j1_teacher" in r["comparison"],
              "the receipt carries both required comparisons")
        check(r["stack"]["injected"] is True and r["stack"]["operational"] is False,
              "a receipt from an injected test stack says so")
        check(r["j5_gate"]["source"].startswith("inherited VERBATIM"),
              "and states that the gates were inherited verbatim")
        check(env.closed is True, "the environment is closed afterwards")

        # ------------------------------------------------------------ raw action, clipping ------
        env2, mod2 = FakeEnv(NAMES), FakeModule(tm, mean=(1.5, -1.25))
        r2 = _run(td, env2, mod2, tm, leaf="raw")
        received = np.asarray(env2.received)
        check(received.shape == (500, 2) and np.allclose(received[:, 0], 1.5),
              "the environment is stepped with the RAW action, unclipped")
        t2 = json.loads((Path(td) / "raw" / "j5_trace.json").read_text())
        check(t2[0]["applied_action_diagnostic"] == [1.0, -1.0]
              and t2[0]["stepped_with"] == "raw_action",
              "while the clipped action is a diagnostic mirror only")
        check(r2["summary"]["action_clipped_steps"] == 500 and r2["verdict"] == "PASS",
              "clipping alone changes no verdict")
        check(np.array_equal(np.asarray(mod2.seen_obs[0]).reshape(-1),
                             np.asarray(env2.obs_at(1)).reshape(-1)),
              "and the policy sees the FULL 84D observation")

        # ------------------------------------------------------------ contract refusals ---------
        expect(lambda: _run(td, FakeEnv(NAMES, n_actor=33), FakeModule(tm), tm, leaf="c1"),
               FAILCLOSED, "a 33D actor must be refused")
        expect(lambda: _run(td, FakeEnv(NAMES, n_full=80), FakeModule(tm), tm, leaf="c2"),
               FAILCLOSED, "an 80D observation must be refused")
        bad = list(NAMES)
        bad[7] = "renamed_feature"
        expect(lambda: _run(td, FakeEnv(tuple(bad)), FakeModule(tm), tm, leaf="c3"),
               FAILCLOSED, "a runtime manifest differing from the pinned one is refused")
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm, n_full=88), tm, leaf="c4"),
               FAILCLOSED, "a module declaring n_full=88 is refused")
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm, offset=1e-6), tm, leaf="noisy"),
               FAILCLOSED, "an action that differs from the mean is refused as noise")
        expect(lambda: _run(td, FakeEnv(NAMES, nan_reward_at=3), FakeModule(tm), tm, leaf="nan"),
               FAILCLOSED, "a non-finite reward aborts before it is serialised")
        expect(lambda: _run(td, FakeEnv(NAMES, drop_field="grf_penetration_m"), FakeModule(tm),
                            tm, leaf="drop"),
               FAILCLOSED, "a missing reward_terms field is never defaulted")
        expect(lambda: _run(td, FakeEnv(NAMES, end_reason=""), FakeModule(tm), tm, leaf="noend"),
               FAILCLOSED, "an absent end_reason is never assumed to be a success")

        # ------------------------------------------------------------ FAIL and quarantine -------
        rf = _run(td, FakeEnv(NAMES, penetration=0.0201), FakeModule(tm), tm, leaf="fail")
        check(rf["verdict"] == "FAIL" and rf["gate"]["failed"] == ["max_penetration_m"],
              "20.1 mm fails the 0.020 m gate at J5 exactly as at J3")
        check((Path(td) / "fail" / "j5_trace.json").exists()
              and rf["quarantine"]["applies"] is True
              and rf["quarantine"]["artefacts_preserved"] is True
              and "FORBIDDEN" in rf["quarantine"]["retry"],
              "the evidence is preserved and quarantined, and retry stays forbidden")
        check(rf["deployable"] is False and rf["promotion"] == "NONE",
              "and a FAIL promotes nothing either")
        check(rf["comparison"]["vs_j3_base"]["delta_max_penetration_m"]
              == 0.0201 - J5.J3_MAX_PENETRATION_M,
              "the comparison is still reported on a FAIL, as evidence")
        ri = _run(td, FakeEnv(NAMES, counts=lambda s: (0, min(3, s // 150), min(2, s // 200))),
                  FakeModule(tm), tm, leaf="invalid")
        check(ri["verdict"] == "INVALID" and ri["gate_pass"] is True
              and ri["qualification_technically_valid"] is False,
              "contradictory HS/TO evidence yields INVALID, not FAIL")

        # ------------------------------------------------------------ no-clobber and cleanup ----
        expect(lambda: _run(td, FakeEnv(NAMES), FakeModule(tm), tm, leaf="ok"),
               J5.J5Error, "no-clobber: an existing leaf is never overwritten")
        check(json.loads((leaf / J5.RECEIPT_NAME).read_text())["outputs_sha256"]
              == r["outputs_sha256"], "and the existing leaf is left byte-identical")

        def _boom(cfg):
            raise RuntimeError("the environment could not be constructed")

        expect(lambda: J5.run(authorized_stage=J5.STAGE, out_dir=Path(td) / "boom",
                              stack=J3._Stack(name="fake", operational=False, torch_mod=tm,
                                              load_module=lambda p: FakeModule(tm),
                                              make_env=_boom),
                              progress=False),
               RuntimeError, "a construction failure propagates")
        check(not (Path(td) / "boom").exists(),
              "and removes only the empty leaf it had just created")
        for protected in (J5.J4_LEAF, J5.J4_MODULE_DIR, J5.J3_LEAF, J5.J1_LEAF, HERE):
            for inj in (True, False):
                expect(lambda q=protected, i=inj: J5._remove_leaf(q, injected=i), J5.J5Error,
                       f"_remove_leaf(injected={inj}) refuses to touch {protected.name}")

    # ------------------------------------------- the r2 amendment and r1 preservation -----------
    a2 = json.loads(J5.AMENDMENT_R2.read_text())
    check(J5._sha_file(J5.AMENDMENT_R2) == J5.PIN_AMENDMENT_R2,
          "the r2 amendment is pinned by exact hash")
    check(a2["kind"] == "ADDITIVE, IMMUTABLE" and a2["attempt"] == "r2"
          and a2["authorised_by"].startswith("THE USER, explicitly"),
          "it is additive, names the r2 attempt and attributes the decision to the user")
    check(a2["does_not_rewrite"]["j5_amendment_sha256"] == J5.PIN_AMENDMENT
          and J5._sha_file(J5.AMENDMENT) == J5.PIN_AMENDMENT,
          "and it does not rewrite the J5 authorisation amendment")
    cause = a2["r1_attempt"]["exact_cause"]
    check("RuntimeWarning" in cause["exception"] and "-W error" in cause["summary"]
          and "SLSQP" in cause["raised_in"] and cause["not_a_gate_failure"] is True
          and cause["not_a_defect_of_the_actor"] is True,
          "the exact cause is recorded: an SLSQP clipping RuntimeWarning promoted by -W error")
    check(cause["classification"].startswith("INVOCATION-PROTOCOL ERROR")
          and cause["not_a_defect_of_j5_tooling"] is False,
          "classified as an invocation-protocol error, NOT as 'no tooling defect'")
    gap = cause["j5_tooling_gap"]
    check("did NOT detect or block" in gap["gap"] and gap["status"] == "CLOSED in the r2 tooling"
          and "BLOCKED with exit 1" in gap["closed_by"],
          "the r1 preflight gap is recorded and declared closed by the r2 tooling")
    check(cause["warning_nature"]
          == "a standard SLSQP clipping warning, non-fatal under Python's default warning policy",
          "the warning is described precisely")
    check("disable ALL warnings globally" in cause["rollout_eval_precedent"]
          and "does NOT document this exact SciPy _check_clip_x warning"
          in cause["rollout_eval_precedent"],
          "and the rollout_eval precedent is stated accurately, not overclaimed")
    check("nothing about whether the rollout would have been physically valid"
          in cause["what_this_does_not_establish"],
          "r1 establishes nothing about physical validity")
    check("physically valid" not in cause["summary"]
          and "physically valid" not in a2["r2_execution"]["warning_policy"]["why"],
          "and no text claims the aborted rollout was physically valid")
    check(a2["r1_attempt"]["status"] == "TECHNICALLY INVALID - NO VERDICT"
          and a2["r1_attempt"]["env_step_returns"] == 0
          and a2["r1_attempt"]["completed_trace_rows"] == 0
          and "steps_completed" not in a2["r1_attempt"]
          and a2["r1_attempt"]["exit_code"] == 1,
          "r1 is recorded as technically invalid: 0 env.step returns, 0 completed trace rows")
    check("zero env.step() calls returned and zero trace rows were completed"
          in a2["r1_attempt"]["why_invalid"],
          "with that exact, measurable formulation")
    check(a2["authorises"]["attempts_authorised"] == 1
          and a2["authorises"]["scope"] == "exactly one authorised r2 re-run"
          and "a third attempt" in a2["authorises"]["forbidden"][0],
          "exactly one authorised r2 re-run; a third attempt is forbidden")
    check("-W error" not in a2["r2_execution"]["warning_policy"]["command"]
          and a2["r2_execution"]["warning_policy"]["rule"]
          == "the production command MUST NOT use -W error",
          "and the r2 production command drops -W error")
    check(a2["gates"]["common"] == dict(J3.J3_COMMON_GATE)
          and a2["gates"]["kinematic_quality"]["knee_amplitude_min_rad"] == 0.60,
          "while the gates stay exactly the J3 ones")
    check(a2["superseded_artefacts_sha256"]["v26c_j5_revalidation.py"]
          == "3bf6ed0d8a5b7d2b10a151e05784c1b59917efbf9d609a52589a231403d245d7"
          and a2["superseded_artefacts_sha256"]["test_v26c_j5_revalidation.py"]
          == "bfbdcd79dda400c3b46dc34c660ee92aaba61f091d40aa346e596c8d7ec726f0",
          "the r1-era runner and test are recorded for provenance")
    check("NOT verified at runtime" in a2["superseded_artefacts_sha256"]["note"],
          "and are explicitly excluded from the runtime manifest, because r2 edits them")
    r1_pins = {k: v for k, v in a2["pinned_artefacts_sha256"].items()
               if k.startswith(f"j5_runs/{J5.R1_LEAF_NAME}/")}
    check(len(r1_pins) == 19 and all(k.startswith(f"j5_runs/{J5.R1_LEAF_NAME}/sim_outputs/")
                                     for k in r1_pins),
          f"all 19 r1 files are pinned, and they are all sim_outputs: {len(r1_pins)}")

    r1 = J5.verify_r1_preserved()
    check(r1["files"] == 19 and r1["preserved_byte_for_byte"] is True
          and r1["status"] == "TECHNICALLY INVALID - NO VERDICT" and r1["steps_completed"] == 0,
          "the preserved r1 leaf verifies byte-for-byte and carries no verdict")
    check(r1["sha256"] == {k[len(f"j5_runs/{J5.R1_LEAF_NAME}/"):]: v for k, v in r1_pins.items()},
          "against exactly the hashes the amendment pins")
    check(not (J5.R1_LEAF / J5.RECEIPT_NAME).exists()
          and not (J5.R1_LEAF / "j5_trace.json").exists()
          and not (J5.R1_LEAF / "j5_kinematics.npz").exists(),
          "with no receipt, trace or kinematics: nothing was added to it")
    # a mutated or extended r1 must abort - proven on a COPY, never on the real leaf
    with tempfile.TemporaryDirectory() as tdr1:
        copy = Path(tdr1) / "r1"
        shutil.copytree(J5.R1_LEAF, copy)
        saved_r1 = J5.R1_LEAF
        try:
            J5.R1_LEAF = copy
            check(J5.verify_r1_preserved()["files"] == 19, "a faithful copy verifies clean")
            (copy / "j5_trace.json").write_text("[]", encoding="utf-8")
            e = expect(J5.verify_r1_preserved, J5.J5Error,
                       "a file ADDED to r1 must abort")
            check("expected exactly" in str(e) or "j5_trace.json" in str(e),
                  "because r1 is preserved byte-for-byte")
            (copy / "j5_trace.json").unlink()
            target = copy / "sim_outputs" / "j1_teacher_episode_states.sto"
            target.write_text("tampered\n", encoding="utf-8")
            e = expect(J5.verify_r1_preserved, J5.J5Error, "a MUTATED r1 artefact must abort")
            check("changed" in str(e), "and the failure says it changed")
        finally:
            J5.R1_LEAF = saved_r1
    check(J5.verify_r1_preserved()["files"] == 19,
          "MEASURED: the real r1 leaf is untouched after those refusals")

    # ------------------------------------------- the -W error policy ----------------------------
    wp = J5.verify_warning_policy()
    check(wp["forbidden"] == ["-W error"] and "SLSQP" in wp["why"],
          "the production command is declared to forbid -W error, with the reason")
    check(wp["error_filter_active"] is False,
          "this interpreter is not running under -W error")
    check('if not injected and verify_warning_policy()["error_filter_active"]:' in src
          and "Run the authorised command without -W error." in src,
          "and the production path REFUSES to start when it is")
    run_src = src[src.index("def run("):]
    check(run_src.index("error_filter_active") < run_src.index("J3.production_stack()")
          and run_src.index("error_filter_active") < run_src.index("out.mkdir("),
          "refused BEFORE the production stack is built and before the leaf is created")
    # the blocker itself, proven by flipping the interpreter state and restoring it
    saved_opts = list(sys.warnoptions)
    try:
        sys.warnoptions.append("error")
        check(J5.verify_warning_policy()["error_filter_active"] is True,
              "the guard detects an -W error filter when one is present")
        blocked = J5.preflight()
        check(blocked["verdict"] == "BLOCKED" and len(blocked["blockers"]) == 1
              and blocked["blockers"][0].startswith("INVOCATION-PROTOCOL:"),
              f"the preflight is BLOCKED under -W error: {blocked['blockers']}")
        check("-W error" in blocked["blockers"][0]
              and "SLSQP clipping RuntimeWarning" in blocked["blockers"][0],
              "naming the flag and the warning it promotes")
        check(blocked["inert"]["environment_constructed"] is False
              and blocked["inert"]["environment_stepped"] is False,
              "and still constructing, resetting and stepping nothing")
        expect(lambda: J5.run(authorized_stage=J5.STAGE,
                              out_dir=J5.OUT_ROOT / J5.EXPECTED_LEAF, stack=None,
                              progress=False),
               J5.J5Error, "a production run refuses outright under -W error")
        check(not (J5.OUT_ROOT / J5.EXPECTED_LEAF).exists(),
              "and creates no r2 leaf doing so")
    finally:
        sys.warnoptions[:] = saved_opts
    check(J5.verify_warning_policy()["error_filter_active"] is False
          and J5.preflight()["verdict"] == "GO",
          "after restoring the interpreter state the normal preflight is GO again")
    # the same, through the real CLI
    import subprocess
    runner = str(HERE / "v26c_j5_revalidation.py")
    proc = subprocess.run([sys.executable, "-W", "error", runner, "--preflight"],
                          capture_output=True, text=True, cwd=str(HERE))
    check(proc.returncode == 1, f"the CLI exits 1 under -W error (got {proc.returncode})")
    payload = json.loads(proc.stdout)
    check(payload["verdict"] == "BLOCKED"
          and payload["blockers"][0].startswith("INVOCATION-PROTOCOL:"),
          "with a BLOCKED verdict and the invocation-protocol blocker")
    check(payload["inert"]["environment_constructed"] is False,
          "and nothing constructed")
    ok_proc = subprocess.run([sys.executable, runner, "--preflight"],
                             capture_output=True, text=True, cwd=str(HERE))
    check(ok_proc.returncode == 0
          and json.loads(ok_proc.stdout)["verdict"] == "GO",
          "while the authorised command, without -W error, is GO and exits 0")

    # ------------------------------------------- output isolation -------------------------------
    with tempfile.TemporaryDirectory() as tdp:
        safe = Path(tdp) / "leaf"
        iso = J5.validate_out_dir(safe, injected=True)
        check(iso["injected_stack"] is True and iso["production_leaf_enforced"] is False
              and Path(iso["path"]) == safe.resolve(),
              "an injected stack may write to a temp leaf")
        check(set(iso["protected_trees"]) >= {"REPO", "HERE", "J5_OUT_ROOT", "J1_RUN_ROOT",
                                              "J1_LEAF", "J2_RUN_ROOT", "J3_RUN_ROOT", "J3_LEAF",
                                              "J4_RUN_ROOT", "J4_LEAF", "J4_MODULE"},
              "and the protected trees are enumerated")
        check(iso["disjoint_from_all_protected_trees"] is True,
              "an injected leaf must be DISJOINT from every protected tree")
        # malicious or careless paths, refused BEFORE anything is created
        malicious = {
            "the J4 leaf itself": J5.J4_LEAF,
            "inside the J4 leaf": J5.J4_LEAF / "evil",
            "inside the J4 module": J5.J4_MODULE_DIR / "evil",
            "the J4 run root": HERE / "j4_runs",
            "inside the J4 run root": HERE / "j4_runs" / "evil",
            "the J3 leaf": J5.J3_LEAF,
            "inside the J3 leaf": J5.J3_LEAF / "evil",
            "the J3 run root": J3.OUT_ROOT,
            "the J1 leaf": J5.J1_LEAF,
            "inside the J1 leaf": J5.J1_LEAF / "sim_outputs" / "evil",
            "the J2 run root": HERE / "j2_runs",
            "inside the J2 run root": HERE / "j2_runs" / "evil",
            "the J5 run root itself": J5.OUT_ROOT,
            "the stage directory": HERE,
            "the repository root": J5.REPO,
            "an ancestor of the repo": J5.REPO.parent,
            # an innocent-looking NEW directory is still inside a protected tree: an injected
            # stack could otherwise create it and then have _remove_leaf delete it
            "a new directory inside the stage dir": HERE / "innocent_new_dir",
            "a new directory inside the repo": J5.REPO / "innocent_new_dir",
            "a nested new directory in the repo": J5.REPO / "a" / "b" / "innocent_new_dir",
            "a new directory inside the J5 run root": J5.OUT_ROOT / "innocent_new_dir",
            # even the production leaf is refused when the stack is injected
            "the production leaf under an injected stack": J5.OUT_ROOT / J5.EXPECTED_LEAF,
        }
        for label, path in malicious.items():
            before = path.exists()
            e = expect(lambda q=path: J5.validate_out_dir(q, injected=True), J5.J5Error,
                       f"{label} must be refused as an output leaf")
            check("refusing" in str(e), f"and the refusal says so for {label}")
            check(path.exists() == before,
                  f"and {label} is unchanged - nothing was created")
        # the same refusal reaches run(), before the preflight and before any mkdir
        for label, path in (("inside the J4 leaf", J5.J4_LEAF / "evil"),
                            ("the J5 run root", J5.OUT_ROOT),
                            ("the stage directory", HERE)):
            expect(lambda q=path: J5.run(authorized_stage=J5.STAGE, out_dir=q,
                                         stack=_stack(FakeEnv(NAMES), FakeModule(tm), tm),
                                         progress=False),
                   J5.J5Error, f"run() refuses {label}")
        check(not (J5.J4_LEAF / "evil").exists()
              and sorted(str(q.relative_to(J5.J4_LEAF)) for q in J5.J4_LEAF.rglob("*")
                         if q.is_file()) == sorted(J5.PIN_J4),
              "MEASURED: no 'evil' directory was created inside the J4 leaf")
        # production accepts ONE path and nothing else
        prod = (J5.OUT_ROOT / J5.EXPECTED_LEAF)
        ok = J5.validate_out_dir(prod, injected=False)
        check(ok["production_leaf_enforced"] is True and Path(ok["path"]) == prod.resolve(),
              "production accepts exactly j5_runs/j5_revalidation_v26c_2026-08-26_r2")
        for label, path in (("another basename", J5.OUT_ROOT / "j5_other"),
                            ("another parent", HERE / "j5_elsewhere" / J5.EXPECTED_LEAF),
                            ("a temp path", safe)):
            expect(lambda q=path: J5.validate_out_dir(q, injected=False), J5.J5Error,
                   f"production refuses {label}")
        check(not (J5.OUT_ROOT / "j5_other").exists()
              and not (HERE / "j5_elsewhere").exists(),
              "and none of those were created")
        # _remove_leaf enforces the SAME isolation, in the caller's own context
        for label, path in (("the J4 leaf", J5.J4_LEAF), ("the J3 leaf", J5.J3_LEAF),
                            ("the stage directory", HERE), ("the J5 root", J5.OUT_ROOT),
                            ("a new directory inside the stage dir", HERE / "innocent_new_dir"),
                            ("a new directory inside the repo", J5.REPO / "innocent_new_dir")):
            expect(lambda q=path: J5._remove_leaf(q, injected=True), J5.J5Error,
                   f"_remove_leaf(injected=True) refuses {label}")
        check(not (HERE / "innocent_new_dir").exists()
              and not (J5.REPO / "innocent_new_dir").exists()
              and not (J5.OUT_ROOT / "innocent_new_dir").exists(),
              "MEASURED: no innocent_new_dir was ever created anywhere")
        # an injected cleanup removes a DISJOINT temp leaf, and only that
        victim = Path(tdp) / "disposable"
        victim.mkdir()
        (victim / "f.txt").write_text("x", encoding="utf-8")
        J5._remove_leaf(victim, injected=True)
        check(not victim.exists(), "a disjoint temp leaf is removed by an injected cleanup")
        # a production cleanup may remove EXACTLY the production leaf. Proven under a
        # monkeypatched OUT_ROOT so no real path is ever created or deleted; restored in finally.
        real_root = J5.OUT_ROOT

        def snapshot_root(root: Path) -> dict[str, Any]:
            """Existence plus the byte identity of everything under the real J5 run root.

            Taken BEFORE the monkeypatch so the restoration can be checked against reality,
            whatever that reality is: empty before the authorised rollout, populated after it.
            """
            if not root.exists():
                return {"exists": False, "files": {}}
            return {"exists": True,
                    "files": {q.relative_to(root).as_posix(): J5._sha_file(q)
                              for q in sorted(root.rglob("*")) if q.is_file()}}

        root_before = snapshot_root(real_root)
        try:
            J5.OUT_ROOT = Path(tdp) / "fake_j5_runs"
            fake_leaf = J5.OUT_ROOT / J5.EXPECTED_LEAF
            fake_leaf.mkdir(parents=True)
            (fake_leaf / "f.txt").write_text("x", encoding="utf-8")
            ok_prod = J5.validate_out_dir(fake_leaf, injected=False)
            check(ok_prod["production_leaf_enforced"] is True
                  and "ONLY path allowed inside the repository" in ok_prod[
                      "intentional_exception"],
                  "the production exception is verified explicitly, by equality")
            expect(lambda: J5.validate_out_dir(fake_leaf, injected=True), J5.J5Error,
                   "and the very same path is refused when the stack is injected")
            expect(lambda: J5._remove_leaf(J5.OUT_ROOT / "other", injected=False), J5.J5Error,
                   "a production cleanup refuses any other basename")
            J5._remove_leaf(fake_leaf, injected=False)
            check(not fake_leaf.exists(),
                  "while the exact production leaf is removable by a production cleanup")
        finally:
            J5.OUT_ROOT = real_root
        root_after = snapshot_root(real_root)
        check(J5.OUT_ROOT == real_root, "MEASURED: OUT_ROOT is restored to the real run root")
        check(root_after == root_before,
              "MEASURED: the real j5_runs is byte-identical to its pre-monkeypatch snapshot - "
              "this suite must keep passing after the authorised rollout, so the invariant is "
              "'unchanged', never 'absent'")
        check({rel: J5._sha_file(J5.J4_LEAF / rel) for rel in J5.PIN_J4} == J5.PIN_J4,
              "MEASURED: the J4 leaf survived every malicious-path attempt")

    # ------------------------------------------- output evidence and parity ---------------------
    with tempfile.TemporaryDirectory() as tds:
        env3, mod3 = FakeEnv(NAMES), FakeModule(tm)
        leaf3 = Path(tds) / "ev"

        class _SimEnv(FakeEnv):
            """A fake env that flushes a sim_output on close, as the real simulator does."""

            def close(self):
                sim = leaf3 / "sim_outputs"
                sim.mkdir(parents=True, exist_ok=True)
                (sim / "episode_states.sto").write_text("fake sto\n", encoding="utf-8")
                (sim / "nested").mkdir(exist_ok=True)
                (sim / "nested" / "detail.csv").write_text("a,b\n1,2\n", encoding="utf-8")
                super().close()

        env3 = _SimEnv(NAMES)
        r3 = J5.run(authorized_stage=J5.STAGE, out_dir=leaf3,
                    stack=_stack(env3, mod3, tm), progress=False)
        outs = r3["outputs_sha256"]
        check("sim_outputs/episode_states.sto" in outs
              and "sim_outputs/nested/detail.csv" in outs,
              f"sim_outputs are hashed RECURSIVELY with POSIX relative paths: {sorted(outs)}")
        check(all("\\" not in k for k in outs), "with no backslashes in any key")
        check(set(outs) == {"j5_trace.json", "j5_kinematics.npz",
                            "sim_outputs/episode_states.sto", "sim_outputs/nested/detail.csv"},
              "trace, npz and every sim output are included")
        check(J5.RECEIPT_NAME not in outs
              and r3["outputs_sha256_excludes"]["file"] == J5.RECEIPT_NAME,
              "only the receipt itself is excluded, and the exclusion is declared")
        for rel, digest in outs.items():
            check(J5._sha_file(leaf3 / rel) == digest, f"the recorded hash of {rel} is correct")
        check(r3["sim_outputs"]["count"] == 2 and r3["sim_outputs"]["required"] is False,
              "the sim_outputs inventory is recorded; with a fake stack it is not required")
        check(env3.closed is True, "the environment was closed before the inventory was taken")
        # an injected run without sim_outputs stays legal
        r4 = _run(tds, FakeEnv(NAMES), FakeModule(tm), tm, leaf="nosim")
        check(r4["sim_outputs"]["count"] == 0 and r4["verdict"] == "PASS",
              "a fake run with no sim_outputs is accepted")
        check(r4["output_isolation"]["injected_stack"] is True,
              "and the receipt records that the stack was injected")

    # production preconditions: non-operational stack or missing parity helper are refused
    src_run = src[src.index("def run("):]
    check('if not getattr(stack, "operational", False):' in src_run
          and 'if getattr(stack, "reference_action", None) is None:' in src_run,
          "production refuses a non-operational stack or a missing rollout_eval reference")
    check("parity was proven on" in src_run and "production requires every step" in src_run,
          "and requires parity on EVERY step")
    check("if not injected and not sim_outputs:" in src_run,
          "and requires non-empty sim_outputs")
    check("_remove_leaf(out, injected=injected)" in src_run,
          "the failure cleanup passes the REAL injected flag, so it can only remove what it was "
          "entitled to create")
    check(src_run.index("env.close()") < src_run.index('receipt["outputs_sha256"]'),
          "the environment is closed BEFORE the output inventory is taken")

    # ---------------------------------------------------------------- static guarantees ---------
    heavy = {"torch", "ray", "env_factory", "rollout_eval", "opensim", "gymnasium"}
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & heavy), f"J5 imports no heavy module at file scope ({sorted(top & heavy)})")
    for name in ("prescribed_teacher_action", "train_ppo", "PPOConfig", "dagger", "DAgger",
                 "subprocess", "adapt_actor", "forward_exploration",
                 "get_exploration_action_dist_cls", "optimizer", "Adam", "backward",
                 "aggregate_dagger_traces", "build_markov_recovery_dataset"):
        check(name not in ids, f"the module never references {name}")
    check(not any(i.lower().startswith("v26b") for i in ids),
          "and no v26b identifier appears")
    j3_src = (HERE / "v26c_j3_closed_loop.py").read_text()
    check("J3.deterministic_action(" in src and "J3.production_stack()" in src
          and "J3._verify_runtime_contract(" in src,
          "J5 DELEGATES action selection, the stack and the runtime contract to J3, so they "
          "cannot drift")
    check("forward_inference" in j3_src and "to_deterministic" in j3_src,
          "and that delegated implementation is the deterministic inference path")
    check("J3.evaluate_gate(" in src and "J3.telemetry_integrity(" in src
          and "J3.overall_verdict(" in src,
          "the gate, the integrity invariant and the verdict rule are J3's own functions")
    check(J5.OUT_ROOT.name == "j5_runs" and J5.EXPECTED_LEAF
          == "j5_revalidation_v26c_2026-08-26_r2" and J5.STAGE == "V26C-J5-REVALIDATION",
          "the stage token and output root are unchanged; the accepted leaf is now r2")
    check(J5.R1_LEAF_NAME == "j5_revalidation_v26c_2026-08-26_r1"
          and J5.R1_LEAF == J5.OUT_ROOT / J5.R1_LEAF_NAME
          and J5.EXPECTED_LEAF != J5.R1_LEAF_NAME,
          "and r1 is a distinct, preserved leaf that production may never reuse")
    # the J0-J4 artefacts this stage must never modify
    for rel, pin in (("v26c_j3_closed_loop.py", J5.PIN_J3_RUNNER),
                     ("v26c_j4_recovery.py", J5.PIN_J4_RUNNER),
                     ("v26c_j3_amendment_recovery_only.json", J5.PIN_J4_AMENDMENT)):
        check(J5._sha_file(HERE / rel) == pin,
              f"MEASURED: {rel} is byte-identical after the whole suite")
    check({rel: J5._sha_file(J5.J4_LEAF / rel) for rel in J5.PIN_J4} == J5.PIN_J4,
          "MEASURED: the J4 leaf is byte-identical after the whole suite")
    existing = sorted(q.name for q in J5.OUT_ROOT.iterdir()) if J5.OUT_ROOT.is_dir() else []
    check(set(existing) <= {J5.R1_LEAF_NAME, J5.EXPECTED_LEAF},
          f"the only J5 leaves that may exist are the preserved r1 and the authorised r2: "
          f"{existing}")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS,
                      "j5_leaves_present": existing}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
