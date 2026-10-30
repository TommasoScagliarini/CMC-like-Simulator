"""Pins, gates and inert preflight for the V26C J3 tooling. The runner itself is exercised by
test_v26c_j3_runner.py against a fake stack. Nothing here constructs, resets or steps anything.
"""
from __future__ import annotations
import ast, builtins, io, json, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j3_closed_loop as J3  # noqa: E402
import v26c_j1_collect as J1  # noqa: E402

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


def main() -> int:
    src = (HERE / "v26c_j3_closed_loop.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    j2_before = {n: J3._sha_file(J3.J2_LEAF / n) for n in J3.PIN_J2}

    # ---------------------------------------------------------------- the J2 actor, pinned ------
    j2 = J3.verify_j2_actor()
    check(len(J3.PIN_J2) == 6 and j2["artefacts_sha256"] == J3.PIN_J2,
          "the six J2 artefacts are pinned and every one matches its hash")
    check(J3.PIN_J2["rl_module/module_state.pkl"]
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          and J3.PIN_J2["v26c_j2_fit_receipt.json"]
          == "dd997dc9d465184de6e25a3488c9752799fb60feca304ac876f35dfbdcd9871e",
          "including the fitted weights and the J2 receipt")
    check(j2["deployable"] is False and j2["promotion"] == "NONE",
          "the J2 actor is neither deployable nor promoted")
    check(j2["best_epoch"] == 378 and abs(j2["adapted_rmse"] - 0.013858708553016186) < 1e-15,
          "and J3 reads its selection and metrics from that receipt")
    for name in J3.PIN_J2:
        saved = J3.PIN_J2[name]
        J3.PIN_J2[name] = "0" * 64
        e = expect(J3.verify_j2_actor, J3.J3Error, f"a mutated {name} must be refused")
        check(name in str(e), f"and the failure names {name}")
        J3.PIN_J2[name] = saved
    saved_leaf = J3.J2_LEAF
    J3.J2_LEAF = HERE / "j2_runs" / "does_not_exist"
    expect(J3.verify_j2_actor, J3.J3Error, "a missing J2 leaf must be refused")
    J3.J2_LEAF = saved_leaf
    # an EXTRA file must be refused too - proven on a COPY, never on the immutable leaf
    with tempfile.TemporaryDirectory() as td:
        copy = Path(td) / "j2_copy"
        shutil.copytree(J3.J2_LEAF, copy)
        J3.J2_LEAF = copy
        check(J3.verify_j2_actor()["artefacts_sha256"] == J3.PIN_J2,
              "a faithful COPY of the leaf verifies clean")
        (copy / "_stray.txt").write_text("x", encoding="utf-8")
        e = expect(J3.verify_j2_actor, J3.J3Error, "an EXTRA file in the leaf must be refused")
        check("_stray.txt" in str(e), "and the failure names it")
        (copy / "history.json").write_text("{}", encoding="utf-8")
        (copy / "_stray.txt").unlink()
        e = expect(J3.verify_j2_actor, J3.J3Error, "a MUTATED artefact must be refused")
        check("history.json" in str(e), "and the failure names that too")
    J3.J2_LEAF = saved_leaf
    check(all(J3._sha_file(J3.J2_LEAF / n) == h for n, h in j2_before.items()),
          "the production J2 leaf was never written to: only a temporary copy was mutated")

    # ---------------------------------------------------------------- J1 lineage preserved -----
    lin = J3.verify_j1_lineage()
    check(lin["j1_verdict"] == "FAIL" and lin["j1_failed"] == ["max_penetration_m"],
          "the J1 receipt still records its ORIGINAL FAIL on max_penetration_m")
    check(lin["j1_max_penetration_m"] == 0.02294380435912411, "with the same measured penetration")
    check(lin["j1_receipt_sha256"] == J3.PIN_J1_RECEIPT
          and lin["j1_amendment_sha256"] == J3.PIN_J1_AMENDMENT,
          "and both the receipt and the amendment are byte-identical")
    check("neither re-scores it nor retroactively promotes it" in lin["statement"],
          "J3 declares that it does not retroactively promote the J1 result")
    for attr in ("PIN_J1_RECEIPT", "PIN_J1_AMENDMENT"):
        saved = getattr(J3, attr)
        setattr(J3, attr, "0" * 64)
        expect(J3.verify_j1_lineage, J3.J3Error, f"a changed {attr} must be refused")
        setattr(J3, attr, saved)

    # ---------------------------------------------------------------- the runtime --------------
    check(J3.PIN_RUNTIME_CONFIG_SHA
          == "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db",
          "the runtime config is the pinned a870cc38...")
    env_cfg = J3.build_env_config()
    for k, v in J1.V3_LITERAL_CROSSCHECK.items():
        check(env_cfg[k] == v, f"the env carries the v3 field {k} = {v!r}")
    check(env_cfg["event_contract_id"] != env_cfg["binary_phase_event_contract_id"],
          "and the two contract ids stay distinct")
    check(env_cfg["grf_penetration_penalty_threshold_m"] == 0.020
          and env_cfg["grf_penetration_termination_m"] == 0.028,
          "with the current 0.020 / 0.028 thresholds")
    check(len(env_cfg["reward"]) == 125, "and the whole 125-key reward block")
    check("build_full_env_config" in src and "J1.verify_env_config" in src,
          "J3 reuses J1's hardened builder rather than assembling its own")
    check(J3.FULL_OBS_WIDTH == 84 and J3.ACTOR_WIDTH == 35,
          "the contract is a 35D actor inside an 84D asymmetric observation")

    # ---------------------------------------------------------------- the binding gate ---------
    check(J3.J3_COMMON_GATE == J1.J1_GATE,
          "the COMMON part of the J3 gate is inherited verbatim from J1")
    check(J3.J3_COMMON_GATE == {"steps_required": 500, "end_reason": "episode_time_limit",
                                "valid_cycles_min": 2, "phase_timeout_stance_max": 0,
                                "phase_timeout_swing_max": 0,
                                "morphology_causal_contract_failure_max": 0,
                                "hs_cancelled_count_max": 0, "resync_count_max": 1,
                                "max_penetration_m_max": 0.020},
          "500 steps, episode_time_limit, >=2 cycles, zero timeouts, zero morphology failure, "
          "zero cancellations, resync <=1, penetration <= 0.020 m")
    kg = J3.J3_KINEMATIC_GATE
    check(kg["ankle_min_rad_max"] == -0.03 and kg["ankle_amplitude_min_rad"] == 0.30
          and kg["knee_amplitude_min_rad"] == 0.60 and kg["knee_strictly_flexed"] is True
          and kg["knee_bounds_rad"] == (-1.5, 0.0) and kg["ankle_bounds_rad"] == (-0.7, 0.7),
          "ankle min <= -0.03, ankle ROM >= 0.30, knee ROM >= 0.60, knee strictly flexed, "
          "and the preregistered physical bounds")
    sys.path.insert(0, str(HERE.parent / "v26b_bridge_2026-08-24"))
    import v26b_b1_base_fit as B1  # noqa: E402
    srcg = B1.declared_closed_loop_gates()["kinematic_quality"]
    check(kg["ankle_min_rad_max"] == srcg["ankle_min_rad"]
          and kg["ankle_amplitude_min_rad"] == srcg["ankle_amplitude_min_rad"]
          and kg["knee_amplitude_min_rad"] == srcg["knee_amplitude_min_rad"]
          and kg["knee_strictly_flexed"] == srcg["knee_strictly_flexed"],
          "and they equal v26b_b1_base_fit.declared_closed_loop_gates verbatim, none relaxed")
    bounds_src = (HERE.parent / "v26b_bridge_2026-08-24" / "v26b_r0a_rollout.py").read_text()
    bounds = next(ast.literal_eval(ast.get_source_segment(bounds_src, n.value))
                  for n in ast.walk(ast.parse(bounds_src))
                  if isinstance(n, ast.Assign) and len(n.targets) == 1
                  and isinstance(n.targets[0], ast.Tuple)
                  and [getattr(t, "id", None) for t in n.targets[0].elts]
                  == ["KNEE_BOUNDS", "ANKLE_BOUNDS"])
    check(bounds == (kg["knee_bounds_rad"], kg["ankle_bounds_rad"]),
          f"and the physical bounds equal the preregistered {bounds}")
    check(J3.DIAGNOSTIC_NOT_BINDING == ("action_clipped_steps",),
          "action clipping is declared diagnostic")
    check("action_clipped_steps" not in json.dumps(J3.J3_COMMON_GATE)
          and "action_clipped_steps" not in json.dumps(
              {k: list(v) if isinstance(v, tuple) else v for k, v in kg.items()}),
          "and appears in NO binding gate")

    # ---------------------------------------------------------------- gate evaluation ----------
    good = {"steps": 500, "end_reason": "episode_time_limit", "valid_cycle_count": 2,
            "valid_hs_count": 3, "valid_to_count": 3,
            "phase_timeout_stance": 0, "phase_timeout_swing": 0,
            "morphology_causal_contract_failure": 0, "hs_cancelled_count": 0,
            "resync_count": 0, "max_penetration_m": 0.0199, "action_clipped_steps": 0}
    knee = np.linspace(-0.75, -0.05, 500)
    ankle = np.linspace(-0.20, 0.15, 500)
    g = J3.evaluate_gate(good, knee, ankle)
    check(g["pass"] is True and g["failed"] == [], "a conforming episode passes the full J3 gate")
    check(g["kinematic_quality_applies"] is True, "and the kinematic part applies at J3")
    check(J3.evaluate_gate({**good, "action_clipped_steps": 500}, knee, ankle)["pass"] is True,
          "clipping alone never fails J3")
    for label, k2, a2 in (
            ("ankle_min", knee, np.linspace(-0.01, 0.34, 500)),
            ("ankle_amplitude", knee, np.linspace(-0.20, -0.05, 500)),
            ("knee_amplitude", np.linspace(-0.35, -0.05, 500), ankle),
            ("knee_strictly_flexed", np.linspace(-0.70, 0.05, 500), ankle),
            ("knee_out_of_bounds", np.linspace(-1.90, -0.05, 500), ankle),
            ("ankle_out_of_bounds", knee, np.linspace(-0.90, -0.05, 500))):
        check(J3.evaluate_gate(good, k2, a2)["pass"] is False,
              f"a violating {label} fails the J3 gate")
    for field, bad in (("steps", 499), ("end_reason", "grf_penetration"),
                       ("valid_cycle_count", 1), ("phase_timeout_stance", 1),
                       ("phase_timeout_swing", 1), ("morphology_causal_contract_failure", 1),
                       ("hs_cancelled_count", 1), ("resync_count", 2),
                       ("max_penetration_m", 0.0201)):
        check(J3.evaluate_gate({**good, field: bad}, knee, ankle)["pass"] is False,
              f"a violating {field} fails the J3 gate")
    check(J3.evaluate_gate({**good, "max_penetration_m": 0.020}, knee, ankle)["pass"] is True,
          "the penetration bound is inclusive at exactly 0.020 m")
    expect(lambda: J3.kinematic_quality(np.array([np.nan, -0.5]), ankle[:2]), J3.J3Error,
           "non-finite kinematics must fail closed")
    expect(lambda: J3.kinematic_quality(np.array([]), np.array([])), J3.J3Error,
           "empty kinematics must fail closed")

    # ------------------------------------------------- telemetry integrity, kept SEPARATE ------
    ti = J3.telemetry_integrity(good)
    check(ti["pass"] is True and ti["kind"] == "TELEMETRY INTEGRITY INVARIANT",
          "HS/TO coherence is a telemetry integrity invariant")
    check(ti["is_behavioural_gate"] is False and ti["is_promotion_threshold"] is False,
          "declared as neither a behavioural gate nor a promotion threshold")
    check("hs_to_coherence" not in g["checks"]
          and not any("telemetry" in k for k in g["checks"]),
          "and it appears in NO behavioural gate check")
    check(g["telemetry_integrity_evaluated_separately"] is True,
          "the behavioural gate records that the invariant is evaluated elsewhere")
    for label, s2 in (("hs below cycles", {**good, "valid_hs_count": 1}),
                      ("to below cycles", {**good, "valid_to_count": 1}),
                      ("hs/to differ by two", {**good, "valid_hs_count": 5})):
        bad_ti = J3.telemetry_integrity(s2)
        check(bad_ti["pass"] is False and bad_ti["qualification_technically_valid"] is False,
              f"{label} makes the qualification technically invalid")
        check(J3.evaluate_gate(s2, knee, ankle)["pass"] is True,
              f"{label} does NOT by itself fail the behavioural gate")
        check(J3.overall_verdict(J3.evaluate_gate(s2, knee, ankle), bad_ti) == "INVALID",
              f"{label} yields the verdict INVALID, not FAIL")
    check(J3.overall_verdict(g, ti) == "PASS"
          and J3.overall_verdict(J3.evaluate_gate({**good, "steps": 499}, knee, ankle), ti)
          == "FAIL",
          "an ordinary gate violation is a FAIL; contradictory evidence is INVALID")

    # ---------------------------------------------------------------- preflight is inert -------
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
        pre = J3.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: the preflight imported no heavy stack ({tripped})")
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight verdict is GO")
    check(pre["inert"] == {"environment_constructed": False, "environment_reset": False,
                           "environment_stepped": False,
                           "note": "the env factory is imported inside run() only"},
          "and it declares that no environment was constructed, reset or stepped")
    layers = pre["actor_contract"]["input_layers_verified"]
    check(pre["actor_contract"]["zero_columns_observed"] == list(J3.MASKED_COLUMNS)
          and sorted(layers) == ["pi.0.0.weight", "pi_encoder.0.weight"]
          and all(v == list(J3.MASKED_COLUMNS) for v in layers.values()),
          "EVERY 35D input layer carries the mask on columns 0,1 and 25..34")
    check(pre["determinism"]["seed"] == 123
          and pre["determinism"]["exploration_noise"] == "NONE",
          "the rollout is deterministic at seed 123 with no exploration noise")
    check(pre["execution_requires"] == f"--authorized-stage {J3.STAGE} --out-dir <fresh leaf>",
          "and execution needs both the exact token and an explicit fresh leaf")
    check("confers NO promotion by itself" in pre["promotion_policy"],
          "a J3 PASS confers no promotion by itself")
    for forbidden in ("retry", "Markov", "DAgger", "sigma", "LOTO", "LOCO", "B1R1", "B1R2",
                      "promotion", "ex-novo"):
        check(forbidden in pre["forbidden_here"], f"{forbidden} is declared forbidden")

    # ---------------------------------------------------------------- static guarantees --------
    # NO V26B OPERATIONAL INPUT. Provenance may be CITED in comments and strings - that is how the
    # gate's source is documented - but no v26b module may be imported or referenced as code.
    check(not any(i.lower().startswith("v26b") for i in ids),
          f"the runner never references a v26b identifier "
          f"({sorted(i for i in ids if i.lower().startswith('v26b'))})")
    imported = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check(not any(m.lower().startswith("v26b") for m in imported),
          f"and imports no v26b module anywhere ({sorted(imported)})")
    check(any("v26b_b1_base_fit.declared_closed_loop_gates" in s for s in
              (n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
               and isinstance(n.value, str))),
          "while still CITING the preregistered source of the kinematic thresholds")
    heavy = {"torch", "ray", "env_factory", "rollout_eval", "opensim", "gymnasium"}
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check(not (top & heavy), f"and imports no heavy module at file scope ({sorted(top & heavy)})")
    fn = next(f for f in ast.walk(tree)
              if isinstance(f, ast.FunctionDef) and f.name == "production_stack")
    inside = {n.names[0].name.split(".")[0] for n in ast.walk(fn) if isinstance(n, ast.Import)}
    inside |= {(n.module or "").split(".")[0]
               for n in ast.walk(fn) if isinstance(n, ast.ImportFrom)}
    check({"torch", "rollout_eval", "env_factory", "ray"} <= inside,
          "the heavy stack is imported inside production_stack() only")
    for name in ("prescribed_teacher_action", "train_ppo", "PPOConfig", "dagger", "DAgger",
                 "subprocess", "fit_masked", "adapt_actor", "forward_exploration",
                 "get_exploration_action_dist_cls"):
        check(name not in ids, f"the module never references {name}")
    check("forward_inference" in ids and "to_deterministic" in ids,
          "it uses the inference path only")
    # ONE authorised J3 rollout has been executed. Its leaf is pinned; no second run may appear.
    AUTHORISED_J3_LEAF = "j3_base_v26c_2026-08-26_r1"
    J3_PINS = {
        "v26c_j3_closed_loop_receipt.json":
            "34d856b0b4acabd000a1e6257767c6049a1f9f2147eb99d6f3801ca7559ff422",
        "j3_trace.json":
            "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae",
        "j3_kinematics.npz":
            "402040d7cc794a26213ece8abd04a6e945fa46050f20d4ecc402bf78df0b97fc",
    }
    leaves = sorted(p.name for p in J3.OUT_ROOT.iterdir()) if J3.OUT_ROOT.is_dir() else []
    check(leaves in ([], [AUTHORISED_J3_LEAF]),
          f"MEASURED: at most the ONE authorised J3 leaf exists; found {leaves}")
    if leaves:
        leaf3 = J3.OUT_ROOT / AUTHORISED_J3_LEAF
        top = sorted(p.name for p in leaf3.iterdir() if p.is_file())
        check(top == sorted(J3_PINS),
              f"holding exactly the three pinned artefacts; found {top}")
        for n, pin in J3_PINS.items():
            check(J3._sha_file(leaf3 / n) == pin, f"and {n} is byte-identical to its pin")
        r3 = json.loads((leaf3 / "v26c_j3_closed_loop_receipt.json").read_text())
        check(r3["verdict"] == "FAIL" and r3["gate"]["failed"] == ["max_penetration_m"]
              and r3["deployable"] is False and r3["promotion"] == "NONE"
              and r3["next_stage_authorized"] is False
              and r3["quarantine"]["applies"] is True,
              "the executed rollout stays FAIL, undeployable, unpromoted and quarantined")
    check({n: J3._sha_file(J3.J2_LEAF / n) for n in J3.PIN_J2} == j2_before == J3.PIN_J2,
          "MEASURED: the immutable J2 leaf is byte-identical before and after this suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
