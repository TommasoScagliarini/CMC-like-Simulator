"""Synthetic tests for V26B stages A0 and A1. No fit, no training, no rollout, no collection."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_a0_transplant25 as A0  # noqa: E402
import v26b_a1_ik_imitation as A1  # noqa: E402
import v26b_student as VS  # noqa: E402
import f2r_refit as RF  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def synth39(hidden=8, seed=3):
    rng = np.random.default_rng(seed)
    f32 = np.float32
    W1 = rng.normal(size=(hidden, 39)).astype(f32); b1 = rng.normal(size=(hidden,)).astype(f32)
    W2 = rng.normal(size=(hidden, hidden)).astype(f32); b2 = rng.normal(size=(hidden,)).astype(f32)
    W3 = rng.normal(size=(4, hidden)).astype(f32); b3 = rng.normal(size=(4,)).astype(f32)
    p = lambda a: (np.ascontiguousarray(a.copy()), np.ascontiguousarray(a.copy()))  # noqa: E731
    e0w, p0w = p(W1); e0b, p0b = p(b1); e2w, p2w = p(W2); e2b, p2b = p(b2)
    return {"pi_encoder.0.weight": e0w, "pi_encoder.0.bias": e0b,
            "pi_encoder.2.weight": e2w, "pi_encoder.2.bias": e2b,
            "pi.0.0.weight": p0w, "pi.0.0.bias": p0b,
            "pi.0.2.weight": p2w, "pi.0.2.bias": p2b,
            "pi.1.weight": np.ascontiguousarray(W3), "pi.1.bias": np.ascontiguousarray(b3)}


def main() -> int:
    # --- contract 25D ---------------------------------------------------------------------------
    ct = A0.contract_25d()
    check(len(ct["names25"]) == 25 and len(ct["removed_39d_indices"]) == 14,
          "the 25D contract keeps 25 features and removes 14")
    check(len(ct["target_features_removed"]) == 4 and len(ct["controller_features_removed"]) == 10,
          "the removal set is 4 healthy targets + 10 controller-state features")
    check(ct["removed_39d_indices"] == [2, 3, 4, 5, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38],
          "the removed 39D indices are the two documented contiguous blocks")
    check(ct["kept_39d_indices"] == sorted(ct["kept_39d_indices"]), "kept indices increase")
    check(ct["contract_yaml_sha256"] == A0.PIN_CONTRACT_25D
          and ct["parent_config_sha256"] == A0.PIN_PARENT_CONFIG, "contract and parent config pinned")
    check(not any(A0._is_controller(n) for n in ct["names25"]),
          "no controller-state feature survives into the 25D contract")
    check(ct["full_observation_dim"] == 84, "the full observation stays 84")
    yaml = A0.CONTRACT_25D.read_text(encoding="utf-8")
    active = [ln for ln in yaml.splitlines() if not ln.lstrip().startswith("#")]
    flag = [ln for ln in active if "include_controller_state_observation" in ln]
    check(len(flag) == 1 and flag[0].strip() == "include_controller_state_observation: false",
          "exactly one ACTIVE setting line, and it is false (the header comment documents the flip)")
    check("include_controller_state_observation: true -> false" in yaml,
          "the header documents which single flag was flipped")
    for k in ("grf_penetration_termination_m: 0.028", "grf_penetration_penalty_threshold_m: 0.02",
              "phase_stance_hard_timeout_s: 2.2", "pros_ankle_target_slew_rate_limit_rad_s: 2.0"):
        check(k in yaml, f"the 25D contract preserves the v3 value {k!r} (physics/guards untouched)")
    for attr, bad in (("PIN_CONTRACT_25D", "0" * 64), ("PIN_PARENT_CONFIG", "0" * 64)):
        old = getattr(A0, attr)
        try:
            setattr(A0, attr, bad); expect(A0.contract_25d, A0.A0Error, f"tampered {attr} refused")
        finally: setattr(A0, attr, old)

    # --- forbidden sources ------------------------------------------------------------------------
    for bad in ("/x/candidates/S0D_35D_DISTILLED/rl_module", "/x/candidates/S1A_IK_AB06_35D/rl",
                "/x/candidates/REV4E_R2REPLAY_35D/rl", "/x/runs/training/target_domain_dagger_2026-07-11_r2",
                "/x/student/V2_DAGGER_R1/rl_module"):
        e = expect(lambda b=bad: A0.assert_not_forbidden(b, "test"), A0.A0Error, f"{bad} refused")
        check("forbidden source" in str(e), "the guard explains the refusal")
    A0.assert_not_forbidden(A0.V26_MODULE, "parent")
    check(True, "the August V26 parent passes the source guard")
    check(A0.PIN_V26_ACTOR_DIGEST == "5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e",
          "the pinned parent is the August V26 39D actor")

    # --- synthetic transplant --------------------------------------------------------------------
    src = synth39()
    means = [float(x) for x in np.linspace(-0.5, 0.5, 14)]
    ct2 = dict(ct); ct2["removed_names_in_39d_order"] = [ct["names39"][i] for i in ct["removed_39d_indices"]]
    st, rep = A0.transplant_39_to_25(src, ct2, means)
    W1s = np.asarray(src["pi.0.0.weight"]); W1n = np.asarray(st["pi.0.0.weight"])
    check(W1n.shape == (W1s.shape[0], 25), "the transplanted first layer is 25 columns wide")
    for j25, j39 in enumerate(ct["kept_39d_indices"]):
        if j25 >= 2:
            check(np.array_equal(W1n[:, j25], W1s[:, j39]), f"kept column {j25} bit-identical")
    check(np.all(W1n[:, 0:2] == 0.0), "clock columns zeroed")
    want = (np.asarray(src["pi.0.0.bias"]).astype(np.float64)
            + W1s[:, ct["removed_39d_indices"]].astype(np.float64) @ np.asarray(means)).astype(np.float32)
    check(np.array_equal(np.asarray(st["pi.0.0.bias"]), want),
          "the bias equals b1 + W1[:, removed] @ mean_removed, exactly")
    check(np.array_equal(np.asarray(st["pi.0.2.weight"]), np.asarray(src["pi.0.2.weight"])),
          "the hidden layer is untouched")
    check(np.array_equal(np.asarray(st["pi.1.weight"])[:2], np.asarray(src["pi.1.weight"])[:2]),
          "the mean head is untouched")
    check(np.all(np.asarray(st["pi.1.weight"])[2:] == 0.0)
          and np.allclose(np.asarray(st["pi.1.bias"])[2:], np.float32(np.log(A0.SIGMA_PLACEHOLDER))),
          "the log-std head is the serialisation placeholder")
    check("NOT a decision" in rep["logstd_placeholder"]["statement"]
          and "MEASURED on A1" in rep["logstd_placeholder"]["statement"],
          "the receipt states sigma is not decided here")
    check(tuple(st.keys()) == RF.EXPECTED_KEY_ORDER, "key order preserved")
    RF.validate_init_state(st, expected_actor_digest=None, width=25)
    check(True, "the transplanted state passes the frozen structural validator at width 25")
    # fail-closed
    bad = {k: v.copy() for k, v in src.items()}; bad["pi.0.0.weight"] = bad["pi.0.0.weight"][:, :38]
    expect(lambda: A0.transplant_39_to_25(bad, ct2, means), A0.A0Error, "wrong source width refused")
    bad2 = {k: v.copy() for k, v in src.items()}; bad2["pi_encoder.0.bias"] = bad2["pi_encoder.0.bias"] + 1
    expect(lambda: A0.transplant_39_to_25(bad2, ct2, means), A0.A0Error, "broken alias refused")
    bad3 = {k: v.astype(np.float64) for k, v in src.items()}
    expect(lambda: A0.transplant_39_to_25(bad3, ct2, means), A0.A0Error, "float64 source refused")
    expect(lambda: A0.transplant_39_to_25(src, ct2, means[:13]), A0.A0Error, "wrong mean count refused")
    expect(lambda: A0.transplant_39_to_25(src, ct2, means[:13] + [float("nan")]), A0.A0Error,
           "non-finite mean refused")

    # --- A1 split: fail-closed, never random -------------------------------------------------------
    sp = A1.blocked_temporal_split(500)
    check(sp["mode"] == "blocked_temporal_with_embargo" and sp["folds"] == 5
          and sp["embargo_steps"] == 10, "the default split is blocked temporal with an embargo")
    for f in sp["per_fold"]:
        lo, hi = f["holdout"]
        check(f["train_rows"] <= 500 - (hi - lo) - 1, "the embargo removes rows around the holdout")
    expect(lambda: A1.blocked_temporal_split(30), A1.A1Error, "too few rows refused")
    expect(lambda: A1.trajectory_split([0] * 100), A1.A1Error, "a single trajectory cannot be split by trajectory")
    ts = A1.trajectory_split([0] * 100 + [1] * 100 + [2] * 50)
    check(ts["mode"] == "by_trajectory" and ts["trajectories"] == 3, "trajectory split when available")
    ch = A1.build_split({"observations": np.zeros((500, 25)),
                         "trajectory_id": np.array([0] * 250 + [1] * 250)})
    check(ch["mode"] == "by_trajectory", "the split prefers trajectories when there are several")
    ch2 = A1.build_split({"observations": np.zeros((500, 25))})
    check(ch2["mode"] == "blocked_temporal_with_embargo", "it falls back to blocked temporal")
    src_a1 = (HERE / "v26b_a1_ik_imitation.py").read_text()
    check("permutation" not in src_a1 and "shuffle" not in src_a1,
          "no random permutation or shuffle anywhere in the split policy")

    # --- A1 dataset contract ------------------------------------------------------------------------
    good = {"observations": np.zeros((40, 25), np.float32), "actions": np.zeros((40, 2), np.float32),
            "actor_feature_names": np.asarray(ct["names25"], dtype=str)}
    A1.assert_dataset_contract_25(good)
    check(True, "a well-formed 25D teacher dataset passes")
    b = dict(good); b["observations"] = np.zeros((40, 35), np.float32)
    expect(lambda: A1.assert_dataset_contract_25(b), A1.A1Error, "a 35D dataset is refused")
    b2 = dict(good); b2["actions"] = np.full((40, 2), 2.0, np.float32)
    expect(lambda: A1.assert_dataset_contract_25(b2), A1.A1Error, "out-of-range actions refused")
    b3 = dict(good); b3["actor_feature_names"] = np.asarray(ct["names35"], dtype=str)
    expect(lambda: A1.assert_dataset_contract_25(b3), A1.A1Error, "wrong feature names refused")
    b4 = dict(good); o = good["observations"].copy(); o[5, 0] = 1.0; b4["observations"] = o
    expect(lambda: A1.assert_dataset_contract_25(b4), A1.A1Error, "a non-constant clock column refused")

    # --- kinematic gates, with the architect's explicit case ------------------------------------------
    check(A1.ANKLE_MIN_RAD == -0.03, "the ankle minimum threshold is -0.03 rad")
    g = A1.kinematic_gates(knee_q=np.linspace(-0.9, -0.2, 100), ankle_q=np.linspace(-0.0099, 0.35, 100))
    check(g["gates"]["ankle_plantarflexion"]["pass"] is False,
          "an ankle minimum of -0.0099 rad does NOT satisfy q_min <= -0.03")
    check(not (-0.0099 <= A1.ANKLE_MIN_RAD), "-0.0099 is greater than -0.03, arithmetically")
    g2 = A1.kinematic_gates(knee_q=np.linspace(-0.9, -0.2, 100), ankle_q=np.linspace(-0.031, 0.35, 100))
    check(g2["gates"]["ankle_plantarflexion"]["pass"] is True, "-0.031 satisfies it")
    g3 = A1.kinematic_gates(knee_q=np.linspace(-0.03, -0.02, 100), ankle_q=np.linspace(-0.2, 0.35, 100))
    check(g3["gates"]["knee_amplitude"]["pass"] is False, "a collapsed knee fails the amplitude gate")
    g4 = A1.kinematic_gates(knee_q=np.linspace(-0.9, -0.2, 100), ankle_q=np.linspace(-0.05, -0.04, 100))
    check(g4["gates"]["ankle_amplitude"]["pass"] is False, "a flat ankle fails the amplitude gate")
    g5 = A1.kinematic_gates(knee_q=np.linspace(-0.9, 0.2, 100), ankle_q=np.linspace(-0.2, 0.35, 100))
    check(g5["gates"]["knee_stays_flexed"]["pass"] is False and g5["gates"]["within_bounds"]["pass"] is False,
          "a knee crossing zero fails flexion and bounds")
    for k in ("ankle_min_rad", "ankle_amplitude_min_rad", "knee_amplitude_min_rad"):
        check(k in A1.GATE_MOTIVATION and len(A1.GATE_MOTIVATION[k]) > 40,
              f"{k} carries a written motivation")
    check("19.3%" in A1.GATE_MOTIVATION["ankle_min_rad"] and "-0.0099" in A1.GATE_MOTIVATION["ankle_min_rad"],
          "the ankle motivation cites the reference fraction and the refused value")
    d = A1.declared_closed_loop_gates()
    check(d["completion"]["steps"] == 500 and d["valid_cycle_count_min"] == 2
          and d["penetration_max_m"] == 0.020 and d["resync_count_max"] == 1,
          "the declared closed-loop gates are 500/500, >=2 cycles, 0.020 m, resync <=1")
    check(set(d["critical_counters_zero"]) == {"phase_timeout_stance", "phase_timeout_swing",
                                               "morphology_causal_contract_failure", "hs_cancelled_count"},
          "the critical v3 counters are enumerated")
    check("No guard is relaxed" in d["motivation"]["penetration"], "no guard is relaxed")

    # --- A1 is tooling only ---------------------------------------------------------------------------
    expect(lambda: A1.run_a1(authorized_stage="V26B-A1-IK-IMITATION"), A1.A1Error,
           "A1 refuses to run even with its own token")
    expect(A1.require_a1a, A1.A1Error, "A1 fails closed while the A1a teacher dataset is absent")
    check("S0D-visited" in A1.preflight()["a1a_prerequisite"]["why"],
          "the preflight states why the existing corpus cannot be used")
    check(A1.SIGMA_VERIFICATION["status"].startswith("OPEN") and "NOT assumed" in A1.SIGMA_VERIFICATION["status"],
          "sigma 0.005 is explicitly not assumed")
    src_a0 = (HERE / "v26b_a0_transplant25.py").read_text()
    for s in (src_a0, src_a1):
        check(not any(t in s for t in ("subprocess", "rollout_eval", "train_ppo", "PPOConfig",
                                       "collect_teacher_dataset(", "fit_july", "adapt_actor")),
              "neither module can fit, train, roll out or collect")
        check("os.system" not in s and "os.sep" not in s and "from pathlib import Path" in s,
              "pathlib only, no shell, no os-specific path handling")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
