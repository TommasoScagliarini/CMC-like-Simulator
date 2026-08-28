"""Tests for the B1R2 diagnostic study. No candidate, no rollout, no collection, no Markov."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b1r2_diagnostics as D  # noqa: E402
import v26b_b_exec as EX  # noqa: E402
import v26b_b1r1_exec as R1  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    # --- row groups -----------------------------------------------------------------------------
    g = D.row_groups()
    check(g["wait"].size == 187 and g["cycle"].size == 910 and g["uncovered"].size == 403,
          "187 WAIT + 910 B1R1-covered + 403 B1R1-uncovered rows partition the 1500")
    check(len(np.unique(np.concatenate([g["wait"], g["cycle"], g["uncovered"]]))) == 1500,
          "the three coverage groups are disjoint and cover everything")
    segs = g["uncovered_segments"]
    check(len(segs) == 5 and sum(s["rows"] for s in segs) == 403,
          "five B1R1-uncovered startup/tail segments, 403 rows")
    check([(s["name"], s["position"], s["rows"]) for s in segs] ==
          [("minus020", "tail", 11), ("nominal", "head", 164), ("nominal", "tail", 32),
           ("plus020", "head", 144), ("plus020", "tail", 52)],
          "the five segments are exactly the observed head and tail runs")

    # --- structural reclassification: the two heads are COMPLETE cycles, not partial ------------
    check([(s["name"], s["structure"]) for s in segs] ==
          [("minus020", "tail"), ("nominal", "complete_cycle"), ("nominal", "tail"),
           ("plus020", "complete_cycle"), ("plus020", "tail")],
          "nominal 0:163 and plus020 0:143 are structurally COMPLETE cycles; the three tails are not")
    for s in segs:
        if s["structure"] == "complete_cycle":
            check(s["phase_sequence"] == ["stance", "swing"] and s["closed_by_heel_strike"]
                  and not s["ends_at_episode_end"],
                  f"{s['name']} {s['position']} is complete because a full stance-swing closes on a "
                  "heel-strike, derived from the FSM columns and not hardcoded")
        else:
            check(s["ends_at_episode_end"] and not s["closed_by_heel_strike"],
                  f"{s['name']} {s['position']} is a tail because the episode ends inside it")
    ss = g["structure_summary"]
    check(ss["complete_cycles_total"] == 8 and ss["tails_total"] == 3,
          "the design is semantically EIGHT complete cycles plus THREE tails")
    check(ss["structurally_complete_rows"] == 1218 and ss["tail_rows"] == 95
          and g["structurally_complete"].size == 1218 and g["tails"].size == 95,
          "1218 structurally complete rows and 95 tail rows")
    check(len(np.unique(np.concatenate([g["structurally_complete"], g["tails"]]))) == 1313
          and 1218 + 95 + 187 == 1500,
          "complete and tails are disjoint and cover the 1313 non-WAIT rows; WAIT is neither")
    check(not (set(g["structurally_complete"].tolist()) & set(g["wait"].tolist()))
          and not (set(g["tails"].tolist()) & set(g["wait"].tolist())),
          "no WAIT row is counted as a complete cycle or as a tail")
    check(set(g["cycle"].tolist()) <= set(g["structurally_complete"].tolist()),
          "every B1R1-covered cycle row is also structurally complete")

    # --- the instrumented fit is a BIT-IDENTICAL twin of fit_masked ---------------------------------
    init, _ = R1.load_parent()
    rng = np.random.default_rng(11)
    n = 96
    obs = B0.apply_input_mask(rng.normal(size=(n, 35)).astype(np.float32))
    act = np.clip(rng.normal(size=(n, 2)) * 0.2, -1, 1).astype(np.float32)
    snaps: dict[int, dict[str, np.ndarray]] = {}
    D.fit_all_instrumented(init, obs, act, epochs=9,
                           on_epoch=lambda e, s: snaps.__setitem__(e, s))
    for k in (1, 4, 9):
        ref, rep = EX.fit_masked(init, obs, act, train_idx=np.arange(n), val_idx=None,
                                 epochs=k, patience=None)
        check(all(np.array_equal(np.asarray(snaps[k][key]), np.asarray(ref[key])) for key in ref),
              f"the instrumented state at epoch {k} is BIT-IDENTICAL to fit_masked run for {k} epochs")
        check(rep["selection_mode"] == "fixed_final_epoch", "the reference runs in fixed mode")
    check(len(snaps) == 9, "the callback fires once per epoch")
    for e, s in snaps.items():
        B0.assert_masked_columns_zero(s, f"epoch {e}")
        B0.assert_clock_columns_zero(s, f"epoch {e}")
    check(True, "every per-epoch snapshot keeps the masked and clock columns exactly zero")
    check(np.array_equal(np.asarray(snaps[9]["pi.1.weight"])[2:],
                         np.asarray(init["pi.1.weight"])[2:]),
          "the log-std head stays byte-identical to B0 throughout")
    src = (HERE / "v26b_b1r2_diagnostics.py").read_text()
    for tok in ("EX.J_SEED", "EX.J_BATCH", "EX.J_LR", "EX.J_CLIP_W", "EX.J_LOGSTD_W", "EX.J_ANCHOR_W"):
        check(tok in src, f"the twin reuses the pinned constant {tok} rather than restating it")
    check("clip_grad" not in src and "train_ppo" not in src, "no gradient clipping, no PPO")

    # --- gates are not relaxed --------------------------------------------------------------------------
    check(D.GATES == {"aggregate_rmse_max": 0.02, "per_joint_rmse_max": 0.03,
                      "per_joint_max_abs_max": 0.15},
          "the reconstruction gates are the unchanged B1R1 values")
    check(D.GATES["aggregate_rmse_max"] == R1.FINAL_RMSE_MAX
          and D.GATES["per_joint_rmse_max"] == R1.FINAL_PER_JOINT_RMSE_MAX
          and D.GATES["per_joint_max_abs_max"] == R1.FINAL_PER_JOINT_MAXABS_MAX,
          "and they are taken from the executor, not restated by hand")

    # --- full coverage --------------------------------------------------------------------------------------
    cov = D.full_coverage()
    check(cov["n_folds"] == 11, "eleven folds")
    check(cov["n_complete_cycle_folds"] == 8 and cov["n_tail_folds"] == 3,
          "described semantically as EIGHT complete cycles plus THREE tails")
    check(sum(1 for f in cov["folds"] if f["origin"] == "b1r1_loco") == 6
          and sum(1 for f in cov["folds"] if f["origin"] == "b1r1_uncovered") == 5,
          "six folds come from B1R1 and five from the rows B1R1 left uncovered")
    check(cov["every_non_wait_row_held_out_exactly_once"] is True, "the coverage claim is asserted")
    tot = sum(f["validation_rows"] for f in cov["folds"])
    check(tot == 1313, "the eleven folds validate all 1313 non-WAIT rows")
    check(all(f["wait_rows_in_train"] > 0 for f in cov["folds"]),
          "WAIT rows remain in training in every fold")
    check(not cov["aggregate"]["any_temporal_leakage"]
          and cov["aggregate"]["min_temporal_distance"] == D.EMBARGO_ROWS + 1,
          "no temporal leakage and the embargo is honoured")
    check(not cov["aggregate"]["any_signature_missing"], "every discrete signature is covered")
    check(all(f["support_dimensions"] == 23 for f in cov["folds"]), "23D support in every fold")
    check(cov["scaler"] == "fit on TRAINING ROWS ONLY", "the scaler is train-only")

    # --- support sparsity: named correctly, never an information floor ---------------------------
    check(not hasattr(D, "ambiguity_analysis"),
          "the misnamed ambiguity_analysis no longer exists")
    sp = D.support_sparsity_diagnostic()
    check(sp["measure"] == "support sparsity and local smoothness diagnostic",
          "the measure is named support sparsity / local smoothness")
    names = [s["group"] for s in sp["strata"]]
    check(names[:6] == ["all", "b1r1_uncovered_startup_tails", "structurally_complete_cycles",
                        "tails", "b1r1_covered_cycles", "wait"],
          "the strata use the corrected names: uncovered startup/tails, complete cycles, tails")
    check("partial_segments" not in names and "partial" not in names,
          "no stratum is called partial any more")
    check(any(g2.startswith("trajectory_") for g2 in names)
          and any(g2.startswith("signature_") for g2 in names),
          "and also stratify by trajectory and by FSM signature")
    check(f"greater than {D.EMBARGO_ROWS}" in sp["neighbour_rule"],
          "the neighbour rule excludes temporally adjacent rows")
    check(sp["interpretation"] is D.NOT_AN_INFORMATION_FLOOR
          and "NOT an information floor" in sp["interpretation"]
          and "never a gate" in sp["interpretation"],
          "the interpretation explicitly denies being an information floor and denies being a gate")
    check(all("label_disagreement_median" not in s for s in sp["strata"]),
          "the old label_disagreement naming is gone")
    check(all(("target_disagreement_median" in s and "nn_distance_median" in s)
              for s in sp["strata"] if s["rows"]),
          "every stratum reports the neighbour DISTANCE beside the target disagreement, so a large "
          "disagreement can never be read without the distance at which it occurs")
    # every mention of the phrase in the module must be a denial, not a claim.
    # whitespace is normalised first, so a mention wrapped across source lines is still caught.
    low = " ".join(src.lower().split())
    pos, occurrences = 0, 0
    while True:
        h = low.find("information floor", pos)
        if h < 0:
            break
        occurrences += 1
        before = low[max(0, h - 40):h]
        check("not" in before or "never" in before,
              f"every mention of the phrase is a denial, but found: ...{before}<<here>>")
        pos = h + 1
    check(occurrences >= 3, "the denial is stated explicitly, not merely implied")
    check("informational floor" not in low and "informational limit" not in low
          and "q3_informational_limit" not in src,
          "no informational-floor or informational-limit wording or receipt key survives")

    # --- real collision audit ---------------------------------------------------------------------
    ca = D.collision_audit()
    check(tuple(ca["thresholds"]) == (1e-8, 1e-6, 1e-4, 1e-3),
          "the four required distance thresholds are audited")
    check(tuple(ca["target_delta_queries"]) == (0.02, 0.05),
          "the two required target-delta queries are audited")
    check(set(ca["by_space"]) == {"raw", "standardised"},
          "collisions are audited in the raw effective 23D and in the standardised space")
    for space, rep in ca["by_space"].items():
        check(rep["all"]["rows"] == 1500 and rep["all"]["pairs"] == 1500 * 1499 // 2,
              f"the {space} audit is exhaustive over every unordered pair, no sampling")
        sigs = [k for k in rep if k.startswith("signature_")]
        check(len(sigs) == 5 and sum(rep[k]["rows"] for k in sigs) == 1500,
              f"the {space} audit is stratified by the five exact discrete signatures")
        for k, v in rep.items():
            if not v.get("pairs"):
                continue
            check(set(v["at_distance"]) == {"1e-08", "1e-06", "0.0001", "0.001"},
                  f"{space}/{k} reports pair counts at each threshold")
            check(all(("pairs_within" in d and "max_target_disagreement" in d)
                      for d in v["at_distance"].values()),
                  f"{space}/{k} reports the maximum target disagreement at each threshold")
            check(set(v["min_distance_among_pairs_with_target_delta"]) == {"gt_0.02", "gt_0.05"},
                  f"{space}/{k} reports the minimum distance for both target-delta queries")
            for thr, d in v["at_distance"].items():
                check(d["pairs_within"] == 0 or d["max_target_disagreement"] is not None,
                      "a non-empty threshold bucket always carries its disagreement")
                check(float(thr) < v["min_distance"] or d["pairs_within"] > 0,
                      "the threshold counts agree with the measured minimum distance")
    check(ca["any_pair_within_any_threshold"] is False,
          "MEASURED: no pair of rows is within even 1e-3 in the effective 23D support, so the "
          "dataset contains no real collision and no non-identifiability is demonstrated")

    # --- residual predictability: out-of-fold only -------------------------------------------------
    rp = D.residual_predictability_cv()
    check(rp["status"] == "OK", "the residual study runs against the B1R1 state")
    check(len(rp["folds"]) == 12, "eleven non-WAIT segment folds plus WAIT")
    check("out_of_fold_r2" in rp and "in_sample" not in json.dumps(rp["out_of_fold_r2"]),
          "only out-of-fold R^2 is reported")
    check(rp["in_sample_cluster_fits"].startswith("DELIBERATELY NOT COMPUTED"),
          "the in-sample peak-cluster fit is excluded by construction, not merely unreported")
    check("no fold mixes contiguous rows of the same held-out segment" in rp["validation"]
          and f"embargo {D.EMBARGO_ROWS}" in rp["validation"],
          "the split never mixes contiguous rows of the same segment")
    check("scaler fit on training rows only" in rp["validation"], "train-only scaling")
    for k, v in rp["out_of_fold_r2"].items():
        if "knee" not in v:
            continue
        for j in ("knee", "ankle"):
            check(v[j]["out_of_fold_r2"] is None or v[j]["out_of_fold_r2"] <= 1.0,
                  f"the out-of-fold R^2 for {k}/{j} is a genuine held-out score")
    check(max(v[j]["out_of_fold_r2"] for v in rp["out_of_fold_r2"].values() if "knee" in v
              for j in ("knee", "ankle")) < 0.5,
          "MEASURED: no group reaches even R^2 0.5 out of fold, so the in-sample 0.90/0.9996 "
          "figures were overfitting and cannot support any conclusion")

    # --- label and action audit ---------------------------------------------------------------------------------
    aud = D.label_and_action_audit(top_n=5)
    check(aud["rederived_labels_bit_identical_to_the_dataset"] is True,
          "every label re-derived from the pinned caches is bit-identical to the dataset")
    a = aud["action_semantics"]
    check(a["all_labels_within_[-1,1]"] and a["decoded_angles_within_absolute_bounds"]
          and a["encode_decode_round_trip_max_abs"] < 1e-6,
          "the action semantics round-trip and stay inside the absolute bounds")
    check(all(t["lookup"] == "exact float equality" and t["monotone_index"]
              for t in aud["labels_rederived_from_the_pinned_caches"].values()),
          "every trajectory resolves to a contiguous, exactly-matched cache index range")
    check(len(aud["worst_prediction_rows"]) == 5
          and all(r["label_matches_cache"] for r in aud["worst_prediction_rows"]),
          "the worst prediction rows carry correct labels")
    check(all(r["b1r1_coverage"] in {"wait", "uncovered", "covered"}
              and r["structure"] in {"wait", "tail", "complete_cycle"}
              for r in aud["worst_prediction_rows"]),
          "each worst row is tagged by B1R1 coverage AND by structure, so a row can be uncovered "
          "without being called partial")

    # --- nothing is materialised or promoted ---------------------------------------------------------------------
    check(not any(t in src for t in ("subprocess", "rollout_eval", "collect_teacher_dataset",
                                     "promote_staging", "PPOConfig")),
          "the study cannot roll out, collect, promote or run PPO")
    check("write_json(OUT_DIR" in src and "rl_module" not in src.replace('"rl_module"', ""),
          "it writes only its own diagnostic receipt, never an actor module")
    check("os.system" not in src and "os.sep" not in src and "from pathlib import Path" in src,
          "pathlib only, no shell, no os-specific path handling")
    check("if False" not in src, "no dead conditional")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
