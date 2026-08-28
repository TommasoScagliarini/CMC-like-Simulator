"""Fail-closed tests for the V26C J8 recovery fit.

The suite proves the preflight is read-only and torch-free, that every input pin and invariant
holds, that the gate logic classifies correctly, and that the whole write path works - all of it
on SYNTHETIC payloads in temporary roots. It runs NO fit on the J7 dataset and never creates
j8_runs under the validation root.
"""
from __future__ import annotations
import ast, builtins, io, json, os, pickle, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j8_recovery_fit as J8  # noqa: E402

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


def snapshot(root: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for p in sorted(root.rglob("*")):
        if "__pycache__" in p.parts or not p.is_file():
            continue
        out[str(p.relative_to(root))] = J8._sha_file(p)
    return out


def synthetic(*, improve: bool = True) -> tuple[dict, dict]:
    """A plausible fitted payload built from the REAL parent, with fabricated predictions.

    Nothing here fits anything: the arrays are fabricated so the gate logic can be exercised
    without touching the optimiser or the J7 dataset.
    """
    parent = J8.load_parent_state()
    final = {k: v.copy() for k, v in parent.items()}
    # a change the fit could plausibly have made, respecting every standing invariant
    rng = np.random.default_rng(7)
    delta = rng.normal(0.0, 1e-3, size=final["pi.0.0.weight"].shape).astype(np.float32)
    final["pi.0.0.weight"] = (final["pi.0.0.weight"] + delta).astype(np.float32)
    final["pi.0.0.weight"][:, list(J8.CLOCK_COLUMNS)] = 0.0
    for alias, direct in J8.ALIAS_PAIRS:
        final[alias] = final[direct].copy()

    labels = np.zeros((J8.TOTAL_ROWS, J8.ACTION_DIM), dtype=np.float32)
    before = np.full((J8.TOTAL_ROWS, J8.ACTION_DIM), 0.20, dtype=np.float32)
    after = np.full((J8.TOTAL_ROWS, J8.ACTION_DIM), 0.10 if improve else 0.30, dtype=np.float32)
    history = [{"epoch": 1.0, "train_loss": 0.5, "validation_mse": 0.5},
               {"epoch": 2.0, "train_loss": 0.3, "validation_mse": 0.25},
               {"epoch": 3.0, "train_loss": 0.29, "validation_mse": 0.26}]
    split = J8.build_split(J8.TOTAL_ROWS)
    fitted = {
        "final": final, "parent": parent,
        "prereg": {"file": "synthetic", "sha256": J8.PIN_PREREG, "manifest_entries": 12},
        "data": {"observations": np.zeros((J8.TOTAL_ROWS, J8.ACTOR_WIDTH), dtype=np.float32),
                 "actions": labels, "names": J8.actor_feature_names(),
                 "sha256": J8.PIN_DATASET, "receipt_sha256": J8.PIN_J7_RECEIPT,
                 "content_hashes": {"observations": J8.PIN_OBS_CONTENT,
                                    "actions": J8.PIN_ACT_CONTENT}},
        "split": split, "history": history, "best_epoch": 2,
        "best_validation_mse": 0.25, "stopped_early": True, "epochs_run": 3,
        "before": before, "after": after, "names": J8.actor_feature_names(),
        "clock": list(J8.CLOCK_COLUMNS),
        "controller": list(J8.controller_indices(J8.actor_feature_names())),
        "scales_verified": dict(J8.MARKOV_CONTROLLER_FEATURE_SCALES),
        "preconditions": J8.parent_preconditions(
            parent, J8.actor_feature_names(),
            np.zeros((16, J8.ACTOR_WIDTH), dtype=np.float32)),
        "numpy_legacy_seed": {"call": "np.random.seed", "value": 123, "count": 1,
                              "is_a_generator": False,
                              "note": "July's legacy global seed, carried for protocol fidelity; "
                                      "it is not a second Generator"},
        # observed, never forced - the synthetic payload mirrors what the fit records
        "torch_backend_observed": {"are_deterministic_algorithms_enabled": False,
                                   "get_num_threads": 8, "forced_by_j8": False,
                                   "binding": False,
                                   "why_not_forced": "the July run recorded neither setting",
                                   "observed_at": "fit time"},
        "seed_order": ["import torch", "torch.manual_seed", "np.random.seed",
                       "deterministic init from parent (no RNG)",
                       "np.default_rng in build_split", "optimizer"],
    }
    return fitted, J8.audit(fitted)


def main() -> int:
    src = (HERE / "v26c_j8_recovery_fit.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    prereg = json.loads(J8.PREREG.read_text())
    before_all = snapshot(HERE)

    # ---------------------------------------------------------------- preregistration -----------
    check(prereg["kind"] == "ADDITIVE, IMMUTABLE, PREREGISTRATION"
          and prereg["stage_authorised"] == "V26C-J8-RECOVERY-FIT" == J8.STAGE,
          "the prereg is additive, immutable and names the J8 stage")
    check(J8._sha_file(J8.PREREG) == J8.PIN_PREREG
          == "8f661320348f6b6f034bbbe7b2c04ca85efc10cac48375349261ce2dbcba737b",
          "and is pinned by exact hash")

    # ------------------------------------------------- what "July-faithful" actually means ------
    check("value for value" not in prereg["what_this_stage_is"]
          and "reproducing the selected July run" not in prereg["what_this_stage_is"],
          "the prereg does NOT claim to reproduce the July run value for value")
    jf = prereg["what_july_faithful_means"]
    check(jf["replicates"] == ["the algorithm", "the physical scaling", "the hyperparameters",
                               "the documented seeding protocol",
                               "the Generator / split / shuffle order"],
          "it replicates the algorithm, the scaling, the hyperparameters, the DOCUMENTED SEEDING "
          "protocol and the Generator/split/shuffle order")
    check("the determinism protocol" not in jf["replicates"],
          "and does NOT claim to replicate 'the determinism protocol'")
    check(jf["makes_no_claim_about"] == ["the deterministic-algorithms backend",
                                         "the thread count"],
          "it makes no claim about the backend or the thread count")
    check("the determinism protocol" not in json.dumps(prereg),
          "the phrase is absent from the ENTIRE preregistration")
    check(jf["does_not_replicate"] == ["July's dataset", "July's parent", "July's actor",
                                       "July's output"],
          "and NOT July's dataset, parent, actor or output")
    check("AUGUST V26 J2 parent" in jf["operational_lineage"]
          and "16713 rows" in jf["operational_lineage"],
          "the operational lineage is the August V26 J2 parent plus the current J7 dataset")
    check(jf["july_multistart_training_block"] == "ABSENT / DEFERRED",
          "and July's multistart training block is absent/deferred")
    flat = " ".join(src.split())          # whitespace-insensitive: the docstring wraps
    check("does NOT replicate July's dataset, parent, actor or output" in flat,
          "the runner's docstring carries the corrected formulation, not the false one")
    check("DOCUMENTED SEEDING PROTOCOL" in flat
          and "Generator / split / shuffle ORDER" in flat,
          "and names the documented seeding protocol and the Generator/split/shuffle order")
    check("makes NO claim about the deterministic-algorithms backend or the thread count" in flat,
          "and disclaims the backend and the thread count explicitly")
    check("the determinism protocol" not in flat,
          "the phrase 'the determinism protocol' is absent from the ENTIRE runner")
    check("reproduces the selected July run value for value" not in src,
          "the false phrase is gone from the runner entirely")
    check(len(prereg["pinned_artefacts_sha256"]) == 11
          and all(J8._sha_file(HERE / r) == h
                  for r, h in prereg["pinned_artefacts_sha256"].items()),
          "all 11 local pins verify on disk")
    check(len(prereg["pinned_repo_artefacts_sha256"]) == 1
          and all(J8._sha_file(J8.REPO / r) == h
                  for r, h in prereg["pinned_repo_artefacts_sha256"].items()),
          "and the one repo-level pin (the July scaling source) verifies too")
    check(prereg["inputs"]["dataset"]["sha256"] == J8.PIN_DATASET
          == "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
          and prereg["inputs"]["parent"]["module_state_sha256"] == J8.PIN_PARENT_STATE
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130",
          "the accepted J7 dataset and the single J2 parent are the pinned inputs")
    hp = prereg["hyperparameters"]
    for key, value in (("seed", 123), ("epochs", 400), ("batch_size", 128),
                       ("learning_rate", 5e-05), ("validation_fraction", 0.2),
                       ("patience", 60), ("clip_weight", 1.0), ("logstd_weight", 0.0),
                       ("anchor_weight", 0.01), ("trainable_first_layer_features", None)):
        check(hp[key] == value == J8.JULY_HP[key],
              f"July hyperparameter {key} = {value!r}, in the prereg and in the runner")
    check(prereg["future_leaf"]["relative_leaf"] == "j8_runs/j8_recovery_fit_v26c_2026-08-26_r1"
          == J8.RELATIVE_LEAF, "the future leaf is the one the architect fixed")
    check(prereg["future_leaf"]["does_not_exist_yet"]
          and not (HERE / "j8_runs").exists(),
          "MEASURED: j8_runs does not exist")
    check(prereg["outcome_policy"] == {"deployable": False, "promotion": "NONE",
                                       "next_stage_authorized": False, "closed_loop": False,
                                       "single_execution": True,
                                       "no_autonomous_retry": "on failure the evidence is "
                                                              "preserved and the run stops"},
          "it confers no deployability, no promotion, no next stage and no closed-loop claim")
    todo = prereg["deferred_todo"]
    for key in ("loto", "loco", "b1r1", "b1r2"):
        check("NOT integrated" in todo[key], f"{key.upper()} is a future TODO, not integrated")
    check(todo["multistart_training_data"].startswith("DEFERRED")
          and "DIFFERENT AND UNRELATED" in todo["multistart_closed_loop_validation"],
          "multistart TRAINING DATA is deferred and explicitly not the closed-loop validation")

    # ---------------------------------------------------------------- the July scaling ----------
    extracted = J8.july_scales_from_source()
    check(extracted == J8.MARKOV_CONTROLLER_FEATURE_SCALES,
          "the AST extraction matches the runner's transcription")
    check(extracted == {"pros_knee_angle_previous_endpoint": 1.0,
                        "pros_knee_angle_served_ref": 1.0,
                        "pros_knee_angle_served_ref_vel": 4.0,
                        "pros_knee_angle_served_ref_accel": 60.0,
                        "pros_knee_angle_sea_u": 1.0,
                        "pros_ankle_angle_previous_endpoint": 1.0,
                        "pros_ankle_angle_served_ref": 1.0,
                        "pros_ankle_angle_served_ref_vel": 3.5,
                        "pros_ankle_angle_served_ref_accel": 55.0,
                        "pros_ankle_angle_sea_u": 1.0},
          "knee 1/1/4/60/1 and ankle 1/1/3.5/55/1, exactly as the architect fixed them")
    saved_pin = J8.PIN_JULY_SCALES_SOURCE
    try:
        J8.PIN_JULY_SCALES_SOURCE = "0" * 64
        e = expect(J8.july_scales_from_source, J8.J8Error, "a changed scaling source")
        check("scaling source changed" in str(e), "a changed July source is caught")
    finally:
        J8.PIN_JULY_SCALES_SOURCE = saved_pin
    # The comparison against the REAL module is deferred to the end of the suite: importing it
    # pulls torch, and the torch-free assertions on the preflight must be made first.
    scales = J8.scale_vector(J8.actor_feature_names())
    check(scales.dtype == np.float32 and scales.shape == (35,)
          and float(scales[27]) == 4.0 and float(scales[32]) == 3.5,
          "the scale vector places the values on the columns their NAMES resolve to")
    check(float(np.sum(scales != 1.0)) == 4.0,
          "only the four non-unit scales differ from 1.0")

    # ---------------------------------------------------------------- dataset and parent --------
    data = J8.load_dataset()
    check(data["sha256"] == J8.PIN_DATASET
          and data["content_hashes"] == {"observations": J8.PIN_OBS_CONTENT,
                                         "actions": J8.PIN_ACT_CONTENT},
          "the materialised dataset reproduces the accepted file and content hashes")
    obs, act = data["observations"], data["actions"]
    check(obs.shape == (16713, 35) and act.shape == (16713, 2)
          and obs.dtype == np.float32 and act.dtype == np.float32,
          "16713 x 35 and 16713 x 2, float32")
    check(all(np.array_equal(obs[i * 500:(i + 1) * 500], obs[:500]) for i in range(32)),
          "the nominal block is 32 bit-identical repeats of 500 states")
    check(J8.NOMINAL_ROWS == 16000 and J8.RECOVERY_ROWS == 713,
          "16000 nominal + 713 recovery")
    check(float(np.max(np.abs(obs[:, [0, 1]]))) == 0.0, "the clock columns are exactly zero")
    parent = J8.load_parent_state()
    check(tuple(sorted(parent)) == tuple(sorted(J8.STATE_KEYS)) and len(parent) == 10,
          "the parent carries exactly the ten expected keys")
    check(all(np.array_equal(parent[a], parent[d]) for a, d in J8.ALIAS_PAIRS),
          "and its four pi_encoder.* are already bit-identical to their pi.0.* counterparts")
    check(bool(np.all(parent["pi.1.weight"][2:] == 0.0))
          and [float(v) for v in parent["pi.1.bias"][2:]]
          == [-5.2983174324035645, -5.2983174324035645],
          "the parent's logstd head is the one the prereg records")
    check(not any("critic" in k or k.startswith("vf") for k in parent),
          "and it carries no critic key")
    check(J8.controller_indices(data["names"]) == tuple(range(25, 35)),
          "the controller columns resolve BY NAME to 25..34")

    # ---------------------------------------------------------------- binding preconditions -----
    pc = J8.parent_preconditions(parent, data["names"], obs)
    czero = pc["controller_columns_bit_zero"]
    check(czero["all_bit_zero"] is True and czero["binding"] is True
          and len(czero["columns"]) == 10,
          "BINDING: all ten parent controller columns 25:35 are bit-zero")
    check(all(v["bit_zero"] is True and v["norm"] == 0.0 and v["max_abs"] == 0.0
              for v in czero["columns"].values()),
          "with per-column norm and max_abs recorded, all exactly zero")
    eq = pc["raw_vs_scaled_equivalence"]
    check(eq["max_abs_diff"] == 0.0 and eq["bit_identical"] is True,
          "BINDING: the parent gives bit-identical output on raw and physically scaled inputs")
    check(eq["tolerance"] == "NONE - exactly 0.0 is required"
          and eq["measured_not_assumed"] is True,
          "exactly 0.0 is REQUIRED, measured rather than assumed, with no tolerance granted")
    check(eq["scaled_columns"] == [27, 28, 32, 33]
          and eq["scaled_columns_inside_bit_zero_block"] is True,
          "the four columns whose scale differs from 1.0 all lie inside the bit-zero block - "
          "which is WHY the equivalence is exact")
    check("absorbing the scale" in eq["justifies"],
          "and the equivalence is recorded as the justification for the inverse-scale absorption")
    # a parent whose controller columns are NOT bit-zero must abort before the optimizer
    dirty = {k: v.copy() for k, v in parent.items()}
    dirty["pi.0.0.weight"][:, 27] = np.float32(0.01)
    e = expect(lambda: J8.parent_preconditions(dirty, data["names"], obs), J8.J8Error,
               "a parent with a non-zero scaled column must be refused")
    check("not bit-zero" in str(e) and "before the optimizer" in str(e),
          "aborting before the optimizer exists, naming the offending column")
    # and with that column non-zero the equivalence genuinely breaks: the precondition is real
    scales_v = J8.scale_vector(data["names"])
    raw_out = J8._numpy_forward(dirty, obs.astype(np.float32))
    scl_out = J8._numpy_forward(dirty, (obs / scales_v).astype(np.float32))
    check(float(np.max(np.abs(raw_out.astype(np.float64) - scl_out.astype(np.float64)))) > 0.0,
          "PROVEN NECESSARY: with a non-zero scaled column the raw/scaled equivalence fails")

    # ---------------------------------------------------------------- the split ----------------
    s1, s2 = J8.build_split(16713), J8.build_split(16713)
    check(np.array_equal(s1["val_idx"], s2["val_idx"])
          and np.array_equal(s1["train_idx"], s2["train_idx"])
          and s1["digest"] == s2["digest"],
          "the split is deterministic: one seeded generator, same indices every time")
    check(s1["n_val"] == 3343 and s1["n_train"] == 13370
          and s1["n_val"] + s1["n_train"] == 16713,
          f"20% validation: {s1['n_val']} + {s1['n_train']} = 16713")
    union = np.concatenate([s1["val_idx"], s1["train_idx"]])
    check(np.array_equal(np.sort(union), np.arange(16713)),
          "the two partitions cover every row exactly once")
    check(s1["row_level"] is True and s1["group_split"] is False,
          "the split is ROW-LEVEL and introduces no group split")

    # ------------------------------------- July's split ORDERING, reconstructed independently ---
    # target_domain_imitation._resolve_adaptation_split, seeded_random_fraction branch:
    #   indices = rng.permutation(sample_count)
    #   validation_count = max(1, int(round(len(indices) * validation_fraction)))
    #   resolved_validation = np.sort(indices[:validation_count])
    #   resolved_training   = np.asarray(indices[validation_count:], dtype=int)
    fresh = np.random.default_rng(123)
    july_perm = fresh.permutation(16713)
    july_nval = max(1, int(round(len(july_perm) * 0.2)))
    july_val = np.sort(july_perm[:july_nval])
    july_train = np.asarray(july_perm[july_nval:], dtype=int)
    check(july_nval == 3343 == s1["n_val"],
          "the validation count matches, max(1, int(round(rows * fraction))) included")
    check(np.array_equal(s1["val_idx"], july_val),
          "the VALIDATION half matches July exactly, and is sorted")
    check(np.array_equal(s1["train_idx"], july_train),
          "the TRAINING half matches July exactly, in the order the permutation gave it")
    check(np.array_equal(s1["train_idx"], july_perm[july_nval:]),
          "which is literally perm[n_val:], untouched")
    check(not np.array_equal(s1["train_idx"], np.sort(s1["train_idx"])),
          "PROVEN: the training half is NOT sorted - not by accident either")
    check(s1["train_is_sorted"] is False and s1["train_order_preserved"] is True
          and s1["validation_sorted"] is True,
          "and the runner reports exactly that")
    check(np.array_equal(s1["val_idx"], np.sort(s1["val_idx"])),
          "while the validation half IS sorted")
    check(np.array_equal(np.sort(s1["train_idx"]), np.sort(july_train)),
          "membership is of course identical either way - which is why membership digests alone "
          "would NOT have caught this")
    # the order is not cosmetic: it changes every epoch shuffle
    r_ordered = np.random.default_rng(123).permutation(s1["train_idx"])
    r_sorted = np.random.default_rng(123).permutation(np.sort(s1["train_idx"]))
    check(not np.array_equal(r_ordered, r_sorted),
          "PROVEN MATERIAL: shuffling the preserved order and the sorted order gives different "
          "epoch batches from the same Generator state")
    check(s1["dtype"] == "int64" and s1["train_idx"].dtype == np.int64,
          "the index dtype is explicitly int64, identical on macOS and Windows")
    check(s1["train_sha256"] != s1["train_membership_sha256"],
          "the order-sensitive hash differs from the membership hash: the receipt records both")
    check(s1["val_sha256"] == s1["val_membership_sha256"],
          "for the sorted validation half the two coincide, as they must")
    check("initial permutation first" in s1["rule"]
          and "validation membership sorted" in s1["rule"]
          and "training order preserved" in s1["rule"],
          "and the rule is stated in exactly those terms")
    so = prereg["determinism"]["split_ordering"]
    check("_resolve_adaptation_split" in so["source"]
          and so["measured"]["train_is_sorted"] is False
          and so["measured"]["validation_is_sorted"] is True
          and so["dtype"].startswith("np.int64"),
          "the prereg declares the ordering rule, its source and the measured values")
    check("NOT cosmetic" in so["training_is_not_sorted"],
          "and says why the training order matters")
    rng_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == "default_rng"]
    check(len(rng_calls) == 1,
          f"AST: exactly ONE default_rng call in the whole runner ({len(rng_calls)})")
    # July's legacy GLOBAL seed: exactly one call, in the fit path, before the Generator
    legacy = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "seed"]
    check(len(legacy) == 1,
          f"AST: exactly ONE legacy np.random.seed call ({len(legacy)})")
    check(isinstance(legacy[0].func.value, ast.Attribute)
          and legacy[0].func.value.attr == "random",
          "and it is np.random.seed, the legacy global seed")
    run_fit_node = next(n for n in ast.walk(tree)
                        if isinstance(n, ast.FunctionDef) and n.name == "run_fit")
    in_run_fit = [n for n in ast.walk(run_fit_node) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute) and n.func.attr == "seed"]
    check(len(in_run_fit) == 1, "it lives in the --fit path, inside run_fit")
    rng_in_split = [n for n in ast.walk(next(n for n in ast.walk(tree)
                                             if isinstance(n, ast.FunctionDef)
                                             and n.name == "build_split"))
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "default_rng"]
    check(len(rng_in_split) == 1, "and the single Generator is created in build_split")
    check(legacy[0].lineno < next(n.lineno for n in ast.walk(run_fit_node)
                                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                                  and n.func.id == "build_split"),
          "the legacy seed is set BEFORE build_split draws from the Generator")
    check(not [n for n in ast.walk(tree) if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute) and n.func.attr == "RandomState"],
          "and there is no RandomState anywhere: the legacy seed is not a second Generator")
    pre_seed = J8.preflight()["determinism"]["numpy_legacy_global_seed"]
    check(pre_seed["call"] == "np.random.seed" and pre_seed["value"] == 123
          and pre_seed["count_in_fit_path"] == 1 and pre_seed["is_a_generator"] is False
          and pre_seed["set_by_this_preflight"] is False,
          "the preflight records the legacy seed unambiguously, and that it does not set it")
    pre_gen = J8.preflight()["determinism"]["generators"]
    check(pre_gen["call"] == "np.random.default_rng" and pre_gen["count"] == 1,
          "and records exactly one Generator, distinct from the legacy seed")
    check("rng.permutation(split[\"train_idx\"])" in src,
          "and the SAME generator produces the epoch shuffles")
    perms = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == "permutation"]
    check(len(perms) == 2,
          f"exactly two permutations: the split, then the epoch shuffle ({len(perms)})")

    # ---------------------------------------------------------------- preflight is read-only ----
    pre = J8.preflight()
    check(pre["verdict"] == "GO" and pre["blockers"] == [] and pre["read_only"] is True,
          "the preflight is GO and declares itself read-only")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed during the preflight")
    check(pre["inert"]["optimizer_constructed"] is False
          and pre["inert"]["fit_executed"] is False
          and pre["inert"]["environment_constructed"] is False
          and pre["inert"]["rollout_executed"] is False
          and pre["inert"]["critic_touched"] is False
          and pre["inert"]["ppo_updates"] == 0,
          "and declares optimizer/fit/env/rollout/critic absent and PPO zero")
    check(pre["inert"]["torch_imported"] is False and "torch" not in sys.modules,
          "MEASURED: torch is not imported by the preflight")
    tp = pre["torch_provenance"]
    check(tp["torch_present_before"] is False and tp["torch_present_after"] is False
          and tp["torch_imported_by_preflight"] is False,
          "torch_provenance: absent before, absent after, so the preflight introduced none")
    check(tp["torch_imported_by_preflight"]
          == bool(tp["torch_present_after"] and not tp["torch_present_before"]),
          "and the flag is exactly (after and not before), as specified")
    check(tp["forced_by_j8"] is False and "OBSERVED at fit time" in tp["backend_values"]
          and "assumes nothing about them" in tp["backend_values"],
          "the preflight declares the backend values will be observed at FIT time, and assumes "
          "nothing about them")
    pjf = pre["july_faithfulness"]
    check(pjf["replicates"][-2:] == ["the documented seeding protocol",
                                     "the Generator / split / shuffle order"]
          and pjf["makes_no_claim_about"] == ["the deterministic-algorithms backend",
                                              "the thread count"],
          "and the preflight carries the same precise formulation")
    check("the determinism protocol" not in json.dumps(pre),
          "the phrase is absent from the ENTIRE preflight report")

    tripped: list[str] = []
    real = {"rename": os.rename, "replace": os.replace, "mkdir": Path.mkdir,
            "write_text": Path.write_text, "write_bytes": Path.write_bytes,
            "open": builtins.open, "rmtree": shutil.rmtree, "copyfile": shutil.copyfile,
            "dump": pickle.dump}

    def boom(tag):
        def f(*a, **k):  # noqa: ANN001
            tripped.append(tag)
            raise AssertionError(f"the preflight attempted {tag}")
        return f

    def guarded_open(file, mode="r", *a, **k):  # noqa: ANN001
        if any(m in str(mode) for m in ("w", "a", "x", "+")):
            tripped.append(f"open({mode})")
            raise AssertionError(f"the preflight opened {file} for writing")
        return real["open"](file, mode, *a, **k)

    os.rename, os.replace = boom("os.rename"), boom("os.replace")
    Path.mkdir, Path.write_text = boom("Path.mkdir"), boom("Path.write_text")
    Path.write_bytes, shutil.rmtree = boom("Path.write_bytes"), boom("shutil.rmtree")
    shutil.copyfile, pickle.dump = boom("shutil.copyfile"), boom("pickle.dump")
    builtins.open = guarded_open
    banned = {"torch", "ray", "env_factory", "opensim", "rollout_eval", "gymnasium"}
    real_import = builtins.__import__

    def guarded_import(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(f"import {name}")
            raise AssertionError(f"the preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded_import
    try:
        pre2 = J8.preflight()
    finally:
        os.rename, os.replace = real["rename"], real["replace"]
        Path.mkdir, Path.write_text = real["mkdir"], real["write_text"]
        Path.write_bytes, shutil.rmtree = real["write_bytes"], real["rmtree"]
        shutil.copyfile, pickle.dump = real["copyfile"], real["dump"]
        builtins.open, builtins.__import__ = real["open"], real_import
    check(tripped == [],
          f"PROVEN: the preflight called no write primitive and imported no heavy stack ({tripped})")
    check(pre2["split"]["digest"] == pre["split"]["digest"],
          "and two preflights draw the identical split")
    e = expect(lambda: J8.main(["--out", "/tmp/anything"]), J8.J8Error,
               "--out without --fit must be refused")
    check("meaningless without --fit" in str(e), "with a reason")

    # ---------------------------------------------------------------- the gate logic ------------
    fitted, report = synthetic(improve=True)
    check(report["verdict"] == "PASS" and report["failed"] == [],
          "a synthetic result that improves the recovery RMSE and keeps every invariant PASSES")
    check(report["outcome"]["recovery_rmse_decreases"] is True
          and report["diagnostics"]["recovery_rmse"]["after"]
          < report["diagnostics"]["recovery_rmse"]["before"],
          "the binding recovery-RMSE check compares after against before, strictly")
    check(report["outcome"]["controller_columns_nonzero"] is True
          and len(report["diagnostics"]["controller_column_norms"]) == 10,
          "all ten controller column norms are reported and strictly positive")
    check(report["diagnostics"]["binding"] is False
          and set(report["diagnostics"]) >= {"nominal_rmse", "nominal_shift_vs_parent",
                                             "clipping_out_of_bounds", "best_validation_mse"},
          "the diagnostics are reported and explicitly non-binding")
    check(set(report["diagnostics"]["clipping_out_of_bounds"]) ==
          {"aggregate", "nominal_block", "recovery_block", "convention"},
          "clipping is reported for the aggregate, the nominal block and the recovery block")
    check(report["selection"]["best_epoch"] == 2
          and report["selection"]["recomputed_best_epoch"] == 2
          and report["selection"]["reconstructible"] is True,
          "the best epoch is recomputed from the history and agrees")

    _, worse = synthetic(improve=False)
    check(worse["verdict"] == "FAIL"
          and worse["outcome"]["recovery_rmse_decreases"] is False
          and "recovery_rmse_decreases" in worse["failed"],
          "a result that does NOT improve the recovery RMSE FAILS the binding gate")
    check(all(worse["integrity"].values()),
          "while every integrity invariant still holds: the two groups are independent")

    def broken(mutate) -> dict:
        f, _ = synthetic()
        mutate(f)
        return J8.audit(f)

    def kill_alias(f):
        f["final"]["pi_encoder.0.weight"] = f["final"]["pi_encoder.0.weight"] + np.float32(1.0)

    def dirty_clock(f):
        f["final"]["pi.0.0.weight"][0, 0] = np.float32(0.5)

    def move_logstd(f):
        f["final"]["pi.1.bias"] = f["final"]["pi.1.bias"].copy()
        f["final"]["pi.1.bias"][2] = np.float32(0.0)

    def zero_controller(f):
        f["final"]["pi.0.0.weight"][:, 27] = 0.0
        for alias, direct in J8.ALIAS_PAIRS:
            f["final"][alias] = f["final"][direct].copy()

    def wrong_best(f):
        f["best_epoch"] = 3

    def nonfinite(f):
        f["final"]["pi.1.bias"] = f["final"]["pi.1.bias"].copy()
        f["final"]["pi.1.bias"][0] = np.float32("nan")

    for mutate, key, group in ((kill_alias, "aliases_bit_identical", "integrity"),
                               (dirty_clock, "clock_bit_zero", "integrity"),
                               (move_logstd, "logstd_bit_identical_to_parent", "integrity"),
                               (nonfinite, "all_parameters_finite", "integrity"),
                               (wrong_best, "best_state_reconstructible_from_history",
                                "integrity"),
                               (zero_controller, "controller_columns_nonzero", "outcome")):
        r = broken(mutate)
        check(r[group][key] is False and key in r["failed"] and r["verdict"] == "FAIL",
              f"the gate catches a violated {key}")

    # ---------------------------------------------------------------- destination guards --------
    leaf = J8.authorized_leaf()
    check(leaf == HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1",
          "the authorised leaf resolves under the validation root")
    e = expect(lambda: J8.fit(str(leaf), None), J8.J8Error, "a missing stage token")
    check("--authorized-stage must be exactly" in str(e), "naming the exact token required")
    for bad in ("V26C-J7-MATERIALIZE", "v26c-j8-recovery-fit", "V26C-J8-RECOVERY-FIT ", ""):
        expect(lambda b=bad: J8.fit(str(leaf), b), J8.J8Error,
               f"the stage token {bad!r} must be refused")
    e = expect(lambda: J8.fit(None, J8.STAGE), J8.J8Error, "--fit without --out")
    check("requires --out" in str(e), "and say so")
    for bad_out in (str(HERE), str(HERE / "j8_runs"), str(HERE / "j8_runs" / "other"),
                    str(HERE / "j7_runs" / "j8_recovery_fit_v26c_2026-08-26_r1")):
        e = expect(lambda b=bad_out: J8.fit(b, J8.STAGE), J8.J8Error,
                   f"--out {bad_out} is not the authorised leaf")
        check("not the authorised leaf" in str(e), f"refused: {Path(bad_out).name}")
    check(snapshot(HERE) == before_all and not (HERE / "j8_runs").exists(),
          "MEASURED: every refusal wrote nothing and j8_runs still does not exist")

    # ---------------------------------------------------------------- the write path ------------
    # Exercised end to end on the SYNTHETIC payload, in a temporary root. No fit is run on the
    # J7 dataset and the real leaf is never created.
    saved_root = J8.OUTPUT_ROOT_OVERRIDE
    tmp = Path(tempfile.mkdtemp(prefix="v26c_j8_commit_test_"))
    try:
        J8.OUTPUT_ROOT_OVERRIDE = tmp
        target = J8.authorized_leaf()
        lock = target.parent / J8.LOCK_NAME
        check(target == tmp / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1",
              "the override redirects the ROOT only; the relative leaf is unchanged")
        # the AUTHORISED fit refuses outright while the override is set
        e = expect(lambda: J8.fit(str(target), J8.STAGE), J8.J8Error,
                   "the authorised fit must refuse while the override is set")
        check("OUTPUT_ROOT_OVERRIDE is set" in str(e)
              and "synthetic, isolated tests" in str(e)
              and "refuses to run" in str(e),
              "a BLOCKER, not a warning: the override is test-only and never authoritative")
        check(not target.exists(), "and it created nothing")

        # an integrity violation must abort before anything is created
        bad_fitted, bad_report = synthetic()
        bad_fitted["final"]["pi.0.0.weight"][0, 1] = np.float32(0.3)
        bad_report = J8.audit(bad_fitted)
        e = expect(lambda: J8.commit(bad_fitted, bad_report, target), J8.J8Error,
                   "an integrity violation must abort the write")
        check("refusing to write anything" in str(e), "explicitly refusing to write")
        check(not target.exists() and not (tmp / "j8_runs").exists(),
              "and nothing at all was created")

        # cleanup on failure, with concurrent content that must survive
        sentinel = tmp / "j8_runs" / "concurrent_sentinel.txt"

        def dump_then_fail(*a, **k):  # noqa: ANN001
            sentinel.write_text("written by another run", encoding="utf-8")
            raise AssertionError("forced failure after the parent, lock and staging exist")

        pickle.dump = dump_then_fail
        try:
            expect(lambda: J8.commit(fitted, report, target), AssertionError, "forced failure")
        finally:
            pickle.dump = real["dump"]
        check(sentinel.is_file(), "PROVEN: the concurrent sentinel survived the cleanup")
        check((tmp / "j8_runs").is_dir(), "and so did the parent: rmdir is non-recursive")
        check(not (target.parent / J8.STAGING_NAME).exists(),
              "only the exact staging directory was removed")
        check(not lock.exists(), "and the run released its own lock")
        check(not target.exists(), "the leaf was never created")

        # a foreign lock is fail-closed and never removed
        held = json.dumps({"stage": J8.STAGE, "pid": 999999})
        lock.write_text(held, encoding="utf-8")
        e = expect(lambda: J8.commit(fitted, report, target), J8.J8Error, "a held lock")
        check("lock already exists" in str(e) and "removes no lock it does not own" in str(e),
              "the refusal names the lock and refuses to remove it")
        check(lock.read_text() == held, "MEASURED: the foreign lock was left untouched")
        check(J8.preflight()["verdict"] == "BLOCKED",
              "and the read-only preflight reports it as a blocker")
        lock.unlink()

        # the real commit
        res = J8.commit(fitted, report, target)
        check(res["verdict"] == "PASS" and res["staging_removed"] is True
              and res["lock_released"] is True, "the commit succeeds and releases its lock")
        check(res["authoritative"] is False,
              "and declares itself NON-authoritative: the root was overridden")
        listing = sorted(p.name for p in target.iterdir())
        check(listing == ["history.json", "rl_module", "v26c_j8_recovery_fit_receipt.json"],
              f"the leaf holds the module directory, the receipt and the history ({listing})")
        module = sorted(p.name for p in (target / "rl_module").iterdir())
        check(module == ["actor_feature_manifest.json", "class_and_ctor_args.pkl",
                         "metadata.json", "module_state.pkl"],
              f"and rl_module holds the state plus the three sidecars ({module})")
        for name, pin in J8.SIDECARS.items():
            check(J8._sha_file(target / "rl_module" / name) == pin
                  == J8._sha_file(J8.PARENT_MODULE_DIR / name),
                  f"the sidecar {name} is BYTE-IDENTICAL to the parent's")
        with (target / "rl_module" / "module_state.pkl").open("rb") as fh:
            saved = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}
        check(tuple(sorted(saved)) == tuple(sorted(J8.STATE_KEYS)) and len(saved) == 10,
              "the saved state round-trips with exactly the parent's ten keys")
        check(all(saved[k].shape == parent[k].shape for k in parent),
              "with identical shapes")
        check(all(np.array_equal(saved[a], saved[d]) for a, d in J8.ALIAS_PAIRS),
              "the aliases are bit-identical on disk")
        check(float(np.max(np.abs(saved["pi.0.0.weight"][:, [0, 1]]))) == 0.0
              and float(np.max(np.abs(saved["pi_encoder.0.weight"][:, [0, 1]]))) == 0.0,
              "the clock columns are bit-zero on disk in BOTH aliases")
        check(np.array_equal(saved["pi.1.weight"][2:], parent["pi.1.weight"][2:])
              and np.array_equal(saved["pi.1.bias"][2:], parent["pi.1.bias"][2:]),
              "and the logstd head is bit-identical to the parent's")

        rc = json.loads((target / J8.RECEIPT_NAME).read_text())
        check(rc["stage"] == J8.STAGE and rc["schema"] == "v26c_j8_recovery_fit_receipt.1",
              "the receipt names the stage")
        check(rc["inputs"]["dataset"]["sha256"] == J8.PIN_DATASET
              and rc["inputs"]["parent"]["module_state_sha256"] == J8.PIN_PARENT_STATE
              and rc["inputs"]["july_scaling_source"]["sha256"] == J8.PIN_JULY_SCALES_SOURCE,
              "and records every input hash")
        check(rc["inputs"]["dataset"]["builder_imported"] is False
              and rc["inputs"]["dataset"]["rebuilt"] is False
              and rc["inputs"]["j4_reference"]["imported"] is False,
              "attesting that the dataset was read, not rebuilt, and J4 was not imported")
        check(rc["split"]["digest"] == s1["digest"]
              and len(rc["split"]["train_indices"]) == 13370
              and len(rc["split"]["validation_indices"]) == 3343,
              "the split digest AND the full index arrays are in the receipt")
        check(rc["history"] == fitted["history"] and len(rc["history"]) == 3,
              "the complete history is in the receipt")
        check(json.loads((target / "history.json").read_text()) == fitted["history"],
              "and in history.json")
        check(rc["selection"]["reconstructible"] is True
              and rc["selection"]["recomputed_best_epoch"] == rc["selection"]["best_epoch"],
              "the best state is reconstructible from the recorded history")
        check(rc["output"]["files"][f"rl_module/module_state.pkl"]
              == J8._sha_file(target / "rl_module" / "module_state.pkl"),
              "the output hashes are the files' actual hashes")
        check(set(rc["gate"]["binding_integrity"]) >= {
            "keys_and_shapes_match_parent", "all_parameters_finite", "all_metrics_finite",
            "clock_bit_zero", "aliases_bit_identical", "logstd_bit_identical_to_parent",
            "no_critic_key", "inputs_unchanged", "best_state_reconstructible_from_history"}
            and set(rc["gate"]["binding_outcome"]) == {"recovery_rmse_decreases",
                                                       "controller_columns_nonzero"},
            "every binding invariant is attested in the receipt")
        check(rc["inert"] == {"critic_touched": False, "ppo_updates": 0,
                              "environment_constructed": False, "rollout_executed": False,
                              "closed_loop_evaluated": False, "dataset_rebuilt": False},
              "no critic, no PPO, no environment, no rollout, no closed loop")
        check(rc["outcome"]["deployable"] is False and rc["outcome"]["promotion"] == "NONE"
              and rc["outcome"]["next_stage_authorized"] is False
              and rc["outcome"]["closed_loop"] is False,
              "deployable false, promotion NONE, no next stage, closed_loop false")
        check(rc["output"]["sidecars_byte_identical_to_parent"] is True, "sidecars attested")
        jr = rc["july_faithfulness"]
        check(jr["does_not_replicate"] == ["July's dataset", "July's parent", "July's actor",
                                           "July's output"]
              and "a reproduction of the July run" in jr["is_not"]
              and jr["july_multistart_training_block"] == "ABSENT / DEFERRED",
              "the receipt states precisely what July-faithful does and does not mean")
        rpc = rc["parent_preconditions"]
        check(rpc["controller_columns_bit_zero"]["all_bit_zero"] is True
              and rpc["raw_vs_scaled_equivalence"]["max_abs_diff"] == 0.0
              and rpc["raw_vs_scaled_equivalence"]["bit_identical"] is True,
              "and records both binding preconditions with their measured values")
        rd = rc["determinism"]
        check(rd["replicated"].startswith("July's DOCUMENTED seeding protocol")
              and "Generator / split / shuffle order" in rd["replicated"],
              "the receipt claims the DOCUMENTED seeding protocol and the Generator/split/"
              "shuffle order")
        check(rd["torch_use_deterministic_algorithms"] == "NOT SET by J8"
              and rd["torch_num_threads"] == "NOT SET by J8"
              and "not forced and not claimed" in rd["not_replicated"],
              "and states plainly that neither backend setting is forced or claimed")
        tb = rd["torch_backend_observed"]
        check(tb["forced_by_j8"] is False and tb["binding"] is False
              and set(tb) >= {"are_deterministic_algorithms_enabled", "get_num_threads"},
              "the OBSERVED backend values are recorded, non-binding, forced_by_j8 false")
        check(rd["executive_order"] == ["import torch", "torch.manual_seed", "np.random.seed",
                                        "deterministic init from parent (no RNG)",
                                        "np.default_rng in build_split", "optimizer"]
              and rd["executive_order_source"] == "target_domain_imitation.adapt_actor",
              "and the executive order is recorded with its July source")
        check("the determinism protocol" not in json.dumps(rc),
              "the phrase is absent from the ENTIRE receipt")
        rls = rc["determinism"]["numpy_legacy_global_seed"]
        check(rls["call"] == "np.random.seed" and rls["value"] == 123 and rls["count"] == 1
              and rls["is_a_generator"] is False,
              "the legacy global seed is recorded unambiguously as NOT a Generator")
        check(rc["determinism"]["generators"]["count"] == 1
              and rc["determinism"]["generators"]["call"] == "np.random.default_rng",
              "alongside the single Generator")
        json.dumps(rc, allow_nan=False)
        check(True, "the receipt is strictly JSON: no NaN, no Infinity")

        after_first = snapshot(target)
        e = expect(lambda: J8.validate_out(str(target)), J8.J8Error,
                   "a second run must be refused")
        check("no-clobber and single-execution" in str(e), "because the leaf already exists")
        check(snapshot(target) == after_first, "MEASURED: the existing leaf was left untouched")
        check(sentinel.is_file(), "and the concurrent sentinel is still there")
    finally:
        J8.OUTPUT_ROOT_OVERRIDE = saved_root
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------------------------------------------------------- static guarantees ---------
    imported = {n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    check(imported == {"__future__", "argparse", "ast", "hashlib", "json", "os", "pickle",
                       "shutil", "sys", "pathlib", "typing", "numpy", "torch", "torch.nn"},
          f"the WHOLE import surface is the standard library, numpy and torch ({sorted(imported)})")
    check(not any(m.startswith(("v26c_j4", "v26c_j5", "v26c_j6", "v26c_j7")) for m in imported),
          "J8 imports no J4/J5/J6/J7 tooling: it reads their artefacts by hash")
    check(not any(i.startswith(("v26c_j4", "v26c_j5", "v26c_j6", "v26c_j7")) for i in ids),
          "and names none of them as an identifier")
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check("torch" not in top,
          "torch is NOT a module-scope import: the preflight can stay free of it")
    run_fit_fn = next(n for n in ast.walk(tree)
                      if isinstance(n, ast.FunctionDef) and n.name == "run_fit")
    torch_imports = [n for n in ast.walk(run_fit_fn) if isinstance(n, ast.Import)
                     and n.names[0].name == "torch"]
    check(len(torch_imports) == 1, "it is imported lazily, inside run_fit, exactly once")
    for name in ("PPOConfig", "train_ppo", "make_cmc_env", "env_factory", "rollout_eval",
                 "opensim", "gymnasium", "ray"):
        check(name not in ids, f"the runner never references {name}")
    check(src.count("shutil.rmtree") == 1,
          "there is exactly ONE recursive removal in the whole module")
    commit_fn = next(n for n in ast.walk(tree)
                     if isinstance(n, ast.FunctionDef) and n.name == "commit")
    rmtrees = [n for n in ast.walk(commit_fn) if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
    check(len(rmtrees) == 1 and isinstance(rmtrees[0].args[0], ast.Name)
          and rmtrees[0].args[0].id == "staging_created",
          "and its only argument is staging_created - never the parent, never j8_runs")
    rmdirs = [n for n in ast.walk(commit_fn) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "rmdir"]
    check(len(rmdirs) == 1 and isinstance(rmdirs[0].func.value, ast.Name)
          and rmdirs[0].func.value.id == "parent_created" and not rmdirs[0].args,
          "the parent gets a bare, non-recursive rmdir and nothing more")
    check("O_CREAT | os.O_EXCL" in src, "the lock is taken with O_CREAT|O_EXCL")

    run_fit_node = next(n for n in ast.walk(tree)
                        if isinstance(n, ast.FunctionDef) and n.name == "run_fit")
    # ---------------------------------- the backend is NOT forced, and the order IS July's ------
    det_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute)
                 and n.func.attr == "use_deterministic_algorithms"]
    check(len(det_calls) == 0,
          f"AST: ZERO calls to use_deterministic_algorithms ({len(det_calls)})")
    thread_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == "set_num_threads"]
    check(len(thread_calls) == 0, f"AST: ZERO calls to set_num_threads ({len(thread_calls)})")
    check(not [n for n in ast.walk(tree) if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute)
               and n.func.attr in ("use_deterministic_algorithms", "set_num_threads")],
          "neither is called with False or any other argument: they are simply absent")
    # the OBSERVED values are read, never set
    observed = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr in ("are_deterministic_algorithms_enabled", "get_num_threads")}
    check(observed == {"are_deterministic_algorithms_enabled", "get_num_threads"},
          f"the runner OBSERVES both backend values instead of forcing them ({sorted(observed)})")

    # the real executive order inside run_fit, by line number, not by mere presence
    def first_line(node, pred):
        hits = [n.lineno for n in ast.walk(node) if pred(n)]
        return min(hits) if hits else None

    rf = run_fit_node
    l_import = first_line(rf, lambda n: isinstance(n, ast.Import)
                          and n.names[0].name == "torch")
    l_manual = first_line(rf, lambda n: isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Attribute) and n.func.attr == "manual_seed")
    l_npseed = first_line(rf, lambda n: isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Attribute) and n.func.attr == "seed")
    l_split = first_line(rf, lambda n: isinstance(n, ast.Call)
                         and isinstance(n.func, ast.Name) and n.func.id == "build_split")
    l_adam = first_line(rf, lambda n: isinstance(n, ast.Call)
                        and isinstance(n.func, ast.Attribute) and n.func.attr == "Adam")
    l_params = first_line(rf, lambda n: isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Attribute) and n.func.attr == "Parameter")
    check(None not in (l_import, l_manual, l_npseed, l_split, l_adam, l_params),
          "every step of the executive order is present in run_fit")
    check(l_import < l_manual < l_npseed < l_params < l_split < l_adam,
          f"JULY EXECUTIVE ORDER: import torch ({l_import}) -> manual_seed ({l_manual}) -> "
          f"np.random.seed ({l_npseed}) -> deterministic init ({l_params}) -> "
          f"default_rng in build_split ({l_split}) -> optimizer ({l_adam})")
    l_pre = first_line(rf, lambda n: isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                       and n.func.id == "parent_preconditions")
    check(l_pre < l_import,
          "the numpy input verification and preconditions run BEFORE the torch import, which is "
          "what lets the preflight stay torch-free")
    check("manual_seed" in src, "the torch manual seed is still set, as July sets it")

    # ------------------------------------------------- the scales against the REAL module -------
    # Deliberately last: this import pulls torch, which is exactly why the runner extracts the
    # literal by AST instead of importing it.
    check("torch" not in sys.modules, "torch is still absent right up to this point")
    sys.path.insert(0, str(J8.REPO / "Trajectory Generator" / "baseline_MLP"))
    import target_domain_markov_adaptation as JM  # noqa: E402
    check("torch" in sys.modules,
          "MEASURED: importing the July module pulls torch - the reason the runner never does")
    check(extracted == JM.MARKOV_CONTROLLER_FEATURE_SCALES,
          "PROVEN: the AST extraction equals the real module's MARKOV_CONTROLLER_FEATURE_SCALES")
    check(J8.preflight()["verdict"] == "GO"
          and J8.preflight()["inert"]["torch_imported"] is True,
          "and the preflight still passes with torch loaded by someone else, reporting the fact "
          "without treating it as a blocker")

    # ------------------------------------------- the J7 materialize suite is SPENT, not broken --
    # test_v26c_j7_materialize.py asserts its preflight is GO. That precondition died the moment
    # the authorised J7 leaf was materialised. It is a historical, non-rerunnable-in-GO-state
    # artefact, it is NOT modified, and it is NOT a J8 blocker. Only its applicable read-only
    # checks are exercised here.
    hist = prereg["historical_tests"]["test_v26c_j7_materialize.py"]
    check("EXPECTED FAIL" in hist and "NOT a regression" in hist and "SPENT" in hist,
          "the prereg documents the J7 materialize suite as expected/spent, not a regression")
    import v26c_j7_materialize as M7  # noqa: E402
    m7 = M7.preflight()
    check(m7["verdict"] == "BLOCKED"
          and any("already exists" in b for b in m7["blockers"]),
          f"and the cause is exactly the spent stage ({m7['blockers']})")
    check(M7.authorized_leaf().exists()
          and M7.authorized_leaf() == HERE / "j7_runs" / "j7_markov_dataset_v26c_2026-08-26_r1",
          "the authorised J7 leaf exists: the stage was executed on GO")
    # the applicable read-only checks from that layer still hold, and are exercised here
    a7 = M7.verify_authorization()
    check(a7["sha256"] == M7.PIN_AUTHORIZATION
          and a7["bootstrap"]["reused_a_preloaded_module"] is False,
          "the J7 authorisation still verifies, with its bootstrap intact")
    check(all(M7._sha_file(HERE / r) == h for r, h in M7.FROZEN.items()),
          "and the three frozen J7 inputs are still byte-identical")
    check(J8._sha_file(J8.DATASET) == J8.PIN_DATASET
          and M7._sha_file(M7.authorized_leaf() / M7.DATASET_NAME) == J8.PIN_DATASET,
          "the materialised dataset is the one J8 pins, seen from both layers")
    check(not any("j8" in b.lower() for b in m7["blockers"]),
          "nothing about that BLOCKED verdict concerns J8")

    # ---------------------------------------------------------------- nothing was disturbed -----
    check(not (HERE / "j8_runs").exists(),
          "MEASURED: this suite never created j8_runs under the validation root")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")
    check(J8._sha_file(J8.DATASET) == J8.PIN_DATASET
          and J8._sha_file(J8.PARENT_MODULE_DIR / "module_state.pkl") == J8.PIN_PARENT_STATE,
          "the J7 dataset and the J2 parent are byte-identical afterwards")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
