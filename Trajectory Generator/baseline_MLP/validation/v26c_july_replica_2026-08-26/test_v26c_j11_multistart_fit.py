#!/usr/bin/env python
"""Fail-closed tests for the V26C J11 multistart offline fit.

Hermetic: the authorised leaf is never created, and every write path is exercised in a temporary
root through OUTPUT_ROOT_OVERRIDE with an INJECTED fit, so no test ever runs a real 400-epoch
training run.

DELIBERATELY NOT BORN SPENT. The J8 suite asserts unconditionally that its own preflight is GO
and that its run directory does not exist; both assertions died the moment J8 committed, which
is why that suite cannot be re-run today as evidence. Here every such assertion is CONDITIONAL on
the leaf not yet existing, and the post-commit state is asserted instead once it does.
"""
from __future__ import annotations
import ast, builtins, hashlib, io, json, os, pickle, shutil, sys, tempfile, tokenize
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j11_multistart_fit as J11  # noqa: E402
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
        if p.is_file() and "__pycache__" not in p.parts:
            out[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


def synthetic_fit(*, pass_gate: bool = True) -> tuple[dict, dict]:
    """A fit result shaped exactly like run_fit's, built WITHOUT torch and without training.

    The 'after' predictions are the labels nudged slightly closer than 'before', which is what a
    real successful fit produces on every binding subset.
    """
    data = J11.build_aggregate()
    parent = J11.load_parent_state()
    names = data["names"]
    obs, act = data["observations"], data["actions"]
    before = J11._numpy_forward(parent, obs)[:, :J11.ACTION_DIM]
    final = {k: np.array(v, dtype=np.float32, copy=True) for k, v in parent.items()}
    # bring the controller block to life, exactly as a real fit does
    rng = np.random.default_rng(7)
    for c in range(*J11.CONTROLLER_SPAN):
        col = rng.standard_normal(final["pi.0.0.weight"].shape[0]).astype(np.float32) * 1e-3
        final["pi.0.0.weight"][:, c] = col
    final["pi.0.0.weight"][:, list(J11.CLOCK_COLUMNS)] = 0.0
    for alias, direct in J11.ALIAS_PAIRS:
        final[alias] = final[direct].copy()
    # 0.5 halves the residual (RMSE falls); -1.0 moves AWAY from the label and doubles it
    # (RMSE rises) on every subset. 1.5 would NOT do: it overshoots to |0.5 * residual|,
    # which is still an improvement.
    frac = 0.5 if pass_gate else -1.0
    after = (before + frac * (act - before)).astype(np.float32)
    history = [{"epoch": 1.0, "train_loss": 0.02, "validation_mse": 0.02},
               {"epoch": 2.0, "train_loss": 0.01, "validation_mse": 0.01}]
    split = J11.build_split(J11.TOTAL_ROWS)
    fit = {"final": final, "parent": parent, "prereg": {"file": "synthetic", "sha256": "0" * 64},
           "data": data, "split": split, "history": history, "best_epoch": 2,
           "best_validation_mse": 0.01, "stopped_early": False, "epochs_run": 2,
           "before": before, "after": after, "names": names,
           "clock": list(J11.CLOCK_COLUMNS),
           "controller": list(J8.controller_indices(names)),
           "scales_verified": dict(J8.MARKOV_CONTROLLER_FEATURE_SCALES),
           "preconditions": J8.parent_preconditions(parent, names, obs),
           "torch_backend_observed": {"forced_by_j11": False, "binding": False},
           "modules": {"v26c_j8_recovery_fit.py": J11.PIN_J8_MODULE},
           "seed_order": ["synthetic"],
           "numpy_legacy_seed": {"call": "np.random.seed", "value": 123, "count": 1}}
    return fit, J11.audit(fit)


def main() -> int:
    src = (HERE / "v26c_j11_multistart_fit.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    prereg = json.loads(J11.PREREG.read_text())
    before_all = snapshot(HERE)
    leaf_exists_at_start = J11.authorized_leaf().exists()

    # ---------------------------------------------------------------- preregistration ----------
    check(prereg["kind"] == "ADDITIVE, IMMUTABLE, PREREGISTRATION"
          and prereg["stage_proposed"] == "V26C_J11_MULTISTART_FIT" == J11.STAGE,
          "the prereg is additive, immutable and proposes the J11 stage")
    check("stage_authorised" not in prereg
          and prereg["authorisation_status"].startswith("NOT_GRANTED"),
          "it does not claim an authorisation it cannot grant")
    check(J11._sha_file(J11.PREREG) == J11.PIN_PREREG
          == "49e352466cc82ea9c0a1d7bf29608d41ba4457dfb8a83f3a050a6f1cd752e472",
          "and is pinned by exact hash")
    check(all(J11._sha_file(HERE / r) == h
              for r, h in prereg["pinned_artefacts_sha256"].items()),
          f"all {len(prereg['pinned_artefacts_sha256'])} local pins verify on disk")
    check(all(J11._sha_file(J11.REPO / r) == h
              for r, h in prereg["pinned_repo_artefacts_sha256"].items()),
          f"all {len(prereg['pinned_repo_artefacts_sha256'])} repo pins verify on disk")

    # ---------------------------------------------------------------- lineage ------------------
    check(prereg["parent"]["is_j2"] is True and prereg["parent"]["is_j8"] is False
          and prereg["parent"]["is_july"] is False,
          "the parent is J2: not J8, not July")
    check(J11.PIN_PARENT_STATE
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          != J11.PIN_OBSERVATION_CONTRACT_ANCESTOR,
          "the weight parent and the observation-contract ancestor are DISTINCT hashes")
    check(J11.PIN_OBSERVATION_CONTRACT_ANCESTOR
          == "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd",
          "and the contract ancestor is the 39-wide August V26 imitative actor")
    j8_leaf = HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1" / "rl_module"
    check(J11._sha_file(j8_leaf / "module_state.pkl") != J11.PIN_PARENT_STATE,
          "MEASURED: the J8 student's module is NOT this stage's parent")
    check("j8" not in str(J11.PARENT_MODULE_DIR).lower()
          and "j2_runs" in str(J11.PARENT_MODULE_DIR),
          "and the parent directory is the J2 leaf")

    # ---------------------------------------------------------------- the parent ---------------
    parent = J11.load_parent_state()
    check(sorted(parent) == sorted(J11.STATE_KEYS) and len(parent) == 10,
          "the parent holds the ten actor keys")
    check(parent["pi.0.0.weight"].shape == (256, 35)
          and parent["pi.1.weight"].shape == (4, 256),
          "35 in, 4 out: two means and two log-stds")
    check(all(np.asarray(v).dtype == np.float32 for v in parent.values()),
          "every parent tensor is float32")
    W = parent["pi.0.0.weight"]
    check(bool(np.all(W[:, list(J11.CLOCK_COLUMNS)] == 0.0)),
          "MEASURED: the parent's clock columns are EXACTLY zero")
    check(bool(np.all(W[:, J11.CONTROLLER_SPAN[0]:J11.CONTROLLER_SPAN[1]] == 0.0)),
          "MEASURED: and so are the ten controller columns - J2 is the double-masked base")
    check(bool(np.all(parent["pi.1.weight"][2:] == 0.0)),
          "the log-std rows carry no weight: sigma is state-independent")
    check(not any("critic" in k or k.startswith("vf") for k in parent),
          "the parent carries no critic tensor")

    # ---------------------------------------------------------------- the aggregate ------------
    data = J11.build_aggregate()
    obs, act, names = data["observations"], data["actions"], data["names"]
    check(obs.shape == (24713, 35) and act.shape == (24713, 2),
          f"the aggregate is 24713 x 35 / 24713 x 2 ({obs.shape}/{act.shape})")
    check(obs.dtype == np.float32 and act.dtype == np.float32, "float32 throughout")
    check(J11.TOTAL_ROWS == 16713 + 4000 + 4000 == 24713,
          "16713 + 4000 + 4000 = 24713")
    check(len(names) == 35 and names[0] == "gait_phase_sin" and names[1] == "gait_phase_cos"
          and names[17] == "phase_fsm_wait_hs",
          "the 35 runtime feature names, with the clock first and wait_hs at 17")

    # ---- TILE, not REPEAT: the distinction the layout depends on
    for b in data["blocks"]:
        seg = obs[b["start"]:b["stop"]]
        if b["tiled"]:
            head = seg[:b["unique"]]
            check(all(np.array_equal(seg[r * b["unique"]:(r + 1) * b["unique"]], head)
                      for r in range(b["repeat"])),
                  f"{b['id']}: all {b['repeat']} tiles are bit-identical to the first")
            check(b["distinct_rows_measured"] == b["unique"],
                  f"{b['id']}: exactly {b['unique']} distinct rows, as tiling implies")
            if b["unique"] > 1:
                check(not np.array_equal(seg[0], seg[1]),
                      f"{b['id']}: consecutive rows DIFFER - this is np.tile, not np.repeat")
        else:
            check(b["repeat"] == 1, f"{b['id']} is taken once")
    check([b["id"] for b in data["blocks"]]
          == ["j7_nominal", "j7_recovery", "cell_B", "cell_C"],
          "the order is J7 nominal, J7 recovery, cell B, cell C")
    check([(b["start"], b["stop"]) for b in data["blocks"]]
          == [(0, 16000), (16000, 16713), (16713, 20713), (20713, 24713)],
          "and the block boundaries are exactly as preregistered")
    check(data["repeat_semantics"].startswith("np.tile"), "np.tile is declared")

    # a np.repeat build would have the same SHAPE and different content - prove we would notice
    repeat_style = np.repeat(obs[16713:17213], 8, axis=0)
    check(repeat_style.shape == obs[16713:20713].shape
          and not np.array_equal(repeat_style, obs[16713:20713]),
          "MEASURED: np.repeat would give the SAME SHAPE and DIFFERENT content, so the "
          "distinction is not cosmetic")

    # ---- the sources survive intact
    z7 = np.load(J11.J7_DATASET, allow_pickle=False)
    check(np.array_equal(obs[:16713], z7["observations"])
          and np.array_equal(act[:16713], z7["actions"]),
          "the J7 rows are bit-identical inside the aggregate")
    for cid, start in (("B", 16713), ("C", 20713)):
        zc = np.load(J11.CELL_DATASETS[cid][0], allow_pickle=False)
        proj = zc["observations"].copy()
        proj[:, :2] = 0.0
        check(np.array_equal(obs[start:start + 500], proj),
              f"cell {cid}'s PROJECTED rows are bit-identical inside the aggregate")
        check(np.array_equal(act[start:start + 500], zc["actions"]),
              f"cell {cid}'s prescribed labels are bit-identical inside the aggregate")
        check(np.array_equal(zc["actions"], zc["executed_actions"]),
              f"cell {cid}: the labels ARE the executed actions, bit for bit")
        check(float(np.max(np.abs(zc["action_noises"]))) == 0.0,
              f"cell {cid}: the action noise is exactly zero")

    # ---------------------------------------------------------------- the clock fact -----------
    for cid in ("B", "C"):
        zc = np.load(J11.CELL_DATASETS[cid][0], allow_pickle=False)
        raw = zc["observations"]
        check(sorted({float(v) for v in raw[:, 0]}) == [0.0]
              and sorted({float(v) for v in raw[:, 1]}) == [1.0],
              f"MEASURED: cell {cid} records the RAW clock as (sin, cos) = (0, 1)")
    check(float(np.max(np.abs(z7["observations"][:, :2]))) == 0.0,
          "MEASURED: J7 is already projected to exact zero")
    check(float(np.max(np.abs(obs[:, :2]))) == 0.0,
          "and the aggregate's clock columns are EXACTLY zero, not merely small")
    src_b = data["sources"]["cell_B"]
    check(src_b["raw_clock"]["col_1_unique"] == [1.0]
          and src_b["content_hashes"]["observations_raw"]
          != src_b["content_hashes"]["observations_projected"],
          "both the RAW and the PROJECTED content hashes are recorded, and they differ")
    check("never a reason to reject" in data["clock_projection"]["expected_not_a_defect"],
          "a raw cos of 1 is documented as expected, not as a defect")
    # the projection touches ONLY the clock
    for cid, start in (("B", 16713), ("C", 20713)):
        zc = np.load(J11.CELL_DATASETS[cid][0], allow_pickle=False)
        check(np.array_equal(obs[start:start + 500, 2:], zc["observations"][:, 2:]),
              f"cell {cid}: columns 2..34 are untouched by the projection")

    # ---------------------------------------------------------------- provenance ---------------
    prov = data["j10r1_provenance"]
    check(prov["commit_verification_pass"] is True and prov["verdict"] == "PASS"
          and prov["technical_invalid_marker"] is False,
          "the J10R1 leaf is consumed only because its own commit verification PASSED")
    check(prov["dataset_hashes_agree_with_the_j10r1_receipt"] is True,
          "and the J10R1 receipt's binding dataset hashes are honoured before consumption")

    # ---------------------------------------------------------------- the split ----------------
    split = J11.build_split(J11.TOTAL_ROWS)
    check(split["n_val"] == 4943 and split["n_train"] == 19770
          and split["n_val"] + split["n_train"] == 24713,
          "the split is 4943 / 19770")
    # reconstructed INDEPENDENTLY, not by calling the same helper
    rng = np.random.default_rng(123)
    idx = rng.permutation(24713)
    n_val = max(1, int(round(24713 * 0.2)))
    check(n_val == 4943 and int(24713 * 0.2) == 4942,
          "int(round(24713*0.2)) = 4943, while a truncating int() would give 4942")
    check(np.array_equal(split["val_idx"], np.sort(idx[:n_val])),
          "MEASURED: the validation half is the sorted first slice of a fresh default_rng(123)")
    check(np.array_equal(split["train_idx"], np.asarray(idx[n_val:], dtype=np.int64)),
          "and the training half keeps the permutation's order, unsorted")
    check(bool(np.all(np.diff(split["val_idx"]) > 0)), "validation is sorted")
    check(not bool(np.all(np.diff(split["train_idx"]) > 0)), "training is NOT sorted")
    check(split["train_idx"].dtype == np.int64 and split["val_idx"].dtype == np.int64,
          "both index arrays are int64 explicitly, so Windows matches macOS")
    # sorting the training half would change every epoch
    g1, g2 = np.random.default_rng(123), np.random.default_rng(123)
    g1.permutation(24713), g2.permutation(24713)
    check(not np.array_equal(g1.permutation(split["train_idx"]),
                             g2.permutation(np.sort(split["train_idx"]))),
          "PROVEN: sorting the training half would change the epoch batch sequence")
    # a declared count that does not hold must abort
    real_bs = J8.build_split
    try:
        J8.build_split = lambda rows: {**real_bs(rows), "n_val": 1, "n_train": rows - 1}
        expect(lambda: J11.build_split(J11.TOTAL_ROWS), J11.J11Error,
               "a split that does not produce the declared counts is REFUSED")
    finally:
        J8.build_split = real_bs
    check(J11.build_split(J11.TOTAL_ROWS)["n_val"] == 4943, "restored")

    # ---------------------------------------------------------------- preconditions ------------
    pre = J8.parent_preconditions(parent, names, obs)
    check(pre["controller_columns_bit_zero"]["all_bit_zero"] is True
          and pre["controller_columns_bit_zero"]["binding"] is True,
          "BINDING: the parent's controller columns are bit-zero")
    check(pre["raw_vs_scaled_equivalence"]["max_abs_diff"] == 0.0
          and pre["raw_vs_scaled_equivalence"]["bit_identical"] is True,
          "BINDING: raw and physically scaled inputs give bit-identical output, difference "
          "EXACTLY 0.0 - which is what makes absorbing the scale legitimate")
    bad_parent = {k: np.array(v, copy=True) for k, v in parent.items()}
    bad_parent["pi.0.0.weight"][0, 25] = np.float32(0.1)
    expect(lambda: J8.parent_preconditions(bad_parent, names, obs), J8.J8Error,
           "a parent with a live controller column is REFUSED before the optimizer exists")

    # ---------------------------------------------------------------- the actor digest ---------
    check(J11._sha_file(J11.WARM_START_SOURCE) == J11.PIN_WARM_START,
          "warm_start.py is pinned")
    sys.path.insert(0, str(J11.REPO / "Trajectory Generator" / "baseline_MLP"))
    import warm_start as WS  # noqa: E402
    check(J11.actor_state_digest(parent) == WS.actor_state_digest(parent),
          "MEASURED: the TRANSCRIBED actor_state_digest reproduces warm_start's own function "
          "exactly - transcribed rather than imported, because importing pulls torch")
    check(sorted(J11.WARM_START_ACTOR_KEYS) == sorted(WS._ACTOR_KEYS),
          "and over the same ten keys")
    check(J11.actor_state_digest(parent) != J11.actor_state_digest(
              {**parent, "pi.1.bias": parent["pi.1.bias"] + np.float32(1e-6)}),
          "the digest is sensitive to a single perturbed tensor")

    # ---------------------------------------------------------------- the gate -----------------
    fit_ok, gate_ok = synthetic_fit(pass_gate=True)
    check(gate_ok["pass"] is True and gate_ok["failed"] == [],
          f"a well-formed synthetic fit PASSES the gate ({gate_ok['failed']})")
    for key in ("aggregate", "j7_recovery_original", "cell_B_unique", "cell_C_unique"):
        r = gate_ok["rmse"][key]
        check(r["binding"] is True and r["decreased"] is True and r["after"] < r["before"],
              f"binding subset {key}: RMSE after < before ({r['after']:.6g} < {r['before']:.6g})")
    check(gate_ok["rmse"]["j7_recovery_original"]["rows"] == 713
          and gate_ok["rmse"]["cell_B_unique"]["rows"] == 500
          and gate_ok["rmse"]["cell_C_unique"]["rows"] == 500
          and gate_ok["rmse"]["aggregate"]["rows"] == 24713,
          "the four binding subsets are 24713 / 713 / 500 / 500 rows")
    check(gate_ok["rmse"]["cell_B_unique"]["slice"] == [16713, 17213]
          and gate_ok["rmse"]["cell_C_unique"]["slice"] == [20713, 21213],
          "and the cell subsets are the UNIQUE 500 rows, not the tiled 4000")
    check(gate_ok["controller_columns_nonzero"] is True
          and len(gate_ok["controller_column_norms"]) == 10
          and all(v > 0.0 for v in gate_ok["controller_column_norms"].values()),
          "all ten controller columns are live after the fit")
    for k in ("nominal_rmse", "nominal_mean_shift", "clipping_out_of_bounds_rows",
              "best_validation_mse"):
        check(gate_ok["diagnostics"][k]["binding"] is False,
              f"{k} is a DIAGNOSTIC, explicitly not binding")
    check("STRICT inequality" in gate_ok["no_invented_thresholds"],
          "the gate declares that it invents no threshold")
    # AST: no binding rule may compare against a numeric literal margin
    audit_fn = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == "audit")
    # scoped to the BINDING logic: the diagnostics block legitimately compares |action| > 1.0,
    # which is the action bound, not a gate threshold
    diag_assign = next(n for n in ast.walk(audit_fn) if isinstance(n, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id == "diagnostics"
                               for t in n.targets))
    diag_nodes = {id(n) for n in ast.walk(diag_assign)}
    literal_cmps = [n for n in ast.walk(audit_fn)
                    if isinstance(n, ast.Compare) and id(n) not in diag_nodes
                    and any(isinstance(c, ast.Constant) and isinstance(c.value, float)
                            and c.value != 0.0 for c in n.comparators)]
    check(not literal_cmps,
          f"AST: no BINDING comparison uses a non-zero float literal - no invented margin "
          f"(offenders at lines {[n.lineno for n in literal_cmps]})")
    diag_lits = [n for n in ast.walk(diag_assign) if isinstance(n, ast.Compare)
                 and any(isinstance(c, ast.Constant) and c.value == 1.0 for c in n.comparators)]
    check(len(diag_lits) == 1,
          "and the single 1.0 in the whole function is the action bound, inside the "
          "explicitly non-binding clipping diagnostic")

    fit_bad, gate_bad = synthetic_fit(pass_gate=False)
    check(gate_bad["pass"] is False and gate_bad["failed"],
          f"an overshooting fit FAILS, naming what failed ({gate_bad['failed']})")
    check("aggregate_rmse_decreases" in gate_bad["failed"], "including the aggregate RMSE")

    # each integrity violation, individually
    def broken(mutate):
        f, _ = synthetic_fit()
        mutate(f["final"])
        return J11.audit(f)

    g = broken(lambda s: s.__setitem__("pi.0.0.weight",
                                       _with(s["pi.0.0.weight"], (0, 0), 0.5)))
    check("integrity_aliases_bit_identical" in g["failed"],
          "a direct tensor changed without its alias FAILS on alias bit-identity")
    g = broken(lambda s: s.__setitem__("pi.1.bias", _with(s["pi.1.bias"], (2,), 0.0)))
    check("integrity_logstd_bit_identical_to_parent" in g["failed"],
          "a touched log-std row FAILS")
    g = broken(lambda s: s.update({"vf.weight": np.zeros((1, 256), np.float32)}))
    check("integrity_no_critic_key" in g["failed"], "a critic key FAILS")

    def clock_live(s):
        s["pi.0.0.weight"] = _with(s["pi.0.0.weight"], (0, 1), 0.2)
        for a, d in J11.ALIAS_PAIRS:
            s[a] = s[d].copy()
    check("integrity_clock_bit_zero" in broken(clock_live)["failed"],
          "a live clock column FAILS, in the direct tensor AND its alias")

    def controller_dead(s):
        s["pi.0.0.weight"] = np.array(s["pi.0.0.weight"], copy=True)
        s["pi.0.0.weight"][:, 25:35] = 0.0
        for a, d in J11.ALIAS_PAIRS:
            s[a] = s[d].copy()
    check("controller_columns_nonzero" in broken(controller_dead)["failed"],
          "a dead controller block FAILS")

    # ---------------------------------------------------------------- the manifest -------------
    state_sha = "f" * 64
    man = J11.build_manifest(fit_ok, gate_ok, state_sha)
    check(man["is_a_copy_of_the_parent_manifest"] is False
          and man["module_state_sha256"] == state_sha,
          "the manifest is REGENERATED and names the module beside it")
    parent_man = json.loads((J11.PARENT_MODULE_DIR / "actor_feature_manifest.json").read_text())
    check(man["module_state_sha256"] != parent_man["module_state_sha256"],
          "MEASURED: it does NOT carry the parent's module hash - the J8 defect")
    j8_man = json.loads((j8_leaf / "actor_feature_manifest.json").read_text())
    check(j8_man["module_state_sha256"] == parent_man["module_state_sha256"]
          != J11._sha_file(j8_leaf / "module_state.pkl"),
          "MEASURED: J8's committed manifest names its PARENT's module, not its own")
    check(j8_man["controller_state_mask"]["active"] is True,
          "and declares the controller block masked, which J8's own module contradicts")
    check(man["controller_contract"]["masked"] is False
          and man["controller_contract"]["state"].startswith("LIVE"),
          "J11's manifest declares the controller block LIVE, which is what its fit produces")
    check(man["clock_contract"]["weights_are_exactly_zero"] is True
          and man["clock_contract"]["columns"] == [0, 1],
          "and the clock contract is truthful")
    check(man["actor_feature_names"] == list(names) and man["actor_feature_count"] == 35,
          "the 35 runtime names, in order")
    check(man["actor_digest"] == J11.actor_state_digest(fit_ok["final"])
          != man["source_actor_digest"] == J11.actor_state_digest(parent),
          "actor_digest describes THIS module; source_actor_digest describes the parent")
    check("actor_digest" not in parent_man and "actor_digest" not in j8_man,
          "MEASURED: neither J2's nor J8's manifest carries actor_digest - the hole J8 slipped "
          "through")
    check(man["deployable"] is False and "PENDING" in man["closed_loop_qualification"]
          and man["critic"].startswith("ABSENT"),
          "it claims no deployability, leaves closed-loop PENDING and declares no critic")
    check(man["offline_gate_pass"] is True and "PASS" in man["status"],
          "and reports the offline gate outcome")
    man_fail = J11.build_manifest(fit_bad, gate_bad, state_sha)
    check(man_fail["offline_gate_pass"] is False and "FAIL" in man_fail["status"],
          "a FAIL is reported as FAIL, not hidden")
    check(len(man["lineage"]["two_distinct_ancestors_do_not_conflate_them"]) == 2,
          "and the two ancestors are kept distinct")
    json.dumps(man, allow_nan=False)      # must be strictly serialisable
    check(True, "the manifest is strictly JSON")

    # ---------------------------------------------------------------- preflight is INERT -------
    real = {"open": builtins.open, "mkdir": Path.mkdir, "rename": os.rename,
            "write_text": Path.write_text, "savez": np.savez_compressed, "dump": pickle.dump}
    banned = []

    def deny(name):
        def f(*a, **k):
            banned.append(name)
            raise AssertionError(f"the preflight called {name}")
        return f
    Path.mkdir, os.rename = deny("mkdir"), deny("rename")
    Path.write_text, np.savez_compressed = deny("write_text"), deny("savez")
    pickle.dump = deny("pickle.dump")
    try:
        pre_res = J11.preflight()
    finally:
        Path.mkdir, os.rename = real["mkdir"], real["rename"]
        Path.write_text, np.savez_compressed = real["write_text"], real["savez"]
        pickle.dump = real["dump"]
    check(not banned, f"MEASURED: the preflight called no write primitive ({banned})")
    check(pre_res["read_only"] is True and pre_res["inert"]["fit_executed"] is False
          and pre_res["inert"]["optimizer_steps"] == 0
          and pre_res["inert"]["environment_constructed"] is False
          and pre_res["inert"]["leaf_created"] is False,
          "and reports itself inert on every axis")
    check(pre_res["inert"]["torch_imported_by_this_preflight"] is False,
          "MEASURED: the preflight did not import torch")
    check("torch" not in sys.modules,
          "MEASURED: torch is STILL absent from sys.modules after a full preflight")
    check(not J11.PREFLIGHT_SENTINEL.exists(), "and the sentinel was never created")
    # CONDITIONAL, so this suite does not become spent the moment J11 commits
    if not leaf_exists_at_start:
        check(pre_res["verdict"] == "GO" and pre_res["blockers"] == [],
              "with the leaf absent the preflight is GO")
    else:
        check(pre_res["verdict"] == "BLOCKED"
              and any("already exists" in b for b in pre_res["blockers"]),
              "with the leaf present the preflight is BLOCKED, which is the correct answer "
              "AFTER the single execution - this assertion does not expire")

    # the torch-free contract, statically
    top = {n.names[0].name.split(".")[0] for n in tree.body if isinstance(n, ast.Import)}
    top |= {(n.module or "").split(".")[0] for n in tree.body if isinstance(n, ast.ImportFrom)}
    check("torch" not in top, "no torch import at module scope")
    rf = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run_fit")
    torch_imports = [n for n in ast.walk(tree) if isinstance(n, ast.Import)
                     and any(al.name == "torch" for al in n.names)]
    check(len(torch_imports) == 1
          and rf.lineno < torch_imports[0].lineno < max(
              m.lineno for m in ast.walk(rf) if hasattr(m, "lineno")),
          "torch is imported exactly once, lazily, inside run_fit")
    check("warm_start" not in top and "asymmetric_rl_module" not in ids,
          "warm_start and asymmetric_rl_module are pinned and transcribed, never imported")
    pf = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "preflight")
    loads = [n for n in ast.walk(pf) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == "load"
             and isinstance(n.func.value, ast.Name) and n.func.value.id == "pickle"]
    check(not loads,
          "AST: the preflight never calls pickle.load directly - class_and_ctor_args.pkl needs "
          "torch to unpickle and is only ever hashed and copied")

    # ---------------------------------------------------------------- determinism, statically --
    dets = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr in ("use_deterministic_algorithms", "set_num_threads")]
    check(not dets,
          "AST: neither use_deterministic_algorithms nor set_num_threads is CALLED - July "
          "recorded neither and forcing them would be an invention")
    rngs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == "default_rng"]
    check(not rngs, "the runner creates no Generator of its own: the split helper owns the only "
                    "one, in the frozen J8 module")
    legacy = [n for n in ast.walk(rf) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "seed"]
    check(len(legacy) == 1, "np.random.seed is called exactly once, in run_fit")
    manual = next(n for n in ast.walk(rf) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute) and n.func.attr == "manual_seed")
    check(manual.lineno < legacy[0].lineno,
          "and torch.manual_seed precedes it, as in July's executive order")
    adam = next(n for n in ast.walk(rf) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute) and n.func.attr == "Adam")
    bs = next(n for n in ast.walk(rf) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Name) and n.func.id == "build_split")
    check(legacy[0].lineno < bs.lineno < adam.lineno,
          "the Generator is created after the legacy seed and before the optimizer")

    # ---- exactly the six direct tensors are optimised
    check(tuple(J11.DIRECT_KEYS) == ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight",
                                     "pi.0.2.bias", "pi.1.weight", "pi.1.bias"),
          "exactly the six direct mean-network tensors are trainable")
    check(len(J11.ALIAS_PAIRS) == 4 and len(J11.STATE_KEYS) == 10,
          "four aliases are rebuilt, for ten keys in total")
    adam_arg = adam.args[0] if adam.args else None
    check(isinstance(adam_arg, ast.Call) and getattr(adam_arg.func, "id", "") == "list",
          "the optimizer receives list(params.values()) - the six, and nothing else")
    check(all(k in J11.JULY_HP for k in ("seed", "epochs", "batch_size", "learning_rate",
                                         "validation_fraction", "patience", "clip_weight",
                                         "logstd_weight", "anchor_weight")),
          "every July hyperparameter is present")
    check((J11.JULY_HP["seed"], J11.JULY_HP["epochs"], J11.JULY_HP["batch_size"],
           J11.JULY_HP["learning_rate"], J11.JULY_HP["validation_fraction"],
           J11.JULY_HP["patience"], J11.JULY_HP["clip_weight"], J11.JULY_HP["logstd_weight"],
           J11.JULY_HP["anchor_weight"])
          == (123, 400, 128, 5e-05, 0.2, 60, 1.0, 0.0, 0.01),
          "and each holds its exact July value")
    check(J11.BEST_EPSILON == 1e-9, "the best-model rule uses val < best - 1e-9")

    # ---------------------------------------------------------------- destination guards -------
    expect(lambda: J11.validate_stage(None), J11.J11Error, "a missing stage token is refused")
    for bad in ("V26C_J8_RECOVERY_FIT", "V26C_J11_MULTISTART_FIT ", "", "v26c_j11_multistart_fit"):
        expect(lambda b=bad: J11.validate_stage(b), J11.J11Error,
               f"the stage token {bad!r} is refused")
    J11.validate_stage(J11.STAGE)
    check(True, "and the exact token is accepted")
    expect(lambda: J11.validate_out(None), J11.J11Error, "--out is required")
    expect(lambda: J11.validate_out("/tmp/somewhere-else"), J11.J11Error,
           "a different destination is refused")

    # ---------------------------------------------------------------- the write path -----------
    tmp = Path(tempfile.mkdtemp(prefix="v26c_j11_write_"))
    saved_root = J11.OUTPUT_ROOT_OVERRIDE
    try:
        J11.OUTPUT_ROOT_OVERRIDE = tmp
        target = tmp / "j11_runs" / "j11_multistart_fit_v26c_2026-08-27_r1"
        lock = target.parent / J11.LOCK_NAME
        fit, gate = synthetic_fit()

        # a REAL fit may never be written into a redirected root
        e = expect(lambda: J11.commit(str(target), J11.STAGE), J11.J11Error,
                   "a real fit into an overridden root is REFUSED")
        check("refuses to run a real fit into a redirected one" in str(e),
              f"and says so plainly: {e}")
        check(not target.exists(), "and created nothing")

        # a foreign lock is fail-closed and never removed
        target.parent.mkdir(parents=True, exist_ok=True)
        held = json.dumps({"stage": J11.STAGE, "pid": 999999})
        lock.write_text(held, encoding="utf-8")
        e = expect(lambda: J11.commit(str(target), J11.STAGE, fit=fit, gate=gate), J11.J11Error,
                   "a held lock is fail-closed")
        check("lock already exists" in str(e) and "does not own" in str(e),
              "the refusal names the lock and refuses to remove it")
        check(lock.read_text() == held, "MEASURED: the foreign lock was left untouched")
        check(J11.preflight()["verdict"] == "BLOCKED",
              "and the read-only preflight reports it as a blocker")
        lock.unlink()

        # cleanup on failure, with concurrent content that must survive
        sentinel = tmp / "j11_runs" / "concurrent_sentinel.txt"
        sentinel.write_text("written by another run", encoding="utf-8")

        def dump_then_fail(*a, **k):
            raise AssertionError("forced failure after the lock and staging exist")
        pickle.dump = dump_then_fail
        try:
            expect(lambda: J11.commit(str(target), J11.STAGE, fit=fit, gate=gate),
                   AssertionError, "a forced failure mid-staging")
        finally:
            pickle.dump = real["dump"]
        check(sentinel.is_file(), "PROVEN: the concurrent sentinel survived the cleanup")
        check(not (target.parent / J11.STAGING_NAME).exists(),
              "only the exact staging directory was removed")
        check(not lock.exists() and not target.exists(),
              "the lock was released and no leaf was created")

        # the real commit, with the injected fit
        res = J11.commit(str(target), J11.STAGE, fit=fit, gate=gate)
        check(res["verdict"] == "PASS" and res["gate_pass"] is True,
              "the commit succeeds")
        check(res["authoritative"] is False,
              "and declares itself NON-authoritative: the root was overridden")
        listing = sorted(p.name for p in target.iterdir())
        check(listing == ["commit_verification.json", "history.json", "rl_module",
                          "v26c_j11_aggregate_dataset.npz",
                          "v26c_j11_multistart_fit_receipt.json"],
              f"the leaf holds exactly the declared inventory ({listing})")
        mod = sorted(p.name for p in (target / "rl_module").iterdir())
        check(mod == ["actor_feature_manifest.json", "class_and_ctor_args.pkl",
                      "metadata.json", "module_state.pkl"],
              f"and the module directory holds the four expected files ({mod})")
        check(not (target / J11.TECHNICAL_INVALID_NAME).exists(),
              "the born-invalid marker is GONE after a passing verification")
        cv = json.loads((target / J11.COMMIT_VERIFICATION_NAME).read_text())
        check(cv["pass"] is True and not cv["paths_missing"] and not cv["hash_mismatches"]
              and cv["receipt_matches_staging_bytes"] is True,
              f"and the post-commit verification passed over {cv['files_checked']} files")
        rc = json.loads((target / J11.RECEIPT_NAME).read_text())
        check(len(rc["committed_files_sha256"]) == cv["files_checked"],
              "every committed file is hashed in the receipt and re-verified after the rename")
        for rel, sha in rc["committed_files_sha256"].items():
            check(not rel.startswith("/") and ".." not in rel.split("/")
                  and (target / rel).is_file() and J11._sha_file(target / rel) == sha,
                  f"MEASURED: {rel} is leaf-relative, resolves and reproduces its hash")
        check(J11.STAGING_NAME not in (target / J11.RECEIPT_NAME).read_text(),
              "no staging path appears anywhere in the receipt")

        # the sidecars: two copied byte-identically, the manifest regenerated
        for name in J11.BYTE_IDENTICAL_SIDECARS:
            check(J11._sha_file(target / "rl_module" / name) == J11.SIDECARS[name],
                  f"{name} is byte-identical to the parent's")
        wrote_man = target / "rl_module" / "actor_feature_manifest.json"
        check(J11._sha_file(wrote_man) != J11.SIDECARS["actor_feature_manifest.json"],
              "MEASURED: the manifest is NOT the parent's bytes")
        wm = json.loads(wrote_man.read_text())
        real_state_sha = J11._sha_file(target / "rl_module" / "module_state.pkl")
        check(wm["module_state_sha256"] == real_state_sha,
              "MEASURED: and it names the module_state.pkl actually written beside it")
        st = pickle.loads((target / "rl_module" / "module_state.pkl").read_bytes())
        check(wm["actor_digest"] == WS.actor_state_digest(st),
              "MEASURED: and its actor_digest matches the committed state under warm_start's "
              "own function - so the binding is enforceable, not decorative")
        check(sorted(st) == sorted(J11.STATE_KEYS) and len(st) == 10,
              "all ten actor keys are emitted: dropping the aliases would break "
              "warm_start.transplant_actor_state")
        check(all(np.asarray(v).dtype == np.float32 for v in st.values()),
              "every committed tensor is float32")

        # the aggregate NPZ is auditable and reproduces the receipt
        za = np.load(target / J11.AGGREGATE_NAME, allow_pickle=False)
        check(sorted(za.files) == ["actions", "actor_feature_names", "observations"],
              "the aggregate NPZ holds the three auditable keys")
        check(za["observations"].shape == (24713, 35) and za["actions"].shape == (24713, 2),
              "with the full 24713 rows")
        check(J11._sha_array(za["observations"])
              == rc["aggregate"]["content_hashes"]["observations"],
              "MEASURED: and reproduces the content hash the receipt records")
        check([str(n) for n in za["actor_feature_names"]] == list(names),
              "and carries the 35 runtime names")

        # a second commit is refused: no-clobber, single-execution
        e = expect(lambda: J11.commit(str(target), J11.STAGE, fit=fit, gate=gate), J11.J11Error,
                   "a second commit onto the same leaf is REFUSED")
        check("already exists" in str(e), "as a no-clobber, single-execution stage")

        # ---- the post-commit verifier actually fails on tampering
        victim = target / "rl_module" / "module_state.pkl"
        keep = victim.read_bytes()
        victim.write_bytes(keep + b"tampered")
        v = J11.verify_committed_leaf(target)
        check(v["pass"] is False and any(m["path"].endswith("module_state.pkl")
                                         for m in v["hash_mismatches"]),
              "MEASURED: a tampered module_state is caught by the post-commit verifier")
        victim.write_bytes(keep)
        check(J11.verify_committed_leaf(target)["pass"] is True, "and restoring it passes again")
        victim.unlink()
        v = J11.verify_committed_leaf(target)
        check(v["pass"] is False and any(p.endswith("module_state.pkl")
                                         for p in v["paths_missing"]),
              "MEASURED: a deleted file is caught as a path that does not resolve")
        victim.write_bytes(keep)
        v = J11.verify_committed_leaf(target, expected_receipt_sha="0" * 64)
        check(v["pass"] is False and v["receipt_matches_staging_bytes"] is False,
              "MEASURED: a receipt whose bytes differ from the staged bytes FAILS - a corrupted "
              "receipt cannot validate itself")
        for bad in ("/etc/passwd", "../outside.pkl", "", "a/../../b"):
            expect(lambda b=bad: J11._resolve_inside(target, b), J11.J11Error,
                   f"a recorded path {bad!r} is refused as not leaf-relative")

        # ---- a FAILED verification must never leave an apparently-valid leaf
        shutil.rmtree(target)
        real_verify = J11.verify_committed_leaf
        J11.verify_committed_leaf = lambda leaf, **k: {"pass": False, "paths_missing": ["x"],
                                                       "hash_mismatches": []}
        try:
            expect(lambda: J11.commit(str(target), J11.STAGE, fit=fit, gate=gate), J11.J11Error,
                   "a FAILED post-commit verification RAISES")
        finally:
            J11.verify_committed_leaf = real_verify
        check(target.is_dir() and (target / J11.RECEIPT_NAME).is_file(),
              "MEASURED: the leaf is PRESERVED - a corrupted commit is evidence about the commit")
        check((target / J11.TECHNICAL_INVALID_NAME).is_file(),
              "MEASURED: and is marked TECHNICAL_INVALID, so it never reads as valid")
        check(json.loads((target / J11.COMMIT_VERIFICATION_NAME).read_text())["pass"] is False,
              "with commit_verification.json declaring pass false")
        check(not lock.exists() and not (target.parent / J11.STAGING_NAME).exists(),
              "and the lock and staging are still released")

        # ---- static: the marker is born with the leaf
        cf = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "commit")
        nodes = list(ast.walk(cf))
        rename_line = next(n.lineno for n in nodes if isinstance(n, ast.Call)
                           and isinstance(n.func, ast.Attribute) and n.func.attr == "rename")
        marker_writes = [n.lineno for n in nodes if isinstance(n, ast.Call)
                         and isinstance(n.func, ast.Attribute) and n.func.attr == "write_text"
                         and any(isinstance(d, ast.Name) and d.id == "TECHNICAL_INVALID_NAME"
                                 for d in ast.walk(n.func.value))]
        check(marker_writes and min(marker_writes) < rename_line,
              f"STATIC: the marker is written into STAGING at line {min(marker_writes)}, BEFORE "
              f"the rename at line {rename_line}")
        unlinks = [n.lineno for n in nodes if isinstance(n, ast.Call)
                   and isinstance(n.func, ast.Attribute) and n.func.attr == "unlink"
                   and isinstance(n.func.value, ast.Name) and n.func.value.id == "marker"]
        check(unlinks and min(unlinks) > rename_line,
              "and removed only AFTER the commit, as the last write of a passing run")
        vc = next(n for n in nodes if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                  and n.func.id == "verify_committed_leaf")
        wrapping = [n for n in nodes if isinstance(n, ast.Try)
                    and any(c is vc for b in n.body for c in ast.walk(b))]
        inner = max(wrapping, key=lambda n: n.lineno)
        check({h.type.id for h in inner.handlers if isinstance(h.type, ast.Name)} == {"Exception"},
              "STATIC: the verification catches Exception, never BaseException - a Ctrl-C "
              "propagates instead of being recorded as a verdict")
        check(src.count("shutil.rmtree") == 1, "exactly ONE recursive removal")
        rm = [n for n in nodes if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute) and n.func.attr == "rmtree"]
        check(len(rm) == 1 and isinstance(rm[0].args[0], ast.Name)
              and rm[0].args[0].id == "staging_created",
              "and its only argument is staging_created")
        check("O_CREAT | os.O_EXCL" in src, "the lock uses O_CREAT|O_EXCL")
    finally:
        J11.OUTPUT_ROOT_OVERRIDE = saved_root
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------------------------------------------------------- nothing disturbed --------
    check(not (HERE / "j11_runs").exists() if not leaf_exists_at_start else True,
          "MEASURED: this suite never created the authorised j11_runs")
    check(snapshot(HERE) == before_all,
          "MEASURED: not one byte under the validation root changed across the WHOLE suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


def _with(arr, idx, value):
    out = np.array(arr, copy=True)
    out[idx] = np.float32(value)
    return out


if __name__ == "__main__":
    raise SystemExit(main())
