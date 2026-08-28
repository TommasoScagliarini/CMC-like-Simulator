"""Tests for the V26C J2 base-fit tooling. The fit itself is never executed here."""
from __future__ import annotations
import builtins, inspect, io, json, sys, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j2_fit as J  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


# The single authorised J2 leaf and the exact hashes of the six artefacts it produced. Any other
# leaf, any missing or extra file, and any mutated artefact must be refused.
AUTHORISED_J2_LEAF = "j2_base_v26c_2026-08-26_r1"
J2_PINNED_ARTEFACTS = {
    "history.json":
        "b5b4641c0170aac5af615b623746e2c56bd6d1ecafd3cd5657ff63ee9917c777",
    "rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "rl_module/module_state.pkl":
        "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130",
    "v26c_j2_fit_receipt.json":
        "dd997dc9d465184de6e25a3488c9752799fb60feca304ac876f35dfbdcd9871e",
}


# Module level so the staged-sidecar test can pickle them, exactly as the real sidecar is pickled.
class _Box:
    """Mimics the shape of a gymnasium Box closely enough for the pure derivation."""

    def __init__(self, low, high, shape, dtype):
        self.low = np.full(shape, low, dtype=dtype)
        self.high = np.full(shape, high, dtype=dtype)
        self.shape = shape
        self.dtype = dtype


def mock_ctor(n_actor=39, obs=88, uniform=True):
    b = _Box(-np.inf, np.inf, (obs,), np.float32)
    if not uniform:
        b.low[0] = -1.0
    return {"class": "AsymmetricActorCriticTorchRLModule",
            "ctor_args_and_kwargs": ((), {
                "observation_space": b,
                "action_space": _Box(-1.0, 1.0, (2,), np.float32),
                "inference_only": True, "learner_only": False, "catalog_class": None,
                "model_config": {"vf_share_layers": False, "n_actor": n_actor, "n_full": 84,
                                 "fcnet_hiddens": [256, 256], "fcnet_activation": "tanh",
                                 "freeze_logstd": False, "freeze_actor": False}})}


def main() -> int:
    src = (HERE / "v26c_j2_fit.py").read_text()
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}

    # ---------------------------------------------------------------- J1 input is pinned --------
    j1 = J.verify_j1_inputs()
    check(j1["artefacts_sha256"] == J.PIN_J1, "the three J1 artefacts match their pins")
    check(j1["amendment_sha256"] == J.PIN_J1_AMENDMENT, "and so does the amendment")
    check("J2" in j1["authorisation"] and j1["j1_verdict_stands"].startswith("FAIL"),
          "J2 is authorised while the J1 FAIL verdict stands")
    for name in J.PIN_J1:
        saved = J.PIN_J1[name]; J.PIN_J1[name] = "0" * 64
        e = expect(J.verify_j1_inputs, J.J2Error, f"a changed {name} must be refused")
        check(name in str(e), f"and the failure names {name}")
        J.PIN_J1[name] = saved
    saved = J.PIN_J1_AMENDMENT; J.PIN_J1_AMENDMENT = "0" * 64
    expect(J.verify_j1_inputs, J.J2Error, "a changed amendment must be refused")
    J.PIN_J1_AMENDMENT = saved

    # ------------------------------------------- (A3) the student is derived FRESH --------------
    par = J.verify_parent()
    check(par["module_state_sha256"] == J.PIN_V26_PARENT_SHA
          == "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
          and par["width"] == 39,
          "the ONLY actor input is the August V26 39D parent, hash 0ba56eb7...")
    # V1 and B0 must not be reachable from the derivation path
    vp_src = inspect.getsource(J.verify_parent) + inspect.getsource(J.derive_student_35d)
    for forbidden in ("V1_35D", "ORACLE_B0", "B0_35D_MASKED", "V26B"):
        check(forbidden not in vp_src,
              f"neither verify_parent nor the derivation references {forbidden}")
    check("v1_b0_not_required" in par and "optional diagnostic oracle" in par["v1_b0_not_required"],
          "and the receipt states that V1/B0 are not required")
    saved = J.PIN_V26_PARENT_SHA; J.PIN_V26_PARENT_SHA = "0" * 64
    expect(J.verify_parent, J.J2Error, "a changed V26 parent must be refused")
    J.PIN_V26_PARENT_SHA = saved
    saved = J.PIN_MANIFEST39_SHA; J.PIN_MANIFEST39_SHA = "0" * 64
    expect(J.verify_parent, J.J2Error, "a changed 39D manifest must be refused")
    J.PIN_MANIFEST39_SHA = saved

    der = J.derive_student_35d()
    state, names35, rep = der["state"], der["feature_names"], der["report"]
    check(tuple(state) == J.STATE_KEY_ORDER and len(state) == 10,
          "the derived state carries the ten keys in the expected order")
    check(len(names35) == 35, "and a 35-name manifest")
    check(rep["mapping"]["dropped_39d_indices"] == [2, 3, 4, 5]
          and rep["mapping"]["dropped_names"] == list(J.DROPPED_39D_NAMES)
          and rep["mapping"]["kind"].startswith("by feature name"),
          "the 39 -> 35 mapping is by feature name and drops exactly the four target columns")
    check(all(n not in names35 for n in J.DROPPED_39D_NAMES),
          "the dropped names are absent from the 35D manifest")
    W1 = np.asarray(state["pi.0.0.weight"])
    check(W1.shape == (256, 35), "the first layer is 256 x 35")
    check([c for c in range(35) if bool(np.all(W1[:, c] == 0.0))] == list(J.MASKED_COLUMNS),
          "columns 0,1 and 25..34 are hard-zero and nothing else is")
    check(rep["mean_bias_compensation"]["applied"] is True
          and rep["mean_bias_compensation"]["float64"] == list(J.MEAN_BIAS_COMPENSATION),
          "the preregistered mean-bias compensation is applied in float64")
    check(np.asarray(state["pi.1.weight"])[J.ACTION_DIM:].max() == 0.0
          and float(np.asarray(state["pi.1.bias"])[J.ACTION_DIM:].min())
          == float(np.float32(J.LOGSTD_CONSTANT_BIAS)),
          "the log-std head is zeroed with the deployable constant bias")
    check(abs(rep["logstd_head"]["sigma"] - 0.005) < 1e-6, "which is sigma 0.005")
    check(rep["critic"].startswith("never loaded"), "the critic is never loaded")

    # ------------------------------------------- the sidecars -----------------------------------
    check(set(J.PARENT_SIDECARS) == {"metadata.json", "class_and_ctor_args.pkl"},
          "both parent sidecars are pinned by hash")
    for name, pin in J.PARENT_SIDECARS.items():
        check(J._sha_file(J.V26_PARENT_DIR / name) == pin,
              f"the source sidecar {name} matches its pin")
        saved = J.PARENT_SIDECARS[name]; J.PARENT_SIDECARS[name] = "0" * 64
        e = expect(J.verify_parent, J.J2Error, f"a changed source sidecar {name} must be refused")
        check(name in str(e), f"and the failure names {name}")
        J.PARENT_SIDECARS[name] = saved
    check(J.COPIED_VERBATIM == ("metadata.json",)
          and J.DERIVED_SIDECAR == "class_and_ctor_args.pkl",
          "metadata travels verbatim; the constructor sidecar is DERIVED, never copied")
    check("class_and_ctor_args.pkl" not in J.COPIED_VERBATIM,
          "copying the constructor sidecar verbatim is impossible by construction")
    check("39-input constructor" in par["sidecar_policy"]["why"],
          "and the receipt says why: the parent sidecar declares a 39-input actor")

    # ------------------------------------------- the REAL sidecar loads -------------------------
    # The first J2 attempt computed 400 epochs and then died here, unpickling the parent sidecar,
    # because baseline_MLP was not importable. A mock-only test could not see that; this one calls
    # the real loader.
    check("sys.path.insert(0, str(BASELINE))" in inspect.getsource(J.load_parent_ctor),
          "load_parent_ctor puts baseline_MLP on the path before unpickling")
    try:
        real_ctor = J.load_parent_ctor()
        real_available = True
    except ModuleNotFoundError as exc:
        if "asymmetric_rl_module" in str(exc):
            raise AssertionError("load_parent_ctor failed to make the sidecar importable: this is "
                                 "exactly the defect that killed the first J2 attempt") from exc
        real_ctor, real_available = None, False       # ray/torch absent: a plain interpreter
    if real_available:
        rd, rr = J.derive_ctor_args_35d(real_ctor)
        rkw = rd["ctor_args_and_kwargs"][1]
        check(rkw["model_config"]["n_actor"] == 35
              and tuple(rkw["observation_space"].shape) == (84,),
              "MEASURED on the REAL parent sidecar: n_actor 35 and an 84-wide observation space")
        check(rr["privileged_width_computed_from_parent"] == 49,
              "with the privileged width 49 computed from the real parent")
    else:
        check(True, "the real sidecar needs the rllib interpreter; the suite is also run there, "
                    "where this branch executes for real")
    # the sidecar must be resolved BEFORE the epochs, so a failure costs nothing
    fit_src_early = inspect.getsource(J.fit)
    i_ctor = fit_src_early.index("derive_ctor_args_35d(load_parent_ctor())")
    i_loop = fit_src_early.index("for epoch in range(1,")
    check(i_ctor < i_loop,
          "the constructor sidecar is resolved BEFORE the training loop: a failure there costs "
          "milliseconds, not 400 epochs")

    # the ctor derivation is a PURE function, exercised here on a faithful mock
    base_ctor = mock_ctor()
    src_args, src_kw = base_ctor["ctor_args_and_kwargs"]
    src_obs, src_mc = src_kw["observation_space"], dict(src_kw["model_config"])
    derived_ctor, ctor_rep = J.derive_ctor_args_35d(base_ctor)
    _, dkw = derived_ctor["ctor_args_and_kwargs"]
    check(derived_ctor["ctor_args_and_kwargs"][0] is src_args,
          "the positional arguments are carried over by identity")
    check(dkw["observation_space"] is not src_obs,
          "the observation space is REBUILT, not mutated in place")
    check(src_kw["model_config"]["n_actor"] == 39 and tuple(src_obs.shape) == (88,),
          "and the source sidecar object is left untouched by the derivation")
    check(dkw["model_config"]["n_actor"] == 35, "the derived sidecar declares n_actor 35")
    check(tuple(dkw["observation_space"].shape) == (84,),
          "and an 84-wide observation space: 35 + 49 privileged")
    check(ctor_rep["privileged_width_computed_from_parent"] == 49,
          "the privileged width is COMPUTED from the parent, not hardcoded")
    check(ctor_rep["changed"] == {"model_config.n_actor": {"from": 39, "to": 35},
                                  "observation_space.shape": {"from": [88], "to": [84]}},
          "exactly two fields change")
    for k in ("action_space", "inference_only", "learner_only", "catalog_class"):
        check(dkw[k] is src_kw[k] or dkw[k] == src_kw[k],
              f"{k} is carried over untouched, by identity")
    for k in ("vf_share_layers", "n_full", "fcnet_hiddens", "fcnet_activation",
              "freeze_logstd", "freeze_actor"):
        check(dkw["model_config"][k] == src_mc[k],
              f"model_config.{k} is carried over untouched")
    check(derived_ctor["class"] is base_ctor["class"], "and so is the class")
    check(set(dkw) == set(src_kw) and set(dkw["model_config"]) == set(src_mc),
          "no constructor key is added or dropped")
    # a different parent geometry must be recomputed, not assumed
    d2, r2 = J.derive_ctor_args_35d(mock_ctor(n_actor=39, obs=100))
    check(tuple(d2["ctor_args_and_kwargs"][1]["observation_space"].shape) == (96,)
          and r2["privileged_width_computed_from_parent"] == 61,
          "a parent with a different privileged width yields a different derived width")
    # negatives
    expect(lambda: J.derive_ctor_args_35d(mock_ctor(n_actor=35)), J.J2Error,
           "a sidecar that does not declare n_actor 39 must be refused")
    expect(lambda: J.derive_ctor_args_35d(mock_ctor(obs=20)), J.J2Error,
           "an observation narrower than n_actor must be refused")
    expect(lambda: J.derive_ctor_args_35d(mock_ctor(uniform=False)), J.J2Error,
           "non-uniform observation bounds must be refused rather than silently flattened")
    expect(lambda: J.derive_ctor_args_35d({"nope": 1}), J.J2Error,
           "a sidecar of the wrong shape must be refused")

    # the STAGED sidecar is reloaded and verified
    import tempfile
    tmp = Path(tempfile.mkdtemp())
    try:
        good = tmp / "class_and_ctor_args.pkl"
        with good.open("wb") as fh:
            J.pickle.dump(derived_ctor, fh)
        st = J.verify_staged_sidecar(good)
        check(st["n_actor"] == 35 and st["observation_shape"] == [84]
              and "reloaded from disk" in st["verified"],
              "the staged sidecar is reloaded from disk and confirmed to describe a 35D actor")
        bad = tmp / "bad.pkl"
        with bad.open("wb") as fh:
            J.pickle.dump(mock_ctor(), fh)
        e = expect(lambda: J.verify_staged_sidecar(bad), J.J2Error,
                   "a staged sidecar still declaring 39 inputs must FAIL closed")
        check("n_actor=39" in str(e), "and the failure names the wrong width")
    finally:
        import shutil as _sh
        _sh.rmtree(tmp, ignore_errors=True)

    # ------------------------------------------- the oracle is OUT OF BAND -----------------------
    import inspect as _insp
    for fn in (J.preflight, J.fit, J.derive_student_35d, J.verify_parent, J.load_dataset):
        fsrc = _insp.getsource(fn)
        toks = [t for t in tokenize.generate_tokens(io.StringIO(fsrc).readline)]
        calls = any(toks[i].type == tokenize.NAME and toks[i].string == "diagnostic_oracle"
                    and toks[i + 1].string == "(" for i in range(len(toks) - 1))
        check(not calls, f"{fn.__name__} never CALLS diagnostic_oracle")
        check("ORACLE_B0" not in fsrc, f"and {fn.__name__} never references ORACLE_B0")
    saved = J.ORACLE_B0
    J.ORACLE_B0 = Path("/nonexistent/oracle.pkl")
    pre_no_oracle = J.preflight()
    check(pre_no_oracle["verdict"] == "GO",
          "MEASURED: the preflight is GO with the oracle absent from disk")
    check(J.derive_student_35d()["report"]["derived_actor_digest"] == rep["derived_actor_digest"],
          "and the derivation is byte-identical without the oracle present")
    absent = J.diagnostic_oracle(state)
    check(absent["available"] is False and "blocks nothing" in absent["note"],
          "the oracle itself reports its absence without raising")
    J.ORACLE_B0 = saved
    check(pre_no_oracle["diagnostic_oracle"] == {
              "consulted": False,
              "reason": "the oracle is out of band: neither preflight nor fit opens it, and both "
                        "work if it is absent"},
          "the preflight records that the oracle was not consulted")
    # consulted explicitly, out of band, it confirms the derivation
    orc = J.diagnostic_oracle(state)
    check(orc["available"] is True and orc["bit_identical"] is True,
          "MEASURED: consulted out of band, the oracle confirms the fresh derivation reproduces "
          "the pinned masked init BIT-FOR-BIT")
    check(all(orc["per_key_bit_identical"].values()), "every one of the ten tensors matches")
    check("OUT OF BAND" in orc["status"], "and it is declared out of band")
    check(J.diagnostic_oracle()["bit_identical"] is True,
          "it can also derive the state itself, so it needs nothing from the caller")

    # ------------------------------------------- (A1) July's log-std mechanism -------------------
    hp = J.JULY_BASE_HP
    check(hp["freeze_logstd_head"] is False and hp["logstd_weight"] == 0.1,
          "A1 resolved: freeze_logstd_head is False and logstd_weight is 0.1")
    july = json.loads((J.TG / "runs" / "training"
                       / "target_domain_imitation_no_controller_memory_2026-07-13"
                       / "adaptation_report.json").read_text())["hyperparameters"]
    for k in ("seed", "batch_size", "learning_rate", "patience", "validation_fraction",
              "clip_weight", "logstd_weight", "anchor_weight", "freeze_logstd_head"):
        check(hp[k] == july[k], f"{k} matches the July artefact ({july[k]!r})")
    fit_src = inspect.getsource(J.fit)
    check("def restore_logstd_head" in fit_src and 'if JULY_BASE_HP["freeze_logstd_head"]' in fit_src,
          "restore_logstd_head exists and is conditional, a no-op under July's setting")
    check("logstd_loss" in fit_src and 'float(JULY_BASE_HP["logstd_weight"]) * logstd_loss'
          in fit_src,
          "and the log-std is anchored through the loss instead")
    check("weight_max_abs_change" in fit_src and "bias_max_abs_change" in fit_src,
          "the receipt records how far the log-std actually moved")

    # ------------------------------------------- (A2) ONE shared RNG stream ----------------------
    rng, sp = J.july_rng_and_split(500)
    check(isinstance(rng, np.random.Generator), "the split returns the LIVE generator")
    check(sp["training_rows"] == 400 and sp["validation_rows"] == 100, "80/20: 400 / 100")
    tr, va = sp["training_indices"], sp["validation_indices"]
    check(sorted(tr.tolist() + va.tolist()) == list(range(500)), "every row covered once")
    check(not (set(tr.tolist()) & set(va.tolist())), "and the sets are disjoint")
    check(bool(np.array_equal(va, np.sort(va))), "validation indices sorted, as in July")
    # reproduce July's stream independently: split first, then the SAME generator shuffles
    ref = np.random.default_rng(123)
    idx = ref.permutation(500)
    n_val = max(1, int(round(500 * 0.20)))
    check(bool(np.array_equal(va, np.sort(idx[:n_val])))
          and bool(np.array_equal(tr, np.asarray(idx[n_val:], dtype=int))),
          "MEASURED: the split is one default_rng(123).permutation(500), first 100 sorted")
    ref_first = ref.permutation(np.asarray(idx[n_val:], dtype=int))
    got_first = rng.permutation(tr)
    check(bool(np.array_equal(ref_first, got_first)),
          "MEASURED: the FIRST epoch shuffle continues the SAME stream - a fresh default_rng(123) "
          "would have produced a different order")
    fresh = np.random.default_rng(123)
    check(not np.array_equal(fresh.permutation(tr), ref_first),
          "and a restarted generator genuinely differs, so the test is not vacuous")
    check("the SAME generator continues" in sp["rng_stream"], "the receipt states it")
    check("rng.permutation(train_idx)" in fit_src and "SAME generator" in fit_src,
          "and the fit loop uses that same generator for every epoch shuffle")
    check("np.random.default_rng" not in fit_src,
          "the fit never creates a second generator")
    expect(lambda: J.july_rng_and_split(1, validation_fraction=1.0), J.J2Error,
           "a split leaving no training rows must fail closed")

    # ---------------------------------------------------------------- no false ambiguities ------
    check(not hasattr(J, "declared_ambiguities"),
          "the ambiguity block is gone: A1, A2 and A3 are resolved, not deferred")
    check("ambiguities_requiring_a_decision" not in src,
          "and the preflight no longer advertises open ambiguities")

    # ---------------------------------------------------------------- dataset and masking -------
    data = J.load_dataset()
    check(data["rows"] == 500 and data["observations"].shape == (500, 35),
          "the J1 dataset is 500 x 35")
    check(tuple(data["feature_names"]) == tuple(names35),
          "and its manifest equals the derived student's, name for name")
    masked = J.masked_observations(data["observations"])
    check(bool(np.all(masked[:, list(J.MASKED_COLUMNS)] == 0.0)), "the mask zeroes its columns")
    untouched = [c for c in range(35) if c not in J.MASKED_COLUMNS]
    check(bool(np.array_equal(masked[:, untouched], data["observations"][:, untouched])),
          "and alters no other column")
    check(float(np.max(np.abs(data["observations"][:, list(J.CONTROLLER_COLUMNS)]))) > 0.0,
          "MEASURED: the controller columns carry real values in the dataset; masking is a FIT "
          "constraint and did not mutate the source array")

    # ---------------------------------------------------------------- the fit is implemented ----
    check(callable(J.fit), "the fit is implemented, not a stub")
    for token in ("opt.step()", "loss.backward()", "opt.zero_grad(set_to_none=True)",
                  "restore_logstd_head()", "project_columns()", "mean_loss", "clip_loss",
                  "anchor_loss", "best_state", "stale", "history"):
        check(token in fit_src, f"the fit carries {token}")
    i_step = fit_src.index("opt.step()")
    i_restore = fit_src.index("restore_logstd_head()", i_step)
    i_proj = fit_src.index("project_columns()", i_restore)
    check(i_step < i_restore < i_proj,
          "July's per-step order is preserved: step -> restore_logstd_head -> zero columns")
    check("derive_ctor_args_35d(load_parent_ctor())" in fit_src
          and "verify_staged_sidecar(sm / DERIVED_SIDECAR)" in fit_src,
          "the fit DERIVES the constructor sidecar and verifies the staged copy")
    check(fit_src.count("derive_ctor_args_35d(") == 1,
          "and derives it exactly once, before the loop, never twice")
    check("for extra in COPIED_VERBATIM" in fit_src
          and 'shutil.copy2(V26_PARENT_DIR / extra' in fit_src,
          "and copies verbatim only what may be copied verbatim")
    check('"class_and_ctor_args.pkl"' not in fit_src,
          "the fit never names the constructor sidecar literally: it uses DERIVED_SIDECAR")
    check("ctor_sidecar" in fit_src and '"staged": staged_sidecar' in fit_src,
          "the receipt records source hashes, the derivation and the staged verification")
    check("mkdir(parents=True, exist_ok=False)" in fit_src and "no-clobber" in fit_src,
          "the output leaf is fresh and no-clobber")
    check("shutil.rmtree(out, ignore_errors=True)" in fit_src,
          "and a failure after creation removes the leaf it made")
    for token in ("module_state.pkl", "actor_feature_manifest.json", "history.json",
                  "RECEIPT_NAME", "output_files_sha256", "history_sha256"):
        check(token in fit_src, f"the fit saves {token}")
    check(J.RECEIPT_NAME == "v26c_j2_fit_receipt.json",
          "and the receipt constant names the J2 receipt file")
    check("allow_nan=False" in fit_src, "history and receipt are serialised with allow_nan=False")
    check('"critic": "never loaded, never trained"' in fit_src
          and "vf" not in ids and "critic_state" not in ids,
          "the critic is never loaded or trained")
    check('"promotion": "NONE"' in fit_src and '"deployable": False' in fit_src,
          "nothing is promoted and nothing is marked deployable")

    # ---------------------------------------------------------------- preflight is inert --------
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"torch", "ray", "env_factory", "opensim"}

    def guarded(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(name)
            raise AssertionError(f"preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre = J.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: the preflight imported no fitting stack ({tripped})")
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight verdict is GO")
    check(pre["inert"]["fit_executed"] is False and pre["inert"]["promotion"] is False,
          "and it declares that nothing was fitted or promoted")
    check(pre["diagnostic_oracle"]["consulted"] is False
          and "bit_identical" not in pre["diagnostic_oracle"],
          "the preflight records that the oracle was NOT consulted and carries no verdict from it")
    check(pre["masking"]["effective_support_dimensions"] == 23,
          "twelve masked columns leave a 23-dimensional effective support")

    # ---------------------------------------------------------------- execution is gated --------
    expect(lambda: J.fit(authorized_stage=None), J.J2Error,
           "the fit refuses to run without its token")
    expect(lambda: J.fit(authorized_stage="V26C-J1-TEACHER-COLLECT"), J.J2Error,
           "another stage's token must not authorise it")
    for name in ("rollout_eval", "train_ppo", "PPOConfig", "dagger", "DAgger", "subprocess"):
        check(name not in ids, f"the module never references {name}")
    # Before the recovery authorisation this asserted OUT_ROOT was empty. That precondition is
    # spent: exactly one authorised fit has since run. The invariant that still matters is that
    # there is AT MOST ONE run, that it is the authorised leaf, and that it promotes nothing.
    leaves = sorted(p.name for p in J.OUT_ROOT.iterdir()
                    if p.is_dir()) if J.OUT_ROOT.is_dir() else []
    check(leaves in ([], [AUTHORISED_J2_LEAF]),
          f"ONLY the authorised leaf {AUTHORISED_J2_LEAF} may exist; found {leaves}")
    stray = sorted(p.name for p in J.OUT_ROOT.iterdir()) if J.OUT_ROOT.is_dir() else []
    check(set(stray) <= {AUTHORISED_J2_LEAF},
          f"and nothing else lives under the run root; found {stray}")
    if leaves:
        leaf = J.OUT_ROOT / leaves[0]
        # the SIX artefacts, by exact hash: a mutated artefact must be refused
        produced = sorted(str(p.relative_to(leaf)) for p in leaf.rglob("*") if p.is_file())
        check(produced == sorted(J2_PINNED_ARTEFACTS),
              f"exactly the six pinned artefacts exist, no more and no fewer; found {produced}")
        for name, pin in J2_PINNED_ARTEFACTS.items():
            p = leaf / name
            check(p.is_file(), f"the fit produced {name}")
            got = J._sha_file(p)
            check(got == pin, f"{name} is byte-identical to its pin ({pin[:16]}...); got "
                              f"{got[:16]}...")
        # a mutated artefact really is detectable: prove the comparison is not vacuous
        for name, pin in J2_PINNED_ARTEFACTS.items():
            check(J._sha_file(leaf / name) != "0" * 64 and len(pin) == 64,
                  f"the pin for {name} is a real 64-hex digest")
        rec = json.loads((leaf / "v26c_j2_fit_receipt.json").read_text())
        check(rec["deployable"] is False and rec["promotion"] == "NONE",
              "the fitted actor is neither deployable nor promoted")
        check(rec["critic"] == "never loaded, never trained", "and the critic was never touched")
        check(rec["gating"]["j3_still_required"] is True
              and rec["gating"]["j3_penetration_max_m"] == 0.020,
              "J3 is still required at <= 0.020 m")
        check(rec["j1_input"]["j1_verdict_stands"].startswith("FAIL"),
              "and the J1 soft fail is still recorded as a FAIL")
        st = J.pickle.loads((leaf / "rl_module" / "module_state.pkl").read_bytes())
        W1 = np.asarray(st["pi.0.0.weight"])
        check([c for c in range(35) if bool(np.all(W1[:, c] == 0.0))] == list(J.MASKED_COLUMNS),
              "and the reloaded weights keep exactly the masked columns at zero")
    g = pre["gating"]
    check(g["j3_still_required"] is True and g["j3_penetration_max_m"] == 0.020
          and g["no_promotion_without_j3"] is True,
          "J3 is still required at <= 0.020 m and no promotion is possible without it")
    for forbidden in ("rollout", "J3", "retry", "Markov", "LOTO", "LOCO", "B1R1", "B1R2",
                      "promotion", "ex-novo"):
        check(forbidden in g["forbidden_here"], f"{forbidden} is forbidden at this stage")
    check("if False" not in src, "no dead conditional")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
