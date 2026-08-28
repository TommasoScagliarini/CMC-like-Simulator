"""Fail-closed tests for the V26C J7 Markov recovery dataset tooling.

Read-only and in memory: no dataset is written, nothing is fitted, no environment is constructed
and no rollout is run. Every pinned artefact is read but never modified.
"""
from __future__ import annotations
import ast, builtins, io, json, sys, tokenize
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_j7_markov_dataset as J7  # noqa: E402
import v26c_penetration_contract as PC  # noqa: E402

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
    src = (HERE / "v26c_j7_markov_dataset.py").read_text()
    tree = ast.parse(src)
    ids = {t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
           if t.type == tokenize.NAME}
    prereg = json.loads(J7.PREREG.read_text())
    pinned_before = {rel: J7._sha_file(HERE / rel)
                     for rel in prereg["pinned_artefacts_sha256"]}

    # ---------------------------------------------------------------- the preregistration ------
    check(prereg["kind"] == "ADDITIVE, IMMUTABLE, PREREGISTRATION"
          and prereg["stage_authorised"] == "V26C-J7-MARKOV-DATASET",
          "the prereg is additive, immutable and names the J7 stage")
    check(J7._sha_file(J7.PREREG) == J7.PIN_PREREG, "and is pinned by exact hash")
    check(len(prereg["pinned_artefacts_sha256"]) == 26
          and all(J7._sha_file(HERE / r) == h
                  for r, h in prereg["pinned_artefacts_sha256"].items()),
          "all 26 pinned artefacts verify on disk")
    # the penetration authority is pinned in BOTH halves: thresholds and the code applying them
    pins = prereg["pinned_artefacts_sha256"]
    check(pins.get("v26c_penetration_contract_2026-08-26.json")
          == "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461",
          "the contract JSON is pinned")
    check(pins.get("v26c_penetration_contract.py")
          == "9257e9b8cdf54d9a59bfe2ee25526b283408d325a900156a69d64dbf196298dc"
          == J7._sha_file(J7.PENETRATION_EVALUATOR),
          "and so is the evaluator MODULE - the JSON alone would not bind the implementation")
    check(Path(PC.__file__).resolve() == J7.PENETRATION_EVALUATOR.resolve(),
          "the imported evaluator IS the pinned file, not a namesake on sys.path")
    # the three penetration series are pinned DIRECTLY: the receipt does not substitute for it
    for seed, sha in ((123, "0250c2c82afca4406b08fb8f69174c15da41d60b99d63c400ce442da32939b01"),
                      (124, "19dff986e90aae27622e23904316091e4eb85bc9b0754a61696479ca0fc4c87e"),
                      (125, "5276af956f0003820fc9a2f3e55e0946b38e7f9b9023fd300731ad151de9f89c")):
        rel = f"j6_runs/j6_recovery_probe_v26c_2026-08-26_seed{seed}/j6_penetration.npz"
        check(pins.get(rel) == sha == J7.PIN_J6_PENETRATION[seed]
              == J7._sha_file(HERE / rel),
              f"seed {seed}'s j6_penetration.npz is pinned directly and verifies on disk")
    # the aggregate ratio is EXACT, not the nominal-denominator value
    exact = 713 / 16713
    check(prereg["dataset_specification"]["ratios"]["recovery_over_aggregate"] == exact
          == 0.04266140130437384,
          "recovery_over_aggregate is exactly 713/16713 = 0.04266140130437384")
    check(prereg["dataset_specification"]["ratios"]["recovery_over_aggregate"] != 713 / 16000,
          "and is NOT the recovery-over-nominal value")
    d = prereg["decision_recorded"]
    check(d["sigma_selected"] == 0.005 and d["sigma_status"].startswith("SELECTED")
          and d["recovery_repeat"] == 1,
          "sigma 0.005 is SELECTED and recovery_repeat is 1")
    check("4.45625%" in d["why_repeat_1"] and "double the recovery weight" in d["why_repeat_1"],
          "with the ratio argument recorded")
    check(d["multistart"].startswith("EXPLICITLY DEFERRED"), "multistart stays deferred")
    fixes = prereg["corrects_two_defects_of_j4"]
    check("J1 TEACHER trace" in fixes["defect_1_alignment_reference"]["j4_did"]
          and "12 aligned rows" in fixes["defect_1_alignment_reference"]["consequence"]
          and "J1 is NEVER the alignment reference"
          in fixes["defect_1_alignment_reference"]["j7_does"],
          "defect 1 (alignment reference) is stated with its consequence and its fix")
    check("J1 teacher STATES" in fixes["defect_2_nominal_states"]["j4_did"]
          and "actually visited in J3" in fixes["defect_2_nominal_states"]["j7_does"],
          "defect 2 (nominal states) is stated with its fix")
    check("IMPLEMENTATION SOURCE ONLY" in fixes["j4_role_here"],
          "and J4 is cited only as an implementation source")

    # ---------------------------------------------------------------- source lineage ------------
    pre = J7.preflight()
    check(pre["verdict"] == "GO" and pre["blockers"] == [], "the preflight is GO")
    lin = pre["source_lineage"]
    check(lin["operational_parent"]["module_state_sha256"] == J7.PIN_PARENT_STATE
          == "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
          and lin["operational_parent"]["j4_module_refused"] == J7.J4_MODULE_STATE_SHA,
          "the parent is the J2 module and the J4 module is named as refused")
    check(lin["nominal_trace"]["sha256"] == J7.PIN_NOMINAL_TRACE
          and "only truncation reference" in lin["nominal_trace"]["role"],
          "the J3 trace supplies the nominal states AND the truncation reference")
    check("RECOVERY LABELS ONLY" in lin["teacher"]["role"]
          and "never the alignment" in lin["teacher"]["role"],
          "the teacher supplies labels only")
    check(set(lin["recovery_traces"]) == {"123", "124", "125"}
          and lin["recovery_traces"]["123"] == J7.PIN_J6_TRACES[123],
          "the three J6 traces are pinned")
    check("no July artefact is an operational input" in lin["july_is_methodological_only"],
          "and July remains methodological only")

    # ---------------------------------------------------------------- the J1 negative guard -----
    g = pre["negative_guard_j1_not_used_as_states"]
    check(g["nominal_block_equals_teacher_states"] is False
          and g["nominal_block_equals_j3_states"] is True,
          "PROVEN: the nominal block is the J3 states and is NOT the J1 teacher states")
    check(g["row_wise_coincidence_fraction"] < 0.01,
          f"and the two barely coincide row-wise ({g['row_wise_coincidence_fraction']})")

    # ---------------------------------------------------------------- schema and composition ----
    s = pre["schema"]
    check(s["observations"] == [16713, 35] and s["actions"] == [16713, 2]
          and s["dtype"] == "float32/float32" and s["actor_width"] == 35,
          "the assembled dataset is 16713 x 35 float32 with 2-D actions")
    c = pre["composition"]
    check(c["nominal_states"] == 500 and c["nominal_repeat"] == 32
          and c["nominal_rows"] == 16000, "nominal 500 x 32 = 16000")
    check(c["recovery_unique_rows"] == 713 and c["recovery_repeat"] == 1
          and c["recovery_rows"] == 713, "recovery 713 x 1 = 713")
    check(c["total_rows"] == 16713, "total exactly 16713")
    check(c["order"] == "nominal block first, then recovery concatenated by seed 123, 124, 125",
          "the order is nominal then recovery by seed")
    check(c["multistart"] == "OMITTED / DEFERRED" and c["dedup"] is False
          and c["downsampling"] is False, "no multistart, no dedup, no downsampling")
    r = pre["ratios"]
    check(abs(r["recovery_over_nominal"] - 0.0445625) < 1e-12
          and abs(r["july_recovery_over_nominal"] - 0.0445) < 1e-12,
          f"recovery/nominal is {r['recovery_over_nominal']} against July's "
          f"{r['july_recovery_over_nominal']}")
    check(abs(r["recovery_over_nominal"] - r["july_recovery_over_nominal"]) < 1e-4,
          "the two ratios agree to better than 0.01 percentage points")
    check(r["recovery_over_aggregate"] == 713 / 16713 == 0.04266140130437384
          == prereg["dataset_specification"]["ratios"]["recovery_over_aggregate"],
          "the preflight and the prereg carry the SAME exact aggregate ratio")

    # ---------------------------------------------------------------- nominal labels ------------
    n = pre["nominal_label_identity"]
    check(n["method"].startswith("target_domain_markov_adaptation._actor_means"),
          "the nominal labels use July's _actor_means path")
    check(n["cross_checked_against"] == "the policy_mean recorded in the J3 trace",
          "cross-checked against what J3 recorded")
    check(n["bit_identical"] is False and n["max_abs_residual"] < 1e-6
          and n["atol"] == 1e-5,
          f"agreement is {n['max_abs_residual']:.3e}, not bit-exact, inside atol 1e-5")
    check("torch" in n["rationale"] and "numpy" in n["rationale"],
          "and the reason is stated: two kernels, same float32 arithmetic")
    # the transcription must equal the real July functions
    sys.path.insert(0, str(J7.BASELINE))
    try:
        import target_domain_noise_adaptation as JN
        import target_domain_markov_adaptation as JM
    except Exception as exc:                                    # pragma: no cover
        raise AssertionError("the July modules must be importable to prove the transcription; run "
                             f"under /opt/anaconda3/envs/envCMC-rllib/bin/python ({exc})")
    names = J7.actor_feature_names()
    state = J7.load_parent_state()
    nominal_rows = json.loads(J7.NOMINAL_TRACE.read_text())
    obs = np.asarray([r["actor_observation_vector_before"] for r in nominal_rows],
                     dtype=np.float32)
    check(np.array_equal(J7.july_forward(state, obs), JN._forward(state, obs)),
          "j7.july_forward is BIT-EQUAL to target_domain_noise_adaptation._forward")
    check(np.array_equal(J7.actor_means(state, obs), JM._actor_means(state, obs)),
          "and j7.actor_means is BIT-EQUAL to target_domain_markov_adaptation._actor_means")
    check(list(J7.discrete_feature_indices(names)) == list(JN._discrete_feature_indices(names)),
          "the discrete selector matches July's")

    # ---------------------------------------------------------------- prefixes and times --------
    teacher_rows = json.loads(J7.TEACHER_TRACE.read_text())
    # against the J1 teacher every seed collapses to 11-12 rows; against J3 they reach 429/273/11
    TEACHER_PREFIX = {123: 12, 124: 12, 125: 11}
    for seed, expected in ((123, 429), (124, 273), (125, 11)):
        leaf = Path(str(J7.J6_LEAF).format(seed=seed))
        trace = json.loads((leaf / "j6_trace.json").read_text())
        mine, rep = J7.truncate_before_discrete_mismatch(nominal_rows, trace, names)
        theirs, their_rep = JN.truncate_before_discrete_mismatch(nominal_rows, trace, names)
        check(rep["retained_steps"] == expected == their_rep["retained_steps"],
              f"seed {seed} retains {expected} rows, agreeing with the July function")
        # against the TEACHER the prefix would collapse - the defect J7 fixes
        _, teacher_rep = J7.truncate_before_discrete_mismatch(teacher_rows, trace, names)
        check(teacher_rep["retained_steps"] == TEACHER_PREFIX[seed]
              and teacher_rep["retained_steps"] <= 12,
              f"seed {seed} against the J1 teacher retains only "
              f"{TEACHER_PREFIX[seed]} rows - the J4 defect")
        check(rep["retained_steps"] >= teacher_rep["retained_steps"],
              f"seed {seed}: the J3 reference is never worse than the teacher one "
              f"({rep['retained_steps']} vs {teacher_rep['retained_steps']})")
        # the teacher labels are the SAME fixed step
        with np.load(J7.TEACHER_DATASET, allow_pickle=False) as archive:
            t_act = np.asarray(archive["actions"], dtype=np.float32)
        for k in (0, expected // 2, expected - 1):
            check(abs(float(trace[k]["time_before"])
                      - float(nominal_rows[k]["time_before"])) <= 1e-9,
                  f"seed {seed} row {k + 1} shares the nominal time")
        check(t_act.shape[0] >= expected, f"the teacher covers seed {seed}'s prefix")
    check(sum((429, 273, 11)) == J7.EXPECTED_RECOVERY_ROWS == 713, "429 + 273 + 11 = 713")

    # ---------------------------------------------------------------- column policy -------------
    cp = pre["column_policy"]
    check(cp["clock_columns"] == [0, 1] and cp["clock_zero_in_dataset"] is True
          and cp["clock_hard_zero_in_weights"] is True,
          "the clock is zero in the data AND hard-zero in the weights")
    check(cp["clock_raw_abs_max_before_projection"] == 1.0,
          "the raw clock is a constant 1.0 before projection")
    check(cp["clock_projection_changed_labels"] is False,
          "and projecting it did not move the nominal labels")
    check(cp["controller_columns"] == list(range(25, 35))
          and cp["controller_masked_in_dataset"] is False
          and cp["controller_non_degenerate"] is True,
          "the controller memory is RAW, unmasked and non-degenerate")
    check(min(cp["controller_abs_max"].values()) > 0.0
          and min(cp["controller_std"].values()) > 0.0,
          "every controller column carries signal and varies")

    # ---------------------------------------------------------------- metrics, not gates --------
    for seed, prefix in (("123", 429), ("124", 273), ("125", 11)):
        v = pre["per_seed"][seed]
        check(v["retained_steps"] == prefix, f"seed {seed} prefix {prefix} is reported")
        check(set(v["policy_mean_vs_teacher"]) == {"rmse", "max_abs", "per_action_rmse"},
              f"seed {seed} reports policy-mean-vs-teacher")
        check(set(v["phase_coverage"]) == {"stance_rows", "swing_rows", "stance_fraction"},
              f"seed {seed} reports stance/swing coverage")
        p = v["penetration_band_coverage"]
        check({"counts", "fractions", "band", "binding_pass", "binding_verdict", "semantics",
               "flags", "thresholds_m", "series_sha256", "bit_identical_to_trace",
               "evaluated_by"} <= set(p),
              f"seed {seed} reports the evaluator's counts, fractions, band, verdict and semantics")
        check(p["evaluated_by"] == "v26c_penetration_contract.evaluate_series"
              and p["contract_sha256"] == PC._sha_file(PC.CONTRACT_FILE),
              f"seed {seed}'s penetration comes from the contract evaluator, at its pinned hash")
        check(p["counts"]["above_hard_binding"] == 0 and p["binding_pass"] is True
              and p["binding_verdict"] == "PASS",
              f"seed {seed} has no sample above the binding band")
        check(p["is_a_gate_here"] is False,
              f"and seed {seed}'s penetration is reported, not enforced, at this stage")
    check(pre["per_seed"]["125"]["phase_coverage"]["stance_rows"] == 0,
          "seed 125's eleven rows are entirely swing - reported, not judged")
    check("REPORTED" in pre["metrics_are_not_gates"]
          and "No minimum is applied" in pre["metrics_are_not_gates"],
          "and the metrics are explicitly not gates")
    check("a minimum-prefix gate" in pre["forbidden_here"],
          "a minimum-prefix gate is forbidden")

    # -------------------------------------------------- penetration: pinned bytes, one authority --
    contract = PC.load_contract()
    for seed, prefix in ((123, 429), (124, 273), (125, 11)):
        leaf = Path(str(J7.J6_LEAF).format(seed=seed))
        pen_file = leaf / "j6_penetration.npz"
        with np.load(pen_file, allow_pickle=False) as arch:
            series = np.asarray(arch["penetration_m"], dtype=np.float64)
        trace = json.loads((leaf / "j6_trace.json").read_text())
        from_trace = np.asarray([float(r["reward_terms"]["grf_penetration_m"]) for r in trace],
                                dtype=np.float64)
        check(J7._sha_array(series) == J7._sha_array(from_trace),
              f"seed {seed}: j6_penetration.npz is BIT-IDENTICAL to the trace's "
              f"reward_terms.grf_penetration_m")
        check(series.tobytes() == from_trace.tobytes(),
              f"seed {seed}: and byte-for-byte equal, not merely close")
        mine = PC.evaluate_series(series[:prefix], contract, label=f"seed {seed}")
        got = pre["per_seed"][str(seed)]["penetration_band_coverage"]
        for key in ("samples", "band", "binding_pass", "binding_verdict", "max_penetration_m",
                    "mean_penetration_m", "argmax_index_1based", "counts", "fractions", "flags",
                    "thresholds_m", "semantics", "counting_conventions"):
            check(got[key] == mine[key],
                  f"seed {seed}: the reported {key} equals an independent evaluate_series call")
        check(mine["samples"] == prefix,
              f"seed {seed}: the evaluator saw exactly the retained prefix ({prefix})")
    # PROVEN: the builder really calls the evaluator; it does not reimplement it
    real_eval, calls = PC.evaluate_series, []

    def spy(series, contract_, **kw):  # noqa: ANN001
        calls.append((len(np.asarray(series)), kw.get("label")))
        return real_eval(series, contract_, **kw)

    PC.evaluate_series = spy
    try:
        J7.assemble()
    finally:
        PC.evaluate_series = real_eval
    check([n for n, _ in calls] == [429, 273, 11],
          f"the builder called evaluate_series once per seed, on the retained prefix ({calls})")
    check(all("J6 seed" in str(lbl) for _, lbl in calls),
          "each call is labelled with its seed")
    # and it writes down no threshold of its own
    for literal in ("0.020", "0.025", "0.028", "0.02,", "0.025,", "0.028,"):
        check(literal not in src, f"the builder never hard-codes the threshold {literal}")
    auth = pre["penetration_authority"]
    check(auth["local_thresholds_in_this_module"] == 0
          and auth["module_sha256"] == J7._sha_file(J7.PENETRATION_EVALUATOR)
          and auth["contract_sha256"] == PC._sha_file(PC.CONTRACT_FILE),
          "the report names the single authority, both halves, and claims no local threshold")
    check(auth["thresholds_m"] == {"soft_diagnostic": 0.020, "july_legacy": 0.025,
                                   "hard_binding": 0.028},
          "and the bands it reports are the contract's, read from the contract")

    # -------------------------------------------------- the three-way clock identity -------------
    with np.load(J7.TEACHER_DATASET, allow_pickle=False) as arch:
        t_times = np.asarray(arch["times"], dtype=np.float64)
    for seed, prefix in ((123, 429), (124, 273), (125, 11)):
        leaf = Path(str(J7.J6_LEAF).format(seed=seed))
        trace = json.loads((leaf / "j6_trace.json").read_text())
        worst = 0.0
        for k in range(prefix):
            a = float(trace[k]["time_before"])
            b = float(t_times[k])
            c_ = float(nominal_rows[k]["time_before"])
            worst = max(worst, abs(a - b), abs(a - c_), abs(b - c_))
        check(worst <= 1e-9,
              f"seed {seed}: EVERY retained row satisfies recovery == teacher == J3 "
              f"(worst {worst:.3e} <= 1e-9)")
        rep = pre["per_seed"][str(seed)]["time_identity"]
        check(rep["rows_checked"] == prefix and rep["atol"] == 1e-9
              and rep["max_abs_residual_s"] == worst,
              f"and seed {seed} reports that check over all {prefix} rows")
        check(rep["teacher_observations_read"] is False,
              f"seed {seed}: without ever reading the teacher's observations")
    check(J7.TIME_IDENTITY_ATOL == 1e-9, "the identity tolerance is 1e-9")
    # a broken alignment must FAIL CLOSED, not be reported
    shifted = np.roll(t_times, 1)
    check(abs(float(nominal_rows[1]["time_before"]) - float(shifted[1])) > 1e-9,
          "NEGATIVE: a one-step shift of the teacher clock would break the identity")

    # -------------------------------------------------- the teacher's states are never read ------
    # STATIC: inside the `with np.load(TEACHER_DATASET)` block, only labels, times and names.
    teacher_keys: set[str] = set()
    teacher_blocks = 0
    for node in ast.walk(tree):
        if not isinstance(node, ast.With):
            continue
        for item in node.items:
            call = item.context_expr
            if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "load" and call.args
                    and isinstance(call.args[0], ast.Name)
                    and call.args[0].id == "TEACHER_DATASET"):
                continue
            teacher_blocks += 1
            var = item.optional_vars.id
            for sub in ast.walk(node):
                if isinstance(sub, ast.Subscript) and isinstance(sub.value, ast.Name) \
                        and sub.value.id == var and isinstance(sub.slice, ast.Constant):
                    teacher_keys.add(str(sub.slice.value))
    check(teacher_blocks == 1, f"the teacher archive is opened exactly once ({teacher_blocks})")
    check(teacher_keys == {"actions", "times", "actor_feature_names"},
          f"STATIC: the builder reads only labels, times and names from it ({sorted(teacher_keys)})")
    check("observations" not in teacher_keys,
          "STATIC GUARD: the builder never accesses the teacher's observations")
    # DYNAMIC: watch every npz key the builder actually touches
    accessed: list[tuple[str, str]] = []
    real_np_load = np.load

    class _Watched:
        def __init__(self, real, path):
            self._real, self._path = real, path

        def __enter__(self):
            self._real.__enter__()
            return self

        def __exit__(self, *a):
            return self._real.__exit__(*a)

        def __getitem__(self, key):
            accessed.append((Path(self._path).name, str(key)))
            return self._real[key]

        def __getattr__(self, name):
            return getattr(self._real, name)

    def watched_load(path, *a, **k):  # noqa: ANN001
        return _Watched(real_np_load(path, *a, **k), path)

    np.load = watched_load
    try:
        J7.assemble()
    finally:
        np.load = real_np_load
    teacher_touched = {k for f, k in accessed if f == "teacher_dataset.npz"}
    check(teacher_touched == {"actions", "times", "actor_feature_names"},
          f"DYNAMIC: the teacher archive yielded only {sorted(teacher_touched)}")
    check(("teacher_dataset.npz", "observations") not in accessed,
          "MEASURED AT RUNTIME: the teacher's observations were never read")
    check({k for f, k in accessed if f == "j6_penetration.npz"} == {"penetration_m"},
          "and the penetration archives yielded only penetration_m")
    check(sum(1 for f, _ in accessed if f == "j6_penetration.npz") == 3,
          "one penetration series per seed")

    # ---------------------------------------------------------------- inert and no output -------
    tripped: list[str] = []
    real_import = builtins.__import__
    banned = {"torch", "ray", "env_factory", "opensim", "rollout_eval", "gymnasium"}

    def guarded(name, *a, **k):  # noqa: ANN001
        if name.split(".")[0] in banned:
            tripped.append(name)
            raise AssertionError(f"the preflight imported {name}")
        return real_import(name, *a, **k)

    builtins.__import__ = guarded
    try:
        pre2 = J7.preflight()
    finally:
        builtins.__import__ = real_import
    check(tripped == [], f"PROVEN INERT: no heavy stack was imported ({tripped})")
    check(pre2["inert"] == {"dataset_written": False, "fit_executed": False,
                            "environment_constructed": False, "rollout_executed": False,
                            "critic_touched": False, "ppo_updates": 0,
                            "note": "the dataset is assembled in memory and discarded"},
          "and the preflight declares that it wrote, fitted and ran nothing")
    check(pre2["content_hashes"] == pre["content_hashes"],
          "two assemblies produce identical content hashes: the build is deterministic")
    e = expect(lambda: J7.main(["--build", "--out", "/tmp/never.npz"]), J7.J7Error,
               "the dataset write must be refused in this phase")
    check("not authorised in this phase" in str(e), "and the refusal says why")
    check(not (HERE / J7.DATASET_NAME).exists()
          and not (HERE / J7.RECEIPT_NAME).exists(),
          "MEASURED: no dataset and no receipt exist")
    check(pre["output_policy"]["content_addressed"] is True
          and pre["output_policy"]["no_clobber"] is True
          and pre["output_policy"]["requires_explicit_out"] is True,
          "the output policy is content-addressed, no-clobber and explicit")

    # ---------------------------------------------------------------- static guarantees ---------
    heavy = {"torch", "ray", "env_factory", "rollout_eval", "opensim"}
    top = {n_.names[0].name.split(".")[0] for n_ in tree.body if isinstance(n_, ast.Import)}
    top |= {(n_.module or "").split(".")[0] for n_ in tree.body if isinstance(n_, ast.ImportFrom)}
    check(not (top & heavy), f"no heavy import at file scope ({sorted(top & heavy)})")
    for name in ("adapt_actor", "PPOConfig", "train_ppo", "optimizer", "Adam", "backward",
                 "critic", "make_cmc_env", "forward_exploration"):
        check(name not in ids, f"the module never references {name}")
    # J7 must not IMPORT the J4/J5/J6 tooling. Naming their artefacts in paths and prose is
    # required, so the check is at the import level, not on substrings.
    imported = {n_.names[0].name for n_ in ast.walk(tree) if isinstance(n_, ast.Import)}
    imported |= {(n_.module or "") for n_ in ast.walk(tree) if isinstance(n_, ast.ImportFrom)}
    check(not any(m.startswith(("v26c_j4", "v26c_j5", "v26c_j6")) for m in imported),
          f"J7 imports none of the J4/J5/J6 tooling ({sorted(imported)})")
    check(not any(i.startswith(("v26c_j4", "v26c_j5", "v26c_j6")) for i in ids),
          "and references no such module as an identifier: their artefacts are read by hash only")
    check(imported == {"__future__", "argparse", "hashlib", "json", "math", "pickle", "sys",
                       "pathlib", "typing", "numpy", "v26c_penetration_contract"},
          f"the whole import surface is the standard library, numpy and the contract ({sorted(imported)})")
    check("MARKOV_CONTROLLER_FEATURE_SCALES" in src,
          "and cites the July scales for the future fit")
    after = {rel: J7._sha_file(HERE / rel) for rel in prereg["pinned_artefacts_sha256"]}
    check(after == pinned_before == prereg["pinned_artefacts_sha256"],
          "MEASURED: every pinned artefact is byte-identical after the whole suite")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
