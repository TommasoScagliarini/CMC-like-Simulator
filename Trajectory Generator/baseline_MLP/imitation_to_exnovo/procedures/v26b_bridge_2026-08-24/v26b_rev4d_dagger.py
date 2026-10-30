"""V26B rev4d - single-variable experiment on the DAgger tiling operator: trace_repeat 4 -> 1.

Token V26B-REV4D-REPEAT.  REV4D differs from REV4B in EXACTLY ONE dimension, the trace_repeat.
The on-policy prefix stays the FULL frozen S1A rollout (steps 1..392), so the negative-ankle
coverage stays at its maximum (65 distinct time indices over 5 windows) while the on-policy share
drops from 75.8221% to 392/892 = 43.9462%.

This deconfounds rev4c, which lowered the share by CUTTING the prefix and therefore moved the share
and the coverage together (65 -> 9 distinct indices).  REV4D holds the coverage and moves the weight.

Declared residual confound: trace_repeat controls the on-policy weight AND the duplicate leakage
across the validation split, which adapt_actor draws as a seeded random permutation over the whole
aggregate.  The two are inseparable inside that parameter, so the primary gate is CLOSED-LOOP
survival and validation MSE is diagnostic only.

Analyzer contract: actor labels are parameterised in KEYS and in PROSE.  The seven-gate meaning
string inherited from the frozen S1B tool is rewritten with this actor's label at write time; the
frozen tool is never modified.  No foreign offline_reference_block at any depth.

July artifacts are BENCHMARK AND PROTOCOL REFERENCE ONLY: never init, never labels.
No PPO, no further round, no sigma, no other candidate, no retry, no promotion.
"""

from __future__ import annotations

import argparse
import json
import pickle
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_rev4c_dagger as C4  # noqa: E402   (rev4c lineage; its analyzer primitives are reused)
import v26b_rev4b_dagger as B4  # noqa: E402   (July fit and the 392-row preflight; hyperparameters)
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_r2i_rollout_postrun_audit as PA  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class Rev4dError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-REV4D-REPEAT"
AMENDMENT_REV4D = HERE / "v26b_amendment_rev4d_repeat.json"
PIN_AMENDMENT_REV4D = "34f8a08ee3fbdb1a23ec5d87cfde6fb07562fef0733e32debdc49532fedc2f32"
ADDENDUM_REV4C_A = HERE / "v26b_addendum_rev4c_a_corrections.json"
PIN_ADDENDUM_REV4C_A = "377fd4e0ff68fc9135f769d8ce2f23935ec9977e185feaa92624360e1942d9ad"

PREFIX_ROWS = 392                 # the FULL frozen S1A prefix: coverage is held at its maximum
TRACE_REPEAT = 1                  # <-- THE SINGLE VARIABLE (rev4b used 4)
INTERPOLATION_STEPS = 0
EXPECTED_AGGREGATE = 892          # 500 + 392*1
EXPECTED_SHARE = 392 / 892
EXPECTED_DISTINCT_NEGATIVE = 65
EXPECTED_NEGATIVE_WINDOWS = [[6, 14], [112, 132], [174, 179], [267, 290], [330, 334]]
ACTOR_LABEL = "REV4D"
SURVIVAL_THRESHOLD_STEPS = 116    # strictly greater than

OUT_CAND = VA.OUT_ROOT / "candidates" / "REV4D_REPEAT_35D_NONDEPLOYABLE"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "rev4d_nominal_det" / "REV4D_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_rev4d_receipt.json"
ROLLOUT_RECEIPT_NAME = "v26b_rev4d_rollout_receipt.json"
LOG_NAME = "rev4d_nominal_det_rollout.log"

FORBIDDEN_KEY_SUBSTRINGS = ("L20", "A2_", "S1B", "kinematics_L", "offline_reference_block")
# Provenance and benchmark subtrees exist precisely to NAME other artifacts, so a foreign actor
# label there is correct, not a defect.  The contract is: no foreign actor label in a key or in a
# prose string that describes THIS actor's own data.
EXEMPT_SUBTREES = ("lineage", "parents_immutable", "lineage_exclusive", "preflight",
                   "reference_points", "benchmark_reference", "framing")
# Narrow, targeted prose templates: these are the descriptive strings the frozen per-actor tools
# emit.  Left unrewritten they would label THIS receipt's own gate/metric blocks with another
# actor's name - the defect recorded by the rev4c addendum.  The scan is deliberately specific
# rather than a blunt actor-name search, which would also reject correct comparative prose.
FORBIDDEN_VALUE_PATTERNS = (
    r"eligibility of (?!%s)[A-Z0-9_]+" % ACTOR_LABEL,
    r"states (?!%s)(?:L\d+|A\d+|S1[AB]|REV\d[A-Z]?) actually visited" % ACTOR_LABEL,
)


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV4D)
    if got != PIN_AMENDMENT_REV4D:
        raise Rev4dError(f"rev4d sha {got} != pinned")
    return json.loads(AMENDMENT_REV4D.read_text(encoding="utf-8"))


def verify_lineage() -> dict[str, Any]:
    """rev4d stands on rev4c, which stands on rev4b. Every parent must be byte-identical."""
    lin = C4.verify_lineage()
    for path, pin, key in ((AMENDMENT_REV4D, PIN_AMENDMENT_REV4D, "amendment_rev4d"),
                           (ADDENDUM_REV4C_A, PIN_ADDENDUM_REV4C_A, "addendum_rev4c_a")):
        got = C.sha256_file(path)
        if got != pin:
            raise Rev4dError(f"{key} sha {got} != pinned")
        lin[key] = got
    am = _amendment()
    for name, pin in am["lineage_exclusive"]["parents_immutable"].items():
        lin.setdefault("rev4d_parents", {})[name] = pin
    lin["framing"] = ("single variable against rev4b: trace_repeat 4 -> 1 with the full 392-row prefix, "
                      "so the on-policy share drops to 43.9462% while the negative-ankle coverage is held "
                      "at its maximum of 65 distinct indices; this deconfounds rev4c")
    lin["july_artifacts_role"] = "benchmark and protocol reference only; never init, never labels"
    return lin


# --- receipt hygiene: keys AND prose ----------------------------------------------------------------

def assert_no_foreign_labels(obj: Any, where: str = "receipt", *, exempt: bool = False) -> None:
    """No foreign actor label in a key, and no frozen-tool prose template that would describe THIS
    actor's own data under another actor's name.  offline_reference_block is forbidden globally.
    Declared provenance and benchmark subtrees are exempt from both scans by construction."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            ks = str(k)
            if ks == "offline_reference_block":
                raise Rev4dError(f"{where}: foreign offline_reference_block is forbidden")
            if not exempt:
                for bad in FORBIDDEN_KEY_SUBSTRINGS:
                    if bad in ks:
                        raise Rev4dError(f"{where}: key {ks!r} carries a hard-coded foreign actor label {bad!r}")
            assert_no_foreign_labels(v, f"{where}.{ks}", exempt=exempt or ks in EXEMPT_SUBTREES)
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_foreign_labels(v, f"{where}[{i}]", exempt=exempt)
    elif isinstance(obj, str) and not exempt:
        for pat in FORBIDDEN_VALUE_PATTERNS:
            m = re.search(pat, obj)
            if m:
                raise Rev4dError(f"{where}: prose carries a foreign actor label: {m.group(0)!r}")


def relabel_seven_gates(seven: Mapping[str, Any], label: str) -> dict[str, Any]:
    """Rewrite the meaning prose inherited from the frozen S1B tool with THIS actor's label.
    The frozen tool is never modified; only the string written into this receipt is parameterised."""
    out = dict(seven)
    out["meaning"] = (f"closed-loop eligibility of {label}. A PASS marks ONLY "
                      "CLOSED_LOOP_ELIGIBLE_PENDING_ARCHITECT_AUDIT: it is NOT a promotion, the candidate "
                      "stays NON-DEPLOYABLE, and it does NOT resolve sigma")
    out["meaning_relabelled_from_frozen_tool"] = (
        "the arithmetic comes verbatim from the frozen v26b_s1b_rollout.eligibility_gates; only the "
        "descriptive string was parameterised at write time, per the rev4c addendum forward rule")
    return out


# --- preflight on the FULL prefix -------------------------------------------------------------------

def preflight() -> dict[str, Any]:
    """No-write, fail-closed preflight on the full 392-row S1A prefix."""
    base = B4.preflight()                       # full fail-closed audit of the 392-row prefix
    view = base["_view"]
    obs500 = np.asarray(view["obs"], dtype=np.float32)
    u_ik500 = np.asarray(view["u_ik"], dtype=np.float64)
    obsS = np.asarray(base["_obsS"], dtype=np.float32)
    labels = np.asarray(base["_labels"], dtype=np.float64)
    n = int(obsS.shape[0])
    if n != PREFIX_ROWS:
        raise Rev4dError(f"prefix rows {n} != {PREFIX_ROWS}")
    rows = json.loads((SA.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    if len(rows) < n:
        raise Rev4dError("recorded trace shorter than the prefix")
    rows = rows[:n]
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, n + 1)):
        raise Rev4dError(f"steps 1..{n} are not contiguous")
    for i, r in enumerate(rows):
        rec = np.asarray(r["actor_observation_vector_before"], dtype=np.float32).reshape(-1)
        if rec.shape != (R.ENV_ACTOR_WIDTH,):
            raise Rev4dError(f"row {i + 1}: obs width {rec.shape} != {(R.ENV_ACTOR_WIDTH,)}")
        if not np.array_equal(rec, obsS[i]):
            raise Rev4dError(f"row {i + 1}: obs35 differs from the recorded vector")
    if not np.all(np.isfinite(obsS)) or not np.all(np.isfinite(labels)):
        raise Rev4dError("non-finite obs35 or labels in the prefix")
    tS = np.asarray(DS.trajectory_from_job(SA.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)[:n]
    t500 = np.asarray(DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)[:n]
    dt = float(np.abs(tS - t500).max())
    if dt != 0.0:
        raise Rev4dError(f"time alignment broken on the prefix: {dt}")
    kS = [obsS[i].tobytes() for i in range(n)]
    dup = len(kS) - len(set(kS))
    m500: dict[bytes, np.ndarray] = {}
    for i in range(len(obs500)):
        m500.setdefault(obs500[i].tobytes(), u_ik500[i])
    coll = [i for i in range(n) if kS[i] in m500]
    conf = [i for i in coll if not np.array_equal(m500[kS[i]], labels[i])]
    if conf:
        raise Rev4dError(f"{len(conf)} collisions carry CONFLICTING labels: NO-GO")

    q_ik = SC.decode_action(u_ik500)[:, 1]
    neg = q_ik < 0.0
    neg_pref = neg[:n]
    wins, s = [], None
    for i, f in enumerate(neg_pref):
        if f and s is None:
            s = i
        if not f and s is not None:
            wins.append([s + 1, i]); s = None
    if s is not None:
        wins.append([s + 1, n])
    distinct_neg = int(neg_pref.sum())
    if distinct_neg != EXPECTED_DISTINCT_NEGATIVE or wins != EXPECTED_NEGATIVE_WINDOWS:
        raise Rev4dError(f"negative coverage {distinct_neg}/{wins} != expected "
                         f"{EXPECTED_DISTINCT_NEGATIVE}/{EXPECTED_NEGATIVE_WINDOWS}")
    agg_rows = len(obs500) + n * TRACE_REPEAT
    if agg_rows != EXPECTED_AGGREGATE:
        raise Rev4dError(f"aggregate {agg_rows} != {EXPECTED_AGGREGATE}")
    share = n * TRACE_REPEAT / agg_rows
    if share != EXPECTED_SHARE:
        raise Rev4dError(f"share {share!r} != {EXPECTED_SHARE!r}")
    return {"verdict": "GO", "prefix_rows": n, "trace_repeat": TRACE_REPEAT,
            "interpolation_steps": INTERPOLATION_STEPS,
            "steps_contiguous_1_to_392": True, "obs35_matches_recorded_vectors": True,
            "obs35_and_labels_finite": True,
            "time_alignment_max_abs_difference_s": dt,
            "labels": {"rule": "same-step u_IK at teacher_index = step - 1", "rows": n,
                       "sha256": __import__("hashlib").sha256(labels.tobytes()).hexdigest()},
            "duplicates_in_prefix": dup,
            "bitwise_collisions_with_corpus": len(coll), "collisions_with_conflicting_labels": len(conf),
            "negative_ankle_coverage": {"distinct_negative_time_indices": distinct_neg,
                                        "windows": wins,
                                        "rows_in_full_corpus": int(neg.sum()),
                                        "note": "coverage held at the maximum available from the S1A trace"},
            "composition": {"teacher_corpus_rows": int(len(obs500)), "prefix_rows": n,
                            "trace_repeat": TRACE_REPEAT, "aggregate_rows": agg_rows,
                            "on_policy_share": share, "on_policy_share_exact_fraction": f"{n * TRACE_REPEAT}/{agg_rows}",
                            "rev4b_share_for_contrast": 1568 / 2068},
            "_view": view, "_obsS": obsS, "_labels": labels, "_init": base["_init"]}


def build_aggregate(pre: Mapping[str, Any]) -> dict[str, np.ndarray]:
    """concat(teacher, tile(prefix, TRACE_REPEAT)). The single variable lives here."""
    view = pre["_view"]
    obs_t = np.asarray(view["obs"], dtype=np.float32)
    act_t = np.asarray(view["u_ik"], dtype=np.float32)
    obs_v = np.asarray(pre["_obsS"], dtype=np.float32)
    act_v = np.asarray(pre["_labels"], dtype=np.float32)
    return {"observations": np.concatenate([obs_t, np.tile(obs_v, (TRACE_REPEAT, 1))], axis=0),
            "actions": np.concatenate([act_t, np.tile(act_v, (TRACE_REPEAT, 1))], axis=0)}


def offline_report(init_raw, export_raw, scaled, pre, vec, t1: float, save_reload: bool) -> dict[str, Any]:
    f32 = np.float32
    view = pre["_view"]
    obs500 = np.asarray(view["obs"], dtype=np.float32); u_ik = np.asarray(view["u_ik"], dtype=np.float64)
    obsP = np.asarray(pre["_obsS"], dtype=np.float32); labP = np.asarray(pre["_labels"], dtype=np.float64)
    q_ik = SC.decode_action(u_ik)[:, 1]; neg = q_ik < 0.0
    m500 = RF.numpy_mean(export_raw, obs500); mP = RF.numpy_mean(export_raw, obsP)
    i500 = RF.numpy_mean(init_raw, obs500)
    obs_all = np.concatenate([obs500, obsP]); tgt_all = np.concatenate([u_ik, labP])
    m_all = RF.numpy_mean(export_raw, obs_all); i_all = RF.numpy_mean(init_raw, obs_all)
    agg_before = float(np.sqrt(np.mean((i_all - tgt_all) ** 2)))
    agg_after = float(np.sqrt(np.mean((m_all - tgt_all) ** 2)))
    t2 = float(np.max(np.abs(m500 - RF.numpy_mean(scaled, (obs500.astype(np.float64) / vec[None, :]).astype(f32)))))
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obs500[:64])
    integrity = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER,
                 "clock_columns_zero": bool(struct["clock_columns_zero"]),
                 "clock_invariance_bit_identical": bool(inv["bit_identical"]),
                 "logstd_byte_identical_to_init": bool(np.array_equal(np.asarray(export_raw["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
                                                       and np.array_equal(np.asarray(export_raw["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:])),
                 "save_reload_exact": bool(save_reload), "no_critic": True}
    binding = {"integrity_invariants": {"binding": True, **integrity, "pass": bool(all(integrity.values()))},
               "function_preservation": {"binding": True, "T1_maxabs": float(t1), "T2_maxabs": t2,
                                         "tol": G.PRESERVATION_TOL, "pass": bool(t1 <= G.PRESERVATION_TOL and t2 <= G.PRESERVATION_TOL)},
               "fit_convergence": {"binding": True, "aggregate_rmse_before": agg_before, "aggregate_rmse_after": agg_after,
                                   "pass": bool(agg_after < agg_before)}}

    def shape(pred, ref):
        out = {}
        for j, jn in ((0, "knee"), (1, "ankle")):
            x, y = pred[:, j], ref[:, j]
            sx, sy = float(x.std()), float(y.std())
            out[jn] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                       "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                       "amplitude_ratio": (sx / sy if sy > 0 else None)}
        return out
    s0d = B4._s0d_state()
    d_s0d = m500 - RF.numpy_mean(s0d, obs500)
    h = np.asarray(L.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    qh = SC.decode_action(m500)
    q_cmd = qh[:, 1]
    measures = {"note": "reported without invented thresholds; promotion is left to the architect audit",
                "on_corpus_500": shape(m500, u_ik),
                "on_full_prefix_392": shape(mP, labP),
                "on_all_97_negative_rows_of_the_corpus": shape(m500[neg], u_ik[neg]),
                "negative_window_sign": {"rows": int(neg.sum()),
                                         "fraction_positive_command_before": float(np.mean(SC.decode_action(i500)[neg, 1] > 0.0)),
                                         "fraction_positive_command_after": float(np.mean(q_cmd[neg] > 0.0)),
                                         "min_commanded_ankle": float(q_cmd[neg].min())},
                "max_abs_vs_source_actor_INFORMATIONAL": {"max_abs_per_joint": [float(np.abs(d_s0d[:, j]).max()) for j in (0, 1)],
                                                          "mean_abs_per_joint": [float(np.abs(d_s0d[:, j]).mean()) for j in (0, 1)],
                                                          "status": "INFORMATIONAL only"},
                "healthy_symmetry_DIAGNOSTIC": {"rmse_per_joint": [float(np.sqrt(np.mean((qh[:, j] - h[:, j]) ** 2))) for j in (0, 1)],
                                                "status": "DIAGNOSTIC only"}}
    failed = [k for k, v in binding.items() if not v["pass"]]
    return {"binding": binding, "failed": failed, "all_binding_pass": bool(not failed),
            "measures": measures, "actor_digest": struct["actor_digest"], "structure": struct}


# --- closed loop ------------------------------------------------------------------------------------

def rollout_command(python_exe: str = SB.PYTHON_EXE) -> list[str]:
    return [python_exe, str(F1.ROLLOUT_EVAL), "--checkpoint", str(OUT_CAND / "rl_module"),
            "--no-auto-config", "--config", str(F1.RUNTIME_CONFIG),
            "--episode-start-offset-s", repr(float(R.EXACT_STARTS["nominal"])),
            "--action-selection", "deterministic", "--seed", str(R.DET_SEED),
            "--output-dir", str(JOB_DIR), "--record-outputs", "--record-policy-trace",
            *[str(a) for a in F1.JOB_TIMEOUT_ARGS], *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]]]


def run_closed_loop(cand_receipt_sha: str, python_exe: str = SB.PYTHON_EXE) -> dict[str, Any]:
    if JOB_DIR.exists():
        raise Rev4dError(f"no-clobber: {JOB_DIR} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    rec: dict[str, Any] = {"schema": "v26b_rev4d_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
                           "amendment_rev4d": PIN_AMENDMENT_REV4D, "actor_label": ACTOR_LABEL,
                           "candidate_receipt_sha256": cand_receipt_sha, "deployable": False,
                           "sigma": "UNRESOLVED; deterministic mean only", "command": cmd,
                           "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
                           "log": C.rel(log_path), "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise Rev4dError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        seven = relabel_seven_gates(SB.eligibility_gates(analysis, rowscan), ACTOR_LABEL)
        steps = int(analysis["completion"]["steps"])
        rec["analysis"] = analysis
        rec["fsm_counters_rowscan"] = rowscan
        rec["seven_gates"] = seven
        rec["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                              "b3_phase_window": PA.b3_late_stance(rows),
                              "actor_comparison": C4.actor_comparison(JOB_DIR, ACTOR_LABEL)}
        rec["primary_gate"] = {
            "measure": "survival in steps", "observed_steps": steps,
            "threshold": SURVIVAL_THRESHOLD_STEPS, "rule": "STRICTLY GREATER THAN 116",
            "hypothesis": "with the coverage held at its maximum, lowering the on-policy weight via the "
                          "tiling operator (trace_repeat 4 -> 1) improves closed-loop survival",
            "verdict": ("PASS" if steps > SURVIVAL_THRESHOLD_STEPS else "FAIL"),
            "interpretation": _amendment()["CRITERIA"]["interpretation_grid"],
            "reference_points": _amendment()["CRITERIA"]["primary_gate"]["reference_points"]}
        rec["promotion"] = {"promoted": False,
                            "requires_all_of": ["500/500 completion", "zero critical counters", ">= 2 valid cycles",
                                                "penetration within the current gate", "kinematic quality gates"],
                            "note": "no automatic promotion even if every criterion passes"}
        status = "PROMOTION_CRITERIA_MET_NOT_PROMOTED" if seven["all_pass"] else "NOT_PROMOTED"
    finally:
        rec["status"] = status
        assert_no_foreign_labels(rec, "rollout_receipt")
        R.write_json_exclusive(JOB_DIR / ROLLOUT_RECEIPT_NAME, rec)
    return rec


def run_stage(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise Rev4dError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    pre = preflight()
    if OUT_CAND.exists():
        raise FileExistsError(f"no-clobber: {OUT_CAND} exists")
    agg = build_aggregate(pre)
    if agg["observations"].shape[0] != EXPECTED_AGGREGATE:
        raise Rev4dError("aggregate row count mismatch")
    vec, scale_table = G.scale_vector()
    init_raw = pre["_init"]
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, np.asarray(pre["_view"]["obs"], dtype=np.float32), vec)
    if t1 > G.PRESERVATION_TOL:
        raise Rev4dError(f"prefit T1 FAILED {t1:.3e}")
    obs_scaled = (agg["observations"].astype(np.float64) / vec[None, :]).astype(np.float32)
    scaled, frep = B4.fit_july(init_scaled, obs_scaled, agg["actions"], progress=progress)
    export_raw = G.export_state_from_scaled(scaled, vec)
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    OUT_CAND.parent.mkdir(parents=True, exist_ok=True)
    lock, tok = RF.acquire_export_lock(OUT_CAND)
    staging = None; promoted = False
    try:
        staging = RF._staging_dir_for(OUT_CAND)
        sm = staging / "rl_module"; sm.mkdir(parents=True, exist_ok=False)
        shutil.copy2(B4.S1A_MODULE / "metadata.json", sm / "metadata.json")
        shutil.copy2(B4.S1A_MODULE / "class_and_ctor_args.pkl", sm / "class_and_ctor_args.pkl")
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(sm)
        exact = bool(W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded).get("exact"))
        rep = offline_report(init_raw, export_raw, scaled, pre, vec, t1, exact)
        names35, _, mshas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": rep["actor_digest"], "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                    "manifest35_sha256": mshas["manifest35_sha256"],
                    "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE,
                    "derived_from": C.rel(B4.S1A_MODULE), "source_actor_digest": B4.PIN_S1A_ACTOR,
                    "contract": A.CONTRACT_STRING, "deployable": False, "sigma_unresolved": True,
                    "actor_label": ACTOR_LABEL,
                    "experiment": "single variable against rev4b: trace_repeat 4 -> 1 at full prefix coverage",
                    "offline_verdict": ("PASS" if rep["all_binding_pass"] else "FAIL"),
                    "status": "INTERMEDIATE CANDIDATE (rev4d); never promoted automatically"}
        assert_no_foreign_labels(manifest, "manifest")
        A.assert_no_deployable_marking(manifest, "manifest")
        C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        receipt = {"schema": "v26b_rev4d_fit.1", "amendment_rev4d": PIN_AMENDMENT_REV4D,
                   "authorized_stage": AUTHORIZED_STAGE, "actor_label": ACTOR_LABEL,
                   "deployable": False, "sigma_unresolved": True, "lineage": lineage,
                   "framing": _amendment()["ARCHITECTURAL_FRAMING_BINDING"],
                   "init": {"module": C.rel(B4.S1A_MODULE), "actor_digest": B4.PIN_S1A_ACTOR,
                            "role": "S1A is the sole init, the anchor parameter and the frozen collection policy"},
                   "preflight": {k: v for k, v in pre.items() if not k.startswith("_")},
                   "dataset": {"teacher_corpus_rows": int(len(pre["_view"]["obs"])),
                               "prefix_rows": PREFIX_ROWS, "trace_repeat": TRACE_REPEAT,
                               "interpolation_steps": INTERPOLATION_STEPS,
                               "aggregate_rows": int(agg["observations"].shape[0]),
                               "on_policy_share": float(PREFIX_ROWS * TRACE_REPEAT / agg["observations"].shape[0]),
                               "operation": "concat(teacher, tile(full prefix, 1)); no dedup, no truncation",
                               "single_variable": "trace_repeat 4 -> 1; every other dimension is identical to rev4b"},
                   "july_protocol": {k: v for k, v in frep.items() if k != "history"},
                   "loss_history_first_last": {"first": frep["history"][0], "last": frep["history"][-1]},
                   "loss_history_full": frep["history"],
                   "validation_note": "validation MSE and the early-stopping epoch are DIAGNOSTIC only; with "
                                      "trace_repeat 1 there are no exact duplicates straddling the split",
                   "scaling": {"table": scale_table, "T1_maxabs": t1},
                   "offline": rep, "save_reload_exact": exact,
                   "output_module": C.rel(OUT_CAND / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "one repeat-operator experiment; no PPO, no further round, no sigma, no other candidate, no promotion",
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        assert_no_foreign_labels(receipt, "receipt")
        A.assert_no_deployable_marking(receipt, "receipt")
        C.write_json(staging / RECEIPT_NAME, receipt)
        csha = C.sha256_file(staging / RECEIPT_NAME)
        RF.promote_staging(staging, OUT_CAND); promoted = True
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        RF.release_export_lock(lock, tok)
    out = {"candidate": C.rel(OUT_CAND), "receipt_sha256": csha, "offline": rep}
    out["rollout"] = (run_closed_loop(csha) if rep["all_binding_pass"]
                      else {"status": "NOT_RUN", "reason": "a binding offline gate failed"})
    return out


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev4d trace_repeat single-variable experiment")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if args.preflight_only:
        verify_lineage()
        print(json.dumps({k: v for k, v in preflight().items() if not k.startswith("_")}, indent=2, default=str))
        return 0
    if not args.execute:
        verify_lineage()
        print(json.dumps({"mode": "dry", "ok": True}, indent=2))
        return 0
    out = run_stage(authorized_stage=args.authorized_stage)
    r = out["rollout"]
    print(json.dumps({"candidate": out["candidate"], "receipt_sha256": out["receipt_sha256"],
                      "offline_pass": out["offline"]["all_binding_pass"],
                      "rollout_status": r.get("status"),
                      "primary_gate": r.get("primary_gate"),
                      "seven_gates_failed": (r.get("seven_gates") or {}).get("failed")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
