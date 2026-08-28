"""V26B rev4c — single-variable balance experiment: July on-policy share restored to 35.23%.

Token V26B-REV4C-BALANCE.  Identical to rev4b in every verified July hyperparameter, loss term and
aggregation operator; the ONLY change is the DAgger collection budget: the on-policy prefix is
capped at the first 68 contiguous S1A rows, giving 500 + 68*4 = 772 rows and a 35.23% on-policy
share against rev4b's 75.82%.

The 68 cap is a CONTROLLED COLLECTION BUDGET, not a 'causally valid prefix' and not a literal
replication of the July incident: rev4c is faithful to the July BALANCE and, by construction, not
to the July OPERATOR.  Neither rev4b nor rev4c is 'July-faithful' without that qualification.

Analyzer contract (rev4c tooling requirement): actor labels are PARAMETERISED - no hard-coded
actor name may appear in a key - and no foreign offline_reference_block may be embedded.

No PPO, no further round, no sigma, no other candidate, no retry, no promotion.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_rev4b_dagger as B4  # noqa: E402      (rev4b lineage, July fit and preflight primitives)
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s1a_bc as A  # noqa: E402
import v26b_s1_prereg as S1  # noqa: E402
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


class Rev4cError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-REV4C-BALANCE"
AMENDMENT_REV4C = HERE / "v26b_amendment_rev4c_balance.json"
PIN_AMENDMENT_REV4C = "b25b0fe4f89a9ea4a7ac6b6b9ca8b5d2d393e4535a2a0944edd5386dd62790bb"
ADDENDUM_REV4B_A = HERE / "v26b_addendum_rev4b_a_corrections.json"
PIN_ADDENDUM_REV4B_A = "74f7c50a83877a2005708e95ee911992f301bbe3799f41c823f807dbbb847ce1"

PREFIX_CAP = 68
EXPECTED_AGGREGATE = 772
ACTOR_LABEL = "REV4C"
SURVIVAL_THRESHOLD_STEPS = 100

OUT_CAND = VA.OUT_ROOT / "candidates" / "REV4C_BALANCE_35D_NONDEPLOYABLE"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "rev4c_nominal_det" / "REV4C_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_rev4c_receipt.json"
ROLLOUT_RECEIPT_NAME = "v26b_rev4c_rollout_receipt.json"
LOG_NAME = "rev4c_nominal_det_rollout.log"
FORBIDDEN_KEY_SUBSTRINGS = ("L20", "A2_", "S1B", "kinematics_L", "offline_reference_block")
# Provenance subtrees exist precisely to NAME other artifacts, so a foreign actor label there is
# correct, not a defect.  The contract is: no foreign actor label in a key that names THIS actor's
# own data.  The offline_reference_block prohibition stays GLOBAL, because that field embeds another
# actor's METRICS, which is the defect the rev4b addendum recorded.
EXEMPT_PROVENANCE_SUBTREES = ("lineage", "parents_immutable", "preflight")
INCIDENT_RECORD = {
    "what": "the first execution of this stage aborted AFTER the fit and BEFORE materialisation",
    "cause": "the receipt-hygiene scanner raised a FALSE POSITIVE on receipt.lineage.rev3z_aggregate_states_preserved, "
             "whose keys L05/L10/L20 are legitimate provenance names of other candidates, not mislabelled rev4c metrics",
    "state_after_the_abort": "nothing was materialised, the staging directory was removed, no lock remained and the rollout never started",
    "fix": "the key scan now exempts declared provenance subtrees; the offline_reference_block prohibition remains global",
    "re_execution": "the stage was executed again from scratch. The fit is bit-deterministic (seed 123, fixed epochs, "
                    "deterministic algorithms), so the re-execution reproduces the same computation; it is not a retry of "
                    "an experiment whose outcome was reshuffled, and no artifact from the aborted attempt survived",
    "disclosed": True,
}


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV4C)
    if got != PIN_AMENDMENT_REV4C:
        raise Rev4cError(f"rev4c sha {got} != pinned")
    return json.loads(AMENDMENT_REV4C.read_text(encoding="utf-8"))


def verify_lineage() -> dict[str, Any]:
    lin = B4.verify_lineage()
    for path, pin, key in ((AMENDMENT_REV4C, PIN_AMENDMENT_REV4C, "amendment_rev4c"),
                           (ADDENDUM_REV4B_A, PIN_ADDENDUM_REV4B_A, "addendum_rev4b_a")):
        got = C.sha256_file(path)
        if got != pin:
            raise Rev4cError(f"{key} sha {got} != pinned")
        lin[key] = got
    lin["framing"] = ("the 68 cap is a controlled collection budget restoring the July 35.23% on-policy share; "
                      "rev4c is balance-faithful and operator-divergent, the mirror image of rev4b")
    return lin


# --- receipt hygiene (rev4c tooling requirement) --------------------------------------------------------

def assert_no_foreign_labels(obj: Any, where: str = "receipt", *, in_provenance: bool = False) -> None:
    """No hard-coded foreign actor name in a key that names THIS actor's own data, and no
    offline_reference_block anywhere.  Declared provenance subtrees are exempt from the KEY scan,
    because naming other artifacts is exactly their purpose; the block prohibition stays global."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            ks = str(k)
            if ks == "offline_reference_block":
                raise Rev4cError(f"{where}: foreign offline_reference_block is forbidden")
            if not in_provenance:
                for bad in FORBIDDEN_KEY_SUBSTRINGS:
                    if bad in ks:
                        raise Rev4cError(f"{where}: key {ks!r} carries a hard-coded foreign actor label {bad!r}")
            assert_no_foreign_labels(v, f"{where}.{ks}",
                                     in_provenance=in_provenance or ks in EXEMPT_PROVENANCE_SUBTREES)
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_foreign_labels(v, f"{where}[{i}]", in_provenance=in_provenance)


def actor_comparison(job_dir: Path, label: str) -> dict[str, Any]:
    """Closed-loop comparison with PARAMETERISED actor labels. Diagnostic only, never a gate."""
    traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(traj["obs35"], dtype=np.float32)
    act = np.asarray(traj["b_raw_action"], dtype=np.float64)
    cc = L.load_cache(R.OUT_CACHE, "nominal")
    idx = cc.lookup(np.asarray(traj["t_pre"], dtype=np.float64))
    u_ik = cc.ik_action[idx].astype(np.float64)
    q_ik = SC.decode_action(u_ik)
    s0d_on = RF.numpy_mean(B4._s0d_state(), obs)
    knee = obs[:, RO.IDX_KNEE_Q].astype(np.float64); ankle = obs[:, RO.IDX_ANKLE_Q].astype(np.float64)
    q_cmd = SC.decode_action(act)
    neg = q_ik[:, 1] < 0.0

    def jstats(x, bounds):
        return {"min": float(x.min()), "max": float(x.max()), "mean": float(x.mean()), "range": float(np.ptp(x)),
                "frac_negative": float(np.mean(x < 0.0)),
                "outside_bounds_steps": int(np.sum((x < bounds[0]) | (x > bounds[1])))}

    def vs(x, y):
        sx, sy = float(x.std()), float(y.std())
        deg = bool(np.all(y >= 0.0) or np.all(y <= 0.0))
        return {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                "amplitude_ratio": (sx / sy if sy > 0 else None),
                "sign_agreement": (None if deg else float(np.mean(np.sign(x) == np.sign(y)))),
                "sign_agreement_verdict": ("VOID_degenerate_reference" if deg else "valid")}

    per = lambda d: [float(np.abs(d[:, j]).mean()) for j in (0, 1)]  # noqa: E731
    return {"actor_label": label,
            "scope": f"CLOSED-LOOP metrics on the states {label} actually visited. Diagnostic only, never a gate",
            "kinematics": {"knee_q": jstats(knee, RO.KNEE_BOUNDS), "ankle_q": jstats(ankle, RO.ANKLE_BOUNDS)},
            "vs_prosthetic_IK_target": {"knee_q": vs(knee, q_ik[:, 0]), "ankle_q": vs(ankle, q_ik[:, 1])},
            "actions_vs_source_actor": {"mean_abs": per(act - s0d_on),
                                        "max_abs": [float(np.abs((act - s0d_on)[:, j]).max()) for j in (0, 1)],
                                        "source": "the S0D actor, used only as a numerical reference"},
            "actions_vs_uIK_same_times": {"mean_abs": per(act - u_ik),
                                          "rmse": [float(np.sqrt(np.mean((act[:, j] - u_ik[:, j]) ** 2))) for j in (0, 1)]},
            "negative_window_sign": {"rows": int(neg.sum()),
                                     "fraction_positive_ankle_command": (float(np.mean(q_cmd[neg, 1] > 0.0)) if neg.any() else None),
                                     "min_realised_ankle_q": (float(ankle[neg].min()) if neg.any() else None)}}


# --- preflight on the capped prefix ------------------------------------------------------------------------

def preflight() -> dict[str, Any]:
    """No-write, fail-closed preflight on the first 68 contiguous S1A rows."""
    base = B4.preflight()                       # full fail-closed audit of the 392-row prefix
    n = PREFIX_CAP
    view = base["_view"]
    obs500 = np.asarray(view["obs"], dtype=np.float32)
    u_ik500 = np.asarray(view["u_ik"], dtype=np.float64)
    obsS = np.asarray(base["_obsS"], dtype=np.float32)[:n]
    labels = np.asarray(base["_labels"], dtype=np.float64)[:n]
    rows = json.loads((SA.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))[:n]
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, n + 1)):
        raise Rev4cError("steps 1..68 are not contiguous")
    for i, r in enumerate(rows):
        rec = np.asarray(r["actor_observation_vector_before"], dtype=np.float32).reshape(-1)
        if not np.array_equal(rec, obsS[i]):
            raise Rev4cError(f"row {i + 1}: obs35 differs from the recorded vector")
    if not np.all(np.isfinite(obsS)) or not np.all(np.isfinite(labels)):
        raise Rev4cError("non-finite obs35 or labels in the capped prefix")
    tS = np.asarray(DS.trajectory_from_job(SA.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)[:n]
    t500 = np.asarray(DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)[:n]
    dt = float(np.abs(tS - t500).max())
    if dt != 0.0:
        raise Rev4cError(f"time alignment broken on the capped prefix: {dt}")
    kS = [obsS[i].tobytes() for i in range(n)]
    dup = len(kS) - len(set(kS))
    m500: dict[bytes, np.ndarray] = {}
    for i in range(len(obs500)):
        m500.setdefault(obs500[i].tobytes(), u_ik500[i])
    coll = [i for i in range(n) if kS[i] in m500]
    conf = [i for i in coll if not np.array_equal(m500[kS[i]], labels[i])]
    if conf:
        raise Rev4cError(f"{len(conf)} collisions carry CONFLICTING labels: NO-GO")
    q_ik = SC.decode_action(u_ik500)[:, 1]
    neg_pref = (q_ik < 0.0)[:n]
    wins, s = [], None
    for i, f in enumerate(neg_pref):
        if f and s is None:
            s = i
        if not f and s is not None:
            wins.append([s + 1, i]); s = None
    if s is not None:
        wins.append([s + 1, n])
    agg_rows = len(obs500) + n * B4.J_TRACE_REPEAT
    if agg_rows != EXPECTED_AGGREGATE:
        raise Rev4cError(f"aggregate {agg_rows} != {EXPECTED_AGGREGATE}")
    return {"verdict": "GO", "prefix_cap": n,
            "steps_contiguous_1_to_68": True, "obs35_matches_recorded_vectors": True,
            "obs35_and_labels_finite": True,
            "time_alignment_max_abs_difference_s": dt,
            "labels": {"rule": "same-step u_IK at teacher_index = step - 1", "rows": n,
                       "sha256": __import__("hashlib").sha256(labels.tobytes()).hexdigest()},
            "duplicates_in_capped_prefix": dup,
            "bitwise_collisions_with_corpus": len(coll), "collisions_with_conflicting_labels": len(conf),
            "negative_ankle_coverage": {"rows_in_capped_prefix": int(neg_pref.sum()),
                                        "windows": wins,
                                        "rows_in_full_corpus": int((q_ik < 0.0).sum()),
                                        "note": "the cap is a collection budget; coverage is reported, not claimed as causal validity"},
            "composition": {"teacher_corpus_rows": int(len(obs500)), "prefix_rows": n,
                            "trace_repeat": B4.J_TRACE_REPEAT, "aggregate_rows": agg_rows,
                            "on_policy_share": float(n * B4.J_TRACE_REPEAT / agg_rows),
                            "july_r1_share": 0.3523316062176166},
            "_view": view, "_obsS": obsS, "_labels": labels, "_init": base["_init"]}


def build_aggregate(pre: Mapping[str, Any]) -> dict[str, np.ndarray]:
    view = pre["_view"]
    obs_t = np.asarray(view["obs"], dtype=np.float32)
    act_t = np.asarray(view["u_ik"], dtype=np.float32)
    obs_v = np.asarray(pre["_obsS"], dtype=np.float32)
    act_v = np.asarray(pre["_labels"], dtype=np.float32)
    return {"observations": np.concatenate([obs_t, np.tile(obs_v, (B4.J_TRACE_REPEAT, 1))], axis=0),
            "actions": np.concatenate([act_t, np.tile(act_v, (B4.J_TRACE_REPEAT, 1))], axis=0)}


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
    import f2r_labeller as LB
    h = np.asarray(LB.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    qh = SC.decode_action(m500)
    q_cmd = SC.decode_action(m500)[:, 1]
    measures = {"note": "reported without invented thresholds; promotion is left to the architect audit",
                "on_corpus_500": shape(m500, u_ik),
                "on_capped_prefix_68": shape(mP, labP),
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


# --- closed loop ---------------------------------------------------------------------------------------

def rollout_command(python_exe: str = SB.PYTHON_EXE) -> list[str]:
    return [python_exe, str(F1.ROLLOUT_EVAL), "--checkpoint", str(OUT_CAND / "rl_module"),
            "--no-auto-config", "--config", str(F1.RUNTIME_CONFIG),
            "--episode-start-offset-s", repr(float(R.EXACT_STARTS["nominal"])),
            "--action-selection", "deterministic", "--seed", str(R.DET_SEED),
            "--output-dir", str(JOB_DIR), "--record-outputs", "--record-policy-trace",
            *[str(a) for a in F1.JOB_TIMEOUT_ARGS], *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]]]


def run_closed_loop(cand_receipt_sha: str, python_exe: str = SB.PYTHON_EXE) -> dict[str, Any]:
    if JOB_DIR.exists():
        raise Rev4cError(f"no-clobber: {JOB_DIR} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    rec: dict[str, Any] = {"schema": "v26b_rev4c_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
                           "amendment_rev4c": PIN_AMENDMENT_REV4C, "actor_label": ACTOR_LABEL,
                           "candidate_receipt_sha256": cand_receipt_sha, "deployable": False,
                           "sigma": "UNRESOLVED; deterministic mean only", "command": cmd,
                           "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
                           "log": C.rel(log_path), "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise Rev4cError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        seven = SB.eligibility_gates(analysis, rowscan)
        steps = int(analysis["completion"]["steps"])
        rec["analysis"] = analysis
        rec["fsm_counters_rowscan"] = rowscan
        rec["seven_gates"] = seven
        rec["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                              "b3_phase_window": PA.b3_late_stance(rows),
                              "actor_comparison": actor_comparison(JOB_DIR, ACTOR_LABEL)}
        rec["primary_causal_test"] = {
            "measure": "survival in steps", "observed_steps": steps,
            "threshold": SURVIVAL_THRESHOLD_STEPS,
            "hypothesis": "the rev4b collapse was caused by on-policy over-weighting (75.82% share)",
            "verdict": ("SUPPORTED" if steps > SURVIVAL_THRESHOLD_STEPS else "REFUTED"),
            "consequence": ("the balance matters; the architect decides the next step"
                            if steps > SURVIVAL_THRESHOLD_STEPS else
                            "the on-policy over-weighting hypothesis is REFUTED and the single-round DAgger direction is closed"),
            "reference_points": {"rev4b_75.82pct": 42, "july_r1_35.23pct": 45, "july_r2": 356, "S1A_init": 392}}
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
        raise Rev4cError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    pre = preflight()
    if OUT_CAND.exists():
        raise FileExistsError(f"no-clobber: {OUT_CAND} exists")
    agg = build_aggregate(pre)
    if agg["observations"].shape[0] != EXPECTED_AGGREGATE:
        raise Rev4cError("aggregate row count mismatch")
    vec, scale_table = G.scale_vector()
    init_raw = pre["_init"]
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, np.asarray(pre["_view"]["obs"], dtype=np.float32), vec)
    if t1 > G.PRESERVATION_TOL:
        raise Rev4cError(f"prefit T1 FAILED {t1:.3e}")
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
                    "actor_label": ACTOR_LABEL, "experiment": "on-policy balance restored to the July 35.23% share",
                    "offline_verdict": ("PASS" if rep["all_binding_pass"] else "FAIL"),
                    "status": "INTERMEDIATE CANDIDATE (rev4c); never promoted automatically"}
        assert_no_foreign_labels(manifest, "manifest")
        A.assert_no_deployable_marking(manifest, "manifest")
        C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        receipt = {"schema": "v26b_rev4c_fit.1", "amendment_rev4c": PIN_AMENDMENT_REV4C,
                   "authorized_stage": AUTHORIZED_STAGE, "actor_label": ACTOR_LABEL,
                   "deployable": False, "sigma_unresolved": True, "lineage": lineage,
                   "framing": _amendment()["ARCHITECTURAL_FRAMING_BINDING"],
                   "execution_incident_disclosed": INCIDENT_RECORD,
                   "init": {"module": C.rel(B4.S1A_MODULE), "actor_digest": B4.PIN_S1A_ACTOR,
                            "role": "S1A is the sole init, the anchor parameter and the frozen collection policy"},
                   "preflight": {k: v for k, v in pre.items() if not k.startswith("_")},
                   "dataset": {"teacher_corpus_rows": int(len(pre["_view"]["obs"])),
                               "prefix_rows": PREFIX_CAP, "trace_repeat": B4.J_TRACE_REPEAT,
                               "aggregate_rows": int(agg["observations"].shape[0]),
                               "on_policy_share": float(PREFIX_CAP * B4.J_TRACE_REPEAT / agg["observations"].shape[0]),
                               "operation": "concat(teacher, tile(capped prefix, 4)); the cap is a collection budget applied before the July operator"},
                   "july_protocol": {k: v for k, v in frep.items() if k != "history"},
                   "loss_history_first_last": {"first": frep["history"][0], "last": frep["history"][-1]},
                   "loss_history_full": frep["history"],
                   "scaling": {"table": scale_table, "T1_maxabs": t1},
                   "offline": rep, "save_reload_exact": exact,
                   "output_module": C.rel(OUT_CAND / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "one balance experiment; no PPO, no further round, no sigma, no other candidate, no promotion",
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
    parser = argparse.ArgumentParser(description="V26B rev4c on-policy balance experiment")
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
                      "primary_causal_test": r.get("primary_causal_test"),
                      "seven_gates_failed": (r.get("seven_gates") or {}).get("failed")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
