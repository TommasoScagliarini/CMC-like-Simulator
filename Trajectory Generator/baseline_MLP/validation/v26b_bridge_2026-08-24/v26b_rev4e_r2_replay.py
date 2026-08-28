"""V26B rev4e - replay of the JULY R2 PROTOCOL on the current V26 lineage (no July artifact used).

Token V26B-REV4E-R2-REPLAY.  Sequential DAgger round:
  init = anchor = the REV4C module (fit_july clones what it receives, so the anchor IS the init)
  cumulative traces = S1A[1..68] (exactly the rows REV4C consumed) + the ENTIRE REV4C rollout (116)
  interpolation_steps = 2 with July's exact semantics (2 points ADDED, alpha = i/(k+1), factor 3)
  trace_repeat = 1
  aggregate = 500 + 552 = 1052, on-policy share 552/1052 = 52.4715%

THIS IS A MULTIVARIATE PROTOCOL REPLAY, NOT A ONE-VARIABLE EXPERIMENT: against rev4c the init/anchor,
the cumulativity, the interpolation and the repeat all change together.  A success attributes causality
to NOTHING in particular; a failure closes the current R2-analogous direction.

Validation MSE is DIAGNOSTIC ONLY: the interpolated groups are distinct but correlated and straddle
the seeded random split.

July is BENCHMARK AND PROTOCOL REFERENCE ONLY: no July module, dataset, trace or label is ever used.
No PPO, no sigma, no further round, no other candidate, no retry, no promotion.
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

import v26b_rev4d_dagger as D4  # noqa: E402   (rev4d lineage; hygiene contract reused and extended)
import v26b_rev4c_dagger as C4  # noqa: E402   (rev4c lineage; parameterised analyzer)
import v26b_rev4b_dagger as B4  # noqa: E402   (July fit and preflight primitives; J_* constants)
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


class Rev4eError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-REV4E-R2-REPLAY"
AMENDMENT_REV4E = HERE / "v26b_amendment_rev4e_r2_replay.json"
PIN_AMENDMENT_REV4E = "6caf9b8b547d53ac6ce856de9339ab616a0d34191a7b33b1b4a82c531d38eb3f"
PIN_AMENDMENT_REV4D = D4.PIN_AMENDMENT_REV4D

# --- pinned inputs (all current-lineage; NO July artifact) -----------------------------------------
INIT_MODULE = VA.OUT_ROOT / "candidates" / "REV4C_BALANCE_35D_NONDEPLOYABLE" / "rl_module"
PIN_INIT_FILES = {"module_state.pkl": "dc6a7dab9fe202776501f3010b17ac37c45e7d087da3de8df0531bdd9f1c202e",
                  "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
                  "class_and_ctor_args.pkl": "c9a6722ff95642795bfe1146d0087a68b5861fd508cbe3692195b2d820d810a7",
                  "actor_feature_manifest.json": "b5a7745abb5cfc4c7886f68e2570fdc09e4bcc13bba4cad87a1e60008749e6cc"}
PIN_INIT_ACTOR_DIGEST = "35fef3042211d9fbf6abcc6277c5f82269c44e98b633da4f825dab3e3bb6b0e9"
TRACE2_JOB = VA.OUT_ROOT / "rollouts" / "rev4c_nominal_det" / "REV4C_35D__v3_canonical__nominal__det"
PIN_TRACE1_S1A = "6546befcd4a2e26711137a807cd43a797e47abd62500243fe6c015d7abcfcf21"
PIN_TRACE2_REV4C = "8f2310928b586bed5750122478bd30cd08beba34e47e318ee95b0c2044474f63"

TRACE1_ROWS = 68
TRACE2_ROWS = 116
RAW_VISITED = 184
INTERPOLATION_STEPS = 2
INTERP_FACTOR = 1 + INTERPOLATION_STEPS
INTERPOLATED_ROWS = 368
UNIQUE_DAGGER = 552
TRACE_REPEAT = 1
EXPECTED_AGGREGATE = 1052
EXPECTED_SHARE = 552 / 1052
EXPECTED_DISTINCT_NEGATIVE = 14
EXPECTED_NEGATIVE_WINDOWS = [[6, 14], [112, 116]]
DISCRETE_INDICES = (11, 12, 13, 17, 18, 19, 20, 21)
ACTOR_LABEL = "REV4E"
SURVIVAL_THRESHOLD_STEPS = 116
SECONDARY_MARKER_MIN_CYCLES = 1

OUT_CAND = VA.OUT_ROOT / "candidates" / "REV4E_R2REPLAY_35D_NONDEPLOYABLE"
JOB_DIR = VA.OUT_ROOT / "rollouts" / "rev4e_nominal_det" / "REV4E_35D__v3_canonical__nominal__det"
RECEIPT_NAME = "v26b_rev4e_receipt.json"
ROLLOUT_RECEIPT_NAME = "v26b_rev4e_rollout_receipt.json"
LOG_NAME = "rev4e_nominal_det_rollout.log"

JULY_PATH_MARKERS = ("2026-07", "target_domain_dagger_", "target_domain_imitation_2026")


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV4E)
    if got != PIN_AMENDMENT_REV4E:
        raise Rev4eError(f"rev4e sha {got} != pinned")
    return json.loads(AMENDMENT_REV4E.read_text(encoding="utf-8"))


def assert_no_july_artifact(paths: Sequence[Path | str]) -> list[str]:
    """Fail closed if any operational input path points at a July artifact."""
    resolved = []
    for p in paths:
        s = str(p)
        for mark in JULY_PATH_MARKERS:
            if mark in s:
                raise Rev4eError(f"JULY ARTIFACT used operationally: {s} (matches {mark!r})")
        resolved.append(s)
    return resolved


def verify_lineage() -> dict[str, Any]:
    lin = D4.verify_lineage()
    got = C.sha256_file(AMENDMENT_REV4E)
    if got != PIN_AMENDMENT_REV4E:
        raise Rev4eError(f"amendment_rev4e sha {got} != pinned")
    lin["amendment_rev4e"] = got
    for name, sha in PIN_INIT_FILES.items():
        real = C.sha256_file(INIT_MODULE / name)
        if real != sha:
            raise Rev4eError(f"init/anchor file {name} sha {real} != pinned {sha}")
    lin["init_anchor_module"] = {"path": C.rel(INIT_MODULE), "files_sha256": dict(PIN_INIT_FILES),
                                 "actor_digest": PIN_INIT_ACTOR_DIGEST,
                                 "role": "REV4C is the sole init AND, by construction of fit_july, the anchor"}
    for job, pin, key in ((SA.JOB_DIR, PIN_TRACE1_S1A, "trace_1_s1a"),
                          (TRACE2_JOB, PIN_TRACE2_REV4C, "trace_2_rev4c")):
        real = C.sha256_file(job / "rollout_policy_trace.json")
        if real != pin:
            raise Rev4eError(f"{key} sha {real} != pinned {pin}")
        lin[key] = real
    lin["july_artifacts_role"] = "benchmark and protocol reference only; never init, anchor or label"
    lin["declarations"] = _amendment()["MANDATORY_DECLARATIONS"]
    return lin


# --- hygiene: keys AND prose (inherited from rev4d, actor label re-parameterised) --------------------
FORBIDDEN_KEY_SUBSTRINGS = D4.FORBIDDEN_KEY_SUBSTRINGS
EXEMPT_SUBTREES = D4.EXEMPT_SUBTREES + ("declarations", "pinned_inputs", "init_anchor_module")
FORBIDDEN_VALUE_PATTERNS = (r"eligibility of (?!%s)[A-Z0-9_]+" % ACTOR_LABEL,
                            r"states (?!%s)(?:L\d+|A\d+|S1[AB]|REV\d[A-Z]?) actually visited" % ACTOR_LABEL)


def assert_no_foreign_labels(obj: Any, where: str = "receipt", *, exempt: bool = False) -> None:
    if isinstance(obj, dict):
        for k, v in obj.items():
            ks = str(k)
            if ks == "offline_reference_block":
                raise Rev4eError(f"{where}: foreign offline_reference_block is forbidden")
            if not exempt:
                for bad in FORBIDDEN_KEY_SUBSTRINGS:
                    if bad in ks:
                        raise Rev4eError(f"{where}: key {ks!r} carries a foreign actor label {bad!r}")
            assert_no_foreign_labels(v, f"{where}.{ks}", exempt=exempt or ks in EXEMPT_SUBTREES)
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_no_foreign_labels(v, f"{where}[{i}]", exempt=exempt)
    elif isinstance(obj, str) and not exempt:
        for pat in FORBIDDEN_VALUE_PATTERNS:
            m = re.search(pat, obj)
            if m:
                raise Rev4eError(f"{where}: prose carries a foreign actor label: {m.group(0)!r}")


def relabel_seven_gates(seven: Mapping[str, Any], label: str) -> dict[str, Any]:
    return D4.relabel_seven_gates(seven, label)


# --- July interpolation, exact semantics -------------------------------------------------------------

def july_interpolate(visited: np.ndarray, teacher: np.ndarray, k: int,
                     discrete_indices: Sequence[int] = DISCRETE_INDICES) -> list[np.ndarray]:
    """target_domain_imitation.py:665-672 verbatim: k points ADDED, alpha = i/(k+1), discrete pinned.
    Returns [raw, interp_1, ..., interp_k] so the row order matches July's local_observations."""
    if k < 0:
        raise Rev4eError("interpolation_steps must be >= 0")
    obs = np.asarray(visited, dtype=np.float32).reshape(-1)
    tea = np.asarray(teacher, dtype=np.float32).reshape(-1)
    out = [obs]
    for i in range(1, k + 1):
        alpha = i / float(k + 1)
        interp = tea + alpha * (obs - tea)
        if len(discrete_indices):
            interp[list(discrete_indices)] = obs[list(discrete_indices)]
        out.append(interp.astype(np.float32))
    return out


def _load_init_state() -> dict[str, np.ndarray]:
    with (INIT_MODULE / "module_state.pkl").open("rb") as fh:
        state = pickle.load(fh)
    raw = {k: np.asarray(v) for k, v in state.items()}
    if tuple(raw.keys()) != RF.EXPECTED_KEY_ORDER:
        raise Rev4eError("init module does not carry the 10 expected keys in order")
    val = RF.validate_init_state(raw, expected_actor_digest=None)
    if val["actor_digest"] != PIN_INIT_ACTOR_DIGEST:
        raise Rev4eError(f"init actor digest {val['actor_digest']} != pinned")
    if not val["clock_columns_zero"]:
        raise Rev4eError("init clock columns are not zero")
    return raw


def _trace_rows(job: Path, n: int, label: str) -> tuple[np.ndarray, list[int]]:
    rows = json.loads((job / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    if len(rows) < n:
        raise Rev4eError(f"{label}: recorded trace has {len(rows)} rows < {n}")
    rows = rows[:n]
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, n + 1)):
        raise Rev4eError(f"{label}: steps 1..{n} are not contiguous")
    obs = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float32)
    if obs.shape != (n, R.ENV_ACTOR_WIDTH):
        raise Rev4eError(f"{label}: obs shape {obs.shape} != {(n, R.ENV_ACTOR_WIDTH)}")
    if not np.all(np.isfinite(obs)):
        raise Rev4eError(f"{label}: non-finite obs35")
    return obs, steps


def preflight() -> dict[str, Any]:
    """No-write, fail-closed preflight on the full July-R2-analogous corpus."""
    _amendment()
    base = B4.preflight()
    view = base["_view"]
    obsT = np.asarray(view["obs"], dtype=np.float32)
    uT = np.asarray(view["u_ik"], dtype=np.float64)
    if len(obsT) != 500:
        raise Rev4eError(f"teacher corpus is {len(obsT)} rows, expected 500")
    assert_no_july_artifact([INIT_MODULE, SA.JOB_DIR, TRACE2_JOB, SR.JOB_DIR, R.OUT_CACHE,
                             F1.RUNTIME_CONFIG, F1.ROLLOUT_EVAL])
    init_raw = _load_init_state()

    obs1, _ = _trace_rows(SA.JOB_DIR, TRACE1_ROWS, "trace_1_S1A")
    obs2, _ = _trace_rows(TRACE2_JOB, TRACE2_ROWS, "trace_2_REV4C")
    # trace 1 must be bit-identical to the rows rev4b/rev4c audited
    if not np.array_equal(obs1, np.asarray(base["_obsS"], dtype=np.float32)[:TRACE1_ROWS]):
        raise Rev4eError("trace 1 differs from the audited S1A prefix rows")

    tT = np.asarray(DS.trajectory_from_job(SR.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)
    dt = {}
    for job, n, key in ((SA.JOB_DIR, TRACE1_ROWS, "trace_1_s1a"), (TRACE2_JOB, TRACE2_ROWS, "trace_2_rev4c")):
        tX = np.asarray(DS.trajectory_from_job(job, expected_width=R.ENV_ACTOR_WIDTH)["t_pre"], dtype=np.float64)[:n]
        d = float(np.abs(tX - tT[:n]).max())
        if d != 0.0:
            raise Rev4eError(f"time alignment broken on {key}: {d}")
        dt[key] = d

    # --- build the visited pool with July's exact expansion and row order -------------------------
    pool_obs: list[np.ndarray] = []
    pool_idx: list[int] = []
    pool_kind: list[str] = []
    raw_count = 0
    for tname, arr in (("trace_1_s1a", obs1), ("trace_2_rev4c", obs2)):
        for i in range(len(arr)):
            local = july_interpolate(arr[i], obsT[i], INTERPOLATION_STEPS)
            raw_count += 1
            for j, o in enumerate(local):
                pool_obs.append(o); pool_idx.append(i)
                pool_kind.append("raw" if j == 0 else f"interp_{j}")
    if raw_count != RAW_VISITED:
        raise Rev4eError(f"raw visited rows {raw_count} != {RAW_VISITED}")
    n_interp = sum(1 for k in pool_kind if k != "raw")
    if n_interp != INTERPOLATED_ROWS:
        raise Rev4eError(f"interpolated rows {n_interp} != {INTERPOLATED_ROWS}")
    if len(pool_obs) != UNIQUE_DAGGER:
        raise Rev4eError(f"unique DAgger rows {len(pool_obs)} != {UNIQUE_DAGGER}")
    P = np.asarray(pool_obs, dtype=np.float32)
    labels = np.asarray([uT[i] for i in pool_idx], dtype=np.float64)
    if not np.all(np.isfinite(P)) or not np.all(np.isfinite(labels)):
        raise Rev4eError("non-finite pool rows or labels")

    # --- verify the interpolation properties row by row -------------------------------------------
    disc = list(DISCRETE_INDICES)
    cont = [c for c in range(R.ENV_ACTOR_WIDTH) if c not in DISCRETE_INDICES]
    checked = 0
    for p in range(0, len(pool_obs), INTERP_FACTOR):
        rawrow = P[p]; ti = pool_idx[p]
        for j in range(1, INTERP_FACTOR):
            child = P[p + j]
            if pool_idx[p + j] != ti:
                raise Rev4eError("interpolated child carries a different teacher index")
            if not np.array_equal(child[disc], rawrow[disc]):
                raise Rev4eError(f"discrete columns not pinned on child {j} of pool row {p}")
            alpha = j / float(INTERP_FACTOR)
            want = (obsT[ti].astype(np.float32) + np.float32(alpha) * (rawrow - obsT[ti].astype(np.float32)))
            want[disc] = rawrow[disc]
            if not np.array_equal(child[cont], want[cont]):
                raise Rev4eError(f"continuous columns off the teacher-visited segment on child {j}")
            checked += 1

    # --- duplicates, collisions, label conflicts ---------------------------------------------------
    keys = [P[i].tobytes() for i in range(len(P))]
    seen: dict[bytes, int] = {}
    dup = dup_conf = 0
    for k, ti in zip(keys, pool_idx):
        if k in seen:
            dup += 1
            if seen[k] != ti:
                dup_conf += 1
        else:
            seen[k] = ti
    mT: dict[bytes, int] = {}
    for i in range(len(obsT)):
        mT.setdefault(obsT[i].tobytes(), i)
    coll = [(i, mT[keys[i]]) for i in range(len(keys)) if keys[i] in mT]
    coll_conf = [c for c in coll if not np.array_equal(uT[pool_idx[c[0]]], uT[c[1]])]
    if dup_conf or coll_conf:
        raise Rev4eError(f"CONFLICTING labels: {dup_conf} duplicates, {len(coll_conf)} collisions: NO-GO")

    # --- negative coverage -------------------------------------------------------------------------
    negT = SC.decode_action(uT)[:, 1] < 0.0
    neg_idx = sorted({i for i in pool_idx if negT[i]})
    wins, s, p_ = [], None, None
    for i in neg_idx:
        if s is None:
            s = p_ = i; continue
        if i == p_ + 1:
            p_ = i
        else:
            wins.append([s + 1, p_ + 1]); s = p_ = i
    if s is not None:
        wins.append([s + 1, p_ + 1])
    if len(neg_idx) != EXPECTED_DISTINCT_NEGATIVE or wins != EXPECTED_NEGATIVE_WINDOWS:
        raise Rev4eError(f"negative coverage {len(neg_idx)}/{wins} != {EXPECTED_DISTINCT_NEGATIVE}/{EXPECTED_NEGATIVE_WINDOWS}")

    agg_rows = len(obsT) + len(P) * TRACE_REPEAT
    if agg_rows != EXPECTED_AGGREGATE:
        raise Rev4eError(f"aggregate {agg_rows} != {EXPECTED_AGGREGATE}")
    share = len(P) * TRACE_REPEAT / agg_rows
    if share != EXPECTED_SHARE:
        raise Rev4eError(f"share {share!r} != {EXPECTED_SHARE!r}")

    return {"verdict": "GO",
            "init_anchor": {"path": C.rel(INIT_MODULE), "actor_digest": PIN_INIT_ACTOR_DIGEST,
                            "files_sha256": dict(PIN_INIT_FILES), "ten_keys": True, "clock_columns_zero": True,
                            "anchor_is_the_init": "fit_july clones the parameters it receives"},
            "teacher_corpus_rows": int(len(obsT)),
            "traces": [{"index": 1, "source": "S1A rows 1..68 actually consumed by the parent round",
                        "rows": TRACE1_ROWS, "sha256": PIN_TRACE1_S1A, "steps_contiguous": True,
                        "obs35_matches_recorded_vectors": True},
                       {"index": 2, "source": "the entire parent rollout as executed",
                        "rows": TRACE2_ROWS, "sha256": PIN_TRACE2_REV4C, "steps_contiguous": True,
                        "obs35_matches_recorded_vectors": True}],
            "time_alignment_max_abs_difference_s": dt,
            "interpolation": {"steps_added": INTERPOLATION_STEPS, "total_factor": INTERP_FACTOR,
                              "alphas": [round(j / INTERP_FACTOR, 12) for j in range(1, INTERP_FACTOR)],
                              "discrete_indices_pinned": list(DISCRETE_INDICES),
                              "children_verified": checked,
                              "semantics": "July exact: k points ADDED, alpha = i/(k+1), discrete pinned to the visited row"},
            "composition": {"raw_visited_rows": raw_count, "interpolated_rows": n_interp,
                            "unique_dagger_rows": int(len(P)), "trace_repeat": TRACE_REPEAT,
                            "on_policy_rows": int(len(P) * TRACE_REPEAT),
                            "teacher_corpus_rows": int(len(obsT)),
                            "aggregate_rows": agg_rows,
                            "formula": f"{len(obsT)} + ({RAW_VISITED} x {INTERP_FACTOR}) x {TRACE_REPEAT} = {agg_rows}",
                            "on_policy_share": share,
                            "on_policy_share_exact_fraction": f"{len(P) * TRACE_REPEAT}/{agg_rows}"},
            "labels": {"rule": "same-step u_IK at teacher_index = step - 1; a raw row and its interpolated children share the label",
                       "rows": int(len(labels)),
                       "sha256": __import__("hashlib").sha256(labels.tobytes()).hexdigest()},
            "duplicates_in_pool": dup, "duplicates_with_conflicting_labels": dup_conf,
            "bitwise_collisions_with_corpus": len(coll), "collisions_with_conflicting_labels": len(coll_conf),
            "collision_origin": "episode-reset rows where the visited state is bit-identical to the teacher state, so the interpolated children collapse onto the same point",
            "negative_ankle_coverage": {"distinct_negative_time_indices": len(neg_idx), "windows": wins,
                                        "rows_in_full_corpus": int(negT.sum()),
                                        "second_window_is_partial": "the corpus window is 112-132; only 112-116 is covered"},
            "no_july_artifact_operational": True,
            "_view": view, "_pool": P, "_labels": labels, "_idx": pool_idx, "_kind": pool_kind,
            "_init": init_raw, "_obs1": obs1, "_obs2": obs2}


def build_aggregate(pre: Mapping[str, Any]) -> dict[str, np.ndarray]:
    """concat(teacher, tile(pool, TRACE_REPEAT)) with July's row order."""
    view = pre["_view"]
    obs_t = np.asarray(view["obs"], dtype=np.float32)
    act_t = np.asarray(view["u_ik"], dtype=np.float32)
    obs_v = np.asarray(pre["_pool"], dtype=np.float32)
    act_v = np.asarray(pre["_labels"], dtype=np.float32)
    return {"observations": np.concatenate([obs_t, np.tile(obs_v, (TRACE_REPEAT, 1))], axis=0),
            "actions": np.concatenate([act_t, np.tile(act_v, (TRACE_REPEAT, 1))], axis=0)}


def offline_report(init_raw, export_raw, scaled, pre, vec, t1: float, save_reload: bool) -> dict[str, Any]:
    f32 = np.float32
    view = pre["_view"]
    obsT = np.asarray(view["obs"], dtype=np.float32); uT = np.asarray(view["u_ik"], dtype=np.float64)
    pool = np.asarray(pre["_pool"], dtype=np.float32); labP = np.asarray(pre["_labels"], dtype=np.float64)
    obs1 = np.asarray(pre["_obs1"], dtype=np.float32); obs2 = np.asarray(pre["_obs2"], dtype=np.float32)
    negT = SC.decode_action(uT)[:, 1] < 0.0
    mT = RF.numpy_mean(export_raw, obsT); iT = RF.numpy_mean(init_raw, obsT)
    obs_all = np.concatenate([obsT, pool]); tgt_all = np.concatenate([uT, labP])
    m_all = RF.numpy_mean(export_raw, obs_all); i_all = RF.numpy_mean(init_raw, obs_all)
    agg_before = float(np.sqrt(np.mean((i_all - tgt_all) ** 2)))
    agg_after = float(np.sqrt(np.mean((m_all - tgt_all) ** 2)))
    t2 = float(np.max(np.abs(mT - RF.numpy_mean(scaled, (obsT.astype(np.float64) / vec[None, :]).astype(f32)))))
    struct = RF.validate_init_state(export_raw, expected_actor_digest=None)
    inv = RF.invariance_test(export_raw, obsT[:64])
    integrity = {"ten_keys": tuple(export_raw.keys()) == RF.EXPECTED_KEY_ORDER,
                 "clock_columns_zero": bool(struct["clock_columns_zero"]),
                 "clock_invariance_bit_identical": bool(inv["bit_identical"]),
                 "logstd_byte_identical_to_init": bool(np.array_equal(np.asarray(export_raw["pi.1.weight"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.weight"])[R.ACTION_DIM:])
                                                       and np.array_equal(np.asarray(export_raw["pi.1.bias"])[R.ACTION_DIM:], np.asarray(init_raw["pi.1.bias"])[R.ACTION_DIM:])),
                 "save_reload_exact": bool(save_reload), "no_critic": True}
    binding = {"integrity_invariants": {"binding": True, **integrity, "pass": bool(all(integrity.values()))},
               "function_preservation": {"binding": True, "T1_maxabs": float(t1), "T2_maxabs": t2,
                                         "tol": G.PRESERVATION_TOL,
                                         "pass": bool(t1 <= G.PRESERVATION_TOL and t2 <= G.PRESERVATION_TOL)},
               "fit_convergence": {"binding": True, "aggregate_rmse_before": agg_before,
                                   "aggregate_rmse_after": agg_after, "pass": bool(agg_after < agg_before)}}

    def shape(pred, ref):
        out = {}
        for j, jn in ((0, "knee"), (1, "ankle")):
            x, y = pred[:, j], ref[:, j]
            sx, sy = float(x.std()), float(y.std())
            out[jn] = {"rmse": float(np.sqrt(np.mean((x - y) ** 2))),
                       "pearson_r": (float(np.corrcoef(x, y)[0, 1]) if sx > 0 and sy > 0 else None),
                       "amplitude_ratio": (sx / sy if sy > 0 else None)}
        return out

    def band(obs, lo, hi):
        o = obs[lo:hi]
        return shape(RF.numpy_mean(export_raw, o), uT[lo:hi])

    s0d = B4._s0d_state()
    d_s0d = mT - RF.numpy_mean(s0d, obsT)
    h = np.asarray(L.load_cache(R.OUT_CACHE, "nominal").targets)[:, [0, 2]]
    qh = SC.decode_action(mT); q_cmd = qh[:, 1]
    measures = {"note": "reported without invented thresholds; promotion is left to the architect audit",
                "on_corpus_500": shape(mT, uT),
                "on_trace1_s1a_rows_1_68": band(obs1, 0, 68),
                "on_trace2_rows_1_68": band(obs2, 0, 68),
                "on_trace2_rows_69_116": band(obs2, 68, 116),
                "on_all_97_negative_rows_of_the_corpus": shape(mT[negT], uT[negT]),
                "on_full_interpolated_pool_552": shape(RF.numpy_mean(export_raw, pool), labP),
                "negative_window_sign": {"rows": int(negT.sum()),
                                         "fraction_positive_command_before": float(np.mean(SC.decode_action(iT)[negT, 1] > 0.0)),
                                         "fraction_positive_command_after": float(np.mean(q_cmd[negT] > 0.0)),
                                         "min_commanded_ankle": float(q_cmd[negT].min())},
                "max_abs_vs_source_actor_INFORMATIONAL": {"max_abs_per_joint": [float(np.abs(d_s0d[:, j]).max()) for j in (0, 1)],
                                                          "mean_abs_per_joint": [float(np.abs(d_s0d[:, j]).mean()) for j in (0, 1)],
                                                          "status": "INFORMATIONAL only"},
                "healthy_symmetry_DIAGNOSTIC": {"rmse_per_joint": [float(np.sqrt(np.mean((qh[:, j] - h[:, j]) ** 2))) for j in (0, 1)],
                                                "status": "DIAGNOSTIC only"}}
    failed = [k for k, v in binding.items() if not v["pass"]]
    return {"binding": binding, "failed": failed, "all_binding_pass": bool(not failed),
            "measures": measures, "actor_digest": struct["actor_digest"], "structure": struct}


# --- closed loop --------------------------------------------------------------------------------------

def rollout_command(python_exe: str = SB.PYTHON_EXE) -> list[str]:
    return [python_exe, str(F1.ROLLOUT_EVAL), "--checkpoint", str(OUT_CAND / "rl_module"),
            "--no-auto-config", "--config", str(F1.RUNTIME_CONFIG),
            "--episode-start-offset-s", repr(float(R.EXACT_STARTS["nominal"])),
            "--action-selection", "deterministic", "--seed", str(R.DET_SEED),
            "--output-dir", str(JOB_DIR), "--record-outputs", "--record-policy-trace",
            *[str(a) for a in F1.JOB_TIMEOUT_ARGS], *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]]]


def run_closed_loop(cand_receipt_sha: str, python_exe: str = SB.PYTHON_EXE) -> dict[str, Any]:
    if JOB_DIR.exists():
        raise Rev4eError(f"no-clobber: {JOB_DIR} exists (exactly one launch; no retry)")
    cmd = rollout_command(python_exe)
    JOB_DIR.mkdir(parents=True, exist_ok=False)
    VA.OUT_LOGS.mkdir(parents=True, exist_ok=True)
    log_path = VA.OUT_LOGS / LOG_NAME
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n"); log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(R.BASELINE_DIR))
    rec: dict[str, Any] = {"schema": "v26b_rev4e_rollout.1", "authorized_stage": AUTHORIZED_STAGE,
                           "amendment_rev4e": PIN_AMENDMENT_REV4E, "actor_label": ACTOR_LABEL,
                           "candidate_receipt_sha256": cand_receipt_sha, "deployable": False,
                           "sigma": "UNRESOLVED; deterministic mean only", "command": cmd,
                           "returncode": int(proc.returncode), "duration_s": round(time.time() - t0, 3),
                           "log": C.rel(log_path), "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    status = "FAILED"
    try:
        if proc.returncode != 0:
            raise Rev4eError(f"rollout returncode {proc.returncode} (log {log_path})")
        analysis = RO.analyse_rollout(JOB_DIR)
        rows = json.loads((JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        rowscan = R1.counter_rowscan(rows)
        seven = relabel_seven_gates(SB.eligibility_gates(analysis, rowscan), ACTOR_LABEL)
        steps = int(analysis["completion"]["steps"])
        cycles = int(analysis["counters"]["valid_cycle_count"])
        rec["analysis"] = analysis
        rec["fsm_counters_rowscan"] = rowscan
        rec["seven_gates"] = seven
        rec["diagnostics"] = {"action_stats_whole_trace": R1.action_stats(rows),
                              "b3_phase_window": PA.b3_late_stance(rows),
                              "actor_comparison": C4.actor_comparison(JOB_DIR, ACTOR_LABEL)}
        rec["primary_gate"] = {"measure": "survival in steps", "observed_steps": steps,
                               "threshold": SURVIVAL_THRESHOLD_STEPS, "rule": "STRICTLY GREATER THAN 116",
                               "verdict": ("PASS" if steps > SURVIVAL_THRESHOLD_STEPS else "FAIL"),
                               "reference_points": _amendment()["CRITERIA"]["primary_gate"]["reference_points"]}
        rec["secondary_marker"] = {"measure": "valid_cycle_count", "observed": cycles,
                                   "rule": f">= {SECONDARY_MARKER_MIN_CYCLES}",
                                   "verdict": ("PASS" if cycles >= SECONDARY_MARKER_MIN_CYCLES else "FAIL"),
                                   "status": "declared marker; it does not by itself promote"}
        rec["attribution_limits"] = _amendment()["MANDATORY_DECLARATIONS"]["b_attribution_limits"]
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
        raise Rev4eError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage()
    pre = preflight()
    if OUT_CAND.exists():
        raise FileExistsError(f"no-clobber: {OUT_CAND} exists")
    agg = build_aggregate(pre)
    if agg["observations"].shape[0] != EXPECTED_AGGREGATE:
        raise Rev4eError("aggregate row count mismatch")
    vec, scale_table = G.scale_vector()
    init_raw = pre["_init"]
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, np.asarray(pre["_view"]["obs"], dtype=np.float32), vec)
    if t1 > G.PRESERVATION_TOL:
        raise Rev4eError(f"prefit T1 FAILED {t1:.3e}")
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
        shutil.copy2(INIT_MODULE / "metadata.json", sm / "metadata.json")
        shutil.copy2(INIT_MODULE / "class_and_ctor_args.pkl", sm / "class_and_ctor_args.pkl")
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
                    "derived_from": C.rel(INIT_MODULE), "source_actor_digest": PIN_INIT_ACTOR_DIGEST,
                    "contract": A.CONTRACT_STRING, "deployable": False, "sigma_unresolved": True,
                    "actor_label": ACTOR_LABEL,
                    "experiment": "sequential DAgger round replaying the July R2 protocol on the current lineage",
                    "offline_verdict": ("PASS" if rep["all_binding_pass"] else "FAIL"),
                    "status": "INTERMEDIATE CANDIDATE (rev4e); never promoted automatically"}
        assert_no_foreign_labels(manifest, "manifest")
        A.assert_no_deployable_marking(manifest, "manifest")
        C.write_json(sm / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        receipt = {"schema": "v26b_rev4e_fit.1", "amendment_rev4e": PIN_AMENDMENT_REV4E,
                   "authorized_stage": AUTHORIZED_STAGE, "actor_label": ACTOR_LABEL,
                   "deployable": False, "sigma_unresolved": True, "lineage": lineage,
                   "declarations": _amendment()["MANDATORY_DECLARATIONS"],
                   "init_anchor_module": {"path": C.rel(INIT_MODULE), "actor_digest": PIN_INIT_ACTOR_DIGEST,
                                          "files_sha256": dict(PIN_INIT_FILES),
                                          "role": "sole init AND anchor; fit_july clones what it receives"},
                   "preflight": {k: v for k, v in pre.items() if not k.startswith("_")},
                   "dataset": {"teacher_corpus_rows": int(len(pre["_view"]["obs"])),
                               "trace_rows": [TRACE1_ROWS, TRACE2_ROWS],
                               "raw_visited_rows": RAW_VISITED, "interpolation_steps": INTERPOLATION_STEPS,
                               "interpolated_rows": INTERPOLATED_ROWS, "unique_dagger_rows": UNIQUE_DAGGER,
                               "trace_repeat": TRACE_REPEAT,
                               "aggregate_rows": int(agg["observations"].shape[0]),
                               "on_policy_share": float(UNIQUE_DAGGER * TRACE_REPEAT / agg["observations"].shape[0]),
                               "operation": "concat(teacher, tile(pool, 1)); pool = per row [raw, interp 1/3, interp 2/3]"},
                   "july_protocol": {k: v for k, v in frep.items() if k != "history"},
                   "loss_history_first_last": {"first": frep["history"][0], "last": frep["history"][-1]},
                   "loss_history_full": frep["history"],
                   "validation_note": "DIAGNOSTIC ONLY: interpolated groups are distinct but correlated and straddle the seeded split",
                   "scaling": {"table": scale_table, "T1_maxabs": t1},
                   "offline": rep, "save_reload_exact": exact,
                   "output_module": C.rel(OUT_CAND / "rl_module"),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "one July-R2-protocol replay; no PPO, no sigma, no further round, no other candidate, no promotion",
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
    parser = argparse.ArgumentParser(description="V26B rev4e July-R2-protocol replay")
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
                      "rollout_status": r.get("status"), "primary_gate": r.get("primary_gate"),
                      "secondary_marker": r.get("secondary_marker"),
                      "seven_gates_failed": (r.get("seven_gates") or {}).get("failed")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
