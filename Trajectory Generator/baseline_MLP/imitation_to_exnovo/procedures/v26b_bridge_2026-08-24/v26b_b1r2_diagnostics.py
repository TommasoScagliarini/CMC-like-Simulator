"""V26B B1R2 - diagnostic study. It materialises NO candidate, promotes nothing, and never
rolls out, collects or enters the Markov phase.

Four questions, in order:

  1. RECONSTRUCTION CURVE.  Restarting byte-identical from B0 and fitting all 1500 rows, does ANY
     epoch in 1..400 satisfy the reconstruction gates?  Measured per epoch on the global set, on the
     two B1R1-coverage groups (910 covered, 403 B1R1-uncovered startup/tails), on the two STRUCTURAL
     groups (1218 structurally complete cycle rows, 95 tail rows) and on the 187 WAIT rows.
     The instrumented fit is a bit-identical twin of v26b_b_exec.fit_masked; the test proves it.

  2. FULL VALIDATION COVERAGE.  A design that adds the five segments B1R1 left uncovered to the
     six leave-one-complete-cycle-out folds, so that EVERY non-WAIT row is held out exactly once,
     with embargo 20, train-only scaling, the effective 23D support and exact discrete signatures.
     Two of those five are STRUCTURALLY COMPLETE cycles that merely start at the episode origin, so
     the design is semantically EIGHT complete cycles plus THREE tails.  Quantified only - the
     corrective final fit is NOT run here.

  3. SUPPORT SPARSITY AND LOCAL SMOOTHNESS.  Nearest-neighbour target disagreement in the effective
     23D support, plus a genuine COLLISION AUDIT at distances 1e-8 to 1e-3.  This is a diagnostic of
     how sparse the support is and how smooth the target is locally.  It is NOT an information
     floor: a neighbour at normalised distance 0.4 is not a collision, and nothing here bounds what
     a function of the support can achieve.

  4. LABEL AND ACTION AUDIT.  Whether any label-lookup or action-semantics defect explains the
     error peaks.

Residual predictability, where reported, is an OUT-OF-FOLD ridge estimate on segment-blocked folds.
In-sample fits on small row clusters are excluded from every conclusion: with 23 regressors and a
few dozen rows they are not evidence.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_b_exec as EX  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import v26b_b1r1_exec as R1  # noqa: E402
import v26b_b1r1_loco_study as LO  # noqa: E402
import v26b_b1r1_split_study as SS  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class B1R2Error(RuntimeError):
    pass


STAGE = "V26B-B1R2-DIAGNOSTICS"
TRAJECTORY_NAMES = ("minus020", "nominal", "plus020")
EPOCHS = R1.EPOCHS_MAX
EMBARGO_ROWS = LO.EMBARGO_ROWS

# reconstruction gates, unchanged and not relaxed
GATES = {"aggregate_rmse_max": R1.FINAL_RMSE_MAX,
         "per_joint_rmse_max": R1.FINAL_PER_JOINT_RMSE_MAX,
         "per_joint_max_abs_max": R1.FINAL_PER_JOINT_MAXABS_MAX}

OUT_DIR = VA.OUT_ROOT / "diagnostics" / "b1r2"
# additive, no clobber: the pre-revision receipt v26b_b1r2_diagnostics.json is preserved untouched
RECEIPT_NAME = "v26b_b1r2_diagnostics_rev1.json"
SUPERSEDED_RECEIPT = "v26b_b1r2_diagnostics.json"


# ================================================================ row groups =====================

STANCE_COLUMN, SWING_COLUMN = 18, 19


def _segment_structure(raw: np.ndarray, traj: np.ndarray, a: int, b: int) -> dict[str, Any]:
    """Classify one contiguous segment as a STRUCTURALLY COMPLETE CYCLE or a TAIL.

    The distinction is derived from the FSM columns, never hardcoded.  A segment is a complete
    cycle when it traverses a full stance and then a full swing AND is terminated by the next
    heel-strike - that is, the row immediately after it exists in the same trajectory and is in
    stance.  It is a tail when it stops because the episode stops, so its last phase is truncated.

    A segment starting at the episode origin is therefore complete whenever the episode began in
    stance and the whole stance-swing sequence closes on an observed heel-strike.  It is NOT
    'partial': the only thing missing is the heel-strike BEFORE it, which no episode can observe.
    """
    sub = raw[a: b + 1]
    stance = sub[:, STANCE_COLUMN] > 0.5
    swing = sub[:, SWING_COLUMN] > 0.5
    phases: list[str] = []
    for st, sw in zip(stance, swing):
        lab = "stance" if st else ("swing" if sw else "other")
        if not phases or phases[-1] != lab:
            phases.append(lab)
    nxt = b + 1
    closed_by_heel_strike = bool(nxt < len(raw) and traj[nxt] == traj[a]
                                 and raw[nxt, STANCE_COLUMN] > 0.5)
    ends_at_episode_end = bool(nxt >= len(raw) or traj[nxt] != traj[a])
    complete = bool(phases == ["stance", "swing"] and closed_by_heel_strike)
    return {"phase_sequence": phases, "closed_by_heel_strike": closed_by_heel_strike,
            "ends_at_episode_end": ends_at_episode_end,
            "structure": "complete_cycle" if complete else "tail",
            "reason": ("full stance then full swing, closed by the next heel-strike"
                       if complete else
                       "truncated by the end of the episode, the last phase never closes")}


def row_groups() -> dict[str, Any]:
    """WAIT, the six cycles B1R1 held out, and the five segments B1R1 left UNCOVERED.

    The uncovered group is named for what it is - B1R1-uncovered startup/tails - not for a
    structural property it does not have.  Two of its five segments are structurally COMPLETE
    cycles; only three are genuine tails.  Both partitions are returned:

      * by B1R1 coverage : b1r1_covered (910) / b1r1_uncovered (403) / wait (187)
      * by structure     : structurally_complete (1218) / tails (95) / wait (187)
    """
    inp = R1.build_inputs()
    raw, traj = inp["raw"], inp["traj"]
    wait = set(int(i) for i in inp["wait_rows"])
    cycle = set()
    for f in inp["split"]["folds"]:
        cycle.update(int(i) for i in f["_val"])
    non_wait = [i for i in range(len(raw)) if i not in wait]
    rest = sorted(set(non_wait) - cycle)
    runs: list[tuple[int, int]] = []
    s = p = rest[0]
    for i in rest[1:]:
        if i == p + 1 and traj[i] == traj[p]:
            p = i
        else:
            runs.append((s, p)); s = p = i
    runs.append((s, p))
    uncovered = []
    for a, b in runs:
        t = int(traj[a]); off = int(np.searchsorted(traj, t))
        seg = {"trajectory": t, "name": TRAJECTORY_NAMES[t],
               "local_lo": a - off, "local_hi": b - off, "rows": b - a + 1,
               "position": "head" if a - off == 0 else "tail",
               "global_lo": int(a), "global_hi": int(b)}
        seg.update(_segment_structure(raw, traj, a, b))
        uncovered.append(seg)
    if len(uncovered) != 5 or sum(p["rows"] for p in uncovered) != 403:
        raise B1R2Error(f"expected five uncovered segments of 403 rows, got {len(uncovered)} of "
                        f"{sum(p['rows'] for p in uncovered)}")
    if len(wait) != 187 or len(cycle) != 910:
        raise B1R2Error(f"expected 187 WAIT and 910 cycle rows, got {len(wait)} and {len(cycle)}")
    heads = [s for s in uncovered if s["structure"] == "complete_cycle"]
    tails = [s for s in uncovered if s["structure"] == "tail"]
    if len(heads) != 2 or len(tails) != 3:
        raise B1R2Error(f"expected two structurally complete uncovered cycles and three tails, "
                        f"got {len(heads)} and {len(tails)}")
    tail_rows = sorted(i for s in tails for i in range(s["global_lo"], s["global_hi"] + 1))
    complete_rows = sorted(set(cycle) | {i for s in heads
                                         for i in range(s["global_lo"], s["global_hi"] + 1)})
    return {"inp": inp, "wait": np.array(sorted(wait), dtype=int),
            "cycle": np.array(sorted(cycle), dtype=int),
            "uncovered": np.array(rest, dtype=int), "uncovered_segments": uncovered,
            "structurally_complete": np.array(complete_rows, dtype=int),
            "tails": np.array(tail_rows, dtype=int),
            "structure_summary": {
                "complete_cycles_total": len(inp["split"]["folds"]) + len(heads),
                "tails_total": len(tails),
                "structurally_complete_rows": len(complete_rows),
                "tail_rows": len(tail_rows),
                "semantics": (f"{len(inp['split']['folds'])} B1R1 cycles + {len(heads)} uncovered "
                              f"complete cycles = {len(inp['split']['folds']) + len(heads)} complete "
                              f"cycles, plus {len(tails)} tails")}}


# ================================================================ 1. reconstruction curve ========

def fit_all_instrumented(init: Mapping[str, np.ndarray], obs: np.ndarray, act: np.ndarray,
                         *, epochs: int, on_epoch: Callable[[int, dict[str, np.ndarray]], None],
                         progress: bool = False) -> dict[str, np.ndarray]:
    """Bit-identical twin of v26b_b_exec.fit_masked in fixed_final_epoch mode over ALL rows, with a
    per-epoch callback.  The callback runs under no_grad and touches neither the parameters nor the
    random stream, so the trajectory is exactly the one fit_masked would follow."""
    import torch
    f32 = np.float32
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(EX.J_SEED)
    np.random.seed(EX.J_SEED)
    rng = np.random.default_rng(EX.J_SEED)
    Xt = torch.as_tensor(np.asarray(obs, dtype=f32))
    Yt = torch.as_tensor(np.asarray(act, dtype=f32))
    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(init[k], dtype=f32)))  # noqa: E731
    W1, b1, W2, b2 = P("pi.0.0.weight"), P("pi.0.0.bias"), P("pi.0.2.weight"), P("pi.0.2.bias")
    W3 = torch.nn.Parameter(torch.as_tensor(np.array(init["pi.1.weight"], dtype=f32)))
    b3 = torch.nn.Parameter(torch.as_tensor(np.array(init["pi.1.bias"], dtype=f32)))
    params = [W1, b1, W2, b2, W3, b3]
    anchor = [p.detach().clone() for p in params]
    src_w = W3.detach()[R.ACTION_DIM:].clone()
    src_b = b3.detach()[R.ACTION_DIM:].clone()
    zero_cols = sorted(set(B0.controller_columns()) | set(B0.CLOCK_COLUMNS))
    col_mask = torch.ones_like(W1)
    col_mask[:, zero_cols] = 0.0
    with torch.no_grad():
        W1.mul_(col_mask)
    fwd = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2) @ W3.T + b3  # noqa: E731
    opt = torch.optim.Adam(params, lr=EX.J_LR)
    with torch.no_grad():
        source_logstd = fwd(Xt)[:, R.ACTION_DIM:].detach().clone()

    def snapshot() -> dict[str, np.ndarray]:
        w1 = W1.detach().numpy().copy(); bb1 = b1.detach().numpy().copy()
        w2 = W2.detach().numpy().copy(); bb2 = b2.detach().numpy().copy()
        w3 = W3.detach().numpy().copy(); bb3 = b3.detach().numpy().copy()
        return {"pi_encoder.0.weight": np.ascontiguousarray(w1.copy()),
                "pi_encoder.0.bias": np.ascontiguousarray(bb1.copy()),
                "pi_encoder.2.weight": np.ascontiguousarray(w2.copy()),
                "pi_encoder.2.bias": np.ascontiguousarray(bb2.copy()),
                "pi.0.0.weight": np.ascontiguousarray(w1), "pi.0.0.bias": np.ascontiguousarray(bb1),
                "pi.0.2.weight": np.ascontiguousarray(w2), "pi.0.2.bias": np.ascontiguousarray(bb2),
                "pi.1.weight": np.ascontiguousarray(w3), "pi.1.bias": np.ascontiguousarray(bb3)}

    tr = np.arange(len(obs), dtype=np.int64)
    for epoch in range(1, int(epochs) + 1):
        order = rng.permutation(tr)
        for s0 in range(0, len(order), EX.J_BATCH):
            idx = torch.as_tensor(order[s0: s0 + EX.J_BATCH])
            opt.zero_grad(set_to_none=True)
            logits = fwd(Xt[idx])
            means = logits[:, : R.ACTION_DIM]
            mean_loss = ((means - Yt[idx]) ** 2).mean()
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = ((logits[:, R.ACTION_DIM:] - source_logstd[idx]) ** 2).mean()
            anchor_loss = torch.stack([(p - a).square().mean()
                                       for p, a in zip(params, anchor)]).mean()
            loss = (mean_loss + EX.J_CLIP_W * clip_loss + EX.J_LOGSTD_W * logstd_loss
                    + EX.J_ANCHOR_W * anchor_loss)
            loss.backward()
            if W1.grad is not None:
                W1.grad.mul_(col_mask)
            opt.step()
            with torch.no_grad():
                W3[R.ACTION_DIM:] = src_w
                b3[R.ACTION_DIM:] = src_b
                W1.mul_(col_mask)
            if not bool(torch.all(W1.detach()[:, zero_cols] == 0.0)):
                raise B1R2Error("a masked or clock column is not exactly zero after the step")
        on_epoch(epoch, snapshot())
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps({"epoch": epoch}), flush=True)
    return snapshot()


def reconstruction_curve(*, progress: bool = True) -> dict[str, Any]:
    """Question 1: does any epoch satisfy the reconstruction gates, on any row group?"""
    g = row_groups()
    inp = g["inp"]
    init, _ = R1.load_parent()
    obs, act = inp["obs"], inp["act"]
    groups = {"all_1500": None,
              # by B1R1 coverage
              "b1r1_covered_cycles_910": g["cycle"],
              "b1r1_uncovered_startup_tails_403": g["uncovered"],
              # by structure
              "structurally_complete_cycles_1218": g["structurally_complete"],
              "tails_95": g["tails"],
              "wait_187": g["wait"]}
    rows: list[dict[str, Any]] = []

    def on_epoch(epoch: int, state: dict[str, np.ndarray]) -> None:
        rec: dict[str, Any] = {"epoch": epoch}
        for name, idx in groups.items():
            m = EX.metrics(state, obs, act, idx)
            rec[name] = {"aggregate_rmse": m["aggregate_rmse"],
                         "knee_rmse": m["per_joint"]["knee"]["rmse"],
                         "ankle_rmse": m["per_joint"]["ankle"]["rmse"],
                         "knee_max_abs": m["per_joint"]["knee"]["max_abs"],
                         "ankle_max_abs": m["per_joint"]["ankle"]["max_abs"]}
        rows.append(rec)

    final = fit_all_instrumented(init, obs, act, epochs=EPOCHS, on_epoch=on_epoch, progress=progress)
    B0.assert_masked_columns_zero(final, "instrumented final")
    B0.assert_clock_columns_zero(final, "instrumented final")

    def gate_ok(r: Mapping[str, Any]) -> bool:
        return (r["aggregate_rmse"] <= GATES["aggregate_rmse_max"]
                and max(r["knee_rmse"], r["ankle_rmse"]) <= GATES["per_joint_rmse_max"]
                and max(r["knee_max_abs"], r["ankle_max_abs"]) <= GATES["per_joint_max_abs_max"])

    summary: dict[str, Any] = {}
    for name in groups:
        ok = [r["epoch"] for r in rows if gate_ok(r[name])]
        best = min(rows, key=lambda r: r[name]["aggregate_rmse"])
        summary[name] = {
            "any_epoch_satisfies_the_reconstruction_gates": bool(ok),
            "first_satisfying_epoch": (int(ok[0]) if ok else None),
            "satisfying_epochs": len(ok),
            "best_epoch_by_aggregate_rmse": int(best["epoch"]),
            "best_aggregate_rmse": best[name]["aggregate_rmse"],
            "at_best": best[name],
            "final_epoch_400": rows[-1][name],
        }
    return {"epochs": EPOCHS, "gates": GATES, "per_group": summary, "curve": rows}


# ================================================================ 2. full coverage ===============

def full_coverage(*, embargo: int = EMBARGO_ROWS) -> dict[str, Any]:
    """Question 2: eleven folds that hold out every non-WAIT row EXACTLY ONCE.

    Semantically the design is EIGHT COMPLETE CYCLES plus THREE TAILS: the six cycles B1R1 already
    held out, the two structurally complete cycles that merely start at the episode origin, and the
    three genuinely truncated tails.  Coverage, embargo, train-only scaling, the effective 23D
    support, the exact discrete signatures and the gates are all unchanged by this renaming.
    WAIT is never held out; it stays reconstruction-only."""
    g = row_groups()
    inp = g["inp"]
    raw, traj, act = inp["raw"], inp["traj"], inp["act"]
    n = len(raw)
    blocks = []
    for f in inp["split"]["folds"]:
        blocks.append({"kind": "complete_cycle", "origin": "b1r1_loco", "name": f["name"],
                       "cycle": int(f["cycle"]), "rows": np.asarray(f["_val"], dtype=int)})
    for p in g["uncovered_segments"]:
        blocks.append({"kind": p["structure"], "origin": "b1r1_uncovered",
                       "name": p["name"], "position": p["position"],
                       "structure_reason": p["reason"],
                       "rows": np.arange(p["global_lo"], p["global_hi"] + 1, dtype=int)})
    seen = np.concatenate([b["rows"] for b in blocks])
    if len(seen) != len(np.unique(seen)):
        raise B1R2Error("the folds overlap: some row would be held out more than once")
    non_wait = np.array([i for i in range(n) if i not in set(g["wait"].tolist())], dtype=int)
    if not np.array_equal(np.sort(seen), np.sort(non_wait)):
        raise B1R2Error("the folds do not cover every non-WAIT row exactly once")
    folds = []
    for k, b in enumerate(blocks):
        val = b["rows"]
        is_val = np.zeros(n, dtype=bool); is_val[val] = True
        emb = np.zeros(n, dtype=bool)
        tid = int(traj[val[0]])
        idx = np.where(traj == tid)[0]
        for p in np.where(is_val[idx])[0]:
            lo = max(0, p - embargo); hi = min(len(idx) - 1, p + embargo)
            emb[idx[lo:hi + 1]] = True
        train = np.where(~is_val & ~emb)[0]
        if train.size == 0:
            raise B1R2Error(f"fold {k} has an empty training set")
        if np.any(raw[val, LO.WAIT_COLUMN] > 0.5):
            raise B1R2Error(f"fold {k} would validate WAIT rows")
        wait_tr = int(np.sum(raw[train, LO.WAIT_COLUMN] > 0.5))
        cov = LO.signature_coverage(raw, val, train)
        sup = LO.support_and_baseline(raw, act, val, train)
        tem = LO.temporal_leakage(val, train, traj)
        folds.append({"fold": k, "kind": b["kind"], "origin": b["origin"], "name": b["name"],
                      "detail": (b.get("cycle") if b.get("cycle") is not None
                                 else b.get("position")),
                      "validation_rows": int(val.size), "train_rows": int(train.size),
                      "dropped_to_embargo": int(n - val.size - train.size),
                      "wait_rows_in_train": wait_tr,
                      "min_temporal_distance": tem["min_temporal_distance_same_trajectory"],
                      "immediate_neighbours": tem["validation_rows_with_an_immediate_training_neighbour"],
                      "signature_coverage_pass": cov["pass"],
                      "missing_signature_rows": cov["validation_rows_whose_signature_is_absent_from_train"],
                      "support_dimensions": sup["support_dimensions"],
                      "fraction_beyond_train_p99": sup["fraction_beyond_train_internal_p99"],
                      "nn_label_baseline_DIAGNOSTIC_ONLY":
                          sup["nn_label_baseline_DIAGNOSTIC_ONLY"]["aggregate_rmse"]})
    n_complete = sum(1 for f in folds if f["kind"] == "complete_cycle")
    n_tail = sum(1 for f in folds if f["kind"] == "tail")
    return {"mode": "eight_complete_cycles_plus_three_tails", "n_folds": len(folds),
            "n_complete_cycle_folds": n_complete, "n_tail_folds": n_tail,
            "semantics": g["structure_summary"]["semantics"],
            "embargo_rows": embargo,
            "every_non_wait_row_held_out_exactly_once": True,
            "wait_policy": "WAIT is never held out; it remains reconstruction-only",
            "scaler": "fit on TRAINING ROWS ONLY", "support": f"{len(LO.SUPPORT_COLUMNS)}D",
            "folds": folds,
            "aggregate": {
                "any_temporal_leakage": any(f["min_temporal_distance"] is not None
                                            and f["min_temporal_distance"] <= 1 for f in folds),
                "min_temporal_distance": min((f["min_temporal_distance"] for f in folds
                                              if f["min_temporal_distance"] is not None), default=None),
                "any_signature_missing": any(not f["signature_coverage_pass"] for f in folds),
                "worst_nn_label_baseline": max(f["nn_label_baseline_DIAGNOSTIC_ONLY"] for f in folds),
                "worst_fraction_beyond_p99": max(f["fraction_beyond_train_p99"] for f in folds),
                "train_rows_min": min(f["train_rows"] for f in folds),
                "dropped_median": float(np.median([f["dropped_to_embargo"] for f in folds])),
                "validation_rows_total": int(sum(f["validation_rows"] for f in folds))}}


# ============================================ 3. support sparsity and local smoothness ===========

COLLISION_THRESHOLDS = (1e-8, 1e-6, 1e-4, 1e-3)
TARGET_DELTA_QUERIES = (0.02, 0.05)

NOT_AN_INFORMATION_FLOOR = (
    "This section measures how SPARSE the effective 23D support is and how SMOOTH the target is "
    "locally. It is NOT an information floor and must never be read as one. A nearest neighbour at "
    "normalised distance 0.4 is a far-away point, not a collision: the target may legitimately "
    "differ there, and a smooth function of the support can fit both rows exactly. Only a genuine "
    "collision - two rows nearly identical in the inputs but disagreeing in the target - would "
    "bound achievable accuracy, which is why the collision audit below is reported separately and "
    "at real distance thresholds. This measure is diagnostic and never a gate.")


def _support_spaces(raw: np.ndarray) -> dict[str, np.ndarray]:
    """The effective 23D support in raw units and standardised. Both are reported: raw distances
    are what the actor literally receives, standardised distances make heterogeneous units
    comparable. A collision must be a collision in both to be taken seriously."""
    sup = LO.support_matrix(raw)
    sc = LO.train_only_scaler(sup, np.arange(len(sup)))
    return {"raw": np.asarray(sup, dtype=np.float64),
            "standardised": np.asarray(LO.apply_scaler(sup, sc), dtype=np.float64)}


def _collide(Z: np.ndarray, A: np.ndarray, rows: np.ndarray) -> dict[str, Any]:
    """Exhaustive pairwise audit over one stratum: every unordered pair, no sampling."""
    from scipy.spatial.distance import pdist
    if rows.size < 2:
        return {"rows": int(rows.size), "pairs": 0, "note": "fewer than two rows, no pair exists"}
    d = pdist(Z[rows])                              # euclidean distance in the support
    t = pdist(A[rows], metric="chebyshev")          # max |delta target| over the two joints
    out: dict[str, Any] = {"rows": int(rows.size), "pairs": int(d.size),
                           "min_distance": float(d.min()),
                           "max_target_disagreement_over_all_pairs": float(t.max())}
    at: dict[str, Any] = {}
    for thr in COLLISION_THRESHOLDS:
        m = d <= thr
        at[f"{thr:g}"] = {"pairs_within": int(m.sum()),
                          "max_target_disagreement": (float(t[m].max()) if m.any() else None)}
    out["at_distance"] = at
    mind: dict[str, Any] = {}
    for q in TARGET_DELTA_QUERIES:
        m = t > q
        mind[f"gt_{q:g}"] = {"pairs": int(m.sum()),
                             "min_distance": (float(d[m].min()) if m.any() else None)}
    out["min_distance_among_pairs_with_target_delta"] = mind
    return out


def collision_audit() -> dict[str, Any]:
    """Are there REAL collisions in the effective 23D inputs?

    A collision is two rows at a genuinely small input distance whose targets disagree. This is the
    only measurement in the study that could bound achievable accuracy, so it is done exhaustively
    over every unordered pair, at four real distance thresholds, and stratified by the EXACT
    discrete signature - because two rows in different FSM states are distinguishable by
    construction and their proximity in the continuous columns is not a collision at all."""
    g = row_groups()
    inp = g["inp"]
    raw, act = inp["raw"], np.asarray(inp["act"], dtype=np.float64)
    spaces = _support_spaces(raw)
    sig = LO.discrete_signature(raw)
    n = len(raw)
    idx = np.arange(n)
    strata: list[tuple[str, np.ndarray]] = [("all", idx)]
    for s in sorted({tuple(x) for x in sig}, key=str):
        strata.append((f"signature_{list(s)}", np.array([i for i in idx if sig[i] == s], dtype=int)))
    report: dict[str, Any] = {}
    for space, Z in spaces.items():
        report[space] = {tag: _collide(Z, act, rows) for tag, rows in strata}
    any_real = any(report["standardised"][t]["at_distance"][f"{thr:g}"]["pairs_within"] > 0
                   for t in report["standardised"] for thr in COLLISION_THRESHOLDS)
    return {"thresholds": list(COLLISION_THRESHOLDS),
            "target_delta_queries": list(TARGET_DELTA_QUERIES),
            "distance": "euclidean over the effective 23D support",
            "target_disagreement": "max absolute difference over the two normalised joint actions",
            "pairs": "exhaustive, every unordered pair, no sampling",
            "stratification": "exact discrete signature over columns 17,18,19,20,21,24",
            "any_pair_within_any_threshold": bool(any_real),
            "by_space": report}


def support_sparsity_diagnostic() -> dict[str, Any]:
    """Question 3: SUPPORT SPARSITY AND LOCAL SMOOTHNESS in the effective 23D support.

    For every row, the nearest OTHER row is taken among rows at temporal distance greater than the
    embargo, and the target disagreement with it is recorded together with HOW FAR that neighbour
    actually is.  Large disagreement at a large distance means the support is sparse there and the
    target varies over that scale - a statement about sampling density and local smoothness.  It is
    NOT a statement that the mapping is unlearnable; see NOT_AN_INFORMATION_FLOOR.  The genuine
    non-identifiability question is answered by collision_audit(), not here."""
    from scipy.spatial import cKDTree
    g = row_groups()
    inp = g["inp"]
    raw, act, traj = inp["raw"], inp["act"], inp["traj"]
    Z = _support_spaces(raw)["standardised"]
    sig = LO.discrete_signature(raw)
    n = len(Z)
    uncovered = set(int(i) for i in g["uncovered"])
    wait = set(int(i) for i in g["wait"])
    complete = set(int(i) for i in g["structurally_complete"])
    tails = set(int(i) for i in g["tails"])
    tree = cKDTree(Z)
    k = min(200, n)
    D, I = tree.query(Z, k=k)
    out_d = np.full(n, np.nan); out_l = np.full(n, np.nan); out_same_sig = np.zeros(n, dtype=bool)
    out_same_traj = np.zeros(n, dtype=bool)
    for i in range(n):
        for dd, j in zip(D[i], I[i]):
            j = int(j)
            if j == i:
                continue
            if traj[j] == traj[i] and abs(j - i) <= EMBARGO_ROWS:
                continue
            out_d[i] = float(dd)
            out_l[i] = float(np.max(np.abs(act[i] - act[j])))
            out_same_sig[i] = sig[i] == sig[j]
            out_same_traj[i] = bool(traj[i] == traj[j])
            break
    ok = np.isfinite(out_l)

    def block(mask: np.ndarray, tag: str) -> dict[str, Any]:
        m = mask & ok
        if not m.any():
            return {"group": tag, "rows": 0}
        return {"group": tag, "rows": int(m.sum()),
                "nn_distance_median": float(np.median(out_d[m])),
                "nn_distance_p10": float(np.quantile(out_d[m], 0.1)),
                "nn_distance_min": float(out_d[m].min()),
                "target_disagreement_median": float(np.median(out_l[m])),
                "target_disagreement_p90": float(np.quantile(out_l[m], 0.9)),
                "target_disagreement_max": float(out_l[m].max()),
                "fraction_disagreement_gt_per_joint_rmse_gate":
                    float(np.mean(out_l[m] > GATES["per_joint_rmse_max"])),
                "fraction_neighbour_same_signature": float(np.mean(out_same_sig[m])),
                "fraction_neighbour_same_trajectory": float(np.mean(out_same_traj[m]))}

    idx = np.arange(n)
    strata = [block(np.ones(n, dtype=bool), "all"),
              block(np.array([i in uncovered for i in idx]), "b1r1_uncovered_startup_tails"),
              block(np.array([i in (complete - wait) for i in idx]), "structurally_complete_cycles"),
              block(np.array([i in tails for i in idx]), "tails"),
              block(np.array([i in g["cycle"].tolist() for i in idx]), "b1r1_covered_cycles"),
              block(np.array([i in wait for i in idx]), "wait")]
    for t, name in enumerate(TRAJECTORY_NAMES):
        strata.append(block(traj == t, f"trajectory_{name}"))
    for s in sorted({tuple(x) for x in sig}, key=str):
        strata.append(block(np.array([sig[i] == s for i in idx]), f"signature_{list(s)}"))
    return {"measure": "support sparsity and local smoothness diagnostic",
            "neighbour_rule": f"nearest row at temporal distance greater than {EMBARGO_ROWS} within "
                              "the same trajectory, or any row of another trajectory",
            "support": f"{len(LO.SUPPORT_COLUMNS)}D, standardised, scaler fit on the rows used",
            "interpretation": NOT_AN_INFORMATION_FLOOR,
            "strata": strata}


def residual_predictability_cv(*, alpha: float = 1.0) -> dict[str, Any]:
    """Is the B1R1 residual predictable OUT OF FOLD from the effective 23D support?

    Ridge regression, evaluated only on held-out rows, with leave-one-segment-out folds that never
    mix contiguous rows of the same segment: each fold holds out one whole segment and the embargo
    neighbourhood is removed from its training set.  The scaler is fit on training rows only.

    In-sample fits on small row clusters are DELIBERATELY ABSENT.  A 23-regressor least squares over
    a few dozen contiguous rows interpolates noise and reports R^2 near one whatever the truth is;
    such a number is not evidence and is excluded from every conclusion drawn here."""
    g = row_groups()
    inp = g["inp"]
    raw, act, traj = inp["raw"], np.asarray(inp["act"], dtype=np.float64), inp["traj"]
    state_path = VA.OUT_ROOT / "candidates" / "B1R1_BASE35_LOCO" / "rl_module" / "module_state.pkl"
    if not state_path.is_file():
        return {"status": "SKIPPED", "reason": "the B1R1 module state is not present"}
    import pickle
    with state_path.open("rb") as fh:
        st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    pred = np.asarray(RF.numpy_mean(dict(st), np.asarray(inp["obs"], dtype=np.float32)),
                      dtype=np.float64)
    resid = pred - act
    sup = LO.support_matrix(raw)
    n = len(raw)

    blocks: list[dict[str, Any]] = []
    for f in inp["split"]["folds"]:
        blocks.append({"name": f"b1r1_{f['name']}_c{int(f['cycle'])}", "kind": "complete_cycle",
                       "rows": np.asarray(f["_val"], dtype=int)})
    for p in g["uncovered_segments"]:
        blocks.append({"name": f"{p['name']}_{p['position']}", "kind": p["structure"],
                       "rows": np.arange(p["global_lo"], p["global_hi"] + 1, dtype=int)})
    blocks.append({"name": "wait", "kind": "wait", "rows": np.asarray(g["wait"], dtype=int)})

    oof = np.full_like(resid, np.nan)
    per_fold: list[dict[str, Any]] = []
    for b in blocks:
        val = b["rows"]
        is_val = np.zeros(n, dtype=bool); is_val[val] = True
        emb = np.zeros(n, dtype=bool)
        tid = int(traj[val[0]])
        tidx = np.where(traj == tid)[0]
        for p in np.where(is_val[tidx])[0]:
            lo = max(0, p - EMBARGO_ROWS); hi = min(len(tidx) - 1, p + EMBARGO_ROWS)
            emb[tidx[lo:hi + 1]] = True
        tr = np.where(~is_val & ~emb)[0]
        sc = LO.train_only_scaler(sup, tr)
        Xtr = np.asarray(LO.apply_scaler(sup, sc), dtype=np.float64)[tr]
        Xva = np.asarray(LO.apply_scaler(sup, sc), dtype=np.float64)[val]
        ytr = resid[tr]
        xm = Xtr.mean(axis=0); ym = ytr.mean(axis=0)
        Xc = Xtr - xm
        gram = Xc.T @ Xc + alpha * np.eye(Xc.shape[1])
        w = np.linalg.solve(gram, Xc.T @ (ytr - ym))
        oof[val] = (Xva - xm) @ w + ym
        per_fold.append({"fold": b["name"], "kind": b["kind"], "validation_rows": int(val.size),
                         "train_rows": int(tr.size)})

    def r2(rows: np.ndarray) -> dict[str, Any]:
        rows = np.asarray(rows, dtype=int)
        if rows.size < 2:
            return {"rows": int(rows.size)}
        y, yh = resid[rows], oof[rows]
        out = {"rows": int(rows.size)}
        for j, name in enumerate(("knee", "ankle")):
            ss_tot = float(((y[:, j] - y[:, j].mean()) ** 2).sum())
            ss_res = float(((y[:, j] - yh[:, j]) ** 2).sum())
            out[name] = {"out_of_fold_r2": (1.0 - ss_res / ss_tot) if ss_tot > 0 else None,
                         "residual_rms": float(np.sqrt((y[:, j] ** 2).mean()))}
        return out

    groups = {"all": np.arange(n), "b1r1_uncovered_startup_tails": g["uncovered"],
              "b1r1_covered_cycles": g["cycle"],
              "structurally_complete_cycles": np.array(sorted(set(int(i) for i in
                                                                 g["structurally_complete"])
                                                              - set(int(i) for i in g["wait"])),
                                                       dtype=int),
              "tails": g["tails"], "wait": g["wait"]}
    return {"status": "OK", "estimator": f"ridge, alpha={alpha}, on the standardised 23D support",
            "target": "signed per-joint residual of the B1R1 actor, prediction minus label",
            "validation": "leave-one-segment-out, embargo "
                          f"{EMBARGO_ROWS}, scaler fit on training rows only, "
                          "no fold mixes contiguous rows of the same held-out segment",
            "folds": per_fold,
            "in_sample_cluster_fits": "DELIBERATELY NOT COMPUTED - not valid evidence",
            "out_of_fold_r2": {k: r2(v) for k, v in groups.items()}}


# ================================================================ 4. label and action audit ======

def label_and_action_audit(top_n: int = 25) -> dict[str, Any]:
    """Question 4: does a label-lookup or action-semantics defect explain the error peaks?"""
    g = row_groups()
    inp = g["inp"]
    raw, act, traj = inp["raw"], inp["act"], inp["traj"]
    # (a) re-derive every label from the pinned caches by exact time lookup
    rederived = np.empty_like(act)
    per_traj = {}
    for tid, (start, spec) in enumerate(sorted(R.ANCHORS.items())):
        job = Path(spec["job_dir"])
        t = np.asarray(DS.trajectory_from_job(job, expected_width=B0.ACTOR_WIDTH)["t_pre"],
                       dtype=np.float64)
        cache = L.load_cache(R.OUT_CACHE, start)
        idx = cache.lookup(t)
        u = np.asarray(cache.ik_action, dtype=np.float32)[idx]
        rows = np.where(traj == tid)[0]
        rederived[rows] = u
        per_traj[start] = {"rows": int(rows.size), "cache_digest": cache.digest(),
                           "lookup": "exact float equality", "index_span": [int(idx[0]), int(idx[-1])],
                           "monotone_index": bool(np.all(np.diff(idx) == 1))}
    identical = bool(np.array_equal(rederived, act))
    # (b) action semantics: bounds and encode/decode round trip
    q = SC.decode_action(np.asarray(act, dtype=np.float64))
    re_enc = L.encode_absolute_action(q)
    round_trip = float(np.max(np.abs(np.asarray(re_enc, dtype=np.float64) - act)))
    in_bounds = bool(np.all(np.abs(act) <= 1.0 + 1e-6))
    kb, ab = R.ABSOLUTE_BOUNDS_RAD["pros_knee_angle"], R.ABSOLUTE_BOUNDS_RAD["pros_ankle_angle"]
    q_in = bool(np.all(q[:, 0] >= kb[0] - 1e-9) and np.all(q[:, 0] <= kb[1] + 1e-9)
                and np.all(q[:, 1] >= ab[0] - 1e-9) and np.all(q[:, 1] <= ab[1] + 1e-9))
    # (c) label continuity: a lookup defect would show as a jump against time
    jumps = []
    for tid in range(len(TRAJECTORY_NAMES)):
        rows = np.where(traj == tid)[0]
        d = np.abs(np.diff(act[rows], axis=0)).max(axis=1)
        jumps.append({"trajectory": TRAJECTORY_NAMES[tid], "max_step_change": float(d.max()),
                      "p999_step_change": float(np.quantile(d, 0.999)),
                      "argmax_local_row": int(np.argmax(d)) + 1})
    # (d) where the fitted actor peaks, and what the label looks like there
    state_path = VA.OUT_ROOT / "candidates" / "B1R1_BASE35_LOCO" / "rl_module" / "module_state.pkl"
    peaks: list[dict[str, Any]] = []
    if state_path.is_file():
        import pickle
        with state_path.open("rb") as fh:
            st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
        pred = RF.numpy_mean(dict(st), np.asarray(inp["obs"], dtype=np.float32))
        err = np.abs(pred - act)
        worst = np.argsort(-err.max(axis=1))[:top_n]
        uncovered = set(int(i) for i in g["uncovered"]); wait = set(int(i) for i in g["wait"])
        tails = set(int(i) for i in g["tails"])
        for i in worst:
            i = int(i)
            tid = int(traj[i]); off = int(np.searchsorted(traj, tid))
            neigh = [j for j in (i - 1, i + 1) if 0 <= j < len(act) and traj[j] == tid]
            peaks.append({"global_row": i, "trajectory": TRAJECTORY_NAMES[tid],
                          "local_row": i - off,
                          "b1r1_coverage": ("wait" if i in wait else
                                            "uncovered" if i in uncovered else "covered"),
                          "structure": ("wait" if i in wait else
                                        "tail" if i in tails else "complete_cycle"),
                          "label_matches_cache": bool(np.array_equal(rederived[i], act[i])),
                          "abs_error": [float(err[i, 0]), float(err[i, 1])],
                          "label": [float(act[i, 0]), float(act[i, 1])],
                          "max_label_step_to_time_neighbour": float(max(
                              np.max(np.abs(act[i] - act[j])) for j in neigh)) if neigh else None})
    return {"labels_rederived_from_the_pinned_caches": per_traj,
            "rederived_labels_bit_identical_to_the_dataset": identical,
            "action_semantics": {"all_labels_within_[-1,1]": in_bounds,
                                 "encode_decode_round_trip_max_abs": round_trip,
                                 "decoded_angles_within_absolute_bounds": q_in,
                                 "bounds": {"knee": list(kb), "ankle": list(ab)}},
            "label_continuity_per_trajectory": jumps,
            "worst_prediction_rows": peaks,
            "verdict": ("no label-lookup or action-semantics defect is present"
                        if identical and in_bounds and q_in and round_trip < 1e-6
                        else "a label or action-semantics anomaly was found and must be resolved "
                             "before any conclusion about the fit")}


# ================================================================ driver =========================

def run_study(*, write: bool = True, progress: bool = True) -> dict[str, Any]:
    g = row_groups()
    audit = label_and_action_audit()
    sparsity = support_sparsity_diagnostic()
    collisions = collision_audit()
    resid = residual_predictability_cv()
    cov = full_coverage()
    rec = reconstruction_curve(progress=progress)
    receipt = {"schema": "v26b_b1r2_diagnostics.2", "stage": STAGE,
               "kind": "DIAGNOSTIC study. No candidate is materialised or promoted, no rollout, no "
                       "collection, no Markov phase, no corrective fit",
               "revision": "mandatory architectural revision: the nearest-neighbour measure is a "
                           "support-sparsity and local-smoothness diagnostic and never an "
                           "information floor; a real collision audit is added; residual "
                           "predictability is out-of-fold ridge only; the two uncovered head "
                           "segments are reclassified as structurally complete cycles",
               "gates_unchanged": GATES,
               "row_groups": {"wait": int(g["wait"].size),
                              "b1r1_covered_cycles": int(g["cycle"].size),
                              "b1r1_uncovered_startup_tails": int(g["uncovered"].size),
                              "structurally_complete_rows": int(g["structurally_complete"].size),
                              "tail_rows": int(g["tails"].size),
                              "structure_summary": g["structure_summary"],
                              "uncovered_segments": g["uncovered_segments"]},
               "q4_label_and_action_audit": audit,
               "q3_support_sparsity_and_local_smoothness": sparsity,
               "q3_collision_audit": collisions,
               "q3_residual_predictability_out_of_fold": resid,
               "q2_full_validation_coverage": cov,
               "q1_reconstruction_curve": rec,
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    if write:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        C.write_json(OUT_DIR / RECEIPT_NAME, receipt, clobber=False)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B B1R2 diagnostic study")
    p.add_argument("--run", action="store_true")
    p.add_argument("--no-write", action="store_true")
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if not a.run:
        row_groups(); print(json.dumps({"mode": "dry", "ok": True}, indent=2)); return 0
    r = run_study(write=not a.no_write, progress=not a.no_progress)
    print(json.dumps({"q4": r["q4_label_and_action_audit"]["verdict"],
                      "q1": {k: {"satisfies_gates": v["any_epoch_satisfies_the_reconstruction_gates"],
                                 "best_epoch": v["best_epoch_by_aggregate_rmse"],
                                 "best_rmse": v["best_aggregate_rmse"]}
                             for k, v in r["q1_reconstruction_curve"]["per_group"].items()},
                      "q2": {"semantics": r["q2_full_validation_coverage"]["semantics"],
                             **r["q2_full_validation_coverage"]["aggregate"]},
                      "q3_sparsity_all": r["q3_support_sparsity_and_local_smoothness"]["strata"][0],
                      "q3_collisions_any": r["q3_collision_audit"]["any_pair_within_any_threshold"],
                      "q3_residual_oof_r2": r["q3_residual_predictability_out_of_fold"]
                                             .get("out_of_fold_r2")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
