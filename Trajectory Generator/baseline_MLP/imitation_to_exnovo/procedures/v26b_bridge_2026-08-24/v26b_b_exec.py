"""V26B executable stage: materialise B0 and run the B1 base fit. Fail-closed.

B0  the masked 35D transplant from the August V26 parent, written no-clobber.
B1  three leave-one-trajectory-out folds from the SAME B0 for epoch selection, then one final
    fit from B0 on all 1500 rows, with no validation, for the MEDIAN of the three best epochs.

The fit follows the July imitative protocol pinned in v26b_july_imitation_protocol_pin.json:
Adam, flat MSE, 400 epochs max, batch 64, lr 3e-4, patience 60, clip weight 1.0, log-std weight
0.1, anchor weight 1e-5, seed 123, no gradient clipping, no PPO.  The log-std head is HARD
FROZEN and restored after every step; the critic is never loaded or touched.

At every batch: the input is masked on columns 25:35, the first-layer columns 25:35 and the two
clock columns are projected to exact zero after the optimizer step, and the projection is
asserted.  No scaling is applied: every live feature has scale 1.0 (the only non-unit scales in
the pinned table are indices 27, 28, 32 and 33, all inside the masked channel).

No rollout, no collection, no Markov phase.  Cross-platform: pathlib only, no shell.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class ExecError(RuntimeError):
    pass


STAGE_B0 = "V26B-B0-MASKED35-TRANSPLANT"
STAGE_B1 = "V26B-B1-BASE-FIT-MASKED35"

AMENDMENT = HERE / "v26b_amendment_b_masked35.json"
PIN_AMENDMENT = "0830d82feb641043dffc4457c366a8dc6aa12979a3d5605e4a424903e19bf3ae"
JULY_PIN = HERE / "v26b_july_imitation_protocol_pin.json"
PIN_JULY = "0198e10965000ed908194083e23a682fec72fe5fc9c2c22e0f022a4f30a049d7"
SUPERSESSION = HERE / "v26b_addendum_a0a1_superseded.json"
PIN_SUPERSESSION = "cc24eec8cb56a459b7a6d280af9d0232014018638bba9f1e223325295a4b6791"

# --- July hyperparameters, pinned ---------------------------------------------------------------
J_EPOCHS = 400
J_BATCH = 64
J_LR = 3e-4
J_PATIENCE = 60
J_CLIP_W = 1.0
J_LOGSTD_W = 0.1
J_ANCHOR_W = 1e-5
J_SEED = 123

# --- preregistered binding offline gates ---------------------------------------------------------
FOLD_HELDOUT_RMSE_MAX = 0.05
LOTO_WEIGHTED_MEAN_MAX = 0.03
FINAL_RMSE_MAX = 0.02
FINAL_PER_JOINT_RMSE_MAX = 0.03
FINAL_PER_JOINT_MAXABS_MAX = 0.15

B0_DIR = B0.OUT_DIR
B1_DIR = B1.OUT_DIR
B0_RECEIPT = "v26b_b0_receipt.json"
B1_RECEIPT = "v26b_b1_receipt.json"
MODULE_DIR_NAME = "rl_module"


def verify_pins() -> dict[str, str]:
    out = {}
    for path, pin, key in ((AMENDMENT, PIN_AMENDMENT, "amendment_b_masked35"),
                           (JULY_PIN, PIN_JULY, "july_protocol_pin"),
                           (SUPERSESSION, PIN_SUPERSESSION, "supersession_addendum")):
        got = C.sha256_file(path)
        if got != pin:
            raise ExecError(f"{key} sha {got} != pinned {pin}")
        out[key] = got
    if any(m in sys.modules for m in B0.SUPERSEDED_MODULES):
        raise ExecError("a superseded 25D module is loaded; the quarantine forbids it")
    return out


# ============================================================== B0 materialisation ==============

def materialize_b0(*, authorized_stage: str | None) -> dict[str, Any]:
    if authorized_stage != STAGE_B0:
        raise ExecError(f"requires --authorized-stage {STAGE_B0}; got {authorized_stage!r}")
    pins = verify_pins()
    pre = B0.preflight()                       # fail-closed, also refuses if B0_DIR exists
    state = {k: np.asarray(v) for k, v in B0.build_b0_state()[0].items()}
    reports = B0.build_b0_state()[1]
    names35, _, mshas = VS.pinned_names()
    src_module = (R.BASELINE_DIR.parent / "runs" / "training"
                  / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
    B0.assert_not_forbidden(src_module, "B0 parent")
    ctor, ctor_report = VS.build_ctor_35(src_module)
    staging = B0_DIR.parent / (B0_DIR.name + ".staging")
    if staging.exists():
        shutil.rmtree(staging)
    sm = staging / MODULE_DIR_NAME
    sm.mkdir(parents=True, exist_ok=False)
    try:
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in state.items()}, fh,
                        protocol=pickle.HIGHEST_PROTOCOL)
        with (sm / "class_and_ctor_args.pkl").open("wb") as fh:
            pickle.dump(ctor, fh, protocol=pickle.HIGHEST_PROTOCOL)
        shutil.copy2(VA.OUT_ROOT / "student" / "V1_35D_transplant" / MODULE_DIR_NAME / "metadata.json",
                     sm / "metadata.json")
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": RF.actor_state_digest(state),
                    "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                    "manifest35_sha256": mshas["manifest35_sha256"],
                    "exploration_sigma": [B0.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                    "sigma_note": B0.SIGMA_STATEMENT,
                    "derived_from": C.rel(src_module),
                    "source_actor_digest": B0.PIN_V26_ACTOR_DIGEST,
                    "deployable": False, "sigma_unresolved": True,
                    "actor_label": "B0_35D_MASKED",
                    "controller_state_mask": {
                        "active": True, "columns": list(B0.controller_columns()),
                        "names": [names35[i] for i in B0.controller_columns()],
                        "mask_value": float(B0.MASK_VALUE),
                        "first_layer_columns": "exactly zero, frozen, restored after every step",
                        "note": "the ten features stay in the vector and in this manifest; they "
                                "are neutralised during the base phase and return in the Markov phase"},
                    "clock_columns": list(B0.CLOCK_COLUMNS),
                    "status": "BASE-PHASE INIT (B0); never promoted, never deployable"}
        C.write_json(sm / "actor_feature_manifest.json", manifest, clobber=False)
        receipt = {"schema": "v26b_b0_transplant.1", "authorized_stage": STAGE_B0,
                   "pins": pins, "deployable": False, "sigma_unresolved": True,
                   "parent": pre["parent"] if "parent" in pre else {
                       "module": C.rel(src_module), "actor_digest": B0.PIN_V26_ACTOR_DIGEST},
                   "contract": pre["contract"], "transplant": reports["transplant"],
                   "functional_equivalence_25": pre["functional_equivalence_25"],
                   "bit_exact_unmask_transition": pre["bit_exact_unmask_transition"],
                   "ctor": ctor_report,
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "output_module": C.rel(B0_DIR / MODULE_DIR_NAME),
                   "scope": "base-phase init only; no fit, no rollout, no promotion",
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / B0_RECEIPT, receipt, clobber=False)
        csha = C.sha256_file(staging / B0_RECEIPT)
        if B0_DIR.exists():
            raise ExecError(f"no-clobber: {B0_DIR} appeared during staging")
        staging.rename(B0_DIR)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {"module": C.rel(B0_DIR / MODULE_DIR_NAME), "receipt_sha256": csha,
            "actor_digest": RF.actor_state_digest(state)}


def load_b0_state() -> dict[str, np.ndarray]:
    p = B0_DIR / MODULE_DIR_NAME / "module_state.pkl"
    if not p.is_file():
        raise ExecError(f"B0 is not materialised: {C.rel(p)} is missing")
    with p.open("rb") as fh:
        st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    RF.validate_init_state(st, expected_actor_digest=None, width=B0.ACTOR_WIDTH)
    B0.assert_masked_columns_zero(st, "B0 on disk")
    B0.assert_clock_columns_zero(st, "B0 on disk")
    return st


# ============================================================== the masked July fit =============

def fit_masked(init_raw: Mapping[str, np.ndarray], obs_masked: np.ndarray, actions: np.ndarray,
               *, train_idx: np.ndarray, val_idx: np.ndarray | None, epochs: int,
               patience: int | None, progress: bool = False,
               lr: float | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """July loss and per-step order, with the controller and clock columns hard-zeroed.

    val_idx None  ->  fixed_final_epoch: no validation, exactly `epochs` epochs, final weights kept.

    `lr` is the ONLY injectable hyperparameter and it defaults to the pinned July J_LR, so omitting
    it reproduces the previous behaviour exactly, byte for byte.  It exists so a learning-rate arm
    can be run without editing this function's semantics or the pinned J_LR constant; every other
    hyperparameter remains module-level and un-injectable by construction.
    """
    lr = J_LR if lr is None else float(lr)
    import torch
    f32 = np.float32
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(J_SEED)
    np.random.seed(J_SEED)
    rng = np.random.default_rng(J_SEED)
    X = torch.as_tensor(np.asarray(obs_masked, dtype=f32))
    Y = torch.as_tensor(np.asarray(actions, dtype=f32))
    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(init_raw[k], dtype=f32)))  # noqa: E731
    W1, b1, W2, b2 = P("pi.0.0.weight"), P("pi.0.0.bias"), P("pi.0.2.weight"), P("pi.0.2.bias")
    W3 = torch.nn.Parameter(torch.as_tensor(np.array(init_raw["pi.1.weight"], dtype=f32)))
    b3 = torch.nn.Parameter(torch.as_tensor(np.array(init_raw["pi.1.bias"], dtype=f32)))
    params = [W1, b1, W2, b2, W3, b3]
    anchor = [p.detach().clone() for p in params]
    src_logstd_w = W3.detach()[R.ACTION_DIM:].clone()
    src_logstd_b = b3.detach()[R.ACTION_DIM:].clone()
    zero_cols = sorted(set(B0.controller_columns()) | set(B0.CLOCK_COLUMNS))
    col_mask = torch.ones_like(W1)
    col_mask[:, zero_cols] = 0.0
    with torch.no_grad():
        W1.mul_(col_mask)
    fwd = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2) @ W3.T + b3  # noqa: E731
    opt = torch.optim.Adam(params, lr=lr)
    with torch.no_grad():
        source_logstd = fwd(X)[:, R.ACTION_DIM:].detach().clone()

    def restore_logstd() -> None:
        with torch.no_grad():
            W3[R.ACTION_DIM:] = src_logstd_w
            b3[R.ACTION_DIM:] = src_logstd_b

    def project_columns() -> None:
        with torch.no_grad():
            W1.mul_(col_mask)

    def assert_projected() -> None:
        if not bool(torch.all(W1.detach()[:, zero_cols] == 0.0)):
            raise ExecError("a masked or clock first-layer column is not exactly zero after the step")

    def val_mse() -> float:
        with torch.no_grad():
            m = fwd(X[val_idx])[:, : R.ACTION_DIM]
            return float(((m - Y[val_idx]) ** 2).mean().item())

    fixed = val_idx is None
    hist: list[dict[str, Any]] = []
    best = float("inf") if not fixed else None
    best_state = [p.detach().clone() for p in params]
    best_epoch = 0
    stale = 0
    tr = np.asarray(train_idx, dtype=np.int64)
    for epoch in range(1, int(epochs) + 1):
        order = rng.permutation(tr)
        losses = []
        for s0 in range(0, len(order), J_BATCH):
            idx = torch.as_tensor(order[s0: s0 + J_BATCH])
            opt.zero_grad(set_to_none=True)
            logits = fwd(X[idx])
            means = logits[:, : R.ACTION_DIM]
            mean_loss = ((means - Y[idx]) ** 2).mean()
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = ((logits[:, R.ACTION_DIM:] - source_logstd[idx]) ** 2).mean()
            anchor_loss = torch.stack([(p - a).square().mean() for p, a in zip(params, anchor)]).mean()
            loss = mean_loss + J_CLIP_W * clip_loss + J_LOGSTD_W * logstd_loss + J_ANCHOR_W * anchor_loss
            loss.backward()
            if W1.grad is not None:                      # defensive: already zero under the mask
                W1.grad.mul_(col_mask)
            opt.step()
            restore_logstd()
            project_columns()
            assert_projected()
            losses.append(float(loss.item()))
        rec = {"epoch": epoch, "train_loss": float(np.mean(losses))}
        if fixed:
            hist.append({**rec, "val_mse": None})
        else:
            v = val_mse()
            hist.append({**rec, "val_mse": v})
            if v < best - 1e-9:
                best = v
                best_epoch = epoch
                best_state = [p.detach().clone() for p in params]
                stale = 0
            else:
                stale += 1
            if patience is not None and stale >= patience:
                break
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(hist[-1]), flush=True)
    if fixed:
        best_epoch = len(hist)
        best_state = [p.detach().clone() for p in params]
    with torch.no_grad():
        for p, b in zip(params, best_state):
            p.copy_(b)
        W3[R.ACTION_DIM:] = src_logstd_w
        b3[R.ACTION_DIM:] = src_logstd_b
        W1.mul_(col_mask)
    out = {"pi.0.0.weight": W1.detach().numpy().copy(), "pi.0.0.bias": b1.detach().numpy().copy(),
           "pi.0.2.weight": W2.detach().numpy().copy(), "pi.0.2.bias": b2.detach().numpy().copy(),
           "pi.1.weight": W3.detach().numpy().copy(), "pi.1.bias": b3.detach().numpy().copy()}
    state = {"pi_encoder.0.weight": np.ascontiguousarray(out["pi.0.0.weight"].copy()),
             "pi_encoder.0.bias": np.ascontiguousarray(out["pi.0.0.bias"].copy()),
             "pi_encoder.2.weight": np.ascontiguousarray(out["pi.0.2.weight"].copy()),
             "pi_encoder.2.bias": np.ascontiguousarray(out["pi.0.2.bias"].copy()),
             "pi.0.0.weight": np.ascontiguousarray(out["pi.0.0.weight"]),
             "pi.0.0.bias": np.ascontiguousarray(out["pi.0.0.bias"]),
             "pi.0.2.weight": np.ascontiguousarray(out["pi.0.2.weight"]),
             "pi.0.2.bias": np.ascontiguousarray(out["pi.0.2.bias"]),
             "pi.1.weight": np.ascontiguousarray(out["pi.1.weight"]),
             "pi.1.bias": np.ascontiguousarray(out["pi.1.bias"])}
    B0.assert_masked_columns_zero(state, "fit output")
    B0.assert_clock_columns_zero(state, "fit output")
    report = {"protocol": "July imitative protocol, pinned", "epochs_requested": int(epochs),
              "epochs_run": len(hist), "best_epoch": int(best_epoch),
              "best_val_mse": (None if fixed else float(best)),
              "selection_mode": ("fixed_final_epoch" if fixed else "early_stopping"),
              "batch_size": J_BATCH, "learning_rate": lr,
              "patience": (None if fixed else patience),
              "clip_weight": J_CLIP_W, "logstd_weight": J_LOGSTD_W, "anchor_weight": J_ANCHOR_W,
              "seed": J_SEED, "gradient_clipping": "none", "ppo_updates": 0,
              "critic": "never loaded or modified",
              "hard_zero_columns": zero_cols,
              "input_mask_columns": list(B0.controller_columns()),
              "history_first_last": {"first": hist[0], "last": hist[-1]},
              "history": hist}
    return state, report


def metrics(state: Mapping[str, np.ndarray], obs_masked: np.ndarray, actions: np.ndarray,
            idx: np.ndarray | None = None) -> dict[str, Any]:
    o = np.asarray(obs_masked, dtype=np.float32)
    a = np.asarray(actions, dtype=np.float32)
    if idx is not None:
        o = o[idx]; a = a[idx]
    m = RF.numpy_mean(dict(state), o)
    d = m - a
    return {"rows": int(o.shape[0]),
            "aggregate_rmse": float(np.sqrt(np.mean(d ** 2))),
            "per_joint": {jn: {"rmse": float(np.sqrt(np.mean(d[:, j] ** 2))),
                               "max_abs": float(np.max(np.abs(d[:, j])))}
                          for j, jn in ((0, "knee"), (1, "ankle"))},
            "max_abs_overall": float(np.max(np.abs(d)))}


def median_best_epoch(values: Sequence[int]) -> dict[str, Any]:
    """Median of the fold best epochs, fixed BEFORE the fit: sort, take the middle for an odd
    count, the mean of the two middles for an even count, then round to the NEAREST integer with
    ties resolved deterministically upward (round-half-up, never banker's rounding)."""
    v = sorted(int(x) for x in values)
    if not v:
        raise ExecError("no best epoch to take the median of")
    n = len(v)
    raw = float(v[n // 2]) if n % 2 == 1 else (v[n // 2 - 1] + v[n // 2]) / 2.0
    chosen = int(np.floor(raw + 0.5))          # round-half-up, deterministic
    if chosen < 1:
        raise ExecError(f"median best epoch {chosen} is not a positive epoch count")
    return {"fold_best_epochs": v, "median_raw": raw, "chosen_epochs": chosen,
            "rule": "sorted median; even counts average the two middles; rounded to the nearest "
                    "integer with ties resolved upward (round-half-up). Fixed before any fit"}


# ============================================================== B1 =================================

def run_b1(*, authorized_stage: str | None, progress: bool = True) -> dict[str, Any]:
    if authorized_stage != STAGE_B1:
        raise ExecError(f"requires --authorized-stage {STAGE_B1}; got {authorized_stage!r}")
    pins = verify_pins()
    if B1_DIR.exists():
        raise FileExistsError(f"no-clobber: {B1_DIR} exists")
    init = load_b0_state()
    b0_digest = RF.actor_state_digest(init)
    data = B1.build_dataset()
    obs = np.asarray(data["observations_masked"], dtype=np.float32)
    act = np.asarray(data["actions"], dtype=np.float32)
    traj = np.asarray(data["trajectory_id"], dtype=np.int64)
    split = B1.trajectory_split(traj)

    folds: list[dict[str, Any]] = []
    for u in sorted(np.unique(traj).tolist()):
        hold = np.where(traj == u)[0]
        keep = np.where(traj != u)[0]
        base_m = metrics(init, obs, act, hold)
        st, rep = fit_masked(init, obs, act, train_idx=keep, val_idx=hold,
                             epochs=J_EPOCHS, patience=J_PATIENCE, progress=progress)
        after = metrics(st, obs, act, hold)
        folds.append({"holdout_trajectory": int(u), "train_rows": int(keep.size),
                      "holdout_rows": int(hold.size), "best_epoch": rep["best_epoch"],
                      "epochs_run": rep["epochs_run"], "best_val_mse": rep["best_val_mse"],
                      "heldout_before_b0": base_m, "heldout_after": after,
                      "improved_over_b0": bool(after["aggregate_rmse"] < base_m["aggregate_rmse"]),
                      "history_first_last": rep["history_first_last"]})
        if progress:
            print(json.dumps({"fold": int(u), "best_epoch": rep["best_epoch"],
                              "heldout_rmse": after["aggregate_rmse"]}), flush=True)

    med = median_best_epoch([f["best_epoch"] for f in folds])
    wsum = sum(f["holdout_rows"] for f in folds)
    loto_weighted = float(sum(f["heldout_after"]["aggregate_rmse"] * f["holdout_rows"]
                              for f in folds) / wsum)

    final_state, final_rep = fit_masked(init, obs, act, train_idx=np.arange(len(obs)),
                                        val_idx=None, epochs=med["chosen_epochs"],
                                        patience=None, progress=progress)
    final_m = metrics(final_state, obs, act, None)

    integrity = {
        "ten_keys": tuple(final_state.keys()) == RF.EXPECTED_KEY_ORDER,
        "width_35": int(np.asarray(final_state["pi.0.0.weight"]).shape[1]) == B0.ACTOR_WIDTH,
        "masked_columns_zero": True, "clock_columns_zero": True,
        "logstd_byte_identical_to_b0": bool(
            np.array_equal(np.asarray(final_state["pi.1.weight"])[R.ACTION_DIM:],
                           np.asarray(init["pi.1.weight"])[R.ACTION_DIM:])
            and np.array_equal(np.asarray(final_state["pi.1.bias"])[R.ACTION_DIM:],
                               np.asarray(init["pi.1.bias"])[R.ACTION_DIM:])),
        "no_masked_update": True, "no_critic": True,
    }
    try:
        B0.assert_masked_columns_zero(final_state, "final")
        B0.assert_clock_columns_zero(final_state, "final")
        B0.assert_no_masked_update(init, final_state)
    except B0.B0Error as exc:
        integrity["masked_columns_zero"] = False
        integrity["no_masked_update"] = False
        integrity["violation"] = str(exc)
    equiv = B0.functional_equivalence_25(final_state, np.asarray(data["observations_raw"], np.float32))

    gates = {
        "integrity_invariants": {"binding": True, **integrity, "pass": bool(all(
            v for k, v in integrity.items() if isinstance(v, bool)))},
        "functional_equivalence_25": {"binding": True, "bit_identical": equiv["bit_identical"],
                                      "pass": bool(equiv["bit_identical"])},
        "every_fold_improves_over_b0": {"binding": True,
                                        "per_fold": [f["improved_over_b0"] for f in folds],
                                        "pass": all(f["improved_over_b0"] for f in folds)},
        "fold_heldout_rmse": {"binding": True, "threshold": FOLD_HELDOUT_RMSE_MAX,
                              "observed": [f["heldout_after"]["aggregate_rmse"] for f in folds],
                              "pass": all(f["heldout_after"]["aggregate_rmse"] <= FOLD_HELDOUT_RMSE_MAX
                                          for f in folds)},
        "loto_weighted_mean": {"binding": True, "threshold": LOTO_WEIGHTED_MEAN_MAX,
                               "observed": loto_weighted,
                               "pass": bool(loto_weighted <= LOTO_WEIGHTED_MEAN_MAX)},
        "final_aggregate_rmse": {"binding": True, "threshold": FINAL_RMSE_MAX,
                                 "observed": final_m["aggregate_rmse"],
                                 "pass": bool(final_m["aggregate_rmse"] <= FINAL_RMSE_MAX)},
        "final_per_joint_rmse": {"binding": True, "threshold": FINAL_PER_JOINT_RMSE_MAX,
                                 "observed": {k: v["rmse"] for k, v in final_m["per_joint"].items()},
                                 "pass": all(v["rmse"] <= FINAL_PER_JOINT_RMSE_MAX
                                             for v in final_m["per_joint"].values())},
        "final_per_joint_max_abs": {"binding": True, "threshold": FINAL_PER_JOINT_MAXABS_MAX,
                                    "observed": {k: v["max_abs"] for k, v in final_m["per_joint"].items()},
                                    "pass": all(v["max_abs"] <= FINAL_PER_JOINT_MAXABS_MAX
                                                for v in final_m["per_joint"].values())},
    }
    failed = [k for k, v in gates.items() if not v["pass"]]
    verdict = "GO" if not failed else "NO-GO"

    staging = B1_DIR.parent / (B1_DIR.name + ".staging")
    if staging.exists():
        shutil.rmtree(staging)
    sm = staging / MODULE_DIR_NAME
    sm.mkdir(parents=True, exist_ok=False)
    csha = None
    try:
        names35, _, mshas = VS.pinned_names()
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in final_state.items()}, fh,
                        protocol=pickle.HIGHEST_PROTOCOL)
        for f in ("metadata.json", "class_and_ctor_args.pkl"):
            shutil.copy2(B0_DIR / MODULE_DIR_NAME / f, sm / f)
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35,
                    "actor_digest": RF.actor_state_digest(final_state),
                    "module_state_sha256": C.sha256_file(sm / "module_state.pkl"),
                    "manifest35_sha256": mshas["manifest35_sha256"],
                    "exploration_sigma": [B0.SIGMA_PLACEHOLDER] * R.ACTION_DIM,
                    "sigma_note": B0.SIGMA_STATEMENT,
                    "derived_from": C.rel(B0_DIR / MODULE_DIR_NAME),
                    "source_actor_digest": b0_digest,
                    "deployable": False, "sigma_unresolved": True,
                    "actor_label": "B1_BASE35_MASKED",
                    "controller_state_mask": {"active": True, "columns": list(B0.controller_columns()),
                                              "mask_value": float(B0.MASK_VALUE)},
                    "clock_columns": list(B0.CLOCK_COLUMNS),
                    "offline_verdict": verdict,
                    "status": "BASE-PHASE CANDIDATE (B1); never promoted, never deployable; no "
                              "rollout has been run"}
        C.write_json(sm / "actor_feature_manifest.json", manifest, clobber=False)
        receipt = {"schema": "v26b_b1_base_fit.1", "authorized_stage": STAGE_B1, "pins": pins,
                   "deployable": False, "sigma_unresolved": True,
                   "parent": {"module": C.rel(B0_DIR / MODULE_DIR_NAME), "actor_digest": b0_digest,
                              "note": "every fold and the final fit start from THIS state"},
                   "dataset": data["report"], "split": split,
                   "july_protocol": {k: v for k, v in final_rep.items()
                                     if k not in ("history", "history_first_last")},
                   "folds": folds,
                   "median_best_epoch": med,
                   "loto_weighted_mean_rmse": loto_weighted,
                   "final_fit": {"epochs": med["chosen_epochs"],
                                 "selection_mode": final_rep["selection_mode"],
                                 "history_first_last": final_rep["history_first_last"],
                                 "metrics_all_1500_rows": final_m},
                   "final_history": final_rep["history"],
                   "functional_equivalence_25": equiv,
                   "gates": gates, "failed": failed, "verdict": verdict,
                   "output_module": C.rel(B1_DIR / MODULE_DIR_NAME),
                   "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(sm.iterdir())},
                   "scope": "base-phase fit only. No rollout, no collection, no Markov phase, no "
                            "promotion. A NO-GO forbids any rollout and forbids post-hoc "
                            "hyperparameter correction",
                   "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
        C.write_json(staging / B1_RECEIPT, receipt, clobber=False)
        csha = C.sha256_file(staging / B1_RECEIPT)
        if B1_DIR.exists():
            raise ExecError(f"no-clobber: {B1_DIR} appeared during staging")
        staging.rename(B1_DIR)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return {"verdict": verdict, "failed": failed, "receipt_sha256": csha,
            "median_best_epoch": med, "loto_weighted_mean_rmse": loto_weighted,
            "final_metrics": final_m,
            "fold_summary": [{"holdout": f["holdout_trajectory"], "best_epoch": f["best_epoch"],
                              "heldout_rmse": f["heldout_after"]["aggregate_rmse"],
                              "improved": f["improved_over_b0"]} for f in folds]}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B executable: materialise B0, run B1")
    p.add_argument("--stage", choices=("b0", "b1"), required=True)
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.stage == "b0":
        print(json.dumps(materialize_b0(authorized_stage=a.authorized_stage), indent=2, default=str))
    else:
        out = run_b1(authorized_stage=a.authorized_stage, progress=not a.no_progress)
        print(json.dumps(out, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
