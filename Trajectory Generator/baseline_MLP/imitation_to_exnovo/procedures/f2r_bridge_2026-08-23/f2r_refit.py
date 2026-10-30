"""Student refit for the F2R bridge (rev 3) — tooling only in S0 (no real fit).

Student = the deployable 35D stateless MLP (35-256-256-2 mean + 2 log-std),
initialised from the **pinned JUL_H0** state (never B0820_H0) and trained
on labelled rows (obs35 -> teacher action) with:

* hard constraints after every optimiser step and at export:
  ``W1[:, clock_columns] == 0`` (the two dead prescribed-clock inputs) and the
  log-std head fixed (rows 0, bias ``ln 0.005`` float32);
* an **auxiliary phase head** ``h`` (linear on the second hidden layer) trained
  against the privileged label ``c(t) = (sin, cos)`` of the prescribed clock
  — training-time only: it is never exported and the exported checkpoint
  carries exactly the original 10 ``pi*`` keys of the init;
* an L2 anchor toward the previous round's parameters (``lambda_anchor``);
* a frozen budget (epochs, batch, lr, seed) from ``f2r_protocol.json``;
* **no validation split inside the fit** (selection is closed-loop only, and
  seed 125 rows can never be present: structural assert).

Export verification: keys/shapes/dtypes identical to the init, encoder
aliases bit-identical, clock columns zero, constant-sigma invariants
(``f1_sigma_variant.constant_sigma_invariants``), save/reload bit-exact, and
an **invariance test**: perturbing indices 0:2 of the input leaves the mean
bit-identical (numpy forward).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402

CANONICAL_KEYS = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")
ALIAS_KEYS = {"pi_encoder.0.weight": "pi.0.0.weight", "pi_encoder.0.bias": "pi.0.0.bias", "pi_encoder.2.weight": "pi.0.2.weight", "pi_encoder.2.bias": "pi.0.2.bias"}
EXPECTED_KEY_ORDER = ("pi_encoder.0.weight", "pi_encoder.0.bias", "pi_encoder.2.weight", "pi_encoder.2.bias", "pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")


class RefitError(RuntimeError):
    pass


def _sha_array(arr: np.ndarray) -> str:
    a = np.ascontiguousarray(arr)
    h = hashlib.sha256(); h.update(str(a.dtype).encode()); h.update(repr(tuple(int(d) for d in a.shape)).encode()); h.update(a.tobytes(order="C"))
    return h.hexdigest()


def actor_state_digest(state: Mapping[str, Any]) -> str:
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W  # production, import only

    return W.actor_state_digest(state)


def validate_init_state(state: Mapping[str, Any], *, expected_actor_digest: str | None, width: int = R.ENV_ACTOR_WIDTH, sigma: float = R.SIGMA_CONSTANT) -> dict[str, Any]:
    """Structural checks of an init/student state (fail-closed)."""
    keys = tuple(state.keys())
    if keys != EXPECTED_KEY_ORDER:
        raise RefitError(f"state keys/order {keys} != expected 10 pi* keys")
    for alias, canon in ALIAS_KEYS.items():
        if not np.array_equal(np.asarray(state[alias]), np.asarray(state[canon])):
            raise RefitError(f"alias {alias} not bit-identical to {canon}")
    w1 = np.asarray(state["pi.0.0.weight"]); w2 = np.asarray(state["pi.0.2.weight"]); w3 = np.asarray(state["pi.1.weight"]); b3 = np.asarray(state["pi.1.bias"])
    if w1.dtype != np.float32 or any(np.asarray(state[k]).dtype != np.float32 for k in keys):
        raise RefitError("all actor tensors must be float32")
    if w1.shape[1] != width or w3.shape != (2 * R.ACTION_DIM, w2.shape[0]) or b3.shape != (2 * R.ACTION_DIM,):
        raise RefitError(f"ABI mismatch: W1 {w1.shape}, W3 {w3.shape}, b3 {b3.shape}")
    if not np.all(w1[:, list(R.CLOCK_COLUMNS)] == 0.0):
        raise RefitError("clock columns of W1 are not zero (deployable contract)")
    ok, head = SV.constant_sigma_invariants(dict(state), sigma=sigma)
    if not ok:
        raise RefitError(f"log-std head is not the constant-sigma head: {head}")
    digest = actor_state_digest(state)
    if expected_actor_digest is not None and digest != expected_actor_digest:
        raise RefitError(f"actor digest {digest} != pinned {expected_actor_digest}")
    return {"keys": list(keys), "shapes": {k: list(np.asarray(v).shape) for k, v in state.items()}, "hidden": int(w2.shape[0]), "actor_digest": digest, "clock_columns_zero": True, "sigma_head": head}


def load_init_state(module_dir: Path | None = None, *, expected_actor_digest: str | None = R.INIT_PRIMARY["actor_digest"]) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load the pinned init (JUL_H0 by default) and validate it structurally."""
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W

    module_dir = Path(module_dir) if module_dir is not None else Path(R.INIT_PRIMARY["module"])
    state = W.load_module_state(module_dir)
    info = validate_init_state(state, expected_actor_digest=expected_actor_digest)
    info["module"] = C.rel(module_dir)
    info["module_state_sha256"] = C.sha256_file(module_dir / "module_state.pkl")
    return dict(state), info


def numpy_mean(state: Mapping[str, Any], obs: np.ndarray) -> np.ndarray:
    arrays = DS.load_actor_arrays(dict(state), expected_width=int(np.asarray(state["pi.0.0.weight"]).shape[1]))
    return DS.actor_logits_numpy(arrays, np.asarray(obs, dtype=np.float64))[:, : R.ACTION_DIM]


def assert_dataset_contract(data: Mapping[str, Any]) -> dict[str, Any]:
    """Structural dataset checks before any fit: widths, dtypes, seeds (125 / sealed never), purposes."""
    obs = np.asarray(data["obs35"]); act = np.asarray(data["actions"]); clk = np.asarray(data["clock"])
    n = obs.shape[0]
    if obs.ndim != 2 or obs.shape[1] != R.ENV_ACTOR_WIDTH:
        raise R.F2RContractError(f"obs35 must be (N,35), got {obs.shape}")
    if act.shape != (n, R.ACTION_DIM) or clk.shape != (n, 2):
        raise R.F2RContractError("actions must be (N,2) and clock (N,2)")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act)) and np.all(np.isfinite(clk))):
        raise R.F2RContractError("non-finite values in the dataset")
    seeds = np.asarray(data.get("seed", np.full(n, R.DET_SEED)), dtype=np.int64).reshape(-1)
    if seeds.shape[0] != n:
        raise R.F2RContractError("seed array length mismatch")
    for s in sorted({int(v) for v in seeds}):
        R.assert_collection_seed(s)
    purposes = data.get("purpose")
    if purposes is not None:
        bad = sorted({str(p) for p in np.asarray(purposes).reshape(-1)} - {"anchor", "det", "stoch"})
        if bad:
            raise R.F2RContractError(f"invalid dataset purposes {bad}")
    forbidden = [k for k in data.keys() if k in ("time", "t", "step", "index", "t_pre_as_feature")]
    # t_pre may be stored for provenance, but it is never a feature: the fit reads only obs35
    return {"rows": int(n), "seeds": sorted({int(v) for v in seeds}), "forbidden_feature_keys_present": forbidden}


def fit_student(init_state: Mapping[str, Any], data: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, anchor_state: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """Train the mean network (+ training-time auxiliary phase head) and return
    (new_state with exactly the 10 pi* keys, report).  ``anchor_state`` =
    theta_{r-1} (defaults to the init)."""
    import torch
    import torch.nn.functional as F

    budget = dict(budget or R.REFIT_BUDGET)
    contract = assert_dataset_contract(data)
    info = validate_init_state(init_state, expected_actor_digest=None)
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(int(budget["seed"]))
    np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.asarray(data["obs35"], dtype=f32))
    tgt = torch.as_tensor(np.asarray(data["actions"], dtype=f32))
    clk = torch.as_tensor(np.asarray(data["clock"], dtype=f32))
    anchor = dict(anchor_state) if anchor_state is not None else dict(init_state)
    validate_init_state(anchor, expected_actor_digest=None)
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.weight"], dtype=f32)))
    b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.weight"], dtype=f32)))
    b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.weight"][: R.ACTION_DIM], dtype=f32)))
    b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    hidden = int(W2.shape[0])
    g = torch.Generator().manual_seed(int(budget["seed"]))
    Wh = torch.nn.Parameter(torch.randn(2, hidden, generator=g) * 0.01)   # auxiliary phase head (training-time only)
    bh = torch.nn.Parameter(torch.zeros(2))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [torch.as_tensor(np.array(anchor[k], dtype=f32)) for k in ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias")] + [torch.as_tensor(np.array(anchor["pi.1.weight"][: R.ACTION_DIM], dtype=f32)), torch.as_tensor(np.array(anchor["pi.1.bias"][: R.ACTION_DIM], dtype=f32))]
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32)
    logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)

    def encoder(x: torch.Tensor) -> torch.Tensor:
        return torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)

    def mean_of(h2: torch.Tensor) -> torch.Tensor:
        return h2 @ W3m.T + b3m

    opt = torch.optim.Adam(mean_params + [Wh, bh], lr=float(budget["lr"]))
    rng = np.random.default_rng(int(budget["seed"]))
    n = int(obs.shape[0]); epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip, lam_phi, lam_a = float(budget["clip_weight"]), float(budget["lambda_phi"]), float(budget["lambda_anchor"])
    history = []
    steps = 0

    def eval_all() -> dict[str, float]:
        with torch.no_grad():
            h2 = encoder(obs); m = mean_of(h2); hp = h2 @ Wh.T + bh
            rmse = float(torch.sqrt(torch.mean((m - tgt) ** 2)).item())
            ph = torch.atan2(hp[:, 0], hp[:, 1]) / (2 * np.pi); pt = torch.atan2(clk[:, 0], clk[:, 1]) / (2 * np.pi)
            d = torch.remainder(ph - pt + 0.5, 1.0) - 0.5
            return {"train_rmse": rmse, "aux_phase_err_median": float(torch.median(torch.abs(d)).item()), "aux_phase_err_p90": float(torch.quantile(torch.abs(d), 0.9).item())}

    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []
        for s0 in range(0, n, batch):
            idx = perm[s0: s0 + batch]
            opt.zero_grad(set_to_none=True)
            h2 = encoder(obs[idx]); m = mean_of(h2); hp = h2 @ Wh.T + bh
            loss = F.mse_loss(m, tgt[idx]) + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + lam_phi * F.mse_loss(hp, clk[idx])
            if lam_a > 0.0:
                loss = loss + lam_a * torch.stack([(p - a0).square().sum() for p, a0 in zip(mean_params, anchor_t)]).sum()
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item()))
        ev = eval_all(); history.append({"epoch": epoch, "loss": float(np.mean(losses)), **ev})
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps(history[-1]))
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0)
    b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state: dict[str, Any] = {}
    for key in EXPECTED_KEY_ORDER:  # exact original order; NO auxiliary head keys
        new_state[key] = canon[key].copy() if key in canon else canon[ALIAS_KEYS[key]].copy()
    validate_init_state(new_state, expected_actor_digest=None)
    aux_digest = _sha_array(Wh.detach().numpy()) + ":" + _sha_array(bh.detach().numpy())
    init_mean = numpy_mean(init_state, np.asarray(data["obs35"], dtype=f32)); fit_mean = numpy_mean(new_state, np.asarray(data["obs35"], dtype=f32))
    report = {
        "budget": budget, "optimizer_steps": int(steps), "epochs_run": epochs, "rows": contract["rows"], "seeds": contract["seeds"], "selection": "closed_loop_only", "validation_split_in_fit": False,
        "init_actor_digest": info["actor_digest"], "anchor_actor_digest": actor_state_digest(anchor), "new_actor_digest": actor_state_digest(new_state),
        "rmse_vs_labels": {"init": float(np.sqrt(np.mean((init_mean - np.asarray(data["actions"], dtype=np.float64)) ** 2))), "fit": float(np.sqrt(np.mean((fit_mean - np.asarray(data["actions"], dtype=np.float64)) ** 2)))},
        "history": history, "aux_head": {"exported": False, "training_time_only": True, "param_digest": aux_digest, "final": history[-1] if history else None},
        "structure": validate_init_state(new_state, expected_actor_digest=None),
    }
    return new_state, report


T1R_CRITERIA = {  # preregistered offline fail-closed criteria of the corrective commissioning T1R (report section 14.15/14.16)
    "P1_preservation_rmse_max_per_joint": 0.10,   # deviation from JUL_H0 on the JUL_H0 states, per joint
    "P2_ankle_rmse_max_anchors": 0.10,             # ankle vs u_IK on the anchor rows
    "P2_ankle_rmse_max_t1_states": 0.15,           # ankle vs u_IK on the T1 on-policy rows
    "P3_knee_rmse_max_anchors": 0.50,              # knee vs u_T on the anchor rows (sanity)
    "P3b_knee_rmse_max_t1_states": 0.75,           # knee vs u_T on the T1 on-policy rows (mandatory)
    "P4_invariants": ["clock_columns_zero", "logstd_constant", "ten_keys", "save_reload_exact", "clock_invariance_bit_identical"],
    "P5_parameter_shift": "informational",
}
T1R_BETA = 1.0


def assert_preservation_contract(task: Mapping[str, Any], pres: Mapping[str, Any]) -> dict[str, Any]:
    """Structural checks of the two T1R roles before any fit: both pass ``assert_dataset_contract``
    (seed 125 / sealed never); task rows carry purposes {anchor, det} (T1 on-policy states are
    ``det`` seed 123) and preservation rows purpose ``det`` only; widths; finite; no shared key."""
    ct = assert_dataset_contract(task); cp = assert_dataset_contract(pres)
    tp = {str(x) for x in np.asarray(task.get("purpose", [])).reshape(-1)}; pp = {str(x) for x in np.asarray(pres.get("purpose", [])).reshape(-1)}
    if not tp <= {"anchor", "det"} or "anchor" not in tp:
        raise R.F2RContractError(f"task roles must be anchors (+ det T1 states), got purposes {sorted(tp)}")
    if pp != {"det"}:
        raise R.F2RContractError(f"preservation rows must be det seed-123 JUL_H0 states, got purposes {sorted(pp)}")
    if ct["rows"] == 0 or cp["rows"] == 0:
        raise R.F2RContractError("empty role")
    return {"task": ct, "preservation": cp}


def normalised_role_term(m: Any, y: Any, mask: Any, var: Any) -> Any:
    """Group-balanced normalised loss of ONE role inside a mini-batch (torch tensors):
    ``mean_j [ mean_{rows of the role in the batch} ((m_j - y_j)^2 / var_j) ]`` — a per-role,
    per-joint mean, independent of how many rows of the role the batch contains (the other role's
    rows never enter).  Zero when the batch holds no row of the role."""
    import torch

    if not bool(mask.any()):
        return torch.zeros((), dtype=m.dtype)
    return (((m[mask] - y[mask]) ** 2) / var).mean(dim=0).mean()


def fit_student_preserving(init_state: Mapping[str, Any], task: Mapping[str, Any], pres: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, beta: float = T1R_BETA, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """T1R corrective refit: the ``fit_student`` graph (encoder 35-H-H tanh, mean head, training-time
    auxiliary phase head, clip penalty, clock columns re-zeroed, parameter anchor) with TWO roles and
    a scale-free loss::

        L = mean_j  E_task[(m_j - y_j)^2] / Var_task_j  +  beta * mean_j  E_pres[(m_j - a_j)^2] / Var_pres_j
            + lambda_phi * L_aux(all rows) + lambda_clip * clip(all rows) + lambda_a * ||theta - theta_0||^2

    ``Var_task_j`` / ``Var_pres_j`` are the per-joint variances of the task labels / preservation
    targets, fixed a priori from the training rows and reported.  Within every mini-batch the task
    and preservation terms are **per-role, per-joint means** (group-balanced: neither term is weighted
    by its row count).  Returns (state with the 10 pi* keys, report) — the report carries the
    preregistered T1R criteria P1-P5 evaluated on the training rows (P4 via ``validate_init_state`` +
    ``invariance_test``; save/reload is verified at export) and ``criteria_pass`` (P1, P2, P3, P3b, P4)."""
    import torch
    import torch.nn.functional as F

    budget = dict(budget or R.REFIT_BUDGET)
    contract = assert_preservation_contract(task, pres)
    info = validate_init_state(init_state, expected_actor_digest=None)
    beta = float(beta)
    if not (beta > 0.0 and math.isfinite(beta)):
        raise R.F2RContractError("beta must be a positive finite number")
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.concatenate([np.asarray(task["obs35"], dtype=f32), np.asarray(pres["obs35"], dtype=f32)]))
    tgt = torch.as_tensor(np.concatenate([np.asarray(task["actions"], dtype=f32), np.asarray(pres["actions"], dtype=f32)]))
    clk = torch.as_tensor(np.concatenate([np.asarray(task["clock"], dtype=f32), np.asarray(pres["clock"], dtype=f32)]))
    n_task, n_pres = int(np.asarray(task["obs35"]).shape[0]), int(np.asarray(pres["obs35"]).shape[0])
    is_task = torch.as_tensor(np.concatenate([np.ones(n_task, bool), np.zeros(n_pres, bool)]))
    var_task = np.var(np.asarray(task["actions"], dtype=np.float64), axis=0); var_pres = np.var(np.asarray(pres["actions"], dtype=np.float64), axis=0)
    if np.any(var_task <= 0.0) or np.any(var_pres <= 0.0):
        raise R.F2RContractError("degenerate per-joint variance in a role (normalisation undefined)")
    vt = torch.as_tensor(var_task.astype(f32)); vp = torch.as_tensor(var_pres.astype(f32))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.weight"], dtype=f32))); b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.weight"], dtype=f32))); b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.weight"][: R.ACTION_DIM], dtype=f32))); b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_state["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    hidden = int(W2.shape[0]); g = torch.Generator().manual_seed(int(budget["seed"]))
    Wh = torch.nn.Parameter(torch.randn(2, hidden, generator=g) * 0.01); bh = torch.nn.Parameter(torch.zeros(2))
    mean_params = [W1, b1, W2, b2, W3m, b3m]; anchor_t = [p.detach().clone() for p in mean_params]
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32); logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    mean_of = lambda h2: h2 @ W3m.T + b3m  # noqa: E731
    opt = torch.optim.Adam(mean_params + [Wh, bh], lr=float(budget["lr"])); rng = np.random.default_rng(int(budget["seed"]))
    n = n_task + n_pres; epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip, lam_phi, lam_a = float(budget["clip_weight"]), float(budget["lambda_phi"]), float(budget["lambda_anchor"])

    role_term = normalised_role_term
    history = []; steps = 0
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []; lt_acc = []; lp_acc = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            h2 = encoder(obs[idx]); m = mean_of(h2); hp = h2 @ Wh.T + bh; mb = is_task[idx]
            l_task = role_term(m, tgt[idx], mb, vt); l_pres = role_term(m, tgt[idx], ~mb, vp)
            loss = l_task + beta * l_pres + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + lam_phi * F.mse_loss(hp, clk[idx])
            if lam_a > 0.0:
                loss = loss + lam_a * torch.stack([(p - a0).square().sum() for p, a0 in zip(mean_params, anchor_t)]).sum()
            loss.backward(); opt.step(); steps += 1
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); lt_acc.append(float(l_task.item())); lp_acc.append(float(l_pres.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "task_norm": float(np.mean(lt_acc)), "pres_norm": float(np.mean(lp_acc))})
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps(history[-1]))
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state: dict[str, Any] = {}
    for key in EXPECTED_KEY_ORDER:
        new_state[key] = canon[key].copy() if key in canon else canon[ALIAS_KEYS[key]].copy()
    struct = validate_init_state(new_state, expected_actor_digest=None)
    report = {
        "tool": "fit_student_preserving", "variant": "T1R", "beta": beta, "budget": budget, "optimizer_steps": int(steps), "epochs_run": epochs, "rows": {"task": n_task, "preservation": n_pres}, "seeds": {"task": contract["task"]["seeds"], "preservation": contract["preservation"]["seeds"]},
        "normalisation": {"var_task_per_joint": var_task.tolist(), "var_pres_per_joint": var_pres.tolist(), "rule": "per-joint variance of the role targets, fixed a priori; per-role per-joint means (group-balanced, not row-count weighted)"},
        "selection": "closed_loop_only", "validation_split_in_fit": False, "init_actor_digest": info["actor_digest"], "new_actor_digest": actor_state_digest(new_state), "history": history,
        "aux_head": {"exported": False, "training_time_only": True, "param_digest": _sha_array(Wh.detach().numpy()) + ":" + _sha_array(bh.detach().numpy())}, "structure": struct,
    }
    report["criteria"] = evaluate_t1r_criteria(init_state, new_state, task, pres)
    return new_state, report


def evaluate_t1r_criteria(init_state: Mapping[str, Any], state: Mapping[str, Any], task: Mapping[str, Any], pres: Mapping[str, Any], *, criteria: Mapping[str, Any] = T1R_CRITERIA) -> dict[str, Any]:
    """Preregistered T1R criteria on the training rows (no closed-loop, no held-out seed):
    P1 preservation RMSE per joint on the JUL rows; P2 ankle RMSE vs u_IK on anchors / T1 states;
    P3 knee RMSE vs u_T on anchors; P3b knee on T1 states; P4 invariants (clock columns, log-std,
    10 keys, clock invariance; save/reload is checked at export); P5 parameter shift (informational).
    ``pass_all`` = P1 and P2 and P3 and P3b and P4."""
    f32 = np.float32
    pm = numpy_mean(state, np.asarray(pres["obs35"], dtype=f32)); pa = np.asarray(pres["actions"], dtype=np.float64)
    p1 = np.sqrt(np.mean((pm - pa) ** 2, axis=0)).tolist()
    tobs = np.asarray(task["obs35"], dtype=f32); tl = np.asarray(task["actions"], dtype=np.float64); tp = np.asarray(task["purpose"]).reshape(-1)
    tm = numpy_mean(state, tobs)
    anc = tp == "anchor"; det = tp == "det"
    def rm(mask, j):
        return float(np.sqrt(np.mean((tm[mask, j] - tl[mask, j]) ** 2))) if mask.any() else None
    p2a, p2t = rm(anc, 1), rm(det, 1); p3a, p3t = rm(anc, 0), rm(det, 0)
    struct = validate_init_state(state, expected_actor_digest=None); inv = invariance_test(state, tobs[: min(64, tobs.shape[0])])
    p4 = {"clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant": bool(struct["sigma_head"]["logstd_bias_exact"]), "ten_keys": tuple(state.keys()) == EXPECTED_KEY_ORDER, "clock_invariance_bit_identical": bool(inv["bit_identical"])}
    shift = float(sum(np.sum((np.asarray(state[k], np.float64) - np.asarray(init_state[k], np.float64)) ** 2) for k in CANONICAL_KEYS))
    out = {
        "P1": {"value_knee_ankle": p1, "max": criteria["P1_preservation_rmse_max_per_joint"], "pass": bool(all(v <= criteria["P1_preservation_rmse_max_per_joint"] for v in p1))},
        "P2": {"ankle_rmse_anchors": p2a, "ankle_rmse_t1_states": p2t, "max_anchors": criteria["P2_ankle_rmse_max_anchors"], "max_t1_states": criteria["P2_ankle_rmse_max_t1_states"], "pass": bool(p2a is not None and p2a <= criteria["P2_ankle_rmse_max_anchors"] and (p2t is None or p2t <= criteria["P2_ankle_rmse_max_t1_states"]))},
        "P3": {"knee_rmse_anchors": p3a, "max": criteria["P3_knee_rmse_max_anchors"], "pass": bool(p3a is not None and p3a <= criteria["P3_knee_rmse_max_anchors"])},
        "P3b": {"knee_rmse_t1_states": p3t, "max": criteria["P3b_knee_rmse_max_t1_states"], "pass": bool(p3t is not None and p3t <= criteria["P3b_knee_rmse_max_t1_states"]), "mandatory": True},
        "P4": {**p4, "pass": bool(all(p4.values())), "save_reload_exact": "verified at export"},
        "P5": {"param_shift_sq": shift, "max_abs_dW1": float(np.max(np.abs(np.asarray(state["pi.0.0.weight"], np.float64) - np.asarray(init_state["pi.0.0.weight"], np.float64)))), "informational": True},
    }
    out["pass_all"] = bool(out["P1"]["pass"] and out["P2"]["pass"] and out["P3"]["pass"] and out["P3b"]["pass"] and out["P4"]["pass"])
    return out


def assert_t1r_criteria(report: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed gate before export/rollout: every mandatory criterion (P1, P2, P3, P3b, P4) must pass."""
    c = report.get("criteria") if isinstance(report, Mapping) else None
    if not isinstance(c, Mapping) or c.get("pass_all") is not True:
        failed = [k for k in ("P1", "P2", "P3", "P3b", "P4") if not (isinstance(c, Mapping) and isinstance(c.get(k), Mapping) and c[k].get("pass") is True)]
        raise RefitError(f"T1R offline criteria failed {failed}: export and rollouts are refused")
    return dict(c)


def invariance_test(state: Mapping[str, Any], obs: np.ndarray, *, rng_seed: int = 0) -> dict[str, Any]:
    """Mean must be bit-identical when indices 0:2 are perturbed (clock columns zero)."""
    rng = np.random.default_rng(rng_seed)
    x = np.asarray(obs, dtype=np.float32).copy(); y = x.copy()
    y[:, list(R.CLOCK_COLUMNS)] = rng.standard_normal((x.shape[0], len(R.CLOCK_COLUMNS))).astype(np.float32) * 10.0
    m0 = numpy_mean(state, x); m1 = numpy_mean(state, y)
    return {"rows": int(x.shape[0]), "bit_identical": bool(np.array_equal(m0, m1)), "max_abs_diff": float(np.abs(m0 - m1).max()) if x.size else 0.0}


STAGING_PREFIX = ".staging-"
LOCK_SUFFIX = ".lock"
COMPLETION_MARKER = "f2r_refit_report.json"  # moved/renamed LAST: an out_dir without it is an incomplete export


class NoReplaceUnavailable(OSError):
    """The platform/filesystem offers no atomic no-replace directory rename (fallback path is used)."""


class ExportLockError(FileExistsError):
    """Another cooperative export holds the final path (lock present) — the caller is the loser."""


def atomic_noreplace_rename_dir(src: Path, dst: Path) -> str:
    """Rename ``src`` -> ``dst`` atomically and FAIL with ``FileExistsError`` if ``dst`` exists —
    never replacing an existing file or (even empty) directory — using the native primitive:

    * Darwin: ``renameatx_np(AT_FDCWD, src, AT_FDCWD, dst, RENAME_EXCL)`` (libSystem, macOS >= 10.12);
    * Linux: ``renameat2(AT_FDCWD, src, AT_FDCWD, dst, RENAME_NOREPLACE)`` (glibc >= 2.28, kernel >= 3.15);
    * Windows: ``os.rename`` = ``MoveFileExW`` without ``MOVEFILE_REPLACE_EXISTING`` (a directory target
      is never replaced; ``FileExistsError`` if it exists).

    A plain POSIX ``rename(2)`` is NEVER used for this purpose: it silently replaces an empty
    destination directory.  Raises ``NoReplaceUnavailable`` when the primitive is missing or the
    filesystem rejects the flag (ENOTSUP/EINVAL/ENOSYS) so the caller can take the fail-closed fallback.
    Returns the name of the primitive used."""
    import ctypes
    import ctypes.util
    import errno as _errno
    import platform

    src_b, dst_b = os.fsencode(str(src)), os.fsencode(str(dst))
    system = platform.system()
    if system == "Windows":
        try:
            os.rename(src, dst)
        except FileExistsError:
            raise
        except OSError as exc:
            if exc.winerror in (183, 80) or exc.errno == _errno.EEXIST:  # ERROR_ALREADY_EXISTS / ERROR_FILE_EXISTS
                raise FileExistsError(str(exc)) from exc
            raise
        return "windows_MoveFileExW_no_replace"
    if system == "Darwin":
        lib = ctypes.CDLL(ctypes.util.find_library("System") or "libSystem.B.dylib", use_errno=True)
        fn = getattr(lib, "renameatx_np", None)
        if fn is None:
            raise NoReplaceUnavailable("renameatx_np not available")
        fn.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]; fn.restype = ctypes.c_int
        at_fdcwd, rename_excl = -2, 0x4
        rc = fn(at_fdcwd, src_b, at_fdcwd, dst_b, rename_excl)
        if rc != 0:
            err = ctypes.get_errno()
            if err in (_errno.EEXIST, _errno.ENOTEMPTY):
                raise FileExistsError(f"destination exists (no-replace rename refused): {dst}")
            if err in (_errno.ENOTSUP, _errno.EINVAL, _errno.ENOSYS, getattr(_errno, "EOPNOTSUPP", _errno.ENOTSUP)):
                raise NoReplaceUnavailable(f"renameatx_np RENAME_EXCL unsupported here (errno {err})")
            raise OSError(err, os.strerror(err), str(src), None, str(dst))
        return "darwin_renameatx_np_RENAME_EXCL"
    if system == "Linux":
        lib = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
        fn = getattr(lib, "renameat2", None)
        if fn is None:
            raise NoReplaceUnavailable("renameat2 not available")
        fn.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]; fn.restype = ctypes.c_int
        at_fdcwd, rename_noreplace = -100, 0x1
        rc = fn(at_fdcwd, src_b, at_fdcwd, dst_b, rename_noreplace)
        if rc != 0:
            err = ctypes.get_errno()
            if err in (_errno.EEXIST, _errno.ENOTEMPTY):
                raise FileExistsError(f"destination exists (no-replace rename refused): {dst}")
            if err in (_errno.EINVAL, _errno.ENOSYS, _errno.ENOTSUP, getattr(_errno, "EOPNOTSUPP", _errno.ENOTSUP)):
                raise NoReplaceUnavailable(f"renameat2 RENAME_NOREPLACE unsupported here (errno {err})")
            raise OSError(err, os.strerror(err), str(src), None, str(dst))
        return "linux_renameat2_RENAME_NOREPLACE"
    raise NoReplaceUnavailable(f"no native no-replace rename on {system}")


def promote_by_exclusive_mkdir(staging: Path, out_dir: Path) -> str:
    """Fail-closed fallback when no native no-replace rename exists: ``os.mkdir(out_dir)`` (exclusive
    and atomic on every platform: fails if anything exists there) then move the staged entries into
    the directory WE created, the completion marker (``COMPLETION_MARKER``) last.  Never replaces
    anything; the final directory may be observed partially populated for a short window — a reader
    must treat an ``out_dir`` without the marker as incomplete.  On a failure mid-way the entries
    already moved are moved back to staging and the (empty) final directory is removed; foreign
    content that appeared inside it is never deleted (rmdir only)."""
    os.mkdir(out_dir)  # exclusive creation: FileExistsError if the final path exists in any form
    entries = sorted(staging.iterdir(), key=lambda q: (q.name == COMPLETION_MARKER, q.name))  # marker last
    moved: list[tuple[Path, Path]] = []
    try:
        for child in entries:
            target = out_dir / child.name
            if target.exists():
                raise FileExistsError(f"foreign entry inside the freshly created final directory: {target}")
            os.rename(child, target)  # target does not exist: nothing can be replaced
            moved.append((child, target))
    except BaseException:
        for child, target in reversed(moved):
            try:
                os.rename(target, child)
            except OSError:
                pass
        try:
            os.rmdir(out_dir)  # only if empty: never deletes foreign content
        except OSError:
            pass
        raise
    try:
        os.rmdir(staging)  # the staging is now empty (ours); best effort, never raises after the marker moved
    except OSError:
        pass
    return "fallback_exclusive_mkdir_then_child_moves"


def promote_staging(staging: Path, out_dir: Path) -> dict[str, Any]:
    """Promotion step (called under the export lock, after the final-absent re-check): native atomic
    no-replace rename, else the exclusive-mkdir fallback; never a plain directory rename."""
    try:
        method = atomic_noreplace_rename_dir(staging, out_dir)
        return {"method": method, "single_step_atomic": True}
    except NoReplaceUnavailable as exc:
        method = promote_by_exclusive_mkdir(staging, out_dir)
        return {"method": method, "single_step_atomic": False, "native_unavailable": str(exc), "staging_removed": not staging.exists()}


def _lock_path_for(out_dir: Path) -> Path:
    return out_dir.parent / f".{out_dir.name}{LOCK_SUFFIX}"


def acquire_export_lock(out_dir: Path) -> tuple[Path, str]:
    """Cooperative mutual exclusion per final path: exclusive creation (``O_CREAT | O_EXCL``, atomic on
    POSIX and Windows) of the sibling ``.<name>.lock`` holding ``{pid, token, final, created_utc}``.
    Non-blocking: if the lock exists the caller is the loser (``ExportLockError``, a ``FileExistsError``).
    A lock left behind by a crashed process is NOT broken automatically (fail-closed: an operator must
    inspect and remove it)."""
    import secrets

    lock = _lock_path_for(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    token = f"{os.getpid()}-{secrets.token_hex(8)}"
    try:
        fd = os.open(str(lock), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError as exc:
        raise ExportLockError(f"another export holds {out_dir} (lock {lock.name}); this invocation loses") from exc
    try:
        os.write(fd, json.dumps({"pid": os.getpid(), "token": token, "final": str(out_dir), "created_utc": C.utc_now()}).encode("utf-8"))
    finally:
        os.close(fd)
    return lock, token


def release_export_lock(lock: Path, token: str) -> bool:
    """Release only our own lock (token match); returns False (never raises) if the lock is not ours or
    cannot be removed — a leftover lock blocks future exports to that path until an operator removes it."""
    try:
        owner = json.loads(lock.read_text(encoding="utf-8")).get("token")
        if owner != token:
            return False
        os.unlink(lock)
        return True
    except (OSError, ValueError):
        return False


def _staging_dir_for(out_dir: Path) -> Path:
    """Exclusive staging directory, SIBLING of ``out_dir`` on the same filesystem (so the promotion is
    a same-volume rename); never visible under the final path."""
    import secrets

    out_dir.parent.mkdir(parents=True, exist_ok=True)
    for _ in range(50):
        cand = out_dir.parent / f".{out_dir.name}{STAGING_PREFIX}{os.getpid()}-{secrets.token_hex(4)}"
        try:
            cand.mkdir(exist_ok=False)
            return cand
        except FileExistsError:
            continue
    raise RefitError("could not create an exclusive staging directory")


RUNTIME_ONLY_STATUS_KEYS = ("promotion", "lock_released", "canonical_report_sha256", "final_path")


def export_student(source_module: Path, out_dir: Path, new_state: Mapping[str, Any], report: Mapping[str, Any], *, names35: list[str], extra: Mapping[str, Any] | None = None, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    """Transactional, fail-closed, mutually exclusive export of ``<out_dir>/rl_module_student``.

    **Provenance contract.**  The return value is the CANONICAL report: the dict parsed from the
    bytes of the completion marker ``<out_dir>/f2r_refit_report.json`` (written and verified in
    staging BEFORE the commit, immutable afterwards; ``json.load`` of the file equals the returned
    dict).  Facts that are only knowable after the commit — the promotion method actually used,
    whether it was single-step atomic, whether the lock was released — are NEVER written into the
    canonical report and never returned inside it: they are RUNTIME-ONLY status, delivered through
    the optional out-parameter ``runtime_status`` (a dict the caller passes and this function
    updates post-commit with ``RUNTIME_ONLY_STATUS_KEYS``; a plain dict update cannot raise).  The
    canonical report states the pre-commit policy (``export_transaction.promotion_policy``) and
    lists the runtime-only keys that are not persisted.

    Protocol (guarantees stated precisely):
    1. **Lock** — the sibling ``.<name>.lock`` is created exclusively (atomic ``O_EXCL`` on POSIX and
       Windows) BEFORE anything else; a concurrent cooperative exporter fails immediately with
       ``ExportLockError`` (a ``FileExistsError``) and creates nothing.  The lock is always released
       in ``finally`` (token-checked); a release failure after a successful promotion is reported in
       the result (``lock_released``) and never raised.
    2. Under the lock: the final path must be absent; everything is built and verified in an
       exclusive **staging** sibling on the same filesystem (module copy, ``module_state.pkl``,
       save/reload bit-exactness, structure, clock invariance, feature manifest, file table, report
       naming the FINAL path); the final path is re-checked absent **under the lock** right before
       the promotion.
    3. **Promotion** — native atomic no-replace rename (Darwin ``renameatx_np`` RENAME_EXCL, Linux
       ``renameat2`` RENAME_NOREPLACE, Windows ``MoveFileExW`` no-replace); where unavailable, the
       fail-closed fallback (exclusive ``mkdir`` + child moves, completion marker last).  A plain
       POSIX directory rename is never used, so an empty directory created by a NON-cooperative
       writer between the check and the promotion is never replaced (the native primitive or the
       exclusive mkdir fails instead).  Nothing that can raise runs after a successful promotion:
       the export is complete when the promotion returns.
    4. On any failure before/at the promotion: the staging directory is removed, the lock released,
       no final directory exists (fallback: our own empty final directory removed; foreign content
       never deleted).  Semantics for the legacy refits are unchanged (same files, same checks)."""
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W

    out_dir = Path(out_dir); out_module = out_dir / "rl_module_student"
    if out_dir.exists():
        raise FileExistsError(f"refusing to overwrite {out_dir}")
    src_state = W.load_module_state(Path(source_module))
    validate_init_state(src_state, expected_actor_digest=None)
    if tuple(new_state.keys()) != tuple(src_state.keys()):
        raise RefitError("exported keys must equal the init keys (10 pi* keys, same order)")
    for k in new_state:
        if np.asarray(new_state[k]).shape != np.asarray(src_state[k]).shape or np.asarray(new_state[k]).dtype != np.asarray(src_state[k]).dtype:
            raise RefitError(f"shape/dtype of {k} differs from the init")
    lock, token = acquire_export_lock(out_dir)
    lock_released: bool | None = None
    staging: Path | None = None
    promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError(f"final path exists (checked under the lock): {out_dir}")
        staging = _staging_dir_for(out_dir)
        stage_module = staging / "rl_module_student"
        shutil.copytree(Path(source_module), stage_module, copy_function=shutil.copy2)
        with (stage_module / "module_state.pkl").open("wb") as h:
            pickle.dump(dict(new_state), h, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states(dict(new_state), reloaded)
        if not cmp.get("exact"):
            raise RefitError(f"save/reload mismatch: {cmp}")
        struct = validate_init_state(reloaded, expected_actor_digest=None)
        if set(reloaded.keys()) != set(EXPECTED_KEY_ORDER):
            raise RefitError("exported state has extra/missing keys")
        inv = invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise RefitError("exported student is not invariant to the clock columns")
        manifest = {"schema_version": 1, "actor_feature_names": list(names35), "actor_feature_count": len(names35), "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "exploration_sigma": [R.SIGMA_CONSTANT] * R.ACTION_DIM, "contract": "deployable_markov_controller_state", "derived_from": C.rel(Path(source_module)), "aux_head_exported": False}
        mpath = stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
        C.write_json(mpath, manifest, clobber=mpath.exists())
        table = SV.module_files_table(stage_module)  # digests of the staged bytes == digests after the promotion
        full = {**dict(report), "schema_version": 1, "tool": "f2r_refit", "tool_sha256": C.sha256_file(Path(__file__).resolve()), "source_module": C.rel(Path(source_module)), "source_module_files_sha256": SV.module_files_table(Path(source_module)), "output_module": C.rel(out_module), "output_module_files_sha256": table, "export_structure": struct, "save_reload_exact": True, "clock_invariance": inv,
                "export_transaction": {"lock": lock.name, "staging": "exclusive sibling on the same filesystem, verified in full before promotion", "final_path": C.rel(out_dir), "completion_marker": COMPLETION_MARKER,
                                       "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL / Linux renameat2 RENAME_NOREPLACE / Windows MoveFileExW no-replace), else the fail-closed exclusive-mkdir fallback with the completion marker moved last; never a plain POSIX directory rename",
                                       "runtime_only_not_persisted": list(RUNTIME_ONLY_STATUS_KEYS), "canonical": "this file is the canonical report: immutable after the commit; the export API returns exactly its parsed content"},
                "generated_at_utc": C.utc_now(), "git": C.git_snapshot(), **dict(extra or {})}
        C.write_json(staging / COMPLETION_MARKER, full)
        canonical = json.loads((staging / COMPLETION_MARKER).read_text(encoding="utf-8"))  # the report as persisted
        if canonical != json.loads(json.dumps(full, default=str)):
            raise RefitError("canonical report on disk differs from the report built in memory")
        canonical_sha = C.sha256_file(staging / COMPLETION_MARKER)
        if not (stage_module / "module_state.pkl").is_file():
            raise RefitError("staging incomplete")
        if out_dir.exists():  # re-check under the lock, immediately before the promotion
            raise FileExistsError(f"final path appeared during the export: {out_dir}")
        promotion = promote_staging(staging, out_dir)
        promoted = True  # from here on nothing may raise: the export is complete and the canonical report immutable
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        lock_released = release_export_lock(lock, token)
    if runtime_status is not None:  # runtime-only, NOT canonical, NOT persisted (dict update: cannot raise)
        runtime_status.update({"promotion": promotion, "lock_released": bool(lock_released), "canonical_report_sha256": canonical_sha, "final_path": C.rel(out_dir)})
    return canonical


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="F2R student refit (S0: dry-run only; --fit is reserved for S1+ under authorisation).")
    parser.add_argument("--dataset-npz", required=True)
    parser.add_argument("--dataset-receipt", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--anchor-module", default=None, help="theta_{r-1} module dir (default: init JUL_H0)")
    parser.add_argument("--fit", action="store_true")
    args = parser.parse_args(argv)
    init, info = load_init_state()
    plan = {"mode": "fit" if args.fit else "dry_run", "init": info, "budget": R.REFIT_BUDGET, "anchor_module": args.anchor_module or C.rel(Path(R.INIT_PRIMARY["module"])), "dataset_npz": args.dataset_npz}
    if not args.fit:
        print(json.dumps(plan, indent=2)); return 0
    raise SystemExit("--fit is not authorised in S0 (no real refit); use the S1 driver after the architect's go")


if __name__ == "__main__":
    raise SystemExit(main())
