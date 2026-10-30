"""Minimal supervised refit of the 35D actor mean — candidate D (F1 stage 2).

Preregistered recipe (``f1_protocol.json`` -> ``refit``):

* initialisation: B's actor (hard-drop transplant, 10 ``pi*`` keys);
* trainable: the whole mean network (``pi_encoder`` layers + mean rows of the
  ``pi.1`` head); the log-std rows of ``pi.1`` are **not parameters**: they are
  fixed buffers (weights 0, bias ``ln(0.005)`` in float32), i.e. constant
  sigma 0.005 by construction, verified after save/reload;
* the two first-layer columns of ``gait_phase_sin/cos`` (zero in B) stay
  hard-zero (re-zeroed after every optimiser step, as July's ``adapt_actor``);
* loss = MSE(mean, teacher) + clip_weight * mean(relu(|mean| - 1)^2)
  + anchor_weight * mean over parameters of (p - p_init)^2  (anchor 0.0);
* Adam, fixed epochs/batch/lr/seed, mini-batches over the training rows
  only, fixed final epoch (no early stopping, no model selection on the
  validation rows or on closed-loop results); validation RMSE is only
  reported;
* no DAgger, no new data collection after the fit (flags recorded).

Torch only (no ray).  Deterministic: ``torch.manual_seed``, single thread,
``torch.use_deterministic_algorithms(True)``.  Output module directory is a
copy of the B module directory (architecture files) with the new
``module_state.pkl`` (10 keys, encoder aliases bit-identical), plus
``actor_feature_manifest.json`` and ``f1_refit_report.json`` (no-clobber).
Default is dry-run: validates inputs and prints the budget; ``--fit`` trains.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
for entry in (str(HERE),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402

CANONICAL_KEYS = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")
ALIAS_KEYS = {"pi_encoder.0.weight": "pi.0.0.weight", "pi_encoder.0.bias": "pi.0.0.bias", "pi_encoder.2.weight": "pi.0.2.weight", "pi_encoder.2.bias": "pi.0.2.bias"}
CLOCK_COLUMNS = (0, 1)  # gait_phase_sin, gait_phase_cos in the 35D manifest


class RefitError(RuntimeError):
    pass


def default_budget() -> dict[str, Any]:
    return dict(F1.load_protocol()["refit"]["budget"])


def validate_state_for_refit(state: dict[str, Any], *, width: int, names35: list[str]) -> dict[str, Any]:
    for k in CANONICAL_KEYS:
        if k not in state:
            raise RefitError(f"state missing {k}")
    for alias, canon in ALIAS_KEYS.items():
        if alias not in state or not np.array_equal(np.asarray(state[alias]), np.asarray(state[canon])):
            raise RefitError(f"alias {alias} missing or not bit-identical to {canon}")
    w1 = np.asarray(state["pi.0.0.weight"])
    if w1.shape[1] != width or np.asarray(state["pi.1.weight"]).shape[0] != 2 * F1.ACTION_DIM:
        raise RefitError("ABI mismatch for the 35D actor")
    if w1.dtype != np.float32:
        raise RefitError(f"expected float32 actor tensors, got {w1.dtype}")
    cols = [names35.index(n) for n in ("gait_phase_sin", "gait_phase_cos")]
    if tuple(cols) != CLOCK_COLUMNS:
        raise RefitError(f"clock columns at {cols}, expected {CLOCK_COLUMNS}")
    clock_zero = bool(np.all(w1[:, list(CLOCK_COLUMNS)] == 0.0))
    return {"clock_columns": list(CLOCK_COLUMNS), "clock_columns_zero_in_init": clock_zero, "width": int(width), "dtype": str(w1.dtype)}


def numpy_forward_mean(state: dict[str, Any], obs: np.ndarray) -> np.ndarray:
    arrays = DS.load_actor_arrays(state, expected_width=int(np.asarray(state["pi.0.0.weight"]).shape[1]))
    return DS.actor_logits_numpy(arrays, obs)[:, :F1.ACTION_DIM]


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)) ** 2)))


def fit_mean(state: dict[str, Any], data: dict[str, np.ndarray], *, budget: dict[str, Any], sigma: float = F1.SIGMA_CONSTANT, names35: list[str], progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """Train and return (new_state, report).  ``data`` must carry obs35,
    teacher_mean, split (0 train / 1 val)."""
    import torch
    import torch.nn.functional as F

    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(int(budget["seed"]))
    np.random.seed(int(budget["seed"]))
    info = validate_state_for_refit(state, width=F1.ENV_ACTOR_WIDTH, names35=names35)
    if not info["clock_columns_zero_in_init"]:
        raise RefitError("init actor must have zero clock columns (deployable contract)")
    split = np.asarray(data["split"]).reshape(-1)
    tr = np.flatnonzero(split == DS.SPLIT_TRAIN)
    va = np.flatnonzero(split == DS.SPLIT_VAL)
    if tr.size == 0 or va.size == 0:
        raise RefitError("need both train and validation rows")
    obs = torch.as_tensor(np.asarray(data["obs35"], dtype=np.float32))
    tgt = torch.as_tensor(np.asarray(data["teacher_mean"], dtype=np.float32))
    if obs.shape[1] != F1.ENV_ACTOR_WIDTH or tgt.shape[1] != F1.ACTION_DIM:
        raise RefitError("dataset shapes mismatch")
    f32 = np.float32
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.0.0.weight"], dtype=f32)))
    b1 = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.0.2.weight"], dtype=f32)))
    b2 = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.1.weight"][: F1.ACTION_DIM], dtype=f32)))
    b3m = torch.nn.Parameter(torch.as_tensor(np.array(state["pi.1.bias"][: F1.ACTION_DIM], dtype=f32)))
    params = [W1, b1, W2, b2, W3m, b3m]
    init = [p.detach().clone() for p in params]
    clock_mask = torch.ones_like(W1)
    clock_mask[:, list(CLOCK_COLUMNS)] = 0.0
    logstd_w = np.zeros((F1.ACTION_DIM, W2.shape[0]), dtype=f32)
    logstd_b = np.asarray(np.log(np.full(F1.ACTION_DIM, float(sigma))), dtype=f32)

    def mean_of(x: torch.Tensor) -> torch.Tensor:
        h1 = torch.tanh(x @ W1.T + b1)
        h2 = torch.tanh(h1 @ W2.T + b2)
        return h2 @ W3m.T + b3m

    opt = torch.optim.Adam(params, lr=float(budget["lr"]))
    rng = np.random.default_rng(int(budget["seed"]))
    epochs, batch = int(budget["epochs"]), int(budget["batch_size"])
    clip_w, anchor_w = float(budget["clip_weight"]), float(budget["anchor_weight"])
    history: list[dict[str, float]] = []

    def eval_rmse(idx: np.ndarray) -> float:
        with torch.no_grad():
            m = mean_of(obs[idx])
            return float(torch.sqrt(torch.mean((m - tgt[idx]) ** 2)).item())

    steps = 0
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(tr)
        losses = []
        for start in range(0, perm.size, batch):
            idx = perm[start: start + batch]
            opt.zero_grad(set_to_none=True)
            m = mean_of(obs[idx])
            mse = F.mse_loss(m, tgt[idx])
            clip = torch.relu(torch.abs(m) - 1.0).square().mean()
            loss = mse + clip_w * clip
            if anchor_w > 0.0:
                loss = loss + anchor_w * torch.stack([(p - p0).square().mean() for p, p0 in zip(params, init)]).mean()
            loss.backward()
            opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)  # hard-zero clock columns (deployable contract)
            losses.append(float(loss.item()))
            steps += 1
        history.append({"epoch": epoch, "train_loss": float(np.mean(losses)), "train_rmse": eval_rmse(tr), "val_rmse": eval_rmse(va)})
        if progress and (epoch % 25 == 0 or epoch == 1):
            print(json.dumps(history[-1]))
    new_state: dict[str, Any] = {}
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0)
    b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {
        "pi.0.0.weight": W1.detach().numpy().astype(f32).copy(),
        "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(),
        "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(),
        "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(),
        "pi.1.weight": w3.astype(f32).copy(),
        "pi.1.bias": b3.astype(f32).copy(),
    }
    for key in state:  # keep the original key order (OrderedDict from the module)
        if key in canon:
            new_state[key] = canon[key]
        elif key in ALIAS_KEYS:
            new_state[key] = canon[ALIAS_KEYS[key]].copy()
        else:
            raise RefitError(f"unexpected non-actor key in an inference-only module: {key}")
    ok, head = SV.constant_sigma_invariants(new_state, sigma=sigma)
    if not ok:
        raise RefitError(f"constant-sigma head invariants failed after fit: {head}")
    if not np.all(new_state["pi.0.0.weight"][:, list(CLOCK_COLUMNS)] == 0.0):
        raise RefitError("clock columns not zero after fit")
    mean_init = numpy_forward_mean(state, np.asarray(data["obs35"], dtype=np.float32))
    mean_fit = numpy_forward_mean(new_state, np.asarray(data["obs35"], dtype=np.float32))
    teacher = np.asarray(data["teacher_mean"], dtype=np.float64)
    report = {
        "budget": dict(budget),
        "optimizer_steps": int(steps),
        "epochs_run": int(epochs),
        "rows_train": int(tr.size),
        "rows_val": int(va.size),
        "selection": "fixed_final_epoch",
        "dagger_rounds": 0,
        "new_collection_after_fit": False,
        "history": history,
        "init_rmse_vs_teacher": {"train": rmse(mean_init[tr], teacher[tr]), "val": rmse(mean_init[va], teacher[va])},
        "fit_rmse_vs_teacher": {"train": rmse(mean_fit[tr], teacher[tr]), "val": rmse(mean_fit[va], teacher[va])},
        "fit_rmse_vs_teacher_per_action_val": [rmse(mean_fit[va][:, j], teacher[va][:, j]) for j in range(F1.ACTION_DIM)],
        "init_rmse_vs_teacher_per_action_val": [rmse(mean_init[va][:, j], teacher[va][:, j]) for j in range(F1.ACTION_DIM)],
        "mean_abs_max_on_dataset": float(np.max(np.abs(mean_fit))),
        "logstd_head": head,
        "clock_columns_zero": True,
        "init_info": info,
    }
    return new_state, report


def save_refit_module(source_module: Path, out_dir: Path, new_state: dict[str, Any], report: dict[str, Any], *, names35: list[str], dataset_receipt: dict[str, Any] | None = None) -> dict[str, Any]:
    import warm_start as W  # production, import only

    out_dir = Path(out_dir)
    out_module = out_dir / "rl_module_refit"
    if out_dir.exists():
        raise FileExistsError(f"refusing to overwrite {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=False)
    shutil.copytree(Path(source_module), out_module, copy_function=shutil.copy2)
    with (out_module / "module_state.pkl").open("wb") as handle:
        pickle.dump(new_state, handle, protocol=pickle.HIGHEST_PROTOCOL)
    reloaded = W.load_module_state(out_module)
    cmp = W.compare_actor_states(new_state, reloaded)
    if not cmp["exact"]:
        raise RefitError(f"save/reload mismatch: {cmp}")
    ok, head = SV.constant_sigma_invariants(reloaded, sigma=F1.SIGMA_CONSTANT)
    if not ok:
        raise RefitError("constant-sigma invariants failed after reload")
    digest = W.actor_state_digest(reloaded)
    manifest = {
        "schema_version": 1,
        "actor_feature_names": list(names35),
        "actor_feature_count": len(names35),
        "actor_digest": digest,
        "module_state_sha256": C.sha256_file(out_module / "module_state.pkl"),
        "exploration_sigma": [F1.SIGMA_CONSTANT] * F1.ACTION_DIM,
        "exploration_log_std": head["logstd_bias"],
        "contract": "deployable_markov_controller_state",
        "derived_from": C.rel(Path(source_module)),
        "refit": "F1 candidate D (minimal supervised refit, see f1_refit_report.json)",
    }
    manifest_path = out_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    C.write_json(manifest_path, manifest, clobber=manifest_path.exists())
    full_report = {
        **report,
        "schema_version": 1,
        "tool": "f1_refit",
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "source_module": C.rel(Path(source_module)),
        "source_module_files_sha256": SV.module_files_table(Path(source_module)),
        "output_module": C.rel(out_module),
        "output_module_files_sha256": SV.module_files_table(out_module),
        "actor_digest": digest,
        "save_reload_exact": True,
        "dataset_receipt": dataset_receipt,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
    }
    C.write_json(out_dir / "f1_refit_report.json", full_report)
    return full_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Candidate D minimal supervised refit (stage 2).")
    parser.add_argument("--source-module", required=True, help="B rl_module directory (init)")
    parser.add_argument("--dataset-npz", required=True)
    parser.add_argument("--dataset-receipt", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--fit", action="store_true", help="run the fit (default: dry-run)")
    parser.add_argument("--progress", action="store_true")
    args = parser.parse_args(argv)
    if str(F1.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(F1.BASELINE_DIR))
    import warm_start as W

    budget = default_budget()
    state = W.load_module_state(Path(args.source_module))
    names35, _ = __import__("f1_obs_adapter").read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    info = validate_state_for_refit(state, width=F1.ENV_ACTOR_WIDTH, names35=names35)
    ds = DS.load_dataset(Path(args.dataset_npz), Path(args.dataset_receipt))
    plan = {"mode": "fit" if args.fit else "dry_run", "budget": budget, "source_actor_digest": W.actor_state_digest(state), "init_info": info, "rows": int(ds["arrays"]["obs35"].shape[0]), "rows_train": int(np.sum(ds["arrays"]["split"] == DS.SPLIT_TRAIN)), "rows_val": int(np.sum(ds["arrays"]["split"] == DS.SPLIT_VAL))}
    if not args.fit:
        print(json.dumps(plan, indent=2))
        return 0
    new_state, report = fit_mean(state, ds["arrays"], budget=budget, names35=names35, progress=args.progress)
    full = save_refit_module(Path(args.source_module), Path(args.out_dir), new_state, report, names35=names35, dataset_receipt={"path": C.rel(Path(args.dataset_receipt)), "sha256": C.sha256_file(Path(args.dataset_receipt)), "npz_sha256": ds["receipt"].get("npz_sha256")})
    print(json.dumps({k: full[k] for k in ("output_module", "actor_digest", "init_rmse_vs_teacher", "fit_rmse_vs_teacher", "optimizer_steps")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
