"""V26B rev3i — corrective R2 stage: EXACT historical anchor + deterministic global-constant loss.

Differences vs the AUDIT-REJECTED rev3g run (everything else identical, shared code
imported from v26b_r2g READ-ONLY):
* anchor = 0.01 * mean of the SIX July tensor terms with explicit 0.5 factors on the
  head mean-only terms (== zero-padded full [4,256]/[4] deltas; rev3h supplement);
* data loss = K * sum_B(lambda_i e_i), K = 42 CONSTANT (per-row weight K*lambda_i,
  composition-independent; epoch-sum == K * global objective exactly);
* Adam declaration: the optimiser trajectory is NOT equivalent to full-batch and no
  such equivalence is claimed.
Init = R1 (c7bcee1c...), NEVER R2G.  No rollout/PPO/critic/sigma sweep.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r2g as G  # noqa: E402  (shared split/gates/scaling/transforms, read-only)
import v26b_r2_offline as R2  # noqa: E402
import v26b_dagger_r1 as D1  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402


class R2IError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-R2I"
AMENDMENT_REV3I = HERE / "v26b_amendment_rev3i_r2_corrected.json"
PIN_AMENDMENT_REV3I = "7cde9329afd90aadd30206e8c060ef341f34b67049867cf3f441881ec78b73d6"
PIN_REV3H_MEMO = "e2f31d1eab306625456195d9e71348457d66e7d527060e012c9a06a58dc85081"
PIN_REV3H_SUPPLEMENT = "b7fe2b9a3de943892e4855eee25b6d8949f37c7083ca99667a0d9fc72e018a00"
PIN_R2G_RECEIPT_REJECTED = "5374f1af7529272827b8327b1a00483d4ffe9e81f24374c920d370373251bc3c"
R2G_REJECTED_DIGEST = "65276df60b20b0180cfea6695bbbf4fb82e685b0dd0ea52578297e0f147c891a"
K_CONST = 42  # = ceil(10727/256), declared constant of the a' loss
ANCHOR_WEIGHT = 0.01
OUT_R2I = VA.OUT_ROOT / "student" / "V2_R2I"
RECEIPT_NAME = "v26b_r2i_receipt.json"
ADAM_DECLARATION = ("each Adam step minimises l(B) = K*sum_B(lambda*e) + clip_mean(B) + 0.01*anchor; "
                    "the per-row weight is the CONSTANT K*lambda_i and the epoch-sum of data terms equals "
                    "K*L_global exactly, but the Adam TRAJECTORY is NOT equivalent to a full-batch "
                    "optimisation and no such equivalence is claimed")
REJECTED_NOTE = "V2_R2G (digest 65276df6...) is AUDIT-REJECTED: preserved on disk, never an init, never rolled out"


def anchor_loss_july_exact(mean_params: Sequence[Any], anchors: Sequence[Any]):
    """0.01-weighted EXACT historical anchor term (weight applied by the caller):
    mean of the 6 July tensor terms; the two head mean-only terms carry the explicit 0.5
    (logstd rows sit at the anchor at every loss evaluation -> zero-padded full deltas)."""
    import torch
    W1, b1, W2, b2, W3m, b3m = mean_params
    A1, a1, A2, a2, A3m, a3m = anchors
    terms = [
        (W1 - A1).square().mean(), (b1 - a1).square().mean(),
        (W2 - A2).square().mean(), (b2 - a2).square().mean(),
        0.5 * (W3m - A3m).square().mean(), 0.5 * (b3m - a3m).square().mean(),
    ]
    return torch.stack(terms).mean()


def verify_lineage_r2i() -> dict[str, Any]:
    lin = G.verify_lineage_r2g()
    for path, pin, key in ((AMENDMENT_REV3I, PIN_AMENDMENT_REV3I, "amendment_rev3i"),
                           (C.REPO / "reports/user/2026-08-24_v26b_rev3h_audit_memo.md", PIN_REV3H_MEMO, "rev3h_memo"),
                           (C.REPO / "reports/user/2026-08-24_v26b_rev3h_supplemento_anchor_head.md", PIN_REV3H_SUPPLEMENT, "rev3h_supplement")):
        p = Path(path)
        got = C.sha256_file(p)
        if got != pin:
            raise R2IError(f"lineage violated: {p.name} sha {got} != pinned {pin}")
        lin[key] = {"path": C.rel(p), "sha256": got}
    rej = G.OUT_R2G / G.RECEIPT_NAME
    got = C.sha256_file(rej)
    if got != PIN_R2G_RECEIPT_REJECTED:
        raise R2IError("R2G rejected-evidence receipt changed on disk")
    lin["r2g_rejected_preserved"] = {"receipt_sha256": got, "status": REJECTED_NOTE}
    # amendment pins this stage's shared tooling; verify not edited since
    am = json.loads(AMENDMENT_REV3I.read_text(encoding="utf-8"))
    for label, pin in am["parents_immutable"]["preexisting_tooling"].items():
        fname = label.split(" ")[0]
        disk = C.sha256_file(HERE / fname)
        if disk != pin:
            raise R2IError(f"{fname} changed after the rev3i amendment")
    return lin


def provenance_block_r2i() -> dict[str, Any]:
    block = {**G.provenance_block(), "rev3h_memo": PIN_REV3H_MEMO, "rev3h_supplement": PIN_REV3H_SUPPLEMENT, "rev3i": PIN_AMENDMENT_REV3I}
    for k, v in block.items():
        if not (isinstance(v, str) and len(v) == 64):
            raise R2IError(f"provenance block: {k} invalid")
    return block


def fit_r2i(init_scaled: Mapping[str, Any], data: Mapping[str, np.ndarray], split: Mapping[str, np.ndarray], vec: np.ndarray, *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    budget = dict(budget or G.BUDGET)
    train = split["train"]
    roles_all = np.asarray(data["role"])
    obs_scaled = (np.asarray(data["obs35"], dtype=np.float64) / vec[None, :]).astype(np.float32)
    f32 = np.float32
    obs_t = torch.as_tensor(obs_scaled[train])
    tgt = torch.as_tensor(np.asarray(data["actions"], dtype=f32)[train])
    roles = roles_all[train]
    n_train = {r: int(np.sum(roles == r)) for r in G.MASSES}
    if any(v == 0 for v in n_train.values()):
        raise R2IError(f"empty role in train: {n_train}")
    n = int(obs_t.shape[0])
    if K_CONST != int(np.ceil(n / int(budget["batch_size"]))):
        raise R2IError(f"K constant {K_CONST} != ceil({n}/{budget['batch_size']})")
    lam = np.empty(n, dtype=np.float64)
    for r, w in G.MASSES.items():
        lam[roles == r] = w / n_train[r]
    lam_t = torch.as_tensor(lam.astype(f32))
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    W1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.weight"], dtype=f32))); b1 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.0.bias"], dtype=f32)))
    W2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.weight"], dtype=f32))); b2 = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.0.2.bias"], dtype=f32)))
    W3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.weight"][: R.ACTION_DIM], dtype=f32))); b3m = torch.nn.Parameter(torch.as_tensor(np.array(init_scaled["pi.1.bias"][: R.ACTION_DIM], dtype=f32)))
    mean_params = [W1, b1, W2, b2, W3m, b3m]
    anchor_t = [p.detach().clone() for p in mean_params]  # theta_R1 in the scaled space
    clock_mask = torch.ones_like(W1); clock_mask[:, list(R.CLOCK_COLUMNS)] = 0.0
    with torch.no_grad():
        W1.mul_(clock_mask)
    hidden = int(W2.shape[0])
    logstd_w = np.zeros((R.ACTION_DIM, hidden), dtype=f32); logstd_b = np.asarray(np.log(np.full(R.ACTION_DIM, float(R.SIGMA_CONSTANT))), dtype=f32)
    encoder = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2)  # noqa: E731
    opt = torch.optim.Adam(mean_params, lr=float(budget["lr"]))
    rng = np.random.default_rng(int(budget["seed"]))
    epochs = int(budget["epochs"]); batch = int(budget["batch_size"]); lam_clip = float(budget["clip_weight"])
    history = []
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []; dsum = 0.0
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            m = encoder(obs_t[idx]) @ W3m.T + b3m
            e = ((m - tgt[idx]) ** 2).mean(dim=1)
            l_data = float(K_CONST) * (lam_t[idx] * e).sum()       # a': constant K, per-row weight K*lambda_i
            anchor = anchor_loss_july_exact(mean_params, anchor_t)  # exact historical semantics
            loss = l_data + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + ANCHOR_WEIGHT * anchor
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item())); dsum += float(l_data.item())
        history.append({"epoch": epoch, "loss": float(np.mean(losses)), "epoch_sum_data_over_K": dsum / K_CONST})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    scaled_fit = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    report = {"tool": "v26b_r2i.fit_r2i (rev3i: exact historical anchor 0.01*mean of 6 July terms with 0.5 head factors; data loss K*sum(lambda*e), K=42 constant)", "rows_train": n, "budget": budget, "K": K_CONST, "anchor_weight": ANCHOR_WEIGHT, "masses": G.MASSES, "n_train_by_role": n_train, "lambda_per_row_by_role": {r: G.MASSES[r] / n_train[r] for r in G.MASSES}, "adam_declaration": ADAM_DECLARATION, "history": history, "sigma_note": G.SIGMA_NOTE}
    return scaled_fit, report


def run_r2i(*, authorized_stage: str | None, out_dir: Path = OUT_R2I, progress: bool = True, runtime_status=None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise R2IError(f"R2I requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_r2i()
    provenance = provenance_block_r2i()  # built BEFORE any fit
    vec, scale_table = G.scale_vector()
    data, ds_report = R2.build_r2_dataset()
    split = G.preregistered_split(data)
    prefit = G.prefit_gates(data, split, ds_report)
    if not prefit["pass_prefit"]:
        diag = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2i_prefit_stop_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(diag, json.dumps({"schema": "v26b_r2i_prefit_stop.1", "prefit": prefit}, indent=2, default=str) + "\n")
        raise R2IError(f"prefit binding gate FAILED: STOP; diagnostics {C.rel(diag)}")
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(D1.OUT_R1 / "rl_module").items()}
    RF.validate_init_state(init_raw, expected_actor_digest=R1.PIN_R1_ACTOR_DIGEST)
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    t1 = G.preservation_test(init_raw, init_scaled, np.asarray(data["obs35"]), vec)
    if t1 > G.PRESERVATION_TOL:
        raise R2IError(f"T1 FAILED: {t1:.3e}")
    scaled_fit, fit_report = fit_r2i(init_scaled, data, split, vec, progress=progress)
    export_raw = G.export_state_from_scaled(scaled_fit, vec)
    post = G.postfit_gates(init_raw, export_raw, scaled_fit, data, split, vec, t1)
    if not post["pass_postfit"]:
        rej = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_r2i_REJECTED_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
        VA._atomic_fill_reserved(rej, json.dumps({"schema": "v26b_r2i_rejected.1", "REJECTED": True, "postfit": post, "prefit": prefit}, indent=2, default=str) + "\n")
        raise R2IError(f"postfit gate FAILED: rejected artefact {C.rel(rej)}; NO export")
    r1_pre = VS.source_files_table(D1.OUT_R1 / "rl_module")
    out_dir = Path(out_dir)
    if out_dir.exists():
        raise FileExistsError(f"no-clobber: {out_dir} exists")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    lock, token = RF.acquire_export_lock(out_dir)
    staging = None; promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError("final path exists (under lock)")
        staging = RF._staging_dir_for(out_dir)
        stage_module = staging / "rl_module"; stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(D1.OUT_R1 / "rl_module" / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(D1.OUT_R1 / "rl_module" / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in export_raw.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in export_raw.items()}, reloaded)
        if not cmp.get("exact"):
            raise R2IError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=post["actor_digest_new"])
        smoke = RF.numpy_mean(reloaded, np.asarray(data["obs35"], dtype=np.float32)[:16])
        if not np.all(np.isfinite(smoke)):
            raise R2IError("export-space smoke test non-finite")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": 35, "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": G.SIGMA_NOTE, "derived_from": C.rel(D1.OUT_R1 / "rl_module"), "source_actor_digest": R1.PIN_R1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state", "note_scaling": "July physical scaling absorbed at export (raw physical observations)"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(D1.OUT_R1 / "rl_module") != r1_pre:
            raise R2IError("SOURCE R1 changed during export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_r2i.1",
            "amendments": provenance,
            "audit_rejected_predecessor": {"artifact": "student/V2_R2G", "actor_digest": R2G_REJECTED_DIGEST, "receipt_sha256": PIN_R2G_RECEIPT_REJECTED, "status": REJECTED_NOTE},
            "authorized_stage": AUTHORIZED_STAGE, "lineage": lineage,
            "init": {"module": C.rel(D1.OUT_R1 / "rl_module"), "actor_digest": R1.PIN_R1_ACTOR_DIGEST, "files_sha256": r1_pre, "note": "init = R1, NOT R2G"},
            "scaling": {"table": scale_table, "T1_maxabs": post["function_preservation"]["T1_prefit_maxabs"], "T2_maxabs": post["function_preservation"]["T2_export_maxabs"], "tol": G.PRESERVATION_TOL},
            "dataset": {"report": dict(ds_report), "split": {"train": int(split["train"].sum()), "hold": int(split["hold"].sum()), "embargo": int(split["embargo"].sum()), "held_trajectories": split["held_trajectories"]}},
            "prefit_gates": prefit,
            "fit": {k: fit_report[k] for k in ("tool", "rows_train", "budget", "K", "anchor_weight", "masses", "n_train_by_role", "lambda_per_row_by_role", "adam_declaration", "sigma_note")},
            "anchor_semantics": "0.01 * mean([MSE_W1, MSE_b1, MSE_W2, MSE_b2, 0.5*MSE_W3m, 0.5*MSE_b3m]) towards theta_R1_scaled (exact July semantics per rev3h supplement; value+gradient tested vs full zero-padded reference)",
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1]},
            "loss_history_full": hist,
            "postfit_gates": post,
            "structure": struct, "save_reload_exact": True,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": G.SIGMA_NOTE},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_r2i.py": C.sha256_file(Path(__file__).resolve()), "test_v26b_r2i.py": C.sha256_file(HERE / "test_v26b_r2i.py") if (HERE / "test_v26b_r2i.py").is_file() else None, "v26b_r2g.py": C.sha256_file(HERE / "v26b_r2g.py"), "v26b_r2_offline.py": C.sha256_file(HERE / "v26b_r2_offline.py")},
            "scope": "R2I offline only: no rollout, no PPO, no critic, no sigma sweep, no production change; STOP for the architect audit",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise R2IError("canonical receipt differs from memory")
        canonical_sha = C.sha256_file(staging / RECEIPT_NAME)
        if out_dir.exists():
            raise FileExistsError("final path appeared during export")
        promotion = RF.promote_staging(staging, out_dir)
        promoted = True
    except BaseException:
        if staging is not None and not promoted:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        released = RF.release_export_lock(lock, token)
    if runtime_status is not None:
        runtime_status.update({"promotion": promotion, "lock_released": bool(released), "canonical_receipt_sha256": canonical_sha, "final_path": C.rel(out_dir)})
    return canonical


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3i corrected R2 (dry by default; NO rollout)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_r2i(); provenance_block_r2i()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "K": K_CONST, "anchor": "0.01 * mean of 6 July terms (0.5 head factors)"}, indent=2))
        return 0
    runtime = {}
    canonical = run_r2i(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "new_actor_digest": canonical["postfit_gates"]["actor_digest_new"], "postfit_pass": canonical["postfit_gates"]["pass_postfit"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
