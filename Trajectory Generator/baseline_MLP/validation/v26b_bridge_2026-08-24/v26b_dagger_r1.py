"""V26B rev3d — DAgger ROUND 1 (July-faithful): dataset build + offline refit + gate + export.

Per amendment ``rev3d_dagger_round1_july_faithful``: the FULL 493-row early-ended
prefix of the R0a nominal det rollout is aggregated (no truncation — July-11
semantics for deterministic rollouts) with the 500 BC rows, ALL labelled u_IK
same-step on the FULL-pinned nominal cache grid; bitwise dedup fail-closed
(label conflicts are hard errors); the FULL mean network is refit from R0a
EXACTLY with the chain-frozen budget (deviations from the July recipe
preregistered in rev3d); log-std stays the frozen constant placeholder — THE
SIGMA VALUE 0.005 REMAINS UNDECIDED AND IS NOT USED HERE.  Offline gate:
RMSE vs u_IK <= 0.15 per joint on EACH role and on the aggregate + Q3.
NO rollout of the new actor in this stage.
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
VALIDATION_DIR = HERE.parent
for _entry in (str(VALIDATION_DIR / "f0_freeze_2026-08-22"), str(VALIDATION_DIR / "f1_ablation_2026-08-23"), str(VALIDATION_DIR / "f2r_bridge_2026-08-23"), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_r0a as A  # noqa: E402
import v26b_r0a_rollout as RO  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class DaggerError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-DAGGER-R1"
AMENDMENT_REV3D = HERE / "v26b_amendment_rev3d_dagger_round1.json"
PIN_AMENDMENT_REV3D = "7b995277103bb5cf01ed8cbf46098425747a371e573b35ec44627b37907a5adf"
PIN_ROLLOUT_TRACE = "8e3aff5465f53f9d9ba5178fe709ffc4329e373f2dc1332d15d43a731ae454ce"
PIN_ROLLOUT_RECEIPT = "ae04bcda13e5cc45fc9199f644920c399db1de4fd542adba19e614e5e0933a14"
PIN_ROLLOUT_ADDENDUM = "0884a25be8c919e89992d75cee81bdb4a5996f57a5194b19046dae32ac7da41b"
RMSE_MAX_PER_JOINT = 0.15  # unchanged rev3b threshold, per role and aggregate (rev3d)
HOLDOUT_FRACTION = 0.2     # diagnostic only (leakage-free: rows bitwise-unique after dedup)
ALLOWED_PURPOSES = {"ik_nominal_det", "ik_onpolicy_det"}
OUT_R1 = VA.OUT_ROOT / "student" / "V2_DAGGER_R1"
RECEIPT_NAME = "v26b_dagger_r1_receipt.json"
SIGMA_NOTE = "the sigma value 0.005 remains UNDECIDED and is NOT used here: it is only the frozen serialisation placeholder of the log-std head (rows zero, bias float32(ln 0.005)); no gate reads it; the operational sigma is decided exclusively by the preregistered V4 sweep"


def verify_lineage_r1() -> dict[str, Any]:
    lin = RO.verify_lineage_rollout()  # chain rev3..rev3c + R0a receipt/module + config + caches
    for path, pin, key in ((AMENDMENT_REV3D, PIN_AMENDMENT_REV3D, "amendment_rev3d"),
                           (RO.JOB_DIR / "rollout_policy_trace.json", PIN_ROLLOUT_TRACE, "rollout_trace"),
                           (RO.JOB_DIR / RO.RECEIPT_NAME, PIN_ROLLOUT_RECEIPT, "rollout_receipt"),
                           (RO.JOB_DIR / "v26b_r0a_rollout_receipt_audit_addendum.json", PIN_ROLLOUT_ADDENDUM, "rollout_addendum")):
        p = Path(path)
        if p.is_symlink() or not p.is_file():
            raise DaggerError(f"lineage file missing (or symlink): {p}")
        got = C.sha256_file(p)
        if got != pin:
            raise DaggerError(f"lineage violated: {p.name} sha {got} != pinned {pin}")
        lin[key] = {"path": C.rel(p), "sha256": got}
    return lin


# --- dataset --------------------------------------------------------------------------------------

def build_r1_dataset() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """500 BC rows + 493 on-policy rows, ALL labelled u_IK same-step; bitwise dedup fail-closed."""
    bc_role, bc_ds = A.build_r0a_dataset()  # re-validates anchors + FULL cache pins; 500 rows u_IK
    ik = A.verify_ik_caches_full_productive()
    cc = ik["caches"]["nominal"]
    traj = DS.trajectory_from_job(RO.JOB_DIR, expected_width=R.ENV_ACTOR_WIDTH)
    if traj["trace_sha256"] != PIN_ROLLOUT_TRACE:
        raise DaggerError("R0a rollout trace digest != pinned")
    if int(traj["seed"]) != R.DET_SEED or traj["stochastic"] or str(traj["action_selection"]) != "deterministic":
        raise DaggerError("on-policy trace is not the deterministic seed-123 R0a rollout")
    obs_op = np.ascontiguousarray(np.asarray(traj["obs35"], dtype=np.float64).astype(np.float32))
    t_op = np.asarray(traj["t_pre"], dtype=np.float64)
    if obs_op.shape[0] != 493:
        raise DaggerError(f"on-policy rows {obs_op.shape[0]} != 493 (pinned early-ended prefix)")
    idx = cc.lookup(t_op)  # exact grid; fails closed if any t_pre off the nominal grid
    u_op = np.ascontiguousarray(cc.ik_action[idx])
    clock_op = np.ascontiguousarray(cc.clock[idx].astype(np.float32))
    def role_dict(obs, t, u, clk, purpose, job_id):
        n = obs.shape[0]
        return {"obs35": obs, "t_pre": t, "actions": u.astype(np.float32), "clock": clk,
                "seed": np.full(n, R.DET_SEED, dtype=np.int64), "purpose": np.asarray([purpose] * n, dtype=str),
                "job_id": np.asarray([job_id] * n, dtype=str)}
    onpol = role_dict(obs_op, t_op, u_op, clock_op, "ik_onpolicy_det", "R0A_35D__v3_canonical__nominal__det")
    bc = dict(bc_role); bc["purpose"] = np.asarray(["ik_nominal_det"] * 500, dtype=str)
    # union + bitwise dedup (fail-closed on label conflicts)
    union = {k: np.concatenate([np.asarray(bc[k]), np.asarray(onpol[k])]) for k in ("obs35", "t_pre", "actions", "clock", "seed", "purpose", "job_id")}
    for p in np.unique(union["purpose"]):
        if str(p) not in ALLOWED_PURPOSES:
            raise DaggerError(f"forbidden purpose {p!r} (raw V26 / alt-start / JUL_H0 / T1 / T1R / B0820 never enter)")
    seen: dict[bytes, int] = {}
    keep: list[int] = []
    collisions: list[dict[str, Any]] = []
    for i in range(union["obs35"].shape[0]):
        k = union["obs35"][i].tobytes()
        j = seen.get(k)
        if j is None:
            seen[k] = i; keep.append(i)
        else:
            if union["actions"][i].tobytes() != union["actions"][j].tobytes():
                raise DaggerError(f"label CONFLICT on bitwise-identical observation (rows {j}/{i}, purposes {union['purpose'][j]}/{union['purpose'][i]}): fail-closed")
            collisions.append({"kept_row": int(j), "dropped_row": int(i), "kept_purpose": str(union["purpose"][j]), "dropped_purpose": str(union["purpose"][i]), "labels_identical": True, "input_sha256": DS.sha256_array(np.ascontiguousarray(union["obs35"][i]))[:16]})
    sel = np.asarray(keep, dtype=np.int64)
    data = {k: np.ascontiguousarray(v[sel]) for k, v in union.items()}
    report = {
        "roles": {"ik_nominal_det": 500, "ik_onpolicy_det": 493},
        "rows_after_dedup": int(sel.size),
        "collisions": {"count": len(collisions), "detail": collisions, "rule": "identical obs + identical label collapsed (reported); different label = hard error"},
        "labels": {"rule": "u_IK same-step on the FULL-pinned nominal cache grid (July semantics: teacher action at the same fixed-start step)", "cache_digest": ik["digests"]["nominal"]},
        "on_policy": {"trace_sha256": PIN_ROLLOUT_TRACE, "rows": 493, "includes_last_row": True, "note": "the joint_divergence guard fired AFTER the step-493 action; its pre-action observation is a legitimately visited state (rev3d)"},
        "no_truncation": "July-11 semantics for deterministic rollouts (rev3d supersession of the rev3c 500/500 clause); NO V26-anchor reference used",
        "forbidden_absent": "raw V26 labels, alt-start, JUL_H0, T1, T1R, B0820",
        "bc_dataset_report": bc_ds["report"],
    }
    return data, report


# --- refit (chain-frozen budget; single u_IK objective over the aggregate) ------------------------

def fit_dagger_r1(init_state: Mapping[str, Any], data: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """Same numeric loop as the productive R0a fit (bit-proven lineage), single u_IK objective on
    the deduped aggregate; contract accepts both round-1 purposes."""
    import torch
    import torch.nn.functional as F

    budget = dict(budget or V2.BUDGET_R0)
    V2._role_contract(data, allowed_purposes=ALLOWED_PURPOSES, what="dagger_r1_aggregate")
    info = RF.validate_init_state(init_state, expected_actor_digest=None)
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.asarray(data["obs35"], dtype=f32))
    tgt = torch.as_tensor(np.asarray(data["actions"], dtype=f32))
    clk = torch.as_tensor(np.asarray(data["clock"], dtype=f32))
    var_role = np.var(np.asarray(data["actions"], dtype=np.float64), axis=0)
    if np.any(var_role <= 0.0):
        raise DaggerError("degenerate per-joint variance")
    vr = torch.as_tensor(var_role.astype(f32))
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
    opt = torch.optim.Adam(mean_params + [Wh, bh], lr=float(budget["lr"])); rng = np.random.default_rng(int(budget["seed"]))
    n = int(obs.shape[0]); epochs = int(budget["epochs"]); batch = int(budget["batch_size"])
    lam_clip, lam_phi, lam_a = float(budget["clip_weight"]), float(budget["lambda_phi"]), float(budget["lambda_anchor"])
    history = []
    for epoch in range(1, epochs + 1):
        perm = rng.permutation(n); losses = []
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch]); opt.zero_grad(set_to_none=True)
            h2 = encoder(obs[idx]); m = h2 @ W3m.T + b3m; hp = h2 @ Wh.T + bh
            l_role = (((m - tgt[idx]) ** 2) / vr).mean(dim=0).mean()
            loss = l_role + lam_clip * torch.relu(torch.abs(m) - 1.0).square().mean() + lam_phi * F.mse_loss(hp, clk[idx])
            if lam_a > 0.0:
                loss = loss + lam_a * torch.stack([(p - a0).square().sum() for p, a0 in zip(mean_params, anchor_t)]).sum()
            loss.backward(); opt.step()
            with torch.no_grad():
                W1.mul_(clock_mask)
            losses.append(float(loss.item()))
        history.append({"epoch": epoch, "loss": float(np.mean(losses))})
        if progress and (epoch == 1 or epoch % 50 == 0):
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state: dict[str, Any] = {}
    for key in RF.EXPECTED_KEY_ORDER:
        new_state[key] = canon[key].copy() if key in canon else canon[RF.ALIAS_KEYS[key]].copy()
    struct = RF.validate_init_state(new_state, expected_actor_digest=None)
    report = {"tool": "v26b_dagger_r1.fit_dagger_r1 (July-faithful round 1; chain-frozen budget; deviations preregistered in rev3d)", "rows": n, "budget": budget, "var_per_joint": var_role.tolist(), "init_actor_digest": info["actor_digest"], "new_actor_digest": RF.actor_state_digest(new_state), "history": history, "structure": struct, "sigma_note": SIGMA_NOTE, "aux_head": {"exported": False, "training_time_only": True}}
    return new_state, report


# --- gate -----------------------------------------------------------------------------------------

def evaluate_r1_gate(init_state: Mapping[str, Any], state: Mapping[str, Any], data: Mapping[str, np.ndarray]) -> dict[str, Any]:
    f32 = np.float32
    m = RF.numpy_mean(state, np.asarray(data["obs35"], dtype=f32))
    y = np.asarray(data["actions"], dtype=np.float64)
    purpose = np.asarray(data["purpose"]).reshape(-1)
    def rmse(mask):
        return np.sqrt(np.mean((m[mask] - y[mask]) ** 2, axis=0)).tolist()
    masks = {"aggregate": np.ones(y.shape[0], bool), "bc_nominal": purpose == "ik_nominal_det", "on_policy": purpose == "ik_onpolicy_det"}
    per = {k: {"rows": int(v.sum()), "rmse_knee_ankle": rmse(v), "pass": bool(all(x <= RMSE_MAX_PER_JOINT for x in rmse(v)))} for k, v in masks.items()}
    rng = np.random.default_rng(2026)
    perm = rng.permutation(y.shape[0]); n_hold = int(round(y.shape[0] * HOLDOUT_FRACTION))
    hold = np.zeros(y.shape[0], bool); hold[perm[:n_hold]] = True
    diag_hold = {"rows": int(n_hold), "rmse_knee_ankle": rmse(hold), "split": "deterministic default_rng(2026) permutation, 20%; leakage-free: rows bitwise-unique after dedup (unlike July's tiled split); DIAGNOSTIC only, never a gate", "holdout_indices_sha256": DS.sha256_array(np.sort(perm[:n_hold]).astype(np.int64))}
    struct = RF.validate_init_state(state, expected_actor_digest=None)
    inv = RF.invariance_test(state, np.asarray(data["obs35"], dtype=f32)[:64])
    q3 = {"ten_keys": tuple(state.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant_placeholder": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "save_reload_exact": "verified at export"}
    init_m = RF.numpy_mean(init_state, np.asarray(data["obs35"], dtype=f32))
    out = {
        "threshold_per_joint": RMSE_MAX_PER_JOINT,
        "per_role": per,
        "holdout_diagnostic": diag_hold,
        "improvement_vs_R0a_init": {k: {"init_rmse": np.sqrt(np.mean((init_m[v] - y[v]) ** 2, axis=0)).tolist()} for k, v in masks.items()},
        "Q3": {**q3, "pass": bool(q3["ten_keys"] and q3["clock_columns_zero"] and q3["logstd_constant_placeholder"] and q3["clock_invariance_bit_identical"])},
        "parameter_shift_sq_informational": float(sum(np.sum((np.asarray(state[k], np.float64) - np.asarray(init_state[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
        "primary_requirement": "correct the on-policy states WITHOUT losing the nominal BC (both role gates mandatory)",
        "sigma_note": SIGMA_NOTE,
    }
    out["pass_r1"] = bool(per["aggregate"]["pass"] and per["bc_nominal"]["pass"] and per["on_policy"]["pass"] and out["Q3"]["pass"])
    return out


def assert_r1_gate(gate: Mapping[str, Any]) -> None:
    if gate.get("pass_r1") is not True:
        failed = [k for k, v in (gate.get("per_role") or {}).items() if not (isinstance(v, Mapping) and v.get("pass") is True)]
        if not (isinstance(gate.get("Q3"), Mapping) and gate["Q3"].get("pass") is True):
            failed.append("Q3")
        raise DaggerError(f"DAgger round-1 offline gate failed {failed}: export and rollouts are refused")


# --- export ---------------------------------------------------------------------------------------

def export_r1(new_state: Mapping[str, Any], *, fit_report: Mapping[str, Any], gate: Mapping[str, Any], dataset_report: Mapping[str, Any], dataset_files: Mapping[str, Any], lineage: Mapping[str, Any], out_dir: Path = OUT_R1, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    assert_r1_gate(gate)
    r0a_pre = VS.source_files_table(A.OUT_R0A / "rl_module")
    v1_pre = VS.source_files_table(V2.V1_MODULE)
    out_dir = Path(out_dir)
    if out_dir.exists():
        raise FileExistsError(f"no-clobber: {out_dir} exists")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    lock, token = RF.acquire_export_lock(out_dir)
    staging: Path | None = None
    promoted = False
    try:
        if out_dir.exists():
            raise FileExistsError(f"final path exists (checked under the lock): {out_dir}")
        staging = RF._staging_dir_for(out_dir)
        stage_module = staging / "rl_module"
        stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(A.OUT_R0A / "rl_module" / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(A.OUT_R0A / "rl_module" / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in new_state.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        if str(R.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in new_state.items()}, reloaded)
        if not cmp.get("exact"):
            raise DaggerError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=fit_report["new_actor_digest"])
        inv = RF.invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise DaggerError("clock invariance not bit-identical")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": len(names35), "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": SIGMA_NOTE, "derived_from": C.rel(A.OUT_R0A / "rl_module"), "source_actor_digest": RO.PIN_R0A_ACTOR_DIGEST, "contract": "deployable_markov_controller_state"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(A.OUT_R0A / "rl_module") != r0a_pre or VS.source_files_table(V2.V1_MODULE) != v1_pre:
            raise DaggerError("SOURCE (R0a or V1) changed during the export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_dagger_r1.1",
            "amendments": {"rev3a": V2.PIN_AMENDMENT, "rev3b": A.PIN_AMENDMENT_REV3B, "rev3c": RO.PIN_AMENDMENT_REV3C, "rev3d": PIN_AMENDMENT_REV3D},
            "protocol_parents_immutable": {"v26b_protocol.json": V2.PIN_PROTOCOL_JSON, "PROTOCOL_V26B.md": V2.PIN_PROTOCOL_MD},
            "authorized_stage": AUTHORIZED_STAGE,
            "lineage": dict(lineage),
            "init": {"module": C.rel(A.OUT_R0A / "rl_module"), "actor_digest": RO.PIN_R0A_ACTOR_DIGEST, "files_sha256": r0a_pre, "r0a_receipt_sha256": RO.PIN_R0A_RECEIPT},
            "on_policy_source": {"trace_sha256": PIN_ROLLOUT_TRACE, "rollout_receipt_sha256": PIN_ROLLOUT_RECEIPT, "audit_addendum_sha256": PIN_ROLLOUT_ADDENDUM},
            "dataset": {"report": dict(dataset_report), "files": dict(dataset_files)},
            "fit": {k: fit_report[k] for k in ("tool", "rows", "budget", "var_per_joint", "init_actor_digest", "new_actor_digest", "sigma_note", "aux_head")},
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1], "epochs": len(hist)},
            "loss_history_full": hist,
            "gate": dict(gate),
            "structure": struct, "save_reload_exact": True, "clock_invariance": inv,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": SIGMA_NOTE},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_dagger_r1.py": C.sha256_file(Path(__file__).resolve()), "v26b_r0a.py": C.sha256_file(HERE / "v26b_r0a.py"), "v26b_r0a_rollout.py": C.sha256_file(HERE / "v26b_r0a_rollout.py"), "v26b_v2.py": C.sha256_file(HERE / "v26b_v2.py"), "f2r_refit.py": C.sha256_file(VALIDATION_DIR / "f2r_bridge_2026-08-23" / "f2r_refit.py"), "warm_start.py": C.sha256_file(R.BASELINE_DIR / "warm_start.py")},
            "scope": "DAgger round-1 offline only: NO rollout of the new actor, no PPO, no critic, no production change; STOP for the Codex audit",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise DaggerError("canonical receipt on disk differs from the receipt in memory")
        canonical_sha = C.sha256_file(staging / RECEIPT_NAME)
        if out_dir.exists():
            raise FileExistsError(f"final path appeared during the export: {out_dir}")
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


# --- pipeline -------------------------------------------------------------------------------------

def load_r0a_init() -> dict[str, np.ndarray]:
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    state = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(A.OUT_R0A / "rl_module").items()}
    RF.validate_init_state(state, expected_actor_digest=RO.PIN_R0A_ACTOR_DIGEST)
    return state


def save_r1_dataset(data: Mapping[str, np.ndarray], report: Mapping[str, Any]) -> dict[str, Any]:
    out_dir = VA.OUT_ROOT / "datasets"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    import io, os
    path = R.unique_artifact_path(out_dir, f"v26b_dataset_DAGGER_R1_{stamp}", ".npz")
    buf = io.BytesIO(); np.savez(buf, **{k: np.asarray(v) for k, v in data.items()})
    part = path.with_name(path.name + f".part-{os.getpid()}")
    with open(part, "xb") as fh:
        fh.write(buf.getvalue()); fh.flush(); os.fsync(fh.fileno())
    os.replace(part, path)
    rpath = R.unique_artifact_path(out_dir, f"v26b_dataset_DAGGER_R1_receipt_{stamp}", ".json")
    VA._atomic_fill_reserved(rpath, json.dumps({"schema": "v26b_dagger_r1_dataset.1", "file": {"path": C.rel(path), "sha256": C.sha256_file(path), "rows": int(data["obs35"].shape[0])}, **dict(report), "generated_at_utc": C.utc_now()}, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"npz": {"path": C.rel(path), "sha256": C.sha256_file(path)}, "receipt": {"path": C.rel(rpath), "sha256": C.sha256_file(rpath)}}


def run_r1(*, authorized_stage: str | None, out_dir: Path = OUT_R1, progress: bool = True, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise DaggerError(f"DAgger round 1 requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_r1()
    data, ds_report = build_r1_dataset()
    ds_files = save_r1_dataset(data, ds_report)
    init_state = load_r0a_init()
    new_state, fit_report = fit_dagger_r1(init_state, data, progress=progress)
    gate = evaluate_r1_gate(init_state, new_state, data)
    assert_r1_gate(gate)
    return export_r1(new_state, fit_report=fit_report, gate=gate, dataset_report=ds_report, dataset_files=ds_files, lineage=lineage, out_dir=out_dir, runtime_status=runtime_status)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B DAgger round 1 (rev3d; dry by default; NO rollout)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_r1()
        data, rep = build_r1_dataset()
        print(json.dumps({"mode": "dry (no fit, no export)", "lineage_ok": True, "rows_after_dedup": int(data["obs35"].shape[0]), "collisions": rep["collisions"]["count"]}, indent=2))
        return 0
    runtime: dict[str, Any] = {}
    canonical = run_r1(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "promotion": runtime.get("promotion"), "new_actor_digest": canonical["fit"]["new_actor_digest"], "gate_pass": canonical["gate"]["pass_r1"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
