"""V26B R0a — PRODUCTIVE mono-role BC on the nominal IK teacher (amendment rev3b).

July-faithful first stage (2026-07-11 'Strategia 3'): mono-role behaviour
cloning on the NOMINAL prescribed-IK teacher, init = the approved V1 35D
transplant.  Gate (fail-closed): RMSE vs u_IK <= 0.15 per joint on ALL 500
nominal-grid rows + Q3 invariants; the V26-mean comparison is DIAGNOSTIC only.
Frozen budget (300 epochs / Adam 1e-4 / batch 256 / seed 2026) and identical
regularisers; log-std frozen; no critic; full mean network.

FULL 64-hex IK-cache digests are verified fail-closed in this productive path
(rev3b hardened pins).  The diagnostic task-only state is never reused: the fit
re-runs here.  No rollout, no DAgger, no alt-start rows, no raw-V26 labels.
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
import v26b_student as VS  # noqa: E402
import v26b_v2 as V2  # noqa: E402


class R0aError(RuntimeError):
    pass


AUTHORIZED_STAGE_R0A = "V26B-V2-R0A"
AMENDMENT_REV3B = HERE / "v26b_amendment_rev3b_r0a_order_correction.json"
PIN_AMENDMENT_REV3B = "52c879996ce36302474d1f7efe0da8fa86271c82418ba2efa451b20ff5fb23e7"
PIN_DIAG_REPORT = "a7a740d72cdb08c9a6152640408d7d17e136d69efc45d316302760a59e4b48cd"
PIN_DIAG_RESULTS = "61a09baf9a2ad3465312687180e4ad07fc7d6dd6f1480cf6a5637e8bb0f1b5f8"
PIN_R0_FAIL_REPORT = "994ec5974b316e77cd0a418146c4011b81eb42b4e9818cbb999cbfc41b5e84be"
DIAG_REPORT = C.REPO / "reports" / "user" / "2026-08-24_v26b_r0_feasibility_diagnostic.md"
R0_FAIL_REPORT = C.REPO / "reports" / "user" / "2026-08-24_v26b_v2_r0_offline_gate_fail.md"
DIAG_RESULTS = VA.OUT_ROOT / "r0_feasibility_diag_20260824_144743.json"

IK_CACHE_DIGESTS_FULL = {  # rev3b hardened pins: FULL 64-hex, fail-closed in this productive path
    "minus020": "f97ad154f75541626565f7d6fd392dface4f723c2861143f1dc8172536767230",
    "nominal": "3dd878d4d6d2930d730c1a67f39d6799f20221e7659a435c02d062bfd553d9b0",
    "plus020": "f15d624cc910b815ee4c511e6702e1312cc4be0f67e93ae435b3e14b850f7d20",
}

R0A_RMSE_MAX_PER_JOINT = 0.15
HOLDOUT_FRACTION = 0.2  # July-style 400/100 diagnostic split (never a gate)
OUT_R0A = VA.OUT_ROOT / "student" / "V2_R0a"
RECEIPT_NAME = "v26b_r0a_receipt.json"


# --- lineage (rev3 + rev3a + rev3b + negative evidence + sources) ---------------------------------

def verify_lineage_rev3b() -> dict[str, Any]:
    lin = V2.verify_lineage()  # rev3 parents, rev3a, V1 receipt+module, V26, coverage
    for path, pin, key in ((AMENDMENT_REV3B, PIN_AMENDMENT_REV3B, "amendment_rev3b"), (DIAG_REPORT, PIN_DIAG_REPORT, "diagnostic_report"), (DIAG_RESULTS, PIN_DIAG_RESULTS, "diagnostic_results"), (R0_FAIL_REPORT, PIN_R0_FAIL_REPORT, "r0_fail_negative_evidence")):
        p = Path(path)
        if p.is_symlink() or not p.is_file():
            raise R0aError(f"lineage file missing (or symlink): {p}")
        got = C.sha256_file(p)
        if got != pin:
            raise R0aError(f"lineage violated: {p.name} sha {got} != pinned {pin}")
        lin[key] = {"path": C.rel(p), "sha256": got}
    return lin


def verify_ik_caches_full_productive() -> dict[str, Any]:
    """rev3b: FULL-digest fail-closed verification in the productive path; returns the caches."""
    caches = {}
    digests = {}
    for start in R.STARTS:
        cc = L.load_cache(R.OUT_CACHE, start)
        d = cc.digest()
        if d != IK_CACHE_DIGESTS_FULL[start]:
            raise R0aError(f"IK cache {start}: digest {d} != FULL rev3b pin {IK_CACHE_DIGESTS_FULL[start]}")
        if cc.ik_action is None or cc.rows != 500:
            raise R0aError(f"IK cache {start}: missing u_IK or wrong grid")
        caches[start] = cc
        digests[start] = d
    return {"caches": caches, "digests": digests}


# --- dataset --------------------------------------------------------------------------------------

def build_r0a_dataset() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Exactly the 500 nominal-grid rows: obs35 of the pinned DET nominal anchor trace, labels
    u_IK from the FULL-pinned nominal cache (exact grid lookup).  No alt-start, no raw-V26 labels."""
    pins = R.verify_anchor_pins()
    if not pins.get("all_match"):
        raise R0aError("pinned det anchors do not match their content digests")
    ik = verify_ik_caches_full_productive()
    cc = ik["caches"]["nominal"]
    traj = DS.trajectory_from_job(Path(R.ANCHORS["nominal"]["job_dir"]), expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.ascontiguousarray(np.asarray(traj["obs35"], dtype=np.float64).astype(np.float32))
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    if obs.shape != (500, R.ENV_ACTOR_WIDTH):
        raise R0aError(f"nominal det anchor is not 500x35: {obs.shape}")
    idx = cc.lookup(t_pre)  # exact float equality on the grid
    u_ik = np.ascontiguousarray(cc.ik_action[idx])
    clock = np.ascontiguousarray(cc.clock[idx].astype(np.float32))
    keys = {}
    dup = []
    for i in range(obs.shape[0]):
        k = obs[i].tobytes()
        if k in keys:
            dup.append((keys[k], i))
        else:
            keys[k] = i
    if dup:
        raise R0aError(f"bitwise duplicates inside the 500 nominal det rows: {dup[:5]} (unexpected; STOP for audit)")
    if not (np.all(np.isfinite(u_ik)) and np.all(np.abs(u_ik) <= 1.0)):
        raise R0aError("u_IK labels non-finite or outside [-1, 1]")
    # July-style deterministic 400/100 split: DIAGNOSTIC only (never a gate, no early stopping)
    rng = np.random.default_rng(2026)
    perm = rng.permutation(500)
    n_hold = int(round(500 * HOLDOUT_FRACTION))
    hold_idx = np.sort(perm[:n_hold]); train_idx = np.sort(perm[n_hold:])
    split_digest = DS.sha256_array(hold_idx.astype(np.int64))
    role = {
        "obs35": obs, "t_pre": t_pre, "actions": u_ik.astype(np.float32), "clock": clock,
        "seed": np.full(500, R.DET_SEED, dtype=np.int64),
        "purpose": np.asarray(["ik_nominal_det"] * 500, dtype=str),
        "job_id": np.asarray(["anchor_det_nominal"] * 500, dtype=str),
    }
    report = {
        "rows": 500, "bitwise_duplicates": 0,
        "source_trace": {"job_dir": C.rel(Path(R.ANCHORS["nominal"]["job_dir"])), "trace_sha256": traj["trace_sha256"], "pinned": R.ANCHORS["nominal"]["trace_sha256"]},
        "labels": {"source": "nominal privileged cache (FULL rev3b pin)", "cache_digest": ik["digests"]["nominal"], "rule": "u_IK(t_pre) exact grid lookup; knee AND ankle"},
        "exclusions": "no alt-start rows, no raw-V26 action labels (rev3b)",
        "deviation_from_july": "July's 500 BC states were teacher-visited (collect_teacher_dataset, target_domain_imitation.py:328); R0a uses V26-det-visited states on the same nominal grid (no env stepping authorised): same-state IK relabelling",
        "diagnostic_split_july_style": {"train_rows": int(train_idx.size), "holdout_rows": int(n_hold), "rule": "numpy default_rng(2026) permutation (July: rng.permutation + validation_fraction, target_domain_imitation.py:794-918; 11/07 used 400/100 with early stopping; the frozen 300-epoch budget corresponds to the historical fixed_final_epoch mode requiring validation_fraction=0, patience=0)", "holdout_indices_sha256": split_digest, "role": "DIAGNOSTIC only, never a gate"},
        "terminology_note": "continuity_anchor/target_anchor (11/07 report) are runtime slew-limiter anchors, NOT dataset roles",
    }
    return role, {"report": report, "train_idx": train_idx, "hold_idx": hold_idx, "ik_digests": ik["digests"]}


# --- productive mono-role fit (same loop as the diagnostic, re-run here; state never reused) ------

def fit_r0a(init_state: Mapping[str, Any], role: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch
    import torch.nn.functional as F

    budget = dict(budget or V2.BUDGET_R0)
    V2._role_contract(role, allowed_purposes={"ik_nominal_det"}, what="r0a_nominal_ik")
    info = RF.validate_init_state(init_state, expected_actor_digest=None)
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.asarray(role["obs35"], dtype=f32))
    tgt = torch.as_tensor(np.asarray(role["actions"], dtype=f32))
    clk = torch.as_tensor(np.asarray(role["clock"], dtype=f32))
    var_role = np.var(np.asarray(role["actions"], dtype=np.float64), axis=0)
    if np.any(var_role <= 0.0):
        raise R0aError("degenerate per-joint variance")
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
    report = {"tool": "v26b_r0a.fit_r0a (productive mono-role BC, rev3b)", "rows": n, "budget": budget, "var_role_per_joint": var_role.tolist(), "init_actor_digest": info["actor_digest"], "new_actor_digest": RF.actor_state_digest(new_state), "history": history, "structure": struct, "aux_head": {"exported": False, "training_time_only": True}}
    return new_state, report


# --- gate -----------------------------------------------------------------------------------------

def evaluate_r0a_gate(init_state: Mapping[str, Any], state: Mapping[str, Any], role: Mapping[str, np.ndarray], split: Mapping[str, Any]) -> dict[str, Any]:
    f32 = np.float32
    m = RF.numpy_mean(state, np.asarray(role["obs35"], dtype=f32))
    y = np.asarray(role["actions"], dtype=np.float64)
    rmse_all = np.sqrt(np.mean((m - y) ** 2, axis=0)).tolist()
    hold = np.asarray(split["hold_idx"], dtype=np.int64); train = np.asarray(split["train_idx"], dtype=np.int64)
    rmse_hold = np.sqrt(np.mean((m[hold] - y[hold]) ** 2, axis=0)).tolist()
    rmse_train = np.sqrt(np.mean((m[train] - y[train]) ** 2, axis=0)).tolist()
    struct = RF.validate_init_state(state, expected_actor_digest=None)
    inv = RF.invariance_test(state, np.asarray(role["obs35"], dtype=f32)[:64])
    q3 = {"ten_keys": tuple(state.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "save_reload_exact": "verified at export"}
    # DIAGNOSTIC (not a criterion): distance from the deterministic V26 mean on the same rows
    v26_raw = None
    try:
        traj = DS.trajectory_from_job(Path(R.ANCHORS["nominal"]["job_dir"]), expected_width=R.ENV_ACTOR_WIDTH)
        v26_raw = np.sqrt(np.mean((m - np.asarray(traj["b_raw_action"], dtype=np.float64)) ** 2, axis=0)).tolist()
    except Exception as exc:  # diagnostic only: never blocks
        v26_raw = f"unavailable: {exc}"
    out = {
        "RMSE_vs_uIK_all_500": {"knee_ankle": rmse_all, "max_per_joint": R0A_RMSE_MAX_PER_JOINT, "pass": bool(all(v <= R0A_RMSE_MAX_PER_JOINT for v in rmse_all))},
        "RMSE_vs_uIK_holdout_100_diagnostic": rmse_hold,
        "RMSE_vs_uIK_train_400_diagnostic": rmse_train,
        "Q3": {**q3, "pass": bool(q3["ten_keys"] and q3["clock_columns_zero"] and q3["logstd_constant"] and q3["clock_invariance_bit_identical"])},
        "diagnostic_vs_V26_mean_rmse": v26_raw,
        "diagnostic_note": "the V26-mean comparison is recorded as DIAGNOSTICS (rev3b), not a criterion; the holdout RMSE is diagnostic (July-style split), the gate reads ALL 500 rows",
        "parameter_shift_sq_informational": float(sum(np.sum((np.asarray(state[k], np.float64) - np.asarray(init_state[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS)),
    }
    out["pass_r0a"] = bool(out["RMSE_vs_uIK_all_500"]["pass"] and out["Q3"]["pass"])
    return out


def assert_r0a_gate(gate: Mapping[str, Any]) -> None:
    if gate.get("pass_r0a") is not True:
        failed = [k for k in ("RMSE_vs_uIK_all_500", "Q3") if not (isinstance(gate.get(k), Mapping) and gate[k].get("pass") is True)]
        raise R0aError(f"R0a gate failed {failed}: export and rollouts are refused")


# --- export ---------------------------------------------------------------------------------------

def export_r0a(new_state: Mapping[str, Any], *, fit_report: Mapping[str, Any], gate: Mapping[str, Any], dataset_report: Mapping[str, Any], lineage: Mapping[str, Any], ik_digests: Mapping[str, str], out_dir: Path = OUT_R0A, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    assert_r0a_gate(gate)
    v1_pre = VS.source_files_table(V2.V1_MODULE)
    v26_pre = VS.source_files_table(Path(R.TEACHER["module"]))
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
        shutil.copy2(V2.V1_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(V2.V1_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in new_state.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        if str(R.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in new_state.items()}, reloaded)
        if not cmp.get("exact"):
            raise R0aError(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=fit_report["new_actor_digest"])
        inv = RF.invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise R0aError("clock invariance not bit-identical")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": len(names35), "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": VS.SIGMA_STATEMENT, "derived_from": C.rel(V2.V1_MODULE), "source_actor_digest": V2.PIN_V1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(V2.V1_MODULE) != v1_pre or VS.source_files_table(Path(R.TEACHER["module"])) != v26_pre:
            raise R0aError("SOURCE (V1 or V26) changed during the export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_r0a.1",
            "protocol_parents_immutable": {"v26b_protocol.json": V2.PIN_PROTOCOL_JSON, "PROTOCOL_V26B.md": V2.PIN_PROTOCOL_MD},
            "amendments": {"rev3a": {"path": C.rel(V2.AMENDMENT_FILE), "sha256": V2.PIN_AMENDMENT}, "rev3b": {"path": C.rel(AMENDMENT_REV3B), "sha256": PIN_AMENDMENT_REV3B}},
            "negative_evidence": {"r0_fail_report": {"path": C.rel(R0_FAIL_REPORT), "sha256": PIN_R0_FAIL_REPORT}, "diagnostic_report": {"path": C.rel(DIAG_REPORT), "sha256": PIN_DIAG_REPORT}, "diagnostic_results": {"path": C.rel(DIAG_RESULTS), "sha256": PIN_DIAG_RESULTS}},
            "authorized_stage": AUTHORIZED_STAGE_R0A,
            "lineage": dict(lineage),
            "init": {"module": C.rel(V2.V1_MODULE), "actor_digest": V2.PIN_V1_ACTOR_DIGEST, "module_state_sha256": V2.PIN_V1_MODULE_STATE, "v1_receipt_sha256": V2.PIN_V1_RECEIPT, "files_sha256": v1_pre},
            "v26_source_untouched": v26_pre,
            "ik_cache_digests_full": dict(ik_digests),
            "dataset": dict(dataset_report),
            "fit": {k: fit_report[k] for k in ("tool", "rows", "budget", "var_role_per_joint", "init_actor_digest", "new_actor_digest", "aux_head")},
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1], "epochs": len(hist)},
            "loss_history_full": hist,
            "gate": dict(gate),
            "structure": struct,
            "save_reload_exact": True,
            "clock_invariance": inv,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": VS.SIGMA_STATEMENT},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_r0a.py": C.sha256_file(Path(__file__).resolve()), "v26b_v2.py": C.sha256_file(HERE / "v26b_v2.py"), "v26b_r0_diag.py": C.sha256_file(HERE / "v26b_r0_diag.py"), "v26b_student.py": C.sha256_file(HERE / "v26b_student.py"), "v26b_anchors.py": C.sha256_file(HERE / "v26b_anchors.py"), "f2r_refit.py": C.sha256_file(VALIDATION_DIR / "f2r_bridge_2026-08-23" / "f2r_refit.py"), "warm_start.py": C.sha256_file(R.BASELINE_DIR / "warm_start.py")},
            "scope": "R0a mono-role BC only: no rollout, no DAgger, no V3, no PPO, no critic, no production change; STOP after export for the architect audit",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(),
            "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise R0aError("canonical receipt on disk differs from the receipt in memory")
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

def run_r0a(*, authorized_stage: str | None, out_dir: Path = OUT_R0A, progress: bool = True, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE_R0A:
        raise R0aError(f"R0a requires --authorized-stage {AUTHORIZED_STAGE_R0A} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage_rev3b()
    role, ds = build_r0a_dataset()
    init_state = V2.load_v1_init()
    new_state, fit_report = fit_r0a(init_state, role, progress=progress)
    gate = evaluate_r0a_gate(init_state, new_state, role, ds)
    assert_r0a_gate(gate)
    return export_r0a(new_state, fit_report=fit_report, gate=gate, dataset_report=ds["report"], lineage=lineage, ik_digests=ds["ik_digests"], out_dir=out_dir, runtime_status=runtime_status)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B R0a: mono-role BC on the nominal IK teacher (rev3b; dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        lineage = verify_lineage_rev3b()
        role, ds = build_r0a_dataset()
        print(json.dumps({"mode": "dry (no fit, no writes)", "lineage_ok": True, "rows": int(role["obs35"].shape[0]), "dataset": ds["report"]["labels"]}, indent=2))
        return 0
    runtime: dict[str, Any] = {}
    canonical = run_r0a(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "promotion": runtime.get("promotion"), "new_actor_digest": canonical["fit"]["new_actor_digest"], "gate_pass": canonical["gate"]["pass_r0a"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
