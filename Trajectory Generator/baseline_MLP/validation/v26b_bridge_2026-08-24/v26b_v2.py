"""V26B V2 — R0 static offline adaptation (architect GO 2026-08-24, after the V1 gate).

R0 per protocol ``V26B-bridge-rev3`` (parents IMMUTABLE, pinned below) read with
amendment ``rev3a_q2b_causal_order`` (ADDITIVE file, pinned below): the dataset
is anchors_preservation (nominal-start anchor rows, target = deterministic V26
mean) + the alternative-start prescribed IK rows reconstructed on the anchor
grids (minus020/plus020 anchor rows, labels u_IK knee AND ankle from the pinned
F2R privileged caches); R0 must pass Q1 + Q2 + Q3 before the first V3; Q2b is
not applicable at R0 (no on-policy rows exist yet).

Role assignment (start split, July-faithful — documented for audit): the rev3
criteria names encode it (Q2 = "alt_start_rows"); the July recipe the protocol
replicates used nominal source-actor anchors + alt-start teacher rows; the
measured |u_T - u_IK| gap (~0.37 mean per joint on identical states) makes any
same-input dual-target reading unsatisfiable under Q1 AND Q2; the two roles are
bitwise disjoint (0 cross-start duplicate observations, verified fail-closed).

The training loop is a FAITHFUL MIRROR of the immutable
``f2r_refit.fit_student_preserving`` (bit-identical result proven by the
selftest on numerically identical inputs); only the dataset CONTRACT (V26B
roles/seeds) and the CRITERIA (Q instead of T1R's P) differ.  beta = 1.0 frozen
makes the (task, preservation) weight assignment numerically symmetric with the
protocol's written form (anchor term + beta * teacher term).

No rollout, no V3, no PPO, no critic, no production change.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
F2R_DIR = VALIDATION_DIR / "f2r_bridge_2026-08-23"
F1_DIR = VALIDATION_DIR / "f1_ablation_2026-08-23"
F0_DIR = VALIDATION_DIR / "f0_freeze_2026-08-22"
for _entry in (str(F0_DIR), str(F1_DIR), str(F2R_DIR), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402


class V2Error(RuntimeError):
    """Fail-closed violation in the V2/R0 stage."""


AUTHORIZED_STAGE_R0 = "V26B-V2-R0"

# --- immutable lineage pins (verified fail-closed before any fit/export) --------------------------
PIN_PROTOCOL_JSON = "5e0b6a5cdbec89f97b03bb0262142e53ba4c03affe0dd31fde9f813c778708d3"
PIN_PROTOCOL_MD = "7ef70cbf2f7253406e15e0714c68c05455d672120a039dd43fdfa139152b913c"
AMENDMENT_FILE = HERE / "v26b_amendment_rev3a_q2b_causal_order.json"
PIN_AMENDMENT = "f74672ad9d60775f18049bec058d793c246bac739fbf18a023a32798d9bb78a0"
V1_DIR = VA.OUT_ROOT / "student" / "V1_35D_transplant"
V1_RECEIPT = V1_DIR / "v26b_v1_receipt.json"
PIN_V1_RECEIPT = "65b447ea331d58b457fe6df48e67697dbd6693986552411c0f9780c75ffb995c"
V1_MODULE = V1_DIR / "rl_module"
PIN_V1_ACTOR_DIGEST = "ae846220a6f7f1ac1289ccc9636e3ad2e5bc7842ba7ece0b62bb9d7590e7f587"
PIN_V1_MODULE_STATE = "16c2d1ae9fb4e77fffa092d74d37e78f54ba24d990774e91bf1d412c551bb031"
PIN_F2R_CACHE_DIGESTS = {  # privileged caches (anchor grids + u_IK), produced by the audited F2R T1 build
    "minus020": "f97ad154f755", "nominal": "3dd878d4d6d2", "plus020": "f15d624cc910",  # first-12 prefixes, full digest re-checked at load
}

Q1_MAX = 0.10  # preservation RMSE per joint vs deterministic V26 mean (anchors)
Q2_MAX = 0.15  # RMSE per joint vs u_IK on the alternative-start rows
EXPECTED_COUNTS = {"pres": 6438, "task": 12876, "total": 19314, "cross_role_collisions": 0}
ALT_STARTS = ("minus020", "plus020")
PRES_START = "nominal"

BUDGET_R0 = {  # == protocol budget_frozen (verified against the parent at runtime)
    "epochs": 300, "batch_size": 256, "optimizer": "Adam", "lr": 0.0001, "seed": 2026,
    "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 0.001,
    "selection": "closed_loop_only", "aux_head": "training_time_only",
}
BETA_R0 = 1.0

OUT_DATASETS = VA.OUT_ROOT / "datasets"
OUT_R0 = VA.OUT_ROOT / "student" / "V2_R0"
RECEIPT_NAME = "v26b_v2_r0_receipt.json"


# --- lineage verification -------------------------------------------------------------------------

def verify_lineage() -> dict[str, Any]:
    """Fail-closed: rev3 parents byte-identical to the V1-pinned hashes; amendment file pinned;
    V1 receipt + module and V26 source unchanged."""
    checks: dict[str, Any] = {}
    for path, pin, key in ((VA.PROTOCOL_JSON, PIN_PROTOCOL_JSON, "protocol_json"), (VA.PROTOCOL_MD, PIN_PROTOCOL_MD, "protocol_md"), (AMENDMENT_FILE, PIN_AMENDMENT, "amendment_rev3a")):
        if Path(path).is_symlink() or not Path(path).is_file():
            raise V2Error(f"lineage file missing (or symlink): {path}")
        got = C.sha256_file(Path(path))
        if got != pin:
            raise V2Error(f"lineage violated: {Path(path).name} sha {got} != pinned {pin}")
        checks[key] = {"path": C.rel(Path(path)), "sha256": got}
    proto = VA.load_protocol()  # rev3 binding (id, coverage, seeds, grid)
    b = proto["stages"]["V2_target_domain_adaptation"]["budget_frozen"]
    if any(BUDGET_R0[k] != b[k] for k in BUDGET_R0):
        raise V2Error(f"frozen budget constants differ from the rev3 parent: {b}")
    q = proto["stages"]["V2_target_domain_adaptation"]["offline_criteria_fail_closed_before_any_rollout"]
    if float(q["Q1_preservation_on_anchors_rmse_max_per_joint"]) != Q1_MAX or float(q["Q2_vs_uIK_rmse_max_per_joint_alt_start_rows"]) != Q2_MAX:
        raise V2Error("Q thresholds differ from the rev3 parent")
    if C.sha256_file(V1_RECEIPT) != PIN_V1_RECEIPT:
        raise V2Error("V1 receipt digest != pinned")
    v1r = json.loads(V1_RECEIPT.read_text(encoding="utf-8"))
    for name, sha in v1r["output_files_sha256"].items():
        got = C.sha256_file(V1_MODULE / name)
        if got != sha:
            raise V2Error(f"V1 module file {name} changed: {got} != receipt {sha}")
    if C.sha256_file(V1_MODULE / "module_state.pkl") != PIN_V1_MODULE_STATE:
        raise V2Error("V1 module_state != pinned")
    v26_state_sha = C.sha256_file(Path(R.TEACHER["module"]) / "module_state.pkl")
    if v26_state_sha != R.TEACHER["module_state_sha256"]:
        raise V2Error("V26 source module_state changed")
    if VS.COVERAGE_JSON.is_symlink() or C.sha256_file(VS.COVERAGE_JSON) != VS.COVERAGE_SHA256:
        raise V2Error("coverage JSON digest != architect-verified pin")
    checks["v1_receipt_sha256"] = PIN_V1_RECEIPT
    checks["v1_actor_digest"] = PIN_V1_ACTOR_DIGEST
    checks["v26_module_state_sha256"] = v26_state_sha
    checks["coverage_sha256"] = VS.COVERAGE_SHA256
    return checks


def load_ik_caches() -> dict[str, Any]:
    caches = {}
    for start in R.STARTS:
        cc = L.load_cache(R.OUT_CACHE, start)
        d = cc.digest()
        if not d.startswith(PIN_F2R_CACHE_DIGESTS[start]):
            raise V2Error(f"privileged cache {start} digest {d[:12]} != pinned prefix {PIN_F2R_CACHE_DIGESTS[start]}")
        if cc.ik_action is None or cc.rows != 500:
            raise V2Error(f"privileged cache {start} lacks u_IK or is not the 500-row anchor grid")
        caches[start] = cc
    return caches


# --- dataset build --------------------------------------------------------------------------------

def _start_of_job_id(job_id: str) -> str:
    for s in R.STARTS:
        if f"__{s}__" in job_id or job_id.endswith(s):
            return s
    raise V2Error(f"cannot derive the start from job_id {job_id!r}")


def _seed_of_job_id(job_id: str) -> int:
    if job_id.startswith("anchor_det_"):
        return R.DET_SEED
    tail = job_id.rsplit("seed", 1)
    if len(tail) == 2 and tail[1].isdigit():
        return int(tail[1])
    raise V2Error(f"cannot derive the seed from job_id {job_id!r}")


def build_r0_dataset() -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, Any]]:
    """R0 roles from the coverage-PASS anchor collection (fail-closed, deterministic).

    preservation = nominal rows, actions = deterministic V26 mean (collection targets);
    task = minus020/plus020 rows, actions = u_IK from the pinned caches (exact grid lookup);
    clock = prescribed (sin, cos) from the caches (aux-head target, training-time only)."""
    ctx = VA.teacher_context()
    arrays, coverage = VA.collect_anchor_rows(VA.job_specs(), ctx, include_det=True)  # re-validates every trace + coverage gate
    caches = load_ik_caches()
    obs = arrays["obs35"]; t_pre = arrays["t_pre"]; u_t = arrays["targets"]; jid = arrays["job_id"]
    starts = np.asarray([_start_of_job_id(j) for j in jid])
    seeds = np.asarray([_seed_of_job_id(j) for j in jid], dtype=np.int64)
    pres_mask = starts == PRES_START
    task_mask = ~pres_mask
    if not all(s in ALT_STARTS for s in np.unique(starts[task_mask])):
        raise V2Error("task rows contain a non-alternative start")
    # cross-role bitwise collisions (expected exactly 0; any collision -> architect audit)
    pres_keys = {obs[i].tobytes() for i in np.where(pres_mask)[0]}
    collisions = [i for i in np.where(task_mask)[0] if obs[i].tobytes() in pres_keys]
    if len(collisions) != EXPECTED_COUNTS["cross_role_collisions"]:
        raise V2Error(f"{len(collisions)} cross-role bitwise collisions (expected {EXPECTED_COUNTS['cross_role_collisions']}): STOP for architect audit")
    def role_arrays(mask: np.ndarray, actions: np.ndarray, purpose_prefix: str) -> dict[str, np.ndarray]:
        idx = np.where(mask)[0]
        clocks = np.empty((idx.size, 2), dtype=np.float32)
        for k, i in enumerate(idx):
            cc = caches[str(starts[i])]
            clocks[k] = cc.clock[cc.lookup(np.asarray([t_pre[i]]))[0]].astype(np.float32)
        purpose = np.asarray([f"{purpose_prefix}_det" if str(j).startswith("anchor_det_") else f"{purpose_prefix}_stoch" for j in jid[idx]], dtype=str)
        return {"obs35": np.ascontiguousarray(obs[idx]), "t_pre": np.ascontiguousarray(t_pre[idx]), "actions": np.ascontiguousarray(actions[idx].astype(np.float32)), "clock": clocks, "seed": np.ascontiguousarray(seeds[idx]), "purpose": purpose, "job_id": np.ascontiguousarray(jid[idx]), "start": np.ascontiguousarray(starts[idx])}
    # u_IK per task row (exact grid lookup on its own start's cache)
    u_ik = np.empty_like(u_t)
    for start in ALT_STARTS:
        m = starts == start
        cc = caches[start]
        u_ik[m] = cc.ik_action[cc.lookup(t_pre[m])]
    pres = role_arrays(pres_mask, u_t, "anchor_nom")
    task = role_arrays(task_mask, u_ik, "ik_alt")
    for role_name, role in (("pres", pres), ("task", task)):
        keys = {role["obs35"][i].tobytes() for i in range(role["obs35"].shape[0])}
        if len(keys) != role["obs35"].shape[0]:
            raise V2Error(f"bitwise duplicate inside role {role_name} (within-role dedup violated)")
    if pres["obs35"].shape[0] != EXPECTED_COUNTS["pres"] or task["obs35"].shape[0] != EXPECTED_COUNTS["task"]:
        raise V2Error(f"role counts {pres['obs35'].shape[0]}/{task['obs35'].shape[0]} != expected {EXPECTED_COUNTS['pres']}/{EXPECTED_COUNTS['task']}")
    if not (np.all(np.abs(task["actions"]) <= 1.0) and np.all(np.isfinite(task["actions"]))):
        raise V2Error("u_IK labels outside [-1, 1] or non-finite")
    report = {
        "roles": {
            "anchors_preservation": {"start": PRES_START, "rows": int(pres["obs35"].shape[0]), "targets": "deterministic V26 mean on visited obs39 (coverage-verified <= 1e-5 vs recorded mean)", "seeds": sorted({int(s) for s in pres["seed"]})},
            "ik_teacher_rows": {"starts": list(ALT_STARTS), "rows": int(task["obs35"].shape[0]), "targets": "u_IK knee+ankle reconstructed on the anchor grids (pinned F2R privileged caches, exact float lookup)", "seeds": sorted({int(s) for s in task["seed"]})},
        },
        "role_assignment_rationale": "start split (July-faithful): Q2 is named alt_start_rows in rev3; July used nominal anchors + alt-start teacher rows; |u_T-u_IK| ~0.37 mean per joint on identical states makes a same-input dual-target reading unsatisfiable under Q1 AND Q2; roles bitwise disjoint (0 cross-start duplicates)",
        "cross_role_collisions": {"count": len(collisions), "expected": EXPECTED_COUNTS["cross_role_collisions"], "rule": "any unexpected collision fails closed (reset rows would win for preservation with input digest + target delta reported)"},
        "dedup": {"within_role_bitwise_unique": True, "global_unique_rows": int(obs.shape[0])},
        "coverage": {"unique_rows": coverage["unique_rows"], "pass": coverage["pass"], "sha256_pinned": VS.COVERAGE_SHA256},
        "ik_caches": {s: caches[s].digest() for s in R.STARTS},
        "teacher": {"actor_digest": ctx["digest"], "module_state_sha256": ctx["module_state_sha256"]},
    }
    return task, pres, report


def save_r0_dataset(task: Mapping[str, np.ndarray], pres: Mapping[str, np.ndarray], report: Mapping[str, Any], out_dir: Path = OUT_DATASETS) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    files = {}
    import io, os
    def fill_bytes(path: Path, data: bytes) -> None:  # atomic fill of a reserved path
        part = path.with_name(path.name + f".part-{os.getpid()}")
        with open(part, "xb") as fh:
            fh.write(data); fh.flush(); os.fsync(fh.fileno())
        os.replace(part, path)
    for name, role in (("task", task), ("pres", pres)):
        path = R.unique_artifact_path(out_dir, f"v26b_dataset_R0_{name}_{stamp}", ".npz")
        buf = io.BytesIO()
        np.savez(buf, **{k: np.asarray(v) for k, v in role.items()})
        fill_bytes(path, buf.getvalue())
        files[name] = {"path": C.rel(path), "sha256": C.sha256_file(path), "rows": int(role["obs35"].shape[0])}
    receipt_path = R.unique_artifact_path(out_dir, f"v26b_dataset_R0_receipt_{stamp}", ".json")
    payload = {"schema": "v26b_r0_dataset.1", "files": files, **dict(report), "generated_at_utc": C.utc_now()}
    VA._atomic_fill_reserved(receipt_path, json.dumps(payload, indent=2, ensure_ascii=False, default=str) + "\n")
    payload["receipt_path"] = C.rel(receipt_path)
    payload["receipt_sha256"] = C.sha256_file(receipt_path)
    return payload


# --- V2 dataset contract --------------------------------------------------------------------------

_TASK_PURPOSES = {"ik_alt_stoch", "ik_alt_det"}
_PRES_PURPOSES = {"anchor_nom_stoch", "anchor_nom_det"}


def _role_contract(role: Mapping[str, Any], *, allowed_purposes: set[str], what: str) -> dict[str, Any]:
    obs = np.asarray(role["obs35"]); act = np.asarray(role["actions"]); clk = np.asarray(role["clock"])
    n = obs.shape[0]
    if obs.ndim != 2 or obs.shape[1] != R.ENV_ACTOR_WIDTH:
        raise V2Error(f"{what}: obs35 must be (N, 35), got {obs.shape}")
    if act.shape != (n, R.ACTION_DIM) or clk.shape != (n, 2):
        raise V2Error(f"{what}: actions must be (N, 2) and clock (N, 2)")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act)) and np.all(np.isfinite(clk))):
        raise V2Error(f"{what}: non-finite values")
    if n == 0:
        raise V2Error(f"{what}: empty role")
    seeds = np.asarray(role["seed"], dtype=np.int64).reshape(-1)
    if seeds.shape[0] != n:
        raise V2Error(f"{what}: seed array length mismatch")
    for s in sorted({int(v) for v in seeds}):
        if s == R.DET_SEED:
            continue  # the pinned det anchors (seed 123, read-only reuse)
        VA.assert_anchor_seed(s, context="anchor_collection_stoch")  # 124/125/126-128 and unknown seeds refused
    purposes = {str(p) for p in np.asarray(role["purpose"]).reshape(-1)}
    if not purposes <= allowed_purposes:
        raise V2Error(f"{what}: purposes {sorted(purposes - allowed_purposes)} not allowed (expected subset of {sorted(allowed_purposes)})")
    return {"rows": int(n), "seeds": sorted({int(v) for v in seeds}), "purposes": sorted(purposes)}


def assert_r0_contract(task: Mapping[str, Any], pres: Mapping[str, Any]) -> dict[str, Any]:
    return {"task": _role_contract(task, allowed_purposes=_TASK_PURPOSES, what="task(ik_alt)"), "preservation": _role_contract(pres, allowed_purposes=_PRES_PURPOSES, what="preservation(anchor_nom)")}


# --- Q criteria -----------------------------------------------------------------------------------

def evaluate_q_criteria(init_state: Mapping[str, Any], state: Mapping[str, Any], task: Mapping[str, Any], pres: Mapping[str, Any]) -> dict[str, Any]:
    f32 = np.float32
    pm = RF.numpy_mean(state, np.asarray(pres["obs35"], dtype=f32)); pa = np.asarray(pres["actions"], dtype=np.float64)
    q1 = np.sqrt(np.mean((pm - pa) ** 2, axis=0)).tolist()
    tm = RF.numpy_mean(state, np.asarray(task["obs35"], dtype=f32)); ta = np.asarray(task["actions"], dtype=np.float64)
    q2 = np.sqrt(np.mean((tm - ta) ** 2, axis=0)).tolist()
    struct = RF.validate_init_state(state, expected_actor_digest=None)
    inv = RF.invariance_test(state, np.asarray(task["obs35"], dtype=f32)[:64])
    q3 = {"ten_keys": tuple(state.keys()) == RF.EXPECTED_KEY_ORDER, "clock_columns_zero": bool(struct["clock_columns_zero"]), "logstd_constant": bool(struct["sigma_head"]["logstd_bias_exact"]), "clock_invariance_bit_identical": bool(inv["bit_identical"]), "save_reload_exact": "verified at export"}
    shift = float(sum(np.sum((np.asarray(state[k], np.float64) - np.asarray(init_state[k], np.float64)) ** 2) for k in RF.CANONICAL_KEYS))
    out = {
        "Q1": {"rmse_knee_ankle_vs_V26_mean_on_anchors": q1, "max_per_joint": Q1_MAX, "pass": bool(all(v <= Q1_MAX for v in q1))},
        "Q2": {"rmse_knee_ankle_vs_uIK_on_alt_start_rows": q2, "max_per_joint": Q2_MAX, "pass": bool(all(v <= Q2_MAX for v in q2))},
        "Q2b": {"applicable": False, "reason": "rev3a amendment: no on-policy rows exist at R0 (static); mandatory from R1 onwards", "amendment_sha256": PIN_AMENDMENT},
        "Q3": {**q3, "pass": bool(q3["ten_keys"] and q3["clock_columns_zero"] and q3["logstd_constant"] and q3["clock_invariance_bit_identical"])},
        "parameter_shift_sq_informational": shift,
        "max_abs_dW1": float(np.max(np.abs(np.asarray(state["pi.0.0.weight"], np.float64) - np.asarray(init_state["pi.0.0.weight"], np.float64)))),
    }
    out["pass_r0"] = bool(out["Q1"]["pass"] and out["Q2"]["pass"] and out["Q3"]["pass"])
    return out


def assert_q_r0(report: Mapping[str, Any]) -> dict[str, Any]:
    c = report.get("criteria") if isinstance(report, Mapping) else None
    if not isinstance(c, Mapping) or c.get("pass_r0") is not True:
        failed = [k for k in ("Q1", "Q2", "Q3") if not (isinstance(c, Mapping) and isinstance(c.get(k), Mapping) and c[k].get("pass") is True)]
        raise V2Error(f"R0 offline criteria failed {failed}: export and rollouts are refused (rev3a: Q1+Q2+Q3 at R0)")
    return dict(c)


# --- faithful mirror of f2r_refit.fit_student_preserving ------------------------------------------

def fit_r0(init_state: Mapping[str, Any], task: Mapping[str, Any], pres: Mapping[str, Any], *, budget: Mapping[str, Any] | None = None, beta: float = BETA_R0, progress: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    """Bit-faithful transcription of the immutable ``f2r_refit.fit_student_preserving`` training
    loop (equivalence proven by the selftest); V26B differences: the dataset contract
    (``assert_r0_contract``) and the criteria (Q instead of P)."""
    import torch
    import torch.nn.functional as F

    budget = dict(budget or BUDGET_R0)
    contract = assert_r0_contract(task, pres)
    info = RF.validate_init_state(init_state, expected_actor_digest=None)
    beta = float(beta)
    if not (beta > 0.0 and math.isfinite(beta)):
        raise V2Error("beta must be a positive finite number")
    torch.use_deterministic_algorithms(True); torch.set_num_threads(1); torch.manual_seed(int(budget["seed"])); np.random.seed(int(budget["seed"]))
    f32 = np.float32
    obs = torch.as_tensor(np.concatenate([np.asarray(task["obs35"], dtype=f32), np.asarray(pres["obs35"], dtype=f32)]))
    tgt = torch.as_tensor(np.concatenate([np.asarray(task["actions"], dtype=f32), np.asarray(pres["actions"], dtype=f32)]))
    clk = torch.as_tensor(np.concatenate([np.asarray(task["clock"], dtype=f32), np.asarray(pres["clock"], dtype=f32)]))
    n_task, n_pres = int(np.asarray(task["obs35"]).shape[0]), int(np.asarray(pres["obs35"]).shape[0])
    is_task = torch.as_tensor(np.concatenate([np.ones(n_task, bool), np.zeros(n_pres, bool)]))
    var_task = np.var(np.asarray(task["actions"], dtype=np.float64), axis=0); var_pres = np.var(np.asarray(pres["actions"], dtype=np.float64), axis=0)
    if np.any(var_task <= 0.0) or np.any(var_pres <= 0.0):
        raise V2Error("degenerate per-joint variance in a role (normalisation undefined)")
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

    role_term = RF.normalised_role_term
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
            print(json.dumps(history[-1]), flush=True)
    w3 = np.concatenate([W3m.detach().numpy().astype(f32), logstd_w], axis=0); b3 = np.concatenate([b3m.detach().numpy().astype(f32), logstd_b], axis=0)
    canon = {"pi.0.0.weight": W1.detach().numpy().astype(f32).copy(), "pi.0.0.bias": b1.detach().numpy().astype(f32).copy(), "pi.0.2.weight": W2.detach().numpy().astype(f32).copy(), "pi.0.2.bias": b2.detach().numpy().astype(f32).copy(), "pi.1.weight": w3.astype(f32).copy(), "pi.1.bias": b3.astype(f32).copy()}
    new_state: dict[str, Any] = {}
    for key in RF.EXPECTED_KEY_ORDER:
        new_state[key] = canon[key].copy() if key in canon else canon[RF.ALIAS_KEYS[key]].copy()
    struct = RF.validate_init_state(new_state, expected_actor_digest=None)
    report = {
        "tool": "v26b_v2.fit_r0 (faithful mirror of the immutable f2r_refit.fit_student_preserving; bit-equivalence proven by test_v26b_v2)",
        "variant": "V2_R0_static", "beta": beta, "budget": budget, "optimizer_steps": int(steps), "epochs_run": epochs,
        "rows": {"task": n_task, "preservation": n_pres}, "contract": contract,
        "normalisation": {"var_task_per_joint": var_task.tolist(), "var_pres_per_joint": var_pres.tolist(), "rule": "per-joint variance of the role targets, fixed a priori; per-role per-joint means (group-balanced, not row-count weighted)"},
        "loss_form": "L = task_norm(u_IK rows) + beta * pres_norm(V26-mean anchors) + lambda_clip*clip + lambda_phi*aux(prescribed clock, training-time head) + lambda_anchor*||theta - theta_V1||^2; beta = 1.0 frozen makes the role/beta assignment numerically identical to the rev3 written form (anchor term + beta * teacher term)",
        "selection": "closed_loop_only", "validation_split_in_fit": False,
        "init_actor_digest": info["actor_digest"], "new_actor_digest": RF.actor_state_digest(new_state), "history": history,
        "aux_head": {"exported": False, "training_time_only": True, "param_digest": RF._sha_array(Wh.detach().numpy()) + ":" + RF._sha_array(bh.detach().numpy())},
        "structure": struct,
    }
    report["criteria"] = evaluate_q_criteria(init_state, new_state, task, pres)
    return new_state, report


# --- export ---------------------------------------------------------------------------------------

def export_r0(new_state: Mapping[str, Any], *, fit_report: Mapping[str, Any], dataset_receipt: Mapping[str, Any], lineage: Mapping[str, Any], out_dir: Path = OUT_R0, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    assert_q_r0(fit_report)
    v1_pre = VS.source_files_table(V1_MODULE)
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
        shutil.copy2(V1_MODULE / "metadata.json", stage_module / "metadata.json")
        shutil.copy2(V1_MODULE / "class_and_ctor_args.pkl", stage_module / "class_and_ctor_args.pkl")  # identical 35D serialisation
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in new_state.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        if str(R.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in new_state.items()}, reloaded)
        if not cmp.get("exact"):
            raise V2Error(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=fit_report["new_actor_digest"])
        inv = RF.invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise V2Error("clock invariance not bit-identical")
        names35, _, manifest_shas = VS.pinned_names()
        manifest = {"schema_version": 1, "actor_feature_names": names35, "actor_feature_count": len(names35), "actor_digest": struct["actor_digest"], "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"), "manifest35_sha256": manifest_shas["manifest35_sha256"], "exploration_sigma": [VS.SIGMA_PLACEHOLDER] * R.ACTION_DIM, "sigma_note": VS.SIGMA_STATEMENT, "derived_from": C.rel(V1_MODULE), "source_actor_digest": PIN_V1_ACTOR_DIGEST, "contract": "deployable_markov_controller_state"}
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        if VS.source_files_table(V1_MODULE) != v1_pre or VS.source_files_table(Path(R.TEACHER["module"])) != v26_pre:
            raise V2Error("SOURCE (V1 or V26) changed during the export")
        hist = fit_report["history"]
        receipt = {
            "schema": "v26b_v2_r0.1",
            "protocol_parents_immutable": {"v26b_protocol.json": PIN_PROTOCOL_JSON, "PROTOCOL_V26B.md": PIN_PROTOCOL_MD},
            "amendment_rev3a": {"path": C.rel(AMENDMENT_FILE), "sha256": PIN_AMENDMENT},
            "authorized_stage": AUTHORIZED_STAGE_R0,
            "lineage": dict(lineage),
            "init": {"module": C.rel(V1_MODULE), "actor_digest": PIN_V1_ACTOR_DIGEST, "module_state_sha256": PIN_V1_MODULE_STATE, "v1_receipt_sha256": PIN_V1_RECEIPT, "files_sha256": v1_pre},
            "v26_source_untouched": v26_pre,
            "dataset": dict(dataset_receipt),
            "fit": {k: fit_report[k] for k in ("tool", "variant", "beta", "budget", "optimizer_steps", "epochs_run", "rows", "contract", "normalisation", "loss_form", "init_actor_digest", "new_actor_digest", "aux_head")},
            "loss_history_first_last": {"first": hist[0], "epoch_150": hist[min(149, len(hist) - 1)], "last": hist[-1], "epochs": len(hist)},
            "loss_history_full": hist,
            "criteria": fit_report["criteria"],
            "structure": struct,
            "save_reload_exact": True,
            "clock_invariance": inv,
            "sigma_placeholder": {"value": VS.SIGMA_PLACEHOLDER, "statement": VS.SIGMA_STATEMENT},
            "output_module": C.rel(out_dir / "rl_module"),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "code_digests": {"v26b_v2.py": C.sha256_file(Path(__file__).resolve()), "v26b_student.py": C.sha256_file(HERE / "v26b_student.py"), "v26b_anchors.py": C.sha256_file(HERE / "v26b_anchors.py"), "f2r_refit.py": C.sha256_file(F2R_DIR / "f2r_refit.py"), "f2r_labeller.py": C.sha256_file(F2R_DIR / "f2r_labeller.py"), "warm_start.py": C.sha256_file(R.BASELINE_DIR / "warm_start.py")},
            "scope": "R0 static offline adaptation only: no rollout, no V3, no PPO, no critic, no production change",
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback", "completion_marker": RECEIPT_NAME},
            "generated_at_utc": C.utc_now(),
            "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise V2Error("canonical receipt on disk differs from the receipt in memory")
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

def load_v1_init() -> dict[str, np.ndarray]:
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    state = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(V1_MODULE).items()}
    RF.validate_init_state(state, expected_actor_digest=PIN_V1_ACTOR_DIGEST)
    return state


def run_r0(*, authorized_stage: str | None, out_dir: Path = OUT_R0, progress: bool = True, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE_R0:
        raise V2Error(f"R0 requires --authorized-stage {AUTHORIZED_STAGE_R0} (architect GO); got {authorized_stage!r}")
    lineage = verify_lineage()
    task, pres, ds_report = build_r0_dataset()
    ds_receipt = save_r0_dataset(task, pres, ds_report)
    init_state = load_v1_init()
    new_state, fit_report = fit_r0(init_state, task, pres, progress=progress)
    assert_q_r0(fit_report)  # blocks export on Q1/Q2/Q3 (rev3a: Q2b not applicable at R0)
    return export_r0(new_state, fit_report=fit_report, dataset_receipt=ds_receipt, lineage=lineage, out_dir=out_dir, runtime_status=runtime_status)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B V2 R0 (static offline adaptation; dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        lineage = verify_lineage()
        task, pres, ds_report = build_r0_dataset()
        print(json.dumps({"mode": "dry (no fit, no writes)", "lineage_ok": True, "rows": {"task": int(task["obs35"].shape[0]), "pres": int(pres["obs35"].shape[0])}, "cross_role_collisions": ds_report["cross_role_collisions"]["count"]}, indent=2))
        return 0
    runtime: dict[str, Any] = {}
    canonical = run_r0(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "promotion": runtime.get("promotion"), "new_actor_digest": canonical["fit"]["new_actor_digest"], "criteria_pass_r0": canonical["criteria"]["pass_r0"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
