"""V26B V1 — init transplant 39D -> 35D (preregistered, architect GO 2026-08-24).

Creates the NEW pure deployable 35D student from the August V26 39D imitation
actor ONLY (actor digest ``5bbc6cbd…``, ``module_state.pkl`` ``0ba56eb7…``), per
protocol ``V26B-bridge-rev3`` stage ``V1_init_transplant_39_to_35``:

* feature-NAME aligned weight transfer via the pinned 35D/39D manifests
  (order-preserving bijection on the 35 shared names);
* the 4 healthy-target columns (39D indices 2:6) removed WITH the preregistered
  mean-bias compensation ``b1 += W1_39[:, 2:6] @ mean(targets)`` where the mean
  is computed over the 1500 rows of the 3 pinned deterministic anchor traces
  (float32 values as consumed by the actor, float64 accumulation);
* prescribed-clock columns (35D indices 0:2) ZEROED (deployable rule, 13/07);
* hidden layers and deterministic mean head copied bit-identically;
* state-dependent log-std head REPLACED by the constant serialisation
  placeholder sigma = 0.005 (rows zero, bias float32(ln 0.005)).  This value is
  required by the module format and is NOT the operational sigma: no gate reads
  it and the operational sigma is decided exclusively by the V4 sweep.

Actor-only: no critic, no fit, no dataset build, no env stepping, no rollout,
no production env/reward/FSM/morphology/SEA/C++ change.  F0/F1/F2R libraries
are reused READ-ONLY (export transaction, structural battery, numpy forward).
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
VALIDATION_DIR = HERE.parent
F2R_DIR = VALIDATION_DIR / "f2r_bridge_2026-08-23"
F1_DIR = VALIDATION_DIR / "f1_ablation_2026-08-23"
F0_DIR = VALIDATION_DIR / "f0_freeze_2026-08-22"
for _entry in (str(F0_DIR), str(F1_DIR), str(F2R_DIR), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402
import v26b_anchors as VA  # noqa: E402


class V1Error(RuntimeError):
    """Fail-closed violation in the V1 transplant."""


AUTHORIZED_STAGE_V1 = "V26B-V1"
OUT_STUDENT = VA.OUT_ROOT / "student" / "V1_35D_transplant"
MODULE_DIR_NAME = "rl_module"
RECEIPT_NAME = "v26b_v1_receipt.json"
SIGMA_PLACEHOLDER = 0.005
SIGMA_STATEMENT = ("sigma 0.005 is a SERIALISATION PLACEHOLDER required by the module format: "
                   "log-std rows zero, bias float32(ln 0.005). It is NOT the operational sigma "
                   "selection; no deterministic gate reads it; the operational sigma is decided "
                   "exclusively by the preregistered V4 sweep.")
COVERAGE_JSON = VA.OUT_ROOT / "coverage" / "anchor_coverage_20260824_140923.json"
COVERAGE_SHA256 = "5c8123607bba3bf6339ad220163f95966a0f9cf4497854ed32aac863e1e9218e"
CRITIC_EXTRAS = 49  # full observation = n_actor + 49 (V26: 88 = 39 + 49; deployable: 84 = 35 + 49)
TARGET_SLICE = (2, 6)  # healthy-target block in the 39D layout


# --- manifests ------------------------------------------------------------------------------------

def pinned_names() -> tuple[list[str], list[str], dict[str, str]]:
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    return list(names35), list(names39), {"manifest35_sha256": sha35, "manifest39_sha256": sha39}


# --- anchor target means (preregistered compensation input) ---------------------------------------

def anchor_target_means() -> dict[str, Any]:
    """Per-column mean of the 4 healthy targets over the 3 pinned det anchor traces.

    Values as CONSUMED by the actor: float32 cast of the recorded privileged
    targets, accumulated in float64.  Fail-closed on pins and row counts."""
    pins_report = R.verify_anchor_pins()
    if not pins_report.get("all_match"):
        raise V1Error("pinned det anchors do not match their content digests")
    per_trace: dict[str, Any] = {}
    blocks: list[np.ndarray] = []
    for start, spec in R.ANCHORS.items():
        cache = L.PrivilegedCache.from_adapter_sidecar(Path(spec["job_dir"]), pins=spec)
        if cache.rows != 500:
            raise V1Error(f"anchor {start}: {cache.rows} rows != 500")
        t32 = cache.targets.astype(np.float32)
        if not np.all(np.isfinite(t32)):
            raise V1Error(f"anchor {start}: non-finite targets")
        blocks.append(t32.astype(np.float64))
        per_trace[start] = {
            "rows": int(cache.rows),
            "mean_float64": np.mean(t32.astype(np.float64), axis=0).tolist(),
            "adapter_trace_sha256": spec["adapter_trace_sha256"],
            "job_dir": C.rel(Path(spec["job_dir"])),
        }
    pooled = np.concatenate(blocks, axis=0)
    m64 = np.mean(pooled, axis=0)
    return {
        "target_names": list(OA.TARGET_FEATURE_NAMES),
        "rows_total": int(pooled.shape[0]),
        "per_trace": per_trace,
        "pooled_mean_float64": m64.tolist(),
        "pooled_mean_float32": m64.astype(np.float32).astype(float).tolist(),
        "value_dtype": "float32 targets as consumed by the actor (float64 accumulation)",
    }


# --- transplant -----------------------------------------------------------------------------------

def transplant_39_to_35(source_state: Mapping[str, Any], names39: Sequence[str], names35: Sequence[str], mean_targets: Sequence[float]) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Pure feature-name-aligned transplant (preregistered V1 recipe); returns (state35, report)."""
    keys = tuple(source_state.keys())
    if keys != RF.EXPECTED_KEY_ORDER:
        raise V1Error(f"source keys/order {keys} != the 10 pi* keys")
    for alias, canon in RF.ALIAS_KEYS.items():
        if not np.array_equal(np.asarray(source_state[alias]), np.asarray(source_state[canon])):
            raise V1Error(f"source alias {alias} not bit-identical to {canon}")
    src = {k: np.asarray(source_state[k]) for k in keys}
    if any(v.dtype != np.float32 for v in src.values()):
        raise V1Error("source tensors must be float32")
    names39 = [str(n) for n in names39]
    names35 = [str(n) for n in names35]
    if len(names39) != 39 or len(names35) != 35 or len(set(names39)) != 39 or len(set(names35)) != 35:
        raise V1Error("manifest name lists must be 39/35 unique names")
    lo, hi = TARGET_SLICE
    removed = names39[lo:hi]
    if removed != list(OA.TARGET_FEATURE_NAMES):
        raise V1Error(f"39D indices {lo}:{hi} are {removed}, expected the 4 healthy targets {list(OA.TARGET_FEATURE_NAMES)}")
    if [n for n in names39 if n not in removed] != names35:
        raise V1Error("35D names are not the 39D names minus the 4 targets in the same order (bijection broken)")
    if names35[:2] != list(OA.CLOCK_FEATURE_NAMES) or names39[:2] != list(OA.CLOCK_FEATURE_NAMES):
        raise V1Error("prescribed clock pair must be the first two features of both manifests")
    keep_idx = [names39.index(n) for n in names35]
    if keep_idx != sorted(keep_idx):
        raise V1Error("kept-column indices are not strictly increasing")
    W1s, b1s = src["pi.0.0.weight"], src["pi.0.0.bias"]
    if W1s.shape[1] != 39 or b1s.shape != (W1s.shape[0],):
        raise V1Error(f"source first layer must be (H, 39)/(H,), got {W1s.shape}/{b1s.shape}")
    W3s, b3s = src["pi.1.weight"], src["pi.1.bias"]
    if W3s.shape[0] != 4 or b3s.shape != (4,):
        raise V1Error(f"source head must be (4, H)/(4,), got {W3s.shape}/{b3s.shape}")
    m64 = np.asarray(mean_targets, dtype=np.float64)
    if m64.shape != (hi - lo,) or not np.all(np.isfinite(m64)):
        raise V1Error(f"mean_targets must be {hi - lo} finite values")
    # 1) name-aligned column transfer (bit-identical for every kept column)
    W1n = np.ascontiguousarray(W1s[:, keep_idx])
    # 2) mean-bias compensation for the removed target columns (float64, cast once)
    Wt64 = W1s[:, lo:hi].astype(np.float64)
    delta64 = Wt64 @ m64
    b1n = (b1s.astype(np.float64) + delta64).astype(np.float32)
    # 3) prescribed-clock columns zeroed (deployable rule)
    W1n[:, 0:2] = np.float32(0.0)
    # 4) hidden layer and mean head copied bit-identically; 5) constant log-std placeholder
    W3n = W3s.copy(); W3n[2:4, :] = np.float32(0.0)
    b3n = b3s.copy(); b3n[2:4] = np.float32(np.log(SIGMA_PLACEHOLDER))
    def pair(a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.ascontiguousarray(a.copy()), np.ascontiguousarray(a.copy())
    e0w, p0w = pair(W1n); e0b, p0b = pair(b1n)
    e2w, p2w = pair(src["pi.0.2.weight"]); e2b, p2b = pair(src["pi.0.2.bias"])
    new_state: dict[str, np.ndarray] = {
        "pi_encoder.0.weight": e0w, "pi_encoder.0.bias": e0b,
        "pi_encoder.2.weight": e2w, "pi_encoder.2.bias": e2b,
        "pi.0.0.weight": p0w, "pi.0.0.bias": p0b,
        "pi.0.2.weight": p2w, "pi.0.2.bias": p2b,
        "pi.1.weight": np.ascontiguousarray(W3n), "pi.1.bias": np.ascontiguousarray(b3n),
    }
    if tuple(new_state.keys()) != RF.EXPECTED_KEY_ORDER:
        raise V1Error("output key order broken")
    # fail-closed self-checks
    if not np.array_equal(new_state["pi.0.2.weight"], src["pi.0.2.weight"]) or not np.array_equal(new_state["pi.0.2.bias"], src["pi.0.2.bias"]):
        raise V1Error("hidden layer changed by the transplant")
    if not np.array_equal(new_state["pi.1.weight"][:2], W3s[:2]) or not np.array_equal(new_state["pi.1.bias"][:2], b3s[:2]):
        raise V1Error("mean head changed by the transplant")
    if not np.all(new_state["pi.0.0.weight"][:, 0:2] == 0.0):
        raise V1Error("clock columns not zero after the transplant")
    for j35, j39 in enumerate(keep_idx):
        if j35 >= 2 and not np.array_equal(new_state["pi.0.0.weight"][:, j35], W1s[:, j39]):
            raise V1Error(f"kept column {names35[j35]!r} not bit-identical")
    if any(not np.all(np.isfinite(v)) for v in new_state.values()):
        raise V1Error("non-finite values in the transplanted state")
    report = {
        "recipe": "V26B-bridge-rev3 / V1_init_transplant_39_to_35 (July 25/06 design + 09/07 implementation + 13/07 deployable rule)",
        "feature_mapping": {"kept_39d_indices": keep_idx, "removed_39d_indices": list(range(lo, hi)), "removed_names": removed, "bijection": "names35 == names39 minus the 4 targets, order preserved (verified)"},
        "mean_bias_compensation": {
            "formula": "b1_35 = float32(float64(b1_39) + float64(W1_39[:, 2:6]) @ mean_targets)",
            "mean_targets_float64": m64.tolist(),
            "delta_float64": {"norm_l2": float(np.linalg.norm(delta64)), "min": float(delta64.min()), "max": float(delta64.max()), "sha256_float64_bytes": DS.sha256_array(np.ascontiguousarray(delta64))},
            "bias_after_sha256": DS.sha256_array(new_state["pi.0.0.bias"]),
            "bias_max_abs_change_float32": float(np.max(np.abs(new_state["pi.0.0.bias"].astype(np.float64) - b1s.astype(np.float64)))),
        },
        "clock_columns_zeroed": {"indices_35d": [0, 1], "names": names35[:2], "note": "no clock compensation (preregistered: zeroing only; the deployed env emits dead constants)"},
        "logstd_replaced": {
            "source_rows_l2": float(np.linalg.norm(W3s[2:4].astype(np.float64))),
            "source_bias": b3s[2:4].astype(float).tolist(),
            "constant_bias_float32": float(np.float32(np.log(SIGMA_PLACEHOLDER))),
            "statement": SIGMA_STATEMENT,
        },
        "source_actor_digest": RF.actor_state_digest(source_state),
        "new_actor_digest": RF.actor_state_digest(new_state),
    }
    return new_state, report


# --- ctor args (module serialisation metadata, 39 -> 35) ------------------------------------------

def build_ctor_35(source_module: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    with open(Path(source_module) / "class_and_ctor_args.pkl", "rb") as fh:
        cc = pickle.load(fh)
    args, kwargs = cc["ctor_args_and_kwargs"]
    if args != ():
        raise V1Error(f"unexpected positional ctor args: {args!r}")
    import gymnasium as gym
    obs = kwargs["observation_space"]
    if tuple(obs.shape) != (39 + CRITIC_EXTRAS,):
        raise V1Error(f"source observation space {obs.shape} != ({39 + CRITIC_EXTRAS},)")
    mc = dict(kwargs["model_config"])
    if mc.get("n_actor") != 39:
        raise V1Error(f"source n_actor {mc.get('n_actor')} != 39")
    new_obs = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(35 + CRITIC_EXTRAS,), dtype=np.float32)
    new_kwargs = dict(kwargs)
    new_kwargs["observation_space"] = new_obs
    new_kwargs["model_config"] = {**mc, "n_actor": 35}
    new_cc = {"class": cc["class"], "ctor_args_and_kwargs": ((), new_kwargs)}
    report = {
        "class": f"{cc['class'].__module__}.{cc['class'].__name__}",
        "observation_space": {"source_dim": int(obs.shape[0]), "new_dim": 35 + CRITIC_EXTRAS, "dtype": "float32", "bounds": "(-inf, inf)"},
        "model_config_new": {k: v for k, v in new_kwargs["model_config"].items()},
        "action_space_unchanged": True,
        "note": "derived by editing the V26 ctor only (n_actor 39->35, observation dim 88->84 = n_actor + 49 critic extras; n_full and freeze flags copied from V26)",
    }
    if repr(new_kwargs["action_space"]) != repr(kwargs["action_space"]):
        raise V1Error("action space changed")
    return new_cc, report


# --- fidelity (informational, NOT a V1 gate) ------------------------------------------------------

def fidelity_vs_v26(new_state: Mapping[str, Any]) -> dict[str, Any]:
    """Transplanted 35D mean vs the recorded V26 deterministic action on the 3 pinned anchors."""
    per_start: dict[str, Any] = {}
    diffs_all: list[np.ndarray] = []
    for start, spec in R.ANCHORS.items():
        traj = DS.trajectory_from_job(Path(spec["job_dir"]), expected_width=R.ENV_ACTOR_WIDTH)
        obs35 = traj["obs35"].astype(np.float32)
        u35 = np.asarray(RF.numpy_mean(new_state, obs35), dtype=np.float64)
        v26 = np.asarray(traj["b_raw_action"], dtype=np.float64)  # det trace: recorded action == V26 mean (verified <= 1e-5 in coverage)
        d = u35 - v26
        diffs_all.append(d)
        per_start[start] = {
            "rows": int(d.shape[0]),
            "per_joint": {joint: {"rmse": float(np.sqrt(np.mean(d[:, j] ** 2))), "mae": float(np.mean(np.abs(d[:, j]))), "max_abs": float(np.max(np.abs(d[:, j])))} for j, joint in enumerate(("knee", "ankle"))},
            "overall_rmse": float(np.sqrt(np.mean(d ** 2))),
        }
    d = np.concatenate(diffs_all, axis=0)
    return {
        "comparison": "numpy float64 mean of the transplanted 35D student on recorded obs35 (dead clock) vs the recorded V26 deterministic action (39D actor with privileged targets + prescribed clock)",
        "expected": "degraded (B0820-like baseline point): informational, NOT a V1 gate; V2 adaptation is the corrective stage",
        "per_start": per_start,
        "overall": {"rows": int(d.shape[0]), "per_joint": {joint: {"rmse": float(np.sqrt(np.mean(d[:, j] ** 2))), "mae": float(np.mean(np.abs(d[:, j]))), "max_abs": float(np.max(np.abs(d[:, j])))} for j, joint in enumerate(("knee", "ankle"))}, "rmse": float(np.sqrt(np.mean(d ** 2)))},
    }


# --- export (transactional, no-clobber, content-addressed) ----------------------------------------

def source_files_table(module: Path) -> dict[str, str]:
    out = {}
    for name in ("module_state.pkl", "class_and_ctor_args.pkl", "metadata.json", "actor_feature_manifest.json"):
        p = Path(module) / name
        if p.is_symlink() or not p.is_file():
            raise V1Error(f"source module file missing (or symlink): {p}")
        out[name] = C.sha256_file(p)
    return out


def export_v1(new_state: Mapping[str, Any], *, reports: Mapping[str, Any], names35: Sequence[str], manifest_shas: Mapping[str, str], out_dir: Path = OUT_STUDENT, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    """Build, verify and atomically promote ``<out_dir>/rl_module`` + receipt (marker last).

    Returns the CANONICAL receipt (the dict parsed from the staged marker bytes,
    immutable after the promotion) — same provenance contract as F2R."""
    source_module = Path(R.TEACHER["module"])
    pre_table = source_files_table(source_module)
    if pre_table["module_state.pkl"] != R.TEACHER["module_state_sha256"]:
        raise V1Error("source module_state.pkl differs from its pin BEFORE export")
    if COVERAGE_JSON.is_symlink() or not COVERAGE_JSON.is_file() or C.sha256_file(COVERAGE_JSON) != COVERAGE_SHA256:
        raise V1Error(f"coverage JSON missing or digest != architect-verified {COVERAGE_SHA256[:16]}…")
    proto = VA.load_protocol()
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
        stage_module = staging / MODULE_DIR_NAME
        stage_module.mkdir(parents=True, exist_ok=False)
        shutil.copy2(source_module / "metadata.json", stage_module / "metadata.json")
        new_cc, ctor_report = build_ctor_35(source_module)
        with (stage_module / "class_and_ctor_args.pkl").open("wb") as fh:
            pickle.dump(new_cc, fh, protocol=pickle.HIGHEST_PROTOCOL)
        with (stage_module / "module_state.pkl").open("wb") as fh:
            pickle.dump({k: np.asarray(v) for k, v in new_state.items()}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        if str(R.BASELINE_DIR) not in sys.path:
            sys.path.insert(0, str(R.BASELINE_DIR))
        import warm_start as W
        reloaded = W.load_module_state(stage_module)
        cmp = W.compare_actor_states({k: np.asarray(v) for k, v in new_state.items()}, reloaded)
        if not cmp.get("exact"):
            raise V1Error(f"save/reload mismatch: {cmp}")
        struct = RF.validate_init_state(reloaded, expected_actor_digest=None)  # 10 keys, aliases, f32, width 35, clock zero, constant-sigma head
        inv = RF.invariance_test(reloaded, np.random.default_rng(1).standard_normal((64, R.ENV_ACTOR_WIDTH)).astype(np.float32))
        if not inv.get("bit_identical"):
            raise V1Error("clock invariance not bit-identical")
        with (stage_module / "class_and_ctor_args.pkl").open("rb") as fh:
            cc_check = pickle.load(fh)
        _, kw = cc_check["ctor_args_and_kwargs"]
        if tuple(kw["observation_space"].shape) != (35 + CRITIC_EXTRAS,) or kw["model_config"].get("n_actor") != 35:
            raise V1Error("staged ctor args do not describe a 35D module")
        manifest = {
            "schema_version": 1,
            "actor_feature_names": list(names35),
            "actor_feature_count": len(names35),
            "actor_digest": struct["actor_digest"],
            "module_state_sha256": C.sha256_file(stage_module / "module_state.pkl"),
            "manifest35_sha256": manifest_shas["manifest35_sha256"],
            "exploration_sigma": [SIGMA_PLACEHOLDER] * R.ACTION_DIM,
            "sigma_note": SIGMA_STATEMENT,
            "derived_from": C.rel(source_module),
            "source_actor_digest": R.TEACHER["actor_digest"],
            "contract": "deployable_markov_controller_state",
        }
        C.write_json(stage_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=False)
        post_table = source_files_table(source_module)
        if post_table != pre_table:
            raise V1Error("SOURCE MODULE CHANGED during the export (immutability violated)")
        receipt = {
            "schema": "v26b_v1.1",
            "protocol": {"id": VA.PROTOCOL_ID, "sha256": C.sha256_file(VA.PROTOCOL_JSON)},
            "authorized_stage": AUTHORIZED_STAGE_V1,
            "source": {"module": C.rel(source_module), "files_sha256": pre_table, "actor_digest": R.TEACHER["actor_digest"], "module_state_sha256": R.TEACHER["module_state_sha256"], "immutability": "files re-hashed after the build: unchanged"},
            "manifests": dict(manifest_shas),
            "coverage_gate": {"path": C.rel(COVERAGE_JSON), "sha256": COVERAGE_SHA256, "role": "anchor coverage PASS verified by the architect before this GO"},
            "code_digests": {
                "v26b_student.py": C.sha256_file(Path(__file__).resolve()),
                "v26b_anchors.py": C.sha256_file(HERE / "v26b_anchors.py"),
                "warm_start.py": C.sha256_file(R.BASELINE_DIR / "warm_start.py"),
                "f2r_refit.py": C.sha256_file(F2R_DIR / "f2r_refit.py"),
                "f2r_labeller.py": C.sha256_file(F2R_DIR / "f2r_labeller.py"),
                "f1_dataset.py": C.sha256_file(F1_DIR / "f1_dataset.py"),
                "f1_obs_adapter.py": C.sha256_file(F1_DIR / "f1_obs_adapter.py"),
            },
            "anchor_target_means": reports["anchor_means"],
            "transplant": reports["transplant"],
            "ctor": ctor_report,
            "fidelity_vs_v26_informational": reports["fidelity"],
            "structure": struct,
            "save_reload_exact": True,
            "clock_invariance": inv,
            "sigma_placeholder": {"value": SIGMA_PLACEHOLDER, "statement": SIGMA_STATEMENT},
            "output_module": C.rel(out_dir / MODULE_DIR_NAME),
            "output_files_sha256": {p.name: C.sha256_file(p) for p in sorted(stage_module.iterdir())},
            "export_transaction": {"lock": lock.name, "promotion_policy": "native atomic no-replace rename (Darwin renameatx_np RENAME_EXCL), else F2R exclusive-mkdir fallback; receipt written in staging before the commit", "completion_marker": RECEIPT_NAME},
            "scope": "actor-only: no critic, no fit, no dataset, no env stepping, no rollout, no production change",
            "generated_at_utc": C.utc_now(),
            "git": C.git_snapshot(),
        }
        C.write_json(staging / RECEIPT_NAME, receipt)
        canonical = json.loads((staging / RECEIPT_NAME).read_text(encoding="utf-8"))
        if canonical != json.loads(json.dumps(receipt, default=str)):
            raise V1Error("canonical receipt on disk differs from the receipt in memory")
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

def build_v1_state() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """In-memory V1 build: load pinned V26, compute means, transplant, fidelity. No writes."""
    ctx = VA.teacher_context()  # verifies module_state + actor digest pins
    names35, names39, manifest_shas = pinned_names()
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    source_state = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(Path(R.TEACHER["module"])).items()}
    means = anchor_target_means()
    new_state, t_report = transplant_39_to_35(source_state, names39, names35, means["pooled_mean_float64"])
    if t_report["source_actor_digest"] != R.TEACHER["actor_digest"]:
        raise V1Error("source digest mismatch after load")
    fid = fidelity_vs_v26(new_state)
    return new_state, {"anchor_means": means, "transplant": t_report, "fidelity": fid, "names35": names35, "manifest_shas": manifest_shas}


def run_v1(*, authorized_stage: str | None, out_dir: Path = OUT_STUDENT, runtime_status: dict[str, Any] | None = None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE_V1:
        raise V1Error(f"V1 export requires --authorized-stage {AUTHORIZED_STAGE_V1} (architect GO); got {authorized_stage!r}")
    new_state, reports = build_v1_state()
    return export_v1(new_state, reports=reports, names35=reports["names35"], manifest_shas=reports["manifest_shas"], out_dir=out_dir, runtime_status=runtime_status)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B V1: 39D->35D init transplant (dry by default)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        _, reports = build_v1_state()
        print(json.dumps({"mode": "dry (no writes)", "mean_targets": reports["anchor_means"]["pooled_mean_float64"], "new_actor_digest": reports["transplant"]["new_actor_digest"], "fidelity_overall": reports["fidelity"]["overall"]}, indent=2))
        return 0
    runtime: dict[str, Any] = {}
    canonical = run_v1(authorized_stage=args.authorized_stage, runtime_status=runtime)
    print(json.dumps({"receipt_sha256": runtime.get("canonical_receipt_sha256"), "final_path": runtime.get("final_path"), "promotion": runtime.get("promotion"), "new_actor_digest": canonical["transplant"]["new_actor_digest"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
