"""V26B stage A0 - init transplant 39D -> 25D INTERMEDIATE contract.

Parent: EXCLUSIVELY the August V26 imitation actor (39D), digest 5bbc6cbd...
S0D, S1A and every REV4* artifact are DIAGNOSTIC EVIDENCE ONLY and are never
init, anchor or data of this branch.  No July checkpoint, dataset, label or anchor.

Recipe (mirrors the frozen August V1 recipe, extended from 4 to 14 removed columns):
  1. name-aligned column transfer, bit-identical for every kept column;
  2. mean-bias compensation for the 14 removed columns, accumulated in float64;
  3. the two prescribed-clock columns zeroed;
  4. hidden layer and mean head copied bit-identically;
  5. log-std head replaced by the SERIALISATION PLACEHOLDER - sigma is NOT decided here.

The 14 removed columns are the 4 healthy imitation targets (39D indices 2..5, privileged
and anti-ex-novo) and the 10 controller-state Markov features (39D indices 29..38), which
July 13 diagnosed as an autoregressive proxy of the prescribed action.

Nothing in production is touched.  Cross-platform: pathlib only, no shell, no os-specific path.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_student as VS  # noqa: E402   (frozen V1 recipe: pinned names, ctor helpers)
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_labeller as L  # noqa: E402
import f2r_refit as RF  # noqa: E402


class A0Error(RuntimeError):
    pass


STAGE = "V26B-A0-TRANSPLANT-25D"

# --- pinned parent (August V26 only) ---------------------------------------------------------
V26_MODULE = (R.BASELINE_DIR.parent / "runs" / "training"
              / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
PIN_V26_ACTOR_DIGEST = "5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e"
CONTRACT_25D = HERE / "v26b_contract_25d_v1.yaml"
PIN_CONTRACT_25D = "b7292a73a9d342dd61773ab5bc85d66b1532bc2a0d76fd4d1dad37758fa67fae"
PIN_PARENT_CONFIG = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"

# artifacts that may NEVER be init, anchor or data of this branch
FORBIDDEN_SOURCE_MARKERS = ("S0D_35D_DISTILLED", "S1A_", "S1B_", "S1C1_", "S1C2Z_",
                            "REV4B", "REV4C", "REV4D", "REV4E", "V2_", "2026-07",
                            "target_domain_dagger_", "target_domain_imitation_2026")

CONTROLLER_SUFFIXES = ("_previous_endpoint", "_served_ref", "_served_ref_vel",
                       "_served_ref_accel", "_sea_u")
N_ACTOR_25 = 25
FULL_OBS_DIM = 84            # 25 actor + 59 critic; identical total to the 35D contract
CLOCK_COLUMNS = (0, 1)
SIGMA_PLACEHOLDER = VS.SIGMA_PLACEHOLDER
SIGMA_STATEMENT = ("sigma is a SERIALISATION PLACEHOLDER here and is NOT a decision: the "
                   "exploration scale of the A1 actor must be MEASURED on A1 before any "
                   "stochastic recovery is collected (open TODO, architect decision)")

OUT_DIR = VA.OUT_ROOT / "student" / "A0_25D_TRANSPLANT"
RECEIPT_NAME = "v26b_a0_receipt.json"


def _is_controller(name: str) -> bool:
    return any(name.endswith(s) for s in CONTROLLER_SUFFIXES)


def contract_25d() -> dict[str, Any]:
    """The intermediate 25D actor contract, derived fail-closed from the pinned manifests."""
    names35, names39, shas = VS.pinned_names()
    if len(names35) != 35 or len(names39) != 39:
        raise A0Error(f"pinned manifests are {len(names35)}/{len(names39)}, expected 35/39")
    ctrl = [n for n in names35 if _is_controller(n)]
    if len(ctrl) != 10:
        raise A0Error(f"expected 10 controller-state features, found {len(ctrl)}: {ctrl}")
    names25 = [n for n in names35 if not _is_controller(n)]
    if len(names25) != N_ACTOR_25:
        raise A0Error(f"25D contract has {len(names25)} names")
    if [n for n in names39 if n in names25] != names25:
        raise A0Error("25D names are not a subsequence of the 39D names (order broken)")
    keep = [names39.index(n) for n in names25]
    if keep != sorted(keep):
        raise A0Error("kept indices are not strictly increasing")
    removed = [i for i in range(39) if i not in keep]
    targets = [names39[i] for i in removed if not _is_controller(names39[i])]
    if len(removed) != 14 or len(targets) != 4:
        raise A0Error(f"removal set is {len(removed)} columns with {len(targets)} targets, expected 14/4")
    if names25[:2] != list(names39[:2]):
        raise A0Error("the prescribed clock pair must open both contracts")
    got = C.sha256_file(CONTRACT_25D)
    if got != PIN_CONTRACT_25D:
        raise A0Error(f"25D contract yaml sha {got} != pinned")
    text = CONTRACT_25D.read_text(encoding="utf-8")
    if "include_controller_state_observation: false" not in text:
        raise A0Error("the 25D contract does not disable the controller-state observation")
    if f"Parent config sha256: {PIN_PARENT_CONFIG}" not in text:
        raise A0Error("the 25D contract does not declare the pinned parent config")
    return {"names25": names25, "names39": names39, "names35": names35,
            "controller_features_removed": ctrl, "target_features_removed": targets,
            "kept_39d_indices": keep, "removed_39d_indices": removed,
            "manifest_shas": dict(shas), "contract_yaml_sha256": got,
            "parent_config_sha256": PIN_PARENT_CONFIG,
            "full_observation_dim": FULL_OBS_DIM,
            "note": "the 10 controller-state columns MOVE to the privileged critic suffix; "
                    "the 4 healthy targets are REMOVED outright (privileged and anti-ex-novo)"}


def removal_means() -> dict[str, Any]:
    """float64 means of the 14 removed columns over the 3 pinned deterministic anchors.

    Healthy targets come from the privileged cache (as the frozen V1 recipe does); the ten
    controller-state columns come from the obs35 of the SAME anchor traces, same rows."""
    rep = R.verify_anchor_pins()
    if not rep.get("all_match"):
        raise A0Error("pinned deterministic anchors do not match their content digests")
    obs_blocks: list[np.ndarray] = []
    tgt_blocks: list[np.ndarray] = []
    per_trace: dict[str, Any] = {}
    for start, spec in R.ANCHORS.items():
        job = Path(spec["job_dir"])
        assert_not_forbidden(job, "anchor job")
        traj = DS.trajectory_from_job(job, expected_width=R.ENV_ACTOR_WIDTH)
        obs = np.asarray(traj["obs35"], dtype=np.float32)
        cache = L.PrivilegedCache.from_adapter_sidecar(job, pins=spec)
        if obs.shape[0] != 500 or cache.rows != 500:
            raise A0Error(f"anchor {start}: {obs.shape[0]}/{cache.rows} rows, expected 500/500")
        tgt = np.asarray(cache.targets, dtype=np.float32)
        if not np.all(np.isfinite(obs)) or not np.all(np.isfinite(tgt)):
            raise A0Error(f"anchor {start}: non-finite anchor data")
        obs_blocks.append(obs.astype(np.float64))
        tgt_blocks.append(tgt.astype(np.float64))
        per_trace[start] = {"rows": 500, "job_dir": C.rel(job),
                            "adapter_trace_sha256": spec["adapter_trace_sha256"]}
    ct = contract_25d()
    ctrl_idx35 = [ct["names35"].index(n) for n in ct["controller_features_removed"]]
    O = np.concatenate(obs_blocks, axis=0)
    T = np.concatenate(tgt_blocks, axis=0)
    mean_targets = T.mean(axis=0)
    mean_ctrl = O[:, ctrl_idx35].mean(axis=0)
    ordered: list[float] = []
    for i in ct["removed_39d_indices"]:
        name = ct["names39"][i]
        if _is_controller(name):
            ordered.append(float(mean_ctrl[ct["controller_features_removed"].index(name)]))
        else:
            ordered.append(float(mean_targets[ct["target_features_removed"].index(name)]))
    return {"rows_total": int(O.shape[0]), "per_trace": per_trace,
            "removed_names_in_39d_order": [ct["names39"][i] for i in ct["removed_39d_indices"]],
            "mean_float64_in_39d_order": ordered,
            "sources": {"healthy_targets": "privileged cache targets of the pinned anchors "
                                           "(identical source to the frozen V1 recipe)",
                        "controller_state": "obs35 columns of the SAME anchor traces, same rows"}}


def assert_not_forbidden(path: Path | str, where: str) -> None:
    s = str(path)
    for mark in FORBIDDEN_SOURCE_MARKERS:
        if mark in s:
            raise A0Error(f"{where}: {s} uses a forbidden source ({mark!r}); this branch takes "
                          "only the August V26 parent")


def transplant_39_to_25(source_state: Mapping[str, Any], ct: Mapping[str, Any],
                        mean_removed: Sequence[float]) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    keys = tuple(source_state.keys())
    if keys != RF.EXPECTED_KEY_ORDER:
        raise A0Error(f"source keys/order {keys} != the 10 pi* keys")
    for alias, canon in RF.ALIAS_KEYS.items():
        if not np.array_equal(np.asarray(source_state[alias]), np.asarray(source_state[canon])):
            raise A0Error(f"source alias {alias} not bit-identical to {canon}")
    src = {k: np.asarray(source_state[k]) for k in keys}
    if any(v.dtype != np.float32 for v in src.values()):
        raise A0Error("source tensors must be float32")
    W1s, b1s = src["pi.0.0.weight"], src["pi.0.0.bias"]
    if W1s.shape[1] != 39 or b1s.shape != (W1s.shape[0],):
        raise A0Error(f"source first layer must be (H, 39)/(H,), got {W1s.shape}/{b1s.shape}")
    W3s, b3s = src["pi.1.weight"], src["pi.1.bias"]
    if W3s.shape[0] != 4 or b3s.shape != (4,):
        raise A0Error(f"source head must be (4, H)/(4,), got {W3s.shape}/{b3s.shape}")
    keep, removed = list(ct["kept_39d_indices"]), list(ct["removed_39d_indices"])
    m64 = np.asarray(mean_removed, dtype=np.float64)
    if m64.shape != (len(removed),) or not np.all(np.isfinite(m64)):
        raise A0Error(f"mean_removed must be {len(removed)} finite values")
    W1n = np.ascontiguousarray(W1s[:, keep])
    delta64 = W1s[:, removed].astype(np.float64) @ m64
    b1n = (b1s.astype(np.float64) + delta64).astype(np.float32)
    W1n[:, list(CLOCK_COLUMNS)] = np.float32(0.0)
    W3n = W3s.copy(); W3n[2:4, :] = np.float32(0.0)
    b3n = b3s.copy(); b3n[2:4] = np.float32(np.log(SIGMA_PLACEHOLDER))

    def pair(a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.ascontiguousarray(a.copy()), np.ascontiguousarray(a.copy())
    e0w, p0w = pair(W1n); e0b, p0b = pair(b1n)
    e2w, p2w = pair(src["pi.0.2.weight"]); e2b, p2b = pair(src["pi.0.2.bias"])
    new_state = {"pi_encoder.0.weight": e0w, "pi_encoder.0.bias": e0b,
                 "pi_encoder.2.weight": e2w, "pi_encoder.2.bias": e2b,
                 "pi.0.0.weight": p0w, "pi.0.0.bias": p0b,
                 "pi.0.2.weight": p2w, "pi.0.2.bias": p2b,
                 "pi.1.weight": np.ascontiguousarray(W3n), "pi.1.bias": np.ascontiguousarray(b3n)}
    if tuple(new_state.keys()) != RF.EXPECTED_KEY_ORDER:
        raise A0Error("output key order broken")
    if new_state["pi.0.0.weight"].shape[1] != N_ACTOR_25:
        raise A0Error("first layer is not 25 columns wide")
    if not np.array_equal(new_state["pi.0.2.weight"], src["pi.0.2.weight"]) or \
       not np.array_equal(new_state["pi.0.2.bias"], src["pi.0.2.bias"]):
        raise A0Error("hidden layer changed by the transplant")
    if not np.array_equal(new_state["pi.1.weight"][:2], W3s[:2]) or \
       not np.array_equal(new_state["pi.1.bias"][:2], b3s[:2]):
        raise A0Error("mean head changed by the transplant")
    if not np.all(new_state["pi.0.0.weight"][:, list(CLOCK_COLUMNS)] == 0.0):
        raise A0Error("clock columns not zero after the transplant")
    for j25, j39 in enumerate(keep):
        if j25 >= 2 and not np.array_equal(new_state["pi.0.0.weight"][:, j25], W1s[:, j39]):
            raise A0Error(f"kept column {ct['names25'][j25]!r} not bit-identical")
    if any(not np.all(np.isfinite(v)) for v in new_state.values()):
        raise A0Error("non-finite values in the transplanted state")
    report = {
        "recipe": "V26B / A0_init_transplant_39_to_25 (frozen V1 recipe extended to the 14-column removal)",
        "feature_mapping": {"kept_39d_indices": keep, "removed_39d_indices": removed,
                            "removed_names": list(ct["removed_names_in_39d_order"])
                            if "removed_names_in_39d_order" in ct else
                            [ct["names39"][i] for i in removed]},
        "mean_bias_compensation": {
            "formula": "b1_25 = float32(float64(b1_39) + float64(W1_39[:, removed]) @ mean_removed)",
            "mean_removed_float64": m64.tolist(),
            "delta_float64": {"norm_l2": float(np.linalg.norm(delta64)),
                              "min": float(delta64.min()), "max": float(delta64.max()),
                              "sha256_float64_bytes": DS.sha256_array(np.ascontiguousarray(delta64))},
            "bias_max_abs_change_float32": float(np.max(np.abs(
                new_state["pi.0.0.bias"].astype(np.float64) - b1s.astype(np.float64)))),
        },
        "clock_columns_zeroed": {"indices_25d": list(CLOCK_COLUMNS), "names": ct["names25"][:2]},
        "logstd_placeholder": {"constant_bias_float32": float(np.float32(np.log(SIGMA_PLACEHOLDER))),
                               "statement": SIGMA_STATEMENT},
        "source_actor_digest": RF.actor_state_digest(source_state),
        "new_actor_digest": RF.actor_state_digest(new_state),
    }
    return new_state, report


def build_ctor_25(source_module: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    with open(Path(source_module) / "class_and_ctor_args.pkl", "rb") as fh:
        cc = pickle.load(fh)
    args, kwargs = cc["ctor_args_and_kwargs"]
    if args != ():
        raise A0Error(f"unexpected positional ctor args: {args!r}")
    import gymnasium as gym
    obs = kwargs["observation_space"]
    mc = dict(kwargs["model_config"])
    if mc.get("n_actor") != 39:
        raise A0Error(f"source n_actor {mc.get('n_actor')} != 39")
    new_kwargs = dict(kwargs)
    new_kwargs["observation_space"] = gym.spaces.Box(low=-np.inf, high=np.inf,
                                                     shape=(FULL_OBS_DIM,), dtype=np.float32)
    new_kwargs["model_config"] = {**mc, "n_actor": N_ACTOR_25}
    if repr(new_kwargs["action_space"]) != repr(kwargs["action_space"]):
        raise A0Error("action space changed")
    return ({"class": cc["class"], "ctor_args_and_kwargs": ((), new_kwargs)},
            {"class": f"{cc['class'].__module__}.{cc['class'].__name__}",
             "observation_space": {"source_dim": int(obs.shape[0]), "new_dim": FULL_OBS_DIM},
             "model_config_new": {k: v for k, v in new_kwargs["model_config"].items()},
             "action_space_unchanged": True,
             "note": "n_actor 39->25; the full vector stays 84 because the ten controller-state "
                     "columns move to the critic suffix while the four healthy targets are removed"})


def load_source() -> dict[str, np.ndarray]:
    assert_not_forbidden(V26_MODULE, "A0 parent")
    with open(V26_MODULE / "module_state.pkl", "rb") as fh:
        st = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    d = RF.actor_state_digest(st)
    if d != PIN_V26_ACTOR_DIGEST:
        raise A0Error(f"V26 parent digest {d} != pinned {PIN_V26_ACTOR_DIGEST}")
    return st


def preflight() -> dict[str, Any]:
    """No-write, fail-closed. Everything A0 needs must already exist and match its pin."""
    ct = contract_25d()
    src = load_source()
    means = removal_means()
    ct["removed_names_in_39d_order"] = means["removed_names_in_39d_order"]
    state, report = transplant_39_to_25(src, ct, means["mean_float64_in_39d_order"])
    ctor, ctor_report = build_ctor_25(V26_MODULE)
    val = RF.validate_init_state(state, expected_actor_digest=None, width=N_ACTOR_25)
    if not val["clock_columns_zero"]:
        raise A0Error("clock columns are not zero in the transplanted state")
    if OUT_DIR.exists():
        raise A0Error(f"no-clobber: {OUT_DIR} already exists")
    return {"verdict": "GO", "stage": STAGE, "contract": ct, "removal_means": means,
            "transplant": report, "ctor": ctor_report,
            "structure": {k: v for k, v in val.items() if k != "sigma_head"},
            "sigma_head": val["sigma_head"],
            "parent": {"module": C.rel(V26_MODULE), "actor_digest": PIN_V26_ACTOR_DIGEST,
                       "module_state_sha256": C.sha256_file(V26_MODULE / "module_state.pkl")},
            "_state": state, "_ctor": ctor}


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B A0: 39D -> 25D intermediate transplant")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    a = p.parse_args(argv)
    if a.authorized_stage is not None and a.authorized_stage != STAGE:
        raise A0Error(f"requires --authorized-stage {STAGE}; got {a.authorized_stage!r}")
    pre = preflight()
    print(json.dumps({k: v for k, v in pre.items() if not k.startswith("_")}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
