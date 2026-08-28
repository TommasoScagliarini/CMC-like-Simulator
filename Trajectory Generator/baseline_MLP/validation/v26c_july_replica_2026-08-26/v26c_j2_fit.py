"""V26C J2 - July-faithful supervised BASE FIT on the J1 dataset.

The July BASE stage replayed on the current lineage: the ENTIRE mean actor is fitted on the 500
observation -> prescribed-teacher pairs collected by J1, with a seeded random 80/20 split.
Reconstructed from `target_domain_imitation_no_controller_memory_2026-07-13`, not from prose.

THE STUDENT IS DERIVED FRESH.
    The ONLY actor input is the August V26 39D parent, module_state 0ba56eb7... The 39 -> 35
    mapping is by FEATURE NAME, the four dropped `healthy_*_imitation_target*` columns are folded
    into the first-layer bias by the preregistered mean compensation, the log-std head is replaced
    by the deployable constant, and columns 0,1 and 25..34 are hard-zeroed. V1 and B0 are NEVER
    read by the derivation, by verify_parent or by the fit: they are consulted only afterwards, as
    an OPTIONAL diagnostic oracle.

JULY'S MECHANISM, NOT ONLY ITS RESULT.
    freeze_logstd_head = False with logstd_weight 0.1: the log-std head is anchored by the loss,
    not frozen by construction. ONE np.random.default_rng(123) draws the split permutation FIRST
    and the SAME generator object then continues for every epoch shuffle, exactly as July's
    adapt_actor does.

Per-step order, identical to July: zero_grad -> forward -> loss -> backward -> step ->
restore_logstd_head (a no-op here) -> zero clock columns -> zero controller columns.

The critic is never loaded and never trained. Nothing here promotes anything: J3 remains the gate
and its penetration criterion stays <= 0.020 m.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import copy
import datetime as _dt
import hashlib
import json
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
TG = REPO / "Trajectory Generator"
BASELINE = TG / "baseline_MLP"


class J2Error(RuntimeError):
    pass


STAGE = "V26C-J2-BASE-FIT"
RECEIPT_NAME = "v26c_j2_fit_receipt.json"
MODULE_DIR_NAME = "rl_module"

# ------------------------------------------------------------------ J1 input, pinned ------------
J1_LEAF = HERE / "j1_runs" / "j1_nominal_v26c_2026-08-26_r1"
PIN_J1 = {
    "teacher_dataset.npz": "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41",
    "teacher_trace.json": "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3",
    "v26c_j1_collection_receipt.json":
        "f54028d58dc9bfde01ede3c2a72f7ea63b67aeead02979291b03cd468bf37cdd",
}
J1_AMENDMENT = HERE / "v26c_j1_amendment_soft_fail.json"
PIN_J1_AMENDMENT = "db2aa552ab517ed3f2f8f5a74e276a88b549d24ef6d931f2fe3521cf34906bd9"

# ------------------------------------------------------------------ the ONLY actor input --------
V26_PARENT_DIR = (TG / "runs" / "training"
                  / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
V26_PARENT_STATE = V26_PARENT_DIR / "module_state.pkl"
PIN_V26_PARENT_SHA = "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
PIN_MANIFEST39_SHA = "2837779ceb5953b18cf9be0e62836c8cc56d3fe871c48b869a5cedb6b0d81945"

# Source sidecars, pinned. They travel with the module and one of them must be REWRITTEN for 35D.
PARENT_SIDECARS = {
    "metadata.json": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "class_and_ctor_args.pkl": "79dce56ebc56cc53a0b87d06532b00ed0df827e49a08ea454637e35779327700",
}
COPIED_VERBATIM = ("metadata.json",)
DERIVED_SIDECAR = "class_and_ctor_args.pkl"

# ------------------------------------------------------------------ OPTIONAL diagnostic oracle --
# COMPLETELY OUTSIDE the preflight and the fit. Neither opens it, and both work if it is absent.
# It exists so a human can confirm, after the fact, that the fresh derivation reproduces the
# previously pinned masked init. It is never an input to anything.
ORACLE_B0 = (TG / "runs" / "rollout" / "validation" / "v26b_bridge_runs"
             / "2026-08-24_V26B_anchors_r1" / "student" / "B0_35D_MASKED" / "rl_module"
             / "module_state.pkl")

# ------------------------------------------------------------------ derivation recipe -----------
# Preregistered as V26B-bridge-rev3 / V1_init_transplant_39_to_35 and verified to reproduce the
# masked 35D init bit-for-bit from the 39D parent alone.
DROPPED_39D_NAMES = ("healthy_knee_angle_imitation_target",
                     "healthy_knee_angle_imitation_target_vel",
                     "healthy_ankle_angle_imitation_target",
                     "healthy_ankle_angle_imitation_target_vel")
MEAN_BIAS_COMPENSATION = (-0.4178210582236449, 0.013012918564180534,
                          0.2354582386215528, 0.006400180162861944)
LOGSTD_CONSTANT_BIAS = -5.2983174324035645
ACTOR_WIDTH = 35
PARENT_WIDTH = 39
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
MASKED_COLUMNS = tuple(sorted(CLOCK_COLUMNS + CONTROLLER_COLUMNS))
ACTION_DIM = 2
DATASET_ROWS = 500
STATE_KEY_ORDER = ("pi_encoder.0.weight", "pi_encoder.0.bias",
                   "pi_encoder.2.weight", "pi_encoder.2.bias",
                   "pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias",
                   "pi.1.weight", "pi.1.bias")

# ------------------------------------------------------------------ July base hyperparameters ---
JULY_BASE_HP: dict[str, Any] = {
    "seed": 123,
    "batch_size": 64,
    "learning_rate": 3e-4,
    "epochs_max": 400,
    "patience": 60,
    "validation_fraction": 0.20,
    "clip_weight": 1.0,
    "logstd_weight": 0.1,
    "anchor_weight": 1e-05,
    "freeze_logstd_head": False,
    "scope": "the ENTIRE mean network",
    "critic": "excluded, never loaded, never trained",
}
JULY_BASE_PROVENANCE = ("Trajectory Generator/runs/training/"
                        "target_domain_imitation_no_controller_memory_2026-07-13/"
                        "adaptation_report.json :: hyperparameters")
JULY_BASE_REFERENCE_OUTCOME = {"best_epoch": 183, "epochs_run": 243,
                               "best_validation_mse": 0.00014186832413543016,
                               "adapted_rmse": 0.011782177113774129,
                               "note": "July's own numbers on July's own data; a REFERENCE, never "
                                       "a target and never a gate"}
IMPROVEMENT_EPS = 1e-9

OUT_ROOT = HERE / "j2_runs"


def _sha_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ================================================================ J1 inputs ======================

def verify_j1_inputs() -> dict[str, Any]:
    if not J1_LEAF.is_dir():
        raise J2Error(f"the J1 leaf is missing: {J1_LEAF}")
    got: dict[str, str] = {}
    for name, pin in PIN_J1.items():
        p = J1_LEAF / name
        if not p.is_file():
            raise J2Error(f"the J1 artefact {name} is missing")
        h = _sha_file(p)
        if h != pin:
            raise J2Error(f"the J1 artefact {name} changed: {h} != {pin}")
        got[name] = h
    if not J1_AMENDMENT.is_file():
        raise J2Error("the J1 soft-fail amendment is missing; J2 is not authorised without it")
    ah = _sha_file(J1_AMENDMENT)
    if ah != PIN_J1_AMENDMENT:
        raise J2Error(f"the J1 amendment changed: {ah} != {PIN_J1_AMENDMENT}")
    amendment = json.loads(J1_AMENDMENT.read_text())
    auth = amendment["user_authorisation"]
    if auth.get("scope_is_exclusive") is not True or "J2" not in auth.get("granted", ""):
        raise J2Error("the amendment does not carry an exclusive J2 authorisation")
    return {"artefacts_sha256": got, "amendment_sha256": ah,
            "authorisation": auth["granted"], "not_granted": auth["not_granted"],
            "j1_verdict_stands": "FAIL, recorded and not overturned",
            "soft_fail": {k: amendment["the_soft_fail"][k]
                          for k in ("observed_m", "soft_threshold_m", "hard_termination_m")}}


def load_dataset() -> dict[str, Any]:
    d = np.load(J1_LEAF / "teacher_dataset.npz", allow_pickle=False)
    for name in ("observations", "actions", "executed_actions", "action_noises", "times",
                 "actor_feature_names"):
        if name not in d.files:
            raise J2Error(f"the J1 dataset lacks {name}")
    obs = np.asarray(d["observations"], dtype=np.float32)
    act = np.asarray(d["actions"], dtype=np.float32)
    names = tuple(str(n) for n in d["actor_feature_names"])
    if obs.shape != (DATASET_ROWS, ACTOR_WIDTH):
        raise J2Error(f"observations are {obs.shape}, expected {(DATASET_ROWS, ACTOR_WIDTH)}")
    if act.shape != (DATASET_ROWS, ACTION_DIM):
        raise J2Error(f"actions are {act.shape}, expected {(DATASET_ROWS, ACTION_DIM)}")
    if len(names) != ACTOR_WIDTH:
        raise J2Error(f"the feature manifest has {len(names)} names, expected {ACTOR_WIDTH}")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(act))):
        raise J2Error("the J1 dataset contains non-finite values")
    if not np.array_equal(np.asarray(d["executed_actions"], dtype=np.float32), act):
        raise J2Error("executed actions differ from the teacher actions")
    if float(np.max(np.abs(np.asarray(d["action_noises"], dtype=np.float32)))) != 0.0:
        raise J2Error("the recorded action noise is not exactly zero")
    if float(np.max(np.abs(act))) > 1.0 + 1e-6:
        raise J2Error("a teacher label lies outside the normalised action bounds")
    return {"observations": obs, "actions": act, "feature_names": names,
            "rows": int(obs.shape[0]),
            "label_source": "prescribed_teacher_action on states visited by that teacher",
            "controller_columns_present_in_data": True,
            "note": "the ten controller columns carry real values in the dataset; the mask is a "
                    "FIT constraint, not a reason to have omitted them from the collection"}


def masked_observations(obs: np.ndarray) -> np.ndarray:
    out = np.array(obs, dtype=np.float32, copy=True)
    out[:, list(MASKED_COLUMNS)] = 0.0
    return out


# ================================================================ fresh derivation ===============

def verify_parent() -> dict[str, Any]:
    """The ONLY actor input. V1 and B0 are not read here and are not required."""
    if not V26_PARENT_STATE.is_file():
        raise J2Error(f"the V26 August parent is missing: {V26_PARENT_STATE}")
    h = _sha_file(V26_PARENT_STATE)
    if h != PIN_V26_PARENT_SHA:
        raise J2Error(f"the V26 August parent changed: {h} != {PIN_V26_PARENT_SHA}")
    manifest = V26_PARENT_DIR / "actor_feature_manifest.json"
    mh = _sha_file(manifest)
    if mh != PIN_MANIFEST39_SHA:
        raise J2Error(f"the 39D manifest changed: {mh} != {PIN_MANIFEST39_SHA}")
    names = tuple(str(n) for n in json.loads(manifest.read_text())["actor_feature_names"])
    if len(names) != PARENT_WIDTH:
        raise J2Error(f"the parent manifest has {len(names)} names, expected {PARENT_WIDTH}")
    sidecars: dict[str, str] = {}
    for name, pin in PARENT_SIDECARS.items():
        p = V26_PARENT_DIR / name
        if not p.is_file():
            raise J2Error(f"the parent sidecar {name} is missing")
        sh = _sha_file(p)
        if sh != pin:
            raise J2Error(f"the parent sidecar {name} changed: {sh} != {pin}")
        sidecars[name] = sh
    return {"path": str(V26_PARENT_DIR.relative_to(REPO)),
            "module_state_sha256": h, "manifest39_sha256": mh, "width": PARENT_WIDTH,
            "sidecars_sha256": sidecars,
            "sidecar_policy": {"copied_verbatim": list(COPIED_VERBATIM),
                               "derived_for_35d": DERIVED_SIDECAR,
                               "why": "the parent sidecar declares a 39-input actor; copying it "
                                      "verbatim would wrap 35-input weights in a 39-input "
                                      "constructor"},
            "role": "the ONLY actor input to J2",
            "v1_b0_not_required": "neither V1 nor B0 is read by verify_parent, by the derivation "
                                  "or by the fit; they are an optional diagnostic oracle only",
            "july_status": "methodological evidence only; no July checkpoint or dataset is an input"}


def derive_student_35d() -> dict[str, Any]:
    """Build the 35D masked student FRESH, from the 39D parent alone.

    1. map 39 -> 35 by FEATURE NAME, order-preserving;
    2. fold the four dropped `healthy_*_imitation_target*` columns into the first-layer bias with
       the preregistered mean compensation, in float64;
    3. replace the log-std head with the deployable constant bias;
    4. hard-zero the first-layer columns 0,1 and 25..34.
    """
    parent = verify_parent()
    with V26_PARENT_STATE.open("rb") as fh:
        st39 = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    n39 = tuple(str(n) for n in json.loads(
        (V26_PARENT_DIR / "actor_feature_manifest.json").read_text())["actor_feature_names"])
    dropped = [i for i, n in enumerate(n39) if n in DROPPED_39D_NAMES]
    if [n39[i] for i in dropped] != list(DROPPED_39D_NAMES):
        raise J2Error(f"the dropped columns are {[n39[i] for i in dropped]}, expected "
                      f"{list(DROPPED_39D_NAMES)}")
    keep = [i for i in range(PARENT_WIDTH) if i not in dropped]
    if len(keep) != ACTOR_WIDTH:
        raise J2Error(f"the mapping keeps {len(keep)} columns, expected {ACTOR_WIDTH}")
    if keep != sorted(keep):
        raise J2Error("the 39 -> 35 mapping must be order-preserving")
    names35 = tuple(n39[i] for i in keep)

    W1 = np.asarray(st39["pi.0.0.weight"], dtype=np.float32)
    b1 = np.asarray(st39["pi.0.0.bias"], dtype=np.float32)
    m = np.asarray(MEAN_BIAS_COMPENSATION, dtype=np.float64)
    W1n = np.ascontiguousarray(W1[:, keep].copy())
    b1n = np.ascontiguousarray(
        (b1.astype(np.float64) + W1[:, dropped].astype(np.float64) @ m).astype(np.float32))
    W1n[:, list(MASKED_COLUMNS)] = 0.0

    W3 = np.asarray(st39["pi.1.weight"], dtype=np.float32).copy()
    b3 = np.asarray(st39["pi.1.bias"], dtype=np.float32).copy()
    W3[ACTION_DIM:] = 0.0
    b3[ACTION_DIM:] = np.float32(LOGSTD_CONSTANT_BIAS)

    W2 = np.ascontiguousarray(np.asarray(st39["pi.0.2.weight"], dtype=np.float32).copy())
    b2 = np.ascontiguousarray(np.asarray(st39["pi.0.2.bias"], dtype=np.float32).copy())
    state = {"pi_encoder.0.weight": W1n, "pi_encoder.0.bias": b1n,
             "pi_encoder.2.weight": W2, "pi_encoder.2.bias": b2,
             "pi.0.0.weight": W1n, "pi.0.0.bias": b1n,
             "pi.0.2.weight": W2, "pi.0.2.bias": b2,
             "pi.1.weight": np.ascontiguousarray(W3), "pi.1.bias": np.ascontiguousarray(b3)}
    if tuple(state) != STATE_KEY_ORDER:
        raise J2Error("the derived state does not carry the ten expected keys in order")
    zero = [c for c in range(ACTOR_WIDTH) if bool(np.all(W1n[:, c] == 0.0))]
    if zero != list(MASKED_COLUMNS):
        raise J2Error(f"the derived zero columns are {zero}, expected {list(MASKED_COLUMNS)}")
    report = {
        "parent": parent,
        "mapping": {"kind": "by feature name, order-preserving",
                    "dropped_39d_indices": dropped,
                    "dropped_names": list(DROPPED_39D_NAMES),
                    "kept": len(keep)},
        "mean_bias_compensation": {"applied": True, "float64": list(MEAN_BIAS_COMPENSATION),
                                   "formula": "b1 += W1_39[:, dropped] @ m, in float64",
                                   "bias_max_abs_change":
                                       float(np.max(np.abs(b1n - b1)))},
        "logstd_head": {"rows_zeroed": True, "constant_bias": LOGSTD_CONSTANT_BIAS,
                        "sigma": float(np.exp(LOGSTD_CONSTANT_BIAS))},
        "hard_zero_columns": list(MASKED_COLUMNS),
        "derived_actor_digest": _sha_obj({k: np.asarray(v).tobytes().hex()
                                          for k, v in state.items()}),
        "width": ACTOR_WIDTH,
        "critic": "never loaded; the parent's non-actor tensors are not read",
    }
    return {"state": state, "feature_names": names35, "report": report}


def derive_ctor_args_35d(ctor: Any) -> tuple[Any, dict[str, Any]]:
    """Rewrite the 39D constructor arguments for the 35D student. PURE: takes the loaded object.

    Only two fields change, and both are COMPUTED from the parent rather than hardcoded:
      * model_config['n_actor']: 39 -> 35;
      * observation_space: its width becomes 35 + privileged, where privileged is the parent's own
        observation width minus its own n_actor.
    Everything else - the class, the positional arguments, the action space, inference_only,
    learner_only, catalog_class and every other model_config entry - is carried over untouched.
    Copying the parent's sidecar verbatim would declare a 39-input actor around 35-input weights.
    """
    if not isinstance(ctor, Mapping) or "class" not in ctor or "ctor_args_and_kwargs" not in ctor:
        raise J2Error("the constructor sidecar does not have the expected shape")
    args, kwargs = ctor["ctor_args_and_kwargs"]
    kwargs = dict(kwargs)
    model_config = dict(kwargs["model_config"])
    obs = kwargs["observation_space"]
    n_actor = int(model_config["n_actor"])
    if n_actor != PARENT_WIDTH:
        raise J2Error(f"the parent sidecar declares n_actor={n_actor}, expected {PARENT_WIDTH}")
    shape = tuple(int(v) for v in obs.shape)
    if len(shape) != 1:
        raise J2Error(f"the observation space is {shape}, expected one-dimensional")
    privileged = shape[0] - n_actor
    if privileged < 0:
        raise J2Error("the observation width is smaller than n_actor")
    low = np.asarray(obs.low).reshape(-1)
    high = np.asarray(obs.high).reshape(-1)
    if not (np.all(low == low[0]) and np.all(high == high[0])):
        raise J2Error("the observation bounds are not uniform; refusing to rebuild the space")
    new_shape = (ACTOR_WIDTH + privileged,)
    model_config["n_actor"] = ACTOR_WIDTH
    kwargs["model_config"] = model_config
    kwargs["observation_space"] = type(obs)(low=float(low[0]), high=float(high[0]),
                                            shape=new_shape, dtype=obs.dtype)
    derived = {"class": ctor["class"], "ctor_args_and_kwargs": (args, kwargs)}
    report = {"changed": {"model_config.n_actor": {"from": n_actor, "to": ACTOR_WIDTH},
                          "observation_space.shape": {"from": list(shape),
                                                      "to": list(new_shape)}},
              "privileged_width_computed_from_parent": privileged,
              "unchanged": sorted(k for k in kwargs if k not in
                                  ("model_config", "observation_space")),
              "model_config_unchanged": sorted(k for k in model_config if k != "n_actor"),
              "rule": "observation width = n_actor + privileged; privileged is the parent's own "
                      "observation width minus its own n_actor, never a literal"}
    return derived, report


def load_parent_ctor() -> Any:
    """Unpickle the parent constructor sidecar.

    The pickle names `asymmetric_rl_module`, which lives in baseline_MLP, so that directory must
    be importable first. Omitting this is exactly what made the first J2 attempt die AFTER the 400
    epochs had already been computed, while saving.
    """
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    with (V26_PARENT_DIR / DERIVED_SIDECAR).open("rb") as fh:
        return pickle.load(fh)


def verify_staged_sidecar(path: Path) -> dict[str, Any]:
    """Reload the sidecar actually written and confirm it describes a 35D actor."""
    with path.open("rb") as fh:
        ctor = pickle.load(fh)
    args, kwargs = ctor["ctor_args_and_kwargs"]
    n_actor = int(kwargs["model_config"]["n_actor"])
    shape = tuple(int(v) for v in kwargs["observation_space"].shape)
    if n_actor != ACTOR_WIDTH:
        raise J2Error(f"the staged sidecar declares n_actor={n_actor}, expected {ACTOR_WIDTH}")
    if shape[0] - n_actor < 0:
        raise J2Error("the staged observation width is smaller than n_actor")
    return {"path": str(path.name), "sha256": _sha_file(path),
            "n_actor": n_actor, "observation_shape": list(shape),
            "verified": "reloaded from disk and confirmed to describe a 35D actor"}


def diagnostic_oracle(state: Mapping[str, np.ndarray] | None = None) -> dict[str, Any]:
    """OPTIONAL and OUT OF BAND. Neither preflight() nor fit() calls this.

    It exists so a human can confirm, after the fact, that the fresh derivation reproduces the
    previously pinned masked init. Its absence blocks nothing.
    """
    if state is None:
        state = derive_student_35d()["state"]
    if not ORACLE_B0.is_file():
        return {"available": False, "note": "the oracle is optional; its absence blocks nothing"}
    with ORACLE_B0.open("rb") as fh:
        b0 = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    per_key = {k: bool(np.array_equal(np.asarray(state[k]), np.asarray(b0[k])))
               for k in state if k in b0}
    return {"available": True, "oracle_sha256": _sha_file(ORACLE_B0),
            "same_keys": sorted(state) == sorted(b0),
            "per_key_bit_identical": per_key,
            "bit_identical": all(per_key.values()) and sorted(state) == sorted(b0),
            "status": "DIAGNOSTIC ONLY, OUT OF BAND - not called by preflight or fit"}


# ================================================================ split and shared RNG ===========

def july_rng_and_split(rows: int = DATASET_ROWS, *, seed: int | None = None,
                       validation_fraction: float | None = None
                       ) -> tuple[np.random.Generator, dict[str, Any]]:
    """ONE generator: the split permutation is drawn first, and the SAME object is returned so the
    epoch shuffles continue that stream - exactly as target_domain_imitation.adapt_actor does."""
    seed = JULY_BASE_HP["seed"] if seed is None else seed
    frac = JULY_BASE_HP["validation_fraction"] if validation_fraction is None \
        else validation_fraction
    rng = np.random.default_rng(int(seed))
    indices = rng.permutation(int(rows))
    validation_count = max(1, int(round(len(indices) * float(frac))))
    validation = np.sort(indices[:validation_count])
    training = np.asarray(indices[validation_count:], dtype=int)
    if training.size == 0:
        raise J2Error("the validation split left no training samples")
    if set(training.tolist()) & set(validation.tolist()):
        raise J2Error("the split overlaps")
    if sorted(training.tolist() + validation.tolist()) != list(range(int(rows))):
        raise J2Error("the split does not cover every row exactly once")
    report = {"training_indices": training, "validation_indices": validation,
              "training_rows": int(training.size), "validation_rows": int(validation.size),
              "mode": "seeded_random_fraction", "seed": int(seed),
              "validation_fraction": float(frac),
              "training_sha256": _sha_obj(sorted(int(i) for i in training)),
              "validation_sha256": _sha_obj(sorted(int(i) for i in validation)),
              "rng_stream": "the SAME generator continues into the epoch shuffles",
              "provenance": "target_domain_imitation._resolve_adaptation_split + adapt_actor"}
    return rng, report


# ================================================================ preflight ======================

def preflight() -> dict[str, Any]:
    j1 = verify_j1_inputs()
    derived = derive_student_35d()
    data = load_dataset()
    if tuple(data["feature_names"]) != tuple(derived["feature_names"]):
        raise J2Error("the dataset feature manifest differs from the derived student's")
    masked = masked_observations(data["observations"])
    if not bool(np.all(masked[:, list(MASKED_COLUMNS)] == 0.0)):
        raise J2Error("the input mask did not zero every masked column")
    untouched = [c for c in range(ACTOR_WIDTH) if c not in MASKED_COLUMNS]
    if not bool(np.array_equal(masked[:, untouched], data["observations"][:, untouched])):
        raise J2Error("the input mask altered a column it must not touch")
    _, split = july_rng_and_split(data["rows"])
    leaves = sorted(p.name for p in OUT_ROOT.iterdir()) if OUT_ROOT.is_dir() else []
    return {
        "verdict": "GO", "stage": STAGE, "blockers": [],
        "inert": {"fit_executed": False, "rollout_executed": False, "promotion": False,
                  "note": "torch is imported inside fit() only"},
        "j1_input": j1,
        "derivation": derived["report"],
        "diagnostic_oracle": {"consulted": False,
                              "reason": "the oracle is out of band: neither preflight nor fit "
                                        "opens it, and both work if it is absent"},
        "dataset": {k: v for k, v in data.items()
                    if k not in ("observations", "actions", "feature_names")},
        "masking": {"masked_columns": list(MASKED_COLUMNS), "clock": list(CLOCK_COLUMNS),
                    "controller": list(CONTROLLER_COLUMNS),
                    "effective_support_dimensions": ACTOR_WIDTH - len(MASKED_COLUMNS)},
        "hyperparameters": dict(JULY_BASE_HP),
        "hyperparameter_provenance": JULY_BASE_PROVENANCE,
        "july_reference_outcome_NOT_A_TARGET": dict(JULY_BASE_REFERENCE_OUTCOME),
        "split": {k: v for k, v in split.items()
                  if k not in ("training_indices", "validation_indices")},
        "no_clobber": {"scope": "PER LEAF", "root": str(OUT_ROOT.relative_to(REPO)),
                       "existing_leaves": leaves},
        "execution_requires": f"--authorized-stage {STAGE}",
        "gating": {"j3_still_required": True, "j3_penetration_max_m": 0.020,
                   "no_promotion_without_j3": True,
                   "forbidden_here": ["rollout", "J3", "retry", "Markov", "LOTO", "LOCO",
                                      "B1R1", "B1R2", "promotion", "ex-novo"]},
        "generated_at_utc": _utc(),
    }


# ================================================================ the fit ========================

def fit(*, authorized_stage: str | None, out_dir: Path | None = None,
        progress: bool = True) -> dict[str, Any]:
    """July's mechanism, executed. Requires the explicit token."""
    if authorized_stage != STAGE:
        raise J2Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    pre = preflight()
    if pre["blockers"]:
        raise J2Error(f"preflight BLOCKED: {pre['blockers']}")

    out = Path(out_dir) if out_dir is not None else (OUT_ROOT / f"j2_{_utc().replace(':', '')}")
    if out.exists():
        raise J2Error(f"no-clobber: the leaf {out} already exists; choose a fresh --out-dir")

    # Everything the saving stage will need is resolved and validated BEFORE a single epoch runs.
    # The first J2 attempt computed all 400 epochs and then died unpickling this sidecar; the cost
    # of that mistake is paid here instead, in milliseconds.
    derived_ctor, ctor_report = derive_ctor_args_35d(load_parent_ctor())
    derived = derive_student_35d()
    init_state = {k: np.asarray(v, dtype=np.float32) for k, v in derived["state"].items()}
    data = load_dataset()
    obs = masked_observations(data["observations"])
    act = np.asarray(data["actions"], dtype=np.float32)
    rng, split = july_rng_and_split(data["rows"])          # the SAME rng continues below
    train_idx = split["training_indices"]
    val_idx = split["validation_indices"]

    import torch                                            # heavy import, fit only
    from torch.nn import functional
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(int(JULY_BASE_HP["seed"]))

    X = torch.as_tensor(obs)
    Y = torch.as_tensor(act)
    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(init_state[k])))  # noqa: E731
    W1, b1 = P("pi.0.0.weight"), P("pi.0.0.bias")
    W2, b2 = P("pi.0.2.weight"), P("pi.0.2.bias")
    W3, b3 = P("pi.1.weight"), P("pi.1.bias")
    params = [W1, b1, W2, b2, W3, b3]
    names = ["pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias",
             "pi.1.weight", "pi.1.bias"]
    anchor = {n: p.detach().clone() for n, p in zip(names, params)}
    fwd = lambda x: torch.tanh(torch.tanh(x @ W1.T + b1) @ W2.T + b2) @ W3.T + b3  # noqa: E731
    with torch.no_grad():
        source_logstd = fwd(X)[:, ACTION_DIM:].detach().clone()
    opt = torch.optim.Adam(params, lr=float(JULY_BASE_HP["learning_rate"]))
    zero_cols = list(MASKED_COLUMNS)

    def project_columns() -> None:
        with torch.no_grad():
            W1[:, zero_cols] = 0.0

    def restore_logstd_head() -> None:
        # July: a no-op when freeze_logstd_head is False. The log-std is anchored by the loss.
        if JULY_BASE_HP["freeze_logstd_head"]:
            with torch.no_grad():
                W3[ACTION_DIM:] = torch.as_tensor(np.array(init_state["pi.1.weight"]))[ACTION_DIM:]
                b3[ACTION_DIM:] = torch.as_tensor(np.array(init_state["pi.1.bias"]))[ACTION_DIM:]

    def snapshot() -> dict[str, np.ndarray]:
        w1 = W1.detach().numpy().copy(); bb1 = b1.detach().numpy().copy()
        w2 = W2.detach().numpy().copy(); bb2 = b2.detach().numpy().copy()
        w3 = W3.detach().numpy().copy(); bb3 = b3.detach().numpy().copy()
        return {"pi_encoder.0.weight": w1, "pi_encoder.0.bias": bb1,
                "pi_encoder.2.weight": w2, "pi_encoder.2.bias": bb2,
                "pi.0.0.weight": w1, "pi.0.0.bias": bb1,
                "pi.0.2.weight": w2, "pi.0.2.bias": bb2,
                "pi.1.weight": w3, "pi.1.bias": bb3}

    def validation_mse() -> float:
        with torch.no_grad():
            m = fwd(X[val_idx])[:, :ACTION_DIM]
            return float(functional.mse_loss(m, Y[val_idx]).item())

    history: list[dict[str, Any]] = []
    best_state = copy.deepcopy(snapshot())
    best_val = float("inf")
    best_epoch = 0
    stale = 0
    stopped_at: int | None = None
    epochs_max = int(JULY_BASE_HP["epochs_max"])
    batch = int(JULY_BASE_HP["batch_size"])
    for epoch in range(1, epochs_max + 1):
        shuffled = rng.permutation(train_idx)               # SAME generator, July's stream
        epoch_losses: list[float] = []
        for start in range(0, len(shuffled), batch):
            idx = torch.as_tensor(np.asarray(shuffled[start:start + batch], dtype=np.int64))
            opt.zero_grad(set_to_none=True)
            logits = fwd(X[idx])
            means = logits[:, :ACTION_DIM]
            mean_loss = functional.mse_loss(means, Y[idx])
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = functional.mse_loss(logits[:, ACTION_DIM:], source_logstd[idx])
            anchor_loss = torch.stack([(p - anchor[n]).square().mean()
                                       for n, p in zip(names, params)]).mean()
            loss = (mean_loss
                    + float(JULY_BASE_HP["clip_weight"]) * clip_loss
                    + float(JULY_BASE_HP["logstd_weight"]) * logstd_loss
                    + float(JULY_BASE_HP["anchor_weight"]) * anchor_loss)
            loss.backward()
            opt.step()
            restore_logstd_head()
            project_columns()
            if not bool(torch.all(W1.detach()[:, zero_cols] == 0.0)):
                raise J2Error("a masked column is not exactly zero after the step")
            epoch_losses.append(float(loss.item()))
        val = validation_mse()
        history.append({"epoch": epoch, "train_loss": float(np.mean(epoch_losses)),
                        "validation_mse": val})
        if val < best_val - IMPROVEMENT_EPS:
            best_val, best_epoch, stale = val, epoch, 0
            best_state = copy.deepcopy(snapshot())
        else:
            stale += 1
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps({"epoch": epoch, "val_mse": val}), flush=True)
        if stale >= int(JULY_BASE_HP["patience"]):
            stopped_at = epoch
            break

    final = {k: np.ascontiguousarray(np.asarray(v, dtype=np.float32))
             for k, v in best_state.items()}
    W1f = final["pi.0.0.weight"]
    if [c for c in range(ACTOR_WIDTH) if bool(np.all(W1f[:, c] == 0.0))] != list(MASKED_COLUMNS):
        raise J2Error("the selected state does not keep exactly the masked columns at zero")

    def predict(state: Mapping[str, np.ndarray], rows: np.ndarray) -> np.ndarray:
        h = np.tanh(obs[rows] @ np.asarray(state["pi.0.0.weight"]).T
                    + np.asarray(state["pi.0.0.bias"]))
        h = np.tanh(h @ np.asarray(state["pi.0.2.weight"]).T + np.asarray(state["pi.0.2.bias"]))
        return (h @ np.asarray(state["pi.1.weight"]).T
                + np.asarray(state["pi.1.bias"]))[:, :ACTION_DIM]

    all_rows = np.arange(data["rows"])
    init_pred = predict(init_state, all_rows)
    final_pred = predict(final, all_rows)
    metrics = {
        "initial": {"rmse": float(np.sqrt(np.mean((init_pred - act) ** 2))),
                    "max_abs_error": float(np.max(np.abs(init_pred - act)))},
        "adapted": {"rmse": float(np.sqrt(np.mean((final_pred - act) ** 2))),
                    "max_abs_error": float(np.max(np.abs(final_pred - act)))},
        "adapted_validation_rmse": float(np.sqrt(np.mean((final_pred[val_idx]
                                                          - act[val_idx]) ** 2))),
        "adapted_training_rmse": float(np.sqrt(np.mean((final_pred[train_idx]
                                                        - act[train_idx]) ** 2))),
    }
    logstd_change = float(np.max(np.abs(final["pi.1.weight"][ACTION_DIM:]
                                        - init_state["pi.1.weight"][ACTION_DIM:])))
    logstd_bias_change = float(np.max(np.abs(final["pi.1.bias"][ACTION_DIM:]
                                             - init_state["pi.1.bias"][ACTION_DIM:])))

    out.mkdir(parents=True, exist_ok=False)
    try:
        sm = out / MODULE_DIR_NAME
        sm.mkdir(parents=True, exist_ok=False)
        with (sm / "module_state.pkl").open("wb") as fh:
            pickle.dump(final, fh, protocol=pickle.HIGHEST_PROTOCOL)
        for extra in COPIED_VERBATIM:
            shutil.copy2(V26_PARENT_DIR / extra, sm / extra)
        with (sm / DERIVED_SIDECAR).open("wb") as fh:
            pickle.dump(derived_ctor, fh, protocol=pickle.HIGHEST_PROTOCOL)
        staged_sidecar = verify_staged_sidecar(sm / DERIVED_SIDECAR)
        (sm / "actor_feature_manifest.json").write_text(json.dumps({
            "schema_version": 1,
            "actor_feature_names": list(derived["feature_names"]),
            "actor_feature_count": ACTOR_WIDTH,
            "module_state_sha256": _sha_file(sm / "module_state.pkl"),
            "derived_from": str(V26_PARENT_DIR.relative_to(REPO)),
            "source_module_state_sha256": PIN_V26_PARENT_SHA,
            "controller_state_mask": {"active": True, "columns": list(CONTROLLER_COLUMNS)},
            "clock_columns": list(CLOCK_COLUMNS),
            "deployable": False, "sigma_unresolved": True,
            "actor_label": "J2_BASE35_JULY_FAITHFUL",
            "status": "base-phase fit; never promoted; J3 not run",
        }, indent=2) + "\n", encoding="utf-8")
        receipt = {
            "schema": "v26c_j2_fit.1", "stage": STAGE,
            "j1_input": pre["j1_input"], "derivation": pre["derivation"],
            "ctor_sidecar": {"source_sha256": dict(PARENT_SIDECARS),
                             "copied_verbatim": list(COPIED_VERBATIM),
                             "derived": ctor_report, "staged": staged_sidecar},
            "diagnostic_oracle": pre["diagnostic_oracle"],
            "dataset": pre["dataset"], "masking": pre["masking"],
            "hyperparameters": dict(JULY_BASE_HP),
            "hyperparameter_provenance": JULY_BASE_PROVENANCE,
            "july_reference_outcome_NOT_A_TARGET": dict(JULY_BASE_REFERENCE_OUTCOME),
            "split": {k: v for k, v in split.items()
                      if k not in ("training_indices", "validation_indices")},
            "selection": {"best_epoch": best_epoch, "best_validation_mse": best_val,
                          "epochs_run": len(history), "stopped_at_epoch": stopped_at,
                          "rule": "first epoch improving by more than 1e-9; stop after "
                                  f"{JULY_BASE_HP['patience']} stale epochs; best state restored"},
            "metrics": metrics,
            "logstd": {"freeze_logstd_head": JULY_BASE_HP["freeze_logstd_head"],
                       "logstd_weight": JULY_BASE_HP["logstd_weight"],
                       "weight_max_abs_change": logstd_change,
                       "bias_max_abs_change": logstd_bias_change,
                       "note": "anchored by the loss, not frozen by construction"},
            "critic": "never loaded, never trained",
            "deployable": False, "promotion": "NONE",
            "gating": pre["gating"],
            "output_files_sha256": {p.name: _sha_file(p)
                                    for p in sorted(sm.iterdir()) if p.is_file()},
            "generated_at_utc": _utc(),
        }
        (out / "history.json").write_text(
            json.dumps(history, indent=1, allow_nan=False), encoding="utf-8")
        receipt["history_sha256"] = _sha_file(out / "history.json")
        (out / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
    except BaseException:
        shutil.rmtree(out, ignore_errors=True)
        raise
    if progress:
        print(json.dumps({"best_epoch": best_epoch, "best_validation_mse": best_val,
                          "adapted_rmse": metrics["adapted"]["rmse"]}, indent=2))
    return receipt


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J2 July-faithful base fit")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    fit(authorized_stage=a.authorized_stage,
        out_dir=(Path(a.out_dir) if a.out_dir else None), progress=not a.no_progress)
    return 0


if __name__ == "__main__":
    sys.exit(main())
