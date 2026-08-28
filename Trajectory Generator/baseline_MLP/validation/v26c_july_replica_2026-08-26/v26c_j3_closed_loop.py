"""V26C J3 - closed-loop qualification of the J2 actor. The rollout is NOT run by the preflight.

J3 rolls the FITTED J2 actor deterministically under the pinned EGRF/FSM v3 runtime and applies
the full closed-loop gate: the J1 COMMON gate plus the kinematic quality that binds J3 only.

WHAT IT USES
    * the six J2 artefacts, each pinned by exact SHA-256; a mutated or missing one aborts;
    * the pinned runtime config a870cc38..., through the SAME full env_config builder J1 uses,
      already hardened for the twelve v3 keys, the 44 verified fields and the whole reward block.

HOW IT RUNS
    `--preflight` is inert: it constructs, resets and steps NOTHING, and imports no heavy stack.
    The rollout itself needs BOTH the exact token `--authorized-stage V26C-J3-CLOSED-LOOP` and an
    explicit `--out-dir`, which must be a fresh leaf. Torch, RLlib and OpenSim are imported inside
    run() only.

DETERMINISTIC ONLY
    numpy, torch and env.reset are all seeded with 123. The action is the deterministic sample of
    the module's inference distribution, verified bit for bit against rollout_eval's own helper.
    No exploration noise. Sigma is forbidden at this stage and is neither read nor set.

RAW ACTION IN, CLIPPED MIRROR RECORDED
    The environment is stepped with the RAW deterministic action, exactly as rollout_eval does, so
    the reward shaping still sees the raw-vs-applied excursion. The clipped action is recorded as a
    diagnostic and is never a gate.

CLIPPING IS DIAGNOSTIC
    Recorded, never a gate. Promoting it would need an already-approved source, which does not
    exist.

WHAT IT DOES NOT DO
    No retry, no Markov phase, no DAgger, no sigma, no LOTO/LOCO/B1R1/B1R2, no promotion, no
    ex-novo. A J3 PASS confers no promotion by itself.

THE J1 SOFT FAIL STANDS
    The J1 receipt keeps its FAIL verdict and its amendment. J3 neither re-scores nor retroactively
    promotes them, whatever it finds.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
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
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1  # noqa: E402  the hardened, tested env builder and trace parser


class J3Error(RuntimeError):
    pass


STAGE = "V26C-J3-CLOSED-LOOP"
RECEIPT_NAME = "v26c_j3_closed_loop_receipt.json"

# ------------------------------------------------------------------ the J2 actor, pinned --------
J2_LEAF = HERE / "j2_runs" / "j2_base_v26c_2026-08-26_r1"
PIN_J2 = {
    "history.json":
        "b5b4641c0170aac5af615b623746e2c56bd6d1ecafd3cd5657ff63ee9917c777",
    "rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "rl_module/module_state.pkl":
        "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130",
    "v26c_j2_fit_receipt.json":
        "dd997dc9d465184de6e25a3488c9752799fb60feca304ac876f35dfbdcd9871e",
}
J2_MODULE_DIR = J2_LEAF / "rl_module"

# ------------------------------------------------------------------ J1 lineage, preserved -------
J1_LEAF = J1.OUT_ROOT / "j1_nominal_v26c_2026-08-26_r1"
PIN_J1_RECEIPT = "f54028d58dc9bfde01ede3c2a72f7ea63b67aeead02979291b03cd468bf37cdd"
J1_AMENDMENT = HERE / "v26c_j1_amendment_soft_fail.json"
PIN_J1_AMENDMENT = "db2aa552ab517ed3f2f8f5a74e276a88b549d24ef6d931f2fe3521cf34906bd9"

# ------------------------------------------------------------------ contract --------------------
ACTOR_WIDTH = J1.ACTOR_WIDTH
MASKED_COLUMNS = J1.MASKED_COLUMNS
PIN_RUNTIME_CONFIG_SHA = J1.PIN_RUNTIME_CONFIG_SHA
# The FULL asymmetric observation: 35 actor features + 49 privileged critic features. Carried by
# the pinned J2 ctor sidecar (observation_space Box(84,), n_actor 35, n_full 84) and re-verified
# against the live runtime before the first step.
FULL_OBS_WIDTH = 84

# ------------------------------------------------------------------ the binding J3 gate ---------
# COMMON part, inherited verbatim from J1.
J3_COMMON_GATE: dict[str, Any] = dict(J1.J1_GATE)
# KINEMATIC part, from v26b_b1_base_fit.declared_closed_loop_gates. Binds J3 only.
J3_KINEMATIC_GATE: dict[str, Any] = {
    "ankle_min_rad_max": -0.03,
    "ankle_amplitude_min_rad": 0.30,
    "knee_amplitude_min_rad": 0.60,
    "knee_strictly_flexed": True,
    "knee_bounds_rad": (-1.5, 0.0),
    "ankle_bounds_rad": (-0.7, 0.7),
}
# HS/TO coherence is a FAIL-CLOSED TELEMETRY INTEGRITY INVARIANT, not a behavioural gate and not a
# promotion threshold. It asks whether the recorded event evidence is self-consistent. If it is
# not, the qualification is technically INVALID - which is a statement about the evidence, never a
# statement about the gait. It is evaluated and recorded SEPARATELY from the behavioural gate.
TELEMETRY_INTEGRITY_INVARIANT = {
    "hs_at_least_cycles": True,
    "to_at_least_cycles": True,
    "hs_to_difference_max": 1,
    "kind": "TELEMETRY INTEGRITY INVARIANT",
    "note": "a valid cycle is bounded by its own HS and TO, so neither count may fall below the "
            "cycle count, and the two may differ by at most one because an episode can end "
            "mid-cycle. This checks the SELF-CONSISTENCY of the recorded evidence; it invents no "
            "behavioural or promotion threshold and is not part of the behavioural gate.",
    "on_violation": "the qualification is technically INVALID: the evidence contradicts itself, so "
                    "neither a PASS nor a FAIL on gait can be asserted from it",
}
DIAGNOSTIC_NOT_BINDING = ("action_clipped_steps",)

# ------------------------------------------------------------------ determinism -----------------
ROLLOUT_SEED = 123
NO_EXPLORATION_NOISE = True
EXPECTED_STEPS = J1.EXPECTED_STEPS

OUT_ROOT = HERE / "j3_runs"


def _sha_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ================================================================ inputs =========================

def verify_j2_actor() -> dict[str, Any]:
    """The six J2 artefacts, by exact hash. Nothing else may be used as the actor."""
    if not J2_LEAF.is_dir():
        raise J3Error(f"the J2 leaf is missing: {J2_LEAF}")
    present = sorted(str(p.relative_to(J2_LEAF)) for p in J2_LEAF.rglob("*") if p.is_file())
    if present != sorted(PIN_J2):
        raise J3Error(f"the J2 leaf holds {present}, expected exactly {sorted(PIN_J2)}")
    got: dict[str, str] = {}
    for name, pin in PIN_J2.items():
        h = _sha_file(J2_LEAF / name)
        if h != pin:
            raise J3Error(f"the J2 artefact {name} changed: {h} != {pin}")
        got[name] = h
    receipt = json.loads((J2_LEAF / "v26c_j2_fit_receipt.json").read_text())
    if receipt["deployable"] is not False or receipt["promotion"] != "NONE":
        raise J3Error("the J2 receipt claims deployability or promotion; refusing")
    return {"leaf": _rel(J2_LEAF), "artefacts_sha256": got,
            "best_epoch": receipt["selection"]["best_epoch"],
            "adapted_rmse": receipt["metrics"]["adapted"]["rmse"],
            "deployable": receipt["deployable"], "promotion": receipt["promotion"],
            "role": "the ONLY actor J3 may qualify"}


def verify_j1_lineage() -> dict[str, Any]:
    """The J1 FAIL and its amendment are preserved. J3 does not re-score or promote them."""
    r = J1_LEAF / "v26c_j1_collection_receipt.json"
    if not r.is_file():
        raise J3Error("the J1 receipt is missing")
    h = _sha_file(r)
    if h != PIN_J1_RECEIPT:
        raise J3Error(f"the J1 receipt changed: {h} != {PIN_J1_RECEIPT}")
    ah = _sha_file(J1_AMENDMENT)
    if ah != PIN_J1_AMENDMENT:
        raise J3Error(f"the J1 amendment changed: {ah} != {PIN_J1_AMENDMENT}")
    receipt = json.loads(r.read_text())
    if receipt["verdict"] != "FAIL" or receipt["gate"]["failed"] != ["max_penetration_m"]:
        raise J3Error("the J1 receipt no longer records its original FAIL on max_penetration_m")
    return {"j1_receipt_sha256": h, "j1_amendment_sha256": ah,
            "j1_verdict": receipt["verdict"],
            "j1_failed": receipt["gate"]["failed"],
            "j1_max_penetration_m": receipt["summary"]["max_penetration_m"],
            "statement": "the J1 soft fail STANDS. J3 neither re-scores it nor retroactively "
                         "promotes it, whatever J3 itself finds."}


def build_env_config(*, output_dir: Path | None = None) -> dict[str, Any]:
    """The FULL env_config, from the SAME hardened builder J1 uses."""
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_dir)
    J1.verify_env_config(env, cfg)
    return env


# ================================================================ kinematics =====================

def kinematic_quality(knee: np.ndarray, ankle: np.ndarray) -> dict[str, Any]:
    """The kinematic gate that binds J3 only, evaluated on the prosthetic joint trajectories."""
    k = np.asarray(knee, dtype=np.float64)
    a = np.asarray(ankle, dtype=np.float64)
    if k.size == 0 or a.size == 0:
        raise J3Error("no kinematics were recorded")
    if not (np.all(np.isfinite(k)) and np.all(np.isfinite(a))):
        raise J3Error("the recorded kinematics contain non-finite values")
    kb = J3_KINEMATIC_GATE["knee_bounds_rad"]
    ab = J3_KINEMATIC_GATE["ankle_bounds_rad"]
    ankle_min = float(a.min())
    ankle_amp = float(a.max() - a.min())
    knee_amp = float(k.max() - k.min())
    return {
        "ankle_min": {"observed_rad": ankle_min,
                      "threshold_rad": J3_KINEMATIC_GATE["ankle_min_rad_max"],
                      "pass": bool(ankle_min <= J3_KINEMATIC_GATE["ankle_min_rad_max"])},
        "ankle_amplitude": {"observed_rad": ankle_amp,
                            "threshold_rad": J3_KINEMATIC_GATE["ankle_amplitude_min_rad"],
                            "pass": bool(ankle_amp >= J3_KINEMATIC_GATE["ankle_amplitude_min_rad"])},
        "knee_amplitude": {"observed_rad": knee_amp,
                           "threshold_rad": J3_KINEMATIC_GATE["knee_amplitude_min_rad"],
                           "pass": bool(knee_amp >= J3_KINEMATIC_GATE["knee_amplitude_min_rad"])},
        "knee_strictly_flexed": {"fraction_negative": float((k < 0.0).mean()),
                                 "max_rad": float(k.max()),
                                 "pass": bool(np.all(k < 0.0))},
        "within_bounds": {"knee_outside": int(np.sum((k < kb[0]) | (k > kb[1]))),
                          "ankle_outside": int(np.sum((a < ab[0]) | (a > ab[1]))),
                          "knee_bounds_rad": list(kb), "ankle_bounds_rad": list(ab),
                          "pass": bool(np.all((k >= kb[0]) & (k <= kb[1]))
                                       and np.all((a >= ab[0]) & (a <= ab[1])))},
    }


def telemetry_integrity(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Is the recorded event evidence self-consistent? NOT a behavioural or promotion threshold."""
    hs = int(summary["valid_hs_count"])
    to = int(summary["valid_to_count"])
    cycles = int(summary["valid_cycle_count"])
    checks = {
        "hs_at_least_cycles": hs >= cycles,
        "to_at_least_cycles": to >= cycles,
        "hs_to_difference_within_one":
            abs(hs - to) <= TELEMETRY_INTEGRITY_INVARIANT["hs_to_difference_max"],
    }
    ok = all(checks.values())
    return {"kind": TELEMETRY_INTEGRITY_INVARIANT["kind"],
            "valid_hs_count": hs, "valid_to_count": to, "valid_cycle_count": cycles,
            "checks": checks, "pass": ok,
            "qualification_technically_valid": ok,
            "invariant": dict(TELEMETRY_INTEGRITY_INVARIANT),
            "is_behavioural_gate": False, "is_promotion_threshold": False}


def evaluate_gate(summary: Mapping[str, Any], knee: np.ndarray, ankle: np.ndarray
                  ) -> dict[str, Any]:
    """The BEHAVIOURAL J3 gate: the J1 COMMON gate + the full kinematic quality. Nothing else.

    HS/TO coherence is deliberately absent: it is a telemetry integrity invariant, evaluated and
    recorded separately by telemetry_integrity(), never a behavioural criterion.
    """
    common = J1._evaluate_gate(summary)
    kin = kinematic_quality(knee, ankle)
    checks = dict(common["checks"])
    for name, block in kin.items():
        checks[f"kinematic_{name}"] = bool(block["pass"])
    failed = sorted(k for k, v in checks.items() if not v)
    return {"common": common, "kinematic_quality": kin,
            "checks": checks, "failed": failed, "pass": not failed,
            "kinematic_quality_applies": True,
            "telemetry_integrity_evaluated_separately": True,
            "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING}}


def overall_verdict(gate: Mapping[str, Any], integrity: Mapping[str, Any]) -> str:
    """INVALID when the evidence contradicts itself; otherwise the behavioural gate decides."""
    if not integrity["pass"]:
        return "INVALID"
    return "PASS" if gate["pass"] else "FAIL"


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: NO environment is constructed, reset or stepped."""
    j2 = verify_j2_actor()
    j1 = verify_j1_lineage()
    env = build_env_config()
    # the actor weights must still carry the mask
    with (J2_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    # EVERY input layer, not just one: an unmasked mirror would read the forbidden columns.
    inputs = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and v.ndim == 2 and v.shape[1] == ACTOR_WIDTH)
    if not inputs:
        raise J3Error(f"the J2 state holds no {ACTOR_WIDTH}D input layer")
    masked_layers: dict[str, list[int]] = {}
    for key in inputs:
        W = state[key]
        z = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if z != list(MASKED_COLUMNS):
            raise J3Error(f"the J2 input layer {key} has zero columns {z}; "
                          f"expected {list(MASKED_COLUMNS)}")
        masked_layers[key] = z
    widths = sorted({int(state[k].shape[1]) for k in inputs})
    if widths != [ACTOR_WIDTH]:
        raise J3Error(f"the J2 input layers are {widths}D; expected {ACTOR_WIDTH}D")
    zero = masked_layers[inputs[0]]
    blockers: list[str] = []
    leaves = sorted(p.name for p in OUT_ROOT.iterdir()) if OUT_ROOT.is_dir() else []
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False,
                  "note": "the env factory is imported inside run() only"},
        "j2_actor": j2,
        "j1_lineage_preserved": j1,
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config, already hardened and tested",
                    "v3_keys": len(J1.V3_ENV_TO_CONFIG)},
        "actor_contract": {"width": ACTOR_WIDTH, "masked_columns": list(MASKED_COLUMNS),
                           "zero_columns_observed": zero,
                           "input_layers_verified": masked_layers,
                           "note": "one 35D actor; EVERY input layer carries the mask in J3"},
        "determinism": {"seed": ROLLOUT_SEED, "exploration_noise": "NONE",
                        "sigma": "forbidden at this stage; neither read nor set"},
        "gate": {"common": dict(J3_COMMON_GATE),
                 "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                       for k, v in J3_KINEMATIC_GATE.items()},
                 "source": "J1 common gate + v26b_b1_base_fit.declared_closed_loop_gates; "
                           "no threshold relaxed"},
        "telemetry_integrity_invariant": dict(TELEMETRY_INTEGRITY_INVARIANT),
        "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
        "no_clobber": {"scope": "PER LEAF", "root": str(OUT_ROOT.relative_to(REPO)),
                       "existing_leaves": leaves},
        "execution_requires": f"--authorized-stage {STAGE} --out-dir <fresh leaf>",
        "deterministic_semantics": dict(DETERMINISTIC_SEMANTICS),
        "full_observation_width": FULL_OBS_WIDTH,
        "forbidden_here": ["retry", "Markov", "DAgger", "sigma", "LOTO", "LOCO", "B1R1", "B1R2",
                           "promotion", "ex-novo"],
        "promotion_policy": "a J3 PASS confers NO promotion by itself; promotion remains a "
                            "separate, explicitly authorised decision",
        "generated_at_utc": _utc(),
    }


# ================================================================ deterministic inference ========

DETERMINISTIC_SEMANTICS: dict[str, Any] = {
    "seed": ROLLOUT_SEED,
    "seeded": ["numpy.random.seed(123)", "torch.manual_seed(123)", "env.reset(seed=123)"],
    "action_selection": "deterministic",
    "path": "module.forward_inference({'obs': ...}) -> get_inference_action_dist_cls()"
            ".from_logits(logits).to_deterministic().sample(); flat Box fallback = the Gaussian "
            "mean, i.e. logits[..., :dim]",
    "exploration_noise": "NONE: no noise is drawn, injected or held",
    "sigma": "forbidden at this stage. It is never set and never varied; the policy std is only "
             "RECORDED as a per-step diagnostic",
    "stepped_with": "the RAW deterministic action, exactly as rollout_eval.run does. The clipped "
                    "action is a DIAGNOSTIC mirror and is never passed to the environment",
}


class _Stack:
    """The heavy runtime, injected. Production builds it inside run(); tests inject a fake."""

    def __init__(self, *, name: str, operational: bool, torch_mod: Any,
                 load_module: Any, make_env: Any, reference_action: Any = None) -> None:
        self.name = name
        self.operational = bool(operational)
        self.torch = torch_mod
        self.load_module = load_module
        self.make_env = make_env
        self.reference_action = reference_action

    def seed(self, seed: int) -> None:
        np.random.seed(int(seed))
        self.torch.manual_seed(int(seed))


def production_stack() -> _Stack:
    """Torch / RLlib / OpenSim. Imported HERE only, so the preflight stays inert."""
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import rollout_eval as RE           # the production rollout semantics, reused read-only
    RE._load_inference_stack()
    import torch
    import env_factory
    from ray.rllib.core.rl_module.rl_module import RLModule

    return _Stack(
        name="production", operational=True, torch_mod=torch,
        load_module=lambda path: RLModule.from_checkpoint(str(Path(path).resolve())),
        make_env=env_factory.make_cmc_env,
        # the same helper rollout_eval steps with: every step is compared against it bit for bit
        reference_action=lambda m, o, shape: RE._policy_action_with_diagnostics(
            m, o, shape, action_selection="deterministic"),
    )


def deterministic_action(module: Any, obs: Any, action_shape: Any, *, torch_mod: Any
                         ) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """rollout_eval's deterministic branch: the inference distribution's deterministic sample."""
    obs_t = torch_mod.as_tensor(np.asarray(obs), dtype=torch_mod.float32).reshape(1, -1)
    with torch_mod.no_grad():
        fwd = module.forward_inference({"obs": obs_t})
    logits = fwd["action_dist_inputs"]
    try:
        dist_cls = module.get_inference_action_dist_cls()
        action_t = dist_cls.from_logits(logits).to_deterministic().sample()
        path = "inference_action_dist.to_deterministic().sample()"
    except Exception:
        # Flat Box fallback: the logits are the Gaussian mean followed by the log-std.
        action_t = logits[..., : logits.shape[-1] // 2]
        path = "flat_box_fallback_mean"
    dim = logits.shape[-1] // 2
    mean_t = logits[..., :dim]
    std_t = torch_mod.exp(logits[..., dim:])

    def arr(v: Any) -> np.ndarray:
        return v.detach().cpu().numpy().reshape(action_shape).astype(np.float32)

    return arr(action_t), arr(mean_t), arr(std_t), path


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


def _remove_leaf(leaf: Path) -> None:
    """Delete ONLY the empty-on-failure leaf this call created. Never a pinned artefact."""
    leaf = Path(leaf)
    if not leaf.is_dir():
        return
    r = leaf.resolve()
    protected = {OUT_ROOT.resolve(), HERE.resolve(), REPO.resolve(),
                 J2_LEAF.resolve(), J2_MODULE_DIR.resolve(), J1_LEAF.resolve()}
    if r in protected or r in J2_LEAF.resolve().parents or r in J1_LEAF.resolve().parents:
        raise J3Error(f"refusing to remove {leaf}: it is a root or a pinned artefact")
    shutil.rmtree(leaf, ignore_errors=True)


def _verify_runtime_contract(base: Any, module: Any, obs: Any, expected_features: tuple[str, ...],
                             cfg: Mapping[str, Any]) -> dict[str, Any]:
    """35 actor features, 84 full observation, the EXACT pinned manifest, the expected step count."""
    names = tuple(str(n) for n in getattr(base, "actor_feature_names", ()))
    full_names = tuple(str(n) for n in getattr(base, "observation_feature_names", ()))
    if names != expected_features:
        diff = [i for i, (a, b) in enumerate(zip(names, expected_features)) if a != b]
        raise J3Error(f"the runtime actor manifest differs from the J2 manifest at {diff[:8]} "
                      f"({len(names)} vs {len(expected_features)} names)")
    if len(names) != ACTOR_WIDTH:
        raise J3Error(f"the runtime exposes {len(names)} actor features, expected {ACTOR_WIDTH}")
    if int(getattr(base, "n_actor", -1)) != ACTOR_WIDTH:
        raise J3Error(f"base.n_actor is {getattr(base, 'n_actor', None)}, expected {ACTOR_WIDTH}")
    if len(full_names) != FULL_OBS_WIDTH:
        raise J3Error(f"the runtime exposes {len(full_names)} observation features, expected "
                      f"{FULL_OBS_WIDTH}")
    obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
    if obs_vec.size != FULL_OBS_WIDTH:
        raise J3Error(f"the reset observation is {obs_vec.size}D, expected {FULL_OBS_WIDTH}")
    for attr, expected in (("_n_actor", ACTOR_WIDTH), ("_n_full", FULL_OBS_WIDTH)):
        got = getattr(module, attr, None)
        if got is not None and int(got) != expected:
            raise J3Error(f"the J2 module declares {attr}={int(got)}, expected {expected}")
    steps = int(math.ceil(float(base.env_cfg.episode_duration)
                          / float(base.env_cfg.segment_duration)))
    if steps != EXPECTED_STEPS:
        raise J3Error(f"the pinned config yields {steps} steps, expected {EXPECTED_STEPS}")
    return {"actor_feature_names": list(names), "actor_feature_count": len(names),
            "observation_feature_count": len(full_names), "n_actor": int(base.n_actor),
            "expected_steps": steps,
            "manifest_matches_pinned_j2": True,
            "source": "the runtime manifest is compared name-by-name with the pinned J2 manifest"}


# ================================================================ the rollout ====================

def run(*, authorized_stage: str | None, out_dir: Path | None, stack: _Stack | None = None,
        progress: bool = True) -> dict[str, Any]:
    """ONE deterministic closed-loop rollout of the pinned J2 actor. Requires the exact token."""
    if authorized_stage != STAGE:
        raise J3Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    if out_dir is None:
        raise J3Error("--out-dir is required: the J3 leaf must be named explicitly, never guessed")
    pre = preflight()
    if pre["blockers"]:
        raise J3Error(f"preflight BLOCKED: {pre['blockers']}")

    out = Path(out_dir)
    if out.exists():
        raise J3Error(f"no-clobber: the leaf {out} already exists; choose a fresh --out-dir")

    cfg = J1.load_pinned_config()
    env_config = J1.build_full_env_config(cfg, output_dir=out)
    J1.verify_env_config(env_config, cfg)
    manifest = json.loads((J2_MODULE_DIR / "actor_feature_manifest.json").read_text())
    expected_features = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(expected_features) != ACTOR_WIDTH:
        raise J3Error(f"the pinned J2 manifest holds {len(expected_features)} names, "
                      f"expected {ACTOR_WIDTH}")

    injected = stack is not None
    stack = stack if stack is not None else production_stack()

    out.mkdir(parents=True, exist_ok=False)
    try:
        stack.seed(ROLLOUT_SEED)
        module = stack.load_module(J2_MODULE_DIR)
        env = stack.make_env(env_config)
    except BaseException:
        _remove_leaf(out)
        raise
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=ROLLOUT_SEED)
        contract = _verify_runtime_contract(base, module, obs, expected_features, cfg)
        feature_names = tuple(contract["actor_feature_names"])
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        trace: list[dict[str, Any]] = []
        knee: list[float] = []
        ankle: list[float] = []
        clipped = 0
        parity_steps = 0
        max_abs_noise = 0.0

        for step in range(1, EXPECTED_STEPS + 1):
            obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
            if obs_vec.size != FULL_OBS_WIDTH:
                raise J3Error(f"step {step}: the observation is {obs_vec.size}D, expected "
                              f"{FULL_OBS_WIDTH}")
            actor_obs = obs_vec[:ACTOR_WIDTH]
            raw, mean, std, path = deterministic_action(module, obs_vec, action_shape,
                                                        torch_mod=stack.torch)
            # determinism is a contract: the deterministic action IS the mean, exactly
            noise = np.asarray(raw, dtype=np.float64) - np.asarray(mean, dtype=np.float64)
            max_abs_noise = max(max_abs_noise, float(np.max(np.abs(noise))))
            if max_abs_noise != 0.0:
                raise J3Error(f"step {step}: the deterministic action differs from the policy mean "
                              f"by {max_abs_noise}; noise is forbidden at this stage")
            if stack.reference_action is not None:
                r_a, r_m, r_s, _ = stack.reference_action(module, obs_vec, action_shape)
                if not (np.array_equal(raw, r_a) and np.array_equal(mean, r_m)
                        and np.array_equal(std, r_s)):
                    raise J3Error(f"step {step}: the deterministic action does not match "
                                  "rollout_eval's own deterministic helper bit for bit")
                parity_steps += 1
            # DIAGNOSTIC mirror only. The environment is stepped with the RAW action.
            applied = np.clip(raw, low.reshape(raw.shape), high.reshape(raw.shape)
                              ).astype(np.float32)
            was_clipped = bool(np.any(applied != raw))
            clipped += int(was_clipped)
            pros = J1._prosthetic_state(actor_obs, feature_names)
            knee.append(pros["pros_knee_angle"])
            ankle.append(pros["pros_ankle_angle"])
            t_before = J1._finite(base.t, f"step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(raw)

            if "time" not in info:
                raise J3Error(f"step {step}: info exposes no 'time'; refusing to record a row with "
                              "an unknown timestamp")
            row: dict[str, Any] = {
                "step": step,
                "time_before": t_before,
                "time_after": J1._finite(info["time"], f"step {step}: info.time"),
                "reward": J1._finite(reward, f"step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "raw_action": np.asarray(raw, dtype=float).reshape(-1).tolist(),
                "policy_mean": np.asarray(mean, dtype=float).reshape(-1).tolist(),
                "policy_std_diagnostic": np.asarray(std, dtype=float).reshape(-1).tolist(),
                "applied_action_diagnostic": np.asarray(applied, dtype=float).reshape(-1).tolist(),
                "action_clipped_diagnostic": was_clipped,
                "action_selection_path": path,
                "stepped_with": "raw_action",
                "reward_terms": J1._jsonable(info.get("reward_terms", {}), "reward_terms"),
                J1.FSM_KEY: J1._jsonable(info.get(J1.FSM_KEY), J1.FSM_KEY),
                "prosthetic_state": pros,
            }
            for extra in ("observation", "morphology_causal_diagnostics",
                          "morphology_ledger_diagnostics", "online_grf", "online_grf_detector",
                          "observer_raw_sensor_journal"):
                if extra in info:
                    row[extra] = J1._jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: J1._jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", J1.FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            trace.append(row)
            if progress and (step % 25 == 0 or step == 1):
                print(json.dumps({"step": step, "reward": row["reward"]}), flush=True)
            if terminated or truncated:
                break

        # resolved from the recorded telemetry alone; never inferred from the step count
        end_reason = J1._resolve_end_reason(trace)
        knee_arr = np.asarray(knee, dtype=np.float64)
        ankle_arr = np.asarray(ankle, dtype=np.float64)
        # realized_noise_rms is EXPLICIT, not None: J3 injects no noise, so the realised RMS is
        # exactly zero on both channels and the receipt records that as a measured statement.
        summary = J1._summarise(trace, end_reason, clipped, [0.0, 0.0])
        if list(summary["action_noise_sigma"]) != [0.0, 0.0]:
            raise J3Error("the summariser reports a non-zero action_noise_sigma; J3 injects none")
        if list(summary["realized_noise_rms"]) != [0.0, 0.0]:
            raise J3Error("the realised action-noise RMS is not zero; J3 injects no noise")
        if max_abs_noise != 0.0:
            raise J3Error(f"the action deviated from the policy mean by {max_abs_noise}")
        gate = evaluate_gate(summary, knee_arr, ankle_arr)
        integrity = telemetry_integrity(summary)
        verdict = overall_verdict(gate, integrity)

        receipt: dict[str, Any] = {
            "schema": "v26c_j3_closed_loop.1", "stage": STAGE,
            "verdict": verdict,
            "gate_pass": bool(gate["pass"]),
            "qualification_technically_valid": bool(integrity["pass"]),
            "stack": {"name": stack.name, "operational": stack.operational,
                      "injected": injected,
                      "note": ("an INJECTED stack is a test double: the receipt is NOT operational "
                               "evidence" if injected else "the production torch/RLlib/OpenSim "
                               "stack")},
            "inputs_sha256": {
                "pinned_runtime_config": PIN_RUNTIME_CONFIG_SHA,
                "env_config": _sha_obj(env_config),
                "j2_artefacts": dict(PIN_J2),
                "j1_receipt": PIN_J1_RECEIPT,
                "j1_amendment": PIN_J1_AMENDMENT,
            },
            "j2_actor": pre["j2_actor"],
            "j1_lineage_preserved": pre["j1_lineage_preserved"],
            "runtime_contract": contract,
            "runtime_identity": {"fsm_behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION,
                                 "event_source": J1.EXPECTED_EVENT_SOURCE,
                                 "observed_versions": summary["fsm_behaviour_versions"],
                                 "observed_event_sources": summary["event_sources"]},
            "deterministic_semantics": {**DETERMINISTIC_SEMANTICS,
                                        "max_abs_action_minus_mean": max_abs_noise,
                                        "rollout_eval_parity_steps": parity_steps,
                                        "rollout_eval_parity_checked": parity_steps > 0},
            "actor_contract": pre["actor_contract"],
            "summary": summary,
            "kinematics": {"steps": int(knee_arr.size),
                           "knee_min_rad": float(knee_arr.min()),
                           "knee_max_rad": float(knee_arr.max()),
                           "ankle_min_rad": float(ankle_arr.min()),
                           "ankle_max_rad": float(ankle_arr.max()),
                           "source": "the per-step PRE-ACTION prosthetic_state, read by name from "
                                     "the pinned actor feature manifest"},
            "j3_gate": {"common": dict(J3_COMMON_GATE),
                        "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                              for k, v in J3_KINEMATIC_GATE.items()}},
            "gate": gate,
            "telemetry_integrity": integrity,
            "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
            "deployable": False,
            "promotion": "NONE",
            "next_stage_authorized": False,
            "forbidden_here": list(pre["forbidden_here"]),
            "quarantine": {
                "applies": verdict != "PASS",
                "artefacts_preserved": True,
                "retry": "FORBIDDEN without an explicit architect authorisation; this stage never "
                         "retries itself",
                "rule": "an ordinary gate FAIL is preserved as evidence and quarantined. It is not "
                        "an error to be repaired by rerunning",
            },
            "generated_at_utc": _utc(),
        }
        (out / "j3_trace.json").write_text(
            json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
        np.savez_compressed(out / "j3_kinematics.npz", knee_rad=knee_arr, ankle_rad=ankle_arr,
                            actor_feature_names=np.asarray(feature_names, dtype=str))
        receipt["outputs_sha256"] = {p.name: _sha_file(p)
                                     for p in sorted(out.iterdir()) if p.is_file()}
        (out / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
        if progress:
            print(json.dumps({"verdict": verdict, "steps": summary["steps"],
                              "end_reason": end_reason, "failed": gate["failed"]}, indent=2))
        return receipt
    finally:
        try:
            env.close()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J3 closed-loop qualification")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    r = run(authorized_stage=a.authorized_stage,
            out_dir=(Path(a.out_dir) if a.out_dir else None),
            progress=not a.no_progress)
    return 0 if r["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
