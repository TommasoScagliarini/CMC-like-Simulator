"""Shared registry and helpers for Phase 1 (F1) — causal ablation of the
39->35 hard-drop and of the stochastic scale (plan of 2026-08-22, section F1).

Stage S1 (this delivery): preregistered protocol, tooling, synthetic tests and
a dry-run job matrix.  Nothing here runs a rollout, a simulation or a fit.

Reuse policy: the F0 modules (``f0_common``, ``f0_closure``, ``f0_artifacts``,
``f0_rollout_matrix``, ``f0_matrix_analysis``, ``f0_overlays``,
``f0_actor_drift``) are imported as *libraries* through their own module names
(same ``sys.path`` seam F0 uses) and are never modified.  No F0 global is
rebound; every F1 behaviour lives in this directory.

Output root (no-clobber, revisioned like F0):
``Trajectory Generator/runs/rollout/validation/f1_ablation_runs/<TAG>_<REV>``
with ``REV`` from ``CMC_F1_REV`` (default ``r1``).  Paths inside artefacts use
POSIX separators; no absolute path is hard-coded (cross-platform).
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent  # baseline_MLP/validation
F0_DIR = VALIDATION_DIR / "f0_freeze_2026-08-22"
if str(F0_DIR) not in sys.path:
    sys.path.insert(0, str(F0_DIR))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f0_common as C  # noqa: E402  (F0 library, immutable)

BASELINE_DIR = C.BASELINE_DIR
TG_DIR = C.TG_DIR
REPO = C.REPO

F1_REV = os.environ.get("CMC_F1_REV", "r1").strip() or "r1"
F1_TAG = "2026-08-23_F1_ablation_39to35_sigma"
OUT_ROOT = C.RUNS_ROLLOUT / "validation" / "f1_ablation_runs" / f"{F1_TAG}_{F1_REV}"
OUT_MANIFEST = OUT_ROOT / "manifest"
OUT_ROLLOUTS = OUT_ROOT / "rollouts"
OUT_DERIVED = OUT_ROOT / "derived_modules"
OUT_DATASETS = OUT_ROOT / "datasets"
OUT_REFIT = OUT_ROOT / "refit"
OUT_METRICS = OUT_ROOT / "metrics"
OUT_GATE = OUT_ROOT / "gate"
OUT_LOGS = OUT_ROOT / "logs"

PROTOCOL_JSON = HERE / "f1_protocol.json"
PROTOCOL_MD = HERE / "PROTOCOL.md"
ROLLOUT_EVAL = C.ROLLOUT_EVAL
AISO_DRIVER = HERE / "f1_rollout_aiso.py"

# F0 pinned roots reused read-only (verified F0 outputs, immutable).
F0_OUT_ROOT = C.OUT_ROOT
F0_ROLLOUTS = C.OUT_ROLLOUTS
F0_ANALYSIS_JSON = F0_OUT_ROOT / "metrics" / "f0_matrix_analysis_20260823_011338.json"
F0_ANALYSIS_SHA256 = "0ac942432d0505731b93e6322ac337edf91ab7dca8c9c8e9c1cb122089148987"

EXACT_STARTS = dict(C.EXACT_STARTS)
DEVELOPMENT_SEEDS = tuple(C.DEVELOPMENT_SEEDS)
SEALED_SEEDS = tuple(C.SEALED_SEEDS)
STARTS = ("minus020", "nominal", "plus020")

SIGMA_CONSTANT = 0.005
ACTION_DIM = 2
ENV_ACTOR_WIDTH = 35
MODULE_WIDTH_39 = 39
FULL_OBS_WIDTH_35 = 84
FULL_OBS_WIDTH_39 = 88

# Runtime shared by every isometric F1 job (B's runtime, pinned in F0).
TARGET_RUNTIME = "v3_canonical"
RUNTIME_CONFIG = C.RUNTIMES[TARGET_RUNTIME]["config"]
RUNTIME_CONFIG_SHA256 = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"
V26_NATIVE_CONFIG = C.RUNTIMES["v26_imitation_native"]["config"]
V26_NATIVE_CONFIG_SHA256 = "aaf84cfd5be75672d003af6139cb0e977667f2b207fbfda91971a145e4586086"

# Config keys that must be identical between the V26 native config and the v3
# canonical config for the prescribed-target reconstruction to reproduce the
# V26 env values bit-exactly (checked by the driver, recorded in the manifest).
TARGET_RECONSTRUCTION_KEYS = (
    # keys absent from both resolved configs fall back to the CMCEnvConfig
    # defaults (recorded as "<absent>"; equality of the absence is what matters)
    "simulation.imitation_sound_coords",
    "simulation.imitation_phase_shifts",
    "simulation.imitation_phase_shift",
    "simulation.imitation_phase_samples",
    "simulation.gait_clock_side",
    "simulation.gait_clock_phase_offset",
    "simulation.setup_xml",
    "simulation.segment_duration",
    "simulation.episode_duration",
    "simulation.policy_knots",
    "model.asymmetric_actor_critic",
)
# Keys that are EXPECTED to differ between the V26 native runtime and B's v3
# runtime (A_iso deliberately inherits B's values): recorded, never equalised.
RUNTIME_DIVERGENCE_KEYS = (
    "simulation.gait_clock_enable",
    "simulation.pros_knee_target_slew_rate_limit_rad_s",
    "simulation.pros_ankle_target_slew_rate_limit_rad_s",
    "simulation.include_controller_diagnostic_observation",
    "simulation.episode_start_offset_s",
    "reward.reward_mode",
    "reward.morphology_weight",
    "grf.binary_phase_actor_fsm_version",
)

ADAPTER_MODES = ("aiso4", "aiso6clk", "passthrough")
DRIVERS = ("rollout_eval", "f1_rollout_aiso")

# --- candidates ---------------------------------------------------------------
# ``env_width``      width of the actor block emitted by the env under the job runtime
# ``module_width``   width consumed by the RLModule (39 only for the V26 actor)
# ``driver``         rollout_eval (production, unchanged) or f1_rollout_aiso (validation-only)
# ``adapter``        adapter mode for f1_rollout_aiso jobs
# ``source``         "frozen" (F0-pinned module), "derived_sigma" (stage 1, sigma tool),
#                    "refit" (stage 2 output), "reuse_f0" (no new rollout; F0 job dirs)
CANDIDATES: dict[str, dict[str, Any]] = {
    "A_ISO39_V3": {
        "role": "A_iso",
        "comparison_class": "isometric_privileged_39D",
        "module": C.CANDIDATES["V26_39D"]["module"],
        "module_state_sha256_pin": None,
        "source": "frozen",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": MODULE_WIDTH_39,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_39,
        "module_manifest_sha256": C.ACTOR_MANIFEST_39_SHA256,
        "driver": "f1_rollout_aiso",
        "adapter": "aiso4",
        "sigma": "native_state_dependent",
        "description": "V26 native 39D actor fed by the validation-only adapter under B's runtime: the 4 prescribed healthy_* targets are inserted at indices 2:6; obs[0:2] stay as B sees them (clock disabled -> (0,1)). Same env/reward/reset/physics/slew/OOB/terminations/starts as B.",
    },
    "A_ISO39_V3_S005": {
        "role": "A_iso_sigma005",
        "comparison_class": "isometric_privileged_39D",
        "module": None,  # derived at stage 1: OUT_DERIVED/V26_39D_sigma0005/rl_module
        "derived_from": "V26_39D",
        "source": "derived_sigma",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": MODULE_WIDTH_39,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_39,
        "module_manifest_sha256": C.ACTOR_MANIFEST_39_SHA256,
        "driver": "f1_rollout_aiso",
        "adapter": "aiso4",
        "sigma": SIGMA_CONSTANT,
        "description": "A_iso with the log-std head forced to constant sigma 0.005 (rows zero, bias ln 0.005); mean bit-identical to V26. Used for the sigma-matched stochastic comparison with D and C.",
    },
    "B": {
        "role": "B",
        "comparison_class": "isometric_comparable",
        "module": C.CANDIDATES["B0820_H0"]["module"],
        "f0_candidate": "B0820_H0",
        "source": "frozen",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": ENV_ACTOR_WIDTH,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_35,
        "module_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "driver": "rollout_eval",
        "adapter": None,
        "sigma": "native_state_dependent",
        "description": "Deployable 35D hard-drop actor (B0820_H0 rl_module_last: transplant 39->35 actor bit-exact, critic warmup with frozen actor). Deterministic and native stochastic.",
    },
    "C": {
        "role": "C",
        "comparison_class": "isometric_comparable",
        "module": None,  # derived at stage 1: OUT_DERIVED/B_sigma0005/rl_module
        "derived_from": "B0820_H0",
        "source": "derived_sigma",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": ENV_ACTOR_WIDTH,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_35,
        "module_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "driver": "rollout_eval",
        "adapter": None,
        "sigma": SIGMA_CONSTANT,
        "description": "B with the log-std head forced to constant sigma 0.005; mean bit-identical to B, hence C-det must be bit-identical to B-det.",
    },
    "D": {
        "role": "D",
        "comparison_class": "isometric_comparable",
        "module": None,  # stage 2 output: OUT_REFIT/<stamp>/rl_module_refit
        "derived_from": "B0820_H0",
        "source": "refit",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": ENV_ACTOR_WIDTH,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_35,
        "module_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "driver": "rollout_eval",
        "adapter": None,
        "sigma": SIGMA_CONSTANT,
        "description": "Minimal supervised refit of the 35D mean (init B) on frozen B trajectories; teacher = V26 deterministic mean on the same-state/time obs39; sigma 0.005 constant; no DAgger, no new collection after the fit.",
    },
    "A_ISO39CLK_V3": {
        "role": "A_iso_clock_diagnostic",
        "comparison_class": "diagnostic_privileged_clock",
        "module": C.CANDIDATES["V26_39D"]["module"],
        "source": "frozen",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": MODULE_WIDTH_39,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_39,
        "module_manifest_sha256": C.ACTOR_MANIFEST_39_SHA256,
        "driver": "f1_rollout_aiso",
        "adapter": "aiso6clk",
        "sigma": "native_state_dependent",
        "optional_diagnostic": True,
        "description": "DIAGNOSTIC ONLY (not a gate): A_iso plus the prescribed sound-side clock restored in obs[0:2] (6/6 lost signals). Isolates the bootstrap-deadlock mechanism of the online clock if A_iso (4/6) fails. Projection rule differs (obs[0:2] intentionally not equal to B).",
    },
    "PASSTHROUGH_B": {
        "role": "integrity_passthrough",
        "comparison_class": "integrity_control",
        "module": C.CANDIDATES["B0820_H0"]["module"],
        "source": "frozen",
        "lineage": "b0820",
        "env_width": ENV_ACTOR_WIDTH,
        "module_width": ENV_ACTOR_WIDTH,
        "env_manifest": C.ACTOR_MANIFEST_35,
        "env_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "module_manifest": C.ACTOR_MANIFEST_35,
        "module_manifest_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "driver": "f1_rollout_aiso",
        "adapter": "passthrough",
        "sigma": "native_state_dependent",
        "description": "G1 control: f1_rollout_aiso with the adapter in passthrough mode on the B module must reproduce the rollout_eval B-det job bit-exactly (proves the hook path is inert).",
    },
}

# Non-isometric controls reused from verified F0 outputs (no new rollouts).
REUSE_F0: dict[str, dict[str, Any]] = {
    "A_NATIVE_39D": {
        "role": "A_native",
        "comparison_class": "compatibility_control_39D",
        "f0_candidate": "V26_39D",
        "f0_family": "ctrl39",
        "f0_runtime": "v26_imitation_native",
        "description": "V26 39D actor under its own imitation runtime (F0 ctrl39 jobs): non-isometric control (slew 0, prescribed clock, imitation reward).",
    },
    "E": {
        "role": "E",
        "comparison_class": "historical_control",
        "f0_candidate": "JUL_H0",
        "f0_family": ("det", "stoch"),
        "f0_runtime": "v3_canonical",
        "description": "July H0 (constant sigma 0.005 native) under the v3 runtime (F0 det+stoch jobs): historical non-isometric control.",
    },
    "B_F0_REFERENCE": {
        "role": "B_cross_phase_reference",
        "comparison_class": "isometric_comparable",
        "f0_candidate": "B0820_H0",
        "f0_family": ("det", "stoch"),
        "f0_runtime": "v3_canonical",
        "description": "F0 B0820_H0 det/stoch jobs used only for the cross-phase bit-exact reproducibility check of the F1 B jobs (G1).",
    },
}

FAMILIES = (
    "aiso_det", "aiso_stoch_s005", "b_det", "b_stoch_native", "c_det", "c_stoch_s005",
    "d_det", "d_stoch_s005", "passthrough", "aiso_clk_diag",
)
STAGES = {
    0: "S1 protocol + dry-run (no execution)",
    1: "derive sigma modules (C, A_ISO39_V3_S005) and run A_iso/B/C/passthrough/diagnostic rollouts",
    2: "build D dataset from F1 B-stoch traces, refit D, verify D module",
    3: "run D rollouts",
    4: "analysis, gates G1-G5, report",
}

JOB_TIMEOUT_ARGS = [
    "--no-progress", "--run-timeout-s", "7200", "--stall-timeout-s", "1500",
    "--step-timeout-s", "900", "--startup-timeout-s", "900",
]


# --- protocol ------------------------------------------------------------------


def load_protocol() -> dict[str, Any]:
    payload = json.loads(PROTOCOL_JSON.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("protocol_id") != "F1-S1-ablation-39to35-sigma-v1":
        raise RuntimeError("f1_protocol.json missing or wrong protocol_id")
    return payload


def protocol_digests() -> dict[str, str]:
    return {
        "f1_protocol.json": C.sha256_file(PROTOCOL_JSON),
        "PROTOCOL.md": C.sha256_file(PROTOCOL_MD) if PROTOCOL_MD.is_file() else None,
    }


def entropy_diag_gauss(log_std: list[float] | tuple[float, ...]) -> float:
    """Entropy of a diagonal Gaussian with the given per-dimension log-std."""
    return float(sum(0.5 * math.log(2.0 * math.pi * math.e) + float(v) for v in log_std))


def ensure_out_dirs() -> None:
    for directory in (OUT_ROOT, OUT_MANIFEST, OUT_ROLLOUTS, OUT_DERIVED, OUT_DATASETS, OUT_REFIT, OUT_METRICS, OUT_GATE, OUT_LOGS):
        directory.mkdir(parents=True, exist_ok=True)


def assert_development_seed(seed: int) -> int:
    seed = int(seed)
    if seed in SEALED_SEEDS:
        raise RuntimeError(f"sealed seed {seed} must never be used in F1")
    if seed not in DEVELOPMENT_SEEDS:
        raise RuntimeError(f"seed {seed} is not a development seed {DEVELOPMENT_SEEDS}")
    return seed


def flat_config_values(cfg: dict[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    flat = C.flatten(cfg)
    return {k: flat.get(k, "<absent>") for k in keys}


def reconstruction_config_equality() -> dict[str, Any]:
    """Compare the V26 native config with the v3 canonical config on the keys the
    prescribed-target reconstruction depends on (fail-closed in the driver)."""
    cfg26 = C.load_yaml(V26_NATIVE_CONFIG)
    cfg3 = C.load_yaml(RUNTIME_CONFIG)
    v26 = flat_config_values(cfg26, TARGET_RECONSTRUCTION_KEYS)
    v3 = flat_config_values(cfg3, TARGET_RECONSTRUCTION_KEYS)
    mismatches = {k: {"v26": v26[k], "v3": v3[k]} for k in TARGET_RECONSTRUCTION_KEYS if v26[k] != v3[k]}
    divergence = {
        k: {"v26": v, "v3": w}
        for k, v, w in (
            (k, flat_config_values(cfg26, (k,))[k], flat_config_values(cfg3, (k,))[k])
            for k in RUNTIME_DIVERGENCE_KEYS
        )
    }
    return {
        "keys": list(TARGET_RECONSTRUCTION_KEYS),
        "v26_native": v26,
        "v3_canonical": v3,
        "equal": not mismatches,
        "mismatches": mismatches,
        "expected_runtime_divergence_inherited_by_A_iso_from_B": divergence,
        "config_sha256": {"v26_native": C.sha256_file(V26_NATIVE_CONFIG), "v3_canonical": C.sha256_file(RUNTIME_CONFIG)},
    }
