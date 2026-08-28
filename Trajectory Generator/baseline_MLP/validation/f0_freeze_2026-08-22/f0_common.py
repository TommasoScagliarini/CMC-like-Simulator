"""Shared registry and helpers for the Phase-0 (F0) freeze/inventory protocol.

Phase 0 of the 2026-08-22 recovery plan: freeze digests, provenance matrix,
runtime-difference matrix, B0820 census, comparable baseline re-sampling.
Nothing here trains, modifies weights, or writes outside the repository.

Output versioning: every artefact lands under
``Trajectory Generator/runs/rollout/validation/f0_freeze_runs/<TAG>_<REV>``
where ``REV`` comes from ``CMC_F0_REV`` (default ``r3``). Earlier revisions are
never overwritten: r1 (wrong embedded-module selection, cumulative-counter
misreading) and r2 (generated before the second audit round: no digest
re-verification of outputs, no synchronous preflight, "reproduction" wording,
host-specific tree-digest paths) are superseded and carry a SUPERSEDED.md
marker. Artefact paths inside digests use POSIX separators (``as_posix``).

Interpreter selection (cross-platform, no hard-coded paths): the optional
``CMC_F0_PYTHON`` override is validated first; otherwise the running
interpreter (``sys.executable``) is used when it carries the simulation stack
(torch, ray, opensim, yaml); otherwise the active ``CONDA_PREFIX`` python.
The effective executable is recorded in every receipt/manifest.
"""

from __future__ import annotations

import functools
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
BASELINE_DIR = HERE.parents[1]  # Trajectory Generator/baseline_MLP
TG_DIR = BASELINE_DIR.parent  # Trajectory Generator
REPO = TG_DIR.parent
RUNS_TRAINING = TG_DIR / "runs" / "training"
RUNS_ROLLOUT = TG_DIR / "runs" / "rollout"
ROOT_VALIDATION = REPO / "validation"

F0_REV = os.environ.get("CMC_F0_REV", "r3").strip() or "r3"
SUPERSEDED_REVISIONS = {
    "r1": "first pass: embedded module taken from the empty MultiRLModule module_state.pkl; termination counters summed across iterations",
    "r2": "generated before the second audit round: no summary/trace digest re-verification, no synchronous preflight, 'historical reproduction' wording, host-specific tree-digest paths",
}
F0_TAG = "2026-08-22_F0_freeze_inventory_baseline"
OUT_ROOT = RUNS_ROLLOUT / "validation" / "f0_freeze_runs" / f"{F0_TAG}_{F0_REV}"
OUT_MANIFEST = OUT_ROOT / "manifest"
OUT_CENSUS = OUT_ROOT / "census"
OUT_ROLLOUTS = OUT_ROOT / "rollouts"
OUT_METRICS = OUT_ROOT / "metrics"
OUT_OVERLAYS = OUT_ROOT / "overlays"
OUT_DRIFT = OUT_ROOT / "drift"
OUT_GATE = OUT_ROOT / "gate"
OUT_LOGS = OUT_ROOT / "logs"

ROLLOUT_EVAL = BASELINE_DIR / "rollout_eval.py"
REQUIRED_MODULES = ("torch", "ray", "opensim", "yaml", "numpy")

EXACT_STARTS: dict[str, float] = {
    "minus020": 1.756870983805102,
    "nominal": 1.956870983805102,
    "plus020": 2.156870983805102,
}
DEVELOPMENT_SEEDS = (123, 124, 125)
SEALED_SEEDS = (126, 127, 128)

# --- frozen artefacts --------------------------------------------------------

JUN23_RUN = RUNS_TRAINING / "MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter"
JUL_PORT_RUN = (
    RUNS_TRAINING / "validation" / "controller_memory_ablation" / "2026-07-13_markov35_zero_iter_port"
)
JUL_ADAPT_RUN = RUNS_TRAINING / "target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13"
JUL_WARMUP = ROOT_VALIDATION / "critic_warmup" / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
JUL_PILOT_RUN = (
    RUNS_TRAINING
    / "validation"
    / "warm_start_h1_runs"
    / "2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50"
)
V26_IMIT_RUN = RUNS_TRAINING / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
B0820_WARMUP = ROOT_VALIDATION / "critic_warmup" / "2026-08-20_B0820_native_v26_frozen_actor_iter5"
B0820_V2_RUN = RUNS_TRAINING / "MLP_ExNovo_B0820_from_zero_50iter"
B0820_V3_RUN = RUNS_TRAINING / "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter"

CANONICAL_CFG = BASELINE_DIR / "training_exnovo_cfg.yaml"
CHAIN_SNAPSHOT_CFG = BASELINE_DIR / "experimental_configs" / "exnovo_v26_B0820_chain.yaml"
JUL_IMIT_CFG = BASELINE_DIR / "training_cfg.yaml"

GRF_PROFILE_TANGENT_V2 = (
    REPO / "online_grf_profiles" / "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
)
GRF_PROFILE_CORRECT_MAGNITUDE = (
    REPO / "online_grf_profiles" / "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)
GRF_DETECTOR_PROFILE = REPO / "online_grf_profiles" / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
BINARY_DETECTOR_PROFILE = (
    ROOT_VALIDATION
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
MORPH_PROFILE_EVENT_WARPED = (
    BASELINE_DIR / "morphology_profiles" / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
MORPH_PROFILE_LEGACY = BASELINE_DIR / "morphology_profiles" / "ab06_prosthetic_mean_std_corridor.json"
SETUP_XML = REPO / "models" / "AB06_SEASEA_Threadmill" / "AB06_SEASEA_stiff321_500_pi_setup.xml"

# RLlib full-checkpoint layout (Algorithm checkpoint): the learner module
# carries actor + critic; the MultiRLModule-level module_state.pkl is an empty
# mapping (5 bytes) and must never be used as the embedded module.
LEARNER_MODULE_REL = Path("learner_group") / "learner" / "rl_module" / "default_policy"
LEARNER_STATE_REL = Path("learner_group") / "learner" / "state.pkl"
ENV_RUNNER_STATE_REL = Path("env_runner") / "state.pkl"

# Content-addressed actor feature manifests (exact name lists the served actor
# prefix must equal). The 35-feature contract is the July markov35 manifest,
# which every 35D summary recorded in F0 (July and B0820 chains) matches
# exactly; the 39-feature contract is the V26 imitation run manifest (39-slice
# convention: runtime actor block minus the trailing sea_u_abs/sea_u_saturated
# diagnostics). Each entry pins the SHA-256 of the manifest file; a manifest
# whose digest differs is refused (see load_actor_feature_manifest).
ACTOR_MANIFEST_35 = JUL_ADAPT_RUN / "actor_feature_manifest.json"
ACTOR_MANIFEST_35_SHA256 = "c6f8602816059b5c3508c626e5ce08d3ba922710f8b48c9557af9056f1155004"
ACTOR_MANIFEST_39 = V26_IMIT_RUN / "actor_feature_manifest.json"
ACTOR_MANIFEST_39_SHA256 = "2837779ceb5953b18cf9be0e62836c8cc56d3fe871c48b869a5cedb6b0d81945"
# Controller diagnostics that must NEVER sit in the served actor block of an F0
# candidate: with --no-include-controller-diagnostic-observation the env moves
# them to the privileged suffix at the same insertion point (osim_trj_cmc_like.py,
# `priv.update(controller_diagnostics)`), so the full observation keeps width
# and order (35D: 84 = 35 + 4 + 45; 39D: 88 = 39 + 4 + 45).
CONTROLLER_DIAGNOSTIC_FEATURES = ("pros_knee_angle_sea_u_abs", "pros_knee_angle_sea_u_saturated", "pros_ankle_angle_sea_u_abs", "pros_ankle_angle_sea_u_saturated")
NO_CONTROLLER_DIAGNOSTIC_FLAG = "--no-include-controller-diagnostic-observation"

# Candidate actors for rollouts (module dir, pinned config, role).
CANDIDATES: dict[str, dict[str, Any]] = {
    "JUL_H0": {
        "module": JUL_WARMUP / "rl_module_last",
        "full_checkpoint": JUL_WARMUP / "checkpoint_last",
        "producing_run": JUL_WARMUP,
        "lineage": "july",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "july_legacy",
        "description": "H0 luglio: actor adattato target-domain (markov35 phase-aligned, sigma 0.005) + critic warmup 1 iter (13/07).",
    },
    "JUL_BEST": {
        "module": JUL_PILOT_RUN / "rl_module_best",
        "full_checkpoint": JUL_PILOT_RUN / "checkpoint_best",
        "producing_run": JUL_PILOT_RUN,
        "lineage": "july",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "july_legacy",
        "description": "Pilot 15/07 checkpoint_best (logical 24, scelto per return, fuori milestone).",
    },
    "JUL_LAST": {
        "module": JUL_PILOT_RUN / "rl_module_last",
        "full_checkpoint": JUL_PILOT_RUN / "checkpoint_last",
        "producing_run": JUL_PILOT_RUN,
        "lineage": "july",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "july_legacy",
        "description": "Pilot 15/07 checkpoint_last (logical 51).",
    },
    "V26_39D": {
        "module": V26_IMIT_RUN / "rl_module_best",
        "full_checkpoint": V26_IMIT_RUN / "checkpoint_best",
        "producing_run": V26_IMIT_RUN,
        "lineage": "b0820",
        "width": 39,
        "actor_feature_manifest": ACTOR_MANIFEST_39,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_39_SHA256,
        "native_runtime": "v26_imitation_native",
        "description": "Actor imitativo nativo V26 (iter 87, june_equiv), 39 feature (prefisso esatto del manifest content-addressed; le 4 diagnostiche sea_u_abs/saturated restano nel suffisso privilegiato), sorgente del trapianto 39->35.",
    },
    "B0820_H0": {
        "module": B0820_WARMUP / "rl_module_last",
        "full_checkpoint": B0820_WARMUP / "checkpoint_last",
        "producing_run": B0820_WARMUP,
        "lineage": "b0820",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "v2_b0820",
        "description": "H0 B0820: trapianto 39->35 (hard drop) + critic warmup 5 iter, actor congelato (20/08). Warmup eseguito con FSM v2 (chiave v3 assente).",
    },
    "B0820_V2_BEST": {
        "module": B0820_V2_RUN / "rl_module_best",
        "full_checkpoint": B0820_V2_RUN / "checkpoint_best",
        "producing_run": B0820_V2_RUN,
        "lineage": "b0820",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "v2_b0820",
        "description": "Run MLP_ExNovo_B0820_from_zero_50iter (FSM v2), checkpoint_best logical 33.",
    },
    "B0820_V2_LAST": {
        "module": B0820_V2_RUN / "rl_module_last",
        "full_checkpoint": B0820_V2_RUN / "checkpoint_last",
        "producing_run": B0820_V2_RUN,
        "lineage": "b0820",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "v2_b0820",
        "description": "Run MLP_ExNovo_B0820_from_zero_50iter (FSM v2), checkpoint_last logical 55.",
    },
    "B0820_V3_BEST": {
        "module": B0820_V3_RUN / "rl_module_best",
        "full_checkpoint": B0820_V3_RUN / "checkpoint_best",
        "producing_run": B0820_V3_RUN,
        "lineage": "b0820",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "v3_canonical",
        "description": "Run MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter (FSM v3), checkpoint_best logical 39.",
    },
    "B0820_V3_LAST": {
        "module": B0820_V3_RUN / "rl_module_last",
        "full_checkpoint": B0820_V3_RUN / "checkpoint_last",
        "producing_run": B0820_V3_RUN,
        "lineage": "b0820",
        "width": 35,
        "actor_feature_manifest": ACTOR_MANIFEST_35,
        "actor_feature_manifest_sha256": ACTOR_MANIFEST_35_SHA256,
        "native_runtime": "v3_canonical",
        "description": "Run MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter (FSM v3), checkpoint_last logical 55.",
    },
}

# Pinned (immutable) runtime configurations used by the rollout matrix.
# ``expected_observation_width`` = full observation (actor prefix + privileged
# suffix) the env must report; ``include_controller_diagnostic_observation`` =
# False for every F0 runtime (the actor never carries the 4 diagnostics).
RUNTIMES: dict[str, dict[str, Any]] = {
    "v3_canonical": {
        "config": B0820_V3_RUN / "training_cfg.resolved.yaml",
        "description": "Runtime target: resolved yaml della run B0820 fsmv3_fixedcorridor (V26 heel-qualified, FSM attore v3, reject_continue, penetrazione 20/28 mm, swing hard timeout 2.6 s, corridoio event-anchored causale peso 0.0025).",
        "extra_args": [],
        "expected_observation_width": 84,
        "include_controller_diagnostic_observation": False,
    },
    "v2_b0820": {
        "config": B0820_V2_RUN / "training_cfg.resolved.yaml",
        "description": "Runtime storico della run B0820 from_zero: identico a v3_canonical salvo FSM attore v2 (resolved yaml senza chiave -> passata esplicitamente).",
        "extra_args": ["--binary-phase-actor-fsm-version", "v2"],
        "expected_observation_width": 84,
        "include_controller_diagnostic_observation": False,
    },
    "july_legacy": {
        "config": JUL_PILOT_RUN / "training_cfg.resolved.yaml",
        "description": "Runtime storico di luglio (pilot 15/07): profilo GRF grf_correct_magnitude, nessun blocco binary_phase (detector legacy), penetrazione 15/25 mm, swing hard timeout 1.1 s, morphology weight 0.",
        "extra_args": [],
        "expected_observation_width": 84,
        "include_controller_diagnostic_observation": False,
    },
    "v26_imitation_native": {
        "config": V26_IMIT_RUN / "training_cfg.resolved.yaml",
        "description": "Runtime nativo della baseline imitativa V26 (reward imitation, gait clock prescritto); FSM attore v3 passata esplicitamente (assente nello yaml) e --no-include-controller-diagnostic-observation esplicito: il resolved yaml ha include_controller_diagnostic_observation: true (actor 43 = 39 + 4 diagnostiche), il modulo consuma 39; con il flag le 4 diagnostiche passano al suffisso privilegiato nello stesso punto d'inserzione, osservazione completa 88 con ordine invariato, prefisso actor esattamente 39.",
        "extra_args": ["--binary-phase-actor-fsm-version", "v3", NO_CONTROLLER_DIAGNOSTIC_FLAG],
        "expected_observation_width": 88,
        "include_controller_diagnostic_observation": False,
    },
}


def load_actor_feature_manifest(candidate: str) -> dict[str, Any]:
    """Names of the content-addressed actor feature manifest pinned for a
    candidate; refuses a file whose SHA-256 differs from the registry pin."""
    spec = CANDIDATES[candidate]
    path = Path(spec["actor_feature_manifest"])
    if path.is_symlink() or not path.is_file():
        raise RuntimeError(f"actor feature manifest missing or symlink for {candidate}: {rel(path)}")
    digest = sha256_file(path)
    if digest != spec["actor_feature_manifest_sha256"]:
        raise RuntimeError(f"actor feature manifest digest mismatch for {candidate}: pinned {spec['actor_feature_manifest_sha256']} vs disk {digest}")
    payload = read_json(path)
    names = payload.get("actor_feature_names") if isinstance(payload, dict) else None
    if not isinstance(names, list) or len(names) != spec["width"] or not all(isinstance(n, str) and n for n in names) or len(set(names)) != len(names):
        raise RuntimeError(f"actor feature manifest of {candidate} malformed or width != {spec['width']}")
    return {"path": rel(path), "sha256": digest, "actor_feature_names": list(names), "actor_feature_count": len(names)}


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(chunk), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(Path(path).stat().st_mtime).isoformat(timespec="seconds")


def git(*args: str) -> str:
    try:
        out = subprocess.run(
            ["git", *args], cwd=str(REPO), capture_output=True, text=True, check=False
        )
        return out.stdout.strip()
    except OSError as exc:  # pragma: no cover
        return f"<git unavailable: {exc}>"


def git_snapshot() -> dict[str, Any]:
    return {
        "head": git("rev-parse", "HEAD"),
        "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
        "status_porcelain": git("status", "--porcelain").splitlines(),
        "describe": git("log", "-1", "--format=%h %ad %s", "--date=iso"),
    }


def git_blob_hash(path: Path) -> str | None:
    rel_path = os.path.relpath(path, REPO)
    tracked = git("ls-files", "--error-unmatch", rel_path)
    if not tracked:
        return None
    return git("hash-object", rel_path) or None


def env_snapshot() -> dict[str, Any]:
    return {
        "python_executable": sys.executable,
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.machine(),
        "os_name": os.name,
        "cpu_count": os.cpu_count(),
        "conda_prefix": os.environ.get("CONDA_PREFIX"),
        "conda_default_env": os.environ.get("CONDA_DEFAULT_ENV"),
    }


# --- interpreter selection ---------------------------------------------------


def _validate_interpreter(candidate: Path) -> dict[str, Any]:
    """Run the candidate interpreter and check it imports the simulation stack."""
    result: dict[str, Any] = {"path": str(candidate), "exists": candidate.is_file()}
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        result["valid"] = False
        result["reason"] = "not an executable file"
        return result
    code = (
        "import importlib, json, sys\n"
        f"mods = {list(REQUIRED_MODULES)!r}\n"
        "out = {'python_version': sys.version.split()[0], 'executable': sys.executable, 'modules': {}}\n"
        "for m in mods:\n"
        "    try:\n"
        "        mod = importlib.import_module(m)\n"
        "        out['modules'][m] = getattr(mod, '__version__', 'ok')\n"
        "    except Exception as exc:\n"
        "        out['modules'][m] = 'MISSING: ' + type(exc).__name__\n"
        "print(json.dumps(out))\n"
    )
    try:
        proc = subprocess.run([str(candidate), "-c", code], capture_output=True, text=True, timeout=300, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        result["valid"] = False
        result["reason"] = f"probe failed: {exc}"
        return result
    try:
        payload = json.loads(proc.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        result["valid"] = False
        result["reason"] = f"probe returned no JSON (rc={proc.returncode}): {proc.stderr[-300:]}"
        return result
    missing = [m for m, v in payload["modules"].items() if str(v).startswith("MISSING")]
    result.update(payload)
    result["valid"] = not missing
    result["reason"] = "ok" if not missing else f"missing modules: {missing}"
    return result


@functools.lru_cache(maxsize=1)
def select_python() -> dict[str, Any]:
    """Pick the interpreter for rollout subprocesses, cross-platform.

    Order: ``CMC_F0_PYTHON`` override (must validate, otherwise hard error so a
    wrong override never silently falls back) -> ``sys.executable`` ->
    ``$CONDA_PREFIX`` python. Returns a record with the validation evidence.
    """
    attempts: list[dict[str, Any]] = []
    override = os.environ.get("CMC_F0_PYTHON", "").strip()
    if override:
        rec = _validate_interpreter(Path(override).expanduser())
        rec["source"] = "CMC_F0_PYTHON"
        attempts.append(rec)
        if not rec["valid"]:
            raise RuntimeError(f"CMC_F0_PYTHON override is not a valid simulation interpreter: {rec}")
        return {"selected": rec["path"], "source": "CMC_F0_PYTHON", "validation": rec, "attempts": attempts}
    rec = _validate_interpreter(Path(sys.executable))
    rec["source"] = "sys.executable"
    attempts.append(rec)
    if rec["valid"]:
        return {"selected": rec["path"], "source": "sys.executable", "validation": rec, "attempts": attempts}
    prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if prefix:
        candidate = Path(prefix) / ("python.exe" if os.name == "nt" else "bin/python")
        rec = _validate_interpreter(candidate)
        rec["source"] = "CONDA_PREFIX"
        attempts.append(rec)
        if rec["valid"]:
            return {"selected": rec["path"], "source": "CONDA_PREFIX", "validation": rec, "attempts": attempts}
    raise RuntimeError(
        "no valid simulation interpreter found (need " + ", ".join(REQUIRED_MODULES) + "); "
        "run this driver inside the simulation conda env or set CMC_F0_PYTHON. Attempts: "
        + json.dumps(attempts)
    )


# --- small utilities -----------------------------------------------------------


def rel(path: Path | str) -> str:
    """Repository-relative path with POSIX separators (portable across hosts)."""
    try:
        return Path(os.path.relpath(Path(path), REPO)).as_posix()
    except ValueError:
        return Path(path).as_posix()


def write_json(path: Path, payload: Any, *, clobber: bool = False) -> Path:
    path = Path(path)
    if path.exists() and not clobber:
        raise FileExistsError(f"refusing to overwrite existing artefact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False, default=str), encoding="utf-8")
    return path


def write_text(path: Path, text: str, *, clobber: bool = False) -> Path:
    path = Path(path)
    if path.exists() and not clobber:
        raise FileExistsError(f"refusing to overwrite existing artefact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def read_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def flatten(tree: Any, prefix: str = "") -> dict[str, Any]:
    """Flatten nested mappings into dotted keys (lists kept as leaves)."""
    out: dict[str, Any] = {}
    if isinstance(tree, dict):
        for key, value in tree.items():
            dotted = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten(value, dotted))
    else:
        out[prefix] = tree
    return out


def md_table(headers: list[str], rows: list[list[Any]]) -> str:
    def cell(value: Any) -> str:
        if isinstance(value, float):
            return f"{value:.6g}"
        return str(value).replace("|", "\\|").replace("\n", " ")

    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(cell(v) for v in row) + " |")
    return "\n".join(lines)


def ensure_out_dirs() -> None:
    for directory in (
        OUT_ROOT,
        OUT_MANIFEST,
        OUT_CENSUS,
        OUT_ROLLOUTS,
        OUT_METRICS,
        OUT_OVERLAYS,
        OUT_DRIFT,
        OUT_GATE,
        OUT_LOGS,
    ):
        directory.mkdir(parents=True, exist_ok=True)
