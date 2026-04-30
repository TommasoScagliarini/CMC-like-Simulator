"""
path_resolver.py
================
Centralized path resolution for model bundles and repo-relative resources.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config import SimulatorConfig


REPO_ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class ResolvedSimulatorPaths:
    repo_root: Path
    model_bundle_dir: Path
    model_path: Path
    plugin_path: Path
    kinematics_path: Path
    external_loads_path: Path
    reserve_actuators_path: Path
    healthy_dir: Path | None


def resolve_repo_path(raw: str | Path) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else REPO_ROOT / path


def normalize_cli_existing_path(raw: str | Path) -> str:
    """
    Turn an existing repo-relative CLI path into an absolute path.

    This preserves bundle-relative filenames such as ``Adjusted_SEASEA.osim`` or
    ``data/3DGaitModel2392_Kinematics_q.sto`` when no repo-level file exists.
    """
    text = str(raw)
    path = Path(text)
    if path.is_absolute():
        return str(path)
    repo_candidate = REPO_ROOT / path
    return str(repo_candidate) if repo_candidate.exists() else text


def _looks_like_explicit_path(raw: str) -> bool:
    return raw.startswith(".") or "/" in raw or "\\" in raw


def _resolve_bundle_dir(raw: str | Path) -> Path:
    bundle_dir = resolve_repo_path(raw)
    if not bundle_dir.is_dir():
        raise FileNotFoundError(f"Model bundle directory not found: {bundle_dir}")
    return bundle_dir


def _bundle_model_candidates(bundle_dir: Path) -> list[Path]:
    return sorted(
        [
            path for path in bundle_dir.iterdir()
            if path.is_file() and path.suffix.lower() == ".osim"
        ],
        key=lambda path: path.name.lower(),
    )


def _resolve_model_path(bundle_dir: Path, model_file: str | None) -> Path:
    requested = (model_file or "").strip()
    if requested:
        if _looks_like_explicit_path(requested):
            model_path = resolve_repo_path(requested)
            if model_path.is_file():
                return model_path
            raise FileNotFoundError(f"Model file not found: {model_path}")

        model_path = bundle_dir / requested
        if model_path.is_file():
            return model_path
        available = [path.name for path in _bundle_model_candidates(bundle_dir)]
        raise FileNotFoundError(
            "Model file not found in bundle: "
            f"{model_path}\n  Available .osim files: {available}"
        )

    candidates = _bundle_model_candidates(bundle_dir)
    if not candidates:
        raise FileNotFoundError(
            f"No .osim files found in model bundle: {bundle_dir}"
        )
    if len(candidates) > 1:
        names = [path.name for path in candidates]
        raise RuntimeError(
            "Multiple .osim files found in model bundle; specify model_file. "
            f"Bundle: {bundle_dir}\n  Candidates: {names}"
        )
    return candidates[0]


def _resolve_bundle_input_path(bundle_dir: Path, raw: str | Path) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else bundle_dir / path


def _resolve_healthy_dir(bundle_dir: Path) -> Path | None:
    candidates = [
        bundle_dir / "data" / "health",
        bundle_dir / "data" / "healthy",
        REPO_ROOT / "data" / "health",
        REPO_ROOT / "data" / "healthy",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return None


def resolve_simulator_paths(cfg: "SimulatorConfig") -> ResolvedSimulatorPaths:
    bundle_dir = _resolve_bundle_dir(cfg.model_bundle_dir)
    model_path = _resolve_model_path(bundle_dir, cfg.model_file)
    return ResolvedSimulatorPaths(
        repo_root=REPO_ROOT,
        model_bundle_dir=bundle_dir,
        model_path=model_path,
        plugin_path=resolve_repo_path(cfg.plugin_name),
        kinematics_path=_resolve_bundle_input_path(bundle_dir, cfg.kinematics_file),
        external_loads_path=_resolve_bundle_input_path(bundle_dir, cfg.external_loads_xml),
        reserve_actuators_path=_resolve_bundle_input_path(bundle_dir, cfg.reserve_actuators_xml),
        healthy_dir=_resolve_healthy_dir(bundle_dir),
    )
