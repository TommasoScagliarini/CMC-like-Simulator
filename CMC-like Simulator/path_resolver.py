"""
path_resolver.py
================
Centralized path resolution for model bundles and repo-relative resources.
"""

from __future__ import annotations

import os
import posixpath
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config import SimulatorConfig


SIMULATOR_DIR = Path(__file__).resolve().parent
REPO_ROOT = SIMULATOR_DIR.parent
SHARED_GEOMETRY_DIR = REPO_ROOT / "models" / "Geometry"
SEA_PLUGIN_DIR = REPO_ROOT / "plugins" / "SEA"
ONLINE_GRF_PLUGIN_DIR = REPO_ROOT / "plugins" / "online_grf_contact"
ONLINE_GRF_DIR = REPO_ROOT / "online_GRF"
ONLINE_GRF_PROFILES_DIR = ONLINE_GRF_DIR / "profiles"
ONLINE_GRF_PROFILE_FILENAMES = frozenset((
    "AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json",
    "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
))
BINARY_DETECTOR_PROFILE_PATH = (
    ONLINE_GRF_DIR / "binary_detector" / "operational"
    / "heel_toe_v25.json"
)
LEGACY_BINARY_DETECTOR_PROFILE = (
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)
POLICY_TRANSFER_DIR = (
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "imitation_to_exnovo"
)
POLICY_TRANSFER_PROCEDURES_DIR = POLICY_TRANSFER_DIR / "procedures"
POLICY_TRANSFER_PROCEDURES = frozenset((
    "f0_freeze_2026-08-22", "f1_ablation_2026-08-23",
    "f2r_bridge_2026-08-23", "v26b_bridge_2026-08-24",
    "v26c_july_replica_2026-08-26",
))
LEGACY_POLICY_TRANSFER_DIR = "Trajectory Generator/baseline_MLP/validation"
MORPHOLOGY_PROFILES_DIR = (
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "morphology_corridor" / "profiles"
)
MORPHOLOGY_PROFILE_FILENAMES = frozenset((
    "ab06_prosthetic_mean_std_corridor.json",
    "ab06_prosthetic_event_warped_mean_std_corridor.json",
))

# These names identify the current implementations. Older SEA binaries have
# different dynamics and must never be redirected to the current PI plugin.
PLUGIN_DIRECTORIES = {
    "SEA_Plugin_BlackBox_mCMC_impedence_ff": "SEA",
    "OnlineGRFContact": "online_grf_contact",
}

# Only top-level model bundles and shared Geometry were reorganized. Archived
# workspaces under validation/.../models must keep their own paths and bytes.
LEGACY_MODEL_DIRECTORIES = {
    f"{subject}{suffix}": f"{subject}/{subject}{suffix}"
    for subject in ("AB06", "AB07", "AB08")
    for suffix in ("-raw", "_SEASEA-raw", "_SEASEA_Threadmill")
}
LEGACY_MODEL_DIRECTORIES["SEASEA - whealthy data"] = "SEASEA_healthy"
MODEL_FAMILIES = frozenset(("AB06", "AB07", "AB08", "SEASEA", "SEASEA_healthy"))


def remap_morphology_profile_path(raw: str | Path) -> Path | None:
    """Locate the two moved profiles without changing saved reward configs.

    Only known baseline/repo namespaces and recognized checkout addresses are
    aliases. Existing external files and nested snapshots keep their identity.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    parts = PurePosixPath(text).parts
    absolute = path.is_absolute() or PureWindowsPath(text).is_absolute()
    baseline = ("Trajectory Generator", "baseline_MLP")
    namespaces = {
        ("morphology_profiles",),
        ("morphology_corridor", "profiles"),
    }
    relative_parts: tuple[str, ...] | None = parts if not absolute else None
    if absolute:
        try:
            relative_parts = path.relative_to(REPO_ROOT).parts
        except ValueError:
            if path.is_absolute() and path.exists():
                return None
            checkout_names = {
                REPO_ROOT.name.casefold(), "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if part.casefold() not in checkout_names:
                    continue
                if any(p.casefold() in {"validation", "runs", "snapshot", "snapshots"}
                       for p in parts[:index]):
                    continue
                candidate = parts[index + 1:]
                if candidate[:2] == baseline:
                    relative_parts = candidate
                    break
    if relative_parts is None:
        return None
    if relative_parts[:2] == baseline:
        relative_parts = relative_parts[2:]
    if (not relative_parts
            or relative_parts[:-1] not in namespaces
            or relative_parts[-1] not in MORPHOLOGY_PROFILE_FILENAMES):
        return None
    return MORPHOLOGY_PROFILES_DIR / relative_parts[-1]


def remap_policy_transfer_path(raw: str | Path) -> Path | None:
    """Resolve the renamed policy-transfer workspace without rewriting history.

    Accept its exact repo-relative namespace and saved checkout addresses on
    macOS/Windows. Existing external resources and nested source snapshots keep
    their own identity. Root ``validation/`` and other workspaces are unrelated.
    The five retained procedures also accept their former direct-child paths;
    shared adaptation and utility modules stay at the workspace root. Missing
    historical inputs remain missing at their relocated address.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    absolute = path.is_absolute() or PureWindowsPath(text).is_absolute()
    parts = PurePosixPath(text).parts
    namespaces = {
        PurePosixPath(LEGACY_POLICY_TRANSFER_DIR).parts,
        POLICY_TRANSFER_DIR.relative_to(REPO_ROOT).parts,
    }
    relative_parts: tuple[str, ...] | None = None
    if not absolute:
        relative_parts = parts
    else:
        try:
            relative_parts = path.relative_to(REPO_ROOT).parts
        except ValueError:
            if path.is_absolute() and path.exists():
                return None
            checkout_names = {
                REPO_ROOT.name.casefold(), "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if part.casefold() not in checkout_names:
                    continue
                if any(p.casefold() in {"validation", "runs", "snapshot", "snapshots"}
                       for p in parts[:index]):
                    continue
                candidate = parts[index + 1:]
                if candidate[:3] in namespaces:
                    relative_parts = candidate
                    break
    if relative_parts is None or relative_parts[:3] not in namespaces:
        return None
    tail = relative_parts[3:]
    if tail and tail[0] in POLICY_TRANSFER_PROCEDURES:
        return POLICY_TRANSFER_PROCEDURES_DIR.joinpath(*tail)
    return POLICY_TRANSFER_DIR.joinpath(*tail)


def remap_model_path(raw: str | Path) -> Path | None:
    """Resolve known model/geometry paths, including saved checkout paths.

    Return None for unrelated paths. Existing external files and nested archived
    model namespaces are deliberately not redirected to this checkout.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    windows_absolute = PureWindowsPath(text).is_absolute()
    absolute = path.is_absolute() or windows_absolute
    if path.is_absolute() and path.exists():
        try:
            path.relative_to(REPO_ROOT)
        except ValueError:
            return None

    parts = PurePosixPath(text).parts
    model_parts: tuple[str, ...] | None = None
    if not absolute:
        if len(parts) >= 2 and parts[0] == "models":
            model_parts = parts[1:]
        elif parts and parts[0] == "Geometry":
            model_parts = parts
    else:
        # Handle this workspace first: a nested validation copy must not fall
        # through to the foreign-checkout compatibility rule below.
        try:
            relative = path.relative_to(REPO_ROOT)
        except ValueError:
            relative = None
        if relative is not None:
            if len(relative.parts) >= 2 and relative.parts[0] == "models":
                model_parts = relative.parts[1:]
            elif relative.parts and relative.parts[0] == "Geometry":
                model_parts = relative.parts
        else:
            checkout_names = {
                REPO_ROOT.name.casefold(),
                "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if (part in {"models", "Geometry"} and index > 0
                        and parts[index - 1].casefold() in checkout_names
                        and not any(p.casefold() in {"validation", "runs"}
                                    for p in parts[:index - 1])):
                    model_parts = parts[index + 1:] if part == "models" else parts[index:]
                    break
    if not model_parts:
        return None
    first, *tail = model_parts
    relocated = LEGACY_MODEL_DIRECTORIES.get(first)
    if relocated is not None:
        return REPO_ROOT / "models" / relocated / Path(*tail)
    if first in MODEL_FAMILIES or first == "Geometry":
        return REPO_ROOT / "models" / Path(*model_parts)
    return None


def _current_plugin_stem(filename: str) -> str | None:
    stem = filename
    for extension in (".dll", ".dylib", ".so"):
        if stem.lower().endswith(extension):
            stem = stem[:-len(extension)]
            break
    if stem.startswith("lib") and stem[3:] in PLUGIN_DIRECTORIES:
        stem = stem[3:]
    return stem if stem in PLUGIN_DIRECTORIES else None


def _plugin_variant_exists(path: Path, stem: str) -> bool:
    return path.exists() or any(
        (path.parent / filename).is_file()
        for filename in (stem + ".dll", stem + ".dylib", stem + ".so",
                         "lib" + stem + ".dylib", "lib" + stem + ".so")
    )


def remap_plugin_path(raw: str | Path) -> Path | None:
    """Map the two current plugins into their dedicated directories.

    Preserve the requested filename or extension-free OpenSim basename. Only
    the top-level plugins namespace of this or a recognized former checkout is
    relocated; existing external libraries and frozen nested copies keep their
    own paths. No removed SEA implementation is substituted by the current PI.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    stem = _current_plugin_stem(path.name)
    if stem is None:
        return None
    absolute = path.is_absolute() or PureWindowsPath(text).is_absolute()
    parts = PurePosixPath(text).parts
    plugin_parts: tuple[str, ...] | None = None
    if not absolute:
        if len(parts) >= 2 and parts[0] == "plugins":
            plugin_parts = parts[1:]
    else:
        try:
            relative = path.relative_to(REPO_ROOT)
        except ValueError:
            relative = None
            if path.is_absolute() and _plugin_variant_exists(path, stem):
                return None
        if relative is not None:
            # Do not reinterpret this checkout's validation/.../plugins copies.
            if len(relative.parts) >= 2 and relative.parts[0] == "plugins":
                plugin_parts = relative.parts[1:]
        else:
            checkout_names = {
                REPO_ROOT.name.casefold(), "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if (part == "plugins" and index > 0
                        and parts[index - 1].casefold() in checkout_names
                        and not any(p.casefold() in {"validation", "runs"}
                                    for p in parts[:index - 1])):
                    plugin_parts = parts[index + 1:]
                    break
    directory = PLUGIN_DIRECTORIES[stem]
    if plugin_parts is None or not (
        len(plugin_parts) == 1
        or (len(plugin_parts) == 2 and plugin_parts[0] == directory)
    ):
        return None
    return REPO_ROOT / "plugins" / directory / plugin_parts[-1]


def remap_online_grf_path(raw: str | Path) -> Path | None:
    """Locate moved GRF resources without changing saved configuration bytes.

    Only the top-level old/current namespaces and recognized checkout addresses
    are aliases. Existing external files and nested snapshots keep their paths;
    retired profile names are never substituted with an operational profile.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    parts = PurePosixPath(text).parts
    absolute = path.is_absolute() or PureWindowsPath(text).is_absolute()
    namespaces = {"online_grf_profiles", "online_GRF"}
    relative_parts: tuple[str, ...] | None = parts if not absolute else None
    if absolute:
        try:
            relative_parts = path.relative_to(REPO_ROOT).parts
        except ValueError:
            if path.is_absolute() and path.exists():
                return None
            checkout_names = {
                REPO_ROOT.name.casefold(), "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if part.casefold() not in checkout_names:
                    continue
                if any(p.casefold() in {"validation", "runs", "snapshot", "snapshots"}
                       for p in parts[:index]):
                    continue
                candidate = parts[index + 1:]
                if candidate and candidate[0] in namespaces:
                    relative_parts = candidate
                    break
    if not relative_parts or relative_parts[0] not in namespaces:
        return None
    namespace, *tail = relative_parts
    if not tail:
        return ONLINE_GRF_DIR
    if namespace == "online_grf_profiles":
        if len(tail) == 1 and tail[0] in ONLINE_GRF_PROFILE_FILENAMES:
            return ONLINE_GRF_PROFILES_DIR / tail[0]
        if tail[0] not in {
            "binary_detector", "calibration", "contact", "detector", "validation",
        }:
            return None
    elif tail[0] not in {
        "profiles", "binary_detector", "calibration", "contact", "detector",
        "validation", "sources",
    }:
        return None
    return ONLINE_GRF_DIR.joinpath(*tail)


def remap_binary_detector_profile_path(raw: str | Path) -> Path | None:
    """Resolve the relocated operational V25 geometry, including saved configs.

    Match only its exact former/current repo-relative address. Existing external
    files and nested source snapshots keep their own identity; other detector
    candidates must never be substituted with the operational geometry.
    """
    text = posixpath.normpath(os.fspath(raw).replace("\\", "/"))
    path = Path(text).expanduser()
    absolute = path.is_absolute() or PureWindowsPath(text).is_absolute()
    parts = PurePosixPath(text).parts
    relative_parts: tuple[str, ...] | None = None
    known = {
        PurePosixPath(LEGACY_BINARY_DETECTOR_PROFILE).parts,
        ("online_grf_profiles", "binary_detector", "operational", "heel_toe_v25.json"),
        BINARY_DETECTOR_PROFILE_PATH.relative_to(REPO_ROOT).parts,
    }
    if not absolute:
        relative_parts = parts
    else:
        try:
            relative_parts = path.relative_to(REPO_ROOT).parts
        except ValueError:
            if path.is_absolute() and path.exists():
                return None
            checkout_names = {
                REPO_ROOT.name.casefold(), "cmc-like-simulator",
                "cmc-like-simulator - claude",
            }
            for index, part in enumerate(parts):
                if (part.casefold() in checkout_names
                        and not any(p.casefold() in {"validation", "runs", "snapshots"}
                                    for p in parts[:index])):
                    candidate = parts[index + 1:]
                    if candidate in known:
                        relative_parts = candidate
                        break
    return BINARY_DETECTOR_PROFILE_PATH if relative_parts in known else None


def normalize_cli_path_text(raw: str | Path) -> str:
    """Normalize command-line paths accepted on both Windows and POSIX.

    Windows accepts forward slashes in most Python-facing paths, but POSIX treats
    backslashes as literal filename characters. Re-map Windows-style separators
    only off Windows so commands copied from PowerShell still resolve on macOS.
    """
    text = os.fspath(raw)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return text


def cli_path(raw: str | Path) -> Path:
    return Path(normalize_cli_path_text(raw)).expanduser()


@dataclass(frozen=True)
class ResolvedSimulatorPaths:
    repo_root: Path
    model_bundle_dir: Path
    model_path: Path
    plugin_path: Path
    kinematics_path: Path
    external_loads_path: Path | None
    online_grf_profile_path: Path | None
    online_grf_detector_profile_path: Path | None
    binary_phase_detector_profile_path: Path | None
    online_grf_contact_plugin_path: Path
    reserve_actuators_path: Path
    healthy_dir: Path | None


def resolve_repo_path(raw: str | Path) -> Path:
    relocated = remap_policy_transfer_path(raw)
    if relocated is None:
        relocated = remap_morphology_profile_path(raw)
    if relocated is None:
        relocated = remap_model_path(raw)
    if relocated is None:
        relocated = remap_plugin_path(raw)
    if relocated is None:
        relocated = remap_binary_detector_profile_path(raw)
    if relocated is None:
        relocated = remap_online_grf_path(raw)
    if relocated is not None:
        return relocated
    path = cli_path(raw)
    return path if path.is_absolute() else REPO_ROOT / path


def resolve_plugin_path(raw: str | Path) -> Path:
    """Return the basename OpenSim needs, independent of host library suffix."""
    path = resolve_repo_path(raw)
    stem = _current_plugin_stem(path.name)
    return path.with_name(stem) if stem is not None else path


def normalize_cli_existing_path(raw: str | Path) -> str:
    """
    Turn an existing repo-relative CLI path into an absolute path.

    This preserves bundle-relative filenames such as ``Adjusted_SEASEA.osim`` or
    ``data/3DGaitModel2392_Kinematics_q.sto`` when no repo-level file exists.
    """
    text = normalize_cli_path_text(raw)
    relocated = remap_policy_transfer_path(raw)
    if relocated is None:
        relocated = remap_morphology_profile_path(raw)
    if relocated is None:
        relocated = remap_model_path(raw)
    if relocated is None:
        relocated = remap_plugin_path(raw)
    if relocated is None:
        relocated = remap_binary_detector_profile_path(raw)
    if relocated is None:
        relocated = remap_online_grf_path(raw)
    if relocated is not None:
        return str(relocated)
    path = Path(text)
    if path.is_absolute():
        return str(path)
    repo_candidate = REPO_ROOT / path
    return str(repo_candidate) if repo_candidate.exists() else text


def _looks_like_explicit_path(raw: str) -> bool:
    raw = normalize_cli_path_text(raw)
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
    relocated = remap_policy_transfer_path(raw)
    if relocated is None:
        relocated = remap_model_path(raw)
    if relocated is None:
        relocated = remap_binary_detector_profile_path(raw)
    if relocated is None:
        relocated = remap_online_grf_path(raw)
    if relocated is not None:
        return relocated
    path = cli_path(raw)
    return path if path.is_absolute() else bundle_dir / path


def _resolve_optional_bundle_input_path(
    bundle_dir: Path,
    raw: str | Path | None,
) -> Path | None:
    text = "" if raw is None else str(raw).strip()
    return None if not text else _resolve_bundle_input_path(bundle_dir, text)


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
        plugin_path=resolve_plugin_path(cfg.plugin_name),
        kinematics_path=_resolve_bundle_input_path(bundle_dir, cfg.kinematics_file),
        external_loads_path=_resolve_optional_bundle_input_path(
            bundle_dir, cfg.external_loads_xml
        ),
        online_grf_profile_path=_resolve_optional_bundle_input_path(
            bundle_dir, getattr(cfg, "online_grf_profile_file", "")
        ),
        online_grf_detector_profile_path=_resolve_optional_bundle_input_path(
            bundle_dir, getattr(cfg, "online_grf_detector_profile_file", "")
        ),
        binary_phase_detector_profile_path=_resolve_optional_bundle_input_path(
            bundle_dir,
            getattr(cfg, "binary_phase_detector_profile_file", ""),
        ),
        online_grf_contact_plugin_path=resolve_plugin_path(
            getattr(cfg, "online_grf_contact_plugin",
                    "plugins/online_grf_contact/OnlineGRFContact")
        ),
        reserve_actuators_path=_resolve_bundle_input_path(bundle_dir, cfg.reserve_actuators_xml),
        healthy_dir=_resolve_healthy_dir(bundle_dir),
    )
