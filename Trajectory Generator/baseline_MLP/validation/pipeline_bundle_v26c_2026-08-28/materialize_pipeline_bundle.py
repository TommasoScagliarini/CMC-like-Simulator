#!/usr/bin/env python3
"""Materialize and verify the content-addressed V26C validation bundle.

The historical V26C workspace is sealed by path and SHA-256.  This utility
therefore creates byte-identical snapshots under validation instead of moving
or rewriting canonical runtime files, reports, or attested leaves.

Only the Python standard library is used.  Repository paths stored in the
manifests are POSIX-style and relative, so the bundle can be checked on macOS,
Linux, and Windows.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Iterable


BUNDLE_SCHEMA = "cmc.validation-pipeline-bundle/v1"
IN_PLACE_SCHEMA = "cmc.validation-in-place-manifest/v1"
BUNDLE_ID = "v26c_july_replica_2026-08-26__closure_2026-08-28"
BUNDLE_DATE = "2026-08-28"
BUNDLE_DIR = Path(__file__).resolve().parent
OBJECTS_DIR = BUNDLE_DIR / "objects" / "sha256"
LEGACY_SNAPSHOT_DIR = BUNDLE_DIR / "repository_snapshot"
FORENSICS_DIR = BUNDLE_DIR / "forensics"
BUNDLE_MANIFEST = BUNDLE_DIR / "bundle_manifest.json"
IN_PLACE_MANIFEST = BUNDLE_DIR / "in_place_manifest.json"

J5_LOG_SHA256 = "83ec482c2261c757aee9450d97bdf65be952d1bc5cc5e71a8c4a0aed5622f887"
J5_LOG_BUNDLE_RELPATH = "forensics/j5_run.log"

EXCLUDED_DIR_NAMES = {
    "__pycache__",
    ".pytest_cache",
    ".ruff_cache",
    ".mypy_cache",
}
EXCLUDED_FILE_NAMES = {".DS_Store", "opensim.log"}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}

TG_RUNTIME_SOURCES = (
    "Trajectory Generator/osim_trj_cmc_like.py",
    "Trajectory Generator/binary_phase_adapter.py",
    "Trajectory Generator/binary_phase_adapter_v26.py",
    "Trajectory Generator/binary_phase_fsm.py",
    "Trajectory Generator/binary_phase_fsm_v26.py",
    "Trajectory Generator/prosthetic_phase_fsm.py",
)

SIMULATOR_RUNTIME_SOURCES = (
    "binary_phase_detector.py",
    "config.py",
    "inverse_dynamics.py",
    "kinematics_interpolator.py",
    "model_loader.py",
    "online_grf.py",
    "outer_loop.py",
    "output.py",
    "path_resolver.py",
    "prosthesis_controller.py",
    "setup_io.py",
    "simulation_runner.py",
    "static_optimization.py",
)

BASELINE_RUNTIME_DEPENDENCIES = (
    "_bootstrap.py",
    "asymmetric_rl_module.py",
    "env_factory.py",
    "experimental_morphology_corridor.py",
    "exploration_noise.py",
    "primary_split_input_adapter.py",
    "primary_split_v25_residual.py",
    "process_watchdog.py",
    "progress_display.py",
    "reward_function.py",
    "rollout_eval.py",
    "start_condition_metrics.py",
    "start_sampling.py",
    "target_domain_imitation.py",
    "target_domain_markov_adaptation.py",
    "target_domain_noise_adaptation.py",
    "tb_logging.py",
    "train_ppo_mlp.py",
    "training_config.py",
    "update_historical_runs.py",
    "warm_start.py",
    "win_runtime.py",
)

RUNTIME_ASSETS = (
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
    "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
    "models/AB06_SEASEA_Threadmill/data/ExternalForces.xml",
    "models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml",
    "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot",
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json",
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
    "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json",
    "Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json",
    "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
    "plugins/libOnlineGRFContact.dylib",
)

VISUALIZATION_ASSETS = (
    "Geometry/AM_foot_l.STL",
    "Geometry/osseo_femur.STL",
    "Geometry/pors_tibia.STL",
    "Geometry/transfemur_l.stl",
)

HISTORICAL_EVIDENCE_FILES = (
    "validation/controller_memory_ablation/2026-07-13_zero_iter_port/rl_module_initial_warm_start/module_state.pkl",
    "Trajectory Generator/runs/training/validation/controller_memory_ablation/2026-07-13_markov35_zero_iter_port/rl_module_initial_warm_start/module_state.pkl",
    "validation/controller_memory_ablation/2026-07-13_markov35_final_gate.json",
    "validation/controller_memory_ablation/2026-07-13_markov35_teacher_start_minus020/training_cfg.resolved.yaml",
    "validation/controller_memory_ablation/2026-07-13_markov35_teacher_start_plus020/training_cfg.resolved.yaml",
    "Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/student/V1_35D_transplant/rl_module/module_state.pkl",
    "Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/student/B0_35D_MASKED/rl_module/module_state.pkl",
    "validation/verify_h1_readiness.py",
    "validation/test_asymmetric_rl_module.py",
    "validation/warm_start_port_runs/2026-07-12_sigma0003_h1_readiness/h1_readiness.json",
    "validation/critic_warmup/2026-07-13_deployable_sigma0005_iter1/robustness_critic_warmup_gate.json",
)

JULY_RUN_NAMES = (
    "target_domain_imitation_no_controller_memory_2026-07-13",
    "target_domain_markov35_phase_aligned_scaled_cols_r16_alt8_2026-07-13",
    "target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13",
    "target_domain_markov35_phase_aligned_scaled_full_r64_alt8_anchor05_2026-07-13",
    "target_domain_markov35_robust_2026-07-13",
    "target_domain_markov35_robust_conservative_r64_2026-07-13",
    "target_domain_markov35_robust_conservative_r96_2026-07-13",
)

DIRECTORY_SPECS = (
    (
        "Trajectory Generator/baseline_MLP/experimental_configs",
        "validation_configuration_snapshot",
        ("historical_validation",),
    ),
    (
        "Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter",
        "context_snapshot_extra",
        ("analysis",),
    ),
    (
        "Trajectory Generator/runs/training/MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter",
        "context_snapshot_extra",
        ("analysis",),
    ),
) + tuple(
    (
        f"Trajectory Generator/runs/training/{run_name}",
        "context_snapshot_extra",
        ("analysis",),
    )
    for run_name in JULY_RUN_NAMES
)

IN_PLACE_ROOTS = (
    (
        "Trajectory Generator/baseline_MLP/validation/f0_freeze_2026-08-22",
        "upstream_validation_evidence",
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/f1_ablation_2026-08-23",
        "upstream_validation_evidence",
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/f2r_bridge_2026-08-23",
        "upstream_validation_evidence",
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v26b_bridge_2026-08-24",
        "upstream_validation_evidence",
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26",
        "sealed_v26c_stage_workspace",
    ),
)

IN_PLACE_FILES = (
    (
        "Trajectory Generator/baseline_MLP/validation/README.md",
        "validation_workspace_policy",
    ),
    (
        "Trajectory Generator/baseline_MLP/validation/test_training_health_telemetry.py",
        "shared_validation_test",
    ),
)

CONTROL_FILES = (
    "README.md",
    "materialize_pipeline_bundle.py",
    "test_pipeline_bundle.py",
)


class BundleError(RuntimeError):
    """Raised when materialization or validation would be unsafe."""


@dataclass(frozen=True)
class SourceSpec:
    repo_relpath: str
    roles: tuple[str, ...]
    used_by: tuple[str, ...]


def find_repo_root(start: Path) -> Path:
    """Locate the repository by sentinels instead of fragile parent indexes."""
    candidate = start.resolve()
    for directory in (candidate, *candidate.parents):
        if (directory / "AGENTS.md").is_file() and (
            directory / "Trajectory Generator"
        ).is_dir():
            return directory
    raise BundleError(f"repository root not found above {start}")


def from_posix(root: Path, relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (
        not relative
        or pure.is_absolute()
        or ".." in pure.parts
        or any("\\" in part for part in pure.parts)
        or (pure.parts and re.fullmatch(r"[A-Za-z]:", pure.parts[0]))
    ):
        raise BundleError(f"unsafe relative POSIX path: {relative!r}")
    return root.joinpath(*pure.parts)


def repo_relpath(path: Path, repo: Path) -> str:
    try:
        return path.resolve().relative_to(repo.resolve()).as_posix()
    except ValueError as exc:
        raise BundleError(f"path escapes repository: {path}") from exc


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_excluded(path: Path) -> bool:
    return (
        bool(set(path.parts) & EXCLUDED_DIR_NAMES)
        or path.name in EXCLUDED_FILE_NAMES
        or path.suffix.lower() in EXCLUDED_SUFFIXES
    )


def _merge_spec(
    specs: dict[str, SourceSpec],
    relative: str,
    role: str,
    used_by: Iterable[str],
) -> None:
    normalized = PurePosixPath(relative).as_posix()
    previous = specs.get(normalized)
    roles = set(previous.roles if previous else ())
    consumers = set(previous.used_by if previous else ())
    roles.add(role)
    consumers.update(used_by)
    specs[normalized] = SourceSpec(
        repo_relpath=normalized,
        roles=tuple(sorted(roles)),
        used_by=tuple(sorted(consumers)),
    )


def _add_directory(
    repo: Path,
    specs: dict[str, SourceSpec],
    relative: str,
    role: str,
    used_by: Iterable[str],
) -> None:
    directory = from_posix(repo, relative)
    if not directory.is_dir():
        raise BundleError(f"required directory is missing: {relative}")
    for path in sorted(directory.rglob("*")):
        if is_excluded(path):
            continue
        if path.is_symlink():
            raise BundleError(f"symlink is not portable and cannot be bundled: {path}")
        if path.is_file():
            _merge_spec(specs, repo_relpath(path, repo), role, used_by)


def _discover_reports(repo: Path) -> list[str]:
    guide = repo / "reports" / "user" / (
        "2026-08-28_guida_ricostruzione_pipeline_exnovo_"
        "training_ready_da_imitativo_v26_agosto.md"
    )
    if not guide.is_file():
        raise BundleError(f"reconstruction guide is missing: {guide}")

    discovered = {guide}
    link_pattern = re.compile(r"\(([^)\n]+\.md)(?:#[^)\n]*)?\)")
    for raw_target in link_pattern.findall(guide.read_text(encoding="utf-8")):
        target = raw_target.strip()
        if "://" in target:
            continue
        candidate = (guide.parent / target).resolve()
        try:
            candidate.relative_to(repo.resolve())
        except ValueError as exc:
            raise BundleError(f"report link escapes repository: {target}") from exc
        if candidate.is_file():
            discovered.add(candidate)

    user_reports = repo / "reports" / "user"
    # J13 records the reports it read as the complete July 11--15 and August
    # 22--27 date ranges rather than as an exact filename list.  Preserve the
    # full declared ranges, plus the final 28 August attestation reports.
    for day in range(22, 29):
        prefix = f"2026-08-{day:02d}_"
        discovered.update(user_reports.glob(f"{prefix}*.md"))
    for day in range(11, 16):
        discovered.update(user_reports.glob(f"2026-07-{day:02d}_*.md"))

    daily_reports = repo / "reports" / "daily"
    for day in range(22, 29):
        candidate = daily_reports / f"2026-08-{day:02d}_daily-report.md"
        if candidate.is_file():
            discovered.add(candidate)

    recovery_plan = repo / "reports" / "plans" / (
        "2026-08-22_piano_operativo_recupero_pipeline_exnovo_"
        "v26_fsmv3_morphology.md"
    )
    if recovery_plan.is_file():
        discovered.add(recovery_plan)

    return sorted(repo_relpath(path, repo) for path in discovered)


def collect_source_specs(repo: Path) -> list[SourceSpec]:
    """Return the audited source/evidence closure outside the sealed workspace."""
    specs: dict[str, SourceSpec] = {}

    baseline = repo / "Trajectory Generator" / "baseline_MLP"
    if not baseline.is_dir():
        raise BundleError(f"baseline directory is missing: {baseline}")
    for path in sorted(baseline.iterdir()):
        if path.is_file() and not is_excluded(path):
            _merge_spec(
                specs,
                repo_relpath(path, repo),
                "baseline_source_context_snapshot",
                ("analysis", "historical_validation"),
            )

    for filename in BASELINE_RUNTIME_DEPENDENCIES:
        _merge_spec(
            specs,
            f"Trajectory Generator/baseline_MLP/{filename}",
            "v26c_runtime_dependency_source",
            ("V26C",),
        )
    _merge_spec(
        specs,
        "Trajectory Generator/baseline_MLP/target_domain_dagger.py",
        "manual_analysis_dependency_source",
        ("J13",),
    )

    for relative in TG_RUNTIME_SOURCES:
        _merge_spec(
            specs,
            relative,
            "trajectory_generator_runtime_source_snapshot",
            ("J1", "J3", "J9", "J12", "J16", "J19", "J20"),
        )
    for relative in SIMULATOR_RUNTIME_SOURCES:
        _merge_spec(
            specs,
            relative,
            "simulator_runtime_source_snapshot",
            ("J1", "J3", "J9", "J12", "J16", "J19", "J20"),
        )
    for relative in RUNTIME_ASSETS:
        _merge_spec(
            specs,
            relative,
            "runtime_asset_snapshot",
            ("J1", "J3", "J9", "J12", "J16", "J19", "J20"),
        )
    for relative in VISUALIZATION_ASSETS:
        _merge_spec(
            specs,
            relative,
            "available_visualization_asset_snapshot",
            ("analysis", "visualization"),
        )
    for relative in HISTORICAL_EVIDENCE_FILES:
        _merge_spec(
            specs,
            relative,
            "historical_methodological_or_critic_evidence",
            ("J0", "J13", "J20", "analysis"),
        )
    for relative in _discover_reports(repo):
        _merge_spec(
            specs,
            relative,
            "report_snapshot_canonical_original_retained",
            ("reconstruction", "analysis", "provenance"),
        )
    for relative, role, used_by in DIRECTORY_SPECS:
        _add_directory(repo, specs, relative, role, used_by)

    parent_best = (
        "Trajectory Generator/runs/training/"
        "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best"
    )
    for filename in (
        "actor_feature_manifest.json",
        "class_and_ctor_args.pkl",
        "metadata.json",
        "module_state.pkl",
    ):
        _merge_spec(
            specs,
            f"{parent_best}/{filename}",
            "operational_input",
            ("J0", "J1", "J2", "J19A", "J20", "J21"),
        )

    _merge_spec(
        specs,
        "Trajectory Generator/runs/training/MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml",
        "runtime_configuration_anchor",
        ("J1", "J20"),
    )

    july_root = "Trajectory Generator/runs/training"
    for run_name in JULY_RUN_NAMES:
        for filename in ("run_summary.json", "adaptation_report.json"):
            _merge_spec(
                specs,
                f"{july_root}/{run_name}/{filename}",
                "methodological_evidence_opened_by_pipeline",
                ("J0",),
            )
    july_base = f"{july_root}/{JULY_RUN_NAMES[0]}"
    for filename in ("teacher_dataset.npz", "teacher_trace.json"):
        _merge_spec(
            specs,
            f"{july_base}/{filename}",
            "methodological_evidence_opened_by_pipeline",
            ("J0",),
        )
    july_selected = f"{july_root}/{JULY_RUN_NAMES[2]}"
    _merge_spec(
        specs,
        f"{july_selected}/markov_dataset_report.json",
        "methodological_evidence_opened_by_pipeline",
        ("J0",),
    )
    for filename in (
        "actor_feature_manifest.json",
        "rl_module_target_adapted/class_and_ctor_args.pkl",
        "rl_module_target_adapted/metadata.json",
        "rl_module_target_adapted/module_state.pkl",
    ):
        _merge_spec(
            specs,
            f"{july_selected}/{filename}",
            "manual_analysis_input",
            ("J13",),
        )

    missing = [
        spec.repo_relpath
        for spec in specs.values()
        if not from_posix(repo, spec.repo_relpath).is_file()
    ]
    if missing:
        raise BundleError("required source files are missing:\n  " + "\n  ".join(missing))
    return [specs[key] for key in sorted(specs)]


def _copy_no_clobber(source: Path, destination: Path) -> None:
    """Copy bytes once; an existing different destination is a hard failure."""
    if source.is_symlink():
        raise BundleError(f"refusing to snapshot symlink: {source}")
    source_sha = sha256_file(source)
    if destination.exists() or destination.is_symlink():
        if destination.is_symlink() or not destination.is_file():
            raise BundleError(f"snapshot destination is not a regular file: {destination}")
        if sha256_file(destination) != source_sha:
            raise BundleError(f"no-clobber mismatch at {destination}")
        return

    destination.parent.mkdir(parents=True, exist_ok=True)
    created_by_this_call = False
    try:
        try:
            destination_stream = destination.open("xb")
            created_by_this_call = True
        except FileExistsError:
            # A concurrent materializer won the exclusive create.  Never
            # remove its file: accept it only after it is complete and equal;
            # otherwise fail safely and let a later retry verify it.
            if destination.is_symlink() or not destination.is_file():
                raise BundleError(
                    f"concurrent snapshot destination is not regular: {destination}"
                )
            if sha256_file(destination) != source_sha:
                raise BundleError(
                    f"concurrent no-clobber snapshot is not complete/equal: {destination}"
                )
            return
        with source.open("rb") as src, destination_stream as dst:
            shutil.copyfileobj(src, dst, length=1024 * 1024)
    except Exception:
        if created_by_this_call and destination.exists():
            destination.unlink()
        raise
    if sha256_file(destination) != source_sha:
        if created_by_this_call and destination.exists():
            destination.unlink()
        raise BundleError(f"copy verification failed: {destination}")


def object_path_for_sha256(digest: str) -> Path:
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise BundleError(f"invalid SHA-256 object id: {digest!r}")
    return OBJECTS_DIR / digest[:2] / digest


def _entry_for_snapshot(repo: Path, spec: SourceSpec) -> dict[str, object]:
    source = from_posix(repo, spec.repo_relpath)
    digest = sha256_file(source)
    destination = object_path_for_sha256(digest)
    return {
        "bundle_relpath": destination.relative_to(BUNDLE_DIR).as_posix(),
        "original_repo_relpath": spec.repo_relpath,
        "roles": list(spec.roles),
        "sha256": digest,
        "size_bytes": source.stat().st_size,
        "used_by": list(spec.used_by),
    }


def _control_entries() -> list[dict[str, object]]:
    entries = []
    for relative in CONTROL_FILES:
        path = BUNDLE_DIR / relative
        if not path.is_file():
            raise BundleError(f"bundle control file is missing: {relative}")
        entries.append(
            {
                "bundle_relpath": relative,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    return entries


def _manifest_bytes(payload: dict[str, object]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _write_manifest(path: Path, payload: dict[str, object], refresh: bool) -> None:
    content = _manifest_bytes(payload)
    if path.exists():
        if path.read_bytes() == content:
            return
        if not refresh:
            raise BundleError(
                f"manifest differs and was not overwritten: {path}; "
                "use --refresh-manifests after reviewing the diff"
            )
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_bytes(content)
        os.replace(temporary, path)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(content)


def _iter_in_place_files(repo: Path) -> Iterable[tuple[Path, str]]:
    for relative, role in IN_PLACE_ROOTS:
        root = from_posix(repo, relative)
        if not root.is_dir():
            raise BundleError(f"in-place validation root is missing: {relative}")
        for path in sorted(root.rglob("*")):
            if is_excluded(path):
                continue
            if path.is_symlink():
                raise BundleError(f"in-place symlink is not portable: {path}")
            if path.is_file():
                yield path, role
    for relative, role in IN_PLACE_FILES:
        path = from_posix(repo, relative)
        if not path.is_file():
            raise BundleError(f"in-place validation file is missing: {relative}")
        yield path, role


def build_in_place_manifest(repo: Path) -> dict[str, object]:
    entries = []
    root_counts = {relative: 0 for relative, _ in IN_PLACE_ROOTS}
    total_bytes = 0
    for path, role in _iter_in_place_files(repo):
        relative = repo_relpath(path, repo)
        size = path.stat().st_size
        total_bytes += size
        for root_relative in root_counts:
            prefix = root_relative.rstrip("/") + "/"
            if relative.startswith(prefix):
                root_counts[root_relative] += 1
                break
        entries.append(
            {
                "repo_relpath": relative,
                "role": role,
                "sha256": sha256_file(path),
                "size_bytes": size,
            }
        )
    entries.sort(key=lambda item: str(item["repo_relpath"]))
    return {
        "bundle_id": BUNDLE_ID,
        "entries": entries,
        "exclusions": {
            "directory_names": sorted(EXCLUDED_DIR_NAMES),
            "file_names": sorted(EXCLUDED_FILE_NAMES),
            "suffixes": sorted(EXCLUDED_SUFFIXES),
            "rationale": "Caches, bytecode, Finder metadata, and incidental OpenSim logs are not authoritative evidence.",
        },
        "root_file_counts": root_counts,
        "schema": IN_PLACE_SCHEMA,
        "summary": {"file_count": len(entries), "size_bytes": total_bytes},
        "validation_root_repo_relpath": "Trajectory Generator/baseline_MLP/validation",
    }


def build_bundle_manifest(
    repo: Path,
    specs: list[SourceSpec],
    j5_destination: Path,
    in_place_payload: dict[str, object],
) -> dict[str, object]:
    entries = [_entry_for_snapshot(repo, spec) for spec in specs]
    external_entries = [
        {
            "bundle_relpath": j5_destination.relative_to(BUNDLE_DIR).as_posix(),
            "original_source_locator": "external:session-private-tmp/j5_run.log",
            "roles": ["J5_forensic_execution_log"],
            "sha256": sha256_file(j5_destination),
            "size_bytes": j5_destination.stat().st_size,
            "used_by": ["J5", "reconstruction", "analysis"],
        }
    ]
    unique_objects = {
        str(entry["sha256"]): int(entry["size_bytes"]) for entry in entries
    }
    bundle_repo_relpath = repo_relpath(BUNDLE_DIR, repo)
    checkout_relpaths = {
        str(entry["repo_relpath"])
        for entry in in_place_payload.get("entries", [])
    }
    checkout_relpaths.update(
        f"{bundle_repo_relpath}/{entry['bundle_relpath']}" for entry in entries
    )
    checkout_relpaths.update(
        f"{bundle_repo_relpath}/{entry['bundle_relpath']}"
        for entry in external_entries
    )
    checkout_relpaths.update(
        f"{bundle_repo_relpath}/{relative}"
        for relative in (*CONTROL_FILES, BUNDLE_MANIFEST.name, IN_PLACE_MANIFEST.name)
    )
    max_checkout_relative_chars = max(
        len(path.replace("/", "\\")) for path in checkout_relpaths
    )
    legacy_windows_path_budget = 248
    short_root_max_chars = (
        legacy_windows_path_budget - 1 - max_checkout_relative_chars
    )

    return {
        "bundle_date": BUNDLE_DATE,
        "bundle_id": BUNDLE_ID,
        "compatibility_policy": {
            "canonical_reports_retained": True,
            "historical_paths_retained": True,
            "production_runtime_retained": True,
            "snapshot_is_runtime_canonical": False,
            "symlinks_used": False,
            "live_source_audit": {
                "default": False,
                "scope": "host_source_workspace_only",
                "note": "Optional comparison requires ignored historical runs and canonical originals; archive integrity does not depend on them.",
            },
            "windows_checkout": {
                "legacy_path_budget_chars": legacy_windows_path_budget,
                "max_checkout_relative_path_chars": max_checkout_relative_chars,
                "short_root_max_chars_without_long_path_support": short_root_max_chars,
                "requirement": "Use a checkout root no longer than short_root_max_chars, or enable Windows LongPathsEnabled and Git core.longpaths before checkout.",
            },
        },
        "control_files": _control_entries(),
        "entries": entries,
        "external_entries": external_entries,
        "gaps": [
            {
                "id": "windows_plugin_binaries",
                "status": "missing",
                "detail": "The attested run used the two bundled macOS dylibs; equivalent SEA ff and OnlineGRFContact Windows DLLs were not present.",
            },
            {
                "id": "ad_hoc_analysis_scripts",
                "status": "not_created_historically",
                "detail": "Reusable scripts do not exist for the manual J13 and J17 analyses and part of J18 recalibration; reports and JSON measurements are retained instead.",
            },
            {
                "id": "opensim_visual_meshes",
                "status": "not_closed",
                "detail": "The four referenced meshes available in Geometry are bundled. Fourteen referenced VTP meshes are absent from the repository; they were not required by the attested headless run.",
                "missing_mesh_files": [
                    "bofoot.vtp",
                    "femur_r.vtp",
                    "fibula.vtp",
                    "foot.vtp",
                    "hat_jaw.vtp",
                    "hat_ribs.vtp",
                    "hat_skull.vtp",
                    "hat_spine.vtp",
                    "l_pelvis.vtp",
                    "pelvis.vtp",
                    "sacrum.vtp",
                    "talus.vtp",
                    "tibia_r.vtp",
                    "treadmill.vtp",
                ],
            },
            {
                "id": "environment_reproducibility",
                "status": "partial",
                "detail": "J20 records key Python, Ray, Torch, and NumPy versions, but no complete baseline_MLP environment lock or native-library lock exists. The byte closure is complete; cross-machine environment reconstruction is not.",
            },
        ],
        "related_manifests": [
            {
                "bundle_relpath": IN_PLACE_MANIFEST.relative_to(BUNDLE_DIR).as_posix(),
                "sha256": hashlib.sha256(_manifest_bytes(in_place_payload)).hexdigest(),
                "size_bytes": len(_manifest_bytes(in_place_payload)),
            }
        ],
        "schema": BUNDLE_SCHEMA,
        "summary": {
            "external_file_count": len(external_entries),
            "object_bytes": sum(unique_objects.values()),
            "object_count": len(unique_objects),
            "source_entry_count": len(entries),
            "source_logical_bytes": sum(
                int(item["size_bytes"]) for item in entries
            ),
        },
    }


def _resolve_j5_source(j5_log: Path | None, destination: Path) -> Path | None:
    if destination.is_file() and sha256_file(destination) == J5_LOG_SHA256:
        return None
    if j5_log is None:
        raise BundleError(
            "J5 forensic log snapshot is absent; pass its current path with --j5-log"
        )
    source = j5_log.expanduser().resolve()
    if not source.is_file():
        raise BundleError(f"J5 log source is missing: {source}")
    observed = sha256_file(source)
    if observed != J5_LOG_SHA256:
        raise BundleError(
            f"J5 log hash mismatch: expected {J5_LOG_SHA256}, observed {observed}"
        )
    return source


def materialize(repo: Path, j5_log: Path | None, refresh_manifests: bool) -> None:
    specs = collect_source_specs(repo)
    j5_destination = BUNDLE_DIR / J5_LOG_BUNDLE_RELPATH
    j5_source = _resolve_j5_source(j5_log, j5_destination)

    for spec in specs:
        source = from_posix(repo, spec.repo_relpath)
        destination = object_path_for_sha256(sha256_file(source))
        _copy_no_clobber(source, destination)
    if j5_source is not None:
        _copy_no_clobber(j5_source, j5_destination)
    if sha256_file(j5_destination) != J5_LOG_SHA256:
        raise BundleError("materialized J5 forensic log no longer matches its audited hash")

    in_place_payload = build_in_place_manifest(repo)
    bundle_payload = build_bundle_manifest(
        repo, specs, j5_destination, in_place_payload
    )
    _write_manifest(IN_PLACE_MANIFEST, in_place_payload, refresh_manifests)
    _write_manifest(BUNDLE_MANIFEST, bundle_payload, refresh_manifests)


def _load_json(path: Path) -> dict[str, object]:
    if not path.is_file():
        raise BundleError(f"manifest is missing: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise BundleError(f"manifest root is not an object: {path}")
    return payload


def _verify_entry(path: Path, expected_size: int, expected_sha: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise BundleError(f"expected regular file is missing: {path}")
    if path.stat().st_size != expected_size:
        raise BundleError(f"size mismatch: {path}")
    observed = sha256_file(path)
    if observed != expected_sha:
        raise BundleError(
            f"SHA-256 mismatch: {path}; expected {expected_sha}, observed {observed}"
        )


def verify(
    repo: Path | None,
    full: bool,
    check_live_sources: bool = False,
    windows_long_paths_enabled: bool = False,
) -> dict[str, int]:
    bundle = _load_json(BUNDLE_MANIFEST)
    in_place = _load_json(IN_PLACE_MANIFEST)
    if bundle.get("schema") != BUNDLE_SCHEMA:
        raise BundleError("unexpected bundle manifest schema")
    if in_place.get("schema") != IN_PLACE_SCHEMA:
        raise BundleError("unexpected in-place manifest schema")

    windows_policy = bundle.get("compatibility_policy", {}).get(
        "windows_checkout", {}
    )
    if os.name == "nt" and repo is not None and not windows_long_paths_enabled:
        max_relative = int(windows_policy["max_checkout_relative_path_chars"])
        budget = int(windows_policy["legacy_path_budget_chars"])
        current_max = len(str(repo)) + 1 + max_relative
        if current_max > budget:
            raise BundleError(
                "Windows checkout exceeds the legacy path budget: "
                f"root={len(str(repo))} chars, longest={current_max}, budget={budget}. "
                "Move the repository to the short root documented in the manifest, "
                "or enable OS LongPathsEnabled plus Git core.longpaths and pass "
                "--windows-long-paths-enabled."
            )

    related_manifests = list(bundle.get("related_manifests", []))
    if len(related_manifests) != 1:
        raise BundleError("bundle must pin exactly one in-place manifest")
    related = related_manifests[0]
    if str(related.get("bundle_relpath")) != IN_PLACE_MANIFEST.name:
        raise BundleError("bundle pins an unexpected related manifest")
    _verify_entry(
        IN_PLACE_MANIFEST,
        int(related["size_bytes"]),
        str(related["sha256"]),
    )

    if check_live_sources:
        if repo is None:
            raise BundleError("--check-live-sources requires a repository workspace")
        recorded_specs = {
            str(entry["original_repo_relpath"])
            for entry in bundle.get("entries", [])
        }
        current_specs = {spec.repo_relpath for spec in collect_source_specs(repo)}
        if current_specs != recorded_specs:
            missing = sorted(current_specs - recorded_specs)
            stale = sorted(recorded_specs - current_specs)
            raise BundleError(
                f"source closure drift; missing_from_manifest={missing}, stale={stale}"
            )

    source_count = 0
    source_logical_bytes = 0
    verified_objects: dict[str, int] = {}
    for entry in bundle.get("entries", []):
        relative = str(entry["original_repo_relpath"])
        bundle_relative = str(entry["bundle_relpath"])
        expected_size = int(entry["size_bytes"])
        expected_sha = str(entry["sha256"])
        expected_object = object_path_for_sha256(expected_sha)
        snapshot = from_posix(BUNDLE_DIR, bundle_relative)
        if snapshot != expected_object:
            raise BundleError(
                f"manifest object path does not match SHA-256 id: {bundle_relative}"
            )
        if expected_sha not in verified_objects:
            _verify_entry(snapshot, expected_size, expected_sha)
            verified_objects[expected_sha] = expected_size
        elif verified_objects[expected_sha] != expected_size:
            raise BundleError(f"one object id has conflicting sizes: {expected_sha}")
        if check_live_sources:
            source = from_posix(repo, relative)
            _verify_entry(source, expected_size, expected_sha)
        source_count += 1
        source_logical_bytes += expected_size

    external_count = 0
    for entry in bundle.get("external_entries", []):
        snapshot = from_posix(BUNDLE_DIR, str(entry["bundle_relpath"]))
        _verify_entry(snapshot, int(entry["size_bytes"]), str(entry["sha256"]))
        external_count += 1

    for entry in bundle.get("control_files", []):
        path = from_posix(BUNDLE_DIR, str(entry["bundle_relpath"]))
        _verify_entry(path, int(entry["size_bytes"]), str(entry["sha256"]))

    in_place_entries = list(in_place.get("entries", []))
    if full:
        if repo is None:
            raise BundleError("--full requires the repository validation workspace")
        for entry in in_place_entries:
            path = from_posix(repo, str(entry["repo_relpath"]))
            if path.is_symlink() or not path.is_file():
                raise BundleError(f"in-place evidence is missing: {path}")
            if path.stat().st_size != int(entry["size_bytes"]):
                raise BundleError(f"in-place evidence size drift: {path}")
            if sha256_file(path) != str(entry["sha256"]):
                raise BundleError(f"in-place evidence SHA-256 drift: {path}")

    return {
        "external_file_count": external_count,
        "in_place_file_count": len(in_place_entries),
        "object_bytes": sum(verified_objects.values()),
        "object_count": len(verified_objects),
        "source_entry_count": source_count,
        "source_logical_bytes": source_logical_bytes,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--materialize",
        action="store_true",
        help="copy missing snapshots and create deterministic manifests",
    )
    action.add_argument(
        "--verify",
        action="store_true",
        help="verify archive objects, controls, and manifests only (default)",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="also re-hash every in-place V26C/upstream validation artefact",
    )
    parser.add_argument(
        "--check-live-sources",
        action="store_true",
        help="also compare every current canonical source with its archived object",
    )
    parser.add_argument(
        "--windows-long-paths-enabled",
        action="store_true",
        help="acknowledge that OS LongPathsEnabled and Git core.longpaths are active",
    )
    parser.add_argument(
        "--j5-log",
        type=Path,
        help="one-time source path for the audited J5 private-tmp log",
    )
    parser.add_argument(
        "--refresh-manifests",
        action="store_true",
        help="replace generated manifests only; mirrored files remain no-clobber",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        needs_repo = bool(args.materialize or args.full or args.check_live_sources)
        repo = find_repo_root(BUNDLE_DIR) if needs_repo else None
        if args.materialize:
            if repo is None:  # defensive; materialize always sets needs_repo
                raise BundleError("materialization requires a repository workspace")
            materialize(repo, args.j5_log, args.refresh_manifests)
        counts = verify(
            repo,
            full=args.full,
            check_live_sources=bool(args.check_live_sources or args.materialize),
            windows_long_paths_enabled=args.windows_long_paths_enabled,
        )
    except (BundleError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(
        "PASS bundle={bundle} source_entries={sources} objects={objects} "
        "object_bytes={bytes_} external={external} in_place={in_place} "
        "full={full} live_sources={live}".format(
            bundle=BUNDLE_ID,
            sources=counts["source_entry_count"],
            objects=counts["object_count"],
            bytes_=counts["object_bytes"],
            external=counts["external_file_count"],
            in_place=counts["in_place_file_count"],
            full=bool(args.full),
            live=bool(args.check_live_sources or args.materialize),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
