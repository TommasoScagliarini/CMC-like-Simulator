"""Freeze collection/corpus execution, with all actor updates still closed."""

from __future__ import annotations

import json
import os
import platform
import sys
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for root in (VALIDATION_ROOT, BASELINE_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import compare_h0_v25_abc as strict_io  # noqa: E402
import run_h0_primary_grf_split_v1_adaptation as driver  # noqa: E402


OUTPUT = driver.EXECUTION_LOCK
TEST_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v1_preflight_test_receipt_v2.json"
LAYOUT_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v1_layout_preflight_receipt_v2.json"
PRIMARY_CORE_LOCK = VALIDATION_ROOT / "primary_grf_core_lock_2026-08-03.json"
RESERVE_ACTUATORS = REPO_ROOT / "models" / "AB06_SEASEA_Threadmill" / "data" / "CMC_Actuators.xml"
SEA_PLUGIN_SOURCE = REPO_ROOT / "tools" / "sea_plugin_relative_d" / "SeriesElasticActuator.cpp"
SEA_PLUGIN_HEADER = REPO_ROOT / "tools" / "sea_plugin_relative_d" / "SeriesElasticActuator.h"
SEA_PLUGIN_MACOS = REPO_ROOT / "plugins" / "libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
MORPHOLOGY_PROFILE = (
    BASELINE_ROOT
    / "morphology_profiles"
    / "ab06_prosthetic_mean_std_corridor.json"
)

SOURCE_PATHS = {
    "collection_freezer": Path(__file__).resolve(),
    "execution_driver": Path(driver.__file__).resolve(),
    "h0_runtime_adapter": VALIDATION_ROOT / "run_h0_v25_abc_preflight.py",
    "pure_contract": BASELINE_ROOT / "primary_grf_split_adaptation.py",
    "adapt_actor": BASELINE_ROOT / "target_domain_imitation.py",
    "bootstrap": BASELINE_ROOT / "_bootstrap.py",
    "experimental_morphology_corridor": BASELINE_ROOT / "experimental_morphology_corridor.py",
    "exploration_noise": BASELINE_ROOT / "exploration_noise.py",
    "process_watchdog": BASELINE_ROOT / "process_watchdog.py",
    "progress_display": BASELINE_ROOT / "progress_display.py",
    "win_runtime": BASELINE_ROOT / "win_runtime.py",
    "comparator": VALIDATION_ROOT / "compare_h0_primary_grf_split.py",
    "strict_io": VALIDATION_ROOT / "compare_h0_v25_abc.py",
    "rollout_eval": BASELINE_ROOT / "rollout_eval.py",
    "reward_function": BASELINE_ROOT / "reward_function.py",
    "asymmetric_rl_module": BASELINE_ROOT / "asymmetric_rl_module.py",
    "training_config": BASELINE_ROOT / "training_config.py",
    "warm_start": BASELINE_ROOT / "warm_start.py",
    "env_factory": BASELINE_ROOT / "env_factory.py",
    "environment": REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py",
    "phase_fsm": REPO_ROOT / "Trajectory Generator" / "prosthetic_phase_fsm.py",
    "binary_phase_adapter": REPO_ROOT / "Trajectory Generator" / "binary_phase_adapter.py",
    "binary_phase_fsm": REPO_ROOT / "Trajectory Generator" / "binary_phase_fsm.py",
    "binary_phase_detector": REPO_ROOT / "binary_phase_detector.py",
    "simulation_runner": REPO_ROOT / "simulation_runner.py",
    "static_optimization": REPO_ROOT / "static_optimization.py",
    "inverse_dynamics": REPO_ROOT / "inverse_dynamics.py",
    "outer_loop": REPO_ROOT / "outer_loop.py",
    "prosthesis_controller": REPO_ROOT / "prosthesis_controller.py",
    "kinematics_interpolator": REPO_ROOT / "kinematics_interpolator.py",
    "root_config": REPO_ROOT / "config.py",
    "path_resolver": REPO_ROOT / "path_resolver.py",
    "model_loader": REPO_ROOT / "model_loader.py",
    "online_grf": REPO_ROOT / "online_grf.py",
    "output": REPO_ROOT / "output.py",
    "setup_io": REPO_ROOT / "setup_io.py",
    "input_preparer": VALIDATION_ROOT / "prepare_h0_primary_grf_split_inputs.py",
    "noise_tape_preparer": VALIDATION_ROOT / "prepare_h0_primary_grf_split_noise_tapes.py",
}


class FreezeError(RuntimeError):
    pass


def _mapping(path: Path) -> dict[str, Any]:
    value = strict_io.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise FreezeError(f"expected object: {path}")
    return dict(value)


def _require_status(path: Path, status: str) -> dict[str, Any]:
    value = _mapping(path)
    if value.get("status") != status or value.get("passed") is not True:
        raise FreezeError(f"required PASS receipt is invalid: {path}")
    return value


def _verify_nested_source_records(
    values: Any, label: str, *, required: bool = True
) -> None:
    if not isinstance(values, Mapping):
        if required:
            raise FreezeError(f"{label} source ledger is malformed")
        return
    for key, record in values.items():
        driver._frozen_path_record(record, f"{label}.{key}")


def _input_records() -> dict[str, Any]:
    records: dict[str, Any] = {
        "protocol_lock": driver.source_record(driver.PROTOCOL_LOCK),
        "h0_config": driver.source_record(driver.H0_CONFIG),
        "h0_module_state": driver.source_record(driver.H0_MODULE / "module_state.pkl"),
        "h0_module_ctor": driver.source_record(
            driver.H0_MODULE / "class_and_ctor_args.pkl"
        ),
        "h0_module_metadata": driver.source_record(driver.H0_MODULE / "metadata.json"),
        "actor_layout_reference": driver.source_record(driver.ACTOR_LAYOUT_REFERENCE),
        "full_layout_reference": driver.source_record(driver.FULL_LAYOUT_REFERENCE),
        "profile_overlay_manifest": driver.source_record(driver.INPUT_MANIFEST),
        "noise_tape_manifest": driver.source_record(driver.NOISE_MANIFEST),
        "primary_core_lock": driver.source_record(PRIMARY_CORE_LOCK),
        "reserve_actuators": driver.source_record(RESERVE_ACTUATORS),
        "sea_plugin_source": driver.source_record(SEA_PLUGIN_SOURCE),
        "sea_plugin_header": driver.source_record(SEA_PLUGIN_HEADER),
        "sea_plugin_macos": driver.source_record(SEA_PLUGIN_MACOS),
        "morphology_profile_weight_zero_runtime_input": driver.source_record(
            MORPHOLOGY_PROFILE
        ),
    }
    core = _mapping(PRIMARY_CORE_LOCK)
    if core.get("status") != "PRIMARY_CONTRACT_FROZEN_LIMITED_HYBRID_CLAIM":
        raise FreezeError("primary core lock is not authoritative")
    for section_name in ("scientific_core", "platform_attestations"):
        section = core.get(section_name)
        if not isinstance(section, Mapping):
            raise FreezeError(f"primary core {section_name} is malformed")
        for key, record in section.items():
            if not isinstance(record, Mapping):
                # The Windows pending-attestation string is not an executable
                # input for this macOS-only numerical claim.
                continue
            path = driver._frozen_path_record(
                record, f"primary_core.{section_name}.{key}"
            )
            records[f"primary_core_{section_name}_{key}"] = driver.source_record(path)
    for trial_id, trial in driver.TRIALS.items():
        records[f"trial_{trial_id}_setup"] = driver.source_record(trial["setup"])
        records[f"trial_{trial_id}_primary_overlay"] = driver.source_record(
            driver.INPUT_ROOT / f"trial_{trial_id}_primary_surface_velocity.json"
        )
        records[f"trial_{trial_id}_analog_overlay"] = driver.source_record(
            driver.INPUT_ROOT / f"trial_{trial_id}_analog_surface_velocity.json"
        )
        source_receipt_path = (
            driver.SETUP_ROOT
            / "unit_source_receipts"
            / f"dev{trial_id}_p04_10ms.json"
        )
        source_receipt = _mapping(source_receipt_path)
        if (
            source_receipt.get("status") != "PASS"
            or source_receipt.get("source_trial_id") != trial_id
            or source_receipt.get("plateau_id") != "04"
            or float(source_receipt.get("plateau_speed_mps", float("nan")))
            != float(trial["speed_mps"])
        ):
            raise FreezeError(f"trial {trial_id} source receipt drifted")
        records[f"trial_{trial_id}_source_receipt"] = driver.source_record(
            source_receipt_path
        )
        semantic_sources = source_receipt.get("semantic_sources")
        if not isinstance(semantic_sources, Mapping):
            raise FreezeError(f"trial {trial_id} semantic sources are malformed")
        for source_name in ("setup", "ik_sto", "grf_sto", "external_loads_xml"):
            source_path = driver._frozen_path_record(
                semantic_sources.get(source_name),
                f"trial_{trial_id}.semantic_sources.{source_name}",
            )
            records[
                f"trial_{trial_id}_semantic_{source_name}"
            ] = driver.source_record(source_path)
        if records[f"trial_{trial_id}_setup"]["sha256"] != records[
            f"trial_{trial_id}_semantic_setup"
        ]["sha256"]:
            raise FreezeError(f"trial {trial_id} setup hash linkage failed")
        records[f"trial_{trial_id}_collection_noise_tape"] = driver.source_record(
            driver.NOISE_ROOT / f"collection_trial_{trial_id}_standard_normal.npz"
        )
        records[f"trial_{trial_id}_qualification_noise_tape"] = driver.source_record(
            driver.NOISE_ROOT
            / f"qualification_trial_{trial_id}_standard_normal.npz"
        )
    records["qualification_deterministic_noise_tape"] = driver.source_record(
        driver.NOISE_ROOT / "qualification_deterministic_all_zero.npz"
    )
    return records


def _destination_paths() -> list[Path]:
    return [
        driver.RUN_ROOT / "collection" / f"trial_{trial}"
        for trial in ("02", "04", "08")
    ] + [driver.RUN_ROOT / "corpus"]


def _preallocate_destinations(destinations: list[Path]) -> None:
    if driver.RUN_ROOT.exists():
        raise FreezeError(f"run root already exists: {driver.RUN_ROOT}")
    for destination in destinations:
        destination.mkdir(parents=True, exist_ok=False)
        if any(destination.iterdir()):
            raise FreezeError(f"destination is not empty: {destination}")


def freeze() -> dict[str, Any]:
    if os.path.lexists(OUTPUT):
        raise FreezeError(f"refusing to clobber: {OUTPUT}")
    protocol = _mapping(driver.PROTOCOL_LOCK)
    if protocol.get("status") != (
        "H0_PRIMARY_GRF_SPLIT_V1_ADAPTATION_PROTOCOL_FROZEN_"
        "EXECUTION_AND_UPDATES_NOT_AUTHORIZED"
    ):
        raise FreezeError("protocol-only lock is not authoritative")
    protocol_authority = protocol.get("authority")
    if (
        not isinstance(protocol_authority, Mapping)
        or protocol_authority.get(
            "h0_primary_split_supervised_adaptation_protocol_authorized"
        )
        is not True
        or protocol_authority.get("actor_updates_authorized") is not False
    ):
        raise FreezeError("protocol authority is malformed")
    tests = _require_status(
        TEST_RECEIPT, "PASS_H0_PRIMARY_GRF_SPLIT_PREFLIGHT_TESTS"
    )
    _verify_nested_source_records(
        tests.get("frozen_sources_at_test"), "tests.frozen_sources_at_test"
    )
    layout = _require_status(
        LAYOUT_RECEIPT, "PASS_H0_PRIMARY_GRF_SPLIT_LAYOUT_PREFLIGHT"
    )
    driver._frozen_path_record(
        layout.get("execution_driver"), "layout.execution_driver"
    )
    overlay = _mapping(driver.INPUT_MANIFEST)
    if (
        overlay.get("status") != "H0_PRIMARY_GRF_SPLIT_INPUTS_PREPARED"
        or overlay.get("mutation_whitelist") != ["ground.surface_velocity"]
        or overlay.get("protected_trials_opened") != []
    ):
        raise FreezeError("profile overlay manifest is invalid")
    frozen_sources = {
        key: driver.source_record(path) for key, path in SOURCE_PATHS.items()
    }
    frozen_inputs = _input_records()
    destination_paths = _destination_paths()
    destinations = [
        str(path.relative_to(REPO_ROOT)) for path in destination_paths
    ]
    payload = {
        "schema_version": 1,
        "status": "H0_PRIMARY_GRF_SPLIT_V1_COLLECTION_EXECUTION_UNLOCKED",
        "scope": "TEACHER_COLLECTION_AND_CORPUS_FREEZE_NO_ACTOR_UPDATE",
        "authority": {
            "h0_primary_split_teacher_collection_authorized": True,
            "h0_primary_split_supervised_adaptation_execution_authorized": False,
            "actor_updates_authorized": False,
            "critic_updates_authorized": False,
            "ppo_updates_authorized": False,
            "general_training_authorized": False,
            "h0_sep_authorized": False,
            "h0_v25_abc_execution_authorized": False,
            "v25_ab_c_execution_authorized": False,
            "protected_trial_access_authorized": False,
            "reserve_trial_access_authorized": False,
            "detector_retuning_authorized": False,
            "primary_grf_modification_authorized": False,
            "sea_semantic_modification_authorized": False,
            "corridor_authorized": False,
            "runtime_promotion_authorized": False,
        },
        "frozen_sources": frozen_sources,
        "frozen_inputs": frozen_inputs,
        "preflight_receipts": {
            "tests": driver.source_record(TEST_RECEIPT),
            "layout": driver.source_record(LAYOUT_RECEIPT),
        },
        "collection": {
            trial_id: {
                "trial_id": trial_id,
                "plateau_id": "04",
                "speed_mps": trial["speed_mps"],
                "absolute_start_s": trial["collection_absolute_s"],
                "offset_s": trial["collection_offset_s"],
                "seed": trial["collection_seed"],
                "sigma": driver.COLLECTION_SIGMA,
                "steps": driver.EXPECTED_STEPS,
            }
            for trial_id, trial in driver.TRIALS.items()
        },
        "dataset_split": {
            "training_trials": ["02", "04"],
            "validation_trials": ["08"],
            "records_per_state": 2,
            "training_records": 2000,
            "validation_records": 1000,
            "group_leakage_allowed": False,
        },
        "adaptation": {
            "execution_authorized": False,
            "candidate_count": 0,
            "optimizer": "Adam",
            "learning_rate": 1.0e-4,
            "epochs_max": 300,
            "batch_size": 128,
            "patience": 60,
            "anchor_weight": 1.0e-3,
            "clip_weight": 1.0,
            "logstd_weight": 0.0,
            "seed": 123,
            "trainable_scope": "full_mean_actor",
            "logstd_frozen": True,
            "ppo_updates": 0,
            "retry_allowed": False,
        },
        "destinations": destinations,
        "run_root": str(driver.RUN_ROOT.relative_to(REPO_ROOT)),
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "numerical_claim": "macOS-only",
            "windows_schema_and_replay_only": True,
        },
        "old_h0_v25_matrix_reopened": False,
        "protected_trials_opened": [],
        "no_clobber": True,
    }
    _preallocate_destinations(destination_paths)
    strict_io.write_json_exclusive(OUTPUT, payload)
    return payload


if __name__ == "__main__":
    print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
