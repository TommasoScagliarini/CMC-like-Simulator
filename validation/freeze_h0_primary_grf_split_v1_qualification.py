"""Freeze zero-update closed-loop qualification for the adapted H0 actor."""

from __future__ import annotations

import json
import os
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


OUTPUT = driver.QUALIFICATION_LOCK


class QualificationFreezeError(RuntimeError):
    pass


def _mapping(path: Path) -> dict[str, Any]:
    value = strict_io.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise QualificationFreezeError(f"expected object: {path}")
    return dict(value)


def _require_frozen_path(record: Any, expected: Path, label: str) -> Path:
    try:
        resolved = driver._frozen_path_record(record, label)
    except Exception as exc:
        raise QualificationFreezeError(f"{label} is invalid: {exc}") from exc
    if resolved != expected.resolve():
        raise QualificationFreezeError(f"{label} path drifted")
    return resolved


def _destination_paths() -> list[Path]:
    qualification_root = driver.RUN_ROOT / "qualification"
    destinations: list[Path] = []
    for role in ("reference", "candidate"):
        for trial in ("02", "04", "08"):
            for selection in ("deterministic", "stochastic"):
                destinations.append(
                    qualification_root / role / f"trial_{trial}_{selection}"
                )
    destinations.append(qualification_root / "gates")
    return destinations


def _preallocate(destinations: list[Path]) -> None:
    qualification_root = driver.RUN_ROOT / "qualification"
    if qualification_root.exists():
        raise QualificationFreezeError(
            f"qualification root already exists: {qualification_root}"
        )
    for destination in destinations:
        destination.mkdir(parents=True, exist_ok=False)
        if any(destination.iterdir()):
            raise QualificationFreezeError(
                f"qualification destination is nonempty: {destination}"
            )


def freeze() -> dict[str, Any]:
    if os.path.lexists(OUTPUT):
        raise QualificationFreezeError(f"refusing to clobber: {OUTPUT}")
    try:
        driver._verify_lock(driver.ADAPTATION_LOCK)
    except Exception as exc:
        raise QualificationFreezeError(
            f"actor-update lock continuity failed: {exc}"
        ) from exc
    try:
        driver._verify_lock(driver.COLLECTION_LOCK)
    except Exception as exc:
        raise QualificationFreezeError(
            f"collection lock continuity failed: {exc}"
        ) from exc
    adaptation_ledger_path = driver.RUN_ROOT / "adaptation_execution_ledger.json"
    adaptation_ledger = _mapping(adaptation_ledger_path)
    if (
        adaptation_ledger.get("status")
        != "PASS_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
        or adaptation_ledger.get("passed") is not True
        or adaptation_ledger.get("ppo_updates") != 0
        or adaptation_ledger.get("protected_trials_opened") != []
        or adaptation_ledger.get("candidate_created") is not True
        or adaptation_ledger.get("actor_update_candidates") != 1
    ):
        raise QualificationFreezeError("offline adaptation has not passed")
    _require_frozen_path(
        adaptation_ledger.get("adaptation_lock"),
        driver.ADAPTATION_LOCK,
        "adaptation ledger actor-update lock",
    )
    _require_frozen_path(
        adaptation_ledger.get("collection_ledger"),
        driver.RUN_ROOT / "collection_execution_ledger.json",
        "adaptation ledger collection ledger",
    )
    adaptation_receipt_path = driver.RUN_ROOT / "adaptation" / "receipt.json"
    adaptation_receipt = _mapping(adaptation_receipt_path)
    if (
        adaptation_receipt.get("status")
        != "PASS_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
        or adaptation_receipt.get("passed") is not True
        or adaptation_receipt.get("ppo_updates") != 0
        or adaptation_receipt.get("protected_trials_opened") != []
    ):
        raise QualificationFreezeError("candidate receipt is not a PASS")
    offline_gate_path = driver.RUN_ROOT / "adaptation" / "offline_gate.json"
    _require_frozen_path(
        adaptation_receipt.get("offline_gate"),
        offline_gate_path,
        "adaptation receipt offline gate",
    )
    offline_gate = _mapping(offline_gate_path)
    if (
        offline_gate.get("status")
        != "PASS_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
        or offline_gate.get("passed") is not True
        or offline_gate.get("ppo_updates") != 0
        or offline_gate.get("protected_trials_opened") != []
    ):
        raise QualificationFreezeError("offline gate is not an exact PASS")
    _require_frozen_path(
        offline_gate.get("dataset_receipt"),
        driver.RUN_ROOT / "corpus" / "receipt.json",
        "offline gate corpus receipt",
    )
    _require_frozen_path(
        offline_gate.get("adaptation_report"),
        driver.RUN_ROOT / "adaptation" / "adaptation_report.json",
        "offline gate adaptation report",
    )
    _require_frozen_path(
        offline_gate.get("adaptation_history"),
        driver.RUN_ROOT / "adaptation" / "adaptation_history.json",
        "offline gate adaptation history",
    )
    candidate_dir = (
        REPO_ROOT / str(adaptation_receipt.get("candidate_path", ""))
    ).resolve()
    if candidate_dir != (driver.RUN_ROOT / "adaptation" / "rl_module_target_adapted").resolve():
        raise QualificationFreezeError("candidate path drifted")
    candidate_records = {
        "module_state": driver.source_record(candidate_dir / "module_state.pkl"),
        "class_and_ctor_args": driver.source_record(
            candidate_dir / "class_and_ctor_args.pkl"
        ),
        "metadata": driver.source_record(candidate_dir / "metadata.json"),
    }
    gate_candidate = offline_gate.get("candidate_module")
    if not isinstance(gate_candidate, Mapping):
        raise QualificationFreezeError("offline gate candidate ledger is malformed")
    for filename, record_key in (
        ("module_state.pkl", "module_state"),
        ("class_and_ctor_args.pkl", "class_and_ctor_args"),
        ("metadata.json", "metadata"),
    ):
        _require_frozen_path(
            gate_candidate.get(filename),
            candidate_dir / filename,
            f"offline gate candidate {filename}",
        )
        if gate_candidate.get(filename) != candidate_records[record_key]:
            raise QualificationFreezeError(
                f"offline gate candidate {filename} record drifted"
            )
    adaptation_report = _mapping(
        driver.RUN_ROOT / "adaptation" / "adaptation_report.json"
    )
    if (
        adaptation_report.get("ppo_updates") != 0
        or adaptation_report.get("adapted_actor_digest")
        != adaptation_receipt.get("candidate_actor_digest")
    ):
        raise QualificationFreezeError("adaptation report provenance drifted")
    destination_paths = _destination_paths()
    destinations = [
        str(path.relative_to(REPO_ROOT)) for path in destination_paths
    ]
    source_paths = {
        "qualification_freezer": Path(__file__).resolve(),
        "execution_driver": Path(driver.__file__).resolve(),
        "pure_contract": BASELINE_ROOT / "primary_grf_split_adaptation.py",
        "comparator": VALIDATION_ROOT / "compare_h0_primary_grf_split.py",
        "strict_io": VALIDATION_ROOT / "compare_h0_v25_abc.py",
        "rollout_eval": BASELINE_ROOT / "rollout_eval.py",
        "environment": REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py",
        "phase_fsm": REPO_ROOT / "Trajectory Generator" / "prosthetic_phase_fsm.py",
        "simulation_runner": REPO_ROOT / "simulation_runner.py",
        "model_loader": REPO_ROOT / "model_loader.py",
        "online_grf": REPO_ROOT / "online_grf.py",
    }
    frozen_inputs = {
        "protocol_lock": driver.source_record(driver.PROTOCOL_LOCK),
        "actor_update_lock": driver.source_record(driver.ADAPTATION_LOCK),
        "collection_lock": driver.source_record(driver.COLLECTION_LOCK),
        "adaptation_execution_ledger": driver.source_record(adaptation_ledger_path),
        "adaptation_receipt": driver.source_record(adaptation_receipt_path),
        "offline_gate": driver.source_record(
            offline_gate_path
        ),
        "profile_overlay_manifest": driver.source_record(driver.INPUT_MANIFEST),
        "noise_tape_manifest": driver.source_record(driver.NOISE_MANIFEST),
        "h0_module_state": driver.source_record(driver.H0_MODULE / "module_state.pkl"),
    }
    for key, record in candidate_records.items():
        frozen_inputs[f"candidate_{key}"] = record
    for trial_id, trial in driver.TRIALS.items():
        frozen_inputs[f"trial_{trial_id}_setup"] = driver.source_record(trial["setup"])
        frozen_inputs[f"trial_{trial_id}_primary_overlay"] = driver.source_record(
            driver.INPUT_ROOT / f"trial_{trial_id}_primary_surface_velocity.json"
        )
        frozen_inputs[f"trial_{trial_id}_analog_overlay"] = driver.source_record(
            driver.INPUT_ROOT / f"trial_{trial_id}_analog_surface_velocity.json"
        )
        frozen_inputs[f"trial_{trial_id}_qualification_noise_tape"] = driver.source_record(
            driver.NOISE_ROOT
            / f"qualification_trial_{trial_id}_standard_normal.npz"
        )
    frozen_inputs["qualification_deterministic_noise_tape"] = driver.source_record(
        driver.NOISE_ROOT / "qualification_deterministic_all_zero.npz"
    )
    payload = {
        "schema_version": 1,
        "status": "H0_PRIMARY_GRF_SPLIT_V1_QUALIFICATION_UNLOCKED",
        "scope": "ZERO_UPDATE_CLOSED_LOOP_DEVELOPMENT_QUALIFICATION_ONLY",
        "authority": {
            "h0_primary_split_zero_update_qualification_authorized": True,
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
        "frozen_sources": {
            key: driver.source_record(path) for key, path in source_paths.items()
        },
        "frozen_inputs": frozen_inputs,
        "preflight_receipts": {},
        "candidate_module": candidate_records,
        "conditions": {
            trial_id: {
                "trial_id": trial_id,
                "plateau_id": "04",
                "speed_mps": trial["speed_mps"],
                "absolute_start_s": trial["qualification_absolute_s"],
                "offset_s": trial["qualification_offset_s"],
                "seed": trial["qualification_seed"],
                "action_selections": ["deterministic", "stochastic"],
                "stochastic_sigma": driver.EXPECTED_SIGMA,
                "steps": driver.EXPECTED_STEPS,
            }
            for trial_id, trial in driver.TRIALS.items()
        },
        "execution_order": [
            "all_six_teacher_references",
            "all_six_autonomous_candidates_if_references_pass",
        ],
        "destinations": destinations,
        "retry_allowed": False,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "no_clobber": True,
    }
    _preallocate(destination_paths)
    strict_io.write_json_exclusive(OUTPUT, payload)
    return payload


if __name__ == "__main__":
    print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
