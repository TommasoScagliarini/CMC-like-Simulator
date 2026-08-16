"""Freeze the sole actor update after the teacher corpus has passed."""

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


OUTPUT = driver.ADAPTATION_LOCK
TEST_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v1_preflight_test_receipt_v2.json"
LAYOUT_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v1_layout_preflight_receipt_v2.json"


class ActorUpdateFreezeError(RuntimeError):
    pass


def _mapping(path: Path) -> dict[str, Any]:
    value = strict_io.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise ActorUpdateFreezeError(f"expected object: {path}")
    return dict(value)


def _require_pass(path: Path, status: str) -> dict[str, Any]:
    value = _mapping(path)
    if value.get("status") != status or value.get("passed") is not True:
        raise ActorUpdateFreezeError(f"required PASS is absent: {path}")
    return value


def freeze() -> dict[str, Any]:
    if os.path.lexists(OUTPUT):
        raise ActorUpdateFreezeError(f"refusing to clobber: {OUTPUT}")
    try:
        driver._verify_lock(driver.COLLECTION_LOCK)
    except Exception as exc:
        raise ActorUpdateFreezeError(
            f"collection lock continuity failed: {exc}"
        ) from exc
    tests = _require_pass(
        TEST_RECEIPT, "PASS_H0_PRIMARY_GRF_SPLIT_PREFLIGHT_TESTS"
    )
    layout = _require_pass(
        LAYOUT_RECEIPT, "PASS_H0_PRIMARY_GRF_SPLIT_LAYOUT_PREFLIGHT"
    )
    frozen_test_sources = tests.get("frozen_sources_at_test")
    if not isinstance(frozen_test_sources, Mapping):
        raise ActorUpdateFreezeError("test source ledger is malformed")
    for key, record in frozen_test_sources.items():
        driver._frozen_path_record(record, f"tests.frozen_sources_at_test.{key}")
    driver._frozen_path_record(
        layout.get("execution_driver"), "layout.execution_driver"
    )
    collection_ledger_path = driver.RUN_ROOT / "collection_execution_ledger.json"
    _require_pass(
        collection_ledger_path, "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN"
    )
    corpus_receipt_path = driver.RUN_ROOT / "corpus" / "receipt.json"
    corpus = _require_pass(
        corpus_receipt_path, "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN"
    )
    if corpus.get("actor_updates") != 0 or corpus.get("ppo_updates") != 0:
        raise ActorUpdateFreezeError("corpus was not produced at zero update")
    corpus_dataset = driver._frozen_path_record(corpus.get("dataset"), "corpus dataset")
    corpus_manifest = driver._frozen_path_record(corpus.get("manifest"), "corpus manifest")
    destination = driver.RUN_ROOT / "adaptation"
    if destination.exists():
        raise ActorUpdateFreezeError(f"adaptation destination exists: {destination}")

    source_paths = {
        "actor_update_freezer": Path(__file__).resolve(),
        "execution_driver": Path(driver.__file__).resolve(),
        "pure_contract": BASELINE_ROOT / "primary_grf_split_adaptation.py",
        "adapt_actor": BASELINE_ROOT / "target_domain_imitation.py",
        "strict_io": VALIDATION_ROOT / "compare_h0_v25_abc.py",
        "warm_start": BASELINE_ROOT / "warm_start.py",
    }
    frozen_inputs: dict[str, Any] = {
        "protocol_lock": driver.source_record(driver.PROTOCOL_LOCK),
        "collection_lock": driver.source_record(driver.COLLECTION_LOCK),
        "collection_ledger": driver.source_record(collection_ledger_path),
        "corpus_receipt": driver.source_record(corpus_receipt_path),
        "corpus_manifest": driver.source_record(corpus_manifest),
        "corpus_dataset": driver.source_record(corpus_dataset),
        "h0_module_state": driver.source_record(driver.H0_MODULE / "module_state.pkl"),
        "h0_module_ctor": driver.source_record(
            driver.H0_MODULE / "class_and_ctor_args.pkl"
        ),
        "h0_module_metadata": driver.source_record(driver.H0_MODULE / "metadata.json"),
    }
    for trial in ("02", "04", "08"):
        frozen_inputs[f"trial_{trial}_terminal_receipt"] = driver.source_record(
            driver.RUN_ROOT
            / "collection"
            / f"trial_{trial}"
            / "terminal_receipt.json"
        )
    payload = {
        "schema_version": 1,
        "status": "H0_PRIMARY_GRF_SPLIT_V1_ACTOR_UPDATE_UNLOCKED",
        "scope": "ONE_SHOT_FULL_MEAN_ACTOR_SUPERVISED_UPDATE_ONLY",
        "authority": {
            "h0_primary_split_teacher_collection_authorized": False,
            "h0_primary_split_supervised_adaptation_execution_authorized": True,
            "actor_updates_authorized": True,
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
        "preflight_receipts": {
            "tests": driver.source_record(TEST_RECEIPT),
            "layout": driver.source_record(LAYOUT_RECEIPT),
        },
        "interstage_gate": {
            "all_teacher_gates_pass": True,
            "corpus_materialized_and_hash_frozen": True,
            "teacher_manifest_strict_atomic_no_clobber": True,
            "actor_update_may_start": True,
        },
        "adaptation": {
            "candidate_count_max": 1,
            "optimizer": "Adam",
            "learning_rate": 1.0e-4,
            "epochs_max": 300,
            "batch_size": 128,
            "patience": 60,
            "anchor_weight": 1.0e-3,
            "clip_weight": 1.0,
            "logstd_weight": 0.0,
            "seed": 123,
            "training_records": 2000,
            "validation_records": 1000,
            "trainable_scope": "full_mean_actor",
            "logstd_frozen": True,
            "critic_non_actor_unchanged": True,
            "retry_allowed": False,
        },
        "destination": str(destination.relative_to(REPO_ROOT)),
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "no_clobber": True,
    }
    destination.mkdir(parents=True, exist_ok=False)
    strict_io.write_json_exclusive(OUTPUT, payload)
    return payload


if __name__ == "__main__":
    print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
