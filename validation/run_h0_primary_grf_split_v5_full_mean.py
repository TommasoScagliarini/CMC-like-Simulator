"""One-shot H0 primary-split V5 full-mean adaptation and final holdout.

The protocol consumes the immutable passing V3 train corpus, fits exactly one
new actor from the original H0, freezes it, and only then gives the frozen V3
semantic-replay engine access to seed 125.  V3 artifacts and its terminal FAIL
remain untouched.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import numbers
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_v25_abc as common_gates  # noqa: E402
import h0_primary_grf_split_v5_freeze_contract as contract  # noqa: E402
import primary_grf_split_adaptation as split_contract  # noqa: E402
import run_h0_primary_grf_split_v3_semantic_replay as v3  # noqa: E402
import target_domain_imitation as imitation  # noqa: E402
import warm_start  # noqa: E402


LOCK = REPO_ROOT / contract.LOCK_RELATIVE
RUN_ROOT = REPO_ROOT / contract.RUN_ROOT_RELATIVE
ATTEMPT_CLAIM = RUN_ROOT / "attempt_claim.json"
ADAPTATION_DIR = RUN_ROOT / "adaptation"
CANDIDATE_DIR = ADAPTATION_DIR / "rl_module_target_adapted"
CANDIDATE_FREEZE = ADAPTATION_DIR / "candidate_freeze.json"
HOLDOUT_ACCESS_CLAIM = RUN_ROOT / "holdout_access_claim.json"
HOLDOUT_REPLAY_DIR = RUN_ROOT / "replay" / "seed_125"
HOLDOUT_REPLAY_RECEIPT = RUN_ROOT / "holdout_replay_receipt.json"
HOLDOUT_DIR = RUN_ROOT / "holdout"
EXECUTION_LEDGER = RUN_ROOT / "execution_ledger.json"
RUNNER = Path(__file__).resolve()
V4_PREEXECUTION_FAILURE = REPO_ROOT / contract.V4_PREEXECUTION_FAILURE_RELATIVE
V4_HARDENED_TEMPLATE = (
    REPO_ROOT
    / contract.INHERITED_SOURCE_RELATIVE_PATHS["v4_hardened_runner_template"]
)
H0_MODULE = v3.H0_MODULE
H0_CONFIG = v3.H0_CONFIG

PROTOCOL_ID = contract.PROTOCOL_ID
CANDIDATE_ID = contract.CANDIDATE_ID
REVISION = contract.REVISION
TRAIN_SEEDS = contract.TRAIN_SEEDS
FINAL_HOLDOUT_SEED = contract.FINAL_HOLDOUT_SEED
FIT = dict(contract.FIT)
EXPECTED_STEPS = contract.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = contract.EXPECTED_ACTOR_FEATURES
WORKER_TIMEOUT_S = 2400.0

LOCK_TOP_LEVEL_KEYS = {
    "schema_version",
    "status",
    "protocol_id",
    "revision",
    "candidate_id",
    "run_root",
    "train_seeds",
    "final_holdout_seed",
    "expected_steps",
    "expected_actor_features",
    "event_contract_id",
    "so_policy_id",
    "so_policy",
    "fit",
    "trainable_scope",
    "logstd_policy",
    "offline_thresholds",
    "train_only_preflight_evidence",
    "destinations",
    "authority",
    "sources",
    "inputs",
    "v3_terminal_expectation",
    "v4_preexecution_expectation",
    "actor_update_candidate_count",
    "critic_updates",
    "ppo_updates",
    "protected_trials_opened",
}


class H0PrimarySplitV5Error(RuntimeError):
    """Raised whenever the V5 one-shot boundary cannot be proven."""


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    try:
        with resolved.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=lambda token: (_ for _ in ()).throw(
                    ValueError(f"non-finite JSON token {token}")
                ),
            )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise H0PrimarySplitV5Error(f"invalid strict JSON object: {resolved}") from exc
    if not isinstance(value, Mapping):
        raise H0PrimarySplitV5Error(f"expected JSON object: {resolved}")
    _require_finite_json(value, str(resolved))
    return dict(value)


def _require_finite_json(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, numbers.Real):
        if not math.isfinite(float(value)):
            raise H0PrimarySplitV5Error(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise H0PrimarySplitV5Error(f"{label} has an invalid key")
            _require_finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise H0PrimarySplitV5Error(f"{label} contains a non-JSON value")


def _write_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    _require_finite_json(value, "JSON output")
    return v3._write_json_exclusive(path, dict(value))


def source_record(path: str | Path) -> dict[str, Any]:
    return v3.source_record(path)


def _canonical_bytes(value: Any) -> bytes:
    return common_gates.canonical_json_bytes(value)


def _record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and _canonical_bytes(record) == _canonical_bytes(
        source_record(path)
    )


def _require_int(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        raise H0PrimarySplitV5Error(f"{label} must be a non-negative integer")
    return value


def verify_lock() -> dict[str, Any]:
    if not LOCK.is_file():
        raise H0PrimarySplitV5Error("V5 execution lock is absent")
    observed = _strict_mapping(LOCK)
    from freeze_h0_primary_grf_split_v5_execution import _build_lock_payload

    expected = _build_lock_payload()
    if set(observed) != LOCK_TOP_LEVEL_KEYS or _canonical_bytes(observed) != (
        _canonical_bytes(expected)
    ):
        raise H0PrimarySplitV5Error("V5 execution lock drifted")
    return observed


def _validate_v3_terminal_lineage() -> tuple[dict[str, Any], dict[str, Any]]:
    ledger = _strict_mapping(
        REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
    )
    for key, expected in contract.V3_TERMINAL_EXPECTATION.items():
        if ledger.get(key) != expected:
            raise H0PrimarySplitV5Error(f"V3 terminal lineage drifted at {key}")
    if (
        ledger.get("protected_trials_opened") != []
        or _require_int(ledger.get("actor_update_candidates"), "V3 actor candidates")
        != 1
        or _require_int(ledger.get("critic_updates"), "V3 critic updates") != 0
        or _require_int(ledger.get("ppo_updates"), "V3 PPO updates") != 0
    ):
        raise H0PrimarySplitV5Error("V3 terminal authority/footer drifted")

    gate = _strict_mapping(
        REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["failed_offline_gate"]
    )
    metrics = gate.get("train_fit_metrics")
    if (
        gate.get("status") != "FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE"
        or gate.get("passed") is not False
        or not isinstance(metrics, Mapping)
        or float(metrics["adapted_student"]["rmse"])
        != contract.V3_FAILED_METRICS["student_rmse"]
        or float(metrics["adapted_teacher"]["rmse"])
        != contract.V3_FAILED_METRICS["teacher_rmse"]
    ):
        raise H0PrimarySplitV5Error("V3 failed offline gate drifted")

    v3_root = (
        REPO_ROOT
        / "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay"
    )
    forbidden = (
        v3_root / "adaptation/candidate_freeze.json",
        v3_root / "holdout_access_claim.json",
        v3_root / "replay/seed_125",
        v3_root / "holdout",
    )
    if any(os.path.lexists(path) for path in forbidden):
        raise H0PrimarySplitV5Error(
            "V3 seed-125/candidate-freeze boundary changed after terminal FAIL"
        )
    return ledger, gate


def _validate_v4_preexecution_failure() -> dict[str, Any]:
    failure = _strict_mapping(V4_PREEXECUTION_FAILURE)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "failure",
        "failure_stage",
        "execution_error",
        "execution_lock",
        "locked_preflight_receipt",
        "locked_runner",
        "observed_preflight_receipt_exists",
        "observed_runner",
        "v4_execution_started",
        "attempt_claim_created",
        "run_root_exists",
        "seed125_semantic_accessed",
        "actor_update_candidates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
        "v4_retry_allowed",
        "next_stage",
    }
    if set(failure) != expected_keys:
        raise H0PrimarySplitV5Error("V4 pre-execution failure schema drifted")
    for key, expected in contract.V4_PREEXECUTION_EXPECTATION.items():
        if failure.get(key) != expected:
            raise H0PrimarySplitV5Error(
                f"V4 pre-execution failure drifted at {key}"
            )
    if failure.get("execution_error") != (
        "V4FreezeError: V4 preflight source drifted: runner"
    ):
        raise H0PrimarySplitV5Error("V4 pre-execution error identity drifted")
    v4_lock = REPO_ROOT / "validation/h0_primary_grf_split_v4_execution_lock.json"
    if not _record_matches(failure.get("execution_lock"), v4_lock):
        raise H0PrimarySplitV5Error("V4 failed execution-lock provenance drifted")
    if not _record_matches(failure.get("observed_runner"), V4_HARDENED_TEMPLATE):
        raise H0PrimarySplitV5Error("V4 observed hardened runner drifted")
    if failure.get("locked_runner") != {
        "path": "validation/run_h0_primary_grf_split_v4_full_mean.py",
        "sha256": "f638d2559bb2b7e8e7b8ae2474ce32ac58d9fff9b14b359399ffbe8fa340d8a8",
        "size_bytes": 45916,
    }:
        raise H0PrimarySplitV5Error("V4 originally locked runner record drifted")
    if failure.get("locked_preflight_receipt") != {
        "path": "validation/h0_primary_grf_split_v4_preflight_receipt.json",
        "sha256": "eecddfe5a9d238b1dbc983976c1c251dc7cd72da5e3e141ff952bbab98b83740",
        "size_bytes": 4977,
    }:
        raise H0PrimarySplitV5Error("V4 locked preflight record drifted")
    return failure


def _validate_v3_corpus() -> tuple[dict[str, Any], dict[str, Any], Path]:
    # This validator parses only the V3 corpus and train-seed 123/124 receipts.
    # It never calls historical_inputs(125).
    receipt, manifest, corpus_path = v3._validate_corpus_receipt()
    if (
        receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN"
        or receipt.get("passed") is not True
        or manifest.get("training_trials") != ["123", "124"]
        or manifest.get("validation_trials") != []
        or manifest.get("records") != 2000
        or manifest.get("validation_records") != 0
    ):
        raise H0PrimarySplitV5Error("V3 corpus is not the frozen train-only PASS")
    expected_paths = {
        "corpus": corpus_path,
        "corpus_manifest": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["corpus_manifest"],
        "corpus_receipt": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["corpus_receipt"],
        "v4_preexecution_failure": V4_PREEXECUTION_FAILURE,
    }
    lock = verify_lock()
    inputs = lock.get("inputs")
    if not isinstance(inputs, Mapping):
        raise H0PrimarySplitV5Error("V5 lock inputs are missing")
    for key, expected in expected_paths.items():
        if not _record_matches(inputs.get(key), expected):
            raise H0PrimarySplitV5Error(f"V5 lock inherited {key} drifted")
    return receipt, manifest, corpus_path


def _attempt_claim_payload() -> dict[str, Any]:
    return {
        "schema_version": 5,
        "status": "H0_PRIMARY_SPLIT_V5_EXECUTION_ATTEMPT_CLAIMED",
        "protocol_id": PROTOCOL_ID,
        "candidate_id": CANDIDATE_ID,
        "run_root": contract.RUN_ROOT_RELATIVE,
        "execution_lock": source_record(LOCK),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "actor_update_candidates_authorized": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _claim_attempt_root() -> Path:
    verify_lock()
    _validate_v3_terminal_lineage()
    _validate_v4_preexecution_failure()
    RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    try:
        RUN_ROOT.mkdir()
    except FileExistsError as exc:
        raise H0PrimarySplitV5Error(
            f"V5 execution attempt already exists/no retry: {RUN_ROOT}"
        ) from exc
    return _write_json_exclusive(ATTEMPT_CLAIM, _attempt_claim_payload())


def _verify_attempt_claim() -> dict[str, Any]:
    observed = _strict_mapping(ATTEMPT_CLAIM)
    expected = _attempt_claim_payload()
    if _canonical_bytes(observed) != _canonical_bytes(expected):
        raise H0PrimarySplitV5Error("V5 attempt claim drifted")
    return observed


def _claim_directory(path: str | Path, expected: Path) -> Path:
    destination = Path(path).expanduser().resolve()
    if destination != expected.resolve():
        raise H0PrimarySplitV5Error(
            f"non-canonical V5 destination: {destination} != {expected.resolve()}"
        )
    _verify_attempt_claim()
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        destination.mkdir()
    except FileExistsError as exc:
        raise H0PrimarySplitV5Error(
            f"V5 destination already exists/no-clobber: {destination}"
        ) from exc
    return destination


def _load_train_corpus() -> tuple[Any, ...]:
    import numpy as np

    _receipt, manifest, corpus_path = _validate_v3_corpus()
    with np.load(corpus_path, allow_pickle=False) as archive:
        expected_keys = {
            "observations",
            "actions",
            "actor_feature_names",
            "trial_ids",
            "view_roles",
            "group_ids",
            "training_indices",
            "validation_indices",
        }
        if set(archive.files) != expected_keys:
            raise H0PrimarySplitV5Error("V3 corpus NPZ schema drifted")
        observations = np.ascontiguousarray(archive["observations"])
        targets = np.ascontiguousarray(archive["actions"])
        names = np.ascontiguousarray(archive["actor_feature_names"])
        trial_ids = np.ascontiguousarray(archive["trial_ids"])
        roles = np.ascontiguousarray(archive["view_roles"])
        groups = np.ascontiguousarray(archive["group_ids"])
        train = np.ascontiguousarray(archive["training_indices"])
        validation = np.ascontiguousarray(archive["validation_indices"])
    if (
        observations.dtype != np.float32
        or observations.shape != (2000, EXPECTED_ACTOR_FEATURES)
        or targets.dtype != np.float32
        or targets.shape != (2000, 2)
        or names.dtype != np.dtype("U64")
        or names.shape != (EXPECTED_ACTOR_FEATURES,)
        or roles.dtype != np.dtype("U16")
        or roles.shape != (2000,)
        or train.dtype != np.int64
        or not np.array_equal(train, np.arange(2000, dtype=np.int64))
        or validation.dtype != np.int64
        or validation.shape != (0,)
        or set(trial_ids[train].astype(str).tolist()) != {"123", "124"}
        or set(roles.astype(str).tolist())
        != {split_contract.STUDENT_VIEW, split_contract.TEACHER_VIEW}
        or len(set(groups.astype(str).tolist())) != 1000
        or not np.all(np.isfinite(observations))
        or not np.all(np.isfinite(targets))
        or tuple(names.astype(str).tolist())
        != tuple(manifest["actor_feature_names"])
    ):
        raise H0PrimarySplitV5Error("V3 train corpus arrays drifted")
    digests = {
        "observations_sha256": split_contract.array_sha256(observations),
        "actions_sha256": split_contract.array_sha256(targets),
        "training_indices_sha256": split_contract.array_sha256(train),
        "validation_indices_sha256": split_contract.array_sha256(validation),
    }
    if any(manifest.get(key) != digest for key, digest in digests.items()):
        raise H0PrimarySplitV5Error("V3 train corpus digest drifted")
    return observations, targets, names, roles, train, corpus_path, manifest


def _module_logits(module_path: Path, observations: Any) -> Any:
    return v3._module_logits(module_path, observations)


def full_mean_update_audit(
    source_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> dict[str, Any]:
    """Prove that changes are mean-network-only and logstd is bit exact."""
    import numpy as np

    expected_keys = {
        "pi_encoder.0.weight",
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
        "pi.0.0.weight",
        "pi.0.0.bias",
        "pi.0.2.weight",
        "pi.0.2.bias",
        "pi.1.weight",
        "pi.1.bias",
    }
    if set(source_state) != expected_keys or set(candidate_state) != expected_keys:
        raise H0PrimarySplitV5Error("actor state schema drifted")
    arrays: dict[str, tuple[Any, Any]] = {}
    changed: list[str] = []
    all_finite = True
    for key in sorted(expected_keys):
        source = np.asarray(warm_start._as_numpy(source_state[key]))
        candidate = np.asarray(warm_start._as_numpy(candidate_state[key]))
        if source.shape != candidate.shape:
            raise H0PrimarySplitV5Error(f"actor tensor shape drifted: {key}")
        arrays[key] = (source, candidate)
        all_finite = all_finite and bool(np.all(np.isfinite(candidate)))
        if not np.array_equal(source, candidate):
            changed.append(key)

    output_source_w, output_candidate_w = arrays["pi.1.weight"]
    output_source_b, output_candidate_b = arrays["pi.1.bias"]
    logstd_exact = bool(
        np.array_equal(output_source_w[2:], output_candidate_w[2:])
        and np.array_equal(output_source_b[2:], output_candidate_b[2:])
    )
    mean_output_changed = bool(
        not np.array_equal(output_source_w[:2], output_candidate_w[:2])
        or not np.array_equal(output_source_b[:2], output_candidate_b[:2])
    )
    hidden_keys = expected_keys - {"pi.1.weight", "pi.1.bias"}
    hidden_changed = any(key in changed for key in hidden_keys)
    aliases_exact = all(
        np.array_equal(arrays[left][1], arrays[right][1])
        for left, right in (
            ("pi_encoder.0.weight", "pi.0.0.weight"),
            ("pi_encoder.0.bias", "pi.0.0.bias"),
            ("pi_encoder.2.weight", "pi.0.2.weight"),
            ("pi_encoder.2.bias", "pi.0.2.bias"),
        )
    )
    clock_columns_zero = all(
        np.count_nonzero(arrays[key][1][:, :2]) == 0
        for key in ("pi_encoder.0.weight", "pi.0.0.weight")
    )
    return {
        "schema_version": 1,
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "changed_keys": changed,
        "all_candidate_actor_tensors_finite": all_finite,
        "hidden_mean_network_changed": hidden_changed,
        "mean_output_changed": mean_output_changed,
        "logstd_parameter_rows_bit_exact": logstd_exact,
        "encoder_aliases_bit_exact": aliases_exact,
        "disabled_clock_columns_zero": clock_columns_zero,
        "changes_confined_to_full_mean_network": bool(
            changed and hidden_changed and mean_output_changed and logstd_exact
        ),
    }


def adapt_worker(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    _validate_v3_terminal_lineage()
    _validate_v4_preexecution_failure()
    destination = _claim_directory(output_dir, ADAPTATION_DIR)
    import numpy as np

    observations, targets, names, roles, train, corpus_path, _manifest = (
        _load_train_corpus()
    )
    dataset = {
        "observations": observations,
        "actions": targets,
        "actor_feature_names": names,
    }
    report = imitation.adapt_actor(
        H0_MODULE.resolve(),
        dataset,
        destination,
        seed=FIT["seed"],
        epochs=FIT["epochs"],
        batch_size=FIT["batch_size"],
        learning_rate=FIT["learning_rate"],
        validation_fraction=FIT["validation_fraction"],
        patience=FIT["patience"],
        clip_weight=FIT["clip_weight"],
        logstd_weight=FIT["logstd_weight"],
        anchor_weight=FIT["anchor_weight"],
        freeze_logstd_head=True,
        trainable_first_layer_features=None,
        selection_mode=FIT["selection_mode"],
    )
    source_logits = _module_logits(H0_MODULE, observations)
    candidate_logits = _module_logits(CANDIDATE_DIR, observations)
    metrics = split_contract.offline_adaptation_gate(
        source_predictions=source_logits[:, :2],
        adapted_predictions=candidate_logits[:, :2],
        targets=targets,
        validation_indices=train,
        view_roles=roles,
        all_adapted_logits=candidate_logits,
    )
    source_state = warm_start.load_module_state(H0_MODULE)
    candidate_state = warm_start.load_module_state(CANDIDATE_DIR)
    scope = full_mean_update_audit(source_state, candidate_state)
    hyperparameters = report.get("hyperparameters")
    checks = {
        **metrics["checks"],
        "fixed_final_epoch_mode": report.get("selection_mode")
        == "fixed_final_epoch",
        "epochs_exact_400": report.get("epochs_run") == 400
        and report.get("best_epoch") == 400,
        "zero_validation_samples": report.get("validation_samples") == 0,
        "all_2000_rows_used": report.get("training_samples") == 2000,
        "full_mean_scope_declared": isinstance(hyperparameters, Mapping)
        and hyperparameters.get("trainable_first_layer_features") is None,
        "hidden_mean_network_changed": bool(
            scope["hidden_mean_network_changed"]
        ),
        "mean_output_changed": bool(scope["mean_output_changed"]),
        "changes_confined_to_full_mean_network": bool(
            scope["changes_confined_to_full_mean_network"]
        ),
        "all_candidate_actor_tensors_finite": bool(
            scope["all_candidate_actor_tensors_finite"]
        ),
        "encoder_aliases_bit_exact": bool(scope["encoder_aliases_bit_exact"]),
        "disabled_clock_columns_zero": bool(scope["disabled_clock_columns_zero"]),
        "logstd_parameters_bit_exact": bool(
            scope["logstd_parameter_rows_bit_exact"]
        ),
        "logstd_outputs_bit_exact": bool(
            np.array_equal(source_logits[:, 2:], candidate_logits[:, 2:])
        ),
        "actor_digest_changed": warm_start.actor_state_digest(source_state)
        != warm_start.actor_state_digest(candidate_state),
        "save_reload_actor_exact": bool(report["save_reload"]["exact"]),
        "non_actor_exact_or_absent": report["non_actor_verification"]
        in {"exact", "not_available_in_inference_only_rl_module"},
        "actor_layout_35": len(names) == EXPECTED_ACTOR_FEATURES,
        "source_is_original_h0": Path(report["source_checkpoint"]).resolve()
        == H0_MODULE.resolve(),
        "v3_failed_candidate_not_reused": True,
        "v4_candidate_not_reused": True,
    }
    passed = all(checks.values())
    source_h0 = {
        "state": source_record(H0_MODULE / "module_state.pkl"),
        "ctor": source_record(H0_MODULE / "class_and_ctor_args.pkl"),
        "metadata": source_record(H0_MODULE / "metadata.json"),
        "config": source_record(H0_CONFIG),
    }
    actor_manifest = {
        "schema_version": 5,
        "candidate_id": CANDIDATE_ID,
        "observation_contract_id": "primary_grf_split_v1",
        "event_contract_id": "legacy_events_v1",
        "actor_feature_count": EXPECTED_ACTOR_FEATURES,
        "actor_feature_names": names.astype(str).tolist(),
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "logstd_policy": contract.LOGSTD_POLICY,
        "actor_digest": warm_start.actor_state_digest(candidate_state),
        "module_state_sha256": v3.sha256_file(CANDIDATE_DIR / "module_state.pkl"),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "v3_corpus": source_record(corpus_path),
        "v3_terminal_ledger": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
        ),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "source_h0": source_h0,
    }
    manifest_path = _write_json_exclusive(
        CANDIDATE_DIR / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        actor_manifest,
    )
    gate = {
        "schema_version": 5,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V5_OFFLINE"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V5_OFFLINE"
        ),
        "passed": passed,
        "checks": checks,
        "train_fit_metrics": metrics,
        "full_mean_update_audit": scope,
        "fit": FIT,
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "v3_corpus": source_record(corpus_path),
        "v3_terminal_ledger": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
        ),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "candidate_actor_digest": warm_start.actor_state_digest(candidate_state),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = _write_json_exclusive(destination / "offline_gate.json", gate)
    receipt = {
        "schema_version": 5,
        "status": gate["status"],
        "passed": passed,
        "candidate_module_state": source_record(CANDIDATE_DIR / "module_state.pkl"),
        "candidate_module_ctor": source_record(
            CANDIDATE_DIR / "class_and_ctor_args.pkl"
        ),
        "candidate_module_metadata": source_record(CANDIDATE_DIR / "metadata.json"),
        "actor_feature_manifest": source_record(manifest_path),
        "adaptation_report": source_record(destination / "adaptation_report.json"),
        "offline_gate": source_record(gate_path),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "v3_corpus": source_record(corpus_path),
        "v3_corpus_receipt": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus_receipt"]
        ),
        "v3_corpus_manifest": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus_manifest"]
        ),
        "v3_terminal_ledger": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
        ),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "source_h0": source_h0,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise H0PrimarySplitV5Error("V5 offline adaptation gate failed")
    return receipt


def _validate_candidate_receipt() -> tuple[dict[str, Any], Path]:
    _validate_v3_terminal_lineage()
    _validate_v4_preexecution_failure()
    _validate_v3_corpus()
    receipt = _strict_mapping(ADAPTATION_DIR / "receipt.json")
    if (
        set(receipt) != contract.ADAPTATION_RECEIPT_KEYS
        or receipt.get("schema_version") != 5
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V5_OFFLINE"
        or receipt.get("passed") is not True
        or receipt.get("actor_updates") != 1
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise H0PrimarySplitV5Error("V5 candidate receipt is not a PASS")
    expected_records = {
        "candidate_module_state": CANDIDATE_DIR / "module_state.pkl",
        "candidate_module_ctor": CANDIDATE_DIR / "class_and_ctor_args.pkl",
        "candidate_module_metadata": CANDIDATE_DIR / "metadata.json",
        "actor_feature_manifest": CANDIDATE_DIR
        / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        "adaptation_report": ADAPTATION_DIR / "adaptation_report.json",
        "offline_gate": ADAPTATION_DIR / "offline_gate.json",
        "execution_lock": LOCK,
        "attempt_claim": ATTEMPT_CLAIM,
        "v3_corpus": REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus"],
        "v3_corpus_receipt": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["corpus_receipt"],
        "v3_corpus_manifest": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["corpus_manifest"],
        "v3_terminal_ledger": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"],
        "v4_preexecution_failure": V4_PREEXECUTION_FAILURE,
    }
    for key, path in expected_records.items():
        if not _record_matches(receipt.get(key), path):
            raise H0PrimarySplitV5Error(f"candidate record drifted: {key}")
    gate = _strict_mapping(ADAPTATION_DIR / "offline_gate.json")
    report = _strict_mapping(ADAPTATION_DIR / "adaptation_report.json")
    manifest = _strict_mapping(
        CANDIDATE_DIR / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    )
    source_h0 = receipt.get("source_h0")
    expected_source_h0 = {
        "state": H0_MODULE / "module_state.pkl",
        "ctor": H0_MODULE / "class_and_ctor_args.pkl",
        "metadata": H0_MODULE / "metadata.json",
        "config": H0_CONFIG,
    }
    if not isinstance(source_h0, Mapping) or set(source_h0) != set(
        expected_source_h0
    ):
        raise H0PrimarySplitV5Error("V5 candidate source-H0 schema drifted")
    for key, path in expected_source_h0.items():
        if not _record_matches(source_h0[key], path):
            raise H0PrimarySplitV5Error(f"V5 candidate source H0 drifted: {key}")
    manifest_records = {
        "execution_lock": LOCK,
        "attempt_claim": ATTEMPT_CLAIM,
        "v3_corpus": REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus"],
        "v3_terminal_ledger": REPO_ROOT
        / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"],
        "v4_preexecution_failure": V4_PREEXECUTION_FAILURE,
    }
    for key, path in manifest_records.items():
        if not _record_matches(manifest.get(key), path):
            raise H0PrimarySplitV5Error(f"V5 candidate manifest drifted: {key}")
    hyperparameters = report.get("hyperparameters")
    if (
        set(manifest) != contract.ACTOR_MANIFEST_KEYS
        or gate.get("passed") is not True
        or gate.get("status") != receipt.get("status")
        or not _record_matches(
            gate.get("v4_preexecution_failure"), V4_PREEXECUTION_FAILURE
        )
        or _canonical_bytes(gate.get("fit")) != _canonical_bytes(FIT)
        or not all(gate.get("checks", {}).values())
        or report.get("selection_mode") != "fixed_final_epoch"
        or report.get("epochs_run") != 400
        or report.get("best_epoch") != 400
        or report.get("validation_samples") != 0
        or not isinstance(hyperparameters, Mapping)
        or hyperparameters.get("trainable_first_layer_features") is not None
        or hyperparameters.get("freeze_logstd_head") is not True
        or hyperparameters.get("learning_rate") != FIT["learning_rate"]
        or hyperparameters.get("batch_size") != FIT["batch_size"]
        or hyperparameters.get("anchor_weight") != FIT["anchor_weight"]
        or manifest.get("candidate_id") != CANDIDATE_ID
        or manifest.get("trainable_scope") != contract.TRAINABLE_SCOPE
        or manifest.get("logstd_policy") != contract.LOGSTD_POLICY
        or manifest.get("actor_digest") != gate.get("candidate_actor_digest")
        or manifest.get("module_state_sha256")
        != v3.sha256_file(CANDIDATE_DIR / "module_state.pkl")
        or _canonical_bytes(manifest.get("source_h0"))
        != _canonical_bytes(source_h0)
    ):
        raise H0PrimarySplitV5Error("V5 candidate gate/report/manifest drifted")
    return receipt, CANDIDATE_DIR


def _candidate_freeze_payload() -> dict[str, Any]:
    receipt, candidate = _validate_candidate_receipt()
    gate = _strict_mapping(ADAPTATION_DIR / "offline_gate.json")
    return {
        "schema_version": 5,
        "status": "H0_PRIMARY_SPLIT_V5_CANDIDATE_FROZEN_BEFORE_HOLDOUT",
        "protocol_id": PROTOCOL_ID,
        "candidate_id": CANDIDATE_ID,
        "train_seeds": list(TRAIN_SEEDS),
        "final_holdout_seed": FINAL_HOLDOUT_SEED,
        "fit": FIT,
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "logstd_policy": contract.LOGSTD_POLICY,
        "candidate_receipt": source_record(ADAPTATION_DIR / "receipt.json"),
        "candidate_offline_gate": source_record(ADAPTATION_DIR / "offline_gate.json"),
        "candidate_module_state": source_record(candidate / "module_state.pkl"),
        "candidate_module_ctor": source_record(candidate / "class_and_ctor_args.pkl"),
        "candidate_module_metadata": source_record(candidate / "metadata.json"),
        "actor_feature_manifest": receipt["actor_feature_manifest"],
        "candidate_actor_digest": gate["candidate_actor_digest"],
        "v3_terminal_ledger": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
        ),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "v3_corpus": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus"]
        ),
        "holdout_accessed_before_freeze": False,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def freeze_candidate_before_holdout() -> dict[str, Any]:
    forbidden = (
        HOLDOUT_REPLAY_DIR,
        HOLDOUT_ACCESS_CLAIM,
        HOLDOUT_REPLAY_RECEIPT,
        HOLDOUT_DIR,
    )
    if any(os.path.lexists(path) for path in forbidden):
        raise H0PrimarySplitV5Error("holdout artifact exists before candidate freeze")
    payload = _candidate_freeze_payload()
    _write_json_exclusive(CANDIDATE_FREEZE, payload)
    return payload


def verify_candidate_freeze() -> dict[str, Any]:
    observed = _strict_mapping(CANDIDATE_FREEZE)
    expected = _candidate_freeze_payload()
    if set(observed) != contract.CANDIDATE_FREEZE_KEYS or _canonical_bytes(
        observed
    ) != _canonical_bytes(expected):
        raise H0PrimarySplitV5Error("V5 candidate freeze drifted")
    return observed


def _holdout_access_payload() -> dict[str, Any]:
    verify_candidate_freeze()
    return {
        "schema_version": 5,
        "status": "H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT_ACCESS_CLAIMED",
        "protocol_id": PROTOCOL_ID,
        "candidate_id": CANDIDATE_ID,
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_seed": FINAL_HOLDOUT_SEED,
        "fit_updates_complete_before_access": True,
        "additional_actor_updates_authorized": False,
        "replay_engine": source_record(v3.RUNNER),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "so_policy_id": contract.SO_POLICY_ID,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def claim_holdout_access() -> dict[str, Any]:
    if os.path.lexists(HOLDOUT_REPLAY_DIR) or os.path.lexists(
        HOLDOUT_REPLAY_RECEIPT
    ):
        raise H0PrimarySplitV5Error("holdout replay exists before access claim")
    payload = _holdout_access_payload()
    _write_json_exclusive(HOLDOUT_ACCESS_CLAIM, payload)
    return payload


def verify_holdout_access_claim() -> dict[str, Any]:
    observed = _strict_mapping(HOLDOUT_ACCESS_CLAIM)
    expected = _holdout_access_payload()
    if set(observed) != contract.HOLDOUT_ACCESS_CLAIM_KEYS or _canonical_bytes(
        observed
    ) != _canonical_bytes(expected):
        raise H0PrimarySplitV5Error("V5 holdout access claim drifted")
    return observed


@contextlib.contextmanager
def _bind_frozen_v3_holdout_engine() -> Iterator[None]:
    """Bind V3's frozen replay engine to V5 provenance in this worker only."""
    # Validate with the unmodified V3 module first.  Calling the rich V5
    # validators after RUN_ROOT is rebound would make their inherited-corpus
    # audit look under the V5 root.  The engine therefore receives immutable
    # snapshots that were fully verified before any process-local binding.
    lock_snapshot = verify_lock()
    attempt_snapshot = _verify_attempt_claim()
    candidate_snapshot = verify_candidate_freeze()
    access_snapshot = verify_holdout_access_claim()

    def frozen_lock_snapshot() -> dict[str, Any]:
        return lock_snapshot

    def frozen_attempt_snapshot() -> dict[str, Any]:
        return attempt_snapshot

    def frozen_candidate_snapshot() -> dict[str, Any]:
        return candidate_snapshot

    def frozen_access_snapshot() -> dict[str, Any]:
        return access_snapshot

    replacements = {
        "LOCK": LOCK,
        "RUN_ROOT": RUN_ROOT,
        "ATTEMPT_CLAIM": ATTEMPT_CLAIM,
        "CANDIDATE_FREEZE": CANDIDATE_FREEZE,
        "HOLDOUT_ACCESS_CLAIM": HOLDOUT_ACCESS_CLAIM,
        "PROTOCOL_ID": PROTOCOL_ID,
        "CANDIDATE_ID": CANDIDATE_ID,
        "REVISION": REVISION,
        "verify_lock": frozen_lock_snapshot,
        "_verify_attempt_claim": frozen_attempt_snapshot,
        "verify_candidate_freeze": frozen_candidate_snapshot,
        "verify_holdout_access_claim": frozen_access_snapshot,
    }
    original = {name: getattr(v3, name) for name in replacements}
    try:
        for name, value in replacements.items():
            setattr(v3, name, value)
        yield
    finally:
        for name, value in original.items():
            setattr(v3, name, value)


def _validate_engine_replay_receipt() -> tuple[dict[str, Any], dict[str, Path]]:
    with _bind_frozen_v3_holdout_engine():
        receipt, paths = v3._validate_replay_receipt(FINAL_HOLDOUT_SEED)
    if (
        receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_REPLAY"
        or receipt.get("passed") is not True
        or receipt.get("seed") != FINAL_HOLDOUT_SEED
        or receipt.get("diagnostic") is not False
        or not _record_matches(receipt.get("execution_lock"), LOCK)
        or not _record_matches(receipt.get("attempt_claim"), ATTEMPT_CLAIM)
    ):
        raise H0PrimarySplitV5Error("frozen V3 holdout-engine receipt drifted")
    return receipt, paths


def holdout_replay_worker() -> dict[str, Any]:
    verify_lock()
    verify_candidate_freeze()
    verify_holdout_access_claim()
    if os.path.lexists(HOLDOUT_REPLAY_RECEIPT):
        raise H0PrimarySplitV5Error("V5 holdout wrapper receipt already exists")
    with _bind_frozen_v3_holdout_engine():
        v3.run_replay(
            seed=FINAL_HOLDOUT_SEED,
            output_dir=HOLDOUT_REPLAY_DIR,
            diagnostic=False,
            max_steps=EXPECTED_STEPS,
        )
    engine_receipt, engine_paths = _validate_engine_replay_receipt()
    engine_gate = _strict_mapping(engine_paths["gate"])
    passed = bool(
        engine_receipt.get("passed") is True
        and engine_gate.get("passed") is True
        and engine_gate.get("so_policy_id") == contract.SO_POLICY_ID
    )
    wrapper = {
        "schema_version": 5,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V5_REPLAY"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V5_REPLAY"
        ),
        "passed": passed,
        "seed": FINAL_HOLDOUT_SEED,
        "engine_schema": "H0_PRIMARY_SPLIT_V3_REPLAY",
        "engine_runner": source_record(
            REPO_ROOT
            / contract.INHERITED_SOURCE_RELATIVE_PATHS["v3_runner_replay_engine"]
        ),
        "v5_wrapper_runner": source_record(RUNNER),
        "engine_receipt": source_record(HOLDOUT_REPLAY_DIR / "receipt.json"),
        "engine_gate": source_record(engine_paths["gate"]),
        "engine_arrays": source_record(engine_paths["arrays"]),
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_access_claim": source_record(HOLDOUT_ACCESS_CLAIM),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "so_policy_id": contract.SO_POLICY_ID,
        "actor_updates": 1,
        "additional_actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(HOLDOUT_REPLAY_RECEIPT, wrapper)
    if not passed:
        raise H0PrimarySplitV5Error("V5 holdout replay wrapper failed")
    return wrapper


def _validate_holdout_replay_wrapper() -> tuple[dict[str, Any], dict[str, Path]]:
    wrapper = _strict_mapping(HOLDOUT_REPLAY_RECEIPT)
    _engine, paths = _validate_engine_replay_receipt()
    if (
        set(wrapper) != contract.HOLDOUT_REPLAY_RECEIPT_KEYS
        or wrapper.get("schema_version") != 5
        or wrapper.get("status") != "PASS_H0_PRIMARY_SPLIT_V5_REPLAY"
        or wrapper.get("passed") is not True
        or wrapper.get("seed") != FINAL_HOLDOUT_SEED
        or wrapper.get("engine_schema") != "H0_PRIMARY_SPLIT_V3_REPLAY"
        or wrapper.get("so_policy_id") != contract.SO_POLICY_ID
        or wrapper.get("actor_updates") != 1
        or wrapper.get("additional_actor_updates") != 0
        or wrapper.get("critic_updates") != 0
        or wrapper.get("ppo_updates") != 0
        or wrapper.get("protected_trials_opened") != []
        or not _record_matches(
            wrapper.get("engine_receipt"), HOLDOUT_REPLAY_DIR / "receipt.json"
        )
        or not _record_matches(wrapper.get("engine_gate"), paths["gate"])
        or not _record_matches(wrapper.get("engine_arrays"), paths["arrays"])
        or not _record_matches(wrapper.get("candidate_freeze"), CANDIDATE_FREEZE)
        or not _record_matches(
            wrapper.get("holdout_access_claim"), HOLDOUT_ACCESS_CLAIM
        )
        or not _record_matches(
            wrapper.get("v4_preexecution_failure"), V4_PREEXECUTION_FAILURE
        )
        or not _record_matches(
            wrapper.get("engine_runner"),
            REPO_ROOT
            / contract.INHERITED_SOURCE_RELATIVE_PATHS["v3_runner_replay_engine"],
        )
        or not _record_matches(wrapper.get("v5_wrapper_runner"), RUNNER)
    ):
        raise H0PrimarySplitV5Error("V5 holdout replay wrapper drifted")
    return wrapper, paths


def evaluate_final_holdout(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    freeze = verify_candidate_freeze()
    verify_holdout_access_claim()
    wrapper, replay_paths = _validate_holdout_replay_wrapper()
    destination = _claim_directory(output_dir, HOLDOUT_DIR)
    import numpy as np

    with np.load(replay_paths["arrays"], allow_pickle=False) as archive:
        students = np.ascontiguousarray(archive["target_observations"])
        teachers = np.ascontiguousarray(
            archive["reconstructed_teacher_observations"]
        )
        targets_one = np.ascontiguousarray(archive["teacher_means"])
        names = np.ascontiguousarray(archive["actor_feature_names"])
    if (
        students.dtype != np.float32
        or students.shape != (EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES)
        or teachers.dtype != np.float32
        or teachers.shape != students.shape
        or targets_one.dtype != np.float32
        or targets_one.shape != (EXPECTED_STEPS, 2)
        or names.dtype != np.dtype("U64")
        or names.shape != (EXPECTED_ACTOR_FEATURES,)
        or not np.all(np.isfinite(students))
        or not np.all(np.isfinite(teachers))
        or not np.all(np.isfinite(targets_one))
    ):
        raise H0PrimarySplitV5Error("holdout replay arrays drifted")
    observations = np.empty((EXPECTED_STEPS * 2, EXPECTED_ACTOR_FEATURES), np.float32)
    targets = np.empty((EXPECTED_STEPS * 2, 2), np.float32)
    roles = np.empty(EXPECTED_STEPS * 2, dtype="U16")
    observations[0::2] = students
    observations[1::2] = teachers
    targets[0::2] = targets_one
    targets[1::2] = targets_one
    roles[0::2] = split_contract.STUDENT_VIEW
    roles[1::2] = split_contract.TEACHER_VIEW
    indices = np.arange(len(observations), dtype=np.int64)
    _receipt, candidate = _validate_candidate_receipt()
    source_logits = _module_logits(H0_MODULE, observations)
    candidate_logits = _module_logits(candidate, observations)
    metrics = split_contract.offline_adaptation_gate(
        source_predictions=source_logits[:, :2],
        adapted_predictions=candidate_logits[:, :2],
        targets=targets,
        validation_indices=indices,
        view_roles=roles,
        all_adapted_logits=candidate_logits,
    )
    candidate_unchanged = source_record(candidate / "module_state.pkl") == freeze[
        "candidate_module_state"
    ]
    checks = {
        **metrics["checks"],
        "holdout_seed_125": FINAL_HOLDOUT_SEED == 125,
        "candidate_frozen_before_holdout": True,
        "candidate_module_unchanged": candidate_unchanged,
        "holdout_records_1000": len(observations) == 1000,
        "actor_layout_35": len(names) == EXPECTED_ACTOR_FEATURES,
        "status0_policy": wrapper.get("so_policy_id") == contract.SO_POLICY_ID,
        "frozen_v3_replay_engine": wrapper.get("engine_schema")
        == "H0_PRIMARY_SPLIT_V3_REPLAY",
        "no_updates_during_holdout": True,
    }
    passed = all(checks.values())
    gate = {
        "schema_version": 5,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT"
        ),
        "passed": passed,
        "checks": checks,
        "metrics": metrics,
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_access_claim": source_record(HOLDOUT_ACCESS_CLAIM),
        "holdout_replay_receipt": source_record(HOLDOUT_REPLAY_RECEIPT),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "actor_updates": 1,
        "additional_actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = _write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": 5,
        "status": gate["status"],
        "passed": passed,
        "gate": source_record(gate_path),
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_access_claim": source_record(HOLDOUT_ACCESS_CLAIM),
        "holdout_replay_receipt": source_record(HOLDOUT_REPLAY_RECEIPT),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "actor_updates": 1,
        "additional_actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    if set(receipt) != contract.HOLDOUT_RECEIPT_KEYS:
        raise H0PrimarySplitV5Error("V5 holdout receipt schema disagrees with contract")
    _write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise H0PrimarySplitV5Error("V5 final holdout gate failed")
    return receipt


def _worker_command(*arguments: str) -> list[str]:
    return [sys.executable, str(RUNNER), *arguments]


def execute() -> dict[str, Any]:
    verify_lock()
    _claim_attempt_root()
    started = time.time()
    stage = "adaptation"
    status = "FAIL_H0_PRIMARY_SPLIT_V5_OFFLINE"
    passed = False
    error: str | None = None
    try:
        completed = subprocess.run(
            _worker_command("--adapt-worker", "--output-dir", str(ADAPTATION_DIR)),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise H0PrimarySplitV5Error(
                f"V5 adaptation worker exited {completed.returncode}"
            )
        _validate_candidate_receipt()
        stage = "candidate_freeze"
        freeze_candidate_before_holdout()
        stage = "final_holdout_replay"
        status = "FAIL_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT"
        claim_holdout_access()
        completed = subprocess.run(
            _worker_command("--holdout-replay-worker"),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise H0PrimarySplitV5Error(
                f"V5 holdout replay worker exited {completed.returncode}"
            )
        stage = "final_holdout_evaluation"
        evaluate_final_holdout(HOLDOUT_DIR)
        stage = "final_holdout_complete"
        status = "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT"
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 5,
        "status": status,
        "passed": passed,
        "terminal_stage": stage,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "v3_terminal_ledger": source_record(
            REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
        ),
        "v4_preexecution_failure": source_record(V4_PREEXECUTION_FAILURE),
        "v3_corpus_reused": True,
        "v3_failed_candidate_reused": False,
        "v4_candidate_reused": False,
        "candidate_created": (ADAPTATION_DIR / "receipt.json").is_file(),
        "candidate_frozen_before_holdout": CANDIDATE_FREEZE.is_file(),
        "holdout_access_claimed": HOLDOUT_ACCESS_CLAIM.is_file(),
        "holdout_replay_completed": HOLDOUT_REPLAY_RECEIPT.is_file(),
        "final_holdout_completed": (HOLDOUT_DIR / "receipt.json").is_file(),
        "actor_update_candidates": int(
            (ADAPTATION_DIR / "adaptation_report.json").is_file()
        ),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "retry_or_retuning_allowed": False,
        "next_stage": (
            "CANONICAL_CLOSED_LOOP_QUALIFICATION"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    if set(ledger) != contract.EXECUTION_LEDGER_KEYS:
        raise H0PrimarySplitV5Error("V5 ledger schema disagrees with contract")
    _write_json_exclusive(EXECUTION_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise H0PrimarySplitV5Error(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--adapt-worker", action="store_true")
    mode.add_argument("--holdout-replay-worker", action="store_true")
    mode.add_argument("--evaluate-holdout", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--output-dir")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.execute:
        execute()
        return 0
    if args.adapt_worker:
        if not args.output_dir:
            raise H0PrimarySplitV5Error("--output-dir is required")
        adapt_worker(args.output_dir)
        return 0
    if args.holdout_replay_worker:
        if args.output_dir:
            raise H0PrimarySplitV5Error(
                "holdout replay has a fixed canonical output"
            )
        holdout_replay_worker()
        return 0
    if not args.output_dir:
        raise H0PrimarySplitV5Error("--output-dir is required")
    evaluate_final_holdout(args.output_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
