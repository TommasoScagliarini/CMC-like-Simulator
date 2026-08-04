"""Post-training, fail-closed screen for the preregistered 50-update PPO pilot.

The harness deliberately has no training, held-out, checkpoint-copy, or
checkpoint-promotion path.  It pins the preregistered protocol, validates all
50 PPO updates plus the separately generated Ray restart audit, then screens
only the eight preregistered development milestones.  At each milestone the
seed-123 stochastic +0.20 s rollout is run first; the three deterministic
rollouts are launched only when that critical case passes.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import yaml

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from validation import audit_training_restarts as restart_audit
from validation import robust_ppo_gate as gate
from validation.compare_policy_checkpoints import _actor_digest, _load_state


REPORT_SCHEMA_VERSION = 1
PROTOCOL_SCHEMA_VERSION = 1
CANONICAL_PROTOCOL_CONTENT_SHA256 = (
    "07940d370144b80707bfcadcb4c967157a58c326f8a74dfd5e9d2cbd40c5d92e"
)
EXPECTED_PILOT_UPDATE_INDICES = (1, 2, 5, 10, 20, 30, 40, 50)
EXPECTED_SCREENED_LOGICAL_ITERATIONS = (2, 3, 6, 11, 21, 31, 41, 51)
EXPECTED_RETAINED_LOGICAL_ITERATIONS = tuple(range(2, 52))
EXPECTED_DEVELOPMENT_SEED = 123
SEALED_HELD_OUT_SEEDS = (126, 127, 128)
MILESTONE_PREFIX = "milestone_iteration_"
REPORT_FILENAME = "ppo_pilot_screen.json"

_CONFIG_OVERRIDE_CONTRACT = (
    ("simulation", "iterations", 2, 51),
    ("supervision", "max_consecutive_skips", 5, 1),
    ("supervision", "max_consecutive_crash_restarts", 5, 1),
)
_CONFIG_ADDITION_CONTRACT = (
    ("supervision", "retain_iteration_checkpoints", True),
    ("supervision", "max_minibatch_mean_kl_loss", 0.01),
)


@dataclass(frozen=True)
class PilotScreenConfig:
    protocol: Path
    output_dir: Path
    restart_audit: Path
    ray_log_dir: Path
    expected_protocol_content_sha256: str = CANONICAL_PROTOCOL_CONTENT_SHA256
    python_executable: str = sys.executable
    rollout_script: Path = gate.DEFAULT_ROLLOUT_SCRIPT
    rollout_timeout_s: float = 900.0


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _path_from_protocol(value: Any) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT_DIR / path
    return path.resolve(strict=False)


def _exact_numbers(value: Any, expected: Sequence[float]) -> bool:
    return (
        isinstance(value, list)
        and len(value) == len(expected)
        and all(
            gate._numeric_equal(actual, float(target), tolerance=gate.OFFSET_TOLERANCE_S)
            for actual, target in zip(value, expected)
        )
    )


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_json_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_yaml_mapping(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        return None, str(exc)
    if not isinstance(value, Mapping):
        return None, f"expected a YAML mapping: {path}"
    return dict(value), None


def _typed_differences(
    expected: Any,
    actual: Any,
    *,
    path: tuple[str, ...] = (),
) -> list[dict[str, Any]]:
    location = "/" + "/".join(path)
    if type(expected) is not type(actual):
        return [
            {
                "path": location,
                "reason": "type_mismatch",
                "expected_type": type(expected).__name__,
                "actual_type": type(actual).__name__,
                "expected": expected,
                "actual": actual,
            }
        ]
    if isinstance(expected, dict):
        differences: list[dict[str, Any]] = []
        for key in sorted(set(expected) | set(actual), key=str):
            nested_path = (*path, str(key))
            nested_location = "/" + "/".join(nested_path)
            if key not in expected:
                differences.append(
                    {
                        "path": nested_location,
                        "reason": "unexpected_key",
                        "expected": "<absent>",
                        "actual": actual[key],
                    }
                )
            elif key not in actual:
                differences.append(
                    {
                        "path": nested_location,
                        "reason": "missing_key",
                        "expected": expected[key],
                        "actual": "<absent>",
                    }
                )
            else:
                differences.extend(
                    _typed_differences(
                        expected[key],
                        actual[key],
                        path=nested_path,
                    )
                )
        return differences
    if isinstance(expected, list):
        if len(expected) != len(actual):
            return [
                {
                    "path": location,
                    "reason": "length_mismatch",
                    "expected": expected,
                    "actual": actual,
                }
            ]
        differences = []
        for index, (expected_item, actual_item) in enumerate(
            zip(expected, actual)
        ):
            differences.extend(
                _typed_differences(
                    expected_item,
                    actual_item,
                    path=(*path, str(index)),
                )
            )
        return differences
    if expected != actual:
        return [
            {
                "path": location,
                "reason": "value_mismatch",
                "expected": expected,
                "actual": actual,
            }
        ]
    return []


def _resolved_config_contract(
    *,
    source_path: Path,
    run_path: Path,
    expected_source_sha256: str,
) -> dict[str, Any]:
    """Require the run YAML to equal its pinned source plus five CLI overrides."""

    try:
        source_sha256 = gate._sha256(source_path)
    except OSError:
        source_sha256 = None
    try:
        run_sha256 = gate._sha256(run_path)
    except OSError:
        run_sha256 = None
    source, source_error = _read_yaml_mapping(source_path)
    run, run_error = _read_yaml_mapping(run_path)
    source_map = _mapping(source)
    run_map = _mapping(run)
    source_simulation = _mapping(source_map.get("simulation"))
    source_supervision = _mapping(source_map.get("supervision"))
    run_simulation = _mapping(run_map.get("simulation"))
    run_supervision = _mapping(run_map.get("supervision"))

    checks = [
        gate._check(
            "resolved_config_source_sha256",
            source_sha256 == expected_source_sha256,
            source_sha256,
            expected_source_sha256,
        ),
        gate._check("resolved_config_source_yaml", source_error is None, source_error, None),
        gate._check("run_resolved_config_yaml", run_error is None, run_error, None),
    ]

    expected_run: dict[str, Any] = copy.deepcopy(dict(source_map))
    for section, key, source_value, run_value in _CONFIG_OVERRIDE_CONTRACT:
        source_section = (
            source_simulation if section == "simulation" else source_supervision
        )
        run_section = run_simulation if section == "simulation" else run_supervision
        checks.append(
            gate._check(
                f"resolved_config_source.{section}.{key}",
                type(source_section.get(key)) is type(source_value)
                and source_section.get(key) == source_value,
                source_section.get(key),
                source_value,
            )
        )
        checks.append(
            gate._check(
                f"run_resolved_config.{section}.{key}",
                type(run_section.get(key)) is type(run_value)
                and run_section.get(key) == run_value,
                run_section.get(key),
                run_value,
            )
        )
        expected_section = expected_run.get(section)
        if not isinstance(expected_section, dict):
            expected_section = {}
            expected_run[section] = expected_section
        expected_section[key] = run_value

    for section, key, run_value in _CONFIG_ADDITION_CONTRACT:
        source_section = source_supervision
        run_section = run_supervision
        checks.append(
            gate._check(
                f"resolved_config_source.{section}.{key}_absent",
                key not in source_section,
                source_section.get(key, "<absent>"),
                "<absent>",
            )
        )
        checks.append(
            gate._check(
                f"run_resolved_config.{section}.{key}",
                type(run_section.get(key)) is type(run_value)
                and run_section.get(key) == run_value,
                run_section.get(key),
                run_value,
            )
        )
        expected_section = expected_run.get(section)
        if not isinstance(expected_section, dict):
            expected_section = {}
            expected_run[section] = expected_section
        expected_section[key] = run_value

    differences = (
        _typed_differences(expected_run, dict(run_map))
        if source_error is None and run_error is None
        else [{"reason": "yaml_read_failed"}]
    )
    checks.append(
        gate._check(
            "run_resolved_config_exact_preregistered_overrides",
            differences == [],
            differences,
            [],
        )
    )
    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    return {
        "status": "PASS" if not failed else "FAIL",
        "source_path": gate._portable_path(source_path),
        "source_sha256": source_sha256,
        "run_path": gate._portable_path(run_path),
        "run_sha256": run_sha256,
        "allowed_changes": [
            {
                "path": f"/{section}/{key}",
                "source": source_value,
                "run": run_value,
            }
            for section, key, source_value, run_value in _CONFIG_OVERRIDE_CONTRACT
        ]
        + [
            {
                "path": f"/{section}/{key}",
                "source": "<absent>",
                "run": run_value,
            }
            for section, key, run_value in _CONFIG_ADDITION_CONTRACT
        ],
        "unexpected_differences": differences,
        "checks": checks,
        "failed_checks": failed,
    }


def _protocol_contract(protocol: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any] | None]:
    source = _mapping(protocol.get("source"))
    run = _mapping(protocol.get("run"))
    optimization = _mapping(protocol.get("optimization"))
    sampling = _mapping(protocol.get("sampling"))
    online = _mapping(protocol.get("online_stop_contract"))
    milestones = _mapping(protocol.get("milestones"))
    development = _mapping(protocol.get("development_gate"))
    selection = _mapping(protocol.get("candidate_selection"))
    held_out = _mapping(protocol.get("held_out_gate"))
    interpretation = _mapping(protocol.get("interpretation"))

    checks = [
        gate._check("schema_version", protocol.get("schema_version") == PROTOCOL_SCHEMA_VERSION, protocol.get("schema_version"), PROTOCOL_SCHEMA_VERSION),
        gate._check("status", protocol.get("status") == "preregistered", protocol.get("status"), "preregistered"),
        gate._check("purpose", protocol.get("purpose") == "diagnostic_50_update_ppo_pilot_without_automatic_promotion", protocol.get("purpose"), "diagnostic_50_update_ppo_pilot_without_automatic_promotion"),
        gate._check("source.logical_iteration", source.get("logical_iteration") == 1, source.get("logical_iteration"), 1),
        gate._check("source.canonical_name", source.get("canonical_name") == "H0", source.get("canonical_name"), "H0"),
        gate._check("source.immutable", source.get("immutable") is True, source.get("immutable"), True),
        gate._check("source.checkpoint", _path_from_protocol(source.get("checkpoint")) is not None, source.get("checkpoint"), "non-empty path"),
        gate._check("source.checkpoint_meta", _path_from_protocol(source.get("checkpoint_meta")) is not None, source.get("checkpoint_meta"), "non-empty path"),
        gate._check("source.checkpoint_meta_sha256", _is_sha256(source.get("checkpoint_meta_sha256")), source.get("checkpoint_meta_sha256"), "lowercase SHA-256"),
        gate._check("source.actor_digest", _is_sha256(source.get("actor_digest")), source.get("actor_digest"), "lowercase SHA-256"),
        gate._check("run.output_dir", _path_from_protocol(run.get("output_dir")) is not None, run.get("output_dir"), "non-empty path"),
        gate._check("run.resolved_config_source", _path_from_protocol(run.get("resolved_config_source")) is not None, run.get("resolved_config_source"), "non-empty path"),
        gate._check("run.resolved_config_source_sha256", _is_sha256(run.get("resolved_config_source_sha256")), run.get("resolved_config_source_sha256"), "lowercase SHA-256"),
        gate._check("run.new_actor_updates", run.get("new_actor_updates") == 50, run.get("new_actor_updates"), 50),
        gate._check("run.first_logical_iteration", run.get("first_logical_iteration") == 2, run.get("first_logical_iteration"), 2),
        gate._check("run.last_logical_iteration", run.get("last_logical_iteration") == 51, run.get("last_logical_iteration"), 51),
        gate._check("run.real_transitions_per_update", run.get("real_transitions_per_update") == 4608, run.get("real_transitions_per_update"), 4608),
        gate._check("run.total_planned_real_transitions", run.get("total_planned_real_transitions") == 230400, run.get("total_planned_real_transitions"), 230400),
        gate._check("run.checkpoint_every", run.get("checkpoint_every") == 1, run.get("checkpoint_every"), 1),
        gate._check("run.retain_iteration_checkpoints", run.get("retain_iteration_checkpoints") is True, run.get("retain_iteration_checkpoints"), True),
        gate._check("run.update_history", run.get("update_history") is False, run.get("update_history"), False),
        gate._check("run.automatic_promotion", run.get("automatic_promotion") is False, run.get("automatic_promotion"), False),
        gate._check("optimization.learning_rate", gate._numeric_equal(optimization.get("learning_rate"), 5.0e-7), optimization.get("learning_rate"), 5.0e-7),
        gate._check("optimization.train_batch_size", optimization.get("train_batch_size") == 4608, optimization.get("train_batch_size"), 4608),
        gate._check("optimization.minibatch_size", optimization.get("minibatch_size") == 512, optimization.get("minibatch_size"), 512),
        gate._check("optimization.num_epochs", optimization.get("num_epochs") == 1, optimization.get("num_epochs"), 1),
        gate._check("optimization.freeze_logstd", optimization.get("freeze_logstd") is True, optimization.get("freeze_logstd"), True),
        gate._check("optimization.freeze_actor", optimization.get("freeze_actor") is False, optimization.get("freeze_actor"), False),
        gate._check("optimization.num_env_runners", optimization.get("num_env_runners") == 12, optimization.get("num_env_runners"), 12),
        gate._check("optimization.ray_num_cpus", optimization.get("ray_num_cpus") == 13, optimization.get("ray_num_cpus"), 13),
        gate._check("optimization.clip_param", gate._numeric_equal(optimization.get("clip_param"), 0.05), optimization.get("clip_param"), 0.05),
        gate._check("optimization.kl_coeff", gate._numeric_equal(optimization.get("kl_coeff"), 1.0), optimization.get("kl_coeff"), 1.0),
        gate._check("optimization.kl_target", gate._numeric_equal(optimization.get("kl_target"), 0.01), optimization.get("kl_target"), 0.01),
        gate._check("optimization.hard_max_minibatch_mean_kl_loss", gate._numeric_equal(optimization.get("hard_max_minibatch_mean_kl_loss"), gate.KL_LIMIT), optimization.get("hard_max_minibatch_mean_kl_loss"), gate.KL_LIMIT),
        gate._check("optimization.hard_min_minibatch_mean_kl_loss", gate._numeric_equal(optimization.get("hard_min_minibatch_mean_kl_loss"), gate.KL_MIN_NUMERICAL_TOLERANCE), optimization.get("hard_min_minibatch_mean_kl_loss"), gate.KL_MIN_NUMERICAL_TOLERANCE),
        gate._check("optimization.required_kl_minibatch_count", optimization.get("required_kl_minibatch_count") == gate.EXPECTED_KL_MINIBATCH_COUNT, optimization.get("required_kl_minibatch_count"), gate.EXPECTED_KL_MINIBATCH_COUNT),
        gate._check("optimization.required_kl_nonfinite_count", optimization.get("required_kl_nonfinite_count") == 0, optimization.get("required_kl_nonfinite_count"), 0),
        gate._check("sampling.mode", sampling.get("mode") == "exact_post_gae_interleaved", sampling.get("mode"), "exact_post_gae_interleaved"),
        gate._check("sampling.start_offsets_s", _exact_numbers(sampling.get("start_offsets_s"), gate.DEFAULT_START_OFFSETS_S), sampling.get("start_offsets_s"), list(gate.DEFAULT_START_OFFSETS_S)),
        gate._check("sampling.runners_per_start", sampling.get("runners_per_start") == 4, sampling.get("runners_per_start"), 4),
        gate._check("sampling.rollout_fragment_length", sampling.get("rollout_fragment_length") == 384, sampling.get("rollout_fragment_length"), 384),
        gate._check("sampling.real_transitions_per_start_per_update", sampling.get("real_transitions_per_start_per_update") == gate.EXPECTED_START_STEPS, sampling.get("real_transitions_per_start_per_update"), gate.EXPECTED_START_STEPS),
        gate._check("sampling.maximum_start_run_length", sampling.get("maximum_start_run_length") == 1, sampling.get("maximum_start_run_length"), 1),
        gate._check("online_stop_contract.stop_on_exact_start_balance_failure", online.get("stop_on_exact_start_balance_failure") is True, online.get("stop_on_exact_start_balance_failure"), True),
        gate._check("online_stop_contract.stop_on_post_gae_compaction_failure", online.get("stop_on_post_gae_compaction_failure") is True, online.get("stop_on_post_gae_compaction_failure"), True),
        gate._check("online_stop_contract.stop_on_interleaving_failure", online.get("stop_on_interleaving_failure") is True, online.get("stop_on_interleaving_failure"), True),
        gate._check("online_stop_contract.stop_on_missing_or_nonfinite_kl_audit", online.get("stop_on_missing_or_nonfinite_kl_audit") is True, online.get("stop_on_missing_or_nonfinite_kl_audit"), True),
        gate._check("online_stop_contract.stop_on_kl_limit_failure", online.get("stop_on_kl_limit_failure") is True, online.get("stop_on_kl_limit_failure"), True),
        gate._check("online_stop_contract.max_consecutive_skips", online.get("max_consecutive_skips") == 1, online.get("max_consecutive_skips"), 1),
        gate._check("online_stop_contract.any_timeout_skip_or_restart_invalidates_promotion_eligibility", online.get("any_timeout_skip_or_restart_invalidates_promotion_eligibility") is True, online.get("any_timeout_skip_or_restart_invalidates_promotion_eligibility"), True),
        gate._check("online_stop_contract.physical_failure_of_an_intermediate_checkpoint_does_not_stop_training", online.get("physical_failure_of_an_intermediate_checkpoint_does_not_stop_training") is True, online.get("physical_failure_of_an_intermediate_checkpoint_does_not_stop_training"), True),
        gate._check("milestones.retained_logical_iterations", milestones.get("retained_logical_iterations") == "all integers from 2 through 51 inclusive", milestones.get("retained_logical_iterations"), "all integers from 2 through 51 inclusive"),
        gate._check("milestones.screened_pilot_update_indices", milestones.get("screened_pilot_update_indices") == list(EXPECTED_PILOT_UPDATE_INDICES), milestones.get("screened_pilot_update_indices"), list(EXPECTED_PILOT_UPDATE_INDICES)),
        gate._check("milestones.screened_logical_iterations", milestones.get("screened_logical_iterations") == list(EXPECTED_SCREENED_LOGICAL_ITERATIONS), milestones.get("screened_logical_iterations"), list(EXPECTED_SCREENED_LOGICAL_ITERATIONS)),
        gate._check("milestones.screening_timing", milestones.get("screening_timing") == "after_training_to_avoid_cpu_contention", milestones.get("screening_timing"), "after_training_to_avoid_cpu_contention"),
        gate._check("milestones.screening_order_per_milestone", milestones.get("screening_order_per_milestone") == ["training_integrity_and_kl_audit", "stochastic_plus020_seed123_fail_fast", "three_deterministic_condition_matched_rollouts_only_if_critical_passes"], milestones.get("screening_order_per_milestone"), ["training_integrity_and_kl_audit", "stochastic_plus020_seed123_fail_fast", "three_deterministic_condition_matched_rollouts_only_if_critical_passes"]),
        gate._check("development_gate.stochastic_seed", development.get("stochastic_seed") == EXPECTED_DEVELOPMENT_SEED, development.get("stochastic_seed"), EXPECTED_DEVELOPMENT_SEED),
        gate._check("development_gate.seed_status", development.get("seed_status") == "development_and_contaminated_not_held_out", development.get("seed_status"), "development_and_contaminated_not_held_out"),
        gate._check("development_gate.expected_sigma", gate._numeric_equal(development.get("expected_sigma"), 0.005), development.get("expected_sigma"), 0.005),
        gate._check("development_gate.reference_checkpoint", _path_from_protocol(development.get("reference_checkpoint")) is not None, development.get("reference_checkpoint"), "non-empty path"),
        gate._check("development_gate.reference_summaries_root", _path_from_protocol(development.get("reference_summaries_root")) is not None, development.get("reference_summaries_root"), "non-empty path"),
        gate._check("development_gate.reserve_mode", development.get("reserve_mode") == gate.RESERVE_MODE_CONDITION_MATCHED, development.get("reserve_mode"), gate.RESERVE_MODE_CONDITION_MATCHED),
        gate._check("development_gate.penetration_limit_m_strict", gate._numeric_equal(development.get("penetration_limit_m_strict"), gate.PENETRATION_LIMIT_M), development.get("penetration_limit_m_strict"), gate.PENETRATION_LIMIT_M),
        gate._check("development_gate.minimum_valid_cycles", development.get("minimum_valid_cycles") == gate.EXPECTED_MIN_CYCLES, development.get("minimum_valid_cycles"), gate.EXPECTED_MIN_CYCLES),
        gate._check("development_gate.required_steps", development.get("required_steps") == gate.EXPECTED_STEPS, development.get("required_steps"), gate.EXPECTED_STEPS),
        gate._check("development_gate.maximum_action_clipped_steps", development.get("maximum_action_clipped_steps") == 0, development.get("maximum_action_clipped_steps"), 0),
        gate._check("candidate_selection.maximum_candidates_opened_on_held_out", selection.get("maximum_candidates_opened_on_held_out") == 1, selection.get("maximum_candidates_opened_on_held_out"), 1),
        gate._check("candidate_selection.eligible_set", selection.get("eligible_set") == "only preregistered screened milestones that pass every development gate", selection.get("eligible_set"), "only preregistered screened milestones that pass every development gate"),
        gate._check("candidate_selection.primary_order", selection.get("primary_order") == "minimum worst condition-matched reserve ratio versus H0", selection.get("primary_order"), "minimum worst condition-matched reserve ratio versus H0"),
        gate._check("candidate_selection.tie_breakers", selection.get("tie_breakers") == ["minimum worst penetration ratio versus H0", "minimum cumulative empirical KL versus H0", "earlier pilot update"], selection.get("tie_breakers"), ["minimum worst penetration ratio versus H0", "minimum cumulative empirical KL versus H0", "earlier pilot update"]),
        gate._check("candidate_selection.fallback_after_held_out_failure", selection.get("fallback_after_held_out_failure") is False, selection.get("fallback_after_held_out_failure"), False),
        gate._check("candidate_selection.checkpoint_best_is_not_a_selector", selection.get("checkpoint_best_is_not_a_selector") is True, selection.get("checkpoint_best_is_not_a_selector"), True),
        gate._check("held_out_gate.status", held_out.get("status") == "sealed_until_one_candidate_digest_is_fixed", held_out.get("status"), "sealed_until_one_candidate_digest_is_fixed"),
        gate._check("held_out_gate.seeds", held_out.get("seeds") == list(SEALED_HELD_OUT_SEEDS), held_out.get("seeds"), list(SEALED_HELD_OUT_SEEDS)),
        gate._check("held_out_gate.seeds_124_125_status", held_out.get("seeds_124_125_status") == "contaminated_development_only", held_out.get("seeds_124_125_status"), "contaminated_development_only"),
        gate._check("held_out_gate.start_offsets_s", _exact_numbers(held_out.get("start_offsets_s"), gate.DEFAULT_START_OFFSETS_S), held_out.get("start_offsets_s"), list(gate.DEFAULT_START_OFFSETS_S)),
        gate._check("held_out_gate.sigma", gate._numeric_equal(held_out.get("sigma"), 0.005), held_out.get("sigma"), 0.005),
        gate._check("held_out_gate.paired_reference", held_out.get("paired_reference") == "H0 in every seed/start cell", held_out.get("paired_reference"), "H0 in every seed/start cell"),
        gate._check("held_out_gate.reserve_mode", held_out.get("reserve_mode") == gate.RESERVE_MODE_CONDITION_MATCHED, held_out.get("reserve_mode"), gate.RESERVE_MODE_CONDITION_MATCHED),
        gate._check("held_out_gate.open_once", held_out.get("open_once") is True, held_out.get("open_once"), True),
        gate._check("interpretation.reward_reserve_residual_weight", gate._numeric_equal(interpretation.get("reward_reserve_residual_weight"), 0.0), interpretation.get("reward_reserve_residual_weight"), 0.0),
        gate._check("interpretation.training_return_alone_is_not_evidence_of_reserve_robustness", interpretation.get("training_return_alone_is_not_evidence_of_reserve_robustness") is True, interpretation.get("training_return_alone_is_not_evidence_of_reserve_robustness"), True),
        gate._check("interpretation.successful_completion_does_not_promote_a_checkpoint", interpretation.get("successful_completion_does_not_promote_a_checkpoint") is True, interpretation.get("successful_completion_does_not_promote_a_checkpoint"), True),
        gate._check("interpretation.long_training_tests_optimization_not_long_horizon_closed_loop_stability", interpretation.get("long_training_tests_optimization_not_long_horizon_closed_loop_stability") is True, interpretation.get("long_training_tests_optimization_not_long_horizon_closed_loop_stability"), True),
    ]
    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    report = {
        "status": "PASS" if not failed else "FAIL",
        "checks": checks,
        "failed_checks": failed,
    }
    if failed:
        return report, None

    context = {
        "source_checkpoint": _path_from_protocol(source["checkpoint"]),
        "source_checkpoint_meta": _path_from_protocol(source["checkpoint_meta"]),
        "source_checkpoint_meta_sha256": source["checkpoint_meta_sha256"],
        "source_actor_digest": source["actor_digest"],
        "run_dir": _path_from_protocol(run["output_dir"]),
        "resolved_config_source": _path_from_protocol(run["resolved_config_source"]),
        "resolved_config_source_sha256": run["resolved_config_source_sha256"],
        "reference_checkpoint": _path_from_protocol(development["reference_checkpoint"]),
        "reference_summaries_root": _path_from_protocol(development["reference_summaries_root"]),
        "start_offsets_s": tuple(float(value) for value in sampling["start_offsets_s"]),
        "expected_sigma": float(development["expected_sigma"]),
    }
    assert all(value is not None for key, value in context.items() if key.endswith("checkpoint") or key.endswith("meta") or key.endswith("dir") or key.endswith("source") or key.endswith("root"))
    return report, context


def _directory_nonempty(path: Path) -> bool:
    try:
        return path.is_dir() and next(path.iterdir(), None) is not None
    except OSError:
        return False


def _safe_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        return gate._read_json_object(path), None
    except (OSError, ValueError) as exc:
        return None, str(exc)


def _milestone_artifact(run_dir: Path, logical_iteration: int) -> dict[str, Any]:
    root = run_dir / f"{MILESTONE_PREFIX}{logical_iteration:06d}"
    checkpoint = root / "checkpoint_last"
    module = root / "rl_module_last"
    checkpoint_meta_path = root / "checkpoint_last_meta.json"
    module_meta_path = root / "rl_module_last_meta.json"
    checkpoint_meta, checkpoint_meta_error = _safe_json(checkpoint_meta_path)
    module_meta, module_meta_error = _safe_json(module_meta_path)
    failures: list[str] = []
    if not _directory_nonempty(checkpoint):
        failures.append("checkpoint_last_missing_or_empty")
    module_report = gate._checkpoint_artifact_report(module)
    if module_report["status"] != "PASS":
        failures.append("rl_module_last_invalid")
    if checkpoint_meta_error is not None:
        failures.append("checkpoint_last_meta_read")
    if module_meta_error is not None:
        failures.append("rl_module_last_meta_read")
    if checkpoint_meta is not None and (
        checkpoint_meta.get("logical_iteration") != logical_iteration
        or checkpoint_meta.get("rllib_training_iteration") != logical_iteration
        or not gate._same_path(checkpoint_meta.get("checkpoint"), checkpoint)
    ):
        failures.append("checkpoint_last_meta_contract")
    if module_meta is not None and (
        module_meta.get("logical_iteration") != logical_iteration
        or module_meta.get("rllib_training_iteration") != logical_iteration
        or not gate._same_path(module_meta.get("rl_module"), module)
    ):
        failures.append("rl_module_last_meta_contract")
    return {
        "logical_iteration": logical_iteration,
        "status": "PASS" if not failures else "FAIL",
        "root": gate._portable_path(root),
        "checkpoint": gate._portable_path(checkpoint),
        "rl_module": module_report,
        "checkpoint_meta_error": checkpoint_meta_error,
        "rl_module_meta_error": module_meta_error,
        "failures": failures,
    }


def _validate_completed_run(context: Mapping[str, Any]) -> dict[str, Any]:
    run_dir = Path(context["run_dir"])
    summary_path = run_dir / "summary.json"
    history_path = run_dir / "train_iterations.jsonl"
    supervisor_path = run_dir / "supervisor_state.json"
    summary, summary_error = _safe_json(summary_path)
    supervisor, supervisor_error = _safe_json(supervisor_path)
    try:
        history = gate._jsonl_objects(history_path)
        history_error = None
    except (OSError, ValueError) as exc:
        history = []
        history_error = str(exc)

    expected_iterations = list(EXPECTED_RETAINED_LOGICAL_ITERATIONS)
    history_iterations = [row.get("iteration") for row in history]
    summary_map = _mapping(summary)
    retention = _mapping(summary_map.get("iteration_checkpoint_retention"))
    sampling_contract = _mapping(summary_map.get("exact_start_sampling_contract"))
    kl_guard = _mapping(summary_map.get("kl_update_guard"))
    reward_config = _mapping(summary_map.get("reward_config"))
    summary_history = summary_map.get("history")
    summary_history_iterations = (
        [row.get("iteration") for row in summary_history if isinstance(row, Mapping)]
        if isinstance(summary_history, list)
        else None
    )
    reported_milestones = retention.get("milestones")
    expected_milestone_paths = [
        run_dir / f"{MILESTONE_PREFIX}{iteration:06d}"
        for iteration in EXPECTED_RETAINED_LOGICAL_ITERATIONS
    ]
    reported_milestones_ok = (
        isinstance(reported_milestones, list)
        and len(reported_milestones) == len(expected_milestone_paths)
        and all(
            gate._same_path(actual, expected)
            for actual, expected in zip(reported_milestones, expected_milestone_paths)
        )
    )
    present_names = sorted(
        path.name
        for path in run_dir.glob(f"{MILESTONE_PREFIX}*")
        if path.is_dir()
    ) if run_dir.is_dir() else []
    expected_names = [path.name for path in expected_milestone_paths]
    staging = sorted(
        path.name for path in run_dir.glob(f".{MILESTONE_PREFIX}*.tmp-*")
    ) if run_dir.is_dir() else []

    source_checkpoint = Path(context["source_checkpoint"])
    source_actor_module = (
        source_checkpoint
        / "learner_group"
        / "learner"
        / "rl_module"
        / "default_policy"
    )
    source_meta_path = Path(context["source_checkpoint_meta"])
    config_source = Path(context["resolved_config_source"])
    run_config = run_dir / "training_cfg.resolved.yaml"
    source_meta, source_meta_error = _safe_json(source_meta_path)

    def digest_or_none(path: Path) -> str | None:
        try:
            return gate._sha256(path)
        except OSError:
            return None

    source_meta_digest = digest_or_none(source_meta_path)
    source_meta_map = _mapping(source_meta)
    resolved_config = _resolved_config_contract(
        source_path=config_source,
        run_path=run_config,
        expected_source_sha256=context["resolved_config_source_sha256"],
    )
    try:
        source_checkpoint_actor_digest = _actor_digest(
            _load_state(source_actor_module)
        )
        source_checkpoint_actor_error = None
    except (OSError, ValueError, TypeError) as exc:
        source_checkpoint_actor_digest = None
        source_checkpoint_actor_error = str(exc)
    all_iteration_audits = [
        gate.classify_training_iterations(
            history_path,
            start_offsets_s=tuple(context["start_offsets_s"]),
            expected_training_iteration=iteration,
        )
        for iteration in EXPECTED_RETAINED_LOGICAL_ITERATIONS
    ]
    failed_iteration_audits = [
        audit["expected_iteration"]
        for audit in all_iteration_audits
        if audit.get("status") != "PASS"
    ]
    checks = [
        gate._check("run_directory", run_dir.is_dir(), gate._portable_path(run_dir), "existing directory"),
        gate._check("summary_read", summary_error is None, summary_error, None),
        gate._check("summary.ok", summary_map.get("ok") is True, summary_map.get("ok"), True),
        gate._check("summary.stop_reason", summary_map.get("stop_reason") == "completed", summary_map.get("stop_reason"), "completed"),
        gate._check("summary.interrupted", summary_map.get("interrupted") is False, summary_map.get("interrupted"), False),
        gate._check("summary.timed_out", summary_map.get("timed_out") is False, summary_map.get("timed_out"), False),
        gate._check("summary.error", summary_map.get("error") is None, summary_map.get("error"), None),
        gate._check("summary.output_dir", gate._same_path(summary_map.get("output_dir"), run_dir), summary_map.get("output_dir"), gate._portable_path(run_dir)),
        gate._check("summary.iterations_run", summary_map.get("iterations_run") == 50, summary_map.get("iterations_run"), 50),
        gate._check("summary.iterations_completed", summary_map.get("iterations_completed") == 51, summary_map.get("iterations_completed"), 51),
        gate._check("summary.iterations_completed_this_process", summary_map.get("iterations_completed_this_process") == 50, summary_map.get("iterations_completed_this_process"), 50),
        gate._check("summary.iteration_start", summary_map.get("iteration_start") == 2, summary_map.get("iteration_start"), 2),
        gate._check("summary.next_iteration", summary_map.get("next_iteration") == 52, summary_map.get("next_iteration"), 52),
        gate._check("summary.restored_logical_iteration", summary_map.get("restored_logical_iteration") == 1, summary_map.get("restored_logical_iteration"), 1),
        gate._check("summary.restored_training_iteration", summary_map.get("restored_training_iteration") == 1, summary_map.get("restored_training_iteration"), 1),
        gate._check("summary.resume_from", gate._same_path(summary_map.get("resume_from"), source_checkpoint), summary_map.get("resume_from"), gate._portable_path(source_checkpoint)),
        gate._check("summary.supervisor_resume_from", gate._same_path(summary_map.get("supervisor_resume_from"), source_checkpoint), summary_map.get("supervisor_resume_from"), gate._portable_path(source_checkpoint)),
        gate._check("summary.warm_start_requested", summary_map.get("warm_start_requested") is False, summary_map.get("warm_start_requested"), False),
        gate._check("summary.warm_start_applied", summary_map.get("warm_start_applied") is False, summary_map.get("warm_start_applied"), False),
        gate._check("summary.skipped_iterations", summary_map.get("skipped_iterations") == [], summary_map.get("skipped_iterations"), []),
        gate._check("summary.restart_count", summary_map.get("restart_count") == 0, summary_map.get("restart_count"), 0),
        gate._check("summary.crash_restart_count", summary_map.get("crash_restart_count") == 0, summary_map.get("crash_restart_count"), 0),
        gate._check("summary.crash_restarts", summary_map.get("crash_restarts") == [], summary_map.get("crash_restarts"), []),
        gate._check("summary.max_consecutive_skips", summary_map.get("max_consecutive_skips") == 1, summary_map.get("max_consecutive_skips"), 1),
        gate._check("summary.freeze_logstd", summary_map.get("freeze_logstd") is True, summary_map.get("freeze_logstd"), True),
        gate._check("summary.freeze_actor", summary_map.get("freeze_actor") is False, summary_map.get("freeze_actor"), False),
        gate._check("summary.lr", gate._numeric_equal(summary_map.get("lr"), 5.0e-7), summary_map.get("lr"), 5.0e-7),
        gate._check("summary.num_epochs", summary_map.get("num_epochs") == 1, summary_map.get("num_epochs"), 1),
        gate._check("summary.num_env_runners", summary_map.get("num_env_runners") == 12, summary_map.get("num_env_runners"), 12),
        gate._check("summary.ray_num_cpus", summary_map.get("ray_num_cpus") == 13, summary_map.get("ray_num_cpus"), 13),
        gate._check("summary.exact_start_sampling", summary_map.get("exact_start_sampling") is True, summary_map.get("exact_start_sampling"), True),
        gate._check("summary.episode_start_offset_choices_s", _exact_numbers(summary_map.get("episode_start_offset_choices_s"), context["start_offsets_s"]), summary_map.get("episode_start_offset_choices_s"), list(context["start_offsets_s"])),
        gate._check("summary.exact_start_sampling_contract.offsets_s", _exact_numbers(sampling_contract.get("offsets_s"), context["start_offsets_s"]), sampling_contract.get("offsets_s"), list(context["start_offsets_s"])),
        gate._check("summary.exact_start_sampling_contract.rollout_fragment_length", sampling_contract.get("rollout_fragment_length") == 384, sampling_contract.get("rollout_fragment_length"), 384),
        gate._check("summary.exact_start_sampling_contract.expected_steps_per_start", sampling_contract.get("expected_steps_per_start") == gate.EXPECTED_START_STEPS, sampling_contract.get("expected_steps_per_start"), gate.EXPECTED_START_STEPS),
        gate._check("summary.exact_start_sampling_contract.runners_per_start", sampling_contract.get("runners_per_start") == 4, sampling_contract.get("runners_per_start"), 4),
        gate._check("summary.clip_param", gate._numeric_equal(summary_map.get("clip_param"), 0.05), summary_map.get("clip_param"), 0.05),
        gate._check("summary.kl_coeff", gate._numeric_equal(summary_map.get("kl_coeff"), 1.0), summary_map.get("kl_coeff"), 1.0),
        gate._check("summary.kl_target", gate._numeric_equal(summary_map.get("kl_target"), 0.01), summary_map.get("kl_target"), 0.01),
        gate._check("summary.kl_update_guard.enabled", kl_guard.get("enabled") is True, kl_guard.get("enabled"), True),
        gate._check("summary.kl_update_guard.max_minibatch_mean_kl_loss_limit", gate._numeric_equal(kl_guard.get("max_minibatch_mean_kl_loss_limit"), gate.KL_LIMIT), kl_guard.get("max_minibatch_mean_kl_loss_limit"), gate.KL_LIMIT),
        gate._check("summary.kl_update_guard.min_minibatch_mean_kl_loss_floor", gate._numeric_equal(kl_guard.get("min_minibatch_mean_kl_loss_floor"), gate.KL_MIN_NUMERICAL_TOLERANCE), kl_guard.get("min_minibatch_mean_kl_loss_floor"), gate.KL_MIN_NUMERICAL_TOLERANCE),
        gate._check("summary.kl_update_guard.required_kl_nonfinite_count", gate._numeric_equal(kl_guard.get("required_kl_nonfinite_count"), 0.0), kl_guard.get("required_kl_nonfinite_count"), 0.0),
        gate._check("summary.reward_config.reserve_residual_weight", gate._numeric_equal(reward_config.get("reserve_residual_weight"), 0.0), reward_config.get("reserve_residual_weight"), 0.0),
        gate._check("summary.history_iterations", summary_history_iterations == expected_iterations, summary_history_iterations, expected_iterations),
        gate._check("summary.history_matches_train_iterations", summary_history == history, summary_history, history),
        gate._check("summary.iteration_checkpoint_retention.enabled", retention.get("enabled") is True, retention.get("enabled"), True),
        gate._check("summary.iteration_checkpoint_retention.milestones", reported_milestones_ok, reported_milestones, [gate._portable_path(path) for path in expected_milestone_paths]),
        gate._check("train_iterations_read", history_error is None, history_error, None),
        gate._check("train_iterations_exact_history", history_iterations == expected_iterations, history_iterations, expected_iterations),
        gate._check("all_50_training_iterations_pass_update_contract", failed_iteration_audits == [], failed_iteration_audits, []),
        gate._check("milestone_directory_set", present_names == expected_names, present_names, expected_names),
        gate._check("milestone_staging_absent", staging == [], staging, []),
        gate._check("supervisor_state_read", supervisor_error is None, supervisor_error, None),
        gate._check("supervisor_state.status", _mapping(supervisor).get("status") == "completed", _mapping(supervisor).get("status"), "completed"),
        gate._check("supervisor_state.restart_count", _mapping(supervisor).get("restart_count") == 0, _mapping(supervisor).get("restart_count"), 0),
        gate._check("supervisor_state.crash_restart_count", _mapping(supervisor).get("crash_restart_count") == 0, _mapping(supervisor).get("crash_restart_count"), 0),
        gate._check("supervisor_state.consecutive_skips", _mapping(supervisor).get("consecutive_skips") == 0, _mapping(supervisor).get("consecutive_skips"), 0),
        gate._check("supervisor_state.consecutive_crash_restarts", _mapping(supervisor).get("consecutive_crash_restarts") == 0, _mapping(supervisor).get("consecutive_crash_restarts"), 0),
        gate._check("supervisor_state.skipped_iterations", _mapping(supervisor).get("skipped_iterations") == [], _mapping(supervisor).get("skipped_iterations"), []),
        gate._check("supervisor_state.resume_from", gate._same_path(_mapping(supervisor).get("resume_from"), source_checkpoint), _mapping(supervisor).get("resume_from"), gate._portable_path(source_checkpoint)),
        gate._check("source_checkpoint", _directory_nonempty(source_checkpoint), gate._portable_path(source_checkpoint), "non-empty directory"),
        gate._check("source_checkpoint_actor_digest", source_checkpoint_actor_error is None and source_checkpoint_actor_digest == context["source_actor_digest"], {"module": gate._portable_path(source_actor_module), "digest": source_checkpoint_actor_digest, "error": source_checkpoint_actor_error}, context["source_actor_digest"]),
        gate._check("source_checkpoint_meta_read", source_meta_error is None, source_meta_error, None),
        gate._check("source_checkpoint_meta_sha256", source_meta_digest == context["source_checkpoint_meta_sha256"], source_meta_digest, context["source_checkpoint_meta_sha256"]),
        gate._check("source_checkpoint_meta_contract", source_meta_map.get("logical_iteration") == 1 and source_meta_map.get("rllib_training_iteration") == 1 and gate._same_path(source_meta_map.get("checkpoint"), source_checkpoint), dict(source_meta_map), {"logical_iteration": 1, "rllib_training_iteration": 1, "checkpoint": gate._portable_path(source_checkpoint)}),
        *resolved_config["checks"],
    ]
    artifacts = [
        _milestone_artifact(run_dir, iteration)
        for iteration in EXPECTED_RETAINED_LOGICAL_ITERATIONS
    ]
    if any(artifact["status"] != "PASS" for artifact in artifacts):
        checks.append(gate._check("milestone_artifact_contracts", False, [artifact["logical_iteration"] for artifact in artifacts if artifact["status"] != "PASS"], []))
    else:
        checks.append(gate._check("milestone_artifact_contracts", True, [], []))
    final_module = gate._checkpoint_artifact_report(run_dir / "rl_module_last")
    checks.append(gate._check("canonical_final_rl_module", final_module["status"] == "PASS", final_module.get("failures"), []))
    final_meta, final_meta_error = _safe_json(run_dir / "checkpoint_last_meta.json")
    checks.append(gate._check("canonical_final_checkpoint", _directory_nonempty(run_dir / "checkpoint_last") and final_meta_error is None and _mapping(final_meta).get("logical_iteration") == 51 and gate._same_path(_mapping(final_meta).get("checkpoint"), run_dir / "checkpoint_last"), {"meta": final_meta, "error": final_meta_error}, {"logical_iteration": 51, "checkpoint": gate._portable_path(run_dir / "checkpoint_last")}))
    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    return {
        "status": "PASS" if not failed else "FAIL",
        "run_dir": gate._portable_path(run_dir),
        "summary_path": gate._portable_path(summary_path),
        "train_iterations_path": gate._portable_path(history_path),
        "checks": checks,
        "failed_checks": failed,
        "milestone_artifacts": artifacts,
        "canonical_final_rl_module": final_module,
        "training_iteration_audits": all_iteration_audits,
        "resolved_config_contract": resolved_config,
    }


def _path_within(path: Path, root: Path) -> bool:
    try:
        path.expanduser().resolve(strict=False).relative_to(
            root.expanduser().resolve(strict=False)
        )
    except (OSError, RuntimeError, ValueError):
        return False
    return True


def _restart_audit_contract(
    context: Mapping[str, Any],
    *,
    audit_path: Path,
    expected_ray_log_dir: Path,
) -> dict[str, Any]:
    """Bind a successful post-run Ray restart audit to this exact run/log."""

    run_dir = Path(context["run_dir"])
    summary_path = run_dir / "summary.json"
    history_path = run_dir / "train_iterations.jsonl"
    watchdog_path = run_dir / "watchdog_state.json"
    payload, payload_error = _safe_json(audit_path)
    watchdog, watchdog_error = _safe_json(watchdog_path)
    audit = _mapping(payload)
    audit_summary = _mapping(audit.get("summary"))
    iteration_history = _mapping(audit.get("iteration_history"))
    contract = _mapping(audit.get("contract"))
    ray_logs = _mapping(audit.get("ray_driver_logs"))
    audit_checks = _mapping(audit.get("checks"))
    expected_ray_log_dir = expected_ray_log_dir.expanduser().resolve(strict=False)
    declared_ray_log_dir = _path_from_protocol(audit.get("ray_log_dir"))
    watchdog_pid = _mapping(watchdog).get("pid")
    driver_pid = ray_logs.get("driver_pid")
    required_audit_checks = {
        "run_directory_exists",
        "completed_summary_available",
        "training_completed_ok",
        "no_supervisor_restarts",
        "no_skipped_iterations",
        "iteration_history_available",
        "iteration_history_parseable",
        "expected_env_runner_count_resolved",
        "driver_pid_resolved",
        "scoped_ray_driver_log_available",
        "ray_driver_log_readable",
        "ray_driver_has_env_runner_lifecycle_evidence",
        "no_ray_restart_or_failure_evidence",
        "fault_tolerance_metrics_well_formed_if_present",
        "fault_tolerance_restart_count_zero_if_present",
        "healthy_env_runner_count_matches_if_present",
    }
    scanned_files = ray_logs.get("scanned_files")
    scanned_paths = (
        [
            _path_from_protocol(item.get("path"))
            for item in scanned_files
            if isinstance(item, Mapping)
        ]
        if isinstance(scanned_files, list)
        else []
    )
    scoped_scanned_paths = (
        bool(scanned_paths)
        and all(
            path is not None and _path_within(path, expected_ray_log_dir)
            for path in scanned_paths
        )
    )
    checks = [
        gate._check("restart_audit.read", payload_error is None, payload_error, None),
        gate._check("restart_audit.schema_version", audit.get("schema_version") == restart_audit.SCHEMA_VERSION, audit.get("schema_version"), restart_audit.SCHEMA_VERSION),
        gate._check("restart_audit.ok", audit.get("ok") is True, audit.get("ok"), True),
        gate._check("restart_audit.status", audit.get("status") == "PASS", audit.get("status"), "PASS"),
        gate._check("restart_audit.run_dir", gate._same_path(audit.get("run_dir"), run_dir), audit.get("run_dir"), gate._portable_path(run_dir)),
        gate._check("restart_audit.summary_path", gate._same_path(audit.get("summary_path"), summary_path), audit.get("summary_path"), gate._portable_path(summary_path)),
        gate._check("restart_audit.history_path", gate._same_path(audit.get("history_path"), history_path), audit.get("history_path"), gate._portable_path(history_path)),
        gate._check("restart_audit.ray_log_dir", declared_ray_log_dir is not None and declared_ray_log_dir == expected_ray_log_dir, gate._portable_path(declared_ray_log_dir) if declared_ray_log_dir is not None else audit.get("ray_log_dir"), gate._portable_path(expected_ray_log_dir)),
        gate._check("restart_audit.ray_log_dir_exists", expected_ray_log_dir.is_dir(), gate._portable_path(expected_ray_log_dir), "existing directory"),
        gate._check("restart_audit.contract.post_run_only", contract.get("post_run_only") is True, contract.get("post_run_only"), True),
        gate._check("restart_audit.contract.require_completed_summary", contract.get("require_completed_summary") is True, contract.get("require_completed_summary"), True),
        gate._check("restart_audit.contract.require_no_supervisor_restart_or_skip", contract.get("require_no_supervisor_restart_or_skip") is True, contract.get("require_no_supervisor_restart_or_skip"), True),
        gate._check("restart_audit.contract.require_no_ray_restart_evidence", contract.get("require_no_ray_restart_evidence") is True, contract.get("require_no_ray_restart_evidence"), True),
        gate._check("restart_audit.contract.expected_num_env_runners", contract.get("expected_num_env_runners") == 12, contract.get("expected_num_env_runners"), 12),
        gate._check("restart_audit.summary.ok", audit_summary.get("ok") is True, audit_summary.get("ok"), True),
        gate._check("restart_audit.summary.stop_reason", audit_summary.get("stop_reason") == "completed", audit_summary.get("stop_reason"), "completed"),
        gate._check("restart_audit.summary.iterations_completed", audit_summary.get("iterations_completed") == 51, audit_summary.get("iterations_completed"), 51),
        gate._check("restart_audit.summary.restart_count", audit_summary.get("restart_count") == 0, audit_summary.get("restart_count"), 0),
        gate._check("restart_audit.summary.crash_restart_count", audit_summary.get("crash_restart_count") == 0, audit_summary.get("crash_restart_count"), 0),
        gate._check("restart_audit.summary.crash_restarts", audit_summary.get("crash_restarts") == [], audit_summary.get("crash_restarts"), []),
        gate._check("restart_audit.summary.skipped_iterations", audit_summary.get("skipped_iterations") == [], audit_summary.get("skipped_iterations"), []),
        gate._check("restart_audit.iteration_history.rows", iteration_history.get("rows") == 50, iteration_history.get("rows"), 50),
        gate._check("restart_audit.iteration_history.invalid_line_numbers", iteration_history.get("invalid_line_numbers") == [], iteration_history.get("invalid_line_numbers"), []),
        gate._check("restart_audit.failed_checks", audit.get("failed_checks") == [], audit.get("failed_checks"), []),
        gate._check("restart_audit.checks_complete_and_pass", set(audit_checks) == required_audit_checks and all(value is True for value in audit_checks.values()), dict(audit_checks), {name: True for name in sorted(required_audit_checks)}),
        gate._check("restart_audit.watchdog_read", watchdog_error is None, watchdog_error, None),
        gate._check("restart_audit.driver_pid_matches_watchdog", type(driver_pid) is int and type(watchdog_pid) is int and driver_pid == watchdog_pid and driver_pid > 0, driver_pid, watchdog_pid),
        gate._check("restart_audit.driver_log_resolution", ray_logs.get("resolution") == "pid", ray_logs.get("resolution"), "pid"),
        gate._check("restart_audit.scanned_logs_scoped", scoped_scanned_paths, [gate._portable_path(path) if path is not None else None for path in scanned_paths], f"one or more files within {gate._portable_path(expected_ray_log_dir)}"),
        gate._check("restart_audit.ray_driver_log_read_errors", ray_logs.get("read_errors") == [], ray_logs.get("read_errors"), []),
        gate._check("restart_audit.ray_driver_log_findings", ray_logs.get("findings") == [] and ray_logs.get("finding_count") == 0, {"findings": ray_logs.get("findings"), "finding_count": ray_logs.get("finding_count")}, {"findings": [], "finding_count": 0}),
        gate._check("restart_audit.ray_driver_lifecycle_evidence", isinstance(ray_logs.get("env_runner_lifecycle_lines"), int) and ray_logs.get("env_runner_lifecycle_lines") > 0, ray_logs.get("env_runner_lifecycle_lines"), "> 0"),
    ]
    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    return {
        "status": "PASS" if not failed else "FAIL",
        "path": gate._portable_path(audit_path),
        "sha256": (
            gate._sha256(audit_path)
            if payload_error is None and audit_path.is_file()
            else None
        ),
        "checks": checks,
        "failed_checks": failed,
        "audit": dict(audit),
    }


def _reference_contract(context: Mapping[str, Any], output_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any] | None], tuple[gate.RolloutSpec, ...]]:
    offsets = tuple(context["start_offsets_s"])
    specs = gate._rollout_specs(offsets)
    reference_root = Path(context["reference_summaries_root"])
    summaries = tuple(reference_root / spec.name / "rollout_summary.json" for spec in specs)
    reference_checkpoint = Path(context["reference_checkpoint"])
    reference_config = gate.GateConfig(
        checkpoint=reference_checkpoint,
        output_dir=output_dir,
        reserve_reference_checkpoint=reference_checkpoint,
        reserve_reference_summaries=summaries,
        expected_sigma=float(context["expected_sigma"]),
        start_offsets_s=offsets,
    )
    report, caps = gate._classify_reserve_reference(reference_config, specs)
    assert report is not None
    try:
        actual_actor_digest = _actor_digest(_load_state(reference_checkpoint))
        actor_digest_error = None
    except (OSError, ValueError, TypeError) as exc:
        actual_actor_digest = None
        actor_digest_error = str(exc)
    expected_actor_digest = context["source_actor_digest"]
    actor_identity = {
        "status": (
            "PASS"
            if actor_digest_error is None
            and actual_actor_digest == expected_actor_digest
            else "FAIL"
        ),
        "reference_checkpoint": gate._portable_path(reference_checkpoint),
        "actual_actor_digest": actual_actor_digest,
        "expected_actor_digest": expected_actor_digest,
        "error": actor_digest_error,
    }
    report["source_actor_identity"] = actor_identity
    if actor_identity["status"] != "PASS":
        report["status"] = "FAIL"
        caps = [None] * len(specs)
    return report, caps, specs


def _case_result(
    *,
    candidate_checkpoint: Path,
    spec: gate.RolloutSpec,
    cap: Mapping[str, Any],
    case_dir: Path,
    config: PilotScreenConfig,
    start_offsets_s: tuple[float, float, float],
    expected_sigma: float,
    command_runner: Callable[..., Any],
) -> dict[str, Any]:
    rollout_config = gate.GateConfig(
        checkpoint=candidate_checkpoint,
        output_dir=case_dir.parent,
        expected_sigma=expected_sigma,
        start_offsets_s=start_offsets_s,
        run_rollouts=True,
        python_executable=config.python_executable,
        rollout_script=config.rollout_script,
        rollout_timeout_s=config.rollout_timeout_s,
    )
    command = gate._rollout_command(rollout_config, spec, case_dir)
    seed_index = command.index("--seed") + 1
    command_seed = int(command[seed_index])
    invocation: dict[str, Any] = {
        "name": spec.name,
        "status": "FAIL",
        "command": command,
        "output_dir": gate._portable_path(case_dir),
        "returncode": None,
    }
    if command_seed != EXPECTED_DEVELOPMENT_SEED or command_seed in SEALED_HELD_OUT_SEEDS:
        invocation["error"] = "development-only seed guard rejected rollout command"
        return {
            "status": "FAIL",
            "operational_failure": True,
            "invocation": invocation,
            "classification": None,
        }
    if case_dir.exists():
        invocation["error"] = "refusing to reuse an existing rollout output"
        return {
            "status": "FAIL",
            "operational_failure": True,
            "invocation": invocation,
            "classification": None,
        }
    try:
        completed = command_runner(command, check=False)
        returncode = int(completed.returncode)
        invocation["returncode"] = returncode
        invocation["status"] = "PASS" if returncode == 0 else "FAIL"
        if returncode != 0:
            invocation["error"] = f"rollout process returned {returncode}"
    except Exception as exc:  # command boundary: convert launch failures to report data
        invocation["error"] = f"could not launch rollout: {exc}"

    summary_path = case_dir / "rollout_summary.json"
    classification: dict[str, Any]
    try:
        summary = gate._read_json_object(summary_path)
        reference_cap = float(cap["reference_cap_nm"])
        numerical_tolerance = float(cap["numerical_tolerance_nm"])
        classification = gate.classify_rollout_summary(
            summary,
            expected_checkpoint=candidate_checkpoint,
            spec=spec,
            expected_sigma=expected_sigma,
            max_reserve_norm_nm=reference_cap,
            reserve_numerical_tolerance_nm=numerical_tolerance,
        )
        classification["summary_path"] = gate._portable_path(summary_path)
        classification["summary_sha256"] = gate._sha256(summary_path)
        observed_reserve = gate._finite_number(summary.get("reserve_norm_max_nm"))
        observed_penetration = gate._finite_number(
            summary.get("grf_penetration_max_m")
        )
        reference_summary = gate._read_json_object(Path(cap["summary_path"]))
        reference_penetration = gate._finite_number(
            reference_summary.get("grf_penetration_max_m")
        )
        classification["observed"] = {
            "reserve_norm_max_nm": summary.get("reserve_norm_max_nm"),
            "grf_penetration_max_m": summary.get("grf_penetration_max_m"),
            "reserve_ratio_vs_h0": (
                observed_reserve / reference_cap
                if observed_reserve is not None and reference_cap > 0.0
                else None
            ),
            "penetration_ratio_vs_h0": (
                observed_penetration / reference_penetration
                if observed_penetration is not None
                and reference_penetration is not None
                and reference_penetration > 0.0
                else None
            ),
        }
        classification["reserve_contract"].update(
            {
                "mode": gate.RESERVE_MODE_CONDITION_MATCHED,
                "reference_summary_path": cap["summary_path"],
                "reference_summary_sha256": cap["summary_sha256"],
                "reference_cap_nm": reference_cap,
                "tolerance_formula": cap["tolerance_formula"],
            }
        )
        classification_read_failure = False
    except (OSError, ValueError, KeyError, TypeError) as exc:
        classification = {
            "name": spec.name,
            "status": "FAIL",
            "summary_path": gate._portable_path(summary_path),
            "error": str(exc),
            "checks": [],
            "failed_checks": ["summary_read_or_reserve_contract"],
        }
        classification_read_failure = True
    passed = invocation["status"] == "PASS" and classification["status"] == "PASS"
    return {
        "status": "PASS" if passed else "FAIL",
        "operational_failure": invocation["status"] != "PASS" or classification_read_failure,
        "invocation": invocation,
        "classification": classification,
    }


def _screen_milestone(
    *,
    logical_iteration: int,
    run_dir: Path,
    history_path: Path,
    milestone_output: Path,
    specs: tuple[gate.RolloutSpec, ...],
    caps: Sequence[Mapping[str, Any] | None],
    config: PilotScreenConfig,
    start_offsets_s: tuple[float, float, float],
    expected_sigma: float,
    command_runner: Callable[..., Any],
) -> dict[str, Any]:
    candidate_checkpoint = run_dir / f"{MILESTONE_PREFIX}{logical_iteration:06d}" / "rl_module_last"
    training = gate.classify_training_iterations(
        history_path,
        start_offsets_s=start_offsets_s,
        expected_training_iteration=logical_iteration,
    )
    result: dict[str, Any] = {
        "logical_iteration": logical_iteration,
        "pilot_update_index": logical_iteration - 1,
        "candidate_checkpoint": gate._checkpoint_artifact_report(candidate_checkpoint),
        "training": training,
        "critical": None,
        "deterministic": [],
        "status": "REJECTED",
        "operational_failure": False,
    }
    if training.get("status") != "PASS":
        result["critical"] = {
            "status": "SKIPPED",
            "reason": "training_iteration_audit_failed",
        }
        result["deterministic"] = {
            "status": "SKIPPED",
            "reason": "training_iteration_audit_failed",
            "cases": [],
        }
        return result

    rollout_root = milestone_output / "rollouts"
    rollout_root.mkdir(parents=True, exist_ok=False)
    critical_spec = specs[3]
    critical_cap = caps[3]
    assert critical_cap is not None
    critical = _case_result(
        candidate_checkpoint=candidate_checkpoint,
        spec=critical_spec,
        cap=critical_cap,
        case_dir=rollout_root / critical_spec.name,
        config=config,
        start_offsets_s=start_offsets_s,
        expected_sigma=expected_sigma,
        command_runner=command_runner,
    )
    result["critical"] = critical
    result["operational_failure"] = bool(critical["operational_failure"])
    if critical["status"] != "PASS":
        result["deterministic"] = {
            "status": "SKIPPED",
            "reason": "critical_stochastic_plus020_failed",
            "cases": [],
        }
        return result

    deterministic_cases = []
    for index, spec in enumerate(specs[:3]):
        cap = caps[index]
        assert cap is not None
        case = _case_result(
            candidate_checkpoint=candidate_checkpoint,
            spec=spec,
            cap=cap,
            case_dir=rollout_root / spec.name,
            config=config,
            start_offsets_s=start_offsets_s,
            expected_sigma=expected_sigma,
            command_runner=command_runner,
        )
        deterministic_cases.append(case)
        result["operational_failure"] = bool(result["operational_failure"] or case["operational_failure"])
    deterministic_passed = all(case["status"] == "PASS" for case in deterministic_cases)
    result["deterministic"] = {
        "status": "PASS" if deterministic_passed else "FAIL",
        "cases": deterministic_cases,
    }
    if deterministic_passed:
        result["status"] = "ELIGIBLE"
    return result


def screen_pilot(
    config: PilotScreenConfig,
    *,
    command_runner: Callable[..., Any] = subprocess.run,
) -> dict[str, Any]:
    """Validate and screen the pilot, writing one aggregate report atomically."""

    output_dir = config.output_dir.expanduser().resolve(strict=False)
    if output_dir.exists():
        raise FileExistsError(f"screen output already exists: {output_dir}")

    output_created = False

    def create_output() -> None:
        nonlocal output_created
        if output_created:
            return
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        output_dir.mkdir(exist_ok=False)
        output_created = True

    report_path = output_dir / REPORT_FILENAME
    protocol_path = config.protocol.expanduser().resolve(strict=False)
    report: dict[str, Any] = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "screen": "ppo_pilot_screen",
        "status": "FAIL",
        "ok": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": {
            "path": gate._portable_path(protocol_path),
            "sha256": None,
            "canonical_content_sha256": None,
            "contract": None,
        },
        "output": gate._portable_path(report_path),
        "run_validation": None,
        "restart_audit": None,
        "reserve_reference": None,
        "screened_logical_iterations": list(EXPECTED_SCREENED_LOGICAL_ITERATIONS),
        "milestones": [],
        "eligible_logical_iterations": [],
        "rejected_logical_iterations": [],
        "screening_completed": False,
        "operational_failures": [],
        "held_out": {
            "status": "SEALED",
            "declared_seeds": list(SEALED_HELD_OUT_SEEDS),
            "seeds_used": [],
            "opened": False,
        },
        "fallback_attempted": False,
        "checkpoint_selected": None,
        "checkpoint_promoted": False,
        "checkpoint_copied": False,
    }
    try:
        protocol = gate._read_json_object(protocol_path)
        report["protocol"]["sha256"] = gate._sha256(protocol_path)
        report["protocol"]["canonical_content_sha256"] = (
            _canonical_json_sha256(protocol)
        )
    except (OSError, ValueError) as exc:
        report["protocol"]["contract"] = {
            "status": "FAIL",
            "error": str(exc),
            "checks": [],
            "failed_checks": ["protocol_read"],
        }
        create_output()
        gate._write_report(report_path, report)
        return report

    if (
        report["protocol"]["canonical_content_sha256"]
        != config.expected_protocol_content_sha256
    ):
        report["protocol"]["contract"] = {
            "status": "FAIL",
            "checks": [
                gate._check(
                    "protocol_canonical_content_sha256",
                    False,
                    report["protocol"]["canonical_content_sha256"],
                    config.expected_protocol_content_sha256,
                )
            ],
            "failed_checks": ["protocol_canonical_content_sha256"],
        }
        create_output()
        gate._write_report(report_path, report)
        return report

    protocol_report, context = _protocol_contract(protocol)
    report["protocol"]["contract"] = protocol_report
    if context is None:
        create_output()
        gate._write_report(report_path, report)
        return report

    protected_directories = {
        Path(context["run_dir"]),
        Path(context["source_checkpoint"]).parent,
        Path(context["resolved_config_source"]).parent,
        Path(context["reference_checkpoint"]).parent,
        Path(context["reference_summaries_root"]),
        protocol_path.parent,
        config.ray_log_dir,
    }
    collisions = sorted(
        gate._portable_path(path)
        for path in protected_directories
        if _path_within(output_dir, path)
    )
    if collisions:
        raise ValueError(
            "screen output must be outside immutable input/run/log directories: "
            + ", ".join(collisions)
        )
    create_output()

    timeout = gate._finite_number(config.rollout_timeout_s)
    runtime_failures = []
    if timeout is None or timeout <= 0.0:
        runtime_failures.append("rollout_timeout_s_must_be_positive")
    if not config.rollout_script.is_file():
        runtime_failures.append("rollout_script_not_found")
    elif not gate._same_path(str(config.rollout_script), gate.DEFAULT_ROLLOUT_SCRIPT):
        runtime_failures.append("rollout_script_must_be_canonical")
    if not config.python_executable.strip():
        runtime_failures.append("python_executable_empty")
    elif not gate._same_path(config.python_executable, Path(sys.executable)):
        runtime_failures.append("python_executable_must_match_screen_interpreter")
    if runtime_failures:
        report["operational_failures"] = runtime_failures
        gate._write_report(report_path, report)
        return report

    run_validation = _validate_completed_run(context)
    report["run_validation"] = run_validation
    restart_report = _restart_audit_contract(
        context,
        audit_path=config.restart_audit.expanduser().resolve(strict=False),
        expected_ray_log_dir=config.ray_log_dir,
    )
    report["restart_audit"] = restart_report
    reference, caps, specs = _reference_contract(context, output_dir)
    report["reserve_reference"] = reference
    if (
        run_validation["status"] != "PASS"
        or restart_report["status"] != "PASS"
        or reference["status"] != "PASS"
        or any(cap is None for cap in caps)
    ):
        gate._write_report(report_path, report)
        return report

    run_dir = Path(context["run_dir"])
    history_path = run_dir / "train_iterations.jsonl"
    for logical_iteration in EXPECTED_SCREENED_LOGICAL_ITERATIONS:
        milestone_output = output_dir / f"{MILESTONE_PREFIX}{logical_iteration:06d}"
        milestone_output.mkdir(exist_ok=False)
        milestone_result = _screen_milestone(
            logical_iteration=logical_iteration,
            run_dir=run_dir,
            history_path=history_path,
            milestone_output=milestone_output,
            specs=specs,
            caps=caps,
            config=config,
            start_offsets_s=tuple(context["start_offsets_s"]),
            expected_sigma=float(context["expected_sigma"]),
            command_runner=command_runner,
        )
        report["milestones"].append(milestone_result)

    eligible = [
        item["logical_iteration"]
        for item in report["milestones"]
        if item["status"] == "ELIGIBLE"
    ]
    rejected = [
        item["logical_iteration"]
        for item in report["milestones"]
        if item["status"] != "ELIGIBLE"
    ]
    operational = [
        item["logical_iteration"]
        for item in report["milestones"]
        if item["operational_failure"]
    ]
    report["eligible_logical_iterations"] = eligible
    report["rejected_logical_iterations"] = rejected
    report["operational_failures"] = operational
    report["screening_completed"] = len(report["milestones"]) == len(EXPECTED_SCREENED_LOGICAL_ITERATIONS) and not operational
    passed = bool(eligible) and report["screening_completed"]
    report["status"] = "PASS" if passed else "FAIL"
    report["ok"] = passed
    gate._write_report(report_path, report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--restart-audit", type=Path, required=True)
    parser.add_argument("--ray-log-dir", type=Path, required=True)
    parser.add_argument("--rollout-timeout-s", type=float, default=900.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = screen_pilot(
            PilotScreenConfig(
                protocol=args.protocol,
                output_dir=args.output_dir,
                restart_audit=args.restart_audit,
                ray_log_dir=args.ray_log_dir,
                rollout_timeout_s=args.rollout_timeout_s,
            )
        )
    except (FileExistsError, ValueError) as exc:
        print(json.dumps({"ok": False, "status": "FAIL", "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
