"""Validate the imitation-to-ex-novo actor warm-start port.

This is a pre-training gate. It checks actor tensor/function equivalence,
feature semantics, actor-only isolation, RLlib integration evidence, and
optional real-observation rollout traces. It does not train PPO.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import pickle
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import training_config
import warm_start

DEFAULT_TARGET_MODULE = (
    REPO_ROOT
    / "Trajectory Generator"
    / "runs"
    / "training"
    / "_warm_start_smoke_20260709_smallbatch"
    / "rl_module_initial_warm_start"
)
DEFAULT_TARGET_CONFIG = BASELINE_DIR / "training_exnovo_cfg.yaml"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _file_sha256(path: Path) -> str:
    return _sha256(path.read_bytes())


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_pickle(path: Path) -> Any:
    with path.open("rb") as handle:
        return pickle.load(handle)


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _nested(mapping: Mapping[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = mapping
    for key in keys:
        if not isinstance(value, Mapping) or key not in value:
            return default
        value = value[key]
    return value


def _module_paths(module_dir: Path) -> dict[str, Path]:
    return {
        "state": module_dir / "module_state.pkl",
        "ctor": module_dir / "class_and_ctor_args.pkl",
        "metadata": module_dir / "metadata.json",
    }


def _resolve_report(module_dir: Path, explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    return module_dir.parent / "actor_transplant_report.json"


def _forward(state: Mapping[str, Any], inputs: np.ndarray) -> np.ndarray:
    hidden = np.tanh(
        inputs @ _array(state["pi.0.0.weight"]).T
        + _array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ _array(state["pi.0.2.weight"]).T
        + _array(state["pi.0.2.bias"])
    )
    return (
        hidden @ _array(state["pi.1.weight"]).T
        + _array(state["pi.1.bias"])
    )


def _adapt_source_state(
    source_state: Mapping[str, Any],
    source_features: Sequence[str],
    zero_features: Sequence[str],
) -> dict[str, Any]:
    adapted = {
        key: value.copy() if hasattr(value, "copy") else copy.deepcopy(value)
        for key, value in source_state.items()
    }
    index = {name: i for i, name in enumerate(source_features)}
    for key in warm_start._FIRST_LAYER_WEIGHT_KEYS:
        for name in zero_features:
            if name in index:
                adapted[key][:, index[name]] = 0.0
    return adapted


def _profile_payload(
    configured_path: str,
    output_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    relative = Path(configured_path.replace("\\", "/"))
    path = relative if relative.is_absolute() else REPO_ROOT / relative
    if path.is_file():
        data = path.read_bytes()
        provenance = "worktree"
        resolved = path.resolve()
    else:
        git_key = relative.as_posix()
        data = subprocess.check_output(
            ["git", "show", f"HEAD:{git_key}"],
            cwd=REPO_ROOT,
        )
        provenance = "git_HEAD_recovery"
        resolved = output_dir / "recovered_source_online_grf_profile.json"
        resolved.write_bytes(data)
    return (
        json.loads(data.decode("utf-8")),
        {
            "configured_path": configured_path,
            "resolved_path": str(resolved),
            "provenance": provenance,
            "sha256": _sha256(data),
        },
        resolved,
    )


def _profile_physics(profile: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "ground": profile.get("ground"),
        "material": profile.get("material"),
        "spheres": profile.get("spheres"),
    }


class Audit:
    def __init__(self) -> None:
        self.checks: list[dict[str, Any]] = []

    def require(self, name: str, condition: bool, details: Any = None) -> None:
        self.checks.append(
            {
                "name": name,
                "status": "PASS" if condition else "FAIL",
                "details": details,
            }
        )

    def warn(self, name: str, details: Any) -> None:
        self.checks.append({"name": name, "status": "WARN", "details": details})

    @property
    def failures(self) -> list[dict[str, Any]]:
        return [item for item in self.checks if item["status"] == "FAIL"]

    @property
    def warnings(self) -> list[dict[str, Any]]:
        return [item for item in self.checks if item["status"] == "WARN"]


def _config_contract(
    audit: Audit,
    source_cfg: Mapping[str, Any],
    target_cfg: Mapping[str, Any],
) -> dict[str, Any]:
    equal_fields = (
        ("model", "num_hidden_layers"),
        ("model", "dim_hidden_layers"),
        ("model", "fcnet_activation"),
        ("simulation", "segment_duration"),
        ("simulation", "policy_knots"),
        ("simulation", "action_mode"),
        ("simulation", "max_delta_rad"),
        ("simulation", "pros_ref_governor"),
        ("simulation", "pros_ref_model"),
        ("simulation", "pros_ref_cutoff_hz"),
        ("simulation", "pros_knee_ref_velocity_limit_rad_s"),
        ("simulation", "pros_ankle_ref_velocity_limit_rad_s"),
        ("simulation", "pros_knee_ref_acceleration_limit_rad_s2"),
        ("simulation", "pros_ankle_ref_acceleration_limit_rad_s2"),
        ("simulation", "pros_knee_ref_jerk_limit_rad_s3"),
        ("simulation", "pros_ankle_ref_jerk_limit_rad_s3"),
    )
    results = {}
    for section, key in equal_fields:
        source_value = _nested(source_cfg, section, key)
        target_value = _nested(target_cfg, section, key)
        same = source_value == target_value
        label = f"config_contract.{section}.{key}"
        results[label] = {"source": source_value, "target": target_value}
        audit.require(label, same, results[label])
    return results


def _ctor_contract(
    audit: Audit,
    source_ctor: Mapping[str, Any],
    target_ctor: Mapping[str, Any],
) -> dict[str, Any]:
    source_kwargs = source_ctor["ctor_args_and_kwargs"][1]
    target_kwargs = target_ctor["ctor_args_and_kwargs"][1]
    source_model = source_kwargs["model_config"]
    target_model = target_kwargs["model_config"]
    details = {
        "source_class": source_ctor["class"].__name__,
        "target_class": target_ctor["class"].__name__,
        "source_action_shape": list(source_kwargs["action_space"].shape),
        "target_action_shape": list(target_kwargs["action_space"].shape),
        "source_n_actor": int(source_model["n_actor"]),
        "target_n_actor": int(target_model["n_actor"]),
        "source_hiddens": list(source_model["fcnet_hiddens"]),
        "target_hiddens": list(target_model["fcnet_hiddens"]),
        "source_activation": source_model["fcnet_activation"],
        "target_activation": target_model["fcnet_activation"],
    }
    audit.require(
        "module_class_match",
        details["source_class"] == details["target_class"],
        details,
    )
    audit.require(
        "action_space_shape_match",
        details["source_action_shape"] == details["target_action_shape"] == [2],
        details,
    )
    audit.require(
        "hidden_architecture_match",
        details["source_hiddens"] == details["target_hiddens"]
        and details["source_activation"] == details["target_activation"],
        details,
    )
    return details


def _trace_equivalence(
    trace_path: Path,
    source_state: Mapping[str, Any],
    target_state: Mapping[str, Any],
    source_features: Sequence[str],
    target_features: Sequence[str],
) -> dict[str, Any]:
    rows = _load_json(trace_path)
    source_outputs = []
    target_outputs = []
    recorded_actions = []
    for row in rows:
        values = row.get("actor_observation_before", {})
        if not isinstance(values, Mapping):
            continue
        source_inputs = np.asarray(
            [[float(values[name]) for name in source_features]], dtype=np.float32
        )
        target_inputs = np.asarray(
            [[float(values[name]) for name in target_features]], dtype=np.float32
        )
        source_outputs.append(_forward(source_state, source_inputs)[0])
        target_outputs.append(_forward(target_state, target_inputs)[0])
        recorded_actions.append(np.asarray(row["raw_policy_action"], dtype=float))
    if not source_outputs:
        raise ValueError(f"No actor_observation_before rows in {trace_path}")
    source_array = np.asarray(source_outputs)
    target_array = np.asarray(target_outputs)
    action_array = np.asarray(recorded_actions)
    return {
        "samples": int(len(source_array)),
        "source_target_logits_max_abs_diff": float(
            np.max(np.abs(source_array - target_array))
        ),
        "source_target_logits_rmse": float(
            np.sqrt(np.mean((source_array - target_array) ** 2))
        ),
        "target_mean_recorded_action_max_abs_diff": float(
            np.max(np.abs(target_array[:, : action_array.shape[1]] - action_array))
        ),
    }


def _write_markdown(path: Path, report: Mapping[str, Any]) -> None:
    lines = [
        "# Warm-start Port Validation",
        "",
        f"- Status: **{report['status']}**",
        f"- Hard failures: {len(report['failures'])}",
        f"- Warnings: {len(report['warnings'])}",
        f"- Source: `{report['inputs']['source_module']}`",
        f"- Target: `{report['inputs']['target_module']}`",
        "",
        "## Checks",
        "",
    ]
    for check in report["checks"]:
        lines.append(f"- [{check['status']}] `{check['name']}`")
    lines.extend(
        [
            "",
            "## Decision",
            "",
            (
                "The actor port is technically validated. Documented domain shifts "
                "remain for the behavioral initial rollout."
                if report["status"].startswith("PASS")
                else "The actor port is not ready for H1; resolve hard failures first."
            ),
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    source_module = warm_start.resolve_source_checkpoint(args.source_module)
    target_module = Path(args.target_module).expanduser().resolve()
    source_config_path = Path(args.source_config).expanduser().resolve()
    target_config_path = Path(args.target_config).expanduser().resolve()
    transplant_report_path = _resolve_report(target_module, args.transplant_report)

    audit = Audit()
    source_paths = _module_paths(source_module)
    target_paths = _module_paths(target_module)
    for label, path in {**{f"source_{k}": v for k, v in source_paths.items()},
                        **{f"target_{k}": v for k, v in target_paths.items()},
                        "source_config": source_config_path,
                        "target_config": target_config_path,
                        "transplant_report": transplant_report_path}.items():
        audit.require(f"artifact_exists.{label}", path.is_file(), str(path))
    if audit.failures:
        raise FileNotFoundError(audit.failures)

    source_state = warm_start.load_module_state(source_module)
    target_state = warm_start.load_module_state(target_module)
    source_ctor = _load_pickle(source_paths["ctor"])
    target_ctor = _load_pickle(target_paths["ctor"])
    source_metadata = _load_json(source_paths["metadata"])
    target_metadata = _load_json(target_paths["metadata"])
    source_cfg = training_config.load(source_config_path)
    target_cfg = training_config.load(target_config_path)
    integration_report = _load_json(transplant_report_path)
    source_features = tuple(
        integration_report.get(
            "source_actor_feature_names",
            warm_start.ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES,
        )
    )
    target_features = tuple(integration_report["target_actor_feature_names"])
    target_gait_clock = bool(
        _nested(target_cfg, "simulation", "gait_clock_enable", default=True)
    )
    forced_zero = (
        () if target_gait_clock else warm_start.DISABLED_GAIT_CLOCK_FEATURES
    )

    expected_target_state, expected_report = warm_start.transplant_actor_state(
        target_state=target_state,
        target_actor_feature_names=target_features,
        source_checkpoint=source_module,
        source_config=source_config_path,
        source_actor_feature_manifest=_nested(
            integration_report,
            "source_actor_feature_manifest",
            "path",
        ),
        zero_target_features=forced_zero,
    )
    actual_comparison = warm_start.compare_actor_states(
        expected_target_state,
        target_state,
    )
    audit.require(
        "saved_target_matches_recomputed_transplant",
        actual_comparison["exact"],
        {key: actual_comparison[key] for key in ("max_abs_diff", "expected_digest", "actual_digest")},
    )
    audit.require(
        "source_checkpoint_is_actor_only",
        expected_report["source_state_is_actor_only"],
        expected_report["source_non_actor_keys"],
    )
    integration = integration_report.get("integration_validation", {})
    learner_non_actor = integration.get("learner_non_actor", {})
    audit.require(
        "target_critic_state_unchanged_during_transplant",
        bool(integration_report.get("target_non_actor_state_unchanged"))
        and bool(learner_non_actor.get("exact"))
        and len(learner_non_actor.get("keys", ())) >= 1
        and learner_non_actor.get("expected_digest")
        == learner_non_actor.get("actual_digest"),
        learner_non_actor,
    )
    audit.require(
        "disabled_gait_clock_columns_zeroed",
        set(forced_zero).issubset(
            set(integration_report.get("shared_features_zeroed", []))
        ),
        integration_report.get("shared_features_zeroed"),
    )

    audit.require(
        "learner_actor_matches_transplant",
        bool(_nested(integration, "learner_actor", "exact", default=False)),
        _nested(integration, "learner_actor", "actual_digest"),
    )
    audit.require(
        "env_runner_actors_synced_before_sampling",
        bool(integration.get("weights_synced_before_first_sample"))
        and bool(integration.get("env_runner_actors_exact"))
        and int(integration.get("env_runner_count_checked", 0)) >= 1,
        {
            "count": integration.get("env_runner_count_checked"),
            "digests": integration.get("env_runner_actor_digests"),
        },
    )
    audit.require(
        "saved_initial_actor_matches_live_actor",
        bool(_nested(integration, "saved_initial_actor", "exact", default=False)),
        _nested(integration, "saved_initial_actor", "actual_digest"),
    )
    audit.require(
        "source_optimizer_not_loaded",
        integration.get("optimizer_source_loaded") is False,
        integration.get("optimizer_source_loaded"),
    )

    config_contract = _config_contract(audit, source_cfg, target_cfg)
    ctor_contract = _ctor_contract(audit, source_ctor, target_ctor)
    audit.require(
        "ray_checkpoint_version_match",
        source_metadata.get("ray_version") == target_metadata.get("ray_version")
        and source_metadata.get("checkpoint_version")
        == target_metadata.get("checkpoint_version"),
        {"source": source_metadata, "target": target_metadata},
    )
    audit.require(
        "feature_manifest_widths_match_modules",
        len(source_features) == ctor_contract["source_n_actor"]
        and len(target_features) == ctor_contract["target_n_actor"],
        {"source": len(source_features), "target": len(target_features)},
    )

    source_index = {name: i for i, name in enumerate(source_features)}
    target_index = {name: i for i, name in enumerate(target_features)}
    shared = [name for name in source_features if name in target_index]
    target_only = [name for name in target_features if name not in source_index]
    adapted_source = _adapt_source_state(source_state, source_features, forced_zero)
    rng = np.random.default_rng(args.seed)
    source_inputs = rng.normal(size=(args.samples, len(source_features))).astype(
        np.float32
    )
    target_inputs = rng.normal(size=(args.samples, len(target_features))).astype(
        np.float32
    )
    for name in shared:
        target_inputs[:, target_index[name]] = source_inputs[:, source_index[name]]
    source_logits = _forward(adapted_source, source_inputs)
    target_logits = _forward(target_state, target_inputs)
    logits_diff = source_logits - target_logits
    synthetic_equivalence = {
        "samples": int(args.samples),
        "logits_max_abs_diff": float(np.max(np.abs(logits_diff))),
        "logits_rmse": float(np.sqrt(np.mean(logits_diff**2))),
        "deterministic_action_max_abs_diff": float(
            np.max(np.abs(logits_diff[:, :2]))
        ),
    }
    audit.require(
        "functional_equivalence_on_aligned_inputs",
        synthetic_equivalence["logits_max_abs_diff"] <= args.tolerance,
        synthetic_equivalence,
    )

    target_inputs_zero_extra = target_inputs.copy()
    for name in target_only:
        target_inputs_zero_extra[:, target_index[name]] = 0.0
    target_only_invariance = float(
        np.max(
            np.abs(
                target_logits - _forward(target_state, target_inputs_zero_extra)
            )
        )
    )
    audit.require(
        "target_only_features_initially_inert",
        target_only_invariance <= args.tolerance,
        {"max_abs_diff": target_only_invariance, "features": target_only},
    )

    source_profile_path = str(_nested(source_cfg, "grf", "online_grf_profile"))
    target_profile_path = str(_nested(target_cfg, "grf", "online_grf_profile"))
    source_profile, source_profile_info, recovered_profile = _profile_payload(
        source_profile_path,
        output_dir,
    )
    target_profile, target_profile_info, _ = _profile_payload(
        target_profile_path,
        output_dir,
    )
    profile_physics_equal = _profile_physics(source_profile) == _profile_physics(
        target_profile
    )
    audit.require(
        "online_grf_physics_profile_equivalent",
        profile_physics_equal,
        {"source": source_profile_info, "target": target_profile_info},
    )
    if source_profile_info["provenance"] != "worktree":
        audit.warn(
            "source_profile_recovered_from_git_not_worktree",
            source_profile_info,
        )

    source_detector = _nested(source_cfg, "grf", "online_grf_detector_profile")
    target_detector = _nested(target_cfg, "grf", "online_grf_detector_profile")
    if source_detector != target_detector:
        audit.warn(
            "online_event_detector_domain_shift",
            {"source": source_detector, "target": target_detector},
        )
    source_clock = _nested(
        source_cfg, "simulation", "gait_clock_enable", default="not_recorded"
    )
    if source_clock != target_gait_clock:
        audit.warn(
            "sound_gait_clock_domain_shift_adapted_by_zeroing",
            {
                "source": source_clock,
                "target": target_gait_clock,
                "zeroed_features": list(forced_zero),
            },
        )
    episode_start = {
        "source": _nested(source_cfg, "simulation", "episode_start_offset_s"),
        "target": _nested(target_cfg, "simulation", "episode_start_offset_s"),
    }
    if episode_start["source"] != episode_start["target"]:
        audit.warn("episode_start_domain_shift", episode_start)
    slew_limiter = {
        "source_knee": _nested(
            source_cfg,
            "simulation",
            "pros_knee_target_slew_rate_limit_rad_s",
            default=0.0,
        ),
        "target_knee": _nested(
            target_cfg,
            "simulation",
            "pros_knee_target_slew_rate_limit_rad_s",
            default=0.0,
        ),
        "source_ankle": _nested(
            source_cfg,
            "simulation",
            "pros_ankle_target_slew_rate_limit_rad_s",
            default=0.0,
        ),
        "target_ankle": _nested(
            target_cfg,
            "simulation",
            "pros_ankle_target_slew_rate_limit_rad_s",
            default=0.0,
        ),
    }
    if (
        slew_limiter["source_knee"] != slew_limiter["target_knee"]
        or slew_limiter["source_ankle"] != slew_limiter["target_ankle"]
    ):
        audit.warn("target_slew_limiter_domain_shift", slew_limiter)

    source_first = _array(source_state["pi.0.0.weight"])
    column_norms = np.linalg.norm(source_first, axis=0)
    rank_order = np.argsort(column_norms)[::-1]
    semantic_sensitivity = {}
    for name in forced_zero:
        index = source_index[name]
        semantic_sensitivity[name] = {
            "first_layer_column_l2": float(column_norms[index]),
            "rank_descending": int(np.where(rank_order == index)[0][0]) + 1,
            "feature_count": len(source_features),
        }

    source_rollout = None
    if args.source_rollout_summary:
        source_rollout = _load_json(Path(args.source_rollout_summary))
        audit.require(
            "source_runtime_feature_manifest_matches",
            tuple(source_rollout.get("actor_feature_names", ())) == source_features,
            source_rollout.get("actor_feature_names"),
        )
    target_rollout = None
    if args.target_rollout_summary:
        target_rollout = _load_json(Path(args.target_rollout_summary))
        audit.require(
            "target_runtime_feature_manifest_matches",
            tuple(target_rollout.get("actor_feature_names", ())) == target_features,
            target_rollout.get("actor_feature_names"),
        )

    real_trace = None
    if args.target_policy_trace:
        real_trace = _trace_equivalence(
            Path(args.target_policy_trace),
            adapted_source,
            target_state,
            source_features,
            target_features,
        )
        audit.require(
            "functional_equivalence_on_real_target_observations",
            real_trace["source_target_logits_max_abs_diff"] <= args.tolerance,
            real_trace,
        )
        audit.require(
            "recorded_action_matches_target_actor_mean",
            real_trace["target_mean_recorded_action_max_abs_diff"]
            <= args.tolerance,
            real_trace,
        )
    else:
        audit.require(
            "functional_equivalence_on_real_target_observations",
            False,
            "--target-policy-trace was not provided",
        )

    status = (
        "FAIL"
        if audit.failures
        else ("PASS_WITH_WARNINGS" if audit.warnings else "PASS")
    )
    report = {
        "status": status,
        "port_validated": not bool(audit.failures),
        "behavioral_initial_rollout_pending": True,
        "inputs": {
            "source_module": str(source_module),
            "target_module": str(target_module),
            "source_config": str(source_config_path),
            "target_config": str(target_config_path),
            "transplant_report": str(transplant_report_path),
            "source_rollout_summary": args.source_rollout_summary,
            "target_rollout_summary": args.target_rollout_summary,
            "target_policy_trace": args.target_policy_trace,
        },
        "artifact_hashes": {
            "source_module_state": _file_sha256(source_paths["state"]),
            "target_module_state": _file_sha256(target_paths["state"]),
            "source_config": _file_sha256(source_config_path),
            "target_config": _file_sha256(target_config_path),
        },
        "checks": audit.checks,
        "failures": audit.failures,
        "warnings": audit.warnings,
        "config_contract": config_contract,
        "module_contract": ctor_contract,
        "feature_contract": {
            "source_count": len(source_features),
            "target_count": len(target_features),
            "copied": expected_report["copied_features"],
            "shared_zeroed": expected_report["shared_features_zeroed"],
            "target_only_zeroed": expected_report["target_only_features_zeroed"],
        },
        "synthetic_functional_equivalence": synthetic_equivalence,
        "real_trace_equivalence": real_trace,
        "semantic_sensitivity": semantic_sensitivity,
        "profile_provenance": {
            "source": source_profile_info,
            "target": target_profile_info,
            "recovered_source_path": str(recovered_profile),
        },
        "runtime_manifests": {
            "source": source_rollout,
            "target": target_rollout,
        },
    }
    json_path = output_dir / "warm_start_port_validation.json"
    md_path = output_dir / "warm_start_port_validation.md"
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    _write_markdown(md_path, report)
    report["json_path"] = str(json_path)
    report["markdown_path"] = str(md_path)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-module",
        default=str(warm_start.DEFAULT_WARM_START_SOURCE),
    )
    parser.add_argument("--target-module", default=str(DEFAULT_TARGET_MODULE))
    parser.add_argument(
        "--source-config",
        default=str(warm_start.DEFAULT_WARM_START_SOURCE_CONFIG),
    )
    parser.add_argument("--target-config", default=str(DEFAULT_TARGET_CONFIG))
    parser.add_argument("--transplant-report", default=None)
    parser.add_argument("--source-rollout-summary", default=None)
    parser.add_argument("--target-rollout-summary", default=None)
    parser.add_argument("--target-policy-trace", default=None)
    parser.add_argument(
        "--output-dir",
        default=str(REPO_ROOT / "validation" / "warm_start_port_runs" / "latest"),
    )
    parser.add_argument("--samples", type=int, default=4096)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--tolerance", type=float, default=1e-7)
    return parser.parse_args()


def main() -> int:
    report = run(parse_args())
    print(json.dumps(report, indent=2))
    return 0 if report["port_validated"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
