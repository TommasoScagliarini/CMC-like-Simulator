"""Audit a critic-only rewarm performed with split morphology weights."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from typing import Any

import numpy as np
import yaml


ACTOR_KEYS = (
    "pi.0.0.bias",
    "pi.0.0.weight",
    "pi.0.2.bias",
    "pi.0.2.weight",
    "pi.1.bias",
    "pi.1.weight",
    "pi_encoder.0.bias",
    "pi_encoder.0.weight",
    "pi_encoder.2.bias",
    "pi_encoder.2.weight",
)
LOGSTD_ROWS = slice(2, 4)


def _state(path: Path) -> dict[str, np.ndarray]:
    payload = pickle.loads(path.read_bytes())
    if not isinstance(payload, dict):
        raise TypeError(f"module state is not a dict: {path}")
    return {str(key): np.asarray(value) for key, value in payload.items()}


def _same_bytes(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(left)
    b = np.ascontiguousarray(right)
    return bool(
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def audit(source_module: Path, run_dir: Path) -> dict[str, Any]:
    source_path = source_module / "module_state.pkl"
    target_path = (
        run_dir
        / "checkpoint_last"
        / "learner_group"
        / "learner"
        / "rl_module"
        / "default_policy"
        / "module_state.pkl"
    )
    summary_path = run_dir / "summary.json"
    config_path = run_dir / "training_cfg.resolved.yaml"
    source = _state(source_path)
    target = _state(target_path)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    per_actor = {
        key: bool(key in source and key in target and _same_bytes(source[key], target[key]))
        for key in ACTOR_KEYS
    }
    logstd_exact = bool(
        _same_bytes(source["pi.1.weight"][LOGSTD_ROWS], target["pi.1.weight"][LOGSTD_ROWS])
        and _same_bytes(source["pi.1.bias"][LOGSTD_ROWS], target["pi.1.bias"][LOGSTD_ROWS])
    )
    sigma = np.exp(target["pi.1.bias"][LOGSTD_ROWS].astype(np.float64)).tolist()

    critic_audit = list(summary.get("critic_state_audit") or [])
    critic_before = critic_audit[0].get("critic_digest") if critic_audit else None
    critic_after = critic_audit[-1].get("critic_digest") if critic_audit else None
    actor_audit = list(summary.get("actor_freeze_audit") or [])
    actor_runtime_exact = bool(actor_audit) and all(
        item.get("exact") is True and float(item.get("max_abs_diff", 1.0)) == 0.0
        for item in actor_audit
    )
    history = list(summary.get("history") or [])
    iteration = history[-1] if history else {}
    health = dict(iteration.get("training_health") or {})
    reward = dict(config.get("reward") or {})
    split_weights_exact = bool(
        float(reward.get("morphology_weight")) == 1.05
        and float(reward.get("morphology_knee_weight")) == 1.05
        and float(reward.get("morphology_ankle_weight")) == 1.05
    )
    health_ok = bool(
        float(health.get("observed_rows", 0.0)) == 4096.0
        and float(health.get("missing_telemetry_rows", 1.0)) == 0.0
        and float(health.get("phase_timeout_stance_rows", 1.0)) == 0.0
        and float(health.get("morphology_causal_contract_failure_rows", 1.0)) == 0.0
        and float(health.get("resync_count_max", 1.0)) == 0.0
        and float(health.get("hs_cancelled_count_max", 1.0)) == 0.0
    )
    checks = {
        "run_completed": summary.get("ok") is True and summary.get("stop_reason") == "completed",
        "single_iteration": int(summary.get("iterations_completed", 0)) == 1,
        "split_weights_explicit_equal_1p05": split_weights_exact,
        "actor_saved_byte_identical": bool(per_actor) and all(per_actor.values()),
        "actor_runtime_freeze_exact": actor_runtime_exact,
        "logstd_saved_byte_identical": logstd_exact,
        "critic_digest_changed": bool(critic_before and critic_after and critic_before != critic_after),
        "mean_kl_zero": float(iteration.get("mean_kl_loss", 1.0)) == 0.0,
        "training_health_clean": health_ok,
    }
    return {
        "ok": all(checks.values()),
        "audit": "morphology_joint_split_critic_rewarm_v1",
        "checks": checks,
        "source_module": str(source_module),
        "run_dir": str(run_dir),
        "actor_per_key_byte_identical": per_actor,
        "logstd_rows_byte_identical": logstd_exact,
        "sigma": sigma,
        "critic_digest_before": critic_before,
        "critic_digest_after": critic_after,
        "training_health": health,
        "reward_weights": {
            "legacy": reward.get("morphology_weight"),
            "knee": reward.get("morphology_knee_weight"),
            "ankle": reward.get("morphology_ankle_weight"),
        },
        "iteration_metrics": {
            "sampled_steps": iteration.get("num_env_steps_sampled_lifetime"),
            "vf_loss": iteration.get("vf_loss"),
            "vf_explained_var": (iteration.get("learner_metrics") or {}).get(
                "learners/default_policy/vf_explained_var"
            ),
            "mean_kl_loss": iteration.get("mean_kl_loss"),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-module", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.source_module.resolve(), args.run_dir.resolve())
    text = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is not None:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
