"""Consolidate the final warm-start evidence without running PPO H1."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

import yaml


ROOT_DIR = Path(__file__).resolve().parents[1]
BASELINE_DIR = ROOT_DIR / "Trajectory Generator" / "baseline_MLP"
for path in (ROOT_DIR, BASELINE_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def _read_object(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    value = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"expected a JSON object: {resolved}")
    return dict(value)


def _gradient_audit(module_path: Path) -> dict[str, Any]:
    import torch
    from ray.rllib.core.columns import Columns
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(module_path)
    if not bool(getattr(module, "_freeze_logstd", False)):
        return {"pass": False, "reason": "reloaded module has freeze_logstd=false"}
    action_dim = int(module._action_dim)
    n_full = int(module._n_full)
    output_layer = module.pi[-1]
    module.pi.zero_grad(set_to_none=True)
    logits = module._policy_logits(
        {Columns.OBS: torch.zeros((4, n_full), dtype=torch.float32)}
    )
    logits.sum().backward()
    weight_gradient = output_layer.weight.grad
    bias_gradient = output_layer.bias.grad
    if weight_gradient is None or bias_gradient is None:
        return {"pass": False, "reason": "actor output gradients are missing"}
    mean_gradient_abs_max = float(
        max(
            torch.max(torch.abs(weight_gradient[:action_dim])).item(),
            torch.max(torch.abs(bias_gradient[:action_dim])).item(),
        )
    )
    logstd_gradient_abs_max = float(
        max(
            torch.max(torch.abs(weight_gradient[action_dim:])).item(),
            torch.max(torch.abs(bias_gradient[action_dim:])).item(),
        )
    )
    return {
        "pass": mean_gradient_abs_max > 0.0 and logstd_gradient_abs_max == 0.0,
        "freeze_logstd": True,
        "mean_gradient_abs_max": mean_gradient_abs_max,
        "logstd_gradient_abs_max": logstd_gradient_abs_max,
        "logstd_output_weight_abs_max": float(
            torch.max(torch.abs(output_layer.weight[action_dim:])).item()
        ),
        "logstd_output_bias": output_layer.bias[action_dim:]
        .detach()
        .cpu()
        .numpy()
        .astype(float)
        .tolist(),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    gate = _read_object(args.gate)
    configuration = _read_object(args.configuration_report)
    trainer = _read_object(args.trainer_summary)
    transplant = _read_object(args.transplant_report)
    integration = transplant.get("integration_validation", {})
    candidate_digest = configuration.get("configured_actor_digest")
    resolved_config_path = Path(args.resolved_config).expanduser().resolve()
    resolved_config = yaml.safe_load(resolved_config_path.read_text(encoding="utf-8"))
    gradient = _gradient_audit(Path(args.module).expanduser().resolve())

    checks = {
        "relative_gate_pass": gate.get("status") == "PASS",
        "candidate_mean_exact": (
            bool(configuration.get("configuration", {}).get("mean_parameters_exact"))
            and float(
                configuration.get("trace_validation", {}).get(
                    "mean_max_abs_diff", float("nan")
                )
            )
            == 0.0
            and bool(configuration.get("save_reload", {}).get("exact"))
        ),
        "sigma_is_0_003": configuration.get("configuration", {}).get("sigma")
        == [0.003, 0.003],
        "trainer_completed_without_h1": (
            bool(trainer.get("ok"))
            and int(trainer.get("iterations_run", -1)) == 0
            and int(trainer.get("iterations_completed_this_process", -1)) == 0
            and trainer.get("history") == []
        ),
        "warm_start_applied": bool(trainer.get("warm_start_applied")),
        "actor_digest_exact": (
            transplant.get("source_actor_digest") == candidate_digest
            and transplant.get("target_actor_digest_after") == candidate_digest
            and bool(integration.get("learner_actor", {}).get("exact"))
            and bool(integration.get("saved_initial_actor", {}).get("exact"))
            and bool(integration.get("env_runner_actors_exact"))
        ),
        "critic_unchanged": (
            bool(transplant.get("target_non_actor_state_unchanged"))
            and bool(integration.get("learner_non_actor", {}).get("exact"))
        ),
        "optimizer_fresh": integration.get("optimizer_source_loaded") is False,
        "weights_synced_before_sampling": bool(
            integration.get("weights_synced_before_first_sample")
        ),
        "resolved_freeze_logstd": (
            resolved_config.get("model", {}).get("freeze_logstd") is True
        ),
        "runtime_gradient_freeze": bool(gradient.get("pass")),
    }
    report = {
        "status": "READY_FOR_H1" if all(checks.values()) else "NOT_READY",
        "ready_for_h1": all(checks.values()),
        "h1_executed": False,
        "checks": checks,
        "candidate_actor_digest": candidate_digest,
        "exploration_sigma": configuration.get("configuration", {}).get("sigma"),
        "gate": {
            "median_steps": gate.get("median_steps"),
            "valid_to_count": gate.get("valid_to_count"),
            "valid_cycle_count": gate.get("valid_cycle_count"),
            "median_action_clipped_fraction": gate.get(
                "median_action_clipped_fraction"
            ),
        },
        "trainer": {
            "iterations_run": trainer.get("iterations_run"),
            "num_env_runners": trainer.get("num_env_runners"),
            "env_runner_count_checked": integration.get(
                "env_runner_count_checked"
            ),
        },
        "gradient_audit": gradient,
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not report["ready_for_h1"]:
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"H1 readiness checks failed: {failed}")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", required=True)
    parser.add_argument("--configuration-report", required=True)
    parser.add_argument("--trainer-summary", required=True)
    parser.add_argument("--transplant-report", required=True)
    parser.add_argument("--resolved-config", required=True)
    parser.add_argument("--module", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
