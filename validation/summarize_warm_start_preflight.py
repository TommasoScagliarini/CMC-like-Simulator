"""Consolidate the selected-actor warm-start preflight gates."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from statistics import median
from typing import Any, Mapping, Sequence

import numpy as np


def _read_json(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    payload = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"expected JSON object: {resolved}")
    return dict(payload)


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _exploration_statistics(module_path: Path, trace_path: Path) -> dict[str, Any]:
    with module_path.open("rb") as handle:
        state = pickle.load(handle)
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    observations = np.asarray(
        [row["actor_observation_vector_before"] for row in rows],
        dtype=np.float32,
    )
    recorded_actions = np.asarray(
        [row["raw_policy_action"] for row in rows],
        dtype=float,
    )
    hidden = np.tanh(
        observations @ _array(state["pi.0.0.weight"]).T
        + _array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ _array(state["pi.0.2.weight"]).T
        + _array(state["pi.0.2.bias"])
    )
    logits = (
        hidden @ _array(state["pi.1.weight"]).T
        + _array(state["pi.1.bias"])
    )
    action_dim = recorded_actions.shape[1]
    means = logits[:, :action_dim]
    logstd = logits[:, action_dim:]
    std = np.exp(logstd)
    return {
        "samples": int(len(observations)),
        "mean_vs_recorded_action_max_abs_diff": float(
            np.max(np.abs(means - recorded_actions))
        ),
        "logstd_min": np.min(logstd, axis=0).astype(float).tolist(),
        "logstd_median": np.median(logstd, axis=0).astype(float).tolist(),
        "logstd_max": np.max(logstd, axis=0).astype(float).tolist(),
        "std_min": np.min(std, axis=0).astype(float).tolist(),
        "std_median": np.median(std, axis=0).astype(float).tolist(),
        "std_max": np.max(std, axis=0).astype(float).tolist(),
    }


def classify_stochastic_preflight(
    summaries: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    if len(summaries) < 3:
        raise ValueError("at least three stochastic summaries are required")
    technical_pass = all(
        bool(item.get("ok")) and item.get("action_selection") == "stochastic"
        for item in summaries
    )
    reached_to = [int(item.get("phase_valid_to_count", 0) or 0) >= 1 for item in summaries]
    reached_cycle = [
        int(item.get("phase_valid_cycle_count", 0) or 0) >= 1
        for item in summaries
    ]
    penetration_terminations = [
        item.get("end_reason") == "grf_penetration" for item in summaries
    ]
    clipping = [float(item.get("action_clipped_fraction", 0.0)) for item in summaries]
    useful_fraction = float(sum(reached_to) / len(summaries))
    penetration_fraction = float(sum(penetration_terminations) / len(summaries))
    median_clipping = float(median(clipping))
    behavioral_pass = (
        useful_fraction >= 2.0 / 3.0
        and penetration_fraction < 2.0 / 3.0
        and median_clipping <= 0.10
    )
    reasons = []
    if useful_fraction < 2.0 / 3.0:
        reasons.append("fewer than two thirds of probes reached a valid TO")
    if penetration_fraction >= 2.0 / 3.0:
        reasons.append("at least two thirds terminated on GRF penetration")
    if median_clipping > 0.10:
        reasons.append("median raw-action clipping exceeded 10%")
    return {
        "status": "PASS" if technical_pass and behavioral_pass else "FAIL",
        "technical_pass": technical_pass,
        "behavioral_pass": behavioral_pass,
        "probe_count": len(summaries),
        "valid_to_fraction": useful_fraction,
        "valid_cycle_fraction": float(sum(reached_cycle) / len(summaries)),
        "penetration_termination_fraction": penetration_fraction,
        "median_action_clipped_fraction": median_clipping,
        "failure_reasons": reasons,
        "probes": [
            {
                key: item.get(key)
                for key in (
                    "action_seed",
                    "steps",
                    "episode_return",
                    "action_abs_max",
                    "action_clipped_fraction",
                    "grf_penetration_max_m",
                    "phase_valid_hs_count",
                    "phase_valid_to_count",
                    "phase_valid_cycle_count",
                    "invalid_event_count",
                    "end_reason",
                )
            }
            for item in summaries
        ],
    }


def classify_relative_h1_gate(
    summaries: Sequence[Mapping[str, Any]],
    *,
    deterministic_steps: int,
    deterministic_unchanged: bool,
    minimum_survival_fraction: float = 0.60,
    maximum_median_clipping: float = 0.01,
) -> dict[str, Any]:
    """Classify small-noise probes relative to the deterministic baseline."""
    if len(summaries) < 3:
        raise ValueError("at least three stochastic summaries are required")
    if deterministic_steps <= 0:
        raise ValueError("deterministic_steps must be > 0")

    steps = [int(item.get("steps", 0) or 0) for item in summaries]
    clipping = [float(item.get("action_clipped_fraction", np.nan)) for item in summaries]
    returns = [float(item.get("episode_return", np.nan)) for item in summaries]
    penetrations = [
        float(item.get("grf_penetration_max_m", np.nan)) for item in summaries
    ]
    reached_to = [int(item.get("phase_valid_to_count", 0) or 0) >= 1 for item in summaries]
    reached_cycle = [
        int(item.get("phase_valid_cycle_count", 0) or 0) >= 1
        for item in summaries
    ]
    timeout_free = [
        "timeout" not in str(item.get("end_reason", "")).lower()
        for item in summaries
    ]
    finite = all(
        np.isfinite(value)
        for values in (clipping, returns, penetrations)
        for value in values
    )
    technical_pass = all(
        bool(item.get("ok")) and item.get("action_selection") == "stochastic"
        for item in summaries
    ) and finite and all(timeout_free)
    median_steps = float(median(steps))
    required_steps = float(minimum_survival_fraction * deterministic_steps)
    median_clipping = float(median(clipping))

    checks = {
        "technical": technical_pass,
        "deterministic_mean_unchanged": bool(deterministic_unchanged),
        "all_reach_valid_to": all(reached_to),
        "at_least_one_valid_cycle": any(reached_cycle),
        "median_survival": median_steps >= required_steps,
        "median_clipping": median_clipping < maximum_median_clipping,
    }
    reasons = []
    if not checks["technical"]:
        reasons.append("a probe failed technical, finite-value, or timeout checks")
    if not checks["deterministic_mean_unchanged"]:
        reasons.append("the configured actor did not preserve the deterministic mean")
    if not checks["all_reach_valid_to"]:
        reasons.append("not all probes reached a valid TO")
    if not checks["at_least_one_valid_cycle"]:
        reasons.append("no probe completed a valid HS-to-HS cycle")
    if not checks["median_survival"]:
        reasons.append(
            f"median survival {median_steps:.0f} was below {required_steps:.1f} steps"
        )
    if not checks["median_clipping"]:
        reasons.append(
            f"median clipping {median_clipping:.6f} was not below "
            f"{maximum_median_clipping:.6f}"
        )
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "probe_count": len(summaries),
        "deterministic_steps": int(deterministic_steps),
        "minimum_survival_fraction": float(minimum_survival_fraction),
        "required_median_steps": required_steps,
        "median_steps": median_steps,
        "median_survival_fraction": median_steps / deterministic_steps,
        "maximum_median_clipping": float(maximum_median_clipping),
        "median_action_clipped_fraction": median_clipping,
        "valid_to_count": int(sum(reached_to)),
        "valid_cycle_count": int(sum(reached_cycle)),
        "failure_reasons": reasons,
        "probes": [
            {
                key: item.get(key)
                for key in (
                    "action_seed",
                    "steps",
                    "episode_return",
                    "action_clipped_fraction",
                    "grf_penetration_max_m",
                    "phase_valid_hs_count",
                    "phase_valid_to_count",
                    "phase_valid_cycle_count",
                    "invalid_event_count",
                    "end_reason",
                )
            }
            for item in summaries
        ],
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    freeze = _read_json(args.freeze_manifest)
    trainer = _read_json(args.trainer_summary)
    port = _read_json(args.port_audit)
    deterministic = _read_json(args.deterministic_summary)
    stochastic = [_read_json(path) for path in args.stochastic_summary]
    integration = trainer.get("warm_start", {}).get("integration_validation", {})

    gate1_pass = bool(freeze.get("ok")) and bool(
        freeze.get("source_to_frozen_actor", {}).get("exact")
    )
    actor_digest = freeze.get("actor_digest")
    gate2_pass = (
        bool(trainer.get("ok"))
        and int(trainer.get("iterations_run", -1)) == 0
        and bool(trainer.get("warm_start_applied"))
        and trainer.get("warm_start", {}).get("source_actor_digest") == actor_digest
        and trainer.get("warm_start", {}).get("target_actor_digest_after")
        == actor_digest
        and bool(integration.get("learner_actor", {}).get("exact"))
        and bool(integration.get("learner_non_actor", {}).get("exact"))
        and bool(integration.get("env_runner_actors_exact"))
        and bool(integration.get("saved_initial_actor", {}).get("exact"))
        and integration.get("optimizer_source_loaded") is False
        and bool(port.get("port_validated"))
    )
    deterministic_terms = json.loads(
        Path(args.deterministic_trace).read_text(encoding="utf-8")
    )[-1]["reward_terms"]
    deterministic_evidence = {
        "steps": deterministic.get("steps"),
        "episode_return": deterministic.get("episode_return"),
        "valid_hs_count": deterministic_terms.get("phase_valid_hs_count"),
        "valid_to_count": deterministic_terms.get("phase_valid_to_count"),
        "valid_cycle_count": deterministic_terms.get("phase_valid_cycle_count"),
        "max_penetration_m": deterministic_terms.get("grf_penetration_m"),
        "action_clipped_fraction": deterministic.get("action_clipped_fraction"),
        "fresh_replay_attempts": [
            _read_json(path) for path in args.fresh_replay_watchdog
        ],
        "offline_target_equivalence": port.get("real_trace_equivalence"),
    }
    exploration = _exploration_statistics(
        Path(args.target_module_state).expanduser().resolve(),
        Path(args.deterministic_trace).expanduser().resolve(),
    )
    gate3 = classify_stochastic_preflight(stochastic)
    ready = gate1_pass and gate2_pass and gate3["status"] == "PASS"
    report = {
        "status": "PASS" if ready else "STOP_BEFORE_H1",
        "ready_for_h1": ready,
        "selected_actor_digest": actor_digest,
        "gate_1_freeze": {"status": "PASS" if gate1_pass else "FAIL"},
        "gate_2_zero_iteration_transplant": {
            "status": "PASS" if gate2_pass else "FAIL",
            "iterations_run": trainer.get("iterations_run"),
            "env_runner_count_checked": integration.get("env_runner_count_checked"),
            "critic_exact": integration.get("learner_non_actor", {}).get("exact"),
            "optimizer_source_loaded": integration.get("optimizer_source_loaded"),
            "port_audit_status": port.get("status"),
        },
        "deterministic_actor_evidence": deterministic_evidence,
        "exploration_distribution_on_deterministic_trace": exploration,
        "gate_3_stochastic_preflight": gate3,
        "decision": (
            "Proceed to H1."
            if ready
            else "Do not start H1 with the inherited exploration variance. "
            "Preserve the validated action mean, reduce or reinitialize the "
            "Gaussian log-standard-deviation output, then repeat gate 3."
        ),
    }
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "warm_start_preflight_summary.json"
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    lines = [
        "# Warm-start Preflight Summary",
        "",
        f"- Overall: **{report['status']}**",
        f"- Gate 1 freeze: **{report['gate_1_freeze']['status']}**",
        f"- Gate 2 zero-iteration transplant: **{report['gate_2_zero_iteration_transplant']['status']}**",
        f"- Gate 3 stochastic preflight: **{gate3['status']}**",
        "",
        "## Decision",
        "",
        report["decision"],
        "",
    ]
    (output_dir / "warm_start_preflight_summary.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze-manifest", required=True)
    parser.add_argument("--trainer-summary", required=True)
    parser.add_argument("--port-audit", required=True)
    parser.add_argument("--target-module-state", required=True)
    parser.add_argument("--deterministic-summary", required=True)
    parser.add_argument("--deterministic-trace", required=True)
    parser.add_argument("--fresh-replay-watchdog", nargs="*", default=[])
    parser.add_argument("--stochastic-summary", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
