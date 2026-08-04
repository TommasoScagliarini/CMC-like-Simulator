"""Post-training cumulative policy-drift audit for retained PPO milestones.

The audit is deliberately offline: it consumes already-exported RLModules and
fixed development traces, and refuses runs whose final training summary does
not report successful completion. Held-out seeds 126-128 are forbidden.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Sequence

if __package__:
    from .compare_policy_checkpoints import _actor_digest, _load_state, compare
else:  # pragma: no cover - exercised by direct CLI use.
    from compare_policy_checkpoints import _actor_digest, _load_state, compare


_REPO_ROOT = Path(__file__).resolve().parents[1]
_MILESTONE_PATTERN = re.compile(r"^milestone_iteration_(\d{6})$")
_HELDOUT_SEED_PATTERN = re.compile(
    r"(?:^|[^0-9])seed[_-]?(126|127|128)(?:[^0-9]|$)",
    re.IGNORECASE,
)

DEFAULT_DEVELOPMENT_TRACE_SPECS = (
    {
        "condition": "deterministic_minus020",
        "action_selection": "deterministic",
        "episode_start_offset_s": 1.756870983805102,
        "action_seed": None,
        "path": _REPO_ROOT
        / "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_short_minus020/"
        "rollout_policy_trace.json",
    },
    {
        "condition": "deterministic_nominal",
        "action_selection": "deterministic",
        "episode_start_offset_s": 1.956870983805102,
        "action_seed": None,
        "path": _REPO_ROOT
        / "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_short_nominal/"
        "rollout_policy_trace.json",
    },
    {
        "condition": "deterministic_plus020",
        "action_selection": "deterministic",
        "episode_start_offset_s": 2.156870983805102,
        "action_seed": None,
        "path": _REPO_ROOT
        / "validation/controller_memory_ablation/"
        "2026-07-13_markov35_corrected_full_short_plus020/"
        "rollout_policy_trace.json",
    },
    {
        "condition": "stochastic_plus020_seed123",
        "action_selection": "stochastic",
        "episode_start_offset_s": 2.156870983805102,
        "action_seed": 123,
        "path": _REPO_ROOT
        / "validation/exploration_noise_runs/"
        "2026-07-14_h0_sigma0005_stochastic_plus020_seed123/"
        "rollout_policy_trace.json",
    },
)
DEFAULT_DEVELOPMENT_TRACES = tuple(
    spec["path"] for spec in DEFAULT_DEVELOPMENT_TRACE_SPECS
)
_DEFAULT_TRACE_SPECS_BY_PATH = {
    Path(spec["path"]).resolve(): spec for spec in DEFAULT_DEVELOPMENT_TRACE_SPECS
}


def _read_json_mapping(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read valid JSON mapping: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON mapping: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_recorded_checkpoint(value: Any) -> Path:
    text = str(value or "").replace("\\", "/")
    candidate = Path(text).expanduser()
    if candidate.is_absolute() and candidate.exists():
        return candidate.resolve()
    relative = _REPO_ROOT / candidate
    if relative.exists():
        return relative.resolve()
    for marker in ("validation/", "Trajectory Generator/"):
        if marker in text:
            mapped = _REPO_ROOT / (marker + text.split(marker, 1)[1])
            if mapped.exists():
                return mapped.resolve()
    raise ValueError(f"recorded trace checkpoint does not exist: {value!r}")


def _development_trace_descriptor(
    path: Path,
    reference_actor_digest: str,
) -> dict[str, Any]:
    summary_path, summary = _validated_trace_summary(path)
    descriptor: dict[str, Any] = {
        "path": str(path),
        "sha256": _sha256(path),
        "protocol_role": "explicit_development",
        "summary": str(summary_path),
        "action_selection": summary["action_selection"],
        "episode_start_offset_s": summary.get("episode_start_offset_s"),
        "action_seed": summary["action_seed"],
        "source_checkpoint_recorded": summary["checkpoint"],
    }
    spec = _DEFAULT_TRACE_SPECS_BY_PATH.get(path)
    if spec is None:
        return descriptor

    if summary.get("action_selection") != spec["action_selection"]:
        raise ValueError(
            f"default development trace action mode mismatch: {summary_path}"
        )
    try:
        recorded_offset = float(summary["episode_start_offset_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"default development trace has no valid start offset: {summary_path}"
        ) from exc
    if not math.isclose(
        recorded_offset,
        float(spec["episode_start_offset_s"]),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(f"default development trace start mismatch: {summary_path}")
    expected_seed = spec["action_seed"]
    if expected_seed is not None and summary.get("action_seed") != expected_seed:
        raise ValueError(f"default development trace seed mismatch: {summary_path}")

    source_checkpoint = _resolve_recorded_checkpoint(summary.get("checkpoint"))
    source_actor_digest = _actor_digest(_load_state(source_checkpoint))
    if source_actor_digest != reference_actor_digest:
        raise ValueError(
            "default development trace was not generated by an actor bit-exact "
            f"to canonical H0: {summary_path}"
        )
    descriptor.update(
        {
            "protocol_role": str(spec["condition"]),
            "action_selection": str(spec["action_selection"]),
            "episode_start_offset_s": recorded_offset,
            "action_seed": summary.get("action_seed"),
            "source_checkpoint": str(source_checkpoint),
            "source_actor_digest": source_actor_digest,
            "source_actor_bit_exact_to_reference_h0": True,
        }
    )
    return descriptor


def _validated_trace_summary(path: Path) -> tuple[Path, dict[str, Any]]:
    summary_path = path.parent / "rollout_summary.json"
    if not summary_path.is_file():
        raise ValueError(
            "development trace provenance is missing; expected adjacent "
            f"rollout_summary.json: {path}"
        )
    summary = _read_json_mapping(summary_path)
    if summary.get("ok") is not True:
        raise ValueError(f"development trace rollout is not successful: {summary_path}")
    seed = summary.get("action_seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError(
            f"development trace provenance has no integer action_seed: {summary_path}"
        )
    if seed in {126, 127, 128}:
        raise ValueError(
            "held-out seeds 126-128 are forbidden in policy-drift audit: "
            f"{summary_path} records action_seed={seed}"
        )
    if summary.get("action_selection") not in {"deterministic", "stochastic"}:
        raise ValueError(
            "development trace provenance has no valid action_selection: "
            f"{summary_path}"
        )
    checkpoint = summary.get("checkpoint")
    if not isinstance(checkpoint, str) or not checkpoint.strip():
        raise ValueError(
            f"development trace provenance has no checkpoint: {summary_path}"
        )
    return summary_path, summary


def _validated_development_traces(
    trace_paths: Sequence[str | Path],
) -> list[Path]:
    if not trace_paths:
        raise ValueError("at least one development trace is required")
    resolved: list[Path] = []
    seen: set[Path] = set()
    for value in trace_paths:
        path = Path(value).expanduser().resolve()
        if _HELDOUT_SEED_PATTERN.search(path.as_posix()):
            raise ValueError(
                "held-out seeds 126-128 are forbidden in policy-drift audit: "
                f"{path}"
            )
        if not path.is_file():
            raise ValueError(f"development trace does not exist: {path}")
        _validated_trace_summary(path)
        if path in seen:
            raise ValueError(f"duplicate development trace: {path}")
        seen.add(path)
        resolved.append(path)
    return resolved


def _completed_retained_milestones(run_dir: str | Path) -> list[dict[str, Any]]:
    run = Path(run_dir).expanduser().resolve()
    summary_path = run / "summary.json"
    summary = _read_json_mapping(summary_path)
    if summary.get("ok") is not True or summary.get("stop_reason") != "completed":
        raise ValueError(
            "policy-drift audit requires a successfully completed training run; "
            f"refusing an active or incomplete run: {summary_path}"
        )
    retention = summary.get("iteration_checkpoint_retention")
    if not isinstance(retention, dict) or retention.get("enabled") is not True:
        raise ValueError("training summary does not enable iteration retention")
    registered = retention.get("milestones")
    if not isinstance(registered, list) or not registered:
        raise ValueError("training summary contains no registered milestones")

    registered_names: list[str] = []
    for value in registered:
        name = Path(str(value)).name
        if _MILESTONE_PATTERN.fullmatch(name) is None:
            raise ValueError(f"invalid registered milestone name: {value!r}")
        registered_names.append(name)
    if len(registered_names) != len(set(registered_names)):
        raise ValueError("training summary registers duplicate milestones")

    discovered_names = sorted(
        path.name
        for path in run.iterdir()
        if path.is_dir() and _MILESTONE_PATTERN.fullmatch(path.name)
    )
    if sorted(registered_names) != discovered_names:
        raise ValueError(
            "registered and published milestone sets differ: "
            f"registered={sorted(registered_names)}, discovered={discovered_names}"
        )

    milestones: list[dict[str, Any]] = []
    for name in sorted(registered_names):
        match = _MILESTONE_PATTERN.fullmatch(name)
        assert match is not None
        logical_iteration = int(match.group(1))
        root = run / name
        module = root / "rl_module_last"
        state_path = module / "module_state.pkl"
        meta_path = root / "rl_module_last_meta.json"
        if not state_path.is_file():
            raise ValueError(f"milestone RLModule state is missing: {state_path}")
        meta = _read_json_mapping(meta_path)
        if meta.get("logical_iteration") != logical_iteration:
            raise ValueError(
                f"milestone metadata iteration mismatch: {meta_path}"
            )
        milestones.append(
            {
                "logical_iteration": logical_iteration,
                "milestone": root,
                "rl_module": module,
                "metadata": meta_path,
            }
        )
    return milestones


def _finite_drift_metrics(aggregate: dict[str, Any]) -> bool:
    names = (
        "mean_delta_rmse",
        "mean_delta_abs_max",
        "empirical_kl_reference_to_candidate_mean",
        "empirical_kl_reference_to_candidate_max",
    )
    try:
        return all(math.isfinite(float(aggregate[name])) for name in names)
    except (KeyError, TypeError, ValueError):
        return False


def audit_policy_milestones(
    reference_path: str | Path,
    run_dir: str | Path,
    trace_paths: Sequence[str | Path] = DEFAULT_DEVELOPMENT_TRACES,
) -> dict[str, Any]:
    """Compare every registered retained milestone cumulatively against H0."""
    reference = Path(reference_path).expanduser().resolve()
    run = Path(run_dir).expanduser().resolve()
    traces = _validated_development_traces(trace_paths)
    milestones = _completed_retained_milestones(run)
    reference_actor_digest = _actor_digest(_load_state(reference))
    trace_descriptors = [
        _development_trace_descriptor(path, reference_actor_digest)
        for path in traces
    ]

    rows: list[dict[str, Any]] = []
    for milestone in milestones:
        comparison = compare(
            reference,
            milestone["rl_module"],
            traces,
        )
        aggregate = comparison["fixed_observation_aggregate"]
        per_trace = comparison["fixed_observation_per_trace"]
        logstd_head_exact = bool(
            comparison["logstd_head_comparison"]["bit_exact"]
        )
        logstd_trace_exact = bool(
            aggregate["logstd_bit_exact"]
            and all(item["logstd_bit_exact"] for item in per_trace)
        )
        metrics_finite = _finite_drift_metrics(aggregate)
        logstd_bit_exact = logstd_head_exact and logstd_trace_exact
        status = "PASS" if logstd_bit_exact and metrics_finite else "FAIL"
        rows.append(
            {
                "logical_iteration": milestone["logical_iteration"],
                "milestone": str(milestone["milestone"]),
                "rl_module": str(milestone["rl_module"]),
                "status": status,
                "logstd_bit_exact": logstd_bit_exact,
                "logstd_head_bit_exact": logstd_head_exact,
                "logstd_trace_bit_exact": logstd_trace_exact,
                "action_mean_rmse": aggregate["mean_delta_rmse"],
                "action_mean_abs_max": aggregate["mean_delta_abs_max"],
                "kl_reference_to_candidate_mean": aggregate[
                    "empirical_kl_reference_to_candidate_mean"
                ],
                "kl_reference_to_candidate_max": aggregate[
                    "empirical_kl_reference_to_candidate_max"
                ],
                "metrics_finite": metrics_finite,
                "reference_actor_digest": comparison["reference_actor_digest"],
                "candidate_actor_digest": comparison["candidate_actor_digest"],
                "parameter_comparison": comparison["parameter_comparison"],
                "logstd_head_comparison": comparison[
                    "logstd_head_comparison"
                ],
                "per_development_trace": per_trace,
            }
        )

    failed_iterations = [
        row["logical_iteration"] for row in rows if row["status"] != "PASS"
    ]
    return {
        "schema_version": 1,
        "audit": "cumulative_policy_drift_from_h0",
        "ok": not failed_iterations,
        "reference_h0_rl_module": str(reference),
        "reference_h0_actor_digest": reference_actor_digest,
        "training_run": str(run),
        "heldout_seeds_excluded": [126, 127, 128],
        "development_traces": trace_descriptors,
        "milestone_count": len(rows),
        "failed_logical_iterations": failed_iterations,
        "milestones": rows,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-h0", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument(
        "--trace",
        "--development-trace",
        dest="development_trace",
        action="append",
        default=None,
        help=(
            "Fixed development rollout_policy_trace.json; repeat as needed. "
            "Defaults to the preregistered H0 deterministic three-start and "
            "stochastic +0.20 seed 123 traces. Seeds 124-125 are opt-in only; "
            "seeds 126-128 are forbidden. Every explicit trace requires an "
            "adjacent rollout_summary.json with complete provenance."
        ),
    )
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run = Path(args.run_dir).expanduser().resolve()
    traces = (
        args.development_trace
        if args.development_trace is not None
        else DEFAULT_DEVELOPMENT_TRACES
    )
    result = audit_policy_milestones(args.reference_h0, run, traces)
    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else run / "policy_drift_from_h0_milestones.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
