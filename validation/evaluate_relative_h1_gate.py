"""Evaluate the relative stochastic gate before the first warm-start PPO update."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from validation.summarize_warm_start_preflight import classify_relative_h1_gate


def _read_object(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    value = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"expected a JSON object: {resolved}")
    return dict(value)


def run(args: argparse.Namespace) -> dict[str, Any]:
    deterministic = _read_object(args.deterministic_summary)
    configuration = _read_object(args.configuration_report)
    stochastic = [_read_object(path) for path in args.stochastic_summary]
    deterministic_unchanged = (
        bool(configuration.get("ok"))
        and bool(configuration.get("configuration", {}).get("mean_parameters_exact"))
        and bool(configuration.get("save_reload", {}).get("exact"))
        and float(
            configuration.get("trace_validation", {}).get(
                "mean_max_abs_diff", float("nan")
            )
        )
        == 0.0
    )
    report = classify_relative_h1_gate(
        stochastic,
        deterministic_steps=int(deterministic["steps"]),
        deterministic_unchanged=deterministic_unchanged,
    )
    report["configuration_report"] = str(
        Path(args.configuration_report).expanduser().resolve()
    )
    report["deterministic_summary"] = str(
        Path(args.deterministic_summary).expanduser().resolve()
    )
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deterministic-summary", required=True)
    parser.add_argument("--configuration-report", required=True)
    parser.add_argument("--stochastic-summary", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
