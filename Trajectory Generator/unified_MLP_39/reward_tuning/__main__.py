"""Command-line entrypoint; only 'run' launches training or full evaluations."""
from __future__ import annotations

import argparse
import json

from .io import load_config, resolve


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("analyze", "prepare", "run", "compare"))
    parser.add_argument("--config", required=True, help="Tuning YAML; paths inside are relative to repository root")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    output = resolve(config["output_dir"])
    if args.operation == "analyze":
        from .analysis import analyze
        result = analyze(config, output / "analysis")
        print(json.dumps({"status": "analyzed_offline", "datasets": len(result["datasets"]),
                          "candidates": len(result["candidates"]), "report": str(output / "analysis/report.md")}, indent=2))
    elif args.operation == "prepare":
        from .pilot import prepare
        analysis_path = resolve(config["analysis_file"]) if config.get("analysis_file") else output / "analysis/analysis.json"
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        config["_analysis_path"] = str(analysis_path)
        result = prepare(config, analysis, output / "pilot")
        print(json.dumps({"status": result["status"], "manifest": str(output / "pilot/pilot_manifest.json"),
                          "training_started": False}, indent=2))
    elif args.operation == "run":
        from .pilot import run
        from .compare import compare
        from .report import build_report
        state = run(output / "pilot")
        report = compare(config, output)
        previous = resolve(config["descriptive_comparison_file"]) if config.get("descriptive_comparison_file") else None
        artifacts = build_report(config, output, previous_comparison=previous)
        print(json.dumps({"run_status": state["status"], "comparison_status": report["status"],
                          "report_artifacts": artifacts}, indent=2))
    else:
        from .compare import compare
        from .report import build_report
        result = compare(config, output)
        previous = resolve(config["descriptive_comparison_file"]) if config.get("descriptive_comparison_file") else None
        artifacts = build_report(config, output, previous_comparison=previous)
        print(json.dumps({"status": result["status"], "missing_cases": result["missing_cases"],
                          "report_artifacts": artifacts}, indent=2))


if __name__ == "__main__":
    main()
