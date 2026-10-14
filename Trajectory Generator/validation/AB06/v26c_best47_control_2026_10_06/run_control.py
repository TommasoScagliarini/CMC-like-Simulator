"""Delegate the current baseline evaluator and record read-only runtime metadata.

The sole environment-factory observer calls the original factory unchanged.
No action, observation, reset, reference or simulator operation is replaced.
The ordinary evaluator CLI and process watchdog remain in use.
"""

from __future__ import annotations

import importlib.util
import json
import platform
import sys
import traceback
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path


REPO = Path(__file__).resolve().parents[4]
AUDIT = Path(__file__).resolve().parent
EVALUATOR = REPO / "Trajectory Generator/baseline_MLP/rollout_eval.py"


def write_json(path: Path, data: dict) -> None:
    path.write_text(
        json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    spec = importlib.util.spec_from_file_location("ab06_control_evaluator", EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import evaluator: {EVALUATOR}")
    evaluator = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = evaluator
    spec.loader.exec_module(evaluator)
    args = evaluator.parse_args()
    output = evaluator._resolve_output_dir(args.output_dir, "ab06_control")
    if not args.worker_process:
        output.mkdir(parents=True, exist_ok=True)
        child_args = [value for value in sys.argv[1:] if value != "--worker-process"]
        child_args.extend(["--output-dir", str(output), "--worker-process"])
        result = evaluator.process_watchdog.supervise_process(
            [sys.executable, str(Path(__file__).resolve()), *child_args],
            heartbeat_file=output / evaluator._WATCHDOG_FILENAME,
            summary_file=output / "watchdog_summary.json",
            startup_timeout_s=args.startup_timeout_s,
            stall_timeout_s=args.stall_timeout_s,
            run_timeout_s=args.run_timeout_s,
            label="AB06 V26C best47 control",
        )
        if result["timeout_reason"] is not None:
            return 124 if result["timeout_reason"] != "user_interrupt" else 130
        return int(result["returncode"])

    manifest = json.loads((AUDIT / "execution_manifest.json").read_text())
    receipt = {
        "schema_version": 1,
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "started",
        "scope": "Single deterministic AB06 control with unchanged V26C best47 checkpoint",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "capture_mechanism": "Read-only observer of original env_factory.make_cmc_env; evaluator.run and all numerical operations unchanged",
        "source_fingerprints": manifest["source_fingerprints"],
        "checkpoint": str(evaluator._resolve_input_path(args.checkpoint)),
        "checkpoint_logical_iteration": 47,
        "output_dir": str(output),
        "requested_duration_s": float(args.episode_duration),
        "seed": int(args.seed),
        "action_selection": args.action_selection,
        "requested_cli": {key: value for key, value in vars(args).items() if key != "_cfg_reward"},
        "runtime": {"python": sys.version, "executable": sys.executable, "platform": platform.platform()},
    }
    captured = []
    original_loader = evaluator._load_inference_stack
    original_factory = None

    def observing_loader() -> None:
        nonlocal original_factory
        original_loader()
        if original_factory is not None:
            return
        original_factory = evaluator.env_factory.make_cmc_env

        def observing_factory(env_config=None):
            env = original_factory(env_config)
            captured.append(env)
            base_env = env.unwrapped
            receipt["requested_env_config"] = evaluator._jsonable(dict(env_config or {}))
            receipt["effective_env_config"] = evaluator._jsonable(asdict(base_env.env_cfg))
            receipt["effective_simulator_config"] = evaluator._jsonable(asdict(base_env.cfg))
            receipt["actual_episode_start_s"] = float(base_env._episode_start)
            receipt["actual_episode_end_s"] = float(base_env._episode_end)
            receipt["actor_feature_names"] = list(base_env.actor_feature_names)
            receipt["observation_feature_names"] = list(base_env.observation_feature_names)
            historical = json.loads(Path(manifest["historical_summary"]).read_text())
            receipt["feature_order_matches_historical"] = all(
                receipt[key] == historical[key]
                for key in ("actor_feature_names", "observation_feature_names")
            )
            write_json(output / "receipt.json", receipt)
            if not receipt["feature_order_matches_historical"]:
                raise RuntimeError("Feature names/order differ from the best47 historical rollout")
            return env

        evaluator.env_factory.make_cmc_env = observing_factory

    evaluator._load_inference_stack = observing_loader
    try:
        summary = evaluator.run(args)
        if len(captured) != 1:
            raise RuntimeError(f"Expected one environment, observed {len(captured)}")
        base_env = captured[0].unwrapped
        receipt["final_time_s"] = float(base_env.t)
        receipt["actual_episode_start_s"] = float(base_env._episode_start)
        receipt["actual_episode_end_s"] = float(base_env._episode_end)
        receipt["effective_env_config"] = evaluator._jsonable(asdict(base_env.env_cfg))
        receipt["effective_simulator_config"] = evaluator._jsonable(asdict(base_env.cfg))
        receipt["steps"] = int(summary["steps"])
        receipt["end_reason"] = summary["end_reason"]
        receipt["terminated"] = bool(summary["terminated"])
        receipt["truncated"] = bool(summary["truncated"])
        receipt["status"] = "completed"
        receipt["completed_requested_duration"] = bool(
            summary["end_reason"] == "episode_time_limit"
            and not summary["terminated"]
            and abs(base_env.t - base_env._episode_end) <= 1e-8
        )
        receipt["runtime"].update({"numpy": evaluator.np.__version__, "torch": evaluator.torch.__version__})
        write_json(output / "receipt.json", receipt)
        write_json(AUDIT / "runtime_receipt.json", receipt)
        print(json.dumps({"status": receipt["status"], "steps": receipt["steps"], "end_reason": receipt["end_reason"], "completed_requested_duration": receipt["completed_requested_duration"], "summary": str(output / "rollout_summary.json")}))
        return 0
    except Exception as exc:
        receipt["status"] = "failed"
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        write_json(output / "receipt.json", receipt)
        write_json(AUDIT / "runtime_receipt.json", receipt)
        traceback.print_exc()
        return 1
    finally:
        evaluator._load_inference_stack = original_loader
        if original_factory is not None:
            evaluator.env_factory.make_cmc_env = original_factory


if __name__ == "__main__":
    raise SystemExit(main())
