"""DAgger correction for a target-domain imitation actor.

The validated prescribed teacher trajectory remains the reference. States
visited by one or more deterministic student rollouts are relabelled with the
teacher action at the same fixed-start episode step, then used together with
the original teacher dataset for another actor-only supervised update.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

import target_domain_imitation as imitation


THIS_DIR = Path(__file__).resolve().parent
TRAJ_GEN_DIR = THIS_DIR.parent
RUNS_ROOT = TRAJ_GEN_DIR / "runs" / "training"


def _path(value: str | Path, *, must_exist: bool = True) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    path = path.resolve()
    if must_exist and not path.exists():
        raise FileNotFoundError(path)
    return path


def _load_dataset(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as archive:
        return {name: np.asarray(archive[name]).copy() for name in archive.files}


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--teacher-run", required=True)
    parser.add_argument("--trace", action="append", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument("--trace-repeat", type=int, default=4)
    parser.add_argument("--interpolation-steps", type=int, default=0)
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--validation-fraction", type=float, default=0.20)
    parser.add_argument("--patience", type=int, default=60)
    parser.add_argument("--clip-weight", type=float, default=1.0)
    parser.add_argument("--logstd-weight", type=float, default=0.1)
    parser.add_argument("--anchor-weight", type=float, default=1e-5)
    parser.add_argument("--seed", type=int, default=123)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = _path(args.checkpoint)
    teacher_run = _path(args.teacher_run)
    trace_paths = [_path(value) for value in args.trace]
    output_dir = (
        _path(args.output_dir, must_exist=False)
        if args.output_dir
        else (
            RUNS_ROOT / f"target_domain_dagger_{datetime.now():%Y%m%d_%H%M%S}"
        ).resolve()
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    teacher_summary = _load_json(teacher_run / "teacher_summary.json")
    if not bool(teacher_summary.get("gate_pass")):
        raise ValueError("teacher run did not pass the full target-domain gate")
    teacher_dataset = _load_dataset(teacher_run / "teacher_dataset.npz")
    traces = [_load_json(path) for path in trace_paths]
    aggregate, dagger_summary = imitation.aggregate_dagger_traces(
        teacher_dataset,
        traces,
        trace_repeat=args.trace_repeat,
        interpolation_steps=args.interpolation_steps,
    )
    np.savez_compressed(output_dir / "adaptation_dataset.npz", **aggregate)
    (output_dir / "dagger_dataset_report.json").write_text(
        json.dumps(
            {
                **dagger_summary,
                "teacher_run": str(teacher_run),
                "trace_paths": [str(path) for path in trace_paths],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    resolved_config = teacher_run / "training_cfg.resolved.yaml"
    if not resolved_config.is_file():
        raise FileNotFoundError(resolved_config)
    shutil.copy2(resolved_config, output_dir / resolved_config.name)
    adaptation = imitation.adapt_actor(
        checkpoint,
        aggregate,
        output_dir,
        seed=args.seed,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        validation_fraction=args.validation_fraction,
        patience=args.patience,
        clip_weight=args.clip_weight,
        logstd_weight=args.logstd_weight,
        anchor_weight=args.anchor_weight,
    )
    summary = {
        "ok": True,
        "stage": "complete",
        "checkpoint": str(checkpoint),
        "teacher_run": str(teacher_run),
        "output_dir": str(output_dir),
        "dagger": dagger_summary,
        "adaptation": adaptation,
    }
    (output_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
