"""Compare a saved onlineGRF rollout output with prescribed ExternalLoads."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import _read_storage_table
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import _external_grf, _metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--online-grf", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--threshold", type=float, default=20.0)
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Comparison requires prescribed ExternalLoads in the setup.")
    times, columns, data = _read_storage_table(str(resolve_repo_path(args.online_grf)))
    indices = {name: index for index, name in enumerate(columns)}
    predicted = {}
    for side in ("left", "right"):
        predicted[side] = np.column_stack(
            [
                data[:, indices[f"{side}_force_{axis}"]]
                for axis in ("x", "y", "z")
            ]
        )
    reference = _external_grf(setup, times)
    mask = np.ones(len(times), dtype=bool)
    report = {
        "setup": str(Path(args.setup).resolve()),
        "online_grf": str(resolve_repo_path(args.online_grf).resolve()),
        "samples": int(len(times)),
        "time_range": [float(times[0]), float(times[-1])],
        "metrics": _metrics(reference, predicted, times, mask, args.threshold),
    }
    destination = resolve_repo_path(args.report)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
