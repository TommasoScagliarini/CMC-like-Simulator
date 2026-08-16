#!/usr/bin/env python3
"""CLI for the guarded V12R11 direct source-H0-on-V26 +0.20 probe."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

import probe  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--describe",
        action="store_true",
        help="Print the frozen, execution-free protocol description.",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="Execute the irreversible one-shot physical rollout.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.describe:
        result = probe.describe_protocol()
    else:
        executed = probe.run_probe()
        result = {
            "destination": executed["destination"]
            .relative_to(probe.REPO_ROOT)
            .as_posix(),
            "summary": executed["summary"],
            "gate": executed["gate"],
            "gate_record": executed["gate_record"],
            "closure_receipt": executed["closure_verification"]["closure_receipt"],
        }
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result.get("gate", {}).get("passed", True) else 2


if __name__ == "__main__":
    raise SystemExit(main())
