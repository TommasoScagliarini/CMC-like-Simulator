"""CLI for the no-clobber V12R15 masked safe-teacher actor builder."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r15_masked_teacher_fitter as fitter  # noqa: E402


EXECUTION_ACKNOWLEDGEMENT = "V12R15_ONE_SHOT_FIXED_FIT"
GOVERNANCE_ACKNOWLEDGEMENT = "V12R15_FREEZE_REVIEWED_SOURCES"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect, preflight, or execute the one-shot V12R15 invariant-18D "
            "safe-teacher W256 fit."
        )
    )
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument(
        "--describe", action="store_true", help="print the frozen design; no I/O"
    )
    operation.add_argument(
        "--preflight",
        action="store_true",
        help="hash and validate all inputs without fitting or writing",
    )
    operation.add_argument(
        "--execute",
        action="store_true",
        help="irreversibly claim the canonical output and execute the sole fit",
    )
    operation.add_argument(
        "--write-protocol-freeze",
        action="store_true",
        help="exclusively freeze the reviewed builder, runner, tests, and design",
    )
    operation.add_argument(
        "--write-execution-lock",
        action="store_true",
        help="exclusively lock the already frozen sources for P0/P2 execution",
    )
    parser.add_argument(
        "--stage",
        choices=("p0", "p2"),
        default="p0",
        help="fit stage for --describe, --preflight, or --execute",
    )
    parser.add_argument(
        "--acknowledge-one-shot",
        default=None,
        help=(f"required with --execute; exact value: {EXECUTION_ACKNOWLEDGEMENT}"),
    )
    parser.add_argument(
        "--acknowledge-governance",
        default=None,
        help=(
            "required for either governance write; exact value: "
            f"{GOVERNANCE_ACKNOWLEDGEMENT}"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.describe:
            payload = fitter.describe_protocol(args.stage)
        elif args.preflight:
            payload = fitter.preflight(args.stage)
        elif args.write_protocol_freeze or args.write_execution_lock:
            if args.acknowledge_governance != GOVERNANCE_ACKNOWLEDGEMENT:
                raise fitter.V12R15MaskedTeacherFitError(
                    "governance write requires the exact reviewed-source acknowledgement"
                )
            payload = (
                fitter.write_protocol_freeze()
                if args.write_protocol_freeze
                else fitter.write_execution_lock()
            )
        else:
            if args.acknowledge_one_shot != EXECUTION_ACKNOWLEDGEMENT:
                raise fitter.V12R15MaskedTeacherFitError(
                    "--execute requires the exact one-shot acknowledgement"
                )
            payload = fitter.run_production_fit(stage=args.stage)
        print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
        return 0 if payload.get("passed", True) is True else 2
    except fitter.V12R15MaskedTeacherFitError as exc:
        print(
            json.dumps(
                {
                    "status": "FAIL_H0_V12R15_MASKED_SAFE_TEACHER_CLI",
                    "passed": False,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                },
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
