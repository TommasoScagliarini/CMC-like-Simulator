"""Fail-closed V7 harness correction for the frozen V6 sensor grid.

V6 stopped before selection because its protocol omitted the inherited
``sealed_validation_gate`` dictionary required by the V3/V4 gate helper.  V7
adds only that missing contract.  Candidate geometry, both sampling rates,
sensor thresholds, dwell, evidence routing, selection, and all scientific
criteria remain identical to V6.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (REPO_ROOT, VALIDATION_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_timing_placements_prescribed_v6 as v6  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v7.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_multires_geometry_v7"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_timing_placement_v7"
PROTOCOL_ID = "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V7"
EXPECTED_BASE_PROTOCOL = (
    "validation/two_sensor_timing_placement_sweep_protocol_v6.json"
)
EXPECTED_BASE_PROTOCOL_SHA256 = (
    "d339db6a18f1072a83d086fc5436b804ee3a05dc6d5f1bb7b8f1ca4eff844f78"
)
EXPECTED_INVALID_V6_MANIFEST = (
    "validation/two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_multires_geometry_v6/manifest.json"
)
SEALED_GATE = {
    "precision": 1.0,
    "recall": 1.0,
    "max_abs_hs_error_s": 0.05,
    "max_abs_toe_off_error_s": 0.08,
    "minimum_confirmed_fsm_stance_f1": 0.95,
    "minimum_confirmed_fsm_stance_iou": 0.90,
    "maximum_confirmed_fsm_iou_regression_vs_baseline": 0.01,
    "maximum_confirmed_time_worst_timing_regression_vs_baseline": 0.0,
}


class ProtocolError(v6.ProtocolError):
    """Raised before sampling if the V7 correction contract drifts."""


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    correction_path = v6.v1.resolve_repo_path(path).resolve()
    try:
        correction = json.loads(correction_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V7 protocol: {correction_path}") from exc
    if not isinstance(correction, dict):
        raise ProtocolError("V7 protocol root must be an object")
    expected = {
        "schema_version": 7,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_multiresolution_geometry_harness_correction",
        "base_protocol": {
            "path": EXPECTED_BASE_PROTOCOL,
            "sha256": EXPECTED_BASE_PROTOCOL_SHA256,
        },
        "scientific_contract_changed_from_v6": False,
        "correction": {
            "reason": "add_missing_inherited_sealed_validation_gate",
            "sealed_validation_gate": SEALED_GATE,
        },
    }
    for key, value in expected.items():
        if correction.get(key) != value:
            raise ProtocolError(f"V7 correction field drifted: {key}")
    sources = correction.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("V7 hash-pinned sources are required")
    for label, record in sources.items():
        source_path = v6.v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V7 pinned source: {label}")
        if v6.v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V7 source hash drift: {label}")

    base_path = v6.v1.resolve_repo_path(EXPECTED_BASE_PROTOCOL).resolve()
    if v6.v1._sha256(base_path) != EXPECTED_BASE_PROTOCOL_SHA256:
        raise ProtocolError("frozen V6 base protocol drifted")
    base = v6.load_and_validate_protocol(base_path)
    base.update(
        {
            "schema_version": 7,
            "protocol_id": PROTOCOL_ID,
            "stage": correction["stage"],
            "objective": correction["objective"],
            "sealed_validation_gate": dict(SEALED_GATE),
            "interpretation_limits": correction["interpretation_limits"],
            "_protocol_path": correction_path.as_posix(),
            "_protocol_sha256": v6.v1._sha256(correction_path),
            "_v7_harness_correction": {
                "scientific_contract_changed_from_v6": False,
                "reason": correction["correction"]["reason"],
                "invalid_v6_manifest": EXPECTED_INVALID_V6_MANIFEST,
            },
        }
    )
    return base


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    manifest = v6.run_sweep(protocol, output_dir, plot_dir)
    manifest.update(
        {
            "schema_version": 7,
            "stage": "development_multiresolution_geometry_harness_correction",
            "harness_revision": dict(protocol["_v7_harness_correction"]),
        }
    )
    (output_dir / "manifest.json").write_text(
        json.dumps(
            v6.v1._json_safe(manifest),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return manifest


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run V7 harness correction on the frozen V6 two-sensor grid."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = v6.v1.resolve_repo_path(args.output_dir).resolve()
    plot_dir = v6.v1.resolve_repo_path(args.plot_dir).resolve()
    try:
        v6._preflight_no_clobber(output_dir, plot_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_sweep(protocol, output_dir, plot_dir)
    except v6.NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": 7,
                    "status": "ERROR",
                    "ok": False,
                    "no_clobber": True,
                    "filesystem_mutated": False,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 2
    except Exception as exc:  # pragma: no cover
        print(
            json.dumps(
                {
                    "schema_version": 7,
                    "status": "ERROR",
                    "ok": False,
                    "sealed_block_opened": False,
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                },
                indent=2,
            )
        )
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "conclusion": manifest["conclusion"],
                "selected_pair": manifest["selected_pair"],
                "sealed_block_opened": False,
                "manifest": v6.v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
