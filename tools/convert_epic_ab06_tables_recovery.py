#!/usr/bin/env python3
"""Recovery converter for EPIC AB06 trials with wider force-plate coverage.

The frozen V14 converter remains untouched because its file identity is part of
the failed-run provenance.  This module reuses that converter's decoder,
mapping, writers, IK setup, and receipt chain while making the downstream time
range explicit.  Source streams may have different endpoints, but every stream
must cover the complete protocol-required range and is written without crop.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np

if __package__:
    from . import convert_epic_ab06_tables as original
else:  # pragma: no cover - exercised by direct CLI use.
    import convert_epic_ab06_tables as original


ConversionError = original.ConversionError
TableSchemaError = original.TableSchemaError
NoClobberError = original.NoClobberError

CONVERSION_MANIFEST_SCHEMA_VERSION = original.CONVERSION_MANIFEST_SCHEMA_VERSION
REQUIRED_OPENSIM_VERSION = original.REQUIRED_OPENSIM_VERSION
IK_ACCURACY = original.IK_ACCURACY
MARKER_NAMES = original.MARKER_NAMES

ORIGINAL_CONVERTER_SHA256 = (
    "015f7e96f4f3dd6a923e3c1115d4f41d07faf1d997419138e11e9d6f9d8433cb"
)
SOURCE_TIME_COVERAGE_SCHEMA_VERSION = 1
SOURCE_TIME_ENDPOINT_TOLERANCE_S = 1.0e-9
SOURCE_COVERAGE_RULE = "every_source_covers_downstream_required_time_range"
LEGACY_EQUAL_ENDPOINT_RULE = "all_source_endpoints_equal_within_tolerance"


def _validate_original_converter_identity() -> tuple[Path, str]:
    converter_path = Path(original.__file__).resolve()
    observed = original._sha256_file(converter_path)
    if observed != ORIGINAL_CONVERTER_SHA256:
        raise ConversionError(
            "original converter identity drifted: "
            f"expected={ORIGINAL_CONVERTER_SHA256}, observed={observed}"
        )
    return converter_path, observed


def _time_range(
    time: np.ndarray,
    *,
    label: str,
) -> tuple[np.ndarray, tuple[float, float]]:
    validated = original._validate_time(np.asarray(time), Path(label))
    return validated, (float(validated[0]), float(validated[-1]))


def validate_source_time_coverage(
    *,
    dataset_ik_time: np.ndarray,
    force_plate_time: np.ndarray,
    marker_time: np.ndarray,
    required_time_range_s: Sequence[float] | None,
) -> dict[str, object]:
    """Validate source coverage and return immutable manifest evidence."""
    ik_time, ik_range = _time_range(dataset_ik_time, label="dataset IK")
    fp_time, fp_range = _time_range(force_plate_time, label="force plate")
    markers, marker_range = _time_range(marker_time, label="markers")
    tolerance = SOURCE_TIME_ENDPOINT_TOLERANCE_S
    stream_data = {
        "dataset_ik": (ik_time, ik_range),
        "force_plate": (fp_time, fp_range),
        "markers": (markers, marker_range),
    }

    if required_time_range_s is None:
        endpoints = (*ik_range, *fp_range, *marker_range)
        if not all(
            math.isclose(
                endpoint,
                ik_range[index % 2],
                rel_tol=0.0,
                abs_tol=tolerance,
            )
            for index, endpoint in enumerate(endpoints)
        ):
            raise TableSchemaError(
                "required_time_range_s is mandatory when source endpoints "
                "differ; legacy inference requires equal endpoints"
            )
        required_range = ik_range
        range_origin = "legacy_equal_source_endpoints"
    else:
        required_range = original._validated_time_range(required_time_range_s)
        range_origin = "explicit_protocol_range"

    streams: dict[str, object] = {}
    uncovered: list[str] = []
    for name, (time, stream_range) in stream_data.items():
        covers = (
            stream_range[0] <= required_range[0] + tolerance
            and stream_range[1] >= required_range[1] - tolerance
        )
        streams[name] = {
            "time_range_s": list(stream_range),
            "rows": int(time.size),
            "covers_required_range": covers,
        }
        if not covers:
            uncovered.append(name)

    if uncovered:
        raise TableSchemaError(
            "source time range does not fully cover downstream required "
            f"range {list(required_range)} within {tolerance:.1e} s; "
            f"uncovered={uncovered}"
        )

    return {
        "schema_version": SOURCE_TIME_COVERAGE_SCHEMA_VERSION,
        "endpoint_tolerance_s": tolerance,
        "coverage_rule": SOURCE_COVERAGE_RULE,
        "legacy_inference_rule": LEGACY_EQUAL_ENDPOINT_RULE,
        "range_origin": range_origin,
        "downstream_required_time_range_s": list(required_range),
        "all_sources_cover_required_range": True,
        "dataset_ik_used_downstream": False,
        "streams": streams,
    }


def convert_trial(
    *,
    ik_mat: str | Path,
    fp_mat: str | Path,
    markers_mat: str | Path,
    output_dir: str | Path,
    trial: str | None = None,
    ik_model: str | Path | None = None,
    ik_plugin: str | Path | None = None,
    required_time_range_s: Sequence[float] | None = None,
) -> dict[str, object]:
    """Convert a trial using the recovery source-time coverage contract."""
    destination = Path(output_dir).expanduser().resolve()
    original._preflight_output_dir(destination)
    original_converter_path, original_converter_sha256 = (
        _validate_original_converter_identity()
    )
    sources = tuple(
        Path(item).expanduser().resolve()
        for item in (ik_mat, fp_mat, markers_mat)
    )
    trial_id = original._validate_trial_sources(sources, trial)
    resolved_ik_model = (
        None if ik_model is None else original._validate_ik_model(ik_model)
    )
    if (resolved_ik_model is None) != (ik_plugin is None):
        raise ConversionError(
            "--ik-model and --ik-plugin must be supplied together"
        )
    plugin_spec = (
        None if ik_plugin is None else original.resolve_plugin_spec(ik_plugin)
    )

    coordinate_data = original.extract_coordinate_data(
        original.decode_mcos_table(sources[0])
    )
    grf_data = original.extract_grf_data(
        original.decode_mcos_table(sources[1])
    )
    marker_data = original.extract_marker_data(
        original.decode_mcos_table(sources[2])
    )
    time_coverage = validate_source_time_coverage(
        dataset_ik_time=coordinate_data.time,
        force_plate_time=grf_data.time,
        marker_time=marker_data.time,
        required_time_range_s=required_time_range_s,
    )
    downstream_time_range = time_coverage[
        "downstream_required_time_range_s"
    ]
    assert isinstance(downstream_time_range, list)

    destination.mkdir(parents=True, exist_ok=False)
    ik_path = destination / f"{trial_id}_ik_dataset_ab06_seasea.mot"
    grf_path = destination / f"{trial_id}_grf.mot"
    trc_path = destination / f"{trial_id}.trc"
    external_path = destination / f"{trial_id}_ExternalLoads.xml"
    ik_setup_path = destination / f"{trial_id}_iksetup.xml"
    ik_output_name = f"{trial_id}_ik.mot"
    manifest_path = destination / f"{trial_id}_conversion_manifest.json"
    original.write_coordinate_mot(ik_path, coordinate_data)
    original.write_grf_mot(grf_path, grf_data)
    original.write_trc(trc_path, marker_data)
    original.write_external_loads_xml(external_path, grf_path)

    inverse_kinematics: dict[str, object] | None = None
    if resolved_ik_model is not None:
        assert plugin_spec is not None
        original.write_ik_setup_xml(
            ik_setup_path,
            model_path=resolved_ik_model,
            marker_path=trc_path,
            time_range_s=downstream_time_range,
            output_motion_file=ik_output_name,
        )
        inverse_kinematics = {
            "status": "SETUP_ONLY_NOT_RUN",
            "required_opensim_version": REQUIRED_OPENSIM_VERSION,
            "accuracy": IK_ACCURACY,
            "time_range_s": downstream_time_range,
            "model": {
                "path": original._portable_relative_path(
                    resolved_ik_model, destination
                ),
                "sha256": original._sha256_file(resolved_ik_model),
            },
            "plugin": {
                "loader_basename": original._portable_relative_path(
                    plugin_spec.loader_basename, destination
                ),
                "binary_path": original._portable_relative_path(
                    plugin_spec.binary, destination
                ),
                "binary_sha256": original._sha256_file(plugin_spec.binary),
            },
            "marker_tasks": [
                {"name": name, "apply": True, "weight": 1.0}
                for name in MARKER_NAMES
            ],
            "setup": {
                "path": ik_setup_path.name,
                "sha256": original._sha256_file(ik_setup_path),
            },
            "output_motion_file": ik_output_name,
            "output_ik_sha256": None,
            "run_helper_command": [
                "python",
                original._portable_relative_path(Path(__file__), destination),
                "--run-ik-setup",
                ik_setup_path.name,
                "--ik-plugin",
                original._portable_relative_path(
                    plugin_spec.loader_basename, destination
                ),
            ],
            "receipt_helper_command": [
                "python",
                original._portable_relative_path(Path(__file__), destination),
                "--finalize-ik-output",
                ik_output_name,
                "--ik-setup",
                ik_setup_path.name,
                "--ik-plugin",
                original._portable_relative_path(
                    plugin_spec.loader_basename, destination
                ),
                "--opensim-version",
                REQUIRED_OPENSIM_VERSION,
            ],
        }

    manifest = {
        "schema_version": CONVERSION_MANIFEST_SCHEMA_VERSION,
        "status": "CONVERTED",
        "trial": trial_id,
        "recovery_lineage": {
            "original_converter": {
                "path": original._portable_relative_path(
                    original_converter_path, destination
                ),
                "sha256": original_converter_sha256,
            }
        },
        "sources": {
            role: {
                "filename": path.name,
                "sha256": original._sha256_file(path),
            }
            for role, path in zip(("ik", "force_plate", "markers"), sources)
        },
        "time_range_s": downstream_time_range,
        "source_time_coverage": time_coverage,
        "outputs": {
            role: {
                "path": path.name,
                "sha256": original._sha256_file(path),
            }
            for role, path in (
                ("dataset_ik", ik_path),
                ("grf", grf_path),
                ("markers_trc", trc_path),
                ("external_loads", external_path),
            )
        },
        "inverse_kinematics": inverse_kinematics,
    }
    original._write_json_no_clobber(manifest_path, manifest)
    return {
        "status": "ok",
        "trial": trial_id,
        "output_dir": destination.as_posix(),
        "ik_mot": ik_path.as_posix(),
        "grf_mot": grf_path.as_posix(),
        "trc": trc_path.as_posix(),
        "external_loads_xml": external_path.as_posix(),
        "ik_setup_xml": (
            None if resolved_ik_model is None else ik_setup_path.as_posix()
        ),
        "conversion_manifest": manifest_path.as_posix(),
        "time_range_s": downstream_time_range,
        "source_time_coverage": time_coverage,
        "ik_rows": int(coordinate_data.time.size),
        "grf_rows": int(grf_data.time.size),
        "marker_rows": int(marker_data.time.size),
    }


def run_ik_from_setup(
    *,
    setup_xml: str | Path,
    ik_plugin: str | Path,
) -> dict[str, object]:
    """Run the original hash-linked IK chain on a schema-1 manifest."""
    _validate_original_converter_identity()
    return original.run_ik_from_setup(
        setup_xml=setup_xml,
        ik_plugin=ik_plugin,
    )


def finalize_ik_receipt(
    *,
    setup_xml: str | Path,
    output_ik_mot: str | Path,
    ik_plugin: str | Path,
    opensim_version: str | None = None,
    receipt_path: str | Path | None = None,
) -> dict[str, object]:
    """Finalize IK using the original schema-1-compatible verifier."""
    _validate_original_converter_identity()
    return original.finalize_ik_receipt(
        setup_xml=setup_xml,
        output_ik_mot=output_ik_mot,
        ik_plugin=ik_plugin,
        opensim_version=opensim_version,
        receipt_path=receipt_path,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    """Return the original CLI parser; only conversion dispatch changes."""
    parser = original.build_arg_parser()
    parser.add_argument(
        "--required-time-range-s",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help=(
            "Frozen downstream time range; every source must cover it. "
            "Omit only for legacy sources with equal endpoints."
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        if args.run_ik_setup is not None:
            if args.ik_plugin is None:
                parser.error("--run-ik-setup requires --ik-plugin")
            result = run_ik_from_setup(
                setup_xml=args.run_ik_setup,
                ik_plugin=args.ik_plugin,
            )
        elif args.finalize_ik_output is not None:
            missing = [
                name
                for name, value in (
                    ("--ik-setup", args.ik_setup),
                    ("--ik-plugin", args.ik_plugin),
                )
                if value is None
            ]
            if missing:
                parser.error(
                    "--finalize-ik-output requires " + ", ".join(missing)
                )
            result = finalize_ik_receipt(
                setup_xml=args.ik_setup,
                output_ik_mot=args.finalize_ik_output,
                ik_plugin=args.ik_plugin,
                opensim_version=args.opensim_version,
                receipt_path=args.receipt_path,
            )
        else:
            missing = [
                name
                for name, value in (
                    ("--ik-mat", args.ik_mat),
                    ("--fp-mat", args.fp_mat),
                    ("--markers-mat", args.markers_mat),
                    ("--output-dir", args.output_dir),
                )
                if value is None
            ]
            if missing:
                parser.error("conversion requires " + ", ".join(missing))
            result = convert_trial(
                ik_mat=args.ik_mat,
                fp_mat=args.fp_mat,
                markers_mat=args.markers_mat,
                output_dir=args.output_dir,
                trial=args.trial,
                ik_model=args.ik_model,
                ik_plugin=args.ik_plugin,
                required_time_range_s=args.required_time_range_s,
            )
    except (ConversionError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "ok": False,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
