"""V9 final local heel-placement sweep for a two-sensor gait detector.

The V8 best candidate missed the 1 ms HS gate by 1.469 ms and retained a
three-sample initial transfer gap.  V9 extrapolates only the monotonic heel
depth/anterior trend and makes one small F79/F79.5 forefoot comparison at
fixed P35.  Every candidate remains one heel sphere plus one forefoot sphere.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (REPO_ROOT, VALIDATION_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_timing_placements_prescribed_v8 as v8  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v9.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_heel_micro_v9"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_timing_placement_v9"
PROTOCOL_ID = "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V9"
BASE_PROTOCOL = "validation/two_sensor_timing_placement_sweep_protocol_v8.json"
BASE_PROTOCOL_SHA256 = (
    "4d2065b9751b1ad179379ef4bb952a80620ac3b0996301af7f1e640744f25178"
)
V8_MANIFEST = (
    "validation/two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_heel_micro_v8/manifest.json"
)
V8_MANIFEST_SHA256 = (
    "5fff95a44da968c54df81dc02a87128dc4bd025448feed8bed416e36c38d4840"
)
HEEL_OFFSETS_MM = (2.5, 2.75)
HEEL_X_DELTAS_MM = (3.0, 3.25)
FOREFOOT_FRACTIONS = (0.79, 0.795)
FOREFOOT_PROTRUSION_MM = 35.0
V8_COMPARATOR_ID = "v8_h2p25_x2p75_f79p5_p35_comparator"
SELECTABLE_COUNT = 8
PAIR_COUNT = 10
EXPECTED_DETECTOR_SPHERES = 9
EXPECTED_PRIMARY_SPHERES = 8
EXPECTED_TOTAL_SPHERES = 17


class ProtocolError(v8.ProtocolError):
    """Raised before sampling if the frozen V9 contract drifts."""


def _numeric_list(value: Any, expected: Sequence[float], label: str) -> None:
    try:
        observed = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be a numeric list") from exc
    if observed != [float(item) for item in expected]:
        raise ProtocolError(f"{label} drifted: {observed}")


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v8.v6.v1.resolve_repo_path(path).resolve()
    try:
        revision = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V9 protocol: {protocol_path}") from exc
    if not isinstance(revision, dict):
        raise ProtocolError("V9 protocol root must be an object")
    for key, expected in {
        "schema_version": 9,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_final_heel_micro_multiresolution",
        "base_protocol": {"path": BASE_PROTOCOL, "sha256": BASE_PROTOCOL_SHA256},
        "source_v8_manifest": {"path": V8_MANIFEST, "sha256": V8_MANIFEST_SHA256},
        "threshold_fsm_gate_or_evidence_routing_changed": False,
        "declared_final_local_iteration": True,
    }.items():
        if revision.get(key) != expected:
            raise ProtocolError(f"V9 protocol field drifted: {key}")
    grid = revision.get("placement_grid", {})
    _numeric_list(
        grid.get("heel_vertical_offsets_below_current_mm"),
        HEEL_OFFSETS_MM,
        "heel offset",
    )
    _numeric_list(
        grid.get("heel_anterior_x_deltas_mm"), HEEL_X_DELTAS_MM, "heel x"
    )
    _numeric_list(
        grid.get("forefoot_longitudinal_fractions_mesh_x"),
        FOREFOOT_FRACTIONS,
        "forefoot fraction",
    )
    _numeric_list(
        grid.get("forefoot_absolute_local_plantar_protrusion_mm"),
        [FOREFOOT_PROTRUSION_MM],
        "forefoot depth",
    )
    for key, expected in {
        "selectable_candidate_count": SELECTABLE_COUNT,
        "nonselectable_comparator_count": 2,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_absolute_heel_forefoot_bottom_offset_mm": 20.0,
    }.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"V9 grid field drifted: {key}")
    expected_sample = {
        "method": "all_required_spheres_directly_in_one_opensim_pass",
        "expected_unique_detector_spheres": EXPECTED_DETECTOR_SPHERES,
        "expected_primary_load_spheres": EXPECTED_PRIMARY_SPHERES,
        "expected_total_unique_spheres": EXPECTED_TOTAL_SPHERES,
        "evaluated_pair_count": PAIR_COUNT,
    }
    if revision.get("sampling") != {
        "runtime_10ms": expected_sample,
        "fine_1ms": expected_sample,
        "affine_reconstruction": False,
        "shared_spheres_are_offline_sampling_optimization_only": True,
    }:
        raise ProtocolError("V9 sampling contract drifted")
    if revision.get("selection") != {
        "eligibility": "full_strict_pass_at_both_10ms_and_1ms",
        "ranking": [
            "multiresolution_worst_event_normalized_max_abs_error:min",
            "multiresolution_mean_event_normalized_mean_abs_error:min",
            "minimum_multiresolution_confirmed_fsm_stance_iou:max",
            "geometry_displacement_from_current_m:min",
            "candidate_id:lexicographic",
        ],
    }:
        raise ProtocolError("V9 selection contract drifted")
    sources = revision.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("V9 hash-pinned sources are required")
    for label, record in sources.items():
        source_path = v8.v6.v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V9 pinned source: {label}")
        if v8.v6.v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V9 source hash drift: {label}")
    base_path = v8.v6.v1.resolve_repo_path(BASE_PROTOCOL).resolve()
    if v8.v6.v1._sha256(base_path) != BASE_PROTOCOL_SHA256:
        raise ProtocolError("V8 base protocol drifted")
    if v8.v6.v1._sha256(v8.v6.v1.resolve_repo_path(V8_MANIFEST)) != V8_MANIFEST_SHA256:
        raise ProtocolError("V8 result manifest drifted")
    base = v8.load_and_validate_protocol(base_path)
    base.update(
        {
            "schema_version": 9,
            "protocol_id": PROTOCOL_ID,
            "stage": revision["stage"],
            "objective": revision["objective"],
            "placement_grid": grid,
            "sampling": revision["sampling"],
            "selection": revision["selection"],
            "interpretation_limits": revision["interpretation_limits"],
            "_protocol_path": protocol_path.as_posix(),
            "_protocol_sha256": v8.v6.v1._sha256(protocol_path),
            "_v9_lineage": {
                "base_protocol": revision["base_protocol"],
                "source_v8_manifest": revision["source_v8_manifest"],
                "scientific_change": "final_local_placement_grid_only",
            },
        }
    )
    return base


def _token(value: float, digits: int = 2) -> str:
    return f"{float(value):.{digits}f}".replace(".", "p")


def _candidate_id(offset_mm: float, x_mm: float, fraction: float) -> str:
    fraction_token = f"{100.0 * float(fraction):04.1f}".replace(".", "p")
    return (
        f"H{_token(offset_mm)}_X{_token(x_mm)}_F{fraction_token}_"
        f"P{_token(FOREFOOT_PROTRUSION_MM)}"
    )


def build_placement_candidates(
    protocol: Mapping[str, Any],
) -> tuple[
    v8.v6.v1.OnlineGRFProfile,
    list[v8.v6.v1.PlacementCandidate],
    dict[str, Any],
]:
    base = v8.v6.v1.load_online_grf_profile(
        v8.v6.v1.resolve_repo_path(
            str(protocol["detector_template_profile"])
        ).resolve()
    )
    sensors = v8.v6.v1._left_sensor_spheres(base)
    heel_template = sensors["left_heel"]
    toe_template = sensors["left_toe"]
    setup = v8.v6.v1.read_setup_xml(
        v8.v6.v1.resolve_repo_path(str(protocol["setup"])).resolve()
    )
    mesh_path = v8.v6.v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v8.v6.v1._load_stl_triangles(mesh_path)
    radius = float(heel_template.radius)
    toe_locations: dict[float, tuple[tuple[float, float, float], dict[str, Any]]] = {}
    for fraction in FOREFOOT_FRACTIONS:
        toe_locations[fraction] = v8.v6.v1._derive_forefoot_location(
            triangles,
            toe_template,
            fraction=fraction,
            protrusion_mm=FOREFOOT_PROTRUSION_MM,
        )
    candidates: list[v8.v6.v1.PlacementCandidate] = []
    bottom_offsets: list[float] = []
    for offset_mm in HEEL_OFFSETS_MM:
        for x_mm in HEEL_X_DELTAS_MM:
            heel_location = (
                float(heel_template.location[0] + x_mm / 1000.0),
                float(heel_template.location[1] - offset_mm / 1000.0),
                float(heel_template.location[2]),
            )
            heel_geometry = v8.v6.v3._mesh_geometry(
                heel_location, radius, triangles
            )
            for fraction in FOREFOOT_FRACTIONS:
                toe_location, derived = toe_locations[fraction]
                toe_geometry = v8.v6.v3._mesh_geometry(
                    toe_location, radius, triangles
                )
                bottom_offset_mm = 1000.0 * (
                    float(toe_location[1] - radius)
                    - float(heel_location[1] - radius)
                )
                bottom_offsets.append(bottom_offset_mm)
                checks = {
                    "heel_within_5mm_of_mesh": heel_geometry["within_5mm_of_mesh"],
                    "forefoot_within_5mm_of_mesh": toe_geometry[
                        "within_5mm_of_mesh"
                    ],
                    "heel_forefoot_bottoms_within_20mm": bool(
                        abs(bottom_offset_mm) <= 20.0 + 1e-9
                    ),
                    "exactly_two_spheres": True,
                }
                candidates.append(
                    v8.v6.v1.PlacementCandidate(
                        candidate_id=_candidate_id(offset_mm, x_mm, fraction),
                        heel_location=heel_location,
                        forefoot_location=toe_location,
                        heel_offset_below_current_mm=offset_mm,
                        forefoot_fraction_mesh_x=fraction,
                        forefoot_protrusion_mm=FOREFOOT_PROTRUSION_MM,
                        selectable=True,
                        role="development_final_heel_micro_candidate",
                        geometry={
                            **derived,
                            "heel": heel_geometry,
                            "forefoot": toe_geometry,
                            "heel_anterior_x_delta_mm": x_mm,
                            "signed_forefoot_minus_heel_bottom_offset_mm": (
                                bottom_offset_mm
                            ),
                            "pre_gate_checks": checks,
                            "pre_gate_ok": bool(all(checks.values())),
                            "detector_representation": "two_spheres_only",
                        },
                    )
                )
    comparator_heel = (
        float(heel_template.location[0] + 0.00275),
        float(heel_template.location[1] - 0.00225),
        float(heel_template.location[2]),
    )
    comparator_toe, comparator_derived = v8.v6.v1._derive_forefoot_location(
        triangles, toe_template, fraction=0.795, protrusion_mm=35.0
    )
    candidates.append(
        v8.v6.v1.PlacementCandidate(
            candidate_id=V8_COMPARATOR_ID,
            heel_location=comparator_heel,
            forefoot_location=comparator_toe,
            heel_offset_below_current_mm=2.25,
            forefoot_fraction_mesh_x=0.795,
            forefoot_protrusion_mm=35.0,
            selectable=False,
            role="nonselectable_v8_best_comparator",
            geometry={
                **comparator_derived,
                "heel_anterior_x_delta_mm": 2.75,
                "source": "V8_best_H2p25_X2p75_F79p5_P35",
                "detector_representation": "two_spheres_only",
            },
        )
    )
    candidates.append(
        v8.v6.v1.PlacementCandidate(
            candidate_id=v8.v6.CURRENT_COMPARATOR_ID,
            heel_location=tuple(heel_template.location),
            forefoot_location=tuple(toe_template.location),
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=False,
            role="nonselectable_current_geometry_comparator",
            geometry={
                "source": "current_detector_profile",
                "detector_representation": "two_spheres_only",
            },
        )
    )
    selectable = [item for item in candidates if item.selectable]
    if len(selectable) != SELECTABLE_COUNT or len(candidates) != PAIR_COUNT:
        raise ProtocolError("V9 candidate grid drifted")
    if len({item.candidate_id for item in candidates}) != PAIR_COUNT:
        raise ProtocolError("V9 candidate IDs are not unique")
    if not all(bool(item.geometry.get("pre_gate_ok")) for item in selectable):
        raise ProtocolError("a selectable V9 geometry failed its pre-gate")
    maximum_offset = max(abs(value) for value in bottom_offsets)
    if maximum_offset > 20.0 + 1e-9:
        raise ProtocolError("V9 bottom-offset pre-gate failed")
    return base, candidates, {
        "mesh": v8.v6.v1._source_record(mesh_path),
        "selectable_candidate_count": SELECTABLE_COUNT,
        "nonselectable_comparator_count": 2,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_absolute_bottom_offset_mm": maximum_offset,
        "detector_representation": "two_spheres_per_pair",
        "shared_spheres_are_offline_sampling_optimization_only": True,
        "affine_reconstruction_used": False,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot(
    path: Path,
    runtime_rows: Sequence[Mapping[str, Any]],
    fine_rows: Sequence[Mapping[str, Any]],
    winner: str | None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runtime = {str(row["candidate_id"]): row for row in runtime_rows}
    fine = {str(row["candidate_id"]): row for row in fine_rows}
    ids = [
        str(row["candidate_id"])
        for row in runtime_rows
        if bool(row.get("selectable"))
    ]
    positions = np.arange(len(ids), dtype=float)
    width = 0.36
    fig, axes = plt.subplots(2, 3, figsize=(17, 10), sharex=True)
    metrics = (
        ("max_abs_hs_error_s", "max |HS| [ms]", 50.0, 1000.0, True),
        ("max_abs_toe_off_error_s", "max |TO| [ms]", 80.0, 1000.0, True),
        ("predicted_hs_count", "HS accepted", 51.0, 1.0, False),
        (
            "transfer_both_latches_off_sample_count",
            "heel+toe both OFF [samples]",
            0.0,
            1.0,
            False,
        ),
        (
            "minimum_causal_toe_clear_before_next_hs_onset_s",
            "minimum causal toe-clear [ms]",
            30.0,
            1000.0,
            False,
        ),
        (
            "toe_latch_recontact_episode_count",
            "toe recontact [diagnostic]",
            None,
            1.0,
            False,
        ),
    )
    for axis, (metric, ylabel, threshold, scale, hide_sentinel) in zip(
        axes.flat, metrics
    ):
        def values(rows: Mapping[str, Mapping[str, Any]]) -> list[float]:
            result: list[float] = []
            for candidate_id in ids:
                value = float(rows[candidate_id][metric])
                result.append(
                    float("nan") if hide_sentinel and value >= 900.0 else scale * value
                )
            return result

        axis.bar(positions - width / 2, values(runtime), width, label="10 ms")
        axis.bar(positions + width / 2, values(fine), width, label="1 ms")
        if threshold is not None:
            axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.8)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)
        if winner is not None:
            selected = ids.index(winner)
            axis.axvspan(selected - 0.48, selected + 0.48, color="gold", alpha=0.16)
    axes[0, 0].legend(loc="best")
    for axis in axes[-1, :]:
        axis.set_xticks(positions, ids, rotation=45, ha="right")
    fig.suptitle(
        "Two-sensor V9 final local sweep — 1 heel + 1 forefoot, 10 ms / 1 ms"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    v8.v6._preflight_no_clobber(output_dir, plot_dir)
    base, candidates, geometry = build_placement_candidates(protocol)
    rows_by_dt: dict[str, list[dict[str, Any]]] = {}
    details_by_dt: dict[str, dict[str, Any]] = {}
    access_by_dt: dict[str, Any] = {}
    for label, sample_dt_s in (
        ("runtime_10ms", v8.v6.PRIMARY_DT_S),
        ("fine_1ms", v8.v6.FINE_DT_S),
    ):
        streams, access = v8.v6.v4.sample_streams_once(
            protocol,
            base,
            candidates,
            sample_dt_s=sample_dt_s,
            expected_detector_spheres=EXPECTED_DETECTOR_SPHERES,
            expected_total_spheres=EXPECTED_TOTAL_SPHERES,
        )
        rows: list[dict[str, Any]] = []
        details: dict[str, Any] = {}
        for candidate in candidates:
            row, detail = v8.v6.evaluate_placement(
                protocol,
                candidate,
                streams[candidate.candidate_id],
                sample_dt_s=sample_dt_s,
            )
            rows.append(row)
            details[candidate.candidate_id] = detail
        rows_by_dt[label] = rows
        details_by_dt[label] = details
        access_by_dt[label] = access
    winner_id, selection = v8.select_multiresolution_winner(
        rows_by_dt["runtime_10ms"], rows_by_dt["fine_1ms"], protocol
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_10 = output_dir / "timing_placement_v9_runtime_10ms_metrics.csv"
    csv_1 = output_dir / "timing_placement_v9_fine_1ms_metrics.csv"
    plot_path = plot_dir / "01_timing_placement_v9_multiresolution.png"
    _write_csv(csv_10, rows_by_dt["runtime_10ms"])
    _write_csv(csv_1, rows_by_dt["fine_1ms"])
    _plot(
        plot_path,
        rows_by_dt["runtime_10ms"],
        rows_by_dt["fine_1ms"],
        winner_id,
    )
    ok = winner_id is not None
    manifest = {
        "schema_version": 9,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": protocol["stage"],
        "objective": protocol["objective"],
        "protocol": {
            "path": v8.v6.v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
        },
        "lineage": protocol["_v9_lineage"],
        "data_access": {
            "already_open_block_s": [
                v8.v6.v1.BLOCK_START_S,
                v8.v6.v1.SEALED_START_S,
            ],
            "sealed_block_s": [v8.v6.v1.SEALED_START_S, v8.v6.v1.SEALED_END_S],
            "sealed_block_opened": False,
            **access_by_dt,
        },
        "detector_contract": {
            "sensors_per_pair": 2,
            "sensor_roles": ["heel", "forefoot"],
            "event_guard_source": "candidate_two_sensor_forces_only",
            "normal_force_bw_source": "primary_online_grf_left_aggregate",
            "in_contact_source": (
                "primary_online_grf_left_union_physical_penetration"
            ),
            "detector_spheres_generate_grf": False,
            "fsm_thresholds_and_dwell_unchanged": True,
            "shared_batch_spheres_are_not_runtime_sensors": True,
        },
        "geometry": geometry,
        "candidates": [
            {
                "candidate_id": item.candidate_id,
                "selectable": item.selectable,
                "role": item.role,
                "heel_location_m": list(item.heel_location),
                "forefoot_location_m": list(item.forefoot_location),
                "heel_offset_below_current_mm": item.heel_offset_below_current_mm,
                "heel_anterior_x_delta_mm": item.geometry.get(
                    "heel_anterior_x_delta_mm"
                ),
                "forefoot_fraction_mesh_x": item.forefoot_fraction_mesh_x,
                "forefoot_protrusion_mm": item.forefoot_protrusion_mm,
                "sensor_count": 2,
                "geometry": dict(item.geometry),
            }
            for item in candidates
        ],
        "runtime_10ms": {
            "rows": rows_by_dt["runtime_10ms"],
            "details": details_by_dt["runtime_10ms"],
        },
        "fine_1ms": {
            "rows": rows_by_dt["fine_1ms"],
            "details": details_by_dt["fine_1ms"],
        },
        "selection": selection,
        "selected_pair": {
            "candidate_id": winner_id,
            "passed_10ms_and_1ms": ok,
            "profile_created": False,
            "promotable": False,
            "requires_future_sealed_holdout": True,
        },
        "conclusion": (
            "STRICT_MULTIRESOLUTION_DEVELOPMENT_WINNER"
            if ok
            else "NO_STRICT_MULTIRESOLUTION_DEVELOPMENT_WINNER"
        ),
        "artifacts": {
            "runtime_metrics_csv": v8.v6.v1._source_record(csv_10),
            "fine_metrics_csv": v8.v6.v1._source_record(csv_1),
            "multiresolution_plot": v8.v6.v1._source_record(plot_path),
        },
        "non_actions": {
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "production_fsm_modified": False,
            "current_profile_modified": False,
            "candidate_profile_created_or_promoted": False,
            "sealed_block_opened": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    safe = v8.v6.v1._json_safe(manifest)
    (output_dir / "manifest.json").write_text(
        json.dumps(safe, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run preregistered two-sensor V9 final local sweep."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = v8.v6.v1.resolve_repo_path(args.output_dir).resolve()
    plot_dir = v8.v6.v1.resolve_repo_path(args.plot_dir).resolve()
    try:
        v8.v6._preflight_no_clobber(output_dir, plot_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_sweep(protocol, output_dir, plot_dir)
    except v8.v6.NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": 9,
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
                    "schema_version": 9,
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
                "manifest": v8.v6.v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
