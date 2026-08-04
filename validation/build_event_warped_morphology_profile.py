"""Build the AB06 event-warped prosthetic morphology profile.

The production window deliberately ends at 153 s.  With the default AB06
prescribed GRF this selects 123 complete HS->TO->HS cycles and excludes the
next otherwise-complete cycle because its next heel strike is beyond that
window.  The exclusion is reported explicitly in the output metadata.

Each selected cycle is mapped piecewise to a canonical phase:

* HS -> TO maps to ``[0, alpha]``;
* TO -> next HS maps to ``[alpha, 1]``.

``alpha`` is the mean duty factor of the selected production cycles.  The
profile keeps a uniform 201-point base grid and inserts ``alpha`` when it is
not already one of those points, ensuring that toe-off is represented exactly.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import _cycles_from_vertical_grf, _read_storage_table  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402


DEFAULT_IK = "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
DEFAULT_GRF = "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot"
DEFAULT_OUTPUT = (
    "Trajectory Generator/baseline_MLP/morphology_profiles/"
    "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
DEFAULT_COORDINATES = ("pros_knee_angle", "pros_ankle_angle")
FLOAT_DIGITS = 10


def _stable_float(value: float, digits: int = FLOAT_DIGITS) -> float:
    """Round output numbers and normalize negative zero for stable JSON."""
    result = round(float(value), digits)
    return 0.0 if result == 0.0 else result


def _stable_array(values: np.ndarray) -> list[float]:
    return [_stable_float(value) for value in np.asarray(values, dtype=float)]


def _portable_path(path: Path) -> str:
    """Prefer repo-relative POSIX paths so metadata is OS-independent."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _storage_in_degrees(path: Path) -> bool | None:
    """Read OpenSim's optional ``inDegrees`` header flag."""
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            stripped = line.strip()
            lower = stripped.lower()
            if lower == "endheader":
                break
            if lower.startswith("indegrees"):
                if "=" in lower:
                    value = lower.split("=", 1)[1].strip()
                else:
                    fields = lower.split()
                    value = fields[-1] if len(fields) > 1 else ""
                if value in {"yes", "true", "1"}:
                    return True
                if value in {"no", "false", "0"}:
                    return False
                raise ValueError(f"invalid inDegrees header in {path}: {stripped!r}")
    return None


def _series_stats(values: Sequence[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    if array.size == 0 or not np.all(np.isfinite(array)):
        raise ValueError("cannot summarize an empty or non-finite series")
    return {
        "min": _stable_float(np.min(array)),
        "mean": _stable_float(np.mean(array)),
        "median": _stable_float(np.median(array)),
        "std_ddof0": _stable_float(np.std(array, ddof=0)),
        "p05": _stable_float(np.quantile(array, 0.05)),
        "p95": _stable_float(np.quantile(array, 0.95)),
        "max": _stable_float(np.max(array)),
    }


def _sample_interval_stats(times: np.ndarray) -> dict[str, float]:
    intervals = np.diff(np.asarray(times, dtype=float))
    if intervals.size == 0 or np.any(intervals <= 0.0):
        raise ValueError("storage time must be strictly increasing")
    return {
        "min_s": _stable_float(np.min(intervals)),
        "median_s": _stable_float(np.median(intervals)),
        "max_s": _stable_float(np.max(intervals)),
    }


def _cycle_record(cycle: tuple[float, float, float]) -> dict[str, float]:
    heel_strike, next_heel_strike, contact_duration = cycle
    toe_off = heel_strike + contact_duration
    period = next_heel_strike - heel_strike
    return {
        "heel_strike_s": _stable_float(heel_strike),
        "toe_off_s": _stable_float(toe_off),
        "next_heel_strike_s": _stable_float(next_heel_strike),
        "stance_duration_s": _stable_float(contact_duration),
        "swing_duration_s": _stable_float(next_heel_strike - toe_off),
        "period_s": _stable_float(period),
        "duty_factor": _stable_float(contact_duration / period),
    }


def _validate_storage(
    *,
    path: Path,
    times: np.ndarray,
    columns: Sequence[str],
    data: np.ndarray,
    required_columns: Sequence[str],
) -> None:
    if times.ndim != 1 or data.ndim != 2 or data.shape[0] != times.size:
        raise ValueError(f"invalid storage shape: {path}")
    if data.shape[1] != len(columns):
        raise ValueError(f"storage column/data mismatch: {path}")
    if times.size < 2 or not np.all(np.isfinite(times)):
        raise ValueError(f"invalid storage time: {path}")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError(f"storage time is not strictly increasing: {path}")
    missing = [name for name in required_columns if name not in columns]
    if missing:
        raise ValueError(f"missing columns in {path}: {missing}")
    indices = [columns.index(name) for name in required_columns]
    if not np.all(np.isfinite(data[:, indices])):
        raise ValueError(f"non-finite required data in {path}")


def load_inputs(
    ik: str | Path = DEFAULT_IK,
    grf: str | Path = DEFAULT_GRF,
    *,
    coordinates: Sequence[str] = DEFAULT_COORDINATES,
    grf_column: str = "ground_force1_vy",
    ik_units: str = "auto",
) -> dict[str, Any]:
    """Load and validate the IK/GRF sources used to build the profile.

    The returned dictionary intentionally exposes the normalized arrays needed
    by validation utilities: ``ik_time``, ``coordinate_values_rad``,
    ``grf_time`` and ``vertical_grf_n``.  Paths are absolute ``Path`` objects;
    source metadata is also included so callers do not need to parse headers a
    second time.
    """
    if ik_units not in {"auto", "degrees", "radians"}:
        raise ValueError("ik_units must be one of: auto, degrees, radians")

    ik_path = resolve_repo_path(ik).resolve()
    grf_path = resolve_repo_path(grf).resolve()
    for label, path in (("IK", ik_path), ("GRF", grf_path)):
        if not path.is_file():
            raise FileNotFoundError(f"{label} file not found: {path}")

    coordinate_names = tuple(coordinates)
    if not coordinate_names or len(set(coordinate_names)) != len(coordinate_names):
        raise ValueError("coordinates must be a non-empty sequence of unique names")

    ik_time, ik_columns, ik_data = _read_storage_table(str(ik_path))
    grf_time, grf_columns, grf_data = _read_storage_table(str(grf_path))
    _validate_storage(
        path=ik_path,
        times=ik_time,
        columns=ik_columns,
        data=ik_data,
        required_columns=coordinate_names,
    )
    _validate_storage(
        path=grf_path,
        times=grf_time,
        columns=grf_columns,
        data=grf_data,
        required_columns=(grf_column,),
    )

    header_in_degrees = _storage_in_degrees(ik_path)
    if ik_units == "auto":
        if header_in_degrees is None:
            raise ValueError(
                f"IK units are not declared in {ik_path}; pass --ik-units explicitly"
            )
        ik_in_degrees = header_in_degrees
    else:
        ik_in_degrees = ik_units == "degrees"

    coordinate_values_rad: dict[str, np.ndarray] = {}
    for coordinate in coordinate_names:
        source = np.asarray(
            ik_data[:, ik_columns.index(coordinate)],
            dtype=float,
        )
        coordinate_values_rad[coordinate] = (
            np.deg2rad(source) if ik_in_degrees else source.copy()
        )

    return {
        "ik_path": ik_path,
        "grf_path": grf_path,
        "coordinates": coordinate_names,
        "grf_column": grf_column,
        "ik_time": np.asarray(ik_time, dtype=float),
        "grf_time": np.asarray(grf_time, dtype=float),
        "coordinate_values_rad": coordinate_values_rad,
        "vertical_grf_n": np.asarray(
            grf_data[:, grf_columns.index(grf_column)],
            dtype=float,
        ),
        "ik_header_in_degrees": header_in_degrees,
        "ik_in_degrees": bool(ik_in_degrees),
    }


def detect_cycles(
    grf_time: np.ndarray,
    vertical_grf_n: np.ndarray,
    *,
    threshold_n: float = 20.0,
    t_start: float | None = None,
    t_end: float = 153.0,
    min_contact_duration_s: float = 0.05,
    min_cycle_duration_s: float = 0.30,
    expected_cycles: int | None = 123,
) -> dict[str, Any]:
    """Detect production and full-span HS->TO->HS cycles from vertical GRF.

    ``production_cycles`` and ``full_span_cycles`` contain tuples in the
    parser's native form: ``(heel_strike, next_heel_strike, stance_duration)``.
    The separate ``excluded_cycles`` records make the default 153 s cutoff
    auditable.
    """
    times = np.asarray(grf_time, dtype=float)
    force = np.asarray(vertical_grf_n, dtype=float)
    if times.ndim != 1 or force.ndim != 1 or times.size != force.size:
        raise ValueError("GRF time and vertical force must be equal-length vectors")
    if times.size < 2 or not np.all(np.isfinite(times)):
        raise ValueError("invalid GRF time vector")
    if not np.all(np.isfinite(force)):
        raise ValueError("vertical GRF contains non-finite values")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("GRF time must be strictly increasing")

    window_start = float(times[0]) if t_start is None else float(t_start)
    window_end = float(t_end)
    if not float(times[0]) <= window_start < window_end <= float(times[-1]):
        raise ValueError(
            "production window must lie inside GRF time range: "
            f"GRF=[{times[0]}, {times[-1]}], "
            f"requested=[{window_start}, {window_end}]"
        )

    detector_args = (times, force, float(threshold_n))
    production_cycles = _cycles_from_vertical_grf(
        *detector_args,
        window_start,
        window_end,
        float(min_contact_duration_s),
        float(min_cycle_duration_s),
    )
    full_span_cycles = _cycles_from_vertical_grf(
        *detector_args,
        window_start,
        float(times[-1]),
        float(min_contact_duration_s),
        float(min_cycle_duration_s),
    )
    if not production_cycles:
        raise ValueError("no complete production cycles were detected")
    if (
        expected_cycles is not None
        and expected_cycles > 0
        and len(production_cycles) != expected_cycles
    ):
        raise ValueError(
            f"expected {expected_cycles} production cycles, "
            f"detected {len(production_cycles)}"
        )

    production_keys = {
        (_stable_float(cycle[0]), _stable_float(cycle[1]))
        for cycle in production_cycles
    }
    excluded_cycles: list[dict[str, Any]] = []
    for cycle_index, cycle in enumerate(full_span_cycles):
        key = (_stable_float(cycle[0]), _stable_float(cycle[1]))
        if key in production_keys:
            continue
        record: dict[str, Any] = {
            "full_span_cycle_index": int(cycle_index),
            **_cycle_record(cycle),
        }
        if cycle[0] <= window_end < cycle[1]:
            record["exclusion_reason"] = "next_heel_strike_after_t_end"
        elif cycle[0] > window_end:
            record["exclusion_reason"] = "heel_strike_after_t_end"
        else:
            record["exclusion_reason"] = "outside_production_window"
        excluded_cycles.append(record)

    return {
        "production_cycles": list(production_cycles),
        "full_span_cycles": list(full_span_cycles),
        "excluded_cycles": excluded_cycles,
        "t_start": window_start,
        "t_end": window_end,
    }


def _phase_grid(base_points: int, alpha: float) -> tuple[np.ndarray, bool]:
    if base_points < 2:
        raise ValueError("base phase grid must contain at least two points")
    base = np.asarray(
        [_stable_float(value) for value in np.linspace(0.0, 1.0, base_points)],
        dtype=float,
    )
    present = bool(np.any(np.isclose(base, alpha, rtol=0.0, atol=1.0e-12)))
    if present:
        phase = base
    else:
        phase = np.sort(np.concatenate((base, np.asarray([alpha], dtype=float))))
    if phase[0] != 0.0 or phase[-1] != 1.0 or np.any(np.diff(phase) <= 0.0):
        raise ValueError("constructed phase grid is invalid")
    return phase, not present


def _warped_sample_times(
    cycle: tuple[float, float, float],
    phase: np.ndarray,
    alpha: float,
) -> np.ndarray:
    heel_strike, next_heel_strike, contact_duration = cycle
    toe_off = heel_strike + contact_duration
    if not heel_strike < toe_off < next_heel_strike:
        raise ValueError(
            "invalid HS->TO->HS cycle: "
            f"{heel_strike}, {toe_off}, {next_heel_strike}"
        )
    stance = phase <= alpha
    sample_times = np.empty_like(phase)
    sample_times[stance] = heel_strike + (phase[stance] / alpha) * (
        toe_off - heel_strike
    )
    sample_times[~stance] = toe_off + (
        (phase[~stance] - alpha) / (1.0 - alpha)
    ) * (next_heel_strike - toe_off)
    return sample_times


def event_warp_cycles(
    ik_time: np.ndarray,
    coordinate_values_rad: np.ndarray,
    cycles: Sequence[tuple[float, float, float]],
    phase_grid: np.ndarray,
    canonical_to_phase: float,
) -> np.ndarray:
    """Resample one IK coordinate for each cycle on the event-warped grid.

    Returns an array shaped ``(n_cycles, n_phase_points)``.  Input coordinate
    values must already be in radians, as returned by :func:`load_inputs`.
    """
    times = np.asarray(ik_time, dtype=float)
    values = np.asarray(coordinate_values_rad, dtype=float)
    phase = np.asarray(phase_grid, dtype=float)
    alpha = float(canonical_to_phase)
    if times.ndim != 1 or values.ndim != 1 or times.size != values.size:
        raise ValueError("IK time and coordinate values must be equal-length vectors")
    if times.size < 2 or np.any(np.diff(times) <= 0.0):
        raise ValueError("IK time must be strictly increasing")
    if not np.all(np.isfinite(times)) or not np.all(np.isfinite(values)):
        raise ValueError("IK time/coordinate values contain non-finite data")
    if phase.ndim != 1 or phase.size < 2 or not np.all(np.isfinite(phase)):
        raise ValueError("phase_grid must be a finite vector with at least two points")
    if (
        not np.isclose(phase[0], 0.0, rtol=0.0, atol=1.0e-12)
        or not np.isclose(phase[-1], 1.0, rtol=0.0, atol=1.0e-12)
        or np.any(np.diff(phase) <= 0.0)
    ):
        raise ValueError("phase_grid must be strictly increasing from 0 to 1")
    if not 0.0 < alpha < 1.0:
        raise ValueError("canonical_to_phase must lie strictly inside (0, 1)")
    if not cycles:
        raise ValueError("at least one cycle is required")

    rows: list[np.ndarray] = []
    for cycle in cycles:
        if cycle[0] < float(times[0]) - 1.0e-12:
            raise ValueError(f"cycle starts before IK coverage: {cycle[0]}")
        if cycle[1] > float(times[-1]) + 1.0e-12:
            raise ValueError(f"cycle ends after IK coverage: {cycle[1]}")
        sample_times = _warped_sample_times(cycle, phase, alpha)
        rows.append(np.interp(sample_times, times, values))
    return np.vstack(rows)


def _profile_bytes(profile: dict[str, Any]) -> bytes:
    text = json.dumps(
        profile,
        indent=2,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=False,
    )
    return (text + "\n").encode("utf-8")


def build_profile(
    *,
    ik: str | Path = DEFAULT_IK,
    grf: str | Path = DEFAULT_GRF,
    output: str | Path = DEFAULT_OUTPUT,
    coordinates: Sequence[str] = DEFAULT_COORDINATES,
    grf_column: str = "ground_force1_vy",
    threshold_n: float = 20.0,
    min_contact_duration_s: float = 0.05,
    min_cycle_duration_s: float = 0.30,
    t_start: float | None = None,
    t_end: float = 153.0,
    base_grid_points: int = 201,
    expected_cycles: int | None = 123,
    ik_units: str = "auto",
    check: bool = False,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build the deterministic event-warped profile and its run summary.

    This pure construction entry point does not write.  The encoded bytes and
    resolved output path are returned in private summary fields for the CLI to
    handle; callers interested only in data can use the first tuple element.
    """
    inputs = load_inputs(
        ik,
        grf,
        coordinates=coordinates,
        grf_column=grf_column,
        ik_units=ik_units,
    )
    ik_path = inputs["ik_path"]
    grf_path = inputs["grf_path"]
    output_path = resolve_repo_path(output).resolve()
    coordinate_names = inputs["coordinates"]
    ik_time = inputs["ik_time"]
    grf_time = inputs["grf_time"]
    header_in_degrees = inputs["ik_header_in_degrees"]
    ik_in_degrees = inputs["ik_in_degrees"]

    detection = detect_cycles(
        grf_time,
        inputs["vertical_grf_n"],
        threshold_n=threshold_n,
        t_start=t_start,
        t_end=t_end,
        min_contact_duration_s=min_contact_duration_s,
        min_cycle_duration_s=min_cycle_duration_s,
        expected_cycles=expected_cycles,
    )
    production_cycles = detection["production_cycles"]
    full_span_cycles = detection["full_span_cycles"]
    excluded_cycles = detection["excluded_cycles"]
    t_start = detection["t_start"]
    t_end = detection["t_end"]

    periods = np.asarray([end - start for start, end, _ in production_cycles])
    stance_durations = np.asarray([contact for _, _, contact in production_cycles])
    swing_durations = periods - stance_durations
    duty_factors = stance_durations / periods
    alpha = _stable_float(np.mean(duty_factors))
    if not 0.0 < alpha < 1.0:
        raise ValueError(f"canonical toe-off phase is outside (0, 1): {alpha}")
    phase, alpha_inserted = _phase_grid(base_grid_points, alpha)

    profile_coordinates: dict[str, dict[str, list[float]]] = {}
    coordinate_metadata: dict[str, dict[str, float]] = {}
    for coordinate in coordinate_names:
        cycle_values = event_warp_cycles(
            ik_time,
            inputs["coordinate_values_rad"][coordinate],
            production_cycles,
            phase,
            alpha,
        )
        mean = np.mean(cycle_values, axis=0)
        std = np.std(cycle_values, axis=0, ddof=0)
        if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
            raise ValueError(f"non-finite profile values for {coordinate}")
        profile_coordinates[coordinate] = {
            "mean_rad": _stable_array(mean),
            "std_rad": _stable_array(std),
        }
        coordinate_metadata[coordinate] = {
            "mean_min_rad": _stable_float(np.min(mean)),
            "mean_max_rad": _stable_float(np.max(mean)),
            "std_min_rad": _stable_float(np.min(std)),
            "std_mean_rad": _stable_float(np.mean(std)),
            "std_max_rad": _stable_float(np.max(std)),
        }

    first_cycle = production_cycles[0]
    last_cycle = production_cycles[-1]
    profile: dict[str, Any] = {
        "version": 2,
        "name": "ab06_prosthetic_event_warped_mean_std_corridor",
        "description": (
            "AB06 left/prosthetic knee and ankle mean/std corridor with "
            "piecewise HS-to-TO and TO-to-HS event warping."
        ),
        "units": "radian",
        "phase_grid": _stable_array(phase),
        "coordinates": profile_coordinates,
        "metadata": {
            # Backward-compatible summary fields used by existing diagnostics.
            "ik_source": _portable_path(ik_path),
            "grf_source": _portable_path(grf_path),
            "grf_vertical_column": grf_column,
            "grf_threshold_n": _stable_float(threshold_n),
            "min_contact_duration_s": _stable_float(min_contact_duration_s),
            "min_cycle_duration_s": _stable_float(min_cycle_duration_s),
            "n_cycles": int(len(production_cycles)),
            "mean_period_s": _stable_float(np.mean(periods)),
            "mean_to_phase": alpha,
            "std_to_phase": _stable_float(np.std(duty_factors, ddof=0)),
            "first_cycle_start_s": _stable_float(first_cycle[0]),
            "last_cycle_end_s": _stable_float(last_cycle[1]),
            # Event-warped phase contract.
            "phase_parameterization": "event_warped_hs_to_to_to_hs_v1",
            "canonical_to_phase": alpha,
            "canonical_to_phase_method": "mean_production_cycle_duty_factor",
            "laterality": {
                "prosthetic_side": "left",
                "event_detection_grf_side": "left",
            },
            "coordinate_conventions": {
                "pros_knee_angle": {
                    "runtime_units": "radian",
                    "runtime_sign": "OpenSim model coordinate sign",
                    "display_convention": "multiply by -1 for flexion-positive plots",
                },
                "pros_ankle_angle": {
                    "runtime_units": "radian",
                    "runtime_sign": "OpenSim model coordinate sign",
                    "display_convention": "identity",
                },
            },
            "phase_grid_base": "uniform_0_to_1_inclusive",
            "phase_grid_base_points": int(base_grid_points),
            "phase_grid_alpha_inserted": bool(alpha_inserted),
            "phase_grid_points": int(phase.size),
            "cycle_resampling": {
                "stance": "HS->TO mapped linearly to [0, canonical_to_phase]",
                "swing": "TO->next_HS mapped linearly to [canonical_to_phase, 1]",
                "interpolation": "numpy.interp linear",
            },
            "aggregation": {
                "mean": "arithmetic mean across production cycles",
                "std": "population standard deviation across production cycles",
                "std_ddof": 0,
                "output_round_digits": FLOAT_DIGITS,
            },
            "cycle_selection": {
                "production_t_start_s": _stable_float(t_start),
                "production_t_end_s": _stable_float(t_end),
                "rule": (
                    "complete sustained-contact HS->TO->HS cycles whose HS and "
                    "next HS lie inside the production window"
                ),
                "production_cycle_count": int(len(production_cycles)),
                "full_grf_span_complete_cycle_count": int(len(full_span_cycles)),
                "excluded_full_span_complete_cycle_count": int(
                    len(excluded_cycles)
                ),
                "excluded_full_span_complete_cycles": excluded_cycles,
                "note": (
                    "The default t_end=153 s intentionally excludes the next "
                    "otherwise-complete AB06 cycle because that cycle extends "
                    "beyond the production window."
                ),
            },
            "cycle_statistics": {
                "period_s": _series_stats(periods),
                "stance_duration_s": _series_stats(stance_durations),
                "swing_duration_s": _series_stats(swing_durations),
                "duty_factor": _series_stats(duty_factors),
            },
            "coordinate_statistics": coordinate_metadata,
            "source_data": {
                "ik_sha256": _sha256(ik_path),
                "grf_sha256": _sha256(grf_path),
                "ik_rows": int(ik_time.size),
                "grf_rows": int(grf_time.size),
                "ik_time_start_s": _stable_float(ik_time[0]),
                "ik_time_end_s": _stable_float(ik_time[-1]),
                "grf_time_start_s": _stable_float(grf_time[0]),
                "grf_time_end_s": _stable_float(grf_time[-1]),
                "ik_sample_interval": _sample_interval_stats(ik_time),
                "grf_sample_interval": _sample_interval_stats(grf_time),
                "ik_header_in_degrees": header_in_degrees,
                "ik_units_interpreted_as": (
                    "degrees" if ik_in_degrees else "radians"
                ),
            },
            "builder": _portable_path(Path(__file__)),
        },
    }

    encoded = _profile_bytes(profile)
    existing = output_path.read_bytes() if output_path.is_file() else None
    summary: dict[str, Any] = {
        "ok": True,
        "mode": "check" if check else "write",
        "output": _portable_path(output_path),
        "output_exists": existing is not None,
        "matches_existing": (
            None if existing is None else bool(existing == encoded)
        ),
        "profile_name": profile["name"],
        "profile_sha256": hashlib.sha256(encoded).hexdigest(),
        "production_cycles": int(len(production_cycles)),
        "full_span_complete_cycles": int(len(full_span_cycles)),
        "excluded_full_span_complete_cycles": int(len(excluded_cycles)),
        "canonical_to_phase": alpha,
        "phase_grid_base_points": int(base_grid_points),
        "phase_grid_points": int(phase.size),
        "canonical_to_phase_inserted": bool(alpha_inserted),
        "production_first_hs_s": _stable_float(first_cycle[0]),
        "production_last_next_hs_s": _stable_float(last_cycle[1]),
    }
    summary["_encoded_profile"] = encoded
    summary["_output_path"] = output_path
    return profile, summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ik", default=DEFAULT_IK, help="IK Storage/.mot input.")
    parser.add_argument("--grf", default=DEFAULT_GRF, help="GRF Storage/.mot input.")
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help="New event-warped morphology profile JSON destination.",
    )
    parser.add_argument(
        "--coordinates",
        nargs="+",
        default=list(DEFAULT_COORDINATES),
        help="IK coordinate columns to aggregate.",
    )
    parser.add_argument("--grf-column", default="ground_force1_vy")
    parser.add_argument("--threshold-n", type=float, default=20.0)
    parser.add_argument("--min-contact-duration-s", type=float, default=0.05)
    parser.add_argument("--min-cycle-duration-s", type=float, default=0.30)
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Production-window start; defaults to the first GRF sample.",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=153.0,
        help=(
            "Production-window end. Default 153 s intentionally excludes the "
            "next complete AB06 cycle extending beyond this cutoff."
        ),
    )
    parser.add_argument("--base-grid-points", type=int, default=201)
    parser.add_argument(
        "--expected-cycles",
        type=int,
        default=123,
        help="Expected production count; use 0 to disable the count assertion.",
    )
    parser.add_argument(
        "--ik-units",
        choices=("auto", "degrees", "radians"),
        default="auto",
        help="IK rotational units; auto reads the inDegrees header.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help=(
            "Build and validate deterministically without creating directories "
            "or writing the output profile."
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    _profile, summary = build_profile(**vars(args))
    encoded = summary.pop("_encoded_profile")
    output_path = summary.pop("_output_path")

    if not args.check:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(encoded)
        summary["bytes_written"] = int(len(encoded))

    print(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
            ensure_ascii=True,
            allow_nan=False,
        )
    )
    if args.check and summary["matches_existing"] is False:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
