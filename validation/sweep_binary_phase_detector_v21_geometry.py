"""Development-only V21 coarse-to-fine sweep for binary detector geometry.

The sweep varies only mesh-anchored heel/toe point geometry.  The V19 ground
plane, binary contact rule, 1 ms sampling, and V20 5 ms FSM are frozen.  Only
DEV02/04 are accessible; trial 08, protected trials, and reserve trials are
hard-closed.

Running the numerical sweep requires an explicit ``--execute`` and a new
``--output-dir`` under ``validation/``.  No active profile/configuration is
modified and no candidate is called eligible unless every frozen V20 gate
passes in the final scalar/batch verification.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
VALIDATION_ROOT = REPO_ROOT / "validation"
for import_root in (REPO_ROOT, TRAJECTORY_ROOT, VALIDATION_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))


SCHEMA_VERSION = 21
SWEEP_ID = "AB06_BINARY_POINT_GEOMETRY_V21_COARSE_TO_FINE_DEV02_04"
TRIALS = ("02", "04")
FORBIDDEN_TRIALS = ("01", "03", "05", "06", "07", "08")
SAMPLE_DT_S = 0.001
POLICY_SAMPLES = 10
DEBOUNCE_SAMPLES = 5  # elapsed 5 ms: onset i confirms at i+5.
MIN_STABLE_RUN_SAMPLES = DEBOUNCE_SAMPLES + 1

V19_PROFILE_PATH = (
    VALIDATION_ROOT
    / "experimental_detector_profiles/two_point_binary_v19_outsole_25mm.json"
)
V19_PROFILE_SHA256 = (
    "fddb17f7bd24e004504de662676d7b5a2cb9e5d0fda77de8dea2664c0b5c7a86"
)
MESH_PATH = REPO_ROOT / "Geometry/AM_foot_l.STL"
MESH_SHA256 = "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9"
FOOT_FRAME = "/bodyset/foot_l"
GROUND_ORIGIN = (0.0, 0.0148208231, 0.0)
GROUND_NORMAL = (0.0, 1.0, 0.0)

# Preserve the V19 lateral placement as a fraction of each constant-x mesh
# section.  The toe is the section midpoint; the heel retains its V19 offset.
LATERAL_FRACTIONS = {
    "left_heel": 0.8030205847202724,
    "left_toe": 0.5,
}
V19_BASE_X_M = {
    "left_heel": -0.0946600475,
    "left_toe": 0.11574858501553537,
}
V19_REACH_M = 0.025
V17_SPHERE_BOTTOM_EQUIVALENT_REACH_M = {
    "left_heel": 0.013319582186,
    "left_toe": 0.03575,
}

X_FRACTION_BOUNDS = {
    "left_heel": (0.02, 0.20),
    "left_toe": (0.62, 0.92),
}
REACH_BOUNDS_M = {
    "left_heel": (0.005, 0.035),
    "left_toe": (0.015, 0.045),
}

EXPECTED_V19_BIT_TRACE_SHA256 = {
    "02": "6b1529b6db7e0b4ebfbd6eafc4f8581ac8ee9fb3a88f34da701f36fee0bd5731",
    "04": "99ffe2216e4184d82392fe3a9be951ebf3d709cdf8c7f0974dd3a7e75728f4d9",
}

PINNED_SOURCES = {
    "validation/validate_binary_phase_detector_v19_raw_geometry.py": (
        "0e16fba4a26b5910d38004c0c56a98435e9bc889656ea728b831f158d30a0eea"
    ),
    "validation/validate_binary_phase_fsm_v20_development.py": (
        "1ce0b82a4e9db3b2d90b4dc132de798f8eb452e784fe2492aa76ccbe10e8e431"
    ),
    "Trajectory Generator/binary_phase_fsm.py": (
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1"
    ),
    "binary_phase_detector.py": (
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6"
    ),
    "validation/binary_phase_detector_v19_geometry_receipt.json": (
        "7d93ef5f5b5c877246025c0f6d7a607783621e7019065ca63b4b889f5a30e0ff"
    ),
    "validation/binary_phase_fsm_v20_development_receipt.json": (
        "43fe41ba938b020f75604a5c21dbe433a12bc3e8233be47f98bc697d9ac41e2c"
    ),
    "validation/build_two_sensor_mesh_profile_v4.py": (
        "671b59ca546112b720b915816a89eec14eaf3dd3cf52a9a37388074b23ef4bda"
    ),
    "validation/audit_two_sensor_prescribed_geometry.py": (
        "c82d114ab8550d6963f394a2cfa99decf94a992880e751792ac56af8dd76a307"
    ),
    "validation/build_canonical_grf_event_oracle.py": (
        "246c7cb326c209fe5bf732e3c5b2d3a9125b33d3cb3208378f7ded0bd7c40a89"
    ),
    "validation/canonical_event_oracles/2026-08-03_v17_development/"
    "trial_02_canonical_event_ledger.json": (
        "acfb502bd742055dda49ae9f5398900f87f33368c434e986c98fae127c98894d"
    ),
    "validation/canonical_event_oracles/2026-08-03_v17_development/"
    "trial_04_canonical_event_ledger.json": (
        "4f48813bd8c6bd5117cd52926e9dc921b01296dc548c0d51f3799171d398f813"
    ),
    "kinematics_interpolator.py": (
        "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede"
    ),
    "config.py": (
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3"
    ),
    "model_loader.py": (
        "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76"
    ),
}

PINNED_TRIAL_INPUTS = {
    "02": {
        "lock_sha256": "5b086a1a49e6dc8cf893ec43b6f5b1fa1dddb84bfffcb4d216a9287208cb472d",
        "ik_sha256": "4018b3001a5d293edda799839158f4c154747af5c908a2eb530dae3e37e5a982",
    },
    "04": {
        "lock_sha256": "e616ef95bae42ac6e99060abca1dc4a323ddaae1b68b01dcaafce36c407d6804",
        "ik_sha256": "ade6b105d2f6f3cdb350852f92caa48b09cdfa8cd769bf5c02ef5fbdd7b7218f",
    },
}
PINNED_MODEL_SHA256 = (
    "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d"
)
PINNED_PLUGIN_SHA256 = (
    "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b"
)

DEFAULT_HEEL_X_FRACTIONS = "0.03,0.08,0.13,0.18"
DEFAULT_TOE_X_FRACTIONS = "0.62,0.70,0.78,0.86,0.92"
DEFAULT_HEEL_REACH_MM = "8,13.319582186,18,23,25,28"
DEFAULT_TOE_REACH_MM = "20,25,30,35.75,40"
DEFAULT_FINE_X_RADIUS_MM = 12.0
DEFAULT_FINE_REACH_RADIUS_MM = 3.0
DEFAULT_FINE_SEED_MIN_DISTANCE = 1.0


class V21SweepError(RuntimeError):
    """Raised when the V21 sweep cannot preserve its frozen contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def format_duration(seconds: float | None) -> str:
    """Render a nonnegative duration as HH:MM:SS or an unknown placeholder."""

    if seconds is None or not math.isfinite(seconds) or seconds < 0.0:
        return "--:--:--"
    total = int(round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


class SweepProgress:
    """Zero-dependency ASCII progress bar with elapsed time and ETA.

    TTY streams are updated in place. Redirected streams receive an initial
    line, periodic newline snapshots, and a forced final line. Rendering errors
    are swallowed so cosmetic output can never invalidate a scientific run.
    """

    def __init__(
        self,
        *,
        total: int,
        label: str,
        stream: TextIO | None = None,
        width: int = 24,
        min_redraw_interval_s: float = 0.10,
        non_tty_interval_s: float = 30.0,
        clock: Callable[[], float] = time.monotonic,
        enabled: bool = True,
    ) -> None:
        if type(total) is not int or total <= 0:
            raise ValueError("progress total must be a positive integer")
        if type(width) is not int or width <= 0:
            raise ValueError("progress width must be a positive integer")
        self.total = total
        self.label = str(label)
        self.stream = stream if stream is not None else sys.stderr
        self.width = width
        self.min_redraw_interval_s = float(min_redraw_interval_s)
        self.non_tty_interval_s = float(non_tty_interval_s)
        self.clock = clock
        self.enabled = bool(enabled)
        self.completed = 0
        self.start_time = float(self.clock())
        self.last_render_time: float | None = None
        self.last_line_length = 0
        self.finished = False
        try:
            self.is_tty = bool(self.stream.isatty())
        except Exception:
            self.is_tty = False
        self.render(force=True)

    def _line(self, *, status: str | None = None) -> str:
        now = float(self.clock())
        elapsed = max(0.0, now - self.start_time)
        fraction = self.completed / self.total
        filled = int(round(max(0.0, min(1.0, fraction)) * self.width))
        bar = "#" * filled + "-" * (self.width - filled)
        eta = None
        if self.completed > 0:
            eta = elapsed / self.completed * (self.total - self.completed)
        parts = [
            f"{self.label} [{bar}]",
            f"{self.completed}/{self.total}",
            f"{100.0 * fraction:6.2f}%",
            f"time elapsed {format_duration(elapsed)}",
            f"ETA {format_duration(eta)}",
        ]
        if status:
            parts.append(str(status))
        return " | ".join(parts)

    def render(self, *, force: bool = False, status: str | None = None) -> None:
        if not self.enabled or self.finished:
            return
        try:
            now = float(self.clock())
            interval = (
                self.min_redraw_interval_s
                if self.is_tty
                else self.non_tty_interval_s
            )
            if (
                not force
                and self.last_render_time is not None
                and now - self.last_render_time < interval
            ):
                return
            line = self._line(status=status)
            if self.is_tty:
                padding = " " * max(0, self.last_line_length - len(line))
                self.stream.write("\r" + line + padding)
                self.last_line_length = len(line)
            else:
                self.stream.write(line + "\n")
            self.stream.flush()
            self.last_render_time = now
        except Exception:
            return

    def update(self, completed: int) -> None:
        if type(completed) is not int:
            raise TypeError("progress completed must be an integer")
        if completed < self.completed or completed > self.total:
            raise ValueError("progress completed must be monotonic and in range")
        self.completed = completed
        # The caller closes the bar with finish(), which is responsible for
        # the single forced terminal frame (and its DONE/FAILED status).  Do
        # not force here or redirected logs would receive two 100% snapshots.
        self.render()

    def advance(self, amount: int = 1) -> None:
        if type(amount) is not int or amount < 0:
            raise ValueError("progress advance must be a nonnegative integer")
        self.update(self.completed + amount)

    def finish(self, status: str = "DONE") -> None:
        if self.finished:
            return
        self.render(force=True, status=status)
        if self.enabled and self.is_tty:
            try:
                self.stream.write("\n")
                self.stream.flush()
            except Exception:
                pass
        self.finished = True


@dataclass(frozen=True)
class AffineTrialTrace:
    trial_id: str
    time_s: np.ndarray
    normal_coefficients: np.ndarray
    normal_offset: np.ndarray
    source: Mapping[str, Any]


@dataclass(frozen=True)
class PointGeometry:
    role: str
    x_m: float
    reach_m: float
    location_m: tuple[float, float, float]
    surface_location_m: tuple[float, float, float]
    lateral_fraction: float

    def payload(self) -> dict[str, Any]:
        return {
            "role": self.role,
            "x_m": self.x_m,
            "reach_m": self.reach_m,
            "location_m": list(self.location_m),
            "surface_location_m": list(self.surface_location_m),
            "lateral_fraction": self.lateral_fraction,
        }


@dataclass(frozen=True)
class GeometryCandidate:
    heel: PointGeometry
    toe: PointGeometry
    stage: str
    parent_id: str | None = None

    @property
    def candidate_id(self) -> str:
        core = {
            "heel_x_m": round(self.heel.x_m, 12),
            "heel_reach_m": round(self.heel.reach_m, 12),
            "toe_x_m": round(self.toe.x_m, 12),
            "toe_reach_m": round(self.toe.reach_m, 12),
        }
        return "v21_" + canonical_sha256(core)[:16]

    @property
    def key(self) -> tuple[float, float, float, float]:
        return (
            round(self.heel.x_m, 12),
            round(self.heel.reach_m, 12),
            round(self.toe.x_m, 12),
            round(self.toe.reach_m, 12),
        )

    def payload(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "stage": self.stage,
            "parent_id": self.parent_id,
            "heel": self.heel.payload(),
            "toe": self.toe.payload(),
            "comparators": {
                "v19_common_25mm": bool(
                    abs(self.heel.x_m - V19_BASE_X_M["left_heel"]) <= 1e-12
                    and abs(self.toe.x_m - V19_BASE_X_M["left_toe"]) <= 1e-12
                    and abs(self.heel.reach_m - V19_REACH_M) <= 1e-12
                    and abs(self.toe.reach_m - V19_REACH_M) <= 1e-12
                ),
                "v17_sphere_bottom_equivalent": bool(
                    abs(self.heel.x_m - V19_BASE_X_M["left_heel"]) <= 1e-12
                    and abs(self.toe.x_m - V19_BASE_X_M["left_toe"]) <= 1e-12
                    and abs(
                        self.heel.reach_m
                        - V17_SPHERE_BOTTOM_EQUIVALENT_REACH_M["left_heel"]
                    )
                    <= 1e-12
                    and abs(
                        self.toe.reach_m
                        - V17_SPHERE_BOTTOM_EQUIVALENT_REACH_M["left_toe"]
                    )
                    <= 1e-12
                ),
            },
        }


def _parse_csv_floats(raw: str, *, label: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in str(raw).split(","))
    except (TypeError, ValueError) as exc:
        raise V21SweepError(f"{label} must be a comma-separated float list") from exc
    if not values or any(not math.isfinite(item) for item in values):
        raise V21SweepError(f"{label} must contain finite values")
    return tuple(sorted(set(round(item, 12) for item in values)))


def _grid_values(center: float, radius: float, step: float) -> tuple[float, ...]:
    if not all(math.isfinite(item) for item in (center, radius, step)):
        raise V21SweepError("fine-grid parameters must be finite")
    if radius < 0.0 or step <= 0.0:
        raise V21SweepError("fine-grid radius/step must be nonnegative/positive")
    count = int(math.floor(radius / step + 1e-12))
    offsets = [index * step for index in range(-count, count + 1)]
    if not any(abs(item) <= 1e-15 for item in offsets):
        offsets.append(0.0)
    return tuple(sorted(set(round(center + item, 12) for item in offsets)))


class MeshPointFactory:
    """Derive mesh-anchored point locations from x and plantar reach."""

    def __init__(
        self,
        triangles: np.ndarray,
        *,
        section_z_bounds: Callable[[np.ndarray, float], tuple[float, float]],
        vertical_y_intersections: Callable[[np.ndarray, float, float], list[float]],
    ) -> None:
        self.triangles = np.asarray(triangles, dtype=float)
        vertices = self.triangles.reshape(-1, 3)
        self.x_min = float(np.min(vertices[:, 0]))
        self.x_max = float(np.max(vertices[:, 0]))
        self._section_z_bounds = section_z_bounds
        self._vertical_y_intersections = vertical_y_intersections
        self._cache: dict[tuple[str, float, float], PointGeometry] = {}

    def x_from_fraction(self, fraction: float) -> float:
        if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
            raise V21SweepError("mesh x fraction must be finite in [0, 1]")
        return float(self.x_min + fraction * (self.x_max - self.x_min))

    def x_fraction(self, x_m: float) -> float:
        return float((x_m - self.x_min) / (self.x_max - self.x_min))

    def build(self, role: str, x_m: float, reach_m: float) -> PointGeometry:
        if role not in LATERAL_FRACTIONS:
            raise V21SweepError(f"unsupported detector role: {role}")
        x_m = round(float(x_m), 12)
        reach_m = round(float(reach_m), 12)
        if not math.isfinite(x_m) or not math.isfinite(reach_m):
            raise V21SweepError("point geometry must be finite")
        x_fraction = self.x_fraction(x_m)
        x_low, x_high = X_FRACTION_BOUNDS[role]
        # Candidate coordinates are canonicalized to picometres before they
        # become cache/receipt keys.  Allow the corresponding sub-nanometric
        # fraction error at an inclusive anatomical-band boundary.
        if not x_low - 1e-9 <= x_fraction <= x_high + 1e-9:
            raise V21SweepError(
                f"{role} x={x_m:.9g} leaves anatomical fraction band "
                f"[{x_low}, {x_high}]"
            )
        reach_low, reach_high = REACH_BOUNDS_M[role]
        if not reach_low - 1e-12 <= reach_m <= reach_high + 1e-12:
            raise V21SweepError(
                f"{role} reach={reach_m:.9g} leaves physical bounds "
                f"[{reach_low}, {reach_high}]"
            )
        key = (role, x_m, reach_m)
        if key in self._cache:
            return self._cache[key]
        z_min, z_max = self._section_z_bounds(self.triangles, x_m)
        lateral_fraction = LATERAL_FRACTIONS[role]
        z_m = float(z_min + lateral_fraction * (z_max - z_min))
        intersections = self._vertical_y_intersections(
            self.triangles, x_m, z_m
        )
        surface_y_m = float(min(intersections))
        surface = (x_m, surface_y_m, z_m)
        location = (x_m, surface_y_m - reach_m, z_m)
        if not np.all(np.isfinite(location)):
            raise V21SweepError(f"{role} derived a non-finite point")
        point = PointGeometry(
            role=role,
            x_m=x_m,
            reach_m=reach_m,
            location_m=location,
            surface_location_m=surface,
            lateral_fraction=lateral_fraction,
        )
        self._cache[key] = point
        return point


def generate_coarse_candidates(
    factory: MeshPointFactory,
    *,
    heel_x_fractions: Sequence[float],
    toe_x_fractions: Sequence[float],
    heel_reaches_m: Sequence[float],
    toe_reaches_m: Sequence[float],
) -> list[GeometryCandidate]:
    heel_x = {
        factory.x_from_fraction(float(value)) for value in heel_x_fractions
    }
    toe_x = {
        factory.x_from_fraction(float(value)) for value in toe_x_fractions
    }
    heel_x.add(V19_BASE_X_M["left_heel"])
    toe_x.add(V19_BASE_X_M["left_toe"])
    heel_reach = {float(value) for value in heel_reaches_m}
    toe_reach = {float(value) for value in toe_reaches_m}
    heel_reach.update(
        {
            V19_REACH_M,
            V17_SPHERE_BOTTOM_EQUIVALENT_REACH_M["left_heel"],
        }
    )
    toe_reach.update(
        {
            V19_REACH_M,
            V17_SPHERE_BOTTOM_EQUIVALENT_REACH_M["left_toe"],
        }
    )
    heels = [
        factory.build("left_heel", x_m, reach_m)
        for x_m, reach_m in itertools.product(sorted(heel_x), sorted(heel_reach))
    ]
    toes = [
        factory.build("left_toe", x_m, reach_m)
        for x_m, reach_m in itertools.product(sorted(toe_x), sorted(toe_reach))
    ]
    return [
        GeometryCandidate(heel=heel, toe=toe, stage="coarse")
        for heel, toe in itertools.product(heels, toes)
    ]


def generate_fine_candidates(
    factory: MeshPointFactory,
    seeds: Sequence[GeometryCandidate],
    *,
    x_radius_m: float,
    x_step_m: float,
    reach_radius_m: float,
    reach_step_m: float,
    excluded_keys: set[tuple[float, float, float, float]],
) -> list[GeometryCandidate]:
    result: dict[tuple[float, float, float, float], GeometryCandidate] = {}
    for seed in seeds:
        axes = (
            _grid_values(seed.heel.x_m, x_radius_m, x_step_m),
            _grid_values(seed.heel.reach_m, reach_radius_m, reach_step_m),
            _grid_values(seed.toe.x_m, x_radius_m, x_step_m),
            _grid_values(seed.toe.reach_m, reach_radius_m, reach_step_m),
        )
        for heel_x, heel_reach, toe_x, toe_reach in itertools.product(*axes):
            try:
                candidate = GeometryCandidate(
                    heel=factory.build("left_heel", heel_x, heel_reach),
                    toe=factory.build("left_toe", toe_x, toe_reach),
                    stage="fine",
                    parent_id=seed.candidate_id,
                )
            except V21SweepError:
                continue
            if candidate.key not in excluded_keys:
                result.setdefault(candidate.key, candidate)
    return sorted(result.values(), key=lambda item: item.candidate_id)


def point_contact_bits(trace: AffineTrialTrace, point: PointGeometry) -> np.ndarray:
    location = np.asarray(point.location_m, dtype=float)
    clearance = trace.normal_coefficients @ location + trace.normal_offset
    if clearance.shape != trace.time_s.shape or not np.all(np.isfinite(clearance)):
        raise V21SweepError(
            f"trial {trace.trial_id} produced malformed affine clearance"
        )
    return np.asarray(clearance <= 0.0, dtype=bool)


def bit_trace_sha256(heel: np.ndarray, toe: np.ndarray) -> str:
    heel = np.asarray(heel, dtype=bool)
    toe = np.asarray(toe, dtype=bool)
    if heel.shape != toe.shape or heel.ndim != 1:
        raise V21SweepError("bit trace channels must be aligned vectors")
    packed = np.packbits(
        np.column_stack((heel, toe)).reshape(-1), bitorder="little"
    )
    return hashlib.sha256(packed.tobytes()).hexdigest()


def _maximum_run_samples(mask: np.ndarray) -> int:
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 1 or values.size == 0:
        raise V21SweepError("run-length input must be a nonempty vector")
    padded = np.r_[False, values, False]
    starts = np.flatnonzero(padded[1:] & ~padded[:-1])
    ends = np.flatnonzero(~padded[1:] & padded[:-1])
    return int(np.max(ends - starts)) if starts.size else 0


def two_sensor_channel_gate(
    heel: np.ndarray, toe: np.ndarray
) -> dict[str, Any]:
    """Require two independently informative, debounce-stable raw channels."""

    heel_bits = np.asarray(heel, dtype=bool)
    toe_bits = np.asarray(toe, dtype=bool)
    if (
        heel_bits.ndim != 1
        or heel_bits.size == 0
        or toe_bits.shape != heel_bits.shape
    ):
        raise V21SweepError("two-sensor gate requires aligned nonempty vectors")
    masks = {
        "heel_on": heel_bits,
        "heel_off": ~heel_bits,
        "toe_on": toe_bits,
        "toe_off": ~toe_bits,
        "heel_only": heel_bits & ~toe_bits,
        "toe_only": toe_bits & ~heel_bits,
    }
    maximum_runs = {
        name: _maximum_run_samples(mask) for name, mask in masks.items()
    }
    requirement_pass = {
        name: count >= MIN_STABLE_RUN_SAMPLES
        for name, count in maximum_runs.items()
    }
    return {
        "pass": bool(all(requirement_pass.values())),
        "minimum_stable_run_samples": MIN_STABLE_RUN_SAMPLES,
        "minimum_stable_run_s": (
            MIN_STABLE_RUN_SAMPLES - 1
        ) * SAMPLE_DT_S,
        "maximum_run_samples": maximum_runs,
        "requirements_pass": requirement_pass,
    }


def two_sensor_view_gate(
    time_s: np.ndarray,
    heel: np.ndarray,
    toe: np.ndarray,
    views: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Apply the two-channel gate independently to every speed plateau."""

    times = np.asarray(time_s, dtype=float)
    heel_bits = np.asarray(heel, dtype=bool)
    toe_bits = np.asarray(toe, dtype=bool)
    if (
        times.ndim != 1
        or times.size == 0
        or heel_bits.shape != times.shape
        or toe_bits.shape != times.shape
        or not np.all(np.isfinite(times))
        or not np.all(np.diff(times) > 0.0)
    ):
        raise V21SweepError("view gate requires aligned monotonic traces")
    by_view: dict[str, dict[str, Any]] = {}
    for view in views:
        view_id = str(view["view_id"])
        interval = view["interval_s"]
        if (
            not isinstance(interval, Sequence)
            or len(interval) != 2
            or not all(math.isfinite(float(value)) for value in interval)
            or float(interval[0]) >= float(interval[1])
        ):
            raise V21SweepError(f"view {view_id} has an invalid interval")
        mask = (times >= float(interval[0]) - 1e-12) & (
            times <= float(interval[1]) + 1e-12
        )
        if int(np.count_nonzero(mask)) < MIN_STABLE_RUN_SAMPLES:
            raise V21SweepError(f"view {view_id} has too few detector samples")
        record = two_sensor_channel_gate(heel_bits[mask], toe_bits[mask])
        record["interval_s"] = [float(interval[0]), float(interval[1])]
        by_view[view_id] = record
    return {
        "pass": bool(by_view and all(item["pass"] for item in by_view.values())),
        "full_trace": two_sensor_channel_gate(heel_bits, toe_bits),
        "views": by_view,
    }


def fast_fsm_events(
    time_s: np.ndarray,
    heel: np.ndarray,
    toe: np.ndarray,
) -> list[dict[str, Any]]:
    """Exact RLE screen for the frozen V20 aggregate-contact FSM semantics."""

    times = np.asarray(time_s, dtype=float)
    heel_bits = np.asarray(heel, dtype=bool)
    toe_bits = np.asarray(toe, dtype=bool)
    if (
        times.ndim != 1
        or times.size < 2
        or heel_bits.shape != times.shape
        or toe_bits.shape != times.shape
        or not np.all(np.isfinite(times))
        or not np.all(np.diff(times) > 0.0)
    ):
        raise V21SweepError("fast FSM requires aligned finite monotonic traces")
    raw_contact = heel_bits | toe_bits
    changes = np.flatnonzero(raw_contact[1:] != raw_contact[:-1]) + 1
    starts = np.r_[0, changes]
    ends = np.r_[changes, raw_contact.size]
    latch = bool(raw_contact[0])
    last_hs_time: float | None = None
    events: list[dict[str, Any]] = []
    for start, end in zip(starts[1:], ends[1:]):
        target = bool(raw_contact[start])
        if target == latch or int(end - start) < MIN_STABLE_RUN_SAMPLES:
            continue
        confirmed_index = int(start + DEBOUNCE_SAMPLES)
        delivered_index = min(
            int(math.ceil(confirmed_index / POLICY_SAMPLES) * POLICY_SAMPLES),
            int(times.size - 1),
        )
        event_name = "heel_strike" if target else "toe_off"
        onset_state = (
            "BOTH"
            if heel_bits[start] and toe_bits[start]
            else "HEEL"
            if heel_bits[start]
            else "TOE"
            if toe_bits[start]
            else "AIR"
        )
        event: dict[str, Any] = {
            "side": "left",
            "event": event_name,
            "time": float(times[start]),
            "event_time_s": float(times[start]),
            "confirmed_time": float(times[confirmed_index]),
            "confirmed_time_s": float(times[confirmed_index]),
            "delivered_time_s": float(times[delivered_index]),
            "source": "binary_phase_fsm_v20_fast_screen",
            "onset_contact_state": onset_state,
        }
        if target:
            leader = (
                "both"
                if heel_bits[start] and toe_bits[start]
                else "heel"
                if heel_bits[start]
                else "toe"
            )
            event["contact_leader"] = leader
            event["landing_sensor"] = leader
            last_hs_time = float(times[start])
        else:
            event["startup_partial_stance"] = last_hs_time is None
        events.append(event)
        latch = target
    return events


def event_signature(events: Sequence[Mapping[str, Any]]) -> list[tuple[Any, ...]]:
    return [
        (
            str(event.get("event")),
            float(event.get("event_time_s")),
            float(event.get("confirmed_time_s")),
            float(event.get("delivered_time_s")),
            event.get("contact_leader"),
        )
        for event in events
    ]


def _compact_unit(unit: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "trial_id": str(unit["trial_id"]),
        "view_id": str(unit["view_id"]),
        "pass": bool(unit["pass"]),
        "exact_global_event_order": bool(unit["exact_global_event_order"]),
        "cycles": {
            "expected": int(unit["expected_complete_cycle_count"]),
            "observed": int(unit["observed_complete_cycle_count"]),
        },
        "heel_strike": {
            key: unit["events"]["heel_strike"][key]
            for key in (
                "reference_count",
                "predicted_count",
                "matched_within_tolerance",
                "precision",
                "recall",
                "nearest_confirmed_error_median_s",
                "maximum_absolute_confirmed_error_s",
                "pass",
            )
        },
        "toe_off": {
            key: unit["events"]["toe_off"][key]
            for key in (
                "reference_count",
                "predicted_count",
                "matched_within_tolerance",
                "precision",
                "recall",
                "nearest_confirmed_error_median_s",
                "maximum_absolute_confirmed_error_s",
                "pass",
            )
        },
        "phase_f1": float(unit["confirmed_phase"]["f1"]),
        "phase_iou": float(unit["confirmed_phase"]["iou"]),
        "accepted_flight_pass": bool(unit["accepted_flight_pass"]),
    }


def evaluate_fast_candidate(
    candidate: GeometryCandidate,
    *,
    traces: Mapping[str, AffineTrialTrace],
    ledgers: Mapping[str, Mapping[str, Any]],
    score_view: Callable[..., dict[str, Any]],
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
) -> dict[str, Any]:
    units: list[dict[str, Any]] = []
    event_counts: dict[str, dict[str, int]] = {}
    channel_trials: dict[str, dict[str, Any]] = {}
    fingerprint = hashlib.sha256()
    for trial_id in TRIALS:
        trace = traces[trial_id]

        def bits(point: PointGeometry) -> np.ndarray:
            key = (trial_id, point.role, tuple(round(x, 12) for x in point.location_m))
            if key not in bit_cache:
                bit_cache[key] = point_contact_bits(trace, point)
            return bit_cache[key]

        heel = bits(candidate.heel)
        toe = bits(candidate.toe)
        views = ledgers[trial_id]["scientific_core"]["views"]
        channel_trials[trial_id] = two_sensor_view_gate(
            trace.time_s, heel, toe, views
        )
        packed = np.packbits(
            np.column_stack((heel, toe)).reshape(-1), bitorder="little"
        )
        fingerprint.update(trial_id.encode("ascii"))
        fingerprint.update(packed.tobytes())
        events = fast_fsm_events(trace.time_s, heel, toe)
        event_counts[trial_id] = {
            name: sum(event["event"] == name for event in events)
            for name in ("heel_strike", "toe_off")
        }
        for view in views:
            unit = score_view(
                trial_id=trial_id,
                mode="v21_fast_screen",
                trace={"time_s": trace.time_s},
                events=events,
                view=view,
                parity_pass=True,
            )
            units.append(_compact_unit(unit))

    structure_failures = 0
    unmatched_events = 0
    cycle_error = 0
    normalized_maximum_errors: list[float] = []
    for unit in units:
        if (
            not unit["exact_global_event_order"]
            or unit["cycles"]["expected"] != unit["cycles"]["observed"]
            or unit["heel_strike"]["reference_count"]
            != unit["heel_strike"]["predicted_count"]
            or unit["toe_off"]["reference_count"]
            != unit["toe_off"]["predicted_count"]
            or not unit["accepted_flight_pass"]
        ):
            structure_failures += 1
        cycle_error += abs(unit["cycles"]["expected"] - unit["cycles"]["observed"])
        for event_name, tolerance in (("heel_strike", 0.05), ("toe_off", 0.08)):
            metric = unit[event_name]
            unmatched_events += (
                int(metric["reference_count"])
                + int(metric["predicted_count"])
                - 2 * int(metric["matched_within_tolerance"])
            )
            counts_exact = (
                int(metric["reference_count"]) == int(metric["predicted_count"])
            )
            maximum_error = metric["maximum_absolute_confirmed_error_s"]
            normalized_maximum_errors.append(
                1e6
                if not counts_exact or maximum_error is None
                else abs(float(maximum_error)) / tolerance
            )
    unit_pass_count = sum(bool(unit["pass"]) for unit in units)
    channel_gate = bool(
        len(channel_trials) == len(TRIALS)
        and all(item["pass"] for item in channel_trials.values())
    )
    minimum_f1 = min(float(unit["phase_f1"]) for unit in units)
    minimum_iou = min(float(unit["phase_iou"]) for unit in units)
    v19_distance = math.sqrt(
        ((candidate.heel.x_m - V19_BASE_X_M["left_heel"]) / 0.01) ** 2
        + ((candidate.toe.x_m - V19_BASE_X_M["left_toe"]) / 0.01) ** 2
        + ((candidate.heel.reach_m - V19_REACH_M) / 0.005) ** 2
        + ((candidate.toe.reach_m - V19_REACH_M) / 0.005) ** 2
    )
    rank_key = [
        int(not channel_gate),
        int(structure_failures),
        int(unmatched_events),
        int(cycle_error),
        float(max(normalized_maximum_errors)),
        float(max(0.0, 0.95 - minimum_f1)),
        float(max(0.0, 0.90 - minimum_iou)),
        int(len(units) - unit_pass_count),
        float(v19_distance),
    ]
    result = {
        **candidate.payload(),
        "fast_screen": {
            "pass": bool(channel_gate and unit_pass_count == len(units)),
            "channel_non_degenerate_pass": channel_gate,
            "two_sensor_channel_gate_by_trial": channel_trials,
            "unit_count": len(units),
            "unit_pass_count": unit_pass_count,
            "structure_failure_count": structure_failures,
            "unmatched_event_count": unmatched_events,
            "cycle_count_absolute_error": cycle_error,
            "minimum_phase_f1": minimum_f1,
            "minimum_phase_iou": minimum_iou,
            "event_counts_full_trace": event_counts,
            "bit_fingerprint_sha256": fingerprint.hexdigest(),
            "rank_key": rank_key,
            "units": units,
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def result_sort_key(result: Mapping[str, Any]) -> tuple[Any, ...]:
    return (
        *tuple(result["fast_screen"]["rank_key"]),
        str(result["candidate_id"]),
    )


def select_distinct_seeds(
    results: Sequence[Mapping[str, Any]],
    candidates: Mapping[str, GeometryCandidate],
    count: int,
    min_normalized_distance: float = DEFAULT_FINE_SEED_MIN_DISTANCE,
) -> list[GeometryCandidate]:
    if count <= 0:
        return []
    if not math.isfinite(min_normalized_distance) or min_normalized_distance < 0.0:
        raise V21SweepError("fine-seed minimum distance must be nonnegative")
    selected: list[GeometryCandidate] = []
    fingerprints: set[str] = set()
    for result in sorted(results, key=result_sort_key):
        fingerprint = str(result["fast_screen"]["bit_fingerprint_sha256"])
        if fingerprint in fingerprints:
            continue
        candidate = candidates[str(result["candidate_id"])]
        if any(
            _normalized_geometry_distance(candidate, previous)
            < min_normalized_distance
            for previous in selected
        ):
            continue
        selected.append(candidate)
        fingerprints.add(fingerprint)
        if len(selected) == count:
            break
    return selected


def _normalized_geometry_distance(
    left: GeometryCandidate, right: GeometryCandidate
) -> float:
    scales = (0.01, 0.005, 0.01, 0.005)
    return float(
        math.sqrt(
            sum(
                ((left_value - right_value) / scale) ** 2
                for left_value, right_value, scale in zip(
                    left.key, right.key, scales
                )
            )
        )
    )


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V21SweepError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    try:
        descriptor = os.open(str(path), flags)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def _atomic_write_exclusive(
    path: Path, writer: Callable[[TextIO], None]
) -> Path:
    """Publish a complete file atomically without replacing an existing one."""

    path = Path(path)
    if not path.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {path.parent}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(
            descriptor, "w", encoding="utf-8", newline="\n"
        ) as stream:
            descriptor_open = False
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        # A hard-link publication is atomic and, unlike os.replace(), cannot
        # clobber a receipt created by another process.  Both paths are in the
        # same run directory, so they are necessarily on the same filesystem.
        os.link(temporary, path)
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    def writer(stream: TextIO) -> None:
        json.dump(
            dict(payload), stream, indent=2, sort_keys=True, allow_nan=False
        )
        stream.write("\n")

    return _atomic_write_exclusive(path, writer)


def _write_jsonl_exclusive(
    path: Path, rows: Sequence[Mapping[str, Any]]
) -> Path:
    def writer(stream: TextIO) -> None:
        for row in rows:
            stream.write(
                json.dumps(dict(row), sort_keys=True, allow_nan=False) + "\n"
            )

    return _atomic_write_exclusive(path, writer)


def _verify_pinned_sources() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative, expected in PINNED_SOURCES.items():
        path = REPO_ROOT / relative
        if not path.is_file():
            raise V21SweepError(f"pinned source is missing: {relative}")
        observed = sha256_file(path)
        if observed != expected:
            raise V21SweepError(
                f"pinned source drifted: {relative}: {observed} != {expected}"
            )
        records[relative] = {"path": relative, "sha256": observed}
    if not V19_PROFILE_PATH.is_file() or sha256_file(V19_PROFILE_PATH) != V19_PROFILE_SHA256:
        raise V21SweepError("V19 binary profile drifted")
    if not MESH_PATH.is_file() or sha256_file(MESH_PATH) != MESH_SHA256:
        raise V21SweepError("left-foot mesh drifted")
    records["v19_profile"] = _source_record(V19_PROFILE_PATH)
    records["foot_mesh"] = _source_record(MESH_PATH)
    return records


def _acquire_affine_trial(
    trial_id: str,
    *,
    v19: Any,
    opensim: Any,
    SimulatorConfig: Any,
    KinematicsInterpolator: Any,
    load_plugin: Callable[[str], None],
    progress_options: Mapping[str, Any],
) -> AffineTrialTrace:
    if trial_id not in TRIALS:
        raise V21SweepError(f"unauthorized trial access: {trial_id}")
    inputs = v19._trial_inputs(trial_id)
    if inputs["lock_sha256"] != PINNED_TRIAL_INPUTS[trial_id]["lock_sha256"]:
        raise V21SweepError(f"trial {trial_id} preprocessing lock drifted")
    if sha256_file(Path(inputs["ik_path"])) != PINNED_TRIAL_INPUTS[trial_id]["ik_sha256"]:
        raise V21SweepError(f"trial {trial_id} IK hash drifted")
    if sha256_file(Path(inputs["model_path"])) != PINNED_MODEL_SHA256:
        raise V21SweepError(f"trial {trial_id} model hash drifted")
    if sha256_file(Path(inputs["plugin_binary_path"])) != PINNED_PLUGIN_SHA256:
        raise V21SweepError(f"trial {trial_id} plugin binary hash drifted")

    load_plugin(str(inputs["plugin_loader"]))
    model = opensim.Model(str(inputs["model_path"]))
    state = model.initSystem()
    frame = opensim.PhysicalFrame.safeDownCast(model.getComponent(FOOT_FRAME))
    if frame is None:
        raise V21SweepError(f"cannot resolve PhysicalFrame {FOOT_FRAME}")
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(Path(inputs["model_path"]).parent)
    cfg.model_file = str(inputs["model_path"])
    cfg.kinematics_file = str(inputs["ik_path"])
    cfg.t_start, cfg.t_end = v19.EXPECTED_INTERVALS_S[trial_id]
    kin = KinematicsInterpolator(cfg)
    times = v19._time_grid(trial_id)
    coefficients = np.empty((times.size, 3), dtype=float)
    offsets = np.empty(times.size, dtype=float)
    coordinates = model.getCoordinateSet()
    normal = np.asarray(GROUND_NORMAL, dtype=float)
    ground_origin = np.asarray(GROUND_ORIGIN, dtype=float)
    local_stations = (
        opensim.Vec3(0.0, 0.0, 0.0),
        opensim.Vec3(1.0, 0.0, 0.0),
        opensim.Vec3(0.0, 1.0, 0.0),
        opensim.Vec3(0.0, 0.0, 1.0),
    )
    progress = SweepProgress(
        total=int(times.size),
        label=f"V21 cache DEV{trial_id}",
        **progress_options,
    )
    try:
        for row, time_value in enumerate(times):
            q, _qdot, _qddot = kin.get(float(time_value))
            state.setTime(float(time_value))
            for index in range(coordinates.getSize()):
                coordinate = coordinates.get(index)
                name = coordinate.getName()
                if name in q:
                    coordinate.setValue(state, float(q[name]), False)
            model.realizePosition(state)
            stations = []
            for local in local_stations:
                ground = frame.findStationLocationInGround(state, local)
                stations.append(
                    np.asarray([float(ground.get(i)) for i in range(3)], dtype=float)
                )
            origin = stations[0]
            coefficients[row] = [
                float(normal @ (stations[index] - origin))
                for index in (1, 2, 3)
            ]
            offsets[row] = float(normal @ (origin - ground_origin))
            if row % 100 == 0 or row + 1 == times.size:
                progress.update(row + 1)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    if (
        not np.all(np.isfinite(coefficients))
        or not np.all(np.isfinite(offsets))
    ):
        raise V21SweepError(f"trial {trial_id} affine cache is non-finite")
    source = {
        "preprocessing_lock": _source_record(Path(inputs["lock_path"])),
        "model": _source_record(Path(inputs["model_path"])),
        "ik_motion": _source_record(Path(inputs["ik_path"])),
        "plugin_binary": _source_record(Path(inputs["plugin_binary_path"])),
    }
    return AffineTrialTrace(
        trial_id=trial_id,
        time_s=times,
        normal_coefficients=coefficients,
        normal_offset=offsets,
        source=source,
    )


def _verify_affine_fast_path(
    factory: MeshPointFactory,
    traces: Mapping[str, AffineTrialTrace],
    profile: Any,
) -> dict[str, Any]:
    by_role = {point.name: point for point in profile.points}
    baseline_points = {
        role: factory.build(role, V19_BASE_X_M[role], V19_REACH_M)
        for role in ("left_heel", "left_toe")
    }
    for role, point in baseline_points.items():
        expected = np.asarray(by_role[role].location, dtype=float)
        observed = np.asarray(point.location_m, dtype=float)
        if float(np.max(np.abs(expected - observed))) > 1e-10:
            raise V21SweepError(
                f"mesh parameterization does not reconstruct V19 {role}: "
                f"{observed.tolist()} != {expected.tolist()}"
            )
    result: dict[str, Any] = {}
    for trial_id, trace in traces.items():
        heel = point_contact_bits(trace, baseline_points["left_heel"])
        toe = point_contact_bits(trace, baseline_points["left_toe"])
        observed = bit_trace_sha256(heel, toe)
        expected = EXPECTED_V19_BIT_TRACE_SHA256[trial_id]
        if observed != expected:
            raise V21SweepError(
                f"trial {trial_id} affine fast path changed V19 bits: "
                f"{observed} != {expected}"
            )
        result[trial_id] = {
            "bit_trace_sha256": observed,
            "sample_count": int(trace.time_s.size),
            "pass": True,
        }
    return result


def _evaluate_stage(
    label: str,
    candidates: Sequence[GeometryCandidate],
    *,
    traces: Mapping[str, AffineTrialTrace],
    ledgers: Mapping[str, Mapping[str, Any]],
    score_view: Callable[..., dict[str, Any]],
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
    progress_options: Mapping[str, Any],
) -> list[dict[str, Any]]:
    if not candidates:
        raise V21SweepError(f"{label} candidate grid is empty")
    progress = SweepProgress(
        total=len(candidates), label=f"V21 {label}", **progress_options
    )
    results: list[dict[str, Any]] = []
    try:
        for index, candidate in enumerate(candidates, start=1):
            results.append(
                evaluate_fast_candidate(
                    candidate,
                    traces=traces,
                    ledgers=ledgers,
                    score_view=score_view,
                    bit_cache=bit_cache,
                )
            )
            progress.update(index)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    return results


def _verify_candidate(
    candidate: GeometryCandidate,
    *,
    traces: Mapping[str, AffineTrialTrace],
    ledgers: Mapping[str, Mapping[str, Any]],
    run_mode: Callable[..., dict[str, Any]],
    score_view: Callable[..., dict[str, Any]],
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
) -> dict[str, Any]:
    modes = ("sequential_1ms", "batched_10ms_same_samples")
    units: list[dict[str, Any]] = []
    parity: list[dict[str, Any]] = []
    channel_trials: dict[str, dict[str, Any]] = {}
    for trial_id in TRIALS:
        affine = traces[trial_id]

        def bits(point: PointGeometry) -> np.ndarray:
            key = (trial_id, point.role, tuple(round(x, 12) for x in point.location_m))
            if key not in bit_cache:
                bit_cache[key] = point_contact_bits(affine, point)
            return bit_cache[key]

        heel = bits(candidate.heel)
        toe = bits(candidate.toe)
        views = ledgers[trial_id]["scientific_core"]["views"]
        channel_trials[trial_id] = two_sensor_view_gate(
            affine.time_s, heel, toe, views
        )
        trace = {"time_s": affine.time_s, "heel": heel, "toe": toe}
        results = {mode: run_mode(trace, mode) for mode in modes}
        sequential = results[modes[0]]
        batched = results[modes[1]]
        fast = fast_fsm_events(affine.time_s, heel, toe)
        parity_record = {
            "trial_id": trial_id,
            "events_exact": sequential["events"] == batched["events"],
            "transitions_exact": (
                sequential["contact_state_transitions"]
                == batched["contact_state_transitions"]
            ),
            "cancellations_exact": (
                sequential["candidate_cancellations"]
                == batched["candidate_cancellations"]
            ),
            "state_digest_exact": (
                sequential["boundary_snapshots_sha256"]
                == batched["boundary_snapshots_sha256"]
            ),
            "final_payload_exact": (
                sequential["final_payload"] == batched["final_payload"]
            ),
            "fast_event_signature_exact": (
                event_signature(fast) == event_signature(sequential["events"])
            ),
        }
        parity_record["pass"] = all(
            bool(value)
            for key, value in parity_record.items()
            if key != "trial_id"
        )
        parity.append(parity_record)
        for mode in modes:
            for view in views:
                unit = score_view(
                    trial_id=trial_id,
                    mode=mode,
                    trace=trace,
                    events=results[mode]["events"],
                    view=view,
                    parity_pass=bool(parity_record["pass"]),
                )
                units.append(_compact_unit(unit))
    channel_gate = bool(
        len(channel_trials) == len(TRIALS)
        and all(item["pass"] for item in channel_trials.values())
    )
    eligible = bool(
        len(units) == 16
        and channel_gate
        and all(unit["pass"] for unit in units)
        and all(item["pass"] for item in parity)
    )
    return {
        **candidate.payload(),
        "eligible": eligible,
        "two_sensor_channel_gate": {
            "pass": channel_gate,
            "trials": channel_trials,
        },
        "unit_count": len(units),
        "unit_pass_count": sum(unit["pass"] for unit in units),
        "parity": parity,
        "units": units,
    }


def candidate_profile(candidate: GeometryCandidate, profile: Any) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "detector_type": str(profile.detector_type),
        "ground": {
            "origin": list(profile.ground.origin),
            "normal": list(profile.ground.normal),
        },
        "points": [
            {
                "name": candidate.heel.role,
                "frame": FOOT_FRAME,
                "location": list(candidate.heel.location_m),
            },
            {
                "name": candidate.toe.role,
                "frame": FOOT_FRAME,
                "location": list(candidate.toe.location_m),
            },
        ],
        "contact_rule": {"contact_when": "signed_clearance_le_zero"},
    }


def _progress_options(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "width": int(args.progress_width),
        "min_redraw_interval_s": float(args.progress_interval_s),
        "non_tty_interval_s": float(args.non_tty_progress_interval_s),
        "enabled": not bool(args.no_progress),
    }


def _parse_sweep_axes(args: argparse.Namespace) -> dict[str, tuple[float, ...]]:
    axes = {
        "heel_x_fractions": _parse_csv_floats(
            args.heel_x_fractions, label="heel-x-fractions"
        ),
        "toe_x_fractions": _parse_csv_floats(
            args.toe_x_fractions, label="toe-x-fractions"
        ),
        "heel_reaches_m": tuple(
            value / 1000.0
            for value in _parse_csv_floats(
                args.heel_reach_mm, label="heel-reach-mm"
            )
        ),
        "toe_reaches_m": tuple(
            value / 1000.0
            for value in _parse_csv_floats(
                args.toe_reach_mm, label="toe-reach-mm"
            )
        ),
    }
    for role, key in (
        ("left_heel", "heel_x_fractions"),
        ("left_toe", "toe_x_fractions"),
    ):
        low, high = X_FRACTION_BOUNDS[role]
        if any(value < low or value > high for value in axes[key]):
            raise V21SweepError(
                f"{key} must stay inside the frozen [{low}, {high}] band"
            )
    for role, key in (
        ("left_heel", "heel_reaches_m"),
        ("left_toe", "toe_reaches_m"),
    ):
        low, high = REACH_BOUNDS_M[role]
        if any(value < low or value > high for value in axes[key]):
            raise V21SweepError(
                f"{key} must stay inside the frozen [{low}, {high}] m band"
            )
    return axes


def _selection_configuration(
    args: argparse.Namespace, axes: Mapping[str, Sequence[float]]
) -> dict[str, Any]:
    return {
        "heel_x_fractions": list(axes["heel_x_fractions"]),
        "toe_x_fractions": list(axes["toe_x_fractions"]),
        "heel_reach_mm": [
            1000.0 * value for value in axes["heel_reaches_m"]
        ],
        "toe_reach_mm": [
            1000.0 * value for value in axes["toe_reaches_m"]
        ],
        "fine_seed_count": int(args.fine_seed_count),
        "fine_seed_min_distance": float(args.fine_seed_min_distance),
        "fine_x_radius_mm": float(args.fine_x_radius_mm),
        "fine_x_step_mm": float(args.fine_x_step_mm),
        "fine_reach_radius_mm": float(args.fine_reach_radius_mm),
        "fine_reach_step_mm": float(args.fine_reach_step_mm),
        "verify_top_k": int(args.verify_top_k),
    }


def run_sweep(args: argparse.Namespace) -> dict[str, Any]:
    if tuple(TRIALS) != ("02", "04") or set(TRIALS) & set(FORBIDDEN_TRIALS):
        raise V21SweepError("development access allowlist drifted")
    axes = _parse_sweep_axes(args)
    selection_configuration = _selection_configuration(args, axes)
    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        output_dir.relative_to(VALIDATION_ROOT.resolve())
    except ValueError as exc:
        raise V21SweepError("output-dir must stay under validation/") from exc
    output_dir.mkdir(parents=True, exist_ok=False)
    started_at = time.monotonic()
    try:
        _write_json_exclusive(
            output_dir / "run_start.json",
            {
                "schema_version": SCHEMA_VERSION,
                "sweep_id": SWEEP_ID,
                "status": "V21_GEOMETRY_SWEEP_STARTED",
                "development_trials": list(TRIALS),
                "trial_08_opened": False,
                "protected_trials_opened": [],
                "reserve_trials_opened": [],
                "rerun_in_same_directory_allowed": False,
                "selection_configuration": selection_configuration,
            },
        )
        sources = _verify_pinned_sources()

        import opensim
        import validate_binary_phase_detector_v19_raw_geometry as v19
        import validate_binary_phase_fsm_v20_development as v20
        from binary_phase_detector import load_binary_phase_detector_profile
        from config import SimulatorConfig
        from kinematics_interpolator import KinematicsInterpolator
        from model_loader import _load_plugin
        from audit_two_sensor_prescribed_geometry import _load_stl_triangles
        from build_two_sensor_mesh_profile_v4 import (
            _section_z_bounds_at_x,
            _vertical_surface_intersections_y,
        )

        platform_receipt = v20._platform_preflight()
        ledgers = v20._load_oracles()
        profile = load_binary_phase_detector_profile(V19_PROFILE_PATH)
        if (
            tuple(profile.ground.origin) != GROUND_ORIGIN
            or tuple(profile.ground.normal) != GROUND_NORMAL
        ):
            raise V21SweepError("V19 ground plane drifted")
        triangles = _load_stl_triangles(MESH_PATH)
        factory = MeshPointFactory(
            triangles,
            section_z_bounds=_section_z_bounds_at_x,
            vertical_y_intersections=_vertical_surface_intersections_y,
        )
        heel_x_fractions = axes["heel_x_fractions"]
        toe_x_fractions = axes["toe_x_fractions"]
        heel_reaches_m = axes["heel_reaches_m"]
        toe_reaches_m = axes["toe_reaches_m"]
        coarse_candidates = generate_coarse_candidates(
            factory,
            heel_x_fractions=heel_x_fractions,
            toe_x_fractions=toe_x_fractions,
            heel_reaches_m=heel_reaches_m,
            toe_reaches_m=toe_reaches_m,
        )
        progress_options = _progress_options(args)
        traces = {
            trial_id: _acquire_affine_trial(
                trial_id,
                v19=v19,
                opensim=opensim,
                SimulatorConfig=SimulatorConfig,
                KinematicsInterpolator=KinematicsInterpolator,
                load_plugin=_load_plugin,
                progress_options=progress_options,
            )
            for trial_id in TRIALS
        }
        fast_path_receipt = _verify_affine_fast_path(factory, traces, profile)

        bit_cache: dict[
            tuple[str, str, tuple[float, float, float]], np.ndarray
        ] = {}
        coarse_results = _evaluate_stage(
            "coarse",
            coarse_candidates,
            traces=traces,
            ledgers=ledgers,
            score_view=v20._score_view,
            bit_cache=bit_cache,
            progress_options=progress_options,
        )
        _write_jsonl_exclusive(output_dir / "coarse_results.jsonl", coarse_results)
        coarse_by_id = {
            candidate.candidate_id: candidate for candidate in coarse_candidates
        }
        seeds = select_distinct_seeds(
            coarse_results,
            coarse_by_id,
            int(args.fine_seed_count),
            min_normalized_distance=float(args.fine_seed_min_distance),
        )
        fine_candidates = generate_fine_candidates(
            factory,
            seeds,
            x_radius_m=float(args.fine_x_radius_mm) / 1000.0,
            x_step_m=float(args.fine_x_step_mm) / 1000.0,
            reach_radius_m=float(args.fine_reach_radius_mm) / 1000.0,
            reach_step_m=float(args.fine_reach_step_mm) / 1000.0,
            excluded_keys={candidate.key for candidate in coarse_candidates},
        )
        fine_results = _evaluate_stage(
            "fine",
            fine_candidates,
            traces=traces,
            ledgers=ledgers,
            score_view=v20._score_view,
            bit_cache=bit_cache,
            progress_options=progress_options,
        )
        _write_jsonl_exclusive(output_dir / "fine_results.jsonl", fine_results)

        all_candidates = {
            candidate.candidate_id: candidate
            for candidate in (*coarse_candidates, *fine_candidates)
        }
        all_results = sorted(
            [*coarse_results, *fine_results], key=result_sort_key
        )
        verify_count = min(int(args.verify_top_k), len(all_results))
        verification_progress = SweepProgress(
            total=verify_count,
            label="V21 final verification",
            **progress_options,
        )
        verifications: list[dict[str, Any]] = []
        try:
            for index, result in enumerate(all_results[:verify_count], start=1):
                verifications.append(
                    _verify_candidate(
                        all_candidates[str(result["candidate_id"])],
                        traces=traces,
                        ledgers=ledgers,
                        run_mode=v20._run_mode,
                        score_view=v20._score_view,
                        bit_cache=bit_cache,
                    )
                )
                verification_progress.update(index)
        except BaseException:
            verification_progress.finish("FAILED")
            raise
        verification_progress.finish("DONE")
        _write_json_exclusive(
            output_dir / "final_verification.json",
            {"schema_version": SCHEMA_VERSION, "candidates": verifications},
        )
        eligible = [item for item in verifications if item["eligible"]]
        selected_verification = eligible[0] if eligible else verifications[0]
        selected_candidate = all_candidates[selected_verification["candidate_id"]]
        profile_name = (
            "eligible_finalist_profile.json"
            if eligible
            else "diagnostic_best_not_promotable_profile.json"
        )
        profile_path = _write_json_exclusive(
            output_dir / profile_name,
            candidate_profile(selected_candidate, profile),
        )
        elapsed_s = float(time.monotonic() - started_at)
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "sweep_id": SWEEP_ID,
            "status": (
                "V21_GEOMETRY_SWEEP_ELIGIBLE_FINALIST_DEV02_04_TRIAL08_CLOSED"
                if eligible
                else "V21_GEOMETRY_SWEEP_NO_ELIGIBLE_FINALIST_TRIAL08_CLOSED"
            ),
            "eligible_finalist": bool(eligible),
            "selected_candidate_id": selected_candidate.candidate_id,
            "selected_profile": _source_record(profile_path),
            "diagnostic_best_is_promotable": bool(eligible),
            "candidate_counts": {
                "coarse": len(coarse_candidates),
                "fine": len(fine_candidates),
                "total": len(all_results),
                "final_verified": len(verifications),
            },
            "grid": {
                "heel_x_fractions": list(heel_x_fractions),
                "toe_x_fractions": list(toe_x_fractions),
                "heel_reach_mm": [1000.0 * value for value in heel_reaches_m],
                "toe_reach_mm": [1000.0 * value for value in toe_reaches_m],
                "fine_seed_count": int(args.fine_seed_count),
                "fine_seed_count_selected": len(seeds),
                "fine_seed_ids": [candidate.candidate_id for candidate in seeds],
                "fine_seed_min_distance": float(args.fine_seed_min_distance),
                "fine_x_radius_mm": float(args.fine_x_radius_mm),
                "fine_x_step_mm": float(args.fine_x_step_mm),
                "fine_reach_radius_mm": float(args.fine_reach_radius_mm),
                "fine_reach_step_mm": float(args.fine_reach_step_mm),
                "verify_top_k": int(args.verify_top_k),
            },
            "frozen_contract": {
                "ground_origin": list(GROUND_ORIGIN),
                "ground_normal": list(GROUND_NORMAL),
                "contact_rule": "signed_clearance_le_zero",
                "sample_dt_s": SAMPLE_DT_S,
                "debounce_s": DEBOUNCE_SAMPLES * SAMPLE_DT_S,
                "policy_step_s": POLICY_SAMPLES * SAMPLE_DT_S,
                "fsm_changed": False,
                "primary_grf_changed": False,
                "sea_semantics_changed": False,
            },
            "data_access": {
                "development_trials_opened": list(TRIALS),
                "trial_08_opened": False,
                "protected_trials_opened": [],
                "reserve_trials_opened": [],
                "prescribed_grf_read": False,
                "canonical_ledgers_read": list(TRIALS),
            },
            "affine_fast_path": fast_path_receipt,
            "platform": platform_receipt,
            "elapsed_s": elapsed_s,
            "sources": {
                **sources,
                "script": _source_record(Path(__file__)),
                "trial_inputs": {
                    trial_id: traces[trial_id].source for trial_id in TRIALS
                },
            },
            "artifacts": {
                "coarse_results": _source_record(
                    output_dir / "coarse_results.jsonl"
                ),
                "fine_results": _source_record(output_dir / "fine_results.jsonl"),
                "final_verification": _source_record(
                    output_dir / "final_verification.json"
                ),
            },
            "next_stage": (
                "FREEZE_BEFORE_ONE_SHOT_TRIAL08"
                if eligible
                else "STOP_NO_AUTOMATIC_GEOMETRY_PROMOTION"
            ),
        }
        _write_json_exclusive(output_dir / "manifest.json", manifest)
        return manifest
    except BaseException as exc:
        failure = {
            "schema_version": SCHEMA_VERSION,
            "sweep_id": SWEEP_ID,
            "status": "V21_GEOMETRY_SWEEP_FAILED_CLOSED",
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "data_access": {
                "authorized_development_trials": list(TRIALS),
                "trial_08_opened": False,
                "protected_trials_opened": [],
                "reserve_trials_opened": [],
            },
        }
        try:
            _write_json_exclusive(output_dir / "failure.json", failure)
        except Exception:
            pass
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="explicitly authorize the DEV02/04 numerical sweep",
    )
    parser.add_argument(
        "--output-dir",
        help="new no-clobber run directory under validation/ (required with --execute)",
    )
    parser.add_argument("--heel-x-fractions", default=DEFAULT_HEEL_X_FRACTIONS)
    parser.add_argument("--toe-x-fractions", default=DEFAULT_TOE_X_FRACTIONS)
    parser.add_argument("--heel-reach-mm", default=DEFAULT_HEEL_REACH_MM)
    parser.add_argument("--toe-reach-mm", default=DEFAULT_TOE_REACH_MM)
    parser.add_argument("--fine-seed-count", type=int, default=4)
    parser.add_argument(
        "--fine-seed-min-distance",
        type=float,
        default=DEFAULT_FINE_SEED_MIN_DISTANCE,
        help="minimum normalized 4-D distance between refinement seeds",
    )
    parser.add_argument(
        "--fine-x-radius-mm", type=float, default=DEFAULT_FINE_X_RADIUS_MM
    )
    parser.add_argument("--fine-x-step-mm", type=float, default=3.0)
    parser.add_argument(
        "--fine-reach-radius-mm",
        type=float,
        default=DEFAULT_FINE_REACH_RADIUS_MM,
    )
    parser.add_argument("--fine-reach-step-mm", type=float, default=1.0)
    parser.add_argument("--verify-top-k", type=int, default=3)
    parser.add_argument("--progress-width", type=int, default=24)
    parser.add_argument("--progress-interval-s", type=float, default=0.10)
    parser.add_argument("--non-tty-progress-interval-s", type=float, default=30.0)
    parser.add_argument("--no-progress", action="store_true")
    return parser


def _validate_cli(args: argparse.Namespace) -> None:
    if args.execute and not args.output_dir:
        raise V21SweepError("--output-dir is required with --execute")
    if int(args.fine_seed_count) <= 0:
        raise V21SweepError("fine-seed-count must be positive")
    if int(args.verify_top_k) <= 0:
        raise V21SweepError("verify-top-k must be positive")
    if int(args.progress_width) <= 0:
        raise V21SweepError("progress-width must be positive")
    for name in (
        "fine_x_radius_mm",
        "fine_x_step_mm",
        "fine_reach_radius_mm",
        "fine_reach_step_mm",
        "fine_seed_min_distance",
        "progress_interval_s",
        "non_tty_progress_interval_s",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            raise V21SweepError(f"{name} must be finite and nonnegative")
    if args.fine_x_step_mm <= 0.0 or args.fine_reach_step_mm <= 0.0:
        raise V21SweepError("fine grid steps must be positive")
    if (
        args.fine_x_radius_mm + 1e-12 < args.fine_x_step_mm
        and args.fine_reach_radius_mm + 1e-12 < args.fine_reach_step_mm
    ):
        raise V21SweepError(
            "at least one fine-grid radius must reach its corresponding step"
        )
    # Validate every selection-affecting CSV before readiness is reported or
    # an output directory / OpenSim trace is acquired.
    _parse_sweep_axes(args)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        _validate_cli(args)
        if not args.execute:
            print(
                json.dumps(
                    {
                        "status": "V21_SWEEP_SCRIPT_READY_NOT_EXECUTED",
                        "message": (
                            "Pass --execute and a new --output-dir under "
                            "validation/ to run DEV02/04."
                        ),
                        "development_trials": list(TRIALS),
                        "forbidden_trials": list(FORBIDDEN_TRIALS),
                        "progress_display": [
                            "progress_bar",
                            "elapsed",
                            "ETA",
                        ],
                    },
                    indent=2,
                    sort_keys=True,
                    allow_nan=False,
                )
            )
            return 0
        manifest = run_sweep(args)
    except KeyboardInterrupt:
        print("V21 sweep interrupted; gate remains closed.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(
            f"V21 sweep failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False))
    return 0 if manifest["eligible_finalist"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
