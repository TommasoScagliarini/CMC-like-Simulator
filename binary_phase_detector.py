"""Force-free binary heel/toe contact geometry.

The detector in this module is deliberately smaller than an online-GRF
contact model: two body-fixed stations are compared with one ground plane and
produce two boolean contact bits.  It does not add OpenSim components, compute
forces, debounce transitions, or assign gait events.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import opensim


BINARY_PHASE_DETECTOR_TYPE = "binary_point_clearance_v1"
BINARY_PHASE_CONTACT_RULE = "signed_clearance_le_zero"
BINARY_PHASE_ROLES = ("left_heel", "left_toe")


class BinaryPhaseDetectorProfileError(ValueError):
    """Raised when the force-free binary detector profile is malformed."""


class BinaryPhaseDetectorSamplingError(RuntimeError):
    """Raised when a station cannot produce one finite binary sample."""


@dataclass(frozen=True)
class BinaryGroundPlane:
    origin: tuple[float, float, float]
    normal: tuple[float, float, float]


@dataclass(frozen=True)
class BinaryContactPoint:
    name: str
    frame: str
    location: tuple[float, float, float]


@dataclass(frozen=True)
class BinaryPhaseDetectorProfile:
    schema_version: int
    detector_type: str
    ground: BinaryGroundPlane
    points: tuple[BinaryContactPoint, BinaryContactPoint]
    contact_when: str


@dataclass(frozen=True)
class BinaryPhaseDetectorReading:
    """One raw geometric reading, before any edge or event interpretation."""

    time_s: float
    contacts: Mapping[str, bool]
    signed_clearance_m: Mapping[str, float]

    def public_sample(self) -> dict[str, float | bool]:
        """Return the consumer-facing payload containing only time and bits."""
        for role in BINARY_PHASE_ROLES:
            if type(self.contacts.get(role)) is not bool:
                raise BinaryPhaseDetectorSamplingError(
                    f"Binary detector contact {role!r} must be a Python bool."
                )
        return {
            "time_s": float(self.time_s),
            "left_heel_contact": self.contacts["left_heel"],
            "left_toe_contact": self.contacts["left_toe"],
        }

    def diagnostics(self) -> dict[str, Any]:
        """Return clearances separately so they cannot be mistaken for events."""
        return {
            "time_s": float(self.time_s),
            "detector_type": BINARY_PHASE_DETECTOR_TYPE,
            "signed_clearance_m": {
                role: float(self.signed_clearance_m[role])
                for role in BINARY_PHASE_ROLES
            },
            "contacts": {
                role: bool(self.contacts[role])
                for role in BINARY_PHASE_ROLES
            },
        }


def _reject_json_constant(value: str) -> None:
    raise BinaryPhaseDetectorProfileError(
        f"Binary detector profile contains non-finite JSON constant {value!r}."
    )


def _strict_object(
    value: object,
    *,
    label: str,
    required_keys: set[str],
) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise BinaryPhaseDetectorProfileError(f"{label} must be a JSON object.")
    observed = {str(key) for key in value}
    if observed != required_keys:
        missing = sorted(required_keys - observed)
        unexpected = sorted(observed - required_keys)
        raise BinaryPhaseDetectorProfileError(
            f"{label} fields differ from the frozen schema; "
            f"missing={missing}, unexpected={unexpected}."
        )
    return value


def _finite_vec3(value: object, *, label: str) -> tuple[float, float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != 3
    ):
        raise BinaryPhaseDetectorProfileError(
            f"{label} must contain exactly three numbers."
        )
    result: list[float] = []
    for item in value:
        if isinstance(item, (bool, np.bool_)):
            raise BinaryPhaseDetectorProfileError(
                f"{label} must contain numbers, not booleans."
            )
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise BinaryPhaseDetectorProfileError(
                f"{label} must contain exactly three numbers."
            ) from exc
        if not math.isfinite(number):
            raise BinaryPhaseDetectorProfileError(
                f"{label} must contain only finite numbers."
            )
        result.append(number)
    return (result[0], result[1], result[2])


def load_binary_phase_detector_profile(
    path: str | Path,
) -> BinaryPhaseDetectorProfile:
    """Load the exact V19 force-free point-detector schema, fail-closed."""
    profile_path = Path(path)
    try:
        payload = json.loads(
            profile_path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except BinaryPhaseDetectorProfileError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BinaryPhaseDetectorProfileError(
            f"Cannot read binary detector profile {profile_path}: {exc}"
        ) from exc

    root = _strict_object(
        payload,
        label="binary detector profile",
        required_keys={
            "schema_version",
            "detector_type",
            "ground",
            "points",
            "contact_rule",
        },
    )
    schema_version = root["schema_version"]
    if type(schema_version) is not int or schema_version != 1:
        raise BinaryPhaseDetectorProfileError(
            "binary detector schema_version must be the integer 1."
        )
    detector_type = root["detector_type"]
    if detector_type != BINARY_PHASE_DETECTOR_TYPE:
        raise BinaryPhaseDetectorProfileError(
            "binary detector_type must be "
            f"{BINARY_PHASE_DETECTOR_TYPE!r}."
        )

    ground_raw = _strict_object(
        root["ground"],
        label="ground",
        required_keys={"origin", "normal"},
    )
    origin = _finite_vec3(ground_raw["origin"], label="ground.origin")
    normal = _finite_vec3(ground_raw["normal"], label="ground.normal")
    normal_norm = float(np.linalg.norm(np.asarray(normal, dtype=float)))
    if abs(normal_norm - 1.0) > 1e-9:
        raise BinaryPhaseDetectorProfileError(
            "ground.normal must already be unit length; observed norm "
            f"{normal_norm:.12g}."
        )

    points_raw = root["points"]
    if (
        not isinstance(points_raw, Sequence)
        or isinstance(points_raw, (str, bytes, bytearray))
        or len(points_raw) != 2
    ):
        raise BinaryPhaseDetectorProfileError(
            "binary detector profile must contain exactly two points."
        )
    points: list[BinaryContactPoint] = []
    for index, point_raw in enumerate(points_raw):
        point = _strict_object(
            point_raw,
            label=f"points[{index}]",
            required_keys={"name", "frame", "location"},
        )
        name = point["name"]
        frame = point["frame"]
        if not isinstance(name, str) or not name:
            raise BinaryPhaseDetectorProfileError(
                f"points[{index}].name must be a non-empty string."
            )
        if not isinstance(frame, str) or not frame.startswith("/"):
            raise BinaryPhaseDetectorProfileError(
                f"points[{index}].frame must be an absolute component path."
            )
        points.append(
            BinaryContactPoint(
                name=name,
                frame=frame,
                location=_finite_vec3(
                    point["location"],
                    label=f"points[{index}].location",
                ),
            )
        )
    observed_roles = tuple(point.name for point in points)
    if observed_roles != BINARY_PHASE_ROLES:
        raise BinaryPhaseDetectorProfileError(
            "binary detector points must be ordered exactly as "
            f"{BINARY_PHASE_ROLES}; observed {observed_roles}."
        )

    rule_raw = _strict_object(
        root["contact_rule"],
        label="contact_rule",
        required_keys={"contact_when"},
    )
    contact_when = rule_raw["contact_when"]
    if contact_when != BINARY_PHASE_CONTACT_RULE:
        raise BinaryPhaseDetectorProfileError(
            "contact_rule.contact_when must be "
            f"{BINARY_PHASE_CONTACT_RULE!r}."
        )

    return BinaryPhaseDetectorProfile(
        schema_version=1,
        detector_type=BINARY_PHASE_DETECTOR_TYPE,
        ground=BinaryGroundPlane(origin=origin, normal=normal),
        points=(points[0], points[1]),
        contact_when=BINARY_PHASE_CONTACT_RULE,
    )


def signed_clearance_m(
    point_in_ground: Sequence[float],
    ground: BinaryGroundPlane,
) -> float:
    """Signed point-to-plane distance; positive is above the detector plane."""
    point = np.asarray(point_in_ground, dtype=float)
    origin = np.asarray(ground.origin, dtype=float)
    normal = np.asarray(ground.normal, dtype=float)
    if (
        point.shape != (3,)
        or origin.shape != (3,)
        or normal.shape != (3,)
        or not np.all(np.isfinite(point))
        or not np.all(np.isfinite(origin))
        or not np.all(np.isfinite(normal))
    ):
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector point and ground plane must contain finite Vec3s."
        )
    normal_norm = float(np.linalg.norm(normal))
    if abs(normal_norm - 1.0) > 1e-9:
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector ground normal must be unit length."
        )
    clearance = float(np.dot(point - origin, normal))
    if not math.isfinite(clearance):
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector signed clearance is non-finite."
        )
    return clearance


def contact_bit(clearance_m: float) -> bool:
    """Hard binary switch: contact at or below the frozen ground plane."""
    if isinstance(clearance_m, (bool, np.bool_)):
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector clearance cannot be boolean."
        )
    try:
        clearance = float(clearance_m)
    except (TypeError, ValueError) as exc:
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector clearance must be numeric."
        ) from exc
    if not math.isfinite(clearance):
        raise BinaryPhaseDetectorSamplingError(
            "Binary detector clearance must be finite."
        )
    return bool(clearance <= 0.0)


def validate_binary_phase_detector_frames(
    model: opensim.Model,
    profile: BinaryPhaseDetectorProfile,
) -> None:
    """Resolve both station frames without adding anything to the model."""
    for point in profile.points:
        try:
            component = model.getComponent(point.frame)
            frame = opensim.PhysicalFrame.safeDownCast(component)
        except Exception as exc:
            raise BinaryPhaseDetectorProfileError(
                f"Cannot resolve binary detector frame {point.frame!r}."
            ) from exc
        if frame is None:
            raise BinaryPhaseDetectorProfileError(
                f"Binary detector frame {point.frame!r} is not a PhysicalFrame."
            )


class BinaryPhaseDetectorSampler:
    """Sample two body-fixed stations without creating an OpenSim Force."""

    def __init__(
        self,
        model: opensim.Model,
        profile: BinaryPhaseDetectorProfile,
    ) -> None:
        validate_binary_phase_detector_frames(model, profile)
        self._model = model
        self._profile = profile
        self._frames: dict[str, opensim.PhysicalFrame] = {}
        for point in profile.points:
            frame = opensim.PhysicalFrame.safeDownCast(
                model.getComponent(point.frame)
            )
            if frame is None:  # Defensive: validation above is authoritative.
                raise BinaryPhaseDetectorProfileError(
                    f"Binary detector frame {point.frame!r} is not physical."
                )
            self._frames[point.name] = frame

    @property
    def profile(self) -> BinaryPhaseDetectorProfile:
        return self._profile

    def sample(
        self,
        state: opensim.State,
        time_s: float,
    ) -> BinaryPhaseDetectorReading:
        try:
            timestamp = float(time_s)
        except (TypeError, ValueError) as exc:
            raise BinaryPhaseDetectorSamplingError(
                "Binary detector timestamp must be numeric."
            ) from exc
        if not math.isfinite(timestamp):
            raise BinaryPhaseDetectorSamplingError(
                "Binary detector timestamp must be finite."
            )
        try:
            self._model.realizePosition(state)
        except Exception as exc:
            raise BinaryPhaseDetectorSamplingError(
                f"Cannot realize position at t={timestamp:.12g}."
            ) from exc

        clearances: dict[str, float] = {}
        contacts: dict[str, bool] = {}
        for point in self._profile.points:
            try:
                station = self._frames[point.name].findStationLocationInGround(
                    state,
                    opensim.Vec3(*point.location),
                )
                station_ground = [float(station.get(i)) for i in range(3)]
            except Exception as exc:
                raise BinaryPhaseDetectorSamplingError(
                    f"Cannot locate binary detector point {point.name!r} "
                    f"at t={timestamp:.12g}."
                ) from exc
            clearance = signed_clearance_m(
                station_ground,
                self._profile.ground,
            )
            clearances[point.name] = clearance
            contacts[point.name] = contact_bit(clearance)

        if tuple(clearances) != BINARY_PHASE_ROLES:
            raise BinaryPhaseDetectorSamplingError(
                "Binary detector roles changed during sampling."
            )
        return BinaryPhaseDetectorReading(
            time_s=timestamp,
            contacts=contacts,
            signed_clearance_m=clearances,
        )
