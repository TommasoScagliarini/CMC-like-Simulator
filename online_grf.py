"""
online_grf.py
=============
Model-agnostic online ground-reaction-force support.

The runtime is configured by a small JSON profile that describes a ground
plane, contact material, and contact spheres attached to model frames.  The
profile is intentionally separate from the .osim model so the same simulator
code can support different subjects and prosthesis geometries.

Three simulator modes are supported:

``prescribed``
    Existing ExternalLoads-only CMC-like behaviour.
``online_sensor``
    ExternalLoads still drive dynamics; online contact forces are evaluated
    without applying them to the model.
``online``
    Online contacts drive dynamics. ExternalLoads, when provided, are retained
    only as a validation oracle and are not added to the ForceSet.
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Mapping, Sequence

import numpy as np
import opensim


GRF_MODES = {"prescribed", "online_sensor", "online"}
ONLINE_GRF_COMPONENT_TYPE = "OnlineGRFSphereHalfSpaceForce"
ONLINE_GRF_RECORD_WIDTH = 18


def _vec3(values: Sequence[float], label: str) -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{label} must contain three finite numbers.")
    return float(arr[0]), float(arr[1]), float(arr[2])


def _normalise(values: Sequence[float], label: str) -> tuple[float, float, float]:
    arr = np.asarray(_vec3(values, label), dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        raise ValueError(f"{label} must be non-zero.")
    arr /= norm
    return float(arr[0]), float(arr[1]), float(arr[2])


@dataclass(frozen=True)
class OnlineGRFMaterial:
    stiffness: float = 1.0e6
    exponent: float = 1.5
    dissipation: float = 2.0
    static_friction: float = 0.8
    dynamic_friction: float = 0.8
    viscous_friction: float = 0.0
    transition_velocity: float = 0.2
    smoothing: float = 1.0e-5

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "OnlineGRFMaterial":
        values = dict(raw or {})
        known = {name for name in cls.__dataclass_fields__}
        material = cls(**{key: values[key] for key in values if key in known})
        for name in known:
            value = float(getattr(material, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"material.{name} must be finite and non-negative.")
        if material.stiffness <= 0.0:
            raise ValueError("material.stiffness must be positive.")
        if material.exponent <= 0.0:
            raise ValueError("material.exponent must be positive.")
        if material.transition_velocity <= 0.0:
            raise ValueError("material.transition_velocity must be positive.")
        if material.smoothing <= 0.0:
            raise ValueError("material.smoothing must be positive.")
        return material

    def to_dict(self) -> dict:
        return {
            name: float(getattr(self, name))
            for name in self.__dataclass_fields__
        }


@dataclass(frozen=True)
class OnlineGRFGround:
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    normal: tuple[float, float, float] = (0.0, 1.0, 0.0)
    surface_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "OnlineGRFGround":
        values = dict(raw or {})
        return cls(
            origin=_vec3(values.get("origin", cls.origin), "ground.origin"),
            normal=_normalise(values.get("normal", cls.normal), "ground.normal"),
            surface_velocity=_vec3(
                values.get("surface_velocity", cls.surface_velocity),
                "ground.surface_velocity",
            ),
        )

    def to_dict(self) -> dict:
        return {
            "origin": list(self.origin),
            "normal": list(self.normal),
            "surface_velocity": list(self.surface_velocity),
        }


@dataclass(frozen=True)
class OnlineGRFSphere:
    name: str
    side: str
    frame: str
    location: tuple[float, float, float]
    radius: float
    material: OnlineGRFMaterial | None = None
    residual_force_ratio: tuple[float, float, float] = (0.0, 0.0, 0.0)
    residual_moment_ratio_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    residual_penetration_reference_m: float = 0.0
    residual_force_penetration_gain_per_m: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )
    residual_force_penetration_rate_gain_s_per_m: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object]) -> "OnlineGRFSphere":
        name = str(raw.get("name", "")).strip()
        side = str(raw.get("side", "")).strip().lower()
        frame = str(raw.get("frame", "")).strip()
        radius = float(raw.get("radius", 0.0))
        if not name:
            raise ValueError("Each onlineGRF sphere requires a non-empty name.")
        if side not in {"left", "right"}:
            raise ValueError(f"Sphere {name!r} side must be 'left' or 'right'.")
        if not frame:
            raise ValueError(f"Sphere {name!r} requires a frame component path.")
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError(f"Sphere {name!r} radius must be positive.")
        residual_reference = float(raw.get("residual_penetration_reference_m", 0.0))
        if not math.isfinite(residual_reference) or residual_reference < 0.0:
            raise ValueError(
                f"sphere {name} residual_penetration_reference_m "
                "must be finite and non-negative."
            )
        return cls(
            name=name,
            side=side,
            frame=frame,
            location=_vec3(raw.get("location", ()), f"sphere {name} location"),
            radius=radius,
            material=(
                OnlineGRFMaterial.from_mapping(raw["material"])
                if raw.get("material") is not None
                else None
            ),
            residual_force_ratio=_vec3(
                raw.get("residual_force_ratio", (0.0, 0.0, 0.0)),
                f"sphere {name} residual_force_ratio",
            ),
            residual_moment_ratio_m=_vec3(
                raw.get("residual_moment_ratio_m", (0.0, 0.0, 0.0)),
                f"sphere {name} residual_moment_ratio_m",
            ),
            residual_penetration_reference_m=residual_reference,
            residual_force_penetration_gain_per_m=_vec3(
                raw.get(
                    "residual_force_penetration_gain_per_m",
                    (0.0, 0.0, 0.0),
                ),
                f"sphere {name} residual_force_penetration_gain_per_m",
            ),
            residual_force_penetration_rate_gain_s_per_m=_vec3(
                raw.get(
                    "residual_force_penetration_rate_gain_s_per_m",
                    (0.0, 0.0, 0.0),
                ),
                f"sphere {name} residual_force_penetration_rate_gain_s_per_m",
            ),
        )

    def to_dict(self) -> dict:
        values = {
            "name": self.name,
            "side": self.side,
            "frame": self.frame,
            "location": list(self.location),
            "radius": float(self.radius),
        }
        if self.material is not None:
            values["material"] = self.material.to_dict()
        if any(value != 0.0 for value in self.residual_force_ratio):
            values["residual_force_ratio"] = list(self.residual_force_ratio)
        if any(value != 0.0 for value in self.residual_moment_ratio_m):
            values["residual_moment_ratio_m"] = list(
                self.residual_moment_ratio_m
            )
        if self.residual_penetration_reference_m != 0.0:
            values["residual_penetration_reference_m"] = float(
                self.residual_penetration_reference_m
            )
        if any(
            value != 0.0
            for value in self.residual_force_penetration_gain_per_m
        ):
            values["residual_force_penetration_gain_per_m"] = list(
                self.residual_force_penetration_gain_per_m
            )
        if any(
            value != 0.0
            for value in self.residual_force_penetration_rate_gain_s_per_m
        ):
            values["residual_force_penetration_rate_gain_s_per_m"] = list(
                self.residual_force_penetration_rate_gain_s_per_m
            )
        return values


@dataclass(frozen=True)
class OnlineGRFProfile:
    ground: OnlineGRFGround
    material: OnlineGRFMaterial
    spheres: tuple[OnlineGRFSphere, ...]
    heel_strike_confirmation_threshold_n: float | None = None
    source: str = "explicit"
    version: int = 1
    metadata: Mapping[str, object] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object]) -> "OnlineGRFProfile":
        version = int(raw.get("version", 1))
        if version != 1:
            raise ValueError(f"Unsupported onlineGRF profile version: {version}")
        spheres_raw = raw.get("spheres", ())
        if not isinstance(spheres_raw, list) or not spheres_raw:
            raise ValueError("onlineGRF profile requires a non-empty spheres list.")
        spheres = tuple(OnlineGRFSphere.from_mapping(item) for item in spheres_raw)
        names = [sphere.name for sphere in spheres]
        if len(names) != len(set(names)):
            raise ValueError("onlineGRF sphere names must be unique.")
        sides = {sphere.side for sphere in spheres}
        if sides != {"left", "right"}:
            raise ValueError("onlineGRF profile must contain left and right spheres.")
        confirmation_raw = raw.get("heel_strike_confirmation_threshold_n")
        confirmation = (
            None if confirmation_raw is None else float(confirmation_raw)
        )
        if confirmation is not None and (
            not math.isfinite(confirmation) or confirmation <= 0.0
        ):
            raise ValueError(
                "heel_strike_confirmation_threshold_n must be positive."
            )
        return cls(
            version=version,
            source=str(raw.get("source", "explicit")),
            ground=OnlineGRFGround.from_mapping(raw.get("ground")),
            material=OnlineGRFMaterial.from_mapping(raw.get("material")),
            spheres=spheres,
            heel_strike_confirmation_threshold_n=confirmation,
            metadata=dict(raw.get("metadata") or {}),
        )

    def to_dict(self) -> dict:
        values = {
            "version": self.version,
            "source": self.source,
            "ground": self.ground.to_dict(),
            "material": self.material.to_dict(),
            "spheres": [sphere.to_dict() for sphere in self.spheres],
            "metadata": dict(self.metadata),
        }
        if self.heel_strike_confirmation_threshold_n is not None:
            values["heel_strike_confirmation_threshold_n"] = float(
                self.heel_strike_confirmation_threshold_n
            )
        return values


def load_online_grf_profile(path: str | os.PathLike[str]) -> OnlineGRFProfile:
    profile_path = Path(path)
    if not profile_path.is_file():
        raise FileNotFoundError(f"onlineGRF profile not found: {profile_path}")
    with profile_path.open("r", encoding="utf-8") as fh:
        payload = json.load(fh)
    if not isinstance(payload, dict):
        raise ValueError(f"onlineGRF profile root must be an object: {profile_path}")
    return OnlineGRFProfile.from_mapping(payload)


def write_online_grf_profile(
    profile: OnlineGRFProfile,
    path: str | os.PathLike[str],
) -> Path:
    profile_path = Path(path)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    profile_path.write_text(
        json.dumps(profile.to_dict(), indent=2) + "\n",
        encoding="utf-8",
    )
    return profile_path.resolve()


def _set_property(component: opensim.OpenSimObject, name: str, value) -> None:
    prop = component.updPropertyByName(name)
    if isinstance(value, bool):
        opensim.PropertyHelper.setValueBool(value, prop)
    elif isinstance(value, str):
        opensim.PropertyHelper.setValueString(value, prop)
    elif isinstance(value, (tuple, list, np.ndarray)) and len(value) == 3:
        for index, item in enumerate(_vec3(value, name)):
            opensim.PropertyHelper.setValueVec3(item, prop, index)
    else:
        opensim.PropertyHelper.setValueDouble(float(value), prop)


def _new_online_grf_force(
    model: opensim.Model,
    sphere: OnlineGRFSphere,
    profile: OnlineGRFProfile,
    applies_force: bool,
) -> opensim.Force:
    obj = opensim.OpenSimObject.newInstanceOfType(ONLINE_GRF_COMPONENT_TYPE)
    if obj is None:
        raise RuntimeError(
            f"OpenSim type {ONLINE_GRF_COMPONENT_TYPE!r} is not registered. "
            "Build and load the onlineGRF contact plugin."
        )
    force = opensim.Force.safeDownCast(obj)
    if force is None:
        raise RuntimeError(f"{ONLINE_GRF_COMPONENT_TYPE} is not an OpenSim Force.")
    force.setName(f"online_grf_{sphere.name}")
    _set_property(force, "appliesForce", applies_force)
    _set_property(force, "sphere_location", sphere.location)
    _set_property(force, "sphere_radius", sphere.radius)
    _set_property(force, "plane_origin", profile.ground.origin)
    _set_property(force, "plane_normal", profile.ground.normal)
    _set_property(force, "surface_velocity", profile.ground.surface_velocity)
    _set_property(force, "residual_force_ratio", sphere.residual_force_ratio)
    _set_property(
        force,
        "residual_moment_ratio_m",
        sphere.residual_moment_ratio_m,
    )
    _set_property(
        force,
        "residual_penetration_reference_m",
        sphere.residual_penetration_reference_m,
    )
    _set_property(
        force,
        "residual_force_penetration_gain_per_m",
        sphere.residual_force_penetration_gain_per_m,
    )
    _set_property(
        force,
        "residual_force_penetration_rate_gain_s_per_m",
        sphere.residual_force_penetration_rate_gain_s_per_m,
    )
    material = sphere.material or profile.material
    for name, value in material.to_dict().items():
        _set_property(force, name, value)
    frame = model.getComponent(sphere.frame)
    force.updSocket("sphere_frame").connect(frame)
    return force


def add_online_grf_forces(
    model: opensim.Model,
    profile: OnlineGRFProfile,
    *,
    applies_force: bool,
    apply_sides: Iterable[str] | None = None,
) -> tuple[list[str], Dict[str, str]]:
    """Add one registered onlineGRF force per profile sphere.

    If ``apply_sides`` is given, the per-sphere ``appliesForce`` flag is set to
    ``sphere.side in apply_sides`` (hybrid per-side application). Otherwise the
    global ``applies_force`` is used for every sphere (legacy behaviour).
    """
    apply_set = (
        None
        if apply_sides is None
        else {str(side).strip().lower() for side in apply_sides}
    )
    force_paths: list[str] = []
    force_sides: Dict[str, str] = {}
    for sphere in profile.spheres:
        sphere_applies = (
            applies_force if apply_set is None else (sphere.side in apply_set)
        )
        force = _new_online_grf_force(model, sphere, profile, sphere_applies)
        name = force.getName()
        model.addForce(force)
        force_paths.append(f"/forceset/{name}")
        force_sides[name] = sphere.side
    return force_paths, force_sides


def _record_values(force: opensim.Force, state: opensim.State) -> np.ndarray:
    values = force.getRecordValues(state)
    result = np.array([float(values.get(i)) for i in range(values.getSize())])
    if result.size != ONLINE_GRF_RECORD_WIDTH:
        raise RuntimeError(
            f"Unexpected onlineGRF record width for {force.getName()!r}: "
            f"{result.size}, expected {ONLINE_GRF_RECORD_WIDTH}."
        )
    return result


def online_grf_column_names() -> list[str]:
    columns = []
    for side in ("left", "right"):
        columns.extend(
            [
                f"{side}_force_x",
                f"{side}_force_y",
                f"{side}_force_z",
                f"{side}_moment_x",
                f"{side}_moment_y",
                f"{side}_moment_z",
                f"{side}_cop_x",
                f"{side}_cop_y",
                f"{side}_cop_z",
                f"{side}_normal_force",
                f"{side}_penetration",
                f"{side}_slip_speed",
                f"{side}_in_contact",
            ]
        )
    return columns


def read_online_grf(
    model: opensim.Model,
    state: opensim.State,
    force_paths: Iterable[str],
    force_sides: Mapping[str, str],
) -> dict:
    """
    Aggregate online contact forces into left/right GRF, ground moment and COP.

    Component record layout:
      force(3), moment-about-ground-origin(3), contact-point(3),
      normal-force, penetration, slip-speed, sphere-center(3), normal(3).
    """
    aggregated = {
        side: {
            "force": np.zeros(3),
            "moment": np.zeros(3),
            "weighted_point": np.zeros(3),
            "normal_force": 0.0,
            "penetration": 0.0,
            "slip_speed": 0.0,
            "in_contact": False,
        }
        for side in ("left", "right")
    }
    per_sphere = {}
    model.realizeDynamics(state)
    for component_path in force_paths:
        force = opensim.Force.safeDownCast(model.getComponent(component_path))
        if force is None:
            raise RuntimeError(f"onlineGRF component is not a Force: {component_path}")
        values = _record_values(force, state)
        name = force.getName()
        side = force_sides[name]
        force_vec = values[0:3]
        moment = values[3:6]
        point = values[6:9]
        normal_force = max(0.0, float(values[9]))
        penetration = max(0.0, float(values[10]))
        slip_speed = max(0.0, float(values[11]))
        normal = values[15:18]
        cop_point = point + penetration * normal
        bucket = aggregated[side]
        bucket["force"] += force_vec
        bucket["moment"] += moment
        bucket["weighted_point"] += normal_force * cop_point
        bucket["normal_force"] += normal_force
        bucket["penetration"] = max(bucket["penetration"], penetration)
        bucket["slip_speed"] = max(bucket["slip_speed"], slip_speed)
        bucket["in_contact"] = bool(bucket["in_contact"] or penetration > 0.0)
        per_sphere[name] = {
            "side": side,
            "force": force_vec.copy(),
            "moment": moment.copy(),
            "point": point.copy(),
            "cop_point": cop_point.copy(),
            "normal_force": normal_force,
            "penetration": penetration,
            "slip_speed": slip_speed,
        }

    for bucket in aggregated.values():
        if bucket["normal_force"] > 1e-12:
            bucket["cop"] = bucket["weighted_point"] / bucket["normal_force"]
        else:
            bucket["cop"] = np.full(3, np.nan)
        del bucket["weighted_point"]

    return {"sides": aggregated, "spheres": per_sphere}


def flatten_online_grf(grf: Mapping[str, object]) -> np.ndarray:
    values: list[float] = []
    sides = grf["sides"]
    for side in ("left", "right"):
        item = sides[side]
        values.extend(np.asarray(item["force"], dtype=float))
        values.extend(np.asarray(item["moment"], dtype=float))
        values.extend(np.asarray(item["cop"], dtype=float))
        values.extend(
            [
                float(item["normal_force"]),
                float(item["penetration"]),
                float(item["slip_speed"]),
                float(bool(item["in_contact"])),
            ]
        )
    return np.asarray(values, dtype=float)


class StreamingGaitEventDetector:
    """Dual-threshold streaming heel-strike/toe-off detector per side."""

    def __init__(
        self,
        threshold_n: float,
        min_contact_duration_s: float,
        min_cycle_duration_s: float,
        confirmation_threshold_n: float | None = None,
    ) -> None:
        self.threshold_n = float(threshold_n)
        self.confirmation_threshold_n = (
            self.threshold_n
            if confirmation_threshold_n is None
            else max(self.threshold_n, float(confirmation_threshold_n))
        )
        self.min_contact_duration_s = float(min_contact_duration_s)
        self.min_cycle_duration_s = float(min_cycle_duration_s)
        self.reset()

    def reset(self) -> None:
        self._state = {
            side: {
                "candidate_start": None,
                "confirmed_start": None,
                "last_hs": None,
                "above": False,
                "confirmation_reached": False,
            }
            for side in ("left", "right")
        }

    def update(self, time: float, vertical_forces: Mapping[str, float]) -> list[dict]:
        events: list[dict] = []
        for side in ("left", "right"):
            value = float(vertical_forces.get(side, 0.0))
            above = value > self.threshold_n
            item = self._state[side]
            if above and not item["above"]:
                item["candidate_start"] = float(time)
                item["confirmation_reached"] = (
                    value > self.confirmation_threshold_n
                )
            if above and item["candidate_start"] is not None:
                item["confirmation_reached"] = bool(
                    item["confirmation_reached"]
                    or value > self.confirmation_threshold_n
                )
                duration = float(time) - float(item["candidate_start"])
                if (
                    item["confirmed_start"] is None
                    and item["confirmation_reached"]
                    and duration >= self.min_contact_duration_s
                ):
                    hs = float(item["candidate_start"])
                    last_hs = item["last_hs"]
                    cycle_duration = None if last_hs is None else hs - float(last_hs)
                    if last_hs is None or cycle_duration >= self.min_cycle_duration_s:
                        events.append(
                            {
                                "time": hs,
                                "confirmed_time": float(time),
                                "side": side,
                                "event": "heel_strike",
                                "cycle_duration_s": cycle_duration,
                                "confirmation_latency_s": float(time) - hs,
                                "confirmation_threshold_n": (
                                    self.confirmation_threshold_n
                                ),
                            }
                        )
                        item["last_hs"] = hs
                    item["confirmed_start"] = hs
            if not above and item["above"]:
                if item["confirmed_start"] is not None:
                    events.append(
                        {
                            "time": float(time),
                            "confirmed_time": float(time),
                            "side": side,
                            "event": "toe_off",
                            "contact_duration_s": (
                                float(time) - float(item["confirmed_start"])
                            ),
                        }
                    )
                item["candidate_start"] = None
                item["confirmed_start"] = None
                item["confirmation_reached"] = False
            item["above"] = above
        return events


_SIDE_PATTERNS = {
    "left": (r"(^|[_\-.])l($|[_\-.])", r"left"),
    "right": (r"(^|[_\-.])r($|[_\-.])", r"right"),
}
_HEEL_TOKENS = ("heel", "calc", "cal", "hee")
_TOE_TOKENS = ("toe", "mtp")


def _infer_marker_side(name: str) -> str | None:
    text = name.lower()
    for side, patterns in _SIDE_PATTERNS.items():
        if any(re.search(pattern, text) for pattern in patterns):
            return side
    return None


def _marker_kind(name: str) -> str | None:
    text = name.lower()
    if any(token in text for token in _HEEL_TOKENS):
        return "heel"
    if any(token in text for token in _TOE_TOKENS):
        return "toe"
    return None


def infer_profile_from_markers(
    model: opensim.Model,
    *,
    radius: float = 0.035,
    ground: OnlineGRFGround | None = None,
    material: OnlineGRFMaterial | None = None,
) -> OnlineGRFProfile:
    """
    Infer heel/toe contact spheres from marker names and parent frames.

    The result must be materialised and reviewed before runtime use. This keeps
    marker-name heuristics out of production simulations.
    """
    marker_set = model.getMarkerSet()
    candidates: Dict[tuple[str, str], list] = {}
    for i in range(marker_set.getSize()):
        marker = marker_set.get(i)
        side = _infer_marker_side(marker.getName())
        kind = _marker_kind(marker.getName())
        if side is None or kind is None:
            continue
        location = marker.get_location()
        frame_path = marker.getParentFrame().getAbsolutePathString()
        candidates.setdefault((side, kind), []).append(
            (
                marker.getName(),
                frame_path,
                (float(location.get(0)), float(location.get(1)), float(location.get(2))),
            )
        )

    spheres = []
    missing = []
    for side in ("left", "right"):
        for kind in ("heel", "toe"):
            found = candidates.get((side, kind), [])
            if not found:
                missing.append(f"{side}_{kind}")
                continue
            # Prefer the marker with the simplest/shortest name. This selects
            # R_Heel over cluster-marker variants and remains deterministic.
            marker_name, frame_path, location = sorted(found, key=lambda item: (len(item[0]), item[0]))[0]
            spheres.append(
                OnlineGRFSphere(
                    name=f"{side}_{kind}",
                    side=side,
                    frame=frame_path,
                    location=location,
                    radius=float(radius),
                )
            )
    if missing:
        raise ValueError(
            "Could not infer required heel/toe markers: " + ", ".join(missing)
        )
    return OnlineGRFProfile(
        source="marker_inference",
        ground=ground or OnlineGRFGround(),
        material=material or OnlineGRFMaterial(),
        spheres=tuple(spheres),
        metadata={"marker_inference": "heel/toe aliases"},
    )
