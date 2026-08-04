#!/usr/bin/env python3
"""Portable EPIC AB06 MATLAB-table converter.

The EPIC files are MATLAB v5 files whose public variable is a ``table``
stored as an MCOS object.  SciPy exposes the object as ``MatlabOpaque`` and
keeps the actual table payload in ``__function_workspace__``.  This module
decodes that mini-MAT workspace without requiring MATLAB and writes the four
files needed by the AB06 OpenSim pipeline:

* dataset IK ``.mot``;
* prescribed GRF ``.mot``;
* marker ``.trc``;
* ``ExternalLoads.xml``.

The MCOS representation and the SciPy reader used for it are private formats.
Consequently every structural assumption is checked and conversion fails
closed on any schema drift.  Output directories and individual files are
strictly no-clobber.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence, TextIO
from xml.etree import ElementTree as ET

import numpy as np

try:
    from scipy.io.matlab._mio5 import MatFile5Reader
except ImportError as exc:  # pragma: no cover - depends on the SciPy build.
    raise ImportError(
        "SciPy with scipy.io.matlab._mio5.MatFile5Reader is required to "
        "decode EPIC MATLAB table/MCOS files"
    ) from exc


IK_SOURCE_COLUMNS = (
    "Header",
    "pelvis_tilt",
    "pelvis_list",
    "pelvis_rotation",
    "pelvis_tx",
    "pelvis_ty",
    "pelvis_tz",
    "hip_flexion_r",
    "hip_adduction_r",
    "hip_rotation_r",
    "knee_angle_r",
    "ankle_angle_r",
    "subtalar_angle_r",
    "mtp_angle_r",
    "hip_flexion_l",
    "hip_adduction_l",
    "hip_rotation_l",
    "knee_angle_l",
    "ankle_angle_l",
    "subtalar_angle_l",
    "mtp_angle_l",
    "lumbar_extension",
    "lumbar_bending",
    "lumbar_rotation",
)

TARGET_COORDINATES = (
    "pelvis_tilt",
    "pelvis_list",
    "pelvis_rotation",
    "pelvis_tx",
    "pelvis_ty",
    "pelvis_tz",
    "hip_flexion_r",
    "hip_adduction_r",
    "hip_rotation_r",
    "knee_angle_r",
    "ankle_angle_r",
    "subtalar_angle_r",
    "mtp_angle_r",
    "hip_flexion_l",
    "hip_adduction_l",
    "hip_rotation_l",
    "pros_knee_angle",
    "pros_ankle_angle",
    "lumbar_extension",
    "lumbar_bending",
    "lumbar_rotation",
)

IK_SOURCE_FOR_TARGET = {
    **{name: name for name in TARGET_COORDINATES},
    "pros_knee_angle": "knee_angle_l",
    "pros_ankle_angle": "ankle_angle_l",
}

FP_SOURCE_COLUMNS = (
    "Header",
    "Treadmill_R_vx",
    "Treadmill_R_vy",
    "Treadmill_R_vz",
    "Treadmill_R_px",
    "Treadmill_R_py",
    "Treadmill_R_pz",
    "Treadmill_R_moment_x",
    "Treadmill_R_moment_y",
    "Treadmill_R_moment_z",
    "Treadmill_L_vx",
    "Treadmill_L_vy",
    "Treadmill_L_vz",
    "Treadmill_L_px",
    "Treadmill_L_py",
    "Treadmill_L_pz",
    "Treadmill_L_moment_x",
    "Treadmill_L_moment_y",
    "Treadmill_L_moment_z",
)

GRF_LABELS = (
    "ground_force1_vx",
    "ground_force1_vy",
    "ground_force1_vz",
    "ground_force1_px",
    "ground_force1_py",
    "ground_force1_pz",
    "ground_force2_vx",
    "ground_force2_vy",
    "ground_force2_vz",
    "ground_force2_px",
    "ground_force2_py",
    "ground_force2_pz",
    "ground_torque1_x",
    "ground_torque1_y",
    "ground_torque1_z",
    "ground_torque2_x",
    "ground_torque2_y",
    "ground_torque2_z",
)

MARKER_NAMES = (
    "R_ASIS",
    "L_ASIS",
    "L_PSIS",
    "R_Thigh_Upper",
    "R_Thigh_Front",
    "R_Thigh_Rear",
    "R_Knee_Lat",
    "R_Shank_Upper",
    "R_Shank_Front",
    "R_Shank_Rear",
    "R_Ankle_Lat",
    "R_Heel",
    "R_Toe_Lat",
    "R_Toe_Med",
    "R_Toe_Tip",
    "L_Thigh_Upper",
    "L_Thigh_Front",
    "L_Thigh_Rear",
    "L_Knee_Lat",
    "L_Shank_Upper",
    "L_Shank_Front",
    "L_Shank_Rear",
    "L_Ankle_Lat",
    "L_Heel",
    "L_Toe_Lat",
    "L_Toe_Med",
    "L_Toe_Tip",
    "R_PSIS",
)

MARKER_REQUIRED_COLUMNS = frozenset(
    {"Header"}
    | {
        f"{marker}_{axis}"
        for marker in MARKER_NAMES
        for axis in ("x", "y", "z")
    }
)
MARKER_ALLOWED_EXTRA_COLUMNS = frozenset(
    f"C_{index}_{axis}"
    for index in (28, 29)
    for axis in ("x", "y", "z")
)

REQUIRED_OPENSIM_VERSION = "4.5.2"
IK_ACCURACY = 1.0e-5
CONVERSION_MANIFEST_SCHEMA_VERSION = 1
IK_EXECUTION_RECEIPT_SCHEMA_VERSION = 1
IK_RECEIPT_SCHEMA_VERSION = 1


class ConversionError(RuntimeError):
    """Base class for deterministic conversion failures."""


class TableSchemaError(ConversionError):
    """Raised when a MATLAB/MCOS table violates the frozen schema."""


class NoClobberError(ConversionError):
    """Raised before a conversion would modify an existing destination."""


@dataclass(frozen=True)
class DecodedTable:
    """One decoded MATLAB table with one-dimensional float64 columns."""

    source: Path
    names: tuple[str, ...]
    columns: Mapping[str, np.ndarray]
    nrows: int

    def column(self, name: str) -> np.ndarray:
        try:
            return self.columns[name]
        except KeyError as exc:
            raise TableSchemaError(
                f"{self.source} is missing required column {name!r}"
            ) from exc


@dataclass(frozen=True)
class CoordinateData:
    time: np.ndarray
    labels: tuple[str, ...]
    values: np.ndarray


@dataclass(frozen=True)
class GrfData:
    time: np.ndarray
    labels: tuple[str, ...]
    values: np.ndarray


@dataclass(frozen=True)
class MarkerData:
    time: np.ndarray
    names: tuple[str, ...]
    values: np.ndarray
    units: str
    rate_hz: float


@dataclass(frozen=True)
class PluginSpec:
    loader_basename: Path
    binary: Path


def _single_bytes(value: object, label: str) -> bytes:
    array = np.asarray(value, dtype=object)
    if array.size != 1:
        raise TableSchemaError(f"{label} must contain exactly one byte string")
    item = array.item()
    if not isinstance(item, bytes):
        raise TableSchemaError(f"{label} is not a byte string")
    return item


def _single_exact_integer(value: object, label: str) -> int:
    array = np.asarray(value)
    if array.size != 1:
        raise TableSchemaError(f"{label} must be scalar")
    try:
        number = float(array.item())
    except (TypeError, ValueError) as exc:
        raise TableSchemaError(f"{label} is not numeric") from exc
    if not math.isfinite(number) or number < 1.0 or not number.is_integer():
        raise TableSchemaError(f"{label} is not a positive integer: {number}")
    return int(number)


def _matlab_text(value: object, label: str) -> str:
    array = np.asarray(value)
    if array.size == 0 or array.dtype.kind not in {"U", "S"}:
        raise TableSchemaError(f"{label} is not a MATLAB character array")
    flat = array.ravel(order="F").tolist()
    if array.dtype.kind == "S":
        try:
            text = b"".join(flat).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise TableSchemaError(f"{label} is not valid UTF-8") from exc
    else:
        text = "".join(str(item) for item in flat)
    if not text:
        raise TableSchemaError(f"{label} is empty")
    return text


def _validate_root_table(root: Mapping[str, object], source: Path) -> None:
    opaque = root.get("None")
    if not isinstance(opaque, np.ndarray) or opaque.size != 1:
        raise TableSchemaError(f"{source} has no single public MCOS table")
    if opaque.dtype.names != ("s0", "s1", "s2", "arr"):
        raise TableSchemaError(f"{source} public MCOS descriptor drifted")
    record = opaque.reshape(-1)[0]
    if not (
        record["s0"] == b"data"
        and record["s1"] == b"MCOS"
        and record["s2"] == b"table"
    ):
        raise TableSchemaError(f"{source} public object is not data/MCOS/table")


def _read_workspace_wrapper(
    reader: MatFile5Reader,
    workspace: np.ndarray,
    source: Path,
) -> np.ndarray:
    if workspace.dtype != np.uint8 or workspace.ndim != 2:
        raise TableSchemaError(f"{source} MCOS workspace is not a uint8 matrix")
    stream = io.BytesIO(workspace.tobytes(order="C"))
    version = stream.read(2)
    endian = stream.read(2)
    padding = stream.read(4)
    if version != b"\x00\x01" or endian not in {b"IM", b"MI"}:
        raise TableSchemaError(f"{source} MCOS mini-MAT header drifted")
    if padding != b"\x00\x00\x00\x00":
        raise TableSchemaError(f"{source} MCOS mini-MAT padding drifted")

    reader.mat_stream = stream
    reader.byte_order = "<" if endian == b"IM" else ">"
    reader.initialize_read()
    try:
        header, _next_position = reader.read_var_header()
        wrapper = reader.read_var_array(header, process=False)
    except Exception as exc:
        raise TableSchemaError(f"cannot decode {source} MCOS workspace") from exc

    if header.name not in {None, b""}:
        raise TableSchemaError(f"{source} MCOS wrapper unexpectedly has a name")
    if not isinstance(wrapper, np.ndarray) or wrapper.size != 1:
        raise TableSchemaError(f"{source} MCOS FileWrapper has invalid cardinality")
    if wrapper.dtype.names != ("MCOS",):
        raise TableSchemaError(f"{source} MCOS FileWrapper schema drifted")
    file_wrapper = wrapper["MCOS"].item()
    if not isinstance(file_wrapper, np.ndarray) or file_wrapper.size != 1:
        raise TableSchemaError(f"{source} has no single MCOS FileWrapper object")
    if file_wrapper.dtype.names != ("s0", "s1", "s2", "arr"):
        raise TableSchemaError(f"{source} FileWrapper descriptor drifted")
    if not (
        _single_bytes(file_wrapper["s0"], "FileWrapper.s0") == b""
        and _single_bytes(file_wrapper["s1"], "FileWrapper.s1") == b"MCOS"
        and _single_bytes(file_wrapper["s2"], "FileWrapper.s2")
        == b"FileWrapper__"
    ):
        raise TableSchemaError(f"{source} is not an MCOS FileWrapper__")
    payload = np.asarray(file_wrapper["arr"].item(), dtype=object)
    if payload.shape != (11, 1):
        raise TableSchemaError(
            f"{source} FileWrapper payload shape drifted: {payload.shape}"
        )
    return payload.ravel(order="F")


def decode_mcos_table(path: str | Path) -> DecodedTable:
    """Decode one EPIC MATLAB v5 table and reject all schema ambiguity."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"EPIC MATLAB table not found: {source}")
    if source.suffix.lower() != ".mat":
        raise TableSchemaError(f"EPIC source must be a .mat file: {source}")

    try:
        with source.open("rb") as stream:
            reader = MatFile5Reader(stream, struct_as_record=True)
            root = reader.get_variables()
    except Exception as exc:
        raise TableSchemaError(f"cannot read MATLAB v5 file: {source}") from exc
    _validate_root_table(root, source)
    workspace = root.get("__function_workspace__")
    if not isinstance(workspace, np.ndarray):
        raise TableSchemaError(f"{source} has no MCOS function workspace")
    payload = _read_workspace_wrapper(reader, workspace, source)

    data_cells = np.asarray(payload[2], dtype=object).ravel(order="F")
    nrows = _single_exact_integer(payload[4], "FileWrapper.nrows")
    nvars = _single_exact_integer(payload[6], "FileWrapper.nvars")
    name_cells = np.asarray(payload[7], dtype=object).ravel(order="F")
    if data_cells.size != nvars or name_cells.size != nvars:
        raise TableSchemaError(
            f"{source} table cardinality drifted: "
            f"data={data_cells.size}, names={name_cells.size}, expected={nvars}"
        )

    names = tuple(
        _matlab_text(value, f"table variable name {index}")
        for index, value in enumerate(name_cells)
    )
    if len(set(names)) != len(names):
        raise TableSchemaError(f"{source} contains duplicate table names")

    columns: dict[str, np.ndarray] = {}
    for name, value in zip(names, data_cells):
        array = np.asarray(value)
        if array.dtype.kind not in {"b", "i", "u", "f"}:
            raise TableSchemaError(f"{source} column {name!r} is not numeric")
        if array.size != nrows:
            raise TableSchemaError(
                f"{source} column {name!r} has {array.size} rows, expected {nrows}"
            )
        column = np.asarray(array, dtype=np.float64).reshape(-1, order="F")
        if np.any(np.isinf(column)):
            raise TableSchemaError(f"{source} column {name!r} contains infinity")
        column.setflags(write=False)
        columns[name] = column
    return DecodedTable(
        source=source,
        names=names,
        columns=columns,
        nrows=nrows,
    )


def _validate_time(time: np.ndarray, source: Path) -> np.ndarray:
    result = np.asarray(time, dtype=np.float64).reshape(-1)
    if result.size < 2 or not np.all(np.isfinite(result)):
        raise TableSchemaError(f"{source} has an invalid time vector")
    if np.any(np.diff(result) <= 0.0):
        raise TableSchemaError(f"{source} time is not strictly increasing")
    return result


def _require_exact_names(
    table: DecodedTable,
    expected: Sequence[str],
    label: str,
) -> None:
    if table.names != tuple(expected):
        missing = sorted(set(expected) - set(table.names))
        extra = sorted(set(table.names) - set(expected))
        raise TableSchemaError(
            f"{table.source} {label} schema drifted; "
            f"missing={missing}, extra={extra}, order_changed="
            f"{not missing and not extra}"
        )


def extract_coordinate_data(table: DecodedTable) -> CoordinateData:
    """Map the EPIC IK table to the AB06_SEASEA coordinate convention."""
    _require_exact_names(table, IK_SOURCE_COLUMNS, "IK")
    time = _validate_time(table.column("Header"), table.source)
    values = np.column_stack(
        [table.column(IK_SOURCE_FOR_TARGET[name]) for name in TARGET_COORDINATES]
    )
    if values.shape != (table.nrows, len(TARGET_COORDINATES)):
        raise TableSchemaError(f"{table.source} IK matrix shape drifted")
    if not np.all(np.isfinite(values)):
        raise TableSchemaError(f"{table.source} IK contains non-finite values")
    return CoordinateData(time=time, labels=TARGET_COORDINATES, values=values)


def _triplet(table: DecodedTable, prefix: str) -> np.ndarray:
    return np.column_stack(
        [table.column(f"{prefix}{axis}") for axis in ("x", "y", "z")]
    )


def _auto_scale(data: np.ndarray, *, threshold: float, scaled: float) -> float:
    finite = np.asarray(data, dtype=np.float64)[np.isfinite(data)]
    if finite.size == 0:
        return 1.0
    return scaled if float(np.median(np.abs(finite))) > threshold else 1.0


def extract_grf_data(table: DecodedTable) -> GrfData:
    """Reproduce the MATLAB AB06 force-plate mapping and torque conversion."""
    _require_exact_names(table, FP_SOURCE_COLUMNS, "force-plate")
    time = _validate_time(table.column("Header"), table.source)

    forces = [_triplet(table, f"Treadmill_{side}_v") for side in ("L", "R")]
    points = [_triplet(table, f"Treadmill_{side}_p") for side in ("L", "R")]
    moments = [
        _triplet(table, f"Treadmill_{side}_moment_") for side in ("L", "R")
    ]
    if not all(np.all(np.isfinite(item)) for item in (*forces, *points, *moments)):
        raise TableSchemaError(f"{table.source} force-plate data are non-finite")

    point_scale = _auto_scale(
        np.concatenate(points, axis=0), threshold=20.0, scaled=0.001
    )
    torque_scale = _auto_scale(
        np.concatenate(moments, axis=0), threshold=1000.0, scaled=0.001
    )
    points = [item * point_scale for item in points]
    moments = [item * torque_scale for item in moments]

    free_vertical: list[np.ndarray] = []
    for force, point, moment in zip(forces, points, moments):
        free = moment - np.cross(point, force)
        vertical = np.zeros_like(free)
        vertical[:, 1] = free[:, 1]
        free_vertical.append(vertical)
    values = np.column_stack(
        (
            forces[0],
            points[0],
            forces[1],
            points[1],
            free_vertical[0],
            free_vertical[1],
        )
    )
    if values.shape != (table.nrows, len(GRF_LABELS)):
        raise TableSchemaError(f"{table.source} GRF matrix shape drifted")
    return GrfData(time=time, labels=GRF_LABELS, values=values)


def _infer_rate(time: np.ndarray, fallback_hz: float) -> float:
    differences = np.diff(np.asarray(time, dtype=np.float64))
    valid = differences[np.isfinite(differences) & (differences > 0.0)]
    return fallback_hz if valid.size == 0 else 1.0 / float(np.median(valid))


def extract_marker_data(table: DecodedTable) -> MarkerData:
    """Extract the 28-marker AB06 TRC payload from an EPIC marker table."""
    observed = set(table.names)
    missing = sorted(MARKER_REQUIRED_COLUMNS - observed)
    extras = sorted(observed - MARKER_REQUIRED_COLUMNS)
    if missing or not set(extras).issubset(MARKER_ALLOWED_EXTRA_COLUMNS):
        raise TableSchemaError(
            f"{table.source} marker schema drifted; missing={missing}, extras={extras}"
        )
    time = _validate_time(table.column("Header"), table.source)
    values = np.empty((table.nrows, len(MARKER_NAMES), 3), dtype=np.float64)
    for marker_index, marker in enumerate(MARKER_NAMES):
        for axis_index, axis in enumerate(("x", "y", "z")):
            values[:, marker_index, axis_index] = table.column(
                f"{marker}_{axis}"
            )
    if np.any(np.isinf(values)):
        raise TableSchemaError(f"{table.source} markers contain infinity")
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        raise TableSchemaError(f"{table.source} markers contain no finite values")
    units = "mm" if float(np.median(np.abs(finite))) > 20.0 else "m"
    return MarkerData(
        time=time,
        names=MARKER_NAMES,
        values=values,
        units=units,
        rate_hz=_infer_rate(time, 100.0),
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _portable_relative_path(path: Path, base: Path) -> str:
    try:
        relative = os.path.relpath(path.resolve(), start=base.resolve())
    except ValueError as exc:
        raise ConversionError(
            f"cannot make {path} portable relative to {base}"
        ) from exc
    return Path(relative).as_posix()


def _write_json_no_clobber(path: Path, payload: Mapping[str, object]) -> Path:
    with _exclusive_text(path) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return path.resolve()


def _exclusive_text(path: Path) -> TextIO:
    path = path.expanduser().resolve()
    try:
        return path.open("x", encoding="utf-8", newline="\n")
    except FileExistsError as exc:
        raise NoClobberError(f"refusing to overwrite existing file: {path}") from exc


def write_coordinate_mot(path: str | Path, data: CoordinateData) -> Path:
    destination = Path(path).expanduser().resolve()
    if destination.suffix.lower() != ".mot":
        raise ConversionError(f"IK output must use .mot: {destination}")
    if data.values.shape != (data.time.size, len(data.labels)):
        raise ConversionError("IK output shape is inconsistent")
    with _exclusive_text(destination) as stream:
        stream.write("Coordinates\n")
        stream.write("version=1\n")
        stream.write(f"nRows={data.time.size}\n")
        stream.write(f"nColumns={len(data.labels) + 1}\n")
        stream.write("inDegrees=yes\n\n")
        stream.write("Units are S.I. units (second, meters, Newtons, ...)\n")
        stream.write("Angles are in degrees.\n\n")
        stream.write("endheader\n")
        stream.write("time\t" + "\t".join(data.labels) + "\n")
        np.savetxt(
            stream,
            np.column_stack((data.time, data.values)),
            delimiter="\t",
            fmt="%.10f",
        )
    return destination


def write_grf_mot(path: str | Path, data: GrfData) -> Path:
    destination = Path(path).expanduser().resolve()
    if destination.suffix.lower() != ".mot":
        raise ConversionError(f"GRF output must use .mot: {destination}")
    if data.values.shape != (data.time.size, len(data.labels)):
        raise ConversionError("GRF output shape is inconsistent")
    with _exclusive_text(destination) as stream:
        stream.write(f"name {destination.name}\n")
        stream.write(f"datacolumns {len(data.labels) + 1}\n")
        stream.write(f"datarows {data.time.size}\n")
        stream.write(f"range {data.time[0]:.10g} {data.time[-1]:.10g}\n")
        stream.write("endheader\n\n")
        stream.write("time\t" + "\t".join(data.labels) + "\n")
        np.savetxt(
            stream,
            np.column_stack((data.time, data.values)),
            delimiter="\t",
            fmt="%.10f",
        )
    return destination


def write_trc(path: str | Path, data: MarkerData) -> Path:
    destination = Path(path).expanduser().resolve()
    if destination.suffix.lower() != ".trc":
        raise ConversionError(f"marker output must use .trc: {destination}")
    expected_shape = (data.time.size, len(data.names), 3)
    if data.values.shape != expected_shape:
        raise ConversionError(
            f"TRC output shape is {data.values.shape}, expected {expected_shape}"
        )
    frames = np.arange(1, data.time.size + 1, dtype=np.int64)
    rows = np.column_stack((frames, data.time, data.values.reshape(data.time.size, -1)))
    formats = ["%d", "%.10f", *(["%.10f"] * (3 * len(data.names)))]
    rate = f"{data.rate_hz:.10g}"
    with _exclusive_text(destination) as stream:
        stream.write(f"PathFileType\t4\t(X/Y/Z)\t{destination.name}\n")
        stream.write(
            "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\t"
            "OrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n"
        )
        stream.write(
            f"{rate}\t{rate}\t{data.time.size}\t{len(data.names)}\t"
            f"{data.units}\t{rate}\t1\t{data.time.size}\n"
        )
        stream.write("Frame#\tTime")
        for marker in data.names:
            stream.write(f"\t{marker}\t\t")
        stream.write("\n\t")
        for index in range(1, len(data.names) + 1):
            stream.write(f"\tX{index}\tY{index}\tZ{index}")
        stream.write("\n")
        np.savetxt(stream, rows, delimiter="\t", fmt=formats)
    return destination


def write_external_loads_xml(
    path: str | Path,
    grf_path: str | Path,
    *,
    left_body: str = "foot_l",
    right_body: str = "calcn_r",
) -> Path:
    destination = Path(path).expanduser().resolve()
    grf = Path(grf_path).expanduser().resolve()
    if destination.suffix.lower() != ".xml":
        raise ConversionError(f"ExternalLoads output must use .xml: {destination}")
    if grf.parent != destination.parent:
        raise ConversionError(
            "GRF and ExternalLoads must share a directory for portable paths"
        )
    if not grf.is_file():
        raise FileNotFoundError(f"GRF output not found: {grf}")
    if not left_body.strip() or not right_body.strip():
        raise ConversionError("ExternalLoads body names cannot be empty")

    root = ET.Element("OpenSimDocument", Version="40000")
    loads = ET.SubElement(root, "ExternalLoads", name="externalloads")
    objects = ET.SubElement(loads, "objects")
    for name, body, plate in (
        ("externalforce_L", left_body, 1),
        ("externalforce_R", right_body, 2),
    ):
        force = ET.SubElement(objects, "ExternalForce", name=name)
        fields = {
            "applied_to_body": body,
            "force_expressed_in_body": "ground",
            "point_expressed_in_body": "ground",
            "force_identifier": f"ground_force{plate}_v",
            "point_identifier": f"ground_force{plate}_p",
            "torque_identifier": f"ground_torque{plate}_",
            "data_source_name": grf.name,
        }
        for tag, value in fields.items():
            ET.SubElement(force, tag).text = value
    ET.SubElement(loads, "groups")
    ET.SubElement(loads, "datafile").text = grf.name
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", xml_declaration=False)
    with _exclusive_text(destination) as stream:
        stream.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        stream.write(xml)
        stream.write("\n")
    return destination


def _validated_time_range(time_range_s: Sequence[float]) -> tuple[float, float]:
    if len(time_range_s) != 2:
        raise ConversionError("IK time range must contain exactly two values")
    start, end = (float(value) for value in time_range_s)
    if not (math.isfinite(start) and math.isfinite(end) and end > start):
        raise ConversionError(f"invalid IK time range: {[start, end]}")
    return start, end


def _validate_ik_model(path: str | Path) -> Path:
    model = Path(path).expanduser().resolve()
    if not model.is_file():
        raise FileNotFoundError(f"IK model not found: {model}")
    if model.suffix.lower() != ".osim":
        raise ConversionError(f"IK model must use .osim: {model}")
    return model


def _plugin_binary_candidates(loader_basename: Path) -> tuple[Path, ...]:
    name = loader_basename.name
    if sys.platform == "darwin":
        preferred = (loader_basename.with_name(f"lib{name}.dylib"),)
    elif os.name == "nt":
        preferred = (loader_basename.with_suffix(".dll"),)
    else:
        preferred = (loader_basename.with_name(f"lib{name}.so"),)
    portable_fallbacks = (
        loader_basename.with_name(f"lib{name}.dylib"),
        loader_basename.with_suffix(".dll"),
        loader_basename.with_name(f"lib{name}.so"),
    )
    return tuple(dict.fromkeys((*preferred, *portable_fallbacks)))


def _plugin_loader_from_binary(binary: Path) -> Path:
    name = binary.name
    for suffix in (".dylib", ".dll", ".so"):
        if name.endswith(suffix):
            name = name[: -len(suffix)]
            break
    if name.startswith("lib"):
        name = name[3:]
    return binary.with_name(name)


def resolve_plugin_spec(path: str | Path) -> PluginSpec:
    """Resolve a cross-platform OpenSim loader basename to its binary."""
    requested = Path(path).expanduser().resolve()
    if requested.is_file():
        binary = requested
        loader = _plugin_loader_from_binary(binary)
    else:
        loader = requested
        matches = [item for item in _plugin_binary_candidates(loader) if item.is_file()]
        if not matches:
            raise FileNotFoundError(
                f"OpenSim plugin binary not found for loader basename: {loader}"
            )
        binary = matches[0]
    if binary.suffix.lower() not in {".dylib", ".dll", ".so"}:
        raise ConversionError(f"unsupported OpenSim plugin binary: {binary}")
    return PluginSpec(loader_basename=loader, binary=binary)


def write_ik_setup_xml(
    path: str | Path,
    *,
    model_path: str | Path,
    marker_path: str | Path,
    time_range_s: Sequence[float],
    output_motion_file: str,
) -> Path:
    """Write the frozen, equal-weight OpenSim 4.5.2 IK setup.

    The setup is portable: model and marker paths are relative to the setup
    directory, and the results directory is the setup directory itself.
    """
    destination = Path(path).expanduser().resolve()
    if destination.suffix.lower() != ".xml":
        raise ConversionError(f"IK setup output must use .xml: {destination}")
    model = _validate_ik_model(model_path)
    markers = Path(marker_path).expanduser().resolve()
    if not markers.is_file() or markers.suffix.lower() != ".trc":
        raise FileNotFoundError(f"IK marker TRC not found: {markers}")
    motion_name = Path(output_motion_file)
    if (
        motion_name.name != output_motion_file
        or motion_name.suffix.lower() != ".mot"
    ):
        raise ConversionError(
            "IK output motion file must be a local .mot filename"
        )
    start, end = _validated_time_range(time_range_s)

    root = ET.Element("OpenSimDocument", Version="40000")
    tool = ET.SubElement(root, "InverseKinematicsTool")
    fields_before_tasks = (
        ("results_directory", "."),
        ("input_directory", ""),
        (
            "model_file",
            _portable_relative_path(model, destination.parent),
        ),
        ("constraint_weight", "Inf"),
        ("accuracy", "1e-05"),
    )
    for tag, value in fields_before_tasks:
        ET.SubElement(tool, tag).text = value
    task_set = ET.SubElement(tool, "IKTaskSet")
    objects = ET.SubElement(task_set, "objects")
    for marker in MARKER_NAMES:
        task = ET.SubElement(objects, "IKMarkerTask", name=marker)
        ET.SubElement(task, "apply").text = "true"
        ET.SubElement(task, "weight").text = "1"
    ET.SubElement(task_set, "groups")
    trailing_fields = (
        (
            "marker_file",
            _portable_relative_path(markers, destination.parent),
        ),
        ("coordinate_file", "Unassigned"),
        ("time_range", f"{start:.10g} {end:.10g}"),
        ("report_errors", "false"),
        ("output_motion_file", output_motion_file),
        ("report_marker_locations", "false"),
    )
    for tag, value in trailing_fields:
        ET.SubElement(tool, tag).text = value
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", xml_declaration=False)
    with _exclusive_text(destination) as stream:
        stream.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        stream.write(xml)
        stream.write("\n")
    return destination


def _preflight_output_dir(output_dir: Path) -> None:
    resolved = output_dir.expanduser().resolve()
    if resolved.exists():
        raise NoClobberError(
            f"refusing to modify existing output directory: {resolved}"
        )


def _validate_trial_sources(paths: Sequence[Path], trial: str | None) -> str:
    stems = {path.stem for path in paths}
    if len(stems) != 1:
        raise ConversionError(f"IK/FP/marker trial stems differ: {sorted(stems)}")
    inferred = next(iter(stems))
    if trial is not None and trial != inferred:
        raise ConversionError(
            f"explicit trial {trial!r} does not match source stem {inferred!r}"
        )
    return inferred


def convert_trial(
    *,
    ik_mat: str | Path,
    fp_mat: str | Path,
    markers_mat: str | Path,
    output_dir: str | Path,
    trial: str | None = None,
    ik_model: str | Path | None = None,
    ik_plugin: str | Path | None = None,
) -> dict[str, object]:
    """Convert one explicitly named trial into a new no-clobber directory."""
    destination = Path(output_dir).expanduser().resolve()
    _preflight_output_dir(destination)
    sources = tuple(
        Path(item).expanduser().resolve()
        for item in (ik_mat, fp_mat, markers_mat)
    )
    trial_id = _validate_trial_sources(sources, trial)
    resolved_ik_model = None if ik_model is None else _validate_ik_model(ik_model)
    if (resolved_ik_model is None) != (ik_plugin is None):
        raise ConversionError(
            "--ik-model and --ik-plugin must be supplied together"
        )
    plugin_spec = None if ik_plugin is None else resolve_plugin_spec(ik_plugin)

    # Decode and validate every source before creating any destination.
    coordinate_data = extract_coordinate_data(decode_mcos_table(sources[0]))
    grf_data = extract_grf_data(decode_mcos_table(sources[1]))
    marker_data = extract_marker_data(decode_mcos_table(sources[2]))
    if not (
        coordinate_data.time[0] == marker_data.time[0]
        and coordinate_data.time[-1] == marker_data.time[-1]
        and grf_data.time[0] == marker_data.time[0]
        and grf_data.time[-1] == marker_data.time[-1]
    ):
        raise TableSchemaError("IK, FP, and marker time ranges do not match")

    destination.mkdir(parents=True, exist_ok=False)
    ik_path = destination / f"{trial_id}_ik_dataset_ab06_seasea.mot"
    grf_path = destination / f"{trial_id}_grf.mot"
    trc_path = destination / f"{trial_id}.trc"
    external_path = destination / f"{trial_id}_ExternalLoads.xml"
    ik_setup_path = destination / f"{trial_id}_iksetup.xml"
    ik_output_name = f"{trial_id}_ik.mot"
    manifest_path = destination / f"{trial_id}_conversion_manifest.json"
    write_coordinate_mot(ik_path, coordinate_data)
    write_grf_mot(grf_path, grf_data)
    write_trc(trc_path, marker_data)
    write_external_loads_xml(external_path, grf_path)
    inverse_kinematics: dict[str, object] | None = None
    if resolved_ik_model is not None:
        assert plugin_spec is not None
        write_ik_setup_xml(
            ik_setup_path,
            model_path=resolved_ik_model,
            marker_path=trc_path,
            time_range_s=(marker_data.time[0], marker_data.time[-1]),
            output_motion_file=ik_output_name,
        )
        inverse_kinematics = {
            "status": "SETUP_ONLY_NOT_RUN",
            "required_opensim_version": REQUIRED_OPENSIM_VERSION,
            "accuracy": IK_ACCURACY,
            "time_range_s": [
                float(marker_data.time[0]),
                float(marker_data.time[-1]),
            ],
            "model": {
                "path": _portable_relative_path(
                    resolved_ik_model, destination
                ),
                "sha256": _sha256_file(resolved_ik_model),
            },
            "plugin": {
                "loader_basename": _portable_relative_path(
                    plugin_spec.loader_basename, destination
                ),
                "binary_path": _portable_relative_path(
                    plugin_spec.binary, destination
                ),
                "binary_sha256": _sha256_file(plugin_spec.binary),
            },
            "marker_tasks": [
                {"name": name, "apply": True, "weight": 1.0}
                for name in MARKER_NAMES
            ],
            "setup": {
                "path": ik_setup_path.name,
                "sha256": _sha256_file(ik_setup_path),
            },
            "output_motion_file": ik_output_name,
            "output_ik_sha256": None,
            "run_helper_command": [
                "python",
                _portable_relative_path(Path(__file__), destination),
                "--run-ik-setup",
                ik_setup_path.name,
                "--ik-plugin",
                _portable_relative_path(
                    plugin_spec.loader_basename, destination
                ),
            ],
            "receipt_helper_command": [
                "python",
                _portable_relative_path(Path(__file__), destination),
                "--finalize-ik-output",
                ik_output_name,
                "--ik-setup",
                ik_setup_path.name,
                "--ik-plugin",
                _portable_relative_path(
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
        "sources": {
            role: {"filename": path.name, "sha256": _sha256_file(path)}
            for role, path in zip(("ik", "force_plate", "markers"), sources)
        },
        "time_range_s": [
            float(marker_data.time[0]),
            float(marker_data.time[-1]),
        ],
        "outputs": {
            role: {"path": path.name, "sha256": _sha256_file(path)}
            for role, path in (
                ("dataset_ik", ik_path),
                ("grf", grf_path),
                ("markers_trc", trc_path),
                ("external_loads", external_path),
            )
        },
        "inverse_kinematics": inverse_kinematics,
    }
    _write_json_no_clobber(manifest_path, manifest)
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
        "time_range_s": [
            float(marker_data.time[0]),
            float(marker_data.time[-1]),
        ],
        "ik_rows": int(coordinate_data.time.size),
        "grf_rows": int(grf_data.time.size),
        "marker_rows": int(marker_data.time.size),
    }


def _read_json_object(path: Path, label: str) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8", errors="strict"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ConversionError(f"cannot read {label}: {path}") from exc
    if not isinstance(value, dict):
        raise ConversionError(f"{label} is not a JSON object: {path}")
    return value


def _resolve_portable_reference(
    text: str | None,
    *,
    base: Path,
    label: str,
) -> Path:
    if text is None or not text.strip():
        raise ConversionError(f"IK setup has no {label}")
    reference = Path(text.strip())
    if reference.is_absolute():
        raise ConversionError(f"IK setup {label} is not portable: {reference}")
    return (base / reference).resolve()


def _read_trc_marker_contract(path: Path) -> tuple[str, ...]:
    try:
        with path.open("r", encoding="utf-8", errors="strict") as stream:
            lines = [next(stream).rstrip("\r\n") for _ in range(5)]
    except (OSError, UnicodeError, StopIteration) as exc:
        raise ConversionError(f"cannot read IK marker TRC header: {path}") from exc
    keys = lines[1].split("\t")
    values = lines[2].split("\t")
    header = dict(zip(keys, values))
    try:
        marker_count = int(header["NumMarkers"])
    except (KeyError, ValueError) as exc:
        raise ConversionError(f"invalid NumMarkers in IK marker TRC: {path}") from exc
    names = tuple(item for item in lines[3].split("\t")[2:] if item.strip())
    if marker_count != len(MARKER_NAMES) or names != MARKER_NAMES:
        raise ConversionError(
            f"IK marker contract drifted: count={marker_count}, names={names}"
        )
    return names


def _read_ik_output_contract(
    path: Path,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    except (OSError, UnicodeError) as exc:
        raise ConversionError(f"cannot read OpenSim IK output: {path}") from exc
    try:
        end_header = next(
            index for index, line in enumerate(lines) if line.strip() == "endheader"
        )
        label_index = next(
            index
            for index in range(end_header + 1, len(lines))
            if lines[index].strip()
        )
    except StopIteration as exc:
        raise ConversionError(f"invalid OpenSim IK storage header: {path}") from exc
    header: dict[str, str] = {}
    for line in lines[:end_header]:
        if "=" in line:
            key, value = line.split("=", 1)
            header[key.strip()] = value.strip()
    labels = tuple(lines[label_index].split())
    expected_labels = ("time", *TARGET_COORDINATES)
    if labels != expected_labels:
        raise ConversionError(
            f"OpenSim IK coordinate schema drifted: {labels}"
        )
    if header.get("inDegrees", "").lower() != "yes":
        raise ConversionError("OpenSim IK output must declare inDegrees=yes")
    try:
        values = np.loadtxt(path, skiprows=label_index + 1, ndmin=2)
        nrows = int(header["nRows"])
        ncolumns = int(header["nColumns"])
    except (OSError, ValueError, KeyError) as exc:
        raise ConversionError(f"invalid OpenSim IK numeric payload: {path}") from exc
    if values.shape != (nrows, ncolumns) or ncolumns != len(expected_labels):
        raise ConversionError(
            f"OpenSim IK shape drifted: data={values.shape}, "
            f"header={(nrows, ncolumns)}"
        )
    if not np.all(np.isfinite(values)):
        raise ConversionError("OpenSim IK output contains non-finite values")
    time = _validate_time(values[:, 0], path)
    return time, values[:, 1:]


def _validate_ik_output_time_range(
    time: np.ndarray,
    expected: Sequence[float],
) -> None:
    expected_start, expected_end = _validated_time_range(expected)
    tolerance = 5.1e-9
    if not (
        math.isclose(
            float(time[0]), expected_start, rel_tol=0.0, abs_tol=tolerance
        )
        and math.isclose(
            float(time[-1]), expected_end, rel_tol=0.0, abs_tol=tolerance
        )
    ):
        raise ConversionError(
            f"IK output time range {[float(time[0]), float(time[-1])]} "
            f"does not match setup {[expected_start, expected_end]}"
        )


def _read_ik_setup_contract(path: Path) -> dict[str, object]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ConversionError(f"cannot parse IK setup: {path}") from exc
    if root.tag != "OpenSimDocument" or root.get("Version") != "40000":
        raise ConversionError("IK setup is not an OpenSimDocument Version 40000")
    tools = list(root.iter("InverseKinematicsTool"))
    if len(tools) != 1:
        raise ConversionError("IK setup must contain one InverseKinematicsTool")
    tool = tools[0]
    if (tool.findtext("results_directory") or "").strip() != ".":
        raise ConversionError("IK results_directory must be portable '.'")
    try:
        accuracy = float(tool.findtext("accuracy", ""))
    except ValueError as exc:
        raise ConversionError("IK setup accuracy is not numeric") from exc
    if accuracy != IK_ACCURACY:
        raise ConversionError(
            f"IK setup accuracy drifted: {accuracy} != {IK_ACCURACY}"
        )
    tasks = list(tool.findall("./IKTaskSet/objects/IKMarkerTask"))
    task_names = tuple(task.get("name", "") for task in tasks)
    if task_names != MARKER_NAMES:
        raise ConversionError(f"IK marker task schema drifted: {task_names}")
    for task in tasks:
        if (task.findtext("apply") or "").strip().lower() != "true":
            raise ConversionError(f"IK marker {task.get('name')} is not applied")
        try:
            weight = float(task.findtext("weight", ""))
        except ValueError as exc:
            raise ConversionError(
                f"IK marker {task.get('name')} weight is not numeric"
            ) from exc
        if weight != 1.0:
            raise ConversionError(
                f"IK marker {task.get('name')} weight is not 1"
            )
    time_text = (tool.findtext("time_range") or "").split()
    try:
        time_range = _validated_time_range(tuple(float(item) for item in time_text))
    except ValueError as exc:
        raise ConversionError("IK setup time_range is not numeric") from exc
    model = _resolve_portable_reference(
        tool.findtext("model_file"), base=path.parent, label="model_file"
    )
    marker = _resolve_portable_reference(
        tool.findtext("marker_file"), base=path.parent, label="marker_file"
    )
    if not model.is_file() or not marker.is_file():
        raise ConversionError("IK setup model or marker reference does not exist")
    _read_trc_marker_contract(marker)
    output_text = (tool.findtext("output_motion_file") or "").strip()
    output_name = Path(output_text)
    if output_name.name != output_text or output_name.suffix.lower() != ".mot":
        raise ConversionError("IK output_motion_file must be a local .mot filename")
    return {
        "accuracy": accuracy,
        "time_range_s": time_range,
        "model": model,
        "marker": marker,
        "output": (path.parent / output_name).resolve(),
    }


def _require_mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ConversionError(f"conversion manifest {label} is not an object")
    return value


def _load_pinned_opensim() -> tuple[object, str]:
    try:
        import opensim as osim
    except ImportError as exc:
        raise ConversionError(
            "OpenSim Python bindings are required for IK attestation"
        ) from exc
    version = str(getattr(osim, "__version__", ""))
    if version != REQUIRED_OPENSIM_VERSION:
        raise ConversionError(
            f"installed OpenSim version must be {REQUIRED_OPENSIM_VERSION}, "
            f"got {version or 'UNKNOWN'}"
        )
    return osim, version


def _validated_conversion_manifest(
    *,
    setup: Path,
    trial: str,
    setup_contract: Mapping[str, object],
    plugin: PluginSpec,
) -> tuple[Path, dict[str, object]]:
    manifest_path = setup.parent / f"{trial}_conversion_manifest.json"
    manifest = _read_json_object(manifest_path, "conversion manifest")
    if (
        manifest.get("schema_version") != CONVERSION_MANIFEST_SCHEMA_VERSION
        or manifest.get("status") != "CONVERTED"
        or manifest.get("trial") != trial
    ):
        raise ConversionError("conversion manifest identity/status drifted")
    inverse = _require_mapping(
        manifest.get("inverse_kinematics"), "inverse_kinematics"
    )
    model_manifest = _require_mapping(inverse.get("model"), "IK model")
    plugin_manifest = _require_mapping(inverse.get("plugin"), "IK plugin")
    setup_manifest = _require_mapping(inverse.get("setup"), "IK setup")
    if inverse.get("output_ik_sha256") is not None:
        raise ConversionError("immutable conversion manifest already claims an IK hash")
    if inverse.get("required_opensim_version") != REQUIRED_OPENSIM_VERSION:
        raise ConversionError("conversion manifest OpenSim version drifted")
    if inverse.get("accuracy") != IK_ACCURACY:
        raise ConversionError("conversion manifest IK accuracy drifted")
    if inverse.get("marker_tasks") != [
        {"name": name, "apply": True, "weight": 1.0}
        for name in MARKER_NAMES
    ]:
        raise ConversionError("conversion manifest marker task contract drifted")
    if tuple(inverse.get("time_range_s", ())) != tuple(
        setup_contract["time_range_s"]
    ):
        raise ConversionError("conversion manifest IK time range drifted")
    if inverse.get("output_motion_file") != Path(
        setup_contract["output"]
    ).name:
        raise ConversionError("conversion manifest IK output name drifted")

    model = Path(setup_contract["model"])
    model_reference = _resolve_portable_reference(
        str(model_manifest.get("path", "")),
        base=setup.parent,
        label="manifest IK model path",
    )
    if model_reference != model or model_manifest.get("sha256") != _sha256_file(model):
        raise ConversionError("IK model identity/hash does not match manifest")
    if (
        setup_manifest.get("path") != setup.name
        or setup_manifest.get("sha256") != _sha256_file(setup)
    ):
        raise ConversionError("IK setup identity/hash does not match manifest")
    plugin_hash = _sha256_file(plugin.binary)
    if plugin_manifest.get("binary_sha256") != plugin_hash:
        raise ConversionError("IK plugin hash does not match conversion manifest")
    expected_loader = _resolve_portable_reference(
        str(plugin_manifest.get("loader_basename", "")),
        base=setup.parent,
        label="plugin loader_basename",
    )
    expected_binary = _resolve_portable_reference(
        str(plugin_manifest.get("binary_path", "")),
        base=setup.parent,
        label="plugin binary_path",
    )
    if (
        plugin.loader_basename != expected_loader
        or plugin.binary != expected_binary
    ):
        raise ConversionError("IK plugin identity does not match manifest")
    outputs = _require_mapping(manifest.get("outputs"), "outputs")
    marker_output = _require_mapping(outputs.get("markers_trc"), "markers_trc")
    marker = Path(setup_contract["marker"])
    if (
        marker_output.get("path") != marker.name
        or marker_output.get("sha256") != _sha256_file(marker)
    ):
        raise ConversionError("IK marker TRC identity/hash does not match manifest")
    return manifest_path, manifest


def finalize_ik_receipt(
    *,
    setup_xml: str | Path,
    output_ik_mot: str | Path,
    ik_plugin: str | Path,
    opensim_version: str | None = None,
    receipt_path: str | Path | None = None,
) -> dict[str, object]:
    """Validate an OpenSim IK result and write an immutable receipt."""
    setup = Path(setup_xml).expanduser().resolve()
    output = Path(output_ik_mot).expanduser().resolve()
    if not setup.is_file():
        raise FileNotFoundError(f"IK setup not found: {setup}")
    if not output.is_file():
        raise FileNotFoundError(f"OpenSim IK output not found: {output}")
    if not setup.stem.endswith("_iksetup"):
        raise ConversionError(f"cannot infer trial from IK setup name: {setup.name}")
    trial = setup.stem[: -len("_iksetup")]
    receipt = (
        setup.parent / f"{trial}_ik_receipt.json"
        if receipt_path is None
        else Path(receipt_path).expanduser().resolve()
    )
    if receipt.exists():
        raise NoClobberError(f"refusing to overwrite IK receipt: {receipt}")
    if receipt.parent != setup.parent:
        raise ConversionError("IK receipt must share the conversion directory")
    _osim, installed_opensim_version = _load_pinned_opensim()
    if (
        opensim_version is not None
        and opensim_version != installed_opensim_version
    ):
        raise ConversionError(
            f"claimed OpenSim version {opensim_version!r} does not match "
            f"installed version {installed_opensim_version!r}"
        )

    setup_contract = _read_ik_setup_contract(setup)
    if output != setup_contract["output"]:
        raise ConversionError(
            f"IK output path {output} does not match setup "
            f"{setup_contract['output']}"
        )
    time, coordinates = _read_ik_output_contract(output)
    _validate_ik_output_time_range(time, setup_contract["time_range_s"])
    plugin = resolve_plugin_spec(ik_plugin)
    manifest_path, manifest = _validated_conversion_manifest(
        setup=setup,
        trial=trial,
        setup_contract=setup_contract,
        plugin=plugin,
    )
    inverse = _require_mapping(
        manifest.get("inverse_kinematics"), "inverse_kinematics"
    )
    model_manifest = _require_mapping(inverse.get("model"), "IK model")
    plugin_manifest = _require_mapping(inverse.get("plugin"), "IK plugin")
    model = Path(setup_contract["model"])
    plugin_hash = _sha256_file(plugin.binary)

    execution_path = setup.parent / f"{trial}_ik_execution_receipt.json"
    execution = _read_json_object(execution_path, "IK execution receipt")
    if (
        execution.get("schema_version") != IK_EXECUTION_RECEIPT_SCHEMA_VERSION
        or execution.get("status") != "IK_EXECUTED"
        or execution.get("trial") != trial
        or execution.get("opensim_version") != installed_opensim_version
    ):
        raise ConversionError("IK execution receipt identity/version drifted")
    execution_manifest = _require_mapping(
        execution.get("conversion_manifest"), "execution conversion_manifest"
    )
    execution_setup = _require_mapping(execution.get("setup"), "execution setup")
    execution_model = _require_mapping(execution.get("model"), "execution model")
    execution_plugin = _require_mapping(
        execution.get("plugin"), "execution plugin"
    )
    execution_output = _require_mapping(
        execution.get("output_ik"), "execution output_ik"
    )
    expected_marker_contract = {
        "count": len(MARKER_NAMES),
        "names": list(MARKER_NAMES),
        "apply": True,
        "weight": 1.0,
        "accuracy": IK_ACCURACY,
    }
    expected_time_range = [float(time[0]), float(time[-1])]
    if execution_manifest != {
        "path": manifest_path.name,
        "sha256": _sha256_file(manifest_path),
    }:
        raise ConversionError("IK execution receipt manifest link drifted")
    if execution_setup != {
        "path": setup.name,
        "sha256": _sha256_file(setup),
    }:
        raise ConversionError("IK execution receipt setup link drifted")
    if execution_model != {
        "path": str(model_manifest.get("path")),
        "sha256": _sha256_file(model),
    }:
        raise ConversionError("IK execution receipt model link drifted")
    if execution_plugin != {
        "loader_basename": str(plugin_manifest.get("loader_basename")),
        "binary_path": str(plugin_manifest.get("binary_path")),
        "binary_sha256": plugin_hash,
    }:
        raise ConversionError("IK execution receipt plugin link drifted")
    if execution.get("marker_contract") != expected_marker_contract:
        raise ConversionError("IK execution receipt marker contract drifted")
    if execution_output != {
        "path": output.name,
        "sha256": _sha256_file(output),
        "rows": int(time.size),
        "coordinate_count": int(coordinates.shape[1]),
        "coordinates": list(TARGET_COORDINATES),
        "time_range_s": expected_time_range,
    }:
        raise ConversionError("IK execution receipt output link drifted")

    payload: dict[str, object] = {
        "schema_version": IK_RECEIPT_SCHEMA_VERSION,
        "status": "IK_OUTPUT_VERIFIED",
        "trial": trial,
        "opensim_version": installed_opensim_version,
        "conversion_manifest": {
            "path": manifest_path.name,
            "sha256": _sha256_file(manifest_path),
        },
        "execution_receipt": {
            "path": execution_path.name,
            "sha256": _sha256_file(execution_path),
        },
        "setup": {"path": setup.name, "sha256": _sha256_file(setup)},
        "model": {
            "path": str(model_manifest.get("path")),
            "sha256": _sha256_file(model),
        },
        "plugin": {
            "loader_basename": str(plugin_manifest.get("loader_basename")),
            "binary_path": str(plugin_manifest.get("binary_path")),
            "binary_sha256": plugin_hash,
        },
        "marker_contract": expected_marker_contract,
        "output_ik": {
            "path": output.name,
            "sha256": _sha256_file(output),
            "rows": int(time.size),
            "coordinate_count": int(coordinates.shape[1]),
            "coordinates": list(TARGET_COORDINATES),
            "time_range_s": expected_time_range,
        },
    }
    _write_json_no_clobber(receipt, payload)
    return {"status": "ok", "receipt": receipt.as_posix(), **payload}


def run_ik_from_setup(
    *,
    setup_xml: str | Path,
    ik_plugin: str | Path,
) -> dict[str, object]:
    """Run IK and emit a same-process, hash-linked execution receipt."""
    setup = Path(setup_xml).expanduser().resolve()
    if not setup.is_file():
        raise FileNotFoundError(f"IK setup not found: {setup}")
    if not setup.stem.endswith("_iksetup"):
        raise ConversionError(f"cannot infer trial from IK setup name: {setup.name}")
    trial = setup.stem[: -len("_iksetup")]
    execution_receipt = setup.parent / f"{trial}_ik_execution_receipt.json"
    if execution_receipt.exists():
        raise NoClobberError(
            f"refusing to overwrite IK execution receipt: {execution_receipt}"
        )
    contract = _read_ik_setup_contract(setup)
    output = Path(contract["output"])
    if output.exists():
        raise NoClobberError(f"refusing to overwrite OpenSim IK output: {output}")
    plugin = resolve_plugin_spec(ik_plugin)
    manifest_path, manifest = _validated_conversion_manifest(
        setup=setup,
        trial=trial,
        setup_contract=contract,
        plugin=plugin,
    )
    osim, opensim_version = _load_pinned_opensim()
    previous = Path.cwd()
    try:
        os.chdir(setup.parent)
        osim.LoadOpenSimLibrary(str(plugin.loader_basename))
        tool = osim.InverseKinematicsTool(setup.name)
        succeeded = tool.run()
    finally:
        os.chdir(previous)
    if succeeded is False or not output.is_file():
        raise ConversionError("OpenSim InverseKinematicsTool did not produce output")
    time, coordinates = _read_ik_output_contract(output)
    _validate_ik_output_time_range(time, contract["time_range_s"])
    inverse = _require_mapping(
        manifest.get("inverse_kinematics"), "inverse_kinematics"
    )
    model_manifest = _require_mapping(inverse.get("model"), "IK model")
    plugin_manifest = _require_mapping(inverse.get("plugin"), "IK plugin")
    execution_payload: dict[str, object] = {
        "schema_version": IK_EXECUTION_RECEIPT_SCHEMA_VERSION,
        "status": "IK_EXECUTED",
        "trial": trial,
        "opensim_version": opensim_version,
        "conversion_manifest": {
            "path": manifest_path.name,
            "sha256": _sha256_file(manifest_path),
        },
        "setup": {"path": setup.name, "sha256": _sha256_file(setup)},
        "model": {
            "path": str(model_manifest.get("path")),
            "sha256": _sha256_file(Path(contract["model"])),
        },
        "plugin": {
            "loader_basename": str(plugin_manifest.get("loader_basename")),
            "binary_path": str(plugin_manifest.get("binary_path")),
            "binary_sha256": _sha256_file(plugin.binary),
        },
        "marker_contract": {
            "count": len(MARKER_NAMES),
            "names": list(MARKER_NAMES),
            "apply": True,
            "weight": 1.0,
            "accuracy": IK_ACCURACY,
        },
        "output_ik": {
            "path": output.name,
            "sha256": _sha256_file(output),
            "rows": int(time.size),
            "coordinate_count": int(coordinates.shape[1]),
            "coordinates": list(TARGET_COORDINATES),
            "time_range_s": [float(time[0]), float(time[-1])],
        },
    }
    _write_json_no_clobber(execution_receipt, execution_payload)
    return {
        "status": "ok",
        "setup": setup.as_posix(),
        "output_ik": output.as_posix(),
        "execution_receipt": execution_receipt.as_posix(),
        "execution_receipt_sha256": _sha256_file(execution_receipt),
        "plugin_binary_sha256": _sha256_file(plugin.binary),
        "opensim_version": opensim_version,
        "next_step": "finalize immutable IK receipt",
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert one EPIC AB06 MATLAB-table trial without MATLAB, run a "
            "generated IK setup, or finalize its immutable IK receipt."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--run-ik-setup",
        help="Run this generated IK setup after loading --ik-plugin.",
    )
    mode.add_argument(
        "--finalize-ik-output",
        help="Validate this OpenSim IK .mot and create an immutable receipt.",
    )
    parser.add_argument("--ik-mat")
    parser.add_argument("--fp-mat")
    parser.add_argument("--markers-mat")
    parser.add_argument("--output-dir")
    parser.add_argument(
        "--trial",
        help="Optional exact trial id; otherwise all three source stems must agree.",
    )
    parser.add_argument(
        "--ik-model",
        help="Optional calibrated .osim model; requires --ik-plugin.",
    )
    parser.add_argument(
        "--ik-plugin",
        help=(
            "SEA plugin binary or loader basename without lib prefix/extension."
        ),
    )
    parser.add_argument(
        "--ik-setup",
        help="Generated setup associated with --finalize-ik-output.",
    )
    parser.add_argument(
        "--opensim-version",
        help="Exact OpenSim version attested by receipt finalization.",
    )
    parser.add_argument(
        "--receipt-path",
        help="Optional receipt path; defaults beside the IK setup.",
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
