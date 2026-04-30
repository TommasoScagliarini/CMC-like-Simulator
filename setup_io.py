"""
setup_io.py
===========
Helpers for simulator setup XML files and the persisted "last setup" state.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

from path_resolver import REPO_ROOT, resolve_repo_path


SETUP_XML_VERSION = "40000"
SETUP_ROOT_TAG = "OpenSimDocument"
SETUP_NODE_TAG = "CMC_Simulator_Setup"
LAST_SETUP_STATE_PATH = REPO_ROOT / ".simulator_last_setup.json"


@dataclass(frozen=True)
class SimulationSetup:
    model_file: Path
    kinematics_file: Path
    external_loads_xml: Path
    reserve_actuators_xml: Path
    t_start: float
    t_end: float


def _xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _resolve_saved_path(raw: str) -> Path:
    text = raw.strip()
    if not text:
        raise ValueError("Setup XML contains an empty path value.")
    return resolve_repo_path(text)


def _repo_relative_or_absolute(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def _format_float(value: float) -> str:
    return f"{value:.15g}"


def _validate_existing_file(
    label: str,
    raw: str | Path,
    suffixes: Iterable[str],
) -> Path:
    path = resolve_repo_path(raw)
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")

    allowed = {suffix.lower() for suffix in suffixes}
    if allowed and path.suffix.lower() not in allowed:
        suffix_text = ", ".join(sorted(allowed))
        raise ValueError(f"{label} must use one of these extensions: {suffix_text}")
    return path.resolve()


def read_kinematics_time_range(raw: str | Path) -> tuple[float, float]:
    path = _validate_existing_file("Kinematics file", raw, (".sto", ".mot"))
    first_time: float | None = None
    last_time: float | None = None

    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line:
                continue

            first_token = line.split()[0]
            try:
                time_value = float(first_token)
            except ValueError:
                continue

            if first_time is None:
                first_time = time_value
            last_time = time_value

    if first_time is None or last_time is None:
        raise ValueError(f"Unable to infer t_start/t_end from kinematics file: {path}")
    return first_time, last_time


def _resolve_time_range(
    kinematics_file: Path,
    t_start: float | str | None,
    t_end: float | str | None,
) -> tuple[float, float]:
    file_t_start, file_t_end = read_kinematics_time_range(kinematics_file)

    start_text = "" if t_start is None else str(t_start).strip()
    end_text = "" if t_end is None else str(t_end).strip()

    resolved_t_start = file_t_start if not start_text else float(start_text)
    resolved_t_end = file_t_end if not end_text else float(end_text)

    if resolved_t_end < resolved_t_start:
        raise ValueError(
            f"Invalid time window: t_end ({resolved_t_end}) < t_start ({resolved_t_start})"
        )
    if resolved_t_start < file_t_start - 1e-9:
        raise ValueError(
            f"t_start ({resolved_t_start}) is earlier than the kinematics file start ({file_t_start})"
        )
    if resolved_t_end > file_t_end + 1e-9:
        raise ValueError(
            f"t_end ({resolved_t_end}) is later than the kinematics file end ({file_t_end})"
        )
    return resolved_t_start, resolved_t_end


def build_simulation_setup(
    model_file: str | Path,
    kinematics_file: str | Path,
    external_loads_xml: str | Path,
    reserve_actuators_xml: str | Path,
    t_start: float | str | None = None,
    t_end: float | str | None = None,
) -> SimulationSetup:
    resolved_model = _validate_existing_file("Model file", model_file, (".osim",))
    resolved_kinematics = _validate_existing_file(
        "Kinematics file",
        kinematics_file,
        (".sto", ".mot"),
    )
    resolved_external = _validate_existing_file(
        "External loads XML",
        external_loads_xml,
        (".xml",),
    )
    resolved_reserve = _validate_existing_file(
        "Reserve actuators XML",
        reserve_actuators_xml,
        (".xml",),
    )
    resolved_t_start, resolved_t_end = _resolve_time_range(
        resolved_kinematics,
        t_start,
        t_end,
    )

    return SimulationSetup(
        model_file=resolved_model,
        kinematics_file=resolved_kinematics,
        external_loads_xml=resolved_external,
        reserve_actuators_xml=resolved_reserve,
        t_start=resolved_t_start,
        t_end=resolved_t_end,
    )


def write_setup_xml(setup: SimulationSetup, destination: str | Path) -> Path:
    target = resolve_repo_path(destination)
    if target.suffix.lower() != ".xml":
        target = target.with_suffix(".xml")
    target.parent.mkdir(parents=True, exist_ok=True)

    root = ET.Element(SETUP_ROOT_TAG, Version=SETUP_XML_VERSION)
    node = ET.SubElement(root, SETUP_NODE_TAG, name=target.stem)

    values = {
        "model_file": setup.model_file,
        "kinematics_file": setup.kinematics_file,
        "external_loads_xml": setup.external_loads_xml,
        "reserve_actuators_xml": setup.reserve_actuators_xml,
        "t_start": _format_float(setup.t_start),
        "t_end": _format_float(setup.t_end),
    }
    for tag, path in values.items():
        child = ET.SubElement(node, tag)
        child.text = (
            _repo_relative_or_absolute(path)
            if isinstance(path, Path)
            else str(path)
        )

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target.resolve()


def read_setup_xml(setup_xml_path: str | Path) -> SimulationSetup:
    setup_path = _validate_existing_file("Setup XML", setup_xml_path, (".xml",))
    tree = ET.parse(setup_path)
    root = tree.getroot()

    if _xml_local_name(root.tag) != SETUP_ROOT_TAG:
        raise ValueError(
            f"Invalid setup XML root: expected {SETUP_ROOT_TAG}, got {root.tag}"
        )

    setup_node = None
    for child in root:
        if _xml_local_name(child.tag) == SETUP_NODE_TAG:
            setup_node = child
            break
    if setup_node is None:
        raise ValueError(f"Missing <{SETUP_NODE_TAG}> node in setup XML: {setup_path}")

    values: dict[str, str] = {}
    for child in setup_node:
        values[_xml_local_name(child.tag)] = (child.text or "").strip()

    required = (
        "model_file",
        "kinematics_file",
        "external_loads_xml",
        "reserve_actuators_xml",
    )
    missing = [name for name in required if not values.get(name)]
    if missing:
        raise ValueError(
            "Setup XML is missing required entries: "
            f"{', '.join(sorted(missing))}"
        )

    return build_simulation_setup(
        _resolve_saved_path(values["model_file"]),
        _resolve_saved_path(values["kinematics_file"]),
        _resolve_saved_path(values["external_loads_xml"]),
        _resolve_saved_path(values["reserve_actuators_xml"]),
        values.get("t_start"),
        values.get("t_end"),
    )


def read_last_setup_path() -> Path | None:
    if not LAST_SETUP_STATE_PATH.is_file():
        return None
    try:
        payload = json.loads(LAST_SETUP_STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return None

    raw = payload.get("setup_xml_path")
    if not isinstance(raw, str) or not raw.strip():
        return None
    return resolve_repo_path(raw).resolve()


def write_last_setup_state(setup_xml_path: str | Path) -> Path:
    setup_path = resolve_repo_path(setup_xml_path).resolve()
    payload = {
        "setup_xml_path": _repo_relative_or_absolute(setup_path),
        "loaded_at": datetime.now().isoformat(timespec="seconds"),
    }
    LAST_SETUP_STATE_PATH.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    return LAST_SETUP_STATE_PATH


def ask_open_setup_xml_path(initial_path: str | Path | None = None) -> Path | None:
    import tkinter as tk
    from tkinter import filedialog

    initial_dir = (REPO_ROOT / "models").resolve()

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    root.update()
    previous_cwd = Path.cwd()
    try:
        os.chdir(initial_dir)
        root.tk.call("cd", str(initial_dir))
        filename = filedialog.askopenfilename(
            title="Select simulator setup XML",
            initialdir=str(initial_dir),
            initialfile="",
            filetypes=[
                ("Simulator setup XML", "*.xml"),
                ("All files", "*.*"),
            ],
        )
    finally:
        os.chdir(previous_cwd)
        root.destroy()

    if not filename:
        return None
    return Path(filename).resolve()
