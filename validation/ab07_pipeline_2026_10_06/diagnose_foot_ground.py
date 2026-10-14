"""Sample foot marker poses against measured GRF without forward dynamics.

This diagnostic uses the unfiltered IK values and OpenSim realizePosition only.
Marker height is not sole clearance; a measured-GRF stance mask is context, not
proof that the foot geometry or a contact profile is dynamically qualified.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from validate_bundle import read_storage, resolve_reference, sha256


def statistics(values: np.ndarray) -> dict:
    if not len(values):
        return {"samples": 0}
    return {"samples": len(values), "min_m": float(values.min()),
            "p05_m": float(np.percentile(values, 5)),
            "median_m": float(np.median(values)),
            "p95_m": float(np.percentile(values, 95)), "max_m": float(values.max())}


def diagnose(args: argparse.Namespace) -> dict:
    import opensim
    from config import SimulatorConfig
    from model_loader import _configure_geometry_search_paths, _load_plugin

    threshold = float(SimulatorConfig().grf_contact_threshold_n)
    model_path, ik_path, external_path = map(Path.resolve, (args.model, args.ik, args.external_loads))
    labels, values, degrees = read_storage(ik_path)
    if (len(values) != 28612
            or not np.allclose(values[[0, -1], 0], [10.36, 153.415], atol=1e-8, rtol=0)
            or not np.allclose(np.diff(values[:, 0]), 0.005, atol=1e-8, rtol=0)):
        raise ValueError("Diagnostic requires the native AB07 trial_01 IK interval")
    if args.sample_step < 1 or not np.isfinite(threshold) or threshold <= 0:
        raise ValueError("Require positive sample step and contact threshold")
    indices = np.unique(np.r_[np.arange(0, len(values), args.sample_step), len(values) - 1])
    times = values[indices, 0]
    sources = {"new_ik": (labels, values, degrees)}

    external = ET.parse(external_path)
    grf_path = resolve_reference(external.findtext(".//datafile"), external_path.parent)
    grf_labels, grf, _ = read_storage(grf_path)
    if grf[0, 0] > times[0] or grf[-1, 0] < times[-1]:
        raise ValueError("Measured GRF does not cover sampled IK")
    forces = {}
    for load in external.findall(".//ExternalForce"):
        body = load.findtext("applied_to_body")
        if body not in {"foot_l", "calcn_r"}:
            raise ValueError(f"Unexpected ExternalForce body: {body}")
        if load.findtext("force_expressed_in_body") != "ground":
            raise ValueError("Vertical force must be expressed in ground")
        identifier = load.findtext("force_identifier")
        column = grf_labels.index(identifier + "y")
        forces["left" if body == "foot_l" else "right"] = np.interp(times, grf[:, 0], grf[:, column])
    if set(forces) != {"left", "right"}:
        raise ValueError("Require both measured GRF sides")

    marker_names = [f"{prefix}_{part}" for prefix in ("L", "R")
                    for part in ("Heel", "Toe_Lat", "Toe_Med", "Toe_Tip")]
    tree = ET.parse(model_path)
    stations = {}
    for name in marker_names:
        marker = tree.find(f".//MarkerSet/objects/Marker[@name='{name}']")
        if marker is None:
            raise ValueError(f"Missing foot marker: {name}")
        local = np.fromstring(marker.findtext("location", ""), sep=" ")
        if local.shape != (3,) or not np.all(np.isfinite(local)):
            raise ValueError(f"Invalid foot marker station: {name}")
        stations[name] = (marker.findtext("socket_parent_frame"), local)
    stations.update({"foot_l_origin": ("/bodyset/foot_l", np.zeros(3)),
                     "calcn_r_origin": ("/bodyset/calcn_r", np.zeros(3))})

    _load_plugin(str(args.plugin))
    _configure_geometry_search_paths(str(model_path))
    model = opensim.Model(str(model_path))
    state = model.initSystem()
    coordinates = model.getCoordinateSet()
    coordinate_names = tuple(coordinates.get(i).getName() for i in range(coordinates.getSize()))
    if labels[1:] != coordinate_names or len(coordinate_names) != 21:
        raise ValueError("IK must contain the model's 21 coordinate names and order")
    frames = {name: opensim.PhysicalFrame.safeDownCast(model.getComponent(frame))
              for name, (frame, _) in stations.items()}
    if any(frame is None for frame in frames.values()):
        raise ValueError("Every foot marker parent must be a PhysicalFrame")
    heights = {}
    for source, (_, source_values, source_degrees) in sources.items():
        positions = source_values[indices, 1:].copy()
        if source_degrees:
            for column, name in enumerate(coordinate_names):
                if name not in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}:
                    positions[:, column] *= np.pi / 180.0
        source_heights = {name: np.empty(len(times)) for name in stations}
        for row, time in enumerate(times):
            state.setTime(float(time))
            for column in range(coordinates.getSize()):
                coordinates.get(column).setValue(state, float(positions[row, column]), False)
            model.realizePosition(state)
            for name, (_, local) in stations.items():
                point = frames[name].findStationLocationInGround(state, opensim.Vec3(*local))
                source_heights[name][row] = float(point.get(1))
        if any(not np.all(np.isfinite(v)) for v in source_heights.values()):
            raise ValueError("Non-finite sampled foot positions")
        heights[source] = source_heights

    summaries = {}
    for source, station_heights in heights.items():
        summaries[source] = {}
        for name, values_y in station_heights.items():
            side = "left" if name.startswith("L_") or name == "foot_l_origin" else "right"
            stance = forces[side] > threshold
            summaries[source][name] = {"all": statistics(values_y),
                                      "measured_grf_stance": statistics(values_y[stance])}
    paths = [model_path, ik_path, external_path, grf_path, REPO / "config.py"]
    return {"status": "PASS", "scope": "Position-only diagnostic; no integration or contact qualification",
            "limitations": "Ground y=0 is a coordinate datum. Marker and body-origin heights do not establish sole clearance. No healthy-model foot comparison or online contact profile is evaluated.",
            "samples": len(times), "sample_step_native_rows": args.sample_step,
            "time_range_s": times[[0, -1]].tolist(), "measured_grf_stance_threshold_n": threshold,
            "threshold_source": "config.py SimulatorConfig.grf_contact_threshold_n (shared AB06/AB07)",
            "coordinate_units_internal": {name: "m" if name in {"pelvis_tx", "pelvis_ty", "pelvis_tz"} else "rad"
                                          for name in coordinate_names},
            "measured_grf": {side: {"peak_vertical_n": float(force.max()),
                                    "stance_fraction": float(np.mean(force > threshold))}
                             for side, force in forces.items()},
            "station_local_m": {name: {"frame": frame, "location": local.tolist()}
                                for name, (frame, local) in stations.items()},
            "ground_y_m": summaries,
            "sha256": {str(path): sha256(path) for path in paths}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    bundle = REPO / "models/AB07_SEASEA_Threadmill"
    parser.add_argument("--model", type=Path, default=bundle / "AB07_SEASEA_stiff321_500_pi.osim")
    parser.add_argument("--ik", type=Path, default=bundle / "data/IK_results_AB07_SEASEA.mot")
    parser.add_argument("--external-loads", type=Path, default=bundle / "data/ExternalForces.xml")
    parser.add_argument("--plugin", type=Path, default=REPO / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff")
    parser.add_argument("--sample-step", type=int, default=10)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = diagnose(args)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: result[key] for key in ("status", "scope", "samples", "time_range_s")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
