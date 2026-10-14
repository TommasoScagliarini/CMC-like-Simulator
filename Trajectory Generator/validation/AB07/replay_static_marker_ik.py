"""Re-solve a short static marker interval without scaling or dynamics.

Use the production marker-IK setup writer and existing calibrated models.
This diagnostic writes only its own setup, short motion and receipt.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import opensim as osim

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "tools"))
from run_opensim_sea_pipeline import build_ik_setup, default_plugin  # noqa: E402
from calibrate_ab06_seasea_markers import (  # noqa: E402
    from_vec3, interpolate_motion, load_plugin, parse_motion, parse_trc,
    set_state_from_motion, vec3,
)


def fp(path: Path) -> dict:
    return {"path": str(path.relative_to(REPO)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def storage(path: Path):
    lines = path.read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    body = [line for line in lines[end+1:] if line.strip()]
    return body[0].split(), np.loadtxt(body[1:], ndmin=2)


def main() -> None:
    output = REPO / "validation/ab07_detector_investigation_2026_10_06/stage2/static_ik_replay"
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    plugin = default_plugin()
    load_plugin(plugin)
    osim.Logger.setLevelString("error")
    profile_path = REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
    profile = json.loads(profile_path.read_text())
    before = {"script": fp(Path(__file__)), "plugin": fp(plugin), "binary_profile": fp(profile_path),
              "ik_setup_writer": fp(REPO / "scripts/run_opensim_sea_pipeline.py")}
    cases = {}
    for subject, trial in (("AB06", "static_01"), ("AB07", "static01")):
        model_path = REPO / f"models/{subject}_SEASEA_Threadmill/{subject}_SEASEA_stiff321_500_pi.osim"
        static_dir = REPO / f"models/{subject}_SEASEA-raw/data/converted/static/{trial}"
        trc_path = static_dir / f"{trial}.trc"
        remap_path = static_dir / f"{trial}_ik_dataset_ab06_seasea.mot"
        for label, path in (("model", model_path), ("trc", trc_path), ("remap", remap_path)):
            before[f"{subject}_{label}"] = fp(path)
        trc, remap = parse_trc(trc_path), parse_motion(remap_path)
        directory = output / subject
        directory.mkdir()
        motion_path = directory / "static_marker_ik_0p04s.mot"
        setup = build_ik_setup(directory / "ik_setup.xml", model_path, trc_path,
                               list(trc.markers), directory, str(motion_path), 0.0, 0.04, 1.0)
        tool = osim.InverseKinematicsTool(str(setup))
        if tool.run() is False:
            raise RuntimeError(f"Static IK failed: {subject}")
        motion = parse_motion(motion_path)
        model = osim.Model(str(model_path)); state = model.initSystem()
        body = model.getBodySet().get("foot_l")
        evaluated = {}
        for name, qmotion in (("recomputed_static_ik", motion), ("healthy_remap_baseline", remap)):
            sensors = []
            for time in motion.times:
                set_state_from_motion(model, state, interpolate_motion(qmotion, float(time)), qmotion.in_degrees)
                xyz = np.stack([from_vec3(body.findStationLocationInGround(state, vec3(p["location"]))) for p in profile["points"]])
                clearance = (xyz - np.array(profile["ground"]["origin"])) @ np.array(profile["ground"]["normal"])
                sensors.append(clearance)
            clearance = np.stack(sensors)
            evaluated[name] = {p["name"]: {"mean_clearance_m": float(clearance[:, i].mean()),
                "min_clearance_m": float(clearance[:, i].min()), "max_clearance_m": float(clearance[:, i].max()),
                "contact_fraction": float(np.mean(clearance[:, i] <= 0))} for i, p in enumerate(profile["points"])}
        error_path = directory / "static_marker_ik_0p04s_ik_marker_errors.sto"
        labels, errors = storage(error_path)
        cases[subject] = {"samples": len(motion.times), "time_range_s": [float(motion.times[0]), float(motion.times[-1])],
            "sensor_geometry": evaluated, "marker_error_columns": labels,
            "marker_error_column_means": dict(zip(labels[1:], errors[:, 1:].mean(axis=0).tolist())),
            "setup": fp(setup), "motion": fp(motion_path), "marker_errors": fp(error_path)}
    assert all(fp(REPO / receipt["path"])["sha256"] == receipt["sha256"] for receipt in before.values())
    receipt = {"schema_version": 1, "scope": "Native marker IK only over 0.04s static interval, no forward dynamics/policy/full pipeline",
        "sources": before, "source_files_unchanged": True, "cases": cases,
        "interpretation": "Checks whether marker-only IK corrects the geometric heel deficit already present in the calibration pose.",
        "limitations": ["Short static diagnostic only; no dynamic contact qualification.",
                         "Joint constraints have infinite weight; they do not impose foot-ground contact."]}
    target = output.parent / "static_ik_replay.json"
    target.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(target), "cases": cases}))


if __name__ == "__main__":
    main()
