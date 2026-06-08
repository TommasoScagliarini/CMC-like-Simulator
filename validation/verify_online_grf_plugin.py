"""Verify that the offline onlineGRF evaluator matches the C++ plugin."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import opensim

from model_loader import _load_plugin
from online_grf import add_online_grf_forces, load_online_grf_profile, read_online_grf
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import _calculate_grf, _sample_spheres


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--sample-dt", type=float, default=0.25)
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    parser.add_argument("--report", default="results/online_grf_plugin_audit.json")
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    profile = load_online_grf_profile(args.profile)
    times = np.arange(setup.t_start, setup.t_end + args.sample_dt * 0.25, args.sample_dt)
    samples = _sample_spheres(setup, profile, times, args.sea_plugin)
    expected = _calculate_grf(profile, samples)

    _load_plugin(str(resolve_repo_path(args.sea_plugin)))
    _load_plugin(str(resolve_repo_path("plugins/OnlineGRFContact")))
    model = opensim.Model(str(setup.model_file))
    force_paths, force_sides = add_online_grf_forces(
        model,
        profile,
        applies_force=False,
    )
    state = model.initSystem()
    coord_set = model.getCoordinateSet()

    actual = {
        "left": np.zeros_like(expected["left"]),
        "right": np.zeros_like(expected["right"]),
    }
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator

    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(setup.model_file.parent)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.t_start = float(times[0])
    cfg.t_end = float(times[-1])
    kin = KinematicsInterpolator(cfg)

    for row, time in enumerate(times):
        q, qdot, _ = kin.get(float(time))
        state.setTime(float(time))
        for i in range(coord_set.getSize()):
            coord = coord_set.get(i)
            name = coord.getName()
            if name in q:
                coord.setValue(state, float(q[name]), False)
                coord.setSpeedValue(state, float(qdot[name]))
        grf = read_online_grf(model, state, force_paths, force_sides)
        for side in ("left", "right"):
            actual[side][row] = grf["sides"][side]["force"]

    report = {"samples": int(len(times)), "sample_dt": float(args.sample_dt), "sides": {}}
    maximum = 0.0
    for side in ("left", "right"):
        error = actual[side] - expected[side]
        side_maximum = float(np.max(np.abs(error)))
        maximum = max(maximum, side_maximum)
        report["sides"][side] = {
            "rmse_n": np.sqrt(np.mean(error * error, axis=0)).tolist(),
            "max_abs_error_n": side_maximum,
        }
    report["max_abs_error_n"] = maximum
    report["equivalent_within_1e-8_n"] = bool(maximum <= 1.0e-8)

    destination = resolve_repo_path(args.report)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["equivalent_within_1e-8_n"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
