"""Read-only check: does the contact produce a smooth heel->toe COP rollover?

Compares profiles on the same forward states. For the chosen side, during
contact, reports COP longitudinal range, smoothness (max sample-to-sample jump,
count of large jumps), monotonic-forward fraction, peak vertical force and peak
penetration. No dynamics are run.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import load_online_grf_profile  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _calculate_wrench,
    _sample_spheres_from_coordinate_states,
)


def _analyze(setup, profile_path, states, side, sea_plugin, threshold):
    profile = load_online_grf_profile(str(resolve_repo_path(profile_path)))
    times, samples = _sample_spheres_from_coordinate_states(
        setup, profile, str(resolve_repo_path(states)), sea_plugin
    )
    wrench = _calculate_wrench(profile, samples)
    fz = wrench[side]["force"][:, 1]  # vertical
    cop = wrench[side]["cop"]
    # longitudinal axis = world x (treadmill walking direction)
    cop_long = cop[:, 0]
    contact = fz > threshold
    n_contact = int(np.sum(contact))
    out = {
        "profile": str(resolve_repo_path(profile_path).name),
        "n_spheres_side": sum(1 for s in profile.spheres if s.side == side),
        "samples": int(len(times)),
        "contact_fraction": float(np.mean(contact)),
        "peak_vertical_force_n": float(np.nanmax(fz)),
    }
    if n_contact >= 3:
        cl = cop_long[contact]
        cl = cl[np.isfinite(cl)]
        d = np.diff(cl)
        out.update(
            {
                "cop_long_range_m": float(np.nanmax(cl) - np.nanmin(cl)),
                "cop_max_jump_m": float(np.nanmax(np.abs(d))) if d.size else 0.0,
                "cop_large_jumps_gt_15mm": int(np.sum(np.abs(d) > 0.015)),
                "cop_forward_fraction": float(np.mean(d > 0)) if d.size else 0.0,
                "cop_std_of_steps_m": float(np.nanstd(d)) if d.size else 0.0,
            }
        )
    pen = []
    normal = np.array(profile.ground.normal, float)
    normal /= np.linalg.norm(normal)
    origin = np.array(profile.ground.origin, float)
    for s in profile.spheres:
        if s.side != side:
            continue
        c = samples["centers"][s.name]
        p = s.radius - ((c - origin) @ normal)
        pen.append(np.maximum(0.0, p))
    out["peak_penetration_mm"] = float(np.max(pen) * 1000.0) if pen else None
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--states-sto", required=True)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument("--profiles", nargs="+", required=True)
    parser.add_argument("--report", default="")
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    results = [
        _analyze(setup, p, args.states_sto, args.side, args.sea_plugin, args.threshold)
        for p in args.profiles
    ]
    for r in results:
        print(json.dumps(r, indent=2))
    if args.report:
        resolve_repo_path(args.report).write_text(
            json.dumps({"side": args.side, "results": results}, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
