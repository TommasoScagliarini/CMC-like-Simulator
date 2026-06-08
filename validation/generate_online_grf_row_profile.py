"""Generate a physical heel->toe ROW contact profile anchored to foot markers.

Unlike the calibrate-to-prescribed basis, this profile is NOT fit to the
measured GRF. It lays a uniform longitudinal row of spheres from the heel marker
to the toe marker (optionally in 2 lateral columns), with physical Hunt-Crossley
material sized to bear body weight, and auto-fits the ground height so peak
penetration over a reference trajectory hits a small target. The goal is a smooth
heel->midstance->toe COP rollover, not magnitude fidelity to the force plate.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import (  # noqa: E402
    OnlineGRFGround,
    OnlineGRFMaterial,
    OnlineGRFProfile,
    OnlineGRFSphere,
    _infer_marker_side,
    _marker_kind,
    write_online_grf_profile,
)
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _sample_spheres_from_coordinate_states,
)


def _heel_toe_markers(model) -> dict:
    """Return {side: {'heel': (frame, loc), 'toe': (frame, loc)}}."""
    marker_set = model.getMarkerSet()
    found: dict = {}
    for i in range(marker_set.getSize()):
        marker = marker_set.get(i)
        side = _infer_marker_side(marker.getName())
        kind = _marker_kind(marker.getName())
        if side is None or kind is None:
            continue
        loc = marker.get_location()
        entry = (
            marker.getName(),
            marker.getParentFrame().getAbsolutePathString(),
            np.array([float(loc.get(0)), float(loc.get(1)), float(loc.get(2))]),
        )
        found.setdefault((side, kind), []).append(entry)
    out: dict = {}
    for (side, kind), items in found.items():
        name, frame, loc = sorted(items, key=lambda it: (len(it[0]), it[0]))[0]
        out.setdefault(side, {})[kind] = (frame, loc)
    return out


def _build_row_spheres(
    side: str,
    frame: str,
    heel: np.ndarray,
    toe: np.ndarray,
    *,
    n_long: int,
    n_lat: int,
    half_width: float,
    radius: float,
    flat_sole: bool = True,
    rocker_radius: float = 0.0,
    apex_fraction: float = 0.45,
) -> list[OnlineGRFSphere]:
    # Toe markers usually sit on top of the toe (high), heel markers near the
    # sole (low). Interpolating their heights makes anterior spheres float and
    # the COP never rolls to the toe. Flatten the sole to the lowest marker
    # height so every sphere can engage; the heel->toe roll then comes from foot
    # pitch during stance.
    if flat_sole:
        sole_y = min(float(heel[1]), float(toe[1]))
        heel = np.array([heel[0], sole_y, heel[2]])
        toe = np.array([toe[0], sole_y, toe[2]])
    long_vec = toe - heel
    long_len = float(np.linalg.norm(long_vec))
    # Lateral axis: perpendicular to the longitudinal axis, in the sole (x-z)
    # plane (ground normal is +y). For a foot lying roughly in x-z this is
    # robust; degenerate cases fall back to the model z axis.
    up = np.array([0.0, 1.0, 0.0])
    lat = np.cross(up, long_vec)
    if np.linalg.norm(lat) < 1e-9:
        lat = np.array([0.0, 0.0, 1.0])
    lat = lat / np.linalg.norm(lat)
    if n_lat == 1:
        lat_offsets = [0.0]
    else:
        lat_offsets = list(np.linspace(-half_width, half_width, n_lat))

    # Rocker: lift sphere centres along +y on a convex-down circular arc so the
    # lowest point sits at apex_fraction and the heel/toe ends ride up. As the
    # foot pitches the contact point (and COP) then migrates heel<->toe. A flat
    # sole (rocker_radius<=0) keeps all centres coplanar and the COP stays put.
    def _rocker_lift(t: float) -> float:
        if rocker_radius <= 0.0:
            return 0.0
        arc = (t - apex_fraction) * long_len  # along-axis distance from apex [m]
        return rocker_radius - float(np.sqrt(max(0.0, rocker_radius**2 - arc**2)))

    spheres: list[OnlineGRFSphere] = []
    for i in range(n_long):
        t = i / (n_long - 1) if n_long > 1 else 0.0
        center_long = heel + t * long_vec
        center_long = center_long + _rocker_lift(t) * up
        for j, off in enumerate(lat_offsets):
            center = center_long + off * lat
            spheres.append(
                OnlineGRFSphere(
                    name=f"{side}_row_{i:02d}_{j}",
                    side=side,
                    frame=frame,
                    location=(float(center[0]), float(center[1]), float(center[2])),
                    radius=float(radius),
                )
            )
    return spheres


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--out-profile", required=True)
    parser.add_argument(
        "--states-sto",
        required=True,
        help="Forward CoordinateStates used to auto-fit the ground height.",
    )
    parser.add_argument("--n-long", type=int, default=6)
    parser.add_argument("--n-lat", type=int, default=2)
    parser.add_argument("--half-width", type=float, default=0.03)
    parser.add_argument("--radius", type=float, default=0.02)
    parser.add_argument(
        "--rocker-radius",
        type=float,
        default=0.0,
        help="Convex-down rocker radius [m] for the sole (0 = flat). Smaller = "
        "more curved = faster heel->toe COP migration with foot pitch.",
    )
    parser.add_argument(
        "--apex-fraction",
        type=float,
        default=0.45,
        help="Heel->toe fraction of the rocker's lowest point.",
    )
    parser.add_argument("--target-penetration-m", type=float, default=0.006)
    parser.add_argument(
        "--stiffness",
        type=float,
        default=1.0e6,
        help="Per-contact Hunt-Crossley stiffness. Lower = softer = fewer "
        "impact spikes but deeper penetration.",
    )
    parser.add_argument("--dissipation", type=float, default=2.0)
    parser.add_argument(
        "--fit-side",
        default="left",
        choices=["left", "right"],
        help="Side whose stance is used to auto-fit the ground height.",
    )
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    import opensim
    from model_loader import _load_plugin

    setup = read_setup_xml(args.setup)
    _load_plugin(str(resolve_repo_path(args.sea_plugin)))
    model = opensim.Model(str(setup.model_file))

    markers = _heel_toe_markers(model)
    material = OnlineGRFMaterial(
        stiffness=args.stiffness, dissipation=args.dissipation
    )  # physical: n=1.5, mu=0.8
    spheres: list[OnlineGRFSphere] = []
    for side in ("left", "right"):
        if side not in markers or "heel" not in markers[side] or "toe" not in markers[side]:
            raise ValueError(f"Missing heel/toe markers for {side}.")
        frame, heel = markers[side]["heel"]
        toe_frame, toe = markers[side]["toe"]
        if toe_frame != frame:
            # heel/toe should share the foot frame; if not, keep heel frame and
            # express toe in it is out of scope -> require same frame.
            raise ValueError(
                f"{side} heel/toe markers live on different frames "
                f"({frame} vs {toe_frame})."
            )
        spheres.extend(
            _build_row_spheres(
                side,
                frame,
                heel,
                toe,
                n_long=args.n_long,
                n_lat=args.n_lat,
                half_width=args.half_width,
                radius=args.radius,
                rocker_radius=args.rocker_radius,
                apex_fraction=args.apex_fraction,
            )
        )

    profile = OnlineGRFProfile(
        source="physical_heel_toe_row",
        ground=OnlineGRFGround(),
        material=material,
        spheres=tuple(spheres),
        metadata={
            "source": "physical_heel_toe_row",
            "note": "Physical contact row anchored to foot markers; NOT fit to "
            "prescribed GRF. Goal: smooth heel->toe COP rollover + weight bearing.",
            "online_mode_status": "requires_forward_validation",
            "n_long": args.n_long,
            "n_lat": args.n_lat,
            "radius_m": args.radius,
            "rocker_radius_m": args.rocker_radius,
            "apex_fraction": args.apex_fraction,
            "target_penetration_m": args.target_penetration_m,
        },
    )

    # ── Auto-fit ground height along the normal so peak penetration on the fit
    #    side hits the target over the reference trajectory. ──────────────────
    times, samples = _sample_spheres_from_coordinate_states(
        setup, profile, str(resolve_repo_path(args.states_sto)), args.sea_plugin
    )
    normal = np.array(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    fit_names = [s.name for s in spheres if s.side == args.fit_side]
    heights = np.concatenate(
        [samples["centers"][name] @ normal for name in fit_names]
    )
    min_height = float(np.min(heights))
    origin_along_normal = args.target_penetration_m - args.radius + min_height
    origin = (origin_along_normal * normal).tolist()
    profile = OnlineGRFProfile(
        source=profile.source,
        ground=OnlineGRFGround(
            origin=(origin[0], origin[1], origin[2]),
            normal=tuple(profile.ground.normal),
            surface_velocity=tuple(profile.ground.surface_velocity),
        ),
        material=profile.material,
        spheres=profile.spheres,
        metadata={
            **dict(profile.metadata),
            "ground_origin_along_normal_m": origin_along_normal,
            "fit_side": args.fit_side,
            "fit_states": str(resolve_repo_path(args.states_sto)),
            "fit_min_center_height_m": min_height,
        },
    )

    destination = write_online_grf_profile(profile, resolve_repo_path(args.out_profile))
    print(f"Row profile written: {destination}")
    print(
        f"  spheres: {len(spheres)} "
        f"({args.n_long} long x {args.n_lat} lat per foot)"
    )
    print(f"  radius: {args.radius} m, material k={material.stiffness:g}, mu={material.static_friction}")
    print(
        f"  ground origin along normal: {origin_along_normal:.5f} m "
        f"(target peak penetration {args.target_penetration_m*1000:.1f} mm on {args.fit_side})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
