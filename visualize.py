"""
visualize.py
============
Post-simulation playback of kinematics using the Simbody visualizer.

Usage
-----
    python visualize.py                          # defaults
    python visualize.py --speed 0.5              # half-speed
    python visualize.py --sto results/other.sto  # custom file
    python visualize.py --loop                   # repeat forever
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import numpy as np
import opensim

from config import SimulatorConfig


# ─────────────────────────────────────────────────────────────────────────────
#  .sto parser
# ─────────────────────────────────────────────────────────────────────────────
def read_sto(path: str):
    """
    Parse an OpenSim .sto file (version=1, tab-separated).

    Returns
    -------
    times       : np.ndarray shape (n_frames,)
    col_names   : list[str]  (excluding 'time')
    data        : np.ndarray shape (n_frames, n_cols)
    in_degrees  : bool
    """
    with open(path) as f:
        in_degrees = False
        for line in f:
            stripped = line.strip()
            if stripped.lower().startswith("indegrees"):
                in_degrees = "yes" in stripped.lower()
            if stripped == "endheader":
                break

        header_line = f.readline().strip()
        col_names = header_line.split("\t")  # first is "time"

        rows = []
        for line in f:
            vals = line.strip().split("\t")
            if len(vals) == len(col_names):
                rows.append([float(v) for v in vals])

    arr = np.array(rows)
    times = arr[:, 0]
    data = arr[:, 1:]
    col_names = col_names[1:]  # drop "time"
    return times, col_names, data, in_degrees


def _configure_geometry_search_paths(extra_dirs: list[str] | None = None) -> None:
    """
    Register geometry directories so OpenSim can resolve mesh filenames used
    by the model.  The workspace Geometry folder is added first, then common
    OpenSim installation paths.
    """
    candidate_dirs: list[str] = []

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate_dirs.extend(
        [
            os.path.join(script_dir, "geometry"),
            os.path.join(script_dir, "Geometry"),
        ]
    )
    if extra_dirs:
        candidate_dirs.extend(extra_dirs)

    candidate_dirs.extend(
        glob.glob(
            "/Applications/OpenSim*/OpenSim*.app/Contents/Resources/opensim/Geometry"
        )
    )

    seen = set()
    for path in candidate_dirs:
        abs_path = os.path.abspath(path)
        if abs_path in seen or not os.path.isdir(abs_path):
            continue
        seen.add(abs_path)
        opensim.ModelVisualizer.addDirToGeometrySearchPaths(abs_path)
        print(f"[Viz] Geometry path added: {abs_path}")


# ─────────────────────────────────────────────────────────────────────────────
#  Visualizer
# ─────────────────────────────────────────────────────────────────────────────
def run_visualizer(
    sto_path: str,
    speed: float = 1.0,
    loop: bool = False,
    t_start: float | None = None,
    t_end: float | None = None,
    cfg: SimulatorConfig | None = None,
    geometry_dirs: list[str] | None = None,
) -> None:
    if cfg is None:
        cfg = SimulatorConfig()

    # ── Load plugin ──────────────────────────────────────────────────────────
    print(f"[Viz] Loading plugin: {cfg.plugin_name}")
    opensim.LoadOpenSimLibrary(cfg.plugin_name)

    # ── Make the workspace Geometry folder visible to the visualizer ───────
    _configure_geometry_search_paths(geometry_dirs)

    # ── Load model WITH visualizer ───────────────────────────────────────────
    print(f"[Viz] Loading model : {cfg.model_file}")
    model = opensim.Model(cfg.model_file)
    model.setUseVisualizer(True)

    # ── Read kinematics ──────────────────────────────────────────────────────
    print(f"[Viz] Reading STO   : {sto_path}")
    times, col_names, data, in_degrees = read_sto(sto_path)
    # ── Crop to time window ────────────────────────────────────────────────
    if t_start is not None or t_end is not None:
        ts = t_start if t_start is not None else times[0]
        te = t_end   if t_end   is not None else times[-1]
        mask = (times >= ts - 1e-9) & (times <= te + 1e-9)
        times = times[mask]
        data = data[mask]

    n_frames = len(times)
    print(f"[Viz] Frames: {n_frames}, t=[{times[0]:.3f} .. {times[-1]:.3f}] s")

    # ── Init system (opens the visualizer window) ────────────────────────────
    state = model.initSystem()

    # Map .sto columns → Coordinate objects
    coord_set = model.getCoordinateSet()
    col_coord_map: list[tuple[int, opensim.Coordinate]] = []
    for ci, name in enumerate(col_names):
        try:
            coord = coord_set.get(name)
            col_coord_map.append((ci, coord))
        except RuntimeError:
            print(f"[Viz] WARNING: column '{name}' not found in model, skipping")

    # Configure Simbody visualizer
    viz = model.getVisualizer()
    simbody_viz = viz.getSimbodyVisualizer()
    simbody_viz.setShowSimTime(True)
    simbody_viz.setShowFrameRate(True)
    simbody_viz.setDesiredFrameRate(60)

    # Identify translation columns (no deg→rad conversion)
    translation_set = set(cfg.translation_coords)

    # ── Playback loop ────────────────────────────────────────────────────────
    deg2rad = np.pi / 180.0
    pass_num = 0

    while True:
        pass_num += 1
        tag = f" (pass {pass_num})" if loop else ""
        print(f"[Viz] Playing{tag} at {speed:.2f}x …")

        for frame_idx in range(n_frames):
            t_frame = times[frame_idx]

            # Set coordinate values from .sto row
            for ci, coord in col_coord_map:
                val = data[frame_idx, ci]
                if in_degrees and col_names[ci] not in translation_set:
                    val *= deg2rad
                coord.setValue(state, val, False)

            # Update time and realize
            state.setTime(t_frame)
            model.realizePosition(state)
            simbody_viz.drawFrameNow(state)

            # Sleep to achieve real-time * speed factor
            if frame_idx < n_frames - 1:
                dt_frames = times[frame_idx + 1] - t_frame
                sleep_s = dt_frames / speed
                if sleep_s > 0:
                    time.sleep(sleep_s)

        if not loop:
            break

    print("[Viz] Playback complete. Close the visualizer window to exit.")
    # Keep process alive until user closes the window
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Visualize simulation kinematics with the Simbody viewer"
    )
    parser.add_argument(
        "--sto",
        default=None,
        help="Path to kinematics .sto file (default: results/<prefix>_kinematics.sto). "
             "Supports both inDegrees=yes (IK reference) and inDegrees=no (sim output).",
    )
    parser.add_argument(
        "--ik",
        action="store_true",
        help="Use the IK reference file directly (data/kinematics_q.sto) "
             "instead of the simulation output.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0 = real-time)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop playback indefinitely",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Crop playback start time [s] (default: first frame in .sto)",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="Crop playback end time [s] (default: last frame in .sto)",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Override model file path from config.py",
    )
    parser.add_argument(
        "--geometry-dir",
        action="append",
        default=[],
        help="Additional mesh directory to add to OpenSim geometry search paths.",
    )
    args = parser.parse_args()

    cfg = SimulatorConfig()
    if args.model is not None:
        cfg.model_file = args.model

    sto_path = args.sto
    if sto_path is None:
        import os
        if args.ik:
            sto_path = cfg.kinematics_file
        else:
            sto_path = os.path.join(cfg.output_dir, f"{cfg.output_prefix}_kinematics.sto")

    try:
        run_visualizer(
            sto_path,
            speed=args.speed,
            loop=args.loop,
            t_start=args.t_start,
            t_end=args.t_end,
            cfg=cfg,
            geometry_dirs=args.geometry_dir,
        )
    except FileNotFoundError as exc:
        print(f"[Viz] ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[Viz] Interrupted.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
