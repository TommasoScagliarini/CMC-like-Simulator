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
    python visualize.py --save-video             # save MP4 to results/video/
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from xml.etree import ElementTree as ET

import numpy as np
import opensim

from config import SimulatorConfig
from output import read_sto


def _library_variant_exists(path_without_ext: str) -> bool:
    """Return True if a platform shared-library variant exists for a basename."""
    directory = os.path.dirname(path_without_ext)
    basename = os.path.basename(path_without_ext)
    candidates = [
        path_without_ext + ".dll",
        path_without_ext + ".dylib",
        path_without_ext + ".so",
        os.path.join(directory, "lib" + basename + ".dylib"),
        os.path.join(directory, "lib" + basename + ".so"),
    ]
    return any(os.path.isfile(path) for path in candidates)


def _resolve_project_path(path: str, library_basename: bool = False) -> str:
    """Resolve repo-relative defaults even if visualize.py is launched elsewhere."""
    if os.path.isabs(path):
        return path

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.abspath(path),
        os.path.join(script_dir, path),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
        if library_basename and _library_variant_exists(candidate):
            return candidate
    return candidates[0]


def _xml_local_name(tag: str) -> str:
    """Return an XML tag name without a namespace, if present."""
    return tag.rsplit("}", 1)[-1]


def _mesh_files_from_model(model_file: str) -> list[str]:
    """Read mesh_file entries from a model XML for diagnostics."""
    try:
        root = ET.parse(model_file).getroot()
    except Exception:
        return []

    mesh_files: list[str] = []
    for element in root.iter():
        if _xml_local_name(element.tag) == "mesh_file" and element.text:
            mesh_files.append(element.text.strip())
    return sorted(set(mesh_files))


def _mesh_exists(mesh_file: str, search_dirs: list[str]) -> bool:
    if os.path.isabs(mesh_file):
        return os.path.isfile(mesh_file)
    return any(os.path.isfile(os.path.join(path, mesh_file)) for path in search_dirs)


def _configure_geometry_search_paths(
    extra_dirs: list[str] | None = None,
    model_file: str | None = None,
) -> list[str]:
    """
    Register geometry directories so OpenSim can resolve mesh filenames used
    by the model.  The workspace Geometry folder is added first, then common
    OpenSim installation paths.
    """
    candidate_dirs: list[str] = []

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate_dirs.extend(
        [
            os.path.join(script_dir, "Geometry"),
            os.path.join(script_dir, "geometry"),
        ]
    )
    if model_file:
        model_dir = os.path.dirname(os.path.abspath(model_file))
        candidate_dirs.extend(
            [
                model_dir,
                os.path.join(model_dir, "geometry"),
                os.path.join(model_dir, "Geometry"),
                os.path.join(os.path.dirname(model_dir), "Geometry"),
            ]
        )
    if extra_dirs:
        candidate_dirs.extend(extra_dirs)

    for env_name in ["OPENSIM_HOME", "OPENSIM_ROOT", "OPENSIM_INSTALL_DIR"]:
        root = os.environ.get(env_name)
        if not root:
            continue
        candidate_dirs.extend(
            [
                os.path.join(root, "Geometry"),
                os.path.join(root, "Resources", "opensim", "Geometry"),
                os.path.join(root, "share", "opensim", "Geometry"),
            ]
        )

    candidate_dirs.extend(
        glob.glob(
            "/Applications/OpenSim*/OpenSim*.app/Contents/Resources/opensim/Geometry"
        )
    )
    candidate_dirs.extend(glob.glob("/Applications/OpenSim*/Geometry"))
    candidate_dirs.extend(glob.glob("/Applications/OpenSim*/Resources/opensim/Geometry"))

    if os.name == "nt":
        candidate_dirs.extend(glob.glob(r"C:\OpenSim*\Geometry"))
        candidate_dirs.extend(glob.glob(r"C:\Program Files\OpenSim*\Geometry"))
        candidate_dirs.extend(glob.glob(r"C:\Program Files (x86)\OpenSim*\Geometry"))
        candidate_dirs.extend(
            glob.glob(
                os.path.join(
                    os.path.expanduser("~"),
                    "Documents",
                    "OpenSim",
                    "*",
                    "Geometry",
                )
            )
        )

    seen = set()
    added_dirs: list[str] = []
    for path in candidate_dirs:
        abs_path = os.path.abspath(path)
        key = os.path.normcase(abs_path)
        if key in seen or not os.path.isdir(abs_path):
            continue
        seen.add(key)
        opensim.ModelVisualizer.addDirToGeometrySearchPaths(abs_path)
        added_dirs.append(abs_path)
        print(f"[Viz] Geometry path added: {abs_path}")

    if model_file:
        missing = [
            mesh_file
            for mesh_file in _mesh_files_from_model(model_file)
            if not _mesh_exists(mesh_file, added_dirs)
        ]
        if missing:
            preview = ", ".join(missing[:12])
            suffix = " ..." if len(missing) > 12 else ""
            print(
                "[Viz] WARNING: some model meshes were not found in the "
                f"registered Geometry paths: {preview}{suffix}\n"
                "      Add the OpenSim Geometry folder with --geometry-dir "
                "if the visualizer still shows missing meshes."
            )

    return added_dirs


# ─────────────────────────────────────────────────────────────────────────────
#  Video capture helpers (macOS)
# ─────────────────────────────────────────────────────────────────────────────
_SWIFT_FIND_WINDOW = """\
import CoreGraphics
let wl = CGWindowListCopyWindowInfo(.optionOnScreenOnly, kCGNullWindowID) \
    as! [[String:Any]]
for w in wl {
    let owner = w["kCGWindowOwnerName"] as? String ?? ""
    if owner.lowercased().contains("simbody") \
       || owner.lowercased().contains("visuali") {
        let wid = w["kCGWindowNumber"] as? Int ?? 0
        if wid > 0 { print(wid); break }
    }
}
"""


def _find_simbody_window_id() -> int | None:
    """Return the CGWindowID of the Simbody visualizer, or None."""
    try:
        r = subprocess.run(
            ["swift", "-e", _SWIFT_FIND_WINDOW],
            capture_output=True, text=True, timeout=5,
        )
        wid = r.stdout.strip()
        return int(wid) if wid else None
    except Exception:
        return None


def _capture_frame(window_id: int | None, dest: str) -> bool:
    """Capture a single frame to a PNG file via macOS screencapture."""
    cmd = ["screencapture", "-x"]
    if window_id is not None:
        cmd += ["-l", str(window_id)]
    cmd.append(dest)
    try:
        subprocess.run(cmd, check=True, timeout=10,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception:
        return False


def _frames_to_mp4(
    frame_dir: str,
    output_path: str,
    fps: float,
) -> bool:
    """Combine numbered PNGs into an MP4 using ffmpeg."""
    cmd = [
        "ffmpeg", "-y",
        "-r", str(fps),
        "-i", os.path.join(frame_dir, "frame_%06d.png"),
        "-vcodec", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        output_path,
    ]
    try:
        subprocess.run(
            cmd, check=True, timeout=300,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        return True
    except FileNotFoundError:
        print("[Viz] ERROR: ffmpeg not found. Install it with: brew install ffmpeg")
        return False
    except subprocess.CalledProcessError as exc:
        print(f"[Viz] ERROR: ffmpeg failed (exit {exc.returncode})")
        return False


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
    save_video: bool = False,
    video_fps: float = 30.0,
) -> None:
    if cfg is None:
        cfg = SimulatorConfig()

    plugin_name = _resolve_project_path(cfg.plugin_name, library_basename=True)
    model_file = _resolve_project_path(cfg.model_file)
    sto_path = _resolve_project_path(sto_path)

    # ── Load plugin ──────────────────────────────────────────────────────────
    print(f"[Viz] Loading plugin: {plugin_name}")
    opensim.LoadOpenSimLibrary(plugin_name)

    # ── Make the workspace Geometry folder visible to the visualizer ───────
    _configure_geometry_search_paths(geometry_dirs, model_file=model_file)

    # ── Load model WITH visualizer ───────────────────────────────────────────
    print(f"[Viz] Loading model : {model_file}")
    model = opensim.Model(model_file)
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

    # ── Decimate frames ─────────────────────────────────────────────────────
    # The .sto data is typically at 1000 Hz which is far too fast for either
    # screen rendering (~60 fps) or screencapture (~10 fps).  Subsample to a
    # feasible display rate so that time.sleep() actually controls the pace
    # and the --speed flag works as expected.
    display_fps = video_fps if save_video else 60.0
    if n_frames > 1:
        data_dt = (times[-1] - times[0]) / (n_frames - 1)
        data_fps = 1.0 / data_dt if data_dt > 0 else display_fps
        if data_fps > display_fps:
            step = max(1, int(round(data_fps / display_fps)))
            indices = list(range(0, n_frames, step))
            times = times[indices]
            data = data[indices]
            n_frames = len(times)
            print(f"[Viz] Decimated to {n_frames} frames "
                  f"(~{display_fps:.0f} fps target, step={step})")

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

    # ── Video recording setup ────────────────────────────────────────────────
    frame_dir: str | None = None
    window_id: int | None = None

    if save_video:
        frame_dir = tempfile.mkdtemp(prefix="opensim_viz_")
        print(f"[Viz] Video mode: frames -> {frame_dir}")

        # Draw the first frame so the window exists, then find its ID
        if col_coord_map:
            for ci, coord in col_coord_map:
                val = data[0, ci]
                if in_degrees and col_names[ci] not in translation_set:
                    val *= np.pi / 180.0
                coord.setValue(state, val, False)
        state.setTime(times[0])
        model.realizePosition(state)
        simbody_viz.drawFrameNow(state)
        time.sleep(0.5)  # let the window appear

        window_id = _find_simbody_window_id()
        if window_id is not None:
            print(f"[Viz] Found Simbody window (id={window_id})")
        else:
            print("[Viz] WARNING: Simbody window not found, "
                  "capturing full screen instead")

    # ── Playback loop ────────────────────────────────────────────────────────
    deg2rad = np.pi / 180.0
    pass_num = 0
    captured_count = 0          # sequential counter for successfully captured frames
    captured_times: list[float] = []  # timestamps of captured frames (for FPS calc)

    while True:
        pass_num += 1
        tag = f" (pass {pass_num})" if loop else ""

        if save_video:
            print(f"[Viz] Recording{tag} ({n_frames} frames) ...")
        else:
            print(f"[Viz] Playing{tag} at {speed:.2f}x ...")

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

            if save_video:
                # Capture frame to PNG (blocking — sets the pace)
                # Use captured_count for naming so ffmpeg always gets a
                # gapless sequence even if some captures fail.
                frame_path = os.path.join(
                    frame_dir, f"frame_{captured_count:06d}.png"
                )
                ok = _capture_frame(window_id, frame_path)
                if ok and os.path.isfile(frame_path):
                    captured_times.append(t_frame)
                    captured_count += 1
                else:
                    print(f"  [Viz] WARNING: capture failed for sim "
                          f"frame {frame_idx} (t={t_frame:.3f}s)")
                if frame_idx % 50 == 0:
                    print(f"  frame {frame_idx}/{n_frames} "
                          f"(captured: {captured_count})")
            else:
                # Normal real-time playback
                if frame_idx < n_frames - 1:
                    dt_frames = times[frame_idx + 1] - t_frame
                    sleep_s = dt_frames / speed
                    if sleep_s > 0:
                        time.sleep(sleep_s)

        # When recording, only do one pass (ignore --loop)
        if save_video:
            break
        if not loop:
            break

    # ── Encode video ─────────────────────────────────────────────────────────
    if save_video and frame_dir is not None:
        # Compute effective FPS from actually captured frames
        if captured_count > 1 and len(captured_times) > 1:
            total_sim_time = captured_times[-1] - captured_times[0]
            fps = (captured_count - 1) / total_sim_time if total_sim_time > 0 else 30.0
        else:
            fps = 30.0
        print(f"[Viz] Captured {captured_count}/{n_frames} frames")

        video_dir = os.path.join(cfg.output_dir, "video")
        os.makedirs(video_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%H%M%S_%d%m%Y")
        video_path = os.path.join(video_dir, f"{timestamp}.mp4")

        print(f"[Viz] Encoding video ({fps:.1f} fps) -> {video_path}")
        ok = _frames_to_mp4(frame_dir, video_path, fps)

        # Clean up temporary frames
        shutil.rmtree(frame_dir, ignore_errors=True)

        if ok:
            print(f"[Viz] Video saved: {video_path}")
        return

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
    parser.add_argument(
        "--save-video",
        action="store_true",
        help="Capture each frame and encode as MP4 in results/video/. "
             "Requires ffmpeg (brew install ffmpeg). Playback will be slower "
             "during capture; the output video plays at the correct FPS.",
    )
    parser.add_argument(
        "--video-fps",
        type=float,
        default=30.0,
        help="Target FPS for video recording (default: 30). The .sto data is "
             "decimated to this rate before capture to avoid overwhelming "
             "screencapture. Ignored when --save-video is not set.",
    )
    args = parser.parse_args()

    cfg = SimulatorConfig()
    if args.model is not None:
        cfg.model_file = args.model

    sto_path = args.sto
    if sto_path is None:
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
            save_video=args.save_video,
            video_fps=args.video_fps,
        )
    except FileNotFoundError as exc:
        print(f"[Viz] ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[Viz] Interrupted.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
