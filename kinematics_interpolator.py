"""
kinematics_interpolator.py
==========================
Reads an OpenSim IK result file (positions only, inDegrees=yes) and builds a
set of cubic splines so that q(t), q̇(t), q̈(t) can be evaluated continuously
at any time step.

Cubic splines (scipy.interpolate.CubicSpline) give C² continuity, meaning the
second derivatives are computed analytically — no numerical differencing noise.

Usage
-----
    ki = KinematicsInterpolator(cfg)
    q, qdot, qddot = ki.get(t)   # dicts: {coord_name: value [rad or m]}
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from config import SimulatorConfig


class KinematicsInterpolator:
    """
    Pre-processes the IK .sto file and exposes a single method :meth:`get`
    that returns positions, velocities, and accelerations at an arbitrary time.

    All rotational values are stored internally in **radians**.
    Translation coordinates (pelvis_tx/ty/tz) are kept in **metres**.
    """

    def __init__(self, cfg: SimulatorConfig) -> None:
        self._cfg = cfg
        self._translation_set = set(cfg.translation_coords)

        print(f"[KinInterp] Reading {cfg.kinematics_file} ...")
        time, coord_names, data = _read_sto(cfg.kinematics_file)

        # Check that the file covers the simulation window
        if time[0] > cfg.t_start + 1e-6:
            raise ValueError(
                f"Kinematics file starts at t={time[0]:.4f} s, "
                f"but simulation requests t_start={cfg.t_start:.4f} s."
            )
        if time[-1] < cfg.t_end - 1e-6:
            raise ValueError(
                f"Kinematics file ends at t={time[-1]:.4f} s, "
                f"but simulation requests t_end={cfg.t_end:.4f} s."
            )

        self._coord_names: List[str] = coord_names
        self._splines: Dict[str, CubicSpline] = {}

        deg2rad = np.pi / 180.0

        for idx, name in enumerate(coord_names):
            col = data[:, idx]
            # Convert rotational coordinates from degrees to radians
            if name not in self._translation_set:
                col = col * deg2rad
            # Build cubic spline over the full time vector
            # bc_type='not-a-knot' matches the default behaviour and gives
            # the smoothest interpolant for dense IK data.
            self._splines[name] = CubicSpline(time, col, bc_type="not-a-knot")

        print(
            f"[KinInterp] Ready. {len(coord_names)} coordinates, "
            f"t in [{time[0]:.3f}, {time[-1]:.3f}] s, "
            f"{len(time)} samples."
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def get(
        self,
        t: float,
    ) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
        """
        Evaluate kinematics at time *t*.

        Returns
        -------
        q      : dict coord_name → position  [rad] or [m]
        qdot   : dict coord_name → velocity  [rad/s] or [m/s]
        qddot  : dict coord_name → acceleration [rad/s²] or [m/s²]
        """
        q:     Dict[str, float] = {}
        qdot:  Dict[str, float] = {}
        qddot: Dict[str, float] = {}

        for name, spline in self._splines.items():
            # CubicSpline.derivative(n) computes the n-th analytical derivative
            q[name]     = float(spline(t))
            qdot[name]  = float(spline(t, 1))   # 1st derivative
            qddot[name] = float(spline(t, 2))   # 2nd derivative

        return q, qdot, qddot

    @property
    def coord_names(self) -> List[str]:
        """List of coordinate names as read from the IK file."""
        return self._coord_names


# ─────────────────────────────────────────────────────────────────────────────
#  Private helpers
# ─────────────────────────────────────────────────────────────────────────────
def _read_sto(filepath: str) -> Tuple[np.ndarray, List[str], np.ndarray]:
    """
    Parse an OpenSim .sto / .mot file.

    Returns
    -------
    time        : np.ndarray shape (N,)
    coord_names : list of str  (column headers excluding 'time')
    data        : np.ndarray shape (N, n_coords)  — raw values (possibly degrees)
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Kinematics file not found: {filepath}")

    header_done = False
    col_names: List[str] = []
    rows: List[List[float]] = []

    with path.open("r") as fh:
        for line in fh:
            line = line.strip()

            # Detect end of header
            if line.lower() == "endheader":
                header_done = True
                continue

            if not header_done:
                # Check for column-header line (starts with 'time')
                if line.lower().startswith("time"):
                    col_names = line.split()
                continue

            # Data rows
            if line == "":
                continue
            try:
                values = [float(v) for v in line.split()]
                rows.append(values)
            except ValueError:
                # Might be an extra header line; skip it
                if not col_names and line.lower().startswith("time"):
                    col_names = line.split()

    if not col_names:
        raise ValueError(
            f"Could not parse column headers from {filepath}. "
            "Expected a line starting with 'time' before 'endheader'."
        )

    arr  = np.array(rows, dtype=float)        # shape (N, 1 + n_coords)
    time = arr[:, 0]
    data = arr[:, 1:]                          # shape (N, n_coords)
    coord_names = col_names[1:]               # drop 'time'

    if data.shape[1] != len(coord_names):
        raise ValueError(
            f"Column count mismatch: header has {len(coord_names)} coords "
            f"but data has {data.shape[1]} columns."
        )

    # Verify monotonically increasing time
    _, unique_idx = np.unique(time, return_index=True)
    if len(unique_idx) < len(time):
        print(f"[KinInterp] WARNING: rimossi {len(time) - len(unique_idx)} "
            f"timestamp duplicati dal file IK.")
        time = time[unique_idx]
        data = data[unique_idx]

    if np.any(np.diff(time) <= 0):
        raise ValueError("Time vector in kinematics file is not strictly monotonic.")

    return time, coord_names, data
