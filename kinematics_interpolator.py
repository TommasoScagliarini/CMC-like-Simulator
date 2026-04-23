"""
kinematics_interpolator.py
==========================
Reads an OpenSim IK result file (positions only, inDegrees=yes) and builds a
set of cubic splines so that q(t), qdot(t), qddot(t) can be evaluated
continuously at any time step.

Before the spline is built, the IK reference can optionally be resampled on a
uniform grid and low-pass filtered. This keeps the spline derivatives usable in
the CMC-like pipeline, where qdot_ref and especially qddot_ref are highly
sensitive to high-frequency noise in the raw IK data.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.signal import butter, sosfiltfilt

from config import SimulatorConfig


class KinematicsInterpolator:
    """
    Pre-processes the IK .sto file and exposes a single method :meth:`get`
    that returns positions, velocities, and accelerations at an arbitrary time.

    All rotational values are stored internally in radians.
    Translation coordinates (pelvis_tx/ty/tz) are kept in metres.
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

        data = np.array(data, dtype=float, copy=True)
        deg2rad = np.pi / 180.0

        # Convert rotational coordinates from degrees to radians before any
        # resampling/filtering so the whole preprocessing pipeline stays in
        # coherent physical units.
        for idx, name in enumerate(coord_names):
            if name not in self._translation_set:
                data[:, idx] *= deg2rad

        ready_suffix = ""
        if getattr(cfg, "enable_kinematics_lowpass_filter", False):
            time, data = self._lowpass_and_resample(time, data)
            ready_suffix = (
                f" (low-pass {cfg.kinematics_lowpass_cutoff_hz:g} Hz, "
                f"dt={cfg.kinematics_resample_dt:g} s)"
            )

        for idx, name in enumerate(coord_names):
            col = data[:, idx]
            # bc_type='not-a-knot' matches the default behaviour and gives
            # the smoothest interpolant for dense IK data.
            self._splines[name] = CubicSpline(time, col, bc_type="not-a-knot")

        print(
            f"[KinInterp] Ready. {len(coord_names)} coordinates, "
            f"t in [{time[0]:.3f}, {time[-1]:.3f}] s, "
            f"{len(time)} samples{ready_suffix}."
        )

    def _lowpass_and_resample(
        self,
        time: np.ndarray,
        data: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Resample IK data on a uniform grid and apply a zero-phase LPF."""
        cutoff_hz = float(getattr(self._cfg, "kinematics_lowpass_cutoff_hz", 6.0))
        order = int(getattr(self._cfg, "kinematics_lowpass_order", 4))
        dt = float(getattr(self._cfg, "kinematics_resample_dt", 0.001))

        if order < 1:
            raise ValueError(
                f"kinematics_lowpass_order must be >= 1, got {order}."
            )
        if cutoff_hz <= 0.0:
            raise ValueError(
                "kinematics_lowpass_cutoff_hz must be > 0."
            )
        if dt <= 0.0:
            raise ValueError("kinematics_resample_dt must be > 0.")

        start = float(time[0])
        stop = float(time[-1])
        n_samples = int(np.floor((stop - start) / dt)) + 1
        if n_samples < 2:
            raise ValueError(
                "Not enough IK samples to build a uniform resampling grid."
            )

        uniform_time = start + np.arange(n_samples, dtype=float) * dt
        if uniform_time[-1] < stop - 1e-12:
            uniform_time = np.append(uniform_time, stop)

        uniform_data = np.empty((uniform_time.size, data.shape[1]), dtype=float)
        for col_idx in range(data.shape[1]):
            uniform_data[:, col_idx] = np.interp(
                uniform_time,
                time,
                data[:, col_idx],
            )

        fs = 1.0 / dt
        nyquist = 0.5 * fs
        if cutoff_hz >= nyquist:
            raise ValueError(
                f"kinematics_lowpass_cutoff_hz={cutoff_hz:g} Hz must be below "
                f"Nyquist ({nyquist:g} Hz) for resample_dt={dt:g} s."
            )

        sos = butter(order, cutoff_hz / nyquist, btype="low", output="sos")
        try:
            filtered_data = sosfiltfilt(sos, uniform_data, axis=0)
        except ValueError as exc:
            raise ValueError(
                "Could not apply kinematics low-pass filter; the resampled IK "
                "window is too short for the requested Butterworth settings."
            ) from exc

        print(
            "[KinInterp] Applied IK low-pass: "
            f"cutoff={cutoff_hz:g} Hz, order={order}, dt={dt:g} s, "
            f"samples={len(time)}->{len(uniform_time)}"
        )
        return uniform_time, filtered_data

    # Public API
    def get(
        self,
        t: float,
    ) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
        """
        Evaluate kinematics at time *t*.

        Returns
        -------
        q      : dict coord_name -> position [rad] or [m]
        qdot   : dict coord_name -> velocity [rad/s] or [m/s]
        qddot  : dict coord_name -> acceleration [rad/s^2] or [m/s^2]
        """
        q: Dict[str, float] = {}
        qdot: Dict[str, float] = {}
        qddot: Dict[str, float] = {}

        for name, spline in self._splines.items():
            q[name] = float(spline(t))
            qdot[name] = float(spline(t, 1))
            qddot[name] = float(spline(t, 2))

        return q, qdot, qddot

    @property
    def coord_names(self) -> List[str]:
        """List of coordinate names as read from the IK file."""
        return self._coord_names


def _read_sto(filepath: str) -> Tuple[np.ndarray, List[str], np.ndarray]:
    """
    Parse an OpenSim .sto / .mot file.

    Returns
    -------
    time        : np.ndarray shape (N,)
    coord_names : list of str (column headers excluding 'time')
    data        : np.ndarray shape (N, n_coords) - raw values (possibly degrees)
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Kinematics file not found: {filepath}")

    header_done = False
    col_names: List[str] = []
    rows: List[List[float]] = []

    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()

            if line.lower() == "endheader":
                header_done = True
                continue

            if not header_done:
                if line.lower().startswith("time"):
                    col_names = line.split()
                continue

            if not line:
                continue
            try:
                values = [float(v) for v in line.split()]
                rows.append(values)
            except ValueError:
                if not col_names and line.lower().startswith("time"):
                    col_names = line.split()

    if not col_names:
        raise ValueError(
            f"Could not parse column headers from {filepath}. "
            "Expected a line starting with 'time' before 'endheader'."
        )
    if not rows:
        raise ValueError(f"No numeric data rows found in {filepath}.")

    arr = np.array(rows, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError(f"Malformed IK data matrix in {filepath}.")

    time = arr[:, 0]
    data = arr[:, 1:]
    coord_names = col_names[1:]

    if data.shape[1] != len(coord_names):
        raise ValueError(
            f"Column count mismatch: header has {len(coord_names)} coords "
            f"but data has {data.shape[1]} columns."
        )

    unique_time, unique_idx = np.unique(time, return_index=True)
    if len(unique_idx) < len(time):
        keep_idx = np.sort(unique_idx)
        removed = len(time) - len(keep_idx)
        print(
            f"[KinInterp] WARNING: removed {removed} duplicate "
            f"timestamp(s) from IK file."
        )
        time = time[keep_idx]
        data = data[keep_idx]
    else:
        time = unique_time

    if np.any(np.diff(time) <= 0):
        raise ValueError(
            "Time vector in kinematics file is not strictly monotonic after "
            "duplicate cleanup."
        )

    return time, coord_names, data
