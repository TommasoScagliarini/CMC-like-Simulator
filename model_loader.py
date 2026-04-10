"""
model_loader.py
===============
Loads the OpenSim model, the SEA C++ plugin, the ExternalLoads (GRF), and
the reserve actuators.  Returns a :class:`SimulationContext` that caches
every index, name-list, and pre-computed constant that the simulation loop
needs.  Computing these once here avoids repeated string lookups in the
hot path.
"""

from __future__ import annotations

import os
import platform
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

import opensim

from config import SimulatorConfig


# ─────────────────────────────────────────────────────────────────────────────
#  SimulationContext  –  immutable topology snapshot computed at setup time
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SimulationContext:
    """
    Frozen topology info passed to every component of the simulation.

    All lists are ordered consistently with the OpenSim coordinate/actuator
    sets so that integer indices can be used directly inside NumPy arrays.
    """
    model: opensim.Model
    state: opensim.State              # reference state from initSystem()

    # ── Coordinate metadata ──────────────────────────────────────────────────
    # All coordinates in the model, in the order returned by getCoordinateSet()
    coord_names:      List[str]
    # Biological subset (all coords NOT in cfg.pros_coords)
    bio_coord_names:  List[str]
    # Prosthetic subset
    pros_coord_names: List[str]

    # Map: coordinate_name → index in coord_names  (= mobility index for
    # gait2392-style models without quaternions)
    coord_mob_idx:    Dict[str, int]

    # ── Muscle metadata ───────────────────────────────────────────────────────
    muscle_names: List[str]
    f_max:        np.ndarray    # shape (n_muscles,) [N]  — constant per model

    # ── Reserve actuator metadata (bio DOFs only) ─────────────────────────────
    reserve_names:     List[str]
    reserve_f_opt:     np.ndarray   # shape (n_reserves,) [N·m]
    # reserve_bio_row[j] = row index in bio_coord_names for reserve actuator j
    reserve_bio_row:   np.ndarray   # shape (n_reserves,) dtype int

    # ── Control vector indices ────────────────────────────────────────────────
    # Position in the model-level control Vector (size = model.getNumControls())
    sea_ctrl_idx:     Dict[str, int]   # {"SEA_knee": i, "SEA_ankle": i}
    muscle_ctrl_idx:  Dict[str, int]   # {muscle_name: ctrl_idx}
    reserve_ctrl_idx: Dict[str, int]   # {reserve_name: ctrl_idx}

    # ── State variable indices ────────────────────────────────────────────────
    # Indices into the SimTK::Vector returned by model.getStateVariableValues()
    # Used to set q and qdot at every integration step.
    q_sv_idx:    Dict[str, int]    # coord_name → "/value" state var index
    qdot_sv_idx: Dict[str, int]    # coord_name → "/speed" state var index

    # ── Convenience scalars ───────────────────────────────────────────────────
    n_mob:      int   # total mobilities  (= len(coord_names) for gait2392)
    n_bio:      int   # len(bio_coord_names)
    n_muscles:  int   # len(muscle_names)
    n_reserves: int   # len(reserve_names)
    n_controls: int   # model.getNumControls()


# ─────────────────────────────────────────────────────────────────────────────
#  Plugin loader
# ─────────────────────────────────────────────────────────────────────────────
def _load_plugin(plugin_name: str) -> None:
    """
    Load the SEA C++ shared library.  The extension is chosen based on the
    running OS so the same config works on Windows, macOS, and Linux.
    """
    ext_map = {"Windows": ".dll", "Darwin": ".dylib"}
    ext = ext_map.get(platform.system(), ".so")
    lib_path = plugin_name + ext
    print(f"[ModelLoader] Loading plugin  : {lib_path}")
    opensim.LoadOpenSimLibrary(lib_path)


# ─────────────────────────────────────────────────────────────────────────────
#  Main setup entry-point
# ─────────────────────────────────────────────────────────────────────────────
def setup_model(cfg: SimulatorConfig) -> SimulationContext:
    """
    Load the model and all ancillary files.  Build the system.
    Cache all indices needed by the simulation loop.

    Parameters
    ----------
    cfg : SimulatorConfig
        Fully populated configuration object.

    Returns
    -------
    SimulationContext
        Ready-to-use topology snapshot.
    """
    # ── 1. C++ plugin ────────────────────────────────────────────────────────
    _load_plugin(cfg.plugin_name)

    # ── 2. OpenSim Model ─────────────────────────────────────────────────────
    print(f"[ModelLoader] Loading model    : {cfg.model_file}")
    model = opensim.Model(cfg.model_file)
    model.setName("ProstheticGaitSim")
    model.setUseVisualizer(False)

    # ── 3. External Loads (GRF) ───────────────────────────────────────────────
    # ExternalLoads reads a setup XML that contains ExternalForce definitions
    # and a pointer to the GRF .mot file.
    print(f"[ModelLoader] Loading GRF      : {cfg.external_loads_xml}")
    ext_loads = opensim.ExternalLoads(cfg.external_loads_xml, True)
    # Add each ExternalForce to the model's ForceSet
    for i in range(ext_loads.getSize()):
        model.addForce(ext_loads.get(i).clone())

    # ── 4. Reserve Actuators ──────────────────────────────────────────────────
    # The ForceSet XML contains CoordinateActuators for every coordinate.
    # We load them all into the model; at QP-build time we will filter the ones
    # belonging to prosthetic coordinates out of the constraint matrix.
    print(f"[ModelLoader] Loading reserves : {cfg.reserve_actuators_xml}")
    reserve_fs = opensim.ForceSet(cfg.reserve_actuators_xml)
    for i in range(reserve_fs.getSize()):
        model.addForce(reserve_fs.get(i).clone())

    # ── 5. Build system & initialise state ───────────────────────────────────
    print("[ModelLoader] Building system …")
    model.finalizeConnections()
    state = model.initSystem()

    # Equilibrate muscles so activation/fibre-length state is physiologically
    # plausible at t_start (reduces initial transients).
    model.equilibrateMuscles(state)

    # ── 6. Cache topology ─────────────────────────────────────────────────────
    coord_set    = model.getCoordinateSet()
    actuator_set = model.getActuators()
    muscle_set   = model.getMuscles()

    n_mob = state.getNU()   # number of mobilities

    # --- Coordinate names & mobility index map ---
    coord_names = [coord_set.get(i).getName()
                   for i in range(coord_set.getSize())]
    coord_mob_idx = {name: i for i, name in enumerate(coord_names)}

    bio_coord_names  = [n for n in coord_names if n not in cfg.pros_coords]
    pros_coord_names = [n for n in coord_names if n     in cfg.pros_coords]
    bio_coord_row    = {name: i for i, name in enumerate(bio_coord_names)}

    # --- Muscle names & F_max ---
    muscle_names = [muscle_set.get(i).getName()
                    for i in range(muscle_set.getSize())]
    f_max = np.array([
        muscle_set.get(name).getMaxIsometricForce()
        for name in muscle_names
    ])

    # --- Reserve actuator names, F_opt, and which bio row they control ---
    reserve_names:        List[str] = []
    reserve_f_opt_list:   List[float] = []
    reserve_bio_row_list: List[int]  = []

    for i in range(actuator_set.getSize()):
        act = actuator_set.get(i)
        if act.getConcreteClassName() != "CoordinateActuator":
            continue
        ca = opensim.CoordinateActuator.safeDownCast(act)
        if ca is None:
            continue
        coord_name = ca.getCoordinate().getName()
        # Only biological coordinates enter the SO constraint
        if coord_name not in bio_coord_row:
            continue
        reserve_names.append(act.getName())
        reserve_f_opt_list.append(ca.getOptimalForce())
        reserve_bio_row_list.append(bio_coord_row[coord_name])

    reserve_f_opt   = np.array(reserve_f_opt_list,   dtype=float)
    reserve_bio_row = np.array(reserve_bio_row_list, dtype=int)

    # --- Control vector indices ---
    # Actuator.getControlIndex() returns the position of this actuator's first
    # control signal in the model-level control Vector.
    def ctrl_idx(act_name: str) -> int:
        return actuator_set.get(act_name).getControlIndex()

    sea_ctrl_idx = {
        cfg.sea_knee_name:  ctrl_idx(cfg.sea_knee_name),
        cfg.sea_ankle_name: ctrl_idx(cfg.sea_ankle_name),
    }
    muscle_ctrl_idx  = {name: ctrl_idx(name) for name in muscle_names}
    reserve_ctrl_idx = {name: ctrl_idx(name) for name in reserve_names}

    # --- State variable indices (for setting q and qdot) ---
    # model.getStateVariableNames() returns strings like
    # "/bodyset/joint/coord/value"  and  "/bodyset/joint/coord/speed".
    sv_names_os = model.getStateVariableNames()
    sv_name_list = [sv_names_os.get(i)
                    for i in range(sv_names_os.getSize())]

    q_sv_idx: Dict[str, int] = {}
    qdot_sv_idx: Dict[str, int] = {}
    for coord_name in coord_names:
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{coord_name}/value"):
                q_sv_idx[coord_name] = sv_idx
                break
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{coord_name}/speed"):
                qdot_sv_idx[coord_name] = sv_idx
                break

    # Sanity check
    missing_q    = [n for n in coord_names if n not in q_sv_idx]
    missing_qdot = [n for n in coord_names if n not in qdot_sv_idx]
    if missing_q or missing_qdot:
        raise RuntimeError(
            f"[ModelLoader] Could not find state variables for: "
            f"q={missing_q}  qdot={missing_qdot}\n"
            f"  Available: {sv_name_list}"
        )

    n_controls = model.getNumControls()

    print(
        f"[ModelLoader] Done.\n"
        f"  Coordinates : {len(coord_names)} total "
        f"({len(bio_coord_names)} bio, {len(pros_coord_names)} pros)\n"
        f"  Muscles     : {len(muscle_names)}\n"
        f"  Reserves    : {len(reserve_names)} (bio coords only)\n"
        f"  Controls    : {n_controls}"
    )

    return SimulationContext(
        model            = model,
        state            = state,
        coord_names      = coord_names,
        bio_coord_names  = bio_coord_names,
        pros_coord_names = pros_coord_names,
        coord_mob_idx    = coord_mob_idx,
        muscle_names     = muscle_names,
        f_max            = f_max,
        reserve_names    = reserve_names,
        reserve_f_opt    = reserve_f_opt,
        reserve_bio_row  = reserve_bio_row,
        sea_ctrl_idx     = sea_ctrl_idx,
        muscle_ctrl_idx  = muscle_ctrl_idx,
        reserve_ctrl_idx = reserve_ctrl_idx,
        q_sv_idx         = q_sv_idx,
        qdot_sv_idx      = qdot_sv_idx,
        n_mob            = n_mob,
        n_bio            = len(bio_coord_names),
        n_muscles        = len(muscle_names),
        n_reserves       = len(reserve_names),
        n_controls       = n_controls,
    )
