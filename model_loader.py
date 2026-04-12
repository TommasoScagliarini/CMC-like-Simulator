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

    # ── GRF data (MUST stay alive for the entire simulation) ─────────────────
    # ExternalForce(Storage, ...) stores a raw C++ pointer to the Storage.
    # If the Python wrapper is garbage-collected, the C++ object is destroyed
    # and the ExternalForce pointer dangles → native crash on realizeAcceleration.
    # Storing the reference here keeps it alive as long as SimulationContext exists.
    grf_storage: object = None   # opensim.Storage


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
    # Strategy:
    #   a) Remove any ExternalForce objects already in the .osim ForceSet
    #      (they may carry stale absolute paths from a previous CMC setup).
    #   b) Parse the ExternalLoads XML to read ExternalForce definitions
    #      and the global <datafile> path to the .mot file.
    #   c) Load the GRF .mot file into an opensim.Storage object.
    #   d) Create each ExternalForce using the Storage-based constructor,
    #      which binds the GRF data directly to the Force — no name-based
    #      lookup at runtime.
    #   e) Add each ExternalForce to the model via model.addForce().
    #
    # WHY NOT addModelComponent(ext_loads)?
    #   ExternalLoads is a ModelComponentSet, NOT a Force.  When added as a
    #   generic ModelComponent, the child ExternalForce objects are NOT
    #   registered in the model's ForceSet.  During Stage::Dynamics the model
    #   iterates the ForceSet to call computeForce() — ExternalForces that
    #   are not in the ForceSet produce zero GRF, leading to a native crash
    #   or wildly incorrect dynamics.
    print(f"[ModelLoader] Loading GRF      : {cfg.external_loads_xml}")

    # ── 3a. Remove stale ExternalForce objects baked into the .osim ──────────
    try:
        force_set = model.updForceSet()
        i = 0
        n_removed = 0
        while i < force_set.getSize():
            if force_set.get(i).getConcreteClassName() == "ExternalForce":
                force_set.remove(i)
                n_removed += 1
                # do NOT increment i — indices shift after removal
            else:
                i += 1
        print(f"[ModelLoader] Removed {n_removed} ExternalForce(s) from .osim")
    except AttributeError:
        print("[ModelLoader] WARNING: updForceSet() unavailable — "
              "ExternalForce(s) from .osim may still be present")

    # ── 3b. Parse ExternalLoads XML → read .mot path + force definitions ────
    ext_loads = opensim.ExternalLoads(cfg.external_loads_xml, True)
    mot_file = ext_loads.getDataFileName()

    # Resolve relative .mot path against the XML's directory
    if not os.path.isabs(mot_file):
        xml_dir = os.path.dirname(os.path.abspath(cfg.external_loads_xml))
        mot_file = os.path.join(xml_dir, mot_file)

    if not os.path.isfile(mot_file):
        raise FileNotFoundError(
            f"[ModelLoader] GRF data file not found: {mot_file}\n"
            f"  (resolved from <datafile> in {cfg.external_loads_xml})"
        )
    print(f"[ModelLoader] GRF datafile       : {mot_file}")

    # ── 3c. Load the GRF .mot data into an opensim.Storage ──────────────────
    # The Storage object holds the time-series of force/point/torque columns.
    # By passing it directly to the ExternalForce constructor, the data
    # binding is immediate — no deferred name-based lookup that could fail
    # if the .mot header contains a stale internal path.
    grf_storage = opensim.Storage(mot_file)
    print(f"[ModelLoader] GRF Storage loaded : "
          f"{grf_storage.getSize()} rows, "
          f"{grf_storage.getColumnLabels().getSize() - 1} columns")

    # ── 3d. Create each ExternalForce with the Storage constructor ──────────
    # ExternalForce(Storage, force_id, point_id, torque_id) binds the data
    # directly at construction time.  The remaining properties (applied_to_body,
    # force_expressed_in_body, point_expressed_in_body) must be set manually.
    for i in range(ext_loads.getSize()):
        ef_template = ext_loads.get(i)

        ef = opensim.ExternalForce(
            grf_storage,
            ef_template.getForceIdentifier(),
            ef_template.getPointIdentifier(),
            ef_template.getTorqueIdentifier(),
        )
        ef.setName(ef_template.getName())
        ef.set_applied_to_body(ef_template.get_applied_to_body())
        ef.set_force_expressed_in_body(
            ef_template.get_force_expressed_in_body()
        )
        ef.set_point_expressed_in_body(
            ef_template.get_point_expressed_in_body()
        )

        model.addForce(ef)
        print(f"[ModelLoader]   '{ef.getName()}' → addForce OK  "
              f"(applied_to: {ef.get_applied_to_body()}, "
              f"force_id: {ef.getForceIdentifier()})")
   
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

    # equilibrateMuscles at the default pose reduces initial transients.
    # Some muscles (e.g. flex_dig_r) may fail at boundary configurations —
    # this is acceptable; they will settle during the first integration steps.
    try:
        model.equilibrateMuscles(state)
    except RuntimeError as e:
        print(
            f"[ModelLoader] WARNING: equilibrateMuscles failed at default pose. "
            f"Continuing.\n  Detail: {e}"
        )

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
        idx = actuator_set.getIndex(act_name)
        if idx < 0:
            raise RuntimeError(
                f"[ModelLoader] Attuatore '{act_name}' non trovato nell'ActuatorSet."
            )
        return idx

    sea_ctrl_idx = {
        cfg.sea_knee_name:  actuator_set.getIndex(cfg.sea_knee_name),
        cfg.sea_ankle_name: actuator_set.getIndex(cfg.sea_ankle_name),
    }
    muscle_ctrl_idx  = {name: actuator_set.getIndex(name) for name in muscle_names}
    reserve_ctrl_idx = {name: actuator_set.getIndex(name) for name in reserve_names}

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
        grf_storage      = grf_storage,
    )