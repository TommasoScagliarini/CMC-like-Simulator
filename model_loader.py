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
import glob
import ctypes
from dataclasses import dataclass, field
from typing import Dict, List
from xml.etree import ElementTree as ET

import numpy as np

import opensim

from config import SimulatorConfig


_DLL_DIR_HANDLES = []
_PRELOADED_DLL_HANDLES = []


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

    # Map: coordinate_name → index in the Simbody mobility vector (U/UDot).
    # This is not necessarily the CoordinateSet order for custom models.
    coord_mob_idx:    Dict[str, int]

    # ── Muscle metadata ───────────────────────────────────────────────────────
    muscle_names: List[str]
    f_max:        np.ndarray    # shape (n_muscles,) [N]  — constant per model
    muscle_activation_sv_idx:    Dict[str, int]
    muscle_fiber_length_sv_idx:  Dict[str, int]

    # ── Reserve actuator metadata (bio DOFs only) ─────────────────────────────
    reserve_names:     List[str]
    reserve_f_opt:     np.ndarray   # shape (n_reserves,) [N·m]
    # reserve_bio_row[j] = row index in bio_coord_names for reserve actuator j
    reserve_bio_row:   np.ndarray   # shape (n_reserves,) dtype int
    # All reserve CoordinateActuators, including prosthetic coordinates, for
    # diagnostics only. The SO still uses reserve_names/reserve_f_opt above.
    reserve_all_names:       List[str]
    reserve_all_coord_names: List[str]
    reserve_all_f_opt:       np.ndarray

    # ── Control vector indices ────────────────────────────────────────────────
    # Position in the model-level control Vector (size = model.getNumControls())
    sea_ctrl_idx:     Dict[str, int]   # {"SEA_knee": i, "SEA_ankle": i}
    muscle_ctrl_idx:  Dict[str, int]   # {muscle_name: ctrl_idx}
    reserve_ctrl_idx: Dict[str, int]   # {reserve_name: ctrl_idx}
    reserve_all_ctrl_idx: Dict[str, int] # {reserve_name: ctrl_idx}

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

    # ── SEA actuator metadata ────────────────────────────────────────────────
    sea_f_opt:               Dict[str, float] = field(default_factory=dict)
    sea_props:               Dict[str, dict]  = field(default_factory=dict)
    sea_motor_angle_sv_idx:  Dict[str, int]   = field(default_factory=dict)
    sea_motor_speed_sv_idx:  Dict[str, int]   = field(default_factory=dict)
    pros_mob_indices:        np.ndarray = field(default_factory=lambda: np.array([], dtype=int))

    # ── GRF data (MUST stay alive for the entire simulation) ─────────────────
    # ExternalForce(Storage, ...) stores a raw C++ pointer to the Storage.
    # If the Python wrapper is garbage-collected, the C++ object is destroyed
    # and the ExternalForce pointer dangles → native crash on realizeAcceleration.
    # Storing the reference here keeps it alive as long as SimulationContext exists.
    grf_storage: object = None   # opensim.Storage
    grf_data_file: str = ""
    grf_vertical_force_columns: Dict[str, str] = field(default_factory=dict)


# ─────────────────────────────────────────────────────────────────────────────
#  Plugin loader
# ─────────────────────────────────────────────────────────────────────────────
def _opensim_install_root_from_python_module() -> str | None:
    """Infer the OpenSim installation root used by the imported Python module."""
    module_file = getattr(opensim, "__file__", "")
    if not module_file:
        return None

    path = os.path.abspath(module_file)
    # Typical Windows layout:
    #   <root>/sdk/Python/opensim/__init__.py
    opensim_pkg = os.path.dirname(path)
    python_dir = os.path.dirname(opensim_pkg)
    sdk_dir = os.path.dirname(python_dir)
    root = os.path.dirname(sdk_dir)
    bin_dir = os.path.join(root, "bin")
    return root if os.path.isdir(bin_dir) else None


def _configure_windows_dll_search_path() -> None:
    """
    Keep plugin dependency resolution on the same OpenSim tree as Python.

    Windows can otherwise resolve osim*.dll/SimTK*.dll from another installed
    OpenSim version in PATH. That is a common cause of native loader exceptions
    such as 0xc06d007f when loading a C++ plugin.
    """
    if os.name != "nt":
        return

    root = _opensim_install_root_from_python_module()
    if not root:
        print("[ModelLoader] WARNING: could not infer OpenSim root from Python.")
        return

    bin_dir = os.path.abspath(os.path.join(root, "bin"))
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    norm_bin = os.path.normcase(bin_dir)
    if not path_entries or os.path.normcase(os.path.abspath(path_entries[0])) != norm_bin:
        path_entries = [
            entry for entry in path_entries
            if os.path.normcase(os.path.abspath(entry or ".")) != norm_bin
        ]
        os.environ["PATH"] = os.pathsep.join([bin_dir] + path_entries)

    if hasattr(os, "add_dll_directory"):
        handle = os.add_dll_directory(bin_dir)
        _DLL_DIR_HANDLES.append(handle)

    preload_order = [
        "SimTKcommon.dll",
        "SimTKmath.dll",
        "SimTKsimbody.dll",
        "osimCommon.dll",
        "osimSimulation.dll",
        "osimActuators.dll",
        "osimTools.dll",
        "osimAnalyses.dll",
    ]
    for dll_name in preload_order:
        dll_path = os.path.join(bin_dir, dll_name)
        if os.path.isfile(dll_path):
            _PRELOADED_DLL_HANDLES.append(ctypes.WinDLL(dll_path))

    print(f"[ModelLoader] OpenSim DLL path : {bin_dir}")


def _load_plugin(plugin_name: str) -> None:
    """
    Load the SEA C++ shared library via OpenSim's plugin loader.

    opensim.LoadOpenSimLibrary() already applies the platform-specific
    naming convention internally:
      - macOS  : prepends 'lib' and appends '.dylib'
                 e.g. "plugins/SEA_…" → dlopen("plugins/libSEA_….dylib")
      - Windows: appends '.dll'  (no 'lib' prefix)
      - Linux  : prepends 'lib' and appends '.so'

    Therefore we must pass the bare basename WITHOUT any extension.
    Adding '.dylib' here would produce a double extension (.dylib.dylib).
    """
    _configure_windows_dll_search_path()
    print(f"[ModelLoader] Loading plugin  : {plugin_name}")
    opensim.LoadOpenSimLibrary(plugin_name)


def _configure_geometry_search_paths(model_file: str) -> None:
    """
    Register local and installed OpenSim Geometry folders before model loading.

    OpenSim emits missing-mesh warnings while constructing the Model, even when
    the visualizer is disabled. Registering these paths here keeps the simulator
    and visualizer behavior consistent across Windows and macOS.
    """
    module_dir = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.dirname(os.path.abspath(model_file))

    candidate_dirs = [
        os.path.join(module_dir, "Geometry"),
        os.path.join(module_dir, "geometry"),
        model_dir,
        os.path.join(model_dir, "Geometry"),
        os.path.join(model_dir, "geometry"),
        os.path.join(os.path.dirname(model_dir), "Geometry"),
    ]

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

    seen = set()
    for path in candidate_dirs:
        abs_path = os.path.abspath(path)
        key = os.path.normcase(abs_path)
        if key in seen or not os.path.isdir(abs_path):
            continue
        seen.add(key)
        opensim.ModelVisualizer.addDirToGeometrySearchPaths(abs_path)
        print(f"[ModelLoader] Geometry path : {abs_path}")


def _xml_local_name(tag: str) -> str:
    """Return an XML tag name without a namespace, if present."""
    return tag.rsplit("}", 1)[-1]


def _child_text(element: ET.Element, child_name: str) -> str | None:
    """Find direct child text while tolerating namespaced OpenSim XML."""
    for child in list(element):
        if _xml_local_name(child.tag) == child_name:
            return child.text.strip() if child.text is not None else ""
    return None


def _infer_external_force_side(force_name: str, applied_to_body: str) -> str | None:
    """Infer left/right from ExternalForce names and applied body names."""
    text = f"{force_name} {applied_to_body}".lower()
    tokens = text.replace("-", "_").replace(".", "_").split("_")
    if "l" in tokens or text.endswith("_l") or "foot_l" in text or "calcn_l" in text:
        return "left"
    if "r" in tokens or text.endswith("_r") or "foot_r" in text or "calcn_r" in text:
        return "right"
    return None


def _read_sea_properties_from_osim(
    model_file: str,
    sea_names: List[str],
) -> Dict[str, dict]:
    """
    Read SeriesElasticActuator properties from the .osim XML.

    On some Windows OpenSim/Python builds, custom plugin properties are not
    exposed through Object.getPropertyByName() even though the plugin itself
    loads and uses them correctly. The .osim file is the authoritative source
    for these values, so reading it preserves the plugin semantics without
    relying on fragile Python reflection.
    """
    model_path = os.path.abspath(model_file)
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"[ModelLoader] Model file not found: {model_path}")

    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError as exc:
        raise RuntimeError(
            f"[ModelLoader] Could not parse model XML for SEA properties: "
            f"{model_path}\n  Detail: {exc}"
        ) from exc

    sea_elements: Dict[str, ET.Element] = {}
    for element in root.iter():
        if _xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name")
        if name:
            sea_elements[name] = element

    def required_float(element: ET.Element, sea_name: str, tag: str) -> float:
        text = _child_text(element, tag)
        if text is None or text == "":
            raise RuntimeError(
                f"[ModelLoader] Missing <{tag}> for SeriesElasticActuator "
                f"'{sea_name}' in {model_path}"
            )
        return float(text)

    def required_bool(element: ET.Element, sea_name: str, tag: str) -> bool:
        text = _child_text(element, tag)
        if text is None or text == "":
            raise RuntimeError(
                f"[ModelLoader] Missing <{tag}> for SeriesElasticActuator "
                f"'{sea_name}' in {model_path}"
            )
        return text.strip().lower() in {"true", "1", "yes"}

    sea_props: Dict[str, dict] = {}
    missing = [name for name in sea_names if name not in sea_elements]
    if missing:
        raise RuntimeError(
            "[ModelLoader] Missing SeriesElasticActuator(s) in model XML: "
            f"{missing}\n  Model: {model_path}"
        )

    for sea_name in sea_names:
        element = sea_elements[sea_name]
        sea_props[sea_name] = {
            "K": required_float(element, sea_name, "stiffness"),
            "Kp": required_float(element, sea_name, "Kp"),
            "Kd": required_float(element, sea_name, "Kd"),
            "Bm": required_float(element, sea_name, "motor_damping"),
            "Jm": required_float(element, sea_name, "motor_inertia"),
            "F_opt": required_float(element, sea_name, "optimal_force"),
            "impedance": required_bool(element, sea_name, "Impedence"),
        }

    print(f"[ModelLoader] SEA XML props    : {sea_props}")
    return sea_props


def _infer_coordinate_mobility_indices(
    model: opensim.Model,
    state: opensim.State,
    coord_names: List[str],
    actuator_set: opensim.Set,
    probe_time: float,
) -> Dict[str, int]:
    """
    Infer coordinate_name -> Simbody U/UDot index by applying one reserve
    CoordinateActuator at a time.

    CoordinateSet order is not guaranteed to match the global mobility vector
    order.  The inverse-dynamics residual, mass matrix, and udot vectors are all
    in mobility order, so using CoordinateSet order here can route torques and
    accelerations to the wrong DOF.
    """
    probe_actuator_by_coord: Dict[str, str] = {}

    for i in range(actuator_set.getSize()):
        act = actuator_set.get(i)
        if act.getConcreteClassName() != "CoordinateActuator":
            continue
        ca = opensim.CoordinateActuator.safeDownCast(act)
        if ca is None:
            continue
        coord_name = ca.getCoordinate().getName()
        act_name = act.getName()
        if coord_name not in probe_actuator_by_coord or act_name.startswith("reserve_"):
            probe_actuator_by_coord[coord_name] = act_name

    missing = [name for name in coord_names if name not in probe_actuator_by_coord]
    if missing:
        raise RuntimeError(
            "[ModelLoader] Cannot infer mobility indices; no CoordinateActuator "
            f"probe found for coordinates: {missing}"
        )

    n_mob = state.getNU()
    n_controls = model.getNumControls()
    zero_udot = opensim.Vector(n_mob, 0.0)

    state.setTime(probe_time)
    controls = opensim.Vector(n_controls, 0.0)
    model.realizeVelocity(state)
    model.setControls(state, controls)
    model.realizeDynamics(state)

    id_solver = opensim.InverseDynamicsSolver(model)
    base_residual_os = id_solver.solve(state, zero_udot)
    base_residual = np.array([base_residual_os.get(i) for i in range(n_mob)])

    coord_mob_idx: Dict[str, int] = {}
    used_indices = set()

    for coord_name in coord_names:
        for j in range(n_controls):
            controls.set(j, 0.0)

        act_name = probe_actuator_by_coord[coord_name]
        ctrl_idx = actuator_set.getIndex(act_name)
        controls.set(ctrl_idx, 1.0)

        model.setControls(state, controls)
        model.realizeDynamics(state)
        residual_os = id_solver.solve(state, zero_udot)
        residual = np.array([residual_os.get(i) for i in range(n_mob)])

        # A positive CoordinateActuator control changes exactly one generalized
        # mobility residual.  Use the response location as the global U index.
        applied = base_residual - residual
        mob_idx = int(np.argmax(np.abs(applied)))
        if abs(applied[mob_idx]) < 1e-8:
            raise RuntimeError(
                f"[ModelLoader] Mobility probe for '{coord_name}' via "
                f"'{act_name}' produced no measurable generalized force."
            )

        coord_mob_idx[coord_name] = mob_idx
        used_indices.add(mob_idx)

    for j in range(n_controls):
        controls.set(j, 0.0)
    model.setControls(state, controls)

    if len(used_indices) != len(coord_names):
        raise RuntimeError(
            "[ModelLoader] Coordinate mobility probe produced duplicate indices: "
            f"{coord_mob_idx}"
        )

    print("[ModelLoader] Mobility index map inferred from reserve actuators.")
    return coord_mob_idx


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
    _configure_geometry_search_paths(cfg.model_file)

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
        print("[ModelLoader] WARNING: updForceSet() unavailable - "
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
    grf_vertical_force_columns: Dict[str, str] = {}

    # ── 3d. Create each ExternalForce with the Storage constructor ──────────
    # ExternalForce(Storage, force_id, point_id, torque_id) binds the data
    # directly at construction time.  The remaining properties (applied_to_body,
    # force_expressed_in_body, point_expressed_in_body) must be set manually.
    for i in range(ext_loads.getSize()):
        ef_template = ext_loads.get(i)
        force_id = ef_template.getForceIdentifier()
        side = _infer_external_force_side(
            ef_template.getName(),
            ef_template.get_applied_to_body(),
        )
        if side in {"left", "right"}:
            # Current GRF files use OpenSim's y-up convention and columns like
            # ground_force1_vx/ground_force1_vy/ground_force1_vz.
            grf_vertical_force_columns[side] = f"{force_id}y"

        ef = opensim.ExternalForce(
            grf_storage,
            force_id,
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
        print(f"[ModelLoader]   '{ef.getName()}' -> addForce OK  "
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
    print("[ModelLoader] Building system ...")
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
    coord_mob_idx = _infer_coordinate_mobility_indices(
        model, state, coord_names, actuator_set, cfg.t_start
    )

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
    reserve_all_names:        List[str] = []
    reserve_all_coord_names:  List[str] = []
    reserve_all_f_opt_list:   List[float] = []

    for i in range(actuator_set.getSize()):
        act = actuator_set.get(i)
        if act.getConcreteClassName() != "CoordinateActuator":
            continue
        ca = opensim.CoordinateActuator.safeDownCast(act)
        if ca is None:
            continue
        act_name = act.getName()
        coord_name = ca.getCoordinate().getName()
        if act_name.startswith("reserve_"):
            reserve_all_names.append(act_name)
            reserve_all_coord_names.append(coord_name)
            reserve_all_f_opt_list.append(ca.getOptimalForce())
        # Only biological coordinates enter the SO constraint
        if coord_name not in bio_coord_row:
            continue
        reserve_names.append(act_name)
        reserve_f_opt_list.append(ca.getOptimalForce())
        reserve_bio_row_list.append(bio_coord_row[coord_name])

    reserve_f_opt   = np.array(reserve_f_opt_list,   dtype=float)
    reserve_bio_row = np.array(reserve_bio_row_list, dtype=int)
    reserve_all_f_opt = np.array(reserve_all_f_opt_list, dtype=float)

    # --- Control vector indices ---
    def ctrl_idx(act_name: str) -> int:
        idx = actuator_set.getIndex(act_name)
        if idx < 0:
            raise RuntimeError(
                f"[ModelLoader] Attuatore '{act_name}' non trovato nell'ActuatorSet."
            )
        return idx

    sea_ctrl_idx = {
        cfg.sea_knee_name:  ctrl_idx(cfg.sea_knee_name),
        cfg.sea_ankle_name: ctrl_idx(cfg.sea_ankle_name),
    }
    muscle_ctrl_idx  = {name: actuator_set.getIndex(name) for name in muscle_names}
    reserve_ctrl_idx = {name: actuator_set.getIndex(name) for name in reserve_names}
    reserve_all_ctrl_idx = {
        name: actuator_set.getIndex(name) for name in reserve_all_names
    }

    # --- SEA actuator properties ---
    sea_names = [cfg.sea_knee_name, cfg.sea_ankle_name]
    sea_props = _read_sea_properties_from_osim(cfg.model_file, sea_names)
    sea_f_opt = {name: props["F_opt"] for name, props in sea_props.items()}

    # --- State variable indices (for setting q and qdot) ---
    # model.getStateVariableNames() returns strings like
    # "/bodyset/joint/coord/value"  and  "/bodyset/joint/coord/speed".
    sv_names_os = model.getStateVariableNames()
    sv_name_list = [sv_names_os.get(i)
                    for i in range(sv_names_os.getSize())]

    q_sv_idx: Dict[str, int] = {}
    qdot_sv_idx: Dict[str, int] = {}
    muscle_activation_sv_idx: Dict[str, int] = {}
    muscle_fiber_length_sv_idx: Dict[str, int] = {}
    for coord_name in coord_names:
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{coord_name}/value"):
                q_sv_idx[coord_name] = sv_idx
                break
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{coord_name}/speed"):
                qdot_sv_idx[coord_name] = sv_idx
                break

    for muscle_name in muscle_names:
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{muscle_name}/activation"):
                muscle_activation_sv_idx[muscle_name] = sv_idx
                break
        for sv_idx, sv_name in enumerate(sv_name_list):
            if sv_name.endswith(f"{muscle_name}/fiber_length"):
                muscle_fiber_length_sv_idx[muscle_name] = sv_idx
                break

    # Sanity check
    missing_q    = [n for n in coord_names if n not in q_sv_idx]
    missing_qdot = [n for n in coord_names if n not in qdot_sv_idx]
    missing_activation = [
        n for n in muscle_names if n not in muscle_activation_sv_idx
    ]
    missing_fiber = [
        n for n in muscle_names if n not in muscle_fiber_length_sv_idx
    ]
    if missing_q or missing_qdot:
        raise RuntimeError(
            f"[ModelLoader] Could not find state variables for: "
            f"q={missing_q}  qdot={missing_qdot}\n"
            f"  Available: {sv_name_list}"
        )
    if missing_activation or missing_fiber:
        raise RuntimeError(
            f"[ModelLoader] Could not find muscle state variables for: "
            f"activation={missing_activation}  fiber_length={missing_fiber}\n"
            f"  Available: {sv_name_list}"
        )

    # --- SEA motor state variable indices ---
    sea_motor_angle_sv_idx: Dict[str, int] = {}
    sea_motor_speed_sv_idx: Dict[str, int] = {}
    for sv_idx, sv_name in enumerate(sv_name_list):
        for sea_name in [cfg.sea_knee_name, cfg.sea_ankle_name]:
            if sv_name.endswith(f"{sea_name}/motor_angle"):
                sea_motor_angle_sv_idx[sea_name] = sv_idx
            elif sv_name.endswith(f"{sea_name}/motor_speed"):
                sea_motor_speed_sv_idx[sea_name] = sv_idx

    # --- Prosthetic mobility indices ---
    pros_mob_indices = np.array([coord_mob_idx[n] for n in pros_coord_names], dtype=int)

    n_controls = model.getNumControls()

    print(
        f"[ModelLoader] Done.\n"
        f"  Coordinates : {len(coord_names)} total "
        f"({len(bio_coord_names)} bio, {len(pros_coord_names)} pros)\n"
        f"  Muscles     : {len(muscle_names)}\n"
        f"  Reserves    : {len(reserve_names)} SO, "
        f"{len(reserve_all_names)} diagnostic total\n"
        f"  Controls    : {n_controls}"
    )

    print(
        f"  SEA F_opt   : {sea_f_opt}\n"
        f"  SEA motor SV: angle={sea_motor_angle_sv_idx}, "
        f"speed={sea_motor_speed_sv_idx}"
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
        muscle_activation_sv_idx   = muscle_activation_sv_idx,
        muscle_fiber_length_sv_idx = muscle_fiber_length_sv_idx,
        reserve_names    = reserve_names,
        reserve_f_opt    = reserve_f_opt,
        reserve_bio_row  = reserve_bio_row,
        reserve_all_names       = reserve_all_names,
        reserve_all_coord_names = reserve_all_coord_names,
        reserve_all_f_opt       = reserve_all_f_opt,
        sea_ctrl_idx     = sea_ctrl_idx,
        muscle_ctrl_idx  = muscle_ctrl_idx,
        reserve_ctrl_idx = reserve_ctrl_idx,
        reserve_all_ctrl_idx = reserve_all_ctrl_idx,
        q_sv_idx         = q_sv_idx,
        qdot_sv_idx      = qdot_sv_idx,
        n_mob            = n_mob,
        n_bio            = len(bio_coord_names),
        n_muscles        = len(muscle_names),
        n_reserves       = len(reserve_names),
        n_controls       = n_controls,
        sea_f_opt             = sea_f_opt,
        sea_props             = sea_props,
        sea_motor_angle_sv_idx = sea_motor_angle_sv_idx,
        sea_motor_speed_sv_idx = sea_motor_speed_sv_idx,
        pros_mob_indices      = pros_mob_indices,
        grf_storage      = grf_storage,
        grf_data_file    = mot_file,
        grf_vertical_force_columns = grf_vertical_force_columns,
    )
