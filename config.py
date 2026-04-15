"""
config.py
=========
Central configuration for the prosthetic gait simulator.

Edit the FILE PATHS and TUNABLE PARAMETERS sections before running.
Every other module imports this; no magic numbers live anywhere else.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class SimulatorConfig:
    # =========================================================================
    # FILE PATHS  ← edit these to match your project layout
    # =========================================================================

    # Biomechanical model (contains muscles + SEA CoordinateActuators)
    model_file: str = "models/Adjusted_SEASEA - Copia.osim"

    # C++ plugin basename WITHOUT OS extension.
    # Loader adds .dll / .dylib / .so automatically.
    plugin_name: str = "plugins/SEA_Plugin_BlackBox_mCMC_impedence"

    # IK result: positions only, inDegrees=yes  (e.g. Kinematics_q.sto)
    kinematics_file: str = "data/3DGaitModel2392_Kinematics_q.sto"

    # GRF ExternalLoads setup XML  (points to the .mot data file internally)
    external_loads_xml: str = "data/Externall_Loads.xml"

    # Reserve actuators ForceSet XML  (CMC_Actuators.xml)
    reserve_actuators_xml: str = "data/CMC_Actuators.xml"

    # Output directory (created automatically if missing)
    output_dir: str = "results"
    output_prefix: str = "sim_output"

    # =========================================================================
    # SIMULATION TIME  [seconds]
    # =========================================================================
    t_start: float = 4.26     # must be >= first time stamp in kinematics file
    t_end:   float = 11.06    # must be <= last  time stamp in kinematics file
    dt:      float = 0.01     # integration step  (semi-implicit Euler, ZOH)

    # =========================================================================
    # MODEL TOPOLOGY  ← edit if your .osim uses different names
    # =========================================================================

    # Names of the SEA plugin actuators as they appear in the .osim
    sea_knee_name:  str = "SEA_Knee"
    sea_ankle_name: str = "SEA_Ankle"

    # Coordinates controlled exclusively by the two SEAs.
    # These are EXCLUDED from the biological outer-loop and from the QP.
    pros_coords: List[str] = field(default_factory=lambda: [
        "pros_knee_angle",
        "pros_ankle_angle",
    ])

    # Translation (non-rotational) coordinates in the IK file.
    # These are NOT converted deg→rad and have special gain treatment.
    translation_coords: List[str] = field(default_factory=lambda: [
        "pelvis_tx",
        "pelvis_ty",
        "pelvis_tz",
    ])

    # =========================================================================
    # SEA HIGH-LEVEL CONTROLLER  (outer PD, prosthetic side)
    #
    #   τ_cmd = τ_ff + Kp*(q_ref – q) + Kd*(qdot_ref – qdot)
    #   u     = clip(τ_cmd/F_opt, -1, +1)
    #
    # u is passed to the plugin's inner PD torque loop.
    # Units: [N·m/rad] for Kp, [N·m·s/rad] for Kd.
    # =========================================================================
    sea_kp: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  5.0,
        "pros_ankle_angle": 5.0,
    })
    sea_kd: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  0.5,
        "pros_ankle_angle": 0.5,
    })

    # SEA spring stiffness [N·m/rad] — must match each plugin <stiffness> property.
    # Used to update motor_angle at each step (non-impedance mode).
    sea_stiffness: Dict[str, float] = field(default_factory=lambda: {
        "SEA_Knee":  250.0,
        "SEA_Ankle": 500.0,
    })

    # =========================================================================
    # OUTER LOOP – biological kinematic tracking  (PD)
    #
    #   q_ddot_des = q_ddot_ref + Kp*(q_ref – q) + Kd*(qdot_ref – qdot)
    #
    # Default gains apply to all biological coordinates unless overridden below.
    # Pelvis translations are root/residual DOFs: they are tracked with moderate
    # residual-actuator feedback to prevent open-loop integration drift, while
    # the muscle-driven joint recruitment remains handled by the SO.
    # =========================================================================
    default_tracking_kp: float = 100.0   # [rad^-1 · s^-2]  or [m^-1 · s^-2]
    default_tracking_kd: float = 20.0    # [rad^-1 · s^-1]

    # Per-coordinate overrides  (coordinate_name → gain)
    tracking_kp: Dict[str, float] = field(default_factory=lambda: {
        "pelvis_tx": 25.0,
        "pelvis_ty": 25.0,
        "pelvis_tz": 25.0,
    })
    tracking_kd: Dict[str, float] = field(default_factory=lambda: {
        "pelvis_tx": 10.0,
        "pelvis_ty": 10.0,
        "pelvis_tz": 10.0,
    })

    # =========================================================================
    # STATIC OPTIMIZATION (QP) -- muscle-first biological recruitment
    #
    #   min   Σ a_i²  +  w_res · Σ u_res_j²
    #   s.t.  A_muscle(q, qdot, fiber_length)·(a - a_min)
    #       + R_res·(u_res ⊙ F_opt_res)  =  τ_des_bio
    #         a_min ≤ a_i ≤ a_max
    #         –u_res_max ≤ u_res_j ≤ u_res_max
    #
    # A_muscle is built frame-by-frame from Thelen equilibrium tendon-force
    # changes between a_min and a_max, not from the old moment_arm * Fmax
    # approximation. Reserves remain in the problem only as residual actuators.
    # The selected force application keeps the optimized muscle contribution on
    # the OpenSim muscle actuators while avoiding per-frame fiber-state resets
    # during forward dynamics.
    # =========================================================================
    use_muscles_in_so: bool = True
    muscle_mapping_strategy: str = "equilibrium_activation"
    muscle_force_application: str = "override_actuation"
    muscle_min_activation: float = 0.01
    muscle_max_activation: float = 1.0
    muscle_activation_weight: float = 1.0
    muscle_active_threshold: float = 0.02
    muscle_row_capacity_threshold: float = 1e-6

    # Large reserve penalty + finite residual bounds: reserves can close
    # infeasible directions, but they should not be the primary tracking path.
    # Pelvis/root DOFs are treated as residual coordinates in this model, so
    # they get a larger cap and are excluded from the muscle-capable diagnostic
    # denominator.
    reserve_weight:  float = 1.0e6    # w_res  (dimensionless)
    reserve_u_max:   float = 50.0     # joint residual cap, normalised [–]
    unactuated_reserve_u_max: float = 1000.0
    unactuated_reserve_coord_prefixes: List[str] = field(
        default_factory=lambda: ["pelvis_"]
    )

    # Recruitment diagnostics are written to results/*_recruitment.sto.
    save_recruitment_diagnostics: bool = True
    recruitment_diagnostics_interval: int = 50

    # QP solver backend: "slsqp" (scipy, zero extra deps) | "osqp" (faster,
    # needs `pip install qpsolvers[osqp]`)
    qp_solver: str = "slsqp"

    # Max solver iterations (SLSQP)
    qp_max_iter: int = 1000

    # =========================================================================
    # OUTPUT FLAGS
    # =========================================================================
    save_activations:    bool = True
    save_kinematics:     bool = True
    save_sea_controls:   bool = True
    save_tau_bio:        bool = True   # generalised forces after SO
    save_muscle_forces:  bool = True   # F_i = a_i * F_max_i  [N]
    save_sea_torques:    bool = True   # spring + motor torque per SEA  [N·m]
    save_states:         bool = True   # q, qdot, qddot for all coordinates
    save_reserve_controls: bool = True # reserve actuator controls
    save_reserve_torques:  bool = True # reserve torque = control * optimal_force
    save_sea_states:       bool = True # SEA motor_angle + motor_speed
    save_power:            bool = True # SEA joint + motor power
    save_gait_events:      bool = True # gait cycles from GRF threshold crossings

    # GRF event extraction / plotting defaults
    grf_contact_threshold_n: float = 20.0
    plot_gait_side: str = "left"
