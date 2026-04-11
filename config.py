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
    model_file: str = "models\Adjusted_SEASEA - Copia.osim"

    # C++ plugin basename WITHOUT OS extension.
    # Loader adds .dll / .dylib / .so automatically.
    plugin_name: str = "plugins\SEA_Plugin_BlackBox_mCMC_impedence"

    # IK result: positions only, inDegrees=yes  (e.g. Kinematics_q.sto)
    kinematics_file: str = "data\3DGaitModel2392_Kinematics_q.sto"

    # GRF ExternalLoads setup XML  (points to the .mot data file internally)
    external_loads_xml: str = "data\Externall_Loads.xml"

    # Reserve actuators ForceSet XML  (CMC_Actuators.xml)
    reserve_actuators_xml: str = "data\CMC_Actuators.xml"

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
    #   u = clip( Kp*(q_ref – q) + Kd*(qdot_ref – qdot) , -1, +1 )
    #
    # u is passed to the plugin's inner PD torque loop.
    # Units: [rad^-1] for Kp, [s·rad^-1] for Kd.
    # =========================================================================
    sea_kp: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  5.0,
        "pros_ankle_angle": 5.0,
    })
    sea_kd: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  0.5,
        "pros_ankle_angle": 0.5,
    })

    # =========================================================================
    # OUTER LOOP – biological kinematic tracking  (PD)
    #
    #   q_ddot_des = q_ddot_ref + Kp*(q_ref – q) + Kd*(qdot_ref – qdot)
    #
    # Default gains apply to all biological coordinates unless overridden below.
    # Pelvis translations: zero gain (no net muscle force actuator available).
    # =========================================================================
    default_tracking_kp: float = 100.0   # [rad^-1 · s^-2]  or [m^-1 · s^-2]
    default_tracking_kd: float = 20.0    # [rad^-1 · s^-1]

    # Per-coordinate overrides  (coordinate_name → gain)
    tracking_kp: Dict[str, float] = field(default_factory=lambda: {
        "pelvis_tx": 0.0,
        "pelvis_ty": 0.0,
        "pelvis_tz": 0.0,
    })
    tracking_kd: Dict[str, float] = field(default_factory=lambda: {
        "pelvis_tx": 0.0,
        "pelvis_ty": 0.0,
        "pelvis_tz": 0.0,
    })

    # =========================================================================
    # STATIC OPTIMIZATION (QP)
    #
    #   min   Σ a_i²  +  w_res · Σ u_res_j²
    #   s.t.  R·(a ⊙ F_max)  +  R_res·(u_res ⊙ F_opt_res)  =  τ_des_bio
    #         0 ≤ a_i ≤ 1
    #         –u_res_max ≤ u_res_j ≤ u_res_max
    #
    # Reserve actuators are deliberately penalised (w_res >> 1) so muscles
    # are preferred; reserves only activate when muscles are insufficient.
    # =========================================================================
    reserve_weight:  float = 1000.0   # w_res  (dimensionless)
    reserve_u_max:   float = 10.0     # max normalised reserve control  [–]

    # QP solver backend: "slsqp" (scipy, zero extra deps) | "osqp" (faster,
    # needs `pip install qpsolvers[osqp]`)
    qp_solver: str = "slsqp"

    # Max solver iterations (SLSQP)
    qp_max_iter: int = 1000

    # =========================================================================
    # OUTPUT FLAGS
    # =========================================================================
    save_activations:   bool = True
    save_kinematics:    bool = True
    save_sea_controls:  bool = True
    save_tau_bio:       bool = True   # generalised forces after SO
