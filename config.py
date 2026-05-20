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

    # Model bundle directory. Each bundle keeps its .osim files at the root
    # and the supporting XML/STO files inside a local data/ folder.
    model_bundle_dir: str = "models/SEASEA"

    # Biomechanical model (contains muscles + SEA CoordinateActuators).
    # If this is only a filename, it is resolved inside model_bundle_dir.
    # Absolute or explicit repo-relative paths are also supported.
    model_file: str = "Adjusted_SEASEA - Copia_tuned.osim"

    # C++ plugin basename WITHOUT OS extension.
    # Loader adds .dll / .dylib / .so automatically.
    plugin_name: str = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"

    # IK result: positions only, inDegrees=yes  (e.g. Kinematics_q.sto).
    # Relative paths are resolved inside model_bundle_dir.
    kinematics_file: str = "data/3DGaitModel2392_Kinematics_q.sto"

    # GRF ExternalLoads setup XML  (points to the .mot data file internally).
    # Relative paths are resolved inside model_bundle_dir.
    external_loads_xml: str = "data/Externall_Loads.xml"

    # Reserve actuators ForceSet XML  (CMC_Actuators.xml).
    # Relative paths are resolved inside model_bundle_dir.
    reserve_actuators_xml: str = "data/CMC_Actuators.xml"

    # Output directory (created automatically if missing)
    output_dir: str = "results"
    output_prefix: str = "sim_output"

    # =========================================================================
    # SIMULATION TIME  [seconds]
    # =========================================================================
    t_start: float = 4.2300  # must be >= first time stamp in kinematics file
    t_end:   float = 11.0690 # must be <= last  time stamp in kinematics file
    dt:      float = 0.001    # integration step for validated plugin mode

    # =========================================================================
    # CONTROL WINDOW  (CMC-like two-level loop)
    #
    # When use_control_window=True the simulation uses a two-level loop:
    #   - Outer: compute controls (PD, ID, SO) every T_control seconds
    #   - Inner: integrate state with the selected scheme at integration_dt
    # Controls are held constant during the integration substeps (Zero-Order
    # Hold), mirroring the CMC architecture in OpenSim.
    #
    # When use_control_window=False the loop degenerates to the legacy
    # single-step mode where T_control = dt (backward compatible).
    # =========================================================================
    use_control_window: bool = True
    T_control: float = 0.001          # control window [s] (1 ms, CMC-like)
    integration_dt: float = 0.001     # integration substep [s]
    integration_scheme: str = "rk4_bypass"  # "semi_implicit_euler" | "rk4_bypass"

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
    # KINEMATICS PRE-PROCESSING  (IK reference smoothing)
    #
    # The IK .sto provides positions only. Before building the spline used to
    # derive q / qdot / qddot, the signal can be:
    #   1. cleaned of duplicate timestamps,
    #   2. resampled onto a uniform grid,
    #   3. low-pass filtered with a zero-phase Butterworth filter.
    #
    # This reduces spurious high-frequency content that would otherwise be
    # amplified by the spline derivatives, especially in qddot_ref.
    # =========================================================================
    enable_kinematics_lowpass_filter: bool = True
    kinematics_lowpass_cutoff_hz: float = 6.0
    kinematics_lowpass_order: int = 4
    kinematics_resample_dt: float = 0.001

    # =========================================================================
    # SEA HIGH-LEVEL CONTROLLER  (outer PD/PID/CASCADE, prosthetic side)
    #
    #   PD:  τ_cmd = Kp*(q_ref – q) + Kd*(qdot_ref – qdot)
    #   PID: τ_cmd = PD + Ki*∫(q_ref – q)dt
    #   CASCADE:
    #        qdot_cas = qdot_ref + Kp_outer*(q_ref - q)
    #        τ_cmd    = Kp_inner*(qdot_cas - qdot)
    #                 + Ki_inner*∫(qdot_cas - qdot)dt
    #   u     = clip(τ_cmd/F_opt, -1, +1)
    #
    # tau_ff from inverse dynamics may still be saved as an oracle diagnostic,
    # but it is not part of the prosthetic outer-loop command.
    #
    # u is passed to the plugin's inner PD torque loop.
    # Units:
    #   PD/PID: [N·m/rad] for Kp, [N·m·s/rad] for Kd,
    #           [N·m/(rad·s)] for Ki.
    #   CASCADE: [1/s] for Kp_outer, [N·m·s/rad] for Kp_inner,
    #            [N·m/rad] for Ki_inner.
    # =========================================================================
    sea_outer_controller_mode: str = "cascade"  # "pd" | "pid" | "cascade"
    sea_kp: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  160,
        "pros_ankle_angle": 420,
    })
    sea_kd: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  12,
        "pros_ankle_angle": 1,
    })
    sea_ki: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  20.0,
        "pros_ankle_angle": 60.0,
    })
    sea_integral_limit: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  0.25,
        "pros_ankle_angle": 0.25,
    })
    sea_integral_leak_s_inv: float = 0.1
    sea_cascade_kp_outer: Dict[str, float] = field(default_factory=lambda: {
        # Morning best 2026-05-18:
        # full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200.
        "pros_knee_angle":  18.85,
        "pros_ankle_angle": 47.125,
    })
    sea_cascade_kp_inner: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  29.2,
        "pros_ankle_angle": 2.8275,
    })
    sea_cascade_ki_inner: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  1377.0,
        "pros_ankle_angle": 213.0,
    })
    sea_cascade_inner_i_torque_limit: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  50.0,
        "pros_ankle_angle": 200.0,
    })

    # First-order LPF on the SEA outer command u, applied per SEA in the Python
    # outer controller before the value is injected into the model controls.
    # A non-positive cutoff disables the filter on that SEA.
    sea_u_lpf_cutoff_hz: Dict[str, float] = field(default_factory=lambda: {
        "pros_knee_angle":  0.0,
        "pros_ankle_angle": 0.0,
    })

    # SEA spring stiffness [N·m/rad] — must match each plugin <stiffness> property.
    # Used to update motor_angle at each step (non-impedance mode).
    # Forward SEA mode:
    #   "plugin"       -> motor_angle/motor_speed evolve from derivatives
    #                     computed and exposed by the C++ SEA plugin.
    #   "ideal_torque" -> legacy diagnostic baseline that algebraically sets
    #                     motor_angle to produce the commanded spring torque.
    # Only "plugin" is valid for SEA controller validation.
    sea_forward_mode: str = "plugin"

    sea_stiffness: Dict[str, float] = field(default_factory=lambda: {
        "SEA_Knee":  250.0,
        "SEA_Ankle": 700.0,
    })
    # Optional CMC-like feasibility guard on the prosthetic high-level PD term.
    # It scales only the PD correction, never tau_ff, and is meant as a last
    # resort when fixed 100/20 gains would ask the SEA inner loop to saturate.
    enable_sea_feasibility_scaling: bool = False
    sea_feasibility_tau_input_limit: float = 450.0
    sea_feasibility_u_limit: float = 0.95
    sea_feasibility_scales: List[float] = field(default_factory=lambda: [
        1.0,
        0.75,
        0.5,
        0.25,
        0.125,
        0.0625,
        0.03125,
        0.0,
    ])
    # Numerical substeps for stiff SEA/plugin forward dynamics. The derivative
    # values still come from the C++ plugin; Python only advances time.
    sea_motor_substeps: int = 5
    sea_motor_max_substeps: int = 80

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

    # Feasibility guard for the CMC-like control window. The runner scales only
    # the PD correction part of qddot_des before ID/SO, preserving the reference
    # feed-forward while avoiding torque requests the SO cannot deliver.
    enable_so_feasibility_backtracking: bool = True
    so_residual_abs_tol: float = 1e-6
    so_residual_rel_tol: float = 1e-3
    so_backtracking_scales: List[float] = field(default_factory=lambda: [
        1.0,
        0.5,
        0.25,
        0.125,
        0.0625,
        0.03125,
        0.015625,
        0.0078125,
        0.00390625,
    ])

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
    save_sea_derivatives:  bool = True # plugin motor_angle_dot + motor_speed_dot
    save_sea_diagnostics:  bool = True # plugin/Python SEA interface checks
    save_so_torque_diagnostics: bool = True # per-coordinate SO tau/residuals
    save_power:            bool = True # SEA joint + motor power
    save_gait_events:      bool = True # gait cycles from GRF threshold crossings

    # GRF event extraction / plotting defaults
    grf_contact_threshold_n: float = 20.0
    grf_min_contact_duration_s: float = 0.05
    grf_min_cycle_duration_s: float = 0.30
    # Optional runtime GRF cleaner. Disabled by default so existing datasets
    # keep their historical ExternalLoads unless a run explicitly opts in.
    enable_grf_contact_filter: bool = False
    plot_gait_side: str = "left"
