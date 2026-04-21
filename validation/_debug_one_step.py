import os
import sys

import numpy as np
import opensim

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SimulatorConfig
from model_loader import setup_model
from kinematics_interpolator import KinematicsInterpolator
from simulation_runner import SimulationRunner
from inverse_dynamics import compute_udot_bypass


def main():
    cfg = SimulatorConfig()
    cfg.t_start = 4.26
    cfg.t_end = 4.27
    cfg.output_dir = r"results\_manual_one_step_debug"

    ctx = setup_model(cfg)
    kin = KinematicsInterpolator(cfg)
    runner = SimulationRunner(cfg, ctx, kin)

    model = ctx.model
    state = ctx.state
    t = cfg.t_start

    q0, qdot0, _ = kin.get(t)
    runner._set_state(state, q0, qdot0, t)
    runner._q0_for_sea_init = q0
    runner._init_muscle_states(state)
    del runner._q0_for_sea_init

    controls = opensim.Vector(ctx.n_controls, 0.0)

    print("A", flush=True)
    model.realizeVelocity(state)

    print("B", flush=True)
    q_ref, qdot_ref, qddot_ref = kin.get(t)

    print("C", flush=True)
    qddot_des_bio = runner._outer_loop.compute_desired_accelerations(
        state, q_ref, qdot_ref, qddot_ref
    )
    qddot_des_all = dict(qddot_des_bio)
    for coord_name in cfg.pros_coords:
        qddot_des_all[coord_name] = qddot_ref.get(coord_name, 0.0)

    if cfg.use_muscles_in_so:
        runner._so.prepare_muscle_baseline(state)
        model.realizeVelocity(state)

    print("D", flush=True)
    tau_bio, tau_pros_ff = runner._id_computer.compute_tau(
        state, controls, qddot_des_all
    )
    model.realizeVelocity(state)

    print("E", flush=True)
    u_sea = runner._prosthesis_ctrl.compute(
        state, q_ref, qdot_ref, controls
    )
    print(f"u_sea={u_sea}", flush=True)

    print("F", flush=True)
    a, u_res = runner._so.solve(state, tau_bio)

    print("G", flush=True)
    runner._so.apply_to_controls(a, u_res, controls, state)
    model.realizeVelocity(state)
    model.setControls(state, controls)

    sv = model.getStateVariableValues(state)
    for sea_name in [cfg.sea_knee_name, cfg.sea_ankle_name]:
        print(
            sea_name,
            "ctrl",
            controls.get(ctx.sea_ctrl_idx[sea_name]),
            "ma",
            sv.get(ctx.sea_motor_angle_sv_idx[sea_name]),
            "ms",
            sv.get(ctx.sea_motor_speed_sv_idx[sea_name]),
            flush=True,
        )

    print("H realizeDynamics", flush=True)
    model.realizeDynamics(state)
    print("H dynamics ok", flush=True)

    print("H compute_udot_bypass", flush=True)
    e_vec = opensim.Vector(ctx.n_mob, 0.0)
    me_vec = opensim.Vector(ctx.n_mob, 0.0)
    udot = compute_udot_bypass(
        model.getMatterSubsystem(), model, state, ctx.n_mob, e_vec, me_vec
    )
    print("udot finite", np.all(np.isfinite(udot)), udot[:5], flush=True)

    sea_derivatives = runner._sea_state_derivatives_from_outputs(state)
    print("sea derivatives", sea_derivatives, flush=True)


if __name__ == "__main__":
    main()
