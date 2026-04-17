import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from model_loader import setup_model


def main():
    cfg = SimulatorConfig()
    ctx = setup_model(cfg)
    kin = KinematicsInterpolator(cfg)
    state = ctx.state
    q0, _qd0, _ = kin.get(cfg.t_start)

    sv = ctx.model.getStateVariableValues(state)
    for name, val in q0.items():
        idx = ctx.q_sv_idx.get(name)
        if idx is not None:
            sv.set(idx, val)
    for name in ctx.coord_names:
        idx = ctx.qdot_sv_idx.get(name)
        if idx is not None:
            sv.set(idx, 0.0)
    ctx.model.setStateVariableValues(state, sv)
    ctx.model.realizeVelocity(state)

    for name in ctx.coord_names:
        sv = ctx.model.getStateVariableValues(state)
        for other in ctx.coord_names:
            idx = ctx.qdot_sv_idx.get(other)
            if idx is not None:
                sv.set(idx, 0.0)
        sv.set(ctx.qdot_sv_idx[name], 1.0)
        ctx.model.setStateVariableValues(state, sv)
        ctx.model.realizeVelocity(state)
        u = state.getU()
        nonzero = [
            (i, float(u.get(i)))
            for i in range(state.getNU())
            if abs(float(u.get(i))) > 1e-9
        ]
        print(f"{name:24s} inferred={ctx.coord_mob_idx[name]:2d} U={nonzero}")


if __name__ == "__main__":
    main()
