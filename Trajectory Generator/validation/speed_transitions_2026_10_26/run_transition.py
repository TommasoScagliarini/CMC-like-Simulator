"""Small frozen-policy treadmill perturbations; production modules stay unchanged.

Synthetic time scaling is a stress test, not measured stop/start biomechanics.
Run one case per process: --case control|stop_restart|speed_changes.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE = ROOT / "Trajectory Generator/baseline_MLP"
sys.path.insert(0, str(BASELINE))
import _bootstrap
_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: F401
import numpy as np
import opensim

from output import _read_storage_table, write_sto
from kinematics_interpolator import KinematicsInterpolator
from online_GRF.runtime import _set_property
import rollout_eval

SETUP = ROOT / "models/AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
PRESET = BASELINE / "training_exnovo_cfg.yaml"
CHECKPOINT = ROOT / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best"
START = 11.99 + 1.956870983805102


class Clock:
    """C2 rate transitions; integral, rate and rate derivative are analytic."""
    def __init__(self, transitions):
        self.transitions = transitions

    def at(self, time):
        elapsed = np.asarray(time, dtype=float) - START
        source = np.asarray(time, dtype=float).copy()
        rate = np.ones_like(elapsed)
        acceleration = np.zeros_like(elapsed)
        for start, duration, delta in self.transitions:
            x = np.clip((elapsed - start) / duration, 0.0, 1.0)
            smooth = 10*x**3 - 15*x**4 + 6*x**5
            integral = duration * (2.5*x**4 - 3*x**5 + x**6)
            source += delta * (integral + np.maximum(elapsed-start-duration, 0.0))
            rate += delta * smooth
            acceleration += delta * (30*x**2-60*x**3+30*x**4) / duration
        return source, rate, acceleration


def fingerprint(paths):
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def protocol(case, grf_time, columns, grf):
    if case == "stop_restart":
        # Choose before seeing policy results: first sustained source double support
        # after two walking seconds, both recorded vertical forces >=100 N.
        mask = ((grf_time >= START+2.25) & (grf_time <= START+3.25)
                & (grf[:, columns.index("ground_force1_vy")] >= 100)
                & (grf[:, columns.index("ground_force2_vy")] >= 100))
        candidates = np.flatnonzero(mask)
        runs = np.split(candidates, np.flatnonzero(np.diff(candidates) != 1)+1)
        valid = [r for r in runs if len(r) and grf_time[r[-1]]-grf_time[r[0]] >= .03]
        if not valid:
            raise RuntimeError("No source double-support hold in the preregistered window")
        hold_source = float(grf_time[valid[0][len(valid[0])//2]])
        walk = hold_source - START - .25
        # The binary transport requires complete 1 ms samples; keep the final
        # policy segment on the 10 ms grid even though the hold onset is exact.
        duration = float(np.ceil((walk + 6.0) / .01) * .01)
        stages = [("walk", 0., walk, 1.), ("decelerate", walk, walk+.5, None),
                  ("hold", walk+.5, walk+3.5, 0.),
                  ("accelerate", walk+3.5, walk+4., None),
                  ("restart", walk+4., duration, 1.)]
        transitions = [(walk, .5, -1.), (walk+3.5, .5, 1.)]
    elif case == "speed_changes":
        # Three-second plateaus allow a complete slowed HS-to-HS cycle.
        duration = 11.5
        stages = [("nominal", 0., 2., 1.), ("decelerate", 2., 2.5, None),
                  ("slow", 2.5, 5.5, .8), ("accelerate", 5.5, 6., None),
                  ("fast", 6., 9., 1.2), ("return", 9., 9.5, None),
                  ("nominal_return", 9.5, duration, 1.)]
        transitions = [(2., .5, -.2), (5.5, .5, .4), (9., .5, -.2)]
        hold_source = None
    else:
        duration, stages, transitions, hold_source = 9.5, [("nominal", 0., 9.5, 1.)], [], None
    return Clock(transitions), {"case": case, "start_s": START,
        "duration_s": duration, "stages": stages, "transitions": transitions,
        "hold_source_s": hold_source,
        "interpretation": "synthetic time scaling; GRF amplitudes retained; no measured transition data",
        "speed_units": "relative to calibrated baseline surface vector, not validated walking m/s"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=["control", "stop_restart", "speed_changes"], required=True)
    parser.add_argument("--max-steps", type=int, default=2000)
    parser.add_argument("--suffix", default="")
    args = parser.parse_args()
    out = HERE / (args.case + args.suffix)
    out.mkdir(parents=True, exist_ok=True)
    if (out / "rollout_summary.json").exists():
        raise FileExistsError(f"Preserving existing result: {out}")
    original = ET.parse(SETUP)
    setup_node = original.getroot().find("CMC_Simulator_Setup")
    paths = {name: ROOT / setup_node.find(name).text for name in
             ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml")}
    external = ET.parse(paths["external_loads_xml"])
    source_grf = paths["external_loads_xml"].parent / external.find(".//datafile").text
    gt, names, gd = _read_storage_table(str(source_grf))
    clock, spec = protocol(args.case, gt, names, gd)
    stop = START + spec["duration_s"] + .02
    protected = [SETUP, PRESET, *paths.values(), source_grf,
                 *CHECKPOINT.glob("*"),
                 ROOT / "Trajectory Generator/osim_trj_cmc_like.py",
                 ROOT / "Trajectory Generator/phase_tracking/binary_phase.py",
                 ROOT / "Trajectory Generator/phase_tracking/prosthetic_phase_fsm.py",
                 *BASELINE.glob("*.py"),
                 *ROOT.glob("CMC-like Simulator/*.py"),
                 *ROOT.glob("online_GRF/profiles/*.json"),
                 ROOT / "online_GRF/binary_detector/operational/heel_toe_v25.json",
                 *ROOT.glob("plugins/SEA/*.dylib"), *ROOT.glob("plugins/online_grf_contact/*.dylib")]
    before = fingerprint(protected)
    spec["input_sha256"] = before
    spec["harness_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    grid = np.arange(11.99, stop+.001, .001)
    source, rate, rate_dot = clock.at(grid)
    assert np.all(rate >= -1e-12) and np.all(np.diff(source) >= -1e-12)
    assert source.min() >= gt.min() and source.max() <= gt.max()
    if args.case != "control":
        data = np.column_stack([np.interp(source, gt, gd[:, i]) for i in range(len(names))])
        warped_grf = out / "prescribed_grf.mot"
        write_sto(str(warped_grf), "synthetic synchronized GRF", grid, names, data)
        for node in external.findall(".//datafile") + external.findall(".//data_source_name"):
            node.text = str(warped_grf)
        external.write(out / "ExternalForces.xml", encoding="utf-8", xml_declaration=True)
        paths["external_loads_xml"] = out / "ExternalForces.xml"
    for name, path in paths.items():
        setup_node.find(name).text = str(path)
    setup_node.find("t_end").text = str(stop)
    original.write(out / "setup.xml", encoding="utf-8", xml_declaration=True)
    np.savetxt(out / "schedule.csv", np.column_stack([grid, source, rate, rate_dot]),
               delimiter=",", header="simulation_time,source_time,speed_factor,rate_derivative", comments="")

    class WarpedKinematics(KinematicsInterpolator):
        def get(self, t):
            s, r, a = map(float, clock.at(t))
            if not self.time_bounds[0] <= s <= self.time_bounds[1]:
                raise ValueError(f"Source time outside IK domain: {s}")
            q, v, acc = super().get(s)
            return q, {k: v[k]*r for k in q}, {k: acc[k]*r*r+v[k]*a for k in q}

    rollout_eval._load_inference_stack()
    import osim_trj_cmc_like as env_module
    env_module.KinematicsInterpolator = WarpedKinematics
    factory = rollout_eval.env_factory.make_cmc_env
    journal = (out / "journal.jsonl").open("w", encoding="utf-8")
    native_check = {}

    def make_env(config):
        env = factory(config)
        base = env.unwrapped
        force_paths = list(dict.fromkeys(base.ctx.online_grf_force_paths + base.ctx.online_grf_detector_force_paths))
        forces = [base.ctx.model.getComponent(p) for p in force_paths]
        velocities = [[opensim.PropertyHelper.getValueVec3(f.getPropertyByName("surface_velocity"), i)
                       for i in range(3)] for f in forces]
        native_check["force_count"] = len(forces)
        native_check["baseline_surface_vectors"] = velocities
        # Native smoke: write/read a changed property on the initialized model,
        # realize after invalidating state caches, restore before any rollout.
        for factor in (.8, 0., 1.):
            for f, v in zip(forces, velocities):
                _set_property(f, "surface_velocity", np.asarray(v)*factor)
                actual = [opensim.PropertyHelper.getValueVec3(f.getPropertyByName("surface_velocity"), i) for i in range(3)]
                np.testing.assert_allclose(actual, np.asarray(v)*factor, atol=1e-12)
            sv = base.ctx.model.getStateVariableValues(base.ctx.state)
            base.ctx.model.setStateVariableValues(base.ctx.state, sv)
            base.ctx.model.realizeDynamics(base.ctx.state)
        native_check["property_update_readback_and_realize"] = "passed"
        original_set_state = base.runner._set_state
        previous = [1.]

        def set_state(state, q, qdot, t):
            factor = float(clock.at(t)[1])
            if factor != previous[0]:
                for f, v in zip(forces, velocities):
                    _set_property(f, "surface_velocity", np.asarray(v)*factor)
                previous[0] = factor
            original_set_state(state, q, qdot, t)
        base.runner._set_state = set_state
        # Check chain rule with numerical differentiation at a ramp interior.
        probe_t = START + (spec["transitions"][0][0] + .213 if spec["transitions"] else .213)
        h = 1e-5
        qm, _, _ = base.base_kin.get(probe_t-h)
        q, v, a = base.base_kin.get(probe_t)
        qp, _, _ = base.base_kin.get(probe_t+h)
        native_check["velocity_derivative_max_error"] = max(abs((qp[k]-qm[k])/(2*h)-v[k]) for k in q)
        native_check["acceleration_derivative_max_error"] = max(abs((qp[k]-2*q[k]+qm[k])/(h*h)-a[k]) for k in q)
        assert native_check["velocity_derivative_max_error"] < 1e-5
        assert native_check["acceleration_derivative_max_error"] < .002
        if spec["hold_source_s"] is not None:
            ht = START + spec["stages"][2][1] + 1.
            _, hv, ha = base.base_kin.get(ht)
            native_check["hold_reference_max_abs_velocity"] = max(map(abs, hv.values()))
            native_check["hold_reference_max_abs_acceleration"] = max(map(abs, ha.values()))
            assert max(map(abs, hv.values())) < 1e-12
            assert max(map(abs, ha.values())) < 1e-12
        spec["native_checks"] = native_check
        (out / "protocol.json").write_text(json.dumps(spec, indent=2)+"\n", encoding="utf-8")
        original_step = env.step

        def step(action):
            result = original_step(action)
            obs, reward, terminated, truncated, info = result
            journal.write(json.dumps(rollout_eval._jsonable({
                "time": base.t, "physical_time": base.ctx.state.getTime(),
                "speed_factor": float(clock.at(base.t)[1]),
                "terminated": terminated, "truncated": truncated,
                "end_reason": info.get("end_reason"),
                "phase_fsm": info.get("phase_fsm"),
                "binary_phase_fsm": info.get("binary_phase_fsm"),
                "reward_terms": info.get("reward_terms"),
            }))+"\n")
            journal.flush()
            if base._step_index % 100 == 0:
                print(f"{args.case}: {base.t-START:.2f}s, factor={float(clock.at(base.t)[1]):.3f}", flush=True)
            return result
        env.step = step
        return env

    rollout_eval.env_factory.make_cmc_env = make_env
    sys.argv = ["rollout_eval.py", "--checkpoint", str(CHECKPOINT), "--config", str(PRESET),
                "--no-auto-config", "--setup-xml", str(out / "setup.xml"),
                "--output-dir", str(out), "--episode-duration", str(spec["duration_s"]),
                "--max-steps", str(args.max_steps), "--no-progress", "--record-policy-trace"]
    try:
        result = rollout_eval.run(rollout_eval.parse_args())
        print(json.dumps({k: result[k] for k in ("steps", "end_reason", "terminated", "truncated", "phase_valid_cycle_count")}))
    finally:
        journal.close()
        after = fingerprint(protected)
        (out / "integrity.json").write_text(json.dumps({"unchanged": before == after,
            "changed": [p for p in before if before[p] != after[p]]}, indent=2)+"\n", encoding="utf-8")
        if before != after:
            raise RuntimeError("Protected inputs changed")


if __name__ == "__main__":
    main()
