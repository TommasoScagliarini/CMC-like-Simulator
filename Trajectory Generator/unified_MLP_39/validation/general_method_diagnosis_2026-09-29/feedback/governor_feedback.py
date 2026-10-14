"""One-segment reference-state feedback probe, using exact production governor.

Only raw action -> governor state -> next policy mean is varied. All physical,
contact, motor, clock and FSM observations at the next step are held recorded.
This is a partial-path intervention, not a simulation of the full feedback loop.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import analyze as policy

np = policy.np
HERE = Path(__file__).resolve().parent
HELPER = HERE.parent / "governor/analyze_governor.py"
spec = importlib.util.spec_from_file_location("diagnostic_reference_governor", HELPER)
gov = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = gov
spec.loader.exec_module(gov)


def step(case, index, state, action):
    limits, cfg = case["limits"], case["cfg"]
    assert np.all(np.isinf(case["slew"]))
    kin = gov.Kinematics(gov.Base(), gov.JOINTS,
                         ref_model=cfg["pros_ref_model"], ref_lpf_cutoff_hz=cfg["pros_ref_cutoff_hz"],
                         ref_governor_enable=cfg["pros_ref_governor"], ref_velocity_limits=limits[0],
                         ref_acceleration_limits=limits[1], ref_jerk_limits=limits[2], control_dt=.001)
    kin._filt_q, kin._filt_v, kin._filt_a = [state[k:k+2].copy() for k in (0,2,4)]
    q, v = state[:2], state[2:4]
    command = case["bounds"][:, 0] + .5*(np.clip(action, -1, 1)+1)*np.diff(case["bounds"], axis=1).ravel()
    t0, t1 = case["starts"][index], case["ends"][index]
    kin.set_segment(np.array([t0,t1]), np.array([q,command]), np.array([v,(command-q)/(t1-t0)]))
    after = np.concatenate([np.array([values[j] for j in gov.JOINTS]) for values in kin.get(t1)])
    return np.r_[command, after]


def main():
    data, names = policy.load_sources()
    archive = np.load(HERE/"jacobians.npz")
    policy.record(HERE/"jacobians.npz")
    policy.record(HERE/"analyze.py")
    policy.record(HELPER)
    indices = [names.index(f"pros_{joint}_angle_{suffix}") for suffix in
               ("previous_endpoint", "served_ref", "served_ref_vel", "served_ref_accel") for joint in ("knee", "ankle")]
    results, arrays = {}, {}
    for label, item in data.items():
        case = gov.load_case({"LEFT_best80":"LEFT", "RIGHT_last60":"RIGHT", "V26_best87":"V26"}[label])
        ep, _, _, verification = gov.replay(case, compare=True)
        x = archive[label+"_observation_common_order"]
        np.testing.assert_allclose(x[1:, indices[2:]], ep[:-1], atol=5e-5, rtol=0)
        j = archive[label+"_jacobian_raw_physical_units"]
        raw = np.asarray([r["raw_policy_action"] for r in item["trace"]])
        src_indices = [item["canonical"].index(names[k]) for k in indices]
        matrices, halves, nonlinear_errors, applied_nonlinear_errors, discrepancies = [], [], [], [], []
        contributions = {"endpoint":[], "q":[], "qdot":[], "qddot":[]}
        for i in range(1, len(ep)-1):
            state = ep[i-1]
            base = step(case, i, state, raw[i])
            discrepancies.append(np.max(np.abs(base[2:]-ep[i])))
            bs = []
            for h in (1e-4, 5e-5):
                columns = []
                for action_index in range(2):
                    plus = raw[i].copy(); minus = raw[i].copy()
                    plus[action_index] += h; minus[action_index] -= h
                    yp, ym = step(case, i, state, plus), step(case, i, state, minus)
                    column = (yp-ym)/(2*h)
                    columns.append(column)
                    if h == 1e-4 and i % 10 == 0:
                        # Physically consistent governor increments are inserted
                        # into the next recorded observation; plant stays fixed.
                        xp = item["obs"][i+1:i+2].copy(); xm = xp.copy()
                        xp[0,src_indices] += yp-base
                        xm[0,src_indices] += ym-base
                        with policy.ev.torch.no_grad():
                            mean_plus = item["module"].pi(policy.ev.torch.from_numpy(xp)).numpy()[0,:2]
                            mean_minus = item["module"].pi(policy.ev.torch.from_numpy(xm)).numpy()[0,:2]
                            actual = (mean_plus-mean_minus)/2
                        expected = j[i+1][:,indices] @ column * h
                        nonlinear_errors.append(actual-expected)
                        applied_actual = (np.clip(mean_plus,-1,1)-np.clip(mean_minus,-1,1))/2
                        applied_expected = (np.abs(raw[i+1])<1)*expected
                        applied_nonlinear_errors.append(applied_actual-applied_expected)
                bs.append(np.stack(columns,axis=1))
            derivative = bs[0]
            matrix = j[i+1][:,indices] @ derivative
            matrices.append(matrix)
            halves.append(j[i+1][:,indices] @ bs[1])
            for k,g in enumerate(contributions):
                contributions[g].append(j[i+1][:,indices[2*k:2*k+2]] @ derivative[2*k:2*k+2])
        matrices, halves = np.asarray(matrices), np.asarray(halves)
        next_clip_derivative = (np.abs(raw[2:]) < 1).astype(float)
        applied_matrices = next_clip_derivative[:,:,None] * matrices
        applied_halves = next_clip_derivative[:,:,None] * halves
        selected = item["mask"][1:-1] & item["mask"][2:]
        singular = np.linalg.svd(matrices, compute_uv=False)[:,0]
        radii = np.max(np.abs(np.linalg.eigvals(matrices)),axis=1)
        # Block contributions sum exactly, but their norms do not (cancellation).
        np.testing.assert_allclose(sum(np.asarray(a) for a in contributions.values()),matrices,atol=1e-12,rtol=0)
        assert max(discrepancies) < 1e-7
        result = {"production_replay_verification":verification,
                  "one_step_reset_replay_max_abs_error":float(max(discrepancies)),
                  "samples":int(selected.sum()), "action_central_difference_steps":[1e-4,5e-5],
                  "spectral_norm":policy.stats(singular[selected]),
                  "spectral_radius":policy.stats(radii[selected]),
                  "fraction_norm_above_one":float(np.mean(singular[selected]>1)),
                  "fraction_radius_above_one":float(np.mean(radii[selected]>1)),
                  "matrix_mean":matrices[selected].mean(axis=0).tolist(),
                  "halving_delta_matrix_difference_rms":float(policy.rms((matrices-halves)[selected])),
                  "halving_delta_matrix_difference_max":float(np.max(np.abs((matrices-halves)[selected]))),
                  "nonlinear_composed_response_max_abs_error":float(np.max(np.abs(nonlinear_errors))),
                  "contribution_spectral_norm_mean":{g:float(np.mean(np.linalg.svd(np.asarray(v)[selected],compute_uv=False)[:,0])) for g,v in contributions.items()}}
        applied_norm = np.linalg.svd(applied_matrices,compute_uv=False)[:,0]
        applied_radius = np.max(np.abs(np.linalg.eigvals(applied_matrices)),axis=1)
        result["raw_next_mean_output"] = "Top-level spectral statistics refer to raw next Gaussian mean, before output clipping."
        result["applied_next_action"] = {
            "definition":"Dclip(mean_next) @ raw_next_mean_matrix, normalized command clipped to [-1,1]; "
                         "zero local derivative outside bounds, unit derivative inside. No samples exactly on boundary.",
            "spectral_norm":policy.stats(applied_norm[selected]),
            "spectral_radius":policy.stats(applied_radius[selected]),
            "fraction_norm_above_one":float(np.mean(applied_norm[selected]>1)),
            "fraction_radius_above_one":float(np.mean(applied_radius[selected]>1)),
            "matrix_mean":applied_matrices[selected].mean(axis=0).tolist(),
            "next_command_clip_fraction_by_joint":(1-next_clip_derivative[selected]).mean(axis=0).tolist(),
            "halving_delta_matrix_difference_rms":float(policy.rms((applied_matrices-applied_halves)[selected])),
            "halving_delta_matrix_difference_max":float(np.max(np.abs((applied_matrices-applied_halves)[selected]))),
            "nonlinear_composed_response_max_abs_error":float(np.max(np.abs(applied_nonlinear_errors))),
            "contribution_spectral_norm_mean":{g:float(np.mean(np.linalg.svd(
                (next_clip_derivative[:,:,None]*np.asarray(v))[selected],compute_uv=False)[:,0])) for g,v in contributions.items()}}
        assert not np.any(np.abs(raw[2:]) == 1), "Clip derivative at exact boundary needs one-sided treatment"
        assert np.max(applied_norm-singular) < 1e-12, "Output clipping cannot increase this Euclidean path norm"
        results[label] = result
        arrays[label+"_one_segment_memory_feedback"] = matrices
        arrays[label+"_one_segment_memory_feedback_applied_next"] = applied_matrices
        arrays[label+"_next_action_clip_derivative"] = next_clip_derivative
        arrays[label+"_time"] = item["time"][1:-1]
        for g,v in contributions.items():
            arrays[label+"_contribution_"+g] = np.asarray(v)
        print(label,json.dumps(result),flush=True)
    policy.HASHES.update(gov.HASHES)
    policy.record(Path(__file__))
    for path,expected in policy.HASHES.items():
        assert policy.digest(policy.REPO/path) == expected, f"Input changed during analysis: {path}"
    policy.write(HERE/"governor_feedback_metrics.json",{
        "method":"Raw output: J_mu(next recorded state),memory @ d(next accepted endpoint + served q,qdot,qddot)/d(raw action current). "
                 "Applied output: Dclip(mu_next) @ raw matrix; reported separately in normalized action units.",
        "scope":"One step partial controller-memory feedback, with physical/SEA/contact/FSM/clock observations frozen to recorded next values. "
                "The exact governor, clipping and local state at each segment are included. Not a complete closed-loop Jacobian or stability test.",
        "initialization":"First segment excluded to avoid initial observed qddot vs governor acceleration-zero reset difference.",
        "policies":results})
    np.savez_compressed(HERE/"governor_feedback_matrices.npz",**arrays)
    policy.write(HERE/"governor_feedback_input_hashes.json",policy.HASHES)
    policy.write(HERE/"governor_feedback_verification.json",{"all_input_hashes_unchanged":True,"input_count":len(policy.HASHES),
           "no_physical_rollout":True,"no_training":True,
           "output_hashes":{p.name:policy.digest(p) for p in (HERE/"governor_feedback_metrics.json",HERE/"governor_feedback_matrices.npz")}})


if __name__ == "__main__":
    main()
