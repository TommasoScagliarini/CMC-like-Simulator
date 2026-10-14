"""Read-only policy sensitivity on recorded AB06 rollouts, without simulation.

Reproduce the inference contract before computing derivatives. Features use a
common within-policy dispersion, not policy-specific normalization. All output
is confined to this diagnostic folder; no checkpoint or production file changes.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[key] = "1"
sys.path.insert(0, str(TG))

import numpy as np
from unified_MLP_39 import rollout_eval as ev

SOURCES = {
    "LEFT_best80": PACKAGE / "runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic",
    "RIGHT_last60": PACKAGE / "runs/rollout/MLP39_right_v26_60iter_2026-09-23_last60",
    "V26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}
HASHES: dict[str, str] = {}
WINDOW = (13.15, 17.95)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(path: Path) -> None:
    HASHES[str(path.relative_to(REPO))] = digest(path)


def read(path: Path):
    record(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def portable_path(value: str) -> Path:
    path = Path(value)
    if path.exists():
        return path
    # Archived absolute paths can originate from another OS/user directory.
    parts = value.replace("\\", "/").split("/")
    if "Trajectory Generator" in parts:
        path = REPO.joinpath(*parts[parts.index("Trajectory Generator"):])
    if not path.exists():
        raise FileNotFoundError(value)
    return path


def canonical(name: str) -> str:
    if "imitation_target" in name or name.startswith("prescribed_"):
        joint = "knee" if "knee" in name else "ankle"
        derivative = "qdot" if "qdot" in name or name.endswith("_vel") else "q"
        return f"target_{joint}_{derivative}"
    return name


def group(name: str) -> str:
    if name.startswith("target_"):
        return "target_qdot" if name.endswith("qdot") else "target_q"
    if name.startswith("gait_phase_"):
        return "prescribed_clock"
    if name.endswith("previous_endpoint"):
        return "previous_endpoint"
    if name.endswith("served_ref_accel"):
        return "served_qddot"
    if name.endswith("served_ref_vel"):
        return "served_qdot"
    if name.endswith("served_ref"):
        return "served_q"
    if name.endswith("_sea_u"):
        return "SEA_command"
    if name.startswith("SEA_"):
        return "SEA_motor_state"
    if name.startswith("pros_"):
        return "joint_state"
    if name == "online_left_normal_grf_bw":
        return "contact_load"
    if name in {"online_left_in_contact", "online_left_heel_strike", "online_left_toe_off"}:
        return "contact_discrete"
    if name.startswith("phase_fsm_") or name.startswith("phase_expected_"):
        return "FSM_discrete"
    return "FSM_continuous"


def rms(x, axis=None):
    return np.sqrt(np.mean(np.asarray(x) ** 2, axis=axis))


def stats(x) -> dict:
    x = np.asarray(x)
    return {"mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.quantile(x, .95)), "max": float(x.max())}


def csv_write(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        out = csv.DictWriter(f, fieldnames=list(rows[0]))
        out.writeheader()
        out.writerows(rows)


def load_sources():
    ev._load_inference_stack()
    ev.torch.set_num_threads(1)
    data = {}
    for label, folder in SOURCES.items():
        summary = read(folder / "rollout_summary.json")
        trace = read(folder / "rollout_policy_trace.json")
        checkpoint = portable_path(summary["checkpoint"])
        for path in checkpoint.rglob("*"):
            if path.is_file():
                record(path)
        module = ev.RLModule.from_checkpoint(str(checkpoint))
        module.eval()
        n_actor = module._n_actor
        assert n_actor == 39, (label, n_actor)
        names = summary["actor_feature_names"][:n_actor]
        obs = np.asarray([r["actor_observation_vector_before"][:n_actor] for r in trace], np.float32)
        assert all(np.array_equal(obs[k], np.asarray([r["actor_observation_before"][name]
                            for name in names], np.float32)) for k, r in enumerate(trace))
        semantic_names = [canonical(n) for n in names]
        for joint in ("knee", "ankle"):
            for derivative, target_field in (("q", "imitation_target_q"), ("qdot", "imitation_target_qdot")):
                col = semantic_names.index(f"target_{joint}_{derivative}")
                expected = np.asarray([r[target_field][f"pros_{joint}_angle"] for r in trace[:-1]], np.float32)
                np.testing.assert_allclose(obs[1:, col], expected, atol=1e-7, rtol=0)
        time = np.asarray([r["policy_segment_times"][0] for r in trace])
        mask = (time >= WINDOW[0]) & (time <= WINDOW[1])
        data[label] = dict(summary=summary, trace=trace, checkpoint=checkpoint, module=module,
                           names=names, canonical=[canonical(n) for n in names], obs=obs,
                           time=time, mask=mask)
    reference = data["LEFT_best80"]["canonical"]
    assert len(set(reference)) == 39
    for item in data.values():
        assert set(item["canonical"]) == set(reference)
        item["perm"] = [item["canonical"].index(n) for n in reference]
    return data, reference


def reproduce(item):
    module, x = item["module"], item["obs"]
    torch = ev.torch
    means, stds, raw_samples, logits_all = [], [], [], []
    # Original rollout uses one 88-wide row per call; preserve GEMV arithmetic.
    with torch.no_grad():
        for row in x:
            full = np.zeros(module._n_full, np.float32)
            full[:39] = row
            output = module.forward_inference({"obs": torch.from_numpy(full[None])})
            logits = output["action_dist_inputs"]
            direct = module.pi(torch.from_numpy(row[None]))
            assert torch.equal(logits, direct), "Direct differentiable pi must equal inference"
            dist = module.get_inference_action_dist_cls().from_logits(logits)
            sample = dist.to_deterministic().sample().numpy()[0]
            means.append(logits.numpy()[0, :2]); stds.append(np.exp(logits.numpy()[0, 2:]))
            raw_samples.append(sample); logits_all.append(logits.numpy()[0])
    mean, std, sample = map(np.asarray, (means, stds, raw_samples))
    recorded = np.asarray([r["raw_policy_action"] for r in item["trace"]])
    applied = np.asarray([r["applied_policy_action"] for r in item["trace"]])
    assert item["summary"]["action_selection"] == "deterministic"
    np.testing.assert_allclose(mean, recorded, atol=2e-6, rtol=0)
    np.testing.assert_array_equal(sample, mean)
    np.testing.assert_allclose(np.clip(recorded, -1, 1), applied, atol=1e-7, rtol=0)
    endpoint = np.asarray([r["policy_segment_values"][-1] for r in item["trace"]])
    midpoint = np.asarray([-.75, 0.]); half = np.asarray([.75, .7])
    np.testing.assert_allclose(midpoint + half * applied, endpoint, atol=2e-7, rtol=0)
    previous = x[1:, [item["canonical"].index("pros_knee_angle_previous_endpoint"),
                     item["canonical"].index("pros_ankle_angle_previous_endpoint")]]
    np.testing.assert_allclose(previous, endpoint[:-1], atol=1e-7, rtol=0)
    item.update(mean=mean, std=std, endpoint=endpoint)
    return {"actor_width_checkpoint": module._n_actor, "actor_width_summary": item["summary"]["n_actor"],
            "full_width_checkpoint": module._n_full, "rows": len(x),
            "max_raw_action_error": float(np.abs(mean-recorded).max()),
            "bit_exact_raw_action_rows": int(np.all(mean == recorded, axis=1).sum()),
            "deterministic_distribution_sample_equals_mean": True,
            "differentiable_pi_equals_inference": True,
            "clipping_and_endpoint_mapping_verified": True,
            "previous_endpoint_matches_last_command": True,
            "target_q_qdot_physical_scale_one_and_previous_endpoint_time_verified": True,
            "raw_abs_max": float(np.abs(recorded).max()),
            "clip_fraction_by_joint": np.mean(np.abs(recorded) > 1, axis=0).tolist(),
            "mean_gaussian_std_by_joint": std.mean(axis=0).tolist(),
            "note": "std is distribution width; it is not sampled in these deterministic rollouts. "
                    "V26 summary width may be stale; checkpoint and verified forward establish consumed inputs."}


def derivatives(item, scale):
    torch, module = ev.torch, item["module"]
    x = torch.tensor(item["obs"], requires_grad=True)
    logits = module.pi(x)
    jacobian = np.stack([torch.autograd.grad(logits[:, j].sum(), x, retain_graph=j == 0)[0]
                          .detach().numpy() for j in range(2)], axis=1)
    perm = item["perm"]
    j = jacobian[:, :, perm].astype(float)
    xs = item["obs"][:, perm]
    names = item["canonical"]
    # Independent coordinate probes are deliberately tiny. Discrete/clock
    # coordinates are excluded: a fractional FSM state or off-circle clock is
    # not a realizable local perturbation. Marginal range checks are not a full
    # support certificate for correlated physical states.
    checks = []
    for fraction in (.001, .01):
        pred, measured, interior_count = [], [], 0
        for k, canon in enumerate([names[i] for i in perm]):
            if group(canon) in {"contact_discrete", "FSM_discrete", "prescribed_clock"}:
                continue
            if "gait_phase" in canon or scale[k] < 1e-9:
                continue
            idx = perm[k]
            h = fraction * scale[k]
            lo, hi = xs[:, k].min(), xs[:, k].max()
            eligible = item["mask"] & (xs[:, k]-h >= lo) & (xs[:, k]+h <= hi)
            selected = np.flatnonzero(eligible)[::5]
            if not len(selected):
                continue
            plus = item["obs"][selected].copy(); minus = plus.copy()
            plus[:, idx] += h; minus[:, idx] -= h
            with torch.no_grad():
                yp = module.pi(torch.from_numpy(plus)).numpy()[:, :2]
                ym = module.pi(torch.from_numpy(minus)).numpy()[:, :2]
            change = (yp-ym)/2
            measured.append(change); pred.append(j[selected, :, k]*h)
            interior_count += len(selected)
        actual, expected = np.concatenate(measured), np.concatenate(pred)
        checks.append({"common_sd_fraction": fraction, "interior_coordinate_state_pairs": interior_count,
                       "response_rms": float(rms(actual)), "linear_prediction_error_rms": float(rms(actual-expected)),
                       "relative_rms_error": float(rms(actual-expected)/max(rms(actual), 1e-12)),
                       "maximum_abs_error": float(np.abs(actual-expected).max())})
    return j, checks


def endpoint_proxy(item, j, common_names):
    indices = [common_names.index(f"pros_{joint}_angle_previous_endpoint") for joint in ("knee", "ankle")]
    # A_t = d mu_(t+1) / d previous_endpoint * d endpoint_t / d raw_action_t.
    # This only closes the immediately accepted endpoint observation path.
    # It holds all other observations fixed and excludes reference/plant paths.
    accepted_derivative = np.asarray([.75, .7])[None, :] * (np.abs(item["mean"][:-1]) < 1)
    matrix = j[1:, :, indices] * accepted_derivative[:, None, :]
    assert matrix.shape == (499, 2, 2)
    singular = np.linalg.svd(matrix, compute_uv=False)[:, 0]
    radius = np.max(np.abs(np.linalg.eigvals(matrix)), axis=1)
    selected = item["mask"][1:] & item["mask"][:-1]
    finite_errors = []
    source_indices = [item["canonical"].index(common_names[i]) for i in indices]
    for t in np.flatnonzero(selected)[::10]:
        for joint in range(2):
            h = .001
            if abs(abs(item["mean"][t, joint])-1) < 2*h:
                continue
            plus = item["obs"][t+1:t+2].copy(); minus = plus.copy()
            center = item["mean"][t, joint]
            plus[0, source_indices[joint]] += (.75, .7)[joint]*(np.clip(center+h,-1,1)-np.clip(center,-1,1))
            minus[0, source_indices[joint]] += (.75, .7)[joint]*(np.clip(center-h,-1,1)-np.clip(center,-1,1))
            with ev.torch.no_grad():
                actual = (item["module"].pi(ev.torch.from_numpy(plus)).numpy()[0,:2]
                         -item["module"].pi(ev.torch.from_numpy(minus)).numpy()[0,:2])/2
            finite_errors.append(actual - matrix[t,:,joint]*h)
    return {"spectral_norm": stats(singular[selected]), "spectral_radius": stats(radius[selected]),
            "fraction_norm_above_one": float(np.mean(singular[selected] > 1)),
            "fraction_radius_above_one": float(np.mean(radius[selected] > 1)),
            "matrix_mean": matrix[selected].mean(axis=0).tolist(),
            "finite_difference_at_action_delta_0p001_max_abs_response_error": float(np.abs(finite_errors).max()),
            "scope": "Only raw_action_t -> accepted endpoint_t -> next previous_endpoint feature -> raw mean_(t+1); "
                     "all other next observations fixed. Not a stability/causality claim or full governor/plant loop."}, matrix


def main():
    data, names = load_sources()
    pooled_cov = np.mean([np.cov(d["obs"][d["mask"]][:, d["perm"]], rowvar=False)
                          for d in data.values()], axis=0)
    scale = np.sqrt(np.maximum(np.diag(pooled_cov), 0))
    groups = {g: [i for i, n in enumerate(names) if group(n) == g] for g in sorted({group(n) for n in names})}
    groups["controller_memory_all"] = [i for i, n in enumerate(names)
                                         if group(n) in {"previous_endpoint", "served_q", "served_qdot", "served_qddot", "SEA_command"}]
    groups["served_reference_all"] = [i for i, n in enumerate(names) if group(n).startswith("served_")]
    groups["target_all"] = [i for i, n in enumerate(names) if group(n).startswith("target_")]
    metrics = {"method": {"scope": "Recorded deterministic AB06 LEFT/RIGHT/V26 policy sensitivity; no training or simulation",
                           "python_version": sys.version, "numpy_version": np.__version__, "torch_version": ev.torch.__version__,
                           "time_basis": "policy_segment_times[0], observation before step", "window_s": list(WINDOW),
                           "scale": "sqrt(mean of each policy's within-window covariance diagonal), identical across policies",
                           "covariance": "mean within-policy covariance; includes within-group correlations, excludes mean shifts",
                           "units": "dimensionless Gaussian action mean; multiply knee by .75 rad, ankle by .7 rad for unclipped endpoints",
                           "causality": "Descriptive local derivatives on each policy's own state distribution, not a controlled policy effect",
                           "discrete": "FSM/contact discrete Jacobians are ambient network sensitivities only, not feasible state transitions",
                           "clock": "Formal coordinate derivatives plus separate unit-circle tangent diagnostic; no off-circle finite probes"},
               "common_features": names, "common_feature_scale": scale.tolist(), "groups": groups, "policies": {}}
    rows, phase_rows, feature_rows, arrays = [], [], [], {}
    for label, item in data.items():
        reproduction = reproduce(item)
        j, checks = derivatives(item, scale)
        mask = item["mask"]
        accepted_j = j * (np.abs(item["mean"]) < 1)[:, :, None] * np.asarray([.75, .7])[None, :, None]
        group_metrics = {}
        xs = item["obs"][:, item["perm"]]
        phi = np.mod(np.arctan2(xs[:, 0], xs[:, 1]), 2*np.pi)/(2*np.pi)
        for g, idx in groups.items():
            local = j[:, :, idx]
            scaled = local * scale[idx][None, None, :]
            independent = np.sqrt(np.sum(scaled**2, axis=2))
            cov = pooled_cov[np.ix_(idx, idx)]
            correlated = np.sqrt(np.maximum(np.einsum("nai,ij,naj->na", local, cov, local), 0))
            applied = accepted_j[:, :, idx] * scale[idx][None, None, :]
            gm = {"raw_mean_independent_scale_rms_by_joint": rms(independent[mask], axis=0).tolist(),
                  "raw_mean_covariance_scale_rms_by_joint": rms(correlated[mask], axis=0).tolist(),
                  "accepted_endpoint_independent_scale_rms_rad_by_joint": np.sqrt(np.mean(np.sum(applied[mask]**2, axis=2), axis=0)).tolist(),
                  "fraction_total_independent_sensitivity_energy_by_joint": (np.sum(scaled[mask]**2, axis=(0, 2)) /
                     np.maximum(np.sum((j[mask]*scale[None,None,:])**2, axis=(0,2)), 1e-16)).tolist()}
            group_metrics[g] = gm
            rows.append({"policy": label, "group": g, "features": len(idx),
                         "raw_knee_common_sd": gm["raw_mean_independent_scale_rms_by_joint"][0],
                         "raw_ankle_common_sd": gm["raw_mean_independent_scale_rms_by_joint"][1],
                         "covariance_knee": gm["raw_mean_covariance_scale_rms_by_joint"][0],
                         "covariance_ankle": gm["raw_mean_covariance_scale_rms_by_joint"][1],
                         "accepted_knee_rad": gm["accepted_endpoint_independent_scale_rms_rad_by_joint"][0],
                         "accepted_ankle_rad": gm["accepted_endpoint_independent_scale_rms_rad_by_joint"][1]})
            for b in range(10):
                sel = mask & (phi >= b/10) & (phi < (b+1)/10)
                if sel.any():
                    v = rms(independent[sel], axis=0)
                    phase_rows.append({"policy": label, "own_clock_phase_start": b/10, "group": g,
                                       "samples": int(sel.sum()), "knee_common_sd": v[0], "ankle_common_sd": v[1]})
        for k, name in enumerate(names):
            v = rms(j[mask, :, k]*scale[k], axis=0)
            feature_rows.append({"policy": label, "feature": name, "group": group(name), "common_scale": scale[k],
                                 "own_std": float(np.std(xs[mask,k], ddof=1)), "knee_scaled_derivative_rms": v[0],
                                 "ankle_scaled_derivative_rms": v[1]})
        # Clock tangent is physically consistent with sin/cos angle changes.
        clock_tangent = j[:, :, 0]*xs[:, 1,None] - j[:, :, 1]*xs[:, 0,None]
        proxy, matrix = endpoint_proxy(item, j, names)
        first = item["module"].pi[0][0]
        with ev.torch.no_grad():
            pre = first(ev.torch.from_numpy(item["obs"])).numpy()
        # No attribution of variance fractions: correlated groups need not add.
        weight = first.weight.detach().numpy()[:, item["perm"]]
        conditioning = {g: float(np.sqrt(np.mean(np.einsum("hi,ij,hj->h", weight[:, idx],
                                        pooled_cov[np.ix_(idx,idx)], weight[:,idx])))) for g,idx in groups.items()}
        metrics["policies"][label] = {"rollout": str(SOURCES[label].relative_to(REPO)),
                    "checkpoint": str(item["checkpoint"].relative_to(REPO)), "reproduction": reproduction,
                    "samples_window": int(mask.sum()), "finite_difference_checks": checks, "group_sensitivity": group_metrics,
                    "clock_tangent_raw_action_rms_per_phase_radian": rms(clock_tangent[mask],axis=0).tolist(),
                    "endpoint_feedback_partial_proxy": proxy,
                    "first_layer_conditioning": {"saturation_fraction_abs_preactivation_gt3": float(np.mean(np.abs(pre[mask])>3)),
                       "group_common_covariance_preactivation_rms": conditioning,
                       "note": "Group amplitudes are not additive variance shares or a normalization prescription."}}
        arrays[label+"_jacobian_raw_physical_units"] = j
        arrays[label+"_observation_common_order"] = xs
        arrays[label+"_mean"] = item["mean"]
        arrays[label+"_time"] = item["time"]
        arrays[label+"_endpoint_partial_loop"] = matrix
        print(label, json.dumps({"reproduction": reproduction, "memory": group_metrics["controller_memory_all"],
                                "endpoint_proxy": proxy}), flush=True)
    record(Path(__file__))
    for source in (PACKAGE/"rollout_eval.py", PACKAGE/"asymmetric_rl_module.py", TG/"baseline_MLP/asymmetric_rl_module.py"):
        if source.exists():
            record(source)
    for path, expected in HASHES.items():
        assert digest(REPO/path) == expected, f"Input modified: {path}"
    write(HERE/"metrics.json", metrics)
    write(HERE/"input_hashes.json", HASHES)
    csv_write(HERE/"group_sensitivity.csv", rows)
    csv_write(HERE/"feature_sensitivity.csv", feature_rows)
    csv_write(HERE/"phase_sensitivity.csv", phase_rows)
    np.savez_compressed(HERE/"jacobians.npz", common_scale=scale, common_covariance=pooled_cov, **arrays)
    write(HERE/"verification.json", {"inputs_checked": len(HASHES), "input_hashes_unchanged": True,
          "policy_count": len(data), "total_recorded_states": sum(len(d["obs"]) for d in data.values()),
          "no_training": True, "no_simulation": True,
          "output_hashes": {p.name:digest(p) for p in HERE.iterdir() if p.is_file() and p.suffix in {".json", ".csv", ".npz", ".py", ".md"} and p.name != "verification.json" and not p.name.startswith("governor_feedback")}})


if __name__ == "__main__":
    main()
