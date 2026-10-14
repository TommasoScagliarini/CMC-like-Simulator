"""Analytic clipped-Gaussian command statistics on frozen, real batch states.

No actor, environment or training is run. For a~N(mu,sigma²), report E[clip(a)],
clip(mu), P(-1<a<1), and dE[clip(a)]/dmu=P(-1<a<1). Angular command sensitivity
is available only if recorded applied actions and pre-governor command endpoints
verify one affine map. It describes the command, never the physical joint or the
served governor response. Analytic expectations use real arithmetic, not an
integration over every float32 rounding state of the connector.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.special import ndtr

from exploration_analysis import jsonable


def clipped_gaussian(mean, sigma, low=-1., high=1.):
    mean, sigma = np.broadcast_arrays(np.asarray(mean, float), np.asarray(sigma, float))
    if not (np.isfinite(mean).all() and np.isfinite(sigma).all() and np.all(sigma > 0) and low < high):
        raise ValueError("Finite Gaussian means, positive sigma and increasing bounds required")
    alpha, beta = (low-mean)/sigma, (high-mean)/sigma
    phi_a = np.exp(-.5*alpha**2)/np.sqrt(2*np.pi)
    phi_b = np.exp(-.5*beta**2)/np.sqrt(2*np.pi)
    p_low, p_high = ndtr(alpha), ndtr(-beta)
    p_inside = np.maximum(0., 1-p_low-p_high)
    expected = low*p_low + mean*p_inside + sigma*(phi_a-phi_b) + high*p_high
    return {"expected_clipped_action": expected, "clipped_mean": np.clip(mean, low, high),
        "expected_minus_clipped_mean": expected-np.clip(mean, low, high),
        "probability_inside_bounds": p_inside, "probability_clipped_low": p_low,
        "probability_clipped_high": p_high, "d_expected_clipped_d_mean": p_inside,
        "d_expected_clipped_d_sigma": phi_a-phi_b}


def stats(values):
    x = np.asarray(values, float)
    return {"n": int(x.size), "mean": float(x.mean()), "std": float(x.std()),
            "rms": float(np.sqrt(np.mean(x*x))), "min": float(x.min()), "max": float(x.max())}


def affine_command_mapping(records, dimensions):
    """Identify and verify the logged deterministic action→command conversion.

    This does not fit a policy/trajectory or modify a reference. Nonlinear slew
    or an underdetermined mapping disables the angular interpretation.
    """
    used = [r for r in records if r["info"].get("segment_completed")]
    if not used:
        return {"available": False, "reason": "No completed logged commands"}, None
    actions = np.asarray([r["actions"]["applied_reconstructed"] for r in used], float)
    endpoints = np.asarray([r["info"]["policy_segment_values"][-1] for r in used], float)
    if dimensions != 2 or actions.shape[1] != 2 or endpoints.shape[1] != 2:
        return {"available": False, "reason": "Angular audit requires the two endpoint-action dimensions"}, None
    mapping = []
    for j in range(dimensions):
        if np.std(actions[:, j]) < 1e-8:
            return {"available": False, "reason": "Insufficient action range to verify affine mapping"}, None
        design = np.column_stack((np.ones(len(actions)), actions[:, j]))
        offset, scale = np.linalg.lstsq(design, endpoints[:, j], rcond=None)[0]
        max_error = float(np.max(np.abs(design @ [offset, scale]-endpoints[:, j])))
        mapping.append({"component": j, "offset_rad": float(offset), "scale_rad_per_normalized_action": float(scale),
                        "max_recorded_endpoint_reconstruction_error_rad": max_error})
    if any(m["max_recorded_endpoint_reconstruction_error_rad"] > 1e-6 for m in mapping):
        return {"available": False, "reason": "Recorded mapping is not affine to 1e-6 rad (possible slew)", "fit": mapping}, None
    return {"available": True, "verified_completed_records": len(used), "mapping": mapping,
            "scope": "Command endpoint before governor; no physical or filtered sensitivity inferred"}, mapping


def analyze(predictions, records=()):
    mask = np.asarray(predictions["mask"], bool).reshape(-1)
    if not mask.any():
        raise ValueError("No real batch rows")
    before, final = (np.asarray(predictions[name], float)[mask] for name in ("before__logits", "final__logits"))
    if before.shape != final.shape or before.ndim != 2 or before.shape[1] % 2:
        raise ValueError("Expected aligned diagonal-Gaussian logits on the same observations")
    dimensions = before.shape[1]//2
    map_audit, mapping = affine_command_mapping(records, dimensions)
    result = {"schema": "gaussian_clipping_frozen_states_v1", "n_real_states": int(mask.sum()),
        "n_bootstrap_rows_excluded": int((~mask).sum()), "command_mapping_audit": map_audit,
        "conditioning": "Before and final distributions on identical recorded pre-update batch observations; not final-policy visited states",
        "stages": {}, "changes_on_same_states": [],
        "limitations": ["Analytic continuous Gaussian and hard clipping; connector float32 rounding is not integrated.",
            "Derivative holds sigma fixed. A neural parameter change may change both mean and sigma and alter visited states.",
            "Angular derivative concerns pre-governor command expectation. It is not actual joint motion, filtered exploration bandwidth or causal reward sensitivity."]}
    arrays = {"real_batch_indices": np.flatnonzero(mask)}
    stage_data = {}
    for name, logits in (("before", before), ("final", final)):
        mean, logstd = np.split(logits, 2, axis=1)
        sigma = np.exp(logstd)
        values = {"mean": mean, "sigma": sigma, **clipped_gaussian(mean, sigma)}
        stage_data[name] = values
        result["stages"][name] = []
        for j in range(dimensions):
            component = {"component": j, **{k: stats(v[:, j]) for k, v in values.items()}}
            if mapping is not None:
                scale, offset = mapping[j]["scale_rad_per_normalized_action"], mapping[j]["offset_rad"]
                component["expected_command_rad"] = stats(offset+scale*values["expected_clipped_action"][:, j])
                component["nominal_clipped_mean_command_rad"] = stats(offset+scale*values["clipped_mean"][:, j])
                component["expected_minus_nominal_command_deg"] = stats(np.degrees(scale*values["expected_minus_clipped_mean"][:, j]))
                component["d_expected_command_rad_d_mean"] = stats(scale*values["d_expected_clipped_d_mean"][:, j])
            result["stages"][name].append(component)
        arrays.update({name+"__"+k: v for k, v in values.items()})
    for j in range(dimensions):
        change = {"component": j}
        for key in ("mean", "sigma", "expected_clipped_action", "clipped_mean", "probability_inside_bounds"):
            delta = stage_data["final"][key][:, j]-stage_data["before"][key][:, j]
            change[key+"_final_minus_before"] = stats(delta)
            if mapping is not None and key in ("expected_clipped_action", "clipped_mean"):
                change[key+"_command_change_deg"] = stats(np.degrees(mapping[j]["scale_rad_per_normalized_action"]*delta))
        result["changes_on_same_states"].append(change)
    return jsonable(result), arrays


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("predictions", type=Path)
    parser.add_argument("--records", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    paths = [args.predictions, Path(__file__)]
    records = []
    if args.records:
        paths.append(args.records)
        records = [json.loads(line) for line in args.records.read_text(encoding="utf-8").splitlines() if line.strip()]
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with np.load(args.predictions) as data:
        result, arrays = analyze(data, records)
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
        raise RuntimeError("Input changed during analytic audit")
    result["input_hashes"] = hashes
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "gaussian_clipping.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    np.savez_compressed(args.output_dir / "gaussian_clipping_arrays.npz", **arrays)
    print(json.dumps({"real_states": result["n_real_states"], "angular_mapping_available": result["command_mapping_audit"]["available"]}))


if __name__ == "__main__":
    main()
