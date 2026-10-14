"""Isolated rollout worker with common Gaussian draws and baseline sigma schedule.

Imports the ordinary rollout entrypoint only when explicitly invoked. The baseline
nominal records sigma at each observation. Other cases use that frozen schedule,
so both noise samples and sigma match by decision index despite state divergence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]


def _write(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(data, indent=2, allow_nan=False) + "\n"
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(text, encoding="utf-8")
    temp.replace(path)


def paired_noise(seed: int, steps: int, action_dim: int):
    """A complete stream, generated before simulation and independent of state."""
    import numpy as np
    return np.random.Generator(np.random.PCG64(seed)).standard_normal((steps, action_dim)).astype(np.float32)


def noise_digest(draws) -> str:
    # Canonical little endian bytes keep receipts portable across OS/CPU.
    return hashlib.sha256(draws.astype("<f4").tobytes(order="C")).hexdigest()


def fixed_sigma_selector(torch, np, sigma_schedule, gaussian, records, *, nominal=False):
    """Return ordinary logits means with explicit frozen perturbation parameters."""
    def select(module, obs, action_shape, **_kwargs):
        index = len(records)
        obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
        with torch.no_grad():
            logits = module.forward_inference({"obs": obs_t})["action_dist_inputs"]
        dim = logits.shape[-1] // 2
        mean = logits[..., :dim].detach().cpu().numpy().reshape(action_shape).astype(np.float32)
        learned_sigma = torch.exp(logits[..., dim:]).detach().cpu().numpy().reshape(action_shape).astype(np.float32)
        if sigma_schedule is None:
            if not nominal:
                raise ValueError("Noisy cases require the frozen best80 sigma schedule")
            sigma = learned_sigma
        else:
            if index >= len(sigma_schedule):
                raise ValueError("Rollout exceeded baseline sigma schedule; extrapolation is forbidden")
            sigma = np.asarray(sigma_schedule[index], dtype=np.float32).reshape(action_shape)
        z = np.zeros(action_shape, dtype=np.float32) if nominal else gaussian[index].reshape(action_shape)
        perturbation = (sigma * z).astype(np.float32)
        records.append({"step": index, "standard_normal": z.reshape(-1).tolist(),
                        "sigma": sigma.reshape(-1).tolist(), "learned_sigma": learned_sigma.reshape(-1).tolist()})
        return mean + perturbation, mean, sigma, perturbation
    return select


def execute_case(case: dict) -> dict:
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = "1"
    sys.path.insert(0, str(PACKAGE.parent))
    from unified_MLP_39 import rollout_eval as ev
    output = Path(case["output_dir"])
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Evaluation output already contains files: {output}")
    argv = [str(PACKAGE / "rollout_eval.py"), "--checkpoint", str(case["checkpoint"]),
            "--output-dir", str(output), "--seed", str(case.get("environment_seed", 123)),
            "--action-selection", "deterministic" if case["scenario"] == "nominal" else "stochastic",
            "--record-outputs", "--record-policy-trace", "--max-steps", str(case["maximum_steps"]),
            "--no-progress"]
    old_argv = sys.argv
    try:
        sys.argv = argv
        args = ev.parse_args()
    finally:
        sys.argv = old_argv
    ev._load_inference_stack()
    np, torch = ev.np, ev.torch
    freeze = bool(case.get("freeze_sigma", False))
    schedule_path = Path(case["sigma_path"])
    schedule = None
    if not freeze:
        payload = json.loads(schedule_path.read_text(encoding="utf-8"))
        schedule = np.asarray(payload["sigma"], dtype=np.float32)
        if schedule.shape != (case["maximum_steps"], 2) or not np.all(np.isfinite(schedule)) or np.any(schedule <= 0):
            raise ValueError("Baseline sigma schedule is incomplete or invalid")
    draws = paired_noise(int(case.get("noise_seed", 123)), case["maximum_steps"], 2)
    draws *= float(case.get("noise_scale", 1.0))
    records = []
    original = ev._policy_action_with_diagnostics
    ev._policy_action_with_diagnostics = fixed_sigma_selector(torch, np, schedule, draws, records, nominal=case["scenario"] == "nominal")
    summary = None
    try:
        summary = ev.run(args)
        complete = bool(summary["steps"] == case["maximum_steps"] and summary.get("end_reason") == "episode_time_limit")
        summary.update(completed_full_episode=complete, evaluation_protocol={**case, "sampler": "numpy.PCG64 float32 by step; fixed baseline nominal sigma"})
        _write(output / "rollout_summary.json", summary)
        if freeze:
            if not complete:
                raise RuntimeError("Baseline nominal did not complete; cannot freeze full sigma schedule")
            frozen = {"schema": "best80_nominal_sigma_by_step_v1", "checkpoint": case["checkpoint"],
                      "source_rollout": str(output), "sigma": [row["sigma"] for row in records]}
            if schedule_path.exists():
                existing = json.loads(schedule_path.read_text(encoding="utf-8"))
                if existing["checkpoint"] != frozen["checkpoint"] or existing["sigma"] != frozen["sigma"]:
                    raise ValueError("Previously frozen sigma schedule disagrees with rerun")
            else:
                _write(schedule_path, frozen)
    finally:
        ev._policy_action_with_diagnostics = original
        _write(output / "paired_noise_receipt.json", {"protocol": case, "records": records,
               "full_draws_sha256": noise_digest(draws), "draws_used": len(records),
               "sigma_sha256": hashlib.sha256(schedule_path.read_bytes()).hexdigest() if schedule_path.exists() else None,
               "note": "Same complete Gaussian stream and baseline sigma schedule; early failures use only the common prefix."})
    # Publish completeness only after every paired-noise and rollout artifact.
    # An interrupted write can never make a partially recorded case skippable.
    artifacts = [output / name for name in ("rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json")]
    artifacts += sorted((output / "sim_outputs").glob("*.sto"))
    hashes = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest() for path in artifacts}
    _write(output / "evaluation_complete.json", {"role": case["role"], "scenario": case["scenario"],
           "completed_full_episode": complete, "steps": len(records), "artifact_hashes": hashes})
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-json", type=Path, required=True)
    args = parser.parse_args()
    execute_case(json.loads(args.case_json.read_text(encoding="utf-8")))


if __name__ == "__main__":
    main()
