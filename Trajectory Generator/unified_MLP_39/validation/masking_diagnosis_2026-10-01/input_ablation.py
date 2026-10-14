"""Frozen, same-state input ablations; endpoint changes are not physical rollouts."""
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[key] = "1"
sys.path.insert(0, str(PACKAGE.parent))
import numpy as np
from unified_MLP_39 import rollout_eval as ev


def main():
    ev._load_inference_stack()
    ev.torch.set_num_threads(1)
    source = PACKAGE / "runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic"
    trace_file = source / "rollout_policy_trace.json"
    trace = json.loads(trace_file.read_text())
    obs = np.asarray([r["actor_observation_vector_before"] for r in trace], dtype=np.float32)
    recorded = np.asarray([r["raw_policy_action"] for r in trace])
    training = PACKAGE / "runs/training"
    masking = training / "MLP39_LEFT_best90_masking_100iter_2026-09-30"
    checkpoints = {
        "cp90": training / "MLP39_LEFT_fresh_best80config_100iter_2026-09-29/rl_module_best",
        "cp112": masking / "stage01_mask25/rl_module_best",
        "cp184": masking / "stage04_mask100/rl_module_best",
    }
    # Preserve one-row arithmetic of the recorded deployment inference.
    outputs, files = {}, [trace_file]
    for name, checkpoint in checkpoints.items():
        files.extend(p for p in checkpoint.rglob("*") if p.is_file())
        module = ev.RLModule.from_checkpoint(str(checkpoint))
        module.eval()
        outputs[name] = {}
        for variant, columns in {"visible": [], "clock_hidden": [0, 1],
                                 "target_hidden": [35, 36, 37, 38],
                                 "all_hidden": [0, 1, 35, 36, 37, 38]}.items():
            x = obs.copy()
            x[:, columns] = 0
            values = []
            with ev.torch.no_grad():
                for row in x:
                    full = np.zeros(module._n_full, np.float32)
                    full[:39] = row
                    logits = module.forward_inference({"obs": ev.torch.from_numpy(full[None])})["action_dist_inputs"]
                    values.append(logits.numpy()[0])
            outputs[name][variant] = np.asarray(values)
    np.testing.assert_allclose(outputs["cp90"]["visible"][:, :2], recorded, atol=2e-6, rtol=0)
    baseline = np.clip(outputs["cp90"]["visible"][:, :2], -1, 1)
    rows = []
    for name, variants in outputs.items():
        visible = np.clip(variants["visible"][:, :2], -1, 1)
        for variant, logits in variants.items():
            action = np.clip(logits[:, :2], -1, 1)
            factor = np.array([.75, .7]) * 180 / np.pi
            delta = (action - visible) * factor
            diff = (action - baseline) * factor
            rows.append({"checkpoint": name, "variant": variant,
                         "endpoint_rms_change_vs_same_weights_visible_deg": np.sqrt(np.mean(delta ** 2, axis=0)).tolist(),
                         "endpoint_rms_change_vs_cp90_visible_deg": np.sqrt(np.mean(diff ** 2, axis=0)).tolist(),
                         "gaussian_sigma_mean": np.exp(logits[:, 2:]).mean(axis=0).tolist(),
                         "clipped_fraction": (np.abs(logits[:, :2]) > 1).mean(axis=0).tolist()})
    result = {"note": "All models evaluated on CP90 visible visited states. Instantaneous input sensitivity only; ablations and changed models can be off-distribution. Degrees describe commanded endpoints before governor, not joints. Joint order knee, ankle.",
              "rows": len(obs), "max_reproduction_error": float(np.abs(outputs["cp90"]["visible"][:, :2]-recorded).max()),
              "results": rows,
              "input_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
    (HERE / "input_ablation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "input_sha256"}, indent=2))


if __name__ == "__main__":
    main()
