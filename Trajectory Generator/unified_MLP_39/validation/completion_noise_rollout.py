"""Paired noise diagnostic; never changes a checkpoint or training settings.

Keep the environment seed fixed, varying only an independent Gaussian generator.
Scale 1 reproduces the exploration distribution, 0 the deterministic mean.
The standard evaluator supplies environment/config checks, traces and watchdog.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

TG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TG))


def paired_selector(torch, scale, seed, records):
    generator = torch.Generator(device="cpu").manual_seed(seed)

    def select(module, obs, action_shape, *, action_selection="deterministic"):
        import numpy as np
        obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
        with torch.no_grad():
            forward = (module.forward_inference if scale == 0 else module.forward_exploration)
            logits = forward({"obs": obs_t})["action_dist_inputs"]
            dim = logits.shape[-1] // 2
            mean, std = logits[..., :dim], torch.exp(logits[..., dim:])
            if scale:
                # The configured policy is a CPU diagonal Gaussian, as in PPO.
                dist = module.get_exploration_action_dist_cls().from_logits(logits)
                if not torch.allclose(dist._dist.mean, mean) or not torch.allclose(dist._dist.stddev, std):
                    raise RuntimeError("Exploration distribution is not the expected Gaussian")
                z = torch.randn(mean.shape, generator=generator, dtype=mean.dtype)
            else:
                z = torch.zeros_like(mean)
            noise = std * z * scale
            action = mean + noise
        def array(value):
            return value.detach().cpu().numpy().reshape(action_shape).astype(np.float32)
        records.append({"step": len(records) + 1, "standard_normal": array(z).tolist(),
                        "learned_std": array(std).tolist(),
                        "action_dist_inputs": logits.detach().cpu().numpy().reshape(-1).tolist()})
        return array(action), array(mean), array(std * scale), array(noise)
    return select


def main():
    extra = argparse.ArgumentParser(add_help=False)
    extra.add_argument("--noise-scale", type=float, required=True)
    extra.add_argument("--noise-seed", type=int, default=123)
    extra.add_argument("--telemetry-dir", type=Path, default=None,
                       help="Optional physical A/B preflight of the episode observer")
    options, remaining = extra.parse_known_args()
    if not 0 <= options.noise_scale <= 1:
        extra.error("noise-scale must be in [0, 1]")
    from unified_MLP_39 import rollout_eval
    original_argv = sys.argv
    try:
        sys.argv = [original_argv[0], *remaining]
        args = rollout_eval.parse_args()
    finally:
        sys.argv = original_argv
    args.action_selection = "deterministic" if options.noise_scale == 0 else "stochastic"
    if not args.worker_process:
        # Preserve this diagnostic wrapper (and its arguments) in the child.
        rollout_eval._SCRIPT_PATH = Path(__file__).resolve()
        return rollout_eval.run_supervised(args)
    import torch
    torch.set_num_threads(1)
    records = []
    rollout_eval._policy_action_with_diagnostics = paired_selector(
        torch, options.noise_scale, options.noise_seed, records)
    telemetry_timing = []
    sink = None
    if options.telemetry_dir is not None:
        if not options.telemetry_dir.is_absolute():
            raise ValueError("telemetry-dir must be absolute")
        from types import SimpleNamespace
        import gymnasium as gym
        from ray.rllib.env.single_agent_episode import SingleAgentEpisode
        from unified_MLP_39 import env_factory
        from unified_MLP_39.episode_telemetry import EpisodeTelemetry
        sink = EpisodeTelemetry(str(options.telemetry_dir))
        runner = SimpleNamespace(worker_index=0, _weights_seq_no=40)
        original_make = env_factory.make_cmc_env

        class ObservedEnvironment(gym.Wrapper):
            def reset(self, **kwargs):
                observation, info = self.env.reset(**kwargs)
                self.episode = SingleAgentEpisode(id_="physical-preflight-fixed-id",
                    observation_space=self.observation_space, action_space=self.action_space)
                self.episode.add_env_reset(observation, infos=info)
                return observation, info

            def step(self, action):
                result = self.env.step(action)
                observation, reward, terminated, truncated, info = result
                self.episode.add_env_step(observation, action, reward, infos=info,
                    terminated=terminated, truncated=truncated,
                    extra_model_outputs={"action_dist_inputs": records[-1]['action_dist_inputs'],
                                         "weights_seq_no":40})
                tick=time.perf_counter()
                sink.observe(self.episode, env_runner=runner)
                if terminated or truncated:
                    sink.finish(self.episode, env_runner=runner, prev_episode_chunks=[])
                telemetry_timing.append(time.perf_counter()-tick)
                return result
        env_factory.make_cmc_env = lambda config: ObservedEnvironment(original_make(config))
    started = time.monotonic()
    summary = rollout_eval.run(args)
    output = Path(args.output_dir)
    encoded = json.dumps(records, separators=(",", ":")) + "\n"
    (output / "paired_noise_draws.json").write_text(encoded)
    provenance = {"noise_scale": options.noise_scale, "noise_seed": options.noise_seed,
                  "environment_seed": args.seed, "generator": "torch.Generator(cpu)",
                  "draws_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
                  "wall_seconds": time.monotonic() - started,
                  "note": "Independent per-step Gaussian; scale affects evaluation only."}
    if sink is not None:
        provenance['telemetry_preflight'] = {
            "directory":str(options.telemetry_dir), "metrics":sink.metrics,
            "observer_wall_seconds":sum(telemetry_timing),
            "observer_mean_microseconds":1e6*sum(telemetry_timing)/len(telemetry_timing),
            "observer_max_microseconds":1e6*max(telemetry_timing),
        }
    (output / "noise_diagnostic.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps({**provenance, **{k: summary.get(k) for k in (
        "steps", "end_reason", "episode_return", "action_clipped_fraction",
        "grf_penetration_max_m")}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
