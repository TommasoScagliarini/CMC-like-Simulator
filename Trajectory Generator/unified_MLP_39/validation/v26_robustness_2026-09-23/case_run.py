"""One isolated V26 RIGHT evaluation, with unchanged simulator/FSM v3.

Reuse the frozen diagnostic loop and its runtime gates. The only intervention
is the action selector: original arithmetic via an input adapter, or paired
Gaussian exploration of the converted unified39 policy. No weights are edited.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
VALIDATION = HERE.parent
PACKAGE = VALIDATION.parent
REPO = PACKAGE.parents[1]
ROOT = PACKAGE / "runs/rollout/V26_robustness_2026-09-23"
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[name] = "1"
sys.path.insert(0, str(PACKAGE.parent))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


probe = load("frozen_v26_transfer_probe", VALIDATION / "v26_weight_transfer_2026-09-23/probe.py")
noise_helper = load("paired_noise_diagnostic", VALIDATION / "completion_noise_rollout.py")


def guard():
    hashes = probe.read(VALIDATION / "v26_weight_transfer_2026-09-23/input_hashes.json")
    for path, digest in hashes.items():
        if probe.sha(REPO / path) != digest:
            raise RuntimeError(f"Frozen diagnostic input changed: {path}")
    return len(hashes)


class OriginalOrderAdapter:
    def __init__(self, module, torch):
        self.module, self.torch = module, torch

    def _batch(self, batch):
        x = batch["obs"]
        restored = x.clone()
        restored[..., probe.PERM] = x[..., :39]
        return {**batch, "obs": restored}

    def forward_inference(self, batch):
        return self.module.forward_inference(self._batch(batch))

    def forward_exploration(self, batch):
        return self.module.forward_exploration(self._batch(batch))

    def get_exploration_action_dist_cls(self):
        return self.module.get_exploration_action_dist_cls()


def preflight():
    ev = probe.stack()
    np, torch = ev.np, ev.torch
    source = ev.RLModule.from_checkpoint(probe.SOURCE)
    source.eval()
    adapter = OriginalOrderAdapter(source, torch)
    trace = probe.read(probe.HISTORICAL / "rollout_policy_trace.json")
    recorded = np.asarray([r["raw_policy_action"] for r in trace], np.float32)
    values = []
    with torch.no_grad():
        for row in trace:
            # Preserve the 88-wide layout used by the ordinary rollout. Only
            # the actor prefix is consumed; the archived suffix is unavailable.
            old = np.zeros(88, np.float32)
            old[:39] = np.asarray(row["actor_observation_vector_before"][:39], np.float32)
            new = old.copy()
            new[:39] = old[probe.PERM]
            a = source.forward_inference({"obs": torch.from_numpy(old[None])})["action_dist_inputs"]
            b = adapter.forward_inference({"obs": torch.from_numpy(new[None])})["action_dist_inputs"]
            assert torch.equal(a, b), "Input adapter must preserve original arithmetic exactly"
            values.append(b.numpy()[0, :2])
    values = np.asarray(values)
    np.testing.assert_allclose(values, recorded, atol=2e-6, rtol=0)
    result = {"frozen_inputs_verified": guard(), "rows": len(trace), "adapter_vs_original_bitwise_equal": True,
              "historical_actions_max_abs_error": float(abs(values-recorded).max()),
              "historical_actions_exact_rows": int(np.all(values == recorded, axis=1).sum())}
    probe.write(HERE / "preflight.json", result)
    print(json.dumps(result, indent=2))


def worker(args, folder):
    guard()
    ev = probe.stack()
    import gymnasium as gym
    np, torch = ev.np, ev.torch
    source = ev.RLModule.from_checkpoint(probe.SOURCE)
    source.eval()
    adapter = OriginalOrderAdapter(source, torch)
    records, transitions = [], []
    selector = noise_helper.paired_selector(torch, args.scale, args.seed, records)
    actual_checkpoint = probe.SOURCE if args.order == "original" else probe.EXPORT
    protocol = {"case": args.case, "arithmetic_order": args.order, "noise_scale": args.scale,
                "noise_seed": args.seed, "environment_seed": 123,
                "checkpoint_actually_used": str(actual_checkpoint), "fsm": "v3",
                "training": False, "maximum_steps": 500, "target_source": "prescribed_right_phase_template",
                "sampler": "completion_noise_rollout.paired_selector; CPU Torch Gaussian, per step"}
    folder.mkdir(parents=True, exist_ok=True)
    if (folder / "rollout_summary.json").exists():
        raise FileExistsError(f"Completed case already exists: {folder}")
    probe.write(folder / "case_protocol.json", protocol)
    old_selector, old_make, old_write, old_folder = ev._policy_action_with_diagnostics, probe.make_env, probe.write, probe.ROLLOUT
    captured = {}

    def select(module, obs, shape, **kwargs):
        return selector(adapter if args.order == "original" else module, obs, shape, **kwargs)

    class TerminalObserver(gym.Wrapper):
        def step(self, action):
            time_before = float(self.unwrapped.t)
            result = self.env.step(action)
            obs, reward, terminated, truncated, info = result
            time_after = float(info.get("time", self.unwrapped.t))
            transitions.append({"step": len(transitions) + 1, "time_before": time_before,
                                "time_after": time_after, "time_advanced_s": time_after-time_before,
                                "terminated": bool(terminated), "truncated": bool(truncated),
                                "end_reason": info.get("end_reason")})
            if terminated or truncated:
                old_write(folder / "terminal_info.json", info)
            return result

    def make(output):
        env, raw = old_make(output)
        return TerminalObserver(env), raw

    def write(path, value):
        if Path(path).name == "rollout_policy_trace.json":
            captured["trace"] = value
        if Path(path).name == "rollout_summary.json":
            value = dict(value)
            trace = captured["trace"]
            raw = np.asarray([r["raw_policy_action"] for r in trace])
            applied = np.asarray([r["applied_policy_action"] for r in trace])
            clipped = raw != applied
            value.update(action_selection="deterministic" if args.scale == 0 else "stochastic",
                         action_seed=args.seed, environment_seed=123, checkpoint=str(actual_checkpoint),
                         evaluation_protocol=protocol, exploration_noise_scale=args.scale,
                         exploration_noise_hold_steps=1, action_clipped_steps=int(np.any(clipped, axis=1).sum()),
                         action_clipped_fraction=float(clipped.mean()), action_abs_max=float(np.abs(raw).max()),
                         applied_action_abs_max=float(np.abs(applied).max()))
            value["diagnostic_intervention"] = dict(value["diagnostic_intervention"], evaluation_protocol=protocol)
            value["terminal_step_partial"] = bool(transitions[-1]["time_advanced_s"] < .01-1e-7)
            value["completed_full_episode"] = bool(value["steps"] == 500 and value["end_reason"] == "episode_time_limit")
            captured["summary"] = value
        old_write(path, value)

    started = time.monotonic()
    try:
        ev._policy_action_with_diagnostics = select
        probe.make_env, probe.write, probe.ROLLOUT = make, write, folder
        probe.rollout()
        assert len(records) == len(transitions) == captured["summary"]["steps"]
        guard()
    finally:
        ev._policy_action_with_diagnostics = old_selector
        probe.make_env, probe.write, probe.ROLLOUT = old_make, old_write, old_folder
        encoded = json.dumps(records, separators=(",", ":")) + "\n"
        (folder / "paired_noise_draws.json").write_text(encoded)
        old_write(folder / "transition_timing.json", transitions)
        old_write(folder / "noise_diagnostic.json", {
            **protocol, "generator": "torch.Generator(cpu)",
            "draws_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
            "wall_seconds": time.monotonic()-started,
            "note": "Same paired Gaussian selector as best72; each policy retains its own learned std. Hard partial terminal is recorded separately."})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--case")
    parser.add_argument("--order", choices=("original", "converted"), default="converted")
    parser.add_argument("--scale", type=float, choices=(0., .5, 1.), default=0.)
    parser.add_argument("--seed", type=int, choices=range(123, 128), default=123)
    parser.add_argument("--worker", action="store_true")
    args = parser.parse_args()
    if args.preflight:
        return preflight()
    if not args.case or Path(args.case).name != args.case:
        parser.error("A simple --case directory name is required")
    if args.order == "original" and args.scale != 0:
        parser.error("Original-order intervention is the deterministic control only")
    folder = ROOT / args.case
    if args.worker:
        return worker(args, folder)
    from baseline_MLP import process_watchdog
    result = process_watchdog.supervise_process(
        [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"],
        heartbeat_file=folder / "watchdog_state.json", summary_file=folder / "watchdog_summary.json",
        startup_timeout_s=600., stall_timeout_s=240., run_timeout_s=2400., label=args.case)
    raise SystemExit(result["returncode"] if result["timeout_reason"] is None else 124)


if __name__ == "__main__":
    main()
