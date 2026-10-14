"""Audit paired Gaussian trials, including archived V26 and LEFTbest72.

No simulation, training or checkpoint mutation. A partial terminal is kept for
failure/reward counts but excluded from common-horizon physical metrics.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
PLAN = PACKAGE / "runs/plans/right_reference_screening_2026-09-23"
ROOT = PACKAGE / "runs/rollout/MLP39_right_screening_2026-09-23"
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(name, "1")
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


prior = load("right_screening_prior_metrics", HERE.parent / "v26_robustness_2026-09-23/analyze.py")
h, np, torch, plt = prior.helpers, prior.np, prior.torch, prior.plt
physical = prior.previous
JOINTS = prior.JOINTS
FIGURES = REPO / "plot/09_23_2026_right_screening_comparison"
ROOTS = {"best35": ROOT / "best35", "last40": ROOT / "last40",
         "v26": prior.ROOT, "left_best72": prior.BEST72_ROOT}


def counters(trace):
    keys = ("valid_cycle_count", "valid_hs_count", "valid_to_count", "invalid_event_count", "resync_count", "hs_cancelled_count")
    return {key: max((float(r["phase_fsm"][key]) for r in trace if r.get("phase_fsm", {}).get(key) is not None), default=None) for key in keys}


def guard():
    hashes = json.loads((PLAN / "input_hashes.json").read_text())
    for path, digest in hashes.items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == digest, path
    return len(hashes)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--partial", action="store_true", help="Summarize only completed new cases; does not certify the full screening")
    args = p.parse_args()
    frozen = guard()
    if not args.partial:
        assert h.read(PLAN / "status.json")["state"] == "completed"
        assert h.read(PLAN / "results.json")["technical_pass"]
    episodes, valid, draws, traces = {}, {}, {}, {}
    historical = h.read(physical.OLD / "rollout_policy_trace.json")
    for label, base in ROOTS.items():
        for seed in range(123, 128):
            folder = base / f"noise_1_seed{seed}"
            if args.partial and label in ("best35", "last40") and not (folder / "watchdog_summary.json").exists():
                continue
            metric, trace, good, noise = prior.noise_episode(folder, label, 1., seed)
            metric["fsm_counters"] = counters(trace)
            if label in ("best35", "last40"):
                assert all(r["target_availability"]["visible"] for r in trace)
                for row, old in zip(good, historical):
                    np.testing.assert_allclose(row["time"], old["time"], rtol=0, atol=2e-8)
                    for joint in JOINTS:
                        np.testing.assert_allclose(row["imitation_target_q"][joint], old["imitation_target_q"][joint], rtol=0, atol=1e-10)
                        np.testing.assert_allclose(row["imitation_target_qdot"][joint], old["imitation_target_qdot"][joint], rtol=0, atol=1e-10)
                    for key in ("gait_phase_sin", "gait_phase_cos"):
                        np.testing.assert_allclose(row["actor_observation_before"][key], old["actor_observation_before"][key], rtol=0, atol=1e-7)
                metric["right_target_clock_visible_audit"] = True
            episodes[label, seed], traces[label, seed], valid[label, seed], draws[label, seed] = metric, trace, good, noise
    if not args.partial:
        assert len(episodes) == 20
    common = min(len(x) for x in valid.values())
    for key, metric in episodes.items():
        metric["common_prefix_rmse_degrees"] = prior.rmse_rows(valid[key][:common])
        metric["common_prefix_sea_metrics"] = prior.sea_metrics(valid[key][:common])
        metric["common_prefix_action_metrics"] = prior.prefix_action_diagnostics(valid[key][:common], draws[key][:common], 1.)
        folder = Path(metric["rollout"])
        if metric["signed_torque_sto_available"]:
            table = physical.read_table(folder, "sea_torques")
            # Keep the same shared horizon for every policy whose STO exists.
            mask = (table.time >= table.time[0] + .2 - 1e-8) & (table.time < table.time[0] + common * .01 - 1e-8)
            if mask.sum() >= 20:
                metric["common_prefix_signed_torque_after200ms"] = {
                    short: physical.torque_window(table.time,
                        table.data[:, table.index[f"SEA_{short.title()}_tau_spring"]] * physical.plotter.joint_sign(short), mask)
                    for short in ("knee", "ankle")}
    paired = []
    for seed in range(123, 128):
        keys = [key for key in draws if key[1] == seed]
        n = min(len(draws[key]) for key in keys)
        baseline = [d["standard_normal"] for d in draws[keys[0]][:n]]
        assert all([d["standard_normal"] for d in draws[key][:n]] == baseline for key in keys)
        paired.append({"seed": seed, "policies": [key[0] for key in keys], "matched_normal_draws": n})
    groups = []
    for label in ROOTS:
        group = [m for (policy, _), m in episodes.items() if policy == label]
        if not group:
            continue
        successes = sum(m["completed"] for m in group)
        counter_values = lambda name: [m["fsm_counters"][name] for m in group if m["fsm_counters"][name] is not None]
        groups.append({"policy": label, "episodes": len(group), "completed": successes,
            "completion_fraction": successes / len(group), "wilson95": prior.wilson(successes, len(group)),
            "mean_steps": float(np.mean([m["steps"] for m in group])),
            "mean_return": float(np.mean([m["episode_return"] for m in group])),
            "end_reasons": {reason: sum(m["end_reason"] == reason for m in group) for reason in sorted({m["end_reason"] for m in group})},
            "two_valid_cycles": sum(m["fsm_counters"]["valid_cycle_count"] >= 2 for m in group),
            "completed_with_two_cycles_and_zero_invalid": sum(m["completed"] and m["fsm_counters"]["valid_cycle_count"] >= 2 and m["fsm_counters"]["invalid_event_count"] == 0 for m in group),
            "total_invalid_events": sum(counter_values("invalid_event_count")),
            "total_resync": sum(counter_values("resync_count")) if counter_values("resync_count") else None,
            "max_recorded_penetration_mm": 1000 * max(m["grf_penetration_max_m"] for m in group),
            "hard_guard_cases": sum(m["hard_guard_exceeded"] for m in group),
            "common_prefix_mean_rmse_degrees": {j: float(np.mean([m["common_prefix_rmse_degrees"][j] for m in group])) for j in JOINTS},
            "common_prefix_mean_learned_std": np.mean([m["common_prefix_action_metrics"]["learned_std_mean_normalized"] for m in group], axis=0).tolist()})
    result = {"complete_screening": not args.partial, "frozen_inputs": frozen, "groups": groups,
        "episodes": list(episodes.values()), "paired_noise_audit": paired, "common_prefix_steps": common,
        "method": {"noise_scale": 1, "seeds": list(range(123, 128)), "environment_seed": 123,
            "same_gaussian_draws": True, "each_policy_retains_learned_std": True,
            "completed_with_two_cycles_and_zero_invalid": "Additional descriptive criterion, not the definition of episode completion."},
        "limitations": ["Five noise seeds at one fixed start; not statistical equivalence or independent training replicates.",
            "V26best87 and LEFTbest72 are more mature; LEFT uses a different target.",
            "Policy std differs and depends on visited states; matching Gaussian draws does not match absolute noise.",
            "Terminal partial samples are excluded from physical metrics; hard penetration may exceed the saved maximum.",
            "Full-episode counters remain unnormalized when episodes end early; common-prefix physical metrics share a horizon.",
            "LEFTbest72 stochastic STOs were not recorded; no signed torque or highpass peak is invented."]}
    path = HERE / ("partial_metrics.json" if args.partial else "metrics.json")
    h.dump(path, result)
    h.dump(HERE / ("partial_input_hashes.json" if args.partial else "input_hashes.json"), h.INPUT_HASHES)
    if not args.partial:
        FIGURES.mkdir(parents=True, exist_ok=True)
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained", sharex=True, sharey=True)
        for ax, label in zip(axes.flat, ROOTS):
            for seed in range(123, 128):
                trace = valid[label, seed]
                t = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
                ax.plot(t, [1000 * r["reward_terms"].get("grf_penetration_m", 0) for r in trace], label=str(seed), lw=1)
            group = next(g for g in groups if g["policy"] == label)
            ax.axhline(28, color="gray", ls="--", label="Soglia terminale")
            ax.set(title=f"{label}: {group['completed']}/5 completi; {group['two_valid_cycles']}/5 con 2 cicli", xlabel="Tempo dal reset [s]", ylabel="Penetrazione registrata [mm]")
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
        fig.suptitle("Rumore originale: stessi seed gaussiani, std appresa da ogni policy")
        fig.savefig(FIGURES / "paired_penetration.png", dpi=165)
        plt.close(fig)
    print(json.dumps({"complete_screening": not args.partial, "groups": groups, "common_prefix_steps": common}, indent=2))


if __name__ == "__main__":
    main()
