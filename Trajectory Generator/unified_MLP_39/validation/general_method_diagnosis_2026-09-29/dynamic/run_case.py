"""Frozen-policy impulse/sequence diagnostics through the ordinary evaluator."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
RUNS = PACKAGE / "runs/rollout/general_method_diagnosis_2026-09-29"
SOURCES = {
    "LEFT": PACKAGE / "runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic",
    "RIGHT": PACKAGE / "runs/rollout/MLP39_right_v26_60iter_2026-09-23_last60",
}
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"
sys.path.insert(0, str(TG))


def write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args, folder):
    from unified_MLP_39 import rollout_eval as ev
    source = SOURCES[args.source]
    archived = json.loads((source / "rollout_summary.json").read_text())
    checkpoint = Path(archived["checkpoint"])
    if not checkpoint.exists():
        marker = "Trajectory Generator/"
        checkpoint = TG / archived["checkpoint"].replace("\\", "/").split(marker, 1)[1]
    state = checkpoint / "module_state.pkl"
    before = digest(state)
    argv = [str(PACKAGE / "rollout_eval.py"), "--checkpoint", str(checkpoint),
            "--output-dir", str(folder), "--seed", "123", "--action-selection", "deterministic",
            "--record-outputs", "--record-policy-trace", "--max-steps", str(args.steps), "--no-progress"]
    old_argv = sys.argv
    try:
        sys.argv = argv
        eval_args = ev.parse_args()
    finally:
        sys.argv = old_argv
    ev._load_inference_stack()
    ev.torch.set_num_threads(1)
    np = ev.np
    original = ev._policy_action_with_diagnostics
    original_factory = ev.env_factory.make_cmc_env
    captured = {}
    def factory(config):
        env = original_factory(config)
        captured["base"] = env.unwrapped
        return env
    records = []
    actions = None
    if args.sequence:
        data = np.load(args.sequence)
        actions = data["normalized_action"].astype(np.float32)
        if actions.shape != (args.steps, 2) or not np.all(np.isfinite(actions)) or np.max(np.abs(actions)) > 1:
            raise ValueError("Candidate sequence must have finite, bounded (steps, 2) normalized_action")
    protocol = {"source": args.source, "mode": args.mode, "impulse_index_zero_based": args.impulse_index,
                "impulse_normalized": args.epsilon if args.mode.startswith("impulse") else 0,
                "checkpoint": str(checkpoint.relative_to(REPO)), "checkpoint_sha256": before,
                "sequence": str(args.sequence) if args.sequence else None,
                "sequence_sha256": digest(args.sequence) if args.sequence else None,
                "steps_requested": args.steps, "seed": 123, "training": False,
                "prescribed_visible": True, "fsm": "v3"}
    write(folder / "diagnostic_protocol.json", protocol)

    def select(module, obs, shape, **kwargs):
        index = len(records)
        action, mean, std, _ = original(module, obs, shape, **kwargs)
        if args.mode == "sequence":
            base = captured["base"]
            np.testing.assert_allclose(base.t, data["time_start"][index], atol=1e-9, rtol=0)
            target_q, target_qdot, _ = base.imitation_target(float(data["time_end"][index]))
            np.testing.assert_allclose([target_q[k] for k in base.cfg.pros_coords],data["target_q"][index],atol=1e-7,rtol=0)
            np.testing.assert_allclose([target_qdot[k] for k in base.cfg.pros_coords],data["target_qdot"][index],atol=1e-7,rtol=0)
            command = actions[index].reshape(shape).copy()
        else:
            command = action.copy()
            if index == args.impulse_index and args.mode.startswith("impulse"):
                joint = 0 if args.mode == "impulse_knee" else 1
                command.reshape(-1)[joint] += np.float32(args.epsilon)
        records.append({"index": index, "policy_mean": mean.reshape(-1).tolist(),
                        "command": command.reshape(-1).tolist(),
                        "intervention": (command-action).reshape(-1).tolist(),
                        "clipped_intervention": (np.clip(command, -1, 1)-np.clip(action, -1, 1)).reshape(-1).tolist()})
        return command, mean, std, command-mean

    started = time.monotonic()
    ev._policy_action_with_diagnostics = select
    ev.env_factory.make_cmc_env = factory
    try:
        summary = ev.run(eval_args)
    finally:
        ev._policy_action_with_diagnostics = original
        ev.env_factory.make_cmc_env = original_factory
        write(folder / "action_interventions.json", records)
    assert digest(state) == before, "Checkpoint mutated during inference"
    trace = json.loads((folder / "rollout_policy_trace.json").read_text())
    assert len(records) == len(trace) == summary["steps"]
    old = json.loads((source / "rollout_policy_trace.json").read_text())
    count = min(args.impulse_index, len(trace)) if args.mode.startswith("impulse") else (len(trace) if args.mode == "baseline" else 0)
    differences = {}
    for key in ("raw_policy_action", "actor_observation_vector_before", "policy_segment_values"):
        if count:
            a = np.asarray([r[key] for r in trace[:count]], dtype=float)
            b = np.asarray([r[key] for r in old[:count]], dtype=float)
            differences[key] = {"max_abs": float(np.max(np.abs(a-b))), "exact": bool(np.array_equal(a,b))}
    summary["diagnostic_intervention"] = protocol
    summary["diagnostic_wall_seconds"] = time.monotonic()-started
    write(folder / "rollout_summary.json", summary)
    receipt = {"steps": summary["steps"], "end_reason": summary["end_reason"],
               "checkpoint_unchanged": True, "comparison_to_archive_prefix_steps": count,
               "archive_replay": differences, "wall_seconds": summary["diagnostic_wall_seconds"]}
    if args.mode == "sequence":
        completed = [i for i,r in enumerate(trace) if r.get("segment_completed",True)]
        keys = ("_served_ref", "_served_ref_vel", "_served_ref_accel")
        errors = {}
        for suffix, name in zip(keys,("served_q","served_qdot","served_qddot")):
            actual = np.asarray([[trace[i]["prosthetic_state"][f"pros_{j}_angle{suffix}"] for j in ("knee","ankle")] for i in completed])
            expected = data[name][completed]
            errors[name] = float(np.max(np.abs(actual-expected))) if completed else None
            if completed:
                np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=1e-6)
        receipt["offline_governor_vs_physical_served_max_error"] = errors
    write(folder / "diagnostic_receipt.json", receipt)
    print(json.dumps(receipt), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=SOURCES, required=True)
    parser.add_argument("--mode", choices=("baseline", "impulse_knee", "impulse_ankle", "sequence"), required=True)
    parser.add_argument("--impulse-index", type=int, default=101)
    parser.add_argument("--epsilon", type=float, default=.005)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--sequence", type=Path)
    parser.add_argument("--worker", action="store_true")
    args = parser.parse_args()
    if (args.mode == "sequence") != bool(args.sequence):
        parser.error("--sequence is required exactly for sequence mode")
    folder = RUNS / f"{args.source}_{args.mode}"
    if args.worker:
        return run(args, folder)
    if folder.exists():
        raise FileExistsError(f"Refusing to overwrite diagnostic case {folder}")
    folder.mkdir(parents=True)
    from baseline_MLP import process_watchdog
    result = process_watchdog.supervise_process(
        [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"],
        heartbeat_file=folder / "watchdog_state.json", summary_file=folder / "watchdog_summary.json",
        startup_timeout_s=180., stall_timeout_s=120., run_timeout_s=1800., label=folder.name)
    raise SystemExit(result["returncode"] if result["timeout_reason"] is None else 124)


if __name__ == "__main__":
    main()
