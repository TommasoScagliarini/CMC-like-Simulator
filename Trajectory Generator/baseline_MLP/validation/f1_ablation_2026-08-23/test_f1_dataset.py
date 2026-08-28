"""Self-test of f1_dataset on synthetic job directories (temp only)."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
from test_f1_obs_adapter import FakeClock, FakeTarget  # noqa: E402
from test_f1_sigma_variant import synthetic_state  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect(fn, exc, label):
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


def write_job(dir_: Path, *, steps: int, seed: int, start: str, stochastic: bool, reset_time: float, width: int = 35, rng=None) -> None:
    rng = rng or np.random.default_rng(seed)
    dir_.mkdir(parents=True)
    times = [reset_time + 0.01 * (k + 1) for k in range(steps)]
    rows = []
    for k in range(steps):
        obs = rng.standard_normal(width).tolist()
        obs[0], obs[1] = 0.0, 1.0
        mean = rng.standard_normal(2).tolist()
        noise = rng.standard_normal(2).tolist() if stochastic else None
        raw = [mean[0] + noise[0], mean[1] + noise[1]] if stochastic else mean
        rows.append({"step": k + 1, "time": times[k], "actor_observation_vector_before": obs, "raw_policy_action": raw, "policy_action_mean": mean if stochastic else None, "exploration_action_noise": noise})
    (dir_ / DS.TRACE_FILE).write_text(json.dumps(rows), encoding="utf-8")
    (dir_ / DS.SUMMARY_FILE).write_text(json.dumps({"ok": True, "steps": steps, "n_actor": width, "action_seed": seed, "action_selection": "stochastic" if stochastic else "deterministic", "episode_start_offset_s": F1.EXACT_STARTS[start], "end_reason": "episode_time_limit"}), encoding="utf-8")
    (dir_ / DS.RESET_FILE).write_text(json.dumps({"time": reset_time}), encoding="utf-8")


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="f1_dataset_"))
    names35, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39)
    clock = FakeClock([10.0 + k * 1.1 for k in range(12)])
    recon = OA.PrescribedTargetReconstructor(clock, FakeTarget(clock))
    teacher = DS.load_actor_arrays(synthetic_state(39, seed=7), expected_width=39)
    trajs = []
    adapters = {}
    for start in F1.STARTS:
        reset = 11.99 + F1.EXACT_STARTS[start]
        for seed in (123, 124, 125):
            d = tmp / f"B__{start}__stoch_seed{seed}"
            write_job(d, steps=40 if seed != 125 else 30, seed=seed, start=start, stochastic=True, reset_time=reset)
            trajs.append(DS.trajectory_from_job(d))
        ddet = tmp / f"B__{start}__det"
        write_job(ddet, steps=20, seed=123, start=start, stochastic=False, reset_time=reset)
        trajs.append(DS.trajectory_from_job(ddet))
        # adapter trace of the same start (35 steps: shorter than some B runs)
        t_pre = [reset] + [reset + 0.01 * (k + 1) for k in range(34)]
        side = [{"step_index": i + 1, "t_pre": t, "targets": recon.targets(t).tolist()} for i, t in enumerate(t_pre)]
        p = tmp / f"aiso__{start}" / DS.ADAPTER_TRACE_FILE
        p.parent.mkdir(parents=True)
        p.write_text(json.dumps(side), encoding="utf-8")
        adapters[repr(float(F1.EXACT_STARTS[start]))] = DS.targets_from_adapter_trace(p)
    tr = trajs[0]
    check(tr["steps"] == 40 and tr["stochastic"] and tr["seed"] == 123 and tr["t_pre"][0] == tr["reset_time"] and tr["t_pre"][1] == 11.99 + F1.EXACT_STARTS["minus020"] + 0.01, "trajectory parsing and t_pre")
    check(np.array_equal(tr["b_mean"] + 0.0, tr["b_mean"]) and tr["b_mean"].shape == (40, 2), "b_mean from policy_action_mean")
    det = [t for t in trajs if not t["stochastic"]][0]
    check(np.array_equal(det["b_mean"], det["b_raw_action"]), "deterministic trajectory: mean == raw action")
    # adapter shorter than B -> needs targets_fn; without it -> error
    expect(lambda: DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=adapters), DS.DatasetError, "adapter trace shorter than B without targets_fn refused")
    built = DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=adapters, targets_fn=recon.targets)
    a, r = built["arrays"], built["receipt"]
    check(r["rows"] == 3 * (40 + 40 + 30) and r["rows_train"] == 3 * 80 and r["rows_val"] == 3 * 30, "row counts per split (det excluded)")
    check(sorted(r["trajectories_train"]) == [i for i, t in enumerate(trajs) if t["stochastic"] and t["seed"] in (123, 124)] and sorted(r["trajectories_val"]) == [i for i, t in enumerate(trajs) if t["stochastic"] and t["seed"] == 125], "per-trajectory split by seed")
    check(not (set(r["trajectories_train"]) & set(r["trajectories_val"])), "no trajectory in both splits")
    check(a["obs35"].dtype == np.float32 and a["obs39"].shape == (r["rows"], 39) and a["teacher_mean"].shape == (r["rows"], 2), "array dtypes/shapes")
    check(np.array_equal(OA.project39to35(a["obs39"], spec), a["obs35"]), "obs39 projects exactly to obs35")
    per = [p for p in r["per_trajectory"] if "traj_index" in p]
    check(all(p["targets_source"]["from_adapter_trace"] == min(35, p["steps"]) and p["targets_source"]["from_targets_fn"] == max(0, p["steps"] - 35) for p in per), "targets from adapter trace then targets_fn")
    i0 = int(np.flatnonzero(a["traj_index"] == per[0]["traj_index"])[0])
    t0 = float(a["t_pre"][i0])
    check(np.array_equal(a["obs39"][i0, 2:6], recon.targets(t0).astype(np.float32)), "inserted targets at t_pre")
    logits = DS.actor_logits_numpy(teacher, a["obs39"][i0:i0 + 1].astype(np.float64))
    check(np.array_equal(a["teacher_mean"][i0], logits[0, :2].astype(np.float32)), "teacher mean = numpy forward on float32-cast obs39")
    check(all(k in r["arrays_sha256"] for k in a) and "baseline_b_vs_teacher_rmse" in r, "receipt digests")
    # include_det
    built2 = DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=adapters, targets_fn=recon.targets, include_det=True)
    check(built2["receipt"]["rows"] == r["rows"] + 3 * 20 and built2["receipt"]["rows_train"] == r["rows_train"] + 60, "include_det adds det rows to train")
    # negatives
    expect(lambda: DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, targets_fn=recon.targets, train_seeds=(126,), val_seeds=(125,)), RuntimeError, "sealed seed refused")
    expect(lambda: DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, targets_fn=recon.targets, train_seeds=(123,), val_seeds=(123,)), DS.DatasetError, "overlapping seeds refused")
    wrong = {k: {**v, "t_pre": v["t_pre"] + 1e-9} for k, v in adapters.items()}
    expect(lambda: DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=wrong, targets_fn=recon.targets), DS.DatasetError, "t_pre grid mismatch refused")
    other = OA.PrescribedTargetReconstructor(FakeClock([10.0 + k * 1.2 for k in range(12)]), FakeTarget(FakeClock([10.0 + k * 1.2 for k in range(12)])))
    expect(lambda: DS.build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=adapters, targets_fn=other.targets), DS.DatasetError, "targets_fn disagreeing with the adapter trace refused")
    # write / load round trip with digest verification
    written = DS.write_dataset(tmp / "ds", "T1", built)
    loaded = DS.load_dataset(Path(written["npz"]), Path(written["receipt"]))
    check(all(np.array_equal(loaded["arrays"][k], a[k]) for k in a), "npz round trip exact")
    expect(lambda: DS.write_dataset(tmp / "ds", "T1", built), FileExistsError, "no-clobber")
    npz = Path(written["npz"])
    data = dict(np.load(npz))
    data["teacher_mean"] = data["teacher_mean"] + np.float32(1e-3)
    np.savez(npz, **data)
    expect(lambda: DS.load_dataset(npz, Path(written["receipt"])), DS.DatasetError, "tampered npz refused")
    # malformed job
    badj = tmp / "bad"
    write_job(badj, steps=5, seed=123, start="nominal", stochastic=True, reset_time=13.9)
    rows = json.loads((badj / DS.TRACE_FILE).read_text())
    rows[2]["step"] = 9
    (badj / DS.TRACE_FILE).write_text(json.dumps(rows))
    expect(lambda: DS.trajectory_from_job(badj), DS.DatasetError, "non-contiguous steps refused")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
