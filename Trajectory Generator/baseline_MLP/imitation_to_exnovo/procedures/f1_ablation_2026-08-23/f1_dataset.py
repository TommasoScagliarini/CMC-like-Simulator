"""Build the supervised dataset for candidate D (F1 stage 2) from frozen B
trajectories.  Pure numpy; no simulation.

Rows: every step of the selected B rollouts (F1 matrix, stochastic native,
development seeds only).  For each step

* ``obs35``   = ``actor_observation_vector_before`` (the real env vector);
* ``t_pre``   = env time at which the observation was taken
                (``[reset_time] + time[:-1]``, exact floats);
* ``obs39``   = ``obs35`` with the four prescribed targets inserted at 2:6.
                The targets come from the A_iso adapter side-car trace of the
                **same start** (``f1_adapter_trace.json``: the env time grid
                depends only on start/dt, so ``t_pre`` must match exactly) or,
                when the A_iso run ended earlier than the B run, from an
                injected ``targets_fn(t)`` (the env-built reconstructor);
* ``teacher`` = deterministic mean of the V26 39D actor on ``obs39``
                (float32-cast inputs, float64 numpy forward identical to
                ``f0_actor_drift.actor_logits``; max deviation from the torch
                path measured in F0 at 2e-7);
* ``b_mean``  = B's own recorded mean (``policy_action_mean``) for the
                baseline fidelity.

Split: **per trajectory** — train = seeds ``123, 124`` (all three starts),
validation = seed ``125``; never mixed; deterministic rollouts excluded unless
``include_det`` (preregistered: excluded).  Sealed seeds are refused.

Output (no-clobber): ``<out_dir>/f1_dataset_<stamp>.npz`` + ``.json`` receipt
with SHA-256 of every consumed file and of every array.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
for entry in (str(HERE),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402

TRACE_FILE = "rollout_policy_trace.json"
SUMMARY_FILE = "rollout_summary.json"
RESET_FILE = "rollout_reset_diagnostics.json"
RECEIPT_FILE = "f1_receipt.json"
ADAPTER_TRACE_FILE = "f1_adapter_trace.json"
TRAIN_SEEDS = (123, 124)
VAL_SEEDS = (125,)
SPLIT_TRAIN = 0
SPLIT_VAL = 1


class DatasetError(RuntimeError):
    pass


def sha256_array(arr: np.ndarray) -> str:
    a = np.ascontiguousarray(arr)
    h = hashlib.sha256()
    h.update(str(a.dtype).encode("ascii"))
    h.update(repr(tuple(int(d) for d in a.shape)).encode("ascii"))
    h.update(a.tobytes(order="C"))
    return h.hexdigest()


def actor_logits_numpy(arrays: dict[str, np.ndarray], obs: np.ndarray) -> np.ndarray:
    """tanh-tanh-linear forward, float64 (same algebra as f0_actor_drift.actor_logits)."""
    x = np.asarray(obs, dtype=np.float64)
    h1 = np.tanh(x @ arrays["pi.0.0.weight"].T + arrays["pi.0.0.bias"])
    h2 = np.tanh(h1 @ arrays["pi.0.2.weight"].T + arrays["pi.0.2.bias"])
    return h2 @ arrays["pi.1.weight"].T + arrays["pi.1.bias"]


def load_actor_arrays(state: dict[str, Any], *, expected_width: int) -> dict[str, np.ndarray]:
    keys = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")
    out = {}
    for k in keys:
        if k not in state:
            raise DatasetError(f"actor state missing {k}")
        out[k] = np.asarray(state[k], dtype=np.float64)
    if out["pi.0.0.weight"].shape[1] != expected_width or out["pi.1.weight"].shape[0] != 2 * F1.ACTION_DIM:
        raise DatasetError(f"actor ABI mismatch: first layer {out['pi.0.0.weight'].shape}, head {out['pi.1.weight'].shape}")
    return out


def trajectory_from_job(job_dir: Path, *, expected_width: int = F1.ENV_ACTOR_WIDTH) -> dict[str, Any]:
    """Read one rollout_eval job directory into arrays (obs35 rows, t_pre, means)."""
    job_dir = Path(job_dir)
    trace_path, summary_path, reset_path = job_dir / TRACE_FILE, job_dir / SUMMARY_FILE, job_dir / RESET_FILE
    for p in (trace_path, summary_path, reset_path):
        if p.is_symlink() or not p.is_file():
            raise DatasetError(f"missing job file: {p}")
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    reset = json.loads(reset_path.read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise DatasetError(f"empty or malformed trace: {trace_path}")
    if int(summary.get("n_actor", -1)) != expected_width or int(summary.get("steps", -1)) != len(rows):
        raise DatasetError(f"summary/trace mismatch in {job_dir}: n_actor={summary.get('n_actor')} steps={summary.get('steps')} rows={len(rows)}")
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, len(rows) + 1)):
        raise DatasetError("trace steps are not 1..N")
    reset_time = reset.get("time")
    if not isinstance(reset_time, (int, float)):
        raise DatasetError("reset diagnostics without a numeric 'time'")
    times = [float(r["time"]) for r in rows]
    t_pre = OA.t_pre_from_trace(float(reset_time), times)
    obs = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float64)
    if obs.shape != (len(rows), expected_width) or not np.all(np.isfinite(obs)):
        raise DatasetError(f"obs rows malformed in {job_dir}")
    raw = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float64)
    means = [r.get("policy_action_mean") for r in rows]
    if all(m is None for m in means):
        b_mean = raw.copy()  # deterministic trace: raw action == mean
        stochastic = False
    elif all(isinstance(m, list) for m in means):
        b_mean = np.asarray(means, dtype=np.float64)
        stochastic = True
    else:
        raise DatasetError("mixed policy_action_mean presence in trace")
    if b_mean.shape != (len(rows), F1.ACTION_DIM):
        raise DatasetError("action rows malformed")
    return {
        "job_dir": C.rel(job_dir),
        "trace_sha256": C.sha256_file(trace_path),
        "summary_sha256": C.sha256_file(summary_path),
        "reset_sha256": C.sha256_file(reset_path),
        "steps": len(rows),
        "seed": int(summary.get("action_seed")),
        "action_selection": str(summary.get("action_selection")),
        "stochastic": stochastic,
        "episode_start_offset_s": float(summary.get("episode_start_offset_s")),
        "end_reason": summary.get("end_reason"),
        "reset_time": float(reset_time),
        "t_pre": np.asarray(t_pre, dtype=np.float64),
        "obs35": obs,
        "b_mean": b_mean,
        "b_raw_action": raw,
    }


def targets_from_adapter_trace(adapter_trace_path: Path) -> dict[str, Any]:
    path = Path(adapter_trace_path)
    if path.is_symlink() or not path.is_file():
        raise DatasetError(f"adapter trace missing: {path}")
    rows = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise DatasetError(f"empty adapter trace: {path}")
    t_pre = np.asarray([float(r["t_pre"]) for r in rows], dtype=np.float64)
    targets = np.asarray([r["targets"] for r in rows], dtype=np.float64)
    if targets.shape != (len(rows), 4) or not np.all(np.isfinite(targets)):
        raise DatasetError("adapter trace targets malformed")
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "t_pre": t_pre, "targets": targets}


def targets_for_trajectory(t_pre: np.ndarray, *, adapter: dict[str, Any] | None, targets_fn: Callable[[float], np.ndarray] | None) -> tuple[np.ndarray, dict[str, Any]]:
    """Targets at the trajectory's pre-step times: exact t_pre match against the
    adapter trace where available, else the injected reconstructor."""
    n = int(t_pre.shape[0])
    out = np.empty((n, 4), dtype=np.float64)
    from_adapter = 0
    from_fn = 0
    if adapter is not None:
        m = min(n, int(adapter["t_pre"].shape[0]))
        if not np.array_equal(adapter["t_pre"][:m], t_pre[:m]):
            raise DatasetError("A_iso adapter trace t_pre grid differs from the B trajectory grid (same start expected)")
        out[:m] = adapter["targets"][:m]
        from_adapter = m
    if from_adapter < n:
        if targets_fn is None:
            raise DatasetError(f"{n - from_adapter} steps without targets: adapter trace shorter than the B trajectory and no targets_fn")
        for i in range(from_adapter, n):
            out[i] = np.asarray(targets_fn(float(t_pre[i])), dtype=np.float64).reshape(4)
            from_fn += 1
    if adapter is not None and targets_fn is not None and from_adapter:
        check = np.stack([np.asarray(targets_fn(float(t)), dtype=np.float64).reshape(4) for t in t_pre[:from_adapter]])
        if not np.array_equal(check, out[:from_adapter]):
            raise DatasetError("targets_fn disagrees with the adapter trace at identical t_pre")
    return out, {"from_adapter_trace": from_adapter, "from_targets_fn": from_fn}


def build_dataset(trajectories: Sequence[dict[str, Any]], *, teacher_arrays: dict[str, np.ndarray], spec: OA.InsertionSpec, adapter_traces: dict[str, dict[str, Any]] | None = None, targets_fn: Callable[[float], np.ndarray] | None = None, train_seeds: Sequence[int] = TRAIN_SEEDS, val_seeds: Sequence[int] = VAL_SEEDS, include_det: bool = False) -> dict[str, Any]:
    """Assemble arrays + receipt.  ``adapter_traces`` maps the start key
    (``episode_start_offset_s`` repr) to an adapter-trace record."""
    for s in (*train_seeds, *val_seeds):
        F1.assert_development_seed(int(s))
    if set(train_seeds) & set(val_seeds):
        raise DatasetError("train and validation seeds overlap")
    arrays: dict[str, list[np.ndarray]] = {k: [] for k in ("obs35", "obs39", "teacher_mean", "teacher_logstd", "b_mean", "b_raw_action", "t_pre", "traj_index", "step_index", "split")}
    per_traj: list[dict[str, Any]] = []
    for ti, traj in enumerate(trajectories):
        if not traj["stochastic"] and not include_det:
            per_traj.append({"job_dir": traj["job_dir"], "excluded": "deterministic trajectory (include_det=false)"})
            continue
        seed = int(traj["seed"])
        F1.assert_development_seed(seed)
        if seed in train_seeds:
            split = SPLIT_TRAIN
        elif seed in val_seeds:
            split = SPLIT_VAL
        else:
            raise DatasetError(f"seed {seed} is neither train nor validation")
        key = repr(float(traj["episode_start_offset_s"]))
        adapter = (adapter_traces or {}).get(key)
        targets, src = targets_for_trajectory(traj["t_pre"], adapter=adapter, targets_fn=targets_fn)
        obs35_f32 = traj["obs35"].astype(np.float32)
        obs39_f32 = OA.insert_targets(obs35_f32, targets.astype(np.float32), spec)
        OA.assert_projection_exact(obs39_f32, obs35_f32, spec)
        logits = actor_logits_numpy(teacher_arrays, obs39_f32.astype(np.float64))
        if not np.all(np.isfinite(logits)):
            raise DatasetError("non-finite teacher logits")
        n = obs35_f32.shape[0]
        arrays["obs35"].append(obs35_f32)
        arrays["obs39"].append(obs39_f32)
        arrays["teacher_mean"].append(logits[:, :F1.ACTION_DIM].astype(np.float32))
        arrays["teacher_logstd"].append(logits[:, F1.ACTION_DIM:].astype(np.float32))
        arrays["b_mean"].append(traj["b_mean"].astype(np.float32))
        arrays["b_raw_action"].append(traj["b_raw_action"].astype(np.float32))
        arrays["t_pre"].append(traj["t_pre"].astype(np.float64))
        arrays["traj_index"].append(np.full(n, ti, dtype=np.int64))
        arrays["step_index"].append(np.arange(1, n + 1, dtype=np.int64))
        arrays["split"].append(np.full(n, split, dtype=np.int64))
        err = traj["b_mean"] - logits[:, :F1.ACTION_DIM]
        per_traj.append({
            "traj_index": ti, "job_dir": traj["job_dir"], "seed": seed, "split": "train" if split == SPLIT_TRAIN else "val",
            "episode_start_offset_s": float(traj["episode_start_offset_s"]), "steps": n, "end_reason": traj["end_reason"],
            "targets_source": src, "adapter_trace": adapter["path"] if adapter else None,
            "trace_sha256": traj["trace_sha256"], "summary_sha256": traj["summary_sha256"], "reset_sha256": traj["reset_sha256"],
            "b_mean_vs_teacher_rmse": float(np.sqrt(np.mean(err ** 2))),
        })
    if not arrays["obs35"]:
        raise DatasetError("no trajectory selected")
    data = {k: np.concatenate(v, axis=0) for k, v in arrays.items()}
    n_train = int(np.sum(data["split"] == SPLIT_TRAIN))
    n_val = int(np.sum(data["split"] == SPLIT_VAL))
    if n_train == 0 or n_val == 0:
        raise DatasetError(f"degenerate split: train {n_train}, val {n_val}")
    train_traj = sorted({int(t) for t, s in zip(data["traj_index"], data["split"]) if s == SPLIT_TRAIN})
    val_traj = sorted({int(t) for t, s in zip(data["traj_index"], data["split"]) if s == SPLIT_VAL})
    if set(train_traj) & set(val_traj):
        raise DatasetError("trajectory appears in both splits")
    receipt = {
        "schema_version": 1,
        "tool": "f1_dataset",
        "rows": int(data["obs35"].shape[0]),
        "rows_train": n_train,
        "rows_val": n_val,
        "trajectories_train": train_traj,
        "trajectories_val": val_traj,
        "train_seeds": [int(s) for s in train_seeds],
        "val_seeds": [int(s) for s in val_seeds],
        "split_rule": "per trajectory by stochastic seed; deterministic trajectories excluded" if not include_det else "per trajectory by seed; deterministic included as train",
        "include_det": bool(include_det),
        "insertion": spec.to_dict(),
        "teacher": {"forward": "numpy float64 tanh MLP on float32-cast obs39", "width": int(teacher_arrays["pi.0.0.weight"].shape[1])},
        "per_trajectory": per_traj,
        "arrays_sha256": {k: sha256_array(v) for k, v in data.items()},
        "array_shapes": {k: list(v.shape) for k, v in data.items()},
        "baseline_b_vs_teacher_rmse": {
            "train": float(np.sqrt(np.mean((data["b_mean"][data["split"] == SPLIT_TRAIN] - data["teacher_mean"][data["split"] == SPLIT_TRAIN]) ** 2))),
            "val": float(np.sqrt(np.mean((data["b_mean"][data["split"] == SPLIT_VAL] - data["teacher_mean"][data["split"] == SPLIT_VAL]) ** 2))),
        },
    }
    return {"arrays": data, "receipt": receipt}


def write_dataset(out_dir: Path, stamp: str, built: dict[str, Any], *, extra_receipt: dict[str, Any] | None = None) -> dict[str, str]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    npz = out_dir / f"f1_dataset_{stamp}.npz"
    rec = out_dir / f"f1_dataset_{stamp}.json"
    if npz.exists() or rec.exists():
        raise FileExistsError(f"refusing to overwrite dataset {stamp}")
    np.savez(npz, **built["arrays"])
    receipt = {**built["receipt"], **(extra_receipt or {}), "npz": C.rel(npz), "npz_sha256": C.sha256_file(npz), "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    C.write_json(rec, receipt)
    return {"npz": str(npz), "receipt": str(rec), "npz_sha256": receipt["npz_sha256"], "receipt_sha256": C.sha256_file(rec)}


def load_dataset(npz_path: Path, receipt_path: Path) -> dict[str, Any]:
    receipt = json.loads(Path(receipt_path).read_text(encoding="utf-8"))
    if C.sha256_file(npz_path) != receipt.get("npz_sha256"):
        raise DatasetError("dataset npz digest != receipt")
    with np.load(npz_path) as z:
        data = {k: z[k] for k in z.files}
    for k, sha in receipt["arrays_sha256"].items():
        if sha256_array(data[k]) != sha:
            raise DatasetError(f"array {k} digest mismatch")
    return {"arrays": data, "receipt": receipt}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build the D dataset from frozen B trajectories (stage 2).")
    parser.add_argument("--b-job-dir", action="append", required=True, help="B stochastic job directory (repeatable)")
    parser.add_argument("--aiso-adapter-trace", action="append", default=[], help="f1_adapter_trace.json of an A_iso job (repeatable, one per start)")
    parser.add_argument("--teacher-module", required=True, help="V26 39D rl_module directory")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--include-det", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="validate inputs and print the plan; write nothing")
    args = parser.parse_args(argv)
    if str(F1.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(F1.BASELINE_DIR))
    import warm_start as W  # production, import only

    state = W.load_module_state(Path(args.teacher_module))
    teacher = load_actor_arrays(state, expected_width=F1.MODULE_WIDTH_39)
    spec = OA.derive_insertion(*[OA.read_manifest_names(p, expected_sha256=s, sha256_fn=C.sha256_file)[0] for p, s in ((C.ACTOR_MANIFEST_35, C.ACTOR_MANIFEST_35_SHA256), (C.ACTOR_MANIFEST_39, C.ACTOR_MANIFEST_39_SHA256))], manifest35_sha256=C.ACTOR_MANIFEST_35_SHA256, manifest39_sha256=C.ACTOR_MANIFEST_39_SHA256)
    trajs = [trajectory_from_job(Path(p)) for p in args.b_job_dir]
    adapters: dict[str, dict[str, Any]] = {}
    for p in args.aiso_adapter_trace:
        rec = targets_from_adapter_trace(Path(p))
        summary = json.loads((Path(p).parent / SUMMARY_FILE).read_text(encoding="utf-8"))
        adapters[repr(float(summary["episode_start_offset_s"]))] = rec
    if args.dry_run:
        print(json.dumps({"mode": "dry_run", "trajectories": [{k: t[k] for k in ("job_dir", "steps", "seed", "stochastic", "episode_start_offset_s")} for t in trajs], "adapter_traces": {k: v["path"] for k, v in adapters.items()}, "teacher_actor_digest": W.actor_state_digest(state)}, indent=2))
        return 0
    built = build_dataset(trajs, teacher_arrays=teacher, spec=spec, adapter_traces=adapters, include_det=args.include_det)
    written = write_dataset(Path(args.out_dir), args.stamp, built, extra_receipt={"teacher_module": C.rel(Path(args.teacher_module)), "teacher_actor_digest": W.actor_state_digest(state), "tool_sha256": C.sha256_file(Path(__file__).resolve())})
    print(json.dumps(written, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
