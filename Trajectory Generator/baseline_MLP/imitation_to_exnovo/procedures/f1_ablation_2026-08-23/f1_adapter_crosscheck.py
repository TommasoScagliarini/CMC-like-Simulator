"""G1 cross-check (stage 1, before any A_iso rollout): the adapter's
reconstructed prescribed targets must reproduce, with no tolerance, what the
env itself recorded in the F0 ctrl39 traces (V26 under its native runtime):

* cast to the env **observation dtype** (read from the env, not hard-coded:
  ``observation_space.dtype``), the reconstruction must equal
  ``obs39[:, 2:6]`` bit-for-bit at the identical pre-step times — the bits the
  actor actually receives (the adapter casts to the dtype of the observation it
  is given, exactly as the env casts its own observation);
* in float64, the reconstruction at the post-step times must equal the env's
  own ``imitation_target_q/qdot`` recorded in the same trace bit-for-bit —
  proves the reconstruction reproduces the env computation.

The env is built under B's v3 runtime through the production path
(``rollout_eval.run`` with ``--max-steps 0``: model load + reset, **no
simulation step**); the adapter hooks bind the reconstructor; the comparison
is pure numpy.  Output (no-clobber, stamped): ``metrics/f1_adapter_crosscheck_<stamp>.json``
(schema 2) consumed by ``f1_analysis`` (check ``adapter_vs_f0_ctrl39``).
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402

PAYLOAD_SCHEMA_VERSION = 2
OUTPUT_PREFIX = "f1_adapter_crosscheck_"


def ctrl39_trace_dirs() -> dict[str, Path]:
    out = {}
    for s in F1.STARTS:
        d = F1.F0_ROLLOUTS / "ctrl39" / f"V26_39D__v26_imitation_native__{s}__det"
        if (d / "rollout_policy_trace.json").is_file() and (d / "rollout_reset_diagnostics.json").is_file():
            out[s] = d
    return out


def post_step_targets_from_rows(rows: list[dict[str, Any]]) -> np.ndarray:
    """The env's own float64 targets recorded after each step (same coordinate
    order as the actor block: knee q, knee qdot, ankle q, ankle qdot)."""
    out = []
    for r in rows:
        q, qd = r["imitation_target_q"], r["imitation_target_qdot"]
        out.append([q["pros_knee_angle"], qd["pros_knee_angle"], q["pros_ankle_angle"], qd["pros_ankle_angle"]])
    return np.asarray(out, dtype=np.float64)


def run_crosscheck(start: str, *, out_dir: Path, stamp: str) -> dict[str, Any]:
    import f1_rollout_aiso as RA

    spec = RA.load_insertion_spec()
    tmp = Path(tempfile.mkdtemp(prefix="f1_crosscheck_"))
    argv = [
        "--checkpoint", str(F1.CANDIDATES["A_ISO39_V3"]["module"]), "--no-auto-config", "--config", str(F1.RUNTIME_CONFIG),
        "--episode-start-offset-s", repr(float(F1.EXACT_STARTS[start])), "--action-selection", "deterministic", "--seed", "123",
        "--output-dir", str(tmp), "--no-record-outputs", "--no-record-policy-trace", "--no-progress", "--max-steps", "0", "--worker-process",
        *list(C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]),
    ]
    sys.argv = [sys.argv[0], *argv]
    import rollout_eval as RE

    args = RE.parse_args()
    holder: dict[str, Any] = {}
    RA.install_hooks(RE, "aiso4", spec, holder, env_width=F1.ENV_ACTOR_WIDTH, env_full_width=F1.FULL_OBS_WIDTH_35)
    summary = RE.run(args)
    if int(summary.get("steps", -1)) != 0:
        raise RuntimeError("cross-check must not step the simulation")
    env = holder["env"]
    obs_dtype = np.dtype(env.observation_space.dtype)  # the env contract, read from the env
    module = holder["module"]
    recon = module._adapter.reconstructor
    results: dict[str, Any] = {}
    for s, d in ctrl39_trace_dirs().items():
        rows = json.loads((d / "rollout_policy_trace.json").read_text(encoding="utf-8"))
        reset = json.loads((d / "rollout_reset_diagnostics.json").read_text(encoding="utf-8"))
        times = [float(r["time"]) for r in rows]
        t_pre = OA.t_pre_from_trace(float(reset["time"]), times)
        obs39 = [r["actor_observation_vector_before"] for r in rows]
        res = OA.crosscheck_targets_against_trace(recon, spec, obs39, t_pre, obs_dtype=obs_dtype, post_step_targets=post_step_targets_from_rows(rows), post_step_times=times)
        results[s] = {**res, "f0_job_dir": C.rel(d), "trace_sha256": C.sha256_file(d / "rollout_policy_trace.json")}
    payload = {
        "schema_version": PAYLOAD_SCHEMA_VERSION,
        "stamp": stamp,
        "built_under_start": start,
        "env_build": {"steps": int(summary["steps"]), "n_actor": int(summary["n_actor"]), "n_observation": int(summary["n_observation"]), "gait_clock_enable": bool(summary["gait_clock_enable"]), "checkpoint": C.rel(Path(summary["checkpoint"])), "config": C.rel(F1.RUNTIME_CONFIG), "config_sha256": C.sha256_file(F1.RUNTIME_CONFIG)},
        "observation_dtype": str(obs_dtype),
        "dtype_rule": "reconstruction cast to the env observation dtype (observation_space.dtype) == recorded obs39[:,2:6] bit-for-bit; float64 reconstruction == env imitation_target_q/qdot at post-step times bit-for-bit; no tolerance",
        "projection_contract": spec.to_dict(),
        "reconstructor": recon.provenance,
        "results": results,
        "all_exact": bool(results) and all(v["exact"] for v in results.values()),
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    C.write_json(out_dir / f"{OUTPUT_PREFIX}{stamp}.json", payload)
    return payload


if __name__ == "__main__":
    F1.ensure_out_dirs()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    payload = run_crosscheck("nominal", out_dir=F1.OUT_METRICS, stamp=stamp)
    print(json.dumps({"stamp": stamp, "observation_dtype": payload["observation_dtype"], "all_exact": payload["all_exact"], **{s: {k: v[k] for k in ("exact", "exact_runtime_dtype", "exact_float64_post_step", "exact_cells_runtime_dtype", "cells", "max_abs_diff_runtime_dtype", "max_abs_diff_float64_post_step", "max_abs_diff_float64_vs_recorded_informational")} for s, v in payload["results"].items()}}, indent=2))
