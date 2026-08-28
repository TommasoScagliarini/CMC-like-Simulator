"""Derive a constant-sigma module (C from B, A_ISO39_V3_S005 from V26) — F1 stage 1.

Deterministic checkpoint surgery, no fit: the log-std rows of ``pi.1.weight``
are zeroed and ``pi.1.bias[action_dim:]`` is set to ``ln(sigma)`` in the
stored dtype (float32), exactly the July recipe
(``configure_actor_exploration.configure_constant_std``, reused by import).
The action mean and every other tensor stay bit-identical, so the
deterministic rollout of the derived module must be bit-identical to the
source (G1: C-det == B-det).

Default is **dry-run** (nothing written): the tool reports source digests,
the planned output and the post-transform invariants computed in memory.
``--materialize`` writes ``<output_dir>/rl_module`` by copying the source
module directory (``class_and_ctor_args.pkl``, ``metadata.json`` carry the
architecture) and overwriting ``module_state.pkl``; then reloads and
verifies exactness, writes ``actor_feature_manifest.json`` and
``f1_sigma_variant_receipt.json`` (no-clobber).
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
BASELINE_DIR = HERE.parents[1]
for entry in (str(HERE), str(BASELINE_DIR)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import configure_actor_exploration as CAE  # noqa: E402  (production, import only)
import warm_start as W  # noqa: E402  (production, import only)

MODULE_FILES = ("module_state.pkl", "metadata.json", "class_and_ctor_args.pkl")
LOGSTD_ROW_START = F1.ACTION_DIM
ENTROPY_EXPECTED = -7.758758
ENTROPY_TOL = 1e-5


class SigmaVariantError(RuntimeError):
    pass


def _as_np(value: Any) -> np.ndarray:
    return np.asarray(W._as_numpy(value))


def logstd_head_report(state: dict[str, Any], *, sigma: float, action_dim: int = F1.ACTION_DIM) -> dict[str, Any]:
    weight = _as_np(state["pi.1.weight"])
    bias = _as_np(state["pi.1.bias"])
    rows = weight[action_dim:]
    logstd_bias = bias[action_dim:]
    expected_bias = np.asarray(np.log(np.full(action_dim, float(sigma))), dtype=bias.dtype)
    entropy = F1.entropy_diag_gauss([float(v) for v in logstd_bias.astype(np.float64)])
    return {
        "dtype": str(bias.dtype),
        "logstd_weight_rows_all_zero": bool(np.all(rows == 0.0)),
        "logstd_weight_rows_l2": float(np.linalg.norm(rows.astype(np.float64))),
        "logstd_bias": logstd_bias.astype(float).tolist(),
        "logstd_bias_expected": expected_bias.astype(float).tolist(),
        "logstd_bias_exact": bool(np.array_equal(logstd_bias, expected_bias)),
        "sigma_effective": np.exp(logstd_bias.astype(np.float64)).tolist(),
        "entropy_nat": entropy,
        "entropy_expected_nat": ENTROPY_EXPECTED,
        "entropy_within_tolerance": bool(abs(entropy - ENTROPY_EXPECTED) <= ENTROPY_TOL),
    }


def constant_sigma_invariants(state: dict[str, Any], *, sigma: float) -> tuple[bool, dict[str, Any]]:
    rep = logstd_head_report(state, sigma=sigma)
    ok = rep["logstd_weight_rows_all_zero"] and rep["logstd_bias_exact"] and rep["entropy_within_tolerance"]
    return bool(ok), rep


def module_files_table(module_dir: Path) -> dict[str, str]:
    table: dict[str, str] = {}
    for name in MODULE_FILES:
        path = module_dir / name
        if path.is_symlink() or not path.is_file():
            raise SigmaVariantError(f"module file missing or symlink: {path}")
        table[name] = C.sha256_file(path)
    return table


def derive_constant_sigma_module(source_module: Path, output_dir: Path, *, sigma: float = F1.SIGMA_CONSTANT, materialize: bool = False, expected_source_actor_digest: str | None = None, manifest_sha256: str | None = None) -> dict[str, Any]:
    source_module = Path(source_module)
    output_dir = Path(output_dir)
    out_module = output_dir / "rl_module"
    source_files = module_files_table(source_module)
    state = W.load_module_state(source_module)
    source_digest = W.actor_state_digest(state)
    if expected_source_actor_digest is not None and source_digest != expected_source_actor_digest:
        raise SigmaVariantError(f"source actor digest {source_digest} != expected {expected_source_actor_digest}")
    configured, cae_report = CAE.configure_constant_std(state, sigma=sigma, action_dim=F1.ACTION_DIM)
    ok, head = constant_sigma_invariants(configured, sigma=sigma)
    if not ok:
        raise SigmaVariantError(f"constant-sigma invariants failed in memory: {head}")
    mean_compare = {
        key: bool(np.array_equal(_as_np(state[key]), _as_np(configured[key])))
        for key in state
        if key not in ("pi.1.weight", "pi.1.bias")
    }
    mean_rows_exact = bool(
        np.array_equal(_as_np(state["pi.1.weight"])[:F1.ACTION_DIM], _as_np(configured["pi.1.weight"])[:F1.ACTION_DIM])
        and np.array_equal(_as_np(state["pi.1.bias"])[:F1.ACTION_DIM], _as_np(configured["pi.1.bias"])[:F1.ACTION_DIM])
    )
    if not (all(mean_compare.values()) and mean_rows_exact):
        raise SigmaVariantError("mean parameters changed by the sigma transform")
    width = int(_as_np(state["pi.0.0.weight"]).shape[1])
    derived_digest = W.actor_state_digest(configured)
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "f1_sigma_variant",
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "mode": "materialize" if materialize else "dry_run",
        "source_module": C.rel(source_module),
        "source_module_files_sha256": source_files,
        "source_actor_digest": source_digest,
        "source_head": logstd_head_report(state, sigma=sigma),
        "sigma": float(sigma),
        "action_dim": F1.ACTION_DIM,
        "actor_input_width": width,
        "configure_constant_std_report": cae_report,
        "derived_head": head,
        "derived_actor_digest": derived_digest,
        "mean_parameters_bit_exact": True,
        "output_module": C.rel(out_module),
        "materialized": False,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
    }
    if not materialize:
        return report
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing derived module dir: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source_module, out_module, copy_function=shutil.copy2)
    with (out_module / "module_state.pkl").open("wb") as handle:
        pickle.dump(configured, handle, protocol=pickle.HIGHEST_PROTOCOL)
    reloaded = W.load_module_state(out_module)
    compare = W.compare_actor_states(configured, reloaded)
    if not compare["exact"]:
        raise SigmaVariantError(f"save/reload mismatch: {compare}")
    ok2, head2 = constant_sigma_invariants(reloaded, sigma=sigma)
    if not ok2:
        raise SigmaVariantError(f"constant-sigma invariants failed after reload: {head2}")
    manifest_src = source_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    names: list[str] | None = None
    if manifest_src.is_file():
        names = json.loads(manifest_src.read_text(encoding="utf-8")).get("actor_feature_names")
    manifest = {
        "schema_version": 1,
        "actor_feature_names": names,
        "actor_feature_count": width,
        "actor_digest": derived_digest,
        "module_state_sha256": C.sha256_file(out_module / "module_state.pkl"),
        "exploration_sigma": [float(sigma)] * F1.ACTION_DIM,
        "exploration_log_std": head2["logstd_bias"],
        "derived_from": C.rel(source_module),
        "source_actor_digest": source_digest,
        "contract": "constant_sigma_head_F1",
    }
    C.write_json(out_module / W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=manifest_src.is_file())
    report.update({
        "materialized": True,
        "derived_module_files_sha256": module_files_table(out_module),
        "save_reload_exact": True,
        "derived_head_after_reload": head2,
        "manifest": manifest,
    })
    C.write_json(output_dir / "f1_sigma_variant_receipt.json", report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--source-module", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sigma", type=float, default=F1.SIGMA_CONSTANT)
    parser.add_argument("--expected-source-actor-digest", default=None)
    parser.add_argument("--materialize", action="store_true", help="write the derived module (default: dry-run)")
    args = parser.parse_args(argv)
    report = derive_constant_sigma_module(Path(args.source_module), Path(args.output_dir), sigma=args.sigma, materialize=args.materialize, expected_source_actor_digest=args.expected_source_actor_digest)
    print(json.dumps({k: report[k] for k in ("mode", "source_module", "source_actor_digest", "derived_actor_digest", "derived_head", "output_module", "materialized")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
