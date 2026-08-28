"""V26B anchor collection — V0 tooling (planner, preflight, dry-run, fail-closed collector).

Authorised scope (architect, 2026-08-24, rev3): tooling + synthetic tests +
no-clobber dry-run ONLY.  Nothing here runs a rollout, fits a student or
evaluates a gate unless the collection CLI is invoked with the explicit
architect stage token (``--collect --authorized-stage V26B-ANCHORS``), which
this revision documents but does NOT execute.

Frozen decisions implemented here (protocol ``V26B-bridge-rev3``):

* coverage target MANDATORY, no waiver: >= 16000 bitwise-unique rows and
  >= 3 complete 500-row traces per sigma, otherwise ``CoverageStop`` (architect
  decision; NO seed is added automatically);
* targets ALWAYS the deterministic MEAN action of the August V26 actor computed
  offline on the visited obs39 (A_iso6clk route), never the recorded stochastic
  action; the offline mean must match the recorded ``policy_action_mean``
  within 1e-5 on every included trace (functional teacher/runtime equality);
* collection behaviour = constant-sigma 39D modules derived from V26 (mean
  bit-identical) on the FIXED grid {0.0025, 0.005, 0.01}; the grid explores
  state coverage only and does NOT determine the operational sigma (V4);
* anchor-only seeds {1000, 1001, 1002, 1003}; seeds 123-128 keep their frozen
  roles and are refused here; anchor seeds are refused in any DAgger/gate/
  holdout/sealed context;
* F0/F1/F2R libraries are imported READ-ONLY and never modified.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
F2R_DIR = VALIDATION_DIR / "f2r_bridge_2026-08-23"
F1_DIR = VALIDATION_DIR / "f1_ablation_2026-08-23"
F0_DIR = VALIDATION_DIR / "f0_freeze_2026-08-22"
for _entry in (str(F0_DIR), str(F1_DIR), str(F2R_DIR), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402  (immutable F0 library)
import f1_common as F1  # noqa: E402  (immutable F1 library)
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402
import f2r_common as R  # noqa: E402  (immutable F2R library)
import f2r_labeller as L  # noqa: E402


class V26BContractError(RuntimeError):
    """Fail-closed contract violation in the V26B anchor tooling."""


class CoverageStop(V26BContractError):
    """Coverage below the mandatory target: STOP, architect decision required.

    Carries the full coverage ``report``; NO seed is added automatically."""

    def __init__(self, message: str, report: Mapping[str, Any]) -> None:
        super().__init__(message)
        self.report = dict(report)


# --- protocol binding -----------------------------------------------------------------------------

PROTOCOL_JSON = HERE / "v26b_protocol.json"
PROTOCOL_MD = HERE / "PROTOCOL_V26B.md"
PROTOCOL_ID = "V26B-bridge-rev3"
AUTHORIZED_STAGE = "V26B-ANCHORS"

V26B_TAG = "2026-08-24_V26B_anchors_r1"
OUT_ROOT = C.RUNS_ROLLOUT / "validation" / "v26b_bridge_runs" / V26B_TAG
OUT_MANIFEST = OUT_ROOT / "manifest"
OUT_DERIVED = OUT_ROOT / "derived_modules"
OUT_ROLLOUTS = OUT_ROOT / "rollouts" / "anchors_stoch"
OUT_LOGS = OUT_ROOT / "logs"
OUT_COVERAGE = OUT_ROOT / "coverage"

# --- frozen collection constants (must equal the rev3 protocol; verified by load_protocol) --------

ANCHOR_SEEDS: tuple[int, ...] = (1000, 1001, 1002, 1003)
SIGMA_GRID: tuple[float, ...] = (0.0025, 0.005, 0.01)
_SIGMA_TAGS: tuple[tuple[float, str], ...] = ((0.0025, "s00025"), (0.005, "s0005"), (0.01, "s001"))
ROWS_PER_COMPLETE_TRACE = 500
MIN_UNIQUE_ROWS = 16000
MIN_COMPLETE_TRACES_PER_SIGMA = 3
EXPECTED_STOCH_TRACES = len(SIGMA_GRID) * len(R.STARTS) * len(ANCHOR_SEEDS)  # 36
MEAN_MATCH_TOL = 1e-5
COMPLETE_END_REASON = "episode_time_limit"
RECEIPT_FILE = "v26b_receipt.json"
RECEIPT_SCHEMA = "v26b.1"
TRACE_OUTPUT_FILES = (
    "rollout_policy_trace.json",
    "rollout_summary.json",
    "rollout_reset_diagnostics.json",
    "f1_adapter_trace.json",
    "f1_adapter_summary.json",
)

RESERVED_SEED_ROLES: dict[int, str] = {
    123: "deterministic training-side seed (F2R DAgger det / gates)",
    124: "stochastic collection seed (F2R DAgger)",
    125: "held-out promotion gate (never in any dataset/label/fit)",
    126: "sealed test (F3 only)",
    127: "sealed test (F3 only)",
    128: "sealed test (F3 only)",
}
_ANCHOR_CONTEXTS = ("anchor_collection_stoch",)
_PROTECTED_CONTEXTS = ("dagger", "gate", "holdout", "sealed")


def sigma_tag(sigma: float) -> str:
    for value, tag in _SIGMA_TAGS:
        if float(sigma) == value:
            return tag
    raise V26BContractError(f"sigma {sigma!r} is not on the frozen grid {SIGMA_GRID}")


def load_protocol() -> dict[str, Any]:
    """Parse the rev3 protocol and verify the tooling <-> protocol binding fail-closed."""
    if PROTOCOL_JSON.is_symlink() or not PROTOCOL_JSON.is_file():
        raise V26BContractError(f"protocol missing (or symlink): {PROTOCOL_JSON}")
    proto = json.loads(PROTOCOL_JSON.read_text(encoding="utf-8"))
    if proto.get("protocol_id") != PROTOCOL_ID:
        raise V26BContractError(f"protocol_id {proto.get('protocol_id')!r} != {PROTOCOL_ID!r}: this tooling only runs under the rev3 protocol")
    anchors = proto.get("anchors")
    if not isinstance(anchors, Mapping):
        raise V26BContractError("protocol without an 'anchors' section")
    cov = anchors.get("coverage_target_mandatory_no_waiver")
    if not isinstance(cov, Mapping) or int(cov.get("min_bitwise_unique_rows", -1)) != MIN_UNIQUE_ROWS or int(cov.get("min_complete_500row_traces_per_sigma", -1)) != MIN_COMPLETE_TRACES_PER_SIGMA:
        raise V26BContractError("protocol coverage target differs from the frozen tooling constants (16000 unique rows / 3 complete traces per sigma)")
    seeds = anchors.get("anchor_only_seeds_preregistered", {})
    if tuple(int(s) for s in (seeds.get("seeds") or [])) != ANCHOR_SEEDS:
        raise V26BContractError(f"protocol anchor seeds {seeds.get('seeds')!r} != frozen {ANCHOR_SEEDS}")
    grid = tuple(float(v) for v in (anchors.get("collection_behaviour", {}).get("sigma_grid_fixed") or []))
    if grid != SIGMA_GRID:
        raise V26BContractError(f"protocol sigma grid {grid!r} != frozen {SIGMA_GRID}")
    return proto


# --- seed governance ------------------------------------------------------------------------------

def assert_anchor_seed(seed: int, *, context: str) -> int:
    """Anchor-only seed governance (complements, never replaces, the F2R rules).

    ``anchor_collection_stoch``: only {1000, 1001, 1002, 1003} are accepted; the
    frozen roles of 123-128 are spelled out in the refusal.  ``dagger``/``gate``/
    ``holdout``/``sealed``: anchor seeds are refused (F2R governs the rest)."""
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
        raise V26BContractError(f"seed must be an int, got {seed!r}")
    seed = int(seed)
    context = str(context)
    if context in _ANCHOR_CONTEXTS:
        if seed in RESERVED_SEED_ROLES:
            raise V26BContractError(f"seed {seed} is reserved ({RESERVED_SEED_ROLES[seed]}) and never collects anchors; anchor-only seeds are {ANCHOR_SEEDS}")
        if seed not in ANCHOR_SEEDS:
            raise V26BContractError(f"seed {seed} is not a preregistered anchor seed {ANCHOR_SEEDS}; adding seeds requires an architect decision (never automatic)")
        return seed
    if context in _PROTECTED_CONTEXTS:
        if seed in ANCHOR_SEEDS:
            raise V26BContractError(f"anchor-only seed {seed} must never enter a {context!r} context (anchor seeds are collection-only)")
        return seed
    raise V26BContractError(f"unknown seed context {context!r}: expected one of {_ANCHOR_CONTEXTS + _PROTECTED_CONTEXTS}")


# --- planner --------------------------------------------------------------------------------------

def sigma_module_plan() -> list[dict[str, Any]]:
    """The 3 constant-sigma 39D modules derived from the pinned V26 teacher (mean bit-identical)."""
    plan = []
    for sigma in SIGMA_GRID:
        tag = sigma_tag(sigma)
        name = f"V26_39D_{tag}"
        out_dir = OUT_DERIVED / name
        plan.append({
            "name": name,
            "sigma": float(sigma),
            "tag": tag,
            "out_dir": out_dir,
            "module": out_dir / "rl_module",
            "source_module": str(R.TEACHER["module"]),
            "expected_source_actor_digest": R.TEACHER["actor_digest"],
            "expected_source_module_state_sha256": R.TEACHER["module_state_sha256"],
            "derivation": "sigma 0.005: f1_sigma_variant.derive_constant_sigma_module (unmodified F1 tool); sigma 0.0025/0.01: v26b faithful mirror (same production primitives; entropy expectation computed from sigma -- the F1 constant is 0.005-specific); mean parameters bit-identical to V26 in every case",
        })
    return plan


def job_specs() -> list[dict[str, Any]]:
    """The 36 stochastic anchor jobs: sigma (3) x start (3) x anchor seed (4), A_iso6clk route."""
    modules = {p["tag"]: p for p in sigma_module_plan()}
    specs: list[dict[str, Any]] = []
    for sigma in SIGMA_GRID:
        tag = sigma_tag(sigma)
        for start in R.STARTS:
            for seed in ANCHOR_SEEDS:
                assert_anchor_seed(seed, context="anchor_collection_stoch")
                job_id = f"A26_{tag}__{start}__seed{seed}"
                specs.append({
                    "job_id": job_id,
                    "sigma": float(sigma),
                    "sigma_tag": tag,
                    "start": str(start),
                    "start_offset_s": float(R.EXACT_STARTS[start]),
                    "seed": int(seed),
                    "candidate": modules[tag]["name"],
                    "module": modules[tag]["module"],
                    "driver": "f1_rollout_aiso",
                    "adapter_mode": "aiso6clk",
                    "action_selection": "stochastic",
                    "runtime": F1.TARGET_RUNTIME,
                    "dir": OUT_ROLLOUTS / f"{modules[tag]['name']}__{F1.TARGET_RUNTIME}__{start}__stoch_seed{seed}",
                    "purpose": "anchor_stoch",
                    "enters_dataset": True,
                    "role": "V26 preservation anchor (target = deterministic V26 mean on the visited obs39, never the sampled action)",
                })
    if len(specs) != EXPECTED_STOCH_TRACES:
        raise V26BContractError(f"planner produced {len(specs)} jobs, expected {EXPECTED_STOCH_TRACES}")
    if len({s["job_id"] for s in specs}) != len(specs) or len({str(s["dir"]) for s in specs}) != len(specs):
        raise V26BContractError("planner job ids/dirs are not unique")
    return specs


def job_command(spec: Mapping[str, Any], python_exe: str) -> list[str]:
    """EXACT F1 A_iso6clk driver command for one anchor job (same shape as the frozen F1 matrix)."""
    return [
        python_exe, str(F1.AISO_DRIVER),
        "--checkpoint", str(spec["module"]),
        "--no-auto-config",
        "--config", str(F1.RUNTIME_CONFIG),
        "--episode-start-offset-s", repr(float(spec["start_offset_s"])),
        "--action-selection", "stochastic",
        "--seed", str(int(spec["seed"])),
        "--output-dir", str(spec["dir"]),
        "--record-outputs", "--record-policy-trace",
        *[str(a) for a in F1.JOB_TIMEOUT_ARGS],
        *[str(a) for a in C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]],
        "--f1-adapter", "aiso6clk",
    ]


def driver_inputs_table() -> dict[str, Any]:
    """Content digests of everything the collection depends on (fail-closed on the pinned ones)."""
    rollout_eval_sha = C.sha256_file(Path(F1.ROLLOUT_EVAL))
    if rollout_eval_sha != R.ROLLOUT_EVAL_SHA256_PINNED:
        raise V26BContractError(f"rollout_eval.py digest {rollout_eval_sha} != pinned {R.ROLLOUT_EVAL_SHA256_PINNED}")
    config_sha = C.sha256_file(Path(F1.RUNTIME_CONFIG))
    if config_sha != F1.RUNTIME_CONFIG_SHA256:
        raise V26BContractError(f"runtime config digest {config_sha} != pinned {F1.RUNTIME_CONFIG_SHA256}")
    return {
        "rollout_eval_sha256": rollout_eval_sha,
        "runtime_config": {"path": C.rel(Path(F1.RUNTIME_CONFIG)), "sha256": config_sha},
        "aiso_driver_sha256": C.sha256_file(Path(F1.AISO_DRIVER)),
        "f1_obs_adapter_sha256": C.sha256_file(F1_DIR / "f1_obs_adapter.py"),
        "f1_sigma_variant_sha256": C.sha256_file(F1_DIR / "f1_sigma_variant.py"),
        "f1_dataset_sha256": C.sha256_file(F1_DIR / "f1_dataset.py"),
        "f2r_labeller_sha256": C.sha256_file(F2R_DIR / "f2r_labeller.py"),
        "v26b_anchors_sha256": C.sha256_file(Path(__file__).resolve()),
        "protocol_sha256": C.sha256_file(PROTOCOL_JSON),
        "manifest35_sha256": C.ACTOR_MANIFEST_35_SHA256,
        "manifest39_sha256": C.ACTOR_MANIFEST_39_SHA256,
    }


# --- teacher context ------------------------------------------------------------------------------

def teacher_context() -> dict[str, Any]:
    """Pinned V26 teacher loaded read-only (arrays + insertion spec + verified digests)."""
    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W

    module = Path(R.TEACHER["module"])
    state_sha = C.sha256_file(module / "module_state.pkl")
    if state_sha != R.TEACHER["module_state_sha256"]:
        raise V26BContractError(f"teacher module_state.pkl digest {state_sha} != pinned {R.TEACHER['module_state_sha256']}")
    state = W.load_module_state(module)
    digest = W.actor_state_digest(state)
    if digest != R.TEACHER["actor_digest"]:
        raise V26BContractError(f"teacher actor digest {digest} != pinned {R.TEACHER['actor_digest']}")
    arrays = DS.load_actor_arrays(state, expected_width=R.MODULE_WIDTH_39)
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)
    return {"arrays": arrays, "spec": spec, "digest": digest, "module_state_sha256": state_sha, "module": str(module)}


def _labeller_for_job(ctx: Mapping[str, Any], job_dir: Path, *, pins: Mapping[str, str] | None = None) -> Any:
    cache = L.PrivilegedCache.from_adapter_sidecar(Path(job_dir), pins=pins)
    return L.TeacherLabeller("T1", ctx["arrays"], ctx["spec"], cache, teacher_digest=ctx["digest"])


# --- receipt --------------------------------------------------------------------------------------

def validate_receipt(job_dir: Path, spec: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed v26b.1 receipt check: schema, binding to the spec, rc 0, pins, output digests."""
    path = Path(job_dir) / RECEIPT_FILE
    if path.is_symlink() or not path.is_file():
        raise V26BContractError(f"receipt missing (or symlink): {path}")
    receipt = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(receipt, Mapping) or receipt.get("schema") != RECEIPT_SCHEMA:
        raise V26BContractError(f"receipt schema {receipt.get('schema') if isinstance(receipt, Mapping) else type(receipt)} != {RECEIPT_SCHEMA!r}")
    for key, want in (("job_id", spec["job_id"]), ("sigma", float(spec["sigma"])), ("seed", int(spec["seed"])), ("start", str(spec["start"]))):
        if receipt.get(key) != want:
            raise V26BContractError(f"receipt {key} {receipt.get(key)!r} != spec {want!r}")
    if receipt.get("returncode") != 0:
        raise V26BContractError(f"receipt returncode {receipt.get('returncode')!r} != 0 (failed jobs never contribute rows)")
    pins = receipt.get("pins")
    if not isinstance(pins, Mapping):
        raise V26BContractError("receipt without pins")
    for key, want in (("runtime_config_sha256", F1.RUNTIME_CONFIG_SHA256), ("rollout_eval_sha256", R.ROLLOUT_EVAL_SHA256_PINNED)):
        if pins.get(key) != want:
            raise V26BContractError(f"receipt pin {key} {pins.get(key)!r} != pinned {want}")
    outputs = receipt.get("outputs")
    if not isinstance(outputs, Mapping):
        raise V26BContractError("receipt without outputs")
    for name in TRACE_OUTPUT_FILES:
        p = Path(job_dir) / name
        if p.is_symlink() or not p.is_file():
            raise V26BContractError(f"job output missing (or symlink): {p}")
        disk = C.sha256_file(p)
        if outputs.get(name) != disk:
            raise V26BContractError(f"receipt output {name} sha {outputs.get(name)!r} != disk {disk}")
    return dict(receipt)


# --- per-trace loading ----------------------------------------------------------------------------

def _fsm_last_row(job_dir: Path) -> dict[str, Any]:
    rows = json.loads((Path(job_dir) / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    fsm = rows[-1].get("phase_fsm") if isinstance(rows, list) and rows and isinstance(rows[-1], Mapping) else None
    if not isinstance(fsm, Mapping):
        raise V26BContractError(f"trace last row without phase_fsm diagnostics: {job_dir}")
    out: dict[str, Any] = {}
    for key in ("timeout_exceeded", "hs_cancelled_count", "resync_count", "invalid_event_count", "valid_cycle_count"):
        if key not in fsm:
            raise V26BContractError(f"phase_fsm without {key!r} (v3 counters are mandatory): {job_dir}")
        out[key] = fsm[key]
    return out


def _trace_rows(job_dir: Path, *, ctx: Mapping[str, Any], expected_seed: int, expected_start: str, expected_selection: str, pins: Mapping[str, str] | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """obs35/t_pre/targets(u_T mean) of one job + quality facts.  Hard errors on identity,
    digest, finiteness or teacher-mean mismatch; quality (completeness, contract) is
    reported for the collector to decide inclusion."""
    job_dir = Path(job_dir)
    try:
        traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    except DS.DatasetError as exc:
        raise V26BContractError(f"malformed job {job_dir}: {exc}") from exc
    if int(traj["seed"]) != int(expected_seed):
        raise V26BContractError(f"recorded action_seed {traj['seed']} != declared {expected_seed} in {job_dir}")
    if float(traj["episode_start_offset_s"]) != float(R.EXACT_STARTS[expected_start]):
        raise V26BContractError(f"recorded start {traj['episode_start_offset_s']!r} != exact {R.EXACT_STARTS[expected_start]!r} ({expected_start})")
    if str(traj["action_selection"]) != expected_selection:
        raise V26BContractError(f"action_selection {traj['action_selection']!r} != {expected_selection!r}")
    if expected_selection == "stochastic" and not traj["stochastic"]:
        raise V26BContractError("stochastic job without per-step policy_action_mean in the trace")
    if expected_selection == "deterministic" and traj["stochastic"]:
        raise V26BContractError("deterministic job with a stochastic trace")
    for key in ("b_raw_action", "b_mean"):
        if not np.all(np.isfinite(np.asarray(traj[key], dtype=np.float64))):
            raise V26BContractError(f"non-finite {key} in {job_dir}")
    obs35 = np.ascontiguousarray(np.asarray(traj["obs35"], dtype=np.float64).astype(np.float32))
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    lab = _labeller_for_job(ctx, job_dir, pins=pins)
    out = lab.label(obs35, t_pre)
    u_mean = np.asarray(out["actions"], dtype=np.float64)  # T1 composition == (u_T, u_T) == V26 mean
    recorded_mean = np.asarray(traj["b_mean"], dtype=np.float64)
    mean_dev = float(np.max(np.abs(u_mean - recorded_mean)))
    if mean_dev > MEAN_MATCH_TOL:
        raise V26BContractError(f"offline V26 mean deviates {mean_dev:.3e} > {MEAN_MATCH_TOL} from the recorded policy mean in {job_dir}: teacher/runtime mismatch (wrong module or wrong route)")
    fsm = _fsm_last_row(job_dir)
    steps = int(traj["steps"])
    end_reason = str(traj["end_reason"])
    contract_failure = bool(fsm["timeout_exceeded"]) or ("morpholog" in end_reason) or ("contract" in end_reason)
    complete = steps == ROWS_PER_COMPLETE_TRACE and end_reason == COMPLETE_END_REASON and not contract_failure
    rows = {"obs35": obs35, "t_pre": t_pre, "targets": np.ascontiguousarray(u_mean.astype(np.float32))}
    report = {
        "job_dir": C.rel(job_dir),
        "steps": steps,
        "end_reason": end_reason,
        "complete": bool(complete),
        "contract_failure": bool(contract_failure),
        "mean_match_max_abs_dev": mean_dev,
        "trace_sha256": traj["trace_sha256"],
        "summary_sha256": traj["summary_sha256"],
        "reset_sha256": traj["reset_sha256"],
        "fsm_counters": {k: fsm[k] for k in ("hs_cancelled_count", "resync_count", "invalid_event_count", "valid_cycle_count", "timeout_exceeded")},
    }
    return rows, report


def det_anchor_rows(ctx: Mapping[str, Any]) -> tuple[list[dict[str, np.ndarray]], list[dict[str, Any]]]:
    """The 3 pinned det A_iso6clk anchors (content-addressed; 1500 rows; targets = V26 mean)."""
    pins_report = R.verify_anchor_pins()
    if not pins_report.get("all_match"):
        raise V26BContractError("pinned det anchors do not match their content digests")
    rows_list, reports = [], []
    for start, spec in R.ANCHORS.items():
        rows, rep = _trace_rows(Path(spec["job_dir"]), ctx=ctx, expected_seed=R.DET_SEED, expected_start=start, expected_selection="deterministic", pins=spec)
        if not rep["complete"]:
            raise V26BContractError(f"pinned det anchor {start} is not a complete 500-row trace")
        rep.update({"role": "anchor_det_existing", "start": start, "seed": R.DET_SEED, "sigma": None})
        rows["job_id"] = np.asarray([f"anchor_det_{start}"] * rows["obs35"].shape[0], dtype=str)
        rows_list.append(rows)
        reports.append(rep)
    return rows_list, reports


# --- fail-closed collector ------------------------------------------------------------------------

def collect_anchor_rows(stoch_jobs: Sequence[Mapping[str, Any]], ctx: Mapping[str, Any], *, include_det: bool = True) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Pool, filter and dedup the anchor rows; enforce the MANDATORY coverage target.

    ``stoch_jobs``: planner specs (each with an existing ``dir``).  Hard errors
    (identity/digest/receipt/teacher-mean/label conflict) raise
    ``V26BContractError``; quality exclusions (incomplete trace, contract
    failure) contribute NO rows and are counted; coverage below target raises
    ``CoverageStop`` carrying the full report (STOP: architect decision)."""
    per_trace: list[dict[str, Any]] = []
    pool: list[dict[str, np.ndarray]] = []
    complete_by_sigma = {sigma_tag(s): 0 for s in SIGMA_GRID}
    for spec in stoch_jobs:
        assert_anchor_seed(int(spec["seed"]), context="anchor_collection_stoch")
        sigma = float(spec["sigma"])
        tag = sigma_tag(sigma)
        job_dir = Path(spec["dir"])
        validate_receipt(job_dir, spec)
        rows, rep = _trace_rows(job_dir, ctx=ctx, expected_seed=int(spec["seed"]), expected_start=str(spec["start"]), expected_selection="stochastic")
        rep.update({"role": "anchor_stoch", "job_id": spec["job_id"], "sigma": sigma, "sigma_tag": tag, "seed": int(spec["seed"]), "start": str(spec["start"])})
        per_trace.append(rep)
        if rep["complete"]:
            complete_by_sigma[tag] += 1
            rows["job_id"] = np.asarray([str(spec["job_id"])] * rows["obs35"].shape[0], dtype=str)
            pool.append(rows)
        # incomplete / contract-failure traces contribute NO rows (counted above)
    det_rows_count = 0
    if include_det:
        det_list, det_reports = det_anchor_rows(ctx)
        det_rows_count = int(sum(r["obs35"].shape[0] for r in det_list))
        pool.extend(det_list)
        per_trace.extend(det_reports)
    if not pool:
        report = _coverage_report(per_trace, complete_by_sigma, unique_rows=0, duplicates=0, det_rows=0, raw_rows=0, ctx=ctx)
        raise CoverageStop("no usable anchor trace: STOP, architect decision required", report)
    obs = np.concatenate([r["obs35"] for r in pool], axis=0)
    t_pre = np.concatenate([r["t_pre"] for r in pool], axis=0)
    targets = np.concatenate([r["targets"] for r in pool], axis=0)
    job_ids = np.concatenate([r["job_id"] for r in pool], axis=0)
    raw_rows = int(obs.shape[0])
    seen: dict[bytes, int] = {}
    keep: list[int] = []
    duplicates = 0
    for i in range(raw_rows):
        key = obs[i].tobytes()
        j = seen.get(key)
        if j is None:
            seen[key] = i
            keep.append(i)
        else:
            if targets[i].tobytes() != targets[j].tobytes():
                raise V26BContractError(f"bitwise-duplicate observation with CONFLICTING targets (rows {j} '{job_ids[j]}' vs {i} '{job_ids[i]}'): label ambiguity, fail-closed")
            duplicates += 1
    idx = np.asarray(keep, dtype=np.int64)
    unique_rows = int(idx.size)
    report = _coverage_report(per_trace, complete_by_sigma, unique_rows=unique_rows, duplicates=duplicates, det_rows=det_rows_count, raw_rows=raw_rows, ctx=ctx)
    failures = []
    if unique_rows < MIN_UNIQUE_ROWS:
        failures.append(f"unique rows {unique_rows} < {MIN_UNIQUE_ROWS}")
    for tag, n in complete_by_sigma.items():
        if n < MIN_COMPLETE_TRACES_PER_SIGMA:
            failures.append(f"sigma {tag}: {n} complete traces < {MIN_COMPLETE_TRACES_PER_SIGMA}")
    if failures:
        raise CoverageStop("coverage below the mandatory target (" + "; ".join(failures) + "): STOP, architect decision required; NO seed is added automatically", report)
    arrays = {
        "obs35": np.ascontiguousarray(obs[idx]),
        "t_pre": np.ascontiguousarray(t_pre[idx]),
        "targets": np.ascontiguousarray(targets[idx]),
        "job_id": np.ascontiguousarray(job_ids[idx]),
    }
    return arrays, report


def _coverage_report(per_trace: list[dict[str, Any]], complete_by_sigma: Mapping[str, int], *, unique_rows: int, duplicates: int, det_rows: int, raw_rows: int, ctx: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema": "v26b_coverage.1",
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": C.sha256_file(PROTOCOL_JSON),
        "coverage_policy": {"min_bitwise_unique_rows": MIN_UNIQUE_ROWS, "min_complete_traces_per_sigma": MIN_COMPLETE_TRACES_PER_SIGMA, "rows_per_complete_trace": ROWS_PER_COMPLETE_TRACE, "waiver": "NONE (rejected by the architect, rev3)"},
        "targets_rule": "deterministic V26 mean on visited obs39 (A_iso6clk); never the sampled action; offline-vs-recorded mean <= 1e-5 per trace",
        "teacher": {"actor_digest": ctx["digest"], "module_state_sha256": ctx["module_state_sha256"]},
        "raw_rows": int(raw_rows),
        "unique_rows": int(unique_rows),
        "bitwise_duplicates_removed": int(duplicates),
        "det_anchor_rows": int(det_rows),
        "complete_traces_by_sigma": dict(complete_by_sigma),
        "traces": per_trace,
        "pass": bool(unique_rows >= MIN_UNIQUE_ROWS and all(n >= MIN_COMPLETE_TRACES_PER_SIGMA for n in complete_by_sigma.values())),
    }


# --- preflight ------------------------------------------------------------------------------------

def output_root_inventory() -> list[str]:
    if not OUT_ROOT.exists():
        return []
    entries = []
    for p in sorted(OUT_ROOT.rglob("*")):
        rel = p.relative_to(OUT_ROOT).as_posix()
        entries.append(rel + "/" if p.is_dir() and not p.is_symlink() else rel)
    return entries


def preflight() -> dict[str, Any]:
    """Read-only preflight: protocol binding, pins, anchors, driver inputs, output-root state."""
    proto = load_protocol()
    table = driver_inputs_table()
    anchors = R.verify_anchor_pins()
    if not anchors.get("all_match"):
        raise V26BContractError("existing det anchor pins do not match the disk content")
    teacher_module = Path(R.TEACHER["module"])
    state_sha = C.sha256_file(teacher_module / "module_state.pkl")
    if state_sha != R.TEACHER["module_state_sha256"]:
        raise V26BContractError("teacher module_state.pkl differs from its pin")
    missing = [str(Path(R.ANCHORS[s]["job_dir"])) for s in R.STARTS if not Path(R.ANCHORS[s]["job_dir"]).is_dir()]
    if missing:
        raise V26BContractError(f"pinned anchor job dirs missing: {missing}")
    return {
        "protocol_id": proto["protocol_id"],
        "protocol_sha256": table["protocol_sha256"],
        "driver_inputs": table,
        "teacher": {"module": C.rel(teacher_module), "module_state_sha256": state_sha, "actor_digest_pinned": R.TEACHER["actor_digest"]},
        "det_anchors_all_match": True,
        "output_root": {"path": C.rel(OUT_ROOT), "exists": OUT_ROOT.exists(), "entries": output_root_inventory()},
        "seeds": {"anchor_only": list(ANCHOR_SEEDS), "reserved_untouched": {str(k): v for k, v in RESERVED_SEED_ROLES.items()}},
    }


# --- dry-run (the ONLY default action; materialises exactly 3 manifest files) ---------------------

def _atomic_fill_reserved(path: Path, text: str) -> None:
    """Fill a reserved (already-created, empty) file atomically: .part + fsync + os.replace."""
    part = path.with_name(path.name + f".part-{os.getpid()}")
    with open(part, "x", encoding="utf-8") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(part, path)


def write_dry_run_manifest(python_exe: str) -> dict[str, Any]:
    """No-clobber dry-run: plan + preflight into exactly 3 new files under OUT_MANIFEST."""
    pre_inventory = output_root_inventory()
    flight = preflight()
    specs = job_specs()
    modules = sigma_module_plan()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    OUT_MANIFEST.mkdir(parents=True, exist_ok=True)
    reserved = R.reserve_unique_set(OUT_MANIFEST, f"v26b_anchors_dry_run_{stamp}", (".json", ".md", ".index.json"))
    manifest = {
        "schema": "v26b_dry_run.1",
        "stage": "V0_dry_run (collection NOT executed)",
        "stamp": stamp,
        "preflight": flight,
        "sigma_modules": [{**m, "out_dir": C.rel(m["out_dir"]), "module": C.rel(m["module"]), "materialised": False} for m in modules],
        "jobs": [{**s, "module": C.rel(s["module"]), "dir": C.rel(s["dir"]), "command": job_command(s, python_exe), "materialised": False} for s in specs],
        "coverage_policy": {"min_bitwise_unique_rows": MIN_UNIQUE_ROWS, "min_complete_traces_per_sigma": MIN_COMPLETE_TRACES_PER_SIGMA, "expected_stoch_traces": EXPECTED_STOCH_TRACES, "expected_raw_rows": EXPECTED_STOCH_TRACES * ROWS_PER_COMPLETE_TRACE + 1500, "waiver": "NONE"},
        "collect_command_proposed": [python_exe, str(Path(__file__).resolve()), "--collect", "--workers", "2", "--authorized-stage", AUTHORIZED_STAGE],
        "executed": False,
    }
    text = json.dumps(manifest, indent=2, ensure_ascii=False, default=str) + "\n"
    md = [
        f"# V26B anchor collection — dry-run {stamp} (nothing executed)",
        "",
        f"- protocol: `{PROTOCOL_ID}` (`{flight['protocol_sha256'][:12]}…`), coverage ≥ {MIN_UNIQUE_ROWS} unique rows / ≥ {MIN_COMPLETE_TRACES_PER_SIGMA} complete traces per sigma, **no waiver**",
        f"- planned: {len(modules)} derived sigma modules {list(SIGMA_GRID)} + {len(specs)} stochastic A_iso6clk jobs (3 starts × seeds {list(ANCHOR_SEEDS)} × 3 sigmas), 0 materialised",
        f"- targets: deterministic V26 mean (actor digest `{R.TEACHER['actor_digest'][:12]}…`), never the sampled action",
        f"- output root: `{C.rel(OUT_ROOT)}` (pre-existing entries: {len(pre_inventory)})",
        "- collection NOT executed; the real run requires `--collect --authorized-stage V26B-ANCHORS` (architect GO)",
        "",
    ]
    index = {"stamp": stamp, "files": {k: str(p.name) for k, p in reserved.items()}, "jobs_planned": len(specs), "jobs_materialised": 0, "executed": False}
    _atomic_fill_reserved(reserved[".json"], text)
    _atomic_fill_reserved(reserved[".md"], "\n".join(md))
    _atomic_fill_reserved(reserved[".index.json"], json.dumps(index, indent=2) + "\n")
    post_inventory = output_root_inventory()
    created = sorted(set(post_inventory) - set(pre_inventory))
    expected_new = {"manifest/"} | {f"manifest/{p.name}" for p in reserved.values()}
    unexpected = [e for e in created if e not in expected_new]
    if unexpected:
        raise V26BContractError(f"dry-run materialised unexpected entries: {unexpected}")
    if any(e.startswith(("rollouts", "derived_modules", "coverage", "logs")) for e in created):
        raise V26BContractError("dry-run must not create rollout/module/coverage/log entries")
    return {"reserved": {k: str(v) for k, v in reserved.items()}, "created_entries": created, "jobs_planned": len(specs), "manifest": manifest}


# --- guarded real collection (NOT part of V0; requires the architect stage token) -----------------

V26B_SIGMA_RECEIPT = "v26b_sigma_variant_receipt.json"


def _derive_constant_sigma_v26b(source_module: Path, output_dir: Path, *, sigma: float, expected_source_actor_digest: str) -> dict[str, Any]:
    """Faithful mirror of ``f1_sigma_variant.derive_constant_sigma_module`` (materialize=True)
    for the off-0.005 sigmas of the frozen grid.

    The F1 tool hard-codes ``ENTROPY_EXPECTED = -7.758758`` (the sigma=0.005 value), so it can
    never accept sigma 0.0025 / 0.01; F1 is immutable.  This mirror reuses the SAME production
    primitives through the F1 namespace (``SV.CAE.configure_constant_std``, ``SV.W`` load/save/
    compare, ``SV.logstd_head_report``, ``SV.module_files_table``) and reproduces every
    materialisation step byte-for-byte; the ONLY functional difference is the entropy
    expectation, computed from the requested sigma (same 1e-5 tolerance).  The bias check
    (``logstd_bias_exact``) is unchanged and remains the binding invariant.  Equivalence with
    the F1 tool at sigma=0.005 is asserted by the selftest (bit-identical module_state)."""
    source_module = Path(source_module)
    output_dir = Path(output_dir)
    out_module = output_dir / "rl_module"
    source_files = SV.module_files_table(source_module)
    state = SV.W.load_module_state(source_module)
    source_digest = SV.W.actor_state_digest(state)
    if source_digest != expected_source_actor_digest:
        raise V26BContractError(f"source actor digest {source_digest} != expected {expected_source_actor_digest}")
    configured, cae_report = SV.CAE.configure_constant_std(state, sigma=sigma, action_dim=F1.ACTION_DIM)

    def head_checked(st: dict[str, Any], what: str) -> dict[str, Any]:
        head = SV.logstd_head_report(st, sigma=sigma)
        expected_nat = F1.entropy_diag_gauss([float(np.log(sigma))] * F1.ACTION_DIM)
        head["entropy_expected_nat_v26b"] = expected_nat
        head["entropy_within_tolerance_v26b"] = bool(abs(head["entropy_nat"] - expected_nat) <= SV.ENTROPY_TOL)
        if not (head["logstd_weight_rows_all_zero"] and head["logstd_bias_exact"] and head["entropy_within_tolerance_v26b"]):
            raise V26BContractError(f"constant-sigma invariants failed {what} (sigma {sigma}): {head}")
        return head

    head = head_checked(configured, "in memory")
    mean_compare = {key: bool(np.array_equal(SV._as_np(state[key]), SV._as_np(configured[key]))) for key in state if key not in ("pi.1.weight", "pi.1.bias")}
    mean_rows_exact = bool(
        np.array_equal(SV._as_np(state["pi.1.weight"])[:F1.ACTION_DIM], SV._as_np(configured["pi.1.weight"])[:F1.ACTION_DIM])
        and np.array_equal(SV._as_np(state["pi.1.bias"])[:F1.ACTION_DIM], SV._as_np(configured["pi.1.bias"])[:F1.ACTION_DIM])
    )
    if not (all(mean_compare.values()) and mean_rows_exact):
        raise V26BContractError("mean parameters changed by the sigma transform")
    width = int(SV._as_np(state["pi.0.0.weight"]).shape[1])
    derived_digest = SV.W.actor_state_digest(configured)
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "v26b_anchors._derive_constant_sigma_v26b (faithful mirror of f1_sigma_variant.derive_constant_sigma_module; entropy expectation computed from sigma instead of the F1 sigma=0.005 constant)",
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "mirror_of_tool_sha256": C.sha256_file(F1_DIR / "f1_sigma_variant.py"),
        "mode": "materialize",
        "source_module": C.rel(source_module),
        "source_module_files_sha256": source_files,
        "source_actor_digest": source_digest,
        "source_head": SV.logstd_head_report(state, sigma=sigma),
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
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing derived module dir: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source_module, out_module, copy_function=shutil.copy2)
    with (out_module / "module_state.pkl").open("wb") as handle:
        pickle.dump(configured, handle, protocol=pickle.HIGHEST_PROTOCOL)
    reloaded = SV.W.load_module_state(out_module)
    compare = SV.W.compare_actor_states(configured, reloaded)
    if not compare["exact"]:
        raise V26BContractError(f"save/reload mismatch: {compare}")
    head2 = head_checked(reloaded, "after reload")
    manifest_src = source_module / SV.W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    names = json.loads(manifest_src.read_text(encoding="utf-8")).get("actor_feature_names") if manifest_src.is_file() else None
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
    C.write_json(out_module / SV.W.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest, clobber=manifest_src.is_file())
    report.update({
        "materialized": True,
        "derived_module_files_sha256": SV.module_files_table(out_module),
        "save_reload_exact": True,
        "derived_head_after_reload": head2,
        "manifest": manifest,
    })
    C.write_json(output_dir / V26B_SIGMA_RECEIPT, report)
    return report


def derive_sigma_modules(plans: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Materialise the constant-sigma modules via the F1 tool, which OWNS each output dir
    (``derive_constant_sigma_module`` refuses an existing ``output_dir`` and creates it
    itself) -- this helper never pre-creates it.  Fail-fast, NO automatic retry: on a
    failure the modules already derived stay on disk untouched, nothing is deleted, and
    the error names the offending dir; resuming or cleaning up is an operator/architect
    decision.  A re-invocation after a partial failure stops on the FIRST already-existing
    module dir (no-clobber), never re-deriving silently."""
    reports: list[dict[str, Any]] = []
    for plan in plans:
        out_dir = Path(plan["out_dir"])
        if out_dir.exists():
            raise V26BContractError(f"no-clobber: derived module dir already exists: {out_dir}; each module is derived exactly once -- no automatic retry, inspection/cleanup requires an architect decision")
        try:
            if float(plan["sigma"]) == float(F1.SIGMA_CONSTANT):  # 0.005: the unmodified F1 tool
                report = SV.derive_constant_sigma_module(Path(plan["source_module"]), out_dir, sigma=float(plan["sigma"]), materialize=True, expected_source_actor_digest=plan["expected_source_actor_digest"])
            else:  # 0.0025 / 0.01: faithful mirror (the F1 entropy constant is sigma=0.005-specific)
                report = _derive_constant_sigma_v26b(Path(plan["source_module"]), out_dir, sigma=float(plan["sigma"]), expected_source_actor_digest=plan["expected_source_actor_digest"])
        except FileExistsError as exc:  # the derivation tool's own no-clobber guard (never pre-empted here)
            raise V26BContractError(f"sigma-module derivation refused for {plan['name']}: {exc}; NO automatic retry (architect decision)") from exc
        module = Path(plan["module"])
        if not (module / "module_state.pkl").is_file():
            raise V26BContractError(f"derivation reported success but {module}/module_state.pkl is missing")
        if report.get("source_actor_digest") != plan["expected_source_actor_digest"]:
            raise V26BContractError(f"derived {plan['name']}: source digest {report.get('source_actor_digest')!r} != pinned {plan['expected_source_actor_digest']}")
        R.write_json_exclusive(out_dir / "derivation_report.json", report)
        reports.append(report)
    return reports


def _run_one_job(spec: Mapping[str, Any], python_exe: str, pins: Mapping[str, Any]) -> dict[str, Any]:
    job_dir = Path(spec["dir"])
    if job_dir.exists():
        raise V26BContractError(f"no-clobber: job dir already exists: {job_dir}")
    job_dir.mkdir(parents=True, exist_ok=False)
    log_path = OUT_LOGS / f"{spec['job_id']}.log"
    cmd = job_command(spec, python_exe)
    t0 = time.time()
    with open(log_path, "x", encoding="utf-8") as log:
        log.write(" ".join(cmd) + "\n\n")
        log.flush()
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(F1_DIR))
    receipt = {
        "schema": RECEIPT_SCHEMA,
        "job_id": spec["job_id"],
        "sigma": float(spec["sigma"]),
        "seed": int(spec["seed"]),
        "start": str(spec["start"]),
        "start_offset_s": float(spec["start_offset_s"]),
        "command": cmd,
        "returncode": int(proc.returncode),
        "duration_s": round(time.time() - t0, 3),
        "module": {"name": spec["candidate"], "path": C.rel(Path(spec["module"]))},
        "pins": {"runtime_config_sha256": F1.RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": R.ROLLOUT_EVAL_SHA256_PINNED, "aiso_driver_sha256": pins["aiso_driver_sha256"], "protocol_sha256": pins["protocol_sha256"]},
        "outputs": {},
        "log": C.rel(log_path),
    }
    if proc.returncode == 0:
        for name in TRACE_OUTPUT_FILES:
            p = job_dir / name
            if not p.is_file():
                receipt["returncode"] = -1
                receipt["error"] = f"missing output {name}"
                break
            receipt["outputs"][name] = C.sha256_file(p)
    R.write_json_exclusive(job_dir / RECEIPT_FILE, receipt)
    if receipt["returncode"] != 0:
        raise V26BContractError(f"job {spec['job_id']} failed (rc={receipt['returncode']}): STOP, no retry (log: {log_path})")
    return receipt


def run_collection(*, python_exe: str, workers: int, authorized_stage: str | None) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise V26BContractError(f"anchor collection requires --authorized-stage {AUTHORIZED_STAGE} (architect GO); got {authorized_stage!r}. V0 authorises tooling/tests/dry-run only")
    flight = preflight()
    if any(e.startswith(("rollouts", "derived_modules")) for e in flight["output_root"]["entries"]):
        raise V26BContractError("no-clobber: rollout/module entries already exist under the output root; collection runs exactly once")
    for d in (OUT_ROOT, OUT_MANIFEST, OUT_DERIVED, OUT_ROLLOUTS.parent, OUT_ROLLOUTS, OUT_LOGS, OUT_COVERAGE):
        d.mkdir(parents=True, exist_ok=True)
    derive_sigma_modules(sigma_module_plan())
    specs = job_specs()
    pins = flight["driver_inputs"]
    receipts: list[dict[str, Any]] = []
    stop = threading.Event()
    lock = threading.Lock()
    errors: list[BaseException] = []
    queue = list(specs)

    def worker() -> None:
        while not stop.is_set():
            with lock:
                if not queue:
                    return
                spec = queue.pop(0)
            try:
                rec = _run_one_job(spec, python_exe, pins)
                with lock:
                    receipts.append(rec)
            except BaseException as exc:  # fail-fast: no new jobs after a failure, no retry
                stop.set()
                with lock:
                    errors.append(exc)
                return

    threads = [threading.Thread(target=worker) for _ in range(max(1, int(workers)))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if errors:
        raise errors[0]
    ctx = teacher_context()
    status = "PASS"
    try:
        arrays, report = collect_anchor_rows(specs, ctx, include_det=True)
    except CoverageStop as exc:
        report = exc.report
        report["status"] = "STOP_coverage_below_target"
        status = "STOP"
        arrays = None
    else:
        report["status"] = "PASS"
    out_path = R.unique_artifact_path(OUT_COVERAGE, f"anchor_coverage_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    _atomic_fill_reserved(out_path, json.dumps(report, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"status": status, "coverage_report": str(out_path), "jobs_executed": len(receipts), "unique_rows": report.get("unique_rows"), "arrays": None if arrays is None else {k: v.shape for k, v in arrays.items()}}


# --- CLI ------------------------------------------------------------------------------------------

def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B anchor collection tooling (V0: dry-run only)")
    parser.add_argument("--collect", action="store_true", help="run the real collection (requires --authorized-stage V26B-ANCHORS)")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--authorized-stage", default=None)
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args(argv)
    if args.collect:
        result = run_collection(python_exe=args.python, workers=args.workers, authorized_stage=args.authorized_stage)
        print(json.dumps(result, indent=2, default=str))
        return 0 if result["status"] == "PASS" else 3
    result = write_dry_run_manifest(args.python)
    print(f"dry-run manifest: {result['reserved']['.json']}")
    print(f"jobs planned: {result['jobs_planned']} (materialised: 0); collection NOT executed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
