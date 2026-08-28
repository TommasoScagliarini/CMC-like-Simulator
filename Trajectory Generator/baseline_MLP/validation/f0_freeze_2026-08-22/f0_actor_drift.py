"""F0 activity 8: actor drift on ONE frozen 35D observation corpus (fail-closed, no-replace publish, byte-bound data inputs).

Scope of the guarantee: DATA inputs are byte-bound (PRE digest, every read re-opened and
checked against PRE, POST re-hash);
Python SOURCE is imported before the runtime ledger, so its recorded hashes attest on-disk
provenance only, not the bytecode already loaded - no code-execution TOCTOU freedom is claimed.

Primary pairs (H0 -> candidate) and secondary diagnostics (best -> last), all 35D:
  B0820_v2 : B0820_H0 -> B0820_V2_BEST / B0820_V2_LAST   (secondary: V2_BEST -> V2_LAST)
  B0820_v3 : B0820_H0 -> B0820_V3_BEST / B0820_V3_LAST   (secondary: V3_BEST -> V3_LAST)
  july     : JUL_H0   -> JUL_BEST / JUL_LAST             (secondary: JUL_BEST -> JUL_LAST)
No 39D module is ever compared (ABI 39 != 35 is a transplant, not drift).

Protocol of ``run`` (see f0_artifacts for the ledger/publish contract):
  discovery  - the 24 corpus jobs are projected purely from the registry (no filesystem read)
               and the exact consumed-input set is enumerated: final analysis, 24 receipts /
               summaries / traces, every module file of the 8 actors, the 35D actor feature
               manifest, the F0 scripts/helpers and warm_start.py used by the computation;
  PRE        - every declared input is hashed BEFORE any computational read; the interpreter
               provenance is recorded;
  load       - every read goes through the ledger and is byte-bound to the PRE digest (bytes
               hashed at each read and compared with PRE, parsers consume those bytes; the
               ledger is not a cache): the exact canonical 58-job index is validated and the
               consumed receipt projection plus summary/trace digests are bound directly to
               their evidence in the pinned analysis; summary / trace also match the receipt;
               the receipt's module_state_sha256 must equal the consumed H0 module; the set is
               re-hashed after loading;
  compute    - forward pass on ALL 24 traces (policy_action_mean for stochastic,
               raw_policy_action for deterministic; arrays exactly (n, 2), finite),
               6 primary + 3 secondary pairs on the same corpus; inputs re-hashed after;
  publish    - every declared input read; staging sibling, artefact-relative paths, strict JSON,
               single sidecar last, staging verified, inputs re-hashed immediately before the
               NO-REPLACE rename, published tree verified (failure raises -> nonzero exit).

Corpus (deliberate architectural choice: ONE set shared by every pair, covering both source
distributions B0820_H0 and JUL_H0 under the target runtime): every row of
``actor_observation_vector_before`` of exactly 24 traces under ``v3_canonical`` - det B0820_H0
(3 starts, seed 123), det JUL_H0 (3 starts, seed 123), stoch B0820_H0 (3 starts x seeds
123/124/125), stoch JUL_H0 (3 x 3) - in canonical order, exact identity, 35 finite values per
row with the manifest feature names, steps 1..n, time strictly increasing, contiguous row
ranges (cursor: no gaps, no overlaps). Saved as NPZ + JSON manifest; ``load_observation_set``
re-validates digests, shapes, contiguity, indices and time monotonicity.

Actor model: canonical ``pi.0.0 -> tanh -> pi.0.2 -> tanh -> pi.1`` (4 logits = mu_knee,
mu_ankle, logstd_knee, logstd_ankle); exact canonical keys, coherent shapes, recorded dtypes,
finite weights, ``pi_encoder.*`` aliases. BIT-EXACT means identical dtype + shape +
C-contiguous bytes (``bytes_equal``): a float32/float64 storage difference or a signed zero
(-0.0 vs +0.0, numerically equal) is NEVER equated. Numeric deltas are computed in float64.

Metrics: digests over dtype/shape/bytes; per tensor / global / mean head / log-std head
(weight AND bias) L2, RMS, max, relative deltas; delta mu and delta applied (runtime clip to
[-1, 1], separate) with RMSE, MAE, signed bias, abs max, p99, per action; sigma min/mean/max +
state dependence; analytic KL of diagonal Gaussians both directions + Jeffreys with
mean/min/p50/p95/p99/max, row-weighted, macro per trace, per origin; overflow / non-finite
detection fails closed. No threshold of "good/bad" drift is invented.
"""

from __future__ import annotations

import argparse
import hashlib
import pickle
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_artifacts as ART  # noqa: E402
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402
from f0_matrix_analysis import as_int, assert_finite_metrics, is_finite_float  # noqa: E402

FINAL_ANALYSIS_FILE = "f0_matrix_analysis_20260823_011338.json"
FINAL_ANALYSIS_SHA256 = "0ac942432d0505731b93e6322ac337edf91ab7dca8c9c8e9c1cb122089148987"
ACTION_DIM = 2
OBS_WIDTH = 35
CANONICAL_ACTOR_KEYS = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")
ENCODER_ALIASES = {"pi_encoder.0.weight": "pi.0.0.weight", "pi_encoder.0.bias": "pi.0.0.bias", "pi_encoder.2.weight": "pi.0.2.weight", "pi_encoder.2.bias": "pi.0.2.bias"}
MODULE_FILES = ("module_state.pkl", "metadata.json", "class_and_ctor_args.pkl")
TARGET_RUNTIME = "v3_canonical"
H0_CANDIDATES = ("B0820_H0", "JUL_H0")
PRIMARY_PAIRS: tuple[tuple[str, str, str], ...] = (("B0820_v2", "B0820_H0", "B0820_V2_BEST"), ("B0820_v2", "B0820_H0", "B0820_V2_LAST"), ("B0820_v3", "B0820_H0", "B0820_V3_BEST"), ("B0820_v3", "B0820_H0", "B0820_V3_LAST"), ("july", "JUL_H0", "JUL_BEST"), ("july", "JUL_H0", "JUL_LAST"))
SECONDARY_PAIRS: tuple[tuple[str, str, str], ...] = (("B0820_v2", "B0820_V2_BEST", "B0820_V2_LAST"), ("B0820_v3", "B0820_V3_BEST", "B0820_V3_LAST"), ("july", "JUL_BEST", "JUL_LAST"))
FORWARD_TOLERANCE = 1e-5
KL_DIRECTIONS = ("kl_h0_to_candidate", "kl_candidate_to_h0", "kl_symmetric_jeffreys")
CORPUS_NOTE = "deliberate architectural choice: ONE frozen 24-trace corpus (H0 actors B0820_H0 and JUL_H0 under the target runtime v3_canonical, det + dev-seed stochastic) shared by every pair, so that all chains are evaluated on both source distributions"
CODE_INPUTS = ("f0_common.py", "f0_rollout_matrix.py", "f0_matrix_analysis.py", "f0_closure.py", "f0_artifacts.py", "f0_actor_drift.py")
RUNTIME_CODE_INPUTS = ("warm_start.py",)
IDENTITY_KEYS = ("job_id", "family", "candidate", "runtime", "start", "action_selection", "seed", "repeat")
BIT_EXACT_NOTE = "bit_exact = identical dtype + shape + C-contiguous bytes (tobytes equality): float32/float64 storage or signed zero (-0.0 vs +0.0) are never equated; numeric deltas computed in float64"


class DriftError(RuntimeError):
    """Fail-closed error of the drift protocol (ABI, seeds, runtime, corpus, provenance, forward pass, finiteness)."""


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def bytes_equal(a: np.ndarray, b: np.ndarray) -> bool:
    """Identical dtype, shape and C-contiguous bytes (signed zeros and storage dtypes differ)."""
    a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
    return bool(a.dtype == b.dtype and a.shape == b.shape and a.tobytes(order="C") == b.tobytes(order="C"))


# --- corpus ---------------------------------------------------------------------------------


def corpus_specs() -> list[M.JobSpec]:
    index = M.canonical_index()
    specs: list[M.JobSpec] = []
    for cand in H0_CANDIDATES:
        for family in ("det", "stoch"):
            for start in ("minus020", "nominal", "plus020"):
                for seed in ((123,) if family == "det" else C.DEVELOPMENT_SEEDS):
                    mode = "deterministic" if family == "det" else "stochastic"
                    jid = M.job_id((family, cand, TARGET_RUNTIME, start, mode, seed, 1, None))
                    spec = index.get(jid)
                    if spec is None or spec[:7] != (family, cand, TARGET_RUNTIME, start, mode, seed, 1):
                        raise DriftError(f"corpus job missing from the canonical matrix or inconsistent: {jid}")
                    specs.append(spec)
    if len(specs) != 24 or len({M.job_id(s) for s in specs}) != 24:
        raise DriftError(f"corpus must have exactly 24 distinct traces, got {len(specs)}")
    return specs


def expected_corpus_identity() -> list[dict[str, Any]]:
    return [{"job_id": M.job_id(s), "family": s[0], "candidate": s[1], "runtime": s[2], "start": s[3], "action_selection": s[4], "seed": s[5], "repeat": s[6]} for s in corpus_specs()]


def load_final_analysis(path: Path | None = None, expected_sha: str | None = FINAL_ANALYSIS_SHA256, *, reader: ART.DirectReader = ART.DIRECT) -> tuple[dict[str, Any], dict[str, Any]]:
    path = Path(path) if path is not None else C.OUT_METRICS / FINAL_ANALYSIS_FILE
    if path.is_symlink() or not path.is_file():
        raise DriftError(f"final analysis artefact missing: {C.rel(path)}")
    raw = reader.read_bytes(path)
    sha = ART.sha256_bytes(raw)
    if expected_sha is not None and sha != expected_sha:
        raise DriftError(f"final analysis artefact digest mismatch: {sha} != pinned {expected_sha}")
    payload = ART.loads_strict(raw)
    if not isinstance(payload, dict) or payload.get("analysis_complete") is not True:
        raise DriftError("final analysis must be complete (analysis_complete == true) before the drift protocol")
    return payload, {"path": C.rel(path), "sha256": sha}


def _rows(raw: bytes, label: str) -> list[dict[str, Any]]:
    rows = ART.loads_strict(raw)
    if not isinstance(rows, list) or not rows or not all(isinstance(r, dict) for r in rows):
        raise DriftError(f"empty or malformed trace: {label}")
    return rows


def read_job_files(rec: dict[str, Any], *, analysis_job: dict[str, Any] | None = None, verify=M.verify_existing, reader: ART.DirectReader = ART.DIRECT, module_digest: str | None = None) -> dict[str, Any]:
    """Consume receipt / summary / trace and bind them to receipt + pinned analysis.

    ``verify`` is retained for isolated helper tests; production uses ``analysis_job`` and no
    live verifier, so no undeclared verifier input can be read outside the ledger.
    """
    out_dir = C.REPO / rec["output_dir"]
    receipt_raw = reader.read_bytes(out_dir / M.RECEIPT_FILE)
    receipt = ART.loads_strict(receipt_raw)
    if not isinstance(receipt, dict):
        raise DriftError(f"{rec['job_id']}: receipt is not a mapping")
    if verify is not None:
        verified = verify(rec, out_dir)
        if verified != receipt:
            raise DriftError(f"{rec['job_id']}: receipt verified on disk differs from the receipt bytes consumed (TOCTOU)")
    summary_raw = reader.read_bytes(out_dir / M.SUMMARY_FILE)
    trace_raw = reader.read_bytes(out_dir / M.TRACE_FILE)
    s_sha, t_sha = ART.sha256_bytes(summary_raw), ART.sha256_bytes(trace_raw)
    if receipt.get("summary_sha256") != s_sha or receipt.get("trace_sha256") != t_sha:
        raise DriftError(f"{rec['job_id']}: summary/trace bytes consumed do not match the receipt digests")
    if module_digest is not None and receipt.get("module_state_sha256") != module_digest:
        raise DriftError(f"{rec['job_id']}: receipt module_state_sha256 {receipt.get('module_state_sha256')} != consumed module {module_digest}")
    analysis_binding: dict[str, Any]
    if analysis_job is not None:
        if {k: analysis_job.get(k) for k in IDENTITY_KEYS} != {k: rec.get(k) for k in IDENTITY_KEYS} or analysis_job.get("output_dir") != rec.get("output_dir"):
            raise DriftError(f"{rec['job_id']}: pinned-analysis identity differs from the static registry record")
        expected_receipt = analysis_job.get("receipt")
        receipt_keys = [k for k in expected_receipt] if isinstance(expected_receipt, dict) else []
        receipt_keys = [k for k in receipt_keys if k != "provenance_class_effective"]  # derived by matrix analysis, absent from the receipt file
        if not isinstance(expected_receipt, dict) or {k: receipt.get(k) for k in receipt_keys} != {k: expected_receipt[k] for k in receipt_keys}:
            raise DriftError(f"{rec['job_id']}: consumed receipt projection differs from the pinned analysis")
        evidence = analysis_job.get("evidence")
        expected = {
            "summary": (C.rel(out_dir / M.SUMMARY_FILE), s_sha),
            "trace": (C.rel(out_dir / M.TRACE_FILE), t_sha),
        }
        if not isinstance(evidence, dict):
            raise DriftError(f"{rec['job_id']}: pinned analysis has no evidence mapping")
        for name, (path, sha) in expected.items():
            item = evidence.get(name)
            if not isinstance(item, dict) or item.get("path") != path or item.get("present") is not True or item.get("valid") is not True or item.get("sha256") != sha:
                raise DriftError(f"{rec['job_id']}: consumed {name} bytes differ from pinned-analysis evidence")
        analysis_binding = {"mode": "pinned_analysis_exact_evidence", "receipt_projection": "deep_equal_except_analysis-derived provenance_class_effective", "summary_trace": "sha256_equal", "evidence_sha256": {k: v[1] for k, v in expected.items()}}
    else:
        analysis_binding = {"mode": "direct_helper_unbound_to_analysis"}
    summary = ART.loads_strict(summary_raw)
    if not isinstance(summary, dict):
        raise DriftError(f"{rec['job_id']}: summary is not a mapping")
    return {"receipt": receipt, "summary": summary, "rows": _rows(trace_raw, rec["job_id"]), "receipt_sha256": ART.sha256_bytes(receipt_raw), "summary_sha256": s_sha, "trace_sha256": t_sha, "analysis_binding": analysis_binding}


def mean_presence(rows: list[dict[str, Any]], label: str) -> bool:
    """Classify the homogeneous policy-mean encoding used by the frozen traces.

    Stochastic rows must all carry a list. Deterministic traces may encode the unavailable mean
    either by omitting the key everywhere or by storing JSON null everywhere. Any mixture or
    other value fails closed.
    """
    states = ["absent" if "policy_action_mean" not in r else ("null" if r["policy_action_mean"] is None else ("list" if isinstance(r["policy_action_mean"], list) else "invalid")) for r in rows]
    if all(s == "list" for s in states):
        return True
    if all(s == "absent" for s in states) or all(s == "null" for s in states):
        return False
    bad = [rows[i].get("step") for i, state in enumerate(states) if state != states[0]]
    raise DriftError(f"{label}: heterogeneous policy_action_mean encoding {sorted(set(states))}; first differing rows {bad[:5]}")


def action_array(rows: list[dict[str, Any]], key: str, label: str) -> np.ndarray:
    """Exactly (n, 2) finite action rows (ragged / wrong width / non-finite / non-numeric fail closed)."""
    values = []
    for r in rows:
        v = r.get(key)
        if not isinstance(v, list) or len(v) != ACTION_DIM or not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
            raise DriftError(f"{label}: {key} must be a list of {ACTION_DIM} numbers at step {r.get('step')}, got {v!r}")
        values.append([float(x) for x in v])
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (len(rows), ACTION_DIM) or not np.all(np.isfinite(arr)):
        raise DriftError(f"{label}: {key} array must be finite with shape ({len(rows)}, {ACTION_DIM})")
    return arr


def build_observation_set(*, analysis_ref: dict[str, Any] | None = None, analysis_jobs: dict[str, dict[str, Any]] | None = None, verify=M.verify_existing, python_exe: str | None = None, feature_manifest: list[str] | None = None, jobs: list[dict[str, Any]] | None = None, expected: list[dict[str, Any]] | None = None, reader: ART.DirectReader = ART.DIRECT, module_digests: dict[str, str] | None = None) -> dict[str, Any]:
    names = feature_manifest if feature_manifest is not None else C.load_actor_feature_manifest("B0820_H0")["actor_feature_names"]
    if len(names) != OBS_WIDTH or len(set(names)) != OBS_WIDTH:
        raise DriftError(f"35D feature manifest expected, got {len(names)} names")
    expected = expected if expected is not None else expected_corpus_identity()
    records = jobs if jobs is not None else [M.describe_job(s, python_exe or C.select_python()["selected"], None) for s in corpus_specs()]
    if len(records) != len(expected) or len(expected) != 24:
        raise DriftError(f"24 corpus jobs expected, got {len(records)}")
    seen: set[str] = set()
    for rec, exp in zip(records, expected):
        ident = {k: rec.get(k) for k in IDENTITY_KEYS}
        if ident != exp:
            raise DriftError(f"corpus identity mismatch (exact set and canonical order required): got {ident}, expected {exp}")
        if (exp["family"] == "det") != (exp["action_selection"] == "deterministic") or exp["runtime"] != TARGET_RUNTIME or exp["candidate"] not in H0_CANDIDATES:
            raise DriftError(f"corpus identity incoherent: {exp}")
        if exp["seed"] in C.SEALED_SEEDS or (exp["family"] == "det" and exp["seed"] != 123) or (exp["family"] == "stoch" and exp["seed"] not in C.DEVELOPMENT_SEEDS):
            raise DriftError(f"corpus seed not allowed: {exp}")
        if exp["job_id"] in seen:
            raise DriftError(f"duplicate corpus job: {exp['job_id']}")
        seen.add(exp["job_id"])
    blocks, times, traces = [], [], []
    cursor = 0
    for i, rec in enumerate(records):
        files = read_job_files(rec, analysis_job=analysis_jobs.get(rec["job_id"]) if analysis_jobs else None, verify=verify, reader=reader, module_digest=module_digests.get(rec["candidate"]) if module_digests else None)
        summary, rows = files["summary"], files["rows"]
        if summary.get("n_actor") != OBS_WIDTH or summary.get("actor_feature_names") != list(names) or summary.get("action_selection") != rec["action_selection"]:
            raise DriftError(f"{rec['job_id']}: summary ABI/action_selection is not the expected 35D H0 record (n_actor={summary.get('n_actor')}, action_selection={summary.get('action_selection')})")
        block = np.asarray([r.get("actor_observation_vector_before") for r in rows], dtype=np.float64) if all(isinstance(r.get("actor_observation_vector_before"), list) and len(r["actor_observation_vector_before"]) == OBS_WIDTH and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in r["actor_observation_vector_before"]) for r in rows) else None
        if block is None or block.ndim != 2 or block.shape != (len(rows), OBS_WIDTH) or not np.all(np.isfinite(block)):
            raise DriftError(f"{rec['job_id']}: observation rows must be {OBS_WIDTH} finite values on every row")
        steps = [as_int(r["step"]) if is_finite_float(r.get("step")) else None for r in rows]
        if steps != list(range(1, len(rows) + 1)):
            raise DriftError(f"{rec['job_id']}: trace steps are not the exact sequence 1..n")
        t = np.asarray([r.get("time") for r in rows], dtype=np.float64) if all(is_finite_float(r.get("time")) for r in rows) else None
        if t is None or not np.all(np.diff(t) > 0):
            raise DriftError(f"{rec['job_id']}: trace time must be finite and strictly increasing")
        action_array(rows, "raw_policy_action", rec["job_id"])
        has_mean = mean_presence(rows, rec["job_id"])
        if has_mean != (rec["action_selection"] == "stochastic"):
            raise DriftError(f"{rec['job_id']}: policy_action_mean presence incoherent with action_selection {rec['action_selection']}")
        if has_mean:
            action_array(rows, "policy_action_mean", rec["job_id"])
        blocks.append(block)
        times.append(t)
        traces.append({"index": i, "job_id": rec["job_id"], "candidate": rec["candidate"], "family": rec["family"], "start": rec["start"], "seed": rec["seed"], "action_selection": rec["action_selection"], "runtime": rec["runtime"], "output_dir": rec["output_dir"],
                       "trace": C.rel(C.REPO / rec["output_dir"] / M.TRACE_FILE), "trace_sha256": files["trace_sha256"], "summary_sha256": files["summary_sha256"], "receipt_sha256": files["receipt_sha256"], "receipt_schema_version": files["receipt"].get("schema_version"), "receipt_module_state_sha256": files["receipt"].get("module_state_sha256"), "analysis_binding": files["analysis_binding"],
                       "row_start": cursor, "row_end": cursor + len(rows), "row_count": len(rows), "t_start_s": float(t[0]), "t_end_s": float(t[-1]), "end_reason": summary.get("end_reason"), "forward_reference_field": "policy_action_mean" if has_mean else "raw_policy_action"})
        cursor += len(rows)
    obs = np.concatenate(blocks, axis=0)
    time_s = np.concatenate(times)
    trace_index = np.concatenate([np.full(t["row_count"], t["index"], dtype=np.int32) for t in traces])
    step_index = np.concatenate([np.arange(1, t["row_count"] + 1, dtype=np.int32) for t in traces])
    manifest = {"schema_version": 2, "generated_at_utc": C.utc_now(), "runtime": TARGET_RUNTIME, "h0_candidates": list(H0_CANDIDATES), "trace_count": len(traces), "row_count": int(obs.shape[0]), "width": OBS_WIDTH, "feature_names": list(names), "feature_count": len(names), "feature_names_sha256": C.sha256_text("\n".join(names)),
                "development_seeds": list(C.DEVELOPMENT_SEEDS), "sealed_seeds_never_used": list(C.SEALED_SEEDS), "source_analysis": analysis_ref, "traces": traces, "order": "candidate (B0820_H0, JUL_H0) > family (det, stoch) > start (minus020, nominal, plus020) > seed (123 | 123, 124, 125)", "corpus_note": CORPUS_NOTE, "row_ranges": "contiguous cursor: row_start[i] == row_end[i-1], no gaps, no overlaps",
                "rows_sha256": hashlib.sha256(np.ascontiguousarray(obs).tobytes()).hexdigest(), "time_sha256": hashlib.sha256(np.ascontiguousarray(time_s).tobytes()).hexdigest(), "trace_index_sha256": hashlib.sha256(np.ascontiguousarray(trace_index).tobytes()).hexdigest(), "step_index_sha256": hashlib.sha256(np.ascontiguousarray(step_index).tobytes()).hexdigest()}
    return {"obs": obs, "trace_index": trace_index, "step_index": step_index, "time_s": time_s, "manifest": manifest}


def write_observation_set(out_dir: Path, stamp: str, obs_set: dict[str, Any]) -> dict[str, Any]:
    npz = out_dir / f"observation_set_{stamp}.npz"
    man = out_dir / f"observation_set_{stamp}.json"
    for p in (npz, man):
        if p.exists() or p.is_symlink():
            raise FileExistsError(f"refusing to overwrite existing artefact: {p}")
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez(npz, obs=obs_set["obs"], trace_index=obs_set["trace_index"], step_index=obs_set["step_index"], time_s=obs_set["time_s"])
    manifest = dict(obs_set["manifest"], npz=npz.name, npz_sha256=C.sha256_file(npz))
    ART.write_json_strict(man, manifest)
    return {"npz": npz.name, "npz_sha256": manifest["npz_sha256"], "manifest": man.name, "manifest_sha256": C.sha256_file(man)}


def load_observation_set(npz_path: Path, manifest_path: Path) -> dict[str, Any]:
    """Re-validate a saved corpus: digests, shapes, contiguous row ranges (cursor), trace/step indices, time monotonicity."""
    manifest = ART.loads_strict(Path(manifest_path).read_bytes())
    sha = C.sha256_file(npz_path)
    if sha != manifest.get("npz_sha256"):
        raise DriftError(f"observation set digest mismatch: {sha} != {manifest.get('npz_sha256')}")
    with np.load(npz_path, allow_pickle=False) as data:
        obs, trace_index, step_index, time_s = (np.asarray(data["obs"], dtype=np.float64), np.asarray(data["trace_index"], dtype=np.int32), np.asarray(data["step_index"], dtype=np.int32), np.asarray(data["time_s"], dtype=np.float64))
    n, w = manifest["row_count"], manifest["width"]
    if obs.shape != (n, w) or trace_index.shape != (n,) or step_index.shape != (n,) or time_s.shape != (n,):
        raise DriftError("observation set shapes do not match the manifest")
    for key, arr in (("rows_sha256", obs), ("time_sha256", time_s), ("trace_index_sha256", trace_index), ("step_index_sha256", step_index)):
        if hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest() != manifest.get(key):
            raise DriftError(f"observation set {key} does not match the manifest")
    if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(time_s))):
        raise DriftError("observation set contains non-finite values")
    cursor = 0
    for i, t in enumerate(manifest["traces"]):
        a, b = t["row_start"], t["row_end"]
        if t["index"] != i or a != cursor or b - a != t["row_count"] or b <= a:
            raise DriftError(f"observation set row ranges not contiguous (gap/overlap) at {t['job_id']}: start {a} expected {cursor}")
        if not np.all(trace_index[a:b] == i) or not np.array_equal(step_index[a:b], np.arange(1, b - a + 1)):
            raise DriftError(f"observation set indices inconsistent for {t['job_id']}")
        seg = time_s[a:b]
        if not np.all(np.diff(seg) > 0) or seg[0] != t["t_start_s"] or seg[-1] != t["t_end_s"]:
            raise DriftError(f"observation set time not strictly increasing or bounds mismatch for {t['job_id']}")
        cursor = b
    if cursor != n or len(manifest["traces"]) != manifest["trace_count"]:
        raise DriftError("observation set row ranges do not cover the rows exactly")
    return {"obs": obs, "trace_index": trace_index, "step_index": step_index, "time_s": time_s, "manifest": manifest}


# --- actor modules ------------------------------------------------------------------------------


def load_actor(module_dir: Path, *, expected_width: int = OBS_WIDTH, reader: ART.DirectReader = ART.DIRECT) -> dict[str, Any]:
    """Canonical actor tensors consumed through the reader: ``_raw`` keeps the ORIGINAL dtype/shape
    (bit-exact comparisons), float64 copies compute the metrics; digests are of the consumed bytes."""
    sys.path.insert(0, str(C.BASELINE_DIR))
    import warm_start as W  # noqa: PLC0415

    module_dir = Path(module_dir)
    state_path = module_dir / MODULE_FILES[0]
    if state_path.is_symlink() or not state_path.is_file():
        raise DriftError(f"module_state.pkl missing or symlink in {C.rel(module_dir)}")
    raw_bytes = reader.read_bytes(state_path)
    digests: dict[str, str | None] = {MODULE_FILES[0]: ART.sha256_bytes(raw_bytes)}
    for name in MODULE_FILES[1:]:
        p = module_dir / name
        if p.is_symlink():
            raise DriftError(f"module file is a symlink: {C.rel(p)}")
        digests[name] = ART.sha256_bytes(reader.read_bytes(p)) if p.is_file() else None
    state = pickle.loads(raw_bytes)
    if not isinstance(state, Mapping):
        raise DriftError(f"module {C.rel(module_dir)}: module_state.pkl is not a mapping")
    missing = [k for k in CANONICAL_ACTOR_KEYS if k not in state]
    if missing:
        raise DriftError(f"module {C.rel(module_dir)} lacks canonical actor tensors {missing}")
    raw = {k: _as_numpy(state[k]) for k in state}
    for k in CANONICAL_ACTOR_KEYS:
        if not np.issubdtype(raw[k].dtype, np.floating):
            raise DriftError(f"module {C.rel(module_dir)}: {k} dtype {raw[k].dtype} is not floating")
    arrays = {k: raw[k].astype(np.float64) for k in CANONICAL_ACTOR_KEYS}
    for k, v in arrays.items():
        if not np.all(np.isfinite(v)):
            raise DriftError(f"module {C.rel(module_dir)}: non-finite values in {k}")
    w1, b1, w2, b2, w3, b3 = (arrays[k] for k in CANONICAL_ACTOR_KEYS)
    if w1.ndim != 2 or w1.shape[1] != expected_width:
        raise DriftError(f"module {C.rel(module_dir)}: actor input width {w1.shape} != {expected_width} (ABI mismatch, no drift comparison across ABIs)")
    if b1.shape != (w1.shape[0],) or w2.ndim != 2 or w2.shape[1] != w1.shape[0] or b2.shape != (w2.shape[0],) or w3.ndim != 2 or w3.shape[1] != w2.shape[0] or w3.shape[0] != 2 * ACTION_DIM or b3.shape != (2 * ACTION_DIM,):
        raise DriftError(f"module {C.rel(module_dir)}: incoherent tensor shapes {[tuple(arrays[k].shape) for k in CANONICAL_ACTOR_KEYS]}")
    for alias, canonical in ENCODER_ALIASES.items():
        if alias not in raw or not bytes_equal(raw[alias], raw[canonical]):
            raise DriftError(f"module {C.rel(module_dir)}: {alias} missing or not a bit-exact alias (dtype/shape/bytes) of {canonical}")
    return {**arrays, "_raw": {k: raw[k] for k in CANONICAL_ACTOR_KEYS}, "_provenance": {"module": C.rel(module_dir), "files_sha256": digests, "tensor_keys": list(CANONICAL_ACTOR_KEYS), "extra_keys": sorted(k for k in raw if k not in CANONICAL_ACTOR_KEYS), "shapes": {k: list(raw[k].shape) for k in CANONICAL_ACTOR_KEYS}, "dtypes": {k: str(raw[k].dtype) for k in CANONICAL_ACTOR_KEYS}, "actor_state_digest_warm_start": W.actor_state_digest(state)}}


def actor_digest(arrays: dict[str, Any]) -> str:
    """Digest over the ORIGINAL dtype, shape and C-contiguous bytes of the canonical tensors."""
    digest = hashlib.sha256()
    for key in CANONICAL_ACTOR_KEYS:
        value = np.ascontiguousarray(arrays["_raw"][key])
        digest.update(key.encode("utf-8"))
        digest.update(str(value.dtype).encode("ascii"))
        digest.update(np.asarray(value.shape, dtype=np.int64).tobytes())
        digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def actor_logits(arrays: dict[str, Any], obs: np.ndarray) -> np.ndarray:
    obs = np.asarray(obs, dtype=np.float64)
    if obs.ndim != 2 or obs.shape[1] != arrays["pi.0.0.weight"].shape[1]:
        raise DriftError(f"observation width {obs.shape} incompatible with the actor input {arrays['pi.0.0.weight'].shape}")
    h1 = np.tanh(obs @ arrays["pi.0.0.weight"].T + arrays["pi.0.0.bias"])
    h2 = np.tanh(h1 @ arrays["pi.0.2.weight"].T + arrays["pi.0.2.bias"])
    logits = h2 @ arrays["pi.1.weight"].T + arrays["pi.1.bias"]
    if not np.all(np.isfinite(logits)):
        raise DriftError("non-finite logits")
    return logits


def verify_forward_pass(arrays: dict[str, Any], out_dir: Path, action_selection: str, *, tolerance: float = FORWARD_TOLERANCE, reader: ART.DirectReader = ART.DIRECT) -> dict[str, Any]:
    """Canonical forward pass vs the recorded mean of the actor on its own trace (consumed through the reader);
    reference array exactly (n, 2) finite; tolerance-based (float32 runtime vs float64 re-evaluation)."""
    rows = _rows(reader.read_bytes(Path(out_dir) / M.TRACE_FILE), C.rel(out_dir))
    obs = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float64)
    mu = actor_logits(arrays, obs)[:, :ACTION_DIM]
    has_mean = mean_presence(rows, C.rel(out_dir))
    if action_selection == "stochastic":
        if not has_mean:
            raise DriftError(f"{C.rel(out_dir)}: stochastic trace without policy_action_mean")
        field = "policy_action_mean"
    elif action_selection == "deterministic":
        if has_mean:
            raise DriftError(f"{C.rel(out_dir)}: deterministic trace carries policy_action_mean")
        field = "raw_policy_action"
    else:
        raise DriftError(f"unknown action_selection {action_selection!r}")
    ref = action_array(rows, field, C.rel(out_dir))
    err = float(np.max(np.abs(mu - ref)))
    if not np.isfinite(err) or err > tolerance:
        raise DriftError(f"forward pass inconsistent with the recorded {field} on {C.rel(out_dir)}: max |delta| {err:.3e} > {tolerance}")
    return {"trace": C.rel(Path(out_dir) / M.TRACE_FILE), "action_selection": action_selection, "compared_with": field, "rows": int(len(rows)), "max_abs_error": err, "tolerance": tolerance, "comparison": "tolerance-based (float32 runtime vs float64 re-evaluation), not bit-exact"}


# --- metrics ---------------------------------------------------------------------------------------


def _tensor_delta(ref: np.ndarray, cand: np.ndarray, ref_raw: np.ndarray | None = None, cand_raw: np.ndarray | None = None) -> dict[str, Any]:
    d = cand - ref
    ref_l2 = float(np.linalg.norm(ref))
    out = {"shape": list(ref.shape), "ref_l2": ref_l2, "cand_l2": float(np.linalg.norm(cand)), "delta_l2": float(np.linalg.norm(d)), "delta_rms": float(np.sqrt(np.mean(np.square(d)))), "delta_abs_max": float(np.max(np.abs(d))), "delta_relative_l2": float(np.linalg.norm(d) / ref_l2) if ref_l2 > 0 else None, "numeric_zero_delta_float64": bool(np.array_equal(ref, cand))}
    if ref_raw is not None and cand_raw is not None:
        out["dtype_ref"], out["dtype_cand"] = str(ref_raw.dtype), str(cand_raw.dtype)
        out["bit_exact"] = bytes_equal(ref_raw, cand_raw)
        out["comparison"] = BIT_EXACT_NOTE
    return out


def _head_bit_exact(ref: dict[str, Any], cand: dict[str, Any], sl: slice) -> bool:
    """Weight rows AND bias entries of a head: dtype + shape + bytes."""
    return bytes_equal(ref["_raw"]["pi.1.weight"][sl], cand["_raw"]["pi.1.weight"][sl]) and bytes_equal(ref["_raw"]["pi.1.bias"][sl], cand["_raw"]["pi.1.bias"][sl])


def parameter_drift(ref: dict[str, Any], cand: dict[str, Any]) -> dict[str, Any]:
    per_tensor = {k: _tensor_delta(ref[k], cand[k], ref["_raw"][k], cand["_raw"][k]) for k in CANONICAL_ACTOR_KEYS}
    cat = lambda a: np.concatenate([np.asarray(a[k]).ravel() for k in CANONICAL_ACTOR_KEYS])  # noqa: E731
    head = lambda a, sl: np.concatenate([np.asarray(a["pi.1.weight"])[sl].ravel(), np.asarray(a["pi.1.bias"])[sl]])  # noqa: E731
    mean_sl, ls_sl = slice(0, ACTION_DIM), slice(ACTION_DIM, 2 * ACTION_DIM)
    g = _tensor_delta(cat(ref), cat(cand))
    g["bit_exact"] = all(per_tensor[k]["bit_exact"] for k in CANONICAL_ACTOR_KEYS)
    g["dtypes_match"] = all(per_tensor[k]["dtype_ref"] == per_tensor[k]["dtype_cand"] for k in CANONICAL_ACTOR_KEYS)
    g["comparison"] = BIT_EXACT_NOTE
    mean_head = _tensor_delta(head(ref, mean_sl), head(cand, mean_sl))
    mean_head["bit_exact"] = _head_bit_exact(ref, cand, mean_sl)
    mean_head["comparison"] = "weight rows 0-1 and bias 0-1: " + BIT_EXACT_NOTE
    ls_head = _tensor_delta(head(ref, ls_sl), head(cand, ls_sl))
    ls_head["bit_exact"] = _head_bit_exact(ref, cand, ls_sl)
    ls_head["comparison"] = "weight rows 2-3 and bias 2-3: " + BIT_EXACT_NOTE
    ls_head.update({"ref_rows_all_zero": bool(np.all(ref["pi.1.weight"][ls_sl] == 0.0)), "cand_rows_all_zero": bool(np.all(cand["pi.1.weight"][ls_sl] == 0.0)), "ref_bias": ref["pi.1.bias"][ls_sl].tolist(), "cand_bias": cand["pi.1.bias"][ls_sl].tolist()})
    return {"global": g, "per_tensor": per_tensor, "mean_head": mean_head, "logstd_head": ls_head, "note": "pi.1 rows 0-1 (+bias) = mean head; rows 2-3 (+bias) = log-std head; all-zero log-std rows = constant sigma = exp(bias); " + BIT_EXACT_NOTE}


def kl_diag_gauss(mu_p: np.ndarray, logstd_p: np.ndarray, mu_q: np.ndarray, logstd_q: np.ndarray) -> np.ndarray:
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            var_p = np.exp(2.0 * logstd_p)
            var_q = np.exp(2.0 * logstd_q)
            kl = np.sum(logstd_q - logstd_p + (var_p + np.square(mu_p - mu_q)) / (2.0 * var_q) - 0.5, axis=1)
    except FloatingPointError as exc:
        raise DriftError(f"KL evaluation overflow/invalid: {exc}") from exc
    if not np.all(np.isfinite(kl)):
        raise DriftError("non-finite KL values")
    return kl


def _stats(x: np.ndarray) -> dict[str, float]:
    x = np.asarray(x, dtype=np.float64)
    if not np.all(np.isfinite(x)):
        raise DriftError("non-finite metric values")
    return {"mean": float(np.mean(x)), "min": float(np.min(x)), "max": float(np.max(x)), "p50": float(np.percentile(x, 50)), "p95": float(np.percentile(x, 95)), "p99": float(np.percentile(x, 99)), "rms": float(np.sqrt(np.mean(np.square(x))))}


def _delta_stats(d: np.ndarray) -> dict[str, Any]:
    return {"rmse": float(np.sqrt(np.mean(np.square(d)))), "mae": float(np.mean(np.abs(d))), "bias_signed_mean": float(np.mean(d)), "abs_max": float(np.max(np.abs(d))), "p99_abs": float(np.percentile(np.abs(d), 99)),
            "per_action": {"rmse": np.sqrt(np.mean(np.square(d), axis=0)).tolist(), "mae": np.mean(np.abs(d), axis=0).tolist(), "bias_signed_mean": np.mean(d, axis=0).tolist(), "abs_max": np.max(np.abs(d), axis=0).tolist(), "p99_abs": np.percentile(np.abs(d), 99, axis=0).tolist()}}


def _sigma(logstd: np.ndarray) -> np.ndarray:
    try:
        with np.errstate(over="raise", invalid="raise"):
            s = np.exp(logstd)
    except FloatingPointError as exc:
        raise DriftError(f"sigma overflow: {exc}") from exc
    if not np.all(np.isfinite(s)) or np.any(s <= 0):
        raise DriftError("non-finite or non-positive sigma")
    return s


def row_metrics(ref: dict[str, Any], cand: dict[str, Any], obs: np.ndarray) -> dict[str, np.ndarray]:
    lr, lc = actor_logits(ref, obs), actor_logits(cand, obs)
    mu_r, ls_r, mu_c, ls_c = lr[:, :ACTION_DIM], lr[:, ACTION_DIM:], lc[:, :ACTION_DIM], lc[:, ACTION_DIM:]
    return {"mu_ref": mu_r, "mu_cand": mu_c, "logstd_ref": ls_r, "logstd_cand": ls_c, "sigma_ref": _sigma(ls_r), "sigma_cand": _sigma(ls_c), "delta_mu": mu_c - mu_r, "delta_logstd": ls_c - ls_r,
            "delta_applied": np.clip(mu_c, -1.0, 1.0) - np.clip(mu_r, -1.0, 1.0), "kl_h0_to_candidate": kl_diag_gauss(mu_r, ls_r, mu_c, ls_c), "kl_candidate_to_h0": kl_diag_gauss(mu_c, ls_c, mu_r, ls_r)}


def aggregate(rm: dict[str, np.ndarray], mask: np.ndarray | None = None) -> dict[str, Any]:
    sel = slice(None) if mask is None else mask
    d, da = rm["delta_mu"][sel], rm["delta_applied"][sel]
    kl1, kl2 = rm["kl_h0_to_candidate"][sel], rm["kl_candidate_to_h0"][sel]
    sr, sc = rm["sigma_ref"][sel], rm["sigma_cand"][sel]
    out = {"rows": int(d.shape[0]), "delta_mu": _delta_stats(d), "delta_applied_clipped": _delta_stats(da), "delta_mu_over_sigma_ref": {"rms_per_action": np.sqrt(np.mean(np.square(d / sr), axis=0)).tolist(), "p99_abs_per_action": np.percentile(np.abs(d / sr), 99, axis=0).tolist()},
           "delta_logstd": {"rms_per_action": np.sqrt(np.mean(np.square(rm["delta_logstd"][sel]), axis=0)).tolist(), "abs_max": float(np.max(np.abs(rm["delta_logstd"][sel])))},
           "sigma_ref_per_action": {"min": np.min(sr, axis=0).tolist(), "mean": np.mean(sr, axis=0).tolist(), "max": np.max(sr, axis=0).tolist()}, "sigma_cand_per_action": {"min": np.min(sc, axis=0).tolist(), "mean": np.mean(sc, axis=0).tolist(), "max": np.max(sc, axis=0).tolist()},
           "kl_h0_to_candidate": _stats(kl1), "kl_candidate_to_h0": _stats(kl2), "kl_symmetric_jeffreys": _stats(kl1 + kl2)}
    assert_finite_metrics(out, nullable=())
    return out


def drift_pair(chain: str, ref_name: str, cand_name: str, obs_set: dict[str, Any], actors: dict[str, dict[str, Any]], *, pair_kind: str) -> dict[str, Any]:
    ref, cand = actors[ref_name], actors[cand_name]
    rm = row_metrics(ref, cand, obs_set["obs"])
    traces = obs_set["manifest"]["traces"]
    tidx = obs_set["trace_index"]
    per_trace = [{"job_id": t["job_id"], "candidate": t["candidate"], "family": t["family"], "start": t["start"], "seed": t["seed"], **aggregate(rm, tidx == t["index"])} for t in traces]
    origins: dict[str, list[int]] = {}
    for t in traces:
        origins.setdefault(f"{t['candidate']}__{t['family']}__{t['start']}", []).append(t["index"])
    per_origin = {k: aggregate(rm, np.isin(tidx, v)) for k, v in origins.items()}
    macro = {"delta_mu_rmse": float(np.mean([pt["delta_mu"]["rmse"] for pt in per_trace])), "delta_mu_mae": float(np.mean([pt["delta_mu"]["mae"] for pt in per_trace])), "delta_mu_abs_max": float(np.max([pt["delta_mu"]["abs_max"] for pt in per_trace])), "delta_applied_clipped_rmse": float(np.mean([pt["delta_applied_clipped"]["rmse"] for pt in per_trace]))}
    for dname in KL_DIRECTIONS:
        macro[f"{dname}_mean"] = float(np.mean([pt[dname]["mean"] for pt in per_trace]))
        macro[f"{dname}_p99"] = float(np.mean([pt[dname]["p99"] for pt in per_trace]))
        macro[f"{dname}_max"] = float(np.max([pt[dname]["max"] for pt in per_trace]))
    ls_ref_dep, ls_cand_dep = float(np.std(rm["logstd_ref"], axis=0).max()), float(np.std(rm["logstd_cand"], axis=0).max())
    return {"pair_kind": pair_kind, "chain": chain, "reference": ref_name, "candidate": cand_name, "reference_provenance": ref["_provenance"], "candidate_provenance": cand["_provenance"], "reference_actor_digest": actor_digest(ref), "candidate_actor_digest": actor_digest(cand),
            "parameters": parameter_drift(ref, cand), "sigma_state_dependence": {"reference_logstd_std_across_rows_max": ls_ref_dep, "candidate_logstd_std_across_rows_max": ls_cand_dep, "reference_constant_sigma": ls_ref_dep == 0.0, "candidate_constant_sigma": ls_cand_dep == 0.0},
            "row_weighted": aggregate(rm), "macro_per_trace": macro, "per_origin": per_origin, "per_trace": per_trace,
            "kl_note": "analytic KL of diagonal Gaussians evaluated on the frozen corpus (not Monte Carlo); h0_to_candidate = KL(reference || candidate); symmetric = Jeffreys sum of both directions; mu never clipped, applied-action delta reported separately with the runtime clip to [-1, 1]"}


# --- rendering and orchestration ----------------------------------------------------------------


def render_png(out_dir: Path, stamp: str, pairs: list[dict[str, Any]], rms: dict[tuple[str, str], dict[str, np.ndarray]]) -> dict[str, Any]:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    fig, axes = plt.subplots(len(pairs), 3, figsize=(15, 3.2 * len(pairs)), squeeze=False)
    for i, p in enumerate(pairs):
        rm = rms[(p["reference"], p["candidate"])]
        axes[i][0].hist(rm["delta_mu"][:, 0], bins=60, alpha=0.6, label="knee")
        axes[i][0].hist(rm["delta_mu"][:, 1], bins=60, alpha=0.6, label="ankle")
        axes[i][0].set_title(f"[{p['pair_kind']}] {p['chain']}: {p['reference']} -> {p['candidate']}  delta mu ({rm['delta_mu'].shape[0]} rows)", fontsize=8)
        axes[i][0].legend(fontsize=7)
        axes[i][1].hist(np.log10(np.maximum(rm["kl_h0_to_candidate"], 1e-12)), bins=60, alpha=0.7, label="KL(ref||cand)")
        axes[i][1].hist(np.log10(np.maximum(rm["kl_candidate_to_h0"], 1e-12)), bins=60, alpha=0.5, label="KL(cand||ref)")
        axes[i][1].set_title("log10 analytic KL per row", fontsize=8)
        axes[i][1].legend(fontsize=7)
        for j, nm in enumerate(("knee", "ankle")):
            axes[i][2].plot(rm["sigma_ref"][:, j], lw=0.5, label=f"sigma_ref {nm}")
            axes[i][2].plot(rm["sigma_cand"][:, j], lw=0.5, label=f"sigma_cand {nm}")
        axes[i][2].set_title("sigma(s) along the corpus rows", fontsize=8)
        axes[i][2].legend(fontsize=6)
    fig.tight_layout()
    png = out_dir / f"actor_drift_{stamp}.png"
    if png.exists() or png.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing artefact: {png}")
    fig.savefig(png, dpi=110, metadata={"Software": "f0_actor_drift"})
    plt.close(fig)
    return {"path": png.name, "sha256": C.sha256_file(png), "byte_stable_cross_host": False}


def render_markdown(payload: dict[str, Any], stamp: str) -> str:
    f = lambda v, nd=4: f"{v:.{nd}g}" if isinstance(v, float) else str(v)  # noqa: E731
    man = payload["observation_set"]["manifest"]
    lines = [f"# F0.8 — Deriva actor su corpus congelato di osservazioni — stamp {stamp}", "", f"Corpus: {man['row_count']} righe da {man['trace_count']} trace (runtime {man['runtime']}, 35D, feature digest `{man['feature_names_sha256'][:12]}…`, NPZ `{payload['observation_set']['npz_sha256'][:12]}…`). {CORPUS_NOTE}. Analisi sorgente `{payload['source_analysis']['sha256'][:12]}…`. Forward pass validato su {len(payload['forward_pass_checks'])} trace (errore max {max(c['max_abs_error'] for c in payload['forward_pass_checks']):.2e}, confronto a tolleranza float32/float64). Input consumati: {len(payload['inputs_snapshot_sha256'])} file; ogni lettura è byte-bound al digest PRE e verificata POST (ledger, non cache).", "", payload["kl_note"], "", BIT_EXACT_NOTE, ""]
    for kind, title in (("primary_h0_to_candidate", "## Coppie primarie H0 → best/last"), ("secondary_best_to_last", "## Diagnostica secondaria best → last (stesso corpus)")):
        rows = [p for p in payload["pairs"] if p["pair_kind"] == kind]
        lines += [title, "", C.md_table(["catena", "coppia", "digest ref / cand", "Δ globale L2 (rel) [bit-exact]", "Δ mean head L2 (bit-exact)", "Δ logstd head L2 (bit-exact)", "Δμ RMSE / MAE / bias (row)", "Δμ RMSE macro", "Δμ |max| / p99", "Δμ/σ_ref RMS knee/ankle", "σ_ref media", "σ_cand media", "KL ref‖c media / p99 / max", "KL c‖ref media / p99 / max", "KL sym media", "σ costante ref/cand"], [[
            p["chain"], f"{p['reference']} → {p['candidate']}", f"{p['reference_actor_digest'][:10]} / {p['candidate_actor_digest'][:10]}", f"{f(p['parameters']['global']['delta_l2'])} ({f(p['parameters']['global']['delta_relative_l2'])}) [{p['parameters']['global']['bit_exact']}]", f"{f(p['parameters']['mean_head']['delta_l2'])} ({p['parameters']['mean_head']['bit_exact']})", f"{f(p['parameters']['logstd_head']['delta_l2'])} ({p['parameters']['logstd_head']['bit_exact']})",
            f"{f(p['row_weighted']['delta_mu']['rmse'])} / {f(p['row_weighted']['delta_mu']['mae'])} / {f(p['row_weighted']['delta_mu']['bias_signed_mean'])}", f(p["macro_per_trace"]["delta_mu_rmse"]), f"{f(p['row_weighted']['delta_mu']['abs_max'])} / {f(p['row_weighted']['delta_mu']['p99_abs'])}", "/".join(f(x, 3) for x in p["row_weighted"]["delta_mu_over_sigma_ref"]["rms_per_action"]), "/".join(f(x, 3) for x in p["row_weighted"]["sigma_ref_per_action"]["mean"]), "/".join(f(x, 3) for x in p["row_weighted"]["sigma_cand_per_action"]["mean"]),
            f"{f(p['row_weighted']['kl_h0_to_candidate']['mean'])} / {f(p['row_weighted']['kl_h0_to_candidate']['p99'])} / {f(p['row_weighted']['kl_h0_to_candidate']['max'])}", f"{f(p['row_weighted']['kl_candidate_to_h0']['mean'])} / {f(p['row_weighted']['kl_candidate_to_h0']['p99'])} / {f(p['row_weighted']['kl_candidate_to_h0']['max'])}", f(p["row_weighted"]["kl_symmetric_jeffreys"]["mean"]), f"{p['sigma_state_dependence']['reference_constant_sigma']}/{p['sigma_state_dependence']['candidate_constant_sigma']}"] for p in rows]), ""]
    origins = sorted({o for p in payload["pairs"] for o in p["per_origin"]})
    lines += ["## Per origine (candidato__famiglia__start): Δμ RMSE / KL ref‖cand media", "", C.md_table(["origine", *[f"{p['reference']}→{p['candidate']}" for p in payload["pairs"]]], [[o, *[f"{f(p['per_origin'][o]['delta_mu']['rmse'])} / {f(p['per_origin'][o]['kl_h0_to_candidate']['mean'])}" for p in payload["pairs"]]] for o in origins]), "", "Gate: tecnico/provenienza (ledger con snapshot PRE prima di ogni lettura computazionale, ogni lettura byte-bound al digest PRE, set invariato dopo il caricamento, dopo il calcolo e subito prima della pubblicazione; forward pass finito e coerente su 24 trace; ABI 35 esatta; receipt verificate e legate ai byte consumati; corpus congelato per SHA con range contigui; moduli con digest dei byte consumati; publish no-replace con sidecar unico); nessuna soglia di deriva 'buona/cattiva' definita.", "", "Perimetro della garanzia: ogni lettura degli input DATI viene riaperta e legata ai byte PRE, seguita dai controlli POST; il codice Python è importato prima del ledger, quindi gli hash dei sorgenti attestano la provenienza su disco, non il bytecode già caricato (nessuna pretesa di assenza di TOCTOU sull'esecuzione del codice)."]
    return "\n".join(lines)


def run(stamp: str | None = None, *, out_root: Path | None = None, python_exe: str | None = None) -> dict[str, Any]:
    stamp = ART.validate_stamp(stamp or time.strftime("%Y%m%d_%H%M%S"))
    out_root = Path(out_root) if out_root is not None else C.OUT_DRIFT
    final_dir = out_root / stamp
    if final_dir.exists() or final_dir.is_symlink():
        raise FileExistsError(f"refusing to reuse existing drift directory: {final_dir}")
    # --- discovery (pure registry projection: no job byte is read before PRE) ------------------------------
    specs = corpus_specs()
    expected = expected_corpus_identity()
    records = [M.static_job_record(s) for s in specs]
    names = sorted({n for _, a, b in (*PRIMARY_PAIRS, *SECONDARY_PAIRS) for n in (a, b)})
    analysis_path = C.OUT_METRICS / FINAL_ANALYSIS_FILE
    manifest_path = Path(C.CANDIDATES["B0820_H0"]["actor_feature_manifest"])
    code_paths = [HERE / n for n in CODE_INPUTS] + [C.BASELINE_DIR / n for n in RUNTIME_CODE_INPUTS]
    declared: list[Path] = [analysis_path, manifest_path, *code_paths]
    for rec in records:
        declared += [C.REPO / rec["output_dir"] / n for n in (M.RECEIPT_FILE, M.SUMMARY_FILE, M.TRACE_FILE)]
    for n in names:
        declared += [C.CANDIDATES[n]["module"] / f for f in MODULE_FILES]
    ledger = ART.InputLedger()
    before = ledger.declare(declared)  # PRE snapshot: before any computational read
    interpreter = ART.interpreter_provenance()
    code_digests = {C.rel(p): ART.sha256_bytes(ledger.read_bytes(p)) for p in code_paths}  # code consumed, hash-bound
    # --- load through the ledger (every read re-opened, digest == PRE) -------------------------------------
    analysis, analysis_ref = load_final_analysis(analysis_path, reader=ledger)
    try:
        analysis_jobs = M.frozen_analysis_index(analysis)
    except RuntimeError as exc:
        raise DriftError(str(exc)) from exc
    if ledger.digest(manifest_path) != C.ACTOR_MANIFEST_35_SHA256:
        raise DriftError("35D feature manifest digest mismatch with the registry pin")
    manifest35 = ledger.read_json(manifest_path)
    names35 = manifest35.get("actor_feature_names") if isinstance(manifest35, dict) else None
    if not isinstance(names35, list) or len(names35) != OBS_WIDTH or len(set(names35)) != OBS_WIDTH or not all(isinstance(n, str) and n for n in names35):
        raise DriftError("35D feature manifest malformed")
    module_digests = {n: ledger.digest(C.CANDIDATES[n]["module"] / MODULE_FILES[0]) for n in names}
    obs_set = build_observation_set(jobs=records, expected=expected, analysis_ref=analysis_ref, analysis_jobs=analysis_jobs, verify=None, feature_manifest=names35, reader=ledger, module_digests=module_digests)
    actors = {n: load_actor(C.CANDIDATES[n]["module"], reader=ledger) for n in names}
    for n in names:
        if actors[n]["_provenance"]["files_sha256"][MODULE_FILES[0]] != module_digests[n]:
            raise DriftError(f"module bytes consumed for {n} differ from the PRE snapshot")
    ledger.assert_unchanged("after loading (consumed set re-hashed)")
    # --- compute --------------------------------------------------------------------------------------------
    checks = [{"candidate": t["candidate"], "job_id": t["job_id"], **verify_forward_pass(actors[t["candidate"]], C.REPO / t["output_dir"], t["action_selection"], reader=ledger)} for t in obs_set["manifest"]["traces"]]
    if len(checks) != 24:
        raise DriftError("forward pass must be validated on all 24 corpus traces")
    pairs, rms = [], {}
    for kind, table in (("primary_h0_to_candidate", PRIMARY_PAIRS), ("secondary_best_to_last", SECONDARY_PAIRS)):
        for chain, a, b in table:
            pairs.append(drift_pair(chain, a, b, obs_set, actors, pair_kind=kind))
            rms[(a, b)] = row_metrics(actors[a], actors[b], obs_set["obs"])
    after_compute = ledger.assert_unchanged("after computation")
    ledger.assert_all_read()
    script_sha = code_digests[C.rel(HERE / "f0_actor_drift.py")]
    git = C.git_snapshot()

    # --- publish (staging, single sidecar, staged verification, re-check, no-replace rename, final verification)
    def build(staging: Path) -> dict[str, Any]:
        set_files = write_observation_set(staging, stamp, obs_set)
        png = render_png(staging, stamp, pairs, rms)
        payload = {"schema_version": 4, "stamp": stamp, "generated_at_utc": C.utc_now(), "git": git, "script_sha256": script_sha, "interpreter": interpreter, "code_inputs_sha256": code_digests, "feature_manifest": {"path": C.rel(manifest_path), "sha256": before[C.rel(manifest_path)], "count": len(names35)},
                   "source_analysis": analysis_ref, "observation_set": {**set_files, "manifest": obs_set["manifest"]}, "corpus_note": CORPUS_NOTE, "forward_pass_checks": checks, "actors": {n: actors[n]["_provenance"] for n in names}, "module_digests_pre": module_digests, "pairs": pairs, "png": png, "kl_note": pairs[0]["kl_note"], "bit_exact_note": BIT_EXACT_NOTE,
                   "excluded": {"V26_39D": "ABI 39 != 35: transplant, not drift"}, "gate": "technical/provenance only; no drift threshold defined", "inputs_snapshot_sha256": before, "inputs_unchanged_after_computation": after_compute == before, "ledger_contract": ART.__doc__.strip().splitlines()[0]}
        ART.write_json_strict(staging / f"actor_drift_{stamp}.json", payload)
        C.write_text(staging / f"actor_drift_{stamp}.md", render_markdown(payload, stamp))
        return {"kind": "f0_actor_drift", "source_analysis": analysis_ref, "script_sha256": script_sha, "git_head": git["head"], "inputs_unchanged": True, "interpreter": interpreter, "consumed_inputs": len(before), "inputs_snapshot_sha256": before}

    published = ART.publish_artifact(final_dir, anchor=out_root, stamp=stamp, build=build, before_publish=lambda: ledger.assert_unchanged("immediately before publish"))
    return {**published, "pairs": len(pairs), "rows": obs_set["manifest"]["row_count"], "forward_checks": len(checks), "consumed_inputs": len(before)}


def exit_code_for(result: dict[str, Any]) -> int:
    return 0 if isinstance(result, dict) and result.get("artifact", {}).get("ok") is True else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="f0_actor_drift.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args(argv)
    try:
        result = run()
    except Exception as exc:  # noqa: BLE001 - any failure of the fail-closed protocol is a nonzero exit
        print(f"[actor-drift] FAILED ({type(exc).__name__}): {exc}", file=sys.stderr)
        return 1
    code = exit_code_for(result)
    print(f"[actor-drift] published {result['final_dir']}; pairs={result['pairs']} rows={result['rows']} forward_checks={result['forward_checks']} consumed_inputs={result['consumed_inputs']} artifact_ok={result['artifact']['ok']} exit={code}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
