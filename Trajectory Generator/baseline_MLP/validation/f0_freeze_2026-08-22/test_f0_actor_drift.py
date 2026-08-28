"""Self-test of f0_actor_drift + f0_artifacts. TEMP-ONLY by default (no real artefact is read); ``--real`` adds the
opt-in read-only smoke on the real traces/modules.

Synthetic: analytic KL (sigma doubled), known delta mu (bias shift) with RMSE/MAE/bias/p99, constant vs
state-dependent sigma, secondary pair kind, BIT-EXACT = dtype + shape + bytes (float32 vs float64 copy and SIGNED
ZERO -0.0/+0.0 are numerically equal but NOT bit-exact, per tensor and per head weight/bias), refusals (39-wide
module, incoherent shapes, encoder alias value/dtype, non-finite weights, integer dtype, overflow in sigma/KL,
action arrays (n,1)/non-finite/non-numeric), corpus: exact 24-identity set with receipts BOUND to the consumed
bytes (summary/trace digests, verified receipt equality, module_state_sha256), contiguous cursor ranges, refusals
(sealed seed, runtime, candidate, family, action_selection, duplicate, count, feature names, width, time, mean
incoherence, (n,1), step gap, tampered summary, foreign receipt, verifier failure, wrong module digest), NPZ round
trip and refusal of tampered digests / row gaps / overlaps / indices / time, InputLedger (no-follow chain: ancestor
symlink at declare, undeclared read, unread input, every-read digest != PRE, leaf swapped with a symlink of
identical content, ancestor retargeted to a symlink before read and before the POST snapshot, declare twice),
strict JSON / .sto parsing, publish protocol (stamps, ancestor symlink before mkdir, race with a competitor
directory, no-replace primitive on an existing empty directory, build failure / empty build cleanup, sidecar-like
file AND directory, special node, tamper pre-rename and post-rename of files and of the sidecar PROVENANCE with
valid types (deep/byte-exact binding), schema mutations, NaN, staging path, nonzero exit code).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pickle
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_actor_drift as D  # noqa: E402
import f0_artifacts as ART  # noqa: E402
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

PASSED = 0
PROV = {"kind": "test", "source_analysis": {"path": "x.json", "sha256": "0" * 64}, "script_sha256": "1" * 64, "git_head": "h", "inputs_unchanged": True, "interpreter": {"x": 1}, "consumed_inputs": 0}
MODSHA = {"B0820_H0": "a" * 64, "JUL_H0": "b" * 64}


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def expect(fn, exc_types, label: str, needle: str | None = None):
    try:
        fn()
    except exc_types as exc:
        if needle and needle not in str(exc):
            raise AssertionError(f"{label}: error does not mention {needle!r}: {exc}")
        ok(f"{label} -> {type(exc).__name__}")
        return exc
    raise AssertionError(f"{label}: expected {exc_types}")


def tmp_base() -> str:
    preferred = Path("/private/tmp")
    if preferred.is_dir() and os.access(preferred, os.W_OK):
        return str(preferred)
    return tempfile.gettempdir()


def synth_state(rng: np.random.Generator, *, width: int = 35, hidden: int = 8, logstd_rows_zero: bool = True, logstd_bias: float = math.log(0.005), mean_bias_shift: float = 0.0, dtype=np.float32) -> dict:
    w1 = rng.normal(0, 0.3, (hidden, width)); b1 = rng.normal(0, 0.1, hidden)
    w2 = rng.normal(0, 0.3, (hidden, hidden)); b2 = rng.normal(0, 0.1, hidden)
    w3 = np.zeros((4, hidden)); w3[:2] = rng.normal(0, 0.3, (2, hidden))
    if not logstd_rows_zero:
        w3[2:] = rng.normal(0, 0.1, (2, hidden))
    b3 = np.array([mean_bias_shift, -mean_bias_shift, logstd_bias, logstd_bias])
    state = {"pi.0.0.weight": w1.astype(dtype), "pi.0.0.bias": b1.astype(dtype), "pi.0.2.weight": w2.astype(dtype), "pi.0.2.bias": b2.astype(dtype), "pi.1.weight": w3.astype(dtype), "pi.1.bias": b3.astype(dtype)}
    state.update({"pi_encoder.0.weight": state["pi.0.0.weight"].copy(), "pi_encoder.0.bias": state["pi.0.0.bias"].copy(), "pi_encoder.2.weight": state["pi.0.2.weight"].copy(), "pi_encoder.2.bias": state["pi.0.2.bias"].copy()})
    return state


def write_module(dir_: Path, state: dict) -> Path:
    dir_.mkdir(parents=True, exist_ok=True)
    (dir_ / "module_state.pkl").write_bytes(pickle.dumps(state))
    (dir_ / "metadata.json").write_text("{}", encoding="utf-8")
    return dir_


def simple_build(text: str = "a", extra=None):
    def build(staging: Path) -> dict:
        (staging / "a.txt").write_text(text, encoding="utf-8")
        if extra:
            extra(staging)
        return dict(PROV)

    return build


class CountingReader(ART.DirectReader):
    def __init__(self) -> None:
        self.reads: list[Path] = []

    def read_bytes(self, path: Path) -> bytes:
        self.reads.append(Path(path))
        return super().read_bytes(path)


def synthetic(tmp: Path) -> None:
    rng = np.random.default_rng(7)
    obs = rng.normal(0, 1, (300, 35))
    manifest = {"row_count": 300, "width": 35, "traces": [{"index": i, "job_id": f"job{i}", "candidate": "B0820_H0", "family": "stoch", "start": "nominal", "seed": 123 + i, "row_count": 100} for i in range(3)]}
    obs_set = {"obs": obs, "trace_index": np.repeat(np.arange(3, dtype=np.int32), 100), "step_index": np.tile(np.arange(1, 101, dtype=np.int32), 3), "time_s": np.arange(300) * 0.01, "manifest": manifest}
    ref_state = synth_state(np.random.default_rng(1))
    actors = {"R": D.load_actor(write_module(tmp / "ref", ref_state))}
    assert actors["R"]["_provenance"]["dtypes"]["pi.1.weight"] == "float32" and actors["R"]["_provenance"]["shapes"]["pi.0.0.weight"] == [8, 35] and actors["R"]["_provenance"]["files_sha256"]["module_state.pkl"] == C.sha256_file(tmp / "ref" / "module_state.pkl") and actors["R"]["_provenance"]["files_sha256"]["class_and_ctor_args.pkl"] is None
    same = D.drift_pair("t", "R", "R", obs_set, actors, pair_kind="primary_h0_to_candidate")
    assert same["parameters"]["global"]["bit_exact"] and same["parameters"]["global"]["dtypes_match"] and same["parameters"]["mean_head"]["bit_exact"] and same["parameters"]["logstd_head"]["bit_exact"] and same["row_weighted"]["delta_mu"]["rmse"] == 0.0 and same["row_weighted"]["kl_h0_to_candidate"]["max"] == 0.0
    ok("identical modules: bit-exact (dtype+shape+bytes) globally and per head, zero delta mu, zero KL; provenance records dtypes/shapes/consumed-bytes digests")
    s64 = {k: v.astype(np.float64) for k, v in ref_state.items()}
    actors["R64"] = D.load_actor(write_module(tmp / "ref64", s64))
    p64 = D.drift_pair("t", "R", "R64", obs_set, actors, pair_kind="primary_h0_to_candidate")
    assert p64["parameters"]["global"]["bit_exact"] is False and p64["parameters"]["global"]["dtypes_match"] is False and p64["parameters"]["global"]["numeric_zero_delta_float64"] is True and p64["parameters"]["per_tensor"]["pi.1.weight"]["dtype_cand"] == "float64" and p64["reference_actor_digest"] != p64["candidate_actor_digest"] and p64["parameters"]["mean_head"]["bit_exact"] is False
    ok("dtype mismatch (float32 vs float64 copy): numeric delta zero but bit_exact False everywhere, digests differ (never silently equated)")
    # signed zero: numerically equal (np.array_equal True) but different bytes
    sz = {k: v.copy() for k, v in ref_state.items()}
    assert sz["pi.1.weight"][2, 0] == 0.0 and sz["pi.1.bias"][0] == 0.0
    sz["pi.1.weight"][2, 0] = np.float32(-0.0)
    actors["SZ"] = D.load_actor(write_module(tmp / "sz", sz))
    psz = D.drift_pair("t", "R", "SZ", obs_set, actors, pair_kind="primary_h0_to_candidate")
    assert np.array_equal(ref_state["pi.1.weight"], sz["pi.1.weight"]) and not D.bytes_equal(ref_state["pi.1.weight"], sz["pi.1.weight"])
    assert psz["parameters"]["per_tensor"]["pi.1.weight"]["numeric_zero_delta_float64"] is True and psz["parameters"]["per_tensor"]["pi.1.weight"]["bit_exact"] is False and psz["parameters"]["global"]["bit_exact"] is False and psz["parameters"]["logstd_head"]["bit_exact"] is False and psz["parameters"]["mean_head"]["bit_exact"] is True and psz["reference_actor_digest"] != psz["candidate_actor_digest"] and psz["row_weighted"]["delta_mu"]["rmse"] == 0.0
    sb = {k: v.copy() for k, v in ref_state.items()}
    sb["pi.1.bias"][0] = np.float32(-0.0)
    actors["SB"] = D.load_actor(write_module(tmp / "sb", sb))
    psb = D.drift_pair("t", "R", "SB", obs_set, actors, pair_kind="primary_h0_to_candidate")
    assert psb["parameters"]["mean_head"]["bit_exact"] is False and psb["parameters"]["logstd_head"]["bit_exact"] is True and psb["parameters"]["per_tensor"]["pi.1.bias"]["bit_exact"] is False and psb["parameters"]["per_tensor"]["pi.1.bias"]["numeric_zero_delta_float64"] is True
    ok("signed zero (-0.0 vs +0.0): np.array_equal True but bit_exact False (tensor, global, log-std head via weight; mean head via bias), digests differ, numeric delta zero")
    s2 = dict(ref_state); b2 = ref_state["pi.1.bias"].copy(); b2[2:] = (b2[2:] + np.float32(math.log(2.0))).astype(np.float32); s2["pi.1.bias"] = b2  # only the log-std entries touched (adding 0.0 to -0.0 would flip bytes)
    actors["S"] = D.load_actor(write_module(tmp / "sig2", s2))
    p = D.drift_pair("t", "R", "S", obs_set, actors, pair_kind="secondary_best_to_last")
    kl_fwd = 2 * (math.log(2.0) + 1.0 / 8.0 - 0.5)
    kl_bwd = 2 * (-math.log(2.0) + 4.0 / 2.0 - 0.5)
    rw = p["row_weighted"]
    assert abs(rw["kl_h0_to_candidate"]["mean"] - kl_fwd) < 1e-6 and abs(rw["kl_h0_to_candidate"]["p99"] - kl_fwd) < 1e-6 and abs(rw["kl_candidate_to_h0"]["max"] - kl_bwd) < 1e-6 and abs(rw["kl_symmetric_jeffreys"]["p50"] - (kl_fwd + kl_bwd)) < 1e-6
    assert rw["delta_mu"]["rmse"] == 0.0 and p["sigma_state_dependence"]["reference_constant_sigma"] and p["parameters"]["logstd_head"]["bit_exact"] is False and p["parameters"]["mean_head"]["bit_exact"] is True and p["pair_kind"] == "secondary_best_to_last"
    ok("analytic KL with sigma doubled (mean/p50/p99/max), Jeffreys, constant sigma, head-level bit-exact, secondary pair kind")
    s3 = dict(ref_state); s3["pi.1.bias"] = (ref_state["pi.1.bias"] + np.array([0.2, -0.2, 0, 0], dtype=np.float32)).astype(np.float32)
    actors["Mv"] = D.load_actor(write_module(tmp / "shift", s3))
    p = D.drift_pair("t", "R", "Mv", obs_set, actors, pair_kind="primary_h0_to_candidate")
    rw, shift = p["row_weighted"], float(np.float32(0.2))
    assert abs(rw["delta_mu"]["per_action"]["bias_signed_mean"][0] - shift) < 1e-6 and abs(rw["delta_mu"]["per_action"]["bias_signed_mean"][1] + shift) < 1e-6 and abs(rw["delta_mu"]["rmse"] - shift) < 1e-6 and abs(rw["delta_mu"]["mae"] - shift) < 1e-6 and abs(rw["delta_mu"]["abs_max"] - shift) < 1e-6 and abs(rw["delta_mu"]["p99_abs"] - shift) < 1e-6 and abs(rw["delta_mu"]["bias_signed_mean"]) < 1e-6
    expected_kl = 2 * (shift ** 2) / (2 * 0.005 ** 2)
    assert abs(rw["kl_h0_to_candidate"]["mean"] - expected_kl) < 1e-3 * expected_kl and abs(rw["delta_mu_over_sigma_ref"]["rms_per_action"][0] - shift / 0.005) < 1e-3 and len(p["per_trace"]) == 3 and set(p["per_origin"]) == {"B0820_H0__stoch__nominal"} and abs(p["macro_per_trace"]["delta_mu_mae"] - shift) < 1e-6 and "kl_h0_to_candidate_p99" in p["macro_per_trace"]
    ok("mean bias shift: delta mu RMSE/MAE/bias/abs max/p99 exact, KL = d^2/(2 sigma^2), delta/sigma, per-trace/per-origin/macro aggregates")
    actors["SD"] = D.load_actor(write_module(tmp / "sd", synth_state(np.random.default_rng(3), logstd_rows_zero=False)))
    assert D.drift_pair("t", "R", "SD", obs_set, actors, pair_kind="primary_h0_to_candidate")["sigma_state_dependence"]["candidate_constant_sigma"] is False
    ok("state-dependent log-std head detected")
    expect(lambda: D.load_actor(write_module(tmp / "w39", synth_state(np.random.default_rng(2), width=39))), D.DriftError, "39-wide module", "ABI mismatch")
    bad = dict(ref_state); bad["pi.0.2.weight"] = np.zeros((8, 7), dtype=np.float32)
    expect(lambda: D.load_actor(write_module(tmp / "shape", bad)), D.DriftError, "incoherent layer shapes", "incoherent tensor shapes")
    bad = dict(ref_state); bad["pi_encoder.0.weight"] = ref_state["pi_encoder.0.weight"] + 1
    expect(lambda: D.load_actor(write_module(tmp / "alias", bad)), D.DriftError, "encoder alias value mismatch", "alias")
    bad = dict(ref_state); bad["pi_encoder.0.weight"] = ref_state["pi_encoder.0.weight"].astype(np.float64)
    expect(lambda: D.load_actor(write_module(tmp / "alias64", bad)), D.DriftError, "encoder alias dtype mismatch", "alias")
    bad = dict(ref_state); bad["pi_encoder.0.bias"] = ref_state["pi_encoder.0.bias"].copy(); bad["pi_encoder.0.bias"][0] = np.float32(-0.0) if ref_state["pi_encoder.0.bias"][0] == 0 else -ref_state["pi_encoder.0.bias"][0]
    expect(lambda: D.load_actor(write_module(tmp / "aliasbytes", bad)), D.DriftError, "encoder alias bytes mismatch", "alias")
    bad = dict(ref_state); bad["pi.0.0.weight"] = ref_state["pi.0.0.weight"].copy(); bad["pi.0.0.weight"][0, 0] = np.nan; bad["pi_encoder.0.weight"] = bad["pi.0.0.weight"].copy()
    expect(lambda: D.load_actor(write_module(tmp / "nan", bad)), D.DriftError, "non-finite weights", "non-finite")
    bad = dict(ref_state); bad.pop("pi.1.bias")
    expect(lambda: D.load_actor(write_module(tmp / "missing", bad)), D.DriftError, "missing canonical key", "lacks canonical")
    bad = dict(ref_state); bad["pi.1.bias"] = ref_state["pi.1.bias"].astype(np.int32)
    expect(lambda: D.load_actor(write_module(tmp / "intdtype", bad)), D.DriftError, "integer dtype tensor", "not floating")
    lnk = tmp / "lnkmod"; lnk.mkdir(); (lnk / "module_state.pkl").symlink_to(tmp / "ref" / "module_state.pkl")
    expect(lambda: D.load_actor(lnk), D.DriftError, "symlinked module_state.pkl", "symlink")
    huge = dict(ref_state); huge["pi.1.bias"] = np.array([0, 0, 1000.0, 1000.0], dtype=np.float32)
    actors["H"] = D.load_actor(write_module(tmp / "huge", huge))
    expect(lambda: D.drift_pair("t", "R", "H", obs_set, actors, pair_kind="primary_h0_to_candidate"), D.DriftError, "sigma overflow (logstd 1000)", "overflow")
    expect(lambda: D.kl_diag_gauss(np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((2, 2)), np.full((2, 2), -400.0)), D.DriftError, "KL division overflow", "overflow")
    expect(lambda: D.actor_logits(actors["R"], np.zeros((3, 36))), D.DriftError, "36-wide observations vs 35 actor", "incompatible")
    assert D.action_array([{"step": 1, "raw_policy_action": [0.1, 0.2]}, {"step": 2, "raw_policy_action": [0.3, 0.4]}], "raw_policy_action", "t").shape == (2, 2)
    expect(lambda: D.action_array([{"step": 1, "raw_policy_action": [0.1]}], "raw_policy_action", "t"), D.DriftError, "action shape (n,1)", "list of 2")
    expect(lambda: D.action_array([{"step": 1, "raw_policy_action": [0.1, float("nan")]}], "raw_policy_action", "t"), D.DriftError, "non-finite action", "finite")
    expect(lambda: D.action_array([{"step": 1, "raw_policy_action": [0.1, "x"]}], "raw_policy_action", "t"), D.DriftError, "non-numeric action", "list of 2")
    ok("refusals: 39-wide ABI, incoherent shapes, encoder alias (value/dtype/bytes), non-finite weights, missing key, integer dtype, symlinked module, sigma/KL overflow, observation width, action arrays (n,1)/non-finite/non-numeric")
    # --- corpus builder on synthetic records with the REAL canonical identities (registry only, no real files) ----
    expected = D.expected_corpus_identity()
    names35 = [f"f{k}" for k in range(35)]

    def seal(d: Path, *, module_sha: str) -> None:
        receipt = {"schema_version": 5, "summary_sha256": C.sha256_file(d / M.SUMMARY_FILE), "trace_sha256": C.sha256_file(d / M.TRACE_FILE), "module_state_sha256": module_sha}
        (d / M.RECEIPT_FILE).write_text(json.dumps(receipt), encoding="utf-8")

    def job(exp: dict, **over) -> dict:
        d = tmp / "jobs" / exp["job_id"]
        d.mkdir(parents=True, exist_ok=True)
        stoch = exp["action_selection"] == "stochastic"
        (d / M.SUMMARY_FILE).write_text(json.dumps({"n_actor": 35, "actor_feature_names": names35, "action_selection": exp["action_selection"], "end_reason": "x"}))
        rows = [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.1 * s] * 35, "raw_policy_action": [0.0, 0.0], **({"policy_action_mean": [0.0, 0.0]} if stoch else {})} for s in range(5)]
        (d / M.TRACE_FILE).write_text(json.dumps(rows))
        seal(d, module_sha=MODSHA[exp["candidate"]])
        rec = dict(exp, output_dir=C.rel(d))
        rec.update(over)
        return rec

    verify = lambda rec, out_dir: json.loads((out_dir / M.RECEIPT_FILE).read_text())  # noqa: E731
    good = [job(e) for e in expected]
    counting = CountingReader()
    built = D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={"path": "x", "sha256": "x"}, reader=counting, module_digests=MODSHA)
    assert built["obs"].shape == (120, 35) and built["time_s"].shape == (120,) and built["manifest"]["traces"][5]["row_start"] == 25 and built["manifest"]["traces"][0]["forward_reference_field"] == "raw_policy_action" and built["manifest"]["traces"][3]["forward_reference_field"] == "policy_action_mean" and len(counting.reads) == 72
    assert all(built["manifest"]["traces"][i]["row_start"] == built["manifest"]["traces"][i - 1]["row_end"] for i in range(1, 24)) and built["manifest"]["traces"][-1]["row_end"] == 120 and built["manifest"]["traces"][0]["receipt_module_state_sha256"] == MODSHA["B0820_H0"]
    analysis_jobs = {}
    for rec in good:
        out = C.REPO / rec["output_dir"]
        receipt = json.loads((out / M.RECEIPT_FILE).read_text())
        analysis_jobs[rec["job_id"]] = {**rec, "receipt": receipt, "evidence": {
            "summary": {"path": C.rel(out / M.SUMMARY_FILE), "present": True, "valid": True, "sha256": C.sha256_file(out / M.SUMMARY_FILE)},
            "trace": {"path": C.rel(out / M.TRACE_FILE), "present": True, "valid": True, "sha256": C.sha256_file(out / M.TRACE_FILE)},
        }}
    bound = D.build_observation_set(jobs=good, expected=expected, analysis_jobs=analysis_jobs, verify=None, feature_manifest=names35, analysis_ref={}, module_digests=MODSHA)
    assert all(t["analysis_binding"]["mode"] == "pinned_analysis_exact_evidence" for t in bound["manifest"]["traces"])
    d0 = tmp / "jobs" / expected[0]["job_id"]
    changed = json.loads((d0 / M.TRACE_FILE).read_text()); changed[0]["raw_policy_action"][0] = 0.25
    (d0 / M.TRACE_FILE).write_text(json.dumps(changed))
    seal(d0, module_sha=MODSHA["B0820_H0"])
    expect(lambda: D.build_observation_set(jobs=good, expected=expected, analysis_jobs=analysis_jobs, verify=None, feature_manifest=names35, analysis_ref={}, module_digests=MODSHA), D.DriftError, "analysis binding refuses coherently re-sealed trace", "pinned analysis")
    good[0] = job(expected[0])
    ok("pinned-analysis binding: canonical identity, deep receipt projection and exact summary/trace evidence digests; coherently re-sealed trace refused")
    for label, idx, over in (("seed 126 (sealed)", 3, {"seed": 126}), ("runtime v2", 0, {"runtime": "v2_b0820"}), ("candidate V3_BEST", 0, {"candidate": "B0820_V3_BEST"}), ("family swapped", 0, {"family": "stoch"}), ("action_selection incoherent", 3, {"action_selection": "deterministic"}), ("duplicate job", 4, dict(expected[3]))):
        bad_jobs = list(good); bad_jobs[idx] = job(expected[idx], **over)
        expect(lambda b=bad_jobs: D.build_observation_set(jobs=b, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}), D.DriftError, f"corpus refuses {label}")
    expect(lambda: D.build_observation_set(jobs=good[:23], expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}), D.DriftError, "23 traces", "24")
    expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=[f"g{k}" for k in range(35)], analysis_ref={}), D.DriftError, "feature names differ", "ABI")
    d0 = tmp / "jobs" / expected[0]["job_id"]
    for label, rows in (("36-wide row", [{"step": 1, "time": 1.0, "actor_observation_vector_before": [0.0] * 36, "raw_policy_action": [0, 0]}]), ("non-monotone time", [{"step": s + 1, "time": 2.0 - 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0]} for s in range(3)]), ("deterministic with policy_action_mean", [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0], "policy_action_mean": [0, 0]} for s in range(3)]), ("action (n,1)", [{"step": 1, "time": 1.0, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0.0]}]), ("step gap", [{"step": s, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0]} for s in (1, 3)]), ("deterministic with partial policy_action_mean", [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0], **({"policy_action_mean": [0, 0]} if s == 1 else {})} for s in range(3)])):
        (d0 / M.TRACE_FILE).write_text(json.dumps(rows))
        seal(d0, module_sha=MODSHA["B0820_H0"])
        expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}, module_digests=MODSHA), D.DriftError, f"corpus refuses {label}")
    good[0] = job(expected[0])
    d3 = tmp / "jobs" / expected[3]["job_id"]  # stochastic corpus job: presence vector must be all-or-nothing
    for label, rows in (("stochastic with partial policy_action_mean", [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0], **({"policy_action_mean": [0, 0]} if s != 2 else {})} for s in range(5)]), ("stochastic with policy_action_mean None on one row", [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0], "policy_action_mean": ([0, 0] if s != 2 else None)} for s in range(5)]), ("stochastic with policy_action_mean absent everywhere", [{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0]} for s in range(5)])):
        (d3 / M.TRACE_FILE).write_text(json.dumps(rows))
        seal(d3, module_sha=MODSHA["B0820_H0"])
        expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}, module_digests=MODSHA), D.DriftError, f"corpus refuses {label}")
    good[3] = job(expected[3])
    assert D.mean_presence([{"policy_action_mean": [0, 0]}, {"policy_action_mean": [1, 1]}], "t") is True and D.mean_presence([{"x": 1}, {"y": 2}], "t") is False and D.mean_presence([{"policy_action_mean": None}, {"policy_action_mean": None}], "t") is False
    expect(lambda: D.mean_presence([{"policy_action_mean": [0, 0]}, {"x": 1}], "t"), D.DriftError, "presence vector: partial presence", "heterogeneous")
    expect(lambda: D.mean_presence([{"policy_action_mean": [0, 0]}, {"policy_action_mean": None}], "t"), D.DriftError, "presence vector: mixed list/null", "heterogeneous")
    actors_sd = D.load_actor(tmp / "ref")
    (d3 / M.TRACE_FILE).write_text(json.dumps([{"step": s + 1, "time": 1.0 + 0.01 * s, "actor_observation_vector_before": [0.0] * 35, "raw_policy_action": [0, 0], **({"policy_action_mean": [0, 0]} if s == 0 else {})} for s in range(3)]))
    expect(lambda: D.verify_forward_pass(actors_sd, d3, "stochastic"), D.DriftError, "forward pass refuses partial policy_action_mean", "heterogeneous")
    good[3] = job(expected[3])
    (d0 / M.SUMMARY_FILE).write_text((d0 / M.SUMMARY_FILE).read_text() + " ")  # summary tampered after the receipt was sealed
    expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}), D.DriftError, "summary bytes != receipt digest", "receipt digests")
    good[0] = job(expected[0])
    expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=lambda rec, out_dir: {"schema_version": 5}, feature_manifest=names35, analysis_ref={}), D.DriftError, "verifier returned a different receipt", "TOCTOU")

    def verify_fail(rec, out_dir):
        raise RuntimeError("receipt verification failed")

    expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify_fail, feature_manifest=names35, analysis_ref={}), RuntimeError, "verifier failure propagates", "verification failed")
    expect(lambda: D.build_observation_set(jobs=good, expected=expected, verify=verify, feature_manifest=names35, analysis_ref={}, module_digests={"B0820_H0": "c" * 64, "JUL_H0": MODSHA["JUL_H0"]}), D.DriftError, "receipt module digest != consumed module", "module_state_sha256")
    ok("corpus: exact 24-identity set, receipts bound to consumed bytes (72 reads), contiguous cursor ranges; refuses sealed seed, runtime, candidate, family, action_selection, duplicate, count, feature names, width, time, mean incoherence/partial presence/None (presence vector), (n,1), step gap, tampered summary, foreign receipt, verifier failure, wrong module digest")
    files = D.write_observation_set(tmp / "out", "t1", built)
    npz_path, man_path = tmp / "out" / files["npz"], tmp / "out" / files["manifest"]
    assert files["npz"] == "observation_set_t1.npz" and files["manifest"] == "observation_set_t1.json"
    loaded = D.load_observation_set(npz_path, man_path)
    assert np.array_equal(loaded["obs"], built["obs"]) and np.array_equal(loaded["time_s"], built["time_s"]) and loaded["manifest"]["npz_sha256"] == files["npz_sha256"]
    expect(lambda: D.write_observation_set(tmp / "out", "t1", built), FileExistsError, "observation set no-clobber")
    man_ok = json.loads(man_path.read_text())
    for label, mutate_manifest in (("row gap", lambda m: m["traces"][2].update(row_start=m["traces"][2]["row_start"] + 1)), ("row overlap", lambda m: m["traces"][2].update(row_start=m["traces"][2]["row_start"] - 1)), ("row_count wrong", lambda m: m["traces"][2].update(row_count=4)), ("trace_count wrong", lambda m: m.update(trace_count=23)), ("index wrong", lambda m: m["traces"][2].update(index=7)), ("t_end wrong", lambda m: m["traces"][2].update(t_end_s=m["traces"][2]["t_end_s"] + 1))):
        m = json.loads(json.dumps(man_ok)); mutate_manifest(m)
        alt_man = tmp / "out" / f"alt_{label.replace(' ', '_')}.json"
        alt_man.write_text(json.dumps(m))
        expect(lambda a=alt_man: D.load_observation_set(npz_path, a), D.DriftError, f"load refuses {label}")
    for label, mutate in (("time tampered", lambda d: d.update(time_s=d["time_s"] + 1.0)), ("trace_index tampered", lambda d: d.update(trace_index=np.roll(d["trace_index"], 1))), ("step_index tampered", lambda d: d.update(step_index=d["step_index"][::-1].copy())), ("obs tampered", lambda d: d.update(obs=d["obs"] * 2))):
        with np.load(npz_path, allow_pickle=False) as z:
            arrs = {k: z[k] for k in z.files}
        mutate(arrs)
        alt = tmp / "out" / f"alt_{label[:4]}.npz"
        np.savez(alt, **arrs)
        m = json.loads(json.dumps(man_ok)); m["npz_sha256"] = C.sha256_file(alt)
        alt_man = tmp / "out" / f"alt_{label[:4]}.json"
        alt_man.write_text(json.dumps(m))
        expect(lambda a=alt, mm=alt_man: D.load_observation_set(a, mm), D.DriftError, f"load refuses {label}")
    npz_path.write_bytes(b"tampered")
    expect(lambda: D.load_observation_set(npz_path, man_path), D.DriftError, "NPZ digest mismatch", "digest mismatch")
    ok("NPZ round trip with time_s; load refuses row gap/overlap, wrong counts/indices/bounds, tampered arrays/digest")
    # --- InputLedger: every read through a no-follow chain ----------------------------------------------------
    inp = tmp / "in" / "dir"
    inp.mkdir(parents=True)
    a, b = inp / "a.txt", inp / "b.txt"
    a.write_text("A"); b.write_text("B")
    (tmp / "in_link").symlink_to(tmp / "in", target_is_directory=True)
    expect(lambda: ART.InputLedger().declare([tmp / "in_link" / "dir" / "a.txt"]), ART.StagingError, "declare refuses an ancestor symlink")
    expect(lambda: ART.InputLedger().declare([Path("relative.txt")]), ART.StagingError, "declare refuses a relative path", "absolute")
    ledger = ART.InputLedger()
    pre = ledger.declare([a, b])
    assert pre[C.rel(a)] == C.sha256_file(a) and ledger.read_bytes(a) == b"A" and ledger.digest(a) == pre[C.rel(a)]
    expect(lambda: ledger.read_bytes(tmp / "undeclared.txt"), ART.InputsChangedError, "undeclared input read", "undeclared")
    expect(lambda: ledger.assert_all_read(), ART.InputsChangedError, "declared input never read", "never read")
    b.write_text("B2")
    expect(lambda: ledger.read_bytes(b), ART.InputsChangedError, "byte-bound read: bytes digest != PRE", "changed since the PRE")
    b.write_text("B")
    assert ledger.read_bytes(b) == b"B"
    ledger.assert_all_read()
    ledger.assert_unchanged("mid")
    same = tmp / "same.txt"; same.write_text("A")
    a.unlink(); a.symlink_to(same)
    expect(lambda: ledger.read_bytes(a), ART.StagingError, "leaf swapped with a symlink of identical content refused at read")
    expect(lambda: ledger.assert_unchanged("post-swap"), ART.StagingError, "POST snapshot refuses the swapped symlink leaf")
    a.unlink(); a.write_text("A")
    ledger.assert_unchanged("restored")
    shutil.move(str(inp), str(tmp / "moved"))
    inp.symlink_to(tmp / "moved", target_is_directory=True)
    assert (inp / "a.txt").read_text() == "A"  # same content through the retargeted ancestor
    expect(lambda: ledger.read_bytes(a), ART.StagingError, "ancestor retargeted to a symlink before read refused")
    expect(lambda: ledger.assert_unchanged("post-retarget"), ART.StagingError, "POST snapshot refuses the retargeted ancestor")
    inp.unlink(); shutil.move(str(tmp / "moved"), str(inp))
    ledger.assert_unchanged("restored2")
    a.write_text("A2")
    expect(lambda: ledger.assert_unchanged("post-mutation"), ART.InputsChangedError, "POST snapshot refuses a mutated input", "changed")
    expect(lambda: ledger.declare([a]), ART.InputsChangedError, "declare twice refused")
    nanf = tmp / "nan.json"; nanf.write_text('{"x": NaN}')
    expect(lambda: ART.DIRECT.read_json(nanf), ART.SerializationError, "strict JSON reader refuses NaN", "non-finite")
    expect(lambda: ART.loads_strict('{"x": 1e999}'), ART.SerializationError, "strict JSON refuses float overflow 1e999", "overflow")
    expect(lambda: ART.loads_strict('{"x": -1e999}'), ART.SerializationError, "strict JSON refuses negative float overflow", "overflow")
    expect(lambda: ART.loads_strict('{"a": 1, "a": 2}'), ART.SerializationError, "strict JSON refuses a duplicate key", "duplicate")
    expect(lambda: ART.loads_strict('{"a": {"b": 1, "b": 2}}'), ART.SerializationError, "strict JSON refuses a nested duplicate key", "duplicate")
    expect(lambda: ART.loads_strict('[{"k": 1}, {"k": 1, "k": 1}]'), ART.SerializationError, "strict JSON refuses a duplicate key inside a list element", "duplicate")
    ovf = tmp / "ovf.json"; ovf.write_text('{"deep": [{"x": 1e999}]}')
    expect(lambda: ART.DIRECT.read_json(ovf), ART.SerializationError, "strict JSON reader refuses nested overflow", "overflow")
    assert ART.loads_strict('{"a": 1, "b": {"c": [1.5, "x", null, true]}}') == {"a": 1, "b": {"c": [1.5, "x", None, True]}}
    names, data = ART.parse_sto("synthetic\nendheader\ntime\tx\n0\t1\n0.001\t2\n")
    assert names == ["time", "x"] and data.shape == (2, 2)
    expect(lambda: ART.parse_sto("no header\n0 1\n"), ValueError, "malformed .sto refused")
    if os.name == "posix":
        fifo = tmp / "fifo"; os.mkfifo(fifo)
        expect(lambda: ART.read_regular_nofollow(fifo), ART.StagingError, "FIFO refused by the no-follow reader without blocking (O_NONBLOCK + fstat)", "regular")
        expect(lambda: ART.InputLedger().declare([fifo]), ART.StagingError, "FIFO refused at declare without blocking", "regular")
    # content-addressed by design: a plain ancestor replaced by a plain copy with the SAME bytes is accepted
    ledger2 = ART.InputLedger(); ledger2.declare([a])
    shutil.copytree(inp, tmp / "copy_dir"); shutil.rmtree(inp); shutil.move(str(tmp / "copy_dir"), str(inp))
    assert ledger2.read_bytes(a) == b"A2" and ledger2.assert_unchanged("same bytes, plain copy") == ledger2.declared
    ok("InputLedger: no-follow chain (ancestor symlink at declare, swapped symlink leaf, ancestor retargeted to a symlink at read and POST), every read byte-bound to the PRE digest (re-opened each call, not a cache), undeclared/unread/mutated/declare-twice refused, FIFO non-blocking refusal, content-addressed acceptance of a same-byte plain copy; strict JSON (overflow, duplicate keys) and .sto parsing")
    # --- publish protocol -------------------------------------------------------------------------------------------
    art = tmp / "art"
    res = ART.publish_artifact(art / "run1", anchor=art, stamp="run1", build=simple_build("a"))
    final = art / "run1"
    assert res["artifact"]["ok"] and res["artifact"]["files"] == 1 and res["artifact"]["total_bytes"] == 1 and res["staged_files"] == 1 and res["sidecar_sha256"] == C.sha256_file(final / "artifact_manifest_run1.json") and not any(p.name.startswith(".staging") for p in art.iterdir()) and ".staging" not in json.dumps(res)
    sealed_manifest = json.loads((final / "artifact_manifest_run1.json").read_text())
    assert ART.verify_artifact_dir(final, anchor=art, strict=True, expected_manifest=sealed_manifest, expected_sidecar_sha256=res["sidecar_sha256"])["ok"] and D.exit_code_for(res) == 0 and D.exit_code_for({"artifact": {"ok": False}}) == 1 and D.exit_code_for(None) == 1
    expect(lambda: ART.publish_artifact(final, anchor=art, stamp="run1", build=simple_build()), FileExistsError, "existing final dir refused")
    for bad_stamp in ("../x", "a/b", "..", ".", ".staging_x", "artifact_manifest_x", "", "a\\b", "x" * 65, "-lead"):
        expect(lambda s=bad_stamp: ART.publish_artifact(art / "y", anchor=art, stamp=s, build=simple_build()), ART.StagingError, f"stamp {bad_stamp!r} refused")
    expect(lambda: ART.publish_artifact(art / "other", anchor=art, stamp="run2", build=simple_build()), ART.StagingError, "basename != stamp refused", "basename")
    anc = tmp / "anc"; anc.mkdir(); (tmp / "anc_link").symlink_to(anc, target_is_directory=True)
    expect(lambda: ART.publish_artifact(tmp / "anc_link" / "sub" / "run", anchor=tmp, stamp="run", build=simple_build()), ART.StagingError, "ancestor symlink refused BEFORE mkdir", "symlink")
    assert not (anc / "sub").exists()
    expect(lambda: ART.assert_no_symlink_components(tmp / ".." / "x", tmp, "t"), ART.StagingError, "path escaping the anchor refused", "traversal")
    expect(lambda: ART.assert_no_symlink_components(Path(f"{tmp}/a/../x"), tmp, "t"), ART.StagingError, "lexical traversal anchor/a/../x refused BEFORE normpath (would normalise inside the anchor)", "traversal")
    expect(lambda: ART.assert_no_symlink_components(Path(f"{tmp}/../{tmp.name}/x"), tmp, "t"), ART.StagingError, "lexical traversal anchor/../anchor/x refused", "traversal")
    expect(lambda: ART.assert_no_symlink_components(tmp / "x", Path(f"{tmp}/a/.."), "t"), ART.StagingError, "traversal inside the anchor refused", "traversal")
    assert ART.assert_no_symlink_components(tmp / "a" / "x", tmp, "t") == tmp / "a" / "x"
    rooted = tmp / "rooted.txt"; rooted.write_text("x")
    expect(lambda: ART.resolve_under_roots(f"{tmp}/sub/../rooted.txt", (tmp,), "t"), ART.StagingError, "resolve_under_roots refuses absolute in-root lexical traversal before normpath", "traversal")
    expect(lambda: ART.resolve_under_roots("sub/../rooted.txt", (tmp,), "t"), ART.StagingError, "resolve_under_roots refuses relative in-root lexical traversal before normpath", "traversal")
    assert ART.resolve_under_roots(str(rooted), (tmp,), "t") == rooted
    expect(lambda: ART.publish_artifact(art / "run3", anchor=art, stamp="run3", build=simple_build(extra=lambda st: (art / "run3").mkdir())), FileExistsError, "race: competitor directory appeared during the build")
    assert (art / "run3").is_dir() and not any((art / "run3").iterdir()) and not any(p.name.startswith(".staging_run3") for p in art.iterdir())
    src, dst = art / "src_dir", art / "dst_dir"
    src.mkdir(); dst.mkdir(); (src / "f").write_text("f")
    expect(lambda: ART.rename_noreplace(src, dst), FileExistsError, "no-replace primitive refuses an existing EMPTY directory")
    assert src.is_dir() and dst.is_dir() and (src / "f").is_file()
    dst.rmdir(); ART.rename_noreplace(src, dst)
    assert dst.is_dir() and (dst / "f").is_file() and not src.exists()

    def failing(staging: Path) -> dict:
        (staging / "b.txt").write_text("b")
        raise RuntimeError("boom")

    expect(lambda: ART.publish_artifact(art / "run4", anchor=art, stamp="run4", build=failing), RuntimeError, "build failure")
    assert not (art / "run4").exists() and not any(p.name.startswith(".staging_run4") for p in art.iterdir())
    expect(lambda: ART.publish_artifact(art / "run5", anchor=art, stamp="run5", build=lambda st: dict(PROV)), ART.StagingError, "empty build refused", "empty")
    expect(lambda: ART.publish_artifact(art / "run6", anchor=art, stamp="run6", build=lambda st: {"kind": "x"}), ART.StagingError, "provenance lacking required keys", "provenance")
    expect(lambda: ART.publish_artifact(art / "run7", anchor=art, stamp="run7", build=simple_build(extra=lambda st: (st / "artifact_manifest_fake.txt").write_text("x"))), ART.StagingError, "sidecar-like FILE in the staging tree refused at seal", "sidecar-like")
    expect(lambda: ART.publish_artifact(art / "run8", anchor=art, stamp="run8", build=simple_build(extra=lambda st: (st / "artifact_manifest_fake").mkdir())), ART.StagingError, "sidecar-like DIRECTORY in the staging tree refused at seal", "sidecar-like")
    expect(lambda: ART.publish_artifact(art / "run9", anchor=art, stamp="run9", build=simple_build(extra=lambda st: (st / "link.txt").symlink_to(st / "a.txt"))), ART.StagingError, "symlink in the staging tree refused", "symlink")
    def nan_provenance(st: Path) -> dict:
        (st / "a.txt").write_text("a", encoding="utf-8")
        return {**PROV, "x": float("nan")}

    expect(lambda: ART.publish_artifact(art / "run10", anchor=art, stamp="run10", build=nan_provenance), ART.SerializationError, "NaN in provenance refused (file built, then non-finite provenance)")
    assert not (art / "run10").exists() and not any(p.name.startswith(".staging") for p in art.iterdir())

    def tamper_sidecar(field: str, value):
        def hook(st: Path) -> None:
            side = st / "artifact_manifest_run11.json"
            m = json.loads(side.read_text()); m[field] = value
            side.write_text(json.dumps(m))

        return hook

    expect(lambda: ART.publish_artifact(art / "run11", anchor=art, stamp="run11", build=simple_build(), after_seal=tamper_sidecar("git_head", "zz")), ART.ArtifactVerificationError, "PRE-rename provenance substitution with valid types refused (byte/deep-exact)", "sealed")
    expect(lambda: ART.publish_artifact(art / "run11", anchor=art, stamp="run11", build=simple_build(), after_seal=lambda st: (st / "a.txt").write_text("tampered")), ART.ArtifactVerificationError, "PRE-rename file tamper refused")
    expect(lambda: ART.publish_artifact(art / "run11", anchor=art, stamp="run11", build=simple_build(), after_seal=lambda st: (st / "artifact_manifest_fake.txt").write_text("x")), ART.StagingError, "PRE-rename sidecar-like file refused")
    assert not (art / "run11").exists() and not any(p.name.startswith(".staging") for p in art.iterdir())
    # before_publish tampers the staging tree: the staged verification runs AFTER it, so the final dir never appears
    staging_seen: list[Path] = []
    bp_build = simple_build(extra=lambda st: staging_seen.append(st))
    expect(lambda: ART.publish_artifact(art / "run12", anchor=art, stamp="run12", build=bp_build, before_publish=lambda: (staging_seen[0] / "a.txt").write_text("tampered-by-before-publish")), ART.ArtifactVerificationError, "before_publish tampering caught by the staged verification immediately before the rename")
    assert not (art / "run12").exists() and not any(p.name.startswith(".staging") or p.name.startswith(".quarantine") for p in art.iterdir())

    def raising_before_publish():
        raise ART.InputsChangedError("inputs changed")

    expect(lambda: ART.publish_artifact(art / "run12", anchor=art, stamp="run12", build=simple_build(), before_publish=raising_before_publish), ART.InputsChangedError, "before_publish failure aborts before any rename")
    assert not (art / "run12").exists() and not any(p.name.startswith(".staging") for p in art.iterdir())
    # post-rename tamper (adversarial hook): the invalid tree is quarantined, nothing remains under the stamp
    expect(lambda: ART.publish_artifact(art / "run13", anchor=art, stamp="run13", build=simple_build(), after_publish=lambda fd: (fd / "a.txt").write_text("tampered-after-rename")), ART.ArtifactVerificationError, "POST-rename tamper -> quarantined and raised", "quarantine")
    quarantined = [p for p in art.iterdir() if p.name.startswith(".quarantine_run13_")]
    assert not (art / "run13").exists() and len(quarantined) == 1 and (quarantined[0] / "a.txt").read_text() == "tampered-after-rename" and (quarantined[0] / "artifact_manifest_run13.json").is_file()
    expect(lambda: ART.publish_artifact(art / "run14", anchor=art, stamp="run14", build=simple_build(), after_publish=lambda fd: (fd / "artifact_manifest_run14.json").write_text(json.dumps(dict(json.loads((fd / "artifact_manifest_run14.json").read_text()), git_head="zz")))), ART.ArtifactVerificationError, "POST-rename provenance substitution -> quarantined and raised", "quarantine")
    assert not (art / "run14").exists() and len([p for p in art.iterdir() if p.name.startswith(".quarantine_run14_")]) == 1
    res13 = ART.publish_artifact(art / "run13", anchor=art, stamp="run13", build=simple_build())
    assert res13["artifact"]["ok"] and (art / "run13").is_dir() and len([p for p in art.iterdir() if p.name.startswith(".quarantine_run13_")]) == 1
    ok("publish ordering: before_publish precedes the last staged verification (tamper never published, failure aborts); post-rename failure quarantines the tree under a unique no-replace sibling and raises; the stamp can then be published validly")
    # after_publish replaces the authoritative stamp with a non-directory leaf: it must be quarantined as the ENTRY
    # itself (never followed), the stamp must be lexically free afterwards, and the same stamp must be publishable again
    import stat as _stat  # noqa: PLC0415

    def planted(kind: str):
        def hook(fd: Path) -> None:
            moved = art / f"moved_{kind}"
            shutil.move(str(fd), str(moved))
            if kind == "symlink":
                fd.symlink_to(moved, target_is_directory=True)
            elif kind == "broken_symlink":
                fd.symlink_to(art / "does_not_exist_anywhere")
            elif kind == "file":
                fd.write_text("planted", encoding="utf-8")
            elif kind == "fifo":
                os.mkfifo(fd)
            elif kind == "removed":
                pass
            else:
                raise AssertionError(kind)

        return hook

    kinds = ["symlink", "broken_symlink", "file", "removed"] + (["fifo"] if os.name == "posix" else [])
    for i, kind in enumerate(kinds):
        stamp = f"run2{i}"
        expect(lambda s=stamp, k=kind: ART.publish_artifact(art / s, anchor=art, stamp=s, build=simple_build(), after_publish=planted(k)), ART.ArtifactVerificationError, f"after_publish planted {kind} under the stamp -> quarantined/reported and raised", "stamp is now free" if True else None)
        assert not os.path.lexists(art / stamp), f"{kind}: stamp still occupied lexically"
        q = [p for p in art.iterdir() if p.name.startswith(f".quarantine_{stamp}_")]
        if kind == "removed":
            assert q == []
        else:
            assert len(q) == 1
            st = os.lstat(q[0])
            if kind == "symlink":
                assert _stat.S_ISLNK(st.st_mode) and q[0].resolve() == (art / "moved_symlink").resolve() and (art / "moved_symlink" / "a.txt").is_file()
            elif kind == "broken_symlink":
                assert _stat.S_ISLNK(st.st_mode) and not q[0].exists()
            elif kind == "file":
                assert _stat.S_ISREG(st.st_mode) and q[0].read_text() == "planted"
            elif kind == "fifo":
                assert _stat.S_ISFIFO(st.st_mode)
        again = ART.publish_artifact(art / stamp, anchor=art, stamp=stamp, build=simple_build())
        assert again["artifact"]["ok"] and (art / stamp).is_dir() and not (art / stamp).is_symlink()
    expect(lambda: ART.quarantine_entry(art / "never_existed"), ART.StagingError, "quarantine of a missing entry refused", "nothing to quarantine")
    ok(f"after_publish planted leaves ({', '.join(kinds)}): the entry itself is quarantined without following it, the stamp is lexically free, retry of the same stamp publishes validly")
    side = final / "artifact_manifest_run1.json"
    good_text = side.read_text()
    m = json.loads(good_text); m["git_head"] = "zz"
    side.write_text(json.dumps(m))
    expect(lambda: ART.verify_artifact_dir(final, anchor=art, strict=True, expected_sidecar_sha256=res["sidecar_sha256"]), ART.ArtifactVerificationError, "POST-rename provenance substitution refused by the pinned sidecar digest", "substitution")
    expect(lambda: ART.verify_artifact_dir(final, anchor=art, strict=True, expected_manifest=sealed_manifest), ART.ArtifactVerificationError, "POST-rename provenance substitution refused by the deep-exact manifest", "deep-exact")
    side.write_text(good_text)
    assert ART.verify_artifact_dir(final, anchor=art, strict=True, expected_manifest=sealed_manifest, expected_sidecar_sha256=res["sidecar_sha256"])["ok"]
    (final / "artifact_manifest_fake.txt").write_text("x")
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, "fake sidecar-like file in the published tree refused", "sidecar-like")
    (final / "artifact_manifest_fake.txt").unlink()
    (final / "artifact_manifest_fake").mkdir()
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, "fake sidecar-like DIRECTORY in the published tree refused", "sidecar-like")
    (final / "artifact_manifest_fake").rmdir()
    (final / "artifact_manifest_other.json").write_text(good_text)
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, "second sidecar refused")
    expect(lambda: ART.verify_artifact_dir(final, anchor=art, expected_stamp="run1"), ART.StagingError, "second sidecar refused even with the expected stamp", "sidecar-like")
    (final / "artifact_manifest_other.json").unlink()
    if os.name == "posix":
        os.mkfifo(final / "fifo")
        expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, "special node (FIFO) in the published tree refused", "special")
        (final / "fifo").unlink()
    (final / "extra.txt").write_text("x")
    v = ART.verify_artifact_dir(final, anchor=art)
    assert v["ok"] is False and v["unlisted"] == ["extra.txt"]
    expect(lambda: ART.verify_artifact_dir(final, anchor=art, strict=True), ART.ArtifactVerificationError, "unlisted file -> strict raises")
    (final / "extra.txt").unlink()
    (final / "a.txt").write_text("tampered")
    v = ART.verify_artifact_dir(final, anchor=art)
    assert v["mismatches"] == ["a.txt"] and D.exit_code_for({"artifact": v}) == 1
    expect(lambda: ART.verify_artifact_dir(final, anchor=art, strict=True), ART.ArtifactVerificationError, "POST-rename file tamper -> strict raises")
    (final / "a.txt").write_text("a")
    (final / "lnk").symlink_to(final / "a.txt")
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, "symlink inside the published tree refused", "symlink")
    (final / "lnk").unlink()
    good_manifest = json.loads(good_text)
    for label, mutate in (("false file_count", lambda m: m.update(file_count=2)), ("false total_bytes", lambda m: m.update(total_bytes=99)), ("schema_version", lambda m: m.update(schema_version=1)), ("self_excluded False", lambda m: m.update(self_excluded=False)), ("missing provenance", lambda m: m.pop("interpreter")), ("bad sha format", lambda m: m["files"][0].update(sha256="zz")), ("traversal path", lambda m: m["files"][0].update(path="../a.txt")), ("absolute path", lambda m: m["files"][0].update(path="/etc/passwd")), ("sidecar-like listed path", lambda m: m["files"][0].update(path="artifact_manifest_x/a.txt")), ("negative bytes", lambda m: (m["files"][0].update(bytes=-1), m.update(total_bytes=-1))), ("sidecar name mismatch", lambda m: m.update(sidecar="artifact_manifest_zz.json")), ("stamp mismatch", lambda m: m.update(stamp="zz")), ("consumed_inputs negative", lambda m: m.update(consumed_inputs=-1)), ("source_analysis without path", lambda m: m.update(source_analysis={"sha256": "0" * 64})), ("inputs_unchanged False", lambda m: m.update(inputs_unchanged=False))):
        mm = json.loads(json.dumps(good_manifest)); mutate(mm)
        side.write_text(json.dumps(mm))
        expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.StagingError, f"sidecar {label} refused")
    side.write_text(json.dumps(dict(good_manifest, extra=1.0)).replace("1.0", "NaN"))
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.SerializationError, "NaN in sidecar refused")
    side.write_text(json.dumps(dict(good_manifest, note=str(tmp / ".staging_run1_1"))))
    expect(lambda: ART.verify_artifact_dir(final, anchor=art), ART.SerializationError, "staging path in sidecar refused", "staging")
    side.write_text(good_text)
    assert ART.verify_artifact_dir(final, anchor=art, strict=True)["ok"]
    expect(lambda: ART.dumps_strict({"x": float("nan")}), ART.SerializationError, "NaN refused by strict JSON")
    expect(lambda: ART.dumps_strict({"p": str(tmp / ".staging_run1_1" / "a.npz")}), ART.SerializationError, "staging path refused", "staging")
    expect(lambda: ART.assert_finite_tree({"a": [1.0, float("inf")]}), ART.SerializationError, "Inf refused by assert_finite_tree")
    files2 = D.write_observation_set(tmp / "out2", "t2", built)
    assert all(".staging_" not in str(v) for v in files2.values())
    ok("publish protocol: stamps, ancestor symlink before mkdir, race, no-replace primitive, build failure/empty/provenance, sidecar-like file/dir/symlink at seal, NaN provenance, PRE-rename provenance/file tamper, POST-rename provenance substitution (digest + deep-exact), fake sidecar file/dir/second sidecar/FIFO, unlisted/mismatch/symlink, schema mutations, nonzero exit code")


def real_smoke() -> None:
    analysis, ref_info = D.load_final_analysis()
    assert ref_info["sha256"] == D.FINAL_ANALYSIS_SHA256 and analysis["analysis_complete"] is True
    specs = D.corpus_specs()
    assert len(specs) == 24 and all(s[2] == "v3_canonical" and s[1] in ("B0820_H0", "JUL_H0") and s[5] in C.DEVELOPMENT_SEEDS for s in specs) and len({M.job_id(s) for s in specs}) == 24
    real_actors = {c: D.load_actor(C.CANDIDATES[c]["module"]) for c in ("B0820_H0", "JUL_H0")}
    checks = [D.verify_forward_pass(real_actors[s[1]], M.output_dir_for(s, None), s[4]) for s in specs]
    assert len(checks) == 24 and all(c["max_abs_error"] <= D.FORWARD_TOLERANCE for c in checks) and sum(1 for c in checks if c["compared_with"] == "policy_action_mean") == 18 and sum(1 for c in checks if c["compared_with"] == "raw_policy_action") == 6
    assert C.sha256_file(Path(C.CANDIDATES["B0820_H0"]["actor_feature_manifest"])) == C.ACTOR_MANIFEST_35_SHA256 and all(Path(HERE / n).is_file() for n in D.CODE_INPUTS) and (C.BASELINE_DIR / "warm_start.py").is_file()
    ok(f"real read-only smoke: pinned final analysis; 24 canonical corpus specs; forward pass of B0820_H0/JUL_H0 on ALL 24 traces (max error {max(c['max_abs_error'] for c in checks):.2e}; 18 stochastic / 6 deterministic); feature manifest and code inputs present")


def main() -> int:
    parser = argparse.ArgumentParser(description="f0_actor_drift self-test (temp-only by default)")
    parser.add_argument("--real", action="store_true", help="also run the opt-in read-only smoke on the real traces/modules")
    args = parser.parse_args()
    base = tmp_base()
    tmp = Path(tempfile.mkdtemp(prefix="f0_drift_selftest_", dir=base)).resolve()
    try:
        assert not str(tmp).startswith(str(C.REPO))
        synthetic(tmp)
        if args.real:
            real_smoke()
        else:
            print("  skip : real read-only smoke (opt-in with --real)")
        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
