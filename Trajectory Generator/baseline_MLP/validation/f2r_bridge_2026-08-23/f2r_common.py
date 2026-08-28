"""Shared registry, pins and structural assertions for the F2R bridge (S0).

S0 = tooling + synthetic tests + dry-run only (authorised by the architect,
2026-08-23, after PASS of proposal revision 3).  Nothing in this package runs
a rollout, steps the simulator, fits a student, or evaluates P0 on real data.

F0 (``validation/f0_freeze_2026-08-22``) and F1
(``validation/f1_ablation_2026-08-23``) are imported as immutable libraries
through their own module names; no global of theirs is rebound.

Binding decisions (report §13, rev 3):
* init primaria = JUL_H0 35D, pinned by actor digest; B0820_H0 = negative
  control only (never an init);
* teacher = V26 39D fed through the F1 adapter (targets + prescribed clock),
  evaluated OFFLINE on the student's visited states; anchors = the three
  existing A_iso6clk traces (content-addressed), no new teacher rollouts;
* seeds: det 123 (training side), collection 123/124, **125 = held-out
  promotion gate (never in any dataset/label/fit/aggregation)**, 126-128
  sealed (F3 only);
* variants: T1 commissioning (1 round, 3 det, always STOP), T2 primary
  (R <= 4), T3 contingency (R <= 2, only on the preregistered trigger);
* student stays the deployable 35D stateless MLP; exported checkpoints carry
  exactly the original 10 ``pi*`` keys (auxiliary phase head training-only).
"""

from __future__ import annotations

import json
import os
import tempfile
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
F1_DIR = VALIDATION_DIR / "f1_ablation_2026-08-23"
F0_DIR = VALIDATION_DIR / "f0_freeze_2026-08-22"
for entry in (str(F0_DIR), str(F1_DIR), str(HERE)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f0_common as C  # noqa: E402  (F0 library, immutable)
import f1_common as F1  # noqa: E402  (F1 library, immutable)

REPO = C.REPO
TG_DIR = C.TG_DIR
BASELINE_DIR = C.BASELINE_DIR

F2R_REV = os.environ.get("CMC_F2R_REV", "r1").strip() or "r1"
F2R_TAG = "2026-08-23_F2R_bridge_rev3"
OUT_ROOT = C.RUNS_ROLLOUT / "validation" / "f2r_bridge_runs" / f"{F2R_TAG}_{F2R_REV}"
OUT_MANIFEST = OUT_ROOT / "manifest"
OUT_ROLLOUTS = OUT_ROOT / "rollouts"
OUT_DATASETS = OUT_ROOT / "datasets"
OUT_CACHE = OUT_ROOT / "privileged_cache"
OUT_REFIT = OUT_ROOT / "refit"
OUT_P0 = OUT_ROOT / "p0"
OUT_METRICS = OUT_ROOT / "metrics"
OUT_GATE = OUT_ROOT / "gate"
OUT_LOGS = OUT_ROOT / "logs"

PROTOCOL_JSON = HERE / "f2r_protocol.json"
PROTOCOL_MD = HERE / "PROTOCOL_F2R.md"
PROTOCOL_ID = "F2R-S0-bridge-rev3"

ROLLOUT_EVAL = C.ROLLOUT_EVAL
RUNTIME_CONFIG = F1.RUNTIME_CONFIG
RUNTIME_CONFIG_SHA256 = F1.RUNTIME_CONFIG_SHA256
STARTS = tuple(F1.STARTS)
EXACT_STARTS = dict(F1.EXACT_STARTS)

ACTION_DIM = 2
ENV_ACTOR_WIDTH = 35
MODULE_WIDTH_39 = 39
SIGMA_CONSTANT = 0.005

# --- seeds (rev 3) -------------------------------------------------------------
DET_SEED = 123
COLLECTION_SEEDS = (123, 124)          # det 123 + stochastic 123/124 may enter datasets/labels/fits
VALIDATION_SEED = 125                  # held-out promotion gate: never in any dataset/label/fit/aggregation
SEALED_SEEDS = tuple(C.SEALED_SEEDS)   # (126, 127, 128): F3 only
DEVELOPMENT_SEEDS = tuple(C.DEVELOPMENT_SEEDS)

# --- pinned actors ---------------------------------------------------------------
INIT_PRIMARY = {
    "name": "JUL_H0",
    "module": C.CANDIDATES["JUL_H0"]["module"],
    "actor_digest": "a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21",
    "module_state_sha256": "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
    "class_and_ctor_args_sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
    "metadata_sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "width": ENV_ACTOR_WIDTH,
    "role": "init_primary_immutable",
    "description": "July H0 35D (target-domain existence proof: bootstraps under v3, negative ankle, constant sigma 0.005, clock columns zero).",
}
NEGATIVE_CONTROL = {
    "name": "B0820_H0",
    "module": C.CANDIDATES["B0820_H0"]["module"],
    "actor_digest": "ce46f729080b069274f6f69a504dfedc92440584f4e4eb6281bdcc153c3a193a",
    "module_state_sha256": "1733fb6df3aa05ab152fdba791c92ad5bba210b5ce48c08e150ee4d6ee8c1e66",
    "width": ENV_ACTOR_WIDTH,
    "role": "negative_control_only",
    "description": "Hard-drop transplant actor: negative control / ablation only, never an init.",
}
TEACHER = {
    "name": "V26_39D",
    "module": C.CANDIDATES["V26_39D"]["module"],
    "actor_digest": "5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e",
    "module_state_sha256": "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd",
    "width": MODULE_WIDTH_39,
    "role": "privileged_teacher_offline",
    "description": "V26 39D actor fed with the 4 prescribed targets at 2:6 and the prescribed right-side clock at 0:2 (= A_iso6clk), evaluated offline on visited states.",
}
CORRIDOR_PROFILE = {"path": C.MORPH_PROFILE_EVENT_WARPED, "sha256": "33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55"}

# --- anchors: the three existing A_iso6clk traces (content-addressed, no new teacher rollouts) ---
_F1_CLK = F1.OUT_ROOT / "rollouts" / "aiso_clk_diag"
ANCHORS: dict[str, dict[str, Any]] = {
    "minus020": {
        "job_dir": _F1_CLK / "A_ISO39CLK_V3__v3_canonical__minus020__det",
        "receipt_sha256": "85a0ce2083b28b047bbe8207bf0e3b1feb0a0cfd85a2ced29d6256cddfbb7dfa",
        "summary_sha256": "d315423abaeb6a92ff0eccf93f9bc6b77387071442a1694d1e17179fb708bd2a",
        "trace_sha256": "05de8103802af4363c12dcd3ee56b87cc1c3a3954640156a68b5d7d83141f79a",
        "reset_sha256": "42d07621fcede8a5d59e7365be002ce4ece548687829013d77b9d93db6d96cc7",
        "adapter_summary_sha256": "c87f7eedbf7b040442848f98f148aee4ee77db7574715247145b0a86c37cf620",
        "adapter_trace_sha256": "cf1691a286e58674ed4b085a24993b038a133644ba00f960a36e628cc87f6075",
    },
    "nominal": {
        "job_dir": _F1_CLK / "A_ISO39CLK_V3__v3_canonical__nominal__det",
        "receipt_sha256": "0b76b014fff70772c7264fdbf22dd240f750b962c446a5f6731009a058949fec",
        "summary_sha256": "6c2b05b2cad24ee386ffa085cb96ce50d8485e9473a4935e14d3ed9f66c27c48",
        "trace_sha256": "3a66d1098a553a1680dff028413f4eb27697d81c64cf8c15193e3937e41b0fed",
        "reset_sha256": "3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76",
        "adapter_summary_sha256": "c87f7eedbf7b040442848f98f148aee4ee77db7574715247145b0a86c37cf620",
        "adapter_trace_sha256": "419581fd745f626371f87efc6ad92e19f7c9d52ce5ed9c800d1c574830f2eaea",
    },
    "plus020": {
        "job_dir": _F1_CLK / "A_ISO39CLK_V3__v3_canonical__plus020__det",
        "receipt_sha256": "42ee7f4069d4f05ea03c0afbb37cfa3c6c6525a1d1e06b61b88ba4f90f0b2ca1",
        "summary_sha256": "e487ad8c2820d04f57068cf5426ac82952d4f6577f349dc83929b092bba2072d",
        "trace_sha256": "5375547a9211e9fce2341feb8c7d57d82b689d25de26147bc49945999b20ad68",
        "reset_sha256": "614e0e5c55fc7f6109418b80d1ed82f494c6da72533efc6a410145846ccc38de",
        "adapter_summary_sha256": "c87f7eedbf7b040442848f98f148aee4ee77db7574715247145b0a86c37cf620",
        "adapter_trace_sha256": "ffa8d670a0b561fd2983a3b9b0d66da8042eedb8e6bed3bf3aed7bcad1d65476",
    },
}
ANCHOR_FILES = {"receipt_sha256": "f1_receipt.json", "summary_sha256": "rollout_summary.json", "trace_sha256": "rollout_policy_trace.json", "reset_sha256": "rollout_reset_diagnostics.json", "adapter_summary_sha256": "f1_adapter_summary.json", "adapter_trace_sha256": "f1_adapter_trace.json"}

# JUL_H0 deterministic F0 jobs under v3 (P0 states; read-only)
P0_JUL_JOBS = {s: F1.F0_ROLLOUTS / "det" / f"JUL_H0__v3_canonical__{s}__det" for s in STARTS}
# A_iso (4/6) F1 jobs: negative / OOD diagnostic only
OOD_AISO_JOBS = {s: F1.OUT_ROOT / "rollouts" / "aiso_det" / f"A_ISO39_V3__v3_canonical__{s}__det" for s in STARTS}

# --- 35D feature indices (derived from the pinned manifest; verified at import) ---
FEATURE_NAMES_35: tuple[str, ...] = tuple(json.loads(Path(C.ACTOR_MANIFEST_35).read_text(encoding="utf-8"))["actor_feature_names"])
if len(FEATURE_NAMES_35) != ENV_ACTOR_WIDTH or C.sha256_file(C.ACTOR_MANIFEST_35) != C.ACTOR_MANIFEST_35_SHA256:
    raise RuntimeError("pinned 35D manifest changed")
IDX = {name: i for i, name in enumerate(FEATURE_NAMES_35)}
CLOCK_COLUMNS = (IDX["gait_phase_sin"], IDX["gait_phase_cos"])                      # (0, 1): dead constants, hard-zero in the student
ONLINE_CLOCK_TRIPLET = (IDX["online_left_gait_phase_sin"], IDX["online_left_gait_phase_cos"], IDX["online_left_cycle_duration_s"])  # (14, 15, 16)
CONTROLLER_MEMORY = tuple(IDX[n] for n in FEATURE_NAMES_35 if n.endswith(("_previous_endpoint", "_served_ref", "_served_ref_vel", "_served_ref_accel", "_sea_u")))
FORBIDDEN_P0_INPUT_NAMES = ("time", "t", "t_pre", "step", "step_index", "index", "gait_phase_sin", "gait_phase_cos")
P0_PRE_CYCLE_EXCLUDED = CLOCK_COLUMNS + ONLINE_CLOCK_TRIPLET

# --- action semantics (verified in code, report §13.3) ---------------------------
ABSOLUTE_BOUNDS_RAD = {"pros_knee_angle": (-1.5, 0.0), "pros_ankle_angle": (-0.7, 0.7)}
PROS_COORDS = ("pros_knee_angle", "pros_ankle_angle")
SLEW_LIMITS_RAD_S = {"pros_knee_angle": 2.5, "pros_ankle_angle": 2.0}
SEGMENT_DURATION_S = 0.01
IK_TEACHER_LOOKAHEAD_S = 0.0

# --- variants --------------------------------------------------------------------
VARIANTS: dict[str, dict[str, Any]] = {
    "T1": {"role": "commissioning", "label": "u_T (A_iso6clk) for both joints", "max_rounds": 1, "rollouts": "3 det (seed 123) only", "stop": "always after gate A (structural inability on B3)", "consumes_T2_budget": False},
    "T2": {"role": "primary", "label": "knee from u_T (A_iso6clk), ankle from u_IK (prescribed prosthetic IK, lookahead 0)", "max_rounds": 4, "rollouts": "per round: 3 det (123) + 6 stoch (123/124) + 3 stoch (125 held-out gate)", "consumes_T2_budget": True},
    "T3": {"role": "contingency", "label": "u_IK for both joints (July recipe)", "max_rounds": 2, "trigger": "T2 passes gate A and fails B3, or the preregistered equivalent diagnosis (B2 and/or B5 FAIL with A PASS)", "tuning": "none (same hyperparameters as T2)", "consumes_T2_budget": False},
}

# --- refit budget (frozen) --------------------------------------------------------
REFIT_BUDGET = {"epochs": 300, "batch_size": 256, "lr": 1e-4, "optimizer": "Adam", "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3, "selection": "closed_loop_only", "aux_head": "training_time_only"}


# --- protocol ----------------------------------------------------------------------


def load_protocol() -> dict[str, Any]:
    payload = json.loads(PROTOCOL_JSON.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("protocol_id") != PROTOCOL_ID:
        raise RuntimeError("f2r_protocol.json missing or wrong protocol_id")
    return payload


def protocol_digests() -> dict[str, str | None]:
    return {"f2r_protocol.json": C.sha256_file(PROTOCOL_JSON), "PROTOCOL_F2R.md": C.sha256_file(PROTOCOL_MD) if PROTOCOL_MD.is_file() else None}


# --- structural assertions (fail-closed) -------------------------------------------


class F2RContractError(RuntimeError):
    """Violation of a preregistered structural rule."""


def assert_collection_seed(seed: int, *, allow_det: bool = True) -> int:
    seed = int(seed)
    if seed == VALIDATION_SEED:
        raise F2RContractError(f"seed {seed} is the held-out promotion gate: it must never enter a dataset, label, fit or aggregation")
    if seed in SEALED_SEEDS:
        raise F2RContractError(f"seed {seed} is sealed (F3 only)")
    if seed not in COLLECTION_SEEDS:
        raise F2RContractError(f"seed {seed} is not a collection seed {COLLECTION_SEEDS}")
    return seed


SEALED_UNLOCK_STAGE = "F3"  # the only stage that opens seeds 126-128 (sealed test, opened once, rollouts only)
KNOWN_STAGES = ("S0", "S1", "F3")


def assert_rollout_seed(seed: int, *, purpose: str, authorized_stage: str | None = None) -> int:
    """Seeds allowed per rollout purpose: 'collection' (123/124), 'validation' (125 only),
    'sealed_f3' (126-128).  The sealed seeds are refused for every purpose and every stage
    except ``purpose='sealed_f3'`` **with** ``authorized_stage == SEALED_UNLOCK_STAGE`` ('F3'):
    the unlock is explicit, fail-closed (None / 'S0' / 'S1' / anything else refuse) and never
    extends to collection/validation purposes (a sealed seed can never enter a dataset, label,
    fit or aggregation: ``assert_collection_seed`` has no stage parameter by design)."""
    seed = int(seed)
    if purpose == "collection":
        return assert_collection_seed(seed)
    if purpose == "validation":
        if seed != VALIDATION_SEED:
            raise F2RContractError(f"validation rollouts use only seed {VALIDATION_SEED}")
        return seed
    if purpose == "sealed_f3":
        if seed not in SEALED_SEEDS:
            raise F2RContractError(f"sealed rollouts use only seeds {SEALED_SEEDS}")
        if authorized_stage != SEALED_UNLOCK_STAGE:
            raise F2RContractError(f"sealed seed {seed} stays sealed: unlock requires authorized_stage={SEALED_UNLOCK_STAGE!r} (got {authorized_stage!r}); F2R (S0/S1) never opens 126-128")
        return seed
    raise F2RContractError(f"unknown rollout purpose {purpose}")


def assert_no_forbidden_inputs(column_names: Sequence[str], *, pre_cycle: bool) -> None:
    names = [str(n) for n in column_names]
    for n in names:
        if n in FORBIDDEN_P0_INPUT_NAMES:
            raise F2RContractError(f"forbidden input feature {n!r} (time/index/prescribed clock)")
        if pre_cycle and n in (FEATURE_NAMES_35[i] for i in ONLINE_CLOCK_TRIPLET):
            raise F2RContractError(f"online clock feature {n!r} not allowed in the pre-cycle P0 variant")
        if n not in IDX:
            raise F2RContractError(f"unknown feature {n!r}: only the 35D manifest features are allowed")


def verify_anchor_pins() -> dict[str, Any]:
    """Re-hash the three anchor job directories against the pinned SHAs (read-only)."""
    out: dict[str, Any] = {}
    for start, spec in ANCHORS.items():
        d = Path(spec["job_dir"])
        rec: dict[str, Any] = {"job_dir": C.rel(d), "exists": d.is_dir()}
        ok = d.is_dir()
        for key, fname in ANCHOR_FILES.items():
            p = d / fname
            sha = C.sha256_file(p) if p.is_file() else None
            rec[key] = {"pinned": spec[key], "disk": sha, "match": sha == spec[key]}
            ok = ok and sha == spec[key]
        rec["all_match"] = ok
        out[start] = rec
    out["all_match"] = all(v["all_match"] for k, v in out.items() if k != "all_match")
    return out


DIGEST_SEMANTICS = {
    "actor_digest": "warm_start.actor_state_digest: SHA-256 over the sorted actor tensor digests (parameter CONTENT of the 10 pi* arrays; independent of how module_state.pkl is serialised)",
    "module_state_sha256": "SHA-256 of the module_state.pkl FILE BYTES (serialisation-dependent: pickle protocol/ordering change it while the actor digest stays)",
    "rule": "both are pinned and both must match; they are different quantities and are never interchangeable",
}


# --- P0 inputs: content pins of the three JUL_H0 deterministic F0 jobs (read-only, fail-closed) ----
# Files (F0 layout): rollout_summary.json, rollout_policy_trace.json, rollout_reset_diagnostics.json
# (the reset snapshot) and f0_receipt.json (the job receipt, F0 schema 4 with internal digests).
P0_JUL_FILES = {"summary": "rollout_summary.json", "trace": "rollout_policy_trace.json", "reset": "rollout_reset_diagnostics.json", "receipt": "f0_receipt.json"}
P0_JUL_RECEIPT_SCHEMA = 4
ROLLOUT_EVAL_SHA256_PINNED = "5433bcbcd90cbfbc04429a42dcda3669649ed9e3b1daa0866c0d96681a127dba"  # production rollout_eval.py that produced every F0/F1/F2R rollout
P0_JUL_PINS: dict[str, dict[str, Any]] = {
    "minus020": {"job_id": "JUL_H0__v3_canonical__minus020__det", "steps": 500, "summary_sha256": "cd85e2dc1a96a265f3e58cbcd57e3e2b6fa2b80398eaf01e05aba1c5c2dfb93c", "trace_sha256": "fa12e462c048d05d002d8a22bcae5156d7f328a66019e94b0c492ec07cf98b04", "reset_sha256": "42d07621fcede8a5d59e7365be002ce4ece548687829013d77b9d93db6d96cc7", "receipt_sha256": "62642b8dd3ae7f23b8d0afe57d6fac147556f0b33d5ef27a4596297202abbe9b"},
    "nominal": {"job_id": "JUL_H0__v3_canonical__nominal__det", "steps": 500, "summary_sha256": "c73e165727d51554da7b9c3c84e8b0a79b59e8dddb41ef7d100d56947896f0d0", "trace_sha256": "1e118690edfbb940da36603f8068e81ca93e37f4213da37e631eadfbf7942fbe", "reset_sha256": "3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76", "receipt_sha256": "a5d507fb188817bfd2b7d71ced1d7b60c0466e5d940197b3b92584f6f3977970"},
    "plus020": {"job_id": "JUL_H0__v3_canonical__plus020__det", "steps": 176, "summary_sha256": "ef3eee23b040a865025d98b9441299903b737b056d4561fa800fcf9b5501c6cb", "trace_sha256": "9d2e7615534ba670afebdc2b9f05e0e0c89a074e2e797bab6c6ac93e6654895c", "reset_sha256": "614e0e5c55fc7f6109418b80d1ed82f494c6da72533efc6a410145846ccc38de", "receipt_sha256": "7954658fb56cba5b0191e2e8c697bffc6a558b2a18993edc16fcd2e6ae0afaca"},
}
for _s, _pin in P0_JUL_PINS.items():
    _pin["job_dir"] = C.rel(P0_JUL_JOBS[_s])  # expected POSIX path relative to the repo root


def _path_problems(path: Path, root: Path, expected_rel: str, *, kind: str) -> list[str]:
    """Regular file/dir, not a symlink, no symlinked component below ``root``, at the expected relative path."""
    problems: list[str] = []
    if path.is_symlink():
        problems.append(f"{kind} is a symlink: {path}")
    if kind == "dir":
        if not path.is_dir():
            problems.append(f"{kind} missing: {path}")
    elif not path.is_file():
        problems.append(f"{kind} missing or not a regular file: {path}")
    try:
        rel = path.relative_to(root).as_posix()
    except ValueError:
        rel = None
    if rel != expected_rel:
        problems.append(f"{kind} not at the expected path: {rel!r} != {expected_rel!r}")
    if path.exists() and Path(os.path.realpath(path)) != Path(os.path.realpath(root)) / expected_rel:
        problems.append(f"{kind} path contains a symlinked component: {path}")
    return problems


def verify_p0_jul_pins(*, jobs: Mapping[str, Path] | None = None, pins: Mapping[str, Mapping[str, Any]] | None = None, root: Path | None = None, f0_analysis: Path | None = None, f0_analysis_sha256: str | None = None) -> dict[str, Any]:
    """Content-pin + receipt verification of the three JUL_H0 deterministic F0 jobs used by P0
    (minus020 / nominal / plus020), read-only and fail-closed.  Per job: directory and the four
    files are regular, non-symlinked, at the expected relative path and with the pinned SHA-256;
    the F0 receipt (schema 4) is internally consistent (its ``summary_sha256`` / ``trace_sha256``
    equal the files on disk; ``module_state_sha256`` = JUL_H0 pin; ``config_sha256`` = runtime pin;
    ``rollout_eval_sha256`` = production pin; job_id/candidate/runtime/seed/start/selection/steps/
    status/returncode/output_dir as pinned); the summary carries the exact start, seed 123,
    deterministic selection and the pinned step count; the reset snapshot has a numeric ``time``;
    and the pinned F0 analysis (``F1.F0_ANALYSIS_SHA256``) records the same summary/trace digests
    for that job_id.  ``jobs``/``pins``/``root``/``f0_analysis`` are injectable for tests only.
    Returns a verdict with ``all_match`` and the verified digests (never raises on mismatch)."""
    jobs = dict(P0_JUL_JOBS) if jobs is None else {k: Path(v) for k, v in jobs.items()}
    pins = dict(P0_JUL_PINS) if pins is None else {k: dict(v) for k, v in pins.items()}
    root = Path(C.REPO) if root is None else Path(root)
    analysis_path = Path(F1.F0_ANALYSIS_JSON) if f0_analysis is None else Path(f0_analysis)
    analysis_sha_pin = F1.F0_ANALYSIS_SHA256 if f0_analysis_sha256 is None else str(f0_analysis_sha256)
    out: dict[str, Any] = {"files": dict(P0_JUL_FILES), "receipt_schema": P0_JUL_RECEIPT_SCHEMA}
    # F0 analysis pin (the chain anchor for summary/trace digests)
    analysis_rec: dict[str, Any] = {"path": C.rel(analysis_path) if analysis_path.is_absolute() else str(analysis_path), "pinned": analysis_sha_pin, "problems": []}
    analysis_jobs: dict[str, Any] = {}
    if analysis_path.is_symlink() or not analysis_path.is_file():
        analysis_rec["problems"].append("F0 analysis missing or symlink")
    else:
        analysis_rec["sha256"] = C.sha256_file(analysis_path)
        if analysis_rec["sha256"] != analysis_sha_pin:
            analysis_rec["problems"].append("F0 analysis digest differs from its pin")
        else:
            try:
                payload = C.read_json(analysis_path)
                for rec in payload.get("jobs", []) if isinstance(payload, dict) else []:
                    r = rec.get("receipt") if isinstance(rec, dict) else None
                    jid = rec.get("job_id") if isinstance(rec, dict) else None
                    if isinstance(r, dict) and isinstance(jid, str):
                        analysis_jobs[jid] = {"summary_sha256": r.get("summary_sha256"), "trace_sha256": r.get("trace_sha256"), "output_dir": rec.get("output_dir")}
            except Exception as exc:  # noqa: BLE001
                analysis_rec["problems"].append(f"F0 analysis unreadable: {type(exc).__name__}")
    analysis_rec["ok"] = not analysis_rec["problems"]
    out["f0_analysis"] = analysis_rec
    all_ok = analysis_rec["ok"]
    for start in STARTS:
        pin = pins.get(start)
        job_dir = jobs.get(start)
        rec: dict[str, Any] = {"start": start, "problems": [], "files": {}}
        if pin is None or job_dir is None:
            rec["problems"].append("no pin/job for this start")
            rec["match"] = False
            out[start] = rec
            all_ok = False
            continue
        rec["job_id"] = pin["job_id"]
        rec["job_dir"] = pin["job_dir"]
        rec["problems"] += _path_problems(job_dir, root, pin["job_dir"], kind="dir")
        disk: dict[str, str | None] = {}
        for key, name in P0_JUL_FILES.items():
            fpath = job_dir / name
            frec: dict[str, Any] = {"path": f"{pin['job_dir']}/{name}", "pinned": pin[f"{key}_sha256"], "problems": _path_problems(fpath, root, f"{pin['job_dir']}/{name}", kind=f"{key} file")}
            if fpath.is_file() and not fpath.is_symlink():
                frec["sha256"] = C.sha256_file(fpath)
                if frec["sha256"] != frec["pinned"]:
                    frec["problems"].append(f"{key} digest {frec['sha256'][:12]}… != pinned {frec['pinned'][:12]}…")
            disk[key] = frec.get("sha256")
            frec["match"] = not frec["problems"]
            rec["files"][key] = frec
            rec["problems"] += frec["problems"]
        # receipt internal consistency (only meaningful when the receipt file itself is readable)
        consistency: dict[str, Any] = {}
        rpath = job_dir / P0_JUL_FILES["receipt"]
        if rpath.is_file() and not rpath.is_symlink():
            try:
                receipt = C.read_json(rpath)
            except Exception as exc:  # noqa: BLE001
                receipt = None
                rec["problems"].append(f"receipt unreadable: {type(exc).__name__}")
            if isinstance(receipt, dict):
                expected = {"schema_version": P0_JUL_RECEIPT_SCHEMA, "job_id": pin["job_id"], "status": "ok", "returncode": 0, "candidate": INIT_PRIMARY["name"], "runtime": F1.TARGET_RUNTIME, "seed": DET_SEED, "start": start, "action_selection": "deterministic", "steps": int(pin["steps"]), "output_dir": pin["job_dir"], "summary_sha256": disk.get("summary"), "trace_sha256": disk.get("trace"), "module_state_sha256": INIT_PRIMARY["module_state_sha256"], "config_sha256": RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": ROLLOUT_EVAL_SHA256_PINNED}
                for k, v in expected.items():
                    ok = receipt.get(k) == v
                    consistency[k] = {"receipt": receipt.get(k), "expected": v, "ok": ok}
                    if not ok:
                        rec["problems"].append(f"receipt.{k} inconsistent: {receipt.get(k)!r} != {v!r}")
            elif receipt is not None:
                rec["problems"].append("receipt is not a mapping")
        rec["receipt_consistency"] = consistency
        # summary + reset content
        spath, zpath = job_dir / P0_JUL_FILES["summary"], job_dir / P0_JUL_FILES["reset"]
        summary_checks: dict[str, Any] = {}
        if spath.is_file() and not spath.is_symlink():
            try:
                summary = C.read_json(spath)
                exp_s = {"episode_start_offset_s": float(EXACT_STARTS[start]), "action_seed": DET_SEED, "action_selection": "deterministic", "steps": int(pin["steps"]), "n_actor": ENV_ACTOR_WIDTH}
                for k, v in exp_s.items():
                    ok = summary.get(k) == v
                    summary_checks[k] = {"summary": summary.get(k), "expected": v, "ok": ok}
                    if not ok:
                        rec["problems"].append(f"summary.{k} inconsistent: {summary.get(k)!r} != {v!r}")
            except Exception as exc:  # noqa: BLE001
                rec["problems"].append(f"summary unreadable: {type(exc).__name__}")
        if zpath.is_file() and not zpath.is_symlink():
            try:
                reset = C.read_json(zpath)
                t = reset.get("time") if isinstance(reset, dict) else None
                if not isinstance(t, (int, float)) or isinstance(t, bool):
                    rec["problems"].append("reset snapshot without a numeric 'time'")
            except Exception as exc:  # noqa: BLE001
                rec["problems"].append(f"reset snapshot unreadable: {type(exc).__name__}")
        rec["summary_consistency"] = summary_checks
        # F0 analysis chain: the pinned analysis must record the same summary/trace digests for this job
        chain = analysis_jobs.get(pin["job_id"])
        rec["f0_analysis_record"] = {"found": chain is not None, "summary_sha256": chain.get("summary_sha256") if chain else None, "trace_sha256": chain.get("trace_sha256") if chain else None}
        if analysis_rec["ok"]:
            if chain is None:
                rec["problems"].append("job not recorded in the pinned F0 analysis")
            elif chain.get("summary_sha256") != pin["summary_sha256"] or chain.get("trace_sha256") != pin["trace_sha256"]:
                rec["problems"].append("pinned F0 analysis records different summary/trace digests for this job")
        rec["match"] = not rec["problems"]
        all_ok = all_ok and rec["match"]
        out[start] = rec
    out["all_match"] = bool(all_ok)
    return out


def require_p0_jul_pins(**kwargs: Any) -> dict[str, Any]:
    """``verify_p0_jul_pins`` that raises ``F2RContractError`` (listing every problem) unless ``all_match``."""
    verdict = verify_p0_jul_pins(**kwargs)
    if verdict.get("all_match") is not True:
        problems = {k: v.get("problems") for k, v in verdict.items() if isinstance(v, dict) and v.get("problems")}
        raise F2RContractError(f"P0 JUL_H0 input pins do not verify: {json.dumps(problems, default=str)[:3000]}")
    return verdict


# --- T1R inputs: content pins of the T1 commissioning student and its 3 deterministic rollouts --------
T1_STUDENT = {
    "name": "STUDENT_T1_r1", "module": OUT_REFIT / "T1" / "round_1" / "rl_module_student",
    "actor_digest": "7f45a7abd2bcc310bd56fa8cf6dcc830f585d562a3f9f07637420278a98e3b37",
    "module_state_sha256": "ede70c0a732ff33dfb01d0ad04b9e6a04f8615bd293618c02be1650bcb35fc26",
    "role": "T1 commissioning student (u_T both joints, anchors only); provider of the on-policy task states of T1R, NEVER the T1R init",
}
T1_JOB_FILES = {"summary": "rollout_summary.json", "trace": "rollout_policy_trace.json", "reset": "rollout_reset_diagnostics.json", "receipt": "f2r_receipt.json"}
T1_JOBS = {s: OUT_ROLLOUTS / "T1" / "round_1" / f"STUDENT_T1_r1__v3_canonical__{s}__det" for s in STARTS}
T1_JOB_PINS: dict[str, dict[str, Any]] = {
    "minus020": {"job_id": "STUDENT_T1_r1__v3_canonical__minus020__det", "steps": 500, "summary_sha256": "7e4330616cec39d1c5c3d06bf38cb9a695c8052603349bdfb93b1f6e51e44566", "trace_sha256": "b7f9accd1036b3e6dc92ca50e64830c23ea97e38c489354adcfe2370e9cda543", "reset_sha256": "42d07621fcede8a5d59e7365be002ce4ece548687829013d77b9d93db6d96cc7", "receipt_sha256": "37812ade65d7d018564f0bb2917683fd12cd4b1cee3ab4de922060ad59201fcb"},
    "nominal": {"job_id": "STUDENT_T1_r1__v3_canonical__nominal__det", "steps": 500, "summary_sha256": "146fd20cd1c7305709e8e410cf0b9f6ea164f71ff791e3f86e029856c7c46ae3", "trace_sha256": "5b0bfa59a7012c45f70a952cb53944abba2f3177056212537cadee59847f28be", "reset_sha256": "3d3b8dfa9164b2e63f0ba6ca795f2abee41ef6c8bd8929e19368ad83a325ed76", "receipt_sha256": "4783ad83e01404b0f14d63fc3218f36b59759072ecb08b649c6b4a34c7b6caf3"},
    "plus020": {"job_id": "STUDENT_T1_r1__v3_canonical__plus020__det", "steps": 500, "summary_sha256": "226c1bd7a8f783147ffcfc0436e99d9b706e9b47cee3c1a6114478a550ddc549", "trace_sha256": "908dafe4be7720c044741e0e6047e1bc4a225b081e2cf568272d9a17dcd8b9ff", "reset_sha256": "614e0e5c55fc7f6109418b80d1ed82f494c6da72533efc6a410145846ccc38de", "receipt_sha256": "9e8f7770ab088b08b39a178ec10d230d72985f765d769cc6a6380689090ec336"},
}
for _s, _pin in T1_JOB_PINS.items():
    _pin["job_dir"] = C.rel(T1_JOBS[_s])


def verify_t1_pins(*, jobs: Mapping[str, Path] | None = None, pins: Mapping[str, Mapping[str, Any]] | None = None, root: Path | None = None, student_module: Path | None = None) -> dict[str, Any]:
    """Content-pin + receipt verification of the T1 commissioning student module and its three
    deterministic rollouts (the on-policy task states of T1R), read-only and fail-closed (verdict with
    ``all_match``; never raises on mismatch).  Per job: directory and the four files regular,
    non-symlinked, at the expected relative path, with the pinned SHA-256; the ``f2r.1`` receipt
    consistent (status ok, rc 0, seed 123, deterministic, candidate/phase/round, steps, summary/trace
    digests = disk, ``module_state_sha256`` = pinned T1 student file digest, ``config_sha256`` =
    runtime pin, ``rollout_eval_sha256`` = production pin); the student ``module_state.pkl`` hashes to
    its pin and its actor digest to the pinned actor digest."""
    jobs = dict(T1_JOBS) if jobs is None else {k: Path(v) for k, v in jobs.items()}
    pins = dict(T1_JOB_PINS) if pins is None else {k: dict(v) for k, v in pins.items()}
    root = Path(C.REPO) if root is None else Path(root)
    module = Path(T1_STUDENT["module"]) if student_module is None else Path(student_module)
    out: dict[str, Any] = {"files": dict(T1_JOB_FILES)}
    srec: dict[str, Any] = {"module": C.rel(module) if module.is_absolute() else str(module), "problems": []}
    state_path = module / "module_state.pkl"
    if state_path.is_symlink() or not state_path.is_file():
        srec["problems"].append("T1 student module_state.pkl missing or symlink")
    else:
        srec["module_state_sha256"] = {"pinned": T1_STUDENT["module_state_sha256"], "disk": C.sha256_file(state_path)}
        if srec["module_state_sha256"]["disk"] != T1_STUDENT["module_state_sha256"]:
            srec["problems"].append("T1 student module_state.pkl digest differs from its pin")
        else:
            sys.path.insert(0, str(BASELINE_DIR)) if str(BASELINE_DIR) not in sys.path else None
            import warm_start as W  # production, import only

            srec["actor_digest"] = {"pinned": T1_STUDENT["actor_digest"], "disk": W.actor_state_digest(W.load_module_state(module))}
            if srec["actor_digest"]["disk"] != T1_STUDENT["actor_digest"]:
                srec["problems"].append("T1 student actor digest differs from its pin")
    srec["match"] = not srec["problems"]
    out["student"] = srec
    all_ok = srec["match"]
    for start in STARTS:
        pin = pins.get(start); job_dir = jobs.get(start)
        rec: dict[str, Any] = {"start": start, "problems": [], "files": {}}
        if pin is None or job_dir is None:
            rec["problems"].append("no pin/job for this start"); rec["match"] = False; out[start] = rec; all_ok = False
            continue
        rec["job_id"] = pin["job_id"]; rec["job_dir"] = pin["job_dir"]
        rec["problems"] += _path_problems(job_dir, root, pin["job_dir"], kind="dir")
        disk: dict[str, str | None] = {}
        for key, name in T1_JOB_FILES.items():
            fpath = job_dir / name
            frec: dict[str, Any] = {"path": f"{pin['job_dir']}/{name}", "pinned": pin[f"{key}_sha256"], "problems": _path_problems(fpath, root, f"{pin['job_dir']}/{name}", kind=f"{key} file")}
            if fpath.is_file() and not fpath.is_symlink():
                frec["sha256"] = C.sha256_file(fpath)
                if frec["sha256"] != frec["pinned"]:
                    frec["problems"].append(f"{key} digest {frec['sha256'][:12]}… != pinned {frec['pinned'][:12]}…")
            disk[key] = frec.get("sha256"); frec["match"] = not frec["problems"]; rec["files"][key] = frec; rec["problems"] += frec["problems"]
        consistency: dict[str, Any] = {}
        rpath = job_dir / T1_JOB_FILES["receipt"]
        if rpath.is_file() and not rpath.is_symlink():
            try:
                receipt = C.read_json(rpath)
            except Exception as exc:  # noqa: BLE001
                receipt = None; rec["problems"].append(f"receipt unreadable: {type(exc).__name__}")
            if isinstance(receipt, dict):
                expected = {"schema_version": "f2r.1", "job_id": pin["job_id"], "status": "ok", "returncode": 0, "candidate": T1_STUDENT["name"], "phase": "T1", "round": 1, "purpose": "det", "seed": DET_SEED, "start": start, "action_selection": "deterministic", "runtime": F1.TARGET_RUNTIME, "output_dir": pin["job_dir"], "summary_sha256": disk.get("summary"), "trace_sha256": disk.get("trace"), "module_state_sha256": T1_STUDENT["module_state_sha256"], "config_sha256": RUNTIME_CONFIG_SHA256, "rollout_eval_sha256": ROLLOUT_EVAL_SHA256_PINNED}
                for k, v in expected.items():
                    ok = receipt.get(k) == v; consistency[k] = {"receipt": receipt.get(k), "expected": v, "ok": ok}
                    if not ok:
                        rec["problems"].append(f"receipt.{k} inconsistent: {receipt.get(k)!r} != {v!r}")
            elif receipt is not None:
                rec["problems"].append("receipt is not a mapping")
        rec["receipt_consistency"] = consistency
        spath = job_dir / T1_JOB_FILES["summary"]
        if spath.is_file() and not spath.is_symlink():
            try:
                summary = C.read_json(spath)
                for k, v in {"episode_start_offset_s": float(EXACT_STARTS[start]), "action_seed": DET_SEED, "action_selection": "deterministic", "steps": int(pin["steps"]), "n_actor": ENV_ACTOR_WIDTH}.items():
                    if summary.get(k) != v:
                        rec["problems"].append(f"summary.{k} inconsistent: {summary.get(k)!r} != {v!r}")
            except Exception as exc:  # noqa: BLE001
                rec["problems"].append(f"summary unreadable: {type(exc).__name__}")
        rec["match"] = not rec["problems"]; all_ok = all_ok and rec["match"]; out[start] = rec
    out["all_match"] = bool(all_ok)
    return out


def require_t1_pins(**kwargs: Any) -> dict[str, Any]:
    verdict = verify_t1_pins(**kwargs)
    if verdict.get("all_match") is not True:
        problems = {k: v.get("problems") for k, v in verdict.items() if isinstance(v, dict) and v.get("problems")}
        raise F2RContractError(f"T1 student/rollout pins do not verify: {json.dumps(problems, default=str)[:3000]}")
    return verdict


def verify_actor_pins() -> dict[str, Any]:
    """Re-hash init / negative control / teacher module files and actor digests (read-only).
    Each record carries ``module_state_sha256`` (file bytes) and ``actor_digest`` (parameter
    content) separately, see ``DIGEST_SEMANTICS``; ``match`` requires both."""
    sys.path.insert(0, str(BASELINE_DIR)) if str(BASELINE_DIR) not in sys.path else None
    import warm_start as W  # production, import only

    out: dict[str, Any] = {}
    for label, spec in (("init_primary", INIT_PRIMARY), ("negative_control", NEGATIVE_CONTROL), ("teacher", TEACHER)):
        module = Path(spec["module"])
        state_path = module / "module_state.pkl"
        rec: dict[str, Any] = {"name": spec["name"], "module": C.rel(module), "role": spec["role"], "exists": state_path.is_file()}
        if state_path.is_file():
            rec["module_state_sha256"] = {"pinned": spec["module_state_sha256"], "disk": C.sha256_file(state_path)}
            state = W.load_module_state(module)
            rec["actor_digest"] = {"pinned": spec["actor_digest"], "disk": W.actor_state_digest(state)}
            w1 = state["pi.0.0.weight"]
            rec["width"] = int(w1.shape[1])
            rec["clock_columns_zero"] = bool((w1[:, list(CLOCK_COLUMNS)] == 0).all()) if spec["width"] == ENV_ACTOR_WIDTH else None
            rec["digests_distinct"] = rec["module_state_sha256"]["disk"] != rec["actor_digest"]["disk"]
            rec["match"] = rec["module_state_sha256"]["pinned"] == rec["module_state_sha256"]["disk"] and rec["actor_digest"]["pinned"] == rec["actor_digest"]["disk"] and rec["width"] == spec["width"] and rec["digests_distinct"]
        else:
            rec["match"] = False
        out[label] = rec
    out["digest_semantics"] = dict(DIGEST_SEMANTICS)
    out["all_match"] = all(v["match"] for v in out.values() if isinstance(v, dict) and "match" in v)
    return out


# --- portable temp dirs and exclusive (no-clobber, atomic) artefact writers ---------------------


def portable_tempdir(prefix: str) -> Path:
    """``tempfile.mkdtemp`` under the platform temp root, returned **resolved** (no symlinked
    components: macOS ``/var`` -> ``/private/var``), so the F0 closure validator — which refuses
    symlinks — accepts it on macOS, Linux and Windows alike.  No hard-coded path."""
    return Path(tempfile.mkdtemp(prefix=prefix)).resolve()


def reserve_exclusive(path: Path) -> Path:
    """Create ``path`` with ``O_CREAT | O_EXCL`` (fails if it exists, on every OS); parents created."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    os.close(fd)
    return path


def _atomic_fill(path: Path, data: bytes) -> Path:
    """Fill a reserved path atomically: write a sibling temp file, fsync, ``os.replace`` onto the reservation."""
    path = Path(path)
    tmp = path.with_name(f"{path.name}.part-{os.getpid()}")
    with open(tmp, "wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    return path


def write_bytes_exclusive(path: Path, data: bytes) -> Path:
    """No-clobber + atomic: the name is reserved exclusively, then filled atomically (readers never see a partial file)."""
    reserve_exclusive(path)
    return _atomic_fill(path, data)


def write_json_exclusive(path: Path, payload: Any) -> Path:
    return write_bytes_exclusive(path, json.dumps(payload, indent=2, sort_keys=False, default=str).encode("utf-8"))


def write_text_exclusive(path: Path, text: str) -> Path:
    return write_bytes_exclusive(path, str(text).encode("utf-8"))


def reserve_unique_set(directory: Path, stem: str, suffixes: Sequence[str], *, max_attempts: int = 100) -> dict[str, Path]:
    """Reserve ``<stem><suffix>`` for every suffix exclusively; on any collision (e.g. two runs in
    the same second) retry with ``<stem>_01``, ``_02``, ... so an existing artefact is never reused
    or overwritten.  Returns the reserved (empty) paths keyed by suffix."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for attempt in range(max_attempts):
        base = stem if attempt == 0 else f"{stem}_{attempt:02d}"
        reserved: dict[str, Path] = {}
        try:
            for suffix in suffixes:
                reserved[suffix] = reserve_exclusive(directory / f"{base}{suffix}")
            return reserved
        except FileExistsError:
            for p in reserved.values():
                p.unlink(missing_ok=True)
            continue
    raise F2RContractError(f"could not reserve a unique artefact set for {stem!r} in {directory} after {max_attempts} attempts")


def reserve_unique_names(directory: Path, tag: str, templates: Mapping[str, str], *, max_attempts: int = 100) -> dict[str, Any]:
    """Reserve a whole SET of differently named artefacts sharing one ``tag`` (e.g. a launch
    stamp): every template is formatted with ``{tag}`` and created exclusively; on any
    collision the entire attempt is rolled back (no orphan) and the tag becomes
    ``<tag>_01``, ``<tag>_02``, ...  Returns ``{"tag": <tag used>, "paths": {key: Path}}``."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for attempt in range(max_attempts):
        base = tag if attempt == 0 else f"{tag}_{attempt:02d}"
        reserved: dict[str, Path] = {}
        try:
            for key, template in templates.items():
                reserved[key] = reserve_exclusive(directory / str(template).format(tag=base))
            return {"tag": base, "paths": reserved}
        except FileExistsError:
            for q in reserved.values():
                q.unlink(missing_ok=True)
            continue
    raise F2RContractError(f"could not reserve a unique artefact set for tag {tag!r} in {directory} after {max_attempts} attempts")


def unique_artifact_path(directory: Path, stem: str, suffix: str) -> Path:
    """Single reserved unique path (``<stem><suffix>``, then ``<stem>_01<suffix>`` ...)."""
    return reserve_unique_set(directory, stem, (suffix,))[suffix]


def ensure_out_dirs() -> None:
    for d in (OUT_ROOT, OUT_MANIFEST, OUT_ROLLOUTS, OUT_DATASETS, OUT_CACHE, OUT_REFIT, OUT_P0, OUT_METRICS, OUT_GATE, OUT_LOGS):
        d.mkdir(parents=True, exist_ok=True)
