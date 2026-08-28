"""V26C J5 - FRESH deterministic closed-loop revalidation of the J4 recovery product.

WHAT IT IS
    The strictest possible replica of J3, with ONE operational difference: the actor is the pinned
    J4 recovery checkpoint instead of the J2 base checkpoint. Same pinned runtime config, same
    hardened env builder, same seed 123, same 500 steps, same deterministic mean with no sigma, the
    same RAW action stepped into the environment, and EXACTLY the same COMMON gate, kinematic
    quality and telemetry-integrity invariant. Nothing is added, moved or relaxed.

THE MASK MOVED AT J4, THE ACTOR DID NOT
    J2/J3 hard-zeroed the clock AND the controller memory. The recovery stage lifted the ten
    controller-memory columns, so at J5 only the clock columns [0, 1] are expected at zero and
    columns [25..34] MUST be non-zero. The actor is still one 35D student inside an 84D asymmetric
    observation; it is not widened and there is no standalone 25D artefact.

WHAT IT PRESERVES
    The J1 FAIL, the J3 FAIL and the J4 technical/offline PASS all stand. J5 re-scores none of them
    and retroactively promotes none of them, whatever it finds.

WHAT IT DOES NOT DO
    No retry. No fit, optimizer, DAgger round or progressive iteration. No sigma. No critic, no PPO.
    No ex-novo. No promotion: a J5 PASS confers none, and authorises no next stage.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
TG = REPO / "Trajectory Generator"
BASELINE = TG / "baseline_MLP"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1  # noqa: E402  the hardened env builder, trace parser and summariser
import v26c_j3_closed_loop as J3  # noqa: E402  the closed-loop semantics this stage replicates


class J5Error(RuntimeError):
    pass


STAGE = "V26C-J5-REVALIDATION"
RECEIPT_NAME = "v26c_j5_revalidation_receipt.json"
OUT_ROOT = HERE / "j5_runs"
EXPECTED_LEAF = "j5_revalidation_v26c_2026-08-26_r2"
# r1 aborted inside its FIRST env.step() when -W error promoted a benign SciPy SLSQP
# RuntimeWarning to an exception. It wrote no trace, no kinematics and no receipt, so it carries
# NO verdict: it is a technically invalid attempt, preserved byte-for-byte and never reused.
R1_LEAF_NAME = "j5_revalidation_v26c_2026-08-26_r1"
R1_LEAF = OUT_ROOT / R1_LEAF_NAME
PRODUCTION_COMMAND_FORBIDS = ("-W error",)

# ------------------------------------------------------------------ the J5 authorisation --------
AMENDMENT = HERE / "v26c_j5_amendment_revalidation.json"
PIN_AMENDMENT = "033061fdd2bc289cdcb431a0c0df04a036bd660e7bda9490ec17870947ebcec0"
# The r2 re-run authorisation. ADDITIVE: it stands beside the amendment above, never replaces it.
AMENDMENT_R2 = HERE / "v26c_j5_amendment_r2_rerun.json"
PIN_AMENDMENT_R2 = "246caf8f61f38cd7cb5640dbec601d355e02ee46cce2a95a63b0b7874f4d17d0"

# ------------------------------------------------------------------ the operational actor -------
J4_LEAF = HERE / "j4_runs" / "j4_recovery_v26c_2026-08-26_r1"
J4_MODULE_DIR = J4_LEAF / "rl_module"
PIN_J4: dict[str, str] = {
    "history.json":
        "e8a07fdfb77656a1a9f3c4d158b949f8a793c015106396015761748d50fb6b87",
    "j4_fit_original.log":
        "88e9312626d23ce49685576e263df1b93c0f4f8d077415cc1d0bdaadb188dbcf",
    "recovery_dataset.npz":
        "28eda638bd5441698611fce7f9d9b65660b3e21581805434f41c57d1a745357e",
    "rl_module/actor_feature_manifest.json":
        "3454a6de085a14510874af8222e266eacac1dd194460e26902ffa29606df3c03",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "rl_module/module_state.pkl":
        "14a3630f757a5da2055eb754f6249fad8e7989a6d5e6c18f526c76415dad31aa",
    "v26c_j4_recovery_receipt.json":
        "887f2b94e5a22a9c8013497164db090c2e910804730c5f3aa9c4f4d5f308c354",
}
PIN_J4_RUNNER = "24dceef98242cc9e02f63d0442e8361bae32994c0829859b1747f0e7091a6841"

# ------------------------------------------------------------------ preserved lineage -----------
J3_LEAF = J3.OUT_ROOT / "j3_base_v26c_2026-08-26_r1"
PIN_J3_RECEIPT = "34d856b0b4acabd000a1e6257767c6049a1f9f2147eb99d6f3801ca7559ff422"
PIN_J3_TRACE = "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae"
PIN_J3_RUNNER = "f14e7ce06161228ef81f44f38863269b283002661d97e7995d48106152850759"
J1_LEAF = J1.OUT_ROOT / "j1_nominal_v26c_2026-08-26_r1"
PIN_J1_RECEIPT = J3.PIN_J1_RECEIPT
PIN_J1_TRACE = "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3"
PIN_J1_AMENDMENT_FILE = J3.PIN_J1_AMENDMENT
PIN_J4_AMENDMENT = "fed5b81666782902ed4ab0187da457cdfbf0516dd676c049fdfe41e48d21614f"

# ------------------------------------------------------------------ contract, inherited ---------
ACTOR_WIDTH = J3.ACTOR_WIDTH                       # 35
FULL_OBS_WIDTH = J3.FULL_OBS_WIDTH                 # 84
PIN_RUNTIME_CONFIG_SHA = J3.PIN_RUNTIME_CONFIG_SHA
EXPECTED_STEPS = J3.EXPECTED_STEPS                 # 500
ROLLOUT_SEED = J3.ROLLOUT_SEED                     # 123
# The ONLY contract difference from J3: the controller memory is live at this stage.
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
EXPECTED_ZERO_COLUMNS = list(CLOCK_COLUMNS)
# The actor is exactly two mirrored input layers of 256x35. Any third 35D input layer, any other
# shape, or a divergence between the two aliases is a different network and is refused.
EXPECTED_INPUT_LAYERS = ("pi.0.0.weight", "pi_encoder.0.weight")
EXPECTED_INPUT_SHAPE = (256, ACTOR_WIDTH)

# ------------------------------------------------------------------ the gates, verbatim ---------
J5_COMMON_GATE: dict[str, Any] = dict(J3.J3_COMMON_GATE)
J5_KINEMATIC_GATE: dict[str, Any] = dict(J3.J3_KINEMATIC_GATE)
TELEMETRY_INTEGRITY_INVARIANT = dict(J3.TELEMETRY_INTEGRITY_INVARIANT)
DIAGNOSTIC_NOT_BINDING = tuple(J3.DIAGNOSTIC_NOT_BINDING)

# ------------------------------------------------------------------ comparison references -------
# EVIDENCE ONLY. A comparison is never a gate and never excuses a FAIL.
J3_MAX_PENETRATION_M = 0.02704966381076714
J3_STEPS_ABOVE_SOFT = 106
J1_MAX_PENETRATION_M = 0.02294380435912411
J1_STEPS_ABOVE_SOFT = 97
SOFT_DIAGNOSTIC_M = 0.020
HARD_GUARD_M = 0.028


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


# ================================================================ inputs =========================

def verify_j4_actor() -> dict[str, Any]:
    """The eight J4 artefacts, by exact hash. Nothing else may be rolled out."""
    if not J4_LEAF.is_dir():
        raise J5Error(f"the J4 leaf is missing: {J4_LEAF}")
    present = sorted(str(q.relative_to(J4_LEAF)) for q in J4_LEAF.rglob("*") if q.is_file())
    if present != sorted(PIN_J4):
        raise J5Error(f"the J4 leaf holds {present}, expected exactly {sorted(PIN_J4)}")
    got: dict[str, str] = {}
    for rel, pin in PIN_J4.items():
        h = _sha_file(J4_LEAF / rel)
        if h != pin:
            raise J5Error(f"the J4 artefact {rel} changed: {h} != {pin}")
        got[rel] = h
    receipt = json.loads((J4_LEAF / "v26c_j4_recovery_receipt.json").read_text())
    if receipt["verdict"] != "PASS" or "POST-CRASH FINALIZED" not in receipt["verdict_kind"]:
        raise J5Error("the J4 receipt is not the finalised technical PASS")
    if receipt["closed_loop_qualified"] is not False or receipt["deployable"] is not False \
            or receipt["promotion"] != "NONE":
        raise J5Error("the J4 receipt claims closed-loop qualification, deployability or promotion")
    return {"leaf": _rel(J4_LEAF), "artefacts_sha256": got,
            "verdict": receipt["verdict"], "verdict_kind": receipt["verdict_kind"],
            "selection": receipt["selection"],
            "recovery_rmse": receipt["pre_write_verification"]["recovery_vs_teacher"],
            "controller_column_norms":
                receipt["pre_write_verification"]["controller_column_norms"],
            "role": "the ONLY actor J5 may roll out"}


def verify_lineage() -> dict[str, Any]:
    """J1 FAIL, J3 FAIL and the J4 technical PASS are preserved. J5 promotes none of them."""
    checks = {
        "j1_receipt": (J1_LEAF / "v26c_j1_collection_receipt.json", PIN_J1_RECEIPT),
        "j1_trace": (J1_LEAF / "teacher_trace.json", PIN_J1_TRACE),
        "j3_receipt": (J3_LEAF / "v26c_j3_closed_loop_receipt.json", PIN_J3_RECEIPT),
        "j3_trace": (J3_LEAF / "j3_trace.json", PIN_J3_TRACE),
        "j3_runner": (HERE / "v26c_j3_closed_loop.py", PIN_J3_RUNNER),
        "j4_runner": (HERE / "v26c_j4_recovery.py", PIN_J4_RUNNER),
        "j4_amendment": (HERE / "v26c_j3_amendment_recovery_only.json", PIN_J4_AMENDMENT),
        "j1_amendment": (HERE / "v26c_j1_amendment_soft_fail.json", PIN_J1_AMENDMENT_FILE),
    }
    got: dict[str, str] = {}
    for label, (path, pin) in checks.items():
        if not path.is_file():
            raise J5Error(f"the pinned {label} is missing: {path}")
        h = _sha_file(path)
        if h != pin:
            raise J5Error(f"the pinned {label} changed: {h} != {pin}")
        got[label] = h
    manifest_checked: dict[str, str] = {}
    amendment_shas: dict[str, str] = {}
    for label, path, pin in (("j5_authorisation", AMENDMENT, PIN_AMENDMENT),
                             ("j5_r2_rerun", AMENDMENT_R2, PIN_AMENDMENT_R2)):
        if not path.is_file():
            raise J5Error(f"the {label} amendment is missing: {path}")
        h = _sha_file(path)
        if h != pin:
            raise J5Error(f"the {label} amendment changed: {h} != {pin}")
        amendment_shas[label] = h
        # each amendment is itself a manifest: every artefact it pins is verified here
        entries = json.loads(path.read_text())["pinned_artefacts_sha256"]
        if not entries:
            raise J5Error(f"the {label} amendment declares no pinned artefacts")
        for rel, artefact_pin in entries.items():
            target = HERE / rel
            if not target.is_file():
                raise J5Error(f"the {label} amendment pins {rel}, which is missing")
            got_h = _sha_file(target)
            if got_h != artefact_pin:
                raise J5Error(f"the amendment-pinned artefact {rel} changed: {got_h} != "
                              f"{artefact_pin}")
            if rel in manifest_checked and manifest_checked[rel] != got_h:
                raise J5Error(f"the two amendments disagree on {rel}")
            manifest_checked[rel] = got_h
    amendment_sha = amendment_shas["j5_authorisation"]
    j1 = json.loads((J1_LEAF / "v26c_j1_collection_receipt.json").read_text())
    j3 = json.loads((J3_LEAF / "v26c_j3_closed_loop_receipt.json").read_text())
    j4 = json.loads((J4_LEAF / "v26c_j4_recovery_receipt.json").read_text())
    if j1["verdict"] != "FAIL" or j3["verdict"] != "FAIL":
        raise J5Error("the J1 or J3 receipt no longer records its original FAIL")
    if j3["gate"]["failed"] != ["max_penetration_m"]:
        raise J5Error("the J3 receipt no longer records its original failed gate")
    return {
        "pins_sha256": got, "j5_amendment_sha256": amendment_sha,
        "j5_amendment_r2_sha256": amendment_shas["j5_r2_rerun"],
        "amendments_verified": sorted(amendment_shas),
        "j5_amendment_manifest_sha256": manifest_checked,
        "j5_amendment_manifest_entries": len(manifest_checked),
        "j1_verdict": j1["verdict"],
        "j1_max_penetration_m": j1["summary"]["max_penetration_m"],
        "j3_verdict": j3["verdict"], "j3_failed": j3["gate"]["failed"],
        "j3_max_penetration_m": j3["summary"]["max_penetration_m"],
        "j4_verdict": j4["verdict"], "j4_verdict_kind": j4["verdict_kind"],
        "statement": "the J1 FAIL, the J3 FAIL and the J4 technical/offline PASS all STAND. J5 "
                     "neither re-scores nor retroactively promotes any of them, whatever it finds.",
    }


def verify_r1_preserved() -> dict[str, Any]:
    """The aborted r1 attempt is preserved BYTE-FOR-BYTE and carries no verdict."""
    entries = json.loads(AMENDMENT_R2.read_text())["pinned_artefacts_sha256"]
    prefix = f"j5_runs/{R1_LEAF_NAME}/"
    pinned = {rel[len(prefix):]: pin for rel, pin in entries.items() if rel.startswith(prefix)}
    if not pinned:
        raise J5Error("the r2 amendment pins no r1 artefact; the aborted attempt is unprotected")
    if not R1_LEAF.is_dir():
        raise J5Error(f"the preserved r1 leaf is missing: {R1_LEAF}")
    present = sorted(q.relative_to(R1_LEAF).as_posix() for q in R1_LEAF.rglob("*") if q.is_file())
    if present != sorted(pinned):
        raise J5Error(f"the r1 leaf holds {len(present)} files, expected exactly {len(pinned)}; "
                      "it must stay byte-for-byte as the aborted attempt left it")
    for rel, pin in pinned.items():
        h = _sha_file(R1_LEAF / rel)
        if h != pin:
            raise J5Error(f"the preserved r1 artefact {rel} changed: {h} != {pin}")
    for forbidden in (RECEIPT_NAME, "j5_trace.json", "j5_kinematics.npz"):
        if (R1_LEAF / forbidden).exists():
            raise J5Error(f"the r1 leaf now holds {forbidden}; it aborted before writing any, so "
                          "its presence means the attempt was altered")
    return {"leaf": _rel(R1_LEAF), "files": len(pinned), "sha256": dict(pinned),
            "status": "TECHNICALLY INVALID - NO VERDICT",
            "steps_completed": 0,
            "cause": "a benign SciPy SLSQP RuntimeWarning promoted to a fatal exception by "
                     "-W error, inside the FIRST env.step()",
            "preserved_byte_for_byte": True,
            "never_reused": "r1 is never read as evidence about the actor and never written to"}


def verify_warning_policy() -> dict[str, Any]:
    """The production command must NOT run under -W error: r1 proved what that costs."""
    active = [opt for opt in sys.warnoptions if opt.split(":")[0].strip() == "error"]
    return {"forbidden": list(PRODUCTION_COMMAND_FORBIDS), "sys_warnoptions": list(sys.warnoptions),
            "error_filter_active": bool(active),
            "why": "the production simulator legitimately emits SciPy RuntimeWarnings from the "
                   "Static Optimization SLSQP fallback; promoting them to exceptions aborts a "
                   "physically valid rollout, as the r1 attempt showed"}


def build_env_config(*, output_dir: Path | None = None) -> dict[str, Any]:
    """The FULL env_config, from the SAME hardened builder J1 and J3 use."""
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_dir)
    J1.verify_env_config(env, cfg)
    return env


def actor_feature_manifest() -> tuple[str, ...]:
    manifest = json.loads((J4_MODULE_DIR / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J5Error(f"the pinned J4 manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


def verify_actor_columns(state: Mapping[str, np.ndarray]) -> dict[str, Any]:
    """Exactly two input layers, identical, 256x35; clock hard-zero; controller memory live."""
    inputs = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and np.ndim(v) == 2
                    and np.shape(v)[1] == ACTOR_WIDTH)
    if inputs != sorted(EXPECTED_INPUT_LAYERS):
        raise J5Error(f"the J4 state holds the {ACTOR_WIDTH}D input layers {inputs}, expected "
                      f"exactly {sorted(EXPECTED_INPUT_LAYERS)}")
    report: dict[str, Any] = {}
    for key in inputs:
        W = np.asarray(state[key])
        if W.shape != EXPECTED_INPUT_SHAPE:
            raise J5Error(f"the J4 input layer {key} has shape {W.shape}, expected "
                          f"{EXPECTED_INPUT_SHAPE}")
        zero = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if zero != EXPECTED_ZERO_COLUMNS:
            raise J5Error(f"the J4 input layer {key} has zero columns {zero}; expected "
                          f"{EXPECTED_ZERO_COLUMNS} - only the clock stays hard-zero at J5")
        norms = {int(c): float(np.linalg.norm(W[:, c])) for c in CONTROLLER_COLUMNS}
        dead = sorted(c for c, v in norms.items() if not v > 0.0)
        if dead:
            raise J5Error(f"the J4 input layer {key} has zero-norm controller columns {dead}; "
                          "the recovery stage was supposed to make them live")
        report[key] = {"shape": list(W.shape), "zero_columns": zero, "controller_norms": norms}
    direct, encoder = EXPECTED_INPUT_LAYERS
    if not bool(np.array_equal(np.asarray(state[direct]), np.asarray(state[encoder]))):
        raise J5Error(f"the input-layer aliases {direct} and {encoder} are not bit-identical")
    report["aliases_bit_identical"] = True
    return report


# ================================================================ historical references ==========

# Parsed penetration series, keyed on (resolved path, ALREADY VERIFIED sha256). The hash is part
# of the key, so a mutated file can never be served from the cache: its digest changes and the
# lookup misses.
_PENETRATION_CACHE: dict[tuple[str, str], dict[str, Any]] = {}


def derive_penetration_reference(path: Path, sha256: str, *, label: str,
                                 expected_rows: int = EXPECTED_STEPS) -> dict[str, Any]:
    """Recompute a historical penetration reference FROM the pinned trace. Fail-closed.

    The caller must pass the digest it has already verified; it becomes part of the cache key.
    """
    path = Path(path)
    key = (str(path.resolve()), str(sha256))
    cached = _PENETRATION_CACHE.get(key)
    if cached is not None:
        return dict(cached)
    if not path.is_file():
        raise J5Error(f"the {label} trace is missing: {path}")
    rows = json.loads(path.read_text())
    if not isinstance(rows, list):
        raise J5Error(f"the {label} trace is a {type(rows).__name__}, expected a list")
    if len(rows) != expected_rows:
        raise J5Error(f"the {label} trace holds {len(rows)} rows, expected {expected_rows}")
    series: list[float] = []
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            raise J5Error(f"{label} row {index} is not a mapping")
        if int(row.get("step", -1)) != index:
            raise J5Error(f"the {label} trace is not contiguous at row {index}")
        terms = row.get("reward_terms")
        if not isinstance(terms, Mapping):
            raise J5Error(f"{label} row {index} has no reward_terms mapping")
        if J1.RT_PENETRATION not in terms:
            raise J5Error(f"{label} row {index} has no reward_terms.{J1.RT_PENETRATION}")
        series.append(J1._finite(terms[J1.RT_PENETRATION],
                                 f"{label} row {index}: {J1.RT_PENETRATION}"))
    arr = np.asarray(series, dtype=np.float64)
    derived = {"label": label, "source": _rel(path), "sha256": str(sha256),
               "rows": int(arr.size),
               "max_penetration_m": float(np.max(arr)),
               "steps_above_soft": int(np.sum(arr > SOFT_DIAGNOSTIC_M)),
               "steps_above_hard": int(np.sum(arr > HARD_GUARD_M)),
               "soft_threshold_m": SOFT_DIAGNOSTIC_M,
               "derivation": "recomputed from the pinned trace; no hard-coded value is used"}
    _PENETRATION_CACHE[key] = dict(derived)
    return derived


def historical_references(lineage: Mapping[str, Any]) -> dict[str, Any]:
    """The J3 and J1 references, DERIVED from the pinned traces and cross-checked three ways."""
    pins = lineage["pins_sha256"]
    j3 = derive_penetration_reference(J3_LEAF / "j3_trace.json", pins["j3_trace"], label="j3")
    j1 = derive_penetration_reference(J1_LEAF / "teacher_trace.json", pins["j1_trace"],
                                      label="j1")
    for derived, receipt_value, expected_max, expected_count, name in (
            (j3, lineage["j3_max_penetration_m"], J3_MAX_PENETRATION_M, J3_STEPS_ABOVE_SOFT, "J3"),
            (j1, lineage["j1_max_penetration_m"], J1_MAX_PENETRATION_M, J1_STEPS_ABOVE_SOFT, "J1")):
        if not math.isclose(derived["max_penetration_m"], float(receipt_value),
                            rel_tol=0.0, abs_tol=1e-15):
            raise J5Error(f"the {name} penetration derived from its trace "
                          f"({derived['max_penetration_m']!r}) disagrees with its pinned receipt "
                          f"({receipt_value!r})")
        if not math.isclose(derived["max_penetration_m"], expected_max,
                            rel_tol=0.0, abs_tol=1e-15):
            raise J5Error(f"the {name} penetration derived from its trace "
                          f"({derived['max_penetration_m']!r}) disagrees with the preregistered "
                          f"{expected_max!r}")
        if derived["steps_above_soft"] != expected_count:
            raise J5Error(f"the {name} count of steps above {SOFT_DIAGNOSTIC_M} m derived from "
                          f"its trace ({derived['steps_above_soft']}) disagrees with the "
                          f"preregistered {expected_count}")
    return {"j3": j3, "j1": j1,
            "cross_checked_against": ["the pinned receipt", "the preregistered expectation"],
            "no_silent_fallback": "the hard-coded constants are only an expectation to agree "
                                  "with; the values USED are the derived ones"}


# ================================================================ gates ==========================

def evaluate_gate(summary: Mapping[str, Any], knee: np.ndarray, ankle: np.ndarray
                  ) -> dict[str, Any]:
    """EXACTLY the J3 behavioural gate, called through J3 itself so it cannot drift."""
    return J3.evaluate_gate(summary, knee, ankle)


def telemetry_integrity(summary: Mapping[str, Any]) -> dict[str, Any]:
    return J3.telemetry_integrity(summary)


def overall_verdict(gate: Mapping[str, Any], integrity: Mapping[str, Any]) -> str:
    return J3.overall_verdict(gate, integrity)


def compare_with_history(summary: Mapping[str, Any], penetrations: np.ndarray,
                         references: Mapping[str, Any]) -> dict[str, Any]:
    """J5 vs J3 and J5 vs J1, against DERIVED references. EVIDENCE ONLY, never a gate."""
    p = np.asarray(penetrations, dtype=np.float64)
    if p.size == 0 or not np.all(np.isfinite(p)):
        raise J5Error("the penetration series is empty or not finite")
    for name in ("j3", "j1"):
        block = references.get(name)
        if not isinstance(block, Mapping) or "max_penetration_m" not in block \
                or "steps_above_soft" not in block:
            raise J5Error(f"the derived {name} reference is missing; comparisons never fall back "
                          "to a hard-coded number")
    j5_max = float(np.max(p))
    j5_above = int(np.sum(p > SOFT_DIAGNOSTIC_M))
    if not math.isclose(j5_max, float(summary["max_penetration_m"]), rel_tol=0.0, abs_tol=1e-12):
        raise J5Error("the per-step penetration series disagrees with the summarised maximum")
    j3, j1 = references["j3"], references["j1"]
    return {
        "status": "EVIDENCE ONLY - a comparison is never a gate, never reinterprets a threshold "
                  "and never excuses a FAIL",
        "soft_diagnostic_m": SOFT_DIAGNOSTIC_M, "hard_guard_m": HARD_GUARD_M,
        "references_derived_from_pinned_traces": {"j3": j3, "j1": j1},
        "j5": {"max_penetration_m": j5_max, "steps_above_soft": j5_above,
               "steps_above_hard": int(np.sum(p > HARD_GUARD_M))},
        "vs_j3_base": {"j3_max_penetration_m": j3["max_penetration_m"],
                       "j3_steps_above_soft": j3["steps_above_soft"],
                       "delta_max_penetration_m": j5_max - float(j3["max_penetration_m"]),
                       "delta_steps_above_soft": j5_above - int(j3["steps_above_soft"])},
        "vs_j1_teacher": {"j1_max_penetration_m": j1["max_penetration_m"],
                          "j1_steps_above_soft": j1["steps_above_soft"],
                          "delta_max_penetration_m": j5_max - float(j1["max_penetration_m"]),
                          "delta_steps_above_soft": j5_above - int(j1["steps_above_soft"])},
    }


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: NO environment is constructed, reset or stepped."""
    warnings_policy = verify_warning_policy()
    blockers: list[str] = []
    if warnings_policy["error_filter_active"]:
        blockers.append(
            "INVOCATION-PROTOCOL: python is running with an -W error filter "
            f"({warnings_policy['sys_warnoptions']}). The production simulator emits a standard "
            "SLSQP clipping RuntimeWarning that is non-fatal under Python's default policy; "
            "promoting it to an exception aborted the r1 attempt inside its first step. Re-run "
            "the authorised command WITHOUT -W error.")
    actor = verify_j4_actor()
    lineage = verify_lineage()
    r1 = verify_r1_preserved()
    env = build_env_config()
    names = actor_feature_manifest()
    with (J4_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    columns = verify_actor_columns(state)
    leaves = sorted(q.name for q in OUT_ROOT.iterdir()) if OUT_ROOT.is_dir() else []
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False,
                  "note": "the env factory is imported inside run() only"},
        "authorisation": {
            "authorised_by": "THE USER, explicitly. The architect is gate owner and reviewer.",
            "scope": "exactly one authorised r2 re-run",
            "amendments": {
                "j5_authorisation": {"file": _rel(AMENDMENT),
                                     "sha256": lineage["j5_amendment_sha256"]},
                "j5_r2_rerun": {"file": _rel(AMENDMENT_R2),
                                "sha256": lineage["j5_amendment_r2_sha256"]},
            },
            "amendment_manifest_entries": lineage["j5_amendment_manifest_entries"],
            "execution_requires": f"--authorized-stage {STAGE} --out-dir <fresh leaf>",
            "expected_leaf": EXPECTED_LEAF,
            "attempts_authorised": 1,
            "production_command_forbids": list(PRODUCTION_COMMAND_FORBIDS),
        },
        "actor": actor,
        "lineage_preserved": lineage,
        "r1_attempt_preserved": r1,
        "warning_policy": warnings_policy,
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config, unchanged",
                    "v3_keys": len(J1.V3_ENV_TO_CONFIG)},
        "actor_contract": {"width": ACTOR_WIDTH, "full_observation_width": FULL_OBS_WIDTH,
                           "hard_zero_columns": EXPECTED_ZERO_COLUMNS,
                           "controller_columns_live": list(CONTROLLER_COLUMNS),
                           "input_layers_verified": columns,
                           "actor_feature_names": list(names),
                           "widening": "NONE - the actor stays 35D",
                           "standalone_25d": "NONE",
                           "contralateral_features": "ABSENT",
                           "note": "J2/J3 hard-zeroed the clock AND the controller memory; the "
                                   "recovery stage lifted the controller columns, so at J5 only "
                                   "the clock stays at zero"},
        "determinism": dict(J3.DETERMINISTIC_SEMANTICS),
        "gate": {"common": dict(J5_COMMON_GATE),
                 "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                       for k, v in J5_KINEMATIC_GATE.items()},
                 "source": "inherited VERBATIM from v26c_j3_closed_loop; no threshold added, "
                           "moved, relaxed or reinterpreted"},
        "telemetry_integrity_invariant": dict(TELEMETRY_INTEGRITY_INVARIANT),
        "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
        "comparisons_required": {"against": ["J3 base rollout", "J1 prescribed teacher"],
                                 "quantities": ["max_penetration_m", "steps_above_0.020_m"],
                                 "status": "EVIDENCE ONLY, never a gate"},
        "no_clobber": {"scope": "PER LEAF", "root": _rel(OUT_ROOT), "existing_leaves": leaves},
        "outcome_policy": {"pass_condition": "every J3 gate passes AND telemetry integrity holds",
                           "deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False,
                           "on_fail": "artefacts preserved and quarantined; retry FORBIDDEN"},
        "forbidden_here": ["retry", "fit", "optimizer", "DAgger", "progressive iteration", "sigma",
                           "critic", "PPO", "ex-novo", "promotion", "LOTO", "LOCO", "B1R1",
                           "B1R2", "standalone 25D", "widening", "contralateral features"],
        "generated_at_utc": _utc(),
    }


# ================================================================ the rollout ====================

# Nothing this stage writes may touch these. OUT_ROOT is listed too: the leaf lives INSIDE it,
# never IS it.
def _protected_trees() -> dict[str, Path]:
    return {
        "REPO": REPO, "HERE": HERE, "J5_OUT_ROOT": OUT_ROOT,
        "J1_RUN_ROOT": J1.OUT_ROOT, "J1_LEAF": J1_LEAF,
        "J2_RUN_ROOT": HERE / "j2_runs", "J3_RUN_ROOT": J3.OUT_ROOT, "J3_LEAF": J3_LEAF,
        "J4_RUN_ROOT": HERE / "j4_runs", "J4_LEAF": J4_LEAF, "J4_MODULE": J4_MODULE_DIR,
    }
def validate_out_dir(out: Path, *, injected: bool) -> dict[str, Any]:
    """Where may this stage write? Checked BEFORE the preflight and BEFORE anything is created.

    There are exactly two legal answers.

    PRODUCTION (injected=False): the single path OUT_ROOT/EXPECTED_LEAF, and nothing else. This is
    the ONE intentional exception to the disjointness rule - the leaf necessarily lives inside the
    repository - so it is verified explicitly and by equality, never by a containment test.

    INJECTED (a test double): a temporary leaf COMPLETELY DISJOINT from every protected tree. Not
    equal to one, not inside one, not containing one - REPO, HERE and the J5 run root included. A
    test may therefore never create, and never later delete, a directory anywhere in the
    repository.
    """
    out = Path(out)
    if not out.is_absolute():
        out = (Path.cwd() / out)
    resolved = out.resolve()
    protected = {name: p.resolve() for name, p in _protected_trees().items()}
    production_leaf = (OUT_ROOT / EXPECTED_LEAF).resolve()

    if not injected:
        # the intentional exception, verified explicitly
        if resolved != production_leaf:
            raise J5Error(f"the production leaf must be exactly {production_leaf}; got {resolved}")
        if resolved.parent != OUT_ROOT.resolve() or resolved.name != EXPECTED_LEAF:
            raise J5Error(f"the production leaf must be {OUT_ROOT.name}/{EXPECTED_LEAF}")
        return {"path": str(resolved), "injected_stack": False,
                "production_leaf_enforced": True,
                "intentional_exception": "the production leaf is the ONLY path allowed inside the "
                                         "repository, and it is matched by equality",
                "protected_trees": {k: str(v) for k, v in protected.items()}}

    for name, p in protected.items():
        if resolved == p:
            raise J5Error(f"refusing {out}: it IS the protected tree {name}")
        if p in resolved.parents:
            raise J5Error(f"refusing {out}: an injected stack may only write to a leaf DISJOINT "
                          f"from every protected tree, and this one is inside {name} ({p})")
        if resolved in p.parents:
            raise J5Error(f"refusing {out}: it CONTAINS the protected tree {name} ({p})")
    return {"path": str(resolved), "injected_stack": True,
            "production_leaf_enforced": False,
            "disjoint_from_all_protected_trees": True,
            "protected_trees": {k: str(v) for k, v in protected.items()}}


def _remove_leaf(leaf: Path, *, injected: bool) -> None:
    """Delete ONLY the leaf this call created, under the SAME isolation rules that allowed it.

    The caller passes the real `injected` flag, so a production cleanup can remove exactly the
    production leaf and an injected cleanup can remove only a disjoint temporary leaf. Neither can
    ever remove a path inside the repository that it was not entitled to create.
    """
    leaf = Path(leaf)
    # validated FIRST, so a protected path is refused loudly even when it does not exist
    validate_out_dir(leaf, injected=injected)
    if not leaf.is_dir():
        return
    shutil.rmtree(leaf, ignore_errors=True)


def run(*, authorized_stage: str | None, out_dir: Path | None, stack: Any = None,
        progress: bool = True) -> dict[str, Any]:
    """ONE fresh deterministic closed-loop rollout of the pinned J4 recovery actor."""
    if authorized_stage != STAGE:
        raise J5Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    if out_dir is None:
        raise J5Error("--out-dir is required: the J5 leaf must be named explicitly, never guessed")
    injected = stack is not None
    # WHERE may this write? Decided before the preflight runs and before anything is created.
    isolation = validate_out_dir(Path(out_dir), injected=injected)
    pre = preflight()
    if pre["blockers"]:
        raise J5Error(f"preflight BLOCKED: {pre['blockers']}")
    references = historical_references(pre["lineage_preserved"])

    out = Path(isolation["path"])
    if out.exists():
        raise J5Error(f"no-clobber: the leaf {out} already exists; choose a fresh --out-dir")

    cfg = J1.load_pinned_config()
    env_config = J1.build_full_env_config(cfg, output_dir=out)
    J1.verify_env_config(env_config, cfg)
    expected_features = actor_feature_manifest()

    if not injected and verify_warning_policy()["error_filter_active"]:
        # refused BEFORE the production stack is imported, before any env is constructed and
        # before the r2 leaf is created
        raise J5Error(
            "refusing to start: python is running with an -W error filter. The production "
            "simulator emits a standard SLSQP clipping RuntimeWarning, non-fatal under Python's "
            "default policy; promoting it to an exception aborted the r1 attempt inside its first "
            "step. Run the authorised command without -W error.")
    stack = stack if stack is not None else J3.production_stack()
    if not injected:
        # production must be the real stack, and must be able to prove parity every step
        if not getattr(stack, "operational", False):
            raise J5Error("the production stack reports itself non-operational")
        if getattr(stack, "reference_action", None) is None:
            raise J5Error("the production stack exposes no rollout_eval reference action; parity "
                          "cannot be proven and the rollout is refused")

    out.mkdir(parents=True, exist_ok=False)
    try:
        stack.seed(ROLLOUT_SEED)
        module = stack.load_module(J4_MODULE_DIR)
        env = stack.make_env(env_config)
    except BaseException:
        _remove_leaf(out, injected=injected)
        raise
    env_closed = False
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=ROLLOUT_SEED)
        contract = J3._verify_runtime_contract(base, module, obs, expected_features, cfg)
        feature_names = tuple(contract["actor_feature_names"])
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        trace: list[dict[str, Any]] = []
        knee: list[float] = []
        ankle: list[float] = []
        penetrations: list[float] = []
        clipped = 0
        parity_steps = 0
        max_abs_noise = 0.0

        for step in range(1, EXPECTED_STEPS + 1):
            obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
            if obs_vec.size != FULL_OBS_WIDTH:
                raise J5Error(f"step {step}: the observation is {obs_vec.size}D, expected "
                              f"{FULL_OBS_WIDTH}")
            actor_obs = obs_vec[:ACTOR_WIDTH]
            raw, mean, std, path = J3.deterministic_action(module, obs_vec, action_shape,
                                                           torch_mod=stack.torch)
            noise = np.asarray(raw, dtype=np.float64) - np.asarray(mean, dtype=np.float64)
            max_abs_noise = max(max_abs_noise, float(np.max(np.abs(noise))))
            if max_abs_noise != 0.0:
                raise J5Error(f"step {step}: the deterministic action differs from the policy mean "
                              f"by {max_abs_noise}; noise is forbidden at this stage")
            if stack.reference_action is not None:
                r_a, r_m, r_s, _ = stack.reference_action(module, obs_vec, action_shape)
                if not (np.array_equal(raw, r_a) and np.array_equal(mean, r_m)
                        and np.array_equal(std, r_s)):
                    raise J5Error(f"step {step}: the deterministic action does not match "
                                  "rollout_eval's own deterministic helper bit for bit")
                parity_steps += 1
            applied = np.clip(raw, low.reshape(raw.shape), high.reshape(raw.shape)
                              ).astype(np.float32)
            was_clipped = bool(np.any(applied != raw))
            clipped += int(was_clipped)
            pros = J1._prosthetic_state(actor_obs, feature_names)
            knee.append(pros["pros_knee_angle"])
            ankle.append(pros["pros_ankle_angle"])
            t_before = J1._finite(base.t, f"step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(raw)

            if "time" not in info:
                raise J5Error(f"step {step}: info exposes no 'time'; refusing to record a row with "
                              "an unknown timestamp")
            row: dict[str, Any] = {
                "step": step, "time_before": t_before,
                "time_after": J1._finite(info["time"], f"step {step}: info.time"),
                "reward": J1._finite(reward, f"step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "raw_action": np.asarray(raw, dtype=float).reshape(-1).tolist(),
                "policy_mean": np.asarray(mean, dtype=float).reshape(-1).tolist(),
                "policy_std_diagnostic": np.asarray(std, dtype=float).reshape(-1).tolist(),
                "applied_action_diagnostic": np.asarray(applied, dtype=float).reshape(-1).tolist(),
                "action_clipped_diagnostic": was_clipped,
                "action_selection_path": path,
                "stepped_with": "raw_action",
                "reward_terms": J1._jsonable(info.get("reward_terms", {}), "reward_terms"),
                J1.FSM_KEY: J1._jsonable(info.get(J1.FSM_KEY), J1.FSM_KEY),
                "prosthetic_state": pros,
            }
            for extra in ("observation", "morphology_causal_diagnostics",
                          "morphology_ledger_diagnostics", "online_grf", "online_grf_detector",
                          "observer_raw_sensor_journal"):
                if extra in info:
                    row[extra] = J1._jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: J1._jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", J1.FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            # fail closed rather than raise a bare KeyError: a missing penetration is missing
            # telemetry, and it must never be defaulted to zero
            terms = row["reward_terms"]
            if not isinstance(terms, Mapping) or J1.RT_PENETRATION not in terms:
                raise J5Error(f"step {step}: reward_terms.{J1.RT_PENETRATION} is absent; refusing "
                              "to default it, because a silent 0.0 would understate penetration")
            penetrations.append(J1._finite(terms[J1.RT_PENETRATION],
                                           f"step {step}: {J1.RT_PENETRATION}"))
            trace.append(row)
            if progress and (step % 25 == 0 or step == 1):
                print(json.dumps({"step": step, "reward": row["reward"]}), flush=True)
            if terminated or truncated:
                break

        end_reason = J1._resolve_end_reason(trace)
        knee_arr = np.asarray(knee, dtype=np.float64)
        ankle_arr = np.asarray(ankle, dtype=np.float64)
        pen_arr = np.asarray(penetrations, dtype=np.float64)
        summary = J1._summarise(trace, end_reason, clipped, [0.0, 0.0])
        if list(summary["action_noise_sigma"]) != [0.0, 0.0] or \
                list(summary["realized_noise_rms"]) != [0.0, 0.0]:
            raise J5Error("the summariser reports injected noise; J5 injects none")
        gate = evaluate_gate(summary, knee_arr, ankle_arr)
        integrity = telemetry_integrity(summary)
        verdict = overall_verdict(gate, integrity)
        if not injected and parity_steps != len(trace):
            raise J5Error(f"rollout_eval parity was proven on {parity_steps} of {len(trace)} "
                          "steps; production requires every step")
        comparison = compare_with_history(summary, pen_arr, references)

        receipt: dict[str, Any] = {
            "schema": "v26c_j5_revalidation.1", "stage": STAGE,
            "verdict": verdict,
            "verdict_kind": "FRESH DETERMINISTIC CLOSED-LOOP REVALIDATION",
            "gate_pass": bool(gate["pass"]),
            "qualification_technically_valid": bool(integrity["pass"]),
            "stack": {"name": stack.name, "operational": stack.operational, "injected": injected,
                      "note": ("an INJECTED stack is a test double: the receipt is NOT operational "
                               "evidence" if injected else "the production torch/RLlib/OpenSim "
                               "stack")},
            "authorisation": pre["authorisation"],
            "attempt": "r2",
            "r1_attempt_preserved": pre["r1_attempt_preserved"],
            "warning_policy": pre["warning_policy"],
            "output_isolation": isolation,
            "inputs_sha256": {"pinned_runtime_config": PIN_RUNTIME_CONFIG_SHA,
                              "env_config": _sha_obj(env_config),
                              "j4_artefacts": dict(PIN_J4),
                              "j5_amendment": PIN_AMENDMENT,
                              "j5_amendment_r2": PIN_AMENDMENT_R2},
            "actor": pre["actor"],
            "lineage_preserved": pre["lineage_preserved"],
            "runtime_contract": contract,
            "runtime_identity": {"fsm_behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION,
                                 "event_source": J1.EXPECTED_EVENT_SOURCE,
                                 "observed_versions": summary["fsm_behaviour_versions"],
                                 "observed_event_sources": summary["event_sources"]},
            "deterministic_semantics": {**J3.DETERMINISTIC_SEMANTICS,
                                        "max_abs_action_minus_mean": max_abs_noise,
                                        "rollout_eval_parity_steps": parity_steps,
                                        "rollout_eval_parity_checked": parity_steps > 0},
            "actor_contract": pre["actor_contract"],
            "summary": summary,
            "kinematics": {"steps": int(knee_arr.size),
                           "knee_min_rad": float(knee_arr.min()),
                           "knee_max_rad": float(knee_arr.max()),
                           "ankle_min_rad": float(ankle_arr.min()),
                           "ankle_max_rad": float(ankle_arr.max()),
                           "source": "the per-step PRE-ACTION prosthetic_state, read by name from "
                                     "the pinned actor feature manifest"},
            "j5_gate": {"common": dict(J5_COMMON_GATE),
                        "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                              for k, v in J5_KINEMATIC_GATE.items()},
                        "source": "inherited VERBATIM from v26c_j3_closed_loop"},
            "gate": gate,
            "telemetry_integrity": integrity,
            "comparison": comparison,
            "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
            "deployable": False, "promotion": "NONE", "next_stage_authorized": False,
            "automatic_promotion": "NONE, under any outcome",
            "quarantine": {"applies": verdict != "PASS", "artefacts_preserved": True,
                           "retry": "FORBIDDEN without an explicit new authorisation; this stage "
                                    "never retries itself"},
            "forbidden_here": list(pre["forbidden_here"]),
            "generated_at_utc": _utc(),
        }
        (out / "j5_trace.json").write_text(
            json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
        np.savez_compressed(out / "j5_kinematics.npz", knee_rad=knee_arr, ankle_rad=ankle_arr,
                            penetration_m=pen_arr,
                            actor_feature_names=np.asarray(feature_names, dtype=str))
        # the simulator flushes its .sto/.csv on close, so close BEFORE taking the inventory
        try:
            env.close()
        except Exception as exc:                       # noqa: BLE001
            raise J5Error(f"the environment could not be closed cleanly: {exc}") from exc
        env_closed = True
        sim_outputs = sorted(q for q in (out / "sim_outputs").rglob("*") if q.is_file()) \
            if (out / "sim_outputs").is_dir() else []
        if not injected and not sim_outputs:
            raise J5Error("the production rollout produced no sim_outputs; the simulator evidence "
                          "is missing and the run cannot be recorded")
        # RECURSIVE, POSIX-relative, everything the leaf holds except the receipt being written
        receipt["outputs_sha256"] = {
            q.relative_to(out).as_posix(): _sha_file(q)
            for q in sorted(out.rglob("*")) if q.is_file() and q.name != RECEIPT_NAME}
        receipt["outputs_sha256_excludes"] = {
            "file": RECEIPT_NAME,
            "why": "a receipt cannot hash itself; its own digest is taken by the reviewer"}
        receipt["sim_outputs"] = {"count": len(sim_outputs),
                                  "files": [q.relative_to(out).as_posix() for q in sim_outputs],
                                  "required": not injected}
        (out / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
        if progress:
            print(json.dumps({"verdict": verdict, "steps": summary["steps"],
                              "end_reason": end_reason, "failed": gate["failed"],
                              "max_penetration_m": summary["max_penetration_m"]}, indent=2))
        return receipt
    finally:
        if not env_closed:
            try:
                env.close()
            except Exception:
                pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J5 fresh closed-loop revalidation")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    r = run(authorized_stage=a.authorized_stage,
            out_dir=(Path(a.out_dir) if a.out_dir else None),
            progress=not a.no_progress)
    return 0 if r["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
