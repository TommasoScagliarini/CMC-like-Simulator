"""V26C J6 - recovery probe: three stochastic traces from the J2 base student, plus a prefix audit.

WHY
    The J4 recovery corpus is 12 aligned rows against July's 356, and a second iteration adds
    nothing: the J5-r2 trace truncates at the same step 13 on the same column. The cause is the
    alignment reference. J4 compared the J1 TEACHER trace with a STUDENT trace; July compared a
    STUDENT nominal trace with STUDENT perturbed traces. Measured here: J3 vs J5-r2 retains 500/500
    with no discrete mismatch, while J1 vs either student retains 12.

WHAT IT DOES
    Rolls out the PINNED J2 base student three times from the nominal start, with held stochastic
    exploration at sigma 0.005 (seeds 123 / 124 / 125), then audits OFFLINE the aligned prefix of
    each trace against the J3 deterministic base-student trace.

WHAT IT DOES NOT DO
    It builds no dataset. It fits nothing, adapts nothing, trains nothing. No critic, no PPO, no
    ex-novo, no promotion. It never uses J4 or J5 as a parent, as weights, as labels or as data.
    It modifies no J0-J5 artefact, no receipt, no trace, no module, not the penetration contract
    and nothing in production.

SIGMA NEEDS NO MODULE COPY
    The pinned J2 module already carries sigma = 0.005 in its frozen log-std head: pi.1.weight[2:]
    is exactly zero and pi.1.bias[2:] = -5.2983174324035645, so exp(bias) = 0.005 to 3.3e-10. The
    parent is therefore used bit-for-bit, and the "mean actor bit-exact / non-log-std tensors
    bit-exact / save-reload exact" obligations hold by identity. If that ever stops being true this
    stage REFUSES rather than editing the head.

PENETRATION
    Evaluated only through v26c_penetration_contract: > 0.020 soft diagnostic, >= 0.025 July legacy
    diagnostic, <= 0.028 the sole binding pass. No local re-implementation, no invented threshold.
    There is NO gate on prefix length; July's 119/118/119 are descriptive reference only.

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
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
TG = REPO / "Trajectory Generator"
BASELINE = TG / "baseline_MLP"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1  # noqa: E402  hardened env builder, trace parser, summariser
import v26c_j3_closed_loop as J3  # noqa: E402  stack, runtime contract, deterministic helpers
import v26c_penetration_contract as PC  # noqa: E402  the ONLY penetration authority


class J6Error(RuntimeError):
    pass


STAGE = "V26C-J6-RECOVERY-PROBE"
RECEIPT_NAME = "v26c_j6_recovery_probe_receipt.json"
AUDIT_NAME = "v26c_j6_prefix_audit.json"
OUT_ROOT = HERE / "j6_runs"
LEAF_TEMPLATE = "j6_recovery_probe_v26c_2026-08-26_seed{seed}"

# rev2 is the OPERATIVE amendment. rev1 declared itself immutable, so it is preserved untouched
# as a forensic draft and pinned here only to prove it was not edited.
AMENDMENT = HERE / "v26c_j6_amendment_recovery_probe_rev2.json"
PIN_AMENDMENT = "cea7b9ac310eebeab3b528ec7bc6f142b59564fc76427686f51627b5c1968d19"
AMENDMENT_REV1 = HERE / "v26c_j6_amendment_recovery_probe.json"
PIN_AMENDMENT_REV1 = "e063946ac323ace9120a1de6214e69f57b1cdf336bf52faaabb0930d0a1a3fa6"

# ------------------------------------------------------------------ the operational parent ------
PARENT_LEAF = HERE / "j2_runs" / "j2_base_v26c_2026-08-26_r1"
PARENT_MODULE_DIR = PARENT_LEAF / "rl_module"
PIN_PARENT = dict(J3.PIN_J2)          # the six J2 artefacts, by their own pins
# EVIDENCE ONLY - never a parent, never weights, never labels, never data.
J4_MODULE_STATE_SHA = "14a3630f757a5da2055eb754f6249fad8e7989a6d5e6c18f526c76415dad31aa"

# ------------------------------------------------------------------ alignment reference ---------
NOMINAL_LEAF = J3.OUT_ROOT / "j3_base_v26c_2026-08-26_r1"
NOMINAL_TRACE = NOMINAL_LEAF / "j3_trace.json"
PIN_NOMINAL_TRACE = "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae"
PIN_NOMINAL_RECEIPT = "34d856b0b4acabd000a1e6257767c6049a1f9f2147eb99d6f3801ca7559ff422"
# The teacher trace is explicitly NOT the alignment reference. It is pinned only because a LATER
# stage will read its actions as recovery labels.
TEACHER_LEAF = J1.OUT_ROOT / "j1_nominal_v26c_2026-08-26_r1"
PIN_TEACHER_DATASET = "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41"
PIN_TEACHER_TRACE = "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3"

# ------------------------------------------------------------------ probe semantics -------------
SIGMA = 0.005
SIGMA_STATUS = "HYPOTHESIS UNDER TEST, NOT A SELECTED VALUE"
SIGMA_TOLERANCE = 1e-6          # |exp(logstd_bias) - SIGMA|; the parent measures 3.3e-10
NOISE_HOLD_STEPS = 1
SEEDS = (123, 124, 125)
ACTOR_WIDTH = J3.ACTOR_WIDTH
FULL_OBS_WIDTH = J3.FULL_OBS_WIDTH
EXPECTED_STEPS = J3.EXPECTED_STEPS
MASKED_COLUMNS = J3.MASKED_COLUMNS          # base student: clock AND controller memory still zero
PIN_RUNTIME_CONFIG_SHA = J3.PIN_RUNTIME_CONFIG_SHA

# July's retained prefixes. DESCRIPTIVE REFERENCE ONLY - never a threshold, never a gate.
JULY_RETAINED_PREFIXES_DESCRIPTIVE = (119, 118, 119)

FORBIDDEN_HERE = ("J4 or J5 as parent/weights/labels/dataset", "log-std head editing",
                  "module derivation", "fit", "optimizer step", "adaptation", "training",
                  "critic warm-up", "PPO", "ex-novo", "promotion", "standalone 25D", "widening",
                  "contralateral features", "LOTO", "LOCO", "B1R1", "B1R2",
                  "July artefacts as operational parent/dataset/label",
                  "a numeric gate on prefix length")


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


def leaf_for(seed: int) -> Path:
    return OUT_ROOT / LEAF_TEMPLATE.format(seed=int(seed))


# ================================================================ the binding gate ===============
# EXACTLY the J1/J3 common criteria, with ONE substitution: the penetration criterion is the
# contract's single hard binding band, not the 0.020 soft diagnostic. Every other criterion is
# inherited verbatim and is just as binding here as it is at J3.
_J1_COMMON = {k: v for k, v in J1.J1_GATE.items() if k != "max_penetration_m_max"}
J6_GATE: dict[str, Any] = {**_J1_COMMON, "max_penetration_m_max_binding": 0.028}
J6_GATE_SOURCE = ("v26c_j1_collect.J1_GATE for every non-penetration criterion; the penetration "
                  "criterion is the penetration contract's sole hard binding band")


def evaluate_gate(summary: Mapping[str, Any], penetration: Mapping[str, Any]) -> dict[str, Any]:
    """A collection is only usable if the EPISODE is intact, not merely if it stayed shallow."""
    checks = {
        "steps": summary["steps"] == J6_GATE["steps_required"],
        "end_reason": summary["end_reason"] == J6_GATE["end_reason"],
        "valid_cycles": summary["valid_cycle_count"] >= J6_GATE["valid_cycles_min"],
        "phase_timeout_stance":
            summary["phase_timeout_stance"] <= J6_GATE["phase_timeout_stance_max"],
        "phase_timeout_swing":
            summary["phase_timeout_swing"] <= J6_GATE["phase_timeout_swing_max"],
        "morphology_causal_contract_failure":
            summary["morphology_causal_contract_failure"]
            <= J6_GATE["morphology_causal_contract_failure_max"],
        "hs_cancelled_count":
            summary["hs_cancelled_count"] <= J6_GATE["hs_cancelled_count_max"],
        "resync_count": summary["resync_count"] <= J6_GATE["resync_count_max"],
        "penetration_hard_binding": bool(penetration["binding_pass"]),
    }
    failed = sorted(k for k, v in checks.items() if not v)
    return {
        "criteria": dict(J6_GATE), "source": J6_GATE_SOURCE,
        "measures": {
            "steps": summary["steps"], "end_reason": summary["end_reason"],
            "valid_cycle_count": summary["valid_cycle_count"],
            "valid_hs_count": summary["valid_hs_count"],
            "valid_to_count": summary["valid_to_count"],
            "phase_timeout_stance": summary["phase_timeout_stance"],
            "phase_timeout_swing": summary["phase_timeout_swing"],
            "morphology_causal_contract_failure":
                summary["morphology_causal_contract_failure"],
            "hs_cancelled_count": summary["hs_cancelled_count"],
            "resync_count": summary["resync_count"],
            "max_penetration_m": penetration["max_penetration_m"],
            "action_clipped_steps": summary["action_clipped_steps"],
        },
        "checks": checks, "failed": failed, "pass": not failed,
        "penetration_bands": {
            "soft_diagnostic_above": penetration["flags"]["above_soft_diagnostic"],
            "july_legacy_breach": penetration["flags"]["july_legacy_breach"],
            "hard_binding_pass": penetration["binding_pass"],
            "note": "> 0.020 and >= 0.025 are DIAGNOSTIC and never fail this gate; only "
                    "> 0.028 does",
        },
        "diagnostics_not_binding": {"action_clipped_steps": summary["action_clipped_steps"]},
    }


# ================================================================ inputs =========================

def verify_parent() -> dict[str, Any]:
    """The six J2 artefacts by exact hash. The J4 module is explicitly refused as a parent."""
    if not PARENT_LEAF.is_dir():
        raise J6Error(f"the J2 parent leaf is missing: {PARENT_LEAF}")
    present = sorted(str(q.relative_to(PARENT_LEAF)) for q in PARENT_LEAF.rglob("*")
                     if q.is_file())
    if present != sorted(PIN_PARENT):
        raise J6Error(f"the J2 leaf holds {present}, expected exactly {sorted(PIN_PARENT)}")
    got: dict[str, str] = {}
    for rel, pin in PIN_PARENT.items():
        h = _sha_file(PARENT_LEAF / rel)
        if h != pin:
            raise J6Error(f"the J2 artefact {rel} changed: {h} != {pin}")
        got[rel] = h
    state_sha = got["rl_module/module_state.pkl"]
    if state_sha == J4_MODULE_STATE_SHA:
        raise J6Error("the parent resolves to the J4 recovery module; J4 is evidence, never a "
                      "parent")
    return {"leaf": _rel(PARENT_LEAF), "module": _rel(PARENT_MODULE_DIR),
            "artefacts_sha256": got, "module_state_sha256": state_sha,
            "role": "the ONLY operational parent of this probe",
            "j4_module_state_sha256_refused": J4_MODULE_STATE_SHA}


def verify_sigma(state: Mapping[str, np.ndarray]) -> dict[str, Any]:
    """The parent must ALREADY carry sigma; this stage never edits a log-std head."""
    w = np.asarray(state["pi.1.weight"])
    b = np.asarray(state["pi.1.bias"])
    action_dim = w.shape[0] // 2
    logstd_w = w[action_dim:]
    logstd_b = b[action_dim:].astype(np.float64)
    if not bool(np.all(logstd_w == 0.0)):
        raise J6Error("the parent's log-std output weights are not zero, so sigma is not constant")
    sigmas = np.exp(logstd_b)
    if not bool(np.all(np.isfinite(sigmas))):
        raise J6Error("the parent's sigma is not finite")
    if float(np.max(np.abs(sigmas - SIGMA))) > SIGMA_TOLERANCE:
        raise J6Error(f"the parent carries sigma {sigmas.tolist()}, expected {SIGMA} within "
                      f"{SIGMA_TOLERANCE}. This stage REFUSES to edit the log-std head; a "
                      "different sigma needs its own authorised derivation stage.")
    return {"sigma": SIGMA, "status": SIGMA_STATUS,
            "logstd_bias": [float(v) for v in logstd_b],
            "sigma_from_parent": [float(v) for v in sigmas],
            "max_abs_deviation": float(np.max(np.abs(sigmas - SIGMA))),
            "tolerance": SIGMA_TOLERANCE,
            "module_copy_required": False,
            "logstd_head_edited": False,
            "mean_actor_bit_exact": True,
            "non_logstd_tensors_bit_exact": True,
            "save_reload_exact": True,
            "why_trivially_satisfied": "the parent module is used bit-for-bit, with no copy and no "
                                       "edit, so every bit-exactness obligation holds by identity",
            "noise_hold_steps": NOISE_HOLD_STEPS}


def verify_alignment_reference() -> dict[str, Any]:
    """The J3 deterministic base-student trace, and the teacher artefacts kept for LATER labels."""
    checks = {
        "j3_trace": (NOMINAL_TRACE, PIN_NOMINAL_TRACE),
        "j3_receipt": (NOMINAL_LEAF / "v26c_j3_closed_loop_receipt.json", PIN_NOMINAL_RECEIPT),
        "j1_teacher_dataset": (TEACHER_LEAF / "teacher_dataset.npz", PIN_TEACHER_DATASET),
        "j1_teacher_trace": (TEACHER_LEAF / "teacher_trace.json", PIN_TEACHER_TRACE),
    }
    got: dict[str, str] = {}
    for label, (path, pin) in checks.items():
        if not path.is_file():
            raise J6Error(f"the pinned {label} is missing: {path}")
        h = _sha_file(path)
        if h != pin:
            raise J6Error(f"the pinned {label} changed: {h} != {pin}")
        got[label] = h
    nominal = json.loads(NOMINAL_TRACE.read_text())
    if not isinstance(nominal, list) or len(nominal) != EXPECTED_STEPS:
        raise J6Error(f"the nominal trace holds {len(nominal)} rows, expected {EXPECTED_STEPS}")
    return {
        "nominal_trace": _rel(NOMINAL_TRACE), "nominal_rows": len(nominal),
        "pins_sha256": got,
        "why_j3_not_j1": "J3 is the deterministic base-student rollout, the analogue of July's "
                         "markov35_zero_column_deterministic nominal trace. The J1 teacher trace "
                         "is NOT the alignment reference; it is kept only for the LATER label "
                         "step.",
        "measured_justification": {
            "j1_teacher_vs_student_retained": 12,
            "j3_student_vs_student_retained": 500,
            "note": "measured read-only before this stage was written",
        },
    }


def verify_amendment() -> dict[str, Any]:
    if not AMENDMENT.is_file():
        raise J6Error("the operative J6 amendment (rev2) is missing")
    h = _sha_file(AMENDMENT)
    if PIN_AMENDMENT != "PENDING" and h != PIN_AMENDMENT:
        raise J6Error(f"the J6 rev2 amendment changed: {h} != {PIN_AMENDMENT}")
    if not AMENDMENT_REV1.is_file():
        raise J6Error("the superseded rev1 amendment is missing; it is preserved as evidence")
    h1 = _sha_file(AMENDMENT_REV1)
    if h1 != PIN_AMENDMENT_REV1:
        raise J6Error(f"the superseded rev1 amendment was edited: {h1} != {PIN_AMENDMENT_REV1}")
    superseded = json.loads(AMENDMENT.read_text()).get("supersedes", {})
    if superseded.get("sha256") != PIN_AMENDMENT_REV1:
        raise J6Error("rev2 does not supersede the rev1 that is on disk")
    manifest = json.loads(AMENDMENT.read_text())["pinned_artefacts_sha256"]
    if not manifest:
        raise J6Error("the J6 amendment pins no artefacts")
    checked: dict[str, str] = {}
    for rel, pin in manifest.items():
        target = HERE / rel
        if not target.is_file():
            raise J6Error(f"the amendment pins {rel}, which is missing")
        got = _sha_file(target)
        if got != pin:
            raise J6Error(f"the amendment-pinned artefact {rel} changed: {got} != {pin}")
        checked[rel] = got
    return {"file": _rel(AMENDMENT), "sha256": h, "revision": 2,
            "supersedes": {"file": _rel(AMENDMENT_REV1), "sha256": h1,
                           "status": "preserved untouched as a forensic draft"},
            "manifest_entries": len(checked), "manifest_sha256": checked}


def load_parent_state() -> dict[str, np.ndarray]:
    with (PARENT_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        return {k: np.asarray(v) for k, v in pickle.load(fh).items()}


def verify_masked_columns(state: Mapping[str, np.ndarray]) -> dict[str, Any]:
    """The BASE student still masks the clock AND the controller memory."""
    inputs = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and np.ndim(v) == 2
                    and np.shape(v)[1] == ACTOR_WIDTH)
    if not inputs:
        raise J6Error(f"the parent holds no {ACTOR_WIDTH}D input layer")
    report: dict[str, list[int]] = {}
    for key in inputs:
        W = np.asarray(state[key])
        zero = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if zero != list(MASKED_COLUMNS):
            raise J6Error(f"the parent layer {key} has zero columns {zero}, expected "
                          f"{list(MASKED_COLUMNS)}; this probe rolls out the BASE student")
        report[key] = zero
    return report


def build_env_config(*, output_dir: Path | None = None) -> dict[str, Any]:
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_dir)
    J1.verify_env_config(env, cfg)
    return env


# ================================================================ production stack ================

def production_stack() -> Any:
    """J3's production stack, EXTENDED with the two production helpers this stage needs.

    Heavy imports live here only, so the preflight stays inert. J3 itself is never modified.
    """
    stack = J3.production_stack()
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import rollout_eval as RE
    import target_domain_noise_adaptation as JN

    # THE production stochastic action: forward_exploration plus the policy's EFFECTIVE std.
    stack.held_stochastic_action = (
        lambda module, obs, shape, unit_noise: RE._held_stochastic_action(
            module, obs, shape, unit_noise))
    # THE production truncation rule, used to prove the local transcription at runtime.
    stack.july_truncate = JN.truncate_before_discrete_mismatch
    stack.july_discrete_indices = JN._discrete_feature_indices
    return stack


# ================================================================ July's truncation rule =========

def discrete_feature_indices(names: Sequence[str]) -> np.ndarray:
    """target_domain_noise_adaptation._discrete_feature_indices, transcribed."""
    return np.asarray(
        [i for i, n in enumerate(names)
         if str(n).endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
         or str(n).startswith(("phase_fsm_", "phase_expected_"))],
        dtype=int)


def truncate_before_discrete_mismatch(nominal_rows: Sequence[Mapping[str, Any]],
                                      recovery_rows: Sequence[Mapping[str, Any]],
                                      names: Sequence[str]) -> tuple[list[Any], dict[str, Any]]:
    """target_domain_noise_adaptation.truncate_before_discrete_mismatch, transcribed.

    No tolerance, no window, no slack: the rule is never relaxed, and there is NO minimum-length
    gate anywhere in it.
    """
    disc = discrete_feature_indices(names)
    limit = min(len(nominal_rows), len(recovery_rows))
    first_mismatch = None
    columns: list[dict[str, Any]] = []
    for i in range(limit):
        a = np.asarray(nominal_rows[i]["actor_observation_vector_before"], dtype=float)
        b = np.asarray(recovery_rows[i]["actor_observation_vector_before"], dtype=float)
        if disc.size and np.any(a[disc] != b[disc]):
            first_mismatch = i + 1
            columns = [{"index": int(j), "feature": str(names[j]),
                        "nominal": float(a[j]), "recovery": float(b[j])}
                       for j in disc if a[j] != b[j]]
            limit = i
            break
    return list(recovery_rows[:limit]), {
        "original_steps": len(recovery_rows), "retained_steps": limit,
        "first_discrete_mismatch_step": first_mismatch, "mismatching_columns": columns,
        "rule": "July-strict; never relaxed; no minimum-length gate",
    }


def actor_feature_names() -> tuple[str, ...]:
    manifest = json.loads((PARENT_MODULE_DIR / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J6Error(f"the pinned manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


# ================================================================ offline prefix audit ===========

def audit_prefixes(leaves: Sequence[Path] | None = None) -> dict[str, Any]:
    """Truncate every collected trace against the J3 nominal trace. Offline: no environment."""
    amendment = verify_amendment()
    alignment = verify_alignment_reference()
    names = actor_feature_names()
    contract = PC.load_contract()
    nominal = json.loads(NOMINAL_TRACE.read_text())
    found = list(leaves) if leaves is not None else [leaf_for(s) for s in SEEDS
                                                     if leaf_for(s).is_dir()]
    traces: dict[str, Any] = {}
    for leaf in found:
        leaf = Path(leaf)
        trace_path = leaf / "j6_trace.json"
        if not trace_path.is_file():
            raise J6Error(f"{leaf.name}: j6_trace.json is missing")
        rows = json.loads(trace_path.read_text())
        kept, report = truncate_before_discrete_mismatch(nominal, rows, names)
        series = []
        for index, row in enumerate(rows, start=1):
            terms = row.get("reward_terms")
            if not isinstance(terms, Mapping) or "grf_penetration_m" not in terms:
                raise J6Error(f"{leaf.name}: row {index} has no reward_terms.grf_penetration_m")
            series.append(float(terms["grf_penetration_m"]))
        traces[leaf.name] = {
            "leaf": _rel(leaf), "trace_sha256": _sha_file(trace_path),
            "rows": len(rows), "prefix": report,
            "penetration": PC.evaluate_series(series, contract, label=f"{leaf.name} penetration"),
        }
    return {
        "schema": "v26c_j6_prefix_audit.1", "stage": STAGE,
        "kind": "OFFLINE PREFIX AUDIT - no environment, no dataset, no fit",
        "amendment": amendment, "alignment_reference": alignment,
        "nominal_trace_sha256": PIN_NOMINAL_TRACE,
        "contract": {"file": contract["path"], "sha256": contract["sha256"]},
        "traces": traces,
        "total_retained_steps": sum(t["prefix"]["retained_steps"] for t in traces.values()),
        "july_retained_prefixes_descriptive_only": list(JULY_RETAINED_PREFIXES_DESCRIPTIVE),
        "no_prefix_length_gate": "the retained lengths are REPORTED. No threshold is applied to "
                                 "them here and none is implied by July's numbers.",
        "builds_no_dataset": True, "fits_nothing": True,
        "deployable": False, "promotion": "NONE", "next_stage_authorized": False,
        "contract_effective_date_utc": contract["data"].get("date_utc"),
    }


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: constructs, resets and steps nothing."""
    amendment = verify_amendment()
    parent = verify_parent()
    alignment = verify_alignment_reference()
    state = load_parent_state()
    sigma = verify_sigma(state)
    masked = verify_masked_columns(state)
    contract = PC.load_contract()
    env = build_env_config()
    names = actor_feature_names()
    blockers: list[str] = []
    existing = sorted(q.name for q in OUT_ROOT.iterdir()) if OUT_ROOT.is_dir() else []
    planned = {int(s): {"leaf": _rel(leaf_for(s)), "exists": leaf_for(s).exists()} for s in SEEDS}
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "dataset_built": False, "fit_executed": False,
                  "note": "the env factory is imported inside collect() only"},
        "authorisation": {
            "authorised_by": "THE USER, explicitly. The architect is gate owner and reviewer.",
            "amendment": amendment["file"], "amendment_sha256": amendment["sha256"],
            "amendment_manifest_entries": amendment["manifest_entries"],
            "scope": "three stochastic recovery traces plus an offline prefix audit",
            "execution_requires": f"--authorized-stage {STAGE} --seed <123|124|125>",
            "planned_leaves": planned,
        },
        "parent": parent,
        "sigma": sigma,
        "alignment_reference": alignment,
        "actor_contract": {"width": ACTOR_WIDTH, "full_observation_width": FULL_OBS_WIDTH,
                           "hard_zero_columns": list(MASKED_COLUMNS),
                           "input_layers_verified": masked,
                           "actor_feature_names": list(names),
                           "widening": "NONE", "standalone_25d": "NONE",
                           "contralateral_features": "ABSENT"},
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config, unchanged",
                    "v3_keys": len(J1.V3_ENV_TO_CONFIG),
                    "steps": EXPECTED_STEPS, "start": "nominal"},
        "penetration_contract": {"file": contract["path"], "sha256": contract["sha256"],
                                 "thresholds_m": {"soft_diagnostic": contract["soft_m"],
                                                  "july_legacy": contract["july_legacy_m"],
                                                  "hard_binding": contract["hard_m"]},
                                 "evaluated_by": "v26c_penetration_contract.evaluate_series"},
        "truncation": {"rule": "first discrete mismatch against the J3 nominal trace",
                       "source": "target_domain_noise_adaptation.truncate_before_discrete_mismatch",
                       "july_retained_prefixes_descriptive_only":
                           list(JULY_RETAINED_PREFIXES_DESCRIPTIVE),
                       "no_prefix_length_gate": True},
        "future_dataset_specification_only": {
            "nominal_observations": "the J3 trace observations",
            "nominal_labels": "self-distilled from the J2 source actor",
            "recovery_states": "the aligned prefixes of the three J6 traces",
            "recovery_labels": "time-aligned J1 teacher actions",
            "multistart_block": "OMITTED / DEFERRED",
            "built_here": False,
        },
        "no_clobber": {"scope": "PER LEAF", "root": _rel(OUT_ROOT), "existing_leaves": existing},
        "outcome_policy": {"deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False},
        "forbidden_here": list(FORBIDDEN_HERE),
        "generated_at_utc": _utc(),
    }


# ================================================================ the collection =================

def _remove_leaf(leaf: Path) -> None:
    leaf = Path(leaf)
    if not leaf.is_dir():
        return
    r = leaf.resolve()
    protected = {OUT_ROOT.resolve(), HERE.resolve(), REPO.resolve(), PARENT_LEAF.resolve(),
                 PARENT_MODULE_DIR.resolve(), NOMINAL_LEAF.resolve(), TEACHER_LEAF.resolve(),
                 J1.OUT_ROOT.resolve(), J3.OUT_ROOT.resolve()}
    if r in protected or r in PARENT_LEAF.resolve().parents \
            or r in NOMINAL_LEAF.resolve().parents:
        raise J6Error(f"refusing to remove {leaf}: it is a root or a pinned artefact")
    shutil.rmtree(leaf, ignore_errors=True)


def collect(*, authorized_stage: str | None, seed: int, out_dir: Path | None = None,
            stack: Any = None, progress: bool = True) -> dict[str, Any]:
    """ONE stochastic recovery trace from the pinned J2 base student. Requires the exact token."""
    if authorized_stage != STAGE:
        raise J6Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    if int(seed) not in SEEDS:
        raise J6Error(f"seed {seed!r} is not one of the authorised seeds {list(SEEDS)}")
    pre = preflight()
    if pre["blockers"]:
        raise J6Error(f"preflight BLOCKED: {pre['blockers']}")

    injected = stack is not None
    out = Path(out_dir) if out_dir is not None else leaf_for(seed)
    if not injected and out.resolve() != leaf_for(seed).resolve():
        raise J6Error(f"the production leaf for seed {seed} must be exactly {leaf_for(seed)}")
    if out.exists():
        raise J6Error(f"no-clobber: the leaf {out} already exists")

    cfg = J1.load_pinned_config()
    env_config = J1.build_full_env_config(cfg, output_dir=out)
    J1.verify_env_config(env_config, cfg)
    names = actor_feature_names()
    contract = PC.load_contract()

    stack = stack if stack is not None else production_stack()
    if getattr(stack, "held_stochastic_action", None) is None:
        raise J6Error("the stack exposes no held_stochastic_action helper; this stage uses the "
                      "PRODUCTION rollout_eval semantics and refuses to improvise its own")
    if not injected:
        if not getattr(stack, "operational", False) \
                or getattr(stack, "reference_action", None) is None:
            raise J6Error("the production stack is not operational or exposes no reference helper")
        if getattr(stack, "july_truncate", None) is None:
            raise J6Error("the production stack exposes no July truncation function; the local "
                          "transcription cannot be proven at runtime")

    out.mkdir(parents=True, exist_ok=False)
    try:
        stack.seed(int(seed))
        module = stack.load_module(PARENT_MODULE_DIR)
        env = stack.make_env(env_config)
    except BaseException:
        _remove_leaf(out)
        raise
    env_closed = False
    try:
        base = env.unwrapped
        obs, _info = env.reset(seed=int(seed))
        contract_report = J3._verify_runtime_contract(base, module, obs, names, cfg)
        feature_names = tuple(contract_report["actor_feature_names"])
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        if str(BASELINE) not in sys.path:
            sys.path.insert(0, str(BASELINE))
        import exploration_noise
        noise = exploration_noise.HeldStandardNormal(
            np.random.default_rng(int(seed)), env.action_space.shape, NOISE_HOLD_STEPS)
        # declared for the receipt only: the ACTION uses the policy's own std, never this vector
        sigma_vec = exploration_noise.broadcast_sigma(
            [SIGMA] * int(np.prod(env.action_space.shape)), int(np.prod(env.action_space.shape))
        ).reshape(env.action_space.shape)

        trace: list[dict[str, Any]] = []
        series: list[float] = []
        clipped = 0
        parity_steps = 0
        noise_rows: list[np.ndarray] = []
        for step in range(1, EXPECTED_STEPS + 1):
            obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
            if obs_vec.size != FULL_OBS_WIDTH:
                raise J6Error(f"step {step}: the observation is {obs_vec.size}D, expected "
                              f"{FULL_OBS_WIDTH}")
            actor_obs = obs_vec[:ACTOR_WIDTH]
            unit = np.asarray(noise.next(), dtype=np.float32).reshape(action_shape)
            # THE PRODUCTION PATH: rollout_eval._held_stochastic_action -> forward_exploration,
            # with the policy's own EFFECTIVE std. The action, the mean, the std and the applied
            # noise are all the ones production returns; nothing is recomputed here.
            raw, policy_mean, std, applied_noise = stack.held_stochastic_action(
                module, obs_vec, action_shape, unit)
            raw = np.asarray(raw, dtype=np.float32)
            policy_mean = np.asarray(policy_mean, dtype=np.float32)
            std = np.asarray(std, dtype=np.float32)
            applied_noise = np.asarray(applied_noise, dtype=np.float32)
            # PARITY: the exploration mean must equal the inference mean. If forward_inference and
            # forward_exploration ever diverged, the stochastic path would be sampling around a
            # different policy than the one J3 rolled out, and this run would be meaningless.
            det_mean, det_policy_mean, det_std, _ = J3.deterministic_action(
                module, obs_vec, action_shape, torch_mod=stack.torch)
            if not np.array_equal(np.asarray(det_policy_mean, dtype=np.float32), policy_mean):
                raise J6Error(
                    f"step {step}: forward_exploration and forward_inference disagree on the "
                    f"policy mean ({policy_mean.tolist()} vs "
                    f"{np.asarray(det_policy_mean).tolist()}). The stochastic probe would not be "
                    "exploring around the J3 policy.")
            if not np.array_equal(np.asarray(det_std, dtype=np.float32), std):
                raise J6Error(f"step {step}: the two forward paths disagree on the std "
                              f"({std.tolist()} vs {np.asarray(det_std).tolist()})")
            if float(np.max(np.abs(std.astype(np.float64) - SIGMA))) > SIGMA_TOLERANCE:
                raise J6Error(f"step {step}: the policy's effective std is {std.tolist()}, "
                              f"expected {SIGMA} within {SIGMA_TOLERANCE}")
            # the production helper returns applied_noise = std * unit; check the identity it
            # documents rather than assuming it
            expected_noise = (std * unit).astype(np.float32)
            if not np.array_equal(applied_noise, expected_noise):
                raise J6Error(f"step {step}: the applied noise is not std * unit_noise")
            parity_steps += 1
            noise_rows.append(np.asarray(applied_noise, dtype=np.float64).reshape(-1).copy())
            applied = np.clip(raw, low.reshape(raw.shape), high.reshape(raw.shape)
                              ).astype(np.float32)
            was_clipped = bool(np.any(applied != raw))
            clipped += int(was_clipped)
            pros = J1._prosthetic_state(actor_obs, feature_names)
            t_before = J1._finite(base.t, f"step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(raw)

            if "time" not in info:
                raise J6Error(f"step {step}: info exposes no 'time'")
            row: dict[str, Any] = {
                "step": step, "time_before": t_before,
                "time_after": J1._finite(info["time"], f"step {step}: info.time"),
                "reward": J1._finite(reward, f"step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "raw_action": np.asarray(raw, dtype=float).reshape(-1).tolist(),
                "policy_mean": np.asarray(policy_mean, dtype=float).reshape(-1).tolist(),
                "policy_std_diagnostic": np.asarray(std, dtype=float).reshape(-1).tolist(),
                "action_noise": np.asarray(applied_noise, dtype=float).reshape(-1).tolist(),
                "applied_action_diagnostic": np.asarray(applied, dtype=float).reshape(-1).tolist(),
                "action_clipped_diagnostic": was_clipped,
                "action_selection_path": f"stochastic_held(sigma={SIGMA}, hold="
                                         f"{NOISE_HOLD_STEPS})",
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
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")}
            terms = row["reward_terms"]
            if not isinstance(terms, Mapping) or J1.RT_PENETRATION not in terms:
                raise J6Error(f"step {step}: reward_terms.{J1.RT_PENETRATION} is absent")
            series.append(J1._finite(terms[J1.RT_PENETRATION], f"step {step}: penetration"))
            trace.append(row)
            if progress and (step % 25 == 0 or step == 1):
                print(json.dumps({"seed": int(seed), "step": step}), flush=True)
            if terminated or truncated:
                break

        end_reason = J1._resolve_end_reason(trace)
        noise_arr = np.asarray(noise_rows, dtype=np.float64)
        realized_rms = [float(v) for v in np.sqrt(np.mean(noise_arr ** 2, axis=0))]
        summary = J1._summarise(trace, end_reason, clipped, realized_rms)
        penetration = PC.evaluate_series(series, contract, label=f"seed {seed} penetration")
        nominal = json.loads(NOMINAL_TRACE.read_text())
        kept, prefix = truncate_before_discrete_mismatch(nominal, trace, names)
        # RUNTIME PARITY: the local transcription must agree with the July production function.
        # It is only importable on the heavy path, which is why the preflight keeps the wrapper.
        july_truncate = getattr(stack, "july_truncate", None)
        if july_truncate is not None:
            july_kept, july_report = july_truncate(nominal, trace, names)
            if (len(july_kept) != len(kept)
                    or july_report["retained_steps"] != prefix["retained_steps"]
                    or july_report["first_discrete_mismatch_step"]
                    != prefix["first_discrete_mismatch_step"]):
                raise J6Error(
                    "the local truncation transcription disagrees with "
                    "target_domain_noise_adaptation.truncate_before_discrete_mismatch: "
                    f"{prefix['retained_steps']} vs {july_report['retained_steps']} retained")
            prefix["july_function_parity"] = {
                "verified": True,
                "function": "target_domain_noise_adaptation.truncate_before_discrete_mismatch",
                "retained_steps": int(july_report["retained_steps"]),
            }
        else:
            prefix["july_function_parity"] = {
                "verified": False,
                "why": "the July module is not importable on this stack (injected test double)",
            }
        gate = evaluate_gate(summary, penetration)

        try:
            env.close()
        except Exception as exc:                      # noqa: BLE001
            raise J6Error(f"the environment could not be closed cleanly: {exc}") from exc
        env_closed = True
        sim_outputs = sorted(q for q in (out / "sim_outputs").rglob("*") if q.is_file()) \
            if (out / "sim_outputs").is_dir() else []
        if not injected and not sim_outputs:
            raise J6Error("the production collection produced no sim_outputs")

        receipt: dict[str, Any] = {
            "schema": "v26c_j6_recovery_probe.1", "stage": STAGE,
            "verdict": "PASS" if gate["pass"] else "FAIL",
            "verdict_kind": "STOCHASTIC RECOVERY PROBE - COLLECTION ONLY",
            "seed": int(seed),
            "stack": {"name": stack.name, "operational": stack.operational, "injected": injected},
            "authorisation": pre["authorisation"], "parent": pre["parent"], "sigma": pre["sigma"],
            "alignment_reference": pre["alignment_reference"],
            "actor_contract": pre["actor_contract"],
            "runtime_contract": contract_report,
            "runtime_identity": {"fsm_behaviour_version": J1.EXPECTED_FSM_BEHAVIOUR_VERSION,
                                 "event_source": J1.EXPECTED_EVENT_SOURCE,
                                 "observed_versions": summary["fsm_behaviour_versions"],
                                 "observed_event_sources": summary["event_sources"]},
            "inputs_sha256": {"pinned_runtime_config": PIN_RUNTIME_CONFIG_SHA,
                              "env_config": _sha_obj(env_config),
                              "parent_artefacts": dict(PIN_PARENT),
                              "nominal_trace": PIN_NOMINAL_TRACE,
                              "amendment": pre["authorisation"]["amendment_sha256"],
                              "penetration_contract": contract["sha256"]},
            "summary": summary,
            "gate": gate,
            "gate_failed": gate["failed"],
            "penetration_under_contract": penetration,
            "prefix_against_j3_nominal": prefix,
            "july_retained_prefixes_descriptive_only":
                list(JULY_RETAINED_PREFIXES_DESCRIPTIVE),
            "no_prefix_length_gate": True,
            "exploration": {"sigma": SIGMA, "status": SIGMA_STATUS,
                            "hold_steps": NOISE_HOLD_STEPS, "seed": int(seed),
                            "realized_noise_rms": realized_rms,
                            "logstd_head_edited": False, "module_copy_required": False,
                            "action_path": "rollout_eval._held_stochastic_action "
                                           "(forward_exploration + the policy's effective std)",
                            "effective_std_used": True,
                            "inference_exploration_parity_steps": parity_steps,
                            "declared_sigma_vector_unused_for_action":
                                [float(v) for v in np.asarray(sigma_vec).reshape(-1)]},
            "sim_outputs": {"count": len(sim_outputs), "required": not injected},
            "builds_no_dataset": True, "fits_nothing": True, "critic_trained": False,
            "ppo_updates": 0,
            "deployable": False, "promotion": "NONE", "next_stage_authorized": False,
            "quarantine": {"applies": not gate["pass"],
                           "artefacts_preserved": True,
                           "retry": "FORBIDDEN without an explicit new authorisation"},
            "forbidden_here": list(FORBIDDEN_HERE),
            "generated_at_utc": _utc(),
        }
        (out / "j6_trace.json").write_text(
            json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
        np.savez_compressed(out / "j6_penetration.npz",
                            penetration_m=np.asarray(series, dtype=np.float64),
                            action_noise=noise_arr,
                            actor_feature_names=np.asarray(feature_names, dtype=str))
        receipt["outputs_sha256"] = {
            q.relative_to(out).as_posix(): _sha_file(q)
            for q in sorted(out.rglob("*")) if q.is_file() and q.name != RECEIPT_NAME}
        receipt["outputs_sha256_excludes"] = {"file": RECEIPT_NAME,
                                              "why": "a receipt cannot hash itself"}
        (out / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8")
        if progress:
            print(json.dumps({"seed": int(seed), "verdict": receipt["verdict"],
                              "failed": gate["failed"],
                              "steps": summary["steps"], "end_reason": end_reason,
                              "retained_prefix": prefix["retained_steps"],
                              "max_penetration_m": penetration["max_penetration_m"]}, indent=2))
        return receipt
    finally:
        if not env_closed:
            try:
                env.close()
            except Exception:
                pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J6 recovery probe")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--audit", action="store_true",
                   help="offline prefix audit over the collected traces; runs no environment")
    p.add_argument("--out", default=None, help="explicit output path for --audit (no-clobber)")
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.audit:
        if not a.out:
            raise J6Error("--audit requires an explicit --out path")
        out = Path(a.out)
        if out.exists():
            raise J6Error(f"no-clobber: {out} already exists")
        record = audit_prefixes()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(record, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                       encoding="utf-8")
        print(json.dumps({"written": _rel(out), "sha256": _sha_file(out),
                          "traces_audited": len(record["traces"])}, indent=2))
        return 0
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    if a.seed is None:
        raise J6Error("--seed is required: each recovery trace is collected under its own seed")
    r = collect(authorized_stage=a.authorized_stage, seed=a.seed,
                out_dir=(Path(a.out_dir) if a.out_dir else None),
                progress=not a.no_progress)
    return 0 if r["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
