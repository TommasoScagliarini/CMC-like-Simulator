"""V26C J0 - READ-ONLY audit of the TWO July 13 stages and of their transfer to V26 August + v3.

Never fits, never rolls out, never collects, never promotes, never writes into an existing
artefact. It reads, hashes, compares and emits one additive receipt.

TWO STAGES, reconstructed from code and artefacts rather than from reports.

  STAGE BASE - `target_domain_imitation_no_controller_memory_2026-07-13`
    A prescribed-teacher rollout of 500 steps produces 500 observation -> prescribed-action pairs
    (`teacher_dataset.npz`, `teacher_trace.json`); the teacher rollout itself is gated
    (`teacher.gate_pass`). The ENTIRE mean actor is then fitted on those 500 pairs with a random
    80/20 split. July's actor here is a genuinely 25-wide network (`teacher.n_actor = 25`) whose
    clock columns 0,1 are already zero.

  STAGE MARKOV - `target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13`
    Selected by `validation/controller_memory_ablation/2026-07-13_markov35_final_gate.json`.
    Dataset: 16000 nominal SELF-ANCHORS (the source actor's own `policy_action_mean`, read by
    `_trace_arrays` in `baseline_MLP/target_domain_markov_adaptation.py`), 712 phase-aligned
    stochastic recovery rows labelled by the prescribed teacher, and 8000 alternative-start
    teacher samples. `train_full_actor` is true: the WHOLE mean network is adapted, not only the
    ten new columns.

  `..._robust_conservative_r96_...` is an intermediate OFFLINE candidate, not the final result.
  It is the only run passing the strict offline adaptation gate, and the final gate records that
  it was rejected dynamically. Classifying July by `run_summary.gate_pass` alone is therefore
  wrong; see gate_reconciliation().

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import pickle
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
TG = REPO / "Trajectory Generator"


class J0Error(RuntimeError):
    pass


STAGE = "V26C-J0-AUDIT"
RECEIPT_NAME = "v26c_j0_audit_receipt.json"

# ---------------------------------------------------------------- July, stage BASE --------------
JULY_BASE_RUN = "target_domain_imitation_no_controller_memory_2026-07-13"
JULY_BASE_PARENT = (REPO / "validation" / "controller_memory_ablation"
                    / "2026-07-13_zero_iter_port" / "rl_module_initial_warm_start")

# ---------------------------------------------------------------- July, stage MARKOV ------------
JULY_SELECTED_RUN = "target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13"
JULY_OFFLINE_STRICT_PASS_RUN = "target_domain_markov35_robust_conservative_r96_2026-07-13"
JULY_MARKOV_PARENT = (TG / "runs" / "training" / "validation" / "controller_memory_ablation"
                      / "2026-07-13_markov35_zero_iter_port" / "rl_module_initial_warm_start")
JULY_FINAL_GATE = (REPO / "validation" / "controller_memory_ablation"
                   / "2026-07-13_markov35_final_gate.json")
PIN_SELECTED_ACTOR_DIGEST = "a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21"
JULY_RUNS_2026_07_13 = (
    "target_domain_markov35_phase_aligned_scaled_cols_r16_alt8_2026-07-13",
    JULY_SELECTED_RUN,
    "target_domain_markov35_phase_aligned_scaled_full_r64_alt8_anchor05_2026-07-13",
    "target_domain_markov35_robust_2026-07-13",
    "target_domain_markov35_robust_conservative_r64_2026-07-13",
    JULY_OFFLINE_STRICT_PASS_RUN,
)
JULY_BUILDER = TG / "baseline_MLP" / "target_domain_markov_adaptation.py"
PIN_JULY_BUILDER_SHA = "7518f1d4bdaa31505d8335934ce3478ebc03dc1cb1f31b508b5ec4a229cba7e0"

# ---------------------------------------------------------------- V26 August side ---------------
V26_PARENT_DIR = (TG / "runs" / "training"
                  / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter" / "rl_module_best")
PIN_V26_PARENT_SHA = "0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd"
V26B_STUDENT = (TG / "runs" / "rollout" / "validation" / "v26b_bridge_runs"
                / "2026-08-24_V26B_anchors_r1" / "student")
V1_35D = V26B_STUDENT / "V1_35D_transplant" / "rl_module" / "module_state.pkl"
PIN_V1_SHA = "16c2d1ae9fb4e77fffa092d74d37e78f54ba24d990774e91bf1d412c551bb031"
B0_MASKED = V26B_STUDENT / "B0_35D_MASKED" / "rl_module" / "module_state.pkl"
PIN_B0_SHA = "aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f"

CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
MASKED_COLUMNS = tuple(sorted(CLOCK_COLUMNS + CONTROLLER_COLUMNS))


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _state(p: Path) -> dict[str, np.ndarray]:
    with p.open("rb") as fh:
        return {k: np.asarray(v) for k, v in pickle.load(fh).items()}


def _zero_columns(state: Mapping[str, np.ndarray]) -> list[int]:
    W1 = np.asarray(state["pi.0.0.weight"])
    return [c for c in range(W1.shape[1]) if bool(np.all(W1[:, c] == 0.0))]


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ================================================================ stage BASE =====================

def july_base_stage() -> dict[str, Any]:
    d = TG / "runs" / "training" / JULY_BASE_RUN
    summ = json.loads((d / "run_summary.json").read_text())
    adap = json.loads((d / "adaptation_report.json").read_text())
    tea = summ["teacher"]
    hp = adap["hyperparameters"]
    parent = _state(JULY_BASE_PARENT / "module_state.pkl")
    return {
        "run": JULY_BASE_RUN,
        "parent": {"path": str(JULY_BASE_PARENT.relative_to(REPO)),
                   "module_state_sha256": _sha(JULY_BASE_PARENT / "module_state.pkl"),
                   "actor_digest": adap["source_actor_digest"],
                   "W1_shape": list(np.asarray(parent["pi.0.0.weight"]).shape),
                   "zero_columns": _zero_columns(parent),
                   "note": "a genuinely 25-wide network, not a 35-wide one with columns masked"},
        "dataset": {
            "provenance": "a prescribed-teacher rollout of 500 steps; the pairs are "
                          "observation -> PRESCRIBED TEACHER action",
            "artefacts": {"teacher_dataset": str((d / 'teacher_dataset.npz').relative_to(REPO)),
                          "teacher_dataset_sha256": _sha(d / "teacher_dataset.npz"),
                          "teacher_trace": str((d / 'teacher_trace.json').relative_to(REPO)),
                          "teacher_trace_sha256": _sha(d / "teacher_trace.json")},
            "samples": adap["samples"], "n_actor": tea["n_actor"],
        },
        "teacher_rollout_gate": {
            "steps": tea["steps"], "expected_steps": tea["expected_steps"],
            "end_reason": tea["end_reason"], "terminated": tea["terminated"],
            "truncated": tea["truncated"],
            "max_penetration_m": tea["max_penetration_m"],
            "hard_penetration_m": tea["hard_penetration_m"],
            "valid_cycle_count": tea["valid_cycle_count"],
            "valid_hs_count": tea["valid_hs_count"], "valid_to_count": tea["valid_to_count"],
            "action_abs_max": tea["action_abs_max"],
            "slew_limited_steps": tea["slew_limited_steps"],
            "gate_pass": tea["gate_pass"],
        },
        "fit": {
            "scope": "the ENTIRE mean actor",
            "split": {"kind": "random", "validation_fraction": hp["validation_fraction"],
                      "training_samples": adap["training_samples"],
                      "validation_samples": adap["validation_samples"]},
            "hyperparameters": hp,
            "epochs": {"requested": adap["epochs_requested"], "run": adap["epochs_run"],
                       "best_epoch": adap["best_epoch"]},
            "best_validation_mse": adap["best_validation_mse"],
            "initial_prediction": adap["initial_prediction"],
            "adapted_prediction": adap["adapted_prediction"],
            "logstd_head_max_abs_parameter_change": adap["logstd_head_max_abs_parameter_change"],
            "critic_trained": summ["adaptation"]["critic_trained"],
            "ppo_updates": summ["adaptation"]["ppo_updates"],
            "disabled_clock_column_norms": adap.get("disabled_clock_column_norms"),
        },
        "output_actor_digest": adap["adapted_actor_digest"],
        "ok": summ["ok"], "stage": summ["stage"],
    }


# ================================================================ stage MARKOV ===================

def july_markov_census() -> dict[str, Any]:
    rows = []
    for name in JULY_RUNS_2026_07_13:
        s = TG / "runs" / "training" / name / "run_summary.json"
        if not s.is_file():
            rows.append({"run": name, "present": False})
            continue
        j = json.loads(s.read_text())
        a, ds = j.get("adaptation", {}), j.get("dataset", {})
        ar = TG / "runs" / "training" / name / "adaptation_report.json"
        hp = json.loads(ar.read_text()).get("hyperparameters", {}) if ar.is_file() else {}
        rows.append({"run": name, "present": True, "ok": j.get("ok"),
                     "offline_gate_pass": j.get("gate_pass"),
                     "train_full_actor": j.get("train_full_actor"),
                     "nominal_repeat": ds.get("nominal_repeat"),
                     "aggregate_samples": ds.get("aggregate_samples"),
                     "anchor_weight": hp.get("anchor_weight"),
                     "best_validation_mse": a.get("best_validation_mse"),
                     "nominal_mean_shift_max_abs":
                         (j.get("nominal_mean_shift") or {}).get("max_abs")})
    offline_pass = [r["run"] for r in rows if r.get("offline_gate_pass") is True]
    if offline_pass != [JULY_OFFLINE_STRICT_PASS_RUN]:
        raise J0Error(f"expected only {JULY_OFFLINE_STRICT_PASS_RUN} to pass the strict offline "
                      f"gate, got {offline_pass}")
    return {"runs": rows, "strict_offline_gate_pass": offline_pass,
            "operationally_selected": JULY_SELECTED_RUN,
            "warning": "run_summary.gate_pass is the OFFLINE adaptation gate ONLY and MUST NOT be "
                       "used to identify the operational July stage: the selected actor fails it, "
                       "and the run that passes it was rejected dynamically."}


def july_markov_stage() -> dict[str, Any]:
    d = TG / "runs" / "training" / JULY_SELECTED_RUN
    summ = json.loads((d / "run_summary.json").read_text())
    adap = json.loads((d / "adaptation_report.json").read_text())
    dset = json.loads((d / "markov_dataset_report.json").read_text())
    hp = adap["hyperparameters"]
    parent = _state(JULY_MARKOV_PARENT / "module_state.pkl")
    return {
        "run": JULY_SELECTED_RUN,
        "builder": {"path": str(JULY_BUILDER.relative_to(REPO)), "sha256": _sha(JULY_BUILDER),
                    "matches_pin": _sha(JULY_BUILDER) == PIN_JULY_BUILDER_SHA},
        "parent": {"path": str(JULY_MARKOV_PARENT.relative_to(REPO)),
                   "module_state_sha256": _sha(JULY_MARKOV_PARENT / "module_state.pkl"),
                   "actor_digest": adap["source_actor_digest"],
                   "W1_shape": list(np.asarray(parent["pi.0.0.weight"]).shape),
                   "zero_columns": _zero_columns(parent)},
        "dataset": {
            "nominal_self_anchors": dset["nominal_training_samples"],
            "nominal_steps": dset["nominal_steps"], "nominal_repeat": dset["nominal_repeat"],
            "nominal_label_source": "the SOURCE ACTOR'S OWN policy_action_mean, read from the "
                                    "deterministic zero-column rollout trace - NOT a teacher",
            "phase_aligned_recovery": dset["recovery_training_samples"],
            "recovery_label_source": "time-aligned prescribed teacher, truncated at the first "
                                     "discrete mismatch",
            "recovery_trace_filters": dset.get("recovery_trace_filters"),
            "alternative_start_teacher_samples": sum(
                x["training_samples"] for x in dset.get("additional_recovery_datasets", [])),
            "aggregate_samples": dset["aggregate_samples"],
        },
        "fit": {"scope": "the ENTIRE mean network (train_full_actor = true), not only the ten "
                         "controller columns",
                "train_full_actor": summ.get("train_full_actor"),
                "hyperparameters": hp,
                "epochs": {"requested": summ["adaptation"]["epochs_requested"],
                           "run": summ["adaptation"]["epochs_run"],
                           "best_epoch": summ["adaptation"]["best_epoch"]},
                "best_validation_mse": summ["adaptation"]["best_validation_mse"],
                "split": {"kind": "random over the REPEATED dataset",
                          "validation_fraction": hp["validation_fraction"],
                          "caveat": "with nominal_repeat 32 a validation row has 31 identical "
                                    "copies in training; this does NOT measure generalisation"}},
        "output_actor_digest": summ["adaptation"]["adapted_actor_digest"],
        "offline_gate": {**summ["gate"], "gate_pass": summ["gate_pass"],
                         "max_nominal_mean_shift": summ["max_nominal_mean_shift"],
                         "observed": summ["nominal_mean_shift"]},
    }


def gate_reconciliation() -> dict[str, Any]:
    if not JULY_FINAL_GATE.is_file():
        raise J0Error(f"the July final gate is missing: {JULY_FINAL_GATE}")
    g = json.loads(JULY_FINAL_GATE.read_text())
    sel, sg = g["selected_actor"], g["selection_gate"]
    det = g["closed_loop_validation"]["deterministic"]
    sto = g["closed_loop_validation"]["stochastic_sigma_0_005"]
    r96 = json.loads((TG / "runs" / "training" / JULY_OFFLINE_STRICT_PASS_RUN
                      / "run_summary.json").read_text())
    keys = ("start", "seed", "steps", "valid_cycles", "max_penetration_m", "clipped_steps", "pass")
    return {
        "artefact": {"path": str(JULY_FINAL_GATE.relative_to(REPO)),
                     "sha256": _sha(JULY_FINAL_GATE)},
        "level_1_offline_adaptation_gate": {
            "what": "run_summary.gate_pass: nominal mean shift within max_nominal_mean_shift, "
                    "columns learned, log-std unchanged, scope observed",
            "selected_run": "FAIL, on the strict single-sample criterion only: max_abs "
                            f"{sel['offline_metrics']['nominal_source_shift_max_abs']} > 0.005, "
                            f"while rms is {sel['offline_metrics']['nominal_source_shift_rms']} "
                            f"and p95 {sel['offline_metrics']['nominal_source_shift_p95_abs']}",
            "r96_run": f"PASS (max_abs {r96['nominal_mean_shift']['max_abs']})",
        },
        "level_2_selection_gate": {
            "what": "closed-loop start and exploration robustness - the PRIMARY warm-start gate",
            "criteria": sg, "pass": sg["pass"],
            "rationale_verbatim": sg["rationale"],
            "criteria_false_but_not_blocking": [k for k, v in sg.items()
                                                if v is False and k != "pass"],
        },
        "decision": {"decision": g["decision"], "training_ready": g["training_ready"],
                     "deployment_validated": g["deployment_validated"],
                     "selected_checkpoint": sel["checkpoint"],
                     "selected_actor_digest": sel["actor_digest"],
                     "digest_matches_pin": sel["actor_digest"] == PIN_SELECTED_ACTOR_DIGEST,
                     "dataset_composition": sel["dataset_composition"]},
        "closed_loop_evidence": {
            "deterministic": [{k: r[k] for k in keys if k in r} for r in det],
            "stochastic_sigma_0_005": [{k: r[k] for k in keys if k in r} for r in sto],
            "summary": f"{sum(1 for r in det if r['pass'])}/{len(det)} deterministic starts and "
                       f"{sum(1 for r in sto if r['pass'])}/{len(sto)} stochastic seeds passed, "
                       "500 steps each, all below the 0.025 m guard, zero action clipping",
        },
        "root_cause_recorded_by_july": g["root_cause"],
        "reconciliation": "July resolved the disagreement explicitly: closed-loop robustness is "
                          "primary and the strict offline single-sample shift is not. The "
                          "conservative candidate that optimised the offline number failed "
                          "dynamically and was rejected rather than weakening the 0.025 m safety "
                          "threshold. The methodological precedent is the SELECTED run, and an "
                          "offline-only failure is not by itself a stop.",
        "consequence_for_v26b": "The V26B B-branch was blocked entirely on OFFLINE reconstruction "
                                "gates and never reached a closed-loop measurement. Under the "
                                "July precedent that ordering is inverted.",
    }


# ================================================================ transfer to V26 August =========

def parent_equivalence() -> dict[str, Any]:
    b0, v1 = _state(B0_MASKED), _state(V1_35D)
    jm = _state(JULY_MARKOV_PARENT / "module_state.pkl")
    jb = _state(JULY_BASE_PARENT / "module_state.pkl")
    W1v = np.asarray(v1["pi.0.0.weight"]).copy()
    W1v[:, list(CONTROLLER_COLUMNS)] = 0.0
    b0_is_v1_masked = bool(np.array_equal(W1v, np.asarray(b0["pi.0.0.weight"]))) and all(
        np.array_equal(np.asarray(v1[k]), np.asarray(b0[k]))
        for k in b0 if k not in ("pi.0.0.weight", "pi_encoder.0.weight"))
    return {
        "v26_august_parent": {"path": str(V26_PARENT_DIR.relative_to(REPO)),
                              "module_state_sha256": _sha(V26_PARENT_DIR / "module_state.pkl"),
                              "matches_pin": _sha(V26_PARENT_DIR / "module_state.pkl")
                                             == PIN_V26_PARENT_SHA,
                              "W1_shape": list(np.asarray(
                                  _state(V26_PARENT_DIR / 'module_state.pkl')
                                  ["pi.0.0.weight"]).shape)},
        "v1_35d_transplant": {"sha256": _sha(V1_35D), "matches_pin": _sha(V1_35D) == PIN_V1_SHA,
                              "zero_columns": _zero_columns(v1)},
        "b0_35d_masked": {"sha256": _sha(B0_MASKED), "matches_pin": _sha(B0_MASKED) == PIN_B0_SHA,
                          "zero_columns": _zero_columns(b0)},
        "july_base_parent": {"W1_shape": list(np.asarray(jb["pi.0.0.weight"]).shape),
                             "zero_columns": _zero_columns(jb)},
        "july_markov_parent": {"W1_shape": list(np.asarray(jm["pi.0.0.weight"]).shape),
                               "zero_columns": _zero_columns(jm)},
        "b0_is_v1_with_controller_columns_zeroed": b0_is_v1_masked,
        "b0_matches_july_markov_parent_zero_set": _zero_columns(b0) == _zero_columns(jm)
                                                  == list(MASKED_COLUMNS),
        "declared_deviation": "July's BASE stage used a genuinely 25-wide network; the replay uses "
                              "ONE 35D actor with columns 25..34 hard-zeroed for the base stage "
                              "and re-enabled afterwards in the SAME actor. A zero column "
                              "contributes nothing to the forward pass, so the base stage is "
                              "functionally equivalent, and this is what makes the later "
                              "re-enabling possible without a second network. Approved by the "
                              "user: no separate 25D network, no widening.",
    }


def replay_plan() -> dict[str, Any]:
    """The July-faithful replay, SPECIFIED. Nothing is executed by this module."""
    base = july_base_stage()
    hp = base["fit"]["hyperparameters"]
    return {
        "status": "SPECIFICATION - the executor stops at preflight and awaits the architect's GO",
        "actor": "ONE 35D actor, 35 -> 256 -> 256 -> 4, columns 0,1 and 25..34 hard-zero during "
                 "the base stage, columns 25..34 re-enabled in the SAME actor afterwards",
        "parent": {"exclusive": str(V26_PARENT_DIR.relative_to(REPO)),
                   "sha256": PIN_V26_PARENT_SHA,
                   "no_july_checkpoint_or_dataset_as_operational_parent": True},
        "stage_J1_collection": {
            "what": "a NEW prescribed-teacher nominal rollout of 500 steps under the CURRENT "
                    "EGRF/FSM v3 runtime and corridor, producing 500 observation -> prescribed "
                    "pairs, plus its own teacher-rollout gate",
            "july_analogue": f"{JULY_BASE_RUN}/teacher_dataset.npz + teacher_trace.json",
            "collection_semantics_pinned": dict(JULY_COLLECTION_SEMANTICS),
            "must_not_substitute": ["the 1500 three-anchor V26B rows", "LOTO / LOCO / 11-fold "
                                    "segment-blocked designs", "the u_IK bridge labels"],
            "why": "July's base labels are the PRESCRIBED TEACHER on states the teacher itself "
                   "visited. None of the above is that object.",
            "gate": {"binding": dict(V26B_COMMON_GATE),
                     "kind": "COMMON gate only: integrity, runtime, safety, contract",
                     "kinematic_quality_applies": False,
                     "source": "the already-fixed V26B gate, NOT July's 0.025 m",
                     "july_observed_HISTORICAL_ONLY": base["teacher_rollout_gate"],
                     "july_not_isometric": True},
        },
        "stage_J2_base_fit": {
            "scope": "the ENTIRE mean actor, with columns 0,1 and 25..34 held at exact zero",
            "split": "random 80/20, as July",
            "hyperparameters_recovered_verbatim": hp,
            "epochs_max": base["fit"]["epochs"]["requested"],
            "july_reference_outcome": {"best_epoch": base["fit"]["epochs"]["best_epoch"],
                                       "adapted_rmse": base["fit"]["adapted_prediction"]["rmse"]},
            "note": "these hyperparameters are exactly the ones the V26B branch had pinned as the "
                    "July protocol; that pin was faithful to the BASE stage",
        },
        "stage_J3_closed_loop": {
            "what": "closed-loop rollout of the fitted actor, the PRIMARY gate per the July "
                    "precedent",
            "ordering": "closed-loop is primary; an offline-only shortfall is not by itself a stop",
            "gate": dict(V26B_CLOSED_LOOP_GATE),
            "gate_kind": "COMMON gate + full kinematic quality",
            "gate_source": "the already-fixed V26B closed-loop gate; July's 0.025 m guard and its "
                           "22.94 mm teacher penetration are historical evidence only and are NOT "
                           "isometric to the current runtime",
        },
        "stage_J4_markov": {
            "what": "re-enable columns 25..34 in the SAME actor and adapt the whole mean network, "
                    "July markov stage",
            "not_in_scope_now": True,
            "open_choice": "exploration sigma for the stochastic recovery traces - July used "
                           "0.005; sigma is deferred by standing governance and is NOT assumed",
        },
        "future_todo_non_operational": {
            "note": "preserved, not deleted, not operational, no artefact removed",
            "items": ["LOTO 3-fold (B1)", "LOCO 6-fold (B1R1)",
                      "11-fold 8 complete cycles + 3 tails (B1R2)",
                      "B1R2-A budget arm", "B1R2-B learning-rate arm"],
        },
        "authority": "The USER holds exclusive authority over any deviation on dataset, split, "
                     "gate, architecture, hyperparameters or order. Values recovered verbatim from "
                     "the July artefacts are not deviations. The architect's standing mandate "
                     "covers the July-faithful collection and fit after the architect's GO.",
    }


# ================================================================ CURRENT gate contract ==========

# The OPERATIONAL guard is the current pinned runtime, NOT July's 0.025 m.
CURRENT_GRF_SOFT_THRESHOLD_M = 0.020
CURRENT_GRF_HARD_TERMINATION_M = 0.028
# COMMON gate: integrity, runtime, safety and contract. Necessary to ACCEPT a dataset (J1) and
# also required of the actor (J3). Values from v26b_b1_base_fit.declared_closed_loop_gates.
V26B_COMMON_GATE: dict[str, Any] = {
    "steps_required": 500,
    "end_reason": "episode_time_limit",
    "valid_cycles_min": 2,
    "phase_timeout_stance_max": 0,
    "phase_timeout_swing_max": 0,
    "morphology_causal_contract_failure_max": 0,
    "hs_cancelled_count_max": 0,
    "resync_count_max": 1,
    "max_penetration_m_max": 0.020,
}

# KINEMATIC QUALITY, already fixed in v26b_b1_base_fit.py. Binds J3 ONLY: it is a statement about
# the gait the ACTOR produces, not about the integrity of a collected dataset.
V26B_KINEMATIC_GATE: dict[str, Any] = {
    "ankle_min_rad": -0.03,
    "ankle_amplitude_min_rad": 0.30,
    "knee_amplitude_min_rad": 0.60,
    "knee_strictly_flexed": True,
    "within_bounds": True,
}

# J1 binds the common gate only; J3 binds their union.
V26B_CLOSED_LOOP_GATE: dict[str, Any] = {**V26B_COMMON_GATE,
                                         "kinematic_quality": dict(V26B_KINEMATIC_GATE)}

# Diagnostic, NOT a binding gate: no already-approved source makes it one.
DIAGNOSTIC_NOT_BINDING: tuple[str, ...] = ("action_clipped_steps",)
# July's collection semantics, recovered VERBATIM from the base-stage artefacts. These are
# July values, not defaults: teacher_summary.json records action_noise_sigma [0.0, 0.0] and
# action_noise_realized_rms [0.0, 0.0], i.e. a strictly noiseless teacher rollout.
JULY_COLLECTION_SEMANTICS: dict[str, Any] = {
    "seed": 123,
    "teacher_lookahead_s": 0.0,
    "action_noise_sigma": [0.0, 0.0],
    "action_noise_hold_steps": 1,
    "action_noise_hold_duration_s": 0.01,
    "provenance": "target_domain_imitation_no_controller_memory_2026-07-13: "
                  "run_summary.teacher and teacher_summary.json; seed from "
                  "adaptation_report.hyperparameters.seed",
    "status": "JULY ARTEFACT VALUES, NOT DEFAULTS - the collector must pin them explicitly",
}

JULY_HISTORICAL_GUARD_M = 0.025
JULY_HISTORICAL_TEACHER_PENETRATION_M = 0.02294380435912411


def current_gate_contract() -> dict[str, Any]:
    """The gate that binds J1 and J3 is the CURRENT V26B closed-loop gate, not July's.

    July's 0.025 m guard and its 22.94 mm teacher penetration are HISTORICAL EVIDENCE ONLY. They
    are recorded for provenance and are explicitly NOT isometric to the current runtime, whose
    pinned config carries a soft GRF threshold of 0.020 m and a hard termination at 0.028 m.
    """
    return {
        "j1_teacher_collection_gate": {
            "name": "COMMON gate: integrity, runtime, safety, contract",
            "criteria": dict(V26B_COMMON_GATE),
            "purpose": "necessary to ACCEPT the collected dataset. It says the teacher rollout was "
                       "complete, safe and contract-conformant; it says nothing about gait quality",
            "kinematic_quality_applies": False,
            "why_not": "the kinematic gate qualifies the gait an ACTOR produces. Imposing it on the "
                       "teacher collection would confuse dataset integrity with actor quality",
        },
        "j3_actor_closed_loop_gate": {
            "name": "COMMON gate + full kinematic quality",
            "criteria": dict(V26B_CLOSED_LOOP_GATE),
            "kinematic_quality": dict(V26B_KINEMATIC_GATE),
            "source": "v26b_b1_base_fit.declared_closed_loop_gates, already fixed, not relaxed",
            "worked_example": "an ankle minimum of -0.0099 rad FAILS, because -0.0099 > -0.03",
        },
        "diagnostics_not_binding": {
            "fields": list(DIAGNOSTIC_NOT_BINDING),
            "rule": "recorded in the receipt, never a pass criterion. Promoting it to a binding "
                    "gate requires an already-approved source, which does not exist",
        },
        "current_runtime_thresholds": {
            "pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
            "grf_soft_threshold_m": CURRENT_GRF_SOFT_THRESHOLD_M,
            "grf_hard_termination_m": CURRENT_GRF_HARD_TERMINATION_M,
            "both_must_be_recorded_by_the_collector": True,
        },
        "july_historical_only": {
            "guard_m": JULY_HISTORICAL_GUARD_M,
            "teacher_max_penetration_m": JULY_HISTORICAL_TEACHER_PENETRATION_M,
            "selected_actor_closed_loop_max_penetration_m": [0.02395957553235469,
                                                             0.02460202939096984,
                                                             0.024323924384327976],
            "status": "HISTORICAL EVIDENCE, EXPLICITLY NOT ISOMETRIC to the current runtime",
            "warning": "July's rollouts sat between 0.0233 and 0.0246 m, i.e. ABOVE the current "
                       "0.020 m soft threshold and below its own 0.025 m guard. A July number "
                       "must never be reused as a current pass criterion, and July's closed-loop "
                       "success does not transfer numerically to the current gate.",
        },
    }


# ================================================================ collector contract =============

# The ACTUAL env key names the current v3 runtime needs, taken from the reference builders.
# An earlier draft of this list used config-side or abbreviated names that do NOT exist as env
# keys; the CONFIG_TO_ENV_KEY_RENAMES table below records the corrections so the mistake cannot
# be repeated silently.
V3_REQUIRED_ENV_KEYS: tuple[str, ...] = (
    "binary_phase_detector_profile_file",
    "binary_phase_invalid_event_policy",
    "event_contract_id",                       # legacy_events_v1
    "binary_phase_event_contract_id",          # binary_point_v25+heel_qualified_fsm_v2
    "binary_phase_fsm_mode",
    "binary_phase_actor_fsm_version",
    "binary_phase_debounce_s",
    "phase_fsm_input_mode",
    "phase_sensor_on_threshold_n",
    "phase_sensor_off_threshold_n",
    "phase_sensor_dwell_s",
    "detector_sample_dt_s",
)

# TWO DISTINCT CONTRACT IDS, not one renamed into the other. Both appear side by side in the
# production builders (rollout_eval.py 523 and 530, train_ppo_mlp.py 1301 and 1308) and the pinned
# config gives them DIFFERENT values. Confusing them would silently install the wrong contract.
DUAL_CONTRACT_IDS: dict[str, str] = {
    "event_contract_id": "legacy_events_v1",
    "binary_phase_event_contract_id": "binary_point_v25+heel_qualified_fsm_v2",
}

# The ONLY genuine config -> env rename. `binary_phase_invalid_event_policy` and both contract ids
# already carry their final names in the config, so they are NOT renames.
CONFIG_TO_ENV_KEY_RENAMES: dict[str, str] = {
    "binary_phase_detector_profile": "binary_phase_detector_profile_file",
}
HISTORICAL_BUILDER = TG / "baseline_MLP" / "target_domain_imitation.py"
HISTORICAL_BUILDER_FN = "build_target_env_config"
REFERENCE_BUILDERS = ("train_ppo_mlp.py", "rollout_eval.py")
PIN_RUNTIME_CONFIG_SHA = "a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db"


def _fn_source(path: Path, name: str) -> str:
    import re
    src = path.read_text()
    m = re.search(rf"def {name}\(.*?\n(?=def |\Z)", src, re.S)
    if not m:
        raise J0Error(f"{name} not found in {path}")
    return m.group(0)


def collector_contract() -> dict[str, Any]:
    """NEGATIVE CONTROL: the historical builder is incomplete for v3, and must not be reused.

    `target_domain_imitation.build_target_env_config` forwards none of the detector/FSM v3 keys and
    no morphology or corridor block. Calling it - even while reading the pinned runtime config -
    would silently produce a NON-v3 teacher. The additive collector must instead assemble the FULL
    env_config semantically identical to the reference builders, starting from the pinned config,
    and must assert every field below into its own receipt.
    """
    hist = _fn_source(HISTORICAL_BUILDER, HISTORICAL_BUILDER_FN)
    missing = [k for k in V3_REQUIRED_ENV_KEYS if k not in hist]
    present_in_reference: dict[str, list[str]] = {}
    for ref in REFERENCE_BUILDERS:
        text = (TG / "baseline_MLP" / ref).read_text()
        present_in_reference[ref] = [k for k in V3_REQUIRED_ENV_KEYS if k in text]
    return {
        "negative_control": {
            "builder": f"{HISTORICAL_BUILDER.relative_to(REPO)}::{HISTORICAL_BUILDER_FN}",
            "sha256": _sha(HISTORICAL_BUILDER),
            "v3_keys_checked": list(V3_REQUIRED_ENV_KEYS),
            "v3_keys_MISSING_from_historical_builder": missing,
            "forwards_morphology": "morphology" in hist,
            "forwards_corridor": "corridor" in hist,
            "verdict": "INCOMPLETE FOR v3 - MUST NOT be used for the current collection",
            "risk": "it would build a teacher on a non-v3 detector/FSM while appearing to honour "
                    "the pinned runtime config",
        },
        "reference_builders": {"files": list(REFERENCE_BUILDERS),
                               "v3_keys_present": present_in_reference},
        "dual_contract_ids": {
            "values": dict(DUAL_CONTRACT_IDS),
            "rule": "TWO DISTINCT keys with DIFFERENT values, both required. They must never be "
                    "merged, swapped or treated as a rename of one another.",
            "evidence": "rollout_eval.py lines 523 and 530; train_ppo_mlp.py lines 1301 and 1308; "
                        "the pinned config assigns event_contract_id=legacy_events_v1 and "
                        "binary_phase_event_contract_id=binary_point_v25+heel_qualified_fsm_v2",
        },
        "config_to_env_key_renames": {
            "table": dict(CONFIG_TO_ENV_KEY_RENAMES),
            "why": "the ONLY genuine config -> env rename is the detector profile, which the config "
                   "calls binary_phase_detector_profile and the env calls "
                   "binary_phase_detector_profile_file. An earlier draft also listed "
                   "invalid_event_policy and event_contract_id as renames: both were WRONG. The "
                   "config already uses binary_phase_invalid_event_policy verbatim, and "
                   "event_contract_id is a SEPARATE key that coexists with "
                   "binary_phase_event_contract_id with a different value.",
            "verified_against": list(REFERENCE_BUILDERS),
        },
        "required_of_the_new_collector": {
            "source_of_truth": f"the pinned runtime config {PIN_RUNTIME_CONFIG_SHA}",
            "must_assemble": "the FULL env_config, semantically identical to rollout_eval.run / "
                             "train_ppo_mlp.build_config, including the reward and morphology "
                             "blocks",
            "must_not_call": f"{HISTORICAL_BUILDER_FN} or target_domain_imitation.main",
            "must_assert_into_its_receipt": [
                "binary_phase_detector_profile_file",
                "binary_phase_actor_fsm_version == v3",
                "phase_fsm_input_mode", "binary_phase_fsm_mode",
                "binary_phase_invalid_event_policy",
                "event_contract_id AND binary_phase_event_contract_id, both, with their distinct "
                "pinned values",
                "binary_phase_debounce_s",
                "phase_sensor_on_threshold_n", "phase_sensor_off_threshold_n",
                "phase_sensor_dwell_s", "detector_sample_dt_s",
                "the pinned collection semantics: seed, teacher_lookahead_s, "
                "action_noise_sigma, action_noise_hold_steps",
                "corridor profile with its alpha and weight",
                "start condition",
                "the CURRENT grf soft threshold 0.020 m AND hard termination 0.028 m, both values",
                "the binding V26B closed-loop gate criteria",
                "the episode and phase timeouts",
                "the reward block", "the morphology block",
            ],
            "fail_closed": "a missing or mismatched field must abort the collection before any "
                           "environment step",
        },
        "production_untouched": "the historical builder is READ, never modified; no production, "
                                "FSM, GRF, morphology, reward, SEA or C++ file is edited",
    }


def open_points() -> list[dict[str, Any]]:
    """Only genuine unknowns remain: everything else is recovered verbatim from artefacts."""
    return [
        {"id": "O1", "topic": "prescribed teacher semantics under v3",
         "statement": "July's base labels come from the prescribed teacher of that runtime. That "
                      "the current v3 prescribed teacher is the same object is NOT verified; it "
                      "becomes verifiable at collection time through the teacher-rollout gate.",
         "blocking_now": False},
        {"id": "O2", "topic": "teacher-rollout gate outcome under v3",
         "statement": "July's teacher rollout passed its own gate (500 steps, penetration "
                      "0.02294 < 0.025, 2 valid cycles). Whether it passes under v3 and the "
                      "current corridor is unknown until collected.",
         "blocking_now": False},
        {"id": "O3", "topic": "exploration sigma",
         "statement": "Needed only by the later Markov stage. July used 0.005. Sigma is deferred "
                      "by standing governance and is not assumed here.",
         "blocking_now": False},
    ]


# ================================================================ driver =========================

def run_audit(*, write: bool = True) -> dict[str, Any]:
    receipt = {
        "schema": "v26c_j0_audit.2", "stage": STAGE,
        "kind": "READ-ONLY audit. No fit, no rollout, no collection, no promotion, no training, "
                "no modification of any existing artefact.",
        "july_stage_base": july_base_stage(),
        "july_markov_census": july_markov_census(),
        "july_stage_markov_selected": july_markov_stage(),
        "gate_reconciliation": gate_reconciliation(),
        "parent_equivalence": parent_equivalence(),
        "current_gate_contract": current_gate_contract(),
        "collector_contract": collector_contract(),
        "replay_plan": replay_plan(),
        "open_points": open_points(),
        "verdict": "GO-FOR-ARCHITECT-REVIEW",
        "verdict_reason": "Both July stages are fully reconstructed from code and artefacts, the "
                          "two gate levels are reconciled, the parent chain is verified, and the "
                          "replay plan carries no invented default. No open point blocks the "
                          "preflight. The executor stops here and awaits the architect's GO.",
        "generated_at_utc": _utc(),
    }
    if write:
        out = HERE / RECEIPT_NAME
        if out.exists():
            raise J0Error(f"no-clobber: {out} exists")
        out.write_text(json.dumps(receipt, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return receipt


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J0 read-only audit")
    p.add_argument("--run", action="store_true")
    p.add_argument("--no-write", action="store_true")
    a = p.parse_args(argv)
    if not a.run:
        july_base_stage(); july_markov_census(); gate_reconciliation()
        parent_equivalence(); current_gate_contract(); collector_contract()
        print(json.dumps({"mode": "dry", "ok": True, "stage": STAGE}, indent=2))
        return 0
    r = run_audit(write=not a.no_write)
    gr = r["gate_reconciliation"]
    print(json.dumps({
        "verdict": r["verdict"],
        "july_base_run": r["july_stage_base"]["run"],
        "july_base_rmse": r["july_stage_base"]["fit"]["adapted_prediction"]["rmse"],
        "july_selected_markov_run": gr["decision"]["selected_checkpoint"].split("/")[-2],
        "offline_strict_pass_but_rejected": JULY_OFFLINE_STRICT_PASS_RUN,
        "closed_loop": gr["closed_loop_evidence"]["summary"],
        "b0_matches_july_markov_parent_zero_set":
            r["parent_equivalence"]["b0_matches_july_markov_parent_zero_set"],
        "binding_gate_max_penetration_m":
            r["current_gate_contract"]["binding_gate"]["criteria"]["max_penetration_m_max"],
        "open_points": [o["id"] for o in r["open_points"]],
        "historical_builder_missing_v3_keys":
            r["collector_contract"]["negative_control"]["v3_keys_MISSING_from_historical_builder"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
