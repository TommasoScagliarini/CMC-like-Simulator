"""V26C J4 - July-faithful RECOVERY stage for the covariate shift measured at J3.

WHAT THIS IS
    The recovery correction reconstructed from the July sources: the states the student ACTUALLY
    visited in the pinned J3 rollout, relabelled with the time-aligned prescribed teacher action,
    aggregated with SELF-DISTILLED nominal anchors, and used for ONE actor-only supervised update
    of the SAME 35D student. The composition of the SELECTED final July Markov run is used
    coherently - nominal 500 x 32 and recovery 12 x 2 - never mixed with the 11 July 1x/1x one.

THE ALIGNMENT RULE IS BINDING
    Each recovery trace is truncated at its FIRST discrete mismatch against the nominal teacher
    trace. After that step the fixed-step teacher labels are phase-invalid, so they are not used.
    Source: target_domain_noise_adaptation.truncate_before_discrete_mismatch, enabled by
    target_domain_markov_adaptation --stop-before-discrete-mismatch (default True).

THERE IS NO MINIMUM-LENGTH GATE
    July's code has none: its 119/118/119 prefixes are descriptive outcomes. The literal July
    fail-closed condition is truncation at the first discrete mismatch plus a NON-EMPTY, aligned
    corpus. A short aligned prefix may therefore produce a QUARANTINED candidate, whose quality is
    judged only by the subsequent closed-loop revalidation.

ONE DECLARED DEROGATION
    The two July teacher datasets at start offsets -0.20 and +0.20 rad are NOT reproduced.
    Multistart is deferred BY USER DECISION, and that omission is declared in the amendment, the
    preflight and the receipt.

THE MASK CHANGES AT THIS STAGE, THE ACTOR DOES NOT
    The student stays 35D and is NOT widened. The clock columns [0, 1] remain hard-zero. The ten
    controller-memory columns [25..34] become TRAINABLE, with their physical scales absorbed into
    the first layer, exactly as in the selected 13 July path. Contralateral features remain absent:
    controller memory is the prosthesis' own deployable internal state, not a contralateral signal.

WHAT IT DOES NOT DO
    No retry of J3. No sigma or exploration noise. No critic, no PPO update. No ex-novo. No
    promotion. No LOTO/LOCO/B1R1/B1R2. July is a METHODOLOGICAL source only: no July artefact is
    ever an operational parent, dataset or label.

THE J3 FAIL STANDS
    The J3 receipt keeps its FAIL and its quarantine. This stage neither re-scores nor promotes it.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
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

import v26c_j1_collect as J1  # noqa: E402
import v26c_j2_fit as J2  # noqa: E402
import v26c_j3_closed_loop as J3  # noqa: E402


class J4Error(RuntimeError):
    pass


STAGE = "V26C-J4-RECOVERY"
RECEIPT_NAME = "v26c_j4_recovery_receipt.json"
OUT_ROOT = HERE / "j4_runs"

# ------------------------------------------------------------------ pinned inputs ---------------
AMENDMENT = HERE / "v26c_j3_amendment_recovery_only.json"
PIN_AMENDMENT = "fed5b81666782902ed4ab0187da457cdfbf0516dd676c049fdfe41e48d21614f"
J1_LEAF = J1.OUT_ROOT / "j1_nominal_v26c_2026-08-26_r1"
J3_LEAF = J3.OUT_ROOT / "j3_base_v26c_2026-08-26_r1"
PIN_INPUTS: dict[str, str] = {
    "j1_runs/j1_nominal_v26c_2026-08-26_r1/teacher_dataset.npz":
        "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41",
    "j1_runs/j1_nominal_v26c_2026-08-26_r1/teacher_trace.json":
        "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3",
    "j1_runs/j1_nominal_v26c_2026-08-26_r1/v26c_j1_collection_receipt.json":
        "f54028d58dc9bfde01ede3c2a72f7ea63b67aeead02979291b03cd468bf37cdd",
    "j3_runs/j3_base_v26c_2026-08-26_r1/j3_trace.json":
        "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae",
    "j3_runs/j3_base_v26c_2026-08-26_r1/j3_kinematics.npz":
        "402040d7cc794a26213ece8abd04a6e945fa46050f20d4ecc402bf78df0b97fc",
    "j3_runs/j3_base_v26c_2026-08-26_r1/v26c_j3_closed_loop_receipt.json":
        "34d856b0b4acabd000a1e6257767c6049a1f9f2147eb99d6f3801ca7559ff422",
    "v26c_j1_amendment_soft_fail.json":
        "db2aa552ab517ed3f2f8f5a74e276a88b549d24ef6d931f2fe3521cf34906bd9",
}
# ------------------------------------------------------------------ the single J4 execution -----
# ONE fit ran, completed training, restored the best state, passed every pre-write verification and
# wrote its six payloads. It then aborted while assembling the receipt. The payloads below are the
# preserved evidence of that single execution and are FROZEN by exact hash: they are never
# rewritten, and the post-crash finalisation may only read them.
J4_LEAF = OUT_ROOT / "j4_recovery_v26c_2026-08-26_r1"
PIN_J4_PAYLOADS: dict[str, str] = {
    "rl_module/module_state.pkl":
        "14a3630f757a5da2055eb754f6249fad8e7989a6d5e6c18f526c76415dad31aa",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "rl_module/actor_feature_manifest.json":
        "3454a6de085a14510874af8222e266eacac1dd194460e26902ffa29606df3c03",
    "recovery_dataset.npz":
        "28eda638bd5441698611fce7f9d9b65660b3e21581805434f41c57d1a745357e",
    "history.json":
        "e8a07fdfb77656a1a9f3c4d158b949f8a793c015106396015761748d50fb6b87",
}
# The runner exactly as it was when it produced those payloads, before the shadowing fix.
PIN_FAILING_RUNNER = "dbf46cd559a1e6da6bf0ceb8ef29874db2fc311a91697b8c0e389958d67ae75d"
ORIGINAL_LOG = Path("/private/tmp/claude-501/-Users-tommy-Documents-CMC-like-Simulator---Claude"
                    "/3f19a1c0-cfce-40b3-bbf6-4703f4fb4c90/scratchpad/j4_fit.log")
PIN_ORIGINAL_LOG = "88e9312626d23ce49685576e263df1b93c0f4f8d077415cc1d0bdaadb188dbcf"
ORIGINAL_EXIT_CODE = 1
ORIGINAL_EXCEPTION = ("TypeError: int() argument must be a string, a bytes-like object or a real "
                     "number, not 'slice'")
ORIGINAL_FAILURE_SITE = ("v26c_j4_recovery.py::fit, receipt assembly: \"split\": {\"rows\": "
                         "int(rows), ...} - `rows` had been shadowed by the log-std verification "
                         "loop variable, so it held a slice instead of the aggregate row count")
FORENSIC_LOG_NAME = "j4_fit_original.log"
FINALISATION_WRITES = (RECEIPT_NAME, FORENSIC_LOG_NAME)

# Tolerance for proving the saved payload IS the restored best state. The training-time validation
# MSE came from float32 arithmetic over 3205 x 2 values, and the payload additionally carries the
# physical scales folded into the first layer (divisions by up to 60.0). float32 eps is 1.19e-7, so
# a relative bound two orders above it is the right size; atol keeps the test meaningful if the MSE
# were ever near zero. The observed relative difference on this incident is 6.6e-7.
BEST_STATE_RTOL = 1e-5
BEST_STATE_ATOL = 1e-12
BEST_STATE_TOLERANCE_RATIONALE = (
    "float32 accumulation over the validation rows plus the first-layer scale absorption; "
    "rtol 1e-5 is ~84x the float32 epsilon of 1.19e-7, atol 1e-12 guards a near-zero MSE")

PIN_J2_MODULE = dict(J3.PIN_J2)
J2_MODULE_DIR = J3.J2_MODULE_DIR

ACTOR_WIDTH = J1.ACTOR_WIDTH
CLOCK_COLUMNS = J2.CLOCK_COLUMNS                    # (0, 1) - hard-zero at EVERY stage
CONTROLLER_COLUMNS = J2.CONTROLLER_COLUMNS          # (25..34) - TRAINABLE at this stage only

# ------------------------------------------------------------------ July provenance -------------
# Every rule below is quoted from a July source file or a July run artefact. Nothing is invented.
JULY_SOURCES: dict[str, str] = {
    "aggregation":
        "baseline_MLP/target_domain_markov_adaptation.py::build_markov_recovery_dataset - the "
        "SELF-DISTILLED nominal block (states from the nominal trace, labels from _actor_means of "
        "the source actor) is tiled nominal_repeat times and placed FIRST, then the recovery block "
        "(visited states, teacher_actions[:len]) is tiled recovery_repeat times; weighting is by "
        "REPETITION, never by sample weights. This is the composition used here",
    "aggregation_not_used":
        "baseline_MLP/target_domain_imitation.py::aggregate_dagger_traces - the EARLIER 11 July "
        "composition, whose anchor block is the TEACHER dataset at 1x. It is NOT the composition "
        "used here and the two are never mixed",
    "nominal_self_distillation":
        "baseline_MLP/target_domain_markov_adaptation.py::_actor_means -> "
        "target_domain_noise_adaptation.py::_forward, evaluated on float32 observations and "
        "float32 weights, returning logits[:, :2] as float",
    "truncation":
        "baseline_MLP/target_domain_noise_adaptation.py::truncate_before_discrete_mismatch, "
        "enabled by target_domain_markov_adaptation.py --stop-before-discrete-mismatch "
        "(default True)",
    "discrete_features":
        "baseline_MLP/target_domain_noise_adaptation.py::_discrete_feature_indices - names ending "
        "in _in_contact/_heel_strike/_toe_off/_saturated or starting with phase_fsm_/phase_expected_",
    "trainable_columns":
        "baseline_MLP/target_domain_markov_adaptation.py::MARKOV_CONTROLLER_FEATURE_SUFFIXES and "
        "markov_feature_names - exactly the 10 deployable controller features, excluding "
        "_sea_u_abs and _sea_u_saturated",
    "feature_scales":
        "baseline_MLP/target_domain_markov_adaptation.py::MARKOV_CONTROLLER_FEATURE_SCALES",
    "scale_absorption":
        "baseline_MLP/target_domain_imitation.py::adapt_actor - training runs on obs/scale, then "
        "first_layer_weight[:, index].div_(scale) folds the scale into the first layer so the "
        "deployed module consumes RAW inputs",
    "clock_hard_zero":
        "baseline_MLP/target_domain_imitation.py::_zero_disabled_clock_columns with "
        "warm_start.DISABLED_GAIT_CLOCK_FEATURES = ('gait_phase_sin', 'gait_phase_cos'), "
        "re-applied after EVERY optimiser step and once more before saving",
    "hyperparameters":
        "runs/training/target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/"
        "run_summary.json::adaptation.hyperparameters (the SELECTED July run)",
    "iteration_growth":
        "runs/training/target_domain_dagger_2026-07-11_r1|r2|r3 - the usable visited corpus grew "
        "68 -> 113 -> 469 samples across iterations",
}

# The SELECTED July run's hyperparameters, copied value for value. NOT defaults, NOT invented.
JULY_RECOVERY_HP: dict[str, Any] = {
    "seed": 123,
    "epochs": 400,
    "batch_size": 128,
    "learning_rate": 5e-05,
    "validation_fraction": 0.2,
    "patience": 60,
    "clip_weight": 1.0,
    "logstd_weight": 0.0,
    "anchor_weight": 0.01,
    "freeze_logstd_head": True,
    "train_full_actor": True,
    "scale_markov_inputs": True,
}
JULY_RECOVERY_REFERENCE = {
    "run": "target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13",
    "epochs_requested": 400, "epochs_run": 400, "best_epoch": 392,
    "best_validation_mse": 6.632255099248141e-05,
    "samples": 24712, "training_samples": 19770, "validation_samples": 4942,
    "critic_trained": False, "ppo_updates": 0,
    "offline_gate_pass": False,
    "note": "this run FAILED the strict offline gate and was still SELECTED on closed-loop "
            "evidence; it is a methodological precedent, never an operational input",
}

# July's own composition of the SELECTED run, recorded so the V26C ratios can be justified.
JULY_RECOVERY_COMPOSITION = {
    "nominal_steps": 500, "nominal_repeat": 32, "nominal_training_samples": 16000,
    "recovery_steps": 356, "recovery_repeat": 2, "recovery_training_samples": 712,
    "additional_recovery_datasets": 2, "recovery_dataset_repeat": 8,
    "additional_training_samples": 8000,
    "aggregate_samples": 24712,
    "retained_prefixes": [119, 118, 119], "first_mismatch_steps": [120, 119, 120],
    "recovery_fraction_of_aggregate": 712 / 24712,
}

MARKOV_CONTROLLER_FEATURE_SCALES: dict[str, float] = {
    "pros_knee_angle_previous_endpoint": 1.0,
    "pros_knee_angle_served_ref": 1.0,
    "pros_knee_angle_served_ref_vel": 4.0,
    "pros_knee_angle_served_ref_accel": 60.0,
    "pros_knee_angle_sea_u": 1.0,
    "pros_ankle_angle_previous_endpoint": 1.0,
    "pros_ankle_angle_served_ref": 1.0,
    "pros_ankle_angle_served_ref_vel": 3.5,
    "pros_ankle_angle_served_ref_accel": 55.0,
    "pros_ankle_angle_sea_u": 1.0,
}
MARKOV_CONTROLLER_FEATURE_SUFFIXES = ("_previous_endpoint", "_served_ref", "_served_ref_vel",
                                      "_served_ref_accel", "_sea_u")
DISABLED_GAIT_CLOCK_FEATURES = ("gait_phase_sin", "gait_phase_cos")

# The composition of the SELECTED final July Markov run, used COHERENTLY. It is never mixed with
# the 11 July DAgger 1x/1x composition.
NOMINAL_REPEAT = 32          # July: nominal_repeat 32 ("r32")
RECOVERY_REPEAT = 2          # July: recovery_repeat 2
MULTISTART_DEROGATION = {
    "omitted": True,
    "what": "the two extra teacher datasets at start offsets -0.20 and +0.20 rad, which July "
            "repeated 8x for 8000 of its 24712 rows",
    "why": "DEFERRED BY USER DECISION. This is the SINGLE explicit derogation from the selected "
           "July composition and is declared, not silent",
    "july_additional_training_samples": 8000,
}

# THERE IS NO MINIMUM-LENGTH GATE. July's code has none: its 119/118/119 prefixes are descriptive
# outcomes, not a threshold. The literal July fail-closed condition is truncation at the first
# discrete mismatch plus a recovery corpus that is NON-EMPTY and time-aligned. A short but strictly
# aligned prefix may produce a QUARANTINED candidate whose quality only the closed-loop can judge.
CORPUS_CONDITION = {
    "rule": "non-empty AND time-aligned, after July-strict truncation",
    "minimum_length_gate": None,
    "note": "July's retained prefixes (119, 118, 119) are DESCRIPTIVE and are never used as a "
            "threshold; the candidate this stage can produce is quarantined by construction",
}


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

def verify_inputs() -> dict[str, Any]:
    """Every operational input, by exact hash. A mutated or missing one aborts."""
    got: dict[str, str] = {}
    for rel, pin in PIN_INPUTS.items():
        path = HERE / rel
        if not path.is_file():
            raise J4Error(f"the pinned input {rel} is missing")
        h = _sha_file(path)
        if h != pin:
            raise J4Error(f"the pinned input {rel} changed: {h} != {pin}")
        got[rel] = h
    j2 = J3.verify_j2_actor()          # the six J2 artefacts, by their own pins
    if j2["artefacts_sha256"] != PIN_J2_MODULE:
        raise J4Error("the J2 module no longer matches its pins")
    if not AMENDMENT.is_file():
        raise J4Error("the recovery-only amendment is missing")
    amendment = _sha_file(AMENDMENT)
    if amendment != PIN_AMENDMENT:
        raise J4Error(f"the recovery-only amendment changed: {amendment} != {PIN_AMENDMENT}")

    # THE AMENDMENT IS THE PINNED MANIFEST. Every artefact it lists is verified HERE, at runtime -
    # including v26c_j3_closed_loop.py, the runner that produced the recovery trace. Relying on a
    # test to check the manifest would leave the execution path unguarded.
    manifest = json.loads(AMENDMENT.read_text())["pinned_artefacts_sha256"]
    if not manifest:
        raise J4Error("the amendment declares no pinned artefacts")
    manifest_checked: dict[str, str] = {}
    for rel, pin in manifest.items():
        path = HERE / rel
        if not path.is_file():
            raise J4Error(f"the amendment pins {rel}, which is missing")
        h = _sha_file(path)
        if h != pin:
            raise J4Error(f"the amendment-pinned artefact {rel} changed: {h} != {pin}")
        manifest_checked[rel] = h
    # the two pin sets must agree wherever they overlap
    for rel, pin in PIN_INPUTS.items():
        if rel in manifest_checked and manifest_checked[rel] != pin:
            raise J4Error(f"the amendment and the runner disagree on {rel}")
    for rel, pin in PIN_J2_MODULE.items():
        full = f"j2_runs/j2_base_v26c_2026-08-26_r1/{rel}"
        if full in manifest_checked and manifest_checked[full] != pin:
            raise J4Error(f"the amendment and the J2 pins disagree on {full}")
    j3r = json.loads((J3_LEAF / "v26c_j3_closed_loop_receipt.json").read_text())
    if j3r["verdict"] != "FAIL" or j3r["gate"]["failed"] != ["max_penetration_m"]:
        raise J4Error("the J3 receipt no longer records its original FAIL")
    if j3r["deployable"] is not False or j3r["promotion"] != "NONE":
        raise J4Error("the J3 receipt claims deployability or promotion")
    j1r = json.loads((J1_LEAF / "v26c_j1_collection_receipt.json").read_text())
    if j1r["verdict"] != "FAIL":
        raise J4Error("the J1 receipt no longer records its original FAIL")
    return {
        "inputs_sha256": got,
        "j2_module_sha256": dict(PIN_J2_MODULE),
        "amendment_sha256": amendment,
        "amendment_manifest_sha256": manifest_checked,
        "amendment_manifest_entries": len(manifest_checked),
        "j3_verdict_preserved": j3r["verdict"],
        "j3_failed": j3r["gate"]["failed"],
        "j3_max_penetration_m": j3r["summary"]["max_penetration_m"],
        "j1_verdict_preserved": j1r["verdict"],
        "statement": "the J1 and J3 FAIL verdicts STAND. This stage neither re-scores nor "
                     "retroactively promotes them, whatever it produces.",
    }


def feature_names() -> tuple[str, ...]:
    manifest = json.loads((J2_MODULE_DIR / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J4Error(f"the pinned manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


def discrete_feature_indices(names: Sequence[str]) -> np.ndarray:
    """July's own selector, transcribed: _discrete_feature_indices."""
    return np.asarray(
        [i for i, n in enumerate(names)
         if str(n).endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
         or str(n).startswith(("phase_fsm_", "phase_expected_"))],
        dtype=int)


def trainable_controller_features(names: Sequence[str]) -> tuple[str, ...]:
    """July's markov_feature_names, transcribed: exactly the 10 deployable controller features."""
    selected = tuple(str(n) for n in names
                     if str(n).endswith(MARKOV_CONTROLLER_FEATURE_SUFFIXES)
                     and not str(n).endswith(("_sea_u_abs", "_sea_u_saturated")))
    if len(selected) != 10:
        raise J4Error(f"expected 10 deployable controller features, found {len(selected)}")
    indices = tuple(names.index(n) for n in selected)
    if indices != CONTROLLER_COLUMNS:
        raise J4Error(f"the controller features sit at columns {indices}, "
                      f"expected {CONTROLLER_COLUMNS}")
    if sorted(selected) != sorted(MARKOV_CONTROLLER_FEATURE_SCALES):
        raise J4Error("the trainable controller features and the July scales disagree")
    return selected


def clock_indices(names: Sequence[str]) -> tuple[int, ...]:
    idx = tuple(names.index(n) for n in DISABLED_GAIT_CLOCK_FEATURES if n in names)
    if idx != CLOCK_COLUMNS:
        raise J4Error(f"the clock features sit at columns {idx}, expected {CLOCK_COLUMNS}")
    return idx


def _obs_matrix(rows: Sequence[Mapping[str, Any]], width: int, label: str) -> np.ndarray:
    """Contiguity, width and finiteness, exactly as July's _trace_arrays checks them."""
    out = np.empty((len(rows), width), dtype=np.float32)
    for expected, row in enumerate(rows, start=1):
        step = int(row.get("step", expected))
        if step != expected:
            raise J4Error(f"{label} is not contiguous at step {step}")
        v = np.asarray(row["actor_observation_vector_before"], dtype=np.float32).reshape(-1)
        if v.shape != (width,):
            raise J4Error(f"{label} step {step} actor width {v.shape} != {(width,)}")
        if not np.all(np.isfinite(v)):
            raise J4Error(f"{label} step {step} observation is non-finite")
        out[expected - 1] = v
    if not len(rows):
        raise J4Error(f"{label} is empty")
    return out


def truncate_before_discrete_mismatch(nominal_rows: Sequence[Mapping[str, Any]],
                                      recovery_rows: Sequence[Mapping[str, Any]],
                                      names: Sequence[str]) -> tuple[list[Any], dict[str, Any]]:
    """July's rule, transcribed. Keep only rows whose deployable event/FSM state still agrees."""
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
        "original_steps": len(recovery_rows),
        "retained_steps": limit,
        "first_discrete_mismatch_step": first_mismatch,
        "mismatching_columns": columns,
        "rule": "July-strict; never relaxed",
        "source": JULY_SOURCES["truncation"],
    }


def discrete_agreement_diagnostics(nominal_rows: Sequence[Mapping[str, Any]],
                                   recovery_rows: Sequence[Mapping[str, Any]],
                                   names: Sequence[str]) -> dict[str, Any]:
    """DIAGNOSTIC ONLY. Never used to extend the prefix or to relax the truncation rule."""
    disc = discrete_feature_indices(names)
    n = min(len(nominal_rows), len(recovery_rows))
    A = _obs_matrix(list(nominal_rows)[:n], len(names), "nominal trace")
    B = _obs_matrix(list(recovery_rows)[:n], len(names), "recovery trace")
    agree = np.all(A[:, disc] == B[:, disc], axis=1)
    best = cur = start = best_start = 0
    for i, ok in enumerate(agree):
        if ok:
            if cur == 0:
                start = i
            cur += 1
            if cur > best:
                best, best_start = cur, start
        else:
            cur = 0
    per_column = {}
    for j in disc:
        mism = np.flatnonzero(A[:, j] != B[:, j])
        per_column[str(names[j])] = {
            "first_mismatch_step": (int(mism[0]) + 1) if mism.size else None,
            "mismatching_steps": int(mism.size)}
    return {
        "compared_steps": int(n),
        "steps_agreeing_on_all_discrete_columns": int(agree.sum()),
        "longest_contiguous_agreeing_run": {
            "steps": int(best),
            "from": int(best_start) + 1 if best else None,
            "to": int(best_start + best) if best else None},
        "per_column": per_column,
        "status": "DIAGNOSTIC, NOT A RELAXATION: the July-strict prefix is the only corpus the "
                  "fit may use. These numbers exist so the architect can decide, not so the "
                  "runner can widen the corpus on its own.",
    }


def load_parent_state() -> dict[str, np.ndarray]:
    import pickle
    with (J2_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        return {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}


def july_forward(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    """A bit-exact transcription of target_domain_noise_adaptation._forward.

    NATIVE PRECISION: the observations arrive float32 and the pinned weights are float32, so the
    whole forward is float32 - exactly July's semantics. It is NOT promoted to float64: that would
    change the labels this stage distils. The platform's vectorised matmul raises spurious FPE
    flags on padding lanes, so the flags are suppressed and replaced by an EXPLICIT finiteness
    check, which is the real guard. Bit-equality with the July function is asserted in the tests.
    """
    def array(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value)

    with np.errstate(all="ignore"):
        hidden = np.tanh(observations @ array(state["pi.0.0.weight"]).T
                         + array(state["pi.0.0.bias"]))
        hidden = np.tanh(hidden @ array(state["pi.0.2.weight"]).T
                         + array(state["pi.0.2.bias"]))
        logits = hidden @ array(state["pi.1.weight"]).T + array(state["pi.1.bias"])
    if not np.all(np.isfinite(logits)):
        raise J4Error("the student's own logits are not finite")
    return logits


def student_mean_actions(observations: np.ndarray,
                         state: Mapping[str, np.ndarray] | None = None) -> np.ndarray:
    """The pinned J2 student's OWN mean actions, as July's _actor_means computes them.

    July: `_forward(state, np.asarray(observations, np.float32))[:, :2]` returned as float. The
    forward is float32 native; only the RETURNED slice is widened, exactly as July does.
    """
    st = dict(state) if state is not None else load_parent_state()
    x = np.asarray(observations, dtype=np.float32)
    if x.ndim != 2 or x.shape[1] != ACTOR_WIDTH:
        raise J4Error(f"student_mean_actions expects [n, {ACTOR_WIDTH}], got {x.shape}")
    logits = july_forward(st, x)
    means = np.asarray(logits[:, :2], dtype=float)
    if not np.all(np.isfinite(means)):
        raise J4Error("the student's own mean actions are not finite")
    return means


def build_recovery_dataset(*, nominal_repeat: int = NOMINAL_REPEAT,
                           recovery_repeat: int = RECOVERY_REPEAT) -> dict[str, Any]:
    """The SELECTED July Markov composition, on the pinned V26C artefacts.

    Nominal anchors: the 500 nominal teacher STATES, labelled with the J2 student's OWN means
                     (self-distilled), repeated nominal_repeat times and placed FIRST.
    Recovery pairs:  the J3 visited states of the July-strict aligned prefix, labelled with the
                     SAME-STEP prescribed teacher action, repeated recovery_repeat times.
    Multistart:      OMITTED - the single explicit derogation, deferred by user decision.
    """
    if nominal_repeat < 1 or recovery_repeat < 1:
        raise J4Error("nominal_repeat and recovery_repeat must be >= 1")
    names = feature_names()
    with np.load(J1_LEAF / "teacher_dataset.npz", allow_pickle=False) as archive:
        teacher_obs = np.asarray(archive["observations"], dtype=np.float32)
        teacher_act = np.asarray(archive["actions"], dtype=np.float32)
        teacher_times = np.asarray(archive["times"], dtype=np.float64)
        dataset_names = tuple(str(n) for n in np.asarray(archive["actor_feature_names"]).tolist())
    if dataset_names != names:
        raise J4Error("the J1 dataset actor schema differs from the pinned J2 manifest")
    nominal_rows = json.loads((J1_LEAF / "teacher_trace.json").read_text())
    recovery_rows = json.loads((J3_LEAF / "j3_trace.json").read_text())
    if len(teacher_obs) != len(nominal_rows):
        raise J4Error("the J1 dataset and its trace disagree on length")

    # the trace observations must BE the dataset observations, row for row
    trace_obs = _obs_matrix(nominal_rows, len(names), "nominal trace")
    if not np.array_equal(trace_obs, teacher_obs):
        raise J4Error("the J1 trace observations differ from the J1 dataset observations")

    kept, filter_report = truncate_before_discrete_mismatch(nominal_rows, recovery_rows, names)
    diagnostics = discrete_agreement_diagnostics(nominal_rows, recovery_rows, names)
    retained = len(kept)
    if retained == 0:
        raise J4Error("the recovery trace has no phase-aligned rows before its first discrete "
                      "mismatch")
    visited = _obs_matrix(kept, len(names), "recovery trace")
    # TIME ALIGNMENT: row k carries the teacher action of the SAME fixed-start episode step.
    labels = teacher_act[:retained].copy()
    label_times = teacher_times[:retained].copy()
    for k, row in enumerate(kept):
        if int(row["step"]) != k + 1:
            raise J4Error(f"recovery row {k + 1} is not the step it claims")
        t_recovery = float(row["time_before"])
        t_nominal = float(nominal_rows[k]["time_before"])
        if abs(t_recovery - t_nominal) > 1e-9:
            raise J4Error(f"step {k + 1}: recovery time {t_recovery} != nominal {t_nominal}; the "
                          "fixed-start alignment the labels depend on does not hold")
        if abs(float(label_times[k]) - t_nominal) > 1e-9:
            raise J4Error(f"step {k + 1}: dataset time {float(label_times[k])} != trace time "
                          f"{t_nominal}")

    # The nominal block is SELF-DISTILLED: the states are the teacher's, the labels are the
    # student's own means. This is what July anchors with, and it is what keeps the recovery
    # update from moving the behaviour the base stage already produces on-distribution.
    clock_for_means = list(clock_indices(names))
    nominal_states = teacher_obs.copy()
    nominal_states[:, clock_for_means] = 0.0
    # July returns the means as float; the values were produced in float32, so storing them back
    # as float32 is an exact round trip and keeps the dataset in the collector's dtype.
    anchor_labels = np.asarray(student_mean_actions(nominal_states), dtype=np.float32)
    observations = np.concatenate([np.tile(nominal_states, (nominal_repeat, 1)),
                                   np.tile(visited, (recovery_repeat, 1))], axis=0)
    actions = np.concatenate([np.tile(anchor_labels, (nominal_repeat, 1)),
                              np.tile(labels, (recovery_repeat, 1))], axis=0)
    times = np.concatenate([np.tile(teacher_times, nominal_repeat),
                            np.tile(label_times, recovery_repeat)], axis=0)
    if not (np.all(np.isfinite(observations)) and np.all(np.isfinite(actions))):
        raise J4Error("the aggregate contains non-finite values")

    # LEAKAGE GUARD, two-sided.
    # The runtime leaves the disabled clock as a CONSTANT (sin = 0.0, cos = 1.0), so the raw data
    # is not zero by itself. The clock is masked in the WEIGHTS at every optimiser step, and the
    # data columns are additionally projected to exact zero here, exactly as the base stage did:
    # a masked feature must be unable to carry signal by either route.
    clk = list(clock_indices(names))
    raw_clock_abs_max = float(np.max(np.abs(observations[:, clk])))
    observations[:, clk] = 0.0
    if float(np.max(np.abs(observations[:, clk]))) != 0.0:
        raise J4Error("the clock columns could not be projected to zero")
    # The controller-memory columns must NOT be masked here: they are what this stage learns from.
    controller = trainable_controller_features(names)
    ctrl_idx = [names.index(n) for n in controller]
    ctrl_abs_max = {n: float(np.max(np.abs(observations[:, names.index(n)]))) for n in controller}
    degenerate = [n for n, v in ctrl_abs_max.items() if v == 0.0]
    if degenerate:
        raise J4Error("the controller-memory columns carry no signal in the collected data "
                      f"({degenerate}); a recovery stage that trains them would learn nothing")
    if float(np.max(np.abs(observations[:, ctrl_idx]))) == 0.0:
        raise J4Error("the whole controller-memory block is zero in the aggregate")
    report = {
        "schema": {"observations": list(observations.shape), "actions": list(actions.shape),
                   "times": list(times.shape), "actor_feature_names": len(names),
                   "dtype": "float32 / float32 / float64"},
        "nominal_states": int(len(teacher_obs)),
        "nominal_repeat": int(nominal_repeat),
        "nominal_training_samples": int(len(teacher_obs) * nominal_repeat),
        "nominal_label_source": "SELF-DISTILLED: the pinned J2 student's own mean actions",
        "nominal_label_rms": float(np.sqrt(np.mean(anchor_labels ** 2))),
        "recovery_steps": int(retained),
        "recovery_repeat": int(recovery_repeat),
        "recovery_training_samples": int(retained * recovery_repeat),
        "aggregate_samples": int(len(observations)),
        "recovery_fraction_of_aggregate": float(retained * recovery_repeat / len(observations)),
        "multistart_derogation": dict(MULTISTART_DEROGATION),
        "corpus_condition": dict(CORPUS_CONDITION),
        "label_rule": "recovery labels are teacher_actions[step - 1], the SAME fixed-start "
                      "episode step; nominal labels are the student's own means",
        "time_alignment_verified": {"tolerance_s": 1e-9, "rows_checked": int(retained),
                                    "sources": ["trace time_before", "dataset times"]},
        "trace_filter": filter_report,
        "discrete_agreement_diagnostics": diagnostics,
        "leakage_guard": {
            "clock_columns": clk,
            "clock_raw_abs_max_before_projection": raw_clock_abs_max,
            "clock_projected_to_exact_zero_in_data": True,
            "clock_masked_in_weights_every_step": True,
            "note": "the runtime leaves the disabled clock as a constant (sin 0.0, cos 1.0), so "
                    "the raw data alone is not zero; both routes are closed",
            "controller_columns_left_intact": ctrl_idx,
            "controller_abs_max_by_feature": ctrl_abs_max,
        },
        "trainable_controller_features": list(controller),
        "aggregation_source": JULY_SOURCES["aggregation"],
        "july_composition_for_comparison": dict(JULY_RECOVERY_COMPOSITION),
    }
    return {"observations": observations, "actions": actions, "times": times,
            "actor_feature_names": np.asarray(names, dtype=str), "report": report}


# ================================================================ preflight (INERT) ==============

def preflight(*, nominal_repeat: int = NOMINAL_REPEAT,
              recovery_repeat: int = RECOVERY_REPEAT) -> dict[str, Any]:
    """Fail-closed. Builds the dataset IN MEMORY, constructs nothing, trains nothing, writes nothing."""
    inputs = verify_inputs()
    names = feature_names()
    controller = trainable_controller_features(names)
    clk = list(clock_indices(names))
    blockers: list[str] = []
    try:
        built = build_recovery_dataset(nominal_repeat=nominal_repeat,
                                       recovery_repeat=recovery_repeat)
        report = built["report"]
        dataset_sha = _sha_obj({"observations": built["observations"].tolist(),
                                "actions": built["actions"].tolist()})
    except J4Error as exc:
        report = {"error": str(exc)}
        dataset_sha = None
        blockers.append(f"the recovery dataset cannot be built: {exc}")

    # The LITERAL July condition: non-empty and aligned. No minimum-length gate exists in July and
    # none is introduced. A short aligned prefix yields a QUARANTINED candidate, judged only by the
    # subsequent closed-loop.
    retained = report.get("recovery_steps")
    if retained is not None and retained < 1:
        blockers.append("the recovery corpus is EMPTY after July-strict truncation")

    # the parent must still be the masked base student: clock AND controller columns zero
    state = J2.load_state(J2_MODULE_DIR / "module_state.pkl") if hasattr(J2, "load_state") else None
    if state is None:
        import pickle
        with (J2_MODULE_DIR / "module_state.pkl").open("rb") as fh:
            state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    layers = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and np.ndim(v) == 2
                    and np.shape(v)[1] == ACTOR_WIDTH)
    parent_zero: dict[str, list[int]] = {}
    for key in layers:
        W = np.asarray(state[key])
        parent_zero[key] = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if parent_zero[key] != list(J3.MASKED_COLUMNS):
            raise J4Error(f"the J2 parent layer {key} has zero columns {parent_zero[key]}, "
                          f"expected {list(J3.MASKED_COLUMNS)}")
    # July requires a zero source column for every scaled feature
    unscalable = [n for n in controller
                  if not bool(np.all(np.asarray(state[layers[0]])[:, names.index(n)] == 0.0))]
    if unscalable:
        blockers.append(f"feature scaling requires zero source columns; non-zero: {unscalable}")

    return {
        "verdict": "GO" if not blockers else "BLOCKED",
        "stage": STAGE,
        "blockers": blockers,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "training_executed": False,
                  "files_written": False,
                  "note": "torch is imported inside fit() only; the preflight builds the dataset "
                          "in memory and discards it"},
        "authorisation": {
            "authorised_by": "THE USER, explicitly. The architect is reviewer, not approver.",
            "amendment": _rel(AMENDMENT), "amendment_sha256": inputs["amendment_sha256"],
            "scope": "RECOVERY ONLY - ONE J4 fit",
            "execution_requires": f"--authorized-stage {STAGE} --out-dir <fresh leaf>",
            "physical_rollout_authorised": False,
            "progressive_rounds_authorised": False,
            "j5_authorised": False,
            "revalidation_required_before_any_claim": True,
            "soft_fail_accepted_by_user": {
                "statement": "the J1/J3 penetration soft fail (above 20 mm, below the 28 mm hard "
                             "guard) is accepted by the user so the recovery-only stage can "
                             "proceed. The old nominal gate is NOT reintroduced as a blocker here.",
                "hard_guard_m": 0.028, "steps_above_hard_guard_j3": 0,
            },
            "multistart_derogation": dict(MULTISTART_DEROGATION),
        },
        "lineage_preserved": {k: inputs[k] for k in
                              ("j3_verdict_preserved", "j3_failed", "j3_max_penetration_m",
                               "j1_verdict_preserved", "statement")},
        "inputs_sha256": inputs["inputs_sha256"],
        "j2_parent": {"module": _rel(J2_MODULE_DIR), "sha256": inputs["j2_module_sha256"],
                      "zero_columns_by_layer": parent_zero,
                      "role": "warm start; the SAME single 35D student, not a new actor"},
        "actor_contract": {
            "width": ACTOR_WIDTH,
            "hard_zero_columns_this_stage": clk,
            "trainable_controller_columns": list(CONTROLLER_COLUMNS),
            "trainable_controller_features": list(controller),
            "feature_scales": dict(MARKOV_CONTROLLER_FEATURE_SCALES),
            "widening": "NONE - the actor stays 35D",
            "standalone_25d": "NONE",
            "contralateral_features": "ABSENT - controller memory is the prosthesis' own "
                                      "deployable internal state, not a contralateral signal",
            "scale_absorption": JULY_SOURCES["scale_absorption"],
            "clock_rule": JULY_SOURCES["clock_hard_zero"],
        },
        "dataset": report,
        "dataset_sha256": dataset_sha,
        "corpus_condition": dict(CORPUS_CONDITION),
        "composition": {"nominal_repeat": nominal_repeat,
                        "recovery_repeat": recovery_repeat,
                        "source": "the SELECTED final July Markov run, used coherently; never mixed with the 11 July DAgger 1x/1x composition"},
        "hyperparameters": dict(JULY_RECOVERY_HP),
        "hyperparameter_source": JULY_SOURCES["hyperparameters"],
        "july_reference_outcome": dict(JULY_RECOVERY_REFERENCE),
        "progressive_iterations": {
            "mechanism": "fit on the aligned prefix -> FRESH deterministic revalidation rollout -> "
                         "recompute the aligned prefix -> aggregate the longer prefix",
            "july_evidence": JULY_SOURCES["iteration_growth"],
            "physical_execution_authorised": False,
        },
        "no_clobber": {"scope": "PER LEAF", "root": _rel(OUT_ROOT),
                       "existing_leaves": sorted(p.name for p in OUT_ROOT.iterdir())
                       if OUT_ROOT.is_dir() else []},
        "forbidden_here": ["retry of J3", "sigma", "critic", "PPO update", "ex-novo", "promotion",
                           "LOTO", "LOCO", "B1R1", "B1R2", "standalone 25D", "widening",
                           "contralateral features", "July artefacts as operational inputs"],
        "generated_at_utc": _utc(),
    }


# ================================================================ the recovery fit ===============

def fit(*, authorized_stage: str | None, out_dir: Path | None,
        nominal_repeat: int = NOMINAL_REPEAT, recovery_repeat: int = RECOVERY_REPEAT,
        progress: bool = True) -> dict[str, Any]:
    """ONE July-faithful recovery update of the pinned J2 student. Requires the exact token."""
    if authorized_stage != STAGE:
        raise J4Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    if out_dir is None:
        raise J4Error("--out-dir is required: the J4 leaf must be named explicitly")
    # ONLY the authorised composition may be written. The builder may be exercised with other
    # repeats for testing; the WRITE path refuses every override.
    if (nominal_repeat, recovery_repeat) != (NOMINAL_REPEAT, RECOVERY_REPEAT):
        raise J4Error(
            f"the authorised composition is nominal_repeat={NOMINAL_REPEAT} and "
            f"recovery_repeat={RECOVERY_REPEAT}; got {nominal_repeat}/{recovery_repeat}. "
            "No override may be written.")
    pre = preflight(nominal_repeat=nominal_repeat, recovery_repeat=recovery_repeat)
    if pre["blockers"]:
        raise J4Error(f"preflight BLOCKED: {pre['blockers']}")
    out = Path(out_dir)
    if out.exists():
        raise J4Error(f"no-clobber: the leaf {out} already exists; choose a fresh --out-dir")

    names = feature_names()
    controller = trainable_controller_features(names)
    clk = list(clock_indices(names))
    built = build_recovery_dataset(nominal_repeat=nominal_repeat,
                                   recovery_repeat=recovery_repeat)
    obs = np.asarray(built["observations"], dtype=np.float32)
    act = np.asarray(built["actions"], dtype=np.float32)

    import pickle
    with (J2_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        parent = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}

    import torch                                   # heavy import, fit only
    from torch.nn import functional
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(int(JULY_RECOVERY_HP["seed"]))
    np.random.seed(int(JULY_RECOVERY_HP["seed"]))

    # July: ONE default_rng(seed); the split is drawn first and the SAME generator shuffles epochs.
    rng = np.random.default_rng(int(JULY_RECOVERY_HP["seed"]))
    rows = len(obs)
    perm = rng.permutation(rows)
    n_val = int(round(rows * float(JULY_RECOVERY_HP["validation_fraction"])))
    val_idx = np.sort(perm[:n_val])
    train_idx = np.sort(perm[n_val:])

    # July: train on obs / scale, then fold the scale into the first layer before saving.
    scales = np.ones(len(names), dtype=np.float32)
    for name, scale in MARKOV_CONTROLLER_FEATURE_SCALES.items():
        scales[names.index(name)] = np.float32(scale)
    X = torch.as_tensor(obs / scales)
    Y = torch.as_tensor(act)

    P = lambda k: torch.nn.Parameter(torch.as_tensor(np.array(parent[k])))   # noqa: E731
    keys = ["pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias",
            "pi.1.weight", "pi.1.bias"]
    params = {k: P(k) for k in keys}
    anchor = {k: params[k].detach().clone() for k in keys}
    action_dim = act.shape[1]
    logstd_w = params["pi.1.weight"].detach()[action_dim:].clone()
    logstd_b = params["pi.1.bias"].detach()[action_dim:].clone()
    if bool(torch.any(logstd_w != 0.0)):
        raise J4Error("freeze_logstd_head requires zero log-std output weights")

    def forward(x: Any) -> Any:
        h = torch.tanh(functional.linear(x, params["pi.0.0.weight"], params["pi.0.0.bias"]))
        h = torch.tanh(functional.linear(h, params["pi.0.2.weight"], params["pi.0.2.bias"]))
        return functional.linear(h, params["pi.1.weight"], params["pi.1.bias"])

    def project() -> None:
        with torch.no_grad():
            params["pi.1.weight"][action_dim:].copy_(logstd_w)
            params["pi.1.bias"][action_dim:].copy_(logstd_b)
            params["pi.0.0.weight"][:, clk].zero_()          # clock stays hard-zero, every step

    project()
    with torch.no_grad():
        source_logstd = forward(X)[:, action_dim:].clone()
        initial = forward(torch.as_tensor(obs))[:, :action_dim].numpy()

    optimiser = torch.optim.Adam(list(params.values()),
                                 lr=float(JULY_RECOVERY_HP["learning_rate"]))
    best = {"epoch": 0, "val": float("inf"), "state": {k: v.detach().clone() for k, v in
                                                       params.items()}}
    stale = 0
    history: list[dict[str, float]] = []
    batch = int(JULY_RECOVERY_HP["batch_size"])
    for epoch in range(1, int(JULY_RECOVERY_HP["epochs"]) + 1):
        shuffled = rng.permutation(train_idx)
        losses: list[float] = []
        for start in range(0, len(shuffled), batch):
            idx = shuffled[start:start + batch]
            logits = forward(X[idx])
            means = logits[:, :action_dim]
            mean_loss = functional.mse_loss(means, Y[idx])
            clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
            logstd_loss = functional.mse_loss(logits[:, action_dim:], source_logstd[idx])
            anchor_loss = torch.stack([(params[k] - anchor[k]).square().mean()
                                       for k in keys]).mean()
            loss = (mean_loss
                    + float(JULY_RECOVERY_HP["clip_weight"]) * clip_loss
                    + float(JULY_RECOVERY_HP["logstd_weight"]) * logstd_loss
                    + float(JULY_RECOVERY_HP["anchor_weight"]) * anchor_loss)
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            optimiser.step()
            project()
            losses.append(float(loss.item()))
        with torch.no_grad():
            val = float(functional.mse_loss(forward(X[val_idx])[:, :action_dim],
                                            Y[val_idx]).item())
        history.append({"epoch": float(epoch), "train_loss": float(np.mean(losses)),
                        "validation_mse": val})
        if val < best["val"] - 1e-9:
            best = {"epoch": epoch, "val": val,
                    "state": {k: v.detach().clone() for k, v in params.items()}}
            stale = 0
        else:
            stale += 1
        if progress and (epoch == 1 or epoch % 25 == 0):
            print(json.dumps({"epoch": epoch, "train": history[-1]["train_loss"],
                              "val": val}), flush=True)
        if stale >= int(JULY_RECOVERY_HP["patience"]):
            break

    with torch.no_grad():
        for k in keys:
            params[k].copy_(best["state"][k])
        project()
        # absorb the physical scales into the first layer: the deployed actor takes RAW inputs
        for name, scale in MARKOV_CONTROLLER_FEATURE_SCALES.items():
            params["pi.0.0.weight"][:, names.index(name)].div_(float(scale))
        params["pi.0.0.weight"][:, clk].zero_()
        adapted = forward(torch.as_tensor(obs))[:, :action_dim].numpy()

    final = {k: params[k].detach().numpy().astype(np.float32) for k in keys}
    final["pi_encoder.0.weight"] = final["pi.0.0.weight"].copy()
    final["pi_encoder.0.bias"] = final["pi.0.0.bias"].copy()
    final["pi_encoder.2.weight"] = final["pi.0.2.weight"].copy()
    final["pi_encoder.2.bias"] = final["pi.0.2.bias"].copy()

    # ---------------------------------------------------------------- pre-receipt verification --
    # Every one of these is computed and CHECKED before a single byte is written. A failure aborts
    # the stage with the leaf never created.
    n_nominal = int(built["report"]["nominal_states"])          # 500 unique nominal states
    n_recovery = int(built["report"]["recovery_steps"])          # 12 unique recovery states
    nominal_states = obs[:n_nominal]
    anchor_targets = act[:n_nominal]
    recovery_states = obs[-n_recovery:]
    recovery_targets = act[-n_recovery:]

    def rmse(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.sqrt(np.mean((np.asarray(a, dtype=np.float64)
                                      - np.asarray(b, dtype=np.float64)) ** 2)))

    parent_nominal = np.asarray(student_mean_actions(nominal_states, parent), dtype=np.float32)
    final_nominal = np.asarray(student_mean_actions(nominal_states, final), dtype=np.float32)
    final_recovery = np.asarray(student_mean_actions(recovery_states, final), dtype=np.float32)
    parent_recovery = np.asarray(student_mean_actions(recovery_states, parent), dtype=np.float32)
    shift = final_nominal.astype(np.float64) - parent_nominal.astype(np.float64)

    controller_norms = {n: float(np.linalg.norm(final["pi.0.0.weight"][:, names.index(n)]))
                        for n in controller}
    dead = sorted(n for n, v in controller_norms.items() if not v > 0.0)
    if dead:
        raise J4Error(f"controller-memory columns stayed at zero norm: {dead}")

    zero_after = [c for c in range(ACTOR_WIDTH)
                  if bool(np.all(final["pi.0.0.weight"][:, c] == 0.0))]
    if zero_after != clk:
        raise J4Error(f"after the fit the zero columns are {zero_after}, expected {clk}")
    for alias in ("pi.0.0.weight", "pi_encoder.0.weight"):
        if not bool(np.all(final[alias][:, clk] == 0.0)):
            raise J4Error(f"the clock columns are not bit-zero in {alias}")
    for direct, encoder in (("pi.0.0.weight", "pi_encoder.0.weight"),
                            ("pi.0.0.bias", "pi_encoder.0.bias"),
                            ("pi.0.2.weight", "pi_encoder.2.weight"),
                            ("pi.0.2.bias", "pi_encoder.2.bias")):
        if not bool(np.array_equal(final[direct], final[encoder])):
            raise J4Error(f"the aliases {direct} and {encoder} are not bit-identical")
    # head_rows, NOT rows: `rows` is the aggregate row count used far below when the receipt is
    # assembled. Shadowing it with a slice is what aborted the single J4 execution.
    for key, head_rows in (("pi.1.weight", slice(action_dim, None)),
                           ("pi.1.bias", slice(action_dim, None))):
        if not bool(np.array_equal(final[key][head_rows], np.asarray(parent[key])[head_rows])):
            raise J4Error(f"the frozen log-std head changed in {key}")

    # Every weight array that would be written must be finite. A non-finite entry aborts here,
    # with the leaf never created.
    for key, value in sorted(final.items()):
        if not bool(np.all(np.isfinite(np.asarray(value)))):
            raise J4Error(f"the fitted parameter {key} contains non-finite values")

    recovery_rmse_before = rmse(parent_recovery, recovery_targets)
    recovery_rmse_after = rmse(final_recovery, recovery_targets)
    nominal_rmse_before = rmse(parent_nominal, anchor_targets)
    nominal_rmse_after = rmse(final_nominal, anchor_targets)
    shift_rms = float(np.sqrt(np.mean(shift ** 2)))
    shift_max_abs = float(np.max(np.abs(shift)))
    shift_per_action = [float(v) for v in np.sqrt(np.mean(shift ** 2, axis=0))]

    # Every reported metric must be finite before it can be believed or written.
    for label, value in (("nominal_rmse_before", nominal_rmse_before),
                         ("nominal_rmse_after", nominal_rmse_after),
                         ("recovery_rmse_before", recovery_rmse_before),
                         ("recovery_rmse_after", recovery_rmse_after),
                         ("nominal_shift_rms", shift_rms),
                         ("nominal_shift_max_abs", shift_max_abs),
                         *((f"nominal_shift_per_action[{i}]", v)
                           for i, v in enumerate(shift_per_action)),
                         *((f"controller_norm[{n}]", v)
                           for n, v in sorted(controller_norms.items()))):
        if not math.isfinite(value):
            raise J4Error(f"the metric {label} is not finite: {value!r}")

    # A RECOVERY-ONLY technical PASS requires the recovery error to actually fall. This is a
    # direction, not a numeric threshold: no magnitude is prescribed and the July figures are
    # never used as a gate.
    if not (recovery_rmse_after < recovery_rmse_before):
        raise J4Error(
            "the recovery error did not improve: "
            f"rmse_after {recovery_rmse_after!r} is not below rmse_before "
            f"{recovery_rmse_before!r}. A recovery-only stage that does not reduce the error on "
            "the states it was built from has nothing to record; aborting before any write.")

    verification = {
        "nominal_self_distillation": {
            "rmse_before": nominal_rmse_before,
            "rmse_after": nominal_rmse_after,
            "note": "the anchors ARE the parent's own means, so rmse_before is 0 by construction "
                    "and rmse_after measures how far the update moved the nominal behaviour"},
        "nominal_shift_vs_parent": {
            "rms": shift_rms,
            "max_abs": shift_max_abs,
            "per_action_rms": shift_per_action,
            "july_reference": {"rms": 0.004175083023500488, "max_abs": 0.03159449528902769,
                               "run": JULY_RECOVERY_REFERENCE["run"],
                               "status": "REFERENCE ONLY, never a gate"}},
        "recovery_vs_teacher": {
            "rmse_before": recovery_rmse_before,
            "rmse_after": recovery_rmse_after,
            "improved": True,
            "requirement": "rmse_after < rmse_before, enforced before any write. A direction, "
                           "not a magnitude: no numeric threshold is prescribed"},
        "all_parameters_finite": True,
        "all_metrics_finite": True,
        "controller_column_norms": controller_norms,
        "controller_columns_all_positive": True,
        "clock_bit_zero_in_both_aliases": {a: clk for a in ("pi.0.0.weight",
                                                            "pi_encoder.0.weight")},
        "aliases_bit_identical": True,
        "logstd_head_bit_identical_to_parent": True,
    }

    out.mkdir(parents=True, exist_ok=False)
    module_dir = out / "rl_module"
    module_dir.mkdir()
    shutil.copy2(J2_MODULE_DIR / "class_and_ctor_args.pkl", module_dir)
    shutil.copy2(J2_MODULE_DIR / "metadata.json", module_dir)
    with (module_dir / "module_state.pkl").open("wb") as fh:
        pickle.dump(final, fh, protocol=4)
    manifest = json.loads((J2_MODULE_DIR / "actor_feature_manifest.json").read_text())
    manifest.update({
        "derived_from": _rel(J2_MODULE_DIR),
        "source_module_state_sha256": PIN_J2_MODULE["rl_module/module_state.pkl"],
        "module_state_sha256": _sha_file(module_dir / "module_state.pkl"),
        "controller_state_mask": {"active": False, "columns": list(CONTROLLER_COLUMNS),
                                  "status": "TRAINABLE at the recovery stage"},
        "clock_columns": clk,
        "actor_label": "J4_RECOVERY35_JULY_FAITHFUL",
        "deployable": False, "sigma_unresolved": True,
        "status": "recovery-stage fit; never promoted; revalidation not run",
    })
    (module_dir / "actor_feature_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(out / "recovery_dataset.npz", observations=obs, actions=act,
                        times=built["times"], actor_feature_names=built["actor_feature_names"])
    (out / "history.json").write_text(json.dumps(history, indent=1, allow_nan=False),
                                      encoding="utf-8")
    rmse = float(np.sqrt(np.mean((adapted - act) ** 2)))
    receipt = {
        "schema": "v26c_j4_recovery.1", "stage": STAGE,
        "verdict": "PASS",
        "verdict_kind": "TECHNICAL / OFFLINE",
        "verdict_meaning": "the recovery update completed and every pre-write structural and "
                           "numerical check passed. This is NOT a closed-loop qualification and "
                           "asserts NOTHING about gait, penetration or deployability. Only a "
                           "fresh deterministic closed-loop revalidation can qualify this actor.",
        "closed_loop_qualified": False,
        "pre_write_verification": verification,
        "authorisation": pre["authorisation"], "lineage_preserved": pre["lineage_preserved"],
        "inputs_sha256": pre["inputs_sha256"], "j2_parent": pre["j2_parent"],
        "actor_contract": pre["actor_contract"], "dataset": built["report"],
        "dataset_sha256": pre["dataset_sha256"],
        "hyperparameters": dict(JULY_RECOVERY_HP),
        "hyperparameter_source": JULY_SOURCES["hyperparameters"],
        "july_sources": dict(JULY_SOURCES),
        "split": {"rows": int(rows), "training": int(len(train_idx)),
                  "validation": int(len(val_idx)), "rule": "ONE default_rng(123); the split is "
                                                           "drawn first and the SAME generator "
                                                           "shuffles every epoch"},
        "selection": {"best_epoch": int(best["epoch"]), "epochs_run": len(history),
                      "best_validation_mse": float(best["val"]),
                      "rule": "first epoch improving by more than 1e-9; stop after 60 stale "
                              "epochs; best state restored"},
        "metrics": {"initial_rmse": float(np.sqrt(np.mean((initial - act) ** 2))),
                    "adapted_rmse": rmse},
        "zero_columns_after_fit": zero_after,
        "critic_trained": False, "ppo_updates": 0,
        "deployable": False, "promotion": "NONE", "next_stage_authorized": False,
        "revalidation": {"required": True, "executed": False, "authorised": False,
                         "note": "no closed-loop claim may be made from this receipt"},
        "composition": {"nominal_repeat": NOMINAL_REPEAT, "recovery_repeat": RECOVERY_REPEAT,
                        "overrides_refused": True},
        "multistart_derogation": dict(MULTISTART_DEROGATION),
        "corpus_condition": dict(CORPUS_CONDITION),
        "amendment_sha256": PIN_AMENDMENT,
        "forbidden_here": list(pre["forbidden_here"]),
        "generated_at_utc": _utc(),
    }
    receipt["outputs_sha256"] = {str(p.relative_to(out)): _sha_file(p)
                                 for p in sorted(out.rglob("*")) if p.is_file()}
    (out / RECEIPT_NAME).write_text(
        json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8")
    return receipt


# ================================================================ post-crash finalisation ========

def verify_j4_payloads(leaf: Path) -> dict[str, Any]:
    """The six preserved payloads, by exact hash. Nothing may be missing, extra or changed."""
    leaf = Path(leaf)
    if not leaf.is_dir():
        raise J4Error(f"the J4 leaf is missing: {leaf}")
    present = sorted(str(q.relative_to(leaf)) for q in leaf.rglob("*") if q.is_file())
    if present != sorted(PIN_J4_PAYLOADS):
        raise J4Error(f"the J4 leaf holds {present}, expected exactly "
                      f"{sorted(PIN_J4_PAYLOADS)}")
    got: dict[str, str] = {}
    for rel, pin in PIN_J4_PAYLOADS.items():
        h = _sha_file(leaf / rel)
        if h != pin:
            raise J4Error(f"the preserved payload {rel} changed: {h} != {pin}")
        got[rel] = h
    return got


def _reconstruct_selection(history: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Replay July's selection rule over the recorded history. No training, pure arithmetic.

    Fail-closed on a malformed history: the epochs must be integral and contiguous 1..N, every
    loss finite, N within the budget, and the stopping condition must be exactly consistent with
    the recorded length - a run shorter than the budget must have stopped on exactly `patience`
    stale epochs, no more and no fewer.
    """
    budget = int(JULY_RECOVERY_HP["epochs"])
    patience = int(JULY_RECOVERY_HP["patience"])
    if not isinstance(history, list):
        raise J4Error(f"the recorded history is a {type(history).__name__}, expected a list")
    if not history:
        raise J4Error("the recorded history is empty")
    n = len(history)
    if n > budget:
        raise J4Error(f"the history holds {n} epochs, above the budget of {budget}")

    best_epoch, best_val, stale = 0, float("inf"), 0
    for index, entry in enumerate(history, start=1):
        if not isinstance(entry, Mapping):
            raise J4Error(f"history entry {index} is not a mapping")
        for field in ("epoch", "train_loss", "validation_mse"):
            if field not in entry:
                raise J4Error(f"history entry {index} has no {field}")
        raw_epoch = entry["epoch"]
        epoch = float(raw_epoch)
        if not math.isfinite(epoch) or epoch != int(epoch):
            raise J4Error(f"history entry {index} has a non-integral epoch {raw_epoch!r}")
        if int(epoch) != index:
            raise J4Error(f"the history is not contiguous: entry {index} declares epoch "
                          f"{int(epoch)}")
        for field in ("train_loss", "validation_mse"):
            value = float(entry[field])
            if not math.isfinite(value):
                raise J4Error(f"the recorded {field} at epoch {index} is not finite: "
                              f"{entry[field]!r}")
        val = float(entry["validation_mse"])
        if val < best_val - 1e-9:
            best_val, best_epoch, stale = val, index, 0
        else:
            stale += 1
        # Reaching patience anywhere BEFORE the last entry is impossible: the training loop would
        # have stopped at that epoch, so no later epoch could have been recorded.
        if stale >= patience and index < n:
            raise J4Error(f"the history is impossible: {stale} stale epochs are reached at epoch "
                          f"{index}, yet the run continued to epoch {n}. The loop stops the "
                          f"moment stale reaches {patience}")

    if not 1 <= best_epoch <= n:
        raise J4Error(f"the reconstructed best epoch {best_epoch} is outside 1..{n}")
    if stale > patience:
        raise J4Error(f"the final entry carries {stale} stale epochs, above the patience of "
                      f"{patience}; the run could not have reached it")
    patience_fired = stale == patience
    if n < budget:
        if not patience_fired:
            raise J4Error(f"the run stopped at epoch {n}, below the budget of {budget}, with "
                          f"{stale} stale epochs; early stopping requires exactly {patience}")
        stopped_by = "patience"
    else:
        # at the budget boundary both conditions can coincide; say so rather than pick one
        stopped_by = ("patience at the final epoch, which is also the epoch budget"
                      if patience_fired else "epoch budget")
    return {"best_epoch": best_epoch, "epochs_run": n,
            "best_validation_mse": best_val, "stale_epochs_at_stop": stale,
            "patience": patience, "epoch_budget": budget,
            "patience_fired": patience_fired, "stopped_by": stopped_by,
            "rule": "first epoch improving by more than 1e-9; stop after 60 stale epochs; best "
                    "state restored",
            "reconstructed_from": "history.json, replayed; no training was run"}


def finalize_existing(*, authorized_stage: str | None, leaf: Path | None = None,
                      original_log: Path | None = None) -> dict[str, Any]:
    """Write the MISSING receipt for the single preserved execution. NO training of any kind.

    This function never constructs an optimiser, never calls fit(), never imports torch and never
    rewrites a payload. It reads the six preserved files, reconstructs every metric and invariant
    from them, copies the original log bit-identically into the leaf, and writes exactly two new
    files: the forensic log copy and the receipt.
    """
    if authorized_stage != STAGE:
        raise J4Error(f"requires --authorized-stage {STAGE}; got {authorized_stage!r}")
    leaf = Path(leaf) if leaf is not None else J4_LEAF
    log_src = Path(original_log) if original_log is not None else ORIGINAL_LOG

    # ---- no-clobber FIRST, so an already finalised leaf is refused as such ------------------
    receipt_path = leaf / RECEIPT_NAME
    log_path = leaf / FORENSIC_LOG_NAME
    for target in (receipt_path, log_path):
        if target.exists():
            raise J4Error(f"no-clobber: {target.name} already exists; this leaf has already been "
                          "finalised and is never finalised twice")
    # ---- then the preserved payloads, before anything is read from them --------------------
    payloads_before = verify_j4_payloads(leaf)

    # ---- the inputs and the amendment manifest ---------------------------------------------
    pre = preflight()
    if pre["blockers"]:
        raise J4Error(f"preflight BLOCKED: {pre['blockers']}")

    # ---- the dataset on disk must BE the one the preflight blessed --------------------------
    with np.load(leaf / "recovery_dataset.npz", allow_pickle=False) as archive:
        obs = np.asarray(archive["observations"], dtype=np.float32)
        act = np.asarray(archive["actions"], dtype=np.float32)
        times = np.asarray(archive["times"], dtype=np.float64)
        stored_names = tuple(str(n) for n in np.asarray(archive["actor_feature_names"]).tolist())
    rebuilt = build_recovery_dataset()
    if not (np.array_equal(obs, rebuilt["observations"]) and np.array_equal(act,
                                                                            rebuilt["actions"])):
        raise J4Error("the stored recovery dataset differs from the one rebuilt from the pinned "
                      "inputs")
    if not np.array_equal(times, rebuilt["times"]) or stored_names != feature_names():
        raise J4Error("the stored dataset times or feature names differ from the rebuilt ones")
    dataset_sha = _sha_obj({"observations": obs.tolist(), "actions": act.tolist()})
    if dataset_sha != pre["dataset_sha256"]:
        raise J4Error(f"the dataset content hash {dataset_sha} differs from the preflight's "
                      f"{pre['dataset_sha256']}")

    # ---- the sidecars are the J2 copies, and the manifest describes THIS module -------------
    for rel, source in (("rl_module/class_and_ctor_args.pkl", "class_and_ctor_args.pkl"),
                        ("rl_module/metadata.json", "metadata.json")):
        if _sha_file(leaf / rel) != PIN_J2_MODULE[f"rl_module/{source}"]:
            raise J4Error(f"{rel} is not a byte-identical copy of the J2 sidecar")
    manifest = json.loads((leaf / "rl_module/actor_feature_manifest.json").read_text())
    if manifest["module_state_sha256"] != payloads_before["rl_module/module_state.pkl"]:
        raise J4Error("the manifest does not describe the module_state it sits next to")
    if manifest["source_module_state_sha256"] != PIN_J2_MODULE["rl_module/module_state.pkl"]:
        raise J4Error("the manifest does not name the pinned J2 parent")
    if manifest["deployable"] is not False or tuple(manifest["actor_feature_names"]) != \
            feature_names():
        raise J4Error("the manifest claims deployability or a different actor schema")

    # ---- selection, metrics and invariants, reconstructed from disk -------------------------
    history = json.loads((leaf / "history.json").read_text())
    selection = _reconstruct_selection(history)
    names = feature_names()
    controller = trainable_controller_features(names)
    clk = list(clock_indices(names))
    import pickle
    with (leaf / "rl_module/module_state.pkl").open("rb") as fh:
        final = {k: np.asarray(v, dtype=np.float32) for k, v in pickle.load(fh).items()}
    parent = load_parent_state()

    rows = len(obs)
    rng = np.random.default_rng(int(JULY_RECOVERY_HP["seed"]))
    perm = rng.permutation(rows)
    n_val = int(round(rows * float(JULY_RECOVERY_HP["validation_fraction"])))
    val_idx, train_idx = np.sort(perm[:n_val]), np.sort(perm[n_val:])

    n_nominal = int(rebuilt["report"]["nominal_states"])
    n_recovery = int(rebuilt["report"]["recovery_steps"])

    def rmse(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.sqrt(np.mean((np.asarray(a, dtype=np.float64)
                                      - np.asarray(b, dtype=np.float64)) ** 2)))

    parent_nominal = student_mean_actions(obs[:n_nominal], parent)
    final_nominal = student_mean_actions(obs[:n_nominal], final)
    parent_recovery = student_mean_actions(obs[-n_recovery:], parent)
    final_recovery = student_mean_actions(obs[-n_recovery:], final)
    shift = np.asarray(final_nominal, dtype=np.float64) - np.asarray(parent_nominal,
                                                                     dtype=np.float64)
    initial_all = student_mean_actions(obs, parent)
    adapted_all = student_mean_actions(obs, final)

    for key, value in sorted(final.items()):
        if not bool(np.all(np.isfinite(value))):
            raise J4Error(f"the preserved parameter {key} contains non-finite values")
    controller_norms = {n: float(np.linalg.norm(final["pi.0.0.weight"][:, names.index(n)]))
                        for n in controller}
    dead = sorted(n for n, v in controller_norms.items() if not v > 0.0)
    if dead:
        raise J4Error(f"controller-memory columns stayed at zero norm: {dead}")
    zero_after = [c for c in range(ACTOR_WIDTH)
                  if bool(np.all(final["pi.0.0.weight"][:, c] == 0.0))]
    if zero_after != clk:
        raise J4Error(f"the preserved zero columns are {zero_after}, expected {clk}")
    for alias in ("pi.0.0.weight", "pi_encoder.0.weight"):
        if not bool(np.all(final[alias][:, clk] == 0.0)):
            raise J4Error(f"the clock columns are not bit-zero in {alias}")
    for direct, encoder in (("pi.0.0.weight", "pi_encoder.0.weight"),
                            ("pi.0.0.bias", "pi_encoder.0.bias"),
                            ("pi.0.2.weight", "pi_encoder.2.weight"),
                            ("pi.0.2.bias", "pi_encoder.2.bias")):
        if not bool(np.array_equal(final[direct], final[encoder])):
            raise J4Error(f"the aliases {direct} and {encoder} are not bit-identical")
    action_dim = act.shape[1]
    for key in ("pi.1.weight", "pi.1.bias"):
        head_rows = slice(action_dim, None)
        if not bool(np.array_equal(final[key][head_rows], parent[key][head_rows])):
            raise J4Error(f"the frozen log-std head changed in {key}")
    if final["pi.0.0.weight"].shape != (final["pi.0.0.weight"].shape[0], ACTOR_WIDTH):
        raise J4Error("the preserved actor is not 35D")

    recovery_before, recovery_after = rmse(parent_recovery, act[-n_recovery:]), \
        rmse(final_recovery, act[-n_recovery:])
    nominal_before, nominal_after = rmse(parent_nominal, act[:n_nominal]), \
        rmse(final_nominal, act[:n_nominal])
    shift_rms, shift_max = float(np.sqrt(np.mean(shift ** 2))), float(np.max(np.abs(shift)))
    shift_per_action = [float(v) for v in np.sqrt(np.mean(shift ** 2, axis=0))]
    metrics = {"initial_rmse": rmse(initial_all, act), "adapted_rmse": rmse(adapted_all, act)}
    for label, value in (("nominal_rmse_before", nominal_before),
                         ("nominal_rmse_after", nominal_after),
                         ("recovery_rmse_before", recovery_before),
                         ("recovery_rmse_after", recovery_after),
                         ("nominal_shift_rms", shift_rms),
                         ("nominal_shift_max_abs", shift_max),
                         ("initial_rmse", metrics["initial_rmse"]),
                         ("adapted_rmse", metrics["adapted_rmse"]),
                         *((f"nominal_shift_per_action[{i}]", v)
                           for i, v in enumerate(shift_per_action)),
                         *((f"controller_norm[{n}]", v)
                           for n, v in sorted(controller_norms.items()))):
        if not math.isfinite(value):
            raise J4Error(f"the reconstructed metric {label} is not finite: {value!r}")
    if not (recovery_after < recovery_before):
        raise J4Error(f"the recovery error did not improve: {recovery_after!r} is not below "
                      f"{recovery_before!r}; the preserved run cannot be finalised as a PASS")

    # ---- the payload must BE the restored best state, not merely the last one ---------------
    # The training-time validation MSE was measured on obs/scale with the pre-absorption weights;
    # the saved payload holds W/scale and is applied to raw obs, which is the same product. Any
    # difference is float32 accumulation, so the comparison is a tight relative one.
    reconstructed_val_mse = float(np.mean(
        (np.asarray(student_mean_actions(obs[val_idx], final), dtype=np.float32)
         - act[val_idx]) ** 2))
    if not math.isfinite(reconstructed_val_mse):
        raise J4Error("the reconstructed validation MSE is not finite")
    if not bool(np.isclose(reconstructed_val_mse, selection["best_validation_mse"],
                           rtol=BEST_STATE_RTOL, atol=BEST_STATE_ATOL)):
        raise J4Error(
            f"the payload does not reproduce the recorded best validation MSE: reconstructed "
            f"{reconstructed_val_mse!r} vs recorded {selection['best_validation_mse']!r} "
            f"(rtol={BEST_STATE_RTOL}, atol={BEST_STATE_ATOL}). The saved module is not the "
            "restored best state.")
    best_state_check = {
        "recorded_best_validation_mse": selection["best_validation_mse"],
        "reconstructed_validation_mse": reconstructed_val_mse,
        "absolute_difference": abs(reconstructed_val_mse - selection["best_validation_mse"]),
        "relative_difference": (abs(reconstructed_val_mse - selection["best_validation_mse"])
                                / abs(selection["best_validation_mse"])
                                if selection["best_validation_mse"] else 0.0),
        "rtol": BEST_STATE_RTOL, "atol": BEST_STATE_ATOL,
        "tolerance_rationale": BEST_STATE_TOLERANCE_RATIONALE,
        "validation_rows": int(len(val_idx)),
        "restored_best_state_verified": True,
    }

    # ---- the original log, preserved bit-identically ---------------------------------------
    if not log_src.is_file():
        raise J4Error(f"the original run log is missing: {log_src}")
    log_sha = _sha_file(log_src)
    if log_sha != PIN_ORIGINAL_LOG:
        raise J4Error(f"the original run log changed: {log_sha} != {PIN_ORIGINAL_LOG}")
    log_text = log_src.read_bytes()
    if ORIGINAL_EXCEPTION.encode() not in log_text:
        raise J4Error("the original log does not contain the recorded exception")

    runner_sha = _sha_file(Path(__file__).resolve())
    if runner_sha == PIN_FAILING_RUNNER:
        raise J4Error("the finaliser is byte-identical to the runner that crashed; the shadowing "
                      "fix is not in place")

    receipt: dict[str, Any] = {
        "schema": "v26c_j4_recovery.1", "stage": STAGE,
        "verdict": "PASS",
        "verdict_kind": "TECHNICAL/OFFLINE - POST-CRASH FINALIZED",
        "verdict_meaning": "the single J4 execution completed training, restored the best state, "
                           "passed every pre-write verification and wrote its six payloads. It "
                           "then aborted while assembling this receipt. The receipt was "
                           "reconstructed from those preserved payloads WITHOUT any further "
                           "training. It asserts NOTHING about gait, penetration or "
                           "deployability.",
        "closed_loop_qualified": False,
        "post_crash_finalisation": {
            "original_exit_code": ORIGINAL_EXIT_CODE,
            "original_exception": ORIGINAL_EXCEPTION,
            "original_failure_site": ORIGINAL_FAILURE_SITE,
            "training_executions": 1,
            "retry": False,
            "failing_runner_sha256": PIN_FAILING_RUNNER,
            "finalizer_sha256": runner_sha,
            "payload_sha256_before": dict(payloads_before),
            "payloads_rewritten": False,
            "files_written": list(FINALISATION_WRITES),
            "original_log_sha256": log_sha,
            "training_invoked_during_finalisation": False,
            "optimizer_constructed_during_finalisation": False,
        },
        "authorisation": pre["authorisation"],
        "lineage_preserved": pre["lineage_preserved"],
        "inputs_sha256": pre["inputs_sha256"],
        "amendment_sha256": PIN_AMENDMENT,
        "amendment_manifest_entries": len(
            json.loads(AMENDMENT.read_text())["pinned_artefacts_sha256"]),
        "j2_parent": pre["j2_parent"], "actor_contract": pre["actor_contract"],
        "dataset": rebuilt["report"], "dataset_sha256": dataset_sha,
        "hyperparameters": dict(JULY_RECOVERY_HP),
        "hyperparameter_source": JULY_SOURCES["hyperparameters"],
        "july_sources": dict(JULY_SOURCES),
        "composition": {"nominal_repeat": NOMINAL_REPEAT, "recovery_repeat": RECOVERY_REPEAT,
                        "overrides_refused": True},
        "multistart_derogation": dict(MULTISTART_DEROGATION),
        "corpus_condition": dict(CORPUS_CONDITION),
        "split": {"rows": int(rows), "training": int(len(train_idx)),
                  "validation": int(len(val_idx)),
                  "rule": "ONE default_rng(123); the split is drawn first and the SAME generator "
                          "shuffles every epoch",
                  "reconstructed": True},
        "selection": {**selection,
                      "reconstructed_validation_mse":
                          best_state_check["reconstructed_validation_mse"],
                      "restored_best_state_verified": True},
        "metrics": metrics,
        "pre_write_verification": {
            "reconstructed_from_payloads": True,
            "nominal_self_distillation": {"rmse_before": nominal_before,
                                          "rmse_after": nominal_after},
            "nominal_shift_vs_parent": {
                "rms": shift_rms, "max_abs": shift_max, "per_action_rms": shift_per_action,
                "july_reference": {"rms": 0.004175083023500488,
                                   "max_abs": 0.03159449528902769,
                                   "run": JULY_RECOVERY_REFERENCE["run"],
                                   "status": "REFERENCE ONLY, never a gate"}},
            "recovery_vs_teacher": {"rmse_before": recovery_before, "rmse_after": recovery_after,
                                    "improved": True,
                                    "requirement": "rmse_after < rmse_before, enforced before the "
                                                   "receipt is written. A direction, not a "
                                                   "magnitude"},
            "controller_column_norms": controller_norms,
            "controller_columns_all_positive": True,
            "zero_columns_after_fit": zero_after,
            "clock_bit_zero_in_both_aliases": {a: clk for a in ("pi.0.0.weight",
                                                                "pi_encoder.0.weight")},
            "aliases_bit_identical": True,
            "logstd_head_bit_identical_to_parent": True,
            "all_parameters_finite": True, "all_metrics_finite": True,
            "restored_best_state": best_state_check,
        },
        "critic_trained": False, "ppo_updates": 0,
        "deployable": False, "promotion": "NONE", "next_stage_authorized": False,
        "revalidation": {"required": True, "executed": False, "authorised": False,
                         "note": "no closed-loop claim may be made from this receipt"},
        "not_authorised_by_this_receipt": ["closed-loop rollout", "revalidation", "J5",
                                           "progressive recovery rounds", "deployment",
                                           "promotion", "critic", "PPO", "ex-novo", "retry"],
        "forbidden_here": list(pre["forbidden_here"]),
        "generated_at_utc": _utc(),
    }

    # ---- WRITE 1 of 2: the log copy, then everything is re-verified BEFORE the receipt -------
    log_path.write_bytes(log_text)
    if _sha_file(log_path) != PIN_ORIGINAL_LOG:
        raise J4Error("the forensic log copy is not bit-identical to the original")

    payloads_after = {rel: _sha_file(leaf / rel) for rel in PIN_J4_PAYLOADS}
    if not (payloads_before == payloads_after == PIN_J4_PAYLOADS):
        raise J4Error("a preserved payload changed during finalisation")
    added_now = sorted(str(q.relative_to(leaf)) for q in leaf.rglob("*") if q.is_file()
                       and str(q.relative_to(leaf)) not in PIN_J4_PAYLOADS)
    if added_now != [FORENSIC_LOG_NAME]:
        raise J4Error(f"after the log copy the leaf holds the extra files {added_now}, expected "
                      f"only {FORENSIC_LOG_NAME}")

    # The receipt is assembled COMPLETE and written exactly once. It hashes the six frozen
    # payloads and the forensic log; it never hashes itself.
    receipt["post_crash_finalisation"]["payload_sha256_after"] = payloads_after
    receipt["post_crash_finalisation"]["payload_sha256_unchanged"] = True
    receipt["outputs_sha256"] = {**payloads_after, FORENSIC_LOG_NAME: _sha_file(log_path)}
    receipt["outputs_sha256_excludes"] = {
        "file": RECEIPT_NAME,
        "why": "a receipt cannot hash itself; its own digest is taken by the reviewer"}

    # ---- WRITE 2 of 2 -----------------------------------------------------------------------
    receipt_path.write_text(
        json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8")

    # ---- verification only: the final file set, never a rewrite -----------------------------
    final_set = sorted(str(q.relative_to(leaf)) for q in leaf.rglob("*") if q.is_file())
    if final_set != sorted(list(PIN_J4_PAYLOADS) + list(FINALISATION_WRITES)):
        raise J4Error(f"the finalised leaf holds {final_set}, expected the six payloads plus "
                      f"{sorted(FINALISATION_WRITES)}")
    if {rel: _sha_file(leaf / rel) for rel in PIN_J4_PAYLOADS} != PIN_J4_PAYLOADS:
        raise J4Error("a preserved payload changed while the receipt was written")
    return receipt


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J4 July-faithful recovery stage")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--finalize-existing", action="store_true",
                   help="write the MISSING receipt for the single preserved execution. Reads the "
                        "six frozen payloads, trains nothing, rewrites nothing.")
    p.add_argument("--original-log", default=None,
                   help="explicit path to the original run log to preserve. The incident-specific "
                        "default is only a fallback.")
    a = p.parse_args(argv)
    if a.finalize_existing:
        r = finalize_existing(authorized_stage=a.authorized_stage,
                              leaf=(Path(a.out_dir) if a.out_dir else None),
                              original_log=(Path(a.original_log) if a.original_log else None))
        print(json.dumps({"verdict": r["verdict"], "kind": r["verdict_kind"],
                          "written": r["post_crash_finalisation"]["files_written"],
                          "original_log_sha256":
                              r["post_crash_finalisation"]["original_log_sha256"],
                          "outputs_sha256_entries": len(r["outputs_sha256"])}, indent=2))
        return 0 if r["verdict"] == "PASS" else 1
    if a.preflight or a.authorized_stage is None:
        r = preflight()
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "GO" else 1
    r = fit(authorized_stage=a.authorized_stage,
            out_dir=(Path(a.out_dir) if a.out_dir else None))
    return 0 if r["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
