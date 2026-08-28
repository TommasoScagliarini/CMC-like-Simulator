"""V26C J7 - the July-faithful Markov recovery dataset. THIS STAGE WRITES NOTHING BY DEFAULT.

WHAT IT CORRECTS
    J4 made two conceptual errors, both fixed here:
      1. it truncated the recovery trace against the J1 TEACHER trace, so the very first
         contact-onset difference (teacher step 14, student step 13) cut the corpus to 12 rows.
         J7's nominal trace and ONLY truncation reference is J3, the deterministic rollout of the
         J2 student. J1 is never the alignment reference.
      2. it used the J1 teacher STATES as the nominal anchor block, anchoring a distribution the
         student never visits. J7's nominal states are the ones actually visited in J3, and the
         nominal labels are the same J2 parent's means on those J3 states.
    J4 is retained ONLY as an implementation source for the physical scaling and the future fit.
    Its module, dataset, receipts and traces are forensic artefacts, never a parent or data.

THE DATASET
    nominal   500 J3 states, self-distilled J2 labels, repeat 32  -> 16000 rows
    recovery  713 RAW J6 states (429 + 273 + 11, July-strict), teacher labels, repeat 1 -> 713
    total     16713, nominal first, then recovery by seed 123, 124, 125
    multistart OMITTED - explicitly deferred

COLUMNS
    clock [0, 1]         projected to exact zero in the data; hard-zero in the weights already
    controller [25..34]  RAW and unmasked, with a non-degeneracy audit; trainable in a LATER fit

PENETRATION
    The per-seed j6_penetration.npz is pinned by its own hash - the J6 receipt attests the file,
    the pin attests the bytes actually read. The series is proved bit-identical to the trace's
    reward_terms.grf_penetration_m, then handed to v26c_penetration_contract.evaluate_series.
    Both the contract JSON and the evaluator module are pinned: the JSON binds the thresholds,
    the module binds their application. NO threshold is written down in this file.

THE TEACHER
    Labels and timing only: actions, times, actor_feature_names. Its observations are NEVER read.
    Every retained row proves a three-way clock identity, recovery == teacher == J3 within 1e-9.

WHAT IT DOES NOT DO
    It writes no dataset unless explicitly told to. It fits nothing, trains nothing, constructs no
    environment, runs no rollout, touches no critic and runs no PPO. It modifies no artefact.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pickle
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

import v26c_penetration_contract as PC  # noqa: E402  the ONLY penetration authority


class J7Error(RuntimeError):
    pass


STAGE = "V26C-J7-MARKOV-DATASET"
PREREG = HERE / "v26c_j7_prereg_markov_dataset.json"
PIN_PREREG = "bea97f268d2080968954cd694c7244cc3dc9d1f40ea4ae4ec79e9726fa9edf96"
# The evaluator MODULE, not only the contract JSON: the JSON binds the thresholds, this binds the
# implementation that applies them. The prereg pins it too, and verify_prereg checks both.
PENETRATION_EVALUATOR = HERE / "v26c_penetration_contract.py"
DATASET_NAME = "v26c_j7_markov_recovery_dataset.npz"
RECEIPT_NAME = "v26c_j7_markov_dataset_receipt.json"

# ------------------------------------------------------------------ the operational parent ------
PARENT_MODULE_DIR = HERE / "j2_runs" / "j2_base_v26c_2026-08-26_r1" / "rl_module"
PIN_PARENT_STATE = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
# EVIDENCE ONLY. Never a parent, never weights, never labels, never data.
J4_MODULE_STATE_SHA = "14a3630f757a5da2055eb754f6249fad8e7989a6d5e6c18f526c76415dad31aa"

# ------------------------------------------------------------------ the alignment reference -----
NOMINAL_TRACE = HERE / "j3_runs" / "j3_base_v26c_2026-08-26_r1" / "j3_trace.json"
PIN_NOMINAL_TRACE = "b36f85dc0b6aa8c0fa6d6d6b404ae8fdd51528129c3aeac5004451ec6d4bcbae"
# The teacher supplies RECOVERY LABELS ONLY. It is never the alignment reference and its states
# never enter the dataset.
TEACHER_DATASET = HERE / "j1_runs" / "j1_nominal_v26c_2026-08-26_r1" / "teacher_dataset.npz"
PIN_TEACHER_DATASET = "724d11342da3f3610152d7bd4cc7ca0dc1e8eb8c26a5b7c0947eb2451d1f8c41"
TEACHER_TRACE = HERE / "j1_runs" / "j1_nominal_v26c_2026-08-26_r1" / "teacher_trace.json"
PIN_TEACHER_TRACE = "39af8f0b2d4b8f7e44f917e82ea0e435fa3889c4d022d06b5d6373212c691bd3"

# ------------------------------------------------------------------ the recovery traces ---------
SEED_ORDER = (123, 124, 125)
J6_LEAF = HERE / "j6_runs" / "j6_recovery_probe_v26c_2026-08-26_seed{seed}"
PIN_J6_TRACES = {
    123: "96115593fcfd36eb2651f5e4e8f3a019b0833cbf0c0259e5bc114547c23d053b",
    124: "729128a8236f7189284e57267e964dd9e478e75d895c3b88287a537f75f67dab",
    125: "8e7060a0d32a05e275f41839cd0d7d453c792e33e65ab8d76bba4d7c588ec02e",
}
PIN_J6_RECEIPTS = {
    123: "77f8df26ce4a026145d1af12bed6061efb3aaf1424788579e10786bff1e7e32a",
    124: "73887c6b652983c3a640975d9d433e0e06672d98feaffc115512eb433e73fa78",
    125: "d7040aeb8a1b95d683d7eb432aefeaced9ab0a14de4dc13513bfb0bbd318ced0",
}
# The penetration series is pinned DIRECTLY, by its own hash. The J6 receipt alone does not
# substitute for the pin: the receipt attests the file, the pin attests the bytes read here.
PIN_J6_PENETRATION = {
    123: "0250c2c82afca4406b08fb8f69174c15da41d60b99d63c400ce442da32939b01",
    124: "19dff986e90aae27622e23904316091e4eb85bc9b0754a61696479ca0fc4c87e",
    125: "5276af956f0003820fc9a2f3e55e0946b38e7f9b9023fd300731ad151de9f89c",
}
EXPECTED_PREFIXES = {123: 429, 124: 273, 125: 11}
EXPECTED_RECOVERY_ROWS = 713

# Every penetration statement in this stage comes from the contract evaluator. There is no local
# threshold, no local comparison and no local band anywhere in this module.
TIME_IDENTITY_ATOL = 1e-9

# ------------------------------------------------------------------ composition -----------------
ACTOR_WIDTH = 35
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
NOMINAL_STEPS = 500
NOMINAL_REPEAT = 32
RECOVERY_REPEAT = 1
EXPECTED_NOMINAL_ROWS = NOMINAL_STEPS * NOMINAL_REPEAT          # 16000
EXPECTED_TOTAL_ROWS = EXPECTED_NOMINAL_ROWS + EXPECTED_RECOVERY_ROWS   # 16713
JULY_RECOVERY_OVER_NOMINAL = 712 / 16000                        # 0.0445
MULTISTART = "OMITTED / DEFERRED"

# The nominal labels are recomputed with the July numpy kernel while J3 recorded them through
# torch. The two agree to float32 rounding, not bit for bit; the measured agreement is 5.96e-07.
NOMINAL_LABEL_RTOL = 0.0
NOMINAL_LABEL_ATOL = 1e-5
NOMINAL_LABEL_TOLERANCE_RATIONALE = (
    "J3 recorded policy_mean through the torch inference path; J7 recomputes it with July's numpy "
    "_forward. Different kernels, same float32 arithmetic: bit-equality does not hold. float32 eps "
    "is 1.19e-07 and the measured agreement is 5.96e-07, so atol 1e-5 is ~84x eps and ~17x the "
    "observed residual.")

FORBIDDEN_HERE = ("writing the dataset without an explicit --out", "fit", "optimizer step",
                  "training", "critic", "PPO", "ex-novo", "environment construction", "rollout",
                  "promotion", "J4/J5 as parent or data", "J1 states in the dataset",
                  "J1 as the alignment reference", "multistart", "dedup", "downsampling",
                  "a minimum-prefix gate")


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_array(a: np.ndarray) -> str:
    arr = np.ascontiguousarray(a)
    h = hashlib.sha256()
    h.update(str(arr.dtype).encode())
    h.update(str(arr.shape).encode())
    h.update(arr.tobytes())
    return h.hexdigest()


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


# ================================================================ inputs =========================

def verify_prereg() -> dict[str, Any]:
    if not PREREG.is_file():
        raise J7Error("the J7 preregistration is missing")
    h = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and h != PIN_PREREG:
        raise J7Error(f"the J7 preregistration changed: {h} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    manifest = data.get("pinned_artefacts_sha256")
    if not manifest:
        raise J7Error("the preregistration pins no artefacts")
    checked: dict[str, str] = {}
    for rel, pin in manifest.items():
        target = HERE / rel
        if not target.is_file():
            raise J7Error(f"the preregistration pins {rel}, which is missing")
        got = _sha_file(target)
        if got != pin:
            raise J7Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        checked[rel] = got
    spec = data["dataset_specification"]
    if spec["total_rows"] != EXPECTED_TOTAL_ROWS \
            or spec["nominal_block"]["repeat"] != NOMINAL_REPEAT \
            or spec["recovery_block"]["repeat"] != RECOVERY_REPEAT:
        raise J7Error("the preregistration and the runner disagree on the composition")
    if data["decision_recorded"]["sigma_selected"] != 0.005:
        raise J7Error("the preregistration does not record sigma 0.005")
    # The imported evaluator must be the pinned file itself, not a namesake found earlier on
    # sys.path. Pinning the JSON binds the thresholds; this binds the code that applies them.
    imported = Path(PC.__file__).resolve()
    if imported != PENETRATION_EVALUATOR.resolve():
        raise J7Error(f"the imported penetration evaluator is {imported}, not the pinned "
                      f"{PENETRATION_EVALUATOR}")
    if "v26c_penetration_contract.py" not in checked:
        raise J7Error("the preregistration does not pin the penetration evaluator module")
    return {"file": _rel(PREREG), "sha256": h, "manifest_entries": len(checked),
            "penetration_evaluator": {
                "module": _rel(imported),
                "sha256": checked["v26c_penetration_contract.py"],
                "contract_json_sha256": checked["v26c_penetration_contract_2026-08-26.json"],
                "why_both": "the JSON binds the thresholds, the module binds their application"},
            "manifest_sha256": checked,
            "sigma_selected": data["decision_recorded"]["sigma_selected"],
            "recovery_repeat": data["decision_recorded"]["recovery_repeat"],
            "multistart": data["decision_recorded"]["multistart"]}


def load_parent_state() -> dict[str, np.ndarray]:
    path = PARENT_MODULE_DIR / "module_state.pkl"
    h = _sha_file(path)
    if h != PIN_PARENT_STATE:
        raise J7Error(f"the J2 parent state changed: {h} != {PIN_PARENT_STATE}")
    if h == J4_MODULE_STATE_SHA:
        raise J7Error("the parent resolves to the J4 module; J4 is forensic, never a parent")
    with path.open("rb") as fh:
        return {k: np.asarray(v) for k, v in pickle.load(fh).items()}


def actor_feature_names() -> tuple[str, ...]:
    manifest = json.loads((PARENT_MODULE_DIR / "actor_feature_manifest.json").read_text())
    names = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(names) != ACTOR_WIDTH:
        raise J7Error(f"the manifest holds {len(names)} names, expected {ACTOR_WIDTH}")
    return names


def _verified_trace(path: Path, pin: str, label: str) -> list[dict[str, Any]]:
    if not path.is_file():
        raise J7Error(f"the {label} is missing: {path}")
    h = _sha_file(path)
    if h != pin:
        raise J7Error(f"the {label} changed: {h} != {pin}")
    rows = json.loads(path.read_text())
    if not isinstance(rows, list) or not rows:
        raise J7Error(f"the {label} is not a non-empty list")
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            raise J7Error(f"{label} row {index} is not a mapping")
        if int(row.get("step", -1)) != index:
            raise J7Error(f"the {label} is not contiguous at row {index}")
    return rows


def _obs_matrix(rows: Sequence[Mapping[str, Any]], label: str) -> np.ndarray:
    out = np.empty((len(rows), ACTOR_WIDTH), dtype=np.float32)
    for i, row in enumerate(rows):
        v = np.asarray(row["actor_observation_vector_before"], dtype=np.float32).reshape(-1)
        if v.shape != (ACTOR_WIDTH,):
            raise J7Error(f"{label} row {i + 1} is {v.shape}, expected ({ACTOR_WIDTH},)")
        if not np.all(np.isfinite(v)):
            raise J7Error(f"{label} row {i + 1} holds a non-finite observation")
        out[i] = v
    return out


# ================================================================ July semantics ==================

def discrete_feature_indices(names: Sequence[str]) -> np.ndarray:
    """target_domain_noise_adaptation._discrete_feature_indices, transcribed."""
    return np.asarray(
        [i for i, n in enumerate(names)
         if str(n).endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated"))
         or str(n).startswith(("phase_fsm_", "phase_expected_"))], dtype=int)


def truncate_before_discrete_mismatch(nominal_rows: Sequence[Mapping[str, Any]],
                                      recovery_rows: Sequence[Mapping[str, Any]],
                                      names: Sequence[str]) -> tuple[list[Any], dict[str, Any]]:
    """target_domain_noise_adaptation.truncate_before_discrete_mismatch, transcribed.

    No tolerance, no window, no slack, and NO minimum-length gate.
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
        "first_discrete_mismatch_step": first_mismatch, "mismatching_columns": columns}


def july_forward(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    """target_domain_noise_adaptation._forward, transcribed. NATIVE float32."""
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
        raise J7Error("the parent's logits are not finite")
    return logits


def actor_means(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    """target_domain_markov_adaptation._actor_means, transcribed."""
    logits = july_forward(state, np.asarray(observations, np.float32))
    return np.asarray(logits[:, :2], dtype=float)


# ================================================================ assembly (IN MEMORY) ===========

def assemble() -> dict[str, Any]:
    """Build the dataset IN MEMORY and audit it. Writes nothing, fits nothing, runs nothing."""
    prereg = verify_prereg()
    names = actor_feature_names()
    state = load_parent_state()
    clock = list(CLOCK_COLUMNS)
    controller = list(CONTROLLER_COLUMNS)

    # ---- the NOMINAL block: J3 states, J2 self-distilled labels ---------------------------
    nominal_rows = _verified_trace(NOMINAL_TRACE, PIN_NOMINAL_TRACE, "J3 nominal trace")
    if len(nominal_rows) != NOMINAL_STEPS:
        raise J7Error(f"the J3 trace holds {len(nominal_rows)} rows, expected {NOMINAL_STEPS}")
    nominal_states = _obs_matrix(nominal_rows, "J3 nominal trace")
    clock_raw_abs_max = float(np.max(np.abs(nominal_states[:, clock])))
    nominal_states[:, clock] = 0.0
    nominal_labels = np.asarray(actor_means(state, nominal_states), dtype=np.float32)
    # cross-check against what J3 recorded through torch
    recorded = np.asarray([r["policy_mean"] for r in nominal_rows], dtype=np.float32)
    residual = float(np.max(np.abs(nominal_labels.astype(np.float64)
                                   - recorded.astype(np.float64))))
    if not bool(np.allclose(nominal_labels, recorded, rtol=NOMINAL_LABEL_RTOL,
                            atol=NOMINAL_LABEL_ATOL)):
        raise J7Error(f"the recomputed nominal labels disagree with the J3 policy_mean by "
                      f"{residual}, beyond atol {NOMINAL_LABEL_ATOL}")
    # zeroing the clock must not move the labels: the parent's clock columns are already zero
    unzeroed = _obs_matrix(nominal_rows, "J3 nominal trace")
    labels_unzeroed = np.asarray(actor_means(state, unzeroed), dtype=np.float32)
    if not np.array_equal(labels_unzeroed, nominal_labels):
        raise J7Error("projecting the clock to zero changed the nominal labels; the parent's "
                      "clock columns are not inert")

    # ---- the RECOVERY block: RAW J6 states, teacher labels --------------------------------
    # The teacher supplies LABELS and TIMING only. Its observations are never read: reading them
    # is exactly the J4 defect this stage exists to correct.
    if _sha_file(TEACHER_DATASET) != PIN_TEACHER_DATASET:
        raise J7Error("the teacher dataset changed")
    with np.load(TEACHER_DATASET, allow_pickle=False) as archive:
        teacher_actions = np.asarray(archive["actions"], dtype=np.float32)
        teacher_times = np.asarray(archive["times"], dtype=np.float64)
        teacher_names = tuple(str(n) for n in np.asarray(archive["actor_feature_names"]).tolist())
    if teacher_names != names:
        raise J7Error("the teacher dataset schema differs from the pinned actor manifest")
    if teacher_times.shape != (len(teacher_actions),):
        raise J7Error(f"the teacher times are {teacher_times.shape}, expected "
                      f"{(len(teacher_actions),)}")

    # The single penetration authority, loaded once. No threshold is ever written down here.
    contract = PC.load_contract()

    recovery_states: list[np.ndarray] = []
    recovery_labels: list[np.ndarray] = []
    per_seed: dict[str, Any] = {}
    for seed in SEED_ORDER:
        leaf = Path(str(J6_LEAF).format(seed=seed))
        trace = _verified_trace(leaf / "j6_trace.json", PIN_J6_TRACES[seed],
                                f"J6 seed {seed} trace")
        receipt_sha = _sha_file(leaf / RECEIPT_NAME.replace(
            "v26c_j7_markov_dataset_receipt", "v26c_j6_recovery_probe_receipt"))
        if receipt_sha != PIN_J6_RECEIPTS[seed]:
            raise J7Error(f"the J6 seed {seed} receipt changed")
        kept, report = truncate_before_discrete_mismatch(nominal_rows, trace, names)
        if report["retained_steps"] != EXPECTED_PREFIXES[seed]:
            raise J7Error(f"seed {seed} retained {report['retained_steps']} rows, expected "
                          f"{EXPECTED_PREFIXES[seed]}")
        states = _obs_matrix(kept, f"J6 seed {seed}")
        labels = teacher_actions[:len(kept)].copy()
        # SAME FIXED STEP: row k carries the teacher action of episode step k. The alignment is
        # proved three ways on the clock: the recovery row, the teacher dataset and J3 must all
        # name the same instant, or the label belongs to a different step than the state.
        time_residuals: list[float] = []
        for k, row in enumerate(kept):
            if int(row["step"]) != k + 1:
                raise J7Error(f"seed {seed} row {k + 1} is not the step it claims")
            t_rec = float(row["time_before"])
            t_tea = float(teacher_times[k])
            t_nom = float(nominal_rows[k]["time_before"])
            if abs(t_rec - t_tea) > TIME_IDENTITY_ATOL:
                raise J7Error(f"seed {seed} step {k + 1}: recovery time {t_rec} != teacher "
                              f"times[{k}] {t_tea}")
            if abs(t_rec - t_nom) > TIME_IDENTITY_ATOL:
                raise J7Error(f"seed {seed} step {k + 1}: recovery time {t_rec} != J3 {t_nom}")
            if abs(t_tea - t_nom) > TIME_IDENTITY_ATOL:
                raise J7Error(f"seed {seed} step {k + 1}: teacher times[{k}] {t_tea} != J3 "
                              f"{t_nom}")
            time_residuals.append(max(abs(t_rec - t_tea), abs(t_rec - t_nom),
                                      abs(t_tea - t_nom)))
        # the RAW states keep their controller memory; only the clock is projected
        states[:, clock] = 0.0
        means = np.asarray([r["policy_mean"] for r in kept], dtype=np.float32)
        diff = means.astype(np.float64) - labels.astype(np.float64)
        c = np.asarray([r["actor_observation_vector_before"][names.index("online_left_in_contact")]
                        for r in kept], dtype=float)

        # ---- penetration: the pinned series, cross-proved against the trace, judged only by
        # ---- the contract evaluator. No threshold appears in this module.
        pen_file = leaf / "j6_penetration.npz"
        pen_sha = _sha_file(pen_file)
        if pen_sha != PIN_J6_PENETRATION[seed]:
            raise J7Error(f"the J6 seed {seed} penetration series changed: {pen_sha} != "
                          f"{PIN_J6_PENETRATION[seed]}")
        with np.load(pen_file, allow_pickle=False) as pen_archive:
            penetration_m = np.asarray(pen_archive["penetration_m"], dtype=np.float64)
        from_trace = np.asarray([float(r["reward_terms"]["grf_penetration_m"]) for r in trace],
                                dtype=np.float64)
        if penetration_m.shape != from_trace.shape:
            raise J7Error(f"seed {seed}: the penetration series is {penetration_m.shape}, the "
                          f"trace carries {from_trace.shape}")
        # bit-equality, on the bytes: not allclose, not isclose
        if _sha_array(penetration_m) != _sha_array(from_trace):
            raise J7Error(f"seed {seed}: j6_penetration.npz is not bit-identical to the trace's "
                          f"reward_terms.grf_penetration_m")
        prefix_pen = penetration_m[:len(kept)]
        pen_eval = PC.evaluate_series(prefix_pen, contract,
                                      label=f"J6 seed {seed} retained prefix")
        per_seed[str(seed)] = {
            "leaf": _rel(leaf), "trace_sha256": PIN_J6_TRACES[seed],
            "receipt_sha256": PIN_J6_RECEIPTS[seed],
            "retained_steps": int(report["retained_steps"]),
            "first_discrete_mismatch_step": report["first_discrete_mismatch_step"],
            "mismatching_columns": report["mismatching_columns"],
            "policy_mean_vs_teacher": {
                "rmse": float(np.sqrt(np.mean(diff ** 2))),
                "max_abs": float(np.max(np.abs(diff))),
                "per_action_rmse": [float(v) for v in np.sqrt(np.mean(diff ** 2, axis=0))]},
            "phase_coverage": {"stance_rows": int(c.sum()), "swing_rows": int((1 - c).sum()),
                               "stance_fraction": float(c.mean())},
            "time_identity": {
                "rule": "recovery time_before == teacher times[k] == J3 time_before",
                "atol": TIME_IDENTITY_ATOL,
                "rows_checked": len(kept),
                "max_abs_residual_s": max(time_residuals) if time_residuals else 0.0,
                "teacher_observations_read": False},
            "penetration_band_coverage": {
                "series": _rel(pen_file), "series_sha256": pen_sha,
                "bit_identical_to_trace": True,
                "evaluated_by": "v26c_penetration_contract.evaluate_series",
                "contract_sha256": contract["sha256"],
                "samples": pen_eval["samples"],
                "max_penetration_m": pen_eval["max_penetration_m"],
                "mean_penetration_m": pen_eval["mean_penetration_m"],
                "argmax_index_1based": pen_eval["argmax_index_1based"],
                "band": pen_eval["band"],
                "counts": pen_eval["counts"],
                "fractions": pen_eval["fractions"],
                "flags": pen_eval["flags"],
                "binding_pass": pen_eval["binding_pass"],
                "binding_verdict": pen_eval["binding_verdict"],
                "thresholds_m": pen_eval["thresholds_m"],
                "semantics": pen_eval["semantics"],
                "counting_conventions": pen_eval["counting_conventions"],
                "is_a_gate_here": False},
        }
        recovery_states.append(states)
        recovery_labels.append(labels)

    recovery_obs = np.concatenate(recovery_states, axis=0)
    recovery_act = np.concatenate(recovery_labels, axis=0)
    if len(recovery_obs) != EXPECTED_RECOVERY_ROWS:
        raise J7Error(f"the recovery block holds {len(recovery_obs)} rows, expected "
                      f"{EXPECTED_RECOVERY_ROWS}")

    # ---- concatenation: nominal first, then recovery by seed --------------------------------
    observations = np.concatenate([np.tile(nominal_states, (NOMINAL_REPEAT, 1)),
                                   np.tile(recovery_obs, (RECOVERY_REPEAT, 1))], axis=0)
    actions = np.concatenate([np.tile(nominal_labels, (NOMINAL_REPEAT, 1)),
                              np.tile(recovery_act, (RECOVERY_REPEAT, 1))], axis=0)
    if observations.shape != (EXPECTED_TOTAL_ROWS, ACTOR_WIDTH) \
            or actions.shape != (EXPECTED_TOTAL_ROWS, teacher_actions.shape[1]):
        raise J7Error(f"the aggregate is {observations.shape}/{actions.shape}, expected "
                      f"{(EXPECTED_TOTAL_ROWS, ACTOR_WIDTH)}")
    if observations.dtype != np.float32 or actions.dtype != np.float32:
        raise J7Error("the aggregate must be float32")
    if not (np.all(np.isfinite(observations)) and np.all(np.isfinite(actions))):
        raise J7Error("the aggregate holds non-finite values")

    # ---- column policy audits ---------------------------------------------------------------
    if float(np.max(np.abs(observations[:, clock]))) != 0.0:
        raise J7Error("the clock columns are not exactly zero in the aggregate")
    W = np.asarray(state["pi.0.0.weight"])
    if not bool(np.all(W[:, clock] == 0.0)):
        raise J7Error("the parent's clock columns are not hard-zero in the weights")
    ctrl_abs = {names[c]: float(np.max(np.abs(observations[:, c]))) for c in controller}
    dead = sorted(n for n, v in ctrl_abs.items() if v == 0.0)
    if dead:
        raise J7Error(f"controller columns carry no signal in the dataset: {dead}")
    ctrl_std = {names[c]: float(np.std(observations[:, c])) for c in controller}
    flat = sorted(n for n, v in ctrl_std.items() if v == 0.0)
    if flat:
        raise J7Error(f"controller columns are constant in the dataset: {flat}")

    report = {
        "composition": {
            "nominal_states": NOMINAL_STEPS, "nominal_repeat": NOMINAL_REPEAT,
            "nominal_rows": EXPECTED_NOMINAL_ROWS,
            "nominal_states_source": "the J3 deterministic student rollout, NOT the J1 teacher",
            "nominal_label_source": "self-distilled: the J2 parent's means on those J3 states",
            "recovery_unique_rows": EXPECTED_RECOVERY_ROWS,
            "recovery_repeat": RECOVERY_REPEAT,
            "recovery_rows": EXPECTED_RECOVERY_ROWS * RECOVERY_REPEAT,
            "recovery_label_source": "J1 teacher_dataset.actions[step - 1], same fixed step",
            "order": "nominal block first, then recovery concatenated by seed 123, 124, 125",
            "total_rows": int(len(observations)),
            "multistart": MULTISTART, "dedup": False, "downsampling": False,
        },
        "ratios": {
            "recovery_over_nominal": EXPECTED_RECOVERY_ROWS * RECOVERY_REPEAT
            / EXPECTED_NOMINAL_ROWS,
            "recovery_over_aggregate": EXPECTED_RECOVERY_ROWS * RECOVERY_REPEAT
            / EXPECTED_TOTAL_ROWS,
            "july_recovery_over_nominal": JULY_RECOVERY_OVER_NOMINAL,
            "note": "recovery_repeat 1 reproduces July's recovery-to-nominal ratio; repeat 2 "
                    "would double it merely because today's prefixes are longer",
        },
        "nominal_label_identity": {
            "method": "target_domain_markov_adaptation._actor_means, transcribed",
            "cross_checked_against": "the policy_mean recorded in the J3 trace",
            "max_abs_residual": residual, "rtol": NOMINAL_LABEL_RTOL, "atol": NOMINAL_LABEL_ATOL,
            "bit_identical": bool(np.array_equal(nominal_labels, recorded)),
            "rationale": NOMINAL_LABEL_TOLERANCE_RATIONALE,
        },
        "column_policy": {
            "clock_columns": clock, "clock_raw_abs_max_before_projection": clock_raw_abs_max,
            "clock_zero_in_dataset": True, "clock_hard_zero_in_weights": True,
            "clock_projection_changed_labels": False,
            "controller_columns": controller, "controller_masked_in_dataset": False,
            "controller_abs_max": ctrl_abs, "controller_std": ctrl_std,
            "controller_non_degenerate": True,
        },
        "penetration_authority": {
            "module": _rel(PENETRATION_EVALUATOR),
            "module_sha256": _sha_file(PENETRATION_EVALUATOR),
            "contract": contract["path"], "contract_sha256": contract["sha256"],
            "thresholds_m": {"soft_diagnostic": contract["soft_m"],
                             "july_legacy": contract["july_legacy_m"],
                             "hard_binding": contract["hard_m"]},
            "series_pinned_directly": {str(s): PIN_J6_PENETRATION[s] for s in SEED_ORDER},
            "series_bit_identical_to_traces": True,
            "local_thresholds_in_this_module": 0,
            "note": "every penetration statement comes from evaluate_series. This stage applies "
                    "no penetration gate: the binding verdict is reported, never enforced here.",
        },
        "teacher_usage": {
            "fields_read": ["actions", "times", "actor_feature_names"],
            "observations_read": False,
            "why": "the teacher supplies recovery labels and the timing that proves their "
                   "alignment. Using its states was the J4 defect.",
        },
        "per_seed": per_seed,
        "schema": {"observations": list(observations.shape), "actions": list(actions.shape),
                   "dtype": "float32/float32", "actor_width": ACTOR_WIDTH,
                   "actor_feature_names": len(names)},
        "content_hashes": {"observations": _sha_array(observations),
                           "actions": _sha_array(actions)},
    }
    return {"observations": observations, "actions": actions,
            "actor_feature_names": np.asarray(names, dtype=str),
            "report": report, "prereg": prereg}


# ================================================================ preflight (READ-ONLY) ==========

def preflight() -> dict[str, Any]:
    """Assemble in memory, audit, and write NOTHING. No environment, no fit, no rollout."""
    built = assemble()
    report = built["report"]
    names = actor_feature_names()
    blockers: list[str] = []

    # NEGATIVE GUARD: the J1 teacher states must NOT appear in the dataset.
    teacher_rows = _verified_trace(TEACHER_TRACE, PIN_TEACHER_TRACE, "J1 teacher trace")
    teacher_states = _obs_matrix(teacher_rows, "J1 teacher trace")
    teacher_states[:, list(CLOCK_COLUMNS)] = 0.0
    nominal_block = built["observations"][:NOMINAL_STEPS]
    if np.array_equal(nominal_block, teacher_states):
        raise J7Error("the nominal block equals the J1 TEACHER states; J7 exists to fix exactly "
                      "that defect")
    nominal_rows = _verified_trace(NOMINAL_TRACE, PIN_NOMINAL_TRACE, "J3 nominal trace")
    j3_states = _obs_matrix(nominal_rows, "J3 nominal trace")
    j3_states[:, list(CLOCK_COLUMNS)] = 0.0
    if not np.array_equal(nominal_block, j3_states):
        raise J7Error("the nominal block is not the J3 states")
    teacher_overlap = float(np.mean(np.all(np.isclose(nominal_block, teacher_states,
                                                      rtol=0.0, atol=1e-9), axis=1)))

    out_root_note = ("no output path is configured in this phase; --build is refused and the "
                     "dataset is never written")
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "inert": {"dataset_written": False, "fit_executed": False,
                  "environment_constructed": False, "rollout_executed": False,
                  "critic_touched": False, "ppo_updates": 0,
                  "note": "the dataset is assembled in memory and discarded"},
        "preregistration": built["prereg"],
        "source_lineage": {
            "operational_parent": {"module": _rel(PARENT_MODULE_DIR),
                                   "module_state_sha256": PIN_PARENT_STATE,
                                   "j4_module_refused": J4_MODULE_STATE_SHA},
            "nominal_trace": {"file": _rel(NOMINAL_TRACE), "sha256": PIN_NOMINAL_TRACE,
                              "role": "nominal states AND the only truncation reference"},
            "teacher": {"dataset": _rel(TEACHER_DATASET), "sha256": PIN_TEACHER_DATASET,
                        "trace": _rel(TEACHER_TRACE), "trace_sha256": PIN_TEACHER_TRACE,
                        "role": "RECOVERY LABELS ONLY - never states, never the alignment "
                                "reference"},
            "recovery_traces": {str(s): PIN_J6_TRACES[s] for s in SEED_ORDER},
            "recovery_receipts": {str(s): PIN_J6_RECEIPTS[s] for s in SEED_ORDER},
            "recovery_penetration_series": {str(s): PIN_J6_PENETRATION[s] for s in SEED_ORDER},
            "july_is_methodological_only": "no July artefact is an operational input",
        },
        "negative_guard_j1_not_used_as_states": {
            "nominal_block_equals_teacher_states": False,
            "nominal_block_equals_j3_states": True,
            "row_wise_coincidence_fraction": teacher_overlap,
        },
        "corrects_two_defects_of_j4": {
            "alignment_reference": "J3, not J1",
            "nominal_states": "J3 visited states, not J1 teacher states",
            "j4_role": "implementation source for the physical scaling and the future fit only",
        },
        "composition": report["composition"], "ratios": report["ratios"],
        "penetration_authority": report["penetration_authority"],
        "teacher_usage": report["teacher_usage"],
        "nominal_label_identity": report["nominal_label_identity"],
        "column_policy": report["column_policy"],
        "schema": report["schema"], "content_hashes": report["content_hashes"],
        "per_seed": report["per_seed"],
        "metrics_are_not_gates": "the per-seed metrics and the prefix lengths are REPORTED. No "
                                 "minimum is applied to any of them.",
        "future_fit_specification_only": {
            "trainable": "the entire mean network", "logstd": "frozen", "critic": "excluded",
            "scales": "target_domain_markov_adaptation.MARKOV_CONTROLLER_FEATURE_SCALES verbatim",
            "executed_here": False},
        "output_policy": {"content_addressed": True, "no_clobber": True,
                          "requires_explicit_out": True, "note": out_root_note},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J7 Markov recovery dataset")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="explicit output path (no-clobber) for --build")
    p.add_argument("--build", action="store_true", help="assemble AND WRITE the dataset")
    a = p.parse_args(argv)
    if a.build:
        raise J7Error("the dataset write is not authorised in this phase: the mandate covers the "
                      "preregistration, the tooling, the tests and a read-only in-memory "
                      "preflight only")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
