"""V26C J9R1 - the closed-loop qualification of the J8 actor, with corrected instrumentation.

WHAT J9R1 IS
    A SEPARATE ADDITIVE STAGE, token V26C_J9R1_CLOSED_LOOP, leaf j9r1_runs/. It is NOT a retry:
    it does not resume, reuse or reference the consumed J9 attempt as anything but evidence. The
    J9 preregistration, runner, test and authorisation are pinned here and NEVER edited.

WHAT CHANGED - INSTRUMENTATION ONLY
    J9 built its base through the J1 builder with output_dir=None, which sets record_outputs False
    and save_outputs_on_close False and omits output_dir and output_prefix (v26c_j1_collect.py:
    325-330). Each cell added back only output_dir, so close() never called save_results()
    (osim_trj_cmc_like.py:2048-2053) and the post-close sim_outputs requirement could not be met.
    J9R1 builds the base with a REAL output root - exactly as J1, J3, J5 and J6 all do - and
    verifies record_outputs, save_outputs_on_close and output_prefix fail-closed before any
    environment exists. Recording is neutral with respect to the simulated result; it is not free
    of RAM and close-time I/O cost, and that residual technical risk is recorded in the amendment.

WHAT DID NOT CHANGE
    The entire scientific contract: the same six cells, order, seeds, sigma, hold, 500 steps, the
    same gates and thresholds, the same actor and lineage, the same reset semantics. Every cell
    still mutates ONLY episode_start_offset_s and its own output_dir.

---- inherited J9 description below, unchanged in substance ----

V26C J9 - the closed-loop qualification of the J8 actor. Six declared cells, one matrix.

THE ACTOR
    ONLY j8_runs/j8_recovery_fit_v26c_2026-08-26_r1, an exact six-file leaf verified by hash
    before AND after the matrix. It is loaded read-only: never copied, edited, re-derived or
    projected. Its FROZEN log-std head must already supply sigma 0.005; this stage never edits one.

    35D, one actor. Clock columns [0, 1] exactly zero; controller columns 25:35 LIVE, every one
    with a strictly positive norm. That is the mask flip J8 produced: in J1/J2/J3 those ten
    columns were still zero. No standalone 25D, no widening, no contralateral input.

THE MATRIX - six cells, 500 steps each, declared in advance, NO behavioural fail-fast
    A  deterministic     seed 123   offset 1.956870983805102 s   nominal
    B  deterministic     seed 123   offset 1.756870983805102 s   -0.20 s
    C  deterministic     seed 123   offset 2.156870983805102 s   +0.20 s
    D  stochastic_held   seed 123   offset 1.956870983805102 s   sigma 0.005, hold 1
    E  stochastic_held   seed 124   offset 1.956870983805102 s   sigma 0.005, hold 1
    F  stochastic_held   seed 125   offset 1.956870983805102 s   sigma 0.005, hold 1

    THE OFFSET UNIT IS SECONDS. A behavioural FAIL in one cell is preserved and the remaining
    cells still run; only a technical or integrity exception stops the matrix, fail-closed.

THE ENVIRONMENT
    The pinned runtime config through the hardened J1 builder. ONLY episode_start_offset_s may
    differ between cells, and the mutation is PROVED by diffing against the base config. One
    environment at a time, closed cleanly and FAIL-CLOSED. The RLlib checkpoint loader is used,
    so ray and torch are imported at run time, but NO Ray cluster, NO Ray worker and NO
    EnvRunner is ever started. Production sim_outputs are required.

THE GATE
    Per cell and binding: 500 steps, episode_time_limit, >= 2 valid cycles, zero phase timeouts,
    zero morphology causal failures, zero cancelled HS, <= 1 resync, penetration binding-pass
    under the contract, and the full unchanged J3 kinematic gate. Telemetry HS/TO coherence is a
    SEPARATE fail-closed technical invariant. Aggregate PASS iff 6/6 behavioural PASS and 6/6
    telemetry-valid.

NOT HERE
    No fit, no critic, no PPO, no promotion, no deployability, no LOTO/LOCO/B1R1/B1R2. FSM,
    morphology, reward, SEA and the C++ plugin are untouched.

Cross-platform: pathlib and os.rename only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pickle
import shutil
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26c_j1_collect as J1            # noqa: E402  hardened env builder + summariser
import v26c_j3_closed_loop as J3        # noqa: E402  deterministic parity + kinematic gate
import v26c_penetration_contract as PC  # noqa: E402  the ONLY penetration authority

REPO = J1.REPO if hasattr(J1, "REPO") else HERE.parents[3]
BASELINE = J3.BASELINE


class J9Error(RuntimeError):
    pass


STAGE = "V26C_J9R1_CLOSED_LOOP"
PREREG = HERE / "v26c_j9r1_recording_amendment.json"
PIN_PREREG = "cf84b4747a9fd1897bb77eeead692ea774213f5f4e110c7ebc8aaf88f66ce6fa"
# the J9 originals this stage supersedes operationally but NEVER edits
J9_PREREG_SHA = "e6a6888a473d5ef76b1f009037f7825995fd62b35cc51749dfe41ad7f718c44a"
J9_RUNNER_SHA = "8b6c36db2fd9bd63bde41d9903d4e1eb1d1e312b8cbf790e9bf1ea5b4875ac67"
J9_TEST_SHA = "c30a1ff40d1f3aeefdb1e02210ecf6e2a04d7749448a7592cba212e8b47d867f"
J9_AUTHORIZATION_SHA = "ff789766cb27798b659f7922ea2d88a0f49f112bb054634c6258a9dd49639597"
J9_FAILURE_RECORD_SHA = "67c97df1b302cfa98a77c673410e67e56ea0dd34b2d380cf65c34f8916645a20"

# ------------------------------------------------------------------ the ONLY actor --------------
J8_LEAF = HERE / "j8_runs" / "j8_recovery_fit_v26c_2026-08-26_r1"
J8_MODULE_DIR = J8_LEAF / "rl_module"
PIN_J8: dict[str, str] = {
    "history.json":
        "150315058a03f5c8e0c1ec851c68125f2036e7b6c9d0ae6024713098f21822a5",
    "rl_module/actor_feature_manifest.json":
        "0c88018d66a648c0a36826f6edbf5e5494ef0c9b496142e1e971e7ab3b1ade81",
    "rl_module/class_and_ctor_args.pkl":
        "897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02",
    "rl_module/metadata.json":
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "rl_module/module_state.pkl":
        "9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82",
    "v26c_j8_recovery_fit_receipt.json":
        "13adc58b81ea372edef43a3d57f62b5f971e6be87c5242a8a0e234e993de6995",
}
PIN_J2_PARENT = "0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130"
PIN_J7_DATASET = "bb9b21f029063562bc0229fcc6601dd98e19d071f115811f7d8cb918be852e27"
PIN_RUNTIME_CONFIG_SHA = J1.PIN_RUNTIME_CONFIG_SHA
PIN_CONTRACT = "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461"

# ------------------------------------------------------------------ the actor contract ----------
ACTOR_WIDTH = 35
CLOCK_COLUMNS = (0, 1)
CONTROLLER_COLUMNS = tuple(range(25, 35))
SIGMA = 0.005
SIGMA_TOLERANCE = 1e-6
NOISE_HOLD_STEPS = 1
EXPECTED_STEPS = 500

# ------------------------------------------------------------------ the frozen matrix -----------
OFFSET_NOMINAL = 1.956870983805102
OFFSET_MINUS = 1.756870983805102
OFFSET_PLUS = 2.156870983805102
OFFSET_UNIT = "seconds"
FROZEN_OFFSETS = (OFFSET_NOMINAL, OFFSET_MINUS, OFFSET_PLUS)
MATRIX: tuple[dict[str, Any], ...] = (
    {"id": "A", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_NOMINAL,
     "label": "nominal"},
    {"id": "B", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_MINUS,
     "label": "-0.20 s"},
    {"id": "C", "mode": "deterministic", "seed": 123, "offset_s": OFFSET_PLUS,
     "label": "+0.20 s"},
    {"id": "D", "mode": "stochastic_held", "seed": 123, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
    {"id": "E", "mode": "stochastic_held", "seed": 124, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
    {"id": "F", "mode": "stochastic_held", "seed": 125, "offset_s": OFFSET_NOMINAL,
     "label": "nominal sigma 0.005"},
)
RESET_TIME_TOLERANCE_S = 1e-9
RESET_TIME_TOLERANCE_WHY = (
    "episode_start_offset_s is an offset RELATIVE TO cfg.t_start, never an absolute OpenSim time. "
    "The expected reset time is computed with the production semantics, so the comparison is an "
    "exact arithmetic identity and 1e-9 s is a cross-platform float-representation allowance, not "
    "a behavioural tolerance. It is checked IMMEDIATELY after reset, never from a post-step "
    "record.")
RESET_SEMANTICS = (
    "osim_trj_cmc_like._initialise_episode: "
    "max_start = max(cfg.t_start, cfg.t_end - episode_duration); "
    "requested = cfg.t_start + max(0.0, episode_start_offset_s); "
    "episode_start = min(requested, max_start); self.t = episode_start")


def expected_reset_time(t_start: float, t_end: float, episode_duration: float | None,
                        offset_s: float) -> dict[str, Any]:
    """The production episode-start arithmetic, transcribed. Offsets are RELATIVE to t_start."""
    max_start = float(t_end)
    if episode_duration is not None:
        max_start = float(t_end) - float(episode_duration)
    max_start = max(float(t_start), max_start)
    requested = float(t_start) + max(0.0, float(offset_s))
    start = min(requested, max_start)
    return {"cfg_t_start_s": float(t_start), "cfg_t_end_s": float(t_end),
            "episode_duration_s": None if episode_duration is None else float(episode_duration),
            "episode_start_offset_s": float(offset_s),
            "requested_unclamped_time_s": requested,
            "max_start_s": max_start,
            "expected_reset_time_s": start,
            "clamped": bool(requested > max_start),
            "semantics": RESET_SEMANTICS}

# ------------------------------------------------------------------ the gate --------------------
# J1's common criteria MINUS its 0.020 penetration bar: the sole binding penetration criterion is
# the contract's 0.028 band, exactly as J6 established.
J9_COMMON_GATE: dict[str, Any] = {k: v for k, v in J1.J1_GATE.items()
                                  if k != "max_penetration_m_max"}
J9_KINEMATIC_GATE: dict[str, Any] = dict(J3.J3_KINEMATIC_GATE)   # UNCHANGED
J9_GATE_SOURCE = ("v26c_j1_collect.J1_GATE for every non-penetration criterion, "
                  "v26c_j3_closed_loop.J3_KINEMATIC_GATE unchanged for the kinematic quality, and "
                  "the penetration contract's 0.028 band as the SOLE binding penetration bar")
DIAGNOSTIC_NOT_BINDING = ("action_clipped_steps", "episode_return", "valid_cycle_count",
                          "realized_noise_rms", "policy_std")

# ------------------------------------------------------------------ the only destination --------
RELATIVE_LEAF_PARTS = ("j9r1_runs", "j9r1_closed_loop_v26c_2026-08-26_r1")
RELATIVE_LEAF = "/".join(RELATIVE_LEAF_PARTS)
STAGING_NAME = ".staging_" + RELATIVE_LEAF_PARTS[-1]
LOCK_NAME = ".lock_" + RELATIVE_LEAF_PARTS[-1]
RECEIPT_NAME = "v26c_j9r1_closed_loop_receipt.json"
# the INERT sentinel output root the preflight builds against. It must never be created.
PREFLIGHT_SENTINEL = HERE / "_j9r1_preflight_sentinel_never_created"
OUTPUT_ROOT_OVERRIDE: Path | None = None

FORBIDDEN_HERE = ("fit", "optimizer step", "weight update", "critic", "PPO", "promotion",
                  "deployability", "editing or copying the actor", "editing a log-std head",
                  "a second actor", "widening", "contralateral features", "a standalone 25D actor",
                  "LOTO", "LOCO", "B1R1", "B1R2", "behavioural fail-fast",
                  "an autonomous retry", "a local penetration threshold",
                  "mutating any env field but episode_start_offset_s",
                  "starting a Ray cluster", "starting Ray workers", "EnvRunners")


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _sha_obj(o: Any) -> str:
    return hashlib.sha256(json.dumps(o, sort_keys=True, default=str).encode()).hexdigest()


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

def verify_amendment() -> dict[str, Any]:
    """Verify the J9R1 AMENDMENT: its own hash, every pin it carries, and its scientific fields."""
    if not PREREG.is_file():
        raise J9Error("the J9R1 amendment is missing")
    digest = _sha_file(PREREG)
    if PIN_PREREG != "PENDING" and digest != PIN_PREREG:
        raise J9Error(f"the J9R1 amendment changed: {digest} != {PIN_PREREG}")
    data = json.loads(PREREG.read_text())
    # the amendment PROPOSES the stage; it does not authorise it
    if data.get("stage_proposed") != STAGE:
        raise J9Error(f"the amendment proposes {data.get('stage_proposed')!r}, not {STAGE}")
    if data.get("stage_authorised") is not None:
        raise J9Error("the amendment must not claim stage_authorised; a fresh user authorisation "
                      "is required and this record does not grant it")
    local: dict[str, str] = {}
    for rel, pin in data["pinned_artefacts_sha256"].items():
        target = HERE / rel
        if not target.is_file():
            raise J9Error(f"the preregistration pins {rel}, which is missing")
        got = _sha_file(target)
        if got != pin:
            raise J9Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        local[rel] = got
    repo: dict[str, str] = {}
    for rel, pin in data["pinned_repo_artefacts_sha256"].items():
        target = REPO / rel
        got = _sha_file(target)
        if got != pin:
            raise J9Error(f"the pinned artefact {rel} changed: {got} != {pin}")
        repo[rel] = got
    # the amendment carries the scientific contract under scientific_contract_unchanged
    sci = data["scientific_contract_unchanged"]
    declared = sci["cell_list"]
    if [c["id"] for c in declared] != [c["id"] for c in MATRIX]:
        raise J9Error("the amendment's matrix order and the runner's disagree")
    for want, got in zip(MATRIX, declared):
        if (got["mode"], got["seed"], got["episode_start_offset_s"]) != \
                (want["mode"], want["seed"], want["offset_s"]):
            raise J9Error(f"cell {want['id']} differs between the amendment and the runner")
    if sci["behavioural_fail_fast"] is not False:
        raise J9Error("the amendment must declare behavioural_fail_fast false")
    if sci["offset_unit"] != "SECONDS of episode start time":
        raise J9Error("the amendment must declare the offset unit as SECONDS")
    if float(sci["sigma"]) != SIGMA or int(sci["noise_hold_steps"]) != NOISE_HOLD_STEPS \
            or int(sci["steps_per_cell"]) != EXPECTED_STEPS or int(sci["cells"]) != len(MATRIX):
        raise J9Error("the amendment and the runner disagree on sigma, hold, steps or cell count")
    # the J9 originals must still be byte-identical: this stage supersedes them operationally,
    # never by edit
    for rel, pin in (("v26c_j9_prereg_closed_loop.json", J9_PREREG_SHA),
                     ("v26c_j9_closed_loop.py", J9_RUNNER_SHA),
                     ("test_v26c_j9_closed_loop.py", J9_TEST_SHA),
                     ("v26c_j9_closed_loop_authorization.json", J9_AUTHORIZATION_SHA),
                     ("v26c_j9_technical_failure_2026-08-26.json", J9_FAILURE_RECORD_SHA)):
        got = _sha_file(HERE / rel)
        if got != pin:
            raise J9Error(f"the J9 artefact {rel} changed: {got} != {pin}. J9R1 supersedes J9 "
                          "operationally and must never edit it.")
    status = str(data.get("authorisation_status", ""))
    if not status.startswith("NOT_GRANTED"):
        raise J9Error(f"the amendment's authorisation_status is {status!r}; it must be "
                      "NOT_GRANTED. This record proposes a stage and grants nothing.")
    # FIELD BY FIELD against the ORIGINAL J9 preregistration: the scientific contract must be
    # value-identical. J9R1 changed instrumentation only, and this proves it against the pinned
    # original rather than against a restatement of it.
    j9 = json.loads((HERE / "v26c_j9_prereg_closed_loop.json").read_text())
    j9m = j9["matrix"]
    if [c["id"] for c in j9m["cell_list"]] != [c["id"] for c in sci["cell_list"]]:
        raise J9Error("the J9R1 amendment and the original J9 preregistration disagree on the "
                      "cell order")
    for a, b in zip(j9m["cell_list"], sci["cell_list"]):
        if (a["mode"], a["seed"], a["episode_start_offset_s"], a["label"]) != \
                (b["mode"], b["seed"], b["episode_start_offset_s"], b["label"]):
            raise J9Error(f"cell {a['id']} differs from the original J9 preregistration")
    for key in ("cells", "steps_per_cell", "offset_unit", "behavioural_fail_fast",
                "noise_hold_steps"):
        if j9m.get(key) != sci.get(key):
            raise J9Error(f"the scientific field {key!r} differs from the original J9 "
                          f"preregistration: {sci.get(key)!r} != {j9m.get(key)!r}")
    j9_bands = j9["penetration_authority"]["bands"]
    r1_bands = sci["penetration_bands"]
    if (float(j9_bands["soft_diagnostic_m"]) != float(r1_bands["soft_diagnostic_m"])
            or float(j9_bands["july_legacy_diagnostic_m"])
            != float(r1_bands["july_legacy_diagnostic_m"])
            or float(j9_bands["hard_binding_m"]) != float(r1_bands["hard_binding_m"])):
        raise J9Error("the penetration bands differ from the original J9 preregistration")
    if j9["per_cell_binding_gate"]["kinematic_quality"]["ankle_min_rad_max"] != -0.03:
        raise J9Error("the original J9 kinematic gate is not the one J9R1 inherits")
    return {"file": _rel(PREREG), "sha256": digest,
            "manifest_entries": len(local) + len(repo),
            "supersedes_j9_without_editing_it": {
                "v26c_j9_prereg_closed_loop.json": J9_PREREG_SHA,
                "v26c_j9_closed_loop.py": J9_RUNNER_SHA,
                "test_v26c_j9_closed_loop.py": J9_TEST_SHA,
                "v26c_j9_closed_loop_authorization.json": J9_AUTHORIZATION_SHA,
                "v26c_j9_technical_failure_2026-08-26.json": J9_FAILURE_RECORD_SHA},
            "pinned_artefacts_sha256": local, "pinned_repo_artefacts_sha256": repo}


def verify_scientific_equivalence() -> dict[str, Any]:
    """Prove FIELD BY FIELD that J9R1 changed instrumentation ONLY.

    The original J9 runner is hashed BEFORE it is imported, then every scientific constant is
    compared. A single difference is a fail-closed refusal: this stage may not quietly alter the
    matrix, a gate, a threshold, a seed, sigma, the hold, the step count or the reset rule.
    Importing it is light - it pulls the same J1/J3/contract modules J9R1 already has, and no
    heavy stack.
    """
    path = HERE / "v26c_j9_closed_loop.py"
    got = _sha_file(path)
    if got != J9_RUNNER_SHA:
        raise J9Error(f"the original J9 runner changed: {got} != {J9_RUNNER_SHA}. It is refused "
                      "before import.")
    import importlib
    orig = importlib.import_module("v26c_j9_closed_loop")
    if Path(orig.__file__).resolve() != path.resolve():
        raise J9Error(f"the imported J9 runner is {orig.__file__}, not the pinned {path}")

    compared: dict[str, Any] = {}
    diffs: list[str] = []

    def same(name: str, mine: Any, theirs: Any) -> None:
        equal = mine == theirs
        compared[name] = {"equal": bool(equal), "value": mine}
        if not equal:
            diffs.append(f"{name}: J9R1 {mine!r} != J9 {theirs!r}")

    same("MATRIX", MATRIX, orig.MATRIX)
    same("EXPECTED_STEPS", EXPECTED_STEPS, orig.EXPECTED_STEPS)
    same("SIGMA", SIGMA, orig.SIGMA)
    same("SIGMA_TOLERANCE", SIGMA_TOLERANCE, orig.SIGMA_TOLERANCE)
    same("NOISE_HOLD_STEPS", NOISE_HOLD_STEPS, orig.NOISE_HOLD_STEPS)
    same("J9_COMMON_GATE", J9_COMMON_GATE, orig.J9_COMMON_GATE)
    same("J9_KINEMATIC_GATE", J9_KINEMATIC_GATE, orig.J9_KINEMATIC_GATE)
    same("DIAGNOSTIC_NOT_BINDING", DIAGNOSTIC_NOT_BINDING, orig.DIAGNOSTIC_NOT_BINDING)
    same("RESET_TIME_TOLERANCE_S", RESET_TIME_TOLERANCE_S, orig.RESET_TIME_TOLERANCE_S)
    same("RESET_SEMANTICS", RESET_SEMANTICS, orig.RESET_SEMANTICS)
    same("PIN_CONTRACT", PIN_CONTRACT, orig.PIN_CONTRACT)
    same("ACTOR_WIDTH", ACTOR_WIDTH, orig.ACTOR_WIDTH)
    same("CLOCK_COLUMNS", CLOCK_COLUMNS, orig.CLOCK_COLUMNS)
    same("CONTROLLER_COLUMNS", CONTROLLER_COLUMNS, orig.CONTROLLER_COLUMNS)
    same("FROZEN_OFFSETS", FROZEN_OFFSETS, orig.FROZEN_OFFSETS)
    same("OFFSET_NOMINAL", OFFSET_NOMINAL, orig.OFFSET_NOMINAL)
    same("OFFSET_MINUS", OFFSET_MINUS, orig.OFFSET_MINUS)
    same("OFFSET_PLUS", OFFSET_PLUS, orig.OFFSET_PLUS)
    same("OFFSET_UNIT", OFFSET_UNIT, orig.OFFSET_UNIT)
    same("PIN_J8", PIN_J8, orig.PIN_J8)
    same("PIN_J2_PARENT", PIN_J2_PARENT, orig.PIN_J2_PARENT)
    same("PIN_J7_DATASET", PIN_J7_DATASET, orig.PIN_J7_DATASET)
    same("PIN_RUNTIME_CONFIG_SHA", PIN_RUNTIME_CONFIG_SHA, orig.PIN_RUNTIME_CONFIG_SHA)

    # the penetration bands, resolved through the shared contract rather than restated
    contract = PC.load_contract()
    same("penetration_bands",
         (contract["soft_m"], contract["july_legacy_m"], contract["hard_m"]),
         (0.020, 0.025, 0.028))

    if diffs:
        raise J9Error(f"J9R1 is NOT instrumentation-only; these scientific constants differ from "
                      f"the original J9 runner: {diffs}")
    # and the things that MUST differ, because they are the stage identity
    intentional = {
        "STAGE": {"j9r1": STAGE, "j9": orig.STAGE},
        "RELATIVE_LEAF": {"j9r1": RELATIVE_LEAF, "j9": orig.RELATIVE_LEAF},
        "RECEIPT_NAME": {"j9r1": RECEIPT_NAME, "j9": orig.RECEIPT_NAME},
    }
    for name, pair in intentional.items():
        if pair["j9r1"] == pair["j9"]:
            raise J9Error(f"{name} must DIFFER from J9: J9R1 is a separate stage with its own "
                          f"leaf, not a retry into the same one")
    return {"instrumentation_only": True, "fields_compared": len(compared),
            "all_equal": True, "compared": compared,
            "intentionally_different": intentional,
            "original_runner_sha256": got,
            "original_runner_unedited": True,
            "note": "every scientific constant is value-identical to the original J9 runner; only "
                    "the stage identity and the recording instrumentation differ"}


def verify_actor() -> dict[str, Any]:
    """The exact six-file leaf, the 35D mask contract, and the actor's own frozen sigma."""
    if not J8_LEAF.is_dir():
        raise J9Error(f"the J8 leaf is missing: {J8_LEAF}")
    present = sorted(str(p.relative_to(J8_LEAF)).replace(os.sep, "/")
                     for p in J8_LEAF.rglob("*") if p.is_file())
    if present != sorted(PIN_J8):
        raise J9Error(f"the J8 leaf holds {present}, expected exactly {sorted(PIN_J8)}")
    checked: dict[str, str] = {}
    for rel, pin in PIN_J8.items():
        got = _sha_file(J8_LEAF / rel)
        if got != pin:
            raise J9Error(f"the J8 artefact {rel} changed: {got} != {pin}")
        checked[rel] = got

    with (J8_MODULE_DIR / "module_state.pkl").open("rb") as fh:
        state = {k: np.asarray(v) for k, v in pickle.load(fh).items()}
    inputs = sorted(k for k, v in state.items()
                    if k.endswith(".weight") and v.ndim == 2 and v.shape[1] == ACTOR_WIDTH)
    if not inputs:
        raise J9Error(f"the J8 state holds no {ACTOR_WIDTH}D input layer")
    clock = list(CLOCK_COLUMNS)
    controller = list(CONTROLLER_COLUMNS)
    layers: dict[str, Any] = {}
    for key in inputs:
        W = state[key]
        zero = [c for c in range(W.shape[1]) if bool(np.all(W[:, c] == 0.0))]
        if zero != clock:
            raise J9Error(f"the J8 input layer {key} has zero columns {zero}; expected exactly "
                          f"the clock {clock}. The controller columns must be LIVE at this stage.")
        norms = {int(c): float(np.linalg.norm(W[:, c])) for c in controller}
        dead = sorted(c for c, v in norms.items() if v <= 0.0)
        if dead:
            raise J9Error(f"the J8 input layer {key} has dead controller columns {dead}")
        layers[key] = {"zero_columns": zero, "controller_norms": norms}
    widths = sorted({int(state[k].shape[1]) for k in inputs})
    if widths != [ACTOR_WIDTH]:
        raise J9Error(f"the J8 input layers are {widths}D; expected {ACTOR_WIDTH}D")

    # sigma comes from the actor's own FROZEN log-std head; this stage never edits one
    w = np.asarray(state["pi.1.weight"])
    b = np.asarray(state["pi.1.bias"])
    dim = w.shape[0] // 2
    if not bool(np.all(w[dim:] == 0.0)):
        raise J9Error("the actor's log-std output weights are not zero, so sigma is not constant")
    sigmas = np.exp(b[dim:].astype(np.float64))
    if not bool(np.all(np.isfinite(sigmas))) \
            or float(np.max(np.abs(sigmas - SIGMA))) > SIGMA_TOLERANCE:
        raise J9Error(f"the actor carries sigma {sigmas.tolist()}, expected {SIGMA} within "
                      f"{SIGMA_TOLERANCE}. This stage REFUSES to edit a log-std head.")

    receipt = json.loads((J8_LEAF / "v26c_j8_recovery_fit_receipt.json").read_text())
    lineage = {
        "parent_module_state_sha256": receipt["inputs"]["parent"]["module_state_sha256"],
        "dataset_sha256": receipt["inputs"]["dataset"]["sha256"],
        "j8_verdict": receipt["verdict"],
    }
    if lineage["parent_module_state_sha256"] != PIN_J2_PARENT:
        raise J9Error("the J8 receipt records a different J2 parent")
    if lineage["dataset_sha256"] != PIN_J7_DATASET:
        raise J9Error("the J8 receipt records a different J7 dataset")
    if lineage["j8_verdict"] != "PASS":
        raise J9Error(f"the J8 receipt verdict is {lineage['j8_verdict']!r}, not PASS")
    return {
        "leaf": _rel(J8_LEAF), "module": _rel(J8_MODULE_DIR), "artefacts_sha256": checked,
        "file_count": len(checked),
        "mask_contract": {"width": ACTOR_WIDTH, "clock_columns": clock,
                          "clock_exactly_zero": True,
                          "controller_columns": controller, "controller_live": True,
                          "layers": layers,
                          "note": "the mask FLIP relative to J1/J2/J3, where 25:35 were zero"},
        "sigma": {"value": SIGMA, "from": "the actor's own frozen log-std head",
                  "logstd_bias": [float(v) for v in b[dim:]],
                  "sigma_from_actor": [float(v) for v in sigmas],
                  "max_abs_deviation": float(np.max(np.abs(sigmas - SIGMA))),
                  "tolerance": SIGMA_TOLERANCE,
                  "logstd_head_edited": False, "module_copied": False},
        "lineage": {**lineage, "august_v26_operational": True,
                    "july_role": "methodology and evidence only"},
    }


def unit_correction() -> dict[str, Any]:
    """The additive correction: the offsets are SECONDS. No numeric artefact was affected."""
    # the offset-unit correction was authored in the ORIGINAL J9 preregistration and is
    # unchanged by J9R1. It is read from that PINNED file, whose hash verify_amendment checks.
    j9 = HERE / "v26c_j9_prereg_closed_loop.json"
    got = _sha_file(j9)
    if got != J9_PREREG_SHA:
        raise J9Error(f"the original J9 preregistration changed: {got} != {J9_PREREG_SHA}")
    data = {**json.loads(j9.read_text())["offset_unit_correction"],
            "source": "v26c_j9_prereg_closed_loop.json (pinned, unedited)",
            "source_sha256": got}
    return {**data, "unit": OFFSET_UNIT, "frozen_offsets_s": list(FROZEN_OFFSETS),
            "nominal_s": OFFSET_NOMINAL,
            "delta_minus_s": OFFSET_MINUS - OFFSET_NOMINAL,
            "delta_plus_s": OFFSET_PLUS - OFFSET_NOMINAL,
            "earlier_records_edited": False}


# ================================================================ env configuration ==============

def base_env_config(output_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """The pinned config through the HARDENED J1 builder, with a REAL output root.

    THE J9R1 CORRECTION. The J1 builder makes recording conditional on this argument
    (v26c_j1_collect.py:325-330): with output_dir=None it sets record_outputs False,
    save_outputs_on_close False and omits output_dir and output_prefix entirely. J9 passed None
    and then required the recorded output, which nothing had been asked to write.

    Passing a REAL root is what J1, J3, J5 and J6 all do. The base therefore carries recording ON,
    and every cell inherits it. The flags are then verified fail-closed, so this stage can never
    again reach an environment that was told not to record.
    """
    if output_root is None:
        raise J9Error("base_env_config requires a REAL output root. Passing None is exactly the "
                      "J9 defect: it disables record_outputs and save_outputs_on_close and omits "
                      "output_dir and output_prefix.")
    cfg = J1.load_pinned_config()
    env = J1.build_full_env_config(cfg, output_dir=output_root)
    J1.verify_env_config(env, cfg)
    if float(env["episode_start_offset_s"]) != OFFSET_NOMINAL:
        raise J9Error(f"the pinned config's episode_start_offset_s is "
                      f"{env['episode_start_offset_s']}, expected the nominal {OFFSET_NOMINAL}")
    # FAIL-CLOSED on the instrumentation itself, BEFORE any environment exists
    if env.get("record_outputs") is not True:
        raise J9Error(f"record_outputs is {env.get('record_outputs')!r}, expected exactly True. "
                      "An environment that is not recording cannot satisfy the sim_outputs "
                      "requirement, and this stage refuses to discover that after a 500-step run.")
    if env.get("save_outputs_on_close") is not True:
        raise J9Error(f"save_outputs_on_close is {env.get('save_outputs_on_close')!r}, expected "
                      "exactly True. Without it osim_trj_cmc_like.close() never calls "
                      "runner.save_results().")
    if not env.get("output_prefix"):
        raise J9Error("output_prefix is absent from the base env config; the J1 builder sets it "
                      "only alongside a real output_dir")
    if not env.get("output_dir"):
        raise J9Error("output_dir is absent from the base env config")
    return env, cfg


def cell_env_config(base: Mapping[str, Any], cfg: Mapping[str, Any], offset_s: float,
                    output_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Mutate ONLY episode_start_offset_s, and PROVE that nothing else moved."""
    if float(offset_s) not in [float(v) for v in FROZEN_OFFSETS]:
        raise J9Error(f"{offset_s} is not one of the three frozen offsets {FROZEN_OFFSETS}")
    env = dict(base)
    env["episode_start_offset_s"] = float(offset_s)
    env["output_dir"] = str(output_dir)
    differing = sorted(k for k in set(base) | set(env)
                       if base.get(k, "<absent>") != env.get(k, "<absent>"))
    allowed = sorted({"episode_start_offset_s", "output_dir"})
    # a SUBSET check: the nominal cell legitimately differs only in output_dir, because its
    # offset already equals the base one. Nothing outside the allowed pair may ever move.
    forbidden = sorted(set(differing) - set(allowed))
    if forbidden:
        raise J9Error(f"the cell config mutates {forbidden}; ONLY episode_start_offset_s (and the "
                      f"per-cell output_dir) may differ from the verified base")
    if float(env["episode_start_offset_s"]) != float(offset_s):
        raise J9Error("the cell config does not carry the requested offset")
    added = sorted(set(env) - set(base))
    removed = sorted(set(base) - set(env))
    if removed or sorted(set(added) - {"output_dir"}):
        raise J9Error(f"the cell config adds {added} and removes {removed} relative to the "
                      f"verified base; only the per-cell output_dir may be added")
    # NO second J1.verify_env_config on the MUTATED config. J1 deliberately pins
    # episode_start_offset_s to the pinned config's own value, including in its RUNTIME LITERAL
    # CROSS-CHECK, so cells B and C would fail it for a purely technical reason. The contract is:
    #   (1) the base is built AND verified in full by J1 BEFORE any mutation;
    #   (2) a shallow copy changes only episode_start_offset_s and output_dir;
    #   (3) fail-closed proof by key-set and value equality of EVERY other field against that
    #       already-verified base.
    # (3) is what carries the guarantee here, and it is stronger than re-running a verifier that
    # cannot express the one difference we declared.
    #
    # THE STABLE-KEY SET, defined explicitly. The J9 form excluded only episode_start_offset_s and
    # required len(base) - 1 identical fields. With a recording-enabled base, output_dir is ALREADY
    # in the base and legitimately differs per cell, so that count would be len(base) - 2 and the
    # guard would fail every cell. The correct statement is: every key that is NOT one of the two
    # declared-mutable ones must be exactly equal.
    stable_keys = set(base) - {"episode_start_offset_s", "output_dir"}
    differing_stable = sorted(k for k in stable_keys if base[k] != env.get(k, "<absent>"))
    if differing_stable:
        raise J9Error(f"these stable fields differ from the verified base: {differing_stable}. "
                      f"Only episode_start_offset_s and output_dir may ever move.")
    return env, {"mutated_keys": differing, "allowed_superset": allowed,
                 "mutated_is_subset_of_allowed": True,
                 "stable_keys_count": len(stable_keys),
                 "stable_keys_all_identical": True,
                 "stable_key_definition": "every base key except episode_start_offset_s and "
                                          "output_dir, compared for exact equality",
                 "episode_start_offset_s": float(offset_s),
                 "base_offset_s": float(base["episode_start_offset_s"]),
                 "delta_s": float(offset_s) - float(base["episode_start_offset_s"]),
                 "unit": OFFSET_UNIT,
                 "every_other_field_identical": True,
                 "base_verified_by_j1_before_mutation": True,
                 "recording_inherited_from_base": True,
                 "second_j1_verify_on_mutated_config": False,
                 "why_no_second_verify": "J1.verify_env_config pins the nominal offset, including "
                                         "in its runtime literal cross-check, so B and C would "
                                         "fail it technically. The base is verified in full "
                                         "BEFORE the mutation and every other field is proved "
                                         "identical to it afterwards."}


# ================================================================ the runtime stack ==============

def production_stack() -> Any:
    """J3's production stack EXTENDED with the production stochastic helper, exactly as J6 does.

    Heavy imports live here only, so the preflight stays inert. J3 and J6 are never modified.
    """
    stack = J3.production_stack()
    if str(BASELINE) not in sys.path:
        sys.path.insert(0, str(BASELINE))
    import rollout_eval as RE
    import exploration_noise as EN
    stack.held_stochastic_action = (
        lambda module, obs, shape, unit: RE._held_stochastic_action(module, obs, shape, unit))
    stack.held_normal = EN.HeldStandardNormal
    return stack


# ================================================================ gate ===========================

def evaluate_cell_gate(summary: Mapping[str, Any], knee: np.ndarray, ankle: np.ndarray,
                       penetration: Mapping[str, Any]) -> dict[str, Any]:
    """The per-cell BINDING behavioural gate. HS/TO coherence is deliberately NOT in here."""
    checks = {
        "steps": summary["steps"] == J9_COMMON_GATE["steps_required"],
        "end_reason": summary["end_reason"] == J9_COMMON_GATE["end_reason"],
        "valid_cycles": summary["valid_cycle_count"] >= J9_COMMON_GATE["valid_cycles_min"],
        "phase_timeout_stance":
            summary["phase_timeout_stance"] <= J9_COMMON_GATE["phase_timeout_stance_max"],
        "phase_timeout_swing":
            summary["phase_timeout_swing"] <= J9_COMMON_GATE["phase_timeout_swing_max"],
        "morphology_causal_contract_failure":
            summary["morphology_causal_contract_failure"]
            <= J9_COMMON_GATE["morphology_causal_contract_failure_max"],
        "hs_cancelled_count":
            summary["hs_cancelled_count"] <= J9_COMMON_GATE["hs_cancelled_count_max"],
        "resync_count": summary["resync_count"] <= J9_COMMON_GATE["resync_count_max"],
        "penetration_hard_binding": bool(penetration["binding_pass"]),
    }
    kin = J3.kinematic_quality(np.asarray(knee, dtype=np.float64),
                               np.asarray(ankle, dtype=np.float64))
    for name, block in kin.items():
        checks[f"kinematic_{name}"] = bool(block["pass"])
    failed = sorted(k for k, v in checks.items() if not v)
    return {
        "criteria": {"common": dict(J9_COMMON_GATE),
                     "kinematic_quality": {k: (list(v) if isinstance(v, tuple) else v)
                                           for k, v in J9_KINEMATIC_GATE.items()},
                     "penetration": "the contract's 0.028 binding band, the SOLE binding "
                                    "penetration criterion"},
        "source": J9_GATE_SOURCE,
        "kinematic_quality": kin,
        "ankle_min_direction": "ankle_min <= -0.03 rad; more negative PASSES, -0.0099 FAILS, "
                               "exactly -0.03 PASSES",
        "checks": checks, "failed": failed, "pass": not failed,
        "telemetry_integrity_evaluated_separately": True,
    }


def cell_verdict(gate: Mapping[str, Any], integrity: Mapping[str, Any]) -> str:
    return J3.overall_verdict(gate, integrity)


def penetration_report(series: Sequence[float], contract: Mapping[str, Any],
                       label: str) -> dict[str, Any]:
    """The ONLY penetration authority. No threshold is written down in this module."""
    ev = PC.evaluate_series(series, contract, label=label)
    return {**ev, "contract_sha256": contract["sha256"],
            "evaluated_by": "v26c_penetration_contract.evaluate_series",
            "soft_and_july_are_diagnostic": True,
            "sole_binding": "above_hard_binding"}


# ================================================================ preflight (INERT) ==============

def preflight() -> dict[str, Any]:
    """Fail-closed and provably inert: no heavy import, no environment, no write."""
    heavy_before = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                      "gymnasium") if m in sys.modules)
    amendment = verify_amendment()
    equivalence = verify_scientific_equivalence()
    actor = verify_actor()
    contract = PC.load_contract()
    if contract["sha256"] != PIN_CONTRACT:
        raise J9Error(f"the penetration contract changed: {contract['sha256']} != {PIN_CONTRACT}")
    # THE SENTINEL. The base is built against an output root that must NEVER be created: the J1
    # builder only formats path strings, and directory creation happens later inside
    # SimulationRunner.__init__, which the preflight never reaches. Existence is measured before
    # and after, so the claim is proved rather than asserted.
    sentinel_before = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                       "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    env, cfg = base_env_config(PREFLIGHT_SENTINEL)
    sentinel_after = {"root": bool(PREFLIGHT_SENTINEL.exists()),
                      "sim_outputs": bool((PREFLIGHT_SENTINEL / "sim_outputs").exists())}
    planned = []
    for cell in MATRIX:
        _, mutation = cell_env_config(env, cfg, cell["offset_s"], HERE / "PREFLIGHT_NOT_A_PATH")
        planned.append({**{k: cell[k] for k in ("id", "mode", "seed", "label")},
                        "episode_start_offset_s": cell["offset_s"],
                        "offset_unit": OFFSET_UNIT,
                        "delta_from_nominal_s": mutation["delta_s"],
                        "mutated_keys": mutation["mutated_keys"],
                        "steps": EXPECTED_STEPS,
                        "sigma": SIGMA if cell["mode"] == "stochastic_held" else None,
                        "noise_hold_steps": (NOISE_HOLD_STEPS
                                             if cell["mode"] == "stochastic_held" else None)})
    leaf = authorized_leaf()
    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    blockers: list[str] = []
    if leaf.exists() or leaf.is_symlink():
        blockers.append(f"the authorised leaf already exists: {leaf}")
    if staging.exists() or staging.is_symlink():
        blockers.append(f"a stale staging directory is in the way: {staging}")
    if lock_path.exists() or lock_path.is_symlink():
        blockers.append(f"a J9 lock is already held or was left behind: {lock_path}. This stage "
                        f"removes no lock it does not own.")
    heavy_after = sorted(m for m in ("torch", "ray", "opensim", "env_factory", "rollout_eval",
                                     "gymnasium") if m in sys.modules)
    introduced = sorted(set(heavy_after) - set(heavy_before))
    if introduced:
        blockers.append(f"the preflight introduced heavy modules: {introduced}")
    if sentinel_after != sentinel_before:
        blockers.append(f"the preflight created the sentinel output root: {sentinel_before} -> "
                        f"{sentinel_after}. Building the env config must touch no filesystem.")
    if sentinel_after["root"] or sentinel_after["sim_outputs"]:
        blockers.append(f"the sentinel output root exists after the preflight: {sentinel_after}")
    return {
        "verdict": "GO" if not blockers else "BLOCKED", "stage": STAGE, "blockers": blockers,
        "read_only": True,
        "inert": {"environment_constructed": False, "environment_reset": False,
                  "environment_stepped": False, "module_loaded": False,
                  "fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                  "leaf_created": False, "staging_created": False, "lock_taken": False,
                  "outputs_written": False,
                  "heavy_modules_before": heavy_before, "heavy_modules_after": heavy_after,
                  "heavy_modules_introduced_by_preflight": introduced,
                  "note": "inputs are verified and the matrix is planned in memory; nothing else"},
        "amendment": amendment,
        "scientific_equivalence_to_j9": equivalence,
        "actor": actor,
        "offset_unit_correction": unit_correction(),
        "recording_instrumentation": {
            "record_outputs": env["record_outputs"],
            "save_outputs_on_close": env["save_outputs_on_close"],
            "output_prefix": env["output_prefix"],
            "base_output_dir_from_builder": env["output_dir"],
            "verified_fail_closed_before_any_env": True,
            "inherited_by_every_cell": True,
            "corrects": "the J9 defect recorded in v26c_j9_technical_failure_2026-08-26.json",
            "parity": "the same recording mode J1, J3, J5 and J6 all used"},
        "preflight_sentinel": {
            "root": _rel(PREFLIGHT_SENTINEL),
            "existed_before": sentinel_before, "exists_after": sentinel_after,
            "created_by_the_preflight": False,
            "why_safe": "the J1 builder only formats path strings; directory creation happens "
                        "inside SimulationRunner.__init__, which the preflight never reaches",
            "measured_not_assumed": True},
        "runtime": {"pinned_config_sha256": PIN_RUNTIME_CONFIG_SHA,
                    "env_config_sha256": _sha_obj(env),
                    "builder": "v26c_j1_collect.build_full_env_config + verify_env_config",
                    "base_offset_s": float(env["episode_start_offset_s"]),
                    "environments": "one at a time, sequential, closed cleanly and fail-closed",
                    "rllib_checkpoint_loader": True,
                    "ray_cluster_started": False, "ray_workers_started": False,
                    "env_runners": False,
                    "ray_import_note": "the RLlib checkpoint loader imports ray and torch at RUN time; importing them is not starting a cluster, a worker or an EnvRunner. The preflight imports neither.",
                    "sim_outputs_required": True},
        "penetration_authority": {
            "contract": contract["path"], "contract_sha256": contract["sha256"],
            "module_sha256": _sha_file(HERE / "v26c_penetration_contract.py"),
            "bands_m": {"soft_diagnostic": contract["soft_m"],
                        "july_legacy": contract["july_legacy_m"],
                        "hard_binding": contract["hard_m"]},
            "semantics": {"above_soft_iff": "> 0.020 (diagnostic)",
                          "july_legacy_breach_iff": ">= 0.025 (diagnostic)",
                          "binding_pass_iff": "<= 0.028, so exactly 0.028 PASSES",
                          "binding_fail_iff": "> 0.028"},
            "local_thresholds_in_this_module": 0},
        "planned_matrix": planned,
        "matrix_policy": {"cells": len(MATRIX), "steps_per_cell": EXPECTED_STEPS,
                          "order": [c["id"] for c in MATRIX],
                          "behavioural_fail_fast": False,
                          "rule": "a behavioural FAIL is preserved and the remaining cells still "
                                  "run; only a technical or integrity exception stops the matrix, "
                                  "fail-closed",
                          "aggregate_pass_iff": "6/6 behavioural PASS and 6/6 telemetry-valid"},
        "gate_specification": {
            "binding_common": dict(J9_COMMON_GATE),
            "binding_kinematic": {k: (list(v) if isinstance(v, tuple) else v)
                                  for k, v in J9_KINEMATIC_GATE.items()},
            "binding_penetration": "the contract's 0.028 band, sole binding penetration criterion",
            "ankle_min_direction": "ankle_min <= -0.03 rad; more negative PASSES, -0.0099 FAILS, "
                                   "exactly -0.03 PASSES",
            "telemetry_integrity": "SEPARATE fail-closed technical invariant, never behavioural",
            "diagnostics_not_binding": list(DIAGNOSTIC_NOT_BINDING),
            "source": J9_GATE_SOURCE},
        "reset_time_check": {"tolerance_s": RESET_TIME_TOLERANCE_S,
                             "justification": RESET_TIME_TOLERANCE_WHY,
                             "semantics": RESET_SEMANTICS,
                             "offset_is_relative_to_t_start": True,
                             "compared_against": "cfg.t_start + offset, clamped to max_start",
                             "never_compared_against_the_offset_itself": True,
                             "checked_immediately_after_reset": True,
                             "recorded_fields": ["cfg_t_start_s", "cfg_t_end_s",
                                                 "episode_duration_s",
                                                 "requested_unclamped_time_s", "max_start_s",
                                                 "expected_reset_time_s",
                                                 "actual_reset_time_s", "reset_time_error_s",
                                                 "clamped"],
                             "checked_not_assumed": True},
        "multistart_disambiguation": {
            "training_data": "the two July teacher datasets of 8000 rows remain DEFERRED and "
                             "absent from J7/J8. This stage does not reintroduce them.",
            "closed_loop_validation": "cells B and C, and they are BINDING here",
            "they_are_different": "deferring the multistart TRAINING DATA says nothing about the "
                                  "multistart CLOSED-LOOP validation; the two must never be "
                                  "conflated"},
        "would_write": {"leaf": _rel(leaf), "relative_leaf": RELATIVE_LEAF,
                        "leaf_exists": bool(leaf.exists()),
                        "per_cell": ["j9_cell_<ID>_trace.json", "j9_cell_<ID>_kinematics.npz",
                                     "j9_cell_<ID>_penetration.npz"],
                        "aggregate": RECEIPT_NAME,
                        "staging": staging.name, "lock": lock_path.name,
                        "protocol": "exclusive sibling lock, staging, verify, re-check, atomic "
                                    "rename",
                        "outcome_fail_still_commits": True,
                        "output_root_override": None if OUTPUT_ROOT_OVERRIDE is None
                        else str(OUTPUT_ROOT_OVERRIDE)},
        "outcome_policy": {"deployable": False, "promotion": "NONE",
                           "next_stage_authorized": False, "critic": "excluded",
                           "ppo_updates": 0, "single_execution": True,
                           "no_autonomous_retry": True},
        "deferred_todo": json.loads(PREREG.read_text())["scientific_contract_unchanged"]["deferred_todo"],
        "requires_to_run": {"flag": "--run", "stage_token": STAGE,
                            "out": "must equal the authorised leaf exactly"},
        "forbidden_here": list(FORBIDDEN_HERE),
    }


# ================================================================ destination guards =============

def authorized_leaf() -> Path:
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    return root.joinpath(*RELATIVE_LEAF_PARTS)


def _refuse_symlink(path: Path, root: Path) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise J9Error(f"refusing a symlinked path component: {current}")
        if current == root or current.parent == current:
            return
        current = current.parent


def validate_stage(token: str | None) -> str:
    if token != STAGE:
        raise J9Error(f"--authorized-stage must be exactly {STAGE!r}, got {token!r}")
    return token


def validate_out(out_arg: str | None) -> Path:
    if out_arg is None:
        raise J9Error("--run requires --out, naming the authorised leaf exactly")
    leaf = authorized_leaf()
    got = Path(out_arg).expanduser()
    if got.is_symlink():
        raise J9Error(f"refusing a symlinked --out: {got}")
    if got.resolve(strict=False) != leaf.resolve(strict=False):
        raise J9Error(f"--out is {got}, which is not the authorised leaf {leaf}")
    root = Path(OUTPUT_ROOT_OVERRIDE) if OUTPUT_ROOT_OVERRIDE is not None else HERE
    _refuse_symlink(leaf.parent, root)
    if leaf.exists() or leaf.is_symlink():
        raise J9Error(f"the authorised leaf already exists; this stage is no-clobber and "
                      f"single-execution: {leaf}")
    return leaf


# ================================================================ one cell =======================

def run_cell(cell: Mapping[str, Any], *, stack: Any, base_env: Mapping[str, Any],
             cfg: Mapping[str, Any], contract: Mapping[str, Any], staging: Path,
             feature_names_expected: Sequence[str], progress: bool = True) -> dict[str, Any]:
    """ONE closed-loop cell. Builds, resets, steps and CLOSES exactly one environment."""
    cid = str(cell["id"])
    seed = int(cell["seed"])
    mode = str(cell["mode"])
    sim_out = staging / f"j9_cell_{cid}_sim_outputs"
    env_config, mutation = cell_env_config(base_env, cfg, float(cell["offset_s"]), sim_out)

    stack.seed(seed)
    module = stack.load_module(J8_MODULE_DIR)
    env = stack.make_env(env_config)
    env_closed = False
    try:
        base = env.unwrapped
        obs, _reset_info = env.reset(seed=seed)
        contract_report = J3._verify_runtime_contract(base, module, obs,
                                                      tuple(feature_names_expected), cfg)
        feature_names = tuple(contract_report["actor_feature_names"])
        action_shape = tuple(int(d) for d in env.action_space.shape)
        low = np.asarray(env.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(env.action_space.high, dtype=np.float64).reshape(-1)

        # THE RESET CHECK, immediately after reset and never from a post-step record.
        # episode_start_offset_s is RELATIVE to cfg.t_start; the expected absolute time is
        # computed with the production semantics, clamp included.
        #
        # Every input to that arithmetic is taken from the LIVE environment and guarded. Reading
        # them off our own env_config would prove nothing about what the environment actually did.
        sim_cfg = getattr(base, "cfg", None)
        if sim_cfg is None:
            raise J9Error(f"cell {cid}: the environment exposes no .cfg; the reset check cannot "
                          "be performed and this stage refuses to proceed without it")
        for attr in ("t_start", "t_end"):
            if not hasattr(sim_cfg, attr):
                raise J9Error(f"cell {cid}: the environment's cfg exposes no {attr}; "
                              "episode_start_offset_s is relative to t_start and cannot be "
                              "checked without it")
            value = getattr(sim_cfg, attr)
            if not isinstance(value, (int, float)) or not np.isfinite(float(value)):
                raise J9Error(f"cell {cid}: cfg.{attr} is {value!r}, not a finite number")
        env_cfg_live = getattr(base, "env_cfg", None)
        if env_cfg_live is None or not hasattr(env_cfg_live, "episode_duration"):
            raise J9Error(f"cell {cid}: the environment exposes no env_cfg.episode_duration; the "
                          "clamp cannot be reproduced and this stage refuses to guess it")
        live_duration = getattr(env_cfg_live, "episode_duration")
        if live_duration is not None:
            if not isinstance(live_duration, (int, float)) \
                    or not np.isfinite(float(live_duration)):
                raise J9Error(f"cell {cid}: env_cfg.episode_duration is {live_duration!r}, not a "
                              "finite number")
            live_duration = float(live_duration)
        # the LIVE value is NORMATIVE; our own config is only cross-checked against it
        declared_duration = env_config.get("episode_duration")
        if (live_duration is None) != (declared_duration is None):
            raise J9Error(f"cell {cid}: the live episode_duration is {live_duration!r} while the "
                          f"config declares {declared_duration!r}")
        if live_duration is not None \
                and abs(live_duration - float(declared_duration)) > 1e-12:
            raise J9Error(f"cell {cid}: the live episode_duration {live_duration} and the declared "
                          f"{float(declared_duration)} differ by more than 1e-12; the environment "
                          "is not running the episode length this stage configured")
        reset_expect = expected_reset_time(
            float(sim_cfg.t_start), float(sim_cfg.t_end), live_duration,
            float(cell["offset_s"]))
        reset_time = float(J1._finite(base.t, f"cell {cid}: reset time"))
        reset_error = abs(reset_time - reset_expect["expected_reset_time_s"])
        reset_report = {**reset_expect, "actual_reset_time_s": reset_time,
                        "reset_time_error_s": reset_error,
                        "tolerance_s": RESET_TIME_TOLERANCE_S,
                        "checked_immediately_after_reset": True,
                        "compared_against": "cfg.t_start + offset, clamped to max_start - NOT the "
                                            "offset itself",
                        "duration_sources": {
                            "live_env_cfg_episode_duration": live_duration,
                            "declared_env_config_episode_duration":
                                None if declared_duration is None else float(declared_duration),
                            "normative": "live_env_cfg_episode_duration",
                            "cross_check_tolerance_s": 1e-12,
                            "agree": True,
                            "why_live_is_normative": "the clamp is computed by the environment "
                                                     "from its OWN env_cfg; reading our config "
                                                     "instead would prove nothing about what the "
                                                     "environment actually did"},
                        "sources": {"t_start": "live base.cfg.t_start",
                                    "t_end": "live base.cfg.t_end",
                                    "episode_duration": "live base.env_cfg.episode_duration",
                                    "all_guarded_fail_closed": True}}
        if reset_error > RESET_TIME_TOLERANCE_S:
            raise J9Error(
                f"cell {cid}: the reset time {reset_time} differs from the expected "
                f"{reset_expect['expected_reset_time_s']} (= t_start {reset_expect['cfg_t_start_s']}"
                f" + offset {cell['offset_s']}, clamped={reset_expect['clamped']}) by "
                f"{reset_error} s, beyond {RESET_TIME_TOLERANCE_S}")

        held = None
        if mode == "stochastic_held":
            if getattr(stack, "held_stochastic_action", None) is None:
                raise J9Error("the stack exposes no held_stochastic_action helper")
            held = stack.held_normal(np.random.default_rng(seed), env.action_space.shape,
                                     NOISE_HOLD_STEPS)

        trace: list[dict[str, Any]] = []
        knee: list[float] = []
        ankle: list[float] = []
        pen: list[float] = []
        noise_rows: list[np.ndarray] = []
        clipped = 0
        parity_steps = 0
        max_abs_noise = 0.0

        for step in range(1, EXPECTED_STEPS + 1):
            obs_vec = np.asarray(obs, dtype=np.float32).reshape(-1)
            actor_obs = obs_vec[:ACTOR_WIDTH]
            if mode == "deterministic":
                raw, mean, std, path = J3.deterministic_action(module, obs_vec, action_shape,
                                                               torch_mod=stack.torch)
                dev = float(np.max(np.abs(np.asarray(raw, dtype=np.float64)
                                          - np.asarray(mean, dtype=np.float64))))
                max_abs_noise = max(max_abs_noise, dev)
                if dev != 0.0:
                    raise J9Error(f"cell {cid} step {step}: the deterministic action differs from "
                                  f"the policy mean by {dev}; noise is forbidden in this cell")
                if getattr(stack, "reference_action", None) is not None:
                    r_a, r_m, r_s, _ = stack.reference_action(module, obs_vec, action_shape)
                    if not (np.array_equal(raw, r_a) and np.array_equal(mean, r_m)
                            and np.array_equal(std, r_s)):
                        raise J9Error(f"cell {cid} step {step}: the deterministic action does not "
                                      "match rollout_eval's own helper bit for bit")
                    parity_steps += 1
                applied_noise = np.zeros(np.shape(raw), dtype=np.float32)
            else:
                unit = np.asarray(held.next(), dtype=np.float32).reshape(action_shape)
                raw, mean, std, applied_noise = stack.held_stochastic_action(
                    module, obs_vec, action_shape, unit)
                raw = np.asarray(raw, dtype=np.float32)
                mean = np.asarray(mean, dtype=np.float32)
                std = np.asarray(std, dtype=np.float32)
                applied_noise = np.asarray(applied_noise, dtype=np.float32)
                d_a, d_m, d_s, _ = J3.deterministic_action(module, obs_vec, action_shape,
                                                          torch_mod=stack.torch)
                if not np.array_equal(np.asarray(d_m, dtype=np.float32), mean):
                    raise J9Error(f"cell {cid} step {step}: forward_exploration and "
                                  "forward_inference disagree on the policy mean")
                if not np.array_equal(np.asarray(d_s, dtype=np.float32), std):
                    raise J9Error(f"cell {cid} step {step}: the two forward paths disagree on the "
                                  "std")
                if float(np.max(np.abs(std.astype(np.float64) - SIGMA))) > SIGMA_TOLERANCE:
                    raise J9Error(f"cell {cid} step {step}: the effective std is {std.tolist()}, "
                                  f"expected {SIGMA}")
                if not np.array_equal(applied_noise, (std * unit).astype(np.float32)):
                    raise J9Error(f"cell {cid} step {step}: applied noise is not std * unit_noise")
                parity_steps += 1
                noise_rows.append(np.asarray(applied_noise, dtype=np.float64).reshape(-1).copy())
                path = "rollout_eval._held_stochastic_action"

            applied = np.clip(raw, low.reshape(np.shape(raw)), high.reshape(np.shape(raw))
                              ).astype(np.float32)
            was_clipped = bool(np.any(applied != raw))
            clipped += int(was_clipped)
            pros = J1._prosthetic_state(actor_obs, feature_names)
            knee.append(pros["pros_knee_angle"])
            ankle.append(pros["pros_ankle_angle"])
            t_before = J1._finite(base.t, f"cell {cid} step {step}: time_before")

            obs, reward, terminated, truncated, info = env.step(raw)

            if "time" not in info:
                raise J9Error(f"cell {cid} step {step}: info exposes no 'time'")
            terms = J1._jsonable(info.get("reward_terms", {}), "reward_terms")
            if "grf_penetration_m" not in terms:
                raise J9Error(f"cell {cid} step {step}: reward_terms carries no "
                              "grf_penetration_m")
            pen.append(float(terms["grf_penetration_m"]))
            row: dict[str, Any] = {
                "step": step, "cell": cid,
                "time_before": t_before,
                "time_after": J1._finite(info["time"], f"cell {cid} step {step}: info.time"),
                "reward": J1._finite(reward, f"cell {cid} step {step}: reward"),
                "terminated": bool(terminated), "truncated": bool(truncated),
                "end_reason": str(info.get("end_reason", "")),
                "actor_observation_vector_before": actor_obs.astype(float).tolist(),
                "raw_action": np.asarray(raw, dtype=float).reshape(-1).tolist(),
                "policy_mean": np.asarray(mean, dtype=float).reshape(-1).tolist(),
                "policy_std_diagnostic": np.asarray(std, dtype=float).reshape(-1).tolist(),
                "action_noise": np.asarray(applied_noise, dtype=float).reshape(-1).tolist(),
                "applied_action_diagnostic": np.asarray(applied, dtype=float).reshape(-1).tolist(),
                "action_clipped_diagnostic": was_clipped,
                "action_selection_path": path,
                "stepped_with": "raw_action",
                "reward_terms": terms,
                J1.FSM_KEY: J1._jsonable(info.get(J1.FSM_KEY), J1.FSM_KEY),
                "prosthetic_state": pros,
            }
            for extra in ("morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                          "online_grf", "online_grf_detector"):
                if extra in info:
                    row[extra] = J1._jsonable(info[extra], extra)
            row["info_scalars"] = {
                k: J1._jsonable(v, k) for k, v in info.items()
                if k not in ("reward_terms", J1.FSM_KEY, "observation",
                             "morphology_causal_diagnostics", "morphology_ledger_diagnostics",
                             "online_grf", "online_grf_detector", "observer_raw_sensor_journal")
            }
            trace.append(row)
            if progress and (step % 50 == 0 or step == 1):
                print(json.dumps({"cell": cid, "step": step, "reward": row["reward"]}), flush=True)
            if terminated or truncated:
                break
    except BaseException:
        # a PRIMARY exception is never masked - but the environment is still closed
        try:
            env.close()
            env_closed = True
        except Exception:                                       # pragma: no cover
            env_closed = False
        raise
    else:
        # FAIL-CLOSED: with no primary exception, a failed close is itself a technical failure.
        # A leaked environment means the evidence was produced by a process we did not shut down.
        # The original cause is CHAINED, never swallowed and never replaced.
        #
        # `Exception`, deliberately NOT `BaseException`: a RuntimeError from close becomes a
        # chained J9Error, but KeyboardInterrupt and SystemExit are NOT exceptions of the run -
        # they are the operator stopping it. Rewriting those as a stage error would misreport an
        # interrupt as a technical failure of the environment, so they propagate untouched. The
        # outer cleanup catches BaseException and therefore still removes the staging and the
        # lock on an interrupt: fail-closed either way.
        try:
            env.close()
        except Exception as exc:
            raise J9Error(f"cell {cid}: the environment failed to close cleanly, so the cell is "
                          f"technically invalid and nothing is committed: {exc!r}") from exc
        env_closed = True

    end_reason = J1._resolve_end_reason(trace)
    knee_arr = np.asarray(knee, dtype=np.float64)
    ankle_arr = np.asarray(ankle, dtype=np.float64)
    pen_arr = np.asarray(pen, dtype=np.float64)
    if noise_rows:
        rms = [float(v) for v in np.sqrt(np.mean(np.asarray(noise_rows) ** 2, axis=0))]
    else:
        rms = [0.0, 0.0]
    summary = J1._summarise(trace, end_reason, clipped, rms)
    penetration = penetration_report(pen_arr, contract, f"J9 cell {cid}")
    gate = evaluate_cell_gate(summary, knee_arr, ankle_arr, penetration)
    integrity = J3.telemetry_integrity(summary)
    verdict = cell_verdict(gate, integrity)

    if not sim_out.is_dir() or not any(sim_out.iterdir()):
        raise J9Error(f"cell {cid}: production sim_outputs are missing at {sim_out}")

    trace_path = staging / f"j9_cell_{cid}_trace.json"
    trace_path.write_text(json.dumps(trace, indent=1, allow_nan=False), encoding="utf-8")
    kin_path = staging / f"j9_cell_{cid}_kinematics.npz"
    np.savez_compressed(kin_path, knee_rad=knee_arr, ankle_rad=ankle_arr,
                        actor_feature_names=np.asarray(feature_names, dtype=str))
    pen_path = staging / f"j9_cell_{cid}_penetration.npz"
    np.savez_compressed(pen_path, penetration_m=pen_arr)

    return {
        "id": cid, "mode": mode, "seed": seed, "label": str(cell["label"]),
        "episode_start_offset_s": float(cell["offset_s"]), "offset_unit": OFFSET_UNIT,
        "env_mutation": mutation,
        "env_config_sha256": _sha_obj(env_config),
        "reset_check": reset_report,
        "runtime_contract": contract_report,
        "action_semantics": {
            "mode": mode, "path": path, "stepped_with": "raw_action",
            "clipping_is_diagnostic": True,
            "rollout_eval_parity_steps": parity_steps,
            "max_abs_action_minus_mean": max_abs_noise if mode == "deterministic" else None,
            "noise_hold_steps": NOISE_HOLD_STEPS if mode == "stochastic_held" else None,
            "sigma": SIGMA if mode == "stochastic_held" else None},
        "summary": summary,
        "kinematics": {"steps": int(knee_arr.size),
                       "knee_min_rad": float(knee_arr.min()), "knee_max_rad": float(knee_arr.max()),
                       "ankle_min_rad": float(ankle_arr.min()),
                       "ankle_max_rad": float(ankle_arr.max())},
        "penetration": penetration,
        "gate": gate, "telemetry_integrity": integrity, "verdict": verdict,
        "behavioural_pass": bool(gate["pass"]),
        "telemetry_valid": bool(integrity["pass"]),
        "diagnostics_not_binding": {k: summary.get(k) for k in DIAGNOSTIC_NOT_BINDING},
        "env_closed": env_closed,
        "sim_outputs": _rel(sim_out),
        "outputs_sha256": {p.name: _sha_file(p) for p in (trace_path, kin_path, pen_path)},
        "content_hashes": {"knee_rad": _sha_array(knee_arr), "ankle_rad": _sha_array(ankle_arr),
                           "penetration_m": _sha_array(pen_arr)},
    }


# ================================================================ the matrix =====================

def run_matrix(out_arg: str | None, stage_token: str | None, *, stack: Any = None,
               progress: bool = True) -> dict[str, Any]:
    """All six cells, in the frozen order, with NO behavioural fail-fast."""
    if OUTPUT_ROOT_OVERRIDE is not None and stack is None:
        raise J9Error(f"OUTPUT_ROOT_OVERRIDE is set to {OUTPUT_ROOT_OVERRIDE}. It is permitted "
                      f"only for synthetic, isolated tests; the authorised matrix refuses it.")
    validate_stage(stage_token)
    leaf = validate_out(out_arg)
    pre = preflight()
    if pre["blockers"]:
        raise J9Error(f"preflight BLOCKED: {pre['blockers']}")

    contract = PC.load_contract()
    manifest = json.loads((J8_MODULE_DIR / "actor_feature_manifest.json").read_text())
    expected_features = tuple(str(n) for n in manifest["actor_feature_names"])
    if len(expected_features) != ACTOR_WIDTH:
        raise J9Error(f"the J8 manifest holds {len(expected_features)} names")
    actor_before = verify_actor()

    injected = stack is not None
    stack = stack if stack is not None else production_stack()

    staging = leaf.parent / STAGING_NAME
    lock_path = leaf.parent / LOCK_NAME
    if staging.exists() or staging.is_symlink():
        raise J9Error(f"a stale staging directory is in the way: {staging}")

    parent_created: Path | None = None
    staging_created: Path | None = None
    lock_owned: Path | None = None
    try:
        if not leaf.parent.exists():
            leaf.parent.mkdir(parents=True)
            parent_created = leaf.parent
        try:
            fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError as exc:
            raise J9Error(f"the J9 lock already exists: {lock_path}. This stage fails closed and "
                          f"removes no lock it does not own.") from exc
        lock_owned = lock_path
        try:
            os.write(fd, json.dumps({"stage": STAGE, "pid": os.getpid(),
                                     "leaf": str(leaf)}).encode("utf-8"))
        finally:
            os.close(fd)
        staging.mkdir()
        staging_created = staging

        # THE BASE IS BUILT HERE, with the STAGING as its output root, BEFORE any environment is
        # created or used. That is what gives every cell record_outputs True,
        # save_outputs_on_close True and an output_prefix - the instrumentation J9 lacked.
        base_env, cfg = base_env_config(staging)
        # DEFENCE IN DEPTH, at the call site and independent of base_env_config: the base that is
        # actually about to be used must carry recording ON. This fires BEFORE any environment is
        # constructed, so a recording-off base can never again be discovered after a 500-step run.
        for flag in ("record_outputs", "save_outputs_on_close"):
            if base_env.get(flag) is not True:
                raise J9Error(f"the base env config about to be used has {flag}="
                              f"{base_env.get(flag)!r}, expected exactly True. Refusing BEFORE "
                              "constructing any environment; this is the J9 defect and it is "
                              "caught here, not after the run.")
        if not base_env.get("output_prefix") or not base_env.get("output_dir"):
            raise J9Error("the base env config about to be used carries no output_prefix or no "
                          "output_dir; refusing before any environment is constructed")

        cells: list[dict[str, Any]] = []
        for cell in MATRIX:
            # NO behavioural fail-fast: every cell runs. A technical exception still fails closed.
            cells.append(run_cell(cell, stack=stack, base_env=base_env, cfg=cfg,
                                  contract=contract, staging=staging,
                                  feature_names_expected=expected_features, progress=progress))

        actor_after = verify_actor()
        if actor_after["artefacts_sha256"] != actor_before["artefacts_sha256"]:
            raise J9Error("the J8 actor changed during the matrix")

        behavioural = sum(1 for c in cells if c["behavioural_pass"])
        valid = sum(1 for c in cells if c["telemetry_valid"])
        aggregate_pass = behavioural == len(MATRIX) and valid == len(MATRIX)
        verdict = ("PASS" if aggregate_pass
                   else ("INVALID" if valid != len(MATRIX) else "FAIL"))

        receipt = {
            "schema": "v26c_j9_closed_loop_receipt.1", "stage": STAGE,
            "verdict": verdict, "aggregate_pass": aggregate_pass,
            "cells_behavioural_pass": behavioural, "cells_telemetry_valid": valid,
            "cells_total": len(MATRIX),
            "aggregate_rule": "PASS iff 6/6 behavioural PASS AND 6/6 telemetry-valid",
            "stack": {"name": getattr(stack, "name", "?"),
                      "operational": bool(getattr(stack, "operational", False)),
                      "injected": injected,
                      "note": ("an INJECTED stack is a test double: this receipt is NOT "
                               "operational evidence" if injected
                               else "the production torch/RLlib/OpenSim stack")},
            "amendment": pre["amendment"],
            "actor_before": actor_before, "actor_after": actor_after,
            "actor_unchanged": True,
            "offset_unit_correction": pre["offset_unit_correction"],
            "runtime": pre["runtime"],
            "recording_instrumentation": {
                "record_outputs": base_env["record_outputs"],
                "save_outputs_on_close": base_env["save_outputs_on_close"],
                "output_prefix": base_env["output_prefix"],
                "base_output_root": _rel(staging),
                "verified_fail_closed_before_any_env": True,
                "inherited_by_every_cell": True,
                "corrects": "the J9 defect recorded in v26c_j9_technical_failure_2026-08-26.json",
                "supersedes_operationally_not_by_edit": {
                    "v26c_j9_prereg_closed_loop.json": J9_PREREG_SHA,
                    "v26c_j9_closed_loop.py": J9_RUNNER_SHA,
                    "test_v26c_j9_closed_loop.py": J9_TEST_SHA,
                    "v26c_j9_closed_loop_authorization.json": J9_AUTHORIZATION_SHA,
                    "v26c_j9_technical_failure_2026-08-26.json": J9_FAILURE_RECORD_SHA,
                    "note": "pinned, never edited"}},
            "penetration_authority": pre["penetration_authority"],
            "gate_specification": pre["gate_specification"],
            "matrix_policy": pre["matrix_policy"],
            "reset_time_check": pre["reset_time_check"],
            "multistart_disambiguation": pre["multistart_disambiguation"],
            "cells": cells,
            "inert": {"fit_executed": False, "critic_touched": False, "ppo_updates": 0,
                      "actor_edited": False, "actor_copied": False,
                      "logstd_head_edited": False,
                      "rllib_checkpoint_loader": True,
                      "ray_cluster_started": False, "ray_workers_started": False,
                      "env_runners": False},
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False, "single_execution": True,
                        "no_autonomous_retry": True,
                        "note": "a closed-loop PASS here authorises nothing further"},
            "forbidden_here": list(FORBIDDEN_HERE),
        }
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False, default=str) + "\n",
            encoding="utf-8")

        if leaf.exists() or leaf.is_symlink():
            raise J9Error(f"the leaf appeared while staging; refusing to clobber: {leaf}")
        os.rename(staging, leaf)
        staging_created = None
    except BaseException:
        if staging_created is not None and staging_created.name == STAGING_NAME \
                and staging_created.is_dir() and not staging_created.is_symlink():
            shutil.rmtree(staging_created, ignore_errors=True)
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass
            lock_owned = None
        if parent_created is not None:
            try:
                parent_created.rmdir()
            except OSError:
                pass
        raise
    finally:
        if lock_owned is not None:
            try:
                lock_owned.unlink()
            except OSError:
                pass

    return {"verdict": verdict, "stage": STAGE, "leaf": _rel(leaf),
            "aggregate_pass": aggregate_pass,
            "cells": [{"id": c["id"], "mode": c["mode"], "seed": c["seed"],
                       "offset_s": c["episode_start_offset_s"], "verdict": c["verdict"],
                       "failed": c["gate"]["failed"],
                       "max_penetration_m": c["penetration"]["max_penetration_m"],
                       "band": c["penetration"]["band"]} for c in cells],
            "receipt_sha256": _sha_file(leaf / RECEIPT_NAME),
            "lock_released": not lock_path.exists(), "staging_removed": True,
            "authoritative": OUTPUT_ROOT_OVERRIDE is None,
            "outcome": {"deployable": False, "promotion": "NONE",
                        "next_stage_authorized": False}}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C J9 closed-loop qualification")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--run", action="store_true")
    p.add_argument("--authorized-stage", default=None)
    p.add_argument("--out", default=None, help="must be exactly the authorised leaf")
    p.add_argument("--no-progress", action="store_true")
    a = p.parse_args(argv)
    if a.run:
        r = run_matrix(a.out, a.authorized_stage, progress=not a.no_progress)
        print(json.dumps(r, indent=2, default=str))
        return 0 if r["verdict"] == "PASS" else 1
    if a.out is not None:
        raise J9Error("--out is meaningless without --run; the preflight writes nothing")
    r = preflight()
    print(json.dumps(r, indent=2, default=str))
    return 0 if r["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
