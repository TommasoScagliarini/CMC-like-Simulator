"""V26C penetration contract - fail-closed loader, pure evaluator, offline reconciliation.

THE CONTRACT
    0.020 m  SOFT DIAGNOSTIC              never binding; a sample or max is "above" iff > 0.020
    0.025 m  JULY LEGACY HARD REFERENCE   comparative only; July passed iff < 0.025, so the
                                          breach diagnostic is >= 0.025
    0.028 m  HARD BINDING                 the ONLY blocking threshold: pass iff <= 0.028,
                                          FAIL/stop iff > 0.028

WHAT THIS MODULE IS NOT
    It runs nothing. No environment, no rollout, no fit, no training. It reads the pinned JSON
    contract, evaluates penetration series or maxima, and - under --reconcile - re-reads the
    pinned J1/J3/J5-r2 traces to produce an INFORMATIONAL reconciliation record.

WHAT IT NEVER DOES
    It never edits a receipt, a trace, a run leaf or an earlier amendment. Historical verdicts
    stand. Nothing here confers deployability, promotion or authorisation of a next stage.

Cross-platform: pathlib only, no shell, no os-specific path handling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]

CONTRACT_FILE = HERE / "v26c_penetration_contract_2026-08-26.json"
PIN_CONTRACT = "95a47d5317be4b1a2f55084fcb3548e479c2333093adc29b4205ad150d48e461"

RECONCILIATION_NAME = "v26c_penetration_contract_reconciliation.json"

# The three bands, restated here ONLY so the module can verify that the JSON it loaded is the
# contract it was written against. The JSON remains authoritative; a disagreement fails closed.
EXPECTED_BANDS = {"soft_diagnostic": 0.020,
                  "july_legacy_hard_reference": 0.025,
                  "hard_binding": 0.028}

BAND_WITHIN_SOFT = "within_soft"
BAND_ABOVE_SOFT = "above_soft_below_july_legacy"
BAND_LEGACY = "july_legacy_breach_within_hard"
BAND_ABOVE_HARD = "above_hard"


class ContractError(RuntimeError):
    pass


def _sha_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _rel(p: Path) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO))
    except ValueError:
        return str(Path(p).resolve())


# ================================================================ loader =========================

def load_contract(path: Path | None = None, *, verify_pins: bool = True) -> dict[str, Any]:
    """Load the contract JSON, fail-closed, and verify every artefact it pins."""
    path = Path(path) if path is not None else CONTRACT_FILE
    if not path.is_file():
        raise ContractError(f"the penetration contract is missing: {path}")
    digest = _sha_file(path)
    if PIN_CONTRACT != "PENDING" and digest != PIN_CONTRACT:
        raise ContractError(f"the penetration contract changed: {digest} != {PIN_CONTRACT}")
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ContractError(f"the penetration contract is not valid JSON: {exc}") from exc
    if data.get("schema") != "v26c_penetration_contract.1":
        raise ContractError(f"unexpected contract schema {data.get('schema')!r}")

    bands = data.get("bands")
    if not isinstance(bands, Mapping):
        raise ContractError("the contract declares no bands")
    for name, expected in EXPECTED_BANDS.items():
        band = bands.get(name)
        if not isinstance(band, Mapping):
            raise ContractError(f"the contract declares no band {name!r}")
        value = band.get("value_m")
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ContractError(f"band {name!r} has a non-finite value: {value!r}")
        if float(value) != expected:
            raise ContractError(f"band {name!r} is {value!r}, expected {expected!r}")
    if not (bands["soft_diagnostic"]["value_m"] < bands["july_legacy_hard_reference"]["value_m"]
            < bands["hard_binding"]["value_m"]):
        raise ContractError("the three bands are not strictly ordered")
    if bands["soft_diagnostic"].get("binding") is not False \
            or bands["july_legacy_hard_reference"].get("binding") is not False:
        raise ContractError("the soft and July legacy bands must be declared non-binding")
    if bands["hard_binding"].get("binding") is not True:
        raise ContractError("the hard band must be declared binding")
    dnd = data.get("does_not_do")
    if not isinstance(dnd, Mapping) or dnd.get("confers_no_deployability") is not True \
            or dnd.get("confers_no_promotion") is not True \
            or dnd.get("authorises_no_next_stage") is not True:
        raise ContractError("the contract must declare that it confers nothing")

    pins = data.get("pinned_artefacts_sha256")
    if not isinstance(pins, Mapping) or not pins:
        raise ContractError("the contract pins no artefacts")
    verified: dict[str, str] = {}
    if verify_pins:
        for rel, pin in pins.items():
            target = HERE / rel
            if not target.is_file():
                raise ContractError(f"the contract pins {rel}, which is missing")
            got = _sha_file(target)
            if got != pin:
                raise ContractError(f"the pinned artefact {rel} changed: {got} != {pin}")
            verified[rel] = got
    return {"path": _rel(path), "sha256": digest, "data": data,
            "soft_m": float(bands["soft_diagnostic"]["value_m"]),
            "july_legacy_m": float(bands["july_legacy_hard_reference"]["value_m"]),
            "hard_m": float(bands["hard_binding"]["value_m"]),
            "pins_verified": verified, "pins_verified_count": len(verified)}


# ================================================================ evaluator ======================

def _finite_nonnegative(values: Iterable[Any], label: str) -> np.ndarray:
    arr = np.asarray(list(values), dtype=np.float64)
    if arr.ndim != 1:
        raise ContractError(f"{label} must be one-dimensional, got shape {arr.shape}")
    if arr.size == 0:
        raise ContractError(f"{label} is empty")
    if not bool(np.all(np.isfinite(arr))):
        bad = int(np.flatnonzero(~np.isfinite(arr))[0])
        raise ContractError(f"{label} holds a non-finite value at index {bad}: {arr[bad]!r}")
    if bool(np.any(arr < 0.0)):
        bad = int(np.flatnonzero(arr < 0.0)[0])
        raise ContractError(f"{label} holds a negative value at index {bad}: {arr[bad]!r}")
    return arr


def classify(max_penetration_m: float, contract: Mapping[str, Any]) -> dict[str, Any]:
    """The band and the flags for a single maximum. Pure: no I/O, no state."""
    value = float(max_penetration_m)
    if not math.isfinite(value):
        raise ContractError(f"the maximum penetration is not finite: {max_penetration_m!r}")
    if value < 0.0:
        raise ContractError(f"the maximum penetration is negative: {max_penetration_m!r}")
    soft = float(contract["soft_m"])
    legacy = float(contract["july_legacy_m"])
    hard = float(contract["hard_m"])
    above_soft = value > soft
    legacy_breach = value >= legacy
    july_legacy_pass = value < legacy
    above_hard = value > hard
    if above_hard:
        band = BAND_ABOVE_HARD
    elif legacy_breach:
        band = BAND_LEGACY
    elif above_soft:
        band = BAND_ABOVE_SOFT
    else:
        band = BAND_WITHIN_SOFT
    return {
        "max_penetration_m": value,
        "band": band,
        "flags": {
            "above_soft_diagnostic": bool(above_soft),
            "july_legacy_breach": bool(legacy_breach),
            "july_legacy_historical_pass": bool(july_legacy_pass),
            "above_hard_binding": bool(above_hard),
        },
        "binding_pass": bool(not above_hard),
        "binding_verdict": "PASS" if not above_hard else "FAIL",
        "thresholds_m": {"soft_diagnostic": soft, "july_legacy": legacy, "hard_binding": hard},
        "semantics": {
            "above_soft_iff": "> 0.020",
            "july_legacy_breach_iff": ">= 0.025",
            "july_legacy_historical_pass_iff": "< 0.025",
            "binding_pass_iff": "<= 0.028",
            "binding_fail_iff": "> 0.028",
        },
    }


def evaluate_series(series: Sequence[Any], contract: Mapping[str, Any], *,
                    label: str = "penetration series") -> dict[str, Any]:
    """Counts, flags, band and binding verdict for a full per-sample series."""
    arr = _finite_nonnegative(series, label)
    soft = float(contract["soft_m"])
    legacy = float(contract["july_legacy_m"])
    hard = float(contract["hard_m"])
    result = classify(float(arr.max()), contract)
    result.update({
        "samples": int(arr.size),
        "counts": {
            "above_soft_diagnostic": int(np.sum(arr > soft)),
            "at_or_above_july_legacy": int(np.sum(arr >= legacy)),
            "above_hard_binding": int(np.sum(arr > hard)),
        },
        "fractions": {
            "above_soft_diagnostic": float(np.mean(arr > soft)),
            "at_or_above_july_legacy": float(np.mean(arr >= legacy)),
            "above_hard_binding": float(np.mean(arr > hard)),
        },
        "mean_penetration_m": float(arr.mean()),
        "argmax_index_1based": int(arr.argmax()) + 1,
        "counting_conventions": {
            "above_soft_diagnostic": "strictly > 0.020",
            "at_or_above_july_legacy": ">= 0.025",
            "above_hard_binding": "strictly > 0.028",
        },
    })
    return result


# ================================================================ reconciliation =================

RECONCILED_RUNS: dict[str, dict[str, str]] = {
    "J1": {
        "stage": "V26C-J1-TEACHER-COLLECTION",
        "receipt": "j1_runs/j1_nominal_v26c_2026-08-26_r1/v26c_j1_collection_receipt.json",
        "trace": "j1_runs/j1_nominal_v26c_2026-08-26_r1/teacher_trace.json",
        "penetration_field": "reward_terms.grf_penetration_m",
    },
    "J3": {
        "stage": "V26C-J3-CLOSED-LOOP",
        "receipt": "j3_runs/j3_base_v26c_2026-08-26_r1/v26c_j3_closed_loop_receipt.json",
        "trace": "j3_runs/j3_base_v26c_2026-08-26_r1/j3_trace.json",
        "penetration_field": "reward_terms.grf_penetration_m",
    },
    "J5-r2": {
        "stage": "V26C-J5-REVALIDATION",
        "receipt": ("j5_runs/j5_revalidation_v26c_2026-08-26_r2/"
                    "v26c_j5_revalidation_receipt.json"),
        "trace": "j5_runs/j5_revalidation_v26c_2026-08-26_r2/j5_trace.json",
        "penetration_field": "reward_terms.grf_penetration_m",
    },
}
PENETRATION_CHECK_KEY = "max_penetration_m"


def _series_from_trace(path: Path, label: str) -> list[float]:
    rows = json.loads(Path(path).read_text())
    if not isinstance(rows, list) or not rows:
        raise ContractError(f"{label}: the trace is not a non-empty list")
    out: list[float] = []
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            raise ContractError(f"{label}: row {index} is not a mapping")
        terms = row.get("reward_terms")
        if not isinstance(terms, Mapping) or "grf_penetration_m" not in terms:
            raise ContractError(f"{label}: row {index} has no reward_terms.grf_penetration_m")
        out.append(float(terms["grf_penetration_m"]))
    return out


def _behavioural_under_new_contract(receipt: Mapping[str, Any], binding_pass: bool
                                    ) -> dict[str, Any]:
    """Only claim a behavioural outcome when the OLD checks demonstrate it. Otherwise abstain."""
    gate = receipt.get("gate")
    if not isinstance(gate, Mapping) or not isinstance(gate.get("checks"), Mapping):
        return {"demonstrable": False,
                "why": "the receipt records no per-criterion gate checks, so no behavioural "
                       "outcome can be derived without re-running"}
    checks = dict(gate["checks"])
    if PENETRATION_CHECK_KEY not in checks:
        return {"demonstrable": False,
                "why": f"the receipt's checks do not include {PENETRATION_CHECK_KEY}"}
    others_failed = sorted(k for k, v in checks.items()
                           if k != PENETRATION_CHECK_KEY and not v)
    if others_failed:
        return {"demonstrable": True, "behavioural_pass_under_new_contract": False,
                "other_failed_checks": others_failed,
                "why": "criteria other than penetration also failed; the new contract changes "
                       "only the penetration band and cannot rescue them"}
    return {"demonstrable": True,
            "behavioural_pass_under_new_contract": bool(binding_pass),
            "other_failed_checks": [],
            "why": "every non-penetration criterion PASSED in the original receipt, so under the "
                   "new contract the behavioural outcome follows from the penetration band alone"}


def _field_statement(runs: Mapping[str, Any], field: str, expected: Any) -> str:
    """State per-run what the ORIGINAL receipts carried. Absent is never reported as false."""
    carried: list[str] = []
    absent: list[str] = []
    other: list[str] = []
    for name in sorted(runs):
        value = runs[name]["unchanged_by_this_record"][field]
        if isinstance(value, str) and value.startswith("ABSENT"):
            absent.append(name)
        elif value == expected:
            carried.append(name)
        else:
            other.append(f"{name}={value!r}")
    parts = ["UNCHANGED."]
    if carried:
        parts.append(f"Recorded as {expected!r} in: {', '.join(carried)}.")
    if absent:
        parts.append(f"ABSENT - never declared by the receipt schema - in: {', '.join(absent)}. "
                     "Absent is NOT the same as an explicit false.")
    if other:
        parts.append(f"Other recorded values: {', '.join(other)}.")
    return " ".join(parts)


def reconcile(contract: Mapping[str, Any]) -> dict[str, Any]:
    """Re-evaluate the pinned runs' PENETRATION component under the new contract. Informational."""
    runs: dict[str, Any] = {}
    for name, spec in RECONCILED_RUNS.items():
        receipt_path = HERE / spec["receipt"]
        trace_path = HERE / spec["trace"]
        for p in (receipt_path, trace_path):
            if not p.is_file():
                raise ContractError(f"{name}: the pinned artefact is missing: {p}")
        pins = contract["data"]["pinned_artefacts_sha256"]
        for rel in (spec["receipt"], spec["trace"]):
            if rel not in pins:
                raise ContractError(f"{name}: {rel} is not pinned by the contract")
            got = _sha_file(HERE / rel)
            if got != pins[rel]:
                raise ContractError(f"{name}: {rel} changed: {got} != {pins[rel]}")
        receipt = json.loads(receipt_path.read_text())
        series = _series_from_trace(trace_path, name)
        evaluated = evaluate_series(series, contract, label=f"{name} penetration")
        recorded_max = receipt.get("summary", {}).get("max_penetration_m")
        if recorded_max is None:
            raise ContractError(f"{name}: the receipt records no summary.max_penetration_m")
        if not math.isclose(float(recorded_max), evaluated["max_penetration_m"],
                            rel_tol=0.0, abs_tol=1e-15):
            raise ContractError(f"{name}: the trace maximum {evaluated['max_penetration_m']!r} "
                                f"disagrees with the receipt {recorded_max!r}")
        ABSENT = "ABSENT - this receipt schema declares no such field"

        def carried(key: str) -> Any:
            """Report the receipt's own value, or say plainly that the field is absent.

            The J1 collection receipt has no deployability/promotion fields at all; inventing
            False for it would assert something the artefact never said.
            """
            return receipt[key] if key in receipt else ABSENT

        historical = {
            "verdict": receipt.get("verdict"),
            "failed": (receipt.get("gate") or {}).get("failed"),
            "max_penetration_m": float(recorded_max),
            "deployable": carried("deployable"),
            "promotion": carried("promotion"),
            "next_stage_authorized": carried("next_stage_authorized"),
            "status": "UNCHANGED - this record never rewrites it",
        }
        for key in ("deployable", "next_stage_authorized"):
            if key in receipt and receipt[key] is not False:
                raise ContractError(f"{name}: the receipt claims {key}={receipt[key]!r}")
        if "promotion" in receipt and receipt["promotion"] != "NONE":
            raise ContractError(f"{name}: the receipt claims promotion={receipt['promotion']!r}")
        runs[name] = {
            "stage": spec["stage"],
            "receipt": spec["receipt"], "receipt_sha256": pins[spec["receipt"]],
            "trace": spec["trace"], "trace_sha256": pins[spec["trace"]],
            "historical": historical,
            "penetration_under_new_contract": evaluated,
            "behavioural_under_new_contract": _behavioural_under_new_contract(
                receipt, evaluated["binding_pass"]),
            "unchanged_by_this_record": {
                "deployable": carried("deployable"),
                "promotion": carried("promotion"),
                "next_stage_authorized": carried("next_stage_authorized"),
                "note": "reported exactly as the original receipt carries them; an absent field "
                        "is reported as absent, never invented",
            },
        }
    return {
        "schema": "v26c_penetration_reconciliation.1",
        "kind": "INFORMATIONAL RECONCILIATION - NOT A VERDICT, NOT A PROMOTION",
        "generated_by": f"{Path(__file__).name} --reconcile",
        "contract": {"file": contract["path"], "sha256": contract["sha256"],
                     "thresholds_m": {"soft_diagnostic": contract["soft_m"],
                                      "july_legacy": contract["july_legacy_m"],
                                      "hard_binding": contract["hard_m"]}},
        "pins_verified_count": contract["pins_verified_count"],
        "runs": runs,
        "statements": {
            "historical_verdicts": "UNCHANGED. Every original verdict stands exactly as recorded.",
            "deployability": _field_statement(runs, "deployable", False),
            "promotion": _field_statement(runs, "promotion", "NONE"),
            "next_stage": _field_statement(runs, "next_stage_authorized", False),
            "authorises": "NOTHING. This record authorises no rollout, fit, training or stage.",
            "absent_is_not_false": "a field the original receipt never carried is reported as "
                                   "ABSENT. Absent means NOT CONFERRED BY THAT SCHEMA; it is not "
                                   "the same claim as an explicit false.",
        },
        # DETERMINISTIC: no wall-clock. Two generations from the same inputs are byte-identical.
        "contract_effective_date_utc": contract["data"].get("date_utc"),
        "determinism": {
            "wall_clock_free": True,
            "note": "this record carries no generation timestamp. It is a pure function of the "
                    "pinned contract and the pinned artefacts, so it can be regenerated and "
                    "compared byte-for-byte.",
        },
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26C penetration contract")
    p.add_argument("--preflight", action="store_true")
    p.add_argument("--reconcile", action="store_true")
    p.add_argument("--out", default=None, help="explicit output path for --reconcile (no-clobber)")
    a = p.parse_args(argv)
    contract = load_contract()
    if a.reconcile:
        if not a.out:
            raise ContractError("--reconcile requires an explicit --out path")
        out = Path(a.out)
        if out.exists():
            raise ContractError(f"no-clobber: {out} already exists")
        record = reconcile(contract)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(record, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                       encoding="utf-8")
        print(json.dumps({"written": _rel(out), "runs": sorted(record["runs"]),
                          "sha256": _sha_file(out)}, indent=2))
        return 0
    print(json.dumps({"contract": contract["path"], "sha256": contract["sha256"],
                      "thresholds_m": {"soft_diagnostic": contract["soft_m"],
                                       "july_legacy": contract["july_legacy_m"],
                                       "hard_binding": contract["hard_m"]},
                      "pins_verified": contract["pins_verified_count"],
                      "binding_thresholds": 1,
                      "runs_nothing": True}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
