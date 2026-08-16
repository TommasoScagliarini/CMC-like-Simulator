"""Replay the frozen V7 raw V25 journal through V26 without reopening V7."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import uuid
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
VALIDATION_ROOT = REPO_ROOT / "validation"
for root in (REPO_ROOT, TRAJECTORY_ROOT, VALIDATION_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from validate_binary_phase_fsm_v26_development import (  # noqa: E402
    MODES,
    _run_mode,
)


JOURNAL_PATH = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-06_h0_primary_split_v7_v25_early_contact_diagnostic"
    / "deterministic_offset_nominal_shadow_diagnostic"
    / "v25_raw_journal.json"
)
JOURNAL_SHA256 = (
    "7741f9a3e47e3d57d8c85f80a79e8ac110be0c47b1b114f0c13c8a968cf44176"
)
V7_FAILURE_PATH = JOURNAL_PATH.with_name("failure.json")
V7_FAILURE_SHA256 = (
    "9095e76c2b963a3ffea7e51aaf3149ea2d2facc7910234731c4a479ffec3a4d9"
)
EXPECTED_EVENTS = (
    ("toe_off", 15.111870983804456),
    ("heel_strike", 15.624870983804172),
    ("toe_off", 16.632870983804736),
    ("heel_strike", 17.151870983805370),
    ("toe_off", 18.282870983806752),
    ("heel_strike", 18.683870983807243),
)
FORBIDDEN_V20_TOE_ONLY_PAIR = (
    ("heel_strike", 18.515870983807037),
    ("toe_off", 18.579870983807115),
)
DEFAULT_RECEIPT = VALIDATION_ROOT / "binary_phase_fsm_v26_v7_replay_receipt.json"


class V26V7ReplayError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay() -> dict[str, Any]:
    if _sha256(JOURNAL_PATH) != JOURNAL_SHA256:
        raise V26V7ReplayError("V7 raw journal drifted")
    if _sha256(V7_FAILURE_PATH) != V7_FAILURE_SHA256:
        raise V26V7ReplayError("V7 terminal failure record drifted")
    journal = json.loads(JOURNAL_PATH.read_text(encoding="utf-8"))
    baseline = journal.get("baseline_raw_sample")
    raw_samples = journal.get("raw_samples")
    if not isinstance(baseline, Mapping) or not isinstance(raw_samples, list):
        raise V26V7ReplayError("V7 journal structure is malformed")
    if len(raw_samples) != 5000 or len(journal.get("policy_steps", [])) != 500:
        raise V26V7ReplayError("V7 journal is incomplete")
    rows = [dict(baseline), *[dict(item) for item in raw_samples]]
    trace = {
        "time_s": np.asarray([item["time_s"] for item in rows], dtype=float),
        "heel": np.asarray(
            [item["left_heel_contact"] for item in rows], dtype=bool
        ),
        "toe": np.asarray(
            [item["left_toe_contact"] for item in rows], dtype=bool
        ),
    }
    if not np.all(np.isfinite(trace["time_s"])) or not np.all(
        np.diff(trace["time_s"]) > 0.0
    ):
        raise V26V7ReplayError("V7 sample time is malformed")
    mode_results = {mode: _run_mode(trace, mode) for mode in MODES}
    sequential = mode_results[MODES[0]]
    batched = mode_results[MODES[1]]
    observed = tuple(
        (str(event["event"]), float(event["event_time_s"]))
        for event in sequential["events"]
    )
    exact_events = len(observed) == len(EXPECTED_EVENTS) and all(
        name == expected_name and abs(time_s - expected_time) <= 1e-12
        for (name, time_s), (expected_name, expected_time) in zip(
            observed, EXPECTED_EVENTS
        )
    )
    forbidden_absent = all(
        all(
            name != forbidden_name or abs(time_s - forbidden_time) > 1e-12
            for name, time_s in observed
        )
        for forbidden_name, forbidden_time in FORBIDDEN_V20_TOE_ONLY_PAIR
    )
    flights = [
        observed[index + 1][1] - observed[index][1]
        for index in range(len(observed) - 1)
        if observed[index][0] == "toe_off"
        and observed[index + 1][0] == "heel_strike"
    ]
    parity = {
        key: sequential[key] == batched[key]
        for key in (
            "events",
            "contact_state_transitions",
            "candidate_cancellations",
            "boundary_snapshots_sha256",
            "final_payload",
        )
    }
    parity["pass"] = all(parity.values())
    passed = bool(
        exact_events
        and forbidden_absent
        and sequential["final_payload"]["cycle_count"] == 2
        and flights
        and min(flights) >= 0.25 - 1e-12
        and parity["pass"]
    )
    result = {
        "schema_version": 26,
        "analysis_id": "V26_POSTMORTEM_REPLAY_OF_FROZEN_V7_RAW_JOURNAL",
        "status": "V26_V7_REPLAY_PASS" if passed else "V26_V7_REPLAY_FAIL",
        "pass": passed,
        "historical_v7_status_preserved": "TERMINAL_FAIL_NO_REINTERPRETATION",
        "journal_completeness": {
            "policy_steps": 500,
            "raw_samples": 5000,
            "complete": True,
        },
        "observed_events": [
            {"event": name, "event_time_s": time_s}
            for name, time_s in observed
        ],
        "expected_events_exact": exact_events,
        "forbidden_v20_toe_only_pair_absent": forbidden_absent,
        "complete_cycle_count": sequential["final_payload"]["cycle_count"],
        "functional_flights_s": flights,
        "minimum_functional_flight_s": min(flights),
        "actor_min_swing_guard_s": 0.25,
        "scalar_batch_parity": parity,
        "data_access": {
            "development_journal_reused": True,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "simulation_rerun": False,
            "ppo_updates": 0,
        },
        "sources": {
            "v7_raw_journal": {
                "path": JOURNAL_PATH.relative_to(REPO_ROOT).as_posix(),
                "sha256": JOURNAL_SHA256,
            },
            "v7_terminal_failure": {
                "path": V7_FAILURE_PATH.relative_to(REPO_ROOT).as_posix(),
                "sha256": V7_FAILURE_SHA256,
            },
            "v26_fsm": {
                "path": "Trajectory Generator/binary_phase_fsm_v26.py",
                "sha256": _sha256(
                    TRAJECTORY_ROOT / "binary_phase_fsm_v26.py"
                ),
            },
            "replay_script": {
                "path": Path(__file__).relative_to(REPO_ROOT).as_posix(),
                "sha256": _sha256(Path(__file__)),
            },
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def _write_no_clobber(path: Path, result: Mapping[str, Any]) -> None:
    if path.exists():
        raise V26V7ReplayError(f"refusing to overwrite receipt: {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(dict(result), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-receipt", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = replay()
        if args.write_receipt:
            _write_no_clobber(DEFAULT_RECEIPT, result)
    except Exception as exc:
        print(
            f"V26/V7 replay failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
