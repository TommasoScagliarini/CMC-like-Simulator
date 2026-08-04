"""Pure tests for the cadence-independent prescribed-GRF oracle."""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import pytest

VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import build_canonical_grf_event_oracle as oracle  # noqa: E402


def _source() -> dict[str, object]:
    return {
        "external_loads": {"path": "development.xml", "sha256": "a" * 64},
        "prescribed_grf": {"path": "development.mot", "sha256": "b" * 64},
        "vertical_force_column": "ground_force1_vy",
        "vertical_axis": "+Y",
        "side": "left",
    }


def _build(force: np.ndarray) -> dict[str, object]:
    times = np.arange(force.size, dtype=float) * 0.001
    return oracle.build_canonical_ledger(
        times,
        force,
        trial_id="02",
        source=_source(),
    )


def test_first_above_and_first_at_or_below_are_exact_sample_timestamps() -> None:
    force = np.zeros(1001)
    force[100] = 20.0
    force[101:201] = 21.0
    force[201] = 20.0
    force[501:601] = 30.0
    ledger = _build(force)
    events = ledger["scientific_core"]["events"]
    assert [(item["event"], item["event_time_s"]) for item in events] == [
        ("heel_strike", 0.101),
        ("toe_off", 0.201),
        ("heel_strike", 0.501),
        ("toe_off", 0.601),
    ]
    cycles = ledger["scientific_core"]["cycles"]
    assert len(cycles) == 1
    assert cycles[0]["heel_strike_time_s"] == 0.101
    assert cycles[0]["toe_off_time_s"] == 0.201
    assert cycles[0]["next_heel_strike_time_s"] == 0.501


def test_49ms_contact_is_rejected_and_50ms_contact_is_accepted() -> None:
    force = np.zeros(501)
    force[10:59] = 100.0
    force[100:150] = 100.0
    ledger = _build(force)
    core = ledger["scientific_core"]
    assert len(core["rejected_contacts"]) == 1
    assert core["rejected_contacts"][0]["observed_duration_s"] == pytest.approx(0.049)
    assert [item["event"] for item in core["events"]] == ["heel_strike", "toe_off"]
    assert core["events"][0]["event_time_s"] == pytest.approx(0.100)
    assert core["events"][0]["persistence_confirmed_time_s"] == pytest.approx(0.150)


def test_boundaries_are_explicit_and_never_invent_an_onset_or_toe_off() -> None:
    force = np.zeros(801)
    force[:80] = 100.0
    force[300:] = 100.0
    ledger = _build(force)
    core = ledger["scientific_core"]
    assert core["boundaries"]["left"]["contact_active"] is True
    assert core["boundaries"]["right"]["contact_active"] is True
    assert core["contacts"][0]["status"] == "left_boundary_censored"
    assert all(item["event_time_s"] != 0.0 for item in core["events"])
    assert [(item["event"], item["event_time_s"]) for item in core["events"]] == [
        ("heel_strike", 0.300)
    ]


def test_open_right_boundary_does_not_assume_an_unobserved_sample_interval() -> None:
    force_49ms = np.zeros(150)
    force_49ms[100:] = 100.0
    rejected = _build(force_49ms)["scientific_core"]
    assert rejected["events"] == []
    assert rejected["rejected_contacts"][0]["observed_duration_s"] == pytest.approx(
        0.049
    )

    force_50ms = np.zeros(151)
    force_50ms[100:] = 100.0
    accepted = _build(force_50ms)["scientific_core"]
    assert [(item["event"], item["event_time_s"]) for item in accepted["events"]] == [
        ("heel_strike", 0.100)
    ]
    assert accepted["boundaries"]["right"]["contact_active"] is True


def test_window_views_reuse_complete_cycles_without_rethresholding() -> None:
    force = np.zeros(1401)
    force[100:200] = 100.0
    force[500:600] = 100.0
    force[900:1000] = 100.0
    ledger = oracle.add_window_views(
        _build(force),
        [
            {
                "view_id": "plateau_01",
                "start_s": 0.05,
                "end_s": 0.95,
                "speed_mps": 1.0,
            }
        ],
    )
    view = ledger["scientific_core"]["views"][0]
    assert view["counts"] == {"complete_cycles": 2, "heel_strike": 3, "toe_off": 2}
    assert [item["event_time_s"] for item in view["scoreable_events"]] == [
        0.1,
        0.2,
        0.5,
        0.6,
        0.9,
    ]
    assert "never threshold" in view["consumer_contract"]
    oracle.validate_ledger(ledger)


@pytest.mark.parametrize(
    "times,force",
    [
        (np.array([0.0, 0.001, 0.001]), np.zeros(3)),
        (np.array([0.0, 0.001, 0.003]), np.zeros(3)),
        (np.array([0.0, np.nan, 0.002]), np.zeros(3)),
        (np.array([0.0, 0.001, 0.002]), np.array([0.0, np.inf, 0.0])),
    ],
)
def test_nonfinite_duplicate_or_gapped_samples_fail_closed(
    times: np.ndarray,
    force: np.ndarray,
) -> None:
    with pytest.raises(oracle.OracleContractError):
        oracle.build_canonical_ledger(
            times,
            force,
            trial_id="02",
            source=_source(),
        )


def test_short_cycle_and_unauthorized_trial_fail_closed() -> None:
    force = np.zeros(501)
    force[10:70] = 100.0
    force[200:260] = 100.0
    with pytest.raises(oracle.OracleContractError, match="below"):
        _build(force)
    with pytest.raises(oracle.OracleContractError, match="not authorized"):
        oracle.build_canonical_ledger(
            np.arange(501) * 0.001,
            np.zeros(501),
            trial_id="05",
            source=_source(),
        )


def test_serialized_core_mutation_is_detected() -> None:
    force = np.zeros(801)
    force[100:200] = 100.0
    force[500:600] = 100.0
    ledger = _build(force)
    oracle.validate_ledger(ledger)
    tampered = copy.deepcopy(ledger)
    tampered["scientific_core"]["events"][0]["event_time_s"] += 0.001
    with pytest.raises(oracle.OracleContractError, match="sha256"):
        oracle.validate_ledger(tampered)
