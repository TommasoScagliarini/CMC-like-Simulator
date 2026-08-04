from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

import diagnose_two_sensor_v15_routing as v15


def _edge(sensor: str, edge: str, onset: float, confirmed: float) -> v15.SensorEdge:
    return v15.SensorEdge(sensor, edge, onset, confirmed)


def test_raw_rising_crossings_reports_every_raw_episode() -> None:
    times = np.arange(7, dtype=float) * 0.01
    force = np.array([0.0, 0.6, 0.7, 0.1, 0.8, 0.9, 0.0])
    assert v15.raw_rising_crossings(times, force, 0.5) == [0.01, 0.04]


def test_debounce_matches_production_onset_and_confirmation_semantics() -> None:
    times = np.arange(8, dtype=float) * 0.01
    force = np.array([0.0, 0.6, 0.7, 0.8, 0.9, 0.1, 0.1, 0.1])
    edges = v15.debounce_sensor_edges(
        times,
        force,
        sensor="heel",
        on_threshold_n=0.5,
        off_threshold_n=0.25,
        dwell_s=0.02,
    )
    assert edges == [
        _edge("heel", "contact_on", 0.01, 0.03),
        _edge("heel", "contact_off", 0.05, 0.07),
    ]


def test_debounce_rejects_a_transient_crossing() -> None:
    times = np.arange(6, dtype=float) * 0.01
    force = np.array([0.0, 0.6, 0.1, 0.0, 0.0, 0.0])
    assert not v15.debounce_sensor_edges(
        times,
        force,
        sensor="heel",
        on_threshold_n=0.5,
        off_threshold_n=0.25,
        dwell_s=0.02,
    )


def test_routing_separates_heel_only_from_first_stable_regional() -> None:
    heel = [
        _edge("heel", "contact_on", 0.13, 0.16),
        _edge("heel", "contact_off", 0.50, 0.53),
    ]
    forefoot = [
        _edge("forefoot", "contact_on", 0.07, 0.10),
        _edge("forefoot", "contact_off", 0.47, 0.50),
    ]
    heel_only = v15.route_hs_events(heel, forefoot, routing_mode="heel_only")
    first = v15.route_hs_events(
        heel, forefoot, routing_mode="first_stable_regional"
    )
    assert [(item.source_sensor, item.confirmed_s) for item in heel_only] == [
        ("heel", 0.16)
    ]
    assert [(item.source_sensor, item.confirmed_s) for item in first] == [
        ("forefoot", 0.10)
    ]


def test_router_rearms_only_after_both_contacts_are_clear() -> None:
    heel = [
        _edge("heel", "contact_on", 0.10, 0.13),
        _edge("heel", "contact_off", 0.40, 0.43),
        _edge("heel", "contact_on", 1.10, 1.13),
    ]
    forefoot = [
        _edge("forefoot", "contact_on", 0.20, 0.23),
        _edge("forefoot", "contact_off", 0.50, 0.53),
    ]
    routed = v15.route_hs_events(
        heel, forefoot, routing_mode="first_stable_regional"
    )
    assert [item.confirmed_s for item in routed] == [0.13, 1.13]


def _reference() -> dict:
    return {
        "trial_id": "02",
        "plateau_index": 1,
        "speed_mps": 0.55,
        "plateau_interval_s": [0.5, 2.5],
        "events": {
            "heel_strike": np.array([1.0, 2.0]),
            "toe_off": np.array([1.5]),
        },
    }


def test_feasibility_requires_exact_counts_and_50ms_timing() -> None:
    passing = [
        v15.RoutedHS("heel_only", "heel", 1.01, 1.04),
        v15.RoutedHS("heel_only", "heel", 2.01, 2.04),
    ]
    row = v15.feasibility_row(
        _reference(),
        passing,
        trial_id="02",
        cadence="runtime_10ms",
        sample_dt_s=0.01,
        threshold_n=0.5,
        dwell_s=0.03,
        routing_mode="heel_only",
        tolerance_s=0.05,
    )
    assert row["exact_count_and_50ms_timing_feasible"] is True
    assert row["diagnostic_failed_reference_count"] == 0

    failing = [replace(passing[0], confirmed_s=1.06), passing[1]]
    row = v15.feasibility_row(
        _reference(),
        failing,
        trial_id="02",
        cadence="runtime_10ms",
        sample_dt_s=0.01,
        threshold_n=0.5,
        dwell_s=0.03,
        routing_mode="heel_only",
        tolerance_s=0.05,
    )
    assert row["exact_count_and_50ms_timing_feasible"] is False
    assert row["diagnostic_failed_reference_count"] == 1
    assert row["diagnostic_confirmed_error_max_s"] == pytest.approx(0.06)


def test_hy_inverse_contact_law_reaches_the_requested_threshold() -> None:
    profile_path = v15.REPO_ROOT / (
        "validation/experimental_detector_profiles/"
        "two_sensor_v13_development_toe_down_p0p75mm_heel_x_p3p5mm.json"
    )
    profile = v15.v1.load_online_grf_profile(profile_path, required_sides=("left",))
    heel = v15.v1._left_sensor_spheres(profile)["left_heel"]
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    center = origin + (heel.radius + 0.001) * normal
    samples = {
        "centers": {heel.name: np.repeat(center[None, :], 3, axis=0)},
        "velocities": {heel.name: np.zeros((3, 3), dtype=float)},
    }
    offsets = v15.required_ground_normal_offsets_m(
        profile, heel, samples, threshold_n=0.5
    )
    heel_profile = replace(profile, spheres=(heel,))
    wrench = v15.v1._calculate_wrench(
        heel_profile, samples, ground_offset=float(offsets[0]) + 1e-12
    )
    assert np.all(wrench["left"]["normal_force"] >= 0.5 - 1e-8)


def test_minimum_hy_offset_uses_a_complete_causal_dwell_window() -> None:
    times = np.arange(0.80, 1.071, 0.01)
    required = np.full(times.shape, 0.004)
    required[(times >= 1.00) & (times <= 1.03)] = 0.00075
    result = v15.minimum_hy_offset_for_reference(
        times,
        required,
        reference_hs_s=1.0,
        dwell_s=0.03,
        maximum_confirmed_error_s=0.05,
        search_lookback_s=0.15,
    )
    assert result["feasible_window_found"] is True
    assert result["equivalent_plantar_ground_normal_offset_mm"] == pytest.approx(
        0.75
    )
    assert result["window_confirmed_s"] <= 1.05 + 1e-12


def test_hy_reporting_checkpoints_include_requested_local_values() -> None:
    values = v15._hy_checkpoint_columns(0.75, [0.0, 0.5, 0.75, 1.0, 2.0])
    assert values == {
        "signal_feasible_at_hy_0p0mm": False,
        "signal_feasible_at_hy_0p5mm": False,
        "signal_feasible_at_hy_0p75mm": True,
        "signal_feasible_at_hy_1p0mm": True,
        "signal_feasible_at_hy_2p0mm": True,
    }


def test_frozen_protocol_and_parent_provenance_are_valid() -> None:
    protocol = v15.load_and_validate_protocol()
    assert protocol["protocol_id"] == v15.PROTOCOL_ID
    assert tuple(protocol["split"]["DEVELOPMENT"]) == v15.ALLOWED_TRIALS
    parent = v15.validate_parent_provenance(protocol)
    assert parent["status"] == "PASS_V14_2_DEVELOPMENT_ONLY_PARENT_VERIFIED"


@pytest.mark.parametrize("trial_id", v15.FORBIDDEN_TRIALS)
def test_preprocessing_has_no_route_to_nondevelopment_trials(trial_id: str) -> None:
    protocol = v15.load_and_validate_protocol()
    with pytest.raises(v15.ProtocolError, match="access forbidden"):
        v15._validate_preprocessing_trial(protocol, trial_id)


def test_v15_preprocessing_reuses_only_locked_parent_products() -> None:
    protocol = v15.load_and_validate_protocol()
    for trial_id in v15.ALLOWED_TRIALS:
        artifacts = v15._validate_preprocessing_trial(protocol, trial_id)
        assert artifacts.lock["dataset_ik_used_downstream"] is False
        assert artifacts.setup.kinematics_file.name == f"treadmill_{trial_id}_01_ik.mot"


def test_no_clobber_rejects_an_existing_or_noncanonical_destination(
    tmp_path: Path,
) -> None:
    with pytest.raises(v15.NoClobberError, match="canonical"):
        v15._preflight_no_clobber(tmp_path / "new")
    existing = v15.DEFAULT_OUTPUT_DIR
    existing.mkdir(parents=True, exist_ok=True)
    try:
        with pytest.raises(v15.NoClobberError, match="exists"):
            v15._preflight_no_clobber(existing)
    finally:
        existing.rmdir()


def test_exclusive_writers_refuse_overwrite(tmp_path: Path) -> None:
    target = tmp_path / "evidence.json"
    v15._write_json_exclusive(target, {"ok": True})
    with pytest.raises(FileExistsError):
        v15._write_json_exclusive(target, {"ok": False})


def test_read_only_cli_preflight_never_starts_canonical_run(capsys) -> None:
    assert v15.main(["--check"]) == 0
    payload = capsys.readouterr().out
    assert "PASS_V15_READ_ONLY_PREFLIGHT" in payload
    assert not v15.DEFAULT_OUTPUT_DIR.exists()
