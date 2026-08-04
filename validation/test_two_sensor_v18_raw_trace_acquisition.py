from __future__ import annotations

import csv
import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).with_name("acquire_two_sensor_v18_raw_traces.py")
SPEC = importlib.util.spec_from_file_location("v18_raw_acquisition", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
v18 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(v18)


def _valid_arrays(trial_id: str = "02") -> tuple[np.ndarray, ...]:
    times = v18.build_time_grid(trial_id)
    ramp = np.linspace(0.0, 2.0, times.size, dtype=float)
    return times, ramp, ramp[::-1].copy(), ramp * 1e-3, ramp[::-1].copy() * 1e-3


def test_protocol_is_frozen_and_hash_pinned() -> None:
    protocol = v18.load_and_validate_protocol()
    assert protocol["protocol_id"] == v18.PROTOCOL_ID
    assert protocol["split"]["DIAGNOSIS_CHALLENGE"] == ["02"]
    assert protocol["split"]["SELECTION"] == ["04"]
    assert protocol["split"]["INTERNAL_HOLDOUT"] == ["08"]
    assert protocol["semantics"]["candidate_ids"] == [
        "heel_only",
        "first_stable_regional",
        "combined_load",
    ]


@pytest.mark.parametrize(
    ("trial_id", "expected_count", "expected_last"),
    [
        ("02", 143206, 153.08),
        ("04", 143541, 156.025),
        ("08", 144201, 154.89),
    ],
)
def test_time_grid_is_absolute_unique_and_does_not_overshoot(
    trial_id: str,
    expected_count: int,
    expected_last: float,
) -> None:
    times = v18.build_time_grid(trial_id)
    assert times.size == expected_count
    assert np.all(np.diff(times) > 0.0)
    assert np.allclose(np.diff(times), 0.001, rtol=0.0, atol=1e-10)
    assert times[-1] <= expected_last + 1e-10
    assert times[0] == pytest.approx(v18.EXPECTED_INTERVALS_S[trial_id][0])


def test_unauthorized_trial_has_no_time_grid_or_sampling_path() -> None:
    for trial_id in sorted(v18.FORBIDDEN_TRIALS):
        with pytest.raises(v18.V18AcquisitionError, match="unauthorized"):
            v18.build_time_grid(trial_id)


def test_trace_validation_accepts_exact_finite_grid() -> None:
    times, heel, toe, heel_pen, toe_pen = _valid_arrays()
    result = v18.validate_trace_arrays(
        trial_id="02",
        times=times,
        heel_force_n=heel,
        toe_force_n=toe,
        heel_penetration_m=heel_pen,
        toe_penetration_m=toe_pen,
    )
    assert result["sample_count"] == 143206
    assert result["timestamps_unique_monotonic_exact_1ms"] is True
    assert result["all_finite"] is True


@pytest.mark.parametrize("fault", ["duplicate", "missing", "nan", "negative"])
def test_trace_validation_fails_closed_on_bad_samples(fault: str) -> None:
    times, heel, toe, heel_pen, toe_pen = _valid_arrays()
    if fault == "duplicate":
        times[10] = times[9]
    elif fault == "missing":
        times[10] += 0.001
    elif fault == "nan":
        heel[10] = np.nan
    elif fault == "negative":
        toe[10] = -1.0
    with pytest.raises(v18.V18AcquisitionError):
        v18.validate_trace_arrays(
            trial_id="02",
            times=times,
            heel_force_n=heel,
            toe_force_n=toe,
            heel_penetration_m=heel_pen,
            toe_penetration_m=toe_pen,
        )


def test_csv_writer_is_no_clobber_and_round_trips_columns(tmp_path: Path) -> None:
    path = tmp_path / "trace.csv"
    values = np.asarray([1.0, 1.001], dtype=float)
    v18._atomic_write_trace_csv(
        path,
        times=values,
        heel_force_n=np.asarray([0.0, 0.5]),
        toe_force_n=np.asarray([0.25, 0.0]),
        heel_penetration_m=np.asarray([0.0, 0.001]),
        toe_penetration_m=np.asarray([0.002, 0.0]),
    )
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == v18.TRACE_COLUMNS
    assert len(rows) == 2
    with pytest.raises(v18.V18NoClobberError):
        v18._atomic_write_trace_csv(
            path,
            times=values,
            heel_force_n=values,
            toe_force_n=values,
            heel_penetration_m=values,
            toe_penetration_m=values,
        )


def test_selection_sampler_rejects_internal_holdout_before_finalist_lock() -> None:
    with pytest.raises(v18.V18AcquisitionError, match="cannot sample trial 08"):
        v18._sample_trial("08", {}, object())
