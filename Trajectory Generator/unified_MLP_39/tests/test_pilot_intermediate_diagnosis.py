"""Nominal eligibility needs the same-update control and complete evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

from test_tuning_comparison import metrics


SCRIPT = Path(__file__).resolve().parents[1] / "validation/compare_pilot_intermediates_2026_09_28.py"
spec = importlib.util.spec_from_file_location("pilot_intermediate_diagnosis", SCRIPT)
diagnosis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnosis)


def fixture(monkeypatch, candidate="ankle_5_82"):
    base = metrics()
    base["episode"].update(start_s=12.99, stop_s=17.99, duration_s=5.)
    base["ankle"] = {"positive_area_nm_s": 10.}
    control = "control_" + candidate.rsplit("_", 1)[1]
    measurements = {role: {"full": deepcopy(base)} for role in ("best80", control, candidate)}
    measurements[candidate]["full"]["ankle"]["positive_area_nm_s"] = 8.
    datasets = {role: entry["full"] for role, entry in measurements.items()}
    monkeypatch.setattr(diagnosis, "measure", lambda dataset, *args, **kwargs: dataset)
    return candidate, datasets, measurements


def test_nominal_pass_requires_best80_and_same_iteration_control(monkeypatch):
    role, data, measured = fixture(monkeypatch)
    result = diagnosis._screen(role, data, measured, diagnosis.DEFAULTS)
    assert result["references"] == ["best80", "control_82"]
    assert result["nominal_screen_passed"]
    assert result["eligible_for_new_noise_evaluation"]
    assert result["noise_stage"] == "not_evaluated"
    measured["control_82"]["full"]["ankle"]["positive_area_nm_s"] = 7.
    result = diagnosis._screen(role, data, measured, diagnosis.DEFAULTS)
    assert not result["eligible_for_new_noise_evaluation"]
    assert result["comparisons"]["best80"]["ankle_area"]["passed"]
    assert not result["comparisons"]["control_82"]["ankle_area"]["passed"]


@pytest.mark.parametrize("failure", ["phase", "missing_contact", "short_episode", "missing_control"])
def test_missing_or_failed_evidence_cannot_qualify(monkeypatch, failure):
    role, data, measured = fixture(monkeypatch)
    if failure == "phase":
        measured[role]["full"]["tracking"]["knee"]["phase_bins"]["3"]["actual_rmse_deg"] = 8.
    elif failure == "missing_contact":
        measured[role]["full"]["contact"] = None
    elif failure == "short_episode":
        measured[role]["full"]["episode"].update(completed=False, duration_s=1., stop_s=13.99)
        measured[role]["full"]["ankle"]["positive_area_nm_s"] = .01
    else:
        del measured["control_82"]
    result = diagnosis._screen(role, data, measured, diagnosis.DEFAULTS)
    assert not result["nominal_screen_passed"]
    assert not result["eligible_for_new_noise_evaluation"]


def test_control_is_never_a_reward_candidate(monkeypatch):
    _, data, measured = fixture(monkeypatch)
    result = diagnosis._screen("control_82", data, measured, diagnosis.DEFAULTS)
    assert result["nominal_screen_passed"]
    assert not result["is_candidate"]
    assert not result["eligible_for_new_noise_evaluation"]
    assert result["references"] == ["best80"]


def test_endpoint_is_descriptive_and_not_scheduled_for_new_noise(monkeypatch):
    role, data, measured = fixture(monkeypatch, "ankle_5_84")
    result = diagnosis._screen(role, data, measured, diagnosis.DEFAULTS)
    assert result["nominal_screen_passed"]
    assert not result["eligible_for_new_noise_evaluation"]
    assert result["noise_stage"] == "previously_evaluated_endpoint"


def test_unavailable_receipt_stays_missing(tmp_path):
    result, reason = diagnosis._load_case({"output_dir": str(tmp_path)}, tmp_path, {})
    assert result is None
    assert "mancante" in reason
