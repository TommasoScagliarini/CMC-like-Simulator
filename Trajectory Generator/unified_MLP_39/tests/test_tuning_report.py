"""Reporting preserves incomplete evidence and never plots failed areas as wins."""
from copy import deepcopy
import csv
import json
from pathlib import Path

import pytest

from unified_MLP_39.reward_tuning.io import sha256, write_json
from unified_MLP_39.reward_tuning.report import build_report


def fixture(tmp_path):
    output = tmp_path / "experiment"
    config = {"_repo_root": str(tmp_path), "plot_dir": "plot/ankle5"}
    metric = {"window_s": [13.15, 17.95], "episode": {"completed": True, "duration_s": 5.,
              "valid_cycles": 2, "penetration_max_mm": 15., "end_reason": "episode_time_limit"},
              "ankle": {"positive_area_nm_s": 10.}, "knee": {"rate_rms_nm_s": 100., "hp10_rms_nm": 5., "hp20_rms_nm": 2.},
              "tracking": {joint: {"actual_rmse_deg": 4., "served_rmse_deg": 3.}
                           for joint in ("knee", "ankle")},
              "contact": {"vertical_impulse_n_s": 1000.},
              "kinematic_smoothness": {joint + "_" + channel: {"jerk_rms_rad_s3": 200., "position_hp10_rms_deg": .1}
                                        for joint in ("knee", "ankle") for channel in ("actual", "served")}}
    full = {role: deepcopy(metric) for role in ("best80", "control", "ankle_5")}
    comparison = {"candidate_role": "ankle_5", "status": "rejected_constraints", "missing_cases": [], "scenarios": {
        "nominal": {"full": full, "common_window": deepcopy(full), "comparisons": {
            "best80": [{"name": "support_impulse", "actual": 800., "bound": 900., "passed": False}]}}}}
    write_json(output / "comparison.json", comparison)
    write_json(output / "pilot/pilot_manifest.json", {"reference_reuse": {"experiment": "old_pilot"}})
    return config, output, comparison


def test_report_masks_interrupted_area_and_preserves_raw_evidence(tmp_path, monkeypatch):
    config, output, comparison = fixture(tmp_path)
    previous = deepcopy(comparison)
    del previous["candidate_role"]
    for view in ("full", "common_window"):
        metric = previous["scenarios"]["nominal"][view].pop("ankle_5")
        metric["episode"].update(completed=False, duration_s=.5, end_reason="invalid_binary_event")
        metric["ankle"]["positive_area_nm_s"] = .05
        previous["scenarios"]["nominal"][view]["ankle_10"] = metric
    previous_path = tmp_path / "previous.json"
    write_json(previous_path, previous)
    before = {path: sha256(path) for path in (output / "comparison.json", previous_path)}
    from matplotlib.axes import Axes
    actual_bar = Axes.bar
    bar_calls = []
    def bar(self, x, height, *args, **kwargs):
        bar_calls.append((kwargs.get("label"), list(height)))
        return actual_bar(self, x, height, *args, **kwargs)
    monkeypatch.setattr(Axes, "bar", bar)
    result = build_report(config, output, previous_path)
    report = Path(result["report"]).read_text(encoding="utf-8")
    assert "respinta dai vincoli" in report
    assert "otto valutazioni del pilot precedente" in report
    assert "invalid_binary_event" in report
    assert "area completa non confrontabile" in report
    assert "ankle_10 (precedente)" in report
    import math
    prior_area = next(values for role, values in bar_calls if role == "ankle_10 (precedente)")
    assert math.isnan(prior_area[0])
    with Path(result["metrics_csv"]).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    previous_row = next(row for row in rows if row["role"] == "ankle_10 (precedente)" and row["view"] == "full")
    assert previous_row["ankle.positive_area_nm_s"] == "0.05"
    assert previous_row["episode.completed"] == "False"
    assert previous_row["descriptive"] == "True"
    assert all(sha256(path) == digest for path, digest in before.items())
    assert len(result["figures"]) == 4
    assert all((tmp_path / "plot") in Path(path).parents and Path(path).is_file() for path in result["figures"])
    receipt = json.loads(Path(result["receipt"]).read_text(encoding="utf-8"))
    assert receipt["status"] == "rejected_constraints"


def test_report_empty_comparison_is_explicitly_incomplete(tmp_path):
    config, output, _ = fixture(tmp_path)
    write_json(output / "comparison.json", {"status": "incomplete", "candidate_role": "ankle_5", "scenarios": {},
                                            "missing_cases": ["ankle_5/nominal"]})
    result = build_report(config, output)
    report = Path(result["report"]).read_text(encoding="utf-8")
    assert "Analisi incompleta: nessuna selezione favorevole" in report
    assert "ankle_5/nominal" in report
    assert "n.d." in report


def test_report_enforces_root_plot_location(tmp_path):
    config, output, _ = fixture(tmp_path)
    config["plot_dir"] = "experiment/plots"
    with pytest.raises(ValueError, match="root plot"):
        build_report(config, output)
