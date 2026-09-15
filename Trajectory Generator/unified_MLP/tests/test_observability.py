"""Meaningful offline guards: time alignment, causal histories and held-out fits."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from unified_MLP.validation import analyze_observability as observability


def write_episode(tmp_path, name="episode", offset=1.0, steps=14, extras=False, regime=False):
    directory = tmp_path / name
    directory.mkdir()
    names = list(observability.ACTOR_FEATURES)
    if extras:
        names += [f"prescribed_left_extra_{i}" for i in range(4)]
    rows = []
    for i in range(steps):
        values = [float(i + feature / 100) for feature in range(35)]
        values[0:2] = [0.0, 0.0]
        values[16] = 1.58 if regime and i >= 7 else 0.0
        if extras:
            values += [1e9 + i] * 4
        rows.append({
            "step": i + 1, "time": offset + (i + 1) * 0.01,
            "policy_segment_times": [offset + i * 0.01, offset + (i + 1) * 0.01],
            "actor_observation_before": dict(zip(names, values)),
            "actor_observation_vector_before": values,
            "imitation_target_q": {"pros_knee_angle": 100.0 + i, "pros_ankle_angle": 200.0 + i},
            "imitation_target_qdot": {"pros_knee_angle": 1000.0 + i, "pros_ankle_angle": 2000.0 + i},
        })
    summary = {"ok": True, "steps": steps, "actor_feature_names": names,
               "episode_start_offset_s": offset}
    (directory / "rollout_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (directory / "rollout_policy_trace.json").write_text(json.dumps(rows), encoding="utf-8")
    return directory


def test_pre_target_alignment_and_identical_history_cohort(tmp_path):
    episode = observability.load_episode(write_episode(tmp_path))
    data = observability.build_dataset([episode])
    first = data["samples"][0]
    assert first["step"] == 5
    assert first["target_source_step"] == 4
    assert first["observation_time_s"] == pytest.approx(first["target_time_s"])
    assert first["history_last_time_s"] == first["observation_time_s"]
    assert first["observation_time_s"] - first["history_first_time_s"] == pytest.approx(0.04)
    np.testing.assert_array_equal(data["targets"][0], [103, 1003, 203, 2003])
    assert {len(x) for x in data["inputs"].values()} == {10}
    assert data["exclusions"][0]["missing_pre_target"] == 1
    assert data["exclusions"][0]["insufficient_past_history"] == 3


def test_history_has_no_future_rows_and_never_crosses_episodes(tmp_path):
    first = observability.load_episode(write_episode(tmp_path, "first"))
    second = observability.load_episode(write_episode(tmp_path, "second", offset=5))
    original = observability.build_dataset([first, second])
    altered = copy.deepcopy(first)
    for row in altered["rows"][5:]:
        row["observations"] = [-999.0] * 35
        row["post_target"] = [-999.0] * 4
    changed = observability.build_dataset([altered, second])
    for name in observability.MODELS:
        np.testing.assert_array_equal(original["inputs"][name][0], changed["inputs"][name][0])
    np.testing.assert_array_equal(original["targets"][0], changed["targets"][0])
    x = original["inputs"]["history175"][0].reshape(5, 35)
    np.testing.assert_array_equal(x[:, 2], np.arange(5) + 0.02)
    second_samples = [s for s in original["samples"] if s["episode_id"] == second["episode_id"]]
    assert second_samples[0]["step"] == 5


def test_appended_teacher39_is_never_an_input(tmp_path):
    episode = observability.load_episode(write_episode(tmp_path, extras=True))
    data = observability.build_dataset([episode])
    raw_path = tmp_path / "episode" / "rollout_policy_trace.json"
    rows = json.loads(raw_path.read_text())
    for row in rows:
        row["actor_observation_vector_before"][35:] = [-1e200] * 4
        for key in episode["actor_feature_names"][35:]:
            row["actor_observation_before"][key] = -1e200
    raw_path.write_text(json.dumps(rows), encoding="utf-8")
    changed = observability.build_dataset([observability.load_episode(raw_path.parent)])
    for name in observability.MODELS:
        np.testing.assert_array_equal(data["inputs"][name], changed["inputs"][name])
    np.testing.assert_array_equal(data["targets"], changed["targets"])
    assert data["inputs"]["instant35"].shape[1] == 35


def test_standardization_and_intercept_use_train_only():
    train_x = np.array([[1, 5], [2, 5], [3, 5]], dtype=float)
    train_y = np.arange(12, dtype=float).reshape(3, 4)
    _, audit = observability.ridge_predict(train_x, train_y, [[1000, 5000]])
    _, changed = observability.ridge_predict(train_x, train_y, [[-999999, 8000]])
    assert audit == changed
    np.testing.assert_array_equal(audit["train_feature_mean"], [2, 5])
    assert audit["train_feature_scale"][1] == 1
    np.testing.assert_array_equal(audit["train_target_mean_physical"], train_y.mean(axis=0))
    assert audit["target_scales"] == [1, 10, 1, 10]


def test_leave_one_start_out_keeps_episodes_out_and_zero_regime_na(tmp_path):
    paths = [write_episode(tmp_path, f"start{i}", offset=float(i)) for i in (1, 2, 3)]
    paths.append(write_episode(tmp_path, "same_start_second_checkpoint", offset=1.0))
    result = observability.analyze(paths, tmp_path / "output")
    assert result["status"] == "completed"
    for fold in result["folds"]:
        assert not set(fold["train_episodes"]) & set(fold["test_episodes"])
        if fold["heldout_start_offset_s"] == 1:
            assert len(fold["test_episodes"]) == 2
    for model in result["models"].values():
        assert model["evaluated_samples"] == 40
        assert model["metrics"]["regime"] == {"status": "not_evaluable", "samples": 0, "metrics": None}
        assert model["metrics"]["startup"]["samples"] == 40
    assert result["protocol"]["oracle"]["measured_prescribed_phase"] is False
    assert result["protocol"]["oracle"]["deployment_input"] is False
    assert len(result["analyzer_sha256"]) == 64
    assert all(len(source["source_hashes"]) == 2 for source in result["sources"])
    saved = json.loads((tmp_path / "output" / "observability.json").read_text())
    assert saved["models"] == result["models"]


def test_regime_uses_current_pre_observation_not_future_events(tmp_path):
    episode = observability.load_episode(write_episode(tmp_path, regime=True))
    samples = observability.build_dataset([episode])["samples"]
    assert [s["regime"] for s in samples[:4]] == [False, False, False, True]


def test_metrics_use_physical_units_duration_weights_and_explicit_coverage():
    result = observability._metrics(
        np.zeros((2, 4)), np.array([[1, 10, 2, 20], [3, 30, 4, 40]], dtype=float),
        [{"dt_s": 0.01, "regime": False}, {"dt_s": 0.03, "regime": True}],
    )
    knee = result["all"]["metrics"]["knee_q"]
    assert knee["rmse_rad"] == pytest.approx(np.sqrt(7))
    assert knee["rmse_deg"] == pytest.approx(np.sqrt(7) * 180 / np.pi)
    assert result["regime"]["samples"] == 1
    assert result["startup"]["metrics"]["ankle_qdot"]["rmse_rad_s"] == 20


def test_module_import_needs_no_numpy_or_simulation_dependencies():
    script = Path(observability.__file__).resolve()
    code = (
        "import importlib.util, sys; "
        "spec=importlib.util.spec_from_file_location('offline',sys.argv[1]); "
        "module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); "
        "assert not any(name in sys.modules for name in ('numpy','ray','torch','opensim'))"
    )
    subprocess.run([sys.executable, "-S", "-c", code, str(script)], check=True, capture_output=True, text=True)


@pytest.mark.parametrize("starts,steps", [(2, 14), (3, 5), (3, 0)])
def test_insufficient_conditions_or_rows_is_inconclusive(tmp_path, starts, steps):
    paths = [write_episode(tmp_path, f"start{i}", offset=float(i), steps=steps) for i in range(starts)]
    result = observability.analyze(paths, tmp_path / "output")
    assert result["status"] == "inconclusive"
    assert result["reasons"]
    assert all(m["metrics"]["all"]["metrics"] is None for m in result["models"].values())


@pytest.mark.parametrize("mutation", ["gap", "backwards", "post_mismatch", "nonfinite"])
def test_malformed_timestamps_fail_explicitly(tmp_path, mutation):
    path = write_episode(tmp_path)
    trace = path / "rollout_policy_trace.json"
    rows = json.loads(trace.read_text())
    if mutation == "gap":
        rows[4]["policy_segment_times"][0] += 0.001
    elif mutation == "backwards":
        rows[4]["policy_segment_times"] = list(reversed(rows[4]["policy_segment_times"]))
    elif mutation == "post_mismatch":
        rows[4]["time"] += 0.001
    else:
        rows[4]["policy_segment_times"][0] = float("nan")
    trace.write_text(json.dumps(rows), encoding="utf-8")
    with pytest.raises(ValueError, match="timestamp|Nonfinite"):
        observability.load_episode(path)


def test_wrong_actor_prefix_and_duplicate_inputs_rejected(tmp_path):
    path = write_episode(tmp_path)
    with pytest.raises(ValueError, match="Duplicate rollout"):
        observability.analyze([path, path], tmp_path / "output")
    summary_path = path / "rollout_summary.json"
    summary = json.loads(summary_path.read_text())
    summary["actor_feature_names"][0] = "prescribed_left_target"
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    with pytest.raises(ValueError, match="35-feature actor prefix"):
        observability.load_episode(path)
