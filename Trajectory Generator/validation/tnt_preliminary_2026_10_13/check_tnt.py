"""Independent deterministic checks of the isolated, training-free TNT replay."""
from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
import traceback

import numpy as np

from tnt_replay import (EARLY, LATE, MIDDLE, STANCE, Parameters, TNT,
                        causal_filter, inverse_quintic, make_template, predict, replay)


HERE = Path(__file__).resolve().parent
ARRAY_KEYS = ("t", "theta", "dtheta", "state", "T", "stance_progress",
              "cycle", "completed", "rate")


def assert_close(actual, expected, tolerance=1e-7):
    np.testing.assert_allclose(actual, expected, atol=tolerance, rtol=0)


def check_inverse():
    # Exact rational values of the forward minimum-jerk polynomial, not outputs
    # from a production forward/inverse helper that could share a defect.
    cases = [(0, 0), (53 / 512, 1 / 4), (1 / 2, 1 / 2),
             (459 / 512, 3 / 4), (1, 1)]
    outputs = [inverse_quintic(y) for y, _ in cases]
    assert_close(outputs, [x for _, x in cases], tolerance=2e-8)
    assert inverse_quintic(-0.1) == 0
    assert inverse_quintic(1.1) == 1
    return {"analytic_pairs_y_x": cases, "actual_x": outputs,
            "max_abs_error": float(max(abs(a - x) for a, (_, x) in zip(outputs, cases)))}


def begin_swing():
    estimator = TNT(Parameters(sigma_deg=-5, rho_deg=20, t_em=0.8, initial_rate=2))
    assert estimator.step(0, 0, 0, 0) == ""
    assert estimator.state == STANCE and estimator.T == 0
    assert estimator.step(0.01, -6, -600, 0.01) == "stance_to_early"
    assert estimator.state == EARLY and estimator.T == 0
    assert estimator.step(0.02, -8, -200, 0.01) == ""
    assert_close(estimator.T, 0.02)
    assert estimator.step(0.03, -10, -200, 0.01) == ""
    assert_close(estimator.T, 0.04)
    assert estimator.step(0.04, -9.5, 50, 0.01) == "early_to_middle"
    assert estimator.state == MIDDLE
    assert_close(estimator.T_bm, 0.06)
    assert_close(estimator.theta_bm, -9.5)
    return estimator


def check_known_swing():
    estimator = begin_swing()
    cases = [(0.14, 53 / 512, 0.245), (0.24, 0.5, 0.43),
             (0.34, 459 / 512, 0.615), (0.44, 1.0, 0.8)]
    measured = []
    for t, normalized_theta, expected_phase in cases:
        event = estimator.step(t, -9.5 + 20 * normalized_theta, 1, 0.1)
        assert event == "" and estimator.state == MIDDLE
        assert_close(estimator.T, expected_phase)
        measured.append(float(estimator.T))
    # The paper uses a strict > rho guard: equality is still middle swing.
    event = estimator.step(0.45, 10.50001, 1, 0.01)
    assert event == "middle_to_late" and estimator.state == LATE
    expected_rate = 0.74 / 0.41
    assert_close(estimator.rate, expected_rate)
    estimator.step(0.50, 12, 1, 0.05)
    assert_close(estimator.T, 0.8 + 0.05 * expected_rate)
    estimator.step(0.55, 14, 1, 0.05)
    assert_close(estimator.T, 0.8 + 0.1 * expected_rate)
    event = estimator.step(0.57, 15, 1, 0.02)
    assert event == "late_to_stance" and estimator.state == STANCE
    assert estimator.T == 0 and estimator.completed == 1
    assert estimator.step(0.65, -6, -1, 0.08) == "stance_to_early"
    estimator.step(0.66, -8, -1, 0.01)
    assert_close(estimator.T, expected_rate * 0.01)
    return {"middle_phase_values": measured, "middle_average_rate": float(estimator.rate),
            "completed_swings": estimator.completed,
            "checks": ["early previous-rate integration", "analytic quintic middle trajectory",
                       "strict excursion guard", "late current-rate integration",
                       "reset at completion", "rate carried into next early swing"]}


def check_insufficient_excursion():
    estimator = begin_swing()
    # Excursion reaches only half rho and then stays there for ten seconds.
    for t in np.arange(0.05, 10.05, 0.01):
        estimator.step(float(t), 0.5, 0.0, 0.01)
    assert estimator.state == MIDDLE
    assert estimator.completed == 0
    assert_close(estimator.T, 0.43)
    return {"state": "middle", "completed": estimator.completed,
            "phase": float(estimator.T), "result": "Correctly exposes a stall; no fabricated completion"}


def check_filter_analytic_step():
    time = np.arange(101) * 0.01
    raw = np.ones_like(time)
    raw[0] = 0
    filtered, derivative = causal_filter(time, raw, cutoff_hz=6)
    expected = 1 - np.exp(-2 * np.pi * 6 * time)
    assert_close(filtered, expected, tolerance=1e-14)
    assert_close(derivative[1:], np.diff(expected) / 0.01, tolerance=1e-12)
    assert derivative[0] == 0
    return {"max_filter_error": float(np.max(abs(filtered - expected))),
            "initial_derivative": float(derivative[0])}


def synthetic_record():
    time = np.arange(3001) * 0.01
    theta = 10 * np.cos(2 * np.pi * time / 1.2) + 0.03 * np.sin(2 * np.pi * 13 * time)
    return time, theta, Parameters(-5, 10, 0.8, 2)


def same_prefix(full, prefix, n):
    for key in ARRAY_KEYS:
        np.testing.assert_array_equal(full[key][:n], prefix[key][:n], err_msg=key)
    expected_events = [event for event in full["events"] if event["i"] < n]
    actual_events = [event for event in prefix["events"] if event["i"] < n]
    assert expected_events == actual_events


def check_prefix_invariance():
    t, theta, p = synthetic_record()
    full = replay(t, theta, p)
    prefixes = [1, 2, 17, 213, 1000, 2037]
    for n in prefixes:
        same_prefix(full, replay(t[:n], theta[:n], p), n)
    n = 1000
    changed = theta.copy()
    changed[n:] += 70 * np.sin(2 * np.pi * 2 * t[n:]) + 40
    perturbed = replay(t, changed, p)
    same_prefix(full, perturbed, n)
    assert not np.array_equal(full["theta"][n:], perturbed["theta"][n:])
    return {"prefix_lengths": prefixes, "compared_arrays": list(ARRAY_KEYS),
            "future_perturbation_starts_at_s": float(t[n]),
            "events_and_all_arrays_identical_before_perturbation": True,
            "completed_synthetic_swings": int(full["completed"][-1])}


def check_target_separation():
    assert "q" not in inspect.signature(replay).parameters
    assert "q" not in inspect.signature(TNT.step).parameters
    assert "q" not in inspect.signature(predict).parameters
    t, theta, p = synthetic_record()
    r = replay(t, theta, p)
    target = np.column_stack((-30 - 25 * np.sin(2 * np.pi * t / 1.2),
                              10 * np.sin(2 * np.pi * t / 1.2 + 0.7)))
    calibration = t < 12
    original = make_template(r, target, calibration)
    changed_target = target.copy()
    changed_target[~calibration] += np.array([1000, -2000])
    changed = make_template(r, changed_target, calibration)
    for key in ("grid", "stance", "swing"):
        np.testing.assert_array_equal(original[key], changed[key])
    np.testing.assert_array_equal(predict(r, original), predict(r, changed))
    return {"replay_inputs": str(inspect.signature(replay)),
            "predict_inputs": str(inspect.signature(predict)),
            "heldout_target_perturbation_deg": [1000, -2000],
            "templates_and_predictions_unchanged": True,
            "calibration_blocks": original["block_counts"]}


def split_notes():
    notes = {}
    for subject in ("ab06", "ab07"):
        receipt = json.loads((HERE / f"{subject}_extraction_receipt.json").read_text())
        start = receipt["output_time_range_s"][0]
        notes[subject] = {
            "extraction_receipt_proposed_split_s": receipt["proposed_temporal_split"]["calibration_end_s"],
            "replay_default_actual_split_s": start + 40,
            "heading_end_s": receipt["angle_definition"]["heading_calibration_interval_s"][1],
            "interpretation": "Extraction receipt offers a possible 30% split; replay actually uses first40s, then heldout. They are deliberately different.",
        }
        assert notes[subject]["heading_end_s"] < start + 25 < start + 40
    return notes


def main():
    source = HERE / "tnt_replay.py"
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    result = {"scope": "Independent analytical and information-flow checks; no dynamics or training",
              "tested_source": source.name, "tested_source_sha256": source_hash, "checks": {}}
    checks = {"inverse_quintic_analytic": check_inverse,
              "known_synthetic_swing": check_known_swing,
              "insufficient_excursion": check_insufficient_excursion,
              "causal_filter_analytic_step": check_filter_analytic_step,
              "prefix_and_future_invariance": check_prefix_invariance,
              "heldout_target_separation": check_target_separation,
              "temporal_split_interpretation": split_notes}
    for name, check in checks.items():
        try:
            result["checks"][name] = {"passed": True, "details": check()}
        except Exception:
            result["checks"][name] = {"passed": False, "traceback": traceback.format_exc()}
    result["tested_source_unchanged"] = hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
    result["all_passed"] = all(value["passed"] for value in result["checks"].values()) and result["tested_source_unchanged"]
    (HERE / "checks.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"all_passed": result["all_passed"],
                      "checks": {name: value["passed"] for name, value in result["checks"].items()}}))
    if not result["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
