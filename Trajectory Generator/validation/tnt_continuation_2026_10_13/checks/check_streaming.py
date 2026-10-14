"""Independent checks of the streaming TNT diagnostic, without OpenSim or training.

Run with a Python environment containing NumPy and SciPy. This writes only
../checks.json. It checks implementation properties, not clinical performance.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import traceback

sys.dont_write_bytecode = True
import numpy as np

HERE = Path(__file__).resolve().parent.parent
PREVIOUS = HERE.parent / "tnt_preliminary_2026_10_13"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PREVIOUS))
from kinematic_generator import KinematicTNTController, RateEstimator, StreamingPhase, stream_replay
from tnt_replay import INVALID, STANCE, Parameters, predict, replay


def fingerprint(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def data_for(subject):
    data = np.genfromtxt(PREVIOUS / f"{subject}_prescribed.csv", delimiter=",", names=True)
    return data["time_s"], data["thigh_deg"]


def bundle_for(subject):
    path = HERE / f"{subject}_bundle.json"
    return json.loads(path.read_text()), path


ARRAY_KEYS = ["theta", "dtheta", "state", "T", "rate", "stance_progress", "cycle", "completed"]


def assert_replay_equal(left, right, keys=ARRAY_KEYS, *, atol=2e-11):
    errors = {}
    for key in keys:
        a, b = np.asarray(left[key]), np.asarray(right[key])
        np.testing.assert_allclose(a, b, rtol=1e-12, atol=atol, err_msg=key)
        errors[key] = float(np.max(np.abs(a - b))) if len(a) else 0.0
    return errors


def literal_and_pfd_batch_equivalence():
    """New streaming filter/phase must reproduce the prior batch interface."""
    results = {}
    old_results = json.loads((PREVIOUS / "results.json").read_text())["results"]
    for item in old_results:
        subject = item["subject"].lower()
        t, theta = data_for(subject)
        p = Parameters(**item["parameters"])
        stance = item["calibration_geometry"]["initial_stance_duration_s"]
        for pfd in [None, item["pfd_swing_duration_s"]]:
            old = replay(t, theta, p, initial_stance_duration=stance, pfd_duration=pfd)
            new = stream_replay(t, theta, p, method="literal", initial_stance_duration=stance,
                                pfd_duration=pfd)
            errors = assert_replay_equal(old, new)
            assert old["events"] == new["events"], "Event timestamps/state transitions changed"
            results[f"{subject}_{'literal' if pfd is None else 'pfd'}"] = {
                "samples": len(t), "events": len(new["events"]), "max_absolute_errors": errors,
                "invalid_samples": int(np.count_nonzero(new["state"] == INVALID)),
            }
    return results


def exercise_cycle(estimator, start, early_duration, middle_duration):
    """Supply known event times directly: no measured extrema or implementation inversion."""
    assert estimator.state == STANCE
    assert estimator.step(start, -1., -1., .01) == "stance_to_early"
    before = estimator.rate
    bm = start + early_duration
    event = estimator.step(bm, -2., 1., early_duration)
    if event == "invalid_T_bm":
        return {"event": event, "T_bm": estimator.T_bm, "next_time": bm}
    assert event == "early_to_middle"
    np.testing.assert_allclose(estimator.T_bm, before * early_duration, atol=1e-12)
    estimator.step(bm + middle_duration / 2, 3., 1., middle_duration / 2)
    end = bm + middle_duration
    assert estimator.step(end, 9., 1., middle_duration / 2) == "middle_to_late"
    rate, raw = estimator.rate, estimator.raw_rate
    finish = end + (1. - estimator.p.t_em) / rate + .01
    assert estimator.step(finish, 12., -1., finish - end) == "late_to_stance"
    assert estimator.T == 0. and estimator.state == STANCE
    return {"event": "completed", "rate": rate, "raw_rate": raw,
            "T_bm": before * early_duration, "next_time": finish + .05}


def analytic_rate_laws():
    """Check the three updates against algebra, including finite-difference loop gain."""
    de, dm, tem, alpha, initial = .20, .40, .80, .25, 1.40
    first = {}
    for method in ["literal", "blend", "duration"]:
        est = RateEstimator(Parameters(0., 10., tem, initial), method=method, alpha=alpha)
        record = exercise_cycle(est, 1., de, dm)
        raw_expected = (tem - initial * de) / dm
        expected = {"literal": raw_expected,
                    "blend": (1 - alpha) * initial + alpha * raw_expected,
                    "duration": tem / (de + dm)}[method]
        np.testing.assert_allclose(record["raw_rate"], raw_expected, atol=1e-12)
        np.testing.assert_allclose(record["rate"], expected, atol=1e-12)
        first[method] = {"actual": record["rate"], "expected": expected}

    gains = []
    for ratio in [.5, 3., 8.]:
        dm, de = .1, .1 * ratio
        equilibrium = tem / (de + dm)
        delta = .001
        row = {"early_to_middle_duration_ratio": ratio}
        for method in ["literal", "blend", "duration"]:
            observed = []
            for initial in [equilibrium, equilibrium + delta]:
                est = RateEstimator(Parameters(0., 10., tem, initial), method=method, alpha=alpha)
                observed.append(exercise_cycle(est, 1., de, dm)["rate"])
            measured_gain = (observed[1] - observed[0]) / delta
            expected_gain = {"literal": -ratio, "blend": 1-alpha*(1+ratio), "duration": 0.}[method]
            np.testing.assert_allclose(measured_gain, expected_gain, atol=1e-10, rtol=1e-10)
            row[method] = {"measured_gain": measured_gain, "expected_gain": expected_gain}
        gains.append(row)

    # A recurrence can be correct yet unstable. Check exposure of the failure,
    # and distinguish this from the filtered update at the same event durations.
    outcomes = {}
    for method in ["literal", "blend", "duration"]:
        est = RateEstimator(Parameters(0., 10., tem, 2.01), method=method, alpha=alpha)
        now, valid_cycles, invalid = 1., 0, False
        for _ in range(10):
            result = exercise_cycle(est, now, .3, .1)
            now = result["next_time"]
            if result["event"] == "invalid_T_bm":
                invalid = True
                break
            valid_cycles += 1
        outcomes[method] = {"valid_cycles": valid_cycles, "invalid": invalid}
    assert outcomes["literal"]["invalid"]
    assert outcomes["blend"] == {"valid_cycles": 10, "invalid": False}
    assert outcomes["duration"] == {"valid_cycles": 10, "invalid": False}
    return {"first_cycle": first, "gain_checks": gains, "unstable_literal_case": outcomes,
            "qualification": "Loop gain assumes fixed event durations and valid within-cycle states; it is not a global stability proof."}


def online_and_prefix_invariance():
    """Changing future thigh measurements must leave earlier phases/commands identical."""
    results = {}
    for subject in ["ab06", "ab07"]:
        bundle, _ = bundle_for(subject)
        full_t, full_theta = data_for(subject)
        t, theta = full_t[:4000], full_theta[:4000]
        n = 1800
        changed = theta.copy()
        changed[n:] = 100 * np.cos(np.arange(len(changed) - n) * .3) - 30
        p = Parameters(**bundle["parameters"])
        stance = bundle["initial_stance_duration_s"]
        subject_checks = {}
        for method in ["literal", "blend", "duration"]:
            kwargs = {"method": method, "alpha": bundle["rate_alpha"],
                      "initial_stance_duration": stance}
            full = stream_replay(t, theta, p, **kwargs)
            prefix = stream_replay(t[:n], theta[:n], p, **kwargs)
            mutated = stream_replay(t, changed, p, **kwargs)
            assert_replay_equal(prefix, {key: full[key][:n] for key in ARRAY_KEYS}, atol=0.)
            assert_replay_equal(prefix, {key: mutated[key][:n] for key in ARRAY_KEYS}, atol=0.)
            online = StreamingPhase(p, initial_time=t[0], initial_thigh_deg=theta[0],
                                    initial_stance_duration=stance, method=method,
                                    alpha=bundle["rate_alpha"])
            records = [online.step(now, angle) for now, angle in zip(t, theta)]
            online_arrays = {key: np.array([row[key] for row in records]) for key in ARRAY_KEYS}
            errors = assert_replay_equal(full, online_arrays, atol=0.)
            subject_checks[method] = {"prefix_samples": n, "total_samples": len(t),
                                      "online_max_absolute_errors": errors}
        a = KinematicTNTController(bundle, t[0], theta[0])
        b = KinematicTNTController(bundle, t[0], theta[0])
        qa, qb = [], []
        for now, angle, mutant in zip(t[:n], theta[:n], changed[:n]):
            qa.append(a.step(now, angle))
            qb.append(b.step(now, mutant))
        np.testing.assert_array_equal(qa, qb)
        subject_checks["controller_identical_prefix"] = n
        results[subject] = subject_checks
    return results


def controller_bundle_equivalence_and_failure():
    results = {}
    for subject in ["ab06", "ab07"]:
        bundle, path = bundle_for(subject)
        t, theta = data_for(subject)
        r = stream_replay(t, theta, Parameters(**bundle["parameters"]),
                          method=bundle["rate_method"], alpha=bundle["rate_alpha"],
                          initial_stance_duration=bundle["initial_stance_duration_s"])
        assert not np.any(r["state"] == INVALID), "Frozen diagnostic bundle becomes invalid"
        template = {key: np.asarray(bundle["template"][key]) for key in ["grid", "stance", "swing"]}
        expected = predict(r, template)
        # Loading from a path exercises the actual saved interface, not only a dict.
        controller = KinematicTNTController(path, t[0], theta[0])
        actual = np.array([controller.step(now, angle) for now, angle in zip(t, theta)])
        assert actual.shape == (len(t), 2) and np.isfinite(actual).all()
        np.testing.assert_allclose(actual, expected, atol=1e-11, rtol=1e-12)
        results[subject] = {"bundle": fingerprint(path), "samples": len(t),
                            "max_reference_error_deg": np.max(np.abs(actual-expected), axis=0).tolist(),
                            "completed_swings": int(r["completed"][-1])}

    # An incomplete early phase can invalidate even a damped inter-cycle law.
    fixture, _ = bundle_for("ab06")
    fixture = json.loads(json.dumps(fixture))
    fixture["parameters"] = {"sigma_deg": 0., "rho_deg": 10., "t_em": .8, "initial_rate": 10.}
    fixture["rate_method"], fixture["rate_alpha"] = "blend", .25
    controller = KinematicTNTController(fixture, 0., -1.)
    controller.step(0., -1.)
    controller.step(.2, -2.)
    did_raise = False
    try:
        controller.step(.21, 10.)
    except RuntimeError:
        did_raise = True
    assert did_raise and controller.last["state"] == INVALID
    results["invalid_command_rejected"] = True
    results["scope"] = "Exact reference generation only; this does not verify slew, dynamic tracking, ground support or clinical safety."
    return results


def main():
    checks = {}
    for function in [literal_and_pfd_batch_equivalence, analytic_rate_laws,
                     online_and_prefix_invariance, controller_bundle_equivalence_and_failure]:
        try:
            checks[function.__name__] = {"passed": True, "details": function()}
        except Exception:
            checks[function.__name__] = {"passed": False, "error": traceback.format_exc()}
    result = {"scope": "Read-only numerical verification of streaming code and frozen bundles; no training or dynamics",
              "all_passed": all(item["passed"] for item in checks.values()),
              "sources": [fingerprint(HERE / "kinematic_generator.py"),
                          fingerprint(PREVIOUS / "tnt_replay.py"), fingerprint(Path(__file__))],
              "checks": checks}
    (HERE / "checks.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"all_passed": result["all_passed"],
                      "checks": {name: item["passed"] for name, item in checks.items()}}))
    if not result["all_passed"]:
        for name, item in checks.items():
            if not item["passed"]:
                print(name, item["error"])
        raise SystemExit(1)


if __name__ == "__main__":
    main()
