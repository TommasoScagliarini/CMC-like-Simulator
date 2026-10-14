"""Hard contact failures preserve the measured sample without advancing events."""
import copy
import json
import pickle
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import simulation_runner as runner_module
from simulation_runner import OnlineGRFPenetrationLimitExceeded, SimulationRunner


def _contact(penetration):
    return {
        "force": np.array([1.0, 500.0, 2.0]),
        "moment": np.array([3.0, 4.0, 5.0]),
        "cop": np.array([0.1, 0.0, -0.2]),
        "normal_force": 500.0,
        "penetration": penetration,
        "slip_speed": 0.15,
        "in_contact": True,
    }


def _runner(monkeypatch, *, left_penetration, right_penetration=0.0):
    grf = {"sides": {"left": _contact(left_penetration), "right": _contact(right_penetration)}}
    runner = SimulationRunner.__new__(SimulationRunner)
    runner._ctx = SimpleNamespace(
        model=SimpleNamespace(getTotalMass=lambda state: 70.0),
        online_grf_force_paths=["primary_left", "primary_right"],
        online_grf_force_sides={"primary_left": "left", "primary_right": "right"},
        online_grf_detector_force_paths=[],
        grf_mode="online_sensor",
        online_grf_applied_sides=["left"],
    )
    runner._cfg = SimpleNamespace(online_grf_max_force_bw=5.0, online_grf_max_penetration_m=0.03)
    runner._online_event_detector = Mock()
    runner._online_event_detector.update.return_value = [{"event": "unchanged"}]
    runner._last_step_info = {"untouched": {"timestamp": 1.0}}
    monkeypatch.setattr(runner_module, "read_online_grf", lambda *args: grf)
    return runner, grf


@pytest.mark.parametrize("args", [(), ("legacy message",), ("legacy", 123)])
def test_legacy_exception_constructor_and_pickle_remain_compatible(args):
    exc = OnlineGRFPenetrationLimitExceeded(*args)
    assert isinstance(exc, FloatingPointError)
    assert exc.args == args
    assert str(exc) == str(FloatingPointError(*args))
    for key in ("time_s", "side", "penetration_m", "limit_m", "online_grf"):
        assert getattr(exc, key) is None
    restored = pickle.loads(pickle.dumps(exc))
    assert restored.args == exc.args
    assert restored.__dict__ == exc.__dict__


def test_exception_snapshot_is_independent_and_pickle_preserves_metadata():
    payload = {"left": {"force": [1.0, 2.0, 3.0], "penetration": 0.031234}}
    exc = OnlineGRFPenetrationLimitExceeded(
        "measured hard contact", time_s=17.648, side="left",
        penetration_m=0.031234, limit_m=0.03, online_grf=payload,
    )
    payload["left"]["force"][0] = -999.0
    payload["left"]["penetration"] = 0.0
    assert exc.online_grf["left"]["force"] == [1.0, 2.0, 3.0]
    assert exc.online_grf["left"]["penetration"] == 0.031234
    restored = pickle.loads(pickle.dumps(exc))
    assert restored.args == exc.args
    assert restored.__dict__ == exc.__dict__
    assert restored.online_grf is not exc.online_grf
    assert restored.online_grf["left"] is not exc.online_grf["left"]
    assert json.loads(json.dumps(restored.online_grf)) == restored.online_grf


def test_hard_guard_carries_actual_sample_without_event_or_state_updates(monkeypatch):
    runner, grf = _runner(monkeypatch, left_penetration=0.031234)
    before = copy.deepcopy(runner._last_step_info)
    with pytest.raises(OnlineGRFPenetrationLimitExceeded) as caught:
        runner._sample_online_grf(object(), 17.648)
    exc = caught.value
    assert exc.time_s == 17.648
    assert exc.side == "left"
    assert exc.penetration_m == 0.031234
    assert exc.limit_m == 0.03
    assert exc.penetration_m != exc.limit_m
    assert exc.online_grf["left"]["penetration"] == exc.penetration_m
    assert exc.online_grf["left"]["moment"] == [3.0, 4.0, 5.0]
    assert exc.online_grf["left"]["cop"] == [0.1, 0.0, -0.2]
    assert json.loads(json.dumps(exc.online_grf)) == exc.online_grf
    runner._online_event_detector.update.assert_not_called()
    assert runner._last_step_info == before
    grf["sides"]["left"]["force"][1] = -100.0
    grf["sides"]["left"]["penetration"] = 0.0
    assert exc.online_grf["left"]["normal_force"] == 500.0
    assert exc.online_grf["left"]["force"][1] == 500.0
    assert exc.online_grf["left"]["penetration"] == 0.031234


@pytest.mark.parametrize("left_penetration", [0.029999, 0.03])
def test_boundary_and_non_applying_side_do_not_change_guard_semantics(monkeypatch, left_penetration):
    runner, grf = _runner(monkeypatch, left_penetration=left_penetration, right_penetration=0.08)
    returned, events = runner._sample_online_grf(object(), 17.648)
    assert returned is grf
    assert events == [{"event": "unchanged"}]
    runner._online_event_detector.update.assert_called_once_with(17.648, {"left": 500.0, "right": 500.0})
