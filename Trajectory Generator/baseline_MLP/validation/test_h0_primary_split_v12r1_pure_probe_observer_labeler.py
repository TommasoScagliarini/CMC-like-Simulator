from __future__ import annotations

import dataclasses
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import pytest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import h0_primary_split_v12r1_autonomy_recovery_contract as contract
import h0_primary_split_v12r1_pure_probe_observer_labeler as labeler


@dataclasses.dataclass(frozen=True)
class FakePhaseConfig:
    event_source: str = "binary_active_v26"
    gain: float = 0.75


class FakePhaseFSM:
    def __init__(self, config: FakePhaseConfig | None = None) -> None:
        self.config = config or FakePhaseConfig()
        self.reset()

    def reset(self) -> None:
        self.state = 0
        self.credit = 0.0

    def update(
        self,
        *,
        time_s: float,
        events: Sequence[Mapping[str, Any]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float,
        prosthetic_ankle_angle_rad: float,
    ) -> dict[str, Any]:
        del time_s, normal_force_bw, in_contact
        del prosthetic_knee_angle_rad, prosthetic_ankle_angle_rad
        for event in events:
            if event["event"] == "heel_strike":
                self.state = 1
                self.credit = float(self.config.gain)
            elif event["event"] == "toe_off":
                self.state = 2
                self.credit = 0.5
        return self.payload()

    def observation(self) -> dict[str, float]:
        return {
            "phase_fsm_wait_hs": float(self.state == 0),
            "phase_fsm_stance_after_hs": float(self.state == 1),
            "phase_fsm_swing_after_to": float(self.state == 2),
            "phase_expected_hs": float(self.state in {0, 2}),
            "phase_expected_to": float(self.state == 1),
            "phase_stance_elapsed_norm": 0.0,
            "phase_swing_elapsed_norm": 0.0,
            "phase_cycle_progress_credit": self.credit,
        }

    def payload(self) -> dict[str, float]:
        return {
            "invalid_event_this_step": 0.0,
            "timeout_exceeded": 0.0,
        }


def _fake_factory(
    config: Mapping[str, Any], module_name: str, class_name: str
) -> FakePhaseFSM:
    assert module_name == __name__
    assert class_name == "FakePhaseFSM"
    return FakePhaseFSM(FakePhaseConfig(**dict(config)))


def _info(
    time_s: float,
    *,
    knee: float,
    knee_speed: float,
    ankle: float,
    ankle_speed: float,
    load_bw: float | None,
    events: Sequence[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    detector: dict[str, Any] = {}
    if load_bw is not None:
        detector = {
            "left": {
                "normal_force": float(load_bw) * 700.0,
                "in_contact": bool(load_bw > 0.0),
            }
        }
    return {
        "time": time_s,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "observation": {
            "pros_knee_angle": knee,
            "pros_knee_angle_vel": knee_speed,
            "pros_ankle_angle": ankle,
            "pros_ankle_angle_vel": ankle_speed,
        },
        "online_grf_detector": detector,
        "legacy_online_events": list(events),
    }


def _actor(knee: float, knee_speed: float, ankle: float, ankle_speed: float) -> np.ndarray:
    actor = np.zeros(35, dtype=np.float32)
    actor[2:6] = (knee, knee_speed, ankle, ankle_speed)
    return actor


def _replay_arrays() -> dict[str, np.ndarray]:
    reset = _info(
        10.0,
        knee=0.1,
        knee_speed=0.2,
        ankle=0.3,
        ankle_speed=0.4,
        load_bw=None,
    )
    recorder = labeler.PureProbeReplayRecorder.from_runtime(
        FakePhaseFSM(),
        700.0,
        labeler.coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES,
        event_contract_id=contract.EVENT_CONTRACT_ID,
    )
    recorder.record_reset(_actor(0.1, 0.2, 0.3, 0.4), reset)
    hs = {
        "side": "left",
        "event": "heel_strike",
        "event_time_s": 10.5,
        "confirmed_time_s": 10.75,
        "delivered_time_s": 11.0,
        "contact_duration_s": 0.1,
        "startup_partial_stance": False,
    }
    recorder.record_step_boundary(
        _actor(0.11, 0.21, 0.31, 0.41),
        _info(
            11.0,
            knee=0.11,
            knee_speed=0.21,
            ankle=0.31,
            ankle_speed=0.41,
            load_bw=0.6,
            events=[hs],
        ),
        previous_penetration_m=0.012,
    )
    recorder.record_step_boundary(
        None,
        _info(
            12.0,
            knee=0.12,
            knee_speed=0.22,
            ankle=0.32,
            ankle_speed=0.42,
            load_bw=0.0,
        ),
    )
    return recorder.arrays()


def _write(path: Path, arrays: Mapping[str, np.ndarray]) -> None:
    np.savez(path, **arrays)


def test_self_contained_replay_round_trip_and_column_24_is_mutable(
    tmp_path: Path,
) -> None:
    path = tmp_path / "replay.npz"
    labeler.write_replay_npz_exclusive(path, _replay_arrays())
    loaded = labeler.load_probe_replay_strict(path)
    assert loaded.n_steps == 2
    assert loaded.boundary_count == 3
    assert loaded.event_count == 1
    assert loaded.config["event_source"] == "legacy_events"
    assert loaded.arrays["legacy_left_event_boundary_offsets"].tolist() == [0, 0, 1, 1]
    replayed = labeler.replay_teacher_views(
        loaded, phase_fsm_factory=_fake_factory
    )
    assert replayed.replayed_boundary_count == 3
    assert replayed.invariant_columns_byte_exact_count == 2
    assert replayed.changed_only_mutable_count == 2
    assert replayed.column_24_changed_count == 1
    assert replayed.teacher_observations[1, 24] == np.float32(0.75)
    immutable = list(labeler.IMMUTABLE_ACTOR_COLUMNS)
    assert (
        replayed.student_observations[:, immutable].tobytes()
        == replayed.teacher_observations[:, immutable].tobytes()
    )


def test_missing_journal_array_and_tampered_delivery_are_rejected(
    tmp_path: Path,
) -> None:
    arrays = _replay_arrays()
    missing = dict(arrays)
    missing.pop("legacy_left_event_contact_duration_s")
    missing_path = tmp_path / "missing.npz"
    _write(missing_path, missing)
    with pytest.raises(labeler.V12R1ObserverLabelError, match="schema drifted"):
        labeler.load_probe_replay_strict(missing_path)

    tampered = {name: value.copy() for name, value in arrays.items()}
    tampered["legacy_left_event_delivered_time_s"][0] = 11.5
    tampered_path = tmp_path / "tampered_event.npz"
    _write(tampered_path, tampered)
    with pytest.raises(
        labeler.V12R1ObserverLabelError,
        match="delivery does not match|topology",
    ):
        labeler.load_probe_replay_strict(tampered_path)


def test_config_hash_and_noncanonical_config_are_rejected(tmp_path: Path) -> None:
    arrays = _replay_arrays()
    hash_tampered = {name: value.copy() for name, value in arrays.items()}
    hash_tampered["legacy_fsm_config_sha256_ascii"][0] = ord("0")
    path = tmp_path / "hash_tampered.npz"
    _write(path, hash_tampered)
    with pytest.raises(labeler.V12R1ObserverLabelError, match="SHA-256 mismatch"):
        labeler.load_probe_replay_strict(path)

    config_tampered = {name: value.copy() for name, value in arrays.items()}
    raw = config_tampered["legacy_fsm_config_json_utf8"].tobytes()
    noncanonical = raw.replace(b":", b": ", 1)
    config_tampered["legacy_fsm_config_json_utf8"] = np.frombuffer(
        noncanonical, dtype=np.uint8
    ).copy()
    digest = labeler.hashlib.sha256(noncanonical).hexdigest().encode("ascii")
    config_tampered["legacy_fsm_config_sha256_ascii"] = np.frombuffer(
        digest, dtype=np.uint8
    ).copy()
    path = tmp_path / "config_tampered.npz"
    _write(path, config_tampered)
    with pytest.raises(labeler.V12R1ObserverLabelError, match="noncanonical"):
        labeler.load_probe_replay_strict(path)


def test_object_dtype_cannot_cross_no_pickle_loader(tmp_path: Path) -> None:
    arrays = _replay_arrays()
    malformed = {name: value.copy() for name, value in arrays.items()}
    malformed["legacy_left_event_side"] = np.asarray(["left"], dtype=object)
    path = tmp_path / "object.npz"
    _write(path, malformed)
    with pytest.raises(labeler.V12R1ObserverLabelError, match="pickle-free"):
        labeler.load_probe_replay_strict(path)


def test_h0_load_occurs_only_after_closure_and_exactly_n_queries(
    tmp_path: Path,
) -> None:
    path = tmp_path / "replay.npz"
    _write(path, _replay_arrays())
    order: list[str] = []
    query_count = 0

    def closure(replay: labeler.LoadedReplay) -> None:
        assert replay.n_steps == 2
        order.append("closure")

    def load(_path: Path) -> object:
        assert order == ["closure"]
        order.append("load")
        return object()

    def query(_module: object, view: np.ndarray) -> np.ndarray:
        nonlocal query_count
        assert order == ["closure", "load"]
        assert view.shape == (35,)
        query_count += 1
        return np.asarray([0.1, -0.2], dtype=np.float32)

    def coverage(observations: np.ndarray) -> Mapping[str, Any]:
        assert observations.shape == (2, 35)
        return {
            "distance_rms_z": np.asarray([0.01, 0.2], dtype=np.float64),
            "nearest_reference_index": np.asarray([1, 2], dtype=np.int64),
            "ood_mask": np.asarray([False, True], dtype=np.bool_),
            "audit": {"passed": True},
        }

    result = labeler.label_closed_probe_in_memory(
        path,
        probe_closure_validator=closure,
        source_h0_path=tmp_path / "fake_h0",
        case_id="deterministic_offset_minus_0p20",
        probe_stage="p0",
        module_loader=load,
        mean_query=query,
        coverage_evaluator=coverage,
        phase_fsm_factory=_fake_factory,
    )
    assert order == ["closure", "load"]
    assert query_count == result.replay.n_steps == result.teacher_query_count == 2
    assert result.environment_reset_calls == 0
    assert result.environment_step_calls == 0
    assert result.action_served_count == 0
    assert result.arrays["coverage_ood_mask"].tolist() == [False, True]
    assert result.arrays["raw_sample_weights"].tolist() == [100.0, 100.0]
    assert sum(result.arrays["reset_mask"].tolist()) == 1
    assert labeler.math.fsum(
        result.arrays["normalized_sample_weights"].tolist()
    ) == 500.0


def test_failed_closure_prevents_fsm_replay_and_h0_load(tmp_path: Path) -> None:
    path = tmp_path / "replay.npz"
    _write(path, _replay_arrays())
    loaded = False

    def reject(_replay: labeler.LoadedReplay) -> None:
        raise RuntimeError("probe not closed")

    def load(_path: Path) -> object:
        nonlocal loaded
        loaded = True
        return object()

    with pytest.raises(labeler.V12R1ObserverLabelError, match="not validly closed"):
        labeler.label_closed_probe_in_memory(
            path,
            probe_closure_validator=reject,
            source_h0_path=tmp_path / "fake_h0",
            case_id="deterministic_offset_minus_0p20",
            probe_stage="p0",
            module_loader=load,
            mean_query=lambda _module, _view: np.zeros(2, dtype=np.float32),
            coverage_evaluator=lambda observations: {
                "distance_rms_z": np.zeros(len(observations)),
                "nearest_reference_index": np.zeros(len(observations), dtype=np.int64),
                "ood_mask": np.zeros(len(observations), dtype=np.bool_),
            },
            phase_fsm_factory=_fake_factory,
        )
    assert loaded is False


def test_replay_npz_is_no_clobber(tmp_path: Path) -> None:
    path = tmp_path / "replay.npz"
    labeler.write_replay_npz_exclusive(path, _replay_arrays())
    with pytest.raises(labeler.V12R1ObserverLabelError, match="clobber"):
        labeler.write_replay_npz_exclusive(path, _replay_arrays())
