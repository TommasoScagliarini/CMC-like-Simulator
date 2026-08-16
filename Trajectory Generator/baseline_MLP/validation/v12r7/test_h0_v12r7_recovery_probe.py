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

import h0_v12r7_recovery_probe as probe  # noqa: E402


forensic = probe.forensic


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

    def update(
        self,
        *,
        time_s: float,
        events: Sequence[Mapping[str, Any]],
        normal_force_bw: float,
        in_contact: bool,
        prosthetic_knee_angle_rad: float,
        prosthetic_ankle_angle_rad: float,
    ) -> dict[str, float]:
        del time_s, normal_force_bw, in_contact
        del prosthetic_knee_angle_rad, prosthetic_ankle_angle_rad
        for event in events:
            if event["event"] == "heel_strike":
                self.state = 1
            elif event["event"] == "toe_off":
                self.state = 2
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
            "phase_cycle_progress_credit": 0.0,
        }

    def payload(self) -> dict[str, float]:
        return {"invalid_event_this_step": 0.0, "timeout_exceeded": 0.0}


def _fake_phase_factory(
    config: Mapping[str, Any], module_name: str, class_name: str
) -> FakePhaseFSM:
    assert module_name == __name__
    assert class_name == "FakePhaseFSM"
    return FakePhaseFSM(FakePhaseConfig(**dict(config)))


def _actor(index: int) -> np.ndarray:
    result = np.zeros(probe.contract.EXPECTED_ACTOR_FEATURES, dtype=np.float32)
    base = np.float32(index / 1000.0)
    result[2:6] = (base, base + 0.1, base + 0.2, base + 0.3)
    return result


def _info(index: int, *, reset: bool) -> dict[str, Any]:
    actor = _actor(index)
    return {
        "time": 10.0 + index * 0.01,
        "event_contract_id": probe.contract.EVENT_CONTRACT_ID,
        "observation": {
            "pros_knee_angle": float(actor[2]),
            "pros_knee_angle_vel": float(actor[3]),
            "pros_ankle_angle": float(actor[4]),
            "pros_ankle_angle_vel": float(actor[5]),
        },
        "online_grf_detector": (
            {} if reset else {"left": {"normal_force": 420.0, "in_contact": True}}
        ),
        "legacy_online_events": [],
    }


def _build_replay_and_trace(
    tmp_path: Path,
    *,
    config: probe.RecoveryProbeConfig,
    rows: int = 100,
) -> tuple[probe.observer.LoadedReplay, list[dict[str, Any]]]:
    recorder = probe.observer.PureProbeReplayRecorder.from_runtime(
        FakePhaseFSM(),
        700.0,
        probe.observer.coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES,
        event_contract_id=probe.contract.EVENT_CONTRACT_ID,
    )
    recorder.record_reset(_actor(0), _info(0, reset=True))
    trace: list[dict[str, Any]] = []
    previous_penetration = 0.0
    for index in range(rows):
        step = index + 1
        last = step == rows
        next_penetration = 0.026 if last else 0.001 + step * 1.0e-6
        recorder.record_step_boundary(
            None if last else _actor(step),
            _info(step, reset=False),
            previous_penetration_m=None if last else next_penetration,
        )
        mean = [0.1, -0.2]
        trace.append(
            {
                "step": step,
                "schema_version": config.schema_version,
                "protocol_id": config.protocol_id,
                "stage_id": f"collect_label__{config.case_id}",
                "case_id": config.case_id,
                "candidate_id": config.expected_candidate_id,
                "v26_observation": _actor(index).tolist(),
                "candidate_mean": mean,
                "candidate_std": [0.005, 0.005],
                "standard_normal": [0.0, 0.0],
                "single_noise": [0.0, 0.0],
                "raw_action": mean,
                "applied_action": mean,
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "mean_blend_count": 0,
                "safety_latch_enabled": False,
                "safety_intervention_count": 0,
                "safety_latch_activation_count": 0,
                "safety_latch_release_count": 0,
                "previous_penetration_m": previous_penetration,
                "raw_sensor_sample_count": probe.contract.RAW_SAMPLES_PER_STEP,
                "observer_raw_sensor_journal": {
                    "samples": [
                        {"sensor_index": sensor + 1}
                        for sensor in range(probe.contract.RAW_SAMPLES_PER_STEP)
                    ]
                },
                "terminated": last,
                "truncated": False,
                "end_reason": "grf_penetration" if last else None,
            }
        )
        previous_penetration = next_penetration
    replay_path = tmp_path / "replay_boundaries.npz"
    recorder.write_exclusive(replay_path)
    loaded = probe.observer.load_probe_replay_strict(
        replay_path, contract_module=probe.replay_contract
    )
    return loaded, trace


def _summary(*, config: probe.RecoveryProbeConfig, rows: int) -> dict[str, Any]:
    raw_samples = rows * probe.contract.RAW_SAMPLES_PER_STEP
    result = {
        "schema_version": config.schema_version,
        "status": probe.PROBE_COMPLETE_STATUS,
        "protocol_id": config.protocol_id,
        "case_id": config.case_id,
        "steps": rows,
        "terminated": True,
        "truncated": False,
        "end_reason": "grf_penetration",
        "safety_stop_count": 1,
        "grf_penetration_max_m": 0.026,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "candidate_id": config.expected_candidate_id,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "trace_step_count": rows,
        "replay_step_count": rows,
        "replay_boundary_count": rows + 1,
        "raw_sensor_sample_count": raw_samples,
        "control_window_count": raw_samples,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": config.event_contract_id,
        "target_contract_id": config.target_contract_id,
        "binary_event_prefix_integrity": {
            "passed": True,
            "sample_count": raw_samples,
        },
        "morphology_weight": 0.0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    for name in (
        "fallback_count",
        "hard_invalid_count",
        "duplicate_event_count",
        "out_of_order_event_count",
        "left_non_v26_source_count",
        "invalid_event_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "action_clipped_values",
        "nonfinite_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        *probe.ZERO_ONLINE_COUNTERS,
    ):
        result[name] = 0
    return result


def _publish_closed_probe_fixture(
    tmp_path: Path,
    *,
    config: probe.RecoveryProbeConfig,
) -> tuple[Path, probe.observer.LoadedReplay, list[dict[str, Any]]]:
    root = tmp_path / "collection"
    root.mkdir()
    replay, trace = _build_replay_and_trace(root, config=config)
    summary = _summary(config=config, rows=replay.n_steps)
    partial = {
        "schema_version": config.schema_version,
        "status": "PERSISTED_H0_V12R7_PROBE_BEFORE_GATE",
        "steps": replay.n_steps,
    }
    forensic.write_json_exclusive(root / "run_start.json", {"started": True})
    forensic.write_json_exclusive(root / "trace.json", trace)
    forensic.write_json_exclusive(root / "partial_summary.json", partial)
    forensic.write_json_exclusive(root / "summary.json", summary)
    gate = probe.recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    assert gate["passed"] is True
    forensic.write_json_exclusive(root / "gate.json", gate)
    receipt = {
        "schema_version": config.schema_version,
        "status": probe.RECEIPT_STATUS,
        "passed": True,
        "integrity_passed": True,
        "recoverable_for_observer_label": True,
        "autonomy_passed": False,
        "protocol_id": config.protocol_id,
        "stage_id": f"collect_label__{config.case_id}",
        "case_id": config.case_id,
        "probe_step_count": replay.n_steps,
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "run_start": probe._artifact(root / "run_start.json", tmp_path),
        "trace": probe._artifact(root / "trace.json", tmp_path),
        "partial_summary": probe._artifact(root / "partial_summary.json", tmp_path),
        "summary": probe._artifact(root / "summary.json", tmp_path),
        "gate": probe._artifact(root / "gate.json", tmp_path),
        "replay_payload": probe._artifact(root / "replay_boundaries.npz", tmp_path),
        "r6_plus_reproduction_audit": gate["r6_plus_reproduction_audit"],
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(root / "receipt.json", receipt)
    return root, replay, trace


def test_existing_r6_plus_prefix_reproduces_itself_and_tamper_fails() -> None:
    historical_root = probe.REPO_ROOT.joinpath(
        *probe.contract.R6_RUN_ROOT.parts,
        "development",
        "deterministic_offset_plus_0p20",
        "steps",
    )
    rows = [
        probe._strict_mapping(path) for path in sorted(historical_root.glob("*.json"))
    ]
    audit = probe.r6_plus_reproduction_audit(rows)
    assert audit["passed"] is True
    assert audit["historical_step_count"] == audit["current_step_count"] == 179

    tampered = [dict(row) for row in rows]
    tampered[149]["candidate_mean"] = [0.0, 0.0]
    failed = probe.r6_plus_reproduction_audit(tampered)
    assert failed["passed"] is False
    assert failed["first_mismatch_step"] == 150
    assert failed["first_mismatch_field"] == "candidate_mean"


def test_recoverable_prefix_gate_is_pure_detector_strict_and_type_strict(
    tmp_path: Path,
) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_minus_0p20", artifact_root=tmp_path
    )
    replay, trace = _build_replay_and_trace(tmp_path, config=config)
    summary = _summary(config=config, rows=replay.n_steps)
    gate = probe.recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    assert gate["passed"] is True
    assert gate["recoverable_prefix"] is True
    assert gate["autonomy_passed"] is False

    teacher = dict(summary)
    teacher["teacher_query_count"] = 1
    assert (
        probe.recovery_prefix_gate(teacher, trace=trace, replay=replay, config=config)[
            "passed"
        ]
        is False
    )
    detector = dict(summary)
    detector["left_non_v26_source_count"] = 1
    assert (
        probe.recovery_prefix_gate(detector, trace=trace, replay=replay, config=config)[
            "passed"
        ]
        is False
    )
    bool_update = dict(summary)
    bool_update["actor_updates"] = False
    typed = probe.recovery_prefix_gate(
        bool_update, trace=trace, replay=replay, config=config
    )
    assert typed["checks"]["zero_updates"] is False
    assert typed["passed"] is False


def test_offline_label_stage_persists_exact_same_state_schema(tmp_path: Path) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_minus_0p20", artifact_root=tmp_path
    )
    root, replay, _trace = _publish_closed_probe_fixture(tmp_path, config=config)
    load_count = 0
    query_count = 0

    def load(_path: Path) -> object:
        nonlocal load_count
        assert probe.verify_probe_closure(root, config=config)["passed"] is True
        load_count += 1
        return object()

    def query(_module: object, view: np.ndarray) -> np.ndarray:
        nonlocal query_count
        assert view.shape == (probe.contract.EXPECTED_ACTOR_FEATURES,)
        query_count += 1
        return np.asarray([0.1, -0.2], dtype=np.float32)

    def coverage(observations: np.ndarray) -> Mapping[str, Any]:
        rows = len(observations)
        return {
            "distance_rms_z": np.zeros(rows, dtype=np.float64),
            "nearest_reference_index": np.zeros(rows, dtype=np.int64),
            "ood_mask": np.zeros(rows, dtype=np.bool_),
            "audit": {"passed": True},
        }

    labelled = probe.label_recovery_probe(
        config=config,
        probe_destination=root,
        label_destination=root / "observer_labels",
        source_h0_path=tmp_path / "fake_h0",
        module_loader=load,
        mean_query=query,
        coverage_evaluator=coverage,
        phase_fsm_factory=_fake_phase_factory,
    )
    assert labelled["passed"] is True
    assert load_count == 1
    assert query_count == replay.n_steps == 100
    label_root = root / "observer_labels"
    assert {path.name for path in label_root.iterdir()} == {
        "labels.npz",
        "summary.json",
        "gate.json",
        "receipt.json",
    }
    with np.load(label_root / "labels.npz", allow_pickle=False) as archive:
        assert set(archive.files) == set(probe.observer.LABEL_ARRAY_DTYPES)
        assert archive["observations"].tobytes(order="C") == replay.arrays[
            "actor_observations"
        ].tobytes(order="C")
        assert archive["actions"].dtype == np.dtype(np.float32)
        assert archive["actions"].shape == (100, 2)
        assert set(archive["tranche_ids"].tolist()) == {"observer_probe_p0"}
        assert archive["origins"][0] == (
            "pure_observer:p0:deterministic_offset_minus_0p20:1"
        )
    receipt = probe._strict_mapping(label_root / "receipt.json")
    assert receipt["passed"] is True
    assert receipt["teacher_query_count"] == 100
    assert receipt["environment_reset_calls"] == 0
    assert receipt["environment_step_calls"] == 0


def test_failed_probe_closure_prevents_h0_load(tmp_path: Path) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_minus_0p20", artifact_root=tmp_path
    )
    root, _replay, _trace = _publish_closed_probe_fixture(tmp_path, config=config)
    loaded = False
    gate = probe._strict_mapping(root / "gate.json")
    gate["passed"] = False
    (root / "gate.json").unlink()
    forensic.write_json_exclusive(root / "gate.json", gate)

    def load(_path: Path) -> object:
        nonlocal loaded
        loaded = True
        return object()

    with pytest.raises(probe.V12R7RecoveryProbeError, match="gate|binding"):
        probe.label_recovery_probe(
            config=config,
            probe_destination=root,
            label_destination=root / "observer_labels",
            source_h0_path=tmp_path / "fake_h0",
            module_loader=load,
            mean_query=lambda _module, _view: np.zeros(2, dtype=np.float32),
            coverage_evaluator=lambda observations: {
                "distance_rms_z": np.zeros(len(observations)),
                "nearest_reference_index": np.zeros(len(observations), dtype=np.int64),
                "ood_mask": np.zeros(len(observations), dtype=np.bool_),
            },
            phase_fsm_factory=_fake_phase_factory,
        )
    assert loaded is False
