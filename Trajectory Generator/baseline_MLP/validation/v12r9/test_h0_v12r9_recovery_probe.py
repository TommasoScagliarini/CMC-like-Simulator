from __future__ import annotations

import copy
import dataclasses
import inspect
import shutil
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r9_recovery_probe as probe  # noqa: E402


@dataclasses.dataclass(frozen=True)
class FakePhaseConfig:
    event_source: str = "binary_active_v26"


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


def _phase_factory(
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
    root: Path, *, config: probe.RecoveryProbeConfig, rows: int = 100
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
    replay_path = root / "replay_boundaries.npz"
    recorder.write_exclusive(replay_path)
    loaded = probe.observer.load_probe_replay_strict(
        replay_path, contract_module=probe.replay_contract
    )
    return loaded, trace


def _raw_summary(*, config: probe.RecoveryProbeConfig, rows: int) -> dict[str, Any]:
    raw_samples = rows * probe.contract.RAW_SAMPLES_PER_STEP
    result = {
        "steps": rows,
        "terminated": True,
        "truncated": False,
        "end_reason": "grf_penetration",
        "safety_stop_count": 1,
        "grf_penetration_max_m": 0.026,
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
        "binary_phase_event_gate": {
            "passed": False,
            "sample_count": raw_samples,
            "event_count": 0,
            "events": [],
            "fallback_count": 0,
            "hard_invalid_count": 0,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
        },
        "binary_event_prefix_integrity": {
            "passed": True,
            "sample_count": raw_samples,
            "expected_sample_count": raw_samples,
            "raw_sensor_sample_count": raw_samples,
            "checks": {"all_exact": True},
        },
        "morphology_weight": 0.0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    for name in (
        "fallback_count",
        "hard_invalid_count",
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


def _closed_fixture(
    tmp_path: Path, *, config: probe.RecoveryProbeConfig
) -> tuple[Path, probe.observer.LoadedReplay]:
    root = tmp_path / "collection"
    root.mkdir()
    source_candidate = probe.REPO_ROOT.joinpath(
        *probe.contract.R6_CANDIDATE_MODULE_PATH.parts
    )
    fixture_candidate = tmp_path.joinpath(
        *probe.contract.R6_CANDIDATE_MODULE_PATH.parts
    )
    fixture_candidate.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_candidate, fixture_candidate)
    replay, trace = _build_replay_and_trace(root, config=config)
    normalization = probe.adjudicator.normalize_v26_prefix_summary(
        _raw_summary(config=config, rows=replay.n_steps),
        expected_steps=replay.n_steps,
    )
    summary = {
        **normalization["summary"],
        "schema_version": config.schema_version,
        "status": probe.PROBE_COMPLETE_STATUS,
        "protocol_id": config.protocol_id,
        "stage_id": f"collect_label__{config.case_id}",
        "case_id": config.case_id,
        "behavior": probe.PROBE_BEHAVIOR,
        "candidate_module": copy.deepcopy(probe.contract.LOCKED_INPUTS["r6_candidate"]),
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "candidate_id": config.expected_candidate_id,
        "trace_step_count": replay.n_steps,
        "replay_step_count": replay.n_steps,
        "replay_boundary_count": replay.boundary_count,
        "replay_event_count": replay.event_count,
        "v26_summary_normalization": probe._normalization_receipt(normalization),
        "historical_plus_rerun": False,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
    }
    partial = {
        "schema_version": config.schema_version,
        "status": "PERSISTED_H0_V12R9_PROBE_BEFORE_GATE",
        "steps": replay.n_steps,
    }
    probe.forensic.write_json_exclusive(root / "run_start.json", {"started": True})
    for step, row in enumerate(trace, start=1):
        assert row["step"] == step
        probe.forensic.write_json_exclusive(root / "steps" / f"{step:06d}.json", row)
    probe.forensic.write_json_exclusive(root / "trace.json", trace)
    probe.forensic.write_json_exclusive(root / "partial_summary.json", partial)
    probe.forensic.write_json_exclusive(root / "summary.json", summary)
    gate = probe.recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    assert gate["passed"] is True
    probe.forensic.write_json_exclusive(root / "gate.json", gate)
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
        "v26_summary_normalization": probe._normalization_receipt(normalization),
        "historical_plus_rerun": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    probe.forensic.write_json_exclusive(root / "receipt.json", receipt)
    return root, replay


def test_probe_rejects_historical_plus_rerun(tmp_path: Path) -> None:
    with pytest.raises(probe.V12R9RecoveryProbeError, match="rerun is forbidden"):
        probe.canonical_probe_case(
            probe.RecoveryProbeConfig(
                case_id=probe.contract.HISTORICAL_CASE_ID,
                artifact_root=tmp_path,
            )
        )


def test_nested_summary_is_projected_before_recovery_gate(tmp_path: Path) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_nominal", artifact_root=tmp_path
    )
    replay, trace = _build_replay_and_trace(tmp_path, config=config)
    raw = _raw_summary(config=config, rows=replay.n_steps)
    normalization = probe.adjudicator.normalize_v26_prefix_summary(
        raw,
        expected_steps=replay.n_steps,
    )
    normalized = {
        **normalization["summary"],
        "schema_version": config.schema_version,
        "protocol_id": config.protocol_id,
        "case_id": config.case_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "candidate_id": config.expected_candidate_id,
        "v26_summary_normalization": probe._normalization_receipt(normalization),
    }
    gate = probe.recovery_prefix_gate(
        normalized, trace=trace, replay=replay, config=config
    )
    assert gate["passed"] is True
    assert gate["checks"]["summary_normalized_before_gate"] is True
    assert normalization["nested_gate_passed_preserved"] is False
    assert raw["binary_phase_event_gate"]["passed"] is False
    assert all(name not in raw for name in probe.adjudicator.PROJECTED_ANOMALY_FIELDS)
    assert "target_contract_id" not in raw


def test_canonical_real_probe_requires_the_complete_locked_candidate_tree(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_nominal", artifact_root=tmp_path
    )
    root, _replay = _closed_fixture(tmp_path, config=config)
    closed = probe.verify_probe_closure(root, config=config)
    assert closed["passed"] is True
    assert (
        closed["summary"]["candidate_module"] == probe.contract.FULL_R6_CANDIDATE_TREE
    )

    drifted = copy.deepcopy(probe.contract.LOCKED_INPUTS)
    drifted["r6_candidate"] = {
        name: drifted["r6_candidate"][name]
        for name in ("path", "tree_sha256", "file_count")
    }
    monkeypatch.setattr(probe.contract, "LOCKED_INPUTS", drifted)
    with pytest.raises(probe.V12R9RecoveryProbeError, match="full locked"):
        probe.verify_probe_closure(root, config=config)


def test_new_probe_offline_label_is_same_state_and_environment_free(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_nominal", artifact_root=tmp_path
    )
    root, replay = _closed_fixture(tmp_path, config=config)
    fake_h0 = tmp_path / "fake_h0"
    fake_h0.mkdir()
    (fake_h0 / "module_state.pkl").write_bytes(b"test-double")
    load_count = 0
    query_count = 0

    def load(_path: Path) -> object:
        nonlocal load_count
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
        source_h0_path=fake_h0,
        module_loader=load,
        mean_query=query,
        coverage_evaluator=coverage,
        phase_fsm_factory=_phase_factory,
    )
    assert labelled["passed"] is True
    assert load_count == 1
    assert query_count == replay.n_steps == 100
    assert labelled["receipt"]["environment_reset_calls"] == 0
    assert labelled["receipt"]["environment_step_calls"] == 0
    with np.load(root / "observer_labels" / "labels.npz", allow_pickle=False) as data:
        assert data["observations"].tobytes(order="C") == replay.arrays[
            "actor_observations"
        ].tobytes(order="C")
    monkeypatch.setattr(
        probe.contract,
        "collection_case_root",
        lambda _case_id: PurePosixPath("collection"),
    )
    verified = probe.verify_observer_label_closure(
        config.case_id,
        artifact_root=tmp_path,
        source_h0_path=fake_h0,
        module_loader=load,
        mean_query=query,
        coverage_evaluator=coverage,
        phase_fsm_factory=_phase_factory,
        require_stage_receipt=False,
    )
    assert verified["passed"] is True
    label_path = root / "observer_labels" / "labels.npz"
    with np.load(label_path, allow_pickle=False) as archive:
        mutated = {name: archive[name].copy() for name in archive.files}
    mutated["actions"][0, 0] = np.float32(mutated["actions"][0, 0] + 0.25)
    label_path.unlink()
    np.savez(label_path, **mutated)
    with pytest.raises(probe.V12R9RecoveryProbeError, match="array bytes"):
        probe.verify_observer_label_closure(
            config.case_id,
            artifact_root=tmp_path,
            source_h0_path=fake_h0,
            module_loader=load,
            mean_query=query,
            coverage_evaluator=coverage,
            phase_fsm_factory=_phase_factory,
            require_stage_receipt=False,
        )


def test_new_collection_label_stage_binding_accepts_its_complete_schema(
    tmp_path: Path,
) -> None:
    case_id = "deterministic_offset_nominal"
    case_root = tmp_path.joinpath(*probe.contract.collection_case_root(case_id).parts)
    label_root = case_root / "observer_labels"
    stage_id = f"collect_label__{case_id}"
    stage_index = tuple(probe.contract.STAGE_IDS).index(stage_id) + 1
    record_paths = {
        "labels": label_root / "labels.npz",
        "label_summary": label_root / "summary.json",
        "label_gate": label_root / "gate.json",
        "label_receipt": label_root / "receipt.json",
        "pipeline_claim": tmp_path.joinpath(*probe.contract.CLAIM_PATH.parts),
        "worker_claim": tmp_path.joinpath(
            *probe.contract.RUN_ROOT.parts,
            "claims",
            f"{stage_index:02d}_{stage_id}.json",
        ),
        "probe_receipt": case_root / "receipt.json",
        "probe_gate": case_root / "gate.json",
        "probe_replay": case_root / "replay_boundaries.npz",
    }
    for index, path in enumerate(record_paths.values()):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"artifact-{index}".encode())

    offline_inputs = {
        "teacher_h0_id": "locked-teacher",
        "teacher_h0": {"path": "teacher", "tree_sha256": "a" * 64},
        "coverage_reference_corpus": {
            "path": "coverage.npz",
            "sha256": "b" * 64,
            "size_bytes": 123,
        },
    }
    stage = {
        "passed": True,
        "protocol_id": probe.contract.PROTOCOL_ID,
        "pipeline_id": probe.contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "teacher_h0_id": offline_inputs["teacher_h0_id"],
        "teacher_h0": copy.deepcopy(offline_inputs["teacher_h0"]),
        "coverage_reference_corpus": copy.deepcopy(
            offline_inputs["coverage_reference_corpus"]
        ),
        "labelled_row_count": 100,
        "same_state_teacher_label_count": 100,
        "teacher_query_count": 100,
        "single_collection_round": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        **{
            name: probe._artifact(path, tmp_path) for name, path in record_paths.items()
        },
    }
    stage_path = case_root / "stage_receipt.json"
    probe.forensic.write_json_exclusive(stage_path, stage)

    binding = probe._verify_label_stage_binding(
        case_id=case_id,
        artifact_root=tmp_path,
        label_root=label_root,
        offline_inputs=offline_inputs,
    )
    assert binding["stage"] == stage

    stage["imported_r8_prefix"] = False
    stage_path.unlink()
    probe.forensic.write_json_exclusive(stage_path, stage)
    with pytest.raises(probe.V12R9RecoveryProbeError, match="stage binding drifted"):
        probe._verify_label_stage_binding(
            case_id=case_id,
            artifact_root=tmp_path,
            label_root=label_root,
            offline_inputs=offline_inputs,
        )


def test_tampered_normalization_receipt_blocks_label_loader(tmp_path: Path) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_nominal", artifact_root=tmp_path
    )
    root, _replay = _closed_fixture(tmp_path, config=config)
    receipt = probe._strict_mapping(root / "receipt.json")
    receipt["v26_summary_normalization"]["passed"] = False
    (root / "receipt.json").unlink()
    probe.forensic.write_json_exclusive(root / "receipt.json", receipt)
    loaded = False

    def load(_path: Path) -> object:
        nonlocal loaded
        loaded = True
        return object()

    with pytest.raises(probe.V12R9RecoveryProbeError, match="normalization"):
        probe.label_recovery_probe(
            config=config,
            probe_destination=root,
            label_destination=root / "observer_labels",
            source_h0_path=tmp_path / "fake_h0",
            module_loader=load,
        )
    assert loaded is False


def test_probe_closure_rejects_rehashed_summary_identity_drift(tmp_path: Path) -> None:
    config = probe.RecoveryProbeConfig(
        case_id="deterministic_offset_nominal", artifact_root=tmp_path
    )
    root, replay = _closed_fixture(tmp_path, config=config)
    summary_path = root / "summary.json"
    gate_path = root / "gate.json"
    receipt_path = root / "receipt.json"
    summary = probe._strict_mapping(summary_path)
    summary["behavior"] = "FORGED_BEHAVIOR"
    summary_path.unlink()
    probe.forensic.write_json_exclusive(summary_path, summary)

    trace = probe._strict_sequence(root / "trace.json")
    gate = probe.recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    assert gate["passed"] is True
    gate_path.unlink()
    probe.forensic.write_json_exclusive(gate_path, gate)
    receipt = probe._strict_mapping(receipt_path)
    receipt["summary"] = probe._artifact(summary_path, tmp_path)
    receipt["gate"] = probe._artifact(gate_path, tmp_path)
    receipt_path.unlink()
    probe.forensic.write_json_exclusive(receipt_path, receipt)

    with pytest.raises(probe.V12R9RecoveryProbeError, match="summary identity"):
        probe.verify_probe_closure(root, config=config)


def test_plus_import_has_no_r9_teacher_execution_surface() -> None:
    signature = inspect.signature(
        probe.adjudicator.verify_r8_plus_label_import
    ).parameters
    assert "source_h0_path" not in signature
    assert "module_loader" not in signature
    assert "mean_query" not in signature
    imported = probe.adjudicator.verify_r8_plus_label_import(semantic_verify=False)
    assert imported["teacher_query_count"] == 0
    assert imported["direct_immutable_reference"] is True
    assert imported["labels_copied"] is False


def test_imported_minus_label_cannot_reach_h0_without_r8_adjudication(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        probe,
        "verify_r8_minus_adjudication",
        lambda **_kwargs: {"passed": False, "offline_label_authorized": False},
    )
    loaded = False

    def load(_path: Path) -> object:
        nonlocal loaded
        loaded = True
        return object()

    label_root = tmp_path.joinpath(
        *probe.contract.collection_case_root(
            probe.contract.IMPORTED_MINUS_CASE_ID
        ).parts,
        "observer_labels",
    )
    with pytest.raises(probe.V12R9RecoveryProbeError, match="not authorized"):
        probe.label_adjudicated_r8_minus(
            adjudication_destination=tmp_path / "adjudication.json",
            label_destination=label_root,
            source_h0_path=tmp_path / "h0",
            artifact_root=tmp_path,
            module_loader=load,
        )
    assert loaded is False


def test_production_probe_normalizes_before_forensic_finalize_and_gate() -> None:
    source = inspect.getsource(probe.run_recovery_probe)
    normalize_at = source.index("normalize_v26_prefix_summary")
    finalize_at = source.index("finalize_before_gate")
    gate_at = source.index("recovery_prefix_gate")
    assert normalize_at < finalize_at < gate_at
