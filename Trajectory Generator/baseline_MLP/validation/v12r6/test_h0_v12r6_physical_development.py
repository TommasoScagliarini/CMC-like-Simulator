from __future__ import annotations

import ast
import copy
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

TEST_ROOT = Path(__file__).resolve().parent
if str(TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_ROOT))

import h0_v12r6_physical_development as physical  # noqa: E402


def _config(root: Path) -> physical.PhysicalDevelopmentConfig:
    return physical.PhysicalDevelopmentConfig(
        protocol_id="AB06_H0_V12R6_TEST",
        start_status="STARTED_H0_V12R6_TEST_DEVELOPMENT",
        partial_status="PERSISTED_H0_V12R6_TEST_BEFORE_GATE",
        complete_status="COMPLETE_H0_V12R6_TEST_DEVELOPMENT",
        artifact_root=root,
        progress_every=0,
    )


def _audit_row(
    *, config: physical.PhysicalDevelopmentConfig, case_id: str, step: int
) -> dict[str, Any]:
    return {
        "step": step,
        "schema_version": config.schema_version,
        "protocol_id": config.protocol_id,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "candidate_mean": [0.25, -0.5],
        "single_noise": [0.01, -0.02],
        "raw_action": [0.26, -0.52],
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in physical.PURE_POLICY_COUNTER_FIELDS},
        "raw_sensor_sample_count": 10,
        "observer_raw_sensor_journal": {"samples": [{} for _ in range(10)]},
    }


def test_trace_audit_is_generic_pure_policy_and_fails_closed(tmp_path: Path) -> None:
    config = _config(tmp_path)
    case_id = "deterministic_offset_plus_0p20"
    trace = [
        _audit_row(config=config, case_id=case_id, step=step)
        for step in range(1, physical.EXPECTED_STEPS + 1)
    ]
    audit = physical.pure_policy_trace_audit(
        trace,
        config=config,
        case_id=case_id,
    )
    assert audit["passed"] is True
    assert audit["row_count"] == 500
    assert audit["counters"] == {
        name: 0 for name in physical.PURE_POLICY_COUNTER_FIELDS
    }

    tampered = copy.deepcopy(trace)
    tampered[12]["teacher_mean"] = [0.0, 0.0]
    tampered[24]["raw_action"][0] += 0.01
    failed = physical.pure_policy_trace_audit(
        tampered,
        config=config,
        case_id=case_id,
    )
    assert failed["passed"] is False
    assert failed["forbidden_teacher_payload_absent"] is False
    assert failed["candidate_mean_plus_noise_exact"] is False


def test_raw_journal_is_observer_only_and_clearance_is_optional() -> None:
    journal = physical.diagnostic_raw_journal(
        {
            "binary_phase_sensor_samples": [
                {
                    "time_s": 1.0,
                    "left_heel_contact": True,
                    "left_toe_contact": False,
                }
            ],
            "binary_phase_fsm": {
                "events_this_step": [{"event": "heel_strike"}],
                "pending_event": None,
            },
        },
        step=1,
    )
    assert journal["observer_only"] is True
    assert journal["control_dependency"] is False
    assert journal["gate_dependency"] is False
    assert journal["blocker_if_field_unavailable"] is False
    assert journal["accepted_events"] == [{"event": "heel_strike"}]
    assert journal["samples"][0]["left_heel_clearance_m"] is None
    assert journal["availability"] == {
        "binary_contact_samples": True,
        "clearance": False,
        "per_sensor_analog_grf": False,
        "step_online_grf": False,
    }


def test_mocked_512_module_runs_exact_500_step_forensic_topology(
    tmp_path: Path, monkeypatch: Any
) -> None:
    config = _config(tmp_path)
    destination = tmp_path / "development" / "deterministic_offset_plus_0p20"
    module_path = tmp_path / "module"
    module_path.mkdir()
    case = {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": 2.156870983805102,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
        "behavior": "R6_PURE_UNBLENDED_NO_TEACHER_NO_LATCH",
    }
    calls = defaultdict(int)
    checkpoint_paths: list[Path] = []

    class FakeModule:
        hidden_widths = (512, 512)

        @classmethod
        def from_checkpoint(cls, path: Path) -> FakeModule:
            calls["checkpoint"] += 1
            checkpoint_paths.append(Path(path))
            return cls()

        def eval(self) -> None:
            calls["eval"] += 1

    class FakeActionSpace:
        low = np.asarray([-1.0, -1.0], dtype=np.float32)
        high = np.asarray([1.0, 1.0], dtype=np.float32)

    class FakeEnv:
        action_space = FakeActionSpace()

        def reset(self, *, seed: int) -> tuple[np.ndarray, dict[str, Any]]:
            calls["reset"] += 1
            assert seed == 123
            return np.zeros(84, dtype=np.float32), {}

        def step(
            self, action: np.ndarray
        ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
            calls["step"] += 1
            assert action.shape == (2,)
            assert action.dtype == np.float32
            step = calls["step"]
            raw_samples = [
                {
                    "time_s": step * 0.01 + sensor * 0.001,
                    "left_heel_contact": True,
                    "left_toe_contact": False,
                }
                for sensor in range(10)
            ]
            return (
                np.zeros(84, dtype=np.float32),
                1.0,
                False,
                step == physical.EXPECTED_STEPS,
                {
                    "time": step * 0.01,
                    "end_reason": "episode_time_limit",
                    "binary_phase_sensor_samples": raw_samples,
                    "binary_phase_fsm": {
                        "events_this_step": [],
                        "pending_event": None,
                    },
                },
            )

        def close(self) -> None:
            calls["close"] += 1

    env = FakeEnv()

    class FakeFactory:
        @staticmethod
        def make_cmc_env(env_config: dict[str, Any]) -> FakeEnv:
            calls["make_env"] += 1
            assert env_config == {"case_id": case["case_id"]}
            return env

    class FakeLegacy:
        @staticmethod
        def _jsonable(value: Any) -> Any:
            return value

    fake_audit = {
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
        "nonfinite_count": 0,
    }

    monkeypatch.setattr(
        physical.runtime,
        "_frozen_innovations",
        lambda _case_id, *, action_selection, np: np.zeros(
            (physical.EXPECTED_STEPS, physical.EXPECTED_ACTION_DIM),
            dtype=np.float32,
        ),
    )

    def validate_layout(**kwargs: Any) -> tuple[tuple[str, ...], tuple[str, ...]]:
        assert kwargs["module"].hidden_widths == (512, 512)
        assert kwargs["observation"].shape == (84,)
        return (
            tuple(f"actor_{index}" for index in range(35)),
            tuple(f"full_{index}" for index in range(84)),
        )

    monkeypatch.setattr(physical.runtime, "_validate_runtime_layout", validate_layout)
    monkeypatch.setattr(
        physical.runtime,
        "_new_physical_audit",
        lambda **_kwargs: fake_audit,
    )

    def query_mean_std(
        module: FakeModule, actor: np.ndarray, *, np: Any, torch: Any
    ) -> tuple[np.ndarray, np.ndarray]:
        del torch
        calls["query"] += 1
        assert module.hidden_widths == (512, 512)
        assert actor.shape == (35,)
        return (
            np.asarray([0.25, -0.5], dtype=np.float32),
            np.asarray([0.005, 0.005], dtype=np.float32),
        )

    monkeypatch.setattr(physical.runtime, "_query_mean_std", query_mean_std)
    monkeypatch.setattr(
        physical.runtime,
        "_consume_physical_step",
        lambda _audit, **_kwargs: {
            "penetration_m": 0.0,
            "reserve_norm_nm": 0.0,
            "residual_norm_nm": 0.0,
            "phase": {"state_name": "STANCE"},
            "checks": {"passed": True},
        },
    )

    def physical_summary(_audit: Any, **kwargs: Any) -> dict[str, Any]:
        assert len(kwargs["rows"]) == 500
        return {
            "steps": 500,
            "control_window_count": 5000,
            "raw_sensor_sample_count": 5000,
            "phase_valid_cycle_count": 2,
            "grf_penetration_max_m": 0.0,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
        }

    monkeypatch.setattr(physical.runtime, "_physical_summary", physical_summary)
    activity = defaultdict(int)

    def activity_callback(name: str, amount: int) -> None:
        activity[name] += amount

    result = physical.run_case(
        config=config,
        case=case,
        destination=destination,
        module_path=module_path,
        activity_callback=activity_callback,
        start_metadata={"candidate_id": "R6:mock"},
        summary_metadata={
            "candidate_id": "R6:mock",
            "target_contract_id": "TARGET_V26_TEST",
            "event_contract_id": "EVENT_V26_TEST",
        },
        stack_loader=lambda: (
            object(),
            np,
            object(),
            FakeModule,
            FakeFactory,
            FakeLegacy,
            object(),
        ),
        env_config_builder=lambda built_case: {"case_id": built_case["case_id"]},
    )

    assert calls == {
        "checkpoint": 1,
        "eval": 1,
        "make_env": 1,
        "reset": 1,
        "query": 500,
        "step": 500,
        "close": 1,
    }
    assert checkpoint_paths == [module_path]
    assert dict(activity) == {
        "environment_reset_calls": 1,
        "environment_step_calls": 500,
        "raw_sensor_sample_count": 5000,
    }
    assert result["pure_policy_trace_audit"]["passed"] is True
    assert result["summary"]["sea_reserve_gate_passed"] is True
    assert result["summary"]["candidate_id"] == "R6:mock"
    assert result["summary"]["target_contract_id"] == "TARGET_V26_TEST"
    assert len(result["trace"]) == 500
    assert result["trace"][0]["v26_observation"] == [0.0] * 35
    assert result["trace"][0]["raw_action"] == [0.25, -0.5]
    assert result["trace"][-1]["truncated"] is True

    assert (destination / "run_start.json").is_file()
    assert len(list((destination / "steps").glob("*.json"))) == 500
    assert (destination / "trace.json").is_file()
    assert (destination / "partial_summary.json").is_file()
    assert (destination / "summary.json").is_file()
    assert not (destination / "gate.json").exists()
    assert not (destination / "receipt.json").exists()
    assert not (destination / "pipeline_ledger.json").exists()
    persisted_trace = physical.forensic.strict_json_load(destination / "trace.json")
    persisted_summary = physical.forensic.strict_json_load(destination / "summary.json")
    assert persisted_trace == result["trace"]
    assert persisted_summary == result["summary"]
    assert set(result["artifacts"]) == {"trace", "partial_summary", "summary"}


def test_source_closure_has_no_r5_global_or_training_and_counts_before_consume() -> (
    None
):
    source = Path(physical.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    attributes = {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }
    assert "run_h0_v12r5_case_balanced" not in imported
    assert "h0_v12r5_case_balanced_contract" not in imported
    assert "h0_v12r5_case_balanced_contract" not in source
    assert "fit_engine" not in source
    assert "learn_on_batch" not in attributes
    assert "train" not in attributes
    assert "publish_gate" not in attributes
    assert "write_json_exclusive" not in attributes
    assert "runtime._load_rollout_stack()" in source
    assert "EXPECTED_STEPS = 500" in source
    assert "EXPECTED_ACTOR_FEATURES = 35" in source
    assert "EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10" in source

    start = source.index("def run_case")
    rollout = source[start:]
    raw_validation = rollout.index(
        'raw_samples = info.get("binary_phase_sensor_samples")'
    )
    raw_activity = rollout.index('"raw_sensor_sample_count",', raw_validation)
    consume = rollout.index("runtime._consume_physical_step", raw_activity)
    assert raw_validation < raw_activity < consume
