"""Pure/mocked tests for the V6 P1 zero-update port scaffold."""

from __future__ import annotations

import ast
import inspect
import json
import sys
import tempfile
from pathlib import Path
from unittest import mock

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(BASELINE))

from validation import h0_primary_split_v6_zero_update_contract as contract  # noqa: E402
from validation import freeze_h0_primary_split_v6_zero_update as freezer  # noqa: E402
from validation import run_h0_primary_split_v6_zero_update_port as driver  # noqa: E402


def _residual_config() -> dict:
    return {
        "input_mean": [0.0] * 33,
        "input_std": [1.0] * 33,
        "limits": [0.175, 0.12],
        "init_seed": 20260806,
        "input_indices": list(range(2, 35)),
        "architecture": [33, 128, 128, 2],
    }


def test_contract_is_zero_update_and_never_authorizes_training() -> None:
    assert contract.TARGET_FIXED_CONFIG["iterations"] == 0
    assert contract.TARGET_FIXED_CONFIG["rl_module_kind"] == (
        "primary_split_v25_residual"
    )
    assert contract.TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
    assert contract.MORPHOLOGY_WEIGHT == 0.0
    assert not contract.AUTHORITY["actor_updates_authorized"]
    assert not contract.AUTHORITY["critic_updates_authorized"]
    assert not contract.AUTHORITY["ppo_updates_authorized"]
    assert not contract.AUTHORITY["environment_sampling_authorized"]
    assert "same_p1_freeze_development_qualification" in contract.REQUIRED_CHECKS
    assert "no_algorithm_train_call" in contract.REQUIRED_CHECKS


def test_runtime_source_contains_no_algorithm_train_call() -> None:
    tree = ast.parse(inspect.getsource(driver._runtime_port))  # noqa: SLF001
    train_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "train"
    ]
    assert train_calls == []


def test_residual_config_is_exact_and_finite() -> None:
    assert driver.validate_residual_config(_residual_config()) == _residual_config()
    bad = _residual_config()
    bad["input_std"][3] = 0.0
    with pytest.raises(driver.ZeroUpdatePortError, match="invalid value"):
        driver.validate_residual_config(bad)
    bad = _residual_config()
    bad["input_indices"] = list(range(1, 34))
    with pytest.raises(driver.ZeroUpdatePortError, match="indices drifted"):
        driver.validate_residual_config(bad)
    bad = _residual_config()
    bad["limits"] = [0.2, 0.12]
    with pytest.raises(driver.ZeroUpdatePortError, match="limits drifted"):
        driver.validate_residual_config(bad)


def test_full_policy_transplant_includes_base_residual_buffers_and_logstd() -> None:
    candidate = {
        "pi.0.weight": np.array([[1.0, 2.0]], dtype=np.float32),
        "pi.1.bias": np.array([0.4, -0.2, -1.1, -1.2], dtype=np.float32),
        "primary_split_v25_residual_input_mean": np.zeros(33, dtype=np.float32),
        "primary_split_v25_residual_input_std": np.ones(33, dtype=np.float32),
        "primary_split_v25_residual_limits": np.array([0.175, 0.12], dtype=np.float32),
        "primary_split_v25_residual.0.weight": np.ones((128, 33), dtype=np.float32),
    }
    target = {key: np.zeros_like(value) for key, value in candidate.items()} | {
        "vf.weight": np.array([[9.0]], dtype=np.float32),
        "vf_encoder.0.weight": np.array([[8.0]], dtype=np.float32),
    }
    merged, report = driver.transplant_policy_state(
        target_state=target, candidate_state=candidate
    )
    assert driver.compare_states(candidate, driver.policy_state(merged))["exact"]
    assert driver.compare_states(
        driver.critic_state(target), driver.critic_state(merged)
    )["exact"]
    assert report["residual_key_count"] == 4
    assert report["base_actor_key_count"] == 2
    assert report["logstd_container"].startswith("pi final-layer")


def test_policy_transplant_rejects_source_critic_and_missing_residual() -> None:
    with pytest.raises(driver.ZeroUpdatePortError, match="contains a critic"):
        driver.transplant_policy_state(
            target_state={"pi.weight": np.ones(1), "vf.weight": np.ones(1)},
            candidate_state={"pi.weight": np.ones(1), "vf.weight": np.ones(1)},
        )
    with pytest.raises(driver.ZeroUpdatePortError, match="base actor or residual"):
        driver.transplant_policy_state(
            target_state={"pi.weight": np.ones(1)},
            candidate_state={"pi.weight": np.ones(1)},
        )


class _Parameter:
    pass


class _Module:
    def __init__(self) -> None:
        self.base = _Parameter()
        self.residual = _Parameter()

    def named_parameters(self):
        return [
            ("pi.weight", self.base),
            ("primary_split_v25_residual.0.weight", self.residual),
        ]


class _Optimizer:
    def __init__(self, parameters, *, state=None) -> None:
        self.param_groups = [{"params": list(parameters)}]
        self._state = {} if state is None else state

    def state_dict(self):
        return {"state": self._state, "param_groups": [{}]}


class _Learner:
    def __init__(self, *, include_residual=True, state=None) -> None:
        self.module = {contract.DEFAULT_POLICY_ID: _Module()}
        module = self.module[contract.DEFAULT_POLICY_ID]
        parameters = [module.base]
        if include_residual:
            parameters.append(module.residual)
        self.optimizer = _Optimizer(parameters, state=state)

    def get_optimizers_for_module(self, module_id):
        assert module_id == contract.DEFAULT_POLICY_ID
        return [("ppo", self.optimizer)]


def test_optimizer_must_be_fresh_and_cover_every_residual_parameter() -> None:
    report = driver.optimizer_empty_and_residual_registered_on_learner(_Learner())
    assert report["optimizer_state_empty"]
    assert report["all_residual_parameters_registered"]
    with pytest.raises(driver.ZeroUpdatePortError, match="missing from optimizer"):
        driver.optimizer_empty_and_residual_registered_on_learner(
            _Learner(include_residual=False)
        )
    with pytest.raises(driver.ZeroUpdatePortError, match="non-empty"):
        driver.optimizer_empty_and_residual_registered_on_learner(
            _Learner(state={1: {"step": 1}})
        )


def test_target_args_serialize_frozen_residual_and_zero_morphology() -> None:
    import train_ppo_mlp as train
    import training_config

    lock = {"residual_model_config": _residual_config()}
    args = driver._target_training_args(  # noqa: SLF001
        train, lock, Path("unused_zero_update")
    )
    assert args.iterations == 0
    assert args.rl_module_kind == "primary_split_v25_residual"
    assert args.binary_phase_fsm_mode == "binary_active"
    assert args.freeze_logstd
    assert not args.freeze_actor
    assert args.primary_split_v25_residual_input_mean == [0.0] * 33
    assert args.primary_split_v25_residual_limits == [0.175, 0.12]
    assert args._cfg_reward["morphology_weight"] == 0.0
    with tempfile.TemporaryDirectory() as tmp:
        snapshot = Path(tmp) / training_config.RESOLVED_CONFIG_NAME
        training_config.dump_resolved(args, args._cfg_reward, snapshot)
        with mock.patch.object(driver, "REPO_ROOT", Path(tmp)):
            audit = driver.audit_resolved_config(snapshot, _residual_config())
    assert audit["morphology_weight"] == 0.0
    assert audit["residual_constructor"]["rl_module_kind"] == (
        "primary_split_v25_residual"
    )


def _write(path: Path, payload: bytes = b"x") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def test_execution_lock_binds_all_sources_and_inputs_no_clobber() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        lock_path = root / "validation" / "lock.json"
        output = root / "validation" / "run" / "zero"
        inputs = {
            name: root / "inputs" / f"{name}.bin" for name in contract.INPUT_PATHS
        }
        sources = {
            name: root / "sources" / f"{name}.py" for name in contract.SOURCE_PATHS
        }
        for path in [*inputs.values(), *sources.values()]:
            _write(path)

        with (
            mock.patch.object(driver, "REPO_ROOT", root),
            mock.patch.object(driver, "LOCK", lock_path),
            mock.patch.object(driver, "OUTPUT_ROOT", output),
            mock.patch.object(driver, "INPUT_PATHS", inputs),
            mock.patch.object(driver, "SOURCE_PATHS", sources),
        ):
            payload = {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.LOCK_STATUS,
                "protocol_id": contract.PROTOCOL_ID,
                "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
                "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
                "revision": contract.REVISION,
                "candidate_id": "H0_primary_split_v6_v25_residual_p1",
                "so_policy_id": contract.SO_POLICY_ID,
                "target_bundle_contract_id": contract.TARGET_BUNDLE_CONTRACT_ID,
                "output_root": driver.repo_relative(output),
                "target_fixed_config": contract.TARGET_FIXED_CONFIG,
                "residual_model_config": _residual_config(),
                "authority": contract.AUTHORITY,
                "required_checks": list(contract.REQUIRED_CHECKS),
                "sources": {
                    name: driver.source_record(path) for name, path in sources.items()
                },
                "inputs": {
                    name: driver.source_record(path) for name, path in inputs.items()
                },
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "environment_samples": 0,
                "protected_trials_opened": [],
            }
            _write(lock_path, json.dumps(payload, allow_nan=False).encode())
            assert driver.verify_lock()["candidate_id"].endswith("p1")
            inputs["qualification_receipt"].write_bytes(b"drift")
            with pytest.raises(driver.ZeroUpdatePortError, match="drifted"):
                driver.verify_lock()


def test_freezer_requires_same_qualified_p1_and_records_model_config() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        lock_path = root / "validation" / "zero_lock.json"
        output = root / "validation" / "run" / "zero"
        candidate_dir = root / "candidate"
        inputs = {
            "candidate_freeze": root / "candidate_freeze.json",
            "development_receipt": root / "development.json",
            "qualification_receipt": root / "qualification.json",
            "qualification_ledger": root / "qualification_ledger.json",
            "candidate_module_state": candidate_dir / "module_state.pkl",
            "candidate_module_ctor": candidate_dir / "class_and_ctor_args.pkl",
            "candidate_module_metadata": candidate_dir / "metadata.json",
            "source_training_config": root / "source_training.yaml",
            "v25_binary_profile": root / "v25.json",
            "legacy_analog_detector_profile": root / "analog.json",
        }
        sources = {
            name: root / "sources" / f"{name}.py" for name in contract.SOURCE_PATHS
        }
        for path in [*inputs.values(), *sources.values()]:
            _write(path)
        candidate_id = "H0_primary_split_v6_v25_residual_p1"
        with (
            mock.patch.object(driver, "REPO_ROOT", root),
            mock.patch.object(driver, "LOCK", lock_path),
            mock.patch.object(driver, "OUTPUT_ROOT", output),
            mock.patch.object(driver, "INPUT_PATHS", inputs),
            mock.patch.object(driver, "SOURCE_PATHS", sources),
            mock.patch.object(driver, "CANDIDATE_DIR", candidate_dir),
        ):
            architecture = {
                "residual_input_indices": list(range(2, 35)),
                "residual_input_count": 33,
                "hidden_dims": [128, 128],
                "action_dim": 2,
                "limits": [0.175, 0.12],
                "init_seed": 20260806,
            }
            model_config = {
                "primary_split_v25_residual_input_mean": [0.0] * 33,
                "primary_split_v25_residual_input_std": [1.0] * 33,
                "primary_split_v25_residual_limits": [0.175, 0.12],
                "primary_split_v25_residual_init_seed": 20260806,
                "primary_split_v25_residual_reset_bypass": False,
            }
            candidate_freeze = {
                "status": contract.CANDIDATE_FREEZE_STATUS,
                "passed": True,
                "candidate_id": candidate_id,
                "candidate_module": driver.tree_record(candidate_dir),
                "target_contract_id": contract.TARGET_BUNDLE_CONTRACT_ID,
                "dagger_rounds": 1,
                "architecture": architecture,
                "model_config": model_config,
            }
            receipts = {
                "candidate_freeze": candidate_freeze,
                "development_receipt": {
                    "status": contract.DEVELOPMENT_PASS_STATUS,
                    "passed": True,
                    "candidate_id": candidate_id,
                },
                "qualification_receipt": {
                    "status": contract.QUALIFICATION_PASS_STATUS,
                    "passed": True,
                    "candidate_id": candidate_id,
                    "protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
                    "morphology_weight": 0.0,
                },
                "qualification_ledger": {
                    "status": contract.QUALIFICATION_PASS_STATUS,
                    "passed": True,
                    "candidate_id": candidate_id,
                    "protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
                },
            }
            for name, payload in receipts.items():
                inputs[name].write_text(
                    json.dumps(payload, allow_nan=False), encoding="utf-8"
                )
            payload = freezer.build_lock()
            assert payload["candidate_id"] == candidate_id
            assert payload["residual_model_config"] == _residual_config()
            freezer.freeze()
            assert driver.verify_lock()["candidate_id"] == candidate_id
            with pytest.raises(driver.ZeroUpdatePortError, match="clobber"):
                freezer.freeze()

            # A qualification receipt for another candidate can never be mixed.
            lock_path.unlink()
            qualification = receipts["qualification_receipt"] | {
                "candidate_id": "different_p1"
            }
            inputs["qualification_receipt"].write_text(
                json.dumps(qualification), encoding="utf-8"
            )
            with pytest.raises(driver.ZeroUpdatePortError, match="one P1"):
                freezer.build_lock()
