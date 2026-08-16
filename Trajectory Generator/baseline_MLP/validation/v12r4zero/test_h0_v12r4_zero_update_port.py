"""Source-only tests for the deferred V12R4/Q2 zero-update lineage."""

from __future__ import annotations

import ast
import json
import os
import stat
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest import mock

import numpy as np
import pytest


ZERO_ROOT = Path(__file__).resolve().parent
VALIDATION_ROOT = ZERO_ROOT.parent
for import_root in (ZERO_ROOT, VALIDATION_ROOT):
    text_root = os.fspath(import_root)
    if text_root not in sys.path:
        sys.path.insert(0, text_root)

import freeze_h0_v12r4_zero_update as freezer  # noqa: E402
import h0_v12r4_zero_update_contract as contract  # noqa: E402
import run_h0_v12r4_zero_update_port as driver  # noqa: E402


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def test_contract_is_candidate_deferred_standard_v26_and_zero_effect() -> None:
    gate = contract.contract_self_check()

    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert contract.CANDIDATE_ID is None
    assert contract.CANDIDATE_MODULE is None
    assert contract.SOURCE_TOPOLOGY_ID == "V5_STANDARD_FULL_MEAN_ZERO_PORT"
    assert contract.STANDARD_RL_MODULE_KIND == "standard"
    assert "residual" not in contract.SOURCE_TOPOLOGY_ID.lower()
    assert contract.TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
    assert (
        contract.TARGET_FIXED_CONFIG["binary_phase_event_contract_id"]
        == contract.q2.EVENT_CONTRACT_ID
    )
    assert contract.TARGET_REWARD_CONFIG["morphology_weight"] == 0.0
    assert contract.TARGET_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
    assert contract.TARGET_FIXED_CONFIG["iterations"] == 0
    assert contract.TARGET_FIXED_CONFIG["num_env_runners"] == 0
    assert contract.AUTHORITY["algorithm_build_authorized_now"] is False
    assert contract.AUTHORITY["checkpoint_publication_authorized_now"] is False


def test_q2_protocol_receipt_and_ledger_paths_are_contract_aliases() -> None:
    q2 = contract.q2

    assert contract.Q2_PROTOCOL_FREEZE_PATH == q2.PROTOCOL_FREEZE_PATH
    assert contract.Q2_FINAL_RECEIPT_PATH == q2.FINAL_RECEIPT_PATH
    assert contract.Q2_PIPELINE_LEDGER_PATH == q2.PIPELINE_LEDGER_PATH
    assert (
        contract.INPUT_RELATIVE_PATHS["q2_protocol_freeze"]
        == q2.PROTOCOL_FREEZE_PATH.as_posix()
    )
    assert (
        contract.INPUT_RELATIVE_PATHS["q2_final_receipt"]
        == q2.FINAL_RECEIPT_PATH.as_posix()
    )
    assert (
        contract.INPUT_RELATIVE_PATHS["q2_pipeline_ledger"]
        == q2.PIPELINE_LEDGER_PATH.as_posix()
    )
    assert contract.CANDIDATE_MODULE_PATH == q2.R4_CANDIDATE_MODULE_PATH


def test_real_v26_target_config_and_profile_hashes_are_exact() -> None:
    target = driver.validate_v26_target_config()

    assert target["grf"]["binary_phase_fsm_mode"] == "binary_active"
    assert (
        target["grf"]["binary_phase_event_contract_id"]
        == contract.BINARY_EVENT_CONTRACT_ID
    )
    assert target["reward"]["morphology_weight"] == 0.0
    assert target["reward"]["morphology_causal_allow_effects"] == 0.0


def test_source_runner_has_no_train_call_and_has_full_save_reload() -> None:
    source = Path(driver.__file__).read_text(encoding="utf-8")
    syntax = ast.parse(source)
    attributes = [
        node.func.attr
        for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]

    assert "train" not in attributes
    assert "save_to_path" in attributes
    assert "restore_from_path" in attributes
    assert set(contract.CHECKPOINT_REQUIRED_SUFFIXES) == {
        "algorithm_state.pkl",
        "class_and_ctor_args.pkl",
        "rllib_checkpoint.json",
        "learner_group/learner/state.pkl",
        "learner_group/learner/rl_module/module_state.pkl",
        "env_runner/state.pkl",
    }


def test_no_real_lock_output_or_checkpoint_was_published() -> None:
    assert not os.path.lexists(driver.LOCK)
    assert not os.path.lexists(driver.OUTPUT_ROOT)


class _Parameter:
    def __init__(self, *, requires_grad: bool = True) -> None:
        self.requires_grad = requires_grad


class _Module:
    def __init__(self, rows: list[tuple[str, _Parameter]]) -> None:
        self._rows = rows

    def named_parameters(self) -> list[tuple[str, _Parameter]]:
        return list(self._rows)


class _Optimizer:
    def __init__(
        self,
        groups: list[dict[str, Any]],
        *,
        state_entries: dict[Any, Any] | None = None,
    ) -> None:
        self.param_groups = groups
        self._state_entries = state_entries or {}

    def state_dict(self) -> dict[str, Any]:
        return {"state": self._state_entries, "param_groups": []}


class _Learner:
    def __init__(self, module: _Module, optimizer: _Optimizer) -> None:
        self.module = {contract.DEFAULT_POLICY_ID: module}
        self._optimizer = optimizer

    def get_optimizers_for_module(self, module_id: str) -> list[tuple[str, Any]]:
        assert module_id == contract.DEFAULT_POLICY_ID
        return [("actor_critic", self._optimizer)]


def test_optimizer_state_empty_and_exact_param_groups() -> None:
    actor = _Parameter()
    critic = _Parameter()
    module = _Module([("pi.weight", actor), ("vf.weight", critic)])
    optimizer = _Optimizer(
        [
            {"params": [actor], "lr": 1.0e-4, "betas": (0.9, 0.999)},
            {"params": [critic], "lr": 2.0e-4, "betas": (0.9, 0.999)},
        ]
    )

    snapshot = driver.optimizer_snapshot_on_learner(_Learner(module, optimizer))

    assert snapshot["optimizer_state_empty"] is True
    assert snapshot["all_trainable_parameters_registered_once"] is True
    assert snapshot["trainable_parameter_names"] == ["pi.weight", "vf.weight"]
    groups = snapshot["optimizers"][0]["param_groups"]
    assert groups[0]["parameter_names"] == ["pi.weight"]
    assert groups[1]["parameter_names"] == ["vf.weight"]
    assert groups[0]["options"]["lr"] == 1.0e-4

    dirty = _Optimizer(
        [{"params": [actor, critic], "lr": 1.0e-4}],
        state_entries={1: {"step": 1}},
    )
    with pytest.raises(driver.ZeroUpdatePortError, match="not empty"):
        driver.optimizer_snapshot_on_learner(_Learner(module, dirty))

    duplicate = _Optimizer(
        [
            {"params": [actor], "lr": 1.0e-4},
            {"params": [actor, critic], "lr": 1.0e-4},
        ]
    )
    with pytest.raises(driver.ZeroUpdatePortError, match="duplicated"):
        driver.optimizer_snapshot_on_learner(_Learner(module, duplicate))

    missing = _Optimizer([{"params": [actor], "lr": 1.0e-4}])
    with pytest.raises(driver.ZeroUpdatePortError, match="missing"):
        driver.optimizer_snapshot_on_learner(_Learner(module, missing))


class _WarmStart:
    @staticmethod
    def actor_state_digest(state: dict[str, np.ndarray]) -> str:
        array = np.asarray(state["actor"])
        return f"digest:{array.tobytes().hex()}"

    @staticmethod
    def compare_actor_states(
        expected: dict[str, np.ndarray], actual: dict[str, np.ndarray]
    ) -> dict[str, Any]:
        exact = "actor" in actual and np.array_equal(expected["actor"], actual["actor"])
        return {"exact": exact, "max_abs_diff": 0.0 if exact else float("inf")}

    @staticmethod
    def compare_non_actor_states(
        expected: dict[str, np.ndarray], actual: dict[str, np.ndarray]
    ) -> dict[str, Any]:
        expected_keys = sorted(set(expected) - {"actor"})
        actual_keys = sorted(set(actual) - {"actor"})
        missing = sorted(set(expected_keys) - set(actual_keys))
        unexpected = sorted(set(actual_keys) - set(expected_keys))
        exact = (
            bool(expected_keys)
            and not missing
            and not unexpected
            and all(np.array_equal(expected[key], actual[key]) for key in expected_keys)
        )
        return {
            "keys": expected_keys,
            "missing_keys": missing,
            "unexpected_keys": unexpected,
            "exact": exact,
        }


def test_standard_actor_transplant_is_exact_and_critic_preserving() -> None:
    candidate = {"actor": np.array([1.0, 2.0], dtype=np.float32)}
    target = {
        "actor": np.array([-1.0, -2.0], dtype=np.float32),
        "critic": np.array([7.0], dtype=np.float32),
    }

    merged, audit = driver.transplant_standard_actor(
        target_state=target,
        candidate_state=candidate,
        warm_start=_WarmStart,
    )

    assert np.array_equal(merged["actor"], candidate["actor"])
    assert np.array_equal(merged["critic"], target["critic"])
    assert audit["residual_parameter_count"] == 0
    assert audit["fresh_critic_preserved"]["exact"] is True

    residual_candidate = {
        **candidate,
        "primary_split_v25_residual.layer": np.array([0.0], dtype=np.float32),
    }
    with pytest.raises(driver.ZeroUpdatePortError, match="standard topology"):
        driver.transplant_standard_actor(
            target_state=target,
            candidate_state=residual_candidate,
            warm_start=_WarmStart,
        )


def test_tree_and_publication_reject_symlink_junction_and_clobber(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path.resolve()
    monkeypatch.setattr(driver, "REPO_ROOT", root)
    regular = root / "tree" / "module_state.pkl"
    regular.parent.mkdir()
    regular.write_bytes(b"actor")
    linked = regular.parent / "alias.pkl"
    linked.symlink_to(regular)

    with pytest.raises(driver.ZeroUpdatePortError, match="symlink/junction"):
        driver.tree_record(regular.parent)

    fake_status = SimpleNamespace(
        st_mode=stat.S_IFDIR | 0o755,
        st_file_attributes=0x400,
    )
    with mock.patch.object(driver.os, "lstat", return_value=fake_status):
        assert driver._is_link_or_reparse(root / "junction") is True  # noqa: SLF001

    linked.unlink()
    destination = root / "receipt.json"
    driver.write_json_exclusive(destination, {"passed": True})
    with pytest.raises(driver.ZeroUpdatePortError, match="clobber"):
        driver.write_json_exclusive(destination, {"passed": False})


def _sandbox_world(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    root = tmp_path.resolve()
    lock = root / "zero" / "execution_lock.json"
    output = root / "zero" / "run"
    candidate_dir = root / "r4" / "candidate"
    candidate_dir.mkdir(parents=True)
    (candidate_dir / "module_state.pkl").write_bytes(b"actor-only-standard-full-mean")
    (candidate_dir / "class_and_ctor_args.pkl").write_bytes(b"standard-module")
    _write_json(candidate_dir / "metadata.json", {"module": "standard"})

    sources = {
        name: root / "sources" / f"{index:02d}_{name}.py"
        for index, name in enumerate(contract.SOURCE_RELATIVE_PATHS)
    }
    for name, path in sources.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"# frozen source {name}\n", encoding="utf-8")

    inputs = {
        name: root / "inputs" / f"{index:02d}_{name}.json"
        for index, name in enumerate(contract.INPUT_RELATIVE_PATHS)
    }
    for name, path in inputs.items():
        _write_json(path, {"placeholder": name})

    monkeypatch.setattr(driver, "REPO_ROOT", root)
    monkeypatch.setattr(driver, "LOCK", lock)
    monkeypatch.setattr(driver, "OUTPUT_ROOT", output)
    monkeypatch.setattr(driver, "CANDIDATE_DIR", candidate_dir)
    monkeypatch.setattr(driver, "SOURCE_PATHS", sources)
    monkeypatch.setattr(driver, "INPUT_PATHS", inputs)
    monkeypatch.setattr(driver, "validate_v26_target_config", lambda: {"passed": True})

    module = driver.tree_record(candidate_dir)
    candidate_id = contract.candidate_id_for_tree(module["tree_sha256"])
    r4_common = {
        "passed": True,
        "protocol_id": "AB06_H0_V12R4_P3_COVERAGE",
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json(
        inputs["r4_candidate_freeze_receipt"],
        {
            **r4_common,
            "status": contract.q2.R4_CANDIDATE_FREEZE_PASS_STATUS,
            "source_h0_byte_exact": True,
            "logstd_byte_exact": True,
            "critic_present": False,
            "save_reload_exact": True,
            "q2_paths_opened": [],
            "actor_updates": 0,
        },
    )
    _write_json(
        inputs["r4_final_development_receipt"],
        {
            **r4_common,
            "status": contract.q2.R4_FINAL_DEVELOPMENT_PASS_STATUS,
            "actor_updates": 1,
        },
    )
    _write_json(
        inputs["r4_pipeline_ledger"],
        {
            **r4_common,
            "status": contract.q2.R4_PIPELINE_PASS_STATUS,
            "terminal": True,
            "error": None,
            "next_stage": "WAIT_SEPARATE_Q2_PROTOCOL",
            "qualification_executed": False,
            "runtime_promoted": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
            "q2_paths_opened": [],
            "actor_updates": 1,
        },
    )
    q2_common = {
        "passed": True,
        "protocol_id": contract.q2.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json(
        inputs["q2_protocol_freeze"],
        {
            **q2_common,
            "status": contract.q2.PROTOCOL_FREEZE_PASS_STATUS,
            "candidate_binding_state": "BOUND",
            "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        },
    )
    _write_json(
        inputs["q2_final_receipt"],
        {
            **q2_common,
            "status": contract.q2.AGGREGATE_PASS_STATUS,
            "next_stage": contract.q2.NEXT_STAGE_AFTER_Q2_PASS,
        },
    )
    _write_json(
        inputs["q2_pipeline_ledger"],
        {
            **q2_common,
            "status": contract.q2.AGGREGATE_PASS_STATUS,
            "terminal": True,
            "error": None,
            "runtime_promoted": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
            "next_stage": contract.q2.NEXT_STAGE_AFTER_Q2_PASS,
        },
    )
    return {
        "root": root,
        "lock": lock,
        "output": output,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "sources": sources,
        "inputs": inputs,
    }


def test_freezer_builds_only_an_in_memory_exact_post_q2_binding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)

    payload = freezer.build_lock()

    assert payload["candidate_binding_state"] == "BOUND_AFTER_Q2_TERMINAL_PASS"
    assert payload["candidate_id"] == world["candidate_id"]
    assert payload["candidate_module"] == world["candidate_module"]
    assert payload["source_topology_id"] == "V5_STANDARD_FULL_MEAN_ZERO_PORT"
    assert payload["target_fixed_config"]["rl_module_kind"] == "standard"
    assert payload["target_reward_config"]["morphology_weight"] == 0.0
    assert payload["actor_updates"] == 0
    assert payload["critic_updates"] == 0
    assert payload["ppo_updates"] == 0
    assert payload["environment_samples"] == 0
    assert not Path(world["lock"]).exists()
    assert not Path(world["output"]).exists()


def test_freezer_rejects_cross_candidate_q2_binding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)
    inputs = world["inputs"]
    receipt = json.loads(inputs["q2_final_receipt"].read_text(encoding="utf-8"))
    receipt["candidate_id"] = "h0_v12r4_p3::" + "f" * 64
    _write_json(inputs["q2_final_receipt"], receipt)

    with pytest.raises(freezer.ZeroUpdateFreezeError, match="exact R4 candidate"):
        freezer.build_lock()
    assert not Path(world["lock"]).exists()
    assert not Path(world["output"]).exists()


def test_freezer_rehash_detects_toctou_source_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)
    original = freezer.validate_prerequisites

    def validate_then_mutate() -> tuple[str, dict[str, Any]]:
        result = original()
        world["sources"]["runner"].write_text("# drift after validation\n")
        return result

    monkeypatch.setattr(freezer, "validate_prerequisites", validate_then_mutate)
    with pytest.raises(freezer.ZeroUpdateFreezeError, match="closure drifted"):
        freezer.build_lock()
    assert not Path(world["lock"]).exists()
    assert not Path(world["output"]).exists()


def test_full_checkpoint_tree_requires_every_rllib_component(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path.resolve()
    monkeypatch.setattr(driver, "REPO_ROOT", root)
    checkpoint = root / "checkpoint_zero"
    for relative in contract.CHECKPOINT_REQUIRED_SUFFIXES:
        path = checkpoint / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode("utf-8"))

    tree = driver._assert_full_checkpoint(checkpoint)  # noqa: SLF001
    assert tree["file_count"] == len(contract.CHECKPOINT_REQUIRED_SUFFIXES)

    (checkpoint / "env_runner/state.pkl").unlink()
    with pytest.raises(driver.ZeroUpdatePortError, match="incomplete"):
        driver._assert_full_checkpoint(checkpoint)  # noqa: SLF001
