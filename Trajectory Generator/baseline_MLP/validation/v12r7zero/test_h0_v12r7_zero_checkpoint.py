"""Source/component tests for the historical V12R7/Q3 zero scaffold."""

from __future__ import annotations

import ast
import hashlib
import json
import stat
import sys
from pathlib import Path, PurePosixPath
from types import SimpleNamespace
from typing import Any
from unittest import mock

import numpy as np
import pytest


ZERO_ROOT = Path(__file__).resolve().parent
VALIDATION_ROOT = ZERO_ROOT.parent
for _root in (
    ZERO_ROOT,
    VALIDATION_ROOT / "v12r7",
    VALIDATION_ROOT / "v12r7q3",
    VALIDATION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r7_zero_checkpoint as freezer  # noqa: E402
import h0_v12r7_zero_checkpoint_contract as contract  # noqa: E402
import run_h0_v12r7_zero_checkpoint as driver  # noqa: E402


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def test_contract_is_deferred_w512_v26_corridor_zero_and_resume_only() -> None:
    audit = contract.contract_self_check()

    assert audit["passed"] is True
    assert all(audit["checks"].values())
    assert contract.CANDIDATE_ID is None
    assert contract.CANDIDATE_MODULE is None
    assert contract.HISTORICAL_SCAFFOLD_ONLY is True
    assert contract.CANONICAL_ENTRYPOINTS_BLOCKED is True
    assert contract.SUCCESSOR_ZERO_NAMESPACE == "v12r8zero"
    assert contract.EXPECTED_HIDDENS == (512, 512)
    assert contract.EXPECTED_ACTOR_FEATURES == 35
    assert contract.EXPECTED_FULL_FEATURES == 84
    assert contract.TARGET_FIXED_CONFIG["num_hidden_layers"] == 2
    assert contract.TARGET_FIXED_CONFIG["dim_hidden_layers"] == 512
    assert contract.TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
    assert contract.TARGET_REWARD_CONFIG["morphology_weight"] == 0.0
    assert contract.TARGET_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
    assert contract.AUTHORITY["algorithm_build_authorized_now"] is False
    assert contract.AUTHORITY["checkpoint_publication_authorized_now"] is False
    assert contract.AUTHORITY["r7_terminal_fail_acknowledged"] is True


def test_public_r7_entrypoints_are_permanently_fail_closed() -> None:
    with pytest.raises(
        freezer.ZeroCheckpointFreezeError,
        match="V12R7_TERMINAL_FAIL_REQUIRES_NEW_V12R8_LINEAGE",
    ):
        freezer.freeze()
    with pytest.raises(
        driver.ZeroCheckpointError,
        match="V12R7_TERMINAL_FAIL_REQUIRES_NEW_V12R8_LINEAGE",
    ):
        driver.execute()
    with pytest.raises(
        driver.ZeroCheckpointError,
        match="V12R7_TERMINAL_FAIL_REQUIRES_NEW_V12R8_LINEAGE",
    ):
        driver.verify_terminal_pass()


def test_terminal_r7_q3_paths_are_exact_contract_aliases() -> None:
    assert contract.R7_TERMINAL_LEDGER_PATH == contract.r7.LEDGER_PATH
    assert contract.R7_FINAL_DEVELOPMENT_PATH == contract.r7.FINAL_DEVELOPMENT_PATH
    assert contract.Q3_FINAL_RECEIPT_PATH == contract.q3.FINAL_RECEIPT_PATH
    assert contract.Q3_TERMINAL_LEDGER_PATH == contract.q3.PIPELINE_LEDGER_PATH
    assert (
        contract.q3.PIPELINE_TERMINAL_PASS_STATUS
        == "PASS_H0_V12R7_Q3_PIPELINE_TERMINAL"
    )
    assert (
        contract.q3.NEXT_STAGE_AFTER_Q3_PASS == "WAIT_SEPARATE_CHECKPOINT_ZERO_PROTOCOL"
    )
    assert contract.RECEIPT_PATH == contract.OUTPUT_ROOT / "receipt.json"
    assert contract.TERMINAL_LEDGER_PATH == (
        contract.OUTPUT_ROOT / "execution_ledger.json"
    )
    assert contract.CHECKPOINT_PATH == contract.OUTPUT_ROOT / "checkpoint_zero"
    assert callable(driver.verify_terminal_pass)


@pytest.mark.parametrize("platform_id", ["macos_arm64", "windows_x86_64"])
def test_cross_platform_handoff_is_full_resume_not_warm_start(
    platform_id: str,
) -> None:
    argv = contract.resume_training_argv(
        platform_id=platform_id,
        output_dir="Trajectory Generator/runs/training/v12r7_positive_test",
        iterations=50,
        morphology_weight=0.0025,
        positive_morphology_authorized=True,
    )
    reward = json.loads(argv[argv.index("--reward-json") + 1])

    assert "--resume-from" in argv
    assert "--warm-start" not in argv
    assert "--warm-start-raw" not in argv
    assert argv[argv.index("--iterations") + 1] == "50"
    assert argv[argv.index("--dim-hidden-layers") + 1] == "512"
    assert argv[argv.index("--binary-phase-fsm-mode") + 1] == "binary_active"
    assert reward["morphology_weight"] == 0.0025
    assert reward["morphology_causal_allow_effects"] == 1.0
    rendered = contract.render_command(argv, platform_id)
    assert "resume-from" in rendered
    if platform_id == "windows_x86_64":
        assert "\\" in argv[1]
    else:
        assert "/" in argv[1]


def test_positive_handoff_requires_separate_authority() -> None:
    with pytest.raises(ValueError, match="separate frozen authority"):
        contract.resume_training_argv(
            platform_id="macos_arm64",
            output_dir="Trajectory Generator/runs/training/blocked",
            iterations=50,
            morphology_weight=0.0025,
        )
    zero = contract.resume_training_argv(
        platform_id="macos_arm64",
        output_dir="Trajectory Generator/runs/training/zero",
        iterations=50,
    )
    reward = json.loads(zero[zero.index("--reward-json") + 1])
    assert reward["morphology_weight"] == 0.0
    assert reward["morphology_causal_allow_effects"] == 0.0


def test_real_v26_target_config_and_profiles_are_hash_exact() -> None:
    target = driver.validate_runtime_target_config()

    assert target["grf"]["binary_phase_fsm_mode"] == "binary_active"
    assert (
        target["grf"]["binary_phase_event_contract_id"]
        == contract.BINARY_EVENT_CONTRACT_ID
    )
    assert target["reward"]["morphology_weight"] == 0.0
    assert target["reward"]["morphology_causal_allow_effects"] == 0.0
    assert target["positive_structure"]["future_weights"] == [0.0025, 0.005]


def test_real_parser_resolves_explicit_zero_target_as_w512() -> None:
    import train_ppo_mlp as train

    args, reward = driver._target_training_args(  # noqa: SLF001
        train,
        Path("Trajectory Generator/runs/training/parser_only"),
        morphology_weight=0.0,
        positive_authorized=False,
    )
    assert args.iterations == 0
    assert args.num_env_runners == 0
    assert args.num_hidden_layers == 2
    assert args.dim_hidden_layers == 512
    assert args.asymmetric_actor_critic is True
    assert args.rl_module_kind == "standard"
    assert args.binary_phase_fsm_mode == "binary_active"
    assert reward["morphology_phase_mode"] == contract.q3.MORPHOLOGY_PHASE_MODE


def test_proposed_positive_resume_argv_is_accepted_by_live_training_parser() -> None:
    import train_ppo_mlp as train

    argv = contract.resume_training_argv(
        platform_id="macos_arm64",
        output_dir="Trajectory Generator/runs/training/v12r7_positive_parser",
        iterations=50,
        morphology_weight=0.0025,
        positive_morphology_authorized=True,
    )
    previous = list(sys.argv)
    try:
        sys.argv = [str(train.__file__), *argv[2:]]
        args = train.parse_args()
    finally:
        sys.argv = previous
    assert args.resume_from == contract.CHECKPOINT_PATH.as_posix()
    assert args.warm_start is False
    assert args.warm_start_raw is False
    assert args.iterations == 50
    assert args.dim_hidden_layers == 512
    assert args.binary_phase_fsm_mode == "binary_active"
    json_reward = json.loads(args.reward_json)
    assert json_reward["morphology_weight"] == 0.0025
    assert json_reward["morphology_causal_allow_effects"] == 1.0


def test_runner_source_has_no_train_call_and_audits_positive_restore_order() -> None:
    source = Path(driver.__file__).read_text(encoding="utf-8")
    syntax = ast.parse(source)
    called_attributes = [
        node.func.attr
        for node in ast.walk(syntax)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]

    assert "train" not in called_attributes
    assert "save_to_path" in called_attributes
    assert "restore_from_path" in called_attributes
    assert "positive.restore_from_path(checkpoint)" in source
    assert "positive_config_before" in source
    assert "positive_config_after" in source
    assert "checkpoint restore replaced the live positive reward/env config" in source
    assert set(contract.CHECKPOINT_REQUIRED_SUFFIXES) == {
        "algorithm_state.pkl",
        "class_and_ctor_args.pkl",
        "rllib_checkpoint.json",
        "learner_group/learner/state.pkl",
        "learner_group/learner/rl_module/module_state.pkl",
        "env_runner/state.pkl",
    }


def test_training_entrypoint_builds_live_cli_config_before_restore() -> None:
    import train_ppo_mlp as train

    audit = driver.training_resume_source_order_audit(train)
    assert audit["passed"] is True
    assert audit["reward_json_merged_before_build_config"] is True
    assert audit["build_config_before_build_algo"] is True
    assert audit["build_algo_before_restore_from_path"] is True


def test_import_surface_does_not_execute_freeze_runner_or_algorithm() -> None:
    for module_path, forbidden_call in (
        (Path(freezer.__file__), "freeze"),
        (Path(driver.__file__), "execute"),
    ):
        tree = ast.parse(module_path.read_text(encoding="utf-8"))
        top_level_calls = [
            node
            for node in tree.body
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
        ]
        assert top_level_calls == []
        assert 'if __name__ == "__main__"' in module_path.read_text(encoding="utf-8")
        assert forbidden_call in module_path.read_text(encoding="utf-8")


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


def test_optimizer_must_be_empty_and_register_every_parameter_once() -> None:
    actor = _Parameter()
    critic = _Parameter()
    module = _Module([("pi.weight", actor), ("vf.weight", critic)])
    optimizer = _Optimizer(
        [
            {"params": [actor], "lr": 1.0e-4},
            {"params": [critic], "lr": 2.0e-4},
        ]
    )
    snapshot = driver.optimizer_snapshot_on_learner(_Learner(module, optimizer))
    assert snapshot["optimizer_state_empty"] is True
    assert snapshot["all_trainable_parameters_registered_once"] is True
    assert snapshot["trainable_parameter_names"] == ["pi.weight", "vf.weight"]

    dirty = _Optimizer(
        [{"params": [actor, critic], "lr": 1.0e-4}],
        state_entries={1: {"step": 1}},
    )
    with pytest.raises(driver.ZeroCheckpointError, match="not empty"):
        driver.optimizer_snapshot_on_learner(_Learner(module, dirty))
    duplicate = _Optimizer(
        [
            {"params": [actor], "lr": 1.0e-4},
            {"params": [actor, critic], "lr": 1.0e-4},
        ]
    )
    with pytest.raises(driver.ZeroCheckpointError, match="duplicated"):
        driver.optimizer_snapshot_on_learner(_Learner(module, duplicate))


def _actor_state(fill: float = 0.0) -> dict[str, np.ndarray]:
    state = {
        name: np.full(shape, np.float32(fill), dtype=np.float32)
        for name, shape in driver.ACTOR_STATE_SHAPES.items()
    }
    state["pi.1.bias"][2:] = np.float32(np.log(contract.EXPECTED_SIGMA))
    return state


def test_w512_actor_transplant_is_exact_and_critic_is_byte_preserved() -> None:
    import warm_start

    candidate = _actor_state()
    target = {
        **_actor_state(1.0),
        "vf_encoder.0.weight": np.arange(8, dtype=np.float32).reshape(2, 4),
    }
    merged, audit = driver.transplant_standard_actor(
        target_state=target,
        candidate_state=candidate,
        warm_start=warm_start,
    )
    assert warm_start.compare_actor_states(candidate, merged)["exact"] is True
    assert warm_start.compare_non_actor_states(target, merged)["exact"] is True
    assert audit["actor_transplants"] == 1
    assert audit["actor_updates"] == 0
    assert np.array_equal(merged["vf_encoder.0.weight"], target["vf_encoder.0.weight"])


def test_tree_and_publication_reject_links_junctions_and_clobber(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path.resolve()
    monkeypatch.setattr(driver, "REPO_ROOT", root)
    regular = root / "tree" / "module_state.pkl"
    regular.parent.mkdir()
    regular.write_bytes(b"actor")
    linked = regular.parent / "alias.pkl"
    linked.symlink_to(regular)
    with pytest.raises(driver.ZeroCheckpointError, match="unsafe tree file"):
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
    with pytest.raises(driver.ZeroCheckpointError, match="clobber"):
        driver.write_json_exclusive(destination, {"passed": False})


def _sandbox_world(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    root = tmp_path.resolve()
    lock = root.joinpath(*contract.LOCK_PATH.parts)
    output = root.joinpath(*contract.OUTPUT_ROOT.parts)
    candidate_dir = root.joinpath(*contract.CANDIDATE_MODULE_PATH.parts)
    candidate_dir.mkdir(parents=True)
    (candidate_dir / "module_state.pkl").write_bytes(b"r7-w512-actor")
    (candidate_dir / "class_and_ctor_args.pkl").write_bytes(b"standard-w512")
    _write_json(candidate_dir / "metadata.json", {"module": "standard"})
    _write_json(
        candidate_dir / "candidate_build_manifest.json",
        {"status": "PASS", "actor_updates": 1},
    )
    module_state_sha = hashlib.sha256(
        (candidate_dir / "module_state.pkl").read_bytes()
    ).hexdigest()
    _write_json(
        candidate_dir / "actor_feature_manifest.json",
        {
            "schema_version": 1,
            "status": contract.r7.ACTOR_FEATURE_MANIFEST_STATUS,
            "topology_id": contract.r7.TOPOLOGY_ID,
            "fit_contract_id": contract.r7.FIT_CONTRACT_ID,
            "actor_feature_count": 35,
            "actor_feature_names": list(contract.q3.ACTOR_FEATURE_NAMES),
            "fcnet_hiddens": [512, 512],
            "disabled_clock_columns": [0, 1],
            "actor_digest": "a" * 64,
            "module_state_sha256": module_state_sha,
        },
    )

    sources = {
        name: root.joinpath(*PurePosixPath(path).parts)
        for name, path in contract.SOURCE_RELATIVE_PATHS.items()
    }
    for name, path in sources.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_text(f"# source {name}\n", encoding="utf-8")
    inputs = {
        name: root.joinpath(*PurePosixPath(path).parts)
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
    }
    for name, path in inputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            _write_json(path, {"placeholder": name})

    monkeypatch.setattr(driver, "REPO_ROOT", root)
    monkeypatch.setattr(driver, "LOCK", lock)
    monkeypatch.setattr(driver, "OUTPUT_ROOT", output)
    monkeypatch.setattr(driver, "CANDIDATE_DIR", candidate_dir)
    monkeypatch.setattr(driver, "SOURCE_PATHS", sources)
    monkeypatch.setattr(driver, "INPUT_PATHS", inputs)
    monkeypatch.setattr(
        driver,
        "validate_runtime_target_config",
        lambda: {
            "grf": {"binary_phase_fsm_mode": "binary_active"},
            "reward": dict(contract.TARGET_REWARD_CONFIG),
            "positive_structure": {"compatible": True},
        },
    )

    candidate_module = driver.tree_record(candidate_dir)
    candidate_id = contract.candidate_id_for_tree(candidate_module["tree_sha256"])
    common = {
        "passed": True,
        "protocol_id": contract.r7.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
    }
    _write_json(
        inputs["r7_candidate_freeze"],
        {
            **common,
            "status": contract.r7.CANDIDATE_FREEZE_PASS_STATUS,
            "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
            "candidate_frozen": True,
            "fit_gate_passed": True,
            "standard_actor": True,
            "warm_start_target_512_compatible": True,
            "actor_fit_count": 1,
            "actor_updates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "q3_paths_opened": [],
            "runtime_promoted": False,
        },
    )
    _write_json(
        inputs["r7_final_development"],
        {
            **common,
            "status": contract.r7.DEVELOPMENT_PASS_STATUS,
            "actor_fit_count": 1,
            "actor_updates": 1,
            "development_rollout_count": 6,
            "critic_updates": 0,
            "ppo_updates": 0,
            "qualification_executed": False,
            "runtime_promoted": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
        },
    )
    _write_json(
        inputs["r7_terminal_ledger"],
        {
            **common,
            "status": contract.r7.PIPELINE_TERMINAL_PASS_STATUS,
            "terminal": True,
            "error": None,
            "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
            "candidate_freeze": driver.artifact_record(inputs["r7_candidate_freeze"]),
            "final_development_receipt": driver.artifact_record(
                inputs["r7_final_development"]
            ),
            "actor_fit_count": 1,
            "actor_updates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "qualification_executed": False,
            "runtime_promoted": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
            "next_stage": "WAIT_SEPARATE_V12R7Q3_PROTOCOL",
        },
    )
    q3_common = {
        "passed": True,
        "protocol_id": contract.q3.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "next_stage": contract.q3.NEXT_STAGE_AFTER_Q3_PASS,
    }
    _write_json(
        inputs["q3_final_receipt"],
        {**q3_common, "status": contract.q3.AGGREGATE_PASS_STATUS},
    )
    _write_json(
        inputs["q3_terminal_ledger"],
        {
            **q3_common,
            "status": contract.q3.PIPELINE_TERMINAL_PASS_STATUS,
            "terminal": True,
            "error": None,
        },
    )
    return {
        "root": root,
        "lock": lock,
        "output": output,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "sources": sources,
        "inputs": inputs,
    }


def test_freezer_builds_in_memory_only_after_semantic_r7_and_q3_pass(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)

    payload = freezer.build_lock()

    assert payload["candidate_binding_state"] == "BOUND_AFTER_R7_AND_Q3_TERMINAL_PASS"
    assert payload["candidate_id"] == world["candidate_id"]
    assert payload["candidate_module"] == world["candidate_module"]
    assert payload["prerequisite_audit"]["r7"]["passed"] is True
    assert payload["prerequisite_audit"]["q3"]["passed"] is True
    assert payload["target_fixed_config"]["dim_hidden_layers"] == 512
    assert payload["target_reward_config"]["morphology_weight"] == 0.0
    assert payload["actor_transplants"] == 1
    assert payload["actor_updates"] == 0
    assert payload["critic_updates"] == 0
    assert payload["ppo_updates"] == 0
    assert not world["lock"].exists()
    assert not world["output"].exists()


def test_freezer_rejects_q3_nonzero_update_or_cross_candidate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)
    receipt_path = world["inputs"]["q3_final_receipt"]
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["actor_updates"] = 1
    _write_json(receipt_path, receipt)

    with pytest.raises(freezer.ZeroCheckpointFreezeError, match="Q3 final receipt"):
        freezer.build_lock()
    assert not world["lock"].exists()
    assert not world["output"].exists()


def test_freezer_rehash_detects_toctou_source_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = _sandbox_world(tmp_path, monkeypatch)
    original = freezer.validate_prerequisites

    def validate_then_mutate() -> dict[str, Any]:
        result = original()
        world["sources"]["runner"].write_text("# drift after validation\n")
        return result

    monkeypatch.setattr(freezer, "validate_prerequisites", validate_then_mutate)
    with pytest.raises(freezer.ZeroCheckpointFreezeError, match="closure drifted"):
        freezer.build_lock()
    assert not world["lock"].exists()
    assert not world["output"].exists()


def test_full_checkpoint_tree_requires_all_rllib_components(
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
    with pytest.raises(driver.ZeroCheckpointError, match="incomplete"):
        driver._assert_full_checkpoint(checkpoint)  # noqa: SLF001


def test_handoff_payload_freezes_positive_0025_after_restore_smoke() -> None:
    checkpoint = {
        "path": contract.CHECKPOINT_PATH.as_posix(),
        "tree_sha256": "b" * 64,
        "file_count": 6,
        "files": [],
    }
    lock = {"candidate_id": "candidate"}
    handoff = driver.training_handoff_payload(
        checkpoint_tree=checkpoint,
        lock=lock,
    )
    assert handoff["target_training_iterations"] == 50
    assert handoff["morphology_weight"] == 0.0025
    assert handoff["morphology_causal_allow_effects"] == 1.0
    assert handoff["positive_live_config_restore_smoke_passed"] is True
    assert handoff["training_authorized"] is False
    assert handoff["training_command_published"] is False
    assert handoff["requires_terminal_positive_ab"] is True
    assert handoff["next_stage"] == contract.NEXT_STAGE_AFTER_ZERO_PASS
    for row in handoff["platforms"].values():
        assert "--resume-from" in row["proposed_resume_argv"]
        assert "--warm-start" not in row["proposed_resume_argv"]
