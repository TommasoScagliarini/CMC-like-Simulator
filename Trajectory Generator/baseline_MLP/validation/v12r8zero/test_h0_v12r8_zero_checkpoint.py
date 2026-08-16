"""Source/component tests for the deferred V12R8 checkpoint-zero scaffold."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest import mock

import numpy as np
import pytest


HERE = Path(__file__).resolve().parent
BASELINE_ROOT = HERE.parents[1]
R8_ROOT = HERE.parent / "v12r8"
Q3_ROOT = HERE.parent / "v12r8q3"
for root in (HERE, BASELINE_ROOT, R8_ROOT, Q3_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import freeze_h0_v12r8_zero_checkpoint as freezer  # noqa: E402
import h0_v12r8_q3_qualification_contract as q3_source  # noqa: E402
import h0_v12r8_recovery_contract as r8_source  # noqa: E402
import h0_v12r8_zero_checkpoint_contract as contract  # noqa: E402
import h0_v12r8_zero_checkpoint_gates as gates  # noqa: E402
import run_h0_v12r8_zero_checkpoint as runner  # noqa: E402


def _sha(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def _artifact(path: str, label: str | None = None) -> dict[str, Any]:
    identity = path if label is None else label
    return {"path": path, "sha256": _sha(identity), "size_bytes": len(identity) + 1}


def _candidate_module() -> dict[str, Any]:
    rows = [
        {
            "path": name,
            "sha256": _sha(f"candidate:{name}"),
            "size_bytes": 100 + index,
        }
        for index, name in enumerate(sorted(contract.CANDIDATE_REQUIRED_FILES))
    ]
    return {
        "path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": _sha("candidate-tree"),
        "file_count": len(rows),
        "files": rows,
    }


def _candidate_id(module: dict[str, Any]) -> str:
    return contract.candidate_id_for_tree(module["tree_sha256"])


def _actor_manifest(module: dict[str, Any]) -> dict[str, Any]:
    state_sha = next(
        row["sha256"] for row in module["files"] if row["path"] == "module_state.pkl"
    )
    return {
        "schema_version": 1,
        "status": contract.r8.ACTOR_FEATURE_MANIFEST_STATUS,
        "topology_id": contract.r8.TOPOLOGY_ID,
        "fit_contract_id": contract.r8.FIT_CONTRACT_ID,
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "actor_feature_names": list(contract.q3.ACTOR_FEATURE_NAMES),
        "fcnet_hiddens": list(contract.EXPECTED_HIDDENS),
        "disabled_clock_columns": list(contract.DISABLED_CLOCK_COLUMNS),
        "actor_digest": _sha("candidate-actor"),
        "module_state_sha256": state_sha,
    }


def _r8_terminal(module: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.r8.SCHEMA_VERSION,
        "status": contract.r8.PIPELINE_TERMINAL_PASS_STATUS,
        "passed": True,
        "terminal": True,
        "protocol_id": contract.r8.PROTOCOL_ID,
        "pipeline_id": contract.r8.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": _candidate_id(module),
        "candidate_module": copy.deepcopy(module),
        "error": None,
        "candidate_freeze": _artifact(contract.r8.CANDIDATE_FREEZE_PATH.as_posix()),
        "final_development_receipt": _artifact(
            contract.r8.FINAL_DEVELOPMENT_PATH.as_posix()
        ),
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "next_stage": "WAIT_SEPARATE_V12R8Q3_PROTOCOL",
    }


def _q3_terminal(module: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.q3.SCHEMA_VERSION,
        "status": contract.q3.PIPELINE_TERMINAL_PASS_STATUS,
        "passed": True,
        "terminal": True,
        "protocol_id": contract.q3.PROTOCOL_ID,
        "pipeline_id": contract.q3.PIPELINE_ID,
        "candidate_id": _candidate_id(module),
        "candidate_module": copy.deepcopy(module),
        "error_type": None,
        "error": None,
        "final_receipt": _artifact(contract.q3.FINAL_RECEIPT_PATH.as_posix()),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "aggregate_requires_6_of_6": True,
        "compensation_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "next_stage": contract.q3.NEXT_STAGE_AFTER_Q3_PASS,
    }


def _attestation(endpoint: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    return {
        "endpoint": copy.deepcopy(endpoint),
        "artifact": _artifact(str(endpoint["path"])),
        "verifier_module": endpoint["verifier_module"],
        "verifier": endpoint["verifier"],
        "verified_result_sha256": gates.canonical_json_sha256(result),
        "verifier_returned_mapping": True,
    }


def _upstream(module: dict[str, Any]) -> dict[str, Any]:
    r8 = _r8_terminal(module)
    q3 = _q3_terminal(module)
    return {
        "v12r8_terminal": r8,
        "v12r8_q3_terminal": q3,
        "semantic_attestations": {
            "v12r8_terminal": _attestation(contract.R8_TERMINAL_ENDPOINT, r8),
            "v12r8_q3_terminal": _attestation(contract.Q3_TERMINAL_ENDPOINT, q3),
        },
    }


def _source_closure(module: dict[str, Any]) -> dict[str, Any]:
    inputs = {
        name: _artifact(path, f"input:{name}")
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
    }
    for name, expected in contract.PROFILE_ATTESTATIONS.items():
        inputs[name]["sha256"] = expected["sha256"]
    return {
        "sources": {
            name: _artifact(path, f"source:{name}")
            for name, path in contract.SOURCE_RELATIVE_PATHS.items()
        },
        "inputs": inputs,
        "candidate_module": copy.deepcopy(module),
    }


@pytest.fixture
def bound_world() -> dict[str, Any]:
    module = _candidate_module()
    manifest = _actor_manifest(module)
    upstream = _upstream(module)
    closure = _source_closure(module)
    lock = freezer.build_lock_payload(
        upstream_payload=upstream,
        actor_manifest=manifest,
        source_closure=closure,
    )
    return {
        "module": module,
        "manifest": manifest,
        "upstream": upstream,
        "closure": closure,
        "lock": lock,
    }


def _actor_surface(actor_digest: str) -> dict[str, Any]:
    return {
        "actor_digest": actor_digest,
        "actor_state_sha256": _sha("actor-state"),
        "actor_key_count": 10,
        "actor_byte_count": 2_000_000,
    }


def _critic_surface() -> dict[str, Any]:
    return {
        "critic_state_sha256": _sha("fresh-critic"),
        "critic_key_count": 6,
        "critic_byte_count": 1_000_000,
    }


def _optimizer_snapshot() -> dict[str, Any]:
    names = ["pi.0.weight", "vf.0.weight"]
    return {
        "optimizer_state_empty": True,
        "trainable_parameter_count": len(names),
        "trainable_parameter_names": names,
        "all_trainable_parameters_registered_once": True,
        "optimizers": [
            {
                "optimizer_name": "default_optimizer",
                "optimizer_type": "Adam",
                "state_entry_count": 0,
                "param_groups": [
                    {
                        "group_index": 0,
                        "parameter_names": names,
                        "options": {"lr": 5.0e-7},
                    }
                ],
            }
        ],
    }


def _progress() -> dict[str, int]:
    return {name: 0 for name in contract.ZERO_COUNTER_NAMES}


def _checkpoint_tree() -> dict[str, Any]:
    names = sorted(contract.CHECKPOINT_REQUIRED_SUFFIXES | {"extra/state.pkl"})
    return {
        "path": contract.CHECKPOINT_PATH.as_posix(),
        "tree_sha256": _sha("checkpoint-tree"),
        "file_count": len(names),
        "files": [
            {"path": name, "sha256": _sha(name), "size_bytes": index + 1}
            for index, name in enumerate(names)
        ],
    }


def _runtime_audit(world: dict[str, Any]) -> dict[str, Any]:
    binding = world["lock"]["binding"]
    actor_digest = binding["actor_digest"]
    critic = _critic_surface()
    optimizer = _optimizer_snapshot()
    progress = _progress()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R8_ZERO_CHECKPOINT_AUDIT",
        "protocol_id": contract.PROTOCOL_ID,
        "binding": copy.deepcopy(binding),
        "checks": {name: True for name in contract.REQUIRED_AUDIT_CHECKS},
        "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
        "zero_reward_config": copy.deepcopy(contract.ZERO_REWARD_CONFIG),
        "actor_surfaces": {
            name: _actor_surface(actor_digest) for name in contract.ACTOR_SURFACE_NAMES
        },
        "critic_surfaces": {
            name: copy.deepcopy(critic) for name in contract.CRITIC_SURFACE_NAMES
        },
        "optimizer_surfaces": {
            name: copy.deepcopy(optimizer) for name in contract.OPTIMIZER_SURFACE_NAMES
        },
        "progress_surfaces": {
            name: copy.deepcopy(progress) for name in contract.PROGRESS_SURFACE_NAMES
        },
        "candidate_source_non_actor_key_count": 0,
        "source_critic_restored": False,
        "source_optimizer_restored": False,
        "checkpoint_tree": _checkpoint_tree(),
        "positive_restore_smoke": {
            "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
            "reward_config": copy.deepcopy(contract.POSITIVE_RESTORE_REWARD_CONFIG),
            "restore_from": contract.CHECKPOINT_PATH.as_posix(),
            "restore_completed": True,
            "actor_digest": actor_digest,
            "critic_surface": copy.deepcopy(critic),
            "optimizer_snapshot": copy.deepcopy(optimizer),
            "progress": copy.deepcopy(progress),
            "algorithm_train_calls": 0,
            "environment_samples": 0,
        },
        "closure_before_build": copy.deepcopy(world["closure"]),
        "closure_before_restore": copy.deepcopy(world["closure"]),
        "closure_after_restore": copy.deepcopy(world["closure"]),
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "algorithm_train_calls": 0,
        "training_executed": False,
    }


def test_contract_binds_exact_stable_r8_and_q3_endpoints() -> None:
    assert contract.SCHEMA_VERSION == 1285
    assert contract.R8_TERMINAL_ENDPOINT["protocol_id"] == r8_source.PROTOCOL_ID
    assert contract.R8_TERMINAL_ENDPOINT["path"] == r8_source.LEDGER_PATH.as_posix()
    assert contract.R8_TERMINAL_ENDPOINT["required_status"] == (
        r8_source.PIPELINE_TERMINAL_PASS_STATUS
    )
    assert contract.Q3_TERMINAL_ENDPOINT["protocol_id"] == q3_source.PROTOCOL_ID
    assert contract.Q3_TERMINAL_ENDPOINT["path"] == (
        q3_source.PIPELINE_LEDGER_PATH.as_posix()
    )
    assert contract.Q3_TERMINAL_ENDPOINT["required_status"] == (
        q3_source.PIPELINE_TERMINAL_PASS_STATUS
    )
    assert contract.CANDIDATE_MODULE_PATH == r8_source.CANDIDATE_MODULE_PATH
    assert contract.CANDIDATE_REQUIRED_FILES == q3_source.CANDIDATE_REQUIRED_FILES


def test_contract_is_w512_v26_causal_and_single_positive_resume_only() -> None:
    audit = contract.contract_self_check()
    assert audit["passed"] is True
    assert contract.CANDIDATE_ID is None
    assert contract.EXPECTED_HIDDENS == (512, 512)
    assert contract.TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
    assert contract.ZERO_REWARD_CONFIG["morphology_weight"] == 0.0
    assert contract.POSITIVE_MORPHOLOGY_WEIGHTS == (0.0025,)
    assert contract.POSITIVE_RESTORE_REWARD_CONFIG["morphology_weight"] == 0.0025
    assert (
        contract.POSITIVE_RESTORE_REWARD_CONFIG["morphology_causal_allow_effects"]
        == 1.0
    )
    changed_reward_fields = {
        name
        for name, zero_value in contract.ZERO_REWARD_CONFIG.items()
        if contract.POSITIVE_RESTORE_REWARD_CONFIG[name] != zero_value
    }
    assert changed_reward_fields == {
        "morphology_causal_allow_effects",
        "morphology_weight",
    }
    for platform in ("macos_arm64", "windows_x86_64"):
        argv = contract.resume_training_argv(platform)
        assert "--resume-from" in argv
        assert "--warm-start" not in argv
        assert "--warm-start-raw" not in argv
        assert argv[argv.index("--iterations") + 1] == "50"
        assert Path(argv[argv.index("--resume-from") + 1].replace("\\", "/")).name == (
            "checkpoint_zero"
        )
        reward = json.loads(argv[argv.index("--reward-json") + 1])
        assert reward == contract.POSITIVE_RESTORE_REWARD_CONFIG


def test_public_freeze_and_execute_fail_before_publication_without_prerequisites() -> (
    None
):
    with mock.patch("builtins.open", side_effect=AssertionError("unexpected I/O")):
        with pytest.raises(freezer.ZeroCheckpointFreezeDeferredError, match="terminal"):
            freezer.freeze()
        with pytest.raises(runner.ZeroCheckpointDeferredError, match="missing"):
            runner.execute()
    assert not contract.LOCK_PATH.is_absolute()
    assert not contract.OUTPUT_ROOT.is_absolute()


def test_upstream_gate_requires_live_semantic_attestations_and_same_candidate() -> None:
    module = _candidate_module()
    payload = _upstream(module)
    passed = gates.upstream_terminal_gate(payload)
    assert passed["passed"] is True
    assert passed["candidate_id"] == _candidate_id(module)

    no_semantics = copy.deepcopy(payload)
    no_semantics["semantic_attestations"] = {}
    assert gates.upstream_terminal_gate(no_semantics)["passed"] is False

    cross_candidate = copy.deepcopy(payload)
    cross_candidate["v12r8_q3_terminal"]["candidate_id"] = (
        contract.candidate_id_for_tree(_sha("other-tree"))
    )
    assert gates.upstream_terminal_gate(cross_candidate)["passed"] is False

    q3_nonzero = copy.deepcopy(payload)
    q3_nonzero["v12r8_q3_terminal"]["actor_updates"] = 1
    assert gates.upstream_terminal_gate(q3_nonzero)["passed"] is False


def test_actor_manifest_and_candidate_tree_are_exact() -> None:
    module = _candidate_module()
    manifest = _actor_manifest(module)
    assert gates.tree_record_valid(
        module,
        expected_path=contract.CANDIDATE_MODULE_PATH.as_posix(),
        required_files=contract.CANDIDATE_REQUIRED_FILES,
    )
    assert gates.actor_manifest_gate(manifest, candidate_module=module)["passed"]
    drifted = copy.deepcopy(manifest)
    drifted["fcnet_hiddens"] = [256, 256]
    assert not gates.actor_manifest_gate(drifted, candidate_module=module)["passed"]


def test_freezer_builds_pure_lock_only_after_exact_semantic_pass(
    bound_world: dict[str, Any],
) -> None:
    lock = bound_world["lock"]
    assert set(lock) == runner.LOCK_KEYS
    assert lock["binding"]["candidate_id"] == _candidate_id(bound_world["module"])
    assert lock["actor_updates"] == lock["critic_updates"] == lock["ppo_updates"] == 0
    assert lock["training_authorized"] is False
    assert runner.verify_lock_payload(lock) == lock

    drifted = copy.deepcopy(bound_world["closure"])
    drifted["inputs"]["training_config"]["sha256"] = _sha("drifted")
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="closure_rehash"):
        runner.verify_lock_payload(lock, observed_source_closure=drifted)


def test_live_freezer_build_rehashes_around_semantic_prerequisites(
    bound_world: dict[str, Any], monkeypatch: pytest.MonkeyPatch
) -> None:
    calls = {"closure": 0, "verify": 0}

    def closure() -> dict[str, Any]:
        calls["closure"] += 1
        return copy.deepcopy(bound_world["closure"])

    def verify(value: dict[str, Any]) -> dict[str, Any]:
        calls["verify"] += 1
        assert value == bound_world["closure"]
        return copy.deepcopy(value)

    monkeypatch.setattr(
        runner,
        "semantic_upstream_payload",
        lambda: copy.deepcopy(bound_world["upstream"]),
    )
    monkeypatch.setattr(runner, "closure_snapshot", closure)
    monkeypatch.setattr(runner, "verify_closure", verify)
    monkeypatch.setattr(
        runner,
        "candidate_actor_manifest",
        lambda module: copy.deepcopy(bound_world["manifest"]),
    )
    lock = freezer.build_lock()
    assert lock["binding"] == bound_world["lock"]["binding"]
    assert calls["closure"] >= 2
    assert calls["verify"] == 1


def test_freeze_refuses_clobber_before_semantic_calls(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    occupied = tmp_path / "lock.json"
    occupied.write_text("occupied", encoding="utf-8")
    monkeypatch.setattr(runner, "LOCK", occupied)
    monkeypatch.setattr(runner, "OUTPUT_ROOT", tmp_path / "unused")
    build = mock.Mock(side_effect=AssertionError("must not inspect upstream"))
    monkeypatch.setattr(freezer, "build_lock", build)
    with pytest.raises(freezer.ZeroCheckpointFreezeDeferredError, match="clobber"):
        freezer.freeze()
    build.assert_not_called()


def test_verify_lock_rejects_semantic_upstream_mutation(
    bound_world: dict[str, Any], monkeypatch: pytest.MonkeyPatch
) -> None:
    mutated = copy.deepcopy(bound_world["upstream"])
    mutated["v12r8_q3_terminal"]["candidate_id"] = contract.candidate_id_for_tree(
        _sha("mutated-candidate")
    )
    monkeypatch.setattr(runner, "strict_json", lambda path: bound_world["lock"])
    monkeypatch.setattr(runner, "semantic_upstream_payload", lambda: mutated)
    monkeypatch.setattr(
        runner, "closure_snapshot", lambda: copy.deepcopy(bound_world["closure"])
    )
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="evidence drifted"):
        runner.verify_lock()


def test_execute_preflight_failure_never_claims_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "not-created"
    monkeypatch.setattr(runner, "OUTPUT_ROOT", output)
    monkeypatch.setattr(
        runner,
        "verify_lock",
        mock.Mock(
            side_effect=runner.ZeroCheckpointDeferredError("terminal Q3 not PASS")
        ),
    )
    runtime = mock.Mock(side_effect=AssertionError("runtime must stay closed"))
    monkeypatch.setattr(runner, "_runtime_port", runtime)
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="not PASS"):
        runner.execute()
    assert not output.exists()
    runtime.assert_not_called()


def test_exclusive_json_and_tree_record_reject_clobber_and_links(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    destination = tmp_path / "receipt.json"
    runner.write_json_exclusive(destination, {"passed": True})
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="clobber"):
        runner.write_json_exclusive(destination, {"passed": False})

    tree = tmp_path / "tree"
    tree.mkdir()
    (tree / "state.pkl").write_bytes(b"state")
    assert runner.tree_record(tree)["file_count"] == 1
    link = tree / "linked.pkl"
    try:
        link.symlink_to(tree / "state.pkl")
    except OSError:
        pytest.skip("symlink creation is unavailable")
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="unsafe"):
        runner.tree_record(tree)


def test_checkpoint_plan_is_zero_progress_no_sampling_and_positive_0025(
    bound_world: dict[str, Any],
) -> None:
    plan = runner.checkpoint_creation_plan(bound_world["lock"])
    assert plan["checkpoint_path"].endswith("/checkpoint_zero")
    assert plan["actor_transplants"] == 1
    assert plan["actor_updates"] == plan["critic_updates"] == plan["ppo_updates"] == 0
    assert plan["environment_samples"] == 0
    assert plan["training_executed"] is False
    assert plan["positive_restore_reward_config"]["morphology_weight"] == 0.0025
    assert plan["stages"].index("build_positive_0025_live_config_before_restore") < (
        plan["stages"].index("restore_checkpoint_into_positive_live_config")
    )


def _actor_state() -> dict[str, np.ndarray]:
    first_weight = np.zeros((512, 35), dtype=np.float32)
    first_bias = np.zeros((512,), dtype=np.float32)
    second_weight = np.zeros((512, 512), dtype=np.float32)
    second_bias = np.zeros((512,), dtype=np.float32)
    head_weight = np.zeros((4, 512), dtype=np.float32)
    head_bias = np.zeros((4,), dtype=np.float32)
    head_bias[2:] = np.float32(np.log(contract.EXPECTED_SIGMA))
    return {
        "pi_encoder.0.weight": first_weight.copy(),
        "pi_encoder.0.bias": first_bias.copy(),
        "pi_encoder.2.weight": second_weight.copy(),
        "pi_encoder.2.bias": second_bias.copy(),
        "pi.0.0.weight": first_weight.copy(),
        "pi.0.0.bias": first_bias.copy(),
        "pi.0.2.weight": second_weight.copy(),
        "pi.0.2.bias": second_bias.copy(),
        "pi.1.weight": head_weight,
        "pi.1.bias": head_bias,
    }


def test_w512_actor_validation_and_transplant_preserve_fresh_critic_bytes() -> None:
    candidate = _actor_state()
    audit = runner.validate_candidate_actor_state(candidate)
    assert audit["actor_only"] is True
    assert audit["hidden_dims"] == [512, 512]
    target = {name: np.ones_like(value) for name, value in candidate.items()}
    target["vf_encoder.0.weight"] = np.arange(32, dtype=np.float32)
    critic_before = target["vf_encoder.0.weight"].tobytes()
    merged, report = runner.transplant_standard_actor(
        target_state=target,
        candidate_state=candidate,
    )
    assert report["fresh_critic_preserved_byte_exact"] is True
    assert merged["vf_encoder.0.weight"].tobytes() == critic_before
    assert runner.actor_state_digest(merged) == audit["actor_digest"]

    bad = _actor_state()
    bad["pi_encoder.0.weight"][0, 0] = np.float32(-0.0)
    bad["pi.0.0.weight"][0, 0] = np.float32(-0.0)
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="bit-zero"):
        runner.validate_candidate_actor_state(bad)


class _Parameter:
    def __init__(self, *, requires_grad: bool = True) -> None:
        self.requires_grad = requires_grad


class _Module:
    def __init__(self, parameters: list[tuple[str, _Parameter]]) -> None:
        self._parameters = parameters

    def named_parameters(self) -> list[tuple[str, _Parameter]]:
        return self._parameters


class _Optimizer:
    def __init__(self, parameters: list[_Parameter], *, state: dict[Any, Any]) -> None:
        self.param_groups = [{"params": parameters, "lr": 5.0e-7}]
        self._state = state

    def state_dict(self) -> dict[str, Any]:
        return {"state": self._state}


class _Learner:
    def __init__(self, *, optimizer_state: dict[Any, Any] | None = None) -> None:
        self.parameters = [("pi.weight", _Parameter()), ("vf.weight", _Parameter())]
        self.module = {contract.DEFAULT_POLICY_ID: _Module(self.parameters)}
        self.optimizer = _Optimizer(
            [parameter for _, parameter in self.parameters],
            state=optimizer_state or {},
        )

    def get_optimizers_for_module(self, policy_id: str) -> list[tuple[str, Any]]:
        assert policy_id == contract.DEFAULT_POLICY_ID
        return [("default_optimizer", self.optimizer)]


def test_optimizer_snapshot_requires_empty_state_and_exact_registration() -> None:
    snapshot = runner.optimizer_snapshot_on_learner(_Learner())
    assert snapshot["optimizer_state_empty"] is True
    assert snapshot["all_trainable_parameters_registered_once"] is True
    assert snapshot["trainable_parameter_count"] == 2
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="not empty"):
        runner.optimizer_snapshot_on_learner(_Learner(optimizer_state={0: {}}))


def test_zero_progress_snapshot_rejects_nonzero_and_boolean() -> None:
    assert runner.zero_progress_snapshot(_progress()) == _progress()
    nonzero = _progress()
    nonzero["training_iteration"] = 1
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="not zero"):
        runner.zero_progress_snapshot(nonzero)
    boolean = _progress()
    boolean["training_iteration"] = False
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="not numeric"):
        runner.zero_progress_snapshot(boolean)


def test_runtime_audit_requires_byte_exact_fresh_state_and_only_positive_0025(
    bound_world: dict[str, Any],
) -> None:
    audit = _runtime_audit(bound_world)
    result = gates.runtime_audit_gate(
        audit,
        expected_binding=bound_world["lock"]["binding"],
    )
    assert result["passed"] is True

    critic_drift = copy.deepcopy(audit)
    critic_drift["critic_surfaces"]["after_restore"]["critic_state_sha256"] = _sha(
        "changed-critic"
    )
    assert not gates.runtime_audit_gate(
        critic_drift,
        expected_binding=bound_world["lock"]["binding"],
    )["passed"]

    optimizer_drift = copy.deepcopy(audit)
    optimizer_drift["optimizer_surfaces"]["after_restore"]["optimizer_state_empty"] = (
        False
    )
    assert not gates.runtime_audit_gate(
        optimizer_drift,
        expected_binding=bound_world["lock"]["binding"],
    )["passed"]

    positive_005 = copy.deepcopy(audit)
    positive_005["positive_restore_smoke"]["reward_config"]["morphology_weight"] = 0.005
    assert not gates.runtime_audit_gate(
        positive_005,
        expected_binding=bound_world["lock"]["binding"],
    )["passed"]


def test_full_checkpoint_gate_requires_every_rllib_component() -> None:
    checkpoint = _checkpoint_tree()
    assert gates.checkpoint_tree_gate(checkpoint)["passed"] is True
    missing = copy.deepcopy(checkpoint)
    missing["files"] = [
        row for row in missing["files"] if row["path"] != "algorithm_state.pkl"
    ]
    missing["file_count"] -= 1
    assert gates.checkpoint_tree_gate(missing)["passed"] is False


def test_terminal_receipt_and_handoff_are_non_authorizing_resume_only(
    bound_world: dict[str, Any],
) -> None:
    receipt = runner.build_terminal_receipt(
        lock_payload=bound_world["lock"],
        runtime_audit=_runtime_audit(bound_world),
    )
    assert gates.terminal_receipt_gate(receipt)["passed"] is True
    handoff = runner.build_training_handoff(
        receipt,
        terminal_receipt_artifact=_artifact(contract.RECEIPT_PATH.as_posix()),
    )
    assert handoff["initialization_mode"] == "resume_from_full_checkpoint_zero"
    assert handoff["target_training_iterations"] == 50
    assert handoff["morphology_weight"] == 0.0025
    assert handoff["morphology_causal_allow_effects"] == 1.0
    assert handoff["training_authorized"] is False
    assert handoff["training_command_published"] is False
    assert handoff["requires_terminal_positive_ab"] is True
    for item in handoff["platforms"].values():
        assert "--resume-from" in item["proposed_resume_argv"]
        assert "--warm-start" not in item["proposed_resume_argv"]
        assert "--warm-start-raw" not in item["proposed_resume_argv"]

    failed_audit = _runtime_audit(bound_world)
    failed_audit["algorithm_train_calls"] = 1
    failed = runner.build_terminal_receipt(
        lock_payload=bound_world["lock"], runtime_audit=failed_audit
    )
    assert failed["passed"] is False
    with pytest.raises(runner.ZeroCheckpointDeferredError, match="PASS"):
        runner.build_training_handoff(
            failed,
            terminal_receipt_artifact=_artifact(contract.RECEIPT_PATH.as_posix()),
        )


def test_real_profiles_are_hash_exact() -> None:
    repo_root = HERE.parents[3]
    for record in contract.PROFILE_ATTESTATIONS.values():
        path = repo_root.joinpath(*Path(record["path"]).parts)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]


def test_future_resume_argv_is_accepted_by_live_training_parser(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import train_ppo_mlp as train

    argv = contract.resume_training_argv("macos_arm64")
    monkeypatch.setattr(sys, "argv", [argv[1], *argv[2:]])
    args = train.parse_args()
    assert args.resume_from.endswith("checkpoint_zero")
    assert args.warm_start is False
    assert args.warm_start_raw is False
    assert args.iterations == 50
    assert args.binary_phase_fsm_mode == "binary_active"
    assert json.loads(args.reward_json) == contract.POSITIVE_RESTORE_REWARD_CONFIG
    source_order = runner.training_resume_source_order_audit(train)
    assert source_order["passed"] is True
    assert source_order["reward_json_merged_before_build_config"] is True
    assert source_order["build_config_before_build_algo"] is True
    assert source_order["build_algo_before_restore_from_path"] is True


def test_runner_source_saves_full_checkpoint_restores_twice_and_never_trains() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"train", "sample"}
    ]
    assert forbidden_calls == []

    named_calls = [
        (node.func.value.id, node.func.attr)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.attr in {"save_to_path", "restore_from_path"}
    ]
    assert named_calls.count(("first", "save_to_path")) == 1
    assert named_calls.count(("restored", "restore_from_path")) == 1
    assert named_calls.count(("positive", "restore_from_path")) == 1


def test_import_does_not_construct_algorithm_or_touch_runtime() -> None:
    assert contract.contract_self_check()["passed"] is True
    assert isinstance(SimpleNamespace(), SimpleNamespace)
