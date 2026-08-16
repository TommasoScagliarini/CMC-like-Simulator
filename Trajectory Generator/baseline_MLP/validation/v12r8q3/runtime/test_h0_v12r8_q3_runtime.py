"""Environment-free tests for the canonical deferred V12R8-Q3 runtime."""

from __future__ import annotations

import copy
import hashlib
import inspect
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest


RUNTIME_ROOT = Path(__file__).resolve().parent
Q3_ROOT = RUNTIME_ROOT.parent
REPO_ROOT = Q3_ROOT.parents[3]
for _root in (Q3_ROOT, RUNTIME_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r8_q3_qualification_protocol as freezer  # noqa: E402
import h0_v12r8_q3_artifacts as artifacts  # noqa: E402
import h0_v12r8_q3_physical_rollout as physical  # noqa: E402
import h0_v12r8_q3_prerequisites as prerequisites  # noqa: E402
import h0_v12r8_q3_qualification_contract as contract  # noqa: E402
import h0_v12r8_q3_qualification_gates as gates  # noqa: E402
import prepare_h0_v12r8_q3_noise_tapes as noise  # noqa: E402
import run_h0_v12r8_q3_qualification as runner  # noqa: E402


def _digest(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def _record(path: str, label: str | None = None) -> dict[str, Any]:
    return {
        "path": path,
        "sha256": _digest(path if label is None else label),
        "size_bytes": 1,
    }


def _candidate() -> tuple[str, dict[str, Any]]:
    rows = [
        {"path": name, "sha256": _digest(name), "size_bytes": index + 1}
        for index, name in enumerate(sorted(contract.CANDIDATE_REQUIRED_FILES))
    ]
    module = {
        "path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "tree_sha256": artifacts.tree_digest(rows),
        "file_count": len(rows),
        "files": rows,
    }
    return contract.r8.candidate_id(module["tree_sha256"]), module


def _snapshot() -> dict[str, Any]:
    candidate_id, module = _candidate()
    return {
        "gate": {
            "passed": True,
            "candidate_id": candidate_id,
            "candidate_module": module,
            "official_verifier_count": 5,
        },
        "records": {},
        "actor_feature_manifest_record": _record(
            f"{module['path']}/actor_feature_manifest.json"
        ),
    }


def _closure() -> dict[str, dict[str, Any]]:
    expected = {
        **contract.QUALIFICATION_SOURCE_PATHS,
        **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
        **contract.QUALIFICATION_INPUT_PATHS,
    }
    rows = {name: _record(path, name) for name, path in expected.items()}
    for name, digest in contract.QUALIFICATION_INPUT_SHA256.items():
        rows[name]["sha256"] = digest
    return rows


@pytest.fixture(autouse=True)
def _clear_binding() -> Any:
    prerequisites.clear_candidate_binding_for_tests()
    yield
    prerequisites.clear_candidate_binding_for_tests()


def test_imports_are_inert_and_deferred() -> None:
    paths = (
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.NOISE_ROOT,
        contract.RUN_ROOT,
    )
    before = {
        path.as_posix(): freezer.resolve_relative(path).exists() for path in paths
    }
    assert freezer.deferred_preflight()["publication_performed"] is False
    state = runner.deferred_preflight()
    assert state["passed"] is False
    assert state["lineage_state"] == contract.LINEAGE_STATE
    assert state["lineage_state"] == "DEFERRED_UNTIL_R8_TERMINAL_PASS"
    assert state["publication_performed"] is False
    assert state["qualification_execution_authorized_now"] is False
    after = {path.as_posix(): freezer.resolve_relative(path).exists() for path in paths}
    assert after == before


def test_clean_subprocess_import_does_not_load_heavy_runtime() -> None:
    script = f"""
import json, sys
sys.path.insert(0, {str(Q3_ROOT)!r})
sys.path.insert(0, {str(RUNTIME_ROOT)!r})
import h0_v12r8_q3_physical_rollout
import run_h0_v12r8_q3_qualification
print(json.dumps(sorted(set(sys.modules) & {{'ray', 'torch', 'opensim'}})))
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert json.loads(result.stdout) == []


def test_protocol_and_lock_pure_builders_bind_exact_candidate() -> None:
    snapshot = _snapshot()
    closure = _closure()
    protocol = freezer.build_protocol_payload(
        prerequisite_snapshot=snapshot,
        closure=closure,
    )
    assert freezer.protocol_payload_gate(protocol)["passed"] is True
    protocol_record = _record(contract.PROTOCOL_FREEZE_PATH.as_posix())
    lock = freezer.build_execution_lock_payload(
        protocol_payload=protocol,
        protocol_record=protocol_record,
        prerequisite_snapshot=snapshot,
        closure=closure,
        occupancy={
            "noise_root_absent": True,
            "run_root_absent": True,
            "pipeline_claim_absent": True,
            "pipeline_ledger_absent": True,
        },
    )
    assert lock["candidate_id"] == snapshot["gate"]["candidate_id"]
    assert lock["candidate_module"] == snapshot["gate"]["candidate_module"]
    assert lock["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert lock["baseline_first"] is True
    assert lock["checkpoint_zero_created"] is False

    bool_update = copy.deepcopy(protocol)
    bool_update["actor_updates"] = False
    assert freezer.protocol_payload_gate(bool_update)["passed"] is False


def test_official_hook_adapter_uses_the_five_r8_entrypoints(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, Any]] = []

    def verify_protocol() -> dict[str, Any]:
        calls.append(("protocol", None))
        return {}

    def verify_lock(*, require_pristine: bool) -> dict[str, Any]:
        calls.append(("lock", require_pristine))
        return {}

    r8_freezer = SimpleNamespace(
        verify_protocol_freeze=verify_protocol,
        verify_execution_lock=verify_lock,
    )
    r8_runner = SimpleNamespace(
        verify_candidate_freeze_receipt=lambda: {},
        verify_final_development_receipt=lambda: {},
        verify_terminal_ledger=lambda: {},
    )

    def fake_import(name: str) -> Any:
        return r8_freezer if name == "freeze_h0_v12r8_recovery" else r8_runner

    monkeypatch.setattr(freezer.importlib, "import_module", fake_import)
    hooks = freezer.official_r8_hooks()
    assert hooks.verify_protocol_freeze() == {}
    assert hooks.verify_execution_lock() == {}
    assert hooks.verify_candidate_freeze_receipt() == {}
    assert hooks.verify_final_development_receipt() == {}
    assert hooks.verify_terminal_ledger() == {}
    assert calls == [("protocol", None), ("lock", False)]


def test_actual_r8_endpoint_surface_is_importable_without_invocation() -> None:
    hooks = freezer.official_r8_hooks()
    callbacks = (
        hooks.verify_protocol_freeze,
        hooks.verify_execution_lock,
        hooks.verify_candidate_freeze_receipt,
        hooks.verify_final_development_receipt,
        hooks.verify_terminal_ledger,
    )
    assert all(callable(callback) for callback in callbacks)
    assert all(
        not any(
            parameter.default is inspect.Parameter.empty
            and parameter.kind
            not in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD)
            for parameter in inspect.signature(callback).parameters.values()
        )
        for callback in callbacks
    )


def test_live_freeze_preflight_is_read_only_and_candidate_bound(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    snapshot = _snapshot()
    closure = _closure()
    monkeypatch.setattr(freezer, "_q3_occupied", lambda: [])
    monkeypatch.setattr(
        freezer,
        "live_r8_prerequisite_snapshot",
        lambda **_kwargs: copy.deepcopy(snapshot),
    )
    monkeypatch.setattr(freezer, "source_closure", lambda: copy.deepcopy(closure))
    result = freezer.live_preflight()
    assert result["passed"] is True
    assert result["official_verifier_count"] == 5
    assert result["candidate_id"] == snapshot["gate"]["candidate_id"]
    assert result["candidate_module"] == snapshot["gate"]["candidate_module"]
    assert result["source_closure"] == closure
    assert result["publication_performed"] is False


def test_live_verifier_payload_must_bind_exact_canonical_file_record() -> None:
    payload = {"status": "PASS", "passed": True}
    encoded = freezer.forensic.canonical_json_bytes(payload)
    path = contract.PREREQUISITE_REQUIREMENTS[0]["path"]
    record = {
        "path": path,
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }
    assert freezer._verifier_payload_matches_record(payload, record, expected_path=path)
    tampered_payload = {**payload, "passed": False}
    assert not freezer._verifier_payload_matches_record(
        tampered_payload, record, expected_path=path
    )
    tampered_record = {**record, "size_bytes": record["size_bytes"] + 1}
    assert not freezer._verifier_payload_matches_record(
        payload, tampered_record, expected_path=path
    )


def test_current_source_closure_is_complete_and_hash_bound() -> None:
    readiness = freezer.source_closure_readiness()
    assert readiness["status"] == "PASS_H0_V12R8_Q3_SOURCE_CLOSURE"
    assert readiness["passed"] is True
    assert readiness["missing_sources"] == []
    closure = readiness["records"]
    result = prerequisites.validate_source_closure(closure)
    assert result["passed"] is True
    assert (
        set(closure)
        == {
            **contract.QUALIFICATION_SOURCE_PATHS,
            **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
            **contract.QUALIFICATION_INPUT_PATHS,
        }.keys()
    )
    assert closure["source_h0_config"]["sha256"] == (contract.SOURCE_H0_CONFIG_SHA256)
    assert closure["historical_analog_profile"]["sha256"] == (
        contract.HISTORICAL_ANALOG_PROFILE_SHA256
    )


def test_tape_generation_is_exact_and_repeatable() -> None:
    first = noise.build_tapes()
    second = noise.build_tapes()
    assert set(first) == set(contract.EXPECTED_TAPE_ARRAY_SHA256)
    for filename, expected_hash in contract.EXPECTED_TAPE_ARRAY_SHA256.items():
        left = first[filename]["standard_normal"]
        right = second[filename]["standard_normal"]
        assert left.shape == (500, 2)
        assert left.dtype == np.float32
        assert left.flags.c_contiguous
        assert np.array_equal(left, right)
        assert noise.array_sha256(left) == expected_hash


def test_tape_hash_binds_dtype_shape_and_bytes() -> None:
    array = noise.build_tapes()["deterministic_all_zero.npz"]["standard_normal"]
    assert noise.array_sha256(array) != noise.array_sha256(array.astype(np.float64))
    assert noise.array_sha256(array) != noise.array_sha256(array.reshape(250, 4))
    changed = array.copy()
    changed[0, 0] = np.float32(1.0)
    assert noise.array_sha256(array) != noise.array_sha256(changed)


def test_tape_prepare_is_no_clobber_and_tamper_evident(tmp_path: Path) -> None:
    root = tmp_path / "tapes"
    snapshot = _snapshot()
    payload = noise.prepare(
        output_root=root,
        prerequisite_snapshot=snapshot,
        protocol_record=_record(contract.PROTOCOL_FREEZE_PATH.as_posix()),
        lock_record=_record(contract.EXECUTION_LOCK_PATH.as_posix()),
        enforce_canonical_destination=False,
    )
    assert payload["passed"] is True
    assert (
        noise.verify_manifest(noise_root=root, enforce_canonical_destination=False)
        == payload
    )
    with pytest.raises(noise.V12R8Q3NoiseError, match="clobber"):
        noise.prepare(
            output_root=root,
            prerequisite_snapshot=snapshot,
            protocol_record=_record(contract.PROTOCOL_FREEZE_PATH.as_posix()),
            lock_record=_record(contract.EXECUTION_LOCK_PATH.as_posix()),
            enforce_canonical_destination=False,
        )
    tape = root / "stochastic_seed_130_standard_normal.npz"
    tape.write_bytes(tape.read_bytes() + b"tamper")
    with pytest.raises(noise.V12R8Q3NoiseError, match="closure drifted"):
        noise.verify_manifest(noise_root=root, enforce_canonical_destination=False)


def test_candidate_config_activates_v26_and_causal_corridor_at_zero() -> None:
    source = {"reward": {"unrelated": 7}, "segment_duration": 0.02}

    def base_builder(_case: Any) -> dict[str, Any]:
        return copy.deepcopy(source)

    result = physical.build_candidate_env_config(
        contract.canonical_case(contract.CASE_IDS[0]),
        base_builder=base_builder,
    )
    assert source == {"reward": {"unrelated": 7}, "segment_duration": 0.02}
    assert physical.resolved_candidate_config(result) == (
        contract.CANDIDATE_RESOLVED_ENV_CONFIG
    )
    assert result["binary_phase_fsm_mode"] == "binary_active"
    assert result["reward"]["morphology_phase_mode"] == (contract.MORPHOLOGY_PHASE_MODE)
    assert result["reward"]["morphology_causal_allow_effects"] == 0.0
    assert result["reward"]["morphology_weight"] == 0.0
    assert result["reward"]["morphology_hard_termination_enabled"] == 0.0


def _recorder(*, mismatch: bool = False) -> SimpleNamespace:
    rewards = [float(index) / 1000.0 for index in range(500)]
    shadows = list(rewards)
    if mismatch:
        shadows[-1] += 1.0
    return SimpleNamespace(
        rewards=rewards,
        shadow_rewards=shadows,
        actions=[[0.0, 0.0] for _ in range(500)],
        observations=[[0.0] * 84 for _ in range(500)],
        morphology_samples=[
            {"phase": float(index) / 496.0, "knee_loss": 0.0} for index in range(496)
        ],
        morphology_diagnostics=[
            {"failed_closed": False, "dropped_pending_sample_count": 4}
        ],
        morphology_term_nonzero_count=0,
        morphology_hard_termination_count=0,
    )


def test_morphology_zero_evidence_proves_live_finite_byte_identity() -> None:
    evidence = physical.morphology_zero_ab_evidence(
        _recorder(), expected_steps=500, detector_sample_count=5_000
    )
    assert evidence["passed"] is True
    assert evidence["corridor_evaluation_count"] == 496
    assert evidence["corridor_unavailable_count"] == 4
    assert (
        evidence["baseline_reward_bytes_sha256"]
        == (evidence["candidate_reward_bytes_sha256"])
    )
    assert gates.morphology_zero_ab_gate(evidence)["passed"] is True


def test_morphology_zero_evidence_fails_reward_or_sample_drift() -> None:
    mismatch = physical.morphology_zero_ab_evidence(
        _recorder(mismatch=True), expected_steps=500, detector_sample_count=5_000
    )
    assert mismatch["passed"] is False
    assert gates.morphology_zero_ab_gate(mismatch)["passed"] is False
    wrong_samples = physical.morphology_zero_ab_evidence(
        _recorder(), expected_steps=500, detector_sample_count=4_999
    )
    assert gates.morphology_zero_ab_gate(wrong_samples)["passed"] is False


def _physical_metrics() -> dict[str, Any]:
    return {
        "episode_metrics": {
            "reserve_norm_nm": {"rms": 1.0, "abs_max": 2.0},
            "residual_norm_nm": {"rms": 0.1, "abs_max": 0.2},
        },
        "sea_episode_metrics": {
            joint: {
                signal: {"rms": 1.0, "abs_max": 2.0} for signal in contract.SEA_SIGNALS
            }
            for joint in contract.JOINTS
        },
    }


def test_comparison_metric_flattening_is_exact_and_finite() -> None:
    values = physical.flatten_comparison_metrics(_physical_metrics())
    assert set(values) == {
        name for name, _absolute, _relative in contract.NONINFERIORITY_TOLERANCES
    }
    malformed = _physical_metrics()
    del malformed["sea_episode_metrics"][contract.JOINTS[0]][contract.SEA_SIGNALS[0]][
        "rms"
    ]
    with pytest.raises(physical.V12R8Q3PhysicalRolloutError, match="missing"):
        physical.flatten_comparison_metrics(malformed)


def test_mature_collector_import_and_contract_adapter_are_available() -> None:
    mature = physical._load_mature_collector()
    assert callable(mature.collect_physical_rollout)
    fallback = SimpleNamespace(SO_POLICY_ID="fallback-policy")
    adapter = physical._ContractAdapter(fallback)
    assert adapter.SCHEMA_VERSION == contract.SCHEMA_VERSION
    assert adapter.SO_POLICY_ID == "fallback-policy"


def test_runner_has_exact_baseline_first_thirteen_stage_order() -> None:
    assert len(runner.STAGE_IDS) == 13
    assert runner.STAGE_IDS[:6] == tuple(
        f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS
    )
    assert runner.STAGE_IDS[6:12] == tuple(
        f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS
    )
    assert runner.STAGE_IDS[-1] == "finalize_qualification"
    assert runner.ACTIVITY_TEMPLATE == {
        **runner.ACTIVITY_TEMPLATE,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def test_rollout_summary_builder_never_fabricates_missing_zero_counters() -> None:
    candidate_id, candidate_module = _candidate()
    case_id = contract.CASE_IDS[0]
    expected = contract.canonical_rollout(contract.CANDIDATE_ROLE, case_id)
    evidence = {
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH.as_posix()),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH.as_posix()),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH.as_posix()),
        "run_start": _record(f"{expected['destination']}/run_start.json"),
        "trace": _record(f"{expected['destination']}/trace.json"),
    }
    summary = runner.build_rollout_summary(
        role=contract.CANDIDATE_ROLE,
        case_id=case_id,
        candidate_id=candidate_id,
        actor_module=candidate_module,
        physical={"steps": 500, "morphology_weight": 9.0},
        noise_tape=_record(expected["noise_tape"]),
        noise_tape_array_sha256=contract.EXPECTED_TAPE_ARRAY_SHA256[
            Path(expected["noise_tape"]).name
        ],
        evidence=evidence,
    )
    assert summary["candidate_id"] == candidate_id
    assert summary["actor_module"] == candidate_module
    assert summary["resolved_env_config"] == contract.CANDIDATE_RESOLVED_ENV_CONFIG
    assert summary["morphology_weight"] == 9.0
    assert summary["checkpoint_zero_created"] is False
    assert summary["positive_morphology_enabled"] is False
    assert summary["runtime_promoted"] is False
    assert all(name not in summary for name in contract.ZERO_REQUIRED_COUNTS)
    prerequisites.bind_candidate(candidate_id, candidate_module)
    gate = gates.common_rollout_gate(
        summary, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    assert gate["checks"]["zero_failures_updates"] is False


def test_r8_q3_cli_reports_deferred_without_publication(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert runner.main(["--deferred-preflight"]) == 0
    assert capsys.readouterr().out.strip() == "DEFERRED_H0_V12R8_Q3_RUNTIME"


def test_live_mutation_paths_remain_deferred_until_r8_terminal_artifacts_exist() -> (
    None
):
    assert freezer.deferred_preflight()["passed"] is False
    assert runner.deferred_preflight()["five_live_r8_verifiers_required"] is True
    assert not freezer.resolve_relative(contract.PROTOCOL_FREEZE_PATH).exists()
    assert not freezer.resolve_relative(contract.EXECUTION_LOCK_PATH).exists()
    assert not freezer.resolve_relative(contract.NOISE_ROOT).exists()
    assert not freezer.resolve_relative(contract.RUN_ROOT).exists()
