from __future__ import annotations

from pathlib import Path

import h0_primary_split_v8_residual_dagger_contract as contract
import run_h0_primary_split_v8_residual_dagger as runner


def test_binding_targets_only_v8r1p1_contracts_and_paths() -> None:
    assert runner.contract is contract
    assert runner.contract.PROTOCOL_ID == contract.PROTOCOL_ID
    assert runner.teacher_contract.PROTOCOL_ID == contract.PROTOCOL_ID
    assert runner.RUN_ROOT == runner.resolve_relative(contract.RUN_ROOT)
    assert "2026-08-07_h0_primary_split_v8r1p1_v26_residual" in str(runner.RUN_ROOT)
    assert runner.P0_MODULE == runner.resolve_relative(contract.P0_MODULE_PATH)
    assert runner.P1_MODULE == runner.resolve_relative(contract.P1_MODULE_PATH)


def test_binding_is_content_pinned_and_recorded_in_claim() -> None:
    binding = runner.IMPLEMENTATION_BINDING
    assert binding["source_sha256"] == (
        "386a43042815c1e14ce1ba3afe09f8e6b5b607694112c780ee3bd4a772ea1ec2"
    )
    assert binding["partial_fsm_rejection_acceptance"] is False
    assert len(binding["bound_source_sha256"]) == 64
    claim = runner._claim_payload("0" * 64)
    assert claim["protocol_id"] == contract.PROTOCOL_ID
    assert claim["implementation_binding"] == binding


def test_worker_subprocess_reenters_v8r1p1_wrapper() -> None:
    command = runner._worker_command("fit_p0", "x" * 32)
    assert Path(command[1]).resolve() == Path(runner.__file__).resolve()
    assert command[2:5] == ["--worker", "--stage", "fit_p0"]


def test_historical_partial_fsm_completion_is_disabled() -> None:
    assert runner._is_fsm_event_rejection(
        ValueError("Actor FSM rejected a V26 active event: {}")
    ) is False
    assert contract.AUTHORITY["partial_fsm_rejection_acceptance_authorized"] is False


def test_bound_engine_uses_v8r1p1_error_and_candidate_namespace() -> None:
    assert runner.V8R1P1ResidualDaggerError.__name__ == "V8R1P1ResidualDaggerError"
    source_constants = {
        value
        for value in runner._freeze_p1.__code__.co_consts
        if isinstance(value, str)
    }
    assert any("H0_PRIMARY_SPLIT_V8R1P1_P1_" in value for value in source_constants)
    assert not any("H0_PRIMARY_SPLIT_V6_P1_" in value for value in source_constants)
