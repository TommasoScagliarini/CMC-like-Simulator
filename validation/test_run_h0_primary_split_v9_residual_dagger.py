from __future__ import annotations

from pathlib import Path

import numpy as np

import h0_primary_split_v9_causal_teacher_contract as teacher_contract
import h0_primary_split_v9_residual_dagger_contract as contract
import run_h0_primary_split_v9_residual_dagger as runner


def test_isolated_binding_targets_only_fresh_v9_paths() -> None:
    assert runner.engine.contract is contract
    assert runner.engine.teacher_contract is teacher_contract
    assert runner.RUN_ROOT == runner.engine.resolve_relative(contract.RUN_ROOT)
    assert "v9_v26_causal_residual" in str(runner.RUN_ROOT)
    assert runner.P0_MODULE == runner.engine.resolve_relative(contract.P0_MODULE_PATH)
    assert runner.P1_MODULE == runner.engine.resolve_relative(contract.P1_MODULE_PATH)


def test_binding_is_content_pinned_and_records_causal_overrides() -> None:
    binding = runner.IMPLEMENTATION_BINDING
    assert binding["source_sha256"] == (
        "386a43042815c1e14ce1ba3afe09f8e6b5b607694112c780ee3bd4a772ea1ec2"
    )
    assert binding["legacy_shadow_fsm_influence"] is False
    assert binding["partial_fsm_rejection_acceptance"] is False
    claim = runner._claim_payload("0" * 64)
    assert claim["protocol_id"] == contract.PROTOCOL_ID
    assert claim["teacher_id"] == contract.TEACHER_ID
    assert claim["implementation_binding"] == binding


def test_worker_subprocess_reenters_v9_wrapper() -> None:
    command = runner._worker_command("fit_p0", "x" * 32)
    assert Path(command[1]).resolve() == Path(runner.__file__).resolve()
    assert command[2:5] == ["--worker", "--stage", "fit_p0"]


def test_dagger_payload_names_event_causal_teacher(monkeypatch) -> None:
    monkeypatch.setitem(runner.engine.__dict__, "_tree_record", lambda path: {"path": str(path)})
    monkeypatch.setitem(runner.engine.__dict__, "_record", lambda path: {"path": str(path)})
    case = contract.canonical_case(contract.DAGGER_CASE_IDS[0], dagger=True)
    stage_id = f"collect_dagger__{case['case_id']}"
    payload = runner._dagger_start_payload(case=case, stage_id=stage_id)
    assert payload["teacher"] == contract.TEACHER_ID
    assert payload["behavior"] == "P0_CLOSED_LOOP_V26_BINARY_ACTIVE"
    assert runner._is_fsm_event_rejection(ValueError("anything")) is False


def test_causal_teacher_corpus_is_complete_and_deterministic() -> None:
    ledger = runner.verify_teacher_replay()
    assert ledger["passed"] is True
    corpus = runner.load_teacher_corpus(np=np)
    assert corpus["observations"].shape == (3000, 35)
    assert corpus["targets"].shape == (3000, 2)
    assert int(np.count_nonzero(corpus["reset_mask"])) == 6
    assert runner.engine.array_sha256(corpus["targets"]) == (
        "e55b61161ff4b671e9a609c36dd7381665dda534b300fdc1efd92b8db198a6e6"
    )
