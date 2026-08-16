from __future__ import annotations

import json
from pathlib import Path

import run_h0_primary_split_v10s_safe_dagger as runner


def test_stage_order_and_cumulative_collection_receipts_are_exact() -> None:
    assert runner._expected_stage_order() == runner.contract.STAGE_IDS
    assert len(runner.contract.STAGE_IDS) == 18
    assert runner._cumulative_collection_receipt_paths("p0") == ()
    assert len(runner._cumulative_collection_receipt_paths("p1")) == 2
    assert len(runner._cumulative_collection_receipt_paths("p2")) == 4
    p3 = runner._cumulative_collection_receipt_paths("p3")
    assert len(p3) == 6
    assert p3[0] == runner._stage_receipt_path(
        "collect_r1__deterministic_offset_minus_0p20"
    )
    assert p3[-1] == runner._stage_receipt_path(
        "collect_r3__stochastic_nominal_seed_126"
    )


def test_real_v10a_receipt_is_the_only_teacher_evidence() -> None:
    path = runner._teacher_evidence_path()
    assert path == runner.REPO_ROOT / (
        "validation/"
        "h0_primary_split_v10_time_alignment_adjudication_receipt.json"
    )
    receipt = runner._mapping(path)
    assert runner._teacher_evidence_passed(receipt) is True
    assert runner.contract.teacher_evidence_gate(receipt)["passed"] is True


def test_claims_authorize_only_four_actor_fits_and_zero_critic_ppo(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {"path": str(path), "sha256": "0" * 64, "size_bytes": 1},
    )
    claim = runner._claim_payload("a" * 64)
    assert claim["actor_updates"] == 0
    assert claim["actor_updates_authorized"] == 4
    assert claim["critic_updates"] == 0
    assert claim["ppo_updates"] == 0
    assert claim["protected_trials_opened"] == []
    assert claim["reserve_trials_opened"] == []
    assert claim["retry_authorized"] is False

    fit = runner._worker_claim_payload(
        stage_id="fit_p0", token_sha256="a" * 64, previous_receipts=[]
    )
    collection = runner._worker_claim_payload(
        stage_id="collect_r1__deterministic_offset_minus_0p20",
        token_sha256="a" * 64,
        previous_receipts=[],
    )
    assert fit["actor_updates"] == 0
    assert fit["actor_updates_authorized"] == 1
    assert collection["actor_updates_authorized"] == 0


def test_collection_start_declares_teacher_dependent_behavior_and_strict_gate(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {"path": str(path), "sha256": "0" * 64, "size_bytes": 1},
    )
    monkeypatch.setattr(
        runner,
        "_tree_record",
        lambda path: {"path": str(path), "tree_sha256": "1" * 64},
    )
    case = runner.contract.canonical_collection_case(
        "deterministic_offset_minus_0p20", 1
    )
    payload = runner._collection_start_payload(
        case=case,
        stage_id="collect_r1__deterministic_offset_minus_0p20",
        candidate_module=Path("candidate"),
    )
    assert payload["behavior"] == runner.contract.COLLECTION_BEHAVIOR
    assert payload["requested_alpha_student_weight"] == 0.25
    assert payload["alpha_semantics"] == "STUDENT_WEIGHT_LATCH_FORCES_ZERO"
    assert payload["physical_gate_relaxed"] is False
    assert payload["teacher_id"] == runner.contract.TEACHER_ID
    assert payload["actor_updates"] == 0


def test_final_start_forbids_teacher_blend_and_latch(monkeypatch) -> None:
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {"path": str(path), "sha256": "0" * 64, "size_bytes": 1},
    )
    case = runner.contract.canonical_final_case("deterministic_offset_nominal")
    payload = runner._final_start_payload(
        case=case,
        stage_id="final__deterministic_offset_nominal",
        freeze={"candidate_id": "H0_PRIMARY_SPLIT_V10S_P3_deadbeef", "candidate_module": {}},
    )
    assert payload["behavior"] == runner.contract.FINAL_BEHAVIOR
    assert payload["teacher_enabled"] is False
    assert payload["blending_enabled"] is False
    assert payload["safety_latch_enabled"] is False
    assert payload["actor_updates"] == 0


def test_worker_dispatch_is_claimed_and_canonical(monkeypatch) -> None:
    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        runner,
        "verify_worker_claim",
        lambda stage_id, token: calls.append((stage_id, token)),
    )
    monkeypatch.setattr(
        runner,
        "_run_fit_stage",
        lambda stage: {"passed": True, "fit_stage": stage},
    )
    result = runner.run_worker(stage_id="fit_p2", supervisor_token="x" * 32)
    assert result == {"passed": True, "fit_stage": "p2"}
    assert calls == [("fit_p2", "x" * 32)]


def test_failed_fit_after_persisted_update_is_not_undercounted(
    monkeypatch, tmp_path: Path
) -> None:
    fit_root = tmp_path / "p1"
    fit_root.mkdir()
    receipt_path = fit_root / "receipt.json"
    summary_path = fit_root / "summary.json"
    claim_path = tmp_path / "fit_p1.claim.json"
    for path in (receipt_path, summary_path):
        path.write_text(json.dumps({"actor_updates": 1}), encoding="utf-8")
    claim_path.write_text("{}", encoding="utf-8")
    monkeypatch.setitem(runner.contract.FIT_RECEIPT_PATHS, "p1", str(receipt_path))
    monkeypatch.setitem(runner.contract.FIT_ROOTS, "p1", str(fit_root))
    monkeypatch.setattr(runner, "resolve_relative", lambda path: Path(path))
    monkeypatch.setattr(runner, "_claim_path", lambda stage_id: claim_path)
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {"path": str(path), "sha256": "0" * 64, "size_bytes": 1},
    )

    evidence = runner._fit_actor_update_evidence("p1")
    assert evidence["confirmed_actor_updates"] == 1
    assert evidence["actor_update_attempted_or_unknown"] is False
    assert evidence["worker_claim_present"] is True

    receipt_path.unlink()
    summary_path.unlink()
    ambiguous = runner._fit_actor_update_evidence("p1")
    assert ambiguous["confirmed_actor_updates"] == 0
    assert ambiguous["actor_update_attempted_or_unknown"] is True

    source = Path(runner.__file__).read_text(encoding="utf-8")
    execute_source = source[source.index("def execute()") : source.index("def _worker_command")]
    assert '"actor_updates": confirmed_actor_updates' in execute_source
    assert "completed_fit_count" not in execute_source


def test_completed_round_count_does_not_depend_on_following_fit() -> None:
    completed = [
        "fit_p0",
        *(
            f"collect_r{round_index}__{case_id}"
            for round_index in (1, 2, 3)
            for case_id in runner.contract.COLLECTION_CASE_IDS
        ),
        "fit_p1",
        "fit_p2",
    ]
    assert "fit_p3" not in completed
    assert runner._completed_safe_dagger_round_count(completed) == 3


def test_source_orders_candidate_query_before_same_state_teacher_and_isolates_final() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    collection = source[
        source.index("def _collect_safe_dagger") : source.index(
            "def verify_collection_receipt"
        )
    ]
    assert collection.index("candidate_mean, candidate_std = _query_mean_std") < (
        collection.index("teacher_mean, teacher_std = _query_mean_std")
    )
    assert '"teacher_queried_on_same_state": True' in collection
    assert '"served_action_teacher_dependency_count"' in collection

    final = source[
        source.index("def _final_rollout") : source.index(
            "def verify_final_rollout_receipt"
        )
    ]
    assert "RLModule.from_checkpoint(SOURCE_H0_MODULE)" not in final
    assert '"teacher_query_count": 0' in final
    assert '"served_action_teacher_dependency_count": 0' in final


def test_cli_rejects_unclaimed_worker_without_writing() -> None:
    assert runner.main(["--worker"]) == 2
