from __future__ import annotations

import ast
from pathlib import Path

import h0_v12r4_p3_coverage_contract as contract
import run_h0_v12r4_p3_coverage as runner


def test_stage_receipts_match_the_five_public_q2_prerequisites() -> None:
    assert runner.PROTOCOL_FREEZE_PATH == runner.resolve_relative(
        contract.PROTOCOL_FREEZE_PATH
    )
    assert runner.LOCK_PATH == runner.resolve_relative(contract.EXECUTION_LOCK_PATH)
    assert runner._stage_receipt_path("freeze_candidate_p3") == runner.resolve_relative(
        contract.CANDIDATE_FREEZE_PATH
    )
    assert runner._stage_receipt_path(
        "finalize_development"
    ) == runner.resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    assert runner.PIPELINE_LEDGER_PATH == runner.resolve_relative(
        contract.PIPELINE_LEDGER_PATH
    )


def test_raw_journal_is_observer_only_and_tolerates_missing_clearance() -> None:
    journal = runner._diagnostic_raw_journal(
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
    assert journal["samples"][0]["left_heel_clearance_m"] is None
    assert journal["accepted_events"] == [{"event": "heel_strike"}]


def test_runner_has_no_training_api_or_qualification_execution() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    attributes = {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }
    assert "train" not in attributes
    assert "learn_on_batch" not in attributes
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert 'checkpoint_zero_created": True' not in source


def test_lock_defers_candidate_identity_but_binds_selection_rule() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert source.count("contract.CANDIDATE_SELECTION_RULE") >= 4
    assert source.count('"DEFERRED_UNTIL_FIT_P3"') >= 4
    assert "RLModule.from_checkpoint(path.resolve())" in source


def test_live_activity_counts_partial_work_without_completed_stage_estimates() -> None:
    runner._reset_activity()
    runner._begin_stage_activity("collect_cov__deterministic_offset_nominal")
    runner._activity_increment("environment_reset_calls")
    runner._activity_increment("teacher_query_count", 3)
    runner._activity_increment("environment_step_calls", 2)
    runner._activity_increment("raw_sensor_sample_count", 20)
    assert runner._ACTIVITY_TOTALS["environment_reset_calls"] == 1
    assert runner._ACTIVITY_TOTALS["teacher_query_count"] == 3
    assert runner._ACTIVITY_TOTALS["environment_step_calls"] == 2
    assert runner._ACTIVITY_TOTALS["raw_sensor_sample_count"] == 20
    row = runner._STAGE_ACTIVITY["collect_cov__deterministic_offset_nominal"]
    assert row["teacher_query_count"] == 3
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert '"activity_totals": copy.deepcopy(_ACTIVITY_TOTALS)' in source
    assert '"environment_step_calls": 500 * sum' not in source
