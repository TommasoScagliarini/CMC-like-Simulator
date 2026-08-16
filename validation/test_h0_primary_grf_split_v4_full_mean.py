from __future__ import annotations

import ast
from pathlib import Path

import numpy as np

import h0_primary_grf_split_v4_freeze_contract as contract
import run_h0_primary_grf_split_v4_full_mean as runner


def _actor_state() -> dict[str, np.ndarray]:
    first_weight = np.zeros((3, 35), dtype=np.float32)
    first_bias = np.zeros(3, dtype=np.float32)
    second_weight = np.zeros((3, 3), dtype=np.float32)
    second_bias = np.zeros(3, dtype=np.float32)
    output_weight = np.zeros((4, 3), dtype=np.float32)
    output_bias = np.zeros(4, dtype=np.float32)
    return {
        "pi_encoder.0.weight": first_weight.copy(),
        "pi_encoder.0.bias": first_bias.copy(),
        "pi_encoder.2.weight": second_weight.copy(),
        "pi_encoder.2.bias": second_bias.copy(),
        "pi.0.0.weight": first_weight.copy(),
        "pi.0.0.bias": first_bias.copy(),
        "pi.0.2.weight": second_weight.copy(),
        "pi.0.2.bias": second_bias.copy(),
        "pi.1.weight": output_weight.copy(),
        "pi.1.bias": output_bias.copy(),
    }


def test_full_mean_audit_accepts_hidden_and_mean_output_with_frozen_logstd() -> None:
    source = _actor_state()
    candidate = {key: value.copy() for key, value in source.items()}
    candidate["pi_encoder.0.weight"][0, 10] = 0.25
    candidate["pi.0.0.weight"][0, 10] = 0.25
    candidate["pi_encoder.2.bias"][0] = 0.1
    candidate["pi.0.2.bias"][0] = 0.1
    candidate["pi.1.weight"][0, 0] = 0.5
    candidate["pi.1.bias"][1] = -0.2
    audit = runner.full_mean_update_audit(source, candidate)
    assert audit["changes_confined_to_full_mean_network"] is True
    assert audit["hidden_mean_network_changed"] is True
    assert audit["mean_output_changed"] is True
    assert audit["logstd_parameter_rows_bit_exact"] is True
    assert audit["encoder_aliases_bit_exact"] is True
    assert audit["disabled_clock_columns_zero"] is True


def test_full_mean_audit_rejects_logstd_change() -> None:
    source = _actor_state()
    candidate = {key: value.copy() for key, value in source.items()}
    candidate["pi_encoder.0.bias"][0] = 0.1
    candidate["pi.0.0.bias"][0] = 0.1
    candidate["pi.1.weight"][0, 0] = 0.2
    candidate["pi.1.weight"][2, 0] = 0.01
    audit = runner.full_mean_update_audit(source, candidate)
    assert audit["logstd_parameter_rows_bit_exact"] is False
    assert audit["changes_confined_to_full_mean_network"] is False


def test_v3_engine_binding_is_process_local_and_restored(monkeypatch) -> None:
    names = (
        "LOCK",
        "RUN_ROOT",
        "ATTEMPT_CLAIM",
        "CANDIDATE_FREEZE",
        "HOLDOUT_ACCESS_CLAIM",
        "verify_lock",
        "_verify_attempt_claim",
        "verify_candidate_freeze",
        "verify_holdout_access_claim",
    )
    original = {name: getattr(runner.v3, name) for name in names}
    snapshots = {
        "lock": {"status": "lock"},
        "attempt": {"status": "attempt"},
        "candidate": {"status": "candidate"},
        "access": {"status": "access"},
    }
    monkeypatch.setattr(runner, "verify_lock", lambda: snapshots["lock"])
    monkeypatch.setattr(runner, "_verify_attempt_claim", lambda: snapshots["attempt"])
    monkeypatch.setattr(
        runner, "verify_candidate_freeze", lambda: snapshots["candidate"]
    )
    monkeypatch.setattr(
        runner, "verify_holdout_access_claim", lambda: snapshots["access"]
    )
    with runner._bind_frozen_v3_holdout_engine():
        assert runner.v3.LOCK == runner.LOCK
        assert runner.v3.RUN_ROOT == runner.RUN_ROOT
        assert runner.v3.verify_lock() is snapshots["lock"]
        assert runner.v3._verify_attempt_claim() is snapshots["attempt"]
        assert runner.v3.verify_candidate_freeze() is snapshots["candidate"]
        assert runner.v3.verify_holdout_access_claim() is snapshots["access"]
    assert all(getattr(runner.v3, name) is value for name, value in original.items())


def test_runner_hardcodes_full_mean_fit_and_post_freeze_holdout_order() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    fit_calls = [
        node
        for node in calls
        if isinstance(node.func, ast.Attribute) and node.func.attr == "adapt_actor"
    ]
    assert len(fit_calls) == 1
    keyword = {
        item.arg: item.value for item in fit_calls[0].keywords if item.arg is not None
    }
    assert isinstance(keyword["trainable_first_layer_features"], ast.Constant)
    assert keyword["trainable_first_layer_features"].value is None
    execute_start = source.index("def execute")
    assert source.index("freeze_candidate_before_holdout()", execute_start) < (
        source.index("claim_holdout_access()", execute_start)
    )
    assert source.index("claim_holdout_access()", execute_start) < source.index(
        '"--holdout-replay-worker"', execute_start
    )


def test_public_schemas_match_runner_literal_outputs() -> None:
    tree = ast.parse(Path(runner.__file__).read_text(encoding="utf-8"))
    observed: dict[str, list[set[str]]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name) or not isinstance(node.value, ast.Dict):
            continue
        keys = {
            key.value
            for key in node.value.keys
            if isinstance(key, ast.Constant) and isinstance(key.value, str)
        }
        observed.setdefault(target.id, []).append(keys)
    assert contract.ADAPTATION_RECEIPT_KEYS in observed["receipt"]
    assert contract.ACTOR_MANIFEST_KEYS in observed["actor_manifest"]
    assert contract.HOLDOUT_REPLAY_RECEIPT_KEYS in observed["wrapper"]
    assert contract.HOLDOUT_RECEIPT_KEYS in observed["receipt"]
    assert contract.EXECUTION_LEDGER_KEYS in observed["ledger"]
