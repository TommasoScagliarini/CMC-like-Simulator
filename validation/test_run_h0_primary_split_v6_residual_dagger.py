"""Isolated tests for V6 residual/DAgger execution scaffolding."""

from __future__ import annotations

import hashlib
import inspect
import json
import tempfile
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from validation import h0_primary_split_v6_residual_dagger_contract as contract
from validation import run_h0_primary_split_v6_residual_dagger as runner


def _teacher_rows() -> list[dict]:
    return [
        {
            "step": step,
            "v25_observation": np.full(35, step / 500.0, dtype=np.float32).tolist(),
            "queried_teacher_mean": [0.1, -0.2],
        }
        for step in range(1, 501)
    ]


def test_resolve_relative_rejects_escape_absolute_and_noncanonical() -> None:
    for value in ("../escape", "/absolute", "a/../b", "a//b", ""):
        with pytest.raises(runner.V6ResidualDaggerError):
            runner.resolve_relative(value)


def test_tree_record_closes_every_file_and_detects_byte_change() -> None:
    validation_root = runner.REPO_ROOT / "validation"
    with tempfile.TemporaryDirectory(dir=validation_root) as temporary:
        root = Path(temporary)
        (root / "nested").mkdir()
        (root / "a.bin").write_bytes(b"a")
        (root / "nested" / "b.bin").write_bytes(b"b")
        first = runner._tree_record(root)
        assert first["file_count"] == 2
        assert [row["path"] for row in first["files"]] == [
            "a.bin",
            "nested/b.bin",
        ]
        (root / "a.bin").write_bytes(b"changed")
        second = runner._tree_record(root)
        assert first["tree_sha256"] != second["tree_sha256"]
        assert not runner._tree_record_matches(first, root)


def test_teacher_corpus_is_exactly_six_by_500_with_six_reset_rows() -> None:
    with (
        mock.patch.object(runner, "verify_teacher_replay", return_value={}),
        mock.patch.object(runner, "_sequence", side_effect=[_teacher_rows()] * 6),
    ):
        corpus = runner.load_teacher_corpus(np=np)
    assert corpus["observations"].shape == (3000, 35)
    assert corpus["observations"].dtype == np.float32
    assert corpus["targets"].shape == (3000, 2)
    assert np.count_nonzero(corpus["reset_mask"]) == 6
    assert corpus["origins"][0].startswith("teacher:")


def test_teacher_corpus_rejects_short_or_nonfinite_trace() -> None:
    with (
        mock.patch.object(runner, "verify_teacher_replay", return_value={}),
        mock.patch.object(runner, "_sequence", return_value=_teacher_rows()[:-1]),
        pytest.raises(runner.V6ResidualDaggerError),
    ):
        runner.load_teacher_corpus(np=np)

    rows = _teacher_rows()
    rows[10]["v25_observation"][2] = float("nan")
    with (
        mock.patch.object(runner, "verify_teacher_replay", return_value={}),
        mock.patch.object(runner, "_sequence", return_value=rows),
        pytest.raises(runner.V6ResidualDaggerError),
    ):
        runner.load_teacher_corpus(np=np)


def test_normalization_uses_only_columns_2_to_34_and_floor() -> None:
    values = np.zeros((4, 35), dtype=np.float32)
    values[:, 0] = [0, 100, 200, 300]
    values[:, 1] = [-10, -20, -30, -40]
    values[:, 2] = [0, 1, 2, 3]
    mean, std = runner._normalization(values, np=np)
    assert mean.shape == (33,)
    assert std.shape == (33,)
    assert mean[0] == np.float32(1.5)
    assert std[0] > contract.NORMALIZATION_STD_FLOOR
    assert np.all(std[1:] == np.float32(contract.NORMALIZATION_STD_FLOOR))


def test_frozen_stochastic_innovations_are_derived_without_rng() -> None:
    case = contract.canonical_case("stochastic_nominal_seed_126")
    rows = []
    for step in range(1, 501):
        mean = np.asarray([0.1, -0.2], dtype=np.float32)
        innovation = np.asarray([0.25, -0.5], dtype=np.float32)
        std = np.asarray([0.005, 0.005], dtype=np.float32)
        rows.append(
            {
                "step": step,
                "frozen_raw_action": (mean + std * innovation).tolist(),
                "frozen_teacher_mean": mean.tolist(),
                "teacher_std": std.tolist(),
            }
        )
    with (
        mock.patch.object(contract, "canonical_case", return_value=case),
        mock.patch.object(runner, "_sequence", return_value=rows),
    ):
        values = runner._frozen_innovations(case["case_id"], np=np)
    np.testing.assert_allclose(
        values,
        np.tile(np.asarray([0.25, -0.5], dtype=np.float32), (500, 1)),
        rtol=0,
        atol=2e-6,
    )


def _adapter_rejection_error() -> ValueError:
    payload = {
        "invalid_event_type": "hs_too_early_after_to",
        "state_name": "SWING_AFTER_TO",
        "adapted_events": [
            {
                "side": "left",
                "event": "heel_strike",
                "source": "v25_fsm_v20",
                "event_contract_id": contract.EVENT_CONTRACT_ID,
                "event_time_s": 18.493,
                "confirmed_time_s": 18.498,
                "delivered_time_s": 18.506,
            }
        ],
    }
    return ValueError(
        "Actor FSM rejected a V20 active event: "
        + json.dumps(payload, allow_nan=False, sort_keys=True, separators=(",", ":"))
    )


def test_only_exact_adapter_fsm_rejection_is_reclassifiable() -> None:
    error = _adapter_rejection_error()
    assert runner._is_fsm_event_rejection(error)


@pytest.mark.parametrize(
    "error",
    [
        ValueError("event timestamps must be finite and causal"),
        RuntimeError("FSM transition rejected: out-of-order heel strike"),
        ValueError("Actor FSM rejected a V20 active event: not-json"),
        TimeoutError("event timeout"),
        RuntimeError("integrator rejected state"),
        ValueError("non-finite observation"),
        KeyError("invalid event"),
    ],
)
def test_unrelated_errors_cannot_be_reclassified_as_dagger_completion(error) -> None:
    assert not runner._is_fsm_event_rejection(error)


def test_worker_requires_token_and_terminal_ledger_blocks_execution() -> None:
    assert runner.main(["--worker", "--stage", "fit_p0"]) == 2
    validation_root = runner.REPO_ROOT / "validation"
    with tempfile.TemporaryDirectory(dir=validation_root) as temporary:
        ledger = Path(temporary) / "ledger.json"
        ledger.write_text("{}\n", encoding="utf-8")
        with (
            mock.patch.object(runner, "PIPELINE_LEDGER", ledger),
            pytest.raises(runner.V6ResidualDaggerError, match="terminal"),
        ):
            runner.verify_worker_claim("fit_p0", "x" * 32)


def test_worker_command_has_ephemeral_token_but_claim_stores_only_hash() -> None:
    token = "supervisor-secret-token-long-enough-123"
    command = runner._worker_command("fit_p0", token)
    assert command[-2:] == ["--supervisor-token", token]
    claim = runner._claim_payload(runner._token_sha256(token))
    assert token not in str(claim)
    assert (
        claim["execution_token_sha256"]
        == hashlib.sha256(token.encode("utf-8")).hexdigest()
    )


def test_source_encodes_causal_dagger_and_persist_before_step() -> None:
    source = inspect.getsource(runner._collect_dagger)
    candidate_position = source.index("raw_action, candidate_mean")
    paired_position = source.index("paired = build_paired_views")
    teacher_position = source.index("_teacher_raw, teacher_mean")
    persist_position = source.index("writer.write_step")
    env_step_position = source.index("env.step")
    assert candidate_position < paired_position < teacher_position
    assert teacher_position < persist_position < env_step_position
    assert "online_grf_detector" in inspect.getsource(
        v1_update := runner.v1._update_shadow_fsm
    )
    assert "legacy_online_events" in inspect.getsource(v1_update)


def test_fit_source_is_residual_only_full_batch_and_never_updates_critic() -> None:
    source = inspect.getsource(runner._fit_stage)
    assert "prepare_residual_fit" in source
    assert "torch.optim.AdamW" in source
    assert "clip_grad_norm_" in source
    assert "DataLoader" not in source
    assert "optimizer.step()" in source
    assert 'critic_updates": 0' in source


def test_import_path_has_no_top_level_execution() -> None:
    source = Path(runner.__file__).read_text(encoding="utf-8")
    assert 'if __name__ == "__main__":' in source
    assert "raise SystemExit(main())" in source
