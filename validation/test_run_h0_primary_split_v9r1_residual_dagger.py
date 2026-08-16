from __future__ import annotations

from pathlib import Path

import numpy as np

import h0_primary_split_v9r1_residual_dagger_contract as contract
import h0_primary_split_v9r1_teacher_compat_contract as teacher_contract
import run_h0_primary_split_v9r1_residual_dagger as runner


def test_binding_targets_fresh_v9r1_paths_and_compat_contract() -> None:
    assert runner.engine.contract is contract
    assert runner.engine.teacher_contract is teacher_contract
    assert runner.engine.RUN_ROOT == runner.engine.resolve_relative(contract.RUN_ROOT)
    assert "v9r1_v26_causal_residual" in str(runner.engine.RUN_ROOT)
    assert runner.IMPLEMENTATION_BINDING["scientific_change_from_v9"] is False


def test_worker_reenters_v9r1_wrapper() -> None:
    command = runner._worker_command("fit_p0", "x" * 32)
    assert Path(command[1]).resolve() == Path(runner.__file__).resolve()
    assert command[2:5] == ["--worker", "--stage", "fit_p0"]


def test_reused_causal_corpus_is_exact() -> None:
    ledger = runner.verify_teacher_replay()
    assert ledger["passed"] is True
    corpus = runner.load_teacher_corpus(np=np)
    assert corpus["observations"].shape == (3000, 35)
    assert runner.engine.array_sha256(corpus["targets"]) == (
        "e55b61161ff4b671e9a609c36dd7381665dda534b300fdc1efd92b8db198a6e6"
    )


def test_preflight_preserves_v9_fail_and_passed_p0() -> None:
    payload = runner.build_preflight()
    assert payload["passed"] is True
    assert payload["checks"]["v9_terminal_fail_preserved"] is True
    assert payload["checks"]["v9_p0_scientific_pass_preserved"] is True
    assert payload["checks"]["feature_name_aliases_exact"] is True
