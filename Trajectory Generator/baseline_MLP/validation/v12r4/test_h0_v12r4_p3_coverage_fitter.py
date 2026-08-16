from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

import h0_v12r4_p3_coverage_contract as contract
import h0_v12r4_p3_coverage_fitter as fitter


def test_adamw_schedule_boundaries() -> None:
    assert fitter.adamw_learning_rate(1) == 3.0e-4
    assert fitter.adamw_learning_rate(1500) == 3.0e-4
    assert fitter.adamw_learning_rate(1501) == 1.0e-4
    assert fitter.adamw_learning_rate(2500) == 1.0e-4
    assert fitter.adamw_learning_rate(2501) == 3.0e-5
    assert fitter.adamw_learning_rate(3000) == 3.0e-5
    with pytest.raises(fitter.V12R4CoverageFitError):
        fitter.adamw_learning_rate(3001)


def test_p2_piece_is_byte_bound_and_semantically_exact() -> None:
    arrays, binding = fitter._load_p2_piece()
    assert arrays["observations"].shape == (contract.P2_CORPUS_ROWS, 35)
    assert arrays["actions"].shape == (contract.P2_CORPUS_ROWS, 2)
    assert int(np.count_nonzero(arrays["reset_mask"])) == 18
    assert binding["artifact"] == contract.P2_CORPUS_ARTIFACT


def test_fitter_ast_has_one_fixed_optimizer_and_no_environment() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    keywords = {
        keyword.arg: ast.literal_eval(keyword.value)
        for call in calls
        for keyword in call.keywords
        if keyword.arg in {"max_iter", "max_eval"}
        and isinstance(keyword.value, ast.Constant)
    }
    assert keywords == {"max_iter": 600, "max_eval": 1200}
    assert ".train(" not in source
    assert ".reset(" not in source
    assert "make_cmc_env" not in source
    assert "env.step(" not in source
    assert 'hard_polish_used": True' not in source


def test_source_checkpoint_load_is_explicitly_absolute() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    assert "RLModule.from_checkpoint(source.resolve())" in source
    assert contract.FIT["continued_from_p2"] is False
