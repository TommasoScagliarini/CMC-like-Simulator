"""Public trainer options cannot silently select a different pipeline."""
from pathlib import Path
import sys

import pytest

from unified_MLP import train_ppo_mlp as trainer

ROOT = Path(trainer.__file__).resolve().parent


def parse(monkeypatch, *extra):
    monkeypatch.setattr(sys, "argv", ["train", "--config", str(ROOT / "configs/pretraining.yaml"), *extra])
    return trainer.parse_args()


def test_pretraining_supports_balanced_collection_with_ten_epochs(monkeypatch):
    args = parse(monkeypatch)
    trainer._validate_unified_args(args)
    trainer._validate_start_sampling_args(args)
    assert args.num_epochs == 10
    assert args._start_sampling_contract.expected_steps_per_start == 1536


def test_hidden_legacy_architecture_override_rejected(monkeypatch):
    args = parse(monkeypatch, "--fcnet-hiddens", "16", "16")
    with pytest.raises(SystemExit, match="256"):
        trainer._validate_unified_args(args)


def test_exnovo_requires_constant_exploration(monkeypatch):
    args = parse(monkeypatch)
    args.reward_mode = "ex_novo"
    args.init_actor_from = "source"
    args.freeze_logstd = True
    args.fixed_exploration_sigma = None
    with pytest.raises(SystemExit, match="constant exploration"):
        trainer._validate_unified_args(args)


def test_baseline_output_and_path_escape_rejected():
    for value in (str(ROOT.parent / "runs/training/protected"), "../../../../baseline_MLP"):
        with pytest.raises(ValueError, match="Output must stay"):
            trainer._resolve_output_dir(value, "test")
    expected = ROOT / "runs/training/new_run"
    assert trainer._resolve_output_dir("Trajectory Generator/unified_MLP/runs/training/new_run", "test") == expected


def test_baseline_history_is_never_updated(monkeypatch):
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if "update_historical_runs" in name:
            raise AssertionError("baseline history imported")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded)
    trainer._update_history_best_effort(True)
