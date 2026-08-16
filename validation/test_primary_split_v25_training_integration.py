"""Pure regression tests for explicit V25-residual trainer selection."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(BASELINE))

import train_ppo_mlp as train  # noqa: E402
import training_config  # noqa: E402


def _residual_args(**overrides):
    values = {
        "rl_module_kind": "primary_split_v25_residual",
        "asymmetric_actor_critic": True,
        "warm_start": False,
        "warm_start_raw": False,
        "phase_fsm_input_mode": "legacy_events",
        "event_contract_id": "legacy_events_v1",
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_event_contract_id": (
            "binary_point_v25+functional_contact_fsm_v1"
        ),
        "binary_phase_detector_profile": "v25.json",
        "online_grf_detector_profile": "legacy_detector.json",
        "primary_split_v25_residual_input_mean": [0.0] * 33,
        "primary_split_v25_residual_input_std": [1.0] * 33,
        "primary_split_v25_residual_limits": [0.175, 0.12],
        "primary_split_v25_residual_init_seed": 20260806,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_historical_module_default_is_unchanged() -> None:
    args = SimpleNamespace(
        rl_module_kind="standard",
        primary_split_v25_residual_input_mean=None,
        primary_split_v25_residual_input_std=None,
        primary_split_v25_residual_limits=None,
        primary_split_v25_residual_init_seed=None,
    )
    train._validate_rl_module_args(args)  # noqa: SLF001
    assert args.rl_module_kind == "standard"


def test_standard_module_rejects_silently_ignored_residual_parameters() -> None:
    args = SimpleNamespace(
        rl_module_kind="standard",
        primary_split_v25_residual_input_mean=[0.0] * 33,
        primary_split_v25_residual_input_std=None,
        primary_split_v25_residual_limits=None,
        primary_split_v25_residual_init_seed=None,
    )
    with pytest.raises(SystemExit, match="V25 residual parameters require"):
        train._validate_rl_module_args(args)  # noqa: SLF001


def test_residual_module_requires_exact_active_routing_and_shapes() -> None:
    args = _residual_args()
    train._validate_rl_module_args(args)  # noqa: SLF001
    assert len(args.primary_split_v25_residual_input_mean) == 33
    assert args.primary_split_v25_residual_limits == [0.175, 0.12]

    for override, message in (
        ({"binary_phase_fsm_mode": "binary_shadow"}, "active event routing"),
        ({"primary_split_v25_residual_input_std": [1.0] * 32}, "exactly 33"),
        ({"primary_split_v25_residual_limits": [0.0, 0.12]}, "must be > 0"),
        ({"warm_start_raw": True}, "would omit residual state"),
    ):
        with pytest.raises(SystemExit, match=message):
            train._validate_rl_module_args(_residual_args(**override))  # noqa: SLF001


def test_residual_selection_builds_the_production_module_and_full_model_config() -> (
    None
):
    from primary_split_v25_residual import PrimarySplitV25ResidualTorchRLModule

    args = _residual_args(
        fcnet_activation="tanh",
        freeze_logstd=True,
        freeze_actor=False,
    )
    train._validate_rl_module_args(args)  # noqa: SLF001
    module_class, model_config = train._asymmetric_module_selection(  # noqa: SLF001
        args,
        n_actor=35,
        n_full=84,
        hiddens=[256, 256],
    )
    assert module_class is PrimarySplitV25ResidualTorchRLModule
    assert model_config["n_actor"] == 35
    assert model_config["n_full"] == 84
    assert model_config["primary_split_v25_residual_input_mean"] == [0.0] * 33
    assert model_config["primary_split_v25_residual_input_std"] == [1.0] * 33
    assert model_config["primary_split_v25_residual_limits"] == [0.175, 0.12]
    assert model_config["primary_split_v25_residual_init_seed"] == 20260806
    assert model_config["primary_split_v25_residual_reset_bypass"] is False


def test_residual_model_values_round_trip_in_resolved_yaml() -> None:
    args = _residual_args()
    # dump_resolved only reads fields present in SECTION_MAP; unrelated sections
    # are intentionally absent from this focused namespace.
    with tempfile.TemporaryDirectory() as tmp:
        destination = Path(tmp) / training_config.RESOLVED_CONFIG_NAME
        training_config.dump_resolved(args, {"morphology_weight": 0.0}, destination)
        payload = yaml.safe_load(destination.read_text(encoding="utf-8"))
        flat, reward = training_config.to_argparse_defaults(payload)

    assert flat["rl_module_kind"] == "primary_split_v25_residual"
    assert flat["primary_split_v25_residual_input_mean"] == [0.0] * 33
    assert flat["primary_split_v25_residual_input_std"] == [1.0] * 33
    assert flat["primary_split_v25_residual_limits"] == [0.175, 0.12]
    assert flat["primary_split_v25_residual_init_seed"] == 20260806
    assert reward["morphology_weight"] == 0.0


def test_parser_default_and_explicit_residual_selection() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        cfg = Path(tmp) / "empty.yaml"
        cfg.write_text("{}\n", encoding="utf-8")
        base = ["train_ppo_mlp.py", "--config", str(cfg), "--output-dir", "unused"]
        with mock.patch.object(sys, "argv", base):
            default = train.parse_args()
        assert default.rl_module_kind == "standard"
        assert default.primary_split_v25_residual_input_mean is None
        assert default.binary_phase_fsm_mode == "disabled"

        explicit = [
            *base,
            "--asymmetric-actor-critic",
            "--rl-module-kind",
            "primary_split_v25_residual",
            "--binary-phase-detector-profile",
            "v25.json",
            "--online-grf-detector-profile",
            "legacy_detector.json",
            "--binary-phase-fsm-mode",
            "binary_active",
            "--binary-phase-event-contract-id",
            "binary_point_v25+functional_contact_fsm_v1",
            "--primary-split-v25-residual-input-mean",
            *("0" for _ in range(33)),
            "--primary-split-v25-residual-input-std",
            *("1" for _ in range(33)),
            "--primary-split-v25-residual-limits",
            "0.175",
            "0.12",
            "--primary-split-v25-residual-init-seed",
            "20260806",
        ]
        with mock.patch.object(sys, "argv", explicit):
            selected = train.parse_args()
        train._validate_rl_module_args(selected)  # noqa: SLF001
        assert selected.rl_module_kind == "primary_split_v25_residual"
        assert selected.binary_phase_fsm_mode == "binary_active"


# Keep pytest import below project imports so a missing optional training stack
# cannot mask collection of this pure validation module.
import pytest  # noqa: E402
