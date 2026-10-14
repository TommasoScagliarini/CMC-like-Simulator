"""Reference selection survives configuration/export and rejects mixed semantics."""
import argparse
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from unified_MLP_39 import actor_transition, runtime_contract, training_config
from unified_MLP_39.target_availability import (
    PRESCRIBED_CLOCK_FEATURES, add_cli_arguments, configuration, env_options,
    target_contract, target_feature_names,
)


def contract(reference="left", scale=1.0):
    names = list(PRESCRIBED_CLOCK_FEATURES) + [f"actor_{i}" for i in range(33)]
    names += list(target_feature_names(reference))
    return actor_transition.build_contract(
        names, names + [f"critic_{i}" for i in range(49)], [-1, -1], [1, 1],
        target_reference=reference, target_velocity_scale=scale,
    )


@pytest.mark.parametrize("reference", ["left", "right_v26"])
@pytest.mark.parametrize("scale", [1.0, 10.0])
def test_reference_contract_export_roundtrip(reference, scale, tmp_path):
    expected = contract(reference, scale)
    actor_transition.write_contract(tmp_path, expected)
    restored = actor_transition.read_contract(tmp_path)
    assert restored == expected
    actor_transition.validate_contract(restored, expected)
    assert restored["target_channels"] == target_contract(scale, target_reference=reference)


@pytest.mark.parametrize("source,destination", [("left", "right_v26"), ("right_v26", "left")])
def test_reference_switch_cannot_silently_restore_policy(source, destination):
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(contract(source), contract(destination))


@pytest.mark.parametrize("key,value", [
    ("phase_shift_cycles", {"pros_knee_angle": 0.5, "pros_ankle_angle": 0.5}),
    ("clock_phase_offset_cycles", 0.465),
    ("clock_encoding", "sin(2*pi*left_phase),cos(2*pi*left_phase)"),
    ("teacher_clock", "prescribed_left_grf_heel_strikes"),
])
def test_right_contract_rejects_shift_and_clock_drift(key, value, tmp_path):
    expected = contract("right_v26")
    wrong = copy.deepcopy(expected)
    wrong["target_channels"][key] = value
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(wrong, expected)
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.write_contract(tmp_path, wrong)


def test_right_contract_rejects_left_named_target_slots():
    wrong = contract("right_v26")
    wrong["actor_feature_names"][35:39] = target_feature_names("left")
    wrong["observation_feature_names"][:39] = wrong["actor_feature_names"]
    with pytest.raises(ValueError, match="contract mismatch"):
        actor_transition.validate_contract(wrong, contract("right_v26"))


@pytest.mark.parametrize("cli_reference", [None, "left", "right_v26"])
def test_yaml_cli_snapshot_environment_reference_are_consistent(cli_reference, tmp_path):
    parser = argparse.ArgumentParser()
    add_cli_arguments(parser)
    defaults, reward = training_config.to_argparse_defaults({"target_availability": {
        "target_reference": "right_v26", "target_velocity_scale": 1.0,
        "target_availability_probability": 1.0,
    }})
    parser.set_defaults(**defaults)
    args = parser.parse_args([] if cli_reference is None else ["--target-reference", cli_reference])
    expected = cli_reference or "right_v26"
    assert args.target_reference == expected
    assert env_options(args)["target_reference"] == expected
    assert env_options(args, evaluation=True)["target_reference"] == expected
    snapshot = tmp_path / "training_cfg.resolved.yaml"
    training_config.dump_resolved(args, reward, snapshot)
    assert training_config.load(snapshot)["target_availability"]["target_reference"] == expected


@pytest.mark.parametrize("invalid", ["right", "LEFT", "right_v62", "", None, True])
def test_invalid_reference_is_rejected_by_yaml_and_environment_options(invalid):
    with pytest.raises(ValueError, match="target_reference"):
        training_config.to_argparse_defaults({"target_availability": {"target_reference": invalid}})
    with pytest.raises(ValueError, match="target_reference"):
        env_options(SimpleNamespace(target_reference=invalid))


def test_invalid_reference_is_rejected_by_cli():
    parser = argparse.ArgumentParser()
    add_cli_arguments(parser)
    with pytest.raises(SystemExit):
        parser.parse_args(["--target-reference", "right"])


def test_left_mask_configuration_keeps_previous_serialized_shape():
    expected = {
        "probability": 0.0, "curriculum_probabilities": [], "block_iterations": 10,
        "mask_seed": 123, "target_velocity_scale": 10.0,
    }
    assert configuration(SimpleNamespace()) == expected
    assert configuration(SimpleNamespace(target_reference="left")) == expected


def test_existing_best72_left_contract_rebuilds_identically():
    # This repository keeps historical artifacts locally; omit only this archive
    # regression when installing the package without its experiment data.
    archived = (Path(__file__).resolve().parents[1] / "runs" / "training"
                / "MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
                / "rl_module_last" / "unified_contract.json")
    if not archived.is_file():
        pytest.skip("historical LEFT checkpoint manifest is not installed")
    old = json.loads(archived.read_text(encoding="utf-8"))
    rebuilt = actor_transition.build_contract(
        old["actor_feature_names"], old["observation_feature_names"],
        old["action_low"], old["action_high"],
        target_velocity_scale=old["target_channels"]["target_velocity_scale"],
    )
    assert rebuilt == old


def test_runtime_default_left_compatible_with_explicit_left_but_not_right():
    original = {
        "model": {"num_hidden_layers": 2, "dim_hidden_layers": 256, "fcnet_activation": "tanh",
                  "asymmetric_actor_critic": True, "rl_module_kind": "asymmetric"},
        "simulation": {"action_mode": "absolute"}, "grf": {"grf_mode": "online_sensor"},
        "target_availability": {"target_velocity_scale": 1.0},
    }
    explicit = copy.deepcopy(original)
    explicit["target_availability"]["target_reference"] = "left"
    runtime_contract.validate_runtime_compatibility(original, explicit)
    explicit["target_availability"]["target_reference"] = "right_v26"
    with pytest.raises(ValueError, match="target_reference"):
        runtime_contract.validate_runtime_compatibility(original, explicit)
