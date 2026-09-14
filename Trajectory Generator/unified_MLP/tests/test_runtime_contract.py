"""Checkpoint runtime compatibility, independent of Ray and OpenSim."""
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from unified_MLP._bootstrap import REPO_ROOT
from unified_MLP.runtime_contract import validate_runtime_compatibility


CONFIGS = Path(__file__).resolve().parents[1] / "configs"


def config(name="pretraining"):
    return yaml.safe_load((CONFIGS / f"{name}.yaml").read_text())


def test_imitation_and_exnovo_presets_are_runtime_compatible():
    source, target = config(), config("ex_novo")
    before = deepcopy((source, target))
    validate_runtime_compatibility(source, target)
    assert (source, target) == before


@pytest.mark.parametrize("section,key,value", [
    ("simulation", "pros_ref_cutoff_hz", 5.0),
    ("simulation", "pros_knee_target_slew_rate_limit_rad_s", 3.0),
    ("simulation", "setup_xml", "models/another/setup.xml"),
    ("simulation", "segment_duration", 0.02),
    ("simulation", "imitation_initialize_to_target", True),
    ("simulation", "random_init", True),
    ("simulation", "include_controller_state_observation", False),
    ("simulation", "reward_reference_range_floor", 0.2),
    ("grf", "online_grf_profile", "online_grf_profiles/other.json"),
    ("grf", "online_grf_applied_side", ["right"]),
    ("grf", "binary_phase_actor_fsm_version", "v2"),
    ("model", "dim_hidden_layers", 128),
    ("model", "fcnet_activation", "relu"),
    ("model", "fcnet_hiddens", [256, 128]),
])
def test_physical_and_architecture_drift_is_rejected(section, key, value):
    source, target = config(), config()
    target[section][key] = value
    with pytest.raises(ValueError, match=rf"{section}\.{key}"):
        validate_runtime_compatibility(source, target)


def test_execution_sizes_starts_and_optimizer_can_change():
    source, target = config(), config()
    target["simulation"].update({
        "episode_duration": 0.04, "iterations": 1, "max_steps": 4,
        "episode_start_offset_s": 0.2, "episode_start_offset_choices_s": [0.2, 0.5],
        "step_wall_timeout_s": 30,
    })
    target["ppo"].update({"lr": 0.5, "num_epochs": 1, "train_batch_size": 8})
    target["parallelism"] = {"num_env_runners": 0, "ray_num_cpus": 1}
    target["model"].update({"freeze_actor": True, "freeze_logstd": True,
                             "fixed_exploration_sigma": [0.01], "seed": 321})
    validate_runtime_compatibility(source, target)


def test_paths_are_repo_relative_and_separator_independent():
    source, target = config(), config()
    target["simulation"]["setup_xml"] = str(REPO_ROOT / source["simulation"]["setup_xml"]).replace("/", "\\")
    for key, value in source["grf"].items():
        if key.endswith("_profile"):
            target["grf"][key] = str(REPO_ROOT / value)
        elif isinstance(value, list):
            target["grf"][key] = tuple(value)
    target["model"]["fcnet_hiddens"] = (256, 256)
    validate_runtime_compatibility(source, target)


def test_external_absolute_path_is_not_confused_with_repo_asset():
    source, target = config(), config()
    target["simulation"]["setup_xml"] = "/external/other_repo/" + source["simulation"]["setup_xml"]
    with pytest.raises(ValueError, match="simulation.setup_xml"):
        validate_runtime_compatibility(source, target)


@pytest.mark.parametrize("missing", [None, {}, {"simulation": {}}])
def test_missing_saved_runtime_is_rejected(missing):
    with pytest.raises(ValueError, match="source"):
        validate_runtime_compatibility(missing, config())


def test_missing_physical_field_is_rejected():
    source, target = config(), config()
    del target["simulation"]["pros_ref_cutoff_hz"]
    with pytest.raises(ValueError, match="simulation.pros_ref_cutoff_hz"):
        validate_runtime_compatibility(source, target)
