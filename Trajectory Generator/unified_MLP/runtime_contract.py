"""Compare saved and requested runtime settings without changing checkpoints.

Actor feature names alone do not capture governor dynamics or the GRF routing.
The resolved configuration is a separate compatibility check; existing unified
V1 exports keep their original manifest schema.
"""
from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path, PureWindowsPath
from typing import Any

from unified_MLP._bootstrap import REPO_ROOT


# The policy/integration time step is deliberately NOT an execution-only field.
EXECUTION_ONLY_SIMULATION_KEYS = frozenset({
    "episode_duration", "iterations", "max_steps", "episode_start_offset_s",
    "episode_start_offset_choices_s", "step_wall_timeout_s",
})
_ARCHITECTURE_KEYS = (
    "num_hidden_layers", "dim_hidden_layers", "fcnet_activation",
    "asymmetric_actor_critic", "rl_module_kind",
)


def _path(value: Any) -> Any:
    if value is None or value == "":
        return value
    text = os.fspath(value).replace("\\", "/")
    # A foreign Windows absolute path must not be silently treated as a relative
    # POSIX path. Portable presets use repository-relative paths on both hosts.
    if os.name != "nt" and PureWindowsPath(text).is_absolute() and not text.startswith("/"):
        return PureWindowsPath(text).as_posix()
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return os.path.normcase(str(path.resolve()))


def _canonical(value: Any, *, key: str = "") -> Any:
    if key == "setup_xml" or key.endswith(("_profile", "_profile_file")):
        return _path(value)
    if isinstance(value, Mapping):
        return {name: _canonical(item, key=str(name)) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    return value


def _section(config: Mapping[str, Any], name: str, label: str) -> Mapping[str, Any]:
    section = config.get(name)
    if not isinstance(section, Mapping) or not section:
        raise ValueError(f"{label} resolved config requires a nonempty {name} section")
    return section


def _runtime(config: Mapping[str, Any] | None, label: str) -> dict:
    if not isinstance(config, Mapping):
        raise ValueError(f"{label} resolved runtime configuration is missing")
    simulation = _section(config, "simulation", label)
    grf = _section(config, "grf", label)
    model = _section(config, "model", label)
    missing = [key for key in _ARCHITECTURE_KEYS if key not in model]
    if missing:
        raise ValueError(f"{label} resolved model architecture is missing {missing}")
    architecture = {key: _canonical(model[key]) for key in _ARCHITECTURE_KEYS}
    # An explicit legacy override that is equivalent to the ordinary dimensions
    # is compatible; a narrower/wider network is not.
    hiddens = model.get("fcnet_hiddens")
    if not hiddens:
        hiddens = [model["dim_hidden_layers"]] * int(model["num_hidden_layers"])
    architecture["fcnet_hiddens"] = _canonical(hiddens)
    return {
        "model": architecture,
        "simulation": _canonical({
            key: value for key, value in simulation.items()
            if key not in EXECUTION_ONLY_SIMULATION_KEYS
        }),
        "grf": _canonical(grf),
    }


def _differences(source: Any, target: Any, prefix: str = "") -> list[str]:
    if isinstance(source, Mapping) and isinstance(target, Mapping):
        result: list[str] = []
        for key in sorted(set(source) | set(target)):
            name = f"{prefix}.{key}" if prefix else str(key)
            if key not in source:
                result.append(f"{name} (missing from source)")
            elif key not in target:
                result.append(f"{name} (missing from target)")
            else:
                result.extend(_differences(source[key], target[key], name))
        return result
    return [] if source == target else [prefix]


def validate_runtime_compatibility(
    source_config: Mapping[str, Any] | None,
    target_config: Mapping[str, Any] | None,
) -> None:
    """Reject dynamics, sensors, input semantics or architecture drift.

    Inputs are nested resolved YAML mappings (``model/simulation/grf``), normally
    loaded with ``training_config.load_resolved_for_checkpoint``. Reward, PPO,
    exploration/freeze settings, resources and output identity are independent
    of this check. Neither mapping is changed. Missing saved config fails closed.
    """
    differences = _differences(_runtime(source_config, "source"), _runtime(target_config, "target"))
    if differences:
        raise ValueError("unified runtime mismatch: " + ", ".join(differences))
