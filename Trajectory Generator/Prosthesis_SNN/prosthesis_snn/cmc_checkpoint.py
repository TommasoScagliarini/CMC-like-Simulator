"""Action-contract checks for CMC SNN checkpoints after absolute migration."""

from __future__ import annotations

import json
from pathlib import Path


def require_absolute_reference(generator) -> None:
    mode = generator.checkpoint_metadata.get("env_config", {}).get("action_mode")
    if mode != "absolute":
        raise ValueError(
            "CMC rollout/training requires an absolute-action checkpoint; "
            "the checkpoint declares a different or missing action mode. "
            "Train a new absolute policy; legacy actions cannot be reinterpreted."
        )


def _contract_path(path: str | Path) -> Path:
    return Path(path).with_suffix(Path(path).suffix + ".action_contract.json")


def write_agent_contract(path: str | Path) -> None:
    _contract_path(path).write_text(
        json.dumps({"action_mode": "absolute"}) + "\n", encoding="utf-8"
    )


def require_absolute_agent(path: str | Path) -> None:
    sidecar = _contract_path(path)
    if not sidecar.is_file() or json.loads(sidecar.read_text(encoding="utf-8")).get(
        "action_mode"
    ) != "absolute":
        raise ValueError("Agent resume requires an absolute action-contract sidecar.")
