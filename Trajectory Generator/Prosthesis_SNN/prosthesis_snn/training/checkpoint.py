"""Checkpoint export helpers for inference-time prosthesis references."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Sequence

import torch

from ..config import SNNConfig
from ..model import ProsthesisReferenceSNN

REFERENCE_CHECKPOINT_FORMAT = "prosthesis_snn_reference"
REFERENCE_CHECKPOINT_VERSION = 1


def build_reference_checkpoint(
    model: Any,
    cfg: SNNConfig | None = None,
    output_scale: Mapping[str, Any] | Sequence[float] | None = None,
    output_offset: Mapping[str, Any] | Sequence[float] | None = None,
    output_transform: str = "identity",
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build an inference checkpoint loadable by ``ReferenceGenerator``.

    ``model`` may be either:

    - ``ProsthesisSNNActorCritic``: only its policy/reference SNN is exported;
    - ``ProsthesisReferenceSNN``: exported directly.

    Optimizer state, value head weights, PPO log-std parameters, and skrl
    bookkeeping are intentionally excluded.
    """

    cfg = _resolve_config(model, cfg)
    state_dict = _reference_state_dict(model)

    checkpoint: dict[str, Any] = {
        "format": REFERENCE_CHECKPOINT_FORMAT,
        "format_version": REFERENCE_CHECKPOINT_VERSION,
        "model_state_dict": _cpu_state_dict(state_dict),
        "config": cfg.to_dict(),
        "output_transform": str(output_transform),
        "output_scale": _plain_value(
            output_scale if output_scale is not None else [1.0] * cfg.output_size
        ),
        "output_offset": _plain_value(
            output_offset if output_offset is not None else [0.0] * cfg.output_size
        ),
        "metadata": {
            "source_class": type(model).__name__,
            **dict(metadata or {}),
        },
    }
    return checkpoint


def save_reference_checkpoint(
    path: str | Path,
    model: Any,
    cfg: SNNConfig | None = None,
    output_scale: Mapping[str, Any] | Sequence[float] | None = None,
    output_offset: Mapping[str, Any] | Sequence[float] | None = None,
    output_transform: str = "identity",
    metadata: Mapping[str, Any] | None = None,
) -> Path:
    """Save an inference checkpoint and return its path."""

    checkpoint = build_reference_checkpoint(
        model=model,
        cfg=cfg,
        output_scale=output_scale,
        output_offset=output_offset,
        output_transform=output_transform,
        metadata=metadata,
    )
    checkpoint_path = Path(path)
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, checkpoint_path)
    return checkpoint_path


def _resolve_config(model: Any, cfg: SNNConfig | None) -> SNNConfig:
    if cfg is not None:
        return cfg
    model_cfg = getattr(model, "cfg", None)
    if isinstance(model_cfg, SNNConfig):
        return model_cfg
    reference_model = getattr(model, "reference_model", None)
    reference_cfg = getattr(reference_model, "cfg", None)
    if isinstance(reference_cfg, SNNConfig):
        return reference_cfg
    raise TypeError("cfg is required when model does not expose an SNNConfig")


def _reference_state_dict(model: Any) -> Mapping[str, torch.Tensor]:
    if hasattr(model, "reference_state_dict"):
        return model.reference_state_dict()
    if isinstance(model, ProsthesisReferenceSNN):
        return model.state_dict()

    reference_model = getattr(model, "reference_model", None)
    if isinstance(reference_model, ProsthesisReferenceSNN):
        return reference_model.state_dict()

    raise TypeError(
        "model must be a ProsthesisReferenceSNN or expose reference_state_dict()"
    )


def _cpu_state_dict(
    state_dict: Mapping[str, torch.Tensor],
) -> dict[str, torch.Tensor]:
    return {
        key: value.detach().cpu().clone()
        for key, value in state_dict.items()
    }


def _plain_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _plain_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_plain_value(item) for item in value]
    if isinstance(value, list):
        return [_plain_value(item) for item in value]
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    return value
