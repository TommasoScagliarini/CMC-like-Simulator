"""Stateful inference wrapper around the prosthesis SNN."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import torch
import yaml

from .config import SNNConfig
from .device import select_device
from .features import as_feature_vector
from .model import ProsthesisReferenceSNN


class ReferenceGenerator:
    """Checkpoint-aware, stateful prosthetic reference generator."""

    def __init__(
        self,
        model: ProsthesisReferenceSNN,
        cfg: SNNConfig | None = None,
        device: str | torch.device | None = "auto",
        output_scale: Mapping[str, float] | Sequence[float] | None = None,
        output_offset: Mapping[str, float] | Sequence[float] | None = None,
        output_transform: str = "identity",
    ) -> None:
        self.cfg = cfg or model.cfg
        self.device = select_device(device)
        self.model = model.to(self.device)
        self.model.eval()
        self.output_transform = output_transform
        self.output_scale = self._coerce_output_vector(output_scale, default=1.0)
        self.output_offset = self._coerce_output_vector(output_offset, default=0.0)
        self.mem: list[torch.Tensor] | None = None

    @classmethod
    def from_config_file(
        cls,
        config_path: str | Path,
        checkpoint_path: str | Path | None = None,
        device: str | torch.device | None = "auto",
    ) -> "ReferenceGenerator":
        with open(config_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        cfg = SNNConfig.from_mapping(data.get("model", data))
        generator = cls(
            ProsthesisReferenceSNN(cfg),
            cfg=cfg,
            device=device,
            output_scale=data.get("output_scale"),
            output_offset=data.get("output_offset"),
            output_transform=data.get("output_transform", "identity"),
        )
        if checkpoint_path is not None:
            generator.load_checkpoint(checkpoint_path)
        return generator

    @classmethod
    def from_checkpoint(
        cls,
        checkpoint_path: str | Path,
        cfg: SNNConfig | None = None,
        device: str | torch.device | None = "auto",
    ) -> "ReferenceGenerator":
        checkpoint = torch.load(checkpoint_path, map_location="cpu")
        checkpoint_cfg = None
        if isinstance(checkpoint, Mapping):
            checkpoint_cfg = checkpoint.get("config") or checkpoint.get("cfg")
        cfg = cfg or SNNConfig.from_mapping(checkpoint_cfg)
        generator = cls(ProsthesisReferenceSNN(cfg), cfg=cfg, device=device)
        generator.load_checkpoint(checkpoint_path)
        return generator

    def load_checkpoint(self, checkpoint_path: str | Path) -> None:
        checkpoint = torch.load(checkpoint_path, map_location="cpu")
        state_dict = checkpoint
        if isinstance(checkpoint, Mapping):
            state_dict = (
                checkpoint.get("model_state_dict")
                or checkpoint.get("state_dict")
                or checkpoint.get("model")
                or checkpoint
            )
            if "output_scale" in checkpoint:
                self.output_scale = self._coerce_output_vector(checkpoint["output_scale"], default=1.0)
            if "output_offset" in checkpoint:
                self.output_offset = self._coerce_output_vector(checkpoint["output_offset"], default=0.0)
            if "output_transform" in checkpoint:
                self.output_transform = str(checkpoint["output_transform"])

        self.model.load_state_dict(state_dict)
        self.model.to(self.device)
        self.model.eval()
        self.reset()

    def reset(self, batch_size: int = 1) -> None:
        self.mem = self.model.init_mem(batch_size, self.device)

    def predict(
        self,
        features: Mapping[str, Any] | Sequence[float] | np.ndarray,
    ) -> dict[str, dict[str, float]]:
        """Run one inference step and return per-channel coordinate dicts.

        Return shape: ``{channel_name: {coord_name: float}}`` keyed by every
        entry of ``cfg.output_channels``. For the default config this is
        ``{"q": {...}, "qdot": {...}, "qddot": {...}}``.
        """
        x_np = as_feature_vector(features, self.cfg.feature_names)
        x = torch.as_tensor(x_np, dtype=torch.float32, device=self.device).unsqueeze(0)
        if self.mem is None:
            self.reset(batch_size=1)

        try:
            return self._predict_tensor(x)
        except RuntimeError:
            if self.device.type == "cpu":
                raise
            self.device = torch.device("cpu")
            self.model.to(self.device)
            self.reset(batch_size=1)
            return self._predict_tensor(x.cpu())

    def _predict_tensor(self, x: torch.Tensor) -> dict[str, dict[str, float]]:
        with torch.no_grad():
            out, self.mem = self.model(x, self.mem)
            if out.dim() == 2:
                values = out[0]
            else:
                values = out[-1, 0]
            values = self._apply_output_transform(values.detach().cpu().numpy())

        coords = self.cfg.output_coords
        channels = self.cfg.output_channels
        n_channels = len(channels)
        result: dict[str, dict[str, float]] = {channel: {} for channel in channels}
        for i, coord in enumerate(coords):
            for j, channel in enumerate(channels):
                result[channel][coord] = float(values[i * n_channels + j])
        return result

    def _apply_output_transform(self, values: np.ndarray) -> np.ndarray:
        if self.output_transform == "identity":
            transformed = values
        elif self.output_transform == "tanh":
            transformed = np.tanh(values)
        else:
            raise ValueError(f"unknown output_transform: {self.output_transform!r}")
        return transformed * self.output_scale + self.output_offset

    def _coerce_output_vector(
        self,
        values: Mapping[str, Any] | Sequence[float] | None,
        default: float,
    ) -> np.ndarray:
        """Coerce a user-supplied scale/offset spec into a flat (output_size,) vector.

        Accepted forms:
        - ``None``: broadcast ``default`` to every element.
        - Flat sequence/ndarray of length ``output_size``: used as-is.
        - Mapping ``{coord: {channel: value}}``: per-(coord, channel) lookup.
        - Mapping ``{coord: scalar}``: scalar broadcast across all channels for
          that coord (useful when channels share the same scale).
        - Missing keys fall back to ``default``.
        """
        coords = self.cfg.output_coords
        channels = self.cfg.output_channels
        n_channels = len(channels)
        output_size = self.cfg.output_size

        if values is None:
            return np.full(output_size, default, dtype=np.float32)

        if isinstance(values, Mapping):
            out = np.full(output_size, default, dtype=np.float32)
            for i, coord in enumerate(coords):
                entry = values.get(coord)
                if entry is None:
                    continue
                if isinstance(entry, Mapping):
                    for j, channel in enumerate(channels):
                        if channel in entry:
                            out[i * n_channels + j] = float(entry[channel])
                else:
                    for j in range(n_channels):
                        out[i * n_channels + j] = float(entry)
            return out

        arr = np.asarray(values, dtype=np.float32)
        if arr.shape != (output_size,):
            raise ValueError(
                f"expected output vector shape {(output_size,)}, got {arr.shape}"
            )
        return arr
