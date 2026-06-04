"""Portable SNN model for prosthetic reference generation."""

from __future__ import annotations

from typing import Optional

import torch
import torch.nn as nn
import snntorch as snn

from .config import SNNConfig
from .encoding import build_encoder
from .surrogate_gradient import rectangular_sg


class NoSpikingLIF(nn.Module):
    """LIF-style membrane accumulation without output spikes."""

    def __init__(
        self,
        in_size: int,
        out_size: int,
        beta: float,
        learn_beta: bool = False,
    ) -> None:
        super().__init__()
        self.fc = nn.Linear(in_size, out_size)
        beta_init = torch.full((out_size,), beta)
        if learn_beta:
            self.beta = nn.Parameter(beta_init)
        else:
            self.register_buffer("beta", beta_init)

    def forward(
        self,
        spk: torch.Tensor,
        mem: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if spk.dim() == 2:  # For inference
            new_mem = self.beta * mem + self.fc(spk)
            return new_mem, new_mem

        outputs = []        # For training
        cur_mem = mem
        current_seq = self.fc(spk)
        for tick in range(spk.shape[0]):
            cur_mem = self.beta * cur_mem + current_seq[tick]
            outputs.append(cur_mem)
        return torch.stack(outputs), cur_mem


class SNNBackbone(nn.Module):
    """Feature encoder plus configurable hidden LIF layers."""

    def __init__(self, cfg: SNNConfig) -> None:
        super().__init__()
        self.cfg = cfg
        self.hidden_size = cfg.hidden_size
        self.num_layers = cfg.num_layers

        spike_grad = rectangular_sg(width=0.5)
        self.encoder = build_encoder(cfg, spike_grad)
        self.fc_hidden = nn.ModuleList(
            [nn.Linear(cfg.hidden_size, cfg.hidden_size) for _ in range(cfg.num_layers)]
        )
        self.lif_layers = nn.ModuleList(
            [
                snn.Leaky(
                    beta=cfg.beta,
                    threshold=cfg.threshold,
                    reset_mechanism=cfg.reset_mechanism,
                    learn_beta=cfg.learn_beta,
                    learn_threshold=cfg.learn_threshold,
                    spike_grad=spike_grad,
                )
                for _ in range(cfg.num_layers)
            ]
        )
        self.last_firing_rate = 0.0

    def forward(
        self,
        x: torch.Tensor,
        mem: list[torch.Tensor],
    ) -> tuple[torch.Tensor, list[torch.Tensor]]:
        if x.dim() == 2:
            x = x.unsqueeze(0) # input

        outputs = []
        cur_mem = list(mem)
        for tick in range(x.shape[0]):
            spk, encoder_mem = self.encoder(x[tick], cur_mem[0])
            next_mem = [encoder_mem]
            for layer_idx, lif in enumerate(self.lif_layers):
                spk, layer_mem = lif(
                    self.fc_hidden[layer_idx](spk),
                    cur_mem[layer_idx + 1],
                )
                next_mem.append(layer_mem)
            outputs.append(spk)
            cur_mem = next_mem

        self.last_firing_rate = float(outputs[-1].mean().detach().cpu())
        features = torch.stack(outputs)
        if features.shape[0] == 1:
            features = features.squeeze(0)
        return features, cur_mem


class ProsthesisReferenceSNN(nn.Module):
    """
    SNN that predicts prosthetic kinematic references.

    The model returns a flat tensor of size `cfg.output_size`, which equals
    `len(output_coords) * len(output_channels)`. With the default config this
    is 6 = 2 joints × 3 channels (q, qdot, qddot). The downstream
    `ReferenceGenerator` is responsible for unpacking the flat output into
    per-channel coordinate dictionaries.

    Memory slots are explicit so the caller can carry membrane state across
    simulator timesteps and reset it at gait-cycle or episode boundaries.
    """

    def __init__(self, cfg: SNNConfig | None = None) -> None:
        super().__init__()
        self.cfg = cfg or SNNConfig()
        self.backbone = SNNBackbone(self.cfg)
        self.output_lif = NoSpikingLIF(
            self.cfg.hidden_size,
            self.cfg.output_size,
            self.cfg.beta,
            self.cfg.learn_beta,
        )

    def init_mem(
        self,
        batch_size: int,
        device: torch.device | str | None = None,
    ) -> list[torch.Tensor]:
        device = torch.device(device) if device is not None else next(self.parameters()).device
        hidden = self.cfg.hidden_size
        return (
            [torch.zeros(batch_size, hidden, device=device) for _ in range(self.cfg.num_layers + 1)]
            + [torch.zeros(batch_size, self.cfg.output_size, device=device)]
        )

    def forward(
        self,
        x: torch.Tensor,
        mem: Optional[list[torch.Tensor]] = None,
    ) -> tuple[torch.Tensor, list[torch.Tensor]]:
        if x.dim() == 1:
            x = x.unsqueeze(0)
        if x.dim() not in (2, 3):
            raise ValueError("x must have shape (features), (batch, features), or (time, batch, features)")

        batch_size = x.shape[0] if x.dim() == 2 else x.shape[1]
        if mem is None:
            mem = self.init_mem(batch_size, x.device)

        bb_mem = mem[:-1]
        out_mem = mem[-1]
        features, new_bb_mem = self.backbone(x, bb_mem)
        out, new_out_mem = self.output_lif(features, out_mem)
        return out, new_bb_mem + [new_out_mem]
