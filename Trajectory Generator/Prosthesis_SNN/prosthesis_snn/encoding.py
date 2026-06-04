"""Continuous-feature to spike encoders."""

from __future__ import annotations

from typing import Callable

import torch
import torch.nn as nn
import snntorch as snn


class DirectEncoder(nn.Module):
    """Project features and feed them to one LIF tick."""

    def __init__(
        self,
        input_size: int,
        hidden_size: int,
        beta: float,
        threshold: float,
        reset_mechanism: str,
        learn_beta: bool,
        learn_threshold: bool,
        spike_grad: Callable,
    ) -> None:
        super().__init__()
        self.hidden_size = hidden_size
        self.fc = nn.Linear(input_size, hidden_size)
        self.lif = snn.Leaky(
            beta=beta,
            threshold=threshold,
            reset_mechanism=reset_mechanism,
            learn_beta=learn_beta,
            learn_threshold=learn_threshold,
            spike_grad=spike_grad,
        )

    def forward(
        self,
        x: torch.Tensor,
        mem: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        cur = self.fc(x)
        return self.lif(cur, mem)


class RateEncoder(nn.Module):
    """Encode features as average firing rate over T LIF ticks."""

    def __init__(
        self,
        input_size: int,
        hidden_size: int,
        timesteps: int,
        beta: float,
        threshold: float,
        reset_mechanism: str,
        learn_beta: bool,
        learn_threshold: bool,
    ) -> None:
        super().__init__()
        self.timesteps = timesteps
        self.hidden_size = hidden_size
        self.fc = nn.Linear(input_size, hidden_size)
        self.lif = snn.Leaky(
            beta=beta,
            threshold=threshold,
            reset_mechanism=reset_mechanism,
            learn_beta=learn_beta,
            learn_threshold=learn_threshold,
        )

    def forward(
        self,
        x: torch.Tensor,
        mem: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        spk_sum = torch.zeros(x.shape[0], self.hidden_size, device=x.device)
        cur = self.fc(x)
        for _ in range(self.timesteps):
            spk, mem = self.lif(cur, mem)
            spk_sum += spk
        return (spk_sum / self.timesteps).clamp(min=0.05), mem


class LatencyEncoder(nn.Module):
    """Encode features through spike latency over T LIF ticks."""

    def __init__(
        self,
        input_size: int,
        hidden_size: int,
        timesteps: int,
        beta: float,
        threshold: float,
        reset_mechanism: str,
        learn_beta: bool,
        learn_threshold: bool,
        tau: float = 5.0,
    ) -> None:
        super().__init__()
        self.timesteps = timesteps
        self.tau = tau
        self.hidden_size = hidden_size
        self.fc = nn.Linear(input_size, hidden_size)
        self.lif = snn.Leaky(
            beta=beta,
            threshold=threshold,
            reset_mechanism=reset_mechanism,
            learn_beta=learn_beta,
            learn_threshold=learn_threshold,
        )
        t = torch.arange(timesteps, dtype=torch.float32)
        self.register_buffer("time_weights", torch.exp(-t / tau))

    def forward(
        self,
        x: torch.Tensor,
        mem: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        x_norm = torch.sigmoid(self.fc(x))
        spk_sum = torch.zeros(x.shape[0], self.hidden_size, device=x.device)
        for tick in range(self.timesteps):
            cur = x_norm * self.time_weights[tick]
            spk, mem = self.lif(cur, mem)
            spk_sum += spk
        return (spk_sum / self.timesteps).clamp(min=0.05), mem


def build_encoder(cfg, spike_grad: Callable) -> nn.Module:
    common = dict(
        input_size=cfg.input_size,
        hidden_size=cfg.hidden_size,
        beta=cfg.beta,
        threshold=cfg.threshold,
        reset_mechanism=cfg.reset_mechanism,
        learn_beta=cfg.learn_beta,
        learn_threshold=cfg.learn_threshold,
    )

    if cfg.encoding == "direct":
        return DirectEncoder(**common, spike_grad=spike_grad)
    if cfg.encoding == "rate":
        return RateEncoder(**common, timesteps=cfg.encoding_timesteps)
    if cfg.encoding == "latency":
        return LatencyEncoder(**common, timesteps=cfg.encoding_timesteps)

    raise ValueError(
        f"unknown encoding {cfg.encoding!r}; use 'direct', 'rate', or 'latency'"
    )
