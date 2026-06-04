"""Surrogate gradients for spiking neurons."""

from __future__ import annotations

import torch


class RectangularSurrogate(torch.autograd.Function):
    """Rectangular surrogate for the backward pass of a spike function."""

    @staticmethod
    def forward(ctx, input_: torch.Tensor, width: float) -> torch.Tensor:
        ctx.save_for_backward(input_)
        ctx.width = width
        return (input_ > 0).float()

    @staticmethod
    def backward(ctx, grad_output: torch.Tensor):
        (input_,) = ctx.saved_tensors
        width = ctx.width
        mask = (torch.abs(input_) < (width / 2.0)).float()
        return grad_output * (1.0 / width) * mask, None


def rectangular_sg(width: float = 0.5):
    """Bind the rectangular surrogate width and return a spike function."""

    def inner(x: torch.Tensor) -> torch.Tensor:
        return RectangularSurrogate.apply(x, width)

    return inner
