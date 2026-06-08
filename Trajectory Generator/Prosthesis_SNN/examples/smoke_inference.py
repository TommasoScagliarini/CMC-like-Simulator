"""Minimal inference example with randomly initialized weights."""

from prosthesis_snn import (
    ProsthesisReferenceSNN,
    ReferenceGenerator,
    SNNConfig,
    SNNProsthesisActionProvider,
)


def main() -> None:
    cfg = SNNConfig(hidden_size=16)
    model = ProsthesisReferenceSNN(cfg)
    generator = ReferenceGenerator(model, cfg=cfg, device="cpu")
    provider = SNNProsthesisActionProvider(generator)

    for t in (0.0, 0.001, 0.002):
        action = provider.get_action(t)
        print(f"t={t:.3f}", action)


if __name__ == "__main__":
    main()
