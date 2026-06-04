"""Minimal inference example with randomly initialized weights."""

from prosthesis_snn import (
    ProsthesisReferenceSNN,
    ReferenceGenerator,
    SNNConfig,
    SNNProsthesisReferenceProvider,
)


def main() -> None:
    cfg = SNNConfig(hidden_size=16)
    model = ProsthesisReferenceSNN(cfg)
    generator = ReferenceGenerator(model, cfg=cfg, device="cpu")
    provider = SNNProsthesisReferenceProvider(generator)

    for t in (0.0, 0.001, 0.002):
        q_ref, qdot_ref, qddot_ref = provider.get(t)
        print(f"t={t:.3f}", q_ref, qdot_ref, qddot_ref)


if __name__ == "__main__":
    main()
