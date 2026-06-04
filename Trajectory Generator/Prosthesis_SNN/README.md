# Prosthesis_SNN

Portable SNN scaffold for prosthetic kinematic reference generation.

This project extracts the reusable SNN pieces from the Crazyflie SNN repo and
reshapes them into a small inference package for a transfemoral prosthesis model
with SEA-actuated knee and ankle. It is intentionally independent from Isaac
Lab, skrl, and OpenSim at runtime import time.

## What Is Included

- Spiking encoders: direct, rate, and latency.
- Rectangular surrogate gradient.
- Portable LIF backbone and non-spiking LIF output head.
- `ProsthesisReferenceSNN` for prosthetic reference outputs.
- `ReferenceGenerator` for checkpoint loading and inference.
- Minimal CMC-like PPO/SNN training smoke entrypoint.
- `ReferenceProvider` adapters matching the CMC-like simulator style:
  `get(t, state=None) -> (q_ref, qdot_ref, qddot_ref)`.
- Documentation for unresolved integration decisions.

## What Is Not Included

- Isaac Lab entrypoints.
- Crazyflie task configs.
- Long-run training entrypoints and tuned PPO experiment configs.
- Training checkpoints from the Crazyflie task.
- Direct edits to `CMC-like-Simulator - Claude`.

Those pieces are intentionally skipped because the target is portable prosthetic
reference generation, not quadcopter RL.

## Install

Create an environment with Python 3.10+ and install:

```powershell
pip install -r requirements.txt
```

Choose the PyTorch build from the official selector for the target machine:

- Windows i7 + NVIDIA: CUDA build when available.
- Mac M-series: CPU or MPS-capable build.
- Any machine: CPU is the baseline fallback.

## Smoke Test

```powershell
python tests/smoke_test.py
```

The smoke test checks:

- SNN inference shape for batch size 1 and batch size N.
- `ReferenceGenerator.predict(...)`.
- Hybrid provider behavior: biological references are preserved while
  prosthetic references are overridden.
- Training helper imports without requiring `skrl` for core inference.

## Optional Training

The training-side PPO agent lives in `prosthesis_snn.training.PPO_SNN`. The
matching shared-backbone actor-critic wrapper is
`prosthesis_snn.training.ProsthesisSNNActorCritic`; it exposes SNN membrane
state through skrl's `rnn` metadata and can be passed as both policy and value
model. The design mirrors the Crazyflie SNN repo while keeping the value head
training-only and the policy/reference head exportable for inference.

After training, export an inference-only checkpoint with
`prosthesis_snn.training.save_reference_checkpoint(...)`. The saved file
contains only the reference SNN weights, config, output scaling/offset, and
metadata, and can be loaded with `ReferenceGenerator.from_checkpoint(...)`.

A minimal CMC-like end-to-end smoke is available:

```powershell
python -m prosthesis_snn.training.cmc_ppo_smoke --setup-xml path/to/setup.xml
```

It runs the current trajectory-knot environment contract, performs one tiny
PPO update, saves an agent checkpoint, exports an inference checkpoint, reloads
it through `ReferenceGenerator`, and verifies finite outputs. If `--setup-xml`
is omitted, the simulator's persisted last setup is used.

Install the optional training dependencies only in the training environment:

```powershell
pip install -e .[training]
```

The core inference package intentionally does not import `skrl` unless the
training module is used.

For the combined SNN + OpenSim simulator environment on macOS, prefer the
`envCMC-like` conda environment with PyTorch installed from `conda-forge`.
The PyPI wheel for `torch` can conflict with OpenSim/conda OpenMP runtimes
(`libomp.dylib` duplicate initialization).

## Current Integration Direction

The future simulator integration should replace direct dependence on
`KinematicsInterpolator` with a provider abstraction:

```python
q_ref, qdot_ref, qddot_ref = reference_provider.get(t, state)
```

For the first integration pass, keep the existing IK/spline reference for all
biological coordinates and override only:

- `pros_knee_angle`
- `pros_ankle_angle`

See `docs/TODO_integration.md` before making training or feature-interface
decisions.
