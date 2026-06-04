# Extraction Notes

Source repository:

`C:\Users\tomma\Desktop\-sim-SNN-Colangelo-Mardaru-Scagliarini`

## Reused Concepts

- LIF SNN backbone from `src/model/snn_model.py`.
- Direct, rate, and latency encoders from `src/model/encoding.py`.
- Rectangular surrogate gradient from `src/model/surrogate_gradient.py`.
- Explicit membrane state handling so inference can carry state across control
  timesteps.

## Adapted Pieces

- Removed `skrl.models.torch.Model`, `GaussianMixin`, and
  `DeterministicMixin`.
- Removed policy/value split and PPO cache logic.
- Reduced the model to one prosthetic reference output head.
- Replaced OmegaConf/DictConfig assumptions with a plain dataclass.
- Added device fallback helpers for CPU/CUDA/MPS portability.
- Added provider adapters shaped like the CMC-like simulator's
  `KinematicsInterpolator.get(t)` interface.

## Intentionally Skipped

- `main_hydra.py` and `play.py`: tied to Isaac Lab and Crazyflie.
- `src/agent/ppo_snn.py`: skrl PPO agent, not portable inference core.
- `src/task/crazyflie/*`: task-specific observation/action definitions.
- Crazyflie checkpoints and logs: trained for quadcopter observations/actions,
  not transferable to prosthetic kinematics.
- `CrazyflieSNNPreprocessor`: hard-coded to a 12-dimensional quadcopter
  observation layout.

## Integration Reference

The future target is expected to resemble:

`C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude`

The relevant current pattern is:

```python
q_ref, qdot_ref, qddot_ref = kin.get(t)
```

Future integration should swap `kin` for a reference provider or wrap it with
`HybridReferenceProvider`, not hard-wire SNN calls into the controllers.
