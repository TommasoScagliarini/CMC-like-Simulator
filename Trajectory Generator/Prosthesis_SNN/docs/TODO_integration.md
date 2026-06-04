# TODO Integration Decisions

This file collects the open decisions that should be resolved before the SNN is
integrated into the current OpenSim/CMC-like project.

## Resolved decisions

The following decisions have been taken and are now load-bearing for the
design. They live here (and not only in the code) so that future contributors
can see *why* a given default exists.

- **Platform priority**: Windows x86_64. macOS arm64 portability is deferred
  and does not block design choices. All training-side libraries are allowed
  (skrl, OpenSim, Hydra/OmegaConf, etc.).
- **Muscle control**: the OpenSim model's muscles are driven by the simulator
  from an external kinematic file. The SNN only generates the SEA references
  for `pros_knee_angle` and `pros_ankle_angle`. The "human" agent split used
  in some upstream multi-agent RL repos is not adopted.
- **Action parametrization**: the policy outputs a `Box(6,)` =
  (q, qdot, qddot) × (knee, ankle). The three channels per joint are emitted
  as independent network outputs.
- **Output coherence strategy**: **soft**. The (q, qdot, qddot) triplet is
  not constrained by construction; instead a coherence penalty is added to
  the RL reward, computed via finite differences between consecutive policy
  steps:
  `−w_coh · (||qdot − Δq/Δt||² + ||qddot − Δqdot/Δt||²)`.
  The weight `w_coh` is open (see "Reward design").
- **RL algorithm**: `PPO_SNN` adapted from skrl's PPO_RNN now lives in
  `prosthesis_snn.training`. The SNN membrane potential is treated as
  recurrent state, carried across steps inside an episode and zeroed at
  episode termination.
- **Training/inference separation**: the SNN backbone in
  `prosthesis_snn/model.py` is a plain `nn.Module` exposing membrane state
  via an explicit API. The skrl-compatible policy/value wrapper must stay in
  the training package when added, and should share the same backbone
  `state_dict` for inference and training.
- **Actor-critic topology**: PPO will use a shared actor-critic backbone. The
  skrl wrapper should run one shared SNN feature extractor/membrane state and
  branch into separate policy and value heads.

## Open implementation TODO

- **Create dedicated training and inference scripts** (production-grade, not
  smokes). Today only smokes exist: training `prosthesis_snn.training.cmc_ppo_smoke`
  (a 2-step manual loop) and inference `examples/smoke_inference.py`. Needed:
  - a real **training script**: long run driven by a trainer, many timesteps,
    logging, evaluation, stop criteria, and a stable checkpoint layout
    (elaborated in the next bullet);
  - a standalone **inference script**: load an inference checkpoint via
    `ReferenceGenerator.from_checkpoint(...)` and drive/expose the simulator
    reference-provider contract
    `q_ref, qdot_ref, qddot_ref = provider.get(t, state=None)`.
- Extend the implemented generic CMC-like PPO/SNN smoke entrypoint
  (`prosthesis_snn.training.cmc_ppo_smoke`) into a long-run training entrypoint
  with tuned experiment configs, logging policy, and production checkpoint
  layout.
- Resolve the action/output contract mismatch between the current trajectory
  env action (`policy_knots x 2` q-reference knots) and the intended SNN
  inference contract (`q/qdot/qddot x knee/ankle`). The smoke uses
  `policy_knots=3` so both are six values, but the semantics are not the same.
- Define and validate input/output normalization: observation feature names,
  units, scaling, output transform, action limits, and checkpoint metadata.
- Discuss and choose the training technique: pure RL, hybrid supervised
  learning + RL, adaptive RL that first imitates and then optimizes
  autonomously, RL from verifiable simulator results, or a similar staged
  strategy.
- Decide how to define the observation space: full current simulator state,
  only signals realistically available from prosthesis sensors (for example
  knee/ankle IMUs plus force-plate/foot-load estimates for prosthetic-side
  GRF), or a hybrid between privileged simulator state and deployable sensor
  signals.
- Evaluate whether the observation space should include the full SEA and
  controller state, including motor angle/speed, spring torque, command
  history, controller errors, filters, integrators, and saturation flags.
- Tune and validate the reward used by the trajectory environment, including
  tracking, smoothness, effort, soft `q/qdot/qddot` coherence, safety terms,
  and truncation thresholds.
- Compare velocity-reference output strategies: generate `q_ref` only and
  derive `qdot_ref` from it, versus generating `q_ref` and `qdot_ref`
  directly. Evaluate tracking stability, smoothness, derivative noise,
  coherence penalties, and how `qddot_ref` should be produced in each option.
- Promote the end-to-end training smoke into CI-style validation once runtime
  cost and required OpenSim/plugin assets are acceptable.

## Target Project

- Confirm the current, up-to-date CMC-like simulator path.
- The local reference structure is `CMC-like-Simulator - Claude`, but it is
  known to be dated and should not be modified yet.
- Confirm whether integration should happen in the simulator repo or remain as
  an external package imported by the simulator.

## Runtime Input Features

Not decided yet.

Candidate feature sets:

- Time or gait phase only: `phase_sin`, `phase_cos`.
- Current simulator state: prosthetic q/qdot, biological q/qdot, tracking
  errors, SEA motor angle/speed.
- Full SEA/controller state: motor angle/speed, spring torque, normalized
  command `u`, controller errors, filters, integrator states, saturation
  flags, and recent command/reference history.
- Hybrid configurable feature list loaded from YAML.

Decision needed:

- Exact feature names.
- Units and normalization.
- Whether features are available before or after `model.realizeVelocity(state)`.
- Whether the SNN hidden state resets at gait-cycle boundaries, simulation
  start only, or explicit events.

## Training Strategy

**Decision (partial)**: hybrid strategy. RL in OpenSim using the CMC-like
simulator as environment via `PPO_SNN`. An optional supervised warm-start
phase on healthy IK reference trajectories may precede the RL phase to
improve sample efficiency. The SNN predicts (q, qdot, qddot) for the
prosthetic SEA joints.

Still open:

- Whether the warm-start supervised phase is included in the first iteration
  of the project or skipped initially and added later if RL alone struggles.
- Source of the warm-start reference: most likely the existing IK splines
  from the healthy gait dataset, but the exact CSV/STO and the time window
  remain TBD.
- See **Reward design** for the RL signal structure.

## Dataset And Windows

Decision needed:

- Dataset path(s).
- Valid time windows with coherent kinematics and GRF.
- Sampling period for training and inference.
- Filtering policy for labels.
- Train/validation/test split.

Known context from prior simulator work:

- The current simulator uses `.sto` kinematic references.
- Valid force-plate windows can be shorter than the full IK window.
- The first SNN scope is prosthetic-only:
  - `pros_knee_angle`
  - `pros_ankle_angle`

## Normalization And Output Contract

Decision needed:

- Input normalization ranges.
- Per-(coord, channel) output scaling and offset values. The scaffold accepts
  nested mappings in YAML (`{coord: {channel: value}}`).
- Whether outputs are raw radians/rad·s⁻¹/rad·s⁻² or normalized values.

Current scaffold default:

- Inputs: `phase_sin`, `phase_cos` (placeholder; see "Runtime Input Features"
  for the open decision on the final feature set).
- Outputs: 6 channels = (q, qdot, qddot) × (knee, ankle).
- qdot and qddot are direct network outputs. Coherence with q is enforced
  *softly* via the RL reward, not via post-processing inside the provider.

## Checkpoint Format

Decision needed:

- Final checkpoint schema.
- Required metadata fields.
- Compatibility with future training scripts.

Current loader accepts:

- A bare PyTorch `state_dict`.
- A dict containing `model_state_dict`, `state_dict`, or `model`.
- Optional `config`, `output_scale`, `output_offset`, and `output_transform`.

Current training export path:

- `prosthesis_snn.training.save_reference_checkpoint(...)` exports an
  inference-only checkpoint from `ProsthesisSNNActorCritic` or
  `ProsthesisReferenceSNN`.
- The exported checkpoint intentionally excludes value head weights, PPO
  `log_std_parameter`, optimizer state, and skrl bookkeeping.
- The exported checkpoint is loadable with
  `ReferenceGenerator.from_checkpoint(...)`.

## Acceptance Criteria

Before real simulator integration:

- CPU inference works on Windows x86_64.
- CUDA inference works on Windows/NVIDIA when available, with CPU fallback.
- Hybrid provider preserves all biological references from the IK provider.
- Prosthetic references override only `pros_knee_angle` and
  `pros_ankle_angle`.

Deferred (not blocking the current iteration):

- macOS arm64 / MPS support.
- Importability of the core package without OpenSim, Isaac Lab, or skrl
  installed remains preserved. The optional training package may require skrl.

## Policy timing

Decision needed:

- SNN policy update frequency. Candidate default: 100 Hz, matching the
  simulator `step_size = 0.01 s` used in upstream OpenSim repos.
- Whether the policy runs at the same rate as the simulator integrator or
  at a lower rate with zero-order-hold (or linear interpolation) between
  updates.

## Reward design

Decision needed:

- Components and weights of the RL reward. Candidate structure (signs
  indicate intent, all weights `w_i > 0`):
  - `+w1 · v_forward` — pelvis forward velocity, clipped above target.
  - `−w2 · ||pelvis_orientation||²` — penalty on pelvis tilt/list/rotation.
  - `−w3 · ||qddot_ref||²` — smoothness of the generated reference.
  - `−w4 · E_sea` — actuator energy (`actuator_force * actuator_speed`).
  - `−w5 · (||qdot − Δq/Δt||² + ||qddot − Δqdot/Δt||²)` — soft coherence
    penalty between the three predicted channels (see "Resolved
    decisions").
  - `−w6 · sat(u)` — penalty on normalized SEA command saturation, for example
    sustained `|u|` near `1` or explicit saturation flags from the controller.
  - `+r_alive` — small per-step survival bonus.
- Optional imitation residual term, active only during the warm-start phase
  and annealed to zero before pure RL.

## Truncation criteria

Decision needed:

- Postural truncation thresholds. Candidate baseline (from upstream repos):
  `pelvis_tx < −0.10`, `pelvis_ty < 0.60`, `|pelvis_tz| > 0.40`.
- Additional caps on `knee_angle` / `ankle_angle` outside the physiological
  range.
- Optional cap on `||qddot_ref||` magnitude to avoid pathological references
  that the CMC-like controller cannot realistically track.

## BPTT length (skrl `sequence_length`)

Decision needed:

- Value of `sequence_length` used by `PPO_SNN` during the update path.
- `1` means membrane-only carry, no temporal credit assignment through past
  steps. Cheapest but may underfit gait dynamics.
- Suggested starting value: `16` or `32`, capturing ~150–300 ms of gait
  structure at 100 Hz.
- Whether the value should be progressively increased during training or
  kept fixed.
