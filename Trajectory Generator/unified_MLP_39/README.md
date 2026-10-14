# Unified MLP 39D — prescribed gait-phase targets

The default remains the LEFT contract described below. An explicit
`target_availability.target_reference: right_v26` (CLI `--target-reference
right_v26`) selects the historical RIGHT V26 template: knee shift 0.465,
ankle shift 0.452 and an **unshifted RIGHT** actor sin/cos clock. Both references
use the same 39/88 layout and six-channel episode mask. The target source,
clock and shifts are persisted and validated; LEFT and RIGHT checkpoints are
not interchangeable. Existing LEFT contracts and mask state remain compatible.

`configs/v26_aligned_right_phase_fsmv3_40iter_2026-09-23.yaml` is the fresh
RIGHT comparison: seed 123, 4096 samples, 512 minibatch, 10 PPO epochs, 13
workers, 5 s episodes from 12.99 s, FSM v3, all prescribed channels visible,
velocity divisor 1 and slew disabled. It does not initialize from V26 weights.
The registered smoke checks and launch receipts are under
`validation/right_reference_training_2026-09-23/` and
`runs/plans/right_reference_training_2026-09-23/`.

The current contract is `unified_mlp39_v3`. The four target channels contain
left knee/ankle position and velocity from a periodic left-leg template indexed
by the **prescribed left heel-strike clock**, with **zero phase shift**. This
replaces the original direct lookup of left kinematics at simulation time.

The template reuses V26's cycle resampling, averaging and periodic cubic spline.
It uses complete cycles within the IK domain that intersect the setup window;
the inherited algorithm uses all available cycles when none intersect and
filters periods more than 25% from the median. Velocity is the template phase
derivative divided by the local prescribed cycle period. Before/after the
recorded heel-strike boundaries the clock extrapolates the nearest period,
within the permitted IK time domain. Missing left events or either coordinate
is an error; no right-leg or direct-time fallback is used.

The actor's two historical clock slots now contain `sin(2*pi*left_phase)` and
`cos(2*pi*left_phase)`. They use the same prescribed LEFT clock as the four
targets, with zero shift. The shared simulator's right-leg clock remains disabled
(`gait_clock_enable: false`); the online prosthetic FSM is unchanged.
Actor/full dimensions remain 39/88, and the same teacher
provides observations and imitation loss targets. Loss normalization is measured
from this phase template. One availability draw per episode masks all six
prescribed actor inputs together: slots 0, 1, 35, 36, 37 and 38. The imitation
reward still evaluates the teacher when these inputs are hidden. By default, in ex-novo,
all six inputs are forced to zero in both actor and critic, including a module
defense against callers bypassing the environment mask. Imitation terms have no
active ex-novo reward effect.
The prescribed clock is derived from recorded GRF, not the prosthesis's online
events; masking does not guarantee autonomous synchronization.

## Explicit objective-transfer curriculum

`target_availability.target_mask_mode: episode` (CLI `--target-mask-mode episode`)
separates the episode mask from the reward objective. Both objectives then use
the configured availability/curriculum, and the module preserves the mask
already recorded in the observation batch. The mode is serialized in the module
configuration and RNG metadata; rollout rejects a module/environment mismatch,
and resume cannot change the mode. Omit this option to retain the historical
mandatory ex-novo mask. Probability zero still hides all six channels.

For a controlled comparison, `model.actor_transfer_warmup: true` allows
`--init-actor-from` with either reward. It requires episode masking and fixed,
frozen exploration. Both objectives receive fresh critic/optimizer state, one
critic-only warm-up iteration, then the selected objective with sigma frozen.
Ordinary resume preserves the completed warm-up and the exploration protocol.

`reward.morphology_termination_mode: ex_novo` explicitly applies the same
morphology contract failure/hard-limit termination rules to both objectives;
it does not activate morphology penalties in the imitation scalar. Its default
`objective` preserves historical behavior. Common FSM settings still need to
be supplied identically when comparing objectives. With a per-minibatch KL
guard and single-start sampling, the KL-only learner records the audit without
adding the three-start batch connector.

The CP90 comparison and its bounded progression gates are documented in
[the experiment protocol](validation/reward_curriculum_2026_10_01/PROTOCOL.md).
Changing objective or masking does not itself certify recovered gait quality.

Use `configs/pretraining_visible_targets.yaml` for the first comparison:
availability 1.0 and an empty curriculum keep all six prescribed inputs visible. Reward
weights, physical reset, online FSM and SEA mechanics are preserved. This preset
inherits the existing budget; it is not a preregistered statistical parity study.

`configs/smoke_pretraining_visible_targets.yaml` uses 192 transitions, three
workers and one PPO epoch. A bounded execution and full checkpoint resume are:

```text
python "Trajectory Generator/unified_MLP_39/train_ppo_mlp.py" --config "Trajectory Generator/unified_MLP_39/configs/smoke_pretraining_visible_targets.yaml" --output-dir <new-smoke-name> --iterations 1
python "Trajectory Generator/unified_MLP_39/train_ppo_mlp.py" --config "Trajectory Generator/unified_MLP_39/configs/smoke_pretraining_visible_targets.yaml" --output-dir <new-smoke-name> --iterations 2 --resume-from "<absolute-run-path>/milestone_iteration_000001/checkpoint_last"
```

`--target-velocity-scale` (YAML: `target_availability.target_velocity_scale`)
sets a finite positive fixed divisor for both qdot observation channels. Default
10 preserves the previous velocity scaling; value 1 exposes rad/s directly for
the ablation. Position and sin/cos scales are 1. This changes observations only:
the phase teacher, loss normalization and reward remain unchanged.

Checkpoint metadata persists the six-channel contract, fixed velocity scale,
curriculum and per-worker RNG. Resume, actor initialization and rollout reject
different velocity scales. Old `unified_mlp39_v1` and `unified_mlp39_v2`
checkpoints are rejected because their target/clock semantics differ. New v3
experiments start fresh; no automatic conversion of historical checkpoints is
performed. Saved mask RNG resumes; the OpenSim episode restarts. Historical runs
and the original materialization receipt remain historical artifacts. The
deferred V1 builder is a legacy temporal-target bundle, not the source of this
new phase-based variant.

The worker fix unwraps the Gymnasium vector wrapper before enumerating the
individual environments, and checks their availability controllers and vector
width. This works with RLlib's `DictInfoToList(SyncVectorEnv(...))` layout.

Tests use the installed variant (run separately from legacy bundle tests, which
import a temporary package under the same module name):

```text
python -m pytest -q "Trajectory Generator/unified_MLP_39/tests"
python "Trajectory Generator/unified_MLP_39/validation/verify_left_phase_runtime.py" --output <new-receipt.json>
python "Trajectory Generator/unified_MLP_39/validation/verify_left_phase_runtime.py" --target-velocity-scale 1 --output <new-scale1-receipt.json>
```

The implementation uses portable Python paths and shared simulation APIs.
Live validation is performed on macOS; Windows execution still requires the
matching SEA and OnlineGRF DLLs and a separate live check.
