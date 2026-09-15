# Deferred feasibility39 package

This bundle prepares phase 5 without creating the runtime variant or starting
training. `build_variant.py` copies the current V1 runtime Python/config files to
the sibling `unified_MLP_39`, applies checked transformations only to that copy,
and records source/payload/generated SHA256 hashes. Existing destinations and
unexpected source changes are rejected. V1 runs, tests and caches are excluded.

Invoke only after the authorized final V1 evaluation:

```python
from unified_MLP.validation.feasibility39_bundle.build_variant import materialize_variant
materialize_variant()  # or an explicit destination ending in unified_MLP_39
```

The CLI is `python build_variant.py --destination <path>/unified_MLP_39`.
Importing this bundle has no side effects.

The 39 actor inputs are the original 35 V1 channels followed by current-time
prescribed-left knee q, knee qdot, ankle q, ankle qdot. Their fixed normalization
scales are 1 rad, 10 rad/s, 1 rad, 10 rad/s. The 49 original privileged channels
follow, giving 88 total inputs. Both historical clock slots remain zero. There
is no availability input, running normalization, auxiliary target loss or raw
teacher duplicate in the critic. The privileged served-reference channels in
V1 remain unchanged: they use `self.kin`, not the substituted left teacher.

The versioned `unified_contract.json` contains target order, units, fixed scales,
zero-after-normalization behavior and mask scope. Existing V1 checkpoints cannot
be loaded into this schema. B and C transfers use identical 39D actor shapes.

Configurations use this section; the same names also round-trip through saved
resolved YAML:

```yaml
target_availability:
  target_availability_probability: 1.0
  target_curriculum_probabilities: [1.0, 0.75, 0.5, 0.25, 0.0]
  target_curriculum_block_iterations: 10
  target_mask_seed: 123
```

For A use probability 1 and an empty curriculum list. B uses probability 0 and an
empty list. C uses the schedule above. Ex-novo uses probability 0 and an empty
list and forcibly closes targets in both environment and RLModule, including
critic warm-up. Explicit rollout `--target-availability-probability 0` overrides
visibility even for A; rollout does not advance the training curriculum. CLI
also exposes the curriculum list, block size and mask seed with hyphenated names.

Each environment samples availability once at episode initialization and places
the resulting masked values in the collected observation. PPO forwards are
deterministic with respect to that stored observation and do not resample masks.
Before each iteration, the runtime broadcasts the next-episode probability to
all configured EnvRunners and audits coverage. Ongoing episodes retain their
collected availability. The phase-5 plan runs **all A/B/C in matching complete
resume blocks ending at iterations 10, 20, 30, 40, 50** so C's final block starts
new episodes at probability zero without altering existing PPO samples.

`unified_training_state.json` preserves the fixed schedule, logical iteration,
current probability, per-worker mask RNG states and episode counters. Resume
rejects changed schedules/seeds, missing workers or changed vector widths. Ray
2.55's native EnvRunner checkpoint does not include environment RNG state, so
this metadata restores the mask RNG explicitly before the first collection.
The first RLlib seeded reset retains that restored stream. A new process starts
a new physical episode and draws its mask using the current curriculum level;
this is **not exact continuation of the interrupted OpenSim episode**.

The copied rollout trace adds `policy_action_std` beside `policy_action_mean`
and records target availability. Paired `stochastic_held` runs with hold 1 and
the same seed can compare `exploration_action_noise / policy_action_std` over
their common prefix to audit shared standardized Gaussian draws.

Actual completed environment steps and visible steps are counted independently
of the nominal episode probability. Per-worker counter differences appear in
each row's `unified_phase_audit.target_availability_batch` and in checkpoint
`unified_training_state.target_availability.last_batch`, including total steps,
visible steps/fraction, worker/vector breakdown and comparison to the requested
training batch size. The batch-size comparison is recorded, not asserted;
negative deltas, incomplete workers and changed vector topology fail closed.

Tests materialize only into temporary pytest directories. They check source
integrity, schema/normalization, unchanged reward teacher, visible positive
control, collected-observation/PPO probability ratio invariance, zero ex-novo
teacher gradients, paired actual RLlib PPO losses/Adam updates, curriculum/RNG
resume and reload into a fresh Python process. These tests do not establish
live 39D OpenSim/Ray execution or walking quality; those checks and the A/B/C
campaign belong to phase 5 after materialization. Windows live execution remains
unverified; this bundle uses portable Python paths and no platform-specific
process launch mechanism.
