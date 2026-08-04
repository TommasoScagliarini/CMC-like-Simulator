# Experimental morphology configurations

`morphology_completed_segment_shadow.json` selects the retrospective
completed-segment corridor without replacing either existing phase mode.

The shadow configuration is deliberately non-effectful:

- `morphology_weight = 0.0`;
- `morphology_hard_termination_enabled = 0.0`;
- `morphology_experimental_allow_effects = 0.0`, so accidental activation
  fails during environment construction;
- the actor observation, policy action, served reference, simulator and SEA
  command path are unchanged;
- only completed `[HS, TO)` and `[TO, next_HS)` segments receive an exact
  retrospective morphology phase and diagnostic loss.

The Gym wrapper can emit a completed segment's accumulated loss at its closing
event, but it cannot rewrite rewards from rollout fragments that PPO has
already consumed.  Positive-weight training therefore remains unsupported in
the current fixed-step sampling protocol.  It requires a separately validated
complete-episode reward-rewrite protocol before this configuration can be
promoted out of `experimental_configs/`.

The optional hard-violation hook is also provisional.  It currently checks
phase-independent global served-coordinate bounds only after a segment closes;
it is not yet the final, immediate outer morphology-corridor termination
described by the intended three-zone design.  The shipped shadow configuration
keeps this hook disabled.

Synthetic tests may set `morphology_experimental_allow_effects = 1` to exercise
the sparse settlement and delayed hard-violation hooks.  That switch is not an
authorization to use them in PPO training.

## Delayed causal candidate

`morphology_event_anchored_causal_candidate.yaml` is a separate preflight-only
override for the two-sensor high-rate event contract.  It does not replace
`training_exnovo_cfg.yaml` and explicitly forbids PPO updates.

Its morphology samples are the knee/ankle references actually served at each
policy step.  The isolated `CausalDelayedMorphologyBuffer` holds them for
`0.04 s`, anchors them to the physical `event_time_s`, and emits only samples
whose detector transition is already confirmed and delivered.  Observation,
detector pulses, policy actions and simulator state remain on their original
cadence.  At episode end, resolved samples are flushed; samples at or after a
still-pending detector onset are dropped and counted.
