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
override for the historical two-sensor high-rate event contract. It does not
replace `training_exnovo_cfg.yaml` and explicitly forbids PPO updates.

Its morphology samples are the knee/ankle references actually served at each
policy step. The isolated `CausalDelayedMorphologyBuffer` holds them for
`0.04 s`, anchors them to the physical `event_time_s`, and emits only samples
whose detector transition is already confirmed and delivered. Observation,
detector pulses, policy actions and simulator state remain on their original
cadence. At episode end, resolved samples are flushed; samples at or after a
still-pending detector onset are dropped and counted.

## Additive V26 readiness candidate

`morphology_event_anchored_causal_v26_candidate.yaml` is the separate additive
contract for frozen V25 geometry plus the heel-qualified V26 event contract.
It keeps the historical candidate above unchanged, does not replace
`training_exnovo_cfg.yaml`, and forbids PPO updates and qualifying rollouts
while Q2 is open.

Its morphology samples are the knee/ankle references actually served at each
policy step. The reward wrapper reads accepted anchors only from
`phase_fsm.accepted_transitions_this_step`, while the frozen V26 payload
provides the pending debounce candidate, confirmed detector events, and the
cancellation journal. The `CausalDelayedMorphologyBuffer` holds samples for
`0.04 s`, anchors them to physical `event_time_s`, and never treats a raw edge,
rejected actor transition, or analog GRF as a morphology event.

Cancellation releases the affected buffered tail without creating an anchor.
`WAIT_HS` and reset-time partial stance remain morphology-unavailable; samples
that can no longer be covered by a causally deliverable first anchor are
dropped and counted. A valid partial-stance TO discards its unobservable stance
prefix and opens swing. Actor timeout forces the terminal flush. At any other
episode end, samples before a still-pending onset are resolved from the last
accepted anchor, while samples at/after that onset are dropped. Observation,
detector pulses, policy actions, served references, and simulator state remain
on their original cadence.

The shipped candidate keeps `morphology_weight = 0.0` and
`morphology_causal_allow_effects = 0.0`. The scalar reward has an explicit
weight-zero branch, so it does not evaluate `0 * loss` and cannot be poisoned
by a non-finite diagnostic loss. Positive weights (`0.0025`, `0.005`) remain a
future Q2-gated A/B; this readiness configuration does not authorize or execute
them.
