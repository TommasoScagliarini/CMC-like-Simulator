# Validation of event-anchored morphology

This is an offline audit. The retrospective oracle uses the next HS and must not be used by a deployed policy or online reward.

## Inputs

| Input | Path | SHA-256 |
|---|---|---|
| trace | `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/runs/rollout/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded/rollout_policy_trace.json` | `f342232d8230c0f097c34bc634d11241060c09148f62b13079b68580f2aa964b` |
| summary | `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/runs/rollout/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded/rollout_summary.json` | `085d46cede965e4981db617f7630eb47a4355a31caf2463a621a9df654d2ae72` |
| legacy_profile | `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json` | `6307119c2bb233831eac7070726fc6e2e0175c8f715a9fbbb787c0763b46fab7` |
| event_warped_profile | `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json` | `33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55` |

## Phase contracts

Fixed event anchor: alpha = **0.622329999** at valid prosthetic TO.

- Legacy: phase recorded in the original trace and legacy profile.
- Causal event-anchored: clipped stance/swing progress; nominal timing before the first complete cycle, then past-only robust medians reconstructed with the runtime window.
- Retrospective oracle: exact durations of complete valid FSM HS-TO-HS cycles and the event-warped profile.

Recovered events: 4 HS, 3 TO, 3 complete cycles.

## Coverage and corridor loss

| Scheme | Coverage | Both inside | Loss sum | Loss mean | Loss p95 | Knee max excursion [rad] | Ankle max excursion [rad] |
|---|---:|---:|---:|---:|---:|---:|---:|
| Legacy logged phase | 99.0% | 51.7% | 78.4999 | 0.158586 | 0.931328 | 0.567125 | 0.39305 |
| Event-anchored causal | 99.0% | 64.6% | 41.9689 | 0.0847857 | 0.561858 | 0.513339 | 0.435571 |
| FSM oracle retrospective | 91.2% | 64.5% | 30.9892 | 0.0679589 | 0.496028 | 0.522695 | 0.256905 |

Common support: **451 / 500** samples (90.2%).

## Event continuity

| Scheme | Comparable events | Max circular phase jump | Max corridor-bound jump [rad] | Max same-value joint-loss jump |
|---|---:|---:|---:|---:|
| Legacy logged phase | 6 | 0.191948 | 0.716188 | 1.28625 |
| Event-anchored causal | 6 | 0.167784 | 0.449086 | 0.398218 |
| FSM oracle retrospective | 6 | 0.0130322 | 0.0827263 | 0.069769 |

## Shadow reward

Unavailable morphology samples contribute zero, matching the runtime contract. Penalties on common support are also reported in JSON.

| Scheme | Weight | Cumulative penalty | Shadow return | Delta | Max step penalty |
|---|---:|---:|---:|---:|---:|
| Legacy logged phase | 0.0025 | 0.19625 | 52.2307 | -0.19625 | 0.00371222 |
| Legacy logged phase | 0.005 | 0.3925 | 52.0344 | -0.3925 | 0.00742445 |
| Legacy logged phase | 0.01 | 0.784999 | 51.6419 | -0.784999 | 0.0148489 |
| Event-anchored causal | 0.0025 | 0.104922 | 52.322 | -0.104922 | 0.00272657 |
| Event-anchored causal | 0.005 | 0.209845 | 52.2171 | -0.209845 | 0.00545313 |
| Event-anchored causal | 0.01 | 0.419689 | 52.0073 | -0.419689 | 0.0109063 |
| FSM oracle retrospective | 0.0025 | 0.0774731 | 52.3495 | -0.0774731 | 0.00282857 |
| FSM oracle retrospective | 0.005 | 0.154946 | 52.272 | -0.154946 | 0.00565713 |
| FSM oracle retrospective | 0.01 | 0.309892 | 52.117 | -0.309892 | 0.0113143 |

## Validation gates

| Gate | Result | Max/absolute difference |
|---|---:|---:|
| `rollout_morphology_weight_is_zero` | PASS | 0 |
| `runtime_reward_reconstruction_exact` | PASS | 0 |
| `original_weight_zero_per_step_exact` | PASS | 0 |
| `legacy_weight_zero_per_step_exact` | PASS | 0 |
| `event_anchored_causal_weight_zero_per_step_exact` | PASS | 0 |
| `oracle_retrospective_weight_zero_per_step_exact` | PASS | 0 |
| `all_weight_zero_gates` | PASS | n/a |
| `event_anchored_to_never_regresses` | PASS | 0.00672276505122 |
| `causal_phase_p95_improves_vs_legacy` | PASS | 0.0927327308556 |
| `causal_common_support_loss_reduced` | PASS | 0.481749520082 |
| `causal_event_bound_jump_improves_vs_legacy` | PASS | 0.449085500537 |
| `common_support_is_sufficient` | PASS | 0.902 |
| `all_mapping_gates` | PASS | n/a |
| `all_gates` | PASS | n/a |

Overall gate: **PASS**.

## Figures

- `phase_alignment.png`: causal phase and oracle error over simulation time.
- `corridor_coverage.png`: served knee/ankle references against each evaluated corridor.
