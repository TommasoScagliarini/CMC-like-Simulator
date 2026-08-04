# Event-warped morphology profile validation

Overall gate: **PASS**.

The held-out split uses contiguous chronological blocks; each fold fits both the corridor and the canonical TO phase only on its training blocks.

## Blocked held-out coverage

| Fold | Held cycles | Period mean [s] | Duty mean | Knee | Ankle |
|---:|---:|---:|---:|---:|---:|
| 1 | 25 | 1.502 | 0.689 | 98.55% | 86.06% |
| 2 | 25 | 1.012 | 0.597 | 100.00% | 100.00% |
| 3 | 25 | 0.928 | 0.599 | 100.00% | 98.69% |
| 4 | 24 | 0.960 | 0.600 | 100.00% | 99.20% |
| 5 | 24 | 1.238 | 0.626 | 100.00% | 100.00% |

## Aggregate

- knee: coverage 99.706%; stance 99.774%; swing 99.594%; outside p95 1.706 deg; mean std reduction 17.77%.
- ankle: coverage 96.744%; stance 97.374%; swing 95.700%; outside p95 8.339 deg; mean std reduction 20.58%.

## Gates

| Gate | Result |
|---|---:|
| `profile_rebuild_byte_exact` | PASS |
| `event_profile_contract` | PASS |
| `knee_aggregate_coverage` | PASS |
| `knee_worst_fold_coverage` | PASS |
| `knee_outside_excursion_p95` | PASS |
| `knee_dispersion_reduced` | PASS |
| `ankle_aggregate_coverage` | PASS |
| `ankle_worst_fold_coverage` | PASS |
| `ankle_outside_excursion_p95` | PASS |
| `ankle_dispersion_reduced` | PASS |
| `all` | PASS |

## Figures

- `profile_dispersion_comparison.png`
- `blocked_cross_validation_coverage.png`
