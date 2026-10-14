# Reward nei primi 25 aggiornamenti LEFT/RIGHT

Lettura offline dei TensorBoard esistenti: stessi 4096 step per iterazione, confronto fino a 102.400 transizioni, prima del problema di ripristino LEFT.

| Braccio | Iterazioni | Imitazione ponderata | Penalità base | Command-rate | Clip lift | Base dopo clip | Completi |
|---|---|---:|---:|---:|---:|---:|---:|
| left | 1–5 | 0.2659 | 0.1901 | 0.1871 | 0.0491 | 0.1249 | 2/119 |
| left | 6–10 | 0.2958 | 0.1862 | 0.1846 | 0.0287 | 0.1383 | 3/81 |
| left | 11–15 | 0.3986 | 0.1836 | 0.1812 | 0.0125 | 0.2275 | 4/72 |
| left | 16–20 | 0.4992 | 0.1813 | 0.1794 | 0.0033 | 0.3213 | 10/66 |
| left | 21–25 | 0.5716 | 0.1808 | 0.1794 | 0.0011 | 0.3920 | 19/51 |
| right | 1–5 | 0.1794 | 0.1898 | 0.1870 | 0.0806 | 0.0703 | 1/112 |
| right | 6–10 | 0.2276 | 0.1854 | 0.1842 | 0.0495 | 0.0917 | 7/71 |
| right | 11–15 | 0.3080 | 0.1803 | 0.1794 | 0.0259 | 0.1536 | 13/60 |
| right | 16–20 | 0.3868 | 0.1804 | 0.1794 | 0.0112 | 0.2176 | 26/50 |
| right | 21–25 | 0.4860 | 0.1781 | 0.1772 | 0.0044 | 0.3124 | 32/42 |

## Metodo e limiti

- Saved TensorBoard scalars and native training JSONL, iterations1-25, before LEFT optimizer restore defect.
- Each update collects4096 transitions in both arms; compare at identical cumulative steps.
- RewardComponentsCallback logs each finite component from each sampled step with reduce=mean, window=100; RLlib merges runner metrics and TensorBoard records float32 scalars. These are smoothed runner aggregates, not exact means over4096 transitions.
- mean(base_clipped) - [0.2 mean(actual_score) +0.8 mean(served_score) -mean(base_penalty)]. Linear recombination preserves a shared averaging operator; it does NOT apply clip to the mean raw base.
- With nonnegative scores/penalties, blends summing to1 and clip bounds[0,1], upper clipping cannot activate. Positive clip lift measures weighted aggregate upward correction from negative raw bases. It does not reveal number, duration or locations of clipped steps.
- Mean reward equals mean base minus the sum of active post-clip terms within float32 precision. This supports aligned averaging of relevant component streams; raw early transition traces are unavailable.
- Five-iteration table entries are arithmetic means of the five logged smoothed values, not pooled step means or episode-return estimates.

## Conclusioni

- Early base clipping is real and materially changes aggregate scalar values in both arms. Mature-rollout clipping findings cannot be generalized to initial learning.
- RIGHT has larger initial clip lift than LEFT yet subsequently better completion; current evidence does not identify clipping as a selective LEFT failure mechanism.
- Initial command-rate cost is almost identical in both arms and dominates base penalty. Whether it blocks recovery corrections is not determined by these smoothed aggregates.
- LEFT has better imitation-score aggregates while worse completion later; target-specific normalized scores do not rank cross-task physical robustness.
