# Experimental two-sensor detector profiles

The canonical development baseline is resolved through
`two_sensor_development_baseline_registry.json`.

As of 2026-07-22, the active development comparator is the V13 profile derived
from the frozen V12 primary geometry. V9 remains immutable as the historical
comparator.

`ACTIVE_DEVELOPMENT_BASELINE` means that new offline detector experiments use
this geometry as their starting point and comparator. It does not mean that the
profile passed sealed validation or that it may be selected by runtime,
training, policy inference, or active closed-loop simulations. Those uses stay
disabled until a new preregistered candidate passes an untouched holdout.

The V13 sealed block is consumed. Its data may be inspected to understand a
failure, but must never be used to rank, tune, or rescue a later candidate.
