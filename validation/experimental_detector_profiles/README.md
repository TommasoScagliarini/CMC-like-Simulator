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

## V19 force-free binary geometry

`two_point_binary_v19_outsole_25mm.json` is a development-only geometry
candidate dated 2026-08-04. It is not another online-GRF profile: it contains
exactly two body-fixed stations and one plane, with no spheres, radii,
materials, force law, thresholds, dwell, or event semantics. The public
runtime stream is `binary_phase_sensor_samples` and contains only timestamp,
heel contact, and toe contact. It is intentionally not connected to the
existing phase FSM; no HS/TO or training claim follows from this profile.

The two stations preserve the V17 heel/toe x/z placement. Their local y is the
corresponding plantar mesh projection plus one common 25 mm virtual outsole
reach. A point placed directly on the display mesh was rejected before event
work because the toe never reached the detector plane on development replays.
The 25 mm choice used raw bit continuity only; no HS/TO oracle or FSM score was
used.

## V21 geometry sweep

`validation/sweep_binary_phase_detector_v21_geometry.py` performs the
development-only coarse-to-fine search over independent heel/toe longitudinal
position and mesh-relative reach. The binary plane, 1 ms samples, and V20 FSM
remain frozen. Only DEV02/04 can be opened; there is no trial-selection CLI.

The command requires an explicit execution flag and a new run directory:

```text
python validation/sweep_binary_phase_detector_v21_geometry.py \
  --execute \
  --output-dir validation/binary_phase_detector_v21_runs/<unique-run-name>
```

Progress is written to stderr with an ASCII bar, percentage, elapsed time, and
ETA. Results and receipts are strict JSON/JSONL, atomically published, and
no-clobber. A candidate remains diagnostic unless both raw channels are stable
and distinct in every speed plateau and the final scalar/batch FSM gates pass.
The script does not edit the active detector profile or start training.
