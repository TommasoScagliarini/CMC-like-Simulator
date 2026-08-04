"""Preregistered AB06 cross-speed V14 two-sensor geometry protocol.

V14 starts from the registry-bound V13 development baseline and uses the
explicitly frozen AB06 treadmill cross-speed split::

    DEVELOPMENT = 02, 04, 08
    VALIDATION  = 05 (one shot)
    SEALED      = 06 (one shot, only after validation PASS)
    RESERVE     = 03, 07 (never opened by this runner)

Each trial is replayed continuously at each cadence, with no FSM reset at a
plateau boundary.  A scored unit is one ``trial x plateau x cadence`` cell.
Reference events are derived separately inside each stable plateau; the
contact active at the left edge is excluded, only complete HS--TO--HS cycles
are retained, and closing cycles without the frozen right observation margin
are prefix-censored.  At least ten scoreable cycles must remain in every unit.

The golden converter schema-checks and archives dataset IK, but the detector
harness never consumes that converted trajectory downstream.  For every
authorized trial it converts markers to TRC and force plates to prescribed
GRF, resolves the extensionless SEA plugin per platform, and runs marker-based
IK with OpenSim 4.5.2 on the hash-pinned marker-calibrated model.  A final IK
receipt binds that provenance before replay.

The converter and path logic remain portable, but this exact frozen V14 run is
macOS-arm64-only: the repository currently has a preregistered hash only for
the ``ff`` dylib.  Windows execution requires a new protocol that first pins a
verified ``ff`` DLL; this harness never substitutes the non-ff DLL.

The module intentionally exposes :func:`expected_protocol_payload` and the
read-only ``--print-protocol-template`` CLI.  Template generation may read and
hash the metadata-audit JSON and source files, but it never semantically
decodes a marker, force, kinematic, gait-event, validation, sealed, or reserve
stream.  Execution requires the separate ``--execute`` flag and a fully
materialized frozen protocol.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import sys
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TOOLS_ROOT = REPO_ROOT / "tools"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for _path in (REPO_ROOT, VALIDATION_ROOT, TOOLS_ROOT, TRAJECTORY_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import audit_two_sensor_v14_trial_metadata as metadata  # noqa: E402
import convert_epic_ab06_tables as converter  # noqa: E402
import setup_io  # noqa: E402
import sweep_two_sensor_prescribed_thresholds as thresholds  # noqa: E402
import validate_two_sensor_sealed_v13 as v13  # noqa: E402


v12 = v13.v12
v11 = v13.v11
v10 = v13.v10
v6 = v13.v6
v1 = v13.v1
dual = v13.dual

SCHEMA_VERSION = 14
PROTOCOL_ID = "AB06_TWO_SENSOR_CROSS_SPEED_INTERPOLATION_2026-07-22_V14"
BASELINE_ID = "V13_BASELINE"
BASELINE_PROFILE_PATH = (
    "validation/experimental_detector_profiles/"
    "two_sensor_v13_development_toe_down_p0p75mm_heel_x_p3p5mm.json"
)
BASELINE_REGISTRY_PATH = (
    "validation/experimental_detector_profiles/"
    "two_sensor_development_baseline_registry.json"
)
V9_PROFILE_PATH = (
    "validation/experimental_detector_profiles/"
    "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
)
MARKER_MODEL_PATH = metadata.DOWNSTREAM_IK_CONTRACT[
    "marker_calibrated_model"
]["path"]
RESERVE_ACTUATORS_PATH = "models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml"
DEFAULT_METADATA_AUDIT = metadata.DEFAULT_OUTPUT
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_cross_speed_v14_protocol.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_runs/2026-07-22_ab06_cross_speed_v14"
)
DEFAULT_EXECUTION_LEDGER = (
    VALIDATION_ROOT / "two_sensor_cross_speed_v14_execution_ledger.json"
)
GOLDEN_CONVERSION_MANIFEST_PATH = (
    "validation/two_sensor_v14_preprocessing_golden/"
    "2026-07-22_treadmill_01_01/treadmill_01_01_conversion_manifest.json"
)
GOLDEN_IK_EXECUTION_RECEIPT_PATH = (
    "validation/two_sensor_v14_preprocessing_golden/"
    "2026-07-22_treadmill_01_01/treadmill_01_01_ik_execution_receipt.json"
)
GOLDEN_IK_RECEIPT_PATH = (
    "validation/two_sensor_v14_preprocessing_golden/"
    "2026-07-22_treadmill_01_01/treadmill_01_01_ik_receipt.json"
)
GOLDEN_V13_PARITY_PATH = (
    "validation/two_sensor_v14_preprocessing_golden/"
    "2026-07-22_treadmill_01_01/treadmill_01_01_v13_ik_parity.json"
)
GOLDEN_MARKER_IK_PATH = (
    "validation/two_sensor_v14_preprocessing_golden/"
    "2026-07-22_treadmill_01_01/treadmill_01_01_ik.mot"
)
V13_GOLDEN_IK_PATH = "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
EXPECTED_GOLDEN_IK_SHA256 = (
    "ce4a948fd8f01f34ff32e4680b8a082f2e44de155aed89b7e4a55d37016c3596"
)
LEFT_FOOT_MESH_PATH = "Geometry/AM_foot_l.stl"
EXPECTED_LEFT_FOOT_MESH_SHA256 = (
    "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9"
)
EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX = {
    ".dylib": "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b",
}

PRIMARY_DT_S = 0.010
FINE_DT_S = 0.001
CADENCES = (
    ("runtime_10ms", PRIMARY_DT_S),
    ("fine_1ms", FINE_DT_S),
)
MINIMUM_SCOREABLE_CYCLES_PER_PLATEAU = 10
NUMERIC_TOLERANCE = 1.0e-12
PARETO_TOLERANCE = 1.0e-12
LEFT_CONTEXT_S = 0.090
RIGHT_OBSERVATION_MARGIN_S = 0.060
EXPECTED_PRIMARY_LOAD_SPHERES = 8
EXPECTED_DEVELOPMENT_DETECTOR_STATIONS = 15
EXPECTED_DEVELOPMENT_TOTAL_STATIONS = 23

HEEL_X_ABSOLUTE_FROM_V9_MM = (3.50, 3.75, 4.00, 4.25, 4.50, 5.00, 5.50, 6.00)
HEEL_RADIUS_REDUCTION_FROM_V13_MM = (0.00, 0.05, 0.10, 0.15, 0.20, 0.25)
TOE_DOWN_ABSOLUTE_FROM_V9_MM = (0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35)
TOE_RADIUS_REDUCTION_FROM_V13_MM = (0.00, 0.05, 0.10, 0.15, 0.20, 0.25)
ARM_ORDER = ("Hx", "Hr", "Ty", "Tr")
ISOLATED_COUNT_BY_ARM = {"Hx": 7, "Hr": 5, "Ty": 6, "Tr": 5}
ISOLATED_SELECTABLE_COUNT = sum(ISOLATED_COUNT_BY_ARM.values())
ISOLATED_PAIR_COUNT = ISOLATED_SELECTABLE_COUNT + 1

ARM_GRIDS = {
    "Hx": HEEL_X_ABSOLUTE_FROM_V9_MM,
    "Hr": HEEL_RADIUS_REDUCTION_FROM_V13_MM,
    "Ty": TOE_DOWN_ABSOLUTE_FROM_V9_MM,
    "Tr": TOE_RADIUS_REDUCTION_FROM_V13_MM,
}
BASELINE_ARM_VALUES = {"Hx": 3.5, "Hr": 0.0, "Ty": 0.75, "Tr": 0.0}
OUTER_BOUNDARY_VALUES = {"Hx": 6.0, "Hr": 0.25, "Ty": 1.35, "Tr": 0.25}
MAXIMUM_STAGE2_CARTESIAN_TUPLES = 3 ** 4

OBJECTIVE = (
    "Improve the registry-bound V13 two-sensor development baseline across "
    "four treadmill speed strata by separately screening heel-x, heel-radius, "
    "toe-down and toe-radius reductions on trials 02/04/08, then evaluating "
    "the preregistered local Cartesian neighborhood around those diagnostic "
    "arm winners before opening trial 05 and trial 06 once each for only the "
    "frozen primary challenger paired with V13."
)

INTERPRETATION_LIMITS = [
    "The split is cross-speed interpolation, not leave-one-trial-out and not exchangeable repeated trials.",
    "Trial 01 and its consumed V13 interval are never used for V14 selection, thresholds, counts, or tuning.",
    "Trials 03 and 07 are reserve data and this runner has no semantic-access route for them.",
    "All gates are evaluated per trial x plateau x cadence; global trial metrics cannot be assigned to a plateau.",
    "Per-unit exact counts are derived from prescribed GRF only after the unit's role is authorized; they cannot alter grids or thresholds.",
    "Isolated arm winners are diagnostic root-safe components and need not pass the full V13 gate in isolation.",
    "Only members of the deterministic Stage-2 Cartesian pool, including reused single-arm rows, may become a development finalist; a finalist must pass every development unit and aggregate gate.",
    "Validation and sealed stages evaluate exactly the frozen primary plus V13 in one paired opening; there is no rescue or reselection.",
    "V13 wins every tie. Promotion requires Pareto non-inferiority of the preregistered minimax vector and at least one improvement beyond tolerance.",
    "A V14 PASS does not automatically edit a detector profile, registry, runtime configuration, policy, reward, or training configuration.",
    "The search is local and staged: it is neither full-factorial over the original grids nor evidence of a global four-dimensional optimum.",
    "One winner anchor and one deterministic adjacent neighbor per arm limit development multiple testing; validation and sealed stages protect against development adaptation.",
    "Trials 05 and 06 are sealed for event/trajectory outcomes, not metadata-blind: their condition speeds, file sizes and SHA-256 identities were already inspected.",
    "All trials are from the same subject and session with the same plateau order; no cross-subject, exchangeable-trial, chronological-holdout, independent-cycle, or statistical-independence claim is allowed.",
    "The 20 N prescribed-GRF reference and the 0.5/0.25 N Hunt-Crossley detector have different physical definitions; V14 tests operational geometric compensability, not a unique physical cause.",
    "This frozen V14 execution is macOS-arm64-only because only the ff dylib has a preregistered binary hash; the golden converter remains cross-platform, but V14 cannot run on Windows until a separate protocol pins the verified ff DLL.",
]


class ProtocolError(RuntimeError):
    """Raised when a frozen V14 contract is violated."""


class NoClobberError(ProtocolError):
    """Raised when a one-shot output path is already occupied."""


@dataclass(frozen=True)
class TrialArtifacts:
    trial_id: str
    stage: str
    setup: setup_io.SimulationSetup
    work_dir: Path
    trc: Path
    grf: Path
    external_loads: Path
    ik_setup: Path
    ik_motion: Path
    ik_receipt: Path
    conversion_manifest: Path


@dataclass
class SamplingPlan:
    sampler: Any
    station_by_key: dict[tuple[str, tuple[float, float, float]], Any]
    profiles: dict[str, Any]
    pairs: dict[str, dict[str, Any]]


@dataclass
class TrialCadenceBundle:
    trial_id: str
    cadence_label: str
    sample_dt_s: float
    plan: SamplingPlan
    samples: Mapping[str, Any]
    shared: dict[str, Any]
    plateau_references: tuple[dict[str, Any], ...]
    access: dict[str, Any]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _source_record(path: str | Path, *, require: bool = True) -> dict[str, Any]:
    resolved = v1.resolve_repo_path(path).resolve()
    if not resolved.is_file():
        if require:
            raise ProtocolError(f"missing V14 source: {_portable_path(resolved)}")
        return {
            "path": _portable_path(resolved),
            "sha256": "<FILL_AFTER_SOURCE_EXISTS>",
        }
    return {"path": _portable_path(resolved), "sha256": _sha256(resolved)}


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(
        v1._json_safe(dict(payload)), indent=2, sort_keys=True, allow_nan=False
    ) + "\n"
    try:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(serialized)
    except FileExistsError as exc:
        raise NoClobberError(f"refusing to overwrite {_portable_path(path)}") from exc
    return path


def _write_text_exclusive(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
    except FileExistsError as exc:
        raise NoClobberError(f"refusing to overwrite {_portable_path(path)}") from exc
    return path


def _token(value: float) -> str:
    return f"{float(value):.3f}".rstrip("0").rstrip(".").replace(".", "p")


def _fixed_gate_contract() -> dict[str, Any]:
    gate = dict(v13._sealed_gate_contract())
    for key in (
        "require_exact_reference_counts",
        "require_exact_detector_counts",
        "require_exact_valid_cycles",
        "require_exact_causal_swing_intervals",
    ):
        gate.pop(key, None)
    return gate


def _dynamic_gate(reference_hs: int, reference_to: int) -> dict[str, Any]:
    if reference_hs != reference_to + 1:
        raise ProtocolError("dynamic gate requires complete HS--TO--HS cycles")
    gate = _fixed_gate_contract()
    gate.update(
        {
            "require_exact_reference_counts": {
                "heel_strike": int(reference_hs),
                "toe_off": int(reference_to),
            },
            "require_exact_detector_counts": {
                "heel_strike": int(reference_hs),
                "toe_off": int(reference_to),
            },
            "require_exact_valid_cycles": int(reference_to),
            "require_exact_causal_swing_intervals": int(reference_to),
        }
    )
    return gate


def _expected_grid() -> dict[str, Any]:
    return {
        "baseline_id": BASELINE_ID,
        "baseline_is_unique_and_nonselectable": True,
        "heel_x_absolute_shift_from_v9_mm": list(HEEL_X_ABSOLUTE_FROM_V9_MM),
        "heel_radius_reduction_from_v13_mm": list(
            HEEL_RADIUS_REDUCTION_FROM_V13_MM
        ),
        "toe_down_absolute_from_v9_mm": list(TOE_DOWN_ABSOLUTE_FROM_V9_MM),
        "toe_radius_reduction_from_v13_mm": list(
            TOE_RADIUS_REDUCTION_FROM_V13_MM
        ),
        "toe_radius_increase_allowed": False,
        "zero_or_baseline_grid_values_are_not_duplicated_as_isolates": True,
        "isolated_selectable_count_by_arm": dict(ISOLATED_COUNT_BY_ARM),
        "isolated_selectable_count": ISOLATED_SELECTABLE_COUNT,
        "stage1_pair_count_including_baseline": ISOLATED_PAIR_COUNT,
        "every_isolated_candidate_changes_exactly_one_v13_parameter": True,
    }


def _expected_combinations() -> dict[str, Any]:
    return {
        "stage2_design": "local_cartesian_product_of_three_values_per_arm",
        "arm_set_rule": [
            "exact_V13_baseline_value",
            "diagnostic_root_safe_arm_winner_or_V13_fallback",
            "one_deterministic_grid-adjacent_neighbor_of_the_winner",
        ],
        "neighbor_rule": {
            "winner_is_v13": "use_the_first_nonbaseline_grid_neighbor",
            "interior_winner": (
                "rank_left_and_right_neighbors_with_the_same_arm_tuple; "
                "tie_then_closest_to_v13_then_grid_index_then_candidate_id"
            ),
            "outer_boundary_winner": "use_the_single_inward_neighbor",
        },
        "canonical_axis_order": list(ARM_ORDER),
        "maximum_cartesian_tuple_count_before_deduplication": (
            MAXIMUM_STAGE2_CARTESIAN_TUPLES
        ),
        "deduplicate_identical_geometry": True,
        "deduplication_rule": "canonical_lexicographic_geometry_tuple",
        "deduplicate_against_v13_and_isolated_stage1_tuples": True,
        "existing_v13_or_isolated_rows_are_reused_when_in_the_cartesian_set": True,
        "new_cartesian_tuples_reuse_stage1_station_samples": True,
        "claim_limit": "best_in_preregistered_staged_neighborhood_not_4d_minimum",
    }


def _expected_sampling() -> dict[str, Any]:
    return {
        "development_stage1": {
            "method": "shared_station_sampling_radius_evaluated_offline",
            "expected_unique_heel_locations": 8,
            "expected_unique_toe_locations": 7,
            "expected_unique_detector_stations": 15,
            "expected_primary_load_spheres": 8,
            "expected_total_sampled_stations": 23,
            "evaluated_pair_count": 24,
        },
        "development_combinations": {
            "maximum_cartesian_tuple_count": MAXIMUM_STAGE2_CARTESIAN_TUPLES,
            "new_opensim_station_sampling_passes": 0,
            "all_locations_must_exist_in_stage1_plan": True,
            "radius_variants_reuse_identical_station_kinematics": True,
        },
        "validation_and_sealed": {
            "pair_count": 2,
            "pairs": ["frozen_primary", "V13_paired_comparator"],
            "unique_detector_station_count_range": [2, 4],
            "primary_load_spheres": 8,
            "one_continuous_trial_sampling_pass_per_cadence": True,
            "other_isolates_or_combinations_forbidden": True,
        },
        "affine_reconstruction": False,
    }


def _expected_boundary_contract() -> dict[str, Any]:
    return {
        "unit": "trial_x_plateau_x_cadence",
        "plateaus_per_trial": 4,
        "minimum_scoreable_complete_cycles_per_plateau": 10,
        "left_boundary": (
            "derive complete cycles inside the plateau, thereby excluding the "
            "contact/cycle already active at plateau start"
        ),
        "fsm_replay_scope": "continuous_over_the_entire_trial_without_plateau_reset",
        "left_context_s": LEFT_CONTEXT_S,
        "left_context_role": "continuous_state_guard_not_a_scoreable_reference_cycle",
        "right_boundary": (
            "retain the chronological prefix whose closing HS plus the common "
            "right observation margin is strictly inside the plateau"
        ),
        "right_observation_margin_s": RIGHT_OBSERVATION_MARGIN_S,
        "right_observation_margin_rule": (
            "max(HS_tolerance_50ms,sensor_dwell_30ms)+runtime_sample_10ms"
        ),
        "common_cycle_set_at_10ms_and_1ms": True,
        "structural_metrics_are_scored_on_each_local_unit_from_continuous_trial_state": True,
        "whole_trial_structural_metrics_cannot_be_attributed_to_a_plateau": True,
        "trial_aggregate_rule": (
            "union_the_four_plateau_cycle_sets_and_sum_counts_per_cadence; "
            "exclude_all_ramp_cycles"
        ),
        "trial_aggregate_is_an_additional_gate_not_a_ranking_weight": True,
    }


def _expected_selection() -> dict[str, Any]:
    return {
        "isolated_arm_root_safety": {
            "full_v13_gate_required": False,
            "finite_priority_and_structural_metrics_required": True,
            "mesh_pre_gate_and_positive_radii_required": True,
            "reference_counts_must_equal_unit_contract": True,
            "per_event_predicted_count_rule": (
                "abs(candidate-reference)<=max(abs(V13-reference),1)"
            ),
            "valid_cycle_deficit_rule": (
                "abs(candidate-reference_cycles)<=max(abs(V13-reference_cycles),1)"
            ),
            "incomplete_transfer_and_unknown_samples_must_not_exceed_v13": True,
            "invalid_plus_unaccepted_and_forbidden_may_exceed_v13_by_at_most_one": True,
        },
        "isolated_arm_ranking": {
            "common_prefix": [
                "worst_event_count_deficit:min",
                "sum_event_count_deficit:min",
                "worst_valid_cycle_deficit:min",
                "worst_incomplete_transfer_count:min",
                "worst_unknown_phase_samples:min",
                "worst_invalid_plus_unaccepted_count:min",
                "worst_forbidden_phase_mismatch_count:min",
            ],
            "Hx": [
                "worst_normalized_hs:min",
                "worst_normalized_to:min",
                "worst_f1_deficit:min",
                "geometry_displacement:min",
                "candidate_id:lexicographic",
            ],
            "Hr": [
                "worst_normalized_hs:min",
                "worst_normalized_to:min",
                "worst_f1_deficit:min",
                "geometry_displacement:min",
                "candidate_id:lexicographic",
            ],
            "Ty": [
                "worst_normalized_to:min",
                "worst_incomplete_transfer_count:min",
                "worst_normalized_hs:min",
                "worst_f1_deficit:min",
                "geometry_displacement:min",
                "candidate_id:lexicographic",
            ],
            "Tr": [
                "worst_normalized_to:min",
                "worst_incomplete_transfer_count:min",
                "worst_normalized_hs:min",
                "worst_f1_deficit:min",
                "geometry_displacement:min",
                "candidate_id:lexicographic",
            ],
            "fallback_when_no_root_safe_candidate": "exact_V13_parameter_value",
        },
        "full_finalist_gate": {
            "must_pass_all_24_trial_x_plateau_x_cadence_units": True,
            "must_also_pass_each_trial_x_cadence_aggregate_gate": True,
            "trial_aggregate_is_not_a_ranking_weight": True,
            "uses_unchanged_v13_thresholds_and_semantics": True,
            "exact_counts_are_dynamic_from_each_unit_reference": True,
        },
        "minimax_vector": [
            "worst_joint_normalized",
            "worst_hs_s",
            "worst_to_s",
            "equal_cell_mean_normalized_error",
            "worst_f1_deficit",
            "worst_iou_deficit",
        ],
        "challenger_rule": {
            "pareto_noninferior_to_v13_with_tolerance": PARETO_TOLERANCE,
            "at_least_one_component_improves_beyond_tolerance": True,
            "v13_is_incumbent_and_wins_ties": True,
        },
        "eligible_finalist_ranking": [
            "worst_joint_normalized:min",
            "worst_hs_s:min",
            "worst_to_s:min",
            "equal_cell_mean_normalized_error:min",
            "worst_f1_deficit:min",
            "worst_iou_deficit:min",
            "geometry_displacement:min",
            "candidate_id:lexicographic",
        ],
    }


def _expected_decision_contract() -> dict[str, Any]:
    return {
        "development_trials": ["02", "04", "08"],
        "validation_trial": "05",
        "sealed_trial": "06",
        "reserve_trials_never_opened": ["03", "07"],
        "validation_receipt_required_before_semantic_decode": True,
        "sealed_receipt_required_before_semantic_decode": True,
        "validation_opens_only_after_one_frozen_dev_finalist": True,
        "sealed_opens_only_after_validation_full_gate_and_paired_tuple_pass": True,
        "holdout_scope_is_frozen_primary_plus_v13_in_one_pass": True,
        "primary_alone_determines_gate_no_rescue_or_reselection": True,
        "holdout_promotion_requires_full_gate_and_pareto_vs_v13": True,
        "holdout_selector_tuple_is_retained_as_diagnostic_ranking": True,
        "development_candidate_lock_written_before_validation_receipt": True,
        "validation_receipt_pins_development_candidate_lock_sha256": True,
        "validation_decision_lock_written_before_sealed_receipt": True,
        "sealed_receipt_pins_development_and_validation_lock_sha256": True,
        "outer_boundary_improving_stop_occurs_before_validation_receipt": True,
        "no_clobber_and_crash_consumes_destination": True,
        "canonical_global_ledger_blocks_alternate_destination_reopen": True,
        "automatic_profile_creation_allowed": False,
        "automatic_profile_or_registry_promotion_allowed": False,
        "runtime_modification_allowed": False,
        "training_allowed": False,
    }


REQUIRED_SOURCE_PATHS = {
    "v14_harness": "validation/sweep_two_sensor_cross_speed_v14.py",
    "v14_tests": "validation/test_two_sensor_cross_speed_v14.py",
    "v14_metadata_audit": "validation/audit_two_sensor_v14_trial_metadata.py",
    "v14_metadata_tests": "validation/test_two_sensor_v14_trial_metadata.py",
    "v13_validator": "validation/validate_two_sensor_sealed_v13.py",
    "v12_sweep": "validation/sweep_two_sensor_toe_compensation_prescribed_v12.py",
    "v11_sweep": "validation/sweep_two_sensor_heel_x_prescribed_v11.py",
    "v10_sweep": "validation/sweep_two_sensor_center_radius_prescribed_v10.py",
    "epic_converter": "tools/convert_epic_ab06_tables.py",
    "setup_io": "setup_io.py",
    "baseline_registry": BASELINE_REGISTRY_PATH,
    "baseline_profile": BASELINE_PROFILE_PATH,
    "v9_profile": V9_PROFILE_PATH,
    "marker_calibrated_model": MARKER_MODEL_PATH,
    "reserve_actuators": RESERVE_ACTUATORS_PATH,
    "load_evidence_profile": dual.LOAD_EVIDENCE_PROFILE,
    "left_foot_mesh": LEFT_FOOT_MESH_PATH,
    "golden_conversion_manifest": GOLDEN_CONVERSION_MANIFEST_PATH,
    "golden_ik_execution_receipt": GOLDEN_IK_EXECUTION_RECEIPT_PATH,
    "golden_ik_final_receipt": GOLDEN_IK_RECEIPT_PATH,
    "golden_v13_ik_parity": GOLDEN_V13_PARITY_PATH,
    "golden_marker_ik": GOLDEN_MARKER_IK_PATH,
    "v13_golden_ik": V13_GOLDEN_IK_PATH,
    "sea_plugin_binary_macos": (
        "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
    ),
}


def _frozen_schedule(trial_id: str) -> dict[str, Any]:
    frozen = metadata.FROZEN_CONDITIONS[trial_id]
    start_s, end_s = frozen["trial_interval_s"]
    return {
        "trial_interval_s": [float(start_s), float(end_s)],
        "plateaus": [item.to_json() for item in frozen["plateaus"]],
    }


def _validate_metadata_audit(payload: Mapping[str, Any]) -> None:
    if not (
        payload.get("schema_version") == metadata.SCHEMA_VERSION
        and payload.get("audit_id") == metadata.AUDIT_ID
        and payload.get("ok") is True
        and payload.get("status") == "PASS_METADATA_ONLY"
    ):
        raise ProtocolError("V14 metadata audit is not a passing frozen audit")
    expected_split = {
        role: list(ids) for role, ids in metadata.FROZEN_SPLIT.items()
    }
    if payload.get("frozen_split") != expected_split:
        raise ProtocolError("V14 metadata-audit split drifted")
    downstream = payload.get("downstream_requirements_not_executed_by_this_audit", {})
    if downstream.get("inverse_kinematics") != metadata.DOWNSTREAM_IK_CONTRACT:
        raise ProtocolError("V14 downstream IK contract drifted")
    if downstream.get("detector_sampling") != metadata.DOWNSTREAM_DETECTOR_CONTRACT:
        raise ProtocolError("V14 downstream detector contract drifted")
    observed_conditions = payload.get("condition_metadata", {})
    inventory = payload.get("inventory", {})
    for trial_id in metadata.SEMANTIC_CONDITION_TRIAL_IDS:
        schedule = _frozen_schedule(trial_id)
        observed = observed_conditions.get(trial_id, {})
        if (
            observed.get("trial_start_s") != schedule["trial_interval_s"][0]
            or observed.get("trial_end_s") != schedule["trial_interval_s"][1]
            or observed.get("plateaus") != schedule["plateaus"]
            or inventory.get(trial_id, {}).get("role")
            != next(
                role for role, ids in metadata.FROZEN_SPLIT.items()
                if trial_id in ids
            )
        ):
            raise ProtocolError(f"V14 metadata drifted for trial {trial_id}")


def _metadata_or_placeholder(
    path: str | Path, *, require: bool
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    resolved = v1.resolve_repo_path(path).resolve()
    if not resolved.is_file():
        if require:
            raise ProtocolError(
                f"metadata audit must exist before protocol freeze: {resolved}"
            )
        return None, {
            "path": _portable_path(resolved),
            "sha256": "<FILL_AFTER_METADATA_AUDIT>",
        }
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot read metadata audit: {resolved}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError("metadata audit root must be an object")
    _validate_metadata_audit(payload)
    return payload, _source_record(resolved)


def _trial_catalog(
    audit: Mapping[str, Any] | None,
) -> dict[str, Any]:
    catalog: dict[str, Any] = {}
    role_by_trial = {
        trial_id: role
        for role, trial_ids in metadata.FROZEN_SPLIT.items()
        for trial_id in trial_ids
    }
    for trial_id in metadata.SEMANTIC_CONDITION_TRIAL_IDS:
        raw_sources: dict[str, Any] = {}
        for stream in ("conditions", "ik", "fp", "markers"):
            if audit is None:
                raw_sources[stream] = {
                    "path": (
                        "models/AB06-raw/10_09_18/treadmill/"
                        f"{stream}/treadmill_{trial_id}_01.mat"
                    ),
                    "size_bytes": "<FILL_AFTER_METADATA_AUDIT>",
                    "sha256": "<FILL_AFTER_METADATA_AUDIT>",
                }
            else:
                source = dict(audit["inventory"][trial_id]["streams"][stream])
                raw_sources[stream] = {
                    "path": source["path"],
                    "size_bytes": source["size_bytes"],
                    "sha256": source["sha256"],
                }
        catalog[trial_id] = {
            "role": role_by_trial[trial_id],
            **_frozen_schedule(trial_id),
            "raw_sources": raw_sources,
            "dataset_ik_is_conversion_only_not_detector_kinematics": True,
        }
    return catalog


def _validated_preprocessing_golden() -> dict[str, Any]:
    """Validate the consumed-trial marker-IK receipt and byte parity evidence."""

    paths = {
        "conversion_manifest": v1.resolve_repo_path(
            GOLDEN_CONVERSION_MANIFEST_PATH
        ).resolve(),
        "execution_receipt": v1.resolve_repo_path(
            GOLDEN_IK_EXECUTION_RECEIPT_PATH
        ).resolve(),
        "final_receipt": v1.resolve_repo_path(GOLDEN_IK_RECEIPT_PATH).resolve(),
        "parity": v1.resolve_repo_path(GOLDEN_V13_PARITY_PATH).resolve(),
        "generated_ik": v1.resolve_repo_path(GOLDEN_MARKER_IK_PATH).resolve(),
        "v13_ik": v1.resolve_repo_path(V13_GOLDEN_IK_PATH).resolve(),
    }
    for label, path in paths.items():
        if not path.is_file():
            raise ProtocolError(f"missing preprocessing golden {label}: {path}")
    receipt = json.loads(paths["final_receipt"].read_text(encoding="utf-8"))
    parity = json.loads(paths["parity"].read_text(encoding="utf-8"))
    output = receipt.get("output_ik", {})
    plugin = receipt.get("plugin", {})
    model = receipt.get("model", {})
    marker = receipt.get("marker_contract", {})
    provenance = parity.get("causal_provenance", {})
    checks = {
        "final_receipt_verified": receipt.get("status") == "IK_OUTPUT_VERIFIED",
        "trial01_identity": receipt.get("trial") == "treadmill_01_01",
        "opensim_4p5p2": receipt.get("opensim_version") == "4.5.2",
        "model_hash": model.get("sha256")
        == metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256,
        "plugin_hash": plugin.get("binary_sha256")
        == EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX[".dylib"],
        "marker_contract": marker.get("count") == 28
        and marker.get("apply") is True
        and float(marker.get("weight", 0.0)) == 1.0
        and float(marker.get("accuracy", 0.0)) == 1.0e-5,
        "full_span_output_contract": output.get("rows") == 28612
        and output.get("coordinate_count") == 21
        and output.get("time_range_s") == [11.99, 155.045]
        and output.get("sha256") == EXPECTED_GOLDEN_IK_SHA256,
        "parity_pass": parity.get("status") == "PASS",
        "byte_identical": parity.get("comparison", {}).get("byte_identical") is True
        and parity.get("comparison", {}).get("cmp_exit_code") == 0,
        "generated_hash": parity.get("generated_ik", {}).get("sha256")
        == EXPECTED_GOLDEN_IK_SHA256,
        "v13_hash": parity.get("golden_v13_ik", {}).get("sha256")
        == EXPECTED_GOLDEN_IK_SHA256,
        "live_generated_hash": _sha256(paths["generated_ik"])
        == EXPECTED_GOLDEN_IK_SHA256,
        "live_v13_hash": _sha256(paths["v13_ik"])
        == EXPECTED_GOLDEN_IK_SHA256,
        "receipt_link": provenance.get("final_receipt", {}).get("sha256")
        == _sha256(paths["final_receipt"]),
        "execution_link": provenance.get("execution_receipt", {}).get("sha256")
        == _sha256(paths["execution_receipt"]),
        "manifest_link": provenance.get("conversion_manifest", {}).get("sha256")
        == _sha256(paths["conversion_manifest"]),
    }
    if not all(checks.values()):
        raise ProtocolError(f"preprocessing golden/parity drifted: {checks}")
    return {
        "status": "PASS_VERIFIED_FULL_SPAN_BYTE_PARITY",
        "checks": checks,
        "expected_marker_ik_sha256": EXPECTED_GOLDEN_IK_SHA256,
        "sources": {label: _source_record(path) for label, path in paths.items()},
    }


def expected_protocol_payload(
    metadata_audit_path: str | Path = DEFAULT_METADATA_AUDIT,
    *,
    require_metadata_audit: bool = True,
    require_all_sources: bool = True,
) -> dict[str, Any]:
    """Return the exact frozen V14 protocol without decoding trial streams."""

    audit, audit_record = _metadata_or_placeholder(
        metadata_audit_path, require=require_metadata_audit
    )
    replay = dict(v11._expected_replay())
    replay["evaluate_all_candidates_at_both_resolutions"] = True
    preprocessing_golden = _validated_preprocessing_golden()
    sources = {
        label: _source_record(path, require=require_all_sources)
        for label, path in REQUIRED_SOURCE_PATHS.items()
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "freeze_date": "2026-07-22",
        "frozen_before_execution": True,
        "stage": "cross_speed_development_then_one_shot_validation_and_sealed",
        "objective": OBJECTIVE,
        "interpretation_limits": list(INTERPRETATION_LIMITS),
        "metadata_audit": audit_record,
        "split": {
            role: list(ids) for role, ids in metadata.FROZEN_SPLIT.items()
        },
        "trials": _trial_catalog(audit),
        "preprocessing": {
            "inverse_kinematics": dict(metadata.DOWNSTREAM_IK_CONTRACT),
            "semantic_raw_inputs_at_converter_boundary": ["ik", "markers", "fp"],
            "detector_replay_semantic_inputs": ["marker_based_ik", "fp"],
            "marker_converter": "tools/convert_epic_ab06_tables.py",
            "force_plate_converter": "tools/convert_epic_ab06_tables.py",
            "trial_time_range_source": "frozen_conditions_trialStarts_trialEnds",
            "sea_plugin_loaded_before_model_or_ik_tool": replay["sea_plugin"],
            "plugin_basename_has_no_platform_extension": True,
            "plugin_binary_sha256_by_platform_suffix": dict(
                EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX
            ),
            "plugin_binary_identity_must_be_verified_before_conversion": True,
            "ik_receipt_required_before_replay": True,
            "dataset_ik_semantically_decoded_for_schema_conversion_only": True,
            "dataset_ik_used_downstream": False,
            "ik_smoke_allowed": False,
            "execution_platform_scope": "macos_arm64_only_for_frozen_v14",
            "converter_implementation_remains_cross_platform": True,
            "windows_execution_allowed_without_preregistered_ff_dll_hash": False,
        },
        "preprocessing_golden": preprocessing_golden,
        "model_file": MARKER_MODEL_PATH,
        "reserve_actuators_xml": RESERVE_ACTUATORS_PATH,
        "detector_template_profile": BASELINE_PROFILE_PATH,
        "baseline_registry": BASELINE_REGISTRY_PATH,
        "historical_v9_profile": V9_PROFILE_PATH,
        "load_evidence_profile": dual.LOAD_EVIDENCE_PROFILE,
        "replay": replay,
        "isolated_parameter_grid": _expected_grid(),
        "combination_formulas": _expected_combinations(),
        "sampling": _expected_sampling(),
        "unit_boundary_handling": _expected_boundary_contract(),
        "gate_contract": {
            "fixed_v13_thresholds_and_semantics": _fixed_gate_contract(),
            "reference_source": {
                "stream": "prescribed_left_vertical_grf",
                "threshold_n": 20.0,
                "minimum_contact_duration_s": 0.05,
                "minimum_cycle_duration_s": 0.3,
                "gcLeft_or_gcRight_allowed": False,
            },
            "runtime_10ms_and_fine_1ms_identical_except_dynamic_counts": True,
            "dynamic_exact_count_fields": [
                "require_exact_reference_counts",
                "require_exact_detector_counts",
                "require_exact_valid_cycles",
                "require_exact_causal_swing_intervals",
            ],
            "dynamic_count_source": (
                "prescribed_reference_complete_cycles_in_each_authorized_unit"
            ),
            "dynamic_counts_cannot_modify_thresholds_or_grids": True,
        },
        "causal_event_diagnostics": {
            "gate_time_field": "confirmed_time_s",
            "saved_per_event_per_cell": [
                "onset_error=event_time_s-prescribed_time_s",
                "confirmation_latency=confirmed_time_s-event_time_s",
                "total_error=confirmed_time_s-prescribed_time_s",
                "residual_quantization_or_routing=confirmation_latency-0.030",
            ],
            "summary_statistics": ["count", "mean", "minimum", "maximum", "max_abs"],
            "expected_confirmation_latency_range_is_cadence_aware": True,
            "operational_compensation_not_unique_physical_cause": True,
        },
        "detail_reporting": {
            "format": "canonical_compact_jsonl_no_clobber",
            "development_stage1_all_pairs": True,
            "development_stage2_all_new_pairs": True,
            "stage2_reused_pairs_are_already_persisted_in_stage1": True,
            "validation_and_sealed_frozen_pairs": True,
            "contains_event_gate_phase_semantic_transfer_and_decomposition": True,
            "all_detail_artifacts_are_hash_pinned_in_locks_or_manifest": True,
        },
        "selection": _expected_selection(),
        "decision_contract": _expected_decision_contract(),
        "execution_destination": {
            "canonical_output_dir": _portable_path(DEFAULT_OUTPUT_DIR),
            "canonical_one_shot_ledger": _portable_path(DEFAULT_EXECUTION_LEDGER),
            "alternate_output_directory_allowed": False,
            "ledger_written_before_any_development_semantic_decode": True,
        },
        "sources": sources,
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V14 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("V14 protocol root must be an object")
    expected = expected_protocol_payload(require_metadata_audit=True)
    if raw != expected:
        differing = sorted(
            key for key in set(raw) | set(expected) if raw.get(key) != expected.get(key)
        )
        raise ProtocolError(f"V14 frozen protocol drifted: {differing}")
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    raw["profile_paths"] = {v1.CURRENT_PROFILE_ID: BASELINE_PROFILE_PATH}
    return raw


def _load_json_source(path: str | Path) -> tuple[Path, dict[str, Any]]:
    resolved = v1.resolve_repo_path(path).resolve()
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load JSON source: {resolved}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError(f"JSON source is not an object: {resolved}")
    return resolved, payload


def _baseline_geometry(protocol: Mapping[str, Any]) -> dict[str, Any]:
    registry_path, registry = _load_json_source(protocol["baseline_registry"])
    profile_path, profile = _load_json_source(protocol["detector_template_profile"])
    v9_path, v9_profile = _load_json_source(protocol["historical_v9_profile"])
    active = registry.get("active_development_baseline", {})
    if not (
        active.get("baseline_id") == "two_sensor_v13_development_baseline"
        and active.get("candidate_id")
        == "toe_down_p0p75mm__heel_x_p3p5mm"
        and active.get("status") == "ACTIVE_DEVELOPMENT_BASELINE"
        and active.get("profile", {}).get("path") == BASELINE_PROFILE_PATH
        and active.get("profile", {}).get("sha256") == _sha256(profile_path)
    ):
        raise ProtocolError("V13 development-baseline registry binding drifted")
    for expected, payload, label in (
        ({"left_heel", "left_toe"}, profile, "V13"),
        ({"left_heel", "left_toe"}, v9_profile, "V9"),
    ):
        names = {item.get("name") for item in payload.get("spheres", [])}
        if names != expected:
            raise ProtocolError(f"{label} profile is not exactly heel + toe")

    def sensors(payload: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
        return {str(item["name"]): item for item in payload["spheres"]}

    current = sensors(profile)
    historical = sensors(v9_profile)
    heel_v13 = current["left_heel"]
    toe_v13 = current["left_toe"]
    heel_v9 = historical["left_heel"]
    toe_v9 = historical["left_toe"]
    checks = {
        "heel_x_shift_is_3p5mm": math.isclose(
            1000.0 * (float(heel_v13["location"][0]) - float(heel_v9["location"][0])),
            3.5,
            abs_tol=NUMERIC_TOLERANCE,
        ),
        "toe_down_is_0p75mm": math.isclose(
            1000.0 * (float(toe_v9["location"][1]) - float(toe_v13["location"][1])),
            0.75,
            abs_tol=NUMERIC_TOLERANCE,
        ),
        "heel_radius_unchanged": float(heel_v13["radius"])
        == float(heel_v9["radius"]),
        "toe_radius_unchanged": float(toe_v13["radius"])
        == float(toe_v9["radius"]),
    }
    if not all(checks.values()):
        raise ProtocolError(f"V13/V9 baseline geometry drifted: {checks}")
    return {
        "registry": _source_record(registry_path),
        "v13_profile": _source_record(profile_path),
        "v9_profile": _source_record(v9_path),
        "v9_heel_location": tuple(float(x) for x in heel_v9["location"]),
        "v9_toe_location": tuple(float(x) for x in toe_v9["location"]),
        "v13_heel_location": tuple(float(x) for x in heel_v13["location"]),
        "v13_toe_location": tuple(float(x) for x in toe_v13["location"]),
        "v13_heel_radius_m": float(heel_v13["radius"]),
        "v13_toe_radius_m": float(toe_v13["radius"]),
        "checks": checks,
    }


def _geometry_tuple(candidate: Any) -> tuple[float, ...]:
    return (
        *tuple(float(value) for value in candidate.heel_location),
        *tuple(float(value) for value in candidate.forefoot_location),
        float(candidate.geometry["heel_radius_m"]),
        float(candidate.geometry["toe_radius_m"]),
    )


def _make_candidate(
    *,
    candidate_id: str,
    role: str,
    arm: str,
    selectable: bool,
    heel_x_shift_from_v9_mm: float,
    heel_radius_reduction_from_v13_mm: float,
    toe_down_from_v9_mm: float,
    toe_radius_reduction_from_v13_mm: float,
    baseline_geometry: Mapping[str, Any],
    triangles: np.ndarray,
) -> Any:
    v9_heel = baseline_geometry["v9_heel_location"]
    v9_toe = baseline_geometry["v9_toe_location"]
    heel_location = (
        float(v9_heel[0]) + heel_x_shift_from_v9_mm / 1000.0,
        float(v9_heel[1]),
        float(v9_heel[2]),
    )
    toe_location = (
        float(v9_toe[0]),
        float(v9_toe[1]) - toe_down_from_v9_mm / 1000.0,
        float(v9_toe[2]),
    )
    heel_radius = (
        float(baseline_geometry["v13_heel_radius_m"])
        - heel_radius_reduction_from_v13_mm / 1000.0
    )
    toe_radius = (
        float(baseline_geometry["v13_toe_radius_m"])
        - toe_radius_reduction_from_v13_mm / 1000.0
    )
    if heel_radius <= 0.0 or toe_radius <= 0.0:
        raise ProtocolError(f"{candidate_id} produced a non-positive radius")
    heel_mesh = v10.v9.v8.v6.v3._mesh_geometry(
        heel_location, heel_radius, triangles
    )
    toe_mesh = v10.v9.v8.v6.v3._mesh_geometry(toe_location, toe_radius, triangles)
    bottom_offset = (toe_location[1] - toe_radius) - (
        heel_location[1] - heel_radius
    )
    baseline_values = (3.5, 0.0, 0.75, 0.0)
    values = (
        float(heel_x_shift_from_v9_mm),
        float(heel_radius_reduction_from_v13_mm),
        float(toe_down_from_v9_mm),
        float(toe_radius_reduction_from_v13_mm),
    )
    changed = sum(
        not math.isclose(a, b, abs_tol=NUMERIC_TOLERANCE)
        for a, b in zip(values, baseline_values)
    )
    expected_changes = 0 if arm == "baseline" else 1 if arm in ARM_ORDER else changed
    checks = {
        "exactly_two_spheres": True,
        "heel_within_5mm_of_mesh": bool(heel_mesh["within_5mm_of_mesh"]),
        "toe_within_5mm_of_mesh": bool(toe_mesh["within_5mm_of_mesh"]),
        "positive_radii": heel_radius > 0.0 and toe_radius > 0.0,
        "heel_to_toe_bottom_offset_within_20mm": abs(bottom_offset)
        <= 0.020 + NUMERIC_TOLERANCE,
        "declared_parameter_change_count": changed == expected_changes,
    }
    displacement_mm = (
        abs(values[0] - baseline_values[0])
        + abs(values[1] - baseline_values[1])
        + abs(values[2] - baseline_values[2])
        + abs(values[3] - baseline_values[3])
    )
    return v1.PlacementCandidate(
        candidate_id=candidate_id,
        heel_location=heel_location,
        forefoot_location=toe_location,
        heel_offset_below_current_mm=None,
        forefoot_fraction_mesh_x=None,
        forefoot_protrusion_mm=None,
        selectable=selectable,
        role=role,
        geometry={
            "source": "registry_bound_v13_development_baseline",
            "parameter_arm": arm,
            "heel_x_shift_from_v9_mm": values[0],
            "heel_radius_reduction_from_v13_mm": values[1],
            "toe_down_from_v9_mm": values[2],
            "toe_radius_reduction_from_v13_mm": values[3],
            # Compatibility aliases used by inherited V10--V13 diagnostics.
            "heel_x_shift_mm": values[0],
            "toe_center_down_mm": values[2],
            "center_raise_mm": -values[2],
            "radius_reduction_mm": values[1] + values[3],
            "heel_center_raise_mm": 0.0,
            "heel_radius_reduction_mm": values[1],
            "toe_center_raise_mm": -values[2],
            "toe_radius_reduction_mm": values[3],
            "heel_radius_m": float(heel_radius),
            "toe_radius_m": float(toe_radius),
            "effective_bottom_raise_mm": -values[2] + values[3],
            "geometry_displacement_from_v13_m": displacement_mm / 1000.0,
            "heel_to_toe_bottom_offset_m": float(bottom_offset),
            "heel": heel_mesh,
            "toe": toe_mesh,
            "pre_gate_checks": checks,
            "pre_gate_ok": bool(all(checks.values())),
            "detector_representation": "two_spheres_only_detector_not_grf_generator",
        },
    )


def build_isolated_candidates(
    protocol: Mapping[str, Any],
) -> tuple[Any, list[Any], dict[str, Any]]:
    baseline_geometry = _baseline_geometry(protocol)
    base = v1.load_online_grf_profile(
        v1.resolve_repo_path(protocol["detector_template_profile"]).resolve(),
        required_sides=("left",),
    )
    model_path = v1.resolve_repo_path(protocol["model_file"]).resolve()
    if _sha256(model_path) != metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256:
        raise ProtocolError("marker-calibrated model identity drifted")
    mesh_path = v1._resolve_left_foot_mesh(model_path)
    expected_mesh_path = v1.resolve_repo_path(LEFT_FOOT_MESH_PATH).resolve()
    if (
        mesh_path.resolve() != expected_mesh_path
        or _sha256(mesh_path) != EXPECTED_LEFT_FOOT_MESH_SHA256
    ):
        raise ProtocolError("left-foot pre-gate mesh identity drifted")
    triangles = v1._load_stl_triangles(mesh_path)
    candidates: list[Any] = []
    candidates.append(
        _make_candidate(
            candidate_id=BASELINE_ID,
            role="nonselectable_v13_incumbent",
            arm="baseline",
            selectable=False,
            heel_x_shift_from_v9_mm=3.5,
            heel_radius_reduction_from_v13_mm=0.0,
            toe_down_from_v9_mm=0.75,
            toe_radius_reduction_from_v13_mm=0.0,
            baseline_geometry=baseline_geometry,
            triangles=triangles,
        )
    )
    serial = 1
    for value in HEEL_X_ABSOLUTE_FROM_V9_MM[1:]:
        candidates.append(
            _make_candidate(
                candidate_id=f"I{serial:02d}_Hx_{_token(value)}mm",
                role="isolated_diagnostic_root_safe_component",
                arm="Hx",
                selectable=True,
                heel_x_shift_from_v9_mm=value,
                heel_radius_reduction_from_v13_mm=0.0,
                toe_down_from_v9_mm=0.75,
                toe_radius_reduction_from_v13_mm=0.0,
                baseline_geometry=baseline_geometry,
                triangles=triangles,
            )
        )
        serial += 1
    for value in HEEL_RADIUS_REDUCTION_FROM_V13_MM[1:]:
        candidates.append(
            _make_candidate(
                candidate_id=f"I{serial:02d}_Hr_{_token(value)}mm",
                role="isolated_diagnostic_root_safe_component",
                arm="Hr",
                selectable=True,
                heel_x_shift_from_v9_mm=3.5,
                heel_radius_reduction_from_v13_mm=value,
                toe_down_from_v9_mm=0.75,
                toe_radius_reduction_from_v13_mm=0.0,
                baseline_geometry=baseline_geometry,
                triangles=triangles,
            )
        )
        serial += 1
    for value in TOE_DOWN_ABSOLUTE_FROM_V9_MM[1:]:
        candidates.append(
            _make_candidate(
                candidate_id=f"I{serial:02d}_Ty_{_token(value)}mm",
                role="isolated_diagnostic_root_safe_component",
                arm="Ty",
                selectable=True,
                heel_x_shift_from_v9_mm=3.5,
                heel_radius_reduction_from_v13_mm=0.0,
                toe_down_from_v9_mm=value,
                toe_radius_reduction_from_v13_mm=0.0,
                baseline_geometry=baseline_geometry,
                triangles=triangles,
            )
        )
        serial += 1
    for value in TOE_RADIUS_REDUCTION_FROM_V13_MM[1:]:
        candidates.append(
            _make_candidate(
                candidate_id=f"I{serial:02d}_Tr_{_token(value)}mm",
                role="isolated_diagnostic_root_safe_component",
                arm="Tr",
                selectable=True,
                heel_x_shift_from_v9_mm=3.5,
                heel_radius_reduction_from_v13_mm=0.0,
                toe_down_from_v9_mm=0.75,
                toe_radius_reduction_from_v13_mm=value,
                baseline_geometry=baseline_geometry,
                triangles=triangles,
            )
        )
        serial += 1
    if len(candidates) != ISOLATED_PAIR_COUNT or serial != 24:
        raise ProtocolError("V14 isolated candidate cardinality drifted")
    if len({item.candidate_id for item in candidates}) != len(candidates):
        raise ProtocolError("V14 isolated candidate IDs are not unique")
    if sum(bool(item.selectable) for item in candidates) != ISOLATED_SELECTABLE_COUNT:
        raise ProtocolError("V14 isolated selectable count drifted")
    if any(not bool(item.geometry["pre_gate_ok"]) for item in candidates):
        failed = [
            item.candidate_id for item in candidates
            if not bool(item.geometry["pre_gate_ok"])
        ]
        raise ProtocolError(f"V14 candidate mesh pre-gate failed: {failed}")
    return base, candidates, {
        **baseline_geometry,
        "mesh": _source_record(mesh_path),
        "isolated_pair_count": len(candidates),
        "isolated_selectable_count": ISOLATED_SELECTABLE_COUNT,
        "triangles": triangles,
    }


def _candidate_record(candidate: Any) -> dict[str, Any]:
    return {
        "candidate_id": candidate.candidate_id,
        "selectable": bool(candidate.selectable),
        "role": candidate.role,
        "heel_location_m": list(candidate.heel_location),
        "toe_location_m": list(candidate.forefoot_location),
        "heel_radius_m": float(candidate.geometry["heel_radius_m"]),
        "toe_radius_m": float(candidate.geometry["toe_radius_m"]),
        "geometry": dict(candidate.geometry),
    }


def _build_sampling_plan(
    base: Any,
    candidates: Sequence[Any],
    *,
    stage_label: str,
    expected_detector_stations: int | None,
) -> SamplingPlan:
    sensors = v1._left_sensor_spheres(base)
    templates = {"heel": sensors["left_heel"], "toe": sensors["left_toe"]}
    station_by_key: dict[tuple[str, tuple[float, float, float]], Any] = {}

    def station(role: str, location: Iterable[float]) -> Any:
        key = (role, tuple(float(value) for value in location))
        if key not in station_by_key:
            station_by_key[key] = replace(
                templates[role],
                name=f"v14_{stage_label}_{role}_{len(station_by_key):02d}",
                location=key[1],
            )
        return station_by_key[key]

    pairs: dict[str, dict[str, Any]] = {}
    profiles: dict[str, Any] = {}
    for candidate in candidates:
        heel_station = station("heel", candidate.heel_location)
        toe_station = station("toe", candidate.forefoot_location)
        heel = replace(
            heel_station, radius=float(candidate.geometry["heel_radius_m"])
        )
        toe = replace(toe_station, radius=float(candidate.geometry["toe_radius_m"]))
        pairs[candidate.candidate_id] = {"heel": heel, "toe": toe}
        profiles[candidate.candidate_id] = replace(
            base,
            source=f"validation_v14_{stage_label}_{candidate.candidate_id}",
            spheres=(heel, toe),
        )
    sampler = replace(
        base,
        source=f"validation_v14_{stage_label}_shared_station_sampler",
        spheres=tuple(station_by_key.values()),
    )
    if (
        expected_detector_stations is not None
        and len(sampler.spheres) != expected_detector_stations
    ):
        raise ProtocolError(
            f"V14 {stage_label} station count {len(sampler.spheres)} != "
            f"{expected_detector_stations}"
        )
    if any(len(profile.spheres) != 2 for profile in profiles.values()):
        raise ProtocolError("V14 detector profile is not exactly heel + toe")
    return SamplingPlan(
        sampler=sampler,
        station_by_key=station_by_key,
        profiles=profiles,
        pairs=pairs,
    )


def _pair_for_candidate(
    base: Any, plan: SamplingPlan, candidate: Any
) -> tuple[Any, dict[str, Any]]:
    if candidate.candidate_id in plan.pairs:
        return plan.profiles[candidate.candidate_id], plan.pairs[candidate.candidate_id]
    heel_key = (
        "heel",
        tuple(float(value) for value in candidate.heel_location),
    )
    toe_key = (
        "toe",
        tuple(float(value) for value in candidate.forefoot_location),
    )
    if heel_key not in plan.station_by_key or toe_key not in plan.station_by_key:
        raise ProtocolError(
            f"combination {candidate.candidate_id} escaped the stage1 station plan"
        )
    heel = replace(
        plan.station_by_key[heel_key],
        radius=float(candidate.geometry["heel_radius_m"]),
    )
    toe = replace(
        plan.station_by_key[toe_key],
        radius=float(candidate.geometry["toe_radius_m"]),
    )
    profile = replace(
        base,
        source=f"validation_v14_reconstructed_{candidate.candidate_id}",
        spheres=(heel, toe),
    )
    return profile, {"heel": heel, "toe": toe}


def _unit_key(trial_id: str, plateau_index: int, cadence_label: str) -> str:
    return f"trial_{trial_id}__plateau_{plateau_index:02d}__{cadence_label}"


def retain_scoreable_plateau_cycles(
    raw_events: Mapping[str, Sequence[float] | np.ndarray],
    *,
    plateau_start_s: float,
    plateau_end_s: float,
    right_observation_margin_s: float = RIGHT_OBSERVATION_MARGIN_S,
    minimum_cycles: int = MINIMUM_SCOREABLE_CYCLES_PER_PLATEAU,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Freeze the cadence-independent complete-cycle prefix for one plateau.

    ``raw_events`` must already come from the prescribed left-GRF extractor,
    which removes contacts crossing either plateau boundary.  This pure layer
    additionally prefix-censors closing heel strikes that cannot be observed
    for the common 60 ms confirmation margin.  The returned event arrays are
    shared by every candidate and both cadences.
    """

    start = float(plateau_start_s)
    end = float(plateau_end_s)
    margin = float(right_observation_margin_s)
    if not (math.isfinite(start) and math.isfinite(end) and end > start):
        raise ProtocolError("invalid plateau interval")
    if not math.isfinite(margin) or margin < 0.0:
        raise ProtocolError("invalid right observation margin")
    heel = np.asarray(raw_events["heel_strike"], dtype=float)
    toe = np.asarray(raw_events["toe_off"], dtype=float)
    if (
        heel.ndim != 1
        or toe.ndim != 1
        or not np.all(np.isfinite(heel))
        or not np.all(np.isfinite(toe))
        or heel.size != toe.size + 1
        or heel.size < 2
    ):
        raise ProtocolError("plateau reference must be finite HS--TO--HS cycles")
    if np.any(np.diff(heel) <= 0.0) or np.any(np.diff(toe) <= 0.0):
        raise ProtocolError("plateau reference events are not strictly ordered")
    left_guard_observed = float(heel[0] - start)
    if left_guard_observed + NUMERIC_TOLERANCE < LEFT_CONTEXT_S:
        raise ProtocolError(
            "first scoreable HS lacks the frozen 90 ms continuous-state guard"
        )
    for index, toe_time in enumerate(toe):
        if not (
            start <= float(heel[index])
            < float(toe_time)
            < float(heel[index + 1])
            <= end
        ):
            raise ProtocolError("reference cycle is not fully internal to plateau")
    confirmable = heel[1:] + margin < end
    if np.any(np.diff(confirmable.astype(int)) > 0):
        raise ProtocolError("right-boundary confirmability is not a prefix")
    retained = int(np.count_nonzero(confirmable))
    if retained < int(minimum_cycles):
        raise ProtocolError(
            f"plateau has {retained} scoreable cycles; minimum is {minimum_cycles}"
        )
    events = {
        "heel_strike": heel[: retained + 1].copy(),
        "toe_off": toe[:retained].copy(),
    }
    return events, {
        "plateau_interval_s": [start, end],
        "raw_complete_cycle_count": int(toe.size),
        "retained_scoreable_cycle_count": retained,
        "excluded_right_boundary_cycle_count": int(toe.size - retained),
        "excluded_closing_hs_s": heel[retained + 1 :].tolist(),
        "scoreable_reference_interval_s": [
            float(events["heel_strike"][0]),
            float(events["heel_strike"][-1]),
        ],
        "right_observation_margin_s": margin,
        "left_context_guard_required_s": LEFT_CONTEXT_S,
        "left_context_guard_observed_s": left_guard_observed,
        "reference_hs_count": retained + 1,
        "reference_to_count": retained,
        "reference_event_sha256": _canonical_sha256(
            {key: value.tolist() for key, value in events.items()}
        ),
    }


def continuous_local_score_mask(
    times: Sequence[float] | np.ndarray,
    reference_events: Mapping[str, Sequence[float] | np.ndarray],
    *,
    plateau_start_s: float,
    plateau_end_s: float,
    left_context_s: float = LEFT_CONTEXT_S,
    right_observation_margin_s: float = RIGHT_OBSERVATION_MARGIN_S,
) -> np.ndarray:
    """Return a plateau-local view into a continuously replayed trial."""

    sample_times = np.asarray(times, dtype=float)
    heel = np.asarray(reference_events["heel_strike"], dtype=float)
    if sample_times.ndim != 1 or heel.ndim != 1 or heel.size < 2:
        raise ProtocolError("continuous local mask requires 1-D times and HS events")
    if not np.all(np.isfinite(sample_times)) or np.any(np.diff(sample_times) <= 0.0):
        raise ProtocolError("continuous trial time grid must be finite and increasing")
    lower = max(float(plateau_start_s), float(heel[0]) - float(left_context_s))
    upper = float(heel[-1]) + float(right_observation_margin_s)
    if upper >= float(plateau_end_s) + NUMERIC_TOLERANCE:
        raise ProtocolError("local score view escapes the plateau")
    mask = (sample_times >= lower - NUMERIC_TOLERANCE) & (
        sample_times <= upper + NUMERIC_TOLERANCE
    )
    if np.count_nonzero(mask) < 2:
        raise ProtocolError("local score view contains fewer than two samples")
    return mask


def _plateau_references(
    protocol: Mapping[str, Any],
    setup: setup_io.SimulationSetup,
    *,
    trial_id: str,
) -> tuple[dict[str, Any], ...]:
    """Derive the four common cycle sets without touching detector state."""

    replay_contract = protocol["replay"]
    result: list[dict[str, Any]] = []
    for plateau_index, plateau in enumerate(
        protocol["trials"][trial_id]["plateaus"], start=1
    ):
        start = float(plateau["start_s"])
        end = float(plateau["end_s"])
        raw_events, provenance = v1._reference_events_from_prescribed_grf(
            replace(setup, t_start=start, t_end=end),
            threshold_n=float(replay_contract["prescribed_contact_threshold_n"]),
            min_contact_duration_s=float(
                replay_contract["reference_min_contact_duration_s"]
            ),
            min_cycle_duration_s=float(
                replay_contract["reference_min_cycle_duration_s"]
            ),
        )
        events, cycle_access = retain_scoreable_plateau_cycles(
            raw_events,
            plateau_start_s=start,
            plateau_end_s=end,
        )
        result.append(
            {
                "trial_id": trial_id,
                "plateau_index": plateau_index,
                "speed_mps": float(plateau["speed_mps"]),
                "events": events,
                "dynamic_gate": _dynamic_gate(
                    len(events["heel_strike"]), len(events["toe_off"])
                ),
                "reference_provenance": provenance,
                **cycle_access,
            }
        )
    return tuple(result)


def sample_trial_cadence_once(
    protocol: Mapping[str, Any],
    setup: setup_io.SimulationSetup,
    base: Any,
    plan: SamplingPlan,
    *,
    trial_id: str,
    cadence_label: str,
    sample_dt_s: float,
) -> TrialCadenceBundle:
    """Sample all stations once on a full-trial continuous time grid."""

    if (cadence_label, sample_dt_s) not in CADENCES:
        raise ProtocolError("V14 permits only frozen 10 ms and 1 ms cadences")
    trial_start, trial_end = (
        float(value) for value in protocol["trials"][trial_id]["trial_interval_s"]
    )
    intervals = int(
        math.floor((trial_end - trial_start) / sample_dt_s + NUMERIC_TOLERANCE)
    )
    times = trial_start + np.arange(intervals + 1, dtype=float) * sample_dt_s
    if times.size < 2 or float(times[-1]) > trial_end + NUMERIC_TOLERANCE:
        raise ProtocolError("invalid full-trial continuous time grid")
    trial_setup = replace(setup, t_start=float(times[0]), t_end=float(times[-1]))
    references = _plateau_references(protocol, setup, trial_id=trial_id)
    primary_path = v1.resolve_repo_path(protocol["load_evidence_profile"]).resolve()
    primary_full = dual.load_online_grf_profile(primary_path)
    primary_spheres = tuple(
        sphere for sphere in primary_full.spheres if sphere.side == "left"
    )
    if len(primary_spheres) != EXPECTED_PRIMARY_LOAD_SPHERES:
        raise ProtocolError("V14 primary load-sphere count drifted")
    primary_left = replace(
        primary_full,
        source=f"validation_v14_trial_{trial_id}_{cadence_label}_primary_left",
        spheres=primary_spheres,
    )
    combined = tuple(plan.sampler.spheres) + primary_spheres
    if len({sphere.name for sphere in combined}) != len(combined):
        raise ProtocolError("V14 sampled station names are not unique")
    sampler = replace(
        base,
        source=f"validation_v14_trial_{trial_id}_{cadence_label}_combined_sampler",
        spheres=combined,
    )
    samples = v1._sample_spheres(
        trial_setup,
        sampler,
        times,
        str(protocol["replay"]["sea_plugin"]),
    )
    primary_aggregate = np.asarray(
        dual._calculate_wrench(primary_left, dict(samples))["left"]["normal_force"],
        dtype=float,
    )
    primary_penetration = dual._primary_physical_penetration(primary_left, samples)
    shared = {
        "setup": trial_setup,
        "times": times,
        "kinematics": v1._prescribed_prosthetic_kinematics(trial_setup, times),
        "prescribed_vertical_n": np.asarray(
            v1._external_grf(trial_setup, times)["left"][:, 1], dtype=float
        ),
        "body_weight_n": v1._model_body_weight_n(trial_setup.model_file),
        "primary_aggregate": primary_aggregate,
        "primary_penetration": primary_penetration,
    }
    access = {
        "trial_id": trial_id,
        "cadence": cadence_label,
        "sample_dt_s": float(sample_dt_s),
        "continuous_fsm_scope": "entire_trial_no_plateau_reset",
        "sample_interval_s": [float(times[0]), float(times[-1])],
        "single_opensim_station_sampling_pass": True,
        "radius_variants_reconstructed_offline": True,
        "sampled_unique_detector_stations": len(plan.sampler.spheres),
        "sampled_primary_load_spheres": len(primary_spheres),
        "sampled_total_stations": len(combined),
        "load_evidence_profile": _source_record(primary_path),
        "plateau_reference_digests": [
            item["reference_event_sha256"] for item in references
        ],
    }
    return TrialCadenceBundle(
        trial_id=trial_id,
        cadence_label=cadence_label,
        sample_dt_s=sample_dt_s,
        plan=plan,
        samples=samples,
        shared=shared,
        plateau_references=references,
        access=access,
    )


PRIORITY_METRICS = (
    "max_abs_hs_error_s",
    "max_abs_toe_off_error_s",
    "confirmed_fsm_stance_f1",
    "confirmed_fsm_stance_iou",
)


def _finite_priority(row: Mapping[str, Any]) -> bool:
    required = (*PRIORITY_METRICS, "precision", "recall")
    try:
        return all(math.isfinite(float(row[key])) for key in required)
    except (KeyError, TypeError, ValueError):
        return False


def _events_for_field(
    accepted: Sequence[Mapping[str, Any]], time_field: str
) -> dict[str, np.ndarray]:
    return thresholds._accepted_events_for_time_field(accepted, time_field)


def _local_valid_cycle_count(accepted: Sequence[Mapping[str, Any]]) -> int:
    transitions = sorted(
        [
            dict(item)
            for values in thresholds._accepted_gait_transitions_for_gate(accepted).values()
            for item in values
        ],
        key=lambda item: float(item["confirmed_time_s"]),
    )
    state = "await_hs"
    cycles = 0
    for item in transitions:
        event = str(item["event"])
        if event == "heel_strike":
            if state == "await_closing_hs":
                cycles += 1
            state = "await_to"
        elif event == "toe_off" and state == "await_to":
            state = "await_closing_hs"
        else:
            state = "invalid"
    return cycles


def _slice_continuous_replay(
    full_replay: Mapping[str, Any],
    full_times: np.ndarray,
    reference: Mapping[str, Any],
    *,
    sample_dt_s: float,
    sensor_dwell_s: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Take a local view without constructing or resetting an FSM."""

    start, end = (float(value) for value in reference["plateau_interval_s"])
    events = reference["events"]
    mask = continuous_local_score_mask(
        full_times,
        events,
        plateau_start_s=start,
        plateau_end_s=end,
    )
    closing_hs = float(events["heel_strike"][-1])
    confirmation_limit = closing_hs + RIGHT_OBSERVATION_MARGIN_S
    local_start = float(full_times[mask][0])
    ownership_floor = max(start, local_start)

    if sample_dt_s <= 0.0 or sensor_dwell_s < 0.0:
        raise ProtocolError("invalid cadence/dwell for local event ownership")

    def event_window(event: str) -> tuple[float, float, float, float]:
        tolerance = 0.05 if event == "heel_strike" else 0.08
        owned_reference = np.asarray(events[event], dtype=float)
        primary_start = max(
            ownership_floor, float(owned_reference[0]) - tolerance
        )
        primary_end = min(
            float(owned_reference[-1]) + tolerance,
            confirmation_limit,
            end - NUMERIC_TOLERANCE,
        )
        onset_start = max(
            ownership_floor,
            primary_start - sensor_dwell_s - sample_dt_s,
        )
        onset_end = primary_end - sensor_dwell_s
        return primary_start, primary_end, onset_start, onset_end

    def accepted_gait_in_scope(item: Mapping[str, Any]) -> bool:
        event = str(item.get("event", ""))
        if event not in {"heel_strike", "toe_off"}:
            return False
        primary_start, primary_end, _onset_start, _onset_end = event_window(event)
        onset = float(item["event_time_s"])
        confirmed = float(item["confirmed_time_s"])
        return bool(
            onset >= ownership_floor
            and confirmed >= ownership_floor
            and primary_start - NUMERIC_TOLERANCE
            <= confirmed
            <= primary_end + NUMERIC_TOLERANCE
        )

    accepted = [
        dict(item)
        for item in full_replay["accepted"]
        if (
            accepted_gait_in_scope(item)
            or (
                str(item.get("event", "")) == "timeout"
                and ownership_floor
                <= float(item.get("confirmed_time_s", item.get("observed_at_s", -math.inf)))
                <= confirmation_limit
            )
        )
    ]
    accepted_candidate_keys = {
        (str(item["event"]), round(float(item["event_time_s"]), 10))
        for item in accepted
        if str(item.get("event", "")) in {"heel_strike", "toe_off"}
    }

    def candidate_in_scope(item: Mapping[str, Any]) -> bool:
        event = str(item.get("event", ""))
        if event not in {"heel_strike", "toe_off"}:
            return False
        onset = float(item["time"])
        observed = float(item.get("observed_at_s", onset))
        _primary_start, _primary_end, onset_start, onset_end = event_window(event)
        key = (event, round(onset, 10))
        return bool(
            onset >= ownership_floor
            and observed >= ownership_floor
            and observed <= confirmation_limit + NUMERIC_TOLERANCE
            and observed < end
            and (
                key in accepted_candidate_keys
                or onset_start - NUMERIC_TOLERANCE
                <= onset
                <= onset_end + NUMERIC_TOLERANCE
            )
        )

    candidates = [
        dict(item)
        for item in full_replay["candidates"]
        if candidate_in_scope(item)
    ]
    invalid = [
        dict(item)
        for item in full_replay["invalid_steps"]
        if ownership_floor
        <= float(item.get("observed_at_s", -math.inf))
        <= confirmation_limit
    ]
    local = {
        "heel_contact": np.asarray(full_replay["heel_contact"])[mask],
        "toe_contact": np.asarray(full_replay["toe_contact"])[mask],
        "state_id": np.asarray(full_replay["state_id"])[mask],
        "accepted": accepted,
        "candidates": candidates,
        "sensor_edges": [
            dict(item)
            for item in full_replay.get("sensor_edges", [])
            if float(item.get("observed_at_s", -math.inf))
            >= float(full_times[mask][0])
            and float(item.get("observed_at_s", math.inf))
            <= float(full_times[mask][-1])
        ],
        "invalid_steps": invalid,
        "continuous_parent_replay": True,
        "fsm_reset_for_plateau": False,
    }
    return mask, local


def _local_semantic_gate(
    replay: Mapping[str, Any],
    times: np.ndarray,
    loads: Mapping[str, np.ndarray],
    *,
    sensor_on_threshold_n: float,
    expected_complete_cycles: int,
) -> dict[str, Any]:
    gait_candidates = [
        item
        for item in replay["candidates"]
        if item.get("event") in {"heel_strike", "toe_off"}
    ]
    gait_accepted = [
        item
        for values in thresholds._accepted_gait_transitions_for_gate(replay["accepted"]).values()
        for item in values
    ]
    unmatched = [item for item in gait_accepted if not item.get("matched_sensor_candidate")]
    candidate_keys = {
        (str(item["event"]), round(float(item["time"]), 10))
        for item in gait_candidates
    }
    accepted_keys = {
        (str(item["event"]), round(float(item["event_time_s"]), 10))
        for item in gait_accepted
    }
    unaccepted = sorted(candidate_keys - accepted_keys)
    heel_without_load: list[dict[str, float]] = []
    for item in gait_candidates:
        if item.get("event") != "heel_strike":
            continue
        event_time = float(item["time"])
        index = int(np.argmin(np.abs(times - event_time)))
        load = float(loads["left_heel"][index])
        if load + NUMERIC_TOLERANCE < sensor_on_threshold_n:
            heel_without_load.append({"event_time_s": event_time, "heel_load_n": load})
    timeout = [item for item in replay["accepted"] if item.get("event") == "timeout"]
    cycles = _local_valid_cycle_count(replay["accepted"])
    checks = {
        "no_invalid": not replay["invalid_steps"],
        "no_timeout": not timeout,
        "all_transitions_match_candidates": not unmatched,
        "all_candidates_accepted": not unaccepted,
        "heel_threshold_evidence": not heel_without_load,
        "exact_local_valid_cycles": cycles == int(expected_complete_cycles),
    }
    return {
        "ok": bool(all(checks.values())),
        "checks": checks,
        "timeout_transitions": timeout,
        "unmatched_accepted_transitions": unmatched,
        "unaccepted_sensor_gait_events": [
            {"event": event, "event_time_s": time_s} for event, time_s in unaccepted
        ],
        "heel_strikes_without_heel_on_threshold": heel_without_load,
        "observed_valid_cycles": cycles,
    }


def _causal_clearance_from_replay(
    times: np.ndarray,
    toe_contact: np.ndarray,
    accepted: Sequence[Mapping[str, Any]],
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    transitions = thresholds._accepted_gait_transitions_for_gate(accepted)
    heels = transitions["heel_strike"]
    cycles: list[dict[str, Any]] = []
    for toe_event in transitions["toe_off"]:
        confirmed_to = float(toe_event["confirmed_time_s"])
        next_heel = next(
            (
                item
                for item in heels
                if float(item["event_time_s"]) > confirmed_to + NUMERIC_TOLERANCE
            ),
            None,
        )
        if next_heel is None:
            continue
        heel_onset = float(next_heel["event_time_s"])
        indices = np.flatnonzero(
            (times >= confirmed_to - NUMERIC_TOLERANCE)
            & (times < heel_onset - NUMERIC_TOLERANCE)
        )
        active = indices[np.asarray(toe_contact, dtype=float)[indices] > 0.5]
        first_clear = (
            float(times[active[-1]]) + sample_dt_s if active.size else confirmed_to
        )
        cycles.append(
            {
                "accepted_to_confirmed_s": confirmed_to,
                "next_accepted_hs_onset_s": heel_onset,
                "first_guaranteed_toe_latch_clear_s": first_clear,
                "causal_clear_margin_s": heel_onset - first_clear,
            }
        )
    margins = [float(item["causal_clear_margin_s"]) for item in cycles]
    return {
        "interval_count": len(cycles),
        "minimum_causal_clear_s": min(margins) if margins else -1.0,
        "cycles": cycles,
    }


def _numeric_summary(values: Sequence[float]) -> dict[str, Any]:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return {"count": 0, "mean": None, "minimum": None, "maximum": None, "max_abs": None}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "minimum": float(np.min(array)),
        "maximum": float(np.max(array)),
        "max_abs": float(np.max(np.abs(array))),
    }


def causal_event_decomposition(
    reference_events: Mapping[str, np.ndarray],
    accepted: Sequence[Mapping[str, Any]],
    *,
    sample_dt_s: float,
    dwell_s: float,
    hs_tolerance_s: float,
    toe_off_tolerance_s: float,
) -> dict[str, Any]:
    transitions = thresholds._accepted_gait_transitions_for_gate(accepted)
    expected_cycles = len(np.asarray(reference_events["toe_off"], dtype=float))
    exact_global_order = thresholds._exact_event_order(
        accepted, expected_cycles, time_field="confirmed_time_s"
    )
    result: dict[str, Any] = {}
    for event in ("heel_strike", "toe_off"):
        reference = np.asarray(reference_events[event], dtype=float)
        items = transitions[event]
        onset_errors: list[float] = []
        latencies: list[float] = []
        total_errors: list[float] = []
        residuals: list[float] = []
        event_records: list[dict[str, float]] = []
        tolerance = (
            float(hs_tolerance_s)
            if event == "heel_strike"
            else float(toe_off_tolerance_s)
        )
        confirmed_values = np.asarray(
            [float(item["confirmed_time_s"]) for item in items], dtype=float
        )
        matching = thresholds.match_events(
            reference, confirmed_values, tolerance
        )
        alignment_complete = bool(
            exact_global_order and len(items) == len(reference)
            and int(matching["matched_count"]) == len(reference)
        )
        if alignment_complete:
            for expected, item in zip(reference, items):
                onset = float(item["event_time_s"])
                confirmed = float(item["confirmed_time_s"])
                latency = confirmed - onset
                onset_errors.append(onset - float(expected))
                latencies.append(latency)
                total_errors.append(confirmed - float(expected))
                residuals.append(latency - dwell_s)
                event_records.append(
                    {
                        "reference_s": float(expected),
                        "onset_s": onset,
                        "confirmed_s": confirmed,
                        "onset_error_s": onset - float(expected),
                        "confirmation_latency_s": latency,
                        "total_error_s": confirmed - float(expected),
                        "residual_quantization_or_routing_s": latency - dwell_s,
                    }
                )
        result[event] = {
            "alignment_complete": alignment_complete,
            "events": event_records,
            "expected_confirmation_latency_range_s": [dwell_s, dwell_s + sample_dt_s],
            "onset_error_s": _numeric_summary(onset_errors),
            "confirmation_latency_s": _numeric_summary(latencies),
            "total_error_s": _numeric_summary(total_errors),
            "residual_quantization_or_routing_s": _numeric_summary(residuals),
        }
    return result


def evaluate_continuous_candidate(
    protocol: Mapping[str, Any],
    base: Any,
    candidate: Any,
    bundle: TrialCadenceBundle,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Run one full-trial FSM, then score its four local plateau views."""

    profile, pair = _pair_for_candidate(base, bundle.plan, candidate)
    loads, penetrations, aggregate = v1._contact_inputs(
        profile, pair, bundle.samples
    )
    runtime_cfg = replace(
        v1._current_runtime_fsm_config(),
        sensor_on_threshold_n=float(protocol["replay"]["sensor_on_threshold_n"]),
        sensor_off_threshold_n=float(protocol["replay"]["sensor_off_threshold_n"]),
        sensor_dwell_s=float(protocol["replay"]["sensor_dwell_s"]),
    )
    primary_union = np.asarray(bundle.shared["primary_penetration"], dtype=float)
    full_replay = v1._run_production_fsm(
        np.asarray(bundle.shared["times"], dtype=float),
        dict(loads),
        {"left_heel": primary_union, "left_toe": np.zeros_like(primary_union)},
        np.asarray(bundle.shared["primary_aggregate"], dtype=float),
        dict(bundle.shared["kinematics"]),
        body_weight_n=float(bundle.shared["body_weight_n"]),
        fsm_config=runtime_cfg,
    )
    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    full_times = np.asarray(bundle.shared["times"], dtype=float)
    for reference in bundle.plateau_references:
        mask, replay = _slice_continuous_replay(
            full_replay,
            full_times,
            reference,
            sample_dt_s=bundle.sample_dt_s,
            sensor_dwell_s=float(runtime_cfg.sensor_dwell_s),
        )
        times = full_times[mask]
        local_loads = {key: np.asarray(value)[mask] for key, value in loads.items()}
        reference_events = reference["events"]
        primary_events = _events_for_field(replay["accepted"], "confirmed_time_s")
        onset_events = _events_for_field(replay["accepted"], "event_time_s")
        event_metrics = {
            "heel_strike": thresholds.match_events(
                reference_events["heel_strike"],
                primary_events["heel_strike"],
                float(protocol["replay"]["hs_tolerance_s"]),
            ),
            "toe_off": thresholds.match_events(
                reference_events["toe_off"],
                primary_events["toe_off"],
                float(protocol["replay"]["toe_off_tolerance_s"]),
            ),
        }
        ordered = {
            "heel_strike": thresholds._ordered_event_diagnostic(
                reference_events["heel_strike"],
                primary_events["heel_strike"],
                float(protocol["replay"]["hs_tolerance_s"]),
            ),
            "toe_off": thresholds._ordered_event_diagnostic(
                reference_events["toe_off"],
                primary_events["toe_off"],
                float(protocol["replay"]["toe_off_tolerance_s"]),
            ),
        }
        onset_ordered = {
            "heel_strike": thresholds._ordered_event_diagnostic(
                reference_events["heel_strike"],
                onset_events["heel_strike"],
                float(protocol["replay"]["hs_tolerance_s"]),
            ),
            "toe_off": thresholds._ordered_event_diagnostic(
                reference_events["toe_off"],
                onset_events["toe_off"],
                float(protocol["replay"]["toe_off_tolerance_s"]),
            ),
        }
        semantic = _local_semantic_gate(
            replay,
            times,
            local_loads,
            sensor_on_threshold_n=float(runtime_cfg.sensor_on_threshold_n),
            expected_complete_cycles=len(reference_events["toe_off"]),
        )
        phase = thresholds._phase_classification_gate(
            times,
            np.asarray(bundle.shared["prescribed_vertical_n"])[mask],
            reference_events,
            primary_events,
            replay,
            prescribed_threshold_n=float(
                protocol["replay"]["prescribed_contact_threshold_n"]
            ),
            hs_tolerance_s=float(protocol["replay"]["hs_tolerance_s"]),
            to_tolerance_s=float(protocol["replay"]["toe_off_tolerance_s"]),
            sensor_dwell_s=float(runtime_cfg.sensor_dwell_s),
            reference_phase_mode="validated_event_intervals",
            primary_event_time_field="confirmed_time_s",
            onset_events=onset_events,
        )
        arrays = phase["_arrays"]
        strict_mask = np.asarray(arrays["strict_mask"], dtype=bool)
        reference_contact = (
            np.asarray(arrays["reference_phase"]) == thresholds.PHASE_STANCE
        )
        fsm_contact = (
            thresholds._fsm_phase_from_state_id(replay["state_id"])
            == thresholds.PHASE_STANCE
        )
        fsm_metrics = thresholds.binary_metrics(
            reference_contact[strict_mask], fsm_contact[strict_mask]
        )
        transfer = v6.v4.v3.heel_to_forefoot_transfer_diagnostics(
            times, replay["heel_contact"], replay["toe_contact"], reference_events
        )
        early_to = v6.v4.v3.early_to_candidate_diagnostics(
            replay["candidates"],
            minimum_stance_duration_s=float(runtime_cfg.min_stance_duration_s),
        )
        causal = _causal_clearance_from_replay(
            times,
            replay["toe_contact"],
            replay["accepted"],
            sample_dt_s=bundle.sample_dt_s,
        )
        latency = thresholds._confirmation_latency_gate(
            replay["accepted"],
            dwell_s=float(runtime_cfg.sensor_dwell_s),
            sample_dt_s=bundle.sample_dt_s,
        )
        decomposition = causal_event_decomposition(
            reference_events,
            replay["accepted"],
            sample_dt_s=bundle.sample_dt_s,
            dwell_s=float(runtime_cfg.sensor_dwell_s),
            hs_tolerance_s=float(protocol["replay"]["hs_tolerance_s"]),
            toe_off_tolerance_s=float(
                protocol["replay"]["toe_off_tolerance_s"]
            ),
        )
        hs_max_raw = float(ordered["heel_strike"]["timing_max_abs_s"])
        to_max_raw = float(ordered["toe_off"]["timing_max_abs_s"])
        hs_max = hs_max_raw if math.isfinite(hs_max_raw) else 999.0
        to_max = to_max_raw if math.isfinite(to_max_raw) else 999.0
        hs_errors = np.abs(np.asarray(ordered["heel_strike"]["ordered_errors_s"], dtype=float))
        to_errors = np.abs(np.asarray(ordered["toe_off"]["ordered_errors_s"], dtype=float))
        cell_mean = (
            0.5
            * (
                float(np.mean(hs_errors)) / 0.05
                + float(np.mean(to_errors)) / 0.08
            )
            if hs_errors.size and to_errors.size
            else 999.0
        )
        row = {
            "candidate_id": candidate.candidate_id,
            "selectable": bool(candidate.selectable),
            "trial_id": bundle.trial_id,
            "plateau_index": int(reference["plateau_index"]),
            "speed_mps": float(reference["speed_mps"]),
            "cadence": bundle.cadence_label,
            "sample_dt_s": bundle.sample_dt_s,
            "parameter_arm": candidate.geometry["parameter_arm"],
            "heel_x_shift_from_v9_mm": candidate.geometry[
                "heel_x_shift_from_v9_mm"
            ],
            "heel_radius_reduction_from_v13_mm": candidate.geometry[
                "heel_radius_reduction_from_v13_mm"
            ],
            "toe_down_from_v9_mm": candidate.geometry["toe_down_from_v9_mm"],
            "toe_radius_reduction_from_v13_mm": candidate.geometry[
                "toe_radius_reduction_from_v13_mm"
            ],
            "heel_radius_m": candidate.geometry["heel_radius_m"],
            "toe_radius_m": candidate.geometry["toe_radius_m"],
            "geometry_displacement_from_v13_m": candidate.geometry[
                "geometry_displacement_from_v13_m"
            ],
            "expected_reference_hs_count": reference["dynamic_gate"][
                "require_exact_reference_counts"
            ]["heel_strike"],
            "expected_reference_to_count": reference["dynamic_gate"][
                "require_exact_reference_counts"
            ]["toe_off"],
            "expected_valid_cycle_count": reference["dynamic_gate"][
                "require_exact_valid_cycles"
            ],
            "reference_hs_count": len(reference_events["heel_strike"]),
            "reference_to_count": len(reference_events["toe_off"]),
            "predicted_hs_count": len(primary_events["heel_strike"]),
            "predicted_to_count": len(primary_events["toe_off"]),
            "event_matched_hs_count": int(event_metrics["heel_strike"]["matched_count"]),
            "event_matched_to_count": int(event_metrics["toe_off"]["matched_count"]),
            "observed_valid_cycle_count": int(semantic["observed_valid_cycles"]),
            "precision": float(
                min(event_metrics["heel_strike"]["precision"], event_metrics["toe_off"]["precision"])
            ),
            "recall": float(
                min(event_metrics["heel_strike"]["recall"], event_metrics["toe_off"]["recall"])
            ),
            "max_abs_hs_error_s": hs_max,
            "max_abs_toe_off_error_s": to_max,
            "diagnostic_onset_max_abs_hs_error_s": float(
                onset_ordered["heel_strike"]["timing_max_abs_s"]
            ),
            "diagnostic_onset_max_abs_toe_off_error_s": float(
                onset_ordered["toe_off"]["timing_max_abs_s"]
            ),
            "cell_mean_normalized_error": cell_mean,
            "confirmed_fsm_stance_f1": float(fsm_metrics["f1"]),
            "confirmed_fsm_stance_iou": float(fsm_metrics["iou"]),
            "fsm_true_positive_samples": int(fsm_metrics["true_positive"]),
            "fsm_false_positive_samples": int(fsm_metrics["false_positive"]),
            "fsm_false_negative_samples": int(fsm_metrics["false_negative"]),
            "fsm_true_negative_samples": int(fsm_metrics["true_negative"]),
            "transfer_both_latches_off_sample_count": int(transfer["both_latches_off_sample_count"]),
            "incomplete_heel_to_forefoot_transfer_count": int(transfer["incomplete_transfer_cycle_count"]),
            "to_candidates_before_min_stance_count": int(early_to["early_toe_off_candidate_count"]),
            "invalid_or_timeout_transition_count": int(
                len(replay["invalid_steps"]) + len(semantic["timeout_transitions"])
            ),
            "unaccepted_sensor_gait_event_count": int(
                len(semantic["unaccepted_sensor_gait_events"])
            ),
            "forbidden_phase_mismatch_count": int(phase["forbidden_mismatch_count"]),
            "unknown_fsm_phase_samples": int(
                phase["settled_outside_transition_windows_agreement"]["unknown_fsm_samples"]
            ),
            "minimum_causal_toe_clear_before_next_hs_onset_s": float(causal["minimum_causal_clear_s"]),
            "causal_swing_interval_count": int(causal["interval_count"]),
            "confirmation_latency_in_range": bool(latency["ok"]),
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": bool(
                thresholds._exact_event_order(
                    replay["accepted"],
                    len(reference_events["toe_off"]),
                    time_field="confirmed_time_s",
                )
            ),
            "mesh_geometry_pre_gate_ok": bool(candidate.geometry["pre_gate_ok"]),
            "unit_metrics_are_local_not_whole_trial": True,
            "continuous_parent_fsm_replay": True,
            "fsm_reset_for_plateau": False,
        }
        for event, event_payload in decomposition.items():
            row[f"{event}_causal_decomposition_alignment_complete"] = bool(
                event_payload["alignment_complete"]
            )
            for component, summary in event_payload.items():
                if not component.endswith("_s") or not isinstance(summary, Mapping):
                    continue
                for statistic, value in summary.items():
                    row[f"{event}_{component}_{statistic}"] = value
        finite = _finite_priority(row)
        gate = v10.strict_gate(row, reference["dynamic_gate"])
        gate = {
            **gate,
            "checks": {
                **gate["checks"],
                "priority_metrics_finite": finite,
                "local_semantic_gate": bool(semantic["ok"]),
                "local_phase_gate": bool(phase["ok"]),
            },
            "ok": bool(gate["ok"] and finite and semantic["ok"] and phase["ok"]),
        }
        row["unit_full_gate_ok"] = bool(gate["ok"])
        row["v14_stage_unit"] = _unit_key(
            bundle.trial_id, int(reference["plateau_index"]), bundle.cadence_label
        )
        rows.append(row)
        details.append(
            {
                "row": row,
                "reference": reference,
                "dynamic_gate_result": gate,
                "event_metrics": event_metrics,
                "confirmed_timing": ordered,
                "onset_timing": onset_ordered,
                "semantic": semantic,
                "phase": {key: value for key, value in phase.items() if key != "_arrays"},
                "transfer": transfer,
                "early_to_candidates": early_to,
                "confirmation_latency": latency,
                "causal_event_decomposition": decomposition,
                "causal_clearance": causal,
                "continuous_parent_replay": True,
            }
        )
    return rows, details


def _rows_by_candidate(
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, dict[str, Mapping[str, Any]]]:
    result: dict[str, dict[str, Mapping[str, Any]]] = {}
    for row in rows:
        candidate_id = str(row["candidate_id"])
        unit = str(row["v14_stage_unit"])
        if unit in result.setdefault(candidate_id, {}):
            raise ProtocolError(f"duplicate row for {candidate_id} / {unit}")
        result[candidate_id][unit] = row
    units = {frozenset(by_unit) for by_unit in result.values()}
    if len(units) > 1:
        raise ProtocolError("candidate unit universes differ")
    return result


def minimax_vector(rows: Sequence[Mapping[str, Any]]) -> dict[str, float]:
    if not rows or not all(_finite_priority(row) for row in rows):
        return {
            "worst_joint_normalized": float("inf"),
            "worst_normalized_hs": float("inf"),
            "worst_normalized_to": float("inf"),
            "worst_hs_s": float("inf"),
            "worst_to_s": float("inf"),
            "equal_cell_mean_normalized_error": float("inf"),
            "worst_f1_deficit": float("inf"),
            "worst_iou_deficit": float("inf"),
        }
    worst_hs = max(float(row["max_abs_hs_error_s"]) for row in rows)
    worst_to = max(float(row["max_abs_toe_off_error_s"]) for row in rows)
    normalized_hs = worst_hs / float(_fixed_gate_contract()["max_abs_hs_error_s"])
    normalized_to = worst_to / float(
        _fixed_gate_contract()["max_abs_toe_off_error_s"]
    )
    vector = {
        "worst_joint_normalized": max(normalized_hs, normalized_to),
        "worst_normalized_hs": normalized_hs,
        "worst_normalized_to": normalized_to,
        "worst_hs_s": worst_hs,
        "worst_to_s": worst_to,
        "equal_cell_mean_normalized_error": float(
            np.mean(
                [
                    float(row["cell_mean_normalized_error"])
                    for row in rows
                ]
            )
        ),
        "worst_f1_deficit": max(
            1.0 - float(row["confirmed_fsm_stance_f1"]) for row in rows
        ),
        "worst_iou_deficit": max(
            1.0 - float(row["confirmed_fsm_stance_iou"]) for row in rows
        ),
    }
    if not all(math.isfinite(float(value)) for value in vector.values()):
        return {key: float("inf") for key in vector}
    return vector


def pareto_challenge(
    challenger: Mapping[str, float],
    incumbent: Mapping[str, float],
    *,
    tolerance: float = PARETO_TOLERANCE,
) -> dict[str, Any]:
    keys = (
        "worst_joint_normalized",
        "worst_hs_s",
        "worst_to_s",
        "equal_cell_mean_normalized_error",
        "worst_f1_deficit",
        "worst_iou_deficit",
    )
    finite = all(
        math.isfinite(float(challenger[key]))
        and math.isfinite(float(incumbent[key]))
        for key in keys
    )
    noninferior = bool(
        finite
        and all(
            float(challenger[key]) <= float(incumbent[key]) + tolerance
            for key in keys
        )
    )
    improved = [
        key for key in keys
        if finite
        and float(challenger[key]) < float(incumbent[key]) - tolerance
    ]
    return {
        "ok": bool(noninferior and improved),
        "finite": finite,
        "pareto_noninferior": noninferior,
        "strictly_improved_components": improved,
        "tolerance": tolerance,
        "v13_wins_tie": True,
    }


def _structural_burden(rows: Sequence[Mapping[str, Any]]) -> dict[str, float]:
    event_deficits = [
        abs(int(row["predicted_hs_count"]) - int(row["expected_reference_hs_count"]))
        + abs(int(row["predicted_to_count"]) - int(row["expected_reference_to_count"]))
        for row in rows
    ]
    valid_deficits = [
        abs(int(row["observed_valid_cycle_count"]) - int(row["expected_valid_cycle_count"]))
        for row in rows
    ]
    return {
        "worst_event_count_deficit": float(max(event_deficits, default=10**9)),
        "sum_event_count_deficit": float(sum(event_deficits)),
        "worst_valid_cycle_deficit": float(max(valid_deficits, default=10**9)),
        "worst_incomplete_transfer_count": float(
            max(
                (int(row["incomplete_heel_to_forefoot_transfer_count"]) for row in rows),
                default=10**9,
            )
        ),
        "worst_unknown_phase_samples": float(
            max((int(row["unknown_fsm_phase_samples"]) for row in rows), default=10**9)
        ),
        "worst_invalid_plus_unaccepted_count": float(
            max(
                (
                    int(row["invalid_or_timeout_transition_count"])
                    + int(row["unaccepted_sensor_gait_event_count"])
                    for row in rows
                ),
                default=10**9,
            )
        ),
        "worst_forbidden_phase_mismatch_count": float(
            max(
                (int(row["forbidden_phase_mismatch_count"]) for row in rows),
                default=10**9,
            )
        ),
    }


def root_safe_isolated(
    candidate_rows: Sequence[Mapping[str, Any]],
    baseline_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    candidate_by_unit = {str(row["v14_stage_unit"]): row for row in candidate_rows}
    baseline_by_unit = {str(row["v14_stage_unit"]): row for row in baseline_rows}
    if set(candidate_by_unit) != set(baseline_by_unit):
        raise ProtocolError("root-safety unit universes differ")
    unit_checks: dict[str, Any] = {}
    for unit in sorted(candidate_by_unit):
        row = candidate_by_unit[unit]
        base = baseline_by_unit[unit]
        ref_hs = int(row["expected_reference_hs_count"])
        ref_to = int(row["expected_reference_to_count"])
        ref_cycles = int(row["expected_valid_cycle_count"])
        event_ok = all(
            abs(int(row[predicted]) - reference)
            <= max(abs(int(base[predicted]) - reference), 1)
            for predicted, reference in (
                ("predicted_hs_count", ref_hs),
                ("predicted_to_count", ref_to),
            )
        )
        checks = {
            "finite": _finite_priority(row),
            "mesh": bool(row["mesh_geometry_pre_gate_ok"]),
            "positive_radii": float(row["heel_radius_m"]) > 0.0
            and float(row["toe_radius_m"]) > 0.0,
            "exact_reference_counts": int(row["reference_hs_count"]) == ref_hs
            and int(row["reference_to_count"]) == ref_to,
            "bounded_event_count_loss": event_ok,
            "bounded_valid_cycle_loss": abs(
                int(row["observed_valid_cycle_count"]) - ref_cycles
            )
            <= max(abs(int(base["observed_valid_cycle_count"]) - ref_cycles), 1),
            "incomplete_not_worse_than_v13": int(
                row["incomplete_heel_to_forefoot_transfer_count"]
            )
            <= int(base["incomplete_heel_to_forefoot_transfer_count"]),
            "unknown_not_worse_than_v13": int(row["unknown_fsm_phase_samples"])
            <= int(base["unknown_fsm_phase_samples"]),
            "invalid_unaccepted_at_most_v13_plus_one": (
                int(row["invalid_or_timeout_transition_count"])
                + int(row["unaccepted_sensor_gait_event_count"])
                <= int(base["invalid_or_timeout_transition_count"])
                + int(base["unaccepted_sensor_gait_event_count"])
                + 1
            ),
            "forbidden_at_most_v13_plus_one": int(
                row["forbidden_phase_mismatch_count"]
            )
            <= int(base["forbidden_phase_mismatch_count"]) + 1,
        }
        unit_checks[unit] = {"ok": bool(all(checks.values())), "checks": checks}
    return {
        "ok": bool(unit_checks and all(item["ok"] for item in unit_checks.values())),
        "full_gate_required": False,
        "units": unit_checks,
    }


def _arm_rank_key(
    candidate_id: str,
    arm: str,
    rows: Sequence[Mapping[str, Any]],
    displacement_m: float,
) -> tuple[Any, ...]:
    burden = _structural_burden(rows)
    vector = minimax_vector(rows)
    common = (
        burden["worst_event_count_deficit"],
        burden["sum_event_count_deficit"],
        burden["worst_valid_cycle_deficit"],
        burden["worst_incomplete_transfer_count"],
        burden["worst_unknown_phase_samples"],
        burden["worst_invalid_plus_unaccepted_count"],
        burden["worst_forbidden_phase_mismatch_count"],
    )
    if arm in {"Hx", "Hr"}:
        target = (
            vector["worst_normalized_hs"],
            vector["worst_normalized_to"],
            vector["worst_f1_deficit"],
        )
    else:
        target = (
            vector["worst_normalized_to"],
            burden["worst_incomplete_transfer_count"],
            vector["worst_normalized_hs"],
            vector["worst_f1_deficit"],
        )
    return (*common, *target, float(displacement_m), candidate_id)


def select_isolated_arm_winners(
    rows: Sequence[Mapping[str, Any]],
    candidates: Sequence[Any],
) -> tuple[dict[str, str], dict[str, Any]]:
    indexed = _rows_by_candidate(rows)
    if BASELINE_ID not in indexed:
        raise ProtocolError("V14 isolated rows are missing V13 baseline")
    baseline_rows = list(indexed[BASELINE_ID].values())
    by_id = {item.candidate_id: item for item in candidates}
    winners: dict[str, str] = {}
    audit: dict[str, Any] = {}
    for arm in ARM_ORDER:
        ids = sorted(
            item.candidate_id
            for item in candidates
            if item.geometry["parameter_arm"] == arm
        )
        assessments: dict[str, Any] = {}
        eligible: list[str] = []
        for candidate_id in ids:
            candidate_rows = list(indexed[candidate_id].values())
            safety = root_safe_isolated(candidate_rows, baseline_rows)
            rank = _arm_rank_key(
                candidate_id,
                arm,
                candidate_rows,
                float(by_id[candidate_id].geometry["geometry_displacement_from_v13_m"]),
            )
            assessments[candidate_id] = {
                "root_safety": safety,
                "minimax_vector": minimax_vector(candidate_rows),
                "structural_burden": _structural_burden(candidate_rows),
                "rank_key": list(rank[:-1]) + [rank[-1]],
                "full_gate_pass_is_not_required_here": True,
            }
            if safety["ok"]:
                eligible.append(candidate_id)
        eligible.sort(
            key=lambda candidate_id: _arm_rank_key(
                candidate_id,
                arm,
                list(indexed[candidate_id].values()),
                float(by_id[candidate_id].geometry["geometry_displacement_from_v13_m"]),
            )
        )
        winner = eligible[0] if eligible else BASELINE_ID
        winners[arm] = winner
        audit[arm] = {
            "winner_id": winner,
            "fallback_to_v13_value": winner == BASELINE_ID,
            "ranked_root_safe_ids": eligible,
            "assessments": assessments,
        }
    return winners, {
        "status": "DIAGNOSTIC_ROOT_SAFE_ARM_WINNERS_FROZEN",
        "winners": winners,
        "arms": audit,
        "full_gate_was_not_required_for_arm_selection": True,
    }


def build_combinations(
    protocol: Mapping[str, Any],
    arm_winners: Mapping[str, str],
    isolated_candidates: Sequence[Any],
    geometry_summary: Mapping[str, Any],
    isolated_rows: Sequence[Mapping[str, Any]],
) -> tuple[list[Any], dict[str, Any]]:
    """Build the <=3^4 frozen local Cartesian Stage-2 neighborhood.

    Existing V13/isolated tuples are returned as their original candidate
    objects so their Stage-1 rows can be reused.  Only genuinely new mixed-arm
    tuples require offline reconstruction from the already sampled stations.
    """

    if set(arm_winners) != set(ARM_ORDER):
        raise ProtocolError("V14 combination builder requires Hx/Hr/Ty/Tr")
    by_id = {item.candidate_id: item for item in isolated_candidates}
    indexed_rows = _rows_by_candidate(isolated_rows)
    baseline = by_id[BASELINE_ID]
    for winner in arm_winners.values():
        if winner not in by_id:
            raise ProtocolError(f"unknown arm winner: {winner}")
    parameter_key = {
        "Hx": "heel_x_shift_from_v9_mm",
        "Hr": "heel_radius_reduction_from_v13_mm",
        "Ty": "toe_down_from_v9_mm",
        "Tr": "toe_radius_reduction_from_v13_mm",
    }
    isolated_by_arm_value: dict[str, dict[float, Any]] = {
        arm: {float(BASELINE_ARM_VALUES[arm]): baseline} for arm in ARM_ORDER
    }
    for candidate in isolated_candidates:
        arm = str(candidate.geometry["parameter_arm"])
        if arm in ARM_ORDER:
            value = float(candidate.geometry[parameter_key[arm]])
            isolated_by_arm_value[arm][value] = candidate

    arm_sets: dict[str, tuple[float, ...]] = {}
    neighbor_audit: dict[str, Any] = {}
    for arm in ARM_ORDER:
        grid = tuple(float(value) for value in ARM_GRIDS[arm])
        winner_id = str(arm_winners[arm])
        winner = by_id[winner_id]
        winner_value = (
            float(BASELINE_ARM_VALUES[arm])
            if winner_id == BASELINE_ID
            else float(winner.geometry[parameter_key[arm]])
        )
        try:
            winner_index = next(
                index
                for index, value in enumerate(grid)
                if math.isclose(value, winner_value, abs_tol=NUMERIC_TOLERANCE)
            )
        except StopIteration as exc:
            raise ProtocolError(f"{arm} winner is outside its frozen grid") from exc
        if winner_id == BASELINE_ID:
            neighbor_index = 1
            neighbor_reason = "fallback_first_nonbaseline_neighbor"
        elif winner_index == len(grid) - 1:
            neighbor_index = winner_index - 1
            neighbor_reason = "outer_boundary_single_inward_neighbor"
        elif winner_index == 0:
            neighbor_index = 1
            neighbor_reason = "baseline_first_nonbaseline_neighbor"
        else:
            choices = (winner_index - 1, winner_index + 1)

            def neighbor_key(index: int) -> tuple[Any, ...]:
                candidate = isolated_by_arm_value[arm][grid[index]]
                rank = _arm_rank_key(
                    candidate.candidate_id,
                    arm,
                    list(indexed_rows[candidate.candidate_id].values()),
                    float(
                        candidate.geometry["geometry_displacement_from_v13_m"]
                    ),
                )
                # The same arm performance tuple is authoritative.  Exact
                # ties resolve toward V13, then by grid index and ID.
                return (
                    *rank[:-2],
                    abs(grid[index] - float(BASELINE_ARM_VALUES[arm])),
                    index,
                    candidate.candidate_id,
                )

            neighbor_index = min(choices, key=neighbor_key)
            neighbor_reason = "best_adjacent_by_same_arm_tuple"
        neighbor_value = grid[neighbor_index]
        values = tuple(
            sorted(
                {
                    float(BASELINE_ARM_VALUES[arm]),
                    float(winner_value),
                    float(neighbor_value),
                }
            )
        )
        arm_sets[arm] = values
        neighbor_candidate = isolated_by_arm_value[arm][neighbor_value]
        neighbor_audit[arm] = {
            "winner_id": winner_id,
            "winner_value": winner_value,
            "neighbor_id": neighbor_candidate.candidate_id,
            "neighbor_value": neighbor_value,
            "neighbor_reason": neighbor_reason,
            "stage2_values": list(values),
        }

    triangles = np.asarray(geometry_summary["triangles"], dtype=float)
    baseline_geometry = geometry_summary
    existing_by_parameters: dict[tuple[float, ...], Any] = {}
    for candidate in isolated_candidates:
        existing_by_parameters[tuple(
            float(candidate.geometry[parameter_key[arm]]) for arm in ARM_ORDER
        )] = candidate
    tuples = sorted(itertools.product(*(arm_sets[arm] for arm in ARM_ORDER)))
    if len(tuples) > MAXIMUM_STAGE2_CARTESIAN_TUPLES:
        raise ProtocolError("Stage-2 Cartesian neighborhood exceeds 3^4")
    pool: list[Any] = []
    reused_ids: list[str] = []
    new_ids: list[str] = []
    tuple_records: list[dict[str, Any]] = []
    for parameter_tuple in tuples:
        existing = existing_by_parameters.get(tuple(float(x) for x in parameter_tuple))
        if existing is not None:
            candidate = existing
            origin = "reused_stage1_rows"
            reused_ids.append(candidate.candidate_id)
        else:
            values = dict(zip(ARM_ORDER, parameter_tuple))
            combo_id = "S2_" + "__".join(
                f"{arm}_{_token(values[arm])}" for arm in ARM_ORDER
            )
            candidate = _make_candidate(
                candidate_id=combo_id,
                role="local_cartesian_full_gate_finalist",
                arm="combination",
                selectable=True,
                heel_x_shift_from_v9_mm=values["Hx"],
                heel_radius_reduction_from_v13_mm=values["Hr"],
                toe_down_from_v9_mm=values["Ty"],
                toe_radius_reduction_from_v13_mm=values["Tr"],
                baseline_geometry=baseline_geometry,
                triangles=triangles,
            )
            origin = "new_offline_reconstruction_zero_new_sampling"
            new_ids.append(candidate.candidate_id)
        tuple_records.append(
            {
                "parameter_tuple_Hx_Hr_Ty_Tr": list(parameter_tuple),
                "candidate_id": candidate.candidate_id,
                "origin": origin,
            }
        )
        if candidate.candidate_id != BASELINE_ID:
            pool.append(candidate)

    if len({_geometry_tuple(item) for item in pool}) != len(pool):
        raise ProtocolError("Stage-2 geometry deduplication failed")
    if len({item.candidate_id for item in pool}) != len(pool):
        raise ProtocolError("Stage-2 candidate IDs are not unique")
    return pool, {
        "arm_winner_ids": dict(arm_winners),
        "arm_sets_and_neighbors": neighbor_audit,
        "canonical_axis_order": list(ARM_ORDER),
        "cartesian_tuple_count_including_v13": len(tuples),
        "finalist_pool_count_excluding_v13": len(pool),
        "reused_stage1_candidate_ids": sorted(set(reused_ids)),
        "new_candidate_ids": sorted(new_ids),
        "tuple_records": tuple_records,
        "new_opensim_station_sampling_passes": 0,
        "claim_limit": "best_in_staged_local_neighborhood_not_global_4d_minimum",
    }


def _full_gate_all(rows: Sequence[Mapping[str, Any]]) -> bool:
    return bool(
        rows
        and all(bool(row.get("unit_full_gate_ok")) for row in rows)
        and all(bool(row.get("trial_aggregate_gate_ok")) for row in rows)
    )


def boundary_saturation_stop(
    finalist: Any,
    isolated_rows: Sequence[Mapping[str, Any]],
    isolated_candidates: Sequence[Any],
) -> dict[str, Any]:
    """Stop before validation when a selected outer arm is still improving."""

    parameter_key = {
        "Hx": "heel_x_shift_from_v9_mm",
        "Hr": "heel_radius_reduction_from_v13_mm",
        "Ty": "toe_down_from_v9_mm",
        "Tr": "toe_radius_reduction_from_v13_mm",
    }
    indexed = _rows_by_candidate(isolated_rows)
    baseline = next(
        item for item in isolated_candidates if item.candidate_id == BASELINE_ID
    )
    by_arm_value: dict[tuple[str, float], Any] = {
        (arm, float(BASELINE_ARM_VALUES[arm])): baseline for arm in ARM_ORDER
    }
    for candidate in isolated_candidates:
        arm = str(candidate.geometry["parameter_arm"])
        if arm in ARM_ORDER:
            by_arm_value[(arm, float(candidate.geometry[parameter_key[arm]]))] = candidate
    assessments: dict[str, Any] = {}
    stop_arms: list[str] = []
    for arm in ARM_ORDER:
        value = float(finalist.geometry[parameter_key[arm]])
        outer = float(OUTER_BOUNDARY_VALUES[arm])
        at_outer = math.isclose(value, outer, abs_tol=NUMERIC_TOLERANCE)
        if not at_outer:
            assessments[arm] = {"at_outer_boundary": False, "stop": False}
            continue
        grid = tuple(float(item) for item in ARM_GRIDS[arm])
        inner_value = grid[-2]
        outer_candidate = by_arm_value[(arm, outer)]
        inner_candidate = by_arm_value[(arm, inner_value)]
        outer_key = _arm_rank_key(
            outer_candidate.candidate_id,
            arm,
            list(indexed[outer_candidate.candidate_id].values()),
            float(outer_candidate.geometry["geometry_displacement_from_v13_m"]),
        )
        inner_key = _arm_rank_key(
            inner_candidate.candidate_id,
            arm,
            list(indexed[inner_candidate.candidate_id].values()),
            float(inner_candidate.geometry["geometry_displacement_from_v13_m"]),
        )
        improves_toward_outer = bool(outer_key < inner_key)
        if improves_toward_outer:
            stop_arms.append(arm)
        assessments[arm] = {
            "at_outer_boundary": True,
            "outer_value": outer,
            "immediate_inner_value": inner_value,
            "outer_candidate_id": outer_candidate.candidate_id,
            "inner_candidate_id": inner_candidate.candidate_id,
            "isolated_rank_improves_toward_outer": improves_toward_outer,
            "stop": improves_toward_outer,
        }
    stop = bool(stop_arms)
    return {
        "stop_before_validation": stop,
        "status": (
            "STOP_REQUIRES_PREREGISTERED_V14X_DEV_EXTENSION"
            if stop
            else "NO_OUTER_BOUNDARY_SATURATION_STOP"
        ),
        "stop_arms": stop_arms,
        "arms": assessments,
        "v13_remains_baseline_if_stopped": True,
        "global_minimum_claim_allowed": False,
    }


def _finalist_rank_key(
    candidate_id: str,
    vector: Mapping[str, float],
    displacement_m: float,
) -> tuple[Any, ...]:
    return (
        float(vector["worst_joint_normalized"]),
        float(vector["worst_hs_s"]),
        float(vector["worst_to_s"]),
        float(vector["equal_cell_mean_normalized_error"]),
        float(vector["worst_f1_deficit"]),
        float(vector["worst_iou_deficit"]),
        float(displacement_m),
        candidate_id,
    )


def select_development_finalist(
    rows: Sequence[Mapping[str, Any]],
    combinations: Sequence[Any],
) -> tuple[str | None, dict[str, Any]]:
    indexed = _rows_by_candidate(rows)
    if BASELINE_ID not in indexed:
        raise ProtocolError("development finalist rows are missing V13")
    baseline_rows = list(indexed[BASELINE_ID].values())
    if len(baseline_rows) != 24:
        raise ProtocolError("development finalist selection requires 24 cells")
    baseline_vector = minimax_vector(baseline_rows)
    by_id = {item.candidate_id: item for item in combinations}
    eligible: list[str] = []
    assessments: dict[str, Any] = {}
    for candidate_id in sorted(by_id):
        candidate_rows = list(indexed[candidate_id].values())
        if len(candidate_rows) != 24:
            raise ProtocolError(f"{candidate_id} does not have 24 development cells")
        vector = minimax_vector(candidate_rows)
        pareto = pareto_challenge(vector, baseline_vector)
        full_gate = _full_gate_all(candidate_rows)
        assessments[candidate_id] = {
            "passes_every_development_unit": full_gate,
            "minimax_vector": vector,
            "pareto_vs_v13": pareto,
            "eligible": bool(full_gate and pareto["ok"]),
        }
        if full_gate and pareto["ok"]:
            eligible.append(candidate_id)
    eligible.sort(
        key=lambda candidate_id: _finalist_rank_key(
            candidate_id,
            assessments[candidate_id]["minimax_vector"],
            float(by_id[candidate_id].geometry["geometry_displacement_from_v13_m"]),
        )
    )
    winner = eligible[0] if eligible else None
    return winner, {
        "status": (
            "FROZEN_DEVELOPMENT_FINALIST"
            if winner is not None
            else "NO_FULL_GATE_PARETO_DEVELOPMENT_FINALIST"
        ),
        "finalist_id": winner,
        "baseline_id": BASELINE_ID,
        "baseline_minimax_vector": baseline_vector,
        "ranked_eligible_ids": eligible,
        "assessments": assessments,
        "v13_wins_tie": True,
    }


def paired_holdout_decision(
    rows: Sequence[Mapping[str, Any]], primary_id: str, *, stage: str
) -> dict[str, Any]:
    if stage not in {"validation", "sealed"}:
        raise ProtocolError("paired holdout decision stage is invalid")
    indexed = _rows_by_candidate(rows)
    if set(indexed) != {BASELINE_ID, primary_id}:
        raise ProtocolError(f"{stage} must contain primary + V13 only")
    primary_rows = list(indexed[primary_id].values())
    baseline_rows = list(indexed[BASELINE_ID].values())
    if len(primary_rows) != 8 or len(baseline_rows) != 8:
        raise ProtocolError(f"{stage} requires four plateaus x two cadences")
    primary_vector = minimax_vector(primary_rows)
    baseline_vector = minimax_vector(baseline_rows)
    full_gate = _full_gate_all(primary_rows)
    performance_keys = (
        "worst_joint_normalized",
        "worst_hs_s",
        "worst_to_s",
        "equal_cell_mean_normalized_error",
        "worst_f1_deficit",
        "worst_iou_deficit",
    )
    paired_primary = tuple(float(primary_vector[key]) for key in performance_keys)
    paired_baseline = tuple(float(baseline_vector[key]) for key in performance_keys)
    first_material_difference: int | None = None
    for index, (primary_value, baseline_value) in enumerate(
        zip(paired_primary, paired_baseline)
    ):
        if abs(primary_value - baseline_value) > PARETO_TOLERANCE:
            first_material_difference = index
            break
    paired_ahead = bool(
        first_material_difference is not None
        and paired_primary[first_material_difference]
        < paired_baseline[first_material_difference] - PARETO_TOLERANCE
    )
    pareto = pareto_challenge(primary_vector, baseline_vector)
    ok = bool(full_gate and pareto["ok"])
    return {
        "stage": stage,
        "status": f"{stage.upper()}_PRIMARY_PASS" if ok else f"{stage.upper()}_PRIMARY_FAIL",
        "ok": ok,
        "primary_candidate_id": primary_id,
        "paired_comparator_id": BASELINE_ID,
        "primary_passes_every_unit": full_gate,
        "primary_minimax_vector": primary_vector,
        "v13_minimax_vector": baseline_vector,
        "paired_selector_tuple_keys": list(performance_keys),
        "primary_paired_selector_tuple": list(paired_primary),
        "v13_paired_selector_tuple": list(paired_baseline),
        "primary_ahead_of_v13_beyond_epsilon": paired_ahead,
        "pareto_vs_v13": pareto,
        "paired_selector_tuple_is_diagnostic_not_a_substitute_for_pareto": True,
        "epsilon": PARETO_TOLERANCE,
        "selector_or_reselection_used": False,
        "v13_can_rescue_primary": False,
    }


COUNT_FIELDS = (
    "reference_hs_count",
    "reference_to_count",
    "predicted_hs_count",
    "predicted_to_count",
    "observed_valid_cycle_count",
    "causal_swing_interval_count",
    "transfer_both_latches_off_sample_count",
    "incomplete_heel_to_forefoot_transfer_count",
    "to_candidates_before_min_stance_count",
    "invalid_or_timeout_transition_count",
    "unaccepted_sensor_gait_event_count",
    "forbidden_phase_mismatch_count",
    "unknown_fsm_phase_samples",
    "event_matched_hs_count",
    "event_matched_to_count",
    "fsm_true_positive_samples",
    "fsm_false_positive_samples",
    "fsm_false_negative_samples",
    "fsm_true_negative_samples",
)


def aggregate_trial_counts(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Build coherent four-plateau trial gates without event/cycle pooling.

    Count/confusion fields are summed because plateau cycle sets are disjoint;
    timing uses the worst local maximum and causal clearance the worst local
    minimum.  This result is an additional eligibility gate only.  Candidate
    ranking continues to weight every plateau cell equally.
    """

    groups: dict[tuple[str, str, str], list[Mapping[str, Any]]] = {}
    for row in rows:
        key = (str(row["candidate_id"]), str(row["trial_id"]), str(row["cadence"]))
        groups.setdefault(key, []).append(row)
    aggregates: list[dict[str, Any]] = []
    for (candidate_id, trial_id, cadence), group in sorted(groups.items()):
        plateau_ids = sorted(int(row["plateau_index"]) for row in group)
        if plateau_ids != [1, 2, 3, 4]:
            raise ProtocolError(
                f"aggregate {candidate_id}/{trial_id}/{cadence} is not four plateaus"
            )
        record: dict[str, Any] = {
            "candidate_id": candidate_id,
            "trial_id": trial_id,
            "cadence": cadence,
            "plateau_count": 4,
            "aggregation": "union_of_four_disjoint_plateau_cycle_sets_ramps_excluded",
            "ranking_weight": False,
        }
        for field in COUNT_FIELDS:
            record[field] = int(sum(int(row[field]) for row in group))
        record["reference_cycle_sum_consistent"] = bool(
            record["reference_hs_count"] - 4 == record["reference_to_count"]
        )
        if not record["reference_cycle_sum_consistent"]:
            raise ProtocolError("four-plateau reference aggregate is inconsistent")
        record["predicted_cycle_sum_consistent"] = bool(
            record["predicted_hs_count"] - 4 == record["predicted_to_count"]
        )
        matched = record["event_matched_hs_count"] + record[
            "event_matched_to_count"
        ]
        predicted = record["predicted_hs_count"] + record["predicted_to_count"]
        reference = record["reference_hs_count"] + record["reference_to_count"]
        record["precision"] = float(matched / max(1, predicted))
        record["recall"] = float(matched / max(1, reference))
        tp = record["fsm_true_positive_samples"]
        fp = record["fsm_false_positive_samples"]
        fn = record["fsm_false_negative_samples"]
        fsm_precision = tp / max(1, tp + fp)
        fsm_recall = tp / max(1, tp + fn)
        record["confirmed_fsm_stance_f1"] = float(
            2.0 * fsm_precision * fsm_recall
            / max(1.0e-12, fsm_precision + fsm_recall)
        )
        record["confirmed_fsm_stance_iou"] = float(tp / max(1, tp + fp + fn))
        record["max_abs_hs_error_s"] = float(
            max(float(row["max_abs_hs_error_s"]) for row in group)
        )
        record["max_abs_toe_off_error_s"] = float(
            max(float(row["max_abs_toe_off_error_s"]) for row in group)
        )
        record["minimum_causal_toe_clear_before_next_hs_onset_s"] = float(
            min(
                float(row["minimum_causal_toe_clear_before_next_hs_onset_s"])
                for row in group
            )
        )
        record["confirmation_latency_in_range"] = bool(
            all(bool(row["confirmation_latency_in_range"]) for row in group)
        )
        record["exact_hs_to_toe_off_to_hs_order_and_cycle_count"] = bool(
            all(
                bool(row["exact_hs_to_toe_off_to_hs_order_and_cycle_count"])
                for row in group
            )
        )
        record["mesh_geometry_pre_gate_ok"] = bool(
            all(bool(row["mesh_geometry_pre_gate_ok"]) for row in group)
        )
        expected_cycles = record["reference_to_count"]
        checks = {
            "coherent_reference_union": record["reference_cycle_sum_consistent"],
            "exact_detector_counts": bool(
                record["predicted_hs_count"] == record["reference_hs_count"]
                and record["predicted_to_count"] == record["reference_to_count"]
                and record["predicted_cycle_sum_consistent"]
            ),
            "exact_valid_cycles": record["observed_valid_cycle_count"]
            == expected_cycles,
            "precision": record["precision"]
            >= float(_fixed_gate_contract()["precision"]),
            "recall": record["recall"] >= float(_fixed_gate_contract()["recall"]),
            "hs_timing": record["max_abs_hs_error_s"]
            <= float(_fixed_gate_contract()["max_abs_hs_error_s"])
            + NUMERIC_TOLERANCE,
            "toe_timing": record["max_abs_toe_off_error_s"]
            <= float(_fixed_gate_contract()["max_abs_toe_off_error_s"])
            + NUMERIC_TOLERANCE,
            "fsm_f1": record["confirmed_fsm_stance_f1"]
            >= float(_fixed_gate_contract()["minimum_confirmed_fsm_stance_f1"]),
            "fsm_iou": record["confirmed_fsm_stance_iou"]
            >= float(_fixed_gate_contract()["minimum_confirmed_fsm_stance_iou"]),
            "zero_structural_anomalies": all(
                record[field] == 0
                for field in (
                    "transfer_both_latches_off_sample_count",
                    "incomplete_heel_to_forefoot_transfer_count",
                    "to_candidates_before_min_stance_count",
                    "invalid_or_timeout_transition_count",
                    "unaccepted_sensor_gait_event_count",
                    "forbidden_phase_mismatch_count",
                    "unknown_fsm_phase_samples",
                )
            ),
            "exact_causal_intervals": record["causal_swing_interval_count"]
            == expected_cycles,
            "minimum_causal_clearance": record[
                "minimum_causal_toe_clear_before_next_hs_onset_s"
            ]
            >= float(
                _fixed_gate_contract()[
                    "minimum_causal_toe_clear_before_next_hs_onset_s"
                ]
            )
            - NUMERIC_TOLERANCE,
            "confirmation_latency": record["confirmation_latency_in_range"],
            "exact_order": record[
                "exact_hs_to_toe_off_to_hs_order_and_cycle_count"
            ],
            "mesh": record["mesh_geometry_pre_gate_ok"],
        }
        record["trial_aggregate_gate"] = {
            "ok": bool(all(checks.values())),
            "checks": checks,
        }
        record["trial_aggregate_gate_ok"] = bool(all(checks.values()))
        aggregates.append(record)
    return aggregates


def attach_trial_aggregate_gates(
    rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Attach the additional trial-level gate outcome to every local row."""

    aggregates = aggregate_trial_counts(rows)
    by_key = {
        (item["candidate_id"], item["trial_id"], item["cadence"]): item
        for item in aggregates
    }
    for row in rows:
        key = (str(row["candidate_id"]), str(row["trial_id"]), str(row["cadence"]))
        aggregate = by_key[key]
        row["trial_aggregate_gate_ok"] = bool(aggregate["trial_aggregate_gate_ok"])
        row["trial_aggregate_key"] = "::".join(key)
    return aggregates


def _verify_raw_identity(record: Mapping[str, Any]) -> Path:
    path = v1.resolve_repo_path(str(record["path"])).resolve()
    if not path.is_file():
        raise ProtocolError(f"missing raw source: {_portable_path(path)}")
    stat = path.stat()
    if int(stat.st_size) != int(record["size_bytes"]):
        raise ProtocolError(f"raw source size drifted: {_portable_path(path)}")
    if _sha256(path) != str(record["sha256"]):
        raise ProtocolError(f"raw source hash drifted: {_portable_path(path)}")
    return path


def _assert_semantic_access(
    protocol: Mapping[str, Any],
    *,
    trial_id: str,
    stage: str,
    access_receipt: Path | None,
) -> None:
    allowed = {
        "development": set(protocol["split"]["DEVELOPMENT"]),
        "validation": set(protocol["split"]["VALIDATION"]),
        "sealed": set(protocol["split"]["SEALED"]),
    }
    if stage not in allowed or trial_id not in allowed[stage]:
        raise ProtocolError(f"semantic access forbidden: stage={stage}, trial={trial_id}")
    if trial_id in set(protocol["split"]["RESERVE"]):
        raise ProtocolError("reserve semantic access is categorically forbidden")
    if stage in {"validation", "sealed"}:
        if access_receipt is None or not access_receipt.is_file():
            raise ProtocolError(f"{stage} receipt must exist before semantic decode")
        expected_receipt = access_receipt.parent / f"{stage}_access_receipt.json"
        if access_receipt.resolve() != expected_receipt.resolve():
            raise ProtocolError(f"{stage} receipt path is not canonical")
        receipt = json.loads(access_receipt.read_text(encoding="utf-8"))
        receipt_protocol = receipt.get("protocol", {})
        if not (
            receipt.get("stage") == stage
            and receipt.get("trial_id") == trial_id
            and receipt.get("semantic_access_started") is True
            and receipt_protocol.get("protocol_id") == protocol["protocol_id"]
            and receipt_protocol.get("sha256") == protocol["_protocol_sha256"]
            and v1.resolve_repo_path(
                str(receipt_protocol.get("path", ""))
            ).resolve()
            == Path(str(protocol["_protocol_path"])).resolve()
            and receipt.get("paired_comparator_id") == BASELINE_ID
            and receipt.get("evaluated_pair_count") == 2
        ):
            raise ProtocolError(f"invalid {stage} access receipt")
        primary = receipt.get("primary_candidate", {})
        primary_id = str(primary.get("candidate_id", ""))
        if not primary_id or not primary.get("record_sha256"):
            raise ProtocolError(f"{stage} receipt lacks a frozen primary")

        dev_source = receipt.get("development_candidate_lock")
        if not isinstance(dev_source, Mapping):
            raise ProtocolError(f"{stage} receipt lacks development lock")
        dev_path = v1.resolve_repo_path(str(dev_source.get("path", ""))).resolve()
        if dev_path != (access_receipt.parent / "development_candidate_lock.json").resolve():
            raise ProtocolError("development lock path is not canonical")
        if not dev_path.is_file() or _sha256(dev_path) != str(dev_source.get("sha256")):
            raise ProtocolError("development lock identity drifted")
        dev_lock = json.loads(dev_path.read_text(encoding="utf-8"))
        finalist_record = dev_lock.get("finalist_record")
        if not (
            dev_lock.get("status")
            == "DEVELOPMENT_DECISION_FROZEN_BEFORE_HOLDOUT_ACCESS"
            and dev_lock.get("protocol_sha256") == protocol["_protocol_sha256"]
            and dev_lock.get("finalist_id") == primary_id
            and isinstance(finalist_record, Mapping)
            and _canonical_sha256(finalist_record) == primary["record_sha256"]
            and dev_lock.get("validation_semantic_access_allowed") is True
            and not bool(
                dev_lock.get("boundary_saturation_stop", {}).get(
                    "stop_before_validation", True
                )
            )
        ):
            raise ProtocolError("development lock does not authorize frozen primary")

        if stage == "validation":
            if receipt.get("validation_decision_lock") is not None:
                raise ProtocolError("validation receipt cannot depend on a future decision")
        else:
            validation_source = receipt.get("validation_decision_lock")
            if not isinstance(validation_source, Mapping):
                raise ProtocolError("sealed receipt lacks validation decision lock")
            validation_path = v1.resolve_repo_path(
                str(validation_source.get("path", ""))
            ).resolve()
            if validation_path != (
                access_receipt.parent / "validation_decision_lock.json"
            ).resolve():
                raise ProtocolError("validation decision lock path is not canonical")
            if (
                not validation_path.is_file()
                or _sha256(validation_path)
                != str(validation_source.get("sha256"))
            ):
                raise ProtocolError("validation decision lock identity drifted")
            validation_lock = json.loads(
                validation_path.read_text(encoding="utf-8")
            )
            linked_dev = validation_lock.get("development_candidate_lock", {})
            if not (
                validation_lock.get("status")
                == "VALIDATION_DECISION_FROZEN_BEFORE_SEALED_ACCESS"
                and validation_lock.get("protocol_sha256")
                == protocol["_protocol_sha256"]
                and validation_lock.get("primary_candidate_id") == primary_id
                and validation_lock.get("sealed_semantic_access_allowed") is True
                and validation_lock.get("validation_decision", {}).get("ok") is True
                and linked_dev.get("sha256") == dev_source.get("sha256")
            ):
                raise ProtocolError("validation decision lock does not authorize sealed")


def prepare_trial(
    protocol: Mapping[str, Any],
    *,
    trial_id: str,
    stage: str,
    work_dir: Path,
    access_receipt: Path | None,
) -> TrialArtifacts:
    """Convert marker/FP and run IK after enforcing the role receipt boundary."""

    _assert_semantic_access(
        protocol,
        trial_id=trial_id,
        stage=stage,
        access_receipt=access_receipt,
    )
    trial = protocol["trials"][trial_id]
    # Conditions were decoded only by the metadata audit.  Recheck their live
    # byte identity here without semantically opening them.
    _verify_raw_identity(trial["raw_sources"]["conditions"])
    dataset_ik_path = _verify_raw_identity(trial["raw_sources"]["ik"])
    fp_path = _verify_raw_identity(trial["raw_sources"]["fp"])
    marker_path = _verify_raw_identity(trial["raw_sources"]["markers"])
    if work_dir.exists():
        raise NoClobberError(f"trial work directory exists: {_portable_path(work_dir)}")
    stem = f"treadmill_{trial_id}_01"
    trial_start, trial_end = (float(x) for x in trial["trial_interval_s"])
    model_file = v1.resolve_repo_path(protocol["model_file"]).resolve()
    if _sha256(model_file) != metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256:
        raise ProtocolError("marker-calibrated model hash drifted before IK")
    plugin = v1.resolve_repo_path(str(protocol["replay"]["sea_plugin"])).resolve()
    if Path(str(protocol["replay"]["sea_plugin"])).suffix:
        raise ProtocolError("SEA plugin must be an extensionless basename")
    plugin_spec = converter.resolve_plugin_spec(plugin)
    expected_plugin_hash = EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX.get(
        plugin_spec.binary.suffix.lower()
    )
    if expected_plugin_hash is None:
        raise ProtocolError(
            "this platform's SEA binary has no preregistered V14 hash"
        )
    if _sha256(plugin_spec.binary) != expected_plugin_hash:
        raise ProtocolError("SEA plugin binary identity drifted before conversion")

    # This golden converter call is the first semantic decode.  It intentionally
    # schema-checks and archives dataset IK, but only the marker-derived IK below
    # is permitted to enter SimulationSetup.
    converted = converter.convert_trial(
        ik_mat=dataset_ik_path,
        fp_mat=fp_path,
        markers_mat=marker_path,
        output_dir=work_dir,
        trial=stem,
        ik_model=model_file,
        ik_plugin=plugin,
    )
    start, end = (float(value) for value in converted["time_range_s"])
    if trial_start < start or trial_end > end:
        raise ProtocolError("frozen trial interval escapes converted time coverage")
    ik_setup = Path(str(converted["ik_setup_xml"])).resolve()
    ik_motion = work_dir / f"{stem}_ik.mot"
    converter.run_ik_from_setup(setup_xml=ik_setup, ik_plugin=plugin)
    finalized = converter.finalize_ik_receipt(
        setup_xml=ik_setup,
        output_ik_mot=ik_motion,
        ik_plugin=plugin,
    )
    ik_receipt = Path(str(finalized["receipt"])).resolve()
    if (
        not ik_receipt.is_file()
        or finalized.get("status") != "IK_OUTPUT_VERIFIED"
    ):
        raise ProtocolError("final marker-based IK receipt is missing")
    trc = Path(str(converted["trc"])).resolve()
    grf = Path(str(converted["grf_mot"])).resolve()
    external = Path(str(converted["external_loads_xml"])).resolve()
    conversion_manifest = Path(str(converted["conversion_manifest"])).resolve()
    setup = setup_io.build_simulation_setup(
        model_file=model_file,
        kinematics_file=ik_motion,
        external_loads_xml=external,
        reserve_actuators_xml=v1.resolve_repo_path(
            protocol["reserve_actuators_xml"]
        ).resolve(),
        t_start=trial_start,
        t_end=trial_end,
        grf_mode="prescribed",
    )
    return TrialArtifacts(
        trial_id=trial_id,
        stage=stage,
        setup=setup,
        work_dir=work_dir,
        trc=trc,
        grf=grf,
        external_loads=external,
        ik_setup=ik_setup,
        ik_motion=ik_motion,
        ik_receipt=ik_receipt,
        conversion_manifest=conversion_manifest,
    )


def _write_stage_access_receipt(
    output_dir: Path,
    protocol: Mapping[str, Any],
    *,
    stage: str,
    trial_id: str,
    primary: Any,
    development_candidate_lock: Path,
    validation_decision_lock: Path | None = None,
) -> Path:
    if stage not in {"validation", "sealed"}:
        raise ProtocolError("only holdout stages receive one-shot receipts")
    if not development_candidate_lock.is_file():
        raise ProtocolError("development candidate lock must precede holdout access")
    dev_lock = json.loads(
        development_candidate_lock.read_text(encoding="utf-8")
    )
    candidate_record = _candidate_record(primary)
    if not (
        dev_lock.get("status")
        == "DEVELOPMENT_DECISION_FROZEN_BEFORE_HOLDOUT_ACCESS"
        and dev_lock.get("protocol_sha256") == protocol["_protocol_sha256"]
        and dev_lock.get("finalist_id") == primary.candidate_id
        and dev_lock.get("finalist_record") == v1._json_safe(candidate_record)
        and dev_lock.get("validation_semantic_access_allowed") is True
        and not bool(
            dev_lock.get("boundary_saturation_stop", {}).get(
                "stop_before_validation", True
            )
        )
    ):
        raise ProtocolError("development lock does not authorize holdout receipt")
    if stage == "sealed" and (
        validation_decision_lock is None or not validation_decision_lock.is_file()
    ):
        raise ProtocolError("validation decision lock must precede sealed access")
    if stage == "sealed":
        assert validation_decision_lock is not None
        validation_lock = json.loads(
            validation_decision_lock.read_text(encoding="utf-8")
        )
        linked_dev = validation_lock.get("development_candidate_lock", {})
        if not (
            validation_lock.get("status")
            == "VALIDATION_DECISION_FROZEN_BEFORE_SEALED_ACCESS"
            and validation_lock.get("protocol_sha256")
            == protocol["_protocol_sha256"]
            and validation_lock.get("primary_candidate_id")
            == primary.candidate_id
            and validation_lock.get("sealed_semantic_access_allowed") is True
            and validation_lock.get("validation_decision", {}).get("ok") is True
            and linked_dev.get("sha256") == _sha256(development_candidate_lock)
        ):
            raise ProtocolError("validation decision lock does not authorize sealed")
    return _write_json_exclusive(
        output_dir / f"{stage}_access_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": f"{stage.upper()}_OPENED_FOR_SINGLE_AUTHORIZED_V14_RUN",
            "stage": stage,
            "trial_id": trial_id,
            "semantic_access_started": True,
            "protocol": {
                "path": _portable_path(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
                "protocol_id": protocol["protocol_id"],
            },
            "primary_candidate": {
                "candidate_id": primary.candidate_id,
                "record_sha256": _canonical_sha256(candidate_record),
            },
            "development_candidate_lock": _source_record(
                development_candidate_lock
            ),
            "validation_decision_lock": (
                None
                if validation_decision_lock is None
                else _source_record(validation_decision_lock)
            ),
            "paired_comparator_id": BASELINE_ID,
            "evaluated_pair_count": 2,
            "no_rescue_or_reselection": True,
            "no_rerun_without_new_explicit_recovery_authorization": True,
        },
    )


def _evaluate_trial(
    protocol: Mapping[str, Any],
    artifacts: TrialArtifacts,
    base: Any,
    candidates: Sequence[Any],
    *,
    stage_label: str,
    expected_detector_stations: int | None,
    retain_bundles: bool,
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[TrialCadenceBundle],
    list[dict[str, Any]],
]:
    if not artifacts.ik_receipt.is_file():
        raise ProtocolError("final IK receipt must exist before detector replay")
    plan = _build_sampling_plan(
        base,
        candidates,
        stage_label=stage_label,
        expected_detector_stations=expected_detector_stations,
    )
    rows: list[dict[str, Any]] = []
    access: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    bundles: list[TrialCadenceBundle] = []
    reference_digest_sets: list[list[str]] = []
    for cadence_label, sample_dt_s in CADENCES:
        bundle = sample_trial_cadence_once(
            protocol,
            artifacts.setup,
            base,
            plan,
            trial_id=artifacts.trial_id,
            cadence_label=cadence_label,
            sample_dt_s=sample_dt_s,
        )
        reference_digest_sets.append(list(bundle.access["plateau_reference_digests"]))
        for candidate in candidates:
            candidate_rows, candidate_details = evaluate_continuous_candidate(
                protocol, base, candidate, bundle
            )
            for row in candidate_rows:
                row["v14_stage"] = stage_label
            rows.extend(candidate_rows)
            for detail in candidate_details:
                detail["v14_stage"] = stage_label
            details.extend(candidate_details)
        for reference in bundle.plateau_references:
            access.append(
                {
                    **bundle.access,
                    "unit_key": _unit_key(
                        artifacts.trial_id,
                        int(reference["plateau_index"]),
                        cadence_label,
                    ),
                    "plateau_index": int(reference["plateau_index"]),
                    "speed_mps": float(reference["speed_mps"]),
                    "reference_hs_count": len(reference["events"]["heel_strike"]),
                    "reference_to_count": len(reference["events"]["toe_off"]),
                    "reference_event_sha256": reference["reference_event_sha256"],
                    "fsm_was_not_reset_for_this_plateau": True,
                }
            )
        if retain_bundles:
            bundles.append(bundle)
    if len(reference_digest_sets) != 2 or reference_digest_sets[0] != reference_digest_sets[1]:
        raise ProtocolError("10 ms and 1 ms do not share the exact plateau cycle sets")
    expected_units = 4 * len(CADENCES)
    if len(access) != expected_units or len(rows) != expected_units * len(candidates):
        raise ProtocolError(f"{stage_label} unit cardinality drifted")
    attach_trial_aggregate_gates(rows)
    if len(details) != len(rows):
        raise ProtocolError(f"{stage_label} detail cardinality drifted")
    return rows, access, bundles, details


def _evaluate_combinations_from_bundles(
    protocol: Mapping[str, Any],
    base: Any,
    combinations: Sequence[Any],
    bundles: Sequence[TrialCadenceBundle],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    for bundle in bundles:
        for candidate in combinations:
            candidate_rows, candidate_details = evaluate_continuous_candidate(
                protocol, base, candidate, bundle
            )
            for row in candidate_rows:
                row["v14_stage"] = "development_stage2_reused_station_samples"
            rows.extend(candidate_rows)
            for detail in candidate_details:
                detail["v14_stage"] = "development_stage2_reused_station_samples"
            details.extend(candidate_details)
    if len(rows) != len(bundles) * len(combinations) * 4:
        raise ProtocolError("V14 combination row cardinality drifted")
    attach_trial_aggregate_gates(rows)
    if len(details) != len(rows):
        raise ProtocolError("V14 combination detail cardinality drifted")
    return rows, details


def _write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    fields = sorted({str(key) for row in rows for key in row})
    try:
        with path.open("x", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    except FileExistsError as exc:
        raise NoClobberError(f"refusing to overwrite {_portable_path(path)}") from exc
    return path


def _write_jsonl_exclusive(
    path: Path, records: Sequence[Mapping[str, Any]]
) -> Path:
    lines = [
        json.dumps(
            v1._json_safe(dict(record)),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        for record in records
    ]
    return _write_text_exclusive(path, "".join(f"{line}\n" for line in lines))


def _preflight_no_clobber(output_dir: Path) -> None:
    if output_dir.resolve() != DEFAULT_OUTPUT_DIR.resolve():
        raise ProtocolError(
            "V14 execution is bound to its canonical output directory"
        )
    if DEFAULT_EXECUTION_LEDGER.exists():
        raise NoClobberError(
            "V14 canonical one-shot ledger already exists; holdouts cannot be reopened"
        )
    if output_dir.exists():
        raise NoClobberError(
            f"refusing to modify occupied V14 output: {_portable_path(output_dir)}"
        )


def run_cross_speed_protocol(
    protocol: Mapping[str, Any], output_dir: Path
) -> dict[str, Any]:
    """Execute the frozen staged protocol.  This is never called at import."""

    _preflight_no_clobber(output_dir)
    # Static profile/model/mesh validation is non-semantic and occurs before
    # consuming the one-shot ledger.  No trial stream is opened here.
    base, isolated, geometry = build_isolated_candidates(protocol)
    execution_ledger = _write_json_exclusive(
        DEFAULT_EXECUTION_LEDGER,
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V14_CANONICAL_DESTINATION_CONSUMED",
            "protocol_id": protocol["protocol_id"],
            "protocol_sha256": protocol["_protocol_sha256"],
            "canonical_output_dir": _portable_path(output_dir),
            "development_semantic_access_started": True,
            "validation_semantic_access_started": False,
            "sealed_semantic_access_started": False,
            "reserve_semantic_access_started": False,
            "rerun_or_alternate_destination_allowed": False,
        },
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    run_receipt = _write_json_exclusive(
        output_dir / "run_start_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V14_DEVELOPMENT_ACCESS_STARTED",
            "protocol": {
                "path": _portable_path(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
                "protocol_id": protocol["protocol_id"],
            },
            "development_trials": protocol["split"]["DEVELOPMENT"],
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
            "no_clobber": True,
        },
    )

    development_rows: list[dict[str, Any]] = []
    development_access: list[dict[str, Any]] = []
    development_details: list[dict[str, Any]] = []
    development_bundles: list[TrialCadenceBundle] = []
    preprocessing: dict[str, Any] = {}
    for trial_id in protocol["split"]["DEVELOPMENT"]:
        artifacts = prepare_trial(
            protocol,
            trial_id=trial_id,
            stage="development",
            work_dir=output_dir / "preprocessed" / f"trial_{trial_id}",
            access_receipt=None,
        )
        preprocessing[trial_id] = {
            "stage": "development",
            "ik_receipt": _source_record(artifacts.ik_receipt),
            "conversion_manifest": _source_record(artifacts.conversion_manifest),
            "dataset_ik_used_downstream": False,
        }
        rows, access, bundles, details = _evaluate_trial(
            protocol,
            artifacts,
            base,
            isolated,
            stage_label=f"development_trial_{trial_id}_isolated",
            expected_detector_stations=EXPECTED_DEVELOPMENT_DETECTOR_STATIONS,
            retain_bundles=True,
        )
        development_rows.extend(rows)
        development_access.extend(access)
        development_bundles.extend(bundles)
        development_details.extend(details)
    if len(development_bundles) != 3 * 2:
        raise ProtocolError("V14 development continuous bundle count is not 6")

    arm_winners, arm_selection = select_isolated_arm_winners(
        development_rows, isolated
    )
    combinations, combination_audit = build_combinations(
        protocol, arm_winners, isolated, geometry, development_rows
    )
    new_ids = set(combination_audit["new_candidate_ids"])
    new_combinations = [
        item for item in combinations if item.candidate_id in new_ids
    ]
    new_combination_rows, new_combination_details = (
        _evaluate_combinations_from_bundles(
            protocol, base, new_combinations, development_bundles
        )
    )
    reused_ids = {
        item.candidate_id for item in combinations if item.candidate_id not in new_ids
    }
    reused_combination_rows = [
        {**row, "v14_stage": "development_stage2_reused_stage1_rows"}
        for row in development_rows
        if row["candidate_id"] in reused_ids
    ]
    combination_rows = [*reused_combination_rows, *new_combination_rows]
    baseline_rows = [
        row for row in development_rows if row["candidate_id"] == BASELINE_ID
    ]
    finalist_rows = [*baseline_rows, *combination_rows]
    finalist_id, development_decision = select_development_finalist(
        finalist_rows, combinations
    )
    by_id = {item.candidate_id: item for item in (*isolated, *combinations)}

    dev_isolated_csv = _write_csv_exclusive(
        output_dir / "development_isolated_unit_metrics.csv", development_rows
    )
    dev_combos_csv = _write_csv_exclusive(
        output_dir / "development_combination_unit_metrics.csv", combination_rows
    )
    dev_stage1_details_jsonl = _write_jsonl_exclusive(
        output_dir / "development_stage1_details.jsonl", development_details
    )
    dev_stage2_new_details_jsonl = _write_jsonl_exclusive(
        output_dir / "development_stage2_new_details.jsonl",
        new_combination_details,
    )
    if finalist_id is None:
        boundary_stop = {
            "stop_before_validation": False,
            "status": "NO_FINALIST_V13_RETAINED",
            "stop_arms": [],
            "arms": {},
            "v13_remains_baseline_if_stopped": True,
            "global_minimum_claim_allowed": False,
        }
    else:
        boundary_stop = boundary_saturation_stop(
            by_id[finalist_id], development_rows, isolated
        )
    development_candidate_lock = _write_json_exclusive(
        output_dir / "development_candidate_lock.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "DEVELOPMENT_DECISION_FROZEN_BEFORE_HOLDOUT_ACCESS",
            "protocol_sha256": protocol["_protocol_sha256"],
            "finalist_id": finalist_id,
            "finalist_record": (
                None if finalist_id is None else _candidate_record(by_id[finalist_id])
            ),
            "development_decision": development_decision,
            "boundary_saturation_stop": boundary_stop,
            "development_isolated_metrics": _source_record(dev_isolated_csv),
            "development_stage2_metrics": _source_record(dev_combos_csv),
            "development_stage1_details": _source_record(
                dev_stage1_details_jsonl
            ),
            "development_stage2_new_details": _source_record(
                dev_stage2_new_details_jsonl
            ),
            "validation_semantic_access_allowed": bool(
                finalist_id is not None
                and not boundary_stop["stop_before_validation"]
            ),
            "no_clobber": True,
        },
    )
    validation_rows: list[dict[str, Any]] = []
    sealed_rows: list[dict[str, Any]] = []
    validation_access: list[dict[str, Any]] = []
    sealed_access: list[dict[str, Any]] = []
    validation_details: list[dict[str, Any]] = []
    sealed_details: list[dict[str, Any]] = []
    validation_decision: dict[str, Any] | None = None
    sealed_decision: dict[str, Any] | None = None
    validation_receipt: Path | None = None
    sealed_receipt: Path | None = None
    validation_decision_lock: Path | None = None
    validation_csv: Path | None = None
    validation_details_jsonl: Path | None = None
    sealed_csv: Path | None = None
    sealed_details_jsonl: Path | None = None

    if finalist_id is not None and not boundary_stop["stop_before_validation"]:
        finalist = by_id[finalist_id]
        validation_trial = protocol["split"]["VALIDATION"][0]
        validation_receipt = _write_stage_access_receipt(
            output_dir,
            protocol,
            stage="validation",
            trial_id=validation_trial,
            primary=finalist,
            development_candidate_lock=development_candidate_lock,
        )
        validation_artifacts = prepare_trial(
            protocol,
            trial_id=validation_trial,
            stage="validation",
            work_dir=output_dir / "preprocessed" / f"trial_{validation_trial}",
            access_receipt=validation_receipt,
        )
        preprocessing[validation_trial] = {
            "stage": "validation",
            "ik_receipt": _source_record(validation_artifacts.ik_receipt),
            "conversion_manifest": _source_record(
                validation_artifacts.conversion_manifest
            ),
            "dataset_ik_used_downstream": False,
        }
        validation_rows, validation_access, _, validation_details = _evaluate_trial(
            protocol,
            validation_artifacts,
            base,
            (finalist, by_id[BASELINE_ID]),
            stage_label="validation_frozen_pair",
            expected_detector_stations=None,
            retain_bundles=False,
        )
        validation_decision = paired_holdout_decision(
            validation_rows, finalist_id, stage="validation"
        )
        validation_csv = _write_csv_exclusive(
            output_dir / "validation_paired_unit_metrics.csv", validation_rows
        )
        validation_details_jsonl = _write_jsonl_exclusive(
            output_dir / "validation_paired_details.jsonl", validation_details
        )
        validation_decision_lock = _write_json_exclusive(
            output_dir / "validation_decision_lock.json",
            {
                "schema_version": SCHEMA_VERSION,
                "status": "VALIDATION_DECISION_FROZEN_BEFORE_SEALED_ACCESS",
                "protocol_sha256": protocol["_protocol_sha256"],
                "development_candidate_lock": _source_record(
                    development_candidate_lock
                ),
                "primary_candidate_id": finalist_id,
                "validation_decision": validation_decision,
                "validation_metrics": _source_record(validation_csv),
                "validation_details": _source_record(
                    validation_details_jsonl
                ),
                "sealed_semantic_access_allowed": bool(
                    validation_decision["ok"]
                ),
                "no_clobber": True,
            },
        )
        if validation_decision["ok"]:
            sealed_trial = protocol["split"]["SEALED"][0]
            sealed_receipt = _write_stage_access_receipt(
                output_dir,
                protocol,
                stage="sealed",
                trial_id=sealed_trial,
                primary=finalist,
                development_candidate_lock=development_candidate_lock,
                validation_decision_lock=validation_decision_lock,
            )
            sealed_artifacts = prepare_trial(
                protocol,
                trial_id=sealed_trial,
                stage="sealed",
                work_dir=output_dir / "preprocessed" / f"trial_{sealed_trial}",
                access_receipt=sealed_receipt,
            )
            preprocessing[sealed_trial] = {
                "stage": "sealed",
                "ik_receipt": _source_record(sealed_artifacts.ik_receipt),
                "conversion_manifest": _source_record(
                    sealed_artifacts.conversion_manifest
                ),
                "dataset_ik_used_downstream": False,
            }
            sealed_rows, sealed_access, _, sealed_details = _evaluate_trial(
                protocol,
                sealed_artifacts,
                base,
                (finalist, by_id[BASELINE_ID]),
                stage_label="sealed_frozen_pair",
                expected_detector_stations=None,
                retain_bundles=False,
            )
            sealed_decision = paired_holdout_decision(
                sealed_rows, finalist_id, stage="sealed"
            )
            sealed_csv = _write_csv_exclusive(
                output_dir / "sealed_paired_unit_metrics.csv", sealed_rows
            )
            sealed_details_jsonl = _write_jsonl_exclusive(
                output_dir / "sealed_paired_details.jsonl", sealed_details
            )

    if finalist_id is None:
        conclusion = "V13_RETAINED_NO_DEVELOPMENT_FINALIST"
        ok = False
    elif boundary_stop["stop_before_validation"]:
        conclusion = "V13_RETAINED_BOUNDARY_SATURATION_REQUIRES_V14X_DEV"
        ok = False
    elif validation_decision is None or not validation_decision["ok"]:
        conclusion = "V13_RETAINED_VALIDATION_FAIL"
        ok = False
    elif sealed_decision is None or not sealed_decision["ok"]:
        conclusion = "V13_RETAINED_SEALED_FAIL"
        ok = False
    else:
        conclusion = "V14_CHALLENGER_PASSED_ALL_STAGES"
        ok = True

    all_rows = [
        *development_rows,
        *combination_rows,
        *validation_rows,
        *sealed_rows,
    ]
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "conclusion": conclusion,
        "objective": protocol["objective"],
        "protocol": {
            "path": _portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
        },
        "run_start_receipt": _source_record(run_receipt),
        "canonical_execution_ledger": _source_record(execution_ledger),
        "development_candidate_lock": _source_record(
            development_candidate_lock
        ),
        "validation_decision_lock": (
            None
            if validation_decision_lock is None
            else _source_record(validation_decision_lock)
        ),
        "preprocessing": preprocessing,
        "geometry": {
            key: value for key, value in geometry.items() if key != "triangles"
        },
        "isolated_candidates": [_candidate_record(item) for item in isolated],
        "arm_selection": arm_selection,
        "combination_generation": combination_audit,
        "combination_candidates": [_candidate_record(item) for item in combinations],
        "development": {
            "unit_count": 24,
            "access": development_access,
            "decision": development_decision,
            "boundary_saturation_stop": boundary_stop,
            "stage1_aggregated_counts": aggregate_trial_counts(development_rows),
            "stage2_aggregated_counts": aggregate_trial_counts(
                [*baseline_rows, *combination_rows]
            ),
        },
        "validation": {
            "opened": validation_receipt is not None,
            "receipt": None
            if validation_receipt is None
            else _source_record(validation_receipt),
            "access": validation_access,
            "decision": validation_decision,
            "aggregated_counts": aggregate_trial_counts(validation_rows)
            if validation_rows
            else [],
        },
        "sealed": {
            "opened": sealed_receipt is not None,
            "receipt": None if sealed_receipt is None else _source_record(sealed_receipt),
            "access": sealed_access,
            "decision": sealed_decision,
            "aggregated_counts": aggregate_trial_counts(sealed_rows)
            if sealed_rows
            else [],
        },
        "reserve": {
            "trial_ids": protocol["split"]["RESERVE"],
            "opened": False,
        },
        "artifacts": {
            "development_isolated_unit_metrics": _source_record(dev_isolated_csv),
            "development_combination_unit_metrics": _source_record(dev_combos_csv),
            "development_stage1_details": _source_record(
                dev_stage1_details_jsonl
            ),
            "development_stage2_new_details": _source_record(
                dev_stage2_new_details_jsonl
            ),
            "validation_paired_unit_metrics": (
                None if validation_csv is None else _source_record(validation_csv)
            ),
            "validation_paired_details": (
                None
                if validation_details_jsonl is None
                else _source_record(validation_details_jsonl)
            ),
            "sealed_paired_unit_metrics": (
                None if sealed_csv is None else _source_record(sealed_csv)
            ),
            "sealed_paired_details": (
                None
                if sealed_details_jsonl is None
                else _source_record(sealed_details_jsonl)
            ),
        },
        "source_identity": protocol["sources"],
        "non_actions": {
            "dataset_ik_used": False,
            "ik_smoke_used": False,
            "trial01_opened": False,
            "reserve_opened": False,
            "threshold_or_fsm_changed": False,
            "profile_or_registry_modified": False,
            "runtime_configuration_modified": False,
            "policy_or_training_run": False,
            "automatic_promotion_performed": False,
            "post_hoc_holdout_reselection_used": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
        "row_count": len(all_rows),
    }
    _write_json_exclusive(output_dir / "manifest.json", manifest)
    return v1._json_safe(manifest)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print or execute the frozen AB06 cross-speed V14 protocol."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--metadata-audit", default=str(DEFAULT_METADATA_AUDIT))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument(
        "--print-protocol-template",
        action="store_true",
        help="Print the expected protocol without semantically decoding any trial.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the frozen staged protocol; required for semantic access.",
    )
    return parser


def _write_failure(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir():
        return
    failure = output_dir / "failure.json"
    if failure.exists():
        return
    _write_json_exclusive(
        failure,
        {
            "schema_version": SCHEMA_VERSION,
            "status": "ERROR_AFTER_V14_RUN_DESTINATION_CONSUMED",
            "ok": False,
            "rerun_allowed": False,
            "validation_receipt_exists": (output_dir / "validation_access_receipt.json").is_file(),
            "sealed_receipt_exists": (output_dir / "sealed_access_receipt.json").is_file(),
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        },
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.print_protocol_template:
        if args.execute:
            parser.error("--print-protocol-template and --execute are mutually exclusive")
        payload = expected_protocol_payload(
            args.metadata_audit,
            require_metadata_audit=False,
            require_all_sources=False,
        )
        print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
        return 0
    if not args.execute:
        parser.error("semantic execution requires the explicit --execute flag")
    output_dir = v1.resolve_repo_path(args.output_dir).resolve()
    try:
        _preflight_no_clobber(output_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_cross_speed_protocol(protocol, output_dir)
    except NoClobberError as exc:
        consumed = bool(output_dir.exists() or DEFAULT_EXECUTION_LEDGER.exists())
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "no_clobber": True,
                    "destination_or_global_ledger_consumed": consumed,
                    "filesystem_mutated_this_attempt": None,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 2
    except Exception as exc:  # pragma: no cover - integration failure path.
        if (
            output_dir.resolve() == DEFAULT_OUTPUT_DIR.resolve()
            and output_dir.is_dir()
            and (output_dir / "run_start_receipt.json").is_file()
        ):
            _write_failure(output_dir, exc)
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "destination_or_global_ledger_consumed": bool(
                        output_dir.exists() or DEFAULT_EXECUTION_LEDGER.exists()
                    ),
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                },
                indent=2,
            )
        )
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "conclusion": manifest["conclusion"],
                "manifest": _portable_path(output_dir / "manifest.json"),
                "validation_opened": manifest["validation"]["opened"],
                "sealed_opened": manifest["sealed"]["opened"],
                "reserve_opened": False,
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
