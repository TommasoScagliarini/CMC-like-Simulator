"""Fail-closed AB06 IK replay for the prosthetic heel/toe detector.

The detector input is reconstructed from the current detector-only onlineGRF
profile by sampling its *separate* ``left_heel`` and ``left_toe`` spheres on
the prescribed IK.  Those two loads are then streamed through the production
``ProstheticPhaseFSM`` in ``two_sensor`` mode (there is no validation-only gait
state machine).

The timing oracle is independent at runtime: left HS/TO events are extracted
from the prescribed ExternalLoads vertical-force channel declared by the
setup.  Boundary-censored contacts are handled by the established
``_cycles_from_vertical_grf`` parser.  No event timestamp is embedded in this
script.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import traceback
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from model_loader import _infer_external_force_side  # noqa: E402
from config import SimulatorConfig  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from online_grf import (  # noqa: E402
    OnlineGRFProfile,
    load_online_grf_profile,
    online_grf_sensor_role,
)
from output import _cycles_from_vertical_grf, _read_storage_table  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
)
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _calculate_wrench,
    _external_grf,
    _sample_spheres,
)
from validation.validate_online_grf_events import (  # noqa: E402
    match_events,
    strict_event_pass,
)


DEFAULT_SETUP = (
    "models/AB06_SEASEA_Threadmill/"
    "AB06_SEASEA_stiff321_500_pi_setup.xml"
)
DEFAULT_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
DEFAULT_SEA_PLUGIN = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"
DEFAULT_OUTPUT_DIR = "validation/two_sensor_prescribed_replay_runs/latest"
CURRENT_TRAINING_CONFIG = (
    REPO_ROOT / "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"
)
EPIC_GC_LEFT = (
    REPO_ROOT
    / "models/AB06-raw/10_09_18/treadmill/gcLeft/treadmill_01_01.mat"
)
EPIC_GC_RIGHT = (
    REPO_ROOT
    / "models/AB06-raw/10_09_18/treadmill/gcRight/treadmill_01_01.mat"
)


PHASE_SWING = 0
PHASE_STANCE = 1
PHASE_UNKNOWN = -1


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"validation source not found: {resolved}")
    return {
        "path": _portable_path(resolved),
        "sha256": _sha256(resolved),
        "bytes": int(resolved.stat().st_size),
    }


def _current_training_segment_duration_s() -> float:
    import yaml

    raw = yaml.safe_load(CURRENT_TRAINING_CONFIG.read_text(encoding="utf-8"))
    try:
        value = float(raw["simulation"]["segment_duration"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "current training config has no valid simulation.segment_duration"
        ) from exc
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"invalid current training segment duration: {value}")
    return value


def _current_runtime_fsm_config() -> ProstheticPhaseFSMConfig:
    """Resolve the counterfactual two-sensor FSM with current training gates."""
    import yaml

    raw = yaml.safe_load(CURRENT_TRAINING_CONFIG.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("current training config must contain a mapping")
    reward = raw.get("reward")
    grf = raw.get("grf")
    if not isinstance(reward, dict) or not isinstance(grf, dict):
        raise ValueError("current training config requires reward and grf mappings")

    defaults = ProstheticPhaseFSMConfig()
    reward_mapping = {
        "min_stance_duration_s": "phase_min_stance_duration_s",
        "min_swing_duration_s": "phase_min_swing_duration_s",
        "landing_window_start_s": "phase_landing_window_start_s",
        "landing_window_end_s": "phase_landing_window_end_s",
        "stance_hard_timeout_s": "phase_stance_hard_timeout_s",
        "swing_hard_timeout_s": "phase_swing_hard_timeout_s",
        "landing_force_full_credit_bw": "phase_landing_force_full_credit_bw",
        "min_stance_contact_fraction": "phase_min_stance_contact_fraction",
        "min_stance_load_bw_s": "phase_min_stance_load_bw_s",
        "min_cycle_knee_excursion_rad": "phase_min_cycle_knee_excursion_rad",
        "hs_event_credit": "phase_hs_event_credit",
        "toe_off_event_credit": "phase_to_event_credit",
        "cycle_complete_bonus": "phase_cycle_complete_bonus",
        "failure_extra_penalty": "phase_failure_extra_penalty",
    }
    values: dict[str, Any] = {
        field_name: float(reward.get(config_name, getattr(defaults, field_name)))
        for field_name, config_name in reward_mapping.items()
    }
    values.update(
        {
            # The checked-in config keeps legacy_events authoritative pending
            # promotion.  This validation changes only the source under test;
            # every acceptance/evidence gate remains the current runtime value.
            "event_source": "two_sensor",
            "sensor_on_threshold_n": float(grf["phase_sensor_on_threshold_n"]),
            "sensor_off_threshold_n": float(grf["phase_sensor_off_threshold_n"]),
            "sensor_dwell_s": float(grf["phase_sensor_dwell_s"]),
            "duration_history_window_cycles": int(
                defaults.duration_history_window_cycles
            ),
        }
    )
    return ProstheticPhaseFSMConfig(**values)


def _fsm_config_payload(config: ProstheticPhaseFSMConfig) -> dict[str, Any]:
    return {
        name: getattr(config, name)
        for name in config.__dataclass_fields__
    }


def _epic_gc_secondary_audit(
    prescribed_heel_strikes: np.ndarray,
) -> dict[str, Any]:
    """Hash-locked audit of raw EPIC gait labels; never a timing gate.

    MATLAB tables are not a portable runtime dependency of this Python
    validator.  The values below were extracted directly on 2026-07-21 from
    the two hash-pinned source files using rising edges of ``HeelStrike`` and
    ``ToeOff``.  If either file changes, the audit fails closed and the stored
    values are not reported as current.
    """
    expected_hashes = {
        "gcLeft": "429e41f56ff4c865e1c444bfd7e0516056960511601b4eae5de3d8c26ec41afa",
        "gcRight": "2fe316f25434420a4ae1d9d865de77206d8c1158bfb9c1cb609f1a6bda5cd4c1",
    }
    sources = {
        "gcLeft": _source_record(EPIC_GC_LEFT),
        "gcRight": _source_record(EPIC_GC_RIGHT),
    }
    hashes_match = all(
        sources[name]["sha256"] == expected_hashes[name]
        for name in expected_hashes
    )
    base: dict[str, Any] = {
        "role": "secondary provenance check only; excluded from PASS/FAIL",
        "audit_date": "2026-07-21",
        "audit_method": (
            "MATLAB table rising edges of HeelStrike/ToeOff in [11.99, 21.0] s"
        ),
        "sources": sources,
        "expected_sha256": expected_hashes,
        "hashes_match_audited_sources": bool(hashes_match),
    }
    if not hashes_match:
        base.update(
            {
                "status": "STALE_SOURCE_HASH",
                "heel_strike_crosscheck": None,
                "toe_off_eligible_as_side_specific_reference": False,
                "toe_off_exclusion_reason": (
                    "source changed after audit; raw labels require reinspection"
                ),
            }
        )
        return base

    left_hs = np.asarray([13.965, 15.635, 17.190, 18.745, 20.305], dtype=float)
    right_hs = np.asarray([13.180, 14.800, 16.400, 17.970, 19.545], dtype=float)
    # These identical rising-edge arrays demonstrate why ToeOff is not a valid
    # side-specific oracle for this trial.
    left_to = np.asarray([12.705, 14.240, 15.915, 17.470, 19.040, 20.615])
    right_to = np.asarray([12.705, 14.240, 15.915, 17.470, 19.040, 20.615])
    hs_delta = np.asarray(prescribed_heel_strikes, dtype=float) - left_hs
    base.update(
        {
            "status": "AUDITED",
            "heel_strike_crosscheck": {
                "gcLeft_rising_edges_s": left_hs.tolist(),
                "gcRight_rising_edges_s": right_hs.tolist(),
                "prescribed_grf_minus_gcLeft_s": hs_delta.tolist(),
                "max_abs_difference_s": float(np.max(np.abs(hs_delta))),
                "interpretation": (
                    "prescribed GRF events are 17-28 ms earlier and remain the "
                    "primary reference used by the established gate"
                ),
            },
            "toe_off_eligible_as_side_specific_reference": False,
            "toe_off_sequences_identical_between_gcLeft_gcRight": bool(
                np.array_equal(left_to, right_to)
            ),
            "toe_off_rising_edges_s": left_to.tolist(),
            "toe_off_exclusion_reason": (
                "gcLeft and gcRight ToeOff rising edges are identical, so the "
                "field is not side-specific and is not used by this validator"
            ),
        }
    )
    return base


def _left_external_force_source(setup: Any) -> tuple[Path, str]:
    """Resolve the prescribed left vertical-force file and column from XML."""
    if setup.external_loads_xml is None:
        raise ValueError("prescribed replay requires setup ExternalLoads")

    import opensim

    external = opensim.ExternalLoads(str(setup.external_loads_xml), True)
    data_file = Path(external.getDataFileName())
    if not data_file.is_absolute():
        data_file = setup.external_loads_xml.parent / data_file
    data_file = data_file.resolve()

    identifiers: list[str] = []
    for index in range(external.getSize()):
        force = external.get(index)
        side = _infer_external_force_side(
            force.getName(),
            force.get_applied_to_body(),
        )
        if side == "left":
            identifiers.append(str(force.getForceIdentifier()))
    if len(identifiers) != 1:
        raise ValueError(
            "expected exactly one prescribed left ExternalForce; observed "
            f"{identifiers}"
        )
    # External loads are expressed in ground and the AB06 setup uses +Y as the
    # vertical axis, matching the detector profile ground normal.
    return data_file, f"{identifiers[0]}y"


def _reference_events_from_prescribed_grf(
    setup: Any,
    *,
    threshold_n: float,
    min_contact_duration_s: float,
    min_cycle_duration_s: float,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Derive only complete-window events from the declared prescribed GRF."""
    data_file, vertical_column = _left_external_force_source(setup)
    time, columns, data = _read_storage_table(str(data_file))
    if vertical_column not in columns:
        raise ValueError(
            f"prescribed GRF is missing {vertical_column!r}: {data_file}"
        )
    vertical_force = np.asarray(data[:, columns.index(vertical_column)], dtype=float)
    cycles = _cycles_from_vertical_grf(
        np.asarray(time, dtype=float),
        vertical_force,
        float(threshold_n),
        float(setup.t_start),
        float(setup.t_end),
        float(min_contact_duration_s),
        float(min_cycle_duration_s),
    )
    if not cycles:
        raise ValueError("prescribed GRF contains no complete left HS-to-HS cycle")

    heel_strikes = [float(cycle[0]) for cycle in cycles]
    heel_strikes.append(float(cycles[-1][1]))
    toe_offs = [float(cycle[0] + cycle[2]) for cycle in cycles]
    if not (
        all(a < b for a, b in zip(heel_strikes, heel_strikes[1:]))
        and all(hs < toe < next_hs for hs, toe, next_hs in zip(
            heel_strikes,
            toe_offs,
            heel_strikes[1:],
        ))
    ):
        raise ValueError("prescribed event sequence is not strictly HS-TO-HS")

    provenance = {
        "method": "output._cycles_from_vertical_grf",
        "boundary_policy": (
            "only complete HS-to-HS cycles; the contact already active at "
            "t_start and the open contact at t_end are boundary-censored"
        ),
        "external_loads_xml": _source_record(setup.external_loads_xml),
        "vertical_force_storage": _source_record(data_file),
        "vertical_force_column": vertical_column,
        "threshold_n": float(threshold_n),
        "min_contact_duration_s": float(min_contact_duration_s),
        "min_cycle_duration_s": float(min_cycle_duration_s),
        "cycle_count": int(len(cycles)),
    }
    return {
        "heel_strike": np.asarray(heel_strikes, dtype=float),
        "toe_off": np.asarray(toe_offs, dtype=float),
    }, provenance


def _left_sensor_spheres(profile: OnlineGRFProfile) -> dict[str, Any]:
    by_role: dict[str, list[Any]] = {"left_heel": [], "left_toe": []}
    for sphere in profile.spheres:
        role = online_grf_sensor_role(sphere.name, sphere.side)
        if role in by_role:
            by_role[role].append(sphere)
    ambiguous = {
        role: [sphere.name for sphere in spheres]
        for role, spheres in by_role.items()
        if len(spheres) != 1
    }
    if ambiguous:
        raise ValueError(
            "detector profile must resolve exactly one sphere per left sensor "
            f"role: {ambiguous}"
        )
    left_names = {sphere.name for sphere in profile.spheres if sphere.side == "left"}
    sensor_names = {spheres[0].name for spheres in by_role.values()}
    if left_names != sensor_names:
        raise ValueError(
            "left detector aggregate contains non heel/toe spheres: "
            f"left={sorted(left_names)}, detector={sorted(sensor_names)}"
        )
    return {role: spheres[0] for role, spheres in by_role.items()}


def _regional_loads_and_penetrations(
    profile: OnlineGRFProfile,
    samples: dict[str, Any],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], np.ndarray]:
    """Use the established contact law separately for heel and toe spheres."""
    sensor_spheres = _left_sensor_spheres(profile)
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)

    loads: dict[str, np.ndarray] = {}
    penetrations: dict[str, np.ndarray] = {}
    for role, sphere in sensor_spheres.items():
        regional_profile = replace(profile, spheres=(sphere,))
        regional_wrench = _calculate_wrench(regional_profile, samples)
        load = np.asarray(regional_wrench["left"]["normal_force"], dtype=float)
        center = np.asarray(samples["centers"][sphere.name], dtype=float)
        penetration = np.maximum(
            0.0,
            float(sphere.radius) - (center - origin) @ normal,
        )
        if not np.all(np.isfinite(load)) or np.any(load < 0.0):
            raise FloatingPointError(f"invalid normal load for {role}")
        if not np.all(np.isfinite(penetration)) or np.any(penetration < 0.0):
            raise FloatingPointError(f"invalid penetration for {role}")
        loads[role] = load
        penetrations[role] = penetration

    aggregate = np.asarray(
        _calculate_wrench(profile, samples)["left"]["normal_force"],
        dtype=float,
    )
    reconstructed = loads["left_heel"] + loads["left_toe"]
    if not np.allclose(aggregate, reconstructed, rtol=1e-12, atol=1e-10):
        raise AssertionError(
            "separate heel/toe loads do not reconstruct the profile left aggregate"
        )
    return loads, penetrations, aggregate


def _model_body_weight_n(model_path: Path) -> float:
    import opensim

    model = opensim.Model(str(model_path))
    state = model.initSystem()
    weight = float(model.getTotalMass(state)) * 9.80665
    if not math.isfinite(weight) or weight <= 0.0:
        raise ValueError(f"invalid model body weight: {weight}")
    return weight


def _prescribed_prosthetic_kinematics(
    setup: Any,
    times: np.ndarray,
) -> dict[str, np.ndarray]:
    """Sample the same filtered prescribed q path used by the runtime env."""
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(setup.model_file.parent)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.t_start = float(times[0])
    cfg.t_end = float(times[-1])
    interpolator = KinematicsInterpolator(cfg)
    knee = np.empty(len(times), dtype=float)
    ankle = np.empty(len(times), dtype=float)
    for index, time_s in enumerate(times):
        q, _qdot, _qddot = interpolator.get(float(time_s))
        knee[index] = float(q["pros_knee_angle"])
        ankle[index] = float(q["pros_ankle_angle"])
    if not np.all(np.isfinite(knee)) or not np.all(np.isfinite(ankle)):
        raise FloatingPointError("non-finite prescribed prosthetic kinematics")
    return {"knee_rad": knee, "ankle_rad": ankle}


def _run_production_fsm(
    times: np.ndarray,
    loads: dict[str, np.ndarray],
    penetrations: dict[str, np.ndarray],
    aggregate_load_n: np.ndarray,
    prosthetic_kinematics: dict[str, np.ndarray],
    *,
    body_weight_n: float,
    fsm_config: ProstheticPhaseFSMConfig,
) -> dict[str, Any]:
    if fsm_config.event_source != "two_sensor":
        raise ValueError("exact integration gate requires event_source=two_sensor")
    fsm = ProstheticPhaseFSM(fsm_config)
    heel_contact = np.zeros(len(times), dtype=float)
    toe_contact = np.zeros(len(times), dtype=float)
    state_id = np.zeros(len(times), dtype=float)
    accepted: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []
    sensor_edges: list[dict[str, Any]] = []
    invalid_steps: list[dict[str, Any]] = []

    for index, time_s in enumerate(times):
        in_contact = bool(
            penetrations["left_heel"][index] > 0.0
            or penetrations["left_toe"][index] > 0.0
        )
        payload = fsm.update(
            time_s=float(time_s),
            events=(),
            normal_force_bw=float(aggregate_load_n[index] / body_weight_n),
            in_contact=in_contact,
            prosthetic_knee_angle_rad=float(
                prosthetic_kinematics["knee_rad"][index]
            ),
            prosthetic_ankle_angle_rad=float(
                prosthetic_kinematics["ankle_rad"][index]
            ),
            heel_normal_force_n=float(loads["left_heel"][index]),
            toe_normal_force_n=float(loads["left_toe"][index]),
        )
        heel_contact[index] = float(payload["sensor_heel_contact"])
        toe_contact[index] = float(payload["sensor_toe_contact"])
        state_id[index] = float(payload["state_id"])

        step_candidates = [dict(item) for item in payload["sensor_events_this_step"]]
        step_transitions = [
            dict(item) for item in payload["accepted_transitions_this_step"]
        ]
        for item in step_candidates:
            item["observed_at_s"] = float(time_s)
            candidates.append(item)
        for item in payload["sensor_edges_this_step"]:
            row = dict(item)
            row["observed_at_s"] = float(time_s)
            sensor_edges.append(row)
        for transition in step_transitions:
            transition["confirmed_time_s"] = float(time_s)
            matches = [
                item
                for item in step_candidates
                if str(item.get("event", "")) == str(transition.get("event", ""))
                and abs(
                    float(item.get("time", float("inf")))
                    - float(transition.get("event_time_s", float("-inf")))
                )
                <= 1e-10
            ]
            transition["matched_sensor_candidate"] = bool(matches)
            accepted.append(transition)
        if float(payload["invalid_event_this_step"]) != 0.0:
            invalid_steps.append(
                {
                    "observed_at_s": float(time_s),
                    "type": str(payload["invalid_event_type"]),
                }
            )

    return {
        "fsm": fsm,
        "heel_contact": heel_contact,
        "toe_contact": toe_contact,
        "state_id": state_id,
        "accepted": accepted,
        "candidates": candidates,
        "sensor_edges": sensor_edges,
        "invalid_steps": invalid_steps,
    }


def _events_for_timing_gate(
    accepted: Sequence[dict[str, Any]],
) -> dict[str, np.ndarray]:
    """Exclude only the explicitly marked boundary-censored bootstrap TO."""
    result: dict[str, list[float]] = {"heel_strike": [], "toe_off": []}
    for transition in accepted:
        event = str(transition.get("event", ""))
        if event not in result:
            continue
        # A TO that closes the stance already active at t_start is a genuine
        # sensor transition, but it has no observable HS inside the replay
        # window.  Production FSM marks exactly this case as segment_valid=0.
        if event == "toe_off" and float(transition.get("segment_valid", 1.0)) == 0.0:
            continue
        result[event].append(float(transition["event_time_s"]))
    return {
        event: np.asarray(values, dtype=float)
        for event, values in result.items()
    }


def _accepted_gait_transitions_for_gate(
    accepted: Sequence[dict[str, Any]],
) -> dict[str, list[dict[str, Any]]]:
    """Return ordered causal gait transitions, excluding bootstrap-only TO."""
    result: dict[str, list[dict[str, Any]]] = {
        "heel_strike": [],
        "toe_off": [],
    }
    for transition in accepted:
        event = str(transition.get("event", ""))
        if event not in result:
            continue
        if (
            event == "toe_off"
            and float(transition.get("segment_valid", 1.0)) == 0.0
        ):
            continue
        result[event].append(dict(transition))
    return result


def _reference_phase_from_prescribed_grf(
    vertical_force_n: np.ndarray,
    *,
    threshold_n: float,
) -> np.ndarray:
    """Build an immediate, causal stance/swing reference from prescribed GRF."""
    force = np.asarray(vertical_force_n, dtype=float)
    if force.ndim != 1 or not np.all(np.isfinite(force)):
        raise ValueError("prescribed vertical force must be a finite 1-D array")
    return np.where(force > float(threshold_n), PHASE_STANCE, PHASE_SWING)


def _reference_phase_from_validated_events(
    times: np.ndarray,
    reference_events: dict[str, np.ndarray],
) -> np.ndarray:
    """Build an event-consistent phase reference from accepted GRF cycles.

    The reference event extractor already rejects contacts shorter than its
    frozen minimum duration.  Reusing those accepted HS/TO intervals prevents
    a rejected one-sample GRF recrossing from becoming an isolated stance
    label in the phase gate.
    """
    sample_times = np.asarray(times, dtype=float)
    hs = np.asarray(reference_events["heel_strike"], dtype=float)
    toe_off = np.asarray(reference_events["toe_off"], dtype=float)
    if (
        sample_times.ndim != 1
        or not np.all(np.isfinite(sample_times))
        or hs.ndim != 1
        or toe_off.ndim != 1
        or not np.all(np.isfinite(hs))
        or not np.all(np.isfinite(toe_off))
    ):
        raise ValueError("phase reference times/events must be finite 1-D arrays")
    if hs.size != toe_off.size + 1 or hs.size < 2:
        raise ValueError("validated phase reference requires HS, TO, ..., HS")
    if np.any(np.diff(hs) <= 0.0) or np.any(np.diff(toe_off) <= 0.0):
        raise ValueError("validated phase reference events must be strictly ordered")
    for index, to_time in enumerate(toe_off):
        if not (float(hs[index]) < float(to_time) < float(hs[index + 1])):
            raise ValueError("validated phase reference must alternate HS/TO/HS")

    phase = np.full(sample_times.shape, PHASE_SWING, dtype=int)
    for index, hs_time in enumerate(hs):
        stance_end = (
            float(toe_off[index]) if index < toe_off.size else float("inf")
        )
        phase[
            (sample_times >= float(hs_time) - 1e-12)
            & (sample_times < stance_end - 1e-12)
        ] = PHASE_STANCE
    return phase


def _fsm_phase_from_state_id(state_id: np.ndarray) -> np.ndarray:
    """Map only the production stance/swing states to binary gait phase."""
    states = np.asarray(state_id, dtype=float)
    if states.ndim != 1 or not np.all(np.isfinite(states)):
        raise ValueError("FSM state_id must be a finite 1-D array")
    phase = np.full(states.shape, PHASE_UNKNOWN, dtype=int)
    phase[np.isclose(states, float(STANCE_AFTER_HS), atol=1e-12)] = PHASE_STANCE
    phase[np.isclose(states, float(SWING_AFTER_TO), atol=1e-12)] = PHASE_SWING
    return phase


def _phase_agreement_metrics(
    reference_phase: np.ndarray,
    predicted_phase: np.ndarray,
    mask: np.ndarray,
) -> dict[str, Any]:
    reference = np.asarray(reference_phase, dtype=int)
    predicted = np.asarray(predicted_phase, dtype=int)
    selected = np.asarray(mask, dtype=bool)
    if reference.shape != predicted.shape or reference.shape != selected.shape:
        raise ValueError("phase arrays and mask must have identical shapes")

    ref = reference[selected]
    pred = predicted[selected]
    total = int(ref.size)
    matches = int(np.count_nonzero(ref == pred))
    unknown = int(np.count_nonzero(pred == PHASE_UNKNOWN))
    confusion = {
        "reference_stance_fsm_stance": int(
            np.count_nonzero(
                (ref == PHASE_STANCE) & (pred == PHASE_STANCE)
            )
        ),
        "reference_stance_fsm_swing": int(
            np.count_nonzero(
                (ref == PHASE_STANCE) & (pred == PHASE_SWING)
            )
        ),
        "reference_stance_fsm_unknown": int(
            np.count_nonzero(
                (ref == PHASE_STANCE) & (pred == PHASE_UNKNOWN)
            )
        ),
        "reference_swing_fsm_stance": int(
            np.count_nonzero(
                (ref == PHASE_SWING) & (pred == PHASE_STANCE)
            )
        ),
        "reference_swing_fsm_swing": int(
            np.count_nonzero(
                (ref == PHASE_SWING) & (pred == PHASE_SWING)
            )
        ),
        "reference_swing_fsm_unknown": int(
            np.count_nonzero(
                (ref == PHASE_SWING) & (pred == PHASE_UNKNOWN)
            )
        ),
    }
    return {
        "samples": total,
        "matches": matches,
        "mismatches": int(total - matches),
        "unknown_fsm_samples": unknown,
        "accuracy": float(matches / total) if total else float("nan"),
        "confusion": confusion,
    }


def _phase_duration_diagnostics(
    reference_events: dict[str, np.ndarray],
    predicted_events: dict[str, np.ndarray],
    accepted: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Compare per-cycle phase durations at physical onset and confirmation."""
    reference_hs = np.asarray(reference_events["heel_strike"], dtype=float)
    reference_to = np.asarray(reference_events["toe_off"], dtype=float)
    predicted_hs = np.asarray(predicted_events["heel_strike"], dtype=float)
    predicted_to = np.asarray(predicted_events["toe_off"], dtype=float)
    accepted_by_event = _accepted_gait_transitions_for_gate(accepted)
    confirmed_hs = np.asarray(
        [float(item["confirmed_time_s"]) for item in accepted_by_event["heel_strike"]],
        dtype=float,
    )
    confirmed_to = np.asarray(
        [float(item["confirmed_time_s"]) for item in accepted_by_event["toe_off"]],
        dtype=float,
    )

    expected_cycles = max(0, int(reference_hs.size) - 1)
    alignment_complete = bool(
        reference_to.size == expected_cycles
        and predicted_hs.size == reference_hs.size
        and predicted_to.size == reference_to.size
        and confirmed_hs.size == reference_hs.size
        and confirmed_to.size == reference_to.size
    )
    rows: list[dict[str, Any]] = []
    if alignment_complete:
        for index in range(expected_cycles):
            reference_cycle = float(reference_hs[index + 1] - reference_hs[index])
            reference_stance = float(reference_to[index] - reference_hs[index])
            reference_swing = float(reference_hs[index + 1] - reference_to[index])

            onset_cycle = float(predicted_hs[index + 1] - predicted_hs[index])
            onset_stance = float(predicted_to[index] - predicted_hs[index])
            onset_swing = float(predicted_hs[index + 1] - predicted_to[index])

            confirmed_cycle = float(confirmed_hs[index + 1] - confirmed_hs[index])
            confirmed_stance = float(confirmed_to[index] - confirmed_hs[index])
            confirmed_swing = float(confirmed_hs[index + 1] - confirmed_to[index])
            rows.append(
                {
                    "cycle_index": int(index),
                    "reference_hs_start_s": float(reference_hs[index]),
                    "reference_hs_end_s": float(reference_hs[index + 1]),
                    "reference": {
                        "cycle_duration_s": reference_cycle,
                        "stance_duration_s": reference_stance,
                        "swing_duration_s": reference_swing,
                        "stance_duty_factor": reference_stance / reference_cycle,
                    },
                    "detector_onset": {
                        "cycle_duration_s": onset_cycle,
                        "stance_duration_s": onset_stance,
                        "swing_duration_s": onset_swing,
                        "stance_duty_factor": onset_stance / onset_cycle,
                    },
                    "fsm_confirmed_state": {
                        "cycle_duration_s": confirmed_cycle,
                        "stance_duration_s": confirmed_stance,
                        "swing_duration_s": confirmed_swing,
                        "stance_duty_factor": confirmed_stance / confirmed_cycle,
                    },
                    "detector_onset_error": {
                        "cycle_duration_s": onset_cycle - reference_cycle,
                        "stance_duration_s": onset_stance - reference_stance,
                        "swing_duration_s": onset_swing - reference_swing,
                        "stance_duty_factor": (
                            onset_stance / onset_cycle
                            - reference_stance / reference_cycle
                        ),
                    },
                    "fsm_confirmed_state_error": {
                        "cycle_duration_s": confirmed_cycle - reference_cycle,
                        "stance_duration_s": confirmed_stance - reference_stance,
                        "swing_duration_s": confirmed_swing - reference_swing,
                        "stance_duty_factor": (
                            confirmed_stance / confirmed_cycle
                            - reference_stance / reference_cycle
                        ),
                    },
                }
            )

    def error_summary(key: str) -> dict[str, float] | None:
        if not rows:
            return None
        names = (
            "cycle_duration_s",
            "stance_duration_s",
            "swing_duration_s",
            "stance_duty_factor",
        )
        result: dict[str, float] = {}
        for name in names:
            values = np.asarray([float(row[key][name]) for row in rows])
            result[f"mean_signed_{name}"] = float(np.mean(values))
            result[f"max_abs_{name}"] = float(np.max(np.abs(values)))
        return result

    return {
        "alignment_complete": alignment_complete,
        "expected_cycle_count": expected_cycles,
        "reported_cycle_count": int(len(rows)),
        "per_cycle": rows,
        "detector_onset_error_summary": error_summary("detector_onset_error"),
        "fsm_confirmed_state_error_summary": error_summary(
            "fsm_confirmed_state_error"
        ),
    }


def _confirmed_state_hold_gate(
    times: np.ndarray,
    predicted_phase: np.ndarray,
    accepted: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Require each confirmed production transition to take and hold effect."""
    transitions: list[dict[str, Any]] = []
    accepted_by_event = _accepted_gait_transitions_for_gate(accepted)
    for event, expected_phase in (
        ("heel_strike", PHASE_STANCE),
        ("toe_off", PHASE_SWING),
    ):
        for item in accepted_by_event[event]:
            transitions.append(
                {
                    "event": event,
                    "confirmed_time_s": float(item["confirmed_time_s"]),
                    "expected_phase": int(expected_phase),
                }
            )
    transitions.sort(key=lambda item: float(item["confirmed_time_s"]))

    records: list[dict[str, Any]] = []
    all_hold = True
    for index, transition in enumerate(transitions):
        start_s = float(transition["confirmed_time_s"])
        end_s = (
            float(transitions[index + 1]["confirmed_time_s"])
            if index + 1 < len(transitions)
            else float(times[-1]) + 1e-12
        )
        interval = (times >= start_s - 1e-12) & (times < end_s - 1e-12)
        expected_phase = int(transition["expected_phase"])
        mismatches = interval & (predicted_phase != expected_phase)
        holds = bool(np.count_nonzero(interval) > 0 and not np.any(mismatches))
        all_hold = all_hold and holds
        records.append(
            {
                "event": str(transition["event"]),
                "confirmed_time_s": start_s,
                "hold_until_s_exclusive": end_s,
                "expected_phase": expected_phase,
                "samples": int(np.count_nonzero(interval)),
                "mismatch_count": int(np.count_nonzero(mismatches)),
                "mismatch_times_s": times[mismatches].tolist(),
                "holds_until_next_confirmed_transition": holds,
            }
        )
    return {
        "ok": bool(transitions and all_hold),
        "transition_count": int(len(transitions)),
        "records": records,
    }


def _phase_classification_gate(
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    reference_events: dict[str, np.ndarray],
    predicted_events: dict[str, np.ndarray],
    replay: dict[str, Any],
    *,
    prescribed_threshold_n: float,
    hs_tolerance_s: float,
    to_tolerance_s: float,
    sensor_dwell_s: float,
    reference_phase_mode: str = "instantaneous_grf",
    primary_event_time_field: str = "event_time_s",
    onset_events: dict[str, np.ndarray] | None = None,
) -> dict[str, Any]:
    """Gate phase state without hiding event timing behind phase accuracy.

    The GRF reference is immediate and causal.  The strict interval starts at
    the first complete prescribed HS because the preceding stance is already
    active at the left replay boundary.  Within that interval every mismatch
    must be confined to the *predeclared* event tolerance plus the causal dwell
    needed before the production FSM can change state.
    """
    sample_times = np.asarray(times, dtype=float)
    if reference_phase_mode == "instantaneous_grf":
        reference_phase = _reference_phase_from_prescribed_grf(
            prescribed_vertical_n,
            threshold_n=float(prescribed_threshold_n),
        )
    elif reference_phase_mode == "validated_event_intervals":
        reference_phase = _reference_phase_from_validated_events(
            sample_times,
            reference_events,
        )
    else:
        raise ValueError(f"unsupported phase reference mode: {reference_phase_mode}")
    if primary_event_time_field not in {"event_time_s", "confirmed_time_s"}:
        raise ValueError(
            f"unsupported primary event timestamp: {primary_event_time_field}"
        )
    predicted_phase = _fsm_phase_from_state_id(replay["state_id"])
    if sample_times.shape != reference_phase.shape:
        raise ValueError("phase time and signal arrays must have identical shapes")

    reference_hs = np.asarray(reference_events["heel_strike"], dtype=float)
    if reference_hs.size == 0:
        raise ValueError("phase gate requires at least one complete prescribed HS")
    strict_start_s = float(reference_hs[0])
    strict_mask = sample_times >= strict_start_s - 1e-12
    allowed_transition_mask = np.zeros(sample_times.shape, dtype=bool)
    transition_windows: list[dict[str, Any]] = []
    accepted_by_event = _accepted_gait_transitions_for_gate(replay["accepted"])
    alignment_complete = True

    for event, tolerance_s in (
        ("heel_strike", float(hs_tolerance_s)),
        ("toe_off", float(to_tolerance_s)),
    ):
        references = np.asarray(reference_events[event], dtype=float)
        predictions = np.asarray(predicted_events[event], dtype=float)
        accepted_items = accepted_by_event[event]
        if not (
            references.size == predictions.size == len(accepted_items)
        ):
            alignment_complete = False
        for index in range(
            min(references.size, predictions.size, len(accepted_items))
        ):
            reference_time_s = float(references[index])
            primary_time_s = float(predictions[index])
            onset_time_s = float(accepted_items[index]["event_time_s"])
            confirmed_time_s = float(
                accepted_items[index]["confirmed_time_s"]
            )
            onset_error_s = onset_time_s - reference_time_s
            primary_error_s = primary_time_s - reference_time_s
            confirmation_latency_s = confirmed_time_s - onset_time_s
            allowed_start_s = reference_time_s - tolerance_s
            additional_dwell_s = (
                sensor_dwell_s
                if primary_event_time_field == "event_time_s"
                else 0.0
            )
            allowed_end_s = (
                reference_time_s + tolerance_s + additional_dwell_s
            )
            in_window = (
                (sample_times >= allowed_start_s - 1e-12)
                & (sample_times <= allowed_end_s + 1e-12)
            )
            allowed_transition_mask |= in_window
            transition_windows.append(
                {
                    "event": event,
                    "event_index": int(index),
                    "reference_onset_s": reference_time_s,
                    "primary_event_time_field": primary_event_time_field,
                    "primary_detector_event_time_s": primary_time_s,
                    "primary_detector_error_s": primary_error_s,
                    "detector_onset_event_time_s": onset_time_s,
                    "fsm_state_change_confirmed_time_s": confirmed_time_s,
                    "detector_onset_error_s": onset_error_s,
                    "confirmation_latency_s": confirmation_latency_s,
                    "fixed_event_tolerance_s": tolerance_s,
                    "allowed_mismatch_window_start_s": allowed_start_s,
                    "allowed_mismatch_window_end_s": allowed_end_s,
                }
            )

    raw_mismatch = reference_phase != predicted_phase
    strict_mismatch = strict_mask & raw_mismatch
    forbidden_mismatch = strict_mismatch & ~allowed_transition_mask
    settled_mask = strict_mask & ~allowed_transition_mask
    full_metrics = _phase_agreement_metrics(
        reference_phase,
        predicted_phase,
        np.ones(sample_times.shape, dtype=bool),
    )
    strict_metrics = _phase_agreement_metrics(
        reference_phase,
        predicted_phase,
        strict_mask,
    )
    settled_metrics = _phase_agreement_metrics(
        reference_phase,
        predicted_phase,
        settled_mask,
    )
    forbidden_times = sample_times[forbidden_mismatch]
    duration_diagnostics = _phase_duration_diagnostics(
        reference_events,
        predicted_events if onset_events is None else onset_events,
        replay["accepted"],
    )
    confirmed_state_hold = _confirmed_state_hold_gate(
        sample_times,
        predicted_phase,
        replay["accepted"],
    )
    phase_ok = bool(
        alignment_complete
        and duration_diagnostics["alignment_complete"]
        and confirmed_state_hold["ok"]
        and not np.any(forbidden_mismatch)
        and settled_metrics["unknown_fsm_samples"] == 0
        and settled_metrics["samples"] > 0
    )
    return {
        "ok": phase_ok,
        "reference_definition": {
            "source": "prescribed left vertical GRF",
            "mode": reference_phase_mode,
            "rule": (
                "stance iff current Fy > prescribed_threshold_n; else swing"
                if reference_phase_mode == "instantaneous_grf"
                else "stance on each accepted reference HS-to-TO interval; "
                "swing on each accepted TO-to-HS interval"
            ),
            "causal": True,
            "lookahead_samples": 0,
            "threshold_n": float(prescribed_threshold_n),
        },
        "state_mapping": {
            str(STANCE_AFTER_HS): "stance",
            str(SWING_AFTER_TO): "swing",
            "other": "unknown",
        },
        "boundary_policy": {
            "strict_gate_start_s": strict_start_s,
            "excluded_left_boundary_samples": int(
                np.count_nonzero(~strict_mask)
            ),
            "reason": (
                "the prescribed stance already active at t_start has no "
                "observable HS inside the replay and production performs a "
                "partial-stance bootstrap"
            ),
        },
        "mismatch_policy": {
            "new_posthoc_tolerance_introduced": False,
            "rule": (
                "inside the strict interval, mismatches are allowed only in "
                "windows derived from the fixed HS/TO timing tolerance and "
                "the configured causal sensor dwell"
            ),
            "hs_fixed_tolerance_s": float(hs_tolerance_s),
            "toe_off_fixed_tolerance_s": float(to_tolerance_s),
            "configured_sensor_dwell_s": float(sensor_dwell_s),
            "primary_event_time_field": primary_event_time_field,
            "additional_dwell_applied_to_window_s": (
                float(sensor_dwell_s)
                if primary_event_time_field == "event_time_s"
                else 0.0
            ),
            "transition_windows": transition_windows,
        },
        "transition_alignment_complete": alignment_complete,
        "full_replay_raw_agreement": full_metrics,
        "strict_interval_raw_agreement": strict_metrics,
        "settled_outside_transition_windows_agreement": settled_metrics,
        "strict_interval_samples": int(np.count_nonzero(strict_mask)),
        "settled_samples": int(np.count_nonzero(settled_mask)),
        "allowed_transition_window_samples": int(
            np.count_nonzero(strict_mask & allowed_transition_mask)
        ),
        "forbidden_mismatch_count": int(np.count_nonzero(forbidden_mismatch)),
        "forbidden_mismatch_times_s": forbidden_times.tolist(),
        "confirmed_state_hold_gate": confirmed_state_hold,
        "phase_duration_diagnostics": duration_diagnostics,
        "_arrays": {
            "reference_phase": reference_phase,
            "predicted_phase": predicted_phase,
            "strict_mask": strict_mask,
            "allowed_transition_mask": allowed_transition_mask,
            "raw_mismatch": raw_mismatch,
            "forbidden_mismatch": forbidden_mismatch,
        },
    }


def _ordered_event_diagnostic(
    reference: np.ndarray,
    predicted: np.ndarray,
    tolerance_s: float,
) -> dict[str, Any]:
    """Expose near misses even when strict matching leaves events unmatched."""
    reference_values = np.asarray(reference, dtype=float)
    predicted_values = np.asarray(predicted, dtype=float)
    equal_counts = bool(reference_values.size == predicted_values.size)
    errors = (
        predicted_values - reference_values
        if equal_counts
        else np.asarray([], dtype=float)
    )
    max_abs = float(np.max(np.abs(errors))) if errors.size else float("nan")
    mean_signed = float(np.mean(errors)) if errors.size else float("nan")
    return {
        "equal_counts": equal_counts,
        "reference_count": int(reference_values.size),
        "predicted_count": int(predicted_values.size),
        "ordered_errors_s": errors.tolist(),
        "mean_signed_error_s": mean_signed,
        "all_detector_onsets_earlier_than_reference": bool(
            errors.size > 0 and np.all(errors < 0.0)
        ),
        "timing_max_abs_s": max_abs,
        "tolerance_s": float(tolerance_s),
        "gate_margin_s": (
            float(tolerance_s - max_abs) if math.isfinite(max_abs) else float("nan")
        ),
        "all_within_tolerance": bool(
            equal_counts
            and errors.size > 0
            and np.all(np.abs(errors) <= float(tolerance_s) + 1e-12)
        ),
    }


def _load_sensitivity_summary(
    raw_path: str,
    *,
    profile_source: dict[str, Any],
) -> dict[str, Any] | None:
    text = str(raw_path).strip()
    if not text:
        return None
    path = resolve_repo_path(text).resolve()
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("sensitivity summary must contain a JSON object")
    sources = raw.get("sources", {})
    detector_source = sources.get("detector_profile", {}) if isinstance(sources, dict) else {}
    if detector_source.get("sha256") != profile_source.get("sha256"):
        raise ValueError("sensitivity summary uses a different detector profile hash")
    if float(raw.get("sample_dt_s", 0.0) or 0.0) != 0.001:
        raise ValueError("sensitivity summary must use sample_dt_s=0.001")
    ordered = raw.get("ordered_event_diagnostics")
    if not isinstance(ordered, dict):
        raise ValueError("sensitivity summary lacks ordered_event_diagnostics")
    metrics = raw.get("metrics")
    strict_gate = raw.get("strict_gate")
    semantic_gate = raw.get("semantic_gate")
    phase_state = raw.get("phase_state_validation")
    if not isinstance(metrics, dict) or not isinstance(strict_gate, dict):
        raise ValueError("sensitivity summary lacks timing gate diagnostics")
    if not isinstance(semantic_gate, dict):
        raise ValueError("sensitivity summary lacks semantic_gate")
    if not isinstance(phase_state, dict):
        raise ValueError("sensitivity summary lacks phase_state_validation")
    event_diagnostics: dict[str, Any] = {}
    for event in ("heel_strike", "toe_off"):
        matched = metrics.get(event)
        ordered_event = ordered.get(event)
        if not isinstance(matched, dict) or not isinstance(ordered_event, dict):
            raise ValueError(f"sensitivity summary lacks {event} diagnostics")
        ordered_errors = np.asarray(
            ordered_event.get("ordered_errors_s", []),
            dtype=float,
        )
        event_diagnostics[event] = {
            "matched_pairs_max_abs_s": matched.get("timing_max_abs_s"),
            "matched_count": matched.get("matched_count"),
            "unmatched_reference": matched.get("unmatched_reference"),
            "unmatched_predicted": matched.get("unmatched_predicted"),
            "ordered_all_events_max_abs_s": ordered_event.get(
                "timing_max_abs_s"
            ),
            "ordered_all_events_tolerance_s": ordered_event.get(
                "tolerance_s"
            ),
            "ordered_all_events_gate_margin_s": ordered_event.get(
                "gate_margin_s"
            ),
            "all_detector_onsets_earlier_than_reference": bool(
                ordered_errors.size > 0 and np.all(ordered_errors < 0.0)
            ),
        }
    return {
        "role": "oversampled sensitivity diagnostic; not the runtime gate",
        "source": _source_record(path),
        "status": str(raw.get("status", "UNKNOWN")),
        "ok": bool(raw.get("ok", False)),
        "timing_gate_status": (
            "PASS" if bool(strict_gate.get("timing_ok", False)) else "FAIL"
        ),
        "semantic_gate_status": (
            "PASS" if bool(semantic_gate.get("ok", False)) else "FAIL"
        ),
        "phase_state_gate_status": (
            "PASS" if bool(phase_state.get("ok", False)) else "FAIL"
        ),
        "phase_state_agreement": {
            "strict_interval_raw": phase_state.get(
                "strict_interval_raw_agreement"
            ),
            "settled_outside_transition_windows": phase_state.get(
                "settled_outside_transition_windows_agreement"
            ),
            "forbidden_mismatch_count": phase_state.get(
                "forbidden_mismatch_count"
            ),
        },
        "sample_dt_s": float(raw["sample_dt_s"]),
        "ordered_event_diagnostics": ordered,
        "event_timing_diagnostics": event_diagnostics,
        "interpretation": (
            "At 1 ms semantic and phase-state gates pass, while timing fails. "
            "Matched-pair maxima omit unmatched near-misses; the ordered all-"
            "event maxima are the authoritative sensitivity diagnostic. This "
            "does not replace the 10 ms runtime-cadence gate."
        ),
    }


def _semantic_gate(
    replay: dict[str, Any],
    times: np.ndarray,
    loads: dict[str, np.ndarray],
    *,
    sensor_on_threshold_n: float,
    expected_complete_cycles: int,
) -> dict[str, Any]:
    accepted = replay["accepted"]
    gait_candidates = [
        item
        for item in replay["candidates"]
        if item.get("event") in {"heel_strike", "toe_off"}
    ]
    unmatched_transitions = [
        item
        for item in accepted
        if item.get("event") in {"heel_strike", "toe_off"}
        and not bool(item.get("matched_sensor_candidate"))
    ]
    candidate_keys = {
        (str(item["event"]), round(float(item["time"]), 10))
        for item in gait_candidates
    }
    accepted_keys = {
        (str(item["event"]), round(float(item["event_time_s"]), 10))
        for item in accepted
        if item.get("event") in {"heel_strike", "toe_off"}
    }
    unaccepted_candidates = sorted(candidate_keys - accepted_keys)

    hs_without_heel_load: list[dict[str, float]] = []
    for item in gait_candidates:
        if item.get("event") != "heel_strike":
            continue
        event_time = float(item["time"])
        index = int(np.argmin(np.abs(times - event_time)))
        heel_load = float(loads["left_heel"][index])
        if heel_load + 1e-9 < float(sensor_on_threshold_n):
            hs_without_heel_load.append(
                {"event_time_s": event_time, "heel_load_n": heel_load}
            )

    timeout_transitions = [
        item for item in accepted if item.get("event") == "timeout"
    ]
    final_payload = replay["fsm"].payload()
    observed_complete_cycles = int(final_payload["valid_cycle_count"])
    cycle_count_matches = bool(
        observed_complete_cycles == int(expected_complete_cycles)
    )
    ok = bool(
        not replay["invalid_steps"]
        and not timeout_transitions
        and not unmatched_transitions
        and not unaccepted_candidates
        and not hs_without_heel_load
        and cycle_count_matches
    )
    return {
        "ok": ok,
        "invalid_steps": replay["invalid_steps"],
        "timeout_transitions": timeout_transitions,
        "unmatched_accepted_transitions": unmatched_transitions,
        "unaccepted_sensor_gait_events": [
            {"event": event, "event_time_s": time_s}
            for event, time_s in unaccepted_candidates
        ],
        "heel_strikes_without_heel_on_threshold": hs_without_heel_load,
        "expected_complete_cycles": int(expected_complete_cycles),
        "observed_valid_cycles": observed_complete_cycles,
        "valid_cycle_count_matches": cycle_count_matches,
    }


def _plot(
    destination: Path,
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    reference_events: dict[str, np.ndarray],
    predicted_events: dict[str, np.ndarray],
    loads: dict[str, np.ndarray],
    replay: dict[str, Any],
    *,
    prescribed_threshold_n: float,
    sensor_on_threshold_n: float,
    sensor_off_threshold_n: float,
) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    axes[0].plot(times, prescribed_vertical_n, color="black", linewidth=1.2)
    axes[0].axhline(
        prescribed_threshold_n,
        color="0.4",
        linestyle=":",
        label=f"prescribed threshold {prescribed_threshold_n:g} N",
    )
    axes[0].set_ylabel("prescribed left Fy [N]")
    axes[0].legend(loc="upper right")

    axes[1].plot(times, loads["left_heel"], label="left heel", linewidth=1.2)
    axes[1].plot(times, loads["left_toe"], label="left toe", linewidth=1.2)
    axes[1].axhline(
        sensor_on_threshold_n,
        color="tab:green",
        linestyle=":",
        label=f"sensor ON {sensor_on_threshold_n:g} N",
    )
    axes[1].axhline(
        sensor_off_threshold_n,
        color="tab:red",
        linestyle=":",
        label=f"sensor OFF {sensor_off_threshold_n:g} N",
    )
    axes[1].set_ylabel("regional normal load [N]")
    axes[1].legend(loc="upper right", ncol=2)

    axes[2].step(
        times,
        replay["heel_contact"],
        where="post",
        label="heel stable contact",
    )
    axes[2].step(
        times,
        replay["toe_contact"] + 1.2,
        where="post",
        label="toe stable contact (+1.2)",
    )
    axes[2].step(
        times,
        replay["state_id"] / 5.0 + 2.4,
        where="post",
        label="FSM state / 5 (+2.4)",
        alpha=0.8,
    )
    axes[2].set_ylabel("contacts / FSM")
    axes[2].set_xlabel("prescribed time [s]")
    axes[2].legend(loc="upper right", ncol=3)

    for axis in axes:
        for event_time in reference_events["heel_strike"]:
            axis.axvline(event_time, color="tab:blue", alpha=0.35, linewidth=1.0)
        for event_time in reference_events["toe_off"]:
            axis.axvline(
                event_time,
                color="tab:blue",
                alpha=0.35,
                linewidth=1.0,
                linestyle="--",
            )
        for event_time in predicted_events["heel_strike"]:
            axis.axvline(event_time, color="tab:orange", alpha=0.45, linewidth=0.9)
        for event_time in predicted_events["toe_off"]:
            axis.axvline(
                event_time,
                color="tab:orange",
                alpha=0.45,
                linewidth=0.9,
                linestyle="--",
            )
        axis.grid(True, alpha=0.22)

    figure.suptitle(
        "AB06 prescribed IK replay — blue: prescribed GRF, orange: two-sensor FSM\n"
        "solid: heel strike, dashed: toe off"
    )
    figure.tight_layout()
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=170)
    plt.close(figure)


def _plot_phase_validation(
    destination: Path,
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    phase_validation: dict[str, Any],
) -> None:
    """Plot causal prescribed phase against the actual confirmed FSM state."""
    import matplotlib.pyplot as plt

    arrays = phase_validation["_arrays"]
    reference_phase = np.asarray(arrays["reference_phase"], dtype=int)
    predicted_phase = np.asarray(arrays["predicted_phase"], dtype=int)
    allowed_mask = np.asarray(arrays["allowed_transition_mask"], dtype=bool)
    raw_mismatch = np.asarray(arrays["raw_mismatch"], dtype=bool)
    forbidden_mismatch = np.asarray(arrays["forbidden_mismatch"], dtype=bool)
    predicted_for_plot = np.where(
        predicted_phase == PHASE_UNKNOWN,
        np.nan,
        predicted_phase,
    )

    figure, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    axes[0].plot(
        times,
        prescribed_vertical_n,
        color="black",
        linewidth=1.1,
        label="prescribed left Fy",
    )
    threshold_n = float(
        phase_validation["reference_definition"]["threshold_n"]
    )
    axes[0].axhline(
        threshold_n,
        color="tab:blue",
        linestyle=":",
        label=f"causal phase threshold {threshold_n:g} N",
    )
    axes[0].set_ylabel("prescribed Fy [N]")
    axes[0].legend(loc="upper right")

    axes[1].step(
        times,
        reference_phase,
        where="post",
        color="tab:blue",
        linewidth=1.5,
        label="reference from current prescribed Fy",
    )
    axes[1].step(
        times,
        predicted_for_plot,
        where="post",
        color="tab:orange",
        linewidth=1.2,
        linestyle="--",
        label="production FSM state after confirmation",
    )
    axes[1].set_yticks([PHASE_SWING, PHASE_STANCE])
    axes[1].set_yticklabels(["swing", "stance"])
    axes[1].set_ylim(-0.2, 1.2)
    axes[1].set_ylabel("gait phase")
    axes[1].legend(loc="upper right")

    axes[2].step(
        times,
        raw_mismatch.astype(float),
        where="post",
        color="tab:red",
        linewidth=1.2,
        label="raw phase mismatch",
    )
    axes[2].step(
        times,
        allowed_mask.astype(float) * 0.65,
        where="post",
        color="tab:green",
        linewidth=1.0,
        alpha=0.8,
        label="predeclared transition window",
    )
    if np.any(forbidden_mismatch):
        axes[2].scatter(
            times[forbidden_mismatch],
            np.full(np.count_nonzero(forbidden_mismatch), 1.1),
            color="black",
            marker="x",
            label="forbidden mismatch",
            zorder=5,
        )
    axes[2].set_ylim(-0.1, 1.25)
    axes[2].set_ylabel("mismatch / window")
    axes[2].set_xlabel("prescribed time [s]")
    axes[2].legend(loc="upper right", ncol=3)

    strict_start_s = float(
        phase_validation["boundary_policy"]["strict_gate_start_s"]
    )
    for axis in axes:
        axis.axvspan(
            float(times[0]),
            strict_start_s,
            color="0.75",
            alpha=0.25,
            label="left-boundary bootstrap" if axis is axes[1] else None,
        )
        for window in phase_validation["mismatch_policy"]["transition_windows"]:
            axis.axvline(
                float(window["reference_onset_s"]),
                color="tab:blue",
                linewidth=0.6,
                alpha=0.35,
            )
            axis.axvline(
                float(window["fsm_state_change_confirmed_time_s"]),
                color="tab:orange",
                linewidth=0.6,
                linestyle=":",
                alpha=0.45,
            )
        axis.grid(True, alpha=0.22)

    settled = phase_validation[
        "settled_outside_transition_windows_agreement"
    ]
    figure.suptitle(
        "AB06 prescribed phase validation — causal GRF reference vs two-sensor FSM\n"
        f"settled agreement {100.0 * float(settled['accuracy']):.3f}% | "
        f"forbidden mismatches "
        f"{phase_validation['forbidden_mismatch_count']}"
    )
    figure.tight_layout()
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=170)
    plt.close(figure)


def _write_phase_samples_csv(
    destination: Path,
    times: np.ndarray,
    prescribed_vertical_n: np.ndarray,
    phase_validation: dict[str, Any],
    replay: dict[str, Any],
) -> None:
    arrays = phase_validation["_arrays"]
    table = np.column_stack(
        (
            np.asarray(times, dtype=float),
            np.asarray(prescribed_vertical_n, dtype=float),
            np.asarray(arrays["reference_phase"], dtype=int),
            np.asarray(replay["state_id"], dtype=int),
            np.asarray(arrays["predicted_phase"], dtype=int),
            np.asarray(arrays["strict_mask"], dtype=int),
            np.asarray(arrays["allowed_transition_mask"], dtype=int),
            np.asarray(arrays["raw_mismatch"], dtype=int),
            np.asarray(arrays["forbidden_mismatch"], dtype=int),
        )
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        destination,
        table,
        delimiter=",",
        header=(
            "time_s,prescribed_left_fy_n,reference_phase_0swing_1stance,"
            "fsm_state_id,fsm_phase_minus1unknown_0swing_1stance,"
            "strict_gate_sample,allowed_transition_window,raw_mismatch,"
            "forbidden_mismatch"
        ),
        comments="",
        fmt=["%.9f", "%.9f", "%d", "%d", "%d", "%d", "%d", "%d", "%d"],
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def run_validation(args: argparse.Namespace) -> dict[str, Any]:
    if args.sample_dt <= 0.0:
        raise ValueError("sample_dt must be positive")
    if not 0.0 <= args.sensor_off_threshold_n < args.sensor_on_threshold_n:
        raise ValueError("sensor thresholds must satisfy 0 <= off < on")
    if args.sensor_dwell_s < 0.0:
        raise ValueError("sensor dwell must be non-negative")

    setup_path = resolve_repo_path(args.setup).resolve()
    profile_path = resolve_repo_path(args.profile).resolve()
    output_dir = resolve_repo_path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    setup = read_setup_xml(setup_path)
    profile = load_online_grf_profile(profile_path)
    current_runtime_dt = _current_training_segment_duration_s()
    runtime_fsm_config = _current_runtime_fsm_config()
    requested_sensor_contract = (
        float(args.sensor_on_threshold_n),
        float(args.sensor_off_threshold_n),
        float(args.sensor_dwell_s),
    )
    current_sensor_contract = (
        float(runtime_fsm_config.sensor_on_threshold_n),
        float(runtime_fsm_config.sensor_off_threshold_n),
        float(runtime_fsm_config.sensor_dwell_s),
    )
    if any(
        abs(requested - current) > 1e-12
        for requested, current in zip(
            requested_sensor_contract,
            current_sensor_contract,
        )
    ):
        raise ValueError(
            "sensor CLI values must match the current training config for the "
            f"exact-config gate: requested={requested_sensor_contract}, "
            f"current={current_sensor_contract}"
        )

    times = np.arange(
        float(setup.t_start),
        float(setup.t_end) + float(args.sample_dt) * 0.25,
        float(args.sample_dt),
        dtype=float,
    )
    if times.size < 2 or times[-1] > float(setup.t_end) + 1e-9:
        raise ValueError("invalid replay time grid")

    reference_events, reference_provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(args.prescribed_threshold_n),
        min_contact_duration_s=float(args.reference_min_contact_duration_s),
        min_cycle_duration_s=float(args.reference_min_cycle_duration_s),
    )
    samples = _sample_spheres(setup, profile, times, args.sea_plugin)
    loads, penetrations, aggregate = _regional_loads_and_penetrations(
        profile,
        samples,
    )
    prosthetic_kinematics = _prescribed_prosthetic_kinematics(setup, times)
    body_weight_n = _model_body_weight_n(setup.model_file)
    replay = _run_production_fsm(
        times,
        loads,
        penetrations,
        aggregate,
        prosthetic_kinematics,
        body_weight_n=body_weight_n,
        fsm_config=runtime_fsm_config,
    )
    predicted_events = _events_for_timing_gate(replay["accepted"])

    metrics = {
        "heel_strike": match_events(
            reference_events["heel_strike"],
            predicted_events["heel_strike"],
            float(args.hs_tolerance_s),
        ),
        "toe_off": match_events(
            reference_events["toe_off"],
            predicted_events["toe_off"],
            float(args.to_tolerance_s),
        ),
    }
    ordered_event_diagnostics = {
        "heel_strike": _ordered_event_diagnostic(
            reference_events["heel_strike"],
            predicted_events["heel_strike"],
            float(args.hs_tolerance_s),
        ),
        "toe_off": _ordered_event_diagnostic(
            reference_events["toe_off"],
            predicted_events["toe_off"],
            float(args.to_tolerance_s),
        ),
    }
    timing_ok = strict_event_pass(
        metrics,
        hs_tolerance_s=float(args.hs_tolerance_s),
        to_tolerance_s=float(args.to_tolerance_s),
    )
    semantic = _semantic_gate(
        replay,
        times,
        loads,
        sensor_on_threshold_n=float(runtime_fsm_config.sensor_on_threshold_n),
        expected_complete_cycles=int(reference_events["toe_off"].size),
    )

    prescribed = _external_grf(setup, times)["left"][:, 1]
    phase_validation = _phase_classification_gate(
        times,
        prescribed,
        reference_events,
        predicted_events,
        replay,
        prescribed_threshold_n=float(args.prescribed_threshold_n),
        hs_tolerance_s=float(args.hs_tolerance_s),
        to_tolerance_s=float(args.to_tolerance_s),
        sensor_dwell_s=float(runtime_fsm_config.sensor_dwell_s),
    )
    plot_path = output_dir / "two_sensor_prescribed_event_timing.png"
    _plot(
        plot_path,
        times,
        prescribed,
        reference_events,
        predicted_events,
        loads,
        replay,
        prescribed_threshold_n=float(args.prescribed_threshold_n),
        sensor_on_threshold_n=float(runtime_fsm_config.sensor_on_threshold_n),
        sensor_off_threshold_n=float(runtime_fsm_config.sensor_off_threshold_n),
    )
    phase_plot_path = output_dir / "two_sensor_prescribed_phase_state.png"
    phase_csv_path = output_dir / "two_sensor_prescribed_phase_samples.csv"
    _plot_phase_validation(
        phase_plot_path,
        times,
        prescribed,
        phase_validation,
    )
    _write_phase_samples_csv(
        phase_csv_path,
        times,
        prescribed,
        phase_validation,
        replay,
    )
    phase_report = {
        key: value
        for key, value in phase_validation.items()
        if key != "_arrays"
    }

    final_payload = replay["fsm"].payload()
    profile_source = _source_record(profile_path)
    sensitivity = _load_sensitivity_summary(
        args.sensitivity_summary,
        profile_source=profile_source,
    )
    ok = bool(
        timing_ok
        and semantic["ok"]
        and phase_report["ok"]
        and plot_path.is_file()
        and phase_plot_path.is_file()
        and phase_csv_path.is_file()
    )
    report = {
        "schema_version": 2,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "objective": (
            "current detector-profile left heel/toe loads -> production "
            "ProstheticPhaseFSM(two_sensor), validating both HS/TO timing and "
            "sample-by-sample stance/swing against prescribed GRF"
        ),
        "input_type": "prescribed_ik_replay",
        "time_range_s": [float(times[0]), float(times[-1])],
        "sample_dt_s": float(args.sample_dt),
        "samples": int(times.size),
        "sampling_contract": {
            "purpose": "match the cadence at which the env updates the FSM",
            "current_training_config": _source_record(CURRENT_TRAINING_CONFIG),
            "current_training_segment_duration_s": float(current_runtime_dt),
            "sample_dt_matches_current_training_runtime": bool(
                abs(float(args.sample_dt) - current_runtime_dt) <= 1e-12
            ),
        },
        "sources": {
            "setup": _source_record(setup_path),
            "model": _source_record(setup.model_file),
            "prescribed_kinematics": _source_record(setup.kinematics_file),
            "detector_profile": profile_source,
            "detector_profile_declared_source": str(profile.source),
            "reference": reference_provenance,
            "epic_gc_secondary_audit": _epic_gc_secondary_audit(
                reference_events["heel_strike"]
            ),
        },
        "detector_contract": {
            "fsm_class": "prosthetic_phase_fsm.ProstheticPhaseFSM",
            "event_source": "two_sensor",
            "exact_current_training_fsm_config": _fsm_config_payload(
                runtime_fsm_config
            ),
            "sensor_roles": {
                role: sphere.name
                for role, sphere in _left_sensor_spheres(profile).items()
            },
            "separate_loads_reconstruct_left_aggregate": True,
            "body_weight_n": float(body_weight_n),
        },
        "strict_gate": {
            "heel_strike": {
                "required_precision": 1.0,
                "required_recall": 1.0,
                "max_abs_error_s": float(args.hs_tolerance_s),
            },
            "toe_off": {
                "required_precision": 1.0,
                "required_recall": 1.0,
                "max_abs_error_s": float(args.to_tolerance_s),
            },
            "timing_ok": bool(timing_ok),
            "semantic_ok": bool(semantic["ok"]),
            "phase_state_ok": bool(phase_report["ok"]),
        },
        "events": {
            "reference": {
                key: values.tolist() for key, values in reference_events.items()
            },
            "accepted_for_timing_gate": {
                key: values.tolist() for key, values in predicted_events.items()
            },
            "all_accepted_transitions": replay["accepted"],
            "all_sensor_candidates": replay["candidates"],
            "all_sensor_edges": replay["sensor_edges"],
        },
        "metrics": metrics,
        "ordered_event_diagnostics": ordered_event_diagnostics,
        "sensitivity_diagnostic_1ms": sensitivity,
        "semantic_gate": semantic,
        "phase_state_validation": phase_report,
        "final_fsm": {
            "state_name": final_payload["state_name"],
            "valid_hs_count": final_payload["valid_hs_count"],
            "valid_to_count": final_payload["valid_to_count"],
            "valid_cycle_count": final_payload["valid_cycle_count"],
            "invalid_event_count": final_payload["invalid_event_count"],
            "timeout_exceeded": final_payload["timeout_exceeded"],
        },
        "load_summary_n": {
            role: {
                "min": float(np.min(values)),
                "median": float(np.median(values)),
                "max": float(np.max(values)),
            }
            for role, values in loads.items()
        },
        "artifacts": {
            "summary_json": _portable_path(output_dir / "summary.json"),
            "plot_png": _portable_path(plot_path),
            "phase_plot_png": _portable_path(phase_plot_path),
            "phase_samples_csv": _portable_path(phase_csv_path),
        },
        "notes": [
            (
                "The prescribed GRF is an independent runtime oracle, but the "
                "profile metadata declares that its geometry/contact parameters "
                "were originally calibrated against prescribed GRF."
            ),
            (
                "The partial stance already active at t_start is replayed so the "
                "FSM bootstrap is exercised; its boundary-censored TO is kept in "
                "diagnostics but excluded from precision/recall."
            ),
            "Event timing uses physical threshold-onset timestamps, not dwell confirmation time.",
            (
                "Phase-state validation separately uses confirmed_time_s, the "
                "causal instant at which the production FSM changes state."
            ),
        ],
    }
    safe_report = _json_safe(report)
    (output_dir / "summary.json").write_text(
        json.dumps(safe_report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe_report


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay the AB06 prescribed IK through the production two-sensor FSM "
            "and gate accepted HS/TO against prescribed GRF."
        )
    )
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--sensitivity-summary",
        default="",
        help=(
            "Optional summary.json from a separate --sample-dt 0.001 run; "
            "hash-checked and embedded as a non-gating sensitivity diagnostic."
        ),
    )
    parser.add_argument(
        "--sample-dt",
        type=float,
        default=_current_training_segment_duration_s(),
        help=(
            "FSM sampling cadence; 10 ms matches the current training/rollout "
            "segment_duration. Use 1 ms as an optional oversampling sensitivity test."
        ),
    )
    parser.add_argument("--sensor-on-threshold-n", type=float, default=5.0)
    parser.add_argument("--sensor-off-threshold-n", type=float, default=2.0)
    parser.add_argument("--sensor-dwell-s", type=float, default=0.030)
    parser.add_argument("--prescribed-threshold-n", type=float, default=20.0)
    parser.add_argument(
        "--reference-min-contact-duration-s",
        type=float,
        default=0.050,
    )
    parser.add_argument(
        "--reference-min-cycle-duration-s",
        type=float,
        default=0.300,
    )
    parser.add_argument("--hs-tolerance-s", type=float, default=0.050)
    parser.add_argument("--to-tolerance-s", type=float, default=0.080)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        report = run_validation(args)
    except Exception as exc:
        output_dir = resolve_repo_path(args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 1,
            "status": "ERROR",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        (output_dir / "summary.json").write_text(
            json.dumps(failure, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(failure, indent=2))
        return 2

    concise = {
        "status": report["status"],
        "heel_strike": report["metrics"]["heel_strike"],
        "toe_off": report["metrics"]["toe_off"],
        "semantic_gate": report["semantic_gate"],
        "phase_state_gate": {
            "ok": report["phase_state_validation"]["ok"],
            "strict_interval_raw_agreement": report[
                "phase_state_validation"
            ]["strict_interval_raw_agreement"],
            "settled_agreement": report["phase_state_validation"][
                "settled_outside_transition_windows_agreement"
            ],
            "forbidden_mismatch_count": report[
                "phase_state_validation"
            ]["forbidden_mismatch_count"],
        },
        "artifacts": report["artifacts"],
    }
    print(json.dumps(concise, indent=2, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
