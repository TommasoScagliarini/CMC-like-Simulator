"""Compare exported AB07 diagnostics without launching a native runtime.

The report records observed changes and inherited diagnostic references. It
does not adopt AB07 acceptance thresholds or qualify a learned policy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parents[4]
BASELINE = "results/ab07_v26_left_prescribed_9p01s_2026_10_06"
POSE_REPLAY = "validation/ab07_detector_investigation_2026_10_06/stage3/ik_contact_verification.json"
PIPELINE = "models/AB07_SEASEA_Threadmill/pipeline_verification.json"


def resolve(raw: str | Path) -> Path:
    value = Path(str(raw).replace("\\", "/"))
    return value.resolve() if value.is_absolute() else (REPO / value).resolve()


def load(source: Path) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"Non-finite JSON constant in {source}: {value}")

    return json.loads(source.read_text(encoding="utf-8"), parse_constant=reject_constant)


def fingerprint(source: Path) -> dict[str, str]:
    return {"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}


def nested(data: dict[str, Any], dotted: str) -> Any:
    current: Any = data
    for key in dotted.split("."):
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


METRICS = {
    "trace_rows_10ms": "count",
    "binary_samples_1ms": "count",
    "actor_cycle_summary.nominal_complete_cycle_count": "count",
    "actor_final_counters.valid_hs_count": "count",
    "actor_final_counters.valid_to_count": "count",
    "actor_final_counters.valid_cycle_count": "count",
    "actor_final_counters.invalid_event_count": "count",
    "actor_final_counters.resync_count": "count",
    "actor_final_counters.hs_cancelled_count": "count",
    "actor_final_counters.timeout_exceeded": "boolean_as_number",
    "actor_final_counters.last_stance_fraction": "fraction",
    "actor_final_counters.last_period_s": "s",
    "actor_cycle_summary.accepted_stance_duration_s.median": "s",
    "actor_cycle_summary.nominal_stance_duration_s.median": "s",
    "actor_cycle_summary.nominal_stance_fraction.median": "fraction",
    "duty_cycles_in_saved_window.binary_heel_fraction_1ms": "fraction",
    "duty_cycles_in_saved_window.binary_toe_fraction_1ms": "fraction",
    "duty_cycles_in_saved_window.binary_any_point_fraction_1ms": "fraction",
    "duty_cycles_in_saved_window.binary_functional_stance_fraction_10ms": "fraction",
    "duty_cycles_in_saved_window.primary_contact_over_20n_fraction_10ms": "fraction",
    "duty_cycles_in_saved_window.prescribed_contact_over_20n_fraction_10ms": "fraction",
    "primary_left_contact.vertical_force_n.mean": "N",
    "primary_left_contact.vertical_force_n.max": "N",
    "primary_left_contact.vertical_error_n.rms": "N",
    "primary_left_contact.vertical_correlation": "correlation",
    "primary_left_contact.relative_load_diagnostics.mean_force_ratio_to_oracle": "ratio",
    "primary_left_contact.relative_load_diagnostics.peak_force_ratio_to_oracle": "ratio",
    "primary_left_contact.relative_load_diagnostics.primary_vertical_impulse_ns": "N*s",
    "primary_left_contact.relative_load_diagnostics.prescribed_vertical_impulse_ns": "N*s",
    "primary_left_contact.relative_load_diagnostics.vertical_impulse_ratio_to_oracle": "ratio",
    "primary_left_contact.penetration_m.max": "m",
    "primary_left_contact.soft_exceedance_fraction": "fraction",
    "primary_left_contact.termination_exceedance_fraction": "fraction",
    "primary_left_contact.contact_20n.f1": "score",
    "primary_left_contact.slip_speed_m_s.rms": "m/s",
    "tracking_actual_vs_served_reference_deg.pros_knee_angle.rms": "deg",
    "tracking_actual_vs_served_reference_deg.pros_ankle_angle.rms": "deg",
    "so_diagnostics_at_saved_boundaries.tau_reserve_norm.rms": "mixed generalized-force norm",
    "so_diagnostics_at_saved_boundaries.muscle_capable_reserve_norm.rms": "mixed generalized-force norm",
    "so_diagnostics_at_saved_boundaries.unactuated_reserve_norm.rms": "mixed generalized-force norm",
    "so_diagnostics_at_saved_boundaries.muscle_share.mean": "fraction",
    "so_diagnostics_at_saved_boundaries.muscle_capable_share.mean": "fraction",
}


def compare(baseline: dict[str, Any], current: dict[str, Any]) -> dict[str, Any]:
    result = {}
    for dotted, unit in METRICS.items():
        old, new = nested(baseline, dotted), nested(current, dotted)
        delta = None
        if isinstance(old, (int, float)) and isinstance(new, (int, float)):
            if not math.isfinite(old) or not math.isfinite(new):
                raise ValueError(f"Non-finite metric: {dotted}")
            delta = new - old
        result[dotted] = {"unit": unit, "baseline": old, "current": new, "delta": delta}
    return result


def build_report(session: Path, baseline_session: Path, replay_file: Path, pipeline_file: Path) -> dict[str, Any]:
    analysis_file = session / "qualification_analysis.json"
    baseline_file = baseline_session / "qualification_analysis.json"
    receipt_file = session / "receipt.json"
    baseline_receipt_file = baseline_session / "receipt.json"
    current, baseline = load(analysis_file), load(baseline_file)
    receipt, old_receipt = load(receipt_file), load(baseline_receipt_file)
    replay, pipeline = load(replay_file), load(pipeline_file)
    if receipt.get("status") not in {"completed", "interrupted", "failed"}:
        raise ValueError("The native run has no final receipt")
    if current.get("status") == "provisional":
        raise ValueError("A provisional analysis cannot produce the final comparison")
    for name, source in (("receipt", receipt_file), ("trace", session / "trace.jsonl")):
        if current["source_files"][name]["sha256"] != fingerprint(source)["sha256"]:
            raise ValueError(f"Analysis source changed: {name}")
    source_checks = {
        "model_matches_prepared_pipeline": receipt["source_files"]["model"]["sha256"] == pipeline["model"]["sha256"],
        "ik_matches_prepared_pipeline": receipt["source_files"]["ik"]["sha256"] == pipeline["kinematics"]["sha256"],
        "frozen_profile_contents_match_historical_run":
            {key: value["sha256"] for key, value in receipt["profile_association"].items()}
            == {key: value["sha256"] for key, value in old_receipt["profile_association"].items()},
        "source_training_config_contents_unchanged": receipt["source_files"]["source_config"]["sha256"]
            == old_receipt["source_files"]["source_config"]["sha256"],
        "same_requested_time_range": receipt["requested_time_range"] == old_receipt["requested_time_range"],
        "same_diagnostic_stage": receipt["stage"] == old_receipt["stage"],
    }
    differing_env = {
        key: {"baseline": old_receipt["env_kwargs"].get(key), "current": receipt["env_kwargs"].get(key)}
        for key in set(old_receipt["env_kwargs"]) | set(receipt["env_kwargs"])
        if key not in {"output_dir", "setup_xml_path"} and old_receipt["env_kwargs"].get(key) != receipt["env_kwargs"].get(key)
    }
    source_checks["same_requested_environment_settings_except_setup_output_dir"] = not differing_env
    effective_env_differences = {
        key: {"baseline": old_receipt["effective_env_config"].get(key), "current": receipt["effective_env_config"].get(key)}
        for key in set(old_receipt["effective_env_config"]) | set(receipt["effective_env_config"])
        if key not in {"output_dir", "setup_xml_path"}
        and old_receipt["effective_env_config"].get(key) != receipt["effective_env_config"].get(key)
    }
    source_checks["same_effective_environment_settings_except_setup_output_dir"] = not effective_env_differences
    if not all(source_checks.values()):
        raise ValueError(f"Comparison contract differs: {source_checks}; {differing_env}")
    pose = replay["cases"]["AB07_ground_registered"]["filtered_window"]
    event_keys = ("confirmed_v26_diagnostic_matching", "delivered_v26_diagnostic_matching", "onset_holdout_reference_matching")
    anchored_stance = {}
    for name, data in (("baseline", baseline), ("current", current)):
        durations = [event["segment_end_time_s"] - event["segment_start_time_s"]
                     for event in data["actor_accepted_transitions"]
                     if event.get("closed_segment_type") == "stance"
                     and event.get("segment_valid") == 1.0 and event.get("anchor_geometry_valid") == 1.0]
        anchored_stance[name] = {
            "count": len(durations), "duration_s": durations,
            "median_duration_s": statistics.median(durations) if durations else None,
        }
    return {
        "schema_version": 1,
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "scope": "AB07 prescribed dynamics before/after ground registration and new IK; no learned policy",
        "status": "observed_comparison_requires_scientific_review",
        "source_files": {name: fingerprint(source) for name, source in {
            "comparison_script": Path(__file__), "current_analysis": analysis_file,
            "baseline_analysis": baseline_file, "current_receipt": receipt_file,
            "baseline_receipt": baseline_receipt_file, "saved_pose_replay": replay_file,
            "pipeline_verification": pipeline_file,
        }.items()},
        "comparison_contract_checks": source_checks,
        "environment_setting_differences": differing_env,
        "effective_environment_setting_differences": effective_env_differences,
        "grf_byte_vs_numeric_parity_from_preparation": replay["AB07_grf_parity"],
        "current_duration_completed": receipt.get("completed_requested_duration", False),
        "analyzed_time_ranges_s": {"baseline": baseline["analyzed_time_range_s"], "current": current["analyzed_time_range_s"]},
        "metrics": compare(baseline, current),
        "anchored_valid_stance_diagnostics": {
            "method": "Accepted stance transitions with segment_valid=1 and anchor_geometry_valid=1; startup partial stance excluded",
            **anchored_stance,
        },
        "event_matching": {event: {label: {"baseline": baseline["binary_event_comparison"][event][label],
                                            "current": current["binary_event_comparison"][event][label]}
                                  for label in event_keys} for event in ("heel_strike", "toe_off")},
        "ground_registered_saved_pose_replay": {
            "scope": "IK pose replay with production binary detector; no forward dynamics or actor FSM v3",
            "time_range_s": pose["time_range_s"], "pose_count": pose["pose_count"],
            "contact_fractions": pose["contact_fractions"],
            "detector_final": pose["v26_detector_fsm"]["final"],
            "nominal_oracle": pose["nominal_oracle"],
            "matching": {event: {kind: pose["event_comparison"][event][kind]["matching"]
                                   for kind in ("onset", "confirmed", "delivered")}
                         for event in ("heel_strike", "toe_off")},
        },
        "diagnostic_references": current["methodology"],
        "operational_penetration_thresholds_m": {name: current["primary_left_contact"][name]
            for name in ("existing_soft_threshold_m", "existing_termination_threshold_m", "existing_runner_hard_threshold_m")},
        "limitations": [
            "Inherited AB06 development and analog holdout timing references are diagnostics, not adopted AB07 acceptance gates.",
            "Duration completion and internal cycle validity do not qualify measured-contact agreement or learned-policy transfer.",
            "Online load, impulse and penetration statistics use 10 ms boundaries; internal peaks require the exported 1 ms outputs.",
            "Total reserve norms include different coordinate roles and generalized-force units; inspect per-coordinate reserve output separately.",
            "The model registration and new IK changed together; this comparison does not isolate their individual causal effects.",
            "Frozen AB06 surface velocity and residual fits remain unqualified assumptions for AB07.",
            "No replay of the old CMC poses through the new model was performed by this comparison.",
            "A single prescribed window does not qualify policy performance, additional starts, Windows or subject generalization.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--baseline", default=BASELINE)
    parser.add_argument("--pose-replay", default=POSE_REPLAY)
    parser.add_argument("--pipeline", default=PIPELINE)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    result = build_report(resolve(args.session), resolve(args.baseline), resolve(args.pose_replay), resolve(args.pipeline))
    output = resolve(args.out)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"Observed comparison saved: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
