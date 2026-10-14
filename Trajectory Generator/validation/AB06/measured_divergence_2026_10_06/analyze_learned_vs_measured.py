"""Compare saved AB06 learned control with its measured left GRF oracle.

No simulation, model, environment, policy or detector is constructed. Existing
saved-data analysis helpers are reused; all outputs live in this new directory.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[4]
AUDIT = Path(__file__).resolve().parent
DEFAULT_RUN = REPO / "Trajectory Generator/runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s"
HELPER = REPO / "Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/compare_transfer.py"


def helpers():
    spec = importlib.util.spec_from_file_location("saved_transfer_comparison", HELPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recorded_path(raw: str) -> Path:
    """Resolve a captured workspace path after moving between macOS/Windows."""
    normalized = str(raw).replace("\\", "/")
    candidate = Path(normalized).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    for marker in ("/models/", "/Trajectory Generator/", "/online_grf_profiles/", "/validation/"):
        if marker in normalized:
            relocated = REPO / marker.strip("/") / normalized.split(marker, 1)[1]
            if relocated.is_file():
                return relocated.resolve()
    if not candidate.is_absolute() and (REPO / candidate).is_file():
        return (REPO / candidate).resolve()
    raise FileNotFoundError(raw)


def summarize_matching(events, nominal, matcher, limits):
    result = {}
    for name, tolerance in (("heel_strike", limits["sensor_holdout_heel_strike_max_error_s"]), ("toe_off", limits["sensor_holdout_toe_off_max_error_s"])):
        selected = [event for event in events if event.get("event") == name]
        onset = [float(event["event_time_s"]) for event in selected]
        result[name] = {"onset_times_s": onset, "matching": matcher(nominal[name], onset, tolerance), "tolerance_s": tolerance}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=AUDIT / "learned_vs_measured.json")
    args = parser.parse_args()
    h = helpers()
    run = h.resolve(args.rollout_dir)
    receipt_path = run / "receipt.json"
    receipt, summary = h.load(receipt_path), h.load(run / "rollout_summary.json")
    if receipt.get("status") != "completed" or not summary.get("ok"):
        raise ValueError("This analysis requires finalized learned control artifacts.")
    start, end = float(receipt["actual_episode_start_s"]), float(receipt["actual_episode_end_s"])
    rows = h.load(run / "rollout_policy_trace.json")
    if not isinstance(rows, list):
        raise ValueError("Unexpected native policy trace schema.")
    if (run / "runtime_diagnostics.jsonl").exists():
        raise ValueError("A runtime sidecar is now available; explicitly adapt the channel inventory before analyzing it.")
    sources = [receipt_path, run / "rollout_summary.json", run / "rollout_policy_trace.json", HELPER, h.ANALYZER, h.MATCHER, h.CROSSING, h.TOLERANCES, h.HOLDOUT]
    source_config = receipt["effective_simulator_config"]
    prefix = source_config["output_prefix"]
    external = recorded_path(source_config["external_loads_xml"])
    external_root = ET.parse(external).getroot()
    measured = external.parent / external_root.findtext(".//ExternalLoads/datafile")
    measured = measured.resolve()
    oracle_path = run / "sim_outputs" / f"{prefix}_gait_events.csv"
    sources.extend((external, measured, oracle_path))
    with oracle_path.open(newline="", encoding="utf-8") as stream:
        oracle_rows = [row for row in csv.DictReader(stream) if row["side"] == "left"]
    threshold_values = {float(row["threshold_n"]) for row in oracle_rows}
    force_columns = {row["source_force"] for row in oracle_rows}
    if threshold_values != {20.0} or len(force_columns) != 1:
        raise ValueError("Measured reference CSV does not preserve the expected left 20 N contract.")
    force_column = next(iter(force_columns))
    reader = h.pure_function(h.ANALYZER, "read_storage")
    oracle_cycles = h.pure_function(h.ANALYZER, "oracle_cycles")
    matcher = h.pure_function(h.MATCHER, "match_events")
    crossing = h.pure_function(h.CROSSING, "_crossing_times")
    ft, flabels, fvalues, _ = reader(measured)
    force = h.table(run, "online_grf", reader, sources)
    times = force["time"]
    if len(times) != 5000 or abs(times[0] - start) > 1e-8 or abs(times[-1] - (end - .001)) > 1e-8:
        raise ValueError("The recorded learned episode no longer has the audited 5 s pre-integration grid.")
    h.require_coverage(times, ft, "measured AB06 GRF")
    cycles, nominal = oracle_cycles(oracle_path, start, end)
    limits = h.load(h.HOLDOUT)
    old_audit = REPO / "Trajectory Generator/validation/AB06/v26c_best47_control_2026_10_06"
    protected = sorted(set(sources + list(run.rglob("*")) + list(old_audit.rglob("*"))))
    protected = [path for path in protected if path.is_file()]
    before = {h.rel(path): h.fingerprint(path)["sha256"] for path in protected}
    force_analysis = h.load_analysis(force, times, (ft, fvalues[:, flabels.index(force_column)]), 20.0, crossing, nominal, limits, matcher)
    phase = h.phase_analysis(rows, nominal, h.inherited_tolerances(), limits, matcher, start)
    delivered_adapter = [event for row in rows for event in row.get("online_events", []) if event.get("side") == "left" and event.get("v26_source") == "binary_phase_fsm_v26"]
    interior_nominal = {name: [time for time in values if time > start + 1e-8] for name, values in nominal.items()}
    matching = {
        "actor_retained_interior_onset": summarize_matching(phase["ordinary_actor_events_retained_after_repairs"], interior_nominal, matcher, limits),
        "binary_active_adapter_delivered_including_reset_boundary": summarize_matching(delivered_adapter, nominal, matcher, limits),
        "binary_active_adapter_delivered_interior_onset": summarize_matching(delivered_adapter, interior_nominal, matcher, limits),
    }
    after = {h.rel(path): h.fingerprint(path)["sha256"] for path in protected}
    if before != after:
        raise ValueError("Protected source artifacts changed during saved-data analysis.")
    source_fingerprints = receipt.get("source_fingerprints", receipt.get("source_files", {}))
    original_grf_record = source_fingerprints["prescribed_grf"]
    measured_hash = h.fingerprint(measured)["sha256"]
    if measured_hash != original_grf_record["sha256"]:
        raise ValueError("Measured GRF no longer matches the run receipt.")
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "condition": "AB06 learned V26C best47 deterministic nominal control, seed 123, saved 5 s rollout; this is a neural-policy baseline",
        "run": h.rel(run), "script": h.fingerprint(Path(__file__)),
        "episode_requested_s": [start, end], "saved_force_timestamps_s": [float(times[0]), float(times[-1])], "saved_force_samples": len(times),
        "policy": {key: receipt.get(key) for key in ("checkpoint", "checkpoint_logical_iteration", "action_selection", "seed")},
        "recorded_channel_inventory": {"left_force_STO": "5000 pre-integration samples at 1 ms", "actor_journal": "500 original policy trace boundaries at 10 ms, including accepted transitions and repairs", "binary_raw_journal": "not recorded: original trace lacks binary_phase_fsm and runtime sidecar", "binary_boolean_samples_1ms": "not recorded", "binary_active_adapter_delivered_events": "left online_events with v26_source=binary_phase_fsm_v26; transported accepted event subset, not complete raw binary journal", "online_threshold_csv": "independent threshold events; not actor or binary journal"},
        "measured_oracle": {"source": h.fingerprint(measured), "matches_receipt_source_fingerprint": True, "reference_csv": h.fingerprint(oracle_path), "vertical_column": force_column, "threshold_n": 20.0, "event_times_s": nominal, "complete_cycles": cycles, "initial_HS_at_reset": [time for time in nominal["heel_strike"] if abs(time-start) < 1e-8], "interior_events_s": interior_nominal, "boundary_note": "The oracle CSV includes HS at the reset boundary. The learned actor starts inside a partial stance and is not assigned an invented HS; both full-window and interior matching are reported."},
        "learned_force_vs_own_measured_grf": force_analysis,
        "actor_journal_analysis": phase,
        "additional_onset_matching": matching,
        "binary_active_adapter_delivered_event_records": delivered_adapter,
        "native_rollout_summary": {key: summary[key] for key in ("steps", "episode_return", "phase_valid_hs_count", "phase_valid_to_count", "phase_valid_cycle_count", "invalid_event_count", "end_reason")},
        "matching_method": "Existing greedy one-to-one match_events helper. Onset diagnostic tolerances inherited from online_grf_acceptance_thresholds.json: HS 0.12 s, TO 0.18 s; these are reported diagnostic references, not new acceptance gates.",
        "provenance": {"sources": [h.fingerprint(path) for path in sorted(set(sources))], "protected_file_count": len(before), "all_protected_source_files_unchanged": before == after, "protected_file_sha256": before},
        "scope": "Saved-data read-only comparison. No new integration or policy inference. No profile, model, run, controller or old audit modifications.",
    }
    empty_statistics = []
    result = h.compliant_statistics(result, empty_statistics)
    result["empty_matching_statistics_converted_to_null"] = empty_statistics
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "vertical_impulse_Ns": force_analysis["vertical_impulse_ns"], "measured_vertical_impulse_Ns": force_analysis["reference_vertical_impulse_ns"], "impulse_ratio": force_analysis["vertical_impulse_ratio_to_oracle"], "actor_onset_full_window": phase["event_matching"]["actor_retained_after_repairs"], "additional_matching": matching, "protected_unchanged": before == after}, indent=2))


if __name__ == "__main__":
    main()
