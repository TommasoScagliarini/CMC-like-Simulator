"""Compare saved best47 control data without loading a policy or simulator.

All equality checks use the recorded values. Numerical scales are diagnostics,
not acceptance tolerances. Output is a new file; source records are untouched.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
from datetime import datetime, timezone
from pathlib import Path


REPO = Path(__file__).resolve().parents[4]
AUDIT = Path(__file__).resolve().parent
DEFAULT_BASELINE = REPO / "Trajectory Generator/runs/rollout/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-02_best_rollout"
DEFAULT_CURRENT = REPO / "Trajectory Generator/runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s"
MISSING = object()
SCALES = (1e-12, 1e-9, 1e-6, 1e-3)
CHANNELS = (
    "time", "raw_policy_action", "applied_policy_action",
    "actor_observation_before", "actor_observation_vector_before",
    "policy_segment_times", "policy_segment_values",
    "policy_segment_derivatives", "imitation_target_q",
    "imitation_target_qdot", "prosthetic_state",
    "reference_governor_diagnostics", "phase_fsm", "detector_sensors",
    "legacy_online_events", "online_events", "morphology_completed_segments",
    "morphology_ledger_diagnostics", "morphology_causal_diagnostics",
    "reward_terms",
)
CONTRACT_FIELDS = (
    "checkpoint", "setup_xml_path",
    "action_mode", "action_selection", "action_seed", "action_shape",
    "episode_start_offset_s", "imitation_initialize_to_target",
    "actor_feature_names", "observation_feature_names", "n_actor",
    "n_observation", "grf_mode", "online_grf_profile_file",
    "online_grf_detector_profile_file", "binary_phase_detector_profile_file",
    "phase_fsm_input_mode", "phase_sensor_on_threshold_n",
    "phase_sensor_off_threshold_n", "phase_sensor_dwell_s",
    "detector_sample_dt_s", "event_contract_id", "binary_phase_fsm_mode",
    "binary_phase_debounce_s", "binary_phase_event_contract_id",
    "online_grf_observation", "gait_clock_enable",
    "deployable_minimal_observation", "include_controller_state_observation",
    "include_controller_diagnostic_observation", "critic_privileged_observation",
    "prescribed_grf_disabled_sides", "online_grf_applied_sides",
    "grf_penetration_penalty_threshold_m", "grf_penetration_termination_m",
    "reward_config",
)
SUMMARY_METRICS = (
    "steps", "episode_return", "reward_mean", "reward_min", "reward_max",
    "action_abs_max", "applied_action_abs_max", "action_clipped_steps",
    "action_clipped_fraction", "pelvis_ty_min", "grf_penetration_max_m",
    "grf_penetration_samples", "reserve_norm_max_nm", "reserve_norm_samples",
    "residual_norm_max_nm", "residual_norm_samples", "phase_valid_hs_count",
    "phase_valid_to_count", "phase_valid_cycle_count", "invalid_event_count",
    "morphology_settled_segments", "morphology_settled_samples",
    "morphology_discarded_segments", "morphology_discarded_samples",
    "end_reason", "terminated", "truncated",
)


def relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def resolve(path: str | Path) -> Path:
    value = Path(str(path).replace("\\", "/")).expanduser()
    return value.resolve() if value.is_absolute() else (REPO / value).resolve()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path):
    def reject_constant(value):
        raise ValueError(f"Non-finite JSON value {value}: {path}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def numeric(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def readable(value):
    return "<missing>" if value is MISSING else value


def leaves(left, right, path=""):
    if isinstance(left, dict) and isinstance(right, dict):
        for key in sorted(set(left) | set(right)):
            yield from leaves(left.get(key, MISSING), right.get(key, MISSING), f"{path}.{key}".lstrip("."))
    elif isinstance(left, list) and isinstance(right, list):
        for index, pair in enumerate(itertools.zip_longest(left, right, fillvalue=MISSING)):
            yield from leaves(*pair, path=f"{path}[{index}]")
    else:
        yield path, left, right


class Comparison:
    def __init__(self):
        self.count = 0
        self.different = 0
        self.missing = 0
        self.numeric_count = 0
        self.sum_square = 0.0
        self.max_abs = 0.0
        self.first = None
        self.samples = []
        self.scale_first = {str(scale): None for scale in SCALES}

    def add(self, left, right, context=None, prefix=""):
        for path, old, new in leaves(left, right, prefix):
            context_value = dict(context or {}, field=path)
            self.count += 1
            if old is MISSING or new is MISSING:
                self.missing += 1
            elif numeric(old) and numeric(new):
                if not math.isfinite(old) or not math.isfinite(new):
                    raise ValueError(f"Non-finite recorded value: {context_value}")
                delta = float(new) - float(old)
                self.numeric_count += 1
                self.sum_square += delta * delta
                self.max_abs = max(self.max_abs, abs(delta))
                for scale in SCALES:
                    if abs(delta) > scale and self.scale_first[str(scale)] is None:
                        self.scale_first[str(scale)] = dict(context_value, historical=old, current=new, delta=delta)
            if old is MISSING or new is MISSING or old != new:
                self.different += 1
                item = dict(context_value, historical=readable(old), current=readable(new))
                if numeric(old) and numeric(new):
                    item["delta"] = float(new) - float(old)
                if self.first is None:
                    self.first = item
                if len(self.samples) < 5:
                    self.samples.append(item)

    def result(self):
        return {
            "leaf_comparisons": self.count,
            "different_recorded_values": self.different,
            "missing_leaves": self.missing,
            "numeric_comparisons": self.numeric_count,
            "numeric_max_abs_difference": self.max_abs,
            "numeric_rms_difference": math.sqrt(self.sum_square / self.numeric_count) if self.numeric_count else None,
            "first_exact_difference": self.first,
            "first_numeric_difference_above_diagnostic_scale": self.scale_first,
            "first_difference_samples": self.samples,
        }


def compare_trace(old, new, names):
    result = {channel: Comparison() for channel in CHANNELS}
    whole = Comparison()
    consistency = {"historical": [], "current": []}
    for label, rows in (("historical", old), ("current", new)):
        for row in rows:
            vector = row.get("actor_observation_vector_before", [])
            mapping = row.get("actor_observation_before", {})
            if len(vector) != len(names) or any(mapping.get(name, MISSING) != vector[index] for index, name in enumerate(names) if index < len(vector)):
                consistency[label].append({"step": row.get("step"), "time": row.get("time")})
    for index, pair in enumerate(itertools.zip_longest(old, new, fillvalue=MISSING)):
        left, right = pair
        context = {"row_index": index, "step": (left if left is not MISSING else right).get("step"),
                   "historical_time": left.get("time") if left is not MISSING else None,
                   "current_time": right.get("time") if right is not MISSING else None}
        if left is MISSING or right is MISSING:
            whole.add(left, right, context)
            continue
        whole.add(left, right, context)
        for channel in CHANNELS:
            result[channel].add(left.get(channel, MISSING), right.get(channel, MISSING), context, channel)
    return {"historical_rows": len(old), "current_rows": len(new),
            "full_record": whole.result(),
            "channels": {channel: comparison.result() for channel, comparison in result.items()},
            "actor_vector_mapping_inconsistencies": consistency}


def reward_schema_differences(old, new):
    added, removed, changed = set(), set(), {}
    for left, right in zip(old, new):
        old_terms, new_terms = left.get("reward_terms", {}), right.get("reward_terms", {})
        added.update(new_terms.keys() - old_terms.keys())
        removed.update(old_terms.keys() - new_terms.keys())
        for key in old_terms.keys() & new_terms.keys():
            if old_terms[key] != new_terms[key]:
                stats = changed.setdefault(key, Comparison())
                stats.add(old_terms[key], new_terms[key], {"step": left.get("step"), "historical_time": left.get("time"), "current_time": right.get("time")}, key)
    return {"current_only_fields": sorted(added), "historical_only_fields": sorted(removed),
            "changed_common_fields": {key: changed[key].result() for key in sorted(changed)}}


def sto_open(path):
    handle = path.open(encoding="utf-8-sig")
    metadata = {}
    for line in handle:
        value = line.strip()
        if value.lower() == "endheader":
            break
        if "=" in value:
            key, text = value.split("=", 1)
            metadata[key] = text
    else:
        handle.close()
        raise ValueError(f"Missing STO endheader: {path}")
    labels = next(handle).split()
    return handle, metadata, labels


def compare_sto(old_path, new_path):
    old_handle, old_meta, old_labels = sto_open(old_path)
    new_handle, new_meta, new_labels = sto_open(new_path)
    common = [name for name in old_labels if name in new_labels]
    old_indices = {name: old_labels.index(name) for name in common}
    new_indices = {name: new_labels.index(name) for name in common}
    columns = {name: Comparison() for name in common}
    counts = {"historical": 0, "current": 0}
    first_shape_diff = None
    try:
        old_rows = (line.split() for line in old_handle if line.strip())
        new_rows = (line.split() for line in new_handle if line.strip())
        for index, pair in enumerate(itertools.zip_longest(old_rows, new_rows, fillvalue=MISSING)):
            left, right = pair
            counts["historical"] += int(left is not MISSING)
            counts["current"] += int(right is not MISSING)
            if left is MISSING or right is MISSING:
                if first_shape_diff is None:
                    first_shape_diff = {"row_index": index, "historical_missing": left is MISSING, "current_missing": right is MISSING}
                continue
            if len(left) != len(old_labels) or len(right) != len(new_labels):
                raise ValueError(f"Malformed STO row {index}: {old_path}, {new_path}")
            left = [float(value) for value in left]
            right = [float(value) for value in right]
            context = {"row_index": index, "historical_time": left[0], "current_time": right[0]}
            for name in common:
                columns[name].add(left[old_indices[name]], right[new_indices[name]], context, name)
    finally:
        old_handle.close()
        new_handle.close()
    return {"historical_rows": counts["historical"], "current_rows": counts["current"],
            "historical_metadata": old_meta, "current_metadata": new_meta,
            "column_order_equal": old_labels == new_labels,
            "historical_only_columns": [name for name in old_labels if name not in new_labels],
            "current_only_columns": [name for name in new_labels if name not in old_labels],
            "first_row_count_difference": first_shape_diff,
            "columns": {name: stats.result() for name, stats in columns.items()}}


def csv_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--current", type=Path, default=DEFAULT_CURRENT)
    parser.add_argument("--out", type=Path, default=AUDIT / "comparison_metrics.json")
    args = parser.parse_args()
    baseline, current, out = resolve(args.baseline), resolve(args.current), resolve(args.out)
    receipt = read_json(current / "receipt.json")
    if receipt.get("status") != "completed":
        raise RuntimeError("Comparison requires the final completed runtime receipt")
    old_summary = read_json(baseline / "rollout_summary.json")
    new_summary = read_json(current / "rollout_summary.json")
    old_trace = read_json(baseline / "rollout_policy_trace.json")
    new_trace = read_json(current / "rollout_policy_trace.json")
    old_reset = read_json(baseline / "rollout_reset_diagnostics.json")
    new_reset = read_json(current / "rollout_reset_diagnostics.json")
    reset = Comparison()
    reset.add(old_reset, new_reset)
    contracts = {}
    for field in CONTRACT_FIELDS:
        old, new = old_summary.get(field, MISSING), new_summary.get(field, MISSING)
        if (field.endswith("_file") or field in ("checkpoint", "setup_xml_path")) and old is not MISSING and new is not MISSING:
            old, new = relative(resolve(old)), relative(resolve(new))
        contracts[field] = {"equal": old is not MISSING and new is not MISSING and old == new,
                            "historical": readable(old), "current": readable(new)}
    metrics = {}
    for field in SUMMARY_METRICS:
        old, new = old_summary.get(field, MISSING), new_summary.get(field, MISSING)
        metrics[field] = {"historical": readable(old), "current": readable(new)}
        if numeric(old) and numeric(new):
            metrics[field]["delta"] = float(new) - float(old)
        metrics[field]["equal"] = old is not MISSING and new is not MISSING and old == new
    inputs = [Path(__file__).resolve(), current / "receipt.json"]
    for root in (baseline, current):
        inputs += [root / name for name in ("rollout_summary.json", "rollout_reset_diagnostics.json", "rollout_policy_trace.json")]
    sto = {}
    old_files = {path.name: path for path in (baseline / "sim_outputs").glob("*.sto")}
    new_files = {path.name: path for path in (current / "sim_outputs").glob("*.sto")}
    for name in sorted(set(old_files) & set(new_files)):
        sto[name] = compare_sto(old_files[name], new_files[name])
        inputs.extend((old_files[name], new_files[name]))
    events = {}
    for old_path in sorted((baseline / "sim_outputs").glob("*.csv")):
        new_path = current / "sim_outputs" / old_path.name
        if new_path.exists():
            left, right = csv_rows(old_path), csv_rows(new_path)
            event_stats = Comparison()
            event_stats.add(left, right)
            events[old_path.name] = {"historical_events": left, "current_events": right,
                                     "comparison": event_stats.result()}
            inputs.extend((old_path, new_path))
    source_checks = {}
    for label, fingerprint in receipt.get("source_fingerprints", {}).items():
        path = resolve(fingerprint["path"])
        actual = sha256(path) if path.is_file() else None
        source_checks[label] = {"path": relative(path), "receipt_sha256": fingerprint["sha256"],
                                "observed_sha256": actual, "unchanged_since_preflight": actual == fingerprint["sha256"]}
    payload = {
        "schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Saved-data comparison of one current AB06 V26C best47 control against the 2026-09-02 deterministic central rollout",
        "status": "posthoc_comparison_requires_scientific_interpretation",
        "baseline": relative(baseline), "current": relative(current),
        "runtime_receipt_status": receipt["status"],
        "completed_requested_duration": receipt.get("completed_requested_duration"),
        "source_integrity_checks": source_checks,
        "protocol_contracts": contracts, "summary_metrics": metrics,
        "reset_diagnostics": reset.result(),
        "policy_trace": compare_trace(old_trace, new_trace, old_summary["actor_feature_names"]),
        "reward_schema_differences": reward_schema_differences(old_trace, new_trace),
        "sto_tables": sto,
        "historical_only_sto_files": sorted(set(old_files) - set(new_files)),
        "current_only_sto_files": sorted(set(new_files) - set(old_files)),
        "event_csv": events,
        "source_files": [{"path": relative(path), "sha256": sha256(path)} for path in inputs],
        "limitations": [
            "Diagnostic numeric scales are reporting aids, not scientific or safety acceptance gates.",
            "JSON channel aggregate differences mix feature units; STO differences are reported per column.",
            "The saved policy trace contains actor observations, not the complete 84D observation values; complete feature names/order are compared from summaries.",
            "Neither trace saves the reward returned at each env.step. Returned episode reward and mean/min/max are compared from summaries; reward_terms are compared separately.",
            "Historical source fingerprints were not recorded, so current source integrity cannot establish historical source identity.",
            "CSV rows are compared in saved order; STO rows are compared in order with explicit time-column differences, without interpolation.",
            "No policy inference, simulator integration, feature adapter, reward reconstruction or acceptance-threshold modification is performed.",
        ],
    }
    with out.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"output": relative(out), "steps": metrics["steps"],
                      "episode_return": metrics["episode_return"],
                      "actor_first_difference": payload["policy_trace"]["channels"]["actor_observation_vector_before"]["first_exact_difference"],
                      "action_first_difference": payload["policy_trace"]["channels"]["raw_policy_action"]["first_exact_difference"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
