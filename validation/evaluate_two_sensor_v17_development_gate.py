"""Evaluate the fixed V17 detector candidate from pinned open development evidence.

This gate is intentionally retrospective and fail-closed.  V17 copies V13's
physical detector profile and its 0.5/0.25 N, 30 ms debounce contract.  The
only admissible detector evidence here is therefore the twelve
``V13_BASELINE``/``fine_1ms`` rows already published by the V14.2 development
run for trials 02/04/08.

The V14.2 products do not contain the raw 1 ms heel/toe trace required to
prove 10 ms batch-consumer parity.  The gate never synthesizes that evidence:
batch parity and policy-delivery checks are reported as NOT_EXECUTED.  A
sequential failure which cannot be repaired by the sub-sample change from the
historical interpolated reference to the canonical 1 ms reference is enough
to close the single-candidate V17 stage.

No protected or reserve trial has a path or dispatch branch in this module.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Iterable, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from validation import readiness_gatekeeper as gate  # noqa: E402


SCHEMA_VERSION = 1
EVENT_CONTRACT_ID = "primary_grf_split_v1+two_sensor_highrate_v1"
AUTHORIZED_DEVELOPMENT_TRIALS = ("02", "04", "08")
FORBIDDEN_TRIALS = frozenset({"01", "03", "05", "06", "07"})

V14_RUN_ROOT = (
    REPO_ROOT
    / "validation/two_sensor_cross_speed_v14_2_runs/"
    "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery"
)
DEFAULT_V14_MANIFEST = V14_RUN_ROOT / "manifest.json"
DEFAULT_V14_METRICS = V14_RUN_ROOT / "development_isolated_unit_metrics.csv"
DEFAULT_V14_DETAILS = V14_RUN_ROOT / "development_stage1_details.jsonl"
DEFAULT_V17_CONTRACT = REPO_ROOT / "validation/two_sensor_v17_high_rate_contract.json"
DEFAULT_ORACLE_BUILDER = REPO_ROOT / "validation/build_canonical_grf_event_oracle.py"
DEFAULT_ORACLE_MANIFEST = (
    REPO_ROOT
    / "validation/canonical_event_oracles/2026-08-03_v17_development/manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "validation/two_sensor_v17_development_runs/"
    "2026-08-03_v13_fine_1ms_evidence/development_gate_receipt.json"
)

# These two files are the complete trial-derived evidence surface used here.
# Changing either one requires a new explicitly authorized development stage.
EXPECTED_V14_MANIFEST_SHA256 = (
    "5fc4ca8f5600a7940e384b57aaa5497a13907fefc7df4c681c379d65e0a13d3c"
)
EXPECTED_V14_METRICS_SHA256 = (
    "805aa3f4db9153fdbfe7ea6726b282be6f43e2417278cfe62620db6150fda3cf"
)
EXPECTED_V14_DETAILS_SHA256 = (
    "3925d59830ce8832d4524b62ab9bd7a11ed6f2c5866e8b6d1a1fb2cf5d036aa7"
)
EXPECTED_ORACLE_MANIFEST_SHA256 = (
    "c743fe0e177e3f5e2c5d12cb4e4def79f18fcfa9dd0fb2b3c018f92364c58ccd"
)
EXPECTED_V13_PROFILE_SHA256 = (
    "06e442bed2e4d2a2bc61f64e3379332d5e8d90695bcc9123585b0fbd008894fc"
)

# V14.2 linearly interpolated a 20 N crossing between adjacent samples.  The
# canonical oracle timestamps the first 1 ms sample on the new side of 20 N,
# so the reference-only timestamp change is strictly bounded by one sample.
CANONICAL_REFERENCE_SHIFT_BOUND_S = 0.001

REQUIRED_NUMERIC_FIELDS = (
    "precision",
    "recall",
    "confirmed_fsm_stance_f1",
    "confirmed_fsm_stance_iou",
    "max_abs_hs_error_s",
    "max_abs_toe_off_error_s",
    "minimum_causal_toe_clear_before_next_hs_onset_s",
    "forbidden_phase_mismatch_count",
    "invalid_or_timeout_transition_count",
    "unaccepted_sensor_gait_event_count",
    "incomplete_heel_to_forefoot_transfer_count",
    "transfer_both_latches_off_sample_count",
    "to_candidates_before_min_stance_count",
    "unknown_fsm_phase_samples",
    "expected_reference_hs_count",
    "expected_reference_to_count",
    "expected_valid_cycle_count",
    "predicted_hs_count",
    "predicted_to_count",
    "observed_valid_cycle_count",
    "sample_dt_s",
    "geometry_displacement_from_v13_m",
    "heel_radius_reduction_from_v13_mm",
    "toe_radius_reduction_from_v13_mm",
)


class V17DevelopmentGateError(ValueError):
    """Raised when provenance or evidence cannot support a safe decision."""


def _source_record(path: Path) -> dict[str, Any]:
    source = path.resolve()
    try:
        relative = source.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V17DevelopmentGateError(f"source escapes repository: {source}") from exc
    if not source.is_file():
        raise V17DevelopmentGateError(f"missing source: {relative}")
    return {
        "path": relative,
        "sha256": gate.sha256_file(source),
        "size_bytes": int(source.stat().st_size),
    }


def _require_sha256(path: Path, expected: str, *, label: str) -> dict[str, Any]:
    if not gate.is_sha256(expected):
        raise V17DevelopmentGateError(f"{label} expected hash is invalid")
    record = _source_record(path)
    if record["sha256"] != expected:
        raise V17DevelopmentGateError(
            f"{label} hash mismatch: expected {expected}, observed {record['sha256']}"
        )
    return record


def _strict_object(path: Path, *, label: str) -> dict[str, Any]:
    value = gate.load_json_strict(path)
    if not isinstance(value, Mapping):
        raise V17DevelopmentGateError(f"{label} must be a JSON object")
    return dict(value)


def _portable_repo_path(value: Any, *, label: str) -> Path:
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise V17DevelopmentGateError(f"{label} must be a trimmed relative path")
    if "\\" in value or ":" in value:
        raise V17DevelopmentGateError(f"{label} must use portable '/' separators")
    posix = PurePosixPath(value)
    windows = PureWindowsPath(value)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise V17DevelopmentGateError(f"{label} must be repository-relative")
    if not posix.parts or any(part in {"", ".", ".."} for part in posix.parts):
        raise V17DevelopmentGateError(f"{label} contains an unsafe component")
    path = REPO_ROOT.joinpath(*posix.parts).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V17DevelopmentGateError(f"{label} escapes repository") from exc
    return path


def _read_csv(path: Path) -> list[dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None or len(reader.fieldnames) != len(set(reader.fieldnames)):
                raise V17DevelopmentGateError("V14.2 metrics header is missing or duplicated")
            rows = [dict(row) for row in reader]
    except (OSError, UnicodeError, csv.Error) as exc:
        raise V17DevelopmentGateError(f"cannot read V14.2 metrics: {exc}") from exc
    if not rows:
        raise V17DevelopmentGateError("V14.2 metrics are empty")
    return rows


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    raise V17DevelopmentGateError(
                        f"V14.2 details contain a blank line at {line_number}"
                    )
                value = gate.loads_json_strict(
                    line, source=f"{path.as_posix()}:{line_number}"
                )
                if not isinstance(value, Mapping):
                    raise V17DevelopmentGateError(
                        f"V14.2 detail {line_number} is not an object"
                    )
                records.append(dict(value))
    except (OSError, UnicodeError) as exc:
        raise V17DevelopmentGateError(f"cannot read V14.2 details: {exc}") from exc
    if not records:
        raise V17DevelopmentGateError("V14.2 details are empty")
    return records


def _finite(value: Any, *, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise V17DevelopmentGateError(f"{label} is not numeric") from exc
    if not math.isfinite(number):
        raise V17DevelopmentGateError(f"{label} is not finite")
    return number


def _boolean(value: Any, *, label: str) -> bool:
    if value == "True" or value is True:
        return True
    if value == "False" or value is False:
        return False
    raise V17DevelopmentGateError(f"{label} is not a strict boolean")


def _selected_rows(rows: Sequence[Mapping[str, str]]) -> list[dict[str, str]]:
    all_trial_ids = {str(row.get("trial_id", "")) for row in rows}
    unexpected = sorted(all_trial_ids - set(AUTHORIZED_DEVELOPMENT_TRIALS))
    if unexpected:
        raise V17DevelopmentGateError(
            f"metrics expose non-development trial ids: {unexpected}"
        )
    selected = [
        dict(row)
        for row in rows
        if row.get("candidate_id") == "V13_BASELINE"
        and row.get("cadence") == "fine_1ms"
    ]
    expected_units = {
        (trial_id, str(plateau))
        for trial_id in AUTHORIZED_DEVELOPMENT_TRIALS
        for plateau in range(1, 5)
    }
    observed_units = {
        (str(row.get("trial_id")), str(row.get("plateau_index")))
        for row in selected
    }
    if len(selected) != 12 or observed_units != expected_units:
        raise V17DevelopmentGateError(
            "expected exactly the 12 V13_BASELINE/fine_1ms development units"
        )
    if len(observed_units) != len(selected):
        raise V17DevelopmentGateError("duplicate V13 fine_1ms development unit")
    return sorted(
        selected,
        key=lambda row: (str(row["trial_id"]), int(row["plateau_index"])),
    )


def _selected_details(records: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    exposed_trials: set[str] = set()
    selected: list[dict[str, Any]] = []
    for record in records:
        row = record.get("row")
        if not isinstance(row, Mapping):
            raise V17DevelopmentGateError("V14.2 detail lacks its row binding")
        trial_id = str(row.get("trial_id", ""))
        exposed_trials.add(trial_id)
        if row.get("candidate_id") == "V13_BASELINE" and row.get("cadence") == "fine_1ms":
            selected.append(dict(record))
    unexpected = sorted(exposed_trials - set(AUTHORIZED_DEVELOPMENT_TRIALS))
    if unexpected:
        raise V17DevelopmentGateError(
            f"details expose non-development trial ids: {unexpected}"
        )
    expected_units = {
        (trial_id, plateau)
        for trial_id in AUTHORIZED_DEVELOPMENT_TRIALS
        for plateau in range(1, 5)
    }
    observed_units = {
        (str(item["row"].get("trial_id")), int(item["row"].get("plateau_index")))
        for item in selected
    }
    if len(selected) != 12 or observed_units != expected_units:
        raise V17DevelopmentGateError(
            "expected exactly 12 V13_BASELINE/fine_1ms detail records"
        )
    return sorted(
        selected,
        key=lambda item: (
            str(item["row"]["trial_id"]),
            int(item["row"]["plateau_index"]),
        ),
    )


def _assert_profile_equivalence(
    v17_profile: Mapping[str, Any],
    v13_profile: Mapping[str, Any],
) -> dict[str, Any]:
    for field in ("ground", "material", "spheres"):
        if v17_profile.get(field) != v13_profile.get(field):
            raise V17DevelopmentGateError(f"V17 {field} differs from V13")
    spheres = v17_profile.get("spheres")
    if not isinstance(spheres, list) or len(spheres) != 2:
        raise V17DevelopmentGateError("V17 must contain exactly two spheres")
    names = [
        str(sphere.get("name"))
        for sphere in spheres
        if isinstance(sphere, Mapping)
    ]
    if names != ["left_heel", "left_toe"]:
        raise V17DevelopmentGateError("V17 roles must be exactly left_heel,left_toe")
    return {
        "ground_exact_v13": True,
        "material_exact_v13": True,
        "geometry_exact_v13": True,
        "sensor_count": 2,
        "roles": names,
    }


def _verified_contract_and_profiles(
    contract_path: Path,
    manifest: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    contract = _strict_object(contract_path, label="V17 contract")
    if contract.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise V17DevelopmentGateError("V17 event contract id drifted")
    if contract.get("data_split", {}).get("development") != list(
        AUTHORIZED_DEVELOPMENT_TRIALS
    ):
        raise V17DevelopmentGateError("V17 development split drifted")
    if set(contract.get("development_sources", {})) != set(
        AUTHORIZED_DEVELOPMENT_TRIALS
    ):
        raise V17DevelopmentGateError("V17 contract exposes a non-development source")
    decision = contract.get("decision_contract")
    if not isinstance(decision, Mapping) or decision.get("single_candidate_only") is not True:
        raise V17DevelopmentGateError("V17 single-candidate decision lock drifted")
    for field in (
        "geometry_threshold_dwell_or_radius_sweep_allowed",
        "fallback_or_retuning_after_failure_allowed",
        "protected_open_allowed_by_this_contract",
        "runtime_or_training_promotion_allowed",
        "positive_morphology_weight_training_allowed",
    ):
        if decision.get(field) is not False:
            raise V17DevelopmentGateError(f"V17 unsafe decision control: {field}")
    candidate = contract.get("candidate")
    if not isinstance(candidate, Mapping):
        raise V17DevelopmentGateError("V17 candidate contract is missing")
    expected_candidate = {
        "sensor_count": 2,
        "roles": ["left_heel", "left_toe"],
        "applies_force": False,
        "geometry_locked_to_v13": True,
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "detector_sample_dt_s": 0.001,
        "policy_step_s": 0.01,
    }
    for name, expected in expected_candidate.items():
        if candidate.get(name) != expected:
            raise V17DevelopmentGateError(
                f"V17 candidate.{name} drifted: {candidate.get(name)!r}"
            )

    source_identity = manifest.get("source_identity")
    if not isinstance(source_identity, Mapping):
        raise V17DevelopmentGateError("V14.2 source_identity is missing")
    v13_record = source_identity.get("baseline_profile")
    if not isinstance(v13_record, Mapping):
        raise V17DevelopmentGateError("V14.2 baseline profile record is missing")
    if v13_record.get("sha256") != EXPECTED_V13_PROFILE_SHA256:
        raise V17DevelopmentGateError("V14.2 V13 profile hash record drifted")
    v13_path = _portable_repo_path(v13_record.get("path"), label="V13 profile path")
    v13_source = _require_sha256(
        v13_path,
        EXPECTED_V13_PROFILE_SHA256,
        label="V13 profile",
    )
    v17_path = _portable_repo_path(
        candidate.get("profile_path"), label="V17 profile path"
    )
    v13_profile = _strict_object(v13_path, label="V13 profile")
    v17_profile = _strict_object(v17_path, label="V17 profile")
    identity = _assert_profile_equivalence(v17_profile, v13_profile)
    metadata = v17_profile.get("metadata")
    if not isinstance(metadata, Mapping):
        raise V17DevelopmentGateError("V17 profile metadata is missing")
    sampling = metadata.get("detector_sampling")
    if not isinstance(sampling, Mapping):
        raise V17DevelopmentGateError("V17 sampling metadata is missing")
    expected_sampling = {
        "sample_dt_s": 0.001,
        "policy_step_s": 0.01,
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
    }
    if dict(sampling) != expected_sampling:
        raise V17DevelopmentGateError("V17 profile sampling contract drifted")
    if metadata.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise V17DevelopmentGateError("V17 profile event contract id drifted")
    return contract, {
        **identity,
        "v13_profile": v13_source,
        "v17_profile": _source_record(v17_path),
        "applies_force": False,
        "threshold_and_dwell_exact_v14_2": True,
    }


def _verified_v14_protocol(manifest: Mapping[str, Any]) -> dict[str, Any]:
    record = manifest.get("protocol")
    if not isinstance(record, Mapping) or not gate.is_sha256(record.get("sha256")):
        raise V17DevelopmentGateError("V14.2 protocol record is invalid")
    path = _portable_repo_path(record.get("path"), label="V14.2 protocol path")
    source = _require_sha256(path, str(record["sha256"]), label="V14.2 protocol")
    protocol = _strict_object(path, label="V14.2 protocol")
    replay = protocol.get("replay")
    if not isinstance(replay, Mapping):
        raise V17DevelopmentGateError("V14.2 replay contract is missing")
    expected = {
        "fine_sample_dt_s": 0.001,
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "prescribed_contact_threshold_n": 20.0,
        "reference_min_contact_duration_s": 0.05,
        "reference_min_cycle_duration_s": 0.3,
    }
    failed = {
        name: {"expected": value, "observed": replay.get(name)}
        for name, value in expected.items()
        if replay.get(name) != value
    }
    if failed:
        raise V17DevelopmentGateError(f"V14.2 replay contract drifted: {failed}")
    return source


def _assignment_value(tree: ast.Module, name: str) -> Any:
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if not any(isinstance(target, ast.Name) and target.id == name for target in targets):
            continue
        value = node.value
        if isinstance(value, ast.Call) and isinstance(value.func, ast.Name):
            if value.func.id == "frozenset" and len(value.args) == 1:
                return frozenset(ast.literal_eval(value.args[0]))
        return ast.literal_eval(value)
    raise V17DevelopmentGateError(f"oracle builder constant {name} is missing")


def _verified_oracle_builder(path: Path) -> dict[str, Any]:
    source = _source_record(path)
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, UnicodeError, SyntaxError) as exc:
        raise V17DevelopmentGateError(f"cannot inspect canonical oracle builder: {exc}") from exc
    if _assignment_value(tree, "EVENT_CONTRACT_ID") != EVENT_CONTRACT_ID:
        raise V17DevelopmentGateError("oracle builder event contract id drifted")
    if tuple(_assignment_value(tree, "AUTHORIZED_DEVELOPMENT_TRIALS")) != (
        AUTHORIZED_DEVELOPMENT_TRIALS
    ):
        raise V17DevelopmentGateError("oracle builder development allowlist drifted")
    forbidden = frozenset(_assignment_value(tree, "FORBIDDEN_TRIALS"))
    if forbidden != FORBIDDEN_TRIALS:
        raise V17DevelopmentGateError("oracle builder forbidden-trial set drifted")
    return {
        **source,
        "event_contract_id": EVENT_CONTRACT_ID,
        "authorized_development_trials": list(AUTHORIZED_DEVELOPMENT_TRIALS),
        "forbidden_trials": sorted(FORBIDDEN_TRIALS),
    }


def _canonical_sha256(value: Any) -> str:
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V17DevelopmentGateError(f"non-finite canonical value: {exc}") from exc
    return hashlib.sha256(encoded).hexdigest()


def _verified_oracle_bundle(
    manifest_path: Path,
    *,
    builder_record: Mapping[str, Any],
    contract_record: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    manifest_record = _require_sha256(
        manifest_path,
        EXPECTED_ORACLE_MANIFEST_SHA256,
        label="canonical oracle manifest",
    )
    manifest = _strict_object(manifest_path, label="canonical oracle manifest")
    if manifest.get("status") != "CANONICAL_ORACLE_FROZEN_DEVELOPMENT_ONLY":
        raise V17DevelopmentGateError("canonical oracle bundle is not frozen")
    if manifest.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise V17DevelopmentGateError("canonical oracle manifest contract drifted")
    data_access = manifest.get("data_access")
    if not isinstance(data_access, Mapping) or data_access.get("opened_trials") != list(
        AUTHORIZED_DEVELOPMENT_TRIALS
    ):
        raise V17DevelopmentGateError("canonical oracle development access drifted")
    if (
        data_access.get("protected_trials_opened") != []
        or data_access.get("reserve_trials_opened") != []
        or data_access.get("trial_01_used") is not False
    ):
        raise V17DevelopmentGateError("canonical oracle bundle accessed forbidden data")
    for name, expected in (
        ("builder", builder_record),
        ("contract", contract_record),
    ):
        observed = manifest.get(name)
        if not isinstance(observed, Mapping):
            raise V17DevelopmentGateError(f"oracle manifest {name} binding is missing")
        if observed.get("path") != expected.get("path") or observed.get(
            "sha256"
        ) != expected.get("sha256"):
            raise V17DevelopmentGateError(f"oracle manifest {name} binding drifted")

    raw_records = manifest.get("ledgers")
    if not isinstance(raw_records, list) or len(raw_records) != 3:
        raise V17DevelopmentGateError("oracle manifest must bind exactly three ledgers")
    by_trial: dict[str, dict[str, Any]] = {}
    source_records: dict[str, dict[str, Any]] = {}
    for raw_record in raw_records:
        if not isinstance(raw_record, Mapping):
            raise V17DevelopmentGateError("oracle ledger record is invalid")
        trial_id = str(raw_record.get("trial_id", ""))
        if trial_id not in AUTHORIZED_DEVELOPMENT_TRIALS or trial_id in by_trial:
            raise V17DevelopmentGateError("oracle ledger trial inventory drifted")
        if raw_record.get("role") != "DEVELOPMENT":
            raise V17DevelopmentGateError("oracle ledger is not development-only")
        path = _portable_repo_path(
            raw_record.get("path"), label=f"oracle ledger {trial_id} path"
        )
        source = _require_sha256(
            path,
            str(raw_record.get("sha256")),
            label=f"oracle ledger {trial_id}",
        )
        ledger = _strict_object(path, label=f"oracle ledger {trial_id}")
        core = ledger.get("scientific_core")
        if not isinstance(core, Mapping) or str(core.get("trial_id")) != trial_id:
            raise V17DevelopmentGateError(f"oracle ledger {trial_id} core drifted")
        if core.get("event_contract_id") != EVENT_CONTRACT_ID:
            raise V17DevelopmentGateError(f"oracle ledger {trial_id} contract drifted")
        core_sha = _canonical_sha256(core)
        if core_sha != ledger.get("scientific_core_sha256") or core_sha != raw_record.get(
            "scientific_core_sha256"
        ):
            raise V17DevelopmentGateError(
                f"oracle ledger {trial_id} scientific core hash drifted"
            )
        views = core.get("views")
        if not isinstance(views, list) or {
            str(view.get("view_id")) for view in views if isinstance(view, Mapping)
        } != {f"plateau_{index:02d}" for index in range(1, 5)}:
            raise V17DevelopmentGateError(
                f"oracle ledger {trial_id} plateau views drifted"
            )
        by_trial[trial_id] = ledger
        source_records[f"trial_{trial_id}"] = {
            **source,
            "scientific_core_sha256": core_sha,
        }
    if set(by_trial) != set(AUTHORIZED_DEVELOPMENT_TRIALS):
        raise V17DevelopmentGateError("oracle bundle development trials are incomplete")
    return manifest_record, by_trial, source_records


def _bind_detail_rows_to_metrics(
    details: Sequence[Mapping[str, Any]],
    rows: Sequence[Mapping[str, str]],
) -> None:
    by_unit = {
        (str(row["trial_id"]), int(row["plateau_index"])): row for row in rows
    }
    fields = (
        "precision",
        "recall",
        "confirmed_fsm_stance_f1",
        "confirmed_fsm_stance_iou",
        "max_abs_hs_error_s",
        "max_abs_toe_off_error_s",
    )
    for detail in details:
        detail_row = detail["row"]
        key = (str(detail_row["trial_id"]), int(detail_row["plateau_index"]))
        if key not in by_unit:
            raise V17DevelopmentGateError(f"detail unit {key} lacks a metrics row")
        metric_row = by_unit[key]
        for field in fields:
            left = _finite(detail_row.get(field), label=f"detail {key} {field}")
            right = _finite(metric_row.get(field), label=f"metrics {key} {field}")
            if left != right:
                raise V17DevelopmentGateError(
                    f"details/metrics mismatch for {key} field {field}"
                )


def _canonical_view(
    ledger: Mapping[str, Any], plateau_index: int
) -> Mapping[str, Any]:
    views = ledger["scientific_core"]["views"]
    expected = f"plateau_{plateau_index:02d}"
    matches = [view for view in views if view.get("view_id") == expected]
    if len(matches) != 1:
        raise V17DevelopmentGateError(f"canonical view {expected} is ambiguous")
    return matches[0]


def _direct_canonical_rescore(
    details: Sequence[Mapping[str, Any]],
    ledgers: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    units: list[dict[str, Any]] = []
    worst_hs: dict[str, Any] | None = None
    maximum_mapping_shift = 0.0
    for detail in details:
        row = detail["row"]
        trial_id = str(row["trial_id"])
        plateau_index = int(row["plateau_index"])
        unit_id = f"{trial_id}:{plateau_index}"
        view = _canonical_view(ledgers[trial_id], plateau_index)
        scoreable = view.get("scoreable_events")
        if not isinstance(scoreable, list):
            raise V17DevelopmentGateError(f"canonical unit {unit_id} has no events")
        unit_result: dict[str, Any] = {
            "unit": unit_id,
            "trial_id": trial_id,
            "plateau_index": plateau_index,
            "events": {},
        }
        sequence_material: dict[str, Any] = {}
        for event, tolerance_s in (("heel_strike", 0.05), ("toe_off", 0.08)):
            reference = detail.get("reference", {}).get("events", {}).get(event)
            ordered_errors = detail.get("confirmed_timing", {}).get(event, {}).get(
                "ordered_errors_s"
            )
            if not isinstance(reference, list) or not isinstance(ordered_errors, list):
                raise V17DevelopmentGateError(
                    f"published confirmed timing is missing for {unit_id} {event}"
                )
            if len(reference) != len(ordered_errors) or not reference:
                raise V17DevelopmentGateError(
                    f"published confirmed timing count drifted for {unit_id} {event}"
                )
            old_reference = [
                _finite(value, label=f"{unit_id} {event} old reference")
                for value in reference
            ]
            errors = [
                _finite(value, label=f"{unit_id} {event} ordered error")
                for value in ordered_errors
            ]
            canonical_all = [
                _finite(item.get("event_time_s"), label="canonical event time")
                for item in scoreable
                if isinstance(item, Mapping) and item.get("event") == event
            ]
            mapped: list[float] = []
            used: set[int] = set()
            for old_time in old_reference:
                candidates = [
                    (index, canonical_time)
                    for index, canonical_time in enumerate(canonical_all)
                    if index not in used
                    and abs(canonical_time - old_time)
                    <= CANONICAL_REFERENCE_SHIFT_BOUND_S + 1.0e-10
                ]
                if len(candidates) != 1:
                    raise V17DevelopmentGateError(
                        f"canonical mapping is not unique for {unit_id} {event} at {old_time}"
                    )
                index, canonical_time = candidates[0]
                shift = canonical_time - old_time
                if shift < -1.0e-10 or shift > CANONICAL_REFERENCE_SHIFT_BOUND_S + 1.0e-10:
                    raise V17DevelopmentGateError(
                        f"canonical mapping direction drifted for {unit_id} {event}"
                    )
                maximum_mapping_shift = max(maximum_mapping_shift, abs(shift))
                used.add(index)
                mapped.append(canonical_time)
            confirmed = [
                old_time + error for old_time, error in zip(old_reference, errors)
            ]
            if any(right <= left for left, right in zip(confirmed, confirmed[1:])):
                raise V17DevelopmentGateError(
                    f"reconstructed confirmed {event} order drifted for {unit_id}"
                )
            canonical_errors = [
                predicted - canonical
                for predicted, canonical in zip(confirmed, mapped)
            ]
            matched = sum(
                abs(error) <= tolerance_s + 1.0e-12 for error in canonical_errors
            )
            maximum_error = max(abs(error) for error in canonical_errors)
            event_result = {
                "published_confirmed_count": len(confirmed),
                "mapped_canonical_reference_count": len(mapped),
                "full_canonical_view_count": len(canonical_all),
                "full_view_coverage_complete": len(mapped) == len(canonical_all),
                "matched_within_tolerance_count": matched,
                "precision": matched / len(confirmed),
                "recall": matched / len(mapped),
                "maximum_absolute_error_s": maximum_error,
                "tolerance_s": tolerance_s,
            }
            unit_result["events"][event] = event_result
            sequence_material[event] = {
                "canonical_reference_s": mapped,
                "published_confirmed_s": confirmed,
                "errors_s": canonical_errors,
            }
            if event == "heel_strike":
                local_index = max(
                    range(len(canonical_errors)),
                    key=lambda index: abs(canonical_errors[index]),
                )
                candidate = {
                    "unit": unit_id,
                    "event_index": local_index,
                    "canonical_event_time_s": mapped[local_index],
                    "published_confirmed_time_s": confirmed[local_index],
                    "absolute_error_s": abs(canonical_errors[local_index]),
                    "maximum_allowed_s": tolerance_s,
                }
                if worst_hs is None or candidate["absolute_error_s"] > worst_hs[
                    "absolute_error_s"
                ]:
                    worst_hs = candidate
        unit_result["reconstructed_sequence_sha256"] = _canonical_sha256(
            sequence_material
        )
        units.append(unit_result)

    if worst_hs is None:
        raise V17DevelopmentGateError("canonical direct rescore produced no HS")
    checks = [
        {
            "id": "canonical_precision_exact_one",
            "status": (
                "PASS"
                if all(
                    event["precision"] == 1.0
                    for unit in units
                    for event in unit["events"].values()
                )
                else "FAIL"
            ),
            "observed_minimum": min(
                event["precision"]
                for unit in units
                for event in unit["events"].values()
            ),
            "threshold": 1.0,
        },
        {
            "id": "canonical_recall_exact_one",
            "status": (
                "PASS"
                if all(
                    event["recall"] == 1.0
                    for unit in units
                    for event in unit["events"].values()
                )
                else "FAIL"
            ),
            "observed_minimum": min(
                event["recall"]
                for unit in units
                for event in unit["events"].values()
            ),
            "threshold": 1.0,
        },
        {
            "id": "canonical_confirmed_hs_error_at_most_0p05s",
            "status": (
                "PASS"
                if all(
                    unit["events"]["heel_strike"]["maximum_absolute_error_s"]
                    <= 0.05 + 1.0e-12
                    for unit in units
                )
                else "FAIL"
            ),
            "observed_maximum_s": max(
                unit["events"]["heel_strike"]["maximum_absolute_error_s"]
                for unit in units
            ),
            "threshold_s": 0.05,
        },
        {
            "id": "canonical_confirmed_to_error_at_most_0p08s",
            "status": (
                "PASS"
                if all(
                    unit["events"]["toe_off"]["maximum_absolute_error_s"]
                    <= 0.08 + 1.0e-12
                    for unit in units
                )
                else "FAIL"
            ),
            "observed_maximum_s": max(
                unit["events"]["toe_off"]["maximum_absolute_error_s"]
                for unit in units
            ),
            "threshold_s": 0.08,
        },
        {
            "id": "published_subset_exact_count_and_order",
            "status": "PASS",
            "unit_count": len(units),
        },
    ]
    incomplete_coverage = [
        {
            "unit": unit["unit"],
            "event": event_name,
            "published_count": event["published_confirmed_count"],
            "canonical_view_count": event["full_canonical_view_count"],
        }
        for unit in units
        for event_name, event in unit["events"].items()
        if not event["full_view_coverage_complete"]
    ]
    return {
        "execution": "EXECUTED_HASH_PINNED_DETAILS_AGAINST_CANONICAL_LEDGER_ONSETS",
        "checks": checks,
        "failed_checks": [check["id"] for check in checks if check["status"] != "PASS"],
        "units": units,
        "maximum_legacy_to_canonical_reference_shift_s": maximum_mapping_shift,
        "full_canonical_view_coverage": {
            "status": "INCOMPLETE_PUBLISHED_V14_2_BOUNDARY_SUBSET"
            if incomplete_coverage
            else "COMPLETE",
            "incomplete_units": incomplete_coverage,
            "interpretation": (
                "Direct timing is scored on the exact historical published subset; "
                "no confirmed event is invented for extra canonical boundary events."
            ),
        },
        "terminal_failure_proof": {
            "id": "direct_canonical_confirmed_hs_failure",
            "status": "FAIL"
            if worst_hs["absolute_error_s"] > worst_hs["maximum_allowed_s"]
            else "PASS",
            **worst_hs,
        },
    }


def _metric_check(
    identifier: str,
    rows: Sequence[Mapping[str, str]],
    field: str,
    predicate: Any,
    *,
    reducer: Any,
    threshold: Any,
) -> dict[str, Any]:
    values = [_finite(row[field], label=f"{identifier}[{index}]") for index, row in enumerate(rows)]
    failed_units = [
        f"{row['trial_id']}:{row['plateau_index']}"
        for row, value in zip(rows, values)
        if not bool(predicate(value))
    ]
    return {
        "id": identifier,
        "execution": "EXECUTED_PINNED_V14_2_FINE_1MS",
        "status": "PASS" if not failed_units else "FAIL",
        "field": field,
        "threshold": threshold,
        "observed": reducer(values),
        "failed_units": failed_units,
    }


def _zero_sum_check(
    identifier: str,
    rows: Sequence[Mapping[str, str]],
    field: str,
) -> dict[str, Any]:
    values = [_finite(row[field], label=f"{identifier}[{index}]") for index, row in enumerate(rows)]
    failed_units = [
        f"{row['trial_id']}:{row['plateau_index']}"
        for row, value in zip(rows, values)
        if value != 0.0
    ]
    return {
        "id": identifier,
        "execution": "EXECUTED_PINNED_V14_2_FINE_1MS",
        "status": "PASS" if not failed_units else "FAIL",
        "field": field,
        "threshold": 0,
        "observed_sum": sum(values),
        "failed_units": failed_units,
    }


def evaluate_sequential_rows(
    rows: Sequence[Mapping[str, str]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if len(rows) != 12:
        raise V17DevelopmentGateError("sequential evaluation requires exactly 12 rows")
    for row_index, row in enumerate(rows):
        for field in REQUIRED_NUMERIC_FIELDS:
            if field not in row:
                raise V17DevelopmentGateError(f"row {row_index} lacks {field}")
            _finite(row[field], label=f"row[{row_index}].{field}")
        if _finite(row["sample_dt_s"], label="sample_dt_s") != 0.001:
            raise V17DevelopmentGateError("selected evidence is not exactly 1 ms")
        for field in (
            "mesh_geometry_pre_gate_ok",
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count",
        ):
            _boolean(row.get(field), label=f"row[{row_index}].{field}")

    checks = [
        _metric_check(
            "precision_exact_one",
            rows,
            "precision",
            lambda value: value == 1.0,
            reducer=min,
            threshold=1.0,
        ),
        _metric_check(
            "recall_exact_one",
            rows,
            "recall",
            lambda value: value == 1.0,
            reducer=min,
            threshold=1.0,
        ),
        _metric_check(
            "f1_at_least_0p95",
            rows,
            "confirmed_fsm_stance_f1",
            lambda value: value >= 0.95,
            reducer=min,
            threshold=0.95,
        ),
        _metric_check(
            "iou_at_least_0p90",
            rows,
            "confirmed_fsm_stance_iou",
            lambda value: value >= 0.90,
            reducer=min,
            threshold=0.90,
        ),
        _metric_check(
            "confirmed_hs_error_at_most_0p05s",
            rows,
            "max_abs_hs_error_s",
            lambda value: value <= 0.05,
            reducer=max,
            threshold=0.05,
        ),
        _metric_check(
            "confirmed_to_error_at_most_0p08s",
            rows,
            "max_abs_toe_off_error_s",
            lambda value: value <= 0.08,
            reducer=max,
            threshold=0.08,
        ),
        _zero_sum_check(
            "zero_forbidden_phase_mismatches",
            rows,
            "forbidden_phase_mismatch_count",
        ),
        _zero_sum_check(
            "zero_invalid_or_timeout_transitions",
            rows,
            "invalid_or_timeout_transition_count",
        ),
        _zero_sum_check(
            "zero_unaccepted_sensor_events",
            rows,
            "unaccepted_sensor_gait_event_count",
        ),
        _zero_sum_check(
            "zero_incomplete_heel_to_forefoot_transfers",
            rows,
            "incomplete_heel_to_forefoot_transfer_count",
        ),
        _zero_sum_check(
            "zero_both_latches_off_samples",
            rows,
            "transfer_both_latches_off_sample_count",
        ),
        _zero_sum_check(
            "zero_early_to_candidates",
            rows,
            "to_candidates_before_min_stance_count",
        ),
        _zero_sum_check(
            "zero_unknown_phase_samples",
            rows,
            "unknown_fsm_phase_samples",
        ),
        _metric_check(
            "minimum_toe_clear_at_least_0p03s",
            rows,
            "minimum_causal_toe_clear_before_next_hs_onset_s",
            lambda value: value >= 0.03,
            reducer=min,
            threshold=0.03,
        ),
    ]

    mesh_failures = [
        f"{row['trial_id']}:{row['plateau_index']}"
        for row in rows
        if not _boolean(row["mesh_geometry_pre_gate_ok"], label="mesh pre-gate")
    ]
    checks.append(
        {
            "id": "mesh_geometry_pre_gate",
            "execution": "EXECUTED_PINNED_V14_2_FINE_1MS",
            "status": "PASS" if not mesh_failures else "FAIL",
            "failed_units": mesh_failures,
        }
    )

    exact_failures: list[str] = []
    for row in rows:
        counts_exact = all(
            (
                _finite(row[observed], label=observed)
                == _finite(row[expected], label=expected)
            )
            for observed, expected in (
                ("predicted_hs_count", "expected_reference_hs_count"),
                ("predicted_to_count", "expected_reference_to_count"),
                ("observed_valid_cycle_count", "expected_valid_cycle_count"),
            )
        )
        order_exact = _boolean(
            row["exact_hs_to_toe_off_to_hs_order_and_cycle_count"],
            label="exact event order",
        )
        if not (counts_exact and order_exact):
            exact_failures.append(f"{row['trial_id']}:{row['plateau_index']}")
    checks.append(
        {
            "id": "exact_event_counts_order_and_cycles",
            "execution": "EXECUTED_PINNED_V14_2_FINE_1MS",
            "status": "PASS" if not exact_failures else "FAIL",
            "failed_units": exact_failures,
        }
    )
    checks.append(
        {
            "id": "required_metrics_finite",
            "execution": "EXECUTED_PINNED_V14_2_FINE_1MS",
            "status": "PASS",
            "required_fields": list(REQUIRED_NUMERIC_FIELDS),
        }
    )

    worst_hs = max(
        (
            {
                "unit": f"{row['trial_id']}:{row['plateau_index']}",
                "historical_error_s": _finite(
                    row["max_abs_hs_error_s"], label="max_abs_hs_error_s"
                ),
            }
            for row in rows
        ),
        key=lambda item: item["historical_error_s"],
    )
    canonical_lower_bound = max(
        0.0,
        worst_hs["historical_error_s"] - CANONICAL_REFERENCE_SHIFT_BOUND_S,
    )
    robust_terminal_failure = canonical_lower_bound > 0.05
    proof = {
        "id": "canonical_1ms_hs_failure_lower_bound",
        "status": "FAIL" if robust_terminal_failure else "BLOCKED",
        "unit": worst_hs["unit"],
        "historical_interpolated_reference_error_s": worst_hs[
            "historical_error_s"
        ],
        "canonical_reference_shift_bound_s": CANONICAL_REFERENCE_SHIFT_BOUND_S,
        "canonical_error_lower_bound_s": canonical_lower_bound,
        "maximum_allowed_s": 0.05,
        "reason": (
            "V13 and V17 share the same detector samples/contract; changing only "
            "the 20 N reference timestamp from an in-sample interpolation to the "
            "first 1 ms sample can move that reference by at most one sample"
        ),
    }
    return checks, proof


def build_receipt(
    *,
    manifest_path: Path = DEFAULT_V14_MANIFEST,
    metrics_path: Path = DEFAULT_V14_METRICS,
    details_path: Path = DEFAULT_V14_DETAILS,
    contract_path: Path = DEFAULT_V17_CONTRACT,
    oracle_builder_path: Path = DEFAULT_ORACLE_BUILDER,
    oracle_manifest_path: Path = DEFAULT_ORACLE_MANIFEST,
) -> dict[str, Any]:
    manifest_record = _require_sha256(
        manifest_path,
        EXPECTED_V14_MANIFEST_SHA256,
        label="V14.2 manifest",
    )
    metrics_record = _require_sha256(
        metrics_path,
        EXPECTED_V14_METRICS_SHA256,
        label="V14.2 development metrics",
    )
    details_record = _require_sha256(
        details_path,
        EXPECTED_V14_DETAILS_SHA256,
        label="V14.2 development details",
    )
    manifest = _strict_object(manifest_path, label="V14.2 manifest")
    artifact = manifest.get("artifacts", {}).get("development_isolated_unit_metrics")
    if not isinstance(artifact, Mapping):
        raise V17DevelopmentGateError("manifest does not bind development metrics")
    if artifact.get("path") != metrics_record["path"] or artifact.get("sha256") != (
        metrics_record["sha256"]
    ):
        raise V17DevelopmentGateError("manifest/metrics artifact binding drifted")
    details_artifact = manifest.get("artifacts", {}).get("development_stage1_details")
    if not isinstance(details_artifact, Mapping):
        raise V17DevelopmentGateError("manifest does not bind development details")
    if details_artifact.get("path") != details_record["path"] or details_artifact.get(
        "sha256"
    ) != details_record["sha256"]:
        raise V17DevelopmentGateError("manifest/details artifact binding drifted")
    if manifest.get("validation", {}).get("opened") is not False:
        raise V17DevelopmentGateError("V14.2 validation state is not closed")
    if manifest.get("sealed", {}).get("opened") is not False:
        raise V17DevelopmentGateError("V14.2 sealed state is not closed")
    access = manifest.get("development", {}).get("access")
    if not isinstance(access, list) or {
        str(item.get("trial_id")) for item in access if isinstance(item, Mapping)
    } != set(AUTHORIZED_DEVELOPMENT_TRIALS):
        raise V17DevelopmentGateError("V14.2 development access inventory drifted")

    contract, profile_identity = _verified_contract_and_profiles(
        contract_path, manifest
    )
    protocol_record = _verified_v14_protocol(manifest)
    oracle_builder_record = _verified_oracle_builder(oracle_builder_path)
    contract_record = _source_record(contract_path)
    (
        oracle_manifest_record,
        oracle_ledgers,
        oracle_ledger_records,
    ) = _verified_oracle_bundle(
        oracle_manifest_path,
        builder_record=oracle_builder_record,
        contract_record=contract_record,
    )
    selected = _selected_rows(_read_csv(metrics_path))
    selected_details = _selected_details(_read_jsonl(details_path))
    _bind_detail_rows_to_metrics(selected_details, selected)
    checks, historical_bound_proof = evaluate_sequential_rows(selected)
    canonical_rescore = _direct_canonical_rescore(selected_details, oracle_ledgers)

    historical_failed = [check["id"] for check in checks if check["status"] != "PASS"]
    terminal_fail = canonical_rescore["terminal_failure_proof"]["status"] == "FAIL"
    status = "FAIL" if terminal_fail else "BLOCKED"
    receipt = {
        "schema_version": SCHEMA_VERSION,
        "gate_id": "TWO_SENSOR_HIGH_RATE_DEVELOPMENT_READY",
        "candidate_id": "two_sensor_v17_high_rate_v13_geometry",
        "event_contract_id": EVENT_CONTRACT_ID,
        "status": status,
        "decision": (
            "TERMINAL_SEQUENTIAL_FAIL_NO_RETUNING"
            if terminal_fail
            else "BLOCKED_CANONICAL_RESCORE_AND_BATCH_TRACE_REQUIRED"
        ),
        "source_records": {
            "v14_2_manifest": manifest_record,
            "v14_2_development_isolated_metrics": metrics_record,
            "v14_2_development_stage1_details": details_record,
            "v14_2_protocol": protocol_record,
            "v17_contract": contract_record,
            "canonical_oracle_builder": oracle_builder_record,
            "canonical_oracle_manifest": oracle_manifest_record,
            "canonical_oracle_ledgers": oracle_ledger_records,
            "v13_profile": profile_identity["v13_profile"],
            "v17_profile": profile_identity["v17_profile"],
        },
        "profile_and_contract_identity": {
            key: value
            for key, value in profile_identity.items()
            if key not in {"v13_profile", "v17_profile"}
        },
        "data_access": {
            "opened_trial_derived_artifacts": [
                manifest_record["path"],
                metrics_record["path"],
                details_record["path"],
                *[
                    oracle_ledger_records[f"trial_{trial_id}"]["path"]
                    for trial_id in AUTHORIZED_DEVELOPMENT_TRIALS
                ],
            ],
            "selected_trials": list(AUTHORIZED_DEVELOPMENT_TRIALS),
            "selected_candidate": "V13_BASELINE",
            "selected_cadence": "fine_1ms",
            "selected_row_count": len(selected),
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "forbidden_trials": sorted(FORBIDDEN_TRIALS),
        },
        "oracle_lineage": {
            "canonical_contract_bound": True,
            "canonical_ledger_consumed": True,
            "canonical_event_contract_id": EVENT_CONTRACT_ID,
            "historical_v14_2_reference": "linearly_interpolated_20N_crossing",
            "canonical_reference": "first_1ms_sample_on_new_side_of_20N",
            "maximum_reference_timestamp_change_s": (
                CANONICAL_REFERENCE_SHIFT_BOUND_S
            ),
            "canonical_ledger_rescore": {
                "execution": canonical_rescore["execution"],
                "status": "FAIL"
                if canonical_rescore["failed_checks"]
                else "PASS",
            },
        },
        "sequential_1ms": {
            "execution": "EXECUTED_FROM_12_HASH_PINNED_V13_BASELINE_ROWS",
            "evidence_semantics": "V14_2_FINE_1MS_WITH_INTERPOLATED_20N_REFERENCE",
            "checks": checks,
            "failed_checks": historical_failed,
            "historical_reference_shift_bound_proof": historical_bound_proof,
            "canonical_direct_rescore": canonical_rescore,
            "trial_aggregates": [
                {
                    "trial_id": trial_id,
                    "status": (
                        "FAIL"
                        if any(
                            unit in check.get("failed_units", [])
                            for check in checks
                            for unit in [
                                f"{trial_id}:{plateau}" for plateau in range(1, 5)
                            ]
                        )
                        else "PASS"
                    ),
                }
                for trial_id in AUTHORIZED_DEVELOPMENT_TRIALS
            ],
        },
        "not_executed": [
            {
                "id": "batched_10ms_same_samples_parity",
                "status": "NOT_EXECUTED",
                "reason": "raw 1 ms detector trace is absent from the pinned V14.2 products",
            },
            {
                "id": "policy_visible_hs_to_latency",
                "status": "NOT_EXECUTED",
                "reason": "no real 10 ms batch-delivery trace exists",
            },
            {
                "id": "delivery_after_confirmation_at_most_0p01s",
                "status": "NOT_EXECUTED",
                "reason": "no real 10 ms batch-delivery trace exists",
            },
            {
                "id": "six_sequential_and_batched_trial_aggregates",
                "status": "NOT_EXECUTED",
                "reason": "three sequential aggregates fail and batch evidence was not fabricated",
            },
        ],
        "stage_controls": {
            "next_gate_allowed": False,
            "h0_evaluation_allowed": False,
            "protected_validation_open_allowed": False,
            "sealed_open_allowed": False,
            "runtime_or_training_promotion_allowed": False,
            "fallback_retuning_or_reselection_allowed": False,
            "positive_morphology_training_allowed": False,
        },
        "interpretation": (
            "The fixed V17 candidate does not reach development readiness.  "
            "Published sequential confirmed events directly rescored against the "
            "hash-pinned canonical 1 ms ledger violate precision, recall and HS "
            "timing.  No statement of batch parity is made."
        ),
        "contract_decision_controls_verified": bool(
            contract.get("decision_contract", {}).get("single_candidate_only")
            and not contract.get("decision_contract", {}).get(
                "fallback_or_retuning_after_failure_allowed", True
            )
        ),
    }
    # Enforce strict, finite JSON before publication.
    json.dumps(receipt, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return receipt


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="immutable receipt path under validation/",
    )
    parser.add_argument(
        "--oracle-builder",
        type=Path,
        default=DEFAULT_ORACLE_BUILDER,
        help="canonical-oracle source to hash and contract-check",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output = args.output.resolve()
    try:
        output.relative_to(REPO_ROOT / "validation")
    except ValueError as exc:
        raise V17DevelopmentGateError("receipt output must stay under validation/") from exc
    receipt = build_receipt(oracle_builder_path=args.oracle_builder.resolve())
    gate.write_json_no_clobber(output, receipt)
    print(
        json.dumps(
            {
                "status": receipt["status"],
                "decision": receipt["decision"],
                "output": output.relative_to(REPO_ROOT).as_posix(),
            },
            sort_keys=True,
            allow_nan=False,
        )
    )
    return 0 if receipt["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
