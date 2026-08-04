"""Synthetic contract tests for primary-GRF baseline forensics."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import runpy
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from validation import audit_primary_grf_training_readiness as audit
from validation import readiness_gatekeeper as gate


def test_direct_script_bootstrap_imports_raw_adapter_dependencies(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if importlib.util.find_spec("opensim") is None:
        pytest.skip("OpenSim is unavailable in this Python environment")
    script = Path(audit.__file__).resolve()
    repo_root = script.parents[1]
    validation_dir = script.parent
    stripped = [
        entry
        for entry in sys.path
        if Path(entry or ".").resolve() != repo_root
    ]
    monkeypatch.setattr(sys, "path", [str(validation_dir), *stripped])

    namespace = runpy.run_path(
        str(script),
        run_name="_primary_grf_direct_cli_import_test",
    )
    assert str(repo_root) in sys.path
    dependencies = namespace["_raw_adapter_dependencies"]()
    assert set(dependencies) == {
        "load_online_grf_profile",
        "read_setup_xml",
        "calculate_wrench",
        "external_wrench",
        "sample_spheres",
        "sample_spheres_from_coordinate_states",
        "runtime_grid_indices",
    }
    assert all(callable(value) for value in dependencies.values())


def _synthetic_trace(
    *,
    unit_id: str = "synthetic",
    trial_id: str = "02",
    plateau_id: str = "01",
    cadence_s: float = 0.01,
    input_mode: str = "ik_prescribed",
    evidence_role: str = "development",
    delay_s: float = 0.02,
    detector_time_s: float = 0.19,
    reference_unit_id: str | None = None,
) -> audit.UnitTrace:
    times = np.arange(0.0, 1.0 + cadence_s / 2.0, cadence_s)
    oracle_y = np.where((times >= 0.20) & (times < 0.80), 700.0, 0.0)
    primary_y = np.where(
        (times >= 0.20 + delay_s) & (times < 0.80 + delay_s),
        680.0,
        0.0,
    )
    oracle_force = np.column_stack(
        (np.zeros_like(times), oracle_y, np.zeros_like(times))
    )
    primary_force = np.column_stack(
        (np.zeros_like(times), primary_y, np.zeros_like(times))
    )
    zeros3 = np.zeros((len(times), 3), dtype=float)
    active = primary_y > 0.0
    penetration = np.where(active, 0.004, 0.0)
    clearance = np.where(active, -0.004, 0.01)
    side = audit.SideSeries(
        oracle_force_n=oracle_force,
        oracle_cop_m=zeros3.copy(),
        oracle_moment_nm=zeros3.copy(),
        primary_force_n=primary_force,
        primary_normal_force_n=primary_y.copy(),
        primary_cop_m=zeros3.copy(),
        primary_moment_nm=zeros3.copy(),
        primary_penetration_m=penetration.copy(),
        primary_slip_speed_m_s=np.where(active, 0.05, 0.0),
        primary_in_contact=active.copy(),
        mesh_min_clearance_m=np.where(active, -0.002, 0.01),
    )
    sphere = audit.SphereSeries(
        name="left_basis_00",
        side="left",
        normal_force_n=primary_y.copy(),
        penetration_m=penetration.copy(),
        clearance_m=clearance.copy(),
        center_height_m=clearance + 0.02,
        slip_speed_m_s=np.where(active, 0.05, 0.0),
    )
    return audit.UnitTrace(
        unit_id=unit_id,
        trial_id=trial_id,
        plateau_id=plateau_id,
        cadence_s=cadence_s,
        input_mode=input_mode,
        evidence_role=evidence_role,
        start_s=float(times[0]),
        end_s=float(times[-1]),
        times_s=times,
        sides={"left": side},
        spheres=(sphere,),
        detector_events=(
            {
                "side": "left",
                "sensor": "heel",
                "event_name": "heel_strike",
                "onset_time_s": detector_time_s,
                "confirmed_time_s": detector_time_s + cadence_s,
            },
        ),
        reference_unit_id=reference_unit_id,
        plateau_speed_mps=1.0 if evidence_role == "development" else None,
        surface_velocity_mps=(
            (0.0, 0.0, 1.0)
            if evidence_role == "development"
            else None
        ),
    )


def _two_cycle_trace() -> audit.UnitTrace:
    template = _synthetic_trace()
    cadence_s = 0.01
    times = np.arange(0.0, 2.2 + cadence_s / 2.0, cadence_s)
    oracle_active = (
        ((times >= 0.20) & (times < 0.60))
        | ((times >= 1.00) & (times < 1.40))
        | ((times >= 1.80) & (times < 2.00))
    )
    primary_active = (
        ((times >= 0.22) & (times < 0.62))
        | ((times >= 1.02) & (times < 1.42))
        | ((times >= 1.82) & (times < 2.02))
    )
    oracle_y = np.where(oracle_active, 700.0, 0.0)
    primary_y = np.where(primary_active, 680.0, 0.0)
    zeros3 = np.zeros((len(times), 3), dtype=float)
    penetration = np.where(primary_active, 0.004, 0.0)
    side = audit.SideSeries(
        oracle_force_n=np.column_stack(
            (np.zeros_like(times), oracle_y, np.zeros_like(times))
        ),
        oracle_cop_m=zeros3.copy(),
        oracle_moment_nm=zeros3.copy(),
        primary_force_n=np.column_stack(
            (np.zeros_like(times), primary_y, np.zeros_like(times))
        ),
        primary_normal_force_n=primary_y,
        primary_cop_m=zeros3.copy(),
        primary_moment_nm=zeros3.copy(),
        primary_penetration_m=penetration,
        primary_slip_speed_m_s=np.where(primary_active, 0.05, 0.0),
        primary_in_contact=primary_active,
        mesh_min_clearance_m=np.where(primary_active, -0.002, 0.01),
    )
    sphere = audit.SphereSeries(
        name="left_basis_00",
        side="left",
        normal_force_n=primary_y,
        penetration_m=penetration,
        clearance_m=np.where(primary_active, -0.004, 0.01),
        center_height_m=np.where(primary_active, 0.016, 0.03),
        slip_speed_m_s=np.where(primary_active, 0.05, 0.0),
    )
    return replace(
        template,
        start_s=float(times[0]),
        end_s=float(times[-1]),
        times_s=times,
        sides={"left": side},
        spheres=(sphere,),
        detector_events=(
            {
                "side": "left",
                "sensor": "heel",
                "event_name": "heel_strike",
                "onset_time_s": 0.22,
                "confirmed_time_s": 0.22,
            },
            {
                "side": "left",
                "sensor": "heel",
                "event_name": "heel_strike",
                "onset_time_s": 0.90,
                "confirmed_time_s": 1.00,
            },
        ),
    )


def _trace_with_vertical_force(
    times: np.ndarray,
    force_n: np.ndarray,
    *,
    cadence_s: float,
) -> audit.UnitTrace:
    """Build a finite development trace whose oracle and primary are identical."""

    template = _synthetic_trace(cadence_s=cadence_s, delay_s=0.0)
    times = np.asarray(times, dtype=float)
    force_n = np.asarray(force_n, dtype=float)
    zeros = np.zeros_like(times)
    zeros3 = np.zeros((len(times), 3), dtype=float)
    active = force_n > 20.0
    penetration = np.where(active, 0.004, 0.0)
    side = audit.SideSeries(
        oracle_force_n=np.column_stack((zeros, force_n, zeros)),
        oracle_cop_m=zeros3.copy(),
        oracle_moment_nm=zeros3.copy(),
        primary_force_n=np.column_stack((zeros, force_n, zeros)),
        primary_normal_force_n=force_n.copy(),
        primary_cop_m=zeros3.copy(),
        primary_moment_nm=zeros3.copy(),
        primary_penetration_m=penetration.copy(),
        primary_slip_speed_m_s=np.where(active, 0.05, 0.0),
        primary_in_contact=active.copy(),
        mesh_min_clearance_m=np.where(active, -0.002, 0.01),
    )
    sphere = audit.SphereSeries(
        name="left_basis_00",
        side="left",
        normal_force_n=force_n.copy(),
        penetration_m=penetration.copy(),
        clearance_m=np.where(active, -0.004, 0.01),
        center_height_m=np.where(active, 0.016, 0.03),
        slip_speed_m_s=np.where(active, 0.05, 0.0),
    )
    return replace(
        template,
        cadence_s=cadence_s,
        start_s=float(times[0]),
        end_s=float(times[-1]),
        times_s=times,
        sides={"left": side},
        spheres=(sphere,),
        detector_events=(),
    )


def _record(path: Path, root: Path) -> dict[str, Any]:
    return gate.source_record(path, root)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _audit_payload(source_record: dict[str, Any]) -> dict[str, Any]:
    roles = {
        "03": "PRIMARY_VALIDATION_ONE_SHOT",
        "05": "DETECTOR_VALIDATION_ONE_SHOT",
        "06": "DETECTOR_SEALED_ONE_SHOT",
        "07": "PRIMARY_SEALED_ONE_SHOT",
    }
    frozen = {
        trial: {
            "role": role,
            "component_outcome_semantic_access_started": False,
            "one_shot_status": "NOT_CONSUMED",
        }
        for trial, role in roles.items()
    }
    protected = {
        trial: {
            "component_outcome_semantic_access_started": False,
            "one_shot_status": "NOT_CONSUMED",
            "outcome_access_receipts_found": [],
            "status": "CLOSED",
        }
        for trial in roles
    }
    return {
        "schema_version": 1,
        "status": "PASS",
        "decision": "GLOBAL_DATA_BUDGET_ADOPTABLE",
        "source_hashes_match": True,
        "protected_outcome_access_found": False,
        "protected_run_authorized": False,
        "protected_conditions_metadata_semantic_access_started": True,
        "protected_component_outcome_semantic_access_started": False,
        "frozen_allocation": frozen,
        "protected_trials": protected,
        "source_records": [source_record],
        "failed_checks": [],
    }


def _build_preflight_bundle(
    root: Path,
    *,
    precomputed: bool = False,
) -> dict[str, Path]:
    source = root / "sources" / "artifact.dat"
    source.parent.mkdir(parents=True)
    source.write_bytes(b"hash-bound synthetic source\n")
    profile = root / "profiles" / "primary.json"
    _write_json(profile, {"schema_version": 1, "profile": "synthetic"})
    source_record = _record(source, root)
    profile_record = _record(profile, root)
    protocol_sources: dict[str, dict[str, Any]] = {
        "unit_fixture": source_record,
    }
    for name, relative in audit._REQUIRED_STATIC_PROTOCOL_SOURCES.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"synthetic {name}\n".encode("utf-8"))
        protocol_sources[name] = _record(path, root)
    for name in sorted(audit._REQUIRED_DYNAMIC_PROTOCOL_SOURCES):
        path = root / "runtime_sources" / f"{name}.dat"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"synthetic {name}\n".encode("utf-8"))
        protocol_sources[name] = _record(path, root)

    audit_path = root / "validation" / "global_audit.json"
    _write_json(audit_path, _audit_payload(source_record))
    audit_record = _record(audit_path, root)

    def records_for_trial(trial_id: str) -> dict[str, Any]:
        trial_source = (
            root / "unit_sources" / f"trial{trial_id}" / "artifact.dat"
        )
        trial_source.parent.mkdir(parents=True, exist_ok=True)
        if not trial_source.exists():
            trial_source.write_bytes(
                f"synthetic trial {trial_id}\n".encode("utf-8")
            )
        trial_record = _record(trial_source, root)
        records = {
            "setup": trial_record,
            "grf_sto": trial_record,
            "external_loads_xml": trial_record,
            "sea_plugin": source_record,
            "ik_sto": trial_record,
        }
        if precomputed:
            records.update(
                {
                    "trace_csv": trial_record,
                    "per_sphere_csv": trial_record,
                    "trace_primary_profile": profile_record,
                }
            )
        return records

    def attach_source_receipt(
        unit: dict[str, Any],
        *,
        access_role: str,
    ) -> dict[str, Any]:
        fields = {
            "setup",
            "ik_sto",
            "grf_sto",
            "external_loads_xml",
        }
        if "trace_csv" in unit:
            fields.update({"trace_csv", "per_sphere_csv"})
        if unit["input_mode"] in {"coordinate_states", "h0_historical"}:
            fields.add("states_sto")
        if unit["input_mode"] == "h0_historical":
            fields.add("online_grf_sto")
        receipt_path = (
            root
            / "validation"
            / "unit_source_receipts"
            / f"{unit['unit_id']}.json"
        )
        _write_json(
            receipt_path,
            {
                "schema_version": 1,
                "status": "PASS",
                "unit_id": unit["unit_id"],
                "source_trial_id": unit["trial_id"],
                "plateau_id": unit["plateau_id"],
                "cadence_s": unit["cadence_s"],
                "start_s": unit["start_s"],
                "end_s": unit["end_s"],
                "plateau_speed_mps": unit.get("plateau_speed_mps"),
                "surface_velocity_mps": unit.get("surface_velocity_mps"),
                "access_role": access_role,
                "protected_component_outcome": False,
                "semantic_sources": {
                    field: unit[field]
                    for field in sorted(fields)
                },
            },
        )
        unit["unit_source_receipt"] = _record(receipt_path, root)
        return unit

    authorized = []
    for trial_id in audit.AUTHORIZED_TRIALS:
        for plateau_id in ("01", "02", "03", "04"):
            for cadence_s in audit.CADENCES_S:
                authorized.append(
                    attach_source_receipt(
                        {
                        "unit_id": (
                            f"dev{trial_id}_p{plateau_id}_"
                            f"{int(cadence_s * 1000):02d}ms"
                        ),
                        "trial_id": trial_id,
                        "plateau_id": plateau_id,
                        "cadence_s": cadence_s,
                        "start_s": 2.0 * (int(plateau_id) - 1),
                        "end_s": 2.0 * (int(plateau_id) - 1) + 1.0,
                        "plateau_speed_mps": float(int(plateau_id)),
                        "surface_velocity_mps": [
                            0.0,
                            0.0,
                            float(int(plateau_id)),
                        ],
                        "input_mode": "ik_prescribed",
                        **records_for_trial(trial_id),
                        },
                        access_role="DEVELOPMENT_OPEN",
                    )
                )
    historical_ik = {
        "unit_id": "historic01_ik_10ms",
        "trial_id": "01",
        "plateau_id": "01",
        "cadence_s": 0.01,
        "start_s": 0.0,
        "end_s": 1.0,
        "input_mode": "ik_prescribed",
        **records_for_trial("01"),
    }
    state_common = records_for_trial("01")
    coordinate = {
        "unit_id": "historic01_states_10ms",
        "trial_id": "01",
        "plateau_id": "01",
        "cadence_s": 0.01,
        "start_s": 0.0,
        "end_s": 1.0,
        "input_mode": "coordinate_states",
        "reference_unit_id": historical_ik["unit_id"],
        "states_sto": state_common["ik_sto"],
        **state_common,
    }
    historical_h0 = {
        "unit_id": "historic01_h0_10ms",
        "trial_id": "01",
        "plateau_id": "01",
        "cadence_s": 0.01,
        "start_s": 0.0,
        "end_s": 1.0,
        "input_mode": "h0_historical",
        "reference_unit_id": historical_ik["unit_id"],
        "states_sto": state_common["ik_sto"],
        "online_grf_sto": state_common["ik_sto"],
        **state_common,
    }
    historical_ik = attach_source_receipt(
        historical_ik,
        access_role="HISTORICAL_CONSUMED",
    )
    coordinate = attach_source_receipt(
        coordinate,
        access_role="HISTORICAL_CONSUMED",
    )
    historical_h0 = attach_source_receipt(
        historical_h0,
        access_role="HISTORICAL_CONSUMED",
    )
    protocol = {
        "schema_version": 1,
        "step_id": "0",
        "status": "FROZEN",
        "global_data_access_audit": audit_record,
        "sources": protocol_sources,
        "primary_profile": profile_record,
        "candidate_grid": source_record,
        "physical_contract": {
            "primary_applied_side": "left",
            "right_physical_support": "prescribed",
            "detector_role": "shadow_diagnostic",
            "detector_may_affect_primary_metrics": False,
        },
        "authorized_units": authorized,
        "diagnostic_units": [historical_ik, coordinate, historical_h0],
        "baseline_forensics": {
            "output_dir": "baseline_forensics",
            "crossing_thresholds_n": [0.5, 5.0, 20.0],
            "oracle_threshold_n": 20.0,
            "oracle_event_conditioning": {
                "scope": "development_oracle_and_primary_20n",
                "algorithm": "StreamingGaitEventDetector",
                "threshold_n": 20.0,
                "confirmation_threshold_n": 20.0,
                "min_contact_duration_s": 0.05,
                "min_cycle_duration_s": 0.30,
                "paired_cadence_right_margin_s": 0.010,
                "common_last_sample_policy": (
                    "max_cadence_grid_from_frozen_window"
                ),
                "boundary_censor_policy": (
                    "exclude_from_gating_retain_diagnostic"
                ),
                "historical_timing_policy": (
                    "preserve_linear_interpolated_raw"
                ),
            },
            "event_pairing_tolerance_s": 0.30,
            "output_contract": {
                "artifact_count": 7,
                "csv_publication": (
                    "single_pass_streaming_fixed_schema"
                ),
                "staging": (
                    "private_sibling_o_excl_fsync_then_revalidate"
                ),
                "finalization": (
                    "atomic_directory_rename_noreplace"
                ),
                "commit_marker": (
                    "atomic_seven_artifact_directory_commit"
                ),
                "per_sphere_scope": (
                    "primary_left_sphere_per_complete_oracle_cycle"
                ),
                "no_complete_cycle_fallback": (
                    "whole_window_diagnostic_only_excluded_from_gating"
                ),
                "per_sphere_row_bound": (
                    "sphere_count_x_max_1_complete_oracle_cycle_count"
                ),
                "dominant_sphere_tie_break": (
                    "normal_impulse_desc_then_peak_force_desc_then_name_asc"
                ),
            },
            "root_cause_thresholds": {
                "timing_late_s": 0.05,
                "material_force_rise_lag_s": 0.02,
                "detector_early_lead_s": 0.01,
                "penetration_limit_m": 0.015,
                "impulse_ratio_min": 0.8,
                "impulse_ratio_max": 1.2,
            },
            "reproduction_targets": [
                {
                    "target_id": "ik_delay",
                    "input_mode": "ik_prescribed",
                    "side": "left",
                    "metric": "primary_hs_delay_median_s",
                    "aggregation": "median",
                    "expected_s": 0.095,
                    "absolute_tolerance_s": 0.010,
                    "evidence_role": "development",
                    "unit_ids": [
                        unit["unit_id"]
                        for unit in authorized
                    ],
                },
                {
                    "target_id": "h0_delay",
                    "input_mode": "h0_historical",
                    "side": "left",
                    "metric": "primary_hs_delay_median_s",
                    "aggregation": "median",
                    "expected_s": 0.173,
                    "absolute_tolerance_s": 0.010,
                    "evidence_role": "diagnostic",
                    "unit_ids": [historical_h0["unit_id"]],
                },
            ],
        },
    }
    protocol_path = root / "validation" / "protocol.json"
    _write_json(protocol_path, protocol)
    protocol_sha = gate.sha256_file(protocol_path)

    run_dir = root / "validation" / "primary_grf_runs" / "synthetic"
    ledger_path = root / "validation" / "ledger.json"
    _write_json(
        ledger_path,
        {
            "schema_version": 1,
            "step_id": "0",
            "status": "RUNNING",
            "run_id": "synthetic",
            "protocol_sha256": protocol_sha,
            "global_data_access_audit_sha256": audit_record["sha256"],
        },
    )
    receipt_path = run_dir / "run_start_receipt.json"
    _write_json(
        receipt_path,
        {
            "schema_version": 1,
            "step_id": "0",
            "status": "RUNNING",
            "run_id": "synthetic",
            "no_clobber": True,
            "process_isolation": {
                "ray_processes_active": False,
                "opensim_processes_active": False,
                "baseline_new_process_required": True,
                "checked_at_utc": "2026-07-24T00:00:00Z",
                "inspection_command": "synthetic process inspection",
            },
            "gate_0_0_tests": [
                {
                    "command": "pytest synthetic",
                    "passed": True,
                    "passed_count": 1,
                    "failed_count": 0,
                }
            ],
            "baseline_launch": {
                "command": "python audit --execute-offline",
                "fresh_process": True,
            },
            "protocol": _record(protocol_path, root),
            "global_data_access_audit": audit_record,
            "execution_ledger": _record(ledger_path, root),
            "primary_profile": profile_record,
            "protected_trials_opened": [],
        },
    )
    return {
        "protocol": protocol_path,
        "audit": audit_path,
        "ledger": ledger_path,
        "receipt": receipt_path,
        "profile": profile,
    }


def _publication_fixture(
    root: Path,
) -> tuple[
    audit.PreflightContext,
    list[audit.UnitTrace],
    dict[str, dict[str, Any]],
]:
    files = _build_preflight_bundle(root)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=root,
    )
    traces: list[audit.UnitTrace] = []
    for unit in context.authorized_units + context.diagnostic_units:
        cadence_s = float(unit["cadence_s"])
        start_s = float(unit["start_s"])
        end_s = float(unit["end_s"])
        times = audit._anchored_time_grid(start_s, end_s, cadence_s)
        relative = times - start_s
        active = (
            ((relative >= 0.10) & (relative < 0.22))
            | ((relative >= 0.45) & (relative < 0.57))
            | ((relative >= 0.80) & (relative < 0.92))
        )
        force = np.where(active, 100.0, 0.0)
        trace = _trace_with_vertical_force(
            times,
            force,
            cadence_s=cadence_s,
        )
        traces.append(
            replace(
                trace,
                unit_id=str(unit["unit_id"]),
                trial_id=str(unit["trial_id"]),
                plateau_id=str(unit["plateau_id"]),
                input_mode=str(unit["input_mode"]),
                evidence_role=str(unit["evidence_role"]),
                reference_unit_id=unit.get("reference_unit_id"),
                plateau_speed_mps=unit.get("plateau_speed_mps"),
                surface_velocity_mps=(
                    tuple(unit["surface_velocity_mps"])
                    if unit.get("surface_velocity_mps") is not None
                    else None
                ),
            )
        )
    analyses = {
        trace.unit_id: audit.analyze_unit(trace)
        for trace in traces
    }
    audit._apply_closed_loop_delays(analyses)
    audit._refresh_required_metric_completeness(analyses)
    assert all(
        analysis["primary_metrics"]["left"]["required_metrics_finite"]
        for analysis in analyses.values()
        if analysis["evidence_role"] == "development"
    )
    return context, traces, analyses


def test_trace_validation_rejects_nan_and_irregular_cadence() -> None:
    trace = _synthetic_trace()
    bad_force = trace.sides["left"].oracle_force_n.copy()
    bad_force[4, 1] = np.nan
    bad_side = replace(trace.sides["left"], oracle_force_n=bad_force)
    with pytest.raises(audit.TraceContractError, match="NaN or Infinity"):
        audit.validate_unit_trace(replace(trace, sides={"left": bad_side}))

    irregular = trace.times_s.copy()
    irregular[30:] += 0.001
    with pytest.raises(
        audit.TraceContractError,
        match="beyond frozen|terminal gap|cadence mismatch",
    ):
        audit.validate_unit_trace(replace(trace, times_s=irregular))


def test_trace_validation_binds_exact_frozen_window() -> None:
    trace = _synthetic_trace()
    exact = audit._anchored_time_grid(11.698, 44.883, 0.001)
    assert exact[0] == 11.698
    assert exact[-1] == 44.883
    audit.validate_unit_trace(trace)

    half_cadence = replace(trace, end_s=trace.end_s + 0.005)
    audit.validate_unit_trace(half_cadence)
    generated = audit._anchored_time_grid(11.698, 44.883, 0.010)
    assert generated[0] == 11.698
    assert generated[-1] <= 44.883
    assert 0.0 <= 44.883 - generated[-1] < 0.010
    np.testing.assert_allclose(
        np.diff(generated),
        0.010,
        rtol=0.0,
        atol=audit._TIME_GRID_ATOL_S,
    )

    overshoot = np.append(trace.times_s, trace.end_s + trace.cadence_s)
    with pytest.raises(audit.TraceContractError, match="beyond frozen"):
        audit.validate_unit_trace(replace(trace, times_s=overshoot))
    tiny_overshoot = trace.times_s.copy()
    tiny_overshoot[-1] = np.nextafter(trace.end_s, np.inf)
    with pytest.raises(audit.TraceContractError, match="not sample beyond"):
        audit.validate_unit_trace(replace(trace, times_s=tiny_overshoot))

    missing_last_tick = trace.times_s[:-1]
    with pytest.raises(audit.TraceContractError, match="terminal gap"):
        audit.validate_unit_trace(replace(trace, times_s=missing_last_tick))


def test_trace_validation_fails_closed_on_missing_primary() -> None:
    trace = _synthetic_trace()
    with pytest.raises(audit.TraceContractError, match="required primary side"):
        audit.validate_unit_trace(replace(trace, sides={}))
    with pytest.raises(audit.TraceContractError, match="no per-sphere evidence"):
        audit.validate_unit_trace(replace(trace, spheres=()))


@pytest.mark.parametrize(
    ("event_name", "onset_time_s", "confirmed_time_s", "message"),
    (
        ("midstance", 0.20, 0.21, "event_name"),
        ("heel_strike", 0.22, 0.21, "trace_start <= onset"),
        ("heel_strike", -0.01, 0.01, "trace_start <= onset"),
        ("toe_off", 0.99, 1.01, "trace_start <= onset"),
    ),
)
def test_trace_validation_rejects_invalid_detector_event_timing(
    event_name: str,
    onset_time_s: float,
    confirmed_time_s: float,
    message: str,
) -> None:
    trace = _synthetic_trace()
    detector_event = {
        "side": "left",
        "sensor": "heel",
        "event_name": event_name,
        "onset_time_s": onset_time_s,
        "confirmed_time_s": confirmed_time_s,
    }
    with pytest.raises(audit.TraceContractError, match=message):
        audit.validate_unit_trace(
            replace(trace, detector_events=(detector_event,))
        )


def test_detector_change_cannot_change_primary_metric_inputs() -> None:
    trace = _synthetic_trace(detector_time_s=0.19)
    changed = replace(
        trace,
        detector_events=(
            {
                "side": "left",
                "sensor": "toe",
                "event_name": "heel_strike",
                "onset_time_s": 0.01,
                "confirmed_time_s": 0.02,
            },
        ),
    )
    first = audit.analyze_unit(trace)
    second = audit.analyze_unit(changed)
    assert audit.metric_input_fingerprint(trace) == audit.metric_input_fingerprint(
        changed
    )
    assert first["primary_metrics"] == second["primary_metrics"]
    assert first["detector_diagnostics"] != second["detector_diagnostics"]


def test_detector_events_are_recorded_and_classified_per_cycle() -> None:
    trace = _two_cycle_trace()
    analysis = audit.analyze_unit(trace)
    detector_cycles = analysis["detector_diagnostics"]["by_side"]["left"]["cycles"]
    assert [cycle["cycle_index"] for cycle in detector_cycles] == [0, 1]
    detector_rows = [
        row
        for row in audit.event_rows(trace)
        if row["source"] == "detector_diagnostic_only"
    ]
    assert [row["cycle_index"] for row in detector_rows] == [0, 1]

    result = audit.classify_all_units(
        {trace.unit_id: analysis},
        thresholds={
            "timing_late_s": 0.05,
            "material_force_rise_lag_s": 0.02,
            "detector_early_lead_s": 0.01,
            "penetration_limit_m": 0.015,
            "impulse_ratio_min": 0.8,
            "impulse_ratio_max": 1.2,
        },
    )
    cycles = result["units"][trace.unit_id]["sides"]["left"]["cycles"]
    assert "detector_early_not_primary_error" not in cycles[0]["classes"]
    assert "detector_early_not_primary_error" in cycles[1]["classes"]


def test_root_cause_rules_are_deterministic() -> None:
    thresholds = {
        "timing_late_s": 0.05,
        "material_force_rise_lag_s": 0.02,
        "detector_early_lead_s": 0.01,
        "penetration_limit_m": 0.015,
        "impulse_ratio_min": 0.8,
        "impulse_ratio_max": 1.2,
    }
    base = {
        "mesh_contact_present": False,
        "sphere_penetration_present": False,
        "primary_20n_present": True,
        "force_rise_after_penetration_s": None,
        "primary_hs_delay_max_abs_s": 0.02,
        "closed_loop_additional_delay_s": None,
        "detector_lead_vs_mesh_or_primary_s": None,
        "penetration_max_m": 0.004,
        "impulse_ratio": 1.0,
    }
    cases = [
        (
            {**base, "mesh_contact_present": True},
            None,
            "geometry_ground_primary",
        ),
        (
            {
                **base,
                "sphere_penetration_present": True,
                "primary_20n_present": False,
            },
            None,
            "material_contact_law",
        ),
        (
            {
                **base,
                "primary_hs_delay_max_abs_s": 0.10,
                "closed_loop_additional_delay_s": 0.08,
            },
            {**base, "primary_hs_delay_max_abs_s": 0.01},
            "initialization_or_closed_loop",
        ),
        (
            {**base, "detector_lead_vs_mesh_or_primary_s": 0.03},
            None,
            "detector_early_not_primary_error",
        ),
        (
            {**base, "penetration_max_m": 0.020},
            None,
            "physical_primary_support",
        ),
    ]
    for metrics, reference, expected in cases:
        first = audit.classify_root_cause(
            metrics,
            thresholds=thresholds,
            reference_metrics=reference,
        )
        second = audit.classify_root_cause(
            copy.deepcopy(metrics),
            thresholds=copy.deepcopy(thresholds),
            reference_metrics=copy.deepcopy(reference),
        )
        assert first == second
        assert expected in first["classes"]

    ambiguous_support = audit.classify_root_cause(
        {
            **base,
            "penetration_max_m": 0.020,
        },
        thresholds=thresholds,
    )
    assert "physical_primary_support" in ambiguous_support["classes"]
    assert ambiguous_support["requires_discriminating_experiment"] is True
    assert ambiguous_support["modifiable_parameter_groups"] == [
        "primary_support_discriminator_required"
    ]


def test_left_censored_contact_is_diagnostic_and_never_arms_detector() -> None:
    trace = _synthetic_trace()
    side = trace.sides["left"]
    always_active = np.full_like(side.primary_normal_force_n, 100.0)
    active_side = replace(
        side,
        primary_force_n=np.column_stack(
            (
                np.zeros_like(always_active),
                always_active,
                np.zeros_like(always_active),
            )
        ),
        primary_normal_force_n=always_active,
        primary_penetration_m=np.full_like(always_active, 0.003),
        primary_in_contact=np.ones_like(side.primary_in_contact),
        mesh_min_clearance_m=np.full_like(always_active, -0.001),
    )
    sphere = replace(
        trace.spheres[0],
        normal_force_n=always_active,
        penetration_m=np.full_like(always_active, 0.003),
        clearance_m=np.full_like(always_active, -0.003),
    )
    analysis = audit.analyze_unit(
        replace(trace, sides={"left": active_side}, spheres=(sphere,))
    )
    metrics = analysis["primary_metrics"]["left"]
    assert metrics["primary_first_20n_s"] is None
    assert metrics["primary_20n_present"] is False
    assert metrics["primary_crossings"]["20N"]["heel_strikes_s"] == []
    assert metrics["primary_crossings"]["20N"]["left_boundary_censored"] is True
    assert metrics["mesh_first_contact_s"] == pytest.approx(trace.times_s[0])
    assert metrics["sphere_first_penetration_s"] == pytest.approx(trace.times_s[0])
    conditioned_rows = [
        row
        for row in audit.event_rows(
            replace(trace, sides={"left": active_side}, spheres=(sphere,))
        )
        if row["source"] == "primary_conditioned_20n"
    ]
    assert conditioned_rows
    assert all(row["diagnostic_only"] for row in conditioned_rows)
    assert all(row["excluded_from_gating"] for row in conditioned_rows)


def test_conditioned_events_reject_short_spike_and_accept_exact_dwell() -> None:
    times = np.arange(0.0, 1.21, 0.01)
    short = np.where((times >= 0.20) & (times < 0.25), 100.0, 0.0)
    short_trace = _trace_with_vertical_force(times, short, cadence_s=0.01)
    short_bundle = audit._conditioned_event_bundle(
        short_trace,
        short,
        20.0,
    )
    assert short_bundle["genuine_heel_strike"].tolist() == []
    assert short_bundle["genuine_toe_off"].tolist() == []

    exact = np.where((times >= 0.20) & (times < 0.26), 100.0, 0.0)
    exact_trace = _trace_with_vertical_force(times, exact, cadence_s=0.01)
    exact_bundle = audit._conditioned_event_bundle(
        exact_trace,
        exact,
        20.0,
    )
    assert exact_bundle["genuine_heel_strike"].tolist() == pytest.approx([0.20])
    assert exact_bundle["genuine_heel_strike_confirmed"].tolist() == pytest.approx(
        [0.25]
    )
    assert exact_bundle["genuine_toe_off"].tolist() == pytest.approx([0.26])


def test_conditioned_events_rearm_only_after_initial_contact_falls_below() -> None:
    times = np.arange(0.0, 1.41, 0.01)
    active = (
        (times < 0.10)
        | ((times >= 0.20) & (times < 0.30))
        | ((times >= 0.60) & (times < 0.70))
        | ((times >= 1.00) & (times < 1.10))
    )
    force = np.where(active, 100.0, 0.0)
    trace = _trace_with_vertical_force(times, force, cadence_s=0.01)
    bundle = audit._conditioned_event_bundle(trace, force, 20.0)
    assert bundle["left_boundary_censored"] is True
    assert bundle["genuine_heel_strike"].tolist() == pytest.approx(
        [0.20, 0.60, 1.00]
    )
    assert bundle["heel_strike"].tolist() == pytest.approx(
        [0.20, 0.60, 1.00]
    )
    assert [record["cycle_index"] for record in bundle["complete_cycles"]] == [
        0,
        1,
    ]
    assert any(
        record["boundary"] == "left"
        for record in bundle["censored_records"]
    )


def test_conditioned_events_suppress_short_cycle_without_orphan_toe_off() -> None:
    times = np.arange(0.0, 1.41, 0.01)
    active = (
        ((times >= 0.20) & (times < 0.30))
        | ((times >= 0.40) & (times < 0.50))
        | ((times >= 0.70) & (times < 0.80))
        | ((times >= 1.10) & (times < 1.20))
    )
    force = np.where(active, 100.0, 0.0)
    trace = _trace_with_vertical_force(times, force, cadence_s=0.01)
    bundle = audit._conditioned_event_bundle(trace, force, 20.0)
    assert bundle["genuine_heel_strike"].tolist() == pytest.approx(
        [0.20, 0.70, 1.10]
    )
    assert bundle["genuine_toe_off"].tolist() == pytest.approx(
        [0.30, 0.80, 1.20]
    )
    assert not np.any(
        np.isclose(
            bundle["genuine_toe_off"],
            0.50,
            rtol=0.0,
            atol=1e-12,
        )
    )
    assert all(
        record["toe_off_count"] == 1
        for record in bundle["complete_cycles"]
    )


def test_conditioned_events_exclude_terminal_and_common_margin_cycles() -> None:
    times = np.arange(0.0, 1.021, 0.01)
    active = (
        ((times >= 0.20) & (times < 0.30))
        | ((times >= 0.60) & (times < 0.70))
        | (times >= 0.97)
    )
    force = np.where(active, 100.0, 0.0)
    trace = _trace_with_vertical_force(times, force, cadence_s=0.01)
    bundle = audit._conditioned_event_bundle(trace, force, 20.0)
    assert bundle["genuine_heel_strike"].tolist() == pytest.approx(
        [0.20, 0.60, 0.97]
    )
    assert [record["cycle_index"] for record in bundle["complete_cycles"]] == [0]
    assert bundle["incomplete_cycles"][0]["exclusion_reason"] == (
        "right_boundary_common_confirmation_margin"
    )
    assert bundle["right_boundary_censored"] is True
    assert bundle["heel_strike"].tolist() == pytest.approx([0.20, 0.60])
    assert bundle["toe_off"].tolist() == pytest.approx([0.30])


def test_conditioned_cycle_selection_matches_one_and_ten_ms_cadences() -> None:
    bundles = []
    for cadence_s in (0.001, 0.010):
        times = audit._anchored_time_grid(0.0, 3.0, cadence_s)
        active = np.zeros_like(times, dtype=bool)
        for start in (0.20, 0.80, 1.40, 2.00, 2.60):
            active |= (times >= start) & (times < start + 0.30)
        force = np.where(active, 100.0, 0.0)
        trace = _trace_with_vertical_force(
            times,
            force,
            cadence_s=cadence_s,
        )
        bundles.append(audit._conditioned_event_bundle(trace, force, 20.0))
    assert [len(bundle["genuine_heel_strike"]) for bundle in bundles] == [5, 5]
    assert [
        [record["cycle_index"] for record in bundle["complete_cycles"]]
        for bundle in bundles
    ] == [[0, 1, 2, 3], [0, 1, 2, 3]]


def test_pair_level_contract_rejects_one_ms_only_short_contact() -> None:
    traces: list[audit.UnitTrace] = []
    for trial_id in audit.AUTHORIZED_TRIALS:
        for plateau_id in ("01", "02", "03", "04"):
            for cadence_s in audit.CADENCES_S:
                times = audit._anchored_time_grid(0.0, 1.70, cadence_s)
                if trial_id == "02" and plateau_id == "01":
                    intervals = (
                        (0.203, 0.257),
                        (0.60, 0.70),
                        (1.00, 1.10),
                        (1.40, 1.50),
                    )
                else:
                    intervals = (
                        (0.20, 0.30),
                        (0.60, 0.70),
                        (1.00, 1.10),
                        (1.40, 1.50),
                    )
                active = np.zeros_like(times, dtype=bool)
                for start, end in intervals:
                    active |= (times >= start) & (times < end)
                trace = _trace_with_vertical_force(
                    times,
                    np.where(active, 100.0, 0.0),
                    cadence_s=cadence_s,
                )
                traces.append(
                    replace(
                        trace,
                        unit_id=(
                            f"dev{trial_id}_p{plateau_id}_"
                            f"{int(cadence_s * 1000):02d}ms"
                        ),
                        trial_id=trial_id,
                        plateau_id=plateau_id,
                    )
                )
    with pytest.raises(
        audit.TraceContractError,
        match="unequal.*complete-cycle counts|unequal conditioned",
    ):
        audit.validate_development_cadence_event_pairs(traces)


def test_geometry_class_uses_mesh_to_eventual_sphere_delay() -> None:
    thresholds = {
        "timing_late_s": 0.05,
        "material_force_rise_lag_s": 0.02,
        "detector_early_lead_s": 0.01,
        "penetration_limit_m": 0.015,
        "impulse_ratio_min": 0.8,
        "impulse_ratio_max": 1.2,
    }
    base = {
        "mesh_contact_present": True,
        "sphere_penetration_present": True,
        "primary_20n_present": True,
        "force_rise_after_penetration_s": 0.0,
        "primary_hs_delay_max_abs_s": 0.02,
        "closed_loop_additional_delay_s": None,
        "detector_lead_vs_mesh_or_primary_s": None,
        "penetration_max_m": 0.004,
        "impulse_ratio": 1.0,
    }
    delayed = audit.classify_root_cause(
        {**base, "mesh_to_sphere_contact_delay_s": 0.085},
        thresholds=thresholds,
    )
    simultaneous = audit.classify_root_cause(
        {**base, "mesh_to_sphere_contact_delay_s": 0.005},
        thresholds=thresholds,
    )
    assert "geometry_ground_primary" in delayed["classes"]
    assert "geometry_ground_primary" not in simultaneous["classes"]


def test_right_shadow_cannot_affect_left_primary_classification() -> None:
    trace = _synthetic_trace()
    left = trace.sides["left"]
    with_right = replace(trace, sides={"left": left, "right": copy.deepcopy(left)})
    hostile_force = np.full_like(left.primary_force_n, 1.0e6)
    hostile_right = replace(
        left,
        primary_force_n=hostile_force,
        primary_normal_force_n=np.full(len(trace.times_s), 1.0e6),
        primary_penetration_m=np.full(len(trace.times_s), 0.5),
        primary_slip_speed_m_s=np.full(len(trace.times_s), 50.0),
        primary_in_contact=np.ones(len(trace.times_s), dtype=bool),
        mesh_min_clearance_m=None,
    )
    hostile = replace(trace, sides={"left": left, "right": hostile_right})
    thresholds = {
        "timing_late_s": 0.05,
        "material_force_rise_lag_s": 0.02,
        "detector_early_lead_s": 0.01,
        "penetration_limit_m": 0.015,
        "impulse_ratio_min": 0.8,
        "impulse_ratio_max": 1.2,
    }
    first = audit.classify_all_units(
        {trace.unit_id: audit.analyze_unit(with_right)},
        thresholds=thresholds,
    )
    second = audit.classify_all_units(
        {trace.unit_id: audit.analyze_unit(hostile)},
        thresholds=thresholds,
    )
    assert first["units"] == second["units"]
    assert set(first["units"][trace.unit_id]["sides"]) == {"left"}


def test_mixed_cycle_causes_are_not_hidden_by_unit_aggregation() -> None:
    trace = _synthetic_trace()
    analysis = audit.analyze_unit(trace)
    aggregate = analysis["primary_metrics"]["left"]
    cycle_base = {
        "cycle_start_s": 0.0,
        "cycle_end_s": 1.0,
        "mesh_contact_present": False,
        "mesh_first_contact_s": None,
        "sphere_penetration_present": False,
        "sphere_first_penetration_s": None,
        "mesh_to_sphere_contact_delay_s": None,
        "primary_20n_present": True,
        "primary_first_20n_s": 0.1,
        "primary_hs_delay_median_s": 0.02,
        "primary_hs_delay_max_abs_s": 0.02,
        "primary_to_delay_median_s": 0.01,
        "force_rise_after_penetration_s": None,
        "impulse_ratio": 1.0,
        "cop_horizontal_rmse_m": 0.01,
        "force": {"rmse": 1.0},
        "moment": {"rmse": 1.0},
        "penetration_max_m": 0.004,
        "slip_speed_max_m_s": 0.1,
        "closed_loop_additional_delay_s": None,
        "required_metrics_finite": True,
    }
    aggregate["cycles"] = [
        {
            **cycle_base,
            "cycle_index": 0,
            "mesh_contact_present": True,
            "mesh_first_contact_s": 0.1,
        },
        {
            **cycle_base,
            "cycle_index": 1,
            "cycle_start_s": 1.0,
            "cycle_end_s": 2.0,
            "sphere_penetration_present": True,
            "sphere_first_penetration_s": 1.1,
            "primary_20n_present": False,
            "primary_first_20n_s": None,
        },
    ]
    thresholds = {
        "timing_late_s": 0.05,
        "material_force_rise_lag_s": 0.02,
        "detector_early_lead_s": 0.01,
        "penetration_limit_m": 0.015,
        "impulse_ratio_min": 0.8,
        "impulse_ratio_max": 1.2,
    }
    result = audit.classify_all_units(
        {trace.unit_id: analysis},
        thresholds=thresholds,
    )
    side = result["units"][trace.unit_id]["sides"]["left"]
    assert "geometry_ground_primary" in side["classes"]
    assert "material_contact_law" in side["classes"]
    assert side["mixed_cycle_geometry_material"] is True
    assert side["requires_discriminating_experiment"] is True
    assert result["status"] == "BLOCKED"


@pytest.mark.parametrize(
    "unsafe",
    ["../escape", "NUL.txt", "nested/CON", "file:stream", r"a\b", "/absolute"],
)
def test_portable_path_contract_rejects_windows_and_escape_cases(
    unsafe: str,
) -> None:
    with pytest.raises(audit.PreflightError):
        audit.safe_protocol_relative_path(unsafe, label="test")


def test_preflight_accepts_exact_matrix_and_refuses_occupied_output(
    tmp_path: Path,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=tmp_path,
    )
    assert len(context.authorized_units) == 24
    assert {unit["input_mode"] for unit in context.diagnostic_units} == {
        "ik_prescribed",
        "coordinate_states",
        "h0_historical",
    }
    context.output_dir.mkdir()
    with pytest.raises(gate.NoClobberError):
        audit.preflight(
            protocol_path=files["protocol"],
            audit_path=files["audit"],
            ledger_path=files["ledger"],
            receipt_path=files["receipt"],
            repo_root=tmp_path,
        )


def test_preflight_rejects_precomputed_production_route(tmp_path: Path) -> None:
    files = _build_preflight_bundle(tmp_path, precomputed=True)
    with pytest.raises(audit.PreflightError, match="forbids precomputed"):
        audit.preflight(
            protocol_path=files["protocol"],
            audit_path=files["audit"],
            ledger_path=files["ledger"],
            receipt_path=files["receipt"],
            repo_root=tmp_path,
        )


def test_development_matrix_binds_cadence_pairs_and_distinct_windows(
    tmp_path: Path,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=tmp_path,
    )
    cadence_mismatch = copy.deepcopy(list(context.authorized_units))
    cadence_mismatch[1]["start_s"] += 0.1
    with pytest.raises(audit.PreflightError, match="same start_s"):
        audit._validate_unit_matrix(
            cadence_mismatch,
            context.diagnostic_units,
        )

    overlapping = copy.deepcopy(list(context.authorized_units))
    for unit in overlapping:
        if unit["trial_id"] == "02" and unit["plateau_id"] == "02":
            unit["start_s"] = 0.5
            unit["end_s"] = 1.5
    with pytest.raises(audit.PreflightError, match="non-overlapping"):
        audit._validate_unit_matrix(
            overlapping,
            context.diagnostic_units,
        )


def test_loaded_profile_hash_is_checked_again_after_preflight(
    tmp_path: Path,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=tmp_path,
    )
    files["profile"].write_text('{"drift": true}\n', encoding="utf-8")
    with pytest.raises(audit.PreflightError, match="differs"):
        audit.verify_loaded_primary_profile(
            files["profile"],
            context.primary_profile_record,
            tmp_path,
        )


def test_preflight_rehashes_global_audit_source_records(tmp_path: Path) -> None:
    files = _build_preflight_bundle(tmp_path)
    source = tmp_path / "sources" / "artifact.dat"
    source.write_bytes(b"tampered after global audit\n")
    with pytest.raises(audit.PreflightError, match="SHA-256 mismatch"):
        audit.preflight(
            protocol_path=files["protocol"],
            audit_path=files["audit"],
            ledger_path=files["ledger"],
            receipt_path=files["receipt"],
            repo_root=tmp_path,
        )


def _patch_production_root(
    monkeypatch: pytest.MonkeyPatch,
    root: Path,
    audit_path: Path,
) -> None:
    monkeypatch.setattr(audit, "REPO_ROOT", root.resolve())
    monkeypatch.setattr(
        audit,
        "_CANONICAL_GLOBAL_AUDIT",
        audit_path.relative_to(root).as_posix(),
    )
    monkeypatch.setattr(
        audit,
        "_CANONICAL_GLOBAL_AUDIT_SHA256",
        gate.sha256_file(audit_path),
    )


def test_cli_preflight_hash_drift_writes_blocked_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    _patch_production_root(monkeypatch, tmp_path, files["audit"])
    (tmp_path / "sources" / "artifact.dat").write_bytes(b"source drift\n")
    exit_code = audit.main(
        [
            "--protocol",
            str(files["protocol"]),
            "--audit",
            str(files["audit"]),
            "--ledger",
            str(files["ledger"]),
            "--receipt",
            str(files["receipt"]),
        ]
    )
    assert exit_code == 2
    failure_path = files["receipt"].parent / "baseline_forensics" / "failure.json"
    failure = gate.load_json_strict(failure_path)
    assert failure["status"] == "BLOCKED"
    assert failure["phase"] == "preflight"
    assert failure["protocol_sha256"] == gate.sha256_file(files["protocol"])
    assert failure["source_hashes_match"] is False
    assert failure["last_data_access"]["semantic_access_started"] is False


def test_cli_raw_production_route_reaches_actual_profile_loader(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    _patch_production_root(monkeypatch, tmp_path, files["audit"])

    calls: list[tuple[Path, tuple[str, ...]]] = []

    def profile_loader_sentinel(
        path: str | Path,
        *,
        required_sides: tuple[str, ...],
    ) -> Any:
        calls.append((Path(path).resolve(), required_sides))
        raise RuntimeError("profile loader sentinel")

    monkeypatch.setattr(
        audit,
        "_raw_adapter_dependencies",
        lambda: {
            "load_online_grf_profile": profile_loader_sentinel,
            "read_setup_xml": lambda *_args, **_kwargs: None,
            "calculate_wrench": lambda *_args, **_kwargs: None,
            "external_wrench": lambda *_args, **_kwargs: None,
            "sample_spheres": lambda *_args, **_kwargs: None,
            "sample_spheres_from_coordinate_states": (
                lambda *_args, **_kwargs: None
            ),
            "runtime_grid_indices": lambda *_args, **_kwargs: None,
        },
    )
    exit_code = audit.main(
        [
            "--protocol",
            str(files["protocol"]),
            "--audit",
            str(files["audit"]),
            "--ledger",
            str(files["ledger"]),
            "--receipt",
            str(files["receipt"]),
            "--execute-offline",
        ]
    )
    assert exit_code == 2
    assert calls == [(files["profile"].resolve(), ("left",))]
    failure = gate.load_json_strict(
        files["receipt"].parent / "baseline_forensics" / "failure.json"
    )
    assert failure["status"] == "ERROR"
    assert failure["last_data_access"]["semantic_access_started"] is True


def test_reproduction_refuses_a_requested_unit_with_missing_metric() -> None:
    trace = _synthetic_trace()
    analysis = audit.analyze_unit(trace)
    analysis["primary_metrics"]["left"]["primary_hs_delay_median_s"] = None
    protocol = {
        "baseline_forensics": {
            "reproduction_targets": [
                {
                    "target_id": "ik_missing",
                    "input_mode": "ik_prescribed",
                    "side": "left",
                    "metric": "primary_hs_delay_median_s",
                    "aggregation": "median",
                    "expected_s": 0.02,
                    "absolute_tolerance_s": 0.01,
                    "evidence_role": "development",
                    "unit_ids": [trace.unit_id],
                },
                {
                    "target_id": "h0_placeholder",
                    "input_mode": "h0_historical",
                    "side": "left",
                    "metric": "primary_hs_delay_median_s",
                    "aggregation": "median",
                    "expected_s": 0.17,
                    "absolute_tolerance_s": 0.01,
                    "evidence_role": "diagnostic",
                    "unit_ids": ["missing_h0"],
                },
            ]
        }
    }
    result = audit.evaluate_reproduction({trace.unit_id: analysis}, protocol)
    assert result["status"] == "ERROR"
    assert result["targets"][0]["missing_metric_unit_ids"] == [trace.unit_id]


@pytest.mark.parametrize(
    "mode",
    ["ik_prescribed", "coordinate_states", "h0_historical"],
)
def test_injected_offline_builder_supports_all_input_modes(
    tmp_path: Path,
    mode: str,
) -> None:
    trace = _synthetic_trace(
        input_mode=mode,
        evidence_role="diagnostic" if mode != "ik_prescribed" else "development",
    )
    unit = {
        "unit_id": trace.unit_id,
        "trial_id": trace.trial_id,
        "plateau_id": trace.plateau_id,
        "cadence_s": trace.cadence_s,
        "input_mode": trace.input_mode,
        "evidence_role": trace.evidence_role,
        "start_s": trace.start_s,
        "end_s": trace.end_s,
        "reference_unit_id": trace.reference_unit_id,
        "plateau_speed_mps": trace.plateau_speed_mps,
        "surface_velocity_mps": (
            list(trace.surface_velocity_mps)
            if trace.surface_velocity_mps is not None
            else None
        ),
    }
    observed = audit.load_unit_trace(
        unit,
        tmp_path,
        trace_builder=lambda _unit, _root: trace,
    )
    assert observed is trace


def test_no_clobber_byte_writer_preserves_existing_file(tmp_path: Path) -> None:
    destination = tmp_path / "summary.json"
    destination.write_bytes(b"original")
    with pytest.raises(gate.NoClobberError):
        audit._write_bytes_no_clobber(destination, b"replacement")
    assert destination.read_bytes() == b"original"


def test_default_trace_builder_invokes_raw_artifact_adapter(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=tmp_path,
    )
    calls: list[dict[str, Any]] = []
    sentinel = _synthetic_trace()

    def fake_adapter(
        unit: dict[str, Any],
        repo_root: Path,
        **kwargs: Any,
    ) -> audit.UnitTrace:
        calls.append(
            {
                "unit": unit,
                "repo_root": repo_root,
                **kwargs,
            }
        )
        return sentinel

    monkeypatch.setattr(audit, "build_offline_artifact_unit", fake_adapter)
    builder = audit.default_trace_builder(context)
    result = builder(context.authorized_units[0], context.repo_root)
    assert result is sentinel
    assert calls[0]["primary_profile_path"] == context.primary_profile_path
    assert context.repo_root / "sources" / "artifact.dat" in calls[0][
        "bound_source_paths"
    ]


def test_hash_bound_plugin_file_derives_platform_loader_token() -> None:
    posix = audit._plugin_loader_token(
        Path("/repo/plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"),
        platform_name="posix",
    )
    windows = audit._plugin_loader_token(
        Path("C:/repo/plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll"),
        platform_name="nt",
    )
    assert posix.endswith("/plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff")
    assert windows.endswith("SEA_Plugin_BlackBox_mCMC_impedence_ff")
    assert not posix.endswith(".dylib")
    assert not windows.endswith(".dll")


def test_chunked_convex_mesh_support_matches_brute_force() -> None:
    generator = np.random.default_rng(20260724)
    vertices = generator.normal(size=(300, 3))
    vertices = np.vstack((vertices, vertices[:20]))
    directions = generator.normal(size=(37, 3))
    expected = np.min(directions @ vertices.T, axis=1)
    observed = audit._convex_mesh_min_support(
        vertices,
        directions,
        chunk_size=5,
    )
    np.testing.assert_allclose(observed, expected, rtol=0.0, atol=1e-12)


def test_hash_bound_mesh_identity_accepts_case_alias_only(
    tmp_path: Path,
) -> None:
    canonical = tmp_path / "AM_foot_l.STL"
    canonical.write_bytes(b"hash-bound mesh")
    case_variant = tmp_path / "AM_foot_l.stl"
    if not case_variant.exists():
        os.link(canonical, case_variant)

    bound = frozenset({canonical.resolve()})
    assert audit._hash_bound_samefile(case_variant.resolve(), bound) == (
        canonical.resolve()
    )
    loaded_paths: list[Path] = []
    sentinel = object()
    loaded = audit._load_hash_bound_mesh_triangles(
        case_variant.resolve(),
        bound,
        lambda path: loaded_paths.append(path) or sentinel,
    )
    assert loaded is sentinel
    assert loaded_paths == [canonical.resolve()]

    different = tmp_path / "different" / "AM_foot_l.stl"
    different.parent.mkdir()
    different.write_bytes(b"different mesh")
    assert audit._hash_bound_samefile(different.resolve(), bound) is None
    with pytest.raises(audit.TraceContractError, match="not hash-bound"):
        audit._load_hash_bound_mesh_triangles(
            different.resolve(),
            bound,
            lambda path: loaded_paths.append(path),
        )
    assert loaded_paths == [canonical.resolve()]


def test_per_sphere_rows_are_cycle_bounded_and_dominance_is_deterministic() -> None:
    trace = _two_cycle_trace()
    equal = replace(trace.spheres[0], name="left_basis_01")
    trace = replace(trace, spheres=(trace.spheres[0], equal))
    rows = list(audit.per_sphere_rows([trace]))
    assert len(rows) == 2 * 2
    assert all(tuple(row) == audit._PER_SPHERE_CSV_FIELDS for row in rows)
    assert {
        (row["cycle_index"], row["sphere_name"])
        for row in rows
    } == {
        (0, "left_basis_00"),
        (0, "left_basis_01"),
        (1, "left_basis_00"),
        (1, "left_basis_01"),
    }
    for cycle_index in (0, 1):
        cycle_rows = [
            row for row in rows if row["cycle_index"] == cycle_index
        ]
        assert sum(row["is_dominant"] for row in cycle_rows) == 1
        assert {row["dominant_sphere_name"] for row in cycle_rows} == {
            "left_basis_00"
        }
        assert [
            row["dominance_fraction"] for row in cycle_rows
        ] == pytest.approx([0.5, 0.5])


def test_per_sphere_rows_use_diagnostic_whole_window_fallback() -> None:
    trace = _synthetic_trace()
    rows = list(audit.per_sphere_rows([trace]))
    assert len(rows) == len(trace.spheres)
    assert all(
        row["scope_kind"] == "WHOLE_WINDOW_NO_COMPLETE_ORACLE_CYCLE"
        and row["diagnostic_only"] is True
        and row["excluded_from_gating"] is True
        and row["cycle_index"] is None
        for row in rows
    )


def test_streaming_csv_consumes_one_shot_iterator_once(tmp_path: Path) -> None:
    class OneShot:
        def __init__(self) -> None:
            self.iterations = 0

        def __iter__(self):
            self.iterations += 1
            if self.iterations != 1:
                raise AssertionError("iterator consumed more than once")
            yield {"index": 1, "value": 2.5}
            yield {"index": 2, "value": 3.5}

    rows = OneShot()
    path, count = audit._stage_streaming_csv(
        tmp_path,
        "rows.csv",
        rows,
        fields=("index", "value"),
        label="rows.csv",
    )
    assert count == 2
    assert rows.iterations == 1
    assert path.read_text(encoding="utf-8").splitlines() == [
        "index,value",
        "1,2.5",
        "2,3.5",
    ]


@pytest.mark.parametrize(
    ("factory", "message"),
    (
        (lambda: iter(()), "at least one row"),
        (lambda: iter(({"index": 1},)), "fixed column order"),
        (
            lambda: iter(({"index": 1, "value": 2.0, "extra": 3},)),
            "fixed column order",
        ),
        (
            lambda: iter(({"value": 2.0, "index": 1},)),
            "fixed column order",
        ),
        (
            lambda: iter(({"index": 1, "value": float("nan")},)),
            "non-finite",
        ),
        (
            lambda: iter(({"index": 1, "value": float("inf")},)),
            "non-finite",
        ),
    ),
)
def test_streaming_csv_rejects_invalid_rows_and_cleans_partial_file(
    tmp_path: Path,
    factory: Any,
    message: str,
) -> None:
    path = tmp_path / "invalid.csv"
    with pytest.raises(audit.TraceContractError, match=message):
        audit._stage_streaming_csv(
            tmp_path,
            path.name,
            factory(),
            fields=("index", "value"),
            label=path.name,
        )
    assert not path.exists()


def test_streaming_csv_cleans_up_after_generator_exception(
    tmp_path: Path,
) -> None:
    def broken():
        yield {"index": 1, "value": 2.0}
        raise RuntimeError("generator sentinel")

    path = tmp_path / "broken.csv"
    with pytest.raises(RuntimeError, match="generator sentinel"):
        audit._stage_streaming_csv(
            tmp_path,
            path.name,
            broken(),
            fields=("index", "value"),
            label=path.name,
        )
    assert not path.exists()


def test_streaming_csv_occupied_path_is_preserved_without_consumption(
    tmp_path: Path,
) -> None:
    path = tmp_path / "occupied.csv"
    path.write_text("original\n", encoding="utf-8")
    consumed = False

    def rows():
        nonlocal consumed
        consumed = True
        yield {"index": 1}

    with pytest.raises(FileExistsError):
        audit._stage_streaming_csv(
            tmp_path,
            path.name,
            rows(),
            fields=("index",),
            label=path.name,
        )
    assert consumed is False
    assert path.read_text(encoding="utf-8") == "original\n"


def test_streaming_csv_has_bounded_memory_for_250k_rows(
    tmp_path: Path,
) -> None:
    import tracemalloc

    def rows():
        for index in range(250_000):
            yield {"index": index, "value": index * 0.5}

    tracemalloc.start()
    try:
        _path, count = audit._stage_streaming_csv(
            tmp_path,
            "large.csv",
            rows(),
            fields=("index", "value"),
            label="large.csv",
        )
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert count == 250_000
    assert peak < 8 * 1024 * 1024


def _fake_plots(
    _traces: Any,
    _analyses: Any,
) -> dict[str, bytes]:
    png = b"\x89PNG\r\n\x1a\nsynthetic"
    return {
        "timing_plot.png": png,
        "penetration_plot.png": png,
    }


def test_publish_streams_all_csv_and_records_exact_row_counts(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)
    assert not hasattr(audit, "_csv_bytes")
    original_commit = audit._atomic_publish_directory_no_clobber
    committed_sets: list[set[str]] = []

    def record_commit(staging: Path, destination: Path) -> None:
        committed_sets.append({path.name for path in staging.iterdir()})
        original_commit(staging, destination)

    monkeypatch.setattr(
        audit,
        "_atomic_publish_directory_no_clobber",
        record_commit,
    )
    output = audit.publish_forensics(
        context,
        traces=traces,
        analyses=analyses,
        root_cause={"status": "PASS"},
        reproduction={"status": "PASS"},
        loaded_profile_record=context.primary_profile_record,
        plot_renderer=_fake_plots,
    )
    assert {path.name for path in output.iterdir()} == set(
        audit.OUTPUT_FILENAMES
    )
    assert committed_sets == [set(audit.OUTPUT_FILENAMES)]
    summary = gate.load_json_strict(output / "summary.json")
    for name, fields in (
        ("trace.csv", audit._TRACE_CSV_FIELDS),
        ("events.csv", audit._EVENT_CSV_FIELDS),
        ("per_sphere.csv", audit._PER_SPHERE_CSV_FIELDS),
    ):
        with (output / name).open(
            "r",
            encoding="utf-8",
            newline="",
        ) as stream:
            rows = list(__import__("csv").DictReader(stream))
        assert tuple(rows[0]) == fields
        assert len(rows) == summary["forensics_row_counts"][name]
    expected_sphere_rows = sum(
        len(trace.spheres)
        * max(
            1,
            len(
                audit._metric_event_bundle(
                    trace,
                    trace.sides["left"].oracle_force_n[:, 1],
                    20.0,
                )["complete_cycles"]
            ),
        )
        for trace in traces
    )
    assert summary["forensics_row_counts"]["per_sphere.csv"] == (
        expected_sphere_rows
    )
    assert not list(
        output.parent.glob(f".{output.name}.stage-*")
    )


def test_publish_occupied_output_does_not_start_row_generators(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    files = _build_preflight_bundle(tmp_path)
    context = audit.preflight(
        protocol_path=files["protocol"],
        audit_path=files["audit"],
        ledger_path=files["ledger"],
        receipt_path=files["receipt"],
        repo_root=tmp_path,
    )
    context.output_dir.mkdir()
    started = False

    def forbidden_rows(_traces: Any):
        nonlocal started
        started = True
        yield {}

    monkeypatch.setattr(audit, "trace_rows", forbidden_rows)
    with pytest.raises(gate.NoClobberError, match="occupied"):
        audit.publish_forensics(
            context,
            traces=(),
            analyses={},
            root_cause={},
            reproduction={},
            loaded_profile_record={},
            plot_renderer=_fake_plots,
        )
    assert started is False


def test_post_staging_source_drift_cleans_stage_before_publication(
    tmp_path: Path,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)

    def drift_after_csv_stage(_traces: Any, _analyses: Any):
        context.primary_profile_path.write_text(
            '{"drift": true}\n',
            encoding="utf-8",
        )
        return _fake_plots(_traces, _analyses)

    with pytest.raises(
        audit.PreflightError,
        match="SHA-256 mismatch|size mismatch|differs",
    ):
        audit.publish_forensics(
            context,
            traces=traces,
            analyses=analyses,
            root_cause={"status": "PASS"},
            reproduction={"status": "PASS"},
            loaded_profile_record=context.primary_profile_record,
            plot_renderer=drift_after_csv_stage,
        )
    assert not context.output_dir.exists()
    assert not list(
        context.output_dir.parent.glob(
            f".{context.output_dir.name}.stage-*"
        )
    )


def test_atomic_commit_failure_exposes_no_scientific_artifact(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)

    def fail_commit(_staging: Path, _destination: Path) -> None:
        raise OSError("injected atomic-commit failure")

    monkeypatch.setattr(
        audit,
        "_atomic_publish_directory_no_clobber",
        fail_commit,
    )
    with pytest.raises(OSError, match="atomic-commit"):
        audit.publish_forensics(
            context,
            traces=traces,
            analyses=analyses,
            root_cause={"status": "PASS"},
            reproduction={"status": "PASS"},
            loaded_profile_record=context.primary_profile_record,
            plot_renderer=_fake_plots,
        )
    assert not context.output_dir.exists()
    assert not list(
        context.output_dir.parent.glob(
            f".{context.output_dir.name}.stage-*"
        )
    )


def test_atomic_commit_race_preserves_competing_directory_unchanged(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)
    original = audit._atomic_publish_directory_no_clobber
    competing_name = "foreign.txt"

    def inject_race(staging: Path, destination: Path) -> None:
        destination.mkdir()
        (destination / competing_name).write_bytes(
            b"competing invocation\n"
        )
        original(staging, destination)

    monkeypatch.setattr(
        audit,
        "_atomic_publish_directory_no_clobber",
        inject_race,
    )
    with pytest.raises(gate.NoClobberError, match="occupied"):
        audit.publish_forensics(
            context,
            traces=traces,
            analyses=analyses,
            root_cause={"status": "PASS"},
            reproduction={"status": "PASS"},
            loaded_profile_record=context.primary_profile_record,
            plot_renderer=_fake_plots,
        )
    assert (context.output_dir / competing_name).read_bytes() == (
        b"competing invocation\n"
    )
    assert {path.name for path in context.output_dir.iterdir()} == {
        competing_name
    }
    assert not list(
        context.output_dir.parent.glob(
            f".{context.output_dir.name}.stage-*"
        )
    )


def test_successful_commit_never_cleans_foreign_recreated_stage_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)
    original = audit._atomic_publish_directory_no_clobber
    recreated: list[Path] = []

    def commit_then_recreate(staging: Path, destination: Path) -> None:
        original(staging, destination)
        staging.mkdir()
        foreign = staging / "foreign.txt"
        foreign.write_bytes(b"foreign replacement\n")
        recreated.append(foreign)

    monkeypatch.setattr(
        audit,
        "_atomic_publish_directory_no_clobber",
        commit_then_recreate,
    )
    audit.publish_forensics(
        context,
        traces=traces,
        analyses=analyses,
        root_cause={"status": "PASS"},
        reproduction={"status": "PASS"},
        loaded_profile_record=context.primary_profile_record,
        plot_renderer=_fake_plots,
    )
    assert len(recreated) == 1
    assert recreated[0].read_bytes() == b"foreign replacement\n"
    recreated[0].unlink()
    recreated[0].parent.rmdir()


def test_post_commit_fsync_failure_writes_authoritative_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    context, traces, analyses = _publication_fixture(tmp_path)
    original = audit._fsync_directory

    def fail_output_fsync(path: Path) -> None:
        if path == context.output_dir:
            raise OSError("injected post-commit fsync failure")
        original(path)

    monkeypatch.setattr(audit, "_fsync_directory", fail_output_fsync)
    with pytest.raises(OSError, match="post-commit fsync"):
        audit.publish_forensics(
            context,
            traces=traces,
            analyses=analyses,
            root_cause={"status": "PASS"},
            reproduction={"status": "PASS"},
            loaded_profile_record=context.primary_profile_record,
            plot_renderer=_fake_plots,
        )
    assert all(
        (context.output_dir / name).is_file()
        for name in audit.OUTPUT_FILENAMES
    )
    failure = gate.load_json_strict(context.output_dir / "failure.json")
    assert failure["status"] == "ERROR"
    assert failure["phase"] == "post_commit_durability"
    assert len(failure["artifacts"]) == len(audit.OUTPUT_FILENAMES)
