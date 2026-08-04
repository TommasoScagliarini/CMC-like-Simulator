"""Fail-closed A/B/C robustness harness for the prosthetic event detector.

The manifest fixes two frozen actors, three starts, and development seeds
123--125.  Held-out seeds 126--128 are forbidden by both the manifest parser
and the generated-case audit.  The three detector roles are:

* ``legacy``: aggregate detector events drive the existing phase FSM;
* ``shadow``: legacy events still drive the FSM while detailed sensor output is
  recorded for an offline two-sensor replay;
* ``two_sensor``: heel/toe signals drive the same phase FSM.

``--list-cases`` and ``--dry-run`` never create output directories or launch
OpenSim.  ``--run`` is deliberately no-clobber and refuses to start unless the
rollout CLI exposes the manifest-pinned phase-FSM input-mode contract.  After a
complete run, every summary is classified with ``robust_ppo_gate`` and the two
non-reference modes are compared with legacy in the exact same actor/start/
selection/seed cell.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import robust_ppo_gate as robust


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = Path(__file__).with_name(
    "heel_detector_robustness_ab_protocol.json"
)
FORBIDDEN_HELDOUT_SEEDS = (126, 127, 128)
REQUIRED_DEVELOPMENT_SEEDS = (123, 124, 125)
REQUIRED_MODE_NAMES = ("legacy", "shadow", "two_sensor")
REQUIRED_CHECKPOINT_NAMES = ("h0", "pilot50_best")
REQUIRED_START_NAMES = ("minus020", "nominal", "plus020")
REPORT_FILENAME = "heel_detector_robustness_ab.json"


class ProtocolError(ValueError):
    """Raised when the preregistered development contract is not satisfied."""


@dataclass(frozen=True)
class MatrixCase:
    checkpoint_name: str
    checkpoint: Path
    mode_name: str
    phase_fsm_input_mode: str
    record_outputs: bool
    record_policy_trace: bool
    shadow_replay: bool
    start_name: str
    offset_s: float
    action_selection: str
    seed: int

    @property
    def cell_key(self) -> tuple[str, str, int, str]:
        return (
            self.checkpoint_name,
            self.action_selection,
            self.seed,
            self.start_name,
        )

    @property
    def case_id(self) -> str:
        action = f"{self.action_selection}_seed{self.seed}"
        return "/".join(
            (self.checkpoint_name, self.mode_name, action, self.start_name)
        )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ProtocolError(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ProtocolError(f"non-finite JSON number in {label}: {token}")
            ),
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise ProtocolError(f"could not read valid {label}: {path}: {exc}") from exc
    _require(isinstance(value, Mapping), f"{label} is not a JSON object: {path}")
    return dict(value)


def _mapping(value: Any, label: str) -> dict[str, Any]:
    _require(isinstance(value, Mapping), f"{label} must be an object")
    return dict(value)


def _sequence(value: Any, label: str) -> list[Any]:
    _require(
        isinstance(value, Sequence) and not isinstance(value, (str, bytes)),
        f"{label} must be an array",
    )
    return list(value)


def _integer(value: Any, label: str) -> int:
    _require(type(value) is int, f"{label} must be an integer")
    return int(value)


def _finite(value: Any, label: str) -> float:
    _require(
        not isinstance(value, bool) and isinstance(value, (int, float)),
        f"{label} must be numeric",
    )
    converted = float(value)
    _require(math.isfinite(converted), f"{label} must be finite")
    return converted


def _boolean(value: Any, label: str) -> bool:
    _require(type(value) is bool, f"{label} must be boolean")
    return bool(value)


def _text(value: Any, label: str) -> str:
    _require(isinstance(value, str) and bool(value.strip()), f"{label} must be text")
    return value.strip()


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _resolve_root_path(value: Any, label: str) -> Path:
    text = _text(value, label)
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = ROOT_DIR / path
    return path.resolve(strict=False)


def load_and_validate_manifest(path: Path = DEFAULT_MANIFEST) -> dict[str, Any]:
    """Read the manifest and enforce its non-negotiable development contract."""

    manifest_path = path.expanduser().resolve(strict=False)
    data = _read_json_object(manifest_path, "detector A/B manifest")
    _require(data.get("schema_version") == 1, "unsupported manifest schema")
    _require(
        data.get("status") == "preregistered_development_only",
        "manifest is not preregistered development-only",
    )

    development = tuple(
        _integer(seed, "development seed")
        for seed in _sequence(data.get("development_seeds"), "development_seeds")
    )
    _require(
        development == REQUIRED_DEVELOPMENT_SEEDS,
        f"development seeds must be exactly {REQUIRED_DEVELOPMENT_SEEDS}",
    )
    heldout = _mapping(data.get("heldout_contract"), "heldout_contract")
    _require(
        heldout.get("status") == "sealed_not_read_or_run",
        "held-out contract is not sealed",
    )
    forbidden = tuple(
        _integer(seed, "forbidden held-out seed")
        for seed in _sequence(heldout.get("forbidden_seeds"), "forbidden_seeds")
    )
    _require(
        forbidden == FORBIDDEN_HELDOUT_SEEDS,
        f"held-out seeds must remain exactly {FORBIDDEN_HELDOUT_SEEDS}",
    )
    _require(
        set(development).isdisjoint(forbidden),
        "development and held-out seed sets overlap",
    )

    starts = _sequence(data.get("start_conditions"), "start_conditions")
    _require(len(starts) == 3, "exactly three start conditions are required")
    start_names: list[str] = []
    start_offsets: list[float] = []
    for index, raw in enumerate(starts):
        item = _mapping(raw, f"start_conditions[{index}]")
        start_names.append(_text(item.get("name"), f"start name {index}"))
        start_offsets.append(_finite(item.get("offset_s"), f"start offset {index}"))
    _require(
        tuple(start_names) == REQUIRED_START_NAMES,
        f"start names must be exactly {REQUIRED_START_NAMES}",
    )
    _require(
        all(
            math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12)
            for actual, expected in zip(start_offsets, robust.DEFAULT_START_OFFSETS_S)
        ),
        "start offsets differ from robust_ppo_gate.DEFAULT_START_OFFSETS_S",
    )

    actions = _mapping(data.get("action_matrix"), "action_matrix")
    deterministic_seed = _integer(
        actions.get("deterministic_seed"), "deterministic_seed"
    )
    stochastic_seeds = tuple(
        _integer(seed, "stochastic seed")
        for seed in _sequence(actions.get("stochastic_seeds"), "stochastic_seeds")
    )
    sigma = _finite(actions.get("expected_sigma"), "expected_sigma")
    _require(deterministic_seed == 123, "deterministic seed must be 123")
    _require(
        stochastic_seeds == development,
        "stochastic seeds must equal the development seed set",
    )
    _require(
        math.isclose(sigma, 0.005, rel_tol=0.0, abs_tol=1.0e-12),
        "expected exploration sigma must remain 0.005",
    )

    modes = _sequence(data.get("modes"), "modes")
    _require(len(modes) == 3, "exactly three detector modes are required")
    normalized_modes: list[dict[str, Any]] = []
    for index, raw in enumerate(modes):
        item = _mapping(raw, f"modes[{index}]")
        normalized_modes.append(
            {
                "name": _text(item.get("name"), f"mode name {index}"),
                "phase_fsm_input_mode": _text(
                    item.get("phase_fsm_input_mode"),
                    f"phase_fsm_input_mode {index}",
                ),
                "comparison_reference": _text(
                    item.get("comparison_reference"),
                    f"comparison_reference {index}",
                ),
                "shadow_replay": _boolean(
                    item.get("shadow_replay"), f"shadow_replay {index}"
                ),
                "record_outputs": _boolean(
                    item.get("record_outputs"), f"record_outputs {index}"
                ),
                "record_policy_trace": _boolean(
                    item.get("record_policy_trace"),
                    f"record_policy_trace {index}",
                ),
            }
        )
    _require(
        tuple(item["name"] for item in normalized_modes) == REQUIRED_MODE_NAMES,
        f"mode names must be exactly {REQUIRED_MODE_NAMES}",
    )
    mode_map = {item["name"]: item for item in normalized_modes}
    _require(
        mode_map["legacy"]["phase_fsm_input_mode"] == "legacy_events",
        "legacy must use legacy_events",
    )
    _require(
        mode_map["shadow"]["phase_fsm_input_mode"] == "shadow"
        and mode_map["shadow"]["shadow_replay"] is True
        and mode_map["shadow"]["record_outputs"] is True
        and mode_map["shadow"]["record_policy_trace"] is True,
        "shadow must keep legacy active and record outputs plus policy trace",
    )
    _require(
        mode_map["two_sensor"]["phase_fsm_input_mode"] == "two_sensor",
        "two_sensor must use the two_sensor FSM input contract",
    )
    _require(
        all(item["comparison_reference"] == "legacy" for item in normalized_modes),
        "all modes must use paired legacy as their reference",
    )

    checkpoints = _sequence(data.get("checkpoints"), "checkpoints")
    _require(len(checkpoints) == 2, "exactly two frozen checkpoints are required")
    normalized_checkpoints: list[dict[str, Any]] = []
    for index, raw in enumerate(checkpoints):
        item = _mapping(raw, f"checkpoints[{index}]")
        actor_digest = item.get("actor_digest")
        state_digest = item.get("module_state_sha256")
        _require(_is_sha256(actor_digest), f"invalid actor digest at checkpoint {index}")
        _require(
            _is_sha256(state_digest),
            f"invalid module_state SHA-256 at checkpoint {index}",
        )
        normalized_checkpoints.append(
            {
                "name": _text(item.get("name"), f"checkpoint name {index}"),
                "path": _resolve_root_path(item.get("path"), f"checkpoint path {index}"),
                "logical_iteration": _integer(
                    item.get("logical_iteration"), f"logical iteration {index}"
                ),
                "actor_digest": actor_digest,
                "module_state_sha256": state_digest,
            }
        )
    _require(
        tuple(item["name"] for item in normalized_checkpoints)
        == REQUIRED_CHECKPOINT_NAMES,
        f"checkpoint names must be exactly {REQUIRED_CHECKPOINT_NAMES}",
    )

    rollout = _mapping(data.get("rollout"), "rollout")
    rollout_script = _resolve_root_path(rollout.get("script"), "rollout script")
    cli_flag = _text(
        rollout.get("phase_fsm_input_mode_cli_flag"),
        "phase FSM input-mode CLI flag",
    )
    summary_field = _text(
        rollout.get("phase_fsm_input_mode_summary_field"),
        "phase FSM input-mode summary field",
    )
    _require(cli_flag == "--phase-fsm-input-mode", "unexpected FSM mode CLI flag")
    _require(summary_field == "phase_fsm_input_mode", "unexpected FSM summary field")
    sensor_on = _finite(
        rollout.get("phase_sensor_on_threshold_n"),
        "phase sensor ON threshold",
    )
    sensor_off = _finite(
        rollout.get("phase_sensor_off_threshold_n"),
        "phase sensor OFF threshold",
    )
    sensor_dwell = _finite(
        rollout.get("phase_sensor_dwell_s"),
        "phase sensor dwell",
    )
    _require(sensor_on > 0.0, "phase sensor ON threshold must be positive")
    _require(
        0.0 <= sensor_off < sensor_on,
        "phase sensor OFF threshold must be below ON",
    )
    _require(sensor_dwell >= 0.0, "phase sensor dwell must be non-negative")
    segment_duration = _finite(
        rollout.get("segment_duration_s"),
        "segment duration",
    )
    _require(
        math.isclose(segment_duration, 0.01, rel_tol=0.0, abs_tol=0.0),
        "segment duration must remain 0.01 s",
    )
    _require(
        sensor_dwell <= 0.0 or segment_duration <= sensor_dwell,
        "segment duration must not exceed sensor dwell",
    )
    _require(
        math.isclose(
            _finite(rollout.get("episode_duration_s"), "episode duration"),
            5.0,
            rel_tol=0.0,
            abs_tol=0.0,
        ),
        "episode duration must remain 5.0 s",
    )
    _require(
        _integer(rollout.get("max_steps"), "max_steps") == robust.EXPECTED_STEPS,
        f"max_steps must remain {robust.EXPECTED_STEPS}",
    )
    _require(rollout.get("action_mode") == "absolute", "action mode must be absolute")
    _require(
        _finite(rollout.get("run_timeout_s"), "run timeout") > 0.0,
        "run timeout must be positive",
    )

    gate = _mapping(data.get("gate"), "gate")
    _require(
        _integer(gate.get("minimum_valid_cycles"), "minimum valid cycles")
        == robust.EXPECTED_MIN_CYCLES,
        "minimum valid-cycle gate differs from robust_ppo_gate",
    )
    _require(
        math.isclose(
            _finite(gate.get("penetration_limit_m_strict"), "penetration limit"),
            robust.PENETRATION_LIMIT_M,
            rel_tol=0.0,
            abs_tol=0.0,
        ),
        "penetration gate differs from robust_ppo_gate",
    )
    _require(
        _integer(gate.get("maximum_action_clipped_steps"), "maximum clipping")
        == 0,
        "action clipping gate must remain zero",
    )
    _require(
        _integer(gate.get("expected_actor_features"), "actor feature count")
        == robust.EXPECTED_ACTOR_FEATURES,
        "actor feature count differs from robust_ppo_gate",
    )
    _require(
        _integer(gate.get("expected_observation_features"), "observation feature count")
        == robust.EXPECTED_OBSERVATION_FEATURES,
        "observation feature count differs from robust_ppo_gate",
    )
    _require(
        gate.get("reserve_reference")
        == "paired_legacy_same_checkpoint_start_selection_seed",
        "reserve reference is not exact-cell paired legacy",
    )
    _require(
        gate.get("reserve_tolerance_formula")
        == "max(1e-6 Nm, 1e-9 * legacy Nm)",
        "reserve numerical tolerance formula changed",
    )
    _require(
        gate.get("invalid_event_non_regression") is True,
        "invalid-event non-regression must remain enabled",
    )
    _finite(gate.get("shadow_float_abs_tolerance"), "shadow float tolerance")
    _sequence(gate.get("shadow_exact_fields"), "shadow_exact_fields")
    _sequence(gate.get("shadow_float_fields"), "shadow_float_fields")

    historical = _sequence(data.get("historical_artifacts"), "historical_artifacts")
    _require(len(historical) >= 5, "historical reference artifacts are incomplete")
    normalized_historical: list[dict[str, Any]] = []
    for index, raw in enumerate(historical):
        item = _mapping(raw, f"historical_artifacts[{index}]")
        digest = item.get("sha256")
        _require(_is_sha256(digest), f"invalid historical SHA-256 at index {index}")
        normalized_historical.append(
            {
                "role": _text(item.get("role"), f"historical role {index}"),
                "path": _resolve_root_path(
                    item.get("path"), f"historical artifact path {index}"
                ),
                "sha256": digest,
            }
        )

    normalized = dict(data)
    normalized["_manifest_path"] = manifest_path
    normalized["_manifest_sha256"] = _sha256(manifest_path)
    normalized["_development_seeds"] = development
    normalized["_start_conditions"] = tuple(zip(start_names, start_offsets))
    normalized["_deterministic_seed"] = deterministic_seed
    normalized["_stochastic_seeds"] = stochastic_seeds
    normalized["_expected_sigma"] = sigma
    normalized["_modes"] = tuple(normalized_modes)
    normalized["_checkpoints"] = tuple(normalized_checkpoints)
    normalized["_rollout_script"] = rollout_script
    normalized["_cli_flag"] = cli_flag
    normalized["_summary_field"] = summary_field
    normalized["_sensor_on_threshold_n"] = sensor_on
    normalized["_sensor_off_threshold_n"] = sensor_off
    normalized["_sensor_dwell_s"] = sensor_dwell
    normalized["_historical"] = tuple(normalized_historical)
    normalized["_output_root"] = _resolve_root_path(
        data.get("output_root"), "output_root"
    )
    return normalized


def build_cases(manifest: Mapping[str, Any]) -> tuple[MatrixCase, ...]:
    """Build the immutable 72-cell development matrix."""

    cases: list[MatrixCase] = []
    for mode in manifest["_modes"]:
        for checkpoint in manifest["_checkpoints"]:
            for start_name, offset_s in manifest["_start_conditions"]:
                cases.append(
                    MatrixCase(
                        checkpoint_name=checkpoint["name"],
                        checkpoint=checkpoint["path"],
                        mode_name=mode["name"],
                        phase_fsm_input_mode=mode["phase_fsm_input_mode"],
                        record_outputs=mode["record_outputs"],
                        record_policy_trace=mode["record_policy_trace"],
                        shadow_replay=mode["shadow_replay"],
                        start_name=start_name,
                        offset_s=offset_s,
                        action_selection="deterministic",
                        seed=manifest["_deterministic_seed"],
                    )
                )
                for seed in manifest["_stochastic_seeds"]:
                    cases.append(
                        MatrixCase(
                            checkpoint_name=checkpoint["name"],
                            checkpoint=checkpoint["path"],
                            mode_name=mode["name"],
                            phase_fsm_input_mode=mode["phase_fsm_input_mode"],
                            record_outputs=mode["record_outputs"],
                            record_policy_trace=mode["record_policy_trace"],
                            shadow_replay=mode["shadow_replay"],
                            start_name=start_name,
                            offset_s=offset_s,
                            action_selection="stochastic",
                            seed=seed,
                        )
                    )
    _require(len(cases) == 72, f"expected 72 cases, built {len(cases)}")
    _require(
        len({case.case_id for case in cases}) == len(cases),
        "matrix contains duplicate case identifiers",
    )
    used_seeds = {case.seed for case in cases}
    _require(
        used_seeds.isdisjoint(FORBIDDEN_HELDOUT_SEEDS),
        "generated matrix contains a forbidden held-out seed",
    )
    _require(
        used_seeds == set(REQUIRED_DEVELOPMENT_SEEDS),
        "generated matrix does not use exactly the development seeds",
    )
    return tuple(cases)


def build_rollout_command(
    manifest: Mapping[str, Any],
    case: MatrixCase,
    case_dir: Path,
    *,
    python_executable: str = sys.executable,
) -> list[str]:
    rollout = manifest["rollout"]
    command = [
        python_executable,
        str(manifest["_rollout_script"]),
        "--checkpoint",
        str(case.checkpoint),
        "--output-dir",
        str(case_dir.resolve(strict=False)),
        "--segment-duration",
        repr(float(rollout["segment_duration_s"])),
        "--episode-duration",
        repr(float(rollout["episode_duration_s"])),
        "--max-steps",
        str(int(rollout["max_steps"])),
        "--episode-start-offset-s",
        repr(case.offset_s),
        "--action-mode",
        str(rollout["action_mode"]),
        "--action-selection",
        case.action_selection,
        "--seed",
        str(case.seed),
        "--run-timeout-s",
        repr(float(rollout["run_timeout_s"])),
        manifest["_cli_flag"],
        case.phase_fsm_input_mode,
        "--phase-sensor-on-threshold-n",
        repr(float(manifest["_sensor_on_threshold_n"])),
        "--phase-sensor-off-threshold-n",
        repr(float(manifest["_sensor_off_threshold_n"])),
        "--phase-sensor-dwell-s",
        repr(float(manifest["_sensor_dwell_s"])),
        "--record-outputs" if case.record_outputs else "--no-record-outputs",
        (
            "--record-policy-trace"
            if case.record_policy_trace
            else "--no-record-policy-trace"
        ),
        "--no-progress",
    ]
    return command


def _checkpoint_reports(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for checkpoint in manifest["_checkpoints"]:
        path = checkpoint["path"]
        failures: list[str] = []
        files: dict[str, Any] = {}
        if not path.is_dir():
            failures.append(f"checkpoint is not a directory: {path}")
        else:
            for filename in robust.REQUIRED_RL_MODULE_FILES:
                artifact = path / filename
                if not artifact.is_file():
                    failures.append(f"missing RLModule artifact: {artifact}")
                    continue
                digest = _sha256(artifact)
                files[filename] = {"path": str(artifact), "sha256": digest}
            module = files.get("module_state.pkl", {})
            if module.get("sha256") != checkpoint["module_state_sha256"]:
                failures.append("module_state.pkl SHA-256 differs from manifest")
        reports.append(
            {
                "name": checkpoint["name"],
                "path": str(path),
                "logical_iteration": checkpoint["logical_iteration"],
                "actor_digest": checkpoint["actor_digest"],
                "status": "PASS" if not failures else "FAIL",
                "failures": failures,
                "files": files,
            }
        )
    return reports


def _historical_reports(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for artifact in manifest["_historical"]:
        path = artifact["path"]
        actual = _sha256(path) if path.is_file() else None
        passed = actual == artifact["sha256"]
        reports.append(
            {
                "role": artifact["role"],
                "path": str(path),
                "expected_sha256": artifact["sha256"],
                "actual_sha256": actual,
                "status": "PASS" if passed else "FAIL",
            }
        )
    return reports


def _runtime_contract_report(manifest: Mapping[str, Any]) -> dict[str, Any]:
    script = manifest["_rollout_script"]
    try:
        source = script.read_text(encoding="utf-8")
    except OSError as exc:
        return {
            "status": "FAIL",
            "script": str(script),
            "failures": [f"could not read rollout script: {exc}"],
        }
    failures: list[str] = []
    if manifest["_cli_flag"] not in source:
        failures.append(f"rollout CLI does not expose {manifest['_cli_flag']}")
    if manifest["_summary_field"] not in source:
        failures.append(
            "rollout summary does not expose " f"{manifest['_summary_field']}"
        )
    return {
        "status": "PASS" if not failures else "FAIL",
        "script": str(script),
        "cli_flag": manifest["_cli_flag"],
        "summary_field": manifest["_summary_field"],
        "failures": failures,
    }


def dry_run_report(
    manifest: Mapping[str, Any],
    cases: Sequence[MatrixCase],
    output_root: Path,
) -> dict[str, Any]:
    checkpoints = _checkpoint_reports(manifest)
    historical = _historical_reports(manifest)
    runtime = _runtime_contract_report(manifest)
    output_collision = os.path.lexists(output_root)
    blockers: list[str] = []
    if any(item["status"] != "PASS" for item in checkpoints):
        blockers.append("checkpoint_artifact_contract")
    if any(item["status"] != "PASS" for item in historical):
        blockers.append("historical_artifact_contract")
    if runtime["status"] != "PASS":
        blockers.append("rollout_runtime_contract")
    if output_collision:
        blockers.append("output_root_exists")
    commands = [
        {
            "case_id": case.case_id,
            "command": build_rollout_command(
                manifest, case, output_root / case.case_id
            ),
        }
        for case in cases
    ]
    return {
        "ok": True,
        "status": (
            "DRY_RUN_READY" if not blockers else "DRY_RUN_VALIDATED_BUT_BLOCKED"
        ),
        "execution_ready": not blockers,
        "blocking_reasons": blockers,
        "manifest": {
            "path": str(manifest["_manifest_path"]),
            "sha256": manifest["_manifest_sha256"],
            "status": manifest["status"],
        },
        "heldout_contract": {
            "status": "SEALED",
            "forbidden_seeds": list(FORBIDDEN_HELDOUT_SEEDS),
            "heldout_cases_generated": 0,
        },
        "development_seeds": list(REQUIRED_DEVELOPMENT_SEEDS),
        "case_count": len(cases),
        "case_count_by_mode": {
            mode: sum(case.mode_name == mode for case in cases)
            for mode in REQUIRED_MODE_NAMES
        },
        "checkpoint_reports": checkpoints,
        "historical_artifact_reports": historical,
        "runtime_contract": runtime,
        "output_root": str(output_root),
        "output_root_exists": output_collision,
        "commands": commands,
    }


def _read_case_summary(case_dir: Path) -> dict[str, Any]:
    return _read_json_object(case_dir / "rollout_summary.json", "rollout summary")


def _finite_summary_value(summary: Mapping[str, Any], key: str) -> float:
    return _finite(summary.get(key), f"rollout summary field {key}")


def _classify_complete_matrix(
    manifest: Mapping[str, Any],
    cases: Sequence[MatrixCase],
    output_root: Path,
) -> dict[str, Any]:
    summaries: dict[str, dict[str, Any]] = {}
    for case in cases:
        summaries[case.case_id] = _read_case_summary(output_root / case.case_id)

    legacy_by_cell: dict[tuple[str, str, int, str], dict[str, Any]] = {}
    for case in cases:
        if case.mode_name == "legacy":
            _require(case.cell_key not in legacy_by_cell, "duplicate legacy cell")
            legacy_by_cell[case.cell_key] = summaries[case.case_id]
    _require(len(legacy_by_cell) == 24, "expected exactly 24 paired legacy cells")

    gate_cfg = manifest["gate"]
    expected_sigma = float(manifest["_expected_sigma"])
    results: list[dict[str, Any]] = []
    for case in cases:
        summary = summaries[case.case_id]
        legacy = legacy_by_cell.get(case.cell_key)
        _require(legacy is not None, f"missing paired legacy cell for {case.case_id}")
        legacy_reserve = _finite_summary_value(legacy, "reserve_norm_max_nm")
        tolerance = (
            0.0
            if case.mode_name == "legacy"
            else robust._reserve_numerical_tolerance_nm(legacy_reserve)
        )
        spec = robust.RolloutSpec(
            name=case.case_id,
            offset_s=case.offset_s,
            action_selection=case.action_selection,
            seed=case.seed,
        )
        classified = robust.classify_rollout_summary(
            summary,
            expected_checkpoint=case.checkpoint,
            spec=spec,
            expected_sigma=expected_sigma,
            max_reserve_norm_nm=legacy_reserve,
            reserve_numerical_tolerance_nm=tolerance,
        )
        extra_checks: list[dict[str, Any]] = []

        def extra(name: str, passed: bool, actual: Any, expected: Any) -> None:
            extra_checks.append(
                {
                    "name": name,
                    "status": "PASS" if passed else "FAIL",
                    "actual": actual,
                    "expected": expected,
                }
            )

        mode_actual = summary.get(manifest["_summary_field"])
        extra(
            "phase_fsm_input_mode",
            mode_actual == case.phase_fsm_input_mode,
            mode_actual,
            case.phase_fsm_input_mode,
        )
        extra(
            "record_outputs",
            summary.get("record_outputs") is case.record_outputs,
            summary.get("record_outputs"),
            case.record_outputs,
        )
        extra(
            "record_policy_trace",
            summary.get("record_policy_trace") is case.record_policy_trace,
            summary.get("record_policy_trace"),
            case.record_policy_trace,
        )
        legacy_invalid = _finite_summary_value(legacy, "invalid_event_count")
        candidate_invalid = _finite_summary_value(summary, "invalid_event_count")
        extra(
            "invalid_event_non_regression",
            candidate_invalid <= legacy_invalid,
            candidate_invalid,
            f"<= paired legacy {legacy_invalid}",
        )

        if case.mode_name == "shadow":
            for field in gate_cfg["shadow_exact_fields"]:
                extra(
                    f"shadow_exact.{field}",
                    summary.get(field) == legacy.get(field),
                    summary.get(field),
                    legacy.get(field),
                )
            float_tol = float(gate_cfg["shadow_float_abs_tolerance"])
            for field in gate_cfg["shadow_float_fields"]:
                actual = _finite_summary_value(summary, field)
                expected = _finite_summary_value(legacy, field)
                extra(
                    f"shadow_float.{field}",
                    math.isclose(actual, expected, rel_tol=0.0, abs_tol=float_tol),
                    actual,
                    {"paired_legacy": expected, "absolute_tolerance": float_tol},
                )
            extra(
                "shadow_policy_trace_artifact",
                (output_root / case.case_id / "rollout_policy_trace.json").is_file(),
                str(output_root / case.case_id / "rollout_policy_trace.json"),
                "existing regular file",
            )
            extra(
                "shadow_sim_outputs_artifact",
                (output_root / case.case_id / "sim_outputs").is_dir(),
                str(output_root / case.case_id / "sim_outputs"),
                "existing directory",
            )

        failed_extra = [
            item["name"] for item in extra_checks if item["status"] != "PASS"
        ]
        classified["extra_checks"] = extra_checks
        classified["paired_legacy"] = {
            "case_id": next(
                candidate.case_id
                for candidate in cases
                if candidate.mode_name == "legacy"
                and candidate.cell_key == case.cell_key
            ),
            "reserve_norm_max_nm": legacy_reserve,
            "reserve_numerical_tolerance_nm": tolerance,
        }
        summary_path = output_root / case.case_id / "rollout_summary.json"
        classified["summary_path"] = str(summary_path)
        classified["summary_sha256"] = _sha256(summary_path)
        if failed_extra:
            classified["status"] = "FAIL"
            classified["failed_checks"] = list(classified["failed_checks"]) + [
                f"extra.{name}" for name in failed_extra
            ]
        results.append(classified)

    passed = all(result["status"] == "PASS" for result in results)
    return {
        "ok": passed,
        "status": "PASS" if passed else "FAIL",
        "manifest": {
            "path": str(manifest["_manifest_path"]),
            "sha256": manifest["_manifest_sha256"],
        },
        "heldout_contract": {
            "status": "SEALED",
            "forbidden_seeds": list(FORBIDDEN_HELDOUT_SEEDS),
            "heldout_cases_run": 0,
        },
        "case_count": len(results),
        "failed_cases": [
            result["name"] for result in results if result["status"] != "PASS"
        ],
        "results": results,
    }


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def run_matrix(
    manifest: Mapping[str, Any],
    cases: Sequence[MatrixCase],
    output_root: Path,
) -> dict[str, Any]:
    preflight = dry_run_report(manifest, cases, output_root)
    _require(
        preflight["execution_ready"] is True,
        "run refused by preflight: " + ", ".join(preflight["blocking_reasons"]),
    )
    output_root.mkdir(parents=True, exist_ok=False)
    report_path = output_root / REPORT_FILENAME
    progress: dict[str, Any] = {
        "ok": False,
        "status": "RUNNING",
        "manifest_sha256": manifest["_manifest_sha256"],
        "heldout_contract": {
            "status": "SEALED",
            "forbidden_seeds": list(FORBIDDEN_HELDOUT_SEEDS),
            "heldout_cases_run": 0,
        },
        "case_count": len(cases),
        "completed_cases": [],
    }
    _write_json_atomic(report_path, progress)
    for case in cases:
        case_dir = output_root / case.case_id
        _require(not os.path.lexists(case_dir), f"case output already exists: {case_dir}")
        command = build_rollout_command(manifest, case, case_dir)
        completed = subprocess.run(command, cwd=ROOT_DIR, check=False)
        if completed.returncode != 0:
            progress["status"] = "FAIL"
            progress["failed_case"] = case.case_id
            progress["failed_returncode"] = completed.returncode
            progress["failed_command"] = command
            _write_json_atomic(report_path, progress)
            raise ProtocolError(
                f"rollout failed for {case.case_id} with code {completed.returncode}"
            )
        _read_case_summary(case_dir)
        progress["completed_cases"].append(case.case_id)
        _write_json_atomic(report_path, progress)
    final = _classify_complete_matrix(manifest, cases, output_root)
    _write_json_atomic(report_path, final)
    return final


def _list_cases(cases: Sequence[MatrixCase]) -> str:
    lines = [
        "case_id\tcheckpoint\tmode\taction\tseed\tstart\toffset_s\tfsm_input"
    ]
    for case in cases:
        lines.append(
            "\t".join(
                (
                    case.case_id,
                    case.checkpoint_name,
                    case.mode_name,
                    case.action_selection,
                    str(case.seed),
                    case.start_name,
                    repr(case.offset_s),
                    case.phase_fsm_input_mode,
                )
            )
        )
    return "\n".join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="No-clobber matrix output root; defaults to the manifest-pinned path.",
    )
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--list-cases", action="store_true")
    operation.add_argument("--dry-run", action="store_true")
    operation.add_argument("--run", action="store_true")
    operation.add_argument("--classify-existing", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        manifest = load_and_validate_manifest(args.manifest)
        cases = build_cases(manifest)
        output_root = (
            args.output_dir.expanduser().resolve(strict=False)
            if args.output_dir is not None
            else manifest["_output_root"]
        )
        if args.list_cases:
            print(_list_cases(cases))
            return 0
        if args.dry_run:
            print(
                json.dumps(
                    dry_run_report(manifest, cases, output_root),
                    indent=2,
                    sort_keys=True,
                    allow_nan=False,
                )
            )
            return 0
        if args.classify_existing:
            report = _classify_complete_matrix(manifest, cases, output_root)
            _write_json_atomic(output_root / REPORT_FILENAME, report)
            print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
            return 0 if report["ok"] else 1
        report = run_matrix(manifest, cases, output_root)
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        return 0 if report["ok"] else 1
    except ProtocolError as exc:
        print(
            json.dumps(
                {"ok": False, "status": "REFUSED", "error": str(exc)},
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
