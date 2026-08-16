"""V25 governed local geometry sweep for the binary phase detector.

V25 is a new development stage.  It does not reopen or rewrite V21--V23 and
it can run only after the V24 clearance-gap diagnostic has produced a PASS.
The parent V21 x/lateral placement, mesh anchor, binary contact rule, V20 FSM,
1 ms sampling, and every numerical event gate are frozen.  Only one plantar
reach is varied at a time in the first stage.  A small two-dimensional reach
cross is opened only when no single-sensor candidate is eligible.

The executable path is intentionally macOS-only because the numerical plugin
attestation is macOS-only.  Parsing and ``--check`` remain platform-neutral.
No runtime profile is modified by this script.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
import platform
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
V25_SCRIPT_PATH = Path(__file__).resolve()
for _import_root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))


SCHEMA_VERSION = 25
SWEEP_ID = "AB06_BINARY_POINT_V25_LOCAL_REACH_SWEEP_DEV02_04_08"
TRIALS = ("02", "04", "08")
PROTECTED_TRIALS = ("05", "06")
RESERVE_TRIALS = ("03", "07")
HISTORICAL_TRIALS = ("01",)
SAMPLE_DT_S = 0.001
POLICY_SAMPLES = 10
EXPECTED_VIEW_COUNT = 4
EXPECTED_VERIFICATION_UNITS = len(TRIALS) * EXPECTED_VIEW_COUNT * 2
FOOT_FRAME = "/bodyset/foot_l"
GROUND_ORIGIN = (0.0, 0.0148208231, 0.0)
GROUND_NORMAL = (0.0, 1.0, 0.0)
MESH_PROXIMITY_TOLERANCE_M = 1e-6

PARENT_CANDIDATE_ID = "v21_678b0b5162b706dd"
PARENT_PROFILE_PATH = (
    VALIDATION_ROOT
    / "binary_phase_detector_v21_runs/2026-08-04_run01/"
    "eligible_finalist_profile.json"
)
PARENT_PROFILE_SHA256 = (
    "be8e063304a4798e5fc9947beb69c7b2ad813b4cab65e3bfdb0f2cd7284439bc"
)
PARENT_HEEL_X_M = -0.059315516055
PARENT_TOE_X_M = 0.135837908089
PARENT_HEEL_REACH_MM = 25.0
PARENT_TOE_REACH_MM = 27.0

V24_DIAGNOSTIC_ID = (
    "AB06_BINARY_POINT_V24_TRIAL08_CLEARANCE_GAP_DIAGNOSTIC"
)
V24_DIAGNOSTIC_RECEIPT = (
    VALIDATION_ROOT
    / "binary_phase_detector_v24_diagnostic_runs/"
    "2026-08-04_trial08_clearance_gap/diagnostic_receipt.json"
)
V24_DIAGNOSTIC_DETAIL = V24_DIAGNOSTIC_RECEIPT.with_name(
    "clearance_gap_diagnostic.json"
)
V24_DIAGNOSTIC_SCRIPT = (
    VALIDATION_ROOT / "diagnose_binary_phase_detector_v24_clearance_gap.py"
)
V24_DIAGNOSTIC_RECEIPT_SHA256 = (
    "3c4c0caa78745bafeaae812f332158f44152c4148a99d6aa4fb3fd0ec36c59c2"
)
V24_DIAGNOSTIC_DETAIL_SHA256 = (
    "72112d308810685263d5f7af0c4371c6bcf0336e92192da9550feed6253a3546"
)
V24_DIAGNOSTIC_SCRIPT_SHA256 = (
    "5a037622848fbd47ee302b158fe7ec17f0fbdde4f5cd51f3681ea5653282a0c0"
)
V24_STATUS = "PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED"
V24_TOE_ONLY_CLOSURE_MM = 0.3467728406723584
V23_TERMINAL_DECISION_SHA256 = (
    "de55bebbd9b1a21bf3aaadd7132cebed34554e14f5034797b159b14969a4510c"
)

# Frozen stage-one axes.  Baseline is evaluated once even though it belongs to
# both conceptual arms.  There are no CLI geometry overrides.
SINGLE_SENSOR_ARMS_MM = {
    "toe_only": {
        "heel": (PARENT_HEEL_REACH_MM,),
        "toe": (27.0, 27.25, 27.50, 27.75, 28.0),
    },
    "heel_only": {
        "heel": (25.0, 25.25, 25.50, 25.75, 26.0),
        "toe": (PARENT_TOE_REACH_MM,),
    },
}

# This cross is not evaluated when stage one yields an exactly verified
# candidate.  Its declaration is nevertheless frozen before any trace access.
CONDITIONAL_STAGE2_MM = {
    "heel": (25.25, 25.50, 25.75, 26.0),
    "toe": (27.25, 27.50, 27.75, 28.0),
}

SELECTION_RULE = {
    "order": [
        "all_frozen_gates_and_exact_scalar_batch_verification",
        "zero_raw_off_off_handoff_gaps_inside_oracle_stance",
        "minimum_distance_to_fixed_arm_center",
        "minimum_l1_reach_change_from_v21_parent",
        "candidate_id_lexical",
    ],
    "note": (
        "The fixed-arm-centre rule is declared before execution; no numerical "
        "winner is hard-coded."
    ),
}

TRIAL_SPECS = {
    "02": {
        "interval_s": (9.875, 153.080),
        "sample_count": 143206,
        "lock": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed/"
            "trial_02/treadmill_02_01_preprocessing_lock.json"
        ),
        "lock_sha256": (
            "5b086a1a49e6dc8cf893ec43b6f5b1fa1dddb84bfffcb4d216a9287208cb472d"
        ),
        "ik_sha256": (
            "4018b3001a5d293edda799839158f4c154747af5c908a2eb530dae3e37e5a982"
        ),
        "oracle": (
            "validation/canonical_event_oracles/2026-08-03_v17_development/"
            "trial_02_canonical_event_ledger.json"
        ),
        "oracle_sha256": (
            "acfb502bd742055dda49ae9f5398900f87f33368c434e986c98fae127c98894d"
        ),
        "oracle_core_sha256": (
            "ad8b8cc9cccaa9d74fe7d12df8765ac1e9ffffc6581818098b7ee30aba589e4a"
        ),
    },
    "04": {
        "interval_s": (12.485, 156.025),
        "sample_count": 143541,
        "lock": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed/"
            "trial_04/treadmill_04_01_preprocessing_lock.json"
        ),
        "lock_sha256": (
            "e616ef95bae42ac6e99060abca1dc4a323ddaae1b68b01dcaafce36c407d6804"
        ),
        "ik_sha256": (
            "ade6b105d2f6f3cdb350852f92caa48b09cdfa8cd769bf5c02ef5fbdd7b7218f"
        ),
        "oracle": (
            "validation/canonical_event_oracles/2026-08-03_v17_development/"
            "trial_04_canonical_event_ledger.json"
        ),
        "oracle_sha256": (
            "4f48813bd8c6bd5117cd52926e9dc921b01296dc548c0d51f3799171d398f813"
        ),
        "oracle_core_sha256": (
            "f2b8e29aff4ce40daaaafae4a382f031afa3ce56043d6c4f259c13ff4e043227"
        ),
    },
    "08": {
        "interval_s": (10.690, 154.890),
        "sample_count": 144201,
        "lock": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed/"
            "trial_08/treadmill_08_01_preprocessing_lock.json"
        ),
        "lock_sha256": (
            "c961e191871f0caab54edef82424cab20ca3e2938525af8fd1f2752c092399ca"
        ),
        "ik_sha256": (
            "87af7ec5e6530f179bb2fd84c7f2caf78da041f6dcd92966ac8ca1999e1ce14a"
        ),
        "oracle": (
            "validation/canonical_event_oracles/2026-08-03_v17_development/"
            "trial_08_canonical_event_ledger.json"
        ),
        "oracle_sha256": (
            "aa0cf5a2b044bcf5faecf012e8eac5a3693a48459b9dfabc1317536288021f16"
        ),
        "oracle_core_sha256": (
            "1e596953892a64cbda8c1026b582f792c8f19b872c376292fe1e4eb26c71768c"
        ),
    },
}

PINNED_MODEL_SHA256 = (
    "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d"
)
PINNED_PLUGIN_SHA256 = (
    "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b"
)

PINNED_SOURCES = {
    "validation/diagnose_binary_phase_detector_v24_clearance_gap.py": (
        "5a037622848fbd47ee302b158fe7ec17f0fbdde4f5cd51f3681ea5653282a0c0"
    ),
    "validation/sweep_binary_phase_detector_v21_geometry.py": (
        "32aeb6dadba42000607e7d1e7a2480d16574b032fb0d59ad2cd5fd6558147a47"
    ),
    "validation/validate_binary_phase_detector_v19_raw_geometry.py": (
        "0e16fba4a26b5910d38004c0c56a98435e9bc889656ea728b831f158d30a0eea"
    ),
    "validation/validate_binary_phase_fsm_v20_development.py": (
        "1ce0b82a4e9db3b2d90b4dc132de798f8eb452e784fe2492aa76ccbe10e8e431"
    ),
    "Trajectory Generator/binary_phase_fsm.py": (
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1"
    ),
    "binary_phase_detector.py": (
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6"
    ),
    "validation/build_two_sensor_mesh_profile_v4.py": (
        "671b59ca546112b720b915816a89eec14eaf3dd3cf52a9a37388074b23ef4bda"
    ),
    "validation/audit_two_sensor_prescribed_geometry.py": (
        "c82d114ab8550d6963f394a2cfa99decf94a992880e751792ac56af8dd76a307"
    ),
    "kinematics_interpolator.py": (
        "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede"
    ),
    "config.py": (
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3"
    ),
    "model_loader.py": (
        "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76"
    ),
    "validation/binary_phase_detector_v21_runs/2026-08-04_run01/manifest.json": (
        "ecc89b780a22a0762a121572234cfde2a1062762e03981935f8484ac0b21055e"
    ),
    "validation/binary_phase_detector_v23_development_runs/"
    "2026-08-04_trial08_assertions_facts_fix/"
    "trial08_development_decision_lock.json": V23_TERMINAL_DECISION_SHA256,
    "Geometry/AM_foot_l.STL": (
        "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9"
    ),
}


class V25SweepError(RuntimeError):
    """Raised when V25 cannot preserve its frozen development contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _reject_json_constant(value: str) -> None:
    raise V25SweepError(f"non-finite JSON constant is forbidden: {value}")


def _strict_json(path: Path, expected_sha256: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise V25SweepError(f"required JSON is missing: {path}")
    observed = sha256_file(path)
    if expected_sha256 is not None and observed != expected_sha256:
        raise V25SweepError(
            f"JSON hash drifted: {path}: {observed} != {expected_sha256}"
        )
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V25SweepError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise V25SweepError(f"JSON root must be an object: {path}")
    return payload


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return str(resolved)


def _source_record(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise V25SweepError(f"required source is missing: {path}")
    return {
        "path": _portable_path(path),
        "sha256": sha256_file(path),
        "bytes": int(path.stat().st_size),
    }


def _atomic_write_exclusive(
    path: Path, writer: Callable[[TextIO], None]
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Path.exists() is false for a dangling symlink; lexists preserves the
    # no-clobber contract for every directory entry type.
    if os.path.lexists(path):
        raise FileExistsError(f"refusing to clobber {path}")
    descriptor, raw_temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(raw_temporary)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            descriptor_open = False
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _fsync_directory(path: Path) -> None:
    """Durably publish a newly linked receipt on the numerical platform."""

    flags = os.O_RDONLY | int(getattr(os, "O_DIRECTORY", 0))
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    def writer(stream: TextIO) -> None:
        json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    return _atomic_write_exclusive(path, writer)


def _write_jsonl_exclusive(
    path: Path, rows: Sequence[Mapping[str, Any]]
) -> Path:
    def writer(stream: TextIO) -> None:
        for row in rows:
            stream.write(json.dumps(dict(row), sort_keys=True, allow_nan=False))
            stream.write("\n")

    return _atomic_write_exclusive(path, writer)


def _verify_v24_prerequisite() -> dict[str, Any]:
    receipt = _strict_json(
        V24_DIAGNOSTIC_RECEIPT, V24_DIAGNOSTIC_RECEIPT_SHA256
    )
    detail = _strict_json(
        V24_DIAGNOSTIC_DETAIL, V24_DIAGNOSTIC_DETAIL_SHA256
    )
    if sha256_file(V24_DIAGNOSTIC_SCRIPT) != V24_DIAGNOSTIC_SCRIPT_SHA256:
        raise V25SweepError("V24 diagnostic script drifted")
    diagnostic_record = receipt.get("diagnostic")
    comparison = detail.get("marker_runtime_comparison", {})
    reach = detail.get("reach_counterfactual", {})
    lineage = detail.get("lineage_assertions", {})
    conclusion = detail.get("conclusion", {})
    oracle_assertions = detail.get("oracle_context", {}).get("assertions", {})
    assertions = {
        "schema_version": receipt.get("schema_version") == 24,
        "diagnostic_id": receipt.get("diagnostic_id") == V24_DIAGNOSTIC_ID,
        "status_pass": receipt.get("status") == V24_STATUS,
        "pass_boolean": receipt.get("pass") is True,
        "candidate_exact": receipt.get("candidate_id") == PARENT_CANDIDATE_ID,
        "trial_exact": receipt.get("trial_id") == "08",
        "v23_false_air_reproduced": receipt.get(
            "v23_false_air_run_reproduced"
        )
        is True,
        "diagnostic_record_hash": isinstance(diagnostic_record, Mapping)
        and diagnostic_record.get("sha256") == V24_DIAGNOSTIC_DETAIL_SHA256,
        "marker_runtime_clearance_equivalent": receipt.get(
            "marker_runtime_clearance_equivalent"
        )
        is True,
        "marker_runtime_difference_zero": comparison.get(
            "maximum_absolute_heel_clearance_difference_m"
        )
        == 0.0
        and comparison.get("maximum_absolute_toe_clearance_difference_m")
        == 0.0
        and comparison.get("maximum_absolute_reach_gain_difference") == 0.0,
        "toe_only_closure_exact": abs(
            float(
                receipt.get(
                    "minimum_toe_only_reach_increase_to_close_gap_mm",
                    math.nan,
                )
            )
            - V24_TOE_ONLY_CLOSURE_MM
        )
        <= 1e-15
        and abs(
            float(
                reach.get(
                    "minimum_toe_only_reach_increase_to_close_gap_mm",
                    math.nan,
                )
            )
            - V24_TOE_ONLY_CLOSURE_MM
        )
        <= 1e-15,
        "v23_identity_and_terminal_failure": lineage.get("candidate_exact")
        is True
        and lineage.get("v23_cycle_consumed") is True
        and lineage.get("v23_terminal_fail_exact") is True
        and lineage.get("v23_unit_result_exact") is True,
        "geometric_not_fsm_conclusion": conclusion.get(
            "geometry_or_kinematic_signal_gap_confirmed"
        )
        is True
        and conclusion.get("fsm_created_raw_gap") is False
        and conclusion.get("fsm_change_required_by_this_diagnostic") is False,
        "oracle_same_stance_no_event": oracle_assertions.get(
            "same_oracle_stance"
        )
        is True
        and oracle_assertions.get("no_oracle_event_in_false_gap") is True,
    }
    if not all(assertions.values()):
        raise V25SweepError(f"V24 diagnostic prerequisite failed: {assertions}")
    return {
        "pass": True,
        "assertions": assertions,
        "receipt": _source_record(V24_DIAGNOSTIC_RECEIPT),
        "diagnostic_detail": _source_record(V24_DIAGNOSTIC_DETAIL),
        "diagnostic_script": _source_record(V24_DIAGNOSTIC_SCRIPT),
        "toe_only_closure_mm": V24_TOE_ONLY_CLOSURE_MM,
    }


def _verify_pinned_sources() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative, expected in PINNED_SOURCES.items():
        path = REPO_ROOT / relative
        observed = sha256_file(path) if path.is_file() else None
        if observed != expected:
            raise V25SweepError(
                f"pinned source drifted: {relative}: {observed} != {expected}"
            )
        records[relative] = _source_record(path)
    if sha256_file(PARENT_PROFILE_PATH) != PARENT_PROFILE_SHA256:
        raise V25SweepError("V21 parent profile drifted")
    records["v21_parent_profile"] = _source_record(PARENT_PROFILE_PATH)
    for trial_id, spec in TRIAL_SPECS.items():
        lock_path = REPO_ROOT / str(spec["lock"])
        if sha256_file(lock_path) != spec["lock_sha256"]:
            raise V25SweepError(f"trial {trial_id} preprocessing lock drifted")
        oracle_path = REPO_ROOT / str(spec["oracle"])
        if sha256_file(oracle_path) != spec["oracle_sha256"]:
            raise V25SweepError(f"trial {trial_id} oracle drifted")
        records[f"trial_{trial_id}_preprocessing_lock"] = _source_record(lock_path)
        records[f"trial_{trial_id}_oracle"] = _source_record(oracle_path)
    return records


def _repo_record_path(record: object, *, label: str) -> Path:
    if not isinstance(record, Mapping):
        raise V25SweepError(f"{label} record is missing")
    raw_path = record.get("path")
    expected = record.get("sha256")
    if (
        not isinstance(raw_path, str)
        or not raw_path
        or not isinstance(expected, str)
        or not expected
        or Path(raw_path).is_absolute()
        or "\\" in raw_path
    ):
        raise V25SweepError(f"{label} record is malformed or non-portable")
    path = (REPO_ROOT / raw_path).resolve()
    try:
        path.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V25SweepError(f"{label} escapes repository") from exc
    if not path.is_file() or sha256_file(path) != expected:
        raise V25SweepError(f"{label} source drifted")
    return path


def _trial_inputs(trial_id: str) -> dict[str, Any]:
    spec = TRIAL_SPECS[trial_id]
    lock_path = REPO_ROOT / str(spec["lock"])
    lock = _strict_json(lock_path, str(spec["lock_sha256"]))
    assertions = {
        "trial": lock.get("trial_id") == trial_id,
        "development": lock.get("stage") == "development",
        "status": lock.get("status")
        == "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
        "interval": lock.get("analysis_interval_s")
        == list(spec["interval_s"]),
        "absolute_time": lock.get("absolute_timestamps_no_rezero") is True,
        "coverage": lock.get("all_sources_cover_analysis_interval") is True,
        "no_adaptive_preprocessing": lock.get(
            "adaptive_crop_resample_or_interpolation_used"
        )
        is False,
        "dataset_ik_not_used": lock.get("dataset_ik_used_downstream") is False,
    }
    if not all(assertions.values()):
        raise V25SweepError(f"trial {trial_id} input contract failed: {assertions}")
    ik_path = _repo_record_path(lock.get("ik_motion"), label=f"trial {trial_id} IK")
    if sha256_file(ik_path) != spec["ik_sha256"]:
        raise V25SweepError(f"trial {trial_id} IK hash drifted")
    live = lock.get("live_replay_inputs")
    if not isinstance(live, Mapping):
        raise V25SweepError(f"trial {trial_id} live replay inputs are missing")
    model_path = _repo_record_path(live.get("model"), label="model")
    if sha256_file(model_path) != PINNED_MODEL_SHA256:
        raise V25SweepError("runtime model hash drifted")
    plugin = live.get("plugin")
    if not isinstance(plugin, Mapping):
        raise V25SweepError("plugin input record is missing")
    binary_raw = plugin.get("binary_path")
    binary_sha = plugin.get("binary_sha256")
    loader = plugin.get("loader_basename")
    if not all(isinstance(item, str) and item for item in (binary_raw, binary_sha, loader)):
        raise V25SweepError("plugin input record is malformed")
    plugin_binary = (REPO_ROOT / str(binary_raw)).resolve()
    if (
        not plugin_binary.is_file()
        or binary_sha != PINNED_PLUGIN_SHA256
        or sha256_file(plugin_binary) != PINNED_PLUGIN_SHA256
    ):
        raise V25SweepError("macOS plugin binary drifted")
    return {
        "trial_id": trial_id,
        "lock_path": lock_path,
        "ik_path": ik_path,
        "model_path": model_path,
        "plugin_binary_path": plugin_binary,
        "plugin_loader": str((REPO_ROOT / str(loader)).resolve()),
        "assertions": assertions,
    }


def _load_oracles() -> dict[str, dict[str, Any]]:
    ledgers: dict[str, dict[str, Any]] = {}
    for trial_id, spec in TRIAL_SPECS.items():
        path = REPO_ROOT / str(spec["oracle"])
        ledger = _strict_json(path, str(spec["oracle_sha256"]))
        core = ledger.get("scientific_core")
        checks = {
            "trial": isinstance(core, Mapping)
            and core.get("trial_id") == trial_id,
            "core_sha_field": ledger.get("scientific_core_sha256")
            == spec["oracle_core_sha256"],
            "core_sha_recomputed": isinstance(core, Mapping)
            and canonical_sha256(core) == spec["oracle_core_sha256"],
            "sample_dt": isinstance(core, Mapping)
            and abs(float(core.get("sample_dt_s", math.nan)) - SAMPLE_DT_S)
            <= 1e-12,
            "view_count": isinstance(core, Mapping)
            and len(core.get("views", [])) == EXPECTED_VIEW_COUNT,
            "cycles_present": isinstance(core, Mapping)
            and len(core.get("cycles", [])) > 0,
        }
        if not all(checks.values()):
            raise V25SweepError(f"trial {trial_id} oracle gate failed: {checks}")
        ledgers[trial_id] = ledger
    return ledgers


@dataclass(frozen=True)
class V25Candidate:
    heel: Any
    toe: Any
    stage: str
    arm_membership: tuple[str, ...]

    @property
    def candidate_id(self) -> str:
        core = {
            "heel_x_m": round(float(self.heel.x_m), 12),
            "heel_reach_m": round(float(self.heel.reach_m), 12),
            "toe_x_m": round(float(self.toe.x_m), 12),
            "toe_reach_m": round(float(self.toe.reach_m), 12),
        }
        return "v25_" + canonical_sha256(core)[:16]

    @property
    def key(self) -> tuple[float, float, float, float]:
        return (
            round(float(self.heel.x_m), 12),
            round(float(self.heel.reach_m), 12),
            round(float(self.toe.x_m), 12),
            round(float(self.toe.reach_m), 12),
        )

    @property
    def reach_delta_l1_mm(self) -> float:
        return float(
            abs(1000.0 * self.heel.reach_m - PARENT_HEEL_REACH_MM)
            + abs(1000.0 * self.toe.reach_m - PARENT_TOE_REACH_MM)
        )

    def payload(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "stage": self.stage,
            "arm_membership": list(self.arm_membership),
            "parent_candidate_id": PARENT_CANDIDATE_ID,
            "heel": self.heel.payload(),
            "toe": self.toe.payload(),
            "reach_delta_l1_mm": self.reach_delta_l1_mm,
        }


def _progress_options(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "width": int(args.progress_width),
        "min_redraw_interval_s": float(args.progress_interval_s),
        "non_tty_interval_s": float(args.non_tty_progress_interval_s),
        "enabled": not bool(args.no_progress),
    }


def _time_grid(trial_id: str) -> np.ndarray:
    spec = TRIAL_SPECS[trial_id]
    start, end = (float(item) for item in spec["interval_s"])
    times = start + np.arange(int(spec["sample_count"]), dtype=float) * SAMPLE_DT_S
    if abs(float(times[-1]) - end) > 1e-10:
        raise V25SweepError(f"trial {trial_id} time lattice drifted")
    return times


def _acquire_affine_trial(
    trial_id: str,
    *,
    v21: Any,
    opensim: Any,
    SimulatorConfig: Any,
    KinematicsInterpolator: Any,
    load_plugin: Callable[[str], None],
    progress_options: Mapping[str, Any],
) -> Any:
    inputs = _trial_inputs(trial_id)
    load_plugin(inputs["plugin_loader"])
    model = opensim.Model(str(inputs["model_path"]))
    state = model.initSystem()
    frame = opensim.PhysicalFrame.safeDownCast(model.getComponent(FOOT_FRAME))
    if frame is None:
        raise V25SweepError(f"cannot resolve PhysicalFrame {FOOT_FRAME}")
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(Path(inputs["model_path"]).parent)
    cfg.model_file = str(inputs["model_path"])
    cfg.kinematics_file = str(inputs["ik_path"])
    cfg.t_start, cfg.t_end = TRIAL_SPECS[trial_id]["interval_s"]
    kinematics = KinematicsInterpolator(cfg)
    times = _time_grid(trial_id)
    coefficients = np.empty((times.size, 3), dtype=float)
    offsets = np.empty(times.size, dtype=float)
    coordinates = model.getCoordinateSet()
    normal = np.asarray(GROUND_NORMAL, dtype=float)
    ground_origin = np.asarray(GROUND_ORIGIN, dtype=float)
    local_stations = (
        opensim.Vec3(0.0, 0.0, 0.0),
        opensim.Vec3(1.0, 0.0, 0.0),
        opensim.Vec3(0.0, 1.0, 0.0),
        opensim.Vec3(0.0, 0.0, 1.0),
    )
    progress = v21.SweepProgress(
        total=int(times.size),
        label=f"V25 cache DEV{trial_id}",
        **progress_options,
    )
    try:
        for row, time_s in enumerate(times):
            q, _qdot, _qddot = kinematics.get(float(time_s))
            state.setTime(float(time_s))
            for index in range(coordinates.getSize()):
                coordinate = coordinates.get(index)
                name = coordinate.getName()
                if name in q:
                    coordinate.setValue(state, float(q[name]), False)
            model.realizePosition(state)
            stations = []
            for local in local_stations:
                ground = frame.findStationLocationInGround(state, local)
                stations.append(
                    np.asarray(
                        [float(ground.get(i)) for i in range(3)], dtype=float
                    )
                )
            origin = stations[0]
            coefficients[row] = [
                float(normal @ (stations[index] - origin))
                for index in (1, 2, 3)
            ]
            offsets[row] = float(normal @ (origin - ground_origin))
            if row % 100 == 0 or row + 1 == times.size:
                progress.update(row + 1)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    if not np.all(np.isfinite(coefficients)) or not np.all(np.isfinite(offsets)):
        raise V25SweepError(f"trial {trial_id} affine cache is non-finite")
    return v21.AffineTrialTrace(
        trial_id=trial_id,
        time_s=times,
        normal_coefficients=coefficients,
        normal_offset=offsets,
        source={
            "preprocessing_lock": _source_record(inputs["lock_path"]),
            "model": _source_record(inputs["model_path"]),
            "ik_motion": _source_record(inputs["ik_path"]),
            "plugin_binary": _source_record(inputs["plugin_binary_path"]),
        },
    )


def _candidate_bits(
    trial_id: str,
    candidate: V25Candidate,
    traces: Mapping[str, Any],
    v21: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    trace = traces[trial_id]

    def bits(point: Any) -> np.ndarray:
        key = (
            trial_id,
            str(point.role),
            tuple(round(float(item), 12) for item in point.location_m),
        )
        if key not in bit_cache:
            bit_cache[key] = v21.point_contact_bits(trace, point)
        return bit_cache[key]

    return bits(candidate.heel), bits(candidate.toe)


def _clearance(trace: Any, point: Any) -> np.ndarray:
    location = np.asarray(point.location_m, dtype=float)
    values = trace.normal_coefficients @ location + trace.normal_offset
    if not np.all(np.isfinite(values)):
        raise V25SweepError("derived signed-clearance trace is non-finite")
    return values


def _false_runs(values: np.ndarray) -> list[tuple[int, int]]:
    mask = np.asarray(values, dtype=bool)
    padded = np.r_[True, mask, True]
    starts = np.flatnonzero(~padded[1:] & padded[:-1])
    ends = np.flatnonzero(padded[1:] & ~padded[:-1]) - 1
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def oracle_stance_continuity_gate(
    candidate: V25Candidate,
    *,
    traces: Mapping[str, Any],
    ledgers: Mapping[str, Mapping[str, Any]],
    v21: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
) -> dict[str, Any]:
    """Reject every raw OFF/OFF handoff gap surrounded by stance contact.

    The oracle HS--TO interval identifies stance, while the first and last raw
    any-contact samples remove ordinary leading/trailing event latency.  A run
    is therefore called an intra-stance handoff gap only when raw contact is
    present both before and after it in the same oracle stance.  Normal swing
    is never included.  The gate is stricter than debounce: even a 1 ms gap is
    rejected, while the >=6-sample diagnostic is retained for causality audit.
    """

    trial_receipts: dict[str, Any] = {}
    all_gaps: list[dict[str, Any]] = []
    for trial_id in TRIALS:
        trace = traces[trial_id]
        heel, toe = _candidate_bits(
            trial_id, candidate, traces, v21, bit_cache
        )
        any_contact = heel | toe
        heel_clearance = _clearance(trace, candidate.heel)
        toe_clearance = _clearance(trace, candidate.toe)
        gaps: list[dict[str, Any]] = []
        cycles = ledgers[trial_id]["scientific_core"]["cycles"]
        for cycle in cycles:
            hs = float(cycle["heel_strike_time_s"])
            toe_off = float(cycle["toe_off_time_s"])
            indices = np.flatnonzero(
                (trace.time_s >= hs - 1e-10)
                & (trace.time_s <= toe_off + 1e-10)
            )
            if indices.size < 3:
                continue
            stance_bits = any_contact[indices]
            contact_indices = np.flatnonzero(stance_bits)
            if contact_indices.size < 2:
                continue
            left = int(contact_indices[0])
            right = int(contact_indices[-1])
            core = stance_bits[left : right + 1]
            for local_start, local_end in _false_runs(core):
                gap_indices = indices[
                    left + local_start : left + local_end + 1
                ]
                r_yy = trace.normal_coefficients[gap_indices, 1]
                if np.any(r_yy <= 0.0) or not np.all(np.isfinite(r_yy)):
                    raise V25SweepError(
                        f"trial {trial_id} has invalid reach sensitivity R_yy"
                    )
                toe_needed = np.maximum(toe_clearance[gap_indices], 0.0) / r_yy
                heel_needed = np.maximum(heel_clearance[gap_indices], 0.0) / r_yy
                record = {
                    "trial_id": trial_id,
                    "cycle_id": str(cycle["cycle_id"]),
                    "start_time_s": float(trace.time_s[gap_indices[0]]),
                    "end_time_s": float(trace.time_s[gap_indices[-1]]),
                    "sample_count": int(gap_indices.size),
                    "elapsed_first_to_last_s": float(
                        trace.time_s[gap_indices[-1]]
                        - trace.time_s[gap_indices[0]]
                    ),
                    "half_open_duration_s": float(gap_indices.size * SAMPLE_DT_S),
                    "debounce_stable": bool(
                        gap_indices.size >= v21.MIN_STABLE_RUN_SAMPLES
                    ),
                    "shorter_than_30ms": bool(
                        gap_indices.size * SAMPLE_DT_S < 0.030 - 1e-12
                    ),
                    "r_yy_minimum": float(np.min(r_yy)),
                    "r_yy_maximum": float(np.max(r_yy)),
                    "signed_clearance_derivative_wrt_reach": "-R_yy",
                    "toe_extra_reach_to_cover_entire_gap_m": float(
                        np.max(toe_needed)
                    ),
                    "heel_extra_reach_to_cover_entire_gap_m": float(
                        np.max(heel_needed)
                    ),
                }
                gaps.append(record)
                all_gaps.append(record)
        trial_receipts[trial_id] = {
            "cycle_count_checked": len(cycles),
            "raw_internal_gap_count": len(gaps),
            "debounce_stable_internal_gap_count": sum(
                bool(item["debounce_stable"]) for item in gaps
            ),
            "pass": not gaps,
            "gaps": gaps,
        }
    return {
        "definition": (
            "raw OFF/OFF run bounded by raw any-contact on both sides inside "
            "one canonical oracle HS-to-TO stance"
        ),
        "ordinary_swing_excluded": True,
        "maximum_allowed_internal_gap_samples": 0,
        "debounce_stability_samples": int(v21.MIN_STABLE_RUN_SAMPLES),
        "trial_count": len(TRIALS),
        "raw_internal_gap_count": len(all_gaps),
        "debounce_stable_internal_gap_count": sum(
            bool(item["debounce_stable"]) for item in all_gaps
        ),
        "pass": not all_gaps,
        "trials": trial_receipts,
    }


def _geometry_gate(
    candidate: V25Candidate,
    *,
    triangles: np.ndarray,
    minimum_mesh_distance: Callable[[Sequence[float], np.ndarray], float],
) -> dict[str, Any]:
    points: dict[str, Any] = {}
    for point in (candidate.heel, candidate.toe):
        surface = np.asarray(point.surface_location_m, dtype=float)
        location = np.asarray(point.location_m, dtype=float)
        distance = float(minimum_mesh_distance(surface, triangles))
        reach_error = abs(float(surface[1] - location[1]) - float(point.reach_m))
        checks = {
            "surface_finite": bool(np.all(np.isfinite(surface))),
            "location_finite": bool(np.all(np.isfinite(location))),
            "mesh_proximity": distance <= MESH_PROXIMITY_TOLERANCE_M,
            "vertical_reach_exact": reach_error <= 1e-12,
            "x_and_z_anchored": bool(
                abs(float(surface[0] - location[0])) <= 1e-12
                and abs(float(surface[2] - location[2])) <= 1e-12
            ),
        }
        points[str(point.role)] = {
            "minimum_surface_to_mesh_distance_m": distance,
            "vertical_reach_error_m": reach_error,
            "checks": checks,
            "pass": all(checks.values()),
        }
    return {
        "mesh_proximity_tolerance_m": MESH_PROXIMITY_TOLERANCE_M,
        "points": points,
        "pass": all(item["pass"] for item in points.values()),
    }


def _compact_unit(v21: Any, unit: Mapping[str, Any]) -> dict[str, Any]:
    """Keep V21 rank fields plus the timing fields required by V25 audit."""

    result = v21._compact_unit(unit)
    result.update(
        {
            "speed_mps": float(unit["speed_mps"]),
            "consumption_mode": str(unit["consumption_mode"]),
            "minimum_accepted_flight_s": unit["minimum_accepted_flight_s"],
            "scalar_batch_parity_pass": bool(
                unit["scalar_batch_parity_pass"]
            ),
        }
    )
    for event_name in ("heel_strike", "toe_off"):
        source = unit["events"][event_name]
        target = result[event_name]
        target.update(
            {
                "maximum_absolute_onset_error_s": source[
                    "maximum_absolute_onset_error_s"
                ],
                "maximum_absolute_delivered_error_s": source[
                    "maximum_absolute_delivered_error_s"
                ],
                "confirmation_delay_min_s": source[
                    "confirmation_delay_min_s"
                ],
                "confirmation_delay_max_s": source[
                    "confirmation_delay_max_s"
                ],
                "delivery_delay_min_s": source["delivery_delay_min_s"],
                "delivery_delay_max_s": source["delivery_delay_max_s"],
            }
        )
    json.dumps(result, allow_nan=False)
    return result


def _evaluate_candidate(
    candidate: V25Candidate,
    *,
    traces: Mapping[str, Any],
    ledgers: Mapping[str, Mapping[str, Any]],
    v21: Any,
    v20: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
    geometry_gate: Mapping[str, Any],
) -> dict[str, Any]:
    units: list[dict[str, Any]] = []
    channel_trials: dict[str, Any] = {}
    event_counts: dict[str, Any] = {}
    fingerprint = hashlib.sha256()
    for trial_id in TRIALS:
        trace = traces[trial_id]
        heel, toe = _candidate_bits(
            trial_id, candidate, traces, v21, bit_cache
        )
        views = ledgers[trial_id]["scientific_core"]["views"]
        channel_trials[trial_id] = v21.two_sensor_view_gate(
            trace.time_s, heel, toe, views
        )
        packed = np.packbits(
            np.column_stack((heel, toe)).reshape(-1), bitorder="little"
        )
        fingerprint.update(trial_id.encode("ascii"))
        fingerprint.update(packed.tobytes())
        events = v21.fast_fsm_events(trace.time_s, heel, toe)
        event_counts[trial_id] = {
            name: sum(event["event"] == name for event in events)
            for name in ("heel_strike", "toe_off")
        }
        for view in views:
            units.append(
                _compact_unit(
                    v21,
                    v20._score_view(
                        trial_id=trial_id,
                        mode="v25_fast_screen",
                        trace={"time_s": trace.time_s},
                        events=events,
                        view=view,
                        parity_pass=True,
                    ),
                )
            )
    continuity = oracle_stance_continuity_gate(
        candidate,
        traces=traces,
        ledgers=ledgers,
        v21=v21,
        bit_cache=bit_cache,
    )
    channel_pass = all(item["pass"] for item in channel_trials.values())
    units_pass = len(units) == len(TRIALS) * EXPECTED_VIEW_COUNT and all(
        item["pass"] for item in units
    )
    fast_pass = bool(channel_pass and units_pass)
    screen_eligible = bool(
        fast_pass and continuity["pass"] and geometry_gate["pass"]
    )
    return {
        **candidate.payload(),
        "fast_screen": {
            "pass": fast_pass,
            "unit_count": len(units),
            "unit_pass_count": sum(bool(item["pass"]) for item in units),
            "channel_non_degenerate_pass": channel_pass,
            "two_sensor_channel_gate_by_trial": channel_trials,
            "event_counts_full_trace": event_counts,
            "bit_fingerprint_sha256": fingerprint.hexdigest(),
            "units": units,
        },
        "oracle_stance_continuity": continuity,
        "geometry_gate": dict(geometry_gate),
        "screen_eligible": screen_eligible,
    }


def _evaluate_stage(
    label: str,
    candidates: Sequence[V25Candidate],
    *,
    traces: Mapping[str, Any],
    ledgers: Mapping[str, Mapping[str, Any]],
    v21: Any,
    v20: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
    triangles: np.ndarray,
    minimum_mesh_distance: Callable[[Sequence[float], np.ndarray], float],
    progress_options: Mapping[str, Any],
) -> list[dict[str, Any]]:
    progress = v21.SweepProgress(
        total=len(candidates), label=f"V25 {label}", **progress_options
    )
    rows: list[dict[str, Any]] = []
    try:
        for index, candidate in enumerate(candidates, start=1):
            geometry = _geometry_gate(
                candidate,
                triangles=triangles,
                minimum_mesh_distance=minimum_mesh_distance,
            )
            rows.append(
                _evaluate_candidate(
                    candidate,
                    traces=traces,
                    ledgers=ledgers,
                    v21=v21,
                    v20=v20,
                    bit_cache=bit_cache,
                    geometry_gate=geometry,
                )
            )
            progress.update(index)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    return rows


def _verify_candidate(
    candidate: V25Candidate,
    screen: Mapping[str, Any],
    *,
    traces: Mapping[str, Any],
    ledgers: Mapping[str, Mapping[str, Any]],
    v21: Any,
    v20: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
) -> dict[str, Any]:
    modes = ("sequential_1ms", "batched_10ms_same_samples")
    units: list[dict[str, Any]] = []
    parity: list[dict[str, Any]] = []
    channel_trials: dict[str, Any] = {}
    for trial_id in TRIALS:
        trace = traces[trial_id]
        heel, toe = _candidate_bits(
            trial_id, candidate, traces, v21, bit_cache
        )
        views = ledgers[trial_id]["scientific_core"]["views"]
        channel_trials[trial_id] = v21.two_sensor_view_gate(
            trace.time_s, heel, toe, views
        )
        sample_trace = {"time_s": trace.time_s, "heel": heel, "toe": toe}
        results = {mode: v20._run_mode(sample_trace, mode) for mode in modes}
        sequential = results[modes[0]]
        batched = results[modes[1]]
        fast = v21.fast_fsm_events(trace.time_s, heel, toe)
        parity_record = {
            "trial_id": trial_id,
            "events_exact": sequential["events"] == batched["events"],
            "transitions_exact": sequential["contact_state_transitions"]
            == batched["contact_state_transitions"],
            "cancellations_exact": sequential["candidate_cancellations"]
            == batched["candidate_cancellations"],
            "state_digest_exact": sequential["boundary_snapshots_sha256"]
            == batched["boundary_snapshots_sha256"],
            "final_payload_exact": sequential["final_payload"]
            == batched["final_payload"],
            "fast_event_signature_exact": v21.event_signature(fast)
            == v21.event_signature(sequential["events"]),
        }
        parity_record["pass"] = all(
            bool(value)
            for key, value in parity_record.items()
            if key != "trial_id"
        )
        parity.append(parity_record)
        for mode in modes:
            for view in views:
                units.append(
                    _compact_unit(
                        v21,
                        v20._score_view(
                            trial_id=trial_id,
                            mode=mode,
                            trace=sample_trace,
                            events=results[mode]["events"],
                            view=view,
                            parity_pass=bool(parity_record["pass"]),
                        ),
                    )
                )
    channel_pass = all(item["pass"] for item in channel_trials.values())
    toe_clearance_pass = all(item["accepted_flight_pass"] for item in units)
    eligible = bool(
        len(units) == EXPECTED_VERIFICATION_UNITS
        and channel_pass
        and all(item["pass"] for item in units)
        and all(item["pass"] for item in parity)
        and bool(screen["oracle_stance_continuity"]["pass"])
        and bool(screen["geometry_gate"]["pass"])
        and toe_clearance_pass
    )
    return {
        **candidate.payload(),
        "eligible": eligible,
        "expected_unit_count": EXPECTED_VERIFICATION_UNITS,
        "unit_count": len(units),
        "unit_pass_count": sum(bool(item["pass"]) for item in units),
        "two_sensor_channel_gate": {
            "pass": channel_pass,
            "trials": channel_trials,
        },
        "toe_clearance_at_least_30ms_pass": toe_clearance_pass,
        "oracle_stance_continuity": screen["oracle_stance_continuity"],
        "geometry_gate": screen["geometry_gate"],
        "parity": parity,
        "units": units,
    }


def _generate_candidates(factory: Any, stage: str) -> list[V25Candidate]:
    values: dict[tuple[float, float], set[str]] = {}
    if stage == "single_sensor_arms":
        for arm, axes in SINGLE_SENSOR_ARMS_MM.items():
            for heel_mm, toe_mm in itertools.product(axes["heel"], axes["toe"]):
                values.setdefault((float(heel_mm), float(toe_mm)), set()).add(arm)
    elif stage == "conditional_two_dimensional_cross":
        for heel_mm, toe_mm in itertools.product(
            CONDITIONAL_STAGE2_MM["heel"], CONDITIONAL_STAGE2_MM["toe"]
        ):
            values.setdefault((float(heel_mm), float(toe_mm)), set()).add(
                "conditional_cross"
            )
    else:
        raise V25SweepError(f"unsupported candidate stage: {stage}")
    candidates = [
        V25Candidate(
            heel=factory.build("left_heel", PARENT_HEEL_X_M, heel_mm / 1000.0),
            toe=factory.build("left_toe", PARENT_TOE_X_M, toe_mm / 1000.0),
            stage=stage,
            arm_membership=tuple(sorted(arms)),
        )
        for (heel_mm, toe_mm), arms in sorted(values.items())
    ]
    if len({candidate.key for candidate in candidates}) != len(candidates):
        raise V25SweepError("candidate generator produced duplicate geometry")
    return candidates


def _arm_center_distance_mm(candidate: V25Candidate) -> float:
    heel_mm = 1000.0 * float(candidate.heel.reach_m)
    toe_mm = 1000.0 * float(candidate.toe.reach_m)
    distances: list[float] = []
    for arm in candidate.arm_membership:
        if arm in SINGLE_SENSOR_ARMS_MM:
            axes = SINGLE_SENSOR_ARMS_MM[arm]
            heel_center = 0.5 * (min(axes["heel"]) + max(axes["heel"]))
            toe_center = 0.5 * (min(axes["toe"]) + max(axes["toe"]))
            distances.append(abs(heel_mm - heel_center) + abs(toe_mm - toe_center))
    if not distances:
        heel_axis = CONDITIONAL_STAGE2_MM["heel"]
        toe_axis = CONDITIONAL_STAGE2_MM["toe"]
        distances.append(
            abs(heel_mm - 0.5 * (min(heel_axis) + max(heel_axis)))
            + abs(toe_mm - 0.5 * (min(toe_axis) + max(toe_axis)))
        )
    return float(min(distances))


def _selection_key(verification: Mapping[str, Any]) -> tuple[Any, ...]:
    return (
        int(not bool(verification["eligible"])),
        float(verification["selection_metrics"]["fixed_arm_center_distance_mm"]),
        float(verification["reach_delta_l1_mm"]),
        str(verification["candidate_id"]),
    )


def _verify_screen_eligible(
    candidates: Sequence[V25Candidate],
    screens: Sequence[Mapping[str, Any]],
    *,
    traces: Mapping[str, Any],
    ledgers: Mapping[str, Mapping[str, Any]],
    v21: Any,
    v20: Any,
    bit_cache: dict[tuple[str, str, tuple[float, float, float]], np.ndarray],
    progress_options: Mapping[str, Any],
) -> list[dict[str, Any]]:
    by_id = {candidate.candidate_id: candidate for candidate in candidates}
    targets = [row for row in screens if row["screen_eligible"]]
    if not targets:
        return []
    progress = v21.SweepProgress(
        total=len(targets), label="V25 exact verify", **progress_options
    )
    result: list[dict[str, Any]] = []
    try:
        for index, screen in enumerate(targets, start=1):
            candidate = by_id[str(screen["candidate_id"])]
            verification = _verify_candidate(
                candidate,
                screen,
                traces=traces,
                ledgers=ledgers,
                v21=v21,
                v20=v20,
                bit_cache=bit_cache,
            )
            verification["selection_metrics"] = {
                "fixed_arm_center_distance_mm": _arm_center_distance_mm(candidate),
                "reach_delta_l1_mm": candidate.reach_delta_l1_mm,
            }
            result.append(verification)
            progress.update(index)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    return result


def _candidate_profile(candidate: V25Candidate) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "detector_type": "binary_point_clearance_v1",
        "ground": {"origin": list(GROUND_ORIGIN), "normal": list(GROUND_NORMAL)},
        "points": [
            {
                "name": candidate.heel.role,
                "frame": FOOT_FRAME,
                "location": list(candidate.heel.location_m),
            },
            {
                "name": candidate.toe.role,
                "frame": FOOT_FRAME,
                "location": list(candidate.toe.location_m),
            },
        ],
        "contact_rule": {"contact_when": "signed_clearance_le_zero"},
    }


def _platform_check(*, numerical: bool) -> dict[str, Any]:
    receipt = {
        "system": platform.system(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "path_code_platform_neutral": True,
        "numerical_claim": "macOS arm64 only",
    }
    if numerical and (
        platform.system() != "Darwin"
        or platform.machine().lower() not in {"arm64", "aarch64"}
    ):
        raise V25SweepError(
            "V25 numerical execution is fail-closed outside macOS arm64; "
            "Windows DLL/parity has not been attested"
        )
    receipt["numerical_execution_allowed"] = bool(
        platform.system() == "Darwin"
        and platform.machine().lower() in {"arm64", "aarch64"}
    )
    return receipt


def _data_scope_receipt() -> dict[str, Any]:
    groups = {
        "development_allowlist": list(TRIALS),
        "protected_denylist": list(PROTECTED_TRIALS),
        "reserve_denylist": list(RESERVE_TRIALS),
        "historical_denylist": list(HISTORICAL_TRIALS),
    }
    sets = [set(values) for values in groups.values()]
    assertions = {
        "groups_pairwise_disjoint": all(
            sets[left].isdisjoint(sets[right])
            for left in range(len(sets))
            for right in range(left + 1, len(sets))
        ),
        "all_trials_partitioned_exactly_once": set().union(*sets)
        == {f"{index:02d}" for index in range(1, 9)}
        and sum(len(values) for values in sets) == 8,
        "trial_specs_equal_development_allowlist": set(TRIAL_SPECS) == set(TRIALS),
        "protected_closed": not set(TRIAL_SPECS) & set(PROTECTED_TRIALS),
        "reserve_closed": not set(TRIAL_SPECS) & set(RESERVE_TRIALS),
        "historical_closed": not set(TRIAL_SPECS) & set(HISTORICAL_TRIALS),
    }
    if not all(assertions.values()):
        raise V25SweepError(f"V25 data-scope contract failed: {assertions}")
    return {
        **groups,
        "opened_development_trials": list(TRIALS),
        "opened_protected_trials": [],
        "opened_reserve_trials": [],
        "opened_historical_trials": [],
        "protected_trials_remain_closed": True,
        "reserve_trials_remain_closed": True,
        "historical_trial_remains_closed": True,
        "assertions": assertions,
        "pass": True,
    }


def preflight(*, numerical: bool) -> dict[str, Any]:
    domain = {
        "single_sensor_arms_mm": SINGLE_SENSOR_ARMS_MM,
        "conditional_stage2_mm": CONDITIONAL_STAGE2_MM,
        "stage2_condition": "no exactly verified eligible stage-one candidate",
        "selection_rule": SELECTION_RULE,
        "x_and_lateral_geometry_frozen": True,
        "fsm_v20_frozen": True,
        "sample_dt_s": SAMPLE_DT_S,
        "trials": list(TRIALS),
    }
    json.dumps(domain, allow_nan=False)
    return {
        "schema_version": SCHEMA_VERSION,
        "sweep_id": SWEEP_ID,
        "status": "V25_PREFLIGHT_PASS",
        "v24_prerequisite": _verify_v24_prerequisite(),
        "pinned_sources": _verify_pinned_sources(),
        "v25_script_live": _source_record(V25_SCRIPT_PATH),
        "platform": _platform_check(numerical=numerical),
        "data_scope": _data_scope_receipt(),
        "frozen_domain": domain,
    }


def execute(args: argparse.Namespace) -> dict[str, Any]:
    if not args.output_dir:
        raise V25SweepError("--execute requires --output-dir")
    output_dir = Path(args.output_dir).expanduser().resolve()
    allowed_root = (VALIDATION_ROOT / "binary_phase_detector_v25_geometry_runs").resolve()
    try:
        output_dir.relative_to(allowed_root)
    except ValueError as exc:
        raise V25SweepError(
            "output-dir must be under validation/binary_phase_detector_v25_geometry_runs/"
        ) from exc
    started = time.monotonic()
    # Fail before creating any run directory if platform, frozen inputs, V24,
    # or the fixed-domain contract is not ready.
    preflight_receipt = preflight(numerical=True)
    output_dir.mkdir(parents=True, exist_ok=False)
    try:
        _write_json_exclusive(output_dir / "run_start.json", preflight_receipt)

        # Imports occur only after their source hashes and the V24 prerequisite
        # have been verified.
        import opensim
        import sweep_binary_phase_detector_v21_geometry as v21
        import validate_binary_phase_fsm_v20_development as v20
        from audit_two_sensor_prescribed_geometry import (
            _load_stl_triangles,
            _minimum_mesh_distance,
        )
        from binary_phase_detector import load_binary_phase_detector_profile
        from build_two_sensor_mesh_profile_v4 import (
            _section_z_bounds_at_x,
            _vertical_surface_intersections_y,
        )
        from config import SimulatorConfig
        from kinematics_interpolator import KinematicsInterpolator
        from model_loader import _load_plugin

        v20_platform = v20._platform_preflight()
        ledgers = _load_oracles()
        profile = load_binary_phase_detector_profile(PARENT_PROFILE_PATH)
        if (
            tuple(profile.ground.origin) != GROUND_ORIGIN
            or tuple(profile.ground.normal) != GROUND_NORMAL
        ):
            raise V25SweepError("V21 parent ground contract drifted")
        triangles = _load_stl_triangles(REPO_ROOT / "Geometry/AM_foot_l.STL")
        factory = v21.MeshPointFactory(
            triangles,
            section_z_bounds=_section_z_bounds_at_x,
            vertical_y_intersections=_vertical_surface_intersections_y,
        )
        parent = V25Candidate(
            heel=factory.build(
                "left_heel", PARENT_HEEL_X_M, PARENT_HEEL_REACH_MM / 1000.0
            ),
            toe=factory.build(
                "left_toe", PARENT_TOE_X_M, PARENT_TOE_REACH_MM / 1000.0
            ),
            stage="v21_parent_baseline",
            arm_membership=("heel_only", "toe_only"),
        )
        profile_points = {point.name: point for point in profile.points}
        for point in (parent.heel, parent.toe):
            expected = np.asarray(profile_points[point.role].location, dtype=float)
            observed = np.asarray(point.location_m, dtype=float)
            if float(np.max(np.abs(expected - observed))) > 1e-10:
                raise V25SweepError(
                    f"factory does not reconstruct V21 parent {point.role}"
                )

        progress_options = _progress_options(args)
        traces = {
            trial_id: _acquire_affine_trial(
                trial_id,
                v21=v21,
                opensim=opensim,
                SimulatorConfig=SimulatorConfig,
                KinematicsInterpolator=KinematicsInterpolator,
                load_plugin=_load_plugin,
                progress_options=progress_options,
            )
            for trial_id in TRIALS
        }
        _write_json_exclusive(
            output_dir / "trace_manifest.json",
            {
                "schema_version": SCHEMA_VERSION,
                "trials": {
                    trial_id: {
                        "sample_count": int(trace.time_s.size),
                        "start_time_s": float(trace.time_s[0]),
                        "end_time_s": float(trace.time_s[-1]),
                        "source": trace.source,
                    }
                    for trial_id, trace in traces.items()
                },
            },
        )

        bit_cache: dict[
            tuple[str, str, tuple[float, float, float]], np.ndarray
        ] = {}
        parent_geometry = _geometry_gate(
            parent,
            triangles=triangles,
            minimum_mesh_distance=_minimum_mesh_distance,
        )
        parent_screen = _evaluate_candidate(
            parent,
            traces=traces,
            ledgers=ledgers,
            v21=v21,
            v20=v20,
            bit_cache=bit_cache,
            geometry_gate=parent_geometry,
        )
        _write_json_exclusive(
            output_dir / "v21_parent_baseline_diagnostic.json", parent_screen
        )

        stage1_candidates = _generate_candidates(factory, "single_sensor_arms")
        stage1_screens = _evaluate_stage(
            "single-sensor arms",
            stage1_candidates,
            traces=traces,
            ledgers=ledgers,
            v21=v21,
            v20=v20,
            bit_cache=bit_cache,
            triangles=triangles,
            minimum_mesh_distance=_minimum_mesh_distance,
            progress_options=progress_options,
        )
        _write_jsonl_exclusive(
            output_dir / "stage1_candidate_results.jsonl", stage1_screens
        )
        stage1_verifications = _verify_screen_eligible(
            stage1_candidates,
            stage1_screens,
            traces=traces,
            ledgers=ledgers,
            v21=v21,
            v20=v20,
            bit_cache=bit_cache,
            progress_options=progress_options,
        )
        _write_json_exclusive(
            output_dir / "stage1_exact_verification.json",
            {"verifications": stage1_verifications},
        )

        eligible = [item for item in stage1_verifications if item["eligible"]]
        stage2_opened = not eligible
        stage2_screens: list[dict[str, Any]] = []
        stage2_verifications: list[dict[str, Any]] = []
        if stage2_opened:
            stage2_candidates = _generate_candidates(
                factory, "conditional_two_dimensional_cross"
            )
            stage2_screens = _evaluate_stage(
                "conditional two-dimensional cross",
                stage2_candidates,
                traces=traces,
                ledgers=ledgers,
                v21=v21,
                v20=v20,
                bit_cache=bit_cache,
                triangles=triangles,
                minimum_mesh_distance=_minimum_mesh_distance,
                progress_options=progress_options,
            )
            _write_jsonl_exclusive(
                output_dir / "stage2_candidate_results.jsonl", stage2_screens
            )
            stage2_verifications = _verify_screen_eligible(
                stage2_candidates,
                stage2_screens,
                traces=traces,
                ledgers=ledgers,
                v21=v21,
                v20=v20,
                bit_cache=bit_cache,
                progress_options=progress_options,
            )
            _write_json_exclusive(
                output_dir / "stage2_exact_verification.json",
                {"verifications": stage2_verifications},
            )
            eligible = [
                item for item in stage2_verifications if item["eligible"]
            ]

        selected = min(eligible, key=_selection_key) if eligible else None
        selected_profile_record = None
        if selected is not None:
            candidate_map = {
                candidate.candidate_id: candidate
                for candidate in (
                    stage1_candidates
                    + (
                        _generate_candidates(
                            factory, "conditional_two_dimensional_cross"
                        )
                        if stage2_opened
                        else []
                    )
                )
            }
            selected_candidate = candidate_map[str(selected["candidate_id"])]
            profile_path = output_dir / "selected_candidate_profile.json"
            _write_json_exclusive(
                profile_path, _candidate_profile(selected_candidate)
            )
            selected_profile_record = _source_record(profile_path)

        final_script_record = _source_record(V25_SCRIPT_PATH)
        if final_script_record != preflight_receipt["v25_script_live"]:
            raise V25SweepError("V25 script changed during numerical execution")
        elapsed = float(time.monotonic() - started)
        terminal_pass = selected is not None
        decision = {
            "schema_version": SCHEMA_VERSION,
            "sweep_id": SWEEP_ID,
            "status": (
                "PASS_V25_LOCAL_GEOMETRY_DEVELOPMENT"
                if terminal_pass
                else "FAIL_V25_LOCAL_GEOMETRY_DEVELOPMENT_TERMINAL"
            ),
            "pass": terminal_pass,
            "selected_candidate_id": (
                selected["candidate_id"] if selected is not None else None
            ),
            "selected_verification": selected,
            "selected_profile": selected_profile_record,
            "selection_rule": SELECTION_RULE,
            "stage1_candidate_count": len(stage1_candidates),
            "stage1_screen_eligible_count": sum(
                bool(item["screen_eligible"]) for item in stage1_screens
            ),
            "stage1_exact_eligible_count": sum(
                bool(item["eligible"]) for item in stage1_verifications
            ),
            "stage2_opened": stage2_opened,
            "stage2_candidate_count": len(stage2_screens),
            "stage2_exact_eligible_count": sum(
                bool(item["eligible"]) for item in stage2_verifications
            ),
            "elapsed_s": elapsed,
            "data_scope": _data_scope_receipt(),
            "development_trials_opened": list(TRIALS),
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "historical_trials_opened": [],
            "protected_trials_remain_closed": True,
            "reserve_trials_remain_closed": True,
            "historical_trial_01_remains_closed": True,
            "h0_executed": False,
            "runtime_profile_modified": False,
            "training_started": False,
            "corridor_started": False,
            "ppo_updates": 0,
            "v21_v22_v23_artifacts_modified": False,
            "v25_script_live": final_script_record,
            "platform_receipt": v20_platform,
        }
        _write_json_exclusive(
            output_dir / "terminal_decision.json", decision
        )
        manifest = {
            **decision,
            "output_files": {
                path.name: _source_record(path)
                for path in sorted(output_dir.iterdir())
                if path.is_file()
            },
            "strict_json_no_nan_inf": True,
            "atomic_no_clobber": True,
        }
        _write_json_exclusive(output_dir / "manifest.json", manifest)
        return manifest
    except BaseException as exc:
        failure = {
            "schema_version": SCHEMA_VERSION,
            "sweep_id": SWEEP_ID,
            "status": "ERROR_V25_FAIL_CLOSED",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
            "elapsed_s": float(time.monotonic() - started),
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "historical_trials_opened": [],
            "runtime_profile_modified": False,
            "training_started": False,
            "ppo_updates": 0,
        }
        try:
            _write_json_exclusive(output_dir / "failure.json", failure)
        except BaseException:
            pass
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--check",
        action="store_true",
        help="verify frozen sources, V24 PASS prerequisite, and fixed domain",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="run the macOS-only numerical development sweep",
    )
    parser.add_argument(
        "--output-dir",
        help=(
            "new no-clobber directory below "
            "validation/binary_phase_detector_v25_geometry_runs/"
        ),
    )
    parser.add_argument("--progress-width", type=int, default=24)
    parser.add_argument("--progress-interval-s", type=float, default=0.10)
    parser.add_argument("--non-tty-progress-interval-s", type=float, default=30.0)
    parser.add_argument("--no-progress", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.check:
            if args.output_dir:
                raise V25SweepError("--check does not accept --output-dir")
            receipt = preflight(numerical=False)
            print(json.dumps(receipt, indent=2, sort_keys=True, allow_nan=False))
            return 0
        manifest = execute(args)
        print(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False))
        return 0 if manifest["pass"] else 1
    except (V25SweepError, FileExistsError, ValueError) as exc:
        print(f"V25 ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
