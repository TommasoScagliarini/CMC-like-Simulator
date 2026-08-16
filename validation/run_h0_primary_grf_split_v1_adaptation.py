"""One-shot H0 adaptation to ``primary_grf_split_v1``.

The driver has separate collection, supervised-update, and closed-loop worker
modes.  Execution is denied unless the corresponding no-clobber lock freezes
the exact source and destinations.  Protected trials and PPO updates are never
available through this module.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_v25_abc as strict_io  # noqa: E402
import primary_grf_split_adaptation as split_contract  # noqa: E402
import run_h0_v25_abc_preflight as h0_runtime  # noqa: E402


PROTOCOL_LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v1_adaptation_protocol_lock.json"
COLLECTION_LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v1_collection_execution_unlock.json"
ADAPTATION_LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v1_actor_update_unlock.json"
# Backward-compatible internal alias used only by collection helpers.
EXECUTION_LOCK = COLLECTION_LOCK
QUALIFICATION_LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v1_qualification_lock.json"
RUN_ROOT = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-05_h0_primary_grf_split_v1_one_shot"
)
H0_RUN = (
    VALIDATION_ROOT
    / "critic_warmup"
    / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
)
H0_MODULE = H0_RUN / "rl_module_last"
H0_CONFIG = H0_RUN / "training_cfg.resolved.yaml"
ACTOR_LAYOUT_REFERENCE = H0_RUN / "actor_transplant_report.json"
FULL_LAYOUT_REFERENCE = Path(h0_runtime.FULL_LAYOUT_REFERENCE).resolve()
INPUT_ROOT = VALIDATION_ROOT / "h0_primary_grf_split_inputs"
INPUT_MANIFEST = INPUT_ROOT / "manifest.json"
NOISE_ROOT = VALIDATION_ROOT / "h0_primary_grf_split_noise_tapes"
NOISE_MANIFEST = NOISE_ROOT / "manifest.json"
SETUP_ROOT = (
    VALIDATION_ROOT
    / "primary_grf_runs"
    / "2026-07-24_ab06_primary_grf_readiness_v1"
    / "inputs"
)
EXPECTED_STEPS = 500
EXPECTED_SIGMA = 0.005
COLLECTION_SIGMA = 0.0025
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
WORKER_TIMEOUT_S = 1800.0
EVENT_CONTRACT = "primary_grf_split_v1+legacy_events_v1"

TRIALS: dict[str, dict[str, Any]] = {
    "02": {
        "speed_mps": 0.95,
        "setup": SETUP_ROOT / "trial_02_development_setup.xml",
        "collection_offset_s": 107.880,
        "collection_absolute_s": 119.578,
        "collection_seed": 123,
        "qualification_offset_s": 117.880,
        "qualification_absolute_s": 129.578,
        "qualification_seed": 126,
    },
    "04": {
        "speed_mps": 1.05,
        "setup": SETUP_ROOT / "trial_04_development_setup.xml",
        "collection_offset_s": 107.550,
        "collection_absolute_s": 122.189,
        "collection_seed": 124,
        "qualification_offset_s": 117.550,
        "qualification_absolute_s": 132.189,
        "qualification_seed": 127,
    },
    "08": {
        "speed_mps": 1.25,
        "setup": SETUP_ROOT / "trial_08_development_setup.xml",
        "collection_offset_s": 106.878,
        "collection_absolute_s": 120.390,
        "collection_seed": 125,
        "qualification_offset_s": 116.878,
        "qualification_absolute_s": 130.390,
        "qualification_seed": 128,
    },
}


class PrimarySplitExecutionError(RuntimeError):
    pass


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    return {
        "path": str(resolved.relative_to(REPO_ROOT)),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    value = strict_io.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise PrimarySplitExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise PrimarySplitExecutionError("non-finite JSON value")
        return value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist())
    if hasattr(value, "item"):
        return _jsonable(value.item())
    raise PrimarySplitExecutionError(
        f"unsupported JSON value {type(value).__name__}"
    )


def _write_npz_exclusive(path: Path, **arrays: Any) -> None:
    import numpy as np

    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        raise PrimarySplitExecutionError(f"refusing to clobber: {path}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise PrimarySplitExecutionError(f"refusing to clobber: {path}") from exc
        temporary.unlink()
    finally:
        if temporary.exists():
            temporary.unlink()


def _claim_empty_destination(path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    if not destination.is_dir() or any(destination.iterdir()):
        raise PrimarySplitExecutionError(
            f"destination must be a preallocated empty directory: {destination}"
        )
    return destination


def _frozen_path_record(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping):
        raise PrimarySplitExecutionError(f"{label} record is malformed")
    relative = record.get("path")
    if not isinstance(relative, str) or not relative:
        raise PrimarySplitExecutionError(f"{label} path is malformed")
    path = (REPO_ROOT / relative).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise PrimarySplitExecutionError(
            f"{label} escapes the repository"
        ) from exc
    if record.get("sha256") != sha256_file(path):
        raise PrimarySplitExecutionError(f"{label} hash drifted")
    if "size_bytes" in record:
        size = record.get("size_bytes")
        if type(size) is not int or size < 0 or size != path.stat().st_size:
            raise PrimarySplitExecutionError(f"{label} size drifted")
    return path


def _verify_lock(path: Path, *, qualification: bool = False) -> dict[str, Any]:
    lock = _strict_mapping(path)
    if path.resolve() == COLLECTION_LOCK.resolve():
        expected_status = "H0_PRIMARY_GRF_SPLIT_V1_COLLECTION_EXECUTION_UNLOCKED"
        required_true = "h0_primary_split_teacher_collection_authorized"
    elif path.resolve() == ADAPTATION_LOCK.resolve():
        expected_status = "H0_PRIMARY_GRF_SPLIT_V1_ACTOR_UPDATE_UNLOCKED"
        required_true = "h0_primary_split_supervised_adaptation_execution_authorized"
    elif path.resolve() == QUALIFICATION_LOCK.resolve():
        expected_status = "H0_PRIMARY_GRF_SPLIT_V1_QUALIFICATION_UNLOCKED"
        required_true = "h0_primary_split_zero_update_qualification_authorized"
    else:
        raise PrimarySplitExecutionError(f"unknown execution lock: {path}")
    if lock.get("status") != expected_status:
        raise PrimarySplitExecutionError(f"lock status mismatch: {path}")
    authority = lock.get("authority")
    if not isinstance(authority, Mapping):
        raise PrimarySplitExecutionError("lock authority is malformed")
    if authority.get(required_true) is not True:
        raise PrimarySplitExecutionError(f"authority {required_true} is closed")
    if qualification and path.resolve() != QUALIFICATION_LOCK.resolve():
        raise PrimarySplitExecutionError(
            "qualification verification requires the qualification lock"
        )
    actor_updates_expected = path.resolve() == ADAPTATION_LOCK.resolve()
    if authority.get("actor_updates_authorized") is not actor_updates_expected:
        raise PrimarySplitExecutionError("stage actor-update authority drifted")
    mandatory_false = (
        "critic_updates_authorized",
        "general_training_authorized",
        "h0_sep_authorized",
        "h0_v25_abc_execution_authorized",
        "v25_ab_c_execution_authorized",
        "ppo_updates_authorized",
        "protected_trial_access_authorized",
        "reserve_trial_access_authorized",
        "detector_retuning_authorized",
        "primary_grf_modification_authorized",
        "sea_semantic_modification_authorized",
        "corridor_authorized",
        "runtime_promotion_authorized",
    )
    for forbidden in mandatory_false:
        if authority.get(forbidden) is not False:
            raise PrimarySplitExecutionError(f"forbidden authority opened: {forbidden}")
    allowed_true = {required_true}
    if actor_updates_expected:
        allowed_true.add("actor_updates_authorized")
    for key, value in authority.items():
        if not isinstance(value, bool):
            raise PrimarySplitExecutionError(
                f"authority {key} is not boolean"
            )
        if key.endswith("_authorized") and value and key not in allowed_true:
            raise PrimarySplitExecutionError(
                f"unexpected authority opened: {key}"
            )
    for section in ("frozen_sources", "frozen_inputs", "preflight_receipts"):
        values = lock.get(section)
        if not isinstance(values, Mapping):
            raise PrimarySplitExecutionError(f"lock {section} is malformed")
        if section != "preflight_receipts" or path.resolve() != QUALIFICATION_LOCK.resolve():
            if not values:
                raise PrimarySplitExecutionError(f"lock {section} is empty")
        for key, record in values.items():
            if not isinstance(record, Mapping) or type(record.get("size_bytes")) is not int:
                raise PrimarySplitExecutionError(
                    f"{section}.{key} lacks a frozen byte size"
                )
            _frozen_path_record(record, f"{section}.{key}")
    if path.resolve() == QUALIFICATION_LOCK.resolve() or qualification:
        candidate = lock.get("candidate_module")
        if not isinstance(candidate, Mapping):
            raise PrimarySplitExecutionError("qualification candidate is malformed")
        expected_candidate_keys = {
            "module_state",
            "class_and_ctor_args",
            "metadata",
        }
        if set(candidate) != expected_candidate_keys:
            raise PrimarySplitExecutionError(
                "qualification candidate artifact set drifted"
            )
        if any(
            not isinstance(record, Mapping)
            or type(record.get("size_bytes")) is not int
            for record in candidate.values()
        ):
            raise PrimarySplitExecutionError(
                "qualification candidate lacks frozen artifact sizes"
            )
        candidate_paths = {
            key: _frozen_path_record(record, f"candidate_module.{key}")
            for key, record in candidate.items()
        }
        if len({value.parent for value in candidate_paths.values()}) != 1:
            raise PrimarySplitExecutionError(
                "qualification candidate artifacts do not share one checkpoint"
            )
    return lock


def preregistered_rollout_provenance(
    summary: Mapping[str, Any],
    *,
    trial_id: str,
    collection: bool,
    behavior_role: str,
    selection: str,
    noise_tape_sha256: str,
) -> dict[str, Any]:
    """Gate one rollout against its frozen stage/window, not merely its pair."""

    trial = TRIALS[trial_id]
    expected_offset = float(
        trial["collection_offset_s" if collection else "qualification_offset_s"]
    )
    expected_absolute = float(
        trial["collection_absolute_s" if collection else "qualification_absolute_s"]
    )
    expected_seed = int(
        trial["collection_seed" if collection else "qualification_seed"]
    )
    expected_sigma = (
        COLLECTION_SIGMA
        if collection
        else (EXPECTED_SIGMA if selection == "stochastic" else 0.0)
    )
    expected_teacher_dynamics = behavior_role == "teacher"
    checks: list[dict[str, Any]] = []

    def add(name: str, actual: Any, expected: Any, passed: bool) -> None:
        checks.append(
            {
                "name": name,
                "actual": actual,
                "expected": expected,
                "status": "PASS" if passed else "FAIL",
            }
        )

    add("trial_id", summary.get("trial_id"), trial_id, summary.get("trial_id") == trial_id)
    add("plateau_id", summary.get("plateau_id"), "04", summary.get("plateau_id") == "04")
    add(
        "behavior_role",
        summary.get("behavior_role"),
        behavior_role,
        summary.get("behavior_role") == behavior_role,
    )
    add(
        "action_selection",
        summary.get("action_selection"),
        selection,
        summary.get("action_selection") == selection,
    )
    add("seed", summary.get("seed"), expected_seed, summary.get("seed") == expected_seed)
    add(
        "episode_start_offset_s",
        summary.get("episode_start_offset_s"),
        expected_offset,
        isinstance(summary.get("episode_start_offset_s"), (int, float))
        and abs(float(summary["episode_start_offset_s"]) - expected_offset) <= 1.0e-12,
    )
    add(
        "episode_start_time_s",
        summary.get("episode_start_time_s"),
        expected_absolute,
        isinstance(summary.get("episode_start_time_s"), (int, float))
        and abs(float(summary["episode_start_time_s"]) - expected_absolute) <= 1.0e-9,
    )
    add(
        "sigma",
        summary.get("sigma"),
        expected_sigma,
        isinstance(summary.get("sigma"), (int, float))
        and float(summary["sigma"]) == expected_sigma,
    )
    add(
        "noise_tape_sha256",
        summary.get("noise_tape_sha256"),
        noise_tape_sha256,
        summary.get("noise_tape_sha256") == noise_tape_sha256,
    )
    add(
        "event_contract_id",
        summary.get("event_contract_id"),
        EVENT_CONTRACT,
        summary.get("event_contract_id") == EVENT_CONTRACT,
    )
    add(
        "binary_phase_fsm_mode",
        summary.get("binary_phase_fsm_mode"),
        "disabled",
        summary.get("binary_phase_fsm_mode") == "disabled",
    )
    add(
        "grf_mode",
        summary.get("grf_mode"),
        "online_sensor",
        summary.get("grf_mode") == "online_sensor",
    )
    add(
        "online_grf_applied_sides",
        summary.get("online_grf_applied_sides"),
        ["left"],
        summary.get("online_grf_applied_sides") == ["left"],
    )
    add(
        "teacher_action_drove_environment",
        summary.get("teacher_action_drove_environment"),
        expected_teacher_dynamics,
        summary.get("teacher_action_drove_environment")
        is expected_teacher_dynamics,
    )
    add("ppo_updates", summary.get("ppo_updates"), 0, summary.get("ppo_updates") == 0)
    add(
        "protected_trials_opened",
        summary.get("protected_trials_opened"),
        [],
        summary.get("protected_trials_opened") == [],
    )
    passed = all(item["status"] == "PASS" for item in checks)
    return {
        "status": "PASS_PREREGISTERED_ROLLOUT_PROVENANCE"
        if passed
        else "FAIL_PREREGISTERED_ROLLOUT_PROVENANCE",
        "passed": passed,
        "stage": "collection" if collection else "qualification",
        "checks": checks,
    }


def build_env_config(trial_id: str, *, offset_s: float, seed: int) -> dict[str, Any]:
    if trial_id not in TRIALS:
        raise PrimarySplitExecutionError(f"unknown development trial {trial_id}")
    trial = TRIALS[trial_id]
    condition = {
        "id": f"primary_split_trial{trial_id}",
        "action_selection": "deterministic",
        "offset_s": float(offset_s),
        "seed": int(seed),
    }
    config = h0_runtime.build_env_config(case_id="A", condition=condition)
    config.update(
        {
            "setup_xml_path": str(trial["setup"]),
            "episode_start_offset_s": float(offset_s),
            "online_grf_profile_file": str(
                INPUT_ROOT / f"trial_{trial_id}_primary_surface_velocity.json"
            ),
            "online_grf_detector_profile_file": str(
                INPUT_ROOT / f"trial_{trial_id}_analog_surface_velocity.json"
            ),
            "binary_phase_detector_profile_file": None,
            "binary_phase_fsm_mode": "disabled",
            "binary_phase_event_contract_id": "binary_events_disabled_v1",
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": EVENT_CONTRACT,
            "record_outputs": False,
            "save_outputs_on_close": False,
        }
    )
    config["reward"] = dict(config["reward"])
    config["reward"]["morphology_weight"] = 0.0
    return config


def _load_stack():
    rollout_eval, np, torch, RLModule, env_factory, _reward = (
        h0_runtime._load_inference_stack()
    )
    return rollout_eval, np, torch, RLModule, env_factory


def _load_noise_tape(*, stage: str, trial_id: str, selection: str):
    import numpy as np

    manifest = _strict_mapping(NOISE_MANIFEST)
    if manifest.get("status") != "H0_PRIMARY_GRF_SPLIT_NOISE_TAPES_FROZEN":
        raise PrimarySplitExecutionError("noise-tape manifest is not frozen")
    if selection == "deterministic":
        record = manifest.get("deterministic")
    else:
        record = manifest.get("tapes", {}).get(stage, {}).get(trial_id)
    if not isinstance(record, Mapping):
        raise PrimarySplitExecutionError("noise-tape record is missing")
    artifact = record.get("artifact")
    path = _frozen_path_record(artifact, "noise tape")
    with np.load(path, allow_pickle=False) as archive:
        tape = np.asarray(archive["standard_normal"], dtype=np.float32).copy()
    if tape.shape != (EXPECTED_STEPS, split_contract.ACTION_DIM):
        raise PrimarySplitExecutionError("noise-tape shape drifted")
    if record.get("dtype") != "float32" or record.get("array_sha256") != split_contract.array_sha256(tape):
        raise PrimarySplitExecutionError("noise-tape dtype or array hash drifted")
    if selection == "deterministic" and np.count_nonzero(tape) != 0:
        raise PrimarySplitExecutionError("deterministic tape is not all-zero")
    return tape


def _policy(module: Any, observation: Any, action_shape: tuple[int, ...], rollout_eval: Any):
    raw, mean, std, _noise = rollout_eval._policy_action_with_diagnostics(
        module,
        observation,
        action_shape,
        action_selection="deterministic",
    )
    return raw, mean, std


def _teacher_full_observation(observation: Any, paired: Any, np: Any):
    result = np.asarray(observation, dtype=np.float32).copy()
    result[:EXPECTED_ACTOR_FEATURES] = paired.teacher
    return result


def _update_shadow_fsm(
    shadow_fsm: Any,
    *,
    info: Mapping[str, Any],
    body_weight_n: float,
) -> dict[str, Any]:
    detector = info.get("online_grf_detector")
    if not isinstance(detector, Mapping) or not isinstance(detector.get("left"), Mapping):
        raise PrimarySplitExecutionError("analog detector aggregate is missing")
    left = detector["left"]
    normal = float(left.get("normal_force"))
    contact = left.get("in_contact")
    if not math.isfinite(normal) or normal < 0.0 or type(contact) is not bool:
        raise PrimarySplitExecutionError("analog detector aggregate is malformed")
    events = info.get("legacy_online_events")
    if not isinstance(events, Sequence) or isinstance(events, (str, bytes, bytearray)):
        raise PrimarySplitExecutionError("legacy event stream is malformed")
    state_observation = info.get("observation")
    if not isinstance(state_observation, Mapping):
        raise PrimarySplitExecutionError("exact runtime observation is missing")
    knee = float(state_observation.get("pros_knee_angle"))
    ankle = float(state_observation.get("pros_ankle_angle"))
    if not math.isfinite(knee) or not math.isfinite(ankle):
        raise PrimarySplitExecutionError("exact runtime joint state is malformed")
    return shadow_fsm.update(
        time_s=float(info["time"]),
        events=[dict(event) for event in events],
        normal_force_bw=normal / float(body_weight_n),
        in_contact=contact,
        prosthetic_knee_angle_rad=knee,
        prosthetic_ankle_angle_rad=ankle,
    )


def _validate_reset_contract(
    *, module: Any, env: Any, observation: Any, actor_names: Sequence[str], full_names: Sequence[str]
) -> None:
    import numpy as np

    vector = np.asarray(observation)
    actor_reference = _strict_mapping(ACTOR_LAYOUT_REFERENCE)
    full_reference = _strict_mapping(FULL_LAYOUT_REFERENCE)
    expected_actor_names = tuple(
        str(name) for name in actor_reference.get("target_actor_feature_names", ())
    )
    expected_full_names = tuple(
        str(name) for name in full_reference.get("observation_feature_names", ())
    )
    if (
        vector.shape != (EXPECTED_FULL_FEATURES,)
        or vector.dtype != np.dtype("float32")
        or len(actor_names) != EXPECTED_ACTOR_FEATURES
        or len(full_names) != EXPECTED_FULL_FEATURES
        or tuple(actor_names) != expected_actor_names
        or tuple(full_names) != expected_full_names
        or tuple(env.action_space.shape) != (2,)
        or getattr(env.action_space, "dtype", None) != np.dtype("float32")
        or int(getattr(module, "_n_actor", -1)) != EXPECTED_ACTOR_FEATURES
        or int(getattr(module, "_n_full", -1)) != EXPECTED_FULL_FEATURES
    ):
        raise PrimarySplitExecutionError(
            "runtime/checkpoint layout is not the exact frozen 35/84 float32 contract"
        )
    if any("binary_phase" in str(name) for name in full_names):
        raise PrimarySplitExecutionError("binary detector leaked into observation")


def layout_preflight(output: str | Path) -> dict[str, Any]:
    protocol = _strict_mapping(PROTOCOL_LOCK)
    if (
        protocol.get("status")
        != "H0_PRIMARY_GRF_SPLIT_V1_ADAPTATION_PROTOCOL_FROZEN_EXECUTION_AND_UPDATES_NOT_AUTHORIZED"
        or protocol.get("protocol_executed") is not False
        or protocol.get("authority", {}).get("actor_updates_authorized") is not False
    ):
        raise PrimarySplitExecutionError("protocol-only lock is not authoritative")
    rollout_eval, np, torch, RLModule, env_factory = _load_stack()
    module = RLModule.from_checkpoint(H0_MODULE.resolve())
    cases: dict[str, Any] = {}
    for trial_id, trial in TRIALS.items():
        np.random.seed(int(trial["collection_seed"]))
        torch.manual_seed(int(trial["collection_seed"]))
        config = build_env_config(
            trial_id,
            offset_s=float(trial["collection_offset_s"]),
            seed=int(trial["collection_seed"]),
        )
        env = env_factory.make_cmc_env(config)
        try:
            observation, info = env.reset(seed=int(trial["collection_seed"]))
            observation = np.asarray(observation, dtype=np.float32)
            base = env.unwrapped
            actor_names = tuple(str(name) for name in base.actor_feature_names)
            full_names = tuple(str(name) for name in base.observation_feature_names)
            rollout_eval._validate_module_observation_contract(
                module, actor_names, full_names
            )
            _validate_reset_contract(
                module=module,
                env=env,
                observation=observation,
                actor_names=actor_names,
                full_names=full_names,
            )
            shadow = copy.deepcopy(base._phase_fsm)
            paired = split_contract.build_paired_views(
                observation,
                actor_names,
                info,
                body_weight_n=float(base._body_weight_n),
                reset_boundary=True,
                teacher_phase_observation=shadow.observation(),
            )
            if paired.student.tobytes() != paired.teacher.tobytes():
                raise PrimarySplitExecutionError("reset teacher/student views differ")
            if list(base.cfg.online_grf_applied_sides) != ["left"]:
                raise PrimarySplitExecutionError("primary routing is not left-only")
            if not base.ctx.online_grf_detector_force_paths:
                raise PrimarySplitExecutionError("analog detector contacts are absent")
            cases[trial_id] = {
                "absolute_start_s": float(info["time"]),
                "expected_absolute_start_s": float(trial["collection_absolute_s"]),
                "body_weight_n": float(base._body_weight_n),
                "actor_feature_names": list(actor_names),
                "observation_feature_names": list(full_names),
                "primary_force_count": len(base.ctx.online_grf_force_paths),
                "detector_force_count": len(base.ctx.online_grf_detector_force_paths),
                "surface_velocity_mps": [0.0, 0.0, float(trial["speed_mps"])],
            }
            if abs(cases[trial_id]["absolute_start_s"] - float(trial["collection_absolute_s"])) > 1e-9:
                raise PrimarySplitExecutionError("development start time mismatch")
        finally:
            env.close()
    payload = {
        "schema_version": 1,
        "status": "PASS_H0_PRIMARY_GRF_SPLIT_LAYOUT_PREFLIGHT",
        "passed": True,
        "cases": cases,
        "protocol_lock": source_record(PROTOCOL_LOCK),
        "execution_driver": source_record(Path(__file__).resolve()),
        "actor_layout_reference": source_record(ACTOR_LAYOUT_REFERENCE),
        "full_layout_reference": source_record(FULL_LAYOUT_REFERENCE),
        "h0_executed": False,
        "actor_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(output, payload)
    return payload


def _run_rollout(
    *,
    trial_id: str,
    offset_s: float,
    seed: int,
    selection: str,
    behavior_role: str,
    output_dir: Path,
    candidate_module_path: Path | None = None,
    collection: bool,
) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory = _load_stack()
    np.random.seed(seed)
    torch.manual_seed(seed)
    h0_module = RLModule.from_checkpoint(H0_MODULE.resolve())
    candidate_module = (
        None
        if candidate_module_path is None
        else RLModule.from_checkpoint(candidate_module_path.resolve())
    )
    if behavior_role == "candidate" and candidate_module is None:
        raise PrimarySplitExecutionError("candidate rollout has no candidate module")
    if behavior_role == "teacher" and candidate_module is not None:
        raise PrimarySplitExecutionError("teacher rollout cannot load candidate")
    env_config = build_env_config(trial_id, offset_s=offset_s, seed=seed)
    env = env_factory.make_cmc_env(env_config)
    action_shape = tuple(int(value) for value in env.action_space.shape)
    noise_tape = _load_noise_tape(
        stage="collection" if collection else "qualification",
        trial_id=trial_id,
        selection=selection,
    )
    student_views: list[Any] = []
    teacher_views: list[Any] = []
    teacher_means: list[Any] = []
    served_means: list[Any] = []
    served_actions: list[Any] = []
    primary_loads: list[float] = []
    detector_loads: list[float] = []
    current_phase_rows: list[Any] = []
    teacher_phase_rows: list[Any] = []
    trace: list[dict[str, Any]] = []
    reserve = h0_runtime._empty_accumulator()
    residual = h0_runtime._empty_accumulator()
    sea = h0_runtime._sea_accumulators()
    clipping_values = 0
    timeout_count = 0
    fallback_count = 0
    hard_invalid_count = 0
    invalid_event_count = 0
    served_action_teacher_dependency_count = 0
    candidate_action_selected_before_teacher_diagnostic_count = 0
    penetrations: list[float] = []
    terminated = False
    truncated = False
    info: dict[str, Any] = {}
    episode_start_time_s = float("nan")
    started = time.monotonic()
    try:
        observation, current_info = env.reset(seed=seed)
        episode_start_time_s = float(current_info["time"])
        expected_start_time_s = float(
            TRIALS[trial_id][
                "collection_absolute_s" if collection else "qualification_absolute_s"
            ]
        )
        if abs(episode_start_time_s - expected_start_time_s) > 1.0e-9:
            raise PrimarySplitExecutionError(
                "rollout absolute start drifted from the preregistered window"
            )
        observation = np.asarray(observation, dtype=np.float32)
        base = env.unwrapped
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        full_names = tuple(str(name) for name in base.observation_feature_names)
        rollout_eval._validate_module_observation_contract(
            h0_module, actor_names, full_names
        )
        if candidate_module is not None:
            rollout_eval._validate_module_observation_contract(
                candidate_module, actor_names, full_names
            )
        _validate_reset_contract(
            module=h0_module,
            env=env,
            observation=observation,
            actor_names=actor_names,
            full_names=full_names,
        )
        shadow_fsm = copy.deepcopy(base._phase_fsm)
        body_weight_n = float(base._body_weight_n)
        for step_index in range(EXPECTED_STEPS):
            candidate_policy_input = None
            served_mean = None
            served_std = None
            raw_action = None
            applied = None
            if behavior_role == "candidate":
                # Causal ordering is intentional: choose the autonomous action
                # from the runtime/student observation before constructing or
                # evaluating any counterfactual teacher view.
                candidate_policy_input = np.asarray(
                    observation, dtype=np.float32
                ).copy()
                _raw_candidate, served_mean, served_std = _policy(
                    candidate_module,
                    candidate_policy_input,
                    action_shape,
                    rollout_eval,
                )
                served_mean = np.asarray(served_mean, dtype=np.float32).reshape(-1)
                served_std = np.asarray(served_std, dtype=np.float32).reshape(-1)
                if not np.allclose(
                    served_std, EXPECTED_SIGMA, rtol=0.0, atol=1e-8
                ):
                    raise PrimarySplitExecutionError(
                        "candidate logstd drifted from the frozen value"
                    )
                if selection == "stochastic":
                    raw_action = (
                        served_mean
                        + served_std * noise_tape[step_index]
                    )
                elif selection == "deterministic":
                    raw_action = served_mean.copy()
                else:
                    raise PrimarySplitExecutionError(
                        f"unknown action selection {selection}"
                    )
                raw_action = np.asarray(raw_action, dtype=np.float32).reshape(
                    action_shape
                )
                applied = np.clip(
                    raw_action, env.action_space.low, env.action_space.high
                ).astype(np.float32)
                candidate_action_selected_before_teacher_diagnostic_count += 1

            paired = split_contract.build_paired_views(
                observation,
                actor_names,
                current_info,
                body_weight_n=body_weight_n,
                reset_boundary=step_index == 0,
                teacher_phase_observation=shadow_fsm.observation(),
            )
            teacher_full = _teacher_full_observation(observation, paired, np)
            _raw_h0, teacher_mean, h0_std = _policy(
                h0_module, teacher_full, action_shape, rollout_eval
            )
            teacher_mean = np.asarray(teacher_mean, dtype=np.float32).reshape(-1)
            h0_std = np.asarray(h0_std, dtype=np.float32).reshape(-1)
            if not np.allclose(h0_std, EXPECTED_SIGMA, rtol=0.0, atol=1e-8):
                raise PrimarySplitExecutionError("H0 logstd drifted")
            if behavior_role == "candidate":
                if candidate_policy_input is None or served_mean is None:
                    raise PrimarySplitExecutionError(
                        "candidate action was not selected causally"
                    )
                if (
                    candidate_policy_input[:EXPECTED_ACTOR_FEATURES].tobytes()
                    != paired.student.tobytes()
                ):
                    served_action_teacher_dependency_count += 1
                if served_std.tobytes() != h0_std.tobytes():
                    raise PrimarySplitExecutionError("candidate logstd differs from H0")
            else:
                served_mean = teacher_mean.copy()
                served_std = h0_std
                if selection == "stochastic":
                    sigma = COLLECTION_SIGMA if collection else served_std
                    raw_action = served_mean + np.asarray(
                        sigma, dtype=np.float32
                    ) * noise_tape[step_index]
                elif selection == "deterministic":
                    raw_action = served_mean.copy()
                else:
                    raise PrimarySplitExecutionError(
                        f"unknown action selection {selection}"
                    )
                raw_action = np.asarray(raw_action, dtype=np.float32).reshape(
                    action_shape
                )
                applied = np.clip(
                    raw_action, env.action_space.low, env.action_space.high
                ).astype(np.float32)
            if raw_action is None or applied is None or served_mean is None:
                raise PrimarySplitExecutionError("action selection is incomplete")
            clipping_values += int(np.count_nonzero(applied != raw_action))
            student_views.append(paired.student.copy())
            teacher_views.append(paired.teacher.copy())
            teacher_means.append(teacher_mean.copy())
            served_means.append(served_mean.copy())
            served_actions.append(raw_action.reshape(-1).copy())
            primary_loads.append(paired.primary_load_bw)
            detector_loads.append(paired.detector_load_bw)
            current_phase_rows.append(
                paired.student[list(split_contract.PHASE_FEATURE_INDICES)].copy()
            )
            teacher_phase_rows.append(
                paired.teacher[list(split_contract.PHASE_FEATURE_INDICES)].copy()
            )

            observation_after, reward, terminated, truncated, info = env.step(raw_action)
            observation_after = np.asarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise PrimarySplitExecutionError("environment info is malformed")
            if info.get("event_contract_id") != EVENT_CONTRACT:
                raise PrimarySplitExecutionError("runtime event contract drifted")
            if info.get("online_grf_applied_sides") != ["left"]:
                raise PrimarySplitExecutionError("runtime primary routing drifted")
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise PrimarySplitExecutionError("reward terms are missing")
            h0_runtime._accumulate_scalar(reserve, reward_terms["reserve_norm_nm"])
            h0_runtime._accumulate_scalar(residual, reward_terms["residual_norm_nm"])
            penetration = float(reward_terms["grf_penetration_m"])
            if not math.isfinite(penetration) or penetration < 0.0:
                raise PrimarySplitExecutionError("GRF penetration is malformed")
            penetrations.append(penetration)
            h0_runtime._accumulate_sea(sea, info.get("sea_segment_diagnostics"))
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise PrimarySplitExecutionError("phase FSM payload is missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count, int(float(phase.get("invalid_event_count", 0.0)))
            )
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or "solver_fallback_used" not in so:
                raise PrimarySplitExecutionError("SO fallback diagnostic is missing")
            fallback_count += int(so["solver_fallback_used"] is True)
            hard_invalid_count += int("failure" in info)
            _update_shadow_fsm(
                shadow_fsm,
                info=info,
                body_weight_n=body_weight_n,
            )
            trace.append(
                {
                    "step": step_index + 1,
                    "time_s": float(info["time"]),
                    "teacher_mean": teacher_mean.tolist(),
                    "served_mean": served_mean.tolist(),
                    "raw_action": raw_action.reshape(-1).tolist(),
                    "standard_normal": (
                        noise_tape[step_index].tolist()
                        if selection == "stochastic"
                        else None
                    ),
                    "primary_load_bw": paired.primary_load_bw,
                    "detector_load_bw": paired.detector_load_bw,
                    "teacher_used_for_action": behavior_role == "teacher",
                    "teacher_used_for_dynamics": behavior_role == "teacher",
                    "teacher_action_drove_environment": behavior_role == "teacher",
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                }
            )
            observation = observation_after
            current_info = dict(info)
            completed = step_index + 1
            if completed == 1 or completed % 25 == 0 or completed == EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (EXPECTED_STEPS - completed)
                print(
                    f"[{behavior_role}/trial{trial_id}/{selection}] "
                    f"{completed:3d}/{EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = h0_runtime._finalize_sea(sea)
    fallback_count += sum(
        int(sea[joint]["fallback_count"]) for joint in h0_runtime.comparator.JOINTS
    )
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    penetration_max = max(penetrations, default=0.0)
    student_array = np.ascontiguousarray(student_views, dtype=np.float32)
    teacher_array = np.ascontiguousarray(teacher_views, dtype=np.float32)
    teacher_mean_array = np.ascontiguousarray(teacher_means, dtype=np.float32)
    served_mean_array = np.ascontiguousarray(served_means, dtype=np.float32)
    counterfactual = split_contract.prediction_metrics(
        served_mean_array, teacher_mean_array
    )
    sigma = (
        COLLECTION_SIGMA
        if collection and selection == "stochastic"
        else (EXPECTED_SIGMA if selection == "stochastic" else 0.0)
    )
    noise_tape_sha256 = split_contract.array_sha256(noise_tape)
    end_reason = info.get("end_reason") if isinstance(info, Mapping) else None
    summary = {
        "schema_version": 1,
        "condition_id": f"trial_{trial_id}_{selection}",
        "trial_id": trial_id,
        "plateau_id": "04",
        "behavior_role": behavior_role,
        "action_selection": selection,
        "seed": seed,
        "episode_start_time_s": episode_start_time_s,
        "episode_start_offset_s": offset_s,
        "sigma": sigma,
        "noise_tape_sha256": noise_tape_sha256,
        "steps": len(trace),
        "end_reason": end_reason,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "invalid_event_count": invalid_event_count,
        "grf_penetration_max_m": penetration_max,
        "action_clipped_values": clipping_values,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "fallback_count": fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": 0,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "action_shape": list(action_shape),
        "action_dtype": "float32",
        "morphology_weight": float(env_config["reward"]["morphology_weight"]),
        "episode_metrics": {
            "reserve_norm_nm": h0_runtime._finalize_accumulator(reserve),
            "residual_norm_nm": h0_runtime._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "counterfactual_teacher": counterfactual,
        "counterfactual_teacher_error": {
            "sample_count": int(counterfactual["samples"]),
            "rmse": float(counterfactual["rmse"]),
            "max_abs_error": float(counterfactual["max_abs_error"]),
            "served_action_teacher_dependency_count": (
                served_action_teacher_dependency_count
            ),
            "action_selected_before_teacher_diagnostic": (
                behavior_role != "candidate"
                or candidate_action_selected_before_teacher_diagnostic_count
                == len(trace)
            ),
        },
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "event_contract_id": EVENT_CONTRACT,
        "binary_phase_fsm_mode": "disabled",
        "grf_mode": str(info.get("grf_mode", "")),
        "online_grf_applied_sides": list(
            info.get("online_grf_applied_sides", [])
        ),
        "teacher_used_for_dynamics": behavior_role == "teacher",
        "teacher_action_drove_environment": behavior_role == "teacher",
        "teacher_used_for_candidate_action": False,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    provenance = preregistered_rollout_provenance(
        summary,
        trial_id=trial_id,
        collection=collection,
        behavior_role=behavior_role,
        selection=selection,
        noise_tape_sha256=noise_tape_sha256,
    )
    summary["preregistered_provenance"] = provenance
    if not provenance["passed"]:
        raise PrimarySplitExecutionError(
            "rollout preregistered provenance gate failed"
        )
    arrays = {
        "student_views": student_array,
        "teacher_views": teacher_array,
        "teacher_means": teacher_mean_array,
        "served_means": served_mean_array,
        "served_actions": np.ascontiguousarray(served_actions, dtype=np.float32),
        "primary_load_bw": np.asarray(primary_loads, dtype=np.float32),
        "detector_load_bw": np.asarray(detector_loads, dtype=np.float32),
        "student_phase_fsm": np.ascontiguousarray(current_phase_rows, dtype=np.float32),
        "teacher_phase_fsm": np.ascontiguousarray(teacher_phase_rows, dtype=np.float32),
        "actor_feature_names": np.asarray(actor_names, dtype="U64"),
    }
    if collection:
        data_path = output_dir / "paired_data.npz"
        _write_npz_exclusive(data_path, **arrays)
    else:
        data_path = output_dir / "rollout_arrays.npz"
        _write_npz_exclusive(data_path, **arrays)
    trace_path = strict_io.write_json_exclusive(output_dir / "trace.json", trace)
    summary_path = strict_io.write_json_exclusive(output_dir / "summary.json", summary)
    receipt = {
        "schema_version": 1,
        "status": "ROLLOUT_COMPLETE_PENDING_GATE",
        "passed": None,
        "trial_id": trial_id,
        "behavior_role": behavior_role,
        "action_selection": selection,
        "artifacts": {
            "arrays": source_record(data_path),
            "trace": source_record(trace_path),
            "summary": source_record(summary_path),
        },
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(output_dir / "receipt.json", receipt)
    return summary


def collect_worker(trial_id: str, output_dir: str | Path) -> dict[str, Any]:
    _verify_lock(EXECUTION_LOCK, qualification=False)
    destination = _claim_empty_destination(output_dir)
    expected = (RUN_ROOT / "collection" / f"trial_{trial_id}").resolve()
    if destination != expected:
        raise PrimarySplitExecutionError("collection destination drifted")
    trial = TRIALS[trial_id]
    return _run_rollout(
        trial_id=trial_id,
        offset_s=float(trial["collection_offset_s"]),
        seed=int(trial["collection_seed"]),
        selection="stochastic",
        behavior_role="teacher",
        output_dir=destination,
        collection=True,
    )


def _load_collection(trial_id: str) -> dict[str, Any]:
    import numpy as np

    path = RUN_ROOT / "collection" / f"trial_{trial_id}" / "paired_data.npz"
    terminal_receipt = _strict_mapping(path.parent / "terminal_receipt.json")
    if (
        terminal_receipt.get("passed") is not True
        or terminal_receipt.get("status") != "PASS_COMMON_ROLLOUT"
    ):
        raise PrimarySplitExecutionError(
            f"trial {trial_id} collection gate has not passed"
        )
    summary_path = _frozen_path_record(
        terminal_receipt.get("summary"), f"trial {trial_id} gated summary"
    )
    gate_path = _frozen_path_record(
        terminal_receipt.get("common_gate"), f"trial {trial_id} common gate"
    )
    import compare_h0_primary_grf_split as comparator

    stored_gate = _strict_mapping(gate_path)
    recomputed_gate = comparator.common_rollout_gate(
        _strict_mapping(summary_path)
    )
    if comparator.payload_sha256(stored_gate) != comparator.payload_sha256(
        recomputed_gate
    ):
        raise PrimarySplitExecutionError(
            f"trial {trial_id} common gate is not reproducible"
        )
    receipt = _strict_mapping(path.parent / "receipt.json")
    record = receipt.get("artifacts", {}).get("arrays")
    frozen_arrays_path = _frozen_path_record(
        record, f"trial {trial_id} paired arrays"
    )
    if frozen_arrays_path != path.resolve():
        raise PrimarySplitExecutionError(
            f"trial {trial_id} paired-array path drifted"
        )
    if summary_path != (path.parent / "summary.json").resolve():
        raise PrimarySplitExecutionError(
            f"trial {trial_id} gated-summary path drifted"
        )
    if gate_path != (path.parent / "common_gate.json").resolve():
        raise PrimarySplitExecutionError(
            f"trial {trial_id} common-gate path drifted"
        )
    if (
        stored_gate.get("status") != "PASS_COMMON_ROLLOUT"
        or stored_gate.get("passed") is not True
        or recomputed_gate.get("status") != "PASS_COMMON_ROLLOUT"
        or recomputed_gate.get("passed") is not True
    ):
        raise PrimarySplitExecutionError(
            f"trial {trial_id} common gate is not a PASS"
        )
    with np.load(path, allow_pickle=False) as archive:
        result = {name: archive[name].copy() for name in archive.files}
    result["trial_id"] = trial_id
    return result


def finalize_corpus(output_dir: str | Path) -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    destination = _claim_empty_destination(output_dir)
    if destination != (RUN_ROOT / "corpus").resolve():
        raise PrimarySplitExecutionError("corpus destination drifted")
    groups = [_load_collection(trial) for trial in ("02", "04", "08")]
    dataset, training_indices, validation_indices, dataset_summary = (
        split_contract.assemble_grouped_dataset(
            groups,
            training_trials=("02", "04"),
            validation_trials=("08",),
        )
    )
    if (
        dataset_summary["training_records"] != 2000
        or dataset_summary["validation_records"] != 1000
    ):
        raise PrimarySplitExecutionError("preregistered dataset cardinality drifted")
    dataset_path = destination / "paired_dataset.npz"
    _write_npz_exclusive(
        dataset_path,
        **dataset,
        training_indices=training_indices,
        validation_indices=validation_indices,
    )
    manifest_path = strict_io.write_json_exclusive(
        destination / "manifest.json",
        {
            "schema_version": 1,
            "status": "PRIMARY_SPLIT_PAIRED_CORPUS_MATERIALIZED_NO_UPDATE",
            "summary": dataset_summary,
            "artifact": source_record(dataset_path),
            "collection_terminal_receipts": {
                trial: source_record(
                    RUN_ROOT / "collection" / f"trial_{trial}" / "terminal_receipt.json"
                )
                for trial in ("02", "04", "08")
            },
            "actor_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        },
    )
    receipt = {
        "schema_version": 1,
        "status": "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN",
        "passed": True,
        "dataset": source_record(dataset_path),
        "manifest": source_record(manifest_path),
        "training_indices_sha256": dataset_summary["training_indices_sha256"],
        "validation_indices_sha256": dataset_summary["validation_indices_sha256"],
        "actor_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _load_frozen_corpus():
    import numpy as np

    receipt_path = RUN_ROOT / "corpus" / "receipt.json"
    receipt = _strict_mapping(receipt_path)
    if (
        receipt.get("status") != "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN"
        or receipt.get("passed") is not True
        or receipt.get("actor_updates") != 0
    ):
        raise PrimarySplitExecutionError("paired corpus receipt is not a PASS")
    dataset_path = _frozen_path_record(receipt.get("dataset"), "paired corpus")
    with np.load(dataset_path, allow_pickle=False) as archive:
        arrays = {name: archive[name].copy() for name in archive.files}
    required = {
        "observations",
        "actions",
        "actor_feature_names",
        "trial_ids",
        "view_roles",
        "group_ids",
        "training_indices",
        "validation_indices",
    }
    if set(arrays) != required:
        raise PrimarySplitExecutionError("paired corpus NPZ schema drifted")
    training_indices = np.asarray(arrays.pop("training_indices"), dtype=np.int64)
    validation_indices = np.asarray(arrays.pop("validation_indices"), dtype=np.int64)
    return arrays, training_indices, validation_indices, receipt_path


def adapt_worker(output_dir: str | Path) -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    _verify_lock(ADAPTATION_LOCK)
    destination = _claim_empty_destination(output_dir)
    if destination != (RUN_ROOT / "adaptation").resolve():
        raise PrimarySplitExecutionError("adaptation destination drifted")
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule
    import target_domain_imitation

    dataset, training_indices, validation_indices, dataset_receipt_path = (
        _load_frozen_corpus()
    )
    report = target_domain_imitation.adapt_actor(
        H0_MODULE.resolve(),
        dataset,
        destination,
        seed=123,
        epochs=300,
        batch_size=128,
        learning_rate=1.0e-4,
        validation_fraction=1.0 / 3.0,
        patience=60,
        clip_weight=1.0,
        logstd_weight=0.0,
        anchor_weight=1.0e-3,
        freeze_logstd_head=True,
        trainable_first_layer_features=None,
        first_layer_feature_scales=None,
        training_indices=training_indices,
        validation_indices=validation_indices,
    )
    candidate_path = Path(report["output_module"]).resolve()
    source_module = RLModule.from_checkpoint(H0_MODULE.resolve())
    candidate_module = RLModule.from_checkpoint(candidate_path)
    observations = torch.as_tensor(dataset["observations"], dtype=torch.float32)
    with torch.no_grad():
        source_logits = source_module.pi(observations).cpu().numpy().astype(np.float32)
        candidate_logits = candidate_module.pi(observations).cpu().numpy().astype(np.float32)
    gate = split_contract.offline_adaptation_gate(
        source_predictions=source_logits[:, :2],
        adapted_predictions=candidate_logits[:, :2],
        targets=dataset["actions"],
        validation_indices=validation_indices,
        view_roles=dataset["view_roles"],
        all_adapted_logits=candidate_logits,
    )
    audit_checks = {
        "offline_metrics": bool(gate["passed"]),
        "logstd_parameter_exact": report["logstd_head_max_abs_parameter_change"] == 0.0,
        "logstd_output_exact": report["logstd_max_abs_change_on_dataset"] == 0.0,
        "save_reload_exact": report["save_reload"]["exact"] is True,
        "non_actor_exact": (
            report["non_actor_verification"] == "not_available_in_inference_only_rl_module"
            or report["non_actor_unchanged"]["exact"] is True
        ),
        "actor_digest_changed": report["source_actor_digest"] != report["adapted_actor_digest"],
        "explicit_group_split": report["data_split"]["mode"] == "explicit_indices",
        "ppo_updates_zero": report["ppo_updates"] == 0,
    }
    passed = all(audit_checks.values())
    gate_payload = {
        "schema_version": 1,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
        ),
        "passed": passed,
        "metrics_gate": gate,
        "audit_checks": audit_checks,
        "dataset_receipt": source_record(dataset_receipt_path),
        "candidate_module": {
            name: source_record(candidate_path / name)
            for name in ("module_state.pkl", "class_and_ctor_args.pkl", "metadata.json")
        },
        "adaptation_report": source_record(destination / "adaptation_report.json"),
        "adaptation_history": source_record(destination / "adaptation_history.json"),
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = strict_io.write_json_exclusive(destination / "offline_gate.json", gate_payload)
    receipt = {
        "schema_version": 1,
        "status": gate_payload["status"],
        "passed": passed,
        "offline_gate": source_record(gate_path),
        "candidate_path": str(candidate_path.relative_to(REPO_ROOT)),
        "candidate_actor_digest": report["adapted_actor_digest"],
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise PrimarySplitExecutionError("offline adaptation gate failed")
    return receipt


def qualification_worker(
    *, role: str, trial_id: str, selection: str, output_dir: str | Path
) -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    lock = _verify_lock(QUALIFICATION_LOCK, qualification=True)
    destination = _claim_empty_destination(output_dir)
    expected = (RUN_ROOT / "qualification" / role / f"trial_{trial_id}_{selection}").resolve()
    if destination != expected:
        raise PrimarySplitExecutionError("qualification destination drifted")
    trial = TRIALS[trial_id]
    candidate_record = lock.get("candidate_module", {}).get("module_state")
    candidate_module = None
    if role == "candidate":
        for reference_trial in ("02", "04", "08"):
            for reference_selection in ("deterministic", "stochastic"):
                reference_dir = (
                    RUN_ROOT
                    / "qualification"
                    / "reference"
                    / f"trial_{reference_trial}_{reference_selection}"
                )
                predecessor = _strict_mapping(reference_dir / "common_gate.json")
                if (
                    predecessor.get("passed") is not True
                    or predecessor.get("status") != "PASS_COMMON_ROLLOUT"
                ):
                    raise PrimarySplitExecutionError(
                        "all six teacher references must pass before any "
                        "candidate rollout"
                    )
                import compare_h0_primary_grf_split as comparator

                recomputed = comparator.common_rollout_gate(
                    _strict_mapping(reference_dir / "summary.json")
                )
                if comparator.payload_sha256(
                    predecessor
                ) != comparator.payload_sha256(recomputed):
                    raise PrimarySplitExecutionError(
                        "teacher reference gate is not reproducible"
                    )
        module_state = _frozen_path_record(candidate_record, "candidate module state")
        candidate_module = module_state.parent
    elif role != "reference":
        raise PrimarySplitExecutionError(f"invalid qualification role {role}")
    return _run_rollout(
        trial_id=trial_id,
        offset_s=float(trial["qualification_offset_s"]),
        seed=int(trial["qualification_seed"]),
        selection=selection,
        behavior_role="teacher" if role == "reference" else "candidate",
        output_dir=destination,
        candidate_module_path=candidate_module,
        collection=False,
    )


def _worker_command(*arguments: str) -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), *arguments]


def _parallel(commands: Sequence[list[str]]) -> None:
    processes = [subprocess.Popen(command, cwd=REPO_ROOT) for command in commands]
    deadline = time.monotonic() + WORKER_TIMEOUT_S
    failure: str | None = None
    try:
        while True:
            returncodes = [process.poll() for process in processes]
            failed = [code for code in returncodes if code not in (None, 0)]
            if failed:
                failure = f"worker failures: {failed}"
                break
            if all(code == 0 for code in returncodes):
                return
            if time.monotonic() >= deadline:
                failure = "worker stage timeout"
                break
            time.sleep(0.2)
    finally:
        if failure is not None:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
            for process in processes:
                if process.poll() is None:
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.0)
    if failure is not None:
        raise PrimarySplitExecutionError(failure)


def execute_collection() -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    started = time.time()
    status = "ERROR_PRIMARY_SPLIT_TEACHER"
    error = None
    try:
        commands = [
            _worker_command(
                "--collect-worker",
                "--trial",
                trial,
                "--output-dir",
                str(RUN_ROOT / "collection" / f"trial_{trial}"),
            )
            for trial in ("02", "04", "08")
        ]
        _parallel(commands)
        # Scientific common gates are applied by the supervisor before update.
        import compare_h0_primary_grf_split as comparator

        for trial in ("02", "04", "08"):
            directory = RUN_ROOT / "collection" / f"trial_{trial}"
            summary = _strict_mapping(directory / "summary.json")
            gate = comparator.common_rollout_gate(summary)
            gate_path = strict_io.write_json_exclusive(
                directory / "common_gate.json", gate
            )
            receipt = _strict_mapping(directory / "receipt.json")
            receipt["status"] = gate["status"]
            receipt["passed"] = gate["passed"]
            receipt["summary"] = source_record(directory / "summary.json")
            receipt["common_gate"] = source_record(gate_path)
            # Replace the provisional receipt atomically via a new terminal file;
            # the original remains immutable evidence of worker completion.
            strict_io.write_json_exclusive(directory / "terminal_receipt.json", receipt)
            if not gate["passed"]:
                raise PrimarySplitExecutionError(f"teacher collection trial {trial} failed")
        status = "ERROR_PRIMARY_SPLIT_CORPUS_FREEZE"
        command = _worker_command(
            "--finalize-corpus", "--output-dir", str(RUN_ROOT / "corpus")
        )
        completed = subprocess.run(
            command, cwd=REPO_ROOT, timeout=WORKER_TIMEOUT_S, check=False
        )
        if completed.returncode != 0:
            raise PrimarySplitExecutionError(
                f"corpus finalizer exited {completed.returncode}"
            )
        status = "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN"
        passed = True
    except Exception as exc:
        passed = False
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 1,
        "status": status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "collection_lock": source_record(COLLECTION_LOCK),
        "protocol_lock": source_record(PROTOCOL_LOCK),
        "collection_trials_completed": [
            trial
            for trial in ("02", "04", "08")
            if (RUN_ROOT / "collection" / f"trial_{trial}" / "summary.json").is_file()
        ],
        "corpus_frozen": (RUN_ROOT / "corpus" / "receipt.json").is_file(),
        "actor_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "FREEZE_SEPARATE_ACTOR_UPDATE_UNLOCK"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    strict_io.write_json_exclusive(RUN_ROOT / "collection_execution_ledger.json", ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise PrimarySplitExecutionError(error or status)
    return ledger


def execute_adaptation() -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    _verify_lock(ADAPTATION_LOCK)
    collection_ledger = _strict_mapping(RUN_ROOT / "collection_execution_ledger.json")
    if (
        collection_ledger.get("status")
        != "PASS_PRIMARY_SPLIT_TEACHER_CORPUS_FROZEN"
        or collection_ledger.get("passed") is not True
        or collection_ledger.get("actor_updates") != 0
    ):
        raise PrimarySplitExecutionError("teacher corpus stage has not passed")
    started = time.time()
    status = "FAIL_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
    error = None
    try:
        completed = subprocess.run(
            _worker_command(
                "--adapt-worker", "--output-dir", str(RUN_ROOT / "adaptation")
            ),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise PrimarySplitExecutionError(
                f"adaptation worker exited {completed.returncode}"
            )
        status = "PASS_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION"
        passed = True
    except Exception as exc:
        passed = False
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 1,
        "status": status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "adaptation_lock": source_record(ADAPTATION_LOCK),
        "collection_ledger": source_record(
            RUN_ROOT / "collection_execution_ledger.json"
        ),
        "candidate_created": (RUN_ROOT / "adaptation" / "receipt.json").is_file(),
        "actor_update_candidates": 1 if (RUN_ROOT / "adaptation" / "adaptation_report.json").is_file() else 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "FREEZE_ZERO_UPDATE_QUALIFICATION"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    strict_io.write_json_exclusive(
        RUN_ROOT / "adaptation_execution_ledger.json", ledger
    )
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise PrimarySplitExecutionError(error or status)
    return ledger


def execute_qualification() -> dict[str, Any]:
    _verify_lock(COLLECTION_LOCK)
    _verify_lock(QUALIFICATION_LOCK, qualification=True)
    import compare_h0_primary_grf_split as comparator

    started = time.time()
    status = "ERROR_H0_PRIMARY_SPLIT_REFERENCE"
    error = None
    gates: dict[str, Any] = {}
    try:
        reference_commands = []
        for trial in ("02", "04", "08"):
            for selection in ("deterministic", "stochastic"):
                reference_commands.append(
                    _worker_command(
                        "--qualification-worker",
                        "--role",
                        "reference",
                        "--trial",
                        trial,
                        "--selection",
                        selection,
                        "--output-dir",
                        str(RUN_ROOT / "qualification" / "reference" / f"trial_{trial}_{selection}"),
                    )
                )
        _parallel(reference_commands)
        for trial in ("02", "04", "08"):
            for selection in ("deterministic", "stochastic"):
                summary = _strict_mapping(
                    RUN_ROOT / "qualification" / "reference" / f"trial_{trial}_{selection}" / "summary.json"
                )
                gate = comparator.common_rollout_gate(summary)
                strict_io.write_json_exclusive(
                    RUN_ROOT
                    / "qualification"
                    / "reference"
                    / f"trial_{trial}_{selection}"
                    / "common_gate.json",
                    gate,
                )
                recomputed = comparator.common_rollout_gate(summary)
                if comparator.payload_sha256(gate) != comparator.payload_sha256(
                    recomputed
                ):
                    raise PrimarySplitExecutionError(
                        f"reference gate {trial}/{selection} is not reproducible"
                    )
                if not gate["passed"]:
                    raise PrimarySplitExecutionError(
                        f"reference trial {trial}/{selection} failed"
                    )
        status = "FAIL_H0_PRIMARY_SPLIT_CLOSED_LOOP"
        candidate_commands = []
        for trial in ("02", "04", "08"):
            for selection in ("deterministic", "stochastic"):
                candidate_commands.append(
                    _worker_command(
                        "--qualification-worker",
                        "--role",
                        "candidate",
                        "--trial",
                        trial,
                        "--selection",
                        selection,
                        "--output-dir",
                        str(RUN_ROOT / "qualification" / "candidate" / f"trial_{trial}_{selection}"),
                    )
                )
        _parallel(candidate_commands)
        for trial in ("02", "04", "08"):
            for selection in ("deterministic", "stochastic"):
                key = f"trial_{trial}_{selection}"
                reference = _strict_mapping(
                    RUN_ROOT / "qualification" / "reference" / key / "summary.json"
                )
                candidate = _strict_mapping(
                    RUN_ROOT / "qualification" / "candidate" / key / "summary.json"
                )
                gate = comparator.condition_matched_gate(reference, candidate)
                strict_io.write_json_exclusive(
                    RUN_ROOT / "qualification" / "gates" / f"{key}.json", gate
                )
                gates[key] = gate
                if not gate["passed"]:
                    raise PrimarySplitExecutionError(f"candidate {key} failed")
        status = "PASS_H0_PRIMARY_SPLIT_CLOSED_LOOP"
        passed = True
    except Exception as exc:
        passed = False
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 1,
        "status": status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "qualification_lock": source_record(QUALIFICATION_LOCK),
        "condition_gates": gates,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "ZERO_ITER_FRESH_CRITIC_PORT"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    strict_io.write_json_exclusive(RUN_ROOT / "qualification_execution_ledger.json", ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise PrimarySplitExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--layout-preflight", action="store_true")
    mode.add_argument("--collect-worker", action="store_true")
    mode.add_argument("--finalize-corpus", action="store_true")
    mode.add_argument("--adapt-worker", action="store_true")
    mode.add_argument("--qualification-worker", action="store_true")
    mode.add_argument("--execute-collection", action="store_true")
    mode.add_argument("--execute-adaptation", action="store_true")
    mode.add_argument("--execute-qualification", action="store_true")
    parser.add_argument("--output")
    parser.add_argument("--output-dir")
    parser.add_argument("--trial", choices=tuple(TRIALS))
    parser.add_argument("--role", choices=("reference", "candidate"))
    parser.add_argument("--selection", choices=("deterministic", "stochastic"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.layout_preflight:
            if not args.output:
                raise PrimarySplitExecutionError("layout preflight requires --output")
            result = layout_preflight(args.output)
        elif args.collect_worker:
            if not args.trial or not args.output_dir:
                raise PrimarySplitExecutionError("collect worker arguments are incomplete")
            result = collect_worker(args.trial, args.output_dir)
        elif args.adapt_worker:
            if not args.output_dir:
                raise PrimarySplitExecutionError("adapt worker requires --output-dir")
            result = adapt_worker(args.output_dir)
        elif args.finalize_corpus:
            if not args.output_dir:
                raise PrimarySplitExecutionError("corpus finalizer requires --output-dir")
            result = finalize_corpus(args.output_dir)
        elif args.qualification_worker:
            if not all((args.role, args.trial, args.selection, args.output_dir)):
                raise PrimarySplitExecutionError("qualification worker arguments are incomplete")
            result = qualification_worker(
                role=args.role,
                trial_id=args.trial,
                selection=args.selection,
                output_dir=args.output_dir,
            )
        elif args.execute_collection:
            result = execute_collection()
        elif args.execute_adaptation:
            result = execute_adaptation()
        else:
            result = execute_qualification()
        if not (
            args.execute_collection
            or args.execute_adaptation
            or args.execute_qualification
        ):
            print(json.dumps(_jsonable(result), indent=2, sort_keys=True, allow_nan=False))
        return 0
    except Exception as exc:
        if args.output_dir:
            failure_path = Path(args.output_dir).expanduser().resolve() / "failure.json"
            if not failure_path.exists():
                try:
                    strict_io.write_json_exclusive(
                        failure_path,
                        {
                            "status": "FAIL_CLOSED",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                            "ppo_updates": 0,
                            "protected_trials_opened": [],
                        },
                    )
                except Exception:
                    pass
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
