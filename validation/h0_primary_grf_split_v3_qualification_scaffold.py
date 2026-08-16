"""Pure, non-executing scaffold for post-holdout H0 V3 qualification.

This module deliberately contains no OpenSim, Ray, Torch, subprocess, or write
path.  It freezes the six qualification *descriptions* and validates the
prerequisites that a future execution driver must consume.  In particular, it
does not authorize qualification from the existence of a candidate alone: a
PASS V3 final holdout and an explicit baseline/tolerance decision receipt are
both mandatory.

The later trainer zero-update port and V25 A/B/C preflight remain separate,
pending stages.  Passing :func:`validate_qualification_prerequisites` therefore
means only ``AUTONOMOUS_QUALIFICATION_INPUTS_READY``; it is not a qualification
PASS and it is not ``CORRIDOR_TRAINING_READY``.
"""

from __future__ import annotations

import hashlib
import json
import math
import numbers
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V3_SEMANTIC_REPLAY"
EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
PHASE_FSM_INPUT_MODE = "legacy_events"
MORPHOLOGY_WEIGHT = 0.0
EXPECTED_STEPS = 500
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
STOCHASTIC_SIGMA = 0.005
CANONICAL_OFFSET_S = 1.956870983805102

RUN_ROOT_RELATIVE = Path(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v3_semantic_replay"
)
HOLDOUT_RECEIPT_RELATIVE = RUN_ROOT_RELATIVE / "holdout" / "receipt.json"
HOLDOUT_GATE_RELATIVE = RUN_ROOT_RELATIVE / "holdout" / "gate.json"
CANDIDATE_FREEZE_RELATIVE = RUN_ROOT_RELATIVE / "adaptation" / "candidate_freeze.json"
DECISION_RECEIPT_RELATIVE = (
    RUN_ROOT_RELATIVE / "qualification" / "baseline_tolerance_decision_receipt.json"
)
NOISE_ROOT_RELATIVE = Path("validation/h0_primary_grf_split_noise_tapes")
NOISE_MANIFEST_RELATIVE = NOISE_ROOT_RELATIVE / "manifest.json"

CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)

ZERO_COUNT_FIELDS = (
    "action_clipped_values",
    "fallback_count",
    "timeout_count",
    "safety_stop_count",
    "hard_invalid_count",
    "nonfinite_count",
)

_EXPECTED_NOISE_MANIFEST_SHA256 = (
    "4f736af3f27fb41c9dfe43bc89032400abce1a45f3941dd52695a11178ae62ce"
)
_EXPECTED_NOISE_MANIFEST_SIZE = 3563
_DETERMINISTIC_TAPE = {
    "path": (
        "validation/h0_primary_grf_split_noise_tapes/"
        "qualification_deterministic_all_zero.npz"
    ),
    "sha256": "41e8d13ec17dfe7e6abc66277c856dd6692a436160ceb3313332c6cf977f7cbe",
    "size_bytes": 4284,
    "array_sha256": (
        "ab89c5ecd7d818ab19f726cffc9ce431f5889448c7a79f84927f7153e546782c"
    ),
}
_STOCHASTIC_TAPES = {
    126: {
        "manifest_key": "02",
        "path": (
            "validation/h0_primary_grf_split_noise_tapes/"
            "qualification_trial_02_standard_normal.npz"
        ),
        "sha256": ("829602b41aa1f355bf8c09d06453f8ba89254c5aeba766c51885fc0a827e7d4d"),
        "size_bytes": 4532,
        "array_sha256": (
            "5a95b863d723205fb1da59c9a57d9d877b8ee9252ce15e1d21e5f30e9f01883e"
        ),
    },
    127: {
        "manifest_key": "04",
        "path": (
            "validation/h0_primary_grf_split_noise_tapes/"
            "qualification_trial_04_standard_normal.npz"
        ),
        "sha256": ("f30794c2ed6b7156224aa62d9869f7b4db076f836e298e19b06d29d6acc4fa74"),
        "size_bytes": 4532,
        "array_sha256": (
            "6f7a90103354e3c9626470f54de159f4d40b5cfa64d16ae686507a5b711b6712"
        ),
    },
    128: {
        "manifest_key": "08",
        "path": (
            "validation/h0_primary_grf_split_noise_tapes/"
            "qualification_trial_08_standard_normal.npz"
        ),
        "sha256": ("4056cbc6cf6652de636505b6d355884a984ba8899b330bceef0c9ad79d62a465"),
        "size_bytes": 4532,
        "array_sha256": (
            "ba3572933c514008a7673f45c5ddd0d7d6833a2a6c7a28f32bb5a860a857548c"
        ),
    },
}

_DECISION_AUTHORITY = {
    "baseline_and_tolerances_explicitly_accepted": True,
    "autonomous_qualification_execution_authorized": True,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "zero_update_port_authorized": False,
    "v25_abc_execution_authorized": False,
    "protected_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
}


class QualificationScaffoldError(RuntimeError):
    """Raised when a post-holdout qualification prerequisite is not exact."""


def _reject_constant(token: str) -> None:
    raise QualificationScaffoldError(f"non-finite JSON constant: {token}")


def _unique_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise QualificationScaffoldError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_constant,
            object_pairs_hook=_unique_pairs,
        )
    except QualificationScaffoldError:
        raise
    except Exception as exc:
        raise QualificationScaffoldError(
            f"cannot read strict JSON {path}: {exc}"
        ) from exc
    if not isinstance(value, Mapping):
        raise QualificationScaffoldError(f"expected JSON object: {path}")
    return dict(value)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise QualificationScaffoldError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _require_bool(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise QualificationScaffoldError(f"{label} must be a JSON boolean")
    return value


def _require_counter(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise QualificationScaffoldError(f"{label} must be a non-negative integer")
    result = int(value)
    if result < 0:
        raise QualificationScaffoldError(f"{label} must be a non-negative integer")
    return result


def _require_finite_nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise QualificationScaffoldError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise QualificationScaffoldError(f"{label} must be finite and non-negative")
    return result


def _require_exact_keys(value: Mapping[str, Any], keys: set[str], label: str) -> None:
    observed = set(value)
    if observed != keys:
        missing = sorted(keys - observed)
        extra = sorted(observed - keys)
        raise QualificationScaffoldError(
            f"{label} schema drifted; missing={missing}, extra={extra}"
        )


def _resolve_repo_path(repo_root: Path, serialized: Any, label: str) -> Path:
    if not isinstance(serialized, str) or not serialized:
        raise QualificationScaffoldError(f"{label} path must be a non-empty string")
    relative = Path(serialized)
    if relative.is_absolute() or ".." in relative.parts:
        raise QualificationScaffoldError(f"{label} path must be repository-relative")
    root = repo_root.resolve()
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise QualificationScaffoldError(f"{label} escaped repository root") from exc
    return resolved


def _verify_artifact_record(
    record: Any,
    *,
    expected: Path | None,
    repo_root: Path,
    label: str,
) -> Path:
    if not isinstance(record, Mapping):
        raise QualificationScaffoldError(f"{label} must be an artifact record")
    _require_exact_keys(record, {"path", "sha256", "size_bytes"}, label)
    path = _resolve_repo_path(repo_root, record["path"], label)
    if expected is not None and path != (repo_root / expected).resolve():
        raise QualificationScaffoldError(f"{label} path drifted")
    digest = record["sha256"]
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise QualificationScaffoldError(f"{label} sha256 is malformed")
    size = _require_counter(record["size_bytes"], f"{label} size_bytes")
    if not path.is_file():
        raise QualificationScaffoldError(f"{label} file is missing: {path}")
    if path.stat().st_size != size or _sha256_file(path) != digest:
        raise QualificationScaffoldError(f"{label} integrity mismatch")
    return path


def _tape_contract(
    *,
    path: str,
    sha256: str,
    size_bytes: int,
    array_sha256: str,
) -> dict[str, Any]:
    return {
        "artifact": {"path": path, "sha256": sha256, "size_bytes": size_bytes},
        "array_sha256": array_sha256,
        "dtype": "float32",
        "shape": [EXPECTED_STEPS, 2],
    }


def canonical_cases() -> tuple[dict[str, Any], ...]:
    """Return fresh descriptions of the six preregistered V3 cases."""

    deterministic_tape = _DETERMINISTIC_TAPE["path"]
    cases: list[dict[str, Any]] = []
    for case_id, delta in zip(CASE_IDS[:3], (-0.2, 0.0, 0.2), strict=True):
        cases.append(
            {
                "case_id": case_id,
                "action_selection": "deterministic",
                "historical_offset_s": CANONICAL_OFFSET_S + delta,
                "offset_delta_s": delta,
                "seed": None,
                "sigma": 0.0,
                "noise_tape": deterministic_tape,
                "expected_steps": EXPECTED_STEPS,
                "event_contract_id": EVENT_CONTRACT_ID,
                "phase_fsm_input_mode": PHASE_FSM_INPUT_MODE,
                "morphology_weight": MORPHOLOGY_WEIGHT,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
            }
        )
    for case_id, seed in zip(CASE_IDS[3:], (126, 127, 128), strict=True):
        cases.append(
            {
                "case_id": case_id,
                "action_selection": "stochastic",
                "historical_offset_s": CANONICAL_OFFSET_S,
                "offset_delta_s": 0.0,
                "seed": seed,
                "sigma": STOCHASTIC_SIGMA,
                "noise_tape": _STOCHASTIC_TAPES[seed]["path"],
                "expected_steps": EXPECTED_STEPS,
                "event_contract_id": EVENT_CONTRACT_ID,
                "phase_fsm_input_mode": PHASE_FSM_INPUT_MODE,
                "morphology_weight": MORPHOLOGY_WEIGHT,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
            }
        )
    return tuple(cases)


def scaffold_manifest() -> dict[str, Any]:
    """Describe pending stages without authorizing or executing any of them."""

    return {
        "schema_version": 1,
        "status": "PENDING_H0_PRIMARY_SPLIT_V3_POST_HOLDOUT_QUALIFICATION",
        "protocol_id": PROTOCOL_ID,
        "scope": "NON_EXECUTING_POST_HOLDOUT_SCAFFOLD",
        "canonical_cases": list(canonical_cases()),
        "fixed_case_gates": {
            "expected_steps": EXPECTED_STEPS,
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "penetration_limit_m": PENETRATION_LIMIT_M,
            "penetration_comparison": "strict_less_than",
            "zero_count_fields": list(ZERO_COUNT_FIELDS),
            "sea_and_reserve_regression": "decision_receipt_required",
        },
        "required_prerequisites": {
            "v3_final_holdout": HOLDOUT_RECEIPT_RELATIVE.as_posix(),
            "baseline_tolerance_decision": DECISION_RECEIPT_RELATIVE.as_posix(),
            "noise_manifest": NOISE_MANIFEST_RELATIVE.as_posix(),
        },
        "authority": {
            "execution_authorized": False,
            "actor_updates_authorized": False,
            "critic_updates_authorized": False,
            "ppo_updates_authorized": False,
            "protected_trial_access_authorized": False,
        },
        "stages": [
            {
                "stage": "autonomous_six_case_qualification",
                "state": "pending_prerequisites",
                "result_required_for_next_stage": (
                    "PASS_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
                ),
            },
            {
                "stage": "trainer_zero_update_port",
                "state": "pending_after_autonomous_qualification",
                "included_in_this_scaffold": False,
            },
            {
                "stage": "v25_abc_preflight",
                "state": "pending_after_zero_update_port",
                "included_in_this_scaffold": False,
            },
        ],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def validate_noise_tapes(repo_root: Path = REPO_ROOT) -> dict[str, Any]:
    """Validate the already-frozen zero tape and seed 126--128 tapes."""

    root = repo_root.resolve()
    manifest_path = root / NOISE_MANIFEST_RELATIVE
    if (
        not manifest_path.is_file()
        or manifest_path.stat().st_size != _EXPECTED_NOISE_MANIFEST_SIZE
        or _sha256_file(manifest_path) != _EXPECTED_NOISE_MANIFEST_SHA256
    ):
        raise QualificationScaffoldError("frozen noise manifest integrity mismatch")
    manifest = _strict_mapping(manifest_path)
    if (
        _require_counter(manifest.get("schema_version"), "noise manifest schema") != 1
        or manifest.get("status") != "H0_PRIMARY_GRF_SPLIT_NOISE_TAPES_FROZEN"
        or manifest.get("protected_trials_opened") != []
        or manifest.get("standard_normal_pre_scaling") is not True
    ):
        raise QualificationScaffoldError("frozen noise manifest header drifted")
    deterministic = manifest.get("deterministic")
    expected_deterministic = _tape_contract(
        path=str(_DETERMINISTIC_TAPE["path"]),
        sha256=str(_DETERMINISTIC_TAPE["sha256"]),
        size_bytes=int(_DETERMINISTIC_TAPE["size_bytes"]),
        array_sha256=str(_DETERMINISTIC_TAPE["array_sha256"]),
    )
    if deterministic != expected_deterministic:
        raise QualificationScaffoldError("deterministic qualification tape drifted")
    _verify_artifact_record(
        deterministic["artifact"],
        expected=Path(_DETERMINISTIC_TAPE["path"]),
        repo_root=root,
        label="deterministic qualification tape",
    )
    tapes = manifest.get("tapes")
    if not isinstance(tapes, Mapping) or set(tapes) != {"collection", "qualification"}:
        raise QualificationScaffoldError("noise tape stage map drifted")
    qualification = tapes.get("qualification")
    if not isinstance(qualification, Mapping) or set(qualification) != {
        "02",
        "04",
        "08",
    }:
        raise QualificationScaffoldError("qualification noise tape keys drifted")
    for seed, expected in _STOCHASTIC_TAPES.items():
        key = str(expected["manifest_key"])
        record = qualification.get(key)
        tape_contract = {
            **_tape_contract(
                path=str(expected["path"]),
                sha256=str(expected["sha256"]),
                size_bytes=int(expected["size_bytes"]),
                array_sha256=str(expected["array_sha256"]),
            ),
            "seed": seed,
        }
        if record != tape_contract:
            raise QualificationScaffoldError(f"qualification seed {seed} tape drifted")
        _verify_artifact_record(
            record["artifact"],
            expected=Path(str(expected["path"])),
            repo_root=root,
            label=f"qualification seed {seed} tape",
        )
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_NOISE_INPUTS",
        "seeds": [126, 127, 128],
        "deterministic_zero_tape": True,
    }


def validate_v3_holdout_pass(repo_root: Path = REPO_ROOT) -> dict[str, Any]:
    """Require an exact PASS V3 holdout and its frozen candidate continuity."""

    root = repo_root.resolve()
    receipt_path = root / HOLDOUT_RECEIPT_RELATIVE
    gate_path = root / HOLDOUT_GATE_RELATIVE
    freeze_path = root / CANDIDATE_FREEZE_RELATIVE
    for path, label in (
        (receipt_path, "V3 holdout receipt"),
        (gate_path, "V3 holdout gate"),
        (freeze_path, "V3 candidate freeze"),
    ):
        if not path.is_file():
            raise QualificationScaffoldError(f"{label} is missing: {path}")
    receipt = _strict_mapping(receipt_path)
    _require_exact_keys(
        receipt,
        {
            "schema_version",
            "status",
            "passed",
            "gate",
            "candidate_freeze",
            "holdout_access_claim",
            "holdout_replay_receipt",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "protected_trials_opened",
        },
        "V3 holdout receipt",
    )
    if (
        _require_counter(receipt["schema_version"], "holdout receipt schema") != 3
        or receipt["status"] != "PASS_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT"
        or _require_bool(receipt["passed"], "holdout receipt passed") is not True
        or _require_counter(receipt["actor_updates"], "holdout actor updates") != 1
        or _require_counter(receipt["critic_updates"], "holdout critic updates") != 0
        or _require_counter(receipt["ppo_updates"], "holdout PPO updates") != 0
        or receipt["protected_trials_opened"] != []
    ):
        raise QualificationScaffoldError("V3 final holdout receipt is not a PASS")
    _verify_artifact_record(
        receipt["gate"],
        expected=HOLDOUT_GATE_RELATIVE,
        repo_root=root,
        label="V3 holdout gate",
    )
    _verify_artifact_record(
        receipt["candidate_freeze"],
        expected=CANDIDATE_FREEZE_RELATIVE,
        repo_root=root,
        label="V3 candidate freeze",
    )
    gate = _strict_mapping(gate_path)
    _require_exact_keys(
        gate,
        {
            "schema_version",
            "status",
            "passed",
            "checks",
            "metrics",
            "candidate_freeze",
            "holdout_access_claim",
            "holdout_replay_receipt",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "protected_trials_opened",
        },
        "V3 holdout gate",
    )
    checks = gate.get("checks")
    if not isinstance(checks, Mapping) or not checks:
        raise QualificationScaffoldError("V3 holdout checks are missing")
    for key, value in checks.items():
        if _require_bool(value, f"holdout check {key}") is not True:
            raise QualificationScaffoldError(f"V3 holdout check failed: {key}")
    for required in (
        "candidate_frozen_before_holdout",
        "candidate_module_unchanged",
        "no_updates_during_holdout",
    ):
        if checks.get(required) is not True:
            raise QualificationScaffoldError(f"V3 holdout lacks check: {required}")
    if (
        _require_counter(gate["schema_version"], "holdout gate schema") != 3
        or gate["status"] != receipt["status"]
        or _require_bool(gate["passed"], "holdout gate passed") is not True
        or _require_counter(gate["actor_updates"], "holdout gate actor updates") != 1
        or _require_counter(gate["critic_updates"], "holdout gate critic updates") != 0
        or _require_counter(gate["ppo_updates"], "holdout gate PPO updates") != 0
        or gate["protected_trials_opened"] != []
        or gate["candidate_freeze"] != receipt["candidate_freeze"]
    ):
        raise QualificationScaffoldError("V3 holdout gate/receipt continuity failed")
    freeze = _strict_mapping(freeze_path)
    if (
        _require_counter(freeze.get("schema_version"), "candidate freeze schema") != 3
        or freeze.get("status") != "H0_PRIMARY_SPLIT_V3_CANDIDATE_FROZEN_BEFORE_HOLDOUT"
        or freeze.get("protocol_id") != PROTOCOL_ID
        or _require_bool(
            freeze.get("holdout_accessed_before_freeze"),
            "candidate freeze prior holdout access",
        )
        is not False
        or _require_counter(
            freeze.get("actor_updates"), "candidate freeze actor updates"
        )
        != 1
        or _require_counter(
            freeze.get("critic_updates"), "candidate freeze critic updates"
        )
        != 0
        or _require_counter(freeze.get("ppo_updates"), "candidate freeze PPO updates")
        != 0
        or freeze.get("protected_trials_opened") != []
    ):
        raise QualificationScaffoldError("V3 candidate freeze is non-canonical")
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_V3_HOLDOUT_PREREQUISITE",
        "receipt_path": HOLDOUT_RECEIPT_RELATIVE.as_posix(),
        "receipt_sha256": _sha256_file(receipt_path),
    }


def _validate_metric_tolerances(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise QualificationScaffoldError(f"{label} must be a non-empty list")
    names: list[str] = []
    for index, item in enumerate(value):
        item_label = f"{label}[{index}]"
        if not isinstance(item, Mapping):
            raise QualificationScaffoldError(f"{item_label} must be an object")
        _require_exact_keys(
            item,
            {"metric", "absolute_tolerance", "relative_tolerance"},
            item_label,
        )
        name = item["metric"]
        if not isinstance(name, str) or not name.strip():
            raise QualificationScaffoldError(f"{item_label}.metric is invalid")
        if name in names:
            raise QualificationScaffoldError(f"duplicate metric in {label}: {name}")
        _require_finite_nonnegative(
            item["absolute_tolerance"], f"{item_label}.absolute_tolerance"
        )
        _require_finite_nonnegative(
            item["relative_tolerance"], f"{item_label}.relative_tolerance"
        )
        names.append(name)
    return tuple(names)


def _validate_baseline_receipt(
    path: Path,
    *,
    baseline_id: str,
    sea_metrics: Sequence[str],
    reserve_metrics: Sequence[str],
) -> None:
    baseline = _strict_mapping(path)
    _require_exact_keys(
        baseline,
        {
            "schema_version",
            "status",
            "passed",
            "baseline_id",
            "case_metrics",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "protected_trials_opened",
        },
        "qualification baseline receipt",
    )
    if (
        _require_counter(baseline["schema_version"], "baseline schema") != 1
        or baseline["status"] != "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_FROZEN"
        or _require_bool(baseline["passed"], "baseline passed") is not True
        or baseline["baseline_id"] != baseline_id
        or _require_counter(baseline["actor_updates"], "baseline actor updates") != 0
        or _require_counter(baseline["critic_updates"], "baseline critic updates") != 0
        or _require_counter(baseline["ppo_updates"], "baseline PPO updates") != 0
        or baseline["protected_trials_opened"] != []
    ):
        raise QualificationScaffoldError("qualification baseline is non-canonical")
    case_metrics = baseline.get("case_metrics")
    if not isinstance(case_metrics, Mapping) or tuple(case_metrics) != CASE_IDS:
        raise QualificationScaffoldError(
            "baseline cases are not the six canonical cases"
        )
    for case_id in CASE_IDS:
        metrics = case_metrics[case_id]
        if not isinstance(metrics, Mapping) or set(metrics) != {"sea", "reserve"}:
            raise QualificationScaffoldError(f"baseline metrics malformed: {case_id}")
        for family, expected_names in (
            ("sea", sea_metrics),
            ("reserve", reserve_metrics),
        ):
            values = metrics[family]
            if not isinstance(values, Mapping) or tuple(values) != tuple(
                expected_names
            ):
                raise QualificationScaffoldError(
                    f"baseline {case_id} {family} metric order/schema drifted"
                )
            for name, value in values.items():
                _require_finite_nonnegative(
                    value, f"baseline {case_id} {family}.{name}"
                )


def validate_baseline_tolerance_decision(
    path: Path | None = None,
    *,
    repo_root: Path = REPO_ROOT,
) -> dict[str, Any]:
    """Validate explicit authority and complete condition-matched thresholds."""

    root = repo_root.resolve()
    decision_path = (
        root / DECISION_RECEIPT_RELATIVE if path is None else Path(path).resolve()
    )
    expected_decision = (root / DECISION_RECEIPT_RELATIVE).resolve()
    if decision_path != expected_decision:
        raise QualificationScaffoldError("decision receipt path is non-canonical")
    if not decision_path.is_file():
        raise QualificationScaffoldError(
            "explicit baseline/tolerance decision receipt is missing"
        )
    decision = _strict_mapping(decision_path)
    _require_exact_keys(
        decision,
        {
            "schema_version",
            "status",
            "passed",
            "protocol_id",
            "decision_authority",
            "candidate_holdout_receipt",
            "baseline",
            "tolerances",
            "fixed_gates",
            "runtime_contract",
            "canonical_case_ids",
            "authority",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "protected_trials_opened",
        },
        "baseline/tolerance decision receipt",
    )
    if (
        _require_counter(decision["schema_version"], "decision schema") != 1
        or decision["status"]
        != "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_TOLERANCE_DECIDED"
        or _require_bool(decision["passed"], "decision passed") is not True
        or decision["protocol_id"] != PROTOCOL_ID
        or decision["decision_authority"] != "EXPLICIT_USER_DECISION"
        or decision["canonical_case_ids"] != list(CASE_IDS)
        or decision["authority"] != _DECISION_AUTHORITY
        or _require_counter(decision["actor_updates"], "decision actor updates") != 0
        or _require_counter(decision["critic_updates"], "decision critic updates") != 0
        or _require_counter(decision["ppo_updates"], "decision PPO updates") != 0
        or decision["protected_trials_opened"] != []
    ):
        raise QualificationScaffoldError("baseline/tolerance decision is non-canonical")
    authority = decision["authority"]
    if not isinstance(authority, Mapping):
        raise QualificationScaffoldError("decision authority must be an object")
    for key, expected in _DECISION_AUTHORITY.items():
        if (
            _require_bool(authority.get(key), f"decision authority {key}")
            is not expected
        ):
            raise QualificationScaffoldError(f"decision authority drifted: {key}")
    _verify_artifact_record(
        decision["candidate_holdout_receipt"],
        expected=HOLDOUT_RECEIPT_RELATIVE,
        repo_root=root,
        label="decision candidate holdout receipt",
    )
    fixed_gates = decision.get("fixed_gates")
    expected_fixed_gates = {
        "expected_steps": EXPECTED_STEPS,
        "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
        "penetration_limit_m": PENETRATION_LIMIT_M,
        "penetration_comparison": "strict_less_than",
        "zero_count_fields": list(ZERO_COUNT_FIELDS),
    }
    if fixed_gates != expected_fixed_gates:
        raise QualificationScaffoldError("fixed qualification gates drifted")
    if not isinstance(fixed_gates, Mapping):
        raise QualificationScaffoldError("fixed qualification gates must be an object")
    if (
        _require_counter(fixed_gates["expected_steps"], "fixed expected steps")
        != EXPECTED_STEPS
        or _require_counter(
            fixed_gates["minimum_valid_cycles"], "fixed minimum valid cycles"
        )
        != MINIMUM_VALID_CYCLES
        or _require_finite_nonnegative(
            fixed_gates["penetration_limit_m"], "fixed penetration limit"
        )
        != PENETRATION_LIMIT_M
    ):
        raise QualificationScaffoldError("fixed qualification gate types drifted")
    runtime = decision.get("runtime_contract")
    if runtime != {
        "event_contract_id": EVENT_CONTRACT_ID,
        "phase_fsm_input_mode": PHASE_FSM_INPUT_MODE,
        "morphology_weight": MORPHOLOGY_WEIGHT,
    }:
        raise QualificationScaffoldError("qualification runtime contract drifted")
    if (
        not isinstance(runtime, Mapping)
        or isinstance(runtime["morphology_weight"], bool)
        or not isinstance(runtime["morphology_weight"], numbers.Real)
        or float(runtime["morphology_weight"]) != 0.0
    ):
        raise QualificationScaffoldError("qualification runtime types drifted")
    tolerances = decision.get("tolerances")
    if not isinstance(tolerances, Mapping):
        raise QualificationScaffoldError("tolerances must be an object")
    _require_exact_keys(
        tolerances,
        {"comparison_formula", "sea", "reserve"},
        "qualification tolerances",
    )
    if tolerances["comparison_formula"] != (
        "candidate <= baseline + max(absolute_tolerance, "
        "relative_tolerance * abs(baseline))"
    ):
        raise QualificationScaffoldError("regression tolerance formula drifted")
    sea_metrics = _validate_metric_tolerances(tolerances["sea"], "SEA tolerances")
    reserve_metrics = _validate_metric_tolerances(
        tolerances["reserve"], "reserve tolerances"
    )
    baseline = decision.get("baseline")
    if not isinstance(baseline, Mapping):
        raise QualificationScaffoldError("baseline decision must be an object")
    _require_exact_keys(
        baseline,
        {"baseline_id", "comparison_scope", "receipt"},
        "qualification baseline decision",
    )
    baseline_id = baseline["baseline_id"]
    if not isinstance(baseline_id, str) or not baseline_id.strip():
        raise QualificationScaffoldError("baseline_id must be explicit")
    if baseline["comparison_scope"] != "condition_matched_six_cases":
        raise QualificationScaffoldError("baseline must be condition-matched")
    baseline_path = _verify_artifact_record(
        baseline["receipt"],
        expected=None,
        repo_root=root,
        label="qualification baseline receipt",
    )
    if baseline_path == decision_path:
        raise QualificationScaffoldError("decision cannot be its own baseline receipt")
    _validate_baseline_receipt(
        baseline_path,
        baseline_id=baseline_id,
        sea_metrics=sea_metrics,
        reserve_metrics=reserve_metrics,
    )
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_DECISION_PREREQUISITE",
        "baseline_id": baseline_id,
        "sea_metrics": list(sea_metrics),
        "reserve_metrics": list(reserve_metrics),
    }


def validate_qualification_prerequisites(
    *,
    repo_root: Path = REPO_ROOT,
    decision_path: Path | None = None,
) -> dict[str, Any]:
    """Fail closed unless all inputs for a future six-case run are exact."""

    noise = validate_noise_tapes(repo_root)
    holdout = validate_v3_holdout_pass(repo_root)
    decision = validate_baseline_tolerance_decision(
        decision_path,
        repo_root=repo_root,
    )
    return {
        "schema_version": 1,
        "status": "AUTONOMOUS_QUALIFICATION_INPUTS_READY",
        "passed": True,
        "qualification_executed": False,
        "canonical_case_ids": list(CASE_IDS),
        "noise_prerequisite": noise,
        "holdout_prerequisite": holdout,
        "decision_prerequisite": decision,
        "runtime_contract": {
            "event_contract_id": EVENT_CONTRACT_ID,
            "phase_fsm_input_mode": PHASE_FSM_INPUT_MODE,
            "morphology_weight": MORPHOLOGY_WEIGHT,
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "later_stages": {
            "trainer_zero_update_port": "PENDING_AFTER_QUALIFICATION_PASS",
            "v25_abc_preflight": "PENDING_AFTER_ZERO_UPDATE_PASS",
        },
    }


__all__ = [
    "CASE_IDS",
    "DECISION_RECEIPT_RELATIVE",
    "EVENT_CONTRACT_ID",
    "QualificationScaffoldError",
    "canonical_cases",
    "scaffold_manifest",
    "validate_baseline_tolerance_decision",
    "validate_noise_tapes",
    "validate_qualification_prerequisites",
    "validate_v3_holdout_pass",
]
