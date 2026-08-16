"""Execute the frozen V12R3-P1 independent qualification exactly once.

The supervisor verifies the post-salvage protocol freeze, publishes a separate
no-clobber execution lock, and then consumes thirteen ordered stages: all six
original-H0 baselines, all six exact P1/V26 candidates, and one aggregate.
Every rollout is persisted before its pure common gate; every candidate is then
paired only with its condition-matched baseline.  Any failure after the run
root is claimed is terminal and produces a one-way ledger.  Importing this
module performs no environment import, model load, write, or rollout.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import os
import secrets
import stat
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12R3_ROOT = LOCAL_VALIDATION / "v12r3"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    ROOT_VALIDATION,
    LOCAL_VALIDATION,
    V12R3_ROOT,
    V12P1Q_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r3_p1_qualification_protocol as protocol_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402
import h0_v12r3_p1_qualification_gates as gates  # noqa: E402
import prepare_h0_v12r3_p1_qualification_noise_tapes as noise  # noqa: E402


class V12R3P1QualificationExecutionError(RuntimeError):
    """Raised when continuing would violate the frozen one-shot protocol."""


LOCK_PASS_STATUS = contract.EXECUTION_LOCK_PASS_STATUS
PIPELINE_CLAIM_STATUS = "CLAIM_H0_V12R3_P1_QUALIFICATION_PIPELINE"
WORKER_CLAIM_STATUS = "CLAIM_H0_V12R3_P1_QUALIFICATION_WORKER"
ROLLOUT_STARTED_STATUS = "STARTED_H0_V12R3_P1_QUALIFICATION_ROLLOUT"
ROLLOUT_PERSISTED_STATUS = "PERSISTED_H0_V12R3_P1_QUALIFICATION_BEFORE_GATE"
STAGE_FAILURE_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_STAGE"
LEDGER_PASS_STATUS = "PASS_H0_V12R3_P1_QUALIFICATION_PIPELINE_TERMINAL"
LEDGER_FAIL_STATUS = "FAIL_H0_V12R3_P1_QUALIFICATION_PIPELINE_TERMINAL"

RUN_ROOT = REPO_ROOT.joinpath(*contract.RUN_ROOT.parts)
LOCK_PATH = REPO_ROOT.joinpath(*contract.EXECUTION_LOCK_PATH.parts)
PROTOCOL_FREEZE_PATH = REPO_ROOT.joinpath(*contract.PROTOCOL_FREEZE_PATH.parts)
PIPELINE_CLAIM_PATH = REPO_ROOT.joinpath(*contract.PIPELINE_CLAIM_PATH.parts)
PIPELINE_LEDGER_PATH = REPO_ROOT.joinpath(*contract.PIPELINE_LEDGER_PATH.parts)
NOISE_ROOT = REPO_ROOT.joinpath(*contract.NOISE_ROOT.parts)

EXECUTION_AUTHORITY = {
    "one_shot": True,
    "rollout_stages": 12,
    "pair_gate_count": 6,
    "aggregate_stages": 1,
    "baseline_first": True,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "teacher_authorized": False,
    "blending_authorized": False,
    "safety_latch_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "positive_morphology_authorized": False,
    "runtime_promotion_authorized": False,
}

_ACTIVITY = {
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "baseline_actor_queries": 0,
    "candidate_actor_queries": 0,
    "teacher_queries": 0,
    "blend_count": 0,
    "latch_count": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}
EXPECTED_TERMINAL_ACTIVITY = {
    "environment_reset_calls": 12,
    "environment_step_calls": 12 * contract.EXPECTED_STEPS,
    "baseline_actor_queries": 6 * contract.EXPECTED_STEPS,
    "candidate_actor_queries": 6 * contract.EXPECTED_STEPS,
    "teacher_queries": 0,
    "blend_count": 0,
    "latch_count": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}


def _raw_relative(value: str | os.PathLike[str] | PurePosixPath) -> str:
    return value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = _raw_relative(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R3P1QualificationExecutionError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attribute = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attribute & reparse)


def _assert_no_link_components(path: Path) -> None:
    cursor = path.absolute()
    while True:
        if os.path.lexists(cursor) and _is_link_or_reparse(cursor):
            raise V12R3P1QualificationExecutionError(
                f"path contains link/reparse component: {cursor}"
            )
        if cursor == cursor.parent:
            break
        cursor = cursor.parent


def _json_value(path: str | os.PathLike[str] | Path) -> Any:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    try:
        payload = forensic.strict_json_load(source)
    except Exception as exc:
        raise V12R3P1QualificationExecutionError(
            f"cannot read strict JSON: {source}"
        ) from exc
    if source.read_bytes() != forensic.canonical_json_bytes(payload):
        raise V12R3P1QualificationExecutionError(
            f"JSON artifact is not byte-canonical: {source}"
        )
    return payload


def _mapping(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    payload = _json_value(source)
    if not isinstance(payload, Mapping):
        raise V12R3P1QualificationExecutionError(f"expected JSON object: {source}")
    return dict(payload)


def _record(path: str | os.PathLike[str] | Path) -> dict[str, Any]:
    source = Path(path).absolute()
    _assert_no_link_components(source)
    if not source.is_file() or _is_link_or_reparse(source):
        raise V12R3P1QualificationExecutionError(
            f"artifact is not a safe regular file: {source}"
        )
    try:
        relative = source.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R3P1QualificationExecutionError(
            f"artifact escaped repository: {source}"
        ) from exc
    return {
        "path": relative,
        "sha256": forensic.sha256_file(source),
        "size_bytes": source.stat().st_size,
    }


def _record_matches(value: Any, path: str | os.PathLike[str] | Path) -> bool:
    try:
        return isinstance(value, Mapping) and dict(value) == _record(path)
    except V12R3P1QualificationExecutionError:
        return False


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V12R3P1QualificationExecutionError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _stage_root(stage_id: str) -> Path:
    if stage_id not in contract.STAGE_IDS:
        raise V12R3P1QualificationExecutionError(f"unknown stage: {stage_id}")
    if stage_id == "finalize_qualification":
        return resolve_relative(contract.FINAL_ROOT)
    descriptor = stage_id.removeprefix("rollout__")
    try:
        role, case_id = descriptor.split("__", 1)
    except ValueError as exc:
        raise V12R3P1QualificationExecutionError(
            f"malformed rollout stage: {stage_id}"
        ) from exc
    return resolve_relative(contract.rollout_root(role, case_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return _stage_root(stage_id) / "receipt.json"


def _stage_failure_path(stage_id: str) -> Path:
    return _stage_root(stage_id) / "failure.json"


def _claim_path(stage_id: str) -> Path:
    return resolve_relative(contract.worker_claim_path(stage_id))


def _pair_path(case_id: str) -> Path:
    return resolve_relative(contract.pair_gate_path(case_id))


def _verified_protocol_freeze() -> dict[str, Any]:
    try:
        verified = protocol_freezer.verify_protocol_freeze()
    except Exception as exc:
        raise V12R3P1QualificationExecutionError(
            "qualification protocol freeze/current closure verification failed"
        ) from exc
    observed = _mapping(PROTOCOL_FREEZE_PATH)
    if observed != verified or observed.get("passed") is not True:
        raise V12R3P1QualificationExecutionError("protocol freeze drifted")
    return observed


def _runtime_record() -> dict[str, Any]:
    """Import and bind the heavyweight runtime only while building the lock."""

    try:
        import run_h0_primary_split_v12r3_autonomy_recovery as r3_runtime

        runtime = dict(r3_runtime._runtime_record())
    except Exception as exc:
        raise V12R3P1QualificationExecutionError(
            "V26 inference runtime is not ready"
        ) from exc
    readiness = runtime.get("platform_plugin_readiness")
    if (
        runtime.get("inference_stack_ready") is not True
        or not isinstance(readiness, Mapping)
        or readiness.get("passed") is not True
    ):
        raise V12R3P1QualificationExecutionError(
            "platform inference/plugin readiness failed"
        )
    return runtime


def _current_runtime_sources(freeze: Mapping[str, Any]) -> dict[str, Any]:
    frozen = freeze.get("runtime_sources")
    if not isinstance(frozen, Mapping) or set(frozen) != set(
        protocol_freezer.RUNTIME_SOURCE_RELATIVE_PATHS
    ):
        raise V12R3P1QualificationExecutionError(
            "protocol runtime source closure is malformed"
        )
    current = {
        name: _record(resolve_relative(path))
        for name, path in protocol_freezer.RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }
    if current != dict(frozen):
        raise V12R3P1QualificationExecutionError("runtime source closure drifted")
    return current


def _current_runtime_inputs(freeze: Mapping[str, Any]) -> dict[str, Any]:
    frozen = freeze.get("runtime_inputs")
    if not isinstance(frozen, Mapping):
        raise V12R3P1QualificationExecutionError("runtime input closure is malformed")
    observed = protocol_freezer._input_gate()
    if observed.get("passed") is not True or observed.get("records") != frozen:
        raise V12R3P1QualificationExecutionError("runtime input closure drifted")
    return copy.deepcopy(dict(frozen))


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Build the exact no-update lock without claiming the rollout root."""

    for path in (LOCK_PATH, RUN_ROOT, PIPELINE_CLAIM_PATH, PIPELINE_LEDGER_PATH):
        _assert_no_link_components(path)
    if require_unoccupied:
        for path, label in (
            (LOCK_PATH, "execution lock"),
            (RUN_ROOT, "qualification run root"),
            (PIPELINE_CLAIM_PATH, "pipeline claim"),
            (PIPELINE_LEDGER_PATH, "pipeline ledger"),
        ):
            if os.path.lexists(path):
                raise V12R3P1QualificationExecutionError(
                    f"{label} already exists/no-clobber"
                )
    freeze = _verified_protocol_freeze()
    sources = _current_runtime_sources(freeze)
    inputs = _current_runtime_inputs(freeze)
    manifest = noise.verify_manifest()
    runtime = _runtime_record()
    checks = {
        "protocol_freeze_pass": freeze.get("status")
        == contract.PROTOCOL_FREEZE_PASS_STATUS
        and freeze.get("passed") is True,
        "design_freeze_exact": freeze.get("qualification_design_freeze")
        == noise.DESIGN_FREEZE_RECORD,
        "same_exact_p1": freeze.get("selected_candidate_id") == contract.P1_CANDIDATE_ID
        and freeze.get("selected_candidate") == contract.P1_CANDIDATE_MODULE,
        "noise_manifest_current": freeze.get("noise_manifest")
        == _record(resolve_relative(contract.NOISE_MANIFEST_PATH))
        and manifest.get("passed") is True,
        "runtime_sources_current": sources == freeze.get("runtime_sources"),
        "runtime_inputs_current": inputs == freeze.get("runtime_inputs"),
        "runtime_ready": runtime.get("inference_stack_ready") is True,
        "matrix_exact_baseline_first": freeze.get("rollout_matrix")
        == list(contract.ROLLOUT_MATRIX)
        and list(contract.STAGE_IDS[:6])
        == [f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS]
        and list(contract.STAGE_IDS[6:12])
        == [f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS],
        "thirteen_stages_exact": len(contract.STAGE_IDS) == 13
        and contract.STAGE_IDS[-1] == "finalize_qualification",
        "no_fit_teacher_blend_latch_update_authority": all(
            EXECUTION_AUTHORITY[name] is False
            for name in (
                "fit_authorized",
                "offline_teacher_labeling_authorized",
                "teacher_authorized",
                "blending_authorized",
                "safety_latch_authorized",
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "prepublication_unoccupied": True,
    }
    if not all(checks.values()):
        failed = [name for name, value in checks.items() if value is not True]
        raise V12R3P1QualificationExecutionError(
            f"execution lock checks failed: {failed}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "checks": checks,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "qualification_design_freeze": copy.deepcopy(
            freeze["qualification_design_freeze"]
        ),
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "noise_manifest": _record(resolve_relative(contract.NOISE_MANIFEST_PATH)),
        "runtime_sources": sources,
        "runtime_inputs": inputs,
        "runtime": runtime,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "next_stage": "EXECUTE_QUALIFICATION_ONCE",
    }


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V12R3P1QualificationExecutionError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return verify_execution_lock(require_run_root_absent=True)


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    observed = _mapping(LOCK_PATH)
    expected = build_execution_lock(require_unoccupied=False)
    if observed != expected or LOCK_PATH.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R3P1QualificationExecutionError(
            "execution lock/current closure drifted"
        )
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V12R3P1QualificationExecutionError(
            "qualification run root already claimed"
        )
    return observed


def _pipeline_claim_payload(token_sha256: str) -> dict[str, Any]:
    lock = verify_execution_lock(require_run_root_absent=False)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PIPELINE_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "execution_token_sha256": token_sha256,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "qualification_design_freeze": copy.deepcopy(
            lock["qualification_design_freeze"]
        ),
        "noise_manifest": copy.deepcopy(lock["noise_manifest"]),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "stage_order": list(contract.STAGE_IDS),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }


def _claim_run_root() -> str:
    _assert_no_link_components(RUN_ROOT)
    if os.path.lexists(RUN_ROOT):
        raise V12R3P1QualificationExecutionError(
            "qualification run root exists; retry/resume forbidden"
        )
    RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    RUN_ROOT.mkdir(exist_ok=False)
    token = secrets.token_urlsafe(48)
    claim_published = False
    try:
        forensic.write_json_exclusive(
            PIPELINE_CLAIM_PATH, _pipeline_claim_payload(_token_sha256(token))
        )
        claim_published = True
        _verify_pipeline_claim()
    except BaseException as exc:
        if not os.path.lexists(PIPELINE_LEDGER_PATH):
            forensic.write_json_exclusive(
                PIPELINE_LEDGER_PATH,
                _preclaim_failure_ledger(exc, claim_published=claim_published),
            )
            _verify_preclaim_failure_ledger(exc, claim_published=claim_published)
        raise
    return token


def _verify_pipeline_claim() -> dict[str, Any]:
    observed = _mapping(PIPELINE_CLAIM_PATH)
    token_hash = observed.get("execution_token_sha256")
    if not isinstance(token_hash, str) or len(token_hash) != 64:
        raise V12R3P1QualificationExecutionError("pipeline claim token hash malformed")
    expected = _pipeline_claim_payload(token_hash)
    if observed != expected:
        raise V12R3P1QualificationExecutionError("pipeline claim closure drifted")
    return observed


def _worker_claim_payload(stage_id: str, token_sha256: str) -> dict[str, Any]:
    index = contract.STAGE_IDS.index(stage_id)
    previous = [
        {"stage_id": prior, "receipt": _record(_stage_receipt_path(prior))}
        for prior in contract.STAGE_IDS[:index]
    ]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": WORKER_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "stage_id": stage_id,
        "stage_index": index,
        "execution_token_sha256": token_sha256,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "previous_receipts": previous,
        "destination": contract.worker_claim_path(stage_id).as_posix(),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _write_worker_claim(stage_id: str, token_sha256: str) -> Path:
    path = _claim_path(stage_id)
    if os.path.lexists(path) or os.path.lexists(_stage_root(stage_id)):
        raise V12R3P1QualificationExecutionError(
            f"stage already consumed/no-clobber: {stage_id}"
        )
    for later in contract.STAGE_IDS[contract.STAGE_IDS.index(stage_id) + 1 :]:
        if os.path.lexists(_claim_path(later)) or os.path.lexists(_stage_root(later)):
            raise V12R3P1QualificationExecutionError(
                "later stage exists before current claim"
            )
    forensic.write_json_exclusive(path, _worker_claim_payload(stage_id, token_sha256))
    return path


def _verify_worker_claim(stage_id: str) -> dict[str, Any]:
    pipeline = _verify_pipeline_claim()
    observed = _mapping(_claim_path(stage_id))
    expected = _worker_claim_payload(stage_id, str(pipeline["execution_token_sha256"]))
    if observed != expected:
        raise V12R3P1QualificationExecutionError(
            f"worker claim/order drifted: {stage_id}"
        )
    return observed


def _load_noise_tape(case_id: str, *, np: Any) -> tuple[Any, dict[str, Any], str]:
    manifest = noise.verify_manifest()
    cases = {row["case_id"]: row for row in manifest["cases"]}
    case = contract.canonical_case(case_id)
    if case_id not in cases or cases[case_id]["noise_tape"] != case["noise_tape"]:
        raise V12R3P1QualificationExecutionError("noise case closure drifted")
    filename = PurePosixPath(case["noise_tape"]).name
    definition = manifest["tapes"][filename]
    path = NOISE_ROOT / filename
    with np.load(path, allow_pickle=False) as archive:
        array = np.ascontiguousarray(archive["standard_normal"])
    if (
        array.shape != noise.TAPE_SHAPE
        or array.dtype != np.float32
        or not np.all(np.isfinite(array))
        or noise.array_sha256(array) != definition["array_sha256"]
    ):
        raise V12R3P1QualificationExecutionError(f"noise array drifted: {case_id}")
    if case["action_selection"] == "deterministic" and np.count_nonzero(array):
        raise V12R3P1QualificationExecutionError("deterministic tape is non-zero")
    return array, _record(path), str(definition["array_sha256"])


def _increment(name: str, amount: int = 1) -> None:
    if name not in _ACTIVITY or type(amount) is not int or amount < 0:
        raise V12R3P1QualificationExecutionError(
            f"activity increment is malformed: {name}"
        )
    _ACTIVITY[name] += amount


def _append_live_step(
    rows: list[dict[str, Any]],
    row: Mapping[str, Any],
    *,
    persist_step: Callable[[int, Mapping[str, Any]], Any] | None,
) -> None:
    """Durably publish a step before retaining it in the in-memory trace."""

    value = dict(row)
    step = value.get("step")
    if type(step) is not int or step != len(rows) + 1:
        raise V12R3P1QualificationExecutionError(
            f"live forensic step order drifted: {step!r}"
        )
    if persist_step is not None:
        persist_step(step, value)
    rows.append(value)


def _accumulate_baseline_so_recovery(
    solver_totals: dict[str, int],
    *,
    entries: Any,
    step: int,
    selected_fallback: bool,
    policy_id: str,
    v3: Any,
    so_recovery: Any,
) -> int:
    """Classify V6 SO recovery without treating accepted bounded-LS as failure."""

    v3._validate_so_solver_audit_entries(
        entries,
        step_index=step,
        selected_fallback=selected_fallback,
    )
    classified = so_recovery.classify_policy_step(entries, policy_id=policy_id)
    counters = classified.get("counters") if isinstance(classified, Mapping) else None
    if not isinstance(counters, Mapping):
        raise V12R3P1QualificationExecutionError(
            "baseline SO classification is malformed"
        )
    for key in v3.SO_RECOVERY_COUNTER_KEYS:
        value = counters.get(key)
        if type(value) is not int or value < 0 or key not in solver_totals:
            raise V12R3P1QualificationExecutionError(
                f"baseline SO counter drifted: {key}"
            )
        solver_totals[key] += value
    if counters.get("control_window_count") != 10:
        raise V12R3P1QualificationExecutionError(
            "baseline SO control-window count drifted"
        )
    return int(
        counters["unaccepted_hard_so_fallback_count"]
        + counters["unaccepted_bounded_ls_count"]
    )


def _collect_physical_rollout(
    *,
    role: str,
    case: Mapping[str, Any],
    persist_step: Callable[[int, Mapping[str, Any]], Any] | None = None,
) -> dict[str, Any]:
    """Run one role with local heavyweight imports and return replayable evidence."""

    import numpy as np

    import h0_v3_so_recovery_contract as so_recovery
    import primary_grf_split_adaptation as split_contract
    import run_h0_primary_grf_split_v1_adaptation as v1
    import run_h0_primary_grf_split_v3_semantic_replay as v3
    import run_h0_primary_split_v6_qualification as v6_runtime
    import run_h0_primary_split_v9_causal_teacher as env_source
    import run_h0_primary_split_v12r3_autonomy_recovery as r3_runtime

    (
        rollout_eval,
        runtime_np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = r3_runtime._load_rollout_stack()
    if runtime_np is not np:
        raise V12R3P1QualificationExecutionError("runtime NumPy identity drifted")
    lock = verify_execution_lock(require_run_root_absent=False)
    module_record = lock["runtime_inputs"][
        "source_h0_module" if role == contract.BASELINE_ROLE else "candidate_module"
    ]
    module_path = resolve_relative(module_record["path"])
    module = RLModule.from_checkpoint(module_path)
    if hasattr(module, "eval"):
        module.eval()
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    if role == contract.CANDIDATE_ROLE:
        env_config = env_source.build_env_config(case)
    else:
        inputs = lock["runtime_inputs"]
        env_config = v6_runtime._build_env_config(
            role=role,
            case=case,
            execution={
                "inputs": {
                    "source_h0_config": inputs["source_h0_config"],
                    "analog_profile": inputs["historical_analog_profile"],
                    "v25_profile": inputs["baseline_shadow_v25_profile"],
                }
            },
            legacy=legacy,
        )
    env = env_factory.make_cmc_env(env_config)
    tape, tape_record, tape_array_sha = _load_noise_tape(str(case["case_id"]), np=np)
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    reserve = legacy._empty_accumulator()
    residual = legacy._empty_accumulator()
    sea = legacy._sea_accumulators()
    solver_totals = v6_runtime._solver_totals(v3)
    penetrations: list[float] = []
    binary_samples = 0
    valid_cycles = 0
    invalid_events = 0
    sea_fallback_count = 0
    timeout_count = 0
    clipping_count = 0
    nonfinite_count = 0
    hard_invalid_count = 0
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    shadow_fsm: Any | None = None
    started = time.monotonic()
    try:
        _increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
        full_names = tuple(
            str(name) for name in env.unwrapped.observation_feature_names
        )
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.float32
            or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
            or len(full_names) != contract.EXPECTED_FULL_FEATURES
            or not np.all(np.isfinite(observation))
            or not isinstance(reset_info, Mapping)
        ):
            raise V12R3P1QualificationExecutionError(
                "35/84 float32 reset contract drifted"
            )
        if role == contract.CANDIDATE_ROLE:
            audit = r3_runtime._new_physical_audit(
                reset_info=reset_info, legacy=legacy, np=np
            )
        else:
            if not v6_runtime._runtime_routing_ok(
                reset_info, role=role, require_samples=False
            ):
                raise V12R3P1QualificationExecutionError(
                    "baseline legacy routing drifted at reset"
                )
            shadow_fsm = copy.deepcopy(env.unwrapped._phase_fsm)
        current_info = dict(reset_info)
        body_weight_n = float(env.unwrapped._body_weight_n)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            if role == contract.BASELINE_ROLE:
                paired = split_contract.build_paired_views(
                    observation_before,
                    actor_names,
                    current_info,
                    body_weight_n=body_weight_n,
                    reset_boundary=index == 0,
                    teacher_phase_observation=shadow_fsm.observation(),
                )
                policy_input = v1._teacher_full_observation(
                    observation_before, paired, np
                )
                actor_observation = np.ascontiguousarray(
                    paired.teacher, dtype=np.float32
                )
                _raw, mean, std = v1._policy(
                    module,
                    policy_input,
                    tuple(env.action_space.shape),
                    rollout_eval,
                )
                _increment("baseline_actor_queries")
            else:
                actor_observation = np.ascontiguousarray(
                    observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                    dtype=np.float32,
                )
                mean, std = r3_runtime._query_mean_std(
                    module, actor_observation, np=np, torch=torch
                )
                _increment("candidate_actor_queries")
            mean = np.ascontiguousarray(mean, dtype=np.float32).reshape(
                contract.EXPECTED_ACTION_SHAPE
            )
            std = np.ascontiguousarray(std, dtype=np.float32).reshape(
                contract.EXPECTED_ACTION_SHAPE
            )
            if (
                not np.all(np.isfinite(mean))
                or not np.all(np.isfinite(std))
                or not np.allclose(
                    std, contract.STOCHASTIC_SIGMA, rtol=0.0, atol=1.0e-8
                )
            ):
                raise V12R3P1QualificationExecutionError(
                    "actor mean/logstd contract drifted"
                )
            single_noise = np.ascontiguousarray(std * tape[index], dtype=np.float32)
            raw_action = np.ascontiguousarray(mean + single_noise, dtype=np.float32)
            applied_action = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            clipping_count += int(np.count_nonzero(raw_action != applied_action))
            _increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(
                applied_action
            )
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R3P1QualificationExecutionError("rollout info is malformed")
            if role == contract.CANDIDATE_ROLE:
                physical = r3_runtime._consume_physical_step(
                    audit,
                    step=step,
                    info=info,
                    observation_before=observation_before,
                    observation_after=observation_after,
                    reward=reward,
                    action=raw_action,
                    applied_action=applied_action,
                    extra_vectors=(actor_observation, mean, std, single_noise),
                    legacy=legacy,
                    v26_collector=v26_collector,
                )
                penetration = float(physical["penetration_m"])
                reserve_value = float(physical["reserve_norm_nm"])
                residual_value = float(physical["residual_norm_nm"])
            else:
                if not v6_runtime._runtime_routing_ok(
                    info, role=role, require_samples=True
                ):
                    raise V12R3P1QualificationExecutionError(
                        "baseline legacy routing drifted"
                    )
                samples = info.get("binary_phase_sensor_samples")
                binary_samples += len(samples)
                reward_terms = info.get("reward_terms")
                phase = info.get("phase_fsm")
                if not isinstance(reward_terms, Mapping) or not isinstance(
                    phase, Mapping
                ):
                    raise V12R3P1QualificationExecutionError(
                        "baseline diagnostics are incomplete"
                    )
                reserve_value = float(reward_terms.get("reserve_norm_nm"))
                residual_value = float(reward_terms.get("residual_norm_nm"))
                penetration = float(reward_terms.get("grf_penetration_m"))
                legacy._accumulate_scalar(reserve, reserve_value)
                legacy._accumulate_scalar(residual, residual_value)
                sea_payload = info.get("sea_segment_diagnostics")
                legacy._accumulate_sea(sea, sea_payload)
                sea_step_fallback = v6_runtime._sea_fallback_count(sea_payload)
                sea_fallback_count += sea_step_fallback
                so = info.get("so_diagnostics")
                if (
                    not isinstance(so, Mapping)
                    or type(so.get("solver_fallback_used")) is not bool
                ):
                    raise V12R3P1QualificationExecutionError(
                        "baseline SO diagnostic is missing"
                    )
                selected_fallback = bool(so["solver_fallback_used"])
                entries = info.get("so_solver_audit_entries")
                _accumulate_baseline_so_recovery(
                    solver_totals,
                    entries=entries,
                    step=step,
                    selected_fallback=selected_fallback,
                    policy_id=v6_runtime.contract.SO_POLICY_ID,
                    v3=v3,
                    so_recovery=so_recovery,
                )
                hard_invalid_count += int("failure" in info)
                valid_cycles = int(float(phase.get("valid_cycle_count", 0.0)))
                invalid_events = max(
                    invalid_events,
                    int(float(phase.get("invalid_event_count", 0.0))),
                )
                timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0)
                v1._update_shadow_fsm(
                    shadow_fsm, info=info, body_weight_n=body_weight_n
                )
            penetrations.append(penetration)
            finite = bool(
                np.all(np.isfinite(observation_after))
                and np.all(np.isfinite(raw_action))
                and math.isfinite(float(reward))
                and math.isfinite(penetration)
                and math.isfinite(reserve_value)
                and math.isfinite(residual_value)
            )
            nonfinite_count += int(not finite)
            if not finite:
                raise V12R3P1QualificationExecutionError(
                    "non-finite physical rollout value"
                )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "role": role,
                "case_id": case["case_id"],
                "actor_input_view": (
                    "historical_analog"
                    if role == contract.BASELINE_ROLE
                    else "primary_split"
                ),
                "actor_observation": actor_observation.tolist(),
                "mean_action": mean.tolist(),
                "standard_normal": tape[index].tolist(),
                "single_noise": single_noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied_action.tolist(),
                "reward": float(reward),
                "grf_penetration_m": penetration,
                "reserve_norm_nm": reserve_value,
                "residual_norm_nm": residual_value,
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            _append_live_step(
                rows,
                {"step": step, **row},
                persist_step=persist_step,
            )
            observation = observation_after
            current_info = dict(info)
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12P1Q {role}/{case['case_id']}] {step:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()
    if role == contract.CANDIDATE_ROLE:
        if audit is None:
            raise V12R3P1QualificationExecutionError(
                "candidate physical audit was not initialized"
            )
        physical_summary = r3_runtime._physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        )
    else:
        phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
        sea_metrics = legacy._finalize_sea(sea)
        aggregate_sea_fallback = sum(
            int(sea_metrics[joint]["fallback_count"]) for joint in contract.JOINTS
        )
        if aggregate_sea_fallback != sea_fallback_count:
            raise V12R3P1QualificationExecutionError(
                "baseline SEA fallback aggregation mismatch"
            )
        so_solver_unaccepted_count = (
            solver_totals["unaccepted_hard_so_fallback_count"]
            + solver_totals["unaccepted_bounded_ls_count"]
        )
        fallback_count = so_solver_unaccepted_count + sea_fallback_count
        physical_summary = {
            "steps": len(rows),
            "end_reason": info.get("end_reason"),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "phase_valid_cycle_count": max(
                valid_cycles, int(float(phase.get("valid_cycle_count", 0.0)))
            ),
            "grf_penetration_max_m": max(penetrations, default=0.0),
            "control_window_count": solver_totals["control_window_count"],
            "raw_sensor_sample_count": binary_samples,
            "binary_phase_sensor_sample_count": binary_samples,
            "action_clipped_values": clipping_count,
            "fallback_count": fallback_count,
            "timeout_count": timeout_count,
            "sea_plugin_fallback_count": sea_fallback_count,
            "hard_invalid_count": hard_invalid_count,
            "invalid_event_count": invalid_events,
            "nonfinite_count": nonfinite_count,
            "so_solver_unaccepted_count": so_solver_unaccepted_count,
            "routing_failure_count": 0,
            "step_contract_failure_count": 0,
            "safety_stop_count": int(bool(terminated)),
            "n_actor": len(actor_names),
            "n_observation": len(full_names),
            "observation_dtype": contract.EXPECTED_DTYPE,
            "action_shape": list(contract.EXPECTED_ACTION_SHAPE),
            "action_dtype": contract.EXPECTED_DTYPE,
            "episode_metrics": {
                "reserve_norm_nm": legacy._finalize_accumulator(reserve),
                "residual_norm_nm": legacy._finalize_accumulator(residual),
            },
            "sea_episode_metrics": sea_metrics,
            "legacy_event_integrity_passed": True,
        }
    return {
        "rows": rows,
        "physical_summary": physical_summary,
        "noise_tape": tape_record,
        "noise_tape_array_sha256": tape_array_sha,
    }


def _build_rollout_summary(
    *,
    role: str,
    case_id: str,
    physical: Mapping[str, Any],
    artifacts: Mapping[str, Mapping[str, Any]],
    noise_tape: Mapping[str, Any],
    noise_tape_array_sha256: str,
    actor_module: Mapping[str, Any],
) -> dict[str, Any]:
    """Normalize physical evidence into the frozen pure common-gate schema."""

    expected = contract.canonical_rollout(role, case_id)
    for field, closed_value in (
        ("protected_trials_opened", []),
        ("reserve_trials_opened", []),
        ("runtime_promoted", False),
    ):
        if field in physical and physical.get(field) != closed_value:
            raise V12R3P1QualificationExecutionError(
                f"physical summary violates closed-data contract: {field}"
            )
    summary = {
        **copy.deepcopy(expected),
        **copy.deepcopy(dict(physical)),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "actor_module": copy.deepcopy(dict(actor_module)),
        "trace_step_count": int(physical.get("steps", -1)),
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "actor_query_count": int(physical.get("steps", -1)),
        "random_noise_draw_count": (
            contract.EXPECTED_STEPS
            if expected["action_selection"] == "stochastic"
            else 0
        ),
        "single_noise_application_count": int(physical.get("steps", -1)),
        "noise_tape": copy.deepcopy(dict(noise_tape)),
        "noise_tape_array_sha256": noise_tape_array_sha256,
        "prerequisite_gate_passed": True,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        **{name: copy.deepcopy(dict(record)) for name, record in artifacts.items()},
    }
    procedural_zero = {
        "safety_latch_activation_count",
        "safety_latch_release_count",
        "safety_intervention_count",
        "physical_gate_bypass_count",
        "multiple_noise_application_count",
        "noise_application_mismatch_count",
        "served_action_teacher_dependency_count",
        "teacher_query_count",
        "mean_blend_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
    for field in procedural_zero:
        observed = summary.get(field, 0)
        if type(observed) is not int or observed != 0:
            raise V12R3P1QualificationExecutionError(
                f"non-zero procedural autonomy counter: {field}={observed!r}"
            )
        summary[field] = 0
    for field in contract.ZERO_REQUIRED_COUNTS:
        if field not in summary:
            raise V12R3P1QualificationExecutionError(
                f"physical summary lacks required counter: {field}"
            )
    if role == contract.BASELINE_ROLE:
        summary["legacy_event_integrity_passed"] = (
            physical.get("legacy_event_integrity_passed") is True
        )
    return summary


def _prospective_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    return {
        "path": path.relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _run_rollout(stage_id: str) -> dict[str, Any]:
    _, role, case_id = stage_id.split("__", 2)
    case = contract.canonical_case(case_id)
    destination = _stage_root(stage_id)
    if os.path.lexists(destination):
        raise V12R3P1QualificationExecutionError(
            f"rollout destination exists: {stage_id}"
        )
    pipeline_claim = _verify_pipeline_claim()
    worker_claim = _verify_worker_claim(stage_id)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_STARTED_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "role": role,
            "case": case,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
            "execution_lock": _record(LOCK_PATH),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
    )
    try:
        collected = _collect_physical_rollout(
            role=role,
            case=case,
            persist_step=writer.write_step,
        )
        rows = collected["rows"]
        partial = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_PERSISTED_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "role": role,
            "case_id": case_id,
            "steps": len(rows),
            "gate_evaluated": False,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
        artifacts = {
            "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
            "execution_lock": _record(LOCK_PATH),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "run_start": _record(writer.run_start_path),
            "trace": _prospective_record(writer.trace_path, rows),
            "partial_summary": _prospective_record(
                writer.partial_summary_path, partial
            ),
        }
        summary = _build_rollout_summary(
            role=role,
            case_id=case_id,
            physical=collected["physical_summary"],
            artifacts=artifacts,
            noise_tape=collected["noise_tape"],
            noise_tape_array_sha256=collected["noise_tape_array_sha256"],
            actor_module=_mapping(LOCK_PATH)["runtime_inputs"][
                (
                    "source_h0_module"
                    if role == contract.BASELINE_ROLE
                    else "candidate_module"
                )
            ],
        )
        persisted = writer.finalize_before_gate(
            trace=rows, partial_summary=partial, summary=summary
        )
        persisted["run_start"] = _record(writer.run_start_path)
        for name in ("run_start", "trace", "partial_summary"):
            if persisted.get(name) != artifacts[name]:
                raise V12R3P1QualificationExecutionError(
                    f"persist-before-gate record drifted: {stage_id}/{name}"
                )
        common_gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
        writer.publish_gate(common_gate)
        if common_gate.get("passed") is not True:
            failed = [
                name
                for name, value in common_gate.get("checks", {}).items()
                if value is not True
            ]
            raise V12R3P1QualificationExecutionError(
                f"common rollout gate failed: {stage_id}: {failed}"
            )
        pair_gate = None
        if role == contract.CANDIDATE_ROLE:
            baseline_summary = _mapping(
                resolve_relative(contract.rollout_root(contract.BASELINE_ROLE, case_id))
                / "summary.json"
            )
            pair_gate = gates.condition_matched_gate(
                baseline_summary, summary, case_id=case_id
            )
            forensic.write_json_exclusive(_pair_path(case_id), pair_gate)
            if pair_gate.get("passed") is not True:
                raise V12R3P1QualificationExecutionError(
                    f"condition-matched pair failed: {case_id}"
                )
        receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": common_gate["status"],
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "role": role,
            "case_id": case_id,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "summary": _record(writer.summary_path),
            "gate": _record(writer.gate_path),
            "trace": _record(writer.trace_path),
            "pair_gate": (
                _record(_pair_path(case_id))
                if role == contract.CANDIDATE_ROLE
                else None
            ),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "resume_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "runtime_promoted": False,
        }
        forensic.write_json_exclusive(_stage_receipt_path(stage_id), receipt)
        return receipt
    except BaseException as exc:
        if writer.run_start_path.is_file() and not writer.failure_path.exists():
            try:
                writer.publish_failure(
                    end_reason="qualification_rollout_failed_terminal_no_retry",
                    error=exc,
                    status=STAGE_FAILURE_STATUS,
                    details={
                        "stage_id": stage_id,
                        "role": role,
                        "case_id": case_id,
                        "pipeline_claim": pipeline_claim,
                        "worker_claim": worker_claim,
                        "activity": copy.deepcopy(_ACTIVITY),
                        "retry_authorized": False,
                        "runtime_promoted": False,
                    },
                )
            except Exception:
                pass
        raise


def _verify_rollout_receipt(stage_id: str) -> dict[str, Any]:
    _, role, case_id = stage_id.split("__", 2)
    receipt = _mapping(_stage_receipt_path(stage_id))
    root = _stage_root(stage_id)
    writer = forensic.ForensicRolloutWriter(root, artifact_root=REPO_ROOT)
    persisted = writer.finalized_artifact_records()
    _json_value(writer.run_start_path)
    journal_rows = _verify_step_journal_canonical(writer)
    if _json_value(writer.trace_path) != journal_rows:
        raise V12R3P1QualificationExecutionError(
            f"trace/journal closure drifted: {stage_id}"
        )
    _json_value(writer.partial_summary_path)
    summary = _mapping(root / "summary.json")
    observed_gate = _mapping(root / "gate.json")
    expected_gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
    pair_record = None
    if role == contract.CANDIDATE_ROLE:
        baseline = _mapping(
            resolve_relative(contract.rollout_root(contract.BASELINE_ROLE, case_id))
            / "summary.json"
        )
        pair = _mapping(_pair_path(case_id))
        expected_pair = gates.condition_matched_gate(baseline, summary, case_id=case_id)
        if pair != expected_pair or pair.get("passed") is not True:
            raise V12R3P1QualificationExecutionError(
                f"condition-matched pair closure drifted: {case_id}"
            )
        pair_record = _record(_pair_path(case_id))
    expected_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": expected_gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "role": role,
        "case_id": case_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "summary": _record(writer.summary_path),
        "gate": _record(writer.gate_path),
        "trace": _record(writer.trace_path),
        "pair_gate": pair_record,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }
    summary_records_exact = all(
        summary.get(name) == record
        for name, record in {
            "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
            "execution_lock": _record(LOCK_PATH),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "run_start": _record(writer.run_start_path),
            "trace": persisted["trace"],
            "partial_summary": persisted["partial_summary"],
            "noise_tape": _record(
                resolve_relative(contract.canonical_case(case_id)["noise_tape"])
            ),
        }.items()
    )
    if (
        receipt != expected_receipt
        or observed_gate != expected_gate
        or observed_gate.get("passed") is not True
        or not summary_records_exact
    ):
        raise V12R3P1QualificationExecutionError(
            f"rollout receipt/gate closure drifted: {stage_id}"
        )
    return receipt


def _verify_step_journal_canonical(
    writer: forensic.ForensicRolloutWriter,
    *,
    expected_steps: int = contract.EXPECTED_STEPS,
) -> list[Any]:
    if type(expected_steps) is not int or expected_steps < 1:
        raise V12R3P1QualificationExecutionError(
            "expected step-journal length is malformed"
        )
    directory = writer.steps_directory
    _assert_no_link_components(directory)
    if not directory.is_dir() or _is_link_or_reparse(directory):
        raise V12R3P1QualificationExecutionError(
            f"step journal directory is missing/unsafe: {directory}"
        )
    entries = sorted(directory.iterdir(), key=lambda item: item.name)
    expected_names = [f"{step:06d}.json" for step in range(1, expected_steps + 1)]
    if [item.name for item in entries] != expected_names:
        raise V12R3P1QualificationExecutionError("step journal file set/order drifted")
    rows = []
    for step, path in enumerate(entries, start=1):
        if not path.is_file() or _is_link_or_reparse(path):
            raise V12R3P1QualificationExecutionError(
                f"step journal artifact is missing/unsafe: {path}"
            )
        row = _json_value(path)
        if not isinstance(row, Mapping) or row.get("step") != step:
            raise V12R3P1QualificationExecutionError(
                f"step journal row drifted: {path.name}"
            )
        rows.append(row)
    return rows


def _aggregate_summary_payload(stage_id: str) -> dict[str, Any]:
    if stage_id != "finalize_qualification":
        raise V12R3P1QualificationExecutionError(f"unknown aggregate stage: {stage_id}")
    bindings = []
    for case_id in contract.CASE_IDS:
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "pair_gate": _record(_pair_path(case_id)),
                "baseline_receipt": _record(
                    resolve_relative(
                        contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id)
                    )
                ),
                "candidate_receipt": _record(
                    resolve_relative(
                        contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id)
                    )
                ),
            }
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "prerequisite_gate_passed": True,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": bindings,
        "pair_count": 6,
        "passing_pair_count": 6,
        "failed_pair_count": 0,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "compensation_or_averaging_used": False,
        "retry_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _aggregate_receipt_payload(
    stage_id: str, gate: Mapping[str, Any]
) -> dict[str, Any]:
    final_root = _stage_root(stage_id)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "case_count": 6,
        "summary": _record(final_root / "summary.json"),
        "gate": _record(final_root / "gate.json"),
        "pair_gates": [_record(_pair_path(case_id)) for case_id in contract.CASE_IDS],
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def _verify_aggregate_receipt(
    stage_id: str = "finalize_qualification",
) -> dict[str, Any]:
    _verify_worker_claim(stage_id)
    for rollout_stage in contract.STAGE_IDS[:12]:
        _verify_rollout_receipt(rollout_stage)
    final_root = _stage_root(stage_id)
    summary = _mapping(final_root / "summary.json")
    expected_summary = _aggregate_summary_payload(stage_id)
    observed_gate = _mapping(final_root / "gate.json")
    expected_gate = gates.aggregate_qualification_gate(summary)
    receipt = _mapping(_stage_receipt_path(stage_id))
    expected_receipt = _aggregate_receipt_payload(stage_id, expected_gate)
    if (
        summary != expected_summary
        or observed_gate != expected_gate
        or observed_gate.get("passed") is not True
        or receipt != expected_receipt
    ):
        raise V12R3P1QualificationExecutionError(
            "aggregate summary/gate/receipt closure drifted"
        )
    return receipt


def _run_aggregate(stage_id: str) -> dict[str, Any]:
    if os.path.lexists(_stage_root(stage_id)):
        raise V12R3P1QualificationExecutionError("aggregate root exists")
    _verify_worker_claim(stage_id)
    for rollout_stage in contract.STAGE_IDS[:12]:
        _verify_rollout_receipt(rollout_stage)
    final_root = _stage_root(stage_id)
    final_root.mkdir(parents=True, exist_ok=False)
    try:
        summary = _aggregate_summary_payload(stage_id)
        forensic.write_json_exclusive(final_root / "summary.json", summary)
        gate = gates.aggregate_qualification_gate(summary)
        forensic.write_json_exclusive(final_root / "gate.json", gate)
        if gate.get("passed") is not True:
            raise V12R3P1QualificationExecutionError("aggregate 6/6 gate failed")
        receipt = _aggregate_receipt_payload(stage_id, gate)
        forensic.write_json_exclusive(_stage_receipt_path(stage_id), receipt)
        return _verify_aggregate_receipt(stage_id)
    except BaseException as exc:
        failure_path = _stage_failure_path(stage_id)
        if not os.path.lexists(failure_path):
            artifacts = {}
            for name in ("summary", "gate", "receipt"):
                path = final_root / f"{name}.json"
                if path.is_file() and not _is_link_or_reparse(path):
                    artifacts[name] = _record(path)
            forensic.write_json_exclusive(
                failure_path,
                {
                    "schema_version": contract.SCHEMA_VERSION,
                    "status": STAGE_FAILURE_STATUS,
                    "passed": False,
                    "protocol_id": contract.PROTOCOL_ID,
                    "pipeline_id": contract.PIPELINE_ID,
                    "stage_id": stage_id,
                    "candidate_id": contract.P1_CANDIDATE_ID,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "artifacts": artifacts,
                    "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
                    "worker_claim": _record(_claim_path(stage_id)),
                    "activity": copy.deepcopy(_ACTIVITY),
                    "retry_authorized": False,
                    "resume_authorized": False,
                    "runtime_promoted": False,
                },
            )
        raise


def _run_stage(stage_id: str) -> dict[str, Any]:
    if stage_id.startswith("rollout__"):
        return _run_rollout(stage_id)
    if stage_id == "finalize_qualification":
        return _run_aggregate(stage_id)
    raise V12R3P1QualificationExecutionError(f"unauthorized stage: {stage_id}")


def _ledger_payload(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed_stages: Sequence[str],
    error: BaseException | None,
) -> dict[str, Any]:
    attempted_claim = (
        _record(_claim_path(attempted_stage))
        if attempted_stage is not None and os.path.isfile(_claim_path(attempted_stage))
        else None
    )
    attempted_failure = (
        _record(_stage_failure_path(attempted_stage))
        if attempted_stage is not None
        and os.path.isfile(_stage_failure_path(attempted_stage))
        else None
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LEDGER_PASS_STATUS if passed else LEDGER_FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "stage_order": list(contract.STAGE_IDS),
        "attempted_stage": attempted_stage,
        "attempted_stage_claim": attempted_claim,
        "attempted_stage_failure": attempted_failure,
        "completed_stages": list(completed_stages),
        "completed_receipts": [
            {"stage_id": stage, "receipt": _record(_stage_receipt_path(stage))}
            for stage in completed_stages
        ],
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "activity": copy.deepcopy(_ACTIVITY),
        "aggregate_requires_6_of_6": True,
        "compensation_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }


def _preclaim_failure_ledger(
    error: BaseException, *, claim_published: bool
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LEDGER_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "attempted_stage": None,
        "completed_stages": [],
        "completed_receipts": [],
        "error_type": type(error).__name__,
        "error": str(error),
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": (_record(PIPELINE_CLAIM_PATH) if claim_published else None),
        "claim_published": claim_published,
        "activity": copy.deepcopy(_ACTIVITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def _verify_preclaim_failure_ledger(
    error: BaseException, *, claim_published: bool
) -> dict[str, Any]:
    observed = _mapping(PIPELINE_LEDGER_PATH)
    expected = _preclaim_failure_ledger(error, claim_published=claim_published)
    if observed != expected:
        raise V12R3P1QualificationExecutionError(
            "preclaim terminal ledger closure drifted"
        )
    return observed


def _verify_pipeline_ledger(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed_stages: Sequence[str],
    error: BaseException | None,
) -> dict[str, Any]:
    observed = _mapping(PIPELINE_LEDGER_PATH)
    expected = _ledger_payload(
        passed=passed,
        attempted_stage=attempted_stage,
        completed_stages=completed_stages,
        error=error,
    )
    if observed != expected:
        raise V12R3P1QualificationExecutionError(
            "terminal pipeline ledger closure drifted"
        )
    if passed:
        if (
            attempted_stage is not None
            or list(completed_stages) != list(contract.STAGE_IDS)
            or _ACTIVITY != EXPECTED_TERMINAL_ACTIVITY
        ):
            raise V12R3P1QualificationExecutionError(
                "PASS ledger terminal activity/order drifted"
            )
        _verify_aggregate_receipt()
    return observed


def execute_qualification_once() -> dict[str, Any]:
    """Consume the complete frozen matrix and terminal aggregate once."""

    verify_execution_lock(require_run_root_absent=True)
    for name in _ACTIVITY:
        _ACTIVITY[name] = 0
    completed: list[str] = []
    attempted: str | None = None
    token: str | None = None
    claimed = False
    try:
        token = _claim_run_root()
        claimed = True
        token_sha256 = _token_sha256(token)
        token = None
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            _write_worker_claim(stage_id, token_sha256)
            receipt = _run_stage(stage_id)
            if receipt.get("passed") is not True:
                raise V12R3P1QualificationExecutionError(
                    f"stage returned non-PASS: {stage_id}"
                )
            if stage_id.startswith("rollout__"):
                _verify_rollout_receipt(stage_id)
            else:
                _verify_aggregate_receipt(stage_id)
            completed.append(stage_id)
        if completed != list(contract.STAGE_IDS):
            raise V12R3P1QualificationExecutionError(
                "thirteen-stage completion order drifted"
            )
        if _ACTIVITY != EXPECTED_TERMINAL_ACTIVITY:
            raise V12R3P1QualificationExecutionError(
                f"terminal activity counters drifted: {_ACTIVITY}"
            )
        ledger = _ledger_payload(
            passed=True,
            attempted_stage=None,
            completed_stages=completed,
            error=None,
        )
        forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
        return _verify_pipeline_ledger(
            passed=True,
            attempted_stage=None,
            completed_stages=completed,
            error=None,
        )
    except BaseException as exc:
        if claimed and not os.path.lexists(PIPELINE_LEDGER_PATH):
            forensic.write_json_exclusive(
                PIPELINE_LEDGER_PATH,
                _ledger_payload(
                    passed=False,
                    attempted_stage=attempted,
                    completed_stages=completed,
                    error=exc,
                ),
            )
            _verify_pipeline_ledger(
                passed=False,
                attempted_stage=attempted,
                completed_stages=completed,
                error=exc,
            )
        raise
    finally:
        token = None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare-execution-lock", action="store_true")
    action.add_argument("--verify-execution-lock", action="store_true")
    action.add_argument("--execute-once", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_execution_lock:
        payload = prepare_execution_lock()
    elif args.verify_execution_lock:
        payload = verify_execution_lock()
    else:
        payload = execute_qualification_once()
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "EXECUTION_AUTHORITY",
    "LEDGER_FAIL_STATUS",
    "LEDGER_PASS_STATUS",
    "V12R3P1QualificationExecutionError",
    "build_execution_lock",
    "execute_qualification_once",
    "prepare_execution_lock",
    "verify_execution_lock",
]
