"""Run the additive V12R3/P1 salvage development matrix exactly once.

The canonical V12R3 run is immutable terminal evidence.  This runner reads
only the exact frozen P1 module selected by the additive protocol and writes
only below the isolated V12P1S run root.  The execution lock is reconstructed
and verified before the run root is claimed.  Afterwards every stage is
one-way, receipt chained, and terminal on any failure; there is no resume,
retry, fitting, update, teacher, blending, latch, or promotion surface here.

Importing this module performs no filesystem write, environment construction,
policy inference, or random draw.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import inspect
import os
import secrets
import stat
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


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
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
V12R3_ROOT = LOCAL_VALIDATION_ROOT / "v12r3"
V12P1Q_ROOT = LOCAL_VALIDATION_ROOT / "v12p1q"
V12P1S_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    VALIDATION_ROOT,
    TRAJECTORY_ROOT,
    BASELINE_ROOT,
    LOCAL_VALIDATION_ROOT,
    V12R3_ROOT,
    V12P1Q_ROOT,
    V12P1S_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import h0_v12r3_p1_salvage_contract as contract  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as r3_runner  # noqa: E402
import freeze_h0_v12r3_p1_qualification_design as q_design_freezer  # noqa: E402
import h0_v12r3_p1_qualification_contract as q_contract  # noqa: E402


class P1SalvageDevelopmentError(RuntimeError):
    """Raised when continuing would violate the frozen salvage protocol."""


LOCK_STATUS = "PASS_H0_V12R3_P1_SALVAGE_EXECUTION_LOCK"
CLAIM_STATUS = contract.PIPELINE_CLAIM_STATUS
WORKER_CLAIM_STATUS = "CLAIM_H0_V12R3_P1_SALVAGE_WORKER"
ROLLOUT_STARTED_STATUS = "STARTED_H0_V12R3_P1_SALVAGE_ROLLOUT"
ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_V12R3_P1_SALVAGE_ROLLOUT_BEFORE_GATE"
ROLLOUT_PERSISTED_STATUS = "PERSISTED_H0_V12R3_P1_SALVAGE_BEFORE_GATE"
AGGREGATE_STATUS = "COMPLETE_H0_V12R3_P1_SALVAGE_DEVELOPMENT_BEFORE_GATE"
STAGE_FAILURE_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_STAGE"
LEDGER_PASS_STATUS = "PASS_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
LEDGER_FAIL_STATUS = "FAIL_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
BEHAVIOR = contract.BEHAVIOR
TARGET_CONTRACT_ID = contract.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = contract.EVENT_CONTRACT_ID
EXPECTED_STEPS = contract.EXPECTED_STEPS
EXPECTED_CONTROL_WINDOWS = contract.EXPECTED_CONTROL_WINDOWS
EXPECTED_ACTOR_FEATURES = contract.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = contract.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = 2
EXPECTED_DTYPE = "float32"
MORPHOLOGY_WEIGHT = contract.MORPHOLOGY_WEIGHT

EXECUTION_SOURCE_RELATIVE_PATHS = {
    "s_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12p1s/"
        "h0_v12r3_p1_salvage_contract.py"
    ),
    "s_protocol_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12p1s/"
        "freeze_h0_v12r3_p1_salvage.py"
    ),
    "s_development_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12p1s/"
        "run_h0_v12r3_p1_salvage_development.py"
    ),
    "q_design_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "freeze_h0_v12r3_p1_qualification_design.py"
    ),
    "q_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12p1q/"
        "h0_v12r3_p1_qualification_contract.py"
    ),
    "v12r3_runtime_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "v12r3_contract": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_autonomy_recovery_contract.py"
    ),
    "v12r3_labeler": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_pure_probe_observer_labeler.py"
    ),
    "v12r3_fitter": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_recovery_weighted_fitter.py"
    ),
    "v12r3_design_audit": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_design_audit.py"
    ),
    "v12r3_protocol_freezer": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "freeze_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "v12_contract": (
        "Trajectory Generator/baseline_MLP/validation/"
        "h0_primary_split_v12_autonomy_recovery_contract.py"
    ),
    "v12_protocol_freezer": (
        "Trajectory Generator/baseline_MLP/validation/"
        "freeze_h0_primary_split_v12_autonomy_recovery.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v10_coherent_teacher": "validation/h0_primary_split_v10_coherent_teacher.py",
    "v10s_blend_constants": "validation/h0_primary_split_v10s_blend.py",
    "v10s_fit": "validation/h0_primary_split_v10s_fit.py",
    "v10s_contract": "validation/h0_primary_split_v10s_safe_dagger_contract.py",
    "v10s_legacy_runner": "validation/run_h0_primary_split_v10s_safe_dagger.py",
    "so_recovery_contract": "validation/h0_v3_so_recovery_contract.py",
    "v11_weighted_fit": "validation/h0_primary_split_v11_weighted_fit.py",
    "v11_contract": "validation/h0_primary_split_v11_weighted_full_mean_contract.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "v9_environment_source": "validation/run_h0_primary_split_v9_causal_teacher.py",
    "v9_causal_features": "validation/h0_primary_split_v9_causal_teacher.py",
    "v9_contract": "validation/h0_primary_split_v9_causal_teacher_contract.py",
    "v8r1p1_runner": "validation/run_h0_primary_grf_split_v8r1p1_teacher_replay.py",
    "v8r1p1_contract": "validation/h0_primary_grf_split_v8r1p1_teacher_replay_contract.py",
    "v8r1_runner": "validation/run_h0_primary_grf_split_v8r1_teacher_replay.py",
    "v8r1_contract": "validation/h0_primary_grf_split_v8r1_teacher_replay_contract.py",
    "v8_runner": "validation/run_h0_primary_grf_split_v8_teacher_replay.py",
    "v8_contract": "validation/h0_primary_grf_split_v8_teacher_replay_contract.py",
    "v6_runner": "validation/run_h0_primary_grf_split_v6_teacher_replay.py",
    "v6_contract": "validation/h0_primary_grf_split_v6_teacher_replay_contract.py",
    "v6_qualification_contract": "validation/h0_primary_split_v6_qualification_contract.py",
    "v6_preflight": "validation/build_h0_primary_grf_split_v6_teacher_replay_preflight.py",
    "v25_env_builder": "validation/run_h0_v25_abc_preflight.py",
    "v25_comparator": "validation/compare_h0_v25_abc.py",
    "primary_grf_split_adaptation": (
        "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py"
    ),
    "rollout_eval": "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "process_watchdog": "Trajectory Generator/baseline_MLP/process_watchdog.py",
    "progress_display": "Trajectory Generator/baseline_MLP/progress_display.py",
    "exploration_noise": "Trajectory Generator/baseline_MLP/exploration_noise.py",
    "training_config": "Trajectory Generator/baseline_MLP/training_config.py",
    "bootstrap": "Trajectory Generator/baseline_MLP/_bootstrap.py",
    "win_runtime": "Trajectory Generator/baseline_MLP/win_runtime.py",
    "env_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "reward_function": "Trajectory Generator/baseline_MLP/reward_function.py",
    "morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "asymmetric_rl_module": (
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"
    ),
    "osim_environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_adapter_v26": "Trajectory Generator/binary_phase_adapter_v26.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_fsm_v26": "Trajectory Generator/binary_phase_fsm_v26.py",
    "prosthetic_phase_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "root_config": "config.py",
    "inverse_dynamics": "inverse_dynamics.py",
    "kinematics_interpolator": "kinematics_interpolator.py",
    "model_loader": "model_loader.py",
    "online_grf": "online_grf.py",
    "outer_loop": "outer_loop.py",
    "output": "output.py",
    "path_resolver": "path_resolver.py",
    "prosthesis_controller": "prosthesis_controller.py",
    "setup_io": "setup_io.py",
    "simulation_runner": "simulation_runner.py",
    "static_optimization": "static_optimization.py",
}

EXECUTION_AUTHORITY = copy.deepcopy(contract.DEVELOPMENT_EXECUTION_AUTHORITY)


def _required_contract_value(name: str) -> Any:
    if not hasattr(contract, name):
        raise P1SalvageDevelopmentError(f"contract lacks required value: {name}")
    return getattr(contract, name)


def _raw_relative(value: str | os.PathLike[str] | PurePosixPath) -> str:
    return value.as_posix() if isinstance(value, PurePosixPath) else os.fspath(value)


def resolve_relative(value: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve one canonical repository-relative path without following links."""

    raw = _raw_relative(value)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise P1SalvageDevelopmentError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    path = REPO_ROOT.joinpath(*pure.parts)
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:  # pragma: no cover - defensive on exotic paths.
        raise P1SalvageDevelopmentError(f"repository path escaped: {raw!r}") from exc
    return path


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
PROTOCOL_FREEZE_PATH = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
LOCK_PATH = resolve_relative(contract.EXECUTION_LOCK_PATH)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)
QUALIFICATION_DESIGN_FREEZE_PATH = resolve_relative(
    contract.QUALIFICATION_DESIGN_FREEZE_PATH
)


def _is_link_or_reparse(path: Path) -> bool:
    """Reject POSIX symlinks and Windows reparse points/junctions."""

    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attribute = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attribute & reparse)


def _assert_no_link_components(path: Path, *, allow_missing_leaf: bool = True) -> None:
    """Fail closed if any existing component below the repo is link-like."""

    lexical = path if path.is_absolute() else REPO_ROOT / path
    try:
        relative = lexical.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise P1SalvageDevelopmentError(f"path escaped repository: {path}") from exc
    current = REPO_ROOT
    if _is_link_or_reparse(current):
        raise P1SalvageDevelopmentError("repository root is link-like")
    for index, part in enumerate(relative.parts):
        current = current / part
        if not os.path.lexists(current):
            if allow_missing_leaf or index < len(relative.parts) - 1:
                continue
            raise P1SalvageDevelopmentError(f"required path is missing: {current}")
        if _is_link_or_reparse(current):
            raise P1SalvageDevelopmentError(f"link/reparse path rejected: {current}")


def _mapping(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    value = forensic.strict_json_load(source)
    if not isinstance(value, Mapping):
        raise P1SalvageDevelopmentError(f"expected strict JSON object: {source}")
    return dict(value)


def _canonical_mapping(path: str | Path) -> dict[str, Any]:
    """Read strict JSON and require the producer's canonical byte encoding."""

    source = Path(path)
    value = _mapping(source)
    if source.read_bytes() != forensic.canonical_json_bytes(value):
        raise P1SalvageDevelopmentError(
            f"JSON bytes are not canonical for the decoded payload: {source}"
        )
    return value


def _record(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _record_matches(value: Any, path: str | Path) -> bool:
    return isinstance(value, Mapping) and dict(value) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    _assert_no_link_components(root, allow_missing_leaf=False)
    if not root.is_dir() or _is_link_or_reparse(root):
        raise P1SalvageDevelopmentError(f"artifact tree is unsafe: {root}")
    entries = sorted(root.rglob("*"), key=lambda item: item.as_posix())
    if any(_is_link_or_reparse(item) for item in entries):
        raise P1SalvageDevelopmentError(f"artifact tree contains a link: {root}")
    files = [item for item in entries if item.is_file()]
    if not files:
        raise P1SalvageDevelopmentError(f"artifact tree is empty: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _prospective_json_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    return {
        "path": path.relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _candidate_module_path() -> Path:
    value = _required_contract_value("P1_CANDIDATE_MODULE")
    if isinstance(value, Mapping):
        value = value.get("path")
    if not isinstance(value, (str, os.PathLike, PurePosixPath)):
        raise P1SalvageDevelopmentError("P1 candidate module path is malformed")
    return resolve_relative(value)


def _candidate_tree() -> dict[str, Any]:
    observed = _tree_record(_candidate_module_path())
    expected_id = _required_contract_value("P1_CANDIDATE_ID")
    if not isinstance(expected_id, str) or not expected_id.endswith(
        observed["tree_sha256"][:16]
    ):
        raise P1SalvageDevelopmentError("P1 candidate id/tree binding drifted")
    expected_record = _required_contract_value("P1_CANDIDATE_MODULE")
    if observed != expected_record:
        raise P1SalvageDevelopmentError("P1 candidate module record drifted")
    return observed


def _execution_source_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(relative))
        for name, relative in sorted(EXECUTION_SOURCE_RELATIVE_PATHS.items())
    }


def _runtime_helper_surface_gate() -> dict[str, Any]:
    """Prove that S reaches only inference/physical-audit R3 helpers."""

    source = "\n".join(
        (
            inspect.getsource(_runtime_record),
            inspect.getsource(_frozen_innovations),
            inspect.getsource(_run_rollout),
        )
    )
    required_calls = (
        "r3_runner._runtime_record(",
        "r3_runner._frozen_innovations(",
        "r3_runner._load_rollout_stack(",
        "r3_runner._validate_runtime_layout(",
        "r3_runner._new_physical_audit(",
        "r3_runner._query_mean_std(",
        "r3_runner._consume_physical_step(",
        "r3_runner._physical_summary(",
    )
    forbidden_calls = (
        "r3_runner._run_fit(",
        "r3_runner._run_label(",
        "r3_runner._run_collect(",
        "r3_runner._run_stage(",
        "fit_engine.",
        "label_engine.",
        "v10s_fit.",
        "safe_dagger.select_safe_dagger_action(",
        "safe_dagger.blend_policy_means(",
        "safe_dagger.apply_single_noise(",
        "safe_dagger.SafetyLatchState(",
        "coherent_teacher.build_teacher_view(",
        "coherent_teacher.LegacyGaitShadow(",
    )
    checks = {
        "required_runtime_helpers_once": all(
            source.count(fragment) == 1 for fragment in required_calls
        ),
        "fit_label_collect_dispatch_absent": all(
            fragment not in source for fragment in forbidden_calls
        ),
        "direct_mean_plus_single_noise": "mean + noise" in source,
        "single_checkpoint_load_surface": source.count(
            "RLModule.from_checkpoint(_candidate_module_path())"
        )
        == 1,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "required_calls": list(required_calls),
        "forbidden_calls": list(forbidden_calls),
    }


def _candidate_selection_gate(freeze: Mapping[str, Any]) -> dict[str, Any]:
    gate_function = _required_contract_value("candidate_selection_gate")
    if not callable(gate_function):
        raise P1SalvageDevelopmentError("candidate selection gate is not callable")
    selection = freeze.get("candidate_selection_payload")
    if not isinstance(selection, Mapping):
        raise P1SalvageDevelopmentError(
            "protocol freeze lacks candidate selection payload"
        )
    gate = dict(gate_function(selection))
    if freeze.get("candidate_selection_gate") != gate:
        raise P1SalvageDevelopmentError("frozen candidate selection gate drifted")
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise P1SalvageDevelopmentError(
            f"candidate selection freeze is not PASS: {failed}"
        )
    return gate


def _qualification_design_freeze_binding() -> dict[str, Any]:
    """Reverify the canonical Q design and return its exact current binding."""

    q_declared = q_design_freezer.resolve_relative(
        q_contract.QUALIFICATION_DESIGN_FREEZE_PATH
    )
    constants_exact = (
        q_contract.QUALIFICATION_DESIGN_FREEZE_PATH
        == contract.QUALIFICATION_DESIGN_FREEZE_PATH
        and q_contract.SCHEMA_VERSION == contract.QUALIFICATION_DESIGN_SCHEMA_VERSION
        and q_contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        == contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        and q_contract.PROTOCOL_ID == contract.QUALIFICATION_DESIGN_PROTOCOL_ID
        and q_declared == QUALIFICATION_DESIGN_FREEZE_PATH
    )
    if not constants_exact:
        raise P1SalvageDevelopmentError(
            "qualification design canonical identity drifted"
        )
    try:
        payload = q_design_freezer.verify_design_freeze()
    except Exception as exc:
        raise P1SalvageDevelopmentError(
            "canonical qualification design freeze verification failed"
        ) from exc
    if not isinstance(payload, Mapping):
        raise P1SalvageDevelopmentError(
            "qualification design verifier returned a malformed payload"
        )
    gate = dict(contract.qualification_design_freeze_gate(payload))
    if gate.get("passed") is not True:
        raise P1SalvageDevelopmentError(
            "canonical qualification design semantics are not PASS"
        )
    return {
        "record": _record(QUALIFICATION_DESIGN_FREEZE_PATH),
        "gate": gate,
    }


def _frozen_qualification_design_binding() -> dict[str, Any]:
    """Read the already-frozen binding for terminal failure publication."""

    freeze = _mapping(PROTOCOL_FREEZE_PATH)
    record = freeze.get("qualification_design_freeze")
    gate = freeze.get("qualification_design_freeze_gate")
    if (
        not isinstance(record, Mapping)
        or not isinstance(gate, Mapping)
        or not contract.artifact_record_matches(
            record, contract.QUALIFICATION_DESIGN_FREEZE_PATH
        )
        or gate.get("passed") is not True
    ):
        raise P1SalvageDevelopmentError(
            "frozen qualification design binding is malformed"
        )
    return {"record": dict(record), "gate": dict(gate)}


def _verified_protocol_freeze() -> (
    tuple[dict[str, Any], dict[str, Any], dict[str, Any]]
):
    freeze = _mapping(PROTOCOL_FREEZE_PATH)
    if (
        freeze.get("schema_version") != contract.SCHEMA_VERSION
        or freeze.get("protocol_id") != contract.PROTOCOL_ID
        or freeze.get("passed") is not True
    ):
        raise P1SalvageDevelopmentError("protocol freeze identity/status drifted")
    try:
        import freeze_h0_v12r3_p1_salvage as freeze_engine

        verified = freeze_engine.verify_protocol_freeze()
    except Exception as exc:
        raise P1SalvageDevelopmentError(
            "protocol freeze/current source bindings drifted"
        ) from exc
    if freeze != verified:
        raise P1SalvageDevelopmentError("protocol freeze verification drifted")
    qualification = _qualification_design_freeze_binding()
    if (
        freeze.get("qualification_design_freeze") != qualification["record"]
        or freeze.get("qualification_design_freeze_gate") != qualification["gate"]
    ):
        raise P1SalvageDevelopmentError(
            "protocol/current qualification design binding drifted"
        )
    return freeze, _candidate_selection_gate(freeze), qualification


def _runtime_record() -> dict[str, Any]:
    runtime = dict(r3_runner._runtime_record())
    if runtime.get("inference_stack_ready") is not True:
        raise P1SalvageDevelopmentError("V26 inference runtime is not ready")
    readiness = runtime.get("platform_plugin_readiness")
    if not isinstance(readiness, Mapping) or readiness.get("passed") is not True:
        raise P1SalvageDevelopmentError("platform plugins are not ready")
    return runtime


def _declared_mutation_paths() -> dict[str, str]:
    raw = _required_contract_value("declared_mutation_paths")()
    if not isinstance(raw, Mapping):
        raise P1SalvageDevelopmentError("declared mutation paths are malformed")
    result: dict[str, str] = {}
    for name, value in sorted(raw.items()):
        path = resolve_relative(value)
        relative = path.relative_to(REPO_ROOT).as_posix()
        if not (
            relative.startswith("Trajectory Generator/baseline_MLP/validation/v12p1s/")
            or relative == "Trajectory Generator/baseline_MLP/validation/v12p1s"
        ):
            raise P1SalvageDevelopmentError(
                f"mutation path escaped isolated V12P1S namespace: {name}"
            )
        result[str(name)] = relative
    return result


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Rebuild the exact no-update execution lock without publishing it."""

    _assert_no_link_components(LOCK_PATH)
    _assert_no_link_components(RUN_ROOT)
    if require_unoccupied:
        for path, label in (
            (LOCK_PATH, "execution lock"),
            (RUN_ROOT, "isolated run root"),
            (PIPELINE_CLAIM_PATH, "pipeline claim"),
            (PIPELINE_LEDGER_PATH, "pipeline ledger"),
        ):
            if os.path.lexists(path):
                raise P1SalvageDevelopmentError(f"{label} already exists/no-clobber")
    freeze, selection_gate, qualification_design = _verified_protocol_freeze()
    candidate = _candidate_tree()
    sources = _execution_source_records()
    runtime = _runtime_record()
    runtime_surface = _runtime_helper_surface_gate()
    cases = [
        contract.canonical_development_case(case_id) for case_id in contract.CASE_IDS
    ]
    checks = {
        "protocol_freeze_pass": True,
        "candidate_selection_pass": selection_gate.get("passed") is True,
        "qualification_design_current_and_freeze_exact": (
            freeze.get("qualification_design_freeze") == qualification_design["record"]
            and freeze.get("qualification_design_freeze_gate")
            == qualification_design["gate"]
        ),
        "candidate_id_exact": freeze.get("selected_candidate_id")
        == _required_contract_value("P1_CANDIDATE_ID")
        and freeze.get("selected_candidate") == candidate,
        "candidate_tree_exact": candidate["tree_sha256"].startswith(
            _required_contract_value("P1_CANDIDATE_ID").rsplit(":", 1)[-1]
        ),
        "six_cases_exact": len(cases) == 6
        and len({row["case_id"] for row in cases}) == 6,
        "seven_stages_exact": len(contract.STAGE_IDS) == 7,
        "aggregate_last": contract.STAGE_IDS[-1] == "finalize_development",
        "execution_sources_complete": set(sources)
        == set(EXECUTION_SOURCE_RELATIVE_PATHS),
        "runtime_ready": runtime.get("inference_stack_ready") is True,
        "runtime_helper_surface_closed": runtime_surface.get("passed") is True,
        "isolated_mutation_paths": bool(_declared_mutation_paths()),
        "no_fit_or_update_authority": all(
            EXECUTION_AUTHORITY[name] is False
            for name in (
                "fit_authorized",
                "offline_teacher_labeling_authorized",
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "no_teacher_blend_latch_authority": all(
            EXECUTION_AUTHORITY[name] is False
            for name in (
                "teacher_authorized",
                "blending_authorized",
                "safety_latch_authorized",
            )
        ),
        "lock_prepublication_absent": True,
        "run_root_prepublication_absent": True,
        "claim_prepublication_absent": True,
        "ledger_prepublication_absent": True,
    }
    if not all(value is True for value in checks.values()):
        failed = [name for name, value in checks.items() if value is not True]
        raise P1SalvageDevelopmentError(f"execution lock checks failed: {failed}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": getattr(contract, "EXECUTION_LOCK_PASS_STATUS", LOCK_STATUS),
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "checks": checks,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "candidate_selection_gate": selection_gate,
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": candidate,
        "execution_sources": sources,
        "runtime": runtime,
        "runtime_helper_surface": runtime_surface,
        "stage_order": list(contract.STAGE_IDS),
        "development_cases": copy.deepcopy(cases),
        "run_root": contract.RUN_ROOT.as_posix(),
        "declared_mutation_paths": _declared_mutation_paths(),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_P1_SALVAGE_DEVELOPMENT_ONCE",
    }


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise P1SalvageDevelopmentError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return verify_execution_lock(require_run_root_absent=True)


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    observed = _canonical_mapping(LOCK_PATH)
    expected = build_execution_lock(require_unoccupied=False)
    if observed != expected:
        raise P1SalvageDevelopmentError("execution lock/current bindings drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise P1SalvageDevelopmentError("isolated run root already claimed")
    return observed


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise P1SalvageDevelopmentError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _claim_path(stage_id: str) -> Path:
    return resolve_relative(contract.worker_claim_path(stage_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return resolve_relative(contract.stage_receipt_path(stage_id))


def _stage_failure_path(stage_id: str) -> Path:
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor["kind"] == "development":
        return resolve_relative(descriptor["case"]["destination"]) / "failure.json"
    if descriptor["kind"] == "finalize_development":
        return resolve_relative(contract.FINAL_DEVELOPMENT_FAILURE_PATH)
    raise P1SalvageDevelopmentError(f"unauthorized stage kind: {descriptor['kind']!r}")


def _pipeline_claim_payload(token_sha256: str) -> dict[str, Any]:
    qualification_design = _qualification_design_freeze_binding()
    lock = _mapping(LOCK_PATH)
    if (
        lock.get("qualification_design_freeze") != qualification_design["record"]
        or lock.get("qualification_design_freeze_gate") != qualification_design["gate"]
    ):
        raise P1SalvageDevelopmentError(
            "execution-lock/current qualification design binding drifted"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK_PATH),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_authorized": False,
        "actor_updates_authorized": False,
        "critic_updates_authorized": False,
        "ppo_updates_authorized": False,
        "teacher_authorized": False,
        "blending_authorized": False,
        "safety_latch_authorized": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _verify_pipeline_claim() -> dict[str, Any]:
    claim = _canonical_mapping(PIPELINE_CLAIM_PATH)
    digest = claim.get("execution_token_sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise P1SalvageDevelopmentError("pipeline claim token digest is malformed")
    if claim != _pipeline_claim_payload(digest):
        raise P1SalvageDevelopmentError("pipeline claim/current lock binding drifted")
    return claim


def _worker_claim_payload(
    *, stage_id: str, token_sha256: str, previous_receipts: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    qualification_design = _qualification_design_freeze_binding()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": WORKER_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "stage_index": contract.STAGE_IDS.index(stage_id),
        "stage_kind": descriptor["kind"],
        "candidate_id": contract.P1_CANDIDATE_ID,
        "execution_token_sha256": token_sha256,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "previous_receipts": [dict(row) for row in previous_receipts],
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _verify_worker_claim(stage_id: str) -> dict[str, Any]:
    pipeline = _verify_pipeline_claim()
    index = contract.STAGE_IDS.index(stage_id)
    previous = [
        {"stage_id": prior, "receipt": _record(_stage_receipt_path(prior))}
        for prior in contract.STAGE_IDS[:index]
    ]
    expected = _worker_claim_payload(
        stage_id=stage_id,
        token_sha256=str(pipeline["execution_token_sha256"]),
        previous_receipts=previous,
    )
    observed = _canonical_mapping(_claim_path(stage_id))
    if observed != expected:
        raise P1SalvageDevelopmentError(f"worker claim chain drifted: {stage_id}")
    return observed


def _write_worker_claim(stage_id: str, token_sha256: str) -> Path:
    _verify_pipeline_claim()
    index = contract.STAGE_IDS.index(stage_id)
    previous: list[dict[str, Any]] = []
    for prior in contract.STAGE_IDS[:index]:
        _verify_stage_receipt(prior, require_pass=True)
        previous.append(
            {"stage_id": prior, "receipt": _record(_stage_receipt_path(prior))}
        )
    for later in contract.STAGE_IDS[index + 1 :]:
        if any(
            os.path.lexists(path)
            for path in (
                _claim_path(later),
                _stage_receipt_path(later),
                _stage_failure_path(later),
            )
        ):
            raise P1SalvageDevelopmentError(
                f"later stage artifacts predate {stage_id}: {later}"
            )
    claim_path = _claim_path(stage_id)
    if any(
        os.path.lexists(path)
        for path in (
            claim_path,
            _stage_receipt_path(stage_id),
            _stage_failure_path(stage_id),
        )
    ):
        raise P1SalvageDevelopmentError(f"stage already consumed: {stage_id}")
    return forensic.write_json_exclusive(
        claim_path,
        _worker_claim_payload(
            stage_id=stage_id,
            token_sha256=token_sha256,
            previous_receipts=previous,
        ),
    )


def _claim_run_root() -> str:
    if os.path.lexists(RUN_ROOT):
        raise P1SalvageDevelopmentError("isolated run root already exists/no-resume")
    _assert_no_link_components(RUN_ROOT.parent, allow_missing_leaf=False)
    root_claimed = False
    claim_published = False
    claim_verified = False
    pipeline_claim_record: dict[str, Any] | None = None
    try:
        RUN_ROOT.mkdir(parents=False, exist_ok=False)
        root_claimed = True
        token = secrets.token_urlsafe(48)
        forensic.write_json_exclusive(
            PIPELINE_CLAIM_PATH, _pipeline_claim_payload(_token_sha256(token))
        )
        claim_published = True
        pipeline_claim_record = _record(PIPELINE_CLAIM_PATH)
        _verify_pipeline_claim()
        claim_verified = True
        return token
    except BaseException as exc:
        if root_claimed and not os.path.lexists(PIPELINE_LEDGER_PATH):
            failure_ledger = _preclaim_failure_ledger(
                exc,
                claim_published=claim_published,
                claim_verified=claim_verified,
                pipeline_claim_record=pipeline_claim_record,
            )
            forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, failure_ledger)
            _verify_preclaim_failure_ledger(failure_ledger)
        raise


def _frozen_innovations(case: Mapping[str, Any], *, np: Any) -> Any:
    selection = case.get("action_selection")
    shape = (EXPECTED_STEPS, EXPECTED_ACTION_DIM)
    if selection not in {"deterministic", "stochastic"}:
        raise P1SalvageDevelopmentError(f"unknown action selection: {selection!r}")
    if selection == "stochastic" and type(case.get("action_seed")) is not int:
        raise P1SalvageDevelopmentError("stochastic action seed is malformed")
    # Reconstruct the authoritative float32 tape from the frozen corpus.  It
    # is deliberately not replaced by a fresh RNG draw from the same seed.
    values = r3_runner._frozen_innovations(
        str(case.get("case_id")), action_selection=str(selection), np=np
    )
    values = np.ascontiguousarray(values, dtype=np.float32)
    if values.shape != shape or not np.all(np.isfinite(values)):
        raise P1SalvageDevelopmentError("frozen innovation matrix is malformed")
    return values


def _rollout_records(
    writer: Any,
    *,
    rows: Sequence[Mapping[str, Any]],
    partial: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    return {
        "run_start": _record(writer.run_start_path),
        "trace": _prospective_json_record(writer.trace_path, list(rows)),
        "partial_summary": _prospective_json_record(
            writer.partial_summary_path, partial
        ),
    }


def _publish_rollout_before_gate(
    writer: Any,
    *,
    rows: Sequence[Mapping[str, Any]],
    partial: Mapping[str, Any],
    summary: Mapping[str, Any],
) -> None:
    expected = _rollout_records(writer, rows=rows, partial=partial)
    persisted = dict(
        writer.finalize_before_gate(
            trace=rows,
            partial_summary=partial,
            summary=summary,
        )
    )
    # Shared writer does not return run_start: re-read after finalization to
    # close the pre/post-finalization TOCTOU window explicitly.
    persisted["run_start"] = _record(writer.run_start_path)
    for name, record in expected.items():
        if persisted.get(name) != record:
            raise P1SalvageDevelopmentError(
                f"post-finalization rollout record drifted: {name}"
            )


_ACTIVITY = {
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "candidate_mean_queries": 0,
}

EXPECTED_TERMINAL_ACTIVITY = {
    "environment_reset_calls": len(contract.CASE_IDS),
    "environment_step_calls": len(contract.CASE_IDS) * EXPECTED_STEPS,
    "candidate_mean_queries": len(contract.CASE_IDS) * EXPECTED_STEPS,
}


def _increment(name: str, amount: int = 1) -> None:
    if name not in _ACTIVITY or type(amount) is not int or amount < 0:
        raise P1SalvageDevelopmentError(f"activity increment is malformed: {name}")
    _ACTIVITY[name] += amount


def _assert_terminal_activity_exact() -> None:
    if _ACTIVITY != EXPECTED_TERMINAL_ACTIVITY:
        raise P1SalvageDevelopmentError(
            f"terminal activity counters drifted: {_ACTIVITY}"
        )


def _rollout_gate(summary: Mapping[str, Any], case_id: str) -> dict[str, Any]:
    function = _required_contract_value("development_rollout_gate")
    try:
        return dict(function(summary, case_id=case_id))
    except TypeError as exc:
        raise P1SalvageDevelopmentError(
            "development_rollout_gate must accept (summary, case_id=...)"
        ) from exc


def _build_rollout_summary(
    *,
    physical: Mapping[str, Any],
    case_id: str,
    stage_id: str,
    row_count: int,
    candidate_tree: Mapping[str, Any],
    artifacts: Mapping[str, Mapping[str, Any]],
    protocol_freeze: Mapping[str, Any],
    execution_lock: Mapping[str, Any],
    pipeline_claim: Mapping[str, Any],
    worker_claim: Mapping[str, Any],
    qualification_design_freeze: Mapping[str, Any],
    qualification_design_freeze_gate: Mapping[str, Any],
) -> dict[str, Any]:
    """Add immutable autonomy/provenance fields to one physical summary."""

    return {
        **dict(physical),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "behavior": BEHAVIOR,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": dict(candidate_tree),
        "candidate_selection_gate_passed": True,
        "trace_step_count": row_count,
        "candidate_mean_query_count": row_count,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "teacher_query_count": 0,
        "teacher_queries": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "blend_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "latch_active_at_episode_end": False,
        "latch_count": 0,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "physical_gate_bypass_count": 0,
        "target_contract_id": TARGET_CONTRACT_ID,
        "logstd_byte_exact": True,
        "disabled_clock_column_indices": list(contract.DISABLED_CLOCK_COLUMN_INDICES),
        "disabled_clock_columns_bit_zero": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "worker_claim": dict(worker_claim),
        "protocol_freeze": dict(protocol_freeze),
        "execution_lock": dict(execution_lock),
        "pipeline_claim": dict(pipeline_claim),
        "qualification_design_freeze": dict(qualification_design_freeze),
        "qualification_design_freeze_gate": dict(qualification_design_freeze_gate),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "rollout_executions": 1,
        "environment_reset_calls": 1,
        "environment_step_calls": row_count,
        "candidate_mean_queries": row_count,
        "fit_executions": 0,
        "actor_fit_executions": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "p2_artifacts_opened": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "development_only": True,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        **{name: dict(record) for name, record in artifacts.items()},
    }


def _trace_row_schema_valid(row: Mapping[str, Any]) -> bool:
    vectors = {
        "v26_observation": EXPECTED_ACTOR_FEATURES,
        "candidate_mean": EXPECTED_ACTION_DIM,
        "candidate_std": EXPECTED_ACTION_DIM,
        "standard_normal": EXPECTED_ACTION_DIM,
        "single_noise": EXPECTED_ACTION_DIM,
        "raw_action": EXPECTED_ACTION_DIM,
        "applied_action": EXPECTED_ACTION_DIM,
    }
    return (
        all(
            isinstance(row.get(name), list) and len(row[name]) == width
            for name, width in vectors.items()
        )
        and row.get("protocol_id") == contract.PROTOCOL_ID
        and row.get("teacher_enabled") is False
        and row.get("teacher_query_count") == 0
        and row.get("served_action_teacher_dependency_count") == 0
        and row.get("blending_enabled") is False
        and row.get("safety_latch_enabled") is False
    )


def _run_rollout(case_id: str, stage_id: str) -> dict[str, Any]:
    case = contract.canonical_development_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise P1SalvageDevelopmentError(f"rollout destination exists: {case_id}")
    qualification_design = _qualification_design_freeze_binding()
    pipeline_claim = _verify_pipeline_claim()
    if (
        pipeline_claim.get("qualification_design_freeze")
        != qualification_design["record"]
        or pipeline_claim.get("qualification_design_freeze_gate")
        != qualification_design["gate"]
    ):
        raise P1SalvageDevelopmentError(
            "pipeline/current qualification design binding drifted"
        )
    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = r3_runner._load_rollout_stack()
    candidate_tree = _candidate_tree()
    candidate = RLModule.from_checkpoint(_candidate_module_path())
    candidate.eval()
    innovations = _frozen_innovations(case, np=np)
    env_config = env_source.build_env_config(case)
    env = None
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_STARTED_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "behavior": BEHAVIOR,
            "candidate_id": contract.P1_CANDIDATE_ID,
            "candidate_module": candidate_tree,
            "execution_lock": _record(LOCK_PATH),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "qualification_design_freeze": copy.deepcopy(
                qualification_design["record"]
            ),
            "qualification_design_freeze_gate": copy.deepcopy(
                qualification_design["gate"]
            ),
            "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
            "rollout_executions": 1,
            "environment_reset_calls_expected": 1,
            "environment_step_calls_expected": EXPECTED_STEPS,
            "candidate_mean_queries_expected": EXPECTED_STEPS,
            "teacher_enabled": False,
            "teacher_loaded_during_rollout": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "fit_executions": 0,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "retry_authorized": False,
            "resume_authorized": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    started = time.monotonic()
    try:
        env = env_factory.make_cmc_env(env_config)
        _increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = r3_runner._validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = r3_runner._new_physical_audit(
            reset_info=reset_info, legacy=legacy, np=np
        )
        actor = np.ascontiguousarray(
            observation[:EXPECTED_ACTOR_FEATURES], dtype=np.float32
        )
        for index in range(EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor_before = actor.copy()
            mean, std = r3_runner._query_mean_std(
                candidate, actor_before, np=np, torch=torch
            )
            _increment("candidate_mean_queries")
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            # Exactly one noise application; no blend/teacher/latch path exists.
            raw_action = np.ascontiguousarray(mean + noise, dtype=np.float32)
            applied_action = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(
                applied_action
            )
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise P1SalvageDevelopmentError("rollout info is malformed")
            physical = r3_runner._consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied_action,
                extra_vectors=(actor_before, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": actor_before.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied_action.tolist(),
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            if not _trace_row_schema_valid(row):
                raise P1SalvageDevelopmentError(
                    f"forensic trace row schema drifted: {case_id}/{step}"
                )
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            actor = np.ascontiguousarray(
                observation_after[:EXPECTED_ACTOR_FEATURES], dtype=np.float32
            )
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (EXPECTED_STEPS - step)
                print(
                    f"[V12P1S {case_id}] {step:3d}/{EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        if not os.path.lexists(writer.gate_path) and not os.path.lexists(
            writer.failure_path
        ):
            writer.publish_failure(
                end_reason="v12r3_p1_salvage_rollout_failed_terminal_no_retry",
                error=exc,
                status=STAGE_FAILURE_STATUS,
                details={
                    "schema_version": contract.SCHEMA_VERSION,
                    "protocol_id": contract.PROTOCOL_ID,
                    "pipeline_id": contract.PIPELINE_ID,
                    "stage_id": stage_id,
                    "case_id": case_id,
                    "candidate_id": contract.P1_CANDIDATE_ID,
                    "candidate_module": candidate_tree,
                    "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
                    "execution_lock": _record(LOCK_PATH),
                    "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
                    "worker_claim": _record(_claim_path(stage_id)),
                    "qualification_design_freeze": copy.deepcopy(
                        qualification_design["record"]
                    ),
                    "qualification_design_freeze_gate": copy.deepcopy(
                        qualification_design["gate"]
                    ),
                    "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
                    "activity": copy.deepcopy(_ACTIVITY),
                    "retry_authorized": False,
                    "resume_authorized": False,
                    "fit_executions": 0,
                    "actor_updates": 0,
                    "critic_updates": 0,
                    "ppo_updates": 0,
                    "teacher_queries": 0,
                    "blend_count": 0,
                    "latch_count": 0,
                    "p2_artifacts_opened": [],
                    "p2_module_loaded": False,
                    "p2_corpus_loaded": False,
                    "protected_trials_opened": [],
                    "reserve_trials_opened": [],
                    "runtime_promoted": False,
                },
            )
        raise
    finally:
        if env is not None:
            env.close()
    if audit is None:
        raise P1SalvageDevelopmentError("physical audit was not initialized")
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": ROLLOUT_PERSISTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "steps": len(rows),
        "gate_evaluated": False,
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "rollout_executions": 1,
        "environment_reset_calls": 1,
        "environment_step_calls": len(rows),
        "candidate_mean_queries": len(rows),
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    artifacts = _rollout_records(writer, rows=rows, partial=partial)
    physical_summary = r3_runner._physical_summary(
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
    summary = _build_rollout_summary(
        physical=physical_summary,
        case_id=case_id,
        stage_id=stage_id,
        row_count=len(rows),
        candidate_tree=candidate_tree,
        artifacts=artifacts,
        protocol_freeze=_record(PROTOCOL_FREEZE_PATH),
        execution_lock=_record(LOCK_PATH),
        pipeline_claim=_record(PIPELINE_CLAIM_PATH),
        worker_claim=_record(_claim_path(stage_id)),
        qualification_design_freeze=qualification_design["record"],
        qualification_design_freeze_gate=qualification_design["gate"],
    )
    _publish_rollout_before_gate(writer, rows=rows, partial=partial, summary=summary)
    gate = _rollout_gate(summary, case_id)
    writer.publish_gate(gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate.get("status"),
        "passed": gate.get("passed") is True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module_tree_sha256": candidate_tree["tree_sha256"],
        "steps": len(rows),
        "summary": _record(writer.summary_path),
        "gate": _record(writer.gate_path),
        "trace": _record(writer.trace_path),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), receipt)
    _verify_stage_receipt(stage_id, require_pass=False)
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise P1SalvageDevelopmentError(
            f"development rollout gate failed terminally: {case_id}: {failed}"
        )
    return receipt


def _aggregate_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    function = _required_contract_value("final_development_gate")
    return dict(function(summary))


def _run_aggregate(stage_id: str) -> dict[str, Any]:
    final_root = resolve_relative(contract.FINAL_ROOT)
    if os.path.lexists(final_root):
        raise P1SalvageDevelopmentError("aggregate destination exists")
    _assert_terminal_activity_exact()
    qualification_design = _qualification_design_freeze_binding()
    pipeline_claim = _verify_pipeline_claim()
    if (
        pipeline_claim.get("qualification_design_freeze")
        != qualification_design["record"]
        or pipeline_claim.get("qualification_design_freeze_gate")
        != qualification_design["gate"]
    ):
        raise P1SalvageDevelopmentError(
            "aggregate qualification design binding drifted"
        )
    rollout_bindings: list[dict[str, Any]] = []
    for case_id in contract.CASE_IDS:
        rollout_stage = next(
            stage
            for stage in contract.STAGE_IDS[:-1]
            if contract.stage_descriptor(stage).get("case", {}).get("case_id")
            == case_id
        )
        receipt = _verify_stage_receipt(rollout_stage, require_pass=True)
        rollout_bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(_stage_receipt_path(rollout_stage)),
                "summary": dict(receipt["summary"]),
                "gate": dict(receipt["gate"]),
            }
        )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": _candidate_tree(),
        "candidate_selection_gate_passed": True,
        "case_order": list(contract.CASE_IDS),
        "rollout_bindings": rollout_bindings,
        "rollout_count": len(rollout_bindings),
        "passing_rollout_count": len(rollout_bindings),
        "failed_rollout_count": 0,
        "all_cases_required": True,
        "compensation_authorized": False,
        "best_k_authorized": False,
        "case_drop_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "fit_executions": 0,
        "actor_fit_executions": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "environment_reset_calls_outside_rollouts": 0,
        "environment_step_calls_outside_rollouts": 0,
        "rollout_environment_reset_calls": _ACTIVITY["environment_reset_calls"],
        "rollout_environment_step_calls": _ACTIVITY["environment_step_calls"],
        "candidate_mean_queries": _ACTIVITY["candidate_mean_queries"],
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "p2_artifacts_opened": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "qualification_executed": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
    }
    final_root.mkdir(parents=True, exist_ok=False)
    summary_path = forensic.write_json_exclusive(final_root / "summary.json", summary)
    gate = _aggregate_gate(summary)
    gate_path = forensic.write_json_exclusive(final_root / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate.get("status"),
        "passed": gate.get("passed") is True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "case_count": len(rollout_bindings),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "case_receipts": [dict(row["receipt"]) for row in rollout_bindings],
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), receipt)
    _verify_stage_receipt(stage_id, require_pass=False)
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise P1SalvageDevelopmentError(
            f"aggregate 6/6 gate failed terminally: {failed}"
        )
    return receipt


def _verify_stage_receipt(
    stage_id: str, *, require_pass: bool = True
) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    receipt_path = _stage_receipt_path(stage_id)
    receipt = _canonical_mapping(receipt_path)
    _verify_worker_claim(stage_id)
    qualification_design = _qualification_design_freeze_binding()
    common = (
        receipt.get("schema_version") == contract.SCHEMA_VERSION
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("candidate_id") == contract.P1_CANDIDATE_ID
        and _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM_PATH)
        and _record_matches(receipt.get("worker_claim"), _claim_path(stage_id))
        and receipt.get("qualification_design_freeze") == qualification_design["record"]
        and receipt.get("qualification_design_freeze_gate")
        == qualification_design["gate"]
        and receipt.get("execution_authority") == EXECUTION_AUTHORITY
        and receipt.get("retry_authorized") is False
        and receipt.get("resume_authorized") is False
        and receipt.get("fit_executions") == 0
        and receipt.get("actor_updates") == 0
        and receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0
        and receipt.get("teacher_queries") == 0
        and receipt.get("blend_count") == 0
        and receipt.get("latch_count") == 0
        and receipt.get("protected_trials_opened") == []
        and receipt.get("reserve_trials_opened") == []
    )
    if not common:
        raise P1SalvageDevelopmentError(f"stage receipt drifted: {stage_id}")
    kind = descriptor["kind"]
    if kind == "development":
        case_id = descriptor["case"]["case_id"]
        destination = resolve_relative(
            contract.canonical_development_case(case_id)["destination"]
        )
        summary_path = destination / "summary.json"
        gate_path = destination / "gate.json"
        summary = _canonical_mapping(summary_path)
        gate = _canonical_mapping(gate_path)
        expected_gate = _rollout_gate(summary, case_id)
        if (
            receipt.get("case_id") != case_id
            or not _record_matches(receipt.get("summary"), summary_path)
            or not _record_matches(receipt.get("gate"), gate_path)
            or not _record_matches(receipt.get("trace"), destination / "trace.json")
            or summary.get("qualification_design_freeze")
            != qualification_design["record"]
            or summary.get("qualification_design_freeze_gate")
            != qualification_design["gate"]
            or gate != expected_gate
            or receipt.get("passed") is not (gate.get("passed") is True)
        ):
            raise P1SalvageDevelopmentError(f"rollout receipt/gate drifted: {stage_id}")
    elif kind == "finalize_development":
        final_root = resolve_relative(contract.FINAL_ROOT)
        summary = _canonical_mapping(final_root / "summary.json")
        gate = _canonical_mapping(final_root / "gate.json")
        expected_gate = _aggregate_gate(summary)
        expected_case_receipts = [
            _record(_stage_receipt_path(prior)) for prior in contract.STAGE_IDS[:-1]
        ]
        if (
            not _record_matches(receipt.get("summary"), final_root / "summary.json")
            or not _record_matches(receipt.get("gate"), final_root / "gate.json")
            or gate != expected_gate
            or receipt.get("passed") is not (gate.get("passed") is True)
            or receipt.get("case_count") != 6
            or receipt.get("case_receipts") != expected_case_receipts
            or summary.get("qualification_design_freeze")
            != qualification_design["record"]
            or summary.get("qualification_design_freeze_gate")
            != qualification_design["gate"]
        ):
            raise P1SalvageDevelopmentError("aggregate receipt/gate drifted")
    else:
        raise P1SalvageDevelopmentError(f"unauthorized stage kind: {kind!r}")
    if require_pass and receipt.get("passed") is not True:
        raise P1SalvageDevelopmentError(f"stage is not PASS: {stage_id}")
    return receipt


def _publish_stage_failure(stage_id: str, error: BaseException) -> dict[str, Any]:
    path = _stage_failure_path(stage_id)
    if os.path.lexists(path):
        return _mapping(path)
    artifacts: list[dict[str, Any]] = []
    root = path.parent
    if root.is_dir() and not _is_link_or_reparse(root):
        for item in sorted(root.rglob("*"), key=lambda value: value.as_posix()):
            if item == path or not item.is_file() or _is_link_or_reparse(item):
                continue
            artifacts.append(_record(item))
    qualification_design = _frozen_qualification_design_binding()
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": STAGE_FAILURE_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "error_type": type(error).__name__,
        "error": str(error),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "worker_claim": (
            _record(_claim_path(stage_id))
            if os.path.lexists(_claim_path(stage_id))
            else None
        ),
        "published_artifacts": artifacts,
        "activity": copy.deepcopy(_ACTIVITY),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "p2_artifacts_opened": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }
    forensic.write_json_exclusive(path, payload)
    return _mapping(path)


def _run_stage(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor["kind"] == "development":
        return _run_rollout(descriptor["case"]["case_id"], stage_id)
    if descriptor["kind"] == "finalize_development":
        return _run_aggregate(stage_id)
    raise P1SalvageDevelopmentError(
        f"contract exposed unauthorized stage kind: {descriptor['kind']!r}"
    )


def _ledger_payload(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed_stages: Sequence[str],
    error: BaseException | None,
    stage_failure: Mapping[str, Any] | None,
    activity: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    qualification_design = _frozen_qualification_design_binding()
    bound_activity = _ACTIVITY if activity is None else activity
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LEDGER_PASS_STATUS if passed else LEDGER_FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_module": (
            _candidate_tree() if passed else copy.deepcopy(contract.P1_CANDIDATE_MODULE)
        ),
        "stage_order": list(contract.STAGE_IDS),
        "attempted_stage": attempted_stage,
        "completed_stages": list(completed_stages),
        "completed_receipts": [
            {"stage_id": stage, "receipt": _record(_stage_receipt_path(stage))}
            for stage in completed_stages
        ],
        "failed_stage_receipt": (
            dict(stage_failure) if stage_failure is not None else None
        ),
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "activity": copy.deepcopy(dict(bound_activity)),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "expected_environment_reset_calls": 6,
        "expected_environment_step_calls": 6 * EXPECTED_STEPS,
        "expected_control_windows": 6 * EXPECTED_CONTROL_WINDOWS,
        "aggregate_requires_6_of_6": True,
        "compensation_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }


def _preclaim_failure_ledger(
    error: BaseException,
    *,
    claim_published: bool,
    claim_verified: bool,
    pipeline_claim_record: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Build a terminal ledger after this process claimed the root but failed.

    The pipeline claim is optional here because its exclusive publication or
    post-publication verification may be the failing operation itself.
    """

    if claim_verified and not claim_published:
        raise P1SalvageDevelopmentError(
            "a pipeline claim cannot be verified before it is published"
        )
    if pipeline_claim_record is not None and not claim_published:
        raise P1SalvageDevelopmentError(
            "a pipeline claim record cannot predate exclusive publication"
        )
    qualification_design = _frozen_qualification_design_binding()
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
        "failed_stage_receipt": None,
        "error_type": type(error).__name__,
        "error": str(error),
        "execution_lock": _record(LOCK_PATH),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "pipeline_claim": (
            dict(pipeline_claim_record) if pipeline_claim_record is not None else None
        ),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "activity": copy.deepcopy(_ACTIVITY),
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "claim_published": claim_published,
        "claim_verified": claim_verified,
        "claim_completed": claim_verified,
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "teacher_queries": 0,
        "blend_count": 0,
        "latch_count": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }


def _verify_preclaim_failure_ledger(
    expected: Mapping[str, Any],
) -> dict[str, Any]:
    observed = _canonical_mapping(PIPELINE_LEDGER_PATH)
    if (
        observed != dict(expected)
        or observed.get("schema_version") != contract.SCHEMA_VERSION
        or observed.get("status") != LEDGER_FAIL_STATUS
        or observed.get("passed") is not False
        or observed.get("terminal") is not True
        or observed.get("attempted_stage") is not None
        or observed.get("completed_stages") != []
        or observed.get("claim_completed") is not observed.get("claim_verified")
    ):
        raise P1SalvageDevelopmentError("preclaim failure ledger drifted")
    return observed


def _failure_activity_is_consistent(
    activity: Any, *, completed_stages: Sequence[str]
) -> bool:
    if (
        not isinstance(activity, Mapping)
        or set(activity) != set(EXPECTED_TERMINAL_ACTIVITY)
        or not all(type(value) is int and value >= 0 for value in activity.values())
    ):
        return False
    completed_rollouts = sum(
        contract.stage_descriptor(stage)["kind"] == "development"
        for stage in completed_stages
    )
    resets = activity["environment_reset_calls"]
    steps = activity["environment_step_calls"]
    queries = activity["candidate_mean_queries"]
    minimum = completed_rollouts * EXPECTED_STEPS
    return (
        completed_rollouts <= resets <= min(completed_rollouts + 1, 6)
        and minimum <= steps <= resets * EXPECTED_STEPS
        and steps <= queries <= min(steps + 1, resets * EXPECTED_STEPS)
    )


def verify_terminal_ledger(
    *,
    require_pass: bool = True,
    expected_failure: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Byte-verify terminal PASS or rigorously close a terminal failure."""

    observed = _canonical_mapping(PIPELINE_LEDGER_PATH)
    if (
        observed.get("schema_version") != contract.SCHEMA_VERSION
        or observed.get("protocol_id") != contract.PROTOCOL_ID
        or observed.get("pipeline_id") != contract.PIPELINE_ID
        or observed.get("terminal") is not True
    ):
        raise P1SalvageDevelopmentError("terminal ledger identity drifted")
    if observed.get("passed") is not True:
        if require_pass:
            raise P1SalvageDevelopmentError("terminal ledger is not PASS")
        attempted = observed.get("attempted_stage")
        if attempted not in contract.STAGE_IDS:
            raise P1SalvageDevelopmentError(
                "post-claim failure ledger has no canonical attempted stage"
            )
        index = contract.STAGE_IDS.index(str(attempted))
        expected_completed = list(contract.STAGE_IDS[:index])
        completed = observed.get("completed_stages")
        completed_receipts = observed.get("completed_receipts")
        qualification_design = _frozen_qualification_design_binding()
        failure_common = (
            observed.get("status") == LEDGER_FAIL_STATUS
            and completed == expected_completed
            and isinstance(completed_receipts, list)
            and completed_receipts
            == [
                {"stage_id": stage, "receipt": _record(_stage_receipt_path(stage))}
                for stage in expected_completed
            ]
            and isinstance(observed.get("failed_stage_receipt"), Mapping)
            and _record_matches(
                observed.get("failed_stage_receipt"),
                _stage_failure_path(str(attempted)),
            )
            and _record_matches(observed.get("protocol_freeze"), PROTOCOL_FREEZE_PATH)
            and _record_matches(observed.get("execution_lock"), LOCK_PATH)
            and _record_matches(observed.get("pipeline_claim"), PIPELINE_CLAIM_PATH)
            and observed.get("qualification_design_freeze")
            == qualification_design["record"]
            and observed.get("qualification_design_freeze_gate")
            == qualification_design["gate"]
            and observed.get("execution_authority") == EXECUTION_AUTHORITY
            and _failure_activity_is_consistent(
                observed.get("activity"), completed_stages=expected_completed
            )
            and observed.get("fit_executions") == 0
            and observed.get("actor_updates") == 0
            and observed.get("critic_updates") == 0
            and observed.get("ppo_updates") == 0
            and observed.get("teacher_queries") == 0
            and observed.get("blend_count") == 0
            and observed.get("latch_count") == 0
            and observed.get("protected_trials_opened") == []
            and observed.get("reserve_trials_opened") == []
            and observed.get("runtime_promoted") is False
            and isinstance(observed.get("error_type"), str)
            and isinstance(observed.get("error"), str)
        )
        if not failure_common:
            raise P1SalvageDevelopmentError("terminal failure ledger drifted")
        for stage_id in expected_completed:
            _verify_stage_receipt(stage_id, require_pass=True)
        if expected_failure is not None and observed != dict(expected_failure):
            raise P1SalvageDevelopmentError(
                "terminal failure ledger differs from its constructed payload"
            )
        return observed

    if observed.get("activity") != EXPECTED_TERMINAL_ACTIVITY:
        raise P1SalvageDevelopmentError("terminal PASS activity drifted")
    for stage_id in contract.STAGE_IDS:
        _verify_stage_receipt(stage_id, require_pass=True)
    expected = _ledger_payload(
        passed=True,
        attempted_stage=None,
        completed_stages=contract.STAGE_IDS,
        error=None,
        stage_failure=None,
        activity=EXPECTED_TERMINAL_ACTIVITY,
    )
    if observed != expected:
        raise P1SalvageDevelopmentError("terminal PASS ledger/current chain drifted")
    return observed


def execute_development_once() -> dict[str, Any]:
    """Execute all six fixed rollouts and the aggregate stage exactly once."""

    verify_execution_lock(require_run_root_absent=True)
    for name in _ACTIVITY:
        _ACTIVITY[name] = 0
    completed: list[str] = []
    attempted: str | None = None
    token: str | None = None
    claim_completed = False
    try:
        token = _claim_run_root()
        claim_completed = True
        token_sha256 = _token_sha256(token)
        token = None
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            _write_worker_claim(stage_id, token_sha256)
            receipt = _run_stage(stage_id)
            if receipt.get("passed") is not True:
                raise P1SalvageDevelopmentError(
                    f"stage returned a non-PASS receipt: {stage_id}"
                )
            _verify_stage_receipt(stage_id, require_pass=True)
            completed.append(stage_id)
        if len(completed) != 7 or completed != list(contract.STAGE_IDS):
            raise P1SalvageDevelopmentError("seven-stage completion order drifted")
        _assert_terminal_activity_exact()
        ledger = _ledger_payload(
            passed=True,
            attempted_stage=None,
            completed_stages=completed,
            error=None,
            stage_failure=None,
        )
        forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
        return verify_terminal_ledger(require_pass=True)
    except BaseException as exc:
        failure: dict[str, Any] | None = None
        if attempted is not None:
            failure = _publish_stage_failure(attempted, exc)
        if claim_completed and not os.path.lexists(PIPELINE_LEDGER_PATH):
            failure_ledger = _ledger_payload(
                passed=False,
                attempted_stage=attempted,
                completed_stages=completed,
                error=exc,
                stage_failure=(
                    _record(_stage_failure_path(attempted))
                    if attempted is not None
                    and os.path.lexists(_stage_failure_path(attempted))
                    else failure
                ),
            )
            forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, failure_ledger)
            verify_terminal_ledger(require_pass=False, expected_failure=failure_ledger)
        raise
    finally:
        token = None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze or execute the isolated V12R3/P1 salvage development run."
    )
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--prepare-execution-lock",
        action="store_true",
        help="Publish the no-clobber execution lock; does not claim or run.",
    )
    action.add_argument(
        "--execute-once",
        action="store_true",
        help="Verify the existing lock and execute the seven stages once.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_execution_lock:
        result = prepare_execution_lock()
    else:
        result = execute_development_once()
    print(result["status"])
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "P1SalvageDevelopmentError",
    "build_execution_lock",
    "execute_development_once",
    "main",
    "prepare_execution_lock",
    "verify_execution_lock",
]
