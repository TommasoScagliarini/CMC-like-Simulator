"""Sole fresh-H0 fitter for the V12R5 case-balanced candidate.

The fitter reads exactly two label components: the immutable 8,732-row P2
corpus and the completed 500-row nominal V12R4 collection.  It verifies but
never loads the failed V12R4 ``+0.20`` label NPZ.  P2 is used only as a frozen
diagnostic model for the preregistered hardness weights and never as the
initial checkpoint.
"""

from __future__ import annotations

import copy
import math
import os
import sys
from pathlib import Path, PurePath, PurePosixPath
from typing import Any, Callable, Mapping, Sequence

import numpy as np


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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import h0_primary_split_v11_weighted_fit as v11  # noqa: E402
import h0_primary_split_v12r3_recovery_weighted_fitter as v12r3_fit  # noqa: E402
import h0_v12r5_case_balanced_contract as contract  # noqa: E402
from freeze_h0_v12r5_case_balanced import (  # noqa: E402
    _production_source_closure,
    _production_source_closure_exact,
    _safe_environment_source_closure,
    tree_record,
)


class V12R5CaseBalancedFitError(RuntimeError):
    """Raised when an input, weight, fit, or persistence invariant drifts."""


RecoveryFitCorpus = v12r3_fit.RecoveryFitCorpus
FrozenNormalization = v11.FrozenNormalization
InMemoryFitResult = v11.InMemoryFitResult
DETERMINISTIC_TORCH_THREADS = v11.DETERMINISTIC_TORCH_THREADS

_P2_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
    "episode_ids",
    "raw_sample_weights",
    "normalized_sample_weights",
    "training_indices",
}
_NOMINAL_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
}

_ARTIFACT_RECORD_FIELDS = frozenset({"path", "sha256", "size_bytes"})
_TREE_RECORD_FIELDS = frozenset({"path", "tree_sha256", "file_count", "files"})
_FIT_RECEIPT_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "candidate_selection_rule",
        "candidate_id",
        "candidate_module",
        "summary",
        "gate",
        "corpus",
        "adaptation_report",
        "adaptation_history",
        "pipeline_claim",
        "worker_claim",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_FIT_SUMMARY_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "fit",
        "candidate_selection_rule",
        "initial_checkpoint_id",
        "continued_from_p2",
        "fit_counts",
        "sample_count",
        "episode_count",
        "reset_row_count",
        "corpus_components",
        "failed_plus_prefix_rows_loaded",
        "weighting",
        "case_mass",
        "weight_hashes",
        "normalized_total_sample_mass",
        "metrics",
        "p2_subset_metrics",
        "nominal_r4_pass_metrics",
        "nominal_r4_student_exposed_metrics",
        "per_case_metrics",
        "critical_window",
        "critical_window_metrics",
        "critical_window_p2_baseline_metrics",
        "critical_window_p2_baseline_recomputed",
        "critical_window_p2_module_tree",
        "worst_row",
        "adamw_epochs_run",
        "lbfgs_max_iter",
        "lbfgs_max_eval",
        "lbfgs_closure_calls",
        "deterministic_algorithms_enabled",
        "source_h0_byte_exact",
        "logstd_byte_exact",
        "critic_present",
        "disabled_clock_columns_bit_zero",
        "save_reload_exact",
        "hard_polish_used",
        "fallback_used",
        "sweep_used",
        "event_contract_id",
        "target_contract_id",
        "detector_or_fsm_modified",
        "candidate_module",
        "candidate_id",
        "corpus",
        "adaptation_report",
        "adaptation_history",
        "pipeline_claim",
        "worker_claim",
        "protocol_freeze",
        "execution_lock",
        "q2_paths_opened",
        "q3_paths_opened",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
)
_FIT_GATE_FIELDS = frozenset(
    {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "checks",
        "candidate_promoted",
        "next_stage",
    }
)
_METRIC_FIELDS = frozenset({"rmse", "max_abs_error", "reset_max_abs_error"})
_CRITICAL_METRIC_FIELDS = frozenset({"rmse", "max_abs_error"})
_WORST_ROW_FIELDS = frozenset(
    {
        "absolute_error",
        "action_dimension",
        "case_id",
        "step_index",
        "tranche_id",
    }
)


def _resolve(path: str | PurePath | Path) -> Path:
    candidate = Path(path)
    return (candidate if candidate.is_absolute() else REPO_ROOT / candidate).resolve()


def _json(path: str | PurePath | Path) -> Any:
    resolved = _resolve(path)
    try:
        return forensic.strict_json_load(resolved)
    except Exception as exc:
        raise V12R5CaseBalancedFitError(
            f"invalid strict JSON artifact: {resolved}"
        ) from exc


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    value = _json(path)
    if not isinstance(value, Mapping):
        raise V12R5CaseBalancedFitError(f"expected JSON object: {_resolve(path)}")
    return dict(value)


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)


def _records_exact(records: Mapping[str, Mapping[str, Any]]) -> bool:
    return all(_record(record["path"]) == dict(record) for record in records.values())


def _strict_equal(left: Any, right: Any) -> bool:
    """Compare JSON-like values without bool/int/float coercion."""

    if type(left) is not type(right):
        return False
    if isinstance(left, Mapping):
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, (list, tuple)):
        return len(left) == len(right) and all(
            _strict_equal(a, b) for a, b in zip(left, right, strict=True)
        )
    return bool(left == right)


def _sha256(value: Any) -> bool:
    return (
        type(value) is str
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_relative_path(value: Any) -> bool:
    if type(value) is not str or not value or "\\" in value:
        return False
    pure = PurePosixPath(value)
    return not pure.is_absolute() and ".." not in pure.parts and str(pure) == value


def _logical_path(path: str | PurePath | Path) -> str:
    return _resolve(path).relative_to(REPO_ROOT.resolve()).as_posix()


def _artifact_record_valid(value: Any, *, expected_path: str | PurePath | Path) -> bool:
    if not isinstance(value, Mapping) or set(value) != _ARTIFACT_RECORD_FIELDS:
        return False
    try:
        expected_logical_path = _logical_path(expected_path)
        observed = _record(expected_path)
    except Exception:
        return False
    return (
        type(value.get("path")) is str
        and value["path"] == expected_logical_path
        and _sha256(value.get("sha256"))
        and type(value.get("size_bytes")) is int
        and value["size_bytes"] > 0
        and _strict_equal(dict(value), observed)
    )


def _tree_record_valid(value: Any, *, expected_path: str | PurePath | Path) -> bool:
    if not isinstance(value, Mapping) or set(value) != _TREE_RECORD_FIELDS:
        return False
    files = value.get("files")
    count = value.get("file_count")
    if (
        type(value.get("path")) is not str
        or value["path"] != _logical_path(expected_path)
        or not _sha256(value.get("tree_sha256"))
        or type(count) is not int
        or count < 1
        or type(files) is not list
        or len(files) != count
    ):
        return False
    paths: list[str] = []
    for row in files:
        if (
            not isinstance(row, Mapping)
            or set(row) != _ARTIFACT_RECORD_FIELDS
            or not _canonical_relative_path(row.get("path"))
            or not _sha256(row.get("sha256"))
            or type(row.get("size_bytes")) is not int
            or row["size_bytes"] < 0
        ):
            return False
        paths.append(row["path"])
    return len(paths) == len(set(paths)) and paths == sorted(paths)


def _metric_record_valid(value: Any, *, fields: frozenset[str]) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == fields
        and all(
            type(value.get(name)) is float
            and math.isfinite(value[name])
            and value[name] >= 0.0
            for name in fields
        )
    )


def _fit_summary_payload_valid(
    summary: Any, *, module: Mapping[str, Any], expected_id: str
) -> bool:
    if not isinstance(summary, Mapping) or set(summary) != _FIT_SUMMARY_FIELDS:
        return False
    integer_expectations = {
        "schema_version": contract.SCHEMA_VERSION,
        "sample_count": contract.CORPUS_ROWS,
        "episode_count": contract.CORPUS_EPISODE_COUNT,
        "reset_row_count": contract.CORPUS_RESET_ROWS,
        "failed_plus_prefix_rows_loaded": 0,
        "adamw_epochs_run": 3000,
        "lbfgs_max_iter": 600,
        "lbfgs_max_eval": 1200,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    if not all(
        type(summary.get(name)) is int and summary[name] == expected
        for name, expected in integer_expectations.items()
    ):
        return False
    boolean_expectations = {
        "continued_from_p2": False,
        "critical_window_p2_baseline_recomputed": True,
        "deterministic_algorithms_enabled": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": True,
        "critic_present": False,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "hard_polish_used": False,
        "fallback_used": False,
        "sweep_used": False,
        "detector_or_fsm_modified": False,
    }
    if not all(
        summary.get(name) is expected for name, expected in boolean_expectations.items()
    ):
        return False
    if (
        summary.get("status") != contract.FIT_COMPLETE_STATUS
        or summary.get("protocol_id") != contract.PROTOCOL_ID
        or summary.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or summary.get("initial_checkpoint_id") != contract.SOURCE_H0_ID
        or summary.get("event_contract_id") != contract.EVENT_CONTRACT_ID
        or summary.get("target_contract_id") != contract.TARGET_CONTRACT_ID
        or summary.get("candidate_id") != expected_id
        or not _strict_equal(summary.get("fit"), contract.FIT)
        or not _strict_equal(
            summary.get("fit_counts"), contract.expected_corpus_counts()
        )
        or not _strict_equal(
            summary.get("corpus_components"),
            ["p2_corpus", "r4_nominal_pass_labels_only"],
        )
        or not _strict_equal(summary.get("weighting"), contract.WEIGHTING)
        or not _strict_equal(summary.get("critical_window"), contract.CRITICAL_WINDOW)
        or not _strict_equal(
            summary.get("critical_window_p2_module_tree"), contract.P2_MODULE_TREE
        )
        or not _strict_equal(summary.get("q2_paths_opened"), [])
        or not _strict_equal(summary.get("q3_paths_opened"), [])
        or type(summary.get("lbfgs_closure_calls")) is not int
        or summary["lbfgs_closure_calls"] < 1
        or type(summary.get("normalized_total_sample_mass")) is not float
        or not math.isclose(
            summary["normalized_total_sample_mass"],
            contract.NORMALIZED_TOTAL_MASS,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    ):
        return False
    metrics = (
        "metrics",
        "p2_subset_metrics",
        "nominal_r4_pass_metrics",
        "nominal_r4_student_exposed_metrics",
    )
    if not all(
        _metric_record_valid(summary.get(name), fields=_METRIC_FIELDS)
        for name in metrics
    ):
        return False
    per_case = summary.get("per_case_metrics")
    case_mass = summary.get("case_mass")
    if (
        not isinstance(per_case, Mapping)
        or set(per_case) != set(contract.CASE_IDS)
        or not all(
            _metric_record_valid(per_case.get(case_id), fields=_METRIC_FIELDS)
            for case_id in contract.CASE_IDS
        )
        or not isinstance(case_mass, Mapping)
        or set(case_mass) != set(contract.CASE_IDS)
        or not all(
            type(case_mass.get(case_id)) is float
            and math.isclose(
                case_mass[case_id],
                contract.CASE_TARGET_MASS,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            for case_id in contract.CASE_IDS
        )
    ):
        return False
    weight_hashes = summary.get("weight_hashes")
    expected_weight_hashes = {
        "p2_max_abs_error_sha256": contract.WEIGHTING[
            "expected_p2_max_abs_error_sha256"
        ],
        "hardness_sha256": contract.WEIGHTING["expected_hardness_sha256"],
        "source_risk_sha256": contract.WEIGHTING["expected_source_risk_sha256"],
        "normalized_weights_sha256": contract.WEIGHTING[
            "expected_normalized_weights_sha256"
        ],
    }
    if not _strict_equal(weight_hashes, expected_weight_hashes):
        return False
    if not all(
        _metric_record_valid(summary.get(name), fields=_CRITICAL_METRIC_FIELDS)
        for name in (
            "critical_window_metrics",
            "critical_window_p2_baseline_metrics",
        )
    ):
        return False
    worst = summary.get("worst_row")
    if (
        not isinstance(worst, Mapping)
        or set(worst) != _WORST_ROW_FIELDS
        or type(worst.get("absolute_error")) is not float
        or not math.isfinite(worst["absolute_error"])
        or worst["absolute_error"] < 0.0
        or type(worst.get("action_dimension")) is not int
        or worst["action_dimension"] not in range(contract.EXPECTED_ACTION_DIM)
        or type(worst.get("step_index")) is not int
        or worst["step_index"] < 0
        or type(worst.get("case_id")) is not str
        or worst["case_id"] not in contract.CASE_IDS
        or type(worst.get("tranche_id")) is not str
        or not worst["tranche_id"]
    ):
        return False
    if not _tree_record_valid(
        module, expected_path=contract.CANDIDATE_MODULE_PATH
    ) or not _strict_equal(summary.get("candidate_module"), module):
        return False
    expected_records = {
        "corpus": contract.CORPUS_PATH,
        "adaptation_report": _resolve(contract.FIT_ROOT) / "adaptation_report.json",
        "adaptation_history": _resolve(contract.FIT_ROOT) / "adaptation_history.json",
        "pipeline_claim": contract.PIPELINE_CLAIM_PATH,
        "worker_claim": contract.WORKER_CLAIMS_ROOT
        / "fit_case_balanced_candidate.json",
        "protocol_freeze": contract.PROTOCOL_FREEZE_PATH,
        "execution_lock": contract.EXECUTION_LOCK_PATH,
    }
    return all(
        _artifact_record_valid(summary.get(name), expected_path=path)
        for name, path in expected_records.items()
    )


def _fit_gate_payload_valid(gate: Any, *, summary: Mapping[str, Any]) -> bool:
    if not isinstance(gate, Mapping) or set(gate) != _FIT_GATE_FIELDS:
        return False
    expected = contract.fit_gate(summary)
    checks = gate.get("checks")
    expected_checks = expected.get("checks")
    return (
        _strict_equal(dict(gate), expected)
        and type(gate.get("schema_version")) is int
        and gate["schema_version"] == contract.SCHEMA_VERSION
        and gate.get("passed") is True
        and gate.get("candidate_promoted") is False
        and isinstance(checks, Mapping)
        and isinstance(expected_checks, Mapping)
        and set(checks) == set(expected_checks)
        and all(type(value) is bool and value for value in checks.values())
    )


def _fit_receipt_payload_valid(
    receipt: Any,
    *,
    summary: Mapping[str, Any],
    gate: Mapping[str, Any],
    module: Mapping[str, Any],
    expected_id: str,
) -> bool:
    if not isinstance(receipt, Mapping) or set(receipt) != _FIT_RECEIPT_FIELDS:
        return False
    if (
        type(receipt.get("schema_version")) is not int
        or receipt["schema_version"] != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FIT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or receipt.get("candidate_id") != expected_id
        or not _strict_equal(receipt.get("candidate_module"), module)
        or type(receipt.get("actor_updates")) is not int
        or receipt["actor_updates"] != 1
        or type(receipt.get("critic_updates")) is not int
        or receipt["critic_updates"] != 0
        or type(receipt.get("ppo_updates")) is not int
        or receipt["ppo_updates"] != 0
    ):
        return False
    expected_records = {
        "summary": _resolve(contract.FIT_ROOT) / "summary.json",
        "gate": _resolve(contract.FIT_ROOT) / "gate.json",
        "corpus": contract.CORPUS_PATH,
        "adaptation_report": _resolve(contract.FIT_ROOT) / "adaptation_report.json",
        "adaptation_history": _resolve(contract.FIT_ROOT) / "adaptation_history.json",
        "pipeline_claim": contract.PIPELINE_CLAIM_PATH,
        "worker_claim": contract.WORKER_CLAIMS_ROOT
        / "fit_case_balanced_candidate.json",
    }
    if not all(
        _artifact_record_valid(receipt.get(name), expected_path=path)
        for name, path in expected_records.items()
    ):
        return False
    return (
        all(
            _strict_equal(receipt.get(name), summary.get(name))
            for name in (
                "corpus",
                "adaptation_report",
                "adaptation_history",
                "pipeline_claim",
                "worker_claim",
            )
        )
        and _strict_equal(gate, contract.fit_gate(summary))
        and _strict_equal(
            receipt.get("gate"), _record(_resolve(contract.FIT_ROOT) / "gate.json")
        )
    )


def array_sha256(value: Any) -> str:
    return v10s_fit.array_sha256(value)


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _read_npz(path: str | PurePath | Path, keys: set[str]) -> dict[str, np.ndarray]:
    resolved = _resolve(path)
    try:
        with np.load(resolved, allow_pickle=False) as archive:
            if set(archive.files) != keys:
                raise V12R5CaseBalancedFitError(
                    f"NPZ keys drifted for {resolved}: {sorted(archive.files)}"
                )
            return {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R5CaseBalancedFitError:
        raise
    except Exception as exc:
        raise V12R5CaseBalancedFitError(f"cannot read exact NPZ: {resolved}") from exc


def _qualification_unopened() -> bool:
    paths = (*contract.Q2_UNOPENED_PATHS.values(), *contract.Q3_UNOPENED_PATHS.values())
    return all(not os.path.lexists(_resolve(path)) for path in paths)


def _attest_locked_inputs() -> dict[str, Any]:
    production_source_closure = _production_source_closure()
    if (
        not _records_exact(contract.R4_TERMINAL_ARTIFACTS)
        or not _records_exact(contract.R4_NOMINAL_REUSABLE_ARTIFACTS)
        or not _records_exact(contract.R4_PLUS_FAILURE_EVIDENCE)
        or not _records_exact(contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS)
        or not _records_exact(contract.FROZEN_EXTERNAL_RUNTIME_SOURCES)
        or _record(contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"])
        != contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
        or len(_safe_environment_source_closure()) != 29
        or len(production_source_closure) != contract.EXPECTED_PRODUCTION_SOURCE_COUNT
        or not _production_source_closure_exact(production_source_closure)
        or _record(contract.P2_CORPUS_ARTIFACT["path"]) != contract.P2_CORPUS_ARTIFACT
        or _record(contract.P2_ADAPTATION_REPORT_ARTIFACT["path"])
        != contract.P2_ADAPTATION_REPORT_ARTIFACT
        or _record(contract.P2_ADAPTATION_HISTORY_ARTIFACT["path"])
        != contract.P2_ADAPTATION_HISTORY_ARTIFACT
        or tree_record(contract.P2_MODULE_TREE["path"]) != contract.P2_MODULE_TREE
    ):
        raise V12R5CaseBalancedFitError("locked predecessor artifact drifted")
    r4 = _mapping(contract.R4_TERMINAL_ARTIFACTS["terminal_ledger"]["path"])
    nominal_receipt = _mapping(
        contract.R4_NOMINAL_REUSABLE_ARTIFACTS["receipt"]["path"]
    )
    nominal_gate = _mapping(contract.R4_NOMINAL_REUSABLE_ARTIFACTS["gate"]["path"])
    nominal_summary = _mapping(
        contract.R4_NOMINAL_REUSABLE_ARTIFACTS["summary"]["path"]
    )
    plus_failure = _mapping(contract.R4_PLUS_FAILURE_EVIDENCE["failure"]["path"])
    plus_gate = _mapping(contract.R4_PLUS_FAILURE_EVIDENCE["gate"]["path"])
    plus_summary = _mapping(contract.R4_PLUS_FAILURE_EVIDENCE["summary"]["path"])
    safe_gate = _mapping(contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS["gate"]["path"])
    safe_summary = _mapping(
        contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS["summary"]["path"]
    )
    semantics = {
        "r4_terminal": r4.get("status") == "FAIL_H0_V12R4_P3_COVERAGE_TERMINAL"
        and r4.get("passed") is False
        and r4.get("attempted_stage") == "collect_cov__deterministic_offset_plus_0p20"
        and r4.get("retry_authorized") is False
        and r4.get("resume_authorized") is False
        and r4.get("collection_count") == 1,
        "nominal_reusable": nominal_receipt.get("passed") is True
        and nominal_receipt.get("sample_count") == contract.NOMINAL_PASS_ROWS
        and nominal_receipt.get("label_corpus")
        == contract.R4_NOMINAL_REUSABLE_ARTIFACTS["labels"]
        and nominal_gate.get("passed") is True
        and nominal_gate.get("collection_data_reusable") is True
        and nominal_summary.get("steps") == contract.NOMINAL_PASS_ROWS
        and nominal_summary.get("phase_valid_cycle_count", -1)
        >= contract.MINIMUM_VALID_CYCLES
        and float(nominal_summary.get("grf_penetration_max_m", math.inf))
        < contract.PENETRATION_LIMIT_M,
        "failed_plus_excluded": plus_failure.get("passed") is False
        and plus_failure.get("last_completed_step") == 212
        and plus_gate.get("passed") is False
        and plus_gate.get("collection_data_reusable") is False
        and plus_summary.get("steps") == 212
        and float(plus_summary.get("grf_penetration_max_m", 0.0))
        >= contract.PENETRATION_LIMIT_M,
        "safe_plus_reference": safe_gate.get("passed") is True
        and safe_summary.get("steps") == contract.EXPECTED_STEPS
        and safe_summary.get("phase_valid_cycle_count", -1)
        >= contract.MINIMUM_VALID_CYCLES
        and math.isclose(
            float(safe_summary.get("grf_penetration_max_m", math.nan)),
            0.024323924384327976,
            rel_tol=0.0,
            abs_tol=1e-15,
        ),
        "qualification_unopened": _qualification_unopened(),
    }
    if not all(semantics.values()):
        raise V12R5CaseBalancedFitError(
            f"locked predecessor semantics drifted: {semantics}"
        )
    return {
        "semantics": semantics,
        "r4_terminal": copy.deepcopy(contract.R4_TERMINAL_ARTIFACTS),
        "r4_nominal_pass": copy.deepcopy(contract.R4_NOMINAL_REUSABLE_ARTIFACTS),
        "r4_plus_failure_forensic_only": copy.deepcopy(
            contract.R4_PLUS_FAILURE_EVIDENCE
        ),
        "safe_v8r1p1_plus_reference": copy.deepcopy(
            contract.SAFE_V8R1P1_PLUS_REPLAY_ARTIFACTS
        ),
        "external_runtime_sources": copy.deepcopy(
            contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
        ),
        "production_source_closure": production_source_closure,
        "safe_environment_execution_lock": copy.deepcopy(
            contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
        ),
        "safe_environment_source_closure": _safe_environment_source_closure(),
    }


def _load_p2_piece() -> dict[str, np.ndarray]:
    arrays = _read_npz(contract.P2_CORPUS_ARTIFACT["path"], _P2_KEYS)
    rows = contract.P2_CORPUS_ROWS
    if (
        arrays["observations"].shape != (rows, contract.EXPECTED_ACTOR_FEATURES)
        or arrays["observations"].dtype != np.dtype(np.float32)
        or arrays["actions"].shape != (rows, contract.EXPECTED_ACTION_DIM)
        or arrays["actions"].dtype != np.dtype(np.float32)
        or arrays["reset_mask"].shape != (rows,)
        or arrays["reset_mask"].dtype != np.dtype(np.bool_)
        or int(np.count_nonzero(arrays["reset_mask"])) != contract.P2_RESET_ROWS
        or len(set(arrays["episode_ids"].astype(str).tolist()))
        != contract.P2_EPISODE_COUNT
        or not np.array_equal(arrays["training_indices"], np.arange(rows))
        or not np.all(np.isfinite(arrays["observations"]))
        or not np.all(np.isfinite(arrays["actions"]))
        or not np.all(np.isfinite(arrays["raw_sample_weights"]))
        or np.any(arrays["raw_sample_weights"] <= 0.0)
        or set(arrays["case_ids"].astype(str).tolist()) != set(contract.CASE_IDS)
    ):
        raise V12R5CaseBalancedFitError("P2 corpus semantic contract drifted")
    return arrays


def _load_nominal_piece() -> tuple[dict[str, np.ndarray], np.ndarray]:
    arrays = _read_npz(
        contract.R4_NOMINAL_REUSABLE_ARTIFACTS["labels"]["path"], _NOMINAL_KEYS
    )
    rows = contract.NOMINAL_PASS_ROWS
    trace = _json(contract.R4_NOMINAL_REUSABLE_ARTIFACTS["trace"]["path"])
    if not isinstance(trace, Sequence) or isinstance(trace, (str, bytes)):
        raise V12R5CaseBalancedFitError("R4 nominal trace is not a sequence")
    if (
        len(trace) != rows
        or arrays["observations"].shape != (rows, contract.EXPECTED_ACTOR_FEATURES)
        or arrays["observations"].dtype != np.dtype(np.float32)
        or arrays["actions"].shape != (rows, contract.EXPECTED_ACTION_DIM)
        or arrays["actions"].dtype != np.dtype(np.float32)
        or arrays["reset_mask"].shape != (rows,)
        or arrays["reset_mask"].dtype != np.dtype(np.bool_)
        or int(np.count_nonzero(arrays["reset_mask"])) != 1
        or not bool(arrays["reset_mask"][0])
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or set(arrays["case_ids"].astype(str).tolist())
        != {"deterministic_offset_nominal"}
        or set(arrays["tranche_ids"].astype(str).tolist()) != {"v12r4_p3_coverage"}
        or set(arrays["origins"].astype(str).tolist())
        != {"V12R4_SHIELDED_SAME_STATE_TEACHER_LABEL"}
        or not np.all(np.isfinite(arrays["observations"]))
        or not np.all(np.isfinite(arrays["actions"]))
    ):
        raise V12R5CaseBalancedFitError("R4 nominal labels drifted")
    exposed = np.zeros(rows, dtype=np.bool_)
    for index, item in enumerate(trace):
        if not isinstance(item, Mapping) or item.get("step") != index + 1:
            raise V12R5CaseBalancedFitError("R4 nominal trace/label alignment drifted")
        alpha = item.get("effective_alpha")
        latch = item.get("safety_latch_active")
        if alpha not in (0.0, 0.5) or type(latch) is not bool:
            raise V12R5CaseBalancedFitError("R4 nominal exposure fields drifted")
        exposed[index] = alpha == 0.5 and latch is False
    if int(np.count_nonzero(exposed)) != contract.NOMINAL_STUDENT_EXPOSED_ROWS:
        raise V12R5CaseBalancedFitError("R4 nominal exposed row count drifted")
    arrays["episode_ids"] = np.repeat(
        np.asarray(["v12r5_source:r4_nominal_pass"], dtype="U192"), rows
    )
    arrays["raw_sample_weights"] = np.where(
        arrays["reset_mask"], np.float64(100.0), np.float64(1.0)
    )
    return arrays, exposed


def _load_bound_p2_state() -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(_resolve(contract.P2_MODULE_TREE["path"]))
    module.eval()
    return v11._clone_state(module.get_state())


def compute_case_balanced_weights(
    *,
    source_risk: Any,
    p2_predictions: Any,
    targets: Any,
    case_ids: Any,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply the sole frozen P2-hardness and six-case balancing formula."""

    risk = np.ascontiguousarray(source_risk, dtype=np.float64)
    prediction = np.ascontiguousarray(p2_predictions, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    cases = np.asarray(case_ids).astype(str)
    rows = len(cases)
    if (
        risk.shape != (rows,)
        or prediction.shape != (rows, contract.EXPECTED_ACTION_DIM)
        or labels.shape != (rows, contract.EXPECTED_ACTION_DIM)
        or not np.all(np.isfinite(risk))
        or not np.all(np.isfinite(prediction))
        or not np.all(np.isfinite(labels))
        or np.any(risk <= 0.0)
        or set(cases.tolist()) != set(contract.CASE_IDS)
    ):
        raise V12R5CaseBalancedFitError("weight inputs are malformed")
    errors = np.max(
        np.abs(prediction.astype(np.float64) - labels.astype(np.float64)), axis=1
    )
    hardness = 1.0 + 99.0 * np.square(
        np.minimum(1.0, errors / contract.HARDNESS_GATE_SCALE)
    )
    raw = np.maximum(risk, hardness)
    normalized = np.empty(rows, dtype=np.float64)
    case_mass: dict[str, float] = {}
    for case_id in contract.CASE_IDS:
        selected = np.flatnonzero(cases == case_id)
        if len(selected) == 0:
            raise V12R5CaseBalancedFitError(f"empty case during balancing: {case_id}")
        denominator = math.fsum(float(raw[index]) for index in selected)
        if not math.isfinite(denominator) or denominator <= 0.0:
            raise V12R5CaseBalancedFitError(f"invalid raw case mass: {case_id}")
        normalized[selected] = raw[selected] * (contract.CASE_TARGET_MASS / denominator)
        observed = math.fsum(float(normalized[index]) for index in selected)
        correction = contract.CASE_TARGET_MASS - observed
        normalized[selected[-1]] += correction
        case_mass[case_id] = math.fsum(float(normalized[index]) for index in selected)
    total = math.fsum(float(value) for value in normalized)
    if (
        not np.all(np.isfinite(normalized))
        or np.any(normalized <= 0.0)
        or not math.isclose(
            total, contract.NORMALIZED_TOTAL_MASS, rel_tol=0.0, abs_tol=1e-9
        )
        or not all(
            math.isclose(value, contract.CASE_TARGET_MASS, rel_tol=0.0, abs_tol=1e-9)
            for value in case_mass.values()
        )
    ):
        raise V12R5CaseBalancedFitError("case-balanced weights failed closure")
    return np.ascontiguousarray(normalized), {
        "weighting": copy.deepcopy(contract.WEIGHTING),
        "p2_max_abs_error_sha256": array_sha256(errors),
        "hardness_sha256": array_sha256(hardness),
        "source_risk_sha256": array_sha256(risk),
        "normalized_weights_sha256": array_sha256(normalized),
        "case_mass": case_mass,
        "normalized_total_sample_mass": total,
    }


def _error_metrics(predictions: np.ndarray, targets: np.ndarray) -> dict[str, float]:
    error = predictions.astype(np.float64) - targets.astype(np.float64)
    if (
        error.ndim != 2
        or error.shape[1] != contract.EXPECTED_ACTION_DIM
        or len(error) == 0
    ):
        raise V12R5CaseBalancedFitError("error metric slice is malformed")
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def load_case_balanced_corpus() -> RecoveryFitCorpus:
    """Load exactly P2 + R4 nominal PASS and compute fixed weights."""

    source_attestation = _attest_locked_inputs()
    p2 = _load_p2_piece()
    nominal, exposed_nominal = _load_nominal_piece()
    row_names = (
        "observations",
        "actions",
        "reset_mask",
        "case_ids",
        "step_indices",
        "tranche_ids",
        "origins",
        "episode_ids",
        "raw_sample_weights",
    )
    combined = {
        name: np.ascontiguousarray(np.concatenate((p2[name], nominal[name])))
        for name in row_names
    }
    features = np.ascontiguousarray(p2["actor_feature_names"], dtype="U64")
    if not _bytes_equal(features, nominal["actor_feature_names"]):
        raise V12R5CaseBalancedFitError("actor feature names drifted")
    if len(combined["observations"]) != contract.CORPUS_ROWS:
        raise V12R5CaseBalancedFitError("combined corpus row count drifted")
    p2_state = _load_bound_p2_state()
    p2_predictions = np.ascontiguousarray(
        v11._state_logits(p2_state, combined["observations"])[:, :2],
        dtype=np.float32,
    )
    normalized, weight_audit = compute_case_balanced_weights(
        source_risk=combined["raw_sample_weights"],
        p2_predictions=p2_predictions,
        targets=combined["actions"],
        case_ids=combined["case_ids"],
    )
    expected_hashes = {
        "p2_max_abs_error_sha256": contract.WEIGHTING[
            "expected_p2_max_abs_error_sha256"
        ],
        "hardness_sha256": contract.WEIGHTING["expected_hardness_sha256"],
        "source_risk_sha256": contract.WEIGHTING["expected_source_risk_sha256"],
        "normalized_weights_sha256": contract.WEIGHTING[
            "expected_normalized_weights_sha256"
        ],
    }
    if any(weight_audit[name] != value for name, value in expected_hashes.items()):
        raise V12R5CaseBalancedFitError("canonical R5 weight hash drifted")
    combined["normalized_sample_weights"] = normalized
    identities = list(
        zip(
            combined["tranche_ids"].astype(str).tolist(),
            combined["case_ids"].astype(str).tolist(),
            combined["step_indices"].tolist(),
        )
    )
    critical = contract.CRITICAL_WINDOW
    p2_indices = np.arange(contract.P2_CORPUS_ROWS, dtype=np.int64)
    critical_mask = (
        (combined["tranche_ids"].astype(str) == critical["tranche_id"])
        & (combined["case_ids"].astype(str) == critical["case_id"])
        & (combined["step_indices"] >= critical["step_start_inclusive"])
        & (combined["step_indices"] <= critical["step_end_inclusive"])
        & (np.arange(contract.CORPUS_ROWS) < contract.P2_CORPUS_ROWS)
    )
    critical_indices = np.flatnonzero(critical_mask)
    nominal_indices = np.arange(
        contract.P2_CORPUS_ROWS, contract.CORPUS_ROWS, dtype=np.int64
    )
    exposed_indices = nominal_indices[np.flatnonzero(exposed_nominal)]
    episode_count = len(set(combined["episode_ids"].astype(str).tolist()))
    if (
        int(np.count_nonzero(combined["reset_mask"])) != contract.CORPUS_RESET_ROWS
        or episode_count != contract.CORPUS_EPISODE_COUNT
        or len(identities) != len(set(identities))
        or len(critical_indices) != critical["expected_rows"]
        or len(exposed_indices) != contract.NOMINAL_STUDENT_EXPOSED_ROWS
        or not np.array_equal(p2_indices, np.arange(contract.P2_CORPUS_ROWS))
        or not _qualification_unopened()
    ):
        raise V12R5CaseBalancedFitError("assembled corpus provenance drifted")
    critical_baseline = _error_metrics(
        p2_predictions[critical_indices], combined["actions"][critical_indices]
    )
    audit = {
        **contract.expected_corpus_counts(),
        "all_finite": True,
        "duplicate_sample_count": 0,
        "component_order_exact": True,
        "failed_plus_prefix_rows_loaded": 0,
        "nominal_pass_indices": nominal_indices.tolist(),
        "nominal_student_exposed_indices": exposed_indices.tolist(),
        "critical_window_indices": critical_indices.tolist(),
        "critical_window_p2_baseline_metrics": critical_baseline,
        "critical_window_p2_baseline_recomputed": True,
        "critical_window_p2_module_tree": copy.deepcopy(contract.P2_MODULE_TREE),
        "weight_audit": weight_audit,
        "qualification_unopened": True,
    }
    return RecoveryFitCorpus(
        observations=combined["observations"],
        actions=combined["actions"],
        reset_mask=combined["reset_mask"],
        actor_feature_names=features,
        case_ids=combined["case_ids"],
        step_indices=combined["step_indices"],
        tranche_ids=combined["tranche_ids"],
        origins=combined["origins"],
        episode_ids=combined["episode_ids"],
        raw_sample_weights=combined["raw_sample_weights"],
        normalized_sample_weights=normalized,
        source_records={
            "p2": {
                "corpus": copy.deepcopy(contract.P2_CORPUS_ARTIFACT),
                "module": copy.deepcopy(contract.P2_MODULE_TREE),
            },
            "locked_input_attestation": source_attestation,
        },
        probe_label_bindings=(),
        collection_bindings=(),
        audit=audit,
    )


def adamw_learning_rate(epoch: int) -> float:
    if not isinstance(epoch, int) or isinstance(epoch, bool) or not 1 <= epoch <= 3000:
        raise V12R5CaseBalancedFitError(
            f"AdamW epoch outside frozen schedule: {epoch!r}"
        )
    if epoch <= 1500:
        return 3.0e-4
    if epoch <= 2500:
        return 1.0e-4
    return 3.0e-5


def fit_case_balanced_full_mean_in_memory(
    *,
    source_state: Mapping[str, Any],
    observations: Any,
    targets: Any,
    reset_mask: Any,
    sample_weights: Any,
    normalization: FrozenNormalization,
    activity_callback: Callable[[str, int], None] | None = None,
) -> InMemoryFitResult:
    """Run the sole AdamW(3000)+LBFGS(600/1200) numerical design."""

    import torch

    try:
        v11.validate_source_h0_state(source_state)
    except Exception as exc:
        raise V12R5CaseBalancedFitError("source H0 validation failed") from exc
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    weights_np = np.ascontiguousarray(sample_weights, dtype=np.float64)
    if (
        raw.shape != (contract.CORPUS_ROWS, contract.EXPECTED_ACTOR_FEATURES)
        or labels.shape != (contract.CORPUS_ROWS, contract.EXPECTED_ACTION_DIM)
        or reset.shape != (contract.CORPUS_ROWS,)
        or weights_np.shape != (contract.CORPUS_ROWS,)
        or not np.all(np.isfinite(raw))
        or not np.all(np.isfinite(labels))
        or not np.all(np.isfinite(weights_np))
        or np.any(weights_np <= 0.0)
    ):
        raise V12R5CaseBalancedFitError("weighted R5 arrays are malformed")
    normalized = v11.normalized_observations(raw, normalization)
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(DETERMINISTIC_TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(20260807)
        model = v11._new_normalized_model(source_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        weights = torch.as_tensor(weights_np, dtype=torch.float64)
        weight_sum = torch.sum(weights)
        history: list[dict[str, Any]] = []
        optimizer = torch.optim.AdamW(
            model.parameters(), lr=3.0e-4, weight_decay=1.0e-7
        )
        for epoch in range(1, 3001):
            rate = adamw_learning_rate(epoch)
            for group in optimizer.param_groups:
                group["lr"] = rate
            optimizer.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            loss = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(loss):
                raise V12R5CaseBalancedFitError(
                    f"non-finite AdamW loss at epoch {epoch}"
                )
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            optimizer.step()
            if activity_callback is not None:
                activity_callback("adamw_epochs_completed", 1)
            if epoch in {1, 250, 500, 1000, 1500, 2000, 2500, 3000}:
                v11._milestone(history, stage="adamw", index=epoch, loss=loss, lr=rate)
        lbfgs = torch.optim.LBFGS(
            model.parameters(),
            lr=0.7,
            max_iter=600,
            max_eval=1200,
            tolerance_grad=1.0e-10,
            tolerance_change=1.0e-12,
            history_size=50,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        last_lbfgs_loss: Any = None

        def closure() -> Any:
            nonlocal closure_calls, last_lbfgs_loss
            lbfgs.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            value = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(value):
                raise V12R5CaseBalancedFitError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            if activity_callback is not None:
                activity_callback("lbfgs_closure_calls", 1)
            last_lbfgs_loss = value
            if closure_calls in {1, 50, 100, 200, 300, 400, 600, 800, 1200}:
                v11._milestone(
                    history,
                    stage="lbfgs_closure",
                    index=closure_calls,
                    loss=value,
                    lr=0.7,
                )
            return value

        lbfgs.step(closure)
        if last_lbfgs_loss is None:
            raise V12R5CaseBalancedFitError("LBFGS never evaluated objective")
        v11._milestone(
            history,
            stage="lbfgs_final",
            index=closure_calls,
            loss=last_lbfgs_loss,
            lr=0.7,
        )
        candidate_state, fold_audit = v11._fold_normalization_into_state(
            model, source_state, normalization
        )
        with torch.no_grad():
            normalized_prediction = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        runtime_logits = v11._state_logits(candidate_state, raw)
        runtime_prediction = np.ascontiguousarray(
            runtime_logits[:, :2], dtype=np.float32
        )
        normalization_audit = {
            **fold_audit,
            **v11.fold_equivalence_audit(normalized_prediction, runtime_prediction),
            "normalization": normalization.record(),
        }
        preservation = v11.full_mean_update_audit(source_state, candidate_state)
        source_logits = v11._state_logits(source_state, raw)
        preservation = {
            **preservation,
            "logstd_outputs_bit_exact": source_logits[:, 2:].tobytes()
            == runtime_logits[:, 2:].tobytes(),
        }
        preservation["passed"] = bool(
            preservation["passed"] and preservation["logstd_outputs_bit_exact"]
        )
        if not preservation["passed"]:
            raise V12R5CaseBalancedFitError("mean-only preservation audit failed")
        metrics = v11.prediction_metrics(runtime_prediction, labels, reset)
        optimizer_audit = {
            "fit_contract_id": contract.FIT_CONTRACT_ID,
            "seed": 20260807,
            "full_batch": True,
            "sample_count": len(raw),
            "explicit_sample_weights": True,
            "sample_weight_dtype": "float64",
            "sample_weights_sha256": array_sha256(weights_np),
            "normalized_total_sample_mass": float(
                math.fsum(float(value) for value in weights_np)
            ),
            "adamw_epochs": 3000,
            "adamw_schedule": copy.deepcopy(
                contract.FIT["adamw"]["learning_rate_schedule"]
            ),
            "adamw_weight_decay": 1.0e-7,
            "gradient_clip_norm": 10.0,
            "lbfgs_lr": 0.7,
            "lbfgs_max_iter": 600,
            "lbfgs_max_eval": 1200,
            "lbfgs_tolerance_grad": 1.0e-10,
            "lbfgs_tolerance_change": 1.0e-12,
            "lbfgs_history_size": 50,
            "lbfgs_line_search": "strong_wolfe",
            "lbfgs_closure_calls": closure_calls,
            "hard_polish": False,
            "fallback": False,
            "sweep": False,
            "torch_threads": DETERMINISTIC_TORCH_THREADS,
            "deterministic_algorithms_enabled": True,
        }
        return InMemoryFitResult(
            candidate_state=candidate_state,
            predictions=runtime_prediction,
            metrics=metrics,
            normalization=normalization,
            normalization_audit=normalization_audit,
            preservation_audit=preservation,
            history=tuple(history),
            optimizer_audit=optimizer_audit,
        )
    except V12R5CaseBalancedFitError:
        raise
    except Exception as exc:
        raise V12R5CaseBalancedFitError("fixed R5 weighted fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _load_source_module_and_state() -> tuple[Any, dict[str, Any]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    source = _resolve(contract.SOURCE_H0_MODULE_PATH)
    module = RLModule.from_checkpoint(source)
    state = v11._clone_state(module.get_state())
    v11.validate_source_h0_state(state)
    return module, state


def _metric_slice(
    predictions: np.ndarray, corpus: RecoveryFitCorpus, selection: np.ndarray
) -> dict[str, float]:
    if len(selection) == 0:
        raise V12R5CaseBalancedFitError("empty metric slice")
    return v11.prediction_metrics(
        predictions[selection],
        corpus.actions[selection],
        corpus.reset_mask[selection],
    )


def run_fit_stage(
    *,
    pipeline_claim_path: str | PurePath | Path,
    worker_claim_path: str | PurePath | Path,
    protocol_freeze_path: str | PurePath | Path,
    execution_lock_path: str | PurePath | Path,
    activity_callback: Callable[[str, int], None] | None = None,
) -> dict[str, Any]:
    """Execute and exclusively persist the sole R5 fit."""

    destination = _resolve(contract.FIT_ROOT)
    if os.path.lexists(destination):
        raise V12R5CaseBalancedFitError("R5 fit destination exists/no-clobber")
    if not _qualification_unopened():
        raise V12R5CaseBalancedFitError("Q2/Q3 execution output already opened")
    pipeline_claim = _record(pipeline_claim_path)
    worker_claim = _record(worker_claim_path)
    protocol_payload = _mapping(protocol_freeze_path)
    execution_lock_payload = _mapping(execution_lock_path)
    for payload in (protocol_payload, execution_lock_payload):
        locked_inputs = payload.get("locked_inputs")
        if not isinstance(
            locked_inputs, Mapping
        ) or not _production_source_closure_exact(
            locked_inputs.get("production_source_closure")
        ):
            raise V12R5CaseBalancedFitError(
                "protocol/lock production source closure drifted"
            )
    protocol_freeze = _record(protocol_freeze_path)
    execution_lock = _record(execution_lock_path)
    corpus = load_case_balanced_corpus()
    source_before = tree_record(contract.SOURCE_H0_MODULE_PATH)
    if source_before.get("tree_sha256") != contract.SOURCE_H0_TREE_SHA256:
        raise V12R5CaseBalancedFitError("source H0 tree drifted")
    source_module, source_state = _load_source_module_and_state()
    normalization = v12r3_fit.frozen_base_normalization(corpus.observations[:3000])
    result = fit_case_balanced_full_mean_in_memory(
        source_state=source_state,
        observations=corpus.observations,
        targets=corpus.actions,
        reset_mask=corpus.reset_mask,
        sample_weights=corpus.normalized_sample_weights,
        normalization=normalization,
        activity_callback=activity_callback,
    )
    if source_before != tree_record(contract.SOURCE_H0_MODULE_PATH):
        raise V12R5CaseBalancedFitError("source H0 changed during R5 fit")
    destination.mkdir(parents=True, exist_ok=False)
    v10s_fit._write_npz_exclusive(destination / "corpus.npz", corpus.arrays())
    save_reload = v11._save_candidate_exact(
        source_module=source_module,
        candidate_state=result.candidate_state,
        destination=_resolve(contract.CANDIDATE_MODULE_PATH),
    )
    module_record = tree_record(contract.CANDIDATE_MODULE_PATH)
    p2_selection = np.arange(contract.P2_CORPUS_ROWS, dtype=np.int64)
    nominal_selection = np.asarray(corpus.audit["nominal_pass_indices"], dtype=np.int64)
    exposed_selection = np.asarray(
        corpus.audit["nominal_student_exposed_indices"], dtype=np.int64
    )
    critical_selection = np.asarray(
        corpus.audit["critical_window_indices"], dtype=np.int64
    )
    per_case_metrics = {
        case_id: _metric_slice(
            result.predictions,
            corpus,
            np.flatnonzero(corpus.case_ids.astype(str) == case_id),
        )
        for case_id in contract.CASE_IDS
    }
    p2_metrics = _metric_slice(result.predictions, corpus, p2_selection)
    nominal_metrics = _metric_slice(result.predictions, corpus, nominal_selection)
    exposed_metrics = _metric_slice(result.predictions, corpus, exposed_selection)
    critical_metrics = _error_metrics(
        result.predictions[critical_selection], corpus.actions[critical_selection]
    )
    absolute_error = np.abs(
        result.predictions.astype(np.float64) - corpus.actions.astype(np.float64)
    )
    flat_index = int(np.argmax(absolute_error))
    row_index, action_dimension = np.unravel_index(flat_index, absolute_error.shape)
    worst_row = {
        "absolute_error": float(absolute_error[row_index, action_dimension]),
        "action_dimension": int(action_dimension),
        "case_id": str(corpus.case_ids[row_index]),
        "step_index": int(corpus.step_indices[row_index]),
        "tranche_id": str(corpus.tranche_ids[row_index]),
    }
    history_path = destination / "adaptation_history.json"
    report_path = destination / "adaptation_report.json"
    summary_path = destination / "summary.json"
    gate_path = destination / "gate.json"
    receipt_path = destination / "receipt.json"
    forensic.write_json_exclusive(history_path, list(result.history))
    report = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R5_CASE_BALANCED_ADAPTATION_REPORT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "corpus_exact": True,
        "training_samples": contract.CORPUS_ROWS,
        "validation_samples": 0,
        "metrics": dict(result.metrics),
        "p2_subset_metrics": p2_metrics,
        "nominal_r4_pass_metrics": nominal_metrics,
        "nominal_r4_student_exposed_metrics": exposed_metrics,
        "per_case_metrics": per_case_metrics,
        "critical_window_metrics": critical_metrics,
        "critical_window_p2_baseline_metrics": corpus.audit[
            "critical_window_p2_baseline_metrics"
        ],
        "worst_row": worst_row,
        "weight_audit": copy.deepcopy(corpus.audit["weight_audit"]),
        "optimizer_audit": dict(result.optimizer_audit),
        "normalization_audit": dict(result.normalization_audit),
        "preservation_audit": dict(result.preservation_audit),
        "save_reload": save_reload,
        "module_reload_exact": save_reload.get("exact") is True,
        "source_records": copy.deepcopy(corpus.source_records),
    }
    forensic.write_json_exclusive(report_path, report)
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit": copy.deepcopy(contract.FIT),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_p2": False,
        "fit_counts": contract.expected_corpus_counts(),
        "sample_count": contract.CORPUS_ROWS,
        "episode_count": contract.CORPUS_EPISODE_COUNT,
        "reset_row_count": contract.CORPUS_RESET_ROWS,
        "corpus_components": ["p2_corpus", "r4_nominal_pass_labels_only"],
        "failed_plus_prefix_rows_loaded": 0,
        "weighting": copy.deepcopy(contract.WEIGHTING),
        "case_mass": copy.deepcopy(corpus.audit["weight_audit"]["case_mass"]),
        "weight_hashes": {
            name: corpus.audit["weight_audit"][name]
            for name in (
                "p2_max_abs_error_sha256",
                "hardness_sha256",
                "source_risk_sha256",
                "normalized_weights_sha256",
            )
        },
        "normalized_total_sample_mass": corpus.audit["weight_audit"][
            "normalized_total_sample_mass"
        ],
        "metrics": dict(result.metrics),
        "p2_subset_metrics": p2_metrics,
        "nominal_r4_pass_metrics": nominal_metrics,
        "nominal_r4_student_exposed_metrics": exposed_metrics,
        "per_case_metrics": per_case_metrics,
        "critical_window": copy.deepcopy(contract.CRITICAL_WINDOW),
        "critical_window_metrics": critical_metrics,
        "critical_window_p2_baseline_metrics": corpus.audit[
            "critical_window_p2_baseline_metrics"
        ],
        "critical_window_p2_baseline_recomputed": True,
        "critical_window_p2_module_tree": copy.deepcopy(contract.P2_MODULE_TREE),
        "worst_row": worst_row,
        "adamw_epochs_run": 3000,
        "lbfgs_max_iter": 600,
        "lbfgs_max_eval": 1200,
        "lbfgs_closure_calls": result.optimizer_audit["lbfgs_closure_calls"],
        "deterministic_algorithms_enabled": True,
        "source_h0_byte_exact": True,
        "logstd_byte_exact": result.preservation_audit.get(
            "logstd_parameter_rows_bit_exact"
        )
        is True,
        "critic_present": False,
        "disabled_clock_columns_bit_zero": result.preservation_audit.get(
            "disabled_clock_columns_bit_zero"
        )
        is True,
        "save_reload_exact": save_reload.get("exact") is True,
        "hard_polish_used": False,
        "fallback_used": False,
        "sweep_used": False,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "candidate_module": module_record,
        "candidate_id": contract.candidate_id(module_record["tree_sha256"]),
        "corpus": _record(destination / "corpus.npz"),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.fit_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R5CaseBalancedFitError("R5 offline fit gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": summary["candidate_id"],
        "candidate_module": module_record,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "corpus": _record(destination / "corpus.npz"),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(receipt_path, receipt)
    # The runner performs the authoritative stage verification before it
    # records completion.  Keeping that readback inside the runner's
    # terminalization boundary avoids a post-publication verification gap.
    return receipt


def verify_fit_stage() -> dict[str, Any]:
    root = _resolve(contract.FIT_ROOT)
    receipt = _mapping(root / "receipt.json")
    summary = _mapping(root / "summary.json")
    gate = _mapping(root / "gate.json")
    try:
        module = tree_record(contract.CANDIDATE_MODULE_PATH)
        expected_id = contract.candidate_id(module["tree_sha256"])
    except Exception as exc:
        raise V12R5CaseBalancedFitError("R5 candidate module drifted") from exc
    if not (
        _fit_summary_payload_valid(summary, module=module, expected_id=expected_id)
        and _fit_gate_payload_valid(gate, summary=summary)
        and _fit_receipt_payload_valid(
            receipt,
            summary=summary,
            gate=gate,
            module=module,
            expected_id=expected_id,
        )
    ):
        raise V12R5CaseBalancedFitError("R5 fit receipt/binding drifted")
    return receipt


__all__ = [
    "V12R5CaseBalancedFitError",
    "adamw_learning_rate",
    "compute_case_balanced_weights",
    "fit_case_balanced_full_mean_in_memory",
    "load_case_balanced_corpus",
    "run_fit_stage",
    "verify_fit_stage",
]
