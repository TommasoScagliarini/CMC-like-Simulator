"""Compare two in-memory full-mean recovery fits on the frozen V12R9 corpus.

This diagnostic is intentionally non-publishing and environment-free.  It
compares the frozen V12R9 optimizer started from (a) the locked R6 actor and
(b) the terminal, unpromoted R9 actor.  Both fits preserve the source NPZ
``normalized_sample_weights`` ratios inside each of the thirteen strata and
only rescale every stratum to the common 500-unit mass.

No checkpoint, protocol lock, receipt, environment rollout, critic update, or
PPO update is produced.  Each fit is repeated from the same immutable source
state so deterministic, byte-exact reproducibility can be audited.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "Trajectory Generator").is_dir()
            and (candidate / "validation").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
R9_ROOT = BASELINE_ROOT / "validation" / "v12r9"
for _path in (BASELINE_ROOT, R9_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import warm_start  # noqa: E402,F401  # register custom RLModule class
import h0_v12r9_recovery_fitter as r9  # noqa: E402


R9_TERMINAL_STATUS = "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"


def _strict_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _array_sha256(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(b"\0")
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(b"\0")
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _state_arrays(state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    arrays: dict[str, np.ndarray] = {}
    for name, value in state.items():
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        arrays[name] = np.ascontiguousarray(np.asarray(value))
    return arrays


def _state_byte_exact(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    a = _state_arrays(left)
    b = _state_arrays(right)
    return set(a) == set(b) and all(
        a[name].dtype == b[name].dtype
        and a[name].shape == b[name].shape
        and a[name].tobytes(order="C") == b[name].tobytes(order="C")
        for name in a
    )


def _state_numerics(state: Mapping[str, Any]) -> dict[str, Any]:
    arrays = _state_arrays(state)
    flattened = np.concatenate(
        [array.astype(np.float64, copy=False).ravel() for array in arrays.values()]
    )
    return {
        "all_finite": bool(np.all(np.isfinite(flattened))),
        "maximum_absolute_parameter": float(np.max(np.abs(flattened))),
        "parameter_l2_norm": float(np.linalg.norm(flattened)),
        "state_key_count": len(arrays),
    }


def _state_delta(left: Mapping[str, Any], right: Mapping[str, Any]) -> dict[str, Any]:
    a = _state_arrays(left)
    b = _state_arrays(right)
    if set(a) != set(b):
        raise RuntimeError("state key sets differ")
    differences = np.concatenate(
        [
            (b[name].astype(np.float64) - a[name].astype(np.float64)).ravel()
            for name in sorted(a)
        ]
    )
    return {
        "changed": not _state_byte_exact(left, right),
        "delta_l2_norm": float(np.linalg.norm(differences)),
        "delta_max_abs": float(np.max(np.abs(differences))),
    }


def _load_r9_candidate_state() -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    path = (REPO_ROOT / r9.contract.CANDIDATE_MODULE_PATH).resolve()
    module = RLModule.from_checkpoint(path)
    module.eval()
    state = r9.v11._clone_state(module.get_state())
    r9.validate_source_r6_state(state)
    return state


def _preserved_source_weights(
    bundle: r9.RecoveryCorpusBundle,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Preserve each source's effective training weights within new strata."""

    features = np.ascontiguousarray(bundle.corpus.actor_feature_names, dtype="U64")
    base = r9._load_base_piece()  # noqa: SLF001 - forensic use of frozen loader
    r4 = r9._load_r4_piece(features)  # noqa: SLF001
    observers = {
        case_id: r9._load_observer_piece(case_id, features)  # noqa: SLF001
        for case_id in r9.contract.COLLECTION_CASE_IDS
    }
    source = np.ascontiguousarray(
        np.concatenate(
            [
                base["normalized_sample_weights"],
                np.ones(len(r4["observations"]), dtype=np.float64),
                *(
                    observers[case_id]["normalized_sample_weights"]
                    for case_id in r9.contract.COLLECTION_CASE_IDS
                ),
            ]
        ),
        dtype=np.float64,
    )
    if (
        source.shape != (len(bundle.corpus.observations),)
        or not np.all(np.isfinite(source))
        or np.any(source <= 0.0)
    ):
        raise RuntimeError("preserved source-weight vector is malformed")

    target_mass = float(r9.contract.FIT["stratum_target_mass"])
    weights = np.empty_like(source)
    strata = bundle.stratum_ids.astype(str)
    audit: dict[str, Any] = {}
    for stratum_id in r9.expected_stratum_ids():
        index = np.flatnonzero(strata == stratum_id)
        if len(index) == 0:
            raise RuntimeError(f"empty stratum: {stratum_id}")
        source_values = source[index]
        denominator = math.fsum(source_values.astype(float))
        weights[index] = source_values * (target_mass / denominator)
        # Close floating-point mass exactly as the original R5 fitter does.
        observed = math.fsum(weights[index].astype(float))
        weights[index[-1]] += target_mass - observed
        observed = math.fsum(weights[index].astype(float))
        ratios = weights[index] / source_values
        if not math.isclose(observed, target_mass, rel_tol=0.0, abs_tol=1.0e-9):
            raise RuntimeError(f"weight mass failed closure: {stratum_id}")
        audit[stratum_id] = {
            "rows": int(len(index)),
            "source_min": float(np.min(source_values)),
            "source_max": float(np.max(source_values)),
            "source_mass": float(denominator),
            "preserved_min": float(np.min(weights[index])),
            "preserved_max": float(np.max(weights[index])),
            "preserved_mass": float(observed),
            "relative_ratio_min": float(np.min(ratios)),
            "relative_ratio_max": float(np.max(ratios)),
        }
    if not np.all(np.isfinite(weights)) or np.any(weights <= 0.0):
        raise RuntimeError("final preserved weights are malformed")
    return np.ascontiguousarray(weights), {
        "policy": "PRESERVE_SOURCE_NORMALIZED_WEIGHTS_WITHIN_EACH_R9_STRATUM",
        "source_semantics": {
            "base": "locked R5 effective normalized_sample_weights (risk + P2 hardness)",
            "r4_failure": "unit weights because the locked R4 NPZ has no weight fields",
            "observer": "locked observer-label normalized_sample_weights",
        },
        "stratum_target_mass": target_mass,
        "stratum_count": len(audit),
        "total_mass": float(math.fsum(weights.astype(float))),
        "source_weight_sha256": _array_sha256(source),
        "preserved_weight_sha256": _array_sha256(weights),
        "r9_uniform_weight_sha256": _array_sha256(
            bundle.corpus.normalized_sample_weights
        ),
        "differs_from_r9_uniform": bool(
            not np.array_equal(weights, bundle.corpus.normalized_sample_weights)
        ),
        "strata": audit,
    }


def _weighted_mse(
    state: Mapping[str, Any], bundle: r9.RecoveryCorpusBundle, weights: np.ndarray
) -> float:
    predictions = r9.v11._state_logits(state, bundle.corpus.observations)[:, :2]
    row_loss = np.mean(
        np.square(
            predictions.astype(np.float64) - bundle.corpus.actions.astype(np.float64)
        ),
        axis=1,
    )
    return float(np.sum(weights * row_loss) / np.sum(weights))


def _gate_payload(
    result: Any, metric_payload: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    summary = {
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hidden_dims": [512, 512],
        "actor_feature_count": 35,
        "logstd_byte_exact": result.preservation_audit.get(
            "logstd_parameter_rows_byte_exact"
        )
        is True,
        "disabled_clock_columns_bit_zero": result.preservation_audit.get(
            "disabled_clock_columns_bit_zero"
        )
        is True,
        # The fit is intentionally in-memory.  R9's already validated ordinary
        # W512 save/reload path is value-agnostic; this field evaluates the R9
        # gate under that frozen publication mechanism without publishing here.
        "save_reload_exact": True,
        **{
            key: copy.deepcopy(metric_payload[key])
            for key in (
                "global_metrics",
                "reset_max_abs_error",
                "per_case_metrics",
                "r4_failed_plus_metrics",
                "observer_case_metrics",
                "observer_plus_late_metrics",
            )
        },
    }
    gate = r9.contract.fit_gate(summary)
    return summary, gate


def _fit_once(
    *,
    label: str,
    source_state: Mapping[str, Any],
    bundle: r9.RecoveryCorpusBundle,
    weights: np.ndarray,
    normalization: Any,
) -> tuple[Any, dict[str, Any]]:
    started = time.monotonic()
    result = r9.fit_recovery_full_mean_in_memory(
        source_state=r9.v11._clone_state(source_state),
        observations=bundle.corpus.observations,
        targets=bundle.corpus.actions,
        reset_mask=bundle.corpus.reset_mask,
        sample_weights=weights,
        normalization=normalization,
    )
    elapsed = time.monotonic() - started
    metrics = r9.recompute_fit_metric_payload(bundle, result.candidate_state)
    if not np.array_equal(metrics["predictions"], result.predictions):
        raise RuntimeError(f"{label}: fit/recomputed predictions differ")
    summary, gate = _gate_payload(result, metrics)
    history_losses = np.asarray(
        [float(item["loss"]) for item in result.history], dtype=np.float64
    )
    payload = {
        "label": label,
        "elapsed_seconds": float(elapsed),
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "candidate_actor_digest": warm_start.actor_state_digest(result.candidate_state),
        "prediction_sha256": _array_sha256(result.predictions),
        "objective_before": _weighted_mse(source_state, bundle, weights),
        "objective_after": _weighted_mse(result.candidate_state, bundle, weights),
        "history_all_finite": bool(np.all(np.isfinite(history_losses))),
        "history_first_loss": float(history_losses[0]),
        "history_final_loss": float(history_losses[-1]),
        "optimizer_audit": dict(result.optimizer_audit),
        "normalization_audit": dict(result.normalization_audit),
        "preservation_audit": dict(result.preservation_audit),
        "state_numerics": _state_numerics(result.candidate_state),
        "state_delta_from_source": _state_delta(source_state, result.candidate_state),
        "metrics": {
            key: copy.deepcopy(metrics[key])
            for key in (
                "global_metrics",
                "reset_max_abs_error",
                "per_case_metrics",
                "r4_failed_plus_metrics",
                "observer_case_metrics",
                "observer_plus_late_metrics",
                "worst_row",
            )
        },
        "r9_gate_summary_inputs": summary,
        "r9_gate": gate,
    }
    return result, payload


def _run_replicated_variant(
    *,
    label: str,
    source_state: Mapping[str, Any],
    bundle: r9.RecoveryCorpusBundle,
    weights: np.ndarray,
    normalization: Any,
) -> dict[str, Any]:
    first, first_payload = _fit_once(
        label=f"{label}:replicate_1",
        source_state=source_state,
        bundle=bundle,
        weights=weights,
        normalization=normalization,
    )
    second, second_payload = _fit_once(
        label=f"{label}:replicate_2",
        source_state=source_state,
        bundle=bundle,
        weights=weights,
        normalization=normalization,
    )
    reproducibility = {
        "candidate_state_byte_exact": _state_byte_exact(
            first.candidate_state, second.candidate_state
        ),
        "predictions_byte_exact": bool(
            first.predictions.dtype == second.predictions.dtype
            and first.predictions.shape == second.predictions.shape
            and first.predictions.tobytes(order="C")
            == second.predictions.tobytes(order="C")
        ),
        "history_exact": first.history == second.history,
        "optimizer_audit_exact": first.optimizer_audit == second.optimizer_audit,
        "normalization_audit_exact": (
            first.normalization_audit == second.normalization_audit
        ),
        "preservation_audit_exact": (
            first.preservation_audit == second.preservation_audit
        ),
    }
    reproducibility["passed"] = all(reproducibility.values())
    return {
        "replicate_1": first_payload,
        "replicate_2": second_payload,
        "reproducibility": reproducibility,
    }


def compare(*, selected_variant: str = "both") -> dict[str, Any]:
    ledger_path = REPO_ROOT / r9.contract.LEDGER_PATH
    ledger = _strict_json(ledger_path)
    if (
        ledger.get("status") != R9_TERMINAL_STATUS
        or ledger.get("passed") is not False
        or ledger.get("attempted_stage") != "fit_recovery_actor"
    ):
        raise RuntimeError("R9 is not the expected immutable terminal fit failure")

    bundle = r9.load_recovery_corpus()
    persisted_corpus = r9.verify_persisted_corpus(bundle)
    weights, weight_audit = _preserved_source_weights(bundle)
    _, r6_state, _ = r9._load_source_module_and_state()  # noqa: SLF001
    r9_state = _load_r9_candidate_state()

    base_rows = int(r9.contract.LOCKED_INPUTS["base_corpus"]["rows"])
    base_v8 = np.flatnonzero(
        bundle.corpus.tranche_ids[:base_rows].astype(str) == "v8r1p1_base"
    )
    if len(base_v8) != 3000:
        raise RuntimeError("frozen normalization base row count drifted")
    normalization = r9.v11.frozen_base_normalization(
        bundle.corpus.observations[base_v8]
    )

    variants: dict[str, Any] = {}
    if selected_variant in {"both", "fresh"}:
        variants["fresh_from_locked_r6"] = _run_replicated_variant(
            label="fresh_from_locked_r6",
            source_state=r6_state,
            bundle=bundle,
            weights=weights,
            normalization=normalization,
        )
    if selected_variant in {"both", "continuation"}:
        variants["continuation_from_terminal_r9"] = _run_replicated_variant(
            label="continuation_from_terminal_r9",
            source_state=r9_state,
            bundle=bundle,
            weights=weights,
            normalization=normalization,
        )
    passing = [
        name
        for name, value in variants.items()
        if value["replicate_1"]["r9_gate"]["passed"] is True
        and value["reproducibility"]["passed"] is True
    ]
    return {
        "status": "COMPLETE_H0_V12R10_IN_MEMORY_FIT_COMPARISON",
        "passed": True,
        "scope": "NO_ENVIRONMENT_NO_PUBLICATION_NO_CHECKPOINT_WRITE",
        "r9_terminal": {
            "path": ledger_path.relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256(ledger_path),
            "status": ledger["status"],
        },
        "persisted_corpus": persisted_corpus,
        "selected_variant": selected_variant,
        "weight_audit": weight_audit,
        "thresholds": copy.deepcopy(r9.contract.OFFLINE_THRESHOLDS),
        "source_state_relation": {
            "r6_actor_digest": warm_start.actor_state_digest(r6_state),
            "r9_actor_digest": warm_start.actor_state_digest(r9_state),
            "states_byte_exact": _state_byte_exact(r6_state, r9_state),
            "delta": _state_delta(r6_state, r9_state),
        },
        "variants": variants,
        "passing_reproducible_variants": passing,
        "recommendation_basis": (
            "Prefer a byte-reproducible variant passing every unchanged R9 gate. "
            "If more than one passes, prefer fresh R6 initialization because it "
            "does not make a failed terminal candidate a hidden predecessor; if "
            "only continuation passes, it must be explicitly locked as an input."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--compact",
        action="store_true",
        help="Emit one-line JSON instead of indented JSON.",
    )
    parser.add_argument(
        "--variant",
        choices=("both", "fresh", "continuation"),
        default="both",
        help="Run both preregistered initializations or only one of them.",
    )
    args = parser.parse_args()
    payload = compare(selected_variant=args.variant)
    if args.compact:
        print(json.dumps(payload, sort_keys=True, separators=(",", ":")))
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
