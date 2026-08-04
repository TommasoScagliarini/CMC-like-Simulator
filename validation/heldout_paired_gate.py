"""One-shot paired held-out gate for the selected PPO pilot candidate.

The workflow is deliberately split in two irreversible stages:

``seal``
    Validates the completed development evidence, checks that the explicitly
    named candidate is the unique winner under the preregistered ordering, and
    binds its actor and RLModule artifact digests.  This stage does not inspect
    or launch held-out rollouts.

``open``
    Consumes only that immutable seal, creates a no-clobber opening receipt,
    and then runs the fixed paired matrix: H0 and the one candidate for each of
    three starts and three held-out seeds.  All rollouts are stochastic with
    sigma 0.005.  Candidate reserve use is compared with H0 in the exact same
    seed/start cell.

There is no checkpoint argument in ``open`` and no retry/fallback mode.  Once
the receipt exists, a second opening is refused even if the first execution is
incomplete.  The tool never copies or promotes a checkpoint.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

if __package__:
    from . import robust_ppo_gate as gate
    from .compare_policy_checkpoints import _actor_digest, _load_state
else:  # pragma: no cover - exercised by direct CLI use.
    import robust_ppo_gate as gate
    from compare_policy_checkpoints import _actor_digest, _load_state


ROOT_DIR = Path(__file__).resolve().parents[1]
SCHEMA_VERSION = 1
EXPECTED_SEEDS = (126, 127, 128)
EXPECTED_START_OFFSETS_S = gate.DEFAULT_START_OFFSETS_S
EXPECTED_SIGMA = 0.005
SEAL_FILENAME = "heldout_candidate_seal.json"
OPEN_RECEIPT_FILENAME = "heldout_open_receipt.json"
REPORT_FILENAME = "heldout_paired_gate.json"
ROLLOUT_ROOT_NAME = "heldout_rollouts"
EXPECTED_SELECTION_ORDER = (
    "primary_worst_condition_matched_reserve_ratio_vs_h0",
    "tie_breaker_worst_penetration_ratio_vs_h0",
    "tie_breaker_cumulative_empirical_kl_from_h0_mean",
    "tie_breaker_earlier_pilot_update",
)
EXPECTED_ELIGIBLE_SET = (
    "only preregistered screened milestones that pass every development gate"
)
EXPECTED_PRIMARY_ORDER = (
    "minimum worst condition-matched reserve ratio versus H0"
)
EXPECTED_TIE_BREAKERS = (
    "minimum worst penetration ratio versus H0",
    "minimum cumulative empirical KL versus H0",
    "earlier pilot update",
)
EXPECTED_EVIDENCE_INPUTS = (
    "protocol",
    "selection_addendum",
    "pilot_screen",
    "policy_drift",
    "training_summary",
    "training_history",
)


class HeldoutRefusal(ValueError):
    """Raised when a fail-closed held-out contract is not satisfied."""


@dataclass(frozen=True)
class MatrixRun:
    seed: int
    start_index: int
    start_name: str
    offset_s: float
    role: str
    checkpoint: Path

    @property
    def cell_name(self) -> str:
        return f"seed_{self.seed}/{self.start_name}"

    @property
    def run_name(self) -> str:
        return f"{self.cell_name}/{self.role}"


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise HeldoutRefusal(message)


def _reject_nonfinite_json(token: str) -> None:
    raise HeldoutRefusal(f"non-finite JSON number {token!r}")


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_nonfinite_json,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise HeldoutRefusal(f"could not read valid {label}: {path}: {exc}") from exc
    _require(isinstance(value, Mapping), f"{label} is not a JSON object: {path}")
    return dict(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise HeldoutRefusal(f"could not hash {path}: {exc}") from exc
    return digest.hexdigest()


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    _require(
        not isinstance(value, bool) and isinstance(value, (int, float)),
        f"{label} is not numeric",
    )
    number = float(value)
    _require(math.isfinite(number), f"{label} is not finite")
    if minimum is not None:
        _require(number >= minimum, f"{label} is below {minimum}")
    return number


def _integer(value: Any, label: str, *, minimum: int | None = None) -> int:
    _require(type(value) is int, f"{label} is not an integer")
    if minimum is not None:
        _require(value >= minimum, f"{label} is below {minimum}")
    return int(value)


def _mapping(value: Any, label: str) -> dict[str, Any]:
    _require(isinstance(value, Mapping), f"{label} is not an object")
    return dict(value)


def _sequence(value: Any, label: str) -> list[Any]:
    _require(
        isinstance(value, Sequence) and not isinstance(value, (str, bytes)),
        f"{label} is not an array",
    )
    return list(value)


def _same_path(left: Any, right: Path) -> bool:
    return gate._same_path(left, right)


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _write_json_no_clobber(path: Path, payload: Mapping[str, Any]) -> None:
    """Atomically publish one immutable JSON file without replacing a path."""

    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        raise HeldoutRefusal(f"refusing existing immutable output: {path}")
    encoded = (
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    staging = path.parent / f".{path.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    descriptor: int | None = None
    try:
        descriptor = os.open(staging, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(staging, path)
        except FileExistsError as exc:
            raise HeldoutRefusal(
                f"refusing existing immutable output: {path}"
            ) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            staging.unlink()
        except FileNotFoundError:
            pass


def _write_json_replace(path: Path, payload: Mapping[str, Any]) -> None:
    """Atomically update the progressive/final gate report."""

    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.parent / f".{path.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    staging.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(staging, path)


def _checkpoint_identity(checkpoint: Path) -> dict[str, Any]:
    path = checkpoint.expanduser().resolve(strict=False)
    artifact = gate._checkpoint_artifact_report(path)
    _require(
        artifact.get("status") == "PASS",
        f"RLModule checkpoint contract failed: {path}: {artifact.get('failures')}",
    )
    try:
        actor_digest = _actor_digest(_load_state(path))
    except (OSError, ValueError, TypeError) as exc:
        raise HeldoutRefusal(f"could not compute actor digest: {path}: {exc}") from exc
    _require(_is_sha256(actor_digest), f"invalid actor digest computed for {path}")
    return {
        "path": str(path),
        "actor_digest": actor_digest,
        "artifacts": {
            name: details["sha256"]
            for name, details in sorted(_mapping(artifact["files"], "artifacts").items())
        },
    }


def _validate_protocol(protocol: Mapping[str, Any]) -> dict[str, Any]:
    selection = _mapping(protocol.get("candidate_selection"), "candidate_selection")
    _require(
        selection.get("eligible_set") == EXPECTED_ELIGIBLE_SET,
        "protocol eligible-candidate scope differs from the frozen contract",
    )
    _require(
        selection.get("primary_order") == EXPECTED_PRIMARY_ORDER,
        "protocol primary candidate ordering differs from the frozen contract",
    )
    _require(
        selection.get("tie_breakers") == list(EXPECTED_TIE_BREAKERS),
        "protocol candidate tie-breakers differ from the frozen contract",
    )
    _require(
        selection.get("maximum_candidates_opened_on_held_out") == 1,
        "protocol does not limit held-out opening to one candidate",
    )
    _require(
        selection.get("fallback_after_held_out_failure") is False,
        "protocol permits fallback after held-out failure",
    )
    _require(
        selection.get("checkpoint_best_is_not_a_selector") is True,
        "protocol permits checkpoint_best to select the candidate",
    )
    heldout = _mapping(protocol.get("held_out_gate"), "held_out_gate")
    _require(
        heldout.get("status") == "sealed_until_one_candidate_digest_is_fixed",
        "held-out protocol is not SEALED pending one candidate digest",
    )
    _require(
        heldout.get("seeds") == list(EXPECTED_SEEDS),
        "held-out seed set or order differs from the frozen contract",
    )
    offsets = tuple(
        _finite(value, "held-out start offset")
        for value in _sequence(heldout.get("start_offsets_s"), "held-out starts")
    )
    _require(
        len(offsets) == 3
        and all(
            math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12)
            for actual, expected in zip(offsets, EXPECTED_START_OFFSETS_S)
        ),
        "held-out start offsets differ from the frozen three-start contract",
    )
    sigma = _finite(heldout.get("sigma"), "held-out sigma", minimum=0.0)
    _require(
        math.isclose(sigma, EXPECTED_SIGMA, rel_tol=0.0, abs_tol=1.0e-12),
        "held-out sigma differs from 0.005",
    )
    _require(
        heldout.get("paired_reference") == "H0 in every seed/start cell",
        "held-out protocol is not paired against H0 in every cell",
    )
    _require(
        heldout.get("reserve_mode") == gate.RESERVE_MODE_CONDITION_MATCHED,
        "held-out reserve mode is not condition-matched non-regression",
    )
    _require(heldout.get("open_once") is True, "held-out protocol is not one-shot")
    source = _mapping(protocol.get("source"), "source")
    development = _mapping(protocol.get("development_gate"), "development_gate")
    h0_path = Path(str(development.get("reference_checkpoint", ""))).expanduser()
    if not h0_path.is_absolute():
        h0_path = ROOT_DIR / h0_path
    h0_path = h0_path.resolve(strict=False)
    _require(
        _is_sha256(source.get("actor_digest")),
        "protocol H0 actor digest is not a SHA-256 digest",
    )
    return {
        "selection": selection,
        "heldout": heldout,
        "h0_path": h0_path,
        "h0_actor_digest": source["actor_digest"],
        "offsets": offsets,
        "sigma": sigma,
    }


def _ranking_tuple(row: Mapping[str, Any]) -> tuple[float, float, float, int]:
    keys = _mapping(row.get("preregistered_ordering_keys"), "ordering keys")
    _require(keys.get("eligible_for_ordering") is True, "candidate is not eligible")
    return (
        _finite(keys.get(EXPECTED_SELECTION_ORDER[0]), EXPECTED_SELECTION_ORDER[0], minimum=0.0),
        _finite(keys.get(EXPECTED_SELECTION_ORDER[1]), EXPECTED_SELECTION_ORDER[1], minimum=0.0),
        _finite(keys.get(EXPECTED_SELECTION_ORDER[2]), EXPECTED_SELECTION_ORDER[2], minimum=-1.0e-7),
        _integer(keys.get(EXPECTED_SELECTION_ORDER[3]), EXPECTED_SELECTION_ORDER[3], minimum=1),
    )


def _validate_evidence(
    evidence: Mapping[str, Any],
    *,
    evidence_path: Path,
    protocol_path: Path,
    protocol_sha256: str,
) -> tuple[list[dict[str, Any]], Path]:
    _require(evidence.get("schema_version") == 1, "development evidence schema changed")
    _require(
        evidence.get("table") == "ppo_pilot_candidate_evidence",
        "wrong development evidence table",
    )
    _require(evidence.get("ok") is True, "development evidence is not successful")
    _require(evidence.get("status") == "COMPLETE", "development evidence is incomplete")
    _require(
        _same_path(evidence.get("output"), evidence_path),
        "development evidence does not identify its own path",
    )
    contract = _mapping(evidence.get("contract"), "evidence contract")
    _require(contract.get("screening_complete") is True, "development screening is incomplete")
    _require(contract.get("held_out_data_read") is False, "development evidence already read held-out data")
    _require(contract.get("rollouts_launched") is False, "evidence builder launched unexpected rollouts")
    _require(contract.get("ranking_performed") is False, "development evidence was already ranked")
    _require(contract.get("checkpoint_selected") is False, "development evidence already selected a checkpoint")
    inputs = _mapping(evidence.get("inputs"), "evidence inputs")
    _require(
        set(inputs) == set(EXPECTED_EVIDENCE_INPUTS),
        "development evidence input set is incomplete or unexpected",
    )
    for name in EXPECTED_EVIDENCE_INPUTS:
        item = _mapping(inputs.get(name), f"evidence input {name}")
        source_path = Path(str(item.get("path", ""))).expanduser().resolve(strict=False)
        _require(_is_sha256(item.get("sha256")), f"evidence input {name} digest is invalid")
        _require(
            _sha256(source_path) == item["sha256"],
            f"evidence input {name} changed after the evidence table was built",
        )
    protocol_input = _mapping(inputs.get("protocol"), "evidence protocol input")
    _require(
        _same_path(protocol_input.get("path"), protocol_path)
        and protocol_input.get("sha256") == protocol_sha256,
        "development evidence is not bound to the current protocol snapshot",
    )
    rows = [
        _mapping(value, "evidence row")
        for value in _sequence(evidence.get("rows"), "evidence rows")
    ]
    _require(evidence.get("row_count") == len(rows), "development evidence row count changed")
    ordering = _mapping(
        evidence.get("preregistered_candidate_ordering"),
        "preregistered candidate ordering",
    )
    _require(ordering.get("eligible_set") == EXPECTED_ELIGIBLE_SET, "evidence eligible scope changed")
    _require(ordering.get("primary_order") == EXPECTED_PRIMARY_ORDER, "evidence primary ordering changed")
    _require(ordering.get("tie_breakers") == list(EXPECTED_TIE_BREAKERS), "evidence tie-breakers changed")
    _require(ordering.get("ordering_performed") is False, "evidence already performed ranking")
    _require(ordering.get("selected_logical_iteration") is None, "evidence already selected a candidate")
    eligible_iterations = _sequence(
        evidence.get("eligible_logical_iterations"), "eligible iterations"
    )
    eligible = [row for row in rows if row.get("screen_eligibility") == "ELIGIBLE"]
    _require(eligible, "no candidate is eligible for held-out evaluation")
    _require(
        [row.get("logical_iteration") for row in eligible] == eligible_iterations,
        "eligible candidate list differs from the evidence rows",
    )
    seen: set[int] = set()
    for row in eligible:
        iteration = _integer(row.get("logical_iteration"), "logical iteration", minimum=2)
        _require(iteration not in seen, "duplicate eligible logical iteration")
        seen.add(iteration)
        _ranking_tuple(row)
        drift = _mapping(row.get("policy_drift_from_h0"), "policy drift")
        _require(
            drift.get("audit_status") == "PASS",
            f"policy-drift audit failed at iteration {iteration}",
        )
        _require(
            drift.get("logstd_bit_exact") is True,
            f"logstd is not bit-exact to H0 at iteration {iteration}",
        )
        _require(
            _is_sha256(drift.get("candidate_actor_digest")),
            f"candidate actor digest is invalid at iteration {iteration}",
        )
    training_summary = _mapping(inputs.get("training_summary"), "training summary input")
    summary_path = Path(str(training_summary.get("path", ""))).expanduser().resolve(strict=False)
    _require(summary_path.name == "summary.json", "training summary input path is invalid")
    _require(summary_path.is_file(), f"training summary input no longer exists: {summary_path}")
    _require(
        _sha256(summary_path) == training_summary.get("sha256"),
        "training summary changed after development evidence was built",
    )
    return eligible, summary_path.parent


def _validate_restart_audit(
    audit: Mapping[str, Any],
    *,
    audit_path: Path,
    run_dir: Path,
    evidence: Mapping[str, Any],
) -> None:
    _require(audit.get("schema_version") == 1, "restart audit schema changed")
    _require(audit.get("ok") is True, "restart audit did not pass")
    _require(audit.get("status") == "PASS", "restart audit status is not PASS")
    _require(audit.get("failed_checks") == [], "restart audit contains failed checks")
    _require(_same_path(audit.get("run_dir"), run_dir), "restart audit targets another training run")
    checks = _mapping(audit.get("checks"), "restart audit checks")
    _require(checks and all(value is True for value in checks.values()), "restart audit checks are incomplete or non-PASS")
    contract = _mapping(audit.get("contract"), "restart audit contract")
    for field in (
        "post_run_only",
        "require_completed_summary",
        "require_scoped_driver_log",
        "require_no_supervisor_restart_or_skip",
        "require_no_ray_restart_evidence",
        "fault_tolerance_metrics_optional_but_fail_if_nonzero",
    ):
        _require(contract.get(field) is True, f"restart audit contract does not enforce {field}")

    inputs = _mapping(evidence.get("inputs"), "evidence inputs")
    expected_summary = Path(
        str(_mapping(inputs.get("training_summary"), "training summary input").get("path", ""))
    ).expanduser().resolve(strict=False)
    expected_history = Path(
        str(_mapping(inputs.get("training_history"), "training history input").get("path", ""))
    ).expanduser().resolve(strict=False)
    _require(_same_path(audit.get("summary_path"), expected_summary), "restart audit summary source differs from evidence")
    _require(_same_path(audit.get("history_path"), expected_history), "restart audit history source differs from evidence")
    _require(audit_path.is_file(), f"restart audit does not exist: {audit_path}")


def build_candidate_seal(
    *,
    protocol_path: str | Path,
    evidence_path: str | Path,
    restart_audit_path: str | Path,
    candidate_logical_iteration: int,
) -> tuple[Path, dict[str, Any]]:
    """Bind the one preregistered winner without opening held-out data."""

    protocol_file = Path(protocol_path).expanduser().resolve(strict=False)
    evidence_file = Path(evidence_path).expanduser().resolve(strict=False)
    restart_audit_file = Path(restart_audit_path).expanduser().resolve(strict=False)
    seal_file = evidence_file.parent / SEAL_FILENAME
    _require(not os.path.lexists(seal_file), f"candidate seal already exists: {seal_file}")
    protocol = _read_json(protocol_file, "pilot protocol")
    protocol_sha256 = _sha256(protocol_file)
    protocol_contract = _validate_protocol(protocol)
    evidence = _read_json(evidence_file, "development evidence table")
    evidence_sha256 = _sha256(evidence_file)
    eligible, run_dir = _validate_evidence(
        evidence,
        evidence_path=evidence_file,
        protocol_path=protocol_file,
        protocol_sha256=protocol_sha256,
    )
    restart_audit = _read_json(restart_audit_file, "training restart audit")
    restart_audit_sha256 = _sha256(restart_audit_file)
    _validate_restart_audit(
        restart_audit,
        audit_path=restart_audit_file,
        run_dir=run_dir,
        evidence=evidence,
    )

    ranked = sorted(eligible, key=_ranking_tuple)
    winner = ranked[0]
    selected_iteration = _integer(
        candidate_logical_iteration, "selected logical iteration", minimum=2
    )
    _require(
        winner.get("logical_iteration") == selected_iteration,
        "explicit candidate is not rank 1 under the preregistered ordering",
    )
    _require(
        sum(row.get("logical_iteration") == selected_iteration for row in eligible) == 1,
        "explicit candidate is not unique in the eligible set",
    )

    candidate_path = (
        run_dir
        / f"milestone_iteration_{selected_iteration:06d}"
        / "rl_module_last"
    ).resolve(strict=False)
    candidate_identity = _checkpoint_identity(candidate_path)
    expected_candidate_digest = _mapping(
        winner.get("policy_drift_from_h0"), "winner policy drift"
    ).get("candidate_actor_digest")
    _require(
        candidate_identity["actor_digest"] == expected_candidate_digest,
        "candidate actor digest differs from the completed development evidence",
    )

    h0_identity = _checkpoint_identity(protocol_contract["h0_path"])
    _require(
        h0_identity["actor_digest"] == protocol_contract["h0_actor_digest"],
        "H0 actor digest differs from the frozen protocol",
    )
    seal = {
        "schema_version": SCHEMA_VERSION,
        "artifact": "heldout_candidate_seal",
        "status": "SEALED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "output": str(seal_file),
        "held_out_data_read": False,
        "held_out_opened": False,
        "candidate_count": 1,
        "fallback_allowed": False,
        "inputs": {
            "protocol": {"path": str(protocol_file), "sha256": protocol_sha256},
            "development_evidence": {"path": str(evidence_file), "sha256": evidence_sha256},
            "training_restart_audit": {
                "path": str(restart_audit_file),
                "sha256": restart_audit_sha256,
            },
        },
        "selection": {
            "method": "explicit_candidate_validated_as_unique_preregistered_rank_1",
            "logical_iteration": selected_iteration,
            "pilot_update_index": winner.get("pilot_update_index"),
            "rank": 1,
            "eligible_candidate_count": len(eligible),
            "ordering_fields": list(EXPECTED_SELECTION_ORDER),
            "ordering_values": list(_ranking_tuple(winner)),
        },
        "candidate": candidate_identity,
        "reference_h0": h0_identity,
        "held_out_contract": {
            "state": "SEALED_UNTIL_OPEN_COMMAND",
            "seeds": list(EXPECTED_SEEDS),
            "start_offsets_s": list(protocol_contract["offsets"]),
            "action_selection": "stochastic",
            "sigma": protocol_contract["sigma"],
            "pairing": "H0_and_candidate_in_every_seed_start_cell",
            "cells": len(EXPECTED_SEEDS) * len(EXPECTED_START_OFFSETS_S),
            "rollouts": 2 * len(EXPECTED_SEEDS) * len(EXPECTED_START_OFFSETS_S),
            "reserve_mode": gate.RESERVE_MODE_CONDITION_MATCHED,
            "maximum_candidates": 1,
            "open_once": True,
            "fallback_after_failure": False,
        },
        "checkpoint_copied": False,
        "checkpoint_promoted": False,
    }
    _write_json_no_clobber(seal_file, seal)
    return seal_file, seal


def _validate_bound_identity(recorded: Any, label: str) -> dict[str, Any]:
    expected = _mapping(recorded, label)
    path = Path(str(expected.get("path", ""))).expanduser().resolve(strict=False)
    current = _checkpoint_identity(path)
    _require(current == expected, f"{label} changed after candidate sealing")
    return current


def validate_open_readiness(seal_path: str | Path) -> dict[str, Any]:
    """Validate the sealed one-candidate binding without opening held-out data."""

    seal_file = Path(seal_path).expanduser().resolve(strict=False)
    seal = _read_json(seal_file, "candidate seal")
    _require(seal.get("artifact") == "heldout_candidate_seal", "wrong seal artifact")
    _require(seal.get("status") == "SEALED", "candidate seal is not SEALED")
    _require(_same_path(seal.get("output"), seal_file), "candidate seal path mismatch")
    _require(seal.get("held_out_data_read") is False, "seal reports held-out data read")
    _require(seal.get("held_out_opened") is False, "seal reports held-out already opened")
    _require(seal.get("candidate_count") == 1, "seal does not bind exactly one candidate")
    _require(seal.get("fallback_allowed") is False, "seal permits fallback")
    contract = _mapping(seal.get("held_out_contract"), "held-out contract")
    _require(contract.get("state") == "SEALED_UNTIL_OPEN_COMMAND", "seal state is invalid")
    _require(contract.get("seeds") == list(EXPECTED_SEEDS), "seal seed matrix changed")
    _require(contract.get("start_offsets_s") == list(EXPECTED_START_OFFSETS_S), "seal start matrix changed")
    _require(contract.get("action_selection") == "stochastic", "seal action mode changed")
    _require(_finite(contract.get("sigma"), "seal sigma") == EXPECTED_SIGMA, "seal sigma changed")
    _require(contract.get("cells") == 9 and contract.get("rollouts") == 18, "seal matrix size changed")
    _require(contract.get("reserve_mode") == gate.RESERVE_MODE_CONDITION_MATCHED, "seal reserve mode changed")
    _require(contract.get("maximum_candidates") == 1, "seal candidate maximum changed")
    _require(contract.get("open_once") is True, "seal is not one-shot")
    _require(contract.get("fallback_after_failure") is False, "seal permits fallback")

    inputs = _mapping(seal.get("inputs"), "seal inputs")
    for name in ("protocol", "development_evidence", "training_restart_audit"):
        item = _mapping(inputs.get(name), f"seal input {name}")
        path = Path(str(item.get("path", ""))).expanduser().resolve(strict=False)
        _require(_is_sha256(item.get("sha256")), f"seal input {name} digest is invalid")
        _require(_sha256(path) == item["sha256"], f"seal input {name} changed after sealing")

    candidate = _validate_bound_identity(seal.get("candidate"), "candidate")
    reference = _validate_bound_identity(seal.get("reference_h0"), "reference H0")
    receipt = seal_file.parent / OPEN_RECEIPT_FILENAME
    report = seal_file.parent / REPORT_FILENAME
    rollout_root = seal_file.parent / ROLLOUT_ROOT_NAME
    _require(not os.path.lexists(receipt), f"held-out was already opened: {receipt}")
    _require(not os.path.lexists(report), f"held-out report already exists: {report}")
    _require(not os.path.lexists(rollout_root), f"held-out rollout root already exists: {rollout_root}")
    return {
        "seal_path": seal_file,
        "seal": seal,
        "seal_sha256": _sha256(seal_file),
        "candidate": candidate,
        "reference_h0": reference,
        "receipt_path": receipt,
        "report_path": report,
        "rollout_root": rollout_root,
    }


def planned_matrix(readiness: Mapping[str, Any]) -> tuple[MatrixRun, ...]:
    candidate_path = Path(str(_mapping(readiness["candidate"], "candidate")["path"]))
    h0_path = Path(str(_mapping(readiness["reference_h0"], "reference H0")["path"]))
    start_names = ("start_minus020", "start_nominal", "start_plus020")
    runs: list[MatrixRun] = []
    for seed in EXPECTED_SEEDS:
        for start_index, (start_name, offset) in enumerate(
            zip(start_names, EXPECTED_START_OFFSETS_S)
        ):
            runs.append(MatrixRun(seed, start_index, start_name, offset, "h0", h0_path))
            runs.append(MatrixRun(seed, start_index, start_name, offset, "candidate", candidate_path))
    return tuple(runs)


def _rollout_command(
    run: MatrixRun,
    *,
    output_dir: Path,
    python_executable: str,
    rollout_script: Path,
    timeout_s: float,
) -> list[str]:
    return [
        python_executable,
        str(rollout_script.expanduser().resolve(strict=False)),
        "--checkpoint",
        str(run.checkpoint.expanduser().resolve(strict=False)),
        "--output-dir",
        str(output_dir),
        "--episode-duration",
        "5.0",
        "--max-steps",
        str(gate.EXPECTED_STEPS),
        "--episode-start-offset-s",
        repr(run.offset_s),
        "--action-mode",
        "absolute",
        "--action-selection",
        "stochastic",
        "--seed",
        str(run.seed),
        "--run-timeout-s",
        str(timeout_s),
        "--no-record-outputs",
        "--no-record-policy-trace",
        "--no-progress",
    ]


def _summary_result(
    *,
    path: Path,
    checkpoint: Path,
    seed: int,
    offset_s: float,
    reserve_cap_nm: float,
    reserve_tolerance_nm: float,
) -> dict[str, Any]:
    try:
        summary = gate._read_json_object(path)
        result = gate.classify_rollout_summary(
            summary,
            expected_checkpoint=checkpoint,
            spec=gate.RolloutSpec(
                name=path.parent.name,
                offset_s=offset_s,
                action_selection="stochastic",
                seed=seed,
            ),
            expected_sigma=EXPECTED_SIGMA,
            max_reserve_norm_nm=reserve_cap_nm,
            reserve_numerical_tolerance_nm=reserve_tolerance_nm,
        )
        result["summary_path"] = str(path)
        result["summary_sha256"] = _sha256(path)
        return result
    except (OSError, ValueError) as exc:
        return {
            "status": "FAIL",
            "summary_path": str(path),
            "error": str(exc),
            "checks": [],
            "failed_checks": ["summary_read"],
        }


def open_heldout_gate(
    *,
    seal_path: str | Path,
    python_executable: str = sys.executable,
    rollout_script: str | Path = gate.DEFAULT_ROLLOUT_SCRIPT,
    rollout_timeout_s: float = 900.0,
    command_runner: Callable[..., Any] = subprocess.run,
) -> dict[str, Any]:
    """Irreversibly open and execute the fixed paired matrix exactly once."""

    readiness = validate_open_readiness(seal_path)
    executable = str(python_executable).strip()
    _require(executable, "Python executable cannot be empty")
    executable_path = Path(executable).expanduser()
    has_path_separator = any(
        separator and separator in executable for separator in (os.sep, os.altsep)
    )
    if has_path_separator:
        _require(
            executable_path.is_file(),
            f"Python executable does not exist: {executable_path}",
        )
    else:
        _require(
            shutil.which(executable) is not None,
            f"Python executable is not available on PATH: {executable}",
        )
    script = Path(rollout_script).expanduser().resolve(strict=False)
    _require(script.is_file(), f"rollout script does not exist: {script}")
    timeout = _finite(rollout_timeout_s, "rollout timeout", minimum=1.0)
    runs = planned_matrix(readiness)
    _require(len(runs) == 18, "internal held-out matrix is not 18 rollouts")

    receipt = {
        "schema_version": SCHEMA_VERSION,
        "artifact": "heldout_open_receipt",
        "status": "OPENED",
        "opened_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate_count": 1,
        "fallback_allowed": False,
        "seal": {
            "path": str(readiness["seal_path"]),
            "sha256": readiness["seal_sha256"],
        },
        "candidate_actor_digest": readiness["candidate"]["actor_digest"],
        "reference_h0_actor_digest": readiness["reference_h0"]["actor_digest"],
        "matrix": [run.run_name for run in runs],
        "rerun_allowed": False,
    }
    # This no-clobber write is the irreversible opening event and deliberately
    # occurs before the first held-out rollout is launched.
    _write_json_no_clobber(readiness["receipt_path"], receipt)

    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "gate": "heldout_paired_gate",
        "status": "IN_PROGRESS",
        "ok": False,
        "opened_once": True,
        "candidate_count": 1,
        "fallback_attempted": False,
        "seal": receipt["seal"],
        "receipt": str(readiness["receipt_path"]),
        "candidate": readiness["candidate"],
        "reference_h0": readiness["reference_h0"],
        "contract": {
            "seeds": list(EXPECTED_SEEDS),
            "start_offsets_s": list(EXPECTED_START_OFFSETS_S),
            "action_selection": "stochastic",
            "sigma": EXPECTED_SIGMA,
            "pairing": "H0_and_candidate_same_seed_same_start",
            "reserve_comparison": "candidate <= paired_H0 + max(1e-6, 1e-9*H0) Nm",
            "cells": 9,
            "rollouts": 18,
            "no_outcome_adaptive_fail_fast": True,
        },
        "invocations": [],
        "cells": [],
        "checkpoint_copied": False,
        "checkpoint_promoted": False,
    }
    _write_json_replace(readiness["report_path"], report)

    for run in runs:
        run_dir = readiness["rollout_root"] / run.cell_name / run.role
        command = _rollout_command(
            run,
            output_dir=run_dir,
            python_executable=executable,
            rollout_script=script,
            timeout_s=timeout,
        )
        invocation: dict[str, Any] = {
            "name": run.run_name,
            "seed": run.seed,
            "start_index": run.start_index,
            "offset_s": run.offset_s,
            "role": run.role,
            "checkpoint": str(run.checkpoint),
            "output_dir": str(run_dir),
            "command": command,
            "status": "FAIL",
        }
        bound_record = (
            readiness["seal"].get("reference_h0")
            if run.role == "h0"
            else readiness["seal"].get("candidate")
        )
        try:
            _validate_bound_identity(bound_record, f"{run.role} pre-rollout binding")
        except (HeldoutRefusal, OSError, ValueError) as exc:
            invocation["binding_preflight"] = "FAIL"
            invocation["error"] = str(exc)
        else:
            invocation["binding_preflight"] = "PASS"
        if invocation["binding_preflight"] != "PASS":
            pass
        elif run_dir.exists():
            invocation["error"] = "refusing pre-existing rollout directory"
        else:
            try:
                completed = command_runner(command, check=False)
                invocation["returncode"] = int(completed.returncode)
                invocation["status"] = (
                    "PASS" if invocation["returncode"] == 0 else "FAIL"
                )
                if invocation["returncode"] != 0:
                    invocation["error"] = (
                        f"rollout process returned {invocation['returncode']}"
                    )
            except (OSError, TypeError, ValueError) as exc:
                invocation["error"] = f"could not launch rollout: {exc}"
        report["invocations"].append(invocation)
        _write_json_replace(readiness["report_path"], report)

    cells: list[dict[str, Any]] = []
    for seed in EXPECTED_SEEDS:
        for start_index, (start_name, offset) in enumerate(
            zip(
                ("start_minus020", "start_nominal", "start_plus020"),
                EXPECTED_START_OFFSETS_S,
            )
        ):
            cell_root = readiness["rollout_root"] / f"seed_{seed}" / start_name
            h0_summary_path = cell_root / "h0" / "rollout_summary.json"
            candidate_summary_path = cell_root / "candidate" / "rollout_summary.json"
            try:
                h0_summary = gate._read_json_object(h0_summary_path)
                h0_reserve = _finite(
                    h0_summary.get("reserve_norm_max_nm"),
                    f"H0 reserve seed {seed} start {start_index}",
                    minimum=0.0,
                )
            except (OSError, ValueError) as exc:
                h0_reserve = 0.0
                h0_read_error = str(exc)
            else:
                h0_read_error = None
            h0_result = _summary_result(
                path=h0_summary_path,
                checkpoint=Path(readiness["reference_h0"]["path"]),
                seed=seed,
                offset_s=offset,
                reserve_cap_nm=h0_reserve,
                reserve_tolerance_nm=0.0,
            )
            if h0_read_error is not None:
                h0_result["reference_read_error"] = h0_read_error
            tolerance = gate._reserve_numerical_tolerance_nm(h0_reserve)
            candidate_result = _summary_result(
                path=candidate_summary_path,
                checkpoint=Path(readiness["candidate"]["path"]),
                seed=seed,
                offset_s=offset,
                reserve_cap_nm=h0_reserve,
                reserve_tolerance_nm=tolerance,
            )
            passed = (
                h0_result.get("status") == "PASS"
                and candidate_result.get("status") == "PASS"
            )
            cells.append(
                {
                    "seed": seed,
                    "start_index": start_index,
                    "start_name": start_name,
                    "offset_s": offset,
                    "status": "PASS" if passed else "FAIL",
                    "reserve_contract": {
                        "reference_h0_nm": h0_reserve,
                        "numerical_tolerance_nm": tolerance,
                        "maximum_candidate_inclusive_nm": h0_reserve + tolerance,
                    },
                    "reference_h0": h0_result,
                    "candidate": candidate_result,
                }
            )

    binding_error: str | None = None
    try:
        final_candidate = _validate_bound_identity(
            readiness["seal"].get("candidate"), "candidate"
        )
        final_reference = _validate_bound_identity(
            readiness["seal"].get("reference_h0"), "reference H0"
        )
        bindings_unchanged = (
            final_candidate == readiness["candidate"]
            and final_reference == readiness["reference_h0"]
        )
    except (HeldoutRefusal, OSError, ValueError) as exc:
        bindings_unchanged = False
        binding_error = str(exc)
    invocation_failures = [
        item["name"] for item in report["invocations"] if item["status"] != "PASS"
    ]
    failed_cells = [
        f"seed_{item['seed']}/{item['start_name']}"
        for item in cells
        if item["status"] != "PASS"
    ]
    passed = not invocation_failures and not failed_cells and bindings_unchanged
    report.update(
        {
            "status": "PASS" if passed else "FAIL",
            "ok": passed,
            "completed_at_utc": datetime.now(timezone.utc).isoformat(),
            "cells": cells,
            "failed_invocations": invocation_failures,
            "failed_cells": failed_cells,
            "bindings_unchanged": bindings_unchanged,
            "binding_error": binding_error,
            "fallback_attempted": False,
        }
    )
    _write_json_replace(readiness["report_path"], report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    seal = commands.add_parser("seal", help="Bind the one candidate; no held-out reads.")
    seal.add_argument("--protocol", type=Path, required=True)
    seal.add_argument("--evidence-table", type=Path, required=True)
    seal.add_argument("--restart-audit", type=Path, required=True)
    seal.add_argument("--candidate-logical-iteration", type=int, required=True)
    readiness = commands.add_parser(
        "check-seal", help="Readiness-only validation; never opens held-out data."
    )
    readiness.add_argument("--seal", type=Path, required=True)
    opened = commands.add_parser(
        "open", help="Irreversibly open and execute the fixed held-out matrix."
    )
    opened.add_argument("--seal", type=Path, required=True)
    opened.add_argument("--python-executable", default=sys.executable)
    opened.add_argument("--rollout-script", type=Path, default=gate.DEFAULT_ROLLOUT_SCRIPT)
    opened.add_argument("--rollout-timeout-s", type=float, default=900.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "seal":
            path, payload = build_candidate_seal(
                protocol_path=args.protocol,
                evidence_path=args.evidence_table,
                restart_audit_path=args.restart_audit,
                candidate_logical_iteration=args.candidate_logical_iteration,
            )
            result: dict[str, Any] = {
                "ok": True,
                "status": "SEALED",
                "output": str(path),
                "candidate": payload["candidate"],
                "held_out_opened": False,
            }
        elif args.command == "check-seal":
            checked = validate_open_readiness(args.seal)
            result = {
                "ok": True,
                "status": "READY_BUT_STILL_SEALED",
                "seal": str(checked["seal_path"]),
                "candidate_actor_digest": checked["candidate"]["actor_digest"],
                "held_out_opened": False,
            }
        else:
            result = open_heldout_gate(
                seal_path=args.seal,
                python_executable=args.python_executable,
                rollout_script=args.rollout_script,
                rollout_timeout_s=args.rollout_timeout_s,
            )
    except (HeldoutRefusal, OSError, ValueError) as exc:
        print(
            json.dumps({"ok": False, "status": "REFUSED", "error": str(exc)}, indent=2),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
