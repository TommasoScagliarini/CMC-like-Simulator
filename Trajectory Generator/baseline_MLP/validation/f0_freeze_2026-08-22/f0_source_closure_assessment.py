"""Retrospective source-closure assessment of the 28 schema-4 rollout receipts.

The 28 replay/det jobs were executed under receipt schema 4, BEFORE the
contemporaneous closure protocol of schema 5 existed. Their receipts are
immutable evidence and are NOT retrofitted: this sidecar correlates them,
read-only, with the current state of the repository and writes ONE new
no-clobber artefact ``manifest/source_closure_assessment_28jobs.json``.

What it attests (Class B, ``attestation_mode = retrospective_correlated_evidence``):
  - SHA-256 of every one of the 28 receipt files, of ``freeze_inventory.json``
    and of both completed execution manifests;
  - for every receipt: git HEAD, rollout_eval / config / module digests as
    recorded, and whether they equal the CURRENT values on disk;
  - the current git HEAD/status, the current runtime-core source table, the
    native plugin binaries and the data assets with their digests
    (``f0_closure``), and the current environment fingerprint.

What it can NOT attest (irrecoverable for these 28 jobs): no proof against
transient modifications between digest and load time, no per-process proof of
the native plugin bytes mapped, no complete imported-module / native
dependency closure, no per-job pre/post closure. It therefore never claims
bit-exactness nor Class A.
"""

from __future__ import annotations

import glob
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_closure as CL  # noqa: E402
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402

ASSESSMENT_FILE = "source_closure_assessment_28jobs.json"
EXPECTED_RECEIPTS = 28
EXPECTED_FAMILIES = ("replay", "det")
EXPECTED_SCHEMA = 4
ATTESTATION_MODE = "retrospective_correlated_evidence"


class AssessmentError(RuntimeError):
    """Raised when the evidence set is not exactly the expected one (fail-closed)."""


def _sha_entry(path: Path) -> dict[str, Any]:
    if path.is_symlink():
        raise AssessmentError(f"symlink not accepted as evidence: {C.rel(path)}")
    if not path.is_file():
        raise AssessmentError(f"evidence file missing: {C.rel(path)}")
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "bytes": path.stat().st_size}


def receipt_entry(path: Path, current: dict[str, Any]) -> dict[str, Any]:
    entry = _sha_entry(path)
    receipt = C.read_json(path)
    if not isinstance(receipt, dict):
        raise AssessmentError(f"receipt is not a mapping: {C.rel(path)}")
    if receipt.get("schema_version") != EXPECTED_SCHEMA:
        raise AssessmentError(f"receipt {C.rel(path)} has schema_version {receipt.get('schema_version')!r}, expected {EXPECTED_SCHEMA}")
    if receipt.get("family") not in EXPECTED_FAMILIES:
        raise AssessmentError(f"receipt {C.rel(path)} has family {receipt.get('family')!r}, expected one of {EXPECTED_FAMILIES}")
    config_path = C.REPO / str(receipt.get("config", ""))
    module_path = C.REPO / str(receipt.get("module", "")) / "module_state.pkl"
    entry.update(
        {
            "job_id": receipt.get("job_id"),
            "family": receipt.get("family"),
            "candidate": receipt.get("candidate"),
            "runtime": receipt.get("runtime"),
            "status": receipt.get("status"),
            "returncode": receipt.get("returncode"),
            "schema_version": receipt.get("schema_version"),
            "started_at_utc": receipt.get("started_at_utc"),
            "finished_at_utc": receipt.get("finished_at_utc"),
            "recorded": {
                "git_head": receipt.get("git_head"),
                "rollout_eval_sha256": receipt.get("rollout_eval_sha256"),
                "config": receipt.get("config"),
                "config_sha256": receipt.get("config_sha256"),
                "module_state_sha256": receipt.get("module_state_sha256"),
                "summary_sha256": receipt.get("summary_sha256"),
                "trace_sha256": receipt.get("trace_sha256"),
                "python": receipt.get("python"),
            },
            "correlation_with_current_tree": {
                "git_head_equals_current": receipt.get("git_head") == current["git_head"],
                "rollout_eval_sha256_equals_current": receipt.get("rollout_eval_sha256") == current["rollout_eval_sha256"],
                "config_sha256_equals_disk": config_path.is_file() and not config_path.is_symlink() and C.sha256_file(config_path) == receipt.get("config_sha256"),
                "module_state_sha256_equals_disk": module_path.is_file() and not module_path.is_symlink() and C.sha256_file(module_path) == receipt.get("module_state_sha256"),
            },
        }
    )
    return entry


def build_assessment(receipt_paths: list[Path], inventory_path: Path, execution_manifest_paths: list[Path], snapshot: dict[str, Any], *, expected_receipts: int = EXPECTED_RECEIPTS) -> dict[str, Any]:
    """Assemble the assessment (read-only over the evidence). Fails closed when
    the receipt set is not exactly ``expected_receipts`` schema-4 receipts or
    an evidence file is missing."""
    if len(receipt_paths) != expected_receipts:
        raise AssessmentError(f"expected exactly {expected_receipts} receipts, found {len(receipt_paths)}")
    current = {"git_head": snapshot["git"]["head"], "rollout_eval_sha256": next(r["sha256"] for r in snapshot["runtime_core"] if r["path"] == C.rel(C.ROLLOUT_EVAL))}
    receipts = [receipt_entry(p, current) for p in sorted(receipt_paths)]
    ids = [r["job_id"] for r in receipts]
    if len(set(ids)) != len(ids):
        raise AssessmentError("duplicate job_id among the receipts")
    if len(execution_manifest_paths) < 1:
        raise AssessmentError("at least one completed execution manifest is required")
    manifests = []
    for p in sorted(execution_manifest_paths):
        entry = _sha_entry(p)
        payload = C.read_json(p)
        entry.update({"mode": payload.get("mode"), "job_count": payload.get("job_count"), "generated_at_utc": payload.get("generated_at_utc"), "git_head": (payload.get("git") or {}).get("head")})
        manifests.append(entry)
    correlation = {
        "receipts_with_git_head_equal_current": sum(1 for r in receipts if r["correlation_with_current_tree"]["git_head_equals_current"]),
        "receipts_with_rollout_eval_equal_current": sum(1 for r in receipts if r["correlation_with_current_tree"]["rollout_eval_sha256_equals_current"]),
        "receipts_with_config_equal_disk": sum(1 for r in receipts if r["correlation_with_current_tree"]["config_sha256_equals_disk"]),
        "receipts_with_module_equal_disk": sum(1 for r in receipts if r["correlation_with_current_tree"]["module_state_sha256_equals_disk"]),
        "receipts_status_ok": sum(1 for r in receipts if r["status"] == "ok" and r["returncode"] == 0),
        "distinct_recorded_git_heads": sorted({str(r["recorded"]["git_head"]) for r in receipts}),
        "distinct_recorded_rollout_eval_sha256": sorted({str(r["recorded"]["rollout_eval_sha256"]) for r in receipts}),
    }
    return {
        "schema_version": 1,
        "artefact": ASSESSMENT_FILE,
        "generated_at_utc": C.utc_now(),
        "attestation_mode": ATTESTATION_MODE,
        "provenance_class": CL.PROVENANCE_CLASS_LEGACY,
        "bit_exact_claimed": False,
        "class_A_claimed": False,
        "receipt_schema_version": EXPECTED_SCHEMA,
        "receipts_retrofitted": False,
        "receipt_count": len(receipts),
        "receipts": receipts,
        "freeze_inventory": _sha_entry(inventory_path),
        "execution_manifests": manifests,
        "correlation_summary": correlation,
        "current_git": snapshot["git"],
        "current_runtime_source_closure_digest": snapshot["runtime_source_closure_digest"],
        "current_runtime_core": snapshot["runtime_core"],
        "current_native_plugins": snapshot["native_plugins"],
        "current_data_assets": snapshot["data_assets"],
        "current_orchestration_digest": snapshot["orchestration_digest"],
        "current_environment_fingerprint": snapshot["environment_fingerprint"],
        "what_this_attests": "correlazione retrospettiva, a sola lettura, fra le 28 receipt schema 4 (replay 9 + det 19), freeze_inventory.json, i manifest di esecuzione completati e lo stato corrente di sorgenti runtime-core, plugin nativi e asset dati; i valori registrati nelle receipt (HEAD, SHA di rollout_eval/config/modulo) sono confrontati con il disco corrente",
        "what_this_does_not_attest": "nessuna riproduzione bit-exact e nessuna classe A: i 28 job sono stati eseguiti dal working tree vivo senza chiusura pre/post per job; la coincidenza dei digest correnti con quelli registrati e' evidenza correlata, non prova di cio' che ogni processo ha caricato",
        "irrecoverable_limitations": list(CL.IRRECOVERABLE_LIMITATIONS),
    }


def collect_receipt_paths(rollouts_dir: Path = C.OUT_ROLLOUTS) -> list[Path]:
    paths = []
    for family in EXPECTED_FAMILIES:
        paths.extend(Path(p) for p in glob.glob(str(rollouts_dir / family / "*" / M.RECEIPT_FILE)))
    return sorted(paths)


def collect_execution_manifests(rollouts_dir: Path = C.OUT_ROLLOUTS) -> list[Path]:
    return sorted(Path(p) for p in glob.glob(str(rollouts_dir / "rollout_matrix_manifest_execute_*.json")))


def main() -> int:
    receipts = collect_receipt_paths()
    manifests = collect_execution_manifests()
    inventory = C.OUT_MANIFEST / "freeze_inventory.json"
    snapshot = CL.closure_snapshot(None)
    payload = build_assessment(receipts, inventory, manifests, snapshot)
    target = C.OUT_MANIFEST / ASSESSMENT_FILE
    C.write_json(target, payload)  # no-clobber: raises FileExistsError if present
    cs = payload["correlation_summary"]
    print(f"[closure-assessment] written {C.rel(target)} sha256={C.sha256_file(target)} receipts={payload['receipt_count']} manifests={len(manifests)} head_equal={cs['receipts_with_git_head_equal_current']} rollout_eval_equal={cs['receipts_with_rollout_eval_equal_current']} config_equal={cs['receipts_with_config_equal_disk']} module_equal={cs['receipts_with_module_equal_disk']} class={payload['provenance_class']} mode={payload['attestation_mode']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
