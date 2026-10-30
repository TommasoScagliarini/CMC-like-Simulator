"""V26B rev3e post-run audit (ADDITIVE; Codex HOLD corrections A+B).

Nothing frozen is modified: receipt 0fc762fd…, trace a73c4d8f…, report 742012fd…,
rev3e and every earlier artefact stay byte-identical.  This module provides:
* ``independent_counters(trace_path)`` — whole-trace counters recomputed straight
  from the trace JSON (never copied from the receipt);
* ``write_addendum()`` — content-addressed audit addendum next to the receipt,
  declaring the authoritative whole-trace values (correcting the report's
  'dropped_wait_hs/terminal_flushed: 0' misstatement);
* post-run-safe verification helpers used by ``test_v26b_r1_rollout_postrun.py``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np

HERE = Path(__file__).resolve().parent
VALIDATION_DIR = HERE.parent
for _entry in (str(VALIDATION_DIR / "f0_freeze_2026-08-22"), str(VALIDATION_DIR / "f1_ablation_2026-08-23"), str(VALIDATION_DIR / "f2r_bridge_2026-08-23"), str(HERE)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import v26b_r1_rollout as R1  # noqa: E402


class PostrunError(RuntimeError):
    pass


PIN_R1_ROLLOUT_RECEIPT = "0fc762fd3271b84907a0cc332f49f16461fa2c948d032eb21962c96038687611"
PIN_R1_ROLLOUT_TRACE = "a73c4d8f8e26e3e5a95fcc1f0ac42521ea1ba263b3b65583c277b7b75ea8a672"
PIN_R1_REPORT = "742012fdc42f1df1d686e0b4b009d93dc0f9750bd63bb7d61d35837a7340f7f3"
R1_REPORT = C.REPO / "reports" / "user" / "2026-08-24_v26b_r1_rollout_ended_early_diagnosis.md"
ADDENDUM_NAME = "v26b_r1_rollout_receipt_audit_addendum.json"

_DIAG_KEYS = ("morphology_causal_dropped_wait_hs_count", "morphology_causal_terminal_flushed",
              "morphology_causal_dropped_pending_count", "morphology_causal_cancelled_transition_count",
              "morphology_causal_timeout_transition_count")


def independent_counters(trace_path: Path) -> dict[str, Any]:
    """Whole-trace counters straight from the trace JSON (independent of any receipt)."""
    rows = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise PostrunError("empty trace")
    def col(key: str) -> np.ndarray:
        out = []
        for i, r in enumerate(rows):
            rt = r.get("reward_terms")
            if not isinstance(rt, Mapping) or key not in rt:
                raise PostrunError(f"row {i + 1}: reward_terms without {key!r}")
            out.append(float(rt[key]))
        return np.asarray(out)
    fc = col("morphology_causal_failed_closed")
    te = col("phase_timeout_exceeded"); side = col("phase_timeout_side")
    diags = {}
    for k in _DIAG_KEYS:
        v = col(k)
        nz = np.where(v > 0)[0] + 1
        diags[k] = {"max": float(v.max()), "last": float(v[-1]), "rows_positive": int(nz.size), "first_positive_step": int(nz[0]) if nz.size else None, "last_positive_step": int(nz[-1]) if nz.size else None}
    return {
        "rows": len(rows),
        "morphology_causal_contract_failure": {"rows_positive": int(np.sum(fc > 0)), "max": float(fc.max()), "failure": bool(np.any(fc > 0))},
        "phase_timeout_stance": int(np.sum((te > 0) & (side == 1.0))),
        "phase_timeout_swing": int(np.sum((te > 0) & (side == 2.0))),
        "diagnostics": diags,
        "provenance": "recomputed directly from rollout_policy_trace.json (whole trace); never copied from the receipt",
    }


def verify_frozen() -> dict[str, str]:
    got = {
        "receipt": C.sha256_file(R1.JOB_DIR / R1.RECEIPT_NAME),
        "trace": C.sha256_file(R1.JOB_DIR / "rollout_policy_trace.json"),
        "report": C.sha256_file(R1_REPORT),
    }
    for key, pin in (("receipt", PIN_R1_ROLLOUT_RECEIPT), ("trace", PIN_R1_ROLLOUT_TRACE), ("report", PIN_R1_REPORT)):
        if got[key] != pin:
            raise PostrunError(f"frozen {key} sha {got[key]} != pinned {pin}")
    return got


def exactly_one_rollout_evidence() -> dict[str, Any]:
    parent = R1.JOB_DIR.parent
    jobs = sorted(p.name for p in parent.iterdir() if p.is_dir())
    if jobs != [R1.JOB_DIR.name]:
        raise PostrunError(f"expected exactly one R1 rollout job dir, found {jobs}")
    receipt = json.loads((R1.JOB_DIR / R1.RECEIPT_NAME).read_text(encoding="utf-8"))
    if receipt.get("returncode") != 0 or receipt.get("status") != "ENDED_EARLY":
        raise PostrunError("receipt does not record the single ENDED_EARLY rc=0 run")
    log_path = Path(receipt["log"])
    return {"job_dirs": jobs, "receipt_status": receipt["status"], "returncode": receipt["returncode"], "log": str(log_path), "single_receipt": True}


def write_addendum() -> dict[str, Any]:
    frozen = verify_frozen()
    counters = independent_counters(R1.JOB_DIR / "rollout_policy_trace.json")
    receipt = json.loads((R1.JOB_DIR / R1.RECEIPT_NAME).read_text(encoding="utf-8"))
    rec_diag = receipt["analysis"]["counters"]["corrected_per_row"]["morphology_causal_diagnostics_last_and_max"]
    consistency = {k: {"receipt_last": rec_diag[k]["last"], "receipt_max": rec_diag[k]["max"], "independent_last": counters["diagnostics"][k]["last"], "independent_max": counters["diagnostics"][k]["max"], "match": bool(rec_diag[k]["last"] == counters["diagnostics"][k]["last"] and rec_diag[k]["max"] == counters["diagnostics"][k]["max"])} for k in _DIAG_KEYS if k in rec_diag}
    addendum = {
        "schema": "v26b_r1_rollout_audit_addendum.1",
        "order": "Codex HOLD (2026-08-24): the immutable report 742012fd... states 'dropped_wait_hs/terminal_flushed: 0'; the authoritative whole-trace values are below (the parent receipt already carried them correctly; the REPORT text was the misstatement)",
        "parents_immutable": {
            "rollout_receipt": {"path": C.rel(R1.JOB_DIR / R1.RECEIPT_NAME), "sha256": frozen["receipt"], "preserved": True},
            "rollout_trace": {"path": C.rel(R1.JOB_DIR / "rollout_policy_trace.json"), "sha256": frozen["trace"]},
            "diagnosis_report": {"path": C.rel(R1_REPORT), "sha256": frozen["report"], "misstatement": "section 2 row 'morphology causale (per-riga)': 'dropped_wait_hs/terminal_flushed: 0'"},
            "amendment_rev3e": {"sha256": R1.PIN_AMENDMENT_REV3E},
        },
        "authoritative_whole_trace_counters": counters,
        "highlight": {
            "morphology_causal_dropped_wait_hs_count": {"max": 1.0, "last": 0.0, "rows_positive": counters["diagnostics"]["morphology_causal_dropped_wait_hs_count"]["rows_positive"], "meaning": "WAIT_HS morphology samples dropped while no valid HS was ever established (consistent with valid_hs_count = 0)"},
            "morphology_causal_terminal_flushed": {"max": 1.0, "last": 1.0, "step": counters["diagnostics"]["morphology_causal_terminal_flushed"]["first_positive_step"], "meaning": "terminal flush at episode end (step 242)"},
        },
        "receipt_vs_independent_consistency": consistency,
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "generated_at_utc": C.utc_now(),
    }
    path = R1.JOB_DIR / ADDENDUM_NAME
    if path.exists():
        raise PostrunError(f"no-clobber: {path} exists")
    R.write_json_exclusive(path, addendum)
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "counters": counters}


if __name__ == "__main__":
    out = write_addendum()
    print(json.dumps({"addendum": out["path"], "sha256": out["sha256"]}, indent=2))
