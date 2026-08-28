"""V26B rev3j post-run audit (ADDITIVE, restricted task): B3 correction addendum.

Nothing original is modified (primary receipt 6d31bc82…, trace 035146ab…,
simulator log, rev3j).  This module: (1) verifies/records the current hashes;
(2) writes a content-addressed addendum that corrects ONLY the B3 diagnostic
using ``reward_terms.pros_ankle_angle_imitation_target_phase`` in [0.55, 0.80]
(0 valid rows -> ``not_evaluable_no_valid_phase_or_cycle``, NOT PASS); the
global ankle min stays a DISTINCT diagnostic.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_r2i_rollout as RR  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import v26b_anchors as VA  # noqa: E402


class PostrunError(RuntimeError):
    pass


PIN_PRIMARY_RECEIPT = "6d31bc82137914540c6969540c621373206f51c336257f10852c66b47716e14b"
PIN_TRACE = "035146abead1e83456f1386241169ca85d7f5b10f917b90474e4c1984edd81ce"
PIN_SIM_LOG = "2c68728573cc2e420120ecf03d4f94f0eb01d72b0ea9165d00a224d33b6204f9"
PHASE_KEY = "pros_ankle_angle_imitation_target_phase"
B3_WINDOW = (0.55, 0.80)
ADDENDUM_NAME = "v26b_r2i_rollout_receipt_audit_addendum.json"
SIM_LOG = VA.OUT_LOGS / "r2i_nominal_det_rollout.log"


def b3_late_stance(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """B3 window diagnostic from the per-row phase field (never the global min)."""
    phases, ankle = [], []
    for i, r in enumerate(rows):
        rt = r.get("reward_terms")
        if not isinstance(rt, Mapping) or PHASE_KEY not in rt:
            raise PostrunError(f"row {i + 1}: reward_terms without {PHASE_KEY!r}")
        phases.append(float(rt[PHASE_KEY]))
        ankle.append(float(r["actor_observation_vector_before"][RR.RO.IDX_ANKLE_Q]))
    ph = np.asarray(phases); an = np.asarray(ankle)
    m = (ph >= B3_WINDOW[0]) & (ph <= B3_WINDOW[1])
    out = {
        "field": f"reward_terms.{PHASE_KEY}", "window": list(B3_WINDOW), "rows": len(rows),
        "rows_in_window": int(m.sum()), "phase_all_zero": bool(np.all(ph == 0.0)),
        "phase_min_max": [float(ph.min()), float(ph.max())],
        "ankle_min_overall_distinct_diagnostic": float(an.min()),
    }
    if m.sum() == 0:
        out["b3_window_min"] = None
        out["verdict"] = "not_evaluable_no_valid_phase_or_cycle"
    else:
        out["b3_window_min"] = float(an[m].min())
        out["verdict"] = "evaluable"
        out["meets_-0.03_in_window"] = bool(an[m].min() <= -0.03)
    return out


def verify_originals() -> dict[str, str]:
    got = {
        "primary_receipt": C.sha256_file(RR.JOB_DIR / RR.RECEIPT_NAME),
        "trace": C.sha256_file(RR.JOB_DIR / "rollout_policy_trace.json"),
        "simulator_log": C.sha256_file(SIM_LOG),
        "v26b_r2i_rollout.py": C.sha256_file(HERE / "v26b_r2i_rollout.py"),
        "test_v26b_r2i_rollout.py": C.sha256_file(HERE / "test_v26b_r2i_rollout.py"),
    }
    for key, pin in (("primary_receipt", PIN_PRIMARY_RECEIPT), ("trace", PIN_TRACE), ("simulator_log", PIN_SIM_LOG)):
        if got[key] != pin:
            raise PostrunError(f"original {key} sha {got[key]} != pinned {pin}")
    return got


def write_addendum() -> dict[str, Any]:
    hashes = verify_originals()
    rows = json.loads((RR.JOB_DIR / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    b3 = b3_late_stance(rows)
    receipt = json.loads((RR.JOB_DIR / RR.RECEIPT_NAME).read_text(encoding="utf-8"))
    superseded = receipt["ankle_negative_tract_diagnostic"]["b3_reference_diagnostic"]
    addendum = {
        "schema": "v26b_r2i_rollout_audit_addendum.1",
        "order": "architect restricted task 2026-08-24: correct ONLY the B3 diagnostic (the primary receipt conflated the GLOBAL ankle min with the late-stance window criterion); with phase == 0 on all 197 rows and 0 rows in [0.55, 0.80], B3 is not_evaluable_no_valid_phase_or_cycle, NOT PASS",
        "artefacts_immutable": {
            "primary_receipt": {"path": C.rel(RR.JOB_DIR / RR.RECEIPT_NAME), "sha256": hashes["primary_receipt"], "preserved": True},
            "trace": {"path": C.rel(RR.JOB_DIR / "rollout_policy_trace.json"), "sha256": hashes["trace"]},
            "simulator_log": {"path": C.rel(SIM_LOG), "sha256": hashes["simulator_log"]},
            "amendment_rev3j": {"sha256": RR.PIN_AMENDMENT_REV3J},
        },
        "tooling_hashes_post_code_pre_addendum": {"v26b_r2i_rollout.py": hashes["v26b_r2i_rollout.py"], "test_v26b_r2i_rollout.py": hashes["test_v26b_r2i_rollout.py"], "note": "recorded as facts, not parent pins (the rev3j amendment predates this tooling)"},
        "superseded_field_in_primary_receipt": {"path": "ankle_negative_tract_diagnostic.b3_reference_diagnostic", "value": superseded, "why_wrong": "'meets: true' used the GLOBAL ankle minimum; B3 is defined in the late-stance phase window [0.55, 0.80], which contains 0 rows here"},
        "b3_corrected": b3,
        "v3_scope_note": "this single nominal rollout is a diagnostic pre-gate homologous to R0a/R1; the full V3 gate requires 3 starts and remains NOT EVALUATED; no second start in this stage",
        "tool_sha256": C.sha256_file(Path(__file__).resolve()),
        "generated_at_utc": C.utc_now(),
    }
    path = RR.JOB_DIR / ADDENDUM_NAME
    if path.exists():
        raise PostrunError(f"no-clobber: {path} exists")
    R.write_json_exclusive(path, addendum)
    return {"path": C.rel(path), "sha256": C.sha256_file(path), "b3": b3, "hashes": hashes}


if __name__ == "__main__":
    out = write_addendum()
    print(json.dumps({"addendum": out["path"], "sha256": out["sha256"], "verdict": out["b3"]["verdict"]}, indent=2))
