"""V26B rev3k — S0D pure-distillation stage: split builder + read-only pre-gate (S0D-1).

This invocation builds/verifies the leakage-free per-job split and runs the kNN
pre-gate ONLY.  The future fit is fail-closed behind the DISTINCT token
``V26B-S0D-FIT`` (not granted); export/rollout are separate future stages.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_anchors as VA  # noqa: E402
import v26b_v2 as V2  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402


class S0DError(RuntimeError):
    pass


AMENDMENT_REV3K = HERE / "v26b_amendment_rev3k_s0d_pure_distillation.json"
PIN_AMENDMENT_REV3K = "8f34c95ee7c69029e0d3e8715f3e62fe9652533a03e0dd4b8fb7b38114377437"
FUTURE_FIT_TOKEN = "V26B-S0D-FIT"  # NOT granted; run_fit always refuses in this stage
PRE_GATE_RMSE_MAX = 0.15           # reused G3-blocked preregistered threshold (rev3g)
DECLARED_START_ORDER = ("minus020", "nominal", "plus020")
DECLARED_SIGMA_ORDER = (0.0025, 0.005, 0.01)
HOLDOUT_SEED_POOL = (1000, 1001, 1002, 1003)
EXPECTED_UNIQUE = 19314


def verify_lineage_s0d() -> dict[str, Any]:
    lin: dict[str, Any] = {}
    got = C.sha256_file(AMENDMENT_REV3K)
    if got != PIN_AMENDMENT_REV3K:
        raise S0DError(f"rev3k amendment sha {got} != pinned")
    lin["amendment_rev3k"] = {"path": C.rel(AMENDMENT_REV3K), "sha256": got}
    am = json.loads(AMENDMENT_REV3K.read_text(encoding="utf-8"))
    cov = am["parents_immutable"]["anchor_coverage_pass"]["coverage_json_sha256"]
    import v26b_student as VS
    if C.sha256_file(VS.COVERAGE_JSON) != cov:
        raise S0DError("coverage JSON digest != rev3k pin")
    for label, pin in am["parents_immutable"]["collection_tooling"].items():
        fname = label.split(" ")[0]
        if C.sha256_file(HERE / fname) != pin:
            raise S0DError(f"{fname} changed after the rev3k amendment")
    v1r = am["parents_immutable"]["init_v1"]
    if C.sha256_file(VA.OUT_ROOT / "student/V1_35D_transplant/v26b_v1_receipt.json") != v1r["receipt_sha256"]:
        raise S0DError("V1 receipt digest != rev3k pin")
    ja = am["parents_immutable"]["july_anchor_evidence"]
    if C.sha256_file(R.BASELINE_DIR / ja["file"]) != ja["sha256_current"]:
        raise S0DError("July anchor evidence file changed")
    if tuple(R.STARTS) != DECLARED_START_ORDER or tuple(VA.SIGMA_GRID) != DECLARED_SIGMA_ORDER:
        raise S0DError(f"declared orders differ from the library constants: {R.STARTS} / {VA.SIGMA_GRID}")
    lin["coverage_sha256"] = cov
    lin["init_v1"] = v1r
    return lin


def select_holdout_jobs() -> dict[str, str]:
    """Deterministic rng-2026 selection: one held-out seed per (start, sigma) cell, declared order."""
    rng = np.random.default_rng(2026)
    held: dict[str, str] = {}
    for start in DECLARED_START_ORDER:
        for sigma in DECLARED_SIGMA_ORDER:
            perm = rng.permutation(len(HOLDOUT_SEED_POOL))
            seed = HOLDOUT_SEED_POOL[int(perm[0])]
            held[f"{start}|{sigma}"] = f"A26_{VA.sigma_tag(sigma)}__{start}__seed{seed}"
    return held


def dedup_and_assign(per_job: Sequence[tuple[str, list[bytes], list[bytes]]], holdout_jobs: set[str]) -> dict[str, Any]:
    """PURE function: bitwise dedup with FULL provenance; holdout-priority assignment.

    ``per_job``: (job_id, [obs_bytes...], [label_bytes...]).  A unique obs whose provenance
    touches ANY holdout job goes HOLDOUT ONLY; label conflicts abort."""
    first: dict[bytes, int] = {}
    labels: list[bytes] = []
    prov: list[set] = []
    order: list[bytes] = []
    for job_id, obs_list, lab_list in per_job:
        if len(obs_list) != len(lab_list):
            raise S0DError(f"{job_id}: obs/label length mismatch")
        for ob, lb in zip(obs_list, lab_list):
            j = first.get(ob)
            if j is None:
                first[ob] = len(order); order.append(ob); labels.append(lb); prov.append({job_id})
            else:
                if labels[j] != lb:
                    raise S0DError(f"label CONFLICT on identical obs (jobs {sorted(prov[j])} vs {job_id}): abort")
                prov[j].add(job_id)
    hold_idx, train_idx = [], []
    for i, p in enumerate(prov):
        (hold_idx if (p & holdout_jobs) else train_idx).append(i)
    ht = {order[i] for i in hold_idx} & {order[i] for i in train_idx}
    if ht:
        raise S0DError("bitwise obs present in BOTH splits (impossible by construction)")
    return {"order": order, "labels": labels, "provenance": prov, "hold_idx": np.asarray(hold_idx, dtype=np.int64), "train_idx": np.asarray(train_idx, dtype=np.int64)}


def build_split() -> dict[str, Any]:
    """Load the 39 traces with full validation, dedup with provenance, assign the split."""
    ctx = VA.teacher_context()
    per_job: list[tuple[str, list[bytes], list[bytes]]] = []
    meta: dict[bytes, tuple[str, float, int]] = {}  # obs -> (start, sigma, seed) of first occurrence (for cell metrics)
    arrays: dict[bytes, tuple[np.ndarray, np.ndarray]] = {}
    for spec in VA.job_specs():
        VA.validate_receipt(Path(spec["dir"]), spec)
        rows, rep = VA._trace_rows(Path(spec["dir"]), ctx=ctx, expected_seed=int(spec["seed"]), expected_start=str(spec["start"]), expected_selection="stochastic")
        if not rep["complete"]:
            raise S0DError(f"{spec['job_id']}: incomplete trace (collection was coverage-PASS; abort)")
        obs = rows["obs35"]; tg = rows["targets"]
        ob_list = [obs[i].tobytes() for i in range(obs.shape[0])]
        lb_list = [tg[i].tobytes() for i in range(tg.shape[0])]
        per_job.append((spec["job_id"], ob_list, lb_list))
        for i, ob in enumerate(ob_list):
            meta.setdefault(ob, (str(spec["start"]), float(spec["sigma"]), int(spec["seed"])))
            arrays.setdefault(ob, (obs[i], tg[i]))
    pins_ok = R.verify_anchor_pins()
    if not pins_ok.get("all_match"):
        raise S0DError("det anchor pins mismatch")
    for start, aspec in R.ANCHORS.items():
        rows, rep = VA._trace_rows(Path(aspec["job_dir"]), ctx=ctx, expected_seed=R.DET_SEED, expected_start=start, expected_selection="deterministic", pins=aspec)
        obs = rows["obs35"]; tg = rows["targets"]
        ob_list = [obs[i].tobytes() for i in range(obs.shape[0])]
        lb_list = [tg[i].tobytes() for i in range(tg.shape[0])]
        per_job.append((f"anchor_det_{start}", ob_list, lb_list))
        for i, ob in enumerate(ob_list):
            meta.setdefault(ob, (start, 0.0, R.DET_SEED))
            arrays.setdefault(ob, (obs[i], tg[i]))
    held = select_holdout_jobs()
    assign = dedup_and_assign(per_job, set(held.values()))
    n = len(assign["order"])
    if n != EXPECTED_UNIQUE:
        raise S0DError(f"unique rows {n} != expected {EXPECTED_UNIQUE}")
    obs_mat = np.stack([arrays[ob][0] for ob in assign["order"]])
    lab_mat = np.stack([arrays[ob][1] for ob in assign["order"]]).astype(np.float64)
    starts = np.asarray([meta[ob][0] for ob in assign["order"]])
    sigmas = np.asarray([meta[ob][1] for ob in assign["order"]])
    # NOTE: (start, sigma) of a row = its FIRST-occurrence job; duplicate provenance recorded separately
    split_records = {
        "held_jobs_per_cell": held,
        "train_rows": int(assign["train_idx"].size), "holdout_rows": int(assign["hold_idx"].size),
        "train_idx_sha256": DS.sha256_array(np.sort(assign["train_idx"])), "holdout_idx_sha256": DS.sha256_array(np.sort(assign["hold_idx"])),
        "rows_with_multi_provenance": int(sum(1 for p in assign["provenance"] if len(p) > 1)),
    }
    return {"obs": obs_mat, "labels": lab_mat, "starts": starts, "sigmas": sigmas, "assign": assign, "records": split_records, "teacher": {"actor_digest": ctx["digest"], "module_state_sha256": ctx["module_state_sha256"]}}


def pre_gate(split: Mapping[str, Any]) -> dict[str, Any]:
    obs = split["obs"].astype(np.float64); u = split["labels"]
    tr = split["assign"]["train_idx"]; ho = split["assign"]["hold_idx"]
    mu = obs[tr].mean(0); sd = obs[tr].std(0); keep = sd > 1e-9
    excluded = [int(i) for i in np.where(~keep)[0]]
    if excluded != [0, 1]:
        raise S0DError(f"excluded features {excluded} != the dead privileged clock [0, 1]")
    z = ((obs - mu) / np.where(keep, sd, 1.0))[:, keep]
    zt = z[tr]; yt = u[tr]; b2 = np.sum(zt ** 2, 1)
    preds = np.empty((ho.size, 2))
    zh = z[ho]
    for s in range(0, ho.size, 256):
        a = zh[s: s + 256]
        d2 = np.sum(a ** 2, 1)[:, None] + b2[None, :] - 2.0 * a @ zt.T
        idx = np.argpartition(d2, 4, 1)[:, :5]
        preds[s: s + 256] = yt[idx].mean(1)
    yh = u[ho]
    def rmse(mask):
        return np.sqrt(np.mean((preds[mask] - yh[mask]) ** 2, axis=0)).tolist()
    # Codex conformity fix (2026-08-24): binding start/cell metrics use PROVENANCE membership in the
    # declared held-out job(s), NEVER first-occurrence metadata; a row shared among several held-out
    # traces legitimately contributes to several cell metrics.  Aggregate = unique holdout rows.
    prov_h = [split["assign"]["provenance"][i] for i in ho.tolist()]
    held = split["records"]["held_jobs_per_cell"]
    out = {"aggregate": {"rows": int(ho.size), "rmse": rmse(np.ones(ho.size, bool))}}
    for st in DECLARED_START_ORDER:
        start_jobs = {held[f"{st}|{sg}"] for sg in DECLARED_SIGMA_ORDER}
        m = np.asarray([bool(p & start_jobs) for p in prov_h])
        out[f"start_{st}"] = {"rows": int(m.sum()), "rmse": rmse(m), "mask_rule": "provenance intersects the union of this start's 3 held-out jobs"}
    for st in DECLARED_START_ORDER:
        for sg in DECLARED_SIGMA_ORDER:
            job = held[f"{st}|{sg}"]
            m = np.asarray([job in p for p in prov_h])
            if m.sum() == 0:
                raise S0DError(f"cell {st}x{sg}: no holdout rows")
            out[f"cell_{st}_{sg}"] = {"rows": int(m.sum()), "rmse": rmse(m), "mask_rule": f"provenance contains {job}"}
    binding = {k: v for k, v in out.items()}
    all_pass = all(all(x <= PRE_GATE_RMSE_MAX for x in v["rmse"]) for v in binding.values())
    # diagnostics (no new thresholds): sigma-scale comparison + causal-signature ambiguity on u_T
    names = list(R.FEATURE_NAMES_35)
    idx_disc = [i for i, nn in enumerate(names) if nn.endswith(("_in_contact", "_heel_strike", "_toe_off", "_saturated")) or nn.startswith(("phase_fsm_", "phase_expected_"))]
    i_el = [names.index("phase_stance_elapsed_norm"), names.index("phase_swing_elapsed_norm")]
    sig = [tuple(split["obs"][i, idx_disc].astype(int)) + tuple(np.round(split["obs"][i, i_el] / 0.05).astype(int)) for i in range(obs.shape[0])]
    from collections import defaultdict
    groups = defaultdict(list)
    for i, s in enumerate(sig):
        groups[s].append(i)
    amb = sum(1 for s, ix in groups.items() if len(ix) > 1 and np.any(u[ix].max(0) - u[ix].min(0) > 0.2))
    return {"threshold_per_joint": PRE_GATE_RMSE_MAX, "threshold_source": "REUSED G3-blocked preregistered (rev3g)", "results": out, "pass": bool(all_pass),
            "diagnostics_no_thresholds": {"sigma_robustness_scale_reference": [0.0025, 0.005, 0.01], "aggregate_rmse_vs_sigma_scale": {"rmse": out["aggregate"]["rmse"], "note": "diagnostic comparison only"}, "ambiguity_groups_uT_spread_gt_0.2": int(amb), "signature": "8 discrete + elapsed@0.05 (rev3f/G2 lineage), diagnostic only"}}


def run_fit(*, authorized_stage: str | None = None, **_):
    raise S0DError(f"the S0D fit requires --authorized-stage {FUTURE_FIT_TOKEN}, which has NOT been granted (rev3k authorises S0D-0/S0D-1 only); got {authorized_stage!r}")


def run_pregate() -> dict[str, Any]:
    lineage = verify_lineage_s0d()
    split = build_split()
    gate = pre_gate(split)
    receipt = {
        "schema": "v26b_s0d_pregate.1",
        "amendment_rev3k": PIN_AMENDMENT_REV3K,
        "lineage": lineage,
        "teacher": split["teacher"],
        "split": split["records"],
        "pre_gate": gate,
        "scope": "S0D-1 read-only pre-gate ONLY: no fit (token V26B-S0D-FIT not granted), no export, no rollout",
        "code_digests": {"v26b_s0d.py": C.sha256_file(Path(__file__).resolve()), "test_v26b_s0d.py": C.sha256_file(HERE / "test_v26b_s0d.py") if (HERE / "test_v26b_s0d.py").is_file() else None},
        "generated_at_utc": C.utc_now(), "git": C.git_snapshot(),
    }
    path = R.unique_artifact_path(VA.OUT_ROOT, f"v26b_s0d_pregate_receipt_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "pass": gate["pass"], "gate": gate, "split": split["records"]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3k S0D-1 pre-gate (read-only; fit locked)")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    if not args.execute:
        lineage = verify_lineage_s0d()
        print(json.dumps({"mode": "dry", "lineage_ok": True, "held_jobs": select_holdout_jobs()}, indent=2))
        return 0
    out = run_pregate()
    print(json.dumps({"pregate_pass": out["pass"], "receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"], "aggregate_rmse": out["gate"]["results"]["aggregate"]["rmse"], "train_holdout": [out["split"]["train_rows"], out["split"]["holdout_rows"]]}, indent=2))
    return 0 if out["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
