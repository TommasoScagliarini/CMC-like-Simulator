"""Self-test S0D (rev3k): split leakage, tamper pins, cell coverage, labels, fit guard."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_s0d as S  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    lin = S.verify_lineage_s0d()
    check(lin["amendment_rev3k"]["sha256"] == S.PIN_AMENDMENT_REV3K, "rev3k lineage verified (coverage, tooling, V1, July evidence, declared orders)")
    old = S.PIN_AMENDMENT_REV3K
    try:
        S.PIN_AMENDMENT_REV3K = "0"*64; expect(S.verify_lineage_s0d, S.S0DError, "tampered rev3k pin -> refused")
    finally: S.PIN_AMENDMENT_REV3K = old
    check(tuple(R.STARTS) == S.DECLARED_START_ORDER and tuple(VA.SIGMA_GRID) == S.DECLARED_SIGMA_ORDER, "declared orders match the libraries (rng contract sound)")
    # holdout-job selection: deterministic, one per cell, seeds only from the pool
    h1 = S.select_holdout_jobs(); h2 = S.select_holdout_jobs()
    check(h1 == h2 and len(h1) == 9, "holdout selection deterministic, 9 cells")
    check(all(any(f"seed{s}" in j for s in S.HOLDOUT_SEED_POOL) for j in h1.values()) and not any("seed123" in j for j in h1.values()), "held jobs only from seeds 1000-1003; det 123 never held")
    # pure dedup_and_assign: conflict, holdout priority, zero cross-split
    pj = [("A", [b"x", b"y"], [b"l1", b"l2"]), ("B", [b"x", b"z"], [b"l1", b"l3"])]
    out = S.dedup_and_assign(pj, {"B"})
    check(len(out["order"]) == 3 and out["hold_idx"].size == 2 and out["train_idx"].size == 1, "holdout-priority: shared row 'x' (A+B) -> HOLDOUT; 'y' -> train; 'z' -> holdout")
    expect(lambda: S.dedup_and_assign([("A", [b"x"], [b"l1"]), ("B", [b"x"], [b"DIFFERENT"])], set()), S.S0DError, "label conflict -> abort")
    # real split (heavy: full 39-trace revalidation)
    split = S.build_split()
    rec = split["records"]
    check(rec["train_rows"] + rec["holdout_rows"] == S.EXPECTED_UNIQUE, "train+holdout == 19314 unique")
    tr = split["assign"]["train_idx"]; ho = split["assign"]["hold_idx"]
    bt = {split["assign"]["order"][i] for i in tr}; bh = {split["assign"]["order"][i] for i in ho}
    check(len(bt & bh) == 0, "ZERO bitwise obs across the split (explicit scan)")
    check(all(len(v) == 64 for v in (rec["train_idx_sha256"], rec["holdout_idx_sha256"])), "index digests recorded")
    starts_h = split["starts"][ho]; sigmas_h = split["sigmas"][ho]
    for st in S.DECLARED_START_ORDER:
        for sg in S.DECLARED_SIGMA_ORDER:
            check(int(((starts_h == st) & (sigmas_h == sg)).sum()) > 0, f"cell {st}x{sg}: holdout present")
    check(np.all(np.isfinite(split["labels"])), "u_T labels all finite")
    mx = float(np.max(np.abs(split["labels"])))
    check(mx < 2.0, f"u_T raw-mean magnitude sane (max |u_T| = {mx:.3f}; RAW pre-clip, may exceed 1 - fact, not a gate)")
    print(f"  max |u_T| = {mx:.4f}")
    check(split["teacher"]["actor_digest"] == R.TEACHER["actor_digest"], "labels bound to the pinned V26 teacher")
    # split determinism (records only; second full build too heavy -> selection + digests already deterministic by rng)
    check(S.select_holdout_jobs() == rec["held_jobs_per_cell"], "recorded held jobs == deterministic selection")
    # Codex conformity fix: binding masks == declared held-out provenance, never first-occurrence
    gate = S.pre_gate(split)
    ho = split["assign"]["hold_idx"]; prov_h = [split["assign"]["provenance"][i] for i in ho.tolist()]
    held = split["records"]["held_jobs_per_cell"]
    cell_rows_sum = 0; multi = 0
    for st in S.DECLARED_START_ORDER:
        for sg in S.DECLARED_SIGMA_ORDER:
            job = held[f"{st}|{sg}"]
            m_ref = np.asarray([job in p2 for p2 in prov_h])
            check(gate["results"][f"cell_{st}_{sg}"]["rows"] == int(m_ref.sum()), f"cell {st}x{sg}: mask == provenance membership of {job}")
            cell_rows_sum += int(m_ref.sum())
    per_row_cells = [sum(1 for st in S.DECLARED_START_ORDER for sg in S.DECLARED_SIGMA_ORDER if held[f"{st}|{sg}"] in p2) for p2 in prov_h]
    multi = sum(1 for c in per_row_cells if c > 1)
    check(all(c >= 1 for c in per_row_cells), "every holdout row belongs to >=1 cell mask (inclusion cause covered)")
    check(cell_rows_sum == ho.size + sum(c - 1 for c in per_row_cells), "cell row-counts sum == holdout + multi-memberships (shared rows count in each causing cell)")
    check(multi == sum(1 for p2 in prov_h if len(p2 & set(held.values())) > 1), "multi-cell rows == rows shared among several held-out traces")
    # prove first-occurrence metadata is NOT what the masks use (the 9 misattributed rows differ)
    fo = 0
    for k, i in enumerate(ho.tolist()):
        st_fo, sg_fo, _ = split["starts"][i], split["sigmas"][i], None
        in_fo_cell = held[f"{st_fo}|{sg_fo}"] in prov_h[k]
        if not in_fo_cell: fo += 1
    check(fo > 0, f"{fo} holdout rows whose FIRST-OCCURRENCE cell is NOT an including held-out trace -> masks cannot come from first-occurrence metadata")
    for st in S.DECLARED_START_ORDER:
        sj = {held[f"{st}|{sg}"] for sg in S.DECLARED_SIGMA_ORDER}
        m_ref = np.asarray([bool(p2 & sj) for p2 in prov_h])
        check(gate["results"][f"start_{st}"]["rows"] == int(m_ref.sum()), f"start {st}: mask == union of the 3 held-out provenances")
    check(gate["results"]["aggregate"]["rows"] == int(ho.size), "aggregate stays the unique holdout")
    check(gate["pass"] in (True, False), "gate computed")

    # fit guard: future token NOT granted
    e = expect(lambda: S.run_fit(authorized_stage=None), S.S0DError, "fit refused without the future token")
    check("V26B-S0D-FIT" in str(e) and "NOT been granted" in str(e), "guard names the distinct future token")
    expect(lambda: S.run_fit(authorized_stage="V26B-S0D-PREGATE"), S.S0DError, "any other token refused")
    check(C.sha256_file(S.AMENDMENT_REV3K) == S.PIN_AMENDMENT_REV3K, "rev3k untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
