"""Self-test for the rev3f R2 offline tooling (gates, weights, dataset; no fit executed here)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r2_offline as R2  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS
    assert cond, what
    CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    lin = R2.verify_lineage_r2()
    check(lin["amendment_rev3f"]["sha256"] == R2.PIN_AMENDMENT_REV3F and lin["base_npz"]["sha256"] == R2.PIN_BASE_NPZ and lin["alt_npz"]["sha256"] == R2.PIN_ALT_NPZ, "rev3f lineage verified")
    for attr in ("PIN_AMENDMENT_REV3F", "PIN_BASE_NPZ"):
        old = getattr(R2, attr)
        try:
            setattr(R2, attr, "0" * 64); expect(R2.verify_lineage_r2, R2.R2Error, f"tampered {attr} -> refused")
        finally:
            setattr(R2, attr, old)
    data, rep = R2.build_r2_dataset()
    check(rep["roles_unique_rows"] == {"base": 992, "r1_prefix": 97, "alt_minus020": 6447, "alt_plus020": 6429} and rep["rows_total_after_dedup"] == 13965, "role counts + dedup (1 reset-row collision -> prefix 97)")
    check(rep["collisions"]["count"] == 1 and rep["collisions"]["detail"][0]["labels_identical"], "single identical-label collision")
    check(rep["G1"]["pass"] and rep["G1"]["excluded_rows_99_242"] == 144 and abs(rep["G1"]["excluded_fraction"] - 144/242) < 1e-9, "G1: 98-prefix agreement, 144 excluded, fraction 0.595 < 0.80")
    w = rep["role_weights_w_r"]
    check(abs(sum(w.values()) - 1.0) < 1e-9 and abs(w["base"] - 16000/24712) < 1e-12 and abs(w["r1_prefix"] - 712/24712) < 1e-12, "July-mass weights sum to 1 and match 16000/712/4000/4000 over 24712")
    lam_total = sum(rep["per_row_lambda"][r] * rep["roles_unique_rows"][r] for r in w)
    check(abs(lam_total - 1.0) < 1e-9, "sum over dataset of per-row lambda == 1 (normalised by unique rows)")
    # G2 synthetic known-answers
    names = list(R.FEATURE_NAMES_35)
    obs = np.zeros((6, 35), dtype=np.float32); obs[:, 1] = 1.0
    d = {"obs35": obs, "actions": np.zeros((6, 2), np.float32), "role": np.asarray(["base"]*6, dtype=str)}
    g = R2.gate_g2(d)
    check(g["pass"] and g["violating_groups"] == 0, "G2 synthetic: identical labels -> pass")
    d2 = {**d, "actions": np.asarray([[0,0],[0.5,0],[0,0],[0,0],[0,0],[0,0]], np.float32)}
    g2 = R2.gate_g2(d2)
    check(g2["pass"] is False and g2["violating_groups"] == 1 and g2["worst"][0]["spread_knee_ankle"][0] == 0.5, "G2 synthetic: 0.5 knee spread in one signature -> fail")
    # G3 synthetic: separable clouds with per-cloud constant labels -> tiny rmse
    rng = np.random.default_rng(3)
    o = np.concatenate([rng.normal(0,1,(200,35)), rng.normal(30,1,(200,35))]).astype(np.float32)
    a = np.concatenate([np.full((200,2),-0.3), np.full((200,2),0.4)]).astype(np.float32)
    roles = np.asarray(["base"]*100 + ["r1_prefix"]*100 + ["alt_minus020"]*100 + ["alt_plus020"]*100, dtype=str)
    ds = {"obs35": o, "actions": a, "role": roles}
    g3 = R2.gate_g3_and_coverage(ds)
    check(g3["G3"]["pass"] and max(g3["G3"]["rmse_knee_ankle"]) < 1e-6, "G3 synthetic: separable -> ~0 rmse")
    check(all(v["rows"] == 100 for v in g3["coverage"]["per_role"].values()), "coverage synthetic: all roles present")
    bad = {**ds, "role": np.asarray(["base"]*400, dtype=str)}
    expect(lambda: R2.gate_g3_and_coverage(bad), R2.R2Error, "coverage: absent role -> refused")
    e = expect(lambda: R2.run_r2(authorized_stage=None), R2.R2Error, "no token -> refused")
    check("V26B-R2-OFFLINE" in str(e), "refusal names the token")
    check(C.sha256_file(R2.BASE_NPZ) == R2.PIN_BASE_NPZ and C.sha256_file(R2.ALT_NPZ) == R2.PIN_ALT_NPZ, "frozen datasets untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
