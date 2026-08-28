"""Self-test for rev3g (scaling preservation, July anchor, flat loss, gates; no real fit)."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r2g as G  # noqa: E402
import v26b_v2 as V2  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_common as R  # noqa: E402
import f2r_refit as RF  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def synth_state(h=8, seed=5):
    rng = np.random.default_rng(seed)
    W1 = rng.normal(0, .4, (h, 35)).astype(np.float32); W1[:, :2] = 0
    b1 = rng.normal(0, .2, h).astype(np.float32); W2 = rng.normal(0, .4, (h, h)).astype(np.float32); b2 = rng.normal(0, .2, h).astype(np.float32)
    W3 = np.concatenate([rng.normal(0, .4, (2, h)).astype(np.float32), np.zeros((2, h), np.float32)])
    b3 = np.concatenate([rng.normal(0, .2, 2).astype(np.float32), np.full(2, np.log(.005), np.float32)])
    return {"pi_encoder.0.weight": W1.copy(), "pi_encoder.0.bias": b1.copy(), "pi_encoder.2.weight": W2.copy(), "pi_encoder.2.bias": b2.copy(),
            "pi.0.0.weight": W1.copy(), "pi.0.0.bias": b1.copy(), "pi.0.2.weight": W2.copy(), "pi.0.2.bias": b2.copy(), "pi.1.weight": W3.copy(), "pi.1.bias": b3.copy()}

def main() -> int:
    lin = G.verify_lineage_r2g()
    check(lin["amendment_rev3g"]["sha256"] == G.PIN_AMENDMENT_REV3G and lin["historical_parent_verified"], "rev3g lineage incl. historical JUL_H0 parent byte-pins")
    old = G.PIN_AMENDMENT_REV3G
    try:
        G.PIN_AMENDMENT_REV3G = "0"*64; expect(G.verify_lineage_r2g, G.R2GError, "tampered rev3g pin -> refused")
    finally: G.PIN_AMENDMENT_REV3G = old
    vec, table = G.scale_vector()
    check(sorted(v["index"] for v in table.values()) == list(range(25, 35)) and vec[28] == 60.0 and vec[33] == 55.0 and vec[27] == 4.0 and vec[32] == 3.5, "scales at exact indices 25-34")
    # scaling function preservation (synthetic)
    st = synth_state()
    rng = np.random.default_rng(9); obs = rng.normal(0, .5, (64, 35)).astype(np.float64)
    sc = G.transform_state_to_scaled(st, vec)
    t1 = G.preservation_test(st, sc, obs, vec)
    check(t1 <= 1e-5, f"T1 synthetic preservation {t1:.2e} <= 1e-5")
    back = G.transform_W1_to_raw(np.asarray(sc["pi.0.0.weight"]), vec)
    check(float(np.max(np.abs(back.astype(np.float64) - st["pi.0.0.weight"].astype(np.float64)))) <= 1e-6, "W1 scale->unscale roundtrip tight")
    # real R1 init preservation on the full corpus (mandated T1)
    if str(R.BASELINE_DIR) not in sys.path: sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    import v26b_dagger_r1 as D1
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(D1.OUT_R1 / "rl_module").items()}
    import v26b_r2_offline as R2
    data, _ = R2.build_r2_dataset()
    t1r = G.preservation_test(init_raw, G.transform_state_to_scaled(init_raw, vec), np.asarray(data["obs35"]), vec)
    check(t1r <= 1e-5, f"T1 REAL R1 preservation on all {data['obs35'].shape[0]} rows: {t1r:.2e} <= 1e-5")
    print(f"  T1 real max abs = {t1r:.3e}")
    # July anchor semantics vs rev3d dimensional check
    import torch
    p = [torch.nn.Parameter(torch.ones(3, 4)), torch.nn.Parameter(torch.ones(2))]
    a0 = [torch.zeros(3, 4), torch.zeros(2)]
    july = torch.stack([(x - y).square().mean() for x, y in zip(p, a0)]).sum().item()
    check(abs(july - 2.0) < 1e-6, "July anchor = sum of per-tensor MEANS (1+1=2 for unit deltas)")
    rev3d = sum(float((x - y).square().sum()) for x, y in zip(p, a0))
    check(abs(rev3d - 14.0) < 1e-6 and rev3d / july == 7.0, "dimensional difference demonstrated (sum vs mean)")
    # flat weighted loss: no variance division; tiny fit on synthetic reduces loss; no aux params
    split = G.preregistered_split(data)
    tiny = {"epochs": 2, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}
    sc_init = G.transform_state_to_scaled(init_raw, vec)
    fit_state, rep = G.fit_r2g(sc_init, data, split, vec, budget=tiny)
    check(rep["history"][-1]["loss"] <= rep["history"][0]["loss"], "tiny fit: loss non-increasing")
    check(abs(sum(rep["lambda_per_row_by_role"][r] * rep["n_train_by_role"][r] for r in G.MASSES) - 1.0) < 1e-9, "sum lambda over TRAIN == 1 (masses exact on the fitted corpus)")
    check("weighted_flat_mse" in rep["history"][0], "flat MSE objective recorded (no variance normalisation)")
    exp_state = G.export_state_from_scaled(fit_state, vec)
    struct = RF.validate_init_state(exp_state, expected_actor_digest=None)
    check(struct["clock_columns_zero"], "export state passes the structural battery (clock zero, placeholder, 10 keys)")
    m_s = RF.numpy_mean(fit_state, (np.asarray(data["obs35"], np.float64) / vec[None, :]).astype(np.float32)[:256])
    m_e = RF.numpy_mean(exp_state, np.asarray(data["obs35"], np.float32)[:256])
    check(float(np.max(np.abs(m_s - m_e))) <= 1e-5, "T2 synthetic-run export equivalence <= 1e-5")
    # split determinism + guard
    s2 = G.preregistered_split(data)
    check(np.array_equal(split["hold"], s2["hold"]) and split["held_trajectories"] == s2["held_trajectories"], "preregistered split deterministic")
    # provenance/canonical block buildable BEFORE any fit (anti-recurrence for the AttributeError)
    blk = G.provenance_block()
    check(sorted(blk.keys()) == ["rev3a", "rev3b", "rev3c", "rev3d", "rev3e", "rev3f", "rev3g"] and all(len(v) == 64 for v in blk.values()), "provenance block builds pre-fit with all 7 amendment digests")
    import v26b_r0a as A0
    check(blk["rev3b"] == A0.PIN_AMENDMENT_REV3B and blk["rev3g"] == G.PIN_AMENDMENT_REV3G, "each constant referenced from its OWNING module")
    rec_path = G.OUT_R2G / G.RECEIPT_NAME
    if rec_path.is_file():
        rec = json.loads(rec_path.read_text())
        check(rec["amendments"] == blk, "existing canonical receipt's amendments block == builder output (provenance verified post-hoc)")
    e = expect(lambda: G.run_r2g(authorized_stage=None), G.R2GError, "no token -> refused")
    check("V26B-R2G" in str(e), "refusal names the token")
    check(C.sha256_file(G.AMENDMENT_REV3G) == G.PIN_AMENDMENT_REV3G, "rev3g amendment untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
