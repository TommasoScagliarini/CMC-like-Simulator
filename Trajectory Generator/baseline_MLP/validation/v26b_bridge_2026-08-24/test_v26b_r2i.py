"""Self-test rev3i: exact-anchor value+gradient vs full-tensor reference, a' loss exactness."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_r2i as I  # noqa: E402
import v26b_r2g as G  # noqa: E402
import v26b_r2_offline as R2  # noqa: E402
import f0_common as C  # noqa: E402

CHECKS = 0
def check(cond, what):
    global CHECKS; assert cond, what; CHECKS += 1
def expect(fn, exc, what):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {what}")

def main() -> int:
    import torch
    # --- lineage (+tamper) ---------------------------------------------------------------
    lin = I.verify_lineage_r2i()
    check(lin["amendment_rev3i"]["sha256"] == I.PIN_AMENDMENT_REV3I and lin["rev3h_supplement"]["sha256"] == I.PIN_REV3H_SUPPLEMENT, "rev3i lineage incl. rev3h memo+supplement")
    check("AUDIT-REJECTED" in lin["r2g_rejected_preserved"]["status"], "R2G rejected-evidence verified on disk, status recorded")
    old = I.PIN_AMENDMENT_REV3I
    try:
        I.PIN_AMENDMENT_REV3I = "0"*64; expect(I.verify_lineage_r2i, I.R2IError, "tampered rev3i pin -> refused")
    finally: I.PIN_AMENDMENT_REV3I = old
    blk = I.provenance_block_r2i()
    check(sorted(blk.keys()) == sorted(["rev3a","rev3b","rev3c","rev3d","rev3e","rev3f","rev3g","rev3h_memo","rev3h_supplement","rev3i"]), "provenance block (10 digests) builds pre-fit")

    # --- EXACT historical anchor: value + gradient vs full zero-padded reference ----------
    torch.manual_seed(0)
    h = 16
    mp = [torch.nn.Parameter(torch.randn(h, 35)), torch.nn.Parameter(torch.randn(h)), torch.nn.Parameter(torch.randn(h, h)), torch.nn.Parameter(torch.randn(h)), torch.nn.Parameter(torch.randn(2, h)), torch.nn.Parameter(torch.randn(2))]
    an = [torch.randn_like(p) for p in mp]
    mine = I.ANCHOR_WEIGHT * I.anchor_loss_july_exact(mp, an)
    # reference: FULL [4,h]/[4] tensors, logstd rows AT the anchor (delta zero) -> July code shape
    W3_full = torch.cat([mp[4], an[4].new_zeros(2, h) + torch.zeros(2, h)])  # placeholder replaced below
    A3_full_w = torch.cat([an[4], torch.zeros(2, h)])
    W3_full = torch.cat([mp[4], torch.zeros(2, h)])
    b3_full = torch.cat([mp[5], torch.zeros(2)])
    a3_full = torch.cat([an[5], torch.zeros(2)])
    terms = [(mp[0]-an[0]).square().mean(), (mp[1]-an[1]).square().mean(), (mp[2]-an[2]).square().mean(), (mp[3]-an[3]).square().mean(), (W3_full-A3_full_w).square().mean(), (b3_full-a3_full).square().mean()]
    ref = I.ANCHOR_WEIGHT * torch.stack(terms).mean()
    check(abs(float(mine) - float(ref)) < 1e-10, f"anchor VALUE == full zero-padded reference ({float(mine):.6e} vs {float(ref):.6e})")
    g_mine = torch.autograd.grad(mine, mp, retain_graph=False, allow_unused=False)
    mine2 = I.ANCHOR_WEIGHT * I.anchor_loss_july_exact(mp, an)  # rebuild graphs
    ref2 = I.ANCHOR_WEIGHT * torch.stack([(mp[0]-an[0]).square().mean(), (mp[1]-an[1]).square().mean(), (mp[2]-an[2]).square().mean(), (mp[3]-an[3]).square().mean(), (torch.cat([mp[4], torch.zeros(2, h)])-A3_full_w).square().mean(), (torch.cat([mp[5], torch.zeros(2)])-a3_full).square().mean()]).mean()
    g_ref = torch.autograd.grad(ref2, mp)
    check(all(float((a-b).abs().max()) < 1e-10 for a, b in zip(torch.autograd.grad(mine2, mp), g_ref)), "anchor GRADIENT == full zero-padded reference on every mean parameter")
    # dimensional sanity: uniform unit deltas -> mean of [1,1,1,1,0.5,0.5] = 5/6
    unit = [torch.nn.Parameter(torch.ones_like(p)) for p in mp]
    zero = [torch.zeros_like(p) for p in mp]
    val = float(I.anchor_loss_july_exact(unit, zero))
    check(abs(val - 5.0/6.0) < 1e-6, "unit deltas -> mean([1,1,1,1,.5,.5]) = 5/6 (July semantics, not sum)")

    # --- a' loss exactness -----------------------------------------------------------------
    data, _ = R2.build_r2_dataset()
    split = G.preregistered_split(data)
    train = split["train"]; roles = np.asarray(data["role"])[train]
    n = int(train.sum())
    check(I.K_CONST == int(np.ceil(n / 256)) == 42, "K = ceil(10727/256) = 42")
    n_tr = {r: int(np.sum(roles == r)) for r in G.MASSES}
    lam = np.empty(n)
    for r, w in G.MASSES.items(): lam[roles == r] = w / n_tr[r]
    rng = np.random.default_rng(11); e = rng.uniform(0, 1, n)
    L_global = float(np.sum(lam * e))
    for pseed in (1, 2):  # two different partitions
        perm = np.random.default_rng(pseed).permutation(n)
        acc = sum(float(I.K_CONST * np.sum(lam[perm[s:s+256]] * e[perm[s:s+256]])) for s in range(0, n, 256))
        check(abs(acc - I.K_CONST * L_global) < 1e-9, f"partition {pseed}: epoch-sum of batch terms == K * global objective")
    i0 = 1234; delta = 1e-3
    for pseed in (1, 2):
        perm = np.random.default_rng(pseed).permutation(n)
        e2 = e.copy(); e2[i0] += delta
        acc1 = sum(float(I.K_CONST * np.sum(lam[perm[s:s+256]] * e[perm[s:s+256]])) for s in range(0, n, 256))
        acc2 = sum(float(I.K_CONST * np.sum(lam[perm[s:s+256]] * e2[perm[s:s+256]])) for s in range(0, n, 256))
        check(abs((acc2 - acc1) - I.K_CONST * lam[i0] * delta) < 1e-9, f"partition {pseed}: row coefficient == K*lambda_i, composition-independent")
    check("NOT equivalent to a full-batch" in I.ADAM_DECLARATION, "Adam declaration present for the receipt")

    # --- tiny fit sanity -------------------------------------------------------------------
    if str(C.REPO / "Trajectory Generator/baseline_MLP") not in sys.path:
        sys.path.insert(0, str(C.REPO / "Trajectory Generator/baseline_MLP"))
    import warm_start as W
    import v26b_dagger_r1 as D1
    init_raw = {k: np.asarray(W._as_numpy(v)) for k, v in W.load_module_state(D1.OUT_R1 / "rl_module").items()}
    vec, _ = G.scale_vector()
    init_scaled = G.transform_state_to_scaled(init_raw, vec)
    tiny = {"epochs": 2, "batch_size": 256, "lr": 1e-4, "seed": 2026, "clip_weight": 1.0}
    st, rep = I.fit_r2i(init_scaled, data, split, vec, budget=tiny)
    check(rep["history"][-1]["loss"] <= rep["history"][0]["loss"], "tiny fit: loss non-increasing")
    check(rep["K"] == 42 and rep["anchor_weight"] == 0.01 and "NOT equivalent" in rep["adam_declaration"], "fit report carries K, anchor weight, Adam declaration")
    exp_state = G.export_state_from_scaled(st, vec)
    import f2r_refit as RF
    check(RF.validate_init_state(exp_state, expected_actor_digest=None)["clock_columns_zero"], "export passes structural battery")

    # --- guard -----------------------------------------------------------------------------
    e2 = expect(lambda: I.run_r2i(authorized_stage=None), I.R2IError, "no token -> refused")
    check("V26B-R2I" in str(e2), "refusal names the token")
    check(C.sha256_file(I.AMENDMENT_REV3I) == I.PIN_AMENDMENT_REV3I, "rev3i untouched")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
