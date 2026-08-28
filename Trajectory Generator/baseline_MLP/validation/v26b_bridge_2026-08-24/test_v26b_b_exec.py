"""Tests for the executable B0/B1 stage. Synthetic fits only; no rollout, no collection."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26b_b_exec as E  # noqa: E402
import v26b_b0_masked35 as B0  # noqa: E402
import v26b_b1_base_fit as B1  # noqa: E402
import f2r_refit as RF  # noqa: E402
import f2r_common as R  # noqa: E402

CHECKS = 0
def check(c, w):
    global CHECKS; assert c, w; CHECKS += 1
def expect(fn, exc, w):
    global CHECKS
    try: fn()
    except exc as e: CHECKS += 1; return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def production_loss(means, targets, logits, source_logstd, params, anchors,
                    clip_weight, logstd_weight, anchor_weight, action_dim):
    """Literal transcription of target_domain_imitation.py lines 1136-1152."""
    import torch, torch.nn.functional as functional
    mean_loss = functional.mse_loss(means, targets)
    clip_loss = torch.relu(torch.abs(means) - 1.0).square().mean()
    logstd_loss = functional.mse_loss(logits[:, action_dim:], source_logstd)
    anchor_terms = [(p - a).square().mean() for p, a in zip(params, anchors)]
    anchor_loss = torch.stack(anchor_terms).mean()
    return mean_loss + clip_weight * clip_loss + logstd_weight * logstd_loss + anchor_weight * anchor_loss


def main() -> int:
    import torch

    # --- pins and July protocol ---------------------------------------------------------------------
    pins = E.verify_pins()
    check(set(pins) == {"amendment_b_masked35", "july_protocol_pin", "supersession_addendum"},
          "the three governing documents are pinned and verified")
    jp = json.loads(E.JULY_PIN.read_text())
    hp = jp["hyperparameters_from_the_july_cli_defaults"]
    check((E.J_EPOCHS, E.J_BATCH, E.J_LR, E.J_PATIENCE, E.J_CLIP_W, E.J_LOGSTD_W, E.J_ANCHOR_W, E.J_SEED)
          == (hp["epochs"], hp["batch_size"], hp["learning_rate"], hp["patience"],
              hp["clip_weight"], hp["logstd_weight"], hp["anchor_weight"], hp["seed"]),
          "the module constants equal the pinned July CLI defaults")
    check(jp["production_sources"]["target_domain_imitation.py"]["loss_and_step_block_sha256"]
          == "b75c3e2d7bc18255aff8a44be2b731d4a12edd7cd6dcbcfec7038761ba4acb52",
          "the July loss+step block is pinned by content")
    import hashlib
    prod = (R.BASELINE_DIR / "target_domain_imitation.py")
    lines = prod.read_text().splitlines()
    lo, hi = jp["production_sources"]["target_domain_imitation.py"]["loss_and_step_block_lines"]
    blk = "\n".join(lines[lo - 1: hi])          # the pinned 1-based inclusive range
    check(hashlib.sha256(blk.encode()).hexdigest()
          == jp["production_sources"]["target_domain_imitation.py"]["loss_and_step_block_sha256"],
          "the pinned block still matches the production source on disk")
    for attr in ("PIN_AMENDMENT", "PIN_JULY", "PIN_SUPERSESSION"):
        old = getattr(E, attr)
        try:
            setattr(E, attr, "0" * 64); expect(E.verify_pins, E.ExecError, f"tampered {attr} refused")
        finally: setattr(E, attr, old)

    # --- DIFFERENTIAL: the sandbox loss == the production transcription --------------------------------
    rng = np.random.default_rng(4)
    for _ in range(50):
        n, h = 16, 6
        means = torch.tensor(rng.normal(size=(n, 2)) * 1.4, dtype=torch.float32, requires_grad=True)
        logstd = torch.tensor(rng.normal(size=(n, 2)), dtype=torch.float32)
        logits = torch.cat([means, logstd], dim=1)
        Y = torch.tensor(rng.normal(size=(n, 2)), dtype=torch.float32)
        src = torch.tensor(rng.normal(size=(n, 2)), dtype=torch.float32)
        ps = [torch.tensor(rng.normal(size=(h, h)), dtype=torch.float32) for _ in range(3)]
        an = [torch.tensor(rng.normal(size=(h, h)), dtype=torch.float32) for _ in range(3)]
        mine = (((means - Y) ** 2).mean()
                + E.J_CLIP_W * torch.relu(torch.abs(means) - 1.0).square().mean()
                + E.J_LOGSTD_W * ((logits[:, 2:] - src) ** 2).mean()
                + E.J_ANCHOR_W * torch.stack([(p - a).square().mean() for p, a in zip(ps, an)]).mean())
        ref = production_loss(means, Y, logits, src, ps, an,
                              E.J_CLIP_W, E.J_LOGSTD_W, E.J_ANCHOR_W, 2)
        check(torch.equal(mine, ref), "the sandbox loss is bit-identical to the production transcription")
    check(True, "50 random differential cases on the July loss")
    src_e = (HERE / "v26b_b_exec.py").read_text()
    for step in ("opt.zero_grad(set_to_none=True)", "loss.backward()", "opt.step()",
                 "restore_logstd()", "project_columns()", "assert_projected()"):
        check(step in src_e, f"the per-step order includes {step}")
    # the ORDER is checked on the batch-loop body, not on the whole file: the helpers are DEFINED
    # earlier, so a naive first-occurrence search would compare definitions with call sites.
    body = src_e[src_e.index("opt.zero_grad(set_to_none=True)"): src_e.index("losses.append(")]
    seq = [ln.strip() for ln in body.splitlines() if ln.strip() in
           ("opt.zero_grad(set_to_none=True)", "loss.backward()", "opt.step()",
            "restore_logstd()", "project_columns()", "assert_projected()")]
    check(seq == ["opt.zero_grad(set_to_none=True)", "loss.backward()", "opt.step()",
                  "restore_logstd()", "project_columns()", "assert_projected()"],
          f"the batch loop follows the production order exactly, got {seq}")
    check("clip_grad" not in src_e and "clip_grad_norm" not in src_e, "no gradient clipping")
    check("PPO" not in src_e.replace("no PPO", ""), "no PPO")

    # --- median best epoch, rule fixed before the fit ----------------------------------------------------
    m = E.median_best_epoch([10, 30, 20])
    check(m["chosen_epochs"] == 20 and m["fold_best_epochs"] == [10, 20, 30], "odd count: the middle")
    check(E.median_best_epoch([10, 21])["chosen_epochs"] == 16, "even count: 15.5 rounds half UP to 16")
    check(E.median_best_epoch([10, 20])["chosen_epochs"] == 15, "even count with an integer mean")
    check(E.median_best_epoch([2, 3])["chosen_epochs"] == 3, "2.5 rounds half UP to 3, never to 2")
    check(E.median_best_epoch([4, 5])["chosen_epochs"] == 5, "4.5 rounds half UP to 5, not banker's 4")
    check(E.median_best_epoch([7, 7, 7])["chosen_epochs"] == 7, "ties are stable")
    expect(lambda: E.median_best_epoch([]), E.ExecError, "an empty list is refused")
    expect(lambda: E.median_best_epoch([0, 0, 0]), E.ExecError, "a zero epoch count is refused")
    check("round-half-up" in m["rule"] and "before any fit" in m["rule"], "the rule is recorded")

    # --- synthetic masked fit -----------------------------------------------------------------------------
    cols = list(B0.controller_columns())
    init, _ = B0.build_b0_state()
    n = 96
    obs = rng.normal(size=(n, 35)).astype(np.float32)
    obs_m = B0.apply_input_mask(obs)
    act = np.clip(rng.normal(size=(n, 2)) * 0.2, -1, 1).astype(np.float32)
    st, rep = E.fit_masked(init, obs_m, act, train_idx=np.arange(0, 64),
                           val_idx=np.arange(64, n), epochs=6, patience=60)
    B0.assert_masked_columns_zero(st); B0.assert_clock_columns_zero(st)
    check(True, "after a real masked fit the masked and clock columns are exactly zero")
    B0.assert_no_masked_update(init, st)
    check(True, "the masked columns are bit-identical to B0 after the fit")
    check(np.array_equal(np.asarray(st["pi.1.weight"])[2:], np.asarray(init["pi.1.weight"])[2:])
          and np.array_equal(np.asarray(st["pi.1.bias"])[2:], np.asarray(init["pi.1.bias"])[2:]),
          "the log-std head is byte-identical to B0")
    check(not np.array_equal(np.asarray(st["pi.0.0.weight"]), np.asarray(init["pi.0.0.weight"])),
          "the kept columns DID move, so the fit is not a no-op")
    check(rep["epochs_run"] == 6 and rep["selection_mode"] == "early_stopping"
          and rep["gradient_clipping"] == "none" and rep["ppo_updates"] == 0,
          "the fit report records the protocol")
    check(rep["hard_zero_columns"] == sorted(set(cols) | set(B0.CLOCK_COLUMNS)),
          "the hard-zero set is the controller columns plus the clock pair")
    RF.validate_init_state(st, expected_actor_digest=None, width=35)
    check(True, "the fitted state passes the frozen structural validator")
    stf, repf = E.fit_masked(init, obs_m, act, train_idx=np.arange(n), val_idx=None,
                             epochs=4, patience=None)
    check(repf["selection_mode"] == "fixed_final_epoch" and repf["best_epoch"] == 4
          and repf["best_val_mse"] is None and repf["epochs_run"] == 4,
          "fixed_final_epoch trains every row for exactly the requested epochs, with no validation")
    B0.assert_masked_columns_zero(stf)
    check(True, "the final-mode fit also leaves the masked columns at zero")

    # --- the log-std term is provably inert with a frozen constant-sigma head --------------------------------
    old = E.J_LOGSTD_W
    try:
        E.J_LOGSTD_W = 0.0
        st0, _ = E.fit_masked(init, obs_m, act, train_idx=np.arange(0, 64),
                              val_idx=np.arange(64, n), epochs=6, patience=60)
    finally:
        E.J_LOGSTD_W = old
    check(all(np.array_equal(np.asarray(st[k]), np.asarray(st0[k])) for k in st),
          "with a frozen constant-sigma head, log-std weight 0.1 and 0.0 give BIT-IDENTICAL results")

    # --- functional equivalence survives the fit -------------------------------------------------------------
    eq = B0.functional_equivalence_25(st, obs)
    check(eq["bit_identical"], "the fitted actor is still bit-identical to its 25-column subnetwork")
    tr = B0.bit_exact_unmask_transition(st, obs)
    check(tr["bit_identical"], "the unmask transition is still bit-exact after the fit")

    # --- gates ------------------------------------------------------------------------------------------------
    check((E.FOLD_HELDOUT_RMSE_MAX, E.LOTO_WEIGHTED_MEAN_MAX, E.FINAL_RMSE_MAX,
           E.FINAL_PER_JOINT_RMSE_MAX, E.FINAL_PER_JOINT_MAXABS_MAX) == (0.05, 0.03, 0.02, 0.03, 0.15),
          "the preregistered offline thresholds are 0.05 / 0.03 / 0.02 / 0.03 / 0.15")
    mm = E.metrics(st, obs_m, act, np.arange(10))
    check(set(mm["per_joint"]) == {"knee", "ankle"} and "max_abs" in mm["per_joint"]["knee"]
          and mm["rows"] == 10, "metrics report aggregate, per-joint RMSE and per-joint max_abs")

    # --- tokens, no-clobber, prohibitions ------------------------------------------------------------------------
    for bad in (None, "V26B-B1-BASE-FIT-MASKED35", "v26b-b0-masked35-transplant"):
        expect(lambda b=bad: E.materialize_b0(authorized_stage=b), E.ExecError, f"B0 token {bad!r} refused")
    for bad in (None, "V26B-B0-MASKED35-TRANSPLANT", "wrong"):
        expect(lambda b=bad: E.run_b1(authorized_stage=b), E.ExecError, f"B1 token {bad!r} refused")
    check(not any(t in src_e for t in ("subprocess", "rollout_eval", "collect_teacher_dataset",
                                       "train_ppo", "PPOConfig")),
          "the module cannot roll out, collect or run PPO")
    check("os.system" not in src_e and "os.sep" not in src_e and "from pathlib import Path" in src_e,
          "pathlib only, no shell, no os-specific path handling")
    check(all(m not in sys.modules for m in B0.SUPERSEDED_MODULES),
          "no superseded 25D module is loaded")
    check("critic" in src_e and "never loaded or modified" in src_e, "the critic is declared untouched")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
