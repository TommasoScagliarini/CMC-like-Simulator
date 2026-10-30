"""Self-test of f1_refit on a synthetic B-like module and a synthetic teacher (temp only, torch)."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f1_refit as RF  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402
from test_f1_sigma_variant import synthetic_state, write_module  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def expect(fn, exc, label):
    try:
        fn()
    except exc:
        check(True, label)
        return
    raise AssertionError(f"expected {exc.__name__}: {label}")


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="f1_refit_"))
    names35, _ = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    # synthetic B: hidden 16, clock columns zero, state-dependent log-std rows (like B0820)
    state = synthetic_state(35, hidden=16, seed=3)
    for k in ("pi_encoder.0.weight", "pi.0.0.weight"):
        state[k][:, :2] = 0.0
    for k in state:
        state[k] = (state[k] * np.float32(0.2)).astype(np.float32)
    info = RF.validate_state_for_refit(state, width=35, names35=names35)
    check(info["clock_columns_zero_in_init"] and info["dtype"] == "float32", "init validation")
    # synthetic dataset: obs random, teacher = a different small network's mean
    rng = np.random.default_rng(11)
    n = 600
    obs = rng.standard_normal((n, 35)).astype(np.float32)
    obs[:, 0], obs[:, 1] = 0.0, 1.0
    teacher_state = synthetic_state(35, hidden=16, seed=21)
    for k in teacher_state:
        teacher_state[k] = (teacher_state[k] * np.float32(0.3)).astype(np.float32)
    teacher = RF.numpy_forward_mean(teacher_state, obs).astype(np.float32)
    split = np.array([0] * 400 + [1] * 200, dtype=np.int64)
    data = {"obs35": obs, "teacher_mean": teacher, "split": split}
    budget = {"epochs": 30, "batch_size": 64, "lr": 3e-3, "optimizer": "Adam", "seed": 2026, "clip_weight": 1.0, "anchor_weight": 0.0}
    new_state, report = RF.fit_mean(state, data, budget=budget, names35=names35)
    check(report["epochs_run"] == 30 and report["optimizer_steps"] == 30 * 7 and report["selection"] == "fixed_final_epoch" and report["dagger_rounds"] == 0 and report["new_collection_after_fit"] is False, "fixed budget recorded")
    check(report["fit_rmse_vs_teacher"]["val"] < 0.5 * report["init_rmse_vs_teacher"]["val"], f"val RMSE halves: {report['init_rmse_vs_teacher']['val']:.4f} -> {report['fit_rmse_vs_teacher']['val']:.4f}")
    check(len(report["history"]) == 30 and report["history"][-1]["val_rmse"] < report["history"][0]["val_rmse"], "history monotone-ish")
    ok, head = SV.constant_sigma_invariants(new_state, sigma=0.005)
    check(ok and report["logstd_head"]["logstd_bias_exact"], "log-std head structurally constant after fit")
    check(np.all(new_state["pi.0.0.weight"][:, :2] == 0.0) and np.all(new_state["pi_encoder.0.weight"][:, :2] == 0.0), "clock columns zero after fit")
    check(list(new_state.keys()) == list(state.keys()) and all(new_state[k].dtype == np.float32 for k in new_state), "key order and float32 preserved")
    check(all(np.array_equal(new_state[a], new_state[b]) for a, b in RF.ALIAS_KEYS.items()), "encoder aliases bit-identical")
    check(not np.array_equal(new_state["pi.1.weight"][:2], state["pi.1.weight"][:2]), "mean rows trained")
    # determinism: same budget, same result
    again, _ = RF.fit_mean(state, data, budget=budget, names35=names35)
    check(all(np.array_equal(again[k], new_state[k]) for k in new_state), "fit is deterministic for a fixed seed")
    # init with non-zero clock columns refused
    badstate = synthetic_state(35, hidden=16, seed=4)
    expect(lambda: RF.fit_mean(badstate, data, budget=budget, names35=names35), RF.RefitError, "non-zero clock columns in init refused")
    # save / reload
    src = tmp / "B" / "rl_module"
    write_module(src, state, names35)
    full = RF.save_refit_module(src, tmp / "D", new_state, report, names35=names35, dataset_receipt={"path": "synthetic", "sha256": "0" * 64})
    out = tmp / "D" / "rl_module_refit"
    import warm_start as W

    reloaded = W.load_module_state(out)
    check(all(np.array_equal(reloaded[k], new_state[k]) for k in new_state), "saved module reloads bit-exact")
    check(full["save_reload_exact"] and full["actor_digest"] == W.actor_state_digest(reloaded) and (tmp / "D" / "f1_refit_report.json").is_file(), "report and digest")
    manifest = json.loads((out / "actor_feature_manifest.json").read_text())
    check(manifest["actor_feature_names"] == names35 and manifest["exploration_sigma"] == [0.005, 0.005], "manifest written")
    expect(lambda: RF.save_refit_module(src, tmp / "D", new_state, report, names35=names35), FileExistsError, "no-clobber")
    # protocol budget is the one the tool will use
    b = RF.default_budget()
    check(b["epochs"] == 300 and b["batch_size"] == 256 and b["lr"] == 1e-4 and b["seed"] == 2026 and b["anchor_weight"] == 0.0 and b["clip_weight"] == 1.0, "protocol budget loaded")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
