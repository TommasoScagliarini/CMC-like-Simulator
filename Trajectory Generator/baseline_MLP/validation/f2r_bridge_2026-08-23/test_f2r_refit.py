"""Self-test of f2r_refit (synthetic fits in temp dirs; read-only checks on the real pinned actors)."""

from __future__ import annotations

import json
import pickle
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f2r_refit as RF  # noqa: E402

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


def synthetic_init(hidden: int = 16, seed: int = 3) -> OrderedDict:
    rng = np.random.default_rng(seed)
    w1 = (rng.standard_normal((hidden, 35)) * 0.2).astype(np.float32); w1[:, list(R.CLOCK_COLUMNS)] = 0.0
    b1 = (rng.standard_normal(hidden) * 0.1).astype(np.float32)
    w2 = (rng.standard_normal((hidden, hidden)) * 0.2).astype(np.float32); b2 = (rng.standard_normal(hidden) * 0.1).astype(np.float32)
    w3 = (rng.standard_normal((4, hidden)) * 0.2).astype(np.float32); w3[2:] = 0.0
    b3 = (rng.standard_normal(4) * 0.1).astype(np.float32); b3[2:] = np.float32(np.log(0.005))
    return OrderedDict([("pi_encoder.0.weight", w1), ("pi_encoder.0.bias", b1), ("pi_encoder.2.weight", w2), ("pi_encoder.2.bias", b2), ("pi.0.0.weight", w1.copy()), ("pi.0.0.bias", b1.copy()), ("pi.0.2.weight", w2.copy()), ("pi.0.2.bias", b2.copy()), ("pi.1.weight", w3), ("pi.1.bias", b3)])


def write_module(dir_: Path, state: OrderedDict) -> None:
    dir_.mkdir(parents=True)
    with (dir_ / "module_state.pkl").open("wb") as h:
        pickle.dump(state, h, protocol=pickle.HIGHEST_PROTOCOL)
    with (dir_ / "class_and_ctor_args.pkl").open("wb") as h:
        pickle.dump({"class": "synthetic", "ctor_args_and_kwargs": ((), {})}, h)
    (dir_ / "metadata.json").write_text(json.dumps({"checkpoint_version": "2.1"}), encoding="utf-8")


def main() -> int:
    tmp = R.portable_tempdir("f2r_refit_")
    names35 = list(R.FEATURE_NAMES_35)
    # --- real read-only: pinned init validates; negative control is refused as init
    init_state, info = RF.load_init_state()
    check(info["actor_digest"] == R.INIT_PRIMARY["actor_digest"] and info["module_state_sha256"] == R.INIT_PRIMARY["module_state_sha256"] and info["hidden"] == 256, "pinned JUL_H0 init validates (digest, sha, hidden 256)")
    check(info["clock_columns_zero"] and info["sigma_head"]["logstd_bias_exact"], "JUL_H0: clock columns zero, constant sigma head")
    expect(lambda: RF.load_init_state(R.NEGATIVE_CONTROL["module"], expected_actor_digest=R.NEGATIVE_CONTROL["actor_digest"]), RF.RefitError, "B0820_H0 refused as init (state-dependent log-std head)")
    expect(lambda: RF.load_init_state(expected_actor_digest="0" * 64), RF.RefitError, "digest mismatch refused")
    # --- synthetic fit
    init = synthetic_init()
    rng = np.random.default_rng(11); n = 400
    obs = rng.standard_normal((n, 35)).astype(np.float32); obs[:, 0], obs[:, 1] = 0.0, 1.0
    teacher = synthetic_init(seed=21)
    labels = RF.numpy_mean(teacher, obs).astype(np.float32)
    phi = rng.uniform(0, 1, n); clock = np.stack([np.sin(2 * np.pi * phi), np.cos(2 * np.pi * phi)], 1).astype(np.float32)
    seeds = rng.choice([123, 124], size=n)
    data = {"obs35": obs, "actions": labels, "clock": clock, "seed": seeds, "purpose": np.array(["anchor"] * 100 + ["det"] * 100 + ["stoch"] * 200)}
    budget = {"epochs": 25, "batch_size": 64, "lr": 3e-3, "optimizer": "Adam", "seed": 2026, "clip_weight": 1.0, "lambda_phi": 0.1, "lambda_anchor": 1e-3, "selection": "closed_loop_only"}
    new_state, rep = RF.fit_student(init, data, budget=budget)
    check(tuple(new_state.keys()) == RF.EXPECTED_KEY_ORDER and all(np.asarray(v).dtype == np.float32 for v in new_state.values()), "exported state: exactly the 10 pi* keys, float32, original order")
    check(rep["rmse_vs_labels"]["fit"] < 0.6 * rep["rmse_vs_labels"]["init"], f"rmse vs labels decreases: {rep['rmse_vs_labels']['init']:.4f} -> {rep['rmse_vs_labels']['fit']:.4f}")
    check(rep["aux_head"]["exported"] is False and rep["aux_head"]["training_time_only"] and "aux" not in " ".join(new_state.keys()), "auxiliary head is training-time only and not exported")
    check(rep["history"][-1]["aux_phase_err_median"] < rep["history"][0]["aux_phase_err_median"], "auxiliary phase head learns (privileged label in the loss only)")
    check(np.all(new_state["pi.0.0.weight"][:, list(R.CLOCK_COLUMNS)] == 0.0) and np.all(new_state["pi_encoder.0.weight"][:, list(R.CLOCK_COLUMNS)] == 0.0), "clock columns zero after fit")
    check(RF.validate_init_state(new_state, expected_actor_digest=None)["sigma_head"]["logstd_bias_exact"], "constant sigma head after fit")
    check(rep["validation_split_in_fit"] is False and rep["selection"] == "closed_loop_only" and rep["seeds"] == [123, 124], "no validation split in the fit; seeds recorded")
    again, _ = RF.fit_student(init, data, budget=budget)
    check(all(np.array_equal(again[k], new_state[k]) for k in new_state), "fit deterministic")
    inv = RF.invariance_test(new_state, obs)
    check(inv["bit_identical"] and inv["max_abs_diff"] == 0.0, "mean invariant (bit-identical) to perturbation of the clock inputs")
    # anchor toward a previous round
    new2, rep2 = RF.fit_student(init, data, budget={**budget, "lambda_anchor": 1.0}, anchor_state=new_state)
    check(rep2["anchor_actor_digest"] == rep["new_actor_digest"], "anchor = previous round state")
    # --- structural refusals
    bad = {**data, "seed": np.where(np.arange(n) == 5, 125, seeds)}
    expect(lambda: RF.fit_student(init, bad, budget=budget), R.F2RContractError, "seed 125 row in the dataset refused")
    bad2 = {**data, "seed": np.where(np.arange(n) == 7, 127, seeds)}
    expect(lambda: RF.fit_student(init, bad2, budget=budget), R.F2RContractError, "sealed seed row refused")
    bad3 = {**data, "purpose": np.array(["validation"] * n)}
    expect(lambda: RF.fit_student(init, bad3, budget=budget), R.F2RContractError, "invalid purpose refused")
    init_bad = synthetic_init(seed=4); init_bad["pi.0.0.weight"][:, 0] = 0.1; init_bad["pi_encoder.0.weight"][:, 0] = 0.1
    expect(lambda: RF.fit_student(init_bad, data, budget=budget), RF.RefitError, "init with non-zero clock columns refused")
    init_bad2 = synthetic_init(seed=5); init_bad2["pi.1.weight"][2, 0] = 0.3
    expect(lambda: RF.fit_student(init_bad2, data, budget=budget), RF.RefitError, "init without constant sigma head refused")
    expect(lambda: RF.fit_student(init, {**data, "obs35": obs[:, :34]}, budget=budget), R.F2RContractError, "obs width 34 refused")
    # --- export round trip
    src = tmp / "init" / "rl_module"; write_module(src, init)
    full = RF.export_student(src, tmp / "round1", new_state, rep, names35=names35)
    out = tmp / "round1" / "rl_module_student"
    check((out / "module_state.pkl").is_file() and (out / "actor_feature_manifest.json").is_file() and (tmp / "round1" / "f2r_refit_report.json").is_file(), "export files written")
    reloaded = pickle.load((out / "module_state.pkl").open("rb"))
    check(tuple(reloaded.keys()) == RF.EXPECTED_KEY_ORDER and all(np.array_equal(reloaded[k], new_state[k]) for k in new_state), "reloaded state bit-exact, 10 keys only")
    check(full["save_reload_exact"] and full["clock_invariance"]["bit_identical"] and full["aux_head"]["exported"] is False, "export report: save/reload exact, invariance, aux not exported")
    manifest = json.loads((out / "actor_feature_manifest.json").read_text())
    check(manifest["actor_feature_count"] == 35 and manifest["aux_head_exported"] is False and manifest["actor_digest"] == full["new_actor_digest"], "manifest")
    expect(lambda: RF.export_student(src, tmp / "round1", new_state, rep, names35=names35), FileExistsError, "no-clobber export")
    # --- transactional export: staging sibling, atomic promotion, fail-closed cleanup on injected failures
    parent = tmp / "tx"; parent.mkdir()
    no_staging = lambda: [p.name for p in parent.iterdir() if ".staging-" in p.name]  # noqa: E731
    import warm_start as W
    real_cmp, real_inv, real_wj = W.compare_actor_states, RF.invariance_test, C.write_json
    W.compare_actor_states = lambda a, b: {"exact": False, "injected": True}
    try:
        expect(lambda: RF.export_student(src, parent / "r_saveload", new_state, rep, names35=names35), RF.RefitError, "injected save/reload failure -> RefitError")
    finally:
        W.compare_actor_states = real_cmp
    check(not (parent / "r_saveload").exists() and no_staging() == [], "save/reload failure: no final directory, no staging left behind")
    RF.invariance_test = lambda state, obs, **k: {"rows": 64, "bit_identical": False, "max_abs_diff": 1.0}
    try:
        expect(lambda: RF.export_student(src, parent / "r_inv", new_state, rep, names35=names35), RF.RefitError, "injected clock-invariance failure -> RefitError")
    finally:
        RF.invariance_test = real_inv
    check(not (parent / "r_inv").exists() and no_staging() == [], "invariance failure: no final directory, no staging left behind")
    def failing_write(path, payload, **kw):
        if Path(path).name == "f2r_refit_report.json":
            raise OSError("injected report write failure")
        return real_wj(path, payload, **kw)
    C.write_json = failing_write
    try:
        expect(lambda: RF.export_student(src, parent / "r_report", new_state, rep, names35=names35), OSError, "injected report-write failure propagates")
    finally:
        C.write_json = real_wj
    check(not (parent / "r_report").exists() and no_staging() == [], "report-write failure: no final directory, no staging left behind")
    no_lock = lambda: [p.name for p in parent.iterdir() if p.name.endswith(RF.LOCK_SUFFIX)]  # noqa: E731
    check(no_lock() == [], "no lock left behind after the injected failures")
    rt_ok: dict = {}
    ok_full = RF.export_student(src, parent / "r_ok", new_state, rep, names35=names35, runtime_status=rt_ok)
    final_mod = parent / "r_ok" / "rl_module_student"
    check(final_mod.is_dir() and (parent / "r_ok" / "f2r_refit_report.json").is_file() and no_staging() == [] and ok_full["output_module"] == C.rel(final_mod) and ok_full["export_transaction"]["final_path"] == C.rel(parent / "r_ok") and ".staging-" not in json.dumps(ok_full), "successful export: final path only, report/output_module name the FINAL path, staging gone")
    disk_rep = json.loads((parent / "r_ok" / "f2r_refit_report.json").read_text(encoding="utf-8"))
    check(disk_rep == ok_full, "PROVENANCE: the returned canonical report == json.load of the completion marker on disk (no post-commit mutation)")
    check(disk_rep["output_module"] == C.rel(final_mod) and disk_rep["output_module_files_sha256"]["module_state.pkl"] == C.sha256_file(final_mod / "module_state.pkl"), "report on disk names the final module path and its file table matches the promoted bytes")
    check(set(rt_ok) == set(RF.RUNTIME_ONLY_STATUS_KEYS) and rt_ok["canonical_report_sha256"] == C.sha256_file(parent / "r_ok" / RF.COMPLETION_MARKER) and "promotion" not in ok_full["export_transaction"] and "lock_released" not in ok_full["export_transaction"] and ok_full["export_transaction"]["runtime_only_not_persisted"] == list(RF.RUNTIME_ONLY_STATUS_KEYS) and "promotion_policy" in ok_full["export_transaction"], "runtime-only status (promotion method, lock release, canonical sha, final path) delivered OUT of the canonical report, which states the pre-commit policy and lists the non-persisted keys")
    sha_before = C.sha256_file(parent / "r_ok" / RF.COMPLETION_MARKER)
    (parent / "r_race").mkdir()
    expect(lambda: RF.export_student(src, parent / "r_race", new_state, rep, names35=names35), FileExistsError, "existing final path -> refused before any staging")
    check(no_staging() == [] and no_lock() == [], "no staging and no lock created when the final path already exists")
    check(rt_ok["promotion"]["method"] in ("darwin_renameatx_np_RENAME_EXCL", "linux_renameat2_RENAME_NOREPLACE", "windows_MoveFileExW_no_replace", "fallback_exclusive_mkdir_then_child_moves") and rt_ok["lock_released"] is True and C.sha256_file(parent / "r_ok" / RF.COMPLETION_MARKER) == sha_before, f"promotion method reported at runtime only ({rt_ok['promotion']['method']}), lock released; canonical marker unchanged after the commit")
    # --- native no-replace primitive: never replaces an existing (even empty) directory; plain rename would
    import os, platform
    prim = parent / "prim"; prim.mkdir()
    psrc = prim / "src"; psrc.mkdir(); (psrc / "f.txt").write_text("x"); pdst = prim / "dst"; pdst.mkdir()
    try:
        RF.atomic_noreplace_rename_dir(psrc, pdst); native_ok = None
    except FileExistsError:
        native_ok = True
    except RF.NoReplaceUnavailable:
        native_ok = False
    if native_ok is None:
        raise AssertionError("native no-replace rename replaced an existing empty directory")
    check(native_ok is False or ((psrc / "f.txt").exists() and pdst.is_dir() and not list(pdst.iterdir())), "native no-replace rename refuses an existing EMPTY destination directory (source and destination untouched)" if native_ok else "native primitive unavailable on this platform: fallback path is the contract (tested below)")
    (pdst / "z.txt").write_text("z")
    expect(lambda: RF.atomic_noreplace_rename_dir(psrc, pdst), (FileExistsError, RF.NoReplaceUnavailable), "native no-replace rename refuses a NON-empty destination too")
    if platform.system() != "Windows":
        ha = prim / "ha"; ha.mkdir(); (ha / "g.txt").write_text("y"); hb = prim / "hb"; hb.mkdir()
        os.rename(ha, hb)
        check((hb / "g.txt").exists() and not ha.exists(), "hazard documented: a plain POSIX os.rename DOES replace an empty destination directory (hence never used for the promotion)")
    # --- fallback path (forced): exclusive mkdir + child moves, marker last; never replaces; cleanup on mid-way failure
    real_native = RF.atomic_noreplace_rename_dir
    RF.atomic_noreplace_rename_dir = lambda a, b: (_ for _ in ()).throw(RF.NoReplaceUnavailable("forced"))
    try:
        rt_fb: dict = {}; order: list = []
        real_rename0 = os.rename
        def recording_rename(a, b):
            order.append(Path(b).name); return real_rename0(a, b)
        os.rename = recording_rename
        try:
            fb = RF.export_student(src, parent / "r_fallback", new_state, rep, names35=names35, runtime_status=rt_fb)
        finally:
            os.rename = real_rename0
        check(rt_fb["promotion"]["method"] == "fallback_exclusive_mkdir_then_child_moves" and rt_fb["promotion"]["single_step_atomic"] is False and (parent / "r_fallback" / RF.COMPLETION_MARKER).is_file() and (parent / "r_fallback" / "rl_module_student" / "module_state.pkl").is_file() and no_staging() == [] and no_lock() == [], "fallback promotion: final directory complete (marker present), no staging, no lock")
        check(order[-1] == RF.COMPLETION_MARKER and order.index("rl_module_student") < order.index(RF.COMPLETION_MARKER), f"fallback moves the completion marker LAST (rename order {order})")
        check(json.loads((parent / "r_fallback" / RF.COMPLETION_MARKER).read_text(encoding="utf-8")) == fb and "promotion" not in fb["export_transaction"], "fallback: returned canonical report == marker on disk; promotion facts runtime-only")
        (parent / "r_fb_exists").mkdir()
        expect(lambda: RF.export_student(src, parent / "r_fb_exists", new_state, rep, names35=names35), FileExistsError, "fallback with an existing final directory -> refused (exclusive mkdir)")
        check(no_staging() == [] and no_lock() == [] and not list((parent / "r_fb_exists").iterdir()), "fallback refusal: foreign empty directory untouched, no staging/lock")
        real_rename = os.rename
        def failing_rename(a, b):
            if Path(b).name == RF.COMPLETION_MARKER:
                raise OSError("injected failure while moving the completion marker")
            return real_rename(a, b)
        os.rename = failing_rename
        try:
            expect(lambda: RF.export_student(src, parent / "r_fb_mid", new_state, rep, names35=names35), OSError, "fallback mid-way failure propagates")
        finally:
            os.rename = real_rename
        check(not (parent / "r_fb_mid").exists() and no_staging() == [] and no_lock() == [], "fallback mid-way failure: our empty final directory removed, staging removed, lock released")
    finally:
        RF.atomic_noreplace_rename_dir = real_native
    # --- lock: acquire/release semantics, stale lock, failure during promote / acquire / release
    lk, tok = RF.acquire_export_lock(parent / "r_lock")
    expect(lambda: RF.acquire_export_lock(parent / "r_lock"), RF.ExportLockError, "second acquisition of the same final path -> ExportLockError (a FileExistsError)")
    check(issubclass(RF.ExportLockError, FileExistsError) and RF.release_export_lock(lk, "not-my-token") is False and lk.exists(), "release with a foreign token refuses and keeps the lock")
    check(RF.release_export_lock(lk, tok) is True and not lk.exists() and RF.release_export_lock(lk, tok) is False, "release with the owner token removes the lock; a second release is a no-op (False)")
    stale = RF._lock_path_for(parent / "r_stale"); stale.write_text(json.dumps({"pid": 0, "token": "dead", "final": str(parent / "r_stale")}))
    expect(lambda: RF.export_student(src, parent / "r_stale", new_state, rep, names35=names35), RF.ExportLockError, "pre-existing (stale) lock -> loser, fail-closed")
    check(not (parent / "r_stale").exists() and no_staging() == [] and stale.exists(), "acquire failure: no staging, no final, the foreign lock is left for the operator")
    stale.unlink()
    real_promote = RF.promote_staging
    RF.promote_staging = lambda a, b: (_ for _ in ()).throw(OSError("injected promotion failure"))
    try:
        expect(lambda: RF.export_student(src, parent / "r_promote", new_state, rep, names35=names35), OSError, "failure during promotion propagates")
    finally:
        RF.promote_staging = real_promote
    check(not (parent / "r_promote").exists() and no_staging() == [] and no_lock() == [], "promotion failure: no final, staging removed, lock released")
    real_unlink = os.unlink
    def failing_unlink(p, *a, **k):
        if str(p).endswith(RF.LOCK_SUFFIX):
            raise OSError("injected release failure")
        return real_unlink(p, *a, **k)
    os.unlink = failing_unlink
    rt_rel: dict = {}
    try:
        rel = RF.export_student(src, parent / "r_release", new_state, rep, names35=names35, runtime_status=rt_rel)
    finally:
        os.unlink = real_unlink
    check((parent / "r_release" / RF.COMPLETION_MARKER).is_file() and rt_rel["lock_released"] is False and RF._lock_path_for(parent / "r_release").exists() and no_staging() == [], "release failure after a successful promotion: export reported complete (no exception), lock left behind and flagged at RUNTIME only")
    def has_key(obj, key):  # the key as a FIELD anywhere in the report (the name may legitimately appear inside the runtime_only_not_persisted list of strings)
        if isinstance(obj, dict):
            return key in obj or any(has_key(v, key) for v in obj.values())
        return isinstance(obj, list) and any(has_key(v, key) for v in obj if isinstance(v, (dict, list)))
    check(json.loads((parent / "r_release" / RF.COMPLETION_MARKER).read_text(encoding="utf-8")) == rel and not has_key(rel, "lock_released") and not has_key(rel, "promotion") and C.sha256_file(parent / "r_release" / RF.COMPLETION_MARKER) == rt_rel["canonical_report_sha256"], "release failure is NOT persisted: canonical report on disk == returned report, with no lock_released/promotion field anywhere")
    expect(lambda: RF.export_student(src, parent / "r_release", new_state, rep, names35=names35), FileExistsError, "a later export to the same path is refused (final exists / stale lock): fail-closed")
    RF._lock_path_for(parent / "r_release").unlink()
    # --- real concurrency: two threads race for the same final path (barrier-synchronised), repeated
    import threading
    for round_ in range(3):
        target = parent / f"r_conc_{round_}"; barrier = threading.Barrier(2); results = {}
        def worker(name):
            barrier.wait()
            try:
                results[name] = ("ok", RF.export_student(src, target, new_state, rep, names35=names35))
            except FileExistsError as exc:  # ExportLockError is a FileExistsError
                results[name] = ("lost", type(exc).__name__)
            except RF.RefitError as exc:
                results[name] = ("lost", type(exc).__name__)
        th = [threading.Thread(target=worker, args=(n,)) for n in ("A", "B")]
        [x.start() for x in th]; [x.join() for x in th]
        outcomes = sorted(v[0] for v in results.values())
        check(outcomes == ["lost", "ok"] and (target / RF.COMPLETION_MARKER).is_file() and (target / "rl_module_student" / "module_state.pkl").is_file() and no_staging() == [] and no_lock() == [], f"round {round_}: exactly one winner and one loser ({[v[1] if v[0] == 'lost' else 'ok' for v in results.values()]}), final complete, no staging/lock residue")
        winner = [v[1] for v in results.values() if v[0] == "ok"][0]
        check(winner["output_module"] == C.rel(target / "rl_module_student") and json.loads((target / RF.COMPLETION_MARKER).read_text(encoding="utf-8")) == winner and winner["output_module_files_sha256"]["module_state.pkl"] == C.sha256_file(target / "rl_module_student" / "module_state.pkl"), f"round {round_}: the promoted export is the winner's canonical report (== marker on disk, bytes consistent)")
    extra = dict(new_state); extra["aux.weight"] = np.zeros((2, 16), dtype=np.float32)
    expect(lambda: RF.export_student(src, tmp / "round2", extra, rep, names35=names35), RF.RefitError, "extra (aux) key refused at export")
    # --- T1R: fit_student_preserving (synthetic, two roles, group-balanced normalised loss) + criteria P1-P5
    init_t = synthetic_init(seed=8); teacher_t = synthetic_init(seed=23); jul_like = synthetic_init(seed=31)
    rng2 = np.random.default_rng(44)
    ta = rng2.standard_normal((300, 35)).astype(np.float32); ta[:, 0], ta[:, 1] = 0.0, 1.0   # anchor rows
    tt = rng2.standard_normal((300, 35)).astype(np.float32) + 0.5; tt[:, 0], tt[:, 1] = 0.0, 1.0  # T1 on-policy rows
    po = rng2.standard_normal((120, 35)).astype(np.float32) - 0.5; po[:, 0], po[:, 1] = 0.0, 1.0  # preservation (JUL) rows: 120 vs 600 task rows (imbalanced on purpose)
    tobs = np.concatenate([ta, tt]); tlab = RF.numpy_mean(teacher_t, tobs).astype(np.float32); tlab[:, 1] = 0.3 * np.tanh(tobs[:, 4])  # ankle "u_IK"-like smooth label
    phi_t = rng2.uniform(0, 1, 600); task = {"obs35": tobs, "actions": tlab, "clock": np.stack([np.sin(2 * np.pi * phi_t), np.cos(2 * np.pi * phi_t)], 1).astype(np.float32), "seed": np.full(600, 123), "purpose": np.array(["anchor"] * 300 + ["det"] * 300)}
    pact = RF.numpy_mean(jul_like, po).astype(np.float32); phi_p = rng2.uniform(0, 1, 120)
    pres = {"obs35": po, "actions": pact, "clock": np.stack([np.sin(2 * np.pi * phi_p), np.cos(2 * np.pi * phi_p)], 1).astype(np.float32), "seed": np.full(120, 123), "purpose": np.array(["det"] * 120)}
    small = {**budget, "epochs": 40, "lr": 3e-3}
    st3, rep3 = RF.fit_student_preserving(jul_like, task, pres, budget=small, beta=1.0)   # init = the "JUL" net itself (preservation target = its own actions)
    check(tuple(st3.keys()) == RF.EXPECTED_KEY_ORDER and rep3["variant"] == "T1R" and rep3["beta"] == 1.0 and rep3["rows"] == {"task": 600, "preservation": 120}, "preserving fit: 10 keys, T1R report, role counts")
    check(len(rep3["normalisation"]["var_task_per_joint"]) == 2 and len(rep3["normalisation"]["var_pres_per_joint"]) == 2 and all(v > 0 for v in rep3["normalisation"]["var_task_per_joint"] + rep3["normalisation"]["var_pres_per_joint"]), "per-joint variances fixed a priori and reported")
    c3 = rep3["criteria"]
    check(set(c3) == {"P1", "P2", "P3", "P3b", "P4", "P5", "pass_all"} and c3["P4"]["clock_columns_zero"] and c3["P4"]["logstd_constant"] and c3["P4"]["ten_keys"] and c3["P4"]["clock_invariance_bit_identical"] and c3["P5"]["informational"] and c3["P3b"]["mandatory"], "criteria structure: P1-P5, P4 invariants hold, P5 informational, P3b mandatory")
    st_nopres, rep_nopres = RF.fit_student_preserving(jul_like, task, {**pres}, budget=small, beta=1e-6)  # (near) no preservation -> larger deviation from the init on the preservation rows
    check(max(c3["P1"]["value_knee_ankle"]) < max(rep_nopres["criteria"]["P1"]["value_knee_ankle"]), f"beta = 1 preserves the init on the preservation rows better than beta ~ 0 ({max(c3['P1']['value_knee_ankle']):.3f} < {max(rep_nopres['criteria']['P1']['value_knee_ankle']):.3f})")
    again3, _ = RF.fit_student_preserving(jul_like, task, pres, budget=small, beta=1.0)
    check(all(np.array_equal(again3[k], st3[k]) for k in st3), "preserving fit deterministic")
    # group balance (definitional): with a 200:10 imbalance inside a batch, each role term equals the mean over ITS rows only (no row-count weighting)
    import torch
    mt = torch.as_tensor(np.concatenate([np.ones(200, bool), np.zeros(10, bool)])); m = torch.as_tensor(rng2.standard_normal((210, 2)).astype(np.float32)); y = torch.as_tensor(rng2.standard_normal((210, 2)).astype(np.float32))
    vt = torch.as_tensor(np.array([0.4, 0.05], np.float32)); vp = torch.as_tensor(np.array([0.1, 0.05], np.float32))
    lt = RF.normalised_role_term(m, y, mt, vt).item(); lp = RF.normalised_role_term(m, y, ~mt, vp).item()
    et = float(np.mean(np.mean(((m[:200] - y[:200]).numpy() ** 2) / vt.numpy(), axis=0))); ep = float(np.mean(np.mean(((m[200:] - y[200:]).numpy() ** 2) / vp.numpy(), axis=0)))
    check(abs(lt - et) < 1e-5 and abs(lp - ep) < 1e-5 and RF.normalised_role_term(m, y, torch.zeros(210, dtype=torch.bool), vp).item() == 0.0, "group-balanced role terms: per-role per-joint means (200 task rows vs 10 preservation rows weigh the same), zero for an absent role")
    # criteria: fail-closed assert
    ok_rep = {"criteria": {"P1": {"pass": True}, "P2": {"pass": True}, "P3": {"pass": True}, "P3b": {"pass": True}, "P4": {"pass": True}, "P5": {"informational": True}, "pass_all": True}}
    check(RF.assert_t1r_criteria(ok_rep)["pass_all"] is True, "assert_t1r_criteria passes when P1-P4 pass")
    bad_rep = {"criteria": {**ok_rep["criteria"], "P3b": {"pass": False, "knee_rmse_t1_states": 0.8}, "pass_all": False}}
    expect(lambda: RF.assert_t1r_criteria(bad_rep), RF.RefitError, "P3b FAIL (knee on T1 states > 0.75) blocks export/rollout")
    expect(lambda: RF.assert_t1r_criteria({"criteria": {**ok_rep["criteria"], "P1": {"pass": False}, "pass_all": False}}), RF.RefitError, "P1 FAIL blocks")
    expect(lambda: RF.assert_t1r_criteria({}), RF.RefitError, "missing criteria -> refused")
    # evaluate_t1r_criteria: thresholds applied exactly (synthetic state far from the targets fails P1/P3b)
    far = synthetic_init(seed=99); ce = RF.evaluate_t1r_criteria(jul_like, far, task, pres)
    check(ce["P1"]["max"] == 0.10 and ce["P2"]["max_anchors"] == 0.10 and ce["P2"]["max_t1_states"] == 0.15 and ce["P3"]["max"] == 0.50 and ce["P3b"]["max"] == 0.75 and isinstance(ce["pass_all"], bool), "criteria thresholds as preregistered (P1 0.10, P2 0.10/0.15, P3 0.50, P3b 0.75)")
    # preservation contract refusals
    expect(lambda: RF.fit_student_preserving(jul_like, task, {**pres, "purpose": np.array(["anchor"] * 120)}, budget=small), R.F2RContractError, "preservation rows must be det (JUL_H0 states)")
    expect(lambda: RF.fit_student_preserving(jul_like, {**task, "purpose": np.array(["det"] * 600)}, pres, budget=small), R.F2RContractError, "task role must contain the anchors")
    expect(lambda: RF.fit_student_preserving(jul_like, task, {**pres, "seed": np.full(120, 125)}, budget=small), R.F2RContractError, "seed 125 in the preservation role refused")
    expect(lambda: RF.fit_student_preserving(jul_like, {**task, "seed": np.where(np.arange(600) == 3, 127, 123)}, pres, budget=small), R.F2RContractError, "sealed seed in the task role refused")
    expect(lambda: RF.fit_student_preserving(jul_like, task, pres, budget=small, beta=0.0), R.F2RContractError, "beta must be positive")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
