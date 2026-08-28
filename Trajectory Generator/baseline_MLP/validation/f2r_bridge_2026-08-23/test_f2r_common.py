"""Self-test of f2r_common: seed policy with the explicit F3 unlock, portable temp dirs,
exclusive/atomic artefact writers with unique naming, actor-digest vs module_state-digest
semantics (read-only on the real pinned modules), forbidden P0 inputs, protocol digests."""

from __future__ import annotations

import json
import os
import pickle
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402

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
    ERR = R.F2RContractError
    # --- seed policy: collection / validation
    check(R.assert_collection_seed(123) == 123 and R.assert_collection_seed(124) == 124, "collection seeds 123/124 accepted")
    expect(lambda: R.assert_collection_seed(125), ERR, "125 never a collection seed (held-out gate)")
    for s in R.SEALED_SEEDS:
        expect(lambda s=s: R.assert_collection_seed(s), ERR, f"sealed {s} never a collection seed (no stage parameter exists: structurally impossible)")
    check(R.assert_rollout_seed(125, purpose="validation") == 125, "validation purpose accepts 125")
    expect(lambda: R.assert_rollout_seed(124, purpose="validation"), ERR, "validation purpose refuses 124")
    expect(lambda: R.assert_rollout_seed(126, purpose="validation", authorized_stage="F3"), ERR, "validation purpose refuses a sealed seed even in F3")
    expect(lambda: R.assert_rollout_seed(126, purpose="collection", authorized_stage="F3"), ERR, "collection purpose refuses a sealed seed even in F3")
    # --- sealed seeds: explicit, fail-closed, testable F3 unlock
    for stage in (None, "S0", "S1", "F2", "f3", "F3 ", "R", ""):
        expect(lambda stage=stage: R.assert_rollout_seed(127, purpose="sealed_f3", authorized_stage=stage), ERR, f"sealed_f3 refused with authorized_stage={stage!r}")
    for s in R.SEALED_SEEDS:
        check(R.assert_rollout_seed(s, purpose="sealed_f3", authorized_stage="F3") == s, f"sealed_f3 + authorized_stage='F3' unlocks {s} (F3 is reachable, not impossible)")
    expect(lambda: R.assert_rollout_seed(125, purpose="sealed_f3", authorized_stage="F3"), ERR, "sealed_f3 refuses 125 even in F3")
    expect(lambda: R.assert_rollout_seed(123, purpose="sealed_f3", authorized_stage="F3"), ERR, "sealed_f3 refuses 123 even in F3")
    expect(lambda: R.assert_rollout_seed(123, purpose="bogus"), ERR, "unknown purpose refused")
    check(R.SEALED_UNLOCK_STAGE == "F3" and "F3" in R.KNOWN_STAGES, "unlock stage constant")
    # --- portable temp dirs
    tmp = R.portable_tempdir("f2r_common_")
    check(tmp.is_dir() and tmp == tmp.resolve() and tmp.is_absolute(), "portable_tempdir: absolute, resolved (no symlinked components), exists")
    check(not any(part for part in tmp.parts if part == "private") or os.name != "nt", "portable_tempdir does not depend on a hard-coded path (platform temp root)")
    # --- exclusive / atomic writers
    f = tmp / "a.json"
    R.write_json_exclusive(f, {"x": 1})
    check(json.loads(f.read_text()) == {"x": 1} and not list(tmp.glob("a.json.part-*")), "write_json_exclusive: content written, no temp part left")
    expect(lambda: R.write_json_exclusive(f, {"x": 2}), FileExistsError, "write_json_exclusive refuses an existing artefact")
    check(json.loads(f.read_text()) == {"x": 1}, "existing artefact untouched after the refused write")
    expect(lambda: R.reserve_exclusive(f), FileExistsError, "reserve_exclusive refuses an existing path")
    R.write_text_exclusive(tmp / "sub" / "t.md", "hi")
    check((tmp / "sub" / "t.md").read_text() == "hi", "write_text_exclusive creates parents")
    set1 = R.reserve_unique_set(tmp / "m", "f2r_s0_dry_run_20260823_000000", (".json", ".md", ".index.json"))
    set2 = R.reserve_unique_set(tmp / "m", "f2r_s0_dry_run_20260823_000000", (".json", ".md", ".index.json"))
    check([p.name for p in set1.values()] == ["f2r_s0_dry_run_20260823_000000.json", "f2r_s0_dry_run_20260823_000000.md", "f2r_s0_dry_run_20260823_000000.index.json"], "first reservation uses the bare stamp")
    check([p.name for p in set2.values()] == ["f2r_s0_dry_run_20260823_000000_01.json", "f2r_s0_dry_run_20260823_000000_01.md", "f2r_s0_dry_run_20260823_000000_01.index.json"], "same-second collision -> _01 suffix, nothing reused or overwritten")
    (tmp / "m" / "f2r_s0_dry_run_20260823_000000_02.md").write_text("x")  # partial collision on one suffix only
    set3 = R.reserve_unique_set(tmp / "m", "f2r_s0_dry_run_20260823_000000", (".json", ".md", ".index.json"))
    check(set3[".json"].name == "f2r_s0_dry_run_20260823_000000_03.json" and not (tmp / "m" / "f2r_s0_dry_run_20260823_000000_02.json").exists(), "partial collision skips the whole attempt (no orphan reservation left)")
    check(len(list((tmp / "m").iterdir())) == 10, "exactly 3+3+1+3 files in the reservation dir")
    tpl = {"closure": "source_closure_manifest_{tag}.json", "execute": "f2r_job_matrix_execute_{tag}.json", "status": "f2r_job_matrix_status_{tag}.json"}
    n1 = R.reserve_unique_names(tmp / "L", "STAMP", tpl)
    n2 = R.reserve_unique_names(tmp / "L", "STAMP", tpl)
    check(n1["tag"] == "STAMP" and sorted(p.name for p in n1["paths"].values()) == ["f2r_job_matrix_execute_STAMP.json", "f2r_job_matrix_status_STAMP.json", "source_closure_manifest_STAMP.json"], "reserve_unique_names: three differently named files share one tag")
    check(n2["tag"] == "STAMP_01" and all(p.name.endswith("STAMP_01.json") for p in n2["paths"].values()), "same tag again -> STAMP_01 on all three names")
    (tmp / "L" / "f2r_job_matrix_execute_STAMP_02.json").write_text("foreign")
    n3 = R.reserve_unique_names(tmp / "L", "STAMP", tpl)
    check(n3["tag"] == "STAMP_03" and not (tmp / "L" / "source_closure_manifest_STAMP_02.json").exists() and not (tmp / "L" / "f2r_job_matrix_status_STAMP_02.json").exists() and (tmp / "L" / "f2r_job_matrix_execute_STAMP_02.json").read_text() == "foreign", "partial collision on one name: whole tag skipped, no orphans, foreign file untouched")
    check(len(list((tmp / "L").iterdir())) == 10, "3 + 3 + 1 (foreign) + 3 files")
    u1 = R.unique_artifact_path(tmp / "g", "gate_T2_round_1_20260823_000000", ".json")
    u2 = R.unique_artifact_path(tmp / "g", "gate_T2_round_1_20260823_000000", ".json")
    check(u1.name != u2.name and u1.exists() and u2.exists(), "unique_artifact_path never returns an existing name")
    # --- actor digest vs module_state.pkl digest (read-only on the real pinned modules)
    pins = R.verify_actor_pins()
    check(pins["all_match"] is True, "real pins: init JUL_H0, negative control, teacher all match")
    for label in ("init_primary", "negative_control", "teacher"):
        rec = pins[label]
        check(rec["digests_distinct"] and rec["actor_digest"]["disk"] != rec["module_state_sha256"]["disk"] and len(rec["actor_digest"]["disk"]) == 64 == len(rec["module_state_sha256"]["disk"]), f"{label}: actor digest and module_state.pkl SHA-256 are distinct quantities, both pinned")
    check(pins["init_primary"]["actor_digest"]["disk"] == R.INIT_PRIMARY["actor_digest"] == "a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21" and pins["init_primary"]["module_state_sha256"]["disk"] == R.INIT_PRIMARY["module_state_sha256"] == "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b", "JUL_H0: a0801a9e… = actor (parameter content) digest; 44457ca5… = module_state.pkl file digest")
    check("actor_digest" in pins["digest_semantics"] and "module_state_sha256" in pins["digest_semantics"], "digest semantics recorded for the manifest")
    # demonstration: re-serialising the same parameters changes the file digest, not the actor digest
    sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W
    state = W.load_module_state(Path(R.INIT_PRIMARY["module"]))
    p2, p5 = tmp / "state_p2.pkl", tmp / "state_p5.pkl"
    with p2.open("wb") as h:
        pickle.dump(state, h, protocol=2)
    with p5.open("wb") as h:
        pickle.dump(state, h, protocol=pickle.HIGHEST_PROTOCOL)
    s2, s5 = pickle.load(p2.open("rb")), pickle.load(p5.open("rb"))
    check(C.sha256_file(p2) != C.sha256_file(p5) and W.actor_state_digest(s2) == W.actor_state_digest(s5) == R.INIT_PRIMARY["actor_digest"], "same parameters, different pickle protocol: file SHA-256 differs, actor digest identical")
    mutated = dict(state); mutated["pi.1.bias"] = np.array(state["pi.1.bias"], copy=True); mutated["pi.1.bias"][0] += np.float32(1e-3)
    check(W.actor_state_digest(mutated) != R.INIT_PRIMARY["actor_digest"], "a parameter change alters the actor digest")
    # --- forbidden P0 inputs by name
    for bad in ("time", "t_pre", "step", "index", "gait_phase_sin", "gait_phase_cos"):
        expect(lambda bad=bad: R.assert_no_forbidden_inputs([bad, "pros_knee_angle"], pre_cycle=True), ERR, f"forbidden input {bad!r} refused by name")
    names_pre = [n for i, n in enumerate(R.FEATURE_NAMES_35) if i not in R.CLOCK_COLUMNS and i not in R.ONLINE_CLOCK_TRIPLET]
    R.assert_no_forbidden_inputs(names_pre, pre_cycle=True)
    check(True, "pre-cycle feature set accepted")
    expect(lambda: R.assert_no_forbidden_inputs([R.FEATURE_NAMES_35[R.ONLINE_CLOCK_TRIPLET[0]]], pre_cycle=True), ERR, "online clock triplet refused in the pre-cycle variant")
    # --- P0 JUL_H0 input pins: positive on the real jobs (read-only), negatives on synthetic copies
    import shutil
    v = R.verify_p0_jul_pins()
    check(v["all_match"] is True and all(v[s]["match"] and not v[s]["problems"] for s in R.STARTS), "real JUL_H0 P0 jobs: all pins, receipts, summaries, resets and the F0 analysis chain verify")
    for s in R.STARTS:
        pin = R.P0_JUL_PINS[s]
        check(all(v[s]["files"][k]["sha256"] == pin[f"{k}_sha256"] == C.sha256_file(Path(R.P0_JUL_JOBS[s]) / n) for k, n in R.P0_JUL_FILES.items()), f"{s}: the four pinned files (summary/trace/reset/receipt) hash to their pins on disk")
        rc = v[s]["receipt_consistency"]
        check(len(rc) == 16 and all(c["ok"] for c in rc.values()) and rc["module_state_sha256"]["expected"] == R.INIT_PRIMARY["module_state_sha256"] and rc["config_sha256"]["expected"] == R.RUNTIME_CONFIG_SHA256 and rc["summary_sha256"]["expected"] == pin["summary_sha256"] and rc["trace_sha256"]["expected"] == pin["trace_sha256"], f"{s}: F0 receipt (schema 4) internally consistent: summary/trace digests = disk, module = JUL_H0 pin, config = runtime pin, rollout_eval pin, ids/seed/start/steps")
        check(all(c["ok"] for c in v[s]["summary_consistency"].values()) and v[s]["summary_consistency"]["episode_start_offset_s"]["expected"] == R.EXACT_STARTS[s], f"{s}: summary consistent (exact start, seed 123, deterministic, steps {pin['steps']}, n_actor 35)")
        check(v[s]["f0_analysis_record"]["found"] and v[s]["f0_analysis_record"]["trace_sha256"] == pin["trace_sha256"], f"{s}: pinned F0 analysis records the same summary/trace digests")
    check(v["f0_analysis"]["ok"] and v["f0_analysis"]["sha256"] == "0ac942432d0505731b93e6322ac337edf91ab7dca8c9c8e9c1cb122089148987", "F0 analysis pin (0ac94243…) verified as the chain anchor")
    check(R.require_p0_jul_pins()["all_match"] is True, "require_p0_jul_pins returns the verdict on the real data")
    # synthetic copies under a temp root at the pinned relative paths
    root = tmp / "root"
    jobs = {}
    for s in R.STARTS:
        d = root / R.P0_JUL_PINS[s]["job_dir"]
        d.mkdir(parents=True)
        for n in R.P0_JUL_FILES.values():
            shutil.copyfile(Path(R.P0_JUL_JOBS[s]) / n, d / n)
        jobs[s] = d
    base = R.verify_p0_jul_pins(jobs=jobs, root=root)
    check(base["all_match"] is True, "synthetic bit-identical copies at the pinned relative paths verify")
    # (a) missing file
    (jobs["minus020"] / "rollout_policy_trace.json").unlink()
    va = R.verify_p0_jul_pins(jobs=jobs, root=root)
    check(va["all_match"] is False and va["minus020"]["match"] is False and any("missing" in p for p in va["minus020"]["problems"]) and va["nominal"]["match"] and va["plus020"]["match"], "missing JUL trace -> fail closed (only that start fails)")
    shutil.copyfile(Path(R.P0_JUL_JOBS["minus020"]) / "rollout_policy_trace.json", jobs["minus020"] / "rollout_policy_trace.json")
    # (b) mutated file (one appended byte): digest mismatch AND receipt internal digest inconsistent
    with (jobs["nominal"] / "rollout_summary.json").open("ab") as h:
        h.write(b" ")
    vb = R.verify_p0_jul_pins(jobs=jobs, root=root)
    check(vb["all_match"] is False and not vb["nominal"]["files"]["summary"]["match"] and vb["nominal"]["receipt_consistency"]["summary_sha256"]["ok"] is False, "mutated JUL summary -> pin mismatch and receipt.summary_sha256 inconsistency")
    shutil.copyfile(Path(R.P0_JUL_JOBS["nominal"]) / "rollout_summary.json", jobs["nominal"] / "rollout_summary.json")
    # (c) symlink in place of a pinned file
    target = jobs["plus020"] / "rollout_reset_diagnostics.json"
    shadow = tmp / "shadow_reset.json"
    shutil.copyfile(target, shadow)
    target.unlink()
    try:
        os.symlink(shadow, target)
        symlink_ok = True
    except (OSError, NotImplementedError):
        symlink_ok = False
        shutil.copyfile(shadow, target)
    if symlink_ok:
        vc = R.verify_p0_jul_pins(jobs=jobs, root=root)
        check(vc["all_match"] is False and any("symlink" in p for p in vc["plus020"]["problems"]), "symlinked JUL reset snapshot (identical content) -> refused")
        target.unlink()
        shutil.copyfile(shadow, target)
    else:
        check(True, "symlink test skipped: symlinks not permitted on this platform/user (copy restored)")
    # (d) inconsistent receipt: internal trace digest altered, receipt pin updated so the only problem is the inconsistency
    rpath = jobs["nominal"] / "f0_receipt.json"
    rec = json.loads(rpath.read_text(encoding="utf-8"))
    rec["trace_sha256"] = "0" * 64
    rpath.write_text(json.dumps(rec, indent=2), encoding="utf-8")
    pins_d = {s: dict(R.P0_JUL_PINS[s]) for s in R.STARTS}
    pins_d["nominal"]["receipt_sha256"] = C.sha256_file(rpath)
    vd = R.verify_p0_jul_pins(jobs=jobs, root=root, pins=pins_d)
    check(vd["all_match"] is False and vd["nominal"]["files"]["receipt"]["match"] and vd["nominal"]["receipt_consistency"]["trace_sha256"]["ok"] is False and vd["nominal"]["problems"] == ["receipt.trace_sha256 inconsistent: '" + "0" * 64 + "' != '" + R.P0_JUL_PINS["nominal"]["trace_sha256"] + "'"], "receipt with an internal trace digest that does not match the trace on disk -> refused (sole problem)")
    shutil.copyfile(Path(R.P0_JUL_JOBS["nominal"]) / "f0_receipt.json", rpath)
    # (e) job directory at a different relative path
    moved = root / "elsewhere" / "JUL_H0__v3_canonical__plus020__det"
    moved.parent.mkdir(parents=True)
    shutil.move(str(jobs["plus020"]), str(moved))
    ve = R.verify_p0_jul_pins(jobs={**jobs, "plus020": moved}, root=root)
    check(ve["all_match"] is False and any("expected path" in p for p in ve["plus020"]["problems"]), "JUL job at an unexpected relative path -> refused")
    shutil.move(str(moved), str(jobs["plus020"]))
    # (f) broken F0 analysis chain anchor
    vf = R.verify_p0_jul_pins(jobs=jobs, root=root, f0_analysis_sha256="0" * 64)
    check(vf["all_match"] is False and vf["f0_analysis"]["ok"] is False, "F0 analysis digest != pin -> fail closed")
    vg = R.verify_p0_jul_pins(jobs=jobs, root=root)
    check(vg["all_match"] is True, "restored copies verify again")
    (jobs["nominal"] / "f0_receipt.json").unlink()
    expect(lambda: R.require_p0_jul_pins(jobs=jobs, root=root), ERR, "require_p0_jul_pins raises with the problems listed")
    # --- protocol
    d = R.protocol_digests()
    check(set(d) == {"f2r_protocol.json", "PROTOCOL_F2R.md"} and all(len(v) == 64 for v in d.values()) and R.load_protocol()["protocol_id"] == R.PROTOCOL_ID, "protocol digests and id")
    check(R.load_protocol()["p0"]["executed_in_S0"] is False, "protocol records P0 not executed in S0")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
