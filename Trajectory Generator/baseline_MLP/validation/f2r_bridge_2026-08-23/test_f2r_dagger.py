"""Self-test of f2r_dagger on synthetic job directories (temp only; S0, no rollout).

The only real artefacts touched are the three pinned anchor job dirs, read
read-only through ``anchor_rows`` when they exist on this machine (skipped
otherwise, with a note).  Every other job dir is synthetic and lives under
``tempfile.mkdtemp``.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f1_obs_adapter as OA  # noqa: E402
import f2r_dagger as DG  # noqa: E402

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


class FakeLabeller:
    """Deterministic labels as a function of the observation (no time input)."""

    def __init__(self) -> None:
        self.calls = 0

    def label(self, obs35: np.ndarray, t_pre: np.ndarray) -> dict:
        self.calls += 1
        obs = np.asarray(obs35, dtype=np.float64)
        return {"actions": np.tanh(obs[:, 2:4] * 0.5), "t_pre_seen": np.asarray(t_pre).shape}


class BadLabeller:
    def label(self, obs35, t_pre):
        return {"actions": np.zeros((obs35.shape[0], 3))}


def write_job(dir_: Path, *, steps: int, seed: int, start: str, stochastic: bool, reset_time: float, rng_seed: int | None = None) -> None:
    rng = np.random.default_rng(seed if rng_seed is None else rng_seed)
    dir_.mkdir(parents=True)
    times = [reset_time + 0.01 * (k + 1) for k in range(steps)]
    rows = []
    for k in range(steps):
        obs = rng.standard_normal(35).tolist()
        obs[0], obs[1] = 0.0, 1.0
        mean = rng.standard_normal(2).tolist()
        noise = rng.standard_normal(2).tolist() if stochastic else None
        raw = [mean[0] + noise[0], mean[1] + noise[1]] if stochastic else mean
        rows.append({"step": k + 1, "time": times[k], "actor_observation_vector_before": obs, "raw_policy_action": raw, "policy_action_mean": mean if stochastic else None, "exploration_action_noise": noise})
    (dir_ / DS.TRACE_FILE).write_text(json.dumps(rows), encoding="utf-8")
    (dir_ / DS.SUMMARY_FILE).write_text(json.dumps({"ok": True, "steps": steps, "n_actor": 35, "action_seed": seed, "action_selection": "stochastic" if stochastic else "deterministic", "episode_start_offset_s": R.EXACT_STARTS[start], "end_reason": "episode_time_limit"}), encoding="utf-8")
    (dir_ / DS.RESET_FILE).write_text(json.dumps({"time": reset_time}), encoding="utf-8")


def synthetic_rows(*, purpose: str, seed: int, start: str, job_id: str, n: int, rng_seed: int, variant=None) -> DG.DatasetRows:
    rng = np.random.default_rng(rng_seed)
    return DG.DatasetRows(obs35=rng.standard_normal((n, 35)), t_pre=10.0 + 0.01 * np.arange(n), actions=rng.standard_normal((n, 2)), job_id=[job_id] * n, step=np.arange(1, n + 1), start=[start] * n, seed=[seed] * n, purpose=[purpose] * n, variant=variant)


def main() -> int:
    tmp = R.portable_tempdir("f2r_dagger_")
    protocol = R.load_protocol()
    lab = FakeLabeller()
    resets = {s: 11.99 + R.EXACT_STARTS[s] for s in R.STARTS}

    # --- synthetic jobs ---------------------------------------------------------------------
    jobs = {}
    for start in R.STARTS:
        d = tmp / f"STUDENT__v3_canonical__{start}__det"
        write_job(d, steps=30, seed=123, start=start, stochastic=False, reset_time=resets[start])
        jobs[("det", start, 123)] = d
        for seed in (123, 124, 125):
            d = tmp / f"STUDENT__v3_canonical__{start}__stoch_seed{seed}_r1"
            write_job(d, steps=25, seed=seed, start=start, stochastic=True, reset_time=resets[start])
            jobs[("stoch", start, seed)] = d
    d126 = tmp / "STUDENT__v3_canonical__nominal__stoch_seed126_r1"
    write_job(d126, steps=10, seed=126, start="nominal", stochastic=True, reset_time=resets["nominal"])

    # --- rows_from_job: positives -------------------------------------------------------------
    det = DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="det", labeller=lab, start="nominal", seed=123)
    check(det.size == 30 and det.obs35.dtype == np.float32 and det.actions.dtype == np.float32 and det.t_pre.dtype == np.float64 and det.step.dtype == np.int64, "det rows dtypes/size")
    check(det.job_id[0] == "STUDENT__v3_canonical__nominal__det" and set(det.purpose.tolist()) == {"det"} and set(det.seed.tolist()) == {123} and set(det.start.tolist()) == {"nominal"}, "det rows metadata")
    check(list(det.step) == list(range(1, 31)), "steps 1..N")
    trace = json.loads((jobs[("det", "nominal", 123)] / DS.TRACE_FILE).read_text())
    t_pre = OA.t_pre_from_trace(resets["nominal"], [r["time"] for r in trace])
    check(np.array_equal(det.t_pre, np.asarray(t_pre)) and det.t_pre[0] == resets["nominal"], "t_pre = [reset] + time[:-1] exact")
    obs_expected = np.asarray([r["actor_observation_vector_before"] for r in trace], dtype=np.float64).astype(np.float32)
    check(np.array_equal(det.obs35, obs_expected), "obs35 = float32-cast trace rows")
    check(np.array_equal(det.actions, np.tanh(obs_expected[:, 2:4].astype(np.float64) * 0.5).astype(np.float32)), "labels from labeller.label(obs35, t_pre)['actions']")
    st124 = DG.rows_from_job(jobs[("stoch", "plus020", 124)], purpose="stoch", labeller=lab, start="plus020", seed=124)
    st123 = DG.rows_from_job(jobs[("stoch", "plus020", 123)], purpose="stoch", labeller=lab, start="plus020", seed=123)
    check(st124.size == 25 and set(st124.seed.tolist()) == {124} and set(st124.purpose.tolist()) == {"stoch"} and st123.size == 25, "stoch rows seed 123/124 ok")
    # --- rows_from_job: structural refusals ------------------------------------------------------
    expect(lambda: DG.rows_from_job(jobs[("stoch", "nominal", 125)], purpose="stoch", labeller=lab, start="nominal", seed=125), R.F2RContractError, "seed 125 (validation gate) refused even as stoch")
    expect(lambda: DG.rows_from_job(d126, purpose="stoch", labeller=lab, start="nominal", seed=126), R.F2RContractError, "sealed seed 126 refused")
    expect(lambda: DG.rows_from_job(jobs[("stoch", "nominal", 124)], purpose="det", labeller=lab, start="nominal", seed=124), R.F2RContractError, "purpose det with seed 124 refused")
    expect(lambda: DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="anchor", labeller=lab, start="nominal", seed=123), R.F2RContractError, "purpose anchor on a non-anchor dir refused")
    expect(lambda: DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="validation_gate", labeller=lab, start="nominal", seed=123), R.F2RContractError, "unknown purpose refused")
    expect(lambda: DG.rows_from_job(jobs[("stoch", "nominal", 123)], purpose="stoch", labeller=lab, start="nominal", seed=124), R.F2RContractError, "declared seed != recorded action_seed refused")
    expect(lambda: DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="det", labeller=lab, start="plus020", seed=123), R.F2RContractError, "declared start != recorded episode_start_offset_s refused")
    expect(lambda: DG.rows_from_job(jobs[("stoch", "nominal", 123)], purpose="det", labeller=lab, start="nominal", seed=123), R.F2RContractError, "purpose det on a stochastic trace refused")
    expect(lambda: DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="stoch", labeller=lab, start="nominal", seed=123), R.F2RContractError, "purpose stoch on a deterministic trace refused")
    expect(lambda: DG.rows_from_job(jobs[("det", "nominal", 123)], purpose="det", labeller=BadLabeller(), start="nominal", seed=123), R.F2RContractError, "labeller with wrong action shape refused")
    expect(lambda: DG.rows_from_job(tmp / "missing", purpose="det", labeller=lab, start="nominal", seed=123), R.F2RContractError, "missing job dir refused")

    # --- DatasetRows basics ---------------------------------------------------------------------
    anchors = DG.DatasetRows.concat([synthetic_rows(purpose="anchor", seed=123, start=s, job_id=f"A_ISO39CLK_V3__v3_canonical__{s}__det", n=40, rng_seed=i) for i, s in enumerate(R.STARTS)], variant="T2")
    anchors.anchor_digest = anchors.digest()
    check(anchors.size == 120 and anchors.per_start_counts() == {s: 40 for s in R.STARTS} and anchors.per_purpose_counts() == {"anchor": 120, "det": 0, "stoch": 0}, "concat / size / per_start_counts")
    check(anchors.validate() is anchors and anchors.anchor_rows().digest() == anchors.anchor_digest, "validate + anchor digest")
    digest_before = anchors.digest()
    longer = DG.DatasetRows.concat([anchors, synthetic_rows(purpose="det", seed=123, start="nominal", job_id="X" * 90, n=3, rng_seed=9)])
    check(longer.anchor_rows().digest() == digest_before, "anchor digest independent of the unicode width after concatenation")
    expect(lambda: DG.DatasetRows(obs35=np.zeros((2, 34)), t_pre=np.zeros(2), actions=np.zeros((2, 2)), job_id=["a", "a"], step=[1, 2], start=["nominal"] * 2, seed=[123] * 2, purpose=["det"] * 2), R.F2RContractError, "obs width checked")
    expect(lambda: DG.DatasetRows(obs35=np.zeros((2, 35)), t_pre=np.zeros(3), actions=np.zeros((2, 2)), job_id=["a", "a"], step=[1, 2], start=["nominal"] * 2, seed=[123] * 2, purpose=["det"] * 2), R.F2RContractError, "row count consistency checked")
    expect(lambda: synthetic_rows(purpose="det", seed=123, start="elsewhere", job_id="j", n=2, rng_seed=1).validate(), R.F2RContractError, "unknown start refused by validate")
    expect(lambda: synthetic_rows(purpose="det", seed=124, start="nominal", job_id="j", n=2, rng_seed=1).validate(), R.F2RContractError, "det rows with seed 124 refused by validate")

    # --- aggregate ------------------------------------------------------------------------------
    det_all = [DG.rows_from_job(jobs[("det", s, 123)], purpose="det", labeller=lab, start=s, seed=123) for s in R.STARTS]
    stoch_all = [DG.rows_from_job(jobs[("stoch", s, seed)], purpose="stoch", labeller=lab, start=s, seed=seed) for s in R.STARTS for seed in (123, 124)]
    d2, rep = DG.aggregate(anchors, det_all + stoch_all, anchor_digest=anchors.anchor_digest)
    check(d2.size == 120 + 3 * 30 + 6 * 25 and rep["added"] == 240 and rep["deduplicated"] == 0 and rep["rows_before"] == 120, "aggregate counts")
    check(rep["per_start_counts"] == {s: 40 + 30 + 50 for s in R.STARTS} and rep["per_start_added"] == {s: 80 for s in R.STARTS}, "per-start counts and added")
    check(d2.variant == "T2" and d2.anchor_digest == anchors.anchor_digest and d2.anchor_rows().digest() == anchors.anchor_digest and rep["anchors_unchanged"], "anchors unchanged after aggregation")
    check(d2.purpose[:120].tolist() == ["anchor"] * 120 and rep["per_purpose_counts"] == {"anchor": 120, "det": 90, "stoch": 150} and rep["per_seed_counts"] == {"123": 120 + 90 + 75, "124": 75}, "order and purpose/seed counts")
    d3, rep3 = DG.aggregate(d2, det_all + [st124], anchor_digest=anchors.anchor_digest)
    check(d3.size == d2.size and rep3["deduplicated"] == 90 + 25 and rep3["added"] == 0, "dedup by (job_id, step) keeps the first occurrence")
    dup_tampered = DG.DatasetRows(**{k: getattr(det_all[0], k) for k in DG.ARRAY_ORDER})
    dup_tampered.actions = dup_tampered.actions + np.float32(0.5)
    d4, rep4 = DG.aggregate(d2, [dup_tampered], anchor_digest=anchors.anchor_digest)
    check(rep4["deduplicated"] == 30 and np.array_equal(d4.actions, d2.actions), "duplicate key: first occurrence (original labels) kept")
    # anchor immutability
    tampered = DG.DatasetRows.concat([d2], variant=d2.variant, anchor_digest=d2.anchor_digest)
    tampered.actions[5] = tampered.actions[5] + np.float32(1e-3)
    expect(lambda: DG.aggregate(tampered, [st124], anchor_digest=anchors.anchor_digest), R.F2RContractError, "tampered anchor action -> anchors re-hash mismatch raises")
    expect(lambda: DG.aggregate(d2, [st124], anchor_digest="0" * 64), R.F2RContractError, "wrong anchor_digest argument raises")
    no_anchor = DG.DatasetRows.concat(det_all, variant="T2")
    expect(lambda: DG.aggregate(no_anchor, [st124], anchor_digest=no_anchor.digest()), R.F2RContractError, "dataset without anchor rows refused")
    expect(lambda: DG.aggregate(d2, [synthetic_rows(purpose="anchor", seed=123, start="nominal", job_id="new_anchor", n=3, rng_seed=4)], anchor_digest=anchors.anchor_digest), R.F2RContractError, "new anchor rows through aggregation refused")
    # seed 125 / sealed / purposes
    v125 = synthetic_rows(purpose="stoch", seed=125, start="nominal", job_id="STUDENT__v3_canonical__nominal__stoch_seed125_r1", n=5, rng_seed=5)
    expect(lambda: DG.aggregate(d2, [v125], anchor_digest=anchors.anchor_digest), R.F2RContractError, "seed-125 DatasetRows injected -> raise")
    v126 = synthetic_rows(purpose="stoch", seed=126, start="nominal", job_id="sealed", n=5, rng_seed=5)
    expect(lambda: DG.aggregate(d2, [v126], anchor_digest=anchors.anchor_digest), R.F2RContractError, "sealed-seed rows injected -> raise")
    cur125 = DG.DatasetRows.concat([d2, v125], variant="T2", anchor_digest=d2.anchor_digest)
    expect(lambda: DG.aggregate(cur125, [st124], anchor_digest=anchors.anchor_digest), R.F2RContractError, "seed 125 already in current -> raise")
    expect(lambda: DG.aggregate(d2, [synthetic_rows(purpose="validation_gate", seed=123, start="nominal", job_id="g", n=2, rng_seed=6)], anchor_digest=anchors.anchor_digest), R.F2RContractError, "invalid purpose refused")
    expect(lambda: DG.aggregate(d2, [synthetic_rows(purpose="det", seed=124, start="nominal", job_id="g", n=2, rng_seed=6)], anchor_digest=anchors.anchor_digest), R.F2RContractError, "det rows with seed 124 refused")
    expect(lambda: DG.aggregate(d2, [synthetic_rows(purpose="stoch", seed=124, start="nominal", job_id="g", n=2, rng_seed=6, variant="T3")], anchor_digest=anchors.anchor_digest), R.F2RContractError, "variant mismatch refused")
    expect(lambda: DG.aggregate(d2, ["not rows"], anchor_digest=anchors.anchor_digest), R.F2RContractError, "non-DatasetRows refused")

    # --- round_plan ---------------------------------------------------------------------------
    p = DG.round_plan("T1", 1, protocol=protocol)
    check(p["max_rounds"] == 1 and p["stop_always"] is True and [x["purpose"] for x in p["rollouts"]] == ["det"] and p["rollouts"][0]["seed"] == 123 and p["gates"] == ["A"] and p["initial_dataset"] == "anchors_only", "T1 round 1: det only, stop always, gate A")
    expect(lambda: DG.round_plan("T1", 2, protocol=protocol), R.F2RContractError, "T1 round 2 raises")
    p1 = DG.round_plan("T2", 1, protocol=protocol)
    check(p1["initial_dataset"] == "anchors_only" and p1["max_rounds"] == 4 and p1["gates"] == ["A", "B", "C", "V"] and p1["next_round"] == 2 and p1["refit"]["init"] == "JUL_H0 (theta_0)", "T2 round 1")
    check([x["purpose"] for x in p1["rollouts"]] == ["det", "stoch", "validation_gate"] and p1["rollouts"][1]["seeds"] == [123, 124] and p1["rollouts"][2]["seed"] == 125 and p1["rollouts"][2]["enters_dataset"] is False and all(x["starts"] == 3 for x in p1["rollouts"]), "T2 rollouts: det 123, stoch 123/124, gate 125 (never in dataset)")
    check(p1["on_all_pass"] == "promote_to_R_without_aggregation" and p1["on_fail"] == "aggregate_123_124_only_if_budget" and p1["aggregation_allowed_after_fail"] is True and p1["seed_125_enters_dataset"] is False, "T2 outcomes")
    p4 = DG.round_plan("T2", 4, protocol=protocol)
    check(p4["initial_dataset"] == "D_4" and p4["next_round"] is None and p4["aggregation_allowed_after_fail"] is False and p4["budget_remaining_after_round"] == 0 and p4["refit"]["init"] == "theta_3", "T2 round 4: last round, no further aggregation")
    expect(lambda: DG.round_plan("T2", 5, protocol=protocol), R.F2RContractError, "T2 round 5 raises")
    expect(lambda: DG.round_plan("T2", 0, protocol=protocol), R.F2RContractError, "round 0 raises")
    p3 = DG.round_plan("T3", 2, protocol=protocol)
    check(p3["max_rounds"] == 2 and p3["requires_trigger"] is True and p3["tuning"] == "none" and p3["next_round"] is None and p3["initial_dataset"] == "D_2", "T3 max 2 rounds, trigger required")
    expect(lambda: DG.round_plan("T3", 3, protocol=protocol), R.F2RContractError, "T3 round 3 raises")
    expect(lambda: DG.round_plan("T4", 1, protocol=protocol), R.F2RContractError, "unknown variant raises")
    check(all(not any(x["purpose"] == "validation_gate" and x["enters_dataset"] for x in DG.round_plan(v, r, protocol=protocol)["rollouts"]) for v, m in (("T1", 1), ("T2", 4), ("T3", 2)) for r in range(1, m + 1)), "validation gate never enters a dataset in any plan")
    json.dumps(p1)
    check(True, "round plan JSON-serialisable")

    # --- save / load ----------------------------------------------------------------------------
    out = DG.save_dataset(tmp / "ds", "R1", d2, extra_receipt={"note": "selftest"})
    rows2, receipt = DG.load_dataset(Path(out["npz"]), Path(out["receipt"]))
    check(rows2.digest() == d2.digest() == out["rows_digest"] and rows2.variant == "T2" and rows2.anchor_digest == anchors.anchor_digest and receipt["note"] == "selftest", "save/load round trip with digest")
    check(all(np.array_equal(getattr(rows2, k), getattr(d2, k)) for k in DG.ARRAY_ORDER), "arrays bit-identical after reload")
    check(receipt["rows"] == d2.size and receipt["per_start_counts"] == d2.per_start_counts() and receipt["anchor_rows"] == 120 and receipt["per_purpose_counts"] == d2.per_purpose_counts(), "receipt counts")
    check(receipt["npz"] == C.rel(npz_path := Path(out["npz"])) and "/" in receipt["npz"] and "\\" not in receipt["npz"] and not Path(receipt["npz"]).is_absolute(), "receipt npz is a POSIX relpath")
    check(set(receipt["arrays_sha256"]) == set(DG.ARRAY_ORDER) and receipt["arrays_sha256"] == d2.array_digests(), "receipt per-array digests")
    expect(lambda: DG.save_dataset(tmp / "ds", "R1", d2), FileExistsError, "no-clobber")
    npz = npz_path
    data = dict(np.load(npz, allow_pickle=False))
    data["actions"] = data["actions"] + np.float32(1e-3)
    np.savez(npz, **data)
    expect(lambda: DG.load_dataset(npz, Path(out["receipt"])), R.F2RContractError, "tampered npz refused")
    out2 = DG.save_dataset(tmp / "ds", "R1b", d2)
    rec_path = Path(out2["receipt"])
    rec = json.loads(rec_path.read_text(encoding="utf-8"))
    data = dict(np.load(Path(out2["npz"]), allow_pickle=False))
    data["seed"][-1] = 125
    np.savez(Path(out2["npz"]), **data)
    rec["npz_sha256"] = C.sha256_file(Path(out2["npz"]))
    rec_path.write_text(json.dumps(rec), encoding="utf-8")
    expect(lambda: DG.load_dataset(Path(out2["npz"]), rec_path), R.F2RContractError, "npz re-hashed with a seed-125 row still refused (array digest)")
    expect(lambda: DG.save_dataset(tmp / "ds", "bad", cur125), R.F2RContractError, "saving a dataset with seed 125 refused")

    # --- real anchors (read-only, only if present on this machine) -------------------------------
    pins = R.verify_anchor_pins()
    if pins["all_match"]:
        real = DG.anchor_rows(FakeLabeller(), variant="T2")
        check(real.size == int(protocol["anchors"]["rows"]) and real.per_start_counts() == {s: 500 for s in R.STARTS}, "real anchors: 1500 rows, 500 per start")
        check(set(real.purpose.tolist()) == {"anchor"} and set(real.seed.tolist()) == {123} and real.anchor_digest == real.digest() and real.variant == "T2", "real anchors: purpose/seed/digest")
        check(sorted(set(real.job_id.tolist())) == sorted(Path(v["job_dir"]).name for v in R.ANCHORS.values()), "real anchors: job ids")
        check(np.array_equal(real.obs35[:, 0], np.zeros(1500, np.float32)) and np.array_equal(real.obs35[:, 1], np.ones(1500, np.float32)), "real anchors: env vector has the dead clock pair (0,1)")
        check(real.anchor_rows().digest() == DG.anchor_rows(FakeLabeller()).anchor_digest, "real anchors: digest reproducible")
        expect(lambda: DG.rows_from_job(R.ANCHORS["nominal"]["job_dir"], purpose="det", labeller=lab, start="nominal", seed=123), R.F2RContractError, "anchor dir can only enter as purpose anchor")
        expect(lambda: DG.rows_from_job(R.ANCHORS["nominal"]["job_dir"], purpose="anchor", labeller=lab, start="plus020", seed=123), R.F2RContractError, "anchor dir with the wrong start refused")
        dd, rr = DG.aggregate(real, det_all, anchor_digest=real.anchor_digest)
        check(dd.size == 1500 + 90 and rr["added"] == 90 and dd.anchor_rows().digest() == real.anchor_digest, "aggregate synthetic det rows onto the real anchors")
    else:
        print("NOTE: pinned anchor job dirs not available on this machine; real-anchor checks skipped")
    # ================= T1R corrective commissioning tooling (architect decision 2026-08-23) =================
    # --- RecordedActionLabeller: recorded actions as preservation targets, exact grid only
    jd = tmp / "t1r_jul_like"; write_job(jd, steps=20, seed=123, start="nominal", stochastic=False, reset_time=30.0)
    ral = DG.RecordedActionLabeller(jd)
    rows_jd = json.loads((jd / DS.TRACE_FILE).read_text())
    obs_jd = np.asarray([r["actor_observation_vector_before"] for r in rows_jd], dtype=np.float32)
    lab_jd = ral.label(obs_jd, ral.t_pre)
    check(lab_jd["source"] == "recorded_raw_policy_action" and np.array_equal(lab_jd["actions"], np.asarray([r["raw_policy_action"] for r in rows_jd], dtype=np.float32)) and ral.t_pre[0] == 30.0, "recorded-action labeller returns the recorded actions (never a teacher label) on the exact grid")
    expect(lambda: ral.label(obs_jd, ral.t_pre + 1e-9), R.F2RContractError, "recorded-action labeller refuses an off-grid t_pre")
    expect(lambda: ral.label(obs_jd[:10], ral.t_pre[:10]), R.F2RContractError, "recorded-action labeller refuses a partial grid")
    # --- dedup within role: first kept; identical inputs with different targets -> fail closed
    base = synthetic_rows(purpose="det", seed=123, start="nominal", job_id="x", n=30, rng_seed=5, variant="T1R")
    dup = DG.DatasetRows(obs35=np.concatenate([base.obs35, base.obs35[:4]]), t_pre=np.concatenate([base.t_pre, base.t_pre[:4] + 5.0]), actions=np.concatenate([base.actions, base.actions[:4]]), job_id=list(base.job_id) + ["y"] * 4, step=np.concatenate([base.step, np.arange(1, 5)]), start=["nominal"] * 34, seed=[123] * 34, purpose=["det"] * 34, variant="T1R")
    dd, st = DG.dedup_within_role(dup, role="task")
    check(dd.size == 30 and st["duplicates_dropped"] == 4 and st["rows_raw"] == 34 and st["duplicates"][0]["kept"] == "x#1" and st["duplicates"][0]["dropped"] == "y#1", "dedup within role: 4 bitwise duplicates dropped, first occurrence kept, provenance listed")
    conflict = DG.DatasetRows(obs35=dup.obs35, t_pre=dup.t_pre, actions=np.concatenate([base.actions, base.actions[:4] + 0.1]), job_id=dup.job_id, step=dup.step, start=dup.start, seed=dup.seed, purpose=dup.purpose, variant="T1R")
    expect(lambda: DG.dedup_within_role(conflict, role="task"), R.F2RContractError, "identical inputs with different targets within a role -> fail closed")
    # --- cross-role collisions: exactly the reset rows (one per start), preservation wins, target delta reported
    def role_rows(tag, seed_obs, reset_rows):
        parts = []
        for i, s_ in enumerate(R.STARTS):
            rr = synthetic_rows(purpose="det", seed=123, start=s_, job_id=f"{tag}:{s_}", n=12, rng_seed=seed_obs + i, variant="T1R")
            rr.obs35[0] = reset_rows[s_]; rr.t_pre = np.array([100.0 + i] + [100.0 + i + 0.01 * k for k in range(1, 12)])
            parts.append(rr)
        return DG.DatasetRows.concat(parts, variant="T1R")
    resets = {s_: np.random.default_rng(77 + i).standard_normal(35).astype(np.float32) for i, s_ in enumerate(R.STARTS)}
    task_r = role_rows("task", 200, resets); pres_r = role_rows("jul", 300, resets)
    reset_times = {s_: 100.0 + i for i, s_ in enumerate(R.STARTS)}
    tf, col = DG.remove_cross_role_collisions(task_r, pres_r, reset_times=reset_times)
    check(tf.size == 33 and col["count"] == 3 and sorted(c["start"] for c in col["collisions"]) == sorted(R.STARTS) and all(c["t_pre"] == reset_times[c["start"]] and len(c["input_sha256"]) == 64 and c["task_row"].endswith("#1") and c["pres_row"].endswith("#1") and c["task_target_knee_ankle"] != c["pres_target_knee_ankle"] and any(abs(d) > 0 for d in c["target_delta_knee_ankle"]) for c in col["collisions"]), "cross-role: exactly the 3 reset collisions removed from task; input digest, both targets and their delta reported (targets differ)")
    extra = DG.DatasetRows.concat([task_r], variant="T1R"); extra.obs35[5] = pres_r.obs35[7]
    expect(lambda: DG.remove_cross_role_collisions(extra, pres_r, reset_times=reset_times), R.F2RContractError, "an additional non-reset cross-role collision -> fail closed")
    missing = DG.DatasetRows.concat([task_r], variant="T1R"); missing.obs35[0] = missing.obs35[0] + 1e-3
    expect(lambda: DG.remove_cross_role_collisions(missing, pres_r, reset_times=reset_times), R.F2RContractError, "a missing reset collision (only 2 of 3) -> fail closed")
    expect(lambda: DG.remove_cross_role_collisions(task_r, pres_r, reset_times={s_: 0.0 for s_ in R.STARTS}), R.F2RContractError, "collision not at the reset time -> fail closed")
    # --- round plan T1R
    plan_t1r = DG.round_plan("T1R", 1, protocol=R.load_protocol())
    plan_t1 = DG.round_plan("T1", 1, protocol=R.load_protocol())
    check(plan_t1["rollouts"] == [{"purpose": "det", "seed": 123, "starts": 3, "enters_dataset": True, "role": "training side: gate measurement A/B/C and aggregation"}] and plan_t1["gates"] == ["A"] and plan_t1["stop_always"], "T1 legacy plan unchanged (det dict byte-identical)")
    check(plan_t1r["rollouts"] == [{"purpose": "det", "seed": 123, "starts": 3, "enters_dataset": False, "generates_labels": False, "role": "commissioning gate measurement A/B/C only; no aggregation"}] and plan_t1r["aggregation"].startswith("none") and plan_t1r["gates"] == ["A"] and plan_t1r["stop_always"] and plan_t1r["on_all_pass"] == "stop" and plan_t1r["unlock_t2_on_A_pass"] and plan_t1r["t3_trigger"] == "never from T1R" and plan_t1r["refit"]["beta"] == 1.0 and "JUL_H0" in plan_t1r["refit"]["init"] and plan_t1r["aggregation_allowed_after_fail"] is False, "round_plan T1R: 3 det only, gate A, stop always, unlock on A pass, no T3, beta 1, init JUL_H0, no aggregation")
    expect(lambda: DG.round_plan("T1R", 2, protocol=R.load_protocol()), R.F2RContractError, "T1R has exactly one round")
    # --- T2 round 1 init lock (no T1R gate artefact yet) and CLI refusal in S0
    expect(lambda: DG.t2_round1_init_module(refit_dir=tmp / "no_refit", gate_dir=tmp / "no_gate", rollouts_dir=tmp / "no_roll"), R.F2RContractError, "T2 round 1 locked without a verified T1R gate artefact (no fallback)")
    expect(lambda: DG.main(["refit-t1r", "--out-dir", str(tmp / "t1r_out")]), SystemExit, "CLI refit-t1r refused without --authorized-stage S1")
    expect(lambda: DG.main(["refit", "--variant", "T1R", "--round", "1", "--out-dir", str(tmp / "t1r_out2"), "--authorized-stage", "S1"]), SystemExit, "refit --variant T1R redirected to refit-t1r (refused)")
    check(not (tmp / "t1r_out").exists() and not (tmp / "t1r_out2").exists(), "refused CLIs wrote nothing")
    # --- REAL artefacts, read-only, no fit: the T1R two-role dataset has exactly the preregistered structure
    pins_t1 = R.verify_t1_pins(); check(pins_t1["all_match"] is True, "T1 student + 3 rollouts match their pins")
    labellers = DG.build_labellers("T2", R.OUT_CACHE)
    task_real, pres_real, info = DG.build_t1r_dataset(labellers)
    check(info["counts"] == DG.T1R_EXPECTED_COUNTS and task_real.size == 2994 and pres_real.size == 1176, f"real T1R dataset: counts {info['counts']} == preregistered")
    check(task_real.per_purpose_counts() == {"anchor": 1497, "det": 1497, "stoch": 0} and pres_real.per_purpose_counts() == {"anchor": 0, "det": 1176, "stoch": 0} and task_real.per_seed_counts() == {"123": 2994} and pres_real.per_seed_counts() == {"123": 1176}, "real roles: task = 1497 anchor + 1497 T1 rows, preservation = 1176 JUL rows, seed 123 only")
    cr = info["cross_role"]["collisions"]
    check(len(cr) == 3 and all(c["t_pre"] == info["reset_times"][c["start"]] and c["task_row"] == f"anchor:{c['start']}#1" and c["pres_row"] == f"jul:{c['start']}#1" and max(abs(d) for d in c["target_delta_knee_ankle"]) > 0.01 for c in cr), "real collisions: the 3 anchor reset rows vs the JUL reset rows (input identical, targets differ; deltas reported)")
    check(info["dedup_within_role"]["task"]["duplicates_dropped"] == 3 and all(d["kept"].startswith("anchor:") and d["dropped"].startswith("t1:") for d in info["dedup_within_role"]["task"]["duplicates"]) and info["dedup_within_role"]["preservation"]["duplicates_dropped"] == 0, "real within-role duplicates: the 3 T1 reset rows (identical to the anchor reset rows, identical labels) dropped; none in preservation")
    # preservation targets are the recorded JUL actions (never u_T): compare with the JUL traces
    for s_ in R.STARTS:
        rows_j = json.loads((Path(R.P0_JUL_JOBS[s_]) / DS.TRACE_FILE).read_text()); act_j = np.asarray([r["raw_policy_action"] for r in rows_j], dtype=np.float32)
        sel = pres_real.start == s_
        check(np.array_equal(pres_real.actions[sel], act_j) and np.array_equal(pres_real.step[sel], np.arange(1, len(rows_j) + 1)), f"{s_}: preservation targets == recorded JUL_H0 actions ({len(rows_j)} rows)")
    # task labels: knee = u_T, ankle = u_IK from the caches at the exact t_pre (spot check on the anchor rows)
    lab_nom = labellers["nominal"]; a_nom = task_real.subset((task_real.start == "nominal") & (task_real.purpose == "anchor"))
    out_nom = lab_nom.label(a_nom.obs35, a_nom.t_pre)
    check(np.array_equal(a_nom.actions[:, 0], out_nom["u_T"][:, 0]) and np.array_equal(a_nom.actions[:, 1], out_nom["u_IK"][:, 1]) and out_nom["components"] == {"knee": "u_T", "ankle": "u_IK"}, "real task labels: knee = u_T, ankle = u_IK (exact t_pre lookup)")
    fa, fp = DG.t1r_fit_arrays(task_real, pres_real, labellers)
    check(fa["obs35"].shape == (2994, 35) and fa["clock"].shape == (2994, 2) and fp["obs35"].shape == (1176, 35) and fp["clock"].shape == (1176, 2) and set(fa["purpose"]) == {"anchor", "det"} and set(fp["purpose"]) == {"det"}, "fit arrays for both roles (clock labels from the caches), no fit performed")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
