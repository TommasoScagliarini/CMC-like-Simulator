"""V26B Fase A - READ-ONLY diagnostics. No fit, no training, no rollout, no candidate.

Four fail-closed checks on frozen artifacts, using frozen PRODUCTION code:

  1. PARENT PROVENANCE   V26 August imitation -> 35D student, walked digest-by-digest.
                         An artifact is DAgger-CONTAMINATED iff its own receipt carries an
                         on-policy dataset block, or any ancestor does.  Fail-closed on a
                         broken chain, a digest mismatch or an unreachable V26 root.
  2. DISCRETE MISMATCH   the July "phase-aligned" rule as the architect defined it: reject /
                         truncate from the FIRST discrete mismatch.  Uses the production
                         target_domain_noise_adaptation.truncate_before_discrete_mismatch
                         verbatim; nothing is re-implemented.
  3. EXECUTABILITY       does the production target slew limiter reach the same-step u_IK label
                         from the row's own recorded previous endpoint, within the steps left in
                         the row's FSM segment?  PREREGISTERED: > 10% non-executable = FAIL.
  4. COVERAGE + CROSS-PHASE
                         stance / swing / event coverage per dataset, and a fail-closed
                         prohibition on interpolating across an FSM-state boundary.

Cross-platform: pathlib only, no shell, no os-specific call.  Windows x86 and macOS arm64 alike.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import v26b_rev4b_dagger as B4  # noqa: E402   (frozen: teacher corpus view and the 392-row preflight)
import v26b_anchors as VA  # noqa: E402
import v26b_student as VS  # noqa: E402
import v26b_s1a_rollout as SA  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_s1c_protocol as SC  # noqa: E402
import f0_common as C  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402

if str(R.BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(R.BASELINE_DIR))
import target_domain_noise_adaptation as NOISE  # noqa: E402  (PRODUCTION, imported not copied)


class FaseAError(RuntimeError):
    pass


STAGE = "V26B-FASEA-DIAGNOSTICS"

# --- pinned provenance roots (August V26 lineage only; no July artifact anywhere) --------------
PIN_V26_IMITATION = "5bbc6cbd3c7e3ec37524b7b6b69ca017af48057cac5207cf755d3b2f72c2709e"
PIN_V1_TRANSPLANT = "ae846220a6f7f1ac1289ccc9636e3ad2e5bc7842ba7ece0b62bb9d7590e7f587"
PIN_S0D_DISTILLED = "481dd0d22919fc1ec04cdb722409b9711caeb61d57449c210aad7386375b764a"
STUDENT_DIR = VA.OUT_ROOT / "student"
CANDIDATE_DIR = VA.OUT_ROOT / "candidates"
FORBIDDEN_PATH_MARKERS = ("2026-07", "target_domain_dagger_", "target_domain_imitation_2026")

# --- frozen production constants of the reference chain (v3 resolved config) --------------------
SEGMENT_DT_S = 0.01
POLICY_KNOTS = 1
SLEW_LIMIT_RAD_S = {"pros_knee_angle": 2.5, "pros_ankle_angle": 2.0}
REACH_TOL_RAD = SEGMENT_DT_S * min(SLEW_LIMIT_RAD_S.values())   # one ankle slew step = 0.02 rad

# --- PREREGISTERED criterion -------------------------------------------------------------------
EXECUTABILITY_FAIL_FRACTION = 0.10

FSM_IDX = {"wait_hs": 17, "stance": 18, "swing": 19}
EVENT_IDX = {"heel_strike": 12, "toe_off": 13}
PREV_ENDPOINT_IDX = {"pros_knee_angle": 25, "pros_ankle_angle": 30}
JOINT_IDX = {"pros_knee_angle": 2, "pros_ankle_angle": 4}
OUT_DIR = VA.OUT_ROOT / "diagnostics" / "faseA"
RECEIPT_NAME = "v26b_fasea_receipt.json"
RECEIPT_NAME_SUPPLEMENT = "v26b_fasea_receipt_supplement.json"


# =================================================================== 1. provenance ==============

def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _artifact_records() -> dict[str, dict[str, Any]]:
    """Every 35D artifact on disk with a manifest, keyed by directory name."""
    out: dict[str, dict[str, Any]] = {}
    for root in (STUDENT_DIR, CANDIDATE_DIR):
        if not root.is_dir():
            continue
        for d in sorted(p for p in root.iterdir() if p.is_dir()):
            man = d / "rl_module" / "actor_feature_manifest.json"
            if not man.is_file():
                continue
            m = _read_json(man)
            receipts = sorted(p for p in d.glob("*receipt*.json"))
            rec = _read_json(receipts[0]) if receipts else {}
            dataset = rec.get("dataset") or {}
            on_policy = any(k in dataset for k in
                            ("trace_repeat", "prefix_rows", "raw_visited_rows", "on_policy_share"))
            derived = str(m.get("derived_from", ""))
            out[d.name] = {"dir": C.rel(d), "actor_digest": m.get("actor_digest"),
                           "source_actor_digest": m.get("source_actor_digest"),
                           "derived_from": derived,
                           "derived_from_name": Path(derived).parent.name if derived else None,
                           "module_state_sha256": C.sha256_file(d / "rl_module" / "module_state.pkl"),
                           "manifest_sha256": C.sha256_file(man),
                           "own_dataset_is_on_policy": bool(on_policy),
                           "generated_at_utc": rec.get("generated_at_utc"),
                           "receipt": C.rel(receipts[0]) if receipts else None}
    return out


def build_provenance() -> dict[str, Any]:
    """Fail-closed provenance DAG. Contamination is inherited from any ancestor."""
    recs = _artifact_records()
    if not recs:
        raise FaseAError("no 35D artifact with a manifest was found")
    by_digest = {v["actor_digest"]: k for k, v in recs.items() if v["actor_digest"]}
    for name, r in recs.items():
        for mark in FORBIDDEN_PATH_MARKERS:
            if mark in r["derived_from"]:
                raise FaseAError(f"{name}: parent path points at a July artifact ({mark!r})")

    def parent_of(name: str) -> str | None:
        r = recs[name]
        pd = r["source_actor_digest"]
        if pd and pd in by_digest:
            return by_digest[pd]
        pn = r["derived_from_name"]
        return pn if pn in recs else None

    def chain(name: str) -> list[str]:
        seen, out = set(), []
        cur: str | None = name
        while cur is not None:
            if cur in seen:
                raise FaseAError(f"provenance cycle at {cur}")
            seen.add(cur); out.append(cur)
            cur = parent_of(cur)
        return out

    result: dict[str, Any] = {}
    for name, r in recs.items():
        ch = chain(name)
        root = recs[ch[-1]]
        reaches_v26 = root["source_actor_digest"] == PIN_V26_IMITATION
        contaminated = any(recs[a]["own_dataset_is_on_policy"] for a in ch)
        result[name] = {**r, "chain": ch, "reaches_v26_root": bool(reaches_v26),
                        "dagger_contaminated": bool(contaminated)}
    return result


def verify_root_chain(prov: Mapping[str, Any]) -> dict[str, Any]:
    """The V26 -> V1 -> S0D spine must be intact and digest-exact."""
    if "V1_35D_transplant" not in prov or "S0D_35D_DISTILLED" not in prov:
        raise FaseAError("the V1/S0D spine is missing from the artifact set")
    v1, s0d = prov["V1_35D_transplant"], prov["S0D_35D_DISTILLED"]
    if v1["source_actor_digest"] != PIN_V26_IMITATION:
        raise FaseAError(f"V1 source {v1['source_actor_digest']} != pinned V26 imitation root")
    if v1["actor_digest"] != PIN_V1_TRANSPLANT:
        raise FaseAError("V1 actor digest != pinned")
    if s0d["source_actor_digest"] != PIN_V1_TRANSPLANT or s0d["actor_digest"] != PIN_S0D_DISTILLED:
        raise FaseAError("S0D does not descend from V1 with the pinned digests")
    return {"v26_imitation_root": PIN_V26_IMITATION, "v1_transplant": PIN_V1_TRANSPLANT,
            "s0d_distilled": PIN_S0D_DISTILLED, "spine_intact": True}


def clean_parents(prov: Mapping[str, Any]) -> dict[str, Any]:
    """Every artifact that reaches the V26 root through no DAgger round, newest first."""
    clean = {k: v for k, v in prov.items()
             if v["reaches_v26_root"] and not v["dagger_contaminated"] and k != "S0D_35D_DISTILLED"
             and k != "V1_35D_transplant"}
    ordered = sorted(clean.items(), key=lambda kv: str(kv[1]["generated_at_utc"] or ""), reverse=True)
    contaminated = sorted(k for k, v in prov.items() if v["dagger_contaminated"])
    return {"clean_count": len(ordered),
            "clean_newest_first": [{"name": k, "actor_digest": v["actor_digest"],
                                    "generated_at_utc": v["generated_at_utc"],
                                    "module_state_sha256": v["module_state_sha256"],
                                    "chain": v["chain"]} for k, v in ordered],
            "dagger_contaminated": contaminated,
            "AMBIGUITY_FOR_THE_ARCHITECT": (
                "'the last clean 35D artifact' is not unique by itself: the chronologically last "
                "clean artifact and the only clean artifact ever authorised as init/anchor are "
                "different objects. Both are listed; the choice is the architect's, not this tool's")}


# ============================================== 2. discrete mismatch (production operator) ======

def _rows_from_obs(obs: np.ndarray) -> list[dict[str, Any]]:
    return [{"actor_observation_vector_before": np.asarray(o, dtype=float).tolist()}
            for o in np.asarray(obs)]


def discrete_truncation(nominal_obs: np.ndarray, visited_obs: np.ndarray,
                        feature_names: Sequence[str]) -> dict[str, Any]:
    """July's phase-aligned rule, verbatim from production: truncate at the first discrete mismatch."""
    kept, summary = NOISE.truncate_before_discrete_mismatch(
        _rows_from_obs(nominal_obs), _rows_from_obs(visited_obs), list(feature_names))
    idx = NOISE._discrete_feature_indices(list(feature_names))
    return {"operator": "target_domain_noise_adaptation.truncate_before_discrete_mismatch (production, imported)",
            "discrete_feature_indices": [int(i) for i in idx],
            "discrete_feature_names": [str(feature_names[i]) for i in idx],
            "nominal_reference": "the teacher corpus states at the SAME absolute index",
            "retained_rows": int(summary["retained_steps"]),
            "original_rows": int(summary["original_steps"]),
            "first_discrete_mismatch_step": summary["first_discrete_mismatch_step"],
            "retained_fraction": float(summary["retained_steps"]) / max(1, int(summary["original_steps"]))}


# ============================================================== 3. executability =================

def slew_limit_sequence(anchor: float, target: float, limit_rad_s: float,
                        dt_s: float = SEGMENT_DT_S, max_steps: int = 10000) -> list[float]:
    """Production target-slew arithmetic (osim_trj_cmc_like._limit_target_slew) for a constant
    commanded target: previous += clip(target - previous, -limit*dt, +limit*dt)."""
    if not (math.isfinite(limit_rad_s) and limit_rad_s > 0.0):
        raise FaseAError(f"invalid slew limit {limit_rad_s!r}")
    if not (math.isfinite(dt_s) and dt_s > 0.0):
        raise FaseAError(f"invalid dt {dt_s!r}")
    out: list[float] = []
    previous = float(anchor)
    max_delta = limit_rad_s * dt_s
    for _ in range(max_steps):
        desired = float(target) - previous
        previous = previous + max(-max_delta, min(max_delta, desired))
        out.append(previous)
        if abs(float(target) - previous) <= 0.0:
            break
    return out


def steps_to_reach(anchor: float, target: float, limit_rad_s: float,
                   tol_rad: float = REACH_TOL_RAD, dt_s: float = SEGMENT_DT_S) -> int:
    """Policy steps for the production limiter to bring the served target within tol of the label."""
    gap = abs(float(target) - float(anchor))
    if gap <= tol_rad:
        return 0
    return int(math.ceil((gap - tol_rad) / (limit_rad_s * dt_s)))


def fsm_labels(obs: np.ndarray) -> np.ndarray:
    o = np.asarray(obs, dtype=float)
    lab = np.full(len(o), "UNKNOWN", dtype=object)
    lab[o[:, FSM_IDX["wait_hs"]] > 0.5] = "WAIT_HS"
    lab[o[:, FSM_IDX["stance"]] > 0.5] = "STANCE"
    lab[o[:, FSM_IDX["swing"]] > 0.5] = "SWING"
    if np.any(lab == "UNKNOWN"):
        raise FaseAError("a row carries no FSM one-hot: fail closed")
    return lab


def steps_remaining_in_segment(labels: np.ndarray) -> np.ndarray:
    """Rows left, inclusive of the current one, before the FSM state changes (or the trace ends)."""
    n = len(labels)
    rem = np.zeros(n, dtype=int)
    end = n
    for i in range(n - 1, -1, -1):
        if i + 1 < n and labels[i + 1] != labels[i]:
            end = i + 1
        rem[i] = end - i
    return rem


def executability_report(obs: np.ndarray, labels_rad: np.ndarray, admitted: np.ndarray,
                         tag: str) -> dict[str, Any]:
    """Can the production limiter serve the label inside the row's own FSM segment?"""
    o = np.asarray(obs, dtype=float)
    lab = fsm_labels(o)
    rem = steps_remaining_in_segment(lab)
    rows = []
    for i in range(len(o)):
        if not admitted[i]:
            continue
        need = max(steps_to_reach(o[i, PREV_ENDPOINT_IDX["pros_knee_angle"]], labels_rad[i, 0],
                                  SLEW_LIMIT_RAD_S["pros_knee_angle"]),
                   steps_to_reach(o[i, PREV_ENDPOINT_IDX["pros_ankle_angle"]], labels_rad[i, 1],
                                  SLEW_LIMIT_RAD_S["pros_ankle_angle"]))
        rows.append((i, int(need), int(rem[i]), str(lab[i]), bool(need > rem[i])))
    if not rows:
        raise FaseAError(f"{tag}: no admitted row to evaluate")
    need = np.array([r[1] for r in rows]); left = np.array([r[2] for r in rows])
    bad = np.array([r[4] for r in rows]); st = np.array([r[3] for r in rows])
    frac = float(bad.mean())
    per_state = {}
    for s in ("STANCE", "SWING", "WAIT_HS"):
        m = st == s
        if m.any():
            per_state[s] = {"rows": int(m.sum()), "non_executable": int(bad[m].sum()),
                            "fraction": float(bad[m].mean()),
                            "steps_needed_median": float(np.median(need[m])),
                            "steps_left_median": float(np.median(left[m]))}
    worst = sorted(rows, key=lambda r: r[2] - r[1])[:5]
    return {"tag": tag, "admitted_rows": len(rows),
            "steps_needed": {"median": float(np.median(need)), "p90": float(np.quantile(need, 0.9)),
                             "max": int(need.max())},
            "steps_left_in_segment": {"median": float(np.median(left)), "min": int(left.min())},
            "non_executable_rows": int(bad.sum()),
            "non_executable_fraction": frac,
            "preregistered_fail_above": EXECUTABILITY_FAIL_FRACTION,
            "verdict": ("FAIL" if frac > EXECUTABILITY_FAIL_FRACTION else "PASS"),
            "per_fsm_state": per_state,
            "worst_rows_step_need_left": [{"step": r[0] + 1, "needed": r[1], "left": r[2],
                                           "fsm": r[3]} for r in worst],
            "reference_chain": {"limiter": "production target slew limiter, target-to-target",
                                "knee_rad_s": SLEW_LIMIT_RAD_S["pros_knee_angle"],
                                "ankle_rad_s": SLEW_LIMIT_RAD_S["pros_ankle_angle"],
                                "dt_s": SEGMENT_DT_S, "policy_knots": POLICY_KNOTS,
                                "tolerance_rad": REACH_TOL_RAD,
                                "note": "the limiter is the binding stage; the butterworth3 4 Hz "
                                        "reference model adds further lag, so this is a LOWER bound "
                                        "on the steps actually needed"}}


DISCRETE_VARIANTS = {"production_all_8_discrete": (11, 12, 13, 17, 18, 19, 20, 21),
                     "fsm_phase_block_only": (17, 18, 19, 20, 21),
                     "fsm_one_hot_only": (17, 18, 19)}


def discrete_truncation_variants(nominal_obs: np.ndarray, visited_obs: np.ndarray) -> dict[str, Any]:
    """The production rule uses all 8 discrete features, including the instantaneous contact flag.
    A single-sample contact flicker therefore truncates as hard as a genuine phase divergence.
    The variants are REPORTED so the architect can see the consequence; the tool picks none."""
    n = min(len(nominal_obs), len(visited_obs))
    out: dict[str, Any] = {}
    for name, idx in DISCRETE_VARIANTS.items():
        first = None
        for i in range(n):
            if any(nominal_obs[i, j] != visited_obs[i, j] for j in idx):
                first = i + 1
                break
        keep = (first - 1) if first is not None else n
        lab = fsm_labels(visited_obs[:keep]) if keep else np.array([], dtype=object)
        out[name] = {"discrete_indices": list(idx), "first_mismatch_step": first,
                     "retained_rows": int(keep),
                     "retained_stance": int((lab == "STANCE").sum()) if keep else 0,
                     "retained_swing": int((lab == "SWING").sum()) if keep else 0}
    return out


# ==================================================== 4. coverage + cross-phase =================

def coverage_report(obs: np.ndarray, tag: str) -> dict[str, Any]:
    o = np.asarray(obs, dtype=float)
    lab = fsm_labels(o)
    n = len(o)
    return {"tag": tag, "rows": n,
            "stance": int((lab == "STANCE").sum()), "swing": int((lab == "SWING").sum()),
            "wait_hs": int((lab == "WAIT_HS").sum()),
            "stance_fraction": float((lab == "STANCE").mean()),
            "swing_fraction": float((lab == "SWING").mean()),
            "heel_strike_events": int((o[:, EVENT_IDX["heel_strike"]] > 0.5).sum()),
            "toe_off_events": int((o[:, EVENT_IDX["toe_off"]] > 0.5).sum()),
            "in_contact_rows": int((o[:, 11] > 0.5).sum()),
            "constant_features": [int(i) for i in range(o.shape[1]) if float(o[:, i].std()) == 0.0]}


def assert_no_cross_phase_interpolation(nominal_obs: np.ndarray, visited_obs: np.ndarray,
                                        admitted: np.ndarray) -> dict[str, Any]:
    """Fail closed: an admitted row may only be interpolated against a teacher parent in the SAME
    FSM state. Pinning the discrete columns across a state boundary fabricates a row that is in no
    physical state at all."""
    fn = fsm_labels(nominal_obs)
    fv = fsm_labels(visited_obs)
    n = min(len(fn), len(fv))
    same = np.array([fn[i] == fv[i] for i in range(n)])
    adm = np.asarray(admitted, dtype=bool)[:n]
    cross = int((adm & ~same).sum())
    return {"admitted_rows": int(adm.sum()), "same_fsm_state": int((adm & same).sum()),
            "cross_phase_pairs": cross,
            "interpolation_allowed_rows": int((adm & same).sum()),
            "rule": "interpolation is permitted ONLY where the teacher parent and the visited row "
                    "share the FSM state; every other admitted row enters with k=0",
            "verdict": ("CLEAN" if cross == 0 else "CROSS_PHASE_PRESENT_INTERPOLATION_MUST_BE_RESTRICTED")}


# ================================================================= driver =======================

def _teacher_view() -> tuple[np.ndarray, np.ndarray]:
    pre = B4.preflight()
    return (np.asarray(pre["_view"]["obs"], dtype=np.float64),
            np.asarray(pre["_view"]["u_ik"], dtype=np.float64))


def _trace(job: Path) -> np.ndarray:
    tr = DS.trajectory_from_job(job, expected_width=R.ENV_ACTOR_WIDTH)
    return np.asarray(tr["obs35"], dtype=np.float64)


def reachability_admission(obs: np.ndarray, q_ik: np.ndarray, tk: float, ta: float) -> np.ndarray:
    o = np.asarray(obs, dtype=float)
    n = len(o)
    return ((np.abs(q_ik[:n, 0] - o[:, JOINT_IDX["pros_knee_angle"]]) <= tk) &
            (np.abs(q_ik[:n, 1] - o[:, JOINT_IDX["pros_ankle_angle"]]) <= ta))


def run_diagnostics(*, write: bool = True) -> dict[str, Any]:
    names, _, mshas = VS.pinned_names()
    prov = build_provenance()
    spine = verify_root_chain(prov)
    parents = clean_parents(prov)

    obsT, uT = _teacher_view()
    q_ik = SC.decode_action(uT)
    traces = {"S1A": _trace(SA.JOB_DIR),
              "REV4C": _trace(VA.OUT_ROOT / "rollouts" / "rev4c_nominal_det" / "REV4C_35D__v3_canonical__nominal__det"),
              "REV4E": _trace(VA.OUT_ROOT / "rollouts" / "rev4e_nominal_det" / "REV4E_35D__v3_canonical__nominal__det")}

    gk = np.abs(q_ik[:500, 0] - obsT[:500, JOINT_IDX["pros_knee_angle"]])
    ga = np.abs(q_ik[:500, 1] - obsT[:500, JOINT_IDX["pros_ankle_angle"]])
    tk, ta = float(np.quantile(gk, 0.90)), float(np.quantile(ga, 0.90))

    cover = {"teacher_corpus_500": coverage_report(obsT[:500], "teacher_corpus_500")}
    trunc: dict[str, Any] = {}
    execu: dict[str, Any] = {}
    cross: dict[str, Any] = {}
    for tag, o in traces.items():
        n = len(o)
        cover[tag] = coverage_report(o, tag)
        trunc[tag] = discrete_truncation(obsT[:n], o, names)
        trunc[tag]["variants_reported_not_chosen"] = discrete_truncation_variants(obsT[:n], o)
        adm_reach = reachability_admission(o, q_ik, tk, ta)
        adm_trunc = np.zeros(n, dtype=bool)
        adm_trunc[: trunc[tag]["retained_rows"]] = True
        execu[tag] = {"reachability_admission": executability_report(o, q_ik[:n], adm_reach, f"{tag}/reachability"),
                      "discrete_truncation_admission": executability_report(o, q_ik[:n], adm_trunc, f"{tag}/discrete-truncation")
                      if adm_trunc.any() else {"tag": f"{tag}/discrete-truncation", "admitted_rows": 0,
                                               "verdict": "NOT_EVALUABLE_EMPTY_ADMISSION"}}
        cross[tag] = {"reachability_admission": assert_no_cross_phase_interpolation(obsT[:n], o, adm_reach),
                      "discrete_truncation_admission": assert_no_cross_phase_interpolation(obsT[:n], o, adm_trunc)}

    verdicts = [execu[t]["reachability_admission"]["verdict"] for t in traces]
    receipt = {"schema": "v26b_fasea_diagnostics.1", "stage": STAGE,
               "kind": "READ-ONLY diagnostics. No fit, no training, no rollout, no candidate",
               "manifest35_sha256": mshas["manifest35_sha256"],
               "provenance": {"spine": spine, "parents": parents,
                              "artifacts": {k: {kk: vv for kk, vv in v.items() if kk != "chain"} | {"chain": v["chain"]}
                                            for k, v in sorted(prov.items())}},
               "reachability_thresholds_from_teacher_corpus_p90": {"knee_rad": tk, "ankle_rad": ta},
               "coverage": cover,
               "discrete_mismatch_truncation": trunc,
               "executability": execu,
               "cross_phase_interpolation": cross,
               "preregistered_criterion": {"measure": "fraction of admitted rows whose label the "
                                                      "production slew limiter cannot serve inside the row's FSM segment",
                                           "FAIL_above": EXECUTABILITY_FAIL_FRACTION},
               "verdicts_reachability_admission": dict(zip(traces, verdicts)),
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    if write:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        target = OUT_DIR / RECEIPT_NAME
        if target.exists():
            receipt["supersedes_note"] = ("the first receipt is preserved byte-identical; this "
                                          "supplement adds the discrete-truncation variants")
            target = OUT_DIR / RECEIPT_NAME_SUPPLEMENT
        C.write_json(target, receipt, clobber=False)
        receipt["written_to"] = C.rel(target)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="V26B Fase A read-only diagnostics")
    p.add_argument("--run", action="store_true")
    p.add_argument("--no-write", action="store_true")
    a = p.parse_args(argv)
    if not a.run:
        build_provenance(); print(json.dumps({"mode": "dry", "ok": True}, indent=2)); return 0
    rec = run_diagnostics(write=not a.no_write)
    print(json.dumps({"stage": rec["stage"],
                      "clean_parents": [c["name"] for c in rec["provenance"]["parents"]["clean_newest_first"]][:6],
                      "dagger_contaminated": rec["provenance"]["parents"]["dagger_contaminated"],
                      "discrete_first_mismatch": {k: v["first_discrete_mismatch_step"] for k, v in rec["discrete_mismatch_truncation"].items()},
                      "executability": {k: {"reach": v["reachability_admission"]["verdict"],
                                            "frac": round(v["reachability_admission"]["non_executable_fraction"], 4)}
                                        for k, v in rec["executability"].items()}}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
