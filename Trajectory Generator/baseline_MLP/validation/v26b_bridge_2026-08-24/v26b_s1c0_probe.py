"""V26B rev3w — S1C-0 feasibility probe: offline reachability of the ankle reference path.

Token V26B-S1C-FEASIBILITY-PROBE.  Strictly offline and read-only on the frozen artifacts: this
module re-implements verbatim the production operators (absolute decode -> target slew limiter ->
third-order jerk-limited reference model with governor), VALIDATES the re-implementation against
the recorded served reference, and only then computes the reachability envelope.

It executes no episode, no fit, no collection and touches no production file.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v26b_s1c_protocol as SC  # noqa: E402    (rev3v lineage, unmodified)
import v26b_s1b_rollout as SB  # noqa: E402
import v26b_s0d_rollout as SR  # noqa: E402
import v26b_anchors as VA  # noqa: E402
import f0_common as C  # noqa: E402
import f1_common as F1  # noqa: E402
import f1_dataset as DS  # noqa: E402
import f2r_common as R  # noqa: E402


class ProbeError(RuntimeError):
    pass


AUTHORIZED_STAGE = "V26B-S1C-FEASIBILITY-PROBE"
AMENDMENT_REV3W = HERE / "v26b_amendment_rev3w_s1c0_probe.json"
PIN_AMENDMENT_REV3W = "92cf4aaf11dc0698054134c49efaaf34e16f5411579da6499ffae4e203dc2b47"
ENV_SOURCE = C.REPO / "Trajectory Generator" / "osim_trj_cmc_like.py"
PIN_ENV_SOURCE = "26458424c44f18fa1dda20b830fa5e7e825c583cc5d10e6d019cb3bd9a0c6d24"
FUTURE_FIT_TOKEN = "V26B-S1C-FIT"

# frozen ankle parameters (rev3w RECONSTRUCTED_PRODUCTION_CHAIN)
LOW, HIGH = -0.7, 0.7
SLEW_RAD_S = 2.0
SEGMENT_S = 0.01
CONTROL_DT = 0.001
N_FINE = 10
CUTOFF_HZ = 4.0
WC = 2.0 * np.pi * CUTOFF_HZ
VEL_LIM, ACC_LIM, JERK_LIM = 3.5, 55.0, 2750.0
B3_THRESHOLD = SC.B3_THRESHOLD          # -0.03 by reference
B3_WINDOW_FRACTION = 0.25               # [0.55, 0.80] of the gait cycle
VALIDATION_TOL = 1e-3
BURN_IN = 100
NAMES = R.FEATURE_NAMES_35
IDX = {n: i for i, n in enumerate(NAMES)}
OUT_DIR = VA.OUT_ROOT / "s1c0_probe"


def _amendment() -> dict[str, Any]:
    got = C.sha256_file(AMENDMENT_REV3W)
    if got != PIN_AMENDMENT_REV3W:
        raise ProbeError(f"rev3w sha {got} != pinned")
    return json.loads(AMENDMENT_REV3W.read_text(encoding="utf-8"))


def verify_lineage_probe() -> dict[str, Any]:
    lin = SC.verify_lineage_s1c()
    lin["amendment_rev3w"] = C.sha256_file(AMENDMENT_REV3W)
    if lin["amendment_rev3w"] != PIN_AMENDMENT_REV3W:
        raise ProbeError("rev3w sha != pinned")
    got = C.sha256_file(ENV_SOURCE)
    if got != PIN_ENV_SOURCE:
        raise ProbeError(f"the production env source changed ({got}): the reconstruction is no longer valid")
    lin["env_source_sha256"] = got
    lin["production_untouched"] = True
    return lin


# --- verbatim operators ---------------------------------------------------------------------------

def decode_absolute(a: np.ndarray) -> np.ndarray:
    """Operator 3: values = low + 0.5*(a+1)*(high-low), after the [-1,1] clip of operator 1."""
    a = np.clip(np.asarray(a, dtype=np.float64), -1.0, 1.0)
    return LOW + 0.5 * (a + 1.0) * (HIGH - LOW)


def slew_limit(q_raw: Sequence[float], anchor0: float = 0.0) -> np.ndarray:
    """Operator 4: target-to-target rate limit against the previous policy endpoint."""
    out = np.empty(len(q_raw), dtype=np.float64)
    prev = float(anchor0)
    md = SLEW_RAD_S * SEGMENT_S
    for i, q in enumerate(q_raw):
        prev = prev + float(np.clip(float(q) - prev, -md, md))
        out[i] = prev
    return out


def reference_step(q: float, v: float, a: float, q_command: float,
                   *, governor: bool = True, dt: float = CONTROL_DT, n: int = N_FINE) -> tuple[float, float, float]:
    """Operator 7: one policy step of the third-order jerk-limited reference model."""
    for _ in range(int(n)):
        raw_j = WC ** 3 * (q_command - q) - 2.0 * WC ** 2 * v - 2.0 * WC * a
        if governor:
            j = float(np.clip(raw_j, -JERK_LIM, JERK_LIM))
            a_low = max(-ACC_LIM, a - JERK_LIM * dt)
            a_high = min(ACC_LIM, a + JERK_LIM * dt)
            a_high = min(a_high, 2.0 * (VEL_LIM - v) / dt - a)
            a_low = max(a_low, 2.0 * (-VEL_LIM - v) / dt - a)
            raw_an = a + j * dt
            a_next = (min(max(raw_an, a_low), a_high) if a_low <= a_high
                      else float(np.clip(raw_an, -ACC_LIM, ACC_LIM)))
            j = (a_next - a) / dt
        else:
            j = raw_j
            a_next = a + j * dt
        v_next = v + 0.5 * (a + a_next) * dt
        if governor:
            v_next = float(np.clip(v_next, -VEL_LIM, VEL_LIM))
        q = q + v * dt + 0.5 * a * dt ** 2 + j * dt ** 3 / 6.0
        v, a = v_next, a_next
    return q, v, a


# --- validation --------------------------------------------------------------------------------------

def replay(job_dir: Path) -> dict[str, Any]:
    """Replay the RECORDED actions through the re-implemented chain and compare with the record."""
    tr = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    obs = np.asarray(tr["obs35"], dtype=np.float64)
    act = np.asarray(tr["b_raw_action"], dtype=np.float64)
    ref_rec = obs[:, IDX["pros_ankle_angle_served_ref"]]
    ep_rec = obs[:, IDX["pros_ankle_angle_previous_endpoint"]]
    q_raw = decode_absolute(act[:, 1])
    q_cmd = slew_limit(q_raw)
    states = []
    q, v, a = float(ref_rec[0]), 0.0, 0.0
    produced = []
    for i in range(len(q_cmd)):
        states.append((q, v, a))
        q, v, a = reference_step(q, v, a, float(q_cmd[i]))
        produced.append(q)
    produced = np.asarray(produced)
    align = {}
    for k in (0, 1, 2):
        err = np.abs(produced[: len(produced) - k] - ref_rec[k:]) if k else np.abs(produced - ref_rec)
        align[str(k)] = {"max_abs": float(err.max()), "max_abs_after_burn_in": float(err[BURN_IN:].max()),
                         "mean_after_burn_in": float(err[BURN_IN:].mean())}
    best = min(align, key=lambda k: align[k]["max_abs_after_burn_in"])
    ep_align = {}
    for k in (0, 1, 2):
        d = np.abs(q_cmd[: len(q_cmd) - k] - ep_rec[k:]) if k else np.abs(q_cmd - ep_rec)
        ep_align[str(k)] = {"max_abs": float(d.max()), "mean": float(d.mean())}
    ep_best = min(ep_align, key=lambda k: ep_align[k]["mean"])
    md = SLEW_RAD_S * SEGMENT_S
    sat = np.abs(np.diff(np.concatenate([[0.0], q_cmd]))) >= md - 1e-12
    return {"trace_sha256": tr["trace_sha256"], "rows": int(obs.shape[0]),
            "served_ref_alignment": align, "best_served_ref_alignment_steps": int(best),
            "validated": bool(align[best]["max_abs_after_burn_in"] <= VALIDATION_TOL),
            "validation_tolerance": VALIDATION_TOL,
            "previous_endpoint_alignment": ep_align, "best_previous_endpoint_alignment_steps": int(ep_best),
            "raw_command": {"min": float(q_raw.min()), "max": float(q_raw.max()), "frac_negative": float(np.mean(q_raw < 0))},
            "slew_limited_command": {"min": float(q_cmd.min()), "max": float(q_cmd.max()), "frac_negative": float(np.mean(q_cmd < 0))},
            "recorded_previous_endpoint": {"min": float(ep_rec.min()), "max": float(ep_rec.max()), "frac_negative": float(np.mean(ep_rec < 0))},
            "slew_saturated_fraction": float(sat.mean()),
            "_states": states, "_q_cmd": q_cmd, "_ep": ep_rec, "_obs": obs}


# --- reachability envelope ------------------------------------------------------------------------------

def envelope(q0: float, v0: float, a0: float, endpoint0: float, horizon_steps: int) -> dict[str, Any]:
    """Most favourable admissible command (action = -1) under the same slew limiter."""
    q, v, a, e = float(q0), float(v0), float(a0), float(endpoint0)
    md = SLEW_RAD_S * SEGMENT_S
    mins = []
    hit = None
    for k in range(int(horizon_steps)):
        e = e + float(np.clip(LOW - e, -md, md))
        q, v, a = reference_step(q, v, a, e)
        mins.append(q)
        if hit is None and q <= B3_THRESHOLD:
            hit = k + 1
    return {"min_reached": float(np.min(mins)), "steps_to_threshold": hit, "horizon_steps": int(horizon_steps)}


def steady_state(q0: float = 0.2, seconds: float = 2.0) -> float:
    q, v, a = float(q0), 0.0, 0.0
    for _ in range(int(seconds / SEGMENT_S)):
        q, v, a = reference_step(q, v, a, LOW)
    return q


def b3_window_steps(obs: np.ndarray) -> dict[str, Any]:
    cyc = obs[:, IDX["online_left_cycle_duration_s"]]
    valid = cyc[cyc > 0]
    if valid.size == 0:
        raise ProbeError("no valid gait-cycle duration in the trace")
    med = float(np.median(valid))
    return {"median_cycle_s": med, "window_fraction": B3_WINDOW_FRACTION,
            "window_steps": int(round(B3_WINDOW_FRACTION * med / SEGMENT_S))}


def b3_field_status(job_dir: Path) -> dict[str, Any]:
    rows = json.loads((Path(job_dir) / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    ph = np.asarray([float((r.get("reward_terms") or {})["pros_ankle_angle_imitation_target_phase"]) for r in rows])
    return {"all_zero": bool(np.all(ph == 0.0)), "min": float(ph.min()), "max": float(ph.max()),
            "case_c": "B3 itself is NOT EVALUABLE on recorded traces while this field is identically zero"}


def verdict(rep: Mapping[str, Any], worst_min: float, hits: int, sampled: int, ss: float) -> dict[str, Any]:
    if not rep["validated"]:
        return {"verdict": "INDETERMINATE", "reason": "the offline model failed validation against the recorded served reference"}
    if ss > B3_THRESHOLD:
        return {"verdict": "UNREACHABLE_a_mathematical",
                "reason": f"even a sustained most-favourable command converges only to {ss:+.6f} > {B3_THRESHOLD}"}
    if hits < sampled or worst_min > B3_THRESHOLD:
        return {"verdict": "UNREACHABLE_b_within_horizon",
                "reason": f"reachable in the limit (steady state {ss:+.6f}) but only {hits}/{sampled} sampled "
                          f"initial states cross {B3_THRESHOLD} inside the B3 window; worst min {worst_min:+.6f}"}
    return {"verdict": "REACHABLE",
            "reason": f"every sampled initial state ({hits}/{sampled}) crosses {B3_THRESHOLD} inside the B3 window; "
                      f"worst-case minimum {worst_min:+.6f}; steady state {ss:+.6f}",
            "scope_limit": "REFERENCE-PATH reachability only: it does NOT assert that such a command sequence keeps the gait, "
                           "nor that a policy can be trained to emit it"}


def run_probe(*, authorized_stage: str | None, out_dir: Path = OUT_DIR, stride: int = 5) -> dict[str, Any]:
    if authorized_stage != AUTHORIZED_STAGE:
        raise ProbeError(f"requires --authorized-stage {AUTHORIZED_STAGE}; got {authorized_stage!r}")
    lineage = verify_lineage_probe()
    reps = {tag: replay(job) for tag, job in (("S0D", SR.JOB_DIR), ("A2", SB.JOB_DIR))}
    for tag, rep in reps.items():
        if not rep["validated"]:
            raise ProbeError(f"{tag}: model validation FAILED "
                             f"({rep['served_ref_alignment'][str(rep['best_served_ref_alignment_steps'])]})")
    rep = reps["S0D"]
    win = b3_window_steps(rep["_obs"])
    ws = win["window_steps"]
    samples = []
    for row in range(0, rep["rows"], stride):
        q0, v0, a0 = rep["_states"][row]
        e = envelope(q0, v0, a0, float(rep["_ep"][row]), ws)
        samples.append({"row": int(row), "q0": float(q0), "endpoint0": float(rep["_ep"][row]), **e})
    worst = max(s["min_reached"] for s in samples)
    best = min(s["min_reached"] for s in samples)
    hits = sum(1 for s in samples if s["steps_to_threshold"] is not None)
    horizons = {str(h): envelope(*rep["_states"][150], float(rep["_ep"][150]), h) for h in (20, ws, 50, 100, 200)}
    ss = steady_state()
    vd = verdict(rep, worst, hits, len(samples), ss)
    receipt = {"schema": "v26b_s1c0_probe.1", "authorized_stage": AUTHORIZED_STAGE, "amendment_rev3w": PIN_AMENDMENT_REV3W,
               "lineage": lineage,
               "reconstructed_chain": _amendment()["RECONSTRUCTED_PRODUCTION_CHAIN"],
               "model_validation": {tag: {k: v for k, v in r.items() if not k.startswith("_")} for tag, r in reps.items()},
               "b3_window": win, "b3_field_status": {tag: b3_field_status(job) for tag, job in (("S0D", SR.JOB_DIR), ("A2", SB.JOB_DIR))},
               "reachability": {"steady_state_sustained_command": ss, "horizons_from_row_150": horizons,
                                "sampled_initial_states": len(samples), "stride": stride,
                                "worst_case_min_reached": worst, "best_case_min_reached": best,
                                "states_crossing_threshold_within_window": hits, "threshold": B3_THRESHOLD,
                                "samples": samples},
               "verdict": vd,
               "executed_in_this_stage": {"episode": False, "fit": False, "collection": False, "promotion": False,
                                          "production_change": False},
               "s1c1_status": ("ALLOWED to be preregistered" if vd["verdict"] == "REACHABLE" else "FORBIDDEN"),
               "code_digests": {"v26b_s1c0_probe.py": C.sha256_file(Path(__file__).resolve()),
                                "test_v26b_s1c0_probe.py": C.sha256_file(HERE / "test_v26b_s1c0_probe.py") if (HERE / "test_v26b_s1c0_probe.py").is_file() else None},
               "generated_at_utc": C.utc_now(), "git": C.git_snapshot()}
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    path = R.unique_artifact_path(out_dir, f"v26b_s1c0_probe_{time.strftime('%Y%m%d_%H%M%S')}", ".json")
    VA._atomic_fill_reserved(path, json.dumps(receipt, indent=2, ensure_ascii=False, default=str) + "\n")
    return {"receipt_path": C.rel(path), "receipt_sha256": C.sha256_file(path), "receipt": receipt}


def run_fit(*_, **kw):
    raise ProbeError(f"the S1C-1 fit requires --authorized-stage {FUTURE_FIT_TOKEN}, NOT granted; got {kw.get('authorized_stage')!r}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V26B rev3w S1C-0 feasibility probe (offline, read-only)")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        verify_lineage_probe()
        print(json.dumps({"mode": "lineage-only", "ok": True}, indent=2))
        return 0
    out = run_probe(authorized_stage=args.authorized_stage)
    r = out["receipt"]
    print(json.dumps({"receipt": out["receipt_path"], "receipt_sha256": out["receipt_sha256"],
                      "validation": {k: v["validated"] for k, v in r["model_validation"].items()},
                      "verdict": r["verdict"], "b3_window_steps": r["b3_window"]["window_steps"],
                      "worst_min": r["reachability"]["worst_case_min_reached"],
                      "hits": r["reachability"]["states_crossing_threshold_within_window"],
                      "s1c1_status": r["s1c1_status"], "executed": r["executed_in_this_stage"]}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
