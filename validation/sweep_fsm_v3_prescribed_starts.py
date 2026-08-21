#!/usr/bin/env python3
"""L2 — actor-FSM v2 vs v3 on the prescribed (perfect-gait) tape, start sweep.

The policy cannot act (``action_mode="delta"``, ``max_delta_rad=0``): the
served reference is the AB06 tape and the prescribed GRF drives the
dynamics, so every FSM outcome is attributable to the FSM, not to a policy.
Runs each start offset under the binary-active V26 stack with the chain
hygiene gates, once per FSM version, in a process pool.
"""

from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TG = REPO / "Trajectory Generator"
for entry in (str(REPO), str(TG)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

EXACT_STARTS = (1.756870983805102, 1.956870983805102, 2.156870983805102)
V25_PROFILE = (
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)


def run_one(task: tuple[float, str, float, int]) -> dict:
    offset, version, duration, seed = task
    import numpy as np
    from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv

    cfg = CMCEnvConfig(
        setup_xml_path="models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
        segment_duration=0.01,
        episode_duration=duration,
        episode_start_offset_s=float(offset),
        policy_knots=1,
        action_mode="delta",
        max_delta_rad=0.0,
        grf_mode="online_sensor",
        online_grf_profile_file=(
            "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
        ),
        online_grf_detector_profile_file=(
            "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
        ),
        include_online_grf_observation=True,
        online_grf_applied_sides=[],
        prescribed_grf_disabled_sides=[],
        gait_clock_enable=True,
        fail_fast=True,
        step_wall_timeout_s=60.0,
        binary_phase_fsm_mode="binary_active",
        binary_phase_detector_profile_file=V25_PROFILE,
        binary_phase_event_contract_id="binary_point_v25+heel_qualified_fsm_v2",
        binary_phase_debounce_s=0.005,
        detector_sample_dt_s=0.001,
        binary_phase_invalid_event_policy="reject_continue",
        binary_phase_actor_fsm_version=version,
        phase_min_stance_duration_s=0.3,
        phase_min_swing_duration_s=0.2,
        phase_min_stance_load_bw_s=0.04,
        phase_min_stance_contact_fraction=0.2,
        phase_min_cycle_knee_excursion_rad=0.12,
        phase_stance_hard_timeout_s=2.2,
        phase_swing_hard_timeout_s=2.6,
    )
    env = CMCLikeProsthesisTrajectoryEnv(cfg)
    out = {"offset": float(offset), "version": version}
    try:
        _obs, info = env.reset(seed=seed)
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        steps, end_reason = 0, None
        while True:
            _obs, _r, term, trunc, info = env.step(action)
            steps += 1
            if term or trunc:
                end_reason = info.get("end_reason")
                break
        pf = info.get("phase_fsm", {}) or {}
        out.update(
            steps=steps,
            end_reason=end_reason,
            valid_cycles=float(pf.get("valid_cycle_count", 0.0)),
            valid_hs=float(pf.get("valid_hs_count", 0.0)),
            invalid_events=float(pf.get("invalid_event_count", 0.0)),
            resync=float(pf.get("resync_count", 0.0)),
            hs_cancelled=float(pf.get("hs_cancelled_count", 0.0)),
            timeout=float(pf.get("timeout_exceeded", 0.0)),
            final_state=str(pf.get("state_name", "")),
        )
    except Exception as exc:  # noqa: BLE001 - report, never crash the pool
        out["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        env.close()
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n-offsets", type=int, default=20)
    ap.add_argument("--period-s", type=float, default=1.58)
    ap.add_argument("--base-offset", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", default=str(REPO / "validation" / "fsm_v3_prescribed_start_sweep.json"))
    args = ap.parse_args()
    offsets = [round(args.base_offset + k * args.period_s / args.n_offsets, 4) for k in range(args.n_offsets)]
    offsets += [round(o, 6) for o in EXACT_STARTS]
    tasks = [(o, v, args.duration, 123) for o in offsets for v in ("v2", "v3")]
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for res in pool.map(run_one, tasks):
            results.append(res)
            tag = "ERR " + res["error"][:60] if "error" in res else (
                f"steps {res['steps']:3d} end {str(res['end_reason']):26s} cycles {res['valid_cycles']:.0f} "
                f"invalid {res['invalid_events']:.0f} resync {res['resync']:.0f} cancel {res['hs_cancelled']:.0f}"
            )
            print(f"offset {res['offset']:.4f} {res['version']}: {tag}", flush=True)
    Path(args.out).write_text(json.dumps(results, indent=1))
    for v in ("v2", "v3"):
        rows = [r for r in results if r["version"] == v and "error" not in r]
        ok = sum(1 for r in rows if r["valid_cycles"] >= 1 and r["timeout"] == 0.0)
        to = sum(1 for r in rows if r["timeout"] > 0.0)
        print(f"SUMMARY {v}: starts {len(rows)} | >=1 ciclo & no timeout {ok} | timeout {to} | errori {sum(1 for r in results if r['version']==v and 'error' in r)}")
    print("receipt:", args.out)


if __name__ == "__main__":
    main()
