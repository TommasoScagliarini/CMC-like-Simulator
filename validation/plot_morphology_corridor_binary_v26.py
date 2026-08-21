#!/usr/bin/env python3
"""Plot the morphology corridor anchored on the binary V26 detector events.

Successor of ``plot_morphology_corridor_two_sensor_v9.py`` for the
heel-qualified binary lineage: no detector replay is needed because V26
rollouts record the accepted left events (full event contract, 5 ms
confirmation) in ``rollout_policy_trace.json``. The corridor profile
(event-warped mean/std, radians, OpenSim model coordinate sign) is warped
over complete accepted HS-TO-HS cycles: stance maps to phase
``[0, canonical_to_phase]``, swing to ``[canonical_to_phase, 1]``.

The plotted trajectory is the policy served kinematic reference recorded by
the rollout (same raw model sign as the profile — no sign conversion).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib")
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = (
    REPO_ROOT
    / "Trajectory Generator/baseline_MLP/morphology_profiles/"
    "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
COORDS = ("pros_knee_angle", "pros_ankle_angle")
COORD_TITLES = {
    "pros_knee_angle": "Prosthetic knee — raw OpenSim sign",
    "pros_ankle_angle": "Prosthetic ankle — raw OpenSim sign (positive = dorsiflexion)",
}
STD_MULTIPLIER = 2.0


def load_sto(path: Path):
    lines = path.read_text().splitlines()
    for i, line in enumerate(lines):
        if line.startswith("time"):
            labels = line.split("\t")
            data = np.array(
                [[float(x) for x in row.split("\t")] for row in lines[i + 1 :] if row]
            )
            return labels, data
    raise ValueError(f"no header in {path}")


def left_binary_events(trace_path: Path) -> list[dict]:
    trace = json.loads(trace_path.read_text())
    events = []
    for row in trace:
        for event in row.get("online_events") or []:
            # Binary V26 events carry the full contract fields; the legacy
            # threshold stream (right side / old runs) does not.
            if event.get("side") == "left" and "delivered_time_s" in event:
                events.append(
                    {
                        "event": str(event["event"]),
                        "onset_s": float(event["event_time_s"]),
                        "confirmed_s": float(event["confirmed_time_s"]),
                        "delivered_s": float(event["delivered_time_s"]),
                    }
                )
    events.sort(key=lambda e: e["onset_s"])
    return events


def complete_cycles(events: list[dict]) -> list[tuple[float, float, float]]:
    """Return (hs, to, next_hs) onset triplets for complete accepted cycles."""
    cycles = []
    for i, event in enumerate(events):
        if event["event"] != "heel_strike":
            continue
        rest = events[i + 1 :]
        toe = next((e for e in rest if e["event"] == "toe_off"), None)
        if toe is None:
            continue
        nxt = next(
            (
                e
                for e in rest
                if e["event"] == "heel_strike" and e["onset_s"] > toe["onset_s"]
            ),
            None,
        )
        if nxt is not None:
            cycles.append((event["onset_s"], toe["onset_s"], nxt["onset_s"]))
    return cycles


def warp_corridor(profile: dict, cycle, n=300):
    """Sample corridor mean/std over one HS-TO-HS cycle in time domain."""
    hs, to, nxt = cycle
    c = float(profile["metadata"]["canonical_to_phase"])
    grid = np.asarray(profile["phase_grid"], float)
    t_stance = np.linspace(hs, to, max(8, int(n * c)))
    t_swing = np.linspace(to, nxt, max(8, int(n * (1.0 - c))))
    ph_stance = (t_stance - hs) / max(to - hs, 1e-9) * c
    ph_swing = c + (t_swing - to) / max(nxt - to, 1e-9) * (1.0 - c)
    out = {}
    for coord in COORDS:
        blob = profile["coordinates"][coord]
        mean = np.asarray(blob["mean_rad"], float)
        std = np.asarray(blob["std_rad"], float)
        out[coord] = {
            "stance": (
                t_stance,
                np.interp(ph_stance, grid, mean),
                np.interp(ph_stance, grid, std),
            ),
            "swing": (
                t_swing,
                np.interp(ph_swing, grid, mean),
                np.interp(ph_swing, grid, std),
            ),
        }
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout", required=True, help="Rollout directory")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--out", required=True, help="Output PNG path")
    parser.add_argument("--title-note", default="")
    args = parser.parse_args()

    rollout = Path(args.rollout)
    profile_path = Path(args.profile)
    profile = json.loads(profile_path.read_text())
    events = left_binary_events(rollout / "rollout_policy_trace.json")
    cycles = complete_cycles(events)

    labels, data = load_sto(
        rollout / "sim_outputs" / "rollout_episode_kinematics_reference.sto"
    )
    t = data[:, 0]
    ref = {
        coord: np.degrees(data[:, labels.index(f"{coord}_q_ref")]) for coord in COORDS
    }

    fig, axes = plt.subplots(2, 1, figsize=(20, 11), sharex=True)
    deg = np.degrees
    for ax, coord in zip(axes, COORDS):
        for k, cycle in enumerate(cycles):
            corr = warp_corridor(profile, cycle)[coord]
            for seg, color in (("stance", "tab:blue"), ("swing", "tab:orange")):
                ts, mean, std = corr[seg]
                ax.fill_between(
                    ts,
                    deg(mean - STD_MULTIPLIER * std),
                    deg(mean + STD_MULTIPLIER * std),
                    color=color,
                    alpha=0.22,
                    label=(
                        f"corridor {seg} ±{STD_MULTIPLIER:g}σ (event-warped)"
                        if k == 0
                        else None
                    ),
                )
                ax.plot(
                    ts,
                    deg(mean),
                    "--",
                    color=color,
                    lw=1.4,
                    label=f"corridor mean ({seg})" if k == 0 else None,
                )
        covered = [(c[0], c[2]) for c in cycles]
        span_lo = t[0]
        for lo, hi in covered + [(t[-1], t[-1])]:
            if lo > span_lo:
                ax.axvspan(
                    span_lo,
                    lo,
                    color="0.75",
                    alpha=0.25,
                    hatch="//",
                    lw=0,
                    label=(
                        "no complete V26 HS-TO-HS corridor"
                        if span_lo == t[0] and coord == COORDS[0]
                        else None
                    ),
                )
            span_lo = max(span_lo, hi)
        ax.plot(t, ref[coord], "k-", lw=2.0, label="policy served reference")
        first = True
        for event in events:
            color = "tab:green" if event["event"] == "heel_strike" else "tab:red"
            ax.axvline(event["onset_s"], color=color, lw=1.2, alpha=0.9,
                       label=(f"{'HS' if color == 'tab:green' else 'TO'} accepted onset"
                              if first else None))
            ax.axvline(event["confirmed_s"], color=color, lw=0.9, ls=":", alpha=0.8,
                       label=("confirmation (+5 ms)" if first else None))
            if event["event"] == "toe_off":
                first = False
        ax.set_title(COORD_TITLES[coord])
        ax.set_ylabel("Angle [deg]")
        ax.grid(alpha=0.3)
        ax.legend(loc="lower left", fontsize=9, framealpha=0.9)
    axes[-1].set_xlabel("Simulation time [s]")
    note = f" — {args.title_note}" if args.title_note else ""
    fig.suptitle(
        "Morphology corridor — binary V26 heel-qualified detector\n"
        "Accepted-FSM HS-TO-HS event warp (onsets anchor; dotted = 5 ms "
        f"confirmation); profile: {profile_path.name}{note}",
        fontsize=15,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    sidecar = {
        "rollout": str(rollout),
        "profile": str(profile_path),
        "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
        "std_multiplier": STD_MULTIPLIER,
        "left_binary_events": events,
        "complete_cycles": cycles,
        "canonical_to_phase": profile["metadata"]["canonical_to_phase"],
    }
    out.with_suffix(".json").write_text(json.dumps(sidecar, indent=2))
    print(f"salvato: {out} | cicli completi: {len(cycles)} | eventi left: {len(events)}")


if __name__ == "__main__":
    main()
