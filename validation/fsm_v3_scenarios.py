"""Shared scripted scenarios for the actor-FSM v3 tests and the v2 golden.

Drives ``ProstheticPhaseFSM`` directly through the binary-active V26 path
with synthetic, contract-valid events at 10 ms policy boundaries.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
TG = REPO_ROOT / "Trajectory Generator"
if str(TG) not in sys.path:
    sys.path.insert(0, str(TG))

from prosthetic_phase_fsm import (  # noqa: E402
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)

DT = 0.010


def make_fsm(**overrides: Any) -> ProstheticPhaseFSM:
    cfg = ProstheticPhaseFSMConfig(event_source="binary_active_v26", **overrides)
    return ProstheticPhaseFSM(cfg)


def event(name: str, boundary: float) -> dict[str, Any]:
    """A contract-valid left event confirmed inside (boundary-10ms, boundary]."""
    onset = round(boundary - 0.009, 6)
    return {
        "side": "left",
        "event": name,
        "event_time_s": onset,
        "confirmed_time_s": round(onset + 0.005, 6),
        "delivered_time_s": boundary,
        "source": BINARY_ACTIVE_V26_ADAPTER_SOURCE,
        "event_contract_id": BINARY_ACTIVE_V26_EVENT_CONTRACT_ID,
    }


def step(
    fsm: ProstheticPhaseFSM,
    boundary: float,
    *,
    events: tuple[dict[str, Any], ...] = (),
    contact: bool = True,
    force_bw: float = 0.8,
    binary_contact: bool | None = None,
) -> dict[str, Any]:
    kwargs: dict[str, Any] = dict(
        time_s=round(boundary, 6),
        previous_time_s=round(boundary - DT, 6),
        events=events,
        normal_force_bw=force_bw,
        in_contact=contact,
    )
    if binary_contact is not None:
        kwargs["binary_contact"] = binary_contact
    return fsm.update_from_binary_events(**kwargs)


def run_script(
    fsm: ProstheticPhaseFSM,
    script: list[tuple[str, Any]],
    *,
    start_time: float = 1.0,
    start_contact: bool = False,
    pass_binary_contact: bool = False,
) -> list[dict[str, Any]]:
    """Execute a script of ('air'|'ground', n) holds and ('hs'|'to', ...) events.

    Items: ("ground", n) / ("air", n) advance n boundaries with that contact;
    ("hs",) / ("to",) emit the event at the next boundary (contact follows the
    event: ground after hs, air after to).
    """
    fsm.reset_from_binary_baseline(time_s=start_time, in_contact=start_contact)
    t = start_time
    contact = start_contact
    payloads: list[dict[str, Any]] = []
    for item in script:
        kind = item[0]
        if kind in {"ground", "air"}:
            contact = kind == "ground"
            for _ in range(int(item[1])):
                t = round(t + DT, 6)
                payloads.append(
                    step(
                        fsm,
                        t,
                        contact=contact,
                        binary_contact=contact if pass_binary_contact else None,
                    )
                )
        elif kind in {"hs", "to"}:
            t = round(t + DT, 6)
            contact = kind == "hs"
            payloads.append(
                step(
                    fsm,
                    t,
                    events=(event("heel_strike" if kind == "hs" else "toe_off", t),),
                    contact=contact,
                    binary_contact=contact if pass_binary_contact else None,
                )
            )
        else:
            raise ValueError(kind)
    return payloads


# Golden scenario: nominal cycle, a heel bounce (HS then TO after 30 ms), an
# early-after-TO HS, a long swing, and a clean recovery cycle.
GOLDEN_SCRIPT: list[tuple[str, Any]] = [
    ("air", 20), ("hs",), ("ground", 60), ("to",), ("air", 45), ("hs",),
    ("ground", 60), ("to",), ("air", 45), ("hs",),            # valid cycle closes
    ("ground", 60), ("to",), ("air", 45),
    ("hs",), ("ground", 2), ("to",),                           # bounce: TO 30 ms after HS
    ("air", 60), ("hs",), ("ground", 60), ("to",),
    ("air", 10), ("hs",),                                       # HS too early after TO
    ("ground", 60), ("to",), ("air", 45), ("hs",), ("ground", 30),
]
