"""Procedural V9R1 compatibility surface for the frozen V9 causal corpus.

The causal relabel contract intentionally described only its 35-column actor
payload.  The reused rollout engine also audits the canonical full 35/84
feature-name layout.  This overlay exports those already frozen names without
changing the V9 target, artifacts, protocol identity, or receipts.
"""

from __future__ import annotations

from validation.h0_primary_grf_split_v8r1p1_teacher_replay_contract import (
    EXPECTED_ACTOR_FEATURE_NAMES,  # noqa: F401
    EXPECTED_OBSERVATION_FEATURE_NAMES,  # noqa: F401
)
from validation.h0_primary_split_v9_causal_teacher_contract import *  # noqa: F403


COMPATIBILITY_ID = "V9R1_ADD_CANONICAL_35_84_FEATURE_NAME_ALIASES_ONLY"
