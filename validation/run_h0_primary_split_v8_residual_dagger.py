"""Run the fresh V8R1P1/V26 one-round residual-DAgger pipeline.

The numerical fit and rollout implementation is the already reviewed V6
engine, loaded through a content-pinned binding.  The binding changes only
the protocol namespace and imports: every subprocess imports this file again,
so workers cannot silently fall back to the consumed V8/V8R1 teacher artifacts,
the V6 contract, or the V6 output root.  V8R1P1 disables V6's historical
shortened ``fsm_event_rejected`` completion path; its gate accepts 500/500
steps only.

No V6 source or artifact is modified by this wrapper.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from types import FunctionType
from typing import Any


_V6_ENGINE_SHA256 = "386a43042815c1e14ce1ba3afe09f8e6b5b607694112c780ee3bd4a772ea1ec2"
_EXPECTED_TOKEN_COUNTS = {"V6": 107, "v6": 3, "V20": 1, "v20": 1}
_ENGINE_PATH = Path(__file__).with_name(
    "run_h0_primary_split_v6_residual_dagger.py"
).resolve()


def _load_bound_engine() -> tuple[dict[str, Any], dict[str, Any]]:
    raw = _ENGINE_PATH.read_bytes()
    observed_sha256 = hashlib.sha256(raw).hexdigest()
    if observed_sha256 != _V6_ENGINE_SHA256:
        raise RuntimeError(
            "refusing an unreviewed V8R1P1 engine binding: V6 source SHA drifted "
            f"({_V6_ENGINE_SHA256} != {observed_sha256})"
        )
    source = raw.decode("utf-8")
    observed_counts = {
        token: source.count(token) for token in _EXPECTED_TOKEN_COUNTS
    }
    if observed_counts != _EXPECTED_TOKEN_COUNTS:
        raise RuntimeError(
            "refusing an ambiguous V8R1P1 namespace binding: "
            f"{observed_counts!r}"
        )

    # Bind the three imports explicitly before renaming all remaining protocol
    # identifiers.  The residual contract file itself was never executed or
    # frozen under V8/V8R1, so it now carries the fresh V8R1P1 protocol/root.
    bound_source = (
        source.replace(
            "h0_primary_grf_split_v6_teacher_replay_contract",
            "h0_primary_grf_split_v8r1p1_teacher_replay_contract",
        )
        .replace(
            "h0_primary_split_v6_residual_dagger_contract",
            "h0_primary_split_v8_residual_dagger_contract",
        )
        .replace(
            "run_h0_primary_grf_split_v6_teacher_replay",
            "run_h0_primary_grf_split_v8r1p1_teacher_replay",
        )
        .replace("V6", "V8R1P1")
        .replace("v6", "v8r1p1")
        .replace("V20", "V26")
        .replace("v20", "v26")
    )
    if any(token in bound_source for token in _EXPECTED_TOKEN_COUNTS):
        raise RuntimeError("V8R1P1 engine binding left a V6/V20 namespace token")
    bound_sha256 = hashlib.sha256(bound_source.encode("utf-8")).hexdigest()
    binding = {
        "binding_id": "V8R1P1_V26_CONTENT_PINNED_V6_NUMERICAL_ENGINE_V1",
        "source_path": _ENGINE_PATH.name,
        "source_sha256": observed_sha256,
        "bound_source_sha256": bound_sha256,
        "namespace_substitutions": {
            "V6": "V8R1P1",
            "v6": "v8r1p1",
            "V20": "V26",
            "v20": "v26",
        },
        "partial_fsm_rejection_acceptance": False,
    }
    namespace: dict[str, Any] = {
        "__builtins__": __builtins__,
        "__file__": str(Path(__file__).resolve()),
        "__name__": "_h0_primary_split_v8r1p1_bound_engine",
        "__package__": None,
    }
    exec(compile(bound_source, str(Path(__file__).resolve()), "exec"), namespace)

    teacher_collector = namespace.get("teacher_collector")
    teacher_engine = getattr(teacher_collector, "engine", None)
    if teacher_collector is None or teacher_engine is None or not hasattr(
        teacher_engine, "_sea_fallback_count"
    ):
        raise RuntimeError("V8R1P1 teacher collector lacks the pinned SEA audit helper")
    # The V8R1P1 teacher entry point intentionally exposes its V6 numerical engine
    # as ``engine``.  The residual engine historically calls the helper on the
    # collector module itself, so bind that one read-only audit helper locally.
    setattr(
        teacher_collector,
        "_sea_fallback_count",
        teacher_engine._sea_fallback_count,
    )

    # V6 preserved a forensic partial-rollout classifier for the V20 failure.
    # V8R1P1/V26 has no such accepted terminal mode: any runtime exception escapes
    # the worker and closes the one-shot stage as FAIL.
    def _never_accept_partial_fsm_rejection(_error: BaseException) -> bool:
        return False

    namespace["_is_fsm_event_rejection"] = _never_accept_partial_fsm_rejection

    # Make the content-pinned implementation visible in the execution claim.
    # All downstream artifacts already content-address that claim.
    original_claim_payload = namespace["_claim_payload"]
    if not isinstance(original_claim_payload, FunctionType):
        raise RuntimeError("V8R1P1 bound engine lacks the execution-claim builder")

    def _v8_claim_payload(token_sha256: str) -> dict[str, Any]:
        payload = dict(original_claim_payload(token_sha256))
        payload["implementation_binding"] = dict(binding)
        return payload

    namespace["_claim_payload"] = _v8_claim_payload
    return namespace, binding


_BOUND, IMPLEMENTATION_BINDING = _load_bound_engine()

# Export the bound implementation for focused tests and normal CLI use.  The
# original module remains independently importable and byte-for-byte intact.
for _name, _value in _BOUND.items():
    if not _name.startswith("__"):
        globals()[_name] = _value


if __name__ == "__main__":
    raise SystemExit(_BOUND["main"]())
