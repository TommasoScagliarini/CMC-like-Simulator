"""F0 activity 6: synchronized overlays of the 58 analysed rollouts (fail-closed, no-replace publish, byte-bound data inputs).

Scope of the guarantee: DATA inputs are byte-bound (declared and hashed BEFORE any computational
read, computed from the same bytes, re-hashed POST and every declared input consumed before the
publish); Python SOURCE is imported before the runtime ledger, so its recorded hashes attest the
files on disk, not the bytecode already imported - no code-execution TOCTOU freedom is claimed.
The ledger is content-addressed (bytes, not inode/path identity). The POSIX no-follow openat
chain refuses symlink ancestors atomically; on Windows the fallback is lstat-then-open and the
ancestor no-follow check is NOT atomic (compatible, weaker).

Protocol of ``run`` (see f0_artifacts for the ledger/publish contract): discovery = PURE registry
projection of the canonical jobs (``static_job_record`` performs no filesystem read); the
consumed-file set is enumerated STATICALLY with no byte read: final
analysis, receipt / summary / trace / 7 .sto per job (fixed names under the registry output
dir), the runtime configs of the registry (``RUNTIMES``), the registry morphology profiles
(``MORPH_PROFILES``), the pinned AB06 event-warped phase profile, the F0 scripts/helpers ->
PRE snapshot of every declared input BEFORE any computational read -> every read goes through
the ledger and is byte-bound to the PRE digest (re-verified POST; the ledger is not a cache):
the exact canonical 58-job index is validated; consumed receipt fields, summary, trace, config
and every STO already analysed are bound to the corresponding evidence in the pinned analysis;
the extra SEA-state STO is explicitly overlay-only and remains ledger-bound; the receipt config
must be the registry config, summary / trace / config digests equal to the receipt, a config
naming a profile outside the declared set fails closed -> consumed set re-hashed after loading,
every declared input read before the publish -> figures/CSVs in a staging sibling, single sidecar
last, inputs re-hashed and set-complete immediately before the staged verification that
precedes the NO-REPLACE rename, published tree verified (failure raises -> nonzero exit).

Groups (GROUP_SPECS): G1 isometric per start (B0820_H0 reference + V2_BEST / V3_BEST / V3_LAST
under v3; rep2 identity check recorded), G2 controls (JUL_H0 v3, JUL_BEST v3, V26_39D
compatibility control: dashed, descriptive, TIME-ALIGNED ONLY by declaration - a phase figure is
never produced for G2 even if every member has cycles), G3 runtime effect (same weights under
two runtimes, never fused with other chains), G4 stochastic (deterministic thick + seeds
123/124/125 as full individual thin lines + min/max band over the replicas on the common span
[0, min end] only with identical validated grids, stated in the manifest; the replay JUL_H0
july_legacy +0.20 seed 123 is an individual dotted control, never banded).

Alignment: time-aligned on the verified 1 kHz grid (each trajectory to its own end
marker/reason, no truncation). Cycles are EXCLUSIVELY: an accepted opening heel strike -> an
accepted toe-off (segment_valid == 1, segment_start == the opening HS) -> accepted
closing heel strike with cycle_valid == 1; accepted HS/TO times finite and strictly increasing;
an HS cancellation resets the chain (and carries the time of the HS it cancels); rejected/invalid
sensor events are recorded but never enter or rewrite the accepted chain; the extracted count must equal BOTH the trace's final
``valid_cycle_count`` and the summary ``phase_valid_cycle_count`` (mandatory non-negative
integers); a mismatch or a runtime-valid cycle without that strict chain disables every phase
product for the job while preserving the independently verified time-domain outputs and records
the contract error. Phase normalisation is PIECEWISE EVENT-WARPED HS->TO->HS with the
canonical_to_phase alpha of the pinned AB06 event-warped morphology profile (stance HS->TO
mapped linearly to [0, alpha], swing TO->HS to [alpha, 1]); the profile is a consumed input
and a missing/invalid profile fails the phase products closed. Phase interpolation = linear
resampling of CONTINUOUS 1 kHz signals only on 101 phase points per cycle; counters / events
are never interpolated. A job with zero valid cycles is ``insufficient_cycles``; a GROUP gets a
phase figure only when phase alignment is allowed by declaration AND every member has >= 1
accepted cycle, otherwise ``group_phase_status`` is ``TIME_ALIGNED_ONLY_BY_DECLARATION`` or
``NOT_PHASE_ALIGNABLE_ZERO_VALID_CYCLES`` (zero-cycle members listed).

Signals: raw / applied / mean action (+ exploration noise), served reference = q_ref of
``rollout_episode_kinematics_reference.sto`` (``policy_segment_*`` diagnostic only), measured
knee/ankle, SEA spring AND motor torques, SEA motor states, ``sea_controls`` = u normalised,
left GRF normal / contact / penetration with thresholds, FSM events and legacy events, EVERY
reserve torque column (declared column list) + norm + max-abs envelope, causal morphology
ledger counters, the morphology corridor exactly as recorded by the reward on settled served
samples with the pinned config/profile provenance. Every number taken from trace / events /
corridor / .sto is validated finite (NaN and +-Inf refused) before any CSV / JSON / plot.
Config/profile paths are resolved only under the approved roots (repo root, baseline_MLP) with
no traversal / escape / symlink ancestor.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_artifacts as ART  # noqa: E402
import f0_common as C  # noqa: E402
import f0_matrix_analysis as A  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402
from f0_replay_analysis import column  # noqa: E402

FINAL_ANALYSIS_FILE = "f0_matrix_analysis_20260823_011338.json"
FINAL_ANALYSIS_SHA256 = "0ac942432d0505731b93e6322ac337edf91ab7dca8c9c8e9c1cb122089148987"
PHASE_PROFILE = C.MORPH_PROFILE_EVENT_WARPED
PHASE_PARAMETERIZATION = "event_warped_hs_to_to_to_hs_v1"
SEA_STATES_FILE = "rollout_episode_sea_states.sto"
PHASE_POINTS = 101
PHASE_INTERPOLATION = "piecewise event-warped HS->TO->HS phase (stance HS->TO -> [0, alpha], swing TO->HS -> [alpha, 1], alpha = canonical_to_phase of the pinned AB06 event-warped profile); linear resampling (numpy.interp) of CONTINUOUS 1 kHz signals only on 101 phase points per accepted cycle; counters/events/discrete signals never interpolated; no min-length truncation, no implicit time grid"
PHASE_ALL = "PHASE_ALIGNED_ALL_MEMBERS"
PHASE_NONE = "NOT_PHASE_ALIGNABLE_ZERO_VALID_CYCLES"
PHASE_DECLARED_TIME_ONLY = "TIME_ALIGNED_ONLY_BY_DECLARATION"
STARTS = ("minus020", "nominal", "plus020")
ISOMETRIC_CANDIDATES = ("B0820_H0", "B0820_V2_BEST", "B0820_V3_BEST", "B0820_V3_LAST")
STOCH_CANDIDATES = ("B0820_H0", "B0820_V3_BEST", "JUL_H0")
COLORS = {"B0820_H0": "#1f77b4", "B0820_V2_BEST": "#2ca02c", "B0820_V3_BEST": "#d62728", "B0820_V3_LAST": "#9467bd", "JUL_H0": "#ff7f0e", "JUL_BEST": "#8c564b", "V26_39D": "#7f7f7f"}
MORPH_TERMS = ("morphology_available", "morphology_settled_this_step", "morphology_phase", "fsm_morphology_phase", "morphology_knee_value_rad", "morphology_knee_min_rad", "morphology_knee_max_rad", "morphology_ankle_value_rad", "morphology_ankle_min_rad", "morphology_ankle_max_rad", "morphology_knee_loss", "morphology_ankle_loss", "morphology_phase_mode_id", "morphology_phase_source_id")
FSM_COUNTERS = ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count", "hs_cancelled_count")
CAUSAL_COUNTERS = ("total_cancelled_transition_count", "total_resolved_sample_count", "total_dropped_sample_count", "timeout_transition_count", "pending_sample_count", "resolved_sample_count", "dropped_sample_count")
PHASE_SERIES = ("knee_measured_rad", "ankle_measured_rad", "knee_served_ref_rad", "ankle_served_ref_rad", "u_knee", "u_ankle", "tau_spring_knee_nm", "tau_spring_ankle_nm", "tau_motor_knee_nm", "tau_motor_ankle_nm", "grf_left_normal_n")
BAND_SERIES = ("knee_measured_rad", "knee_served_ref_rad", "ankle_measured_rad", "ankle_served_ref_rad", "u_knee", "u_ankle", "tau_spring_knee_nm", "tau_spring_ankle_nm", "tau_motor_knee_nm", "tau_motor_ankle_nm", "grf_left_normal_n", "grf_left_penetration_m", "reserve_norm_nm", "reserve_abs_max_envelope_nm")
CODE_INPUTS = ("f0_common.py", "f0_rollout_matrix.py", "f0_matrix_analysis.py", "f0_replay_analysis.py", "f0_closure.py", "f0_artifacts.py", "f0_overlays.py")
APPROVED_ROOTS = (C.REPO, C.BASELINE_DIR)
STO_FILES = (*A.EXPECTED_STO, SEA_STATES_FILE)
MORPH_PROFILES = (C.MORPH_PROFILE_EVENT_WARPED, C.MORPH_PROFILE_LEGACY)  # registry profiles a pinned config may name (static, no read)


class OverlayError(RuntimeError):
    """Fail-closed error of the overlay protocol (provenance, schema, grids, cycles, corridor consistency, finiteness)."""


class PhaseAlignmentError(OverlayError):
    """The recorded FSM cycle claim cannot support a trustworthy phase product."""


def load_final_analysis(path: Path | None = None, expected_sha: str | None = FINAL_ANALYSIS_SHA256, *, reader: ART.DirectReader = ART.DIRECT) -> tuple[dict[str, Any], dict[str, Any]]:
    path = Path(path) if path is not None else C.OUT_METRICS / FINAL_ANALYSIS_FILE
    if path.is_symlink() or not path.is_file():
        raise OverlayError(f"final analysis artefact missing: {C.rel(path)}")
    raw = reader.read_bytes(path)
    sha = ART.sha256_bytes(raw)
    if expected_sha is not None and sha != expected_sha:
        raise OverlayError(f"final analysis artefact digest mismatch: {sha} != pinned {expected_sha}")
    payload = ART.loads_strict(raw)
    if not isinstance(payload, dict) or payload.get("analysis_complete") is not True:
        raise OverlayError("final analysis must be complete before overlays")
    return payload, {"path": C.rel(path), "sha256": sha}


# --- phase profile ---------------------------------------------------------------------------------


def phase_profile_alpha(profile_path: Path = PHASE_PROFILE, *, reader: ART.DirectReader = ART.DIRECT) -> dict[str, Any]:
    """canonical_to_phase alpha of the pinned event-warped profile (fail-closed on missing/invalid)."""
    profile_path = Path(profile_path)
    if profile_path.is_symlink() or not profile_path.is_file():
        raise OverlayError(f"pinned phase profile missing: {profile_path}")
    raw = reader.read_bytes(profile_path)
    prof = ART.loads_strict(raw)
    meta = prof.get("metadata") if isinstance(prof, dict) else None
    if not isinstance(meta, dict) or meta.get("phase_parameterization") != PHASE_PARAMETERIZATION:
        raise OverlayError(f"phase profile is not {PHASE_PARAMETERIZATION}: {profile_path}")
    alpha = meta.get("canonical_to_phase")
    if not A.is_finite_float(alpha) or not (0.0 < float(alpha) < 1.0):
        raise OverlayError(f"phase profile canonical_to_phase invalid: {alpha!r}")
    return {"path": C.rel(profile_path), "sha256": ART.sha256_bytes(raw), "alpha": float(alpha), "parameterization": meta.get("phase_parameterization")}


def phase_times(cycle: dict[str, Any], grid: np.ndarray, alpha: float) -> np.ndarray:
    """Piecewise event-warped mapping phase -> time: [0, alpha] = stance HS->TO, [alpha, 1] = swing TO->HS."""
    hs0, to, hs1 = float(cycle["hs_start_s"]), float(cycle["toe_off_s"]), float(cycle["hs_end_s"])
    if not (hs0 < to < hs1) or not (0.0 < alpha < 1.0):
        raise OverlayError(f"invalid cycle or alpha for phase mapping: {cycle} alpha={alpha}")
    grid = np.asarray(grid, dtype=np.float64)
    return np.where(grid <= alpha, hs0 + (grid / alpha) * (to - hs0), to + ((grid - alpha) / (1.0 - alpha)) * (hs1 - to))


# --- per-job data -------------------------------------------------------------------------------


def as_int_or_none(v: Any) -> int | None:
    return A.as_int(v) if A.is_nonneg_int(v) else None


def _flag(tr: dict[str, Any], key: str, step: Any) -> int | None:
    v = tr.get(key)
    if v is None:
        return None
    if not A.is_finite_float(v):
        raise OverlayError(f"non-finite FSM flag {key} at step {step}: {v!r}")
    fv = float(v)
    if not fv.is_integer() or int(fv) not in (-1, 0, 1):
        raise OverlayError(f"invalid FSM flag {key} at step {step}: {v!r}")
    return int(fv)


def cycles_from_trace(rows: list[dict[str, Any]], *, summary_cycle_count: int | None = None, trace_final_count: int | None = None) -> dict[str, Any]:
    """Valid cycles are EXCLUSIVELY: accepted opening HS -> accepted TO (segment_valid, segment_start == HS) ->
    accepted closing HS with cycle_valid == 1. Accepted HS/TO events must carry finite, STRICTLY increasing times;
    a ``heel_strike_cancelled`` record (which legitimately carries the time of the HS it cancels) resets the chain
    and never invalidates the parser. The count must equal the trace's final ``valid_cycle_count`` and the summary."""
    cycles: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    anomalies: list[dict[str, Any]] = []
    open_hs: float | None = None
    to_time: float | None = None
    last_chain = -np.inf  # last accepted HS/TO time (strictly increasing)
    for r in rows:
        step = as_int_or_none(r.get("step"))
        fsm = r.get("phase_fsm")
        for tr in (fsm.get("accepted_transitions_this_step") if isinstance(fsm, dict) else None) or []:
            event = tr.get("event")
            et = tr.get("event_time_s")
            if not A.is_finite_float(et):
                raise OverlayError(f"accepted transition without finite event_time_s at step {step}")
            et = float(et)
            seg_start = tr.get("segment_start_time_s")
            if seg_start is not None and not A.is_finite_float(seg_start):
                raise OverlayError(f"non-finite segment_start_time_s at step {step}")
            if event == "heel_strike_cancelled":
                if et < last_chain:
                    anomalies.append({"step": step, "kind": "cancellation_time_before_last_accepted_event", "event_time_s": et})
                open_hs, to_time = None, None
                continue
            if event in ("toe_off", "heel_strike"):
                if not et > last_chain:
                    raise OverlayError(f"accepted HS/TO events must be strictly increasing in time at step {step}: {et} <= {last_chain}")
                last_chain = et
            if event == "toe_off":
                if open_hs is None:
                    anomalies.append({"step": step, "kind": "toe_off_without_accepted_opening_hs", "event_time_s": et})
                    to_time = None
                elif _flag(tr, "segment_valid", step) != 1 or seg_start is None or abs(float(seg_start) - open_hs) > 1e-9 or not et > open_hs:
                    anomalies.append({"step": step, "kind": "toe_off_segment_invalid_or_inconsistent", "event_time_s": et})
                    open_hs, to_time = None, None
                else:
                    if to_time is not None:
                        anomalies.append({"step": step, "kind": "toe_off_repeated_without_heel_strike", "event_time_s": et})
                        open_hs, to_time = None, None
                    else:
                        to_time = et
            elif event == "heel_strike":
                cv = _flag(tr, "cycle_valid", step)
                if cv == 1:
                    if open_hs is None or to_time is None or not et > to_time or _flag(tr, "segment_valid", step) != 1:
                        raise PhaseAlignmentError(f"accepted valid cycle at step {step} without the required accepted HS -> TO chain (open_hs={open_hs}, to={to_time})")
                    if seg_start is not None and abs(float(seg_start) - to_time) > 1e-9:
                        raise PhaseAlignmentError(f"closing heel strike at step {step} does not close the accepted swing (segment_start {seg_start} != toe_off {to_time})")
                    cycles.append({"step": step, "hs_start_s": open_hs, "toe_off_s": to_time, "hs_end_s": et, "duration_s": et - open_hs, "stance_s": to_time - open_hs, "swing_s": et - to_time})
                elif tr.get("cycle_reject_reason"):
                    rejected.append({"step": step, "reason": tr.get("cycle_reject_reason"), "event_time_s": et})
                if _flag(tr, "segment_valid", step) != 1:
                    open_hs, to_time = None, None
                else:
                    open_hs, to_time = et, None
            else:
                anomalies.append({"step": step, "kind": f"unknown_event:{event}", "event_time_s": et})
    if trace_final_count is not None and len(cycles) != trace_final_count:
        raise PhaseAlignmentError(f"extracted accepted cycles {len(cycles)} != trace final valid_cycle_count {trace_final_count}")
    if summary_cycle_count is not None and len(cycles) != summary_cycle_count:
        raise PhaseAlignmentError(f"extracted accepted cycles {len(cycles)} != summary phase_valid_cycle_count {summary_cycle_count}")
    return {"cycles": cycles, "rejected_cycles": rejected, "anomalies": anomalies, "valid_cycle_count": len(cycles), "summary_cycle_count": summary_cycle_count, "trace_final_count": trace_final_count, "phase_aligned": "accepted_fsm_cycles" if cycles else "insufficient_cycles", "cycle_definition": "accepted HS -> accepted TO (segment_valid, segment_start == HS) -> accepted HS with cycle_valid == 1; accepted HS/TO times finite and strictly increasing; cancellations reset the chain; count == trace final valid_cycle_count == summary phase_valid_cycle_count", "phase_interpolation": PHASE_INTERPOLATION}


def cycles_for_overlay(rows: list[dict[str, Any]], *, summary_cycle_count: int, trace_final_count: int) -> dict[str, Any]:
    """Fail the phase product closed while preserving independently valid time-domain signals.

    A runtime may increment its cycle counter after a cancelled HS even though the strict
    accepted HS->TO->HS chain no longer exists. Such a job remains useful time-aligned, but it
    must never be phase-normalised or silently credited with the runtime-reported cycle.
    """
    try:
        return cycles_from_trace(rows, summary_cycle_count=summary_cycle_count, trace_final_count=trace_final_count)
    except PhaseAlignmentError as exc:
        return {
            "cycles": [], "rejected_cycles": [], "anomalies": [], "valid_cycle_count": 0,
            "summary_cycle_count": summary_cycle_count, "trace_final_count": trace_final_count,
            "phase_aligned": "invalid_fsm_cycle_contract", "phase_contract_error": str(exc),
            "cycle_definition": "accepted HS -> accepted TO (segment_valid, segment_start == HS) -> accepted HS with cycle_valid == 1; cancellation resets the chain; an inconsistent runtime cycle claim disables phase products but not verified time-domain outputs",
            "phase_interpolation": PHASE_INTERPOLATION,
        }


def events_from_trace(rows: list[dict[str, Any]], t0: float) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for r in rows:
        step = as_int_or_none(r.get("step"))
        fsm = r.get("phase_fsm") if isinstance(r.get("phase_fsm"), dict) else {}
        rt = r.get("time")
        if not A.is_finite_float(rt):
            raise OverlayError(f"non-finite trace time at step {step}")
        rt = float(rt)
        for tr in fsm.get("accepted_transitions_this_step") or []:
            kind = "fsm_hs_cancelled" if tr.get("event") == "heel_strike_cancelled" else f"fsm_accepted_{tr.get('event')}"
            et = tr.get("event_time_s")
            if not A.is_finite_float(et):
                raise OverlayError(f"non-finite event_time_s at step {step}")
            events.append({"step": step, "time_s": float(et), "t_rel_s": float(et) - t0, "kind": kind, "source": "actor_fsm", "detail": f"cycle_valid={tr.get('cycle_valid')} segment_valid={tr.get('segment_valid')} reject={tr.get('cycle_reject_reason') or ''}"})
        for key, kind in (("invalid_event_this_step", f"fsm_rejected:{fsm.get('invalid_event_type') or 'unknown'}"), ("resync_event_this_step", "fsm_resync"), ("cycle_rejected_this_step", f"fsm_cycle_rejected:{fsm.get('cycle_reject_reason') or 'unknown'}")):
            v = fsm.get(key, 0)
            if v is not None and not A.is_finite_float(v):
                raise OverlayError(f"non-finite FSM flag {key} at step {step}")
            if v and float(v) > 0:
                events.append({"step": step, "time_s": rt, "t_rel_s": rt - t0, "kind": kind, "source": "actor_fsm", "detail": f"loss={fsm.get('invalid_event_loss')}" if key == "invalid_event_this_step" else (f"resync_count={fsm.get('resync_count')}" if key == "resync_event_this_step" else "")})
        for ev in fsm.get("sensor_events_this_step") or []:
            events.append({"step": step, "time_s": rt, "t_rel_s": rt - t0, "kind": f"sensor:{ev}", "source": "binary_sensors", "detail": ""})
        for ev in r.get("legacy_online_events") or []:
            if isinstance(ev, dict):
                lt = ev.get("time")
                if lt is not None and not A.is_finite_float(lt):
                    raise OverlayError(f"non-finite legacy event time at step {step}")
                events.append({"step": step, "time_s": float(lt) if lt is not None else None, "t_rel_s": float(lt) - t0 if lt is not None else None, "kind": f"legacy_{ev.get('side')}_{ev.get('event')}", "source": "legacy_detector (informative)", "detail": f"confirmed={ev.get('confirmed_time')}"})
    ART.assert_finite_tree(events, "events")
    return events


def _profile_of(cfg: dict[str, Any]) -> tuple[dict[str, Any], str | None]:
    reward_cfg = cfg.get("reward") if isinstance(cfg.get("reward"), dict) else {}
    mp = reward_cfg.get("morphology_profile")
    return reward_cfg, (mp if isinstance(mp, str) and mp.strip() else None)


def job_consumed_files(rec: dict[str, Any], *, roots: tuple[Path, ...] = APPROVED_ROOTS) -> dict[str, Any]:
    """STATIC enumeration (no byte read) of the files a job load will consume: fixed output names under the
    registry output dir and the registry config of the described job. Morphology profiles are declared from the
    registry constants (MORPH_PROFILES) by the caller: a config naming any other profile fails closed at read."""
    out_dir = C.REPO / rec["output_dir"]
    if not isinstance(rec.get("config"), str):
        raise OverlayError(f"{rec['job_id']}: described job without registry config")
    config_path = ART.resolve_under_roots(rec["config"], roots, f"{rec['job_id']} registry config")
    files = [out_dir / M.RECEIPT_FILE, out_dir / M.SUMMARY_FILE, out_dir / M.TRACE_FILE, config_path, *[out_dir / "sim_outputs" / f for f in STO_FILES]]
    return {"files": files, "config_path": config_path}


def load_job(rec: dict[str, Any], *, analysis_job: dict[str, Any] | None = None, verify=M.verify_existing, alpha: float | None = None, reader: ART.DirectReader = ART.DIRECT, roots: tuple[Path, ...] = APPROVED_ROOTS) -> dict[str, Any]:
    """Consume one job fail-closed.

    Production supplies the matching job from the pinned analysis and disables the live F0
    verifier: the receipt projection plus summary/trace/config/analysed-STO digests are then
    bound directly to that immutable authority. ``verify`` remains a direct-helper/test hook.
    """
    jid = rec["job_id"]
    out_dir = C.REPO / rec["output_dir"]
    receipt_raw = reader.read_bytes(out_dir / M.RECEIPT_FILE)
    receipt = ART.loads_strict(receipt_raw)
    if not isinstance(receipt, dict) or not isinstance(receipt.get("config"), str):
        raise OverlayError(f"{jid}: receipt is not a mapping with a pinned config")
    if verify is not None:
        verified = verify(rec, out_dir)
        if verified != receipt:
            raise OverlayError(f"{jid}: receipt verified on disk differs from the receipt bytes consumed (TOCTOU)")
    summary_raw = reader.read_bytes(out_dir / M.SUMMARY_FILE)
    trace_raw = reader.read_bytes(out_dir / M.TRACE_FILE)
    s_sha, t_sha = ART.sha256_bytes(summary_raw), ART.sha256_bytes(trace_raw)
    if receipt.get("summary_sha256") != s_sha or receipt.get("trace_sha256") != t_sha:
        raise OverlayError(f"{jid}: summary/trace bytes consumed do not match the receipt digests")
    summary = ART.loads_strict(summary_raw)
    rows = ART.loads_strict(trace_raw)
    if not isinstance(summary, dict) or not isinstance(rows, list) or not rows or not all(isinstance(r, dict) for r in rows):
        raise OverlayError(f"{jid}: malformed summary or empty trace")
    if receipt["config"] != rec.get("config"):
        raise OverlayError(f"{jid}: receipt config {receipt['config']!r} is not the registry config {rec.get('config')!r}")
    config_path = ART.resolve_under_roots(receipt["config"], roots, f"{jid} pinned config")
    cfg_raw = reader.read_bytes(config_path)
    cfg_sha = ART.sha256_bytes(cfg_raw)
    if receipt.get("config_sha256") != cfg_sha:
        raise OverlayError(f"{jid}: pinned config bytes consumed do not match the receipt config_sha256")
    import yaml  # noqa: PLC0415

    cfg = yaml.safe_load(cfg_raw.decode("utf-8")) or {}
    reward_cfg, profile_rel = _profile_of(cfg)
    stos: dict[str, tuple[list[str], np.ndarray]] = {}
    sto_sha: dict[str, str] = {}
    for f in STO_FILES:
        raw = reader.read_bytes(out_dir / "sim_outputs" / f)
        sto_sha[f] = ART.sha256_bytes(raw)
        stos[f] = ART.parse_sto(raw.decode("utf-8", errors="replace"))
    analysis_binding: dict[str, Any]
    if analysis_job is not None:
        identity_keys = ("job_id", "family", "comparison_class", "candidate", "runtime", "start", "action_selection", "seed", "repeat", "output_dir", "historical_reference_summary")
        if {k: analysis_job.get(k) for k in identity_keys} != {k: rec.get(k) for k in identity_keys}:
            raise OverlayError(f"{jid}: pinned-analysis identity differs from the static registry record")
        expected_receipt = analysis_job.get("receipt")
        receipt_keys = [k for k in expected_receipt] if isinstance(expected_receipt, dict) else []
        receipt_keys = [k for k in receipt_keys if k != "provenance_class_effective"]  # derived by matrix analysis, absent from the receipt file
        if not isinstance(expected_receipt, dict) or {k: receipt.get(k) for k in receipt_keys} != {k: expected_receipt[k] for k in receipt_keys}:
            raise OverlayError(f"{jid}: consumed receipt projection differs from the pinned analysis")
        evidence = analysis_job.get("evidence")
        if not isinstance(evidence, dict):
            raise OverlayError(f"{jid}: pinned analysis has no evidence mapping")
        current = {"summary": s_sha, "trace": t_sha, **{f: sto_sha[f] for f in A.EXPECTED_STO}}
        expected_paths = {
            "summary": C.rel(out_dir / M.SUMMARY_FILE),
            "trace": C.rel(out_dir / M.TRACE_FILE),
            **{f: C.rel(out_dir / "sim_outputs" / f) for f in A.EXPECTED_STO},
        }
        for name, sha in current.items():
            item = evidence.get(name)
            if not isinstance(item, dict) or item.get("path") != expected_paths[name] or item.get("present") is not True or item.get("valid") is not True or item.get("sha256") != sha:
                raise OverlayError(f"{jid}: consumed {name} bytes differ from the pinned-analysis evidence")
        provenance = analysis_job.get("provenance")
        if not isinstance(provenance, dict) or provenance.get("config") != rec.get("config") or provenance.get("config_sha256") != cfg_sha:
            raise OverlayError(f"{jid}: consumed config differs from pinned-analysis provenance")
        analysis_binding = {
            "mode": "pinned_analysis_exact_evidence",
            "receipt_projection": "deep_equal_except_analysis-derived provenance_class_effective",
            "summary_trace_and_analysed_sto": "sha256_equal",
            "analysed_evidence_sha256": current,
            "overlay_only_inputs": {SEA_STATES_FILE: {"sha256": sto_sha[SEA_STATES_FILE], "classification": "not present in source analysis; overlay-only, PRE/POST ledger-bound"}},
        }
    else:
        analysis_binding = {"mode": "direct_helper_unbound_to_analysis"}
    k_names, k_data = stos[A.KINEMATICS_FILE]
    t = column(k_names, k_data, "time")
    if t is None:
        raise OverlayError(f"{jid}: kinematics without time column")
    A.validate_grid(t, jid)
    for f, (names, data) in stos.items():
        if f != A.KINEMATICS_FILE:
            tb = column(names, data, "time")
            if tb is None:
                raise OverlayError(f"{jid}: {f} without time column")
            A.verify_grid(t, tb, f"{jid} {f}")
    t0 = float(t[0])
    sea_names, sea_data = stos[SEA_STATES_FILE]
    sea_state_cols = [n for n in sea_names if "motor" in n.lower()]
    if not sea_state_cols:
        raise OverlayError(f"{jid}: no SEA motor state column in {SEA_STATES_FILE}: {sea_names}")
    rs_names, rs_data = stos[A.RESERVE_FILE]
    reserve_cols = [n for n in rs_names if n.endswith(A.RESERVE_COLUMN_SUFFIX)]
    if not reserve_cols or "pros_knee_angle_reserve_torque" not in reserve_cols or "pros_ankle_angle_reserve_torque" not in reserve_cols:
        raise OverlayError(f"{jid}: reserve torque columns incomplete: {reserve_cols}")
    block = rs_data[:, [rs_names.index(n) for n in reserve_cols]]
    g_names, g_data = stos[A.GRF_FILE]
    r_names, r_data = stos[A.REFERENCE_FILE]
    u_names, u_data = stos[A.SEA_CONTROLS_FILE]
    tq_names, tq_data = stos[A.SEA_TORQUES_FILE]
    khz = {"t_rel_s": t - t0, "time_s": t, "knee_measured_rad": column(k_names, k_data, "pros_knee_angle"), "ankle_measured_rad": column(k_names, k_data, "pros_ankle_angle"), "knee_served_ref_rad": column(r_names, r_data, "pros_knee_angle_q_ref"), "ankle_served_ref_rad": column(r_names, r_data, "pros_ankle_angle_q_ref"),
           "u_knee": column(u_names, u_data, "pros_knee_angle"), "u_ankle": column(u_names, u_data, "pros_ankle_angle"), "tau_spring_knee_nm": column(tq_names, tq_data, "SEA_Knee_tau_spring"), "tau_spring_ankle_nm": column(tq_names, tq_data, "SEA_Ankle_tau_spring"), "tau_motor_knee_nm": column(tq_names, tq_data, "SEA_Knee_tau_motor"), "tau_motor_ankle_nm": column(tq_names, tq_data, "SEA_Ankle_tau_motor"),
           "grf_left_normal_n": column(g_names, g_data, "left_normal_force"), "grf_left_penetration_m": column(g_names, g_data, "left_penetration"), "grf_left_in_contact": column(g_names, g_data, "left_in_contact"), "reserve_norm_nm": np.sqrt(np.sum(np.square(block), axis=1)), "reserve_abs_max_envelope_nm": np.max(np.abs(block), axis=1)}
    for n in reserve_cols:
        khz[f"reserve__{n}"] = column(rs_names, rs_data, n)
    for n in sea_state_cols:
        khz[f"sea_state__{n}"] = column(sea_names, sea_data, n)
    for k, v in khz.items():
        if v is None:
            raise OverlayError(f"{jid}: required 1 kHz column missing for {k}")
        if not np.all(np.isfinite(v)):
            raise OverlayError(f"{jid}: non-finite 1 kHz series {k}")
    steps: list[dict[str, Any]] = []
    required = ("step", "time", "raw_policy_action", "applied_policy_action", "phase_fsm", "morphology_causal_diagnostics", "reward_terms", "policy_segment_values")
    for r in rows:
        missing = [k for k in required if k not in r]
        if missing:
            raise OverlayError(f"{jid}: trace row {r.get('step')} lacks {missing}")
        fsm, cz, rt, seg = r["phase_fsm"], r.get("morphology_causal_diagnostics") or {}, r["reward_terms"], r["policy_segment_values"]
        if not isinstance(fsm, dict) or not isinstance(rt, dict) or not isinstance(r["raw_policy_action"], list) or len(r["raw_policy_action"]) < 2 or not isinstance(r["applied_policy_action"], list) or len(r["applied_policy_action"]) < 2:
            raise OverlayError(f"{jid}: trace row {r.get('step')} malformed")
        mean, noise = r.get("policy_action_mean"), r.get("exploration_action_noise")
        row = {"step": A.as_int(r["step"]), "time_s": float(r["time"]), "t_rel_s": float(r["time"]) - t0, "raw_knee": float(r["raw_policy_action"][0]), "raw_ankle": float(r["raw_policy_action"][1]), "applied_knee": float(r["applied_policy_action"][0]), "applied_ankle": float(r["applied_policy_action"][1]),
               "mean_knee": float(mean[0]) if isinstance(mean, list) else None, "mean_ankle": float(mean[1]) if isinstance(mean, list) else None, "noise_knee": float(noise[0]) if isinstance(noise, list) else None, "noise_ankle": float(noise[1]) if isinstance(noise, list) else None,
               "policy_segment_end_knee_rad_diagnostic": float(seg[-1][0]) if seg else None, "policy_segment_end_ankle_rad_diagnostic": float(seg[-1][1]) if seg else None, "cycle_progress_credit": fsm.get("cycle_progress_credit"), "pending_cycle_credit": fsm.get("pending_cycle_credit"), "fsm_state_expected_next": fsm.get("expected_next_event")}
        for k in FSM_COUNTERS:
            row[f"fsm_{k}"] = as_int_or_none(fsm.get(k))
        for k in CAUSAL_COUNTERS:
            row[f"causal_{k}"] = as_int_or_none(cz.get(k)) if cz else None
        row["causal_actor_state"] = cz.get("actor_state_name") if cz else None
        for k in MORPH_TERMS:
            row[k] = rt.get(k)
        steps.append(row)
    try:
        ART.assert_finite_tree(steps, f"{jid} steps")
    except ART.SerializationError as exc:
        raise OverlayError(f"{jid}: {exc}") from exc
    morph_available = [s for s in steps if A.is_finite_float(s.get("morphology_available")) and float(s["morphology_available"]) >= 1.0]
    weight = reward_cfg.get("morphology_weight")
    corridor: dict[str, Any] = {"phase_mode": reward_cfg.get("morphology_phase_mode"), "weight": weight, "profile": profile_rel, "profile_path": None, "profile_sha256": None, "config": C.rel(config_path), "config_sha256": cfg_sha, "evaluated_steps": len(morph_available), "settled_points": 0, "semantics": None, "source": "reward_terms recorded per step (morphology_available + morphology_settled_this_step): morphology_phase, served value, min/max = mean +- k*std +- margin"}
    if profile_rel:
        ppath = ART.resolve_under_roots(profile_rel, roots, f"{jid} morphology profile")
        p_raw = reader.read_bytes(ppath)
        prof = ART.loads_strict(p_raw)
        corridor["profile_path"], corridor["profile_sha256"] = C.rel(ppath), ART.sha256_bytes(p_raw)
        corridor["profile_parameterization"] = (prof.get("metadata") or {}).get("phase_parameterization")
        try:
            corridor["profile_mean"] = {c: {"phase": prof["phase_grid"], "mean_rad": prof["coordinates"][c]["mean_rad"], "std_rad": prof["coordinates"][c]["std_rad"]} for c in ("pros_knee_angle", "pros_ankle_angle")}
            ART.assert_finite_tree(corridor["profile_mean"], f"{jid} profile")
        except (KeyError, TypeError, ART.SerializationError) as exc:
            raise OverlayError(f"{jid}: morphology profile malformed or non-finite: {exc}") from exc
    if morph_available and not profile_rel:
        raise OverlayError(f"{jid}: reward_terms report morphology_available on {len(morph_available)} steps but the pinned config has no morphology_profile (inconsistent runtime)")
    settled = [s for s in morph_available if A.is_finite_float(s.get("morphology_settled_this_step")) and float(s["morphology_settled_this_step"]) >= 1.0]
    corridor["settled_points"] = len(settled)
    corridor["semantics"] = (f"corridor evaluated by the reward on settled served samples (mode={corridor['phase_mode']}, weight={weight}); gaps = unsettled/dropped samples" if morph_available else f"corridor NOT evaluated by this runtime (mode={corridor['phase_mode']}, weight={weight}, profile={profile_rel}): no band drawn")
    pvc = summary.get("phase_valid_cycle_count")
    if not A.is_nonneg_int(pvc):
        raise OverlayError(f"{jid}: summary phase_valid_cycle_count missing or invalid (mandatory non-negative integer): {pvc!r}")
    final_fsm = rows[-1].get("phase_fsm")
    vcc = final_fsm.get("valid_cycle_count") if isinstance(final_fsm, dict) else None
    if not A.is_nonneg_int(vcc):
        raise OverlayError(f"{jid}: trace final valid_cycle_count missing or invalid (mandatory non-negative integer): {vcc!r}")
    cyc = cycles_for_overlay(rows, summary_cycle_count=A.as_int(pvc), trace_final_count=A.as_int(vcc))
    phase = None
    if cyc["cycles"]:
        if alpha is None:
            raise OverlayError(f"{jid}: accepted cycles present but no pinned phase profile alpha (phase products fail closed)")
        grid = np.linspace(0.0, 1.0, PHASE_POINTS)
        per_cycle = []
        for ci, c in enumerate(cyc["cycles"]):
            tt = phase_times(c, grid, alpha)
            if tt[-1] > t[-1] + 1e-9 or tt[0] < t[0] - 1e-9:
                raise OverlayError(f"{jid}: accepted cycle {ci} outside the recorded grid")
            per_cycle.append({"cycle": ci, **{k: np.interp(tt, t, khz[k]) for k in PHASE_SERIES}})
        phase = {"grid": grid, "alpha": alpha, "cycles": per_cycle, "series_keys": PHASE_SERIES, "interpolation": PHASE_INTERPOLATION}
    thresholds = {"penalty": summary.get("grf_penetration_penalty_threshold_m"), "termination": summary.get("grf_penetration_termination_m")}
    for k, v in thresholds.items():
        if v is not None and not A.is_finite_float(v):
            raise OverlayError(f"{jid}: non-finite penetration threshold {k}")
    return {"job": rec, "receipt_sha256": ART.sha256_bytes(receipt_raw), "summary_sha256": s_sha, "trace_sha256": t_sha, "sto_sha256": sto_sha, "config": C.rel(config_path), "config_sha256": cfg_sha, "analysis_binding": analysis_binding,
            "summary": summary, "steps": steps, "events": events_from_trace(rows, t0), "khz": khz, "t0": t0, "end_reason": summary.get("end_reason"), "steps_recorded": summary.get("steps"), "cycles": cyc, "phase": phase, "corridor": corridor, "sea_state_columns": sea_state_cols, "reserve_columns": reserve_cols, "penetration_thresholds_m": thresholds}


# --- CSV writers --------------------------------------------------------------------------------


def _write_csv(path: Path, header: list[str], rows: list[list[Any]]) -> dict[str, Any]:
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing artefact: {path}")
    for i, row in enumerate(rows):
        for v in row:
            if isinstance(v, (float, np.floating)) and not np.isfinite(v):
                raise OverlayError(f"non-finite value in CSV row {i} of {path.name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        w = csv.writer(handle)
        w.writerow(header)
        for row in rows:
            w.writerow(["" if v is None else (f"{float(v):.10g}" if isinstance(v, (float, np.floating)) else v) for v in row])
    return {"path": f"{path.parent.name}/{path.name}", "sha256": C.sha256_file(path), "rows": len(rows)}


def write_job_csvs(out_dir: Path, data: dict[str, Any]) -> dict[str, Any]:
    jid = data["job"]["job_id"]
    khz = data["khz"]
    keys = list(khz.keys())
    n = len(khz["t_rel_s"])
    files = {"khz": {**_write_csv(out_dir / "series" / f"{jid}__1khz.csv", keys, [[float(khz[k][i]) for k in keys] for i in range(n)]), "columns": keys, "reserve_columns": data["reserve_columns"], "sea_state_columns": data["sea_state_columns"]}}
    skeys = list(data["steps"][0].keys())
    files["steps"] = _write_csv(out_dir / "series" / f"{jid}__steps.csv", skeys, [[s.get(k) for k in skeys] for s in data["steps"]])
    ekeys = ["step", "time_s", "t_rel_s", "kind", "source", "detail"]
    files["events"] = _write_csv(out_dir / "series" / f"{jid}__events.csv", ekeys, [[e.get(k) for k in ekeys] for e in data["events"]])
    if data["phase"]:
        ph = data["phase"]
        rows = [[c["cycle"], float(p), float(phase_times(data["cycles"]["cycles"][c["cycle"]], np.asarray([p]), ph["alpha"])[0]), *[float(c[k][i]) for k in ph["series_keys"]]] for c in ph["cycles"] for i, p in enumerate(ph["grid"])]
        files["phase"] = {**_write_csv(out_dir / "series" / f"{jid}__phase.csv", ["cycle", "phase", "time_s", *ph["series_keys"]], rows), "interpolation": PHASE_INTERPOLATION, "alpha": ph["alpha"]}
    else:
        phase_status = data["cycles"]["phase_aligned"]
        files["phase"] = {"path": None, "status": phase_status}
        if "phase_contract_error" in data["cycles"]:
            files["phase"]["phase_contract_error"] = data["cycles"]["phase_contract_error"]
    return files


# --- figures ---------------------------------------------------------------------------------------


def _style(data: dict[str, Any], role: str) -> dict[str, Any]:
    color = COLORS.get(data["job"]["candidate"], "#000000")
    return {"reference": {"color": color, "lw": 2.0, "ls": "-"}, "control": {"color": color, "lw": 1.2, "ls": "--"}, "stoch": {"color": color, "lw": 0.6, "ls": "-", "alpha": 0.8}, "replay": {"color": color, "lw": 1.2, "ls": ":"}}.get(role, {"color": color, "lw": 1.2, "ls": "-"})


def _label(data: dict[str, Any]) -> str:
    j = data["job"]
    seed = f" seed{j['seed']}" if j["action_selection"] == "stochastic" else ""
    return f"{j['candidate']} [{j['runtime']}{seed}] {j['comparison_class']} ({data['steps_recorded']} step, {data['end_reason']})"


def group_phase(members: list[tuple[dict[str, Any], str]], *, phase_allowed: bool = True) -> dict[str, Any]:
    zero = [d["job"]["job_id"] for d, _ in members if d["cycles"]["valid_cycle_count"] == 0]
    if not phase_allowed:
        status = PHASE_DECLARED_TIME_ONLY
    else:
        status = PHASE_NONE if zero else PHASE_ALL
    return {"group_phase_status": status, "zero_cycle_members": zero, "phase_png_allowed": bool(phase_allowed and not zero), "phase_allowed_by_declaration": bool(phase_allowed)}


def render_group(out_dir: Path, name: str, members: list[tuple[dict[str, Any], str]], title: str, *, band: list[dict[str, Any]] | None = None, phase_allowed: bool = True) -> dict[str, Any]:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    status = group_phase(members, phase_allowed=phase_allowed)
    outputs = []
    panels = [("knee_measured_rad", "knee measured [rad]"), ("knee_served_ref_rad", "knee served reference q_ref [rad]"), ("ankle_measured_rad", "ankle measured [rad]"), ("ankle_served_ref_rad", "ankle served reference q_ref [rad]"), ("u_knee", "u_knee normalised (sea_controls)"), ("u_ankle", "u_ankle normalised"), ("tau_spring_knee_nm", "SEA knee tau_spring [Nm]"), ("tau_spring_ankle_nm", "SEA ankle tau_spring [Nm]"), ("tau_motor_knee_nm", "SEA knee tau_motor [Nm]"), ("tau_motor_ankle_nm", "SEA ankle tau_motor [Nm]"), ("grf_left_normal_n", "left GRF normal [N]"), ("grf_left_penetration_m", "left penetration [m] (thresholds)"), ("reserve_norm_nm", "reserve norm [Nm]"), ("reserve_abs_max_envelope_nm", "reserve max-abs envelope over ALL reserve columns [Nm]")]
    fig, axes = plt.subplots(len(panels) + 4, 1, figsize=(16, 2.2 * (len(panels) + 4)), sharex=True)
    for ax, (key, lab) in zip(axes, panels):
        for data, role in members:
            st = _style(data, role)
            x, y = data["khz"]["t_rel_s"], data["khz"][key]
            ax.plot(x, y, label=_label(data), **st)
            ax.plot([x[-1]], [y[-1]], marker="|", ms=10, color=st["color"])
        if key == "grf_left_penetration_m":
            thr = members[0][0]["penetration_thresholds_m"]
            for v, n in ((thr.get("penalty"), "penalty"), (thr.get("termination"), "termination")):
                if A.is_finite_float(v):
                    ax.axhline(float(v), color="k", lw=0.6, ls="--")
                    ax.text(0, float(v), n, fontsize=7, va="bottom")
        if band:
            for b in band:
                ax.fill_between(b["t_rel_s"], b[key + "_min"], b[key + "_max"], color=b["color"], alpha=0.12, label=b["label"] if key == panels[0][0] else None)
        ax.set_ylabel(lab, fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=6, loc="upper right")
    ax = axes[len(panels)]
    for data, role in members:
        st = _style(data, role)
        xs = [s["t_rel_s"] for s in data["steps"]]
        ax.step(xs, [s["raw_knee"] for s in data["steps"]], where="post", **st)
        ax.step(xs, [s["applied_knee"] for s in data["steps"]], where="post", **{**st, "alpha": 0.5})
        if data["steps"][0]["mean_knee"] is not None:
            ax.step(xs, [s["mean_knee"] for s in data["steps"]], where="post", **{**st, "ls": "-.", "lw": 0.8})
    ax.set_ylabel("knee action raw / applied (step) / mean (-.)", fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[len(panels) + 1]
    markers = {"fsm_accepted_heel_strike": ("^", 0.0), "fsm_accepted_toe_off": ("v", 0.0), "fsm_hs_cancelled": ("x", 0.0), "fsm_rejected": ("s", -0.3), "fsm_cycle_rejected": ("s", -0.3), "fsm_resync": ("D", 0.3)}
    for data, role in members:
        st = _style(data, role)
        xs = [s["t_rel_s"] for s in data["steps"]]
        ax.step(xs, [s["fsm_valid_hs_count"] for s in data["steps"]], where="post", **st)
        ax.step(xs, [s["fsm_invalid_event_count"] for s in data["steps"]], where="post", **{**st, "ls": ":"})
        for e in data["events"]:
            if e["t_rel_s"] is None:
                continue
            for prefix, (mk, yy) in markers.items():
                if e["kind"].startswith(prefix):
                    ax.plot([e["t_rel_s"]], [yy], marker=mk, ms=5, color=st["color"])
                    break
    ax.set_ylabel("FSM valid_hs (step) / invalid (:) / events ^HS vTO xCanc sRej DResync", fontsize=7)
    ax.grid(alpha=0.3)
    ax = axes[len(panels) + 2]
    for data, role in members:
        st = _style(data, role)
        xs = [s["t_rel_s"] for s in data["steps"]]
        res = [s["causal_total_resolved_sample_count"] for s in data["steps"]]
        if all(v is not None for v in res):
            ax.step(xs, res, where="post", **st)
            ax.step(xs, [s["causal_total_dropped_sample_count"] for s in data["steps"]], where="post", **{**st, "ls": ":"})
    ax.set_ylabel("causal ledger resolved (step) / dropped (:)", fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[len(panels) + 3]
    drawn_profiles: set[str] = set()
    notes = 0
    for data, role in members:
        st = _style(data, role)
        cor = data["corridor"]
        pts = [s for s in data["steps"] if A.is_finite_float(s.get("morphology_available")) and float(s["morphology_available"]) >= 1 and A.is_finite_float(s.get("morphology_settled_this_step")) and float(s["morphology_settled_this_step"]) >= 1]
        if pts:
            ph = np.asarray([s["morphology_phase"] for s in pts], dtype=np.float64)
            order = np.argsort(ph)
            ax.scatter(ph[order], np.asarray([s["morphology_knee_value_rad"] for s in pts])[order], s=6, color=st["color"], label=f"{data['job']['candidate']} settled served knee ({len(pts)} pts, recorded)")
            ax.plot(ph[order], np.asarray([s["morphology_knee_min_rad"] for s in pts])[order], color=st["color"], lw=0.5, ls=":")
            ax.plot(ph[order], np.asarray([s["morphology_knee_max_rad"] for s in pts])[order], color=st["color"], lw=0.5, ls=":")
            if cor.get("profile_sha256") and cor["profile_sha256"] not in drawn_profiles and cor.get("profile_mean"):
                pm = cor["profile_mean"]["pros_knee_angle"]
                ax.plot(pm["phase"], pm["mean_rad"], color="k", lw=0.8, label=f"pinned profile mean ({cor.get('profile_parameterization')}, {cor['profile_sha256'][:8]})")
                drawn_profiles.add(cor["profile_sha256"])
        else:
            ax.text(0.01, 0.9 - 0.12 * notes, f"{data['job']['candidate']} [{data['job']['runtime']}]: {cor['semantics']}", transform=ax.transAxes, fontsize=7, color=st["color"])
            notes += 1
    ax.set_ylabel("corridor (recorded) knee [rad] vs morphology_phase", fontsize=8)
    ax.set_xlabel("t - t_start [s] (corridor panel: morphology phase 0..1)")
    ax.legend(fontsize=6)
    ax.grid(alpha=0.3)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    png = out_dir / "png" / f"{name}.png"
    if png.exists() or png.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing artefact: {png}")
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png, dpi=90, metadata={"Software": "f0_overlays"})
    plt.close(fig)
    outputs.append({"path": f"png/{png.name}", "sha256": C.sha256_file(png), "kind": "time_aligned", "byte_stable_cross_host": False})
    if status["phase_png_allowed"]:
        if any(d["phase"] is None for d, _ in members):
            raise OverlayError(f"{name}: phase figure requested but a member has no phase products")
        fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        alpha = members[0][0]["phase"]["alpha"]
        for data, role in members:
            st = _style(data, role)
            ph = data["phase"]
            for key, ax in zip(("knee_measured_rad", "knee_served_ref_rad", "ankle_measured_rad", "tau_motor_knee_nm", "grf_left_normal_n"), axes):
                for c in ph["cycles"]:
                    ax.plot(ph["grid"], c[key], **{**st, "alpha": 0.6}, label=f"{data['job']['candidate']} ({len(ph['cycles'])} cycles)" if c["cycle"] == 0 else None)
                ax.axvline(alpha, color="k", lw=0.5, ls="--")
                ax.set_ylabel(key, fontsize=8)
                ax.grid(alpha=0.3)
        axes[0].legend(fontsize=6)
        axes[-1].set_xlabel(f"event-warped phase 0..1 (HS->TO->HS, alpha={alpha:.10f} at the dashed line); {PHASE_INTERPOLATION}", fontsize=5)
        fig.suptitle(f"{title} — phase-aligned (all members have accepted cycles)", fontsize=8)
        fig.tight_layout()
        png2 = out_dir / "png" / f"{name}__phase.png"
        if png2.exists() or png2.is_symlink():
            raise FileExistsError(f"refusing to overwrite existing artefact: {png2}")
        fig.savefig(png2, dpi=90, metadata={"Software": "f0_overlays"})
        plt.close(fig)
        outputs.append({"path": f"png/{png2.name}", "sha256": C.sha256_file(png2), "kind": "phase_aligned", "byte_stable_cross_host": False})
    return {**status, "outputs": outputs}


def stochastic_band(replicas: list[dict[str, Any]], color: str, label: str) -> dict[str, Any]:
    """min/max band over the replicas on the common span [0, min end]; the replicas' grids must be identical on that span."""
    if len(replicas) < 2:
        raise OverlayError("stochastic band needs at least two replicas")
    n = min(len(d["khz"]["t_rel_s"]) for d in replicas)
    grid = replicas[0]["khz"]["t_rel_s"][:n]
    for d in replicas[1:]:
        if not np.array_equal(d["khz"]["t_rel_s"][:n], grid):
            raise OverlayError(f"stochastic band: replica {d['job']['job_id']} grid differs from the first replica on the common span")
    out = {"t_rel_s": grid, "color": color, "label": label, "common_samples": n, "definition": "min/max across replicas on the common span [0, min end] (identical validated grids); individual replicas drawn in full"}
    for key in BAND_SERIES:
        stack = np.stack([d["khz"][key][:n] for d in replicas])
        out[key + "_min"], out[key + "_max"] = stack.min(axis=0), stack.max(axis=0)
    return out


# --- orchestration ------------------------------------------------------------------------------


def group_specs(index: dict[str, M.JobSpec]) -> list[dict[str, Any]]:
    def jid(family, cand, runtime, start, mode="deterministic", seed=123, rep=1):
        j = M.job_id((family, cand, runtime, start, mode, seed, rep, None))
        if j not in index:
            raise OverlayError(f"group member missing from the canonical matrix: {j}")
        return j

    groups = []
    for start in STARTS:
        groups.append({"name": f"G1_isometric__{start}", "title": f"G1 isometric comparables under v3_canonical — start {start} (reference B0820_H0 thick)", "members": [(jid("det", c, "v3_canonical", start), "reference" if c == "B0820_H0" else "member") for c in ISOMETRIC_CANDIDATES], "phase_allowed": True})
        groups.append({"name": f"G2_controls__{start}", "title": f"G2 controls (dashed, descriptive, no delta/isometry) vs B0820_H0 — start {start}; V26_39D = compatibility control on its own runtime (time-aligned only, no phase figure by declaration)", "members": [(jid("det", "B0820_H0", "v3_canonical", start), "reference"), (jid("det", "JUL_H0", "v3_canonical", start), "control"), (jid("det", "JUL_BEST", "v3_canonical", start), "control"), (jid("ctrl39", "V26_39D", "v26_imitation_native", start), "control")], "phase_allowed": False})
        for cand in ("JUL_H0", "JUL_BEST"):
            groups.append({"name": f"G3_runtime__{cand}__{start}", "title": f"G3 runtime effect: same weights {cand}, july_legacy (replay, dotted) vs v3_canonical — start {start} (never fused with other chains)", "members": [(jid("replay", cand, "july_legacy", start), "replay"), (jid("det", cand, "v3_canonical", start), "member")], "phase_allowed": True})
    for cand in ("B0820_H0", "B0820_V2_BEST"):
        groups.append({"name": f"G3_runtime__{cand}__nominal", "title": f"G3 runtime effect: same weights {cand}, v2_b0820 (replay, dotted) vs v3_canonical — start nominal", "members": [(jid("replay", cand, "v2_b0820", "nominal"), "replay"), (jid("det", cand, "v3_canonical", "nominal"), "member")], "phase_allowed": True})
    for start in STARTS:
        for cand in STOCH_CANDIDATES:
            members = [(jid("det", cand, "v3_canonical", start), "reference")] + [(jid("stoch", cand, "v3_canonical", start, "stochastic", s), "stoch") for s in C.DEVELOPMENT_SEEDS]
            extra = ""
            if cand == "JUL_H0" and start == "plus020":
                members.append((jid("replay", "JUL_H0", "july_legacy", start, "stochastic", 123), "replay"))
                extra = "; replay july_legacy seed 123 dotted (individual control, never banded)"
            groups.append({"name": f"G4_stochastic__{cand}__{start}", "title": f"G4 stochastic {cand} — start {start}: deterministic thick + seeds 123/124/125 full thin lines (+ min/max band on the common span only){extra}", "members": members, "band_candidate": cand, "phase_allowed": True})
    return groups


def run(stamp: str | None = None, *, out_root: Path | None = None, python_exe: str | None = None) -> dict[str, Any]:
    stamp = ART.validate_stamp(stamp or time.strftime("%Y%m%d_%H%M%S"))
    out_root = Path(out_root) if out_root is not None else C.OUT_OVERLAYS
    final_dir = out_root / stamp
    if final_dir.exists() or final_dir.is_symlink():
        raise FileExistsError(f"refusing to reuse existing overlay directory: {final_dir}")
    # --- discovery (pure registry projection: no job byte is read before PRE) ------------------------------
    index = M.canonical_index()
    described = {M.job_id(s): M.static_job_record(s) for s in index.values()}
    groups = group_specs(index)
    rep2 = M.job_id(("det", "B0820_V3_BEST", "v3_canonical", "nominal", "deterministic", 123, 2, None))
    base = M.job_id(("det", "B0820_V3_BEST", "v3_canonical", "nominal", "deterministic", 123, 1, None))
    needed = sorted({jid for g in groups for jid, _ in g["members"]} | {rep2})
    analysis_path = C.OUT_METRICS / FINAL_ANALYSIS_FILE
    code_paths = [HERE / n for n in CODE_INPUTS]
    declared: list[Path] = [analysis_path, Path(PHASE_PROFILE), *[Path(p) for p in MORPH_PROFILES], *code_paths]
    for jid in needed:
        declared += job_consumed_files(described[jid])["files"]  # static names: no byte is read before declare
    ledger = ART.InputLedger()
    before = ledger.declare(declared)  # PRE snapshot before any computational read
    interpreter = ART.interpreter_provenance()
    code_digests = {C.rel(p): ART.sha256_bytes(ledger.read_bytes(p)) for p in code_paths}
    # --- load through the ledger (every read re-opened, digest == PRE) -------------------------------------
    analysis, analysis_ref = load_final_analysis(analysis_path, reader=ledger)
    try:
        analysis_jobs = M.frozen_analysis_index(analysis)
    except RuntimeError as exc:
        raise OverlayError(str(exc)) from exc
    phase_profile = phase_profile_alpha(PHASE_PROFILE, reader=ledger)
    data = {jid: load_job(described[jid], analysis_job=analysis_jobs[jid], verify=None, alpha=phase_profile["alpha"], reader=ledger) for jid in needed}
    ledger.assert_unchanged("after loading (consumed set re-hashed)")
    ledger.assert_all_read()
    identity = {k: bool(np.array_equal(data[base]["khz"][k], data[rep2]["khz"][k])) for k in data[base]["khz"]}
    identity_check = {"reference": base, "repeat": rep2, "all_1khz_series_identical": all(identity.values()), "per_series": identity, "trace_sha256_identical": data[base]["trace_sha256"] == data[rep2]["trace_sha256"]}
    script_sha = code_digests[C.rel(HERE / "f0_overlays.py")]
    git = C.git_snapshot()

    # --- publish -------------------------------------------------------------------------------------------
    def build(staging: Path) -> dict[str, Any]:
        job_records = {}
        for jid, d in data.items():
            job_records[jid] = {"csv": write_job_csvs(staging, d), "receipt_sha256": d["receipt_sha256"], "summary_sha256": d["summary_sha256"], "trace_sha256": d["trace_sha256"], "sto_sha256": d["sto_sha256"], "config": d["config"], "config_sha256": d["config_sha256"], "analysis_binding": d["analysis_binding"], "corridor": {k: v for k, v in d["corridor"].items() if k != "profile_mean"}, "cycles": {k: v for k, v in d["cycles"].items() if k != "cycles"}, "cycle_list": d["cycles"]["cycles"], "end_reason": d["end_reason"], "steps_recorded": d["steps_recorded"], "sea_state_columns": d["sea_state_columns"], "reserve_columns": d["reserve_columns"]}
        degraded_phase_jobs = [
            {"job_id": jid, "status": rec["cycles"]["phase_aligned"], "phase_contract_error": rec["cycles"].get("phase_contract_error")}
            for jid, rec in job_records.items()
            if rec["cycles"]["phase_aligned"] == "invalid_fsm_cycle_contract"
        ]
        figures = []
        for g in groups:
            members = [(data[jid], role) for jid, role in g["members"]]
            band = None
            if g.get("band_candidate"):
                band = [stochastic_band([d for d, r in members if r == "stoch"], COLORS[g["band_candidate"]], f"min/max over seeds {list(C.DEVELOPMENT_SEEDS)} (common span only)")]
            rendered = render_group(staging, g["name"], members, g["title"], band=band, phase_allowed=g["phase_allowed"])
            figures.append({"group": g["name"], "title": g["title"], "phase_allowed_by_declaration": g["phase_allowed"], "members": [{"job_id": jid, "role": role, "phase_aligned": data[jid]["cycles"]["phase_aligned"], "valid_cycles": data[jid]["cycles"]["valid_cycle_count"]} for jid, role in g["members"]], **rendered, "band": {k: v for k, v in (band[0].items() if band else []) if k in ("common_samples", "definition", "label")}})
        manifest = {"schema_version": 4, "stamp": stamp, "generated_at_utc": C.utc_now(), "git": git, "script_sha256": script_sha, "interpreter": interpreter, "software": {"python": sys.version.split()[0], "numpy": np.__version__, "matplotlib": __import__("matplotlib").__version__}, "code_inputs_sha256": code_digests,
                    "source_analysis": analysis_ref, "phase_profile": phase_profile, "alignment": {"time": "t - t_start on the verified 1 kHz grid, every trajectory to its own end marker/reason, no truncation", "phase": "accepted HS -> accepted TO -> accepted HS (cycle_valid == 1), count cross-checked with the mandatory summary phase_valid_cycle_count; a group gets a phase figure only when allowed by declaration AND every member has >= 1 accepted cycle (else TIME_ALIGNED_ONLY_BY_DECLARATION / NOT_PHASE_ALIGNABLE_ZERO_VALID_CYCLES with the zero-cycle members listed); legacy cycles informative only", "phase_interpolation": PHASE_INTERPOLATION},
                    "signals": {"1khz": "measured/served (q_ref) knee-ankle, u normalised (sea_controls), SEA spring AND motor torques, SEA motor states, left GRF normal/contact/penetration, EVERY reserve torque column + norm + max-abs envelope", "steps": "raw/applied/mean action + exploration noise, policy_segment knot (diagnostic only), FSM counters, causal ledger counters, recorded corridor terms (step-wise, never interpolated)", "events": "accepted/cancelled/rejected/resync/cycle-rejected FSM events, sensor events, legacy detector events (informative)", "corridor": "exactly the reward_terms recorded on morphology_available + settled samples; pinned config/profile path+sha in provenance; runtimes without evaluation labelled"},
                    "jobs": job_records, "degraded_phase_jobs": degraded_phase_jobs, "rep2_identity_check": identity_check, "figures": figures, "groups_phase_summary": {f["group"]: f["group_phase_status"] for f in figures}, "stochastic_band_note": "G4 band = min/max of the replicas on the common span [0, min end] with identical validated grids; individual replicas stay full", "inputs_snapshot_sha256": before, "png_note": "digests recorded, not byte-stable across hosts", "ledger_contract": ART.__doc__.strip().splitlines()[0]}
        ART.write_json_strict(staging / f"overlay_manifest_{stamp}.json", manifest)
        degraded_section = ["", "## Job esclusi dall'allineamento di fase per violazione del contratto FSM", ""]
        if degraded_phase_jobs:
            degraded_section.append(C.md_table(["job", "stato", "errore"], [[d["job_id"], d["status"], d["phase_contract_error"]] for d in degraded_phase_jobs]))
        else:
            degraded_section.append("Nessuno.")
        C.write_text(staging / f"overlay_summary_{stamp}.md", "\n".join([f"# F0.6 — Overlay sincronizzati — stamp {stamp}", "", f"Analisi sorgente `{analysis_ref['sha256'][:12]}…`; {len(job_records)} job letti tramite ledger (ogni lettura byte-bound al digest PRE e verificata POST); identità rep2: {identity_check['all_1khz_series_identical']}; profilo di fase `{phase_profile['sha256'][:12]}…` alpha={phase_profile['alpha']:.10f}; {len(before)} input consumati invariati.", "", PHASE_INTERPOLATION, "", C.md_table(["gruppo", "stato phase", "membri zero-cycle", "membri (ruolo, cicli validi)", "PNG"], [[f["group"], f["group_phase_status"], ", ".join(f["zero_cycle_members"]) or "-", "; ".join(f"{m['job_id']} ({m['role']}, {m['valid_cycles']})" for m in f["members"]), ", ".join(o["path"].split("/")[-1] for o in f["outputs"])] for f in figures]), *degraded_section]))
        return {"kind": "f0_overlays", "source_analysis": analysis_ref, "script_sha256": script_sha, "git_head": git["head"], "inputs_unchanged": True, "interpreter": interpreter, "consumed_inputs": len(before), "inputs_snapshot_sha256": before, "figures": len(figures), "phase_figures": sum(1 for f in figures if f["group_phase_status"] == PHASE_ALL)}

    def before_publish() -> None:
        ledger.assert_unchanged("immediately before publish (POST)")
        ledger.assert_all_read()

    published = ART.publish_artifact(final_dir, anchor=out_root, stamp=stamp, build=build, before_publish=before_publish)
    return {**published, "jobs": len(data), "groups": len(groups), "rep2_identical": identity_check["all_1khz_series_identical"], "consumed_inputs": len(before)}


def exit_code_for(result: dict[str, Any]) -> int:
    return 0 if isinstance(result, dict) and result.get("artifact", {}).get("ok") is True else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="f0_overlays.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args(argv)
    try:
        result = run()
    except Exception as exc:  # noqa: BLE001 - any failure of the fail-closed protocol is a nonzero exit
        print(f"[overlays] FAILED ({type(exc).__name__}): {exc}", file=sys.stderr)
        return 1
    code = exit_code_for(result)
    print(f"[overlays] published {result['final_dir']}; jobs={result['jobs']} groups={result['groups']} rep2_identical={result['rep2_identical']} consumed_inputs={result['consumed_inputs']} artifact_ok={result['artifact']['ok']} exit={code}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
