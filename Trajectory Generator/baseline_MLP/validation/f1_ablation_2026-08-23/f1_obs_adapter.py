"""Validation-only observation adapter for the A_iso39_v3 ablation (F1).

Pure numpy; no simulator, torch or ray import.  The adapter

* derives the 39->35 insertion from the two content-addressed feature
  manifests (never from hard-coded indices): the 39D list must equal the 35D
  list with exactly the four ``healthy_*_imitation_target[_vel]`` names
  inserted contiguously at index 2 (fail-closed otherwise);
* exposes ``project39to35`` (delete the inserted block) and
  ``insert_targets`` (inverse), both valid for the actor block (39 <-> 35)
  and for the full observation (88 <-> 84) because the env appends the
  privileged suffix after the actor block;
* asserts at every step that ``project39to35(obs39) == obs35`` **exactly**
  (``np.array_equal`` on the same dtype), counting the checks;
* reconstructs the V26 prescribed targets read-only from objects that expose
  the env's own ``GaitPhaseClock`` / ``PhaseBasedImitationTarget`` contract
  (``raw_phase``, ``local_period``, ``get(t)``).  The objects are injected, so
  tests use synthetic ones; ``build_reconstructor_from_env`` (in
  ``f1_rollout_aiso``) builds them from the live env with the env's own
  classes and parameters.  The env itself is never mutated: its own clock and
  imitation target stay disabled under the v3 runtime.

Why obs[0:2] are not restored in the gated mode: under the v3 runtime the env
emits ``gait_phase_sin/cos = (0, 1)`` (clock disabled); the hard-drop actor B
has those first-layer columns zeroed.  A_iso (mode ``aiso4``) keeps them as B
sees them, so A_iso restores 4 of the 6 signals lost in the transplant.  Mode
``aiso6clk`` (diagnostic only) additionally restores the prescribed clock.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

TARGET_FEATURE_NAMES = (
    "healthy_knee_angle_imitation_target",
    "healthy_knee_angle_imitation_target_vel",
    "healthy_ankle_angle_imitation_target",
    "healthy_ankle_angle_imitation_target_vel",
)
CLOCK_FEATURE_NAMES = ("gait_phase_sin", "gait_phase_cos")
TARGET_COORDS = ("pros_knee_angle", "pros_ankle_angle")
DISABLED_CLOCK_SIN_COS = (0.0, 1.0)


class AdapterError(RuntimeError):
    """Fail-closed adapter contract violation."""


@dataclass(frozen=True)
class InsertionSpec:
    """Where the 39D-only block sits inside the 39D actor vector."""

    index: int
    count: int
    names35: tuple[str, ...]
    names39: tuple[str, ...]
    inserted_names: tuple[str, ...]
    manifest35_sha256: str | None = None
    manifest39_sha256: str | None = None

    @property
    def stop(self) -> int:
        return self.index + self.count

    def to_dict(self) -> dict[str, Any]:
        return {
            "insert_index": self.index,
            "insert_count": self.count,
            "inserted_names": list(self.inserted_names),
            "width_35": len(self.names35),
            "width_39": len(self.names39),
            "manifest35_sha256": self.manifest35_sha256,
            "manifest39_sha256": self.manifest39_sha256,
            "projection": f"delete indices [{self.index}, {self.stop}) of the 39-wide vector (actor block or full observation)",
        }


def read_manifest_names(path: Path, *, expected_sha256: str | None = None, sha256_fn: Callable[[Path], str] | None = None) -> tuple[list[str], str | None]:
    """Names of an actor feature manifest, optionally pinned by SHA-256."""
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise AdapterError(f"manifest missing or symlink: {path}")
    digest = sha256_fn(path) if sha256_fn is not None else None
    if expected_sha256 is not None:
        if sha256_fn is None:
            raise AdapterError("expected_sha256 given without sha256_fn")
        if digest != expected_sha256:
            raise AdapterError(f"manifest digest mismatch for {path.name}: pinned {expected_sha256} vs disk {digest}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    names = payload.get("actor_feature_names") if isinstance(payload, dict) else None
    if not isinstance(names, list) or not names or not all(isinstance(n, str) and n for n in names) or len(set(names)) != len(names):
        raise AdapterError(f"malformed actor_feature_names in {path}")
    count = payload.get("actor_feature_count")
    if count is not None and int(count) != len(names):
        raise AdapterError(f"actor_feature_count {count} != {len(names)} in {path}")
    return [str(n) for n in names], digest


def derive_insertion(names35: Sequence[str], names39: Sequence[str], *, manifest35_sha256: str | None = None, manifest39_sha256: str | None = None) -> InsertionSpec:
    """Derive the contiguous insertion block from the two name lists (fail-closed)."""
    n35 = tuple(str(n) for n in names35)
    n39 = tuple(str(n) for n in names39)
    if len(set(n35)) != len(n35) or len(set(n39)) != len(n39):
        raise AdapterError("duplicate feature names")
    extra = [n for n in n39 if n not in set(n35)]
    missing = [n for n in n35 if n not in set(n39)]
    if missing:
        raise AdapterError(f"35D names absent from the 39D list: {missing}")
    if tuple(extra) != TARGET_FEATURE_NAMES:
        raise AdapterError(f"39D-only names {extra} != expected {list(TARGET_FEATURE_NAMES)}")
    positions = [n39.index(n) for n in extra]
    index = positions[0]
    if positions != list(range(index, index + len(extra))):
        raise AdapterError(f"39D-only block is not contiguous: {positions}")
    if n39[:index] + n39[index + len(extra):] != n35:
        raise AdapterError("deleting the 39D-only block does not reproduce the 35D list in order")
    if n35[:2] != CLOCK_FEATURE_NAMES or index != 2:
        raise AdapterError(f"expected the clock pair at 0:2 and the block at index 2, got index {index}, head {n35[:2]}")
    return InsertionSpec(index=index, count=len(extra), names35=n35, names39=n39, inserted_names=tuple(extra), manifest35_sha256=manifest35_sha256, manifest39_sha256=manifest39_sha256)


def project39to35(obs39: np.ndarray, spec: InsertionSpec) -> np.ndarray:
    """Delete the inserted block along the last axis (39->35 or 88->84)."""
    arr = np.asarray(obs39)
    if arr.shape[-1] < spec.stop:
        raise AdapterError(f"vector too short for projection: width {arr.shape[-1]} < {spec.stop}")
    return np.concatenate([arr[..., : spec.index], arr[..., spec.stop:]], axis=-1)


def insert_targets(obs35: np.ndarray, values: np.ndarray, spec: InsertionSpec) -> np.ndarray:
    """Insert ``values`` (shape (..., count)) at the block position (35->39 or 84->88),
    cast to the dtype of ``obs35`` (the env vector dtype)."""
    arr = np.asarray(obs35)
    vals = np.asarray(values, dtype=arr.dtype)
    if vals.shape[-1] != spec.count:
        raise AdapterError(f"expected {spec.count} target values, got {vals.shape[-1]}")
    if arr.shape[-1] < spec.index:
        raise AdapterError(f"vector too short for insertion: width {arr.shape[-1]}")
    if arr.ndim > 1:
        vals = np.broadcast_to(vals, arr.shape[:-1] + (spec.count,))
    return np.concatenate([arr[..., : spec.index], vals, arr[..., spec.index:]], axis=-1)


def assert_projection_exact(obs39: np.ndarray, obs35: np.ndarray, spec: InsertionSpec) -> None:
    """Fail closed unless project39to35(obs39) == obs35 exactly (same dtype, bitwise)."""
    a = np.asarray(obs39)
    b = np.asarray(obs35)
    proj = project39to35(a, spec)
    if proj.dtype != b.dtype or proj.shape != b.shape or not np.array_equal(proj, b):
        raise AdapterError("projection assertion failed: project39to35(obs39) != obs35 (exact)")
    if np.ascontiguousarray(proj).tobytes() != np.ascontiguousarray(b).tobytes():
        raise AdapterError("projection assertion failed at the byte level")


def clock_sin_cos(phase: float) -> tuple[float, float]:
    angle = 2.0 * math.pi * float(phase)
    return float(np.sin(angle)), float(np.cos(angle))


class PrescribedTargetReconstructor:
    """Reconstruct the four V26 prescribed targets (and the prescribed clock)
    at time ``t`` from injected clock/target objects with the env contract:

    * ``clock.available`` (bool), ``clock.phase(t)`` -> [0,1), ``clock.heel_strike_times``
    * ``target.available`` (bool), ``target.get(t)`` -> (q_dict, qdot_dict, phase_dict)

    The values follow the env's own assembly (osim_trj_cmc_like.py, actor
    block): ``q[coord]`` then ``qdot[coord]`` for knee, then ankle.
    """

    def __init__(self, clock: Any, target: Any, *, coords: Sequence[str] = TARGET_COORDS, provenance: Mapping[str, Any] | None = None) -> None:
        if not bool(getattr(clock, "available", False)):
            raise AdapterError("reconstructed gait clock is not available (need >= 2 sound-side heel strikes)")
        if not bool(getattr(target, "available", False)):
            raise AdapterError("reconstructed PhaseBasedImitationTarget is not available (no complete sound-side cycle)")
        self._clock = clock
        self._target = target
        self._coords = tuple(str(c) for c in coords)
        if self._coords != TARGET_COORDS:
            raise AdapterError(f"target coords {self._coords} != {TARGET_COORDS}")
        self._provenance = dict(provenance or {})
        self.calls = 0

    @property
    def provenance(self) -> dict[str, Any]:
        hs = np.asarray(getattr(self._clock, "heel_strike_times", []), dtype=float).reshape(-1)
        summary = self._target.summary() if hasattr(self._target, "summary") else {}
        return {
            **self._provenance,
            "clock_available": bool(self._clock.available),
            "clock_n_cycles": int(max(0, hs.size - 1)),
            "clock_heel_strike_times_s": hs.astype(float).tolist(),
            "clock_mean_period_s": float(np.mean(np.diff(hs))) if hs.size >= 2 else 0.0,
            "target_summary": summary,
            "coords": list(self._coords),
            "feature_order": list(TARGET_FEATURE_NAMES),
        }

    def targets(self, t: float) -> np.ndarray:
        """Four float64 values in manifest order at time ``t`` (fail-closed on missing keys)."""
        q, qd, _phases = self._target.get(float(t))
        out = []
        for coord in self._coords:
            if coord not in q or coord not in qd:
                raise AdapterError(f"prescribed target missing coordinate {coord} at t={t}")
            out.append(float(q[coord]))
            out.append(float(qd[coord]))
        values = np.asarray(out, dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise AdapterError(f"non-finite prescribed target at t={t}: {values.tolist()}")
        self.calls += 1
        return values

    def clock_sin_cos(self, t: float) -> tuple[float, float]:
        return clock_sin_cos(self._clock.phase(float(t)))


@dataclass
class AdapterStepRecord:
    step_index: int
    t_pre: float
    obs_width_in: int
    obs_width_out: int
    targets: list[float]
    clock_sin_cos_inserted: list[float] | None
    projection_exact: bool


@dataclass
class ObservationAdapter:
    """Stateful per-step adapter (numpy level).  ``mode``:

    * ``aiso4``        insert the 4 targets at the block; assert projection exact;
    * ``aiso6clk``     also overwrite obs[0:2] with the prescribed clock (diagnostic);
                       assertion restricted to indices outside the clock pair;
    * ``passthrough``  no change, assertion ``obs_out is obs_in`` (G1 control).
    """

    spec: InsertionSpec
    mode: str
    reconstructor: PrescribedTargetReconstructor | None
    records: list[AdapterStepRecord] = field(default_factory=list)
    projection_assert_count: int = 0
    steps: int = 0

    def __post_init__(self) -> None:
        if self.mode not in ("aiso4", "aiso6clk", "passthrough"):
            raise AdapterError(f"unknown adapter mode {self.mode}")
        if self.mode != "passthrough" and self.reconstructor is None:
            raise AdapterError("reconstructor required for non-passthrough modes")

    def adapt(self, obs_in: np.ndarray, t_pre: float) -> np.ndarray:
        obs_in = np.asarray(obs_in)
        if obs_in.ndim != 1:
            raise AdapterError("adapter expects a 1-D observation vector")
        width_in = int(obs_in.shape[-1])
        if width_in < len(self.spec.names35):
            raise AdapterError(f"observation width {width_in} < actor width {len(self.spec.names35)}")
        self.steps += 1
        if self.mode == "passthrough":
            self.records.append(AdapterStepRecord(self.steps, float(t_pre), width_in, width_in, [], None, True))
            self.projection_assert_count += 1
            return obs_in
        assert self.reconstructor is not None
        targets = self.reconstructor.targets(t_pre)
        obs_out = insert_targets(obs_in, targets, self.spec)
        clock = None
        if self.mode == "aiso4":
            assert_projection_exact(obs_out, obs_in, self.spec)
        else:  # aiso6clk: clock pair intentionally differs from obs_in[0:2]
            s, c = self.reconstructor.clock_sin_cos(t_pre)
            clock = [s, c]
            obs_out = obs_out.copy()
            obs_out[0] = np.asarray(s, dtype=obs_out.dtype)
            obs_out[1] = np.asarray(c, dtype=obs_out.dtype)
            proj = project39to35(obs_out, self.spec)
            if not np.array_equal(proj[2:], obs_in[2:]) or proj.dtype != obs_in.dtype:
                raise AdapterError("aiso6clk projection assertion failed outside the clock pair")
        self.projection_assert_count += 1
        self.records.append(AdapterStepRecord(self.steps, float(t_pre), width_in, int(obs_out.shape[-1]), targets.astype(float).tolist(), clock, True))
        return obs_out

    def summary(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "steps": int(self.steps),
            "projection_assert_count": int(self.projection_assert_count),
            "all_steps_asserted": bool(self.steps == self.projection_assert_count),
            "insertion": self.spec.to_dict(),
            "reconstructor": self.reconstructor.provenance if self.reconstructor is not None else None,
        }

    def trace(self) -> list[dict[str, Any]]:
        return [
            {
                "step_index": r.step_index,
                "t_pre": r.t_pre,
                "obs_width_in": r.obs_width_in,
                "obs_width_out": r.obs_width_out,
                "targets": r.targets,
                "clock_sin_cos_inserted": r.clock_sin_cos_inserted,
                "projection_exact": r.projection_exact,
            }
            for r in self.records
        ]


def t_pre_from_trace(reset_time: float, times: Sequence[float]) -> list[float]:
    """Pre-step times of a rollout_eval trace: obs(step k) is taken at the env
    time *before* step k, i.e. the reset time for k=1 and ``time[k-2]`` after.
    The values are the exact floats the env held in ``self.t`` (no arithmetic)."""
    times = [float(x) for x in times]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise AdapterError("trace times must be strictly increasing")
    if times and times[0] <= float(reset_time):
        raise AdapterError("first trace time must exceed the reset time")
    return [float(reset_time)] + times[:-1]


CROSSCHECK_RESULT_SCHEMA = 2


def crosscheck_targets_against_trace(reconstructor: PrescribedTargetReconstructor, spec: InsertionSpec, obs39_rows: np.ndarray, t_pre: Sequence[float], *, obs_dtype: Any, post_step_targets: np.ndarray, post_step_times: Sequence[float]) -> dict[str, Any]:
    """Offline G1 check against a native-39D trace (F0 ctrl39), two EXACT comparisons
    with no tolerance (schema 2):

    * **runtime dtype** — the env emits its observation in ``obs_dtype`` (read from
      the env, e.g. ``observation_space.dtype``), and the adapter casts the inserted
      targets to the dtype of the observation it receives: the reconstruction cast
      to ``obs_dtype`` must equal the recorded ``obs39[:, 2:6]`` bit-for-bit at the
      identical pre-step times (what the actor actually sees);
    * **float64 post-step** — the env also records its own float64 targets after
      each step (``imitation_target_q/qdot`` at ``time_k``): the float64
      reconstruction at ``time_k`` must equal them bit-for-bit (proves the
      reconstruction reproduces the env computation, independent of the cast).

    The recorded observation rows must be representable in ``obs_dtype`` (they are
    the env's own cast); otherwise the trace does not come from this contract and
    the check fails closed.  The raw float64-vs-recorded difference is reported
    for information only (it is the quantisation of the recording)."""
    dtype = np.dtype(obs_dtype)
    if dtype.kind != "f":
        raise AdapterError(f"observation dtype must be floating, got {dtype}")
    rows = np.asarray(obs39_rows, dtype=np.float64)
    if rows.ndim != 2 or rows.shape[1] != len(spec.names39) or rows.shape[0] != len(t_pre):
        raise AdapterError("obs39 rows / t_pre shape mismatch")
    post = np.asarray(post_step_targets, dtype=np.float64)
    if post.shape != (rows.shape[0], spec.count) or len(post_step_times) != rows.shape[0]:
        raise AdapterError("post-step targets / times shape mismatch")
    rec = rows[:, spec.index: spec.stop]
    rec_cast = rec.astype(dtype)
    representable = bool(np.array_equal(rec_cast.astype(np.float64), rec))
    if not representable:
        raise AdapterError(f"recorded targets are not representable in the env observation dtype {dtype}: trace not produced under this contract")
    recon = np.stack([reconstructor.targets(t) for t in t_pre], axis=0)
    recon_cast = recon.astype(dtype)  # same round-to-nearest cast as insert_targets / the env
    eq_cast = recon_cast == rec_cast
    recon_post = np.stack([reconstructor.targets(float(t)) for t in post_step_times], axis=0)
    eq_post = recon_post == post
    diff_cast = np.abs(recon_cast.astype(np.float64) - rec_cast.astype(np.float64))
    diff_post = np.abs(recon_post - post)
    diff_raw = np.abs(recon - rec)
    exact_runtime = bool(eq_cast.all())
    exact_post = bool(eq_post.all())
    return {
        "schema_version": CROSSCHECK_RESULT_SCHEMA,
        "observation_dtype": str(dtype),
        "rows": int(rows.shape[0]),
        "cells": int(eq_cast.size),
        "recorded_representable_in_obs_dtype": representable,
        "exact_rows_runtime_dtype": int(np.sum(eq_cast.all(axis=1))),
        "exact_cells_runtime_dtype": int(eq_cast.sum()),
        "exact_runtime_dtype": exact_runtime,
        "max_abs_diff_runtime_dtype": float(diff_cast.max()) if diff_cast.size else 0.0,
        "float64_post_step_rows": int(post.shape[0]),
        "exact_cells_float64_post_step": int(eq_post.sum()),
        "exact_float64_post_step": exact_post,
        "max_abs_diff_float64_post_step": float(diff_post.max()) if diff_post.size else 0.0,
        "max_abs_diff_float64_vs_recorded_informational": float(diff_raw.max()) if diff_raw.size else 0.0,
        "exact": bool(exact_runtime and exact_post),
    }
