"""Teacher labeller for the F2R bridge (S0): privileged cache + per-joint labels.

The student is the deployable 35D stateless MLP; the teacher is the V26 39D
actor fed through the F1 ``aiso6clk`` adapter (four prescribed targets at
2:6, prescribed right-side clock at 0:2) and evaluated **offline** on the
student's visited states (report section 13.3/13.6, ``f2r_protocol.json``
``action_semantics``).  Everything here is pure numpy; nothing steps an env.

* :class:`PrivilegedCache` holds, on the exact env time grid ``t_pre``, the
  privileged signals the student never sees: the four V26 targets (knee q,
  knee qdot, ankle q, ankle qdot), the prescribed clock ``(sin, cos)`` and,
  optionally, the prescribed-IK action ``u_IK(t)``.  It is built read-only
  from an A_iso6clk side-car (``f1_adapter_trace.json`` +
  ``f1_adapter_summary.json``, both SHA-256 recorded) or from injected
  env-contract objects (tests).  Lookups use **exact float equality** on the
  grid: a time that is not on the grid is a contract violation, never an
  interpolation.
* :func:`encode_absolute_action` / :func:`decode_absolute_action` implement the
  absolute action map of the protocol (identical algebra to
  ``target_domain_imitation.encode_absolute_action``);
  :func:`ik_actions_from_base_kin` mirrors ``prescribed_teacher_action`` with
  ``policy_knots = 1``: ``u_IK(t) = encode(q_IK(t + segment + lookahead))``.
* :class:`TeacherLabeller` computes ``u_T`` exactly as the F1 ``aiso6clk``
  route (float32-cast obs35 -> insert float32-cast targets at 2:6 -> overwrite
  0:2 with the float32-cast clock -> float64 numpy forward, F1-verified
  deviation from the torch path 1.5e-7..2.6e-7) and composes the label per
  variant: T1 = ``u_T`` both joints, T2 = knee ``u_T`` / ankle ``u_IK``,
  T3 = ``u_IK`` both joints.

**Labels are actions only.**  ``t_pre`` is used solely to index the cache; it
is never returned as, nor mixed into, a student feature (the student input is
the env's obs35 with the dead clock constants ``(0, 1)`` at 0:2).  Every
contract violation raises :class:`f2r_common.F2RContractError` (fail-closed).
"""

from __future__ import annotations

import hashlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402  (puts F0/F1 on sys.path)
import f0_common as C  # noqa: E402  (F0 library, immutable)
import f1_obs_adapter as OA  # noqa: E402  (F1 library, immutable)
import f1_dataset as DS  # noqa: E402  (F1 library, immutable)

ADAPTER_TRACE_FILE = "f1_adapter_trace.json"
ADAPTER_SUMMARY_FILE = "f1_adapter_summary.json"
REQUIRED_ADAPTER_MODE = "aiso6clk"
TARGET_DIM = len(OA.TARGET_FEATURE_NAMES)  # 4: knee q, knee qdot, ankle q, ankle qdot (V26 order)
CLOCK_DIM = len(OA.CLOCK_FEATURE_NAMES)  # 2: (sin, cos)
TEACHER_KEYS = ("pi.0.0.weight", "pi.0.0.bias", "pi.0.2.weight", "pi.0.2.bias", "pi.1.weight", "pi.1.bias")
VARIANTS = ("T1", "T2", "T3")
JOINTS = ("knee", "ankle")
# per-joint label source (knee, ankle) per variant -- protocol ``variants.*.label``
COMPOSITION: dict[str, tuple[str, str]] = {"T1": ("u_T", "u_T"), "T2": ("u_T", "u_IK"), "T3": ("u_IK", "u_IK")}
DEAD_CLOCK = tuple(float(v) for v in OA.DISABLED_CLOCK_SIN_COS)  # (0.0, 1.0) emitted by the env under the v3 runtime
CLOCK_UNIT_CIRCLE_TOL = 1e-6

F2RContractError = R.F2RContractError


# --- small validators -------------------------------------------------------------------------


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _time_grid(values: Any, *, what: str) -> np.ndarray:
    """Non-empty, finite, strictly increasing float64 1-D grid (exact env floats)."""
    arr = np.asarray(values)
    if arr.dtype.kind not in "fiu":
        raise F2RContractError(f"{what}: expected numeric times, got dtype {arr.dtype}")
    t = np.ascontiguousarray(arr.astype(np.float64))
    if t.ndim != 1 or t.size == 0:
        raise F2RContractError(f"{what}: expected a non-empty 1-D array, got shape {t.shape}")
    if not np.all(np.isfinite(t)):
        raise F2RContractError(f"{what}: non-finite time")
    if t.size > 1 and np.any(np.diff(t) <= 0.0):
        raise F2RContractError(f"{what}: times must be strictly increasing (exact env grid)")
    return t


def _finite_array(values: Any, shape: tuple[int, ...], dtype: Any, *, what: str, exact_dtype: bool = False) -> np.ndarray:
    arr = np.asarray(values)
    if arr.dtype.kind not in "fiu":
        raise F2RContractError(f"{what}: expected numeric values, got dtype {arr.dtype}")
    if exact_dtype and arr.dtype != np.dtype(dtype):
        raise F2RContractError(f"{what}: expected dtype {np.dtype(dtype)}, got {arr.dtype}")
    if arr.shape != tuple(shape):
        raise F2RContractError(f"{what}: expected shape {tuple(shape)}, got {arr.shape}")
    out = np.ascontiguousarray(arr.astype(dtype, copy=True))
    if not np.all(np.isfinite(out)):
        raise F2RContractError(f"{what}: non-finite value")
    return out


def _read_json_file(path: Path, *, what: str) -> Any:
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise F2RContractError(f"{what} missing or symlink: {C.rel(path)}")
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError as exc:
        raise F2RContractError(f"{what} is not valid JSON ({C.rel(path)}): {exc}") from exc


def _number_list(value: Any, length: int) -> bool:
    return isinstance(value, list) and len(value) == length and all(_is_number(v) for v in value)


# --- absolute action semantics (protocol ``action_semantics``) ----------------------------------


def _bounds_table(bounds: Mapping[str, Sequence[float]], coords: Sequence[str]) -> list[tuple[float, float]]:
    table = []
    for name in coords:
        if name not in bounds:
            raise F2RContractError(f"absolute bounds missing coordinate {name!r}")
        low, high = (float(v) for v in bounds[name])
        if not (np.isfinite(low) and np.isfinite(high) and high > low):
            raise F2RContractError(f"invalid absolute bounds for {name}: {(low, high)}")
        table.append((low, high))
    return table


def encode_absolute_action(values_rad: Any, bounds: Mapping[str, Sequence[float]] = R.ABSOLUTE_BOUNDS_RAD, coords: Sequence[str] = R.PROS_COORDS) -> np.ndarray:
    """Joint angles (N, n_coords) [rad] -> normalised absolute actions (N, n_coords) float64 in [-1, 1]:
    ``a = 2 (q - low) / (high - low) - 1`` then clip.  Same algebra and operation order as
    ``target_domain_imitation.encode_absolute_action`` (bit-identical on finite inputs);
    non-finite inputs are refused."""
    values = np.asarray(values_rad, dtype=np.float64)
    coords = tuple(str(c) for c in coords)
    if values.ndim != 2 or values.shape[1] != len(coords):
        raise F2RContractError(f"encode expects shape (N, {len(coords)}), got {values.shape}")
    if not np.all(np.isfinite(values)):
        raise F2RContractError("encode: non-finite joint angle")
    table = _bounds_table(bounds, coords)
    action = np.empty_like(values)
    for index, (low, high) in enumerate(table):
        action[:, index] = 2.0 * (values[:, index] - low) / (high - low) - 1.0
    return np.clip(action, -1.0, 1.0)


def decode_absolute_action(actions: Any, bounds: Mapping[str, Sequence[float]] = R.ABSOLUTE_BOUNDS_RAD, coords: Sequence[str] = R.PROS_COORDS) -> np.ndarray:
    """Normalised actions (N, n_coords) -> joint angles [rad] float64:
    ``q = low + 0.5 (a + 1) (high - low)`` after clipping ``a`` to [-1, 1] (protocol ``decode``).
    Inverse of :func:`encode_absolute_action` inside the bounds; for tests/diagnostics."""
    acts = np.asarray(actions, dtype=np.float64)
    coords = tuple(str(c) for c in coords)
    if acts.ndim != 2 or acts.shape[1] != len(coords):
        raise F2RContractError(f"decode expects shape (N, {len(coords)}), got {acts.shape}")
    if not np.all(np.isfinite(acts)):
        raise F2RContractError("decode: non-finite action")
    table = _bounds_table(bounds, coords)
    clipped = np.clip(acts, -1.0, 1.0)
    q = np.empty_like(clipped)
    for index, (low, high) in enumerate(table):
        q[:, index] = low + 0.5 * (clipped[:, index] + 1.0) * (high - low)
    return q


def ik_actions_from_base_kin(base_kin_like: Any, t_pre: Any, *, lookahead_s: float = R.IK_TEACHER_LOOKAHEAD_S, segment_s: float = R.SEGMENT_DURATION_S, t_end: float | None = None, coords: Sequence[str] = R.PROS_COORDS, bounds: Mapping[str, Sequence[float]] = R.ABSOLUTE_BOUNDS_RAD) -> np.ndarray:
    """Prescribed-IK teacher action ``u_IK(t)`` for every ``t`` in ``t_pre`` -> (N, 2) float32.

    Mirrors ``target_domain_imitation.prescribed_teacher_action`` with ``policy_knots = 1``:
    ``teacher_time = t + segment_s + lookahead_s`` (clipped to ``t_end`` when given),
    ``q = base_kin_like.get(teacher_time)[0]`` (dict with ``pros_knee_angle`` /
    ``pros_ankle_angle``: the prescribed prosthetic IK, 6 Hz low-pass in the env), then
    :func:`encode_absolute_action` and the production ``astype(float32)``.  ``base_kin_like``
    is injected (env-free); the env itself is never built here.  The production expression
    ``t + max((t + segment) - t, dt)`` can differ from ``t + segment`` by one ulp of ``t``;
    the protocol formula ``u_IK(t) = encode(q_IK(t + 0.01 s + lookahead))`` is the one
    implemented.  ``lookahead_s`` must be finite and >= 0 (preregistered 0.0)."""
    t = np.asarray(t_pre, dtype=np.float64)
    if t.ndim != 1:
        raise F2RContractError(f"ik_actions_from_base_kin expects a 1-D t_pre, got shape {t.shape}")
    if not np.all(np.isfinite(t)):
        raise F2RContractError("ik_actions_from_base_kin: non-finite t_pre")
    lead = float(lookahead_s)
    seg = float(segment_s)
    if not (np.isfinite(lead) and lead >= 0.0):
        raise F2RContractError(f"lookahead_s must be finite and >= 0, got {lookahead_s!r}")
    if not (np.isfinite(seg) and seg > 0.0):
        raise F2RContractError(f"segment_s must be finite and > 0, got {segment_s!r}")
    end = None if t_end is None else float(t_end)
    if end is not None and not np.isfinite(end):
        raise F2RContractError(f"t_end must be finite, got {t_end!r}")
    coords = tuple(str(c) for c in coords)
    values = np.empty((t.size, len(coords)), dtype=np.float64)
    for i, x in enumerate(t.tolist()):
        teacher_time = x + seg + lead
        if end is not None:
            teacher_time = min(teacher_time, end)
        result = base_kin_like.get(teacher_time)
        if not isinstance(result, (tuple, list)) or not result or not isinstance(result[0], Mapping):
            raise F2RContractError("base_kin_like.get(t) must return (q, qdot, qddot) dicts (KinematicsInterpolator contract)")
        q = result[0]
        for j, name in enumerate(coords):
            if name not in q:
                raise F2RContractError(f"prescribed IK missing coordinate {name!r} at t={teacher_time}")
            values[i, j] = float(q[name])
    if not np.all(np.isfinite(values)):
        raise F2RContractError("non-finite prescribed IK angle")
    return encode_absolute_action(values, bounds=bounds, coords=coords).astype(np.float32)


# --- privileged cache ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PrivilegedCache:
    """Privileged signals on the exact env time grid (never student features).

    ``t_pre`` (N,) float64 exact env floats, strictly increasing; ``targets`` (N, 4) float64 in
    V26 order [knee q, knee qdot, ankle q, ankle qdot]; ``clock`` (N, 2) float64 ``(sin, cos)`` of
    the prescribed right clock at ``t_pre``; ``ik_action`` (N, 2) float32 ``u_IK(t)`` encoded
    (knee, ankle) or ``None``; ``provenance`` dict (sources, SHA-256, parameters).  Arrays are
    stored as read-only contiguous copies."""

    t_pre: np.ndarray
    targets: np.ndarray
    clock: np.ndarray
    ik_action: np.ndarray | None = None
    provenance: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        t = _time_grid(self.t_pre, what="cache t_pre")
        n = int(t.size)
        targets = _finite_array(self.targets, (n, TARGET_DIM), np.float64, what="cache targets")
        clock = _finite_array(self.clock, (n, CLOCK_DIM), np.float64, what="cache clock")
        radius = clock[:, 0] ** 2 + clock[:, 1] ** 2
        if np.any(np.abs(radius - 1.0) > CLOCK_UNIT_CIRCLE_TOL):
            raise F2RContractError("cache clock is not a (sin, cos) pair on the unit circle")
        ik = None if self.ik_action is None else _finite_array(self.ik_action, (n, R.ACTION_DIM), np.float32, what="cache ik_action", exact_dtype=True)
        if ik is not None and (np.any(ik < -1.0) or np.any(ik > 1.0)):
            raise F2RContractError("cache ik_action outside [-1, 1] (encoded absolute actions are clipped)")
        for name, arr in (("t_pre", t), ("targets", targets), ("clock", clock), ("ik_action", ik)):
            if arr is not None:
                arr.flags.writeable = False
            object.__setattr__(self, name, arr)
        object.__setattr__(self, "provenance", dict(self.provenance or {}))

    # -- size -----------------------------------------------------------------------------------
    @property
    def rows(self) -> int:
        return int(self.t_pre.size)

    def __len__(self) -> int:
        return self.rows

    # -- constructors ---------------------------------------------------------------------------
    @classmethod
    def from_adapter_sidecar(cls, job_dir: Path, *, pins: Mapping[str, str] | None = None) -> "PrivilegedCache":
        """Read-only cache from an A_iso6clk job directory (``f1_adapter_trace.json`` rows
        ``step_index``/``t_pre``/``targets``/``clock_sin_cos_inserted`` + ``f1_adapter_summary.json``).

        Fail-closed: ``adapter_mode`` must be ``aiso6clk``; ``steps`` must equal the row count
        with ``all_steps_asserted`` and ``steps_match_rollout`` true; ``step_index`` must be
        1..N; ``clock_sin_cos_inserted`` must be a 2-element list on EVERY row (an ``aiso4``
        side-car records ``null``); the recorded insertion must match the pinned manifests.
        ``pins`` (optional, e.g. ``R.ANCHORS[start]``) must carry ``adapter_trace_sha256`` and
        ``adapter_summary_sha256`` equal to the on-disk digests.  No IK here (``ik_action`` is
        ``None``): attach it with :meth:`with_ik_action` from an injected base_kin."""
        job_dir = Path(job_dir)
        if job_dir.is_symlink() or not job_dir.is_dir():
            raise F2RContractError(f"job directory missing or symlink: {C.rel(job_dir)}")
        trace_path = job_dir / ADAPTER_TRACE_FILE
        summary_path = job_dir / ADAPTER_SUMMARY_FILE
        summary = _read_json_file(summary_path, what="adapter summary")
        rows = _read_json_file(trace_path, what="adapter trace")
        if not isinstance(summary, dict):
            raise F2RContractError("adapter summary is not a mapping")
        mode = summary.get("adapter_mode")
        if mode != REQUIRED_ADAPTER_MODE:
            raise F2RContractError(f"adapter_mode {mode!r} != {REQUIRED_ADAPTER_MODE!r}: the privileged cache needs the prescribed clock (A_iso6clk side-car)")
        if "mode" in summary and summary.get("mode") != REQUIRED_ADAPTER_MODE:
            raise F2RContractError(f"adapter summary mode {summary.get('mode')!r} != {REQUIRED_ADAPTER_MODE!r}")
        if summary.get("all_steps_asserted") is not True or summary.get("steps_match_rollout") is not True:
            raise F2RContractError("adapter summary must record all_steps_asserted and steps_match_rollout as true")
        if not isinstance(rows, list) or not rows:
            raise F2RContractError(f"adapter trace empty or malformed: {C.rel(trace_path)}")
        n = len(rows)
        steps = summary.get("steps")
        if not isinstance(steps, int) or isinstance(steps, bool) or steps != n:
            raise F2RContractError(f"adapter summary steps {steps!r} != {n} trace rows")
        insertion = summary.get("insertion")
        if not isinstance(insertion, dict) or insertion.get("insert_index") != 2 or insertion.get("insert_count") != TARGET_DIM or list(insertion.get("inserted_names") or []) != list(OA.TARGET_FEATURE_NAMES) or insertion.get("manifest35_sha256") != C.ACTOR_MANIFEST_35_SHA256 or insertion.get("manifest39_sha256") != C.ACTOR_MANIFEST_39_SHA256:
            raise F2RContractError("adapter summary insertion does not match the pinned 35D/39D manifests (block of 4 at index 2)")
        t_pre: list[float] = []
        targets: list[list[float]] = []
        clock: list[list[float]] = []
        for i, row in enumerate(rows):
            if not isinstance(row, dict):
                raise F2RContractError(f"adapter trace row {i + 1} is not a mapping")
            if row.get("step_index") != i + 1:
                raise F2RContractError(f"adapter trace step_index must be 1..N contiguous (row {i + 1}: {row.get('step_index')!r})")
            t = row.get("t_pre")
            if not _is_number(t):
                raise F2RContractError(f"adapter trace row {i + 1}: t_pre is not a number")
            tg = row.get("targets")
            if not _number_list(tg, TARGET_DIM):
                raise F2RContractError(f"adapter trace row {i + 1}: targets must be a {TARGET_DIM}-element list")
            ck = row.get("clock_sin_cos_inserted")
            if not _number_list(ck, CLOCK_DIM):
                raise F2RContractError(f"adapter trace row {i + 1}: clock_sin_cos_inserted must be a {CLOCK_DIM}-element list (got {ck!r}); only A_iso6clk side-cars carry the prescribed clock")
            t_pre.append(float(t))
            targets.append([float(v) for v in tg])
            clock.append([float(v) for v in ck])
        sha_trace = C.sha256_file(trace_path)
        sha_summary = C.sha256_file(summary_path)
        pins_verified = None
        if pins is not None:
            for key, disk in (("adapter_trace_sha256", sha_trace), ("adapter_summary_sha256", sha_summary)):
                pinned = pins.get(key) if isinstance(pins, Mapping) else None
                if not isinstance(pinned, str) or pinned != disk:
                    raise F2RContractError(f"{key} pin mismatch for {C.rel(job_dir)}: pinned {pinned!r} vs disk {disk}")
            pins_verified = {"adapter_trace_sha256": sha_trace, "adapter_summary_sha256": sha_summary}
        recon = summary.get("reconstructor") if isinstance(summary.get("reconstructor"), dict) else {}
        provenance = {
            "source": "adapter_sidecar",
            "job_dir": C.rel(job_dir),
            "adapter_trace": {"path": C.rel(trace_path), "sha256": sha_trace},
            "adapter_summary": {"path": C.rel(summary_path), "sha256": sha_summary},
            "adapter_mode": mode,
            "driver": summary.get("driver"),
            "driver_sha256": summary.get("driver_sha256"),
            "rows": n,
            "insertion": dict(insertion),
            "reconstructor": {k: recon.get(k) for k in ("builder", "gait_clock_side", "gait_clock_phase_offset", "imitation_sound_coords", "imitation_phase_shifts", "imitation_phase_shift", "imitation_phase_samples", "time_window", "clock_n_cycles", "clock_mean_period_s") if k in recon},
            "pins_verified": pins_verified,
            "ik_action": None,
        }
        return cls(t_pre=np.asarray(t_pre, dtype=np.float64), targets=np.asarray(targets, dtype=np.float64), clock=np.asarray(clock, dtype=np.float64), ik_action=None, provenance=provenance)

    @classmethod
    def from_objects(cls, reconstructor_like: Any, base_kin_like: Any, t_pre: Any, *, lookahead_s: float = R.IK_TEACHER_LOOKAHEAD_S, segment_s: float = R.SEGMENT_DURATION_S, t_end: float | None = None, provenance: Mapping[str, Any] | None = None) -> "PrivilegedCache":
        """Cache from injected env-contract objects (env-free; tests): ``targets`` via
        ``reconstructor_like.targets(t)`` (4 float64, V26 order), ``clock`` via
        ``reconstructor_like.clock_sin_cos(t)``, ``ik_action`` via
        :func:`ik_actions_from_base_kin` when ``base_kin_like`` is not ``None``."""
        t = _time_grid(t_pre, what="t_pre")
        targets = np.empty((t.size, TARGET_DIM), dtype=np.float64)
        clock = np.empty((t.size, CLOCK_DIM), dtype=np.float64)
        for i, x in enumerate(t.tolist()):
            tg = np.asarray(reconstructor_like.targets(x), dtype=np.float64)
            if tg.shape != (TARGET_DIM,):
                raise F2RContractError(f"reconstructor.targets(t) must return {TARGET_DIM} values, got shape {tg.shape}")
            targets[i] = tg
            s, c = reconstructor_like.clock_sin_cos(x)
            clock[i, 0] = float(s)
            clock[i, 1] = float(c)
        ik = None if base_kin_like is None else ik_actions_from_base_kin(base_kin_like, t, lookahead_s=lookahead_s, segment_s=segment_s, t_end=t_end)
        recon_prov = getattr(reconstructor_like, "provenance", None)
        prov = {
            "source": "objects",
            "rows": int(t.size),
            "reconstructor": dict(recon_prov) if isinstance(recon_prov, Mapping) else {"type": type(reconstructor_like).__name__},
            "ik_action": None if ik is None else {"source": "ik_actions_from_base_kin", "base_kin": type(base_kin_like).__name__, "lookahead_s": float(lookahead_s), "segment_s": float(segment_s), "t_end": None if t_end is None else float(t_end), "bounds_rad": {k: list(v) for k, v in R.ABSOLUTE_BOUNDS_RAD.items()}, "coords": list(R.PROS_COORDS)},
            **dict(provenance or {}),
        }
        return cls(t_pre=t, targets=targets, clock=clock, ik_action=ik, provenance=prov)

    def with_ik_action(self, base_kin_like: Any, *, lookahead_s: float = R.IK_TEACHER_LOOKAHEAD_S, segment_s: float = R.SEGMENT_DURATION_S, t_end: float | None = None) -> "PrivilegedCache":
        """New cache with ``u_IK`` computed on this grid from an injected base_kin (env-free)."""
        ik = ik_actions_from_base_kin(base_kin_like, self.t_pre, lookahead_s=lookahead_s, segment_s=segment_s, t_end=t_end)
        prov = {**self.provenance, "ik_action": {"source": "ik_actions_from_base_kin", "base_kin": type(base_kin_like).__name__, "lookahead_s": float(lookahead_s), "segment_s": float(segment_s), "t_end": None if t_end is None else float(t_end), "bounds_rad": {k: list(v) for k, v in R.ABSOLUTE_BOUNDS_RAD.items()}, "coords": list(R.PROS_COORDS)}}
        return PrivilegedCache(t_pre=self.t_pre, targets=self.targets, clock=self.clock, ik_action=ik, provenance=prov)

    # -- access ---------------------------------------------------------------------------------
    def lookup(self, t_query: Any) -> np.ndarray:
        """Indices of ``t_query`` (M,) on the grid by EXACT float equality (searchsorted + check).
        Any query time not on the grid is a contract violation (no interpolation, no tolerance)."""
        t = np.asarray(t_query)
        if t.dtype.kind not in "fiu":
            raise F2RContractError(f"lookup expects numeric times, got dtype {t.dtype}")
        t = t.astype(np.float64)
        if t.ndim != 1:
            raise F2RContractError(f"lookup expects a 1-D array of times, got shape {t.shape}")
        if t.size == 0:
            return np.zeros(0, dtype=np.int64)
        grid = self.t_pre
        idx = np.minimum(np.searchsorted(grid, t, side="left"), grid.size - 1)
        hit = grid[idx] == t
        if not np.all(hit):
            bad = t[~hit]
            raise F2RContractError(f"{int(bad.size)} query time(s) not on the privileged cache grid (exact float equality required), e.g. {bad[:3].tolist()}; grid {grid[0]!r}..{grid[-1]!r} ({grid.size} rows)")
        return idx.astype(np.int64)

    def digest(self) -> str:
        """SHA-256 over the arrays (dtype + shape + bytes each; ``ik_action`` absent -> 'none')."""
        h = hashlib.sha256()
        for name in ("t_pre", "targets", "clock", "ik_action"):
            arr = getattr(self, name)
            h.update(name.encode("ascii"))
            h.update(b"\x00")
            h.update((DS.sha256_array(arr) if arr is not None else "none").encode("ascii"))
            h.update(b"\x00")
        return h.hexdigest()


# --- label composition ----------------------------------------------------------------------------


def compose_actions(variant: str, u_T: Any, u_IK: Any | None = None) -> tuple[np.ndarray, dict[str, str]]:
    """Per-joint composition (protocol: well posed at command level, component-wise):
    returns ``(actions (N, 2) float32, {"knee": source, "ankle": source})``."""
    variant = str(variant)
    if variant not in COMPOSITION:
        raise F2RContractError(f"unknown variant {variant!r}: expected one of {VARIANTS}")
    knee_src, ankle_src = COMPOSITION[variant]
    ut = np.asarray(u_T)
    if ut.ndim != 2 or ut.shape[1] != R.ACTION_DIM or ut.dtype != np.float32:
        raise F2RContractError(f"u_T must be (N, {R.ACTION_DIM}) float32, got {ut.shape} {ut.dtype}")
    sources: dict[str, np.ndarray | None] = {"u_T": ut, "u_IK": None}
    if u_IK is not None:
        ui = np.asarray(u_IK)
        if ui.shape != ut.shape or ui.dtype != np.float32:
            raise F2RContractError(f"u_IK must be (N, {R.ACTION_DIM}) float32 like u_T, got {ui.shape} {ui.dtype}")
        sources["u_IK"] = ui
    for src in (knee_src, ankle_src):
        if sources[src] is None:
            raise F2RContractError(f"variant {variant} needs {src} but it is not available")
    actions = np.empty_like(ut)
    actions[:, 0] = sources[knee_src][:, 0]  # type: ignore[index]
    actions[:, 1] = sources[ankle_src][:, 1]  # type: ignore[index]
    return actions, {"knee": knee_src, "ankle": ankle_src}


# --- teacher labeller -----------------------------------------------------------------------------


def _validate_teacher_arrays(arrays: Mapping[str, Any]) -> dict[str, np.ndarray]:
    if not isinstance(arrays, Mapping):
        raise F2RContractError("teacher_arrays must be a mapping (DS.load_actor_arrays output)")
    out: dict[str, np.ndarray] = {}
    for key in TEACHER_KEYS:
        if key not in arrays:
            raise F2RContractError(f"teacher_arrays missing {key}")
        arr = np.asarray(arrays[key], dtype=np.float64)
        if not np.all(np.isfinite(arr)):
            raise F2RContractError(f"teacher array {key} is not finite")
        out[key] = arr
    w1, w2, w3 = out["pi.0.0.weight"], out["pi.0.2.weight"], out["pi.1.weight"]
    if w1.ndim != 2 or w2.ndim != 2 or w3.ndim != 2:
        raise F2RContractError("teacher weight matrices must be 2-D")
    if w1.shape[1] != R.MODULE_WIDTH_39:
        raise F2RContractError(f"teacher first layer must consume {R.MODULE_WIDTH_39} features, got {w1.shape}")
    if w2.shape[1] != w1.shape[0] or w3.shape != (2 * R.ACTION_DIM, w2.shape[0]) or out["pi.0.0.bias"].shape != (w1.shape[0],) or out["pi.0.2.bias"].shape != (w2.shape[0],) or out["pi.1.bias"].shape != (2 * R.ACTION_DIM,):
        raise F2RContractError("teacher ABI mismatch (expected tanh MLP 39-H-H-4)")
    return out


def _validate_spec(spec: Any) -> OA.InsertionSpec:
    if not isinstance(spec, OA.InsertionSpec):
        raise F2RContractError("spec must be an f1_obs_adapter.InsertionSpec")
    if len(spec.names35) != R.ENV_ACTOR_WIDTH or len(spec.names39) != R.MODULE_WIDTH_39 or spec.index != 2 or spec.count != TARGET_DIM or tuple(spec.names35[:2]) != OA.CLOCK_FEATURE_NAMES:
        raise F2RContractError("insertion spec must map 35 -> 39 with the 4 targets at index 2 and the clock pair at 0:2")
    return spec


def _arrays_digest(arrays: Mapping[str, np.ndarray]) -> str:
    h = hashlib.sha256()
    for key in TEACHER_KEYS:
        h.update(key.encode("utf-8"))
        h.update(DS.sha256_array(np.asarray(arrays[key])).encode("ascii"))
    return h.hexdigest()


class TeacherLabeller:
    """Offline teacher labels on the student's visited states.

    ``variant`` in {T1, T2, T3}; ``teacher_arrays`` = ``DS.load_actor_arrays(state, expected_width=39)``;
    ``spec`` = ``OA.InsertionSpec`` (35 -> 39, block of 4 at index 2); ``cache`` = :class:`PrivilegedCache`
    on the env grid of the states to label; ``teacher_digest`` = the pinned actor digest of the
    teacher module (recorded in :meth:`provenance`).  T2/T3 require ``cache.ik_action``."""

    def __init__(self, variant: str, teacher_arrays: Mapping[str, Any], spec: OA.InsertionSpec, cache: PrivilegedCache, *, teacher_digest: str | None = None) -> None:
        variant = str(variant)
        if variant not in VARIANTS:
            raise F2RContractError(f"unknown variant {variant!r}: expected one of {VARIANTS}")
        if not isinstance(cache, PrivilegedCache):
            raise F2RContractError("cache must be a PrivilegedCache")
        self.variant = variant
        self.teacher_arrays = _validate_teacher_arrays(teacher_arrays)
        self.spec = _validate_spec(spec)
        self.cache = cache
        self.teacher_digest = None if teacher_digest is None else str(teacher_digest)
        self.teacher_arrays_sha256 = _arrays_digest(self.teacher_arrays)
        self.cache_digest = cache.digest()
        self.knee_source, self.ankle_source = COMPOSITION[variant]
        self.needs_ik = "u_IK" in (self.knee_source, self.ankle_source)
        if self.needs_ik and cache.ik_action is None:
            raise F2RContractError(f"variant {variant} needs u_IK but the privileged cache carries no ik_action")
        protocol = R.load_protocol()
        self._protocol_variant = dict(protocol["variants"][variant])
        self._composition_rule = str(protocol["action_semantics"]["composition"])
        self._ik_rule = str(protocol["action_semantics"]["ik_teacher"])
        self._protocol_sha256 = C.sha256_file(R.PROTOCOL_JSON)

    # -- student rows -----------------------------------------------------------------------------
    def _student_rows(self, obs35: Any, t_pre: Any) -> tuple[np.ndarray, np.ndarray]:
        obs = np.asarray(obs35)
        if obs.dtype.kind not in "fiu":
            raise F2RContractError(f"obs35 must be float32-castable numeric rows, got dtype {obs.dtype}")
        if obs.ndim != 2 or obs.shape[1] != R.ENV_ACTOR_WIDTH:
            raise F2RContractError(f"obs35 must have shape (N, {R.ENV_ACTOR_WIDTH}), got {obs.shape}")
        if obs.shape[0] == 0:
            raise F2RContractError("no rows to label")
        obs_f32 = np.ascontiguousarray(obs.astype(np.float32))
        if not np.all(np.isfinite(obs_f32)):
            raise F2RContractError("obs35 contains non-finite values")
        c0, c1 = R.CLOCK_COLUMNS
        if not (np.all(obs_f32[:, c0] == np.float32(DEAD_CLOCK[0])) and np.all(obs_f32[:, c1] == np.float32(DEAD_CLOCK[1]))):
            raise F2RContractError(f"obs35[:, {c0}:{c1 + 1}] must be the dead clock constants {DEAD_CLOCK} emitted by the env under the v3 runtime")
        t = np.asarray(t_pre)
        if t.dtype.kind not in "fiu":
            raise F2RContractError(f"t_pre must be numeric, got dtype {t.dtype}")
        t = t.astype(np.float64)
        if t.ndim != 1 or t.shape[0] != obs_f32.shape[0]:
            raise F2RContractError(f"t_pre must have shape ({obs_f32.shape[0]},), got {t.shape}")
        if not np.all(np.isfinite(t)):
            raise F2RContractError("t_pre contains non-finite values")
        return obs_f32, t

    def teacher_observation(self, obs35: Any, t_pre: Any) -> tuple[np.ndarray, np.ndarray]:
        """(obs39 float32, cache indices): the exact A_iso6clk actor input for each student row
        (float32-cast obs35, float32-cast targets at 2:6, float32-cast prescribed clock at 0:2)."""
        obs_f32, t = self._student_rows(obs35, t_pre)
        idx = self.cache.lookup(t)
        obs39 = OA.insert_targets(obs_f32, self.cache.targets[idx].astype(np.float32), self.spec)
        obs39[:, 0] = self.cache.clock[idx, 0].astype(np.float32)
        obs39[:, 1] = self.cache.clock[idx, 1].astype(np.float32)
        proj = OA.project39to35(obs39, self.spec)
        if proj.dtype != obs_f32.dtype or not np.array_equal(proj[:, 2:], obs_f32[:, 2:]):
            raise F2RContractError("aiso6clk projection assertion failed outside the clock pair")
        return obs39, idx

    # -- labels -----------------------------------------------------------------------------------
    def label(self, obs35: Any, t_pre: Any) -> dict[str, Any]:
        """Labels for ``obs35`` (N, 35) float32-castable at the exact pre-step times ``t_pre`` (N,).

        Returns ``actions`` (N, 2) float32 (the label), ``u_T`` (N, 2) float32 (teacher mean,
        A_iso6clk route), ``u_IK`` (N, 2) float32 or ``None``, ``components`` {knee, ankle ->
        'u_T' | 'u_IK'}, ``rows``, ``cache_digest``, ``variant``.  Labels are actions only:
        ``t_pre`` indexes the cache and is not returned as a feature."""
        obs39, idx = self.teacher_observation(obs35, t_pre)
        logits = DS.actor_logits_numpy(self.teacher_arrays, obs39.astype(np.float64))
        if not np.all(np.isfinite(logits)):
            raise F2RContractError("non-finite teacher logits")
        u_T = np.ascontiguousarray(logits[:, : R.ACTION_DIM].astype(np.float32))
        u_IK = None if self.cache.ik_action is None else np.ascontiguousarray(self.cache.ik_action[idx]).astype(np.float32)
        actions, components = compose_actions(self.variant, u_T, u_IK)
        return {"actions": actions, "u_T": u_T, "u_IK": u_IK, "components": components, "rows": int(obs39.shape[0]), "cache_digest": self.cache_digest, "variant": self.variant}

    # -- provenance -------------------------------------------------------------------------------
    def provenance(self) -> dict[str, Any]:
        return {
            "tool": "f2r_labeller",
            "tool_sha256": C.sha256_file(Path(__file__).resolve()),
            "variant": self.variant,
            "variant_role": self._protocol_variant.get("role"),
            "variant_label": self._protocol_variant.get("label"),
            "composition": {"knee": self.knee_source, "ankle": self.ankle_source},
            "composition_rule": self._composition_rule,
            "ik_teacher_rule": self._ik_rule,
            "protocol": {"id": R.PROTOCOL_ID, "f2r_protocol.json_sha256": self._protocol_sha256},
            "teacher": {"name": R.TEACHER["name"], "actor_digest": self.teacher_digest, "arrays_sha256": self.teacher_arrays_sha256, "width": R.MODULE_WIDTH_39, "forward": "numpy float64 tanh MLP on float32-cast obs39 (targets at 2:6, prescribed clock at 0:2 = A_iso6clk), mean head only"},
            "cache": {"digest": self.cache_digest, "rows": self.cache.rows, "has_ik_action": self.cache.ik_action is not None, "provenance": dict(self.cache.provenance)},
            "insertion": self.spec.to_dict(),
            "labels": "actions only (per-joint u_T / u_IK); t_pre indexes the privileged cache and is never a student feature",
            "student_input": f"obs35 float32 (env vector), dead clock constants {DEAD_CLOCK} at {R.CLOCK_COLUMNS}",
        }


# --- cache persistence + CLI (S0: sidecar-only build in tests; S1: env 0-step for u_IK) --------


def save_cache(cache: PrivilegedCache, out_dir: Path, start: str) -> dict[str, Any]:
    """No-clobber ``<out_dir>/privileged_cache_<start>.npz`` + ``.provenance.json`` (digest recorded)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    npz = out_dir / f"privileged_cache_{start}.npz"
    prov = out_dir / f"privileged_cache_{start}.provenance.json"
    if npz.exists() or prov.exists():
        raise FileExistsError(f"privileged cache for {start!r} already exists in {C.rel(out_dir)} (no-clobber)")
    R.reserve_exclusive(npz)  # content-addressed per start: a second build must fail, never suffix/overwrite
    R.reserve_exclusive(prov)
    arrays = {"t_pre": cache.t_pre, "targets": cache.targets, "clock": cache.clock}
    if cache.ik_action is not None:
        arrays["ik_action"] = cache.ik_action
    import io

    buf = io.BytesIO()
    np.savez(buf, **arrays)
    R._atomic_fill(npz, buf.getvalue())
    payload = {"start": start, "rows": cache.rows, "cache_digest": cache.digest(), "has_ik_action": cache.ik_action is not None, "provenance": cache.provenance, "npz_sha256": C.sha256_file(npz)}
    R._atomic_fill(prov, json.dumps(payload, indent=2, default=str).encode("utf-8"))
    return {"npz": C.rel(npz), "provenance": C.rel(prov), "cache_digest": payload["cache_digest"], "npz_sha256": payload["npz_sha256"]}


def load_cache(out_dir: Path, start: str, *, expected_digest: str | None = None) -> PrivilegedCache:
    """Read a saved cache back (digest re-verified against the provenance file / ``expected_digest``)."""
    out_dir = Path(out_dir)
    npz, prov = out_dir / f"privileged_cache_{start}.npz", out_dir / f"privileged_cache_{start}.provenance.json"
    if npz.is_symlink() or prov.is_symlink() or not npz.is_file() or not prov.is_file():
        raise F2RContractError(f"privileged cache for {start!r} missing in {C.rel(out_dir)}")
    meta = _read_json_file(prov, what="cache provenance")
    if meta.get("npz_sha256") != C.sha256_file(npz):
        raise F2RContractError("privileged cache npz digest differs from its provenance file")
    with np.load(npz) as data:
        cache = PrivilegedCache(t_pre=data["t_pre"], targets=data["targets"], clock=data["clock"], ik_action=data["ik_action"] if "ik_action" in data.files else None, provenance=dict(meta.get("provenance") or {}))
    if cache.digest() != meta.get("cache_digest") or (expected_digest is not None and cache.digest() != expected_digest):
        raise F2RContractError("privileged cache digest mismatch after reload")
    return cache


def build_caches_from_anchors(out_dir: Path, *, starts: Sequence[str] = R.STARTS) -> dict[str, Any]:
    """Sidecar-only caches (targets + prescribed clock on the exact anchor grid; no ``u_IK``)
    from the pinned A_iso6clk anchors, read-only; pins re-verified per start."""
    result: dict[str, Any] = {}
    for start in starts:
        cache = PrivilegedCache.from_adapter_sidecar(R.ANCHORS[start]["job_dir"], pins=R.ANCHORS[start])
        result[start] = save_cache(cache, out_dir, start)
    return result


class _EnvCaptured(RuntimeError):
    pass


def capture_runtime_env_zero_steps(start: str, checkpoint: Path, python_exe: str | None = None) -> Any:
    """S1 only: build the v3_canonical env exactly as ``rollout_eval`` does for ``start`` (same
    resolved yaml, flags and checkpoint) and return the *unwrapped* env right after
    ``env_factory.make_cmc_env`` — before any ``reset``/``step`` (0 steps).  Mirrors the F1
    ``_EnvFactoryProxy`` hook; ``rollout_eval`` itself is not modified.  Not exercised in S0."""
    sys.path.insert(0, str(C.BASELINE_DIR))
    import rollout_eval as RE  # production, unchanged

    tmp = R.portable_tempdir("f2r_cache_env_")
    argv = ["--checkpoint", str(checkpoint), "--no-auto-config", "--config", str(R.RUNTIME_CONFIG), "--episode-start-offset-s", repr(float(R.EXACT_STARTS[start])), "--action-selection", "deterministic", "--seed", str(R.DET_SEED), "--output-dir", str(tmp), "--worker-process", *list(C.RUNTIMES["v3_canonical"]["extra_args"])]
    saved_argv = sys.argv
    sys.argv = [str(R.ROLLOUT_EVAL), *argv]
    try:
        args = RE.parse_args()
    finally:
        sys.argv = saved_argv
    RE._load_inference_stack()
    real_factory = RE.env_factory
    holder: dict[str, Any] = {}

    class _CaptureFactory:
        def make_cmc_env(self, env_config: Any) -> Any:
            env = real_factory.make_cmc_env(env_config)
            holder["env"] = env
            raise _EnvCaptured("env captured before reset (0 steps)")

        def __getattr__(self, name: str) -> Any:
            return getattr(real_factory, name)

    RE.env_factory = _CaptureFactory()
    try:
        RE.run(args)
    except _EnvCaptured:
        pass
    finally:
        RE.env_factory = real_factory
    if "env" not in holder:
        raise F2RContractError("rollout_eval did not build the env (hook order)")
    return holder["env"].unwrapped


def build_caches_with_ik_s1(out_dir: Path, *, starts: Sequence[str] = R.STARTS) -> dict[str, Any]:
    """S1 only: anchor sidecar caches + ``u_IK`` from the env's own ``base_kin`` (captured at
    construction, 0 steps); the reconstructed targets/clock of the captured env are cross-checked
    bit-exactly against the sidecar on the whole grid before the cache is saved."""
    import f1_rollout_aiso as RA  # F1 driver library (immutable): reconstructor from the env's own classes

    result: dict[str, Any] = {}
    for start in starts:
        sidecar = PrivilegedCache.from_adapter_sidecar(R.ANCHORS[start]["job_dir"], pins=R.ANCHORS[start])
        base_env = capture_runtime_env_zero_steps(start, Path(R.INIT_PRIMARY["module"]))
        recon = RA.build_reconstructor_from_env(base_env)
        t_end = float(base_env.cfg.t_end)
        from_env = PrivilegedCache.from_objects(recon, base_env.base_kin, sidecar.t_pre, t_end=t_end, provenance={"env_capture": "rollout_eval env at construction, 0 steps", "start": start})
        if not (np.array_equal(from_env.targets.astype(np.float32), sidecar.targets.astype(np.float32)) and np.array_equal(from_env.clock.astype(np.float32), sidecar.clock.astype(np.float32))):
            raise F2RContractError(f"{start}: targets/clock reconstructed from the captured env differ from the pinned sidecar")
        cache = PrivilegedCache(t_pre=sidecar.t_pre, targets=sidecar.targets, clock=sidecar.clock, ik_action=from_env.ik_action, provenance={**sidecar.provenance, "ik_action": from_env.provenance["ik_action"], "env_crosscheck": "targets/clock bit-exact (float32) vs sidecar on the full grid"})
        result[start] = save_cache(cache, out_dir, start)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="F2R privileged cache / teacher labeller CLI.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build-cache", help="materialise the privileged caches (no-clobber)")
    b.add_argument("--out-dir", required=True)
    b.add_argument("--from-anchors", action="store_true", help="sidecar-only (targets + prescribed clock), no u_IK; env-free")
    b.add_argument("--with-ik", action="store_true", help="S1 only: add u_IK from the env's base_kin captured at construction (0 steps)")
    b.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.cmd == "build-cache":
        if args.with_ik:
            if args.authorized_stage != "S1":
                raise SystemExit("build-cache --with-ik builds the env (0 steps): not authorised in S0; pass --authorized-stage S1 after the architect's go")
            out = build_caches_with_ik_s1(Path(args.out_dir))
        elif args.from_anchors:
            out = build_caches_from_anchors(Path(args.out_dir))
        else:
            raise SystemExit("choose --from-anchors (env-free) or --with-ik (S1)")
        print(json.dumps(out, indent=2))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
