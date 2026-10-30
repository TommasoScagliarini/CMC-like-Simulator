"""P0 tooling — offline observability of the prescribed gait phase from the
35D deployable features (F2R bridge, protocol key ``p0``).

S0 scope: this module is **tooling only**.  It is exercised on synthetic
``StartData`` in ``test_f2r_observability.py``; it is NOT executed on real
data in S0 (``f2r_protocol.json`` -> ``p0.executed_in_S0 = false``).  The
read-only loader ``load_start_data`` is provided for S1 and reads existing
rollout artefacts without modifying them.

Question P0 answers (report §13): can a stateless estimator infer the
prescribed sound-side phase ``phi(t_pre)`` from the 35D observation alone?

* **data**: JUL_H0 deterministic F0 jobs under v3 (3 starts) = test states;
  the three A_iso6clk anchors (3 starts) fixed in every train set;
* **folds**: LOSO over the 3 JUL_H0 starts — train = the OTHER two JUL_H0
  starts + the 3 anchors, test = the held-out JUL_H0 start only;
  standardisation fitted on train only; a structural row-hash check proves
  that no held-out row is bitwise present in the train matrix;
* **label**: ``c(t_pre) = (sin, cos)(2 pi phi)`` with ``phi`` from the
  ``GaitPhaseClock`` formula on the prescribed heel strikes (offset 0),
  computed offline; never a feature;
* **forbidden inputs**: time, step index, ``gait_phase_sin/cos`` (dead
  constants, hard-zero in the student) — asserted on column NAMES through
  ``f2r_common.assert_no_forbidden_inputs`` at every entry point;
* **variants**: ``pre_cycle`` (online clock triplet 14/15/16 excluded) and
  ``post_cycle`` (triplet allowed, reported separately);
* **estimators**: ridge (closed form on standardised features, intercept
  unpenalised) and a 64-64 tanh MLP (Adam 1e-3, 200 epochs, seed 2026,
  deterministic torch);
* **metric**: circular error ``|phi_hat - phi| mod 1`` with
  ``phi_hat = atan2(sin, cos) / 2 pi``;
* **PASS rule** (frozen): MLP, ``pre_cycle`` variant, >= 2 folds of 3 with
  median <= 0.05 and p90 <= 0.20; the ``plus020`` fold is always reported.
  The rule is applied to ALL test rows of the held-out start (the
  before/after-first-cycle subsets are reported alongside).

Tooling details fixed here (not in the JSON) and recorded in every result:
the MLP mini-batch size (``MLP_BATCH_SIZE_DEFAULT``), the ridge form
(sum of squares + ``lam * ||W||^2``, intercept unpenalised), the zero-std
guard of the standardisation (constant train columns get std 1).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402  (puts F0/F1 on sys.path)
import f0_common as C  # noqa: E402  (F0 library, immutable)
import f1_obs_adapter as OA  # noqa: E402  (F1 library, immutable)

TRACE_FILE = "rollout_policy_trace.json"
RESET_FILE = "rollout_reset_diagnostics.json"
SUMMARY_FILE = "rollout_summary.json"
ONLINE_CLOCK_DISABLED = (0.0, 1.0, 0.0)  # (sin, cos, cycle duration) before the first online cycle
ESTIMATORS = ("ridge", "mlp")
VARIANTS = ("pre_cycle", "post_cycle")
MLP_BATCH_SIZE_DEFAULT = 128
RIDGE_FORM = "argmin_W ||Z W - Y||^2 + lam ||W||^2 on standardised features Z (intercept column unpenalised), closed form"
RULE_ROWS = "all"  # test rows of the held-out start on which the frozen PASS rule is applied
TWO_PI = 2.0 * math.pi


# --- phase arithmetic -------------------------------------------------------------------


def phase_from_sincos(sc: np.ndarray) -> np.ndarray:
    """``(N, 2)`` ``(sin, cos)`` rows -> phase in ``[0, 1)`` via ``atan2(sin, cos) / 2 pi mod 1``."""
    arr = np.asarray(sc, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise R.F2RContractError(f"phase_from_sincos expects (N, 2) (sin, cos) rows, got {arr.shape}")
    phi = np.arctan2(arr[:, 0], arr[:, 1]) / TWO_PI
    return np.mod(phi, 1.0)


def sincos_from_phase(phi: np.ndarray) -> np.ndarray:
    """Phase ``(N,)`` -> ``(N, 2)`` ``(sin, cos)(2 pi phi)`` (the P0 regression target)."""
    p = np.asarray(phi, dtype=np.float64).reshape(-1)
    return np.stack([np.sin(TWO_PI * p), np.cos(TWO_PI * p)], axis=1)


def circular_error(phi_hat: np.ndarray, phi: np.ndarray) -> np.ndarray:
    """``|((phi_hat - phi + 0.5) mod 1) - 0.5|`` in ``[0, 0.5]`` (cycle fraction)."""
    a = np.asarray(phi_hat, dtype=np.float64).reshape(-1)
    b = np.asarray(phi, dtype=np.float64).reshape(-1)
    if a.shape != b.shape:
        raise R.F2RContractError(f"circular_error shape mismatch {a.shape} vs {b.shape}")
    return np.abs(np.mod(a - b + 0.5, 1.0) - 0.5)


def _heel_strikes(heel_strikes_sorted: Sequence[float]) -> np.ndarray:
    hs = np.asarray([float(x) for x in heel_strikes_sorted], dtype=np.float64).reshape(-1)
    if hs.size < 2 or not np.all(np.isfinite(hs)) or np.any(np.diff(hs) <= 0.0):
        raise R.F2RContractError("prescribed_phase needs >= 2 finite, strictly increasing heel strikes (sorted)")
    return hs


def prescribed_phase(t: float, heel_strikes_sorted: Sequence[float], offset: float = 0.0) -> float:
    """``GaitPhaseClock`` formula (osim_trj_cmc_like.py) on sorted heel strikes:

    * ``t < hs[0]``:   ``x = (t - hs[0]) / (hs[1] - hs[0])``
    * ``t >= hs[-1]``: ``x = (t - hs[-1]) / (hs[-1] - hs[-2])``
    * else bracket ``k`` (``searchsorted right - 1``): ``x = (t - hs[k]) / (hs[k+1] - hs[k])``

    then ``x -= offset`` and the phase is ``x - floor(x)`` in ``[0, 1)``."""
    hs = _heel_strikes(heel_strikes_sorted)
    t = float(t)
    if not math.isfinite(t):
        raise R.F2RContractError("prescribed_phase: non-finite time")
    if t < hs[0]:
        x = (t - hs[0]) / (hs[1] - hs[0])
    elif t >= hs[-1]:
        x = (t - hs[-1]) / (hs[-1] - hs[-2])
    else:
        k = int(np.searchsorted(hs, t, side="right")) - 1
        x = (t - hs[k]) / (hs[k + 1] - hs[k])
    x -= float(offset)
    return float(x - np.floor(x))


def prescribed_phase_array(times: Sequence[float], heel_strikes_sorted: Sequence[float], offset: float = 0.0) -> np.ndarray:
    """Vectorised convenience wrapper around ``prescribed_phase`` (same formula, row by row)."""
    hs = _heel_strikes(heel_strikes_sorted)
    return np.asarray([prescribed_phase(t, hs, offset) for t in np.asarray(times, dtype=np.float64).reshape(-1)], dtype=np.float64)


# --- feature selection (fail-closed on names) ----------------------------------------------


def _names35(names35: Sequence[str]) -> tuple[str, ...]:
    names = tuple(str(n) for n in names35)
    if names != R.FEATURE_NAMES_35:
        raise R.F2RContractError("select_columns: the names must be the pinned 35D manifest features in manifest order")
    return names


def assert_extra_columns(extra_columns: Sequence[str], *, pre_cycle: bool) -> list[int]:
    """Validate an explicit request for additional columns.  Any forbidden name
    (``R.FORBIDDEN_P0_INPUT_NAMES``: time, step index, prescribed clock) raises
    ``F2RContractError``; the online clock triplet raises in the pre-cycle variant;
    unknown names raise.  Returns the manifest indices of the accepted names."""
    names = [str(n) for n in extra_columns]
    for n in names:
        if n in R.FORBIDDEN_P0_INPUT_NAMES:
            raise R.F2RContractError(f"explicitly requested forbidden P0 input {n!r} (time/index/prescribed clock)")
    R.assert_no_forbidden_inputs(names, pre_cycle=pre_cycle)
    idx = [R.IDX[n] for n in names]
    if any(i in R.CLOCK_COLUMNS for i in idx):
        raise R.F2RContractError("the prescribed clock columns (0, 1) can never be P0 inputs")
    return idx


def select_columns(names35: Sequence[str], *, pre_cycle: bool, extra_columns: Sequence[str] = ()) -> list[int]:
    """Manifest indices used by a P0 variant: all 35 features except
    ``R.CLOCK_COLUMNS`` (always) and, when ``pre_cycle``, except
    ``R.ONLINE_CLOCK_TRIPLET``.  ``extra_columns`` is an explicit request
    checked by ``assert_extra_columns`` (forbidden names fail closed; allowed
    names are already included).  The selected names are re-asserted through
    ``R.assert_no_forbidden_inputs``."""
    names = _names35(names35)
    excluded = set(R.CLOCK_COLUMNS)
    if pre_cycle:
        excluded |= set(R.ONLINE_CLOCK_TRIPLET)
    extra = assert_extra_columns(extra_columns, pre_cycle=pre_cycle)
    for i in extra:
        if i in excluded:
            raise R.F2RContractError(f"column {names[i]!r} is excluded by the {'pre' if pre_cycle else 'post'}-cycle variant")
    cols = [i for i in range(len(names)) if i not in excluded]
    R.assert_no_forbidden_inputs([names[i] for i in cols], pre_cycle=pre_cycle)
    return cols


def assert_columns(columns: Sequence[int]) -> tuple[list[int], list[str], bool]:
    """Validate a column index list against the pinned manifest (unique, in range,
    no prescribed clock, only manifest names); returns (indices, names, pre_cycle)
    where ``pre_cycle`` is True iff no online clock triplet column is used."""
    cols = [int(c) for c in columns]
    if not cols or len(set(cols)) != len(cols) or any(c < 0 or c >= R.ENV_ACTOR_WIDTH for c in cols):
        raise R.F2RContractError(f"invalid column list {cols}")
    if any(c in R.CLOCK_COLUMNS for c in cols):
        raise R.F2RContractError("prescribed clock columns (0, 1) are forbidden P0 inputs")
    names = [R.FEATURE_NAMES_35[c] for c in cols]
    pre_cycle = not any(c in R.ONLINE_CLOCK_TRIPLET for c in cols)
    R.assert_no_forbidden_inputs(names, pre_cycle=pre_cycle)
    return cols, names, pre_cycle


# --- data containers ----------------------------------------------------------------------


def first_cycle_step_from_X(X: np.ndarray) -> int:
    """Index of the first row whose online clock triplet (14, 15, 16) differs
    from the disabled value (0, 1, 0); ``N`` if it never does."""
    arr = np.asarray(X, dtype=np.float64)
    trip = arr[:, list(R.ONLINE_CLOCK_TRIPLET)]
    active = np.any(trip != np.asarray(ONLINE_CLOCK_DISABLED, dtype=np.float64), axis=1)
    hits = np.flatnonzero(active)
    return int(hits[0]) if hits.size else int(arr.shape[0])


@dataclass
class StartData:
    """Rows of one start: ``X`` (N, 35) float64 observation rows (the env's
    ``actor_observation_vector_before``), ``phi`` (N,) prescribed phase in
    [0, 1) at the same pre-step times, ``first_cycle_step`` (index of the first
    row with an informative online clock triplet, N if never), ``source``
    (free text provenance, e.g. ``jul_det_v3`` / ``anchor_aiso6clk`` / ``synthetic``)."""

    start: str
    X: np.ndarray
    phi: np.ndarray
    first_cycle_step: int
    source: str
    t_pre: np.ndarray | None = None  # exact pre-step env times (row k observed at t_pre[k]); t_pre[0] == reset_time
    reset_time: float | None = None  # env reset time of the episode (exact float)

    def __post_init__(self) -> None:
        self.X = np.ascontiguousarray(np.asarray(self.X, dtype=np.float64))
        self.phi = np.asarray(self.phi, dtype=np.float64).reshape(-1)
        if self.t_pre is not None:
            self.t_pre = np.ascontiguousarray(np.asarray(self.t_pre, dtype=np.float64).reshape(-1))
            if self.t_pre.shape[0] != self.X.shape[0] or not np.all(np.isfinite(self.t_pre)) or (self.t_pre.size > 1 and np.any(np.diff(self.t_pre) <= 0.0)):
                raise R.F2RContractError("StartData.t_pre must be finite, strictly increasing and one per row")
        if self.reset_time is not None:
            self.reset_time = float(self.reset_time)
            if not math.isfinite(self.reset_time):
                raise R.F2RContractError("StartData.reset_time must be finite")
            if self.t_pre is not None and self.t_pre[0] != self.reset_time:
                raise R.F2RContractError("StartData.t_pre[0] must equal reset_time exactly (first pre-action observation)")
        if self.X.ndim != 2 or self.X.shape[1] != R.ENV_ACTOR_WIDTH or self.X.shape[0] == 0:
            raise R.F2RContractError(f"StartData.X must be (N, {R.ENV_ACTOR_WIDTH}) with N > 0, got {self.X.shape}")
        if self.phi.shape[0] != self.X.shape[0]:
            raise R.F2RContractError("StartData.phi length != rows of X")
        if not np.all(np.isfinite(self.X)) or not np.all(np.isfinite(self.phi)):
            raise R.F2RContractError("StartData contains non-finite values")
        if np.any(self.phi < 0.0) or np.any(self.phi >= 1.0):
            raise R.F2RContractError("StartData.phi must lie in [0, 1)")
        self.first_cycle_step = int(self.first_cycle_step)
        if self.first_cycle_step < 0 or self.first_cycle_step > self.X.shape[0]:
            raise R.F2RContractError("StartData.first_cycle_step out of range")
        self.start = str(self.start)
        self.source = str(self.source)

    @property
    def n(self) -> int:
        return int(self.X.shape[0])

    def describe(self) -> dict[str, Any]:
        return {"start": self.start, "rows": self.n, "first_cycle_step": self.first_cycle_step, "source": self.source, "rows_pre_cycle": self.first_cycle_step, "rows_post_cycle": self.n - self.first_cycle_step, "has_t_pre": self.t_pre is not None, "reset_time": self.reset_time}


def _start_from_offset(offset: float) -> str:
    for start, exact in R.EXACT_STARTS.items():
        if float(offset) == float(exact):
            return str(start)
    raise R.F2RContractError(f"episode_start_offset_s {offset!r} is not one of the exact starts {R.EXACT_STARTS}")


def load_start_data(job_dir: Path, heel_strikes: Sequence[float], *, offset: float = 0.0, source: str, start: str | None = None) -> StartData:
    """Read-only loader (S1) of one rollout_eval job directory into ``StartData``:
    ``X`` = ``actor_observation_vector_before`` rows of ``rollout_policy_trace.json``,
    pre-step times from ``rollout_reset_diagnostics.json`` (``time``) through
    ``OA.t_pre_from_trace``, ``phi`` = ``prescribed_phase(t_pre)`` on the given
    heel strikes, ``first_cycle_step`` from the online clock triplet.  ``start``
    defaults to the exact ``episode_start_offset_s`` of the summary (fail-closed).
    Time and step index are consumed only to compute the label; they are not
    stored as features."""
    job_dir = Path(job_dir)
    trace_path, reset_path, summary_path = job_dir / TRACE_FILE, job_dir / RESET_FILE, job_dir / SUMMARY_FILE
    for p in (trace_path, reset_path, summary_path):
        if p.is_symlink() or not p.is_file():
            raise R.F2RContractError(f"missing job file (or symlink): {C.rel(p)}")
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    reset = json.loads(reset_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise R.F2RContractError(f"empty or malformed trace: {C.rel(trace_path)}")
    steps = [int(r["step"]) for r in rows]
    if steps != list(range(1, len(rows) + 1)):
        raise R.F2RContractError("trace steps are not 1..N")
    reset_time = reset.get("time")
    if not isinstance(reset_time, (int, float)) or isinstance(reset_time, bool):
        raise R.F2RContractError("reset diagnostics without a numeric 'time'")
    try:
        t_pre = OA.t_pre_from_trace(float(reset_time), [float(r["time"]) for r in rows])
    except OA.AdapterError as exc:
        raise R.F2RContractError(f"trace time grid invalid: {exc}") from exc
    X = np.asarray([r["actor_observation_vector_before"] for r in rows], dtype=np.float64)
    if X.shape != (len(rows), R.ENV_ACTOR_WIDTH) or not np.all(np.isfinite(X)):
        raise R.F2RContractError(f"obs rows malformed in {C.rel(job_dir)}")
    inferred = _start_from_offset(summary.get("episode_start_offset_s"))
    if start is not None and str(start) != inferred:
        raise R.F2RContractError(f"declared start {start!r} != summary start {inferred!r}")
    phi = prescribed_phase_array(t_pre, heel_strikes, offset)
    return StartData(start=inferred, X=X, phi=phi, first_cycle_step=first_cycle_step_from_X(X), source=str(source), t_pre=np.asarray(t_pre, dtype=np.float64), reset_time=float(reset_time))


# --- folds --------------------------------------------------------------------------------


@dataclass(frozen=True)
class FoldSpec:
    """LOSO fold: ``held_out`` JUL start (test), ``train_starts`` = the other JUL
    starts, ``anchors`` = the anchor starts fixed in train."""

    held_out: str
    train_starts: tuple[str, ...]
    anchors: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {"held_out": self.held_out, "train_starts": list(self.train_starts), "anchors": list(self.anchors)}


def build_folds(starts: Sequence[str] = R.STARTS) -> list[FoldSpec]:
    """Three LOSO folds over the JUL_H0 starts; anchors of all three starts in every train set."""
    s = tuple(str(x) for x in starts)
    if len(s) != 3 or len(set(s)) != 3:
        raise R.F2RContractError(f"build_folds expects 3 distinct starts, got {s}")
    return [FoldSpec(held_out=h, train_starts=tuple(x for x in s if x != h), anchors=s) for h in s]


# --- estimators -----------------------------------------------------------------------------


def fit_ridge(Xtr: np.ndarray, Ytr: np.ndarray, lam: float) -> Callable[[np.ndarray], np.ndarray]:
    """Closed-form ridge on (already standardised) features with an unpenalised
    intercept: ``W = (Z^T Z + lam P)^-1 Z^T Y``, ``Z = [X, 1]``, ``P = diag(1..1, 0)``.
    Returns ``predict(X) -> (M, 2)`` float64."""
    X = np.asarray(Xtr, dtype=np.float64)
    Y = np.asarray(Ytr, dtype=np.float64)
    if X.ndim != 2 or Y.ndim != 2 or X.shape[0] != Y.shape[0] or Y.shape[1] != 2 or X.shape[0] == 0:
        raise R.F2RContractError(f"fit_ridge shapes: X {X.shape}, Y {Y.shape}")
    lam = float(lam)
    if not (lam >= 0.0):
        raise R.F2RContractError("ridge lambda must be >= 0")
    Z = np.concatenate([X, np.ones((X.shape[0], 1), dtype=np.float64)], axis=1)
    P = np.eye(Z.shape[1], dtype=np.float64)
    P[-1, -1] = 0.0
    W = np.linalg.solve(Z.T @ Z + lam * P, Z.T @ Y)
    if not np.all(np.isfinite(W)):
        raise R.F2RContractError("ridge solution is not finite")
    d = X.shape[1]

    def predict(Xq: np.ndarray) -> np.ndarray:
        Q = np.asarray(Xq, dtype=np.float64)
        if Q.ndim != 2 or Q.shape[1] != d:
            raise R.F2RContractError(f"ridge predict expects (M, {d}), got {Q.shape}")
        return Q @ W[:-1] + W[-1]

    predict.weights = W  # type: ignore[attr-defined]
    predict.kind = "ridge"  # type: ignore[attr-defined]
    return predict


def fit_mlp(Xtr: np.ndarray, Ytr: np.ndarray, hparams: Mapping[str, Any], seed: int) -> Callable[[np.ndarray], np.ndarray]:
    """Deterministic torch MLP ``d -> hidden[0] -> hidden[1] -> 2`` (tanh), MSE on
    ``(sin, cos)``, Adam ``lr``, ``epochs`` epochs of shuffled mini-batches
    (``batch_size``, default ``MLP_BATCH_SIZE_DEFAULT``; shuffling from
    ``numpy.random.default_rng(seed)``).  ``torch.manual_seed(seed)``,
    ``use_deterministic_algorithms(True)``, single thread, CPU.  Returns
    ``predict(X) -> (M, 2)`` float64 (no grad)."""
    import torch

    X = np.asarray(Xtr, dtype=np.float32)
    Y = np.asarray(Ytr, dtype=np.float32)
    if X.ndim != 2 or Y.ndim != 2 or X.shape[0] != Y.shape[0] or Y.shape[1] != 2 or X.shape[0] == 0:
        raise R.F2RContractError(f"fit_mlp shapes: X {X.shape}, Y {Y.shape}")
    hidden = [int(h) for h in hparams.get("hidden", (64, 64))]
    activation = str(hparams.get("activation", "tanh"))
    if activation != "tanh" or len(hidden) != 2 or any(h <= 0 for h in hidden):
        raise R.F2RContractError(f"P0 MLP is frozen to two tanh hidden layers, got hidden={hidden} activation={activation}")
    lr = float(hparams.get("lr", 1e-3))
    epochs = int(hparams.get("epochs", 200))
    batch = int(hparams.get("batch_size", MLP_BATCH_SIZE_DEFAULT))
    if epochs <= 0 or batch <= 0 or not (lr > 0.0):
        raise R.F2RContractError("invalid MLP hyperparameters")
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)
    torch.manual_seed(int(seed))
    d = X.shape[1]
    model = torch.nn.Sequential(torch.nn.Linear(d, hidden[0]), torch.nn.Tanh(), torch.nn.Linear(hidden[0], hidden[1]), torch.nn.Tanh(), torch.nn.Linear(hidden[1], 2))
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    xt = torch.as_tensor(X)
    yt = torch.as_tensor(Y)
    rng = np.random.default_rng(int(seed))
    n = X.shape[0]
    for _epoch in range(epochs):
        perm = rng.permutation(n)
        for s0 in range(0, n, batch):
            idx = torch.as_tensor(perm[s0: s0 + batch])
            opt.zero_grad(set_to_none=True)
            loss = torch.mean((model(xt[idx]) - yt[idx]) ** 2)
            loss.backward()
            opt.step()
    model.eval()

    def predict(Xq: np.ndarray) -> np.ndarray:
        Q = np.asarray(Xq, dtype=np.float32)
        if Q.ndim != 2 or Q.shape[1] != d:
            raise R.F2RContractError(f"mlp predict expects (M, {d}), got {Q.shape}")
        with torch.no_grad():
            return model(torch.as_tensor(Q)).numpy().astype(np.float64)

    predict.kind = "mlp"  # type: ignore[attr-defined]
    predict.hparams = {"hidden": hidden, "activation": activation, "lr": lr, "epochs": epochs, "batch_size": batch, "seed": int(seed)}  # type: ignore[attr-defined]
    return predict


# --- fold evaluation ----------------------------------------------------------------------


def _row_keys(M: np.ndarray) -> set[bytes]:
    arr = np.ascontiguousarray(np.asarray(M, dtype=np.float64))
    return {row.tobytes() for row in arr}


def _metrics(err: np.ndarray) -> dict[str, Any]:
    e = np.asarray(err, dtype=np.float64).reshape(-1)
    if e.size == 0:
        return {"n": 0, "median": None, "p90": None, "mean": None, "max": None}
    return {"n": int(e.size), "median": float(np.median(e)), "p90": float(np.percentile(e, 90)), "mean": float(np.mean(e)), "max": float(np.max(e))}


def standardise_stats(Xtr_cols: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """Train-only mean/std (ddof 0); columns with zero std get std 1 (reported)."""
    mean = np.mean(Xtr_cols, axis=0)
    std = np.std(Xtr_cols, axis=0)
    constant = [int(i) for i in np.flatnonzero(std == 0.0)]
    std = np.where(std == 0.0, 1.0, std)
    return mean, std, constant


def _row_count(M: np.ndarray, row: np.ndarray) -> int:
    key = np.ascontiguousarray(np.asarray(row, dtype=np.float64)).tobytes()
    return sum(1 for r in np.ascontiguousarray(np.asarray(M, dtype=np.float64)) if r.tobytes() == key)


def apply_reset_row_rule(fold: FoldSpec, jul: Mapping[str, StartData], anchors: Mapping[str, StartData], *, cols: Sequence[int]) -> tuple[list[tuple[str, np.ndarray, np.ndarray]], dict[str, Any]]:
    """Amendment B (preregistered, pre-metric): in each LOSO fold the TRAIN set excludes exactly
    one row — the reset row (index 0 = first pre-action observation, ``t_pre[0] == reset_time``)
    of the anchor of the held-out start — because the env reset is policy-independent and that
    row is bitwise identical to held-out TEST row 0, which stays in TEST.  Structural validation
    (fail-closed ``F2RContractError``): time provenance present on both; ``t_pre[0] == reset_time``
    on both and equal across the two; anchor row 0 bitwise identical to test row 0 (a missing
    expected reset duplicate fails); anchor row 0 unique within its anchor and test row 0 unique
    within the test set (ambiguity fails); after the exclusion **no** held-out row may remain in
    the train matrix (full rows and selected columns): any additional or non-reset duplicate fails.
    Returns the train parts ``(name, X, phi)`` and the per-fold provenance record."""
    if fold.held_out in fold.train_starts:
        raise R.F2RContractError(f"fold leakage: held-out start {fold.held_out!r} is in train_starts {fold.train_starts}")
    test = jul[fold.held_out]
    if fold.held_out not in fold.anchors:
        raise R.F2RContractError("the reset-row rule requires the anchor of the held-out start in the fold anchors (protocol: all 3 anchors in train)")
    anchor_h = anchors[fold.held_out]
    if anchor_h.start != fold.held_out or test.start != fold.held_out:
        raise R.F2RContractError("held-out anchor/test StartData.start mismatch")
    for name, sd in (("held-out test", test), ("same-start anchor", anchor_h)):
        if sd.t_pre is None or sd.reset_time is None:
            raise R.F2RContractError(f"reset-row rule: {name} StartData lacks exact time provenance (t_pre / reset_time)")
        if sd.t_pre[0] != sd.reset_time:
            raise R.F2RContractError(f"reset-row rule: {name} row 0 is not the reset observation (t_pre[0] {sd.t_pre[0]!r} != reset_time {sd.reset_time!r})")
    if anchor_h.reset_time != test.reset_time:
        raise R.F2RContractError(f"reset-row rule: reset time of the same-start anchor ({anchor_h.reset_time!r}) != held-out test ({test.reset_time!r})")
    if anchor_h.X[0].tobytes() != test.X[0].tobytes():
        raise R.F2RContractError("reset-row rule: missing expected reset duplicate (anchor row 0 of the held-out start is not bitwise identical to held-out test row 0)")
    if _row_count(anchor_h.X, anchor_h.X[0]) != 1:
        raise R.F2RContractError("reset-row rule: ambiguous — the anchor reset row occurs more than once within the same-start anchor")
    if _row_count(test.X, test.X[0]) != 1:
        raise R.F2RContractError("reset-row rule: ambiguous — the held-out reset row occurs more than once within the held-out test set")
    parts_before: list[tuple[str, np.ndarray, np.ndarray]] = [(f"jul:{s}", jul[s].X, jul[s].phi) for s in fold.train_starts] + [(f"anchor:{a}", anchors[a].X, anchors[a].phi) for a in fold.anchors]
    Xb = np.concatenate([x for _, x, _ in parts_before], axis=0)
    before_full = len(_row_keys(test.X) & _row_keys(Xb))
    before_sel = len(_row_keys(test.X[:, cols]) & _row_keys(Xb[:, cols]))
    parts_after: list[tuple[str, np.ndarray, np.ndarray]] = [(f"jul:{s}", jul[s].X, jul[s].phi) for s in fold.train_starts]
    for a in fold.anchors:
        sd = anchors[a]
        if a == fold.held_out:
            parts_after.append((f"anchor:{a}[1:]", sd.X[1:], sd.phi[1:]))
        else:
            parts_after.append((f"anchor:{a}", sd.X, sd.phi))
    Xa = np.concatenate([x for _, x, _ in parts_after], axis=0)
    after_full = len(_row_keys(test.X) & _row_keys(Xa))
    after_sel = len(_row_keys(test.X[:, cols]) & _row_keys(Xa[:, cols]))
    if after_full or after_sel:
        raise R.F2RContractError(f"leakage: {after_full} held-out rows ({after_sel} on the selected columns) are bitwise present in the train matrix after the reset-row exclusion (additional/non-reset duplicate)")
    provenance = {
        "amendment": "B_same_start_anchor_reset_row_excluded_from_train",
        "test_rows_excluded": 0,
        "test_rows": int(test.n),
        "train_anchor_reset_rows_excluded": 1,
        "excluded": {"set": "train", "source": f"anchor:{fold.held_out}", "start": fold.held_out, "index": 0, "t_pre": float(anchor_h.t_pre[0]), "reset_time": float(anchor_h.reset_time), "reason": "first pre-action observation of the same-start anchor (policy-independent env reset), bitwise identical to held-out test row 0", "bitwise_identical_to_test_row0": True},
        "anchor_rows_retained": int(sum(anchors[a].n for a in fold.anchors) - 1),
        "train_rows_before": int(Xb.shape[0]),
        "train_rows_after": int(Xa.shape[0]),
        "collisions_before_rule": {"full_rows": int(before_full), "selected_columns": int(before_sel)},
        "collisions_after_rule": {"full_rows": int(after_full), "selected_columns": int(after_sel)},
        "strict_post_filter_assert": "no held-out row bitwise present in train (full rows and selected columns) — passed",
    }
    if before_full != 1 or before_sel < 1:
        raise R.F2RContractError(f"reset-row rule: expected exactly one full-row collision before the exclusion (the reset duplicate), found {before_full} ({before_sel} on the selected columns)")
    return parts_after, provenance


def run_fold(fold: FoldSpec, jul: Mapping[str, StartData], anchors: Mapping[str, StartData], *, columns: Sequence[int], estimator: str, protocol_p0: Mapping[str, Any]) -> dict[str, Any]:
    """One LOSO fold.  train = rows of ``jul[s]`` for ``s in fold.train_starts``
    + rows of ``anchors[a]`` for ``a in fold.anchors``; test = ``jul[fold.held_out]``.

    Structural asserts (``F2RContractError``): the held-out start is not in
    ``fold.train_starts``; no test row is bitwise present in the train matrix
    (row-byte set check on the full 35-wide rows AND on the selected columns);
    columns pass ``assert_columns``.  Standardisation mean/std come from the
    train rows only and are returned.  Metrics: circular error median/p90 on the
    test rows before ``first_cycle_step`` (``pre``), from it on (``post``) and
    on all rows (``all``); train-fit metrics are reported for information."""
    if estimator not in ESTIMATORS:
        raise R.F2RContractError(f"unknown estimator {estimator!r}")
    cols, names, pre_cycle = assert_columns(columns)
    if fold.held_out in fold.train_starts:
        raise R.F2RContractError(f"fold leakage: held-out start {fold.held_out!r} is in train_starts {fold.train_starts}")
    if len(set(fold.train_starts)) != len(fold.train_starts) or len(set(fold.anchors)) != len(fold.anchors):
        raise R.F2RContractError("duplicate starts in the fold specification")
    for s in (fold.held_out, *fold.train_starts):
        if s not in jul:
            raise R.F2RContractError(f"missing JUL start {s!r}")
    for a in fold.anchors:
        if a not in anchors:
            raise R.F2RContractError(f"missing anchor start {a!r}")
    test = jul[fold.held_out]
    if test.start != fold.held_out:
        raise R.F2RContractError("held-out StartData.start != fold.held_out")
    train_parts, reset_rule = apply_reset_row_rule(fold, jul, anchors, cols=cols)
    Xtr = np.concatenate([x for _, x, _ in train_parts], axis=0)
    phi_tr = np.concatenate([p for _, _, p in train_parts], axis=0)
    n_train_jul = int(sum(jul[s].n for s in fold.train_starts))
    # --- strict structural leakage check (bitwise rows), re-asserted on the final train matrix ---
    full_collisions = len(_row_keys(test.X) & _row_keys(Xtr))
    sel_collisions = len(_row_keys(test.X[:, cols]) & _row_keys(Xtr[:, cols]))
    if full_collisions or sel_collisions:
        raise R.F2RContractError(f"leakage: {full_collisions} held-out rows ({sel_collisions} on the selected columns) are bitwise present in the train matrix")
    if Xtr.shape[0] != reset_rule["train_rows_after"] or test.n != reset_rule["test_rows"]:
        raise R.F2RContractError("reset-row rule provenance inconsistent with the train/test matrices")
    # --- standardisation on train only ---
    mean, std, constant = standardise_stats(Xtr[:, cols])
    Ztr = (Xtr[:, cols] - mean) / std
    Zte = (test.X[:, cols] - mean) / std
    Ytr = sincos_from_phase(phi_tr)
    if estimator == "ridge":
        lam = float(protocol_p0["estimators"]["ridge"]["lambda"])
        predict = fit_ridge(Ztr, Ytr, lam)
        hparams: dict[str, Any] = {"lambda": lam, "form": RIDGE_FORM}
    else:
        mh = dict(protocol_p0["estimators"]["mlp"])
        predict = fit_mlp(Ztr, Ytr, mh, int(mh["seed"]))
        hparams = dict(predict.hparams)  # type: ignore[attr-defined]
    phi_hat = phase_from_sincos(predict(Zte))
    err = circular_error(phi_hat, test.phi)
    k = test.first_cycle_step
    err_tr = circular_error(phase_from_sincos(predict(Ztr)), phi_tr)
    return {
        "held_out": fold.held_out,
        "train_starts": list(fold.train_starts),
        "anchors": list(fold.anchors),
        "estimator": estimator,
        "hparams": hparams,
        "variant": "pre_cycle" if pre_cycle else "post_cycle",
        "columns": cols,
        "column_names": names,
        "n_train": int(Xtr.shape[0]),
        "n_train_jul": n_train_jul,
        "n_train_anchor": int(Xtr.shape[0] - n_train_jul),
        "n_test": test.n,
        "first_cycle_step": int(k),
        "test_source": test.source,
        "train_sources": sorted({name for name, _, _ in train_parts}),
        "reset_row_rule": reset_rule,
        "leakage_check": {"held_out_in_train_starts": False, "full_row_collisions": int(full_collisions), "selected_row_collisions": int(sel_collisions), "rule": "bitwise row bytes, test vs train, asserted after the reset-row exclusion (amendment B)"},
        "standardisation": {"fitted_on": "train_only", "mean": mean.astype(float).tolist(), "std": std.astype(float).tolist(), "constant_columns": [cols[i] for i in constant]},
        "metrics": {"pre": _metrics(err[:k]), "post": _metrics(err[k:]), "all": _metrics(err)},
        "train_metrics": {"all": _metrics(err_tr)},
    }


def fold_passes(metrics_all: Mapping[str, Any], thresholds: Mapping[str, Any]) -> bool:
    med, p90 = metrics_all.get("median"), metrics_all.get("p90")
    if med is None or p90 is None:
        return False
    return bool(float(med) <= float(thresholds["median_pre_cycle"]) and float(p90) <= float(thresholds["p90_pre_cycle"]))


def evaluate_p0(jul: Mapping[str, StartData], anchors: Mapping[str, StartData], protocol: Mapping[str, Any], real: bool = False) -> dict[str, Any]:
    """Full P0 evaluation: 3 LOSO folds x 2 variants (``pre_cycle`` / ``post_cycle``
    columns) x 2 estimators (ridge / mlp).  The frozen PASS rule
    (``protocol["p0"]["thresholds_frozen"]``) is applied to the MLP
    ``pre_cycle`` variant on all test rows of each held-out start: PASS iff
    >= 2 folds have median <= 0.05 and p90 <= 0.20.  The ``plus020`` fold is
    always reported explicitly.  ``executed_on_real_data`` records ``real``
    (S0 runs this only on synthetic data).  No time/index input can enter:
    every column list is asserted by name."""
    p0 = protocol["p0"]
    thresholds = dict(p0["thresholds_frozen"])
    for key in ("median_pre_cycle", "p90_pre_cycle"):
        if key not in thresholds:
            raise R.F2RContractError(f"protocol p0.thresholds_frozen missing {key}")
    starts = tuple(R.STARTS)
    for name, table in (("jul", jul), ("anchors", anchors)):
        missing = [s for s in starts if s not in table]
        if missing:
            raise R.F2RContractError(f"{name} missing starts {missing}")
        for s in starts:
            if not isinstance(table[s], StartData) or table[s].start != s:
                raise R.F2RContractError(f"{name}[{s!r}] is not a StartData of that start")
    names = R.FEATURE_NAMES_35
    columns = {"pre_cycle": select_columns(names, pre_cycle=True), "post_cycle": select_columns(names, pre_cycle=False)}
    folds = build_folds(starts)
    results: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for variant in VARIANTS:
        results[variant] = {}
        for estimator in ESTIMATORS:
            results[variant][estimator] = [run_fold(f, jul, anchors, columns=columns[variant], estimator=estimator, protocol_p0=p0) for f in folds]
    rule_results = results["pre_cycle"]["mlp"]
    reset_rule_per_fold = {r["held_out"]: r["reset_row_rule"] for r in rule_results}
    for variant in VARIANTS:
        for estimator in ESTIMATORS:
            for r in results[variant][estimator]:
                same = {k: v for k, v in r["reset_row_rule"].items() if k != "collisions_before_rule" and k != "collisions_after_rule"}
                ref = {k: v for k, v in reset_rule_per_fold[r["held_out"]].items() if k != "collisions_before_rule" and k != "collisions_after_rule"}
                if same != ref or r["reset_row_rule"]["collisions_after_rule"] != {"full_rows": 0, "selected_columns": 0}:
                    raise R.F2RContractError("reset-row rule provenance differs across variants/estimators for the same fold")
    per_fold_pass = {r["held_out"]: fold_passes(r["metrics"][RULE_ROWS], thresholds) for r in rule_results}
    n_pass = int(sum(per_fold_pass.values()))
    summary: dict[str, Any] = {}
    for variant in VARIANTS:
        summary[variant] = {}
        for estimator in ESTIMATORS:
            summary[variant][estimator] = {r["held_out"]: {sub: {"median": r["metrics"][sub]["median"], "p90": r["metrics"][sub]["p90"], "n": r["metrics"][sub]["n"]} for sub in ("pre", "post", "all")} for r in results[variant][estimator]}
    plus = "plus020"
    fold_plus020 = {variant: {estimator: summary[variant][estimator].get(plus) for estimator in ESTIMATORS} for variant in VARIANTS}
    informational = {r["held_out"]: fold_passes(r["metrics"]["pre"], thresholds) for r in rule_results}
    return {
        "protocol_id": protocol.get("protocol_id"),
        "tool": "f2r_observability",
        "executed_on_real_data": bool(real),
        "data": {"jul": {s: jul[s].describe() for s in starts}, "anchors": {s: anchors[s].describe() for s in starts}},
        "folds": [f.to_dict() for f in folds],
        "columns": columns,
        "column_names": {v: [names[i] for i in c] for v, c in columns.items()},
        "forbidden_inputs": list(R.FORBIDDEN_P0_INPUT_NAMES),
        "excluded_pre_cycle": list(R.P0_PRE_CYCLE_EXCLUDED),
        "amendment_B_reset_row": dict(p0.get("amendment_B_reset_row", {})),
        "reset_row_rule_per_fold": reset_rule_per_fold,
        "thresholds": thresholds,
        "rule": {"estimator": "mlp", "variant": "pre_cycle", "rows": RULE_ROWS, "min_folds_pass": 2, "of_folds": len(folds), "median_max": float(thresholds["median_pre_cycle"]), "p90_max": float(thresholds["p90_pre_cycle"])},
        "per_fold_pass": per_fold_pass,
        "n_pass": n_pass,
        "pass": bool(n_pass >= 2),
        "fold_plus020": fold_plus020,
        "fold_plus020_reported": plus in per_fold_pass,
        "per_fold_pass_pre_rows_informational": informational,
        "summary": summary,
        "results": results,
        "baselines_1nn_jul_states": p0.get("baselines_1nn_jul_states"),
        "tooling_choices": {"mlp_batch_size_default": MLP_BATCH_SIZE_DEFAULT, "ridge_form": RIDGE_FORM, "zero_std_guard": "constant train columns standardised with std 1", "rule_rows": RULE_ROWS},
    }


def decision_table(result: Mapping[str, Any]) -> str:
    """Markdown summary (variant x estimator x fold: median / p90 on all test rows)."""
    lines = ["| variant | estimator | fold | n_test | median | p90 | pass(rule) |", "|---|---|---|---|---|---|---|"]
    for variant in VARIANTS:
        for estimator in ESTIMATORS:
            for held_out, subs in result["summary"][variant][estimator].items():
                m = subs["all"]
                flag = result["per_fold_pass"].get(held_out) if (variant == "pre_cycle" and estimator == "mlp") else None
                lines.append(f"| {variant} | {estimator} | {held_out} | {m['n']} | {m['median']:.4f} | {m['p90']:.4f} | {'' if flag is None else ('PASS' if flag else 'FAIL')} |")
    lines.append("")
    lines.append(f"P0 {'PASS' if result['pass'] else 'FAIL'} ({result['n_pass']}/{result['rule']['of_folds']} folds; MLP pre_cycle; median <= {result['rule']['median_max']}, p90 <= {result['rule']['p90_max']}); executed_on_real_data = {result['executed_on_real_data']}")
    return "\n".join(lines)


# --- CLI (S0: plan only; S1: real P0 on existing artefacts, read-only) -------------------


def prescribed_heel_strikes_from_anchor(start: str = "nominal") -> list[float]:
    """Prescribed sound-side heel strikes recorded by the F1 adapter of the pinned
    anchor job (``f1_adapter_summary.json`` -> ``reconstructor.clock_heel_strike_times_s``);
    the file digest is re-verified against ``R.ANCHORS`` before use (read-only)."""
    anchor = R.ANCHORS[start]
    path = Path(anchor["job_dir"]) / "f1_adapter_summary.json"
    if path.is_symlink() or not path.is_file() or C.sha256_file(path) != anchor["adapter_summary_sha256"]:
        raise R.F2RContractError(f"anchor adapter summary missing or not matching its pin: {path}")
    hs = json.loads(path.read_text(encoding="utf-8"))["reconstructor"]["clock_heel_strike_times_s"]
    return [float(x) for x in _heel_strikes(hs)]


def plan(protocol: Mapping[str, Any]) -> dict[str, Any]:
    names = R.FEATURE_NAMES_35
    cols = {"pre_cycle": select_columns(names, pre_cycle=True), "post_cycle": select_columns(names, pre_cycle=False)}
    return {
        "tool": "f2r_observability", "protocol_id": protocol.get("protocol_id"), "executed_in_S0": bool(protocol["p0"].get("executed_in_S0", False)),
        "folds": [f.to_dict() for f in build_folds()], "columns": cols, "column_names": {v: [names[i] for i in c] for v, c in cols.items()},
        "forbidden_inputs": list(R.FORBIDDEN_P0_INPUT_NAMES), "estimators": protocol["p0"]["estimators"], "thresholds_frozen": protocol["p0"]["thresholds_frozen"],
        "jul_jobs": {s: C.rel(p) for s, p in R.P0_JUL_JOBS.items()}, "anchor_jobs": {s: C.rel(a["job_dir"]) for s, a in R.ANCHORS.items()},
        "label": "phi(t_pre) from GaitPhaseClock formula on the prescribed heel strikes of the pinned anchor adapter summary (offset 0)",
    }


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="F2R P0 observability (S0: plan only).")
    parser.add_argument("--real", action="store_true", help="run P0 on the existing JUL_H0/anchor artefacts (read-only); S1 only")
    parser.add_argument("--authorized-stage", default=None, help="must be 'S1' to run --real (architect's go after the S0 audit)")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(list(argv) if argv is not None else None)
    protocol = R.load_protocol()
    if not args.real:
        print(json.dumps(plan(protocol), indent=2))
        return 0
    if args.authorized_stage != "S1":
        raise SystemExit("P0 on real data is not authorised in S0: pass --authorized-stage S1 only after the architect's go")
    if not args.out_dir:
        raise SystemExit("--out-dir is required for --real")
    print(json.dumps(run_p0_real(Path(args.out_dir), protocol=protocol), indent=2))
    return 0


def p0_input_preflight(*, verify_pins: Callable[[], Mapping[str, Any]] = R.verify_anchor_pins, verify_jul: Callable[[], Mapping[str, Any]] = R.verify_p0_jul_pins) -> dict[str, Any]:
    """Complete, fail-closed verification of every P0 input BEFORE any trace is parsed:
    (a) the three A_iso6clk anchors (content pins), (b) the three JUL_H0 deterministic F0 jobs
    (content pins of summary/trace/reset/receipt, receipt internal digests, summary/reset
    consistency, pinned F0 analysis chain).  Raises ``F2RContractError`` on any problem; returns
    an auditable record (verdicts + verified digests) for the P0 provenance."""
    anchors = verify_pins()
    if not isinstance(anchors, Mapping) or anchors.get("all_match") is not True:
        raise R.F2RContractError(f"anchor pins do not match the disk (all_match={anchors.get('all_match') if isinstance(anchors, Mapping) else anchors!r}): P0 aborted before reading any data")
    jul = verify_jul()
    if not isinstance(jul, Mapping) or jul.get("all_match") is not True:
        problems = {k: v.get("problems") for k, v in jul.items() if isinstance(v, dict) and v.get("problems")} if isinstance(jul, Mapping) else jul
        raise R.F2RContractError(f"JUL_H0 P0 job pins do not verify: {json.dumps(problems, default=str)[:2000]}; P0 aborted before reading any data")
    anchor_digests = {s: {k: (anchors[s][k].get("disk") if isinstance(anchors[s].get(k), Mapping) else anchors[s].get(k)) for k in R.ANCHOR_FILES} for s in R.STARTS if isinstance(anchors.get(s), Mapping)}
    jul_digests = {s: {key: jul[s]["files"][key].get("sha256") for key in R.P0_JUL_FILES} for s in R.STARTS}
    return {
        "verified": True,
        "order": "anchors -> JUL_H0 jobs (files, receipt, summary, reset, F0 analysis chain) -> load",
        "anchors": {"all_match": True, "digests": anchor_digests, "verdict": {k: v for k, v in anchors.items() if k != "digest_semantics"}},
        "jul_h0_p0_jobs": {"all_match": True, "files": dict(R.P0_JUL_FILES), "digests": jul_digests, "job_ids": {s: jul[s].get("job_id") for s in R.STARTS}, "job_dirs": {s: jul[s].get("job_dir") for s in R.STARTS}, "receipt_schema": jul.get("receipt_schema"), "f0_analysis": jul.get("f0_analysis"), "verdict": {s: jul[s] for s in R.STARTS}},
    }


def run_p0_real(out_dir: Path, *, protocol: Mapping[str, Any] | None = None, verify_pins: Callable[[], Mapping[str, Any]] = R.verify_anchor_pins, verify_jul: Callable[[], Mapping[str, Any]] = R.verify_p0_jul_pins, load: Callable[..., StartData] = load_start_data) -> dict[str, Any]:
    """S1 only (never called in S0 tests on real data): ``p0_input_preflight`` must pass — anchors
    AND the three JUL_H0 F0 jobs (content pins, receipt digests, F0 analysis chain) — **before any
    trace is read**; then the JUL_H0 det jobs and the anchors are loaded read-only, P0 is evaluated
    and ``p0_result_<stamp>[_NN].json/.md`` are written with exclusive, atomic, unique names.  The
    result carries the full input verification record (``input_verification``)."""
    protocol = R.load_protocol() if protocol is None else protocol
    verification = p0_input_preflight(verify_pins=verify_pins, verify_jul=verify_jul)
    hs = prescribed_heel_strikes_from_anchor()
    jul = {s: load(p, hs, source="jul_det_v3", start=s) for s, p in R.P0_JUL_JOBS.items()}
    anchors = {s: load(a["job_dir"], hs, source="anchor_aiso6clk", start=s) for s, a in R.ANCHORS.items()}
    result = evaluate_p0(jul, anchors, protocol, real=True)
    result["heel_strikes_prescribed"] = hs
    result["input_verification"] = verification
    result["inputs_verified_before_load"] = True
    import time as _time

    reserved = R.reserve_unique_set(Path(out_dir), f"p0_result_{_time.strftime('%Y%m%d_%H%M%S')}", (".json", ".md"))
    R._atomic_fill(reserved[".json"], json.dumps(result, indent=2, default=str).encode("utf-8"))
    R._atomic_fill(reserved[".md"], (decision_table(result) + "\n").encode("utf-8"))
    return {"result": C.rel(reserved[".json"]), "sha256": C.sha256_file(reserved[".json"]), "markdown": C.rel(reserved[".md"]), "pass": result["pass"], "n_pass": result["n_pass"], "inputs_verified_before_load": True}


if __name__ == "__main__":
    raise SystemExit(main())
