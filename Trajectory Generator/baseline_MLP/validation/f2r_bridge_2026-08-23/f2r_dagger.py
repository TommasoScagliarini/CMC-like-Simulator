"""DAgger-style dataset aggregation and round plan for the F2R bridge (rev 3).

S0 scope: tooling only — synthetic tests and dry-run.  Nothing here runs a
rollout, steps the simulator or fits a student; ``rows_from_job`` only reads
existing ``rollout_eval`` job directories and asks an injected ``labeller``
for the labels.

Dataset contract (``f2r_protocol.json`` -> ``dataset``):

* ``D1`` = anchors only (the three content-addressed A_iso6clk traces, 1500
  rows, purpose ``anchor``, seed 123);
* ``D_{r+1}`` = ``D_r`` union the labelled rows of the student's seed-123
  deterministic (purpose ``det``) and seed-123/124 stochastic (purpose
  ``stoch``) rollouts of round ``r``; **seed 125 never** (held-out promotion
  gate V: no label, no dataset, no fit, no aggregation); seeds 126-128 sealed;
* rows carry ``obs35`` (float32-cast env vector), ``t_pre`` (exact env floats
  ``[reset_time] + time[:-1]``), ``actions`` (labels only — no feature
  leakage), ``job_id``/``step`` (dedup key, first occurrence kept), ``start``,
  ``seed``, ``purpose``;
* the anchor rows are immutable across rounds: their digest is pinned in
  ``anchor_digest`` and re-verified at every aggregation and load.

``round_plan`` describes the rev-3 sequence of one round (initial dataset,
refit, rollouts, gates A -> B -> C -> V, promotion or aggregation) with the
budget of the variant (T1: 1 round, det only, always STOP; T2: <= 4; T3: <= 2
on the preregistered trigger only).  ``save_dataset``/``load_dataset`` are
no-clobber and digest-verified.
"""

from __future__ import annotations

import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402  (puts F0/F1 on sys.path)
import f0_common as C  # noqa: E402  (F0 library, immutable)
import f1_dataset as DS  # noqa: E402  (F1 library, immutable; t_pre via OA.t_pre_from_trace inside trajectory_from_job)

PURPOSES = ("anchor", "det", "stoch")
ROLLOUT_PURPOSES = ("det", "stoch")
ARRAY_ORDER = ("obs35", "t_pre", "actions", "job_id", "step", "start", "seed", "purpose")
STRING_ARRAYS = ("job_id", "start", "purpose")
DATASET_PREFIX = "f2r_dataset_"
RECEIPT_SCHEMA = 1
VALIDATION_GATE_PURPOSE = "validation_gate"


# --- digests --------------------------------------------------------------------------------


def sha256_string_array(arr: np.ndarray) -> str:
    """Width-independent digest of a 1-D string array (count + UTF-8 items),
    so that the digest of a row subset does not depend on the unicode dtype
    width inherited from a concatenation."""
    items = [str(x) for x in np.asarray(arr).reshape(-1).tolist()]
    h = hashlib.sha256()
    h.update(f"str[{len(items)}]".encode("ascii"))
    for s in items:
        b = s.encode("utf-8")
        h.update(len(b).to_bytes(8, "little"))
        h.update(b)
    return h.hexdigest()


def array_digest(name: str, arr: np.ndarray) -> str:
    return sha256_string_array(arr) if name in STRING_ARRAYS else DS.sha256_array(arr)


# --- rows ----------------------------------------------------------------------------------


@dataclass
class DatasetRows:
    """Column arrays of N labelled rows (see module docstring) + ``variant``
    (T1/T2/T3 or None when unassigned) and ``anchor_digest`` (digest of the
    anchor rows pinned at D1; None before anchors are attached)."""

    obs35: np.ndarray
    t_pre: np.ndarray
    actions: np.ndarray
    job_id: np.ndarray
    step: np.ndarray
    start: np.ndarray
    seed: np.ndarray
    purpose: np.ndarray
    variant: str | None = None
    anchor_digest: str | None = None

    def __post_init__(self) -> None:
        self.obs35 = np.ascontiguousarray(np.asarray(self.obs35, dtype=np.float32))
        self.t_pre = np.ascontiguousarray(np.asarray(self.t_pre, dtype=np.float64).reshape(-1))
        self.actions = np.ascontiguousarray(np.asarray(self.actions, dtype=np.float32))
        self.job_id = np.asarray([str(x) for x in np.asarray(self.job_id).reshape(-1).tolist()], dtype=str)
        self.step = np.ascontiguousarray(np.asarray(self.step, dtype=np.int64).reshape(-1))
        self.start = np.asarray([str(x) for x in np.asarray(self.start).reshape(-1).tolist()], dtype=str)
        self.seed = np.ascontiguousarray(np.asarray(self.seed, dtype=np.int64).reshape(-1))
        self.purpose = np.asarray([str(x) for x in np.asarray(self.purpose).reshape(-1).tolist()], dtype=str)
        n = int(self.obs35.shape[0]) if self.obs35.ndim == 2 else -1
        if self.obs35.ndim != 2 or self.obs35.shape[1] != R.ENV_ACTOR_WIDTH:
            raise R.F2RContractError(f"obs35 must be (N, {R.ENV_ACTOR_WIDTH}), got {self.obs35.shape}")
        if self.actions.ndim != 2 or self.actions.shape != (n, R.ACTION_DIM):
            raise R.F2RContractError(f"actions must be (N, {R.ACTION_DIM}), got {self.actions.shape}")
        for name in ("t_pre", "job_id", "step", "start", "seed", "purpose"):
            if getattr(self, name).shape != (n,):
                raise R.F2RContractError(f"{name} must have {n} rows, got {getattr(self, name).shape}")
        if n and (not np.all(np.isfinite(self.obs35)) or not np.all(np.isfinite(self.actions)) or not np.all(np.isfinite(self.t_pre))):
            raise R.F2RContractError("non-finite values in the dataset rows")
        self.variant = None if self.variant is None else str(self.variant)
        self.anchor_digest = None if self.anchor_digest is None else str(self.anchor_digest)

    # -- basic API --
    @property
    def size(self) -> int:
        return int(self.obs35.shape[0])

    @classmethod
    def empty(cls, *, variant: str | None = None, anchor_digest: str | None = None) -> "DatasetRows":
        return cls(obs35=np.zeros((0, R.ENV_ACTOR_WIDTH), np.float32), t_pre=np.zeros(0), actions=np.zeros((0, R.ACTION_DIM), np.float32), job_id=np.asarray([], dtype=str), step=np.zeros(0, np.int64), start=np.asarray([], dtype=str), seed=np.zeros(0, np.int64), purpose=np.asarray([], dtype=str), variant=variant, anchor_digest=anchor_digest)

    @classmethod
    def concat(cls, parts: Sequence["DatasetRows"], *, variant: str | None = None, anchor_digest: str | None = None) -> "DatasetRows":
        """Row-wise concatenation in order (no dedup; see ``aggregate``)."""
        parts = list(parts)
        if not parts:
            return cls.empty(variant=variant, anchor_digest=anchor_digest)
        arrays = {name: np.concatenate([getattr(p, name) for p in parts], axis=0) for name in ARRAY_ORDER}
        return cls(**arrays, variant=variant, anchor_digest=anchor_digest)

    def subset(self, mask: np.ndarray) -> "DatasetRows":
        idx = np.asarray(mask)
        return DatasetRows(**{name: getattr(self, name)[idx] for name in ARRAY_ORDER}, variant=self.variant, anchor_digest=self.anchor_digest)

    def anchor_rows(self) -> "DatasetRows":
        return self.subset(self.purpose == "anchor")

    def per_start_counts(self) -> dict[str, int]:
        starts = list(R.STARTS) + sorted(set(self.start.tolist()) - set(R.STARTS))
        return {s: int(np.sum(self.start == s)) for s in starts}

    def per_purpose_counts(self) -> dict[str, int]:
        return {p: int(np.sum(self.purpose == p)) for p in PURPOSES}

    def per_seed_counts(self) -> dict[str, int]:
        return {str(int(s)): int(np.sum(self.seed == s)) for s in sorted(set(self.seed.tolist()))}

    def array_digests(self) -> dict[str, str]:
        return {name: array_digest(name, getattr(self, name)) for name in ARRAY_ORDER}

    def digest(self) -> str:
        """SHA-256 over the per-array digests in ``ARRAY_ORDER`` (content only;
        independent of ``variant``/``anchor_digest`` and of unicode widths)."""
        h = hashlib.sha256()
        for name, d in self.array_digests().items():
            h.update(name.encode("ascii"))
            h.update(b"=")
            h.update(d.encode("ascii"))
            h.update(b"\n")
        return h.hexdigest()

    def validate(self) -> "DatasetRows":
        """Structural rules on every row (fail-closed): purposes in ``PURPOSES``;
        seeds are collection seeds (125 and 126-128 raise); ``det``/``anchor``
        rows carry seed 123; starts are the three exact starts; steps >= 1."""
        for p in sorted(set(self.purpose.tolist())):
            if p not in PURPOSES:
                raise R.F2RContractError(f"unknown row purpose {p!r} (allowed {PURPOSES})")
        for s in sorted(set(self.seed.tolist())):
            R.assert_collection_seed(int(s))
        bad = np.flatnonzero(np.isin(self.purpose, ("det", "anchor")) & (self.seed != R.DET_SEED))
        if bad.size:
            raise R.F2RContractError(f"{bad.size} det/anchor rows with seed != {R.DET_SEED}")
        for s in sorted(set(self.start.tolist())):
            if s not in R.STARTS:
                raise R.F2RContractError(f"unknown start {s!r}")
        if self.size and np.any(self.step < 1):
            raise R.F2RContractError("steps must be >= 1")
        return self

    def dedup_keys(self) -> list[tuple[str, int]]:
        return list(zip(self.job_id.tolist(), (int(x) for x in self.step.tolist())))

    def describe(self) -> dict[str, Any]:
        return {"rows": self.size, "variant": self.variant, "anchor_digest": self.anchor_digest, "per_start_counts": self.per_start_counts(), "per_purpose_counts": self.per_purpose_counts(), "per_seed_counts": self.per_seed_counts(), "jobs": sorted(set(self.job_id.tolist())), "digest": self.digest()}


# --- reading jobs ----------------------------------------------------------------------------


def _anchor_start_for(job_dir: Path) -> str | None:
    resolved = Path(job_dir).resolve()
    for start, spec in R.ANCHORS.items():
        if Path(spec["job_dir"]).resolve() == resolved:
            return str(start)
    return None


def _verify_anchor_content(job_dir: Path, start: str) -> dict[str, str]:
    """Content-addressed anchor: trace/reset/summary SHA-256 must equal the pins."""
    spec = R.ANCHORS[start]
    out: dict[str, str] = {}
    for key in ("trace_sha256", "reset_sha256", "summary_sha256"):
        p = Path(job_dir) / R.ANCHOR_FILES[key]
        if p.is_symlink() or not p.is_file():
            raise R.F2RContractError(f"anchor file missing (or symlink): {p}")
        sha = C.sha256_file(p)
        if sha != spec[key]:
            raise R.F2RContractError(f"anchor {start} {R.ANCHOR_FILES[key]} digest {sha} != pinned {spec[key]}")
        out[key] = sha
    return out


def rows_from_job(job_dir: Path, *, purpose: str, labeller: Any, start: str, seed: int, variant: str | None = None, job_id: str | None = None) -> DatasetRows:
    """Labelled rows of one ``rollout_eval`` job directory (read-only).

    Reads ``obs35`` rows and ``t_pre`` (``OA.t_pre_from_trace`` on the reset
    time + trace times, through ``DS.trajectory_from_job``) and the labels from
    ``labeller.label(obs35_float32, t_pre)["actions"]`` (shape (N, 2), finite).

    Structural asserts (``F2RContractError``): ``R.assert_collection_seed(seed)``
    (125 and 126-128 raise); ``purpose`` in ``PURPOSES``; ``det`` requires
    ``seed == R.DET_SEED`` and a deterministic trace; ``stoch`` requires a
    stochastic trace; ``anchor`` requires ``job_dir`` to be one of the
    ``R.ANCHORS`` job dirs (resolved paths) with content matching the pins, and
    an anchor dir can only enter as ``anchor``; the recorded ``action_seed`` and
    the exact ``episode_start_offset_s`` must match the declared seed/start."""
    job_dir = Path(job_dir)
    if purpose not in PURPOSES:
        raise R.F2RContractError(f"unknown purpose {purpose!r}; rollout rows are {ROLLOUT_PURPOSES}, anchors are 'anchor' (seed {R.VALIDATION_SEED} jobs never enter a dataset)")
    seed = R.assert_collection_seed(seed)
    if purpose in ("det", "anchor") and seed != R.DET_SEED:
        raise R.F2RContractError(f"purpose {purpose!r} requires the deterministic seed {R.DET_SEED}, got {seed}")
    start = str(start)
    if start not in R.EXACT_STARTS:
        raise R.F2RContractError(f"unknown start {start!r}")
    anchor_start = _anchor_start_for(job_dir)
    if purpose == "anchor":
        if anchor_start is None:
            raise R.F2RContractError(f"purpose 'anchor' requires one of the pinned anchor job dirs, got {C.rel(job_dir)}")
        if anchor_start != start:
            raise R.F2RContractError(f"anchor dir is the {anchor_start!r} anchor, declared start {start!r}")
        _verify_anchor_content(job_dir, start)
    elif anchor_start is not None:
        raise R.F2RContractError(f"anchor job dir {C.rel(job_dir)} can only enter a dataset with purpose 'anchor'")
    try:
        traj = DS.trajectory_from_job(job_dir, expected_width=R.ENV_ACTOR_WIDTH)
    except DS.DatasetError as exc:
        raise R.F2RContractError(f"malformed job {C.rel(job_dir)}: {exc}") from exc
    if int(traj["seed"]) != seed:
        raise R.F2RContractError(f"declared seed {seed} != recorded action_seed {traj['seed']} in {C.rel(job_dir)}")
    if float(traj["episode_start_offset_s"]) != float(R.EXACT_STARTS[start]):
        raise R.F2RContractError(f"declared start {start!r} ({R.EXACT_STARTS[start]!r}) != recorded episode_start_offset_s {traj['episode_start_offset_s']!r}")
    if purpose == "stoch" and not traj["stochastic"]:
        raise R.F2RContractError("purpose 'stoch' requires a stochastic trace (policy_action_mean recorded)")
    if purpose in ("det", "anchor") and (traj["stochastic"] or traj["action_selection"] != "deterministic"):
        raise R.F2RContractError(f"purpose {purpose!r} requires a deterministic trace")
    obs35 = traj["obs35"].astype(np.float32)
    t_pre = np.asarray(traj["t_pre"], dtype=np.float64)
    n = int(obs35.shape[0])
    out = labeller.label(obs35, t_pre)
    if not isinstance(out, Mapping) or "actions" not in out:
        raise R.F2RContractError("labeller.label must return a mapping with 'actions'")
    actions = np.asarray(out["actions"], dtype=np.float64)
    if actions.shape != (n, R.ACTION_DIM) or not np.all(np.isfinite(actions)):
        raise R.F2RContractError(f"labels must be finite (N, {R.ACTION_DIM}), got {actions.shape}")
    jid = str(job_id) if job_id is not None else job_dir.name
    if not jid:
        raise R.F2RContractError("empty job_id")
    return DatasetRows(obs35=obs35, t_pre=t_pre, actions=actions.astype(np.float32), job_id=np.asarray([jid] * n, dtype=str), step=np.arange(1, n + 1, dtype=np.int64), start=np.asarray([start] * n, dtype=str), seed=np.full(n, seed, dtype=np.int64), purpose=np.asarray([purpose] * n, dtype=str), variant=variant, anchor_digest=None)


def anchor_rows(labeller: Any, *, variant: str | None = None) -> DatasetRows:
    """D1: the three pinned anchors (purpose ``anchor``, seed 123, start from the
    ``R.ANCHORS`` key) labelled by ``labeller``; ``anchor_digest`` set."""
    parts = [rows_from_job(Path(spec["job_dir"]), purpose="anchor", labeller=labeller, start=start, seed=R.DET_SEED, variant=variant) for start, spec in R.ANCHORS.items()]
    rows = DatasetRows.concat(parts, variant=variant)
    rows.anchor_digest = rows.digest()
    return rows.validate()


# --- aggregation -----------------------------------------------------------------------------


def aggregate(current: DatasetRows, new_list: Sequence[DatasetRows], *, anchor_digest: str) -> tuple[DatasetRows, dict[str, Any]]:
    """``D_{r+1} = D_r union new rows`` with the rev-3 rules (``F2RContractError``):

    * every row (current and new) passes ``validate`` — seed 125 or sealed
      seeds anywhere raise, purposes valid, det rows seed 123;
    * anchors unchanged: ``current`` must contain anchor rows whose digest equals
      ``anchor_digest`` (and ``current.anchor_digest`` when set); new rows may not
      carry purpose ``anchor``; the anchor rows of the result re-hash to the same;
    * variants consistent (a new part with a different non-None variant raises);
    * dedup by ``(job_id, step)`` keeping the first occurrence (count reported).

    Returns ``(rows, report)`` with per-start counts, ``added`` and ``deduplicated``."""
    if not isinstance(current, DatasetRows):
        raise R.F2RContractError("current must be a DatasetRows")
    current.validate()
    anchor_digest = str(anchor_digest)
    if current.anchor_digest is not None and current.anchor_digest != anchor_digest:
        raise R.F2RContractError("anchor_digest argument != current.anchor_digest")
    cur_anchor = current.anchor_rows()
    if cur_anchor.size == 0:
        raise R.F2RContractError("current dataset has no anchor rows (D1 = anchors only)")
    if cur_anchor.digest() != anchor_digest:
        raise R.F2RContractError("anchor rows of the current dataset do not re-hash to anchor_digest (anchors are immutable)")
    parts = list(new_list)
    for i, part in enumerate(parts):
        if not isinstance(part, DatasetRows):
            raise R.F2RContractError(f"new_list[{i}] is not a DatasetRows")
        part.validate()
        if np.any(part.purpose == "anchor"):
            raise R.F2RContractError(f"new_list[{i}] carries anchor rows: anchors are fixed at D1 and never re-aggregated")
        if part.variant is not None and current.variant is not None and part.variant != current.variant:
            raise R.F2RContractError(f"variant mismatch: dataset {current.variant!r} vs new rows {part.variant!r}")
    merged = DatasetRows.concat([current, *parts], variant=current.variant, anchor_digest=anchor_digest)
    seen: set[tuple[str, int]] = set()
    keep = np.zeros(merged.size, dtype=bool)
    for i, key in enumerate(merged.dedup_keys()):
        if key not in seen:
            seen.add(key)
            keep[i] = True
    result = merged.subset(keep)
    result.variant, result.anchor_digest = current.variant, anchor_digest
    if result.anchor_rows().digest() != anchor_digest:
        raise R.F2RContractError("anchor rows changed through aggregation")
    result.validate()
    before = current.per_start_counts()
    after = result.per_start_counts()
    raw_new = int(sum(p.size for p in parts))
    report = {
        "rows_before": current.size,
        "rows_new_raw": raw_new,
        "deduplicated": int(merged.size - result.size),
        "rows_after": result.size,
        "added": int(result.size - current.size),
        "per_start_counts": after,
        "per_start_added": {s: int(after.get(s, 0) - before.get(s, 0)) for s in after},
        "per_purpose_counts": result.per_purpose_counts(),
        "per_seed_counts": result.per_seed_counts(),
        "anchor_rows": int(cur_anchor.size),
        "anchor_digest": anchor_digest,
        "anchors_unchanged": True,
        "new_parts": [{"rows": p.size, "purposes": sorted(set(p.purpose.tolist())), "seeds": sorted(set(int(s) for s in p.seed.tolist())), "jobs": sorted(set(p.job_id.tolist()))} for p in parts],
        "dedup_key": "(job_id, step), first occurrence kept",
        "seed_rules": {"collection": list(R.COLLECTION_SEEDS), "validation_never": R.VALIDATION_SEED, "sealed_never": list(R.SEALED_SEEDS)},
        "variant": result.variant,
    }
    return result, report


# --- round plan ---------------------------------------------------------------------------------


def round_plan(variant: str, r: int, *, protocol: Mapping[str, Any]) -> dict[str, Any]:
    """Rev-3 sequence of round ``r`` of ``variant`` (T1/T2/T3), from the protocol:

    * ``initial_dataset``: ``anchors_only`` (D1) for ``r == 1``, else ``D_r``;
    * ``refit`` with the frozen budget (init theta_{r-1}, theta_0 = JUL_H0);
    * ``rollouts``: det seed 123 (3 starts), stoch seeds 123/124 (3 starts),
      validation gate seed 125 (3 starts, ``enters_dataset`` False);
      T1: det only;
    * ``gates`` in order A, B, C, V (T1: A only, ``stop_always``);
    * ``on_all_pass`` = promote to R without aggregation, ``on_fail`` = aggregate
      123/124 only, if the budget allows (``r < max_rounds``).

    ``F2RContractError`` for an unknown variant or ``r`` outside ``1..max_rounds``
    (T1: 1, T2: 4, T3: 2)."""
    variants = protocol.get("variants", {})
    if variant not in variants:
        raise R.F2RContractError(f"unknown variant {variant!r} (protocol variants {sorted(variants)})")
    spec = variants[variant]
    max_rounds = int(spec["max_rounds"] if "max_rounds" in spec else spec["rounds"])
    r = int(r)
    if r < 1 or r > max_rounds:
        raise R.F2RContractError(f"round {r} outside the {variant} budget 1..{max_rounds}")
    det = {"purpose": "det", "seed": R.DET_SEED, "starts": len(R.STARTS), "enters_dataset": True, "role": "training side: gate measurement A/B/C and aggregation"}
    stoch = {"purpose": "stoch", "seeds": list(R.COLLECTION_SEEDS), "starts": len(R.STARTS), "enters_dataset": True, "role": "collection"}
    gate_v = {"purpose": VALIDATION_GATE_PURPOSE, "seed": R.VALIDATION_SEED, "starts": len(R.STARTS), "enters_dataset": False, "generates_labels": False, "role": "held-out promotion gate V (go/no-go only)"}
    t1 = variant in ("T1", "T1R")  # commissioning variants: det only, gate A, STOP always, no aggregation
    det_gate_only = {"purpose": "det", "seed": R.DET_SEED, "starts": len(R.STARTS), "enters_dataset": False, "generates_labels": False, "role": "commissioning gate measurement A/B/C only; no aggregation"}
    plan: dict[str, Any] = {
        "protocol_id": protocol.get("protocol_id"),
        "variant": variant,
        "role": spec.get("role"),
        "label": spec.get("label"),
        "round": r,
        "max_rounds": max_rounds,
        "initial_dataset": "anchors_only" if r == 1 else f"D_{r}",
        "refit": {"budget": dict(protocol.get("refit_budget", R.REFIT_BUDGET)), "init": "JUL_H0 (theta_0)" if r == 1 else f"theta_{r - 1}", "anchor_parameters": "theta_0 = JUL_H0" if r == 1 else f"theta_{r - 1}", "output": f"theta_{r}", "selection": "closed_loop_only"},
        "rollouts": ([det_gate_only] if variant == "T1R" else [det]) if t1 else [det, stoch, gate_v],
        "gates": ["A"] if t1 else ["A", "B", "C", "V"],
        "stop_always": bool(t1),
        "on_all_pass": "stop" if t1 else "promote_to_R_without_aggregation",
        "on_fail": "stop" if t1 else "aggregate_123_124_only_if_budget",
        "aggregation_allowed_after_fail": bool((not t1) and r < max_rounds),
        "next_round": (r + 1) if (not t1 and r < max_rounds) else None,
        "budget_remaining_after_round": int(max_rounds - r),
        "seed_125_enters_dataset": False,
        "consumes_T2_budget": bool(R.VARIANTS.get(variant, {}).get("consumes_T2_budget", variant == "T2")),
        "audit_after_round": True,
    }
    if t1:
        plan["stop_reason"] = spec.get("stop")
    if variant == "T1R":
        plan["refit"] = {**plan["refit"], "init": "JUL_H0 (theta_0) — never the T1 student", "loss": spec.get("loss"), "beta": spec.get("beta"), "criteria": spec.get("criteria"), "dataset": spec.get("dataset")}
        plan["unlock_t2_on_A_pass"] = True
        plan["t3_trigger"] = "never from T1R"
        plan["aggregation"] = "none: the 3 T1R rollouts are gate-only (enters_dataset false, generates_labels false)"
    if variant == "T3":
        plan["requires_trigger"] = True
        plan["trigger"] = spec.get("trigger")
        plan["tuning"] = spec.get("tuning", "none")
    return plan


# --- persistence --------------------------------------------------------------------------------


def save_dataset(out_dir: Path, stamp: str, rows: DatasetRows, extra_receipt: Mapping[str, Any] | None = None) -> dict[str, str]:
    """No-clobber ``<out_dir>/f2r_dataset_<stamp>.npz`` + ``.json`` receipt with
    per-array SHA-256, rows digest, counts, variant and anchor_digest."""
    rows.validate()
    out_dir = Path(out_dir)
    npz = out_dir / f"{DATASET_PREFIX}{stamp}.npz"
    rec = out_dir / f"{DATASET_PREFIX}{stamp}.json"
    if npz.exists() or rec.exists():
        raise FileExistsError(f"refusing to overwrite dataset {stamp}")
    R.reserve_exclusive(npz)  # exclusive reservation (O_EXCL); a same-stamp collision fails closed
    R.reserve_exclusive(rec)
    import io

    buf = io.BytesIO()
    np.savez(buf, **{name: getattr(rows, name) for name in ARRAY_ORDER})
    R._atomic_fill(npz, buf.getvalue())
    receipt = {
        "schema_version": RECEIPT_SCHEMA,
        "tool": "f2r_dagger",
        "protocol_id": R.PROTOCOL_ID,
        "stamp": str(stamp),
        "variant": rows.variant,
        "anchor_digest": rows.anchor_digest,
        "anchor_rows": int(rows.anchor_rows().size),
        "rows": rows.size,
        "rows_digest": rows.digest(),
        "arrays_sha256": rows.array_digests(),
        "array_shapes": {name: list(getattr(rows, name).shape) for name in ARRAY_ORDER},
        "array_dtypes": {name: str(getattr(rows, name).dtype) for name in ARRAY_ORDER},
        "per_start_counts": rows.per_start_counts(),
        "per_purpose_counts": rows.per_purpose_counts(),
        "per_seed_counts": rows.per_seed_counts(),
        "jobs": sorted(set(rows.job_id.tolist())),
        "seed_rules": {"collection": list(R.COLLECTION_SEEDS), "validation_never": R.VALIDATION_SEED, "sealed_never": list(R.SEALED_SEEDS)},
        "labels": "actions only (no feature leakage); obs35 float32-cast; t_pre exact env floats",
        **dict(extra_receipt or {}),
        "npz": C.rel(npz),
        "npz_sha256": C.sha256_file(npz),
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
    }
    R._atomic_fill(rec, json.dumps(receipt, indent=2, default=str).encode("utf-8"))
    return {"npz": str(npz), "receipt": str(rec), "npz_sha256": receipt["npz_sha256"], "receipt_sha256": C.sha256_file(rec), "rows_digest": receipt["rows_digest"]}


def load_dataset(npz_path: Path, receipt_path: Path) -> tuple[DatasetRows, dict[str, Any]]:
    """Load and verify (file SHA-256, per-array digests, rows digest, anchor
    digest, structural rules).  ``F2RContractError`` on any mismatch."""
    npz_path, receipt_path = Path(npz_path), Path(receipt_path)
    if npz_path.is_symlink() or receipt_path.is_symlink():
        raise R.F2RContractError("symlinked dataset files are refused")
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if receipt.get("tool") != "f2r_dagger" or receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise R.F2RContractError("receipt is not an f2r_dagger dataset receipt")
    if C.sha256_file(npz_path) != receipt.get("npz_sha256"):
        raise R.F2RContractError("dataset npz digest != receipt")
    with np.load(npz_path, allow_pickle=False) as z:
        missing = [name for name in ARRAY_ORDER if name not in z.files]
        if missing:
            raise R.F2RContractError(f"npz missing arrays {missing}")
        arrays = {name: z[name] for name in ARRAY_ORDER}
    rows = DatasetRows(**arrays, variant=receipt.get("variant"), anchor_digest=receipt.get("anchor_digest"))
    for name, sha in receipt["arrays_sha256"].items():
        if array_digest(name, getattr(rows, name)) != sha:
            raise R.F2RContractError(f"array {name} digest mismatch")
    if rows.digest() != receipt.get("rows_digest") or rows.size != int(receipt.get("rows", -1)):
        raise R.F2RContractError("rows digest/count mismatch")
    if rows.anchor_digest is not None and rows.anchor_rows().digest() != rows.anchor_digest:
        raise R.F2RContractError("anchor rows do not re-hash to the pinned anchor_digest")
    rows.validate()
    return rows, receipt


# --- S1 orchestration (refit / aggregate); S0: argument guards only -----------------------------


def build_labellers(variant: str, cache_dir: Path) -> dict[str, Any]:
    """One ``TeacherLabeller`` per start on the saved privileged caches (``f2r_labeller.load_cache``);
    the pinned V26_39D teacher is loaded read-only and its actor digest re-verified."""
    import f2r_labeller as L
    import f1_obs_adapter as OA

    if str(R.BASELINE_DIR) not in sys.path:
        sys.path.insert(0, str(R.BASELINE_DIR))
    import warm_start as W

    module = Path(R.TEACHER["module"])
    if C.sha256_file(module / "module_state.pkl") != R.TEACHER["module_state_sha256"]:
        raise R.F2RContractError("teacher module_state.pkl differs from its pin")
    state = W.load_module_state(module)
    digest = W.actor_state_digest(state)
    if digest != R.TEACHER["actor_digest"]:
        raise R.F2RContractError("teacher actor digest differs from its pin")
    arrays = DS.load_actor_arrays(state, expected_width=R.MODULE_WIDTH_39)
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    spec = OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)
    return {start: L.TeacherLabeller(variant, arrays, spec, L.load_cache(cache_dir, start), teacher_digest=digest) for start in R.STARTS}


def fit_arrays_from_rows(rows: DatasetRows, labellers: Mapping[str, Any]) -> dict[str, Any]:
    """Arrays for ``f2r_refit.fit_student``: ``obs35``/``actions``/``seed``/``purpose`` from the rows and
    the privileged ``clock`` label looked up on each start's cache at the row's exact ``t_pre``
    (auxiliary head target only; never a student feature)."""
    rows.validate()
    clock = np.empty((rows.size, 2), dtype=np.float32)
    for start in sorted(set(rows.start.tolist())):
        mask = rows.start == start
        cache = labellers[start].cache
        idx = cache.lookup(rows.t_pre[mask])
        clock[mask] = cache.clock[idx].astype(np.float32)
    return {"obs35": rows.obs35, "actions": rows.actions, "clock": clock, "seed": rows.seed, "purpose": rows.purpose}


def latest_dataset(dataset_dir: Path, variant: str, round_index: int) -> tuple[DatasetRows, dict[str, Any], Path]:
    """``D_r`` of ``variant``: the single receipt ``f2r_dataset_<variant>_D<r>_*.json`` in ``dataset_dir`` (ambiguity = contract error)."""
    hits = sorted(Path(dataset_dir).glob(f"{DATASET_PREFIX}{variant}_D{round_index}_*.json"))
    if len(hits) != 1:
        raise R.F2RContractError(f"expected exactly one dataset D{round_index} of {variant} in {C.rel(dataset_dir)}, found {len(hits)}")
    rec = hits[0]
    rows, receipt = load_dataset(rec.with_suffix(".npz"), rec)
    return rows, receipt, rec


def refit_round_s1(variant: str, round_index: int, *, dataset: str, out_dir: Path, cache_dir: Path, dataset_dir: Path, refit_dir: Path) -> dict[str, Any]:
    """S1: build/load ``D_r``, fit the student from ``theta_{r-1}`` (JUL_H0 for r = 1) with the
    frozen budget, export ``rl_module_student`` (35D pure, no-clobber).  Never called in S0."""
    import f2r_refit as RF

    plan = round_plan(variant, round_index, protocol=R.load_protocol())
    labellers = build_labellers(variant, cache_dir)
    if round_index == 1:
        if dataset != "anchors":
            raise R.F2RContractError("round 1 uses the anchors-only dataset D1")
        rows = anchor_rows(_PerStart(labellers), variant=variant)
        saved = save_dataset(dataset_dir, f"{variant}_D1_{C.utc_now().replace(':', '').replace('-', '')[:15]}", rows, {"round": 1, "source": "anchors_only"})
        if variant == "T2":
            init_module, init_digest = t2_round1_init_module(refit_dir=refit_dir, gate_dir=R.OUT_GATE, rollouts_dir=R.OUT_ROLLOUTS)  # T1R student, only behind a verified A PASS
            init_state, init_info = RF.load_init_state(init_module, expected_actor_digest=init_digest)
        else:
            init_module = Path(R.INIT_PRIMARY["module"])
            init_state, init_info = RF.load_init_state(init_module)
        anchor_state = init_state
    else:
        rows, receipt, rec = latest_dataset(dataset_dir, variant, round_index)
        saved = {"receipt": str(rec), "rows_digest": receipt["rows_digest"]}
        init_module = refit_dir / variant / f"round_{round_index - 1}" / "rl_module_student"
        prev_report = C.read_json(refit_dir / variant / f"round_{round_index - 1}" / "f2r_refit_report.json")
        init_state, init_info = RF.load_init_state(init_module, expected_actor_digest=prev_report["new_actor_digest"])
        anchor_state = init_state
    data = fit_arrays_from_rows(rows, labellers)
    new_state, report = RF.fit_student(init_state, data, budget=R.REFIT_BUDGET, anchor_state=anchor_state)
    rt: dict[str, Any] = {}
    full = RF.export_student(init_module, out_dir, new_state, report, names35=list(R.FEATURE_NAMES_35), extra={"variant": variant, "round": round_index, "dataset": saved, "plan": plan, "init": init_info}, runtime_status=rt)
    return {"dataset": saved, "export": {k: full.get(k) for k in ("new_actor_digest", "save_reload_exact", "clock_invariance")}, "module": C.rel(Path(out_dir) / "rl_module_student"), "runtime_transaction": {"canonical": False, "persisted": False, **rt}}


class _PerStart:
    """Labeller facade used by ``anchor_rows``/``rows_from_job``: picks the start from the
    first call argument grid by exact t_pre membership of the corresponding cache."""

    def __init__(self, labellers: Mapping[str, Any]) -> None:
        self._labellers = dict(labellers)

    def label(self, obs35: Any, t_pre: Any) -> Any:
        t = np.asarray(t_pre, dtype=np.float64).reshape(-1)
        hits = [s for s, l in self._labellers.items() if np.isin(t, l.cache.t_pre).all()]
        if len(hits) != 1:
            raise R.F2RContractError(f"t_pre grid matches {len(hits)} start caches (need exactly one)")
        return self._labellers[hits[0]].label(obs35, t_pre)


def aggregate_round_s1(variant: str, round_index: int, *, out_dir: Path, cache_dir: Path, dataset_dir: Path, rollouts_dir: Path) -> dict[str, Any]:
    """S1: ``D_{r+1} = D_r`` + labelled rows of the det (123) and stoch (123/124) jobs of round r;
    validation-gate jobs (125) are structurally excluded (``f2r_matrix`` purpose); no-clobber save."""
    import f2r_matrix as MX

    plan = round_plan(variant, round_index, protocol=R.load_protocol())
    if not plan["aggregation_allowed_after_fail"]:
        raise R.F2RContractError(f"aggregation after round {round_index} of {variant} is not allowed by the budget/sequence")
    labellers = _PerStart(build_labellers(variant, cache_dir))
    current, receipt, _ = latest_dataset(dataset_dir, variant, round_index)
    parts = []
    for spec in MX.build_jobs():
        if spec["phase"] != variant or spec["round"] != round_index or not spec["enters_dataset"]:
            continue
        job_dir = rollouts_dir / spec["phase"] / f"round_{spec['round']}" / spec["job_id"]
        rec = C.read_json(job_dir / MX.RECEIPT_FILE)
        if rec.get("status") != "ok":
            raise R.F2RContractError(f"job {spec['job_id']} is not status ok")
        parts.append(rows_from_job(job_dir, purpose=spec["purpose"], labeller=labellers, start=spec["start"], seed=spec["seed"], variant=variant, job_id=spec["job_id"]))
    rows, report = aggregate(current, parts, anchor_digest=current.anchor_digest)
    saved = save_dataset(out_dir, f"{variant}_D{round_index + 1}_{C.utc_now().replace(':', '').replace('-', '')[:15]}", rows, {"round": round_index + 1, "aggregation_report": report, "from": receipt.get("rows_digest")})
    return {"saved": saved, "report": report}


# --- T1R corrective commissioning: dataset (two roles) and refit orchestration ---------------------------

T1R_EXPECTED_COUNTS = {"task_rows": 2994, "preservation_rows": 1176, "cross_role_collisions": 3, "task_within_role_duplicates": 3, "preservation_within_role_duplicates": 0}


class RecordedActionLabeller:
    """Preservation targets of a deterministic job: its RECORDED actions (``raw_policy_action`` of the
    trace = deterministic mean of the policy that produced the rollout), never a teacher label.
    ``label(obs35, t_pre)`` requires the exact ``t_pre`` grid of the job (fail-closed)."""

    def __init__(self, job_dir: Path) -> None:
        import f1_obs_adapter as OA

        job_dir = Path(job_dir)
        rows = C.read_json(job_dir / DS.TRACE_FILE); reset = C.read_json(job_dir / "rollout_reset_diagnostics.json")
        self.t_pre = np.asarray(OA.t_pre_from_trace(float(reset["time"]), [float(r["time"]) for r in rows]), dtype=np.float64)
        self.actions = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float32)
        if self.actions.shape != (len(rows), R.ACTION_DIM) or not np.all(np.isfinite(self.actions)):
            raise R.F2RContractError(f"recorded actions malformed in {C.rel(job_dir)}")
        self.job_dir = job_dir

    def label(self, obs35: Any, t_pre: Any) -> dict[str, Any]:
        t = np.asarray(t_pre, dtype=np.float64).reshape(-1)
        if t.shape != self.t_pre.shape or not np.array_equal(t, self.t_pre):
            raise R.F2RContractError("recorded-action labeller: t_pre grid differs from the job trace (exact float equality required)")
        if np.asarray(obs35).shape[0] != self.actions.shape[0]:
            raise R.F2RContractError("recorded-action labeller: row count mismatch")
        return {"actions": self.actions.copy(), "source": "recorded_raw_policy_action", "job_dir": C.rel(self.job_dir)}


def _row_digests(obs: np.ndarray) -> list[str]:
    return [hashlib.sha256(np.ascontiguousarray(row).tobytes()).hexdigest() for row in np.asarray(obs, dtype=np.float32)]


def dedup_within_role(rows: DatasetRows, *, role: str) -> tuple[DatasetRows, dict[str, Any]]:
    """Bitwise-identical inputs within one role: keep the first occurrence; identical inputs with
    DIFFERENT targets are a label conflict -> ``F2RContractError`` (fail-closed)."""
    keys = [row.tobytes() for row in np.ascontiguousarray(rows.obs35)]
    first: dict[bytes, int] = {}; keep = np.zeros(rows.size, dtype=bool); dups = []; conflicts = []
    for i, k in enumerate(keys):
        if k in first:
            j = first[k]
            if not np.array_equal(rows.actions[i], rows.actions[j]):
                conflicts.append({"kept_index": j, "dropped_index": i, "job_kept": str(rows.job_id[j]), "job_dropped": str(rows.job_id[i]), "target_delta": (rows.actions[i].astype(np.float64) - rows.actions[j].astype(np.float64)).tolist()})
            dups.append({"kept": f"{rows.job_id[j]}#{int(rows.step[j])}", "dropped": f"{rows.job_id[i]}#{int(rows.step[i])}", "t_pre": float(rows.t_pre[i])})
            continue
        first[k] = i; keep[i] = True
    if conflicts:
        raise R.F2RContractError(f"{role}: {len(conflicts)} bitwise-identical inputs with different targets within the role: {json.dumps(conflicts[:3])[:800]}")
    return rows.subset(keep), {"role": role, "rows_raw": rows.size, "duplicates_dropped": len(dups), "duplicates": dups, "rows": int(keep.sum())}


def remove_cross_role_collisions(task: DatasetRows, pres: DatasetRows, *, reset_times: Mapping[str, float]) -> tuple[DatasetRows, dict[str, Any]]:
    """Preservation wins: every task row whose input is bitwise present in the preservation role is
    removed from the task role.  Structural validation (fail-closed): the collisions must be EXACTLY
    the reset rows — one per start, ``t_pre == reset_time[start]`` on both sides, ``step == 1`` — and
    nothing else.  Each collision is reported with the input digest, both targets and their delta
    (the targets are NOT identical: T2 label vs recorded JUL action)."""
    pres_keys = {row.tobytes(): i for i, row in enumerate(np.ascontiguousarray(pres.obs35))}
    records = []; drop = np.zeros(task.size, dtype=bool)
    for i, row in enumerate(np.ascontiguousarray(task.obs35)):
        j = pres_keys.get(row.tobytes())
        if j is None:
            continue
        drop[i] = True
        s = str(task.start[i])
        problems = []
        if str(pres.start[j]) != s:
            problems.append("collision across different starts")
        if float(task.t_pre[i]) != float(reset_times.get(s, float("nan"))) or float(pres.t_pre[j]) != float(reset_times.get(s, float("nan"))):
            problems.append("collision is not at the reset time of its start")
        if int(task.step[i]) != 1 or int(pres.step[j]) != 1:
            problems.append("collision is not the first (pre-action) observation")
        records.append({"start": s, "input_sha256": _row_digests(task.obs35[i: i + 1])[0], "t_pre": float(task.t_pre[i]), "task_row": f"{task.job_id[i]}#{int(task.step[i])}", "pres_row": f"{pres.job_id[j]}#{int(pres.step[j])}", "task_target_knee_ankle": task.actions[i].astype(np.float64).tolist(), "pres_target_knee_ankle": pres.actions[j].astype(np.float64).tolist(), "target_delta_knee_ankle": (task.actions[i].astype(np.float64) - pres.actions[j].astype(np.float64)).tolist(), "problems": problems})
    bad = [r for r in records if r["problems"]]
    if bad or len(records) != len(R.STARTS) or sorted(r["start"] for r in records) != sorted(R.STARTS):
        raise R.F2RContractError(f"cross-role collisions must be exactly the {len(R.STARTS)} reset rows (one per start): found {len(records)}, problems {json.dumps([r['problems'] for r in bad])[:500]}")
    return task.subset(~drop), {"rule": "preservation wins: task rows with an input bitwise present in the preservation role are removed from the task role", "collisions": records, "count": len(records)}


def build_t1r_dataset(labellers_t2: Mapping[str, Any], *, t1_jobs: Mapping[str, Path] | None = None, jul_jobs: Mapping[str, Path] | None = None, expected_counts: Mapping[str, int] | None = T1R_EXPECTED_COUNTS) -> tuple[DatasetRows, DatasetRows, dict[str, Any]]:
    """T1R two-role dataset (architect decisions 2, 3, 5):
    task = 3 pinned anchors (purpose ``anchor``) + on-policy states of the 3 T1 deterministic rollouts
    (purpose ``det``, seed 123), labels knee u_T / ankle u_IK (T2 labellers, exact t_pre lookup);
    preservation = the 3 pinned JUL_H0 deterministic jobs (purpose ``det``, seed 123), target = the
    recorded JUL_H0 action (never u_T).  Dedup within role (conflicts fail closed); cross-role
    collisions = exactly the 3 reset rows, removed from the task role and reported with input digest
    and target delta.  Counts must equal ``expected_counts`` (fail-closed)."""
    t1_jobs = dict(R.T1_JOBS) if t1_jobs is None else {k: Path(v) for k, v in t1_jobs.items()}
    jul_jobs = dict(R.P0_JUL_JOBS) if jul_jobs is None else {k: Path(v) for k, v in jul_jobs.items()}
    disp = _PerStart(labellers_t2)
    task_parts = [rows_from_job(Path(spec["job_dir"]), purpose="anchor", labeller=disp, start=start, seed=R.DET_SEED, variant="T1R", job_id=f"anchor:{start}") for start, spec in R.ANCHORS.items()]
    task_parts += [rows_from_job(t1_jobs[start], purpose="det", labeller=disp, start=start, seed=R.DET_SEED, variant="T1R", job_id=f"t1:{start}") for start in R.STARTS]
    pres_parts = [rows_from_job(jul_jobs[start], purpose="det", labeller=RecordedActionLabeller(jul_jobs[start]), start=start, seed=R.DET_SEED, variant="T1R", job_id=f"jul:{start}") for start in R.STARTS]
    task_raw = DatasetRows.concat(task_parts, variant="T1R"); pres_raw = DatasetRows.concat(pres_parts, variant="T1R")
    task_raw.validate(); pres_raw.validate()
    task_d, st_task = dedup_within_role(task_raw, role="task"); pres_d, st_pres = dedup_within_role(pres_raw, role="preservation")
    reset_times = {start: float(labellers_t2[start].cache.t_pre[0]) for start in R.STARTS}
    task_f, collisions = remove_cross_role_collisions(task_d, pres_d, reset_times=reset_times)
    task_f.anchor_digest = task_f.anchor_rows().digest()
    info = {"roles": {"task": {"purposes": task_f.per_purpose_counts(), "starts": task_f.per_start_counts(), "labels": "knee u_T (teacher, A_iso6clk route) / ankle u_IK (prescribed IK, exact t_pre lookup)", "jobs": sorted(set(task_f.job_id.tolist()))}, "preservation": {"purposes": pres_d.per_purpose_counts(), "starts": pres_d.per_start_counts(), "targets": "recorded JUL_H0 deterministic action (raw_policy_action), never u_T", "jobs": sorted(set(pres_d.job_id.tolist()))}},
            "dedup_within_role": {"task": st_task, "preservation": st_pres}, "cross_role": collisions, "reset_times": reset_times,
            "counts": {"task_rows": task_f.size, "preservation_rows": pres_d.size, "cross_role_collisions": collisions["count"], "task_within_role_duplicates": st_task["duplicates_dropped"], "preservation_within_role_duplicates": st_pres["duplicates_dropped"]},
            "seeds": {"task": task_f.per_seed_counts(), "preservation": pres_d.per_seed_counts()}, "digests": {"task": task_f.digest(), "preservation": pres_d.digest(), "task_anchor_rows": task_f.anchor_digest}}
    if expected_counts is not None:
        mism = {k: (info["counts"][k], v) for k, v in expected_counts.items() if info["counts"].get(k) != v}
        if mism:
            raise R.F2RContractError(f"T1R dataset counts differ from the preregistered ones (actual, expected): {mism}")
    return task_f, pres_d, info


def t1r_fit_arrays(task: DatasetRows, pres: DatasetRows, labellers: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    return fit_arrays_from_rows(task, labellers), fit_arrays_from_rows(pres, labellers)


def refit_t1r_s1(out_dir: Path, *, cache_dir: Path, dataset_dir: Path) -> dict[str, Any]:
    """S1 (never in tooling tests on real data): pins (anchors, T1 student + rollouts, JUL_H0 jobs, F0
    chain) -> two-role dataset -> save both roles (no-clobber) -> ``fit_student_preserving`` from the
    JUL_H0 init with the frozen budget and beta = 1 -> criteria P1-P3b/P4 (fail-closed: no export on
    any FAIL) -> export ``rl_module_student`` (35D pure)."""
    import f2r_refit as RF

    if R.verify_anchor_pins().get("all_match") is not True:
        raise R.F2RContractError("anchor pins do not verify: T1R refit aborted before reading any data")
    R.require_t1_pins(); R.require_p0_jul_pins()
    plan = round_plan("T1R", 1, protocol=R.load_protocol())
    labellers = build_labellers("T2", cache_dir)
    task, pres, info = build_t1r_dataset(labellers)
    stamp = C.utc_now().replace(":", "").replace("-", "")[:15]
    saved_task = save_dataset(dataset_dir, f"T1R_task_{stamp}", task, {"round": 1, "role": "task", "t1r": info})
    saved_pres = save_dataset(dataset_dir, f"T1R_pres_{stamp}", pres, {"round": 1, "role": "preservation", "t1r": info})
    init_module = Path(R.INIT_PRIMARY["module"]); init_state, init_info = RF.load_init_state(init_module)  # JUL_H0, pinned digest — never the T1 student
    task_data, pres_data = t1r_fit_arrays(task, pres, labellers)
    new_state, report = RF.fit_student_preserving(init_state, task_data, pres_data, budget=R.REFIT_BUDGET, beta=RF.T1R_BETA)
    criteria = RF.assert_t1r_criteria(report)  # fail-closed before any export / rollout
    rt: dict[str, Any] = {}
    full = RF.export_student(init_module, out_dir, new_state, report, names35=list(R.FEATURE_NAMES_35), extra={"variant": "T1R", "round": 1, "datasets": {"task": saved_task, "preservation": saved_pres}, "t1r_dataset": info, "plan": plan, "init": init_info, "criteria": criteria}, runtime_status=rt)
    # P4 save/reload + clock invariance are verified INSIDE the export before the commit (a failure raises there, nothing is promoted)
    return {"datasets": {"task": saved_task, "preservation": saved_pres}, "counts": info["counts"], "criteria": criteria, "export": {k: full.get(k) for k in ("new_actor_digest", "save_reload_exact", "clock_invariance")}, "module": C.rel(Path(out_dir) / "rl_module_student"), "runtime_transaction": {"canonical": False, "persisted": False, **rt}}


def t2_round1_init_module(*, refit_dir: Path | None = None, gate_dir: Path | None = None, rollouts_dir: Path | None = None) -> tuple[Path, str]:
    """T2 round 1 starts from the T1R student ONLY behind a verified T1R gate artefact with A PASS
    (``f2r_gates.verify_t1r_unlock``); otherwise fail-closed (no fallback to JUL_H0 or T1)."""
    import f2r_gates as G

    unlock = G.verify_t1r_unlock(gate_dir=gate_dir, rollouts_dir=rollouts_dir, refit_dir=refit_dir)
    if unlock.get("unlocked") is not True:
        raise R.F2RContractError(f"T2 round 1 is locked: {unlock.get('reason')}")
    return Path(unlock["student_module"]), str(unlock["student_actor_digest"])


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="F2R DAgger orchestration (S1 only; S0 prints the round plan).")
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "refit", "aggregate", "refit-t1r"):
        p = sub.add_parser(name)
        if name != "refit-t1r":
            p.add_argument("--variant", required=True, choices=("T1", "T1R", "T2", "T3"))
            p.add_argument("--round", type=int, required=True)
        if name != "plan":
            p.add_argument("--out-dir", required=True)
            p.add_argument("--authorized-stage", default=None)
        if name == "refit":
            p.add_argument("--dataset", default="anchors")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.cmd == "plan":
        print(json.dumps(round_plan(args.variant, args.round, protocol=R.load_protocol()), indent=2))
        return 0
    if args.authorized_stage != "S1":
        raise SystemExit(f"{args.cmd} performs a real {'fit' if args.cmd in ('refit', 'refit-t1r') else 'aggregation'}: not authorised in S0; pass --authorized-stage S1 after the architect's go")
    if args.cmd == "refit-t1r":
        out = refit_t1r_s1(Path(args.out_dir), cache_dir=R.OUT_CACHE, dataset_dir=R.OUT_DATASETS)
    elif args.cmd == "refit":
        if args.variant == "T1R":
            raise SystemExit("use the refit-t1r sub-command for the corrective commissioning")
        out = refit_round_s1(args.variant, args.round, dataset=args.dataset, out_dir=Path(args.out_dir), cache_dir=R.OUT_CACHE, dataset_dir=R.OUT_DATASETS, refit_dir=R.OUT_REFIT)
    else:
        out = aggregate_round_s1(args.variant, args.round, out_dir=Path(args.out_dir), cache_dir=R.OUT_CACHE, dataset_dir=R.OUT_DATASETS, rollouts_dir=R.OUT_ROLLOUTS)
    print(json.dumps(out, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
