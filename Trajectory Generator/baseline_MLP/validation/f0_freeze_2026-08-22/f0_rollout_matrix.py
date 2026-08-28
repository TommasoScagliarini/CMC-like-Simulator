"""F0 activity 5: comparable deterministic/stochastic rollout matrix (driver).

Default mode is DRY-RUN: the driver only writes the manifest (job list,
commands, provenance, comparison classes). Real rollouts require the explicit
``--execute`` flag, which is gated by the architectural review of the
manifest. Every job pins an immutable resolved config
(``--no-auto-config --config``), an explicit start offset, action-selection
mode and seed, and writes into a fresh no-clobber directory with a receipt
(command, SHA-256 of module state / config / rollout_eval.py / outputs,
effective interpreter, git HEAD, timings, exit code).

Status ``ok`` requires ALL of: return code 0, ``rollout_summary.json``
present with ``ok`` exactly ``True``, and ``rollout_policy_trace.json``
present. Anything else is ``failed``.

Existing output directories are never silently reused. Before the thread
pool is created, a SYNCHRONOUS preflight verifies every existing output of
the whole matrix: ``f0_receipt.json`` must match the fresh job description on
job_id, normalised full command, SHA-256 of module state / config /
rollout_eval.py, git HEAD and output_dir; the receipt status must be ``ok``;
``rollout_summary.json`` and ``rollout_policy_trace.json`` must be present,
``summary.ok`` must be ``True`` and their SHA-256 recomputed from disk must
equal the receipt digests. A single invalid existing output aborts the driver
before any job starts. ``--retry-tag`` allocates explicit fresh directories.

Job families (see ``MATRIX_EXPLANATION``):
  replay - historical configuration replayed on the CURRENT code (HEAD) with
           the historical resolved YAML and weights: NOT a bit-exact
           reproduction (the historical commit is not isolated in F0); any
           divergence from the recorded values measures code changes;
  det    - deterministic comparables under the pinned v3 runtime (3 starts);
  stoch  - stochastic comparables (development seeds 123/124/125, 3 starts);
  ctrl39 - 39-feature V26 imitation actor under its native runtime.

Comparison classes (recorded per job, never collapsed):
  isometric_comparable                      - 35D B0820 candidates under the pinned v3 runtime;
  historical_control                        - July checkpoints under the v3 runtime: same runtime,
                                              policy never trained/adapted under V26 semantics;
  compatibility_control_39D                 - native-39D actor under its own imitation runtime;
  historical_config_replay_on_current_code  - replay family above.

The interpreter for rollout subprocesses is resolved by
``f0_common.select_python`` (env override / sys.executable / CONDA_PREFIX);
in dry-run a failed resolution is recorded in the manifest instead of
aborting, because no subprocess is launched.

Receipt schemas and provenance classes (``f0_closure``):
  schema 4 (the 28 replay/det receipts already produced) - verified through
           the clearly labelled LEGACY branch ``legacy_schema_v4_class_B``:
           receipt fields, status/returncode, output digests. They are never
           rewritten and carry no contemporaneous closure (Class B,
           retrospective correlated evidence, see
           ``f0_source_closure_assessment``);
  schema 5 (every NEW job) - in addition, a contemporaneous source closure:
           ``runtime_source_closure_digest_pre/post`` (runtime-core sources +
           native plugins + data assets, recomputed immediately before and
           after the subprocess), ``orchestration_digest`` (F0 scripts, kept
           separate), resolved native plugin paths + SHA-256, environment
           fingerprint, closure manifest path + SHA-256 written at launch,
           ``source_closure_unchanged``. Status ``ok`` additionally requires
           pre == post == manifest digest; a changed or missing closure fails
           the job closed. New receipts are labelled
           ``provenance_class = B_plus_contemporaneous`` - NOT Class A: the
           job still runs from the live working tree, not from a physically
           immutable bundle (limitation recorded in every receipt). No
           symlink is accepted in the closure or as output directory; receipt
           paths are repository-relative POSIX.

No training, no weight modification. Seeds 126-128 are never used.
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402
import f0_closure as CL  # noqa: E402

HIST = C.RUNS_ROLLOUT
JUL_REF = HIST / "validation" / "robust_reference_runs" / "2026-07-14_h0_schema2"
B0820_QUAL = "2026-08-20_B0820_native_v26_frozen_actor_iter5_rollout_qual_B0820"
PYTHON_PLACEHOLDER = "<python: unresolved in dry-run>"
RECEIPT_VERIFY_FIELDS = ("job_id", "command_normalized", "module_state_sha256", "config_sha256", "rollout_eval_sha256", "git_head", "output_dir", "historical_reference_summary_sha256")
OUTPUT_DIGEST_FIELDS = ("summary_sha256", "trace_sha256")
RECEIPT_SCHEMA_VERSION = 5
LEGACY_RECEIPT_SCHEMA_VERSIONS = (4,)
BRANCH_LEGACY_V4 = "legacy_schema_v4_class_B"
BRANCH_CONTEMPORANEOUS_V5 = "contemporaneous_schema_v5_class_B_plus"
CLOSURE_RECEIPT_FIELDS = (
    "runtime_source_closure_digest_pre", "runtime_source_closure_digest_post", "orchestration_digest",
    "runtime_core", "native_plugins", "data_assets", "orchestration", "environment_fingerprint",
    "job_inputs", "job_inputs_digest_pre", "job_inputs_digest_post", "job_inputs_unchanged",
    "closure_manifest", "closure_manifest_sha256", "closure_manifest_verified_pre_launch", "closure_manifest_verified_post", "closure_manifest_unchanged",
    "source_closure_unchanged", "provenance_class", "provenance_limitation",
)
FAMILIES = ("replay", "det", "stoch", "ctrl39")
CLASSES = ("isometric_comparable", "historical_control", "compatibility_control_39D", "historical_config_replay_on_current_code")
SUMMARY_FILE = "rollout_summary.json"
TRACE_FILE = "rollout_policy_trace.json"
RECEIPT_FILE = "f0_receipt.json"
# receipt identity fields that output_digest must never overwrite with summary values (kept under summary_fields)
SUMMARY_KEYS_NOT_COPIED = ("action_selection", "action_seed", "episode_start_offset_s", "n_actor")

MATRIX_EXPLANATION = {
    "purpose": "Fase 0, attivita 5-7 del piano 2026-08-22: ricampionare con un unico protocollo (stessi start esatti, stesso seed deterministico 123, stessi seed di sviluppo stocastici 123/124/125, stessa durata 5 s / 500 step, stessa config risolta congelata) le baseline da confrontare, e rieseguire le configurazioni storiche sul codice corrente per misurare e spiegare ogni divergenza dai valori registrati.",
    "families": {
        "replay (9)": "replay della configurazione storica sul codice HEAD corrente (yaml risolto e pesi storici, codice di oggi): JUL_H0 x3 start det + JUL_H0 +0.20 stocastico seed 123 (valori registrati 14/07), JUL_BEST nominale det (valore registrato 15/07, +52.42694) piu' -0.20/+0.20 det (nuovi), B0820_V2_BEST nominale det e B0820_H0 nominale det sotto FSM v2 (valori registrati 21/08). NON e' una riproduzione bit-exact: il commit storico non viene isolato in F0; una divergenza misura l'effetto delle modifiche di codice (FSM v3, corridoio, detector, reward) a parita' di yaml e pesi. Classe: historical_config_replay_on_current_code.",
        "det (19)": "deterministici sotto runtime v3 pinnato (resolved yaml della run fsmv3_fixedcorridor): B0820_H0 x3, B0820_V3_BEST x3 (+1 ripetizione in-sessione del nominale), B0820_V3_LAST x3, B0820_V2_BEST x3 (13 isometric_comparable); JUL_H0 x3 e JUL_BEST x3 sotto v3 (6 historical_control, non isometrici rispetto alla policy).",
        "stoch (27)": "stocastici sotto runtime v3 pinnato, seed di sviluppo 123/124/125 x 3 start: B0820_H0 e B0820_V3_BEST (sigma state-dependent ~0.5; isometric_comparable) e JUL_H0 (sigma costante 0.005; historical_control).",
        "ctrl39 (3)": "controllo 39D: actor imitativo nativo V26 sotto il proprio runtime imitativo con FSM v3 esplicita; return NON comparabile (reward imitation), cinematica/eventi comparabili solo come controllo (compatibility_control_39D).",
    },
    "total_jobs": 58,
    "estimated_cost": "circa 10-13 min per rollout da 500 step in sequenza; con 5 worker circa 2-2.5 h di wall time; ~50 MB per rollout (trace + sim_outputs).",
    "seeds": {"deterministic_action_seed": 123, "development_stochastic_seeds": list(C.DEVELOPMENT_SEEDS), "sealed_seeds_never_used": list(C.SEALED_SEEDS)},
    "isometry_rule": "Un confronto e' isometrico solo se candidati 35D della stessa catena (B0820) girano sotto lo stesso runtime pinnato (v3_canonical). I checkpoint di luglio e l'actor 39D sono controlli: il piano li vincola esplicitamente a 'controlli di compatibilita', non confronti fra policy a parita di runtime'.",
    "status_ok_rule": "status ok <=> returncode 0 AND rollout_summary.json presente con ok == True AND rollout_policy_trace.json presente.",
    "skip_policy": "preflight sincrono su tutti gli output esistenti della matrice prima della creazione del pool: receipt coincidente su " + ", ".join(RECEIPT_VERIFY_FIELDS) + " (None valido per job senza riferimento storico), status ok E returncode == 0, summary.ok == True, summary e trace presenti con SHA-256 ricalcolati uguali a quelli della receipt; una sola receipt invalida impedisce l'avvio di qualunque job; --retry-tag alloca esplicitamente nuove directory.",
    "provenance": "receipt schema 4 (28 job replay/det gia' prodotti): ramo di verifica legacy " + BRANCH_LEGACY_V4 + ", classe B retrospettiva, mai riscritte; receipt schema 5 (ogni nuovo job): chiusura contemporanea " + ", ".join(CLOSURE_RECEIPT_FIELDS) + " - chiusura runtime (sorgenti, plugin nativi, asset, script F0, fingerprint dell'interprete di rollout sondato in sottoprocesso) e input scientifici per job (config + file risolti dal setup: .osim, IK .mot, ExternalForces.xml + GRF .mot, CMC_Actuators.xml, profili; tutti i file del modulo RLModule; rollout_eval; riferimento storico) ricalcolati prima e dopo il sottoprocesso e confrontati per uguaglianza canonica profonda con il manifest di lancio (status ok solo se nulla e' cambiato), classe " + CL.PROVENANCE_CLASS_CONTEMPORANEOUS + " - NON classe A: esecuzione dal working tree vivo, non da un bundle fisicamente immutabile.",
    "closure_manifest_on_disk": "validatore unico f0_closure.verify_manifest_file (file regolare senza simlink, SHA-256 esatto closure_manifest_sha256, JSON mapping con closure_schema_version, tabelle e fingerprint ben formati, digest runtime/orchestrazione ricalcolati dalle tabelle del manifest e uguali a quelli del riferimento, sezioni canonicamente uguali al riferimento, job_inputs[job_id] == digest degli input del job), chiamato immediatamente PRIMA del sottoprocesso (manifest mancante/manomesso => job non lanciato, returncode None) e subito DOPO (mancante/manomesso => status failed anche con rc 0) e in verify_existing con la receipt come riferimento.",
    "command_binding": "validatore unico command_binding_problems, chiamato in run_job PRIMA di qualunque accesso al filesystem (receipt non legata => eccezione, nessuna directory, nessun sottoprocesso) e in verify_existing (schema 4 senza fingerprint, schema 5 con fingerprint): job_id deve nominare un job dell'indice canonico univoco di build_jobs() e ogni campo di identita' della receipt (family, candidate, runtime, start, action_selection, seed, repeat, riferimento storico) deve essere uguale in tipo e valore alla spec canonica; retry_tag None o singolo componente [A-Za-z0-9][A-Za-z0-9._-]{0,63}; output_dir esattamente C.rel(OUT_ROLLOUTS/family/(job_id|job_id__tag)) derivato dalla spec canonica (relativo, POSIX, senza backslash, '.' o '..'); intero argv RAW uguale token per token a job_command(spec canonica, dir canonica, python) senza alcuna normalizzazione (command_normalized resta solo campo descrittivo d'integrita') - quindi versione FSM, flag diagnostiche, timeout, --record-*/--no-progress, seed/famiglia e path dello script (anche una copia byte-identica o un bridge symlink/..) sono vincolati; command[1] raw esattamente str(C.ROLLOUT_EVAL), senza '.'/'..'/backslash/symlink in alcun componente, file regolare con realpath == realpath(C.ROLLOUT_EVAL) e SHA-256 == rollout_eval_sha256; inoltre command lista non vuota di stringhe, normalize_command(command) == command_normalized ricalcolato, command[0] == python e realpath(command[0]) == realpath(fingerprint.requested_executable) == fingerprint.executable_realpath, command[1] = script esistente (no simlink) con SHA-256 == rollout_eval_sha256, cwd == radice repo, --checkpoint/--config/--output-dir esattamente una volta in forma 'flag valore' con valore risolto == module/config/output_dir risolti, --no-auto-config esattamente una volta, --seed/--action-selection/--episode-start-offset-s esattamente una volta con la stringa emessa; duplicati, flag mancanti, flag senza valore, valori discordanti e token 'flag=valore' respinti.",
    "ctrl39_actor_contract": "runtime v26_imitation_native lanciato con " + C.NO_CONTROLLER_DIAGNOSTIC_FLAG + ": il resolved yaml V26 ha include_controller_diagnostic_observation: true (actor 43 = 39 + 4 diagnostiche sea_u_abs/saturated) mentre il modulo consuma 39; con il flag le 4 diagnostiche passano al suffisso privilegiato nello stesso punto d'inserzione (osim_trj_cmc_like.py priv.update(controller_diagnostics)), osservazione completa 88 con ordine invariato e prefisso actor esattamente 39 (manifest content-addressed " + C.ACTOR_MANIFEST_39_SHA256[:12] + "...); n_actor = 43 non e' accettato.",
}

# (family, candidate, runtime, start, mode, seed, repeat, historical reference summary)
JobSpec = tuple[str, str, str, str, str, int, int, Path | None]


def comparison_class(family: str, cand: str, runtime: str) -> str:
    if family == "replay":
        return "historical_config_replay_on_current_code"
    if family == "ctrl39":
        return "compatibility_control_39D"
    if C.CANDIDATES[cand]["lineage"] == "july":
        return "historical_control"
    return "isometric_comparable"


def build_jobs() -> list[JobSpec]:
    jobs: list[JobSpec] = []
    # --- A. historical configuration replayed on current code ---------------
    for start in ("minus020", "nominal", "plus020"):
        jobs.append(("replay", "JUL_H0", "july_legacy", start, "deterministic", 123, 1, JUL_REF / f"deterministic_{start}" / SUMMARY_FILE))
    jobs.append(("replay", "JUL_H0", "july_legacy", "plus020", "stochastic", 123, 1, JUL_REF / "stochastic_plus020_seed123" / SUMMARY_FILE))
    jobs.append(("replay", "JUL_BEST", "july_legacy", "nominal", "deterministic", 123, 1, HIST / "2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded" / SUMMARY_FILE))
    for start in ("minus020", "plus020"):
        jobs.append(("replay", "JUL_BEST", "july_legacy", start, "deterministic", 123, 1, None))
    jobs.append(("replay", "B0820_V2_BEST", "v2_b0820", "nominal", "deterministic", 123, 1, HIST / "MLP_ExNovo_B0820_from_zero_50iter_rollout_exnovo_best_nominal" / SUMMARY_FILE))
    jobs.append(("replay", "B0820_H0", "v2_b0820", "nominal", "deterministic", 123, 1, HIST / f"{B0820_QUAL}_nominal" / SUMMARY_FILE))
    # --- B. comparable matrix under pinned v3 runtime ---------------------
    for start in ("minus020", "nominal", "plus020"):
        jobs.append(("det", "B0820_H0", "v3_canonical", start, "deterministic", 123, 1, HIST / f"{B0820_QUAL}_fsmv3_{start}" / SUMMARY_FILE))
    for start in ("minus020", "nominal", "plus020"):
        ref = HIST / "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter_rollout_exnovo_best_nominal" / SUMMARY_FILE if start == "nominal" else None
        jobs.append(("det", "B0820_V3_BEST", "v3_canonical", start, "deterministic", 123, 1, ref))
    jobs.append(("det", "B0820_V3_BEST", "v3_canonical", "nominal", "deterministic", 123, 2, None))  # in-session repeat
    for cand in ("B0820_V3_LAST", "B0820_V2_BEST", "JUL_H0", "JUL_BEST"):
        for start in ("minus020", "nominal", "plus020"):
            jobs.append(("det", cand, "v3_canonical", start, "deterministic", 123, 1, None))
    for cand in ("B0820_H0", "B0820_V3_BEST", "JUL_H0"):
        for start in ("minus020", "nominal", "plus020"):
            for seed in C.DEVELOPMENT_SEEDS:
                jobs.append(("stoch", cand, "v3_canonical", start, "stochastic", seed, 1, None))
    # --- C. 39D compatibility control -------------------------------------
    for start in ("minus020", "nominal", "plus020"):
        jobs.append(("ctrl39", "V26_39D", "v26_imitation_native", start, "deterministic", 123, 1, None))
    assert all(j[5] not in C.SEALED_SEEDS for j in jobs), "sealed seeds must not be used"
    assert len(jobs) == MATRIX_EXPLANATION["total_jobs"], len(jobs)
    return jobs


def job_id(spec: JobSpec) -> str:
    family, cand, runtime, start, mode, seed, rep, _ = spec
    mode_tag = "det" if mode == "deterministic" else f"stoch_seed{seed}"
    rep_tag = f"_rep{rep}" if rep > 1 else ""
    return f"{cand}__{runtime}__{start}__{mode_tag}{rep_tag}"


@functools.lru_cache(maxsize=1)
def canonical_index() -> dict[str, JobSpec]:
    """Unique job_id -> canonical spec index of the whole matrix (fail-closed on collisions).

    The job_id does not encode every identity field (deterministic jobs share
    seed 123, the family is implicit): the canonical spec is the ONLY source of
    family / candidate / runtime / start / action_selection / seed / repeat /
    historical reference for a given job_id, and argv + output path are derived
    from it, never from untrusted receipt fields."""
    index: dict[str, JobSpec] = {}
    for spec in build_jobs():
        jid = job_id(spec)
        if jid in index:
            raise RuntimeError(f"job_id collision in the canonical matrix: {jid}")
        index[jid] = spec
    return index


RETRY_TAG_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")


def validate_retry_tag(tag: Any) -> str | None:
    """None, or a single portable path component: [A-Za-z0-9][A-Za-z0-9._-]* (max 64
    chars). Slashes, backslashes, '.'/'..', empty strings and any traversal are
    rejected (RuntimeError) BEFORE any filesystem access."""
    if tag is None:
        return None
    if not isinstance(tag, str) or not RETRY_TAG_RE.fullmatch(tag) or tag in (".", "..") or "/" in tag or "\\" in tag:
        raise RuntimeError(f"invalid --retry-tag {tag!r}: must match [A-Za-z0-9][A-Za-z0-9._-]{{0,63}} (single portable path component, no traversal)")
    return tag


def output_dir_for(spec: JobSpec, retry_tag: str | None) -> Path:
    """Canonical output directory of a job: OUT_ROLLOUTS / family / (job_id or job_id__tag)."""
    tag = validate_retry_tag(retry_tag)
    jid = job_id(spec)
    dir_name = jid if tag is None else f"{jid}__{tag}"
    return C.OUT_ROLLOUTS / spec[0] / dir_name


def static_job_record(spec: JobSpec, retry_tag: str | None = None) -> dict[str, Any]:
    """Pure registry projection used by downstream F0 consumers before their input ledger.

    Unlike :func:`describe_job`, this helper performs no filesystem read, hash, existence
    probe, git command or interpreter probe.  It is therefore safe for declaring the exact
    downstream input set before the PRE snapshot.
    """
    family, cand, runtime, start, mode, seed, rep, hist = spec
    retry_tag = validate_retry_tag(retry_tag)
    return {
        "job_id": job_id(spec),
        "family": family,
        "comparison_class": comparison_class(family, cand, runtime),
        "candidate": cand,
        "runtime": runtime,
        "start": start,
        "action_selection": mode,
        "seed": seed,
        "repeat": rep,
        "retry_tag": retry_tag,
        "historical_reference_summary": C.rel(hist) if hist is not None else None,
        "module": C.rel(C.CANDIDATES[cand]["module"]),
        "config": C.rel(C.RUNTIMES[runtime]["config"]),
        "output_dir": C.rel(output_dir_for(spec, retry_tag)),
    }


FROZEN_ANALYSIS_RECEIPT_KEYS = (
    "status", "returncode", "duration_s", "summary_sha256", "trace_sha256",
    "git_head", "config", "config_sha256", "rollout_eval_sha256",
    "module_state_sha256", "candidate", "schema_version", "provenance_class",
    "source_closure_unchanged", "provenance_class_effective",
)


def frozen_analysis_index(analysis: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Validate the pinned analysis as the exact canonical 58-job authority.

    This is deliberately a pure operation: callers first consume the analysis bytes through
    their ledger, then use this projection to bind live receipt/summary/trace/STO bytes to the
    evidence recorded by that analysis.  No live job file is inspected here.
    """
    canonical = [static_job_record(spec) for spec in build_jobs()]
    expected_ids = [rec["job_id"] for rec in canonical]
    jobs = analysis.get("jobs") if isinstance(analysis, dict) else None
    if (
        analysis.get("analysis_complete") is not True
        or analysis.get("job_count_total") != len(canonical)
        or analysis.get("job_count_executed") != len(canonical)
        or not isinstance(jobs, list)
        or len(jobs) != len(canonical)
    ):
        raise RuntimeError("frozen analysis is not the complete canonical 58-job result")
    got_ids = [j.get("job_id") if isinstance(j, dict) else None for j in jobs]
    if got_ids != expected_ids or len(set(got_ids)) != len(expected_ids):
        raise RuntimeError("frozen analysis job index/order differs from the canonical matrix")
    out: dict[str, dict[str, Any]] = {}
    identity_keys = (
        "job_id", "family", "comparison_class", "candidate", "runtime", "start",
        "action_selection", "seed", "repeat", "output_dir",
        "historical_reference_summary",
    )
    for rec, job in zip(canonical, jobs):
        assert isinstance(job, dict)  # established by got_ids above
        expected = {k: rec.get(k) for k in identity_keys}
        observed = {k: job.get(k) for k in identity_keys}
        if observed != expected:
            raise RuntimeError(f"{rec['job_id']}: frozen analysis identity differs from the canonical registry")
        if not (job.get("executed") is True and job.get("integrity") == "PASS" and job.get("verdict") == "PASS_ANALYSED" and job.get("analysis_complete") is True):
            raise RuntimeError(f"{rec['job_id']}: frozen analysis job is not fully PASS_ANALYSED")
        receipt = job.get("receipt")
        if not isinstance(receipt, dict) or tuple(receipt) != FROZEN_ANALYSIS_RECEIPT_KEYS:
            raise RuntimeError(f"{rec['job_id']}: frozen analysis receipt projection schema differs")
        if receipt.get("provenance_class_effective") not in ("B", "B_plus_contemporaneous"):
            raise RuntimeError(f"{rec['job_id']}: frozen analysis effective provenance class is invalid")
        evidence = job.get("evidence")
        if not isinstance(evidence, dict) or evidence.get("_complete") is not True or evidence.get("_missing_or_invalid") != []:
            raise RuntimeError(f"{rec['job_id']}: frozen analysis evidence is incomplete")
        for label, filename in (("summary", SUMMARY_FILE), ("trace", TRACE_FILE)):
            item = evidence.get(label)
            expected_path = C.rel(C.REPO / rec["output_dir"] / filename)
            if not isinstance(item, dict) or item.get("path") != expected_path or item.get("present") is not True or item.get("valid") is not True or not isinstance(item.get("sha256"), str) or not re.fullmatch(r"[0-9a-f]{64}", item["sha256"]):
                raise RuntimeError(f"{rec['job_id']}: frozen analysis {label} evidence is invalid")
        if receipt.get("candidate") != rec["candidate"] or receipt.get("config") != rec["config"]:
            raise RuntimeError(f"{rec['job_id']}: frozen analysis receipt is not registry-bound")
        out[rec["job_id"]] = job
    return out


def job_command(spec: JobSpec, out_dir: Path, python_exe: str) -> list[str]:
    family, cand, runtime, start, mode, seed, rep, _ = spec
    module = C.CANDIDATES[cand]["module"]
    rt = C.RUNTIMES[runtime]
    cmd = [
        python_exe,
        str(C.ROLLOUT_EVAL),
        "--checkpoint", str(module),
        "--no-auto-config",
        "--config", str(rt["config"]),
        "--episode-start-offset-s", repr(C.EXACT_STARTS[start]),
        "--action-selection", mode,
        "--seed", str(seed),
        "--output-dir", str(out_dir),
        "--record-outputs",
        "--record-policy-trace",
        "--no-progress",
        "--run-timeout-s", "7200",
        "--stall-timeout-s", "1500",
        "--step-timeout-s", "900",
        "--startup-timeout-s", "900",
    ]
    cmd.extend(rt["extra_args"])
    return cmd


def normalize_command(cmd: list[str]) -> list[str]:
    """Normalise absolute-path tokens lexically (os.path.normpath), independent of
    whether the path exists yet, so a receipt written before a run and the
    description rebuilt in a later session compare equal; other tokens verbatim."""
    return [os.path.normpath(token) if os.path.isabs(token) else token for token in cmd]


# --- command binding (single validator: pre-launch in run_job and in verify_existing) -------------

# path-valued identity flags bound to the resolved receipt fields (exactly one occurrence, "--flag value" form only)
IDENTITY_PATH_FLAGS = {"--checkpoint": "module", "--config": "config", "--output-dir": "output_dir"}
# scalar identity flags bound to the receipt fields (exact string as emitted by job_command)
IDENTITY_SCALAR_FLAGS = {"--seed": "seed", "--action-selection": "action_selection", "--episode-start-offset-s": "start"}
# bare identity flags that must appear exactly once
IDENTITY_BARE_FLAGS = ("--no-auto-config",)


def _flag_value(command: list[str], flag: str) -> tuple[str | None, str | None]:
    """Value of ``flag`` when it occurs exactly once as ``flag value`` (value non-empty,
    not another flag); any ``flag=value`` token is ambiguous and rejected."""
    ambiguous = [tok for tok in command if tok.startswith(flag + "=")]
    if ambiguous:
        return None, f"ambiguous token(s) {ambiguous} (only '{flag} value' is accepted)"
    positions = [i for i, tok in enumerate(command) if tok == flag]
    if len(positions) != 1:
        return None, f"expected exactly one occurrence of {flag}, found {len(positions)}"
    i = positions[0]
    if i + 1 >= len(command) or not command[i + 1] or command[i + 1].startswith("--"):
        return None, f"{flag} without value"
    return command[i + 1], None


def _resolve_token(token: str, cwd: str) -> str:
    path = token if os.path.isabs(token) else os.path.join(cwd, token)
    return os.path.normpath(path)


ACTION_SELECTIONS = ("deterministic", "stochastic")
SPEC_IDENTITY_FIELDS = ("family", "candidate", "runtime", "start", "action_selection", "seed", "repeat")


def receipt_spec(receipt: dict[str, Any]) -> tuple[JobSpec | None, dict[str, Any]]:
    """Canonical spec of the receipt: looked up by job_id in the unique canonical
    index, then EVERY identity field of the receipt (family, candidate, runtime,
    start, action_selection, seed, repeat, historical reference) must equal the
    canonical spec exactly (type and value; no KeyError). Untrusted receipt
    fields never define the spec."""
    problems: dict[str, Any] = {}
    jid = receipt.get("job_id")
    if not isinstance(jid, str) or not jid:
        return None, {"job_id": {"existing": jid, "expected": "canonical job_id string"}}
    try:
        index = canonical_index()
    except RuntimeError as exc:
        return None, {"canonical_index": {"existing": str(exc), "expected": "unique job_id -> spec index"}}
    spec = index.get(jid)
    if spec is None:
        return None, {"job_id": {"existing": jid, "expected": "one of the canonical job ids of the matrix"}}
    family, cand, runtime, start, mode, seed, rep, hist = spec
    canonical = {"family": family, "candidate": cand, "runtime": runtime, "start": start, "action_selection": mode, "seed": seed, "repeat": rep}
    for field, expected in canonical.items():
        value = receipt.get(field)
        if type(value) is not type(expected) or value != expected:  # exact type + value (bool never equals int here)
            problems[field] = {"existing": value, "expected_canonical": expected}
    hist_rel = C.rel(hist) if hist is not None else None
    if receipt.get("historical_reference_summary") != hist_rel:
        problems["historical_reference_summary"] = {"existing": receipt.get("historical_reference_summary"), "expected_canonical": hist_rel}
    if problems:
        return None, problems
    return spec, {}


def _path_component_problems(raw: str, label: str) -> dict[str, Any] | None:
    """'.'/'..' components, backslashes or a symlink in ANY prefix of the raw path are rejected."""
    if "\\" in raw:
        return {"existing": raw, "expected": f"{label} without backslashes"}
    parts = Path(raw).parts
    if any(p in (".", "..") for p in parts):
        return {"existing": raw, "expected": f"{label} without '.'/'..' components"}
    probe = Path(raw) if os.path.isabs(raw) else C.REPO / raw
    prefixes = [probe, *probe.parents]
    for prefix in prefixes:
        if prefix.is_symlink():
            return {"existing": raw, "expected": f"{label} with no symlink in any component (symlink: {prefix})"}
    return None


def command_binding_problems(receipt: dict[str, Any], *, fingerprint: dict[str, Any] | None = None, require_fingerprint: bool = False) -> dict[str, Any]:
    """Bind the stored command to the CANONICAL identity; nothing is trusted unless re-derived.

    Rules (all fail-closed, returned as a mismatch mapping; empty == bound):
      spec         job_id must name a canonical job; every receipt identity field
                   equals the canonical spec (type and value);
      retry_tag    None or a validated single path component;
      output_dir   exactly C.rel(OUT_ROLLOUTS / family / (job_id | job_id__tag))
                   derived from the canonical spec + validated tag; relative,
                   POSIX, no backslash, no '.'/'..' component;
      argv         the WHOLE RAW argv equals, token per token (no normalisation),
                   job_command(canonical spec, canonical output dir, python);
      command      non-empty list of non-empty strings; normalize_command(command)
                   == command_normalized (descriptive integrity field only);
      command[0]   == python field; with a fingerprint (schema 5) realpath(command[0])
                   == realpath(fingerprint.requested_executable) == fingerprint.executable_realpath;
      command[1]   raw token exactly str(C.ROLLOUT_EVAL), no '.'/'..'/backslash/symlink
                   in any component, regular file, realpath == realpath(C.ROLLOUT_EVAL),
                   SHA-256 == rollout_eval_sha256;
      cwd          == repository root;
      explicit flag checks (--checkpoint/--config/--output-dir once with the canonical
                   values, --no-auto-config once, --seed/--action-selection/
                   --episode-start-offset-s once with the emitted string) kept as
                   defence in depth."""
    problems: dict[str, Any] = {}
    command = receipt.get("command")
    if not isinstance(command, list) or not command or not all(isinstance(tok, str) and tok for tok in command):
        problems["command"] = {"existing": command if not isinstance(command, list) else f"list[{len(command)}]", "expected": "non-empty list of non-empty strings"}
        return problems
    if normalize_command(command) != receipt.get("command_normalized"):
        problems["command_normalized"] = {"existing": receipt.get("command_normalized"), "recomputed": normalize_command(command)}
    python = receipt.get("python")
    if not isinstance(python, str) or not python or command[0] != python:
        problems["command[0]"] = {"existing": command[0], "expected_python_field": python}
    # --- canonical identity, retry tag and output directory --------------------------------------
    spec, spec_problems = receipt_spec(receipt)
    if spec_problems:
        problems["spec"] = spec_problems
    tag = receipt.get("retry_tag")
    try:
        tag = validate_retry_tag(tag)
    except RuntimeError as exc:
        problems["retry_tag"] = {"existing": receipt.get("retry_tag"), "expected": str(exc)}
        tag = None
    out_rel = receipt.get("output_dir")
    canonical_dir: Path | None = None
    if spec is not None and "retry_tag" not in problems:
        canonical_dir = output_dir_for(spec, tag)
        expected_rel = C.rel(canonical_dir)
        if not isinstance(out_rel, str) or out_rel != expected_rel or os.path.isabs(out_rel) or "\\" in out_rel or any(p in (".", "..") for p in out_rel.split("/") if p) or out_rel.startswith("/"):
            problems["output_dir"] = {"existing": out_rel, "expected_canonical": expected_rel}
            canonical_dir = None
    elif not isinstance(out_rel, str) or not out_rel:
        problems["output_dir"] = {"existing": out_rel, "expected": "canonical repository-relative POSIX path"}
    # --- RAW argv reconstruction from the canonical spec -------------------------------------------
    if spec is not None and canonical_dir is not None and isinstance(python, str) and python:
        expected = job_command(spec, canonical_dir, python)
        if expected != command:
            first = next((i for i, (a, b) in enumerate(zip(command, expected)) if a != b), min(len(command), len(expected)))
            problems["argv"] = {"first_difference_index": first, "existing_token": command[first] if first < len(command) else None, "expected_token": expected[first] if first < len(expected) else None, "existing_len": len(command), "expected_len": len(expected)}
    elif "spec" not in problems and "output_dir" not in problems and "retry_tag" not in problems:
        problems["argv"] = {"existing": "python field missing: argv cannot be rebuilt", "expected": "python executable path"}
    cwd = receipt.get("cwd")
    if not isinstance(cwd, str) or not cwd or os.path.normpath(cwd) != os.path.normpath(str(C.REPO)):
        problems["cwd"] = {"existing": cwd, "expected": str(C.REPO)}
        cwd = str(C.REPO)
    if len(command) < 2:
        problems["command[1]"] = {"existing": None, "expected": "rollout script path"}
    else:
        raw_script = command[1]
        component_problem = _path_component_problems(raw_script, "rollout script path")
        if component_problem:
            problems["command[1]_components"] = component_problem
        if raw_script != str(C.ROLLOUT_EVAL):
            problems["command[1]"] = {"existing": raw_script, "expected": str(C.ROLLOUT_EVAL)}
        script = Path(raw_script) if os.path.isabs(raw_script) else C.REPO / raw_script
        if script.is_symlink() or not script.is_file():
            problems["command[1]_file"] = {"existing": raw_script, "expected": "existing regular file (rollout script), no symlink"}
        else:
            if os.path.realpath(script) != os.path.realpath(C.ROLLOUT_EVAL):
                problems["command[1]_realpath"] = {"existing": os.path.realpath(script), "expected": os.path.realpath(C.ROLLOUT_EVAL)}
            sha = C.sha256_file(script)
            if sha != receipt.get("rollout_eval_sha256"):
                problems["command[1]_sha256"] = {"script": raw_script, "recomputed": sha, "receipt_rollout_eval_sha256": receipt.get("rollout_eval_sha256")}
    for flag, field in IDENTITY_PATH_FLAGS.items():
        value, error = _flag_value(command, flag)
        if error:
            problems[flag] = {"existing": error}
            continue
        expected_rel = receipt.get(field)
        if not isinstance(expected_rel, str) or not expected_rel:
            problems[flag] = {"existing": value, "expected": f"receipt field {field} missing"}
            continue
        expected = os.path.normpath(str(C.REPO / expected_rel.replace("\\", "/")))
        actual = _resolve_token(value, cwd)
        if actual != expected:
            problems[flag] = {"existing": actual, "expected_from_field": f"{field}={expected_rel}", "expected": expected}
    for flag in IDENTITY_BARE_FLAGS:
        count = command.count(flag)
        if count != 1 or any(tok.startswith(flag + "=") for tok in command):
            problems[flag] = {"existing": count, "expected": 1}
    for flag, field in IDENTITY_SCALAR_FLAGS.items():
        value, error = _flag_value(command, flag)
        if error:
            problems[flag] = {"existing": error}
            continue
        if field == "start":
            expected_value = repr(C.EXACT_STARTS[receipt["start"]]) if receipt.get("start") in C.EXACT_STARTS else None
        else:
            expected_value = None if receipt.get(field) is None else str(receipt.get(field))
        if expected_value is None or value != expected_value:
            problems[flag] = {"existing": value, "expected_from_field": f"{field}={receipt.get(field)!r}", "expected": expected_value}
    fp = fingerprint if fingerprint is not None else receipt.get("environment_fingerprint")
    if not isinstance(fp, dict):
        if require_fingerprint:
            problems["environment_fingerprint"] = {"existing": None, "expected": "interpreter fingerprint bound to command[0]"}
    else:
        requested = fp.get("requested_executable")
        probed = fp.get("executable_realpath")
        exe_real = os.path.realpath(command[0])
        if not isinstance(requested, str) or not isinstance(probed, str) or not (exe_real == os.path.realpath(requested) == probed):
            problems["environment_fingerprint_executable"] = {"command[0]_realpath": exe_real, "fingerprint_requested_realpath": os.path.realpath(requested) if isinstance(requested, str) else requested, "fingerprint_executable_realpath": probed}
        if fp.get("role") != "rollout_interpreter" and require_fingerprint:
            problems["environment_fingerprint_role"] = {"existing": fp.get("role"), "expected": "rollout_interpreter"}
    return problems


_log_lock = threading.Lock()


def append_log(path: Path, record: dict[str, Any]) -> None:
    with _log_lock:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, default=str) + "\n")


def output_digest(out_dir: Path) -> dict[str, Any]:
    """Digest and key metrics of the rollout outputs actually present on disk."""
    out: dict[str, Any] = {}
    summary_path = out_dir / SUMMARY_FILE
    trace_path = out_dir / TRACE_FILE
    out["summary_present"] = summary_path.is_file()
    out["trace_present"] = trace_path.is_file()
    out["summary_sha256"] = C.sha256_file(summary_path) if out["summary_present"] else None
    out["trace_sha256"] = C.sha256_file(trace_path) if out["trace_present"] else None
    out["summary_ok"] = False
    if out["summary_present"]:
        try:
            s = C.read_json(summary_path)
        except ValueError:
            s = {}
        if not isinstance(s, dict):
            s = {}
        out["summary_ok"] = s.get("ok") is True
        keys = ("ok", "steps", "episode_return", "end_reason", "phase_valid_hs_count", "phase_valid_to_count", "phase_valid_cycle_count", "invalid_event_count", "grf_penetration_max_m", "reserve_norm_max_nm", "action_abs_max", "action_clipped_steps", "n_actor", "action_selection", "action_seed", "episode_start_offset_s")
        out["summary_fields"] = {k: s.get(k) for k in keys}
        # summary-derived values never overwrite the DESCRIBED identity fields of the receipt (bound by the command)
        out.update({k: s.get(k) for k in keys if k not in SUMMARY_KEYS_NOT_COPIED})
    return out


def status_from(returncode: int, digest: dict[str, Any]) -> str:
    ok = returncode == 0 and digest.get("summary_present") is True and digest.get("summary_ok") is True and digest.get("trace_present") is True
    return "ok" if ok else "failed"


def historical_reference(hist: Path | None) -> dict[str, Any]:
    if hist is None:
        return {"historical_reference_summary": None, "historical_reference_exists": False, "historical_reference_summary_sha256": None, "historical_reference_recorded": None}
    exists = hist.is_file()
    recorded = None
    if exists:
        s = C.read_json(hist)
        recorded = {k: s.get(k) for k in ("steps", "episode_return", "end_reason", "phase_valid_cycle_count", "grf_penetration_max_m", "reserve_norm_max_nm", "action_seed")}
    return {
        "historical_reference_summary": C.rel(hist),
        "historical_reference_exists": exists,
        "historical_reference_summary_sha256": C.sha256_file(hist) if exists else None,
        "historical_reference_recorded": recorded,
        "historical_reference_note": "recorded with the code of its own date; the replay runs the same yaml/weights on current HEAD, so equality is not guaranteed and any divergence must be explained",
    }


def describe_job(spec: JobSpec, python_exe: str, retry_tag: str | None) -> dict[str, Any]:
    family, cand, runtime, start, mode, seed, rep, hist = spec
    jid = job_id(spec)
    retry_tag = validate_retry_tag(retry_tag)  # fail-closed before any filesystem access
    out_dir = output_dir_for(spec, retry_tag)
    module = C.CANDIDATES[cand]["module"]
    cmd = job_command(spec, out_dir, python_exe)
    record = {
        "schema_version": RECEIPT_SCHEMA_VERSION,
        "provenance_class": CL.PROVENANCE_CLASS_CONTEMPORANEOUS,
        "provenance_limitation": CL.PROVENANCE_LIMITATION,
        "job_id": jid,
        "family": family,
        "comparison_class": comparison_class(family, cand, runtime),
        "runtime_is_target_v3": runtime == "v3_canonical",
        "policy_native_to_runtime": C.CANDIDATES[cand]["native_runtime"] == runtime,
        "bit_exact_reproduction_claimed": False,
        "candidate": cand,
        "candidate_description": C.CANDIDATES[cand]["description"],
        "runtime": runtime,
        "runtime_description": C.RUNTIMES[runtime]["description"],
        "start": start,
        "episode_start_offset_s": C.EXACT_STARTS[start],
        "action_selection": mode,
        "seed": seed,
        "repeat": rep,
        "retry_tag": retry_tag,
    }
    record.update(historical_reference(hist))
    record.update(
        {
            "module": C.rel(module),
            "module_state_sha256": C.sha256_file(module / "module_state.pkl") if (module / "module_state.pkl").is_file() else None,
            "config": C.rel(C.RUNTIMES[runtime]["config"]),
            "config_sha256": C.sha256_file(C.RUNTIMES[runtime]["config"]),
            "rollout_eval_sha256": C.sha256_file(C.ROLLOUT_EVAL),
            "python": python_exe,
            "cwd": str(C.REPO),
            "command": cmd,
            "command_normalized": normalize_command(cmd),
            "output_dir": C.rel(out_dir),
            "output_dir_exists": out_dir.exists(),
            "git_head": C.git("rev-parse", "HEAD"),
        }
    )
    # per-job scientific inputs (config + referenced runtime files, every module file, rollout_eval, reference)
    record.update(CL.job_inputs_table(record))
    return record


def verification_branch(old: dict[str, Any]) -> str:
    """Label of the verification branch a stored receipt goes through (by schema version)."""
    schema = old.get("schema_version")
    if schema in LEGACY_RECEIPT_SCHEMA_VERSIONS:
        return BRANCH_LEGACY_V4
    if schema == RECEIPT_SCHEMA_VERSION:
        return BRANCH_CONTEMPORANEOUS_V5
    raise RuntimeError(f"unsupported receipt schema_version {schema!r} (legacy {LEGACY_RECEIPT_SCHEMA_VERSIONS}, current {RECEIPT_SCHEMA_VERSION})")


def verify_existing(receipt: dict[str, Any], out_dir: Path) -> dict[str, Any]:
    """Fail-closed verification of an existing output directory.

    Compares the stored receipt with the fresh job description on the
    provenance fields (including the historical reference summary digest,
    None being valid for jobs without reference), requires status ok AND
    returncode == 0, and recomputes the output digests from disk (summary and
    trace must exist, summary.ok must be True, SHA-256 must equal the
    receipt). Schema-4 receipts (legacy, Class B) stop here; schema-5 receipts
    must additionally carry a consistent contemporaneous closure
    (``f0_closure.verify_closure_fields``). Raises RuntimeError on any
    deviation; the stored receipt is never modified."""
    if out_dir.is_symlink():
        raise RuntimeError(f"output dir is a symlink (not allowed): {out_dir}")
    receipt_path = out_dir / RECEIPT_FILE
    if receipt_path.is_symlink():
        raise RuntimeError(f"{RECEIPT_FILE} is a symlink (not allowed): {receipt_path}")
    if not receipt_path.is_file():
        raise RuntimeError(f"output dir exists without {RECEIPT_FILE} (no silent skip): {out_dir}")
    try:
        old = C.read_json(receipt_path)
    except ValueError as exc:
        raise RuntimeError(f"unreadable {RECEIPT_FILE} in {out_dir}: {exc}") from exc
    if not isinstance(old, dict):
        raise RuntimeError(f"{RECEIPT_FILE} is not a mapping: {receipt_path}")
    branch = verification_branch(old)  # raises on unsupported schema
    mismatches: dict[str, Any] = {}
    if branch == BRANCH_CONTEMPORANEOUS_V5:
        missing = [f for f in CLOSURE_RECEIPT_FIELDS if f not in old]
        if missing:
            mismatches["closure_fields_missing"] = {"existing": missing, "expected": "present"}
        closure_problems = CL.verify_closure_fields(old)
        if closure_problems:
            mismatches["closure"] = closure_problems
        # per-job inputs re-hashed NOW must equal what the job actually consumed
        if old.get("job_inputs_digest_pre") != receipt.get("job_inputs_digest"):
            mismatches["job_inputs_digest"] = {"existing": old.get("job_inputs_digest_pre"), "expected_from_current_inputs": receipt.get("job_inputs_digest")}
    # command binding: stored command re-derived against python / fingerprint / script / module / config / output_dir
    binding = command_binding_problems(old, require_fingerprint=(branch == BRANCH_CONTEMPORANEOUS_V5))
    if binding:
        mismatches["command_binding"] = binding
    for field in RECEIPT_VERIFY_FIELDS:
        if field not in old or old.get(field) != receipt.get(field):
            mismatches[field] = {"existing": old.get(field, "<missing>"), "expected": receipt.get(field)}
    if old.get("status") != "ok":
        mismatches["status"] = {"existing": old.get("status"), "expected": "ok"}
    if old.get("returncode") != 0:
        mismatches["returncode"] = {"existing": old.get("returncode", "<missing>"), "expected": 0}
    on_disk = output_digest(out_dir)
    if not on_disk["summary_present"]:
        mismatches[SUMMARY_FILE] = {"existing": "missing", "expected": "present"}
    if not on_disk["trace_present"]:
        mismatches[TRACE_FILE] = {"existing": "missing", "expected": "present"}
    if on_disk["summary_present"] and on_disk["summary_ok"] is not True:
        mismatches["summary.ok"] = {"existing": on_disk.get("ok"), "expected": True}
    for field in OUTPUT_DIGEST_FIELDS:
        if old.get(field) is None or on_disk.get(field) is None or old.get(field) != on_disk.get(field):
            mismatches[field] = {"receipt": old.get(field), "recomputed_from_disk": on_disk.get(field)}
    if mismatches:
        raise RuntimeError(f"existing output dir fails receipt verification for {receipt['job_id']} [{branch}]: {json.dumps(mismatches, default=str)[:3000]}")
    return old


def preflight_all(described: list[dict[str, Any]]) -> dict[str, Any]:
    """Synchronous verification of every existing output of the whole matrix.

    Returns the preflight report (with the verification branch of every
    accepted receipt); raises RuntimeError listing ALL invalid outputs if at
    least one fails, so that no job can start."""
    verified: list[str] = []
    branches: dict[str, str] = {}
    errors: dict[str, str] = {}
    for rec in described:
        out_dir = C.REPO / rec["output_dir"]
        if not out_dir.exists() and not out_dir.is_symlink():
            continue
        try:
            old = verify_existing(rec, out_dir)
            verified.append(rec["job_id"])
            branches[rec["job_id"]] = verification_branch(old)
        except RuntimeError as exc:
            errors[rec["job_id"]] = str(exc)
    report = {
        "checked_existing": len(verified) + len(errors),
        "verified_identical": verified,
        "verification_branches": branches,
        "verified_by_branch": {b: sum(1 for v in branches.values() if v == b) for b in (BRANCH_LEGACY_V4, BRANCH_CONTEMPORANEOUS_V5)},
        "invalid": errors,
    }
    if errors:
        raise RuntimeError(f"preflight failed: {len(errors)} invalid existing output(s); no job started: {json.dumps(errors, default=str)[:4000]}")
    return report


def _closure_or_error(fn: Callable[[], dict[str, Any]]) -> tuple[dict[str, Any] | None, str | None]:
    """Any failure to establish the closure/inputs is recorded and fails the job closed (never crashes the driver)."""
    try:
        return fn(), None
    except Exception as exc:  # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"


def _abort_job(receipt: dict[str, Any], out_dir: Path, log_path: Path, reason: str, **fields: Any) -> dict[str, Any]:
    """Fail closed BEFORE launching: receipt written with status failed, nothing run."""
    receipt.update(fields)
    receipt.update({"source_closure_unchanged": False, "job_inputs_unchanged": False, "returncode": None, "status": "failed", "status_reason": reason + "; job not launched"})
    receipt.update(output_digest(out_dir))
    C.write_json(out_dir / RECEIPT_FILE, receipt)
    append_log(log_path, receipt)
    return receipt


def run_job(receipt: dict[str, Any], *, log_path: Path, closure: dict[str, Any] | None = None) -> dict[str, Any]:
    """Run one job with the contemporaneous closure protocol (schema 5).

    ``closure`` = {closure_manifest, closure_manifest_sha256,
    runtime_source_closure_digest, launch_snapshot, job_inputs (job_id ->
    digest bound at launch), snapshot_fn}. Immediately BEFORE the subprocess the
    runtime closure (every section, canonical deep equality with the launch
    snapshot) and the per-job scientific inputs (config + referenced runtime
    files, every module file, rollout_eval, reference; digest must equal the
    description AND the launch binding) are recomputed: any difference aborts
    the launch. Immediately AFTER the subprocess both are recomputed again;
    ``status`` is ``ok`` only if the rollout succeeded AND nothing changed.
    Existing directories go through ``verify_existing``."""
    # identity / retry tag / output path / RAW argv / script are validated BEFORE any filesystem access:
    # an invalid receipt never creates a directory, never writes, never launches
    early = command_binding_problems(receipt)
    if early:
        raise RuntimeError(f"refusing to run {receipt.get('job_id')!r}: identity/path/argv not bound to the canonical matrix: {json.dumps(early, default=str)[:1500]}")
    out_dir = output_dir_for(canonical_index()[receipt["job_id"]], receipt.get("retry_tag"))  # canonical, already equal to receipt output_dir
    if out_dir.is_symlink():
        raise RuntimeError(f"output dir is a symlink (not allowed): {out_dir}")
    if out_dir.exists():
        old = verify_existing(receipt, out_dir)
        receipt["status"] = "skipped_verified_identical"
        receipt["verification_branch"] = verification_branch(old)
        receipt["verified_against_receipt"] = {k: old.get(k) for k in ("started_at_utc", "finished_at_utc", "duration_s", "summary_sha256", "trace_sha256", "schema_version", "provenance_class")}
        receipt.update(output_digest(out_dir))
        append_log(log_path, receipt)
        return receipt
    if closure is None or not callable(closure.get("snapshot_fn")) or not isinstance(closure.get("launch_snapshot"), dict) or not isinstance(closure.get("job_inputs"), dict):
        raise RuntimeError("schema-5 execution requires a contemporaneous closure (launch manifest, launch snapshot, job input bindings, snapshot function); refusing to run without it")
    launch = closure["launch_snapshot"]
    out_dir.mkdir(parents=True, exist_ok=False)
    stdout_path = out_dir / "f0_driver_stdout.log"
    receipt["closure_manifest"] = closure["closure_manifest"]
    receipt["closure_manifest_sha256"] = closure["closure_manifest_sha256"]
    # --- pre: runtime closure ------------------------------------------------------------
    pre, pre_error = _closure_or_error(closure["snapshot_fn"])
    if pre is None:
        return _abort_job(receipt, out_dir, log_path, "pre-run source closure could not be established", closure_error=pre_error)
    for section in (*CL.SNAPSHOT_SECTIONS, "runtime_source_closure_digest", "orchestration_digest"):
        receipt[section] = pre[section]
    receipt["runtime_source_closure_digest_pre"] = pre["runtime_source_closure_digest"]
    equal_pre = CL.sections_equal(pre, launch)
    receipt["closure_sections_pre_equal_launch"] = equal_pre
    receipt["closure_matches_launch_manifest_pre"] = all(equal_pre.values()) and pre["runtime_source_closure_digest"] == closure["runtime_source_closure_digest"] == launch.get("runtime_source_closure_digest") and pre["orchestration_digest"] == launch.get("orchestration_digest")
    if not receipt["closure_matches_launch_manifest_pre"]:
        return _abort_job(receipt, out_dir, log_path, f"pre-run source closure differs from the launch manifest (sections equal: {equal_pre})", runtime_source_closure_digest_post=None)
    # --- pre: per-job scientific inputs ---------------------------------------------------
    inputs_pre, inputs_error = _closure_or_error(lambda: CL.job_inputs_table(receipt))
    if inputs_pre is None:
        return _abort_job(receipt, out_dir, log_path, "pre-run job inputs could not be resolved", closure_error=inputs_error, runtime_source_closure_digest_post=None, job_inputs_digest_pre=None, job_inputs_digest_post=None)
    receipt["job_inputs"] = inputs_pre["job_inputs"]
    receipt["job_inputs_digest_pre"] = inputs_pre["job_inputs_digest"]
    bound = closure["job_inputs"].get(receipt["job_id"])
    if not (inputs_pre["job_inputs_digest"] == receipt.get("job_inputs_digest") == bound):
        return _abort_job(receipt, out_dir, log_path, f"pre-run job inputs differ from the description/launch binding (now {inputs_pre['job_inputs_digest'][:12]}, described {str(receipt.get('job_inputs_digest'))[:12]}, bound {str(bound)[:12]})", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    # --- pre: command binding (command <-> python / rollout interpreter fingerprint / script / module / config / output_dir)
    binding = command_binding_problems(receipt, fingerprint=pre["environment_fingerprint"], require_fingerprint=True)
    receipt["command_binding_pre_launch"] = {"ok": not binding, "problems": binding}
    if binding:
        return _abort_job(receipt, out_dir, log_path, f"command binding failed: {json.dumps(binding, default=str)[:1500]}", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    # --- pre: the closure manifest ON DISK (regular file, exact SHA, content/schema, sections and digests == launch,
    #          binding of THIS job) - last check before the subprocess
    manifest_pre = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_pre_launch"] = {"ok": not manifest_pre, "problems": manifest_pre}
    if manifest_pre:
        return _abort_job(receipt, out_dir, log_path, f"closure manifest on disk missing or tampered before launch: {json.dumps(manifest_pre, default=str)[:1500]}", runtime_source_closure_digest_post=None, job_inputs_digest_post=None, closure_manifest_unchanged=False)
    # --- subprocess ---------------------------------------------------------------------------
    receipt["started_at_utc"] = C.utc_now()
    t0 = time.time()
    with stdout_path.open("w", encoding="utf-8") as handle:
        proc = subprocess.run(receipt["command"], cwd=str(C.REPO), stdout=handle, stderr=subprocess.STDOUT, text=True, check=False)
    receipt["finished_at_utc"] = C.utc_now()
    receipt["duration_s"] = round(time.time() - t0, 1)
    receipt["returncode"] = proc.returncode
    # --- post: runtime closure and job inputs ----------------------------------------------
    post, post_error = _closure_or_error(closure["snapshot_fn"])
    receipt["runtime_source_closure_digest_post"] = post["runtime_source_closure_digest"] if post else None
    equal_post = CL.sections_equal(post, pre) if post else {s: False for s in CL.SNAPSHOT_SECTIONS}
    receipt["closure_sections_post_equal_pre"] = equal_post
    receipt["source_closure_unchanged"] = bool(post) and all(equal_post.values()) and post["runtime_source_closure_digest"] == pre["runtime_source_closure_digest"] and post["orchestration_digest"] == pre["orchestration_digest"]
    inputs_post, inputs_post_error = _closure_or_error(lambda: CL.job_inputs_table(receipt))
    receipt["job_inputs_digest_post"] = inputs_post["job_inputs_digest"] if inputs_post else None
    receipt["job_inputs_unchanged"] = bool(inputs_post) and inputs_post["job_inputs_digest"] == receipt["job_inputs_digest_pre"]
    receipt["closure_error"] = post_error or inputs_post_error
    # --- post: the closure manifest ON DISK again (missing/tampered during the job => failed even with rc 0)
    manifest_post = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_post"] = {"ok": not manifest_post, "problems": manifest_post}
    receipt["closure_manifest_unchanged"] = not manifest_post
    digest = output_digest(out_dir)
    receipt.update(digest)
    rollout_status = status_from(proc.returncode, digest)
    receipt["status"] = rollout_status if (receipt["source_closure_unchanged"] and receipt["job_inputs_unchanged"] and receipt["closure_manifest_unchanged"]) else "failed"
    if receipt["status"] == "ok":
        receipt["status_reason"] = "ok"
    elif rollout_status != "ok":
        receipt["status_reason"] = "rollout failed"
    elif not receipt["source_closure_unchanged"]:
        receipt["status_reason"] = f"source closure changed or missing during the job ({post_error or 'section drift: ' + str({k: v for k, v in equal_post.items() if not v})})"
    elif not receipt["job_inputs_unchanged"]:
        receipt["status_reason"] = f"job inputs changed or missing during the job ({inputs_post_error or 'digest drift'})"
    else:
        receipt["status_reason"] = f"closure manifest on disk missing or tampered during the job ({json.dumps(manifest_post, default=str)[:800]})"
    try:
        receipt["stdout_tail"] = stdout_path.read_text(encoding="utf-8", errors="replace").splitlines()[-15:]
    except OSError:
        receipt["stdout_tail"] = []
    C.write_json(out_dir / RECEIPT_FILE, receipt)
    append_log(log_path, receipt)
    return receipt


def execute_matrix(all_described: list[dict[str, Any]], selected: list[dict[str, Any]], *, workers: int, log_path: Path, runner: Callable[..., dict[str, Any]] = run_job, closure: dict[str, Any] | None = None) -> dict[str, Any]:
    """Preflight (synchronous, whole matrix) then run the selected jobs in a pool.

    The pool is created only after the preflight succeeded; a preflight error
    propagates before any job is launched. ``closure`` is handed to the runner
    (contemporaneous closure protocol of schema-5 receipts)."""
    preflight = preflight_all(all_described)
    results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(runner, rec, log_path=log_path, closure=closure) for rec in selected]
        for fut in as_completed(futures):
            rec = fut.result()  # a verification error propagates and aborts the driver (fail-closed)
            results.append(rec)
            print(f"[matrix] {len(results)}/{len(selected)} {rec['job_id']} status={rec['status']} steps={rec.get('steps')} return={rec.get('episode_return')} end={rec.get('end_reason')} dur={rec.get('duration_s')}s", flush=True)
    return {"preflight": preflight, "results": results}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--family", action="append", default=None, help="|".join(FAMILIES) + " (repeatable)")
    parser.add_argument("--only", default=None, help="regex on job id")
    parser.add_argument("--retry-tag", default=None, help="explicit suffix for fresh output dirs when an existing dir fails verification")
    parser.add_argument("--list", action="store_true", help="print the job list and exit (no manifest)")
    parser.add_argument("--execute", action="store_true", help="actually run the rollouts (default: dry-run, manifest only)")
    args = parser.parse_args()
    validate_retry_tag(args.retry_tag)  # fail-closed BEFORE ensure_out_dirs or any other filesystem access
    canonical_index()  # fail-closed on job_id collisions

    C.ensure_out_dirs()
    all_jobs = build_jobs()
    jobs = list(all_jobs)
    if args.family:
        jobs = [j for j in jobs if j[0] in set(args.family)]
    if args.only:
        rx = re.compile(args.only)
        jobs = [j for j in jobs if rx.search(job_id(j))]
    if args.list:
        for j in jobs:
            print(job_id(j), "|", comparison_class(j[0], j[1], j[2]), "| ref:", C.rel(j[7]) if j[7] else "-")
        print(f"{len(jobs)} jobs")
        return 0
    mode = "execute" if args.execute else "dry_run"
    try:
        interpreter = C.select_python()
        python_exe = interpreter["selected"]
    except RuntimeError as exc:
        if args.execute:
            raise
        interpreter = {"selected": None, "error": str(exc)}
        python_exe = PYTHON_PLACEHOLDER
    stamp = time.strftime("%Y%m%d_%H%M%S")
    manifest_path = C.OUT_ROLLOUTS / f"rollout_matrix_manifest_{mode}_{stamp}.json"
    log_path = C.OUT_ROLLOUTS / "driver_log.jsonl"
    all_described = [describe_job(j, python_exe, args.retry_tag) for j in all_jobs]
    selected_ids = {job_id(j) for j in jobs}
    described = [d for d in all_described if d["job_id"] in selected_ids]
    july_or_39d = {cid for cid, spec in C.CANDIDATES.items() if spec["lineage"] == "july" or cid == "V26_39D"}
    # contemporaneous source closure: full manifest written at launch (execute) or digests only (dry-run)
    closure: dict[str, Any] | None = None
    closure_record: dict[str, Any]
    try:
        # the exact rollout interpreter is probed (subprocess, argv list, no shell); in dry-run the current one
        snapshot = CL.closure_snapshot(python_exe if args.execute else None)
        closure_record = {**CL.compact(snapshot), "git_head_at_snapshot": snapshot["git"]["head"], "error": None}
        if args.execute:
            closure_manifest_path = C.OUT_ROLLOUTS / f"source_closure_manifest_{stamp}.json"
            bindings = {d["job_id"]: d["job_inputs_digest"] for d in described}
            written = CL.write_closure_manifest(closure_manifest_path, snapshot, job_inputs=bindings)
            closure = {**written, "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"], "launch_snapshot": snapshot, "job_inputs": bindings, "snapshot_fn": lambda: CL.closure_snapshot(python_exe)}
            closure_record.update(written)
            closure_record["job_inputs_bound"] = len(bindings)
    except (CL.ClosureError, OSError) as exc:
        if args.execute:
            raise
        closure_record = {"error": f"{type(exc).__name__}: {exc}"}
    manifest = {
        "schema_version": RECEIPT_SCHEMA_VERSION,
        "receipt_schema_version_new_jobs": RECEIPT_SCHEMA_VERSION,
        "legacy_receipt_schema_versions": list(LEGACY_RECEIPT_SCHEMA_VERSIONS),
        "provenance_class_new_jobs": CL.PROVENANCE_CLASS_CONTEMPORANEOUS,
        "provenance_class_legacy_jobs": CL.PROVENANCE_CLASS_LEGACY,
        "provenance_limitation": CL.PROVENANCE_LIMITATION,
        "closure": closure_record,
        "revision": C.F0_REV,
        "superseded_revisions": C.SUPERSEDED_REVISIONS,
        "mode": mode,
        "dry_run": not args.execute,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "environment": C.env_snapshot(),
        "interpreter": interpreter,
        "workers": args.workers,
        "filters": {"family": args.family, "only": args.only, "retry_tag": args.retry_tag},
        "matrix_explanation": MATRIX_EXPLANATION,
        "receipt_verify_fields": list(RECEIPT_VERIFY_FIELDS),
        "closure_receipt_fields": list(CLOSURE_RECEIPT_FIELDS),
        "output_digest_fields": list(OUTPUT_DIGEST_FIELDS),
        "job_count_total_matrix": len(all_described),
        "job_count": len(described),
        "job_count_by_family": {fam: sum(1 for d in described if d["family"] == fam) for fam in FAMILIES},
        "job_count_by_comparison_class": {cls: sum(1 for d in described if d["comparison_class"] == cls) for cls in CLASSES},
        "historical_controls_declared_non_isometric": all(d["comparison_class"] != "isometric_comparable" for d in all_described if d["candidate"] in july_or_39d),
        "bit_exact_reproduction_claimed_anywhere": any(d["bit_exact_reproduction_claimed"] for d in all_described),
        "historical_references": [{"job_id": d["job_id"], "summary": d["historical_reference_summary"], "sha256": d["historical_reference_summary_sha256"], "recorded": d["historical_reference_recorded"]} for d in all_described if d["historical_reference_summary"]],
        "existing_output_dirs": [d["output_dir"] for d in all_described if d["output_dir_exists"]],
        "jobs": described,
    }
    C.write_json(manifest_path, manifest)
    print(f"[matrix] mode={mode} manifest={manifest_path} jobs={len(described)}/{len(all_described)} by_family={manifest['job_count_by_family']} by_class={manifest['job_count_by_comparison_class']} controls_non_isometric={manifest['historical_controls_declared_non_isometric']} interpreter={python_exe} existing_dirs={len(manifest['existing_output_dirs'])}", flush=True)
    if not args.execute:
        print("[matrix] dry-run only: no rollout started (pass --execute after architectural review)", flush=True)
        return 0
    t0 = time.time()
    outcome = execute_matrix(all_described, described, workers=args.workers, log_path=log_path, closure=closure)
    results = outcome["results"]
    status = {
        "generated_at_utc": C.utc_now(),
        "manifest": C.rel(manifest_path),
        "closure_manifest": closure["closure_manifest"] if closure else None,
        "closure_manifest_sha256": closure["closure_manifest_sha256"] if closure else None,
        "mode": mode,
        "preflight": outcome["preflight"],
        "elapsed_s": round(time.time() - t0, 1),
        "job_count": len(described),
        "ok": sum(1 for r in results if r["status"] == "ok"),
        "skipped_verified_identical": sum(1 for r in results if r["status"] == "skipped_verified_identical"),
        "failed": sum(1 for r in results if r["status"] == "failed"),
        "results": sorted(results, key=lambda r: r["job_id"]),
    }
    C.write_json(C.OUT_ROLLOUTS / f"rollout_matrix_status_{stamp}.json", status)
    print(f"[matrix] done ok={status['ok']} skipped={status['skipped_verified_identical']} failed={status['failed']} elapsed={status['elapsed_s']}s", flush=True)
    return 0 if status["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
