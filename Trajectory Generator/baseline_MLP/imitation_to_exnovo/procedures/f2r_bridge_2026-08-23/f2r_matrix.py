"""F2R job-matrix driver (rev 3): job specs per phase/round, exact commands,
receipts with the contemporaneous F0 closure, fail-closed execution, and the
S0 **dry-run** manifest (default; no process is launched).

Phases and rollouts (protocol ``variants`` / ``sequence_rev3``):

* T1 commissioning, round 1: 3 det (seed 123) — stop always;
* T2 primary, rounds 1..4: 3 det (123) + 6 stoch (123/124, collection)
  + 3 stoch (125, **held-out promotion gate V**, never in a dataset);
* T3 contingency, rounds 1..2: same shape as T2;
* R: evaluation over the 9 stochastic jobs of the promoted round (no new
  rollouts; the 3 seed-125 jobs are those of V).

Every student rollout uses the unchanged production ``rollout_eval.py`` with
the student module of that round (pending until the refit of that round
exists).  Receipt schema ``f2r.1`` keeps all F0 schema-5 closure fields so
``f0_closure.verify_closure_fields`` verifies F2R receipts unchanged.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import f2r_common as R  # noqa: E402
import f0_common as C  # noqa: E402
import f0_closure as CL  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402  (registry-agnostic helpers only)
import f1_common as F1  # noqa: E402

RECEIPT_FILE = "f2r_receipt.json"
RECEIPT_SCHEMA = "f2r.1"
STDOUT_LOG = "f2r_driver_stdout.log"
RECEIPT_VERIFY_FIELDS = ("job_id", "command_normalized", "module_state_sha256", "config_sha256", "rollout_eval_sha256", "git_head", "output_dir", "phase", "round", "purpose")
F2R_SCRIPTS = ("f2r_common.py", "f2r_labeller.py", "f2r_observability.py", "f2r_dagger.py", "f2r_refit.py", "f2r_gates.py", "f2r_matrix.py", "f2r_protocol.json", "PROTOCOL_F2R.md")
ROLLOUT_AFFECTING_SCRIPTS = ("f2r_common.py", "f2r_matrix.py", "f2r_protocol.json")
F0_DRIVER_LIBS = ("f0_common.py", "f0_closure.py", "f0_rollout_matrix.py")
F1_LIBS = ("f1_common.py",)
PURPOSES = ("det", "stoch", "validation_gate")


class MatrixError(RuntimeError):
    pass


# --- job specs ------------------------------------------------------------------------


def phase_rounds() -> dict[str, int]:
    variants = R.load_protocol()["variants"]
    out = {"T1": int(variants["T1"]["rounds"]), "T2": int(variants["T2"]["max_rounds"]), "T3": int(variants["T3"]["max_rounds"])}
    if "T1R" in variants:
        out["T1R"] = int(variants["T1R"]["rounds"])
    return out


def student_module_dir(phase: str, rnd: int) -> Path:
    return R.OUT_REFIT / phase / f"round_{rnd}" / "rl_module_student"


def job_id(spec: dict[str, Any]) -> str:
    mode_tag = "det" if spec["action_selection"] == "deterministic" else f"stoch_seed{spec['seed']}"
    return f"{spec['candidate']}__{spec['runtime']}__{spec['start']}__{mode_tag}"


def output_dir_for(spec: dict[str, Any]) -> Path:
    return R.OUT_ROLLOUTS / spec["phase"] / f"round_{spec['round']}" / spec["job_id"]


def _spec(phase: str, rnd: int, purpose: str, start: str, seed: int) -> dict[str, Any]:
    if purpose not in PURPOSES:
        raise MatrixError(f"unknown purpose {purpose}")
    if purpose == "validation_gate":
        R.assert_rollout_seed(seed, purpose="validation")
    else:
        R.assert_rollout_seed(seed, purpose="collection")
    if purpose == "det" and seed != R.DET_SEED:
        raise MatrixError("deterministic rollouts use seed 123")
    if phase in ("T1", "T1R") and purpose != "det":
        raise MatrixError(f"commissioning phase {phase} has deterministic seed-123 jobs only (no stochastic, no validation seed)")
    spec = {
        "phase": phase, "round": int(rnd), "purpose": purpose, "candidate": f"STUDENT_{phase}_r{rnd}", "runtime": F1.TARGET_RUNTIME,
        "start": start, "episode_start_offset_s": R.EXACT_STARTS[start], "action_selection": "deterministic" if purpose == "det" else "stochastic", "seed": int(seed),
        "enters_dataset": purpose in ("det", "stoch"), "gate_role": {"det": "A/B/C measurement + aggregation", "stoch": "R measurement + aggregation", "validation_gate": "V held-out promotion gate only (never in any dataset/label/fit)"}[purpose],
        "driver": "rollout_eval", "stage": f"{phase}_round_{rnd}",
    }
    if phase == "T1R":  # corrective commissioning: gate-only rollouts (never aggregated, never labelled)
        spec["enters_dataset"] = False
        spec["generates_labels"] = False
        spec["gate_role"] = "commissioning gate measurement A/B/C only; no aggregation"
    spec["job_id"] = job_id(spec)
    return spec


def build_jobs() -> list[dict[str, Any]]:
    rounds = phase_rounds()
    jobs: list[dict[str, Any]] = []
    for s in R.STARTS:  # T1 commissioning: det only
        jobs.append(_spec("T1", 1, "det", s, R.DET_SEED))
    for phase in ("T2", "T3"):
        for rnd in range(1, rounds[phase] + 1):
            for s in R.STARTS:
                jobs.append(_spec(phase, rnd, "det", s, R.DET_SEED))
            for s in R.STARTS:
                for k in R.COLLECTION_SEEDS:
                    jobs.append(_spec(phase, rnd, "stoch", s, k))
            for s in R.STARTS:
                jobs.append(_spec(phase, rnd, "validation_gate", s, R.VALIDATION_SEED))
    if "T1R" in rounds:  # corrective commissioning: exactly 3 det seed-123 jobs, appended AFTER the 75 legacy jobs (unchanged)
        for s in R.STARTS:
            jobs.append(_spec("T1R", 1, "det", s, R.DET_SEED))
    ids = [j["job_id"] + f"@{j['phase']}r{j['round']}" for j in jobs]
    if len(set(ids)) != len(ids):
        raise MatrixError("duplicate job ids")
    for j in jobs:
        if j["seed"] in R.SEALED_SEEDS:
            raise MatrixError("sealed seed in matrix")
        if j["purpose"] == "validation_gate" and j["enters_dataset"]:
            raise MatrixError("validation gate job cannot enter a dataset")
    return jobs


def canonical_index() -> dict[str, dict[str, Any]]:
    return {f"{j['phase']}/round_{j['round']}/{j['job_id']}": j for j in build_jobs()}


def spec_key(spec: dict[str, Any]) -> str:
    return f"{spec['phase']}/round_{spec['round']}/{spec['job_id']}"


def job_command(spec: dict[str, Any], out_dir: Path, python_exe: str) -> list[str]:
    module = student_module_dir(spec["phase"], spec["round"])
    return [
        python_exe, str(R.ROLLOUT_EVAL), "--checkpoint", str(module), "--no-auto-config", "--config", str(R.RUNTIME_CONFIG),
        "--episode-start-offset-s", repr(float(R.EXACT_STARTS[spec["start"]])), "--action-selection", spec["action_selection"], "--seed", str(int(spec["seed"])),
        "--output-dir", str(out_dir), "--record-outputs", "--record-policy-trace", *F1.JOB_TIMEOUT_ARGS, *list(C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]),
    ]


def driver_inputs_table() -> dict[str, Any]:
    paths = [HERE / p for p in ROLLOUT_AFFECTING_SCRIPTS] + [R.F0_DIR / p for p in F0_DRIVER_LIBS] + [R.F1_DIR / p for p in F1_LIBS]
    table = CL.digest_table(paths, label="f2r driver inputs")
    return {"f2r_driver_inputs": table, "f2r_driver_inputs_digest": CL.table_digest(table), "f2r_driver_inputs_count": len(table)}


def describe_job(spec: dict[str, Any], python_exe: str | None) -> dict[str, Any]:
    out_dir = output_dir_for(spec)
    module = student_module_dir(spec["phase"], spec["round"])
    pending = not (module / "module_state.pkl").is_file()
    python = python_exe or M.PYTHON_PLACEHOLDER
    command = job_command(spec, out_dir, python)
    rec: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA, "provenance_class": CL.PROVENANCE_CLASS_CONTEMPORANEOUS, "provenance_limitation": CL.PROVENANCE_LIMITATION,
        "protocol_id": R.PROTOCOL_ID, "protocol_sha256": R.protocol_digests()["f2r_protocol.json"],
        **{k: spec[k] for k in ("job_id", "phase", "round", "purpose", "candidate", "runtime", "start", "episode_start_offset_s", "action_selection", "seed", "enters_dataset", "gate_role", "driver", "stage")},
        "module": C.rel(module), "module_pending": pending, "module_state_sha256": C.sha256_file(module / "module_state.pkl") if not pending else None,
        "init_primary": {"name": R.INIT_PRIMARY["name"], "actor_digest": R.INIT_PRIMARY["actor_digest"]},
        "config": C.rel(R.RUNTIME_CONFIG), "config_sha256": C.sha256_file(R.RUNTIME_CONFIG), "rollout_eval_sha256": C.sha256_file(R.ROLLOUT_EVAL),
        "python": python, "cwd": str(C.REPO), "command": command, "command_normalized": M.normalize_command(command), "output_dir": C.rel(out_dir), "output_dir_exists": out_dir.exists(), "git_head": C.git("rev-parse", "HEAD"),
    }
    if rec["config_sha256"] != R.RUNTIME_CONFIG_SHA256:
        raise MatrixError("pinned runtime config digest changed")
    rec.update(driver_inputs_table())
    if not pending:
        rec.update(CL.job_inputs_table(rec))
    else:
        rec.update({"job_inputs": None, "job_inputs_digest": None, "job_inputs_count": 0, "module_pending_reason": f"student module of {spec['phase']} round {spec['round']} not refit yet"})
    return rec


# --- binding / verification ---------------------------------------------------------------


def command_binding_problems(receipt: dict[str, Any], *, fingerprint: dict[str, Any] | None = None, require_fingerprint: bool = False) -> dict[str, Any]:
    problems: dict[str, Any] = {}
    key = f"{receipt.get('phase')}/round_{receipt.get('round')}/{receipt.get('job_id')}"
    spec = canonical_index().get(key)
    if spec is None:
        return {"job_id": {"existing": key, "expected": "F2R canonical job"}}
    for k in ("phase", "round", "purpose", "candidate", "runtime", "start", "action_selection", "seed", "enters_dataset"):
        if receipt.get(k) != spec[k] or type(receipt.get(k)) is not type(spec[k]):
            problems[k] = {"existing": receipt.get(k), "expected": spec[k]}
    cmd = receipt.get("command")
    if not isinstance(cmd, list) or not cmd or not all(isinstance(t, str) and t for t in cmd):
        return {**problems, "command": {"existing": cmd, "expected": "non-empty list of non-empty strings"}}
    if M.normalize_command(cmd) != receipt.get("command_normalized"):
        problems["command_normalized"] = {"existing": receipt.get("command_normalized"), "expected": M.normalize_command(cmd)}
    if cmd[0] != receipt.get("python"):
        problems["command[0]"] = {"existing": cmd[0], "expected": receipt.get("python")}
    expected_cmd = job_command(spec, output_dir_for(spec), str(receipt.get("python")))
    if cmd != expected_cmd:
        problems["command_argv"] = {"existing": cmd, "expected": expected_cmd}
    if receipt.get("output_dir") != C.rel(output_dir_for(spec)):
        problems["output_dir"] = {"existing": receipt.get("output_dir"), "expected": C.rel(output_dir_for(spec))}
    if receipt.get("cwd") != str(C.REPO):
        problems["cwd"] = {"existing": receipt.get("cwd"), "expected": str(C.REPO)}
    script = Path(cmd[1])
    if not script.is_file() or script.is_symlink() or os.path.realpath(script) != os.path.realpath(R.ROLLOUT_EVAL):
        problems["command[1]"] = {"existing": cmd[1], "expected": str(R.ROLLOUT_EVAL)}
    elif C.sha256_file(script) != receipt.get("rollout_eval_sha256"):
        problems["rollout_eval_sha256"] = {"existing": receipt.get("rollout_eval_sha256"), "expected": C.sha256_file(script)}
    if cmd.count("--no-auto-config") != 1:
        problems["--no-auto-config"] = "must appear exactly once"
    seed = int(receipt.get("seed", -1))
    if receipt.get("purpose") == "validation_gate" and seed != R.VALIDATION_SEED:
        problems["validation_seed"] = {"existing": seed, "expected": R.VALIDATION_SEED}
    if receipt.get("purpose") in ("det", "stoch") and seed not in R.COLLECTION_SEEDS:
        problems["collection_seed"] = {"existing": seed, "expected": list(R.COLLECTION_SEEDS)}
    if seed in R.SEALED_SEEDS:
        problems["sealed_seed"] = seed
    if require_fingerprint or fingerprint is not None:
        fp = fingerprint or {}
        if fp.get("role") != "rollout_interpreter":
            problems["fingerprint_role"] = {"existing": fp.get("role"), "expected": "rollout_interpreter"}
        else:
            try:
                ok = os.path.realpath(cmd[0]) == os.path.realpath(str(fp.get("requested_executable"))) == fp.get("executable_realpath")
            except TypeError:
                ok = False
            if not ok:
                problems["fingerprint_executable"] = {"command": cmd[0], "requested": fp.get("requested_executable"), "probed": fp.get("executable_realpath")}
    return problems


REVISION_FIELDS = ("f2r_driver_inputs_digest", "git_head")  # current-revision equality: required only for relaunch candidates (selected jobs)


def verify_existing(receipt: dict[str, Any], out_dir: Path, *, relaunch_candidate: bool = True) -> dict[str, Any]:
    """Fail-closed verification of an existing job output against its fresh description.

    Always (archival integrity / self-consistency, independent of the current tooling revision):
    no symlink, receipt ``f2r.1``, F0 closure fields valid, ``job_inputs_digest_pre`` == fresh job
    inputs (module/config/rollout_eval pins), command binding with the recorded fingerprint,
    RECEIPT_VERIFY_FIELDS except ``git_head``, status ok / rc 0, summary/trace SHA == disk.
    Only for ``relaunch_candidate=True`` (a selected job that may be skipped as verified-identical):
    additionally ``f2r_driver_inputs_digest`` and ``git_head`` must equal the current ones
    (amendment "preflight scoping", architect decision 2026-08-23)."""
    if out_dir.is_symlink() or (out_dir / RECEIPT_FILE).is_symlink():
        raise MatrixError(f"symlink in job output: {out_dir}")
    path = out_dir / RECEIPT_FILE
    if not path.is_file():
        raise MatrixError(f"existing output dir without {RECEIPT_FILE}: {out_dir}")
    old = C.read_json(path)
    if not isinstance(old, dict) or old.get("schema_version") != RECEIPT_SCHEMA:
        raise MatrixError(f"unexpected receipt schema in {out_dir}")
    mism: dict[str, Any] = {}
    closure = CL.verify_closure_fields(old)
    if closure:
        mism["closure"] = closure
    if old.get("job_inputs_digest_pre") != receipt.get("job_inputs_digest"):
        mism["job_inputs_digest"] = {"stored_pre": old.get("job_inputs_digest_pre"), "fresh": receipt.get("job_inputs_digest")}
    binding = command_binding_problems(old, fingerprint=old.get("environment_fingerprint"), require_fingerprint=True)
    if binding:
        mism["command_binding"] = binding
    for k in RECEIPT_VERIFY_FIELDS:
        if k in REVISION_FIELDS and not relaunch_candidate:
            continue
        if old.get(k) != receipt.get(k):
            mism[k] = {"stored": old.get(k), "fresh": receipt.get(k)}
    if relaunch_candidate and old.get("f2r_driver_inputs_digest") != receipt.get("f2r_driver_inputs_digest"):
        mism["f2r_driver_inputs_digest"] = {"stored": old.get("f2r_driver_inputs_digest"), "fresh": receipt.get("f2r_driver_inputs_digest")}
    if old.get("status") != "ok" or old.get("returncode") != 0:
        mism["status"] = {"status": old.get("status"), "returncode": old.get("returncode")}
    digest = M.output_digest(out_dir)
    for k in ("summary_sha256", "trace_sha256"):
        if digest.get(k) != old.get(k) or digest.get(k) is None:
            mism[k] = {"stored": old.get(k), "disk": digest.get(k)}
    if not (digest["summary_present"] and digest["summary_ok"] and digest["trace_present"]):
        mism["outputs"] = digest
    if mism:
        raise MatrixError(f"existing output {out_dir.name} does not match: {json.dumps(mism, default=str)[:2000]}")
    return old


def preflight_all(described: list[dict[str, Any]], *, selected_ids: set[str] | None = None) -> dict[str, Any]:
    """Global preflight over EVERY existing output of the described jobs (fail-closed on any
    archival-integrity problem), with the current-revision equality (``f2r_driver_inputs_digest``,
    ``git_head``) required only for the selected jobs (``selected_ids``; ``None`` = every job is a
    relaunch candidate = the strict legacy behaviour).  For non-selected outputs a revision mismatch
    is NOT an error: it is reported explicitly in ``tooling_revision_differs`` (receipts and outputs
    are never rewritten)."""
    checked, invalid, ok, integrity_ok, differs = 0, [], [], [], []
    for rec in described:
        out_dir = C.REPO / rec["output_dir"]
        if not out_dir.exists():
            continue
        checked += 1
        candidate = selected_ids is None or rec["job_id"] in selected_ids
        try:
            old = verify_existing(rec, out_dir, relaunch_candidate=candidate)
        except Exception as exc:  # noqa: BLE001
            invalid.append({"job_id": rec["job_id"], "selected": candidate, "error": str(exc)[:600]})
            continue
        if candidate:
            ok.append(rec["job_id"])
        else:
            integrity_ok.append(rec["job_id"])
            delta = {k: {"stored": old.get(k), "fresh": rec.get(k)} for k in REVISION_FIELDS if old.get(k) != rec.get(k)}
            if delta:
                differs.append({"job_id": rec["job_id"], "phase": rec.get("phase"), "round": rec.get("round"), "output_dir": rec["output_dir"], "integrity": "verified", "note": "produced under a previous tooling revision / git head; not a relaunch candidate; receipt untouched", **delta})
    if invalid:
        raise MatrixError(f"preflight found invalid existing outputs: {json.dumps(invalid)[:2000]}")
    return {"checked_existing": checked, "selected_ids": sorted(selected_ids) if selected_ids is not None else None, "verified_identical": ok, "verified_integrity_only": integrity_ok, "tooling_revision_differs": differs, "invalid": invalid}


# --- execution (S1+ only; mirrors the F1 driver) ---------------------------------------------


def _abort(receipt: dict[str, Any], out_dir: Path, log_path: Path, reason: str, **fields: Any) -> dict[str, Any]:
    receipt.update(fields); receipt.update({"returncode": None, "status": "failed", "status_reason": reason + "; job not launched"})
    receipt.update(M.output_digest(out_dir)); C.write_json(out_dir / RECEIPT_FILE, receipt); M.append_log(log_path, receipt)
    return receipt


def run_job(receipt: dict[str, Any], *, log_path: Path, closure: dict[str, Any] | None = None, subprocess_run: Callable[..., Any] = subprocess.run) -> dict[str, Any]:
    early = command_binding_problems(receipt)
    if early:
        raise MatrixError(f"refusing to run {receipt.get('job_id')!r}: {json.dumps(early, default=str)[:1500]}")
    if receipt.get("module_pending"):
        raise MatrixError(f"refusing to run {receipt['job_id']}: student module pending ({receipt.get('module_pending_reason')})")
    spec = canonical_index()[f"{receipt['phase']}/round_{receipt['round']}/{receipt['job_id']}"]
    out_dir = output_dir_for(spec)
    if out_dir.is_symlink():
        raise MatrixError(f"output dir is a symlink: {out_dir}")
    if out_dir.exists():
        old = verify_existing(receipt, out_dir)
        receipt["status"] = "skipped_verified_identical"; receipt["verified_against_receipt"] = {k: old.get(k) for k in ("started_at_utc", "finished_at_utc", "duration_s", "summary_sha256", "trace_sha256")}
        receipt.update(M.output_digest(out_dir)); M.append_log(log_path, receipt); return receipt
    if closure is None or not callable(closure.get("snapshot_fn")) or not isinstance(closure.get("launch_snapshot"), dict) or not isinstance(closure.get("job_inputs"), dict):
        raise MatrixError("execution requires a contemporaneous closure")
    launch = closure["launch_snapshot"]
    out_dir.mkdir(parents=True, exist_ok=False)
    receipt["closure_manifest"] = closure["closure_manifest"]; receipt["closure_manifest_sha256"] = closure["closure_manifest_sha256"]
    try:
        pre = closure["snapshot_fn"]()
    except Exception as exc:  # noqa: BLE001
        return _abort(receipt, out_dir, log_path, "pre-run source closure could not be established", closure_error=str(exc))
    for section in (*CL.SNAPSHOT_SECTIONS, "runtime_source_closure_digest", "orchestration_digest"):
        receipt[section] = pre[section]
    receipt["runtime_source_closure_digest_pre"] = pre["runtime_source_closure_digest"]
    equal_pre = CL.sections_equal(pre, launch); receipt["closure_sections_pre_equal_launch"] = equal_pre
    receipt["closure_matches_launch_manifest_pre"] = all(equal_pre.values()) and pre["runtime_source_closure_digest"] == closure["runtime_source_closure_digest"] == launch.get("runtime_source_closure_digest") and pre["orchestration_digest"] == launch.get("orchestration_digest")
    if not receipt["closure_matches_launch_manifest_pre"]:
        return _abort(receipt, out_dir, log_path, f"pre-run source closure differs from the launch manifest ({equal_pre})", runtime_source_closure_digest_post=None)
    try:
        inputs_pre = CL.job_inputs_table(receipt); drv_pre = driver_inputs_table()
    except Exception as exc:  # noqa: BLE001
        return _abort(receipt, out_dir, log_path, "pre-run job inputs could not be resolved", closure_error=str(exc), runtime_source_closure_digest_post=None, job_inputs_digest_pre=None, job_inputs_digest_post=None)
    receipt["job_inputs"] = inputs_pre["job_inputs"]; receipt["job_inputs_digest_pre"] = inputs_pre["job_inputs_digest"]
    bound = closure["job_inputs"].get(receipt["job_id"])
    if not (inputs_pre["job_inputs_digest"] == receipt.get("job_inputs_digest") == bound) or drv_pre["f2r_driver_inputs_digest"] != receipt.get("f2r_driver_inputs_digest"):
        return _abort(receipt, out_dir, log_path, "pre-run job/driver inputs differ from the description/launch binding", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    binding = command_binding_problems(receipt, fingerprint=pre["environment_fingerprint"], require_fingerprint=True)
    receipt["command_binding_pre_launch"] = {"ok": not binding, "problems": binding}
    if binding:
        return _abort(receipt, out_dir, log_path, f"command binding failed: {json.dumps(binding, default=str)[:1500]}", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    manifest_pre = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_pre_launch"] = {"ok": not manifest_pre, "problems": manifest_pre}
    if manifest_pre:
        return _abort(receipt, out_dir, log_path, "closure manifest on disk missing or tampered before launch", runtime_source_closure_digest_post=None, job_inputs_digest_post=None, closure_manifest_unchanged=False)
    receipt["started_at_utc"] = C.utc_now(); t0 = time.time()
    with (out_dir / STDOUT_LOG).open("w", encoding="utf-8") as handle:
        proc = subprocess_run(receipt["command"], cwd=str(C.REPO), stdout=handle, stderr=subprocess.STDOUT, text=True, check=False)
    receipt["finished_at_utc"] = C.utc_now(); receipt["duration_s"] = round(time.time() - t0, 1); receipt["returncode"] = proc.returncode
    post_error = None
    try:
        post = closure["snapshot_fn"]()
    except Exception as exc:  # noqa: BLE001
        post, post_error = None, str(exc)
    receipt["runtime_source_closure_digest_post"] = post["runtime_source_closure_digest"] if post else None
    equal_post = CL.sections_equal(post, pre) if post else {s: False for s in CL.SNAPSHOT_SECTIONS}
    receipt["closure_sections_post_equal_pre"] = equal_post
    receipt["source_closure_unchanged"] = bool(post) and all(equal_post.values()) and post["runtime_source_closure_digest"] == pre["runtime_source_closure_digest"] and post["orchestration_digest"] == pre["orchestration_digest"]
    inputs_post_error = None
    try:
        inputs_post = CL.job_inputs_table(receipt); drv_post = driver_inputs_table()
    except Exception as exc:  # noqa: BLE001
        inputs_post, drv_post, inputs_post_error = None, None, str(exc)
    receipt["job_inputs_digest_post"] = inputs_post["job_inputs_digest"] if inputs_post else None
    receipt["job_inputs_unchanged"] = bool(inputs_post) and inputs_post["job_inputs_digest"] == receipt["job_inputs_digest_pre"] and bool(drv_post) and drv_post["f2r_driver_inputs_digest"] == receipt["f2r_driver_inputs_digest"]
    receipt["closure_error"] = post_error or inputs_post_error
    manifest_post = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_post"] = {"ok": not manifest_post, "problems": manifest_post}; receipt["closure_manifest_unchanged"] = not manifest_post
    digest = M.output_digest(out_dir); receipt.update(digest)
    rollout_status = M.status_from(proc.returncode, digest)
    receipt["status"] = rollout_status if (receipt["source_closure_unchanged"] and receipt["job_inputs_unchanged"] and receipt["closure_manifest_unchanged"]) else "failed"
    receipt["status_reason"] = "ok" if receipt["status"] == "ok" else ("rollout failed" if rollout_status != "ok" else "closure/inputs/manifest changed during the job")
    try:
        receipt["stdout_tail"] = (out_dir / STDOUT_LOG).read_text(encoding="utf-8", errors="replace").splitlines()[-15:]
    except OSError:
        receipt["stdout_tail"] = []
    C.write_json(out_dir / RECEIPT_FILE, receipt); M.append_log(log_path, receipt)
    return receipt


def execute(described: list[dict[str, Any]], selected: list[dict[str, Any]], *, workers: int, log_path: Path, closure: dict[str, Any], runner: Callable[..., dict[str, Any]] = run_job) -> dict[str, Any]:
    pre = preflight_all(described, selected_ids={r["job_id"] for r in selected})
    results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=max(1, int(workers))) as pool:
        futures = [pool.submit(runner, dict(rec), log_path=log_path, closure=closure) for rec in selected]
        for fut in futures:
            results.append(fut.result())
    return {"preflight": pre, "results": results}


# --- planned pipeline commands (exact; S1+ only) -----------------------------------------------


def pipeline_commands(python_exe: str) -> dict[str, list[list[str]]]:
    """Exact S1+ commands (no placeholders).  Every command that fits, aggregates, builds the env
    or reads student rollouts carries ``--authorized-stage S1`` and refuses to run without it."""
    py = python_exe
    obs, lab, dag, gat, mat = (str(HERE / n) for n in ("f2r_observability.py", "f2r_labeller.py", "f2r_dagger.py", "f2r_gates.py", "f2r_matrix.py"))
    return {
        "P0_offline_NOT_EXECUTED_IN_S0": [[py, obs, "--real", "--authorized-stage", "S1", "--out-dir", str(R.OUT_P0)]],
        "S1_privileged_cache_build_env_zero_steps": [[py, lab, "build-cache", "--with-ik", "--authorized-stage", "S1", "--out-dir", str(R.OUT_CACHE)]],
        "T1_commissioning": [
            [py, dag, "refit", "--variant", "T1", "--round", "1", "--dataset", "anchors", "--authorized-stage", "S1", "--out-dir", str(R.OUT_REFIT / "T1" / "round_1")],
            [py, mat, "--execute", "--phase", "T1", "--round", "1", "--workers", "2", "--authorized-stage", "S1"],
            [py, gat, "--phase", "T1", "--round", "1", "--authorized-stage", "S1", "--out-dir", str(R.OUT_GATE)],
        ],
        "T1R_corrective_commissioning": [
            [py, dag, "refit-t1r", "--authorized-stage", "S1", "--out-dir", str(R.OUT_REFIT / "T1R" / "round_1")],
            [py, mat, "--execute", "--phase", "T1R", "--round", "1", "--workers", "2", "--authorized-stage", "S1"],
            [py, gat, "--phase", "T1R", "--round", "1", "--authorized-stage", "S1", "--out-dir", str(R.OUT_GATE)],
        ],
        "T2_round_1_template": [
            [py, dag, "refit", "--variant", "T2", "--round", "1", "--dataset", "anchors", "--authorized-stage", "S1", "--out-dir", str(R.OUT_REFIT / "T2" / "round_1")],
            [py, mat, "--execute", "--phase", "T2", "--round", "1", "--workers", "2", "--authorized-stage", "S1"],
            [py, gat, "--phase", "T2", "--round", "1", "--authorized-stage", "S1", "--out-dir", str(R.OUT_GATE)],
            [py, dag, "aggregate", "--variant", "T2", "--round", "1", "--authorized-stage", "S1", "--out-dir", str(R.OUT_DATASETS)],
        ],
    }


# --- dry-run manifest ------------------------------------------------------------------------------


def output_root_inventory(root: Path) -> dict[str, Any]:
    """Every directory (empty too) and file (size + SHA-256) under ``root``, relative POSIX paths;
    symlinks recorded by target, never followed.  ``{}`` when the root does not exist."""
    import hashlib

    if not root.exists():
        return {}
    inv: dict[str, Any] = {".": "dir"}
    for q in sorted(root.rglob("*")):
        rel = q.relative_to(root).as_posix()
        if q.is_symlink():
            inv[rel] = ["symlink", os.readlink(q)]
        elif q.is_dir():
            inv[rel] = "dir"
        elif q.is_file():
            inv[rel] = [q.stat().st_size, hashlib.sha256(q.read_bytes()).hexdigest()]
    return inv


def executed_artefacts() -> dict[str, Any]:
    """What the authorised S1 stages have materialised so far under OUT_ROOT (read-only detection)."""
    p0 = sorted(C.rel(q) for q in R.OUT_P0.glob("p0_result_*.json")) if R.OUT_P0.is_dir() else []
    students = sorted(C.rel(q) for q in R.OUT_REFIT.glob("*/round_*/rl_module_student/module_state.pkl")) if R.OUT_REFIT.is_dir() else []
    ok_jobs = []
    if R.OUT_ROLLOUTS.is_dir():
        for q in sorted(R.OUT_ROLLOUTS.glob("*/round_*/*/" + RECEIPT_FILE)):
            try:
                if C.read_json(q).get("status") == "ok":
                    ok_jobs.append(C.rel(q.parent))
            except Exception:  # noqa: BLE001 - an unreadable receipt is not an executed job
                continue
    gates = sorted(C.rel(q) for q in R.OUT_GATE.glob("gate_*.json")) if R.OUT_GATE.is_dir() else []
    launch_sets = []
    if R.OUT_ROLLOUTS.is_dir():
        for q in sorted(R.OUT_ROLLOUTS.glob("f2r_job_matrix_status_*.json")):
            try:
                st = C.read_json(q)
                launch_sets.append({"tag": st.get("tag") or q.stem.replace("f2r_job_matrix_status_", ""), "phase": st.get("phase"), "round": st.get("round"), "status": st.get("status") or ("ok" if st.get("failed") == 0 and st.get("job_count") else "unknown"), "job_count": st.get("job_count"), "ok": st.get("ok"), "failed": st.get("failed"), "error": (st.get("error") or "")[:160], "file": C.rel(q)})
            except Exception:  # noqa: BLE001 - an unreadable status is reported as such, never hidden
                launch_sets.append({"tag": q.stem.replace("f2r_job_matrix_status_", ""), "status": "unreadable", "file": C.rel(q)})
    caches = sorted(C.rel(q) for q in R.OUT_CACHE.glob("privileged_cache_*.npz")) if R.OUT_CACHE.is_dir() else []
    datasets = sorted(C.rel(q) for q in R.OUT_DATASETS.glob("*.npz")) if R.OUT_DATASETS.is_dir() else []
    stage = "S0" if not (p0 or students or ok_jobs or gates or caches or datasets) else "S1"
    return {"stage": stage, "p0_results": p0, "student_modules": students, "rollout_jobs_ok": ok_jobs, "gate_results": gates, "privileged_caches": caches, "datasets": datasets, "launch_sets": launch_sets}


def immutability_audit() -> dict[str, Any]:
    f1_manifest = F1.OUT_ROOT / "manifest" / "f1_job_matrix_dry_run_20260823_142416.json"
    f1_pins_ok = None
    if f1_manifest.is_file():
        m = C.read_json(f1_manifest)
        f1_pins_ok = all(C.sha256_file(R.F1_DIR / k) == v for k, v in m["f1_scripts_sha256"].items())
    dirty = F1.load_protocol()["artefact_rules"]["dirty_user_files_immutable"]
    return {
        "f0_analysis": {"path": C.rel(F1.F0_ANALYSIS_JSON), "sha256": C.sha256_file(F1.F0_ANALYSIS_JSON) if F1.F0_ANALYSIS_JSON.is_file() else None, "expected": F1.F0_ANALYSIS_SHA256, "ok": F1.F0_ANALYSIS_JSON.is_file() and C.sha256_file(F1.F0_ANALYSIS_JSON) == F1.F0_ANALYSIS_SHA256},
        "f1_scripts_pins_vs_r3_manifest": {"manifest": C.rel(f1_manifest), "ok": f1_pins_ok},
        "dirty_user_files": {p: (C.sha256_file(C.REPO / p) if (C.REPO / p).is_file() else None) for p in dirty},
        "production_rollout_eval_sha256": C.sha256_file(R.ROLLOUT_EVAL),
    }


def write_dry_run_manifest(described: list[dict[str, Any]], *, stamp: str, python_record: dict[str, Any]) -> dict[str, str]:
    inventory_pre = output_root_inventory(R.OUT_ROOT)  # the dry-run may add ONLY its three manifest files
    R.OUT_MANIFEST.mkdir(parents=True, exist_ok=True)  # dry-run materialises only manifest/spec/index files
    protocol = R.load_protocol()
    executed = executed_artefacts()
    manifest = {
        "schema_version": 2, "mode": "dry_run", "dry_run": True, "stage": executed["stage"], "stamp": stamp, "generated_at_utc": C.utc_now(),
        "s0_forbidden": protocol["s0_forbidden"], "s0_forbidden_applies_to_stage": "S0", "executed_artefacts_detected": executed,
        "p0_executed": bool(executed["p0_results"]), "refit_executed": bool(executed["student_modules"]), "rollouts_executed": len(executed["rollout_jobs_ok"]), "dry_run_launches_jobs": False,
        "protocol": {"id": R.PROTOCOL_ID, **R.protocol_digests()},
        "f2r_scripts_sha256": {p: C.sha256_file(HERE / p) for p in F2R_SCRIPTS if (HERE / p).is_file()},
        "f2r_tests_sha256": {p.name: C.sha256_file(p) for p in sorted(HERE.glob("test_f2r_*.py"))},
        "f0_library_sha256": {p: C.sha256_file(R.F0_DIR / p) for p in CL.ORCHESTRATION_FILES if (R.F0_DIR / p).is_file()},
        "f1_library_sha256": {p: C.sha256_file(R.F1_DIR / p) for p in ("f1_common.py", "f1_obs_adapter.py", "f1_dataset.py", "f1_refit.py", "f1_sigma_variant.py", "f1_analysis.py", "f1_matrix.py")},
        "git": C.git_snapshot(), "interpreter": python_record,
        "runtime": {"name": F1.TARGET_RUNTIME, "config": C.rel(R.RUNTIME_CONFIG), "config_sha256": C.sha256_file(R.RUNTIME_CONFIG), "rollout_eval_sha256": C.sha256_file(R.ROLLOUT_EVAL)},
        "actors": R.verify_actor_pins(), "anchors": R.verify_anchor_pins(), "p0_jul_jobs": R.verify_p0_jul_pins(), "corridor_profile": {"path": C.rel(R.CORRIDOR_PROFILE["path"]), "sha256": C.sha256_file(R.CORRIDOR_PROFILE["path"]), "pinned": R.CORRIDOR_PROFILE["sha256"]},
        "immutability": immutability_audit(),
        "seeds": {"det": R.DET_SEED, "collection": list(R.COLLECTION_SEEDS), "validation_gate": R.VALIDATION_SEED, "sealed": list(R.SEALED_SEEDS), "used_in_matrix": sorted({int(r["seed"]) for r in described}), "sealed_seed_found": any(int(r["seed"]) in R.SEALED_SEEDS for r in described), "validation_jobs_enter_dataset": any(r["enters_dataset"] for r in described if r["purpose"] == "validation_gate")},
        "variants": protocol["variants"], "refit_budget": protocol["refit_budget"], "sequence_rev3": protocol["sequence_rev3"], "gates": protocol["gates"], "p0": protocol["p0"],
        "pipeline_commands": pipeline_commands(python_record.get("selected") or M.PYTHON_PLACEHOLDER),
        "job_count": len(described), "job_counts_by_phase_round": {f"{r['phase']}_r{r['round']}": sum(1 for x in described if x["phase"] == r["phase"] and x["round"] == r["round"]) for r in described},
        "job_counts_by_purpose": {p: sum(1 for r in described if r["purpose"] == p) for p in PURPOSES},
        "jobs_pending_module": [f"{r['phase']}/round_{r['round']}/{r['job_id']}" for r in described if r["module_pending"]],
        "existing_output_dirs": [r["output_dir"] for r in described if r["output_dir_exists"]],
        "R_evaluation": "no new rollouts: the 9 stochastic jobs of the promoted round (6 collection + 3 V)",
        "jobs": described,
    }
    if manifest["seeds"]["sealed_seed_found"] or manifest["seeds"]["validation_jobs_enter_dataset"]:
        raise MatrixError("structural violation in the matrix")
    if not (manifest["actors"]["all_match"] and manifest["anchors"]["all_match"] and manifest["p0_jul_jobs"]["all_match"] and manifest["corridor_profile"]["sha256"] == manifest["corridor_profile"]["pinned"]):
        raise MatrixError("pins do not match the disk")
    previous = sorted(p.name for p in R.OUT_MANIFEST.glob("f2r_s0_dry_run_*.index.json")) if R.OUT_MANIFEST.is_dir() else []
    reserved = R.reserve_unique_set(R.OUT_MANIFEST, f"f2r_s0_dry_run_{stamp}", (".json", ".md", ".index.json"))
    path, md_path, index_path = reserved[".json"], reserved[".md"], reserved[".index.json"]
    manifest["artefact_names"] = {"manifest": path.name, "markdown": md_path.name, "index": index_path.name}
    manifest["previous_manifests_superseded_by_this"] = previous
    manifest["materialisation_policy"] = "S0 dry-run writes exactly these three files under OUT_MANIFEST (exclusive O_EXCL reservation, atomic fill); no rollouts/datasets/privileged_cache/refit/p0/gate/logs directory is created"
    manifest["process_policy"] = "dry-run launches no job and no interpreter probe (python = sys.executable of the driver); the only subprocesses are read-only git metadata queries"
    R._atomic_fill(path, json.dumps(manifest, indent=2, sort_keys=False, default=str).encode("utf-8"))
    n_pending = sum(1 for r in described if r["module_pending"]); n_mat = sum(1 for r in described if r["output_dir_exists"])
    md = [f"# F2R dry-run {stamp} — detected stage {executed['stage']} (the dry-run launches no job, no P0, no refit)", "", f"Protocol SHA-256: `{manifest['protocol']['f2r_protocol.json']}` — init JUL_H0 actor digest `{R.INIT_PRIMARY['actor_digest'][:16]}…` (module_state.pkl SHA-256 `{R.INIT_PRIMARY['module_state_sha256'][:16]}…`) — git HEAD `{manifest['git']['head'][:12]}`", "", f"Jobs: {len(described)} — student module pending for {n_pending}, materialised outputs for {n_mat}; executed so far: P0 results {len(executed['p0_results'])}, student modules {len(executed['student_modules'])}, ok rollouts {len(executed['rollout_jobs_ok'])}, gates {len(executed['gate_results'])}; purposes: {manifest['job_counts_by_purpose']}", ""]
    md.append(C.md_table(["key", "phase", "round", "purpose", "start", "sel", "seed", "enters_dataset", "module pending"], [[f"{r['phase']}/r{r['round']}", r["phase"], r["round"], r["purpose"], r["start"], r["action_selection"], r["seed"], r["enters_dataset"], r["module_pending"]] for r in described]))
    R._atomic_fill(md_path, ("\n".join(md) + "\n").encode("utf-8"))
    # materialisation audit (fail closed) BEFORE the index is filled (its name is already reserved):
    # the output root must differ from its pre-dry-run inventory by EXACTLY the three new manifest
    # files (plus manifest/ and the root when they did not exist yet); nothing removed or changed
    inventory_post = output_root_inventory(R.OUT_ROOT)
    expected_new = {f"manifest/{path.name}", f"manifest/{md_path.name}", f"manifest/{index_path.name}"}
    allowed_dirs = {".", "manifest"} - set(inventory_pre)
    added = set(inventory_post) - set(inventory_pre)
    removed = set(inventory_pre) - set(inventory_post)
    changed = {k for k in inventory_post if k in inventory_pre and inventory_post[k] != inventory_pre[k]}
    if added != expected_new | allowed_dirs or removed or changed:
        raise MatrixError(f"dry-run materialisation audit failed: added={sorted(added - expected_new - allowed_dirs)} removed={sorted(removed)} changed={sorted(changed)}")
    # the index is COMPLETE before its atomic fill: what is returned is byte-for-byte what is on disk
    index = {"manifest": C.rel(path), "manifest_sha256": C.sha256_file(path), "markdown": C.rel(md_path), "markdown_sha256": C.sha256_file(md_path), "previous_manifests_superseded": previous, "materialised_files": [path.name, md_path.name, index_path.name], "materialisation_audit": {"pre_entries": len(inventory_pre), "post_entries": len(inventory_post), "added": sorted(added), "removed": [], "changed": []}}
    R._atomic_fill(index_path, json.dumps(index, indent=2).encode("utf-8"))
    return index


# --- S1+ launch: the whole launch set is reserved exclusively BEFORE any job ---------------------------

LAUNCH_TEMPLATES = {"closure": "source_closure_manifest_{tag}.json", "execute": "f2r_job_matrix_execute_{tag}.json", "status": "f2r_job_matrix_status_{tag}.json"}
DRIVER_LOG = "driver_log.jsonl"  # append-only log (kept as such)


def reserve_launch_set(stamp: str) -> dict[str, Any]:
    """Reserve ``source_closure_manifest_<tag>.json``, ``f2r_job_matrix_execute_<tag>.json`` and
    ``f2r_job_matrix_status_<tag>.json`` under OUT_ROLLOUTS with ONE common tag (``<stamp>`` or
    ``<stamp>_NN`` on collision; partial collisions roll the whole attempt back)."""
    return R.reserve_unique_names(R.OUT_ROLLOUTS, stamp, LAUNCH_TEMPLATES)


def write_closure_manifest_f2r(path: Path, snapshot: dict[str, Any], *, job_inputs: dict[str, str] | None = None) -> dict[str, Any]:
    """F2R equivalent of ``f0_closure.write_closure_manifest`` (payload, JSON bytes, schema and
    digests identical; verified by the unchanged ``f0_closure.verify_manifest_file``) filled
    atomically into a path reserved by ``reserve_launch_set`` instead of the legacy writer."""
    payload = dict(snapshot)
    payload["closure_schema_version"] = CL.CLOSURE_SCHEMA_VERSION
    payload["job_inputs"] = dict(job_inputs or {})
    R._atomic_fill(path, json.dumps(payload, indent=2, sort_keys=False, default=str).encode("utf-8"))
    return {"closure_manifest": C.rel(path), "closure_manifest_sha256": C.sha256_file(path)}


def launch_round(described: list[dict[str, Any]], selected: list[dict[str, Any]], *, phase: str, round_index: int, stamp: str, python_record: dict[str, Any], workers: int, snapshot_fn: Callable[[], dict[str, Any]], runner: Callable[..., dict[str, Any]] = run_job, git_snapshot_fn: Callable[[], dict[str, Any]] = C.git_snapshot) -> dict[str, Any]:
    """S1+ launch of the jobs of one phase/round: (1) reserve the launch set (3 names, one tag)
    before anything else; (2) closure snapshot + closure manifest (atomic fill) + launch manifest
    (atomic fill) BEFORE the jobs; (3) jobs through ``execute``; (4) status (atomic fill) AFTER the
    jobs — or an ``aborted`` status if the execution raises (no empty reservation left behind)."""
    if not selected:
        raise MatrixError(f"no jobs selected for {phase} round {round_index}")
    pending = [r["job_id"] for r in selected if r["module_pending"]]
    if pending:
        raise MatrixError(f"cannot execute: student module pending for {pending}")
    reserved = reserve_launch_set(stamp)
    tag, paths = reserved["tag"], reserved["paths"]
    snapshot = snapshot_fn()
    bindings = {r["job_id"]: r["job_inputs_digest"] for r in selected}
    written = write_closure_manifest_f2r(paths["closure"], snapshot, job_inputs=bindings)
    closure = {**written, "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"], "launch_snapshot": snapshot, "job_inputs": bindings, "snapshot_fn": snapshot_fn}
    launch_manifest = {"mode": "execute", "stage": "S1+", "phase": phase, "round": int(round_index), "stamp": stamp, "tag": tag, "launch_set": {k: p.name for k, p in paths.items()}, "selected": [r["job_id"] for r in selected], "closure_manifest": written, "git": git_snapshot_fn(), "interpreter": python_record}
    R._atomic_fill(paths["execute"], json.dumps(launch_manifest, indent=2, default=str).encode("utf-8"))
    t0 = time.time()
    try:
        outcome = execute(described, selected, workers=workers, log_path=R.OUT_ROLLOUTS / DRIVER_LOG, closure=closure, runner=runner)
    except BaseException as exc:
        aborted = {"generated_at_utc": C.utc_now(), "phase": phase, "round": int(round_index), "tag": tag, "status": "aborted", "error": f"{type(exc).__name__}: {exc}"[:2000], "elapsed_s": round(time.time() - t0, 1), "job_count": len(selected)}
        R._atomic_fill(paths["status"], json.dumps(aborted, indent=2, default=str).encode("utf-8"))
        raise
    status = {"generated_at_utc": C.utc_now(), "phase": phase, "round": int(round_index), "tag": tag, "launch_set": {k: p.name for k, p in paths.items()}, "elapsed_s": round(time.time() - t0, 1), "preflight": outcome["preflight"], "job_count": len(selected), "ok": sum(1 for r in outcome["results"] if r.get("status") == "ok"), "skipped_verified_identical": sum(1 for r in outcome["results"] if r.get("status") == "skipped_verified_identical"), "failed": sum(1 for r in outcome["results"] if r.get("status") == "failed"), "results": sorted(outcome["results"], key=lambda r: r["job_id"])}
    R._atomic_fill(paths["status"], json.dumps(status, indent=2, default=str).encode("utf-8"))
    return {"tag": tag, "paths": {k: C.rel(p) for k, p in paths.items()}, "status": status}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="F2R job matrix driver (S0: dry-run only).")
    parser.add_argument("--execute", action="store_true"); parser.add_argument("--phase", default=None); parser.add_argument("--round", type=int, default=None)
    parser.add_argument("--workers", type=int, default=2); parser.add_argument("--list", action="store_true"); parser.add_argument("--authorized-stage", default=None)
    args = parser.parse_args(argv)
    specs = build_jobs()
    if args.list:
        for s in specs:
            print(spec_key(s), s["purpose"], s["seed"])
        return 0
    if args.execute:
        python_record = C.select_python()  # S1+: validated interpreter (probe subprocess), as in F0/F1
    else:
        python_record = {"selected": sys.executable, "source": "sys.executable (dry-run: no interpreter probe, no subprocess)", "validation": None}
    described = [describe_job(s, python_record.get("selected")) for s in specs]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if not args.execute:
        index = write_dry_run_manifest(described, stamp=stamp, python_record=python_record)
        stage = C.read_json(C.REPO / index["manifest"])["stage"]  # detected from the artefacts, never assumed
        print(json.dumps({"mode": "dry_run", "stage": stage, "jobs": len(described), **index}, indent=2))
        return 0
    if args.authorized_stage != "S1":
        raise SystemExit("--execute is not authorised in S0 (no rollout); pass --authorized-stage S1 only after the architect's explicit go")
    if args.phase not in ("T1", "T1R", "T2", "T3") or args.round is None:
        raise SystemExit("--execute requires --phase T1|T2|T3 and --round r")
    selected = [r for r in described if r["phase"] == args.phase and r["round"] == int(args.round)]
    if not selected:
        raise SystemExit(f"no jobs for {args.phase} round {args.round}")
    pending = [r["job_id"] for r in selected if r["module_pending"]]
    if pending:
        raise SystemExit(f"cannot execute: student module pending for {pending}")
    python_exe = python_record["selected"]
    orch = tuple(str(R.F0_DIR / p) for p in CL.ORCHESTRATION_FILES) + tuple(str(HERE / p) for p in F2R_SCRIPTS if (HERE / p).is_file())
    snapshot_fn = lambda: CL.closure_snapshot(python_exe, orchestration_files=orch)  # noqa: E731
    result = launch_round(described, selected, phase=args.phase, round_index=int(args.round), stamp=stamp, python_record=python_record, workers=args.workers, snapshot_fn=snapshot_fn)
    status = result["status"]
    print(json.dumps({"tag": result["tag"], **{k: status[k] for k in ("phase", "round", "job_count", "ok", "skipped_verified_identical", "failed", "elapsed_s")}}, indent=2))
    return 0 if status["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
