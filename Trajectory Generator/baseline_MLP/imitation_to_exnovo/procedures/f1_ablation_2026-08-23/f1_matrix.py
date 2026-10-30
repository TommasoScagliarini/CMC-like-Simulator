"""F1 job-matrix driver: description, dry-run manifest, fail-closed execution.

Default mode is **dry-run**: every job is described (identity, exact argv,
digests of the inputs that exist, pending inputs flagged), the manifest and
a Markdown table are written no-clobber under ``OUT_MANIFEST``; no process is
launched.  ``--execute --stage N`` runs the jobs of stage N (S2 only).

Receipt schema ``f1.1`` keeps every F0 schema-5 closure field (so
``f0_closure.verify_closure_fields`` verifies F1 receipts unchanged) and adds:
``driver`` (rollout_eval | f1_rollout_aiso), ``driver_sha256``,
``adapter_mode``, ``stage``, ``module_pending``, ``f1_driver_inputs`` (+
digest), and for A_iso jobs the side-car digests and assertion counters.

Only registry-agnostic F0 functions are imported (``f0_closure``,
``f0_rollout_matrix.output_digest/status_from/normalize_command/append_log``,
``f0_common`` helpers); no F0 global is rebound; F0 outputs are read-only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
for entry in (str(HERE),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import f1_common as F1  # noqa: E402
import f0_common as C  # noqa: E402
import f0_closure as CL  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402  (only registry-agnostic helpers)
import f1_obs_adapter as OA  # noqa: E402

RECEIPT_FILE = "f1_receipt.json"
RECEIPT_SCHEMA = "f1.1"
STDOUT_LOG = "f1_driver_stdout.log"
ADAPTER_SUMMARY_FILE = "f1_adapter_summary.json"
ADAPTER_TRACE_FILE = "f1_adapter_trace.json"
RECEIPT_VERIFY_FIELDS = ("job_id", "command_normalized", "module_state_sha256", "config_sha256", "rollout_eval_sha256", "driver_sha256", "git_head", "output_dir", "stage", "adapter_mode")
F1_SCRIPTS = ("f1_common.py", "f1_obs_adapter.py", "f1_rollout_aiso.py", "f1_adapter_crosscheck.py", "f1_sigma_variant.py", "f1_dataset.py", "f1_refit.py", "f1_matrix.py", "f1_analysis.py", "f1_protocol.json", "PROTOCOL.md")
# Scripts whose bytes can change the behaviour of a rollout job (bound per job
# and re-verified on preflight); analysis/dataset/refit tools are part of the
# orchestration closure recorded at launch but do not invalidate earlier jobs.
ROLLOUT_AFFECTING_SCRIPTS = ("f1_common.py", "f1_obs_adapter.py", "f1_rollout_aiso.py", "f1_matrix.py", "f1_protocol.json")
F0_DRIVER_LIBS = ("f0_common.py", "f0_closure.py", "f0_rollout_matrix.py")
DERIVED_MODULE_DIRS = {"C": F1.OUT_DERIVED / "B_sigma0005" / "rl_module", "A_ISO39_V3_S005": F1.OUT_DERIVED / "V26_39D_sigma0005" / "rl_module", "D": F1.OUT_REFIT / "D_refit_v1" / "rl_module_refit"}
DATASET_STAMP = "D_v1"

_log_lock = __import__("threading").Lock()


class MatrixError(RuntimeError):
    pass


# --- job specs --------------------------------------------------------------------


def planned_module(candidate: str) -> Path:
    spec = F1.CANDIDATES[candidate]
    if spec["source"] == "frozen":
        return Path(spec["module"])
    return DERIVED_MODULE_DIRS[candidate]


def job_id(spec: dict[str, Any]) -> str:
    mode_tag = "det" if spec["action_selection"] == "deterministic" else f"stoch_seed{spec['seed']}"
    rep_tag = f"_rep{spec['repeat']}" if spec["repeat"] > 1 else ""
    return f"{spec['candidate']}__{spec['runtime']}__{spec['start']}__{mode_tag}{rep_tag}"


def output_dir_for(spec: dict[str, Any]) -> Path:
    return F1.OUT_ROLLOUTS / spec["family"] / spec["job_id"]


def _spec(family: str, stage: int, candidate: str, start: str, mode: str, seed: int, repeat: int = 1, *, gate_relevant: bool = True) -> dict[str, Any]:
    cand = F1.CANDIDATES[candidate]
    F1.assert_development_seed(seed)
    spec = {
        "family": family, "stage": int(stage), "candidate": candidate, "comparison_class": cand["comparison_class"], "role": cand["role"],
        "runtime": F1.TARGET_RUNTIME, "start": start, "episode_start_offset_s": F1.EXACT_STARTS[start],
        "action_selection": mode, "seed": int(seed), "repeat": int(repeat), "driver": cand["driver"], "adapter_mode": cand["adapter"],
        "gate_relevant": bool(gate_relevant and not cand.get("optional_diagnostic", False)), "optional_diagnostic": bool(cand.get("optional_diagnostic", False)),
    }
    spec["job_id"] = job_id(spec)
    return spec


def build_jobs() -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    S = F1.STARTS
    seeds = F1.DEVELOPMENT_SEEDS
    for s in S:
        jobs.append(_spec("aiso_det", 1, "A_ISO39_V3", s, "deterministic", 123))
    for s in S:
        for k in seeds:
            jobs.append(_spec("aiso_stoch_s005", 1, "A_ISO39_V3_S005", s, "stochastic", k))
    for s in S:
        jobs.append(_spec("b_det", 1, "B", s, "deterministic", 123))
    for s in S:
        for k in seeds:
            jobs.append(_spec("b_stoch_native", 1, "B", s, "stochastic", k))
    jobs.append(_spec("b_stoch_native", 1, "B", "nominal", "stochastic", 123, repeat=2))
    for s in S:
        jobs.append(_spec("c_det", 1, "C", s, "deterministic", 123))
    for s in S:
        for k in seeds:
            jobs.append(_spec("c_stoch_s005", 1, "C", s, "stochastic", k))
    jobs.append(_spec("passthrough", 1, "PASSTHROUGH_B", "nominal", "deterministic", 123))
    for s in S:
        jobs.append(_spec("aiso_clk_diag", 1, "A_ISO39CLK_V3", s, "deterministic", 123, gate_relevant=False))
    for s in S:
        jobs.append(_spec("d_det", 3, "D", s, "deterministic", 123))
    for s in S:
        for k in seeds:
            jobs.append(_spec("d_stoch_s005", 3, "D", s, "stochastic", k))
    ids = [j["job_id"] for j in jobs]
    if len(set(ids)) != len(ids):
        raise MatrixError("duplicate job ids")
    for j in jobs:
        if j["seed"] in F1.SEALED_SEEDS:
            raise MatrixError("sealed seed in matrix")
    assert len(jobs) == 53, len(jobs)
    return jobs


def canonical_index() -> dict[str, dict[str, Any]]:
    return {j["job_id"]: j for j in build_jobs()}


def job_command(spec: dict[str, Any], out_dir: Path, python_exe: str) -> list[str]:
    module = planned_module(spec["candidate"])
    script = F1.ROLLOUT_EVAL if spec["driver"] == "rollout_eval" else F1.AISO_DRIVER
    cmd = [
        python_exe, str(script),
        "--checkpoint", str(module),
        "--no-auto-config",
        "--config", str(F1.RUNTIME_CONFIG),
        "--episode-start-offset-s", repr(float(F1.EXACT_STARTS[spec["start"]])),
        "--action-selection", spec["action_selection"],
        "--seed", str(int(spec["seed"])),
        "--output-dir", str(out_dir),
        "--record-outputs", "--record-policy-trace",
        *F1.JOB_TIMEOUT_ARGS,
        *list(C.RUNTIMES[F1.TARGET_RUNTIME]["extra_args"]),
    ]
    if spec["driver"] == "f1_rollout_aiso":
        cmd.extend(["--f1-adapter", str(spec["adapter_mode"])])
    return cmd


def f1_driver_inputs_table() -> dict[str, Any]:
    paths = [HERE / p for p in ROLLOUT_AFFECTING_SCRIPTS]
    paths += [F1.F0_DIR / p for p in F0_DRIVER_LIBS]
    table = CL.digest_table(paths, label="f1 driver inputs")
    return {"f1_driver_inputs": table, "f1_driver_inputs_digest": CL.table_digest(table), "f1_driver_inputs_count": len(table)}


def describe_job(spec: dict[str, Any], python_exe: str | None) -> dict[str, Any]:
    out_dir = output_dir_for(spec)
    module = planned_module(spec["candidate"])
    module_state = module / "module_state.pkl"
    pending = not module_state.is_file()
    script = F1.ROLLOUT_EVAL if spec["driver"] == "rollout_eval" else F1.AISO_DRIVER
    python = python_exe or M.PYTHON_PLACEHOLDER
    command = job_command(spec, out_dir, python)
    cand = F1.CANDIDATES[spec["candidate"]]
    rec: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "provenance_class": CL.PROVENANCE_CLASS_CONTEMPORANEOUS,
        "provenance_limitation": CL.PROVENANCE_LIMITATION,
        "protocol_id": "F1-S1-ablation-39to35-sigma-v1",
        "protocol_sha256": F1.protocol_digests()["f1_protocol.json"],
        **{k: spec[k] for k in ("job_id", "family", "stage", "candidate", "comparison_class", "role", "runtime", "start", "episode_start_offset_s", "action_selection", "seed", "repeat", "driver", "adapter_mode", "gate_relevant", "optional_diagnostic")},
        "candidate_description": cand["description"],
        "runtime_description": C.RUNTIMES[F1.TARGET_RUNTIME]["description"],
        "runtime_is_target_v3": True,
        "isometric_with_B": cand["comparison_class"] in ("isometric_comparable", "isometric_privileged_39D"),
        "env_width": cand["env_width"], "module_width": cand["module_width"],
        "env_manifest": C.rel(cand["env_manifest"]), "env_manifest_sha256": cand["env_manifest_sha256"],
        "module_manifest": C.rel(cand["module_manifest"]), "module_manifest_sha256": cand["module_manifest_sha256"],
        "module": C.rel(module), "module_source": cand["source"], "module_pending": pending,
        "module_state_sha256": C.sha256_file(module_state) if not pending else None,
        "config": C.rel(F1.RUNTIME_CONFIG), "config_sha256": C.sha256_file(F1.RUNTIME_CONFIG),
        "rollout_eval_sha256": C.sha256_file(F1.ROLLOUT_EVAL),
        "driver_script": C.rel(script), "driver_sha256": C.sha256_file(script),
        "python": python, "cwd": str(C.REPO), "command": command, "command_normalized": M.normalize_command(command),
        "output_dir": C.rel(out_dir), "output_dir_exists": out_dir.exists(),
        "git_head": C.git("rev-parse", "HEAD"),
    }
    if rec["config_sha256"] != F1.RUNTIME_CONFIG_SHA256:
        raise MatrixError(f"pinned runtime config digest changed: {rec['config_sha256']}")
    rec.update(f1_driver_inputs_table())
    if not pending:
        rec.update(CL.job_inputs_table(rec))
    else:
        rec.update({"job_inputs": None, "job_inputs_digest": None, "job_inputs_count": 0, "module_pending_reason": f"{cand['source']} module not materialised yet (stage {1 if cand['source'] == 'derived_sigma' else 2})"})
    return rec


# --- command binding ----------------------------------------------------------------


def command_binding_problems(receipt: dict[str, Any], *, fingerprint: dict[str, Any] | None = None, require_fingerprint: bool = False) -> dict[str, Any]:
    problems: dict[str, Any] = {}
    index = canonical_index()
    spec = index.get(receipt.get("job_id"))
    if spec is None:
        return {"job_id": {"existing": receipt.get("job_id"), "expected": "F1 canonical job id"}}
    for k in ("family", "stage", "candidate", "runtime", "start", "action_selection", "seed", "repeat", "driver", "adapter_mode"):
        if receipt.get(k) != spec[k] or type(receipt.get(k)) is not type(spec[k]):
            problems[k] = {"existing": receipt.get(k), "expected": spec[k]}
    cmd = receipt.get("command")
    if not isinstance(cmd, list) or not cmd or not all(isinstance(t, str) and t for t in cmd):
        return {**problems, "command": {"existing": cmd, "expected": "non-empty list of non-empty strings"}}
    if M.normalize_command(cmd) != receipt.get("command_normalized"):
        problems["command_normalized"] = {"existing": receipt.get("command_normalized"), "expected": M.normalize_command(cmd)}
    if cmd[0] != receipt.get("python"):
        problems["command[0]"] = {"existing": cmd[0], "expected": receipt.get("python")}
    expected_cmd = job_command(spec, output_dir_for(spec), str(receipt.get("python")))  # canonical dir (same rule as run_job)
    if cmd != expected_cmd:
        problems["command_argv"] = {"existing": cmd, "expected": expected_cmd}
    if receipt.get("output_dir") != C.rel(output_dir_for(spec)):
        problems["output_dir"] = {"existing": receipt.get("output_dir"), "expected": C.rel(output_dir_for(spec))}
    if receipt.get("cwd") != str(C.REPO):
        problems["cwd"] = {"existing": receipt.get("cwd"), "expected": str(C.REPO)}
    script = Path(cmd[1])
    expected_script = F1.ROLLOUT_EVAL if spec["driver"] == "rollout_eval" else F1.AISO_DRIVER
    if not script.is_file() or script.is_symlink() or os.path.realpath(script) != os.path.realpath(expected_script):
        problems["command[1]"] = {"existing": cmd[1], "expected": str(expected_script)}
    elif C.sha256_file(script) != receipt.get("driver_sha256"):
        problems["driver_sha256"] = {"existing": receipt.get("driver_sha256"), "expected": C.sha256_file(script)}
    if C.sha256_file(F1.ROLLOUT_EVAL) != receipt.get("rollout_eval_sha256"):
        problems["rollout_eval_sha256"] = {"existing": receipt.get("rollout_eval_sha256"), "expected": C.sha256_file(F1.ROLLOUT_EVAL)}
    if "--no-auto-config" not in cmd or cmd.count("--no-auto-config") != 1:
        problems["--no-auto-config"] = "must appear exactly once"
    if spec["driver"] == "f1_rollout_aiso":
        if cmd.count("--f1-adapter") != 1 or cmd[cmd.index("--f1-adapter") + 1] != spec["adapter_mode"]:
            problems["--f1-adapter"] = {"expected": spec["adapter_mode"]}
    elif "--f1-adapter" in cmd:
        problems["--f1-adapter"] = "not allowed for rollout_eval jobs"
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


# --- verification of existing outputs ---------------------------------------------------


def verify_existing(receipt: dict[str, Any], out_dir: Path) -> dict[str, Any]:
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
        if old.get(k) != receipt.get(k):
            mism[k] = {"stored": old.get(k), "fresh": receipt.get(k)}
    if old.get("f1_driver_inputs_digest") != receipt.get("f1_driver_inputs_digest"):
        mism["f1_driver_inputs_digest"] = {"stored": old.get("f1_driver_inputs_digest"), "fresh": receipt.get("f1_driver_inputs_digest")}
    if old.get("status") != "ok" or old.get("returncode") != 0:
        mism["status"] = {"status": old.get("status"), "returncode": old.get("returncode")}
    digest = M.output_digest(out_dir)
    for k in ("summary_sha256", "trace_sha256"):
        if digest.get(k) != old.get(k) or digest.get(k) is None:
            mism[k] = {"stored": old.get(k), "disk": digest.get(k)}
    if not (digest["summary_present"] and digest["summary_ok"] and digest["trace_present"]):
        mism["outputs"] = digest
    if old.get("driver") == "f1_rollout_aiso":
        side = adapter_sidecar_digest(out_dir)
        for k in ("adapter_summary_sha256", "adapter_trace_sha256"):
            if side.get(k) != old.get(k) or side.get(k) is None:
                mism[k] = {"stored": old.get(k), "disk": side.get(k)}
        if not side.get("adapter_ok"):
            mism["adapter"] = side
    if mism:
        raise MatrixError(f"existing output {out_dir.name} does not match: {json.dumps(mism, default=str)[:2000]}")
    return old


def preflight_all(described: list[dict[str, Any]]) -> dict[str, Any]:
    checked, invalid, ok = 0, [], []
    for rec in described:
        out_dir = C.REPO / rec["output_dir"]
        if not out_dir.exists():
            continue
        checked += 1
        try:
            verify_existing(rec, out_dir)
            ok.append(rec["job_id"])
        except Exception as exc:  # noqa: BLE001
            invalid.append({"job_id": rec["job_id"], "error": str(exc)[:600]})
    report = {"checked_existing": checked, "verified_identical": ok, "invalid": invalid}
    if invalid:
        raise MatrixError(f"preflight found invalid existing outputs: {json.dumps(invalid)[:2000]}")
    return report


def adapter_sidecar_digest(out_dir: Path) -> dict[str, Any]:
    s_path, t_path = out_dir / ADAPTER_SUMMARY_FILE, out_dir / ADAPTER_TRACE_FILE
    out: dict[str, Any] = {"adapter_summary_present": s_path.is_file(), "adapter_trace_present": t_path.is_file(), "adapter_summary_sha256": C.sha256_file(s_path) if s_path.is_file() else None, "adapter_trace_sha256": C.sha256_file(t_path) if t_path.is_file() else None, "adapter_ok": False}
    if s_path.is_file():
        try:
            s = C.read_json(s_path)
        except ValueError:
            s = {}
        out["adapter_mode_recorded"] = s.get("adapter_mode")
        out["adapter_steps"] = s.get("steps")
        out["adapter_projection_assert_count"] = s.get("projection_assert_count")
        out["adapter_ok"] = bool(s.get("all_steps_asserted") is True and s.get("steps_match_rollout") is True and out["adapter_trace_present"])
    return out


# --- execution ----------------------------------------------------------------------------


def _abort(receipt: dict[str, Any], out_dir: Path, log_path: Path, reason: str, **fields: Any) -> dict[str, Any]:
    receipt.update(fields)
    receipt.update({"returncode": None, "status": "failed", "status_reason": reason + "; job not launched"})
    receipt.update(M.output_digest(out_dir))
    C.write_json(out_dir / RECEIPT_FILE, receipt)
    M.append_log(log_path, receipt)
    return receipt


def run_job(receipt: dict[str, Any], *, log_path: Path, closure: dict[str, Any] | None = None, subprocess_run: Callable[..., Any] = subprocess.run) -> dict[str, Any]:
    early = command_binding_problems(receipt)
    if early:
        raise MatrixError(f"refusing to run {receipt.get('job_id')!r}: {json.dumps(early, default=str)[:1500]}")
    if receipt.get("module_pending"):
        raise MatrixError(f"refusing to run {receipt['job_id']}: module pending ({receipt.get('module_pending_reason')})")
    out_dir = output_dir_for(canonical_index()[receipt["job_id"]])
    if out_dir.is_symlink():
        raise MatrixError(f"output dir is a symlink: {out_dir}")
    if out_dir.exists():
        old = verify_existing(receipt, out_dir)
        receipt["status"] = "skipped_verified_identical"
        receipt["verified_against_receipt"] = {k: old.get(k) for k in ("started_at_utc", "finished_at_utc", "duration_s", "summary_sha256", "trace_sha256")}
        receipt.update(M.output_digest(out_dir))
        M.append_log(log_path, receipt)
        return receipt
    if closure is None or not callable(closure.get("snapshot_fn")) or not isinstance(closure.get("launch_snapshot"), dict) or not isinstance(closure.get("job_inputs"), dict):
        raise MatrixError("execution requires a contemporaneous closure (launch manifest, snapshot, bindings, snapshot_fn)")
    launch = closure["launch_snapshot"]
    out_dir.mkdir(parents=True, exist_ok=False)
    receipt["closure_manifest"] = closure["closure_manifest"]
    receipt["closure_manifest_sha256"] = closure["closure_manifest_sha256"]
    try:
        pre = closure["snapshot_fn"]()
    except Exception as exc:  # noqa: BLE001
        return _abort(receipt, out_dir, log_path, "pre-run source closure could not be established", closure_error=str(exc))
    for section in (*CL.SNAPSHOT_SECTIONS, "runtime_source_closure_digest", "orchestration_digest"):
        receipt[section] = pre[section]
    receipt["runtime_source_closure_digest_pre"] = pre["runtime_source_closure_digest"]
    equal_pre = CL.sections_equal(pre, launch)
    receipt["closure_sections_pre_equal_launch"] = equal_pre
    receipt["closure_matches_launch_manifest_pre"] = all(equal_pre.values()) and pre["runtime_source_closure_digest"] == closure["runtime_source_closure_digest"] == launch.get("runtime_source_closure_digest") and pre["orchestration_digest"] == launch.get("orchestration_digest")
    if not receipt["closure_matches_launch_manifest_pre"]:
        return _abort(receipt, out_dir, log_path, f"pre-run source closure differs from the launch manifest ({equal_pre})", runtime_source_closure_digest_post=None)
    try:
        inputs_pre = CL.job_inputs_table(receipt)
        drv_pre = f1_driver_inputs_table()
    except Exception as exc:  # noqa: BLE001
        return _abort(receipt, out_dir, log_path, "pre-run job inputs could not be resolved", closure_error=str(exc), runtime_source_closure_digest_post=None, job_inputs_digest_pre=None, job_inputs_digest_post=None)
    receipt["job_inputs"] = inputs_pre["job_inputs"]
    receipt["job_inputs_digest_pre"] = inputs_pre["job_inputs_digest"]
    bound = closure["job_inputs"].get(receipt["job_id"])
    if not (inputs_pre["job_inputs_digest"] == receipt.get("job_inputs_digest") == bound) or drv_pre["f1_driver_inputs_digest"] != receipt.get("f1_driver_inputs_digest"):
        return _abort(receipt, out_dir, log_path, "pre-run job/driver inputs differ from the description/launch binding", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    binding = command_binding_problems(receipt, fingerprint=pre["environment_fingerprint"], require_fingerprint=True)
    receipt["command_binding_pre_launch"] = {"ok": not binding, "problems": binding}
    if binding:
        return _abort(receipt, out_dir, log_path, f"command binding failed: {json.dumps(binding, default=str)[:1500]}", runtime_source_closure_digest_post=None, job_inputs_digest_post=None)
    manifest_pre = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_pre_launch"] = {"ok": not manifest_pre, "problems": manifest_pre}
    if manifest_pre:
        return _abort(receipt, out_dir, log_path, "closure manifest on disk missing or tampered before launch", runtime_source_closure_digest_post=None, job_inputs_digest_post=None, closure_manifest_unchanged=False)
    receipt["started_at_utc"] = C.utc_now()
    t0 = time.time()
    with (out_dir / STDOUT_LOG).open("w", encoding="utf-8") as handle:
        proc = subprocess_run(receipt["command"], cwd=str(C.REPO), stdout=handle, stderr=subprocess.STDOUT, text=True, check=False)
    receipt["finished_at_utc"] = C.utc_now()
    receipt["duration_s"] = round(time.time() - t0, 1)
    receipt["returncode"] = proc.returncode
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
        inputs_post = CL.job_inputs_table(receipt)
        drv_post = f1_driver_inputs_table()
    except Exception as exc:  # noqa: BLE001
        inputs_post, drv_post, inputs_post_error = None, None, str(exc)
    receipt["job_inputs_digest_post"] = inputs_post["job_inputs_digest"] if inputs_post else None
    receipt["job_inputs_unchanged"] = bool(inputs_post) and inputs_post["job_inputs_digest"] == receipt["job_inputs_digest_pre"] and bool(drv_post) and drv_post["f1_driver_inputs_digest"] == receipt["f1_driver_inputs_digest"]
    receipt["closure_error"] = post_error or inputs_post_error
    manifest_post = CL.verify_manifest_file(closure["closure_manifest"], closure["closure_manifest_sha256"], reference=launch, job_id=receipt["job_id"], job_inputs_digest=receipt["job_inputs_digest"])
    receipt["closure_manifest_verified_post"] = {"ok": not manifest_post, "problems": manifest_post}
    receipt["closure_manifest_unchanged"] = not manifest_post
    digest = M.output_digest(out_dir)
    receipt.update(digest)
    rollout_status = M.status_from(proc.returncode, digest)
    adapter_ok = True
    if receipt.get("driver") == "f1_rollout_aiso":
        side = adapter_sidecar_digest(out_dir)
        receipt.update(side)
        adapter_ok = bool(side.get("adapter_ok")) and side.get("adapter_mode_recorded") == receipt.get("adapter_mode")
    receipt["status"] = rollout_status if (receipt["source_closure_unchanged"] and receipt["job_inputs_unchanged"] and receipt["closure_manifest_unchanged"] and adapter_ok) else "failed"
    receipt["status_reason"] = "ok" if receipt["status"] == "ok" else ("rollout failed" if rollout_status != "ok" else ("adapter side-car invalid" if not adapter_ok else "closure/inputs/manifest changed during the job"))
    try:
        receipt["stdout_tail"] = (out_dir / STDOUT_LOG).read_text(encoding="utf-8", errors="replace").splitlines()[-15:]
    except OSError:
        receipt["stdout_tail"] = []
    C.write_json(out_dir / RECEIPT_FILE, receipt)
    M.append_log(log_path, receipt)
    return receipt


def execute(described: list[dict[str, Any]], selected: list[dict[str, Any]], *, workers: int, log_path: Path, closure: dict[str, Any], runner: Callable[..., dict[str, Any]] = run_job) -> dict[str, Any]:
    pre = preflight_all(described)
    results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=max(1, int(workers))) as pool:
        futures = [pool.submit(runner, dict(rec), log_path=log_path, closure=closure) for rec in selected]
        for fut in futures:
            results.append(fut.result())
    return {"preflight": pre, "results": results}


# --- F0 reuse pins ----------------------------------------------------------------------------


def f0_reuse_pins() -> dict[str, Any]:
    """Verified pointers to the F0 job directories reused as controls/references."""
    pins: dict[str, Any] = {"f0_root": C.rel(F1.F0_OUT_ROOT), "f0_analysis": C.rel(F1.F0_ANALYSIS_JSON), "f0_analysis_sha256": C.sha256_file(F1.F0_ANALYSIS_JSON) if F1.F0_ANALYSIS_JSON.is_file() else None, "f0_analysis_sha256_expected": F1.F0_ANALYSIS_SHA256, "jobs": {}}
    pins["f0_analysis_pin_ok"] = pins["f0_analysis_sha256"] == F1.F0_ANALYSIS_SHA256
    for name, spec in F1.REUSE_F0.items():
        fams = spec["f0_family"] if isinstance(spec["f0_family"], tuple) else (spec["f0_family"],)
        entries = []
        for fam in fams:
            fam_dir = F1.F0_ROLLOUTS / fam
            if not fam_dir.is_dir():
                continue
            for d in sorted(fam_dir.iterdir()):
                if not d.is_dir() or not d.name.startswith(spec["f0_candidate"] + "__" + spec["f0_runtime"] + "__"):
                    continue
                rec_path = d / "f0_receipt.json"
                rec = C.read_json(rec_path) if rec_path.is_file() else {}
                digest = M.output_digest(d)
                entries.append({"job_dir": C.rel(d), "family": fam, "receipt_sha256": C.sha256_file(rec_path) if rec_path.is_file() else None, "receipt_status": rec.get("status"), "receipt_returncode": rec.get("returncode"), "summary_sha256": digest.get("summary_sha256"), "trace_sha256": digest.get("trace_sha256"), "summary_matches_receipt": digest.get("summary_sha256") == rec.get("summary_sha256"), "trace_matches_receipt": digest.get("trace_sha256") == rec.get("trace_sha256"), "seed": rec.get("seed"), "start": rec.get("start"), "action_selection": rec.get("action_selection")})
        pins["jobs"][name] = {"role": spec["role"], "comparison_class": spec["comparison_class"], "count": len(entries), "entries": entries, "all_ok": all(e["receipt_status"] == "ok" and e["summary_matches_receipt"] and e["trace_matches_receipt"] for e in entries) if entries else False}
    return pins


# --- dry-run manifest ----------------------------------------------------------------------------


def dirty_user_files_audit() -> dict[str, Any]:
    files = F1.load_protocol()["artefact_rules"]["dirty_user_files_immutable"]
    out = {}
    for rel_path in files:
        p = C.REPO / rel_path
        out[rel_path] = {"exists": p.is_file(), "sha256": C.sha256_file(p) if p.is_file() else None}
    out["training_exnovo_cfg_sha256_expected_from_F0"] = "092363e6bf5a411f0efa2e7d2c0d826c1f0739144cef8c5e540cbfb66df91d2f"
    out["training_exnovo_cfg_unchanged_since_F0"] = out["Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"]["sha256"] == out["training_exnovo_cfg_sha256_expected_from_F0"]
    return out


def stage_commands(python_exe: str) -> dict[str, list[list[str]]]:
    """Exact commands (no placeholders) for the S2 stages, in order."""
    py = python_exe
    b = F1.CANDIDATES["B"]["module"]
    v26 = F1.CANDIDATES["A_ISO39_V3"]["module"]
    b_stoch_dirs = [str(F1.OUT_ROLLOUTS / "b_stoch_native" / f"B__{F1.TARGET_RUNTIME}__{s}__stoch_seed{k}") for s in F1.STARTS for k in F1.DEVELOPMENT_SEEDS]
    aiso_traces = [str(F1.OUT_ROLLOUTS / "aiso_det" / f"A_ISO39_V3__{F1.TARGET_RUNTIME}__{s}__det" / ADAPTER_TRACE_FILE) for s in F1.STARTS]
    return {
        "stage_1_adapter_crosscheck": [[py, str(HERE / "f1_adapter_crosscheck.py")]],
        "stage_1_derive_modules": [
            [py, str(HERE / "f1_sigma_variant.py"), "--source-module", str(b), "--output-dir", str(DERIVED_MODULE_DIRS["C"].parent), "--sigma", "0.005", "--materialize"],
            [py, str(HERE / "f1_sigma_variant.py"), "--source-module", str(v26), "--output-dir", str(DERIVED_MODULE_DIRS["A_ISO39_V3_S005"].parent), "--sigma", "0.005", "--materialize"],
        ],
        "stage_1_rollouts": [[py, str(HERE / "f1_matrix.py"), "--execute", "--stage", "1", "--workers", "5"]],
        "stage_2_dataset": [[py, str(HERE / "f1_dataset.py"), *sum([["--b-job-dir", d] for d in b_stoch_dirs], []), *sum([["--aiso-adapter-trace", t] for t in aiso_traces], []), "--teacher-module", str(v26), "--out-dir", str(F1.OUT_DATASETS), "--stamp", DATASET_STAMP]],
        "stage_2_refit": [[py, str(HERE / "f1_refit.py"), "--source-module", str(b), "--dataset-npz", str(F1.OUT_DATASETS / f"f1_dataset_{DATASET_STAMP}.npz"), "--dataset-receipt", str(F1.OUT_DATASETS / f"f1_dataset_{DATASET_STAMP}.json"), "--out-dir", str(DERIVED_MODULE_DIRS["D"].parent), "--fit"]],
        "stage_3_rollouts": [[py, str(HERE / "f1_matrix.py"), "--execute", "--stage", "3", "--workers", "5"]],
        "stage_4_analysis": [[py, str(HERE / "f1_analysis.py"), "--manifest-root", str(F1.OUT_ROOT)]],
    }


def write_dry_run_manifest(described: list[dict[str, Any]], *, stamp: str, python_record: dict[str, Any]) -> dict[str, str]:
    F1.ensure_out_dirs()
    manifest = {
        "schema_version": 1,
        "mode": "dry_run",
        "dry_run": True,
        "stage_executed": 0,
        "stamp": stamp,
        "generated_at_utc": C.utc_now(),
        "protocol": {"id": "F1-S1-ablation-39to35-sigma-v1", **F1.protocol_digests()},
        "f1_scripts_sha256": {p: C.sha256_file(HERE / p) for p in F1_SCRIPTS if (HERE / p).is_file()},
        "f0_library_sha256": {p: C.sha256_file(F1.F0_DIR / p) for p in CL.ORCHESTRATION_FILES if (F1.F0_DIR / p).is_file()},
        "git": C.git_snapshot(),
        "interpreter": python_record,
        "runtime": {"name": F1.TARGET_RUNTIME, "config": C.rel(F1.RUNTIME_CONFIG), "config_sha256": C.sha256_file(F1.RUNTIME_CONFIG), "rollout_eval_sha256": C.sha256_file(F1.ROLLOUT_EVAL), "aiso_driver_sha256": C.sha256_file(F1.AISO_DRIVER)},
        "reconstruction_config_equality": F1.reconstruction_config_equality(),
        "insertion_spec": OA.derive_insertion(OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)[0], OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)[0], manifest35_sha256=C.ACTOR_MANIFEST_35_SHA256, manifest39_sha256=C.ACTOR_MANIFEST_39_SHA256).to_dict(),
        "candidates": {k: {kk: (C.rel(vv) if isinstance(vv, Path) else vv) for kk, vv in v.items()} for k, v in F1.CANDIDATES.items()},
        "derived_module_plan": {k: C.rel(v) for k, v in DERIVED_MODULE_DIRS.items()},
        "reuse_f0": f0_reuse_pins(),
        "dirty_user_files": dirty_user_files_audit(),
        "seeds": {"development": list(F1.DEVELOPMENT_SEEDS), "sealed_never_used": list(F1.SEALED_SEEDS), "used_in_matrix": sorted({int(r["seed"]) for r in described}), "sealed_seed_found": any(int(r["seed"]) in F1.SEALED_SEEDS for r in described)},
        "stages": {str(k): v for k, v in F1.STAGES.items()},
        "stage_commands": stage_commands(python_record.get("selected") or M.PYTHON_PLACEHOLDER),
        "job_count": len(described),
        "job_counts_by_family": {f: sum(1 for r in described if r["family"] == f) for f in F1.FAMILIES},
        "job_counts_by_stage": {str(s): sum(1 for r in described if r["stage"] == s) for s in sorted({r["stage"] for r in described})},
        "jobs_pending_module": [r["job_id"] for r in described if r["module_pending"]],
        "existing_output_dirs": [r["output_dir"] for r in described if r["output_dir_exists"]],
        "jobs": described,
    }
    if manifest["seeds"]["sealed_seed_found"]:
        raise MatrixError("sealed seed in matrix")
    if not manifest["reconstruction_config_equality"]["equal"]:
        raise MatrixError(f"reconstruction config keys differ: {manifest['reconstruction_config_equality']['mismatches']}")
    path = F1.OUT_MANIFEST / f"f1_job_matrix_dry_run_{stamp}.json"
    C.write_json(path, manifest)
    md = [f"# F1 job matrix — dry-run {stamp}", "", f"Protocol SHA-256: `{manifest['protocol']['f1_protocol.json']}` — runtime `{F1.TARGET_RUNTIME}` config `{manifest['runtime']['config_sha256'][:12]}…` — git HEAD `{manifest['git']['head'][:12]}`", "", f"Jobs: {len(described)} (pending module: {len(manifest['jobs_pending_module'])}); reconstruction config keys equal: {manifest['reconstruction_config_equality']['equal']}; sealed seed found: {manifest['seeds']['sealed_seed_found']}", ""]
    md.append(C.md_table(["job_id", "family", "stage", "candidate", "class", "start", "sel", "seed", "driver", "adapter", "module pending", "gate"], [[r["job_id"], r["family"], r["stage"], r["candidate"], r["comparison_class"], r["start"], r["action_selection"], r["seed"], r["driver"], r["adapter_mode"], r["module_pending"], r["gate_relevant"]] for r in described]))
    md.append("")
    md.append("## Stage commands (exact)")
    for stage, cmds in manifest["stage_commands"].items():
        md.append(f"\n### {stage}\n")
        for cmd in cmds:
            md.append("```\n" + " ".join(_quote(t) for t in cmd) + "\n```")
    md_path = F1.OUT_MANIFEST / f"f1_job_matrix_dry_run_{stamp}.md"
    C.write_text(md_path, "\n".join(md) + "\n")
    index = {"manifest": C.rel(path), "manifest_sha256": C.sha256_file(path), "markdown": C.rel(md_path), "markdown_sha256": C.sha256_file(md_path)}
    C.write_json(F1.OUT_MANIFEST / f"f1_job_matrix_dry_run_{stamp}.index.json", index)
    return index


def _quote(tok: str) -> str:
    return f'"{tok}"' if (" " in tok or "(" in tok) else tok


# --- CLI -----------------------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="F1 job matrix driver (dry-run by default).")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--stage", type=int, default=None, help="stage to execute (1 or 3); required with --execute")
    parser.add_argument("--family", action="append", default=None)
    parser.add_argument("--only", default=None, help="regex on job_id")
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args(argv)
    specs = build_jobs()
    if args.list:
        for s in specs:
            print(s["stage"], s["family"], s["job_id"])
        return 0
    python_record: dict[str, Any]
    try:
        python_record = C.select_python()
    except RuntimeError as exc:
        if args.execute:
            raise
        python_record = {"selected": None, "error": str(exc)[:400]}
    python_exe = python_record.get("selected")
    described = [describe_job(s, python_exe) for s in specs]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if not args.execute:
        index = write_dry_run_manifest(described, stamp=stamp, python_record=python_record)
        print(json.dumps({"mode": "dry_run", "jobs": len(described), **index}, indent=2))
        return 0
    if args.stage not in (1, 3):
        raise SystemExit("--execute requires --stage 1 or --stage 3")
    selected = [r for r in described if r["stage"] == args.stage and (not args.family or r["family"] in args.family) and (not args.only or re.search(args.only, r["job_id"]))]
    pending = [r["job_id"] for r in selected if r["module_pending"]]
    if pending:
        raise SystemExit(f"cannot execute: modules pending for {pending}")
    F1.ensure_out_dirs()
    orch = tuple(str(F1.F0_DIR / p) for p in CL.ORCHESTRATION_FILES) + tuple(str(HERE / p) for p in F1_SCRIPTS if (HERE / p).is_file())
    snapshot_fn = lambda: CL.closure_snapshot(python_exe, orchestration_files=orch)  # noqa: E731
    snapshot = snapshot_fn()
    bindings = {r["job_id"]: r["job_inputs_digest"] for r in selected}
    written = CL.write_closure_manifest(F1.OUT_ROLLOUTS / f"source_closure_manifest_{stamp}.json", snapshot, job_inputs=bindings)
    closure = {**written, "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"], "launch_snapshot": snapshot, "job_inputs": bindings, "snapshot_fn": snapshot_fn}
    launch_manifest = {"mode": "execute", "stage": args.stage, "stamp": stamp, "selected": [r["job_id"] for r in selected], "closure_manifest": written, "git": C.git_snapshot(), "interpreter": python_record}
    C.write_json(F1.OUT_ROLLOUTS / f"f1_job_matrix_execute_{stamp}.json", launch_manifest)
    t0 = time.time()
    outcome = execute(described, selected, workers=args.workers, log_path=F1.OUT_ROLLOUTS / "driver_log.jsonl", closure=closure)
    status = {"generated_at_utc": C.utc_now(), "stage": args.stage, "elapsed_s": round(time.time() - t0, 1), "preflight": outcome["preflight"], "job_count": len(selected), "ok": sum(1 for r in outcome["results"] if r.get("status") == "ok"), "skipped_verified_identical": sum(1 for r in outcome["results"] if r.get("status") == "skipped_verified_identical"), "failed": sum(1 for r in outcome["results"] if r.get("status") == "failed"), "results": sorted(outcome["results"], key=lambda r: r["job_id"])}
    C.write_json(F1.OUT_ROLLOUTS / f"f1_job_matrix_status_{stamp}.json", status)
    print(json.dumps({k: status[k] for k in ("stage", "job_count", "ok", "skipped_verified_identical", "failed", "elapsed_s")}, indent=2))
    return 0 if status["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
