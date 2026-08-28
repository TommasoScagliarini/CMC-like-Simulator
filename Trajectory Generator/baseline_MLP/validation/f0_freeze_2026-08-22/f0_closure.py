"""Source/runtime closure digests for the F0 rollout matrix (provenance).

Three layers are kept apart on purpose:

  runtime_source_closure_digest - SHA-256 over the (path, sha256) table of the
      DECLARED runtime-core sources (root simulator, Trajectory Generator env /
      FSM / adapters, baseline_MLP rollout stack), the native plugin binaries
      found under ``plugins/`` and the global data assets (setup XML, GRF /
      detector / morphology profiles). Shared by every job of a launch.
  job_inputs_digest - per-job scientific inputs: the pinned config YAML, every
      file it references (setup XML and, through it, model .osim, IK .mot,
      ExternalForces.xml and its GRF .mot datafile, CMC_Actuators.xml; GRF /
      detector / morphology profiles), EVERY regular file of the RLModule
      directory (module_state.pkl, class_and_ctor_args.pkl, metadata.json, ...
      i.e. all bytes RLModule.from_checkpoint may use), rollout_eval.py and the
      historical reference summary when one is declared.
  orchestration_digest - the F0 orchestration scripts in this directory.

Interpreter fingerprint: the EXACT rollout interpreter is probed through a
subprocess (argv list, no shell) that reports its own executable, version,
platform and the versions of torch / ray / opensim / numpy / yaml; the probe
fails closed when the executable differs from the requested one or a required
module is missing, so the fingerprint bound to receipts is the rollout
interpreter's, never the driver's.

The closure is DECLARED, not import-traced, and nothing here proves what a
process actually loaded; the strongest class reachable is
``B_plus_contemporaneous`` (all digests recomputed immediately before and after
each job), never Class A. Every file of the closure must be a regular file
(symlinks rejected, fail-closed); paths are repository-relative POSIX; nothing
is written except through ``write_closure_manifest`` (no-clobber).
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402

CLOSURE_SCHEMA_VERSION = 2
PROVENANCE_CLASS_LEGACY = "B"
PROVENANCE_CLASS_CONTEMPORANEOUS = "B_plus_contemporaneous"
PROVENANCE_LIMITATION = (
    "Class B+: runtime source closure, per-job scientific inputs and interpreter fingerprint are recomputed "
    "immediately before and after the job (contemporaneous correlation), but the job still executes from the live "
    "working tree, not from a physically immutable bundle; no proof against modifications during the run between "
    "the two digests, no per-process proof of the native plugin bytes actually mapped, no complete imported-module / "
    "third-party dependency closure. This is NOT Class A and no bit-exact claim follows from it."
)

# Declared runtime-core closure (repository-relative, POSIX). Reviewable list, not import-traced.
RUNTIME_CORE_FILES: tuple[str, ...] = (
    "main.py",
    "config.py",
    "model_loader.py",
    "kinematics_interpolator.py",
    "outer_loop.py",
    "inverse_dynamics.py",
    "static_optimization.py",
    "prosthesis_controller.py",
    "output.py",
    "simulation_runner.py",
    "online_grf.py",
    "binary_phase_detector.py",
    "path_resolver.py",
    "setup_io.py",
    "Trajectory Generator/osim_trj_cmc_like.py",
    "Trajectory Generator/prosthetic_phase_fsm.py",
    "Trajectory Generator/binary_phase_adapter.py",
    "Trajectory Generator/binary_phase_adapter_v26.py",
    "Trajectory Generator/binary_phase_fsm.py",
    "Trajectory Generator/binary_phase_fsm_v26.py",
    "Trajectory Generator/baseline_MLP/__init__.py",
    "Trajectory Generator/baseline_MLP/_bootstrap.py",
    "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "Trajectory Generator/baseline_MLP/env_factory.py",
    "Trajectory Generator/baseline_MLP/training_config.py",
    "Trajectory Generator/baseline_MLP/reward_function.py",
    "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py",
    "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
    "Trajectory Generator/baseline_MLP/exploration_noise.py",
    "Trajectory Generator/baseline_MLP/start_condition_metrics.py",
    "Trajectory Generator/baseline_MLP/start_sampling.py",
    "Trajectory Generator/baseline_MLP/process_watchdog.py",
    "Trajectory Generator/baseline_MLP/progress_display.py",
    "Trajectory Generator/baseline_MLP/primary_grf_split_adaptation.py",
    "Trajectory Generator/baseline_MLP/primary_split_input_adapter.py",
    "Trajectory Generator/baseline_MLP/primary_split_v25_residual.py",
    "Trajectory Generator/baseline_MLP/win_runtime.py",
)
DATA_ASSET_FILES: tuple[Path, ...] = (
    C.SETUP_XML,
    C.GRF_PROFILE_TANGENT_V2,
    C.GRF_PROFILE_CORRECT_MAGNITUDE,
    C.GRF_DETECTOR_PROFILE,
    C.BINARY_DETECTOR_PROFILE,
    C.MORPH_PROFILE_EVENT_WARPED,
    C.MORPH_PROFILE_LEGACY,
)
ORCHESTRATION_FILES: tuple[str, ...] = (
    "f0_common.py",
    "f0_closure.py",
    "f0_rollout_matrix.py",
    "f0_replay_analysis.py",
    "f0_matrix_analysis.py",
    "f0_source_closure_assessment.py",
    "f0_overlays.py",
    "f0_actor_drift.py",
    "f0_artifacts.py",
)
NATIVE_PLUGIN_DIR = C.REPO / "plugins"
NATIVE_SUFFIXES = (".dylib", ".dll", ".so")
REFERENCED_PLUGIN_STEMS = ("SEA_Plugin_BlackBox_mCMC_impedence_ff", "OnlineGRFContact")
# dotted config keys whose string value names a runtime file (resolved fail-closed when present)
CONFIG_FILE_KEYS = ("simulation.setup_xml", "grf.online_grf_profile", "grf.online_grf_detector_profile", "grf.binary_phase_detector_profile", "reward.morphology_profile")
SETUP_XML_FILE_TAGS = ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml")
PROBE_MODULES = ("torch", "ray", "opensim", "numpy", "yaml")
PROBE_TIMEOUT_S = 600
IRRECOVERABLE_LIMITATIONS: tuple[str, ...] = (
    "no proof against transient modifications of sources or plugins between the recorded digests and the actual load time of each process",
    "no per-process proof of which native plugin bytes (dylib/dll) were mapped by a given rollout process",
    "no complete imported-module closure nor third-party/native dependency closure (torch, ray, opensim, system libraries)",
    "no per-job pre/post closure for the jobs executed under receipt schema 4 (digests were not recomputed around each job)",
)
SNAPSHOT_SECTIONS = ("runtime_core", "native_plugins", "data_assets", "orchestration", "environment_fingerprint")


class ClosureError(RuntimeError):
    """Raised when the closure cannot be established fail-closed (missing file, symlink, unreadable, bad probe)."""


def _is_hex_digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(ch in "0123456789abcdef" for ch in value)


def canonical(obj: Any) -> str:
    """Canonical JSON (sorted keys, compact separators) for deep equality and digests."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)


def canonical_sha256(obj: Any) -> str:
    return hashlib.sha256(canonical(obj).encode("utf-8")).hexdigest()


def regular_file(path: Path, label: str) -> Path:
    """A closure file must exist, be a regular file and not a symlink (any component)."""
    path = Path(path)
    if path.is_symlink():
        raise ClosureError(f"{label}: symlink not allowed in the closure: {C.rel(path)}")
    if not path.is_file():
        raise ClosureError(f"{label}: missing closure file: {C.rel(path)}")
    for parent in Path(os.path.abspath(path)).parents:
        if parent.is_symlink():
            raise ClosureError(f"{label}: symlinked directory in the closure path: {C.rel(parent)}")
    return path


def digest_table(paths: list[Path], *, label: str) -> list[dict[str, Any]]:
    """(path, sha256, size) table; every entry must be an existing regular file and not a symlink."""
    table: list[dict[str, Any]] = []
    seen: set[str] = set()
    for path in paths:
        path = regular_file(Path(path), label)
        key = C.rel(path)
        if key in seen:
            continue
        seen.add(key)
        table.append({"path": key, "sha256": C.sha256_file(path), "bytes": path.stat().st_size})
    return sorted(table, key=lambda row: row["path"])


def table_digest(*tables: list[dict[str, Any]]) -> str:
    rows = sorted((row["path"], row["sha256"]) for table in tables for row in table)
    digest = hashlib.sha256()
    for path, sha in rows:
        digest.update(f"{path}\0{sha}\n".encode("utf-8"))
    return digest.hexdigest()


# --- native plugins ------------------------------------------------------------------


def native_plugin_files(plugin_dir: Path = NATIVE_PLUGIN_DIR) -> list[Path]:
    if plugin_dir.is_symlink():
        raise ClosureError(f"plugin directory is a symlink: {C.rel(plugin_dir)}")
    if not plugin_dir.is_dir():
        raise ClosureError(f"plugin directory missing: {C.rel(plugin_dir)}")
    files = []
    for entry in sorted(plugin_dir.iterdir()):
        if entry.is_symlink():
            raise ClosureError(f"symlink not allowed among native plugins: {C.rel(entry)}")
        name = entry.name
        if entry.is_file() and any(name.endswith(sfx) or f"{sfx}." in name for sfx in NATIVE_SUFFIXES):
            files.append(entry)
    if not files:
        raise ClosureError(f"no native plugin binaries under {C.rel(plugin_dir)}")
    return files


def native_plugin_table(plugin_dir: Path = NATIVE_PLUGIN_DIR) -> list[dict[str, Any]]:
    table = digest_table(native_plugin_files(plugin_dir), label="native plugins")
    for row in table:
        stem = Path(row["path"]).name
        row["referenced_by_config"] = any(stem.startswith(ref) or stem.startswith("lib" + ref) for ref in REFERENCED_PLUGIN_STEMS)
        row["loaded_in_process_proven"] = False
    return table


# --- interpreter fingerprint (exact rollout interpreter, subprocess probe, no shell) ----------

_PROBE_CODE = (
    "import importlib, json, os, platform, sys\n"
    f"mods = {list(PROBE_MODULES)!r}\n"
    "out = {'executable': sys.executable, 'executable_realpath': os.path.realpath(sys.executable), 'python_version': sys.version.split()[0], "
    "'platform': platform.platform(), 'machine': platform.machine(), 'modules': {}}\n"
    "for m in mods:\n"
    "    try:\n"
    "        mod = importlib.import_module(m)\n"
    "        out['modules'][m] = str(getattr(mod, '__version__', 'ok'))\n"
    "    except Exception as exc:\n"
    "        out['modules'][m] = 'MISSING: ' + type(exc).__name__\n"
    "print('F0PROBE ' + json.dumps(out))\n"
)


def interpreter_fingerprint(python_exe: str | None = None) -> dict[str, Any]:
    """Probe the exact interpreter that runs the rollouts (argv list, no shell).

    Fails closed when the executable is not a regular executable file, the probe
    does not return its JSON line, the probed executable differs from the
    requested one (realpath) or a required module (torch, ray, opensim, numpy,
    yaml) is missing. With ``python_exe=None`` the current interpreter is probed
    and labelled ``driver_current_interpreter``."""
    role = "rollout_interpreter" if python_exe else "driver_current_interpreter"
    requested = Path(python_exe or sys.executable)
    if requested.is_symlink() and not requested.resolve().is_file():
        raise ClosureError(f"interpreter symlink does not resolve to a file: {requested}")
    if not requested.is_file() or not os.access(requested, os.X_OK):
        raise ClosureError(f"interpreter is not an executable file: {requested}")
    try:
        proc = subprocess.run([str(requested), "-c", _PROBE_CODE], capture_output=True, text=True, timeout=PROBE_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ClosureError(f"interpreter probe failed for {requested}: {exc}") from exc
    line = next((ln for ln in reversed(proc.stdout.splitlines()) if ln.startswith("F0PROBE ")), None)
    if proc.returncode != 0 or line is None:
        raise ClosureError(f"interpreter probe returned no JSON (rc={proc.returncode}) for {requested}: {proc.stderr[-400:]!r}")
    try:
        payload = json.loads(line[len("F0PROBE "):])
    except ValueError as exc:
        raise ClosureError(f"interpreter probe JSON unreadable for {requested}: {exc}") from exc
    if os.path.realpath(str(payload.get("executable_realpath", ""))) != os.path.realpath(str(requested)):
        raise ClosureError(f"interpreter probe executable mismatch: requested {requested} -> probed {payload.get('executable')}")
    missing = [m for m, v in payload["modules"].items() if str(v).startswith("MISSING")]
    if missing:
        raise ClosureError(f"rollout interpreter {requested} lacks required modules {missing}: {payload['modules']}")
    stable = {"executable_realpath": payload["executable_realpath"], "python_version": payload["python_version"], "platform": payload["platform"], "machine": payload["machine"], "modules": payload["modules"]}
    return {
        "role": role,
        "requested_executable": str(requested),
        **payload,
        "fingerprint_sha256": canonical_sha256(stable),
        "note": "probed in a subprocess of the exact interpreter (argv list, no shell); versions are those the rollout subprocess imports",
    }


# --- per-job scientific inputs -----------------------------------------------------------------


def config_path_roots() -> tuple[Path, ...]:
    """Roots for relative config/setup paths, resolved at call time: the repository
    root, then Trajectory Generator/baseline_MLP (training configs, morphology profiles)."""
    return (C.REPO, C.BASELINE_DIR)


def _repo_path(value: str, label: str) -> Path:
    """Resolve a config/setup path: absolute as is; relative against the known
    roots. Exactly one root must hold the file (ambiguity or absence fails
    closed); symlinks rejected."""
    text = str(value).replace("\\", "/")
    path = Path(text)
    if path.is_absolute():
        return regular_file(path, label)
    roots = config_path_roots()
    hits = [root / path for root in roots if (root / path).is_file() or (root / path).is_symlink()]
    if len(hits) > 1 and len({C.sha256_file(h) for h in hits if h.is_file() and not h.is_symlink()}) != 1:
        raise ClosureError(f"{label}: ambiguous relative path {text!r} found under several roots with different content: {[C.rel(h) for h in hits]}")
    if not hits:
        raise ClosureError(f"{label}: missing closure file: {text!r} (searched under {[C.rel(r) or '.' for r in roots]})")
    return regular_file(hits[0], label)


def setup_runtime_files(setup_xml: Path) -> list[Path]:
    """Files the simulator resolves from the setup XML: model .osim, IK .mot,
    ExternalForces.xml (+ its GRF .mot datafile), CMC_Actuators.xml. Fail-closed."""
    setup_xml = regular_file(Path(setup_xml), "setup xml")
    try:
        root = ET.parse(setup_xml).getroot()
    except ET.ParseError as exc:
        raise ClosureError(f"setup xml unreadable: {C.rel(setup_xml)}: {exc}") from exc
    files = [setup_xml]
    for tag in SETUP_XML_FILE_TAGS:
        node = root.find(f".//{tag}")
        text = (node.text or "").strip() if node is not None else ""
        if not text:
            raise ClosureError(f"setup xml {C.rel(setup_xml)} lacks <{tag}>")
        path = _repo_path(text, f"setup <{tag}>")
        files.append(path)
        if tag == "external_loads_xml":
            try:
                ext = ET.parse(path).getroot()
            except ET.ParseError as exc:
                raise ClosureError(f"external loads xml unreadable: {C.rel(path)}: {exc}") from exc
            datafile = ext.find(".//datafile")
            data_text = (datafile.text or "").strip() if datafile is not None else ""
            if not data_text:
                raise ClosureError(f"external loads xml {C.rel(path)} lacks <datafile>")
            data_path = Path(data_text.replace("\\", "/"))
            if not data_path.is_absolute():
                data_path = path.parent / data_path
            files.append(regular_file(data_path, "external loads <datafile>"))
    return files


def config_referenced_files(cfg: dict[str, Any]) -> list[Path]:
    flat = C.flatten(cfg)
    files: list[Path] = []
    for key in CONFIG_FILE_KEYS:
        value = flat.get(key)
        if value is None or (isinstance(value, str) and not value.strip()):
            continue  # absent or explicitly empty ('' = no profile in the resolved yamls), nothing to resolve
        if not isinstance(value, str):
            raise ClosureError(f"config key {key} is not a path string: {value!r}")
        path = _repo_path(value, f"config {key}")
        if key == "simulation.setup_xml":
            files.extend(setup_runtime_files(path))
        else:
            files.append(path)
    if "simulation.setup_xml" not in flat:
        raise ClosureError("config lacks simulation.setup_xml (cannot resolve model/IK/GRF/actuator files)")
    return files


def module_files(module_dir: Path) -> list[Path]:
    """Every regular file of the RLModule directory (recursively); symlinks rejected; must be non-empty."""
    module_dir = Path(module_dir)
    if module_dir.is_symlink() or not module_dir.is_dir():
        raise ClosureError(f"module directory missing or symlink: {C.rel(module_dir)}")
    files: list[Path] = []
    for entry in sorted(module_dir.rglob("*")):
        if entry.is_symlink():
            raise ClosureError(f"symlink not allowed inside the module directory: {C.rel(entry)}")
        if entry.is_file():
            files.append(entry)
    if not files:
        raise ClosureError(f"module directory has no regular files: {C.rel(module_dir)}")
    return files


def job_inputs_table(receipt: dict[str, Any], *, rollout_eval: Path | None = None) -> dict[str, Any]:
    """Per-job scientific inputs with their digests (fail-closed resolution)."""
    rollout_eval = Path(rollout_eval) if rollout_eval is not None else C.ROLLOUT_EVAL  # resolved at call time
    config_rel = receipt.get("config")
    module_rel = receipt.get("module")
    if not isinstance(config_rel, str) or not config_rel or not isinstance(module_rel, str) or not module_rel:
        raise ClosureError("receipt lacks config/module paths")
    config_path = _repo_path(config_rel, "pinned config")
    try:
        cfg = C.load_yaml(config_path)
    except Exception as exc:  # noqa: BLE001 - a corrupted/tampered YAML must fail closed, never crash the driver
        raise ClosureError(f"pinned config unreadable ({type(exc).__name__}): {config_rel}") from exc
    if not isinstance(cfg, dict):
        raise ClosureError(f"pinned config is not a mapping: {config_rel}")
    paths = [config_path, *config_referenced_files(cfg), *module_files(C.REPO / module_rel.replace("\\", "/")), regular_file(rollout_eval, "rollout_eval")]
    hist = receipt.get("historical_reference_summary")
    if isinstance(hist, str) and hist:
        paths.append(_repo_path(hist, "historical reference summary"))
    table = digest_table(paths, label="job inputs")
    return {"job_inputs": table, "job_inputs_digest": table_digest(table), "job_inputs_count": len(table)}


# --- snapshot ---------------------------------------------------------------------------


def closure_snapshot(python_exe: str | None = None, *, plugin_dir: Path = NATIVE_PLUGIN_DIR, runtime_files: tuple[str, ...] = RUNTIME_CORE_FILES, data_assets: tuple[Path, ...] = DATA_ASSET_FILES, orchestration_files: tuple[str, ...] = ORCHESTRATION_FILES, fingerprint: dict[str, Any] | None = None) -> dict[str, Any]:
    """Current closure digests (fail-closed on any missing file, symlink or bad interpreter probe).

    ``fingerprint`` may pass an already probed interpreter fingerprint (the probe
    is expensive); otherwise the exact ``python_exe`` (or the current
    interpreter) is probed now."""
    runtime = digest_table([C.REPO / p for p in runtime_files], label="runtime core")
    plugins = native_plugin_table(plugin_dir)
    assets = digest_table(list(data_assets), label="data assets")
    orchestration = digest_table([HERE / p for p in orchestration_files], label="orchestration")
    env = fingerprint if fingerprint is not None else interpreter_fingerprint(python_exe)
    return {
        "closure_schema_version": CLOSURE_SCHEMA_VERSION,
        "computed_at_utc": C.utc_now(),
        "runtime_source_closure_digest": table_digest(runtime, plugins, assets),
        "runtime_core": runtime,
        "native_plugins": plugins,
        "data_assets": assets,
        "orchestration_digest": table_digest(orchestration),
        "orchestration": orchestration,
        "environment_fingerprint": env,
        "git": C.git_snapshot(),
        "closure_declared_not_import_traced": True,
        "provenance_limitation": PROVENANCE_LIMITATION,
        "irrecoverable_limitations": list(IRRECOVERABLE_LIMITATIONS),
    }


def sections_equal(a: dict[str, Any], b: dict[str, Any]) -> dict[str, bool]:
    """Canonical deep equality of every closure section between two snapshots/receipts."""
    return {section: canonical(a.get(section)) == canonical(b.get(section)) for section in SNAPSHOT_SECTIONS}


def compact(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Digest-level summary of a snapshot (manifest headers)."""
    return {
        "runtime_source_closure_digest": snapshot["runtime_source_closure_digest"],
        "orchestration_digest": snapshot["orchestration_digest"],
        "native_plugins": [{"path": r["path"], "sha256": r["sha256"], "referenced_by_config": r["referenced_by_config"]} for r in snapshot["native_plugins"]],
        "environment_fingerprint_sha256": snapshot["environment_fingerprint"]["fingerprint_sha256"],
        "interpreter_role": snapshot["environment_fingerprint"].get("role"),
        "runtime_core_file_count": len(snapshot["runtime_core"]),
        "data_asset_count": len(snapshot["data_assets"]),
    }


def write_closure_manifest(path: Path, snapshot: dict[str, Any], *, job_inputs: dict[str, str] | None = None) -> dict[str, Any]:
    """No-clobber write of the full closure manifest (snapshot + per-job input
    digests bound at launch); returns its POSIX path and SHA-256."""
    payload = dict(snapshot)
    payload["closure_schema_version"] = CLOSURE_SCHEMA_VERSION
    payload["job_inputs"] = dict(job_inputs or {})
    written = C.write_json(path, payload)
    return {"closure_manifest": C.rel(written), "closure_manifest_sha256": C.sha256_file(written)}


def _table_ok(table: Any) -> bool:
    return isinstance(table, list) and bool(table) and all(isinstance(r, dict) and _is_hex_digest(r.get("sha256")) and isinstance(r.get("path"), str) and r.get("path") for r in table)


def verify_manifest_file(manifest_rel: Any, manifest_sha: Any, *, reference: dict[str, Any], job_id: Any, job_inputs_digest: Any) -> dict[str, Any]:
    """Single validator of the closure manifest ON DISK (used immediately before and
    after each subprocess by the driver, and by verify_closure_fields).

    Checks, all fail-closed (mismatch mapping, empty == valid): path is an
    existing regular file with no symlink in its components; SHA-256 exactly
    ``manifest_sha``; JSON mapping with ``closure_schema_version`` ==
    CLOSURE_SCHEMA_VERSION; the four tables well-formed and the fingerprint
    present; runtime and orchestration digests recomputed from the manifest
    tables equal to the stored ones; every section canonically equal to
    ``reference`` (the launch snapshot, or the receipt) and the digests equal to
    the reference's; ``job_inputs[job_id]`` == ``job_inputs_digest``."""
    problems: dict[str, Any] = {}
    if not isinstance(manifest_rel, str) or not manifest_rel or not _is_hex_digest(manifest_sha):
        problems["closure_manifest"] = {"existing": (manifest_rel, manifest_sha), "expected": "path + 64-hex sha256"}
        return problems
    path = C.REPO / manifest_rel.replace("\\", "/")
    try:
        regular_file(path, "closure manifest")
    except ClosureError as exc:
        problems["closure_manifest_file"] = {"existing": str(exc), "expected": "existing regular file, no symlink"}
        return problems
    on_disk = C.sha256_file(path)
    if on_disk != manifest_sha:
        problems["closure_manifest_sha256"] = {"expected": manifest_sha, "recomputed_from_disk": on_disk}
        return problems
    try:
        manifest = C.read_json(path)
    except ValueError as exc:
        problems["closure_manifest_content"] = {"existing": f"unreadable JSON: {exc}", "expected": "mapping"}
        return problems
    if not isinstance(manifest, dict):
        problems["closure_manifest_content"] = {"existing": type(manifest).__name__, "expected": "mapping"}
        return problems
    if manifest.get("closure_schema_version") != CLOSURE_SCHEMA_VERSION:
        problems["closure_manifest_schema"] = {"existing": manifest.get("closure_schema_version"), "expected": CLOSURE_SCHEMA_VERSION}
    tables_ok = True
    for section in ("runtime_core", "native_plugins", "data_assets", "orchestration"):
        if not _table_ok(manifest.get(section)):
            problems[f"closure_manifest_table_{section}"] = {"existing": "missing/empty/malformed", "expected": "non-empty list of {path, sha256}"}
            tables_ok = False
    env = manifest.get("environment_fingerprint")
    if not isinstance(env, dict) or not _is_hex_digest(env.get("fingerprint_sha256")):
        problems["closure_manifest_table_environment_fingerprint"] = {"existing": "missing/malformed", "expected": "probe payload with fingerprint_sha256"}
    if tables_ok:
        runtime_recomputed = table_digest(manifest["runtime_core"], manifest["native_plugins"], manifest["data_assets"])
        if not (runtime_recomputed == manifest.get("runtime_source_closure_digest") == reference.get("runtime_source_closure_digest")):
            problems["closure_manifest_runtime_digest"] = {"recomputed_from_manifest": runtime_recomputed, "manifest": manifest.get("runtime_source_closure_digest"), "reference": reference.get("runtime_source_closure_digest")}
        orch_recomputed = table_digest(manifest["orchestration"])
        if not (orch_recomputed == manifest.get("orchestration_digest") == reference.get("orchestration_digest")):
            problems["closure_manifest_orchestration_digest"] = {"recomputed_from_manifest": orch_recomputed, "manifest": manifest.get("orchestration_digest"), "reference": reference.get("orchestration_digest")}
    for section, equal in sections_equal(manifest, reference).items():
        if not equal:
            problems[f"closure_manifest_section_{section}"] = {"existing": "manifest differs from the reference (canonical deep equality)", "expected": "identical"}
    bindings = manifest.get("job_inputs")
    bound = bindings.get(job_id) if isinstance(bindings, dict) and isinstance(job_id, str) else None
    if not _is_hex_digest(job_inputs_digest) or bound != job_inputs_digest:
        problems["closure_manifest_job_inputs_binding"] = {"manifest": bound, "job": job_inputs_digest, "job_id": job_id}
    return problems


# --- receipt verification (schema 5) -----------------------------------------------------------


def verify_closure_fields(receipt: dict[str, Any]) -> dict[str, Any]:
    """Integral consistency of the contemporaneous closure stored in a schema-5 receipt.

    Returns the mismatch mapping (empty when consistent). Checks, all fail-closed:
    digests well-formed; runtime digest recomputed from the receipt tables
    (runtime_core + native_plugins + data_assets) == pre == post; orchestration
    digest recomputed from the receipt table == stored; every section
    (runtime_core, native_plugins, data_assets, orchestration,
    environment_fingerprint) canonically EQUAL to the closure manifest on disk
    (manifest SHA-256 re-verified); job inputs table present with digest
    recomputed == pre == post and bound in the manifest under the job id;
    source_closure_unchanged / job_inputs_unchanged True; provenance class
    B_plus_contemporaneous with explicit non-Class-A limitation."""
    problems: dict[str, Any] = {}
    pre = receipt.get("runtime_source_closure_digest_pre")
    post = receipt.get("runtime_source_closure_digest_post")
    for name in ("runtime_source_closure_digest_pre", "runtime_source_closure_digest_post", "orchestration_digest", "job_inputs_digest_pre", "job_inputs_digest_post"):
        if not _is_hex_digest(receipt.get(name)):
            problems[name] = {"existing": receipt.get(name), "expected": "64-hex sha256"}
    if _is_hex_digest(pre) and _is_hex_digest(post) and pre != post:
        problems["runtime_source_closure_digest_pre_vs_post"] = {"pre": pre, "post": post}
    for flag in ("source_closure_unchanged", "job_inputs_unchanged"):
        if receipt.get(flag) is not True:
            problems[flag] = {"existing": receipt.get(flag), "expected": True}
    tables_ok = True
    for section in ("runtime_core", "native_plugins", "data_assets", "orchestration", "job_inputs"):
        table = receipt.get(section)
        if not isinstance(table, list) or not table or not all(isinstance(r, dict) and _is_hex_digest(r.get("sha256")) and isinstance(r.get("path"), str) and r.get("path") for r in table):
            problems[section] = {"existing": "missing/empty/malformed", "expected": "non-empty list of {path, sha256}"}
            tables_ok = False
    env = receipt.get("environment_fingerprint")
    if not isinstance(env, dict) or not _is_hex_digest(env.get("fingerprint_sha256")) or env.get("role") != "rollout_interpreter" or not isinstance(env.get("modules"), dict):
        problems["environment_fingerprint"] = {"existing": "missing/malformed or not the rollout interpreter", "expected": "probe payload with role rollout_interpreter"}
    else:
        # the fingerprint must be the probe of the interpreter named by the receipt (python field == command[0], bound by the driver)
        python = receipt.get("python")
        requested = env.get("requested_executable")
        probed = env.get("executable_realpath")
        if not isinstance(python, str) or not isinstance(requested, str) or not isinstance(probed, str) or not (os.path.realpath(python) == os.path.realpath(requested) == probed):
            problems["environment_fingerprint_executable"] = {"python_field_realpath": os.path.realpath(python) if isinstance(python, str) else python, "fingerprint_requested_realpath": os.path.realpath(requested) if isinstance(requested, str) else requested, "fingerprint_executable_realpath": probed}
    if tables_ok:
        runtime_recomputed = table_digest(receipt["runtime_core"], receipt["native_plugins"], receipt["data_assets"])
        if runtime_recomputed != pre:
            problems["runtime_source_closure_digest_recomputed"] = {"from_receipt_tables": runtime_recomputed, "receipt_pre": pre}
        orch_recomputed = table_digest(receipt["orchestration"])
        if orch_recomputed != receipt.get("orchestration_digest"):
            problems["orchestration_digest_recomputed"] = {"from_receipt_table": orch_recomputed, "receipt": receipt.get("orchestration_digest")}
        inputs_recomputed = table_digest(receipt["job_inputs"])
        if not (inputs_recomputed == receipt.get("job_inputs_digest_pre") == receipt.get("job_inputs_digest_post")):
            problems["job_inputs_digest_recomputed"] = {"from_receipt_table": inputs_recomputed, "pre": receipt.get("job_inputs_digest_pre"), "post": receipt.get("job_inputs_digest_post")}
    if receipt.get("provenance_class") != PROVENANCE_CLASS_CONTEMPORANEOUS:
        problems["provenance_class"] = {"existing": receipt.get("provenance_class"), "expected": PROVENANCE_CLASS_CONTEMPORANEOUS}
    if not isinstance(receipt.get("provenance_limitation"), str) or "NOT Class A" not in receipt.get("provenance_limitation", ""):
        problems["provenance_limitation"] = {"existing": receipt.get("provenance_limitation"), "expected": "explicit non-Class-A limitation statement"}
    # the manifest on disk, through the single validator, with the receipt as reference (its sections == pre snapshot)
    reference = {**{s: receipt.get(s) for s in SNAPSHOT_SECTIONS}, "runtime_source_closure_digest": pre, "orchestration_digest": receipt.get("orchestration_digest")}
    problems.update(verify_manifest_file(receipt.get("closure_manifest"), receipt.get("closure_manifest_sha256"), reference=reference, job_id=receipt.get("job_id"), job_inputs_digest=receipt.get("job_inputs_digest_pre")))
    for flag in ("closure_manifest_verified_pre_launch", "closure_manifest_verified_post"):
        record = receipt.get(flag)
        if not isinstance(record, dict) or record.get("ok") is not True:
            problems[flag] = {"existing": record, "expected": "{ok: true} recorded by the driver"}
    if receipt.get("closure_manifest_unchanged") is not True:
        problems["closure_manifest_unchanged"] = {"existing": receipt.get("closure_manifest_unchanged"), "expected": True}
    return problems
