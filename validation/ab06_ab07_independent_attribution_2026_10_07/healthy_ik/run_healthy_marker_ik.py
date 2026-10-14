#!/usr/bin/env python3
"""Prepare, or explicitly run, fresh full-marker healthy IK in isolated outputs.

Preparation is standard-library read-only except exclusive new protocol JSON.
Execution delegates unchanged run_pipeline with opensim_cmd=None, skip_scaling
and ik_only. No marker remapping/calibration, ID, RRA, or dynamic integration.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import shlex
import shutil
import sys
import time
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AUDIT = HERE.parent
RUNNER = ROOT / "scripts/run_opensim_sea_pipeline.py"
HEALTHY = {"AB06": ROOT / "models/AB06-raw/osimxml/AB06.osim",
           "AB07": ROOT / "models/AB07-raw/osimxml/ab07.osim"}
PROTOCOL = HERE / "protocol.json"


def relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def fp(path: Path) -> dict:
    return {"path": relative(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size_bytes": path.stat().st_size}


def trc_range(path: Path) -> dict:
    first = last = None
    count = 0
    with path.open(encoding="utf-8") as stream:
        for i, line in enumerate(stream):
            if i >= 5 and line.strip():
                stamp = float(line.split()[1])
                first = stamp if first is None else first
                last = stamp
                count += 1
    if first is None or last <= first:
        raise ValueError("Invalid full-span marker clock")
    return {"rows": count, "range_s": [first, last]}


def write_new(path: Path, payload: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")


def load_runner():
    spec = importlib.util.spec_from_file_location("healthy_original_ik_runner", RUNNER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def prepare() -> dict:
    plugin = ROOT / ("plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib" if sys.platform == "darwin" else "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll" if sys.platform == "win32" else "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.so")
    sources = [RUNNER, Path(__file__), plugin]
    sources += sorted((ROOT / "models/SEASEA/data").glob("*.xml"))
    subjects = {}
    for subject, healthy in HEALTHY.items():
        converted = AUDIT / "rebuild/models" / f"{subject}_SEASEA-raw/data/converted/treadmill/treadmill_01_01"
        trc, grf = converted / "treadmill_01_01.trc", converted / "treadmill_01_01_grf.mot"
        bundle_receipt = AUDIT / "rebuild/models" / f"{subject}_SEASEA_Threadmill/pipeline_report.json"
        outer_log = AUDIT / "rebuild/results" / subject / f"{subject.lower()}_seasea_treadmill_treadmill_01_01/logs/opensim_ik_pipeline.log"
        inner_log = AUDIT / "rebuild/results" / subject / f"{subject.lower()}_seasea_treadmill_treadmill_01_01_ik/02_ik/ik_full.log"
        sources += [healthy, trc, grf, bundle_receipt, outer_log, inner_log]
        names = [n.get("name") for n in ET.parse(healthy).findall(".//Coordinate")]
        markers = [n.get("name") for n in ET.parse(healthy).findall(".//MarkerSet/objects/Marker")]
        if len(names) != 23 or len(set(names)) != 23 or len(markers) != 28:
            raise ValueError("Healthy model contract requires 23 unique coordinates and 28 markers")
        argv = [sys.executable, "-B", str(Path(__file__)), "--run", "--subject", subject]
        subjects[subject] = {"model": relative(healthy), "trc": relative(trc), "grf": relative(grf),
                             "marker_set": relative(healthy), "coordinate_names": names, "marker_names": markers,
                             "full_clock": trc_range(trc), "output_root": relative(HERE / subject),
                             "expected_output_root_absent": not (HERE / subject).exists(),
                             "argv": argv, "posix_display": shlex.join(argv)}
    before = {relative(p): fp(p) for p in sources}
    return {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
            "instruction_check_token": "CMC_AGENT_OK_2026", "branch": "Codex",
            "status": "prepared_not_executed", "source_fingerprints": before,
            "subjects": subjects, "plugin": relative(plugin), "support_data": "models/SEASEA/data",
            "runner_contract": {"opensim_cmd": None, "solver_backend": "OpenSim Python API, same interpreter as rebuild argv",
                                "skip_scaling": True, "ik_only": True, "run_smoke": False,
                                "full_marker_weight": 1, "constraint_weight": "Inf", "accuracy": 1e-5,
                                "coordinate_file": "", "marker_calibration": False, "marker_remap": False,
                                "ID": False, "RRA": False, "dynamic_integration": False},
            "backend_evidence": {"rebuild_logs": "Per-subject outer argv uses envCMC-rllib and no explicit opensim-cmd; inner log records same plugin and 28612 frames completed in 13 seconds.",
                                 "opensim_cmd_discovery_observed_at_preparation": shutil.which("opensim-cmd"),
                                 "historical_limit": "Backend/version were not explicitly written by original runner logs. Python API is also attested by root; current discovery absence is recorded without retrospective version claims."},
            "purpose": "Separate the source healthy dataset IK solution from a newly solved healthy reference under the same marker/tool contract as prosthetic rebuilds.",
            "interpretation_limits": ["No inference that all native anatomical differences disappear under a common solver.",
                                      "Joint-center heights will use production full-trial filtering and frozen measured cycle windows; raw and filtered results must be labeled.",
                                      "Marker residuals are fit diagnostics; this control does not establish measured prosthetic ground geometry or dynamic causality."],
            "prepared_inputs_unchanged": all(fp(ROOT / p) == entry for p, entry in before.items())}


def run(subject: str) -> int:
    protocol = json.loads(PROTOCOL.read_text())
    for path, entry in protocol["source_fingerprints"].items():
        if fp(ROOT / path) != entry:
            raise RuntimeError(f"Prepared input changed: {path}")
    item = protocol["subjects"][subject]
    out = ROOT / item["output_root"]
    out.mkdir()  # Strict no-overwrite guard before original runner creates dirs.
    module = load_runner()
    cfg = module.ToolConfig(opensim_cmd=None, plugin=ROOT / protocol["plugin"],
        model=ROOT / item["model"], marker_set=ROOT / item["marker_set"],
        trc=ROOT / item["trc"], grf=ROOT / item["grf"], support_data=ROOT / protocol["support_data"],
        skip_scaling=True, ik_only=True, full_time_range=tuple(item["full_clock"]["range_s"]))
    import opensim
    version = opensim.GetVersionAndDate() if hasattr(opensim, "GetVersionAndDate") else opensim.GetVersion() if hasattr(opensim, "GetVersion") else getattr(opensim, "__version__", "unavailable")
    receipt = {"status": "started", "subject": subject, "start_utc": datetime.now(timezone.utc).isoformat(),
               "protocol": fp(PROTOCOL), "python_executable": sys.executable, "python_version": platform.python_version(),
               "opensim_version": str(version), "solver_backend": "OpenSim Python API", "opensim_cmd": None,
               "effective_tool_config": {k: str(v) if isinstance(v, Path) else v for k, v in asdict(cfg).items()},
               "unchanged_original_run_pipeline": fp(RUNNER)}
    write_new(out / "started_receipt.json", receipt)
    started = time.perf_counter()
    try:
        rc = module.run_pipeline(cfg, module.create_paths(out), run_smoke=False)
        if rc:
            raise RuntimeError(f"Original IK runner returned {rc}")
        label = module.range_label(tuple(item["full_clock"]["range_s"]))
        motion = out / "02_ik" / f"IK_full_{label}.mot"
        labels = module.storage_column_labels(motion)
        if labels[0] != "time" or set(labels[1:]) != set(item["coordinate_names"]) or len(labels) != 24:
            raise ValueError("Fresh healthy IK coordinate contract mismatch")
        receipt.update(status="completed", returncode=rc, output_motion=fp(motion), coordinate_names=labels[1:])
    except Exception as exc:
        receipt.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        receipt["elapsed_wall_s"] = time.perf_counter() - started
        receipt["end_utc"] = datetime.now(timezone.utc).isoformat()
        receipt["source_files_unchanged"] = all(fp(ROOT / p) == entry for p, entry in protocol["source_fingerprints"].items())
        write_new(out / "receipt.json", receipt)
    print(json.dumps({"subject": subject, "status": receipt["status"], "elapsed_wall_s": receipt["elapsed_wall_s"], "receipt": fp(out / "receipt.json")}))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Execute only after root launch assignment; otherwise prepare protocol only")
    parser.add_argument("--subject", choices=tuple(HEALTHY))
    args = parser.parse_args()
    if args.run:
        if args.subject is None:
            parser.error("--run requires exactly one --subject")
        return run(args.subject)
    if args.subject:
        parser.error("Preparation creates the common protocol for both subjects")
    protocol = prepare()
    if not protocol["prepared_inputs_unchanged"]:
        raise RuntimeError("Input changed while preparing protocol")
    write_new(PROTOCOL, protocol)
    print(json.dumps({"status": "prepared_not_executed", "protocol": fp(PROTOCOL), "source_count": len(protocol["source_fingerprints"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
