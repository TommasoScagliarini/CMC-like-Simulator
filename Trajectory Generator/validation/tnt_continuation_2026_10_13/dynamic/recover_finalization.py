"""Recover metadata after a proven post-integration path-rebasing failure.

Preserve the original failed receipt and log. Require the entire requested
native grid, 300 completed 10ms decisions for the 3s diagnostic, unchanged
controller/operational sources and the exact original adapter fingerprint.
No model is loaded and no simulation is repeated.
"""
from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from compare_diagnostics import storage
from run_tnt_diagnostic import REPO, fingerprint, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--observed-process-returncode", type=int, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    receipt_path = run / "receipt.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if receipt["status"] != "setup_or_save_failed" or not receipt.get("error", "").startswith("FileNotFoundError:"):
        raise ValueError("This recovery is restricted to the known finalization failure")
    if "kinematic_generator.py" not in receipt["error"] or "generator_dependencies_unchanged" in receipt:
        raise ValueError("Failure is not the expected final dependency path check")
    preserved = run / "receipt_original_finalization_failed.json"
    if preserved.exists() or (run / "recovery_manifest.json").exists():
        raise FileExistsError("Recovery already attempted")
    original = fingerprint(receipt_path)
    old_adapter = Path(__file__).with_name("run_tnt_diagnostic_executed_v1.py")
    if fingerprint(old_adapter)["sha256"] != receipt["adapter"]["sha256"]:
        raise AssertionError("Preserved adapter does not match the executed source")
    for item in receipt["protected_before"] + receipt["input_bindings"]["generator_dependencies"]:
        if fingerprint(REPO / item["path"])["sha256"] != item["sha256"]:
            raise AssertionError(f"Executed operational/controller source changed: {item['path']}")
    rows = [json.loads(line) for line in (run / "generator_trace.jsonl").read_text(encoding="utf-8").splitlines()]
    start, end = receipt["requested_window_s"]
    expected_decisions = int(round((end - start) / .01))
    if len(rows) != expected_decisions or receipt["generated_samples"] != expected_decisions:
        raise AssertionError("Decision coverage incomplete")
    decisions = np.array([r["time"] for r in rows])
    reached = np.array([r["completed_until_s"] for r in rows])
    if not np.allclose(decisions, start + np.arange(expected_decisions) * .01, atol=1e-8, rtol=0):
        raise AssertionError("Decision grid differs")
    if not np.allclose(reached, start + np.arange(1, expected_decisions + 1) * .01, atol=1e-8, rtol=0):
        raise AssertionError("Completed segment endpoints differ")
    if abs(receipt["final_time_s"] - end) > 1e-8:
        raise AssertionError("Runner did not reach requested end")
    dt = receipt["effective_simulator_config"]["dt"]
    expected_native = start + np.arange(int(round((end - start) / dt))) * dt
    native_files = sorted((run / "sim_outputs").glob("*.sto"))
    if len(native_files) < 15:
        raise AssertionError("Native channel set incomplete")
    native_before = [fingerprint(p) for p in native_files]
    for path in native_files:
        times, values = storage(path)
        if times.shape != expected_native.shape or not np.allclose(times, expected_native, atol=1e-8, rtol=0):
            raise AssertionError(f"Native coverage incomplete: {path.name}")
        if not all(np.isfinite(v).all() for v in values.values()):
            raise AssertionError(f"Nonfinite native table: {path.name}")
    shutil.copyfile(receipt_path, preserved)
    shutil.copyfile(args.log, run / "execution_log_returncode1.txt")
    write_json(run / "generator_trace.json", rows)
    receipt["status"] = "simulation_completed_finalization_recovered"
    receipt["process_returncode"] = args.observed_process_returncode
    receipt["finalization_error"] = receipt.pop("error")
    receipt["finalization_traceback"] = receipt.pop("traceback")
    receipt["generator_dependencies_unchanged"] = True
    receipt["original_failed_receipt"] = fingerprint(preserved)
    receipt["executed_adapter_preserved"] = fingerprint(old_adapter)
    receipt["metadata_recovery"] = {
        "at_utc": datetime.now(timezone.utc).isoformat(), "script": fingerprint(Path(__file__)),
        "native_simulation_repeated": False,
        "elapsed_time_source": "Original receipt recorded time.monotonic elapsed before failing dependency-path finalization check",
        "decision_count_verified": len(rows), "native_samples_per_table_verified": len(expected_native),
        "native_first_last_s": [float(expected_native[0]), float(expected_native[-1])],
        "last_completed_segment_endpoint_s": float(reached[-1]),
    }
    write_json(receipt_path, receipt)
    if native_before != [fingerprint(p) for p in native_files]:
        raise AssertionError("Native data changed during metadata recovery")
    command = ["python", str(Path(__file__).resolve()), "--run", str(run), "--log", str(args.log.resolve()),
               "--observed-process-returncode", str(args.observed_process_returncode)]
    write_json(run / "recovery_manifest.json", {
        "status": receipt["status"], "original_receipt_before_recovery": original,
        "preserved_failed_receipt": fingerprint(preserved), "recovered_receipt": fingerprint(receipt_path),
        "recovery_command_arguments": command,
        "original_process_returncode": args.observed_process_returncode,
        "returncode_source": "Observed and supplied by supervising parent process",
        "native_files_unchanged": native_before, "native_simulation_repeated": False,
        "jsonl_source": fingerprint(run / "generator_trace.jsonl"),
        "json_trace_rebuilt_from_completed_jsonl_records": fingerprint(run / "generator_trace.json"),
    })
    print(json.dumps({"run": str(run), "status": receipt["status"], "native_samples": len(expected_native), "decisions": len(rows)}))


if __name__ == "__main__":
    main()
