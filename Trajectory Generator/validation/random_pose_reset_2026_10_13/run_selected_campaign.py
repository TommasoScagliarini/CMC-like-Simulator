"""Execute all frozen selected random poses with bounded native parallelism.

The separately executed coherent control must have a finalized receipt.
No scientific failure is replaced or omitted; every case has a supervisor.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading

HERE = Path(__file__).resolve().parent


def now():
    return datetime.now(timezone.utc).isoformat()


def main():
    python = sys.executable
    selection_path = HERE / "preflight/selected_cases.json"
    selection = json.loads(selection_path.read_text())
    assert selection["ready_for_preregistered_rollouts"]
    control = json.loads((HERE / "rollouts/coherent_nominal/receipt.json").read_text())
    assert control["status"] != "started" and "elapsed_wall_s" in control
    cases = [case for case in selection["cases"] if case["id"] != "coherent_nominal"]
    assert len(cases) == 20
    output = HERE / "campaign_execution.json"
    if output.exists():
        raise FileExistsError("Preserve previous campaign execution receipt")
    receipt = {"started_utc": now(), "maximum_concurrent_native_cases": 6,
               "python": python, "selected_random_ids": [c["id"] for c in cases],
               "selection_sha256": hashlib.sha256(selection_path.read_bytes()).hexdigest(),
               "control_status": control["status"], "case_executions": []}
    output.write_text(json.dumps(receipt, indent=2)+"\n")
    manifest = json.loads((HERE / "frozen_manifest.json").read_text())
    repo = HERE.parents[2]
    fatal_drift = threading.Event()

    def verify_inputs():
        for name, expected in manifest["protected_sha256"].items():
            if hashlib.sha256((repo / name).read_bytes()).hexdigest() != expected:
                fatal_drift.set()
                raise RuntimeError("Global frozen input drift: " + name)

    def execute(case):
        if fatal_drift.is_set():
            return {"case": case["id"], "attempted": False,
                    "global_input_drift": True, "supervisor_returncode": None}
        verify_inputs()
        argv = [python, "-B", "-u", str(HERE / "random_pose_reset.py"),
                "--run", "--cases", case["id"]]
        log = HERE / (case["id"] + "_parent_supervisor.log")
        started = now()
        with log.open("xb") as stream:
            code = subprocess.call(argv, stdout=stream, stderr=subprocess.STDOUT)
        verify_inputs()
        return {"case": case["id"], "argv": argv, "started_utc": started,
                "completed_utc": now(), "supervisor_returncode": code, "attempted": True}

    with ThreadPoolExecutor(max_workers=6) as pool:
        pending = [pool.submit(execute, case) for case in cases]
        for future in as_completed(pending):
            result = future.result()
            receipt["case_executions"].append(result)
            output.write_text(json.dumps(receipt, indent=2)+"\n")
            print(json.dumps(result), flush=True)
    receipt["completed_utc"] = now()
    receipt["all_selected_attempted"] = sum(x.get("attempted", False) for x in receipt["case_executions"]) == 20
    output.write_text(json.dumps(receipt, indent=2)+"\n")
    return 0 if all(x["supervisor_returncode"] == 0 for x in receipt["case_executions"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
