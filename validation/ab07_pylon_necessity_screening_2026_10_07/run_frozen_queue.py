"""Drain the already frozen screen with three independent native workers.

The first three cases were launched by the parent. Start every remaining case
once a slot closes, irrespective of its physical outcome. Never retry or alter
the scientific protocol, inputs, solver options, or observation windows.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time


AUDIT = Path(__file__).resolve().parent
PROTOCOL = AUDIT / "dynamic_protocol.json"


def now():
    return datetime.now(timezone.utc).isoformat()


def terminal(name):
    path = AUDIT / "dynamics" / name / "run/receipt.json"
    if not path.exists():
        return None
    try:
        receipt = json.loads(path.read_text())
    except json.JSONDecodeError:
        return None
    return receipt if receipt.get("finished_utc") else None


def main():
    protocol = json.loads(PROTOCOL.read_text())
    order = protocol["execution_order"]
    assert len(order) == 18 and len(set(order)) == 18
    journal = AUDIT / "queue_journal.jsonl"
    slots = [{"case": name, "process": None} for name in order[:3]]
    pending = iter(order[3:])
    with journal.open("x", encoding="utf-8", buffering=1) as stream:
        def emit(obj):
            row = {"utc": now(), **obj}
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            print(json.dumps(row), flush=True)

        emit({"event": "queue_frozen", "protocol_sha256": hashlib.sha256(PROTOCOL.read_bytes()).hexdigest(),
              "full_order": order, "maximum_native_workers": 3,
              "already_launched": order[:3], "outcome_dependent_selection": False})
        while slots:
            for slot in list(slots):
                receipt = terminal(slot["case"])
                process = slot["process"]
                ended = process is not None and process.poll() is not None
                if receipt is None and not ended:
                    continue
                # For our children wait for native export/source checks to finish.
                if process is not None and not ended:
                    continue
                emit({"event": "case_closed", "case": slot["case"],
                      "status": receipt.get("status") if receipt else "launcher_failed_without_receipt",
                      "exit_code": process.returncode if process else None,
                      "finished_utc": receipt.get("finished_utc") if receipt else None})
                if process is not None:
                    slot["stdout"].close()
                    slot["stderr"].close()
                try:
                    name = next(pending)
                except StopIteration:
                    slots.remove(slot)
                    continue
                folder = AUDIT / "dynamics" / name
                if (folder / "run").exists():
                    raise FileExistsError("Refuse to retry or overwrite: " + name)
                stdout = (folder / "native.stdout.log").open("x")
                stderr = (folder / "native.stderr.log").open("x")
                child = subprocess.Popen([sys.executable, "-B", str(AUDIT / "run_dynamic_screen.py"), "--case", name],
                                         cwd=folder, stdout=stdout, stderr=stderr)
                slot.update(case=name, process=child, stdout=stdout, stderr=stderr)
                emit({"event": "case_started", "case": name, "pid": child.pid})
            if slots:
                time.sleep(2)
        emit({"event": "all_cases_closed", "required_cases": len(order)})


if __name__ == "__main__":
    main()
