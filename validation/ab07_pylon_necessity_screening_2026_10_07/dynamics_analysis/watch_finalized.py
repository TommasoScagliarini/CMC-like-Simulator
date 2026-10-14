"""Process only finalized queue outputs with the frozen V2 consumer.

No native rollout is spawned. Existing per-case metrics are preserved. Logs and
watch receipts live exclusively in the new diagnostics directory; all physics,
profiles, preparation variants and old audits remain read-only.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
REPO = HERE.parents[2]
PROTOCOL = HERE / "analysis_protocol_v2.json"
CONSUMER = HERE / "analyze_dynamic_screen_v2.py"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def fp(p):
    return {"path": p.relative_to(REPO).as_posix(), "sha256": sha(p)}


def read(p):
    return json.loads(p.read_text(encoding="utf-8"))


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_new(p, data):
    with p.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False); stream.write("\n")


def main():
    protocol, dynamic = read(PROTOCOL), read(AUDIT/"dynamic_protocol.json")
    if sha(CONSUMER) != protocol["source_bindings"]["postprocessor"]["sha256"]:
        raise ValueError("Consumer does not match frozen V2 source.")
    if sha(AUDIT/"dynamic_protocol.json") != protocol["source_bindings"]["dynamic_protocol"]["sha256"]:
        raise ValueError("Dynamic protocol differs.")
    folder = HERE / "watch_receipts"
    folder.mkdir(exist_ok=True)
    write_new(folder/"start.json", {"started_utc": utc(), "watcher": fp(Path(__file__)), "consumer": fp(CONSUMER),
                                   "analysis_protocol": fp(PROTOCOL), "dynamic_protocol": fp(AUDIT/"dynamic_protocol.json"),
                                   "case_ids": dynamic["execution_order"], "poll_s": 20,
                                   "native_rollouts_spawned": False, "new_Claude_branch_read": False})
    deadline = time.monotonic() + 8*3600
    progress = (folder/"progress.jsonl").open("x", encoding="utf-8")
    done = set()
    while len(done) < len(dynamic["cases"]):
        for case in dynamic["execution_order"]:
            if case in done:
                continue
            receipt_path = AUDIT/"dynamics"/case/"run/receipt.json"
            if not receipt_path.is_file():
                continue
            # The producer replaces/writes JSON only at boundaries. A read
            # concurrent with a write is retried, never treated as a failure.
            try:
                receipt = read(receipt_path)
            except json.JSONDecodeError:
                continue
            if receipt.get("status") not in ("completed", "interrupted", "failed") or not receipt.get("finished_utc"):
                continue
            metric_path = HERE/case/"metrics.json"
            if not metric_path.exists():
                with (HERE/f"{case}.analysis_v2.log").open("x", encoding="utf-8") as log:
                    result = subprocess.run([sys.executable, "-B", str(CONSUMER), "--case", case, "--plots"],
                                            cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
                if result.returncode:
                    raise RuntimeError(f"{case}: finalized consumer exit {result.returncode}; inspect preserved analysis_v2 log")
            metric = read(metric_path)
            if metric["receipt"]["sha256"] != sha(receipt_path) or metric["analysis_protocol"]["sha256"] != sha(PROTOCOL):
                raise ValueError("Existing metrics do not match finalized receipt/frozen V2 protocol.")
            entry = {"observed_utc": utc(), "case": case, "native_status": receipt["status"],
                     "analysis_status": metric["analysis_status"], "metrics": fp(metric_path),
                     "primary_coverage": metric["windows"].get("primary_all", {}).get("coverage"),
                     "prefix_exact": metric["prefix_comparison"].get("all_available_tables_exact_numeric_prefix")}
            progress.write(json.dumps(entry)+"\n"); progress.flush()
            done.add(case)
            print(json.dumps({"processed": len(done), "required": len(dynamic["cases"]), **entry}), flush=True)
        if len(done) == len(dynamic["cases"]):
            break
        if time.monotonic() > deadline:
            raise TimeoutError("Eight-hour observational wait expired; no native action or missing-result substitution performed.")
        time.sleep(20)
    progress.close()
    with (HERE/"aggregate_v2.log").open("x", encoding="utf-8") as log:
        aggregate = subprocess.run([sys.executable, "-B", str(CONSUMER), "--all", "--aggregate-only"],
                                   cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
    if aggregate.returncode:
        raise RuntimeError("Aggregate failed; inspect preserved aggregate_v2.log")
    write_new(folder/"completed.json", {"finished_utc": utc(), "processed_count": len(done),
                                        "aggregate": fp(HERE/"aggregate_metrics.json"),
                                        "consumer": fp(CONSUMER), "analysis_protocol": fp(PROTOCOL),
                                        "native_rollouts_spawned": False,
                                        "visual_review_and_scientific_synthesis_owned_by_parent": True})
    print(json.dumps({"status": "all_finalized_cases_processed", "count": len(done), "aggregate": fp(HERE/"aggregate_metrics.json")}), flush=True)


if __name__ == "__main__":
    main()
