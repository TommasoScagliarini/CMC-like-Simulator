"""Preserve initial receipt and qualify only its overly narrow terminal guard.

All recorded sources/outputs are rehashed before/after. No scientific consumer,
native/FK call, outcome relabeling, prior artifact edit, or Claude directory read.
"""
from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
REPO = BASE.parent.parent
INITIAL = HERE / "final_protected_source_audit.json"
OUTPUT = HERE / "final_protected_source_audit_addendum.json"
TERMINAL = {"completed", "interrupted", "failed"}
CORRECTED = {"P5_A0", "P6_A0", "P6_A1"}


def now():
    return datetime.now(timezone.utc).isoformat()


def label(path):
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def resolve(raw):
    p = Path(raw)
    return p if p.is_absolute() else REPO / p


def excluded(path):
    return any(p.lower().startswith("claude") for p in Path(label(path)).parts[:-1])


def fingerprint(path):
    try:
        start = path.stat()
        h = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                h.update(chunk)
        end = path.stat()
        return {"sha256": h.hexdigest(), "size_bytes": end.st_size,
                "stable_during_hash": (start.st_size, start.st_mtime_ns, start.st_ino)
                == (end.st_size, end.st_mtime_ns, end.st_ino)}
    except OSError as exc:
        return {"error": str(exc)}


def main():
    if OUTPUT.exists():
        raise FileExistsError("Refusing to overwrite existing addendum")
    started = now()
    initial_before, helper_before = fingerprint(INITIAL), fingerprint(Path(__file__).resolve())
    initial = json.loads(INITIAL.read_text())
    problems = []
    expected_failed_guards = {f"{case} receipt terminal" for case in CORRECTED}
    if initial["status"] != "FAIL" or set(initial["failures"]) != expected_failed_guards:
        problems.append("Initial failure set differs from the three documented terminal-guard false negatives")
    if not all(row["passed"] for row in initial["files"].values()):
        problems.append("An initial file/hash guard failed; terminal addendum cannot excuse it")
    if any(not guard["passed"] and guard["name"] not in expected_failed_guards
           for guard in initial["identity_and_inventory_guards"]):
        problems.append("A different initial identity/inventory guard failed")
    paths = {resolve(p) for p in initial["files"]} | {INITIAL, Path(__file__).resolve()}
    if any(excluded(p) for p in paths):
        raise ValueError("Forbidden Claude branch entered hash union")
    paths = sorted(paths, key=str)
    before_start = now()
    with ThreadPoolExecutor(max_workers=8) as pool:
        before = dict(zip(paths, pool.map(fingerprint, paths)))
    before_end = now()
    cases = {}
    for case in initial["numeric_metric_files"]:
        case_id = Path(case).parent.name
        path = BASE / "dynamics" / case_id / "run/receipt.json"
        data = json.loads(path.read_text())
        terminal = data.get("status") in TERMINAL and bool(data.get("finished_utc"))
        if not terminal:
            problems.append(f"{case_id}: receipt is not finalized under the source-declared contract")
        if case_id in CORRECTED and (data.get("status") != "interrupted"
                                    or data.get("completed_requested_duration") is not False):
            problems.append(f"{case_id}: changed partial/interrupted outcome")
        cases[case_id] = {"receipt": label(path), "receipt_sha256": before[path].get("sha256"),
                          "status_preserved": data.get("status"), "finished_utc": data.get("finished_utc"),
                          "steps": data.get("steps"),
                          "completed_requested_duration_preserved": data.get("completed_requested_duration"),
                          "terminal_receipt_guard_passed": terminal}
    manifests = []
    primary_union = set()
    for relative, key in [
        ("protocol.json", "original_and_previous_audit_sources_before"),
        ("dynamic_protocol.json", "protected_sources_before"),
        ("dynamics_analysis/analysis_protocol.json", "source_bindings"),
        ("dynamics_analysis/analysis_protocol_v2.json", "source_bindings"),
        ("phase_timing/phase_timing_protocol.json", "protected_sources_before"),
        ("phase_timing/phase_timing_protocol.json", "source_bindings"),
        ("phase_timing/phase_timing_protocol_v2.json", "protected_sources_before"),
        ("phase_timing/phase_timing_protocol_v2.json", "source_bindings"),
        ("dynamics_analysis/dynamics_numeric_seal.json", "source_sha256"),
    ]:
        path = BASE / relative
        source_map = json.loads(path.read_text())[key]
        entries = {resolve(v["path"] if isinstance(v, dict) else k) for k, v in source_map.items()}
        primary_union |= entries
        ignored = {p for p in entries if excluded(p)}
        if not entries - ignored <= set(before):
            problems.append(f"Primary source manifest not fully covered: {relative} {key}")
        manifests.append({"path": label(path), "field": key,
                          "raw_count": len(entries), "excluded_Claude_count": len(ignored),
                          "eligible_verified_count": len(entries - ignored)})
    source_evidence = []
    for relative, low, high in [("run_dynamic_screen.py", 215, 230),
                                ("dynamics_analysis/analyze_dynamic_screen_v2.py", 390, 391)]:
        path = BASE / relative
        lines = path.read_text().splitlines()
        source_evidence.append({"path": label(path), "sha256": before[path]["sha256"],
                                "line_start": low, "line_end": high,
                                "text": "\n".join(lines[low - 1:high])})
    with ThreadPoolExecutor(max_workers=8) as pool:
        after = dict(zip(paths, pool.map(fingerprint, paths)))
    after_end = now()
    checked = {}
    for path in paths:
        raw = label(path)
        previous = initial["files"].get(raw)
        expected = previous["observed_after"] if previous else initial_before if path == INITIAL else helper_before
        passed = before[path] == after[path] == expected and before[path].get("stable_during_hash") is True
        if not passed:
            problems.append(f"Hash changed or unreadable: {raw}")
        checked[raw] = {"expected_from_initial_guard_or_addendum_start": expected,
                        "observed_before": before[path], "observed_after": after[path], "passed": passed}
    ignored_primary = {p for p in primary_union if excluded(p)}
    result = {
        "schema_version": 1, "created_utc": now(), "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "PASS" if not problems else "FAIL", "initial_receipt_preserved": {"path": label(INITIAL), **initial_before},
        "initial_overall_FAIL_preserved": True,
        "qualification": "Initial SHA, identity and inventory checks passed for all files; its overall FAIL came solely from rejecting the source-declared terminal status interrupted. This supplement corrects that guard, requires finished_utc, and preserves every run's status and incomplete-duration flag.",
        "source_declared_terminal_contract": {"allowed_statuses": sorted(TERMINAL), "requires_finished_utc": True,
                                              "evidence": source_evidence},
        "corrected_guard_cases": sorted(CORRECTED), "case_count": len(cases), "cases": cases,
        "primary_manifest_maps": manifests, "primary_protected_union_raw_count": len(primary_union),
        "primary_protected_union_excluded_Claude_count": len(ignored_primary),
        "primary_protected_union_eligible_verified_count": len(primary_union - ignored_primary),
        "primary_protected_union_excluded_paths": sorted(label(p) for p in ignored_primary),
        "original_guard_verified_file_count": len(initial["files"]),
        "numeric_metrics_count": len(initial["numeric_metric_files"]),
        "phase_metrics_count": len(initial["phase_metric_files"]),
        "native_artifact_count": initial["native_artifact_count"], "canonical_PNG_count": initial["canonical_PNG_count"],
        "supplement_rechecked_file_count_including_initial_receipt_and_helper": len(paths),
        "hashes_all_expected_and_unchanged": not problems,
        "fingerprint_timing": {"started_utc": started, "full_union_before_started_utc": before_start,
                               "full_union_before_finished_utc": before_end, "full_union_after_finished_utc": after_end,
                               "meaning": "Repeat current closing-audit guard; no retroactive claim of hashing before experiments or earlier scientific computations."},
        "new_Claude_branch_read": False,
        "Claude_exclusion_policy": "No old or new Claude directory contents opened or hashed; prior local seal references are disclosed as exclusions, not asserted verified.",
        "helper": {"path": label(Path(__file__).resolve()), **helper_before},
        "problems": problems, "file_rechecks": checked,
        "limits": ["PASS is source/output immutability only, with interrupted/incomplete/censored cases unchanged.",
                   "Original FAIL receipt/helper remain byte-exact; no physical, numerical-metric, threshold, controller or plugin modification.",
                   "The supplement itself is new and has no self-referential hash."]
    }
    with OUTPUT.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "receipt": label(OUTPUT), "sha256": fingerprint(OUTPUT)["sha256"],
                      "original_files": len(initial["files"]), "rechecked_including_companions": len(paths),
                      "primary_raw": len(primary_union), "primary_excluded": len(ignored_primary),
                      "primary_verified": len(primary_union - ignored_primary), "problems": problems}, indent=2))


if __name__ == "__main__":
    main()
