"""Independent, read-only SHA-256 closing guard. No native or scientific consumer.

Only the explicitly named local Codex protocols/seals and finalized case JSONs
are parsed. Referenced files are hashed without recursively parsing their
contents, avoiding historical metadata being mistaken for current baselines.
The new Claude directory is excluded even when a local seal references it.
"""
from __future__ import annotations

import hashlib
import json
import re
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path


HERE = Path(__file__).resolve().parent
BASE = HERE.parent
REPO = BASE.parent.parent
OUTPUT = HERE / "final_protected_source_audit.json"
HEX = re.compile(r"^[0-9a-f]{64}$")
INPUTS = [
    "protocol.json", "dynamic_protocol.json",
    "dynamics_analysis/analysis_protocol.json",
    "dynamics_analysis/analysis_protocol_v2.json",
    "dynamics_analysis/dynamics_numeric_seal.json",
    "phase_timing/phase_timing_protocol.json",
    "phase_timing/phase_timing_protocol_v2.json",
    "phase_timing/scientific_review_v2.json",
    "preparation/preparation_protocol.json",
    "preparation/scientific_screening_seal.json",
    "preparation/supplement_pose/supplement_protocol.json",
    "preparation/supplement_pose/scientific_supplement_seal.json",
    "geometry/protocol.json", "geometry/geometry_seal.json",
    "dynamics_analysis/aggregate_metrics.json",
    "phase_timing/aggregate_phase_metrics_v2.json",
    "review/final_screening_scientific_review.json",
    "review/final_screening_phase_addendum.json",
    "root_findings_before_claude.json",
]


def now():
    return datetime.now(timezone.utc).isoformat()


def label(path):
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def forbidden(path):
    # The repository itself contains "Claude" in its name; only child branch
    # directory segments beginning with "claude" trigger this exclusion.
    relative = Path(label(path))
    return any(part.lower().startswith("claude") for part in relative.parts[:-1])


def digest(path):
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
    except (OSError, ValueError) as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def resolve(raw, owner):
    raw = Path(raw)
    if raw.is_absolute():
        if raw.exists():
            return raw.resolve()
        for anchor in ("validation", "models", "Trajectory Generator", "results",
                       "online_grf_profiles", "plugins"):
            if anchor in raw.parts:
                return REPO.joinpath(*raw.parts[raw.parts.index(anchor):]).resolve()
        return raw
    rooted = REPO / raw
    if rooted.exists() or len(raw.parts) > 1:
        return rooted.resolve()
    return (owner.parent / raw).resolve()


def main():
    if OUTPUT.exists():
        raise FileExistsError(f"Refusing to overwrite {OUTPUT}")
    started = now()
    expected, excluded, parsed, initial_json = {}, [], {}, {}
    failures, guards, manifest_maps = [], [], []

    def check(name, passed, detail=None):
        guards.append({"name": name, "passed": bool(passed), "detail": detail})
        if not passed:
            failures.append(name)

    def add(raw, sha, owner, field, size=None):
        path = resolve(raw, owner)
        origin = {"document": label(owner), "field": field, "sha256": sha}
        if size is not None:
            origin["size_bytes"] = size
        if forbidden(path):
            excluded.append({"path": label(path), "reference": origin,
                             "reason": "New Claude branch excluded by instruction; contents not opened or hashed."})
            return
        entry = expected.setdefault(path, {"origins": [], "sha256": set(), "sizes": set()})
        entry["origins"].append(origin)
        entry["sha256"].add(sha)
        if isinstance(size, int):
            entry["sizes"].add(size)

    def collect(value, owner, field="$", depth=0):
        if isinstance(value, dict):
            sha = value.get("sha256")
            raw = value.get("path", value.get("file"))
            if isinstance(raw, str) and isinstance(sha, str) and HEX.fullmatch(sha):
                add(raw, sha, owner, field, value.get("size_bytes"))
            path_hash_map = {k: v for k, v in value.items()
                             if isinstance(k, str) and ("/" in k or "." in Path(k).name)
                             and isinstance(v, str) and HEX.fullmatch(v)}
            if path_hash_map:
                manifest_maps.append({"document": label(owner), "field": field,
                                      "count": len(path_hash_map)})
                for raw_path, checksum in path_hash_map.items():
                    add(raw_path, checksum, owner, field + "." + raw_path)
            for key, child in value.items():
                if isinstance(child, (dict, list)):
                    collect(child, owner, field + "." + str(key), depth + 1)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                if isinstance(child, (dict, list)):
                    collect(child, owner, field + f"[{index}]", depth + 1)

    def read_json(path):
        path = path.resolve()
        if forbidden(path):
            raise ValueError("Forbidden branch selected for parsing")
        initial_json[path] = digest(path)
        obj = json.loads(path.read_text(encoding="utf-8"))
        parsed[path] = obj
        collect(obj, path)
        return obj

    own_before = digest(Path(__file__).resolve())
    for relative in INPUTS:
        read_json(BASE / relative)
    dynamic = parsed[BASE / "dynamic_protocol.json"]
    aggregate = parsed[BASE / "dynamics_analysis/aggregate_metrics.json"]
    phase_aggregate = parsed[BASE / "phase_timing/aggregate_phase_metrics_v2.json"]
    numeric_seal = parsed[BASE / "dynamics_analysis/dynamics_numeric_seal.json"]
    case_ids = list(dynamic["cases"])
    check("Exactly 18 frozen dynamic cases", len(case_ids) == 18)
    for name, obj in (("numeric aggregate", aggregate), ("phase aggregate", phase_aggregate),
                      ("numeric seal", numeric_seal)):
        check(name + " exact case set", set(obj["cases"]) == set(case_ids))
    check("Canonical plots exact case set", set(numeric_seal["canonical_plots"]) == set(case_ids))
    required_protocols = {
        "dynamic_protocol": BASE / "dynamic_protocol.json",
        "analysis_protocol": BASE / "dynamics_analysis/analysis_protocol_v2.json",
        "phase_protocol": BASE / "phase_timing/phase_timing_protocol_v2.json",
    }
    for name, obj in (("numeric aggregate", aggregate), ("phase aggregate", phase_aggregate)):
        for key, protocol_path in required_protocols.items():
            if key in obj:
                fp = obj[key]
                check(name + " " + key + " binding",
                      resolve(fp["path"], BASE) == protocol_path
                      and fp["sha256"] == initial_json[protocol_path]["sha256"])
    native_paths, metric_paths, phase_paths, png_paths = set(), [], [], set()
    for case_id in case_ids:
        numeric_path = resolve(aggregate["cases"][case_id]["metrics"]["path"], BASE / "dynamics_analysis/aggregate_metrics.json")
        phase_path = resolve(phase_aggregate["cases"][case_id]["metrics"]["path"], BASE / "phase_timing/aggregate_phase_metrics_v2.json")
        numeric, phase = read_json(numeric_path), read_json(phase_path)
        sealed_metric = numeric_seal["cases"][case_id]["metrics"]
        check(case_id + " numeric aggregate/seal metric binding",
              resolve(sealed_metric["path"], BASE) == numeric_path
              and sealed_metric["sha256"] == aggregate["cases"][case_id]["metrics"]["sha256"])
        metric_paths.append(label(numeric_path))
        phase_paths.append(label(phase_path))
        receipt_path = BASE / "dynamics" / case_id / "run/receipt.json"
        # Receipts are parsed only for finalized status and identity. The other
        # referenced source files, including historical receipts, are hash-only.
        initial_json[receipt_path] = digest(receipt_path)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        check(case_id + " receipt terminal", receipt.get("status") in ("completed", "failed"), receipt.get("status"))
        for name, obj in (("numeric", numeric), ("phase", phase)):
            check(case_id + " " + name + " case identity", obj.get("case_id") == case_id and obj.get("case") == dynamic["cases"][case_id])
            check(case_id + " " + name + " receipt binding",
                  resolve(obj["receipt"]["path"], numeric_path) == receipt_path
                  and obj["receipt"]["sha256"] == initial_json[receipt_path]["sha256"])
            for key, protocol_path in required_protocols.items():
                if key in obj:
                    fp = obj[key]
                    check(case_id + " " + name + " " + key + " binding",
                          resolve(fp["path"], numeric_path) == protocol_path
                          and fp["sha256"] == initial_json[protocol_path]["sha256"])
        numeric_files = numeric["run_artifacts_unchanged"]["files"]
        phase_before = phase["run_artifacts_unchanged"]["before"]
        phase_after = phase["run_artifacts_unchanged"]["after"]
        check(case_id + " native inventories agree", len(numeric_files) == 22
              and {p: fp["sha256"] for p, fp in numeric_files.items()} == phase_before == phase_after)
        native_paths.update(resolve(p, numeric_path) for p in numeric_files)
        pngs = numeric_seal["canonical_plots"][case_id]["PNG_files"]
        check(case_id + " seven canonical PNGs", len(pngs) == 7)
        png_paths.update(resolve(fp["path"], numeric_path) for fp in pngs)
    check("396 distinct finalized native artifacts", len(native_paths) == 396)
    check("126 distinct canonical PNGs", len(png_paths) == 126)
    all_paths = sorted(set(expected) | set(initial_json) | {Path(__file__).resolve()}, key=str)
    before_hash_start = now()
    with ThreadPoolExecutor(max_workers=8) as pool:
        before = dict(zip(all_paths, pool.map(digest, all_paths)))
    before_hash_end = now()
    # Identity/json checks above do not alter any source; full independent
    # expected-digest verification and repeat hash cover the observation window.
    with ThreadPoolExecutor(max_workers=8) as pool:
        after = dict(zip(all_paths, pool.map(digest, all_paths)))
    after_hash_end = now()
    files = {}
    for path in all_paths:
        exp = expected.get(path, {"origins": [], "sha256": set(), "sizes": set()})
        observed_before, observed_after = before[path], after[path]
        conflict = len(exp["sha256"]) > 1 or len(exp["sizes"]) > 1
        expected_match = (not conflict and not observed_before.get("error")
                          and (not exp["sha256"] or observed_before["sha256"] in exp["sha256"])
                          and (not exp["sizes"] or observed_before["size_bytes"] in exp["sizes"]))
        unchanged = (observed_before == observed_after and observed_before.get("stable_during_hash") is True)
        initial_match = (path not in initial_json or initial_json[path] == observed_before)
        helper_match = (path != Path(__file__).resolve() or own_before == observed_before)
        passed = expected_match and unchanged and initial_match and helper_match
        if not passed:
            failures.append(label(path))
        files[label(path)] = {
            "expected_sha256": sorted(exp["sha256"]), "expected_sizes_bytes": sorted(exp["sizes"]),
            "expected_origins": exp["origins"], "conflicting_expectations": conflict,
            "observed_before": observed_before, "observed_after": observed_after,
            "initial_json_or_helper_observation_matches": initial_match and helper_match,
            "expected_matches": expected_match, "unchanged_during_independent_audit": unchanged,
            "passed": passed,
        }
    protected_manifest_fields = [
        ("protocol.json", "original_and_previous_audit_sources_before"),
        ("dynamic_protocol.json", "protected_sources_before"),
        ("phase_timing/phase_timing_protocol.json", "protected_sources_before"),
        ("phase_timing/phase_timing_protocol_v2.json", "protected_sources_before"),
        ("dynamics_analysis/dynamics_numeric_seal.json", "source_sha256"),
    ]
    primary_union = set()
    for relative, key in protected_manifest_fields:
        owner = BASE / relative
        primary_union.update(resolve(p, owner) for p in parsed[owner][key])
    receipt = {
        "schema_version": 1, "created_utc": now(), "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "PASS" if not failures else "FAIL", "independent_audit_started_utc": started,
        "scope": "Read-only SHA-256 closing guard; no scientific metric recomputation, FK, native integration, solver, controller or operational edits.",
        "new_Claude_branch_read": False, "Claude_references_excluded_without_opening": excluded,
        "fingerprint_timing": {"selected_JSON_hashes": "Immediately before parsing each explicitly selected local document; initial observations compared to subsequent full union hash.",
                               "full_union_before_started_utc": before_hash_start,
                               "full_union_before_finished_utc": before_hash_end,
                               "full_union_after_finished_utc": after_hash_end,
                               "meaning": "Current hashes match prior recorded expectations and are stable during this closing audit; no retroactive before-experiment claim."},
        "helper": {"path": label(Path(__file__).resolve()), **own_before},
        "protocols_and_seals": [{"path": label(p), **initial_json[p]} for p in parsed if p not in [resolve(x, BASE) for x in metric_paths + phase_paths]],
        "manifest_maps": manifest_maps, "primary_protected_manifest_union_count": len(primary_union),
        "all_current_bound_source_and_output_union_count": len(all_paths),
        "baseline_bound_path_count": len(expected), "case_count": len(case_ids),
        "numeric_metric_files": metric_paths, "phase_metric_files": phase_paths,
        "native_artifact_count": len(native_paths), "canonical_PNG_count": len(png_paths),
        "identity_and_inventory_guards": guards, "failures": failures,
        "files": files,
        "limits": ["Hash PASS preserves existing completed, failed, unavailable and censored case labels; it does not qualify scientific or biological performance.",
                   "Only explicitly selected local Codex protocol/seal/case JSONs are parsed; referenced source files are hash-only and historical embedded metadata are not reinterpreted.",
                   "All new Claude branch contents and its packet are excluded by instruction; parent separately owns those checks.",
                   "This receipt is the sole output of the checker and has no self-referential expected hash."],
    }
    with OUTPUT.open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": receipt["status"], "receipt": label(OUTPUT),
                      "receipt_sha256": digest(OUTPUT)["sha256"], "primary_protected_union": len(primary_union),
                      "all_bound_union": len(all_paths), "cases": len(case_ids),
                      "native_artifacts": len(native_paths), "canonical_PNGs": len(png_paths),
                      "excluded_Claude_references": len(excluded), "failures": failures}, indent=2))


if __name__ == "__main__":
    main()
