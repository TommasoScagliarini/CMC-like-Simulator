"""Independent JSON/hash/byte audit; no FK, IK, force integration or native run."""
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
IDS = ("PYLON400_REFERENCE", "STANDING_AB07", "STANDING_FOURROI_AB07")


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def resolve(raw):
    p = Path(str(raw).replace("\\", "/"))
    return p if p.is_absolute() else ROOT / p


def fp(path):
    p = resolve(path)
    return {"path": p.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def identity(value):
    if isinstance(value, dict):
        return ("dict", tuple((key, identity(item)) for key, item in sorted(value.items())))
    if isinstance(value, list):
        return ("list", tuple(identity(item) for item in value))
    if isinstance(value, float):
        return ("float", value.hex())
    return (type(value).__name__, value)


def finite(value):
    if isinstance(value, dict):
        for item in value.values(): finite(item)
    elif isinstance(value, list):
        for item in value: finite(item)
    elif isinstance(value, float):
        assert math.isfinite(value)


def inventory(folder):
    return {fp(p)["path"]: fp(p)["sha256"] for p in sorted(folder.rglob("*")) if p.is_file()}


def main():
    output = HERE / "final_postprocessing_peer_review.json"
    if output.exists():
        raise FileExistsError("Preserve final peer review")
    sources = {}
    def bind(path):
        item = fp(path)
        if item["path"] in sources:
            assert sources[item["path"]] == item["sha256"]
        sources[item["path"]] = item["sha256"]
        return item
    def check_item(item):
        assert fp(item["path"])["sha256"] == item["sha256"]
        if "size_bytes" in item:
            assert resolve(item["path"]).stat().st_size == item["size_bytes"]
        bind(item["path"])
    # Only explicitly bound manifests are traversed; no Claude-directory scan.
    for name in ("dynamic_protocol.json", "full_support_dynamic_protocol.json",
                 "analysis/analysis_protocol.json", "analysis/phase_protocol.json",
                 "analysis/full_support_analysis_protocol.json", "analysis/full_support_phase_protocol.json"):
        p = AUDIT / name
        for label, sha in read(p)["protected_sources_before"].items():
            check_item({"path": label, "sha256": sha})
        bind(p)
    lean_path = AUDIT / "analysis/lean_numeric_summary.json"
    compact_path = AUDIT / "analysis/compact_numeric_packet.json"
    lean, compact = read(lean_path), read(compact_path)
    finite(lean); finite(compact)
    assert set(lean["cases"]) == set(compact["cases"]) == set(IDS)
    assert lean["main_comparison_cases"] == ["STANDING_FOURROI_AB07", "PYLON400_REFERENCE"]
    assert lean["secondary_case"] == "STANDING_AB07"
    assert not lean["clinical_or_hardware_necessity_identified"] and not compact["clinical_or_hardware_necessity_identified"]
    summary_receipt = read(AUDIT / "analysis/lean_summary_receipt.json")
    compact_receipt = read(AUDIT / "analysis/compact_numeric_packet_receipt.json")
    assert summary_receipt["inputs_unchanged"] and compact_receipt["sources_unchanged"]
    check_item(summary_receipt["output"]); check_item(compact_receipt["packet"])
    for item in summary_receipt["inputs"].values(): check_item(item)
    for item in compact_receipt["sources_before"].values(): check_item(item)
    assert compact_receipt["sources_before"] == compact_receipt["sources_after"]
    cases, png_count, reserve_rows = {}, 0, 0
    for name in IDS:
        metric_path = AUDIT / "analysis" / name / "metrics.json"
        phase_path = AUDIT / "analysis/phase" / name / "phase_metrics.json"
        metric, phase = read(metric_path), read(phase_path)
        finite(metric); finite(phase)
        bind(metric_path); bind(phase_path)
        check_item(metric["receipt"]); check_item(phase["receipt"])
        run = resolve(metric["receipt"]["path"]).parent
        current = inventory(run)
        assert current == metric["run_artifacts_unchanged"]["files"] == phase["run_artifacts_unchanged"]["after"]
        assert phase["run_artifacts_unchanged"]["before"] == current
        for label, sha in current.items(): check_item({"path": label, "sha256": sha})
        primary = metric["windows"]["primary_all"]
        if name == "PYLON400_REFERENCE":
            assert primary["coverage"] == "complete" and lean["cases"][name]["primary_full_three_cycle_statistics"] is not None
        else:
            assert primary["coverage"] == "unavailable" and "forces" not in primary and "reserves" not in primary
            assert lean["cases"][name]["primary_full_three_cycle_statistics"] is None
            assert compact["cases"][name]["primary_three_cycle_statistics"] is None
            assert phase["swing_impulse_fraction"]["pooled"] is None
            assert all(c["comparison"] is None and c["status"].startswith("unavailable_primary") for c in phase["primary_channels"].values())
            assert all(c["events"] is None for c in compact["cases"][name]["primary_phase"].values())
        translations = set(read(resolve(metric["receipt"]["path"]))["effective_simulator_config"]["translation_coords"])
        for window in metric["windows"].values():
            assert not window["boundary_clamping_or_extrapolation_used"]
            for coord, entry in window.get("reserves", {}).get("per_coordinate", {}).items():
                reserve_rows += 1
                linear = coord in translations
                assert entry["unit"] == ("N" if linear else "Nm")
                assert entry["impulse_unit"] == ("Ns" if linear else "Nm*s")
                assert entry["speed"]["unit"] == ("m/s" if linear else "rad/s") and entry["power_W"]["unit"] == "W"
                for field in ("impulse", "work_J"):
                    row = entry[field]
                    assert row["positive"] >= -1e-12 and row["negative"] <= 1e-12
                    assert math.isclose(row["signed"], row["positive"] + row["negative"], rel_tol=1e-12, abs_tol=1e-10)
                    assert math.isclose(row["absolute"], row["positive"] - row["negative"], rel_tol=1e-12, abs_tol=1e-10)
                for key, value in entry["work_J"].items():
                    assert math.isclose(entry["work_J_per_kg"][key], value / metric["mass_kg"], rel_tol=1e-12, abs_tol=1e-10)
        full = lean["cases"][name]
        projected = compact["cases"][name]
        assert projected["runtime"] == full["runtime"] and projected["primary_coverage"] == full["primary_coverage"]
        assert projected["primary_phase"] == full["primary_phase"]
        for target, origin in (("primary_three_cycle_statistics", "primary_full_three_cycle_statistics"),
                               ("secondary_startup_statistics_own_support", "partial_startup_only_separately_observed_statistics")):
            a, b = projected[target], full[origin]
            if a is None:
                assert b is None
                continue
            for key, value in a.items():
                if key == "biological_reserves":
                    for coord, row in value.items():
                        assert row == {k: b["all_biological_reserves"][coord][k] for k in row}
                else:
                    assert value == b[key]
        manifest_item = metric["canonical_plots"]
        check_item(manifest_item)
        manifest_path = resolve(manifest_item["path"])
        manifest = read(manifest_path)
        visual_path = manifest_path.parent / "visual_review.json"
        visual = read(visual_path); bind(visual_path)
        assert visual["all_passed"] and visual["rendering_manifest_preserved"]
        check_item(visual["rendering_manifest"])
        assert set(manifest["files"]) == set(visual["figures"]) and len(manifest["files"]) == 7
        assert len(list(manifest_path.parent.glob("*.png"))) == 7
        for filename, item in manifest["files"].items():
            check_item(item)
            assert visual["figures"][filename]["sha256"] == item["sha256"] and visual["figures"][filename]["status"] == "PASS"
        png_count += 7
        if name != "PYLON400_REFERENCE":
            assert not manifest["cycles_used"] and len(manifest["cycles_excluded_for_incomplete_support"]) == 3
        cases[name] = {"primary_coverage": primary["coverage"], "native_file_count": len(current),
                       "metrics": fp(metric_path), "phase": fp(phase_path), "canonical_plots": manifest_item,
                       "visual_review": fp(visual_path), "runtime": projected["runtime"]}
    assert png_count == 21
    fresh = AUDIT / "dynamics/PYLON400_REFERENCE/run"
    receipt = read(fresh / "receipt.json")
    old = resolve(receipt["historical_control_receipt"]["path"]).parent
    sto = list((fresh / "sim_outputs").glob("*.sto"))
    assert len(sto) == 17
    for p in sto:
        assert p.read_bytes() == (old / "sim_outputs" / p.name).read_bytes()
        bind(old / "sim_outputs" / p.name)
    differing = []
    counts = [0, 0]
    with (old / "trace.jsonl").open() as a, (fresh / "trace.jsonl").open() as b:
        for index, (ra, rb) in enumerate(itertools.zip_longest(a, b), 1):
            counts[0] += int(ra is not None); counts[1] += int(rb is not None)
            if ra is None or rb is None or identity(json.loads(ra)) != identity(json.loads(rb)):
                differing.append(index)
    assert counts == [611, 611] and not differing
    assert (old / "trace.jsonl").read_bytes() == (fresh / "trace.jsonl").read_bytes()
    bind(old / "trace.jsonl")
    for name in ("analysis/aggregate_metrics.json", "analysis/full_support_aggregate_metrics.json",
                 "analysis/phase/aggregate_phase_metrics.json", "analysis/phase/full_support_aggregate_phase_metrics.json",
                 "analysis/finalized_three_case_saved_data_diagnostics.json", "analysis/lean_summary_receipt.json",
                 "analysis/compact_numeric_packet_receipt.json", "analysis/lean_summary_attempt_01_receipt.json",
                 "analysis/assemble_lean_summary_attempt_01.py", "review/static_force_sphere_peer_review.json",
                 "review/final_standing_scientific_review.json"):
        bind(AUDIT / name)
    diagnostic = read(AUDIT / "analysis/finalized_three_case_saved_data_diagnostics.json")
    assert diagnostic["sources_unchanged"] and diagnostic["sources_before"] == diagnostic["sources_after"]
    assert diagnostic["secondary_common_observed_prefix"]["momentum_required_or_COM_closure_claim"] is None
    for item in diagnostic["sources_before"].values(): check_item(item)
    bind(Path(__file__))
    after = {label: fp(label)["sha256"] for label in sources}
    assert sources == after
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS", "cases": cases,
              "sources_before": sources, "sources_after": after, "sources_unchanged": True, "protected_source_count": len(sources),
              "reserve_entries_checked": reserve_rows, "canonical_PNG_count": png_count,
              "reserve_unit_and_sign_checks": "N/Nm, Ns/Nm*s, m/s/rad/s, W/J/Jkg; signed and absolute identities checked within floating-roundoff only, without physical acceptance thresholds",
              "fresh400": {"STO_count": 17, "all_17_byte_equal": True, "trace_rows": counts,
                           "trace_byte_equal": True, "trace_exact_decoded_fields": True, "fields_excluded": [],
                           "float_comparison": "float.hex including negative zero; types, strings and array order retained"},
              "summary_projection_and_final_receipt_bindings_pass": True,
              "assembler_attempt01": "Preserved source/receipt: no-local-path guard stopped before output; final projection omits equilibrium_failure_interpretation metadata, original physical metrics retained unchanged",
              "censored_or_unavailable_events_not_detector_failures": True,
              "visual_review_scope": "July inspected all21 PNGs; this audit independently checked those receipts, hashes, cycle eligibility and their immutable rendering manifests, without claiming a second visual inspection",
              "new_native_FK_IK_or_force_integration": False, "new_Claude_content_read": False,
              "physical_or_clinical_qualification": False,
              "limits": ["Observed prefixes cannot replace missing frozen primary cycles.",
                         "Static sphere/STL mismatch does not prove the cause of walking knee divergence.",
                         "400 completion and reproducibility do not establish physiological or hardware necessity."]}
    with output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print({"review": fp(output), "protected_sources": len(sources), "reserve_entries": reserve_rows, "canonical_PNG_count": png_count}, flush=True)


if __name__ == "__main__":
    main()
