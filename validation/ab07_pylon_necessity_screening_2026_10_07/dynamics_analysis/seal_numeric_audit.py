"""Seal all18 finalized descriptive audits after the observational watcher.

Read-only JSON/hash verification; no model/FK/environment/policy is imported.
Visual and physical-necessity judgments remain separate from numerical checks.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
AUDIT = HERE.parent


def read(p):
    return json.loads(p.read_text(encoding="utf-8"))


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def fp(p):
    return {"path": p.relative_to(REPO).as_posix(), "sha256": sha(p), "size_bytes": p.stat().st_size}


def main():
    out = HERE / "dynamics_numeric_seal.json"
    if out.exists():
        raise FileExistsError(out)
    aggregate_path = HERE / "aggregate_metrics.json"
    aggregate, dynamic = read(aggregate_path), read(AUDIT/"dynamic_protocol.json")
    protocol_path = HERE / "analysis_protocol_v2.json"
    protocol = read(protocol_path)
    if set(aggregate["cases"]) != set(dynamic["cases"]) or len(dynamic["cases"]) != 18:
        raise ValueError("All18 prospective cases must be represented; no survivor-only seal.")
    if aggregate["analysis_protocol"]["sha256"] != sha(protocol_path):
        raise ValueError("Aggregate is not bound to current frozen V2 protocol.")
    if aggregate["dynamic_protocol"]["sha256"] != sha(AUDIT/"dynamic_protocol.json"):
        raise ValueError("Aggregate is not bound to current prospective dynamic protocol.")
    protected = dict(dynamic["protected_sources_before"])
    protected.update({v["path"]: v["sha256"] for v in protocol["source_bindings"].values()})
    for p, expected in protected.items():
        if sha(REPO/p) != expected:
            raise ValueError(f"Protected source differs: {p}")
    native_artifacts = {}
    plots, cases = {}, {}
    for name, entry in aggregate["cases"].items():
        p = REPO/entry["metrics"]["path"]
        metric = read(p)
        if sha(p) != entry["metrics"]["sha256"]:
            raise ValueError("Final metrics changed after aggregation.")
        if (metric["case_id"] != name or metric["case"] != dynamic["cases"][name]
                or metric["analysis_protocol"]["sha256"] != sha(protocol_path)
                or metric["dynamic_protocol"]["sha256"] != sha(AUDIT/"dynamic_protocol.json")
                or metric["receipt"]["sha256"] != sha(AUDIT/"dynamics"/name/"run/receipt.json")):
            raise ValueError("Per-case audit identity/protocol/receipt differs from the prospective case.")
        for relative, f in metric["run_artifacts_unchanged"]["files"].items():
            if sha(REPO/relative) != f["sha256"] or (REPO/relative).stat().st_size != f["size_bytes"]:
                raise ValueError(f"Finalized native artifact changed: {relative}")
            native_artifacts[relative] = f
        pngs = []
        plot_binding = metric.get("canonical_plots")
        if plot_binding:
            manifest_path = REPO/plot_binding["path"]
            if sha(manifest_path) != plot_binding["sha256"]:
                raise ValueError("Rendered manifest changed; visual review must use a separate receipt.")
            manifest = read(manifest_path)
            if len(manifest["files"]) != 7:
                raise ValueError("Canonical plot manifest must inventory seven PNGs.")
            for binding in manifest["files"].values():
                image = REPO/binding["path"]
                if sha(image) != binding["sha256"]:
                    raise ValueError("Rendered PNG changed.")
                pngs.append(fp(image))
            plots[name] = {"manifest": fp(manifest_path), "PNG_files": pngs,
                           "visual_review_status_from_renderer": "pending_separate_review"}
        sea = metric.get("SEA_all_recorded", {})
        clamps = {motor: {"plugin_motor_torque_clamp_samples": value["motor_torque_clamp"]["samples"],
                           "plugin_integral_torque_clamp_samples": value["inner_integral_torque_clamp"]["samples"],
                           "command_near_abs_u_0p999_samples": value["normalized_control_saturation"]["samples"],
                           "command_exactly_abs_u_1_samples": value["command_limit_evidence"]["exactly_at_plus_or_minus_one"]["samples"],
                           "unclipped_command_above_unit_bound_samples": value["command_limit_evidence"]["unclipped_command_above_unit_bound"]["samples"],
                           "cascade_integral_clamp_samples": value["command_limit_evidence"]["cascade_i_clamped"]["samples"],
                           "SEA_independent_identity_checks_pass": value["all_identity_checks_pass"]}
                  for motor, value in sea.items()}
        cases[name] = {"metrics": fp(p), "runtime": metric["runtime"], "analysis_status": metric["analysis_status"],
                       "primary_coverage": metric["windows"].get("primary_all", {}).get("coverage"),
                       "native_data_finite": metric.get("numerical_data_finite"), "SEA_and_clamps_all_recorded": clamps,
                       "prefix_exact_comparison": metric["prefix_comparison"], "canonical_PNG_count": len(pngs)}
    after_failures = [p for p, expected in protected.items() if sha(REPO/p) != expected]
    if after_failures:
        raise ValueError(f"Sources changed while sealing: {after_failures}")
    for relative, binding in native_artifacts.items():
        if sha(REPO/relative) != binding["sha256"] or (REPO/relative).stat().st_size != binding["size_bytes"]:
            raise ValueError("Native artifact changed during sealing.")
    for binding in plots.values():
        for artifact in [binding["manifest"], *binding["PNG_files"]]:
            if sha(REPO/artifact["path"]) != artifact["sha256"]:
                raise ValueError("Canonical manifest/PNG changed during sealing.")
    artifacts = [aggregate_path, protocol_path, HERE/"analyze_dynamic_screen_v2.py", HERE/"watch_finalized.py",
                 HERE/"watch_receipts/start.json", HERE/"watch_receipts/completed.json", HERE/"watch_receipts/progress.jsonl",
                 HERE/"AB06_terminal_prefix_supplement.json", HERE/"AB06_terminal_prefix_source_lines.json",
                 HERE/"AB06_terminal_prefix_interpretation_addendum.json",
                 HERE/"AB06_terminal_prefix_fingerprint_timing.json",
                 HERE/"diagnose_ab06_terminal_prefix.py", HERE/"analysis_protocol.json", HERE/"analyze_dynamic_screen.py",
                 HERE/"attempt_01/failure_receipt.json", HERE/"attempt_01/P4_A1.analysis.log"]
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "all18_finalized_descriptive_numerical_audits_sealed_visual_review_separate",
              "instruction_check_token": "CMC_AGENT_OK_2026", "sealed_helper": fp(Path(__file__)),
              "case_count": len(cases), "cases": cases, "canonical_plots": plots,
              "canonical_PNG_count": sum(len(x["PNG_files"]) for x in plots.values()),
              "native_artifact_count": len(native_artifacts), "native_artifacts_unchanged": True,
              "prospective_dynamic_protected_sources_count": len(dynamic["protected_sources_before"]),
              "combined_source_binding_count": len(protected), "sources_all_match_before_and_after": True,
              "source_sha256": protected, "diagnostic_artifacts": [fp(p) for p in artifacts],
              "interpretive_limits": ["Completion, finiteness and SEA identity checks do not establish physiological recruitment, realistic tissue/joint limits or hardware pylon necessity.",
                                      "Signed reserve impulses and F*qdot work remain physical support supplied by the tracking contract; force matching alone is not a pylon-length selection criterion.",
                                      "AB06 all-row exact prefix is retained false; separate source-grounded terminal discretization supplement documents the small final-segment difference.",
                                      "Missing or partial requested-window results are retained as unavailable/partial. No clinical gates, native reruns, forcefit, policy/training or new Claude contents used.",
                                      "All canonical rendering manifests are frozen separately from later visual-review receipts. This seal does not assert visual PASS or Windows native execution."],
              "new_Claude_branch_read": False, "runtime_or_physical_parameter_changes_by_postprocessing": False}
    with out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"seal": fp(out), "case_count": len(cases), "PNG_count": result["canonical_PNG_count"]}))


if __name__ == "__main__":
    main()
