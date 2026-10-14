"""Bind the isolated standing comparison consumer before any new outputs read.

Only standard-library JSON/source/hash work occurs here. No model, simulator,
IK, event stream or numerical runtime table is loaded. The numerical laws and
window metrics are inherited unchanged from the frozen screening V2 analyzer.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
OLD = ROOT / "validation/ab07_pylon_necessity_screening_2026_10_07"
PROTOCOL = HERE / "analysis_protocol.json"
IDS = {"STANDING_AB07", "PYLON400_REFERENCE"}


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def resolve(raw):
    p = Path(raw)
    return (p if p.is_absolute() else ROOT / p).resolve()


def fp(path):
    p = resolve(path)
    try:
        label = p.relative_to(ROOT).as_posix()
    except ValueError:
        label = p.as_posix()
    return {"path": label, "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            "size_bytes": p.stat().st_size}


def ast_function(path, name):
    nodes = [node for node in ast.parse(Path(path).read_text(encoding="utf-8")).body
             if isinstance(node, ast.FunctionDef) and node.name == name]
    if len(nodes) != 1:
        raise ValueError("Require exactly one inherited function: " + name)
    return ast.dump(nodes[0], include_attributes=False)


def freeze():
    if PROTOCOL.exists():
        raise FileExistsError("Preserve the frozen standing analysis protocol")
    dynamic_path = AUDIT / "dynamic_protocol.json"
    dynamic = read(dynamic_path)
    if dynamic["status"] != "frozen_before_both_native_runs" or set(dynamic["cases"]) != IDS:
        raise ValueError("Require the prospectively frozen two-case native protocol")
    if set(dynamic["execution_order"]) != IDS or len(dynamic["execution_order"]) != 2:
        raise ValueError("Both predeclared cases must be retained exactly once")
    inherited_path = OLD / "dynamics_analysis/analysis_protocol_v2.json"
    inherited = read(inherited_path)
    analyzer = HERE / "analyze_standing_comparison.py"
    old_analyzer = OLD / "dynamics_analysis/analyze_dynamic_screen_v2.py"
    unchanged = ("supported", "source_table", "weighted", "signed_integrals", "sparse_com",
                 "tracking", "reserve_metrics", "window_metrics", "trace_audit")
    ast_parity = {name: ast_function(analyzer, name) == ast_function(old_analyzer, name)
                 for name in unchanged}
    if not all(ast_parity.values()):
        raise AssertionError("Inherited scientific numerical functions changed")
    sources = {key: fp(item["path"]) for key, item in inherited["source_bindings"].items()
               if key not in {"dynamic_protocol", "postprocessor"}}
    for key, item in inherited["source_bindings"].items():
        if key not in {"dynamic_protocol", "postprocessor"} and sources[key]["sha256"] != item["sha256"]:
            raise AssertionError("Inherited bound consumer input changed: " + key)
    sources.update(dynamic_protocol=fp(dynamic_path), postprocessor=fp(analyzer),
                   phase_consumer=fp(HERE / "analyze_standing_phase.py"),
                   freezer=fp(Path(__file__)), scope_protocol=fp(AUDIT / "protocol.json"),
                   preparation_protocol=fp(AUDIT / "preparation/preparation_protocol.json"),
                   inherited_numerical_helper=fp(old_analyzer), inherited_analysis_protocol=fp(inherited_path),
                   inherited_phase_helper=fp(OLD / "phase_timing/analyze_phase_timing_v2.py"),
                   inherited_phase_protocol=fp(OLD / "phase_timing/phase_timing_protocol_v2.json"))
    preparation = read(AUDIT / "preparation/preparation_protocol.json")
    sources["geometry_fit_receipt"] = fp(preparation["geometry_fit_receipt"]["path"])
    if sources["geometry_fit_receipt"]["sha256"] != preparation["geometry_fit_receipt"]["sha256"]:
        raise AssertionError("Geometry receipt differs from the frozen preparation binding")
    manifest = dict(dynamic["protected_sources_before"])
    for item in sources.values():
        if item["path"] in manifest and manifest[item["path"]] != item["sha256"]:
            raise AssertionError("Conflicting source binding: " + item["path"])
        manifest[item["path"]] = item["sha256"]
    for label, expected in manifest.items():
        if fp(label)["sha256"] != expected:
            raise AssertionError("Protected source changed: " + label)
    contracts = {key: inherited[key] for key in (
        "receipt_gate", "time_contract", "tracking_contract", "vertical_contract", "reserve_contract",
        "SEA_contract", "biological_contract")}
    value = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
             "instruction_check_token": "CMC_AGENT_OK_2026", "status": "frozen_before_new_numeric_event_plot_consumption",
             "source_bindings": sources, "prospective_case_ids": dynamic["execution_order"],
             "scientific_function_AST_identity_to_inherited_V2": ast_parity,
             "adaptations": ["Isolated new audit/output/plot paths and two predeclared case IDs.",
                             "Fresh400 native prefix compared to historical P3_A1 via an explicit separate receipt binding.",
                             "Prefix comparison requires identical captured numerical configurations, with only local input/output paths excluded.",
                             "Phase consumer preserves inherited original matcher,20N crossing,tolerances and censored closingHS; output paths and case count only changed.",
                             "No legacy attempt01 is claimed as an attempt in the new standing comparison."],
             **contracts,
             "root_native_run_ownership": "Root executes both native simulations. Analysis refuses active receipts; only sparse saved-state Velocity FK for COM is permitted post-finalization.",
             "prefix_contract": "PYLON400_REFERENCE only: all decoded native table times/columns/values against historical P3_A1; exact numerical prefix is reported separately from whole-file byte identity and fresh outcomes.",
             "plots": {"root": "plot/AB07_standing_geometry_fit_2026_10_07/{case}/", "count": 7,
                       "implementation": inherited["plots"]["implementation"],
                       "partial": inherited["plots"]["partial"], "visual_review": inherited["plots"]["visual_review"]},
             "metrics_schema": {**inherited["metrics_schema"], "per_case": "analysis/{case}/metrics.json",
                                "aggregate": "analysis/aggregate_metrics.json"},
             "geometry_and_fit_context": "Geometry receipt plus original12marker calibration/freshfullIK receipts remain distinct from forward metrics; dynamic contrast includes the coherent preparation intervention, not geometry alone.",
             "comparison_limits": ["Reserve-assisted diagnostics under the fixed bench/controller/contact/detector contract, not clinical or hardware necessity.",
                                   "Signed reserve cancellation cannot substitute for RMS/absolute impulse/work and every biological DOF.",
                                   "Actor FSM cycles, binary events and raw20N force events are different observations; unchanged original measured nominal windows remain diagnostic references.",
                                   "Any partial or failed recorded prefix retains its requested windows and observed support without extrapolation or promotion to success."],
             "aggregate_gate": "Both finalized cases bound to exact receipt/dynamic/analysis protocol; preserve runtime failures and partial data.",
             "protected_count": len(manifest), "protected_sources_before": manifest,
             "new_forward_numeric_outputs_read_before_this_freeze": False, "new_event_outputs_read_before_this_freeze": False,
             "new_Claude_branch_read": False, "runtime_input_modifications": False,
             "force_fit_or_outcome_retuning": False,
             "verification_so_far": "AST/source review and separately authorized geometry receipt review only. No OpenSim import, forward simulation output consumption or fresh FK by this numerical consumer before freeze."}
    with PROTOCOL.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"protocol": fp(PROTOCOL), "protected_count": len(manifest),
                      "scientific_AST_identity": ast_parity}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", action="store_true", required=True)
    parser.parse_args()
    freeze()


if __name__ == "__main__":
    main()
