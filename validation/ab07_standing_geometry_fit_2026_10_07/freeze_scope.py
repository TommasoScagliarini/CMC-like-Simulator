"""Freeze the authorized standing-fit scope before any new fit or solver output."""
from pathlib import Path
import json
import hashlib
from datetime import datetime, timezone

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[1]
OLD = ROOT / "validation/ab07_pylon_necessity_screening_2026_10_07"

def read(path):
    return json.loads(Path(path).read_text())

def fp(path):
    path = Path(path).resolve()
    return {"path": path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

def main():
    output = AUDIT / "protocol.json"
    if output.exists():
        raise FileExistsError("Preserve the frozen scope")
    previous = read(OLD / "dynamic_protocol.json")
    protected = dict(previous["protected_sources_before"])
    prep = read(OLD / "preparation/preparation_protocol.json")
    roles = prep["source_roles"]
    phase = read(OLD / "phase_timing/aggregate_phase_metrics_v2.json")
    extra = [Path(__file__), ROOT / "AGENTS.md", OLD / "protocol.json", OLD / "dynamic_protocol.json",
             OLD / "final_synthesis_receipt.json", OLD / "dynamics_analysis/aggregate_metrics.json",
             OLD / "phase_timing/aggregate_phase_metrics_v2.json", OLD / "dynamics/P3_A1/prepared_receipt.json",
             OLD / "dynamics/P3_A1/run/receipt.json", OLD / "dynamics_analysis/P3_A1/metrics.json",
             ROOT / phase["cases"]["P3_A1"]["metrics"]["path"],
             OLD / "preparation/cases/P3_A1/receipt.json", ROOT / "models/AB07-raw/osimxml/ab07.osim",
             ROOT / "Geometry/AM_foot_l.STL"]
    extra += [ROOT / item["path"] for item in roles.values()]
    extra += [path for path in (OLD / "dynamics/P3_A1/run/sim_outputs").iterdir() if path.is_file()]
    for path in extra:
        item = fp(path)
        if item["path"] in protected and protected[item["path"]] != item["sha256"]:
            raise AssertionError("Conflicting previous source seal: " + item["path"])
        protected[item["path"]] = item["sha256"]
    for name, digest in protected.items():
        path = Path(name) if Path(name).is_absolute() else ROOT / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise AssertionError("Changed source: " + name)
    selection = read(ROOT / roles["frozen_selection"]["path"])["subjects"]["AB07"]
    obj = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "scope_and_constraints_frozen_before_new_geometry_pipeline_or_physics_outputs",
        "user_objective": "Prepare AB07 using standing healthy knee target and common grounded foot; derive structural mount distances; compare forward reserve/detector results with400mm; collaborate with Claude",
        "primary_case": "STANDING_AB07", "comparator_case": "PYLON400_REFERENCE",
        "geometry": {
            "healthy_reference": "Complete23-coordinate median of actual AB07 healthy raw static acquisition; target is direct FK healthy knee child-frame groundY in that SAME representative pose",
            "pose_contract": "Quiet standing acquisition representative, not knee-locked zero; preserve healthy biological/pelvis/hip/right pose",
            "ground_plane_Y_m": 0.0,
            "ground_status": "Encoded model/acquisition benchmark convention, not independently calibrated physical floor",
            "foot": "Identical fixed AM_foot_l geometry and fixed SEA-foot hardware",
            "heel_region": "localX <= xmin +0.25*(xmax-xmin)",
            "forefoot_region": "localX >= xmax -0.35*(xmax-xmin)",
            "support_constraints": ["heel minY=0", "forefoot minY=0", "whole mesh no penetration"],
            "adjustable_parameters": ["osseo parent translation.Y", "pros_ankle parent translation.Y", "static pros_ankle reference angle only"],
            "root_choice": "Nearest healthy ankle reference root within unchanged original joint range",
            "preserved": ["anatomy and biological pose", "all prosthetic meshes/hardware", "all joint axes/defaults/ranges/orientations", "SEA controls/semantics", "benchmark body masses/COM/inertias", "all contact and detector profiles/calibrations"],
            "failure_policy": "Preserve geometric failure; any added mount freedom needs an explicit prospective geometry supplement before its output and before dynamics",
            "assembly_scope": "Joint mounting geometry; no physical material or remachined mesh mass identification"},
        "pipeline": {
            "original_static_markers": True, "calibration_markers_count": 12,
            "static_mapping": "Raw biological static rows/clocks preserved; knee and ankle receive ONLY prepared_reference minus healthy_reference from SAME fit receipt",
            "fresh_full_marker_IK": True, "full_time_range_s": [10.36, 153.415],
            "automatic_donor_ground_registration": False, "RRA": False,
            "diagnostic_ID": "Optional original tool diagnostic with declared force exclusions; never the forward reserve evidence",
            "no_dynamic_retuning_or_training": True},
        "comparison": {
            "fresh_dynamics_both_cases": True,
            "400_reference_source": fp(OLD / "preparation/cases/P3_A1/receipt.json"),
            "reference_reproduction": "Require frozen model/IK and runtime provenance; retain and quantify any nonidentity rather than hide it",
            "runtime": "Same original prescribed_action/ownfilteredIK, biological muscles and reserves, actual SEA nonimpedance spring/plugin, cascade, rk4_bypass1ms, seed123",
            "reset_s": selection["reset_hs_s"], "episode_duration_s": 6.11,
            "setup_horizon": "Unchanged old9s matched setup horizon", "primary_cycles": selection["primary_cycles"],
            "measures": ["actual virtual left contact", "left momentum-required load", "signed/absolute reserve impulse and work plusRMS", "tracking/all biological reserves", "SEA command/motor clamp", "binary and retained actor events/cycles", "20N force events with original HS120ms/TO180ms matching"],
            "late_guard_censoring": "Preserve original closingHS right-censoring",
            "failure_policy": "One physical attempt per case; preserve interrupted/failed and partial coverage; no outcome-based changes"},
        "outputs": {"scope": AUDIT.relative_to(ROOT).as_posix(),
                    "canonical_plots": "plot/AB07_standing_geometry_fit_2026_10_07/{case}/", "canonical_plot_count_per_case": 7},
        "claude": {"requested_model": "claude-fable-5-1", "requested_effort": "max", "tools": [],
                   "external_content_scope": "Persistent limited user consent: numerical inputs/control aggregates only; no code, raw series, credentials, file access or local non-numerical argument outside that scope",
                   "local_findings_frozen_before_external_verdict_comparison": True},
        "source_roles": roles, "original_and_previous_audit_sources_before": protected,
        "protected_count": len(protected), "production_modifications": False}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(obj, stream, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps({"protocol": fp(output), "protected_count": len(protected), "created_utc": obj["created_utc"]}))

if __name__ == "__main__":
    main()
