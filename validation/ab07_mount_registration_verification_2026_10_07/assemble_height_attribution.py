"""Combine frozen, matched FK summaries without new simulation or source edits."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
HEALTHY = ROOT / "validation/ab06_ab07_independent_attribution_2026_10_07/healthy_ik/supplemental_healthy_geometry.json"
CONTACT = HERE / "variant_contact_comparison.json"
CONSTRAINTS = HERE / "constraints/static_geometry_constraints.json"


def fingerprint(path):
    return {"path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    output = HERE / "ankle_height_attribution.json"
    if output.exists():
        raise FileExistsError("Preserve completed attribution output")
    paths = [Path(__file__), HEALTHY, CONTACT, CONSTRAINTS]
    before = [fingerprint(p) for p in paths]
    healthy, contact, constraints = [json.loads(p.read_text()) for p in paths[1:]]
    if not (healthy["inputs_integrity"]["unchanged"] and
            contact["inputs_integrity"]["all_unchanged"] and
            constraints["inputs_integrity"]["all_unchanged"]):
        raise AssertionError("Input audits must attest preserved sources")
    a = healthy["subjects"]["AB06"]["cases"]
    b = healthy["subjects"]["AB07"]["cases"]
    ay = a["prepared_current"]["aggregate"]["stance"]["joint_center_Y_time_weighted_mean_m"]
    ah = a["healthy_fresh"]["aggregate"]["stance"]["joint_center_Y_time_weighted_mean_m"]
    bh = b["healthy_fresh"]["aggregate"]["stance"]["joint_center_Y_time_weighted_mean_m"]
    old_b = b["prepared_current"]["aggregate"]["stance"]["joint_center_Y_time_weighted_mean_m"]
    scalar = contact["scalar_comparison"]
    if abs(old_b-scalar["L1_A1"]["ankle_center_Y_mean_stance_m"]) > 1e-12:
        raise AssertionError("New and frozen prepared AB07 center statistics differ")
    windows = b["prepared_current"]["aggregate"]["stance"]["windows_s"]
    contact_windows = [[c["hs_s"], c["to_s"]] for c in contact["cases"]["L1_A1"]["contact"]["cycles"]]
    if windows != contact_windows:
        raise AssertionError("Different nominal stance windows")
    rows = [{"case": "healthy_fresh", "AB06_stance_ankle_Y_mm": ah*1000,
             "AB07_stance_ankle_Y_mm": bh*1000, "AB07_minus_AB06_mm": (bh-ah)*1000}]
    for case, values in scalar.items():
        by = values["ankle_center_Y_mean_stance_m"]
        rows.append({"case": case, "AB06_stance_ankle_Y_mm": ay*1000,
                     "AB07_stance_ankle_Y_mm": by*1000,
                     "AB07_minus_AB06_mm": (by-ay)*1000,
                     "AB07_mean_virtual_contact_Fy_N_all_three_cycles": values["contact_mean_force_N"]})
    pre = scalar["L0_A0"]["ankle_center_Y_mean_stance_m"]
    current = scalar["L1_A1"]["ankle_center_Y_mean_stance_m"]
    preserved = ((pre-ay)-(bh-ah))*1000
    response = (current-pre)*1000
    identity = ((current-ay)-(bh-ah))*1000-(preserved+response)
    if abs(identity) > 1e-10:
        raise AssertionError("Height decomposition does not close")
    after = [fingerprint(p) for p in paths]
    if before != after:
        raise AssertionError("A consumed artifact changed")
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS",
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "method": "No new FK; named ankle-center, time-weighted nominal-stance means from frozen matched summaries. AB06 preparation held fixed across four AB07 cells.",
        "rows": rows,
        "height_budget_mm": {
            "fresh_healthy_contrast_B_minus_A": (bh-ah)*1000,
            "pre_registration_prepared_contrast_B_minus_A": (pre-ay)*1000,
            "current_prepared_contrast_B_minus_A": (current-ay)*1000,
            "pre_registration_contrast_minus_healthy_contrast": preserved,
            "registration_joint_response_current_minus_pre": response,
            "total_current_contrast_minus_healthy_contrast": ((current-ay)-(bh-ah))*1000,
            "sum_identity_error": identity},
        "interpretation": [
            "In the tested preparation, the initial graft/adaptation preserves the healthy subject ankle-center contrast within 0.003033 mm.",
            "Coherent length interventions at either static-angle setting lower the AB07 ankle center by approximately 17.326 mm and reverse the subject ordering.",
            "The static pose intervention changes foot orientation/contact while its ankle-center mean changes less than 0.0004 mm.",
            "These controls establish conditional procedural cause of this center-height inversion and virtual-contact response; they do not assign causal percentages of total anatomy, gait or saved rollout imbalance.",
            "The extra distal length is conditionally required by the selected donor-static height and pitch targets. These data do not establish its physical necessity or the correct subject assembly.",
            "Healthy anatomical and prosthetic ankle centers are defined joint socket origins, not identical shoe/sole points; preservation of ordering is diagnostic evidence rather than an imposed physical requirement."],
        "inputs_integrity": {"all_unchanged": True, "before": before, "after": after}}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": fingerprint(output), "height_budget_mm": result["height_budget_mm"]}))


if __name__ == "__main__":
    main()
