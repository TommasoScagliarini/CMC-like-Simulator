"""Consolidate immutable source, XML, data and contact-pose audits.

No model modification, integrations, IK optimization or policy calls. This is
an evidence index and provenance check, not a physiological acceptance gate.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def source_fingerprints(value):
    if isinstance(value, dict):
        if isinstance(value.get("path"), str) and isinstance(value.get("sha256"), str):
            yield value
        for item in value.values():
            yield from source_fingerprints(item)
    elif isinstance(value, list):
        for item in value:
            yield from source_fingerprints(item)


def resolve_source(raw: str) -> Path:
    path = Path(raw.replace("\\", "/"))
    return path if path.is_absolute() else ROOT / path


def main() -> None:
    names = {
        "observed_pair": "observed_pair_summary.json",
        "model": "model_preservation_audit.json",
        "data": "data_preparation_audit_final.json",
        "contact": "contact_replay.json",
        "preparation_probe": "contact_preparation_probe.json",
        "before": "protected_sources_before.json",
    }
    evidence = {role: json.loads((AUDIT / name).read_text(encoding="utf-8"))
                for role, name in names.items()}
    model, data, contact = (evidence[role] for role in ("model", "data", "contact"))
    if not all(model["checks"].values()):
        raise RuntimeError("An XML preservation check did not pass.")
    if not data["source_files_unchanged_at_completion"]:
        raise RuntimeError("Data audit reported input mutation.")
    if contact["status"] != "PASS" or not contact["inputs_integrity"]["all_unchanged"]:
        raise RuntimeError("Contact replay validation or input preservation failed.")
    probe = evidence["preparation_probe"]
    if probe["validation"]["status"] != "PASS" or not probe["inputs_integrity"]["all_unchanged"]:
        raise RuntimeError("Ankle preparation probe failed validation or input preservation.")
    for subject in ("AB06", "AB07"):
        if contact["subjects"][subject]["validation"]["status"] != "PASS":
            raise RuntimeError("Contact replay validation failed: " + subject)
        checks = data["subjects"][subject]["source_parity"]
        for key in ("all_28_marker_xyz_in_m", "all_18_GRF_columns",
                    "marker_clock_s", "GRF_clock_s", "remapped_dataset_IK",
                    "remapped_dataset_IK_clock_s"):
            if checks[key]["max_abs_difference"] > 1e-8:
                raise RuntimeError("Unexpected conversion difference: " + subject + "/" + key)
    fingerprint_checks = {}
    # Recorded historical preparation receipts can contain the hash of an
    # earlier script version at the same path. Check current audit inputs,
    # rather than silently reclassifying those historical hashes as current.
    current_source_registry = {
        "root_before": evidence["before"],
        "observed_pair_source": evidence["observed_pair"]["source"],
        "model_inputs_before": model["inputs_before"],
        "model_inputs_after": model["inputs_after"],
        "model_source_evidence": model["source_evidence"],
        "model_helper": model["script"],
        "data": data,
        "contact": contact,
        "preparation_probe": probe,
    }
    fingerprint_records = list(source_fingerprints(current_source_registry))
    for record in (contact, probe):
        for phase in ("fingerprints_before", "fingerprints_after"):
            fingerprint_records.extend(
                {"path": raw, "sha256": expected}
                for raw, expected in record["inputs_integrity"][phase].items()
            )
    for item in fingerprint_records:
        path = resolve_source(item["path"])
        relative = path.relative_to(ROOT).as_posix()
        actual = digest(path)
        if actual != item["sha256"]:
            raise RuntimeError("Source hash mismatch: " + relative)
        fingerprint_checks[relative] = {"path": relative, "sha256": actual, "matches_recorded": True}
    profile_path = ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    left = [sphere for sphere in profile["spheres"] if sphere["side"] == "left"]
    if not all(sphere["material"]["exponent"] == 1.0 for sphere in left):
        raise RuntimeError("Static contact sensitivity assumes a linear left basis.")
    coefficient = sum(sphere["material"]["stiffness"] for sphere in left)
    result = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "status": "diagnostic_audit_complete_single_cause_not_identified",
        "scope": "Source/XML/saved-data analysis and contact evaluation on saved poses; no new dynamic rollout, IK solver, policy execution or input edits.",
        "question": "Does the AB06/AB07 difference arise from intrinsic anatomy or preparation?",
        "answer": "The pair changes anatomy, preparation-derived reference kinematics and contact interaction together. No unique anatomical/preparation share is identified. No accidental retained-biological or dynamic conversion mutation was found; explicit preparation conventions and an AB06-calibrated contact law remain mechanisms to isolate.",
        "observed_matched_pair": evidence["observed_pair"]["subjects"],
        "model_preservation_checks": model["checks"],
        "source_data_checks": {
            subject: {
                "source_parity": data["subjects"][subject]["source_parity"],
                "conditions": data["subjects"][subject]["conditions"],
                "marker_quality": data["subjects"][subject]["marker_quality_on_primary"],
                "IK_provenance": data["subjects"][subject]["IK_provenance"],
            } for subject in ("AB06", "AB07")
        },
        "preparation_effect_on_AB07_IK": data["subjects"]["AB07"]["saved_preparation_before_after_IK"],
        "preparation_associated_ankle_probe": {
            "methodology": probe["methodology"],
            "validation": probe["validation"],
            "comparison": probe["comparison"],
            "interpretation": probe["interpretation"],
            "finding": "At fixed registered AB07 geometry and all other current filtered coordinates, the pre-ground ankle reference raises impulse. The current preparation-associated ankle change reduces it by about20.46% relative to that probe. This excludes that isolated change as the origin of the present direct excess, but does not quantify total preparation effect.",
        },
        "contact_pose_audit": {
            "methodology": contact["methodology"],
            "contract": contact["contact_contract"],
            "subjects": contact["subjects"],
            "pair_comparison": contact["pair_comparison"],
            "quantitative_findings": contact["quantitative_findings"],
            "limitations": contact["limitations"],
            "metadata_clarifications": probe["original_replay_metadata_clarifications"],
        },
        "shared_contact_profile": {
            "path": profile_path.relative_to(ROOT).as_posix(), "sha256": digest(profile_path),
            "metadata": profile["metadata"], "left_sphere_count": len(left),
            "sum_linear_stiffness_N_per_m": coefficient,
            "conditional_static_force_increase_per_1mm_N": coefficient * 0.001,
            "sensitivity_conditions": "All eight left basis spheres active, zero normal velocity, and negligible smoothing. This is an analytic local sensitivity, not the integrated result or a new calibrated profile.",
        },
        "causal_distinctions": [
            "Healthy body mass and retained biological geometry are intrinsic inputs; total mass is preserved by the graft.",
            "Donor mass redistribution, donor COM and mass-scaled donor inertias, donor proximal-left muscles and mounting/marker calibration are preparation choices common to or derived for subjects.",
            "A preparation step can comply with its source contract and still require independent physical qualification.",
            "The identical contact profile is calibrated on AB06 states and is not an independently identified subject-independent material/sole law.",
            "Pose-only contact forces do not depend on muscle recruitment or mass; those can change dynamically attained poses indirectly.",
            "Healthy experimental left GRF is a reference for the virtual prosthesis benchmark, not an experimental recording from this virtual hardware.",
            "AB06 original receipt remains failed at its final adapter boundary; the primary physical cycles and native outputs remain intact.",
        ],
        "next_discriminating_experiments_not_executed": [
            {
                "id": "P1", "question": "Does the new preparation pipeline preserve the AB06 benchmark?",
                "design": "Build AB06 in an isolated directory from the same frozen healthy/donor/static sources, verify expected geometry/marker/IK differences, then compare to current AB06 with the same fixed controller and three preselected nominal cycles.",
                "limit": "A successful AB06 round trip excludes a broad pipeline regression but does not prove subject-specific AB07 fitting correct.",
            },
            {
                "id": "P2", "question": "How much comes from preparation on the same anatomy?",
                "design": "Keep healthy AB07, measured loading, profiles and controllers fixed; compare isolated, explicitly justified preparation variants one at a time. Preserve subject-scaled proximal muscle paths/lengths in one branch; test mounting/marker calibration separately with new IK from the same markers and independent static constraints.",
                "limit": "Changing mount geometry with frozen incompatible IK is only a pose sensitivity experiment. A dynamic preparation comparison must document IK/marker consistency for each branch.",
            },
            {
                "id": "P3", "question": "Is the common AB06 contact law transferable?",
                "design": "Evaluate a common independently justified hardware/contact model or fit any candidate on separate calibration data, then compare both subjects on frozen holdout cycles. Preserve left online/right measured application and SEA semantics.",
                "limit": "Refitting contact to the evaluated AB07 GRF and scoring on those same cycles cannot establish generalization or physical identification.",
            },
        ],
        "audit_input_integrity": {
            "original_pair_inputs": 44,
            "root_frozen_source_count": len(evidence["before"]["sources"]),
            "all_recorded_fingerprints_match": True,
            "unique_checked_source_count": len(fingerprint_checks),
            "checks": list(fingerprint_checks.values()),
            "historical_metadata_scope": "Embedded preparation-time script hashes remain historical provenance; current sources are verified against the audit's explicit before/after input registry. Historical receipts are themselves hashed and preserved.",
        },
        "evidence_files": [
            {"role": role, "path": (AUDIT / name).relative_to(ROOT).as_posix(),
             "sha256": digest(AUDIT / name)} for role, name in names.items()
        ],
        "helper": {"path": Path(__file__).resolve().relative_to(ROOT).as_posix(),
                   "sha256": digest(Path(__file__).resolve())},
    }
    output = AUDIT / "attribution_findings.json"
    with output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"output": output.relative_to(ROOT).as_posix(),
                      "sha256": digest(output), "checked_sources": len(fingerprint_checks),
                      "status": result["status"]}, indent=2))


if __name__ == "__main__":
    main()
