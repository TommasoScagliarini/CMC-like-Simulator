#!/usr/bin/env python3
"""Read-only Codex preparation provenance and planned isolated rebuild contract.

Standard-library only. Does not import operational modules, MATLAB/OpenSim,
run a build/IK/rollout, or inspect the independent Claude branch. New JSONs
are exclusive-created beside this helper; existing outputs are never replaced.
Run from any cwd: python -B path/to/preparation_provenance_audit.py
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AUDIT = HERE.parent
HEALTHY = {"AB06": "models/AB06-raw/osimxml/AB06.osim",
           "AB07": "models/AB07-raw/osimxml/ab07.osim"}
FINALS = {s: f"models/{s}_SEASEA_Threadmill/{s}_SEASEA_stiff321_500_pi.osim"
          for s in HEALTHY}
SESSIONS = {"AB06": "10_09_18", "AB07": "10_14_18"}
STATIC = {"AB06": "static_01", "AB07": "static01"}
REFERENCE_MOTION = "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot"
PLANE = "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
SCRIPTS = ["tools/build_abx_seasea_pipeline.py", "tools/fit_seasea_subject.py",
           "tools/register_seasea_ground.py", "tools/calibrate_ab06_seasea_markers.py",
           "scripts/run_opensim_sea_pipeline.py", "tools/convert_epic_ab06_to_opensim.m",
           "tools/export_ab06_healthy_overlay.m"]
REPORTS = ["reports/user/2026-05-11_ab06_seasea_epic_pipeline.md",
           "reports/daily/2026-05-12_daily-report.md",
           "reports/user/2026-10-06_preparazione_ab07_seasea_pipeline_e_qualifica_v26.md",
           "reports/user/2026-10-06_correzione_fitting_ab07_registrazione_piede_terreno.md"]


def relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def fingerprint(path: Path) -> dict:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": relative(path), "size_bytes": path.stat().st_size,
            "sha256": digest.hexdigest()}


def canonical(node: ET.Element) -> tuple:
    tokens = (node.text or "").split()
    try:
        value = tuple(float(token) for token in tokens)
    except ValueError:
        value = tuple(tokens)
    return node.tag, tuple(sorted(node.attrib.items())), value, tuple(canonical(c) for c in node)


def named(model: ET.Element, set_name: str) -> dict:
    return {n.get("name"): n for n in model.findall(f"{set_name}/objects/*")}


def muscle_canonical(node: ET.Element) -> tuple:
    copy = deepcopy(node)
    # OpenSim serialization changes GeometryPath component name from
    # "geometrypath" to "path" without changing any physical muscle property.
    for geometry in copy.iter("GeometryPath"):
        geometry.set("name", "SERIALIZATION_COMPONENT_NAME")
        # Appearance moves before PathPointSet/PathWrapSet when serialized.
        # Normalize this property order only; never reorder path points/wraps.
        geometry[:] = sorted(geometry, key=lambda child: child.tag)
    return canonical(copy)


def model(path: str) -> ET.Element:
    return ET.parse(ROOT / path).getroot().find("Model")


def parent_translation(joint: ET.Element) -> list[float]:
    socket = joint.findtext("socket_parent_frame")
    return list(map(float, joint.findtext(f"frames/PhysicalOffsetFrame[@name='{socket}']/translation").split()))


def storage_metadata(path: Path) -> dict:
    first = last = None
    rows = 0
    labels = []
    header = []
    phase = 0
    with path.open(encoding="utf-8", errors="strict") as stream:
        for line in stream:
            if phase == 0:
                header.append(line.strip())
                if line.strip().lower() == "endheader":
                    phase = 1
            elif phase == 1:
                if line.strip():
                    labels = line.split()
                    phase = 2
            elif line.strip():
                t = float(line.split()[0])
                first = t if first is None else first
                last = t
                rows += 1
    return {"path": relative(path), "rows": rows, "time_start_s": first,
            "time_end_s": last, "labels": labels, "header": header}


def function_evidence(path: str, names: list[str]) -> list[dict]:
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    nodes = {n.name: n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    return [{"file": path, "function": name, "line_start": nodes[name].lineno,
             "line_end": nodes[name].end_lineno} for name in names]


def xml_contract(subject: str) -> dict:
    raw, final, donor = model(HEALTHY[subject]), model(FINALS[subject]), model(FINALS["AB06"])
    raw_bodies, bodies = named(raw, "BodySet"), named(final, "BodySet")
    raw_joints, joints = named(raw, "JointSet"), named(final, "JointSet")
    raw_forces, forces, donor_forces = (named(m, "ForceSet") for m in (raw, final, donor))
    common_bodies = sorted(raw_bodies.keys() & bodies.keys())
    common_joints = sorted(raw_joints.keys() & joints.keys())
    hip = deepcopy(raw_joints["hip_l"])
    replacements = {"femur_l_offset": "transfemur_offset", "/bodyset/femur_l": "/bodyset/transfemur"}
    for element in hip.iter():
        if element.get("name") == "femur_l_offset":
            element.set("name", "transfemur_offset")
        if element.text:
            for old, new in replacements.items():
                element.text = element.text.replace(old, new)
    muscles = [n for n in forces if forces[n].tag.endswith("Muscle")]
    preserved_muscles = sorted(n for n in muscles if n in raw_forces and muscle_canonical(forces[n]) == muscle_canonical(raw_forces[n]))
    donor_muscles = sorted(n for n in muscles if n not in preserved_muscles and n in donor_forces and muscle_canonical(forces[n]) == muscle_canonical(donor_forces[n]))
    sea = {name: {child.tag: (child.text or "").strip() for child in node}
           for name, node in forces.items() if node.tag == "SeriesElasticActuator"}
    marker_sets = [named(m, "MarkerSet") for m in (raw, final)]
    marker_diff = []
    for name in sorted(marker_sets[0].keys() & marker_sets[1].keys()):
        a, b = marker_sets[0][name], marker_sets[1][name]
        if canonical(a) != canonical(b):
            marker_diff.append({"marker": name, "healthy_parent": a.findtext("socket_parent_frame"),
                                "prepared_parent": b.findtext("socket_parent_frame"),
                                "healthy_location_m": a.findtext("location"),
                                "prepared_location_m": b.findtext("location")})
    mass = lambda nodes: sum(float(n.findtext("mass")) for n in nodes.values() if n.tag == "Body")
    return {"subject": subject, "healthy_model": HEALTHY[subject], "production_model": FINALS[subject],
            "healthy_mass_kg": mass(raw_bodies), "production_mass_kg": mass(bodies),
            "retained_own_healthy_bodies": common_bodies,
            "retained_bodies_exact": all(canonical(raw_bodies[n]) == canonical(bodies[n]) for n in common_bodies),
            "own_healthy_unaffected_joints": [n for n in common_joints if n != "hip_l"],
            "unaffected_joints_exact": all(canonical(raw_joints[n]) == canonical(joints[n]) for n in common_joints if n != "hip_l"),
            "hip_preserved_after_body_name_remap": canonical(hip) == canonical(joints["hip_l"]),
            "mount_parent_translations_m": {n: parent_translation(joints[n]) for n in ("osseo", "pros_knee", "pros_ankle")},
            "muscle_count": len(muscles), "own_healthy_muscles_exact": preserved_muscles,
            "muscle_comparison_normalization": "Numeric XML tokens; GeometryPath component name geometrypath/path and its three property child order normalized. All muscle properties/path points and point/wrap order are compared.",
            "donor_muscles_that_differ_from_own_healthy": donor_muscles,
            "unclassified_muscle_changes": sorted(set(muscles) - set(preserved_muscles) - set(donor_muscles)),
            "sea_properties": sea, "changed_markers": marker_diff,
            "limitations": ["The 27 donor proximal-left muscles also differ from healthy AB06; grafting does not preserve complete subject-specific muscle geometry.",
                            "Prosthetic bodies retain donor COM/meshes and mass-proportional donor inertia; total mass preservation is a benchmark convention, not identified hardware mass.",
                            "Remapped/calibrated markers are virtual anatomical observations, not measured prosthetic hardware fiducials."]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-executable", default=sys.executable)
    parser.add_argument("--matlab-executable", default="/Applications/MATLAB_R2024a.app/bin/matlab" if sys.platform == "darwin" else "matlab")
    args = parser.parse_args()
    plugin = "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib" if sys.platform == "darwin" else "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll" if sys.platform == "win32" else "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.so"
    paths = SCRIPTS + REPORTS + list(HEALTHY.values()) + list(FINALS.values()) + [REFERENCE_MOTION, PLANE, "Geometry/AM_foot_l.STL", plugin,
        "models/AB06_SEASEA-raw/osimxml/iksetup.xml", "models/AB06_SEASEA-raw/osimxml/idsetup.xml",
        "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim", "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim",
        "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.marker_calibration.csv",
        "models/AB07_SEASEA_Threadmill/pipeline_report.json",
        "validation/ab06_ab07_causal_investigation_2026_10_07/model/signed_marker_vertical_audit.json"]
    for subject in HEALTHY:
        session = f"models/{subject}-raw/{SESSIONS[subject]}"
        for task, trial in (("static", STATIC[subject]), ("treadmill", "treadmill_01_01")):
            for role in ("markers", "ik", "fp", "conditions", "id", "ik_offset"):
                p = f"{session}/{task}/{role}/{trial}.mat"
                if (ROOT / p).is_file():
                    paths.append(p)
        for p in (ROOT / f"models/{subject}_SEASEA_Threadmill").rglob("*"):
            if p.is_file() and p.suffix != ".DS_Store":
                paths.append(relative(p))
    for p in (ROOT / "models/SEASEA/data").rglob("*"):
        if p.is_file() and not p.name.startswith("."):
            paths.append(relative(p))
    paths.append(relative(Path(__file__)))
    consumed = [ROOT / p for p in sorted(set(paths))]
    before = {relative(p): fingerprint(p) for p in consumed}
    now = datetime.now(timezone.utc).isoformat()
    receipt = json.loads((ROOT / "models/AB07_SEASEA_Threadmill/pipeline_report.json").read_text())
    contracts = {s: xml_contract(s) for s in HEALTHY}
    commands = {}
    for subject in HEALTHY:
        target = AUDIT / "codex" / "rebuild" / subject
        argv = [args.python_executable, "-B", str(ROOT / SCRIPTS[0]), "--subject", subject,
                "--models-dir", str(ROOT / "models"), "--task", "treadmill", "--trial", "treadmill_01_01",
                "--static-trial", STATIC[subject], "--side", "left",
                "--output-models-dir", str(target / "models"), "--bundle-dir", str(target / "bundle"),
                "--results-dir", str(target / "results"), "--model-stem", f"{subject}_SEASEA_stiff321_500_pi",
                "--donor-model", str(ROOT / FINALS["AB06"]), "--fit-reference-model", str(ROOT / HEALTHY["AB06"]),
                "--ground-reference-motion", str(ROOT / REFERENCE_MOTION), "--ground-plane-profile", str(ROOT / PLANE),
                "--foot-mesh", str(ROOT / "Geometry/AM_foot_l.STL"), "--support-data", str(ROOT / "models/SEASEA/data"),
                "--plugin", str(ROOT / plugin), "--python", args.python_executable,
                "--matlab", args.matlab_executable, "--matlab-nojvm", "--default-duration", "9.01", "--skip-smoke-test"]
        commands[subject] = {"argv": argv, "posix_display": shlex.join(argv),
                             "windows_display_same_argv": subprocess.list2cmdline(argv),
                             "portability": "Rebase root and supply local Python, MATLAB and platform plugin. Displaying Windows quoting of macOS paths does not qualify Windows execution.",
                             "execute_only_after_root_assignment": True,
                             "output_root": relative(target), "output_root_exists_now": target.exists(),
                             "expected_bundle_model": relative(target / "bundle" / f"{subject}_SEASEA_stiff321_500_pi.osim"),
                             "expected_raw_models_root": relative(target / "models" / f"{subject}_SEASEA-raw"),
                             "expected_ik_root": relative(target / "results" / f"{subject.lower()}_seasea_treadmill_treadmill_01_01_ik")}
    evidence = function_evidence(SCRIPTS[0], ["mass_preserving_targets", "graft_bodyset", "subject_specific_hip_joint", "graft_jointset", "graft_forceset", "graft_markers", "freeze_preparation_references", "run_matlab_convert", "run_ground_registration", "run_marker_calibration", "run_ik_pipeline", "build_subject", "parse_args", "main"])
    evidence += function_evidence(SCRIPTS[1], ["_anatomy", "fit_seasea_subject"])
    evidence += function_evidence(SCRIPTS[2], ["compute_registration", "main"])
    evidence += function_evidence(SCRIPTS[3], ["calibrate_markers", "parse_args"])
    evidence += function_evidence(SCRIPTS[4], ["build_ik_setup", "run_opensim_tool", "run_pipeline", "main"])
    protocol = {
        "schema_version": 1, "branch": "Codex", "generated_utc": now,
        "scope": "Independent read-only preparation provenance; no new build/IK/rollout or access to independent Claude branch.",
        "source_evidence": evidence, "source_fingerprints": before,
        "production_xml_contracts": contracts,
        "observed_procedure_comparison": [
            {"stage": "healthy anatomy", "AB06": "Own healthy EPIC model retained outside removed left chain; subject hip retained/remapped.", "AB07": "Same retention rule from own healthy EPIC model.", "common_rebuild": "Same subject-specific retention rule, verify XML whitelist."},
            {"stage": "SEA/muscle donor and mass", "AB06": "Legacy prosthetic chain with donor proximal muscles; mass-preserving redistribution May 2026; later PI321/500 SEA tuning.", "AB07": "Copies current AB06 PI321/500 hardware, SEA and proximal-left muscles; same mass-preserving donor-ratio rule.", "common_rebuild": "Same immutable current PI donor; own total mass, donor COM/meshes and proportional inertia. Does not restore full subject-specific muscle anatomy."},
            {"stage": "anatomical mounting fit", "AB06": "Legacy fixed donor mounts; no current fit receipt.", "AB07": "Healthy femur-scale and shank-length ratios relative to AB06 adjust osseo Y and distal ankle Y only.", "common_rebuild": "Required on both; healthy AB06 compared with itself makes the fitting ratios exactly one. Entire preparation still requires verification."},
            {"stage": "static hardware registration", "AB06": "Legacy benchmark pose; no groundfit receipt. Its saved static motion defines the shared donor target.", "AB07": "Required donor-relative median STL clearance and sagittal pitch before marker fit; ankle mount Y and static-only ankle pose adjusted.", "common_rebuild": "Same frozen donor, static motion, binary plane and mesh; preserve source files, dynamic coordinate zero/default/range, controller and contact law."},
            {"stage": "marker calibration", "AB06": "12 left markers calibrated from own static TRC and remapped dataset IK, without the later explicit groundfit stage.", "AB07": "Same free local-coordinate averaging, after registered static geometry/ankle pose.", "common_rebuild": "Same 12 marker names, stride 1, complete overlapping static samples. Static fit is training data, not independent ground validation."},
            {"stage": "full dynamic IK", "AB06": "Real OpenSim full-span marker IK selected May 12; full-span RRA model explicitly rejected.", "AB07": "Fresh full-span OpenSim marker IK after groundfit and marker fit, no RRA.", "common_rebuild": "Mandatory fresh marker IK, accuracy 1e-5, all embedded markers weight 1, constraint_weight Inf, full original acquisition range; no reused-remapped fallback, ScaleTool, ID/RRA or integration."},
            {"stage": "data clocks", "AB06": "Historical converted static motion 0–3.990 s at 10 ms; raw Header clock 0–1.995 s at 5 ms, same 400 acquisition rows (separate saved audit).", "AB07": "Current preparation reads original Header clocks; static 0–2.175 s at 5 ms, 436 rows.", "common_rebuild": "Current converter Header-first for both. Verify row matching, all TRC/MOT clocks and GRF bounds; do not infer whole-pipeline AB06 identity solely from fit ratios."}
        ],
        "production_AB07_numerical_receipts": {"prosthesis_fit": receipt["prosthesis_fit"], "ground_registration": receipt["ground_registration"], "ik_provenance": receipt["ik_provenance"]},
        "historical_code_hash_policy": {"recorded_executed_registration_script": receipt["ground_registration"]["sources"]["script"],
                                         "current_observed_registration_script": before[SCRIPTS[2]],
                                         "interpretation": "Historical execution hash remains historical; current before/after checks use currently observed source bytes. No retrospective substitution."},
        "production_storage_metadata": {s: {"IK": storage_metadata(ROOT / f"models/{s}_SEASEA_Threadmill/data/IK_results_{s}_SEASEA.mot"),
                                           "GRF": storage_metadata(ROOT / f"models/{s}_SEASEA_Threadmill/data/{s}_SEASEA_GRF_FullSpan.mot")} for s in HEALTHY},
        "AB06_ground_reference_storage_metadata": storage_metadata(ROOT / REFERENCE_MOTION),
        "static_force_data_present": {s: (ROOT / f"models/{s}-raw/{SESSIONS[s]}/static/fp/{STATIC[s]}.mat").exists() for s in HEALTHY},
        "planned_rebuild_commands": commands,
        "common_flags": {"task": "treadmill", "trial": "treadmill_01_01", "side": "left", "default_duration_s": 9.01,
                         "skip_smoke_test": True, "run_rra": False, "skip_opensim_ik": False,
                         "force": False, "all_subjects": False, "matlab_nojvm": True,
                         "donor": FINALS["AB06"], "healthy_fit_reference": HEALTHY["AB06"],
                         "static_ground_reference": REFERENCE_MOTION, "plane": PLANE,
                         "mesh": "Geometry/AM_foot_l.STL", "support_data": "models/SEASEA/data", "plugin": plugin},
        "isolation_contract": ["Only new per-subject output directories in codex/rebuild; no operational promotion.",
                               "Current builder snapshots donor/healthy/static references once per build with verified hashes; both builds must use the same pre-audited original reference hashes.",
                               "No --force, no --all-subjects; assert target roots absent before execution.",
                               "Record argv, source hashes, Python/MATLAB/OpenSim versions, solver backend and all build receipts; compare both subjects under one software/tool contract. IK marker weights 1, constraint Inf, accuracy 1e-5, coordinate_file empty, ScaleTool skipped.",
                               "Discovery must resolve sessions 10_09_18/10_14_18, healthy models AB06.osim/ab07.osim and exactly the explicit static/treadmill trial names; fail on drift from this source contract.",
                               "Final setup starts at original trial start and ends after default 9.01 s; downstream paired nominal-HS protocol needs explicit isolated setup timing and is not a build result.",
                               "Dynamic integration, policy/training, controller and plugin semantics are outside this protocol."],
        "post_build_acceptance_before_any_dynamics": ["Protected production/source/profile/plugin hashes unchanged.",
            "All new receipt references resolve inside subject output except declared immutable reference/source files.",
            "SEA XML equal donor; seven retained body and seven unaffected joint XML preserved; hip only name remap; muscles explicit donor/own inventory.",
            "Anatomical fit and groundfit protected-XML guards PASS, finite geometry, static-only offset and unchanged ranges/defaults.",
            "Registered static and dynamic marker residuals report sign/units; low marker RMS alone cannot validate virtual hardware ground placement.",
            "Fresh actual IK output provenance hash matches solver model/TRC and full Header time coverage; all names exactly once, finite q, both subject software/tool contracts equal.",
            "Read-only comparison of legacy/new AB06 and current/new AB07 geometry/full IK using frozen cycle windows and profiles can isolate observed procedural differences; no automatic anatomy/preparation causal fraction."]
    }
    after = {relative(p): fingerprint(p) for p in consumed}
    changed = [p for p in before if before[p] != after[p]]
    if changed:
        raise RuntimeError(f"Protected inputs changed: {changed}")
    protocol["protected_inputs_check"] = {"count": len(before), "changed": [], "pass": True}
    payloads = {"sources_before.json": {"generated_utc": now, "branch": "Codex", "sources": before},
                "preparation_provenance_protocol.json": protocol}
    if any((HERE / name).exists() for name in payloads):
        raise FileExistsError("Provenance outputs already exist; no overwrite supported.")
    for name, payload in payloads.items():
        with (HERE / name).open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps({"status": "read_only_protocol_ready_no_build", "protected_count": len(before),
                      "unchanged": True, "outputs": [fingerprint(HERE / n) for n in payloads]}, indent=2))


if __name__ == "__main__":
    main()
