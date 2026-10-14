"""Read-only independent AB08 static calibration audit (no solver/dynamics)."""
from __future__ import annotations
import csv
import hashlib
import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import opensim as osim
from tools.calibrate_ab06_seasea_markers import (parse_trc, parse_motion, load_plugin,
    set_state_from_motion, from_vec3, DEFAULT_MARKERS)
from tools.register_seasea_ground import _vertices, _pose_geometry, _protected
osim.Logger.setLevelString("error")
BASE = ROOT / "models/AB08_SEASEA-raw"
STATIC = BASE / "data/converted/static/static01"
MODEL = BASE / "osimxml/AB08_SEASEA_marker_calibrated.osim"
BUNDLE = ROOT / "models/AB08_SEASEA_Threadmill/AB08_SEASEA_stiff321_500_pi.osim"
PREMODEL = BASE / "osimxml/AB08_SEASEA.osim"
ANATOMICAL = BASE / "osimxml/AB08_SEASEA_anatomical.osim"
MOTION = BASE / "osimxml/AB08_SEASEA_static_ground_registered.mot"
ORIG = STATIC / "static01_ik_dataset_ab06_seasea.mot"
RECEIPT = json.loads((BASE / "osimxml/AB08_SEASEA_ground_registration.json").read_text())
PLUGIN = ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
load_plugin(PLUGIN)
trc, registered, original = parse_trc(STATIC / "static01.trc"), parse_motion(MOTION), parse_motion(ORIG)
rows = list(csv.DictReader((BASE / "osimxml/AB08_SEASEA_marker_calibrated.marker_calibration.csv").open()))
model = osim.Model(str(MODEL)); state = model.initSystem()
names = list(trc.markers)
markers = model.getMarkerSet()
errors, locals_ = {n: [] for n in names}, {n: [] for n in DEFAULT_MARKERS}
for row in registered.values:
    time = float(row[0])
    set_state_from_motion(model, state, dict(zip(registered.labels[1:], row[1:])), registered.in_degrees)
    i = int(np.argmin(abs(trc.times-time)))
    assert abs(float(trc.times[i])-time) < 1e-9
    for name in names:
        marker = markers.get(name)
        measured = trc.markers[name][i]
        predicted = from_vec3(marker.getLocationInGround(state))
        errors[name].append(float(np.linalg.norm(predicted-measured)))
        if name in locals_:
            local = marker.getParentFrame().getTransformInGround(state).shiftBaseStationToFrame(osim.Vec3(*map(float,measured)))
            locals_[name].append(from_vec3(local))
def stats(v):
    v = np.array(v)
    return {"mean_mm": float(v.mean()*1000), "rms_mm": float(np.sqrt(np.mean(v*v))*1000),
            "max_mm": float(v.max()*1000), "p95_mm": float(np.quantile(v,.95)*1000)}
per_marker = {name: stats(values) for name,values in errors.items()}
centroid = {}
for row in rows:
    n = row["marker"]; fitted=np.mean(locals_[n],axis=0)
    recorded=np.array([float(row[f"new_location_{a}"]) for a in "xyz"])
    actual=from_vec3(markers.get(n).get_location())
    centroid[n] = {"recomputed_vs_csv_m": float(np.linalg.norm(fitted-recorded)),
                  "actual_vs_csv_m": float(np.linalg.norm(actual-recorded)),
                  "recomputed_rms_vs_csv_m": abs(per_marker[n]["rms_mm"]/1000-float(row["new_fit_rms_m"]))}
trees = {"healthy":ET.parse(ROOT/"models/AB08-raw/osimxml/AB08.osim"),
         "anatomical":ET.parse(ANATOMICAL), "registered":ET.parse(PREMODEL),
         "calibrated":ET.parse(MODEL), "bundle":ET.parse(BUNDLE)}
def marker_map(tree):
    return {m.get("name"):{"parent":m.findtext("socket_parent_frame"),
            "local":[float(x) for x in m.findtext("location").split()]}
            for m in tree.findall(".//MarkerSet/objects/Marker")}
maps = {k:marker_map(v) for k,v in trees.items()}
changed_markers = []
for n in maps["registered"]:
    before, after=maps["registered"][n],maps["calibrated"][n]
    if before != after:
        changed_markers.append({"name":n,"parent_changed":before["parent"]!=after["parent"],
            "local_change_mm":float(np.linalg.norm(np.array(before["local"])-after["local"])*1000)})
right_marker_preservation = {}
for n in names:
    if not n.startswith("L_") or n in {"L_ASIS","L_PSIS"}:
        a,b = maps["healthy"][n],maps["calibrated"][n]
        right_marker_preservation[n]={"parent_preserved":a["parent"]==b["parent"],
            "location_delta_m":float(np.linalg.norm(np.array(a["local"])-b["local"]))}
delta = registered.values-original.values
motion_changes={n:stats(abs(delta[:,j])) for j,n in enumerate(registered.labels) if np.max(abs(delta[:,j]))>1e-12}
vertices=_vertices(ROOT/"Geometry/AM_foot_l.STL")
origin=np.array(RECEIPT["ground"]["origin_m"]);normal=np.array(RECEIPT["ground"]["normal"])
h,p,_=_pose_geometry(model,registered,vertices,origin,normal)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source_hash_matches={name:sha(Path(v["path"]))==v["sha256"] for name,v in RECEIPT["sources"].items()}
groups={"all":names,"calibrated_left":DEFAULT_MARKERS,
        "right": [n for n in names if n.startswith("R_")],
        "pelvis":[n for n in names if "ASIS" in n or "PSIS" in n]}
summary = {g:stats(np.concatenate([errors[n] for n in group])) for g,group in groups.items()}
report={"instruction_check_token":"CMC_AGENT_OK_2026", "scope":"FK/XML/CSV read-only; no IK solver or dynamics",
 "inputs":{"trc_units":trc.units,"motion_in_degrees":registered.in_degrees,"samples":len(registered.times),
           "static_time_range_s":[float(registered.times[0]),float(registered.times[-1])],"markers":len(names)},
 "bundle_identical_to_calibrated":sha(MODEL)==sha(BUNDLE),
 "registration_protected_xml_matches":_protected(trees["anatomical"])==_protected(trees["registered"]),
 "source_hash_matches":source_hash_matches,"only_motion_column_changed":list(motion_changes),
 "static_ankle_pose_offset_deg_range":[float(delta[:,registered.labels.index("pros_ankle_angle")].min()),float(delta[:,registered.labels.index("pros_ankle_angle")].max())],
 "ground_target_recomputed":{"median_min_stl_clearance_m":float(np.median(h)),"median_sagittal_pitch_rad":float(np.median(p)),
          "height_residual_m":float(np.median(h))-RECEIPT["reference"]["median_min_stl_clearance_m"],
          "pitch_residual_rad":float(np.median(p))-RECEIPT["reference"]["median_sagittal_pitch_rad"]},
 "marker_changes":changed_markers,"healthy_biological_and_pelvis_marker_preservation":right_marker_preservation,
 "static_residual_groups":summary,"static_residual_per_marker":per_marker,"recomputed_centroids_vs_receipt":centroid}
output=Path(__file__).with_suffix(".json"); output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
print(json.dumps({"output":str(output),"group_errors":summary,"worst_marker":max(per_marker,key=lambda n:per_marker[n]["max_mm"]),
                  "ground_target":report["ground_target_recomputed"],"changed_markers":len(changed_markers),
                  "centroid_max_difference_m":max(v["recomputed_vs_csv_m"] for v in centroid.values())},indent=2))
