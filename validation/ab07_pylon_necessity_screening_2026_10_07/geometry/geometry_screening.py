"""Predeclared AB07 pylon geometry screening, isolated and source-bound.

Rigid XML/NumPy algebra and sparse Position FK only. No model writes, IK,
dynamic simulation, force computation or donor-static target in reachability.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import brentq
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROTOCOL = HERE / "protocol.json"
EXPECTED_PROTOCOL_SHA = "c50b1867cd24c2135705d392d61d79a3627281421092824fa38455e90f2a9753"
PURE_HELPER = ROOT / "validation/ab07_mount_registration_verification_2026_10_07/constraints/relative_right_target_supplement.py"
OUTPUT = HERE / "geometry_screening.json"
sys.path.insert(0, str(ROOT))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def label(path):
    return Path(path).resolve().relative_to(ROOT).as_posix()


def stats(a):
    a = np.asarray(a, dtype=float)
    if not a.size or not np.all(np.isfinite(a)):
        raise ValueError("Finite nonempty result required")
    return {"minimum": float(a.min()), "maximum": float(a.max()), "mean": float(a.mean()), "median": float(np.median(a)), "rms": float(np.sqrt(np.mean(a*a)))}


def canonical(e):
    return e.tag, tuple(sorted(e.attrib.items())), tuple((e.text or "").split()), tuple(canonical(c) for c in e)


def physical_chain_contract(tree, algebra):
    if tree.findtext("Model/length_units") != "meters":
        raise ValueError("Expected model meters")
    joints = {e.get("name"): e for e in tree.findall(".//JointSet/objects/*")}
    gp, hip, osseo, knee, ankle = [joints[n] for n in ("ground_pelvis", "hip_l", "osseo", "pros_knee", "pros_ankle")]
    algebra.active_axes(gp, ("pelvis_tilt", "pelvis_list", "pelvis_rotation"), ([0,0,1], [1,0,0], [0,1,0]), True)
    algebra.active_axes(hip, ("hip_flexion_l", "hip_adduction_l", "hip_rotation_l"), ([0,0,1], [-1,0,0], [0,-1,0]), False)
    if osseo.tag != "WeldJoint" or knee.tag != "PinJoint" or ankle.tag != "PinJoint":
        raise ValueError("Frozen weld/positiveZ Pin topology required")
    for joint in (gp, hip, osseo, knee, ankle):
        if np.any(algebra.frame(joint, "child") != 0.):
            raise ValueError("Expected exact zero child offsets")
    if np.any(algebra.frame(gp, "parent") != 0.):
        raise ValueError("Expected exact zero ground offset")
    if list(tree.findall(".//ConstraintSet/objects/*")):
        raise ValueError("No kinematic constraints/assembly allowed")
    mesh = tree.find(".//BodySet/objects/Body[@name='foot_l']/attached_geometry/Mesh")
    if mesh is None or mesh.findtext("socket_frame", "..").strip() != ".." or Path(mesh.findtext("mesh_file")).name.casefold() != "am_foot_l.stl" or list(map(float, mesh.findtext("scale_factors", "1 1 1").split())) != [1.,1.,1.]:
        raise ValueError("Same fixed directly-attached unscaled foot STL required")
    bounds = {}
    for name in ("pros_knee_angle", "pros_ankle_angle"):
        c = tree.find(f".//JointSet/objects/*/coordinates/Coordinate[@name='{name}']")
        bounds[name] = np.fromstring(c.findtext("range"), sep=" ")
        if c.findtext("locked", "false") != "false" or c.findtext("clamped", "false") != "false" or not np.array_equal(bounds[name], [-math.pi/2, math.pi/2]):
            raise ValueError("Frozen unchanged ±90-degree unlocked/unclamped ranges required")
    return (gp, hip, osseo, knee, ankle), bounds


def zero_mount_chain(tree):
    copy = ET.fromstring(ET.tostring(tree.getroot()))
    ankle = copy.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
    name = ankle.findtext("socket_parent_frame")
    t = ankle.find(f"frames/PhysicalOffsetFrame[@name='{name}']/translation")
    values = t.text.split(); values[1] = "0"; t.text = " ".join(values)
    return canonical(copy.find(".//JointSet")), canonical(copy.find(".//BodySet")), canonical(copy.find(".//ForceSet"))


def base_poses(values, labels, chain, algebra):
    _, hip, osseo, knee, _ = chain
    poses = []
    for i, row in enumerate(values):
        q = dict(zip(labels[1:], row[1:]))
        q = {name: value if name in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else math.radians(value) for name, value in q.items()}
        pelvis = np.array([q[n] for n in ("pelvis_tx", "pelvis_ty", "pelvis_tz")])
        rpelvis = algebra.rz(q["pelvis_tilt"])@algebra.rx(q["pelvis_list"])@algebra.ry(q["pelvis_rotation"])
        hip_p = pelvis+rpelvis@algebra.frame(hip, "parent")
        rupper = rpelvis@algebra.rz(q["hip_flexion_l"])@algebra.rx(-q["hip_adduction_l"])@algebra.ry(-q["hip_rotation_l"])
        knee_p = hip_p+rupper@(algebra.frame(osseo, "parent")+algebra.frame(knee, "parent"))
        poses.append({"index": i, "time_s": float(row[0]), "q": q, "pelvis": pelvis, "hip": hip_p, "knee": knee_p, "rupper": rupper})
    return poses


def height_grid(pose, length, knee_angle, angles, points, radii, plane_y, algebra):
    rtibia = pose["rupper"]@algebra.rz(knee_angle)
    foot_origin_y = pose["knee"][1]-length*rtibia[1,1]
    row = rtibia[1]
    c, s = np.cos(angles), np.sin(angles)
    projected = np.column_stack((row[0]*c+row[1]*s, -row[0]*s+row[1]*c, np.full(len(angles), row[2])))
    return foot_origin_y+np.min(projected@points.T-radii[None,:], axis=1)-plane_y


def boundary_roots(pose, length, knee_angle, angles, grid, points, radii, plane_y, algebra, max_distance=math.inf):
    raw_knee = pose["q"]["pros_knee_angle"]
    raw_ankle = pose["q"]["pros_ankle_angle"]
    residual = lambda a: float(height_grid(pose, length, knee_angle, np.array([a]), points, radii, plane_y, algebra)[0])
    roots = [float(angles[i]) for i in np.flatnonzero(np.abs(grid) <= 1e-12)]
    for i in np.flatnonzero(grid[:-1]*grid[1:] < 0):
        lower_dist = (knee_angle-raw_knee)**2+max(angles[i]-raw_ankle, raw_ankle-angles[i+1], 0.)**2
        if lower_dist <= max_distance+1e-15:
            roots.append(float(brentq(residual, float(angles[i]), float(angles[i+1]), xtol=1e-12)))
    if not roots:
        return None
    ankle = min(roots, key=lambda a: abs(a-raw_ankle))
    distance = (knee_angle-raw_knee)**2+(ankle-raw_ankle)**2
    return {"knee_angle_rad": float(knee_angle), "ankle_angle_rad": ankle,
            "knee_deviation_deg": math.degrees(knee_angle-raw_knee), "ankle_deviation_deg": math.degrees(ankle-raw_ankle),
            "angular_distance_deg": math.degrees(math.sqrt(distance)), "squared_distance_rad2": distance,
            "boundary_residual_m": residual(ankle), "knee_bound_margin_deg": math.degrees(min(knee_angle+math.pi/2, math.pi/2-knee_angle)),
            "ankle_bound_margin_deg": math.degrees(min(ankle+math.pi/2, math.pi/2-ankle))}


def classification(witness, min_bound, max_bound):
    if witness is not None:
        return "zero_boundary_reachable_by_explicit_witness"
    if min_bound > 0.:
        return "always_above_plane_continuous_bound"
    if max_bound < 0.:
        return "always_intersecting_plane_continuous_bound_zero_boundary_absent"
    return "no_grid_root_continuous_domain_unresolved"


def aggregate(rows, mode):
    data = [row[mode] for row in rows]
    found = [d["witness"] for d in data if d["witness"] is not None]
    counts = {}
    for d in data:
        counts[d["classification"]] = counts.get(d["classification"], 0)+1
    result = {"classification_counts": counts, "witness_count": len(found), "rows": len(rows), "continuous_min_bound_m": stats([d["continuous_min_bound_m"] for d in data]), "continuous_max_bound_m": stats([d["continuous_max_bound_m"] for d in data])}
    if found:
        for field in ("knee_deviation_deg", "ankle_deviation_deg", "angular_distance_deg", "boundary_residual_m", "knee_bound_margin_deg", "ankle_bound_margin_deg"):
            result[field] = stats([w[field] for w in found])
    return result


def native_parity(protocol, result, endpoints, poses, full_vertices, surfaces, algebra, extra_paths):
    import opensim
    from model_loader import _configure_geometry_search_paths
    from tools.register_seasea_ground import load_plugin
    suffix = ".dylib" if sys.platform == "darwin" else ".dll" if os.name == "nt" else ".so"
    basename = "SEA_Plugin_BlackBox_mCMC_impedence_ff" if os.name == "nt" else "libSEA_Plugin_BlackBox_mCMC_impedence_ff"
    plugin = ROOT/"plugins"/(basename+suffix)
    extra_paths[plugin] = sha(plugin)
    load_plugin(plugin)
    parity = {"coordinate_angle_max_abs_error_rad": 0., "coordinate_translation_max_abs_error_m": 0., "center_position_max_abs_error_m": 0., "surface_clearance_max_abs_error_m": 0., "rotation_max_abs_error_dimensionless": 0., "poses": 0, "no_assemble_or_dynamics_called": True, "OpenSim_version": opensim.GetVersionAndDate()}
    for endpoint, model_path in endpoints.items():
        _configure_geometry_search_paths(str(model_path))
        model = opensim.Model(str(model_path)); state = model.initSystem(); coords = model.getCoordinateSet()
        length = protocol["endpoints"][endpoint]["length_m"]
        tests = [(i, poses[i]["q"]["pros_knee_angle"], poses[i]["q"]["pros_ankle_angle"], None) for i in (0, len(poses)//2, len(poses)-1)]
        for sid, data in result[endpoint].items():
            for mode in ("raw_knee_ankle_free", "knee_ankle_free"):
                row = data["per_pose"][len(poses)//2]; w = row[mode]["witness"]
                if w is not None:
                    tests.append((row["index"], w["knee_angle_rad"], w["ankle_angle_rad"], sid))
        for i, knee, ankle, sid in tests:
            pose = poses[i]; q = dict(pose["q"]); q["pros_knee_angle"] = knee; q["pros_ankle_angle"] = ankle
            if {coords.get(k).getName() for k in range(coords.getSize())} != set(q):
                raise ValueError("Native state requires every coordinate once")
            state.setTime(pose["time_s"])
            for name, value in q.items():
                coords.get(name).setValue(state, value, False)
                key = "coordinate_translation_max_abs_error_m" if name in ("pelvis_tx","pelvis_ty","pelvis_tz") else "coordinate_angle_max_abs_error_rad"
                parity[key] = max(parity[key], abs(coords.get(name).getValue(state)-value))
            model.realizePosition(state)
            transform = model.getBodySet().get("foot_l").getTransformInGround(state)
            foot_p = np.array([transform.p().get(k) for k in range(3)])
            foot_r = np.array([[transform.R().get(k,j) for j in range(3)] for k in range(3)])
            tibia_r = pose["rupper"]@algebra.rz(knee)
            predicted_p = pose["knee"]+tibia_r@np.array([0.,-length,0.])
            parity["center_position_max_abs_error_m"] = max(parity["center_position_max_abs_error_m"], float(np.max(np.abs(foot_p-predicted_p))))
            for joint_name, expected_p in (("hip_l",pose["hip"]),("pros_knee",pose["knee"])):
                frame = model.getJointSet().get(joint_name).getChildFrame().getTransformInGround(state)
                position = np.array([frame.p().get(k) for k in range(3)])
                parity["center_position_max_abs_error_m"] = max(parity["center_position_max_abs_error_m"], float(np.max(np.abs(position-expected_p))))
            parity["rotation_max_abs_error_dimensionless"] = max(parity["rotation_max_abs_error_dimensionless"], float(np.max(np.abs(foot_r-tibia_r@algebra.rz(ankle)))))
            for s in surfaces:
                points = full_vertices if s["id"].startswith("STL_") else s["points"]
                radii = np.zeros(len(full_vertices)) if s["id"].startswith("STL_") else s["radii"]
                native = float(foot_p[1]+np.min(points@foot_r[1]-radii)-s["plane_Y_m"])
                predicted = float(height_grid(pose,length,knee,np.array([ankle]),s["points"],s["radii"],s["plane_Y_m"],algebra)[0])
                parity["surface_clearance_max_abs_error_m"] = max(parity["surface_clearance_max_abs_error_m"],abs(native-predicted))
            parity["poses"] += 1
    if any(parity[k] > 1e-12 for k in ("coordinate_angle_max_abs_error_rad","coordinate_translation_max_abs_error_m","center_position_max_abs_error_m","surface_clearance_max_abs_error_m","rotation_max_abs_error_dimensionless")):
        raise AssertionError("Native sparse Position FK disagrees with rigid algebra")
    return parity


def main():
    if OUTPUT.exists():
        raise FileExistsError("Existing result is immutable")
    if sha(PROTOCOL) != EXPECTED_PROTOCOL_SHA:
        raise ValueError("Predeclared protocol binding differs")
    protocol = json.loads(PROTOCOL.read_text())
    paths = {ROOT/name:digest for name,digest in protocol["protected_sources_before"].items()}
    for path,digest in paths.items():
        if sha(path) != digest:
            raise ValueError(f"Protected source differs: {path}")
    paths[PROTOCOL] = EXPECTED_PROTOCOL_SHA; paths[Path(__file__).resolve()] = sha(__file__)
    spec = importlib.util.spec_from_file_location("frozen_xml_rigid_algebra", PURE_HELPER)
    algebra = importlib.util.module_from_spec(spec); spec.loader.exec_module(algebra)
    endpoints = {n:ROOT/v["model"] for n,v in protocol["endpoints"].items()}
    trees = {n:ET.parse(p) for n,p in endpoints.items()}
    chains = {n:physical_chain_contract(t,algebra)[0] for n,t in trees.items()}
    if zero_mount_chain(trees["L0"]) != zero_mount_chain(trees["L1"]):
        raise AssertionError("Endpoint bodies/joints/forces differ beyond distal mountingY")
    for endpoint,chain in chains.items():
        if abs(algebra.frame(chain[-1],"parent")[1]+protocol["endpoints"][endpoint]["length_m"]) > 1e-15:
            raise AssertionError("Endpoint length differs from protocol")
    labels,values = algebra.motion(ROOT/protocol["biological_pose"]["source"])
    if len(values) != 436:
        raise AssertionError("Expected complete static 436-row acquisition")
    poses = base_poses(values,labels,chains["L0"],algebra)
    for pose in poses:
        for name in ("pros_knee_angle","pros_ankle_angle"):
            if not -math.pi/2 <= pose["q"][name] <= math.pi/2:
                raise ValueError("Raw prosthetic pose out of frozen bounds")
    full_vertices = algebra.vertices(ROOT/"Geometry/AM_foot_l.STL")
    hull = ConvexHull(full_vertices); vertices = full_vertices[hull.vertices]
    force = json.loads((ROOT/"online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json").read_text())
    force_points = [s for s in force["spheres"] if s["side"] == "left"]
    detector = json.loads((ROOT/"validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json").read_text())
    surfaces = []
    for declared in protocol["surfaces_and_planes"]:
        if declared["id"].startswith("STL_"):
            points,radii = vertices,np.zeros(len(vertices))
        elif declared["id"] == "force_sphere_primary":
            if len(force_points) != 8 or any(s["frame"] != "/bodyset/foot_l" for s in force_points) or force["ground"]["normal"] != [0.,1.,0.] or force["ground"]["origin"][1] != declared["plane_Y_m"]:
                raise ValueError("Frozen primary spheres/frame/plane differs")
            points = np.array([s["location"] for s in force_points]);radii = np.array([s["radius"] for s in force_points])
        else:
            if len(detector["points"]) != 2 or any(s["frame"] != "/bodyset/foot_l" for s in detector["points"]) or detector["ground"]["normal"] != [0.,1.,0.] or detector["ground"]["origin"][1] != declared["plane_Y_m"]:
                raise ValueError("Frozen binary points/frame/plane differs")
            points=np.array([s["location"] for s in detector["points"]]);radii=np.zeros(len(points))
        surfaces.append(dict(declared,points=points,radii=radii))
    angles=np.linspace(-math.pi/2,math.pi/2,181)
    knee_grid=np.linspace(-math.pi/2,math.pi/2,37)
    results={}; extra_paths={}
    hull_parity=0.
    for endpoint in endpoints:
        length=protocol["endpoints"][endpoint]["length_m"]; results[endpoint]={}
        for surface in surfaces:
            print(endpoint,surface["id"],"436 static poses/range screening",flush=True)
            points,radii,plane=surface["points"],surface["radii"],surface["plane_Y_m"]
            radial=float(np.max(np.linalg.norm(points[:,:2],axis=1)))
            ankle_bound=radial*math.radians(.5)
            both_bound=ankle_bound+(length+radial)*math.radians(2.5)
            rows=[]
            for pose in poses:
                raw_knee=pose["q"]["pros_knee_angle"];raw_ankle=pose["q"]["pros_ankle_angle"]
                raw=float(height_grid(pose,length,raw_knee,np.array([raw_ankle]),points,radii,plane,algebra)[0])
                grid=height_grid(pose,length,raw_knee,angles,points,radii,plane,algebra)
                fixed=boundary_roots(pose,length,raw_knee,angles,grid,points,radii,plane,algebra)
                min_grid,max_grid=float(grid.min()),float(grid.max())
                best=fixed
                for knee_angle in np.unique(np.r_[knee_grid,raw_knee]):
                    if knee_angle == raw_knee:
                        continue
                    kgrid=height_grid(pose,length,float(knee_angle),angles,points,radii,plane,algebra)
                    min_grid,max_grid=min(min_grid,float(kgrid.min())),max(max_grid,float(kgrid.max()))
                    if best is None or (knee_angle-raw_knee)**2 <= best["squared_distance_rad2"]+1e-15:
                        candidate=boundary_roots(pose,length,float(knee_angle),angles,kgrid,points,radii,plane,algebra,max_distance=best["squared_distance_rad2"] if best is not None else math.inf)
                        if candidate is not None and (best is None or candidate["squared_distance_rad2"] < best["squared_distance_rad2"]):
                            best=candidate
                fixed_min,fixed_max=float(grid.min())-ankle_bound,float(grid.max())+ankle_bound
                both_min,both_max=min_grid-both_bound,max_grid+both_bound
                rows.append({"index":pose["index"],"time_s":pose["time_s"],"raw_clearance_m":raw,
                    "raw_knee_ankle_free":{"witness":fixed,"grid_min_clearance_m":float(grid.min()),"grid_max_clearance_m":float(grid.max()),"continuous_min_bound_m":fixed_min,"continuous_max_bound_m":fixed_max,"classification":classification(fixed,fixed_min,fixed_max)},
                    "knee_ankle_free":{"witness":best,"grid_min_clearance_m":min_grid,"grid_max_clearance_m":max_grid,"continuous_min_bound_m":both_min,"continuous_max_bound_m":both_max,"classification":classification(best,both_min,both_max)}})
            results[endpoint][surface["id"]]={"surface":surface["surface"],"plane_Y_m":plane,"role":surface["role"],"raw_clearance_m":stats([r["raw_clearance_m"] for r in rows]),
                "raw_sign_counts":{"above":sum(r["raw_clearance_m"]>1e-12 for r in rows),"boundary":sum(abs(r["raw_clearance_m"])<=1e-12 for r in rows),"intersecting":sum(r["raw_clearance_m"]<-1e-12 for r in rows)},
                "raw_knee_ankle_free":aggregate(rows,"raw_knee_ankle_free"),"knee_ankle_free":aggregate(rows,"knee_ankle_free"),
                "Lipschitz_grid_bound_m":{"ankle_only":ankle_bound,"knee_and_ankle":both_bound,"derivation":"For local points, |dY/da|≤maxnormXY. |dY/dk|≤length+maxnormXY; min-envelope is equally Lipschitz; nearest-grid distances0.5deg/2.5deg. Radii are rotation invariant."},"per_pose":rows}
        for pose in (poses[0],poses[len(poses)//2],poses[-1]):
            for k,a in ((pose["q"]["pros_knee_angle"],pose["q"]["pros_ankle_angle"]),(-math.pi/2,-math.pi/2),(0.,0.),(math.pi/2,math.pi/2)):
                full=height_grid(pose,length,k,np.array([a]),full_vertices,np.zeros(len(full_vertices)),0.,algebra)
                reduced=height_grid(pose,length,k,np.array([a]),vertices,np.zeros(len(vertices)),0.,algebra)
                hull_parity=max(hull_parity,float(np.max(np.abs(full-reduced))))
    if hull_parity>1e-12:
        raise AssertionError("Convex hull minima differ")
    parity=native_parity(protocol,results,endpoints,poses,full_vertices,surfaces,algebra,extra_paths)
    for path,digest in paths.items():
        if sha(path)!=digest:raise AssertionError(f"Protected input changed:{path}")
    for path,digest in extra_paths.items():
        if sha(path)!=digest:raise AssertionError(f"Native library changed:{path}")
    existing=json.loads((ROOT/"validation/ab07_mount_registration_verification_2026_10_07/constraints/static_geometry_constraints.json").read_text())
    result={"schema_version":1,"created_utc":datetime.now(timezone.utc).isoformat(),"status":"PASS_numerical_and_protocol_guards","protocol_sha256":EXPECTED_PROTOCOL_SHA,"helper_sha256":sha(__file__),
        "endpoints":protocol["endpoints"],"results":results,"static_rows":len(poses),"native_sparse_parity":parity,
        "fixed_hardware_local_geometry_m":{"STL_total_vertices":len(full_vertices),"convex_hull_vertices":len(vertices),"local_bounds":{"minimum":full_vertices.min(axis=0).tolist(),"maximum":full_vertices.max(axis=0).tolist()},"neutral_pin_to_lowest_visual_STL_vertical_distance":float(-full_vertices[:,1].min()),"min_pin_to_STL_vertex_Euclidean_distance":float(np.linalg.norm(full_vertices,axis=1).min()),"max_pin_to_STL_vertex_Euclidean_distance":float(np.linalg.norm(full_vertices,axis=1).max()),"hull_linear_projection_parity_max_abs_error_m":hull_parity,"dimension_limit":"Model mesh dimensions relative to the ankle hinge, not independently measured subject hardware/shoe dimensions or closest triangle-surface distance."},
        "own_healthy_and_right_geometry_reused_hash_bound":{"source":"validation/ab07_mount_registration_verification_2026_10_07/constraints/static_geometry_constraints.json","AB07_healthy_joint_centers":existing["subjects"]["AB07"]["healthy_original_static"]["joint_center_ground_XYZ_m"],"AB07_healthy_center_lengths":existing["subjects"]["AB07"]["healthy_original_static"]["center_lengths"],"own_right_visual_minimum":existing["subjects"]["AB07"]["healthy_original_static"]["visual_foot_surface_minima"]["right"],"role":"Subject-local pose/geometry evidence; right surface is generic scaled mesh, not a physical support-plane estimator."},
        "interpretation_guards":["A root witness proves zero-clearance geometric boundary reachability in the declared model domain, not clinical feasibility, load support or exact original tracking.","Raw-pose signed clearances explicitly preserve tracking and may differ from nearest free-joint witnesses.","A strictly negative continuous bound means always intersecting geometry with no zero-boundary root, not impossible contact. A strictly positive bound means always above plane.","A force sphere generally needs penetration to support force; no force or event is inferred here. STL, spheres and binary points remain separate.","No donor height/pitch target, GRF fitting, claimed physical-floor survey or required preservation of healthy subject height ordering appears in any solve.","Data do not independently identify a subject-specific physical pylon length: original markers fit anatomical pose, virtual markers were freely recalibrated, static GRF and measured hardware fiducials/shoe/floor/clinical dimensions are absent in inspected artifacts."],
        "inputs_integrity":{"all_unchanged":True,"count":len(paths),"before":{label(p):v for p,v in paths.items()},"after":{label(p):sha(p) for p in paths}},
        "native_library_integrity":{"acquisition":"Hash acquired immediately before native Position FK, after the algebra range screening; no claim it was included in protocol preflight.","before":{label(p):v for p,v in extra_paths.items()},"after":{label(p):sha(p) for p in extra_paths}}}
    with OUTPUT.open("x",encoding="utf-8") as f:json.dump(result,f,indent=2,allow_nan=False);f.write("\n")
    print("PASS",label(OUTPUT),sha(OUTPUT))
    for endpoint,data in results.items():
        print(endpoint,{sid:{"raw_median_m":d["raw_clearance_m"]["median"],"ankle_only_count":d["raw_knee_ankle_free"]["witness_count"],"both_count":d["knee_ankle_free"]["witness_count"],"ankle_only_max_deviation_deg":d["raw_knee_ankle_free"].get("ankle_deviation_deg",{}).get("maximum")}for sid,d in data.items()})


if __name__=="__main__":
    main()
