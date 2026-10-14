"""Fit SEA mounting lengths to an EPIC subject without scaling the hardware.

The AB06 donor is the geometric reference. Its osseo/knee hardware and foot
remain fixed. Only the proximal osseo mounting translation and distal pylon
translation are adjusted to preserve the healthy subject/reference ratios of
upper-leg and shank length. The graft's subject hip remains authoritative.

This is a kinematic mounting convention, not a hardware redesign or ScaleTool:
body meshes, COM, mass and inertia retain the builder's benchmark convention.
Marker fitting and new IK must subsequently qualify the assembled model.
There is no OpenSim import, file write or simulation in this helper.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from xml.etree import ElementTree as ET


def _model(element: ET.Element) -> ET.Element:
    if element.tag == "Model":
        return element
    model = element.find("Model")
    if model is None:
        raise ValueError("Expected an OpenSim Model or OpenSimDocument.")
    return model


def _joint(model: ET.Element, name: str) -> ET.Element:
    matches = [j for j in model.findall("JointSet/objects/*")
               if j.get("name") == name]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one joint {name!r}.")
    return matches[0]


def _frame(joint: ET.Element, role: str) -> ET.Element:
    socket = joint.findtext(f"socket_{role}_frame", "").strip()
    matches = [f for f in joint.findall("frames/PhysicalOffsetFrame")
               if f.get("name") == socket]
    if len(matches) != 1:
        raise ValueError(f"Unsupported {role} frame in {joint.get('name')!r}.")
    return matches[0]


def _vec(element: ET.Element, tag: str) -> list[float]:
    values = [float(v) for v in element.findtext(tag, "").split()]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"Expected finite Vec3 {tag} in {element.get('name')}.")
    return values


def _zero_frames(joint: ET.Element, *, zero_translation: bool = False) -> None:
    for role in ("parent", "child"):
        frame = _frame(joint, role)
        if any(abs(v) > 1e-12 for v in _vec(frame, "orientation")):
            raise ValueError("Mount fitting requires aligned EPIC joint frames.")
        if zero_translation and any(abs(v) > 1e-12
                                    for v in _vec(frame, "translation")):
            raise ValueError("Unsupported healthy knee frame translation.")


def _canonical(element: ET.Element) -> tuple:
    """Compare semantic XML independently of indentation/numeric formatting."""
    tokens = (element.text or "").split()
    try:
        text = tuple(float(token) for token in tokens)
    except ValueError:
        text = tuple(tokens)
    return (element.tag, tuple(sorted(element.attrib.items())), text,
            tuple(_canonical(child) for child in element))


def _digest(element: ET.Element) -> str:
    data = json.dumps(_canonical(element), allow_nan=False).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _anatomy(model: ET.Element) -> dict:
    knee = _joint(model, "knee_l")
    if knee.tag != "CustomJoint":
        raise ValueError("Expected the EPIC healthy CustomJoint knee_l.")
    _zero_frames(knee, zero_translation=True)
    scales = []
    profiles = []
    for axis_name in ("translation1", "translation2", "translation3"):
        axis = knee.find(f"SpatialTransform/TransformAxis[@name='{axis_name}']")
        multiplier = None if axis is None else axis.find("MultiplierFunction")
        if multiplier is None:
            raise ValueError("Expected scaled EPIC knee translation functions.")
        scale = float(multiplier.findtext("scale", "nan"))
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("Healthy femur scale must be finite and positive.")
        scales.append(scale)
        profile = deepcopy(axis)
        profile.find("MultiplierFunction/scale").text = "1"
        profiles.append(_canonical(profile))
    if not all(math.isclose(v, scales[0], rel_tol=1e-12) for v in scales):
        raise ValueError("Only uniform EPIC femur scaling is supported.")
    ankle = _joint(model, "ankle_l")
    _zero_frames(ankle)
    parent = _vec(_frame(ankle, "parent"), "translation")
    child = _vec(_frame(ankle, "child"), "translation")
    if any(abs(v) > 1e-12 for v in parent[::2] + child):
        raise ValueError("Expected the EPIC longitudinal ankle offset.")
    shank = -parent[1]
    if shank <= 0:
        raise ValueError("Healthy shank length must be positive.")
    return {"femur_scale": scales[0], "shank_length_m": shank,
            "knee_translation_profiles": profiles}


def _mounts(model: ET.Element) -> tuple[ET.Element, ET.Element]:
    osseo = _joint(model, "osseo")
    knee = _joint(model, "pros_knee")
    ankle = _joint(model, "pros_ankle")
    if (osseo.tag, knee.tag, ankle.tag) != ("WeldJoint", "PinJoint", "PinJoint"):
        raise ValueError("Expected the fixed SEA donor joint topology.")
    for joint in (osseo, knee, ankle):
        _zero_frames(joint)
        if any(abs(v) > 1e-12 for v in
               _vec(_frame(joint, "child"), "translation")):
            raise ValueError("Expected zero donor child-frame offsets.")
    return _frame(osseo, "parent"), _frame(ankle, "parent")


def _protected_digest(model: ET.Element) -> str:
    protected = deepcopy(model)
    for frame in _mounts(protected):
        frame.find("translation").text = "0 0 0"
    return _digest(protected)


def fit_seasea_subject(
    grafted_model: ET.Element,
    subject_healthy_model: ET.Element,
    reference_healthy_model: ET.Element,
    reference_donor_model: ET.Element,
) -> dict:
    """Adjust two mounting frames in-memory and return a JSON-ready manifest.

    The caller must first graft the subject hip/bodies/forces and preserve the
    benchmark mass convention. All other XML is checked immutable. Dimensions
    are measured relative to the supplied healthy reference; reference-subject
    fitting is an exact identity, including the donor's lateral mount offsets.
    """
    graft, subject, reference, donor = map(
        _model, (grafted_model, subject_healthy_model,
                 reference_healthy_model, reference_donor_model))
    if any(graft is source for source in (subject, reference, donor)):
        raise ValueError("The graft must be independent from its source models.")
    if any(m.findtext("length_units", "").strip() != "meters"
           for m in (graft, subject, reference, donor)):
        raise ValueError("Fitting requires explicit metre length units.")
    source_digests = [_digest(source) for source in (subject, reference, donor)]
    subject_anatomy = _anatomy(subject)
    reference_anatomy = _anatomy(reference)
    if subject_anatomy["knee_translation_profiles"] != reference_anatomy["knee_translation_profiles"]:
        raise ValueError("Healthy knee shapes differ beyond uniform scaling.")
    upper_ratio = subject_anatomy["femur_scale"] / reference_anatomy["femur_scale"]
    shank_ratio = subject_anatomy["shank_length_m"] / reference_anatomy["shank_length_m"]
    graft_frames, donor_frames = _mounts(graft), _mounts(donor)
    donor_mount, donor_ankle = [_vec(frame, "translation") for frame in donor_frames]
    for name in ("osseo", "pros_knee", "pros_ankle"):
        actual, expected = deepcopy(_joint(graft, name)), deepcopy(_joint(donor, name))
        if name in ("osseo", "pros_ankle"):
            if _vec(_frame(actual, "parent"), "translation")[::2] != _vec(_frame(expected, "parent"), "translation")[::2]:
                raise ValueError("Lateral hardware mounting offsets must match the donor.")
            for joint in (actual, expected):
                _frame(joint, "parent").find("translation").text = "0 0 0"
        if _canonical(actual) != _canonical(expected):
            raise ValueError("Fixed prosthetic joint hardware must match the donor.")
    fixed_knee = _vec(_frame(_joint(donor, "pros_knee"), "parent"), "translation")
    if any(abs(v) > 1e-12 for v in fixed_knee[::2] + donor_ankle[::2]):
        raise ValueError("Expected longitudinal distal hardware and pylon.")
    if donor_mount[1] >= 0 or fixed_knee[1] >= 0 or donor_ankle[1] >= 0:
        raise ValueError("Donor mounting lengths must point distally.")
    for role in ("parent", "child"):
        if _vec(_frame(_joint(graft, "hip_l"), role), "translation") != _vec(_frame(_joint(subject, "hip_l"), role), "translation"):
            raise ValueError("The graft must retain the subject-specific hip.")
    graft_seas = [_canonical(e) for e in graft.findall("ForceSet/objects/SeriesElasticActuator")]
    donor_seas = [_canonical(e) for e in donor.findall("ForceSet/objects/SeriesElasticActuator")]
    if len(graft_seas) != 2 or graft_seas != donor_seas:
        raise ValueError("SEA properties must exactly match the reference donor.")
    # Shorten only the adjustable proximal mount; preserve osseo/knee hardware.
    upper_vector = [donor_mount[i] + fixed_knee[i] for i in range(3)]
    upper_length = math.sqrt(sum(v * v for v in upper_vector))
    fitted_mount = list(donor_mount)
    if upper_ratio != 1.0:
        y_squared = (upper_length * upper_ratio) ** 2 - donor_mount[0] ** 2 - donor_mount[2] ** 2
        if y_squared <= 0:
            raise ValueError("Subject upper length cannot accommodate fixed hardware offsets.")
        fitted_mount[1] = -math.sqrt(y_squared) - fixed_knee[1]
        if fitted_mount[1] >= 0:
            raise ValueError("Subject upper length cannot accommodate fixed knee hardware.")
    fitted_ankle = list(donor_ankle)
    if shank_ratio != 1.0:
        fitted_ankle[1] *= shank_ratio
    targets = (fitted_mount, fitted_ankle)
    if not all(math.isfinite(v) for vector in targets for v in vector):
        raise ValueError("Non-finite fitted mounting geometry.")
    protected_before = _protected_digest(graft)
    changes = []
    for joint_name, frame, target in zip(("osseo", "pros_ankle"), graft_frames, targets):
        before = _vec(frame, "translation")
        if before != target:
            frame.find("translation").text = " ".join(format(v, ".17g") for v in target)
            changes.append({"joint": joint_name, "frame": frame.get("name"),
                            "property": "translation", "before_m": before,
                            "after_m": target})
    if _protected_digest(graft) != protected_before:
        raise RuntimeError("Fitting unexpectedly changed protected model XML.")
    if [_digest(source) for source in (subject, reference, donor)] != source_digests:
        raise RuntimeError("Fitting unexpectedly changed a source model.")
    return {
        "schema_version": 1,
        "method": "reference_relative_mounting_fixed_hardware_v1",
        "subject_model": subject.get("name"),
        "reference_healthy_model": reference.get("name"),
        "reference_donor_model": donor.get("name"),
        "source_model_xml_semantic_sha256": dict(zip(
            ("subject_healthy", "reference_healthy", "reference_donor"),
            source_digests)),
        "healthy_reference": {k: v for k, v in reference_anatomy.items() if k != "knee_translation_profiles"},
        "healthy_subject": {k: v for k, v in subject_anatomy.items() if k != "knee_translation_profiles"},
        "length_ratios": {"upper": upper_ratio, "shank": shank_ratio},
        "reference_lengths_m": {"upper_center_distance": upper_length,
                                "shank_center_distance": -donor_ankle[1]},
        "fitted_lengths_m": {"upper_center_distance": math.sqrt(sum((fitted_mount[i] + fixed_knee[i]) ** 2 for i in range(3))),
                             "shank_center_distance": -fitted_ankle[1]},
        "changed_geometry": changes,
        "preserved": ["subject hip", "fixed osseo/knee hardware", "foot hardware",
                      "SEA properties and controls", "joint axes/orientations/ranges",
                      "all body mass/COM/inertia", "all visual meshes", "all markers"],
        "checks": {"protected_xml_unchanged": True, "sources_unchanged": True,
                   "sea_matches_donor": True, "finite_mounting": True},
        "limitations": ["Visual meshes are not scaled; only physical mounting frames change.",
                        "COM/inertia retain the graft benchmark convention, not a hardware identification.",
                        "Static marker fitting, new IK and ground/contact qualification are required."],
    }
