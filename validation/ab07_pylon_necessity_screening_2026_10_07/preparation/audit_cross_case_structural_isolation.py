"""Read saved XML only to verify structural isolation of seventeen cells.

Remove the complete MarkerSet and replace ONLY pros_ankle's parent-offset
translation Y by a common comparison sentinel. Keep every other XML field,
including render geometry, body COM/inertia, coordinate defaults/ranges and
ForceSet/plugin properties. No OpenSim/FK/IK/dynamics or source modification.
"""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
REL_TOL = ABS_TOL = 1e-14


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))


def fp(path):
    path = Path(path).resolve()
    return {'path': path.relative_to(ROOT).as_posix(), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def value(text):
    words = (text or '').split()
    try:
        numbers = [float(w) for w in words]
    except ValueError:
        return words
    if not all(math.isfinite(n) for n in numbers):
        raise ValueError('Nonfinite numeric XML field')
    return numbers


def flat(element, path=''):
    # Whitespace and numeric spelling are normalized; every attribute/property
    # remains. Named component membership and repeated-node occurrence retained.
    result = {}
    if value(element.text):
        result[path + '/#text'] = value(element.text)
    for key, val in element.attrib.items():
        result[path + '/@' + key] = val
    count = Counter()
    for child in element:
        label = child.tag + ('[@name=' + child.attrib['name'] + ']' if 'name' in child.attrib else '')
        count[label] += 1
        if count[label] > 1:
            label += '[occurrence=' + str(count[label]) + ']'
        result.update(flat(child, path + '/' + label))
    return result


def equivalent(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(equivalent(x, y) for x, y in zip(a, b))
    return a == b


def differences(a, b, tolerant=False):
    return [{'field': k, 'reference': a.get(k), 'case': b.get(k)} for k in sorted(a.keys() | b.keys())
            if not (equivalent(a.get(k), b.get(k)) if tolerant else a.get(k) == b.get(k))]


def records(path, case):
    tree = ET.parse(path)
    model = tree.find('Model')
    marker_sets = model.findall('MarkerSet')
    joints = model.findall("JointSet/objects/PinJoint[@name='pros_ankle']")
    if len(marker_sets) != 1 or len(joints) != 1:
        raise ValueError('Exactly one MarkerSet and pros_ankle PinJoint required')
    markers = list(marker_sets[0].find('objects'))
    if len(markers) != 28 or len({e.get('name') for e in markers}) != 28:
        raise ValueError('Exactly28unique markers required')
    joint = joints[0]
    parent_name = joint.findtext('socket_parent_frame')
    frames = joint.findall("frames/PhysicalOffsetFrame[@name='%s']" % parent_name)
    if len(frames) != 1:
        raise ValueError('Resolve parent frame exactly within pros_ankle')
    parent = frames[0]
    translations = parent.findall('translation')
    if len(translations) != 1:
        raise ValueError('Exactly one parent translation required')
    translation = translations[0]
    mount = value(translation.text)
    if len(mount) != 3 or mount[0] != 0 or mount[2] != 0 or mount[1] != case['mount_parent_Y_m']:
        raise ValueError('Case mount geometry differs from prospective exact metre value')
    physical_bodies = list(model.find('BodySet/objects'))
    body_numeric = {b.get('name'): {k: value(b.findtext(k)) for k in ('mass', 'mass_center', 'inertia')}
                    for b in physical_bodies}
    forces = list(model.find('ForceSet/objects'))
    joints_all = list(model.find('JointSet/objects'))
    metadata = {'parent_frame': parent_name, 'parent_socket': parent.findtext('socket_parent'),
                'child_socket': joint.findtext('socket_child_frame'), 'joint_type': joint.tag,
                'parent_translation_m': mount, 'parent_orientation_rad': value(parent.findtext('orientation')),
                'body_count': len(physical_bodies), 'body_mass_COM_inertia': body_numeric,
                'total_mass_kg': sum(row['mass'][0] for row in body_numeric.values()),
                'joint_membership': {j.get('name'): j.tag for j in joints_all},
                'force_type_counts': dict(Counter(f.tag for f in forces)),
                'force_count': len(forces), 'marker_count': len(markers)}
    # Record markers separately; the primary test removes their whole set.
    marker_records = {m.get('name'): flat(m) for m in markers}
    model.remove(marker_sets[0])
    mount[1] = 0.
    translation.text = ' '.join(format(x, '.17g') for x in mount)
    return flat(tree.getroot()), metadata, marker_records


def main():
    output = HERE / 'cross_case_structural_isolation.json'
    if output.exists():
        raise FileExistsError('Preserve earlier structural audit')
    main_path, supplement_path = HERE / 'preparation_protocol.json', HERE / 'supplement_pose/supplement_protocol.json'
    main_protocol, supplement = read(main_path), read(supplement_path)
    cases = dict(main_protocol['cases'])
    if set(cases).intersection(supplement['cases']):
        raise ValueError('Main and supplement cells must be distinct')
    cases.update(supplement['cases'])
    if len(cases) != 17:
        raise ValueError('Require all14main and3supplement cells')
    protected = dict(main_protocol['protected_sources_before'])
    protected.update(supplement['protected_sources_before'])
    paths = {ROOT / p for p in protected}
    paths.update([Path(__file__), main_path, supplement_path])
    receipts = {}
    for name, case in cases.items():
        path = ROOT / case['case_dir'] / 'receipt.json'
        receipt = read(path)
        if receipt['status'] != 'completed' or receipt['case'] != case or fp(ROOT / receipt['output_model']['path']) != receipt['output_model']:
            raise ValueError('Completed coherent model receipt mismatch')
        expected = fp(supplement_path if name.startswith('P0_M') else main_path)
        if receipt['protocol'] != expected:
            raise ValueError('Receipt protocol differs')
        paths.update([path, ROOT / receipt['output_model']['path']])
        receipts[name] = receipt
    for p, sha in protected.items():
        if fp(ROOT / p)['sha256'] != sha:
            raise AssertionError('Protected source changed')
    before = {fp(p)['path']: fp(p)['sha256'] for p in sorted(paths)}
    models, metadata, markers = {}, {}, {}
    for name, receipt in receipts.items():
        models[name], metadata[name], markers[name] = records(ROOT / receipt['output_model']['path'], cases[name])
    reference = 'P4_A1'
    selected_markers = set(main_protocol['calibration_markers'])
    reference_marker_contract = {n: {k: v for k, v in r.items() if not (n in selected_markers and k == '/location/#text')}
                                 for n, r in markers[reference].items()}
    checks = {}
    for name in cases:
        exact, semantic = differences(models[reference], models[name]), differences(models[reference], models[name], True)
        contract = {n: {k: v for k, v in r.items() if not (n in selected_markers and k == '/location/#text')}
                    for n, r in markers[name].items()}
        if contract != reference_marker_contract:
            raise AssertionError('Additional marker membership/attachment or unselected marker change')
        marker_location_changes = [n for n, r in markers[name].items() if r != markers[reference][n]]
        checks[name] = {'model': receipts[name]['output_model'],
                        'receipt': fp(ROOT / cases[name]['case_dir'] / 'receipt.json'),
                        'distal_mount_length_m': cases[name]['distal_mount_length_m'],
                        'static_ankle_offset_deg': cases[name]['static_ankle_offset_deg'],
                        'remaining_XML_field_count': len(models[name]), 'exact_numeric_token_equal': not exact,
                        'semantic_equal_with_serialization_tolerance': not semantic,
                        'exact_differences': exact, 'semantic_differences': semantic,
                        'marker_contract_preserved': True, 'selected_marker_locations_changed_vs_current': marker_location_changes,
                        'metadata': metadata[name]}
    after = {fp(p)['path']: fp(p)['sha256'] for p in sorted(paths)}
    if before != after:
        raise AssertionError('Source changed during structural audit')
    passed = all(c['semantic_equal_with_serialization_tolerance'] for c in checks.values())
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'PASS_cross_case_structural_isolation' if passed else 'FAIL_additional_nonmarker_physical_differences',
              'new_Claude_audit_content_read': False, 'reference_case': reference, 'case_count': len(cases), 'helper': fp(Path(__file__)),
              'normalization': {'removed': 'Exactly complete MarkerSet',
                                'common_sentinel': 'Only translation Y of pros_ankle socket_parent_frame PhysicalOffsetFrame, replaced by0m',
                                'retained': 'Every other root/Model/property/attribute including Appearance/mesh, body frames and COM/inertia, joint axes, coordinates/defaults/ranges, ForceSet/SEA/plugin, groups and geometry paths',
                                'numeric': 'Whitespace and float token spelling normalized; actual numeric tokens also compared exactly',
                                'serialization_tolerance': {'relative': REL_TOL, 'absolute': ABS_TOL, 'used_for_acceptance': True, 'needed_for_any_actual_difference': any(c['exact_differences'] for c in checks.values())}},
              'all17_remaining_XML_exact_numeric_tokens_equal': all(c['exact_numeric_token_equal'] for c in checks.values()),
              'cases': checks,
              'pylon_implementation': {
                  'intervention': 'Single physical distal-mount parent-offset Ytranslation at pros_ankle; fixed tibia_pylon body, fixed foot_l subtree properties',
                  'parent_body_socket': metadata[reference]['parent_socket'],
                  'rendered_mesh_properties_scaled_or_changed': False,
                  'body_local_mass_COM_inertia_changed': False,
                  'interpretation': 'The length variable is distal joint mounting distance. It does not remesh/stretch the rendered pylon or recompute hardware mass/inertia.',
                  'world_COM_note': 'Fixed body-local mass/COM/inertia does not imply fixed world COM or dynamic inertia tensor: changing mounting geometry and own IK changes world poses.'},
              'limitations': ['XML semantic isolation does not prove physical or clinical correctness of any length or foot pitch.',
                              'Selected marker locations and own fresh IK are deliberately different; this test does not establish kinematic identity.',
                              'Bodies mass/inertia are held fixed as a controlled intervention, not recomputed for physical hardware length.',
                              'No OpenSim model initialization, FK, solver, rollout or native inference executed in this audit.'],
              'source_integrity': {'protected_count': len(protected), 'consumed_count': len(before), 'before': before, 'after': after, 'all_unchanged': True}}
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': fp(output), 'status': result['status'], 'remaining_fields': len(models[reference]),
                      'consumed_count': len(before), 'all17_exact': result['all17_remaining_XML_exact_numeric_tokens_equal']}, allow_nan=False))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
