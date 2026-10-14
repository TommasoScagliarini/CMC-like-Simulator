"""Controlled geometry/reference contact interventions, without dynamics.

Run --prepare first, then --evaluate with an OpenSim-enabled Python. Every write
stays beside this helper. Original bundles, runtime code, profiles and previous
audits are hash protected. Variants are diagnostic FK objects, not valid dynamic
subject bundles. No assembly, IK solver, integration or policy inference occurs.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[2]
PREVIOUS = ROOT / 'validation/ab06_ab07_preparation_vs_model_2026_10_07'
PAIR = ROOT / 'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
MODELS = {s: ROOT / f'models/{s}_SEASEA_Threadmill/{s}_SEASEA_stiff321_500_pi.osim'
          for s in ('AB06', 'AB07')}
OLD_MODEL = ROOT / 'validation/ab07_detector_investigation_2026_10_06/stage3/baseline/models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim'
OLD_IK = OLD_MODEL.parent / 'data/IK_results_AB07_SEASEA.mot'
CHAIN = ('ground_pelvis', 'hip_l', 'osseo', 'pros_knee', 'pros_ankle')
SITES = {'A': ("./Model/JointSet/objects/CustomJoint[@name='hip_l']/frames/PhysicalOffsetFrame[@name='pelvis_offset']/translation",),
         'M': ("./Model/JointSet/objects/WeldJoint[@name='osseo']/frames/PhysicalOffsetFrame[@name='transfemur_offset']/translation",
               "./Model/JointSet/objects/PinJoint[@name='pros_ankle']/frames/PhysicalOffsetFrame[@name='tibia_pylon_offset']/translation")}


def fp(path):
    path = Path(path).resolve()
    return {'path': path.relative_to(ROOT).as_posix(),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def numeric_text(text):
    tokens = (text or '').split()
    try:
        return tuple(float(t) for t in tokens)
    except ValueError:
        return tuple(tokens)


def semantic(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            numeric_text(element.text), tuple(semantic(c) for c in element))


def positions(tree):
    return {key: [list(numeric_text(tree.find(p).text)) for p in paths]
            for key, paths in SITES.items()}


def masked(tree):
    result = copy.deepcopy(tree)
    for p in SITES['A']:
        result.find(p).text = '0 0 0'
    for p in SITES['M']:
        values = list(numeric_text(result.find(p).text))
        values[1] = 0.0
        result.find(p).text = ' '.join(format(x, '.17g') for x in values)
    return result


def effective_chain(tree):
    result = masked(tree)
    for joint in result.findall('./Model/JointSet/objects/*'):
        if joint.get('name') not in CHAIN:
            continue
        for axis in joint.findall('./SpatialTransform/TransformAxis'):
            multiplier = axis.find('MultiplierFunction')
            constant = axis.find('MultiplierFunction/function/Constant')
            if multiplier is not None and constant is not None and float(constant.findtext('value')) == 0.0:
                replacement = copy.deepcopy(constant)
                replacement.attrib = dict(multiplier.attrib)
                axis.remove(multiplier)
                axis.append(replacement)
    return tuple(semantic(result.find(f"./Model/JointSet/objects/*[@name='{n}']")) for n in CHAIN)


def write_json_new(path, value):
    with path.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')


def prepare():
    source_index = json.loads((PREVIOUS / 'attribution_findings.json').read_text())['audit_input_integrity']['checks']
    sources = {ROOT / x['path'] for x in source_index}
    sources.update([Path(__file__), OLD_MODEL, OLD_IK, PREVIOUS / 'contact_replay.json',
                    PREVIOUS / 'model_preservation_audit.json', PREVIOUS / 'attribution_findings.json',
                    PREVIOUS / 'contact_preparation_probe.json', PREVIOUS / 'data_preparation_audit_final.json'])
    before = {fp(p)['path']: fp(p)['sha256'] for p in sorted(sources)}
    for x in source_index:
        if before[x['path']] != x['sha256']:
            raise AssertionError('Previous audit input changed: ' + x['path'])
    trees = {s: ET.parse(p).getroot() for s, p in MODELS.items()}
    old = ET.parse(OLD_MODEL).getroot()
    if effective_chain(trees['AB06']) != effective_chain(trees['AB07']):
        raise AssertionError('Left-chain differences exceed declared A/M after effective Constant0 normalization')
    if effective_chain(old) != effective_chain(trees['AB07']):
        raise AssertionError('Archived left-chain differences exceed declared A/M')
    variants = {}
    for anatomy, mount in itertools.product(('AB06', 'AB07'), repeat=2):
        key = f'A{anatomy[-2:]}_M{mount[-2:]}'
        tree = copy.deepcopy(trees['AB06'])
        for p in SITES['A']:
            tree.find(p).text = trees[anatomy].find(p).text
        for p in SITES['M']:
            target = list(numeric_text(tree.find(p).text))
            target[1] = numeric_text(trees[mount].find(p).text)[1]
            tree.find(p).text = ' '.join(format(x, '.17g') for x in target)
        if semantic(masked(tree)) != semantic(masked(trees['AB06'])):
            raise AssertionError('Variant changed protected XML outside A/M')
        path = AUDIT / f'{key}.osim'
        payload = MODELS['AB06'].read_bytes() if key == 'A06_M06' else ET.tostring(tree, encoding='utf-8', xml_declaration=True)
        with path.open('xb') as handle:
            handle.write(payload)
        variants[key] = dict(fp(path), anatomy_site_source=anatomy, mount_source=mount,
                             geometry=positions(tree), protected_XML_outside_A_M_unchanged=True)
    for level, source in [('pre_ground', old), ('current', trees['AB07'])]:
        tree = copy.deepcopy(trees['AB07'])
        p = SITES['M'][1]
        tree.find(p).text = source.find(p).text
        if semantic(masked(tree)) != semantic(masked(trees['AB07'])):
            raise AssertionError('Registration variant changed protected XML')
        path = AUDIT / f'AB07_registration_{level}.osim'
        with path.open('xb') as handle:
            handle.write(ET.tostring(tree, encoding='utf-8', xml_declaration=True))
        variants[f'registration_{level}'] = dict(fp(path), geometry=positions(tree),
                                               protected_XML_outside_A_M_unchanged=True)
    after = {fp(p)['path']: fp(p)['sha256'] for p in sorted(sources)}
    if before != after:
        raise AssertionError('Protected source changed during preparation')
    protocol = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'instruction_check_token': 'CMC_AGENT_OK_2026',
        'status': 'prepared_controlled_FK_interventions', 'helper': fp(__file__),
        'factorial': {
            'A': 'Healthy-preserved left hip parent-frame translation, AB06/AB07. All other effective chain transforms are identical after declared Constant0 normalization.',
            'M': 'Fitted/registered osseo parent Y and ankle parent Y, AB06/AB07. This adjustment depends partly on anatomy; it is not classified automatically as an error.',
            'Q': 'All21 production-filtered coordinate values and spline derivatives, AB06/AB07, each on own frozen three nominal cycles without time/phase warping.',
            'response': 'Frozen left online contact force mean[N] and impulse[N s], conditional differences and interactions; no causal percentages or Shapley.',
            'fixed': 'AB06 base masses/COM/inertias/muscles/markers/SEA and all XML outside three translation fields; frozen force profile and belt speed.',
            'direct_anatomy_scope': 'Hip offset is the only subject-varying retained anatomy affecting this explicit left-foot FK chain. Mass/recruitment may influence attained dynamics indirectly and are not tested here.'},
        'lineage_2_by_2': 'Current versus archived AB07 distal mount crossed with complete current versus archived production-filtered IK. Endpoints quantify direct response of this saved ground-registration stage, not total preparation from healthy anatomy.',
        'limits': ['Hybrid poses need not be mechanically or anatomically feasible; no assembly/integration/IK optimization.',
                   'Q includes subject gait, marker/IK preparation, trial velocity and selected cycle windows; it is not an anatomical-only or marker-only factor.',
                   'Across Q, mean force compares each own frozen-duration average; impulse duration remains explicit. No waveform phase correspondence is assumed.',
                   'Geometry/reference interactions prevent unique order-independent biological/preparation causal shares.'],
        'chain_equivalence_check': 'PASS_except_declared_A_M', 'geometry_by_source': {s: positions(t) for s,t in trees.items()},
        'variants': variants, 'sources_before': before, 'sources_after_prepare': after}
    write_json_new(AUDIT / 'protocol.json', protocol)
    print('PREPARED', len(variants), 'XML copies;', len(before), 'protected sources', flush=True)


def load_replay():
    sys.path.insert(0, str(ROOT))
    spec = importlib.util.spec_from_file_location('geometry_audit_contact_replay', PREVIOUS / 'contact_replay.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def reference(cr, subject, old=False):
    selection = json.loads((PAIR / 'selection.json').read_text())['subjects'][subject]
    receipt = json.loads((ROOT / f'results/{subject}_paired_prescribed_HS_9s_2026_10_07/receipt.json').read_text())
    times_native, _, _ = cr.read_storage(ROOT / f'results/{subject}_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto')
    cycles = selection['primary_cycles']
    start, stop = cycles[0]['hs_s'], cycles[-1]['next_hs_s']
    grid = np.unique(np.r_[times_native[(times_native > start) & (times_native < stop)],
                           [c[k] for c in cycles for k in ('hs_s','to_s','next_hs_s')]])
    model_path = MODELS[subject]
    ik_path = OLD_IK if old else ROOT / selection['source_fingerprints']['IK']['path']
    source_t, _, header = cr.read_storage(ik_path)
    if 'indegrees=yes' not in header.lower() or source_t[0] > grid[0] or source_t[-1] < grid[-1]:
        raise AssertionError('Reference units/support mismatch')
    cfg_dict = dict(receipt['effective_simulator_config'])
    cfg_dict.update(model_bundle_dir=str(model_path.parent), model_file=str(model_path),
                    kinematics_file=str(ik_path), external_loads_xml=str(ROOT / selection['source_fingerprints']['external_loads']['path']))
    cfg = cr.SimulatorConfig(**cfg_dict)
    interp = cr.KinematicsInterpolator(cfg)
    names = list(interp.coord_names)
    q = np.column_stack([interp._splines[n](grid) for n in names])
    qdot = np.column_stack([interp._splines[n](grid,1) for n in names])
    return {'times': grid, 'q': q, 'qdot': qdot, 'names': names, 'cycles': cycles,
            'subject': subject, 'IK': fp(ik_path), 'source_times': source_t,
            'filter': {'enabled': cfg.enable_kinematics_lowpass_filter,
                       'cutoff_hz': cfg.kinematics_lowpass_cutoff_hz,
                       'order': cfg.kinematics_lowpass_order, 'dt_s': cfg.kinematics_resample_dt}}


def evaluate_case(cr, variant, ref, profile, hull, binary, export=None):
    path = ROOT / variant['path']
    cr._configure_geometry_search_paths(str(MODELS['AB06']))
    model = cr.opensim.Model(str(path))
    if model.getConstraintSet().getSize():
        raise AssertionError('FK variant unexpectedly constrained')
    cr.add_online_grf_forces(model, profile, applies_force=False, name_prefix='contact_replay_')
    state = model.initSystem()
    coordinates = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    names = [c.getName() for c in coordinates]
    indices = [ref['names'].index(n) for n in names]
    q, qdot = ref['q'][:,indices], ref['qdot'][:,indices]
    range_bad = {c.getName(): int(np.count_nonzero((q[:,i] < c.getRangeMin()-1e-10) | (q[:,i] > c.getRangeMax()+1e-10)))
                 for i,c in enumerate(coordinates)}
    if not np.isfinite(q).all() or not np.isfinite(qdot).all():
        raise AssertionError('Nonfinite reference')
    check_indices = set(np.linspace(0,len(ref['times'])-1,11,dtype=int).tolist())
    law = cr.ContactLaw(profile)
    values, parity = cr.replay(model,state,coordinates,model.getBodySet().get('foot_l'),law,
                             ref['times'],q,qdot,hull,binary,check_indices)
    # Separate dimensioned parity diagnostics; do not use the original mixed-unit maximum.
    force_error = pen_error = 0.0
    for i in sorted(check_indices):
        cr.set_pose(model,state,coordinates,q[i],qdot[i],ref['times'][i])
        for k,sphere in enumerate(profile.spheres):
            record = model.getForceSet().get('contact_replay_'+sphere.name).getRecordValues(state)
            force_error = max(force_error,max(abs(record.get(a)-values['force'][i,k,a]) for a in range(3)),
                              abs(record.get(9)-values['normal'][i,k]))
            pen_error = max(pen_error,abs(record.get(10)-values['penetration'][i,k]))
    if force_error > 1e-8 or pen_error > 1e-10 or parity['station_transform_max_abs_error_m_or_m_s'] > 1e-10:
        raise AssertionError('Contact/station parity failed')
    aggregate = values['force'].sum(axis=1)
    def summary(mask):
        t, force = ref['times'][mask],aggregate[mask]
        impulse = float(cr.integrate(force[:,1],t))
        duration = float(t[-1]-t[0])
        return {'duration_s':duration,'impulse_ns':impulse,'mean_force_n':impulse/duration,
                'peak_force_n':float(force[:,1].max()),
                'maximum_sphere_penetration_m':float(values['penetration'][mask].max()),
                'per_sphere_impulse_ns':dict(zip([s.name for s in profile.spheres],cr.integrate(values['force'][mask,:,1],t).tolist()))}
    full = summary(np.ones(len(ref['times']),dtype=bool))
    cycles = [summary((ref['times'] >= c['hs_s']) & (ref['times'] <= c['next_hs_s'])) for c in ref['cycles']]
    # Values/rates are assigned without assembly; test getValue/getSpeedValue at probes.
    qerr = derr = 0.0
    for i in sorted(check_indices):
        cr.set_pose(model,state,coordinates,q[i],qdot[i],ref['times'][i])
        qerr=max(qerr,max(abs(c.getValue(state)-q[i,k]) for k,c in enumerate(coordinates)))
        derr=max(derr,max(abs(c.getSpeedValue(state)-qdot[i,k]) for k,c in enumerate(coordinates)))
    if qerr != 0.0 or derr != 0.0:
        raise AssertionError('Silent state adjustment/clamp')
    if export:
        np.savez_compressed(AUDIT/export,times_s=ref['times'],coordinate_names=np.array(names),q=q,qdot=qdot,
                            sphere_names=np.array([s.name for s in profile.spheres]),
                            contact_centers_m=values['centers'],contact_velocity_m_s=values['velocity'],
                            aggregate_force_n=aggregate)
    return {'aggregate':full,'cycles':cycles,'validation':{'status':'PASS','q_assignment_max_abs_error':qerr,
             'qdot_assignment_max_abs_error':derr,'range_exceedance_samples_by_coordinate':range_bad,
             'cpp_force_error_n':force_error,'cpp_penetration_error_m':pen_error,
             'station_transform_error_m_or_m_s':parity['station_transform_max_abs_error_m_or_m_s'],
             'native_mass_kg_fixed_for_geometry_family':float(model.getTotalMass(state))},
            'variant':variant,'reference_source':ref['IK'],'filter':ref['filter'],'pose_count':len(ref['times'])}


def evaluate():
    protocol=json.loads((AUDIT/'protocol.json').read_text())
    before=protocol['sources_before']
    for raw,sha in before.items():
        if fp(ROOT/raw)['sha256'] != sha:
            raise AssertionError('Protected source changed before FK: '+raw)
    for v in protocol['variants'].values():
        if fp(ROOT/v['path'])['sha256'] != v['sha256']:
            raise AssertionError('Prepared XML variant changed')
    cr=load_replay()
    cr._load_plugin(str(ROOT/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    cr._load_plugin(str(ROOT/'plugins/OnlineGRFContact'))
    raw=json.loads(cr.PRIMARY.read_text())
    profile=cr.OnlineGRFProfile.from_mapping(dict(raw,spheres=[s for s in raw['spheres'] if s['side']=='left']),required_sides=('left',))
    hull,_=cr.mesh_hull_vertices();binary=json.loads(cr.BINARY.read_text())
    refs={s:reference(cr,s) for s in ('AB06','AB07')}
    refs['AB07_pre_ground']=reference(cr,'AB07',old=True)
    if not np.array_equal(refs['AB07']['source_times'],refs['AB07_pre_ground']['source_times']):
        raise AssertionError('Archived/current source clocks differ')
    factorial={}
    for anatomy,mount,motion in itertools.product(('AB06','AB07'),repeat=3):
        key=f'A{anatomy[-2:]}_M{mount[-2:]}_Q{motion[-2:]}'
        print('FK',key,flush=True)
        export=f'baseline_{motion}.npz' if anatomy==mount==motion else None
        factorial[key]=evaluate_case(cr,protocol['variants'][key[:7]],refs[motion],profile,hull,binary,export)
    previous=json.loads((PREVIOUS/'contact_replay.json').read_text())
    endpoint_checks={}
    for s in ('AB06','AB07'):
        result=factorial[f'A{s[-2:]}_M{s[-2:]}_Q{s[-2:]}']['aggregate']
        baseline=previous['subjects'][s]['cases']['filtered_IK']['aggregate']
        error={k:abs(result[k]-baseline[k]) for k in ('duration_s','impulse_ns','mean_force_n','maximum_sphere_penetration_m')}
        if error['impulse_ns'] > 1e-7 or error['mean_force_n'] > 1e-8 or error['maximum_sphere_penetration_m'] > 1e-10:
            raise AssertionError('Factorial endpoint differs from immutable original filtered replay')
        endpoint_checks[s]=dict(status='PASS',absolute_errors=error)
    effects={}
    for q in ('06','07'):
        f=lambda a,m:factorial[f'A{a}_M{m}_Q{q}']['aggregate']['mean_force_n']
        effects['Q'+q]={'A07_minus_A06_at_M06_n':f('07','06')-f('06','06'),
                       'A07_minus_A06_at_M07_n':f('07','07')-f('06','07'),
                       'M07_minus_M06_at_A06_n':f('06','07')-f('06','06'),
                       'M07_minus_M06_at_A07_n':f('07','07')-f('07','06'),
                       'A_M_interaction_n':f('07','07')-f('07','06')-f('06','07')+f('06','06')}
    motion_effects={f'A{a}_M{m}':factorial[f'A{a}_M{m}_Q07']['aggregate']['mean_force_n']-factorial[f'A{a}_M{m}_Q06']['aggregate']['mean_force_n']
                    for a,m in itertools.product(('06','07'),repeat=2)}
    lineage={}
    for geom,motion in itertools.product(('pre_ground','current'),repeat=2):
        key=f'G_{geom}__Q_{motion}'
        print('FK',key,flush=True)
        ref=refs['AB07_pre_ground'] if motion=='pre_ground' else refs['AB07']
        lineage[key]=evaluate_case(cr,protocol['variants']['registration_'+geom],ref,profile,hull,binary)
    lf=lambda g,q:lineage[f'G_{g}__Q_{q}']['aggregate']['mean_force_n']
    lineage_effect={'geometry_current_minus_pre_at_pre_reference_n':lf('current','pre_ground')-lf('pre_ground','pre_ground'),
                    'geometry_current_minus_pre_at_current_reference_n':lf('current','current')-lf('pre_ground','current'),
                    'full_reference_current_minus_pre_at_pre_geometry_n':lf('pre_ground','current')-lf('pre_ground','pre_ground'),
                    'full_reference_current_minus_pre_at_current_geometry_n':lf('current','current')-lf('current','pre_ground'),
                    'interaction_n':lf('current','current')-lf('current','pre_ground')-lf('pre_ground','current')+lf('pre_ground','pre_ground'),
                    'matched_current_minus_archived_mean_force_n':lf('current','current')-lf('pre_ground','pre_ground')}
    after={raw:fp(ROOT/raw)['sha256'] for raw in before}
    if before != after:
        raise AssertionError('Protected source changed during FK')
    result={'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026',
            'status':'PASS_controlled_direct_FK_interventions','helper':fp(__file__),'protocol':fp(AUDIT/'protocol.json'),
            'factorial':factorial,'endpoint_original_replay_checks':endpoint_checks,
            'geometry_conditional_effects_n':effects,'full_reference_Q07_minus_Q06_at_fixed_geometry_n':motion_effects,
            'registration_lineage_2_by_2':lineage,'registration_lineage_conditional_effects_n':lineage_effect,
            'interpretation_limits':protocol['limits']+['Lineage endpoints isolate the saved ground-registration stage direct geometry/reference response, not every preparation choice or dynamically attained force.'],
            'baseline_exports':[fp(AUDIT/f'baseline_{s}.npz') for s in ('AB06','AB07')],
            'inputs_integrity':{'status':'PASS','protected_count':len(before),'all_unchanged':before==after,'before':before,'after':after}}
    write_json_new(AUDIT/'factorial_geometry_fk.json',result)
    print(json.dumps({'factorial_mean_n':{k:v['aggregate']['mean_force_n'] for k,v in factorial.items()},
                      'effects_n':effects,'reference_effect_n':motion_effects,'lineage_effects_n':lineage_effect},indent=2),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare',action='store_true')
    parser.add_argument('--evaluate',action='store_true')
    args=parser.parse_args()
    if args.prepare == args.evaluate:
        parser.error('Choose exactly one of --prepare and --evaluate')
    prepare() if args.prepare else evaluate()
