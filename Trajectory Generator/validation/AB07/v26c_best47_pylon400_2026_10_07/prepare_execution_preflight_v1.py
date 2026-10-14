"""Freeze one requested learned rollout and its existing prepared inputs, without simulation."""
from __future__ import annotations

import datetime as dt
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[3]
OLD = REPO / 'Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06'
CMC = REPO / 'validation/ab07_standing_geometry_fit_2026_10_07'
OUT = REPO / 'Trajectory Generator/runs/rollout/MLP_V26C_best47_AB07_pylon400_2026_10_07_6p11s'
PYTHON = sys.executable


def main():
    assert not OUT.exists(), 'Refuse an occupied rollout destination'
    assert not (AUDIT/'execution_manifest.json').exists(), 'Protocol already frozen'
    old = json.loads((OLD/'execution_manifest.json').read_text())
    setup_source = CMC/'dynamics/PYLON400_REFERENCE/setup.xml'
    setup = AUDIT/'setup.xml'
    with setup.open('xb') as f: f.write(setup_source.read_bytes())
    tree = ET.parse(setup).getroot()
    model = REPO / tree.findtext('.//model_file')
    ik = REPO / tree.findtext('.//kinematics_file')
    model_xml = ET.parse(model).getroot()
    distal = model_xml.find('.//PinJoint[@name="pros_ankle"]')
    if distal is None:
        candidates = [x for x in model_xml.iter('PinJoint') if 'ankle' in x.get('name', '').lower()]
        assert len(candidates) == 1
        distal = candidates[0]
    locations = [tuple(map(float, n.text.split())) for n in distal.findall('.//PhysicalOffsetFrame/translation')]
    assert any(abs(v[1]+.4) < 1e-12 for v in locations), locations
    argv = list(old['argv'])
    argv[0] = PYTHON
    argv[3] = str(AUDIT/'run_rollout.py')
    def change(flag, value): argv[argv.index(flag)+1] = str(value)
    change('--setup-xml', setup)
    change('--episode-duration', '6.11')
    change('--episode-start-offset-s', '2.231740519774012')
    change('--max-steps', '611')
    change('--output-dir', OUT)
    evaluator_path = REPO/'Trajectory Generator/baseline_MLP/rollout_eval.py'
    spec = importlib.util.spec_from_file_location('ab07_pylon400_preflight_eval', evaluator_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    previous_argv = sys.argv
    try:
        sys.argv = argv[3:]
        resolved = vars(module.parse_args())
    finally:
        sys.argv = previous_argv
    allowed = {'setup_xml', 'output_dir', 'episode_duration', 'episode_start_offset_s', 'max_steps'}
    diffs = {k: {'old':v, 'new':resolved.get(k)} for k,v in old['resolved_cli'].items()
             if resolved.get(k) != v}
    assert set(diffs) <= allowed, diffs
    assert resolved['action_selection'] == 'deterministic' and resolved['seed'] == 123
    before_old = json.loads((OLD/'input_fingerprint_before.json').read_text())
    sources = {REPO/name for name in before_old}
    sources.update([model, ik, setup, setup_source, AUDIT/'run_rollout.py', Path(__file__).resolve(),
                    CMC/'preparation/cases/PYLON400_REFERENCE/receipt.json',
                    CMC/'dynamics/PYLON400_REFERENCE/run/receipt.json',
                    CMC/'analysis/lean_numeric_summary.json', CMC/'analysis/phase_protocol.json',
                    CMC/'analysis/numerical_protocol.json'])
    protected = {}
    for p in sorted(sources):
        assert p.is_file(), p
        protected[str(p.relative_to(REPO))] = hashlib.sha256(p.read_bytes()).hexdigest()
    fingerprints = {k:dict(v) for k,v in old['source_fingerprints'].items()}
    for key,path in [('setup',setup),('model',model),('ik',ik),('launcher',AUDIT/'run_rollout.py')]:
        fingerprints[key]={'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    assert fingerprints['checkpoint_state']['sha256'] == '2d1d9b30f2b043f4b3b7b4347c06e9b3aace1c7d089b33e2893930554d84bd7c'
    start = float(tree.findtext('.//t_start')) + float(resolved['episode_start_offset_s'])
    end = start + float(resolved['episode_duration'])
    assert abs(start-12.591740519774012) < 1e-12
    assert end <= float(tree.findtext('.//t_end'))
    manifest = {
        'schema_version':1, 'instruction_check_token':'CMC_AGENT_OK_2026',
        'created_utc':dt.datetime.now(dt.timezone.utc).isoformat(), 'status':'FROZEN_BEFORE_NATIVE',
        'argv':argv, 'cwd':str(REPO), 'output_dir':str(OUT),
        'checkpoint_logical_iteration':47, 'historical_summary':old['historical_summary'],
        'paired_prescribed_session':str(CMC/'dynamics/PYLON400_REFERENCE/run'),
        'requested_time_range_s':[start,end], 'duration_s':6.11, 'seed':123,
        'primary_reference_range_s':[14.087704688172044,18.69078019047619],
        'distal_mount_length_mm':400., 'resolved_cli':resolved,
        'CLI_differences_from_previous_learned':diffs,
        'source_fingerprints':fingerprints, 'protected_fingerprints_before':protected,
        'protected_file_count':len(protected),
        'protocol_deviations_from_training': {
            'subject':'Coherently prepared AB07 pylon400 model and own fresh full IK replace the AB06 training bundle',
            'episode_start_offset_s':{'training':1.956870983805102,'evaluation':resolved['episode_start_offset_s'],
                                      'purpose':'Same reset and requested physical support as the completed AB07pylon400 CMC-like test'},
            'duration':'6.11s requested, with one startup nominal cycle and three predefined measured nominal cycles',
            'morphology_profile':'Frozen AB06 corridor and reward retained; no reward retuning',
        },
        'CMC_comparison_limit':'Same subject, prepared model, own IK, GRF and time support. Learned action source, clock/slew and actor-FSM settings are preserved from best47; they differ from prescribed baseline. Not an isolated policy-only ablation.',
        'detector_profile_recalibrations':0, 'native_runs_before_this_freeze':0,
        'allowed_execution_attempts':1, 'training_or_weight_updates':False,
        'capture_scope':'Preserved native evaluator and pass-through legacy observer; no action, observation, RNG, reset, physics, detector or stop-condition transformation',
    }
    with (AUDIT/'execution_manifest.json').open('x') as f:
        json.dump(manifest,f,indent=2,allow_nan=False); f.write('\n')
    print(json.dumps({'status':manifest['status'], 'protected_files':len(protected), 'CLI_changed_fields':list(diffs),
                      'range_s':[start,end], 'output_dir':str(OUT)}))


if __name__ == '__main__': main()
