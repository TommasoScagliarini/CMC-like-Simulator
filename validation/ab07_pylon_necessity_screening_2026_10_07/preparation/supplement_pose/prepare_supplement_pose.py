"""Freeze/build three predeclared coherent short-pylon angular supplements.

The reviewed main14 preparation recipe is reused through an isolated function
globals dictionary. Its source, globals, frozen protocol and outputs are never
modified. Each case recalibrates the same12markers and solves fresh fullIK.
No dynamics, force fitting, MATLAB, source edits or output overwrites.
"""
from datetime import datetime, timezone
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import types

HERE = Path(__file__).resolve().parent
PREPARATION = HERE.parent
ROOT = PREPARATION.parents[2]
PROTOCOL = HERE / 'supplement_protocol.json'
MAIN_HELPER = PREPARATION / 'prepare_pylon_sweep.py'
spec = importlib.util.spec_from_file_location('supplement_original_sweep_recipe', MAIN_HELPER)
main_recipe = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = main_recipe
spec.loader.exec_module(main_recipe)
fp, read, recheck, write_new = main_recipe.fp, main_recipe.read, main_recipe.recheck, main_recipe.write_new


def freeze():
    if PROTOCOL.exists():
        raise FileExistsError('Preserve frozen supplementary protocol')
    main = read(PREPARATION / 'preparation_protocol.json')
    seal = read(PREPARATION / 'scientific_screening_seal.json')
    protected = dict(main['protected_sources_before'])
    protected.update(seal['input_integrity']['after'])
    for path in (Path(__file__), MAIN_HELPER, PREPARATION / 'preparation_protocol.json',
                 PREPARATION / 'batch_receipt.json', PREPARATION / 'contact_replay_screening.json',
                 PREPARATION / 'scientific_screening_seal.json'):
        item = fp(path)
        protected[item['path']] = item['sha256']
    recheck(protected)
    obj = copy.deepcopy(main)
    obj.update(created_utc=datetime.now(timezone.utc).isoformat(),
               status='frozen_before_supplement_preparation_and_contact_results',
               scope='Three fixed angular counterexamples, separate from immutable main14',
               main_protocol=fp(PREPARATION / 'preparation_protocol.json'),
               main_preparation_helper=fp(MAIN_HELPER),
               new_Claude_audit_content_read=False)
    original = main['cases']['P0_A0']
    cases = {}
    for name, angle in [('P0_M2', -2.), ('P0_M5', -5.), ('P0_M10', -10.)]:
        case = copy.deepcopy(original)
        case.update(id=name, angle_branch=name.split('_')[1],
                    static_ankle_offset_deg=angle,
                    case_dir=(HERE / 'cases' / name).relative_to(ROOT).as_posix(),
                    preparation_kind='fresh_marker_calibration_full_IK', reused_endpoint=None)
        cases[name] = case
    obj['confirmed_grid'] = {
        'lengths_mm': [original['distal_mount_length_mm']],
        'exact_distal_mount_length_m': original['distal_mount_length_m'],
        'static_ankle_offsets_deg': [-2., -5., -10.],
        'total_cells': 3, 'fresh_build_cells': 3, 'immutable_reused_cells': 0,
        'endpoint_unit_note': 'Exact shortest endpoint metre value retained; no reconversion from millimetres'}
    obj['cases'] = cases
    obj['execution_order'] = list(cases)
    obj['rationale'] = {
        'intervention': 'Change static calibration ankle angle at fixed proportional pylon length; regenerate own marker locations and fullIK coherently',
        'selection': 'All three angles fixed prospectively, not chosen from their force outcomes; all three proceed to root-owned dynamics',
        'limits': 'Procedural reference-contact sensitivity; a loaded geometric counterexample does not establish a physically validated mount, and all failures would not establish universal impossibility'}
    obj['prospective_dynamic_plan'].update(
        cells='All3 supplementary cells, plus the immutable main14 and AB06 control; all18 before the first native result',
        duration_s=6.11, requested_end_s=12.591740519774012 + 6.11,
        coverage_amendment='Prospective root override from6.10 to6.11 to cover closingHS in pre-step recorded forces; primary windows unchanged', owner='root')
    obj['FK_plan'].update(
        endpoints='No reuse of contact outcomes for these3; each fresh coherent own model/fullIK',
        comparisons='Frozen main P0_A0 and P4_A1 as reference cases only, no model/motion hybrids')
    obj['protected_count'] = len(protected)
    obj['protected_sources_before'] = protected
    recheck(protected)
    write_new(PROTOCOL, obj)
    print(json.dumps({'protocol': fp(PROTOCOL), 'cases': cases,
                      'protected_count': len(protected)}, allow_nan=False), flush=True)


def build(name):
    # A copied globals mapping redirects only protocol and metadata helper path.
    # Main module globals and the exact reviewed build implementation stay intact.
    context = dict(main_recipe.build.__globals__)
    context.update(PROTOCOL=PROTOCOL, __file__=str(Path(__file__).resolve()))
    isolated_build = types.FunctionType(main_recipe.build.__code__, context,
                                        main_recipe.build.__name__, main_recipe.build.__defaults__,
                                        main_recipe.build.__closure__)
    isolated_build(name)


def build_all(peer_reviewed):
    if not peer_reviewed:
        raise ValueError('Peer review PASS is required before the three builds')
    protocol = read(PROTOCOL)
    recheck(protocol['protected_sources_before'])
    logs = HERE / 'logs'
    logs.mkdir(exist_ok=False)
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'status': 'running',
              'helper': fp(Path(__file__)), 'protocol': fp(PROTOCOL), 'cases': []}
    for name in protocol['execution_order']:
        print('SUPPLEMENT_PREPARATION', name, flush=True)
        log = logs / (name + '.log')
        with log.open('x', encoding='utf-8') as stream:
            process = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--case', name],
                                     cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=False)
        receipt_path = ROOT / protocol['cases'][name]['case_dir'] / 'receipt.json'
        row = {'case': name, 'returncode': process.returncode, 'log': fp(log),
               'receipt': fp(receipt_path) if receipt_path.is_file() else None}
        result['cases'].append(row)
        if process.returncode or not receipt_path.is_file() or read(receipt_path)['status'] != 'completed':
            result['status'] = 'failed'
            break
        saved = read(receipt_path)
        print('COMPLETED', name, saved['output_model']['sha256'], saved['full_IK']['sha256'], flush=True)
    else:
        result['status'] = 'completed'
    recheck(protocol['protected_sources_before'])
    result.update(completed_utc=datetime.now(timezone.utc).isoformat(), protected_inputs_unchanged=True,
                  protected_input_count=len(protocol['protected_sources_before']))
    write_new(HERE / 'batch_receipt.json', result)
    print(json.dumps({'status': result['status'], 'cases': len(result['cases']),
                      'batch_receipt': fp(HERE / 'batch_receipt.json')}, allow_nan=False), flush=True)
    if result['status'] != 'completed':
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--freeze', action='store_true')
    group.add_argument('--case', choices=['P0_M2', 'P0_M5', 'P0_M10'])
    group.add_argument('--build-all', action='store_true')
    parser.add_argument('--peer-reviewed', action='store_true')
    args = parser.parse_args()
    if args.freeze:
        freeze()
    elif args.case:
        build(args.case)
    else:
        build_all(args.peer_reviewed)


if __name__ == '__main__':
    main()
