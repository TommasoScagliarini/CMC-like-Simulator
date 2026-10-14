"""Resume the paired protocol after the cumulative iteration audit mismatch.

No training implementation or completed artifact is changed. The original
execute.py is retained verbatim with its original provenance manifest.
"""
import hashlib
import json
import pickle
import sys
import time
from pathlib import Path
import numpy as np
import yaml
import execute as base

HERE, ROOT, PACKAGE = base.HERE, base.ROOT, base.PACKAGE


def audit_training(folder, expected_total, expected_run):
    summary = json.loads((folder / 'summary.json').read_text())
    assert summary['ok'] and summary['iterations_completed'] == expected_total
    assert summary['iterations_completed_this_process'] == expected_run
    assert not summary.get('skipped_iterations')
    rows = [json.loads(line) for line in (folder / 'train_iterations.jsonl').read_text().splitlines() if line.strip()]
    assert rows[-1]['iteration'] == expected_total
    for row in rows:
        audit = row['unified_phase_audit']
        assert audit['critic_changed'] and row['kl_update_guard']['pass']
        assert audit['actor_exact'] if row['unified_phase'] == 'critic_warmup' else audit['mean_changed']
    state = pickle.loads((folder / 'checkpoint_last/learner_group/learner/rl_module/default_policy/module_state.pkl').read_bytes())
    np.testing.assert_array_equal(state['pi.1.weight'][2:], 0)
    np.testing.assert_allclose(np.exp(state['pi.1.bias'][2:]), .01, rtol=1e-6)
    return {'folder': str(folder), 'total_iterations': expected_total,
            'new_iterations': expected_run, 'sigma_constant': True,
            'phases': [row['unified_phase'] for row in rows]}


def verify_sources():
    for name in ('source_manifest.json', 'execution_code_manifest.json', 'continuation_code_manifest.json'):
        manifest = json.loads((HERE / name).read_text())
        assert all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == digest for p, digest in manifest.items()), name


def main():
    base.write('continuation_code_manifest', {
        str(Path(__file__).resolve().relative_to(ROOT)): hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    })
    verify_sources()
    records = json.loads((HERE / 'results.json').read_text())
    assert not records.get('finished')
    assert records['decisions'][-1]['stage'] == 1
    assert records['decisions'][-1]['block'] == 1
    assert records['decisions'][-1]['decision'] == 'extend'
    total = records['decisions'][-1]['actor_updates_per_arm'] + 1
    previous = {arm: base.TRAIN / arm / 'stage01' for arm in base.ARMS}
    stop = 'full_curriculum_completed'
    records['orchestration_recovery'] = {
        'reason': 'iterations_run is cumulative; resumed-process count is iterations_completed_this_process',
        'completed_training_reused': 'train_ex_novo_s1_b2',
        'production_code_changed': False,
        'started': time.time(),
    }
    for stage, probability in enumerate((.75, .5, .25, 0.), 1):
        outputs, configs = {}, {}
        for arm in base.ARMS:
            config = yaml.safe_load((HERE / f'{arm}.yaml').read_text())
            output = base.TRAIN / arm / f'stage{stage:02d}'
            outputs[arm] = output
            config['run']['output_dir'] = str(output)
            if stage > 1:
                config['run'].update(init_actor_from=None, resume_from=str(previous[arm] / 'checkpoint_last'), resume_target_curriculum_fork=True)
                config['target_availability'].update(target_availability_probability=probability,
                    target_curriculum_probabilities=[probability], target_curriculum_iteration_offset=total)
            configs[arm] = config
        for block in ((2,) if stage == 1 else (1, 2)):
            new, target = 5, total + 5
            for arm in base.ARMS:
                config = configs[arm]
                config['simulation']['iterations'] = target
                if block == 2:
                    config['run'].update(init_actor_from=None, resume_from=str(outputs[arm] / 'checkpoint_last'))
                label = f'train_{arm}_s{stage}_b{block}'
                path = HERE / f'run_{arm}_s{stage}_b{block}.yaml'
                if path.exists():
                    assert yaml.safe_load(path.read_text()) == config
                else:
                    path.write_text(yaml.safe_dump(config, sort_keys=False))
                base.write('progress', {'phase': 'training', 'arm': arm, 'stage': stage,
                    'block': block, 'visibility': probability, 'target_iteration': target})
                receipt = HERE / f'{label}_execution.json'
                if receipt.exists():
                    assert stage == 1 and block == 2 and arm == 'ex_novo'
                    assert json.loads(receipt.read_text())['exit_code'] == 0
                else:
                    base.execute(label, [sys.executable, '-u', str(PACKAGE / 'train_ppo_mlp.py'), '--config', str(path)])
                records['training'].append(audit_training(outputs[arm], target, new))
                base.write('results', records)
            total = target
            base.write('progress', {'phase': 'evaluating', 'stage': stage, 'block': block, 'iteration': total})
            cases = [(f'{arm}_s{stage}_u{total-1}_{"visible" if visible else "hidden"}',
                      outputs[arm] / f'milestone_iteration_{total:06d}/rl_module_last', visible)
                     for arm in base.ARMS for visible in (0, 1)]
            evaluations = base.evaluate_group(cases)
            records['evaluations'] += evaluations
            hidden = next(row for row in evaluations if row['label'] == f'ex_novo_s{stage}_u{total-1}_hidden')
            passed = hidden['metrics']['gate']['passed']
            decision = ('advance' if passed and stage < 4 else
                        'complete' if passed and stage == 4 and block == 2 else
                        'extend' if block == 1 else 'stop_no_recovery')
            records['decisions'].append({'stage': stage, 'block': block,
                'actor_updates_per_arm': total - 1, 'visibility': probability,
                'gate': hidden['metrics']['gate'], 'decision': decision})
            base.write('results', records)
            if passed and stage < 4:
                break
            if block == 2:
                if not passed:
                    stop = 'stopped_at_quality_gate'
                break
        previous = outputs
        if stop == 'stopped_at_quality_gate':
            break
    verify_sources()
    records.update(stop_reason=stop, finished=time.time(), actor_updates_per_arm=total - 1, inputs_unchanged=True)
    base.write('results', records)
    base.write('progress', {'phase': 'training_and_rollouts_complete',
        'stop_reason': stop, 'actor_updates_per_arm': total - 1})


if __name__ == '__main__':
    main()
