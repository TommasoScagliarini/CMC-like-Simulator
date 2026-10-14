"""Fail-closed audit of the completed experiments and derived artifacts."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
TRAIN=PACKAGE/'runs/training/MLP39_masking_sigma_verification_2026-10-01'

def read(path):return json.loads(path.read_text())

def main():
    comparison=read(HERE/'comparison_execution.json')
    assert comparison['inputs_unchanged']
    matrix=read(HERE/'matrix_execution.json')
    assert len(matrix)==6 and all(r['mask_verified'] and r['inputs_unchanged'] for r in matrix)
    training=read(HERE/'training_summary.json')
    assert len(training['control_reproduction'])==10
    assert all(r['original_state_exact'] and r['full_learner_state_exact'] and r['critic_exact'] and r['metrics_exact'] and r['mask_batch_exact'] for r in training['control_reproduction'])
    assert all(training['initial_comparison'].values())
    updates={}
    for treatment in ('learned','frozen_cp90'):
        rows=[json.loads(line) for line in (TRAIN/treatment/'train_iterations.jsonl').read_text().splitlines()]
        assert [r['iteration'] for r in rows]==list(range(91,101))
        assert all(r['unified_phase_audit']['mean_changed'] and r['unified_phase_audit']['critic_changed'] for r in rows)
        assert sum(r['unified_phase_audit']['target_availability_batch']['total_steps'] for r in rows)==40960
        summary=read(TRAIN/treatment/'summary.json')
        assert summary['ok'] and summary['iterations_run']==10 and not summary['skipped_iterations']
        updates[treatment]={'updates':10,'sampled_steps':40960,'mean_and_critic_changed_every_update':True,
            'sigma_logstd_delta_per_update':[r['unified_phase_audit']['sigma_diagnostic'][0]['logstd_max_delta_vs_cp90_on_fixed_probes'] for r in rows]}
        if treatment=='frozen_cp90':assert all(v==0 for v in updates[treatment]['sigma_logstd_delta_per_update'])
    assert len(comparison['evaluations'])==8
    distributions=read(HERE/'rollout_distributions.json')['rollouts']
    for case in comparison['evaluations']:
        trace=read(Path(case['rollout'])/'rollout_policy_trace.json')
        visible=case['label'].endswith('_visible')
        assert len(trace)==case['steps']
        for row in trace:
            assert bool(row['target_availability']['visible'])==visible
            obs=row['actor_observation_vector_before']
            if visible:assert abs(obs[0]**2+obs[1]**2-1)<1e-5
            else:assert all(obs[i]==0 for i in (0,1,35,36,37,38))
        if case['label'].startswith('frozen_cp90_'):
            assert distributions[case['label']]['sigma_exact_cp90_on_every_recorded_state']
    plots=read(HERE/'plot_verification.json')
    expected={r['label'] for r in matrix}|{r['label'] for r in comparison['evaluations']}
    analysis=read(HERE/'analysis.json')
    assert expected.issubset(analysis['rollouts'])
    for iteration in (95,100):
        for visibility in ('visible','hidden'):
            pair=analysis['paired_comparisons'][f'cp{iteration}_{visibility}_sigma']
            assert pair and len(pair['rollouts'])==3
    assert {r['label'] for r in plots}==expected and len(expected)==14
    for record in plots:
        assert len(record['png_sha256'])==7
        folder=ROOT/'plot'/f"10_01_2026_masking_verification_{record['label']}"
        assert all(hashlib.sha256((folder/f).read_bytes()).hexdigest()==h for f,h in record['png_sha256'].items())
    manifests=('training_manifest.json','evaluation_manifest.json','analysis_input_hashes.json','distribution_input_hashes.json','training_analysis_hashes.json')
    checked=0
    for manifest in manifests:
        for relative,digest in read(HERE/manifest).items():
            assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest,relative
            checked+=1
    executions=[read(HERE/f"{r['label']}_execution.json") for r in comparison['evaluations']]
    elapsed=lambda records:max(r['started']+r['elapsed_s'] for r in records)-min(r['started'] for r in records)
    times={'matrix_wall_s':elapsed(matrix),'training_wall_s':{},'evaluation_wall_s':elapsed(executions)}
    for treatment in updates:times['training_wall_s'][treatment]=read(HERE/f'train_{treatment}_execution.json')['elapsed_s']
    times['first_matrix_to_last_evaluation_wall_s']=max(r['started']+r['elapsed_s'] for r in executions)-min(r['started'] for r in matrix)
    result={'passed':True,'training':updates,'new_rollouts':14,'canonical_pngs':98,'manifest_entries_verified':checked,
        'visibility_verified_on_every_eval_step':True,'frozen_sigma_verified_on_every_eval_state':True,
        'control_reproduces_all_ten_original_checkpoints':True,'timing':times,
        'limitations':['Optional training episode telemetry was not enabled.','Single training seed and single deterministic initial condition.','Native verification on macOS only.']}
    (HERE/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
