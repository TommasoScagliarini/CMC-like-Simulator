"""Read both sigma arms, keeping episode visibility and tail bias explicit."""
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
TRAIN=PACKAGE/'runs/training/MLP39_masking_sigma_verification_2026-10-01'
spec=importlib.util.spec_from_file_location('training_metrics',HERE.parent/'masking_diagnosis_2026-10-01/training/analyze_training.py')
previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)

def main():
    result={}
    for treatment in ('learned','frozen_cp90'):
        folder=TRAIN/treatment
        if not (folder/'train_iterations.jsonl').exists():continue
        iterations=previous.iteration_rows(folder)
        if not iterations:continue
        episodes=previous.episode_rows(folder)
        rows=[json.loads(line) for line in (folder/'train_iterations.jsonl').read_text().splitlines()]
        audit=[{'iteration':r['iteration'],'sigma':r['unified_phase_audit']['sigma_diagnostic']} for r in rows]
        refs={s['reference_sha256'] for r in audit for s in r['sigma']}
        assert len(refs)==1
        if treatment=='frozen_cp90':
            assert all(s['logstd_max_delta_vs_cp90_on_fixed_probes']==0 for r in audit for s in r['sigma'])
        groups={}
        for visible in (True,False):
            selected=[r for r in episodes if r['visible']==visible]
            groups['visible' if visible else 'hidden']=previous.summarize_eps(selected)
        tails={str(v):[] for v in (True,False)}
        for file in (folder/'episode_telemetry').glob('*.jsonl'):
            for line in file.read_text().splitlines():
                e=json.loads(line)
                for row in e.get('tail',[]):
                    values=row.get('policy_outputs',{}).get('action_dist_inputs')
                    if values is not None:tails[str(row['target_visible'])].append(values[2:])
        sigma={}
        for visible,values in tails.items():
            if values:
                a=np.exp(np.asarray(values))
                sigma[visible]={'tail_rows':len(a),'percentile_order':[50,90,95,99,100],
                    'sigma_knee_ankle':np.percentile(a,[50,90,95,99,100],axis=0).tolist(),
                    'warning':'Last at most 50 steps per completed episode; failure-biased, not whole-run distribution.'}
        result[treatment]={'iterations':iterations,'sigma_audit':audit,'episode_groups':groups,
            'tail_sigma':sigma,'initialization':json.loads((folder/'sigma_initialization_audit.json').read_text())}
    (HERE/'training_analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({t:{'updates':len(r['iterations']),'groups':r['episode_groups'],'sigma_audit':r['sigma_audit'][-1]} for t,r in result.items()},indent=2))

if __name__=='__main__':main()
