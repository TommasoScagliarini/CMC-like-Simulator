"""Inspect original Gaussian outputs and sampled actions in saved episode tails."""
from pathlib import Path
import json
import numpy as np

OUT = Path(__file__).resolve().parent
TRAIN = OUT.parents[2] / 'runs' / 'training'
BASE = TRAIN / 'MLP39_LEFT_fresh_best80config_100iter_2026-09-29'
PATHS = [BASE] + sorted((TRAIN / 'MLP39_LEFT_best90_masking_100iter_2026-09-30').glob('stage*'))

def summarize(rows):
    if not rows:
        return {'tail_rows': 0}
    sigma = np.array([r['sigma'] for r in rows])
    raw = np.array([r['raw_action'] for r in rows])
    delta = np.array([r['logp_reconstruction_error'] for r in rows])
    worst = max(rows, key=lambda r: max(r['sigma']))
    return dict(tail_rows=len(rows),
        quantiles_sigma={str(q): np.quantile(sigma,q,axis=0).tolist() for q in [0,.1,.5,.9,.95,.99,.999,1]},
        fraction_sigma_gt5=(sigma>5).mean(0).tolist(),
        fraction_sigma_gt10=(sigma>10).mean(0).tolist(),
        fraction_raw_action_abs_gt1=(np.abs(raw)>1).mean(0).tolist(),
        fraction_raw_action_abs_gt10=(np.abs(raw)>10).mean(0).tolist(),
        max_logp_reconstruction_error=float(delta.max()),
        mean_logp_reconstruction_error=float(delta.mean()),
        worst=worst)

summary={}
for path in PATHS:
    rows=[]
    for f in (path/'episode_telemetry').glob('*.jsonl'):
        for line in f.open():
            episode=json.loads(line)
            if path==BASE and not (episode['policy_versions']['first']>=80 and episode['policy_versions']['last']<=89):
                continue
            for t in episode['tail']:
                dist=t.get('policy_outputs',{}).get('action_dist_inputs')
                if dist is None:
                    continue
                mu=np.array(dist[:2]); logstd=np.array(dist[2:]); sigma=np.exp(logstd)
                raw=np.array(t['raw_policy_action'])
                logp=float((-0.5*((raw-mu)/sigma)**2-logstd-0.5*np.log(2*np.pi)).sum())
                recorded=t['policy_outputs']['action_logp']
                rows.append(dict(sigma=sigma.tolist(), logits=dist, raw_action=raw.tolist(),
                    commanded_endpoint=t['policy_segment_values'][-1],
                    logp_recorded=recorded, logp_reconstructed=logp,
                    logp_reconstruction_error=abs(logp-recorded),
                    episode_uid=episode['episode_uid'], episode_steps=episode['episode_steps'],
                    episode_return=episode['episode_return'],
                    version=t['policy_outputs']['weights_seq_no'],
                    reason=episode['end_reason'], visible=t['target_visible'],
                    step=t['episode_step'], time_s=t['time_s']))
    groups={'all':rows,
        'time_limit':[r for r in rows if r['reason']=='episode_time_limit'],
        'failure':[r for r in rows if r['reason']!='episode_time_limit'],
        'visible':[r for r in rows if r['visible']],
        'hidden':[r for r in rows if not r['visible']]}
    summary[path.name]={name:summarize(group) for name,group in groups.items()}
    print(path.name, summary[path.name]['all']['quantiles_sigma']['0.99'],
          'logp max error',summary[path.name]['all']['max_logp_reconstruction_error'])
(OUT/'tail_sigma_distribution.json').write_text(json.dumps(summary,indent=2)+'\n')
