"""Read saved training diagnostics; no simulation or checkpoint mutation."""
from pathlib import Path
import csv
import json
import math
from statistics import mean
from collections import Counter

OUT = Path(__file__).resolve().parent
PKG = OUT.parents[2]
TRAIN = PKG / 'runs' / 'training'
MASK = TRAIN / 'MLP39_LEFT_best90_masking_100iter_2026-09-30'
BASE = TRAIN / 'MLP39_LEFT_fresh_best80config_100iter_2026-09-29'
JOINTS = ('pros_knee_angle', 'pros_ankle_angle')

def avg(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return mean(xs) if xs else None

def iteration_rows(path):
    result = []
    for line in (path / 'train_iterations.jsonl').read_text().splitlines():
        r = json.loads(line)
        m = r.get('learner_metrics', {})
        b = r.get('unified_phase_audit', {}).get('target_availability_batch', {})
        result.append(dict(stage=path.name, iteration=r['iteration'],
            return_mean=r['episode_return_mean'], length_mean=r['episode_len_mean'],
            entropy=r['entropy'], kl=r['mean_kl_loss'], kl_coeff=r['current_kl_coeff'],
            policy_loss=r['policy_loss'], vf_loss=r['vf_loss'],
            vf_loss_unclipped=m.get('learners/default_policy/vf_loss_unclipped'),
            explained_variance=m.get('learners/default_policy/vf_explained_var'),
            visible_steps=b.get('visible_steps'), total_steps=b.get('total_steps'),
            visible_fraction=b.get('visible_fraction')))
    return result

def episode_rows(path):
    result = []
    for f in sorted((path / 'episode_telemetry').glob('*.jsonl')):
        for line in f.open():
            e = json.loads(line)
            tails = e.get('tail', [])
            versions = e.get('policy_versions', {})
            visible = set(t.get('target_visible') for t in tails)
            assert len(visible) == 1, (f, visible)
            r = dict(stage=path.name, episode_uid=e['episode_uid'],
                visible=visible.pop(), version_first=versions.get('first'),
                version_last=versions.get('last'), version_switches=versions.get('switch_count'),
                length=e['episode_steps'], episode_return=e['episode_return'],
                end_reason=e['end_reason'], peak_penetration_mm=e['peaks']['penetration_m']['value']*1000,
                partial_episode=e['partial_episode'], missing_steps=e['missing_steps'],
                tail_rows=len(tails))
            for j in JOINTS:
                a=e['joint_error_accumulators'][j]
                r[j+'_n']=a['count']
                r[j+'_ss']=a['sum_squared_rad2']
            for kind in ('reward_components', 'reward_terms'):
                keys=set(k for t in tails for k in t.get(kind,{}))
                for k in sorted(keys):
                    r['tail_'+k]=avg(t.get(kind,{}).get(k) for t in tails)
            for ji,j in enumerate(JOINTS):
                vals=[t['policy_outputs']['action_dist_inputs'] for t in tails
                      if t.get('policy_outputs',{}).get('action_dist_inputs') is not None]
                r['tail_'+j+'_sigma']=avg(math.exp(v[ji+2]) for v in vals)
            result.append(r)
    return result

def summarize_eps(es):
    out=dict(episodes=len(es), steps=sum(r['length'] for r in es),
        return_mean=avg(r['episode_return'] for r in es),
        length_mean=avg(r['length'] for r in es),
        reward_per_step=(sum(r['episode_return'] for r in es)/sum(r['length'] for r in es)) if es else None,
        full_500_fraction=avg(float(r['length']==500) for r in es),
        time_limit_fraction=avg(float(r['end_reason']=='episode_time_limit') for r in es),
        end_reasons=dict(Counter(r['end_reason'] for r in es)),
        peak_penetration_mm_mean=avg(r['peak_penetration_mm'] for r in es),
        policy_version_switch_fraction=avg(float(r['version_switches']>0) for r in es))
    for j in JOINTS:
        n=sum(r[j+'_n'] for r in es)
        out[j+'_rmse_deg']=math.degrees(math.sqrt(sum(r[j+'_ss'] for r in es)/n)) if n else None
    for k in sorted(set(k for r in es for k in r if k.startswith('tail_'))-{'tail_rows'}):
        pairs=[(r[k],r['tail_rows']) for r in es if r.get(k) is not None]
        out[k]=sum(x*n for x,n in pairs)/sum(n for x,n in pairs) if pairs else None
    return out

def csv_write(name, rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)

def main():
    its=[];eps=[];summary={}
    for d in [BASE]+sorted(MASK.glob('stage*')):
        ir=iteration_rows(d)
        er=episode_rows(d)
        if d==BASE:
            ir=[r for r in ir if 81<=r['iteration']<=90]
            er=[r for r in er if 80<=r['version_first'] and r['version_last']<=89]
        its.extend(ir);eps.extend(er)
        stage=dict(iterations=[r['iteration'] for r in ir],
            learner_mean={k:avg(r.get(k) for r in ir) for k in ir[0] if k not in ('stage','iteration')},
            learner_first=ir[0],learner_last=ir[-1],
            learner_max_kl=max(r['kl'] for r in ir),
            groups={})
        for tag,selected in [('all',er),('visible',[r for r in er if r['visible']]),
                             ('hidden',[r for r in er if not r['visible']])]:
            stage['groups'][tag]=summarize_eps(selected)
        start=ir[0]['iteration']-1
        end=ir[-1]['iteration']-1
        for tag,low,high in [('first5',start,start+4),('last5',end-4,end)]:
            for vis in (True,False):
                chosen=[r for r in er if low<=r['version_last']<=high and r['visible']==vis]
                stage['groups'][tag+('_visible' if vis else '_hidden')]=summarize_eps(chosen)
        summary[d.name]=stage
        print(d.name, 'iterations',len(ir),'episodes',len(er),flush=True)
    csv_write('iterations.csv',its);csv_write('episodes.csv',eps)
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (OUT/'README.md').write_text('''# Saved training diagnosis\n\nGenerated by `analyze_training.py`, using only existing JSONL files. No simulation, training or checkpoint edits.\n\nWhole-episode joint RMSE uses `joint_error_accumulators`: physical joint angle minus imitation target, summed over completed segments. Visibility is inferred from recorded `target_visible` in the final 50 rows; the experiment fixes the mask per episode. Episode metrics are exact over stored completed episodes; unfinished episodes at each process boundary have no episode record. Policy versions often change within episodes, so stage aggregates are more robust than attributing results to one checkpoint. Baseline is the last ten sampling policies preceding CP90 updates (first version >=80 and last <=89).\n\n`tail_*` metrics cover only the final at most 50 steps of each episode, not whole episodes, and are weighted by recorded tail length. These tails are biased toward time-limit endings or failure events and must not be compared as unbiased whole-run reward decompositions. Training return/length from iteration logs may be RLlib rolling averages. Stage completion fractions use telemetry, not rolling metrics.\n''')

if __name__=='__main__':
    main()
