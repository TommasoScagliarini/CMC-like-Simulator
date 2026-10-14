"""Offline audit only: saved paired rollouts; no policy forwards or simulation."""
from pathlib import Path
import json
import math
import statistics

BASE=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
JOINTS=['pros_knee_angle','pros_ankle_angle']
SOURCES={40: BASE/'validation/completion_diagnostics_2026-09-22/noise_analysis.json',
 **{i:BASE/f'runs/plans/completion_recovery42_2026-09-22/analysis_iter{i:03d}/noise_analysis.json' for i in (60,80,100)}}
def avg(v): return sum(v)/len(v)
def pct(v,p):
 s=sorted(v); k=(len(s)-1)*p; lo=int(k); hi=math.ceil(k); return s[lo]+(s[hi]-s[lo])*(k-lo)
def stats(v): return {'mean':avg(v),'median':pct(v,.5),'p95':pct(v,.95),'max':max(v)}
def cdf(x): return .5*(1+math.erf(x/math.sqrt(2)))
def phi(x): return math.exp(-x*x/2)/math.sqrt(2*math.pi)
def expected_clip(m,s):
 if s==0: return max(-1,min(1,m))
 a=(-1-m)/s;b=(1-m)/s
 return -cdf(a)+m*(cdf(b)-cdf(a))+s*(phi(a)-phi(b))+1-cdf(b)
def rmse(rows,joint):
 return math.sqrt(avg([(r['prosthetic_state'][joint]-r['imitation_target_q'][joint])**2 for r in rows]))*180/math.pi

result={'scope':'Read-only offline computation on saved paired rollouts. No new simulation.','checkpoints':{},
 'warnings':['Five repeated noise seeds and one fixed environment start; not independent training replicas or an equivalence test.',
 'Learned std is state dependent. Statistics on different executed trajectories include state-visitation differences.',
 'Noise scaling changes the expectation of the clipped action as well as its variance.',
 'Common-prefix RMSE ends at176 steps for the final comparison; it excludes late phases and does not summarize whole-episode gait quality.',
 'Aggregate full-episode RMSE for failing and complete trajectories has survival/time-window confounding.'],
 'normalized_action_to_angle_slope_rad':[.75,.7]}
all_eps={}
for iteration,source in SOURCES.items():
 d=json.loads(source.read_text()); eps=[]
 for e in d['episodes']:
  folder=Path(e['output']); rows=json.loads((folder/'rollout_policy_trace.json').read_text()); draws=json.loads((folder/'paired_noise_draws.json').read_text())
  assert len(rows)==len(draws)==e['steps']
  for r in rows:
   if r['policy_action_mean'] is None:
    assert e['noise_scale']==0
    r['policy_action_mean']=r['raw_policy_action']
  means=[[r['policy_action_mean'][j] for r in rows] for j in range(2)]
  stds=[[r['learned_std'][j] for r in draws] for j in range(2)]
  clipping=[[abs(r['raw_policy_action'][j])>1 for r in rows] for j in range(2)]
  row={k:e[k] for k in ['noise_scale','noise_seed','steps','end_reason','completed','episode_return','grf_penetration_max_m','phase_valid_cycle_count','rmse_degrees','common_prefix_rmse_degrees']}
  row['clipped_component_fraction']=avg([v for values in clipping for v in values])
  assert abs(row['clipped_component_fraction']-e['action_clipped_fraction'])<1e-12
  row['global_176_step_rmse_degrees']={j:rmse(rows[:176],j) for j in JOINTS}
  row['joint_action_diagnostics']={}
  for j,joint in enumerate(JOINTS):
   delta=[expected_clip(m,s*.5)-expected_clip(m,s) for m,s in zip(means[j],stds[j])]
   muclip=[max(-1,min(1,m)) for m in means[j]]
   learned_expected=[expected_clip(m,s) for m,s in zip(means[j],stds[j])]
   row['joint_action_diagnostics'][joint]={
    'learned_std_normalized':stats(stds[j]),'mu_outside_bounds_fraction':avg([abs(m)>1 for m in means[j]]),
    'sampled_clipped_fraction':avg(clipping[j]),'mean_min':min(means[j]),'mean_max':max(means[j]),
    'expected_clip_half_minus_full_sigma_signed_mean':avg(delta),
    'expected_clip_half_minus_full_sigma_abs':stats([abs(x) for x in delta]),
    'expected_clip_half_minus_full_sigma_mean_abs_decoded_degrees':avg([abs(x) for x in delta])*(.75 if j==0 else .7)*180/math.pi,
    'expected_clip_full_minus_deterministic_mean_abs':avg([abs(a-b) for a,b in zip(learned_expected,muclip)])}
  eps.append(row);all_eps[(iteration,e['noise_scale'],e['noise_seed'])]=(rows,draws)
 groups=[]
 for scale in [0,.5,1]:
  selected=[e for e in eps if e['noise_scale']==scale]
  totalsteps=sum(e['steps'] for e in selected)
  group={'noise_scale':scale,'n':len(selected),'complete':sum(e['completed'] for e in selected),
   'mean_return':avg([e['episode_return'] for e in selected]),'mean_steps':avg([e['steps'] for e in selected]),
   'clipped_fraction_step_weighted':sum(e['steps']*e['clipped_component_fraction'] for e in selected)/totalsteps,
   'mean_full_episode_rmse_degrees':{j:avg([e['rmse_degrees'][j] for e in selected]) for j in JOINTS},
   'mean_within_checkpoint_common_prefix_rmse_degrees':{j:avg([e['common_prefix_rmse_degrees'][j] for e in selected]) for j in JOINTS},
   'mean_176_step_rmse_degrees':{j:avg([e['global_176_step_rmse_degrees'][j] for e in selected]) for j in JOINTS},
   'action_diagnostics_pooled':{}}
  for j,joint in enumerate(JOINTS):
   allmeans=[];allstd=[];allclip=[]
   for ep in selected:
    rows,draws=all_eps[(iteration,scale,ep['noise_seed'])]
    allmeans.extend(r['policy_action_mean'][j] for r in rows)
    allstd.extend(r['learned_std'][j] for r in draws)
    allclip.extend(abs(r['raw_policy_action'][j])>1 for r in rows)
   deltas=[expected_clip(m,s*.5)-expected_clip(m,s) for m,s in zip(allmeans,allstd)]
   deterministic_bias=[expected_clip(m,s)-max(-1,min(1,m)) for m,s in zip(allmeans,allstd)]
   group['action_diagnostics_pooled'][joint]={
    'learned_std_normalized':stats(allstd),
    'mean_outside_bounds_fraction':avg([abs(m)>1 for m in allmeans]),
    'sampled_clipped_fraction':avg(allclip),
    'conditional_gaussian_clip_probability':avg([cdf((-1-m)/s)+1-cdf((1-m)/s) for m,s in zip(allmeans,allstd)]),
    'expected_clip_full_vs_deterministic_endpoint_rms_deg':math.sqrt(avg([x*x for x in deterministic_bias]))*(.75 if j==0 else .7)*180/math.pi,
    'expected_clip_half_minus_full_sigma_signed_mean':avg(deltas),
    'expected_clip_half_minus_full_sigma_abs':stats([abs(x) for x in deltas]),
    'expected_clip_half_minus_full_sigma_mean_abs_decoded_degrees':avg([abs(x) for x in deltas])*(.75 if j==0 else .7)*180/math.pi}
  groups.append(group)
 paired=[]
 for seed in range(123,128):
  a=next(e for e in eps if e['noise_scale']==1 and e['noise_seed']==seed)
  b=next(e for e in eps if e['noise_scale']==.5 and e['noise_seed']==seed)
  ra,da=all_eps[(iteration,1,seed)];rb,db=all_eps[(iteration,.5,seed)];n=min(len(ra),len(rb))
  assert ra[0]['actor_observation_vector_before']==rb[0]['actor_observation_vector_before']
  assert all(x['standard_normal']==y['standard_normal'] for x,y in zip(da,db))
  paired.append({'seed':seed,'matched_noise_draws':n,'full_sigma_steps':a['steps'],'half_sigma_steps':b['steps'],
   'return_half_minus_full':b['episode_return']-a['episode_return'],
   'matched_pair_prefix_steps':n,
   'matched_pair_prefix_rmse_full':{j:rmse(ra[:n],j) for j in JOINTS},
   'matched_pair_prefix_rmse_half':{j:rmse(rb[:n],j) for j in JOINTS},
   'rmse176_half_minus_full':{j:b['global_176_step_rmse_degrees'][j]-a['global_176_step_rmse_degrees'][j] for j in JOINTS}})
 result['checkpoints'][str(iteration)]={'source':str(source),'within_checkpoint_common_prefix_steps':d['common_prefix_steps'],
  'groups':groups,'episodes':eps,'paired':paired}
initial_reference=all_eps[(40,1,123)][0][0]['actor_observation_vector_before']
result['same_initial_observation_comparison']=[]
for iteration in [40,60,80,100]:
 rows,draws=all_eps[(iteration,1,123)]
 assert rows[0]['actor_observation_vector_before']==initial_reference
 result['same_initial_observation_comparison'].append({'iteration':iteration,
  'exact_same_39D_observation':True,'mean':rows[0]['policy_action_mean'],
  'learned_std':draws[0]['learned_std']})
v26_source=BASE/'validation/ablation_report_2026-09-20/metrics.json'
v26=json.loads(v26_source.read_text())['historical']['V26_aug20']['policy_audit']
result['historical_v26_existing_audit']={'source':str(v26_source),'audit':v26,
 'interpretation':'Broad Gaussian exploration and distortion by clipping were present in successful V26 too; not a new mechanism unique to unified39. Values conditional on each policy own deterministic states, not matched observations.'}
(OUT/'noise_audit.json').write_text(json.dumps(result,indent=2)+'\n')
lines=['# Audit offline finale del rumore — 2026-09-22','',
 'Analisi delle tracce salvate: nessun rollout aggiuntivo, nessuna modifica al training.','',
 '| Checkpoint | Deterministico return | Completamenti sigma1 | Completamenti sigma0.5 | Return medio sigma1 | Return medio sigma0.5 |',
 '|---|---:|---:|---:|---:|---:|']
for i,ck in result['checkpoints'].items():
 g={e['noise_scale']:e for e in ck['groups']}
 lines.append(f"| {i} | {g[0]['mean_return']:.2f} | {g[1]['complete']}/5 | {g[.5]['complete']}/5 | {g[1]['mean_return']:.2f} | {g[.5]['mean_return']:.2f} |")
lines += ['', 'Alla100 i fallimenti sigma1 sono seed126 (297passi) e127 (176passi), entrambi per penetrazione; i corrispondenti sigma0.5 completano500passi. Nel test80 i tre fallimenti sigma1 erano tardivi (467–490passi).',
 '', 'Sul prefisso comune di176passi, tutte e cinque le coppie alla100 hanno RMSE minore con sigma0.5, sia al ginocchio sia alla caviglia. Questo evita il confronto improprio fra RMSE di episodi brevi e completi, ma riguarda soltanto i primi1.76s.',
 '', '## Distribuzione e clipping alla100', '',
 '| Scala rumore | Componenti tagliate | RMSE ginocchio176passi | RMSE caviglia176passi |',
 '|---|---:|---:|---:|']
for g in result['checkpoints']['100']['groups']:
 lines.append(f"| {g['noise_scale']} | {100*g['clipped_fraction_step_weighted']:.2f}% | {g['mean_176_step_rmse_degrees'][JOINTS[0]]:.3f}° | {g['mean_176_step_rmse_degrees'][JOINTS[1]]:.3f}° |")
lines+=['','Alla100, sulle sole traiettorie sigma1 (statistiche pesate per passi):']
g=next(g for g in result['checkpoints']['100']['groups'] if g['noise_scale']==1)
for joint,v in g['action_diagnostics_pooled'].items():
 lines.append(f"- {joint}: std appresa mediana {v['learned_std_normalized']['median']:.3f},95°percentile {v['learned_std_normalized']['p95']:.3f}; media gaussiana fuori[-1,1] nel{100*v['mean_outside_bounds_fraction']:.1f}% dei passi. Su questi stessi stati, dimezzare sigma cambia la media dell'azione dopo clipping di {v['expected_clip_half_minus_full_sigma_abs']['mean']:.3f} unità normalizzate in valore assoluto medio (equivalente a {v['expected_clip_half_minus_full_sigma_mean_abs_decoded_degrees']:.2f}° dell'endpoint grezzo; il filtro successivo ne modifica l'effetto fisico).")
lines+=['','## Interpretazione e test discriminante','',
 'Il dimezzamento del rumore è un fattore promettente: alla100 migliora il tracking sul prefisso condiviso e salva i due fallimenti osservati. Non identifica tuttavia la sola varianza come causa, perché il clipping sposta anche la media applicata; inoltre5semi, unico stato iniziale e una sola replica di training non consentono conclusioni di robustezza generale.',
 '', 'Prima di modificare il training: rollout deterministici che applichino E[clip(mu+sigma*z)] ed E[clip(mu+0.5*sigma*z)] alle osservazioni via via raggiunte, confrontati con clip(mu). I tre casi separano la distorsione della media prodotta dal clipping dalla variabilità casuale. Poi valutazione accoppiata su più seed per confermare il vantaggio della scala0.5; scegliere anche checkpoint60 e best72, perché last100 non è automaticamente il migliore.',
 '', 'La quantità analitica sopra è calcolata sulle stesse osservazioni salvate e prova la modifica del comando medio, non l’esito di simulazioni controfattuali. Non attribuire tutto il problema al rumore: anche la policy deterministica100 mantiene errore caviglia4.915°, un solo ciclo FSM valido, e non uguaglia V26.',
 '', '## Limiti', '']+['- '+v for v in result['warnings']]
lines += ['', '## Stessa osservazione iniziale: confronto pulito fra checkpoint', '',
 'Le 39 componenti dell’osservazione iniziale risultano esattamente uguali nei quattro checkpoint. In questo singolo stato la deviazione standard aumenta, soprattutto alla caviglia, e la media della caviglia supera il limite +1 già a 60. È una modifica della distribuzione appresa, senza il confondente delle diverse traiettorie successive.', '',
 '| Iterazione | Media ginocchio | Media caviglia | Std ginocchio | Std caviglia |',
 '|---|---:|---:|---:|---:|']
for r in result['same_initial_observation_comparison']:
 lines.append(f"| {r['iteration']} | {r['mean'][0]:.3f} | {r['mean'][1]:.3f} | {r['learned_std'][0]:.3f} | {r['learned_std'][1]:.3f} |")
lines += ['', 'Questo singolo stato non rappresenta da solo la distribuzione dei passi né dimostra la causa dei fallimenti.', '',
 '## Controllo storico V26', '',
 'L’audit salvato il 20 settembre, già eseguito sulle osservazioni deterministiche del V26 best87, riporta:', '',
 '- Std appresa media ginocchio / caviglia: 1,026 / 0,750.',
 '- Clipping deterministico per componente: 10,2% / 0,4%.',
 '- Probabilità gaussiana condizionata di superare i limiti: 36,3% / 22,7%.',
 '- RMS fra endpoint atteso dopo clipping e endpoint deterministico: 7,04° / 5,42°.', '',
 'Quindi anche il V26 riuscito aveva una distribuzione ampia e una differenza fra comando deterministico e comando medio esplorativo. Questo meccanismo non basta da solo a spiegare il divario attuale. Le statistiche storiche sono calcolate sugli stati del V26, non sulle stesse osservazioni della unified39; non sono frequenze misurate in nuovi rollout stocastici.', '',
 'Fonte: `validation/ablation_report_2026-09-20/metrics.json`, `historical.V26_aug20.policy_audit`. Nessuna nuova forward o simulazione è stata eseguita.']
md='\n'.join(lines)+'\n'
for a,b in [('Alla100','Alla 100'),('sigma1','sigma 1'),('sigma0.5','sigma 0.5'),('seed126','seed 126'),('seed127','seed 127'),('297passi','297 passi'),('176passi','176 passi'),('500passi','500 passi'),('467–490passi','467–490 passi'),('test80','test 80'),('cinque le coppie alla100','cinque le coppie alla 100'),('176step','176 step'),('1.76s','1,76 s'),('0.5;','0.5;'),('5semi','5 semi'),('checkpoint60','checkpoint 60'),('best72','best 72'),('last100','last 100'),('caviglia4.915°','caviglia 4,915°'),('nel{','nel {'),('] nel', '] nel ')]:
 md=md.replace(a,b)
import re
md=re.sub(r'(?<=\d),(?=95°)', ', ', md)
md=md.replace('fuori[-1,1]', 'fuori [-1, 1]').replace(' nel24', ' nel 24').replace(' nel23', ' nel 23').replace('95°percentile','95° percentile').replace('scala0.5','scala 0.5')
for a,b in [(' e127', ' e 127'), ('completano500', 'completano 500'), ('di176', 'di 176'), ('primi1,76', 'primi 1,76'), ('alla100', 'alla 100'), ('ginocchio176', 'ginocchio 176'), ('caviglia176', 'caviglia 176'), ('inoltre5', 'inoltre 5'), ('deterministica100', 'deterministica 100'), ('at176', 'at 176'), ('V26 best87', 'V26 best 87'), ('(statistiche pesate per passi):\n-', '(statistiche pesate per passi):\n\n-')]:
 md=md.replace(a,b)
(OUT/'noise_audit.md').write_text(md)
print(json.dumps({'output':str(OUT/'noise_audit.json'),'checkpoint100_groups':result['checkpoints']['100']['groups'],'checkpoint100_paired':result['checkpoints']['100']['paired']},indent=2))
