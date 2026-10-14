"""Final audit of the executed paired protocol and immutable sources."""
import hashlib,json,pickle
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parents[1];ROOT=PACKAGE.parents[1]
r=json.loads((HERE/'results.json').read_text());assert r.get('finished')
assert json.loads((HERE/'initial_physical_parity.json').read_text())['passed']
assert json.loads((HERE/'smoke_verification.json').read_text())['passed']
assert json.loads((HERE/'native_resume_verification.json').read_text())['passed']
assert json.loads((HERE/'test_verification.json').read_text())['passed']
reward_parity=json.loads((HERE/'reward_parity.json').read_text())
assert reward_parity['paired_configs_differ_only_in_objective_and_output_identity']
assert reward_parity['exnovo_reward_vs_J20_preset_exact_on_historical_trace']
for name in ('source_manifest.json','execution_code_manifest.json','continuation_code_manifest.json'):
 manifest=json.loads((HERE/name).read_text())
 assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in manifest.items()),name
folders=sorted({x['folder'] for x in r['training']})
rows={a:[] for a in ('imitation','ex_novo')}
for name in folders:
 p=Path(name);arm=p.parent.name
 if (p/'resume_audit.json').exists():
  resume=json.loads((p/'resume_audit.json').read_text())
  assert not resume['actor_reinitialized']
  assert all(resume['parameter_audit'][k] for k in ('actor_exact','actor_mean_exact','critic_exact'))
  assert json.loads((p/'resume_optimizer_audit.json').read_text())['passed']
 for line in (p/'train_iterations.jsonl').read_text().splitlines():
  row=json.loads(line);i=row['iteration'];rows[arm].append(row)
  assert row['kl_update_guard']['pass']
  audit=row['unified_phase_audit'];assert audit['critic_changed']
  assert audit['target_availability_batch']['matches_expected_train_batch_size']
  if row['unified_phase']=='critic_warmup':assert audit['actor_exact']
  else:assert audit['mean_changed']
  f=p/f'milestone_iteration_{i:06d}/checkpoint_last/learner_group/learner/rl_module/default_policy/module_state.pkl'
  state=pickle.loads(f.read_bytes())
  np.testing.assert_array_equal(state['pi.1.weight'][2:],0)
  np.testing.assert_allclose(np.exp(state['pi.1.bias'][2:]),.01,rtol=1e-6)
for arm,rr in rows.items():
 rr.sort(key=lambda x:x['iteration'])
 assert [x['iteration'] for x in rr]==list(range(1,r['actor_updates_per_arm']+2))
 assert sum(x['unified_phase']=='critic_warmup' for x in rr)==1
 for before,after in zip(rr,rr[1:]):
  for parameter in ('actor','mean','critic'):
   assert before['unified_phase_audit'][parameter+'_after']==after['unified_phase_audit'][parameter+'_before']
first=[rows[a][0]['unified_phase_audit'] for a in ('imitation','ex_novo')]
for k in ('actor_before','mean_before','critic_before','target_availability_batch'):
 assert first[0][k]==first[1][k],k
for e in r['evaluations']:
 assert e['metrics']['mask_verified']
 summary=json.loads((Path(e['output'])/'rollout_summary.json').read_text());assert summary['ok']
plots=json.loads((HERE/'plots_verified.json').read_text())
assert len(plots)==len(r['evaluations'])
for p in plots:
 assert len(p['pngs'])==7
 assert all(hashlib.sha256((Path(p['folder'])/name).read_bytes()).hexdigest()==h for name,h in p['pngs'].items())
if r['stop_reason']=='stopped_at_quality_gate':assert not r['decisions'][-1]['gate']['passed']
else:assert r['decisions'][-1]['stage']==4
report={'passed':True,'stop_reason':r['stop_reason'],'actor_updates_per_arm':r['actor_updates_per_arm'],
 'critic_warmups':2,'main_sampled_steps':sum(x['unified_phase_audit']['target_availability_batch']['total_steps'] for rr in rows.values() for x in rr),
 'rollouts':len(r['evaluations']),'canonical_plots':7*len(plots),'all_checkpoint_sigma_constant':True,
 'same_initial_actor_critic_and_warmup_mask_batch':True,'inputs_and_executed_code_unchanged':True}
(HERE/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
