"""Fixed-state action-mean drift; does not predict closed-loop gait changes."""
import json,pickle
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parents[1]
source=PACKAGE/'runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29/rl_module_best/module_state.pkl'
trace=json.loads((PACKAGE/'runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic/rollout_policy_trace.json').read_text())
visible=np.asarray([r['actor_observation_vector_before'] for r in trace[::25]],np.float32)
hidden=visible.copy();hidden[:,[0,1,35,36,37,38]]=0

def forward(s,x):
 for i in (0,2):x=np.tanh(x@np.asarray(s[f'pi_encoder.{i}.weight']).T+np.asarray(s[f'pi_encoder.{i}.bias']))
 return x@np.asarray(s['pi.1.weight']).T+np.asarray(s['pi.1.bias'])
base=pickle.loads(source.read_bytes());output={}
for p in sorted((PACKAGE/'runs/training/MLP39_reward_curriculum_2026-10-01').glob('*/stage*/milestone_iteration_*/rl_module_last/module_state.pkl')):
 s=pickle.loads(p.read_bytes());record={}
 for mode,x in [('visible',visible),('hidden',hidden)]:
  a=forward(s,x);b=forward(base,x)
  delta=(a[:,:2]-b[:,:2])*np.degrees([.75,.7])
  record[mode]={'action_endpoint_mean_drift_rms_deg':np.sqrt(np.mean(delta**2,axis=0)).tolist(),
                'action_endpoint_mean_drift_max_abs_deg':np.max(np.abs(delta),axis=0).tolist(),
                'sigma_min':np.exp(a[:,2:]).min(axis=0).tolist(),'sigma_max':np.exp(a[:,2:]).max(axis=0).tolist()}
 output[str(p.relative_to(PACKAGE))]=record
(HERE/'fixed_probe_drift.json').write_text(json.dumps({'states_per_visibility':len(visible),'definition':'Mean endpoint before clipping, on fixed CP90 states; not actual generated trajectories.','checkpoints':output},indent=2)+'\n')
print(json.dumps(output,indent=2))
