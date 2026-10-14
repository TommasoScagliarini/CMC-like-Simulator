from pathlib import Path
import os,sys,json,math
os.environ.setdefault('MPLCONFIGDIR','/tmp/cmc_like_matplotlib')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
REPO=OUT.parents[3]
TG=REPO/'Trajectory Generator'
sys.path.insert(0,str(TG))
from unified_MLP_39 import reward_function as newreward
from baseline_MLP import reward_function as oldreward
CASES={
'V26':TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
'39D':TG/'unified_MLP_39/runs/rollout/MLP39_left_phase_best_iter54_center_2026-09-18'}
JOINTS=['pros_knee_angle','pros_ankle_angle']
data={};report={}
for name,path in CASES.items():
 s=json.loads((path/'rollout_summary.json').read_text());t=json.loads((path/'rollout_policy_trace.json').read_text())
 mod=newreward if name=='39D' else oldreward
 cfg=mod.RewardConfig.from_mapping(s['reward_config'])
 scores=[mod.compute_reward(x['reward_terms'],cfg,reference=x['policy_segment_values']) for x in t]
 sums={k:sum(float(comp[k]) for _,comp in scores) for k in scores[0][1] if k.endswith('_term') or k in ['reward','reward_base','penalty','imitation_score','served_imitation_score']}
 assert abs(sum(v for v,_ in scores)-s['episode_return'])<1e-6,(name,sum(v for v,_ in scores),s['episode_return'])
 summary={'source':str(path),'checkpoint':s['checkpoint'],'steps':s['steps'],'return':s['episode_return'],'reward_recomputed_exact':True,'reward_component_sums':sums,'joints':{}}
 for j,joint in enumerate(JOINTS):
  target=np.array([x['imitation_target_q'][joint] for x in t]);served=np.array([x['prosthetic_state'][joint+'_served_ref'] for x in t]);actual=np.array([x['prosthetic_state'][joint] for x in t]);raw=np.array([x['raw_policy_action'][j] for x in t]);applied=np.array([x['applied_policy_action'][j] for x in t]);decoded=(-1.5+.75*(applied+1)) if j==0 else .7*applied
  qrange=t[0]['reward_terms'][joint+'_imitation_position_range_rad'];vrange=t[0]['reward_terms'][joint+'_imitation_velocity_range_rad_s']
  summary['joints'][joint]={'rmse_actual_target_deg':float(np.rad2deg(np.sqrt(np.mean((actual-target)**2)))),'rmse_served_target_deg':float(np.rad2deg(np.sqrt(np.mean((served-target)**2)))),'rmse_actual_served_deg':float(np.rad2deg(np.sqrt(np.mean((actual-served)**2)))),'correlation_actual_target':float(np.corrcoef(target,actual)[0,1]),'q_range_for_loss_rad':qrange,'qdot_range_for_loss_rad_s':vrange,'ranges_deg':{k:list(map(float,np.rad2deg([min(v),max(v)]))) for k,v in [('target',target),('served',served),('actual',actual),('raw_command_decoded',decoded)]}}
 weights = {
 'effort_weight':'effort_loss','smoothness_weight':'smoothness_loss','saturation_weight':'saturation_loss',
 'command_rate_weight':'command_rate_loss','sea_saturation_weight':'sea_saturation_loss',
 'sea_torque_error_weight':'sea_torque_error_loss','sea_motor_speed_weight':'sea_motor_speed_loss',
 'sea_motor_accel_weight':'sea_motor_accel_loss','sea_motor_power_weight':'sea_motor_power_loss',
 'policy_action_clip_weight':'policy_action_clip_loss','phase_invalid_event_weight':'invalid_event_loss',
 'phase_contact_validity_weight':'contact_validity_loss'}
 summary['penalty_components_sums']={key:float(getattr(cfg,key)*sum(x['reward_terms'].get(term,0.0) for x in t)) for key,term in weights.items()}
 summary['imitation_loss_means']={key:float(np.mean([x['reward_terms'][key] for x in t])) for key in ['sound_imitation_loss','served_imitation_loss']}
 phase=np.array([x['reward_terms']['pros_knee_angle_imitation_target_phase'] for x in t]);rew=np.array([v for v,_ in scores])
 summary['reward_by_target_phase']={key:{'samples':int(mask.sum()),'mean':float(rew[mask].mean()),'sum':float(rew[mask].sum())} for key,mask in [('0_to_60pct',phase<.6),('60_to_100pct',phase>=.6)]}
 data[name]=(s,t,scores);report[name]=summary
# Rescore same recorded 39D trajectory with historical penalty coefficients:
# this changes neither target nor loss normalization nor physical trajectory.
s,t,scores=data['39D'];histcfg=oldreward.RewardConfig.from_mapping(data['V26'][0]['reward_config'])
rescored=[oldreward.compute_reward(x['reward_terms'],histcfg,reference=x['policy_segment_values'])[0] for x in t]
report['39D']['same_trajectory_historical_reward_coefficients_return']=sum(rescored)
report['39D']['rescore_caveat']='Offline reward-only rescoring of unchanged 39D state/action trace, using left-target losses. Not a retraining or V26-target replay.'
# Compare native deterministic traces against their own target, visibly separate.
fig,axes=plt.subplots(3,2,figsize=(14,11),sharex='col')
for c,(name,(s,t,scores)) in enumerate(data.items()):
 times=np.array([x['time'] for x in t])-t[0]['policy_segment_times'][0]
 for j,joint in enumerate(JOINTS):
  ax=axes[j,c]
  target=np.rad2deg([x['imitation_target_q'][joint] for x in t]);served=np.rad2deg([x['prosthetic_state'][joint+'_served_ref'] for x in t]);actual=np.rad2deg([x['prosthetic_state'][joint] for x in t])
  sign=-1 if j==0 else 1
  ax.plot(times,sign*target,'k--',lw=1.6,label='Target proprio')
  ax.plot(times,sign*served,color='#d57718',lw=1.3,label='Reference servita')
  ax.plot(times,sign*actual,color='#236aa4',lw=1.3,label='Giunto simulato')
  ax.set_ylabel(('Flessione ginocchio' if j==0 else 'Angolo caviglia')+' [°]');ax.grid(alpha=.2);ax.legend(fontsize=8)
 axes[0,c].set_title(name+' · '+('best 87, 20 agosto' if name=='V26' else 'best 54, 18 settembre'))
 axes[2,c].plot(times,[v for v,_ in scores],color='#42785b');axes[2,c].set_ylabel('Reward per step');axes[2,c].set_xlabel('Tempo dal reset [s]');axes[2,c].grid(alpha=.2)
 axes[0,c].set_ylim(0,75);axes[1,c].set_ylim(-15,30);axes[2,c].set_ylim(-.1,1.05)
fig.suptitle('V26 e 39D: stessa durata, target e start nativi differenti',fontsize=16)
fig.text(.07,.012,'Ogni rollout è confrontato con il proprio target. Un solo seed e uno start per caso: confronto descrittivo, non test statistico di equivalenza.',fontsize=9)
fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
(OUT/'metrics.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
