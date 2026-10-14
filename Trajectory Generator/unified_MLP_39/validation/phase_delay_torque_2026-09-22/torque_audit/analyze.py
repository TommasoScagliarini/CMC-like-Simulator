"""Archived SEA ankle torque/contact audit. No runtime or simulation changes."""
from pathlib import Path
import json
import sys
import os

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[4]
TG=REPO/'Trajectory Generator'
U=TG/'unified_MLP_39'
sys.path.insert(0,str(REPO))
os.environ.setdefault('MPLCONFIGDIR','/private/tmp/cmc-torque-audit-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from output import _read_storage_table

PATHS={
 'iter25':U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25',
 'best29':U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29',
 'last40':U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40',
 'v26_best87':TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
 'june17_asym100':TG/'runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100',
 'june22_hard_grf':TG/'runs/rollout/MLP_imitation_rollout_06-22-2026',
 'june23_soft_grf':TG/'runs/rollout/MLP_imitation_rollout_06-23-2026_grfsoft_knee1_ankle2_100iter',
}
def table(path):
 t,c,a=_read_storage_table(str(path));return {'time':t,**dict(zip(c,a.T))}
def value(data,key,time):return float(np.interp(time,data['time'],data[key]))
def periods(time,mask):
 starts=np.flatnonzero(mask & ~np.r_[False,mask[:-1]])
 ends=np.flatnonzero(mask & ~np.r_[mask[1:],False])
 return [[float(time[a]),float(time[b]),float(time[b]-time[a]+.001)] for a,b in zip(starts,ends)]
results={};series={}
for name,path in PATHS.items():
 if not path.exists():continue
 summary=json.loads((path/'rollout_summary.json').read_text())
 trace=json.loads((path/'rollout_policy_trace.json').read_text())
 sto=path/'sim_outputs'
 torque=table(sto/'rollout_episode_sea_torques.sto')
 diag=table(sto/'rollout_episode_sea_diagnostics.sto')
 grf=table(sto/'rollout_episode_online_grf.sto')
 times=np.array([r['time'] for r in trace]);force=np.array([r['reward_terms'].get('grf_ankle_moment_flip_force_n',np.nan) for r in trace]);mom=np.array([r['reward_terms'].get('grf_ankle_moment_flip_tau_nm',np.nan) for r in trace])
 ts=torque['time'];spring=torque['SEA_Ankle_tau_spring'];ref=np.interp(ts,diag['time'],diag['SEA_Ankle_tau_ref'])
 valid=ts>=ts[0]+.2
 dt=float(np.median(np.diff(ts)))
 at_trace=np.interp(times,ts,spring);refs_at_trace=np.interp(times,diag['time'],diag['SEA_Ankle_tau_ref'])
 loaded=(force>=50)&(times>=ts[0]+.2)&np.isfinite(mom)
 peak=int(np.argmax(np.where(valid,spring,-np.inf)));peak_t=float(ts[peak])
 def snapshot(t):
  tr=trace[int(np.argmin(abs(times-t)))];terms=tr['reward_terms']
  return {'time_s':float(t),'trace_time_s':tr['time'],
    'sea_tau_spring_nm':value(torque,'SEA_Ankle_tau_spring',t),
    'sea_tau_ref_nm':value(diag,'SEA_Ankle_tau_ref',t),
    'sea_cascade_p_nm':value(diag,'SEA_Ankle_cascade_inner_p_cmd',t),
    'sea_cascade_i_nm':value(diag,'SEA_Ankle_cascade_inner_i_cmd',t),
    'sea_tau_error_nm':value(diag,'SEA_Ankle_tau_error',t),
    'left_normal_force_n':value(grf,'left_normal_force',t),
    'left_penetration_mm':1000*value(grf,'left_penetration',t),
    'trace_grf_ankle_moment_nm':terms.get('grf_ankle_moment_flip_tau_nm'),
    'trace_cop_z_minus_ankle_z_m':terms.get('grf_ankle_moment_flip_cop_z_minus_ankle_z_m'),
    'trace_grf_flip_loss':terms.get('grf_ankle_moment_flip_loss'),
    'ankle_actual_deg':float(np.rad2deg(tr['prosthetic_state']['pros_ankle_angle'])),
    'ankle_target_deg':float(np.rad2deg(tr['imitation_target_q']['pros_ankle_angle']))}
 result={'source':str(path.relative_to(REPO)),'time_span_s':[float(ts[0]),float(ts[-1])],
  'reward_config':{k:summary['reward_config'].get(k) for k in ['grf_ankle_moment_flip_weight','grf_ankle_moment_flip_tau_tol_nm','grf_ankle_moment_flip_force_threshold_n','grf_penetration_weight']},
  'post_200ms':{'tau_spring_rms_nm':float(np.sqrt(np.mean(spring[valid]**2))),
   'tau_spring_min_max_nm':[float(spring[valid].min()),float(spring[valid].max())],
   'positive_area_nm_s':float(np.maximum(spring[valid],0).sum()*dt),
   'negative_area_nm_s':float(np.minimum(spring[valid],0).sum()*dt),
   'positive_fraction':float(np.mean(spring[valid]>0)),
   'tau_ref_vs_spring_rmse_nm':float(np.sqrt(np.mean((ref[valid]-spring[valid])**2))),
   'loaded_samples_trace':int(loaded.sum()),
   'loaded_spring_vs_negative_grf_correlation':float(np.corrcoef(at_trace[loaded],-mom[loaded])[0,1]) if loaded.any() else None,
   'loaded_spring_plus_grf_rms_nm':float(np.sqrt(np.mean((at_trace[loaded]+mom[loaded])**2))) if loaded.any() else None,
   'positive_above_8nm_windows_s':periods(ts,(spring>8)&valid)},
  'peak_positive':snapshot(peak_t),'at17_53':snapshot(17.53),'final_sample':snapshot(float(ts[-1]))}
 if loaded.any():result['grf_flip_penalty_total']=float(summary['reward_config']['grf_ankle_moment_flip_weight']*sum(r['reward_terms'].get('grf_ankle_moment_flip_loss',0) for r in trace))
 if name=='last40':
  m=(ts>=17.2)&(ts<=17.85)
  result['last_stance_17_2_17_85']={'positive_fraction':float(np.mean(spring[m]>0)),
    'tau_spring_mean_nm':float(spring[m].mean()),'tau_spring_max_nm':float(spring[m].max()),
    'tau_ref_mean_nm':float(ref[m].mean()),
    'positive_area_nm_s':float(np.maximum(spring[m],0).sum()*dt),
    'normal_force_mean_n':float(np.mean(np.interp(ts[m],grf['time'],grf['left_normal_force'])))}
  dm=(diag['time']>=17.183)&(diag['time']<=17.808)
  result['last_positive_burst_controller_flags']={key:float(diag[key][dm].max()) for key in
    ['SEA_Ankle_cascade_i_clamped','SEA_Ankle_cascade_anti_windup_active','SEA_Ankle_tau_input_saturated']}
  result['tau_ref_equals_cascade_p_plus_i_max_error_nm']=float(np.max(np.abs(diag['SEA_Ankle_tau_ref']-diag['SEA_Ankle_cascade_inner_p_cmd']-diag['SEA_Ankle_cascade_inner_i_cmd'])))
  assert result['tau_ref_equals_cascade_p_plus_i_max_error_nm']<2e-8
 results[name]=result
 series[name]={'t':ts,'spring':spring,'ref':ref,'trace_t':times,'grf_moment':mom,'force':force,'diag':diag,'grf':grf}

fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True,layout='constrained')
for name,color in [('iter25','#64748b'),('best29','#a855f7'),('last40','#2563eb'),('v26_best87','#d97706')]:
 s=series[name]
 axs[0].plot(s['t'],s['spring'],color=color,label=name)
 axs[1].plot(s['trace_t'],s['grf_moment'],color=color,label=name)
axs[0].set_ylabel('SEA ankle tau_spring (Nm)');axs[1].set_ylabel('Momento GRF ankle (Nm)')
axs[1].axhline(-8,color='black',linestyle='--',label='Soglia anti-flip −8 Nm')
s=series['last40'];axs[2].plot(s['t'],s['spring'],label='tau_spring',color='#2563eb');axs[2].plot(s['t'],s['ref'],label='tau_ref',color='#d97706',linestyle='--');axs[2].plot(s['trace_t'],-s['grf_moment'],label='− momento GRF',color='black',linestyle=':')
axs[2].set_ylabel('Last40: coppia SEA / −GRF (Nm)');axs[2].set_xlabel('Tempo simulazione (s)')
axs[0].legend(ncol=4);axs[2].legend(ncol=3)
for ax in axs:ax.grid(alpha=.2);ax.axhline(0,color='gray',linewidth=.7);ax.set_xlim(13.19,17.99)
fig.suptitle('Coppia positiva della caviglia e momento di contatto opposto — stessa ricorrenza meccanica di giugno')
fig.savefig(HERE/'ankle_torque_vs_grf.png',dpi=160);plt.close(fig)

fig,axs=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
axs[0].plot(s['t'],s['spring'],label='tau_spring SEA',color='#2563eb');axs[0].plot(s['t'],s['ref'],label='tau_ref cascade',color='#d97706',linestyle='--');axs[0].plot(s['trace_t'],-s['grf_moment'],label='− momento GRF',color='black',linestyle=':');axs[0].set_ylabel('Coppia (Nm)')
axs[1].plot(s['diag']['time'],s['diag']['SEA_Ankle_cascade_inner_p_cmd'],label='Termine P cascade');axs[1].plot(s['diag']['time'],s['diag']['SEA_Ankle_cascade_inner_i_cmd'],label='Termine I cascade');axs[1].set_ylabel('Contributi controller (Nm)')
axs[2].plot(s['grf']['time'],s['grf']['left_penetration']*1000,label='Penetrazione',color='#2563eb');axs[2].axhline(20,color='black',linestyle='--',label='Soglia penalty penetrazione');axs[2].set_ylabel('Penetrazione (mm)');axs[2].set_xlabel('Tempo simulazione (s)')
for ax in axs:ax.legend();ax.grid(alpha=.2);ax.set_xlim(17.15,17.99);ax.axvline(17.53,color='gray',linestyle=':')
fig.suptitle('Ultimo appoggio last40: il SEA compensa il momento negativo del contatto')
fig.savefig(HERE/'last_stance_cascade.png',dpi=160);plt.close(fig)
(HERE/'metrics.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
print(json.dumps({n:{'post_200ms':v['post_200ms'],'peak_positive':v['peak_positive'],'penalty':v.get('grf_flip_penalty_total')} for n,v in results.items()},indent=2))
