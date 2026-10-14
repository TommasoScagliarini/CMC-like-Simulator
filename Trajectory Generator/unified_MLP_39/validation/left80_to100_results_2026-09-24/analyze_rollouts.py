"""Offline torque, contact and reward audit; leaves source runs unchanged."""
from pathlib import Path
import sys, os, json, io, hashlib
HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
sys.path.insert(0, str(TG))
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ[key] = '1'
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch
from unified_MLP_39.reward_function import RewardConfig, compute_reward
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SOURCES = {
 'best80_pen2': PACKAGE/'runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic',
 'new_best82_pen2': PACKAGE/'runs/rollout/MLP39_LEFT_pen2_from80_to100_2026-09-24_best82_deterministic',
 'new_last100_pen2': PACKAGE/'runs/rollout/MLP39_LEFT_pen2_from80_to100_2026-09-24_last100_deterministic',
 'V26_best87': TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
}
WINDOW_START = 13.15
WINDOW_END = min(17.95, *(json.loads((p/'rollout_policy_trace.json').read_text())[-1]['time']-.04 for p in SOURCES.values()))
assert WINDOW_END > WINDOW_START
HASHES = {}
def read(path):
    b=path.read_bytes(); HASHES[str(path.relative_to(REPO))]=hashlib.sha256(b).hexdigest()
    return b.decode()
def table(path):
    s=read(path).split('endheader',1)[1].strip()
    cols=s.splitlines()[0].split(); a=np.loadtxt(io.StringIO(s),skiprows=1)
    return dict(zip(cols,a.T))
def rms(a): return float(np.sqrt(np.mean(np.asarray(a)**2)))
def corr(a,b): return float(np.corrcoef(a,b)[0,1])
def interp(data,key,t): return np.interp(t,data['time'],data[key])
def periods(t,m):
    a=np.flatnonzero(m & ~np.r_[False,m[:-1]]); b=np.flatnonzero(m & ~np.r_[m[1:],False])
    return [[float(t[i]),float(t[j])] for i,j in zip(a,b)]
results={}; signals={}
for name,folder in SOURCES.items():
    summary=json.loads(read(folder/'rollout_summary.json'))
    trace=json.loads(read(folder/'rollout_policy_trace.json'))
    tables={s:table(folder/'sim_outputs'/f'rollout_episode_{s}.sto') for s in ['sea_torques','sea_diagnostics','kinematics','kinematics_reference','online_grf']}
    torque=tables['sea_torques']; diag=tables['sea_diagnostics']; kin=tables['kinematics']; ref=tables['kinematics_reference']; grf=tables['online_grf']
    t=torque['time']; dt=float(np.median(np.diff(t))); assert np.max(np.abs(np.diff(t)-dt))<1e-7
    mask=(t>=WINDOW_START)&(t<=WINDOW_END)
    tt=np.array([r['time'] for r in trace]); tm=(tt>=WINDOW_START)&(tt<=WINDOW_END)
    terms=[r['reward_terms'] for r in trace]
    force=np.array([r.get('grf_ankle_moment_flip_force_n',np.nan) for r in terms])
    mom=np.array([r.get('grf_ankle_moment_flip_tau_nm',np.nan) for r in terms])
    ankle=torque['SEA_Ankle_tau_spring']; ankle_ref=interp(diag,'SEA_Ankle_tau_ref',t)
    loaded=tm & (force>=50) & np.isfinite(mom)
    peak=int(np.argmax(np.where(mask,ankle,-np.inf))); pt=float(t[peak]); idx=int(np.argmin(abs(tt-pt)))
    cfg=RewardConfig.from_dict(summary['reward_config']) if hasattr(RewardConfig,'from_dict') else RewardConfig(**summary['reward_config'])
    components=[compute_reward(r['reward_terms'],cfg,reference=r['policy_segment_values']) for r in trace]
    ret=sum(x[0] for x in components)
    assert abs(ret-summary['episode_return'])<1e-5,(name,ret,summary['episode_return'])
    costs={k:float(getattr(cfg,k)*sum(r.get(k.replace('_weight','_loss'),0) for r in terms)) for k in ['effort_weight','command_rate_weight','sea_torque_error_weight','sea_motor_speed_weight','sea_motor_accel_weight','sea_motor_power_weight','sea_tau_spring_rate_weight','sea_tau_spring_effort_weight','u_rate_weight','jerk_ref_weight']}
    budget={k:float(sum(c[k] for _,c in components)) for k in ['penalty','grf_penetration_term','grf_ankle_moment_flip_term','reward_base','oob_term']}
    budget.update(served_score=float(cfg.blend_served_imitation*sum(c['served_imitation_score'] for _,c in components)),actual_score=float(cfg.blend_imitation*sum(c['imitation_score'] for _,c in components)),return_total=ret,costs=costs)
    budget['command_rate_parts']={key:float(cfg.command_rate_weight/6*sum(r.get(key,0) for r in terms)) for key in ['segment_delta_loss','qdot_ref_loss','qddot_ref_loss','jerk_ref_loss','reference_governor_loss','u_rate_loss']}
    assert abs(sum(budget['command_rate_parts'].values())-costs['command_rate_weight'])<1e-8
    hf={}
    for cutoff in (10,20):
        sos=butter(4,cutoff,fs=1/dt,btype='highpass',output='sos')
        def hp(x): return sosfiltfilt(sos,x)[mask]
        channels={
            'knee_spring':torque['SEA_Knee_tau_spring'],
            'knee_motor':interp(diag,'SEA_Knee_tau_input_plugin',t),
            'knee_ref_torque':interp(diag,'SEA_Knee_tau_ref',t),
            'knee_P':interp(diag,'SEA_Knee_cascade_inner_p_cmd',t),
            'knee_I':interp(diag,'SEA_Knee_cascade_inner_i_cmd',t),
            'knee_velocity_error':interp(diag,'SEA_Knee_cascade_velocity_error',t),
            'knee_cascade_qdot_ref':interp(diag,'SEA_Knee_cascade_qdot_ref',t),
            'knee_joint_velocity':interp(diag,'SEA_Knee_cascade_qdot_ref',t)-interp(diag,'SEA_Knee_cascade_velocity_error',t),
            'knee_served_q':interp(ref,'pros_knee_angle_q_ref',t),
            'knee_served_qdot':interp(ref,'pros_knee_angle_qdot_ref',t),
            'ankle_spring':ankle,
        }
        filtered={k:hp(v) for k,v in channels.items()}
        vals={k:rms(v) for k,v in filtered.items()}
        freq,psd=welch(filtered['knee_spring'],fs=1/dt,nperseg=min(2048,int(mask.sum())))
        band=(freq>=cutoff)&(freq<=200)
        vals['knee_spring_peak_frequency_hz']=float(freq[band][np.argmax(psd[band])])
        vals['corr_knee_tau_ref_P']=corr(filtered['knee_ref_torque'],filtered['knee_P'])
        vals['corr_knee_tau_ref_I']=corr(filtered['knee_ref_torque'],filtered['knee_I'])
        hf[str(cutoff)]=vals
    tracking={}
    for joint in ('knee','ankle'):
        coord='pros_'+joint+'_angle'
        tracking[joint+'_joint_vs_served_rmse_deg']=float(np.rad2deg(rms(interp(kin,coord,t)[mask]-interp(ref,coord+'_q_ref',t)[mask])))
        tracking[joint+'_actual_vs_target_rmse_deg']=float(np.rad2deg(rms([r['prosthetic_state'][coord]-r['imitation_target_q'][coord] for r in trace if WINDOW_START<=r['time']<=WINDOW_END])))
        tracking[joint+'_served_vs_target_rmse_deg']=float(np.rad2deg(rms([r['prosthetic_state'][coord+'_served_ref']-r['imitation_target_q'][coord] for r in trace if WINDOW_START<=r['time']<=WINDOW_END])))
    dm=(diag['time']>=WINDOW_START)&(diag['time']<=WINDOW_END)
    identity={joint:float(np.max(abs(diag[f'SEA_{joint}_tau_ref'][dm]-diag[f'SEA_{joint}_cascade_inner_p_cmd'][dm]-diag[f'SEA_{joint}_cascade_inner_i_cmd'][dm]))) for joint in ['Knee','Ankle']}
    assert max(identity.values())<1e-6
    result={'source':str(folder.relative_to(REPO)),'window_s':[WINDOW_START,WINDOW_END],'ankle':{
        'min_nm':float(ankle[mask].min()),'max_nm':float(ankle[mask].max()),
        'positive_area_nm_s':float(np.maximum(ankle[mask],0).sum()*dt),
        'negative_area_nm_s':float(np.minimum(ankle[mask],0).sum()*dt),
        'positive_over8_windows_s':periods(t,mask&(ankle>8)),
        'tau_ref_spring_rmse_nm':rms((ankle_ref-ankle)[mask]),
        'loaded_corr_spring_negative_grf':corr(np.interp(tt,t,ankle)[loaded],-mom[loaded]),
        'loaded_balance_rms_nm':rms(np.interp(tt,t,ankle)[loaded]+mom[loaded]),
        'peak':{'time':pt,'tau':float(ankle[peak]),'tau_ref':float(ankle_ref[peak]),
            'cascade_P':float(interp(diag,'SEA_Ankle_cascade_inner_p_cmd',pt)),
            'cascade_I':float(interp(diag,'SEA_Ankle_cascade_inner_i_cmd',pt)),
            'force_n':float(interp(grf,'left_normal_force',pt)),
            'penetration_mm':float(interp(grf,'left_penetration',pt)*1000),
            'nearest_trace_time':float(tt[idx]),'grf_moment':float(mom[idx]),
            'cop_z_minus_ankle_z_m':terms[idx].get('grf_ankle_moment_flip_cop_z_minus_ankle_z_m'),
            'actual_ankle_deg':float(np.rad2deg(trace[idx]['prosthetic_state']['pros_ankle_angle'])),
            'target_ankle_deg':float(np.rad2deg(trace[idx]['imitation_target_q']['pros_ankle_angle']))}},
        'knee_torque_ref_spring_rmse_nm':rms((interp(diag,'SEA_Knee_tau_ref',t)-torque['SEA_Knee_tau_spring'])[mask]),
        'highpass_rms':hf,'tracking':tracking,'budget':budget,'controller_identity_error_nm':identity,
        'controller_flags':{k:float(diag[k][dm].max()) for k in ['SEA_Knee_tau_input_saturated','SEA_Ankle_tau_input_saturated','SEA_Knee_cascade_i_clamped','SEA_Ankle_cascade_i_clamped']},
        'mean_jerk_limit_fraction':float(np.mean([r['reference_governor_diagnostics'].get('jerk_limit_fraction',0) for r in trace])),
        'max_penetration_mm':summary['grf_penetration_max_m']*1000}
    results[name]=result
    signals[name]=(t,torque,diag,tt,mom,grf)
(HERE/'metrics.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
(HERE/'input_hashes.json').write_text(json.dumps(HASHES,indent=2)+'\n')

plotdir=REPO/'plot/09_24_2026_LEFT_from80_to100_diagnosis'; plotdir.mkdir(exist_ok=True)
fig,axs=plt.subplots(4,1,figsize=(12,12),sharex=True,layout='constrained')
t,torque,diag,tt,mom,grf=signals['new_best82_pen2']
axs[0].plot(t,torque['SEA_Ankle_tau_spring'],label='SEA ankle torque')
axs[0].plot(tt,-mom,'--',label='Opposite GRF moment (100 Hz)')
axs[0].set_ylabel('Ankle [Nm]')
axs[1].plot(diag['time'],diag['SEA_Ankle_cascade_inner_p_cmd'],label='Cascade P')
axs[1].plot(diag['time'],diag['SEA_Ankle_cascade_inner_i_cmd'],label='Cascade I')
axs[1].set_ylabel('Ankle command [Nm]')
axs[2].plot(grf['time'],1000*grf['left_penetration'],label='Penetration')
axs[2].axhline(20,color='black',linestyle='--',label='Penalty threshold')
axs[2].set_ylabel('Penetration [mm]')
axs[3].plot(t,torque['SEA_Knee_tau_spring'],label='SEA knee torque')
axs[3].plot(diag['time'],diag['SEA_Knee_tau_ref'],alpha=.6,label='Requested torque')
axs[3].set_ylabel('Knee [Nm]');axs[3].set_xlabel('Time [s]')
for ax in axs: ax.legend(); ax.grid(alpha=.25);ax.set_xlim(WINDOW_START,WINDOW_END)
fig.savefig(plotdir/'new_best82_contact_controller.png',dpi=160);plt.close(fig)
print(json.dumps({k:{'ankle_peak':v['ankle']['peak'],'positive_area':v['ankle']['positive_area_nm_s'],'hf20':v['highpass_rms']['20'],'budget':v['budget'],'flags':v['controller_flags']} for k,v in results.items()},indent=2))
