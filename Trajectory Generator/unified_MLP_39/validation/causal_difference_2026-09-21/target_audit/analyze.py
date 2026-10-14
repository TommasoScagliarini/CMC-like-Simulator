"""Read-only target/contact audit of archived deterministic rollouts."""
from pathlib import Path
import json
import sys
import os

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
TG = REPO / 'Trajectory Generator'
sys.path.insert(0, str(REPO))
os.environ.setdefault('MPLCONFIGDIR', '/private/tmp/cmc-target-audit-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator

U = TG / 'unified_MLP_39'
PATHS = {
    'iter25': U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25',
    'best29': U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29',
    'last40': U/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40',
    'v26_best87': TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
}
JOINTS = ('pros_knee_angle', 'pros_ankle_angle')
traces = {k: json.loads((v/'rollout_policy_trace.json').read_text()) for k, v in PATHS.items()}
time = np.array([r['time'] for r in traces['last40']])
kin = KinematicsInterpolator(SimulatorConfig())
raw = [kin.get(t) for t in time]
data = {'targets': {}, 'contacts': {}, 'limitations': [
    'All deterministic rollouts use the same reset and duration, but V26 policy is mature best87 versus current25/29/40.',
    'Time-local contact comparisons change both joint kinematics and resulting dynamics; these are associations, not isolated interventions.',
    'Effective normalization coefficients describe pre-score errors; complete reward has nonlinear scores and physical penalties.',
]}

for name in ('last40', 'v26_best87'):
    trace = traces[name]
    data['targets'][name] = {}
    for joint in JOINTS:
        q = np.array([r['imitation_target_q'][joint] for r in trace])
        qd = np.array([r['imitation_target_qdot'][joint] for r in trace])
        qa = np.array([x[0][joint] for x in raw])
        terms = trace[0]['reward_terms']
        pr = terms[joint+'_imitation_position_range_rad']
        vr = terms[joint+'_imitation_velocity_range_rad_s']
        pw = 1. if joint==JOINTS[0] else 2.
        vw = .02 if joint==JOINTS[0] else .04
        data['targets'][name][joint] = {
            'position_deg_min_max': np.rad2deg([q.min(), q.max()]).tolist(),
            'velocity_rad_s_min_max': [float(qd.min()),float(qd.max())],
            'position_range_rad':pr,'velocity_range_rad_s':vr,
            'effective_position_coefficient_per_rad2':pw/pr**2,
            'effective_velocity_coefficient_per_rad_s2':vw/vr**2,
            'five_degree_position_error_loss':pw*(np.deg2rad(5)/pr)**2,
            'rmse_vs_raw_prescribed_left_deg':float(np.rad2deg(np.sqrt(np.mean((q-qa)**2)))),
            'target_below_zero_steps':int(np.sum(q<0)),
            'target_velocity_abs_max_rad_s':float(abs(qd).max()),
        }

for name, trace in traces.items():
    terms = [r['reward_terms'] for r in trace]
    flip = np.array([t['grf_ankle_moment_flip_loss'] for t in terms])
    force = np.array([t['grf_ankle_moment_flip_force_n'] for t in terms])
    tau = np.array([t['grf_ankle_moment_flip_tau_nm'] for t in terms])
    active = flip>0
    leftphase = np.array([r['reward_terms'][JOINTS[0]+'_imitation_target_phase'] for r in traces['last40']])
    peak = int(np.argmax(flip))
    def row_info(i):
        r=trace[i]; terms=r['reward_terms']
        return {'time_s':r['time'],'episode_time_s':r['time']-12.99,
            'left_phase':float(leftphase[i]),'tau_nm':float(tau[i]),'normal_force_n':float(force[i]),
            'cop_z_minus_ankle_z_m':terms['grf_ankle_moment_flip_cop_z_minus_ankle_z_m'],
            'flip_penalty':float(.1*flip[i]),
            'joints':{j:{'actual_deg':float(np.rad2deg(r['prosthetic_state'][j])),
                'served_deg':float(np.rad2deg(r['prosthetic_state'][j+'_served_ref'])),
                'target_deg':float(np.rad2deg(r['imitation_target_q'][j])),
                'raw_left_deg':float(np.rad2deg(raw[i][0][j]))} for j in JOINTS}}
    data['contacts'][name]={'penalty_total':float(.1*flip.sum()),'penalized_steps':int(active.sum()),
        'peak':row_info(peak),'at_last40_peak':row_info(453),'at_best29_peak':row_info(449),
        'phase_bins':[{ 'left_phase_range':[lo,lo+.1],
            'samples':int(((leftphase>=lo)&(leftphase<lo+.1)).sum()),
            'penalty_sum':float(.1*flip[(leftphase>=lo)&(leftphase<lo+.1)].sum())}
            for lo in np.arange(0,1,.1)]}

# Check V26 observation target channels against stored target at previous step.
old=traces['v26_best87']
names=list(old[0]['actor_observation_before'])
data['historical_actor_target_feature_names']=[n for n in names if 'target' in n or 'gait_phase' in n]
data['clock_alignment'] = {}
for joint in JOINTS:
    key=joint+'_imitation_target_phase'
    current=np.array([r['reward_terms'][key] for r in traces['last40']])
    historical=np.array([r['reward_terms'][key] for r in old])
    delta=(historical-current+.5)%1-.5
    data['clock_alignment'][joint]={'historical_minus_left_phase_mean':float(delta.mean()),
        'historical_minus_left_phase_min':float(delta.min()),
        'historical_minus_left_phase_max':float(delta.max())}

fig, axes=plt.subplots(4,1,figsize=(13,12),sharex=True,layout='constrained')
for i,joint in enumerate(JOINTS):
    for name,color,label in [('last40','#2563eb','Target LEFT attuale'),('v26_best87','#d97706','Target V26 RIGHT sfasato')]:
        axes[i].plot(time-12.99,np.rad2deg([r['imitation_target_q'][joint] for r in traces[name]]),color=color,label=label)
    axes[i].plot(time-12.99,np.rad2deg([x[0][joint] for x in raw]),color='black',linestyle=':',label='Prescribed LEFT originale')
    axes[i].set_ylabel(('Ginocchio' if i==0 else 'Caviglia')+' target (°)')
for name,color in [('iter25','#64748b'),('best29','#a855f7'),('last40','#2563eb'),('v26_best87','#d97706')]:
    trace=traces[name]
    axes[2].plot(time-12.99,[r['reward_terms']['grf_ankle_moment_flip_tau_nm'] for r in trace],label=name,color=color)
    axes[3].plot(time-12.99,[r['reward_terms']['grf_ankle_moment_flip_force_n'] for r in trace],label=name,color=color)
axes[2].axhline(-8,color='black',linestyle='--',label='Soglia −8 Nm')
axes[2].set_ylabel('Momento GRF caviglia (Nm)')
axes[3].set_ylabel('Carico normale (N)');axes[3].set_xlabel('Tempo episodio (s)')
axes[0].legend(ncol=3,fontsize=9);axes[2].legend(ncol=5,fontsize=9)
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('Target deliberatamente diverso e contatto risultante: confronto descrittivo, non causale')
fig.savefig(HERE/'target_contact_comparison.png',dpi=150);plt.close(fig)
(HERE/'metrics.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
print(json.dumps(data,indent=2))
