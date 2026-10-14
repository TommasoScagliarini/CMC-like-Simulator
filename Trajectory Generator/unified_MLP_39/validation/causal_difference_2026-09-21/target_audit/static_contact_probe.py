"""Instantaneous contact counterfactual at recorded last40 state; no integration."""
from pathlib import Path
import json
import sys
import os

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[4]
TG=REPO/'Trajectory Generator'
PACKAGE=TG/'unified_MLP_39'
sys.path.insert(0,str(TG))
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(key,'1')
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime
from unified_MLP_39 import env_factory, train_ppo_mlp as trainer
import numpy as np
from output import _read_storage_table
from online_grf import read_online_grf

RUN=PACKAGE/'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40'
config=RUN/'training_cfg.resolved.yaml'
sys.argv=['static_contact_probe','--config',str(config)]
args=trainer.parse_args()
trainer._load_training_stack()
cfg=trainer.build_config(args,args._cfg_reward)
env=env_factory.make_cmc_env(cfg.env_config)
base=env.unwrapped
try:
    env.reset(seed=123)
    model=base.ctx.model
    state=base.runner.state
    target_time=17.53
    trace=json.loads((RUN/'rollout_policy_trace.json').read_text())
    step=min(trace,key=lambda r:abs(r['time']-target_time))
    t,columns,matrix=_read_storage_table(str(RUN/'sim_outputs/rollout_episode_states.sto'))
    row=matrix[int(np.argmin(abs(t-target_time)))]
    assert abs(t[np.argmin(abs(t-target_time))]-target_time)<1e-9
    values=dict(zip(columns,row))
    coords=model.getCoordinateSet()
    joint_names=[coords.get(i).getName() for i in range(coords.getSize())]
    assert all(name+'_q' in values and name+'_qdot' in values for name in joint_names)
    baseline_q={name:float(values[name+'_q']) for name in joint_names}
    baseline_qd={name:float(values[name+'_qdot']) for name in joint_names}
    reference_terms={k:v for k,v in step['reward_terms'].items() if k.startswith('grf_ankle_moment_flip_')}
    gt,gcolumns,gmatrix=_read_storage_table(str(RUN/'sim_outputs/rollout_episode_online_grf.sto'))
    grf_record=dict(zip(gcolumns,gmatrix[int(np.argmin(abs(gt-target_time)))]))

    def evaluate(changes):
        state.setTime(target_time)
        for name in joint_names:
            coord=coords.get(name)
            coord.setValue(state,changes.get(name,baseline_q[name]),False)
            coord.setSpeedValue(state,baseline_qd[name])
        model.realizeVelocity(state)
        recorded_q={name:float(coords.get(name).getValue(state)) for name in joint_names}
        recorded_qd={name:float(coords.get(name).getSpeedValue(state)) for name in joint_names}
        assert max(abs(recorded_q[n]-changes.get(n,baseline_q[n])) for n in joint_names)<1e-12
        assert max(abs(recorded_qd[n]-baseline_qd[n]) for n in joint_names)<1e-12
        grf=read_online_grf(model,state,base.ctx.online_grf_force_paths,base.ctx.online_grf_force_sides)
        base._online_grf=grf['sides']
        terms=base._grf_ankle_moment_flip_terms()
        left=grf['sides']['left']
        center,axis=base._prosthetic_ankle_center_axis_ground()
        return {'changes_rad':changes,'changes_deg':{k:float(np.rad2deg(v)) for k,v in changes.items()},
            'max_unchanged_q_delta_rad':max(abs(recorded_q[n]-baseline_q[n]) for n in joint_names if n not in changes),
            'max_qdot_delta':max(abs(recorded_qd[n]-baseline_qd[n]) for n in joint_names),
            'normal_force_n':float(left['normal_force']),'penetration_m':float(left['penetration']),
            'moment_ankle_nm':terms['grf_ankle_moment_flip_tau_nm'],
            'flip_loss':terms['grf_ankle_moment_flip_loss'],
            'reward_flip_penalty':.1*terms['grf_ankle_moment_flip_loss'],
            'cop_xyz_m':left['cop'].tolist(),'ankle_center_xyz_m':center.tolist(),
            'ankle_axis_xyz':axis.tolist(),'grf_force_xyz_n':left['force'].tolist(),
            'grf_moment_ground_xyz_nm':left['moment'].tolist(),
            'terms':terms}

    result={'time_s':target_time,'source_rollout':str(RUN.relative_to(REPO)),
        'baseline_q':baseline_q,'baseline_qdot':baseline_qd,'reference_trace_terms':reference_terms,
        'conditions':{},'caveats':[
            'No time integration; these discontinuous coordinate interventions are not physically executable trajectories.',
            'All 21 coordinate positions and speeds reconstructed from STO rounded to 8 decimals; only explicitly listed positions change.',
            'Muscle states, SEA internal states and controls remain at env reset; contact component is memoryless and depends on pose/velocity, not these states. Baseline contact reproduction validates this for the diagnostic.',
            'qdot is fixed numerically in generalized coordinates; changing q changes Cartesian contact velocities as part of the biomechanical configuration effect.',
            'Physics context is current checkpoint config with unchanged FSMv3; no FSM update or reward recomputation except moment diagnostic.',
            'Trace reward at 17.53 uses last cached contact wrench at 17.529; this static probe recomputes simultaneous pose and wrench at 17.53 and validates against 1 kHz STO, not the lagged trace wrench.',
        ]}
    result['conditions']['recorded_baseline']=evaluate({})
    actual=result['conditions']['recorded_baseline']
    errors={'normal_force_n':abs(actual['normal_force_n']-grf_record['left_normal_force']),
        'force_max_component_n':float(np.max(np.abs(np.array(actual['grf_force_xyz_n'])-np.array([grf_record['left_force_'+a] for a in 'xyz'])))),
        'moment_ground_max_component_nm':float(np.max(np.abs(np.array(actual['grf_moment_ground_xyz_nm'])-np.array([grf_record['left_moment_'+a] for a in 'xyz'])))),
        'cop_max_component_m':float(np.max(np.abs(np.array(actual['cop_xyz_m'])-np.array([grf_record['left_cop_'+a] for a in 'xyz']))))}
    result['baseline_reproduction_absolute_errors']=errors
    result['reference_simultaneous_grf_sto']=grf_record
    assert errors['moment_ground_max_component_nm']<.002,errors
    assert errors['force_max_component_n']<.02,errors
    assert errors['cop_max_component_m']<1e-5,errors
    targets=step['imitation_target_q']
    conditions={
        'knee_to_left_target':{'pros_knee_angle':targets['pros_knee_angle']},
        'ankle_to_left_target':{'pros_ankle_angle':targets['pros_ankle_angle']},
        'both_to_left_targets':dict(targets),
        'knee_to_iter25_value':{'pros_knee_angle':float(np.deg2rad(-19.445461482114386))},
        'knee_to_v26_value':{'pros_knee_angle':float(np.deg2rad(-22.35467212117777))},
    }
    for name,changes in conditions.items():result['conditions'][name]=evaluate(changes)
    (HERE/'static_contact_probe.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'baseline_errors':errors,'conditions':{k:{p:v[p] for p in ('normal_force_n','moment_ankle_nm','penetration_m','reward_flip_penalty')} for k,v in result['conditions'].items()}},indent=2))
finally:
    env.close()
