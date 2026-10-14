"""Offline reference-governor replay and identical-method endpoint optimization.

No OpenSim model is built. Production kinematics is AST-loaded, so importing this
diagnostic cannot accidentally create a physical simulation.
"""
from __future__ import annotations
import ast
import hashlib
import io
import json
import os
from pathlib import Path
import time
from typing import Sequence

for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(key, '1')
import numpy as np
import yaml
from scipy.interpolate import BPoly, CubicHermiteSpline, PchipInterpolator
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
JOINTS = ('pros_knee_angle', 'pros_ankle_angle')
SOURCES = {
    'LEFT': PACKAGE/'runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic',
    'RIGHT': PACKAGE/'runs/rollout/MLP39_right_v26_60iter_2026-09-23_last60',
    'V26': TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
    'oracle_LEFT': PACKAGE/'runs/rollout/causal_difference_2026-09-21_oracle_left',
    'oracle_RIGHT': PACKAGE/'runs/rollout/causal_difference_2026-09-21_oracle_right',
}
HASHES = {}


def read(path):
    b = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(b).hexdigest()
    return b.decode()


def checkpoint_config(summary):
    """Relocate archived Windows/macOS checkpoint paths into this checkout."""
    saved=str(summary['checkpoint']).replace('\\','/')
    checkpoint=TG/saved.split('Trajectory Generator/',1)[1]
    return next(parent/'training_cfg.resolved.yaml' for parent in checkpoint.parents
                if (parent/'training_cfg.resolved.yaml').exists())


source = read(TG/'osim_trj_cmc_like.py')
node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == 'ProstheticSegmentKinematics')
namespace = dict(np=np, Sequence=Sequence, BPoly=BPoly, CubicHermiteSpline=CubicHermiteSpline, PchipInterpolator=PchipInterpolator)
exec(compile('from __future__ import annotations\n' + ast.get_source_segment(source, node), str(TG/'osim_trj_cmc_like.py'), 'exec'), namespace)
Kinematics = namespace['ProstheticSegmentKinematics']


class Base:
    coord_names = JOINTS
    def get(self, t):
        return tuple(dict.fromkeys(JOINTS, 0.) for _ in range(3))


def load_case(label):
    folder = SOURCES[label]
    trace = json.loads(read(folder/'rollout_policy_trace.json'))
    summary = json.loads(read(folder/'rollout_summary.json'))
    cfgpath = folder/'training_cfg.resolved.yaml'
    if not cfgpath.exists():
        cfgpath = checkpoint_config(summary)
    cfg = yaml.safe_load(read(cfgpath))['simulation']
    assert cfg['pros_ref_model'] == 'butterworth3_jerk_limited'
    assert summary['imitation_initialize_to_target'] is False
    limits = np.array([[cfg[f'pros_{j}_ref_{kind}_limit_{unit}'] for j in ('knee','ankle')]
                       for kind,unit in [('velocity','rad_s'),('acceleration','rad_s2'),('jerk','rad_s3')]])
    slew = np.array([cfg.get(f'pros_{j}_target_slew_rate_limit_rad_s',0.) for j in ('knee','ankle')])
    slew = np.where(slew > 0, slew, np.inf)
    bounds = np.array([[-1.5,0.],[-.7,.7]])
    starts = np.array([r['policy_segment_times'][0] for r in trace])
    ends = np.array([r['policy_segment_times'][-1] for r in trace])
    commands = np.array([r['policy_segment_values'][-1] for r in trace])
    actions = np.array([r['applied_policy_action'] for r in trace]).reshape(-1,2)
    previous = np.array(trace[0]['policy_segment_values'][0])
    for i in range(len(trace)):
        mapped = bounds[:,0] + .5*(actions[i]+1)*np.diff(bounds,axis=1).ravel()
        limited = previous + np.clip(mapped-previous, -slew*(ends[i]-starts[i]), slew*(ends[i]-starts[i]))
        np.testing.assert_allclose(limited, commands[i], atol=1e-12, rtol=0.)
        previous = limited
    return dict(label=label, folder=folder, trace=trace, cfg=cfg, limits=limits, slew=slew,
                bounds=bounds, starts=starts, ends=ends, commands=commands,
                initial_q=np.array(trace[0]['policy_segment_values'][0]),
                initial_v=np.array(trace[0]['policy_segment_derivatives'][0]),
                target_q=np.array([[r['imitation_target_q'][j] for j in JOINTS] for r in trace]),
                target_v=np.array([[r['imitation_target_qdot'][j] for j in JOINTS] for r in trace]))


def replay(case, commands=None, compare=False):
    lim=case['limits']; cfg=case['cfg']; commands=case['commands'] if commands is None else commands
    kin = Kinematics(Base(), JOINTS, ref_model=cfg['pros_ref_model'], ref_lpf_cutoff_hz=cfg['pros_ref_cutoff_hz'],
                     ref_governor_enable=cfg['pros_ref_governor'], ref_velocity_limits=lim[0],
                     ref_acceleration_limits=lim[1], ref_jerk_limits=lim[2], control_dt=.001)
    q=case['initial_q'].copy(); v=case['initial_v'].copy(); prev=q.copy()
    endpoint=[]; dense=[]; diagnostics=[]; sto_max=np.zeros(4)
    if compare:
        raw=read(case['folder']/'sim_outputs/rollout_episode_kinematics_reference.sto').split('endheader',1)[1].strip()
        names=raw.splitlines()[0].split(); data=np.loadtxt(io.StringIO(raw),skiprows=1)
        cols=[names.index(j+s) for s in ('_q_ref','_qdot_ref') for j in JOINTS]
    for i,(t0,t1,cmd) in enumerate(zip(case['starts'],case['ends'],commands)):
        cmd=prev+np.clip(cmd-prev,-case['slew']*(t1-t0),case['slew']*(t1-t0));prev=cmd.copy()
        kin.set_segment(np.array([t0,t1]),np.array([q,cmd]),np.array([v,(cmd-q)/(t1-t0)]))
        q,v,a=(np.array([d[j] for j in JOINTS]) for d in kin.get(t1))
        endpoint.append(np.r_[q,v,a]);diagnostics.append(kin.reference_governor_diagnostics)
        # Capture exact substep samples from the production BPoly.
        ts=np.linspace(t0,t1,max(2,int(np.ceil((t1-t0)/.001)))+1)[1:]
        dense.extend(np.c_[ts,kin._segment_spline(ts),kin._segment_spline.derivative(1)(ts),kin._segment_spline.derivative(2)(ts)])
        if compare:
            ix=(data[:,0]>=t0-1e-8)&(data[:,0]<t1-1e-8)
            tt=np.clip(data[ix,0],t0,t1)
            pred=np.c_[kin._segment_spline(tt),kin._segment_spline.derivative(1)(tt)]
            sto_max=np.maximum(sto_max,np.max(np.abs(pred-data[ix][:,cols]),axis=0))
    endpoint=np.array(endpoint);dense=np.array(dense)
    report={}
    if compare:
        recorded=np.array([[r['prosthetic_state'][j+s] for s in ('_served_ref','_served_ref_vel','_served_ref_accel') for j in JOINTS] for r in case['trace']])
        report={'trace_max_abs_error_q_v_a':np.max(abs(endpoint-recorded),axis=0).tolist(),
                'sto_max_abs_error_q_v':sto_max.tolist(),'action_mapping_verified':True,
                'reset_q':case['initial_q'].tolist(),'reset_v':case['initial_v'].tolist(),
                'governor_initial_acceleration':[0.,0.],'slew_enabled':np.isfinite(case['slew']).tolist()}
        assert max(sto_max)<1e-6
        assert np.max(abs(endpoint-recorded))<5e-5
    return endpoint,dense,diagnostics,report


def metrics(case,endpoint,dense,diagnostics):
    mask=case['ends']>=case['starts'][0]+.2
    def one(m):
        return {'q_rmse_rad':np.sqrt(np.mean((endpoint[m,:2]-case['target_q'][m])**2,axis=0)).tolist(),
                'qdot_rmse_rad_s':np.sqrt(np.mean((endpoint[m,2:4]-case['target_v'][m])**2,axis=0)).tolist(),
                'q_correlation':[float(np.corrcoef(endpoint[m,j],case['target_q'][m,j])[0,1]) for j in range(2)]}
    return {'whole':one(np.ones(len(mask),dtype=bool)),'after_0p2s':one(mask),
            'mean_jerk_limit_fraction':float(np.mean([d['jerk_limit_fraction'] for d in diagnostics])),
            'served_qdot_abs_max':np.max(abs(dense[:,3:5]),axis=0).tolist(),
            'served_qddot_rms':np.sqrt(np.mean(dense[:,5:7]**2,axis=0)).tolist(),
            'served_jerk_rms':np.sqrt(np.mean((np.diff(dense[:,5:7],axis=0)/np.diff(dense[:,0])[:,None])**2,axis=0)).tolist()}


def objective(command,case,j,gradient=True):
    """Exact production integration with forward sensitivities through clipping.

    Same dimensionless criterion for both sides and both joints. Target spans
    define units, not hand tuned subject/side weights. Piecewise gradients at
    clipping boundaries are one-sided; final output is checked in production.
    """
    count=len(command);V,A,J=case['limits'][:,j];w=2*np.pi*case['cfg']['pros_ref_cutoff_hz']
    q=float(case['initial_q'][j]);v=float(case['initial_v'][j]);a=0.
    dq=np.zeros(count);dv=np.zeros(count);da=np.zeros(count);grad=np.zeros(count)
    qscale=max(.05,float(np.ptp(case['target_q'][:,j])));vscale=max(.1,float(np.ptp(case['target_v'][:,j])))
    loss=0.; states=[]; jerkloss=0.; jgrad=np.zeros(count);steps=0
    # All audited arms have disabled target slew; fail closed if it changes.
    assert not np.isfinite(case['slew'][j])
    for k,(t0,t1,c) in enumerate(zip(case['starts'],case['ends'],command)):
        n=max(2,int(np.ceil((t1-t0)/.001)));dt=(t1-t0)/n
        for _ in range(n):
            rawj=w**3*(c-q)-2*w*w*v-2*w*a
            dj=-w**3*dq-2*w*w*dv-2*w*da;dj[k]+=w**3
            jerk=min(J,max(-J,rawj))
            if abs(rawj)>J:dj=np.zeros(count)
            lows=(-A,a-J*dt,2*(-V-v)/dt-a); highs=(A,a+J*dt,2*(V-v)/dt-a)
            li=int(np.argmax(lows));hi=int(np.argmin(highs));lo=lows[li];high=highs[hi]
            an=a+jerk*dt;dan=da+dj*dt
            if lo<=high:
                if an<lo:an=lo;dan=(np.zeros(count),da,-2*dv/dt-da)[li]
                elif an>high:an=high;dan=(np.zeros(count),da,-2*dv/dt-da)[hi]
            elif abs(an)>A:an=min(A,max(-A,an));dan=np.zeros(count)
            jerk=(an-a)/dt;dj=(dan-da)/dt
            vn=v+.5*(a+an)*dt;dvn=dv+.5*(da+dan)*dt
            if abs(vn)>V:vn=min(V,max(-V,vn));dvn=np.zeros(count)
            qn=q+v*dt+.5*a*dt*dt+jerk*dt**3/6
            dqn=dq+dv*dt+.5*da*dt*dt+dj*dt**3/6
            jerkloss+=(jerk/J)**2;jgrad+=2*jerk/J**2*dj;steps+=1
            q,v,a=qn,vn,an;dq,dv,da=dqn,dvn,dan
        eq=(q-case['target_q'][k,j])/qscale;ev=(v-case['target_v'][k,j])/vscale
        loss+=eq*eq+.1*ev*ev;grad+=2*eq/qscale*dq+.2*ev/vscale*dv
        states.append([q,v,a])
    loss/=count;grad/=count
    loss+=1e-4*jerkloss/steps;grad+=1e-4*jgrad/steps
    deltas=np.diff(np.r_[case['initial_q'][j],command])/np.diff(np.r_[case['starts'][0],case['ends']])/V
    loss+=1e-4*np.mean(deltas*deltas)
    d=2e-4/count*deltas/np.diff(np.r_[case['starts'][0],case['ends']])/V
    grad+=d;grad[:-1]-=d[1:]
    return loss,grad,np.array(states)


def main():
    start=time.monotonic();results={'protocol':{'weights':{'position':1.,'velocity':.1,'jerk':1e-4,'endpoint_rate':1e-4},
        'normalization':'target position/velocity peak-to-peak, floors .05 rad/.1 rad/s; physical J,V limits for regularity',
        'optimizer':'L-BFGS-B, maxiter 100, ftol 1e-10, gtol 1e-7, maxls 30; same for LEFT and RIGHT',
        'no_physical_simulation':True,'lookahead':'full recorded target sequence; offline diagnostic, not deployable policy'},'replay':{},'candidates':{}}
    cases={}
    for label in SOURCES:
        case=load_case(label);cases[label]=case
        ep,dense,diag,verification=replay(case,compare=True)
        results['replay'][label]={'verification':verification,'metrics':metrics(case,ep,dense,diag)}
        print('REPLAY',label,json.dumps(results['replay'][label]),flush=True)
    (HERE/'replay_metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    for label in ('LEFT','RIGHT'):
        case=cases[label];commands=case['target_q'].copy();solver=[]
        for j in range(2):
            # Derivative feedforward for a third order Butterworth is the same
            # analytic initialization for every target, and requires no fitting.
            w=2*np.pi*case['cfg']['pros_ref_cutoff_hz'];v=case['target_v'][:,j]
            a=np.gradient(v,case['ends']);jerk=np.gradient(a,case['ends'])
            initial=np.clip(case['target_q'][:,j]+2*v/w+2*a/w**2+jerk/w**3,*case['bounds'][j])
            initial_result=objective(initial,case,j)
            # Audit gradients with central finite differences at 5 fixed columns.
            errors=[]
            for k in np.linspace(0,len(initial)-1,5,dtype=int):
                hi=initial.copy();lo=initial.copy();hi[k]+=1e-6;lo[k]-=1e-6
                fd=(objective(hi,case,j)[0]-objective(lo,case,j)[0])/2e-6
                errors.append(abs(fd-initial_result[1][k]))
            assert max(errors)<1e-5,errors
            stamp=time.monotonic();best=[initial_result[0],initial.copy()]
            def fun(x):
                value,grad,_=objective(x,case,j)
                if value<best[0]:best[:]=[value,x.copy()]
                return value,grad
            def callback(x):
                if time.monotonic()-stamp>100:raise TimeoutError('100 second per-joint budget')
            try:
                fit=minimize(fun,initial,jac=True,bounds=[tuple(case['bounds'][j])]*len(initial),method='L-BFGS-B',
                    callback=callback,options={'maxiter':100,'ftol':1e-10,'gtol':1e-7,'maxls':30})
                status={'success':bool(fit.success),'message':str(fit.message),'iterations':fit.nit,'evaluations':fit.nfev}
            except TimeoutError as exc:status={'success':False,'message':str(exc)}
            commands[:,j]=best[1]
            status.update(initial_loss=initial_result[0],final_loss=best[0],gradient_max_abs_error=max(errors),wall_s=time.monotonic()-stamp)
            solver.append(status);print('FIT',label,j,status,flush=True)
        # float32 action is the exact exported contract for subsequent physical rollouts.
        actions=(2*(commands-case['bounds'][:,0])/np.diff(case['bounds'],axis=1).ravel()-1).astype(np.float32)
        commands=case['bounds'][:,0]+.5*(actions.astype(float)+1)*np.diff(case['bounds'],axis=1).ravel()
        ep,dense,diag,_=replay(case,commands)
        fast=np.stack([objective(commands[:,j],case,j)[2] for j in range(2)],axis=2)
        fast=fast.reshape(len(ep),6)
        discrepancy=float(np.max(abs(fast-ep)))
        assert discrepancy<1e-8,discrepancy
        np.savez_compressed(HERE/f'candidates_{label}.npz',time_start=case['starts'],time_end=case['ends'],
            raw_endpoint_rad=commands,normalized_action=actions,served_q=ep[:,:2],served_qdot=ep[:,2:4],served_qddot=ep[:,4:6],
            target_q=case['target_q'],target_qdot=case['target_v'],dense_time=dense[:,0],dense_q=dense[:,1:3],
            dense_qdot=dense[:,3:5],dense_qddot=dense[:,5:7],initial_q=case['initial_q'],initial_qdot=case['initial_v'])
        rows=[{'time_start':float(t0),'time_end':float(t1),'normalized_action':ac.tolist(),'raw_endpoint_rad':cmd.tolist()}
              for t0,t1,ac,cmd in zip(case['starts'],case['ends'],actions,commands)]
        (HERE/f'candidates_{label}.json').write_text(json.dumps(rows,indent=2)+'\n')
        results['candidates'][label]={'solver':solver,'production_replay_max_abs_error':discrepancy,'metrics':metrics(case,ep,dense,diag),
            'action_abs_max':np.max(abs(actions),axis=0).tolist(),'source':str(case['folder'].relative_to(REPO))}
        (HERE/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
        print('CANDIDATE',label,json.dumps(results['candidates'][label]),flush=True)
    results['wall_s']=time.monotonic()-start
    (HERE/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    (HERE/'input_hashes.json').write_text(json.dumps(HASHES,indent=2)+'\n')


if __name__=='__main__':main()
