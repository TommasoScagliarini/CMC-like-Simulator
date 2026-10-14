"""Additional offline audits and a formula-only feedforward comparator."""
import json
import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from analyze_governor import HERE, JOINTS, SOURCES, checkpoint_config, load_case, metrics, objective, replay

results={}
config_identity={}
derivative_checks={}
for label in ('LEFT','RIGHT'):
    c=load_case(label)
    summary=json.loads((SOURCES[label]/'rollout_summary.json').read_text())
    cfgpath=checkpoint_config(summary)
    archived=yaml.safe_load((SOURCES[label]/'training_cfg.resolved.yaml').read_text())
    current=yaml.safe_load(cfgpath.read_text())
    different=[k for k in set(archived)|set(current) if archived.get(k)!=current.get(k)]
    assert set(different)<= {'logging'},different
    config_identity[label]={'source_cfg_equals_checkpoint_cfg':archived==current,
                            'checkpoint_cfg_path':str(cfgpath),'different_top_level_sections':different}
    q_derivative=CubicSpline(c['ends'],c['target_q']).derivative()(c['ends'])
    mask=(c['ends']>=c['starts'][0]+.2)&(c['ends']<c['ends'][-1]-.02)
    derivative_checks[label]={'diagnostic_only':'Derivative of 100 Hz sampled q via CubicSpline; interpolation error and phase-boundary effects are not excluded.',
        'q_derivative_vs_recorded_target_qdot_rmse':np.sqrt(np.mean((q_derivative[mask]-c['target_v'][mask])**2,axis=0)).tolist(),
        'max_abs':np.max(abs(q_derivative[mask]-c['target_v'][mask]),axis=0).tolist()}
    ep,_,_,_=replay(c)
    fast=np.stack([objective(c['commands'][:,j],c,j)[2] for j in range(2)],axis=2).reshape(len(ep),6)
    discrepancy=float(np.max(abs(fast-ep)))
    assert discrepancy<1e-8,discrepancy
    gradient_errors=[]
    for j in range(2):
        cmd=c['commands'][:,j].copy(); value,grad,_=objective(cmd,c,j)
        for k in (2,70,199,310,470):
            hi=cmd.copy();lo=cmd.copy();hi[k]+=1e-7;lo[k]-=1e-7
            fd=(objective(hi,c,j)[0]-objective(lo,c,j)[0])/2e-7
            gradient_errors.append(abs(fd-grad[k]))
    assert max(gradient_errors)<1e-5,gradient_errors
    w=2*np.pi*c['cfg']['pros_ref_cutoff_hz'];v=c['target_v']
    a=np.gradient(v,c['ends'],axis=0);jerk=np.gradient(a,c['ends'],axis=0)
    commands=np.clip(c['target_q']+2*v/w+2*a/w**2+jerk/w**3,c['bounds'][:,0],c['bounds'][:,1])
    actions=(2*(commands-c['bounds'][:,0])/np.diff(c['bounds'],axis=1).ravel()-1).astype(np.float32)
    commands=c['bounds'][:,0]+.5*(actions.astype(float)+1)*np.diff(c['bounds'],axis=1).ravel()
    served,dense,diag,_=replay(c,commands)
    np.savez_compressed(HERE/f'analytic_{label}.npz',time_start=c['starts'],time_end=c['ends'],
        raw_endpoint_rad=commands,normalized_action=actions,served_q=served[:,:2],served_qdot=served[:,2:4],
        served_qddot=served[:,4:6],target_q=c['target_q'],target_qdot=c['target_v'],
        initial_q=c['initial_q'],initial_qdot=c['initial_v'])
    results[label]={'saturated_policy_fast_vs_production_max_abs':discrepancy,
                    'saturated_policy_gradient_max_abs_error':max(gradient_errors),
                    'formula_only_metrics':metrics(c,served,dense,diag),
                    'formula':'command=q+2*qdot/w+2*qddot/w^2+qjerk/w^3, w=2*pi*cutoff; numpy.gradient derivatives; bounds and float32 action applied',
                    'no_formula_parameters_fitted':True}
    print(label,json.dumps(results[label]),flush=True)
(HERE/'additional_checks.json').write_text(json.dumps(results,indent=2)+'\n')
(HERE/'config_identity.json').write_text(json.dumps(config_identity,indent=2)+'\n')
(HERE/'target_derivative_check.json').write_text(json.dumps(derivative_checks,indent=2)+'\n')
