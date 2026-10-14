"""Reconstruct Gaussian parameters on recorded observations, without simulation."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import torch
from scipy.special import ndtr

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
sys.path.insert(0,str(PACKAGE.parent))
from ray.rllib.core.rl_module.rl_module import RLModule
spec=importlib.util.spec_from_file_location('verification_analysis',HERE/'analyze_results.py')
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)
torch.set_num_threads(1)

def checkpoint(summary):
    raw=Path(summary['checkpoint'])
    for candidate in (raw,ROOT/raw,PACKAGE.parent/raw,PACKAGE/raw):
        if candidate.is_dir():return candidate.resolve()
    raise FileNotFoundError(raw)

def logits(module,obs):
    with torch.no_grad():
        return module.forward_inference({'obs':torch.as_tensor(obs,dtype=torch.float32)})['action_dist_inputs'].cpu().numpy()

def main():
    folders=analysis.sources()
    baseline_trace=json.loads((folders['cp90_visible']/'rollout_policy_trace.json').read_text())
    baseline_summary=json.loads((folders['cp90_visible']/'rollout_summary.json').read_text())
    base=RLModule.from_checkpoint(checkpoint(baseline_summary))
    baseobs=np.asarray([r['actor_observation_vector_before'] for r in baseline_trace],np.float32)
    probes=baseobs[::25].copy();hidden=probes.copy();hidden[:,[0,1,35,36,37,38]]=0
    probes=np.concatenate([probes,hidden]);baseprobes=logits(base,probes)
    result={};hashes={}
    for label,folder in folders.items():
        trace=json.loads((folder/'rollout_policy_trace.json').read_text())
        summary=json.loads((folder/'rollout_summary.json').read_text())
        path=checkpoint(summary);module=RLModule.from_checkpoint(path)
        obs=np.asarray([r['actor_observation_vector_before'] for r in trace],np.float32)
        output=logits(module,obs);raw=np.asarray([r['raw_policy_action'] for r in trace])
        np.testing.assert_allclose(output[:,:2],raw,atol=2e-5,rtol=1e-5)
        sigma=np.exp(output[:,2:]);fixed=logits(module,probes)
        mean=output[:,:2].astype(np.float64)
        sd=sigma.astype(np.float64)
        low=np.asarray(module.action_space.low).reshape(1,2)
        high=np.asarray(module.action_space.high).reshape(1,2)
        clipped=np.clip(mean,low,high)
        applied=np.asarray([r['applied_policy_action'] for r in trace]).reshape(-1,2)
        np.testing.assert_allclose(clipped,applied,atol=2e-5,rtol=1e-5)
        endpoints=np.asarray([r['policy_segment_values'][-1] for r in trace])
        # CMCEnvConfig.absolute_bounds_rad; validate the mapping on every row.
        command_low=np.array([[-1.5,-.7]])
        command_high=np.array([[0.,.7]])
        scale=(command_high-command_low)/(high-low)
        np.testing.assert_allclose(command_low+(applied-low)*scale,endpoints,atol=2e-5,rtol=1e-5)
        alpha=(low-mean)/sd;beta=(high-mean)/sd
        density=lambda z: np.exp(-.5*z*z)/np.sqrt(2*np.pi)
        expected=low*ndtr(alpha)+mean*(ndtr(beta)-ndtr(alpha))+sd*(density(alpha)-density(beta))+high*ndtr(-beta)
        clip_probability=ndtr(alpha)+ndtr(-beta)
        record={'steps':len(trace),'checkpoint':str(path.relative_to(ROOT)),
            'module_class':type(module).__name__,
            'max_abs_recomputed_action_error':float(np.max(np.abs(output[:,:2]-raw))),
            'sigma_on_recorded_states':{'percentile_order':[0,50,90,95,99,100],
                'knee_ankle':np.percentile(sigma,[0,50,90,95,99,100],axis=0).tolist(),
                'mean_knee_ankle':sigma.mean(axis=0).tolist()},
            'fixed_probes_logstd_max_delta_vs_cp90':float(np.max(np.abs(fixed[:,2:]-baseprobes[:,2:]))),
            'fixed_probes_mean_sigma_knee_ankle':np.exp(fixed[:,2:]).mean(axis=0).tolist(),
            'conditional_clipping_audit':{'normalized_action_bounds':[low.flatten().tolist(),high.flatten().tolist()],
                'endpoint_bounds_rad':[command_low.flatten().tolist(),command_high.flatten().tolist()],
                'mean_clipping_probability_knee_ankle':clip_probability.mean(axis=0).tolist(),
                'rms_expected_clipped_sample_minus_clipped_mean_deg':np.degrees(np.sqrt(np.mean(((expected-clipped)*scale)**2,axis=0))).tolist(),
                'mean_expected_clipped_sample_minus_clipped_mean_deg':np.degrees(np.mean((expected-clipped)*scale,axis=0)).tolist(),
                'method':'Analytic expectation of a Gaussian clipped to the actual Box bounds, holding each recorded observation fixed. Applied actions and segment endpoints verified against these bounds.',
                'limit':'This is not a stochastic simulation or a prediction of its trajectory. State feedback, governor and contact remain nonlinear.'}}
        if label.startswith('frozen_cp90_'):
            cp90_same=logits(base,obs)
            np.testing.assert_array_equal(output[:,2:],cp90_same[:,2:])
            record['sigma_exact_cp90_on_every_recorded_state']=True
            assert record['fixed_probes_logstd_max_delta_vs_cp90']==0
        result[label]=record
        for file in (folder/'rollout_policy_trace.json',path/'module_state.pkl',PACKAGE.parent/'osim_trj_cmc_like.py'):
            hashes[str(file.relative_to(ROOT))]=hashlib.sha256(file.read_bytes()).hexdigest()
        print(label,record['fixed_probes_logstd_max_delta_vs_cp90'],flush=True)
    (HERE/'rollout_distributions.json').write_text(json.dumps({'rollouts':result,
        'interpretation':'Raw Gaussian sigma in policy-action coordinates. Deterministic rollouts use its mean; sigma here describes the distribution at visited states, not noise applied during evaluation.',
        'fixed_probes':'20 CP90 visible observations plus the same 20 observations with all teacher inputs zeroed.'},indent=2)+'\n')
    hashes[str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (HERE/'distribution_input_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')

if __name__=='__main__':main()
