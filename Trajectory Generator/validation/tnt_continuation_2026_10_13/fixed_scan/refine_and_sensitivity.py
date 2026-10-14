"""Refine AB06 narrow geometry-compatible boundary and inspect fixed-best sensitivity.
All analyses remain exploratory reuse of previously examined recordings.
"""
from pathlib import Path
import itertools,json
import numpy as np
import scan_fixed as s

def prepare(sub):
 d=np.genfromtxt(s.PREVIOUS/f'{sub}_prescribed.csv',delimiter=',',names=True)
 t=d['time_s']; theta=d['thigh_deg']; a,v=s.old.causal_filter(t,theta)
 peaks=s.old.kinematic_minima(dict(t=t,theta=a)); nearest=np.full(len(t),-1,dtype=np.int64)
 for k,i in enumerate(peaks): nearest[np.abs(t-t[i])<=.1]=k
 return t,theta,a,v,peaks,nearest

summary=json.loads((s.HERE/'summary.json').read_text()); result={'scope':'Exploratory finer grid and initialization sensitivity; not independent heldout validation','subjects':{}}
for sub in ['ab06','ab07']:
 t,theta,a,v,peaks,nearest=prepare(sub)
 p=summary['subjects'][sub]['best_full_record']['parameters']; p=list(p.values())
 rates=np.array([.10,.25,.50,p[3],1.,1.4,2.,2.8,4.,6.])
 params=np.array([p[:3]+[r] for r in rates]);out=s.run_many(t,a,v,params,nearest,len(peaks))
 rows=[]
 for par,r in zip(params,out):
  rows.append({'parameters':par.tolist(),'invalid_time_s':None if r[0]<0 else float(t[int(r[0])]),'completed':int(r[1]),'matched_minima':int(r[2]),'longest_state_dwell_s':r[4:8].tolist(),'max_De_Dm':float(r[8])})
 details=json.loads((s.HERE/f'{sub}_top10_details.json').read_text())[0]
 ratio=np.array([r['early_to_middle_duration_ratio'] for r in details['rate_diagnostics'] if 'early_to_middle_duration_ratio' in r])
 # Product of local recurrence sensitivities under fixed state timings.
 logs=np.log(np.maximum(ratio,1e-12)); maxlog=0; accumulator=0
 for x in logs:
  accumulator=max(0,accumulator+x);maxlog=max(maxlog,accumulator)
 fixed=summary['subjects'][sub];grid=np.genfromtxt(s.HERE/f'{sub}_grid.csv',delimiter=',',names=True)
 geom=(grid['sigma_deg']>fixed['required_sigma_gt_max_minimum_deg'])&(grid['rho_deg']<fixed['required_rho_lt_min_complete_excursion_deg'])
 geomout={'count':int(geom.sum()),'invalid_count':int(np.sum(geom&(grid['invalid_sample']>=0))),'max_matched_minima_among_valid':int(max(grid['matched_minima'][geom&(grid['invalid_sample']<0)]))}
 result['subjects'][sub]={'initial_rate_sensitivity':rows,'local_De_Dm':{'count':len(ratio),'quantiles_0_25_50_75_95_100':np.quantile(ratio,[0,.25,.5,.75,.95,1]).tolist(),'fraction_gt1':float(np.mean(ratio>1)),'max_contiguous_product':float(np.exp(maxlog)),'product_interpretation':'Conditional sensitivity to previous rate assuming event timings unchanged; not global nonlinear stability proof'},'geometry_compatible_broad_grid':geomout}
 if sub=='ab06':
  params=np.array(list(itertools.product(np.round(np.arange(-.5,1.001,.1),8),np.round(np.arange(21.5,23.651,.05),8),[.60,.65,.70,.75,.80,.85,.90,.95],[.1,.5,1.,1.5,2.])))
  out=s.run_many(t,a,v,params,nearest,len(peaks));valid=out[:,0]<0;rank=np.lexsort((out[:,4:8].max(axis=1),out[:,3],len(peaks)-out[:,2],~valid));i=int(rank[0])
  result['subjects'][sub]['fine_grid']={'sigma_range_step':[-.5,1,.1],'rho_range_step':[21.5,23.65,.05],'Tem':[.60,.65,.70,.75,.80,.85,.90,.95],'initial_rates':[.1,.5,1.,1.5,2.],'count':len(params),'invalid_count':int(np.sum(~valid)),'valid_count':int(valid.sum()),'best_parameters':params[i].tolist(),'best_matched':int(out[i,2]),'best_completed':int(out[i,1]),'best_invalid_time_s':None if out[i,0]<0 else float(t[int(out[i,0])]),'best_state_dwell_s':out[i,4:8].tolist(),'all_minima_matched_and_valid':int(np.sum(valid&(out[:,2]==len(peaks))))}
  np.savez_compressed(s.HERE/'ab06_fine_grid.npz',parameters=params,metrics=out)
 s.old.write_json(s.HERE/'sensitivity.json',result)
 print(sub,json.dumps(s.old.json_safe(result['subjects'][sub]),indent=2),flush=True)
