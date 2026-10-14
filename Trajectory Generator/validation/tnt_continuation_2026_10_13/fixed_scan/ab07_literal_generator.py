"""Exploratory AB07 fixed-parameter literal TNT trajectory generator, no training.

Parameters were selected using the previously inspected complete thigh recording.
Only joint templates are built from first40s. Thus later-window scores are NOT an
independent held-out test of the parameter-calibration procedure.
"""
from __future__ import annotations
import json
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
CONT=HERE.parent
PREVIOUS=CONT.parent/'tnt_preliminary_2026_10_13'
ROOT=CONT.parents[2]
sys.path.insert(0,str(CONT));sys.path.insert(0,str(PREVIOUS))
import tnt_replay as prior
from kinematic_generator import stream_replay, KinematicTNTController


def diagnostic(r,clean_minima_t):
 t=r['t'];events=r['events']; dwell={}; runs=[]
 for state,name in prior.STATE_NAMES.items():
  edges=np.diff(np.r_[False,r['state']==state,False].astype(int))
  ds=[]
  for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
   dt=float(t[min(b,len(t)-1)]-t[a]);ds.append(dt)
   if state>=0 and dt>2.5:runs.append({'state':name,'start_s':float(t[a]),'duration_s':dt})
  dwell[name]=max(ds,default=0.)
 matched=[];spurious=0
 for e in events:
  if e['event']!='early_to_middle':continue
  k=int(np.argmin(abs(clean_minima_t-e['t'])))
  if abs(clean_minima_t[k]-e['t'])<=.10:matched.append(k)
  else:spurious+=1
 invalid=np.flatnonzero(r['state']<0)
 ratios=[x['early_to_middle_duration_ratio'] for x in prior.rate_diagnostics(r) if 'early_to_middle_duration_ratio' in x]
 return {'completed':int(r['completed'][-1]),'clean_independent_minima':len(clean_minima_t),'matched_clean_minima':len(set(matched)),'duplicate_matched_middle_events':len(matched)-len(set(matched)),'spurious_middle_events':spurious,
         'invalid_samples':int(np.sum(r['state']<0)),'first_invalid_s':float(t[invalid[0]]) if len(invalid) else None,'longest_state_dwell_s':dwell,'stall_criterion_s':2.5,'stalls':runs,
         'rate_min_max':[float(r['rate'].min()),float(r['rate'].max())],'max_De_Dm':max(ratios,default=None)}


def main():
 previous=json.loads((PREVIOUS/'results.json').read_text())
 old=next(s for s in previous['results'] if s['subject']=='AB07')
 data=np.genfromtxt(PREVIOUS/'ab07_prescribed.csv',delimiter=',',names=True)
 t=data['time_s'];theta=data['thigh_deg'];q=np.c_[data['knee_deg'],data['ankle_deg']]
 p=prior.Parameters(-3.,28.,.70,float(old['parameters']['initial_rate']))
 stance=float(old['calibration_geometry']['initial_stance_duration_s'])
 cal=t<t[0]+40;later=~cal
 clean=stream_replay(t,theta,p,method='literal',initial_stance_duration=stance)
 template=prior.make_template(clean,q,cal);prediction=prior.predict(clean,template)
 minima=prior.kinematic_minima(clean);clean_minima_t=t[minima]
 # Confirm full streaming state/phase matches the previous literal implementation.
 oldr=prior.replay(t,theta,p,initial_stance_duration=stance)
 for field in ['state','T','rate','stance_progress','completed']:
  np.testing.assert_allclose(clean[field],oldr[field],atol=1e-12,rtol=0)
 bundle={'parameters':vars(p),'initial_stance_duration_s':stance,'rate_method':'literal','rate_alpha':.25,'filter_cutoff_hz':6.,'template':template}
 controller=KinematicTNTController(bundle,t[0],theta[0]); online=np.array([controller.step(now,angle) for now,angle in zip(t,theta)])
 np.testing.assert_allclose(online,prediction,atol=1e-11,rtol=0)
 result={'scope':'EXPLORATORY: parameters selected on previously seen complete thigh recording; only curve templates use first40s. Later-window scores are NOT independent holdout validation of calibration.',
         'method':'Paper-literal TNT rate with separate previous-stance timer and fixed PCHIP joint curves. No training, GRF, dynamics, or online target fitting.',
         'parameters':vars(p),'initial_stance_duration_s':stance,'curve_calibration_window_s':[float(t[0]),float(t[0]+40)],'later_score_window_s':[float(t[np.flatnonzero(later)[0]]),float(t[-1])],
         'template':template,'checks':{'legacy_phase_replay_parity':True,'streaming_reference_parity_max_error_deg':float(np.max(abs(online-prediction)))},'cases':{}}
 signals={'clean':theta}
 for seed in [11,23,47]:signals[f'noise025deg_seed{seed}']=theta+np.random.default_rng(seed).normal(0,.25,len(theta))
 signals['bias_minus2deg']=theta-2.;signals['bias_plus2deg']=theta+2.
 for name,signal in signals.items():
  r=stream_replay(t,signal,p,method='literal',initial_stance_duration=stance)
  pred=prior.predict(r,template)
  result['cases'][name]={'diagnostics':diagnostic(r,clean_minima_t),'all_recording':prior.score(r,q,pred,np.ones(len(t),dtype=bool)),
                        'after_first40s':prior.score(r,q,pred,later)}
 duration=float(np.median(prior.duration_stats(clean,cal)['swing']))
 pfd=stream_replay(t,theta,p,pfd_duration=duration,initial_stance_duration=stance)
 pfdtemplate=prior.make_template(pfd,q,cal);pfdpred=prior.predict(pfd,pfdtemplate)
 result['fixed_duration_comparator']={'scope':'Same sigma, same first40s template fitting, own phase-indexed curves; not identical waveform under different clocks.', 'swing_duration_s':duration,
                                      'all_recording':prior.score(pfd,q,pfdpred,np.ones(len(t),dtype=bool)),'after_first40s':prior.score(pfd,q,pfdpred,later)}
 np.savetxt(HERE/'ab07_literal_generator_trace.csv',np.c_[t,theta,clean['state'],clean['T'],clean['rate'],clean['stance_progress'],q,prediction,pfdpred],delimiter=',',header='time_s,thigh_deg,state,T,rate,stance_progress,knee_target_deg,ankle_target_deg,knee_tnt_deg,ankle_tnt_deg,knee_pfd_deg,ankle_pfd_deg',comments='')
 prior.write_json(HERE/'ab07_literal_generator.json',result)
 # One visual with full rates and a fixed post-calibration excerpt.
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(4,1,figsize=(13,10),constrained_layout=True)
 axes[0].plot(t,clean['rate'],label='TNT literal rate');axes[0].axvline(t[0]+40,color='gray',ls=':',label='Curve calibration ends');axes[0].set_ylabel('Rate [1/s]');axes[0].legend()
 m=(t>=t[0]+60)&(t<t[0]+65)
 axes[1].plot(t[m],clean['T'][m],label='TNT swing progress');axes[1].plot(t[m],np.where(clean['state'][m]==0,clean['stance_progress'][m],np.nan),label='Stance timer extension');axes[1].set_ylabel('Progress');axes[1].legend()
 for j,joint in enumerate(['Knee','Ankle']):
  ax=axes[j+2];ax.plot(t[m],q[m,j],color='black',label='Prescribed');ax.plot(t[m],prediction[m,j],label='Literal TNT fixed curves');ax.plot(t[m],pfdpred[m,j],label='Fixed duration comparator',alpha=.8);ax.set_ylabel(joint+' [deg]');ax.legend(ncol=3,fontsize=8)
 for ax in axes:ax.grid(alpha=.25);ax.set_xlabel('Recorded time [s]')
 fig.suptitle('AB07: literal TNT, fixed curves, no training\nExploratory full-record parameter choice; joint curves fit first40s only',fontsize=12)
 plot=ROOT/'plot/tnt_continuation_2026_10_13/AB07_literal_generator.png';plot.parent.mkdir(parents=True,exist_ok=True);fig.savefig(plot,dpi=150);plt.close(fig)
 for name,row in result['cases'].items():
  print(name,json.dumps(prior.json_safe({'diagnostics':row['diagnostics'],'later_errors':row['after_first40s']['all']})),flush=True)
 print('PFDlater',json.dumps(prior.json_safe(result['fixed_duration_comparator']['after_first40s']['all'])))

if __name__=='__main__':main()
