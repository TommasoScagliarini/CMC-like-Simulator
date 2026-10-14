"""Exploratory paper-literal TNT parameter scan; no network, GRF, or dynamics.

This revisits the complete previously inspected prescribed recordings. Full-record
rankings are retrospective feasibility diagnostics, not held-out validation.
Only phase-transition arithmetic is accelerated; selected runs are cross-checked
against the existing full TNT replay, including its quintic inverse.
"""
from __future__ import annotations
import hashlib
import itertools
import json
from pathlib import Path
import sys
import numpy as np
from numba import njit
from scipy.signal import find_peaks

HERE=Path(__file__).resolve().parent
PREVIOUS=HERE.parents[1]/'tnt_preliminary_2026_10_13'
sys.path.insert(0,str(PREVIOUS))
import tnt_replay as old

@njit(cache=True)
def run_many(t, angle, velocity, params, nearest, nmin):
    # Output: invalid sample, completed, uniquely matched middle minima, spurious
    # middle transitions, longest dwell each of4states, max ratio, min/max rate,
    # min T_em-T_bm, starts, completed/matched by5temporal bins.
    out=np.zeros((len(params),24))
    for k in range(len(params)):
        sigma,rho,tem,v=params[k]
        state=0; entered=t[0]; T=0.; bm=0.; Tbm=0.; ms=0.; early=0.
        seen=np.zeros(nmin,np.int8)
        out[k,0]=-1; out[k,9]=v; out[k,10]=v; out[k,11]=tem
        for i in range(len(t)):
            dt=0. if i==0 else t[i]-t[i-1]
            now=t[i]; previous=state
            b=0 if now-t[0]<25 else (1 if now-t[0]<40 else (2 if now-t[0]<80 else (3 if now-t[0]<120 else 4)))
            if state==0:
                T=0.
                if angle[i]<sigma:
                    state=1; early=now; out[k,12]+=1
            elif state==1:
                T+=v*dt
                if velocity[i]>0:
                    bm=angle[i]; Tbm=T; ms=now; state=2
                    out[k,11]=min(out[k,11],tem-Tbm)
                    peak=nearest[i]
                    if peak>=0:
                        if seen[peak]==0:
                            seen[peak]=1; out[k,2]+=1; out[k,18+b]+=1
                    else: out[k,3]+=1
                    if not 0<=Tbm<tem:
                        out[k,0]=i; state=-1
            elif state==2:
                # Inverse quintic T never affects this state's transition guard.
                # It is evaluated by full replay for selected cross-checks.
                if angle[i]-bm>rho:
                    duration=now-ms
                    out[k,8]=max(out[k,8],(ms-early)/duration)
                    v=(tem-Tbm)/duration if duration>0 else -1.
                    out[k,9]=min(out[k,9],v); out[k,10]=max(out[k,10],v)
                    T=tem; state=3
                    if not np.isfinite(v) or v<=0:
                        out[k,0]=i; state=-1
            elif state==3:
                T+=v*dt
                if T>=1:
                    state=0; T=0.; out[k,1]+=1; out[k,13+b]+=1
            if previous>=0:
                out[k,4+previous]=max(out[k,4+previous],now-entered)
            if state!=previous: entered=now
            if state<0: break
        out[k,23]=state
    return out


def geometry(t,a,peaks):
    rows=[]
    for i,j in zip(peaks[:-1],peaks[1:]):
        local=a[i:j]; peak=i+int(np.argmax(local))
        rows.append(dict(minimum_time_s=float(t[i]),minimum_deg=float(a[i]),
                         next_minimum_time_s=float(t[j]),maximum_time_s=float(t[peak]),
                         excursion_deg=float(a[peak]-a[i]),period_s=float(t[j]-t[i])))
    return rows


def detail(t,theta,p,peaks):
    r=old.replay(t,theta,old.Parameters(*p))
    diagnostics=old.rate_diagnostics(r)
    rows=diagnostics.get('cycles', diagnostics) if isinstance(diagnostics,dict) else diagnostics
    # Keep original diagnostics shape verbatim, plus complete transition records.
    state_dwell={}
    for state,name in old.STATE_NAMES.items():
        edges=np.diff(np.r_[False,r['state']==state,False].astype(int))
        state_dwell[name]=float(max([t[min(b,len(t)-1)]-t[a] for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1))] or [0]))
    events=r['events']; invalid=np.flatnonzero(r['state']<0)
    perblock=[]
    bounds=[0,25,40,80,120,float(t[-1]-t[0])+.001]
    for lo,hi in zip(bounds[:-1],bounds[1:]):
        perblock.append(dict(relative_window_s=[lo,hi],independent_minima=int(np.sum((t[peaks]-t[0]>=lo)&(t[peaks]-t[0]<hi))),
                             completed=sum(e['event']=='late_to_stance' and lo<=e['t']-t[0]<hi for e in events)))
    return dict(parameters=dict(zip(['sigma_deg','rho_deg','t_em','initial_rate'],map(float,p))),
                invalid_time_s=float(t[invalid[0]]) if len(invalid) else None,
                completed=int(r['completed'][-1]),longest_state_dwell_s=state_dwell,
                temporal_blocks=perblock,rate_diagnostics=diagnostics,events=events)


def main():
    summary={'instruction_check_token':'CMC_AGENT_OK_2026','protocol':'Retrospective full-record feasibility scan. Previously seen recordings; NOT a new held-out validation. No target joint curves, GRF, network or dynamics.',
             'filter':'Same causal first-order6Hz filter and backward difference as first experiment',
             'grid':{'sigma_deg':[-10,8,.5],'rho_deg':[12,40,.5],'t_em':[.65,.70,.75,.80,.85,.90],'initial_rate_multipliers':[.5,1.,1.5]},'subjects':{}}
    for sub in ['ab06','ab07']:
        data=np.genfromtxt(PREVIOUS/f'{sub}_prescribed.csv',delimiter=',',names=True)
        t=data['time_s']; theta=data['thigh_deg']; a,v=old.causal_filter(t,theta)
        peaks=old.kinematic_minima(dict(t=t,theta=a))
        nearest=np.full(len(t),-1,dtype=np.int64)
        for k,i in enumerate(peaks):
            nearest[np.abs(t-t[i])<=.10]=k
        calibration_peaks=peaks[t[peaks]-t[0]<25]
        rate=1/(.45*np.median(np.diff(t[calibration_peaks])))
        params=np.asarray(list(itertools.product(np.arange(-10,8.01,.5),np.arange(12,40.01,.5),[.65,.70,.75,.80,.85,.90],[.5*rate,rate,1.5*rate])))
        out=run_many(t,a,v,params,nearest,len(peaks))
        print(sub,'scan configs',len(params),'completed',flush=True)
        valid=out[:,0]<0
        missed=len(peaks)-out[:,2]
        maxdwell=out[:,4:8].max(axis=1)
        # Ranking explicitly gives invalid runs lower priority; no joint scoring.
        rank=np.lexsort((out[:,8],maxdwell,out[:,3],missed,~valid))
        allcols=np.column_stack((params,out))
        names=['sigma_deg','rho_deg','t_em','initial_rate','invalid_sample','completed','matched_minima','spurious_middle','max_stance_s','max_early_s','max_middle_s','max_late_s','max_De_Dm','min_rate','max_rate','min_Tem_minus_Tbm','starts','completed_b0','completed_b1','completed_b2','completed_b3','completed_b4','matched_b0','matched_b1','matched_b2','matched_b3','matched_b4','final_state']
        np.savetxt(HERE/f'{sub}_grid.csv',allcols,delimiter=',',header=','.join(names),comments='',fmt='%.10g')
        geom=geometry(t,a,peaks)
        old.write_json(HERE/f'{sub}_cycle_geometry.json',geom)
        feasible=valid&(out[:,2]>=len(peaks)-1)&(out[:,3]==0)&(maxdwell<2.5)
        # Distinct sigma/rho/Tem triplets; choose nominal-rate first for detail.
        indices=[]; signatures=set()
        for idx in rank:
            sig=tuple(params[idx,:3])
            if sig in signatures: continue
            signatures.add(sig); indices.append(idx)
            if len(indices)==10: break
        best=[]
        for idx in indices:
            d=detail(t,theta,params[idx],peaks)
            # Event-only acceleration must reproduce literal engine exactly.
            assert d['completed']==int(out[idx,1])
            expected_invalid=None if out[idx,0]<0 else float(t[int(out[idx,0])])
            assert d['invalid_time_s']==expected_invalid
            d['scan_summary']=dict(zip(names[4:],map(float,out[idx])))
            best.append(d)
        old.write_json(HERE/f'{sub}_top10_details.json',best)
        # Separate first40s-only selection vs full replay. This is a diagnostic
        # resplit of known data, not a clean new independent test.
        cal_n=int(np.searchsorted(t,t[0]+40)); cal_peaks=peaks[peaks<cal_n]
        cal_nearest=nearest[:cal_n].copy(); cal_nearest[cal_nearest>=len(cal_peaks)]=-1
        cal_out=run_many(t[:cal_n],a[:cal_n],v[:cal_n],params,cal_nearest,len(cal_peaks))
        cal_valid=cal_out[:,0]<0
        cal_rank=np.lexsort((cal_out[:,8],cal_out[:,4:8].max(axis=1),cal_out[:,3],len(cal_peaks)-cal_out[:,2],~cal_valid))
        ci=int(cal_rank[0])
        cal_detail=detail(t,theta,params[ci],peaks)
        old.write_json(HERE/f'{sub}_first40_selected_details.json',cal_detail)
        # Include original paper thresholds and initial-prototype chosen values.
        previous=json.loads((PREVIOUS/'results.json').read_text())
        oldp=next(item['parameters'] for item in previous['results'] if item['subject'].lower()==sub)
        checks=[[-6,32,.8,rate],[oldp[x] for x in ['sigma_deg','rho_deg','t_em','initial_rate']]]
        check_out=run_many(t,a,v,np.asarray(checks),nearest,len(peaks))
        for k,p in enumerate(checks):
            d=detail(t,theta,p,peaks)
            assert d['completed']==int(check_out[k,1])
            assert d['invalid_time_s']==(None if check_out[k,0]<0 else float(t[int(check_out[k,0])]))
        s={'duration_s':float(t[-1]-t[0]),'independent_minima':len(peaks),'grid_count':len(params),'never_invalid_count':int(valid.sum()),'coverage_criterion':'>=Nmin-1 matched minima,0 spurious,neverINVALID,all state dwells<2.5s','coverage_feasible_count':int(feasible.sum()),
           'required_sigma_gt_max_minimum_deg':float(max(a[peaks])),'required_rho_lt_min_complete_excursion_deg':min(row['excursion_deg'] for row in geom),
           'best_full_record':{k:best[0][k] for k in ['parameters','invalid_time_s','completed','longest_state_dwell_s','temporal_blocks','scan_summary']},
           'first40_only_selection_applied_to_full_record':{k:cal_detail[k] for k in ['parameters','invalid_time_s','completed','longest_state_dwell_s','temporal_blocks']},
           'best_valid_matched_minima':int(max(out[valid,2])) if valid.any() else 0,
           'matched_all_minima_and_valid':int(np.sum(valid&(out[:,2]==len(peaks)))),'parity_checks':'12 full replay checks passed (top10 +paper +oldwinner)',
           'source_sha256':hashlib.sha256((PREVIOUS/f'{sub}_prescribed.csv').read_bytes()).hexdigest()}
        summary['subjects'][sub]=s
        old.write_json(HERE/'summary.json',summary)
        print(json.dumps(old.json_safe(s),indent=2),flush=True)
    old.write_json(HERE/'summary.json',summary)

if __name__=='__main__': main()
