"""Compare marker residuals on existing saved IK; no IK solve or integration."""
from pathlib import Path
import hashlib
import json
import re
import sys
from collections import Counter

import numpy as np
import opensim as osim

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from tools.calibrate_ab06_seasea_markers import parse_trc, parse_motion, load_plugin

HERE = Path(__file__).resolve().parent
PLUGIN = REPO / 'plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib'
TRANSLATION = {'pelvis_tx', 'pelvis_ty', 'pelvis_tz'}


def fingerprint(p):
    return {'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}


def stats(x):
    return {'mean': float(np.mean(x)), 'median': float(np.median(x)),
            'p95': float(np.percentile(x,95)), 'p99': float(np.percentile(x,99)),
            'max': float(np.max(x))}


def intervals(mask, t):
    padded = np.r_[False, mask, False]
    starts = np.flatnonzero(np.diff(padded.astype(int)) == 1)
    ends = np.flatnonzero(np.diff(padded.astype(int)) == -1)-1
    dt = float(np.median(np.diff(t)))
    runs = [{'start_s': float(t[a]), 'end_s': float(t[b]),
             'frames': int(b-a+1), 'duration_s': float((b-a+1)*dt)}
            for a,b in zip(starts,ends)]
    return {'count': int(mask.sum()), 'fraction': float(mask.mean()),
            'time_s': float(mask.sum()*dt), 'interval_count': len(runs),
            'longest': sorted(runs,key=lambda x:x['frames'],reverse=True)[:12]}


def log_rows(p):
    pattern = re.compile(r'Frame (\d+) \(t = ([\d.eE+-]+)\):\s+total squared error = ([\d.eE+-]+), marker error: RMS = ([\d.eE+-]+), max = ([\d.eE+-]+) \(([^)]+)\)')
    return [{'frame':int(m[1]), 't':float(m[2]), 'sum_sq':float(m[3]),
             'rms':float(m[4]), 'max':float(m[5]), 'marker':m[6]}
            for m in pattern.finditer(p.read_text())]


def evaluate(subject):
    model_p=REPO/f'models/{subject}_SEASEA_Threadmill/{subject}_SEASEA_stiff321_500_pi.osim'
    mot_p=REPO/f'models/{subject}_SEASEA_Threadmill/data/IK_results_{subject}_SEASEA.mot'
    trc_p=REPO/f'models/{subject}_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc'
    model=osim.Model(str(model_p)); state=model.initSystem()
    trc=parse_trc(trc_p); motion=parse_motion(mot_p)
    if not np.allclose(trc.times,motion.times,rtol=0,atol=1e-8):
        raise ValueError('Clock mismatch; no implicit resampling allowed')
    markers=[model.getMarkerSet().get(n) for n in trc.markers]
    names=list(trc.markers)
    q=motion.values[:,1:].copy()
    coords=[model.getCoordinateSet().get(n) for n in motion.labels[1:]]
    if motion.in_degrees:
        for j,name in enumerate(motion.labels[1:]):
            if name not in TRANSLATION: q[:,j]=np.deg2rad(q[:,j])
    measured=np.stack([trc.markers[n] for n in names],axis=1)
    residual=np.empty_like(measured)
    for i,row in enumerate(q):
        for coord,v in zip(coords,row): coord.setValue(state,float(v),False)
        model.realizePosition(state)
        for j,marker in enumerate(markers):
            xyz=marker.getLocationInGround(state)
            residual[i,j]=[xyz.get(k)-measured[i,j,k] for k in range(3)]
        if i%10000 == 0: print(subject,i,flush=True)
    err=np.linalg.norm(residual,axis=2)
    rms=np.sqrt(np.mean(err**2,axis=1)); maximum=err.max(axis=1)
    groups={'left_prosthetic':[j for j,n in enumerate(names) if n.startswith('L_') and n not in ['L_ASIS','L_PSIS']],
            'right_biological':[j for j,n in enumerate(names) if n.startswith('R_') and n not in ['R_ASIS','R_PSIS']],
            'pelvis':[j for j,n in enumerate(names) if n.endswith('ASIS') or n.endswith('PSIS')]}
    result={'sources':{'model':fingerprint(model_p),'ik':fingerprint(mot_p),'trc':fingerprint(trc_p)},
            'rows':len(q),'time_range_s':trc.times[[0,-1]].tolist(),'finite':bool(np.isfinite(err).all()),
            'saved_pose_global_rms_m':stats(rms),'saved_pose_max_marker_error_m':stats(maximum),
            'thresholds_max_marker':{str(v):intervals(maximum>v,trc.times) for v in [.02,.03,.05,.10]},
            'thresholds_global_rms':{str(v):intervals(rms>v,trc.times) for v in [.01,.015,.02,.03]},
            'groups':{g:{'mean_frame_rms_m':float(np.mean(np.sqrt(np.mean(err[:,js]**2,axis=1)))),
                          'pooled_rms_m':float(np.sqrt(np.mean(err[:,js]**2))),
                          'markers':len(js)} for g,js in groups.items()},
            'per_marker':{n:{'error_m':stats(err[:,j]),'rms_m':float(np.sqrt(np.mean(err[:,j]**2))),
                             'mean_residual_vector_m':np.mean(residual[:,j],axis=0).tolist(),
                             'peak_time_s':float(trc.times[np.argmax(err[:,j])]),
                             'above_30mm':intervals(err[:,j]>.03,trc.times),
                             'above_50mm':intervals(err[:,j]>.05,trc.times)} for j,n in enumerate(names)}}
    if subject=='AB08':
        log_p=REPO/'results/ab08_seasea_treadmill_treadmill_01_01_ik/02_ik/ik_full.log'
        rows=log_rows(log_p)
        result['native_log']={'file':fingerprint(log_p),'rows':len(rows),
            'agreement_saved_pose_rms_max_abs_m':float(np.max(np.abs(rms-np.array([r['rms'] for r in rows])))),
            'agreement_saved_pose_max_marker_abs_m':float(np.max(np.abs(maximum-np.array([r['max'] for r in rows])))),
            'dominant_counts':dict(Counter(r['marker'] for r in rows)),
            'dominant_above_30mm_counts':dict(Counter(r['marker'] for r in rows if r['max']>.03)),
            'dominant_above_50mm_counts':dict(Counter(r['marker'] for r in rows if r['max']>.05)),
            'top_10_max_frames':sorted(rows,key=lambda r:r['max'],reverse=True)[:10]}
        peak=int(np.argmax(maximum)); focused=[i for i in range(len(q)) if abs(trc.times[i]-trc.times[peak]) <= .15]
        result['peak_region']={'frame_index':peak,'time_s':float(trc.times[peak]),
          'marker_errors_at_peak_m':dict(zip(names,err[peak].tolist())),
          'raw_markers_finite':{n:bool(np.isfinite(x).all()) for n,x in trc.markers.items()},
          'rows':[{'t':float(trc.times[i]),'upper_raw_ground_m':trc.markers['R_Thigh_Upper'][i].tolist(),
                   'upper_error_m':float(err[i,names.index('R_Thigh_Upper')]),
                   'max_marker_error_m':float(maximum[i]),'rms_m':float(rms[i]),
                   'q':dict(zip(motion.labels[1:],motion.values[i,1:].tolist()))} for i in focused]}
        result['raw_step_norm_m']={n:stats(np.linalg.norm(np.diff(x,axis=0),axis=1)) for n,x in trc.markers.items()}
    np.savez_compressed(HERE/f'ik_error_analysis_{subject.lower()}_residuals.npz',
                        times=trc.times,names=np.array(names),residual_ground_m=residual,
                        error_m=err,global_rms_m=rms,max_marker_error_m=maximum)
    return result


def main():
    HERE.mkdir(parents=True,exist_ok=True)
    osim.Logger.setLevelString('error')
    load_plugin(PLUGIN)
    result={'instruction_check_token':'CMC_AGENT_OK_2026',
            'scope':'Native forward kinematics on existing saved IK only; no solver, dynamics or policy.',
            'subjects':{}}
    for s in ['AB06','AB07','AB08']:
        result['subjects'][s]=evaluate(s)
        (HERE/'ik_error_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print('complete',flush=True)


if __name__=='__main__': main()
