"""Recover proper foot poses from saved contact centers; algebra only, no FK.

The frozen eight locations are planar (rank2), not rank3. Two independent local
directions plus det(R)=+1 identify a unique proper rigid transform. Duplicate
points do not create extra independent directions. No OpenSim import occurs.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

AUDIT=Path(__file__).resolve().parent
ROOT=AUDIT.parents[2]
PREVIOUS=ROOT/'validation/ab06_ab07_preparation_vs_model_2026_10_07'
PAIR=ROOT/'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
PROFILE=ROOT/'online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json'
INTEGRATE=np.trapezoid if hasattr(np,'trapezoid') else np.trapz
N_BINS=20


def fp(path):
    path=Path(path).resolve()
    return {'path':path.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def recover(local,centers):
    x=local-local.mean(axis=0)
    y=centers-centers.mean(axis=1,keepdims=True)
    singular=np.linalg.svd(x,compute_uv=False)
    tolerance=max(x.shape)*np.finfo(float).eps*singular[0]
    rank=int(np.count_nonzero(singular>tolerance))
    if rank<2:
        raise AssertionError('Proper rotation not identifiable: locations are collinear or coincident')
    covariance=np.einsum('ni,tnj->tij',x,y)
    u,s,vh=np.linalg.svd(covariance)
    vu=np.matmul(vh.transpose(0,2,1),u.transpose(0,2,1))
    det_before=np.linalg.det(vu)
    correction=np.broadcast_to(np.eye(3),(len(centers),3,3)).copy()
    correction[:,2,2]=np.where(det_before<0,-1.0,1.0)
    rotation=np.matmul(np.matmul(vh.transpose(0,2,1),correction),u.transpose(0,2,1))
    origin=centers.mean(axis=1)-np.einsum('tij,j->ti',rotation,local.mean(axis=0))
    reconstructed=np.einsum('tij,nj->tni',rotation,local)+origin[:,None,:]
    error=float(np.max(np.abs(reconstructed-centers)))
    orth=float(np.max(np.abs(np.matmul(rotation.transpose(0,2,1),rotation)-np.eye(3))))
    determinant=np.linalg.det(rotation)
    if error>1e-12 or orth>1e-12 or np.max(np.abs(determinant-1))>1e-12:
        raise AssertionError('Proper rigid recovery numerical contract failed')
    world_singular=np.linalg.svd(y,compute_uv=False)
    world_rank=np.count_nonzero(world_singular>tolerance,axis=1)
    if np.any(world_rank!=rank):
        raise AssertionError('Saved centers do not preserve local affine rank')
    # Check orientation independently using ordered non-collinear edges.
    indices=None
    for i in range(1,len(local)):
        for j in range(i+1,len(local)):
            normal=np.cross(local[i]-local[0],local[j]-local[0])
            if np.linalg.norm(normal)>1e-8:
                indices=(0,i,j);break
        if indices: break
    i0,i1,i2=indices
    normal=np.cross(local[i1]-local[i0],local[i2]-local[i0]);normal/=np.linalg.norm(normal)
    native_normal=np.cross(centers[:,i1]-centers[:,i0],centers[:,i2]-centers[:,i0])
    native_normal/=np.linalg.norm(native_normal,axis=1)[:,None]
    predicted_normal=np.einsum('tij,j->ti',rotation,normal)
    handedness_error=float(np.max(np.abs(native_normal-predicted_normal)))
    if handedness_error>1e-12:
        raise AssertionError('Ordered-edge handedness differs')
    return rotation,origin,{'status':'PASS_rank2_or_higher_proper_rigid_recovery',
        'local_centered_singular_values_m':singular.tolist(),'rank_tolerance_m':float(tolerance),
        'local_affine_rank':rank,'rank3':rank==3,'world_affine_rank_minimum':int(world_rank.min()),
        'world_affine_rank_maximum':int(world_rank.max()),
        'nonzero_condition_number':float(singular[0]/singular[rank-1]),
        'proper_rotation_reconstruction_max_abs_error_m':error,
        'orthogonality_max_abs_error':orth,'determinant_minimum':float(determinant.min()),
        'determinant_maximum':float(determinant.max()),
        'pre_correction_improper_SVD_count':int(np.count_nonzero(det_before<0)),
        'ordered_edge_normal_handedness_max_abs_error':handedness_error,
        'identifying_edge_indices':list(indices),
        'rank_interpretation':'Rank2 non-collinear locations identify two directions. Their cross product and determinant+1 fix the normal; rank3 is not necessary. A reflection is excluded explicitly.'}


def integral(t,y,start,end):
    if start<t[0] or end>t[-1] or end<=start:
        raise AssertionError('Phase interval outside saved support')
    grid=np.r_[start,t[(t>start)&(t<end)],end]
    return float(INTEGRATE(np.interp(grid,t,y),grid))


def bins(t,series,cycles):
    rows=[]
    for cycle in cycles:
        hs,to=cycle['hs_s'],cycle['to_s'];duration=to-hs
        cb=[]
        for i in range(N_BINS):
            a,b=hs+duration*i/N_BINS,hs+duration*(i+1)/N_BINS
            cb.append({'phase_start':i/N_BINS,'phase_end':(i+1)/N_BINS,'duration_s':b-a,
                       'time_window_s':[a,b],**{name:integral(t,y,a,b)/(b-a) for name,y in series.items()}})
        rows.append({'nominal_HS_TO_s':[hs,to],'bins':cb})
    pooled=[]
    for i in range(N_BINS):
        samples=[c['bins'][i] for c in rows];duration=sum(c['duration_s'] for c in samples)
        pooled.append({'phase_start':i/N_BINS,'phase_end':(i+1)/N_BINS,
                       **{name:sum(c[name]*c['duration_s'] for c in samples)/duration for name in series}})
    return rows,pooled


def shape(values):
    x=np.asarray(values)
    mean=float(np.mean(x))
    return {'phase_weighted_mean_m':mean,'phase_RMS_m':float(np.sqrt(np.mean(x*x))),
            'shape_RMS_after_subtracting_mean_m':float(np.sqrt(np.mean((x-mean)**2))),
            'minimum_bin_mean_m':float(x.min()),'maximum_bin_mean_m':float(x.max())}


def main():
    output=AUDIT/'foot_pose_decomposition.json'
    if output.exists():
        raise FileExistsError('Preserve original foot-pose decomposition')
    marker=json.loads((AUDIT/'signed_marker_vertical_audit.json').read_text())
    paths={ROOT/raw for raw in marker['inputs_integrity']['after']}
    phase_path=AUDIT.parent/'alternative/phase_contact_budget.json'
    paths.update([Path(__file__),PROFILE,PAIR/'selection.json',PREVIOUS/'contact_replay.json',
                  AUDIT/'signed_marker_vertical_audit.json',AUDIT/'baseline_AB06.npz',AUDIT/'baseline_AB07.npz',phase_path])
    before={fp(p)['path']:fp(p)['sha256'] for p in sorted(paths)}
    for raw,sha in marker['inputs_integrity']['after'].items():
        if before[raw]!=sha:
            raise AssertionError('Protected source changed')
    profile=json.loads(PROFILE.read_text());spheres=[s for s in profile['spheres'] if s['side']=='left']
    if profile['ground']['normal']!=[0.,1.,0.]:
        raise AssertionError('Vertical plane contract differs')
    local=np.array([s['location'] for s in spheres]);names=[s['name'] for s in spheres]
    index=names.index('left_basis_07');sphere=spheres[index]
    plane_y=profile['ground']['origin'][1];radius=sphere['radius']
    duplicates=[(names[i],names[j]) for i in range(len(local)) for j in range(i+1,len(local)) if np.array_equal(local[i],local[j])]
    previous=json.loads((PREVIOUS/'contact_replay.json').read_text())
    previous_phase=json.loads(phase_path.read_text())
    selection=json.loads((PAIR/'selection.json').read_text())
    subjects={}
    for subject in ('AB06','AB07'):
        with np.load(AUDIT/f'baseline_{subject}.npz',allow_pickle=False) as data:
            t=data['times_s'].copy();centers=data['contact_centers_m'].copy();saved_names=data['sphere_names'].tolist()
        if saved_names!=names or centers.shape!=(len(t),len(local),3) or not np.isfinite(centers).all():
            raise AssertionError('Saved center schema differs')
        rotation,origin,check=recover(local,centers)
        rotated=np.einsum('tij,j->ti',rotation,local[index])[:,1]
        raw=radius+plane_y-centers[:,index,1]
        reconstructed_raw=radius+plane_y-origin[:,1]-rotated
        identity_error=float(np.max(np.abs(raw-reconstructed_raw)))
        if identity_error>1e-12:
            raise AssertionError('Signed-depth geometry identity failed')
        series={'minus_foot_origin_Y_m':-origin[:,1],
                'minus_rotated_basis07_local_Y_m':-rotated,
                'raw_signed_depth_m':raw,'positive_penetration_m':np.maximum(raw,0)}
        cycle_bins,pooled=bins(t,series,selection['subjects'][subject]['primary_cycles'])
        native=previous['subjects'][subject]['cases']['filtered_IK']['aggregate']
        full_duration=t[-1]-t[0]
        mean_origin=float(INTEGRATE(origin[:,1],t)/full_duration)
        pitch=np.arctan2(rotation[:,1,0],np.hypot(rotation[:,0,0],rotation[:,2,0]))
        mean_pitch=float(INTEGRATE(pitch,t)/full_duration)
        origin_error=abs(mean_origin-native['mean_foot_origin_y_m'])
        pitch_error=abs(mean_pitch-native['mean_sagittal_foot_pitch_rad'])
        if origin_error>1e-12 or pitch_error>1e-12:
            raise AssertionError('Recovered pose does not match saved native FK scalar means')
        old_bins=previous_phase['subjects'][subject]['filtered_IK']['pooled_stance_phase_bins']
        phase_error=max(abs(row['raw_signed_depth_m']-old['basis07_raw_signed_depth_mean_m']) for row,old in zip(pooled,old_bins))
        if phase_error>1e-12:
            raise AssertionError('Raw depth phase bins differ from prior algebraic audit')
        check.update(raw_signed_depth_identity_max_abs_error_m=identity_error,
                     existing_native_FK_mean_origin_Y_error_m=origin_error,
                     existing_native_FK_mean_X_axis_elevation_error_rad=pitch_error,
                     existing_raw_depth_phase_bin_max_error_m=phase_error,
                     native_transform_array_availability='R,p arrays were not previously stored; recovered full-grid poses agree with existing scalar origin/pitch means and all8 saved centers.')
        subjects[subject]={'validation':check,'source_NPZ':fp(AUDIT/f'baseline_{subject}.npz'),
                           'per_cycle_stance_phase_bins':cycle_bins,'pooled_stance_phase_bins':pooled,
                           'recovered_pose_existing_mean_checks':{'full_window_mean_origin_Y_m':mean_origin,
                                                                  'full_window_mean_X_axis_elevation_rad':mean_pitch}}
    aa,bb=[subjects[s]['pooled_stance_phase_bins'] for s in ('AB06','AB07')]
    differences=[]
    for a,b in zip(aa,bb):
        origin=b['minus_foot_origin_Y_m']-a['minus_foot_origin_Y_m']
        orientation=b['minus_rotated_basis07_local_Y_m']-a['minus_rotated_basis07_local_Y_m']
        raw=b['raw_signed_depth_m']-a['raw_signed_depth_m']
        differences.append({'phase_start':a['phase_start'],'phase_end':a['phase_end'],
                            'AB07_minus_AB06_raw_signed_depth_m':raw,
                            'minus_origin_Y_difference_m':origin,
                            'minus_orientation_contribution_difference_m':orientation,
                            'sum_identity_error_m':raw-origin-orientation,
                            'positive_penetration_difference_m':b['positive_penetration_m']-a['positive_penetration_m']})
    raw=np.array([x['AB07_minus_AB06_raw_signed_depth_m'] for x in differences])
    origin=np.array([x['minus_origin_Y_difference_m'] for x in differences])
    orientation=np.array([x['minus_orientation_contribution_difference_m'] for x in differences])
    maximum_error=max(abs(x['sum_identity_error_m']) for x in differences)
    if maximum_error>1e-12:
        raise AssertionError('B-minus-A phase decomposition identity failed')
    centered_cross=float(np.mean((origin-origin.mean())*(orientation-orientation.mean())))
    aggregate={'raw_signed_depth_difference':shape(raw),'minus_origin_Y_difference':shape(origin),
               'minus_orientation_contribution_difference':shape(orientation),
               'phase_bin_sum_identity_max_abs_error_m':maximum_error,
               'mean_sum_identity_error_m':float(raw.mean()-origin.mean()-orientation.mean()),
               'centered_cross_term_m2':centered_cross,
               'shape_variance_identity_error_m2':float(np.var(raw)-np.var(origin)-np.var(orientation)-2*centered_cross),
               'orientation_origin_centered_shape_correlation':float(np.corrcoef(origin,orientation)[0,1])}
    after={raw:fp(ROOT/raw)['sha256'] for raw in before}
    if before!=after:
        raise AssertionError('Protected source changed during algebraic decomposition')
    result={'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026',
        'status':'PASS_algebraic_proper_pose_and_signed_depth_identity','helper':fp(__file__),
        'basis07':sphere,'ground_plane':profile['ground'],'local_rank_and_duplicate_contract':{'duplicate_location_pairs':duplicates,
                          'rank2_is_sufficient_with_proper_rotation':True,'rank3_not_claimed':True},
        'subjects':subjects,'AB07_minus_AB06_stance_phase_decomposition':differences,'aggregate':aggregate,
        'methodology':{'scope':'NumPy SVD/algebra on saved NPZ only; no OpenSim import, FK, integration, model edit, IK solve, force fitting or new detector.',
                      'Kabsch':'Column-vector center=R*local+p; covariance local_centered.T*world_centered; R=V*diag(1,1,sign(det(VU.T)))*U.T, p=world_mean-R*local_mean.',
                      'signed_depth':'radius+planeY-pY-(R*local07)Y. Shared radius and plane cancel exactly in AB07-minus-AB06.',
                      'phase_pooling':'20 equal measured-HS→TO stance bins per frozen cycle; time-weighted bin means, then cycle-duration-weighted pooling. Equal1/20 phase weights for pair difference summary.',
                      'interpolation':'Piecewise-linear recovered pY and rotated-point-Y values at saved native1ms+exactevent grid; interpolation only inserts bin endpoints, with no rotation-matrix interpolation.',
                      'positive_penetration':'max(raw_depth,0) is reported separately; its B-minus-A cannot be decomposed by independently clipping origin/orientation terms.',
                      'orientation':'The entire recovered rigid rotation acting on the eccentric local basis07 point; no isolation of one anatomical joint or preparation parameter.'},
        'limits':['Origin/rotation are descriptive factors of the foot pose under each complete prepared reference, not exclusively anatomy or preparation.',
                  'Shape RMS components are not additive; their covariance/cross-term is required. Mean signed-depth contributions are additive by identity.',
                  'A constant-depth descriptive offset does not identify a physical floor, marker-registration error or a transferable contact law.',
                  'Phase normalization is diagnostic shape comparison, not a replacement for native timing or main impulse scoring.'],
        'inputs_integrity':{'status':'PASS','protected_count':len(before),'all_unchanged':before==after,'before':before,'after':after}}
    with output.open('x',encoding='utf-8') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(aggregate,indent=2),flush=True)
    print('PASS',output.relative_to(ROOT),flush=True)


if __name__=='__main__':
    main()
