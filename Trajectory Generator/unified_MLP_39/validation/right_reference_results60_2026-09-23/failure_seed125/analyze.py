"""Read-only, aligned audit of four existing seed125 noisy rollouts."""
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
BASE = PACKAGE / 'runs/rollout'
CASES = {
 'best58': 'MLP39_right_screening60_2026-09-23/best58/noise_1_seed125',
 'last40': 'MLP39_right_screening_2026-09-23/last40/noise_1_seed125',
 'best35': 'MLP39_right_screening_2026-09-23/best35/noise_1_seed125',
 'v26': 'V26_robustness_2026-09-23/noise_1_seed125',
}
JOINTS = ['pros_knee_angle', 'pros_ankle_angle']
HASHES = {}

def read(path):
 data = path.read_bytes()
 HASHES[str(path.relative_to(PACKAGE))] = hashlib.sha256(data).hexdigest()
 return json.loads(data)

def point(row):
 q = row['prosthetic_state']; target = row['imitation_target_q']
 result = {'step':row['step'], 'elapsed_s':round(row['time']-12.99,6),
  'penetration_mm':1000*row['reward_terms']['grf_penetration_m'],
  'mean_action':row['policy_action_mean'], 'std_action':row['policy_action_std'],
  'raw_action':row['raw_policy_action'], 'applied_action':row['applied_policy_action'],
  'noise':row['exploration_action_noise'],
  'fsm':{k:row['phase_fsm'].get(k) for k in ('state_name','valid_cycle_count','invalid_event_count','resync_count','last_valid_hs_time_s','stance_elapsed_s')},
  'joints':{}}
 for j in JOINTS:
  actual=q[j]; served=q[j+'_served_ref']; prescribed=target[j]
  result['joints'][j]={
   'target_deg':np.degrees(prescribed),'served_deg':np.degrees(served),'actual_deg':np.degrees(actual),
   'served_minus_target_deg':np.degrees(served-prescribed),
   'actual_minus_served_deg':np.degrees(actual-served),
   'actual_minus_target_deg':np.degrees(actual-prescribed),
   'actual_velocity_deg_s':np.degrees(q[j+'_vel']),
   'served_velocity_deg_s':np.degrees(q[j+'_served_ref_vel'])}
 return result

def main():
 result={'cases':{},'method':{'common_end_step':299,'common_end_elapsed_s':2.99,
  'window_elapsed_s':[2.70,2.99],
  'exclude_partial_terminal':'best58 step300 repeats step299 timestamp; its prosthetic_state is partially advanced and stale served/zero penetration diagnostic must not be compared as a fresh full step.',
  'alignment':'Post-segment prosthetic_state and imitation_target_q at completed segment endpoint; actions/means/std describe the segment just applied.',
  'noise':'Shared standard Gaussian draws do not imply equal absolute action perturbations or equal visited states.'}}
 traces={};draws={}
 for name,folder in CASES.items():
  root=BASE/folder
  rows=read(root/'rollout_policy_trace.json');traces[name]=rows
  draws[name]=read(root/'paired_noise_draws.json')
  summary=read(root/'rollout_summary.json')
  common=rows[:299]
  assert all(abs(r['time']-(12.99+r['step']*.01))<1e-6 for r in common)
  selected=[point(r) for r in common if r['step']>=270]
  metrics={}
  for j in JOINTS:
   metrics[j]={key:float(np.sqrt(np.mean([p['joints'][j][key]**2 for p in selected]))) for key in ('served_minus_target_deg','actual_minus_served_deg','actual_minus_target_deg')}
   metrics[j]['served_velocity_peak_abs_deg_s']=max(abs(p['joints'][j]['served_velocity_deg_s']) for p in selected)
   metrics[j]['actual_velocity_peak_abs_deg_s']=max(abs(p['joints'][j]['actual_velocity_deg_s']) for p in selected)
  events=[]
  for r in common:
   for ev in r.get('online_events',[]):
    if ev.get('side')=='left':events.append({'step':r['step'],'elapsed_s':r['time']-12.99,'type':ev.get('event'),'event_elapsed_s':ev.get('time',r['time'])-12.99})
   f=r['phase_fsm']
   if f.get('invalid_event_this_step') or f.get('resync_event_this_step'):
    events.append({'step':r['step'],'elapsed_s':r['time']-12.99,'invalid':f.get('invalid_event_type'),'resync':f.get('resync_event_this_step')})
  last10=common[289:299]
  grf=root/'sim_outputs/rollout_episode_online_grf.sto'
  raw=grf.read_bytes();HASHES[str(grf.relative_to(PACKAGE))]=hashlib.sha256(raw).hexdigest()
  lines=raw.decode().splitlines();index=lines.index('endheader')+1
  arr=np.genfromtxt(lines[index:],names=True)
  w=arr[(arr['time']>=15.69-1e-6)&(arr['time']<=15.98+1e-6)]
  late=arr[(arr['time']>=15.79-1e-6)&(arr['time']<=15.98+1e-6)]
  beyond=arr[(arr['time']>=15.69-1e-6)&(arr['left_penetration']>.020)]
  late_points=[p for p in selected if p['step']>=290]
  result['cases'][name]={
   'summary':{k:summary.get(k) for k in ('steps','end_reason','episode_return','grf_penetration_max_m','invalid_event_count','phase_valid_cycle_count')},
   'last_trace_times':[{'step':r['step'],'time':r['time']} for r in rows[-2:]],
   'common_final':point(rows[298]),
   'window_rmse_deg':metrics,
   'last100ms_rmse_deg':{j:{key:float(np.sqrt(np.mean([p['joints'][j][key]**2 for p in late_points]))) for key in ('served_minus_target_deg','actual_minus_served_deg','actual_minus_target_deg')} for j in JOINTS},
   'milestones':[point(rows[i-1]) for i in (270,280,290,295,299)],
   'all_window_rows':selected,
   'events_before_failure':events,
   'last10_action_std_mean':np.mean([r['policy_action_std'] for r in last10],axis=0).tolist(),
   'last10_action_mean_mean':np.mean([r['policy_action_mean'] for r in last10],axis=0).tolist(),
   'last10_applied_clip_fraction':np.mean(np.abs([r['raw_policy_action'] for r in last10])>1,axis=0).tolist(),
   'physical_grf_window':{'last_saved_time':float(arr['time'][-1]),'common_penetration_peak_mm':float(w['left_penetration'].max()*1000),'common_force_peak_n':float(w['left_normal_force'].max()),'all_saved_penetration_peak_mm':float(arr['left_penetration'].max()*1000),
    'last200ms_duration_above20mm_s':float(np.count_nonzero(late['left_penetration']>.020)*.001),
    'first_time_above20mm_after2p7s_elapsed_s':float(beyond['time'][0]-12.99) if len(beyond) else None},
   'step300_draw':draws[name][299]}
 standard={name:np.array([r['standard_normal'] for r in rows[:300]]) for name,rows in draws.items()}
 assert all(np.array_equal(standard['best58'],a) for a in standard.values())
 result['shared_first300_standard_normal_draws_identical']=True
 ref=np.array([[r['imitation_target_q'][j] for j in JOINTS] for r in traces['best58'][:299]])
 result['target_difference_common_prefix_max_abs_rad']={name:float(np.max(abs(np.array([[r['imitation_target_q'][j] for j in JOINTS] for r in rows[:299]])-ref))) for name,rows in traces.items()}
 (HERE/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
 (HERE/'input_hashes.json').write_text(json.dumps(HASHES,indent=2)+'\n')
 for name,case in result['cases'].items():
  print(name,json.dumps({k:v for k,v in case.items() if k not in ('all_window_rows','milestones','events_before_failure')},indent=2))

if __name__=='__main__':main()
