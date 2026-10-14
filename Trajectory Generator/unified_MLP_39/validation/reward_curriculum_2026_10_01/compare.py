"""Compare physical curves and metrics on identical time windows, never raw returns."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parents[1];ROOT=PACKAGE.parents[1]
PLOTS=ROOT/'plot/10_01_2026_reward_curriculum_comparison';PLOTS.mkdir(exist_ok=True)
data=json.loads((HERE/'results.json').read_text())
rows=data['evaluations'];lookup={r['label']:r for r in rows}
lines=['| Rollout | Passi | Cicli validi | RMSE knee/ankle [°] | ROM knee/ankle [°] | Gate |', '|---|---:|---:|---:|---:|---|']
for r in rows:
 m=r['metrics'];j=m.get('joints',{});fmt=lambda key: '/'.join(f'{j[x][key]:.2f}' for x in ('knee','ankle')) if j else 'n/a'
 rom='/'.join(f'{j[x]["actual"]["excursion_deg"]:.2f}' for x in ('knee','ankle')) if j else 'n/a'
 lines.append(f'| {r["label"]} | {m["steps"]} | {m["fsm"]["phase_valid_cycle_count"]} | {fmt("rmse_actual_target_deg")} | {rom} | {m["gate"]["passed"]} |')
(HERE/'metrics_table.md').write_text('\n'.join(lines)+'\n')
torque_lines=['| Rollout | Finestra completa | Ankle area positiva [N m s] | Ankle tempo positivo [%] | Knee spring HP20 RMS [N m] | Knee motor HP20 RMS [N m] |',
              '|---|---|---:|---:|---:|---:|']
for r in rows:
 m=r['metrics'];t=m.get('torque',{})
 if not t:continue
 knee,ankle=t['knee'],t['ankle']
 torque_lines.append(f'| {r["label"]} | {m["common_window_complete"]} | {ankle["positive_area_nm_s"]:.3f} | '
                     f'{100*ankle["positive_fraction"]:.2f} | {knee["highpass_rms_nm"]["20"]:.3f} | '
                     f'{knee["controller_channels"]["tau_input_plugin"]["highpass_20_rms_nm"]:.3f} |')
(HERE/'torque_table.md').write_text('\n'.join(torque_lines)+'\n')
paired={}
for d in data['decisions']:
 stage=d['stage'];u=d['actor_updates_per_arm']
 for visibility in ('hidden','visible'):
  labels=[f'initial_ex_novo_{visibility}',f'imitation_s{stage}_u{u}_{visibility}',f'ex_novo_s{stage}_u{u}_{visibility}']
  traces={label:json.loads((Path(lookup[label]['output'])/'rollout_policy_trace.json').read_text()) for label in labels}
  end=min(trace[-1]['time'] for trace in traces.values());start=13.19;end=min(end,17.94)
  if end<=start:continue
  key=f's{stage}_u{u}_{visibility}';paired[key]={'window_s':[start,end],'policies':{}}
  fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,layout='constrained')
  for idx,joint in enumerate(('knee','ankle')):
   coord=f'pros_{joint}_angle';sign=-1 if joint=='knee' else 1
   base=traces[labels[0]];axes[idx].plot([r['time'] for r in base],sign*np.degrees([r['imitation_target_q'][coord] for r in base]),color='black',lw=1.3,label='Prescribed target')
   for label,trace in traces.items():
    t=np.array([r['time'] for r in trace]);actual=sign*np.degrees([r['prosthetic_state'][coord] for r in trace]);target=sign*np.degrees([r['imitation_target_q'][coord] for r in trace]);keep=(t>=start-1e-8)&(t<=end+1e-8)
    axes[idx].plot(t,actual,label=label)
    paired[key]['policies'].setdefault(label,{})[joint]={'rmse_actual_target_deg':float(np.sqrt(np.mean((actual[keep]-target[keep])**2))),'excursion_deg':float(np.ptp(actual[keep]))}
   axes[idx].set_ylabel(joint+' [deg]');axes[idx].grid(alpha=.25);axes[idx].legend(fontsize=8);axes[idx].set_xlim(start,end)
  axes[-1].set_xlabel('Simulation time [s]');fig.suptitle(f'CP90 objective transfer — {u} actor updates — {visibility}')
  fig.savefig(PLOTS/f'{key}_kinematics.png',dpi=140);plt.close(fig)
(HERE/'paired_windows.json').write_text(json.dumps(paired,indent=2)+'\n')
print('Compared',len(rows),'rollouts;',len(paired),'paired windows')
