"""Offline lag decomposition on frozen traces, no simulator or policy invocation."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
os.environ.setdefault('MPLCONFIGDIR', str(HERE / '.matplotlib'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

JOINTS = ('pros_knee_angle', 'pros_ankle_angle')
RUNS = {
    'iter25': PACKAGE / 'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25',
    'best29': PACKAGE / 'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29',
    'last40': PACKAGE / 'runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40',
    'v26_best87': TG / 'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
    'oracle_left': PACKAGE / 'runs/rollout/causal_difference_2026-09-21_oracle_left',
    'oracle_right': PACKAGE / 'runs/rollout/causal_difference_2026-09-21_oracle_right',
}
LAGS = np.arange(-20, 21, dtype=int)
START, END = 13.30, 17.75


def arr(trace, *keys):
    out = []
    for row in trace:
        for k in keys:
            row = row[k]
        out.append(row)
    return np.asarray(out, dtype=float)


def rmse(x, y):
    return float(np.rad2deg(np.sqrt(np.mean((x-y)**2))))


def corr(x, y):
    return float(np.corrcoef(x, y)[0, 1])


def lag_stats(x, y, idx, t):
    """Positive lag: output y occurs later; compare x(t) with y(t+lag)."""
    rr = np.asarray([rmse(x[idx], y[idx+k]) for k in LAGS])
    cc = np.asarray([corr(x[idx], y[idx+k]) for k in LAGS])
    kr, kc = int(LAGS[np.argmin(rr)]), int(LAGS[np.argmax(cc)])
    z = y[idx+kr]
    # Affine fit is descriptive only: amplitude/bias mismatch after time alignment.
    a, b = np.linalg.lstsq(np.column_stack((x[idx], np.ones(len(idx)))), z, rcond=None)[0]
    return {
        'samples': len(idx), 'time_window_s': [float(t[idx[0]]), float(t[idx[-1]])],
        'lag_min_rmse_ms': kr*10, 'lag_max_correlation_ms': kc*10,
        'rmse_zero_deg': float(rr[20]), 'rmse_min_deg': float(rr.min()),
        'rmse_at_max_corr_deg': float(rr[np.argmax(cc)]),
        'correlation_zero': float(cc[20]), 'correlation_max': float(cc.max()),
        'fraction_squared_error_removed_by_best_shift': float(1-(rr.min()/rr[20])**2) if rr[20] > 1e-8 else None,
        'aligned_output_gain_vs_input': float(a), 'aligned_output_bias_deg': float(np.rad2deg(b)),
        'aligned_output_residual_after_affine_fit_deg': rmse(z, a*x[idx]+b),
        'boundary_optimum': abs(kr) == 20 or abs(kc) == 20,
        'lag_grid_ms': (LAGS*10).tolist(), 'rmse_curve_deg': rr.tolist(), 'correlation_curve': cc.tolist(),
    }


def imitation_value(data, idx, shifts):
    """Shift recorded q/qdot only; target, scale and state-dependent penalties fixed."""
    losses = {'actual': np.zeros(len(idx)), 'served': np.zeros(len(idx))}
    for j, joint in enumerate(JOINTS):
        d = data[joint]
        k = shifts[j]
        for kind in losses:
            pos = np.clip(((d['target'][idx]-d[kind][idx+k])/d['position_range'])**2, 0, 25)
            vel = np.clip(((d['target_vel'][idx]-d[kind+'_vel'][idx+k])/d['velocity_range'])**2, 0, 25)
            losses[kind] += d['position_weight']*pos + d['velocity_weight']*vel
    cfg = data['cfg']
    a = 1/(1+cfg['imitation_weight']*losses['actual'])
    s = 1/(1+cfg['served_imitation_weight']*losses['served'])
    return cfg['blend_imitation']*a+cfg['blend_served_imitation']*s, losses


def main():
    result = {'method': {
        'positive_lag': 'Compare input(t) to output(t+lag): positive means output is late.',
        'sampling_s': .01, 'lag_grid_ms': (LAGS*10).tolist(),
        'common_window_requested_s': [START, END],
        'fixed_window_for_every_lag': True,
        'interpolation': 'None: integer sample lags avoid attenuating the noisy policy endpoint.',
        'start_transient_excluded_s': .31,
        'endpoint_time': 'Accepted, clipped policy endpoint is dated at policy_segment_times[-1], i.e. trace time after the10ms step; decision was made10ms earlier.',
        'decision_time_convention': 'If endpoint is dated at decision time, its lag relative to target decreases10ms (more apparent anticipation) and endpoint-to-served lag increases10ms; target-to-served lag is unchanged.',
        'local_windows': 'Each complete target-phase cycle; bounds are sampled phase wraps.',
        'reward_counterfactual': 'Frozen-trace rescoring only; q and qdot shifted jointly per joint. No altered physics or successful-contact claim.',
        'reference': '2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md, section4',
    }, 'runs': {}, 'hashes': {}, 'limitations': [
        'A lag is a descriptive best shift, not a pure fixed transport delay or causal fraction of the physical error.',
        'Raw endpoint is oscillatory; low endpoint-to-served correlation makes its single lag weak evidence.',
        'No slew in these traces. Endpoint after action clipping equals accepted endpoint; the within-step raw PCHIP reference runs from the served anchor to this endpoint.',
        'Reported precision is10ms; lowpass delay is frequency-dependent, with nonlinear jerk limiting.',
        'One deterministic episode per checkpoint. Historical best87 is more mature and has a different target.',
        'Offline reward rescore does not recalculate contact, dynamics, phase events or penalties and cannot predict rollout return.',
    ]}
    all_data = {}
    for label, folder in RUNS.items():
        p = folder / 'rollout_policy_trace.json'
        raw = p.read_bytes()
        trace = json.loads(raw)
        summary = json.loads((folder / 'rollout_summary.json').read_text())
        t = arr(trace, 'time')
        assert len(trace) == 500
        np.testing.assert_allclose(np.diff(t), .01, atol=1e-8)
        idx = np.flatnonzero((t >= START-1e-8) & (t <= END+1e-8))
        assert idx.min()+LAGS.min() >= 0 and idx.max()+LAGS.max() < len(t)
        data = {'t': t, 'trace': trace, 'cfg': summary['reward_config'], 'idx': idx}
        run = {'folder': str(folder.relative_to(REPO)), 'joints': {}, 'governor': {}}
        result['hashes'][str(p.relative_to(REPO))] = hashlib.sha256(raw).hexdigest()
        for j, joint in enumerate(JOINTS):
            d = {'target': arr(trace, 'imitation_target_q', joint), 'target_vel': arr(trace, 'imitation_target_qdot', joint),
                 'endpoint': np.asarray([row['policy_segment_values'][-1][j] for row in trace]),
                 'actual': arr(trace, 'prosthetic_state', joint),
                 'actual_vel': arr(trace, 'prosthetic_state', joint+'_vel'),
                 'served': arr(trace, 'prosthetic_state', joint+'_served_ref'),
                 'served_vel': arr(trace, 'prosthetic_state', joint+'_served_ref_vel'),
                 'phase': arr(trace, 'reward_terms', joint+'_imitation_target_phase')}
            for short, key in [('position_range','position_range_rad'), ('velocity_range','velocity_range_rad_s'),
                               ('position_weight','position_weight'), ('velocity_weight','velocity_weight')]:
                d[short] = float(trace[0]['reward_terms'][joint+'_imitation_'+key])
            data[joint] = d
            pairs = [('target','endpoint'), ('endpoint','served'), ('target','served'), ('served','actual'), ('target','actual')]
            stats = {a+'_to_'+b: lag_stats(d[a], d[b], idx, t) for a,b in pairs}
            wraps = np.flatnonzero(np.diff(d['phase']) < -.5)+1
            cycles = []
            for begin, end in zip(wraps[:-1], wraps[1:]):
                ci = np.arange(begin, end)
                if ci.min()-20 < 0 or ci.max()+20 >= len(t): continue
                cycles.append({'time_window_s':[float(t[begin]),float(t[end])],
                               'target_to_served': lag_stats(d['target'], d['served'], ci,t),
                               'served_to_actual': lag_stats(d['served'], d['actual'], ci,t),
                               'target_to_endpoint': lag_stats(d['target'], d['endpoint'], ci,t)})
            k = stats['target_to_served']['lag_min_rmse_ms']//10
            bins = []
            for lo,hi in [(0,.25),(.25,.5),(.5,.75),(.75,1)]:
                bi = idx[(d['phase'][idx]>=lo)&(d['phase'][idx]<hi)]
                bins.append({'phase':[lo,hi],'samples':len(bi),'rmse_zero_deg':rmse(d['target'][bi],d['served'][bi]),
                             'rmse_shifted_deg':rmse(d['target'][bi],d['served'][bi+k])})
            run['joints'][joint] = {'global': stats, 'cycles': cycles, 'phase_bins': bins,
                'full_trace_rmse_target_served_deg': rmse(d['target'],d['served']),
                'full_trace_rmse_target_actual_deg': rmse(d['target'],d['actual']),
                'full_trace_rmse_served_actual_deg': rmse(d['served'],d['actual']),
                'full_trace_rom_target_deg':float(np.rad2deg(np.ptp(d['target']))),
                'full_trace_rom_served_deg':float(np.rad2deg(np.ptp(d['served'])))}
        for key in ('jerk_limit_fraction','velocity_limit_fraction','acceleration_limit_fraction'):
            run['governor'][key+'_mean'] = float(np.mean([row['reference_governor_diagnostics'].get(key,0) for row in trace]))
        # Confirm analytical imitation loss reconstructs runtime losses (float32 trace q introduces rounding).
        zero, losses = imitation_value(data, np.arange(len(trace)), [0,0])
        audits = {}
        for kind, field in [('actual','sound_imitation_loss'),('served','served_imitation_loss')]:
            audits[kind+'_loss_reconstruction_max_abs_error'] = float(np.max(np.abs(losses[kind]-arr(trace,'reward_terms',field))))
            assert audits[kind+'_loss_reconstruction_max_abs_error'] < 2e-6
        shifts = [run['joints'][j]['global']['target_to_served']['lag_min_rmse_ms']//10 for j in JOINTS]
        before,lb = imitation_value(data,idx,[0,0])
        after,la = imitation_value(data,idx,shifts)
        run['reward_frozen_trace'] = {'samples':len(idx),'window_s':[float(t[idx[0]]),float(t[idx[-1]])],
            'shifts_ms': [x*10 for x in shifts],
            'imitation_component_before_mean':float(before.mean()),'imitation_component_after_mean':float(after.mean()),
            'imitation_component_before_sum':float(before.sum()),'imitation_component_after_sum':float(after.sum()),
            'imitation_component_gain_sum':float((after-before).sum()),'reconstruction':audits,
            'actual_loss_mean_before':float(lb['actual'].mean()), 'actual_loss_mean_after':float(la['actual'].mean()),
            'served_loss_mean_before':float(lb['served'].mean()), 'served_loss_mean_after':float(la['served'].mean()),
            'scope':'0.2 actual_score +0.8 served_score only; penalties and physical dynamics not re-evaluated'}
        result['runs'][label]=run
        all_data[label]=data
    (HERE/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    # Dedicated diagnostic figures, separate from canonical plotting.
    fig,axes=plt.subplots(2,3,figsize=(15,8),layout='constrained')
    for col,label in enumerate(['last40','v26_best87','oracle_left']):
        data=all_data[label];t=data['t'];idx=data['idx']
        for row,joint in enumerate(JOINTS):
            d=data[joint];ax=axes[row,col];k=result['runs'][label]['joints'][joint]['global']['target_to_served']['lag_min_rmse_ms']//10
            ax.plot(t[idx]-12.99,np.rad2deg(d['target'][idx]),'k--',label='Target')
            ax.plot(t[idx]-12.99,np.rad2deg(d['served'][idx]),color='#d97706',label='Reference servita')
            ax.plot(t[idx]-12.99,np.rad2deg(d['served'][idx+k]),color='#2563eb',label=f'Servita allineata offline ({k*10}ms)')
            ax.set_title(f'{label}: {"ginocchio" if row==0 else "caviglia"}')
            ax.set_xlabel('Tempo episodio (s)');ax.set_ylabel('Angolo (°)');ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.suptitle('Spostare la reference nel tempo separa ritardo da errore di forma/ampiezza — analisi offline')
    fig.savefig(HERE/'alignment_diagnostic.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for row,joint in enumerate(JOINTS):
        for label in RUNS:
            s=result['runs'][label]['joints'][joint]['global']['target_to_served']
            axes[row,0].plot(s['lag_grid_ms'],s['rmse_curve_deg'],label=label)
            axes[row,1].plot(s['lag_grid_ms'],s['correlation_curve'],label=label)
        for col in (0,1):
            axes[row,col].set_xlabel('Ritardo reference rispetto target (ms)');axes[row,col].grid(alpha=.2)
            axes[row,col].axvline(0,color='gray',linestyle=':');axes[row,col].set_title('Ginocchio' if row==0 else 'Caviglia')
        axes[row,0].set_ylabel('RMSE (°)');axes[row,1].set_ylabel('Correlazione')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Stesso intervallo e stessi campioni per ogni lag:13,30–17,75s')
    fig.savefig(HERE/'lag_curves.png',dpi=160);plt.close(fig)
    compact={label:{'joints':{j:{'target_served':run['joints'][j]['global']['target_to_served']['lag_min_rmse_ms'],
        'target_served_corr':run['joints'][j]['global']['target_to_served']['lag_max_correlation_ms'],
        'rmse_zero':run['joints'][j]['global']['target_to_served']['rmse_zero_deg'],
        'rmse_shift':run['joints'][j]['global']['target_to_served']['rmse_min_deg'],
        'cycles':[(x['target_to_served']['lag_min_rmse_ms'],x['target_to_served']['rmse_zero_deg'],x['target_to_served']['rmse_min_deg']) for x in run['joints'][j]['cycles']]}
        for j in JOINTS},'reward':run['reward_frozen_trace']} for label,run in result['runs'].items()}
    (HERE/'compact.json').write_text(json.dumps(compact,indent=2)+'\n')
    print(json.dumps(compact,indent=2))


if __name__=='__main__':main()
