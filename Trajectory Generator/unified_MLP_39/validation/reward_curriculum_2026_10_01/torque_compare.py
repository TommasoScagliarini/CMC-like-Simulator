"""Plot spring and motor torque on the common surviving evaluation window."""
import io
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PLOTS = ROOT / 'plot/10_01_2026_reward_curriculum_comparison'
PLOTS.mkdir(exist_ok=True)
results = json.loads((HERE / 'results.json').read_text())
lookup = {row['label']: row for row in results['evaluations']}

def load(label):
    text = (Path(lookup[label]['output']) / 'sim_outputs/rollout_episode_sea_torques.sto').read_text()
    return np.genfromtxt(io.StringIO(text.split('endheader', 1)[1].lstrip()), names=True)

for decision in results['decisions']:
    stage, updates = decision['stage'], decision['actor_updates_per_arm']
    for visibility in ('visible', 'hidden'):
        labels = [f'initial_ex_novo_{visibility}',
                  f'imitation_s{stage}_u{updates}_{visibility}',
                  f'ex_novo_s{stage}_u{updates}_{visibility}']
        curves = {label: load(label) for label in labels}
        start, end = 13.19, min(17.94, *(a['time'][-1] for a in curves.values()))
        if end <= start:
            continue
        fig, axes = plt.subplots(2, 2, figsize=(14, 7), sharex=True, layout='constrained')
        for i, joint in enumerate(('Knee', 'Ankle')):
            for j, kind in enumerate(('spring', 'motor')):
                ax = axes[i, j]
                for label, a in curves.items():
                    keep = (a['time'] >= start) & (a['time'] <= end)
                    ax.plot(a['time'][keep], a[f'SEA_{joint}_tau_{kind}'][keep],
                            label=label, linewidth=.8, alpha=.85)
                ax.axhline(0, color='black', linewidth=.6)
                ax.set_title(f'{joint}: {kind}')
                ax.set_ylabel('Torque [N m]')
                ax.set_xlim(start, end)
                ax.grid(alpha=.25)
                ax.legend(fontsize=7)
        for ax in axes[-1]:
            ax.set_xlabel('Simulation time [s]')
        fig.suptitle(f'CP90 reward transfer — {updates} actor updates — {visibility}')
        fig.savefig(PLOTS / f's{stage}_u{updates}_{visibility}_torques.png', dpi=140)
        plt.close(fig)
print('Torque comparisons rendered for', len(results['decisions']), 'decisions')
