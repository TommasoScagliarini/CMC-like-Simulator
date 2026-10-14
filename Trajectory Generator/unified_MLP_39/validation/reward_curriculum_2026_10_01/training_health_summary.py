"""Summarize completed updates without treating training returns as evaluations."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = HERE.parents[1] / 'runs/training/MLP39_reward_curriculum_2026-10-01'
records = []
for path in sorted(RUNS.glob('*/stage*/train_iterations.jsonl')):
    for line in path.read_text().splitlines():
        row = json.loads(line)
        audit = row['unified_phase_audit']
        learner = row['learner_metrics']
        metric = lambda suffix: next((v for k, v in learner.items() if k.endswith('/' + suffix)), None)
        records.append({
            'arm': path.parent.parent.name,
            'stage': path.parent.name,
            'iteration': row['iteration'],
            'phase': row['unified_phase'],
            'steps': audit['target_availability_batch']['total_steps'],
            'visible_fraction': audit['target_availability_batch']['visible_fraction'],
            'vf_loss_clipped': row['vf_loss'],
            'vf_loss_unclipped': metric('vf_loss_unclipped'),
            'vf_explained_variance': metric('vf_explained_var'),
            'max_minibatch_kl': row['max_minibatch_mean_kl_loss'],
            'kl_guard_passed': row['kl_update_guard']['pass'],
            'actor_exact': audit['actor_exact'],
            'mean_changed': audit['mean_changed'],
            'critic_changed': audit['critic_changed'],
            'health': row['training_health'],
            'termination_metrics': row['termination_metrics'],
        })
(HERE / 'training_health_summary.json').write_text(json.dumps(records, indent=2) + '\n')
lines = ['| Reward | Iterazione | Fase | Visibili | VF loss clippata / libera | EV | KL massimo |',
         '|---|---:|---|---:|---:|---:|---:|']
for r in records:
    lines.append(f"| {r['arm']} | {r['iteration']} | {r['phase']} | {r['visible_fraction']:.3f} | "
                 f"{r['vf_loss_clipped']:.3f} / {r['vf_loss_unclipped']:.3f} | "
                 f"{r['vf_explained_variance']:.4f} | {r['max_minibatch_kl']:.6f} |")
(HERE / 'training_health_summary.md').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
