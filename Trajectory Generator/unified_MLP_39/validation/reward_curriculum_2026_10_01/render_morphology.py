"""Reuse the existing morphology plot with the evaluated checkpoint's label."""
import contextlib
import importlib.util
import io
import sys
from pathlib import Path
from matplotlib.figure import Figure

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
script = ROOT / 'validation/plot_morphology_corridor_rollout.py'
spec = importlib.util.spec_from_file_location('existing_morphology_plot', script)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
original_title = Figure.suptitle
original_argv = sys.argv
for label in ('initial_ex_novo_hidden', 'initial_ex_novo_visible',
              'ex_novo_s1_u10_hidden', 'ex_novo_s1_u10_visible'):
    title = 'CP90 actor before adaptation' if label.startswith('initial') else 'CP90 transfer: 10 actor updates'
    title += ' — ' + ('hidden' if label.endswith('hidden') else 'visible')
    def suptitle(self, text, *args, **kwargs):
        return original_title(self, text.replace('Checkpoint best:', title + ':'), *args, **kwargs)
    Figure.suptitle = suptitle
    try:
        sys.argv = [str(script), '--rollout-dir', str(HERE.parents[1] / 'runs/rollout/reward_curriculum_2026-10-01' / label),
                    '--output', str(ROOT / 'plot/10_01_2026_reward_curriculum_comparison' / f'{label}_morphology.png')]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            assert module.main() == 0
        (HERE / f'{label}_morphology.json').write_text(output.getvalue())
        print('Rendered', label)
    finally:
        Figure.suptitle = original_title
        sys.argv = original_argv
