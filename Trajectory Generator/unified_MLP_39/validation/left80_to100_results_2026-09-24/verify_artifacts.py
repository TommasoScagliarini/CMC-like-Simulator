"""Verify the completed evaluation without changing source runs or plots."""
from pathlib import Path
import hashlib
import json

from PIL import Image

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
PLAN = PACKAGE / 'runs/plans/left80_pen2_to100_evaluation_2026-09-24'


def read(path):
    return json.loads(path.read_text())


def main():
    verified = []
    for item in read(PLAN / 'canonical_plots.json'):
        folder = Path(item['folder'])
        receipt = read(folder / 'plot_receipt.json')
        rollout = Path(receipt['rollout'])
        summary = read(rollout / 'rollout_summary.json')
        assert receipt == read(rollout / 'plots/plot_receipt.json')
        assert receipt['checkpoint'] == summary['checkpoint']
        assert Path(summary['checkpoint']).is_dir()
        assert summary['ok'] and summary['steps'] == 500
        assert summary['phase_valid_cycle_count'] == 2
        assert summary['end_reason'] == 'episode_time_limit'
        assert summary['action_selection'] == 'deterministic'
        assert receipt['teacher_source'] == 'prescribed_left_phase_template'
        assert receipt['gait_side'] == 'left'
        assert receipt['figures'] == item['figures']
        assert len(item['figures']) == len(list(folder.glob('*.png'))) == 7
        assert (folder / 'missing_channels.txt').read_text().strip() == 'No missing channels.'
        images = []
        for name in item['figures']:
            path = Path(name)
            assert path.parent == folder and path.is_relative_to(REPO / 'plot')
            with Image.open(path) as im:
                assert im.format == 'PNG'
                size = im.size
                im.verify()
            images.append({'file': str(path.relative_to(REPO)), 'size': size})
        verified.append({'label': item['label'], 'steps': 500, 'valid_cycles': 2,
                         'checkpoint': summary['checkpoint'], 'images': images})

    manifests = [HERE / 'input_hashes.json',
                 PACKAGE / 'runs/plans/left80_penetration_weight2_to100_2026-09-24/source_checkpoint_hashes.json']
    unchanged = {}
    for manifest in manifests:
        hashes = read(manifest)
        for name, expected in hashes.items():
            assert hashlib.sha256((REPO / name).read_bytes()).hexdigest() == expected, name
        unchanged[str(manifest.relative_to(REPO))] = len(hashes)
    result = {'ok': True, 'canonical_png_count': 14, 'evaluations': verified,
              'unchanged_inputs_per_manifest': unchanged}
    (HERE / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'ok': True, 'canonical_png_count': 14,
                      'unchanged_inputs_per_manifest': unchanged}, indent=2))


if __name__ == '__main__':
    main()
