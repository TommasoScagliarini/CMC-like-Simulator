"""Render all completed verification rollouts into dedicated root plot folders."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from PIL import Image

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]

def main():
    results = []
    for rollout in sorted((PACKAGE/'runs/rollout/masking_verification_2026-10-01').iterdir()):
        summary_path = rollout/'rollout_summary.json'
        if not summary_path.exists():
            continue
        summary = json.loads(summary_path.read_text())
        if not summary.get('ok'):
            continue
        destination = ROOT/'plot'/f'10_01_2026_masking_verification_{rollout.name}'
        receipt_path = rollout/'plots/plot_receipt.json'
        if not destination.exists():
            with (HERE/f'{rollout.name}_render.log').open('w') as log:
                subprocess.run([sys.executable,str(PACKAGE/'validation/render_canonical_rollout.py'),
                    '--rollout',str(rollout)],cwd=ROOT,env=dict(os.environ,MPLBACKEND='Agg'),
                    stdout=log,stderr=subprocess.STDOUT,timeout=300,check=True)
            receipt = json.loads(receipt_path.read_text())
            source = Path(receipt['figures'][0]).parent
            assert source.parent == ROOT/'plot'
            source.rename(destination)
            receipt['figures'] = [str(destination/Path(f).name) for f in receipt['figures']]
            content = json.dumps(receipt,indent=2)+'\n'
            receipt_path.write_text(content)
            (destination/'plot_receipt.json').write_text(content)
        pngs = sorted(destination.glob('*.png'))
        assert len(pngs)==7
        for path in pngs:
            with Image.open(path) as img:
                img.verify()
        record = {'label':rollout.name,'steps':summary['steps'],'end_reason':summary['end_reason'],
            'png_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in pngs},
            'missing_channels':(destination/'missing_channels.txt').read_text()}
        (destination/'verification.json').write_text(json.dumps(record,indent=2)+'\n')
        results.append(record)
        print(rollout.name,summary['steps'],len(pngs),flush=True)
    (HERE/'plot_verification.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':
    main()
