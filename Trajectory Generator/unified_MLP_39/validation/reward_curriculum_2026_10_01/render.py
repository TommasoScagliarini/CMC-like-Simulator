"""Render completed experiment rollouts; canonical figures stay in root plot/."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
from PIL import Image
HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parents[1];ROOT=PACKAGE.parents[1]
records=[]
for rollout in sorted((PACKAGE/'runs/rollout/reward_curriculum_2026-10-01').iterdir()):
    summary=rollout/'rollout_summary.json'
    if not summary.exists() or not json.loads(summary.read_text()).get('ok'):continue
    destination=ROOT/'plot'/f'10_01_2026_reward_curriculum_{rollout.name}'
    if not destination.exists():
        with (HERE/f'{rollout.name}_render.log').open('w') as log:
            subprocess.run([sys.executable,str(PACKAGE/'validation/render_canonical_rollout.py'),'--rollout',str(rollout)],cwd=ROOT,env=dict(os.environ,MPLBACKEND='Agg'),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
        receipt_path=rollout/'plots/plot_receipt.json'
        receipt=json.loads(receipt_path.read_text());source=Path(receipt['figures'][0]).parent
        assert source.parent==ROOT/'plot';source.rename(destination)
        receipt['figures']=[str(destination/Path(x).name) for x in receipt['figures']]
        text=json.dumps(receipt,indent=2)+'\n';receipt_path.write_text(text);(destination/'plot_receipt.json').write_text(text)
    pngs=sorted(destination.glob('*.png'));assert len(pngs)==7
    for p in pngs:
        with Image.open(p) as image:image.verify()
    records.append({'rollout':str(rollout),'folder':str(destination),'pngs':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in pngs}})
    print(rollout.name,len(pngs),flush=True)
(HERE/'plots_verified.json').write_text(json.dumps(records,indent=2)+'\n')
