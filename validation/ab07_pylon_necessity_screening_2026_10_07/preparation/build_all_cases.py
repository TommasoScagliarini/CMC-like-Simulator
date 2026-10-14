"""Run the authorized, peer-reviewed preparation sequence with preserved logs."""
from datetime import datetime,timezone
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def fp(p):
    p=Path(p)
    return {"path":p.resolve().relative_to(ROOT).as_posix(),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--peer-reviewed',action='store_true');args=parser.parse_args()
    if not args.peer_reviewed:parser.error('Explicit peer review PASS is required before builds')
    protocol_path=HERE/'preparation_protocol.json';helper=HERE/'prepare_pylon_sweep.py'
    protocol=json.loads(protocol_path.read_text());manifest=protocol['protected_sources_before']
    if fp(helper)['sha256']!=manifest[fp(helper)['path']]:raise AssertionError('Frozen preparation helper changed')
    logs=HERE/'logs';logs.mkdir(exist_ok=False)
    result={'created_utc':datetime.now(timezone.utc).isoformat(),'status':'running','helper':fp(__file__),'preparation_helper':fp(helper),'protocol':fp(protocol_path),'cases':[]}
    for name in protocol['execution_order']:
        print('PREPARATION',name,protocol['cases'][name]['preparation_kind'],flush=True)
        log=logs/(name+'.log')
        command=[sys.executable,'-B',str(helper),'--case',name]
        with log.open('x',encoding='utf-8') as stream:
            ret=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=False)
        receipt=ROOT/protocol['cases'][name]['case_dir']/'receipt.json'
        row={'case':name,'returncode':ret.returncode,'log':fp(log),'receipt':fp(receipt) if receipt.is_file() else None}
        result['cases'].append(row)
        if ret.returncode!=0:
            result['status']='failed';break
        saved=json.loads(receipt.read_text())
        if saved['status']!='completed':raise AssertionError('Case process returned without a completed receipt')
        print('COMPLETED',name,saved['output_model']['sha256'],saved['full_IK']['sha256'],flush=True)
    else:result['status']='completed'
    result['completed_utc']=datetime.now(timezone.utc).isoformat()
    output=HERE/'batch_receipt.json'
    with output.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'status':result['status'],'cases':len(result['cases']),'batch_receipt':fp(output)},allow_nan=False),flush=True)
    if result['status']!='completed':raise SystemExit(1)


if __name__=='__main__':main()
