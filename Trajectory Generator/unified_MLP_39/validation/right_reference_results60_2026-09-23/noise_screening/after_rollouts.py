"""Durable offline analysis after the five rollout supervisors have exited."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
script=HERE/'analyze.py'
expected=hashlib.sha256(script.read_bytes()).hexdigest()
started=time.monotonic()
while True:
    status=json.loads((HERE/'status.json').read_text())
    if status['state']=='failed':
        raise SystemExit('Noise rollout failure; no partial result labeled complete')
    if status['state']=='completed':
        break
    if time.monotonic()-started>3600:
        raise TimeoutError('Timed out waiting for five stochastic rollouts')
    time.sleep(10)
assert hashlib.sha256(script.read_bytes()).hexdigest()==expected
with (HERE/'analysis.log').open('x') as stream:
    process=subprocess.run([sys.executable,str(script)],stdout=stream,stderr=subprocess.STDOUT,
        env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1'},timeout=600)
(HERE/'analysis_execution.json').write_text(json.dumps({'returncode':process.returncode,'script_sha256':expected},indent=2)+'\n')
raise SystemExit(process.returncode)
