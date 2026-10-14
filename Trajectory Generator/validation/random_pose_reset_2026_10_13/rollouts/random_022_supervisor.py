import subprocess, sys
from pathlib import Path
with Path(sys.argv[1]).open('wb') as log:
    raise SystemExit(subprocess.call(sys.argv[2:], stdout=log, stderr=subprocess.STDOUT))
