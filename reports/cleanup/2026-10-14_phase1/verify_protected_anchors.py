from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
d=json.loads((here/'protected_paths_integrity_baseline.json').read_text())
root=Path(d['root'])
fail=[]
for name,meta in d['protected'].items():
 p=root/name
 actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
 if actual!=meta['sha256_before']:fail.append({'path':name,'before':meta['sha256_before'],'after':actual})
r={'passed':not fail,'protected_files':len(d['protected']),'failures':fail}
print(json.dumps(r,indent=2))
raise SystemExit(0 if not fail else 1)
