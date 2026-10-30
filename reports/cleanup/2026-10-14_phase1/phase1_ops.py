"""Exact-file cleanup and whole-worktree integrity comparison."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
import os
import subprocess
import time

ROOT = Path('/Users/tommy/Documents/CMC-like-Simulator - Claude')
OUT = Path('/tmp/cmc_cleanup_phase1_2026_10_14')

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(2 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + '\n')

def files():
    result = []
    for base, dirs, names in os.walk(ROOT, followlinks=False):
        dirs[:] = sorted(d for d in dirs if not (Path(base) == ROOT and d == '.git'))
        for name in sorted(names):
            p = Path(base) / name
            if p.is_symlink():
                raise RuntimeError(f'Unexpected symlink: {p}')
            if p.is_file():
                result.append(p)
    return result

def fingerprint(path):
    before = path.stat()
    value = sha(path)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise RuntimeError(f'File changed while hashing: {path}')
    return path.relative_to(ROOT).as_posix(), {'bytes': before.st_size, 'sha256': value}

def before():
    started = time.monotonic()
    action = json.loads((OUT/'action_manifest.json').read_text())
    removed = {r['path'] for r in action['records']}
    paths = [p for p in files() if p.relative_to(ROOT).as_posix() not in removed]
    print(f'Fingerprinting {len(paths)} retained files', flush=True)
    with ThreadPoolExecutor(max_workers=8) as pool:
        entries = dict(pool.map(fingerprint, paths))
    result = {'root': str(ROOT), 'head': action['head'], 'count': len(entries),
              'bytes': sum(v['bytes'] for v in entries.values()), 'files': entries,
              'elapsed_s': time.monotonic()-started}
    write('whole_worktree_before.json', result)
    print(json.dumps({k:v for k,v in result.items() if k!='files'}), flush=True)

def remove():
    action = json.loads((OUT/'action_manifest.json').read_text())
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==action['head']
    assert (OUT/'whole_worktree_before.json').is_file()
    # Validate all targets before any mutation. No recursive deletion is used.
    for r in action['records']:
        p=ROOT/r['path']
        if not p.is_file() or p.is_symlink() or sha(p)!=r['sha256']:
            raise RuntimeError(f'Cleanup target changed since preflight: {p}')
    for rel,m in action['protected_incidental'].items():
        assert sha(ROOT/rel)==m['sha256'],rel
    journal=[]
    journal_path=OUT/'deletion_journal.jsonl'
    with journal_path.open('x') as f:
        for r in action['records']:
            (ROOT/r['path']).unlink()
            f.write(json.dumps(r)+'\n');f.flush()
            journal.append(r)
    dirs=set()
    for r in journal:
        p=(ROOT/r['path']).parent
        while p!=ROOT:
            if p.name in {'__pycache__','.pytest_cache','.ruff_cache'} or any(
                p == ROOT/q or ROOT/q in p.parents for q in
                ['build','build_online_grf','tools/sea_plugin_relative_d/build','tools/online_grf_contact/build']):
                dirs.add(p)
            p=p.parent
    removed_dirs=[]
    for p in sorted(dirs,key=lambda p:len(p.parts),reverse=True):
        try:
            p.rmdir();removed_dirs.append(p.relative_to(ROOT).as_posix())
        except OSError:
            pass
    result={'removed_files':len(journal),'removed_bytes':sum(r['bytes'] for r in journal),
            'removed_empty_directories':removed_dirs,'protected_incidental':action['protected_incidental']}
    write('cleanup_result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='removed_empty_directories'}),flush=True)

def after():
    started=time.monotonic()
    baseline=json.loads((OUT/'whole_worktree_before.json').read_text())
    paths=files()
    with ThreadPoolExecutor(max_workers=8) as pool:
        observed=dict(pool.map(fingerprint,paths))
    missing=sorted(set(baseline['files'])-set(observed))
    added=sorted(set(observed)-set(baseline['files']))
    changed=[p for p,m in baseline['files'].items() if p in observed and observed[p]!=m]
    ignore_expected=json.loads((OUT/'expected_gitignore.json').read_text())
    expected_changes=['.gitignore']
    failures=[p for p in changed if p not in expected_changes]
    if observed.get('.gitignore',{}).get('sha256')!=ignore_expected['sha256_after']:
        failures.append('.gitignore unexpected content')
    action=json.loads((OUT/'action_manifest.json').read_text())
    regenerated=[r['path'] for r in action['records'] if (ROOT/r['path']).exists()]
    # New files must be reported, not silently tolerated.
    result={'passed':not(missing or added or failures or regenerated),
            'retained_files_checked':len(baseline['files']),
            'retained_bytes_checked':baseline['bytes'],
            'unchanged_files':sum(p in observed and observed[p]==m for p,m in baseline['files'].items()),
            'expected_changes':expected_changes,'changed':changed,'unexpected_changes':failures,
            'missing_retained':missing,'unexpected_added':added,'regenerated_removed_targets':regenerated,
            'elapsed_s':time.monotonic()-started}
    write('whole_worktree_after_check.json',result)
    print(json.dumps(result),flush=True)
    raise SystemExit(0 if result['passed'] else 1)

parser=argparse.ArgumentParser();parser.add_argument('phase',choices=['before','remove','after'])
args=parser.parse_args()
globals()[args.phase]()
