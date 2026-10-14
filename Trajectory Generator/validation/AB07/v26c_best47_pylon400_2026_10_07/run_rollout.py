"""Reuse the verified best47 evaluator with passive diagnostics in a new namespace."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[3]
LEGACY = REPO / 'Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/run_transfer.py'


def main() -> int:
    manifest = json.loads((AUDIT / 'execution_manifest.json').read_text())
    for relative, expected in manifest['protected_fingerprints_before'].items():
        source = REPO / relative
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f'Frozen input changed before rollout: {relative}')
    spec = importlib.util.spec_from_file_location('ab07_pylon400_legacy_delegate', LEGACY)
    if spec is None or spec.loader is None:
        raise RuntimeError('Cannot import the preserved learned rollout observer')
    delegate = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = delegate
    spec.loader.exec_module(delegate)
    delegate.AUDIT = AUDIT
    delegate.REPO = REPO
    delegate.__file__ = str(Path(__file__).resolve())
    original_writer = delegate.write_json

    def writing_receipt(path, data):
        if path.name in ('receipt.json', 'runtime_receipt.json'):
            data['scope'] = 'Single deterministic V26C best47 rollout on coherently prepared AB07 with distal pylon400mm'
            data['case'] = 'V26_PYLON400_AB07'
            data['distal_mount_length_mm'] = 400.0
            data['protocol'] = str(AUDIT / 'execution_manifest.json')
            if data.get('status') == 'completed' and not data.get('completed_requested_duration'):
                data['status'] = 'interrupted'
        original_writer(path, data)

    delegate.write_json = writing_receipt
    return delegate.main()


if __name__ == '__main__':
    raise SystemExit(main())
