"""Generate a reviewable onlineGRF profile from heel/toe model markers."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import opensim

from model_loader import _load_plugin
from online_grf import infer_profile_from_markers, write_online_grf_profile
from path_resolver import resolve_repo_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--radius", type=float, default=0.035)
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    _load_plugin(str(resolve_repo_path(args.sea_plugin)))
    model = opensim.Model(str(resolve_repo_path(args.model)))
    profile = infer_profile_from_markers(model, radius=args.radius)
    destination = write_online_grf_profile(profile, resolve_repo_path(args.out))
    print(f"onlineGRF profile: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
