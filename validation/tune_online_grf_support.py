"""Scale per-side onlineGRF stiffness from a forward sensor impulse audit."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import load_online_grf_profile, write_online_grf_profile
from path_resolver import resolve_repo_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--forward-report", required=True)
    parser.add_argument("--out-profile", required=True)
    parser.add_argument(
        "--run",
        choices=["sensor", "online"],
        default="sensor",
        help="Forward-report run whose per-side impulse ratios drive scaling.",
    )
    parser.add_argument("--minimum-scale", type=float, default=0.5)
    parser.add_argument("--maximum-scale", type=float, default=5.0)
    args = parser.parse_args()

    profile = load_online_grf_profile(args.profile)
    report_path = resolve_repo_path(args.forward_report)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    run = report.get("runs", {}).get(args.run)
    if run is None or "online_grf" not in run:
        raise ValueError(f"Forward report does not contain runs.{args.run}.online_grf.")

    scales = {}
    for side in ("left", "right"):
        ratio = run["online_grf"][side].get("prescribed_impulse_ratio")
        if ratio is None or float(ratio) <= 0.0:
            raise ValueError(f"Invalid {side} prescribed impulse ratio: {ratio!r}")
        scales[side] = min(
            float(args.maximum_scale),
            max(float(args.minimum_scale), 1.0 / float(ratio)),
        )

    spheres = []
    for sphere in profile.spheres:
        material = sphere.material or profile.material
        spheres.append(
            replace(
                sphere,
                material=replace(
                    material,
                    stiffness=float(material.stiffness * scales[sphere.side]),
                ),
            )
        )
    tuned = replace(
        profile,
        source="forward_impulse_tuned_physical_contact_candidate",
        spheres=tuple(spheres),
        metadata={
            **dict(profile.metadata),
            "online_mode_status": "requires_forward_validation",
            "support_tuning": {
                "source_report": str(report_path.resolve()),
                "source_run": args.run,
                "stiffness_scale_by_side": scales,
            },
        },
    )
    destination = write_online_grf_profile(tuned, resolve_repo_path(args.out_profile))
    print(json.dumps({"output_profile": str(destination), "scales": scales}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
