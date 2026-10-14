"""Render/verify seven canonical plots per completed frozen visibility control."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]


def main():
    from PIL import Image
    results = []
    for label, visible in [("cp90_hidden", False), ("cp112_visible", True), ("cp184_visible", True)]:
        rollout = PACKAGE / "runs/rollout/masking_diagnosis_2026-10-01" / label
        summary_path = rollout / "rollout_summary.json"
        if not summary_path.exists():
            continue
        summary = json.loads(summary_path.read_text())
        trace = json.loads((rollout / "rollout_policy_trace.json").read_text())
        assert summary["ok"] and len(trace) == summary["steps"]
        for row in trace:
            assert bool(row["target_availability"]["visible"]) == visible
            obs = row["actor_observation_vector_before"]
            if not visible:
                assert all(obs[i] == 0 for i in [0, 1, 35, 36, 37, 38])
            else:
                assert abs(obs[0]**2 + obs[1]**2 - 1) < 1e-5
        destination = ROOT / "plot" / f"10_01_2026_masking_diagnosis_{label}"
        receipt_path = rollout / "plots/plot_receipt.json"
        if not destination.exists():
            with (HERE / f"{label}_render.log").open("x") as log:
                subprocess.run([sys.executable, str(PACKAGE / "validation/render_canonical_rollout.py"),
                                "--rollout", str(rollout)], cwd=ROOT,
                               env=dict(os.environ, MPLBACKEND="Agg"), stdout=log,
                               stderr=subprocess.STDOUT, timeout=300, check=True)
            receipt = json.loads(receipt_path.read_text())
            source = Path(receipt["figures"][0]).parent
            assert source.parent == ROOT / "plot"
            source.rename(destination)
            receipt["figures"] = [str(destination / Path(f).name) for f in receipt["figures"]]
            content = json.dumps(receipt, indent=2) + "\n"
            receipt_path.write_text(content)
            (destination / "plot_receipt.json").write_text(content)
        pngs = sorted(destination.glob("*.png"))
        assert len(pngs) == 7
        for path in pngs:
            with Image.open(path) as img:
                img.verify()
        record = {"label": label, "steps": len(trace), "visibility_verified": True,
                  "end_reason": summary["end_reason"], "png_count": len(pngs),
                  "png_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in pngs},
                  "missing_channels": (destination / "missing_channels.txt").read_text()}
        (destination / "verification.json").write_text(json.dumps(record, indent=2) + "\n")
        results.append(record)
        print(json.dumps(record), flush=True)
    (HERE / "plot_verification.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
