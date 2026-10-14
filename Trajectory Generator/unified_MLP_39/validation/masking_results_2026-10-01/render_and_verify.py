"""Render the selected rollouts and retain canonical PNGs only under root plot/."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]


def main():
    from PIL import Image
    selections = json.loads((HERE / "selection.json").read_text())["selections"]
    results = []
    for selected in selections:
        rollout = Path(selected["rollout"])
        summary = json.loads((rollout / "rollout_summary.json").read_text())
        trace = json.loads((rollout / "rollout_policy_trace.json").read_text())
        if not summary.get("ok") or not trace or len(trace) != summary["steps"]:
            raise RuntimeError("Rollout incomplete or trace length mismatch")
        if Path(summary["checkpoint"]) != Path(selected["checkpoint"]):
            raise RuntimeError("Wrong checkpoint in rollout summary")
        indices = (0, 1, 35, 36, 37, 38)
        for row in trace:
            if any(row["actor_observation_vector_before"][i] != 0 for i in indices):
                raise RuntimeError("Prescribed input visible in hidden rollout")
            if row["target_availability"].get("visible"):
                raise RuntimeError("Target mask unexpectedly visible")
        destination = Path(selected["plots"])
        if destination.exists():
            raise FileExistsError(destination)
        command = [sys.executable, str(PACKAGE / "validation/render_canonical_rollout.py"),
                   "--rollout", str(rollout)]
        with (HERE / f"best{selected['iteration']}_render.log").open("x") as log:
            subprocess.run(command, cwd=ROOT, env=dict(os.environ, MPLBACKEND="Agg"),
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
        receipt_path = rollout / "plots/plot_receipt.json"
        receipt = json.loads(receipt_path.read_text())
        source = Path(receipt["figures"][0]).parent
        if source.parent != ROOT / "plot" or len(receipt["figures"]) != 7:
            raise RuntimeError("Canonical output must be seven figures under root plot")
        source.rename(destination)
        receipt["figures"] = [str(destination / Path(f).name) for f in receipt["figures"]]
        content = json.dumps(receipt, indent=2) + "\n"
        receipt_path.write_text(content)
        (destination / "plot_receipt.json").write_text(content)
        hashes = {}
        for filename in receipt["figures"]:
            png = Path(filename)
            with Image.open(png) as image:
                image.verify()
            hashes[png.name] = hashlib.sha256(png.read_bytes()).hexdigest()
        if len(list(destination.glob("*.png"))) != 7:
            raise RuntimeError("Unexpected number of PNGs")
        for name, digest in selected["source_hashes"].items():
            if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"Checkpoint modified: {name}")
        missing = (destination / "missing_channels.txt").read_text()
        result = dict(iteration=selected["iteration"], label=selected["label"],
                      training_return=selected["training_return"],
                      rollout=str(rollout), plots=str(destination),
                      steps=summary["steps"], episode_return=summary["episode_return"],
                      end_reason=summary["end_reason"], terminated=summary["terminated"],
                      truncated=summary["truncated"],
                      valid_cycles=summary["phase_valid_cycle_count"],
                      invalid_events=summary["invalid_event_count"],
                      penetration_max_mm=1000*summary["grf_penetration_max_m"],
                      last_time=trace[-1]["time"], hidden_inputs_verified=True,
                      source_hashes_unchanged=True, png_count=7, png_sha256=hashes,
                      missing_channels_report=missing)
        (destination / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
        results.append(result)
        print(json.dumps(result), flush=True)
    (HERE / "results.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
