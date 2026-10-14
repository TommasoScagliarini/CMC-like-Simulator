"""Estimator-only synthetic pause diagnostic, no retuning or physical stop claim."""
from pathlib import Path
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "tnt_preliminary_2026_10_13"
sys.path.insert(0, str(PREVIOUS))
from tnt_replay import Parameters, write_json
from kinematic_generator import stream_replay


def main():
    results = []
    for subject in ["ab06", "ab07"]:
        data = np.genfromtxt(PREVIOUS / (subject+"_prescribed.csv"), delimiter=",", names=True)
        bundle = json.loads((HERE / (subject+"_bundle.json")).read_text())
        t, theta = data["time_s"], data["thigh_deg"]
        parameters = Parameters(**bundle["parameters"])
        kwargs = dict(method="blend", initial_stance_duration=bundle["initial_stance_duration_s"])
        baseline = stream_replay(t, theta, parameters, **kwargs)
        for stage in [0, 1, 2, 3]:
            i = next(i for i in range(5000, len(t)-500)
                     if baseline["state"][i] == stage and baseline["state"][i-2] == stage)
            signal = np.r_[theta[:i+1], np.full(200, theta[i]), theta[i+1:]]
            now = t[0] + np.arange(len(signal)) * .01
            r = stream_replay(now, signal, parameters, **kwargs)
            invalid = np.flatnonzero(r["state"] < 0)
            results.append({
                "subject": subject, "freeze_stage": stage, "freeze_duration_s": 2,
                "freeze_start_s": float(t[i]),
                "first_invalid_s": float(now[invalid[0]]) if len(invalid) else None,
                "maximum_T": float(r["T"].max()), "completed": int(r["completed"][-1]),
                "pause_phase_start": float(r["T"][i]), "pause_phase_end": float(r["T"][i+200]),
            })
    write_json(HERE / "synthetic_pause_diagnostic.json", {
        "scope": "Synthetic 2s held thigh at each internal stage; exploratory estimator-only test, not a physically realistic stop",
        "limitation": "An early-swing pause advances T beyond its domain before the existing middle-entry INVALID guard fires. The blend does not provide a stop/recovery mechanism. Template clipping is not a validated stop policy.",
        "results": results,
    })


if __name__ == "__main__":
    main()
