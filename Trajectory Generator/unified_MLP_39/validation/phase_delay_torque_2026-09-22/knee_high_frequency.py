"""Offline high-frequency torque comparison; no controller or training changes."""
from pathlib import Path
import json
import hashlib
import io
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
RUNS = {
    "last40": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "oracle_left": PACKAGE / "runs/rollout/causal_difference_2026-09-21_oracle_left",
    "v26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}


def main():
    result = {"method": "Fourth-order Butterworth high-pass at 20 Hz, zero-phase offline; full trace filtered then cropped to 13.15–17.95 s. Dominant frequency from Welch PSD within 20–200 Hz.",
              "limitations": "Descriptive fixed-time comparison; V26 best87 is more mature. This does not predict improvement in ex-novo training.",
              "runs": {}, "hashes": {}}
    for name, folder in RUNS.items():
        result["runs"][name] = {}
        for suffix, channels in (("sea_torques", ["SEA_Knee_tau_spring", "SEA_Ankle_tau_spring"]),
                                 ("sea_diagnostics", ["SEA_Knee_tau_input_plugin", "SEA_Ankle_tau_input_plugin"])):
            path = folder / "sim_outputs" / f"rollout_episode_{suffix}.sto"
            result["hashes"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            content = path.read_text().split("endheader", 1)[1].strip()
            names = content.splitlines()[0].split()
            values = np.loadtxt(io.StringIO(content), skiprows=1)
            time = values[:, 0]
            dt = float(np.median(np.diff(time)))
            assert np.max(np.abs(np.diff(time) - dt)) < 1e-7
            fs = 1 / dt
            filt = butter(4, 20, fs=fs, btype="highpass", output="sos")
            mask = (time >= 13.15) & (time <= 17.95)
            for channel in channels:
                signal = values[:, names.index(channel)]
                filtered = sosfiltfilt(filt, signal)[mask]
                window = signal[mask]
                freq, psd = welch(filtered, fs=fs, nperseg=min(2048, len(filtered)))
                band = (freq >= 20) & (freq <= 200)
                result["runs"][name][channel] = {
                    "samples": int(mask.sum()), "rms_nm": float(np.sqrt(np.mean(window ** 2))),
                    "hp20_rms_nm": float(np.sqrt(np.mean(filtered ** 2))),
                    "peak_frequency_in_20_200hz": float(freq[band][np.argmax(psd[band])]),
                    "abs_max_nm": float(np.max(np.abs(window))),
                }
    (HERE / "knee_high_frequency.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["runs"], indent=2))


if __name__ == "__main__":
    main()
