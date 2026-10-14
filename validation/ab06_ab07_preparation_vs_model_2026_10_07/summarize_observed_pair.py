"""Read saved paired-test metrics and record input provenance; never simulate.

Run with Python 3 from any working directory. Uses only the standard library.
The force decomposition is arithmetic, not a dynamical mass counterfactual.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent
PAIR = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    metrics_path = PAIR / "comparison_metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    previous = json.loads((PAIR / "result_summary.json").read_text(encoding="utf-8"))
    records = []
    for item in previous["protected_inputs"]["checks"]:
        path = ROOT / item["path"]
        current = sha256(path)
        records.append({"path": item["path"], "sha256": current,
                        "paired_test_sha256": item["sha256"],
                        "matches_paired_test": current == item["sha256"]})
    additional = [metrics_path, PAIR / "result_summary.json",
                  ROOT / "models/AB06-raw/osimxml/AB06.osim",
                  ROOT / "models/AB07-raw/osimxml/ab07.osim",
                  ROOT / "models/AB07_SEASEA_Threadmill/pipeline_report.json",
                  ROOT / "tools/build_abx_seasea_pipeline.py",
                  ROOT / "tools/fit_seasea_subject.py",
                  ROOT / "tools/register_seasea_ground.py",
                  ROOT / "tools/convert_epic_ab06_to_opensim.m",
                  ROOT / "tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp"]
    additional.extend(
        path for path in (ROOT / "plugins").iterdir()
        if path.is_file() and path.suffix.lower() in (".dll", ".dylib", ".so")
        and any(name in path.name for name in
                ("SEA_Plugin_BlackBox_mCMC_impedence_ff", "OnlineGRFContact"))
    )
    for subject in ("AB06", "AB07"):
        output = ROOT / "results" / f"{subject}_paired_prescribed_HS_9s_2026_10_07"
        additional.extend(path for path in output.rglob("*") if path.is_file())
    seen = {item["path"] for item in records}
    for path in sorted(set(additional)):
        relative = path.relative_to(ROOT).as_posix()
        if relative not in seen:
            records.append({"path": relative, "sha256": sha256(path)})
            seen.add(relative)
    if not all(item.get("matches_paired_test", True) for item in records):
        raise RuntimeError("An original paired-test protected input has changed.")
    results = {}
    for subject, item in metrics["results"].items():
        aggregate = item["primary_aggregate"]
        cycles = item["primary_cycles_frozen"]
        duration = cycles[-1]["next_hs_s"] - cycles[0]["hs_s"]
        impulse = aggregate["vertical_impulse_ns"]
        measured = aggregate["measured_vertical_impulse_ns"]
        body_weight = item["normalization"]["BW_n"]
        results[subject] = {
            "mass_kg": item["normalization"]["mass_kg"],
            "BW_n": body_weight, "primary_duration_s": duration,
            "primary_cycles": cycles,
            "simulated_impulse_ns": impulse,
            "measured_impulse_ns": measured,
            "impulse_error_percent": 100.0 * (impulse / measured - 1.0),
            "simulated_mean_force_n": impulse / duration,
            "measured_mean_force_n": measured / duration,
            "simulated_mean_force_bw": impulse / duration / body_weight,
            "measured_mean_force_bw": measured / duration / body_weight,
            "simulated_peak_bw": aggregate["peak_vertical_force_bw"],
            "measured_peak_bw": aggregate["measured_peak_vertical_force_bw"],
            "native_receipt_status": item["receipt_status"],
        }
    first, second = results["AB06"], results["AB07"]
    mean_sim_ratio = second["simulated_mean_force_n"] / first["simulated_mean_force_n"]
    mean_measured_ratio = second["measured_mean_force_n"] / first["measured_mean_force_n"]
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "scope": "Saved-data audit; no integration, inference, IK optimization or input mutation.",
        "subjects": results,
        "AB07_over_AB06": {
            "body_weight_ratio": second["BW_n"] / first["BW_n"],
            "simulated_mean_force_ratio": mean_sim_ratio,
            "measured_mean_force_ratio": mean_measured_ratio,
            "sim_to_measured_ratio_factor": mean_sim_ratio / mean_measured_ratio,
            "measured_mean_force_bw_ratio": second["measured_mean_force_bw"] / first["measured_mean_force_bw"],
            "simulated_mean_force_bw_ratio": second["simulated_mean_force_bw"] / first["simulated_mean_force_bw"],
        },
        "interpretation": [
            "Percentage errors use each subject's measured left GRF over its three frozen nominal cycles.",
            "BW cancels from the simulated/measured impulse ratio within each subject.",
            "AB07 has lower measured absolute force but higher simulated absolute force in these windows.",
            "Dividing by BW is a unit conversion; it does not isolate the causal influence of physical model mass.",
            "Equal SEA and runtime settings isolate neither anatomy, preparation, kinematics nor right measured loading.",
            "AB06 physical tables cover all three primary cycles despite the original final adapter rounding failure.",
        ],
        "source": {"path": metrics_path.relative_to(ROOT).as_posix(), "sha256": sha256(metrics_path)},
    }
    AUDIT.mkdir(parents=True, exist_ok=True)
    (AUDIT / "observed_pair_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    manifest = {"created_utc": summary["created_utc"], "original_pair_protected_count": 44,
                "all_original_pair_inputs_match": True, "sources": records}
    before_path = AUDIT / "protected_sources_before.json"
    if before_path.exists():
        frozen = json.loads(before_path.read_text(encoding="utf-8"))
        for item in frozen["sources"]:
            if sha256(ROOT / item["path"]) != item["sha256"]:
                raise RuntimeError("An initial audit source has changed: " + item["path"])
    else:
        before_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"subjects": results, "ratios": summary["AB07_over_AB06"],
                      "source_count": len(records)}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
