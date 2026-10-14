"""Freeze/run one coherent FOURROI case; reuse the completed fresh400 control.

Preparation only constructs paths and the frozen receipt. The native command
delegates the original run_case bytecode with only DYNAMIC redirected.
"""
from datetime import datetime, timezone
import argparse
import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

PREPARATION = Path(__file__).resolve().parent
AUDIT = PREPARATION.parent
ROOT = AUDIT.parents[1]
OLD = ROOT / "validation/ab07_pylon_necessity_screening_2026_10_07"
PAIR = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
PROTOCOL = AUDIT / "full_support_dynamic_protocol.json"
PREP_PROTOCOL = PREPARATION / "preparation_protocol.json"
CASE = "STANDING_FOURROI_AB07"
sys.path.insert(0, str(ROOT))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


preparation = module("four_roi_preparation_source", PREPARATION / "prepare_four_roi_case.py")
base = preparation.base
original = module("four_roi_original_dynamic_runner", OLD / "run_dynamic_screen.py")
read, source, fp, recheck, write_new = base.read, base.source, base.fp, base.recheck, base.write_new


def prepare():
    if PROTOCOL.exists():
        raise FileExistsError("Preserve the frozen singlecase dynamic protocol")
    from setup_io import read_setup_xml
    prep = read(PREP_PROTOCOL)
    protected = dict(prep["protected_sources_before"])
    recheck(protected)
    if set(prep["cases"]) != {CASE}:
        raise AssertionError("Require the single prospective FOURROI case")
    prepared_case = prep["cases"][CASE]
    prep_receipt_path = source(prepared_case["case_dir"]) / "receipt.json"
    prep_receipt = read(prep_receipt_path)
    if prep_receipt["status"] != "completed" or prep_receipt["case"] != prepared_case or prep_receipt["protocol"] != fp(PREP_PROTOCOL):
        raise AssertionError("Require the completed coherent FOURROI preparation")
    for key in ("output_model", "full_IK"):
        if not base.matches_fp(prep_receipt[key]):
            raise AssertionError("Own prepared model/full IK changed")
    control_path = source(prep["finalized_fresh400_receipt"]["path"])
    if fp(control_path) != prep["finalized_fresh400_receipt"]:
        raise AssertionError("Finalized fresh400 receipt changed")
    control = read(control_path)
    if control["status"] != "completed" or not control["completed_requested_duration"] or control["steps"] != 611:
        raise AssertionError("Fresh400 reuse requires its completed final receipt")
    old_pair_path = ROOT / "results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json"
    old_pair = read(old_pair_path)
    historical_path = OLD / "dynamics/P3_A1/run/receipt.json"
    selection = read(PAIR / "selection.json")
    subject = selection["subjects"]["AB07"]
    start, duration = subject["reset_hs_s"], 6.11
    original_setup = PAIR / "AB07/matched_setup.xml"
    parsed_old = read_setup_xml(original_setup)
    dynamic_root = AUDIT / "full_support_dynamics"
    if dynamic_root.exists():
        raise FileExistsError("Preserve any FOURROI dynamic preparation attempt")
    dynamic_root.mkdir()
    folder = dynamic_root / CASE
    folder.mkdir()
    setup = folder / "setup.xml"
    tree = ET.parse(original_setup)
    tree.find(".//model_file").text = prep_receipt["output_model"]["path"]
    tree.find(".//kinematics_file").text = prep_receipt["full_IK"]["path"]
    tree.write(setup, encoding="utf-8", xml_declaration=True)
    parsed = read_setup_xml(setup)
    preserved = ("external_loads_xml", "reserve_actuators_xml", "t_start", "t_end", "grf_mode", "online_grf_profile_file")
    if any(getattr(parsed, key) != getattr(parsed_old, key) for key in preserved):
        raise AssertionError("Original 9 s horizon, loads or profiles changed")
    kwargs = copy.deepcopy(old_pair["env_kwargs"])
    kwargs.update(setup_xml_path=str(setup), output_dir=str(folder / "run/sim_outputs"), episode_duration=duration)
    differences = {key: {"previous": old_pair["env_kwargs"].get(key), "standing": value}
                   for key, value in kwargs.items() if old_pair["env_kwargs"].get(key) != value}
    if set(differences) - {"setup_xml_path", "output_dir", "episode_duration"} or not math.isclose(parsed.t_start + kwargs["episode_start_offset_s"], start, abs_tol=1e-12):
        raise AssertionError("Runtime numerical options or reset offset changed")
    case = {"id": CASE, "subject": "AB07", "role": "candidate_four_ROI",
            "preparation": fp(prep_receipt_path), "model": prep_receipt["output_model"], "IK": prep_receipt["full_IK"],
            "length_mm": prepared_case["distal_mount_length_mm"], "static_ankle_offset_deg": prepared_case["static_offsets_deg"]["pros_ankle_angle"],
            "duration_s": duration}
    receipt = original.Q.prepare_receipt(str(setup), kwargs, "left_applied")
    receipt.update(subject="AB07", case=case, requested_time_range=[start, start + duration],
                   old_matched_runtime=fp(old_pair_path), historical_control_receipt=fp(historical_path),
                   finalized_fresh400_receipt=fp(control_path), setup_original=fp(original_setup),
                   differences_from_original_pair=differences, source_selection=fp(PAIR / "selection.json"),
                   primary_cycles=subject["primary_cycles"], output_directory=str(folder / "run"), seed=123,
                   preparation=fp(prep_receipt_path), geometry_fit_receipt=prep["geometry_fit_receipt"],
                   action_source="Original prescribed_action from this case's own full IK; unchanged cascade and SEA spring/plugin",
                   physical_qualification="Reserve-assisted diagnostic of coherent FOURROI preparation; completed fresh400 reused without new integration")
    write_new(folder / "prepared_receipt.json", receipt)
    case["prepared_receipt"] = fp(folder / "prepared_receipt.json")
    for item in receipt["source_files"].values():
        base.add_protected(protected, fp(item["path"]))
    for path in (Path(__file__), PREPARATION / "prepare_four_roi_case.py", PREP_PROTOCOL, prep_receipt_path, setup,
                 folder / "prepared_receipt.json", OLD / "run_dynamic_screen.py", PAIR / "selection.json",
                 original_setup, old_pair_path, historical_path, control_path):
        base.add_protected(protected, fp(path))
    recheck(protected)
    value = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "frozen_before_single_FOURROI_native_run",
             "instruction_check_token": "CMC_AGENT_OK_2026", "scope_protocol": fp(AUDIT / "protocol.json"),
             "preparation_protocol": fp(PREP_PROTOCOL), "geometry_fit_receipt": prep["geometry_fit_receipt"],
             "cases": {CASE: case}, "execution_order": [CASE], "finalized_fresh400_receipt": fp(control_path),
             "control_reuse": "Finalized PYLON400_REFERENCE statistics/phase may be reused by exact fingerprints; no new control preparation/integration",
             "contract": {"seed": 123, "episode_duration_s": duration, "same_original_9s_setup_horizon": True,
                          "same_original_prescribed_action": True, "same_runtime_controller_plugin_profiles": True,
                          "same_reset_and_three_nominal_cycles": True, "network": False,
                          "original_runner": fp(OLD / "run_dynamic_screen.py"),
                          "delegation": "Copy original run_case globals and redirect only DYNAMIC; original bytecode/remaining globals untouched"},
             "recording": "Original trace/physical outputs; pre-step samples and closing-event right censoring remain explicit",
             "failure_policy": "Exactly one extra native attempt; no overwrite/retry/window substitution/outcome retuning; preserve partial data and original two-region case",
             "protected_count": len(protected), "protected_sources_before": protected}
    write_new(PROTOCOL, value)
    print({"protocol": fp(PROTOCOL), "case": CASE, "protected_count": len(protected)}, flush=True)


def run_case():
    copied = dict(original.run_case.__globals__)
    copied["DYNAMIC"] = PROTOCOL
    delegated = types.FunctionType(original.run_case.__code__, copied, original.run_case.__name__, original.run_case.__defaults__, original.run_case.__closure__)
    delegated.__kwdefaults__ = original.run_case.__kwdefaults__
    if delegated.__code__ is not original.run_case.__code__:
        raise AssertionError("Original native runner bytecode changed")
    return delegated(CASE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--prepare", action="store_true")
    commands.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if args.prepare:
        prepare()
        return 0
    return run_case()


if __name__ == "__main__":
    raise SystemExit(main())
