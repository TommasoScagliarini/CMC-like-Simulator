"""Freeze/run two standing-fit controls through the original saved runner.

No integration is performed by --prepare. --case delegates the original
run_dynamic_screen.run_case with its original globals, changing only the frozen
dynamic protocol path. Native runtime, seeding, arithmetic, action source,
step/close calls and recording remain the existing implementation.
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
PROTOCOL = AUDIT / "dynamic_protocol.json"
PREP_PROTOCOL = PREPARATION / "preparation_protocol.json"
PAIR = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
sys.path.insert(0, str(ROOT))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


preparation = module("standing_preparation_source", PREPARATION / "prepare_standing_cases.py")
original = module("standing_original_dynamic_runner", OLD / "run_dynamic_screen.py")
read, fp, recheck, write_new = preparation.read, preparation.fp, preparation.recheck, preparation.write_new


def prepare():
    if PROTOCOL.exists():
        raise FileExistsError("Preserve the frozen dynamic protocol")
    from setup_io import read_setup_xml
    prep = read(PREP_PROTOCOL)
    scope = read(AUDIT / "protocol.json")
    protected = dict(prep["protected_sources_before"])
    for name, sha in scope["original_and_previous_audit_sources_before"].items():
        preparation.add_protected(protected, {"path": name, "sha256": sha})
    recheck(protected)
    if set(prep["cases"]) != {"STANDING_AB07", "PYLON400_REFERENCE"}:
        raise AssertionError("Require exactly the two predeclared preparations")
    old_pair_path = ROOT / "results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json"
    old_pair = read(old_pair_path)
    historical_control_path = OLD / "dynamics/P3_A1/run/receipt.json"
    historical_control = read(historical_control_path)
    if historical_control["status"] != "completed" or not historical_control["completed_requested_duration"]:
        raise AssertionError("Require the completed historical P3_A1 control")
    selection = read(PAIR / "selection.json")
    subject = selection["subjects"]["AB07"]
    start = subject["reset_hs_s"]
    duration = 6.11
    original_setup = PAIR / "AB07/matched_setup.xml"
    parsed_old = read_setup_xml(original_setup)
    cases = {}
    dynamic_root = AUDIT / "dynamics"
    if dynamic_root.exists():
        raise FileExistsError("Preserve any dynamic preparation attempt")
    dynamic_root.mkdir()
    for name, prepared_case in prep["cases"].items():
        prep_receipt_path = ROOT / prepared_case["case_dir"] / "receipt.json"
        prep_receipt = read(prep_receipt_path)
        if prep_receipt["status"] != "completed" or prep_receipt["case"] != prepared_case:
            raise AssertionError("Both coherent preparations must be completed")
        if fp(PREP_PROTOCOL) != prep_receipt["protocol"]:
            raise AssertionError("Preparation receipt is bound to a different protocol")
        for key in ("output_model", "full_IK"):
            if fp(prep_receipt[key]["path"]) != prep_receipt[key]:
                raise AssertionError("Prepared model/own IK changed")
        if name == "PYLON400_REFERENCE":
            expected = prepared_case["expected_byte_reference"]
            if any(not prep_receipt["reference_reproduction"][key]["exact_semantic_or_numeric_equal"] for key in ("output_model", "full_IK")):
                raise AssertionError("Fresh 400 mm control has substantive model/IK differences")
            if any(prep_receipt["reference_reproduction"][key]["historical"] != expected[key] for key in ("output_model", "full_IK")):
                raise AssertionError("Fresh 400 mm comparison is not bound to frozen P3_A1")
        folder = dynamic_root / name
        folder.mkdir()
        setup = folder / "setup.xml"
        tree = ET.parse(original_setup)
        tree.find(".//model_file").text = prep_receipt["output_model"]["path"]
        tree.find(".//kinematics_file").text = prep_receipt["full_IK"]["path"]
        tree.write(setup, encoding="utf-8", xml_declaration=True)
        parsed = read_setup_xml(setup)
        preserved_setup = ("external_loads_xml", "reserve_actuators_xml", "t_start", "t_end", "grf_mode", "online_grf_profile_file")
        if any(getattr(parsed, key) != getattr(parsed_old, key) for key in preserved_setup):
            raise AssertionError("Setup loads, profiles or original 9 s horizon changed")
        kwargs = copy.deepcopy(old_pair["env_kwargs"])
        kwargs.update(setup_xml_path=str(setup), output_dir=str(folder / "run/sim_outputs"), episode_duration=duration)
        allowed = {"setup_xml_path", "output_dir", "episode_duration"}
        differences = {key: {"previous": old_pair["env_kwargs"].get(key), "standing": value}
                       for key, value in kwargs.items() if old_pair["env_kwargs"].get(key) != value}
        if set(differences) - allowed or not math.isclose(parsed.t_start + kwargs["episode_start_offset_s"], start, abs_tol=1e-12):
            raise AssertionError("Runtime options or reset offset changed")
        case = {"id": name, "subject": "AB07", "role": "candidate" if name == "STANDING_AB07" else "control",
                "preparation": fp(prep_receipt_path), "model": prep_receipt["output_model"], "IK": prep_receipt["full_IK"],
                "length_mm": prepared_case["distal_mount_length_mm"],
                "static_ankle_offset_deg": prepared_case["static_offsets_deg"]["pros_ankle_angle"], "duration_s": duration}
        receipt = original.Q.prepare_receipt(str(setup), kwargs, "left_applied")
        receipt.update(subject="AB07", case=case, requested_time_range=[start, start + duration],
                       old_matched_runtime=fp(old_pair_path), historical_control_receipt=fp(historical_control_path),
                       setup_original=fp(original_setup), differences_from_original_pair=differences,
                       source_selection=fp(PAIR / "selection.json"), primary_cycles=subject["primary_cycles"],
                       output_directory=str(folder / "run"), seed=123, preparation=fp(prep_receipt_path),
                       action_source="Original prescribed_action from this case's own full IK; unchanged cascade and SEA spring/plugin",
                       physical_qualification="A reserve-assisted diagnostic of two coherent preparations; no gait/physiological/hardware necessity qualification")
        write_new(folder / "prepared_receipt.json", receipt)
        case["prepared_receipt"] = fp(folder / "prepared_receipt.json")
        cases[name] = case
        for item in receipt["source_files"].values():
            preparation.add_protected(protected, fp(item["path"]))
        for path in (prep_receipt_path, setup, folder / "prepared_receipt.json"):
            preparation.add_protected(protected, fp(path))
    for path in (Path(__file__), PREPARATION / "prepare_standing_cases.py", OLD / "run_dynamic_screen.py",
                 AUDIT / "protocol.json", PREP_PROTOCOL, PAIR / "selection.json", original_setup,
                 old_pair_path, historical_control_path):
        preparation.add_protected(protected, fp(path))
    recheck(protected)
    value = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "frozen_before_both_native_runs",
             "instruction_check_token": "CMC_AGENT_OK_2026", "scope_protocol": fp(AUDIT / "protocol.json"),
             "preparation_protocol": fp(PREP_PROTOCOL), "cases": cases,
             "execution_order": ["PYLON400_REFERENCE", "STANDING_AB07"],
             "contract": {"seed": 123, "episode_duration_s": duration, "same_original_9s_setup_horizon": True,
                          "same_original_prescribed_action": True, "same_runtime_controller_plugin_profiles": True,
                          "same_reset_and_three_nominal_cycles": True, "network": False,
                          "original_runner": fp(OLD / "run_dynamic_screen.py"),
                          "delegation": "Copy original run_case globals and redirect only DYNAMIC; original bytecode/remaining globals untouched"},
             "recording": "Original trace and physical outputs; pre-step samples, physical coverage and event-window right censoring remain explicit",
             "failure_policy": "One attempt per case, no overwrite/retry/window substitution/outcome retuning; preserve incomplete data",
             "protected_count": len(protected), "protected_sources_before": protected}
    write_new(PROTOCOL, value)
    print({"protocol": fp(PROTOCOL), "cases": list(cases), "protected_count": len(protected)}, flush=True)


def run_case(name):
    copied_globals = dict(original.run_case.__globals__)
    copied_globals["DYNAMIC"] = PROTOCOL
    delegated = types.FunctionType(original.run_case.__code__, copied_globals,
                                    original.run_case.__name__, original.run_case.__defaults__, original.run_case.__closure__)
    delegated.__kwdefaults__ = original.run_case.__kwdefaults__
    if delegated.__code__ is not original.run_case.__code__:
        raise AssertionError("Original native runner bytecode changed")
    return delegated(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--prepare", action="store_true")
    commands.add_argument("--case", choices=("PYLON400_REFERENCE", "STANDING_AB07"))
    args = parser.parse_args()
    if args.prepare:
        prepare()
        return 0
    return run_case(args.case)


if __name__ == "__main__":
    raise SystemExit(main())
