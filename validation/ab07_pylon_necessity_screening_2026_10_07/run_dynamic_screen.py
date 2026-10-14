"""Matched physical screening with original prescribed_action and SEA plugin.

No neural policy or production edits. All prepared cells are selected before
outputs. Keep the old setup horizon; shorten only the episode duration, retaining
the original startup and the full three frozen primary measurement cycles.
"""
from datetime import datetime, timezone
from dataclasses import asdict
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[1]
TG = ROOT / "Trajectory Generator"
PREP = AUDIT / "preparation/preparation_protocol.json"
SUPPLEMENT = AUDIT / "preparation/supplement_pose/supplement_protocol.json"
DYNAMIC = AUDIT / "dynamic_protocol.json"
PAIR = TG / "validation/paired_prescribed_AB06_AB07_2026_10_07"
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("pylon_original_qualifier", TG / "validation/AB07/qualify_prescribed_v26.py")
Q = importlib.util.module_from_spec(spec)
spec.loader.exec_module(Q)


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def fp(path):
    path = Path(path).resolve()
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def protected_check(manifest):
    for name, sha in manifest.items():
        path = Path(name) if Path(name).is_absolute() else ROOT / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != sha:
            raise AssertionError("Protected source changed: " + name)


def write_new(path, obj):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(Q._json_value(obj), stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def prepare():
    if DYNAMIC.exists():
        raise FileExistsError("Preserve frozen dynamic protocol")
    from setup_io import read_setup_xml
    scope = read(AUDIT / "protocol.json")
    prep = read(PREP)
    supplement = read(SUPPLEMENT)
    protected = dict(scope["original_and_previous_audit_sources_before"])
    for preparation_protocol in (prep, supplement):
        for name, sha in preparation_protocol["protected_sources_before"].items():
            if name in protected and protected[name] != sha:
                raise AssertionError("Conflicting source seals")
            protected[name] = sha
    cases = {}
    all_prepared_cases = dict(prep["cases"])
    if set(all_prepared_cases).intersection(supplement["cases"]):
        raise AssertionError("Supplement must not replace a main screen case")
    all_prepared_cases.update(supplement["cases"])
    if len(prep["cases"]) != 14 or set(supplement["cases"]) != {"P0_M2", "P0_M5", "P0_M10"}:
        raise AssertionError("The full main grid and prospective pose supplement are required")
    for key, case in all_prepared_cases.items():
        receipt_path = ROOT / case["case_dir"] / "receipt.json"
        receipt = read(receipt_path)
        if receipt["status"] != "completed" or receipt["case"] != case:
            raise AssertionError("All seventeen coherent preparations must be complete")
        for role in ("output_model", "full_IK"):
            item = receipt[role]
            if fp(ROOT / item["path"]) != item:
                raise AssertionError("Prepared model/own IK changed")
        cases[key] = {"id": key, "subject": "AB07", "preparation": fp(receipt_path),
                      "model": receipt["output_model"], "IK": receipt["full_IK"],
                      "length_mm": case["distal_mount_length_mm"],
                      "static_ankle_offset_deg": case["static_ankle_offset_deg"], "duration_s": 6.11}
    a_receipt = read(ROOT / "results/AB06_paired_prescribed_HS_9s_2026_10_07/receipt.json")
    cases["AB06_control"] = {"id": "AB06_control", "subject": "AB06", "preparation": None,
                             "model": {"path": str(Path(a_receipt["source_files"]["model"]["path"]).relative_to(ROOT)), "sha256": a_receipt["source_files"]["model"]["sha256"]},
                             "IK": {"path": str(Path(a_receipt["source_files"]["ik"]["path"]).relative_to(ROOT)), "sha256": a_receipt["source_files"]["ik"]["sha256"]},
                             "length_mm": 430., "static_ankle_offset_deg": 0., "duration_s": 6.21}
    root = AUDIT / "dynamics"
    root.mkdir(exist_ok=True)
    selection = read(PAIR / "selection.json")
    for key, c in cases.items():
        subject = c["subject"]
        old_path = ROOT / f"results/{subject}_paired_prescribed_HS_9s_2026_10_07/receipt.json"
        old = read(old_path)
        original_setup = PAIR / subject / "matched_setup.xml"
        parsed_old = read_setup_xml(original_setup)
        folder = root / key
        folder.mkdir()
        setup = folder / "setup.xml"
        tree = ET.parse(original_setup)
        tree.find(".//model_file").text = c["model"]["path"]
        tree.find(".//kinematics_file").text = c["IK"]["path"]
        tree.write(setup, encoding="utf-8", xml_declaration=True)
        parsed = read_setup_xml(setup)
        if any(getattr(parsed, k) != getattr(parsed_old, k) for k in ("external_loads_xml", "reserve_actuators_xml", "t_start", "t_end", "grf_mode", "online_grf_profile_file")):
            raise AssertionError("Setup horizon or physical loads changed")
        kwargs = copy.deepcopy(old["env_kwargs"])
        kwargs.update(setup_xml_path=str(setup), output_dir=str(folder / "run/sim_outputs"), episode_duration=c["duration_s"])
        allowed = {"setup_xml_path", "output_dir", "episode_duration"}
        differences = {k: {"previous": old["env_kwargs"].get(k), "screen": v} for k, v in kwargs.items() if old["env_kwargs"].get(k) != v}
        if set(differences) - allowed:
            raise AssertionError("Other runtime options changed")
        start = selection["subjects"][subject]["reset_hs_s"]
        if not math.isclose(parsed.t_start + kwargs["episode_start_offset_s"], start, abs_tol=1e-12):
            raise AssertionError("Reset offset changed")
        receipt = Q.prepare_receipt(str(setup), kwargs, "left_applied")
        receipt.update(subject=subject, case=c, requested_time_range=[start, start+c["duration_s"]],
                       old_matched_runtime=fp(old_path), setup_original=fp(original_setup),
                       differences_from_original_pair=differences, source_selection=fp(PAIR / "selection.json"),
                       primary_cycles=selection["subjects"][subject]["primary_cycles"],
                       output_directory=str(folder / "run"), seed=123,
                       action_source="Same original prescribed_action from this case's own full IK; existing cascade and non-impedance SEA spring/plugin integrated",
                       physical_qualification="Completion is a reserve-assisted tracking result, not physical mount necessity")
        write_new(folder / "prepared_receipt.json", receipt)
        c["prepared_receipt"] = fp(folder / "prepared_receipt.json")
        for item in receipt["source_files"].values():
            path = Path(item["path"])
            protected[fp(path)["path"]] = fp(path)["sha256"]
        for path in (original_setup, old_path, setup, folder / "prepared_receipt.json"):
            protected[fp(path)["path"]] = fp(path)["sha256"]
    for path in (Path(__file__), PREP, SUPPLEMENT, AUDIT / "protocol.json", PAIR / "selection.json"):
        protected[fp(path)["path"]] = fp(path)["sha256"]
    protected_check(protected)
    amendment = {"created_utc": datetime.now(timezone.utc).isoformat(),
                 "prospective_before_any_dynamic_output": True,
                 "scope_protocol": fp(AUDIT / "protocol.json"), "preparation_protocol": fp(PREP),
                 "change": {"AB07_episode_duration_s": {"planned": 6.10, "executed": 6.11}},
                 "reason": "Recorder samples pre-step: 6.10s would end export 39.671 microseconds before frozen last HS. 6.11s brackets it without clamping or changing measurement windows.",
                 "setup_t_end": "Unchanged original9s horizon to preserve phase/reference diagnostics; only maximum episode duration shortened",
                 "AB06_duration_s": 6.21, "primary_cycles_unchanged": True}
    write_new(AUDIT / "protocol_amendment_01.json", amendment)
    obj = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "frozen_before_all_dynamic_runs",
           "instruction_check_token": "CMC_AGENT_OK_2026", "cases": cases,
           "execution_order": ["P0_A0", "P4_A1", "AB06_control"] + [k for k in cases if k not in ("P0_A0", "P4_A1", "AB06_control")],
           "contract": {"same_existing_prescribed_action": True, "SEA_forward_mode": "plugin", "SEA_Impedence": False,
                        "SEA_outer_mode": "cascade", "integration": "rk4_bypass1ms", "seed": 123,
                        "all14_main_plus3_pose_controls_plus_AB06_control": True,
                        "pose_supplement": "Proportional pylon with static offsets -2/-5/-10 degrees; geometry-informed controls frozen before their outputs, main14 unchanged",
                        "new_network_runs": False, "gains_profiles_or_forcefit_changes": False},
           "recording": "Existing recorder and step; require actual exported coverage through three exact closing HS/TO windows; clock completion alone insufficient",
           "failure_policy": "One attempt per case; preserve interruption/exception and all physical samples. No outcome-dependent retuning/windows. Infrastructure corrections must be separately versioned.",
           "amendment": fp(AUDIT / "protocol_amendment_01.json"),
           "protected_count": len(protected), "protected_sources_before": protected}
    write_new(DYNAMIC, obj)
    print(json.dumps({"protocol": fp(DYNAMIC), "cases": len(cases), "protected_count": len(protected)}), flush=True)


def run_case(name):
    protocol = read(DYNAMIC)
    protected_check(protocol["protected_sources_before"])
    c = protocol["cases"][name]
    receipt = read(ROOT / c["prepared_receipt"]["path"])
    output = Path(receipt["output_directory"])
    if output.exists():
        raise FileExistsError("Preserve any completed or failed attempt")
    output.mkdir()
    for path in (str(ROOT), str(TG), str(TG / "baseline_MLP")):
        if path not in sys.path:
            sys.path.insert(0, path)
    env = None
    info = {}
    steps = 0
    code = 1
    receipt["started_utc"] = datetime.now(timezone.utc).isoformat()
    began = time.monotonic()
    try:
        import win_runtime
        from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
        from prescribed_action import prescribed_action
        env = CMCLikeProsthesisTrajectoryEnv(CMCEnvConfig(**receipt["env_kwargs"]))
        receipt["effective_simulator_config"] = asdict(env.cfg)
        receipt["effective_env_config"] = asdict(env.env_cfg)
        old = read(ROOT / receipt["old_matched_runtime"]["path"])
        allowed = {"model_file", "model_bundle_dir", "kinematics_file", "output_dir"}
        cfg_diff = {k: {"previous": old["effective_simulator_config"].get(k), "screen": v}
                    for k, v in receipt["effective_simulator_config"].items() if old["effective_simulator_config"].get(k) != v}
        if set(cfg_diff) - allowed:
            raise AssertionError("Effective physical configuration drift: " + repr(cfg_diff))
        receipt["effective_configuration_differences"] = cfg_diff
        _, reset = env.reset(seed=123)
        receipt.update(reset=reset, actual_episode_start_s=float(env._episode_start), actual_episode_end_s=float(env._episode_end))
        if not all(math.isclose(a, b, abs_tol=1e-8) for a, b in zip((env._episode_start, env._episode_end), receipt["requested_time_range"])):
            raise AssertionError("Reset clamped the requested window")
        Q._write_json(output / "receipt.json", receipt)
        with (output / "trace.jsonl").open("x", encoding="utf-8") as stream:
            while True:
                _, _, terminated, truncated, info = env.step(prescribed_action(env))
                steps += 1
                row = {k: v for k, v in info.items() if k not in {"so_solver_audit_entries", "log"}}
                stream.write(json.dumps(Q._json_value(row), allow_nan=False)+"\n")
                if steps % 50 == 0 or terminated or truncated:
                    stream.flush()
                    print(f"{name} step={steps} t={env.t:.9f} end={info.get('end_reason')}", flush=True)
                if terminated or truncated:
                    break
        clocks = {"environment": float(env.t), "runner": float(env.runner.current_time), "state": float(env.runner.state.getTime())}
        target = receipt["requested_time_range"][1]
        complete = all(math.isclose(t, target, abs_tol=1e-7) for t in clocks.values()) and info.get("end_reason") in {"dataset_end", "episode_time_limit"} and not info.get("failure") and not terminated
        receipt.update(status="completed" if complete else "interrupted", steps=steps, completed_requested_duration=complete,
                       final_clocks_s=clocks, final_time_s=clocks["environment"], final_info=info,
                       terminated=bool(terminated), truncated=bool(truncated), final_failure=info.get("failure"))
        code = 0 if complete else 1
    except Exception as exc:
        receipt.update(status="failed", steps=steps, error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
        if env is not None and getattr(env, "runner", None) is not None:
            receipt.update(final_time_s=float(env.t), final_runner_time_s=float(env.runner.current_time), final_state_time_s=float(env.runner.state.getTime()))
    finally:
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                receipt["close_error"] = f"{type(exc).__name__}: {exc}"
                code = 1
        receipt.update(wall_time_s=time.monotonic()-began, finished_utc=datetime.now(timezone.utc).isoformat())
        Q._write_json(output / "receipt.json", receipt)
        protected_check(protocol["protected_sources_before"])
        write_new(output / "integrity_receipt.json", {"protected_count": protocol["protected_count"], "all_unchanged": True, "protocol": fp(DYNAMIC)})
        print(json.dumps({"case": name, "status": receipt["status"], "steps": steps, "wall_time_s": receipt["wall_time_s"]}), flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--prepare", action="store_true")
    group.add_argument("--case")
    args = parser.parse_args()
    if args.prepare:
        prepare()
        return 0
    return run_case(args.case)


if __name__ == "__main__":
    raise SystemExit(main())
