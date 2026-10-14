"""Matched, IK-driven AB06/AB07 diagnostics using the unchanged V26 test-1 contract.

Prepare the protocol and both receipts before launching either subject. Native
workers sample prescribed_action and call env.step exactly as the original
qualifier; added observations only attest reset and physical clocks.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import traceback
from xml.etree import ElementTree as ET

REPO = Path(__file__).resolve().parents[3]
TG = REPO / "Trajectory Generator"
AUDIT = Path(__file__).resolve().parent


def load_qualifier():
    path = TG / "validation/AB07/qualify_prescribed_v26.py"
    spec = importlib.util.spec_from_file_location("paired_original_qualifier", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


Q = load_qualifier()


def write_json(path, value):
    Q._write_json(path, value)


def fingerprint(path):
    return Q._fingerprint(path)


def assert_fingerprints(entries):
    checks = []
    for entry in entries:
        actual = fingerprint(entry["path"])
        checks.append({**entry, "unchanged": actual["sha256"] == entry["sha256"]})
    if not all(item["unchanged"] for item in checks):
        raise RuntimeError("A frozen input changed: " + str([x["path"] for x in checks if not x["unchanged"]]))
    return checks


def prepare():
    import yaml
    sys.path.insert(0, str(REPO))
    from setup_io import read_setup_xml

    selection_path = AUDIT / "selection.json"
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    source = yaml.safe_load(Q.SOURCE_CONFIG.read_text(encoding="utf-8"))
    baseline = json.loads((REPO / "results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1/receipt.json").read_text(encoding="utf-8"))
    receipts = {}
    for subject in ("AB06", "AB07"):
        chosen = selection["subjects"][subject]
        original = Q._path(chosen["setup_original"])
        parsed = read_setup_xml(original)
        start, duration = float(chosen["reset_hs_s"]), float(chosen["duration_s"])
        if not math.isclose(duration, 9.0, abs_tol=1e-12):
            raise ValueError("This paired protocol requires nine seconds")
        if not math.isclose(start - parsed.t_start, chosen["start_offset_s"], abs_tol=1e-9):
            raise ValueError("Selection offset differs from the original setup")
        folder = AUDIT / subject
        if (folder / "prepared_receipt.json").exists():
            raise FileExistsError("Protocol already prepared; do not overwrite it")
        folder.mkdir(parents=True, exist_ok=True)
        setup = folder / "matched_setup.xml"
        tree = ET.parse(original)
        tree.getroot().find(".//t_end").text = repr(start + duration)
        tree.write(setup, encoding="utf-8", xml_declaration=True)
        isolated = read_setup_xml(setup)
        if any(getattr(parsed, key) != getattr(isolated, key) for key in
               ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml", "t_start")):
            raise RuntimeError("Isolated setup changed a field other than t_end")
        output = REPO / "results" / f"{subject}_paired_prescribed_HS_9s_2026_10_07"
        if output.exists() and any(output.iterdir()):
            raise FileExistsError(f"Refusing nonempty output: {output}")
        kwargs = Q.build_env_kwargs(source, setup=str(setup), output_dir=str(output), stage="left_applied", duration=duration)
        kwargs["episode_start_offset_s"] = start - parsed.t_start
        allowed = {"setup_xml_path", "output_dir", "episode_start_offset_s", "episode_duration"}
        delta = {key: {"test1": baseline["env_kwargs"].get(key), "paired": value}
                 for key, value in kwargs.items() if baseline["env_kwargs"].get(key) != value}
        if set(delta) - allowed or set(kwargs) != set(baseline["env_kwargs"]):
            raise RuntimeError(f"Contract differs from test 1: {delta}")
        receipt = Q.prepare_receipt(str(setup), kwargs, "left_applied")
        receipt.update(subject=subject,
                       action_source=f"{subject} setup IK via unchanged prescribed_action; no network",
                       requested_time_range=[start, start + duration],
                       selection=fingerprint(selection_path),
                       protocol="matched_HS_9s_startup_plus_three_nominal_cycles",
                       source_setup=fingerprint(original), output_directory=str(output),
                       differences_from_test1=delta,
                       isolated_setup_changes={"t_end": {"original": parsed.t_end, "paired": isolated.t_end}},
                       comparison_cycles=chosen["primary_cycles"], startup_cycle=chosen["startup_cycle"])
        receipt["diagnostic_overrides"]["episode_start_offset_s"] = kwargs["episode_start_offset_s"]
        receipt["profiles_status"] = "Unchanged shared V26 profiles; this is a paired diagnostic, not a qualification"
        write_json(folder / "prepared_receipt.json", receipt)
        receipts[subject] = receipt

    a, b = (receipts[key]["env_kwargs"] for key in ("AB06", "AB07"))
    allowed = {"setup_xml_path", "output_dir", "episode_start_offset_s"}
    diff = {key: {"AB06": a[key], "AB07": b[key]} for key in a if a[key] != b[key]}
    if set(diff) - allowed:
        raise RuntimeError(f"Pair settings differ: {diff}")
    paths = set(REPO.glob("*.py")) | set(TG.glob("*.py"))
    paths.update({Path(__file__), selection_path, TG / "validation/AB07/qualify_prescribed_v26.py"})
    for receipt in receipts.values():
        paths.add(Path(receipt["source_setup"]["path"]))
        paths.update(Path(x["path"]) for x in receipt["source_files"].values())
        paths.update(Path(x["path"]) for x in receipt["profile_association"].values())
    manifest = [fingerprint(p) for p in sorted(paths)]
    write_json(AUDIT / "protocol.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "subjects": list(receipts),
        "duration_s": 9.0, "seed": 123, "pair_env_differences": diff,
        "physical_controller_settings_equal": True, "network_loaded": False,
        "selection": fingerprint(selection_path), "protected_inputs": manifest,
        "interpretation": "Subject models and experimental datasets differ. This comparison does not isolate anatomy from kinematics or measured right GRF.",
        "failure_policy": "One attempt per subject; retain any startup failure and incomplete measurement cycles. No retuning or opportunistic window changes."
    })
    print(json.dumps({"status": "prepared", "protected_inputs": len(manifest), "subjects": {s: r["requested_time_range"] for s, r in receipts.items()}}), flush=True)


def run(subject):
    protocol = json.loads((AUDIT / "protocol.json").read_text(encoding="utf-8"))
    assert_fingerprints(protocol["protected_inputs"])
    receipt = json.loads((AUDIT / subject / "prepared_receipt.json").read_text(encoding="utf-8"))
    output = Path(receipt["output_directory"])
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Refusing nonempty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    for entry in (str(REPO), str(TG), str(TG / "baseline_MLP")):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    env, info, steps, code = None, {}, 0, 1
    receipt["seed"] = 123
    receipt["started_utc"] = datetime.now(timezone.utc).isoformat()
    began = time.monotonic()
    try:
        import win_runtime  # Windows native-library load-order shim
        from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
        from prescribed_action import prescribed_action

        env = CMCLikeProsthesisTrajectoryEnv(CMCEnvConfig(**receipt["env_kwargs"]))
        receipt["effective_simulator_config"] = asdict(env.cfg)
        receipt["effective_env_config"] = asdict(env.env_cfg)
        _, reset = env.reset(seed=123)
        receipt.update(reset=reset, actual_episode_start_s=float(env._episode_start), actual_episode_end_s=float(env._episode_end))
        if not all(math.isclose(x, y, abs_tol=1e-8) for x, y in zip(
                (env._episode_start, env._episode_end), receipt["requested_time_range"])):
            raise RuntimeError("Native reset clamped the preselected window")
        write_json(output / "receipt.json", receipt)
        with (output / "trace.jsonl").open("w", encoding="utf-8") as stream:
            while True:
                _, _, terminated, truncated, info = env.step(prescribed_action(env))
                steps += 1
                row = {key: value for key, value in info.items() if key not in {"so_solver_audit_entries", "log"}}
                stream.write(json.dumps(Q._json_value(row), allow_nan=False) + "\n")
                if steps % 50 == 0 or terminated or truncated:
                    stream.flush()
                    print(f"{subject} step={steps} t={env.t:.6f} end={info.get('end_reason')}", flush=True)
                if terminated or truncated:
                    break
        clocks = {"environment": float(env.t), "runner": float(env.runner.current_time), "state": float(env.runner.state.getTime())}
        target = receipt["requested_time_range"][1]
        full = all(math.isclose(value, target, abs_tol=1e-7) for value in clocks.values())
        completed = full and info.get("end_reason") in {"dataset_end", "episode_time_limit"} and not info.get("failure") and not terminated
        receipt.update(status="completed" if completed else "interrupted",
                       completed_requested_duration=completed, steps=steps, final_time_s=clocks["environment"],
                       final_runner_time_s=clocks["runner"], final_state_time_s=clocks["state"],
                       terminated=bool(terminated), truncated=bool(truncated), final_info=info,
                       final_failure=info.get("failure"), terminal_contact=info.get("terminal_contact"),
                       final_clocks_s=clocks)
        code = 0 if completed else 1
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
        receipt["wall_time_s"] = time.monotonic() - began
        receipt["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "receipt.json", receipt)
        checks = assert_fingerprints(protocol["protected_inputs"])
        write_json(AUDIT / subject / "input_integrity.json", {"all_unchanged": True, "checks": checks})
        write_json(AUDIT / subject / "runtime_receipt.json", receipt)
        print(json.dumps({"subject": subject, "status": receipt["status"], "steps": steps, "wall_time_s": receipt["wall_time_s"]}), flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--subject", choices=("AB06", "AB07"))
    args = parser.parse_args()
    if args.prepare == bool(args.subject):
        parser.error("Choose either --prepare or --subject")
    if args.prepare:
        prepare()
        return 0
    return run(args.subject)


if __name__ == "__main__":
    raise SystemExit(main())
