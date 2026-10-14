"""Frozen best47 AB06 reset audit; preparation never imports OpenSim/Ray.

Run --prepare first, then --run (serial) or separate --run --cases CASE jobs.
The sole numerical intervention lives here: replace prosthetic joint velocities
in the runner's initial reference read, before SEA initialization and detector
priming. Matching motor speeds preserve zero initial relative spring velocity.
All subsequent reference reads, biology, GRF and policy inference are original.
This is an instantaneous walking-reset perturbation, not standing or gait onset.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import platform
import sys
import time
import traceback
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
TRAJ = REPO / "Trajectory Generator"
EVALUATOR = TRAJ / "baseline_MLP/rollout_eval.py"
TRAINING = TRAJ / "runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01"
HISTORICAL_AUDIT = TRAJ / "validation/AB06/v26c_best47_control_2026_10_06"
HISTORICAL = TRAJ / "runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s"
SETUP = REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
NOMINAL_OFFSET = 1.956870983805102
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
TOKEN = "CMC_AGENT_OK_2026"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)
                    + "\n", encoding="utf-8")


def relative(path):
    return Path(path).resolve().relative_to(REPO).as_posix()


def resolve(path):
    value = Path(str(path).replace("\\", "/")).expanduser()
    return value.resolve() if value.is_absolute() else (REPO / value).resolve()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def storage(path):
    """Strict finite Storage parser for saved-data diagnostics, in original units."""
    with Path(path).open(encoding="utf-8-sig") as handle:
        metadata = []
        for line in handle:
            metadata.append(line.strip())
            if line.strip().lower() == "endheader":
                break
        else:
            raise ValueError(f"Missing Storage endheader: {path}")
        names = next((line.split() for line in handle if line.strip()), None)
        rows = []
        for line in handle:
            if not line.strip():
                continue
            values = [float(v) for v in line.split()]
            if len(values) != len(names) or not all(math.isfinite(v) for v in values):
                raise ValueError(f"Invalid Storage row: {path}")
            if rows and values[0] <= rows[-1][0]:
                raise ValueError(f"Nonincreasing Storage time: {path}")
            rows.append(values)
    if not rows or not names or names[0].lower() != "time":
        raise ValueError(f"Empty/invalid Storage: {path}")
    return {name: [row[i] for row in rows] for i, name in enumerate(names)}, metadata


def sustained_cycles(times, forces, threshold=20.0, min_contact=0.05, min_cycle=0.3):
    """Same threshold interpolation/contact rules as output._cycles_from_vertical_grf.

    Initial contact is not a measured heel strike. Detection uses the complete
    original force file; cycle choice is offline metadata, never an actor input.
    """
    contacts, start = [], None
    for i in range(1, len(times)):
        was, now = forces[i - 1] > threshold, forces[i] > threshold
        if was == now:
            continue
        denominator = forces[i] - forces[i - 1]
        fraction = 0.0 if abs(denominator) < 1e-12 else (threshold - forces[i - 1]) / denominator
        crossing = float(times[i - 1] + fraction * (times[i] - times[i - 1]))
        if now:
            start = crossing
        elif start is not None:
            if crossing - start >= min_contact:
                contacts.append((start, crossing))
            start = None
    if start is not None and times[-1] - start >= min_contact:
        contacts.append((start, times[-1]))
    return [(left[0], right[0], left[1] - left[0])
            for left, right in zip(contacts, contacts[1:])
            if right[0] - left[0] >= min_cycle]


def plugin_file(stem):
    path = resolve(stem)
    suffix = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
    names = [path.with_suffix(suffix)]
    if sys.platform != "win32":
        names.insert(0, path.parent / ("lib" + path.name + suffix))
    return next((p for p in names if p.is_file()), names[0])


def relocated_historical_receipt():
    receipt = read_json(HISTORICAL_AUDIT / "runtime_receipt.json")
    # Old absolute paths are evidence. Rebase their repository-relative suffix
    # explicitly for the new execution; do not rewrite archived receipts.
    for section in ("requested_env_config", "effective_env_config", "effective_simulator_config"):
        for key, value in receipt[section].items():
            if isinstance(value, str) and "CMC-like-Simulator - Claude/" in value:
                receipt[section][key] = str(REPO / value.split("CMC-like-Simulator - Claude/", 1)[1])
    return receipt


def make_plan(duration=5.0, custom_cases=None):
    history = relocated_historical_receipt()
    cfg = history["effective_simulator_config"]
    grf = REPO / "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot"
    data, _ = storage(grf)
    cycles = sustained_cycles(data["time"], data["ground_force1_vy"])
    nominal = float(cfg["t_start"]) + NOMINAL_OFFSET
    cycle_index = next(i for i, cycle in enumerate(cycles) if abs(cycle[0] - nominal) < 1e-7)
    next_hs = cycles[cycle_index + 1][0]
    cases = [
        dict(id="nominal", offset_s=NOMINAL_OFFSET, kind="trained_nominal", velocity="ik"),
        dict(id="trained_minus020", offset_s=NOMINAL_OFFSET - 0.2, kind="trained_start", velocity="ik"),
        dict(id="trained_plus020", offset_s=NOMINAL_OFFSET + 0.2, kind="trained_start", velocity="ik"),
        dict(id="heldout_phase_plus040", offset_s=NOMINAL_OFFSET + 0.4, kind="heldout_reset_phase", velocity="ik"),
        dict(id="heldout_next_left_hs", offset_s=next_hs - cfg["t_start"], kind="heldout_cycle_same_measured_hs_phase", velocity="ik"),
        dict(id="zero_prosthetic_velocity", offset_s=NOMINAL_OFFSET, kind="instantaneous_walking_reset_velocity_perturbation", velocity="prosthetic_zero"),
    ] if custom_cases is None else read_json(custom_cases)
    validate_cases(cases, cfg["t_start"], cfg["t_end"], duration)
    protected = {p for p in REPO.glob("*.py")}
    protected.update((TRAJ / "baseline_MLP").glob("*.py"))
    protected.update((TRAJ / "common").rglob("*.py"))
    protected.update(TRAJ.glob("*.py"))
    protected.update((TRAINING / "rl_module_best").rglob("*"))
    protected.update([TRAINING / "training_cfg.resolved.yaml", SETUP, grf,
                      HISTORICAL_AUDIT / "runtime_receipt.json", REPO / "plot/plotter.py",
                      HISTORICAL / "rollout_summary.json", Path(__file__)])
    for key in ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml",
                "online_grf_profile_file", "online_grf_detector_profile_file", "binary_phase_detector_profile_file"):
        protected.add(resolve(cfg[key]))
    morphology = history["requested_env_config"]["reward"].get("morphology_profile")
    if morphology:
        protected.add(TRAJ / "baseline_MLP" / morphology)
    plugins = [plugin_file(cfg[key]) for key in ("plugin_name", "online_grf_contact_plugin")]
    protected.update(plugins)
    missing = [relative(p) for p in protected if not p.exists()]
    return {
        "schema_version": 1, "instruction_check_token": TOKEN,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "checkpoint": relative(TRAINING / "rl_module_best"), "checkpoint_logical_iteration": 47,
        "setup_xml": relative(SETUP), "duration_s": duration, "segment_duration_s": 0.01,
        "seed": 123, "cases": cases,
        "phase_selection": {"source": relative(grf), "column": "ground_force1_vy",
                            "threshold_n": 20.0, "min_contact_s": 0.05, "min_cycle_s": 0.3,
                            "nominal_left_hs_s": nominal, "next_left_hs_s": next_hs,
                            "cycle_duration_s": cycles[cycle_index][1] - cycles[cycle_index][0],
                            "note": "Measured GRF HS used only to select reset time; online actor FSM stays unchanged."},
        "protected_sha256": {relative(p): sha256(p) for p in sorted(protected) if p.is_file()},
        "missing_required_files": missing, "native_plugin_files": [relative(p) for p in plugins],
        "platform_prepared": platform.platform(),
        "reference_contract": {"biological_support": "original IK tracking, muscles and residual reserves",
                               "left_grf": "original online_sensor", "right_grf": "original prescribed measured",
                               "policy": "frozen deterministic original evaluator; no PPO/critic/Adam update",
                               "zero_velocity": "IK walking pose, biological speeds unchanged; prosthetic joint and matching SEA motor speeds zero; served reference remains original"},
    }


def validate_cases(cases, start, end, duration):
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be positive and finite")
    identifiers = set()
    for case in cases:
        identifier = case["id"]
        if not identifier or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in identifier):
            raise ValueError(f"Unsafe case id: {identifier}")
        if identifier in identifiers:
            raise ValueError(f"Duplicate case: {identifier}")
        identifiers.add(identifier)
        offset = float(case["offset_s"])
        if not math.isfinite(offset) or offset < 0 or start + offset + duration > end + 1e-9:
            raise ValueError(f"Case exceeds original setup interval, would be silently clamped: {identifier}")
        if case["velocity"] not in ("ik", "prosthetic_zero", "prosthetic_delta"):
            raise ValueError(f"Unsupported velocity intervention: {case['velocity']}")
        for coord, delta in case.get("delta_rad_s", {}).items():
            if coord not in JOINTS or not math.isfinite(float(delta)):
                raise ValueError("Only finite prosthetic qdot perturbations are allowed")


def verify_inputs(plan):
    failures = list(plan["missing_required_files"])
    for key, expected in plan["protected_sha256"].items():
        path = resolve(key)
        if not path.is_file() or sha256(path) != expected:
            failures.append(key)
    if failures:
        raise RuntimeError(f"Frozen inputs missing/changed: {failures}")


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def capture_state(base):
    ctx, state = base.ctx, base.runner.state
    values = ctx.model.getStateVariableValues(state)
    names = ctx.model.getStateVariableNames()
    return {str(names.get(i)): float(values.get(i)) for i in range(names.getSize())}


def install_reset_observer(base, case, receipt):
    """Patch one runner instance only, restoring its reference getter every call."""
    runner = base.runner
    original_reset = runner.reset_to_time

    def observed_reset(t):
        original_get = runner._kin.get
        reads = []

        def reset_get(time_value):
            q, qdot, qddot = original_get(time_value)
            if abs(float(time_value) - float(t)) > 1e-12:
                raise RuntimeError("Unexpected reference read during reset; fail closed")
            applied = dict(qdot)
            if case["velocity"] == "prosthetic_zero":
                for coord in JOINTS:
                    applied[coord] = 0.0
            elif case["velocity"] == "prosthetic_delta":
                for coord, delta in case.get("delta_rad_s", {}).items():
                    applied[coord] += float(delta)
            reads.append({"time_s": float(t), "q_ik": dict(q), "qdot_ik": dict(qdot),
                          "qdot_applied": applied, "qddot_ik": dict(qddot)})
            return (q, qdot, qddot) if case["velocity"] == "ik" else (q, applied, qddot)

        runner._kin.get = reset_get
        try:
            state = original_reset(t)
        finally:
            runner._kin.get = original_get
        if len(reads) != 1:
            raise RuntimeError(f"Expected exactly one runner initial reference read, got {len(reads)}")
        record = reads[0]
        values = base.ctx.model.getStateVariableValues(state)
        record["actual_q"] = {name: float(values.get(index)) for name, index in base.ctx.q_sv_idx.items()}
        record["actual_qdot"] = {name: float(values.get(index)) for name, index in base.ctx.qdot_sv_idx.items()}
        record["full_state_after_reset"] = capture_state(base)
        record["position_max_error_rad_or_m"] = max(abs(record["actual_q"][name] - value) for name, value in record["q_ik"].items())
        record["velocity_max_error_rad_s_or_m_s"] = max(abs(record["actual_qdot"][name] - value) for name, value in record["qdot_applied"].items())
        record["served_reference_after_reset"] = {"q": dict(original_get(t)[0]), "qdot": dict(original_get(t)[1])}
        record["sea_motor_speed"] = {name: float(values.get(index)) for name, index in base.ctx.sea_motor_speed_sv_idx.items()}
        record["sea_speed_matches_joint"] = all(abs(record["sea_motor_speed"][sea] - record["qdot_applied"][coord]) < 1e-12 for sea, coord in ((base.cfg.sea_knee_name, JOINTS[0]), (base.cfg.sea_ankle_name, JOINTS[1])))
        if record["position_max_error_rad_or_m"] > 1e-10 or record["velocity_max_error_rad_s_or_m_s"] > 1e-10 or not record["sea_speed_matches_joint"]:
            raise RuntimeError("Initial reset did not realize the specified IK/qdot/SEA contract")
        receipt.setdefault("reset_reads", []).append(record)
        return state

    runner.reset_to_time = observed_reset
    return original_reset


def prepare_worker_output(output):
    """Allow the watchdog's directory while refusing any previous run data."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    permitted = {"watchdog_state.json"}
    previous = [path.name for path in output.iterdir()
                if path.name not in permitted or not path.is_file()]
    if previous:
        raise FileExistsError(f"Preserve existing run outputs; choose a new output root: {output}: {previous}")


def set_probe_start_offset(base, offset):
    """Replace the frozen CMCEnvConfig for the next passive reset only."""
    offset = float(offset)
    if not math.isfinite(offset) or offset < 0:
        raise ValueError("Passive reset offset must be finite and nonnegative")
    base.env_cfg = replace(base.env_cfg, episode_start_offset_s=offset)


def worker(plan, case, output):
    verify_inputs(plan)
    prepare_worker_output(output)
    receipt = {"schema_version": 1, "instruction_check_token": TOKEN, "case": case,
               "status": "started", "policy_updates": 0, "source_hashes": plan["protected_sha256"],
               "runtime": {"python": sys.version, "executable": sys.executable, "platform": platform.platform()}}
    evaluator = import_file("reset_audit_evaluator", EVALUATOR)
    old_argv = sys.argv
    sys.argv = [str(EVALUATOR), "--checkpoint", str(resolve(plan["checkpoint"])),
                "--setup-xml", str(resolve(plan["setup_xml"])), "--episode-duration", str(plan["duration_s"]),
                "--segment-duration", str(plan["segment_duration_s"]), "--policy-knots", "1",
                "--episode-start-offset-s", str(case["offset_s"]), "--max-steps", str(math.ceil(plan["duration_s"] / plan["segment_duration_s"])),
                "--action-mode", "absolute", "--action-selection", "deterministic", "--seed", str(plan["seed"]),
                "--record-outputs", "--record-policy-trace", "--no-progress", "--output-dir", str(output)]
    try:
        args = evaluator.parse_args()
    finally:
        sys.argv = old_argv
    original_loader = evaluator._load_inference_stack
    original_factory = None
    captured = []

    def observing_loader():
        nonlocal original_factory
        original_loader()
        original_factory = evaluator.env_factory.make_cmc_env

        def factory(config=None):
            env = original_factory(config)
            base = env.unwrapped
            captured.append(base)
            historical = relocated_historical_receipt()
            for key in ("actor_feature_names", "observation_feature_names"):
                if list(getattr(base, key)) != historical[key]:
                    raise RuntimeError(f"Frozen checkpoint feature order changed: {key}")
            receipt["requested_env_config"] = evaluator._jsonable(config)
            receipt["effective_env_config"] = evaluator._jsonable(asdict(base.env_cfg))
            receipt["effective_simulator_config"] = evaluator._jsonable(asdict(base.cfg))
            if (base.cfg.use_muscles_in_so is not True or base.cfg.grf_mode != "online_sensor"
                    or list(base.cfg.online_grf_applied_sides) != ["left"]):
                raise RuntimeError("Biological/GRF support contract changed")
            install_reset_observer(base, case, receipt)
            return env

        evaluator.env_factory.make_cmc_env = factory

    evaluator._load_inference_stack = observing_loader
    started = time.monotonic()
    try:
        summary = evaluator.run(args)
        if len(captured) != 1 or len(receipt.get("reset_reads", [])) != 1:
            raise RuntimeError("Unexpected environment/reset count")
        base = captured[0]
        expected_start = base.cfg.t_start + case["offset_s"]
        if abs(base._episode_start - expected_start) > 1e-10:
            raise RuntimeError("Reset start was clamped or changed")
        receipt.update(status="rollout_finished", summary_path=relative(output / "rollout_summary.json"),
                       actual_start_s=float(base._episode_start), actual_end_s=float(base.t),
                       completed_requested_duration=bool(summary["end_reason"] == "episode_time_limit" and not summary["terminated"] and abs(base.t - base._episode_end) < 1e-8))
        verify_inputs(plan)
        receipt["frozen_inputs_unchanged"] = True
        receipt["runtime"].update(numpy=evaluator.np.__version__, torch=evaluator.torch.__version__)
        write_json(output / "receipt.json", receipt)
        metrics = analyze(output)
        render(output)
        verify_inputs(plan)
        receipt.update(status="completed", metrics_file=relative(output / "metrics.json"))
        print(json.dumps({"case": case["id"], "steps": metrics["summary"]["steps"], "end_reason": metrics["summary"]["end_reason"]}))
        return 0
    except Exception as exc:
        receipt.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        traceback.print_exc()
        return 1
    finally:
        receipt["elapsed_wall_s"] = time.monotonic() - started
        try:
            verify_inputs(plan)
            receipt["frozen_inputs_unchanged"] = True
        except Exception as exc:
            receipt["frozen_inputs_unchanged"] = False
            receipt["integrity_error"] = str(exc)
        write_json(output / "receipt.json", receipt)
        evaluator._load_inference_stack = original_loader
        if original_factory is not None:
            evaluator.env_factory.make_cmc_env = original_factory
        if receipt.get("integrity_error"):
            raise RuntimeError(receipt["integrity_error"])


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def integral(times, values):
    return sum((times[i] - times[i - 1]) * (values[i] + values[i - 1]) * 0.5 for i in range(1, len(times)))


def analyze(output):
    output = Path(output)
    summary = read_json(output / "rollout_summary.json")
    receipt = read_json(output / "receipt.json")
    tables = {}
    file_audit = {}
    for path in sorted((output / "sim_outputs").glob("*.sto")):
        table, metadata = storage(path)
        tables[path.stem.removeprefix("rollout_episode_")] = table
        file_audit[path.name] = {"sha256": sha256(path), "rows": len(table["time"]), "all_finite": True,
                                 "first_time_s": table["time"][0], "last_time_s": table["time"][-1]}
    actual, reference = tables["kinematics"], tables["kinematics_reference"]
    if actual["time"] != reference["time"]:
        raise ValueError("Actual/reference time grids differ")
    times, start = actual["time"], receipt["actual_start_s"]
    windows = {"all_recorded_samples": list(range(len(times))),
               "first_050s": [i for i, t in enumerate(times) if t < start + 0.5],
               "after_050s": [i for i, t in enumerate(times) if t >= start + 0.5]}
    tracking = {}
    for coord in actual:
        ref_key = coord + "_q_ref"
        if ref_key not in reference:
            continue
        scale = 1.0 if coord in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else 180 / math.pi
        tracking[coord] = {"unit": "m" if scale == 1 else "deg",
                           "actual_minus_served_reference_rmse": {name: rms([(actual[coord][i] - reference[ref_key][i]) * scale for i in ids]) for name, ids in windows.items()},
                           "excursion": (max(actual[coord]) - min(actual[coord])) * scale}
    grf = tables["online_grf"]
    sea = tables["sea_torques"]
    metrics = {"schema_version": 1, "case": receipt["case"], "summary": summary,
               "completed_requested_duration": receipt["completed_requested_duration"],
               "recorded_duration_s": times[-1] - times[0], "sample_windows": {name: len(ids) for name, ids in windows.items()},
               "tracking": tracking,
               "left_grf": {"max_penetration_m": max(grf["left_penetration"]),
                             "normal_force_peak_n": max(grf["left_normal_force"]),
                             "normal_force_impulse_ns_recorded_interval": integral(grf["time"], grf["left_normal_force"])},
               "sea": {coord: {"spring_rms_nm": rms(sea[sea_name + "_tau_spring"]),
                                "spring_max_abs_nm": max(abs(v) for v in sea[sea_name + "_tau_spring"]),
                                "positive_area_nms_recorded_interval": integral(sea["time"], [max(v, 0) for v in sea[sea_name + "_tau_spring"]])} for coord, sea_name in zip(JOINTS, ("SEA_Knee", "SEA_Ankle"))},
               "reserve_max_abs_by_coordinate": {name: max(abs(v) for v in values) for name, values in tables["reserve_torques"].items() if name != "time"},
               "outputs": file_audit, "scientific_note": "Actual-versus-served is controller tracking, not imitation or autonomous gait qualification. GRF impulse uses saved sample interval; duration/cycles/contact/support remain separate criteria."}
    write_json(output / "metrics.json", metrics)
    return metrics


def render(output):
    output = Path(output)
    os.environ.setdefault("MPLBACKEND", "Agg")
    plotter = import_file("reset_audit_plotter", REPO / "plot/plotter.py")
    context = plotter.valid_mlp_rollout_context(output)
    if context is None:
        raise ValueError("Seven-plot inputs unavailable")
    folder = REPO / "plot/reset_ik_audit_2026_10_12" / output.name
    folder.mkdir(parents=True, exist_ok=False)
    old_discovery, old_next, old_argv = plotter.latest_mlp_rollout_context, plotter.next_output_dir, sys.argv
    plotter.latest_mlp_rollout_context = lambda: context
    plotter.next_output_dir = lambda root: folder
    summary = read_json(output / "rollout_summary.json")
    sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir),
                "--prefix", "rollout_episode", "--setup", summary["setup_xml_path"], "--gait-side", "left", "--out-root", str(folder.parent)]
    try:
        result = plotter.main()
    finally:
        plotter.latest_mlp_rollout_context, plotter.next_output_dir, sys.argv = old_discovery, old_next, old_argv
    pngs = sorted(folder.glob("[0-9][0-9]_*.png"))
    if result != 0 or len(pngs) != 7:
        raise RuntimeError(f"Canonical rendering incomplete: {len(pngs)} figures")
    receipt = {"rollout": relative(output), "figures": {relative(p): sha256(p) for p in pngs},
               "missing_channels_report": (folder / "missing_channels.txt").read_text() if (folder / "missing_channels.txt").exists() else None}
    write_json(output / "plot_receipt.json", receipt)
    write_json(folder / "plot_receipt.json", receipt)


def reset_probe(plan, destination):
    """Three seed resets at the fixed start, then each case; no integration."""
    verify_inputs(plan)
    destination.mkdir(parents=True, exist_ok=False)
    evaluator = import_file("reset_probe_evaluator", EVALUATOR)
    evaluator._load_inference_stack()
    history = relocated_historical_receipt()
    config = dict(history["requested_env_config"])
    config.update(record_outputs=False, save_outputs_on_close=False, output_dir=str(destination / "unused_outputs"),
                  episode_start_offset_s=NOMINAL_OFFSET, episode_duration=plan["duration_s"], random_init=False)
    env = evaluator.env_factory.make_cmc_env(config)
    base = env.unwrapped
    result = {"schema_version": 1, "policy_loaded": False, "integration_steps": 0, "records": []}
    original_reset = base.runner.reset_to_time
    try:
        nominal = next(c for c in plan["cases"] if c["id"] == "nominal")
        requests = [(nominal, seed) for seed in (123, 124, 125)] + [(case, 123) for case in plan["cases"] if case["id"] != "nominal"]
        for case, seed in requests:
            set_probe_start_offset(base, case["offset_s"])
            base.runner.reset_to_time = original_reset
            capture = {}
            install_reset_observer(base, case, capture)
            obs, info = env.reset(seed=seed)
            result["records"].append({"case": case, "seed": seed, "observation": evaluator._jsonable(obs),
                                      "reset_info": evaluator._jsonable(info), "state": capture["reset_reads"][0]})
        first = result["records"][:3]
        result["fixed_start_state_identical_across_seeds"] = all(record["state"]["full_state_after_reset"] == first[0]["state"]["full_state_after_reset"] for record in first[1:])
        result["fixed_start_actor_observation_identical_across_seeds"] = all(record["observation"][:base.n_actor] == first[0]["observation"][:base.n_actor] for record in first[1:])
        verify_inputs(plan)
        result["frozen_inputs_unchanged"] = True
        write_json(destination / "reset_probe.json", result)
    finally:
        base.runner.reset_to_time = original_reset
        env.close()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--run", action="store_true")
    action.add_argument("--worker", metavar="CASE")
    action.add_argument("--reset-probe", action="store_true")
    action.add_argument("--analyze", type=Path, metavar="ROLLOUT")
    parser.add_argument("--manifest", type=Path, default=AUDIT / "frozen_manifest.json")
    parser.add_argument("--cases", nargs="+", help="Subset; separate invocations can run isolated cases concurrently")
    parser.add_argument("--cases-json", type=Path, help="Explicit additional/replacement case definitions at preparation")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--output-root", type=Path, default=AUDIT / "rollouts")
    parser.add_argument("--python", default=sys.executable, help="Native interpreter for child execution; no hardcoded platform path")
    args = parser.parse_args()
    if args.analyze:
        analyze(resolve(args.analyze))
        return 0
    if not any((args.run, args.worker, args.reset_probe)):
        plan = make_plan(args.duration_s, args.cases_json)
        if args.prepare:
            write_json(args.manifest, plan)
        print(json.dumps({"manifest": str(args.manifest) if args.prepare else None,
                          "cases": plan["cases"], "missing_required_files": plan["missing_required_files"],
                          "native_execution_started": False}, indent=2))
        return 0 if not plan["missing_required_files"] else 2
    plan = read_json(args.manifest)
    verify_inputs(plan)
    root = args.output_root.resolve()
    if args.worker:
        case = next(c for c in plan["cases"] if c["id"] == args.worker)
        return worker(plan, case, root / case["id"])
    if args.reset_probe:
        return reset_probe(plan, root / "passive_reset_probe")
    cases = [c for c in plan["cases"] if args.cases is None or c["id"] in args.cases]
    if args.cases and set(args.cases) != {c["id"] for c in cases}:
        raise ValueError("Unknown requested case")
    root.mkdir(parents=True, exist_ok=True)
    results = []
    for case in cases:
        case_output = root / case["id"]
        if case_output.exists():
            raise FileExistsError(f"Preserve existing run; choose new output root: {case_output}")
        evaluator = import_file("reset_audit_supervisor", EVALUATOR)
        argv = [args.python, "-B", "-u", str(Path(__file__).resolve()), "--worker", case["id"],
                "--manifest", str(args.manifest.resolve()), "--output-root", str(root)]
        log = root / (case["id"] + ".log")
        supervisor = root / (case["id"] + "_supervisor.py")
        # Redirect the supervised child with a tiny platform-independent stdlib
        # launcher; the watchdog still observes evaluator's original heartbeats.
        supervisor.write_text("import subprocess, sys\nfrom pathlib import Path\n"
                              "with Path(sys.argv[1]).open('wb') as log:\n"
                              "    raise SystemExit(subprocess.call(sys.argv[2:], stdout=log, stderr=subprocess.STDOUT))\n", encoding="utf-8")
        result = evaluator.process_watchdog.supervise_process(
            [args.python, "-B", str(supervisor), str(log), *argv],
            heartbeat_file=case_output / "watchdog_state.json", summary_file=root / (case["id"] + "_watchdog.json"),
            startup_timeout_s=600, stall_timeout_s=180, run_timeout_s=1800,
            label="Frozen reset audit " + case["id"])
        verify_inputs(plan)
        results.append({"case": case["id"], "argv": argv, "supervision": result})
        write_json(root / (case["id"] + "_execution.json"), results[-1])
        if result["returncode"] != 0 or result["timeout_reason"]:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
