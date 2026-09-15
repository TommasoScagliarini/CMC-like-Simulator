"""Execute the preregistered, deferred 39-feature feasibility campaign.

Only runs after the V1 50- and 100-iteration evaluations are complete. The
runtime package is materialized at that point. This is an exploratory pilot;
neither successful execution nor a training return establishes gait parity.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import pickle
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
UNIFIED = HERE.parent
TRAJECTORY = UNIFIED.parent
REPO = TRAJECTORY.parent
sys.path.insert(0, str(TRAJECTORY))
from unified_MLP.validation import compare_pretraining_checkpoints as metrics
from unified_MLP.validation import evaluate_training_milestone as milestone_metrics
from unified_MLP.validation.run_authorized_plan import read_json, write_json, utcnow

STARTS = milestone_metrics.START_OFFSETS
HELD_OUT = {"heldout_early_mid": 1.856870983805102, "heldout_late_mid": 2.056870983805102}
ALL_STARTS = {**STARTS, **HELD_OUT}
NOISE_SEEDS = (20260914, 20260915, 20260916)
PROBABILITIES = {"A": [1.0] * 5, "B": [0.0] * 5, "C": [1.0, .75, .5, .25, 0.0]}


def preconditions(plan):
    for target in (50, 100):
        report = UNIFIED / "runs" / "validation" / f"pretraining_v1_pen1_milestone_{target}_2026-09-14" / "milestone_evaluation.json"
        result = read_json(report)
        if result.get("status") != "complete" or result.get("selection", {}).get("milestone") != target:
            raise ValueError(f"V1 evaluation {target} is not complete")
    if not (plan / "training_summary_100.json").is_file():
        raise ValueError("The authorized controller has not recorded training through 100")


def publish_config(path, config):
    import yaml
    if path.exists():
        if yaml.safe_load(path.read_text()) != config:
            raise ValueError(f"Existing configuration differs: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def validate_campaign_training(run, target, ex_novo):
    summary = read_json(run / "summary.json")
    history = [json.loads(line) for line in (run / "train_iterations.jsonl").read_text().splitlines() if line]
    if (not summary.get("ok") or summary.get("stop_reason") != "completed"
            or summary.get("iterations_completed") != target or summary.get("skipped_iterations")):
        raise RuntimeError(f"Training incomplete: {run}")
    if [r["iteration"] for r in history] != list(range(1, target + 1)):
        raise RuntimeError("Missing/duplicate iterations in feasibility training")
    phases = [r["unified_phase"] for r in history]
    expected = ["critic_warmup"] + ["ex_novo"] * (target - 1) if ex_novo else ["imitation"] * target
    if phases != expected:
        raise RuntimeError("Wrong reward phase or repeated warm-up")
    if any(not r.get("exact_start_balance", {}).get("pass") for r in history):
        raise RuntimeError("Feasibility training did not preserve start balance")
    for row in history:
        availability = row["unified_phase_audit"].get("target_availability_batch", {})
        if not availability.get("matches_expected_train_batch_size"):
            raise RuntimeError("Target visibility telemetry does not cover the collected batch")
        probability = row["unified_phase_audit"]["target_availability"]["probability"]
        if (ex_novo or probability == 0.0) and availability.get("visible_steps") != 0:
            raise RuntimeError("A fully masked training batch contained visible teacher targets")
    if ex_novo and (not history[0]["unified_phase_audit"].get("actor_exact")
                    or not history[0]["unified_phase_audit"].get("critic_changed")):
        raise RuntimeError("Critic-only warm-up did not preserve actor/update critic")
    milestone = run / f"milestone_iteration_{target:06d}"
    state = read_json(milestone / "checkpoint_last" / "unified_training_state.json")
    meta = read_json(milestone / "checkpoint_last_meta.json")
    if state.get("last_completed_iteration") != target or meta.get("logical_iteration") != target:
        raise ValueError("Feasibility checkpoint does not match its history")
    if not (milestone / "rl_module_last" / "module_state.pkl").is_file():
        raise ValueError("Feasibility module export is missing")
    return summary


def loaded_rollout(directory, selection):
    """Validate a recorded rollout without relabelling stochastic observations."""
    if selection == "deterministic":
        return metrics.load_rollout(directory)
    summary = read_json(directory / "rollout_summary.json")
    rows = read_json(directory / "rollout_policy_trace.json")
    if summary.get("ok") is not True or summary.get("action_selection") != selection:
        raise ValueError(f"Unsuccessful/wrong action selection: {directory}")
    if not rows or len(rows) != summary.get("steps"):
        raise ValueError("Rollout trace is missing samples")
    for i, row in enumerate(rows):
        pre = metrics.finite(row["policy_segment_times"][0], "pre time")
        end = metrics.finite(row["time"], "post time")
        segment_end = metrics.finite(row["policy_segment_times"][-1], "segment end")
        if row["step"] != i + 1 or end <= pre or abs(end - segment_end) > metrics.TIME_TOL:
            raise ValueError("Invalid stochastic trace interval")
        if i and abs(pre - rows[i - 1]["time"]) > metrics.TIME_TOL:
            raise ValueError("Discontinuous stochastic trace")
        row.update(_pre_time=pre, _dt=end - pre)
    result = {"directory": str(directory), "summary": summary, "rows": rows,
              "source_hashes": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                                for name in ("rollout_summary.json", "rollout_policy_trace.json")}}
    metrics.analyze_window(result, rows, 5.0, True)
    return result


def pilot_walk(case):
    m = case["full_episode"]
    return (m["duration_s"] >= 5.0 - metrics.TIME_TOL
            and m["phase"]["valid_cycle_count"] >= 1
            and m["phase"]["invalid_event_count"] == 0
            and case["first_online_period_available_elapsed_s"] is not None
            and m["safety"]["penetration_max_mm"] < 28.0
            and m["termination"]["end_reason"] == "episode_time_limit"
            and not m["termination"]["terminated"])


def paired_comparison(first, second):
    metrics.validate_settings(first, second)
    a, b = metrics.common_rows(first, second)
    return {"first": milestone_metrics.window_metrics(first, a),
            "second": milestone_metrics.window_metrics(second, b)}


def favorable_pilot(masked):
    """A conservative trigger for more seeds, never an assertion of parity."""
    reasons = []
    for start in STARTS:
        b, c = masked[("B", start)], masked[("C", start)]
        case_b = milestone_metrics.describe_case(b, 50, start, ["B_masked"])
        case_c = milestone_metrics.describe_case(c, 50, start, ["C_masked"])
        if not pilot_walk(case_c):
            reasons.append(f"{start}:C_does_not_meet_autonomous_walk_screen")
        if case_c["full_episode"]["phase"]["valid_cycle_count"] < case_b["full_episode"]["phase"]["valid_cycle_count"]:
            reasons.append(f"{start}:C_has_fewer_valid_cycles_than_B")
        pair = paired_comparison(b, c)
        for joint in metrics.JOINTS:
            for output in ("actual", "served"):
                before = pair["first"]["tracking"][joint][output + "_vs_prescribed"]["position"]["rmse_rad"]
                after = pair["second"]["tracking"][joint][output + "_vs_prescribed"]["position"]["rmse_rad"]
                if not after < before:
                    reasons.append(f"{start}:{joint}:{output}:C_not_better_than_B")
        if c["rows"][-1]["time"] < b["rows"][-1]["time"] - metrics.TIME_TOL:
            reasons.append(f"{start}:C_shorter_than_B")
    return {"trigger_replications": not reasons, "reasons": reasons,
            "scope": "strict descriptive dominance on common-prefix position tracking and autonomous-walk screen; exploratory only"}


def paired_initialization(runs):
    from unified_MLP.actor_transition import actor_state_digest
    from unified_MLP.training_runtime import critic_digest
    digests = {}
    for arm, run in runs.items():
        source = run / "rl_module_initial" / "module_state.pkl"
        with source.open("rb") as stream:
            state = pickle.load(stream)
        digests[arm] = {"actor": actor_state_digest(state), "critic": critic_digest(state)}
    if len(digests) != 3 or len({(d["actor"], d["critic"]) for d in digests.values()}) != 1:
        raise ValueError("A/B/C initialization is not paired")
    return digests


def constant_sigma_export(run, iteration, variant):
    from unified_MLP.actor_transition import configure_constant_std, actor_mean_digest
    source = run / f"milestone_iteration_{iteration:06d}" / "rl_module_last"
    digest = hashlib.sha256((source / "module_state.pkl").read_bytes()).hexdigest()
    destination = variant / "runs" / "diagnostics" / "sigma_evaluation" / f"{run.name}_i{iteration}_{digest[:12]}"
    audit_path = destination / "sigma_audit.json"
    if destination.exists():
        audit = read_json(audit_path)
        if audit["source_sha256"] != digest:
            raise ValueError("Sigma export source changed")
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, destination)
        shutil.copy2(run / "training_cfg.resolved.yaml", destination / "training_cfg.resolved.yaml")
        with (source / "module_state.pkl").open("rb") as stream:
            original = pickle.load(stream)
        adjusted, audit = configure_constant_std(original, sigma=.010)
        with (destination / "module_state.pkl").open("wb") as stream:
            pickle.dump(adjusted, stream)
        with (destination / "module_state.pkl").open("rb") as stream:
            reloaded = pickle.load(stream)
        if actor_mean_digest(original) != actor_mean_digest(reloaded):
            raise ValueError("Sigma export changed the mean network on reload")
        audit.update(source_sha256=digest, source=str(source), mean_exact_after_reload=True,
                     configured_state_sha256=hashlib.sha256((destination / "module_state.pkl").read_bytes()).hexdigest())
        write_json(audit_path, audit)
    if hashlib.sha256((destination / "module_state.pkl").read_bytes()).hexdigest() != audit["configured_state_sha256"]:
        raise ValueError("Sigma evaluation weights changed")
    return destination


def verify_common_noise(rollouts):
    import numpy as np
    result = []
    for start in ALL_STARTS:
        for seed in NOISE_SEEDS:
            groups = [rollouts[(arm, start, seed)] for arm in ("A", "B", "C")]
            count = min(len(r["rows"]) for r in groups)
            noises = []
            for r in groups:
                noise = np.asarray([row["exploration_action_noise"] for row in r["rows"][:count]])
                std = np.asarray([row["policy_action_std"] for row in r["rows"][:count]])
                if not np.isfinite(std).all() or (std <= 0).any() or not np.isfinite(noise).all():
                    raise ValueError("Invalid Gaussian noise telemetry")
                if not np.allclose(std, .010, rtol=1e-6, atol=1e-8):
                    raise ValueError("Stochastic evaluation did not use fixed sigma .010")
                noises.append(noise / std)
            error = max(float(np.max(np.abs(noises[0] - n))) for n in noises[1:])
            if error > 1e-6:
                raise ValueError("Standard-normal noise differs across A/B/C")
            result.append({"start": start, "noise_seed": seed, "common_steps": count, "max_abs_standard_noise_difference": error})
    return result


class Campaign:
    def __init__(self, plan):
        self.plan = plan
        self.variant = TRAJECTORY / "unified_MLP_39"
        self.work = plan / "feasibility"
        self.work.mkdir(parents=True, exist_ok=True)
        self.state = {"started_at": utcnow(), "protocol_completed": False, "events": []}
        self.all_rollouts = []

    def execute(self, label, command, timeout=40 * 3600):
        self.state.update(current_action=label, updated_at=utcnow(), command=command)
        write_json(self.work / "state.json", self.state)
        log = self.work / f"{label}.log"
        with log.open("a", encoding="utf-8") as output:
            output.write(f"\n[{utcnow()}] {json.dumps(command)}\n")
            output.flush()
            process = subprocess.run(command, cwd=REPO, stdout=output, stderr=subprocess.STDOUT, timeout=timeout)
        if process.returncode:
            raise RuntimeError(f"{label} failed ({process.returncode}): {log}")
        self.state["events"].append({"action": label, "completed_at": utcnow(), "log": str(log)})
        write_json(self.work / "state.json", self.state)

    def smoke(self):
        """Real 39D collector/learner/transfer smoke, separate from A/B/C budgets."""
        import yaml
        stages = []
        for mode, targets in (("imitation", (1,)), ("ex_novo", (1, 2, 3))):
            source = (self.plan / "pilot5_snapshot/training_cfg.resolved.yaml" if mode == "imitation"
                      else UNIFIED / "configs/ex_novo.yaml")
            config = yaml.safe_load(source.read_text())
            config["ppo"].update(train_batch_size=192, minibatch_size=64, num_epochs=1)
            config["parallelism"].update(num_env_runners=3, ray_num_cpus=4, exact_start_sampling=True)
            config["supervision"].update(checkpoint_every=1, retain_iteration_checkpoints=True)
            config["target_availability"] = {"target_availability_probability": 1.0 if mode == "imitation" else 0.0,
                                             "target_curriculum_probabilities": [], "target_curriculum_block_iterations": 10,
                                             "target_mask_seed": 123}
            path = self.work / "configs" / f"smoke_{mode}.yaml"
            publish_config(path, config)
            run = self.variant / "runs/training" / f"feasibility_smoke_{mode}"
            for target in targets:
                cmd = [sys.executable, "-u", str(self.variant / "train_ppo_mlp.py"), "--config", str(path),
                       "--iterations", str(target), "--output-dir", str(run)]
                if target > 1:
                    cmd += ["--resume-from", str(run / f"milestone_iteration_{target-1:06d}/checkpoint_last")]
                elif mode == "ex_novo":
                    cmd += ["--init-actor-from", str(self.variant / "runs/training/feasibility_smoke_imitation/milestone_iteration_000001/rl_module_last")]
                self.execute(f"smoke_{mode}_{target}", cmd, timeout=7200)
                summary = validate_campaign_training(run, target, mode == "ex_novo")
                if summary["history"][-1]["num_env_steps_sampled_lifetime"] != target * 192:
                    raise ValueError("Smoke sampling count differs from its budget")
                stages.append({"mode": mode, "target": target, "summary": str(run / "summary.json")})
            for start in STARTS:
                self.rollout(run, targets[-1], start, 0.0, tag="_smoke")
        write_json(self.work / "smoke_verification.json", {"pass": True, "stages": stages,
                   "imitation_transitions": 192, "ex_novo_transitions": 576,
                   "total_training_transitions": 768, "rollouts": 6})
    def configuration(self, arm, seed, *, ex_novo=False):
        import yaml
        source = UNIFIED / "configs" / "ex_novo.yaml" if ex_novo else self.plan / "pilot5_snapshot" / "training_cfg.resolved.yaml"
        config = yaml.safe_load(source.read_text())
        config["model"]["seed"] = seed
        config["simulation"]["iterations"] = 11 if ex_novo else 50
        config["supervision"]["retain_iteration_checkpoints"] = True
        config["supervision"]["checkpoint_every"] = 1
        config["target_availability"] = {
            "target_availability_probability": 0.0 if ex_novo else PROBABILITIES[arm][0],
            "target_curriculum_probabilities": [] if ex_novo else PROBABILITIES[arm],
            "target_curriculum_block_iterations": 10,
            "target_mask_seed": seed,
        }
        config.setdefault("provenance", {})["feasibility_arm"] = arm
        config["provenance"]["feasibility_training_seed"] = seed
        config["provenance"]["feasibility_stage"] = "ex_novo_transfer" if ex_novo else "imitation"
        config["provenance"]["actor_schema"] = "39; 35 causal plus four masked current left prescribed copies"
        path = self.work / "configs" / f"{arm}_seed{seed}_{'ex_novo' if ex_novo else 'imitation'}.yaml"
        publish_config(path, config)
        return path

    def training(self, arm, seed, target, *, ex_novo=False):
        name = f"feasibility_{arm}_seed{seed}_{'ex_novo' if ex_novo else 'imitation'}"
        run = self.variant / "runs" / "training" / name
        config = self.configuration(arm, seed, ex_novo=ex_novo)
        command = [sys.executable, "-u", str(self.variant / "train_ppo_mlp.py"), "--config", str(config),
                   "--iterations", str(target), "--output-dir", name]
        if (run / "summary.json").exists():
            previous = read_json(run / "summary.json")
            if previous.get("ok") and previous.get("iterations_completed") == target:
                validate_campaign_training(run, target, ex_novo)
                return run
            if not previous.get("ok"):
                raise RuntimeError(f"Prior unsuccessful training must be inspected before resuming: {run}")
            completed = previous["iterations_completed"]
            if completed >= target:
                raise ValueError("A later run cannot stand in for an earlier evaluation")
            command += ["--resume-from", str(run / f"milestone_iteration_{completed:06d}" / "checkpoint_last")]
        elif ex_novo:
            source = self.variant / "runs" / "training" / f"feasibility_{arm}_seed{seed}_imitation" / "milestone_iteration_000050" / "rl_module_last"
            command += ["--init-actor-from", str(source)]
        self.execute(f"train_{name}_to{target}", command)
        validate_campaign_training(run, target, ex_novo)
        return run

    def rollout(self, run, iteration, start, probability, *, selection="deterministic", noise_seed=123, tag=""):
        label = f"{run.name}_i{iteration}_{start}_p{probability:g}_{selection}_{noise_seed}{tag}"
        output = self.variant / "runs" / "rollout" / label
        checkpoint = run / f"milestone_iteration_{iteration:06d}" / "rl_module_last"
        if selection != "deterministic":
            checkpoint = constant_sigma_export(run, iteration, self.variant)
        identity = {"checkpoint": str(checkpoint), "weight_sha256": hashlib.sha256((checkpoint / "module_state.pkl").read_bytes()).hexdigest(),
                    "start": start, "probability": probability, "selection": selection, "action_seed": noise_seed,
                    "max_steps": 500, "episode_duration": 5.0}
        if (output / "feasibility_identity.json").exists():
            if read_json(output / "feasibility_identity.json") != identity:
                raise ValueError(f"Rollout identity changed: {output}")
        else:
            if output.exists():
                raise ValueError(f"Unidentified preexisting rollout: {output}")
            output.mkdir(parents=True)
            write_json(output / "feasibility_identity.json", identity)
        if not (output / "rollout_summary.json").exists():
            command = [sys.executable, str(self.variant / "rollout_eval.py"), "--checkpoint", str(checkpoint),
                       "--episode-start-offset-s", str(ALL_STARTS[start]), "--episode-duration", "5", "--max-steps", "500",
                       "--action-selection", selection, "--seed", str(noise_seed),
                       "--target-availability-probability", str(probability), "--record-policy-trace", "--record-outputs",
                       "--output-dir", str(output), "--no-progress"]
            self.execute(f"rollout_{label}", command, timeout=3600)
        result = loaded_rollout(output, selection)
        summary = result["summary"]
        if summary.get("action_seed") != noise_seed or summary.get("action_selection") != selection:
            raise ValueError("Rollout seed/action mode differ from its identity")
        if abs(float(summary["episode_start_offset_s"]) - ALL_STARTS[start]) > metrics.TIME_TOL:
            raise ValueError("Rollout reset differs from its identity")
        if not summary.get("record_policy_trace") or not summary.get("record_outputs"):
            raise ValueError("Feasibility requires policy trace and simulator outputs")
        if Path(summary["checkpoint"]).resolve() != checkpoint.resolve():
            raise ValueError("Rollout loaded another checkpoint")
        target_features = summary.get("actor_feature_names", [])[35:39]
        for row in result["rows"]:
            mask = row["target_availability"]
            if abs(float(mask["episode_probability"]) - probability) > 1e-12:
                raise ValueError("Effective rollout availability differs from its requested probability")
            if not mask["visible"] and any(row["actor_observation_before"][name] != 0 for name in target_features):
                raise ValueError("Masked rollout exposed prescribed actor channels")
            if probability == 0.0 and mask["visible"]:
                raise ValueError("Target mask opened during a masked evaluation")
        if result["summary"].get("actor_feature_names", [])[:35] != read_json(self.plan / "pilot5_snapshot" / "unified_contract.json")["actor_feature_names"]:
            raise ValueError("The 35 original actor channels changed")
        if len(result["summary"].get("actor_feature_names", [])) != 39:
            raise ValueError("Feasibility rollout has the wrong observation dimension")
        self.all_rollouts.append(str(output))
        return result

    def run_seed(self, seed):
        checkpoints = {}
        initialization = None
        boundary_cases = []
        for block, target in enumerate((10, 20, 30, 40, 50)):
            for arm in ("A", "B", "C"):
                run = self.training(arm, seed, target)
                checkpoints[arm] = run
                probabilities = {0.0, PROBABILITIES[arm][block]}
                if arm == "C" and block < 4:
                    probabilities.add(PROBABILITIES[arm][block + 1])
                if target == 50:
                    probabilities.add(0.0)
                for probability in sorted(probabilities):
                    for start in STARTS:
                        result = self.rollout(run, target, start, probability)
                        case = milestone_metrics.describe_case(result, target, start, [arm])
                        boundary_cases.append({"arm": arm, "probability": probability, **case})
                write_json(self.work / f"boundary_evaluations_seed{seed}.json", boundary_cases)
                if block > 0:
                    # Checkpoints 11/21/31/41 already exist; evaluating them adds no updates.
                    after_change = target - 9
                    for start in STARTS:
                        result = self.rollout(run, after_change, start, 0.0)
                        boundary_cases.append({"arm": arm, "probability": 0.0,
                                               **milestone_metrics.describe_case(result, after_change, start, [arm, "after_first_block_update"])})
                    write_json(self.work / f"boundary_evaluations_seed{seed}.json", boundary_cases)
            if block == 0:
                initialization = paired_initialization(checkpoints)
                write_json(self.work / f"initialization_seed{seed}.json", initialization)
        masked, stochastic, stochastic_rollouts = {}, [], {}
        for arm, run in checkpoints.items():
            for start in ALL_STARTS:
                masked[(arm, start)] = self.rollout(run, 50, start, 0.0)
                for noise_seed in NOISE_SEEDS:
                    result = self.rollout(run, 50, start, 0.0, selection="stochastic_held", noise_seed=noise_seed)
                    stochastic_rollouts[(arm, start, noise_seed)] = result
                    stochastic.append({"arm": arm, "action_seed": noise_seed,
                                       **milestone_metrics.describe_case(result, 50, start, [arm])})
        noise_audit = verify_common_noise(stochastic_rollouts)
        trigger = favorable_pilot(masked)
        transfers, transfer_rollouts = [], {}
        for arm in ("B", "C"):
            self.training(arm, seed, 1, ex_novo=True)
            run = self.training(arm, seed, 11, ex_novo=True)
            for start in ALL_STARTS:
                result = self.rollout(run, 11, start, 0.0)
                transfer_rollouts[(arm, start)] = result
                transfers.append({"arm": arm, **milestone_metrics.describe_case(result, 11, start, ["ex_novo_transfer"])})
                for noise_seed in NOISE_SEEDS:
                    stochastic_transfer = self.rollout(run, 11, start, 0.0, selection="stochastic_held", noise_seed=noise_seed)
                    transfers.append({"arm": arm, "action_seed": noise_seed,
                                      **milestone_metrics.describe_case(stochastic_transfer, 11, start, ["ex_novo_transfer_stochastic"])})
        transfer_comparison = [{"start": start, **paired_comparison(transfer_rollouts[("B", start)], transfer_rollouts[("C", start)])} for start in ALL_STARTS]
        transfer_trigger = favorable_pilot(transfer_rollouts)
        trigger = {"trigger_replications": trigger["trigger_replications"] and transfer_trigger["trigger_replications"],
                   "reasons": trigger["reasons"] + ["transfer:" + r for r in transfer_trigger["reasons"]],
                   "scope": "strict descriptive improvement in both masked imitation and ex-novo transfer; exploratory only"}
        comparison = [{"start": start, **paired_comparison(masked[("B", start)], masked[("C", start)])} for start in STARTS]
        report = {"seed": seed, "imitation_iterations_per_arm": 50, "transfer_iterations": "1 critic-only + 10 PPO",
                  "paired_initialization": initialization, "common_random_numbers_audit": noise_audit,
                  "boundary_evaluations": boundary_cases, "stochastic_evaluations": stochastic,
                  "B_C_masked_comparison": comparison, "transfers": transfers,
                  "B_C_ex_novo_comparison": transfer_comparison, "replication_trigger": trigger,
                  "scope": "exploratory feasibility; no biomechanical or statistical parity claim"}
        write_json(self.work / f"results_seed{seed}.json", report)
        self.write_seed_report(seed, report, masked)
        return report

    def write_seed_report(self, seed, report, masked):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        lines = [f"# Fattibilità a 39 ingressi — seed {seed}", "",
                 "Risultati esplorativi, checkpoint50 fissato prima degli esperimenti. Tutti i valori in tabella usano maschera chiusa.", "",
                 "| Braccio | Start | Durata s | Cicli | Pen max mm | RMSE knee ° | RMSE ankle ° | Fine |",
                 "|---|---|---:|---:|---:|---:|---:|---|"]
        for (arm, start), rollout in sorted(masked.items()):
            m = metrics.analyze_window(rollout, rollout["rows"], 5.0, True)
            knee = m["tracking"]["knee"]["actual_vs_prescribed"]["position"]["rmse_rad"] * metrics.RAD_TO_DEG
            ankle = m["tracking"]["ankle"]["actual_vs_prescribed"]["position"]["rmse_rad"] * metrics.RAD_TO_DEG
            lines.append(f"| {arm} | {start} | {m['duration_s']:.2f} | {m['phase']['valid_cycle_count']} | {m['safety']['penetration_max_mm']:.2f} | {knee:.2f} | {ankle:.2f} | {m['termination']['end_reason']} |")
        lines += ["", "Confronti sulle finestre comuni, velocità, safety, disponibilità, prove stocastiche e trasferimento sono riportati nel JSON associato.",
                  "Nessun ciclo valido rende il regime non valutabile. Un esito negativo nel budget non dimostra impossibilità informativa.", "",
                  f"Segnale per repliche: {report['replication_trigger']['trigger_replications']}; motivi: {', '.join(report['replication_trigger']['reasons']) or 'criteri esplorativi soddisfatti'}."]
        (self.work / f"results_seed{seed}.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        for start in ALL_STARTS:
            fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
            for arm_index, arm in enumerate(("A", "B", "C")):
                rows = masked[(arm, start)]["rows"]
                t0 = rows[0]["_pre_time"]
                times = [r["time"] - t0 for r in rows]
                for j, joint in enumerate(metrics.JOINTS.values()):
                    for v, (suffix, target) in enumerate((("", "imitation_target_q"), ("_vel", "imitation_target_qdot"))):
                        ax = axes[j, v]
                        ax.plot(times, [r[target][joint] * metrics.RAD_TO_DEG for r in rows], color="black", alpha=.4, label="prescribed" if arm == "A" else None)
                        for served, style in ((False, "-"), (True, "--")):
                            key = joint + ("_served_ref" if served else "") + suffix
                            ax.plot(times, [r["prosthetic_state"][key] * metrics.RAD_TO_DEG for r in rows], color=f"C{arm_index}", ls=style, label=f"{arm} {'served' if served else 'actual'}")
                        ax.set(xlim=(0, 5), ylabel="deg/s" if v else "deg", title=f"{joint}: {'velocity' if v else 'position'}")
                        ax.grid(alpha=.2)
            axes[0, 0].legend(fontsize=7, ncol=2)
            for ax in axes[-1]:
                ax.set_xlabel("Elapsed from reset [s]")
            fig.suptitle(f"Seed {seed} — {start} — CP50 — target channels masked")
            fig.tight_layout()
            for suffix in ("png", "pdf"):
                fig.savefig(self.work / f"masked_tracking_seed{seed}_{start}.{suffix}", dpi=160)
            plt.close(fig)

    def run(self):
        preconditions(self.plan)
        protocol = REPO / "reports/plans/2026-09-14_protocollo_fattibilita_unificata39.md"
        self.state["preregistered_protocol_sha256"] = hashlib.sha256(protocol.read_bytes()).hexdigest()
        v1_rollouts = set()
        for target in (50, 100):
            evaluation = UNIFIED / "runs" / "validation" / f"pretraining_v1_pen1_milestone_{target}_2026-09-14" / "milestone_evaluation.json"
            v1_rollouts.update(case["source_directory"] for case in read_json(evaluation)["cases"])
        self.all_rollouts.extend(sorted(v1_rollouts))
        self.execute("observability_v1", [sys.executable, str(HERE / "analyze_observability.py"),
                     "--rollouts", *sorted(v1_rollouts), "--output-dir", str(self.work / "observability_v1")], timeout=1800)
        self.execute("software_isolation_tests", [sys.executable, "-m", "pytest", str(UNIFIED / "tests/test_feasibility39_bundle.py"), "-q"], timeout=600)
        self.execute("materialize_variant", [sys.executable, str(HERE / "feasibility39_bundle/build_variant.py"), "--destination", str(self.variant)], timeout=120)
        self.smoke()
        results = [self.run_seed(123)]
        if results[0]["replication_trigger"]["trigger_replications"]:
            results.extend(self.run_seed(seed) for seed in (456, 789))
        self.execute("observability", [sys.executable, str(HERE / "analyze_observability.py"),
                     "--rollouts", *sorted(set(self.all_rollouts)), "--output-dir", str(self.work / "observability")], timeout=1800)
        result = {"protocol_completed": True, "completed_at": utcnow(), "training_seeds": [r["seed"] for r in results],
                  "results": [str(self.work / f"results_seed{r['seed']}.json") for r in results],
                  "observability": str(self.work / "observability"),
                  "verdict": "exploratory_results_available; no statistical parity claim",
                  "conditional_replications_executed": len(results) > 1,
                  "windows_live_verified": False}
        write_json(self.plan / "feasibility_result.json", result)
        self.state.update(protocol_completed=True, completed_at=utcnow())
        write_json(self.work / "state.json", self.state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = args.plan_dir.resolve()
    plan.relative_to((UNIFIED / "runs").resolve())
    campaign = Campaign(plan)
    try:
        campaign.run()
    except BaseException as exc:
        campaign.state.update(protocol_completed=False, error=f"{type(exc).__name__}: {exc}", updated_at=utcnow())
        write_json(campaign.work / "state.json", campaign.state)
        raise


if __name__ == "__main__":
    main()
