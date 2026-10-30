"""F0 activity 3: separate census of the two B0820 training runs.

Reads ``summary.json`` and ``train_iterations.jsonl`` of
``MLP_ExNovo_B0820_from_zero_50iter`` (FSM v2; resumes the critic warmup,
it is NOT a random initialisation) and
``MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter`` (FSM v3 + corrected causal
corridor). Writes ``census/census_<run>_<stamp>.json`` and
``census/census_b0820_<stamp>.md`` (timestamped, no-clobber).

Static script: stdlib + PyYAML only (no torch/ray/opensim, no interpreter
selection).

Counter semantics: the ``env_runners/episode_end/*`` termination counters are
RLlib *lifetime* counters restored from the warmup checkpoint. They are NEVER
summed across iterations here. The census reports the lifetime value at
resume (warmup last row), the lifetime value of the final row, their
difference (``delta_over_run``) and, per iteration, the delta versus the
previous row; the first-row delta is measured against warmup last row.
All counter mappings use ``ordered_keys = sorted(keys_seen)`` so the
artefacts are byte-reproducible across runs.

Environment-runner / batch audit: answers the user's doubt about "4096
environments that kept increasing". The runners stayed 12 (no growth); the
growing number is ``num_env_steps_sampled_lifetime`` (cumulative, restored
from the warmup: 27648 -> 253440) which adds exactly 4608 per iteration
because the resolved config sets ``train_batch_size = 4608`` (not 4096) under
exact-start sampling: 12 runners / 3 starts = 4 runners per start,
``rollout_fragment_length = 384`` -> 12 x 384 = 4608 = 3 x 1536, divisible by
the 512 minibatch (9 minibatches); 4096 is not divisible by 12 and would
violate the exact-start contract.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402

RUNS = {
    "MLP_ExNovo_B0820_from_zero_50iter": C.B0820_V2_RUN,
    "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter": C.B0820_V3_RUN,
}
TERM_PREFIX = "env_runners/episode_end/"
CURRENT_PREFIX = "env_runners/episode_start_steps_current/"
LIFETIME_PREFIX = "env_runners/episode_start_steps/"
MINUS020_LABEL = "offset_1p756871s"
COUNTER_SEMANTICS = (
    "RLlib lifetime counters restored from the warmup checkpoint; never summed across iterations. "
    "Reported: lifetime at resume (warmup last row), lifetime of the final row, delta_over_run = final - resume, "
    "and per-row delta versus the previous row; first-row delta is measured against warmup last row."
)
ENV_RUNNER_ANSWER = (
    "Gli environment runner sono rimasti 12 per tutta la run (config risolta e summary); non c'e' alcuna crescita del numero di environment. "
    "Il contatore crescente osservato e' num_env_steps_sampled_lifetime: cumulativo e ripristinato dal checkpoint del warmup "
    "(5 x 4608 = 23040 alla ripresa; prima riga 27648, ultima 253440). Ogni iterazione aggiunge esattamente 4608 step perche' la config risolta "
    "imposta train_batch_size = 4608, non 4096, sotto exact-start: 12 runner / 3 start = 4 runner per start, rollout_fragment_length = 384, "
    "quindi 12 x 384 = 4608 = 3 x 1536 step per start; inoltre 4608 e' divisibile per il minibatch 512 (9 minibatch), mentre 4096 non e' divisibile per 12 "
    "(4096 mod 12 = 4) e violerebbe il contratto exact-start. Il 4096 ricordato proviene dalla baseline imitativa V26 (13 runner, exact-start disabilitato) "
    "e dai due cap diagnostici morphology_*_max_samples = 4096, che non hanno relazione con il numero di environment."
)


def _get(d: dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = d
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _term_lifetime(row: dict[str, Any]) -> dict[str, float]:
    raw = {k[len(TERM_PREFIX):]: float(v or 0) for k, v in (row.get("termination_metrics") or {}).items() if k.startswith(TERM_PREFIX)}
    return {k: raw[k] for k in sorted(raw)}


def _load_rows(run_dir: Path) -> list[dict[str, Any]]:
    with (run_dir / "train_iterations.jsonl").open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _unique(values: list[Any]) -> list[Any]:
    return sorted({v for v in values if v is not None}, key=lambda x: (str(type(x)), x))


def env_runner_and_batch_audit(rows: list[dict[str, Any]], summary: dict[str, Any], cfg: dict[str, Any], warmup_last_lifetime_steps: float | None, v26_cfg: dict[str, Any]) -> dict[str, Any]:
    flat_cfg = C.flatten(cfg)
    lifetimes = [r.get("num_env_steps_sampled_lifetime") for r in rows]
    deltas = [b - a for a, b in zip(lifetimes[:-1], lifetimes[1:]) if a is not None and b is not None]
    current_per_start = _unique([float(v) for r in rows for k, v in (r.get("start_coverage_metrics") or {}).items() if k.startswith(CURRENT_PREFIX)])
    lifetime_per_start_first = {k[len(LIFETIME_PREFIX):]: v for k, v in sorted((rows[0].get("start_coverage_metrics") or {}).items()) if k.startswith(LIFETIME_PREFIX)} if rows else {}
    lifetime_per_start_last = {k[len(LIFETIME_PREFIX):]: v for k, v in sorted((rows[-1].get("start_coverage_metrics") or {}).items()) if k.startswith(LIFETIME_PREFIX)} if rows else {}
    lm0 = rows[0].get("learner_metrics", {}) if rows else {}
    batch = int(_get(cfg, "ppo", "train_batch_size") or 0)
    minibatch = int(_get(cfg, "ppo", "minibatch_size") or 0)
    runners = int(_get(cfg, "parallelism", "num_env_runners") or 0)
    n_starts = len(_get(cfg, "simulation", "episode_start_offset_choices_s") or [])
    frag = _unique([_get(r, "exact_start_balance", "rollout_fragment_length") for r in rows])
    rps = _unique([_get(r, "exact_start_balance", "runners_per_start") for r in rows])
    return {
        "question": "I 4096 environment aumentavano?",
        "answer": ENV_RUNNER_ANSWER,
        "num_env_runners_resolved_config": runners,
        "num_env_runners_summary": summary.get("num_env_runners"),
        "num_env_runners_constant": runners == summary.get("num_env_runners") and rps == [runners // max(n_starts, 1)],
        "local_inference_sync_runner": "1 runner locale di inference/sync non campiona il batch remoto (non sommato agli environment)",
        "exact_start_sampling_resolved_config": _get(cfg, "parallelism", "exact_start_sampling"),
        "exact_start_sampling_summary": summary.get("exact_start_sampling"),
        "exact_start_sampling_contract_summary": summary.get("exact_start_sampling_contract"),
        "start_offsets_s": _get(cfg, "simulation", "episode_start_offset_choices_s"),
        "train_batch_size_resolved_config": batch,
        "minibatch_size_resolved_config": minibatch,
        "rollout_fragment_length_unique_over_iterations": frag,
        "runners_per_start_unique_over_iterations": rps,
        "steps_per_start_current_unique_over_iterations": current_per_start,
        "interleaved_rows_per_start_unique_over_iterations": _unique([_get(r, "exact_start_balance", "interleaved_rows_per_start") for r in rows]),
        "module_steps_trained_unique_over_iterations": _unique([_get(r, "exact_start_balance", "module_steps_trained") for r in rows]),
        "kl_minibatch_count_unique_over_iterations": _unique([_get(r, "exact_start_balance", "kl_minibatch_count") for r in rows]),
        "exact_start_balance_pass_rows": sum(1 for r in rows if _get(r, "exact_start_balance", "pass") is True),
        "rows": len(rows),
        "num_env_steps_sampled_lifetime_warmup_last_row": warmup_last_lifetime_steps,
        "num_env_steps_sampled_lifetime_first_row": lifetimes[0] if lifetimes else None,
        "num_env_steps_sampled_lifetime_last_row": lifetimes[-1] if lifetimes else None,
        "num_env_steps_sampled_lifetime_first_row_equals_warmup_plus_batch": (lifetimes[0] == (warmup_last_lifetime_steps or 0) + batch) if lifetimes else None,
        "num_env_steps_sampled_delta_per_iteration_unique": sorted({float(d) for d in deltas}),
        "episode_start_steps_lifetime_per_start_first_row": lifetime_per_start_first,
        "episode_start_steps_lifetime_per_start_last_row": lifetime_per_start_last,
        "learner_first_row": {
            "num_env_steps_trained (righe di ottimizzazione ripetute, 9 minibatch x righe pre-compaction)": lm0.get("learners/__all_modules__/num_env_steps_trained"),
            "num_module_steps_trained (transizioni uniche addestrate)": lm0.get("learners/__all_modules__/num_module_steps_trained"),
            "pre_compaction_rows": _get(rows[0], "exact_start_balance", "pre_compaction_rows") if rows else None,
            "removed_compaction_rows": _get(rows[0], "exact_start_balance", "removed_compaction_rows") if rows else None,
        },
        "arithmetic": {
            "runners_x_fragment": runners * (frag[0] if frag else 0),
            "starts_x_steps_per_start": n_starts * (current_per_start[0] if current_per_start else 0),
            "minibatches_x_minibatch": (batch // minibatch if minibatch else None, (batch // minibatch) * minibatch if minibatch else None),
            "4608_mod_12": 4608 % 12,
            "4608_mod_512": 4608 % 512,
            "4096_mod_12": 4096 % 12,
            "4096_mod_512": 4096 % 512,
        },
        "values_equal_to_4096_in_resolved_config": sorted(k for k, v in flat_cfg.items() if v == 4096),
        "origin_of_4096_imitation_baseline_v26": {
            "train_batch_size": _get(v26_cfg, "ppo", "train_batch_size"),
            "num_env_runners": _get(v26_cfg, "parallelism", "num_env_runners"),
            "exact_start_sampling": _get(v26_cfg, "parallelism", "exact_start_sampling"),
        },
    }


def census_run(name: str, run_dir: Path, lifetime_at_resume: dict[str, float], warmup_last_lifetime_steps: float | None, v26_cfg: dict[str, Any]) -> dict[str, Any]:
    summary = C.read_json(run_dir / "summary.json")
    cfg = C.load_yaml(run_dir / "training_cfg.resolved.yaml")
    rows = _load_rows(run_dir)
    per_iter = []
    previous = dict(lifetime_at_resume)
    keys_seen: set[str] = set(lifetime_at_resume)
    for row in rows:
        lm = row.get("learner_metrics", {})
        lifetime = _term_lifetime(row)
        keys_seen |= set(lifetime)
        ordered_keys = sorted(keys_seen)
        # a counter absent from a row means "not yet emitted" -> carry previous value
        current = {k: lifetime.get(k, previous.get(k, 0.0)) for k in ordered_keys}
        delta_prev = {k: current[k] - previous.get(k, 0.0) for k in ordered_keys}
        previous = current
        per_start = row.get("per_start_metrics", {})
        starts = {label: {k: per_start[label].get(k) for k in ("sampled_steps", "completed_episodes", "episode_return_mean", "episode_length_mean", "advantage_mean", "advantage_std", "advantage_positive_fraction")} for label in sorted(per_start)}
        per_iter.append(
            {
                "iteration": row.get("iteration"),
                "episode_return_mean": row.get("episode_return_mean"),
                "episode_return_note": None if row.get("episode_return_mean") is not None else "None: nessun episodio completato nell'iterazione (RLlib non riporta il return)",
                "episode_len_mean": row.get("episode_len_mean"),
                "num_env_steps_sampled_lifetime": row.get("num_env_steps_sampled_lifetime"),
                "mean_kl_loss": row.get("mean_kl_loss"),
                "max_minibatch_mean_kl_loss": row.get("max_minibatch_mean_kl_loss"),
                "current_kl_coeff": row.get("current_kl_coeff"),
                "entropy": row.get("entropy"),
                "vf_loss": row.get("vf_loss"),
                "policy_loss": row.get("policy_loss"),
                "vf_explained_var": lm.get("learners/default_policy/vf_explained_var"),
                "lr": lm.get("learners/default_policy/default_optimizer_learning_rate"),
                "exact_start_balance_pass": _get(row, "exact_start_balance", "pass"),
                "learner_batch_pass": _get(row, "exact_start_balance", "learner_batch_pass"),
                "module_steps_trained": _get(row, "exact_start_balance", "module_steps_trained"),
                "kl_guard_pass": _get(row, "kl_update_guard", "pass"),
                "termination_lifetime_as_logged": lifetime,
                "termination_lifetime_delta_vs_previous_row": delta_prev,
                "per_start": starts,
            }
        )
    ordered_keys = sorted(keys_seen)
    returns_valid = [(i["iteration"], float(i["episode_return_mean"])) for i in per_iter if i.get("episode_return_mean") is not None and math.isfinite(float(i["episode_return_mean"]))]
    kls = [float(i["max_minibatch_mean_kl_loss"]) for i in per_iter if i.get("max_minibatch_mean_kl_loss") is not None]
    klc = [float(i["current_kl_coeff"]) for i in per_iter if i.get("current_kl_coeff") is not None]
    ent = [float(i["entropy"]) for i in per_iter if i.get("entropy") is not None]
    lifetimes = [i["num_env_steps_sampled_lifetime"] for i in per_iter]
    deltas = [b - a for a, b in zip(lifetimes[:-1], lifetimes[1:]) if a is not None and b is not None]
    lifetime_final = {k: previous.get(k, 0.0) for k in ordered_keys}
    lifetime_resume = {k: lifetime_at_resume.get(k, 0.0) for k in ordered_keys}
    delta_over_run = {k: lifetime_final[k] - lifetime_resume[k] for k in ordered_keys}
    first_row_delta = {k: per_iter[0]["termination_lifetime_delta_vs_previous_row"].get(k, 0.0) for k in ordered_keys} if per_iter else {}
    best_meta = C.read_json(run_dir / "checkpoint_best_meta.json")
    last_meta = C.read_json(run_dir / "checkpoint_last_meta.json")
    supervisor = C.read_json(run_dir / "supervisor_state.json")
    restart_audit = run_dir / "env_runner_restart_audit.json"
    first_minus = per_iter[0]["per_start"].get(MINUS020_LABEL, {}) if per_iter else {}
    return {
        "run": name,
        "revision": C.F0_REV,
        "path": C.rel(run_dir),
        "summary_sha256": C.sha256_file(run_dir / "summary.json"),
        "train_iterations_sha256": C.sha256_file(run_dir / "train_iterations.jsonl"),
        "resolved_config_sha256": C.sha256_file(run_dir / "training_cfg.resolved.yaml"),
        "resume_from": summary.get("resume_from"),
        "resume_is_critic_warmup_not_random_init": bool(summary.get("resume_from")) and "critic_warmup" in str(summary.get("resume_from")),
        "initialization_mode": summary.get("initialization_mode"),
        "restored_logical_iteration": summary.get("restored_logical_iteration"),
        "restored_training_iteration": summary.get("restored_training_iteration"),
        "warm_start_requested": summary.get("warm_start_requested"),
        "warm_start_applied": summary.get("warm_start_applied"),
        "iterations_completed": summary.get("iterations_completed"),
        "iteration_start": summary.get("iteration_start"),
        "next_iteration": summary.get("next_iteration"),
        "logical_iterations_in_jsonl": [rows[0].get("iteration"), rows[-1].get("iteration")] if rows else None,
        "jsonl_rows": len(rows),
        "env_steps_sampled_lifetime_first_last": [lifetimes[0], lifetimes[-1]],
        "env_steps_sampled_delta_per_iteration_unique": sorted({float(d) for d in deltas}),
        "started_at": summary.get("started_at"),
        "finished_at": summary.get("finished_at"),
        "elapsed_wall_time_s": summary.get("elapsed_wall_time_s"),
        "ok": summary.get("ok"),
        "stop_reason": summary.get("stop_reason"),
        "supervisor_state": supervisor,
        "env_runner_restart_audit_present": restart_audit.is_file(),
        "fsm_actor_version_in_resolved": _get(cfg, "grf", "binary_phase_actor_fsm_version", default="<absent -> v2 behaviour at that code revision>"),
        "invalid_event_policy": _get(cfg, "grf", "binary_phase_invalid_event_policy"),
        "morphology_weight": _get(cfg, "reward", "morphology_weight"),
        "num_env_runners": summary.get("num_env_runners"),
        "exact_start_sampling": summary.get("exact_start_sampling"),
        "freeze_logstd": summary.get("freeze_logstd"),
        "freeze_actor": summary.get("freeze_actor"),
        "best_episode_return_mean_summary": summary.get("best_episode_return_mean"),
        "checkpoint_best_meta": best_meta,
        "checkpoint_last_meta": last_meta,
        "iterations_without_completed_episode": [i["iteration"] for i in per_iter if i.get("episode_return_mean") is None],
        "return_first_valid": returns_valid[0] if returns_valid else None,
        "return_last": returns_valid[-1] if returns_valid else None,
        "return_min": min(returns_valid, key=lambda t: t[1]) if returns_valid else None,
        "return_max": max(returns_valid, key=lambda t: t[1]) if returns_valid else None,
        "return_mean_first10_valid": statistics.fmean([r for _, r in returns_valid[:10]]) if returns_valid else None,
        "return_mean_last10_valid": statistics.fmean([r for _, r in returns_valid[-10:]]) if returns_valid else None,
        "kl_max_minibatch_mean_max": max(kls) if kls else None,
        "kl_max_minibatch_mean_min": min(kls) if kls else None,
        "kl_coeff_first_last": [klc[0], klc[-1]] if klc else None,
        "entropy_min_max": [min(ent), max(ent)] if ent else None,
        "entropy_equiv_sigma_min_max": [math.exp((e - 2 * 0.5 * math.log(2 * math.pi * math.e)) / 2) for e in ([min(ent), max(ent)] if ent else [])],
        "exact_start_balance_pass_count": sum(1 for i in per_iter if i.get("exact_start_balance_pass") is True),
        "learner_batch_pass_count": sum(1 for i in per_iter if i.get("learner_batch_pass") is True),
        "kl_guard_pass_count": sum(1 for i in per_iter if i.get("kl_guard_pass") is True),
        "termination_counter_semantics": COUNTER_SEMANTICS,
        "termination_counter_keys_ordered": ordered_keys,
        "termination_lifetime_at_resume_warmup_last_row": lifetime_resume,
        "termination_lifetime_first_row": {k: per_iter[0]["termination_lifetime_as_logged"].get(k) for k in ordered_keys} if per_iter else {},
        "termination_first_row_delta_vs_warmup_last_row": first_row_delta,
        "termination_lifetime_final_row": lifetime_final,
        "termination_delta_over_run_final_minus_resume": delta_over_run,
        "advantage_minus020_first_row": {"iteration": per_iter[0]["iteration"] if per_iter else None, "advantage_positive_fraction": first_minus.get("advantage_positive_fraction"), "advantage_mean": first_minus.get("advantage_mean"), "completed_episodes": first_minus.get("completed_episodes")},
        "env_runner_and_batch_audit": env_runner_and_batch_audit(rows, summary, cfg, warmup_last_lifetime_steps, v26_cfg),
        "per_iteration": per_iter,
    }


def main() -> int:
    C.ensure_out_dirs()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_md = C.OUT_CENSUS / f"census_b0820_{stamp}.md"
    if out_md.exists():
        raise FileExistsError("census already exists; no-clobber")
    warmup_rows = _load_rows(C.B0820_WARMUP)
    lifetime_at_resume = _term_lifetime(warmup_rows[-1])
    warmup_last_lifetime_steps = warmup_rows[-1].get("num_env_steps_sampled_lifetime")
    v26_cfg = C.load_yaml(C.V26_IMIT_RUN / "training_cfg.resolved.yaml")
    records = []
    for name, run_dir in RUNS.items():
        print(f"[census] {name}", flush=True)
        rec = census_run(name, run_dir, lifetime_at_resume, warmup_last_lifetime_steps, v26_cfg)
        rec["stamp"] = stamp
        C.write_json(C.OUT_CENSUS / f"census_{name}_{stamp}.json", rec)
        records.append(rec)
    headers = ["voce"] + [r["run"] for r in records]
    keys = [
        ("resume_from", "resume_from"), ("riprende il warmup (non init casuale)", "resume_is_critic_warmup_not_random_init"), ("initialization_mode (summary)", "initialization_mode"),
        ("restored_logical_iteration", "restored_logical_iteration"), ("iterazioni logiche nel jsonl", "logical_iterations_in_jsonl"),
        ("righe jsonl", "jsonl_rows"), ("env steps sampled lifetime prima/ultima riga", "env_steps_sampled_lifetime_first_last"), ("delta env steps per iterazione (valori unici)", "env_steps_sampled_delta_per_iteration_unique"),
        ("FSM attore (resolved)", "fsm_actor_version_in_resolved"), ("policy eventi invalidi", "invalid_event_policy"), ("morphology_weight", "morphology_weight"),
        ("inizio", "started_at"), ("fine", "finished_at"), ("wall time [s]", "elapsed_wall_time_s"), ("stop", "stop_reason"),
        ("supervisor restart/crash", "supervisor_state"), ("env_runner_restart_audit.json presente", "env_runner_restart_audit_present"),
        ("iterazioni senza episodio completato (return None)", "iterations_without_completed_episode"),
        ("primo return valido (iter, val)", "return_first_valid"), ("return ultima", "return_last"), ("return min (iter, val)", "return_min"), ("return max (iter, val)", "return_max"),
        ("media return prime 10 valide", "return_mean_first10_valid"), ("media return ultime 10 valide", "return_mean_last10_valid"),
        ("best (meta)", "checkpoint_best_meta"), ("last (meta)", "checkpoint_last_meta"),
        ("KL max minibatch: max sulle iterazioni", "kl_max_minibatch_mean_max"), ("KL coeff prima/ultima", "kl_coeff_first_last"),
        ("entropia min/max [nat]", "entropy_min_max"), ("sigma equivalente min/max", "entropy_equiv_sigma_min_max"),
        ("exact-start balance PASS", "exact_start_balance_pass_count"), ("learner batch PASS", "learner_batch_pass_count"), ("KL guard PASS", "kl_guard_pass_count"),
        ("terminazioni: chiavi (ordinate)", "termination_counter_keys_ordered"),
        ("terminazioni: lifetime alla ripresa (ultima riga warmup)", "termination_lifetime_at_resume_warmup_last_row"), ("terminazioni: lifetime prima riga della run", "termination_lifetime_first_row"),
        ("terminazioni: delta prima riga vs ultima riga warmup", "termination_first_row_delta_vs_warmup_last_row"),
        ("terminazioni: lifetime finale (ultima riga)", "termination_lifetime_final_row"), ("terminazioni: delta run = lifetime finale - lifetime alla ripresa", "termination_delta_over_run_final_minus_resume"),
        ("advantage start -0.20, prima riga", "advantage_minus020_first_row"),
    ]
    audit_keys = [
        ("env runner (config risolta / summary)", lambda a: f"{a['num_env_runners_resolved_config']} / {a['num_env_runners_summary']}"),
        ("runner costanti su tutte le iterazioni", lambda a: a["num_env_runners_constant"]),
        ("runner locale inference/sync", lambda a: a["local_inference_sync_runner"]),
        ("exact-start (config / summary)", lambda a: f"{a['exact_start_sampling_resolved_config']} / {a['exact_start_sampling_summary']}"),
        ("start esatti [s]", lambda a: a["start_offsets_s"]),
        ("train_batch_size (config risolta)", lambda a: a["train_batch_size_resolved_config"]),
        ("minibatch_size", lambda a: a["minibatch_size_resolved_config"]),
        ("rollout_fragment_length (valori unici)", lambda a: a["rollout_fragment_length_unique_over_iterations"]),
        ("runner per start (valori unici)", lambda a: a["runners_per_start_unique_over_iterations"]),
        ("step correnti per start (valori unici)", lambda a: a["steps_per_start_current_unique_over_iterations"]),
        ("righe interleaved per start (valori unici)", lambda a: a["interleaved_rows_per_start_unique_over_iterations"]),
        ("module_steps_trained (valori unici)", lambda a: a["module_steps_trained_unique_over_iterations"]),
        ("minibatch KL per update (valori unici)", lambda a: a["kl_minibatch_count_unique_over_iterations"]),
        ("exact-start balance PASS / righe", lambda a: f"{a['exact_start_balance_pass_rows']} / {a['rows']}"),
        ("num_env_steps_sampled_lifetime: ultima riga warmup / prima riga / ultima riga", lambda a: f"{a['num_env_steps_sampled_lifetime_warmup_last_row']} / {a['num_env_steps_sampled_lifetime_first_row']} / {a['num_env_steps_sampled_lifetime_last_row']}"),
        ("prima riga == warmup + batch", lambda a: a["num_env_steps_sampled_lifetime_first_row_equals_warmup_plus_batch"]),
        ("delta lifetime per iterazione (valori unici)", lambda a: a["num_env_steps_sampled_delta_per_iteration_unique"]),
        ("episode_start_steps lifetime per start: prima riga", lambda a: a["episode_start_steps_lifetime_per_start_first_row"]),
        ("episode_start_steps lifetime per start: ultima riga", lambda a: a["episode_start_steps_lifetime_per_start_last_row"]),
        ("learner prima riga", lambda a: a["learner_first_row"]),
        ("aritmetica", lambda a: a["arithmetic"]),
        ("chiavi con valore 4096 nella config risolta", lambda a: a["values_equal_to_4096_in_resolved_config"]),
        ("origine del 4096: baseline imitativa V26", lambda a: a["origin_of_4096_imitation_baseline_v26"]),
    ]

    def fmt(v: Any) -> str:
        if isinstance(v, dict):
            return "; ".join(f"{k}={('%.4g' % vv) if isinstance(vv, float) else vv}" for k, vv in v.items() if k not in ("checkpoint",))
        if isinstance(v, (list, tuple)):
            return ", ".join(("%.6g" % x) if isinstance(x, float) else str(x) for x in v)
        if isinstance(v, float):
            return "%.6g" % v
        return str(v)

    rows = [[label] + [fmt(r.get(key)) for r in records] for label, key in keys]
    audit_rows = [[label] + [fmt(fn(r["env_runner_and_batch_audit"])) for r in records] for label, fn in audit_keys]
    lines = [
        f"# F0 — Censimento separato delle due run B0820 — revisione {C.F0_REV} — stamp {stamp}",
        "",
        f"Generato: {C.utc_now()} — git HEAD `{C.git_snapshot()['head'][:12]}`. Sostituisce, nella stessa revisione, l'eventuale `census_b0820.md` senza stamp (privo della sezione runner/batch).",
        "",
        "Chiarimento sul nome: `MLP_ExNovo_B0820_from_zero_50iter` **non** e' un'inizializzazione casuale: riprende `checkpoint_last` del critic warmup (actor trapiantato 39->35 + critic caldo, iterazione logica 5) ed e' la run con FSM attore v2 e corridoio causale pre-fix. `MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter` riprende lo stesso checkpoint con FSM v3 e corridoio corretto (la correzione del corridoio e' una differenza di codice, non di yaml).",
        "",
        f"Semantica dei contatori di terminazione: {COUNTER_SEMANTICS}",
        "",
        C.md_table(headers, rows),
        "",
        "## Risposta al dubbio sui «4096 environment che aumentavano»",
        "",
        ENV_RUNNER_ANSWER,
        "",
        C.md_table(["evidenza"] + [r["run"] for r in records], audit_rows),
        "",
        "## Andamento per iterazione (return medio, KL max minibatch, entropia, delta terminazioni vs riga precedente; prima riga vs ultima riga warmup)",
        "",
    ]
    for rec in records:
        lines += [f"### {rec['run']}", "", C.md_table(
            ["iter", "return", "len", "KL max mb", "KL coeff", "entropia", "vf_loss", "vf_expl_var", "exact-start", "Δ time_limit", "Δ stance_to", "Δ swing_to", "Δ contract_fail", "Δ penetr", "adv+ frac -0.20"],
            [[
                i["iteration"],
                ("%.4g" % i["episode_return_mean"]) if i.get("episode_return_mean") is not None else "None",
                ("%.4g" % i["episode_len_mean"]) if i.get("episode_len_mean") is not None else "None",
                "%.3g" % (i.get("max_minibatch_mean_kl_loss") or 0), "%.3g" % (i.get("current_kl_coeff") or 0), "%.4g" % (i.get("entropy") or 0), "%.4g" % (i.get("vf_loss") or 0), "%.3g" % (i.get("vf_explained_var") or 0),
                i.get("exact_start_balance_pass"),
                int(i["termination_lifetime_delta_vs_previous_row"].get("episode_time_limit", 0)), int(i["termination_lifetime_delta_vs_previous_row"].get("phase_timeout_stance", 0)), int(i["termination_lifetime_delta_vs_previous_row"].get("phase_timeout_swing", 0)), int(i["termination_lifetime_delta_vs_previous_row"].get("morphology_causal_contract_failure", 0)), int(i["termination_lifetime_delta_vs_previous_row"].get("grf_penetration", 0)),
                ("%.3f" % i["per_start"].get(MINUS020_LABEL, {}).get("advantage_positive_fraction")) if i["per_start"].get(MINUS020_LABEL, {}).get("advantage_positive_fraction") is not None else "-",
            ] for i in rec["per_iteration"]],
        ), ""]
    C.write_text(out_md, "\n".join(lines))
    print(f"[census] written {out_md}")
    for rec in records:
        a = rec["env_runner_and_batch_audit"]
        print(f"[census] {rec['run']}: runners {a['num_env_runners_resolved_config']}/{a['num_env_runners_summary']} constant={a['num_env_runners_constant']} batch={a['train_batch_size_resolved_config']} frag={a['rollout_fragment_length_unique_over_iterations']} rps={a['runners_per_start_unique_over_iterations']} per-start={a['steps_per_start_current_unique_over_iterations']} lifetime {a['num_env_steps_sampled_lifetime_warmup_last_row']}->{a['num_env_steps_sampled_lifetime_first_row']}->{a['num_env_steps_sampled_lifetime_last_row']} delta={a['num_env_steps_sampled_delta_per_iteration_unique']} first==warmup+batch={a['num_env_steps_sampled_lifetime_first_row_equals_warmup_plus_batch']} 4096 keys={a['values_equal_to_4096_in_resolved_config']}")
        print(f"[census] {rec['run']}: lifetime final {rec['termination_lifetime_final_row']} | delta over run {rec['termination_delta_over_run_final_minus_resume']} | first-row delta vs warmup last row {rec['termination_first_row_delta_vs_warmup_last_row']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
