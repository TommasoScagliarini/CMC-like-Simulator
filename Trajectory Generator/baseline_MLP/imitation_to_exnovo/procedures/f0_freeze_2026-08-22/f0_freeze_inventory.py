"""F0 activity 1+2: freeze digests/manifests and build the provenance matrix.

Read-only over the frozen artefacts (exported actor modules, full Algorithm
checkpoints, resolved configs, profiles). Writes under ``<OUT_ROOT>/manifest``:
``freeze_inventory.json/.md`` and ``provenance_matrix.json/.md``. Refuses to
overwrite.

Digest conventions (both reported):
  * ``actor_digest_warm_start``  - warm_start.actor_state_digest: all actor
    tensors (pi_encoder.* + pi.*), raw dtype bytes; used by transplant
    reports, freeze audits and feature manifests;
  * ``actor_digest_pi_canonical`` - compare_policy_checkpoints._actor_digest:
    keys starting with ``pi`` only; used by the July drift audit.

Where the critic lives: exported ``rl_module_*`` directories are actor-only
(no ``vf*`` keys) so a critic digest is NOT APPLICABLE there; the critic and
the optimizer are read from the Algorithm checkpoint
(``learner_group/learner/rl_module/default_policy/module_state.pkl`` and
``learner_group/learner/state.pkl``). The MultiRLModule-level
``learner_group/learner/rl_module/module_state.pkl`` is an empty mapping and
is deliberately ignored (first-pass r1 bug).
"""

from __future__ import annotations

import hashlib
import json
import math
import pickle
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402

sys.path.insert(0, str(C.BASELINE_DIR))
sys.path.insert(0, str(C.ROOT_VALIDATION))
import warm_start  # noqa: E402
import compare_policy_checkpoints as cpc  # noqa: E402

import numpy as np  # noqa: E402

NA = "N/A"


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def file_entries(directory: Path, *, recursive: bool = False) -> list[dict[str, Any]]:
    """Sorted file list with POSIX-relative names (tree digests are host-portable)."""
    entries = []
    iterator = directory.rglob("*") if recursive else directory.iterdir()
    for path in sorted(iterator, key=lambda p: p.relative_to(directory).as_posix()):
        if path.is_file():
            entries.append(
                {
                    "file": path.relative_to(directory).as_posix(),
                    "bytes": path.stat().st_size,
                    "mtime_iso": C.iso_mtime(path),
                    "sha256": C.sha256_file(path),
                }
            )
    return entries


def logstd_head_stats(state: dict[str, Any]) -> dict[str, Any]:
    weight = _array(state["pi.1.weight"]).astype(np.float64)
    bias = _array(state["pi.1.bias"]).astype(np.float64)
    n_out = int(weight.shape[0])
    action_dim = n_out // 2
    mean_rows = weight[:action_dim]
    logstd_rows = weight[action_dim:]
    logstd_bias = bias[action_dim:]
    rows_abs_max = float(np.max(np.abs(logstd_rows))) if logstd_rows.size else 0.0
    rows_zero = bool(rows_abs_max == 0.0)
    sigma_if_rows_zero = [float(math.exp(b)) for b in logstd_bias]
    entropy_if_rows_zero = float(sum(0.5 * math.log(2 * math.pi * math.e) + b for b in logstd_bias))
    return {
        "head_outputs": n_out,
        "action_dim": action_dim,
        "mean_rows_l2": float(np.linalg.norm(mean_rows)),
        "logstd_rows_l2": float(np.linalg.norm(logstd_rows)),
        "logstd_rows_abs_max": rows_abs_max,
        "logstd_rows_all_zero": rows_zero,
        "logstd_bias": [float(b) for b in logstd_bias],
        "sigma_if_rows_zero": sigma_if_rows_zero,
        "entropy_nat_if_rows_zero": entropy_if_rows_zero,
        "sigma_regime": (
            "constant (state-independent): rows zero, sigma=exp(bias)"
            if rows_zero
            else "state-dependent: log-std rows non-zero, sigma varies with observation (bias value is sigma at zero input only)"
        ),
    }


def state_record(state: dict[str, Any]) -> dict[str, Any]:
    keys = sorted(str(k) for k in state)
    record: dict[str, Any] = {
        "state_keys": {
            key: {
                "shape": list(_array(state[key]).shape),
                "dtype": str(_array(state[key]).dtype),
                "l2_norm": float(np.linalg.norm(_array(state[key]).astype(np.float64))),
            }
            for key in keys
        }
    }
    try:
        record["actor_digest_warm_start"] = warm_start.actor_state_digest(state)
    except KeyError as exc:
        record["actor_digest_warm_start"] = f"<unavailable: {exc}>"
    record["actor_digest_pi_canonical"] = cpc._actor_digest(state)
    non_actor = warm_start.compare_non_actor_states(state, state)
    record["non_actor_keys"] = non_actor["keys"]
    if non_actor["keys"]:
        record["critic_digest"] = non_actor["expected_digest"]
        record["critic_digest_status"] = "computed"
    else:
        record["critic_digest"] = None
        record["critic_digest_status"] = "N/A: state is actor-only (no vf* keys); critic lives in the Algorithm checkpoint learner module"
    first_key = "pi_encoder.0.weight" if "pi_encoder.0.weight" in state else "pi.0.0.weight"
    record["first_layer_width"] = int(_array(state[first_key]).shape[1])
    record["logstd_head"] = logstd_head_stats(state)
    return record


def module_record(name: str, module_dir: Path, *, width_expected: int | None = None) -> dict[str, Any]:
    record: dict[str, Any] = {"name": name, "path": C.rel(module_dir), "exists": module_dir.is_dir()}
    if not module_dir.is_dir():
        return record
    record["files"] = file_entries(module_dir)
    state = warm_start.load_module_state(module_dir)
    record.update(state_record(state))
    record["first_layer_width_expected"] = width_expected
    record["first_layer_width_ok"] = (width_expected is None) or (record["first_layer_width"] == width_expected)
    manifest = module_dir / "actor_feature_manifest.json"
    if not manifest.is_file():
        manifest = module_dir.parent / "actor_feature_manifest.json"
    if manifest.is_file():
        payload = C.read_json(manifest)
        record["actor_feature_manifest"] = {
            "path": C.rel(manifest),
            "sha256": C.sha256_file(manifest),
            "feature_count": payload.get("actor_feature_count", len(payload.get("actor_feature_names", []))),
            "declared_actor_digest": payload.get("actor_digest"),
            "declared_matches_warm_start_digest": payload.get("actor_digest") == record["actor_digest_warm_start"],
            "feature_names": payload.get("actor_feature_names"),
        }
    else:
        record["actor_feature_manifest"] = None
    meta = module_dir / "metadata.json"
    record["metadata_json"] = C.read_json(meta) if meta.is_file() else None
    return record


def optimizer_record(state_path: Path) -> dict[str, Any]:
    record: dict[str, Any] = {"path": C.rel(state_path), "exists": state_path.is_file()}
    if not state_path.is_file():
        return record
    record["sha256"] = C.sha256_file(state_path)
    record["bytes"] = state_path.stat().st_size
    try:
        with state_path.open("rb") as handle:
            payload = pickle.load(handle)
    except Exception as exc:  # pragma: no cover - best effort
        record["parse"] = f"unpickle failed: {type(exc).__name__}: {exc}"
        return record
    record["top_level_keys"] = list(payload.keys()) if isinstance(payload, dict) else type(payload).__name__
    record["weights_seq_no"] = payload.get("weights_seq_no") if isinstance(payload, dict) else None
    optim = payload.get("optimizer") if isinstance(payload, dict) else None
    opt_info: dict[str, Any] = {}
    if isinstance(optim, dict):
        for opt_name, opt_payload in optim.items():
            info: dict[str, Any] = {"module_id": opt_payload.get("module_id") if isinstance(opt_payload, dict) else None}
            torch_state = opt_payload.get("state") if isinstance(opt_payload, dict) else None
            # torch optimizer state_dict: {"state": {idx: {...}}, "param_groups": [...]}
            holder = None
            if isinstance(torch_state, dict) and "param_groups" in torch_state:
                holder = torch_state
            elif isinstance(torch_state, dict):
                for value in torch_state.values():
                    if isinstance(value, dict) and "param_groups" in value:
                        holder = value
                        break
            if holder is not None:
                groups = holder.get("param_groups") or []
                moments = holder.get("state") or {}
                steps = []
                for entry in moments.values():
                    if isinstance(entry, dict) and "step" in entry:
                        step = entry["step"]
                        steps.append(float(_array(step).reshape(-1)[0]) if hasattr(step, "shape") or hasattr(step, "detach") else float(step))
                info.update(
                    {
                        "param_groups": len(groups),
                        "lr": [g.get("lr") for g in groups],
                        "betas": [g.get("betas") for g in groups],
                        "params_in_groups": sum(len(g.get("params", [])) for g in groups),
                        "params_with_moment_state": len(moments),
                        "adam_step_min_max": [min(steps), max(steps)] if steps else None,
                    }
                )
            else:
                info["structure"] = str(type(torch_state).__name__)
            opt_info[opt_name] = info
    record["optimizers"] = opt_info
    return record


def checkpoint_record(name: str, ckpt_dir: Path) -> dict[str, Any]:
    record: dict[str, Any] = {"name": name, "path": C.rel(ckpt_dir), "exists": ckpt_dir.is_dir()}
    if not ckpt_dir.is_dir():
        return record
    files = file_entries(ckpt_dir, recursive=True)
    record["files"] = files
    record["file_count"] = len(files)
    record["total_bytes"] = sum(f["bytes"] for f in files)
    aggregate = hashlib.sha256()
    for f in files:
        aggregate.update(f["file"].encode("utf-8"))
        aggregate.update(f["sha256"].encode("ascii"))
    record["tree_digest"] = aggregate.hexdigest()
    record["tree_digest_convention"] = "sha256 over (posix relative path, file sha256) pairs in posix-sorted order; host-portable"
    rllib_json = ckpt_dir / "rllib_checkpoint.json"
    record["rllib_checkpoint_json"] = C.read_json(rllib_json) if rllib_json.is_file() else None
    meta = ckpt_dir.parent / f"{ckpt_dir.name}_meta.json"
    record["meta_json"] = C.read_json(meta) if meta.is_file() else None
    learner_module = ckpt_dir / C.LEARNER_MODULE_REL
    multi_module_state = ckpt_dir / "learner_group" / "learner" / "rl_module" / "module_state.pkl"
    record["multi_rl_module_state_bytes"] = multi_module_state.stat().st_size if multi_module_state.is_file() else None
    if (learner_module / "module_state.pkl").is_file():
        state = warm_start.load_module_state(learner_module)
        emb = {"path": C.rel(learner_module), "selection_rule": "learner_group/learner/rl_module/default_policy (actor + critic); MultiRLModule-level module_state.pkl ignored"}
        emb.update(state_record(state))
        emb.pop("state_keys", None)
        emb["state_key_names"] = sorted(str(k) for k in state)
        record["embedded_learner_module"] = emb
    else:
        record["embedded_learner_module"] = {"path": C.rel(learner_module), "error": "missing module_state.pkl"}
    record["learner_state"] = optimizer_record(ckpt_dir / C.LEARNER_STATE_REL)
    env_runner_state = ckpt_dir / C.ENV_RUNNER_STATE_REL
    record["env_runner_state"] = {
        "path": C.rel(env_runner_state),
        "bytes": env_runner_state.stat().st_size if env_runner_state.is_file() else None,
        "sha256": C.sha256_file(env_runner_state) if env_runner_state.is_file() else None,
        "note": "EnvRunner weights are not persisted separately in this layout (synced from the learner at restore); only connector/runner state",
    }
    return record


def file_record(name: str, path: Path, *, load_json_keys: bool = False) -> dict[str, Any]:
    record: dict[str, Any] = {"name": name, "path": C.rel(path), "exists": path.is_file()}
    if not path.is_file():
        return record
    record["bytes"] = path.stat().st_size
    record["mtime_iso"] = C.iso_mtime(path)
    record["sha256"] = C.sha256_file(path)
    record["git_blob"] = C.git_blob_hash(path)
    record["git_tracked"] = record["git_blob"] is not None
    if load_json_keys and path.suffix == ".json":
        payload = C.read_json(path)
        record["top_level_keys"] = list(payload.keys()) if isinstance(payload, dict) else f"<{type(payload).__name__}>"
    return record


def run_record(name: str, run_dir: Path) -> dict[str, Any]:
    record: dict[str, Any] = {"name": name, "path": C.rel(run_dir), "exists": run_dir.is_dir()}
    if not run_dir.is_dir():
        return record
    for fname in ("summary.json", "train_iterations.jsonl", "training_cfg.resolved.yaml", "actor_transplant_report.json", "checkpoint_best_meta.json", "checkpoint_last_meta.json", "supervisor_state.json", "run_summary.json", "adaptation_report.json", "actor_feature_manifest.json", "policy_drift_from_h0_milestones.json", "env_runner_restart_audit.json"):
        path = run_dir / fname
        if path.is_file():
            record[fname] = file_record(fname, path)
    summary = run_dir / "summary.json"
    if summary.is_file():
        payload = C.read_json(summary)
        record["summary_fields"] = {
            k: payload.get(k)
            for k in (
                "ok", "stop_reason", "started_at", "finished_at", "elapsed_wall_time_s",
                "iterations_run", "iterations_completed", "iteration_start", "next_iteration",
                "resume_from", "restored_training_iteration", "restored_logical_iteration",
                "warm_start_source", "warm_start_mode", "freeze_logstd", "freeze_actor",
                "best_episode_return_mean", "num_env_runners", "exact_start_sampling",
            )
        }
    jsonl = run_dir / "train_iterations.jsonl"
    if jsonl.is_file():
        with jsonl.open("r", encoding="utf-8") as handle:
            rows = [json.loads(line) for line in handle if line.strip()]
        record["train_iterations_rows"] = len(rows)
        record["train_iterations_first_last"] = [rows[0].get("iteration"), rows[-1].get("iteration")] if rows else None
    for meta_name in ("checkpoint_best_meta.json", "checkpoint_last_meta.json"):
        path = run_dir / meta_name
        if path.is_file():
            record[meta_name.replace(".json", "")] = C.read_json(path)
    milestones = sorted(p.name for p in run_dir.glob("milestone_iteration_*") if p.is_dir())
    if milestones:
        record["milestones"] = {"count": len(milestones), "first": milestones[0], "last": milestones[-1]}
    return record


def build_provenance_matrix(inventory: dict[str, Any]) -> dict[str, Any]:
    modules = {m["name"]: m for m in inventory["modules"]}
    ckpts = {c["name"]: c for c in inventory["full_checkpoints"]}
    b_report = C.read_json(C.B0820_WARMUP / "actor_transplant_report.json")
    j_report = C.read_json(C.JUL_WARMUP / "actor_transplant_report.json")
    adapt = C.read_json(C.JUL_ADAPT_RUN / "adaptation_report.json")

    def cfg_runtime(run_dir: Path) -> dict[str, Any]:
        cfg = C.flatten(C.load_yaml(run_dir / "training_cfg.resolved.yaml"))
        return {
            "resolved_config": C.rel(run_dir / "training_cfg.resolved.yaml"),
            "resolved_config_sha256": C.sha256_file(run_dir / "training_cfg.resolved.yaml"),
            "grf_profile": cfg.get("grf.online_grf_profile"),
            "binary_phase_fsm_mode": cfg.get("grf.binary_phase_fsm_mode", "<absent: legacy detector>"),
            "binary_phase_actor_fsm_version": cfg.get("grf.binary_phase_actor_fsm_version", "<absent>"),
            "binary_phase_invalid_event_policy": cfg.get("grf.binary_phase_invalid_event_policy", "<absent>"),
            "penetration_m": [cfg.get("simulation.grf_penetration_penalty_threshold_m"), cfg.get("simulation.grf_penetration_termination_m")],
            "swing_hard_timeout_s": cfg.get("reward.phase_swing_hard_timeout_s"),
            "reward_mode": cfg.get("reward.reward_mode"),
            "morphology": [cfg.get("reward.morphology_profile"), cfg.get("reward.morphology_phase_mode", "<absent>"), cfg.get("reward.morphology_weight")],
            "exact_start_sampling": cfg.get("parallelism.exact_start_sampling", "<absent>"),
            "freeze_logstd": cfg.get("model.freeze_logstd", "<absent>"),
            "freeze_actor": cfg.get("model.freeze_actor", "<absent>"),
            "gait_clock_enable": cfg.get("simulation.gait_clock_enable", "<absent>"),
        }

    def entry(cid: str, ckpt_name: str | None, *, actor_origin: str, critic_origin: str, optimizer_origin: str, obs_schema: str, normalization: str, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        mod = modules[cid]
        ck = ckpts.get(ckpt_name) if ckpt_name else None
        emb = (ck or {}).get("embedded_learner_module") or {}
        opt = (ck or {}).get("learner_state") or {}
        manifest = mod.get("actor_feature_manifest") or {}
        row = {
            "candidate": cid,
            "module": mod["path"],
            "actor_weights": {"origin": actor_origin, "digest_warm_start": mod["actor_digest_warm_start"], "digest_pi_canonical": mod["actor_digest_pi_canonical"], "first_layer_width": mod["first_layer_width"]},
            "critic": {"origin": critic_origin, "digest": emb.get("critic_digest"), "source": emb.get("path"), "status": emb.get("critic_digest_status", "N/A: no full checkpoint")},
            "optimizer": {"origin": optimizer_origin, "learner_state_sha256": opt.get("sha256"), "weights_seq_no": opt.get("weights_seq_no"), "optimizers": opt.get("optimizers")},
            "log_std": {"regime": mod["logstd_head"]["sigma_regime"], "rows_all_zero": mod["logstd_head"]["logstd_rows_all_zero"], "bias": mod["logstd_head"]["logstd_bias"], "sigma_if_constant": mod["logstd_head"]["sigma_if_rows_zero"], "entropy_nat_if_constant": mod["logstd_head"]["entropy_nat_if_rows_zero"]},
            "observation_schema": {"description": obs_schema, "actor_width": mod["first_layer_width"], "manifest": manifest.get("path"), "manifest_sha256": manifest.get("sha256"), "feature_names": manifest.get("feature_names")},
            "normalization": normalization,
            "runtime": cfg_runtime(C.CANDIDATES[cid]["producing_run"]),
        }
        if extra:
            row.update(extra)
        return row

    rows = [
        entry(
            "JUL_H0", "JUL_WARMUP_checkpoint_last",
            actor_origin=f"June 23 imitation best (31-slice, digest {modules['JUN23_IMIT_BEST']['actor_digest_warm_start'][:12]}) -> zero-iter port to 35D markov (digest {adapt['source_actor_digest'][:12]}) -> supervised target-domain adaptation markov35 phase-aligned r32 (24712 samples, 400 epochs, full actor, logstd head frozen; digest {adapt['adapted_actor_digest'][:12]}) -> transplanted 35->35 (drop mode, gait_phase_sin/cos zeroed) and frozen during critic warmup (1 iter)",
            critic_origin="fresh critic trained 1 warmup iteration (freeze_actor=true, freeze_logstd=false in warmup config but log-std head bit-identical per freeze audit)",
            optimizer_origin="fresh Adam in warmup (1 learner update on critic only); restored by the pilot as its starting state",
            obs_schema="35 deployable Markov features (gait_phase_sin/cos present but zeroed; legacy HS/TO detector events; FSM legacy)",
            normalization="no runtime observation normalisation; July adaptation scaled first-layer columns per feature (first_layer_feature_scales in adaptation_report.json) and used a Markov 35 phase-aligned dataset",
            extra={"transplant_report": C.rel(C.JUL_WARMUP / "actor_transplant_report.json"), "adaptation_report": C.rel(C.JUL_ADAPT_RUN / "adaptation_report.json")},
        ),
        entry(
            "JUL_BEST", "JUL_PILOT_checkpoint_best",
            actor_origin="JUL_H0 + 23 PPO updates (logical 24, pilot 15/07, lr 5e-7, exact-start, freeze_logstd)",
            critic_origin="JUL_H0 critic + 23 PPO updates",
            optimizer_origin="restored from JUL_H0 checkpoint_last then updated (weights_seq_no 24)",
            obs_schema="same 35-feature contract as JUL_H0",
            normalization="none at runtime (see JUL_H0)",
        ),
        entry(
            "JUL_LAST", "JUL_PILOT_checkpoint_last",
            actor_origin="JUL_H0 + 50 PPO updates (logical 51)",
            critic_origin="JUL_H0 critic + 50 PPO updates",
            optimizer_origin="restored from JUL_H0 then updated (weights_seq_no 51)",
            obs_schema="same 35-feature contract as JUL_H0",
            normalization="none at runtime (see JUL_H0)",
        ),
        entry(
            "V26_39D", "V26_IMIT_checkpoint_best",
            actor_origin="PPO imitation training from random init under V26 detector (june_equiv, 100 iter, best iter 87); 39-slice convention of the June lineage",
            critic_origin="trained jointly (imitation PPO, asymmetric 84-feature critic)",
            optimizer_origin="imitation training Adam (lr 1e-4, 10 epochs)",
            obs_schema="39 actor features = 35 deployable + gait_phase_sin/cos (prescribed clock, enabled) + 4 healthy imitation targets (probe bug: env emits 43, module consumes 39)",
            normalization="none at runtime",
        ),
        entry(
            "B0820_H0", "B0820_WARMUP_checkpoint_last",
            actor_origin=f"V26_39D actor (digest {b_report['source_actor_digest'][:12]}) transplanted 39->35 in drop mode: 4 imitation targets dropped, gait_phase_sin/cos columns zeroed (digest {b_report['target_actor_digest_after'][:12]}); NO supervised refit; frozen during 5 warmup iterations",
            critic_origin="fresh critic trained 5 warmup iterations on ex-novo reward (vf_loss 8.86 -> 0.07) under FSM v2 runtime (binary_phase_actor_fsm_version absent)",
            optimizer_origin="fresh Adam in warmup (actor frozen, log-std detached)",
            obs_schema="35 deployable features, same names/order as July (gait_phase_sin/cos zeroed; V26 heel-qualified binary events)",
            normalization="none (hard drop, no rescaling)",
            extra={"transplant_report": C.rel(C.B0820_WARMUP / "actor_transplant_report.json"), "signals_lost": {"source_only_features_dropped": b_report["source_only_features_dropped"], "shared_features_zeroed": b_report["shared_features_zeroed"]}},
        ),
        entry(
            "B0820_V2_BEST", "B0820_V2_checkpoint_best",
            actor_origin="B0820_H0 + 28 PPO updates (logical 33) under FSM v2 / pre-fix corridor",
            critic_origin="B0820_H0 critic + 28 PPO updates",
            optimizer_origin="restored from B0820 warmup checkpoint_last then updated (weights_seq_no 33)",
            obs_schema="same 35-feature contract as B0820_H0",
            normalization="none",
        ),
        entry(
            "B0820_V2_LAST", "B0820_V2_checkpoint_last",
            actor_origin="B0820_H0 + 50 PPO updates (logical 55) under FSM v2 / pre-fix corridor",
            critic_origin="B0820_H0 critic + 50 PPO updates",
            optimizer_origin="restored from B0820 warmup then updated (weights_seq_no 55)",
            obs_schema="same 35-feature contract as B0820_H0",
            normalization="none",
        ),
        entry(
            "B0820_V3_BEST", "B0820_V3_checkpoint_best",
            actor_origin="B0820_H0 + 34 PPO updates (logical 39) under FSM v3 / corrected corridor",
            critic_origin="B0820_H0 critic (warmed under v2 runtime) + 34 PPO updates under v3",
            optimizer_origin="restored from B0820 warmup then updated (weights_seq_no 39)",
            obs_schema="same 35-feature contract as B0820_H0",
            normalization="none",
        ),
        entry(
            "B0820_V3_LAST", "B0820_V3_checkpoint_last",
            actor_origin="B0820_H0 + 50 PPO updates (logical 55) under FSM v3 / corrected corridor",
            critic_origin="B0820_H0 critic + 50 PPO updates under v3",
            optimizer_origin="restored from B0820 warmup then updated (weights_seq_no 55)",
            obs_schema="same 35-feature contract as B0820_H0",
            normalization="none",
        ),
    ]
    return {
        "schema_version": 1,
        "generated_at_utc": C.utc_now(),
        "note": "Pesi actor, critic, optimizer, log_std, schema osservazioni, normalizzazione, runtime FSM e reward per ciascun checkpoint congelato. Il critic e l'optimizer sono letti dal full checkpoint (learner_group); gli rl_module esportati sono actor-only.",
        "june_baseline": {"module": modules["JUN23_IMIT_BEST"]["path"], "actor_digest_warm_start": modules["JUN23_IMIT_BEST"]["actor_digest_warm_start"], "first_layer_width": modules["JUN23_IMIT_BEST"]["first_layer_width"], "logstd": modules["JUN23_IMIT_BEST"]["logstd_head"]},
        "july_adapted_actor": {"module": modules["JUL_ADAPTED_TARGET_DOMAIN"]["path"], "actor_digest_warm_start": modules["JUL_ADAPTED_TARGET_DOMAIN"]["actor_digest_warm_start"], "adaptation": {k: adapt.get(k) for k in ("samples", "training_samples", "validation_samples", "epochs_run", "best_epoch", "best_validation_mse", "logstd_head_max_abs_parameter_change", "source_actor_digest", "adapted_actor_digest")}},
        "transplant_39_to_35": {k: b_report.get(k) for k in ("source_checkpoint", "removed_feature_mode", "critic_init_mode", "source_only_features_dropped", "shared_features_zeroed", "source_actor_digest", "target_actor_digest_after")},
        "candidates": rows,
    }


def main() -> int:
    C.ensure_out_dirs()
    out_json = C.OUT_MANIFEST / "freeze_inventory.json"
    out_md = C.OUT_MANIFEST / "freeze_inventory.md"
    prov_json = C.OUT_MANIFEST / "provenance_matrix.json"
    prov_md = C.OUT_MANIFEST / "provenance_matrix.md"
    for path in (out_json, out_md, prov_json, prov_md):
        if path.exists():
            raise FileExistsError(f"artefact already exists (no-clobber): {path}")

    inventory: dict[str, Any] = {
        "schema_version": 3,
        "revision": C.F0_REV,
        "superseded_revisions": C.SUPERSEDED_REVISIONS,
        "path_convention": "repository-relative POSIX paths",
        "protocol": "F0 freeze inventory (plan 2026-08-22, Fase 0, attivita 1-2)",
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "environment": C.env_snapshot(),
        "digest_conventions": {
            "actor_digest_warm_start": "warm_start.actor_state_digest (pi_encoder.* + pi.*, raw dtype bytes)",
            "actor_digest_pi_canonical": "compare_policy_checkpoints._actor_digest (keys starting with 'pi')",
            "critic_digest": "warm_start.compare_non_actor_states aggregate over vf* keys (learner module only)",
        },
        "modules": [],
        "full_checkpoints": [],
        "training_runs": [],
        "configs": [],
        "profiles": [],
        "digest_crosschecks": [],
    }

    module_specs: list[tuple[str, Path, int | None]] = [
        ("JUN23_IMIT_BEST", C.JUN23_RUN / "rl_module_best", None),
        ("JUL_PORT_INITIAL_WARM_START", C.JUL_PORT_RUN / "rl_module_initial_warm_start", 35),
        ("JUL_ADAPTED_TARGET_DOMAIN", C.JUL_ADAPT_RUN / "rl_module_target_adapted", 35),
        ("JUL_WARMUP_INITIAL_WARM_START", C.JUL_WARMUP / "rl_module_initial_warm_start", 35),
        ("JUL_WARMUP_BEST", C.JUL_WARMUP / "rl_module_best", 35),
        ("V26_IMIT_LAST", C.V26_IMIT_RUN / "rl_module_last", 39),
        ("B0820_WARMUP_INITIAL_WARM_START", C.B0820_WARMUP / "rl_module_initial_warm_start", 35),
        ("B0820_WARMUP_BEST", C.B0820_WARMUP / "rl_module_best", 35),
    ]
    for cid, spec in C.CANDIDATES.items():
        module_specs.append((cid, spec["module"], spec["width"]))
    for name, path, width in module_specs:
        print(f"[inventory] module {name}", flush=True)
        inventory["modules"].append(module_record(name, path, width_expected=width))

    ckpt_specs = [
        ("JUL_WARMUP_checkpoint_last", C.JUL_WARMUP / "checkpoint_last"),
        ("JUL_PILOT_checkpoint_best", C.JUL_PILOT_RUN / "checkpoint_best"),
        ("JUL_PILOT_checkpoint_last", C.JUL_PILOT_RUN / "checkpoint_last"),
        ("V26_IMIT_checkpoint_best", C.V26_IMIT_RUN / "checkpoint_best"),
        ("B0820_WARMUP_checkpoint_last", C.B0820_WARMUP / "checkpoint_last"),
        ("B0820_V2_checkpoint_best", C.B0820_V2_RUN / "checkpoint_best"),
        ("B0820_V2_checkpoint_last", C.B0820_V2_RUN / "checkpoint_last"),
        ("B0820_V3_checkpoint_best", C.B0820_V3_RUN / "checkpoint_best"),
        ("B0820_V3_checkpoint_last", C.B0820_V3_RUN / "checkpoint_last"),
    ]
    for name, path in ckpt_specs:
        print(f"[inventory] checkpoint {name}", flush=True)
        inventory["full_checkpoints"].append(checkpoint_record(name, path))

    for name, path in (
        ("JUN23_IMIT_RUN", C.JUN23_RUN),
        ("JUL_PORT_RUN", C.JUL_PORT_RUN),
        ("JUL_ADAPT_RUN", C.JUL_ADAPT_RUN),
        ("JUL_WARMUP", C.JUL_WARMUP),
        ("JUL_PILOT_RUN", C.JUL_PILOT_RUN),
        ("V26_IMIT_RUN", C.V26_IMIT_RUN),
        ("B0820_WARMUP", C.B0820_WARMUP),
        ("B0820_V2_RUN", C.B0820_V2_RUN),
        ("B0820_V3_RUN", C.B0820_V3_RUN),
    ):
        print(f"[inventory] run {name}", flush=True)
        inventory["training_runs"].append(run_record(name, path))

    for name, path in (
        ("canonical_training_exnovo_cfg (editable, worktree)", C.CANONICAL_CFG),
        ("chain_snapshot_exnovo_v26_B0820_chain", C.CHAIN_SNAPSHOT_CFG),
        ("july_imitation_training_cfg", C.JUL_IMIT_CFG),
        ("JUN23_resolved", C.JUN23_RUN / "training_cfg.resolved.yaml"),
        ("JUL_WARMUP_resolved", C.JUL_WARMUP / "training_cfg.resolved.yaml"),
        ("JUL_PILOT_resolved", C.JUL_PILOT_RUN / "training_cfg.resolved.yaml"),
        ("V26_IMIT_resolved", C.V26_IMIT_RUN / "training_cfg.resolved.yaml"),
        ("B0820_WARMUP_resolved", C.B0820_WARMUP / "training_cfg.resolved.yaml"),
        ("B0820_V2_resolved", C.B0820_V2_RUN / "training_cfg.resolved.yaml"),
        ("B0820_V3_resolved", C.B0820_V3_RUN / "training_cfg.resolved.yaml"),
    ):
        inventory["configs"].append(file_record(name, path))
    for name, path in (
        ("grf_profile_tangent_v2 (B0820/giugno)", C.GRF_PROFILE_TANGENT_V2),
        ("grf_profile_grf_correct_magnitude (luglio)", C.GRF_PROFILE_CORRECT_MAGNITUDE),
        ("grf_detector_profile_HS-TO", C.GRF_DETECTOR_PROFILE),
        ("binary_phase_detector_v25_selected_candidate_profile", C.BINARY_DETECTOR_PROFILE),
        ("morphology_profile_event_warped (B0820)", C.MORPH_PROFILE_EVENT_WARPED),
        ("morphology_profile_legacy_mean_std (luglio, peso 0)", C.MORPH_PROFILE_LEGACY),
        ("setup_xml_AB06_SEASEA", C.SETUP_XML),
    ):
        inventory["profiles"].append(file_record(name, path, load_json_keys=True))

    # --- digest cross-checks ---------------------------------------------
    modules = {m["name"]: m for m in inventory["modules"]}
    ckpts = {c["name"]: c for c in inventory["full_checkpoints"]}
    checks = inventory["digest_crosschecks"]

    def add_check(name: str, expected: Any, actual: Any, source: str) -> None:
        checks.append({"check": name, "expected": expected, "actual": actual, "source": source, "status": "PASS" if expected == actual and expected is not None else "FAIL"})

    def add_na(name: str, reason: str, source: str) -> None:
        checks.append({"check": name, "expected": None, "actual": None, "source": source, "status": NA, "reason": reason})

    b_report = C.read_json(C.B0820_WARMUP / "actor_transplant_report.json")
    j_report = C.read_json(C.JUL_WARMUP / "actor_transplant_report.json")
    b_summary = C.read_json(C.B0820_WARMUP / "summary.json")
    j_summary = C.read_json(C.JUL_WARMUP / "summary.json")
    adapt = C.read_json(C.JUL_ADAPT_RUN / "adaptation_report.json")
    drift = C.read_json(C.JUL_PILOT_RUN / "policy_drift_from_h0_milestones.json")

    add_check("B0820 transplant source_actor_digest == V26_39D actor digest", b_report["source_actor_digest"], modules["V26_39D"]["actor_digest_warm_start"], "actor_transplant_report.json (B0820)")
    add_check("B0820 transplant target_actor_digest_after == B0820_WARMUP_INITIAL_WARM_START", b_report["target_actor_digest_after"], modules["B0820_WARMUP_INITIAL_WARM_START"]["actor_digest_warm_start"], "actor_transplant_report.json (B0820)")
    add_check("B0820 warmup freeze audit (after iter 5) == B0820_H0 exported actor digest", b_summary["actor_freeze_audit"][-1]["actor_digest"], modules["B0820_H0"]["actor_digest_warm_start"], "summary.json actor_freeze_audit (B0820 warmup)")
    add_check("B0820 warmup critic audit (after iter 5) == critic digest embedded in B0820_WARMUP checkpoint_last (learner module)", b_summary["critic_state_audit"][-1]["critic_digest"], ckpts["B0820_WARMUP_checkpoint_last"]["embedded_learner_module"].get("critic_digest"), "summary.json critic_state_audit vs learner_group/learner/rl_module/default_policy")
    add_check("B0820 transplant learner_non_actor digest (fresh critic before warmup) == critic audit before_training", b_report["integration_validation"]["learner_non_actor"]["actual_digest"], b_summary["critic_state_audit"][0]["critic_digest"], "actor_transplant_report.json vs summary.json (B0820)")
    add_check("July transplant source == JUL_ADAPTED_TARGET_DOMAIN actor digest", j_report["source_actor_digest"], modules["JUL_ADAPTED_TARGET_DOMAIN"]["actor_digest_warm_start"], "actor_transplant_report.json (luglio)")
    add_check("July warmup freeze audit (after iter 1) == JUL_H0 exported actor digest", j_summary["actor_freeze_audit"][-1]["actor_digest"], modules["JUL_H0"]["actor_digest_warm_start"], "summary.json actor_freeze_audit (luglio)")
    add_check("July warmup critic audit (after iter 1) == critic digest embedded in JUL_WARMUP checkpoint_last (learner module)", j_summary["critic_state_audit"][-1]["critic_digest"], ckpts["JUL_WARMUP_checkpoint_last"]["embedded_learner_module"].get("critic_digest"), "summary.json critic_state_audit vs learner module (luglio)")
    add_check("July adaptation source_actor_digest == JUL_PORT_INITIAL_WARM_START", adapt["source_actor_digest"], modules["JUL_PORT_INITIAL_WARM_START"]["actor_digest_warm_start"], "adaptation_report.json (markov35 r32)")
    add_check("July adaptation adapted_actor_digest == JUL_ADAPTED_TARGET_DOMAIN", adapt["adapted_actor_digest"], modules["JUL_ADAPTED_TARGET_DOMAIN"]["actor_digest_warm_start"], "adaptation_report.json (markov35 r32)")
    add_check("July pilot drift audit reference_h0_actor_digest (pi-canonical convention) == JUL_H0 pi digest", drift["reference_h0_actor_digest"], modules["JUL_H0"]["actor_digest_pi_canonical"], "policy_drift_from_h0_milestones.json")
    v26_manifest = modules["V26_39D"].get("actor_feature_manifest") or {}
    add_check("V26_39D adjacent manifest actor_digest == computed", v26_manifest.get("declared_actor_digest"), modules["V26_39D"]["actor_digest_warm_start"], "rl_module_best/actor_feature_manifest.json (V26)")
    jul_manifest = modules["JUL_ADAPTED_TARGET_DOMAIN"].get("actor_feature_manifest") or {}
    add_check("JUL_ADAPTED adjacent manifest actor_digest == computed", jul_manifest.get("declared_actor_digest"), modules["JUL_ADAPTED_TARGET_DOMAIN"]["actor_digest_warm_start"], "actor_feature_manifest.json (markov35 r32)")
    add_check("GRF profile SHA tangent_v2 == grf_correct_magnitude", next(p["sha256"] for p in inventory["profiles"] if p["name"].startswith("grf_profile_tangent")), next(p["sha256"] for p in inventory["profiles"] if p["name"].startswith("grf_profile_grf_correct")), "online_grf_profiles/*.json")
    for ck_name, mod_name in (
        ("B0820_WARMUP_checkpoint_last", "B0820_H0"),
        ("JUL_WARMUP_checkpoint_last", "JUL_H0"),
        ("V26_IMIT_checkpoint_best", "V26_39D"),
        ("B0820_V3_checkpoint_best", "B0820_V3_BEST"),
        ("B0820_V3_checkpoint_last", "B0820_V3_LAST"),
        ("B0820_V2_checkpoint_best", "B0820_V2_BEST"),
        ("B0820_V2_checkpoint_last", "B0820_V2_LAST"),
        ("JUL_PILOT_checkpoint_best", "JUL_BEST"),
        ("JUL_PILOT_checkpoint_last", "JUL_LAST"),
    ):
        emb = ckpts[ck_name].get("embedded_learner_module") or {}
        add_check(f"{ck_name} learner-module actor == exported {mod_name} actor", emb.get("actor_digest_warm_start"), modules[mod_name]["actor_digest_warm_start"], "full checkpoint learner_group/learner/rl_module/default_policy vs rl_module export")
    for cid in C.CANDIDATES:
        add_na(f"critic digest on exported module {cid}", "exported rl_module_* are actor-only (state has no vf* keys); critic digest is taken from the learner module of the full checkpoint instead", modules[cid]["path"])
    add_na("env_runner module weights in full checkpoints", "env_runner/ holds only connector/runner state (state.pkl ~50 B); EnvRunner weights are synced from the learner at restore, not persisted separately", "full checkpoints")

    inventory["crosscheck_counts"] = {
        "PASS": sum(1 for c in checks if c["status"] == "PASS"),
        "FAIL": sum(1 for c in checks if c["status"] == "FAIL"),
        "N/A": sum(1 for c in checks if c["status"] == NA),
    }
    inventory["digest_crosschecks_no_fail"] = inventory["crosscheck_counts"]["FAIL"] == 0
    inventory["all_artifacts_exist"] = all(
        item["exists"] for group in ("modules", "full_checkpoints", "training_runs", "configs", "profiles") for item in inventory[group]
    )
    C.write_json(out_json, inventory)
    provenance = build_provenance_matrix(inventory)
    C.write_json(prov_json, provenance)

    # --- markdown -----------------------------------------------------------
    lines = [
        f"# F0 — Freeze inventory (digest e manifest) — revisione {C.F0_REV}",
        "",
        f"Generato: {inventory['generated_at_utc']} — git HEAD `{inventory['git']['head'][:12]}` ({inventory['git']['branch']}), worktree sporco: {len(inventory['git']['status_porcelain'])} voci (preservate).",
        "",
        "Convenzioni digest: `actor (ws)` = `warm_start.actor_state_digest`; `actor (pi)` = `compare_policy_checkpoints._actor_digest`; critic = aggregato su `vf*` del modulo learner nel full checkpoint (gli export `rl_module_*` sono actor-only).",
        "",
        "## Moduli actor esportati (rl_module)",
        "",
        C.md_table(
            ["modulo", "path", "width", "actor (ws)", "actor (pi)", "critic", "log-std rows zero", "log-std bias", "sigma (se costante)", "entropia nat (se costante)"],
            [
                [
                    m["name"], m["path"], m.get("first_layer_width"),
                    (m.get("actor_digest_warm_start") or "")[:16], (m.get("actor_digest_pi_canonical") or "")[:16],
                    "N/A (export actor-only)",
                    m["logstd_head"]["logstd_rows_all_zero"], [round(b, 4) for b in m["logstd_head"]["logstd_bias"]],
                    [round(s, 5) for s in m["logstd_head"]["sigma_if_rows_zero"]], round(m["logstd_head"]["entropy_nat_if_rows_zero"], 4),
                ]
                for m in inventory["modules"] if m["exists"]
            ],
        ),
        "",
        "## Full checkpoint (Algorithm: learner module actor+critic, optimizer)",
        "",
        C.md_table(
            ["checkpoint", "path", "file", "bytes", "tree digest", "logical iter", "actor (ws) learner", "critic digest", "weights_seq_no", "optimizer lr", "Adam step min/max"],
            [
                [
                    c["name"], c["path"], c.get("file_count"), c.get("total_bytes"), (c.get("tree_digest") or "")[:16], (c.get("meta_json") or {}).get("logical_iteration"),
                    (c["embedded_learner_module"].get("actor_digest_warm_start") or "")[:16], (c["embedded_learner_module"].get("critic_digest") or "")[:16],
                    c["learner_state"].get("weights_seq_no"),
                    next(iter((c["learner_state"].get("optimizers") or {}).values()), {}).get("lr"),
                    next(iter((c["learner_state"].get("optimizers") or {}).values()), {}).get("adam_step_min_max"),
                ]
                for c in inventory["full_checkpoints"] if c["exists"]
            ],
        ),
        "",
        "## Config e profili",
        "",
        C.md_table(
            ["artefatto", "path", "sha256", "git blob", "tracked"],
            [[f["name"], f["path"], (f.get("sha256") or "")[:16], (f.get("git_blob") or "")[:12], f.get("git_tracked")] for f in inventory["configs"] + inventory["profiles"] if f["exists"]],
        ),
        "",
        "## Cross-check dei digest dichiarati",
        "",
        C.md_table(["check", "atteso", "calcolato", "sorgente", "esito / motivo"], [[c["check"], (c["expected"] or "-")[:16] if c["expected"] else "-", (c["actual"] or "-")[:16] if c["actual"] else "-", c["source"], c["status"] + (f": {c['reason']}" if c.get("reason") else "")] for c in checks]),
        "",
        f"Tutti gli artefatti risolvibili: **{inventory['all_artifacts_exist']}** — cross-check: PASS {inventory['crosscheck_counts']['PASS']}, FAIL {inventory['crosscheck_counts']['FAIL']}, N/A {inventory['crosscheck_counts']['N/A']} — nessun FAIL: **{inventory['digest_crosschecks_no_fail']}**.",
        "",
    ]
    C.write_text(out_md, "\n".join(lines))

    plines = [
        f"# F0 — Matrice di provenienza — revisione {C.F0_REV}",
        "",
        provenance["note"],
        "",
        "## Radici",
        "",
        f"- Baseline imitativa 24/06 (`{provenance['june_baseline']['module']}`): actor digest `{provenance['june_baseline']['actor_digest_warm_start'][:16]}`, width {provenance['june_baseline']['first_layer_width']} (convenzione 31-slice), log-std state-dependent.",
        f"- Actor adattato target-domain luglio (`{provenance['july_adapted_actor']['module']}`): digest `{provenance['july_adapted_actor']['actor_digest_warm_start'][:16]}`; adattamento: {provenance['july_adapted_actor']['adaptation']}.",
        f"- Trapianto 39->35 B0820: {provenance['transplant_39_to_35']}.",
        "",
        "## Candidati",
        "",
        C.md_table(
            ["candidato", "actor: origine", "actor (ws)", "critic: origine", "critic digest", "optimizer: origine", "weights_seq_no", "log_std", "schema oss.", "normalizzazione", "runtime FSM", "reward/penetr./swing"],
            [
                [
                    r["candidate"], r["actor_weights"]["origin"], r["actor_weights"]["digest_warm_start"][:16], r["critic"]["origin"], (r["critic"]["digest"] or "n/a")[:16],
                    r["optimizer"]["origin"], r["optimizer"]["weights_seq_no"],
                    f"{r['log_std']['regime'].split(':')[0]}; bias {[round(b, 3) for b in r['log_std']['bias']]}; sigma(const) {[round(s, 4) for s in r['log_std']['sigma_if_constant']]}",
                    f"{r['observation_schema']['actor_width']}D: {r['observation_schema']['description']}", r["normalization"],
                    f"mode={r['runtime']['binary_phase_fsm_mode']}, fsm={r['runtime']['binary_phase_actor_fsm_version']}, invalid={r['runtime']['binary_phase_invalid_event_policy']}, grf={Path(str(r['runtime']['grf_profile'])).name}",
                    f"{r['runtime']['reward_mode']}, penetr {r['runtime']['penetration_m']}, swing {r['runtime']['swing_hard_timeout_s']} s, morph {r['runtime']['morphology']}",
                ]
                for r in provenance["candidates"]
            ],
        ),
        "",
    ]
    C.write_text(prov_md, "\n".join(plines))
    print(f"[inventory] written {out_json}, {out_md}, {prov_json}, {prov_md}")
    print(f"[inventory] all_artifacts_exist={inventory['all_artifacts_exist']} crosschecks={inventory['crosscheck_counts']} no_fail={inventory['digest_crosschecks_no_fail']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
