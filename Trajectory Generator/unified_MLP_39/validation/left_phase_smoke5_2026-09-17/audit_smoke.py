"""Audit this bounded five-iteration run; never starts or resumes training."""
from pathlib import Path
import hashlib
import json
import math
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from unified_MLP_39._bootstrap import ensure_sim_paths

ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: E402,F401
import numpy as np  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402
from unified_MLP_39 import actor_transition  # noqa: E402


def read(path):
    return json.loads(path.read_text())


def main():
    run = PACKAGE / "runs/training/left_phase_visible_smoke5_2026-09-17"
    summary = read(run / "summary.json")
    assert summary["ok"] and summary["stop_reason"] == "completed"
    assert summary["iterations_completed"] == 5 and summary["initialization_mode"] == "fresh"
    assert not summary["skipped_iterations"] and not summary["error"]
    assert not summary["crash_restart_count"] and not summary["restart_count"]
    rows = [json.loads(line) for line in (run / "train_iterations.jsonl").read_text().splitlines() if line]
    assert [row["iteration"] for row in rows] == list(range(1, 6))
    manifest = read(HERE / "source_hashes_before.json")
    assert all(hashlib.sha256((REPO / name).read_bytes()).hexdigest() == digest for name, digest in manifest.items())
    previous = None
    results = []
    for row in rows:
        i = row["iteration"]
        audit = row["unified_phase_audit"]
        batch = audit["target_availability_batch"]
        assert row["unified_phase"] == "imitation" and row["exact_start_balance"]["pass"]
        assert row["num_env_steps_sampled_lifetime"] == i * 192
        assert audit["mean_changed"] and audit["critic_changed"]
        assert batch["matches_expected_train_batch_size"]
        assert batch["total_steps"] == batch["visible_steps"] == 192 and batch["visible_fraction"] == 1.0
        assert row["kl_nonfinite_count"] == 0
        for key in ("policy_loss", "vf_loss", "entropy", "mean_kl_loss", "min_minibatch_mean_kl_loss", "max_minibatch_mean_kl_loss"):
            assert math.isfinite(row[key]), (i, key)
        health = row["training_health"]
        assert health["observed_rows"] == 192 and health["missing_telemetry_rows"] == 0
        assert health["morphology_causal_contract_failure_rows"] == 0
        if previous:
            assert audit["actor_before"] == previous["actor_after"]
            assert audit["critic_before"] == previous["critic_after"]
        milestone = run / f"milestone_iteration_{i:06d}"
        state = read(milestone / "checkpoint_last/unified_training_state.json")
        assert state["schema_version"] == "unified_mlp39_v2"
        assert state["last_completed_iteration"] == i
        assert state["target_availability"]["configuration"]["probability"] == 1.0
        assert state["target_availability"]["configuration"]["curriculum_probabilities"] == []
        for worker in ("1", "2", "3"):
            counters = state["target_availability"]["runner_rng_states"][worker][0]
            assert counters["total_steps"] == counters["visible_steps"] == i * 64
        export = milestone / "rl_module_last"
        contract = actor_transition.read_contract(export)
        assert contract["target_channels"]["teacher_source"] == "prescribed_left_phase_template"
        assert contract["target_channels"]["phase_shift_cycles"] == 0.0
        module = RLModule.from_checkpoint(str(export))
        actor_transition.validate_module_contract(module, contract)
        assert actor_transition.actor_state_digest(module.get_state()) == audit["actor_after"]
        for tensor in actor_transition.actor_state(module.get_state()).values():
            assert np.isfinite(np.asarray(tensor)).all()
        results.append({"iteration": i, "return_mean": row["episode_return_mean"],
                        "episode_length_steps_mean": row["episode_len_mean"],
                        "mean_kl": row["mean_kl_loss"], "vf_loss": row["vf_loss"],
                        "target_visibility": batch["visible_fraction"],
                        "actor_and_critic_updated": True, "checkpoint_reloaded_exact": True,
                        "training_health": health, "termination_metrics": row["termination_metrics"]})
        previous = audit
    result = {"pass": True, "run": str(run), "iterations": 5, "transitions": 960,
              "transitions_per_start": 320, "elapsed_wall_time_s": summary["elapsed_wall_time_s"],
              "crash_restarts": 0, "skipped_iterations": [], "missing_telemetry_rows": 0,
              "source_hashes_unchanged": True, "export_reload_exact_count": 5,
              "results": results, "scope": "operational smoke, not gait quality or V26 parity"}
    lines = [
        "# Smoke 39D sulla fase sinistra — 5 iterazioni", "",
        "**PASS operativo su macOS arm64.** Cinque aggiornamenti PPO completati da pesi nuovi, "
        "senza restart del processo o iterazioni saltate. Nessuna modifica al runtime durante il run.", "",
        "Configurazione: 3 worker, batch 192 (64 transizioni per start), minibatch 64, "
        "1 epoca PPO, seed 123. Target sulla fase prescribed sinistra con shift zero; "
        "quattro target visibili al 100%, curriculum vuoto. Totale: 960 transizioni, 320 per start.", "",
        f"Durata del run: {summary['elapsed_wall_time_s']:.1f} s. "
        "I cinque checkpoint esportati sono stati ricostruiti in un processo di verifica separato, "
        "con digest actor identici a quelli registrati dopo ciascun aggiornamento.", "",
        "| Iterazione | Return medio riportato | Lunghezza media riportata (step) | KL medio | Visibilità |",
        "|---:|---:|---:|---:|---:|",
    ]
    for item in results:
        ret = item['return_mean']
        length = item['episode_length_steps_mean']
        ret_text = "N/D" if ret is None else f"{ret:.4f}"
        length_text = "N/D" if length is None else f"{length:.2f}"
        lines.append(f"| {item['iteration']} | {ret_text} | {length_text} | {item['mean_kl']:.6f} | 100% |")
    lines += ["", "Le medie sono quelle riportate da RLlib e non costituiscono repliche indipendenti. "
              "N/D indica una metrica episodica non riportata in quel batch; non viene sostituita con zero. "
              "Sono presenti terminazioni degli episodi per penetrazione: il PASS riguarda il funzionamento "
              "software, non il raggiungimento di un cammino stabile o della parità con V26.", "",
              "Verificati in ogni iterazione: bilanciamento dei tre start, aggiornamento di actor e critic, "
              "continuità dei digest fra aggiornamenti, loss/KL finiti, telemetria completa, maschera "
              "interamente visibile, contatori cumulativi per worker e schema checkpoint v2. "
              "Verificata l'invarianza degli hash delle sorgenti e della configurazione fotografate prima del run.", "",
              "Questo smoke mantiene p=1 per tutte le iterazioni; il cambio di maschera e la ripresa PPO "
              "sono stati verificati separatamente nell'implementazione precedente. Windows non è stato eseguito.", "",
              "- [Verifica dettagliata](verification.json)",
              "- [Configurazione esatta](smoke5.yaml)",
              "- [Log del training](training.log)",
              "- [Manifest iniziale delle sorgenti](source_hashes_before.json)",
              "- [Summary del run](../../runs/training/left_phase_visible_smoke5_2026-09-17/summary.json)",
              "- [Checkpoint finale](../../runs/training/left_phase_visible_smoke5_2026-09-17/milestone_iteration_000005)",
              "- [Script di verifica](audit_smoke.py)", ""]
    (HERE / "README.md").write_text("\n".join(lines), encoding="utf-8")
    (HERE / "verification.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
