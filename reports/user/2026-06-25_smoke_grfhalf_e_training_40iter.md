# Smoke GRF half e preparazione training 40 iter

Data: 2026-06-25

## Problema

Si voleva ridurre il peso delle penalita GRF per cercare uno sweet spot:

- mantenere coppie `tau_spring_ankle` piu plausibili;
- evitare che il guard GRF peggiori troppo il tracking del knee in stance;
- ripristinare il jerk knee a `3000 rad/s^3`.

La proposta da testare era:

```yaml
grf_penetration_weight: 0.5
grf_ankle_moment_flip_weight: 0.05
pros_knee_ref_jerk_limit_rad_s3: 3000.0
```

Durante il tentativo di lanciare un training da terminale e comparso anche un
errore operativo:

```text
ModuleNotFoundError: No module named 'ray'
```

Il training era stato lanciato dall'ambiente `envCMC-like`, mentre PPO/RLlib
richiede `envCMC-rllib`.

## Strategia

1. Verificare dai rollout storici se la penalizzazione GRF agisce direttamente
   nella finestra di stance critica oppure cambia la strategia appresa.
2. Lanciare uno smoke training con i pesi dimezzati `0.5 / 0.05`.
3. Fare rollout dal checkpoint migliore dello smoke.
4. Confrontare metriche globali e stance critica contro i run precedenti:
   `penetration_only_100`, `hard_flip_5/0.25`, `soft_flip_1/0.10`,
   `jerk2000_soft`.
5. Preparare la configurazione per un training piu informativo da 40 iterazioni.

## Soluzione applicata

La configurazione corrente in `Trajectory Generator/baseline_MLP/training_cfg.yaml`
e stata portata a:

```yaml
simulation:
  iterations: 40
  pros_knee_ref_jerk_limit_rad_s3: 3000.0

reward:
  grf_penetration_weight: 0.5
  grf_ankle_moment_flip_weight: 0.05
```

Il comando consigliato per evitare il problema `ray` e:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --iterations 40 \
  --output-dir "runs/training/MLP_imitation_training_06-25-2026_grfhalf_40iter"
```

In alternativa:

```bash
conda activate envCMC-rllib
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --iterations 40 \
  --output-dir "runs/training/MLP_imitation_training_06-25-2026_grfhalf_40iter"
```

## Risultati dello smoke

Training smoke:

- output: `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_smoke_10iter`
- pianificato: 10 iterazioni;
- completate: 9 iterazioni;
- checkpoint migliore: iterazione 9;
- best train return: `56.355`;
- best episode length mean: `382.0`;
- il training non ha scritto `summary.json`, ma ha scritto `checkpoint_best`,
  `checkpoint_last`, `rl_module_best` e `rl_module_last`.

Rollout dal best checkpoint:

- output: `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-25-2026_grfhalf_smoke_9iter_best`
- rollout return: `146.98`;
- reward mean: `0.293`;
- action clip fraction: `0.0`;
- terminazione: `episode_time_limit`.

Confronto diagnostico principale:

| Run | Return rollout | Knee RMSE target-served | Left Fy mean | Ankle tau mean | Root reserve mean |
|---|---:|---:|---:|---:|---:|
| penetration_only_100 | 370.1 | 0.0988 | 116.1 N | +0.33 Nm | 328.4 |
| hard_flip_5/0.25 | 333.7 | 0.1825 | 27.5 N | -0.87 Nm | 412.1 |
| soft_flip_1/0.10 | 348.5 | 0.1366 | 60.1 N | -2.17 Nm | 381.2 |
| jerk2000_soft | 334.8 | 0.1812 | 41.5 N | -1.31 Nm | 399.3 |
| grfhalf_smoke_9iter | 147.0 | 0.3618 | 5.4 N | +0.30 Nm | 432.5 |

Nella finestra critica `phase 0.15-0.40` con `Fy > 50 N`, il rollout
`grfhalf_smoke_9iter` ha `n=0` campioni validi. Questo e il punto decisivo:
lo smoke non ha prodotto una stance caricata utile, quindi non puo verificare
lo sweet spot knee/ankle.

## Interpretazione critica

Lo smoke test verifica solo che la pipeline con `0.5 / 0.05` parte, allena,
salva checkpoint e fa rollout senza crash. Non verifica la qualita biomeccanica
del set di pesi.

Il checkpoint a 9 iterazioni e troppo acerbo:

- il carico verticale sinistro medio e quasi nullo (`5.4 N`);
- non ci sono campioni nella stance critica caricata;
- la `tau_spring_ankle` non mostra ancora il beneficio cercato;
- il tracking knee globale e molto peggiore dei run maturi;
- i reserve restano alti in molte finestre del rollout.

Quindi la proposta `0.5 / 0.05` resta plausibile, ma non e confermata dallo
smoke. Serve un training piu maturo, almeno 40 iterazioni, per fare un controllo
intermedio attendibile.

## File modificati

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - `iterations: 40`
  - `pros_knee_ref_jerk_limit_rad_s3: 3000.0`
  - `grf_penetration_weight: 0.5`
  - `grf_ankle_moment_flip_weight: 0.05`
- `reports/user/2026-06-25_smoke_grfhalf_e_training_40iter.md`
  - nuovo report utente.

Nessuna modifica al plugin C++ o alla semantica del comando SEA.

## Artefatti generati

- `Trajectory Generator/runs/analysis/grf_knee_stance_diagnostics_2026-06-25/`
  - analisi stance/GRF/knee precedente;
  - plot `critical_phase_tradeoff.png`, `critical_phase_deltas.png`,
    `training_stability_summary.png`.
- `Trajectory Generator/runs/training/MLP_imitation_training_06-25-2026_grfhalf_smoke_10iter/`
  - training smoke, checkpoint best/last a iterazione 9.
- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-25-2026_grfhalf_smoke_9iter_best/`
  - rollout registrato dal best checkpoint dello smoke.
- `Trajectory Generator/runs/analysis/grfhalf_smoke_2026-06-25/smoke_comparison_metrics.csv`
  - confronto numerico tra smoke e rollout maturi.

## Test e verifiche

- Verificato che `envCMC-rllib` importa `ray 2.55.1`.
- Verificato che `training_cfg.yaml` contiene `iterations: 40`, jerk knee
  `3000.0`, GRF `0.5 / 0.05`.
- Eseguito `git diff --check` su `training_cfg.yaml`: nessun errore.
- Eseguito smoke training con `envCMC-rllib`.
- Eseguito rollout dal checkpoint migliore dello smoke.
- Eseguita analisi comparativa su metriche globali e stance critica.

## TODO

- Lanciare il training da 40 iterazioni con `envCMC-rllib`:

  ```bash
  /opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
    --iterations 40 \
    --output-dir "runs/training/MLP_imitation_training_06-25-2026_grfhalf_40iter"
  ```

- Al termine, fare rollout dal `rl_module_best`.
- Ripetere il confronto nella finestra `phase 0.15-0.40`, `Fy > 50 N`.
- Criteri minimi per promuovere il set `0.5 / 0.05`:
  - stance critica con campioni validi;
  - knee RMSE target-served sotto il soft `1.0 / 0.10`;
  - `tau_spring_ankle` ancora negativa in early/mid stance;
  - root reserve non peggiore del soft maturo.
