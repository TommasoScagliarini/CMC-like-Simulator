# Validazione smoke timeout GRF/fase

## Problema

Dopo l'introduzione della nuova guardia `phase_timeout` serviva verificare che
il caso patologico gia osservato nel rollout post-training venisse realmente
fermato:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best
```

Il comportamento da validare era:

- perdita del contatto protesico dopo il primo `TO`;
- nessun secondo `HS` protesico;
- crescita della penalita timing GRF dopo il soft bound;
- terminazione hard prima del limite episodio.

## Soluzione

E' stato rilanciato il checkpoint gia noto come problematico usando la config
ex-novo aggiornata, forzando esplicitamente:

```text
--config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"
```

Questo evita di usare solo lo snapshot vecchio del training, che non conteneva i
nuovi campi:

```yaml
phase_timeout_penalty_weight: 0.10
phase_stance_hard_timeout_s: 2.20
phase_swing_hard_timeout_s: 1.30
```

Comando smoke usato:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  --checkpoint "Trajectory Generator/runs/training/MLP_ExNovo_training_06-29-2026/rl_module_best" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --output-dir "runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke" \
  --record-outputs
```

Prima e' stato fatto anche un pass leggero con `--no-record-outputs`; poi e'
stato ripetuto con `--record-outputs` per salvare gli `.sto` necessari ai plot.

## Strategia

La validazione e' stata fatta in tre passaggi:

1. eseguire il rollout deterministico del checkpoint patologico con la reward
   aggiornata;
2. controllare `rollout_summary.json` e `rollout_policy_trace.json` per
   `terminated`, `truncated`, `end_reason`, `phase_timeout_loss` e
   `phase_timeout_exceeded`;
3. generare plot diagnostici da trace e `.sto` per GRF online, eventi HS/TO,
   azioni policy, cinematiche protesiche, recruitment e SEA.

## Risultato

La guardia timing GRF/fase e' stata validata.

Risultato principale:

```text
steps: 142
episode_return: 29.421314
terminated: True
truncated: False
end_reason: phase_timeout:swing
termination_time_s: 14.410
```

Il timing osservato e' coerente con i parametri configurati:

```text
left_TO_time_s: 13.100
soft_swing_expected_s: 14.000
hard_swing_expected_s: 14.400
termination_time_s: 14.410
```

Quindi:

- la `phase_timeout_loss` inizia a crescere dopo il soft bound swing;
- la terminazione scatta al superamento del bound hard swing;
- il rollout non arriva piu al limite episodio con `truncated=True`;
- il caso di missing contact protesico viene fermato con
  `phase_timeout:swing`.

## Diagnostica aggiuntiva

Metriche principali dello smoke:

```text
left_contact_fraction: 0.064085
right_contact_fraction: 0.788028
left_events: HS=1, TO=1
right_events: HS=1, TO=1
raw_knee_action_min: -2.046561
raw_knee_action_mean: -1.525197
applied_knee_action_min: -1.000000
knee_action_clip_fraction_steps: 0.859155
knee_angle_min_rad: -1.517374
knee_angle_last_rad: -1.500372
ankle_angle_mean_rad: 0.159991
ankle_angle_last_rad: 0.100890
tau_reserve_norm_max_nm: 842.873
tau_reserve_norm_last_nm: 734.643
```

La guardia risolve il problema della mancata terminazione, ma conferma che il
checkpoint resta patologico: il canale knee continua a saturare verso il bound
inferiore e il recruitment richiede reserve elevate.

## File modificati o generati

File di codice/configurazione:

- nessun nuovo file di codice modificato durante questa sola fase di smoke;
- la validazione usa le modifiche gia introdotte in
  `Trajectory Generator/baseline_MLP/reward_function.py`;
- la validazione usa i parametri aggiornati in
  `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`.

Output generati:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/rollout_summary.json`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/rollout_policy_trace.json`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/sim_outputs/`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/timeout_smoke_report.md`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/timeout_smoke_summary.json`.

Plot generati:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/01_timeout_guard_reward.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/02_online_grf_events.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/03_actions_kinematics.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_timeout_smoke/plots/04_recruitment_sea.png`.

Report creato:

- `reports/user/2026-06-30_validazione_smoke_timeout_grf_phase.md`.

## Test e verifiche

Verifiche eseguite:

- rollout deterministico leggero con `--no-record-outputs`: completato con
  `terminated=True`, `truncated=False`, `end_reason=phase_timeout:swing`;
- rollout deterministico con `--record-outputs`: stesso esito e output `.sto`
  salvati;
- lettura di `rollout_summary.json` e `rollout_policy_trace.json`;
- generazione plot diagnostici in PNG;
- verifica dimensioni PNG con `file`;
- ispezione visiva del plot principale `01_timeout_guard_reward.png`.

Nota: Matplotlib ha usato una cache temporanea perche' `~/.matplotlib` non era
scrivibile nel sandbox; i file PNG sono stati comunque generati correttamente.

## Conclusione

La penalty/guardia timing GRF e' validata in inference smoke. Il prossimo passo
non e' piu correggere il timeout, ma verificare se un mini training ex-novo
impara a evitare la terminazione oppure collassa ripetutamente su
`phase_timeout:swing`.

## TODO

- Lanciare un mini training ex-novo diagnostico con la reward attuale congelata,
  senza ulteriori modifiche concorrenti.
- Monitorare durante il mini training `phase_timeout_loss`,
  `phase_timeout_exceeded`, `end_reason`, contact fraction protesica ed eventi
  `HS/TO`.
- Se il training evita il timeout ma mantiene knee saturo, valutare
  `policy_action_clip_weight > 0`.
- Se il training termina spesso per timeout, aumentare il gradiente positivo
  verso recupero contatto/eventi prima di aggiungere altri termini reward.
