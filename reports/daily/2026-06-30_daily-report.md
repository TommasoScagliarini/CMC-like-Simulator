# Daily report - 2026-06-30

## Sintesi

Il lavoro del 30/6 si e' concentrato sulla fase GRF/eventi della reward
ex-novo. Il punto centrale era trasformare il problema:

```text
HS protesico iniziale -> TO protesico -> nessun nuovo HS
```

da semplice failure mode osservato a meccanismo esplicito di training e
diagnostica.

Sono stati completati tre passaggi:

1. validazione della guardia `phase_timeout:swing`;
2. progettazione e implementazione della FSM protesica `HS -> TO -> HS`
   dentro l'env;
3. training/rollout diagnostici, inclusi uno smoke test e un run full-batch da
   100 iterazioni.

Risultato finale: la FSM e la terminazione funzionano, ma la policy non ha
ancora imparato a produrre un ciclo completo `HS -> TO -> HS`. Dopo 100
iterazioni con batch 4096 il rollout termina ancora per `phase_timeout:swing`.

## Report utente consolidati

Report del 30/6 inclusi in questo daily:

- `reports/user/2026-06-30_rollout_post_training_exnovo_morphology.md`
- `reports/user/2026-06-30_timeout_grf_phase_penalty_termination.md`
- `reports/user/2026-06-30_validazione_smoke_timeout_grf_phase.md`
- `reports/user/2026-06-30_training_exnovo_timeout_20iter_failure_mode.md`
- `reports/user/2026-06-30_proposta_macchina_fase_protesica_hs_to_hs.md`
- `reports/user/2026-06-30_implementazione_fsm_protesica_hs_to_hs.md`
- `reports/user/2026-06-30_specifica_corridor_band_reward_morphology.md`
- `reports/user/2026-06-30_training_smoke_fsm_exnovo_10iter_rollout.md`
- `reports/user/2026-06-30_training_fullbatch_fsm_exnovo_100iter_rollout.md`

## Diagnosi iniziale

Il rollout post-training ex-novo del checkpoint del 29/6 mostrava:

```text
terminated: false
truncated: true
left contact fraction molto bassa
left events: HS=1, TO=1
morphology_phase: bloccata a 0.0
knee action: spesso clippata/satura
```

Conclusione: il problema principale non era il low-level SEA, ma il riferimento
protesico prodotto dalla policy. La policy riusciva a ottenere return positivo
senza recuperare un nuovo contatto protesico valido.

## Timeout GRF/fase

E' stata introdotta e validata una guardia hard sul timeout della fase:

```text
phase_stance_hard_timeout_s: 2.20
phase_swing_hard_timeout_s: 1.30
phase_timeout_penalty_weight: 0.10
```

Il vecchio checkpoint patologico, rieseguito con la config aggiornata, termina
correttamente:

```text
end_reason: phase_timeout:swing
terminated: true
truncated: false
```

Questa validazione chiude il problema "l'episodio continua senza secondo HS".
Non chiude invece il problema di apprendimento: il timeout da solo ferma il caso
patologico, ma non insegna alla policy a generare il nuovo HS.

## Training timeout 20 iter

E' stato eseguito un training diagnostico da 20 iterazioni con la reward
aggiornata dal timeout.

Risultato:

```text
iterations_completed: 20
best_episode_return_mean: 19.987309
last_episode_len_mean: 142.064
last_phase_timeout_swing_count: 443
```

Il rollout del best checkpoint mostrava ancora:

```text
HS=1
TO=1
nuovo HS=0
end_reason=phase_timeout:swing
```

Conclusione: il solo timeout produce un segnale insufficiente. Serve un credito
positivo di progressione evento e un termine denso per il recupero contatto
nella landing window.

## FSM protesica HS-TO-HS

E' stata implementata una FSM runtime nell'env, non nella reward wrapper.

Stati:

```text
WAIT_HS = 0
STANCE_AFTER_HS = 1
SWING_AFTER_TO = 2
VALID_CYCLE_COMPLETED = 3
TIMEOUT = 4
INVALID_EVENT = 5
```

La FSM usa solo segnali deployable:

- angolo protesico knee;
- angolo protesico ankle;
- online GRF lato left in modalita detector.

La FSM viene esposta:

- nell'observation actor;
- in `info["phase_fsm"]`;
- nei `reward_terms` consumati dalla reward wrapper.

La reward wrapper non mantiene una seconda macchina stati.

## Campi observation/reward

Sono stati aggiunti campi FSM per actor observation e logging:

```text
phase_fsm_wait_hs
phase_fsm_stance_after_hs
phase_fsm_swing_after_to
phase_expected_hs
phase_expected_to
phase_stance_elapsed_norm
phase_swing_elapsed_norm
phase_cycle_progress_credit
```

Nel blocco reward della config ex-novo e' stata aggiunta la sezione 4.1:

```yaml
# 4.1) Prosthetic phase FSM / landing window.
phase_min_stance_duration_s: 0.05
phase_min_swing_duration_s: 0.20
phase_landing_window_start_s: 0.55  # Near nominal swing (~0.60 s): start rewarding landing.
phase_landing_window_end_s: 1.10  # Wide window: after soft timeout, before hard timeout.

blend_phase_event_progress: 0.20
blend_landing_window_contact: 0.15
phase_invalid_event_weight: 0.10
phase_contact_validity_weight: 0.10
```

Semantica:

- `phase_min_*`: rifiuta eventi troppo precoci;
- `phase_landing_window_*`: definisce il reward denso swing -> landing;
- `phase_*timeout*`: punisce/termina eventi troppo tardivi;
- `phase_period*` e `phase_stance_fraction*`: valutano plausibilita' del ciclo
  completo.

## Reward FSM

Progressione evento:

```text
WAIT_HS: 0.00
dopo valid_HS: 0.25
dopo valid_TO: 0.50
sullo step che chiude HS -> TO -> HS: 1.00
```

Landing window:

```text
attiva solo in SWING_AFTER_TO
attiva tra 0.55 s e 1.10 s dopo TO
premia ricomparsa progressiva di GRF valida
non premia oltre la finestra
timeout hard resta terminale
```

Eventi invalidi:

```text
doppio HS prima di TO
doppio TO prima di HS
TO troppo precoce
HS troppo precoce
```

Gli invalid event generano penalita e diagnostica, ma non terminano l'episodio
di default.

## Validazione FSM

La FSM e' stata verificata a piu livelli:

- test unitari FSM pura;
- test reward sui termini FSM;
- test config dei nuovi campi YAML;
- smoke env observation/info;
- test CMC-like con dati prescribed;
- rollout diagnostico del vecchio checkpoint patologico.

Conclusione: la FSM e' validata come infrastruttura runtime/diagnostica. Non e'
ancora validata come soluzione comportamentale del training, perche' la policy
non produce ancora cicli completi.

## Training smoke FSM 10 iter

E' stato eseguito uno smoke test leggero da 10 iterazioni con batch ridotto per
controllare cablaggio e runtime.

Risultato:

```text
iterations_completed: 10
best_episode_return_mean: 12.2647
best checkpoint logical iteration: 8
sampled steps lifetime: 2560
```

Rollout:

```text
HS=1
TO=1
cycle=0
phase_cycle_progress_credit=0.5
end_reason=phase_timeout:swing
```

Lo smoke ha validato la pipeline, ma non poteva validare apprendimento robusto.

## Training full-batch 100 iter

E' stato poi eseguito il run piu importante della giornata:

```text
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter
```

Parametri:

```text
iterations: 100
train_batch_size: 4096
checkpoint_every: 10
morphology_weight: 0.0
```

Esito:

```text
ok: true
stop_reason: completed
iterations_completed: 100
sampled steps: 409600
elapsed_wall_time_s: 46370.78
best_episode_return_mean: 40.951685
best checkpoint logical_iteration: 99
last_episode_return_mean: 40.937618
last_episode_len_mean: 141.307692
```

Il training e' partito il 2026-06-30 alle 18:47:38 e si e' concluso il
2026-07-01 alle 11:27:31, ma viene riportato in questo daily perche' richiesto
come chiusura del lavoro del 30/6.

Trend:

```text
iter 1:   return=-6.037,  len=71.117
iter 10:  return=25.008,  len=142.308
iter 20:  return=29.882,  len=142.000
iter 50:  return=38.026,  len=142.333
iter 80:  return=39.772,  len=141.769
iter 99:  return=40.952,  len=140.974
iter 100: return=40.938,  len=141.308
```

Il return cresce, ma la durata episodio resta bloccata nel regime del timeout.

## Rollout 100 iter

Rollout del best checkpoint:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout
```

Risultato:

```text
steps: 140
episode_return: 43.474741
reward_mean: 0.310534
terminated: true
truncated: false
end_reason: phase_timeout:swing
```

FSM:

```text
valid_HS_count: 1
valid_TO_count: 1
valid_cycle_count: 0
invalid_event_count: 0
cycle_progress_credit: 0.5
phase_event_progress_score: 0.5
```

Landing window:

```text
active steps: 55
swing_elapsed range: 0.553 -> 1.093 s
landing_window_contact_score: sempre 0.0
```

Azioni:

```text
raw action abs max: 2.873095
applied action abs max: 1.000000
action_clipped_steps: 140 / 140
action_clipped_fraction: 0.567857
```

Conclusione: il training full-batch da 100 iterazioni riduce la probabilita' che
il problema sia solo batch size o durata breve. La policy migliora il return
restando nello stesso attrattore:

```text
HS valido -> TO valido -> swing prolungato -> phase_timeout:swing
```

## Morphology corridor

E' stata fissata la semantica futura della morphology come corridor band reward,
non tracking:

```text
hard_min        soft_min        inner_min        inner_max        soft_max        hard_max
   |--------------|---------------|================|---------------|--------------|
 terminate/high   mild loss          flat reward       mild loss      high/terminate
```

Regola centrale:

```text
per ogni q dentro [inner_min, inner_max]:
  stesso identico reward
```

Quindi:

- nessun gradiente verso la media AB06;
- nessun incentivo a stare al centro;
- nessuna imitazione mascherata.

Decisione: morphology resta spenta (`morphology_weight: 0.0`) finche' non
compaiono cicli `HS -> TO -> HS` affidabili e una `morphology_phase` corretta.

## File principali modificati

Modifiche implementative della giornata:

```text
Trajectory Generator/prosthetic_phase_fsm.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
validation/test_reward_function.py
validation/validate_training_config.py
validation/test_phase_fsm_prescribed_env.py
```

Report creati:

```text
reports/user/2026-06-30_implementazione_fsm_protesica_hs_to_hs.md
reports/user/2026-06-30_specifica_corridor_band_reward_morphology.md
reports/user/2026-06-30_training_smoke_fsm_exnovo_10iter_rollout.md
reports/user/2026-06-30_training_fullbatch_fsm_exnovo_100iter_rollout.md
reports/daily/2026-06-30_daily-report.md
```

Artifact principali:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best
Trajectory Generator/runs/training/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout
```

## TODO chiusi oggi

- Rilanciare rollout con nuova guardia e verificare `phase_timeout:swing`.
- Eseguire mini training ex-novo diagnostico.
- Implementare FSM protesica `HS -> TO -> HS`.
- Aggiungere `phase_event_progress_score`.
- Aggiungere `landing_window_contact_score`.
- Aggiungere `invalid_event_loss` e `contact_validity_loss`.
- Aggiungere config/logging dedicati.
- Scrivere test unitari e smoke test env/reward/config.
- Testare FSM con CMC-like sim e dati prescribed.
- Eseguire smoke training/rollout FSM.
- Eseguire training full-batch 100 iter con batch 4096 e rollout.

## TODO aperti propagati

- Rendere piu efficace il segnale positivo verso il nuovo HS: il full-batch
  100 iter resta in `phase_timeout:swing`.
- Ritarare o ridisegnare `landing_window_contact_score`, che nel rollout 100
  iter resta sempre `0.0` pur essendo attivo per 55 step.
- Verificare se il detector online o le soglie GRF rendono troppo difficile
  riconoscere il recupero contatto durante training.
- Valutare `policy_action_clip_weight > 0` o altra strategia anti-saturazione:
  nel rollout 100 iter tutti gli step hanno almeno un canale clippato.
- Tenere `morphology_weight: 0.0` finche' non sono presenti cicli protesici
  validi e una fase morphology affidabile.
- Dopo cicli affidabili, implementare/attivare morphology come corridor band
  plateau reward/loss, non come tracking verso la media AB06.

## Stato finale

La parte infrastrutturale GRF/FSM e' implementata e verificata. La parte
comportamentale non e' ancora risolta: il training full-batch conferma che la
policy non produce un ciclo completo e che il prossimo intervento deve essere
sulla reward densa di recupero contatto/landing e sulla saturazione delle
azioni, non sulla morphology.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
