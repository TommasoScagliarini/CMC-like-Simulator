# Training ex-novo timeout 20 iter: failure mode phase_timeout:swing

## Problema

Dopo la validazione smoke della nuova guardia `phase_timeout`, e' stato eseguito
un mini training ex-novo da 20 iterazioni per verificare se la policy imparasse
a evitare il caso patologico:

```text
HS protesico iniziale -> TO protesico -> nessun nuovo HS -> phase_timeout:swing
```

L'obiettivo non era cercare performance definitiva, ma capire se il solo timeout
hard/soft generasse abbastanza segnale per spingere la policy a recuperare un
nuovo heel strike protesico.

## Soluzione

Training eseguito:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --output-dir "runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter" \
  --iterations 20 \
  --checkpoint-every 1
```

Al termine e' stato valutato il checkpoint migliore:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  --checkpoint "Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter/rl_module_best" \
  --output-dir "runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best" \
  --record-outputs
```

Il checkpoint migliore coincide con l'iterazione 20:

```text
checkpoint_best logical_iteration = 20
checkpoint_last  logical_iteration = 20
```

## Strategia

L'analisi e' stata fatta su due livelli:

1. leggere `summary.json` e `train_iterations.jsonl` del training per valutare
   andamento return, lunghezza episodio e cause di terminazione;
2. eseguire un rollout deterministico del `rl_module_best` e analizzare trace,
   GRF online, eventi HS/TO, azioni policy, cinematica, morphology diagnostica,
   recruitment e SEA.

Sono stati generati plot diagnostici nella cartella del rollout.

## Risultati training

Metriche principali:

```text
iterations_completed: 20
elapsed_wall_time_s: 8657.9
best_episode_return_mean: 19.987309
last_episode_return_mean: 19.987309
last_episode_len_mean: 142.064
last_phase_timeout_swing_count: 443
last_episode_time_limit_count: 1
last_grf_penetration_count: 111
```

Il return medio aumenta durante il training, ma la lunghezza episodio resta
bloccata intorno a `142` step e gli end reason `phase_timeout_swing` diventano
dominanti. Questo indica un miglioramento spurio dentro lo stesso failure mode,
non un apprendimento della sequenza gait valida.

## Risultati rollout

Rollout del `rl_module_best`:

```text
steps: 142
episode_return: 22.723086
reward_mean: 0.160022
reward_min: -0.386998
reward_max: 0.316573
terminated: True
truncated: False
end_reason: phase_timeout:swing
termination_time_s: 14.410
```

Timing evento:

```text
left_TO_time_s: 13.102
hard_timeout_expected_s: 14.402
termination_time_s: 14.410
```

Quindi la terminazione avviene esattamente dove previsto dal nuovo hard timeout
swing.

## Eventi e contatto

```text
left_contact_fraction: 0.069718
right_contact_fraction: 0.788028
left_events: HS=1, TO=1
right_events: HS=1, TO=1
```

Il lato protesico non chiude un nuovo ciclo `HS -> TO -> HS`. Il training non ha
prodotto un nuovo heel strike protesico valido.

## Azioni e cinematica

```text
raw_knee_action_mean: -1.522331
raw_knee_action_min: -2.129700
knee_clip_fraction: 0.880282
raw_ankle_action_mean: 0.339943
ankle_clip_fraction: 0.000000
knee_q_mean_rad: -1.350386
knee_q_last_rad: -1.500387
ankle_q_mean_rad: 0.238404
ankle_q_last_rad: 0.204751
```

Il canale knee resta fortemente spinto verso il bound inferiore e clippato per
circa l'88% degli step. La caviglia non clippa, ma resta in una configurazione
che non produce recupero del contatto protesico.

## Morphology e recruitment

```text
morphology_loss_mean: 18.866721
morphology_knee_loss_mean: 23.411625
morphology_ankle_loss_mean: 14.321817
morphology_phase_range: [0.000000, 0.000000]
tau_reserve_norm_max_nm: 843.598
tau_reserve_norm_last_nm: 734.883
```

La morphology resta diagnostica (`morphology_weight = 0.0`) e conferma che la
cinematica e' fuori corridoio. La fase morphology resta bloccata a `0.0`, quindi
non e' ancora utilizzabile come reward phase-dependent attiva.

## File modificati o generati

File di codice/configurazione:

- nessun file di codice modificato durante questa analisi;
- e' stata usata la reward gia aggiornata in
  `Trajectory Generator/baseline_MLP/reward_function.py`;
- e' stata usata la config
  `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`.

Output training:

- `Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter/summary.json`;
- `Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter/train_iterations.jsonl`;
- `Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter/rl_module_best`;
- `Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter/checkpoint_best`.

Output rollout:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/rollout_summary.json`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/rollout_policy_trace.json`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/sim_outputs/`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/analysis_report.md`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/analysis_summary.json`.

Plot generati:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/00_training_20iter_summary.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/01_timeout_reward_components.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/02_online_grf_events.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/03_actions_kinematics_morphology.png`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best/plots/04_recruitment_sea.png`.

Report creato:

- `reports/user/2026-06-30_training_exnovo_timeout_20iter_failure_mode.md`.

## Test e verifiche

Verifiche eseguite:

- training completato con `ok: true`;
- rollout deterministico del `rl_module_best` completato con `ok: true`;
- output `.sto` generati e disponibili in `sim_outputs/`;
- eventi online verificati: `left HS=1`, `left TO=1`, nessun secondo HS;
- plot PNG generati e verificati con `file`;
- ispezione visiva dei plot `00_training_20iter_summary.png` e
  `01_timeout_reward_components.png`.

## Conclusione

Il timeout funziona come guardia e terminazione, ma non basta come segnale di
apprendimento. Il mini training da 20 iterazioni non ha imparato a produrre una
sequenza protesica valida `HS -> TO -> HS`; ha solo migliorato il return medio
restando nel regime che termina a `phase_timeout:swing`.

Non conviene continuare questo training fino a 100 iterazioni senza cambiare la
reward.

## Proposta specifica: macchina di fase protesica

Il prossimo intervento dovrebbe costruire una macchina di fase protesica
esplicita, separando:

1. detector eventi GRF;
2. stato discreto del ciclo protesico;
3. reward positiva di progressione;
4. penalita/terminazioni per eventi non validi.

### Stati

Stati minimi proposti:

```text
WAIT_HS
STANCE_AFTER_HS
SWING_AFTER_TO
VALID_CYCLE_COMPLETED
TIMEOUT
INVALID_EVENT
```

Significato:

- `WAIT_HS`: nessun HS protesico valido ancora osservato;
- `STANCE_AFTER_HS`: osservato HS valido, si attende TO;
- `SWING_AFTER_TO`: osservato TO valido, si attende nuovo HS;
- `VALID_CYCLE_COMPLETED`: osservata sequenza completa `HS -> TO -> HS`;
- `TIMEOUT`: stance troppo lunga senza TO oppure swing troppo lungo senza HS;
- `INVALID_EVENT`: evento fuori ordine o contatto non valido.

### Transizioni

Transizioni desiderate:

```text
WAIT_HS
  -- valid HS -->
STANCE_AFTER_HS

STANCE_AFTER_HS
  -- valid TO in timing window -->
SWING_AFTER_TO

SWING_AFTER_TO
  -- valid HS in landing window -->
VALID_CYCLE_COMPLETED

VALID_CYCLE_COMPLETED
  -- reset/update cycle counters -->
STANCE_AFTER_HS
```

Transizioni patologiche:

```text
WAIT_HS
  -- no HS entro startup/first-contact guard -->
TIMEOUT

STANCE_AFTER_HS
  -- second HS before TO -->
INVALID_EVENT

STANCE_AFTER_HS
  -- stance_elapsed > phase_stance_hard_timeout_s -->
TIMEOUT

SWING_AFTER_TO
  -- second TO before HS -->
INVALID_EVENT

SWING_AFTER_TO
  -- swing_elapsed > phase_swing_hard_timeout_s -->
TIMEOUT
```

### Definizione di evento valido

Un evento non dovrebbe essere premiato solo perche' il detector lo emette.
L'evento deve superare anche guardrail fisici.

`valid_HS`:

```text
event == heel_strike
side == left
normal_force > confirmation_threshold
contact_duration_predicted/confirmed >= min_contact_duration
penetration <= penetration_valid_max
slip_speed <= slip_valid_max
non e' un doppio HS nello stesso stato stance
```

`valid_TO`:

```text
event == toe_off
side == left
preceduto da valid_HS
contact_duration_s >= min_stance_duration
stance_elapsed_s dentro finestra plausibile
non e' un doppio TO nello stesso stato swing
```

Valori iniziali coerenti con i report precedenti:

```text
event threshold: 20 N nel rollout corrente
detector consigliato validato: low_threshold=15 N, confirmation_threshold=120 N
min_contact_duration: 0.03-0.05 s
phase_period_nominal_s: 1.58 s
phase_stance_fraction_min/max: [0.50, 0.80]
phase_swing_timeout_s: 0.90 s
phase_swing_hard_timeout_s: 1.30 s
phase_stance_timeout_s: 1.45 s
phase_stance_hard_timeout_s: 2.20 s
```

### Reward positiva di progressione

Il problema attuale e' che il timeout punisce il mancato HS, ma non premia in
modo abbastanza esplicito la produzione del nuovo HS. Serve un termine positivo:

```text
phase_event_progress_score
```

Proposta di scoring per episodio/step:

```text
valid HS iniziale:
  +0.25 credito evento, una sola volta per ciclo

valid TO dopo HS:
  +0.25 credito evento, una sola volta per ciclo

valid nuovo HS dopo TO:
  +0.50 credito evento, chiude il ciclo

ciclo HS->TO->HS completo:
  phase_event_progress_score = 1.0 per quel ciclo
```

Per evitare reward spike troppo rari, il credito puo' essere mantenuto come stato
del ciclo e decadere/azzerarsi solo al reset del ciclo:

```text
cycle_progress_credit =
  0.00 in WAIT_HS
  0.25 dopo valid HS
  0.50 dopo valid TO
  1.00 dopo valid HS successivo
```

Poi nella reward:

```text
base += blend_phase_event_progress * cycle_progress_credit
```

Peso iniziale suggerito:

```text
blend_phase_event_progress: 0.15-0.25
```

Questo deve competere con `blend_tracking=0.20`, altrimenti la policy continuera'
a preferire il tracking del proprio riferimento servito.

### Reward densa per landing window

Il nuovo HS e' un evento discreto e raro. Per renderlo apprendibile serve anche
un segnale denso durante `SWING_AFTER_TO`.

Definire una finestra temporale di landing:

```text
swing_elapsed < 0.55 s:
  early swing

0.55 s <= swing_elapsed <= 1.10 s:
  landing window

swing_elapsed > 1.10 s:
  late swing / timeout zone
```

La finestra va tarata rispetto a:

```text
phase_swing_timeout_s = 0.90 s
phase_swing_hard_timeout_s = 1.30 s
```

Termine proposto:

```text
landing_window_contact_score
```

Logica:

```text
if state == SWING_AFTER_TO and swing_elapsed < landing_window_start:
    reward per unloading:
        normal_force_bw vicino a 0

if state == SWING_AFTER_TO and landing_window_start <= swing_elapsed <= landing_window_end:
    reward per ricomparsa progressiva di GRF valida:
        normal_force_bw sale verso contact_load_target_bw
        penetration sotto soglia
        slip_speed sotto soglia
        CoP/contact validity ok

if state == SWING_AFTER_TO and swing_elapsed > landing_window_end:
    nessun bonus landing
    timeout loss cresce
```

Forma numerica iniziale:

```text
landing_force_target_bw = 0.20-0.40
landing_force_full_credit_bw = 0.50-0.65
landing_window_start_s = 0.55
landing_window_end_s = 1.10
landing_invalid_penalty se penetration/slip fuori soglia
```

Score semplice:

```text
force_score = clip(normal_force_bw / landing_force_full_credit_bw, 0, 1)
validity_score = 1 se penetration/slip validi, altrimenti 0
landing_window_contact_score = force_score * validity_score
```

Peso iniziale suggerito:

```text
blend_landing_window_contact: 0.10-0.20
```

### Penalita anti-gaming

La policy potrebbe imparare a generare micro-contatti o penetrazioni per
ottenere eventi. Servono penalita esplicite:

```text
invalid_event_loss:
  doppio HS senza TO
  doppio TO senza HS
  TO troppo precoce dopo HS
  HS troppo precoce dopo TO

contact_validity_loss:
  penetration eccessiva
  slip eccessivo
  normal force impulsiva ma non sostenuta
  durata contatto sotto minimo
```

Le terminazioni hard devono restare per:

```text
phase_timeout:stance
phase_timeout:swing
grf_penetration hard
joint range hard
```

Ma gli eventi invalidi non devono sempre terminare subito: nel primo pass puo'
essere meglio applicare loss forte e lasciare terminare solo se l'errore persiste
o se viola sicurezza fisica.

### Logging necessario

Per debuggare il prossimo training, aggiungere log espliciti:

```text
phase_state_id
phase_state_name
phase_cycle_progress_credit
phase_event_progress_score
landing_window_contact_score
valid_hs_count
valid_to_count
valid_cycle_count
invalid_event_count
invalid_event_loss
landing_force_score
landing_validity_score
swing_elapsed_s
stance_elapsed_s
```

Senza questi segnali in TensorBoard il training resta difficile da interpretare:
il return puo' salire mentre la policy rimane nello stesso failure mode.

### Implementazione consigliata

Implementare dentro:

```text
Trajectory Generator/baseline_MLP/reward_function.py
```

estendendo `_phase_regular_terms()` oppure introducendo una funzione separata:

```python
_phase_fsm_terms(info)
```

Preferenza: funzione separata, per non mescolare:

```text
phase_regular_terms:
  timing/ordine/loss/timeout

phase_fsm_terms:
  stato discreto, progresso evento, landing score, invalid event score
```

Campi config da aggiungere in `training_exnovo_cfg.yaml`:

```yaml
blend_phase_event_progress: 0.20
blend_landing_window_contact: 0.15
phase_landing_window_start_s: 0.55
phase_landing_window_end_s: 1.10
phase_landing_force_full_credit_bw: 0.65
phase_invalid_event_weight: 0.10
phase_contact_validity_weight: 0.10
```

La reward ex-novo diventerebbe:

```text
base =
  blend_tracking * tracking_score
  + blend_contact_load * contact_load_score
  + blend_phase_regular * phase_regular_score
  + blend_phase_event_progress * phase_event_progress_score
  + blend_landing_window_contact * landing_window_contact_score
  - penalty
```

Questa modifica e' il candidato piu diretto per risolvere il failure mode
osservato: non cambia la semantica del SEA, non introduce imitazione cinematica,
ma rende premiata la sequenza di cammino che oggi manca.

## TODO

- Aggiungere un reward positivo esplicito di progressione evento:
  `HS -> TO -> HS`.
- Aggiungere un termine denso di `landing_window_contact_score` durante lo swing,
  premiando il recupero progressivo di GRF valida nella finestra temporale
  corretta.
- Mantenere il timeout hard come guardia, ma non usarlo come unico segnale per
  insegnare il nuovo HS.
- Rimandare l'attivazione di `morphology_weight` finche' la fase morphology non
  avanza correttamente e sono presenti cicli protesici validi.
