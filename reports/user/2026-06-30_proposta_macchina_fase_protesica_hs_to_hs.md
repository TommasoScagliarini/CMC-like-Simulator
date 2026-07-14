# Proposta standalone: macchina di fase protesica HS-TO-HS

## Scopo del documento

Questo documento e' scritto per poter riaprire il lavoro in una nuova chat e
ricostruire senza ambiguita la proposta discussa per la reward ex-novo.

Il problema da risolvere non e' piu la sola terminazione per timeout: quella e'
stata validata. Il problema aperto e' rendere conveniente per la policy generare
una sequenza protesica coerente:

```text
HS -> stance caricata -> TO -> swing scarico -> nuovo HS
```

La sequenza minima desiderata e':

```text
HS -> TO -> HS
```

## Contesto attuale

Training/rollout rilevanti:

```text
Training:
Trajectory Generator/runs/training/MLP_ExNovo_training_06-30-2026_timeout_20iter

Rollout:
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-30-2026_timeout_20iter_best
```

Risultato del training da 20 iterazioni:

```text
iterations_completed: 20
best_episode_return_mean: 19.987309
last_episode_len_mean: 142.064
last_phase_timeout_swing_count: 443
last_episode_time_limit_count: 1
```

Risultato rollout del `rl_module_best`:

```text
steps: 142
episode_return: 22.723086
terminated: True
truncated: False
end_reason: phase_timeout:swing
termination_time_s: 14.410
```

Eventi osservati nel rollout:

```text
left_contact_fraction: 0.069718
right_contact_fraction: 0.788028
left_events: HS=1, TO=1
right_events: HS=1, TO=1
left_TO_time_s: 13.102
hard_timeout_expected_s: 14.402
```

Conclusione: il timeout funziona come guardia e terminazione, ma da solo non
insegna alla policy a generare il nuovo heel strike protesico. Il training ha
migliorato il return medio restando dentro lo stesso failure mode.

## Principio di progetto

Non basta punire il mancato HS. Serve un reward positivo e leggibile per la
progressione degli eventi:

```text
premia valid HS
premia valid TO dopo HS
premia valid nuovo HS dopo TO
premia ciclo completo HS -> TO -> HS
```

Il sistema deve separare:

1. detector eventi GRF;
2. macchina a stati del ciclo protesico;
3. reward positiva di progressione evento;
4. reward densa per landing durante lo swing;
5. penalita anti-gaming;
6. terminazioni hard per casi patologici.

La modifica deve restare dentro:

```text
Trajectory Generator/
```

e in particolare nel perimetro reward/training:

```text
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
Trajectory Generator/baseline_MLP/tb_logging.py
```

Non modificare il plugin C++ SEA e non cambiare la semantica del comando SEA.

## Stati della macchina di fase

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

Codifica numerica suggerita per logging:

```text
WAIT_HS = 0
STANCE_AFTER_HS = 1
SWING_AFTER_TO = 2
VALID_CYCLE_COMPLETED = 3
TIMEOUT = 4
INVALID_EVENT = 5
```

## Transizioni

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

## Definizione di eventi validi

Un evento non deve essere premiato solo perche' il detector lo emette. Deve
superare anche guardrail fisici e temporali.

### valid_HS

```text
event == heel_strike
side == left
normal_force > confirmation_threshold
contact_duration_predicted/confirmed >= min_contact_duration
penetration <= penetration_valid_max
slip_speed <= slip_valid_max
non e' un doppio HS nello stesso stato stance
```

### valid_TO

```text
event == toe_off
side == left
preceduto da valid_HS
contact_duration_s >= min_stance_duration
stance_elapsed_s dentro finestra plausibile
non e' un doppio TO nello stesso stato swing
```

### Valori iniziali

Valori coerenti con i report e con la configurazione corrente:

```text
event threshold nel rollout corrente: 20 N
detector consigliato validato: low_threshold=15 N
confirmation_threshold consigliato: 120 N
min_contact_duration: 0.03-0.05 s
phase_period_nominal_s: 1.58 s
phase_stance_fraction_min/max: [0.50, 0.80]
phase_swing_timeout_s: 0.90 s
phase_swing_hard_timeout_s: 1.30 s
phase_stance_timeout_s: 1.45 s
phase_stance_hard_timeout_s: 2.20 s
```

Nota: il detector `grf_detector_HS-TO` era gia stato validato con errore circa
`13 ms` su HS e `26-27 ms` su TO, quindi il punto non e' ricostruire il detector
da zero. Il punto e' usare gli eventi in una FSM reward-aware.

## Reward positiva di progressione evento

Nuovo termine proposto:

```text
phase_event_progress_score
```

Scoring concettuale:

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

Per evitare che il reward sia solo uno spike raro, mantenere un credito di stato:

```text
cycle_progress_credit =
  0.00 in WAIT_HS
  0.25 dopo valid HS
  0.50 dopo valid TO
  1.00 dopo valid HS successivo
```

Uso nella reward:

```text
base += blend_phase_event_progress * cycle_progress_credit
```

Peso iniziale suggerito:

```text
blend_phase_event_progress: 0.15-0.25
```

Ragionamento: deve competere con `blend_tracking=0.20`, altrimenti la policy
continua a preferire il tracking del proprio riferimento servito senza generare
il nuovo HS.

## Reward densa per landing window

Il nuovo HS e' un evento discreto e raro. Serve un segnale denso durante
`SWING_AFTER_TO`.

Nuovo termine proposto:

```text
landing_window_contact_score
```

Finestra temporale iniziale:

```text
swing_elapsed < 0.55 s:
  early swing

0.55 s <= swing_elapsed <= 1.10 s:
  landing window

swing_elapsed > 1.10 s:
  late swing / timeout zone
```

Questa finestra e' coerente con:

```text
phase_swing_timeout_s = 0.90 s
phase_swing_hard_timeout_s = 1.30 s
```

Logica desiderata:

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

## Penalita anti-gaming

La policy potrebbe imparare a generare micro-contatti o penetrazioni per ottenere
eventi. Servono penalita esplicite.

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

Nel primo pass, gli eventi invalidi non devono necessariamente terminare subito:
meglio loss forte e terminazione solo se l'errore persiste o se viola sicurezza
fisica.

## Formula reward proposta

Formula ex-novo attuale, semplificata:

```text
base =
  blend_tracking * tracking_score
  + blend_contact_load * contact_load_score
  + blend_phase_regular * phase_regular_score
  - penalty
```

Formula proposta:

```text
base =
  blend_tracking * tracking_score
  + blend_contact_load * contact_load_score
  + blend_phase_regular * phase_regular_score
  + blend_phase_event_progress * phase_event_progress_score
  + blend_landing_window_contact * landing_window_contact_score
  - penalty
```

La parte post-clip resta:

```text
reward =
  clipped_base
  - safety_term
  - grf_penetration_term
  - grf_ankle_moment_flip_term
  - phase_timeout_penalty_term
  - exnovo_task_term
  - prosthetic_joint_range_term
  - morphology_term
  - oob_term
```

Il timeout resta una guardia, non il principale segnale di apprendimento.

## Config iniziale proposta

Campi da aggiungere in `training_exnovo_cfg.yaml`:

```yaml
blend_phase_event_progress: 0.20
blend_landing_window_contact: 0.15

phase_landing_window_start_s: 0.55
phase_landing_window_end_s: 1.10
phase_landing_force_full_credit_bw: 0.65

phase_invalid_event_weight: 0.10
phase_contact_validity_weight: 0.10
```

Possibili diagnostiche extra:

```yaml
phase_first_hs_timeout_s: 0.40
phase_min_stance_duration_s: 0.05
phase_min_swing_duration_s: 0.20
phase_contact_penetration_valid_max_m: 0.03
phase_contact_slip_valid_max_m_s: 1.0
```

Questi ultimi valori vanno verificati sui dati online GRF gia disponibili prima
di essere usati come hard guard.

## Logging necessario

Per TensorBoard e rollout trace:

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

Senza questi segnali, il return puo' salire mentre la policy rimane nello stesso
failure mode.

## Implementazione consigliata

File principale:

```text
Trajectory Generator/baseline_MLP/reward_function.py
```

Preferenza implementativa:

```python
_phase_fsm_terms(info)
```

Separare le responsabilita:

```text
_phase_regular_terms:
  timing, ordine eventi, period loss, timeout loss, hard timeout

_phase_fsm_terms:
  stato discreto, progress credit, landing score, invalid event loss
```

Poi in `_task_reward_terms(info)` fondere i termini:

```python
phase_terms = self._phase_regular_terms(info)
phase_fsm_terms = self._phase_fsm_terms(info)
...
return {
    ...
    **phase_terms,
    **phase_fsm_terms,
}
```

In `compute_reward(...)` aggiungere i due score positivi alla base ex-novo:

```python
base = (
    cfg.blend_tracking * tracking_score
    + cfg.blend_contact_load * contact_load_score
    + cfg.blend_phase_regular * phase_regular_score
    + cfg.blend_phase_event_progress * phase_event_progress_score
    + cfg.blend_landing_window_contact * landing_window_contact_score
    - penalty
)
```

In `tb_logging.py`, aggiungere i nuovi termini a `_LOSS_KEYS` o alle diagnostiche
automatiche per assicurare visibilita in TensorBoard.

## Criteri di validazione

Smoke test unitario:

```text
sequenza HS -> TO -> HS:
  valid_cycle_count incrementa
  phase_event_progress_score raggiunge 1.0
  end_reason non e' phase_timeout:swing
```

Rollout atteso dopo modifica:

```text
left_events: almeno HS=2, TO=1
valid_cycle_count >= 1
episode_len_mean > 142 step
phase_timeout_swing_count scende rispetto al training 20 iter
```

Mini training diagnostico:

```text
10-20 iterazioni
morphology_weight ancora 0.0
monitorare:
  phase_event_progress_score
  landing_window_contact_score
  valid_cycle_count
  phase_timeout_swing_count
  episode_len_mean
  left_contact_fraction
  knee_clip_fraction
```

## Relazione con morphology

Non accendere `morphology_weight` prima che la macchina di fase produca cicli
protesici validi. Nel rollout 20 iter:

```text
morphology_phase_range: [0.000000, 0.000000]
```

Quindi la morphology resta diagnostica. Prima generare/validare `HS -> TO -> HS`,
poi correggere o validare `morphology_phase`, infine valutare morphology come
plateau reward/loss con peso piccolo.

## TODO operativo

- Implementare `_phase_fsm_terms(info)`.
- Aggiungere `phase_event_progress_score`.
- Aggiungere `landing_window_contact_score`.
- Aggiungere `invalid_event_loss` e `contact_validity_loss`.
- Aggiungere config e logging dedicati.
- Scrivere test unitari sulla FSM.
- Eseguire uno smoke rollout del vecchio checkpoint per verificare i nuovi
  termini diagnostici.
- Eseguire mini training 10-20 iter con morphology ancora disattiva.
