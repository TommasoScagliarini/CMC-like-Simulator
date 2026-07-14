# Validazione offline reward ex-novo attuale

Data: 2026-07-02

## Problema

Prima di lanciare nuovi training ex-novo era necessario verificare che la reward
attuale non penalizzasse una camminata che sappiamo gia' essere corretta.

Il dubbio principale era logico: se la reward, applicata a dati prescribed,
avesse prodotto valori negativi o problemi di slip, allora un training PPO non
sarebbe stato interpretabile. In particolare, un falso positivo sullo slip
avrebbe indicato un disallineamento tra il caso prescribed e il caso generato.

## Soluzione

E' stato preparato e corretto lo script:

```text
validation/prescribed_reward_probe.py
```

Lo script valuta la reward su dati prescribed usando:

- cinematica prescribed/IK;
- GRF prescribed;
- eventi e FSM derivati dalle GRF prescribed;
- `onlineGRF` solo come diagnostica, non come sorgente reward nel sanity check
  prescribed.

Il termine slip online e' stato disattivato nella config ex-novo corrente:

```yaml
reward:
  grf_slip_weight: 0.0
```

Motivo: lo slip dell'`onlineGRF` non e' direttamente confrontabile con i dati
prescribed e produceva falsi negativi nella validazione.

## Config validata

La reward validata e' la reward ex-novo attuale in:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Punti rilevanti:

```yaml
reward:
  reward_mode: ex_novo
  blend_tracking: 0.0
  blend_reference: 0.0
  blend_bio: 0.0
  blend_contact_load: 0.35
  blend_phase_regular: 0.25
  blend_phase_event_progress: 1.00
  blend_landing_window_contact: 0.25
  grf_slip_weight: 0.0
  morphology_weight: 0.0
```

La morfologia/corridor e' configurata ma resta diagnostica, perche':

```yaml
morphology_weight: 0.0
```

## Strategia di validazione

Sono stati eseguiti due test prescribed:

1. Finestra non allineata:

   ```text
   12.99 -> 17.99 s
   ```

2. Finestra allineata al primo HS sinistro reale:

   ```text
   13.946870983805102 -> 17.99 s
   ```

La seconda finestra e' il test principale, perche' evita il ciclo parziale
iniziale che compare partendo da `12.99 s`.

## Risultati principali

### Test prescribed allineato

Output:

```text
validation/prescribed_reward_probe_runs/prescribed_clean_left_hs_13p946870984_17p99/
```

Metriche:

```text
ok: true
steps: 405
episode_return: 136.289
reward_mean: 0.336516
reward_min: -0.00011
reward_max: 1.0
grf_slip_loss_mean: 0.0
prosthetic_slip_speed_m_s_mean: 0.0
phase_timeout_loss_mean: 0.0
phase_timeout_penalty_term_mean: 0.0
invalid_event_count_final: 0
valid_hs_count_final: 3
valid_to_count_final: 2
valid_cycle_count_final: 2
last_period_s: 1.5480071478688746
last_stance_fraction: 0.6796622091373897
```

Interpretazione:

- una camminata prescribed corretta riceve reward positiva;
- lo slip non genera falsi negativi;
- la FSM riconosce `HS -> TO -> HS`;
- non compaiono timeout;
- non compaiono eventi invalidi;
- il periodo e la stance fraction finali sono fisiologicamente coerenti con i
  parametri configurati.

### Test prescribed non allineato

Output:

```text
validation/prescribed_reward_probe_runs/prescribed_full_12p99_17p99/
```

Metriche:

```text
ok: true
steps: 501
episode_return: 25.3309
reward_mean: 0.05056
reward_min: -1.55
grf_slip_loss_mean: 0.0
valid_hs_count_final: 3
valid_to_count_final: 2
valid_cycle_count_final: 2
```

Interpretazione:

- lo slip e' comunque risolto;
- la reward media e' piu' bassa perche' la finestra parte a meta' stance;
- il timeout iniziale e' un artefatto di bordo, non un fallimento della
  camminata prescribed.

## Conclusione

La reward ex-novo attuale e' stata validata offline come sanity check positivo:

```text
prescribed kinematics + prescribed GRF -> reward positiva e FSM coerente
```

Questo supporta il lancio di uno smoke training con:

```yaml
simulation:
  episode_start_offset_s: 1.956870983805102

reward:
  grf_slip_weight: 0.0
  morphology_weight: 0.0
```

Il nuovo start corrisponde a:

```text
t_start 11.99 + offset 1.956870983805102 = 13.946870983805102 s
```

cioe' un HS sinistro reale.

## Limiti della validazione

La validazione eseguita certifica il caso positivo prescribed allineato. Non
chiude ancora tutta la reward audit suite negativa.

Restano da validare offline:

- gamba protesica ferma;
- missing TO;
- missing second HS;
- carico in swing;
- fuori range articolare;
- slip injection diagnostico;
- morphology corridor con peso non nullo.

Questi test sono pianificati in:

```text
reports/plans/2026-07-02_piano_validazione_reward_exnovo.md
```

## File modificati o coinvolti

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - reward ex-novo attuale;
  - `grf_slip_weight: 0.0`;
  - `morphology_weight: 0.0`;
  - start episodio corretto per il prossimo smoke.

- `validation/prescribed_reward_probe.py`
  - script di validazione offline prescribed.

- `validation/validate_training_config.py`
  - validatore della configurazione reward corrente.

- `validation/test_reward_function.py`
  - test unitari della reward.

- `reports/plans/2026-07-02_piano_validazione_reward_exnovo.md`
  - piano completo di validazione reward, inclusi test offline e training.

## Test e verifiche eseguite

Comandi/azioni eseguite:

```text
validation/validate_training_config.py
validation/test_reward_function.py
validation/prescribed_reward_probe.py --start-time 12.99 --end-time 17.99
validation/prescribed_reward_probe.py --start-time 13.946870983805102 --end-time 17.99
```

Esito:

```text
validate_training_config.py: PASS
test_reward_function.py: PASS
prescribed aligned reward validation: PASS
prescribed misaligned diagnostic: PASS con artefatto di bordo documentato
```

## TODO

- Completare la reward audit suite negativa prima di dichiarare la reward
  definitivamente robusta.
- Dopo lo smoke training con start corretto, confrontare il rollout con lo smoke
  precedente partito a `12.99 s`.
- Eseguire un secondo smoke con `morphology_weight > 0` solo dopo aver valutato
  il primo smoke senza corridor.

