# Osservazione ex-novo con stato reference deployable

Data: 2026-07-01

## Problema

Durante la discussione sul possibile uso di memoria nel `Trajectory Generator`,
e' emerso che alcuni segnali erano stati esclusi dalla configurazione ex-novo
recente per effetto di:

```yaml
deployable_minimal_observation: true
include_reference_state_observation: false
```

Questa scelta rendeva l'actor molto minimale, ma rimuoveva anche segnali che non
sono leakage imitativo e che restano compatibili con un deployment realistico:

- velocita' articolari protesiche;
- endpoint precedente comandato dalla policy;
- reference effettivamente servita dal governor/reference model;
- velocita' e accelerazione della reference servita;
- ultimo comando SEA e stato di saturazione comando.

La distinzione importante e' che questi segnali sono ottenibili da sensori della
protesi o da stato interno del controllore. Non sono equivalenti ai target sani
anti-fase della reward imitativa.

## Soluzione

La configurazione ex-novo e' stata aggiornata per riabilitare l'osservazione
actor non-minimale e includere lo stato della reference servita:

```yaml
include_reference_state_observation: true
deployable_minimal_observation: false
```

Con `reward_mode: ex_novo`, questa modifica non riabilita i quattro target
imitativi:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

Questi restano aggiunti solo quando `reward_mode == "imitation"` tramite
`env_factory.py`.

## Strategia

La modifica e' una forma leggera di "memoria deployable" prima di introdurre una
rete ricorrente o un world model in stile P22. Invece di aggiungere subito
GRU/LSTM, si forniscono all'actor stati interni e sensori realisticamente
disponibili che aiutano a rendere il problema piu' osservabile:

- `pros_knee_angle_vel`, `pros_ankle_angle_vel`: velocita' da encoder/stima;
- `previous_endpoint`: comando precedente noto al generatore;
- `served_ref`, `served_ref_vel`, `served_ref_accel`: stato interno del reference
  model/governor;
- `sea_u`, `sea_u_abs`, `sea_u_saturated`: ultimo comando e saturazione lato SEA;
- stati motore SEA quando disponibili.

La scelta mantiene il principio ex-novo: nessun target prescritto sano/protesico
viene passato all'actor come traiettoria da imitare.

## File modificati

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - `include_reference_state_observation` impostato a `true`;
  - `deployable_minimal_observation` impostato a `false`.

- `validation/validate_training_config.py`
  - aggiornati i check della config ex-novo per aspettarsi observation
    non-minimale e reference state observation attiva.

## Test e verifiche eseguite

Eseguito:

```text
python3 validation/validate_training_config.py
```

Risultato:

```text
ALL SMOKE CHECKS PASSED
```

Il validatore ora conferma:

```text
exnovo deployable minimal observation off
exnovo reference state observation on
train exnovo config deployable minimal obs off
train exnovo config reference state obs on
```

## Stato finale

La prossima esecuzione di training ex-novo con
`Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml` usera' un actor piu'
informato ma ancora deployable. I checkpoint precedenti non sono compatibili con
questo nuovo observation contract.

## TODO

- Lanciare un nuovo training ex-novo con observation actor non-minimale e
  ledger/clawback reward gia aggiornati.
- Confrontare rollout e training metrics contro il run ledger-eval precedente,
  in particolare:
  - `phase_valid_cycle_count`;
  - frequenza di `phase_timeout:swing`;
  - saturazione azioni;
  - uso dei nuovi segnali `served_ref`, `previous_endpoint` e velocita'
    articolari nella dinamica appresa.
