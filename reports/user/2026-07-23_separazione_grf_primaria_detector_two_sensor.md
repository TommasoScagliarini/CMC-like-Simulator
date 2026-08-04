# Separazione netta tra GRF primaria e detector heel/toe

Data: 2026-07-23

## Sintesi esecutiva

È stato corretto immediatamente il routing runtime tra:

- il generatore online di GRF primario, che rappresenta il contatto fisico
  usato dal simulatore;
- il profilo detector, composto da sensori virtuali heel/toe che non applicano
  forze al modello.

La separazione meccanica era già presente: i componenti detector vengono
creati con `appliesForce = false`. Il problema era nel consumo dei segnali:
quando esisteva un profilo detector, `_phase_grf_sides()` preferiva il suo
aggregato alla GRF primaria. Di conseguenza il detector non generava realmente
la GRF, ma i suoi valori aggregati potevano sostituire `normal_force_bw` e
`in_contact` nella FSM, nelle osservazioni actor/critic e, indirettamente, nel
reward.

Questa sostituzione è stata rimossa. Da ora il contratto runtime è:

| Segnale o funzione | Sorgente |
|---|---|
| Forze applicate alla dinamica | GRF primaria |
| Penetrazione e limiti di sicurezza | GRF primaria |
| `normal_force_bw` e `in_contact` | GRF primaria |
| Carico/contact feature dell'actor sinistro | GRF primaria |
| Carico/contact feature privilegiate del critic destro | GRF primaria |
| Evidenza continua di carico/contact della FSM | GRF primaria |
| Carico locale heel e toe | Detector virtuale |
| Candidati/transizioni HS e TO two-sensor | Detector virtuale + FSM |

Il detector resta quindi un detector: misura segnali locali ed è utilizzato per
riconoscere eventi, ma non genera GRF e non sostituisce la sorgente fisica nelle
feature continue.

## Problema

Il metodo precedente era:

```text
se esiste online_grf_detector["left"]:
    usa l'intero aggregato detector
altrimenti:
    usa online_grf primaria
```

Questo comportamento produceva quattro effetti indesiderati:

1. `online_left_normal_grf_bw` e `online_left_in_contact` dipendevano da
   posizione e raggio delle sfere detector;
2. `normal_force_bw` e `in_contact` passati alla FSM provenivano dal detector,
   mentre i replay prescribed dual-stream usavano correttamente la GRF primaria;
3. il reward di supporto, overload e swing unloading riceveva un carico
   detector-derived attraverso `online_gait`;
4. con V13, che contiene soltanto sensori sinistri, l'aggregato detector destro
   era vuoto e poteva azzerare le feature privilegiate destre del critic.

Questo spiega anche il significato corretto di “forza verticale normalizzata”:
non era una GRF generata dal detector. Era la lettura aggregata delle sfere
virtuali, divisa per il peso corporeo, usata impropriamente al posto della GRF
primaria.

## Soluzione

### Routing dell'environment

In `Trajectory Generator/osim_trj_cmc_like.py`:

- `_phase_grf_sides()` è stato sostituito da
  `_physical_online_grf_sides()`;
- il nuovo helper restituisce esclusivamente `_online_grf` e non effettua
  fallback sul detector;
- `_update_phase_fsm()` prende carico normalizzato e contatto dalla GRF
  primaria;
- lo stesso metodo continua a prendere soltanto
  `left_heel.normal_load_n` e `left_toe.normal_load_n` dal detector;
- `_online_gait_info()` usa la GRF primaria per entrambi i lati, quindi le
  feature actor/critic di carico e contatto non cambiano variando il detector.

Lo schema dell'observation non è cambiato: nomi, ordine e dimensionalità
restano gli stessi. È stata corretta la semantica della sorgente dei valori.

### Routing degli eventi nel runner

In `simulation_runner.py` il detector aggregato continua a essere utilizzabile
per il riconoscimento degli eventi legacy, ma soltanto sui lati realmente
presenti nel profilo detector.

Se il detector è left-only:

- gli eventi sinistri possono usare il detector;
- il lato destro conserva il segnale verticale della GRF primaria.

In questo modo un profilo V13 sinistro non può più azzerare il flusso eventi
destro tramite l'aggregato vuoto restituito per il lato non configurato.

### Invarianti preservate

- Nessuna modifica al plugin C++ SEA.
- Nessuna modifica alla semantica dei comandi SEA.
- Nessun detector applica forze al plant.
- Nessuna modifica a checkpoint, policy, profili detector, corridor,
  configurazione di training o reward.
- La penetrazione applicata e le hard termination erano già basate sulla GRF
  primaria e restano tali.

## Strategia di verifica

Sono stati aggiunti test con GRF primaria e detector deliberatamente in
contraddizione. Questo evita falsi positivi: il test fallisce se il detector
torna a essere usato come fallback o come sorgente del carico continuo.

I nuovi gate verificano che:

1. cambiare forza, contatto, posizione implicita o copertura laterale del
   detector non cambi carico/contact sinistro e destro di `online_gait`;
2. la FSM riceva `normal_force_bw` e `in_contact` dalla GRF primaria, ma i
   canali heel/toe dal detector;
3. in assenza della GRF primaria non venga usato silenziosamente l'aggregato
   detector come fallback;
4. un detector left-only non azzeri la forza usata per gli eventi legacy del
   lato destro.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - separazione esplicita della sorgente fisica;
  - routing primary-only per load/contact di FSM e observation.
- `simulation_runner.py`
  - selezione event-only del detector limitata ai lati configurati.
- `validation/test_detector_sensor_data_path.py`
  - test di regressione dual-stream per runner, FSM e observation.
- `reports/user/2026-07-23_separazione_grf_primaria_detector_two_sensor.md`
  - presente report.

## Test e verifiche eseguite

Esito complessivo dei test mirati: **72 test PASS**, oltre ai controlli statici.

```text
validation/test_detector_sensor_data_path.py       15 PASS
validation/test_prosthetic_phase_fsm_two_sensor.py 17 PASS
validation/test_online_grf_core.py                  7 PASS
validation/test_phase_fsm_prescribed_env.py         1 PASS
validation/test_reward_function.py                 32 PASS
```

Controlli aggiuntivi:

```text
py_compile                                  PASS
Ruff sui tre file coinvolti                 PASS
git diff --check                            PASS
smoke OpenSim prescribed CMC-like FSM       PASS
```

Hash SHA-256 dei file correnti al termine della verifica:

```text
e7db6ab0819de67403801e7c1b6efcfb7bb3031355b5f82ec6e38d37d4e5e739  Trajectory Generator/osim_trj_cmc_like.py
b3888b475dcac385dc831c70cb80ac8d791e63120b60e107c6bc70584c5d06f5  simulation_runner.py
048b51efbbab97dbd116533c33b9a691f6d0abab69b3e12911156e84888c88a4  validation/test_detector_sensor_data_path.py
```

## Impatto su H0 e sui training

La dimensionalità dell'observation di H0 non cambia, quindi non nasce
un'incompatibilità strutturale di checkpoint. Cambia però la distribuzione dei
valori delle feature load/contact rispetto ai run storici che avevano subito
il routing errato. Serve quindi una nuova verifica frozen-policy a zero update
prima di considerare H0 compatibile sul piano comportamentale.

Un training avviato prima di questa correzione non costituisce evidenza della
nuova separazione. Non è stato arrestato né riavviato durante questa attività;
i worker che avevano già importato il modulo mantengono il codice caricato,
mentre un eventuale riavvio automatico di worker potrebbe caricare i sorgenti
nuovi. Per evitare qualsiasi ambiguità scientifica, quel run deve restare
diagnostico e non promuovibile.

Inoltre V13 conserva il proprio stato sperimentale: il suo sealed aveva dato
FAIL e i metadati non lo autorizzano per training o inference. La correzione
del routing non promuove automaticamente V13.

## Limiti e TODO

- Eseguire un rollout frozen-policy a zero update da un processo nuovo, con
  primary GRF fissa, confrontando detector assente, `shadow` e `two_sensor`.
- Verificare l'invarianza numerica in `shadow` di load/contact observation,
  azioni, reward fisico e terminazioni.
- Quantificare separatamente l'effetto degli eventi two-sensor dopo aver
  congelato un profilo che superi development, validation e sealed.
- Rivalidare H0 dopo la correzione prima di usare `warm_start` per un nuovo
  training.
- Non promuovere checkpoint prodotti dal training V13/corridor avviato con il
  routing precedente.

