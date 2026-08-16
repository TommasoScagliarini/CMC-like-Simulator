# Daily Report - 2026-08-05

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Protocollo H0/V25 A/B/C congelato](../user/2026-08-05_protocollo_h0_v25_abc_congelato.md)
- [Integrazione `binary_active` V25/V20 verso la FSM di H0](../user/2026-08-05_binary_active_v25_v20_adapter_h0.md)
- [Preflight full-environment H0/V25 A-B-C — `ERROR_H0_REFERENCE`](../user/2026-08-05_preflight_h0_v25_abc_error_h0_reference.md)

## Sintesi ed esito della giornata

La giornata ha portato il detector binario V25 e la FSM V20 fino a un
percorso runtime `binary_active` completo, transazionale e compatibile con
l'interfaccia actor-facing esistente. Ha inoltre congelato il protocollo H0
A/B/C e il relativo execution-unlock.

Il primo rollout reale del preflight, caso A `det_minus020`, ha però chiuso
correttamente in fail-closed con:

```text
ERROR_H0_REFERENCE
```

H0 ha superato la penetrazione massima ammessa al passo 53: `26,628 mm`
contro il gate `<25 mm`. V20 era disabilitata e V25 non partecipava alle
osservazioni actor; il risultato non è quindi un FAIL del detector. Il test ha
dimostrato che l'actor H0 pre-split non è compatibile con le feature continue
corrette `primary_grf_split_v1`, nelle quali carico e contatto provengono dalla
GRF primaria anziché dal vecchio detector analogico.

La matrice A/B/C è stata consumata senza retry. B `binary_shadow`, C
`binary_active`, H0_sep, trial protetti, corridor positivo e PPO non sono stati
aperti. Al termine della giornata non esiste ancora un protocollo H0 training
ready.

## 1. Contesto ereditato e TODO chiusi

La catena development del 3–4 agosto aveva separato definitivamente tre
responsabilità:

- GRF primaria: carico continuo e contatto fisico;
- detector V25: due bit force-free tallone/punta;
- FSM V20: trasformazione causale dei bit in eventi funzionali.

V24/V25 avevano già corretto il gap geometrico OFF/OFF di 12 ms osservato in
V23, ottenendo 24/24 unità development PASS su 02/04/08. Il 5 agosto sono
stati chiusi i seguenti prerequisiti rimasti aperti:

- [x] integrazione V25 dormant/shadow con default legacy invariato;
- [x] protocollo dichiarativo H0 A/B/C congelato;
- [x] correzione del primo lock A/B asimmetrico, preservato come evidenza
      rigettata;
- [x] modalità `binary_active` e adapter V20 -> `ProstheticPhaseFSM`;
- [x] reset AIR/partial-stance senza HS sintetico;
- [x] journal V25 nel caso A, action tape, driver/comparatore e destinazioni
      no-clobber;
- [x] preflight layout actor/full `35/84`, dtype `float32` e assenza di campi
      V25/V20 nell'osservazione actor;
- [x] receipt separato di execution-unlock.

I vecchi TODO di modificare ulteriormente la geometria V19/V21 o la FSM per
correggere gli anticipi non sono più attivi: la lineage autorevole è V25/V20.
I trial 08 V21/V22/V23 restano evidenza storica consumata e non vanno
reinterpretati.

## 2. Protocollo H0/V25 A/B/C congelato

Il lock autorevole è:

```text
validation/h0_v25_abc_protocol_corrected_lock.json
SHA-256 04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b
```

Il primo lock, SHA `4f24deb4...`, è stato conservato ma rigettato perché il
solo caso B conteneva un campo ridondante. Il lock corretto dimostra che A e B
differiscono esclusivamente per nome del caso e modalità della FSM binaria.

La matrice congelata prevedeva:

- A: eventi legacy autorevoli, V25 campionato, V20 disabilitata;
- B: stessa baseline A con V20 soltanto in shadow, da confrontare bit-exact;
- C: V25/V20 unica autorità per gli eventi sinistri, senza fallback legacy;
- tre partenze deterministiche e seed stocastici 123–125;
- 18 rollout fisici, organizzati in 12 unità da 500 step.

Il freeze del protocollo non autorizzava da solo esecuzione, training o
promozione.

## 3. Integrazione runtime `binary_active`

Il percorso implementato è:

```text
V25 bit heel/toe
  -> BinaryPhaseFSM V20
  -> BinaryPhaseActiveAdapter
  -> ProstheticPhaseFSM
  -> osservazioni e reward già attesi da H0
```

L'adapter avanza copie private delle due FSM, verifica eventi, timestamp,
ordine e stato, quindi effettua un commit congiunto soltanto se l'intero policy
step è valido. Un errore non può lasciare uno stato parzialmente aggiornato.

Routing congelato:

- eventi sinistri: esclusivamente V25 -> V20 -> adapter in active;
- eventi destri: stream analogico legacy;
- GRF sinistra continua e `in_contact`: esclusivamente GRF primaria;
- detector: sempre force-free e separato dalla dinamica;
- nessun fallback prescribed o legacy per gli eventi sinistri active.

Il reset con piede già in stance crea una partial stance non accreditata, non
un HS sintetico. Il primo TO può chiuderla; il primo ciclo completo nasce solo
dopo swing e HS reali. Il default attivo resta comunque `legacy_events` con
morphology reward a peso zero.

## 4. Preflight full-environment e causa dell'errore

Il caso A `det_minus020`, offset `1.756870983805102 s` e seed 123, si è
arrestato a 53/500 step:

| Metrica | Risultato |
| --- | ---: |
| Penetrazione al passo 52 | `24,097 mm` |
| Penetrazione al passo 53 | `26,628 mm` |
| Limite | `<25 mm` |
| Cicli completi | `0` |
| Safety stop | `1` |

Il confronto read-only con il riferimento storico del 14 luglio ha isolato la
divergenza:

- reset, 35 feature actor e prima azione: bit-identici;
- dopo il primo step: 33/35 feature ancora bit-identiche;
- divergono soltanto `online_left_normal_grf_bw` e
  `online_left_in_contact`;
- lo storico usava circa `10,533 N` del detector analogico;
- il nuovo runtime usa correttamente la GRF primaria, circa
  `9,455e-05 N` in quel campione.

La conseguente divergenza della seconda azione porta poi alla diversa dinamica
e al safety stop. Ripristinare il detector analogico come sorgente del carico
continuo maschererebbe il problema e violerebbe il contratto
`primary_grf_split_v1`.

Il supervisor ha quindi fermato l'intera matrice. Non sono stati eseguiti
retry, sostituzioni di seed, retuning o modifiche dei gate.

## File modificati o aggiunti

### Runtime e adapter

- `Trajectory Generator/binary_phase_adapter.py`
- `Trajectory Generator/prosthetic_phase_fsm.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `simulation_runner.py`
- `static_optimization.py`

### Governance e validazione

- `validation/freeze_h0_v25_abc_protocol.py`
- `validation/freeze_h0_v25_abc_protocol_corrected.py`
- `validation/test_freeze_h0_v25_abc_protocol.py`
- `validation/test_freeze_h0_v25_abc_protocol_corrected.py`
- `validation/h0_v25_abc_protocol_lock.json`
- `validation/h0_v25_abc_protocol_corrected_lock.json`
- `validation/run_h0_v25_abc_preflight.py`
- `validation/compare_h0_v25_abc.py`
- `validation/freeze_h0_v25_abc_execution.py`
- `validation/test_h0_v25_abc_preflight.py`
- `validation/test_binary_phase_active_adapter.py`
- `validation/test_binary_phase_active_env.py`
- `validation/h0_v25_abc_layout_preflight_receipt_v3.json`
- `validation/h0_v25_abc_preflight_test_receipt_v3.json`
- `validation/h0_v25_abc_execution_unlock.json`
- `validation/h0_v25_abc_runs/2026-08-05_h0_v25_abc_full_environment_preflight/`

Non sono stati modificati V20, il profilo V25, `online_grf.py`, la geometria o
legge di contatto della GRF primaria, il plugin C++, la semantica SEA o i pesi
H0.

## Test e verifiche

- freeze del protocollo e regressioni correlate: 82/82 PASS;
- suite combinata detector/adapter/env/GRF: 129/129 PASS;
- test specifici `binary_active`: 20/20 PASS;
- suite mirata del preflight: 69 PASS prima del freeze;
- layout full/actor `84/35`, azione `(2,)` e dtype `float32`: PASS;
- `py_compile`, Ruff e `git diff --check`: PASS;
- strict JSON, finitezza, hash e destinazioni no-clobber: PASS;
- rollout reale A: FAIL previsto dal gate fisico, con classificazione
  `ERROR_H0_REFERENCE` confermata dall'audit indipendente;
- zero clipping, timeout, fallback, hard-invalid, non-finiti o saturazioni SEA
  prima dello stop.

## Scope rimasto chiuso

- B `binary_shadow` e C `binary_active` non eseguiti;
- H0_sep e adattamento actor non eseguiti;
- trial protetti 05/06 e reserve 03/07 non aperti;
- nessuna promozione V25/V20;
- corridor con reward positivo, PPO e training non avviati;
- claim numerico limitato a macOS arm64; parità/DLL Windows non attestate.

## TODO aperti e propagati

### Nuova lineage H0

- [ ] Non rilanciare né modificare la matrice A/B/C consumata.
- [ ] Non ripristinare il detector analogico come sorgente delle feature
      continue; preservare `primary_grf_split_v1`.
- [ ] Autorizzare, progettare e congelare una nuova lineage per adattare
      l'actor H0 pre-split alla semantica della GRF primaria.
- [ ] Definire un nuovo gate baseline A per il candidato adattato e soltanto
      dopo un PASS predisporre un nuovo confronto shadow/active.
- [ ] Conservare actor H0 originale, lock rigettato, lock corretto e run
      terminale come evidenze immutabili.

### Gate successivi ancora chiusi

- [ ] Eseguire B/C e la qualification H0 soltanto dopo una baseline adattata
      valida.
- [ ] Richiedere una decisione separata prima di aprire one-shot il trial 05
      e, soltanto dopo PASS, il trial 06.
- [ ] Mantenere 03/07 come reserve chiuse.
- [ ] Eseguire save/reload zero-update e creare un checkpoint warm-start solo
      dopo tutti i gate H0.
- [ ] Mantenere corridor positivo, morphology reward positivo, PPO e training
      chiusi fino alla reale training readiness.
- [ ] Attestare DLL e parità Windows prima di estendere il claim numerico oltre
      macOS arm64.

### Corridor e ablation ereditati

- [ ] Completare un rollout live shadow da 500 step e un confronto A/B del
      corridor in un processo OpenSim pulito; il replay offline non sostituisce
      il rollout live.
- [ ] Definire bordo phase-dependent, sicurezza immediata o retrospettiva,
      `WAIT_HS`, disponibilità morfologica e gestione di segmenti incompleti,
      timeout e bootstrap prima del GAE.
- [ ] Aggiungere logging per evento e testare pesi morphology piccoli soltanto
      dopo i gate causali, su start/seed held-out e almeno un profilo o modello
      esterno ad AB06.
- [ ] Mantenere le reserve come gate fisico e non come reward finché l'origine
      dei residui non è attribuita.
- [ ] Documentare l'esito terminale del pilot V13/corridor storico e non
      promuoverne checkpoint contaminati dal routing precedente.

### Robustezza, audit e deployment ereditati

- [ ] Valutare ogni candidato con multistart, seed held-out, recovery,
      cicli, penetrazione, SEA, reserve e clipping; non modificare reward,
      feature actor o hard limit da 25 mm per forzare un PASS.
- [ ] Conservare exact-start, compaction, interleaving e gate reserve
      condition-matched; non riutilizzare le stesse trace/seed per altro tuning
      post-hoc e differire memoria ricorrente finché non emerge un limite
      sequenziale reale.
- [ ] Eseguire prove più lunghe e su trial, velocità e soggetti differenti e
      chiarire il TO precoce storico prima di claim di generalizzazione.
- [ ] Confrontare H0 e logical 24 con protocollo identico ed eseguire ablation
      progressive di reset, perturbazioni, detector, GRF, morphology e
      command-rate modificando un solo fattore per volta.
- [ ] Congelare il contratto hardware di feature, scaling, frequenza, unità,
      filtri, FSM, reset, governor, fallback e watchdog; validare export
      actor-only, equivalenza host-target, latenza worst-case, HIL e poi
      human-in-the-loop.
- [ ] Validare detector con rumore/delay realistici e ground truth localizzata,
      portare la GRF online a stato production-ready mantenendola separata dai
      sensori e valutare la deflessione SEA iniziale coerente con la coppia.
