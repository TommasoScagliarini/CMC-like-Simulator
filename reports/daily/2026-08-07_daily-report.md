# Daily Report - 2026-08-07

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- [Detector V26 e protocollo H0 V8R1P1 — teacher replay PASS, P0 offline FAIL](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md)

## Sintesi ed esito della giornata

La giornata ha risolto il difetto funzionale residuo della combinazione
V25/V20 introducendo la FSM heel-qualified V26. La geometria V25 è rimasta
invariata e V26 ha superato sia tutti i gate development 02/04/08 sia il replay
dinamico del journal V7.

La nuova lineage H0 V8R1P1 ha poi completato il teacher replay con 6/6 casi
PASS e 3.000 transizioni valide. Il successivo adattamento residuale P0 ha
però fallito tutte e tre le soglie offline preregistrate, nonostante una
riduzione dell'RMSE del 90,32%. La pipeline si è fermata in:

```text
FAIL_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_DAGGER_PIPELINE
```

Non esiste un `checkpoint_zero` eleggibile e non deve essere pubblicato un
comando di warm-start. DAgger, P1, qualification, trial protetti, corridor
positivo e PPO sono rimasti chiusi. `H0_TRAINING_READY = NO`.

## 1. Problema V20 e soluzione V26

Nel journal che aveva chiuso V7, durante lo swing compariva un breve contatto
isolato della sola punta. V20 considerava qualunque contatto come inizio dello
stance e generava la coppia spuria:

- TO a `18.515870983807038 s`;
- HS a `18.579870983807116 s`.

V26 conserva geometria V25, campionamento a 1 ms, debounce a 5 ms e consegna
alla policy a 10 ms. Cambia soltanto la semantica funzionale:

- in swing, soltanto `HEEL` o `BOTH` possono armare un HS;
- `TOE` isolato resta diagnostico e non apre lo stance;
- in stance, `HEEL`, `BOTH` e `TOE` mantengono il contatto;
- soltanto `AIR` stabile può armare un TO;
- il reset in contatto resta partial stance senza HS sintetico.

Il nuovo contratto è:

```text
binary_point_v25+heel_qualified_fsm_v2
```

La modalità esterna resta `binary_active`; l'adapter seleziona internamente
V26 e la sorgente actor-facing `binary_active_v26`. La FSM V20 resta
disponibile e byte-identica come artefatto storico.

## 2. Validazione V26

### Development 02/04/08

- 24/24 unità PASS;
- parità scalar 1 ms / batch 10 ms: 3/3 trial PASS;
- F1 minimo `0,9589701041`;
- IoU minimo `0,9211744138`;
- errore HS confermato massimo `27 ms`;
- errore TO confermato massimo `53 ms`;
- zero eventi invalidi, timeout o non-finiti;
- receipt SHA-256
  `a85bfb0581224f4ab36f357eccd86779030062e509dbfb38e70183598f4d28b9`.

### Replay journal V7

V26 riproduce i sei eventi attesi, elimina la coppia spuria V20 e conserva due
cicli completi. I flight funzionali sono `0,513`, `0,519` e `0,401 s`; la
parità scalar/batch è esatta. Il FAIL storico V7 non è stato riscritto o
reinterpretato.

Questi risultati chiudono il TODO tecnico sulla falsa attivazione toe-only,
ma attestano il detector soltanto in development. Non costituiscono ancora
promozione H0 o apertura dei trial protetti.

## 3. Teacher replay H0 V8R1P1

Il replay ha eseguito le azioni V5 congelate nel runtime V26 e interrogato H0
originale soltanto per le label teacher. Nessun peso actor/critic è stato
aggiornato durante questa fase.

| Caso | Step | Cicli | Penetrazione max | Gate |
| --- | ---: | ---: | ---: | --- |
| deterministico -0,20 s | 500 | 2 | `24,602 mm` | PASS |
| deterministico nominale | 500 | 2 | `23,960 mm` | PASS |
| deterministico +0,20 s | 500 | 2 | `24,324 mm` | PASS |
| stocastico seed 126 | 500 | 2 | `22,575 mm` | PASS |
| stocastico seed 127 | 500 | 2 | `24,945 mm` | PASS |
| stocastico seed 128 | 500 | 2 | `23,400 mm` | PASS |

Aggregato:

- 6/6 casi e 3.000/3.000 step PASS;
- 30.000/30.000 campioni binari;
- zero clipping, eventi invalidi, timeout, safety stop, fallback SEA o
  fallback SO non accettati;
- morphology reward sempre a peso zero;
- trial protetti e reserve non aperti;
- ledger SHA-256
  `f8a1807d750762227defd87e185b9fbc36487e5570e917398f87c54ba6b24433`.

Le due chiusure procedurali precedenti V8 e V8R1 sono state preservate. V8R1P1
ha corretto soltanto binding simbolici/literal del verifier in destinazioni
no-clobber separate, senza modificare geometria, action tape, gate o dati.

## 4. Adattamento residuale P0 e causa del FAIL

P0 ha completato 8.000 epoche su 3.000 campioni. H0 base, critic e `logstd`
sono rimasti byte-exact; save/reload è byte-exact. È stato eseguito un solo
update actor supervisionato, zero update critic e zero update PPO.

| Metrica | Osservata | Gate | Esito |
| --- | ---: | ---: | --- |
| RMSE | `0,00329770` | `<=0,0015` | FAIL |
| Errore assoluto massimo | `0,0470886` | `<=0,015` | FAIL |
| Errore massimo reset | `1,44839e-5` | `<=1e-5` | FAIL |

Il fit riduce l'RMSE originale da `0,0340582` a `0,00329770`, ma non raggiunge
i gate congelati. Il massimo errore riguarda l'azione caviglia del campione
`deterministic_offset_plus_0p20:291`.

L'analisi localizza il blocco nel mismatch fra target legacy e osservazione
causale V26:

- 50 delle 55 righe oltre errore `0,015` cadono dove il teacher legacy indica
  contatto mentre V26 è correttamente in AIR;
- sulle 606 righe con contatto legacy/V26 discordante l'RMSE è `0,006204`,
  contro `0,001971` quando concordano;
- sugli 83 pulse disallineati l'RMSE è `0,011414`;
- sui 36 pulse V26 effettivi l'RMSE è invece `0,000842`.

Il FAIL non deriva da geometria V26, warning SciPy/SLSQP, NaN/Inf,
save/reload, mutazione H0 o semplice saturazione dei bound residuali. Non è
quindi lecito allargare retroattivamente le soglie o promuovere P0.

Il ledger presenta una difformità documentale da correggere nella prossima
lineage: non riporta `actor_updates=1` e lascia vuoto
`completed_receipts`, nonostante gli artefatti attestino l'unico fit actor-only.
La difformità non ha causato il FAIL metrico.

## Aggiornamento dei TODO ereditati dal 2026-08-05

- [x] `primary_grf_split_v1` preservato; il detector analogico non è stato
      ripristinato come sorgente del carico continuo.
- [x] progettata e congelata una nuova lineage H0 separata dalla matrice A/B/C
      consumata: V8R1P1.
- [x] corretto il problema toe-only con V26 e verificato il teacher replay su
      sei casi completi.
- [ ] ottenere una baseline actor adattata valida: P0 è stato tentato ma ha
      chiuso in FAIL terminale.
- [ ] eseguire un confronto autonomo shadow/active o qualification del nuovo
      candidato: non raggiunto perché P0 non è eleggibile.
- [ ] creare `checkpoint_zero` e comando warm-start: non consentito.
- [ ] aprire trial 05/06: ancora vietato senza decisione one-shot separata.
- [x] mantenere 03/07, corridor positivo, morphology reward positivo, PPO e
      training chiusi.
- [ ] attestare DLL e parità Windows; il claim numerico resta macOS arm64.

## File modificati o aggiunti

### Runtime V26

- `Trajectory Generator/binary_phase_fsm_v26.py`
- `Trajectory Generator/binary_phase_adapter_v26.py`
- `Trajectory Generator/prosthetic_phase_fsm.py`
- `Trajectory Generator/osim_trj_cmc_like.py`

### Validazione detector

- `validation/test_binary_phase_fsm_v26.py`
- `validation/test_binary_phase_active_adapter_v26.py`
- `validation/test_binary_phase_active_env_v26.py`
- `validation/validate_binary_phase_fsm_v26_development.py`
- `validation/replay_binary_phase_fsm_v26_v7.py`
- receipt development e replay V7 associati.

### Protocollo H0

- contract, runner e test teacher replay V8/V8R1/V8R1P1;
- `validation/h0_primary_split_v8_residual_dagger_contract.py`
- `validation/run_h0_primary_split_v8_residual_dagger.py`
- test contract/runner residuali;
- `validation/h0_primary_grf_split_adaptation_runs/2026-08-07_h0_primary_split_v8r1p1_v26_residual/`.

Non sono stati modificati `online_grf.py`, profilo/materiale/geometria della
GRF primaria, plugin C++, semantica SEA, FSM V20 o checkpoint H0 storici.

## Test e verifiche

- test puri FSM V20+V26: 18 PASS;
- test FSM/adapter combinati: 43 PASS;
- regressioni runtime/environment V20: 69 PASS;
- test teacher V8/V8R1/V8R1P1: PASS;
- test contract e binding residuale V8R1P1: 10 PASS;
- `py_compile` e Ruff: PASS sui file coinvolti;
- strict JSON, finitezza e parità scalar/batch: PASS;
- audit accessi: trial 05/06 e reserve 03/07 integri e chiusi;
- zero update PPO e critic.

## Scope rimasto chiuso

- DAgger P0, fit/freeze P1 e sei rollout development del candidato;
- qualification H0 indipendente;
- trial protetti 05/06 e reserve 03/07;
- zero-update port e `checkpoint_zero`;
- promozione runtime/training;
- morphology reward positivo, corridor training e PPO;
- claim numerico Windows.

## TODO aperti e propagati

### Nuova lineage H0 successiva a V8R1P1

- [ ] Richiedere nuova autorizzazione e congelare una semantica teacher
      coerente con V26; decidere prima se il gate misura imitazione istantanea
      delle azioni o equivalenza comportamentale nei rollout.
- [ ] Motivare indipendentemente qualsiasi nuova soglia; non trasformare il
      FAIL V8R1P1 in PASS retroattivamente.
- [ ] Correggere nel prossimo ledger `actor_updates` e
      `completed_receipts`.
- [ ] Soltanto dopo un nuovo P0 PASS, eseguire due DAgger completi da 500 step,
      fit P1 e sei rollout development.
- [ ] Congelare ed eseguire una qualification indipendente del candidato.

### Gate successivi

- [ ] Richiedere autorizzazione separata prima di aprire one-shot trial 05 e,
      soltanto dopo PASS, trial 06.
- [ ] Dopo tutti i gate, eseguire zero-update save/reload, creare il vero
      `checkpoint_zero` e soltanto allora pubblicare il comando warm-start.
- [ ] Mantenere corridor training, morphology reward positivo e PPO chiusi
      fino a `H0_TRAINING_READY` reale.
- [ ] Conservare V20, V25, V26, H0 originale, matrici consumate e FAIL
      V8R1P1 come artefatti immutabili.
- [ ] Attestare DLL e parità Windows prima di estendere il claim numerico oltre
      macOS arm64.

### Corridor e ablation ereditati

- [ ] Completare un rollout live shadow da 500 step e il confronto A/B del
      corridor in un processo OpenSim pulito; definire bordo causale,
      `WAIT_HS`, segmenti incompleti, timeout e bootstrap prima del GAE.
- [ ] Aggiungere logging per evento e testare morphology weight piccoli
      soltanto dopo i gate, su start/seed held-out e almeno un profilo o
      modello esterno ad AB06.
- [ ] Mantenere le reserve come gate fisico e documentare l'esito terminale
      del pilot V13/corridor storico senza promuoverne checkpoint.

### Robustezza, audit e deployment ereditati

- [ ] Valutare ogni candidato con multistart, seed held-out, recovery, cicli,
      penetrazione, SEA, reserve e clipping; non cambiare reward, feature actor
      o hard limit da 25 mm per ottenere un PASS.
- [ ] Conservare exact-start, compaction, interleaving e gate reserve
      condition-matched; non riutilizzare trace/seed per tuning post-hoc e
      differire memoria ricorrente finché non emerge un limite sequenziale.
- [ ] Eseguire prove più lunghe su trial, velocità e soggetti differenti e
      chiarire il TO precoce storico prima di dichiarare generalizzazione.
- [ ] Confrontare H0 e logical 24 con protocollo identico ed eseguire ablation
      progressive del plant e training A/B modificando un fattore per volta.
- [ ] Congelare il contratto hardware di feature, scaling, frequenza, unità,
      filtri, FSM, reset, governor, fallback e watchdog; validare export
      actor-only, equivalenza host-target, latenza, HIL e human-in-the-loop.
- [ ] Validare detector con rumore/delay e ground truth localizzata, portare la
      GRF online a production-ready mantenendola separata dai sensori e
      valutare una deflessione SEA iniziale coerente con la coppia.
