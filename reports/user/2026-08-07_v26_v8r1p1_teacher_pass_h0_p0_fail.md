# Detector V26 e protocollo H0 V8R1P1 — teacher replay PASS, P0 offline FAIL

Data: 2026-08-07

## Esito

Il detector binario V25 con la nuova interpretazione heel-qualified V26 ha
superato tutti i gate development e il replay dinamico necessario a sbloccare
l'adattamento H0. Il difetto V20 è risolto: un contatto `TOE` isolato durante
lo swing resta visibile in diagnostica, ma non genera più una falsa coppia
TO/HS.

Il protocollo H0, invece, **non è training ready**. Il teacher replay finale
V8R1P1 ha chiuso PASS su 6/6 casi e ha prodotto 3.000 transizioni valide, ma il
primo fit residuale P0 ha mancato le tre soglie offline preregistrate. La
pipeline si è quindi fermata prima di DAgger, P1, qualification, trial protetti
e zero-update port.

Non esiste un `checkpoint_zero` V8R1P1 eleggibile e non viene quindi fornito un
comando di warm-start: lanciarlo adesso significherebbe addestrare da un
candidato che il protocollo ha dichiarato FAIL.

## Problema affrontato

La geometria binaria V25 produce due bit grezzi, tallone e punta. Nel journal
che aveva chiuso V7, durante lo swing compariva un breve contatto della sola
punta. La FSM V20 trattava qualunque contatto come inizio dello stance e
trasformava quel segnale in una coppia funzionale spuria:

- TO a `18,515870983807038 s`;
- HS a `18,579870983807116 s`.

Il risultato violava la durata minima dello swing della FSM actor-facing e
interrompeva il teacher replay H0. La GRF primaria online non era la causa e
non è stata modificata.

## Soluzione V26

V26 conserva immutati geometria V25, campionamento a 1 ms, debounce a 5 ms e
consegna alla policy a 10 ms. Cambia soltanto la semantica funzionale:

- in swing, soltanto `HEEL` o `BOTH` possono armare un HS;
- `TOE` isolato resta uno stato grezzo diagnostico e non apre lo stance;
- in stance, `HEEL`, `BOTH` e `TOE` mantengono il contatto;
- soltanto `AIR` stabile può armare un TO;
- il reset con contatto già presente resta una partial stance senza HS
  sintetico.

Il contratto eventi finale è:

```text
binary_point_v25+heel_qualified_fsm_v2
```

Il runtime esterno continua a usare `binary_active`; internamente l'adapter
seleziona in modo esatto V26 e la sorgente actor-facing `binary_active_v26`.
V20 rimane disponibile e byte-for-byte immutata.

## Validazione V26

### Development 02/04/08

- 24/24 unità PASS;
- tre trial × quattro plateau × processamento scalar 1 ms e batch 10 ms;
- parità scalar/batch: 3/3;
- F1 minimo: `0,9589701041`;
- IoU minimo: `0,9211744138`;
- massimo errore assoluto HS confermato: `27 ms`;
- massimo errore assoluto TO confermato: `53 ms`;
- zero eventi invalidi, timeout o non-finiti;
- receipt SHA-256:
  `a85bfb0581224f4ab36f357eccd86779030062e509dbfb38e70183598f4d28b9`.

### Replay del journal V7

Il replay V26 riproduce esattamente i sei eventi attesi:

```text
TO 15.111870983804456
HS 15.624870983804172
TO 16.632870983804736
HS 17.151870983805370
TO 18.282870983806752
HS 18.683870983807243
```

- coppia spuria V20 `18,515/18,579 s`: assente;
- due cicli completi;
- flight funzionali: `0,513`, `0,519` e `0,401 s`;
- parità scalar/batch: esatta;
- il FAIL storico V7 resta conservato e non viene reinterpretato.

## Teacher replay H0 V8R1P1

Il percorso finale ha riprodotto le azioni V5 congelate nel runtime V26 e ha
interrogato l'H0 originale soltanto per le label teacher. Non ha aggiornato
actor, critic o PPO.

| Caso | Step | Cicli | Eventi | Penetrazione max | Gate |
|---|---:|---:|---:|---:|---|
| deterministic -0,20 s | 500 | 2 | 6 | 24,602 mm | PASS |
| deterministic nominal | 500 | 2 | 6 | 23,960 mm | PASS |
| deterministic +0,20 s | 500 | 2 | 6 | 24,324 mm | PASS |
| stochastic seed 126 | 500 | 2 | 6 | 22,575 mm | PASS |
| stochastic seed 127 | 500 | 2 | 6 | 24,945 mm | PASS |
| stochastic seed 128 | 500 | 2 | 6 | 23,400 mm | PASS |

Risultato aggregato:

- 6/6 casi PASS;
- 3.000/3.000 policy step;
- 30.000/30.000 campioni binari;
- zero clipping, eventi invalidi, timeout, safety stop, fallback SEA o
  fallback SO non accettati;
- morphology reward sempre a peso zero;
- trial protetti e reserve non aperti;
- ledger SHA-256:
  `f8a1807d750762227defd87e185b9fbc36487e5570e917398f87c54ba6b24433`.

### Chiusure procedurali preservate

Prima della lineage finale sono state conservate due chiusure non
scientifiche, entrambe senza policy step utile:

1. V8: il wrapper ereditato cercava il nome simbolico storico
   `V25_ACTIVE_EVENT_CONTRACT_ID`, nonostante il valore richiesto fosse V26;
2. V8R1: il reset runtime era PASS con zero step, ma il verifier richiedeva il
   literal storico del campo `next_stage`.

V8R1P1 corregge esclusivamente questi binding, usa output no-clobber separati
e conserva i ledger precedenti. Non modifica geometria, timing, action tape,
gate o dati.

## Adattamento residuale P0

P0 ha completato le 8.000 epoche previste su tutti i 3.000 campioni teacher.
Il modulo salvato è finito e riproducibile:

- base H0 byte-exact: PASS;
- critic byte-exact: PASS;
- `logstd` byte-exact: PASS;
- save/reload byte-exact: PASS;
- actor update supervisionato: uno;
- critic update: zero;
- PPO update: zero.

Il fit riduce l'RMSE dell'H0 originale sul nuovo dominio da `0,0340582` a
`0,00329770`, pari a una riduzione del `90,32%`. Nonostante il miglioramento,
le soglie congelate non sono soddisfatte:

| Metrica | Osservata | Soglia | Esito |
|---|---:|---:|---|
| RMSE | `0,00329770` | `<= 0,0015` | FAIL |
| errore assoluto massimo | `0,0470886` | `<= 0,015` | FAIL |
| errore massimo sui reset | `1,44839e-5` | `<= 1e-5` | FAIL |

Il massimo errore è sull'azione caviglia del campione
`deterministic_offset_plus_0p20:291`: previsione `0,0666832`, teacher
`0,1137718`. Il residuale non è globalmente saturo ai propri limiti; il FAIL
non è attribuibile ai warning SciPy/SLSQP, a NaN/Inf, al save/reload o a una
mutazione accidentale di H0.

L'analisi indipendente localizza il problema soprattutto nel target usato per
l'imitazione, non nella geometria V26:

- 55/3.000 righe hanno almeno una componente d'azione oltre `0,015`;
- 50 di queste 55 righe cadono dove il teacher legacy indica contatto, mentre
  V26 è correttamente in AIR;
- sulle 606 righe con `legacy contact=1` e `V26 contact=0`, l'RMSE è
  `0,006204`, contro `0,001971` quando i due contatti concordano;
- sulle 83 righe con pulse legacy/V26 disallineato, l'RMSE sale a `0,011414`;
- sui 36 pulse V26 effettivi, invece, l'RMSE è `0,000842` e il massimo errore
  è `0,00269`.

Il fit è inoltre quasi stazionario dopo 8.000 epoche. Undici campioni caviglia
richiederebbero una correzione oltre il bound residuale `0,12`, ma un oracle
semplicemente clippato agli stessi bound avrebbe comunque RMSE `0,000285` e
massimo `0,011964`, entrambi entro gate. La capacità imposta dai bound non
spiega quindi il FAIL globale. Lo sforamento reset è marginale
(`4,48e-6` assoluti); il blocco sostanziale resta la richiesta di ricostruire
risposte istantanee a eventi legacy che non esistono nello stesso istante
nella semantica causale V26, combinata con soglie V8 molto severe.

Come riferimento diagnostico, il fit V5 era nella stessa fascia numerica
(RMSE circa `0,00302`) ma usava gate più larghi. Questo confronto non promuove
V8R1P1 e non autorizza ad allentarne retroattivamente le soglie.

Il gate terminale è:

```text
FAIL_H0_PRIMARY_SPLIT_V8R1P1_RESIDUAL_DAGGER_PIPELINE
```

Ledger SHA-256:
`9a263ea8a1b71fde357a67838bf29ef5e9a60432ecf3f833009da577b98d868d`.

L'audit ha trovato anche una difformità solo documentale: il ledger terminale
non riporta `actor_updates=1` e lascia vuoto `completed_receipts`, benché
summary, checkpoint e traccia di esecuzione attestino l'unico fit actor-only.
Questa omissione non ha causato il FAIL metrico, ma va corretta nel contratto
della prossima lineage.

## Stato dei gate

- V25 geometria binaria development: PASS e congelata;
- V26 FSM heel-qualified development: PASS;
- V26 replay journal V7: PASS;
- V8R1P1 teacher replay H0: PASS;
- V8R1P1 residual P0 offline: **FAIL terminale**;
- DAgger P0: non aperto;
- fit/freeze P1: non aperto;
- qualification H0: non aperta;
- trial protetto 05: chiuso;
- trial sealed 06: chiuso;
- reserve 03/07: chiuse;
- zero-update port: non aperto;
- corridor con reward positivo e PPO: non avviati;
- `H0_TRAINING_READY`: **NO**.

## Trial protetti

L'audit in sola lettura conferma che 05 e 06 sono integri e non consumati. Non
esistono ancora preprocessing, oracle o access receipt V26 per questi trial.
La loro apertura resta inoltre una decisione one-shot separata: non è stata
inferita dall'autorizzazione V8.

## File principali introdotti o modificati

Runtime V26:

- `Trajectory Generator/binary_phase_fsm_v26.py`;
- `Trajectory Generator/binary_phase_adapter_v26.py`;
- `Trajectory Generator/prosthetic_phase_fsm.py`;
- `Trajectory Generator/osim_trj_cmc_like.py`.

Validazione V26:

- `validation/test_binary_phase_fsm_v26.py`;
- `validation/test_binary_phase_active_adapter_v26.py`;
- `validation/test_binary_phase_active_env_v26.py`;
- `validation/validate_binary_phase_fsm_v26_development.py`;
- `validation/replay_binary_phase_fsm_v26_v7.py`;
- receipt development e replay V7.

Protocollo H0 finale:

- contract, runner e test teacher replay V8/V8R1/V8R1P1;
- `validation/h0_primary_split_v8_residual_dagger_contract.py`;
- `validation/run_h0_primary_split_v8_residual_dagger.py`;
- test contract/runner residuali;
- artifact no-clobber sotto
  `validation/h0_primary_grf_split_adaptation_runs/2026-08-07_h0_primary_split_v8r1p1_v26_residual/`.

Non sono stati modificati `online_grf.py`, il profilo/materiale/geometria della
GRF primaria, il plugin C++, la semantica SEA, la FSM V20 o i checkpoint H0
storici.

## Test e verifiche

- test puri FSM V20+V26: 18 PASS;
- test FSM/adapter combinati: 43 PASS;
- regressioni runtime/environment V20: 69 PASS;
- test specifici teacher V8/V8R1/V8R1P1: PASS;
- test contract e binding residuale V8R1P1: 10 PASS;
- `py_compile`: PASS sui file coinvolti;
- `ruff`: PASS sui nuovi file residuali;
- strict JSON e finitezza: PASS;
- parità scalar/batch V26: PASS;
- audit indipendente degli accessi: 05/06 e 03/07 chiusi;
- claim numerico: macOS arm64; Windows non attestato numericamente.

## TODO e decisione richiesta

- [ ] Decidere se autorizzare una nuova lineage H0, separata da V8R1P1, per
  correggere il mismatch fra target legacy e osservazione causale V26. Prima
  dell'esecuzione va congelata una semantica teacher coerente con V26 e va
  deciso se il gate deve misurare imitazione istantanea delle azioni o
  equivalenza comportamentale nei rollout. Un'eventuale revisione delle soglie
  richiede una motivazione indipendente; non è consentito trasformare il FAIL
  V8R1P1 in PASS retroattivamente.
- [ ] Correggere nella prossima lineage il riepilogo terminale di
  `actor_updates` e `completed_receipts`.
- [ ] Solo dopo un nuovo P0 PASS: eseguire due DAgger completi da 500 step, fit
  P1 e sei rollout development.
- [ ] Congelare ed eseguire la qualification indipendente del candidato.
- [ ] Richiedere autorizzazione separata prima di aprire one-shot trial 05 e,
  solo dopo PASS, trial 06.
- [ ] Dopo tutti i gate: eseguire zero-update save/reload e creare il vero
  `checkpoint_zero`; solo allora pubblicare il comando di warm-start.
- [ ] Mantenere morphology reward positivo, corridor training e PPO chiusi
  fino a `H0_TRAINING_READY` reale.
