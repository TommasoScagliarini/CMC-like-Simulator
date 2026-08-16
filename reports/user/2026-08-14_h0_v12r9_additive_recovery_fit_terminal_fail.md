# H0 V12R9 additive recovery: raccolta valida, fit terminal FAIL

Data: 2026-08-14

## Obiettivo

Recuperare un warm-start imitativo stabile a partire dal prefisso V12R8 e dal
teacher H0 canonico, aggiungendo osservazioni `student-exposed` senza ripetere
rollout storici e mantenendo chiusi Q3, checkpoint-zero e Morphology Corridor
fino al superamento dei gate imitativi.

## Strategia eseguita

Il protocollo V12R9 one-shot ha:

- giudicato e importato il prefisso terminale V12R8;
- importato 179 label `+0.20` gia valide;
- prodotto offline 252 label per il prefisso `-0.20`;
- raccolto e labelizzato quattro rollout nuovi da 500 step: nominale e seed
  stocastici 126, 127 e 128;
- costruito un corpus case-balanced di 11.875 righe e 13 strati, ciascuno con
  massa 500;
- eseguito un unico fit dell'attore `35 -> 512 -> 512 -> 2`, senza sweep,
  retry, fallback, critic update o PPO update.

Protocol freeze SHA-256:
`b674ad45f88555fc1fa28766a90ba06d6494b07f12454508a24eaf7773e88e02`.

Execution lock SHA-256:
`4713a0a37c399dad4e2f324d7d4fdacc5e5bfe60707fc22b5143543111a24ec6`.

## Risultato delle raccolte

Le quattro nuove raccolte fisiche hanno completato tutte 500 step. Il ledger
contabilizza:

- 4 reset e 2.000 environment step nuovi;
- 20.000 campioni raw del detector;
- 2.252 query offline del teacher H0: 2.000 nuove, 252 sul prefisso storico;
- 179 label importate;
- zero environment step per i prefissi storici;
- zero sviluppo candidato, Q3, checkpoint-zero o morphology;
- zero critic update e zero PPO update.

Tutti i primi sette stage hanno pubblicato receipt validi. Le evidenze e le
label rimangono quindi riutilizzabili da un lineage successivo senza nuovi
rollout di raccolta.

## Problema emerso nel fit

L'unico fit ha migliorato la loss pesata da circa `3.938e-4` a `1.053e-4`, ma
non ha raggiunto le soglie preregistrate (`RMSE <= 0.006`, errore massimo
`<= 0.06`; al reset errore massimo `<= 0.003`):

- globale: RMSE `0.01100096`, massimo `0.14004979`;
- reset: massimo `0.01345137` su 26 righe;
- peggior riga: caso observer nominale, step 385, dimensione azione 0;
- observer nominale: RMSE `0.01830573`, massimo `0.14004979`;
- observer seed 127: RMSE `0.01457422`, massimo `0.10464972`;
- base `-0.20`: RMSE `0.01439924`, massimo `0.10821137`;
- finestra R4 `+0.20`: RMSE `0.00761341`, massimo `0.05014789`;
- observer `+0.20` tardivo: RMSE `0.00744797`, massimo `0.01988961`.

Sono passati i controlli di architettura, clock disabilitato, log-std byte
exact, save/reload, singolo fit e assenza di critic/PPO. Sono falliti i gate
global, per-case, observer recovery, observer-plus-late, R4 recovery e reset.

Il candidato materiale
`AB06_H0_V12R9_RECOVERY_W512:8bc8554c573f8224ea3fa9d8682d315f9ff30c4aaa1557c4974e4fa422b5d1ff`
e stato osservato ma non congelato ne promosso.

## Esito terminale

Il runner ha pubblicato
`FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL` allo stage
`fit_recovery_actor`, con errore `V12R9RecoveryFitError`. Come previsto dal
contratto non sono stati eseguiti retry o stage successivi.

Ledger terminale SHA-256:
`591fdc6ebe6e2a553b24ff210232a0f16185c4bc7b5e33104e34b780a0a18785`.

## Soluzione e direzione successiva

V12R9 resta terminale e immutabile. Il recupero viene aperto in un nuovo
lineage, riusando read-only corpus, label e receipt V12R9. Prima di scegliere
un nuovo fit vengono misurati conflitti osservazione-label, limiti inferiori e
distribuzione degli errori: un altro tentativo identico o un semplice
allentamento delle soglie non e autorizzato dai dati. Q3, checkpoint-zero e
Morphology Corridor rimangono differiti fino a un candidato imitativo PASS.

## File e artefatti coinvolti

- sorgenti e test del protocollo in
  `Trajectory Generator/baseline_MLP/validation/v12r9/`;
- freeze e lock canonici nella stessa cartella;
- run, claim e ledger in
  `validation/v12r9/h0_v12r9_run_20260814/`;
- corpus, manifest, metriche e candidato non promosso nella sottocartella
  `fit/`;
- label e receipt delle raccolte nella sottocartella
  `observer_collection/`.

## Test e verifiche

Prima del freeze:

- 76 test V12R9 PASS;
- 13/13 controlli del contratto PASS;
- source closure su 89 file PASS;
- Ruff lint e format-check PASS;
- `compileall` PASS;
- preflight reale degli input e import delle evidenze PASS;
- stato finale pre-esecuzione `READY_H0_V12R9_RECOVERY_ONE_SHOT`.

Dopo l'esecuzione:

- integrita dei sette receipt completati verificata;
- corpus SHA-256
  `1b35d0789d11a0f3bca3cae15c5877ceaf68845bf69ed7231d5a4ecc4d5b9dfe`;
- gate e ledger terminali fail-closed pubblicati;
- audit Morphology Corridor correttamente `DEFERRED`, senza freeze o scritture.

## TODO vincolanti

- Non ritentare, modificare o promuovere V12R9.
- Chiudere la forense del fit e costruire un successore in namespace nuovo,
  senza nuove raccolte salvo evidenza che le renda indispensabili.
- Eseguire Q3 indipendente, checkpoint-zero e Morphology Corridor A/B soltanto
  dopo un terminal PASS imitativo del successore.
- Dichiarare training-ready soltanto dopo il preflight reale 0 -> 12 EnvRunner
  e avviare il training da 50 update attraverso il launcher morphology.
