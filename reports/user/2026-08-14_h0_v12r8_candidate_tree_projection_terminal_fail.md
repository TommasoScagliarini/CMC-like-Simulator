# H0 v12r8 — errore terminale di proiezione dell'albero candidato

Data: 2026-08-14

## Esito

L'esecuzione canonica one-shot H0 v12r8 si e chiusa correttamente in stato
terminale di fallimento:

- stato: `FAIL_H0_V12R8_RECOVERY_PIPELINE_TERMINAL`;
- stage tentato: `collect_label__deterministic_offset_minus_0p20`;
- errore: `V12R8RecoveryProbeError: probe summary identity drifted`;
- retry e resume: entrambi non autorizzati;
- nessun fit attore, update attore, update critic o update PPO eseguito.

R8 non viene corretto ne rieseguito. Sorgenti e artefatti della revisione restano
immutabili come evidenza forense; il recupero prosegue in una revisione additiva
R9.

## Problema

Il probe R8 serializza nel `summary.json` l'identita completa del modulo candidato:
`path`, `tree_sha256`, `file_count` e la lista dettagliata `files`. Il controllo
post-persistenza confrontava questa struttura completa con una proiezione del
contratto che conteneva soltanto `path`, `tree_sha256` e `file_count`.

Il confronto stretto ha quindi segnalato un drift pur avendo osservato lo stesso
modulo candidato. I campi sostanziali coincidono:

- path: `validation/v12r6/.../candidate/rl_module_composite`;
- tree SHA-256: `340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3`;
- file count: `5`;
- i cinque file e i relativi hash sono quelli gia bloccati nel protocol freeze.

La causa e pertanto un errore di proiezione nel verificatore, non un drift del
candidato, del detector o della simulazione. Il test unitario non lo aveva
intercettato perche la fixture riproduceva la stessa proiezione incompleta del
contratto invece della struttura completa prodotta dal probe reale.

Un secondo audit indipendente ha ristretto la differenza a
`mismatch_keys = ["candidate_module"]` e
`actual_extra_keys = ["files"]`, con `projection_exact = true`. Un
controfattuale eseguito esclusivamente in memoria, senza modificare R8, fa
passare `verify_probe_closure()` quando usa il record completo; tutti gli altri
campi restano invariati.

## Evidenza fisica prodotta prima dell'arresto

La raccolta `deterministic_offset_minus_0p20` ha prodotto un prefisso fisico
integro prima del confronto fallito:

- 1 reset ambiente e 252 step ambiente;
- 2.520 campioni grezzi dei sensori binari, esattamente 10 per step;
- 252 righe di trace, 252 transizioni di replay e 253 boundary;
- 3 eventi V26: heel strike, toe off, heel strike;
- termine: `grf_penetration`, con massimo `0.025214003999146035 m`;
- zero eventi duplicati, fuori ordine, invalidi o da sorgenti non V26;
- zero fallback, hard-invalid, routing failure e valori non finiti;
- teacher non caricato, zero query al teacher e zero dipendenze teacher nelle
  azioni servite;
- morphology weight `0.0`, come richiesto per la raccolta osservatore;
- zero actor/critic/PPO update.

Il gate del prefisso e `PASS_H0_V12R8_RECOVERABLE_R6_PREFIX`: tutti i controlli
di integrita, replay, identita del candidato, purezza delle azioni e detector
sono `true`. Il gate full-horizon V26 resta naturalmente non applicabile e
`false` perche l'episodio e un prefisso terminato dalla guardia di penetrazione;
la sua normalizzazione specifica per prefissi e invece `passed: true`.

Prima di questa raccolta R8 aveva anche completato, senza nuova dinamica fisica:

1. l'adjudication del prefisso storico `deterministic_offset_plus_0p20`;
2. il labeling offline di quel prefisso con 179 chiamate al teacher H0 bloccato.

## Artefatti e hash

- protocol freeze: `fa593d9ffc213805d47fb9d1c58a63db7aae2fdc337f479eefdffcfb9a9c8710`;
- execution lock: `44ecb103398982cfa9f0c267748e2f6fec466f0fdafd5639cef20bc5a1f0c3f0`;
- terminal ledger: `d5564c9b96976453033885010cc23b709d7071d63c5380fac62f7822485344b9`;
- albero completo del prefisso minus (259 file):
  `5e4a32ab3a46e8de6c833aeb4c1030f3389b54457fac2388caf6e0f0e5feb0c9`;
- gate: `c189a4b19b4cdd2cc65b6117a9386068bd3877dcbfab6d1b31bc639111ab9f7f`;
- receipt: `9da6b83f6c2bde4d096f3c5e10120653f7e0aab4a35bf5ac34340a29132867d9`;
- replay boundaries: `f920419987b3c1f92ef1b99a661f04e5c6bf821473fbee7e2a8e28addb0a670f`;
- summary: `97cf526f612d3ee4f2a835660f31c0c634ef9d4550f2598660f38d077434d6d4`;
- trace: `d6ae27641ab7832c23832290cfd234f2758c02f0ad2afbf4b749404507f67087`.

Input offline ancora bloccati:

- teacher H0 tree SHA-256:
  `f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee`;
- corpus di coverage, 6.000 righe:
  `232e0776f67e7a1425288c4f3979409df998ef34a7e60c618ec6c5d7cd9c4933`.

## Soluzione e strategia di recupero

La soluzione e una revisione R9 completamente additiva:

1. attestare il ledger terminale R8 e l'intero albero del prefisso minus;
2. dimostrare che l'unica divergenza e la proiezione incompleta del record
   candidato, confrontando in R9 strutture complete reali;
3. importare semanticamente i due prefissi storici plus e minus senza riaprire
   o riscrivere R8;
4. completare offline il label minus con lo stesso teacher bloccato;
5. eseguire soltanto le quattro raccolte fisiche mancanti: nominale e seed
   126–128;
6. assemblare le sei casistiche, ricostruire i 13 strati, eseguire un unico fit
   W512 e le sei development rollout;
7. aprire Q3, checkpoint-zero/warm-start e morphology solo dopo il terminale
   PASS di R9.

Attivita fisica attesa in R9: quattro reset di raccolta piu sei reset di
development; un solo fit attore; zero update critic e PPO.

## File coinvolti

Nessun file R8 e stato modificato dopo il freeze. Il nuovo lavoro applicativo
vive esclusivamente sotto `Trajectory Generator/baseline_MLP/validation/v12r9/`.

Questo report e stato aggiunto in:
`reports/user/2026-08-14_h0_v12r8_candidate_tree_projection_terminal_fail.md`.

## Verifiche eseguite

- `verify_protocol_freeze()`: `PASS_H0_V12R8_RECOVERY_PROTOCOL_FREEZE`, 82
  sorgenti;
- `verify_execution_lock(require_pristine=False)`:
  `PASS_H0_V12R8_RECOVERY_EXECUTION_LOCK`;
- `verify_terminal_ledger()`: ledger terminale valido con `passed: false`;
- hash corrente di `experimental_morphology_corridor.py`:
  `258700b5da99aa9110a92039834bf4061ca81b9eb7ea770b93919a0b3105e801`,
  identico al source closure R8;
- audit del gate e dei contatori del prefisso: integrita `true`, anomalie runtime
  e detector tutte a zero.
- audit controfattuale read-only: 6/6 binding artefatti validi, trace/replay
  byte-exact e unico scarto strutturale limitato alla chiave `files` omessa
  dalla proiezione attesa.

## Stato della readiness

R8 non autorizza ancora il training: e una revisione terminalmente fallita. Il
prefisso valido e pero recuperabile e consente di proseguire senza sprecare una
seconda esecuzione identica. La readiness verra dichiarata soltanto dopo i PASS
concatenati di R9, Q3, checkpoint-zero/warm-start, morphology A/B e smoke
training finale.
