# Detector V21 — trial 08 one-shot consumato con errore di contratto oracle

## Stato terminale

`ERROR_INTERNAL_V21_TRIAL08_CONSUMED`

Il trial 08 è stato aperto una sola volta dopo il freeze del candidato V21 e
del validatore. L'esecuzione si è fermata prima del campionamento del detector
per un errore nel gate del validatore, non per un fallimento della geometria
V21 o della FSM V20.

Il contratto one-shot stabiliva che PASS, FAIL, errore o interruzione dopo la
receipt consumassero definitivamente lo stage. Di conseguenza non è stato
eseguito alcun retry, non sono stati modificati gate o sorgenti congelati e non
si è proceduto a H0, corridor o training.

Trial 08 era già classificato dalla lineage V14/V17/V18 come holdout interno,
non scientificamente vergine. I trial indipendenti 05 e 06 e le reserve 03/07
sono rimasti chiusi.

## Problema

Il validatore congelato richiedeva che la griglia globale dell'oracle canonico
coincidesse con la griglia del replay cinematico V21:

- replay IK/detector atteso: `[10.690, 154.890]` s, 144.201 campioni;
- griglia nativa dell'oracle: `[10.678, 154.900]` s, 144.223 campioni.

Questa uguaglianza non era necessaria. L'oracle viene costruito sulla propria
griglia GRF nativa e le quattro view scoreable sono tutte interne sia alla
griglia oracle sia all'intervallo del replay:

| View | Intervallo [s] | Cicli completi | HS | TO |
|---|---:|---:|---:|---:|
| plateau 01 | 13,512–45,696 | 25 | 26 | 25 |
| plateau 02 | 48,363–80,708 | 33 | 34 | 33 |
| plateau 03 | 82,042–115,723 | 38 | 39 | 38 |
| plateau 04 | 118,390–150,738 | 30 | 31 | 30 |

L'oracle ha superato tutti gli altri controlli congelati:

- hash e schema canonici;
- trial e contract ID;
- campionamento a 1 ms;
- soglia 20 N;
- persistenza HS 50 ms;
- ciclo minimo 0,30 s;
- quattro view, hash delle view, boundary, conteggi e minimo cicli.

Sono falliti soltanto `grid_start`, `grid_end` e `grid_count`. Il problema è
quindi un'asserzione eccessiva introdotta nel nuovo validatore. Non esiste una
misura trial-08 delle prestazioni V21, perché l'oracle veniva validato prima di
caricare l'IK e campionare i due punti binari.

## Soluzione e strategia implementate prima dell'apertura

Sono stati aggiunti:

- freeze no-clobber di candidato, geometria, profilo, FSM, scorer, oracle,
  gate, sorgenti e scope post-PASS;
- global execution ledger e access receipt byte-identici, pubblicati prima di
  qualunque lettura trial-08;
- ownership per PID e protezione dalle race/riesecuzioni;
- preflight non semantico con caricamento della dylib verificata,
  inizializzazione del modello e risoluzione dei frame del detector;
- validazione strict dell'oracle e parità V20 scalare/batch;
- persistenza prevista dei bit packed per verifica indipendente;
- manifest di evidenza prima della decisione terminale;
- audit dei file tentati e verificati su ERROR.

Il freeze ha correttamente limitato un eventuale PASS a candidato development
pronto per l'implementazione dell'integrazione H0, senza autorizzare H0,
promozione runtime/training o PPO positivo.

## Evidenza e integrità

Freeze:

- path: `validation/binary_phase_detector_v21_trial08_freeze_lock.json`;
- SHA-256: `68774248090a071221eecdd1ca771d9f598c36b3f89d2f7497ede2cdd4cc2964`.

Stage consumato:

- execution ledger SHA-256:
  `37ce1f009e4f99904063ccb8d997e6bc2a8ad62d105a2ffe676164cc6eafebde`;
- access receipt SHA-256:
  `37ce1f009e4f99904063ccb8d997e6bc2a8ad62d105a2ffe676164cc6eafebde`;
- failure receipt SHA-256:
  `fba0255fd35e25ffc53a09af64221aa7d9868129a442ed0ee915784b4f2db5f4`.

Ledger e access receipt sono byte-identici. La failure receipt registra:

- `stage_consumed=true`;
- `rerun_allowed=false`;
- candidate `v21_678b0b5162b706dd`;
- preprocessing lock, IK, modello, plugin e oracle verificati per hash;
- nessuna apertura di trial protetti o reserve;
- nessuna pubblicazione di manifest, decision lock o trace packed.

L'IK è stato letto per verificarne l'hash ma non è stato decodificato dal
`KinematicsInterpolator`; il detector non è stato campionato. I file raw della
GRF prescritta e `ExternalLoads` non sono stati letti; è stata letta soltanto la
ledger oracle già congelata. La GRF primaria online, il plugin C++,
il contatto primario e la semantica SEA non sono stati modificati.

## File aggiunti

- `validation/freeze_binary_phase_detector_v21_trial08.py`;
- `validation/validate_binary_phase_detector_v21_trial08_one_shot.py`;
- `validation/test_binary_phase_detector_v21_trial08_one_shot.py`;
- `validation/binary_phase_detector_v21_trial08_freeze_lock.json`;
- `validation/binary_phase_detector_v21_trial08_execution_ledger.json`;
- `validation/binary_phase_detector_v21_holdout_runs/2026-08-04_trial08_one_shot/trial08_access_receipt.json`;
- `validation/binary_phase_detector_v21_holdout_runs/2026-08-04_trial08_one_shot/failure.json`;
- `reports/user/2026-08-04_trial08_v21_one_shot_error_terminale.md`.

## Test e verifiche

- 41/41 test consolidati V20/V21/one-shot PASS prima del freeze;
- `py_compile` PASS;
- freeze `--check` PASS prima e dopo la pubblicazione;
- preflight one-shot sul lock pubblicato PASS;
- plugin macOS arm64, modello e sampler V21 inizializzati con successo;
- confronto byte a byte ledger/receipt PASS;
- verifica dell'assenza di manifest, decision lock e trace dopo ERROR PASS;
- parsing dell'oracle dopo il consumo: semantica e quattro view valide;
- nessun retry eseguito.

## TODO e decisione necessaria

- [ ] Non riutilizzare questo stage one-shot e non trasformare l'ERROR in PASS
  o FAIL scientifico.
- [ ] Decidere esplicitamente se autorizzare un nuovo ciclo development sul
  trial 08, ormai dichiaratamente aperto, con il solo vincolo corretto che le
  view oracle siano coperte dalla trace detector. Un simile run sarebbe
  diagnostico/development, non holdout e non indipendente.
- [ ] In alternativa, mantenere lo stop e progettare il gate indipendente 05/06
  senza aprirlo finché validatore, compatibilità modello marker/runtime e
  routing V21 per H0 non sono completamente congelati e verificati.
- [ ] Prima di H0, attestare la compatibilità tra modello replay SHA
  `98cfcbc4...` e modello runtime SHA `33e67d84...`, implementare il target
  contract V21 e chiarire matrice dei 12 casi e semantica di
  `invalid_event_count`.

Fino a una nuova autorizzazione esplicita restano vietati retry del trial 08,
apertura di 05/06, promozione runtime/training e PPO con morphology reward
positivo.
