# Pilot PPO da 50 update: training integro, nessun candidato robusto

## Esito sintetico

Il pilot PPO lungo da H0 è stato completato e validato secondo il protocollo
preregistrato.

Il risultato si separa nettamente in due parti:

1. **successo tecnico del training**:
   - 50/50 nuovi update completati;
   - 230.400 transizioni reali;
   - sampling esattamente bilanciato tra i tre start;
   - zero restart, crash-restart, skip, NaN o violazioni KL;
   - 50 milestone complete e caricabili;
   - log-standard-deviation bit-exact rispetto a H0;
2. **fallimento scientifico della promozione**:
   - tutte le 8 milestone preregistrate sono state respinte;
   - nessuna milestone supera tutti i quattro gate development;
   - nessun checkpoint è stato selezionato, copiato o promosso;
   - i seed held-out 126-128 non sono stati aperti.

Il checkpoint canonico resta quindi H0:

`validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`

Il pilot dimostra che l'actor è realmente allenabile e che il processo PPO è
numericamente stabile. Non dimostra che gli update rendano il controller più
robusto closed-loop. La robustezza oscilla e, nelle milestone tarde, il drift
cumulativo da H0 aumenta senza produrre un candidato promuovibile.

## Problema affrontato

Prima di questo esperimento erano già stati verificati:

- restore completo da H0;
- actor strutturalmente esportabile;
- update PPO non nullo;
- sampling multi-start esattamente bilanciato;
- gate reserve condition-matched;
- capacità di alcuni candidati di recuperare parzialmente il caso critico.

Restava però aperta una domanda: il fallimento dei primi update era soltanto un
fenomeno transitorio, risolvibile continuando il training per 50-100 update,
oppure il protocollo di ottimizzazione produceva una robustezza non monotona?

Un singolo update non poteva rispondere. È stato quindi eseguito un pilot da 50
nuovi update, conservando ogni checkpoint e rinviando gli screening fisici alla
fine per non contendere CPU al training.

## Stato della validazione

| Livello | Esito |
| --- | --- |
| restore completo actor/critic/optimizer da H0 | PASS |
| aggiornamento reale dei pesi actor | PASS |
| stabilità numerica e contratti batch PPO | PASS |
| caricabilità ed esportazione RLModule | PASS |
| robustezza development sui gate fisici | **FAIL** |
| selezione candidato | NON RAGGIUNTA |
| gate held-out 126-128 | NON APERTO |
| promozione per deployment | NON RAGGIUNTA |

La configurazione del run registra inoltre:

- `include_controller_state_observation=true`;
- `deployable_minimal_observation=false`.

Di conseguenza questo pilot non certifica la variante a osservazione minimale.
La qualifica deployable rimane strutturale/parziale: le feature usate devono
essere disponibili nel runtime del controller, ma il controller non ha superato
la validazione fisica robusta.

## Strategia preregistrata

### Sorgente e ottimizzazione

- sorgente immutabile: H0, iterazione logica 1;
- nuovi update: iterazioni logiche 2-51;
- learning rate: `5e-7`;
- train batch: 4.608 transizioni reali;
- minibatch: 512;
- minibatch per update: 9;
- epoche: 1;
- EnvRunner: 12;
- runner per start: 4;
- frammento per runner: 384 step;
- actor allenabile;
- logstd congelato;
- checkpoint e RLModule conservati a ogni iterazione.

### Sampling esatto

Ogni update contiene:

| Start | Transizioni |
| --- | ---: |
| `1,7568709838 s` | 1.536 |
| `1,9568709838 s` | 1.536 |
| `2,1568709838 s` | 1.536 |
| **Totale** | **4.608** |

Il batch post-GAE viene compattato alle sole transizioni reali e interleaved in
round-robin. Il run-length massimo della stessa condizione è uno. Il contratto
fallisce in modo chiuso se conteggi, compaction, interleaving, epoche o
minibatch non corrispondono ai valori attesi.

### Milestone e screening

Le milestone schermate sono state fissate prima del completamento:

| Update pilot | Iterazione logica |
| ---: | ---: |
| 1 | 2 |
| 2 | 3 |
| 5 | 6 |
| 10 | 11 |
| 20 | 21 |
| 30 | 31 |
| 40 | 41 |
| 50 | 51 |

Per ciascuna milestone:

1. audit della riga training esatta;
2. rollout critico stocastico `+0,20 s / seed 123`;
3. soltanto se il critico passa, tre rollout deterministici:
   `-0,20 s`, nominale e `+0,20 s`.

Il gate richiede:

- 500 step;
- fine per time limit, non per terminazione fisica;
- almeno due cicli validi;
- penetrazione GRF strettamente inferiore a 25 mm;
- zero step con azione clippata;
- reserve non superiori a H0 nella stessa condizione, salvo tolleranza numerica.

I cap reserve H0 sono:

| Condizione | Reserve H0 |
| --- | ---: |
| deterministico `-0,20 s` | 548,830304 Nm |
| deterministico nominale | 493,828205 Nm |
| deterministico `+0,20 s` | 624,679256 Nm |
| stocastico `+0,20 s / seed 123` | 596,196563 Nm |

I seed 124-125 restano contaminati/development. I seed 126-128 sono stati
sigillati per un solo candidato finale, senza fallback.

## Soluzione implementata

### Retention e guard del trainer

Il trainer ora supporta, in modo opt-in:

- pubblicazione no-clobber di ogni
  `milestone_iteration_NNNNNN`;
- salvataggio di full checkpoint, RLModule e metadata per iterazione;
- staging privato e rename per evitare milestone parziali visibili;
- guard KL update-scoped:
  - massimo minibatch `<= 0.01`;
  - minimo minibatch `>= -1e-7`;
  - zero valori non finiti;
  - esattamente 9 minibatch;
- registrazione dei contratti exact-start, compaction e interleaving nella
  history e nella summary finale.

### Audit post-run

Sono stati aggiunti audit fail-closed distinti:

- audit restart:
  - summary e history complete;
  - zero restart/skip del supervisor;
  - log Ray legato al PID del driver;
  - rilevazione di RayActorError, restart o worker exit inattesi;
  - distinzione dello shutdown terminale intenzionale;
- audit del drift:
  - confronto diretto H0 -> ognuna delle 50 milestone;
  - quattro trace development fisse;
  - provenienza delle trace legata a un actor bit-exact a H0;
  - esclusione dei seed 126-128 anche se mascherati nel nome del path;
  - logstd bit-exact nei parametri e nelle inferenze;
  - action-mean RMSE e KL empirica cumulativa.

### Screening, evidence table e held-out

Lo screen post-training:

- valida tutte le 50 righe, non soltanto le otto milestone;
- lega il run al full checkpoint H0, all'actor digest e al protocollo;
- richiede il restart audit PASS;
- pinna source config, interpreter e script di rollout;
- non contiene alcun percorso di training, held-out, copia o promozione;
- usa il seed 123 come unico seed stocastico development;
- produce un report aggregato no-clobber.

La evidence table consolida training, drift e rollout senza assegnare punteggi,
ordinare candidati o leggere held-out.

È stato preparato anche un gate held-out paired `seal/check-seal/open`, con
matrice fissa H0/candidato su 3 start x 3 seed. Non è stato eseguito: nessun
seal, receipt o output held-out è stato creato, perché non esiste un candidato
development eleggibile.

## Risultato del training

Run:

`Trajectory Generator/runs/training/validation/warm_start_h1_runs/2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/`

| Voce | Valore |
| --- | ---: |
| inizio | 2026-07-15 01:50:44 +02:00 |
| fine | 2026-07-15 09:00:17 +02:00 |
| wall time | 25.769,077 s, circa 7 h 09 min 29 s |
| nuovi update | 50 |
| iterazioni logiche | 2-51 |
| transizioni reali | 230.400 |
| history | 50 righe contigue |
| milestone | 50 complete |
| exact-start balance | PASS 50/50 |
| KL guard | PASS 50/50 |
| minibatch KL auditati | 450 |
| KL online massimo | 0,001052326 |
| limite KL online | 0,01 |
| valori KL non finiti | 0 |
| restart / crash-restart / skip | 0 / 0 / 0 |
| return minimo | 4,025223, logica 2 |
| return massimo | 48,799704, logica 24 |
| return finale | 47,926732, logica 51 |

Il `checkpoint_best` corrisponde alla logica 24. Non è stato usato come
selettore, come previsto dal protocollo. Il return PPO non osserva direttamente
la robustezza closed-loop né la non-regressione delle reserve.

## Audit del drift actor

Tutte le 50 milestone:

- sono caricabili;
- hanno metriche finite;
- hanno logstd bit-exact a H0;
- hanno digest actor differenti, quindi l'actor viene realmente aggiornato.

Valori principali:

| Metrica H0 -> milestone | Minimo | Massimo |
| --- | ---: | ---: |
| action-mean RMSE | 9,864e-5, logica 2 | 7,756e-4, logica 49 |
| action mean delta assoluto | 3,422e-4 | 2,430e-3 |
| KL empirica media | 3,892e-4 | 2,406e-2, logica 49 |
| KL empirica massima | 2,577e-3 | 1,197e-1, logica 48 |

La KL cumulativa H0 -> milestone non è la KL online del singolo update:

- il guard online verifica che ogni update locale resti entro 0,01;
- l'audit offline misura quanto il checkpoint finale si è allontanato
  complessivamente da H0.

Non c'è quindi contraddizione tra KL online sempre PASS e KL cumulativa oltre
0,01 nelle milestone tarde. Il dato mostra accumulo di drift, non un errore del
trainer.

## Screening delle otto milestone

| Update | Logica | Return training | Critico step/cicli | Penetrazione | Reserve | Esito |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 2 | 4,025 | 212 / 1 | 25,009760 mm | 673,419582 Nm | REJECT |
| 2 | 3 | 39,394 | 214 / 0 | 25,250891 mm | 872,547083 Nm | REJECT |
| 5 | 6 | 18,156 | 500 / 3 | 24,737320 mm | 616,455752 Nm | REJECT reserve |
| 10 | 11 | 44,666 | 500 / 3 | 24,615833 mm | 593,562433 Nm | critico PASS, matrice FAIL |
| 20 | 21 | 42,758 | 214 / 0 | 25,220790 mm | 868,360239 Nm | REJECT |
| 30 | 31 | 33,192 | 500 / 3 | 24,989572 mm | 699,009666 Nm | REJECT reserve |
| 40 | 41 | 44,681 | 212 / 1 | 25,024637 mm | 643,273922 Nm | REJECT |
| 50 | 51 | 47,927 | 211 / 1 | 25,043832 mm | 643,744613 Nm | REJECT |

Sono stati eseguiti 11 rollout:

- 8 casi critici, uno per milestone;
- 3 deterministici per la sola milestone logica 11.

Non si sono verificati errori operativi.

### Dettaglio della milestone logica 11

È l'unica milestone che supera il caso critico.

| Condizione | Step | Cicli | Penetrazione | Reserve candidato / H0 | Esito |
| --- | ---: | ---: | ---: | ---: | --- |
| det. `-0,20 s` | 223 | 1 | 25,015461 mm | 544,959870 / 548,830304 Nm | FAIL fisico |
| det. nominale | 500 | 2 | 23,564383 mm | 547,703215 / 493,828205 Nm | FAIL reserve |
| det. `+0,20 s` | 500 | 2 | 24,300199 mm | 635,388344 / 624,679256 Nm | FAIL reserve |
| stoc. `+0,20 s`, seed 123 | 500 | 3 | 24,615833 mm | 593,562433 / 596,196563 Nm | PASS |

Questa milestone dimostra un miglioramento reale ma parziale:

- recupera il caso stocastico critico;
- mantiene penetrazione e reserve entro cap nel critico;
- perde robustezza sullo start anticipato;
- peggiora le reserve nominali e sullo start ritardato.

Non soddisfa quindi la matrice completa e viene respinta.

## Interpretazione

### Il training lungo è stato utile

Il pilot risponde alla domanda sull'opportunità di 50-100 update:

- più update non producono un miglioramento monotono;
- esistono finestre intermedie di recovery;
- le finestre non preservano simultaneamente tutti gli start e le reserve;
- le milestone tarde possono tornare al cedimento precoce pur avendo return
  elevato;
- il drift cumulativo cresce.

Questo risultato non sarebbe emerso da un solo update.

### Perché non proseguire ciecamente a 100

Estendere lo stesso run a 100 update non è giustificato:

- la logica 11 è parzialmente migliore della 51;
- la logica 31 completa il critico ma peggiora fortemente le reserve;
- 41 e 51 tornano a terminare intorno a step 211-212;
- il return finale è alto ma non predice il gate;
- `reward_config.reserve_residual_weight=0.0`, quindi la non-regressione
  delle reserve non è ottimizzata direttamente dal return corrente.

Altre iterazioni potrebbero generare un'altra finestra favorevole, ma non c'è
evidenza che la rendano stabile o selezionabile. Servirebbe un cambiamento
mirato del protocollo di recovery/ottimizzazione, non soltanto più durata.

## Preflight fail-closed e correzione

Il primo avvio dello screening si è fermato prima di qualsiasi rollout:

- `run_validation.failed_checks=[run_resolved_config_sha256]`;
- milestone eseguite: zero;
- seed letti: zero;
- output: solo report di preflight.

Il controllo confrontava erroneamente byte-per-byte la config sorgente con la
resolved config del run. Le due devono differire per gli override CLI
preregistrati.

La correzione non ha allentato genericamente il gate. Il validatore ora:

- pinna lo SHA della config sorgente;
- confronta i due YAML in modo ricorsivo e type-strict;
- ammette esattamente cinque differenze:
  1. `iterations: 2 -> 51`;
  2. `max_consecutive_skips: 5 -> 1`;
  3. `max_consecutive_crash_restarts: 5 -> 1`;
  4. aggiunta `retain_iteration_checkpoints: true`;
  5. aggiunta `max_minibatch_mean_kl_loss: 0.01`;
- respinge qualunque differenza ulteriore.

Il retry è stato scritto in una nuova directory no-clobber e ha superato il
preflight. Il primo stop era quindi un bug over-strict del validatore, non
un'anomalia del training e non una contaminazione scientifica.

## File modificati

### Training

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

### Audit e screening

- `validation/robust_ppo_gate.py`
- `validation/compare_policy_checkpoints.py`
- `validation/audit_training_restarts.py`
- `validation/audit_policy_milestones.py`
- `validation/ppo_pilot_screen.py`
- `validation/build_pilot_evidence_table.py`
- `validation/heldout_paired_gate.py`

### Test

- `validation/test_training_resume.py`
- `validation/test_training_start_balance.py`
- `validation/test_training_restart_audit.py`
- `validation/test_compare_policy_checkpoints.py`
- `validation/test_robust_ppo_gate.py`
- `validation/test_ppo_pilot_screen.py`
- `validation/test_build_pilot_evidence_table.py`
- `validation/test_heldout_paired_gate.py`

### Protocolli e tabelle

- `validation/pilot_protocols/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50.json`
- `validation/pilot_protocols/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_selection_addendum.json`
- `validation/pilot_evidence_tables/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50.json`

L'infrastruttura exact-start/interleaving introdotta il 14 luglio è stata
riusata, non reimplementata. Non sono stati modificati plugin C++, semantica
SEA, modello, soglia di 25 mm o cap condition-matched.

## Artefatti principali

| Artefatto | SHA-256 |
| --- | --- |
| training `summary.json` | `e97cef887d8581ab4c50f4e986422f2769857ac271981fb94b2f93d9f87748c7` |
| `train_iterations.jsonl` | `f7faab4b7798912ec68ccea7aaacf46bbdcac7cbe8e44cefdcd812e7f8f999cc` |
| restart audit | `faaf4eb11810924203923e8dd92f0593335346ee3a45fffc2ec69432fb6b24a1` |
| drift audit | `743c808ca8ae53bd81fbf9a7578db9ab6597b226fa475ac8685e7232c4b9d273` |
| screen finale retry1 | `c24fca54730d7505f383247d4ae97e9a2a3c06b16e00509f7579870c52ec285d` |
| evidence table | `7c1f0acd37d14590e4e6ef34c866e8f37acd2bfca9c74f1668701a714022df99` |
| protocollo | `45e2abb226df20bfd1860f5ec22ba1d9e3badc112b89ad62f74fe124f30da3b6` |
| selection addendum | `3332b8de1f277a2fcf465c60ac72a23169814a99066902f8ed444d3ca7b67223` |

Screen finale:

`Trajectory Generator/runs/rollout/validation/ppo_pilot_screens/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_retry1/ppo_pilot_screen.json`

Evidence table:

`validation/pilot_evidence_tables/2026-07-15_h0_exact_interleaved_lr5e-7_pilot50.json`

## Test e verifiche

- suite finale `validation/test_*.py`: **199/199 PASS**;
- suite completa pre-screening dopo il fix config: 188/188 PASS;
- test focalizzati screen/config fail-closed: 56 PASS;
- test gate held-out paired: 11 PASS;
- regressioni robust gate: 28 PASS;
- smoke configurazione training: 75/75 PASS;
- `py_compile`: PASS;
- `ruff` sul nuovo gate held-out: PASS;
- `git diff --check`: PASS;
- restart audit:
  - 37.285 righe driver log analizzate;
  - 37 righe lifecycle;
  - 36 evidenze `num_restarts=0`;
  - zero finding;
- checkpoint canonico finale bit-exact alla milestone 51;
- 50/50 milestone con logstd bit-exact a H0;
- evidence table `status=COMPLETE`, senza ranking o lettura held-out.

## Decisione e TODO

Decisione:

- tutte le milestone PPO sono **REJECT**;
- H0 resta canonico;
- nessun held-out viene aperto;
- nessuna promozione o copia di checkpoint;
- il run non viene esteso automaticamente a 100 update.

TODO vincolanti per il prossimo esperimento:

1. ripartire da H0 o da una sorgente esplicitamente preregistrata, non dalla
   milestone finale 51;
2. mantenere sampling exact-start, compaction, interleaving, singola epoca e
   gate condition-matched;
3. concentrare il recovery sugli stati/eventi intorno a step 210-230, dove
   ricorrono le terminazioni per penetrazione;
4. progettare e preregistrare un meccanismo che consideri esplicitamente la
   non-regressione delle reserve, valutando un termine reward, un vincolo o un
   criterio di aggiornamento dedicato;
5. non usare il return o `checkpoint_best` come proxy di robustezza;
6. mantenere i seed 126-128 sigillati finché un solo candidato non supera
   l'intera matrice development;
7. non allentare soglia di 25 mm, numero minimo di cicli, zero clipping o cap
   reserve condition-matched;
8. eseguire un nuovo pilot controllato prima di qualunque altro run lungo.
