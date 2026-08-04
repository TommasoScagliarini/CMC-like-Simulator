# Daily Report - 2026-07-15

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-15_pilot50_ppo_bilanciato_validazione_robusta.md`

## Sintesi

Il pilot PPO lungo da H0 e' stato completato con successo tecnico, ma senza
produrre un checkpoint promuovibile.

Sono stati eseguiti 50 nuovi update, pari a 230.400 transizioni reali, con
sampling esattamente bilanciato fra i tre start. Tutti i checkpoint sono stati
conservati e il training non ha mostrato restart, crash, skip, NaN o violazioni
del guard KL.

La validazione closed-loop ha però respinto tutte le otto milestone
preregistrate. Una milestone intermedia recupera il caso stocastico critico,
ma regredisce su altri start e sulle reserve. Nessun candidato e' stato
selezionato o promosso e i seed held-out 126-128 sono rimasti sigillati.

Il checkpoint canonico resta:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

## Problema affrontato

Gli esperimenti brevi avevano già dimostrato che:

- PPO aggiorna realmente l'actor;
- actor, critic e optimizer possono essere ripristinati integralmente da H0;
- il batch può essere bilanciato esattamente fra i tre start;
- alcuni update recuperano parzialmente il caso sensibile
  `+0,20 s / seed 123`.

Restava da stabilire se la fragilità fosse soltanto transitoria e potesse
sparire continuando per 50-100 update, oppure se la robustezza variasse in modo
non monotono durante l'ottimizzazione.

Un singolo update non poteva rispondere a questa domanda. È stato quindi
eseguito un pilot da 50 update, conservando ogni iterazione e rimandando gli
screening fisici alla fine per non contendere CPU al training.

## Protocollo e strategia

### Sorgente e training

- full checkpoint sorgente immutabile: H0, logical iteration 1;
- nuovi update: logical iteration 2-51;
- learning rate: `5e-7`;
- train batch: 4.608 transizioni reali;
- 12 EnvRunner, quattro per ciascuno start;
- nove minibatch da 512;
- una sola epoca;
- actor allenabile;
- log-standard-deviation congelata;
- full checkpoint, RLModule e metadati salvati a ogni iterazione.

Ogni update contiene esattamente:

| Start | Transizioni |
| --- | ---: |
| `1,7568709838 s` | 1.536 |
| `1,9568709838 s` | 1.536 |
| `2,1568709838 s` | 1.536 |
| **Totale** | **4.608** |

Il batch viene compattato alle sole transizioni reali e interleaved in
round-robin. Il run fallisce in modo chiuso se conteggi, compaction,
interleaving, numero di epoche o minibatch non coincidono con il protocollo.

### Milestone e gate fisici

Sono state preregistrate le milestone agli update pilot:

```text
1, 2, 5, 10, 20, 30, 40, 50
```

Per ogni milestone e' stato eseguito prima il rollout stocastico critico
`+0,20 s / seed 123`. Solo in caso di PASS sono stati aperti i tre rollout
deterministici `-0,20 s`, nominale e `+0,20 s`.

Il gate richiede contemporaneamente:

- 500 step e fine per time limit;
- almeno due cicli validi;
- penetrazione strettamente inferiore a 25 mm;
- zero step con azione clippata;
- reserve non superiori a H0 nella stessa condizione, salvo tolleranza
  numerica.

I seed 124-125 restano development. I seed 126-128 sono riservati a un solo
candidato finale e non possono essere aperti in assenza di un PASS completo
development.

## Implementazioni e protezioni aggiunte

### Retention e guard del trainer

Il trainer supporta ora, in modo opt-in:

- pubblicazione no-clobber di ogni `milestone_iteration_NNNNNN`;
- staging privato e rename, evitando checkpoint parziali visibili;
- full checkpoint, RLModule e metadata per iterazione;
- guard KL update-scoped con massimo `0,01`, minimo `-1e-7`, nove minibatch
  esatti e zero valori non finiti;
- registrazione dei contratti exact-start, compaction e interleaving nella
  history e nella summary.

### Audit post-run

Sono stati separati tre livelli fail-closed:

1. restart audit, che verifica summary, history, log Ray, restart, skip e
   worker exit;
2. drift audit H0-milestone, con quattro trace development congelate,
   esclusione dei seed held-out e verifica bit-exact della logstd;
3. screening fisico preregistrato ed evidence table senza punteggi o ranking.

È stato inoltre predisposto il gate held-out paired `seal/check-seal/open` su
H0 e candidato, tre start e tre seed. Il gate non e' stato aperto perché non
esiste un candidato development eleggibile.

## Risultato del training

Run:

```text
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/
```

| Voce | Risultato |
| --- | ---: |
| nuovi update | 50/50 |
| logical iteration | 2-51 |
| transizioni reali | 230.400 |
| history | 50 righe contigue |
| milestone complete | 50 |
| exact-start balance | PASS 50/50 |
| KL guard | PASS 50/50 |
| minibatch KL auditati | 450 |
| KL online massima | 0,001052326 |
| valori KL non finiti | 0 |
| restart / crash-restart / skip | 0 / 0 / 0 |
| return minimo | 4,025223, logical 2 |
| return massimo | 48,799704, logical 24 |
| return finale | 47,926732, logical 51 |
| wall time | circa 7 h 09 min 29 s |

Il `checkpoint_best` corrisponde a logical 24. Non e' stato usato come
selettore scientifico: il return PPO non osserva direttamente la robustezza
closed-loop o la non-regressione delle reserve.

## Drift dell'actor

Tutte le 50 milestone sono caricabili, finite e hanno digest actor differente
da H0, mentre la logstd resta bit-exact.

| Metrica H0-milestone | Minimo | Massimo |
| --- | ---: | ---: |
| action-mean RMSE | `9,864e-5` | `7,756e-4` |
| delta massimo action mean | `3,422e-4` | `2,430e-3` |
| KL empirica media | `3,892e-4` | `2,406e-2` |
| KL empirica massima | `2,577e-3` | `1,197e-1` |

Il guard online misura ogni singolo update, mentre l'audit offline misura il
drift cumulativo da H0. Una KL cumulativa superiore a `0,01` nelle milestone
tarde non contraddice quindi il PASS del guard per-update.

## Screening delle milestone

| Update | Logica | Critico step/cicli | Penetrazione | Reserve | Esito |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 2 | 212 / 1 | 25,009760 mm | 673,419582 Nm | REJECT |
| 2 | 3 | 214 / 0 | 25,250891 mm | 872,547083 Nm | REJECT |
| 5 | 6 | 500 / 3 | 24,737320 mm | 616,455752 Nm | REJECT reserve |
| 10 | 11 | 500 / 3 | 24,615833 mm | 593,562433 Nm | matrice FAIL |
| 20 | 21 | 214 / 0 | 25,220790 mm | 868,360239 Nm | REJECT |
| 30 | 31 | 500 / 3 | 24,989572 mm | 699,009666 Nm | REJECT reserve |
| 40 | 41 | 212 / 1 | 25,024637 mm | 643,273922 Nm | REJECT |
| 50 | 51 | 211 / 1 | 25,043832 mm | 643,744613 Nm | REJECT |

Sono stati eseguiti 11 rollout: otto critici e tre deterministici per la sola
logical 11.

### Milestone logical 11

È l'unica a superare il caso stocastico critico, ma non la matrice completa:

| Condizione | Step | Penetrazione | Reserve candidato / H0 | Esito |
| --- | ---: | ---: | ---: | --- |
| det. `-0,20 s` | 223 | 25,015461 mm | 544,959870 / 548,830304 Nm | FAIL fisico |
| det. nominale | 500 | 23,564383 mm | 547,703215 / 493,828205 Nm | FAIL reserve |
| det. `+0,20 s` | 500 | 24,300199 mm | 635,388344 / 624,679256 Nm | FAIL reserve |
| stoc. `+0,20 s`, seed 123 | 500 | 24,615833 mm | 593,562433 / 596,196563 Nm | PASS |

La milestone recupera realmente il caso sensibile, ma perde robustezza sullo
start anticipato e peggiora le reserve nominali e sullo start ritardato.

## Interpretazione e decisione

Il pilot dimostra che:

- PPO e' numericamente stabile e aggiorna realmente l'actor;
- più update non producono robustezza monotona;
- esistono finestre intermedie di recovery;
- le finestre favorevoli non preservano simultaneamente tutti gli start e le
  reserve;
- return elevato e checkpoint finale non predicono il gate fisico;
- il drift cumulativo può crescere mentre la policy torna al cedimento precoce.

Estendere automaticamente lo stesso run a 100 update non e' quindi
giustificato. Tutte le milestone sono **REJECT**, H0 resta canonico, nessun
checkpoint viene copiato o promosso e gli held-out restano chiusi.

La configurazione conserva inoltre
`include_controller_state_observation=true` e
`deployable_minimal_observation=false`: il pilot non certifica né un contratto
sensoriale minimale né il deployment hardware.

## Correzione del preflight

Il primo screen si e' fermato prima di ogni rollout perché confrontava
byte-per-byte la config sorgente con la resolved config del run, ignorando gli
override CLI preregistrati.

Il fix mantiene il comportamento fail-closed: confronta i YAML in modo
ricorsivo e type-strict e ammette soltanto cinque differenze dichiarate su
iterazioni, guard restart, retention e soglia KL. Il retry e' stato scritto in
una nuova directory no-clobber e ha superato il preflight. Il primo stop era
quindi un bug over-strict del validatore, non una contaminazione del training.

## File modificati o aggiunti

### Training

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

### Audit, screening e held-out

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

Non sono stati modificati plugin C++, modello SEA, semantica del comando SEA,
reward, soglia di 25 mm o cap reserve condition-matched.

## Test e verifiche

- suite finale `validation/test_*.py`: `199/199` PASS;
- suite pre-screening dopo il fix config: `188/188` PASS;
- test screen/config fail-closed: 56 PASS;
- gate held-out paired: 11 PASS;
- regressioni robust gate: 28 PASS;
- smoke configurazione training: `75/75` PASS;
- `py_compile`: PASS;
- Ruff sul gate held-out: PASS;
- `git diff --check`: PASS;
- 37.285 righe del driver log e 37 righe lifecycle analizzate;
- zero finding di restart o worker exit inatteso;
- 50/50 milestone con logstd bit-exact a H0;
- evidence table completa senza ranking o accesso held-out.

## TODO vincolanti del pilot

- [ ] Ripartire da H0 o da una sorgente esplicitamente preregistrata, non
      dalla milestone finale 51.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
      condition-matched.
- [ ] Concentrare il recovery sugli stati ed eventi intorno agli step 210-230.
- [ ] Progettare e preregistrare un meccanismo che consideri esplicitamente la
      non-regressione delle reserve, tramite reward, vincolo o criterio di
      aggiornamento dedicato.
- [ ] Non usare return o `checkpoint_best` come proxy di robustezza.
- [ ] Mantenere i seed 126-128 sigillati finché un solo candidato non supera
      l'intera matrice development.
- [ ] Non allentare soglia di 25 mm, cicli minimi, zero clipping o cap reserve
      condition-matched.
- [ ] Eseguire un nuovo pilot controllato prima di un altro run lungo.

## TODO ereditati ancora aperti

- [ ] Analizzare return e advantage separatamente per start.
- [ ] Non eseguire ulteriori dimezzamenti della learning rate o proiezioni
      sulla stessa trace/seed.
- [ ] Raccogliere dati recovery event-aligned e indipendenti, includendo la
      regione successiva al mismatch FSM.
- [ ] Separare seed di costruzione, validation e held-out.
- [ ] Riprovare `sigma=0.0075` su almeno tre seed per ogni start soltanto dopo
      un PASS completo a `sigma=0.005`.
- [ ] Estendere la validazione a trial, velocità e soggetti differenti.
- [ ] Mantenere differita una memoria ricorrente finché non emerga un limite
      sequenziale non coperto dallo stato Markov già validato.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
      TODO storico ancora aperto dal 13/6.
