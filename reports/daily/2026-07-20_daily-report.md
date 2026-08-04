# Daily Report - 2026-07-20

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-20_audit_exnovo_prescribed_reward_deployability_checkpoint_best.md`
- `reports/user/2026-07-20_corridor_morfologico_event_anchored_implementazione_validazione.md`
- `reports/user/2026-07-20_corridor_morfologico_retrospettivo_sperimentale.md`

## Sintesi

La giornata ha prodotto tre risultati complementari:

1. un audit completo del checkpoint `rl_module_best`, della sua dipendenza dai
   dati prescribed, della reward effettiva e del livello reale di
   deployabilità;
2. un nuovo Morphology Corridor causale sincronizzato agli eventi HS e TO
   accettati dalla FSM, validato con peso zero e senza interferenza sulla
   policy;
3. una variante retrospettiva completed-segment, capace di stirare o
   comprimere esattamente stance e swing fra gli eventi reali senza modificare
   le traiettorie served.

Il checkpoint produce realmente le reference protesiche in closed-loop e il
ramo PPO usa una reward ex-novo. Non si tratta però di apprendimento da pesi
casuali: il best e' logical 24, cioè 23 update dopo H0, e rimane molto vicino a
un actor derivato da teacher prescribed. Il plant di validazione resta ibrido.

Il nuovo corridor event-anchored e' validato come diagnostica causale
non-interferente, mentre la modalità completed-segment e' approvata soltanto
come shadow sperimentale. Entrambe mantengono `morphology_weight=0`; nessun
training PPO con morphology positiva e' stato validato.

## 1. Audit del checkpoint best

### Problema e strategia

I plot del rollout mostravano cinematica plausibile e coppie SEA regolari. Per
capire se il risultato fosse realmente generato dalla rete e se potesse essere
considerato deployable, l'analisi e' stata separata in:

- provenance completa H0-best;
- data flow prescribed di actor, critic, reset, plant e GRF;
- ricostruzione per-step della reward;
- ablation offline dei gruppi reward;
- audit del contratto actor e degli input necessari al deployment.

La reward e' stata ricostruita sui 500 elementi originali della policy trace,
riutilizzando `RewardConfig` e `compute_reward(...)`. La somma coincide
esattamente con il return registrato.

### Rollout analizzato

```text
Trajectory Generator/runs/rollout/
2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded/
```

| Metrica | Risultato |
| --- | ---: |
| checkpoint | `rl_module_best`, logical 24 |
| nuovi update dopo H0 | 23 |
| step | 500/500 |
| fine episodio | `episode_time_limit` |
| return | 52,4269395298 |
| clipping azione | 0 |
| penetrazione massima | 24,2174 mm |
| margine dal limite | 0,7826 mm |
| reserve massima | 493,449803 Nm |
| residuo dinamico massimo | `3,8868e-9 Nm` |
| HS / TO / cicli validi | 4 / 3 / 3 |
| eventi invalidi | 1 |

Il checkpoint best non coincide con l'endpoint dei 50 update. È il massimo
return incontrato alla logical 24 e non ha superato la selezione robusta del
pilot.

### Provenance e dipendenza prescribed

La classificazione corretta e':

| Domanda | Risposta |
| --- | --- |
| reward PPO ex-novo | sì |
| reference knee/ankle generate closed-loop | sì |
| training da pesi casuali | no |
| IK protesica letta dall'actor a runtime | no |
| actor indipendente da prior prescribed | no |
| plant interamente indipendente dai prescribed | no |
| actor eseguibile nello stack simulativo | sì |
| controller hardware validato | no |

La lineage e' actor imitativo, adaptation supervisionata, porting Markov a 35
feature, recovery, critic warm-up H0 e infine PPO ex-novo. Il teacher
precedente a H0 e' stato addestrato con 24.712 esempi, dei quali 16.000 ancore
nominali, 712 recovery e 8.000 teacher multi-start.

Il lato protesico sinistro usa contatto online e reference generate dalla
policy. I DOF biologici continuano a seguire IK prescribed e la GRF destra
resta una ExternalForce prescribed. Il risultato e' quindi una generazione
closed-loop del sottosistema protesico dentro un plant CMC-like ibrido.

### Drift H0-best

| Metrica | Valore |
| --- | ---: |
| action-mean RMSE | 0,000367987 |
| delta massimo action mean | 0,001278656 |
| KL empirica media | 0,00541658 |
| KL empirica massima | 0,03510496 |
| RMS differenza parametri actor | `3,49094e-6` |
| differenza massima parametro | `1,90997e-5` |
| logstd | bit-exact a H0 |

I pesi sono realmente cambiati, ma il comportamento osservato deriva
prevalentemente dal prior H0 e da correzioni PPO conservative.

### Contabilità esatta della reward

Configurazione determinante:

```text
reward_mode              = ex_novo
blend_tracking           = 0
blend_reference          = 0
blend_bio                = 0
morphology_weight        = 0
reserve_residual_weight  = 0
```

Credito positivo lordo:

| Termine | Somma | Quota |
| --- | ---: | ---: |
| `phase_regular` | 63,642690 | 86,54% |
| `contact_load` | 6,550903 | 8,91% |
| `phase_event_progress` | 3,100000 | 4,22% |
| `contact_support_to` | 0,247497 | 0,34% |
| altri termini positivi | 0 | 0% |

Penalità principali:

| Termine | Somma |
| --- | ---: |
| command-rate | 20,229607 |
| effort | 1,142662 |
| evento invalido | 0,100000 |
| SEA torque error/speed/accel/power | circa 0,001056 |
| penetrazione post-clip | 2,328497 |
| contact-support clawback | 2,472297 |
| swing unloading | 0,424905 |

Bilancio esatto:

```text
base dopo clip       57,804477
penalità post-clip   -5,377538
return               52,426940
```

Ablation controfattuali sulla traiettoria congelata:

| Ablation | Impatto sul return |
| --- | ---: |
| rimozione ramo diretto GRF/contact | `-1,442852` |
| rimozione detector/FSM | `-53,984470` |
| rimozione completa GRF/contact/FSM | return `0` |
| Morphology Corridor corrente | `0` esatto |

Il detector/FSM domina quindi la reward osservata. Queste ablation misurano
la contabilità sulla traiettoria corrente e non l'attribuzione dei gradienti
durante il training.

### Coppie SEA e deployabilità

La policy genera target angolari, non coppie. Fra actor e SEA intervengono
slew limiter, reference model Butterworth3 jerk-limited a 4 Hz, limiti di
velocità/accelerazione/jerk, cascade protesico, PI interno e dinamica
motore-molla. La regolarità osservata deriva dalla catena completa, dal prior
H0 e dalla forte penalità command-rate; non può essere attribuita soltanto ai
23 update PPO del best.

L'actor e' computazionalmente eseguibile, usa 35 feature e due azioni e non
richiede il critic privilegiato durante inferenza. Mancano però export
embedded/HIL validato, preprocessing congelato, benchmark worst-case,
watchdog, equivalenza host-target, sensor noise/delay/dropout, handover
sensor-only e validazione hardware. Lo stato corretto e':

```text
simulation-deployable actor candidate
```

non `hardware-deployable prosthesis controller`.

## 2. Corridor morfologico event-anchored causale

### Problema e soluzione

Il profilo storico usava una singola percentuale HS-HS e poteva confrontare la
served con una porzione cinematicamente errata quando il TO reale non
coincideva con la frazione media del ciclo.

Il nuovo contratto usa:

```text
HS             -> phase 0
TO             -> alpha = 0,6223299989
HS successivo  -> phase 1
```

Stance e swing sono temporizzati separatamente. A runtime vengono usate
soltanto informazioni causali: bootstrap nominale e, dopo cicli completi, la
mediana degli ultimi cinque cicli protesici validi.

È stato costruito un profilo AB06 event-warped da 123 cicli, con 202 punti e
TO inserito esattamente nella griglia. In modalità event-anchored un profilo
legacy viene ora rifiutato fail-fast.

### Validazione del profilo

Blocked cross-validation cronologica:

| Metrica | Knee | Ankle |
| --- | ---: | ---: |
| coverage aggregata | 99,706% | 96,744% |
| fold peggiore | 98,554% | 86,059% |
| escursione esterna p95 | 1,706° | 8,339° |
| riduzione dispersione media | 17,773% | 20,583% |
| riduzione dispersione al TO | 44,501% | 49,894% |

Il fold ankle lento supera il gate di appena 1,06 punti percentuali; la prova
non dimostra generalizzazione multi-soggetto o multi-modello.

### Replay del checkpoint best

Sul supporto comune di 451/500 step:

| Metrica | Legacy | Event-anchored causale |
| --- | ---: | ---: |
| errore medio di fase | 0,10997 cicli | 0,04620 cicli |
| errore p95 | 0,30829 | 0,09273 |
| salto massimo bound | 0,71619 rad | 0,44909 rad |
| entrambi i giunti nel corridor | 51,717% | 64,646% |
| loss cumulativa | 78,4999 | 41,9689 |

La loss sul supporto comune si riduce del 48,175%. La served reference resta
identica: cambia soltanto il contratto profilo-fase.

Nel rollout live con `morphology_weight=0`, original e event-anchored sono
identici esattamente su:

- 500 step e relativi timestamp;
- osservazioni actor;
- azioni raw e applicate;
- reference served e cinematica;
- return `52,42693952981385`;
- tre cicli validi;
- penetrazione massima `24,217431 mm`.

Il nuovo mapping e' quindi validato come diagnostica non-interferente. Non e'
stata addestrata una policy con peso morphology positivo.

## 3. Corridor completed-segment sperimentale

### Motivazione e contratto

Il mapping causale non conosce il prossimo evento. Quando un segmento reale
dura più della stima, la fase può saturare e il plot mostra plateau o apparenti
gradini del corridor. La served non presenta quel gradino.

La nuova modalità separata:

```text
event_anchored_completed_segment_experimental
```

mantiene invariata la served e, dopo che entrambi gli endpoint sono noti,
rimappa esattamente:

```text
stance [HS, TO)       -> [0, alpha)
swing  [TO, next_HS)  -> [alpha, 1)
```

Il contratto half-open evita duplicazioni ai bordi. Un journal FSM registra
timestamp fisico e conferma, mentre un ledger bufferizza i campioni e li
settla quando il segmento si chiude. Knee, ankle, served e corridor usano la
stessa convenzione di segno OpenSim; l'ankle e' positivo in dorsiflessione.

La variante rimane separata in `experimental_configs/`, con peso e hard
termination disattivati e guard contro l'attivazione accidentale.

### Replay reale

Sono stati recuperati 4 HS, 3 TO e 3 cicli completi. Le latenze di conferma
sono comprese fra 6 e 60 ms.

| Metrica ledger | Risultato |
| --- | ---: |
| stance completate | 3 |
| swing completati | 3 |
| campioni settled | 456 |
| duplicati | 0 |
| persi nei segmenti completi | 0 |
| campioni di coda incompleta | 44 |
| copertura grezza | 91,2% |
| copertura corretta bootstrap/coda | 100% |
| overflow / timestamp invalidi | 0 / 0 |

Il validatore riporta:

- zero plateau e incrementi non monotoni;
- errore di ricostruzione fase zero;
- errore dei bound zero;
- errore loss zero;
- served e tempo invariati esattamente;
- identità del segno raw PASS.

Sui 456 campioni valutabili:

| Metrica | Knee | Ankle |
| --- | ---: | ---: |
| frazione dentro il corridor | 86,184% | 76,535% |
| loss cumulativa per giunto | 50,0510 | 11,9275 |

La loss combinata media e' `0,0679588684`, ma resta diagnostica perché il peso
e' zero.

### Rollout live e limite PPO

Due tentativi live del checkpoint best sono stati terminati dal watchdog dopo
oltre 300 secondi senza heartbeat dentro OpenSim, rispettivamente agli step
176 e 22. Nessun tentativo e' stato contato come PASS.

La loss esatta diventa disponibile soltanto quando l'evento futuro chiude il
segmento. Addebitarla tutta sul solo step di chiusura cambierebbe discounting
e advantage. Per un training corretto servono transizioni bufferizzate,
riscrittura delle reward originali e GAE calcolato dopo il settlement.

## Decisioni della giornata

### Checkpoint best

- actor protesico closed-loop promettente in simulazione;
- reward corrente realmente ex-novo;
- forte prior prescribed nei pesi e plant ancora ibrido;
- nessuna qualifica hardware;
- `checkpoint_best` non promosso dal gate robusto.

### Corridor causale

```text
event_anchored: VALIDATED_DIAGNOSTIC_WEIGHT0
positive-weight PPO: NOT_VALIDATED
```

La configurazione resta:

```text
morphology_phase_mode = event_anchored
morphology_weight     = 0
reserve_residual_weight = 0
```

### Corridor completed-segment

```text
EXPERIMENTAL_SHADOW_APPROVED
ACTIVE_TRAINING_NOT_APPROVED
HARD_SAFETY_NOT_APPROVED
```

Il metodo legacy e quello causale corrente restano disponibili e non sono
stati sovrascritti.

## File modificati o aggiunti

### Runtime e configurazione del corridor

- `Trajectory Generator/prosthetic_phase_fsm.py`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py`
- `Trajectory Generator/baseline_MLP/experimental_configs/README.md`
- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_completed_segment_shadow.json`
- `Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json`

Non sono stati modificati plugin C++, semantica SEA, azioni, served reference
o schema dell'actor.

### Test e validazione

- `validation/build_event_warped_morphology_profile.py`
- `validation/test_event_anchored_morphology.py`
- `validation/validate_event_warped_morphology_profile.py`
- `validation/validate_event_anchored_morphology.py`
- `validation/plot_morphology_corridor_rollout.py`
- `validation/test_experimental_retrospective_morphology.py`
- `validation/validate_experimental_retrospective_morphology.py`
- `validation/replay_completed_segment_morphology.py`
- `validation/test_replay_completed_segment_morphology.py`
- aggiornamenti a `validation/test_reward_function.py` e
  `validation/validate_training_config.py`.

Tutti gli script di validazione introdotti risiedono dentro `validation/`.

### Documentazione

- i tre report utente consolidati in questo daily;
- nessun codice e' stato modificato dall'audit di provenance/reward in sé.

## Artefatti principali

```text
validation/event_anchored_morphology_runs/2026-07-20_profile_blocked_cv/
validation/event_anchored_morphology_runs/2026-07-20_best/
Trajectory Generator/runs/rollout/validation/event_anchored_morphology_runs/
  2026-07-20_weight0_live_rollout_final/
validation/experimental_morphology_runs/
  2026-07-20_completed_segment_real_trace_replay/
validation/experimental_morphology_runs/
  2026-07-20_completed_segment_real_trace_validation/
plot/07_20_2026_morphology_completed_segment_experimental/
  01_morphology_corridor_exact_hs_to_hs.png
```

## Test e verifiche

### Audit checkpoint

- summary, trace, STO ed eventi verificati;
- 500 reward ricostruite con differenza totale zero;
- ablation GRF/FSM e reweighting morphology completati;
- manifest actor/critic e 35/84 feature verificati;
- assenza di IK protesica diretta nell'actor verificata;
- GRF sinistra online e GRF destra prescribed verificate;
- sette plot standard presenti e nessun canale mancante.

### Event-anchored causale

- profilo ricostruito byte-exact: PASS;
- test event-anchored: `11/11` PASS;
- reward regression: `32/32` PASS;
- training config smoke: PASS;
- FSM nel vero ambiente OpenSim: PASS;
- blocked cross-validation: PASS;
- replay causal/legacy/oracle: PASS;
- rollout live peso zero e trace invariance: PASS;
- `py_compile`, Ruff e `git diff --check`: PASS.

### Completed-segment sperimentale

- ledger/FSM: `19/19` PASS;
- replay fail-closed: `4/4` PASS;
- event-anchored corrente: `11/11` PASS;
- reward esistente: `32/32` PASS;
- contratto rollout: `5/5` PASS;
- `py_compile`, Ruff, Black e `git diff --check`: PASS;
- due rollout live: FAIL watchdog, conservati come evidenza.

## TODO training e robustezza propagati

- [ ] Ripartire da H0 o da una sorgente esplicitamente preregistrata, non da
      logical 51 o dal best scelto per return.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
      reserve condition-matched.
- [ ] Concentrare recovery e raccolta dati sugli stati/eventi intorno agli
      step 210-230.
- [ ] Progettare e preregistrare la non-regressione delle reserve tramite
      reward, vincolo o criterio di aggiornamento, senza confonderla con errori
      della GRF.
- [ ] Non usare return o `checkpoint_best` come proxy di robustezza o criterio
      di promozione.
- [ ] Mantenere i seed 126-128 sigillati finché un solo candidato non supera
      tutta la matrice development.
- [ ] Non allentare 25 mm, cicli minimi, zero clipping o cap reserve.
- [ ] Eseguire un nuovo pilot controllato prima di un altro run lungo.
- [ ] Analizzare return e advantage separatamente per start.
- [ ] Non eseguire ulteriori dimezzamenti della learning rate o proiezioni
      sulla stessa trace/seed.
- [ ] Raccogliere dati recovery event-aligned e indipendenti dalla trace e dal
      seed usati per il gate.
- [ ] Riprovare `sigma=0.0075` su almeno tre seed per start soltanto dopo un
      PASS completo a `sigma=0.005`.
- [ ] Estendere la validazione a trial, velocità, soggetti e modelli diversi.
- [ ] Mantenere differita una memoria ricorrente finché non emerga un limite
      sequenziale non coperto dallo stato Markov.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo.

## TODO dell'audit ex-novo e deployment

- [ ] Confrontare H0 e logical 24 con protocollo identico, producendo overlay
      di azioni, served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed: reset sensor-based,
      perturbazioni di encoder/carico, detector HS/TO, GRF online bilaterale e
      riduzione della dipendenza dalla IK biologica.
- [ ] Eseguire training A/B da H0, modificando un solo gruppo reward per volta:
      detector/FSM, ramo GRF diretto, morphology e command-rate.
- [ ] Congelare ordine, scaling, frequenza, unità, normalizzazione BW, filtri,
      FSM, reset, governor, limiti, fallback e watchdog del contratto hardware.
- [ ] Validare export actor-only, equivalenza host-target, latenza worst-case,
      HIL e successivamente human-in-the-loop.
- [ ] Promuovere soltanto dopo multi-start, più seed, rumore/latency, orizzonte
      lungo e held-out sigillato.

## TODO del corridor event-anchored causale

- [ ] Eseguire un A/B breve da checkpoint identico con morphology weight
      `0`, `0.0025` e `0.005`, stessi start, seed, batch e learning rate.
- [ ] Conservare ogni update e schermare loss morphology, uscite dal corridor,
      cicli, penetrazione, return, SEA, reserve e drift actor.
- [ ] Definire il trattamento di `WAIT_HS` quando la morphology non e'
      disponibile, senza creare un percorso di reward evitabile.
- [ ] Decidere se esporre all'actor lo stato temporale event-anchored, usare
      timing soltanto nominale/eventi o validare la parziale osservabilità
      introdotta dalle mediane.
- [ ] Introdurre logging per-evento e partire soltanto con peso piccolo, dato
      il salto residuo di fase e dei bound.
- [ ] Definire un gate specifico per il regime ankle lento.
- [ ] Validare almeno un profilo, modello o soggetto esterno ad AB06.
- [ ] Mantenere le reserve come gate fisico e non come reward finché il ramo
      GRF non e' validato o l'origine del residuo non e' attribuibile.

## TODO del corridor completed-segment sperimentale

- [ ] Completare un rollout live shadow da 500 step in una sessione OpenSim
      pulita e ripetere il confronto A/B.
- [ ] Definire il bordo esterno phase-dependent e scegliere se la sicurezza
      debba essere immediata, causale o retrospettiva.
- [ ] Implementare il buffering complete-segment/complete-episode, riscrivere
      le reward sui campioni originali e calcolare GAE solo dopo il settlement.
- [ ] Gestire esplicitamente segmenti incompleti, timeout e bootstrap.
- [ ] Eseguire un pilot corto con peso positivo soltanto dopo i gate
      precedenti.
- [ ] Validare il pilot su start e seed held-out prima di training lunghi o
      decisioni di deployment.

## TODO storico ancora aperto

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
      TODO originato il 13/6 e non ancora chiuso.
