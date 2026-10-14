# Daily Report - 2026-08-30

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- [Audit del corridor morfologico rispetto alle cinematiche prescribed e V26](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md)

## Sintesi

La giornata è stata dedicata a stabilire se le cinematiche prodotte dai
training ex-novo V26 fossero una conseguenza inevitabile del modello corrente
e se il morphology corridor AB06 fosse stato costruito in modo errato.

Il verdetto è che il profilo non presenta errori di soggetto, laterality,
segno, unità o mapping degli eventi. Il modello e i SEA possono seguire
cinematiche prescribed prevalentemente interne al corridor. Le curve V26
mantengono una forma generale prescribed-like, ma la realizzano troppo tardi
nel ciclo; l'errore dominante è nella catena che trasforma il target raw
dell'actor nella served reference, non nel tracking fisico served→actual.

L'aumento del peso morphology da `0.0025` a `1.05`, pari a un fattore 420, non
ha cambiato materialmente le curve. La sola moltiplicazione della penalità non
corregge sfasamento, latenza, reward delayed o scarsa esplorazione dell'actor.
Il corridor resta valido come envelope diagnostico AB06, ma la sua formulazione
come reward online deve essere resa sensibile a cadenza e attribuzione causale.

## Evidenze principali

### Fattibilità del tracking

| Esperimento | Knee RMSE | Ankle RMSE |
|---|---:|---:|
| CMC-like actual vs prescribed | `0,00310 rad` | `0,02020 rad` |
| prescribed controller recente | `0,00439 rad` | `0,00546 rad` |
| V26 peso `1.05`, actual vs served | `0,00995 rad` | `0,00881 rad` |

La simulazione prescribed completa è risultata fuori corridor nello `0%` dei
campioni knee e nel `1,99%` dei campioni ankle. Questo esclude un limite
meccanico generale come spiegazione della mancata plantarflessione V26.

### Sfasamento rispetto al prescribed

Anticipando offline le curve V26 si ottiene una correlazione di circa `0,91`
con il prescribed per entrambi i giunti:

| Giunto | Anticipo ottimo | Equivalente temporale |
|---|---:|---:|
| Knee | `0,106` cicli | circa `160 ms` |
| Ankle | `0,089` cicli | circa `135 ms` |

Vicino al TO l'ankle prescribed è circa `-0,124 rad`, mentre la served V26 è
circa `+0,128 rad`: oltre a essere tardiva, ha segno funzionale opposto nella
finestra biomeccanica di interesse.

### Effetto del peso morphology

Tra peso `0.0025` e `1.05`:

| Giunto | RMSE fra le served | Correlazione |
|---|---:|---:|
| Knee | `0,000477 rad` | `0,9999989` |
| Ankle | `0,000654 rad` | `0,9999891` |

Nel run `1.05`, il knee è fuori corridor in `10/372` campioni e l'ankle in
`62/372`. Il test ha quindi falsificato l'ipotesi che il problema dipendesse
solamente da un peso globale insufficiente.

### Dipendenza dalla cadenza

Il profilo unico aggrega 123 cicli con periodi da `0,879` a `1,669 s`. I 75
cicli veloci hanno ankle vicino al TO pari a circa `-0,318 rad`; i 22 cicli
lenti, più simili al rollout corrente, circa `-0,082 rad`. Il profilo globale è
quindi influenzato maggiormente dai cicli veloci.

La cadenza spiega parte del mismatch con la media globale, ma non assolve il
V26: la sua served a `+0,128 rad` resta distante anche dalla media lenta e con
segno funzionale errato.

## Addendum frozen-policy sulla latenza

Senza modificare policy o avviare training, sono stati confrontati quattro
stadi della catena su tutti e quattro i rollout V26:

```text
actor raw -> slew limiter -> filtro C2 -> served -> cinematica fisica
```

| Tratto | Knee | Ankle |
|---|---:|---:|
| actor raw → dopo slew | `10 ms` | `20 ms` |
| dopo slew → served | `80 ms` | `80 ms` |
| served → actual | `0 ms` | circa `0 ms` |
| actor raw → served | `90 ms` | `100 ms` |

I limiti hard di velocità, accelerazione e jerk del governor non sono
intervenuti. Gli `80 ms` principali derivano quindi dalla dinamica low-pass del
filtro C2 a `4 Hz`.

Al TO del run `1.05`:

| Segnale ankle | Valore |
|---|---:|
| prescribed | `-0,1242 rad` |
| actor raw | `-0,1276 rad` |
| dopo slew | `-0,0222 rad` |
| served | `+0,1284 rad` |
| actual | `+0,1273 rad` |

Il raw actor è quindi già vicino al prescribed in quel campione; slew e filtro
consegnano il movimento in ritardo. A questa latenza fisica si aggiungono
`40 ms` di ritardo nella consegna del campione morphology alla reward PPO. Il
delay reward non rallenta il comando, ma assegna la penalità a una transizione
successiva rispetto a quella che ha originato l'errore.

## Strategia risultante

- Conservare il corridor corrente come baseline diagnostica event-aligned.
- Non spostare il corridor nel tempo per accettare una traiettoria tardiva.
- Separare la metrica finale retrospettiva dal segnale causale apprendibile.
- Progettare un corridor condizionato su periodo, stance e swing.
- Correggere il credito della reward o valutare la served futura predetta dal
  governor.
- Separare i contributi knee/ankle prima di nuovi esperimenti sui pesi.
- Introdurre una fascia interna soft per fornire gradiente prima del bordo.
- Usare pilot preregistrati brevi da `5–10` iterazioni prima di altri run da
  50, sempre dallo stesso parent J20 per confronti controllati.

## File introdotti

- `reports/user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md`;
- `validation/audit_v26_frozen_policy_latency.py`;
- `validation/v26_reference_latency_audit/2026-08-30_frozen_policy_latency_decomposition.json`;
- `plot/08_30_2026_v26_reference_latency_audit/01_frozen_policy_latency_decomposition.png`.

Non sono stati modificati training, checkpoint, reward, configurazioni,
profilo morphology, plugin C++ o semantica SEA.

## Test e verifiche

- provenance del profilo, laterality, segni e unità verificati;
- confronto dei quattro rollout V26 con prescribed e corridor;
- event warping retrospettivo su cicli completi;
- sweep di phase shift con passo `0,001` cicli;
- stratificazione dei 123 cicli AB06 per periodo;
- bound del plot 8 ricostruiti entro `2,22e-16 rad`;
- scomposizione temporale stabile su `4/4` rollout;
- script diagnostico compilato con `py_compile`;
- JSON verificato con assertion strutturali;
- PNG `2160x1620` generato e ispezionato;
- nessun training eseguito e policy non mutata.

## TODO completati

- [x] Escludere errori di segno, unità, lato o provenance del corridor.
- [x] Dimostrare che il modello può seguire il prescribed.
- [x] Quantificare separatamente actor→slew, slew→served e served→actual.
- [x] Verificare che il solo aumento del peso globale non corregge il toe-off.
- [x] Identificare la dipendenza del profilo morphology dalla cadenza.

## TODO aperti e propagati

- [ ] Progettare un segnale predittivo o una riassegnazione causale della loss
  che compensi la catena actor→served senza spostare il gate finale.
- [ ] Separare i pesi morphology knee e ankle e conservare logging per giunto.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Valutare una fascia soft interna al corridor.
- [ ] Preregistrare ogni nuovo pilot con stop rule, rollback e confronto dallo
  stesso checkpoint J20.
- [ ] Rivalutare sigma: `0,005` non deve essere ereditata automaticamente.
- [ ] Monitorare sempre penetrazione con soglie `20 mm` soft, `25 mm`
  diagnostica e `28 mm` hard, insieme ai contatori FSM v3.
- [ ] Mantenere separate served reference e cinematica fisica nelle metriche.
- [ ] Generalizzazione multimodello con dataset EPIC e soggetto held-out.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e richiedono approvazione
  esplicita prima dell'integrazione.
- [ ] Conservare il sidecar J8 stantio e gli artefatti storici senza modifiche
  retroattive.
- [ ] Debiti tecnici/amministrativi ancora aperti: osservazioni non gateate R3,
  marker `RESTORE_AUDIT_PENDING`, processi storici orfani, semantica
  `cycle_valid`, osservabilità RLlib, swing timeout, probe schema, pin V20,
  `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.


## Addendum retrospettivo del 5 ottobre 2026 — completamento dei report di fine agosto

Questa integrazione ricostruisce le informazioni mancanti dai report originali.
Il testo precedente e la data del daily restano invariati. Esiti, test, GO,
training e rollout citati appartengono alle esecuzioni storiche: il 5 ottobre
sono stati controllati soltanto documenti e collegamenti. Le vecchie leaf,
le autorizzazioni consumate e i risultati negativi conservano il loro stato.
I TODO ancora aperti sono riportati separatamente dai passi risolti o superati.

<a id="completion-20261005-2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26"></a>

[Audit del corridor morfologico rispetto alle cinematiche prescribed e V26](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.


### TODO riconciliati dalle fonti di questa data

Il registro distingue passi storici eseguiti, piani superati, debiti aperti e
condizioni/invarianti per eventuali lavori futuri. Le chiusure qui citate sono
retrospettive: nessuna nuova autorizzazione o esecuzione deriva dall’addendum.

- **Aperto — Progettare e validare su cicli held-out un corridor condizionato a cadenza/periodo/stance/swing, senza allargarlo o spostarlo per accettare la V26.** [fonte, riga 346](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Aperto — Valutare fascia soft interna e attribuzione del costo morphology alla decisione originaria o alla served futura predetta, mantenendo immutato il gate diagnostico finale.** [fonte, riga 360](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Condizionato — Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi.** [fonte, riga 358](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Risolto — Misurare a policy congelata actor→served e served→actual prima di nuovo training.** [fonte, riga 372](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md). Chiusura/supersessione 2026-08-30: [Addendum Fase D misura a policy congelata lag raw→slew→served→actual; quattro rollout concordi.](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Condizionato — Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica.** [fonte, riga 374](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Aperto — Definire/implementare il teacher inverse-governor e generatore di label raw anticipati coerenti con slew e C2.** [fonte, riga 392](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Aperto — Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa.** [fonte, riga 376](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Condizionato — Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo.** [fonte, riga 394](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- **Condizionato — Solo dopo audit/teacher/contratto causale, preregistrare un pilot breve 5–10 da J20 con confronto controllato prima di nuovo run lungo.** [fonte, riga 388](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).
- [ ] Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).
- [ ] Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- [ ] Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- [ ] Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).
- [ ] Progettare e validare corridor cadence-conditioned su dati held-out. — [fonte 2026-08-30, riga 346](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Valutare fascia soft interna e assegnazione causale del credito senza spostare il corridor finale. — [fonte 2026-08-30, riga 360](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa. — [fonte 2026-08-30, riga 376](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).
- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).
- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).
- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).
- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).
- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).
- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).
- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).
- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).
- [ ] **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).
- [ ] **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- [ ] **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- [ ] **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).
- [ ] **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).
- [ ] **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- [ ] **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi. — [fonte 2026-08-30, riga 358](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica. — [fonte 2026-08-30, riga 374](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si riprende questa strategia, implementare teacher inverse-governor e label raw da stato effettivamente visitato; validare il beneficio closed-loop. Pista distinta dal RL senza teacher. — [fonte 2026-08-30, riga 392](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo. — [fonte 2026-08-30, riga 394](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare pilot PPO separato 5–10 update, sigma 0,010 isotropico e logstd congelato, conservando J20/canonico; non sostituirlo con pilot di altra ricetta. — [fonte 2026-08-30, riga 388](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Misurare a policy congelata actor→served e served→actual prima di nuovo training. — [fonte 2026-08-30, riga 372](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md). **Chiuso:** Addendum Fase D misura a policy congelata lag raw→slew→served→actual; quattro rollout concordi. ([evidenza](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
