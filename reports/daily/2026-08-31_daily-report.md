# Daily Report - 2026-08-31

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

> Aggiornamento del 2026-10-19: `Trajectory Generator/baseline_MLP/experimental_configs/`
> è stata rimossa. Due preset sono conservati come fixture dei test; gli altri
> preset e i comandi che li selezionavano sono storici e richiedono ripristino.
> La configurazione V26C corrente resta `training_exnovo_cfg.yaml`. Vedere il
> [registro della rimozione](../cleanup/2026-10-19_experimental_configs_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Progetto della rete unificata imitativa/ex-novo e dell'actor predittivo](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md)
- [Separazione morphology knee/ankle e critic-only rewarm J20](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md)
- [Smoke test e analisi del sigma di esplorazione V26C/J20](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md)

## Sintesi

La giornata ha trasformato l'audit di latenza del 30 agosto in una specifica
architetturale per la nuova pipeline e ha completato il primo intervento
infrastrutturale: separazione dei contributi morphology knee/ankle con
retrocompatibilità e critic-only rewarm verificato sullo stesso actor J20.

La rete proposta è un unico actor deployable con lo stesso contratto di
osservazione e azione durante warm start ed ex-novo. Il warm start attiva la
reward imitativa e spegne quella ex-novo; il passaggio successivo spegne
completamente l'imitazione, crea critic e optimizer nuovi, riscalda il critic a
actor congelato e poi abilita PPO ex-novo. Le safety restano sempre attive.

L'actor non deve osservare target prescribed o segnali controlaterali non
disponibili sulla protesi. Tali informazioni possono essere usate dal teacher,
dalla reward e dal critic privilegiato. Il contratto iniziale resta l'MLP J20
`35 -> 256 -> 256 -> 2`; non è stato proposto un widening.

L'actor J20 non è ancora diventato predittivo. È stata progettata la strategia:
un teacher inverse-governor deve produrre label raw che, attraversando la
stessa catena slew+C2 del deployment, generino la served desiderata circa
`90 ms` dopo per il knee e `100 ms` dopo per l'ankle. Il successivo DAgger deve
raccogliere gli stati realmente visitati, incluso lo stato del governor.

È stato inoltre completato lo sweep frozen-policy del sigma sullo stesso actor
J20. Il valore `0.010` passa sei seed nominali e i due start `+/-0.20 s`; è la
raccomandazione per un pilot PPO breve con log-std congelata. `0.015` passa come
ceiling ma arriva a `27.5579 mm`, mentre `0.020` e `0.030` falliscono il limite
hard di `28 mm` sul seed 127.

## Architettura unificata proposta

Il contratto della reward è:

\[
R = R_{safety} + \alpha_I R_{imitation} + \alpha_E R_{exnovo}
\]

| Modalità | `alpha_I` | `alpha_E` | Safety |
|---|---:|---:|---|
| warm start | `1` | `0` | attiva |
| ex-novo | `0` | `1` | attiva |

I termini spenti possono essere calcolati in shadow logging, ma devono avere
effetto matematico nullo. Al cambio di modalità si preservano actor, ordine
delle osservazioni e semantica dell'azione; si sostituiscono critic e optimizer
perché il value function imitativo non rappresenta il nuovo ritorno ex-novo.

Le due colonne storiche `gait_phase_sin/cos` restano bit-zero per compatibilità
tensoriale; le altre 33 feature causali includono cinematica, SEA, GRF online,
FSM v3, durata ciclo, endpoint precedente e stato `q/v/a` della served.

## Actor predittivo

Un semplice phase shift fisso è stato escluso come soluzione definitiva:

- la fase di un filtro low-pass dipende dalla frequenza;
- lo slew limiter è non lineare e dipende dal precedente endpoint;
- knee e ankle hanno limiti e ritardi differenti;
- la cadenza modifica il profilo ankle;
- il raw actor possiede già una propria quota di sfasamento.

La strategia raccomandata conserva la testa a due azioni assolute. Su ogni
finestra, il teacher inverse-governor ottimizza una sequenza raw attraverso il
governor deterministico minimizzando errore served-vs-desired e regolarità del
comando, nel rispetto di slew, bounds e stato iniziale del filtro. Il primo
comando della sequenza ottima diventa la label dello student.

DAgger deve poi:

1. eseguire lo student in closed loop;
2. raccogliere gli stati realmente visitati;
3. interrogare il teacher da quegli stati;
4. aggiungere nominali e recovery al dataset;
5. aggiornare lo stesso actor 35D;
6. validare ogni round esclusivamente closed-loop.

## Separazione del morphology corridor

Il profilo AB06 resta un singolo artefatto di provenance, ma contiene già due
geometrie indipendenti, ciascuna su 202 punti event-warped:

```text
pros_knee_angle:  mean_rad + std_rad
pros_ankle_angle: mean_rad + std_rad
```

Nel contratto corrente:

| Parametro | Knee | Ankle |
|---|---:|---:|
| moltiplicatore standard deviation | `1.6` | `0.6` |
| margine | `7.5 deg` | `7.5 deg` |
| hard bound diagnostico | `[-1.4, 0.0] rad` | `[-0.6, +0.6] rad` |

La served di ciascun giunto viene confrontata esclusivamente con la propria
banda. Le loss, le escursioni, i bound correnti e gli hard excursion sono
calcolati e loggati separatamente. Restano condivisi il profilo sorgente, la
griglia di fase, la FSM v3 e il delay morphology da `40 ms`.

## Implementazione dei pesi separati

`RewardConfig` espone ora:

```yaml
morphology_knee_weight: null
morphology_ankle_weight: null
```

Un valore omesso eredita `morphology_weight`. La nuova composizione è:

\[
T_{morphology} =
\frac{w_{knee}L_{knee} + w_{ankle}L_{ankle}}{2}
\]

Con pesi uguali il codice mantiene esplicitamente la vecchia aritmetica,
ottenendo equivalenza esatta con la baseline. Un giunto a peso zero non valuta
la propria loss, prevenendo `0 * NaN`; pesi negativi o non finiti falliscono
closed.

Poiché non era stato ancora scelto un rapporto asimmetrico, è stata usata la
baseline neutra:

```text
morphology_weight       = 1.05
morphology_knee_weight  = 1.05
morphology_ankle_weight = 1.05
```

Questa configurazione verifica il nuovo plumbing senza cambiare l'obiettivo
numerico e non costituisce ancora un esperimento ankle-specific.

## Critic-only rewarm

È stato caricato l'actor J20 e creato un critic fresco con:

- actor congelato;
- `logstd` congelata;
- optimizer nuovo;
- una iterazione e `4096` step;
- learning rate `1e-4`;
- `10` epoche;
- pesi morphology espliciti `1.05/1.05`;
- FSM v3 e contratto causale invariati.

Run valido:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2
```

| Verifica | Risultato |
|---|---:|
| actor, dieci tensori | byte-identico |
| max actor diff runtime | `0.0` |
| `logstd` | byte-identica |
| sigma knee/ankle | `0.00499999967` |
| mean KL | `0.0` |
| critic digest | cambiato |
| VF loss | `0.00440714` |
| VF explained variance | `0.791205` |
| `phase_timeout_stance` | `0` |
| `morphology_causal_contract_failure` | `0` |
| `resync_count_max` | `0` |
| `hs_cancelled_count_max` | `0` |
| telemetria mancante | `0/4096` |

Il primo tentativo nella directory priva di suffisso `_r2` si è fermato prima
di Ray e prima di qualsiasi update per una restrizione macOS
`psutil/sysctl`. È un artefatto incompleto e non è un checkpoint candidato.

Il checkpoint `_r2` dimostra che il critic può essere adattato senza muovere
l'actor. Poiché `1.05/1.05` è reward-equivalente al peso globale, non dimostra
ancora l'efficacia di una maggiore pressione ankle e non contiene un actor
predittivo.

## Sweep sigma frozen-policy

Sono stati generati moduli isolati constant-sigma mantenendo la mean-network
byte-identica e senza update PPO/critic. La batteria finale comprende `35`
rollout da `500` step o fino a terminazione, common random numbers, sei seed e
tre start.

| sigma | nominale | multi-start | max penetrazione | decisione |
|---:|---:|---:|---:|---|
| `0.010` | `6/6` | `2/2` | `26.8901 mm` | pilot breve raccomandato |
| `0.015` | `6/6` | `2/2` | `27.5579 mm` | ceiling, non primo pilot |
| `0.020` | `5/6` | non avanzato | `28.2061 mm` | hard FAIL |
| `0.030` | `4/5` | non avanzato | `28.1840 mm` | hard FAIL |

Sul seed avverso 127 la penetrazione cresce da `25.6843 mm` a sigma `0.005`,
a `26.8901`, `27.5579` e `28.2061 mm` per sigma `0.010`, `0.015` e `0.020`.
La causa dei failure superiori è l'evoluzione closed-loop accumulata: clipping,
FSM, morphology contract e saturazione SEA restano puliti.

Il raddoppio `0.005 -> 0.010` quadruplica la varianza. Dopo lo slew restano in
media `0.00606/0.00604 rad` RMS su knee/ankle, cioè l'`81.1/86.0%` del rumore
fisico raw. Il pilot deve mantenere `freeze_logstd=true` e durare inizialmente
soltanto `5-10` iterazioni.

## File modificati o introdotti

- `Trajectory Generator/baseline_MLP/reward_function.py`;
- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_1p05_2026-08-31.json`;
- `validation/test_morphology_joint_weights.py`;
- `validation/audit_morphology_split_critic_rewarm.py`;
- `Trajectory Generator/baseline_MLP/validation/analyze_v26c_sigma_exploration_smoke.py`;
- `Trajectory Generator/baseline_MLP/validation/v26c_sigma_exploration_smoke_2026-08-31/`;
- `reports/user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md`;
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/morphology_joint_split_critic_rewarm_audit.json`;
- i tre report utente consolidati sopra.

## Test e verifiche

- `py_compile` sui file Python interessati: PASS;
- 14 test aggregati morphology/split: PASS;
- round-trip dei campi opzionali: PASS;
- equivalenza esatta legacy/split uguale: PASS;
- pesi asimmetrici e guardie non-finite: PASS;
- audit checkpoint: tutti i gate `true`;
- actor e `logstd` immutati, critic aggiornato;
- telemetria e contatori FSM puliti;
- 35 rollout sigma frozen-policy analizzati: PASS completo a `0.010/0.015`,
  hard FAIL riprodotto a `0.020/0.030`;
- sei seed nominali e due start perturbati a `0.010`: `8/8` PASS;
- mean-network exact e save/reload exact su `8/8` moduli sigma;
- riproduzione exact dei baseline J19B deterministico e `sigma=0.005`.

Due suite legacy complete incontrano assertion su valori della configurazione
attiva già diversi dalle aspettative storiche (`morphology_weight` e swing
timeout). I test specifici della modifica passano; il drift preesistente delle
configurazioni resta un debito separato.

## TODO completati

- [x] Formalizzare la rete unica con modalità imitativa ed ex-novo.
- [x] Distinguere feature actor deployable da segnali teacher/reward/critic.
- [x] Progettare il meccanismo generale dell'actor predittivo.
- [x] Separare loss e pesi morphology knee/ankle con retrocompatibilità.
- [x] Eseguire un critic-only rewarm neutro mantenendo J20 byte-identico.
- [x] Auditare critic, actor, sigma e salute FSM del checkpoint risultante.
- [x] Eseguire lo sweep frozen-policy del sigma e misurare l'esplorazione
  effettiva dopo lo slew/governor.
- [x] Identificare `sigma=0.010` come candidato per un pilot breve e
  `sigma=0.015` come ceiling non raccomandato.

## TODO aperti e propagati

- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.
- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.
- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.
- [ ] Implementare e verificare il cambio integrale di modalità reward,
  mantenendo safety sempre attiva e termini spenti in shadow logging.
- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.
- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.
- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.
- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.
- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.
- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.
- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.


## Addendum retrospettivo del 5 ottobre 2026 — completamento dei report di fine agosto

Questa integrazione ricostruisce le informazioni mancanti dai report originali.
Il testo precedente e la data del daily restano invariati. Esiti, test, GO,
training e rollout citati appartengono alle esecuzioni storiche: il 5 ottobre
sono stati controllati soltanto documenti e collegamenti. Le vecchie leaf,
le autorizzazioni consumate e i risultati negativi conservano il loro stato.
I TODO ancora aperti sono riportati separatamente dai passi risolti o superati.

<a id="completion-20261005-2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo"></a>

[Progetto della rete unificata imitativa/ex-novo e dell'actor predittivo](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm"></a>

[Separazione morphology knee/ankle e critic-only rewarm J20](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-31_smoke_test_sigma_esplorazione_v26c_j20"></a>

[Smoke test e analisi del sigma di esplorazione V26C/J20](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.


### TODO riconciliati dalle fonti di questa data

Il registro distingue passi storici eseguiti, piani superati, debiti aperti e
condizioni/invarianti per eventuali lavori futuri. Le chiusure qui citate sono
retrospettive: nessuna nuova autorizzazione o esecuzione deriva dall’addendum.

- **Condizionato — Definire e preregistrare un primo rapporto morphology knee/ankle realmente asimmetrico dopo il gate sul teacher; lo split neutro non sceglie il rapporto.** [fonte, riga 606](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- **Condizionato — Ripetere critic-only con actor/logstd congelati quando si introduce un nuovo rapporto morphology asimmetrico; il rewarm neutro già eseguito non soddisfa quella nuova condizione.** [fonte, riga 346](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).
- **Condizionato — Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora.** [fonte, riga 415](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- **Condizionato — Decidere qualsiasi estensione oltre 10 solo dopo risultati e gate del pilot; la proposta storica non autorizza nuove esecuzioni.** [fonte, riga 419](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- **Condizionato — Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti.** [fonte, riga 417](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- **Aperto — Definire round e composizione nominal/recovery, poi eseguire DAgger sullo stesso actor 35 con nuove label inverse-governor.** [fonte, riga 610](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- **Aperto — Preregistrare e validare frozen-policy una griglia degli orizzonti attorno a 90 ms knee/100 ms ankle su più cadenze/stati del governor.** [fonte, riga 603](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- **Aperto — Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva.** [fonte, riga 604](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- **Risolto — Verificare sigma con uno sweep dell’esplorazione effettiva dopo il governor.** [fonte, riga 347](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md). Chiusura/supersessione 2026-08-31: [Sweep sigma frozen-policy concluso, 35 rollout e gate multi-seed/multi-start.](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).

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
- [ ] Preregistrare il primo rapporto morphology knee/ankle asimmetrico. — [fonte 2026-08-31, riga 606](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Validare frozen-policy gli orizzonti circa 90 ms knee / 100 ms ankle su più cadenze e stati del governor. — [fonte 2026-08-31, riga 603](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva. — [fonte 2026-08-31, riga 604](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

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
- [ ] **Condizionato:** Ripetere il critic-only rewarm quando la reward diventa realmente asimmetrica. — [fonte 2026-08-31, riga 346](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).
- [ ] **Condizionato:** Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora. — [fonte 2026-08-31, riga 415](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Decidere eventuale estensione oltre 10 soltanto dopo rollout deterministici e gate del pilot J20. — [fonte 2026-08-31, riga 419](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti. — [fonte 2026-08-31, riga 417](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e recovery, se si riprende questa strategia. — [fonte 2026-08-31, riga 610](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Verificare sigma con uno sweep dell’esplorazione effettiva dopo il governor. — [fonte 2026-08-31, riga 347](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md). **Chiuso:** Sweep sigma frozen-policy concluso, 35 rollout e gate multi-seed/multi-start. ([evidenza](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
