# Daily Report - 2026-08-27

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

### J9R1–J12: chiusura della matrice multistart

- [J9R1 — qualifica closed-loop](../user/2026-08-27_qualifica_closed_loop_j9r1.md)
- [J10 — readiness teacher multistart](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md)
- [J10R1 — collection teacher multistart](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md)
- [J10R1 — readiness teacher multistart](../user/2026-08-27_v26c_j10r1_multistart_teacher_readiness.md)
- [J11 — esecuzione fit multistart](../user/2026-08-27_v26c_j11_multistart_fit_execution.md)
- [J11 — readiness fit multistart](../user/2026-08-27_v26c_j11_multistart_fit_readiness.md)
- [J12 — esecuzione qualifica closed-loop](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md)
- [J12 — readiness qualifica closed-loop](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md)

### J13–J17: diagnosi del mismatch e fallimento del refit fresh

- [J13 — diagnosi J12 e piano recovery](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md)
- [J14 — readiness dataset DAgger post-mismatch](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md)
- [J14 readiness rev1 — correzione audit](../user/2026-08-27_v26c_j14_readiness_rev1_audit_correction.md)
- [J14 readiness rev2 — correzione ermeticità](../user/2026-08-27_v26c_j14_readiness_rev2_hermeticity_correction.md)
- [J14 readiness rev3 — bytecode e path-open](../user/2026-08-27_v26c_j14_readiness_rev3_bytecode_and_pathopen.md)
- [J14 — esecuzione dataset DAgger post-mismatch](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md)
- [J15 — readiness fresh refit](../user/2026-08-27_v26c_j15_fresh_refit_readiness.md)
- [J15 — esecuzione fresh refit](../user/2026-08-27_v26c_j15_fresh_refit_execution.md)
- [J15R1 — readiness dopo correzione runner](../user/2026-08-27_v26c_j15r1_runner_correction_readiness.md)
- [J15R1 — esecuzione fresh refit](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md)
- [J15R1 — addendum sul mean shift nominale](../user/2026-08-27_v26c_j15r1_nominal_mean_shift_addendum.md)
- [J16 — readiness riqualifica J15R1](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md)
- [J16 — esecuzione riqualifica J15R1](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md)
- [J17 — diagnosi causale della regressione J16](../user/2026-08-27_v26c_j17_diagnosi_causale_regressione_j16.md)
- [J17 — addendum audit dell'architetto](../user/2026-08-27_v26c_j17_architect_audit_addendum.md)
- [J17 — correzione del weighting](../user/2026-08-27_v26c_j17_architect_audit_weighting_correction.md)

### J18–J19: update vincolato e qualifica finale dell'attore

- [J18 — preregistrazione update B-only](../user/2026-08-27_v26c_j18_prereg_update_b_only.md)
- [J18 — addendum v2](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md)
- [J18 — addendum v3 e correzioni dell'architetto](../user/2026-08-27_v26c_j18_addendum_v3_correzioni_architetto.md)
- [J18 — esecuzione update B-only](../user/2026-08-27_v26c_j18_b_only_update_execution.md)
- [J18 — ricalibrazione read-only e selezione](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md)
- [J19A — readiness riproduzione singola](../user/2026-08-27_v26c_j19a_readiness_riproduzione_singola.md)
- [J19A — esecuzione riproduzione singola](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md)
- [J19B — readiness qualifica closed-loop](../user/2026-08-27_v26c_j19b_readiness_qualificazione_closed_loop.md)
- [J19B — esecuzione qualifica closed-loop](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)
- [J19C — readiness held-out G–I](../user/2026-08-27_v26c_j19c_readiness_heldout_g_i.md)
- [J19C — esecuzione held-out G–I](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md)

### J20: critic-only, probe del gradiente e restore audit

- [J20 — precedente luglio sul probe del gradiente](../user/2026-08-27_v26c_j20_precedente_luglio_probe_gradiente.md)
- [J20 — probe K1](../user/2026-08-27_v26c_j20_k1_probe.md)
- [J20 — emendamento K1R1](../user/2026-08-27_v26c_j20_k1r1_prereg_amendment.md)
- [J20 — esecuzione K1R1](../user/2026-08-27_v26c_j20_k1r1_execution.md)
- [J20 — readiness critic warm-up](../user/2026-08-27_v26c_j20_readiness_critic_warmup.md)
- [J20 — preregistrazione esecuzione critic warm-up](../user/2026-08-27_v26c_j20_critic_warmup_execution_prereg.md)
- [J20 — esecuzione critic warm-up](../user/2026-08-27_v26c_j20_critic_warmup_execution.md)
- [J20 — preparazione restore audit](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md)
- [J20 — esecuzione restore audit R1](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md)
- [J20 — preparazione restore audit R2](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md)

## Sintesi

La giornata ha portato lo student 35D dalla qualifica incompleta di J9R1 a un attore robusto J19A e poi a un checkpoint actor+critic J20, fermandosi correttamente prima dell'attestazione training-ready perché il restore audit G9 era ancora aperto.

J9R1 aveva superato **5/6 celle**: lo start B (−0,20) restava in `WAIT_HS` senza contatto utile. J10R1 ha raccolto teacher multistart per B/C e J11 ha eseguito un fit fresh da J2 su **24.713 righe**, superando i gate offline. J12 ha però chiuso solo **4/6**: B è stato recuperato, E ha completato un solo ciclo e F è terminato allo step 354 con swing timeout/HS cancellation. Questo ha dimostrato il covariate shift closed-loop: gli stati visitati autonomamente dallo student dopo il mismatch non erano rappresentati adeguatamente nel dataset.

J14 ha raccolto 854 righe post/pre mismatch e J15R1 ha rifatto il fit fresh su 25.567 righe, ma J16 è collassato a **0/6**. J17 ha identificato la causa: J14 valeva solo il 3,34% delle righe ma circa l'84,81% della massa di gradiente; ripartire da J2 violava inoltre il presupposto on-policy del DAgger per quegli stati. La soluzione architetturale è stata quindi un update funzionalmente vincolato sul miglior attore già closed-loop, **J8**, non un altro fit fresh e non un widening.

J18 ha costruito quattro blocchi disgiunti per 4.221 righe e una griglia di 16 candidati. Il gate iniziale di drift massimo 0,005 non ha prodotto survivor; una ricalibrazione read-only, basata sui margini operativi di J8, ha selezionato `c13` (`lambda=30`, `beta=1`, `lr=5e-5`) senza riaddestrare. J19A ha riprodotto esattamente il candidato, J19B ha superato **6/6 A–F** e J19C **3/3 G–I held-out**, con tutti i contatori FSM/morphology a zero e forme cinematiche adeguate. Le celle F e H sono entrate nella banda diagnostica ≥25 mm, ma nessuna ha superato 28 mm.

J20 ha poi affrontato il critic. Il primo K1 è fallito 28/29 perché un input sintetico tutto zero rendeva nullo il gradiente dei pesi del primo layer; K1R1 ha usato un batch 8×84 non degenere ed è passato **12/12**. Il warm-up critic-only di una iterazione/4096 step ha lasciato actor e logstd esattamente invariati e ha modificato il critic. Il primo restore audit ha però fallito per timeout/import del child dovuto al `PYTHONPATH`; R2 è stato preparato, ma a fine giornata **G9 era ancora aperto e `training_ready` restava false**.

## Lavoro svolto

- Chiuso il supporto multistart con teacher B/C e valutato J11/J12.
- Raccolto DAgger post-mismatch e diagnosticato il fallimento del refit fresh J15/J16.
- Progettato J18 come update limitato e funzionalmente preservante sul parent J8.
- Riprodotto e promosso l'attore J19A; completate qualifica A–F e held-out G–I.
- Corretto il probe del gradiente critic e svolto un warm-up critic-only minimo.
- Preparato ed eseguito un restore audit fail-closed; preparata la correzione R2.
- Nessun PPO actor-updating, ex-novo o deployment avviato.

## Test e verifiche

- J9R1: **5/6**; J12: **4/6**; J16: **0/6**, tutti conservati come evidenza negativa.
- J19A: 11/11 gate offline; actor digest `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96`.
- J19B: **6/6 A–F**; J19C: **3/3 G–I held-out**; `phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`, `resync_count` e `hs_cancelled_count` tutti zero.
- K1R1: **12/12** con gradiente critic non degenere.
- J20 critic-only: una iterazione, 4096 step; actor/logstd invariati, critic modificato; verifica comportamentale 12/12.
- Restore audit R1: FAIL_CLOSED tecnico; nessuna promozione e nessuna dichiarazione training-ready.

## TODO completati

- [x] Correggere il DAgger affinché consideri gli stati post-mismatch visitati dallo student.
- [x] Rigettare il refit fresh quando distrugge il comportamento closed-loop preservato.
- [x] Ottenere un attore 35D con qualifica **6/6 + 3/3 held-out**.
- [x] Dimostrare un gradiente critic non degenere e completare il warm-up critic-only senza muovere l'attore.
- [x] Misurare sigma nel lineage corrente: i seed 123–125 hanno qualificato il valore circa 0,005 per questa fase.

## TODO aperti e propagati

- [ ] Eseguire il restore audit R2/R3 fino a un verdetto valido; G9 resta aperto e `training_ready=false` finché il restore reale non è attestato.
- [ ] Solo dopo G9, progettare un'attestazione separata che aggregi le evidenze senza addestrare.
- [ ] Nessun pilot PPO/ex-novo senza nuova preregistrazione, GO e autorizzazione esplicita dell'utente.
- [ ] Monitorare nel futuro pilot sigma e penetrazione con bande 20 mm soft, 25 mm diagnostica e 28 mm hard.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri, non parte della baseline corrente.
- [ ] Generalizzazione multimodello EPIC.
- [ ] Decidere sui due `gcs_server` orfani del 18/08 e conservare i debiti storici di provenance/test senza riscrivere gli artefatti.
- [ ] Debiti tecnici/amministrativi ereditati: semantica `cycle_valid`, osservabilità RLlib, swing timeout, probe schema e attività amministrative storiche.



## Addendum retrospettivo del 5 ottobre 2026 — completamento dei report di fine agosto

Questa integrazione ricostruisce le informazioni mancanti dai report originali.
Il testo precedente e la data del daily restano invariati. Esiti, test, GO,
training e rollout citati appartengono alle esecuzioni storiche: il 5 ottobre
sono stati controllati soltanto documenti e collegamenti. Le vecchie leaf,
le autorizzazioni consumate e i risultati negativi conservano il loro stato.
I TODO ancora aperti sono riportati separatamente dai passi risolti o superati.

<a id="completion-20261005-2026-08-27_qualifica_closed_loop_j9r1"></a>

### Qualifica closed-loop J9R1 dell'attore J8

Fonte: [2026-08-27_qualifica_closed_loop_j9r1.md](../user/2026-08-27_qualifica_closed_loop_j9r1.md).

J9 fallì per mancato salvataggio degli output; J9R1 corregge la sola strumentazione e qualifica J8 con FAIL 5/6: start B senza cicli in WAIT_HS. Restano debiti sui receipt e sul detector legacy. La base del rollout riceve ora una radice di output effettiva, con `record_outputs=True` e `save_outputs_on_close=True`: l’arresto del predecessore J9 era tecnico, mentre J9R1 produce una qualifica scientificamente negativa e telemetria 6/6 valida. Suite J9R1 360 check e regressioni 288 passate. Restano il receipt con path di staging/ambiguità `valid_cycle_count`, le unità rad/secondi nei record e i quattro tocchi di avampiede B etichettati heel strike dal canale legacy a 20 N; non influenzarono FSM o verdetto. J8 resta storicamente 5/6: la successiva qualifica J19B riguarda un altro actor. Artefatti pertinenti del report: `v26c_j9_technical_failure_2026-08-26.json`, `v26c_penetration_contract_2026-08-26.json`, `v26c_j9r1_closed_loop_authorization.json`.

<a id="completion-20261005-2026-08-27_v26c_j10_multistart_teacher_readiness"></a>

### V26C J10 — raccolta teacher multistart, readiness

Fonte: [2026-08-27_v26c_j10_multistart_teacher_readiness.md](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).

J10 prepara il teacher B/C per il gap WAIT_HS di J9R1; rileva manifest a 39 feature contro runtime a 35 e corregge la fonte dei nomi. Readiness verificata, nessuna raccolta ancora avviata. I nomi e gli indici vengono derivati dal runtime 35D anziché dal manifest 39D ereditato; il preflight deve verificare lo schema effettivamente consumato. La readiness supera 158 check, senza eseguire teacher collection. Il fit e le proporzioni di tiling sono ancora fasi separate. LOTO/LOCO/B1R1/B1R2 ed EPIC rimangono futuri; i difetti storici su unità, receipt e canale legacy non vengono corretti nei leaf congelati. Artefatti pertinenti del report: `v26c_j10_prereg_multistart_teacher.json`, `v26c_j10_multistart_teacher.py`, `test_v26c_j10_multistart_teacher.py`.

<a id="completion-20261005-2026-08-27_v26c_j10r1_multistart_teacher_collection"></a>

### V26C J10R1 — Raccolta multistart prescribed-teacher: esecuzione e audit

Fonte: [2026-08-27_v26c_j10r1_multistart_teacher_collection.md](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

J10R1 raccoglie B/C per 500 step: PASS 2/2 e 20 righe WAIT_HS, tutte da B. J10 era stato rifiutato prima della raccolta; resta aperta la decisione di archiviare quel bundle. La raccolta usa il prescribed come teacher, non una policy interrogata per le label, e conserva la lineage agosto. Suite 357 e commit 46/46 passati. Il coverage WAIT_HS è asimmetrico: 20 righe uniche soltanto B, senza soglia inventata sulla sufficienza. La precedente readiness J10 era stata rifiutata prima dell’esecuzione; il suo bundle resta evidenza immutabile e la decisione di archiviarlo non risulta chiusa. J11 eseguito dopo questa fase risolve il passo di fit, non il limite statistico delle 20 righe. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j10r1_multistart_teacher_authorization.json`, `v26c_j10r1_multistart_teacher_receipt.json`.

<a id="completion-20261005-2026-08-27_v26c_j10r1_multistart_teacher_readiness"></a>

### V26C J10R1 — Multistart prescribed-teacher collection: readiness

Fonte: [2026-08-27_v26c_j10r1_multistart_teacher_readiness.md](../user/2026-08-27_v26c_j10r1_multistart_teacher_readiness.md).

J10 era stato rifiutato per difetti di coverage, hash, inventario e commit. J10R1 corregge cinque aspetti della pipeline di evidenza, preservando scienza e matrice B/C; suite e preflight passano. Il successore corregge la fonte delle feature, il controllo dell’inventario degli output, i pin/hash e la verifica del commit, evitando che un file semplicemente presente valga come evidenza valida. Scienza e matrice B/C restano invariate. Suite 357/preflight passati precedono la raccolta J10R1 documentata separatamente. Il rigetto J10 è un esito di readiness, non una raccolta fallita; il vecchio bundle non è stato eseguito o riscritto. Artefatti pertinenti del report: `v26c_j10_prereg_multistart_teacher.json`, `v26c_j10_multistart_teacher.py`, `test_v26c_j10_multistart_teacher.py`.

<a id="completion-20261005-2026-08-27_v26c_j11_multistart_fit_execution"></a>

### V26C J11 — Multistart offline fit: esecuzione e audit

Fonte: [2026-08-27_v26c_j11_multistart_fit_execution.md](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).

J11 rifitta da J2 su 24.713 righe: PASS 16/16 offline e miglioramento B/C, con critic e logstd invariati. Restano compromesso recovery, nominal_mean_shift=0.0264, repeat 8 senza razionale e coverage WAIT_HS limitata. Il miglioramento offline dei blocchi B/C non basta: recovery è 0,59% peggiore di J8 e `nominal_mean_shift=0.0264` supera il riferimento July 0.005, mantenuto non vincolante per decisione esplicita. Suite 195 e audit indipendenti passati. Le 160 righe WAIT_HS aggregate sono 20 uniche ripetute otto volte; repeat 8 non ha un razionale documentato. Rimangono le discrepanze July fra label raw/mean, gate `ok:false` e numeri di riga; il successivo J12 conserva il fallimento closed-loop. Artefatti pertinenti del report: `v26c_j11_multistart_fit_receipt.json`, `v26c_j11_aggregate_dataset.npz`, `module_state.pkl`.

<a id="completion-20261005-2026-08-27_v26c_j11_multistart_fit_readiness"></a>

### V26C J11 — Multistart offline fit: readiness

Fonte: [2026-08-27_v26c_j11_multistart_fit_readiness.md](../user/2026-08-27_v26c_j11_multistart_fit_readiness.md).

La readiness J11 definisce il fresh fit da J2 e la composizione di 24.713 righe con tiling nominale/multistart; corregge lineage e manifest. Repeat 8 e discrepanze July rimangono debiti dichiarati. L’aggregato combina nominale 500×32, recovery 713 e multistart B/C 500×8 ciascuno; tiling e boundary devono essere verificati contro gli input pinnati. Il fit fresh mantiene scaling fisico, critic escluso e logstd invariata. Suite 195/preflight passati preparano l’esecuzione J11, senza certificarne il cammino. Repeat 8, coverage WAIT_HS asimmetrica e discrepanze documentali July rimangono debiti, distinti dal passo di fit successivamente concluso. Artefatti pertinenti del report: `v26c_j11_prereg_multistart_fit.json`, `v26c_j11_multistart_fit.py`, `test_v26c_j11_multistart_fit.py`.

<a id="completion-20261005-2026-08-27_v26c_j12_closed_loop_qualification_execution"></a>

### V26C J12 — Qualifica closed-loop dell'attore J11: esecuzione e audit

Fonte: [2026-08-27_v26c_j12_closed_loop_qualification_execution.md](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md).

J12 è tecnicamente valido ma fallisce 2 celle: 4/6, B recuperata, E con un ciclo e F ferma a 354 step per timeout/cancellazione HS. Il PASS offline J11 non certifica robustezza closed-loop. Tutte le sei telemetrie sono valide: il FAIL 4/6 riguarda comportamento, non raccolta degli output. J8 e J11 falliscono celle complementari e l’amplificazione della varianza E/F blocca l’avanzamento. Il warning sullo shift nominale di J11 trova riscontro closed-loop; non viene trasformato retroattivamente in gate. Repeat 8 e WAIT_HS senza soglia rimangono osservazioni da valutare. La sostituzione operativa con J8/J19 non rende positiva la qualifica del vecchio J11. Artefatti pertinenti del report: `v26c_j12_architect_go_2026-08-27.json`, `v26c_j12_closed_loop_authorization.json`, `v26c_j12_closed_loop.py`.

<a id="completion-20261005-2026-08-27_v26c_j12_closed_loop_qualification_readiness"></a>

### V26C J12 — Qualifica closed-loop post-J11: readiness

Fonte: [2026-08-27_v26c_j12_closed_loop_qualification_readiness.md](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).

J12 r2 corregge otto etichette e la falsa multistart_disambiguation, preservando matrice A–F e soglie. Qualifica separata dal PASS offline, con rischi nominali/recovery dichiarati. La revisione r2 corregge otto etichette dopo il mancato assenso alla r1, compresa l’affermazione di multistart_disambiguation non dimostrata. Suite 410/preflight passati verificano una readiness a scienza invariata; J12 esegue poi la matrice A–F. Il margine recovery, lo shift nominale e i difetti documentali J9R1 restano evidenza storica. Un PASS offline o di readiness non autorizza warm-up/PPO e non anticipa il gate comportamentale. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j12_prereg_closed_loop_qualification.json`, `v26c_j12_closed_loop.py`.

<a id="completion-20261005-2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan"></a>

### V26C J13 — Diagnosi causale del FAIL J12 e progetto della correzione minima

Fonte: [2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).

J13 identifica la divergenza delle medie actor già al primo step con rumore identico e il gap di supporto degli stati post-mismatch. Propone DAgger senza troncamento; osserva swing clock non azzerato su hs_bounce_cancelled. Il confronto appaiato del primo step scagiona il rumore: a input e rumore identici cambiano le medie dell’actor. La policy visita stati post-mismatch non rappresentati nel training; la proposta include tale regione senza fermarsi al primo errore FSM. È un audit delle trace, senza fit né nuova simulazione. La successiva J14 corregge anche l’erronea esclusione di F/seed 125. Restano il repeat 8 senza razionale, il supporto fragile di WAIT_HS e l’osservazione del clock swing non azzerato su `hs_bounce_cancelled`, da indagare senza assumere un fix già fatto.

<a id="completion-20261005-2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution"></a>

### V26C J14 — Esecuzione: dataset DAgger post-mismatch materializzato

Fonte: [2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md).

J14 materializza 854 righe, 671 post-mismatch e 183 pre-mismatch, senza troncamento e con repeat 1. Corregge il digest autoriferito del GO; una seconda invocazione è respinta senza modifiche. Il GO additivo corregge il digest full-root che includeva il proprio documento, usando un ambito non autoriferito; corregge anche la provenienza degli hash. Suite ermetica 230 e commit passati. Una materializzazione riuscita non equivale a due dataset: la seconda invocazione è respinta fail-closed senza scritture. Le label open-loop sugli stati student-visited restano un’assunzione da arbitrare nel gate closed-loop, non una certificazione autonoma della qualità fisica. Artefatti pertinenti del report: `v26c_j14_architect_go_2026-08-27.json`, `v26c_j14_dagger_dataset_authorization_rev3.json`, `v26c_j14_dagger_dataset.py`.

<a id="completion-20261005-2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness"></a>

### V26C J14 — Dataset DAgger post-mismatch: readiness

Fonte: [2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).

La readiness J14 corregge il falso held-out seed 125/F, include E/F e riserva seed 126–128. Propone 854 righe senza troncamento; le label open-loop e lo swing clock restano limiti dichiarati. F/seed 125 non era più un held-out, essendo già usato nella diagnosi; J14 include E/F e riserva nuovi seed 126–128 al gate finale nominale. L’incremento ha 671 righe post-mismatch e 183 pre-mismatch, queste ultime mantenute per non troncare. Readiness 169/preflight passati. Repeat 1 sulle nuove righe è esplicito; label open-loop, frazione phase-invalid senza soglia e swing clock dopo HS cancellato rimangono limiti diagnostici. A–F è regressione sui failure mode noti, distinta da G–I. Artefatti pertinenti del report: `v26c_j14_prereg_dagger_dataset.json`, `v26c_j14_dagger_dataset.py`, `test_v26c_j14_dagger_dataset.py`.

<a id="completion-20261005-2026-08-27_v26c_j14_readiness_rev1_audit_correction"></a>

### V26C J14 — Correzione di readiness rev1 dopo audit Codex

Fonte: [2026-08-27_v26c_j14_readiness_rev1_audit_correction.md](../user/2026-08-27_v26c_j14_readiness_rev1_audit_correction.md).

J14 rev1 corregge la lettura dell’aggregato J11 senza pin: aggiunge pin e rehash nel punto di lettura. La suite passa 190 check e la rev0 viene superata. Il NO-GO rileva un aggregato J11 letto senza pin. Rev1 aggiunge il pin e ricontrolla l’hash nel punto di consumo, senza cambiare dataset o fit; suite 190 e 24/24 pin passati. La rev0 viene superata, non riscritta. Il problema riguarda la catena di evidenza degli input: un successivo dataset corretto non rende valida la vecchia lettura non pinnata. La race del primo test è dichiarata nel report. Artefatti pertinenti del report: `v26c_j11_aggregate_dataset.npz`, `v26c_j11_multistart_fit_receipt.json`, `v26c_j14_prereg_dagger_dataset.json`.

<a id="completion-20261005-2026-08-27_v26c_j14_readiness_rev2_hermeticity_correction"></a>

### V26C J14 — Correzione di ermeticità rev2 dopo audit Codex

Fonte: [2026-08-27_v26c_j14_readiness_rev2_hermeticity_correction.md](../user/2026-08-27_v26c_j14_readiness_rev2_hermeticity_correction.md).

La suite J14 rev1 mutava artefatti congelati reali nei test negativi. Rev2 sposta i test su mirror temporanei e introduce una guardia fail-closed; runner invariato. Tre test negativi della rev1 potevano modificare artefatti congelati reali. Rev2 li sposta su mirror temporanei e aggiunge guardie di scrittura fail-closed; il runner scientifico resta invariato. I controlli del report verificano l’ermeticità del test, non una nuova raccolta DAgger. La rev1 rimane evidenza storica del difetto; il successo della suite corretta non nasconde le mutazioni temporanee precedenti. Artefatti pertinenti del report: `v26c_j14_prereg_dagger_dataset.json`, `v26c_j14_dagger_dataset_authorization.json`, `v26c_j14_prereg_dagger_dataset_rev1.json`.

<a id="completion-20261005-2026-08-27_v26c_j14_readiness_rev3_bytecode_and_pathopen"></a>

### V26C J14 — Correzione rev3: bytecode all'import e `Path.open` non intercettata

Fonte: [2026-08-27_v26c_j14_readiness_rev3_bytecode_and_pathopen.md](../user/2026-08-27_v26c_j14_readiness_rev3_bytecode_and_pathopen.md).

J14 rev3 impedisce scritture bytecode prima degli import e intercetta Path.open in scrittura. Snapshot inclusivo di pycache, 230 test e 631 file invariati verificano l’ermeticità. `sys.dont_write_bytecode` viene impostato prima dell’import del runner; la guardia intercetta anche `Path.open` in modalità di scrittura e lo snapshot include `__pycache__`. La verifica passa 230 check e conferma 631 file invariati. Sono due correzioni della superficie I/O della suite, con scienza e dataset invariati; nessun PASS di training viene dedotto da questi test. L’ulteriore correzione del digest autoriferito è nello stadio di esecuzione J14. Artefatti pertinenti del report: `v26c_j14_prereg_dagger_dataset.json`, `v26c_j14_dagger_dataset_authorization.json`, `v26c_j14_prereg_dagger_dataset_rev1.json`.

<a id="completion-20261005-2026-08-27_v26c_j15_fresh_refit_execution"></a>

### V26C J15 — Esecuzione: il fit è FALLITO, fail-closed

Fonte: [2026-08-27_v26c_j15_fresh_refit_execution.md](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).

J15 si arresta in 0.062 s per NameError verify_warm_start_source, prima di torch/dati/fit. Non produce leaf né pesi; il GO è consumato e occorrono runner corretto e nuova autorizzazione. La funzione `run_fit` riusata dal modulo J11 cercava `verify_warm_start_source` come nome globale nudo, assente nel namespace J15. L’arresto precede import torch, dati e fit: nessuna leaf o peso prodotto. L’esito tecnico resta immutabile, come il GO già consumato. La proposta di correggere il binding e aggiungere controlli statici è realizzata nella readiness J15R1, con nuovo GO; questo errore è distinto dal successivo collasso scientifico J16. Artefatti pertinenti del report: `v26c_j15_prereg_fresh_refit.json`, `v26c_j15_fresh_refit.py`, `test_v26c_j15_fresh_refit.py`.

<a id="completion-20261005-2026-08-27_v26c_j15_fresh_refit_readiness"></a>

### V26C J15 — Readiness: refit fresco 35D post-mismatch

Fonte: [2026-08-27_v26c_j15_fresh_refit_readiness.md](../user/2026-08-27_v26c_j15_fresh_refit_readiness.md).

La readiness J15 definisce fresh fit da J2 su 25.567 righe con 17 gate e iperparametri J11 invariati. Il rischio di deriva nominale è dichiarato; la suite non intercetta il successivo difetto di namespace. Sono preregistrati 17 gate e il fit fresh da J2, con iperparametri J11 invariati e held-out sigillato. Suite 208 e preflight passati: il rischio di shift nominale è dichiarato, ma la suite preparatoria non copriva il difetto di namespace poi manifestato nell’esecuzione J15. Il passo di fit viene ripreso solo con il runner corretto J15R1; la readiness non autorizza direttamente la qualifica A–F né PPO. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j11_aggregate_dataset.npz`, `v26c_j15_prereg_fresh_refit.json`.

<a id="completion-20261005-2026-08-27_v26c_j15r1_fresh_refit_execution"></a>

### V26C J15R1 — Esecuzione: refit fresco 35D post-mismatch, PASS supervisionato

Fonte: [2026-08-27_v26c_j15r1_fresh_refit_execution.md](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).

J15R1 completa il fit con PASS 17/17 offline. L’errore correttivo diminuisce, ma il nominale peggiora di 2.89× rispetto a J11; il runner dichiara nominal_mean_shift senza misurarlo. Il fit termina in 83,4 s e riduce l’errore delle righe correttive del 60,5%, ma il nominale peggiora di 2,89× rispetto a J11. I 17 gate e la commit verification passano: è un esito offline con actor nuovo, senza rollout. `nominal_mean_shift` dichiarato nel runner non è misurato; l’addendum seguente colma la misura esternamente e lascia aperta la decisione di correggere il codice. J16 verifica successivamente la regressione e fallisce. Artefatti pertinenti del report: `v26c_j15_aggregate_dataset.npz`, `v26c_j15_fresh_refit_receipt.json`, `module_state.pkl`.

<a id="completion-20261005-2026-08-27_v26c_j15r1_nominal_mean_shift_addendum"></a>

### V26C J15R1 — Addendum diagnostico: `nominal_mean_shift`

Fonte: [2026-08-27_v26c_j15r1_nominal_mean_shift_addendum.md](../user/2026-08-27_v26c_j15r1_nominal_mean_shift_addendum.md).

L’addendum misura nominal_mean_shift=0.11720377206802368, pari a 4.44× J11, con NumPy/Torch concordi. La metrica non è vincolante e il runner rimane da correggere. La misura riguarda il massimo cambiamento delle medie knee/ankle su 16.000 righe nominali: argmax riga274, knee. Forward float32 e differenze float64 concordano fra NumPy/Torch. Il valore 0.11720377206802368 è 4,44× il valore J11; non cambia il PASS 17/17 perché non è binding. Non sono eseguiti fit o modifiche al runner. Rimane aperto il suo campo dichiarato ma non calcolato, mentre il gate A–F è poi eseguito in J16. Artefatti pertinenti del report: `module_state.pkl`.

<a id="completion-20261005-2026-08-27_v26c_j15r1_runner_correction_readiness"></a>

### V26C J15R1 — Readiness correttiva: la correzione del runner

Fonte: [2026-08-27_v26c_j15r1_runner_correction_readiness.md](../user/2026-08-27_v26c_j15r1_runner_correction_readiness.md).

J15R1 risolve il NameError collegando verify_warm_start_source alla funzione J11 tramite alias esplicito. Aggiunge test di binding statico; suite 300 e preflight passano senza cambiare la matematica. L’alias di modulo `verify_warm_start_source = J11.verify_warm_start_source` completa il namespace della funzione riusata. La suite controlla i free-name reali del percorso production e distingue il vecchio runner fallito dal successore: 300 check/preflight passati. Non cambiano obiettivo, iperparametri o semantica del commit. Il primo GO non viene riusato; il fit J15R1 successivo dimostra il superamento dell’arresto tecnico, non la qualità closed-loop. Artefatti pertinenti del report: `v26c_j15_fresh_refit.py`, `test_v26c_j15_fresh_refit.py`, `v26c_j15_fresh_refit_authorization.json`.

<a id="completion-20261005-2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness"></a>

### V26C J16 — Readiness: riqualifica closed-loop A–F dell'attore J15R1

Fonte: [2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).

La readiness J16 pone il gate di regressione A–F sul nuovo J15R1: recuperare E/F senza perdere A–D. Deriva nominale già misurata; held-out solo dopo il gate. Restano nominal_mean_shift nel runner e policy_std=null. La readiness chiede se J15R1 recuperi E/F senza far regredire A–D, sulla stessa matrice e soglie; suite 473/preflight passati. Lo shift nominale 4,44× è già noto prima dell’esecuzione e non va cancellato dal riassunto. Restano `nominal_mean_shift` nel runner e `policy_std=null` nella telemetria; il successivo J16 risponde negativamente alla domanda comportamentale. I seed 126–128 sono separati fino a una qualifica operativa positiva. Artefatti pertinenti del report: `v26c_j16_prereg_closed_loop_requalification.json`, `v26c_j16_closed_loop.py`, `test_v26c_j16_closed_loop.py`.

<a id="completion-20261005-2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution"></a>

### V26C J16 — Esecuzione: la riqualifica A–F è FALLITA 0/6

Fonte: [2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md).

J16 fallisce 0/6 con telemetria 6/6 valida: cinque celle terminano per penetrazione e D completa solo un ciclo. Il fresh refit migliora offline ma fa regredire il comportamento. Cinque celle terminano per `grf_penetration`; D arriva a 500 step ma completa un solo ciclo. Questo failure mode è diverso dal precedente swing bloccato, e le sei telemetrie valide impediscono di attribuirlo a un problema di registrazione. I risultati offline migliorati non preservano A–D: occorre diagnosi, poi effettuata in J17. I debiti `policy_std=null` e metrica nominale non calcolata rimangono specifici; nessun receipt del FAIL viene trasformato in PASS. Artefatti pertinenti del report: `v26c_j16_closed_loop_receipt.json`.

<a id="completion-20261005-2026-08-27_v26c_j17_architect_audit_addendum"></a>

### V26C J17 — Addendum di chiusura dopo l'audit dell'architetto

Fonte: [2026-08-27_v26c_j17_architect_audit_addendum.md](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

L’audit J17 sceglie J8, miglior parent operativo 5/6, per update incrementale B-only; J11/J14 rimangono evidenza diagnostica. Distingue self-anchor e rischio del gradiente; elenca debiti di provenance e riproducibilità. L’audit supera la proposta fresh/J11: sceglie J8 operativo e update B-only vincolato, senza widening né PPO correttivo. J2/J11/J14 restano necessari alla diagnosi, non parent del prossimo actor. Distingue la debole anchor parametrica dalla preservazione funzionale. Restano runner non pinnati nei leaf storici, J11 non bit-riproducibile, J2 privo di commit verification/digest, `best_validation_mse` contaminato dal tiling e confronto appaiato pre-breccia da svolgere; J18/J19 risolvono l’intervento operativo, non questi debiti. Artefatti pertinenti del report: `v26c_j9r1_closed_loop_receipt.json`, `v26c_j17_prereg_causal_diagnosis.json`, `v26c_j17_architect_go_2026-08-27.json`.

<a id="completion-20261005-2026-08-27_v26c_j17_architect_audit_weighting_correction"></a>

### V26C J17 — Correzione dell'effective weighting

Fonte: [2026-08-27_v26c_j17_architect_audit_weighting_correction.md](../user/2026-08-27_v26c_j17_architect_audit_weighting_correction.md).

J17 corregge il weighting B/C e separa blocchi disgiunti da viste diagnostiche non additive. Le quote effettive sono 15.6452%, diverse dalle quote delle righe uniche; totale 25.567 e decisione J8 invariati. L’overlay corregge due errori numerici e un errore di categoria: B/C hanno 4.000 righe effettive ciascuno dopo repeat8, quote 15,6452%, contro500 uniche ciascuno, quote 1,9556%. Le viste diagnostiche non sono blocchi sommabili; il totale 25567 è verificato senza double count. La diagnosi causale e la decisione J8 restano invariate. È una correzione documentale del weighting, senza nuovi fit; i debiti di metrica/provenance/reproducibilità elencati nell’audit non risultano chiusi. Artefatti pertinenti del report: `v26c_j17_architect_audit_addendum_weighting_correction_2026-08-27.json`, `v26c_j17_architect_audit_addendum_2026-08-27.json`, `v26c_j17_prereg_causal_diagnosis.json`.

<a id="completion-20261005-2026-08-27_v26c_j17_diagnosi_causale_regressione_j16"></a>

### V26C J17 — Diagnosi causale della regressione J16

Fonte: [2026-08-27_v26c_j17_diagnosi_causale_regressione_j16.md](../user/2026-08-27_v26c_j17_diagnosi_causale_regressione_j16.md).

J17 attribuisce a J14 solo il 3.34% delle righe ma l’84.81% della massa di gradiente. Le self-anchor da J2 iniziano senza gradiente e la penalità parametrica è debole; la proposta J11 è poi superata da J8. Le righe correttive iniziano ad alto errore, mentre le self-anchor prodotte da J2 hanno errore/gradiente iniziale nullo: la percentuale di righe non misura l’influenza effettiva sull’update. L’anchor parametrica debole non impone conservazione delle uscite. La diagnosi da trace e gradienti è distinta da un esperimento causale completamente isolato; la scelta del parent viene fissata dall’addendum J8. Persistono nominal metric, telemetria sigma e debiti di provenance/validation/reproducibilità, senza nuove simulazioni in questo audit. Artefatti pertinenti del report: `v26c_j17_prereg_causal_diagnosis.json`, `v26c_j17_diagnostic_measurements.json`, `v26c_j17_reconciliation_and_corrections.json`.

<a id="completion-20261005-2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione"></a>

### V26C J18 — Addendum v2: preparazione dell'esecuzione

Fonte: [2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md).

J18 v2 esplicita pesi dei minibatch, composizione per blocco e selezione a theta fisso. Sostituisce il falso roundtrip bit-identico dopo training con una verifica funzionale dello scale absorption; 139 test. La versione v2 esplicita normalizzazione per blocco, coefficienti delle righe/minibatch, fattore N/|M| nell’ultimo chunk e valutazione a theta fisso, registrando RNG e step. Un’asserzione di roundtrip bit-identico dopo training avrebbe respinto un update sano: la sostituisce con preservazione funzionale del mapping fisico. Suite 139 passata. La versione successiva corregge la formulazione sullo stimatore e rafforza i controlli; non si deduce un nuovo risultato scientifico dall’addendum preparatorio. Artefatti pertinenti del report: `v26c_j18_prereg_b_only_constrained_update.json`, `v26c_j18_prereg_addendum_v2_minibatch_and_scaling_2026-08-27.json`, `v26c_j18_dataset_manifest.json`.

<a id="completion-20261005-2026-08-27_v26c_j18_addendum_v3_correzioni_architetto"></a>

### V26C J18 — Addendum v3 e correzioni d'architetto

Fonte: [2026-08-27_v26c_j18_addendum_v3_correzioni_architetto.md](../user/2026-08-27_v26c_j18_addendum_v3_correzioni_architetto.md).

J18 v3 corregge la formulazione sul random reshuffling, verifica scale absorption con kernel equivalenti e tolleranza 1e-6, elimina import antecedenti. Aggiunge guardie NaN/JSON rigoroso e identità byte logstd; 249 test. Il random reshuffling non è dichiarato condizionatamente unbiased come il campionamento con rimpiazzo. Lo scale absorption confronta le stesse semantiche dei kernel, tutte 4221 righe e tolleranza 1e−6; tre import antecedenti vengono eliminati. Guardie recursive sui non finiti, JSON rigoroso e byte-identity logstd impediscono passaggi vacui. Suite 249/preflight 42 passati. Restano offline filtering e closed-loop distinti, con i debiti storici specifici ancora dichiarati. Artefatti pertinenti del report: `v26c_j18_prereg_b_only_constrained_update.json`, `v26c_j18_prereg_addendum_v2_minibatch_and_scaling_2026-08-27.json`, `v26c_j18_prereg_addendum_v3_scale_absorption_2026-08-27.json`.

<a id="completion-20261005-2026-08-27_v26c_j18_b_only_update_execution"></a>

### V26C J18 — Esecuzione dell'update B-only vincolato su J8

Fonte: [2026-08-27_v26c_j18_b_only_update_execution.md](../user/2026-08-27_v26c_j18_b_only_update_execution.md).

J18 non trova survivor: 0/16 candidati superano il max drift C/D di 0.005, nessun actor scritto. Il risultato congelato alimenta una successiva ricalibrazione read-only; offline filtra ma non certifica closed-loop. Il filtro offline respinge tutti i16 candidati al max drift C/D0.005: nessun nuovo actor viene scritto e nessuna qualifica fisica parte. Il risultato è congelato come FAIL_NO_SURVIVOR, non ricalcolato con soglie allentate. La ricalibrazione successiva è una nuova decisione read-only sui risultati congelati; J19 riproduce solo il candidato poi scelto. Restano i debiti storici sulle metriche, sulla riproducibilità J11 e sulla provenance dei leaf. Artefatti pertinenti del report: `v26c_j18_b_only_update_receipt.json`, `v26c_j18_selection.json`, `v26c_j18_candidate_metrics.json`.

<a id="completion-20261005-2026-08-27_v26c_j18_prereg_update_b_only"></a>

### V26C J18 — Preregistrazione dell'update B-only vincolato su J8

Fonte: [2026-08-27_v26c_j18_prereg_update_b_only.md](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).

J18 preregistra update da J8 con quattro blocchi globalmente disgiunti, priorità A>B>C>D e 4.221 righe, su 16 candidati. Corregge il gate logstd vacuo usando le righe della testa e definisce i contratti di scaling fisico. La disgiunzione globale A>B>C>D elimina4 righe da4225 e lascia4221, con pesi dei blocchi espliciti. Il gate logstd non cerca soltanto chiavi per nome, altrimenti sarebbe vacuo: controlla anche le righe 2:4 della testa gaussiana. Scaling fisico, controller e preservation delle uscite sono preregistrati separatamente dal futuro gate A–F. Preflight/test preparatori verificano il design, non la riuscita della griglia. I vecchi leaf non diventano riproducibili perché il nuovo runner è pinnato. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j18_b_only_update.py`, `v26c_j18_prereg_b_only_constrained_update.json`.

<a id="completion-20261005-2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione"></a>

### V26C J18 — Diagnosi della coda, ricalibrazione e selezione sui risultati congelati

Fonte: [2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).

La ricalibrazione J18 ritira il max-drift binding derivato dal rumore, adotta ceiling empirici di bias J8 e seleziona il candidato 13: lambda 30, beta 1, lr 5e-5. Analisi read-only, nessun nuovo fit. Il max drift single-sample0.005, derivato dal rumore, viene ritirato come hard gate; resta una chiave diagnostica/tie-break. I ceiling di bias C/D vengono calibrati su J8, distinti da massimi delle singole deviazioni. La scelta congelata c13 (lambda 30, beta 1, lr5e−5) non riaddestra né produce actor: serve J19A. I criteri offline filtrano e A–F certifica dopo la riproduzione; POST e confronto appaiato restano limiti, non prove held-out.

<a id="completion-20261005-2026-08-27_v26c_j19a_esecuzione_riproduzione_singola"></a>

### V26C J19A — Esecuzione della riproduzione singola

Fonte: [2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).

J19A riproduce bit per bit il candidato 13: 11/11 criteri offline, attore promosso, nessun rollout. POST-485=0.957933 non è un held-out; il disegno mantiene fattori confondenti dichiarati. La riproduzione è esattamente bit-identica al candidato congelato, con 11/11 criteri e digest actor registrato; non è un risultato entro semplice tolleranza e non include rollout. POST-485=0.957933 è un diagnostico sugli stati selezionati, con fattori confondenti del disegno, non un held-out. J19B poi esegue la qualifica. Restano metriche nominali mancanti, `policy_std=null`, contaminazione validation e provenance storica; le assertion readiness divenute false dopo GO sono annotate e non riscritte. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j19a_single_reproduction_receipt.json`, `v26c_j19a_result.json`.

<a id="completion-20261005-2026-08-27_v26c_j19a_readiness_riproduzione_singola"></a>

### V26C J19A — Readiness della riproduzione singola

Fonte: [2026-08-27_v26c_j19a_readiness_riproduzione_singola.md](../user/2026-08-27_v26c_j19a_readiness_riproduzione_singola.md).

J19A prepara una singola riproduzione del candidato 13 e inverte l’import J18 per renderla autonoma. Suite 127 e preflight 57 passano; esecuzione ancora separata e GO richiesto. Per evitare una dipendenza circolare dallo stadio precedente, J19A contiene la riproduzione autonoma e inverte l’import di J18; intervento approvato e verificato con 127 check/preflight 57. Il GO successivo pinna quattro file, senza riusare il token consumato. L’esecuzione J19A chiude il passo di riproduzione, mentre J19B deve ancora qualificare. E54/S47 sono invarianti di readiness scadute dopo GO, non nuove regressioni scientifiche; i debiti storici restano elencati. Artefatti pertinenti del report: `v26c_j19a_prereg_single_reproduction.json`, `v26c_j19a_single_reproduction.py`, `test_v26c_j19a_single_reproduction.py`.

<a id="completion-20261005-2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop"></a>

### V26C J19B — Esecuzione della qualificazione closed-loop A–F

Fonte: [2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).

J19B qualifica J19A con PASS 6/6 A–F, FSM senza errori e actor invariato. F entra nella banda diagnostica oltre 25 mm, restando sotto 28; margine 2.38 mm. Held-out e debiti osservabilità/provenance rimangono separati. L’attore resta byte-invariato su tutte le sei celle e i contatori FSM rilevanti restano zero. F raggiunge una penetrazione fra25 e 28 mm: margine 2,38 mm, da mantenere visibile prima di ulteriori passi. A–F include casi già diagnosticati/corretti, quindi non è l’held-out finale. Il successivo G–I chiude quel passo separato. Sono ancora aperti `policy_std=null`, metrica nominale del runner e assenza storica di commit verification J8/J2; i vecchi FAIL J9/J12/J16 restano tali. Artefatti pertinenti del report: `v26c_j19b_closed_loop_receipt.json`, `module_state.pkl`, `v26c_j16_closed_loop.py`.

<a id="completion-20261005-2026-08-27_v26c_j19b_readiness_qualificazione_closed_loop"></a>

### V26C J19B — Readiness della qualificazione closed-loop A–F

Fonte: [2026-08-27_v26c_j19b_readiness_qualificazione_closed_loop.md](../user/2026-08-27_v26c_j19b_readiness_qualificazione_closed_loop.md).

La readiness J19B conserva meccanicamente la qualifica J16, verifica otto funzioni e risolve i nomi feature con fonti concordi. Suite 116 passata; stessa matrice A–F, GO separato. La preparazione copia meccanicamente la qualifica J16 e confronta il bytecode di otto funzioni, evitando cambi di scienza. I nomi feature sono risolti con fonti concordi anziché dal manifest stantio;116 check passati, nessun ambiente o rollout. J19B successivo chiude il gateA–F. La provenienza dei leaf J8/J2 e `policy_std=null` restano debiti documentali; le vecchie assertion dopo GO non vengono riscritte retroattivamente. Artefatti pertinenti del report: `v26c_j19b_derive_from_j16.py`, `v26c_penetration_contract.py`, `v26c_j19b_prereg_closed_loop_qualification.json`.

<a id="completion-20261005-2026-08-27_v26c_j19c_esecuzione_heldout_g_i"></a>

### V26C J19C — Esecuzione della fase held-out G/H/I

Fonte: [2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).

J19C passa 3/3 e 42/42 gate sui seed 126–128 allo start nominale: nessuna partenza nuova. Actor invariato, FSM pulita; H oltre 25 mm ma sotto 28. Critic warm-up è il prossimo stadio separato. I tre seed 126–128 erano sigillati e la prova passa42/42 gate con actor invariato. Si tratta di rumore nuovo allo stesso start nominale, non generalizzazione a nuovi start/soggetti. H entra nella banda25 mm ma resta sotto28. Il critic warm-up viene poi eseguito in uno stadio separato; questo PASS non lo autorizzava da solo. `ENV_MUTATION_POLICY` consentiva più variazioni di quelle effettivamente usate: superset conservativa documentata, insieme ai debiti metriche/provenance. Artefatti pertinenti del report: `module_state.pkl`.

<a id="completion-20261005-2026-08-27_v26c_j19c_readiness_heldout_g_i"></a>

### V26C J19C — Readiness della fase held-out G/H/I

Fonte: [2026-08-27_v26c_j19c_readiness_heldout_g_i.md](../user/2026-08-27_v26c_j19c_readiness_heldout_g_i.md).

J19C prepara i tre seed 126–128 allo stesso start nominale, sigma 0.005 e 500 step. Precisa che non sono partenze nuove; suite 120 passata. Restano debiti osservabilità/provenance. Dopo un rilievo scientifico, il protocollo precisa esplicitamente cheG–I non prova partenze nuove: seed 126–128, start nominale, sigma 0.005 e 500 step. Readiness120 passata, actor/config invariati e GO separato. Il successivo J19C risolve il gate indipendente. Restano `policy_std=null`, nominal metric del runner, sidecar stantio e assenza storica di commit verification; nessuna prova multimodello è dedotta da questi tre seed. Artefatti pertinenti del report: `v26c_j19c_prereg_heldout_g_i.json`, `v26c_j19c_derive_from_j19b.py`, `v26c_j19c_heldout_g_i.py`.

<a id="completion-20261005-2026-08-27_v26c_j20_critic_warmup_execution"></a>

[V26C — J20: esecuzione singola del warm-up critic-only](../user/2026-08-27_v26c_j20_critic_warmup_execution.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-27_v26c_j20_critic_warmup_execution_prereg"></a>

### V26C J20 — Preregistrazione dell'esecuzione del warm-up critic-only

Fonte: [2026-08-27_v26c_j20_critic_warmup_execution_prereg.md](../user/2026-08-27_v26c_j20_critic_warmup_execution_prereg.md).

La preregistrazione J20 rileva che la guardia KL farebbe abortire il warm-up per telemetria assente con exact start disattivo. La disabilita solo nel config warm-up; 208 test. Il risultato previsto è AWAITING_RESTORE_AUDIT. La guardia KL richiedeva telemetria non disponibile nel warm-up con `exact_start_sampling=false`, producendo un aborto infrastrutturale previsto prima del risultato scientifico. L’overlay la disabilita solo per questa fase critic-only;208 test passati. Il miglior verdetto previsto è AWAITING_RESTORE_AUDIT, non PASS. La successiva iterazione4096 e il restore R3 chiudono due passi diversi; i config canonici lunghi non sono modificati da tale eccezione di warm-up. Artefatti pertinenti del report: `train_ppo_mlp.py`, `module_state.pkl`, `v26c_j20_prereg_critic_warmup_execution.json`.

<a id="completion-20261005-2026-08-27_v26c_j20_k1_probe"></a>

[V26C J20 — Esecuzione della sonda K1](../user/2026-08-27_v26c_j20_k1_probe.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-27_v26c_j20_k1r1_execution"></a>

[V26C J20-K1R1 — Esecuzione dell'emendamento al gradiente non degenere](../user/2026-08-27_v26c_j20_k1r1_execution.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-27_v26c_j20_k1r1_prereg_amendment"></a>

[V26C J20-K1R1 — Preregistrazione dell'emendamento al gradiente non degenere](../user/2026-08-27_v26c_j20_k1r1_prereg_amendment.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-27_v26c_j20_precedente_luglio_probe_gradiente"></a>

### V26C J20 — Il precedente di luglio della sonda di gradiente

Fonte: [2026-08-27_v26c_j20_precedente_luglio_probe_gradiente.md](../user/2026-08-27_v26c_j20_precedente_luglio_probe_gradiente.md).

L’indagine corregge la ricostruzione July: esisteva già una sonda autograd degenere, ma con gate più debole e senza lo stesso FAIL. L’addestrabilità del critic era provata su dati reali, non dal gradiente nullo. La ricostruzione storica viene corretta: a luglio esisteva già una sonda autograd alimentata da zeri, con un gate meno stringente, non un precedente identico FAIL. Il critic aveva appreso su dati reali; il gradiente nullo di quello stimolo non ne negava l’addestrabilità. È un’indagine documentale del precedente, senza probe/warm-up rilanciati. La soluzione8×84 di K1R1 è separata; il successo successivo non sostituisce questa rettifica della cronologia.

<a id="completion-20261005-2026-08-27_v26c_j20_readiness_critic_warmup"></a>

### V26C J20 — Readiness del warm-up del solo critic (K0 audit + K1 probe)

Fonte: [2026-08-27_v26c_j20_readiness_critic_warmup.md](../user/2026-08-27_v26c_j20_readiness_critic_warmup.md).

La readiness J20 prepara critic fresco, feature actor 35 e config derivato, aggiungendo telemetria di produzione in tb_logging.py/train_ppo_mlp.py. Corregge conteggio righe, logging robusto e probe del percorso reale; suite e pin passano. Sono aggiunte osservazioni di produzione in `tb_logging.py` e `train_ppo_mlp.py`: numero di righe effettivamente osservate, logging resistente a eccezioni e probe della funzione reale, non solo un surrogate test. Readiness191, telemetria161 e pin 20 passati. Actor35/critic84, trapianto byte-exact e critic fresco sono verificati prima di K1/warm-up; un caricamento actor-only non dimostra critic disponibile. Restano sigma nulla nella vecchia telemetria e i debiti nominal/provenance dei leaf storici. Artefatti pertinenti del report: `v26c_j20_prereg_critic_warmup_readiness.json`, `v26c_j20_critic_warmup_readiness.py`, `test_v26c_j20_critic_warmup_readiness.py`.

<a id="completion-20261005-2026-08-27_v26c_j20_restore_audit_esecuzione"></a>

[V26C — J20: esecuzione singola del restore audit — **FAIL_CLOSED 4/13**](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-27_v26c_j20_restore_audit_preparazione"></a>

### V26C — J20: preparazione del restore audit a zero iterazioni (chiude G9)

Fonte: [2026-08-27_v26c_j20_restore_audit_preparazione.md](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md).

La preparazione restore definisce audit a zero update sul percorso di produzione, con optimizer vivo e confronto indipendente del modulo. Suite 266 e pin 63 passano; draft inerte, nessun restore ancora eseguito. L’audit attraversa `train_ppo_mlp.run`/restore di produzione a zero update, cattura l’optimizer vivo e confronta il modulo con un controllo indipendente, invece di leggere solo un file checkpoint. Suite 266/pin 63 passati preparano un draft inerte e una singola esecuzione. Il successivo R1 fallisce tecnicamente prima del restore; R2/R3 sostituiscono la preparazione per arrivare al gate reale. I GO storici sono consumati e non autorizzano retry in-place. Artefatti pertinenti del report: `v26c_j20_restore_audit_child.py`, `train_ppo_mlp.py`, `v26c_j20_prereg_restore_audit.json`.

<a id="completion-20261005-2026-08-27_v26c_j20_restore_audit_r2_preparazione"></a>

### V26C — J20 R2: preparazione del secondo tentativo correttivo del restore audit

Fonte: [2026-08-27_v26c_j20_restore_audit_r2_preparazione.md](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).

R2 prepende baseline_MLP assoluto al PYTHONPATH del child preservando il vecchio ambiente e timeout 600. Smoke cloudpickle reale negativo/positivo senza Ray cluster; suite 223 e pin 76. Decisione sui gcs_server orfani aperta. Il child eredita `PYTHONPATH` con `baseline_MLP` assoluto anteposto e separatore del sistema operativo, preservando il vecchio valore e timeout600. Smoke cloudpickle reale negativo/positivo senza cluster Ray dimostra il rimedio all’import; suite 223/pin 76 passati. R2 poi raggiunge davvero il restore e scopre un problema diverso. Restano il marker sorgente immutabile e la decisione documentale sui due `gcs_server` del18/08: nessuna evidenza ne certifica la chiusura o lo stato attuale. Artefatti pertinenti del report: `train_ppo_mlp.py`, `v26c_j20_prereg_restore_audit_rev5.json`, `v26c_j20_restore_audit_r2.py`.


### TODO riconciliati dalle fonti di questa data

Il registro distingue passi storici eseguiti, piani superati, debiti aperti e
condizioni/invarianti per eventuali lavori futuri. Le chiusure qui citate sono
retrospettive: nessuna nuova autorizzazione o esecuzione deriva dall’addendum.

- **Aperto — Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza.** [fonte, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- **Risolto — Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi.** [fonte, riga 323](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). Chiusura/supersessione 2026-08-27: [J12 esegue A–F e termina FAIL 4/6.](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md).
- **Risolto — Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi.** [fonte, riga 221](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). Chiusura/supersessione 2026-08-27: [Riqualifica J16 eseguita: FAIL 0/6, telemetrie valide.](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md).
- **Risolto — Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi.** [fonte, riga 209](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Risolto — Approvare ed eseguire il probe critic prima del warm-up, usando stimolo non degenere dopo il FAIL K1.** [fonte, riga 587](../user/2026-08-27_v26c_j20_readiness_critic_warmup.md). Chiusura/supersessione 2026-08-27: [K1R1 passa 12/12 con stimolo non degenere, composito 29/29.](../user/2026-08-27_v26c_j20_k1r1_execution.md).
- **Risolto — Preregistrare/autorizzare critic-only, costruire modulo trainabile fresco e trapiantare tutte le chiavi actor; actor/logstd congelati.** [fonte, riga 325](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). Chiusura/supersessione 2026-08-27: [Warm-up critic-only 1/4096 eseguito: actor/logstd invariati, critic cambia.](../user/2026-08-27_v26c_j20_critic_warmup_execution.md).
- **Condizionato — Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop.** [fonte, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).
- **Condizionato — LOTO, LOCO, B1R1 e B1R2: integrazioni future soggette ad approvazione esplicita.** [fonte, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- **Risolto — Eseguire il fit in uno stadio separato e autorizzato, verificando input pinnati e conteggi.** [fonte, riga 235](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md). Chiusura/supersessione 2026-08-27: [J11 fit eseguito, PASS 16/16 offline.](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- **Risolto — Eseguire il fit in uno stadio separato e autorizzato, verificando input pinnati e conteggi.** [fonte, riga 218](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). Chiusura/supersessione 2026-08-27: [Fit J15R1 completato, PASS 17/17 offline.](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- **Aperto — Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi.** [fonte, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md). È una decisione documentale non chiusa; lo stato attuale dei processi non è attestato.
- **Aperto — Generalizzazione oltre AB06 con dataset EPIC, soggetto/modello held-out e normalizzazione/phase detector/corridor verificati.** [fonte, riga 351](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- **Risolto — Chiudere il gate finale di recupero AB06 della lineage J19/J20/J21.** [fonte, riga 264](../user/2026-08-27_v26c_j20_critic_warmup_execution.md). Chiusura/supersessione 2026-08-28: [J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni.](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md).
- **Superato — Rimanere senza actor-updating PPO finché non sono chiusi restore e attestazione; il training successivo richiede autorizzazione distinta.** [fonte, riga 326](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md). Chiusura/supersessione 2026-08-28: [Dopo separata autorizzazione utente, training da J20 per 50 nuove iterazioni; il vincolo preparatorio non è un divieto permanente.](./2026-08-28_daily-report.md).
- **Aperto — Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate.** [fonte, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Aperto — Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici.** [fonte, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- **Aperto — Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata.** [fonte, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).
- **Aperto — Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto.** [fonte, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- **Aperto — Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti.** [fonte, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Risolto — Pinnare gli otto file definitivi J11 nello stadio successivo senza correggere dopo il manifest.** [fonte, riga 340](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). Chiusura/supersessione 2026-08-27: [J12 esegue A–F e termina FAIL 4/6.](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md).
- **Superato — Decidere fra recovery J8 e miglioramenti multistart J11 prima dell’avanzamento.** [fonte, riga 330](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). Chiusura/supersessione 2026-08-27: [Decisione operativa J8/B-only sostituisce il piano fresh da J2/J11.](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Aperto — Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11.** [fonte, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Superato — Il blocco di robustezza J11 su 124/125 è diagnosticato e superato operativamente scegliendo J8/J19, senza correggere il vecchio J11.** [fonte, riga 327](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Superato — Rischio nominale J15R1: qualificarlo prima di avanzare; J16 lo respinge e la lineage operativa riparte da J8.** [fonte, riga 235](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Risolto — Correggere il binding verify_warm_start_source e aggiungere controllo statico sul namespace production.** [fonte, riga 200](../user/2026-08-27_v26c_j15_fresh_refit_execution.md). Chiusura/supersessione 2026-08-27: [Fit J15R1 completato, PASS 17/17 offline.](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- **Risolto — Diagnosi della regressione J16 prima della correzione operativa.** [fonte, riga 168](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md). Chiusura/supersessione 2026-08-27: [Decisione operativa J8/B-only sostituisce il piano fresh da J2/J11.](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Condizionato — Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18.** [fonte, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- **Condizionato — Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato.** [fonte, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).
- **Risolto — Autorizzare ed eseguire una volta la griglia J18 su input pinnati.** [fonte, riga 208](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md). Chiusura/supersessione 2026-08-27: [J18 esegue la griglia: zero survivor e nessun actor scritto.](../user/2026-08-27_v26c_j18_b_only_update_execution.md).
- **Condizionato — Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione.** [fonte, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- **Risolto — Decidere come procedere dopo 0/16, con nuova preregistrazione e senza riscrivere soglie/risultati della griglia fallita.** [fonte, riga 208](../user/2026-08-27_v26c_j18_b_only_update_execution.md). Chiusura/supersessione 2026-08-27: [J19A riproduce bit per bit il candidato 13 ed emette actor.](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- **Risolto — Approvare/verificare inversione dell’import J18 nella riproduzione autonoma J19A.** [fonte, riga 151](../user/2026-08-27_v26c_j19a_readiness_riproduzione_singola.md). Chiusura/supersessione 2026-08-27: [J19A riproduce bit per bit il candidato 13 ed emette actor.](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- **Superato — Valutare il diagnostico POST-485 e il disegno confondente prima di J19B.** [fonte, riga 190](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Risolto — Riprodurre una volta il candidato 13 selezionato con un nuovo GO e input pinnati.** [fonte, riga 263](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md). Chiusura/supersessione 2026-08-27: [J19A riproduce bit per bit il candidato 13 ed emette actor.](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- **Risolto — Preparare/autorizzare la qualifica J19B solo dopo l’audit J19A.** [fonte, riga 263](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Risolto — Eseguire il gate indipendente G–I sui seed 126–128 riservati allo start nominale dopo la qualifica operativa A–F.** [fonte, riga 216](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). Chiusura/supersessione 2026-08-27: [J19C held-out seed 126–128 allo start nominale PASS 3/3.](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- **Condizionato — Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale.** [fonte, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- **Risolto — Eseguire, con GO separato, la correzione B-only vincolata da J8 e qualificarla.** [fonte, riga 228](../user/2026-08-27_v26c_j17_architect_audit_addendum.md). Chiusura/supersessione 2026-08-27: [J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor.](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Aperto — Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf.** [fonte, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).
- **Aperto — Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1.** [fonte, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- **Condizionato — Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati.** [fonte, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- **Superato — Riconsiderare il supporto multistart assente da J7/J8, senza importare dataset July nella nuova lineage.** [fonte, riga 347](../user/2026-08-27_qualifica_closed_loop_j9r1.md). Chiusura/supersessione 2026-08-27: [J11 fit eseguito, PASS 16/16 offline.](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- **Condizionato — Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding.** [fonte, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- **Aperto — Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto.** [fonte, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md). La misura esterna è conclusa; non equivale a correzione del runner.
- **Superato — Il blocco operativo dovuto a J8/J11 non qualificati viene superato da update su J8 e qualifica del nuovo J19A.** [fonte, riga 239](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md). Chiusura/supersessione 2026-08-27: [J19C held-out seed 126–128 allo start nominale PASS 3/3.](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- **Aperto — Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti.** [fonte, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Condizionato — Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori.** [fonte, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- **Risolto — Includere nella correzione la regione successiva al mismatch FSM invece di arrestare la raccolta al primo mismatch.** [fonte, riga 425](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md). Chiusura/supersessione 2026-08-27: [J14 materializza le 854 righe post/pre-mismatch, senza troncamento.](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md).
- **Risolto — Decisione sulla causa del restore fallito e nuovo stadio/GO, preservando il leaf precedente senza retry in-place.** [fonte, riga 184](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md). Chiusura/supersessione 2026-08-28: [R3 restore PASS 13/13 e G9 chiuso a zero update.](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- **Risolto — Eseguire restore audit reale a zero iterazioni e chiudere G9; R1/R2 restano fallimenti storici, R3 è la prova positiva.** [fonte, riga 257](../user/2026-08-27_v26c_j20_critic_warmup_execution.md). Chiusura/supersessione 2026-08-28: [R3 restore PASS 13/13 e G9 chiuso a zero update.](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- **Superato — Valutare eventuali correzioni preparatorie senza riscrivere rev 1/rev 2.** [fonte, riga 225](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md). Chiusura/supersessione 2026-08-28: [R2 eseguito col rimedio PYTHONPATH: vero restore raggiunto, FAIL 7/13 per rappresentazione.](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- **Risolto — Revisione e GO separato per una singola esecuzione restore R1.** [fonte, riga 223](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md). Chiusura/supersessione 2026-08-27: [Prima esecuzione restore conclusa FAIL_CLOSED tecnico, senza retry in-place.](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md).
- **Risolto — Revisione/autorizzazione e GO separato per R2.** [fonte, riga 271](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md). Chiusura/supersessione 2026-08-28: [R2 eseguito col rimedio PYTHONPATH: vero restore raggiunto, FAIL 7/13 per rappresentazione.](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- **Condizionato — Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy.** [fonte, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- **Condizionato — Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente.** [fonte, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- **Condizionato — Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati.** [fonte, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).
- **Condizionato — Conservare il sidecar J8 stantio immutabile e usare l’overlay di provenance/verifica dei byte quando lo si legge.** [fonte, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- **Aperto — Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21.** [fonte, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- **Condizionato — Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione.** [fonte, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).
- **Condizionato — La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento.** [fonte, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- **Aperto — Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente.** [fonte, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- **Condizionato — Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente.** [fonte, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).

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
- [ ] Completare il milestone di recupero AB06 qualificato e attestato come training-input-only. La ricostruzione J19–J21 sostituisce la vecchia proposta F0/F1–F7, senza dichiarare eseguite tutte quelle fasi né il benchmark generale di parità. — [fonte 2026-08-23, riga 200](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Rimanere senza actor-updating PPO finché non sono chiusi restore e attestazione; il training successivo richiede autorizzazione distinta. — [fonte 2026-08-27, riga 326](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md).
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
- [ ] Decisione sulla causa del restore fallito e nuovo stadio/GO, preservando il leaf precedente senza retry in-place. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md).
- [ ] Eseguire restore audit reale a zero iterazioni e chiudere G9; R1/R2 restano fallimenti storici, R3 è la prova positiva. — [fonte 2026-08-27, riga 257](../user/2026-08-27_v26c_j20_critic_warmup_execution.md).
- [ ] Valutare eventuali correzioni preparatorie senza riscrivere rev 1/rev 2. — [fonte 2026-08-27, riga 225](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md).
- [ ] Revisione/autorizzazione e GO separato per R2. — [fonte 2026-08-27, riga 271](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).

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

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Misurare sigma sul lineage corrente prima delle recovery, senza ereditare il placeholder 0,005. — [fonte 2026-08-24, riga 211](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md). **Chiuso:** Qualificato circa0,005 con seed123–125 per la fase V26C; nuove policy o nuovi contesti richiedono una nuova misura. ([evidenza](2026-08-27_daily-report.md)).
- [x] Ottenere cammino completo con almeno due cicli e plantarflessione, preservando gli start richiesti. — [fonte 2026-08-24, riga 213](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md). **Chiuso:** J19B tabella cinematica: ankle min negativa in tutteA–F, circa−0,111/−0,118rad. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi. — [fonte 2026-08-27, riga 323](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). **Chiuso:** J12 esegue A–F e termina FAIL 4/6. ([evidenza](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md)).
- [x] Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi. — [fonte 2026-08-27, riga 221](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). **Chiuso:** Riqualifica J16 eseguita: FAIL 0/6, telemetrie valide. ([evidenza](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md)).
- [x] Eseguire il gate closed-loop distinto dai PASS offline/readiness, conservandone anche gli esiti negativi. — [fonte 2026-08-27, riga 209](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md). **Chiuso:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Approvare ed eseguire il probe critic prima del warm-up, usando stimolo non degenere dopo il FAIL K1. — [fonte 2026-08-27, riga 587](../user/2026-08-27_v26c_j20_readiness_critic_warmup.md). **Chiuso:** K1R1 passa 12/12 con stimolo non degenere, composito 29/29. ([evidenza](../user/2026-08-27_v26c_j20_k1r1_execution.md)).
- [x] Preregistrare/autorizzare critic-only, costruire modulo trainabile fresco e trapiantare tutte le chiavi actor; actor/logstd congelati. — [fonte 2026-08-27, riga 325](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). **Chiuso:** Warm-up critic-only 1/4096 eseguito: actor/logstd invariati, critic cambia. ([evidenza](../user/2026-08-27_v26c_j20_critic_warmup_execution.md)).
- [x] Eseguire il fit in uno stadio separato e autorizzato, verificando input pinnati e conteggi. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md). **Chiuso:** J11 fit eseguito, PASS 16/16 offline. ([evidenza](../user/2026-08-27_v26c_j11_multistart_fit_execution.md)).
- [x] Eseguire il fit in uno stadio separato e autorizzato, verificando input pinnati e conteggi. — [fonte 2026-08-27, riga 218](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). **Chiuso:** Fit J15R1 completato, PASS 17/17 offline. ([evidenza](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md)).
- [x] Pinnare gli otto file definitivi J11 nello stadio successivo senza correggere dopo il manifest. — [fonte 2026-08-27, riga 340](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). **Chiuso:** J12 esegue A–F e termina FAIL 4/6. ([evidenza](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md)).
- [x] Decidere fra recovery J8 e miglioramenti multistart J11 prima dell’avanzamento. — [fonte 2026-08-27, riga 330](../user/2026-08-27_v26c_j11_multistart_fit_execution.md). **Superato:** Decisione operativa J8/B-only sostituisce il piano fresh da J2/J11. ([evidenza](../user/2026-08-27_v26c_j17_architect_audit_addendum.md)).
- [x] Il blocco di robustezza J11 su 124/125 è diagnosticato e superato operativamente scegliendo J8/J19, senza correggere il vecchio J11. — [fonte 2026-08-27, riga 327](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md). **Superato:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Rischio nominale J15R1: qualificarlo prima di avanzare; J16 lo respinge e la lineage operativa riparte da J8. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md). **Superato:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Correggere il binding verify_warm_start_source e aggiungere controllo statico sul namespace production. — [fonte 2026-08-27, riga 200](../user/2026-08-27_v26c_j15_fresh_refit_execution.md). **Chiuso:** Fit J15R1 completato, PASS 17/17 offline. ([evidenza](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md)).
- [x] Diagnosi della regressione J16 prima della correzione operativa. — [fonte 2026-08-27, riga 168](../user/2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md). **Chiuso:** Decisione operativa J8/B-only sostituisce il piano fresh da J2/J11. ([evidenza](../user/2026-08-27_v26c_j17_architect_audit_addendum.md)).
- [x] Autorizzare ed eseguire una volta la griglia J18 su input pinnati. — [fonte 2026-08-27, riga 208](../user/2026-08-27_v26c_j18_addendum_v2_preparazione_esecuzione.md). **Chiuso:** J18 esegue la griglia: zero survivor e nessun actor scritto. ([evidenza](../user/2026-08-27_v26c_j18_b_only_update_execution.md)).
- [x] Decidere come procedere dopo 0/16, con nuova preregistrazione e senza riscrivere soglie/risultati della griglia fallita. — [fonte 2026-08-27, riga 208](../user/2026-08-27_v26c_j18_b_only_update_execution.md). **Chiuso:** J19A riproduce bit per bit il candidato 13 ed emette actor. ([evidenza](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md)).
- [x] Approvare/verificare inversione dell’import J18 nella riproduzione autonoma J19A. — [fonte 2026-08-27, riga 151](../user/2026-08-27_v26c_j19a_readiness_riproduzione_singola.md). **Chiuso:** J19A riproduce bit per bit il candidato 13 ed emette actor. ([evidenza](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md)).
- [x] Valutare il diagnostico POST-485 e il disegno confondente prima di J19B. — [fonte 2026-08-27, riga 190](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md). **Superato:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Riprodurre una volta il candidato 13 selezionato con un nuovo GO e input pinnati. — [fonte 2026-08-27, riga 263](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md). **Chiuso:** J19A riproduce bit per bit il candidato 13 ed emette actor. ([evidenza](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md)).
- [x] Preparare/autorizzare la qualifica J19B solo dopo l’audit J19A. — [fonte 2026-08-27, riga 263](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md). **Chiuso:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Eseguire il gate indipendente G–I sui seed 126–128 riservati allo start nominale dopo la qualifica operativa A–F. — [fonte 2026-08-27, riga 216](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md). **Chiuso:** J19C held-out seed 126–128 allo start nominale PASS 3/3. ([evidenza](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md)).
- [x] Eseguire, con GO separato, la correzione B-only vincolata da J8 e qualificarla. — [fonte 2026-08-27, riga 228](../user/2026-08-27_v26c_j17_architect_audit_addendum.md). **Chiuso:** J19B qualifica J19A 6/6 A–F; non modifica i FAIL dei vecchi actor. ([evidenza](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md)).
- [x] Riconsiderare il supporto multistart assente da J7/J8, senza importare dataset July nella nuova lineage. — [fonte 2026-08-27, riga 347](../user/2026-08-27_qualifica_closed_loop_j9r1.md). **Superato:** J11 fit eseguito, PASS 16/16 offline. ([evidenza](../user/2026-08-27_v26c_j11_multistart_fit_execution.md)).
- [x] Il blocco operativo dovuto a J8/J11 non qualificati viene superato da update su J8 e qualifica del nuovo J19A. — [fonte 2026-08-27, riga 239](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md). **Superato:** J19C held-out seed 126–128 allo start nominale PASS 3/3. ([evidenza](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md)).
- [x] Includere nella correzione la regione successiva al mismatch FSM invece di arrestare la raccolta al primo mismatch. — [fonte 2026-08-27, riga 425](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md). **Chiuso:** J14 materializza le 854 righe post/pre-mismatch, senza troncamento. ([evidenza](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_execution.md)).
- [x] Revisione e GO separato per una singola esecuzione restore R1. — [fonte 2026-08-27, riga 223](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md). **Chiuso:** Prima esecuzione restore conclusa FAIL_CLOSED tecnico, senza retry in-place. ([evidenza](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
