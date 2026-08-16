# H0 V12R10: forense del fit R9 e selezione del recupero

Data: 2026-08-14

## Obiettivo

Stabilire se il terminal FAIL V12R9 dipende da label incompatibili con le 35
feature, da un errore di assemblaggio/pesatura oppure da ottimizzazione e
forgetting. La diagnosi serve a definire un successore V12R10 import-only,
senza ripetere i rollout di raccolta gia validi.

## Problema analizzato

V12R9 ha completato quattro nuove raccolte fisiche e ha prodotto/importato
2.431 label observer valide, ma l'unico fit W512 ha fallito i gate offline:

- globale `0.01100096 / 0.14004979` contro `0.006 / 0.060`;
- reset massimo `0.01345137` contro `0.003`;
- forgetting sui casi base e residui elevati su observer nominale e seed 127.

## Strategia forense

L'audit read-only ha ricostruito prediction, pesi e slice del corpus congelato,
misurando:

- collisioni esatte sulle 35 feature e sulle 33 feature effettive clock-zero;
- nearest-neighbor nello spazio normalizzato;
- gruppi reset, distribuzione della loss e righe peggiori;
- provenance dei pesi nei NPZ sorgente;
- least-squares counterfactuali del solo head;
- inferenza H0 diretta sulla student-view;
- un confronto full-mean fresh-R6 con lo schedule R9 e i pesi sorgente
  preservati.

Nessun environment, checkpoint canonico, freeze, critic update o PPO update e
stato aperto da queste diagnostiche.

## Risultati

### Il mapping non e contraddittorio sul corpus finito

- 11.875 righe, 11.849 osservazioni distinte;
- 4 gruppi duplicati, 30 righe totali;
- zero gruppi con label discordanti, anche escludendo le colonne clock 0-1;
- lower bound da collisioni: RMSE `0`, max-error `0`;
- distanza distinta normalizzata minima `8.717e-6`, con delta label benigno;
- entro distanza `1e-2`: 223 righe dirette, delta label massimo `6.897e-4`.

Non esiste quindi evidenza di un limite matematico imposto dalle feature sul
corpus osservato.

### R9 ha perso i pesi intra-strato

Il labeler aveva persistito pesi reset/recovery/OOD e il corpus base R5
conteneva anche la hardness P2. Il fitter R9 li ha sostituiti con peso uniforme
per riga all'interno dei 13 strati. Le 26 righe reset, riconducibili a soli tre
stati con label internamente identiche, hanno ricevuto appena lo `0.265%` della
massa totale.

Ripristinando i veri `normalized_sample_weights` sorgente e rinormalizzando
ogni strato a massa 500, la massa reset sale al `3.039%`. Un fit del solo head
porta il reset massimo a `0.001731` e quindi lo chiude, ma resta insufficiente
sul globale: RMSE `0.012290`, massimo `0.135444`. Occorre adattare gli hidden.

### L'errore e concentrato e il fit era ancora in discesa

Tre strati producono il `55.1%` dell'obiettivo R9:

- observer nominale: `24.47%`;
- observer seed 127: `15.51%`;
- base minus: `15.14%`.

Il base minus rappresenta da solo il `53.84%` dell'errore non pesato globale.
La loss R9 era ancora in calo alla closure LBFGS finale, quindi lo stop non
dimostra un limite di capacita.

### Le due scorciatoie semplici non funzionano

L'H0 congelato valutato direttamente sulle osservazioni student-view ottiene
RMSE `0.035772` e massimo `0.200029`: le label corrette dipendono dalla
teacher-view causale ricostruita e non si ottengono con un embedding diretto.

Un full-mean fresh da R6 con i veri pesi sorgente, ripetuto due volte in modo
byte-identico, resta FAIL:

- globale `0.0141549 / 0.139082`;
- reset `0.003717`;
- base minus `0.02037 / 0.13510`;
- observer nominale `0.01860 / 0.13908`;
- loss `0.00046876 -> 0.00012698`.

Anche la continuation dal candidato R9, con gli stessi pesi e due repliche
byte-identiche, resta FAIL:

- globale `0.0122173 / 0.130467`;
- reset `0.0033317`;
- base minus `0.016964 / 0.116243`;
- observer nominale `0.015925 / 0.130467`;
- observer seed 127 `0.011832 / 0.100954`;
- loss `0.00017544 -> 0.000101597`.

La continuation e migliore del fresh fit ma fallisce tutti i gate metrici e
dipenderebbe inoltre da un candidato privo di receipt PASS. La sola correzione
dei pesi o la prosecuzione dello stesso obiettivo non risolvono il forgetting.

## Dry-fit residuale ancorato

La variante primaria da verificare mantiene un actor standard W512 a due
torri:

- contributo `0.70 * P2` congelato;
- ramo residuale inizializzato come `0.30 * R5`;
- funzione al tempo zero equivalente a R6;
- training limitato al ramo residuale verso
  `label - 0.70 * P2`;
- log-std preservata e clock 0-1 bit-zero;
- gate R9 invariati, con protezione esplicita di reset e tail/max-error.

Questa struttura parte dall'unico actor R6 che superava tutti i sei gate base
e riduce il rischio di cancellarli mentre apprende gli stati student-exposed.
Il candidato R9 respinto non viene assunto come predecessore canonico.

Il dry-fit ha preservato correttamente torre P2, cross-block, alias, log-std e
clock, e ha chiuso il reset a `4.29e-5`. Ha tuttavia fallito i gate metrici:

- globale `0.0139955 / 0.121397`;
- observer nominale `0.02128 / 0.12140`;
- observer seed 127 `0.01886 / 0.11044`;
- base minus `0.01742 / 0.09293`;
- R4 `0.01026 / 0.04645`;
- observer `+0.20` tardivo `0.01065 / 0.02491`.

Un primo processo era arrivato a fine fit ma si era fermato nel solo audit di
fold (`1.2517e-6` contro una tolleranza strumentale `1e-6`). La tolleranza e
stata riallineata a quella R6 gia congelata (`2e-6`) e lo stesso identico fit e
stato rieseguito. Tutte le 17 milestone, incluse le 624 closure, sono state
riprodotte esattamente. Il JSON distingue questo rerun diagnostico da un retry
di modello e non costituisce evidenza one-shot o save/reload production.

Il risultato esclude quindi anche il ramo residuale 256 congelando P2 con
questo obiettivo. Nessun candidato e stato pubblicato.

## Dry-fit full-W512 esteso

Il secondo dry-fit ha proseguito dal materiale terminale R9 con adjudication
esplicita, obiettivo equal-stratum originale, righe reset moltiplicate per tre,
nessun peso source-hardness e nessun termine tail. Lo schedule unico e
preregistrato era AdamW per 1.000 epoche seguito da LBFGS fino a 2.000
iterazioni; e stato valutato soltanto lo stato terminale.

Il fit ha ridotto nettamente la loss (`1.05102e-4 -> 4.39747e-5`) e ha chiuso
reset, R4 e observer `+0.20` tardivo:

- reset massimo `0.001091`: PASS;
- R4 `0.005619 / 0.043843`: PASS;
- observer `+0.20` tardivo `0.004350 / 0.017418`: PASS.

Resta tuttavia un FAIL reale sui gate globali e observer:

- globale `0.007256 / 0.124748` contro `0.006 / 0.060`;
- base minus `0.009040 / 0.090573`;
- observer nominale `0.010524 / 0.124748`;
- observer seed 127 `0.008163 / 0.084840`;
- observer seed 128 `0.006507 / 0.063023`.

Il massimo terminale e sulla action 0, caso observer nominale, step 385. Il
modello non e stato persistito, replicato o pubblicato; non sono avvenuti
reset/step di environment, query teacher, checkpoint o update PPO/critic. Il
risultato esclude questa specifica prosecuzione W512 uniforme, ma mostra che il
problema residuo e ormai concentrato nelle code di pochi rollout.

## W512 tail-aware e W1024: entrambe escluse

Prima degli esiti era stata fissata la priorita W512, con W1024 come solo
fallback. Entrambe le strategie hanno terminato in FAIL e, coerentemente con la
regola, non e stato creato alcun candidato.

La W512 tail-aware ha riprodotto byte-exact la Phase A estesa e ha applicato
una sola Phase B IRLS congelata sui residui di confine. Il massimo e sceso da
`0.124748` a `0.094736`, ma l'RMSE globale e salito da `0.007256` a
`0.007788`; il reset e rimasto PASS a `3.28e-5`. Un audit di fold successivo al
verdetto ha rifiutato `1.01328e-6` contro la tolleranza strumentale `1e-6`.
Poiche il modello aveva gia fallito i gate, il processo non e stato ritentato;
il receipt registra in modo conservativo l'abort post-verdetto e l'assenza di
stato persistito.

La rete standard W1024 ha mantenuto R6 byte-esatto nella prima torre e ha
addestrato soltanto una seconda torre residuale isolata. Compatibilita reale
RLModule, save/reload, warm-start, cross-block, clock e logstd sono PASS. La
capacita aggiuntiva ha portato il globale molto vicino alla soglia, ma non l'ha
chiuso:

- globale `0.006105 / 0.110148`;
- base minus `0.007595 / 0.071453`;
- observer nominale `0.008251 / 0.110148`;
- observer seed 127 `0.006340 / 0.054979`;
- observer seed 128 `0.006381 / 0.057292`;
- reset `0.001522`, R4 `0.005099 / 0.039667` e plus-late
  `0.003662 / 0.015090`: PASS.

Il massimo W1024 resta observer nominale, action 0, step 385. La loss uniforme
era ancora in discesa alla closure terminale, ma non e autorizzata una semplice
prosecuzione dello stesso tentativo.

## Diagnosi causale e strategia successiva

L'ispezione temporale riprodotta due volte localizza i picchi observer al
confine in cui il teacher legacy cambia stato per timeout mentre il
detector/FSM V26 rimane correttamente in `STANCE`:

- nominale: transizione teacher allo step 384, eventi V26 adiacenti 320/436;
- seed 127: transizione teacher allo step 386, eventi V26 adiacenti 320/410;
- nessuna transizione V26 nelle finestre 375-395;
- salto action 0 in arrivo: `-0.17746` e `-0.18498`;
- i sei outlier observer principali sono esattamente nominale 385-388 e seed
  127 385-386.

Allo step 385 i due stati student normalizzati distano `0.8003`, mentre le
teacher-view distano `9.1064` e i target differiscono di `0.18372`: entrambi i
record student dichiarano stance, ma soltanto il teacher nominale e gia in
timeout. Non vi sono collisioni byte-exact con label discordanti; la diagnosi e
quindi alias semantico/history-hidden, non collisione geometrica esatta.

Il cluster base-minus e invece una sequenza liscia senza evento nella finestra
di dieci step. Contiene 13 dei 20 errori peggiori: le sue 3.732 righe pesano
`0.133404` ciascuna contro `0.996016` nelle tranche observer, generano il
`48.78%` della SSE non pesata ma soltanto il `14.23%` dell'obiettivo. Questo
spiega perche una loss uniforme riduca bene l'errore medio pesato ma lasci sia
la coda teacher/student sia il caso lungo sottopesato.

Sono ora separati due lavori: un audit di osservabilita causale, che deve
stabilire se Markov35 resta semanticamente difendibile, e un solo dry-fit W1024
con loss simmetrica direttamente normalizzata su tutti i gate RMSE/max/reset.
La nuova loss non allenta soglie e non hardcoda casi o righe. Se anche questa
fallisce, la soluzione successiva dovra correggere la rappresentazione o la
semantica delle label, non accumulare altri fit uniformi.

### Esito del dry-fit gate-aligned

La prova W1024 gate-aligned ha superato integralmente i gate e la sola replica
autorizzata ha riprodotto byte-exact stato, predizioni, metriche e history:

- globale `0.00496814 / 0.04900402`: PASS;
- reset massimo `0.00054288`: PASS;
- observer nominale `0.0055303 / 0.0490040`: PASS;
- observer seed 128 `0.0053846 / 0.0453988`: PASS;
- base minus `0.0053582 / 0.0432718`: PASS;
- observer seed 127 `0.0052508 / 0.0424888`: PASS.

La riproduzione della fase uniforme ha chiuso con digest stato
`52aee29d...fe167`, digest predizioni `2aa5c647...a48d`, 3.072 closure e loss
terminale esatta. Il nuovo stato diagnostico ha digest
`a0ad3cd9...` e predizioni `684ec03d...`; non e stato persistito come modulo
production. RLModule W1024 reale, save/reload temporaneo, trapianto warm-start,
critic preservato, torre R6, cross-block, logstd e clock sono tutti PASS.

Questo risultato autorizza il design della lineage canonica import-only, non
la promozione diretta del dry-fit. Il fit canonico dovra essere congelato ed
eseguito una sola volta, poi superare i sei rollout pure-policy. L'alias
teacher/student resta un rischio scientifico esplicito che i gate fisici e Q3
devono risolvere.

### Audit indipendente di osservabilita

L'audit esteso sulle 2.431 righe observer ha confermato che il target corrente
non e una funzione causalmente pulita dell'osservazione V26-35:

- 232 righe teacher sono in `TIMEOUT`, tutte nei casi nominale e seed 127;
- le corrispondenti righe V26 in timeout sono zero;
- 89 transizioni/eventi legacy non hanno transizione o pulse discreto V26;
- su 232 righe timeout, 203 hanno come nearest non-timeout una label distante
  oltre `0.06`; 75 hanno anche distanza RMS normalizzata33 inferiore a `0.25`;
- nel corpus observer vi sono 110 cliff adiacenti oltre `0.06`, 107 dei quali
  senza transizione/pulse V26.

Non esistono collisioni esatte, quindi l'impossibilita matematica sul corpus
finito non e dimostrata; tuttavia la dipendenza path-dependent rende fragile
una certificazione puramente offline Markov35. L'audit raccomanda come opzione
semanticamente piu pura un futuro relabel tramite proiezione stateless e
byte-deterministica dell'esatta V26-35, vietando sia nuovi timer ridondanti sia
uno shadow legacy online.

La decisione operativa e conservativa ma distinta: non si cambia ora il target
imitativo senza evidenza fisica che H0 alimentato direttamente con V26 sia un
teacher sicuro. Il candidato gate-aligned puo proseguire soltanto come
distillazione approssimata e deve essere adjudicato dai sei development e da
Q3, entrambi fail-closed e senza teacher. Un fallimento fisico chiudera questa
lineage e rendera il relabel V26-puro il successore; nessuna dipendenza legacy
verra introdotta nel runtime.

## File aggiunti

- `Trajectory Generator/baseline_MLP/validation/v12r10/diagnostics/analyze_h0_v12r9_frozen_fit.py`;
- `diagnostics/results/r9_fit_forensics.json`;
- `diagnostics/compare_r6_fresh_vs_r9_continuation.py`;
- `diagnostics/results/r6_fresh_vs_r9_continuation_preserved_weights.json`;
- `diagnostics/continue_h0_v12r9_gate_aligned.py`, conservato come harness
  diagnostico senza risultato terminale e senza candidato pubblicato.
- `diagnostics/dry_fit_anchored_residual.py`;
- `diagnostics/results/dry_fit_anchored_residual.json`;
- `diagnostics/dry_fit_extended_uniform_reset3.py`;
- `diagnostics/results/r9_extended_uniform_reset3_dry_fit.json`;
- `diagnostics/dry_fit_two_stage_irls_tail.py`;
- `diagnostics/results/r9_extended_reset3_two_stage_irls_tail_dry_fit.json`;
- `diagnostics/dry_fit_w1024_r6_residual.py`;
- `diagnostics/results/w1024_r6_residual_reset3_dry_fit.json`;
- `diagnostics/analyze_extended_outliers.py`;
- `diagnostics/results/extended_outlier_analysis.json`;
- `diagnostics/dry_fit_w1024_gate_aligned.py`;
- `diagnostics/results/w1024_gate_aligned_dry_fit.json`;
- `diagnostics/analyze_transition_observability.py`;
- `diagnostics/results/transition_observability.json`.

## Test e verifiche

- doppia esecuzione della forense byte-identica: PASS;
- `py_compile`: PASS;
- Ruff lint e format-check: PASS;
- `git diff --check`: PASS;
- replica fresh-R6: state, prediction e history byte-identici: PASS;
- dry-fit W512 extended: processo completo, gate terminale FAIL e nessuna
  replica ammessa dal protocollo;
- hash source/result W512 extended:
  `0c7a70ac0bdf08e80409f7b5ccdba1a49400cacbc14981e26092d26a8068f6d2` /
  `1aa594a15b6d54a97bf374f1d78801ffa1fafa5050e2add33dd4637f7107c009`;
- hash source/result W512 IRLS:
  `407d2ac046e1b18611262600f9545b4bf0873ec6c4f003569f8e0f4a850af5ae` /
  `c57ef1b7c5a82c3b8931fedc8f4e8a857c77a047aef4b976a7bb44a29228af98`;
- hash source/result W1024:
  `945a2181fc8427209a66eb44a5d5846b557b0383af039432ec72f4406b347c9f` /
  `e4ceb12a6c35276fe8e1cc30a08bb4e3e7a6dcce7ceb3a570bfff6d5afdc8981`;
- compatibilita standard RLModule/save-reload/warm-start W1024: PASS;
- analisi outlier doppia byte-identica; source/result:
  `52e3ebd25bb98f9a329ee91c02532d1cc0563e7294fa64e3d4bfb858990ca59d` /
  `6a8b126e4dac97203edc7aaf966a53b3ab928826c9b03d2f89bfe6997014c776`;
- dry-fit W1024 gate-aligned e replica byte-identica: PASS; source/result:
  `587ffd8e4f8c2c5bf5b63c7880e51b2dc34512a2cf7047377f31c6428b7912dd` /
  `7c69175f7e60d05c2dba7dca7212cbc33ad2c0b8ac39618f9b59d685f1486fdc`;
- audit osservabilita doppio byte-identico; source/result:
  `8602716b6aff073f49bcaac077596bdeace51b0e1fa063bf11b79722bc959ced` /
  `bf1f1c59b9bd730208210b69b5c1d8a9cd0b37b3c8293684e60b4a582acce688`;
- immutabilita di V12R9 e assenza di nuovi rollout/checkpoint: PASS.

## TODO vincolanti

- Concludere l'audit di osservabilita e conservarne il rischio come gate
  esplicito della qualification fisica.
- Implementare V12R10 import-only W1024 in namespace nuovo, congelarlo ed
  eseguire una sola volta i sei development pure-policy.
- Non ritentare o promuovere V12R9.
- Mantenere Q3, checkpoint-zero e Morphology Corridor chiusi fino al terminal
  PASS fisico di V12R10.
