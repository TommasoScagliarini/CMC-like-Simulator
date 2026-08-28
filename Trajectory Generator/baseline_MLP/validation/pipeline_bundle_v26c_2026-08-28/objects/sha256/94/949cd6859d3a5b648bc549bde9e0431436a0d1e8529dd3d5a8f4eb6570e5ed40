# V26C J15 — Readiness: refit fresco 35D post-mismatch

**Data**: 2026-08-27
**Stadio proposto**: `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT`
**Esito**: bundle di readiness completo e verificato, con il 17° gate binding recepito.
**Nessun fit eseguito.**
**Autorizzazione**: `execution_permitted_now: false`.
**Report obbligatorio di fine fase.**

---

## 1. Problema

J12 ha qualificato closed-loop l'attore J11 e ha **fallito 4/6**. J13 ne ha stabilito la causa: uno
swing che non si chiude. Negli stati delle celle E ed F il 41% e il 55% dei passi cadono oltre 3σ
dal supporto di training di J11, guidati da `phase_swing_elapsed_norm` con |z| ≈ 22 e **zero** righe
di training oltre 0.1888. J14 ha materializzato le 854 righe correttive da quelle due celle fallite,
etichettate dal teacher allo stesso step e **senza troncamento**, così che le 671 righe
post-mismatch — l'unica regione in cui il fallimento vive — siano presenti.

J15 è il refit supervisionato che le usa. **Non qualifica nulla**: non costruisce environment, non
fa rollout, non tocca il critic, e non dice niente sul cammino.

---

## 2. Strategia

### Il parent è J2, non J11

Vincolo dell'architetto e scelta corretta: J11 è l'attore che **ha fallito** J12. Ripartire da lui
comporrebbe ciò che il closed-loop ha rigettato. Il parent è lo stesso da cui partì J11 — J2 35D,
`module_state.pkl` = `0f182ea9…`. Il receipt e il manifest di J15 registrano J11 come **evidenza
negativa**, pinnato per hash e mai caricato.

### Gli iperparametri non sono trascritti

Questo è il punto su cui ho lavorato di più. J11 costruisce i suoi iperparametri come
`dict(J8.JULY_HP)` dal modulo J8 congelato e pinnato. J15 riproduce quella costruzione riga per
riga e verifica a run time l'**uguaglianza** con la fonte congelata e con la copia di J11.

Ho dovuto correggermi qui: la mia prima versione asseriva l'**identità di oggetto**, e falliva —
giustamente, perché J11 prende una copia. La garanzia corretta, e quella che il runner ora
verifica, è l'uguaglianza contro la stessa fonte pinnata. Nel runner non compare alcun letterale
`5e-05` né `0.2`: un test lo asserisce staticamente.

I dieci valori, ricavati dalla fonte e non da un report:

```
seed 123 · epochs 400 · batch_size 128 · learning_rate 5e-05 · validation_fraction 0.2
patience 60 · clip_weight 1.0 · logstd_weight 0.0 · anchor_weight 0.01
trainable_first_layer_features None          BEST_EPSILON 1e-9
```

### `run_fit` non è riscritto: è spliciato

Invece di ricopiare a mano il loop di training — dove un refuso cambierebbe la scienza senza che
nessuno se ne accorga — `run_fit` è la funzione di J11 **spliciata verbatim**. Il runner porta
`RUN_FIT_EXPECTED_DIFF`, il diff unificato atteso, e il test lo **ricalcola a run time** e rifiuta
qualunque cosa non sia esattamente quello.

Il diff è di **18 righe**: il nome della classe d'errore, la docstring, e due diagnostici di sola
lettura. Un test verifica in più che **nessuna** riga modificata tocchi l'optimizer, la loss, lo
shuffle o la regola di arresto.

---

## 3. Analisi indipendente della lineage e degli iperparametri J11

Ho letto io stesso il runner J11 e in parallelo ho fatto girare tre audit read-only indipendenti,
riconciliandone i risultati di persona. Tre conclusioni hanno cambiato il progetto.

**«Multistart» riguarda i dati, non l'ottimizzazione.** Non c'è alcun restart multiplo: una sola
run, una sola inizializzazione, nessuna selezione di vincitore. Il nome si riferisce alle due celle
teacher J10R1 raccolte da istanti di gait off-nominal, più il blocco nominale J7 come terza
partenza. J15 è uguale: **una** run.

**La log-std è congelata per proiezione, non per esclusione.** I tensori `pi.1.*` **sono**
nell'optimizer; Adam aggiorna anche le righe log-std, e `project()` le riscrive col valore del
parent dopo ogni step. I pesi committati risultano bit-identici al parent — ed è un gate — ma i
buffer di momento di Adam su quelle voci accumulano aggiornamenti poi scartati. È proiezione, non
freezing, e il bundle lo dice così invece di lasciarlo intendere altrimenti.

**Le celle B e C non sono clock-zeroed su disco.** `gait_phase_cos` vale **1.0** costante nei file
teacher; J11 le proietta in fase di aggregazione. È l'unico punto in cui l'assemblaggio può
sbagliare in silenzio: concatenare i file grezzi inietterebbe una feature costante in 8000 righe su
25567. J15 riusa la `load_cell` congelata di J11, e un test misura entrambe le cose — 1.0 sul disco,
0.0 nell'aggregato.

### Tre lacune del record J11 che J15 chiude

| lacuna | come J15 la chiude |
|---|---|
| Adam riceve solo `lr`; `betas`, `eps`, `weight_decay`, `amsgrad` e la **versione di torch** non sono registrati da nessuna parte, quindi il fit non è bit-riproducibile dai soli artefatti | il receipt registra i `param_groups` risolti e `torch.__version__`. **Nulla cambia**: ciò che era implicito viene scritto |
| `BEST_EPSILON = 1e-9` è il min-delta effettivo dell'early stopping ma non compare in alcun campo di prereg o receipt J11 | preregistrato esplicitamente |
| `actor_digest` è solo nel manifest, non nel receipt | registrato in entrambi |

---

## 4. Il dataset

```
J7 16713 + cella B 500×8 + cella C 500×8 + J14 854×1 = 25567
```

Verificata **per misura sui file**, non da un report. I primi quattro blocchi riproducono
l'aggregato J11 nello stesso ordine: un test verifica che le righe `[0, 24713)` siano
**bit-identiche** a `v26c_j11_aggregate_dataset.npz`. Le 854 righe sono **appese**, così gli indici
di J11 restano invariati e i due fit sono confrontabili riga per riga.

| blocco | slice | righe | unique | repeat | tiled |
|---|---|---|---|---|---|
| `j7_nominal` | `[0, 16000)` | 16000 | 500 | 32 | sì |
| `j7_recovery` | `[16000, 16713)` | 713 | 713 | 1 | no |
| `cell_B` | `[16713, 20713)` | 4000 | 500 | 8 | sì |
| `cell_C` | `[20713, 24713)` | 4000 | 500 | 8 | sì |
| **`j14_increment`** | **`[24713, 25567)`** | **854** | **854** | **1** | no |

Semantica `np.tile`, non `np.repeat` — testato in entrambe le direzioni. Nessun dedup, nessun
bilanciamento, nessun filtro su `post_mismatch`, nessun repeat sulle righe nuove, nessun
troncamento. Clock colonne 0 e 1 **esattamente zero** su tutte le 25567 righe.

Split: **5113 / 20454**, da `max(1, int(round(25567 × 0.2)))`, verificato tramite lo splitter J8
congelato, non assunto. Validazione ordinata, training con **ordine preservato** — ordinarlo
cambierebbe ogni shuffle di epoca.

---

## 5. Matrice dei gate — 17 binding

**Il gate di J15 NON è identico a quello di J11**, e nessun artefatto dice più che lo sia. È
l'insieme di J11 **preservato**, più esattamente uno.

Il runner tiene i due insiemi come tuple separate e nominate — `J11_BINDING_SUBSET` (16) e
`J15_ADDITIONAL_BINDING` (1) — con `J15_BINDING_NAMES` che è letteralmente la loro concatenazione,
e tre `assert` a import time. Così «l'insieme di J11 è preservato» è un fatto verificabile, non una
frase. `audit()` e il receipt riportano `identical_to_j11: false` più un `relation_to_j11` che
nomina il sottoinsieme esatto e l'unico extra; un test verifica che la stringa
`binding_identical_to_j11` **non compaia da nessuna parte** nel receipt.

Nessuna soglia assoluta da nessuna parte — un test statico verifica che la matrice dei gate non
contenga **alcun letterale float**.

`16 preservati da J11 + 1 aggiunto da J15 = 17`: 11 controlli di integrità (chiavi/shape/dtype
come il parent · parametri finiti · clock bit-zero · alias bit-identici · log-std bit-identica al
parent · nessuna chiave critic · input pinnati invariati dopo il fit · l'aggregato riproduce i
propri content hash · split come dichiarato · best state ricostruibile dalla history · metriche
finite), **5** RMSE che devono decrescere, e le 10 colonne controller con norma > 0.

`no_invented_thresholds` ora dice il vero: **cinque** sottoinsiemi RMSE binding, e **solo il blocco
J14 nel suo insieme** vincola.

| RMSE binding | righe | slice (half-open) |
|---|---|---|
| `aggregate_rmse_decreases` | 25567 | `[0, 25567)` |
| `recovery_rmse_decreases` | 713 | `[16000, 16713)` |
| `cell_B_unique_rmse_decreases` | 500 | `[16713, 17213)` |
| `cell_C_unique_rmse_decreases` | 500 | `[20713, 21213)` |
| **`j14_increment_rmse_decreases`** | **854** | **`[24713, 25567)`** |

> **Notazione**: tutti gli intervalli sono **half-open**, `[start, stop)`: l'estremo superiore è
> escluso e la cardinalità è `stop − start`. Così `25567 − 24713 = 854`.

### Il 17° gate

Promosso da diagnostico a **binding** su tua correzione. Non è un'invenzione: il report J13
approvato, righe 372-378, preregistrava già *«RMSE dopo < prima su aggregato, recovery 713, B
unique, C unique **e sul nuovo blocco E**»*. Quando J13 fu scritto, F doveva essere escluso — un
mio errore, che avevi corretto. J14 porta **E e F**, quindi la stessa regola preregistrata copre
tutte le 854 righe.

Disuguaglianza stretta contro una baseline misurata, **nessun numero nuovo**. Vincola il blocco
**intero**: E ed F non sono divisi, perché un gate per cella sarebbe una regola nuova.

Avevo raccomandato questa promozione senza farla da solo; ora è tua decisione, recepita, e il gate
la applica.

### 5.1 Come è verificato che nessun artefatto dica il falso

Il gate di J15 non è quello di J11, e nessuna stringa deve affermare il contrario — né nel runner,
né nella preregistrazione, né in ciò che finirebbe nel receipt di una run vera.

Il tuo audit ha trovato due punti che me n'erano sfuggiti: la docstring di `gate_matrix`
(*«Binding rules are J11's, unchanged»*) e `gate.principle` nella preregistrazione (*«J11's binding
gate, unchanged and name for name, PLUS…»*). Entrambi corretti; entrambi ora dicono la stessa cosa
in modo non ambiguo: **il sottoinsieme di 16 nomi di J11 è preservato, e J15 aggiunge una regola
binding**.

**Il difetto vero però non erano quelle due stringhe, era il mio test.** Conteneva una lista di
quattro frasi scelte da me, e io avevo poi riassunto «nessuna stringa stale» — un'affermazione più
ampia di ciò che il test verificava. L'ho sostituita con una **regola strutturale**:

> Nessuna stringa, nel runner **o** nella preregistrazione, può **affermare** che il gate di J15, o
> le sue regole binding come insieme, siano quelli di J11 invariati.

Quattro regex mirano alla **pretesa**, non alla parola, così una negazione — *«questo gate NON è
identico a quello di J11»* — resta legittima. Prima di fidarsi della regola, il test **prova che
morde** su entrambe le stringhe che gli audit hanno realmente trovato, e **prova che non scatta** su
una negazione. Le docstring di `audit`, `build_receipt`, `gate_matrix` e `verify_prereg` sono
controllate una per una. La stringa esatta che hai trovato resta come ancora di regressione.

Le etichette per-regola sono passate da `«J11, preserved unchanged»` a `«preserved from J11»`, così
nessuna stringa del gate accosta più J11 a quella parola.

### Diagnostici, che restano tali

| diagnostico | righe (slice half-open) | perché non vincola |
|---|---|---|
| `j14_cell_E_rmse` | 500, `[24713, 25213)` | visibilità per cella; un gate per cella sarebbe regola nuova |
| `j14_cell_F_rmse` | 354, `[25213, 25567)` | idem |
| `j14_post_mismatch_rmse` | 671 | riportato, mai usato per filtrare o pesare una riga |
| `j14_pre_mismatch_rmse` | 183 | contraltare: le righe che un troncamento J7 avrebbe tenuto |
| `j14_phase_invalid_fraction` | 671/854 | J13 la elenca fra i diagnostici; resta tale |
| `j11_prefix_rmse` | 24713 | nessuna direzione preregistrata, quindi non può essere un gate |
| `nominal_mean_shift` | 16000 | July usava 0.005 e **J11 misurò 0.0264**; importare quella soglia sarebbe inventare |

`load_j14` **rifiuta** l'incremento se E ed F non sono i blocchi contigui `[0, 500)` e `[500, 854)`,
perché le slice del gate ne dipendono.

Il gate dichiara esplicitamente di essere **solo supervisionato**: superarlo non dice nulla sul
closed-loop.

---

## 6. File preparati

| file | SHA-256 |
|---|---|
| `v26c_j15_prereg_fresh_refit.json` | `49c748b3a20925a0c270768aa6fddcd3adee474eb1de1cfaa885f849719d1ad2` |
| `v26c_j15_fresh_refit.py` | `7b0073c8832e1af64a243cf9bff70662fb786f84691d3a5eab451e230192507a` |
| `test_v26c_j15_fresh_refit.py` | `5bdeaf8d72b3619c5bca0b99b9bfa1da1644804fe043ba32936e95d920fd1176` |
| `v26c_j15_fresh_refit_authorization.json` (rev3) | `61b6fa4fd45fcf00a418efdcb3a450b7eeaddd56b486cc481ef9de44f127a3c4` |

**File modificati**: soltanto il report di esecuzione J14, con le tre correzioni che avevi chiesto
(§7). **Nessun altro file esistente è stato toccato**: leaf, runner, prereg, autorizzazioni e report
di J0–J14 restano byte-identici.

**23 sorgenti pinnate**: dataset e receipt J7 · receipt e commit verification J10R1 · celle B e C ·
i quattro file `rl_module` del parent J2 · incremento, receipt e commit verification J14 · receipt,
commit verification e `module_state` J11 (evidenza negativa) · la catena di governance J14 (prereg
rev1, runner, authorization rev3, record GO) · i moduli libreria J8 e J11 · `warm_start.py` ·
`asymmetric_rl_module.py`.

---

## 7. Le tre correzioni al report J14

Applicate come richiesto, senza toccare leaf, runner, prereg, autorizzazioni o altri report:

1. l'Esito ora recita **«una materializzazione riuscita; due invocazioni totali, la seconda respinta
   fail-closed senza scritture»**;
2. §1 dice **«Una sola materializzazione riuscita»**;
3. §6 riporta la tua disposizione: evidenza accettata, deviazione non invalidante, **probe
   post-successo sull'argv congelato vietati d'ora in poi**.

Nuovo SHA del report J14: `f0f1b31f5d0b765e01289f600afaa3f64177e6312d576cfc3ff42bdc2a924bc1`.
La disposizione è recepita e incisa anche nel record di esecuzione J15.

`apply_patch` non esiste in questo ambiente — l'avevo già segnalato e resta vero; ho usato lo
strumento di edit disponibile.

---

## 8. Verifiche

| controllo | esito |
|---|---|
| `py_compile` runner + test, `PYTHONPYCACHEPREFIX` fuori dalla root | **OK** |
| suite completa, `PYTHONDONTWRITEBYTECODE` e `PYTHONPYCACHEPREFIX` **assenti** | **208 PASS** |
| preflight read-only | **GO**, 0 blocker, 23 sorgenti, torch-free, nessuna sentinella |
| diff `run_fit` vs J11 | **esattamente** `RUN_FIT_EXPECTED_DIFF`, 18 righe, nessuna tocca la matematica |
| iperparametri | uguali a J11 e alla fonte J8 congelata; nessun letterale nel runner |
| aggregato | 25567 righe, (25567, 35)/(25567, 2) float32, cinque blocchi, tile verificato |
| prefisso J11 | **bit-identico** a `v26c_j11_aggregate_dataset.npz` |
| clock | 0.0 esatto in aggregato; **1.0 sulle celle grezze**, come atteso |
| split | 5113/20454, disgiunto, copre ogni riga |
| semi | 124/125 nell'incremento; **126/127/128 mai letti** |
| gate binding | **17** = 16 preservati + 1; `identical_to_j11: false` in gate, audit e receipt |
| semantica stale nel receipt | **assente**: `binding_identical_to_j11` e «four subsets» verificati mancanti |
| pretese «gate J11 unchanged» | **nessuna**, per regola strutturale su runner **e** preregistrazione (§5.1) |
| letterali float nella matrice dei gate | **nessuno** (verifica statica AST) |
| blocchi E/F | contigui, `[24713, 25213)` e `[25213, 25567)`; `load_j14` rifiuta altrimenti |
| torch / ray | mai importati dal preflight né dalla suite; `import torch` **una sola volta**, dentro `run_fit` |
| materializzazione | solo in `tempfile` con `OUTPUT_ROOT_OVERRIDE` e un doppio iniettato; il path reale non è **mai** invocato |
| ermeticità esterna | **643 file** (vedi §8.1): 0 aggiunti, 0 rimossi, 0 cambiati, **invariante su due run** |
| `j15_runs` / lock / staging / sentinella | **assenti** |
| leaf J14, aggregato J11, parent J2 | **intatti per hash** |
| worktree | preservato: solo i tre file tracciati già dirty a inizio sessione |

Il runner rifiuta fail-closed: token di stadio errato, `--out` non autorizzato o relativo, un fit
reale su root ridiretta, un doppio iniettato sulla root autoritativa, una seconda materializzazione,
una sorgente manomessa, una leaf sorgente marcata `TECHNICAL_INVALID`. Tutti testati.

### 8.1 Inventario dell'ermeticità, riproducibile

Una bozza precedente di questo report diceva **642** file e il mio riepilogo finale **643**. Non è
un errore di conteggio: erano due misure in due momenti. **642** fu misurato quando questa fase
aveva tre file; **643** dopo che il record di esecuzione congelato è diventato il quarto. La
differenza era temporale. Il numero coerente, e quello che vale, è **643**.

Perché sia riproducibile, ecco la definizione e la composizione invece del solo totale:

> **Definizione**: ogni file **regolare** sotto la validation root
> `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`, ricorsivamente,
> **senza alcuna esclusione** — `__pycache__` incluso.

| categoria | file |
|---|---|
| `__pycache__` (bytecode generato da esecuzioni precedenti) | 50 |
| dentro le leaf committate `j*_runs/` | 503 |
| i 4 file di questa fase (prereg, runner, test, record) | 4 |
| runner, test e record delle fasi precedenti | 86 |
| **totale** | **643** |

Il record di esecuzione porta il digest dell'inventario **escludendo sé stesso** — 642 file — perché
un record che vive sotto la root che impronta invaliderebbe il proprio valore nell'atto di
scriverlo. È la stessa auto-esclusione già adottata per il record GO di J14.

Ciò che l'ermeticità afferma non è un valore assoluto ma l'**invarianza**: `+0 −0 ~0` su due run
consecutive della suite, con l'ambiente pulito.

---

## 9. Rischi scientifici

- **Le 854 righe sono il 3,3% dell'aggregato.** Se quella massa basti a spostare il comportamento
  closed-loop è **ignoto** e questo stadio non lo verifica.
- **L'etichetta è un'assunzione, non una prova.** Il teacher è funzione pura del tempo, quindi è
  definito ovunque; che sia il bersaglio giusto *dopo* che la FSM è divergata resta non testato.
  J13 lo aveva già dichiarato tale.
- **`validation_mse` non è una stima di generalizzazione.** Ogni blocco ripetuto mette la stessa
  riga unica in entrambe le partizioni. Ereditato da July, J7 e J11; registrato, non corretto in
  silenzio.
- **`nominal_mean_shift`**: J11 misurò 0.0264 contro la soglia storica July di 0.005. Quella soglia
  **non** è applicata qui — importarla ora sarebbe inventare un gate — ma la deriva del blocco
  nominale dal proprio self-anchor è reale e le 854 righe potrebbero accentuarla.
- **Il critic è assente, non preservato.** Un futuro stadio PPO che istanzia questo checkpoint parte
  da una value function inizializzata a caso. Costo reale, che questo stadio non crea né risolve.
- **Il sidecar copiato dichiara `freeze_logstd: false`**: sono flag a livello di modulo per un
  consumatore a valle. Il freeze di questo stadio è interamente offline. Non l'ho alterato:
  cambiarlo romperebbe la byte-identità col parent.
- **Il fit non è bit-riproducibile fra macchine** finché versione di torch e argomenti Adam non
  passati differiscono. J15 li registra; non può sanare il silenzio di J11.
- **La leaf J2 non ha `commit_verification.json`**: precede quella convenzione. L'integrità del
  parent poggia sugli hash dei file, non su una leaf auto-verificante. Dichiarato, non nascosto.
- **Il repeat 8 sulle celle non ha giustificazione documentata** in alcun report o config July.
  J11 lo aveva già registrato come valore osservato; J15 non cambia nulla e ripete la disclosure.

---

## 10. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri, non aperti qui.
- **Semi 126, 127, 128** — riserva held-out finale, mai letti. Non aperti.
- **Il gate A–F di regressione/qualificazione** che dovrà seguire questo refit: è lì che si decide
  se il problema J12 è stato toccato. Questo stadio non lo anticipa.
- **Il fit non è autorizzato da questo stadio**: serve un tuo GO successivo, additivo, che pinni i
  tre hash di input.
- **Regola di governance recepita**: ogni fase si chiude con uno user report dedicato in
  `reports/user/`, e la fase successiva non inizia prima che tu lo abbia auditato. È incisa nella
  preregistrazione e nel record di esecuzione, e vale da qui in avanti.

---

## 11. Regola di governance recepita

Nuova regola vincolante dell'utente, in vigore da ora:

> **Ogni fase si chiude obbligatoriamente con uno user report dedicato in `reports/user/`, e la
> fase successiva non inizia prima che l'architetto lo abbia auditato.**

È incisa in `governance_rule_recorded` dentro la preregistrazione e dentro il record di esecuzione,
così vive negli artefatti e non solo qui.

Conseguenza operativa immediata: **questo report chiude la fase J15-readiness.** Nessun fit
comincia prima del tuo audit, e lo stesso varrà per ogni fase successiva — inclusa la riqualifica
closed-loop.

Recepisco anche, e la porto avanti come standing: **i probe post-successo sull'argv congelato sono
vietati**.

---

## 12. STOP

Readiness completa. **Nessun fit, nessun rollout, nessun critic, nessun PPO, nessuna leaf.**
`execution_permitted_now: false`.

**Fermo in attesa del tuo audit.**
