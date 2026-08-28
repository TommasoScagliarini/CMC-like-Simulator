# V26B S1C-1 (rev3x) — fit offline dei tre candidati pesati in fase: **nessun survivor**, e la diagnostica preregistrata risponde alla domanda

**Token:** `V26B-S1C-1` (solo fit offline) · **Data:** 2026-08-24 · Nessun rollout, episodio, collection, promozione, σ o modifica production. Tre candidati, nessun quarto tentativo, nessuna soglia adattata dopo aver visto i risultati.

## 1. Esito
Tutti e tre i candidati **falliscono il gate P (preservazione)** al primo livello utile della gerarchia, con corto circuito su T e D. **Nessun survivor.** I tre attori sono materializzati NONDEPLOYABLE e **quarantinati** (`may_not_be_source: true`).

| Cand. | w | verdetto | livello fallito | P mean_abs (k/a) | P rms | **P max_abs** (bound 0,25) | ‖Δθ‖² |
|---|---|---|---|---|---|---|---|
| W2 | 2 | FAIL | P_preservation | 0,0622 / 0,0465 | 0,0899 / 0,0630 | **0,38193** / 0,26919 | 0,163 |
| W4 | 4 | FAIL | P_preservation | 0,0636 / 0,0478 | 0,1078 / 0,0726 | **0,53635** / 0,36548 | 0,276 |
| W8 | 8 | FAIL | P_preservation | 0,0641 / 0,0470 | 0,1237 / 0,0810 | **0,65583** / 0,43955 | 0,421 |

La violazione è sempre e solo `max_abs > 0,25`, e cresce **monotonicamente con w**. Il meccanismo è trasparente: concentrare il richiamo del task sul 19,2 % delle righe produce escursioni per-riga grandi proprio lì, mentre le medie restano dentro i limiti. Integrità I passata da tutti e tre. T e D non valutati per corto circuito, come prescritto.

## 2. La diagnostica causale preregistrata: **più profondo, non più a lungo**
Preregistrata in rev3x **prima** di vedere qualunque risultato, misurata sulle 100 righe held-out ordinate (steps 201–300, mai usate da nessuno dei due ruoli):

| | duty cycle cmd < 0 | n. run | run medio | **run max** | cmd min | served ref prevista: min / frac<0 |
|---|---|---|---|---|---|---|
| **Target AB06 IK** | **0,240** | **1** | **24,0** | **24** | −0,13182 | — |
| S0D | 0,150 | 3 | 5,0 | 12 | −0,48290 | +0,10687 / 0,000 |
| A2 (r=5, w=1) | 0,130 | 2 | 6,5 | 11 | −0,35022 | +0,11865 / 0,000 |
| **W2** | **0,130** | **2** | **6,5** | **11** | −0,36711 | +0,11727 / 0,000 |
| **W4** | **0,130** | **2** | **6,5** | **11** | −0,38461 | +0,11995 / 0,000 |
| **W8** | **0,130** | **2** | **6,5** | **11** | −0,40718 | +0,12266 / 0,000 |

Il risultato è inequivocabile e riproducibile: **duty cycle, numero di run, run medio e run massimo sono identici per w = 2, 4, 8 — e identici ad A2 (w = 1)**. L'unica quantità che si muove con w è la **profondità** del comando (−0,367 → −0,385 → −0,407). Il peso localizzato in fase rende il comando **più profondo, non più duraturo**.

Il confronto con il target è la misura del divario strutturale: l'IK AB06 ha **un solo blocco continuo di 24 righe** di plantarflessione (duty 0,240), mentre tutti gli attori della catena — S0D incluso — producono **2–3 frammenti brevi**. Nessun candidato si avvicina alla struttura temporale del target; anzi, tutti e tre hanno duty **inferiore** a S0D (0,130 contro 0,150).

**Replay attraverso la replica di produzione validata (rev3w):** per tutti e tre i candidati la reference servita prevista ha minimo **+0,117…+0,123** e frazione negativa **0,000**. Anche col comando più profondo, la reference non diventa negativa su quegli stati — coerente con S1C-0, che aveva mostrato che ciò che serve è la **persistenza**, non l'ampiezza istantanea.

**Limiti dichiarati, non aggirati.** (i) Gli stati held-out sono quelli visitati da S0D: guidarli con le azioni di un altro attore è un **controfattuale in anello aperto**, non ciò che il candidato visiterebbe in closed loop. (ii) Il verdetto B3 è **INDETERMINATE** — il campo di fase è identicamente zero, quindi nessuna affermazione B3 rigorosa è possibile. La diagnostica **non è un gate**, non ha soglie, e non ha promosso nulla.

## 3. Confronto G2–G4 (surrogati offline, **non vincolanti**)
I gate G2–G4 di rev3v sono definiti su **traccia closed-loop** (angoli realizzati, contatori FSM) e **non sono valutabili** in questo stadio, dove nessun rollout è autorizzato. Riporto quindi i surrogati in **spazio del comando** (target articolare decodificato dall'azione) sulle 500 righe congelate, calcolati identicamente per tutti, e **mai usati per promuovere**:

| | vs IK: knee rmse / r / amp | vs IK: ankle rmse / r / amp | vs healthy: knee / ankle |
|---|---|---|---|
| S0D | 0,37763 / 0,103 / 1,158 | 0,26741 / 0,050 / 1,418 | 0,36047 / 0,21532 |
| A2 | 0,31480 / 0,261 / 0,991 | 0,22753 / 0,158 / 1,210 | 0,30073 / 0,18150 |
| W2 | 0,30660 / 0,302 / 0,989 | 0,22636 / 0,175 / 1,227 | 0,29059 / 0,18405 |
| W4 | 0,30038 / 0,339 / 0,995 | 0,22593 / 0,197 / 1,257 | 0,28217 / 0,18732 |
| W8 | 0,29886 / 0,360 / 1,008 | 0,22886 / 0,207 / 1,300 | 0,27899 / 0,19238 |

Nel solo spazio del comando i candidati migliorano monotonicamente rispetto a S0D su ginocchio (rmse e correlazione) e restano sostanzialmente fermi sulla caviglia (rmse 0,226–0,229 contro 0,228 di A2), con l'ampiezza della caviglia che **si allontana** da 1 al crescere di w (1,227 → 1,300). **Nulla di tutto questo autorizza una conclusione sulla qualità realizzata**: la lezione di rev3v — migliorare nello spazio del comando non implica migliorare al giunto — vale qui esattamente come prima, e con più forza, visto che il replay mostra la reference invariata.

## 4. Addendum correttivo su S1C-0 (immutabile, receipt originale intatto)
`v26b_addendum_rev3w_a_probe_corrections.json` → `7c4f39b9e0e2e2a579a189dffd35807ba71cada88c0e2b20c2a25c9f8214935b`.
1. **Refuso di magnitudine corretto**: 1,038e-04 contro una tolleranza 1e-3 è un fattore **9,634×**, cioè **poco meno di un ordine di grandezza — non due**. Solo formulazione: nessun numero, soglia, parametro o verdetto cambia; la validazione resta accettata.
2. **Digest finale del test registrato**: `dc8486aae5acf8664f8f4353d203b679cd296cc799fdac15790b3f9e9cc2e912` (esito di record: PASS, 40 check). Il campo nel receipt del probe resta **`null`** — era legittimamente nullo al momento della scrittura, perché il file di test non esisteva ancora — e **il receipt non è stato riscritto**: verificato byte-identico prima e dopo (`00e6141f…`).

## 5. Test
`test_v26b_s1c1_fit.py`: **PASS 35 check pre-fit**, **PASS 45 post-fit**. Copertura: pin/tamper di rev3x e dell'addendum; verifica numerica del fattore 9,634 e della coesistenza «receipt null + addendum autoritativo», con il receipt del probe riverificato byte-identico; 4 token negativi e la guardia del rollout; assenza di primitive closed-loop; griglia congelata (W2/W4/W8, r = 5, 300 epoche, batch 256, lr 1e-4, seed 2026, ancora July per riferimento) e divieto del quarto tentativo; **definizione push-off** verificata identica alla regola congelata (73/380 righe); **semantica del termine pesato** — con pesi unitari coincide *esattamente* con la media per-ruolo, in forma chiusa vale `Σw·mse/Σw`, solo le righe del ruolo entrano, un ruolo assente dà zero, e alzare w sposta il termine verso le righe pesate; **prova che il ruolo ancora è sempre a peso unitario**, quindi la pesatura non può alterare r; metrica di run-length su sequenze sintetiche; diagnostica dichiarata non-gate con B3 INDETERMINATE e natura controfattuale; init S0D esclusivo e diverso da S1A e A2; output sotto `candidates/`, mai `student/`; e post-fit la coerenza receipt/peso/push-off più il rifiuto di una seconda esecuzione.

## 6. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_addendum_rev3w_a_probe_corrections.json` | `7c4f39b9e0e2e2a579a189dffd35807ba71cada88c0e2b20c2a25c9f8214935b` |
| `…/v26b_amendment_rev3x_s1c1_fit.json` | `041bbc38dec9209d1b0420be8b641da4224fb256f11e420204710476e871f433` |
| `…/v26b_s1c1_fit.py` | `5ca4bf0fc5a7f0e35c139129d6e1e87d83775f116fa7c01ba1d0151b8e0bf992` |
| `…/test_v26b_s1c1_fit.py` | `a6adcfd8630f3cb2f7288133afe2b92e31f33038bc29e58112cc85808e03fe93` |
| `…/candidates/v26b_s1c1_fit_aggregate_20260824_210626.json` | `58874483683ff3f9eb6741f7f752fbb5b9c8fec96b3277c3543f0eae58b7d3e4` |
| `…/candidates/S1C1_W2_35D_NONDEPLOYABLE/v26b_s1c1_fit_receipt.json` | `69de4d29e469c972c38c2fd3838b441d08e853bd4ae77692c22e2c91635a09fb` |
| `…/candidates/S1C1_W4_35D_NONDEPLOYABLE/v26b_s1c1_fit_receipt.json` | `eb51092ade1f9fe1763c472763bb5288d40fd462f0b556a1bb4458b12cf73202` |
| `…/candidates/S1C1_W8_35D_NONDEPLOYABLE/v26b_s1c1_fit_receipt.json` | `9f1a705e71efe03f77f30dcdc1bef23c9a82acc2dbf6b46966b227281c7c223d` |
| `…/S1C1_W2_35D_NONDEPLOYABLE/rl_module/module_state.pkl` | `52548ea45fb7be39099a088711a8166cbe61df67681822be9e046f8114a17a66` |
| `…/2026-08-24_V26B_anchors_r1/s1c1_fit_20260824_210619.log` (`REAL_EXIT_CODE=0`) | `f58ae7097c40b977fa5e8087ea3f1fa7313417787fcb8dddfa4ac8b5d30a7c73` |
| receipt probe S1C-0 (**invariato**) | `00e6141f4051117ef2425d144e062fd9cb61708a70e18e5d38b56864f67d8d2c` |

Digest actor: W2 `a692dd4c86bd…`, W4 `cfb7d0254d29…`, W8 `e6958e4b2064…`. Tutti con `deployable:false`, `rollout_pending:true`, `sigma_unresolved:true`, `quarantined:true`, `may_not_be_source:true`. `student/` invariato; S0D resta l'unico attore della catena con evidenza closed-loop.

## 7. Stato e lettura conclusiva
**STOP per la tua revisione.** Nessun survivor, nessuna promozione, nessun rollout; B3 resta un blocker separato e nessuna modifica di produzione o di logging è stata fatta.

La lettura che i dati sostengono, senza andare oltre: **il peso localizzato in fase è lo strumento sbagliato per l'obiettivo**. Aumenta la profondità istantanea del comando e, con essa, la deriva massima per-riga che rompe la preservazione — mentre lascia **invariata** la struttura temporale (duty 0,130, 2 run, max 11 per tutti e tre, identica ad A2 e inferiore a S0D). Poiché S1C-0 aveva stabilito che ciò che serve alla reference è la **persistenza**, e poiché la replica validata mostra che nemmeno il comando più profondo la porta sotto zero su quegli stati, un quarto valore di w non cambierebbe l'esito: il grado di libertà su cui agisce non è quello che governa il risultato. Se vorrai proseguire su questa linea, la leva da preregistrare è un obiettivo che agisca **sulla durata** (per esempio una penalità sulla frammentazione dei run o un target definito su finestre di fase anziché per riga) — ma è una scelta di modellazione che spetta a te, e io non ho toccato né rev3v né alcuna soglia.
