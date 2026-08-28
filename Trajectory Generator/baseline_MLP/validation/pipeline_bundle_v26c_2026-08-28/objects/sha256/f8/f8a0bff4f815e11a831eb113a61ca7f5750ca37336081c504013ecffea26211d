# V26B S1C-2Z (rev3z) — hinge di segno sulla caviglia: **nessun survivor**, e la mia previsione è smentita dai dati

**Token:** `V26B-S1C-2Z-FIT` (solo fit offline) · **Data:** 2026-08-24 · Nessun rollout, episodio, collection, promozione, scelta σ o modifica production. Tre candidati esatti, nessun quarto valore, nessun retry. rev3y **rifiutata e non eseguita**, mai mutata.

## 1. Esito
`L05` (λ_h = 0,5), `L10` (1,0) e `L20` (2,0) **falliscono tutti il gate P**, sempre e solo su `max_abs > 0,25`. **Nessun survivor.** Stato derivato dal verdetto offline: tutti e tre **`OFFLINE_FAILED_QUARANTINED`** — quarantinati, `may_not_be_source`, nessun rollout pendente né permesso.

| Cand. | λ_h | verdetto | P mean (k/a) | P rms | **P max (bound 0,25)** | T holdout | ‖Δθ‖² |
|---|---|---|---|---|---|---|---|
| L05 | 0,5 | FAIL | 0,0587 / 0,0603 | 0,0778 / 0,0861 | 0,24625 / **0,43761** | 0,50718 / 0,37720 | 0,203 |
| L10 | 1,0 | FAIL | 0,0585 / 0,0700 | 0,0786 / 0,1058 | **0,25408** / **0,57519** | 0,50629 / 0,36801 | 0,296 |
| L20 | 2,0 | FAIL | 0,0585 / 0,0809 | 0,0792 / 0,1280 | **0,26161** / **0,71209** | 0,50628 / 0,35840 | 0,427 |

La violazione è sulla **caviglia** e cresce monotonicamente con λ_h: 0,438 → 0,575 → 0,712, cioè fino a **2,8 volte** il limite. Il ginocchio è al margine (0,246 → 0,262).

## 2. La mia previsione era sbagliata, e lo dico per prima cosa
Nella critica precedente avevo argomentato che un hinge unilaterale, essendo **esattamente zero** dove il comando è già negativo, «non dovrebbe far esplodere `P.max_abs` come il peso di S1C-1». I dati la smentiscono: `P.max_abs` sulla caviglia esplode comunque, e monotonicamente in λ_h. La proprietà unilaterale è verificata a livello di termine (i test lo dimostrano riga per riga), ma **non impedisce grandi deviazioni**: dove la rete emette comando positivo l'hinge spinge con gradiente proporzionale a `q`, e su input identici deve contemporaneamente soddisfare l'ancora S0D con peso `r = 5`. La tensione si scarica su poche righe con escursioni enormi.

## 3. E il guadagno di segno è quasi nullo — la misura che conta
Le due finestre che mi erano state chieste (`early 6–14` e `holdout 267–290`) sono, **per costruzione, esattamente quelle in cui l'hinge non agisce**: la prima è esclusa dall'hinge, la seconda non entra mai nel fit. Le loro frazioni di comando positivo sono infatti **identiche per tutti** (0,889 e 0,917 per S0D, A2 e i tre candidati): quella diagnostica non poteva dire nulla sull'efficacia dell'hinge. È un limite della mia strumentazione e lo segnalo come tale.

Ho quindi misurato le **64 righe hinge-attive** (finestre 112–132, 174–179, 330–334, 423–445, 482–490), dove il target IK è negativo al 100 %:

| attore | frazione con comando **positivo** | righe positive | min comando | run negativo max | energia di pendenza / IK |
|---|---|---|---|---|---|
| S0D | 0,9375 | 60/64 | −0,29302 | 2 | 87,30 |
| A2 | 0,8906 | 57/64 | −0,24202 | 3 | 61,21 |
| L05 | 0,8750 | 56/64 | −0,21893 | 3 | 40,76 |
| L10 | 0,8750 | 56/64 | −0,20803 | 3 | 32,47 |
| L20 | **0,8438** | 54/64 | −0,19626 | 3 | **25,14** |

Anche a λ_h = 2,0 l'hinge converte **6 righe su 64** rispetto a S0D (60 → 54) e **3 rispetto ad A2**. Il segno resta sbagliato sull'84–94 % delle righe di finestra. E la persistenza sull'holdout non si muove di un millimetro: duty 0,130, 2 run, run massimo 11 — **identici** ad A2 e ai tre candidati W della tornata precedente.

**Costo/beneficio, in una riga: per convertire 6 righe su 64 il gate di preservazione viene violato di 2,8×.**

## 4. Il compromesso morfologico, che va detto in entrambe le direzioni
Non tutto peggiora. Sulle stesse finestre l'hinge **avvicina** la forma al target IK in modo monotono:

- energia di pendenza rapportata al target: S0D **87,3** → A2 61,2 → L05 40,8 → L10 32,5 → **L20 25,1**;
- energia di curvatura (tutte le 7 finestre): S0D 2484,6 → A2 1750,1 → 1381,9 → 1223,3 → **1060,4**;
- RMSE puntuale caviglia sull'intera traccia: S0D 0,26741 → A2 0,22753 → 0,21422 → 0,20721 → **0,20036**.

Cioè: **tutti gli attori della catena sono da 25 a 100 volte più frastagliati del target IK dentro le finestre**, e l'hinge riduce quella frastagliatura avvicinandosi al riferimento, migliorando insieme la fedeltà puntuale. Questo è l'opposto del difetto di rev3y, che avrebbe distrutto il 75–100 % dell'energia di forma **del target**: qui le etichette non sono toccate e a muoversi verso il target è l'**uscita**. Il compromesso reale è quindi: forma e fedeltà puntuale migliorano, la preservazione di S0D si rompe, e il segno resta comunque sbagliato.

## 5. Lettura strutturale
Sulle righe di finestra S0D comanda **positivo** (60 su 64), e l'ancora di preservazione pesa `r = 5` su input identici a quelli del task. Chiedere il segno negativo lì significa chiedere di allontanarsi da S0D proprio dove S0D è più deciso: **il requisito di segno e il gate di preservazione confliggono sulle stesse righe**. Qualunque termine per riga abbastanza forte da ribaltare il segno viola necessariamente il limite di deviazione massima; e uno abbastanza debole da rispettarlo — come si vede a λ_h = 0,5 — ribalta appena 4 righe. Non è un problema di taratura di λ_h: i tre valori spazzano un fattore 4 e il quadro non cambia, mentre la violazione cresce e il guadagno no.

Questa è la stessa tensione, ora localizzata sulle righe di finestra, già incontrata a rev3l/rev3m: **preservare un comportamento e cambiarlo sulle stesse righe non sono simultaneamente soddisfacibili** sotto soglie fissate.

## 6. Conformità verificata
Etichette IK **byte-identiche** prima e dopo ogni fit: l'hash `sha256` del tensore `u_ik` è registrato in ogni receipt e ricontrollato dopo ciascuno dei tre fit (`labels_modified: false`). Hinge attivo su **64 righe** = 73 righe negative di train meno le 9 della finestra precoce 6–14, che **restano nel fit** sotto la loss puntuale ordinaria e sotto la preservazione S0D. Nessuna riga di hinge cade in holdout o embargo; la finestra 267–290 **non entra mai nel fit** sotto alcun termine. Batching T1R congelato: unica unione mescolata deterministica dei due blocchi, medie per-ruolo separate, **nessun minibatch sequenziale**; l'hinge è applicato **al solo blocco task** per non contarlo due volte. Init esclusivamente S0D ricaricato fresh e verificato per candidato; `logstd` bit-identica; colonne clock ri-azzerate; gate I→P→T→D con soglie rev3v byte-per-byte invariate, valutati dal tool immutato. σ non scelta. `student/` invariato. **Production intatta**: `osim_trj_cmc_like.py` `26458424…` e `rollout_eval.py` `5433bcbc…` invariati, nessuna modifica a env/reward/FSM/morphology/SEA/corridor/C++. Aggregate S1C-1 e receipt del preflight riverificati byte-identici.

## 7. Test
`test_v26b_s1c2z_fit.py`: **PASS 55 check pre-fit**, **PASS 71 post-fit**. Copertura: pin/tamper rev3z e catena, prova che nessun candidato rev3y è stato materializzato; 5 token negativi e guardia rollout; assenza di primitive closed-loop e di codice morto; **hinge dimostrato riga per riga** — esattamente zero per q ≤ 0 (incluso q = −1e-9 e q = 0), pari a (0,7·m)² per q > 0 alla precisione float32, lineare in λ_h, zero su batch senza righe hinge, e **cieco al ginocchio** (un comando knee grande non lo attiva); selezione righe (64 hinge, 9 escluse, nessuna in holdout/embargo, 7 finestre esatte); **etichette non modificate** verificate per hash e per assenza di qualunque pesatura/blending nel percorso di fit; semantica T1R congelata verificata sul sorgente e maschera hinge sul solo blocco task; init esclusivo; logstd/clock/10 chiavi; naming e flag di output; **soglie dei gate invariate** rilette da rev3s; diagnostiche non-gating; e post-fit la derivazione dello stato dal verdetto più il rifiuto di una seconda esecuzione.

Trasparenza sui miei errori: una tolleranza di test a 1e-15 su un confronto float32 (differenza reale ~5e-14) resa relativa; una riga di codice morto rimossa dal tool prima dell'esecuzione; e il limite di strumentazione del §3, che ho misurato e riportato invece di lasciarlo passare.

## 8. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_amendment_rev3z_s1c2z_hinge.json` | `7f642c18bec1ba8bd07e15bca576e017eae2db079138e2d9b16b2310410101a9` |
| `…/v26b_s1c2z_fit.py` | `7ae0b6ecdca943f5202ab85c212752997515efa8bdfbfe95db7d4938e5e3dd94` |
| `…/test_v26b_s1c2z_fit.py` | `cdcb17a84de5ffa2e1b70fb97f7fc478db10cd4254cd168f497c79c711b81d77` |
| `…/candidates/v26b_s1c2z_fit_aggregate_20260824_213123.json` | `8836e8b49e17055cebe53dff5de592bf7ca7f427cca18aa35b371b1c3b6956dc` |
| `…/candidates/S1C2Z_L05_35D_NONDEPLOYABLE/v26b_s1c2z_fit_receipt.json` | `d5fdcfc9eda36588272753e42e22a65e133bef9d6e8c91dca4ee29a9df81b94f` |
| `…/candidates/S1C2Z_L10_35D_NONDEPLOYABLE/v26b_s1c2z_fit_receipt.json` | `db3a1354dd3b9370ac5308bbe6e42275c4851ecccaf3185d0554688801970e0f` |
| `…/candidates/S1C2Z_L20_35D_NONDEPLOYABLE/v26b_s1c2z_fit_receipt.json` | `03802eadd3fc6371a1f959116b44694d5736ced59ff056e6e4162aae36ce71a9` |
| `…/2026-08-24_V26B_anchors_r1/s1c2z_fit_20260824_213117.log` (`REAL_EXIT_CODE=0`) | `039c928ec87a75dadcdc9501ade9f928986dc7c40e4ee8c9d5a7d905cf367207` |
| `Trajectory Generator/osim_trj_cmc_like.py` (**invariato**) | `26458424c44f18fa1dda20b830fa5e7e825c583cc5d10e6d019cb3bd9a0c6d24` |
| `baseline_MLP/rollout_eval.py` (**invariato**) | `5433bcbcd90cbfbc04429a42dcda3669649ed9e3b1daa0866c0d96681a127dba` |

## 9. Stato
**STOP.** Nessun survivor offline, quindi **nessun rollout è né pendente né permesso**; nulla è stato promosso; S0D resta l'unico attore della catena con evidenza closed-loop. **B3 resta INDETERMINATE** (campo di fase identicamente nullo) e **σ resta irrisolta**.

Sintesi per la tua decisione, senza andare oltre i dati: l'hinge fa due cose bene — riduce la frastagliatura verso il target (energia di pendenza da 87 a 25) e migliora la fedeltà puntuale della caviglia (0,267 → 0,200) — ma **non risolve il segno** (6 righe su 64 a λ = 2) e **rompe la preservazione** (fino a 2,8× il limite). Poiché sulle righe interessate S0D comanda positivo e l'ancora pesa 5, segno e preservazione sono in conflitto diretto sulle stesse righe: nessun valore di λ_h li concilia. Le vie che restano non sono di taratura ma di impostazione — rilassare esplicitamente la preservazione **solo** su quelle righe (decisione tua, cambia una soglia), oppure accettare che l'escursione negativa della caviglia non sia ottenibile da questo attore su questo split — e non ne ho intrapresa nessuna.
