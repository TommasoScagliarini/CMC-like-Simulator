# V26B S1C-2 preflight (rev3y) — audit di osservabilità: **GO**, con un'eterogeneità che va detta

**Token:** `V26B-S1C-2-PREFLIGHT` (audit read-only + preregistrazione) · **Data:** 2026-08-24 · Nessun fit, rollout, episodio, collection, promozione, scelta σ o modifica production. Nessun artefatto precedente mutato.

## 1. Esito
**GO**, su tutte e tre le condizioni congelate in rev3y **prima** di qualunque classificazione: balanced accuracy **0,78485** ≥ 0,75; recall sulla classe positiva **0,63918** ≥ 0,60; margine sul null a permutazione a blocchi **0,30217** ≥ 0,10 (null medio 0,48268, max 0,65733). Il protocollo S1C-2 può essere preparato; **il fit resta non autorizzato** e richiede un tuo token separato.

## 2. Che cosa è stato misurato, e con quali controlli
Etichetta: `y = 1` se il target IK caviglia **decodificato** è negativo. Struttura misurata: **97 positivi su 500 righe (19,4 %) in 7 run contigui di lunghezze [9, 21, 6, 24, 5, 23, 9]**. Split: la geometria **congelata rev3n** (5 fold contigui da 100 con embargo 10, standardizzazione solo-train, costanti escluse per fold); i 5 fold **partizionano esattamente** le 500 righe, ciascuna held-out una volta sola. Nessuno split nuovo è stato inventato.

Due insiemi di feature, con una distinzione che conta:
- **runtime-available (33)**: tutte le 35 meno le due colonne di clock prescritto, che sono azzerate per costruzione. È ciò che la policy vede davvero.
- **physical-only (25)**, diagnostico: toglie anche l'intera famiglia di fase online (`online_left_gait_phase_sin/cos`, `cycle_duration`, `phase_expected_hs/to`, `stance/swing_elapsed_norm`, `cycle_progress_credit`). Serve a capire se la localizzazione si appoggia alla stima di fase online — che però è **legittimamente disponibile a runtime**, quindi il suo eventuale fallimento non è di per sé un NO-GO.

Controlli: null a permutazione a blocchi contigui di 25 righe (20 ripetizioni, seed 2026) per calibrare quanto la sola autocorrelazione temporale può produrre; contrasto di distanza nearest-neighbour cross-classe vs intra-classe per rilevare aliasing direttamente; riporto per fold.

## 3. Risultati
| | balanced acc | recall + | specificità | precisione + | var. residua / var. target |
|---|---|---|---|---|---|
| **runtime-available (33)** | **0,78485** | **0,63918** | 0,93052 | 0,68889 | **0,2026** |
| physical-only (25) | 0,76938 | 0,60825 | 0,93052 | 0,67816 | 0,1474 |
| null a blocchi | 0,48268 (max 0,65733) | — | — | — | — |

**Lo stato non è aliasato.** Sulle 380 righe di training, per le righe positive il vicino più prossimo dell'altra classe è mediano **2,107** contro **0,788** intra-classe (rapporto **2,675**), e solo l'**11,0 %** delle righe positive ha il vicino più prossimo nell'altra classe (2,0 % per le negative, rapporto 6,0). La finestra occupa una regione propria dello spazio di stato.

**Lo stato spiega circa l'80 % della varianza del target** (rapporto varianza residua/target 0,2026, RMSE 0,0659 con il set runtime; 0,1474 col physical-only).

**La localizzazione non dipende dalla fase online.** Togliendo l'intera famiglia di fase i numeri restano sopra soglia (0,769 / 0,608) e la varianza condizionale addirittura migliora. Quindi la finestra è leggibile dallo **stato fisico**, non da un surrogato di clock.

## 4. Eterogeneità per fold — il dato che non va nascosto
| fold | positivi in hold | runtime: ba / recall | physical-only: ba / recall |
|---|---|---|---|
| [1, 100] | 9 | **0,4725 / 0,0000** | 0,4951 / 0,1111 |
| [101, 200] | 27 | 0,8019 / 0,7407 | 0,8341 / 0,7778 |
| [201, 300] | 24 | 0,8410 / 0,7083 | 0,8684 / 0,7500 |
| [301, 400] | 5 | 0,9684 / 1,0000 | **0,4842 / 0,0000** |
| [401, 500] | 32 | 0,7757 / 0,6250 | 0,7601 / 0,5938 |

Il GO aggregato è **trainato dai fold 2, 3 e 5**, che contengono 83 dei 97 positivi. Nel **primo fold la finestra non è osservabile affatto** (recall 0,000: tutti e 9 i positivi mancati): è il transitorio iniziale dell'episodio, dove lo stato non somiglia a quello del resto della traccia. Il fold 4 ha soli 5 positivi ed è instabile fra i due set di feature (1,000 contro 0,000 di recall), quindi non porta informazione affidabile. Conseguenza operativa per S1C-2: **un target di finestra applicato alla finestra precoce (righe ~1–100) rischia di essere non apprendibile**, e questo va tenuto presente nel disegno, non aggirato.

## 5. Lettura causale rispetto a S1C-1
Poiché la finestra **è** in larga parte osservabile — classi non aliasate, 80 % di varianza spiegata — la frammentazione misurata in S1C-1 **non può essere attribuita all'aliasing dell'input**. L'informazione c'era; l'obiettivo puntuale non l'ha sfruttata. Al tempo stesso, recall 0,639 significa che **35 dei 97 positivi restano indistinguibili** per un kNN in questo spazio: l'osservabilità è statistica e non netta, quindi una parte della frammentazione resta plausibilmente attribuibile a un residuo di ambiguità. Le due affermazioni convivono e le riporto entrambe.

## 6. Confronto delle due famiglie di obiettivo (nessun allenamento)
**A — target supervisionato su finestre di fase/contiguità.** Compatibile in pieno con l'MLP causale 35D: la policy resta `s_t → a_t` senza memoria, l'etichetta è calcolata **offline** dal target già registrato, nessun input a runtime cambia e nessuna osservazione futura entra. Non richiede minibatch sequenziali: il meccanismo T1R a unione mescolata e medie per-ruolo group-balanced resta valido. Nessuna dipendenza da clock assoluto a runtime.

**B — loss di run-fragmentation / consistenza temporale.** **Richiede minibatch sequenziali**: la loss accoppia uscite a passi consecutivi, quindi i batch devono essere segmenti ordinati contigui invece dell'unione mescolata congelata — cambio strutturale del loop che invaliderebbe il meccanismo group-balanced così com'è. Sulla causalità: una run length in *t* richiede uscite in *t+1…*; una variante solo all'indietro (`a_t` vs `a_{t−1}`) è causale nella loss. In entrambi i casi **la policy resta causale**, perché una loss non causale in addestramento non rende non causale l'input a runtime — distinzione da dichiarare esplicitamente in qualunque protocollo futuro. Per evitare dipendenza da clock assoluto la loss deve usare **solo differenze relative** fra uscite consecutive dentro il segmento: se usasse il campo di fase o il tempo assoluto per definire le finestre reintrodurrebbe una dipendenza che la policy a runtime non può sfruttare, avendo le colonne di clock azzerate.

**Limite condiviso e decisivo:** nessuna delle due famiglie può creare informazione che l'input non ha. Se due righe fossero aliasate in 35D, una rete senza memoria dovrebbe emettere la stessa azione per entrambe e la frammentazione sarebbe irriducibile con qualunque loss. È esattamente per questo che l'audit **precede** la scelta invece di seguirla.

**Preferenza, congelata in rev3y prima dell'audit:** in caso di GO si sceglie **la famiglia A**, perché ottiene lo stesso obiettivo senza minibatch sequenziali, senza alcuna dipendenza dal futuro e senza toccare il meccanismo T1R congelato — cambiando una cosa sola alla volta.

## 7. Variante S1C-2 preregistrata (preparata, **non autorizzata**)
Init **esclusivamente S0D** `481dd0d2…` (August V26 imitation → 35D); W2/W4/W8, A1–A6, S1A, A2 vietati come init/source/anchor; **July solo come ancora parametrica 1e-5 e benchmark, mai init né label**; `logstd` ripristinata **bit-identica**; colonne clock ri-azzerate; σ **non scelta**.
Obiettivo famiglia A: due ruoli con **r = 5 invariato**; sulle righe di ciascuna finestra negativa contigua l'etichetta task diventa `(1−β_w)·puntuale + β_w·media della finestra`, con **β_w ∈ {0,50; 0,75; 1,00}** — β_w = 0 riprodurrebbe A2 ed è escluso. **Budget massimo 3 candidati**, ricerca chiusa.
Numerica invariata: 300 epoche, batch 256, Adam 1e-4, seed 2026, deterministico, nessun early stopping, nessun retry. Split invariati (380 train, holdout 201–300, embargo). **Gate P/T/D invariati**, con le soglie rev3v esattamente come congelate: nessuna aggiunta, rimozione o modifica. Diagnostiche di persistenza preregistrate identiche a rev3x (duty cycle, numero/media/max dei run, replay attraverso la replica validata rev3w, più S0D, A2 e il target IK come riferimenti), **non vincolanti e senza soglie**; B3 resta **INDETERMINATE** finché il campo di fase è nullo. Materializzazione in `candidates/S1C2_<ID>_35D_NONDEPLOYABLE`, e — per la regola introdotta oggi — **lo stato di rollout/promozione va derivato dal verdetto offline**, mai ereditato.

## 8. Correzione di stato dell'aggregate S1C-1 (addendum autoritativo)
`v26b_addendum_rev3x_a_aggregate_status_semantics.json` → `de53d529343edd5d82af786036368d7b885ad686af18469088a3dc8ccb563cf5`. L'aggregate e i tre receipt portavano `rollout_pending: true`, `promotion_requires` e `next_stage_locked` ereditati verbatim da rev3q, dove descrivevano un candidato **promosso** dallo stadio offline. Per W2/W4/W8, che hanno **fallito** P ed è per questo che sono quarantinati, nessun rollout è pendente, previsto o permesso. Lo stato autoritativo è quindi **`OFFLINE_FAILED_QUARANTINED`** per tutti e tre. Nessun dato, digest, verdetto o valutazione di gate cambia — **solo la semantica di stato** — e l'aggregate resta **byte-identico** (`58874483…`, riverificato prima e dopo). Regola in avanti: ogni stadio futuro deve **derivare** lo stato di rollout/promozione dal verdetto offline invece di ereditarlo incondizionatamente.

## 9. Limiti del dataset, dichiarati
Una **singola traiettoria deterministica** di 500 righe: 97 positivi ma **solo 7 finestre**, quindi il campione effettivo è di 7 eventi, non di 97 righe indipendenti. Gli stati sono quelli **visitati da S0D**: l'audit descrive l'osservabilità su *quella* distribuzione, non su stati che un altro attore raggiungerebbe. Il fold [301,400] contiene 5 positivi e le sue cifre per fold sono deboli. Nessuna nuova raccolta è stata fatta né è proposta qui.

## 10. Test
`test_v26b_s1c2_preflight.py`: **PASS, 37 check**. Copertura: pin/tamper di rev3y, dell'addendum di stato e dell'aggregate (che deve restare byte-identico con zero survivor); contenuto autoritativo dell'addendum; 4 token negativi e la guardia del fit futuro; assenza di primitive di fit/rollout; regola dell'etichetta e struttura dei 7 run verificate; insiemi di feature (33 e 25) con la famiglia di fase identificata **per nome**; prova che i 5 fold partizionano esattamente le 500 righe; correttezza delle metriche su matrici di confusione sintetiche; **logica GO/NO-GO su fixture con ciascuna delle tre condizioni che fallisce isolatamente**; contrasto di aliasing verificato su dati sintetici separabili contro puro rumore; parametri del null congelati; no-clobber e registrazione che nulla è stato eseguito, σ inclusa.

## 11. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_addendum_rev3x_a_aggregate_status_semantics.json` | `de53d529343edd5d82af786036368d7b885ad686af18469088a3dc8ccb563cf5` |
| `…/v26b_amendment_rev3y_s1c2_preflight.json` | `eb54fd266461e56728dd5387f6019a3eb23243eb4c4e0e79541cfeaf520139c6` |
| `…/v26b_s1c2_preflight.py` | `112eb2d1988c6bfb0694ea828dc0efa8ac01f44410d5a312fc257e5bac8343c1` |
| `…/test_v26b_s1c2_preflight.py` | `1fc3f8dccef43c62d26a954a7fd47374e170f3393bb5dd54a7a77e59da9203e5` |
| `…/s1c2_preflight/v26b_s1c2_preflight_20260824_211539.json` | `6504e4f44d88e41cfa5750c751e83fe4bb9c697427a4a285793c46aeccb0a2dc` |
| aggregate S1C-1 (**invariato**) | `58874483683ff3f9eb6741f7f752fbb5b9c8fec96b3277c3543f0eae58b7d3e4` |

## 12. Stato
**STOP.** Verdetto **GO** secondo il criterio congelato; protocollo S1C-2 **preparato e non eseguito**; nessun fit, rollout o promozione; gate e soglie rev3v intatti; σ non scelta; production invariata. Il fit richiede un tuo token esplicito.

Due cose che ti segnalo prima che decidi: il GO poggia su tre fold su cinque e **la finestra precoce non è osservabile**, quindi suggerirei di considerare, in sede di tua revisione del protocollo, se applicare il target di finestra solo alle finestre in regime stabile; e resta vero che con recall 0,639 un terzo delle righe di finestra è ancora ambiguo, quindi anche la famiglia A potrebbe non azzerare la frammentazione. Non ho modificato rev3y su nessuno di questi due punti: sono osservazioni per la tua decisione.
