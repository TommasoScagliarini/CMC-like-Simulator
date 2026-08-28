# V26B B1R2-B — esito: **NO-GO** e **ipotesi falsificata**, con curva convergente

**Data**: 2026-08-25
**Stadio**: `V26B-B1R2B-LR`, screening a una variabile · **eseguito una sola volta**
**Esito**: **NO-GO** su 6 gate di 9 · **ipotesi di ripidità FALSIFICATA** · nessun candidato promosso
**Nessun rollout, nessuna collection, nessun DAgger, nessun altro braccio.** Nessuna soglia toccata, nessun iperparametro corretto post-hoc.

Report **additivo**. Non sostituisce nulla: il report B1R2-A (`18dc7e16…`) e l'addendum correttivo (`6243495d…`) restano agli atti invariati.

---

## 1. L'esito in una riga

La curva pooled completa **è convergente** al budget (argmin globale 1093 ≤ 1140, seguito da 60 epoche senza miglioramento), quindi i falsificatori **si applicano**: non è uno screening non conclusivo. **Entrambi scattano.** L'ipotesi che un passo di ottimizzazione più fine permettesse di raggiungere una funzione più ripida capace di separare testa e ciclo1 è **falsificata**.

E lo è nel modo più informativo possibile: il meccanismo previsto ha funzionato — il rumore della curva è sceso da 44% a **31,4%** — ma **ogni** misura di generalizzazione è peggiorata.

## 2. Convergenza: questo braccio è leggibile, il braccio A non lo era

| | braccio A (lr 3e-4) | **braccio B (lr 1e-4)** |
|---|---|---|
| arresto della scansione di selezione | 297 | 374 |
| **argmin globale curva completa** | **1171** | **1093** |
| entro l'headroom (≤1140)? | **NO** | **SÌ** |
| nessun miglioramento nelle 60 successive | — | **SÌ** |
| **curva completa convergente al budget** | **NO** | **SÌ** |
| falsificatori leggibili | no | **sì** |

Le due grandezze restano distinte, come da correzione 3: nel braccio B la scansione si è fermata a 374 mentre il minimo vero è a 1093. Che la scansione si fermi non dice nulla sulla convergenza; è la definizione full-curve a deciderlo, e qui dà esito positivo.

Curva pooled per epoca: 100 → 0,003626 · 200 → 0,003096 · 400 → 0,002953 · 600 → 0,002532 · 800 → 0,002226 · 1000 → 0,002242 · 1200 → 0,002266. Il livello si appiattisce e **risale** dopo l'800: la curva è arrivata.

## 3. I due falsificatori, entrambi scattati

| falsificatore | soglia | osservato | esito |
|---|---|---|---|
| **F1** — fold 7 e 9 sopra 0,05 al **proprio best** su 1200 epoche | 0,05 | nominal testa **0,06185** · plus020 testa **0,06445** | **SCATTATO** |
| **F2** — pooled a e\* non scende sotto l'oracle a epoca singola del braccio A | 0,04120 | **0,05204** | **SCATTATO** |

F1 è il più importante: le due teste falliscono **al proprio minimo su 1200 epoche**, cioè in una condizione immune tanto alla selezione quanto al rumore. Falliscono così in **entrambi** i regimi di ottimizzazione testati. F2 mostra che il braccio B non solo non batte il braccio A: fa **peggio** del suo miglior singolo punto.

## 4. Il braccio B è peggiore del braccio A su ogni misura

| misura | braccio A | **braccio B** | gate |
|---|---|---|---|
| pooled validation RMSE a e\* | 0,04805 | **0,05204** | 0,03 |
| minimo curva completa (RMSE) | 0,04120 | **0,04476** | — |
| **oracle per-fold** (irraggiungibile) | 0,03599 | **0,04020** | 0,03 |
| RMSE finale aggregata (1500 righe) | 0,03437 | **0,03934** | 0,02 |
| knee rmse / max_abs finale | 0,03936 / 0,18824 | **0,04410** / 0,16920 | 0,03 / 0,15 |
| ankle rmse / max_abs finale | 0,02853 / 0,12657 | **0,03392** / 0,13368 | 0,03 / 0,15 |
| e\* selezionato | 237 | 314 | — |
| spread epoca-per-epoca attorno a e\* | 44% | **31,4%** | — |

**Il rumore è effettivamente sceso del 29%**, come l'ipotesi prevedeva meccanicamente. Ma non si è tradotto in nulla: l'oracle per-fold del braccio B (0,04020) è peggiore di quello del braccio A (0,03599), ed entrambi restano sopra il gate 0,03. Anche in questo braccio, **nessuna regola di selezione dell'epoca potrebbe passare il gate pooled.**

## 5. Gate a e\*=314

| gate | soglia | osservato | esito |
|---|---|---|---|
| invarianti di integrità | — | tutte vere | **PASS** |
| equivalenza funzionale 25D | bit-exact | bit-identica | **PASS** |
| ogni fold migliora su B0 | — | 11/11 | **PASS** |
| RMSE heldout per fold | ≤0,05 | 4 fold sopra soglia | **FAIL** |
| RMSE pooled validation | ≤0,03 | **0,05204** | **FAIL** |
| RMSE finale aggregata | ≤0,02 | **0,03934** | **FAIL** |
| RMSE finale per giunto | ≤0,03 | knee **0,04410** · ankle **0,03392** | **FAIL** |
| max_abs finale per giunto | ≤0,15 | knee **0,16920** · ankle 0,13368 | **FAIL** |
| ricostruzione WAIT | 0,02/0,03/0,15 | agg **0,03351** · knee **0,03389** / **0,16238** | **FAIL** |

## 6. Fold: dove sta l'errore

| fold | tipo | anchor | n | B a e\* | B al proprio best | argmin B | A a e\* | A al proprio best |
|---|---|---|---|---|---|---|---|---|
| 0 | ciclo | minus020 c1 | 148 | **0,05968** | 0,03929 | 1197 | 0,05439 | 0,03593 |
| 1 | ciclo | minus020 c2 | 154 | 0,03299 | 0,02164 | 1110 | 0,03114 | 0,01624 |
| 2 | ciclo | nominal c1 | 151 | 0,04879 | 0,03675 | 1008 | 0,03953 | 0,03331 |
| 3 | ciclo | nominal c2 | 153 | 0,03768 | 0,02055 | 967 | 0,02392 | 0,01578 |
| 4 | ciclo | plus020 c1 | 151 | 0,04773 | 0,03853 | 753 | 0,05343 | 0,03792 |
| 5 | ciclo | plus020 c2 | 153 | 0,03282 | 0,01856 | 1034 | 0,03715 | 0,01394 |
| 6 | coda | minus020 | 11 | 0,03105 | 0,00644 | 1092 | 0,02450 | 0,00484 |
| **7** | **ciclo** | **nominal testa** | 164 | **0,06725** | **0,06185** | 764 | 0,06168 | 0,05413 |
| 8 | coda | nominal | 32 | 0,04797 | 0,01632 | 1095 | 0,02828 | 0,01253 |
| **9** | **ciclo** | **plus020 testa** | 144 | **0,07319** | **0,06445** | **100** | 0,06680 | 0,05947 |
| 10 | coda | plus020 | 52 | **0,05868** | 0,02832 | 1117 | 0,05892 | 0,02038 |

Falliscono a e\* i fold 0, 7, 9, 10. Ma solo **7 e 9 falliscono anche al proprio best**: le due teste restano il difetto strutturale, ora confermato in due regimi di ottimizzazione distinti e con margini peggiorati (0,05413 → 0,06185 e 0,05947 → 0,06445).

Nota su fold 9: il suo minimo cade all'epoca **100** e poi peggiora fino a 0,07319 a e\*=314. Su quel fold l'addestramento successivo è attivamente controproducente.

## 7. Regioni (fit finale, tutte le 1500 righe)

| regione | righe | B agg | A agg | B knee | B ankle | B max_abs |
|---|---|---|---|---|---|---|
| cicli coperti da B1R1 | 910 | 0,03517 | 0,03026 | 0,03690 | 0,03336 | 0,12179 |
| **B1R1-uncovered startup/tails** | 403 | **0,04947** | 0,04463 | 0,06027 | 0,03551 | **0,16920** |
| cicli strutturalmente completi | 1218 | 0,04002 | 0,03572 | 0,04491 | 0,03445 | 0,16920 |
| code | 95 | 0,04121 | 0,02929 | 0,05095 | 0,02829 | 0,09888 |
| WAIT | 187 | 0,03351 | 0,02703 | 0,03389 | 0,03311 | 0,16238 |

Ogni regione peggiora. Il ginocchio resta il giunto dominante ovunque.

## 8. Controllo di integrità: **coincide esattamente**

Il conflitto testa↔ciclo1 ha riprodotto **tutti e undici** i valori pinnati in uguaglianza esatta in virgola mobile: 188 righe testa-swing, 262 ciclo1-swing, 49 256 coppie, 92 sotto 0,15, mediana \|Δtarget\| **0,1670604795217514**, distanza minima 0,030914171900546916, controllo interno ciclo1 0,04486663639545441 e testa 0,0.

Ha funzionato esattamente come progettato, in entrambe le direzioni: la pipeline dati è intatta (il run è valido) **e** la quantità si è confermata invariante al learning rate, come previsto dalla correzione 1. Se fosse stata lasciata fra i falsificatori avrebbe dato un "non scattato" privo di significato.

## 9. Cosa questo esito **non** dice

Per la correzione 2, vincolante e registrata nel receipt: **un NO-GO in questo braccio non stabilisce un limite del contratto osservativo**, e non lo affermo. La convergenza della curva rende leggibili i falsificatori sull'ipotesi di ripidità, non autorizza una conclusione sul contratto: restano non testate altre leve (capacità, ripesatura per regime, feature aggiuntive), e ciascuna richiede una tua autorizzazione esplicita.

Non affermo nemmeno una non-identificabilità: **non esistono collisioni esatte** (distanza minima fra righe 0,0309; l'audit rev1 non ha trovato coppie entro 1e-3). La sovrapposizione locale con disaccordo elevato resta compatibile sia con un limite del contratto sia con un limite rappresentativo non raggiungibile per questa via.

Nessun confronto con il 6,63e-05 di luglio, che proviene da uno split casuale su dataset ripetuto 32× e non è una misura di generalizzazione.

## 10. Integrità e artefatti

Verificati byte-identici: B0 `aa7ea0fa…`, B1 `e7328ac2…`, B1R1 `6a604ada…` / `9ffcdceb…`, B1R2 rev1 `d0f1e515…`, B1R2-A `a4ebd8d6…`.

`candidates/B1R2B_BASE35_LR1E4` **non esiste** (né la sua staging): nessuna promozione, come impone il NO-GO. Gli artefatti stanno in `diagnostics/b1r2b/`, manifest `deployable: false`, `offline_verdict: NO-GO`.

| file | SHA-256 |
|---|---|
| `diagnostics/b1r2b/v26b_b1r2b_receipt.json` | `abf36dc16179aab91cdc0463a8ba8c9d55d070271a505cd9f16d2b8fbca7647d` |
| `diagnostics/b1r2b/rl_module/module_state.pkl` | `c961c3244d9f3f24f707ede5f0fc78614d125a9291bac2ab353e798fdf724725` |
| `diagnostics/b1r2b/rl_module/actor_feature_manifest.json` | `1dcec13306f20709450ef1d802ea57000856f0522f249ad2920ad19376723d9c` |
| `v26b_amendment_b1r2b_lr.json` | `0bf106e79ba4def044cfabb5ad7e264c238a9983284d5b43a7a93778fcfd10ce` |
| `v26b_b1r2b_exec.py` | `2fe438c1bb13b24a540a94f553b629435b5a58ce00f00c782ab8f7a3398e6d19` |
| `test_v26b_b1r2b_exec.py` | `ca92d987b2b1a92e2b04535db0fcba14cd1168d3d9ac77eac7ed30b54b53d93c` |

**Comando eseguito**, una sola volta:
```
python v26b_b1r2b_exec.py --authorized-stage V26B-B1R2B-LR
```

Preflight GO precedente all'esecuzione; 791 check su 7 suite tutti PASS. Produzione, FSM v3, morfologia, contatto, reward, C++ e SEA invariati.

## 11. Cosa non ho fatto

Nessun rollout, collection, DAgger o braccio successivo. Nessuna soglia rilassata, nessun iperparametro toccato dopo aver visto i numeri, nessuna riesecuzione. Il NO-GO è registrato come tale e l'ipotesi come falsificata. **Attendo la tua revisione.**

## 12. TODO propagati

- **TODO-2 · 3 · 4 · 5 · 7 · 9** — invariati, aperti.
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — Le due teste restano il difetto dominante e falliscono al proprio best in **entrambi** i bracci (0,06185 e 0,06445 in B; 0,05413 e 0,05947 in A). *(aggiornato)*
- **TODO-16** — Patience/selection: confermata non causale. L'oracle per-fold fallisce il gate in entrambi i bracci (0,03599 in A, 0,04020 in B, contro 0,03). *(aggiornato)*
- **TODO-17** — Ipotesi di ripidità raggiungibile via ottimizzazione più fine: **FALSIFICATA** con curva convergente. Resta aperto se il limite sia rappresentativo per altra via o del contratto: non deciso e non decidibile da questo braccio. *(aggiornato)*
- **TODO-18** — `max_abs` WAIT: migliorato da 0,18824 a 0,16238, ancora sopra 0,15. *(aggiornato)*
- **TODO-19** — Il best val MSE di luglio non è una misura di generalizzazione. *(aperto)*
- **TODO-20** — Braccio a budget maggiore: **non più giustificato da questo esito**, la curva B è convergente. *(chiuso)*
- **TODO-21** — Il braccio A non era convergente (argmin 1171 > 1140); il braccio B lo è (1093). Il NO-GO di A resta da non sovra-interpretare. *(aggiornato)*
- **TODO-22** — Nuovo: sul fold 9 il minimo cade all'epoca 100 e l'addestramento successivo peggiora attivamente fino a 0,07319. *(nuovo)*
