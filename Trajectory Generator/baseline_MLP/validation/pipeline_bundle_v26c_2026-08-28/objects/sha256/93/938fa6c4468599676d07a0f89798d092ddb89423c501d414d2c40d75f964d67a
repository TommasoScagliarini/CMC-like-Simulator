# V26B B1R1 — studio read-only dello split. Causa del NO-GO B1 e proposta

**Data**: 2026-08-25 · **Natura**: sola lettura. Nessun training, nessuna modifica di actor, nessun rollout.
**Receipt**: `runs/.../diagnostics/b1r1_split_study/v26b_b1r1_split_study.json` — SHA-256 `6e4821d102c025fb20e550159fe052e13db2787d4f2d454dbfeeaab4160a2698`
**Il NO-GO di B1 resta definitivo e immutato**: receipt `e7328ac2…` e module_state `778ff748…` verificati byte-identici.

---

## 1. La causa del fallimento del fold 0, trovata

`phase_fsm_wait_hs = 1` esiste in **una sola traiettoria**, minus020, e vi occupa le **prime 187 righe contigue** (rows 1–187, 37,4% di quella traiettoria). Nominal e plus020 **non ne hanno nemmeno una**.

Sotto LOTO con minus020 in validation, il training è nominal + plus020 e contiene quindi **zero righe WAIT_HS**, mentre 187 delle 500 righe da predire lo sono. Quel fold non era un test di generalizzazione: era un'**estrapolazione su uno stato discreto mai visto**. Coincide con il 47,4% fuori supporto che avevo misurato e con la sua RMSE held-out di 0,17640, 3,4–3,8× le altre due.

## 2. Inventario delle feature FSM vive (indici 17–24)

| | minus020 | nominal | plus020 |
|---|---|---|---|
| WAIT_HS | **187** | 0 | 0 |
| STANCE | 131 | 227 | 217 |
| SWING | 182 | 273 | 283 |
| expected_hs / expected_to | 369 / 131 | 273 / 227 | 283 / 217 |
| eventi HS / TO | 3 / 2 | 3 / 3 | 3 / 4 |
| stato iniziale | **WAIT_HS** | STANCE | STANCE |
| transizioni | 5 | 7 | 7 |

Transizioni (riga 1-based):
- **minus020**: 188 WAIT_HS→STANCE · 248 STANCE→SWING · 336 SWING→STANCE · 396 STANCE→SWING · 490 SWING→STANCE
- **nominal**: 72 · 165 · 229 · 316 · 377 · 469 · 500
- **plus020**: 50 · 145 · 209 · 296 · 358 · 449 · 491

**22 segmenti FSM** massimali: 1 WAIT_HS (187 righe), 11 STANCE (575), 10 SWING (738). Il segmento più corto è 1 riga (nominal, riga 500).

## 3. Split proposto — grouped/blocked temporal, deterministico e fail-closed

```
1. segmentare ogni traiettoria in run massimali di stato FSM costante;
2. suddividere ogni run in blocchi contigui quasi uguali di al più 40 righe;
3. ordinare i blocchi per (stato, traiettoria, riga iniziale) — ordine totale e riproducibile;
4. fold = rango-nella-propria-classe-di-stato modulo 5, così ogni classe è distribuita
   round-robin fra i fold;
5. validation(f) = le righe dei blocchi assegnati a f;
6. train(f) = ogni altra riga la cui distanza temporale da QUALSIASI riga di validation
   della STESSA traiettoria supera l'embargo;
7. fallire chiuso se uno stato presente in validation(f) è assente da train(f).
```

**Parametri: blocco ≤ 40 righe, 5 fold, embargo 20.** Ciascuno è fissato da un criterio, non per convenzione.

**Perché 40 righe.** È il valore che spezza l'unico segmento WAIT_HS in **esattamente 5 blocchi**, cioè uno per fold: ogni fold *testa* quello stato e ogni fold lo *ha* in training. Con blocchi da 50 i blocchi WAIT_HS diventano 4 e un fold non lo testerebbe affatto.

**Perché embargo 20.** Misurato sulle 25 feature vive: la distanza mediana fra due righe della stessa traiettoria a lag *k*, rapportata alla distanza fra righe scorrelate (6,6465):

| lag | 1 | 5 | 10 | 15 | **16** | 20 | 30 |
|---|---|---|---|---|---|---|---|
| decorrelazione | **6,5%** | 20,9% | 35,8% | 49,8% | **52,7%** | **67,5%** | 89,7% |

Regola: il più piccolo multiplo di 5 step con decorrelazione mediana ≥ 50%. L'attraversamento è a lag 16, quindi 15 (49,8%) non basta e **20** (67,5%) è il minimo ammissibile.

### Quantificazione per fold

| fold | val | train | scartate | WAIT val / train | STANCE val | SWING val | dist. min | oltre p99 | nn-RMSE | knee | ankle |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 295 | 875 | 330 | 37 / 110 | 128 | 130 | **21** | 40,7% | 0,0865 | 0,0823 | 0,0906 |
| 1 | 314 | 896 | 290 | 37 / 110 | 128 | 149 | **21** | 26,4% | 0,0822 | 0,1004 | 0,0588 |
| 2 | 308 | 901 | 291 | 38 / 109 | 117 | 153 | **21** | 21,4% | 0,0976 | 0,0686 | 0,1197 |
| 3 | 300 | 857 | 343 | 37 / 110 | 108 | 155 | **21** | 32,3% | **0,2048** | 0,2512 | 0,1442 |
| 4 | 283 | 882 | 335 | 38 / 129 | 94 | 151 | **21** | 44,9% | 0,1185 | 0,1496 | 0,0755 |

Ogni riga è validata **esattamente una volta**; i validation set sono disgiunti e coprono le 1500 righe. Nessuno stato manca dal training in nessun fold. Nessuna riga di validation ha un vicino temporale immediato nel training. `nn-RMSE` è la baseline **senza modello**: la label del vicino più prossimo nel training, nella stessa metrica a 25 feature.

## 4. Confronto fra i tre split

| | fold | stati mancanti dal train | leakage temporale | dist. min | nn-RMSE peggiore / media | oltre p99 peggiore |
|---|---|---|---|---|---|---|
| **candidato grouped/blocked** | 5 | **nessuno** | **no** | **21** | 0,2048 / 0,1179 | 44,9% |
| leave-one-trajectory-out | 3 | **WAIT_HS** (fold minus020) | no | — | 0,0678 / 0,0531 | 47,4% |
| random 80/20 di luglio | 1 | nessuno | **sì** | **1** | 0,0373 / 0,0373 | 0,7% |

**Il random 80/20 non è accettabile.** La sua distanza temporale minima è **1 step**, e il **97,7%** delle sue righe di validation ha un vicino immediato nel training. A lag 1 due righe sono decorrelate solo al **6,5%**: la validation è quasi una copia del training. La sua nn-RMSE bassissima (0,0373) e il suo 0,7% fuori supporto **sono artefatti del leakage**, non prove di un problema facile.

**LOTO non è accettabile** per il motivo del §1: viola la copertura di stato.

## 5. Regola unica per il best epoch, e configurazione unica di epoche/patience

**POOLED-FOLD EARLY STOPPING.** A ogni epoca *e* si calcola la MSE di validation media sui 5 fold, pesata per righe di validation. Si sceglie *e\** come la **prima** epoca che raggiunge il minimo di quella curva aggregata, applicando la patience di luglio alla curva aggregata. Il fit finale addestra su tutte le 1500 righe per esattamente *e\** epoche, senza validation.

**Epoche max 400, patience 60** — i valori storici pinnati di luglio, invariati.

*Giustificazione da luglio*: la regola di luglio è early stopping sulla MSE di validation con patience 60, massimo 400 epoche e ripristino dei pesi migliori. 400 e 60 sono i valori pinnati; cambiarli sarebbe una scelta post-hoc. La gestione dei pareggi (prima epoca minimizzante) è quella che il confronto stretto `val_mse < best - 1e-9` di luglio già produce.

*Giustificazione dalla geometria*: lo split distribuisce ogni classe di stato round-robin, quindi i 5 validation set sono **scambiabili per costruzione** — ciascuno contiene un blocco WAIT_HS (37–38 righe) e quote comparabili di STANCE e SWING. Sotto scambiabilità la media pesata per righe è la combinazione naturale delle 5 curve e il suo argmin è un singolo intero deterministico. Una mediana di 5 argmin è invece una statistica di 5 argmin rumorosi, richiede una regola di pareggio per K pari e scarta la profondità di ciascuna curva.

**Nessun risultato di B1 è usato per scegliere questa regola o questi numeri.**

## 6. Gate numerici candidati — nessuno rilassato

| gate | soglia | rispetto a B1 |
|---|---|---|
| ogni fold migliora sull'init base | — | invariato |
| RMSE held-out per fold | ≤ 0,05 | **invariato** |
| RMSE di validation aggregata pesata per righe | ≤ 0,03 | **invariato** |
| RMSE finale su tutte le righe | ≤ 0,02 | **invariato** |
| RMSE finale per giunto | ≤ 0,03 | **invariato** |
| max_abs finale per giunto | ≤ 0,15 | **invariato** |
| **RMSE held-out per stato FSM**, WAIT_HS incluso | ≤ 0,05 | **NUOVO e più severo**: B1 non aveva alcun gate stratificato per stato |

## 7. Il rischio che devo dichiarare

La baseline senza modello del candidato è **0,2048 nel fold peggiore e 0,1179 in media**, contro un gate per fold di 0,05. Un regressore liscio può battere il vicino più prossimo di molto — la baseline **non è un limite inferiore** su ciò che una rete può raggiungere — ma è un indicatore onesto di difficoltà, e questo split è circa **2× più difficile di LOTO** con quella misura.

Va letto insieme al §4: i numeri bassi di LOTO e del random non descrivono un problema più facile. LOTO ha una media bassa perché due fold su tre sono facili e il terzo è rotto; il random ha numeri bassi perché perde. **Il candidato è più difficile perché è l'unico onesto.** Non propongo di rilassare nulla; segnalo che il gate a 0,05 per fold, su questo split, è una richiesta severa.

## 8. File

| file | SHA-256 |
|---|---|
| `v26b_b1r1_split_study.py` | `e61c082ddb556ecd6480e6113f8fef12543fcf8269942f015418bef6fc7f42ef` |
| `test_v26b_b1r1_split_study.py` | `08a9562502a21d79e25dcf88aeee7625168a295dfe353061d17e8e3d4bb8c886` |
| receipt dello studio | `6e4821d102c025fb20e550159fe052e13db2787d4f2d454dbfeeaab4160a2698` |

```
python test_v26b_b1r1_split_study.py   -> {"selftest": "PASS", "checks": 81}
python v26b_b1r1_split_study.py --run  -> receipt scritto
```

Il test verifica che lo script **non importi torch, optim, backward, adapt_actor, subprocess o rollout_eval** e non scriva alcun `module_state.pkl`. Verifica inoltre che il solo call site `.permutation(` sia dentro il comparatore random, che lo split sia deterministico fra chiamate, e che un `block_max` che tenga WAIT_HS intero **fallisca chiuso** con la violazione nominata.

`git status` conferma zero modifiche a `osim_trj_cmc_like.py`, `prosthetic_phase_fsm.py`, `online_grf.py`, `reward_function.py`, `target_domain_imitation.py`, al config v3 e a `tools/`. B0 e il NO-GO di B1 sono byte-identici.

## 9. Decisioni che spettano a te

1. Se approvare blocco 40 / 5 fold / embargo 20, o altri parametri.
2. Se accettare la regola pooled-fold early stopping al posto della mediana degli argmin.
3. Se aggiungere il gate per stato FSM a 0,05, che è più severo di quanto B1 avesse.
4. Se il gate per fold a 0,05 vada mantenuto su uno split la cui baseline senza modello è 0,20 nel fold peggiore — sapendo che rilassarlo sarebbe una modifica di soglia, che hai vietato.

Nessun fit avviato. Attendo.

## 10. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione in B0/B1. *(indirizzato)*
- **~~TODO-12~~** — **CHIUSO**: la causa è identificata (WAIT_HS presente in una sola traiettoria) e lo split proposto la risolve per costruzione.
- **TODO-13** — Nuovo: il gate per fold a 0,05 su uno split senza leakage è severo rispetto alla baseline senza modello (0,20 nel fold peggiore). Da decidere prima del fit, non dopo. *(nuovo)*
