# V26B B1R1 — studio LOCO corretto: 6 fold leave-one-complete-cycle-out

**Data**: 2026-08-25 · **Natura**: sola lettura. Nessun training, nessuna modifica di actor, nessun rollout.
**Receipt**: `runs/.../diagnostics/b1r1_loco_study/v26b_b1r1_loco_study.json` — `1da3acc7e785eabefb673210cbea0c1868bcb0916e1b476254746fd5bc49e14c`
**Addendum di supersessione**: `v26b_addendum_b1r1_blocked5_comparator.json` — `2781bc72d2b19e5a36eb6d31e6b73579c2ae62996f01288d5994aa732496641a`

Il NO-GO di B1 (`e7328ac2…`), B0 (`aa7ea0fa…`) e lo studio superseduto (`6e4821d1…`, script `e61c082d…`) sono verificati **byte-identici**.

---

## 1. Le tre correzioni, accolte e verificate

**Supporto 23D, non 25D.** `kept_columns` meno `CLOCK_COLUMNS` = colonne **2..24**. Avevo passato 25 colonne affidandomi a un filtro di varianza nulla per scartare implicitamente il clock: funzionava, ma era implicito. Ora è esplicito e il test verifica che supporto ∪ clock ricostruisca esattamente `kept_columns`. Le sole colonne globalmente costanti sono 0 e 1; tutti e sei i fold hanno **23 dimensioni vive**.

**Scaler solo su train.** Era un difetto reale: standardizzavo con media e deviazione dell'**intero** dataset, facendo entrare le statistiche di validation nella metrica stessa. Ora lo scaler è fittato sulle sole righe di training, e il test verifica che i due scaler **differiscano davvero** (correzione materiale, non cosmetica) e che `support_and_baseline` non contenga alcuna statistica full-dataset.

**Firme discrete esatte.** Non più le sole 3 classi one-hot ma la **tupla esatta su (17, 18, 19, 20, 21, 24)**. Verificato che 22 e 23 sono continue (412 e 557 valori distinti) e che **col 24 `phase_cycle_progress_credit` è categorica a 4 livelli {0, 0.25, 0.5, 1.0}**. Il dataset contiene **5 firme distinte** in totale.

## 2. La verifica formale su WAIT_HS — la tua affermazione è confermata

Nel dataset esiste **esattamente una** run di WAIT_HS: righe globali 0..186, 187 righe, nella sola traiettoria minus020.

Il segmento **non rivisita mai uno stato**: la frazione di coppie di righe distanti più di 20 step la cui distanza in 23D è inferiore alla mediana fra righe adiacenti è **0,0000**. A embargo 20 il vicino WAIT sopravvissuto più prossimo è **6,22×** più lontano di un vicino a lag 1 (mediana 2,177 contro 0,350; minimo 1,441).

Ne segue formalmente che qualunque heldout di righe WAIT è necessariamente uno di tre casi:
1. **fuga temporale** — restano righe di training adiacenti, che a lag 1 sono quasi duplicati;
2. **estrapolazione lungo il segmento unico** — con l'embargo non esiste alcuna ripetizione indipendente da cui generalizzare, perché la ricorrenza è nulla;
3. **fuori distribuzione** — si tiene fuori tutto il segmento e il training non ne ha nessuna riga: è il caso LOTO che ha fatto fallire il fold 0 di B1.

**Non esiste un heldout WAIT in-distribution in questi dati.** Non lo presento come tale.

## 3. Lo split proposto: 6 fold leave-one-complete-cycle-out

Un ciclo completo STANCE+SWING di **una sola** traiettoria per fold, embargo 20 sulla stessa traiettoria. I sei cicli sono verificati dal test come coppie esatte (segmento STANCE, segmento SWING successivo) della segmentazione FSM, non intervalli arbitrari.

| fold | traiettoria | ciclo | range locale | val | train | scartate | WAIT in train | persi | dist. min | firme | dim | oltre p99 | nn-RMSE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | minus020 | 1 | 187:334 | 148 | 1312 | 40 | **167/187** | **20** | 21 | ok | 23 | 12,8% | 0,0459 |
| 1 | minus020 | 2 | 335:488 | 154 | 1315 | 31 | 187/187 | 0 | 21 | ok | 23 | 9,1% | 0,0381 |
| 2 | nominal | 1 | 164:314 | 151 | 1309 | 40 | 187/187 | 0 | 21 | ok | 23 | 0,0% | 0,0640 |
| 3 | nominal | 2 | 315:467 | 153 | 1307 | 40 | 187/187 | 0 | 21 | ok | 23 | 5,9% | 0,0309 |
| 4 | plus020 | 1 | 144:294 | 151 | 1309 | 40 | 187/187 | 0 | 21 | ok | 23 | 0,7% | 0,0486 |
| 5 | plus020 | 2 | 295:447 | 153 | 1307 | 40 | 187/187 | 0 | 21 | ok | 23 | 0,0% | 0,0299 |

Copertura delle firme: 2–3 firme distinte in ogni validation, **5 in ogni training, zero mancanti**. Nessuna riga di validation ha un vicino temporale immediato nel training. Nessun ciclo tenuto fuori contiene una singola riga WAIT — il test lo verifica e rifiuta chiuso un ciclo che ne contenga.

**Una tensione che devo dichiarare.** Il ciclo del fold 0 inizia a local 187, cioè immediatamente dopo la fine del segmento WAIT a 186. L'embargo di 20 arretra quindi **dentro** il segmento e rimuove 20 righe WAIT dal training di quel fold: 167 su 187, l'89,3%. È la scelta priva di fuga: conservarle metterebbe una riga di training a un passo da una di validation. Ho reso il costo un diagnostico misurato con un floor fail-closed all'80%, invece di un'assunzione silenziosa. Riguarda **solo** il fold 0.

## 4. Confronto sotto strumenti corretti (23D, scaler train-only)

| | fold | firme mancanti | fuga | dist. min | nn-RMSE peggiore / media | oltre p99 peggiore | train min | scartate mediane | righe validate |
|---|---|---|---|---|---|---|---|---|---|
| **LOCO 6 fold** | 6 | **no** | **no** | **21** | **0,0640 / 0,0429** | **12,8%** | **1307** | **40** | 910 |
| blocked 5 (superseduto) | 5 | no | no | 21 | 0,2100 / 0,1186 | 44,9% | 857 | 330 | 1500 |
| LOTO 3 | 3 | **sì** | no | — | 0,0691 / 0,0531 | 47,6% | 1000 | 0 | 1500 |
| random 80/20 luglio | 1 | no | **sì** | **1** | 0,0373 / 0,0373 | 0,7% | 1200 | 0 | 300 |

La tua diagnosi era corretta e la tua proposta è nettamente migliore: **nn-RMSE peggiore 3,3× più bassa**, fuori supporto **3,5× più basso**, embargo **8× meno costoso**, e 450 righe di training in più nel fold peggiore.

Le correzioni di misura hanno spostato poco i numeri del design superseduto (0,2048→0,2100 peggiore, 0,1179→0,1186 media, 44,88% invariato): erano difetti metodologici reali ma non erano loro a rendere quel design fragile. Era la geometria.

**Il prezzo del LOCO**, che dichiaro: solo **910 righe su 1500** entrano mai in una validation. Le 590 restanti sono 187 WAIT più 403 di cicli parziali di testa e coda. È inerente al validare cicli completi.

## 5. Gate di ricostruzione WAIT — proposto, non rilassato

Poiché non esiste un heldout onesto, WAIT resta nel training di ogni fold ed è sottoposto a un gate **separato e dichiarato per quello che è**:

| misura sul fit finale all-data, sole righe WAIT_HS | soglia |
|---|---|
| RMSE aggregata | ≤ **0,02** |
| RMSE per giunto | ≤ **0,03** |
| max_abs per giunto | ≤ **0,15** |

Sono **gli stessi numeri già preregistrati per il fit finale all-data**, non allentati. Motivazione: le righe WAIT sono righe di *training* in ogni fold, quindi questo è un requisito di **ricostruzione**, non di generalizzazione, e come tale deve essere almeno altrettanto severo — mai più lasco.

**Dichiarazione di onestà, inclusa nel receipt e verificata dal test**: questo gate non va mai riportato come prova di generalizzazione su WAIT.

## 6. Alternative oneste considerate

Ho valutato se esista un heldout WAIT migliore. Non esiste: il §2 lo dimostra formalmente e le tre alternative sono tutte disoneste in un modo o nell'altro. L'unica variante concepibile — tenere fuori un blocco interno del segmento con embargo su entrambi i lati — ricade nel caso 2, estrapolazione lungo l'unico segmento, perché la ricorrenza misurata è **esattamente zero**. La riporto come esclusa, non come opzione.

Lo split è selezionato **solo dalla geometria delle osservazioni**: segmentazione FSM, struttura dei cicli, distanza temporale, supporto in 23D e copertura delle firme discrete. Nessuna action label entra nella scelta; il baseline nearest-neighbour usa le label ma è marcato **diagnostico e mai un gate**, sia nel codice sia nel receipt.

## 7. File

| file | SHA-256 |
|---|---|
| `v26b_b1r1_loco_study.py` | `bd94ade42b09fa296d4e0711602cabce63024f78ec39e7d0b50bf9a57a93fa58` |
| `test_v26b_b1r1_loco_study.py` | `5b40a9c1566169399e7efcfdf244579dda766878fb221171966b75889a7b6d8f` |
| `v26b_addendum_b1r1_blocked5_comparator.json` | `2781bc72d2b19e5a36eb6d31e6b73579c2ae62996f01288d5994aa732496641a` |
| receipt dello studio | `1da3acc7e785eabefb673210cbea0c1868bcb0916e1b476254746fd5bc49e14c` |

```
python test_v26b_b1r1_loco_study.py   -> {"selftest": "PASS", "checks": 87}
python v26b_b1r1_loco_study.py --run  -> receipt scritto
```

Il test verifica fra l'altro: il supporto 23D esplicito, che lo scaler train-only **differisca** da quello full-dataset, le firme su 6 colonne con rilevamento di una firma mancante, che i sei cicli siano coppie STANCE+SWING esatte della segmentazione, che nessun ciclo contenga WAIT (con rifiuto fail-closed), il floor WAIT all'80%, il determinismo, e che lo script **non importi torch, optim, backward, adapt_actor, subprocess o rollout_eval**.

`git status` conferma produzione, FSM v3, contatto, reward, guardie, config v3 e `tools/` intatti.

## 8. Cosa resta da decidere

1. Se approvare il LOCO a 6 fold con embargo 20.
2. Come trattare la perdita di 20 righe WAIT nel training del fold 0: accettarla come ora, oppure spostare l'inizio del ciclo di quel fold di 20 righe (validando 128 invece di 148 righe) per non intaccare il segmento.
3. Se accettare il gate di ricostruzione WAIT ai valori proposti.
4. Se le 590 righe mai validate siano un prezzo accettabile del validare cicli completi.
5. La regola di selezione del best epoch: la pooled-fold early stopping che avevo proposto vale su 6 fold come su 5, ma non l'ho ri-argomentata qui — dimmi se vuoi che la riesamini alla luce della nuova geometria.

Nessun fit avviato. Attendo la tua revisione.

## 9. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione in B0/B1. *(indirizzato)*
- **~~TODO-13~~** — **CHIUSO**: il gate a 0,05 per fold non è più su uno split la cui baseline è 0,20; con il LOCO la baseline peggiore è 0,0640.
- **TODO-14** — Nuovo: 590 righe su 1500 non entrano mai in validation sotto LOCO. *(nuovo)*
- **TODO-15** — Nuovo: il fold 0 perde 20 righe WAIT dal training per effetto dell'embargo. *(nuovo)*
