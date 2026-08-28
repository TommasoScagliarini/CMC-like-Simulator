# V26B B1R2 rev1 — revisione architettonica obbligatoria della diagnostica

**Data**: 2026-08-25
**Stato**: studio **diagnostico** rieseguito integralmente dopo la revisione richiesta.
**Nessun candidato materializzato o promosso, nessun rollout, nessuna collection, nessuna fase Markov, nessun DAgger, nessuna soglia toccata.**

Questo report **sostituisce** `2026-08-25_v26b_b1r2_diagnostica.md` (`cbb892fe…`), che resta agli atti come versione pre-revisione e **non va considerato valido nelle sue conclusioni Q3**. Il receipt pre-revisione `ebf0c8e8…` è conservato intatto; la revisione scrive un file nuovo, additivo.

Le quattro correzioni sono state applicate tutte. Due di esse non si sono limitate a cambiare il linguaggio: **hanno cambiato il risultato**. La tua obiezione metodologica era fondata, e portata fino in fondo rovescia la diagnosi del report precedente.

---

## 1. Esito delle quattro correzioni, in breve

| # | correzione | effetto |
|---|---|---|
| 1 | rinominata «sparsità del supporto / smoothness locale» | linguaggio; nessun numero cambia |
| 2 | **audit collisioni reali sui 23D effettivi** | **0 coppie a ogni soglia**: nessuna non-identificabilità esiste |
| 3 | **predicibilità del residuo solo cross-validated** | **R² out-of-fold ≤ 0,16, quasi ovunque negativo**: lo 0,9021 e lo 0,9996 erano overfitting |
| 4 | **riclassificazione strutturale** | **le code superano i gate**: l'errore è tutto nelle due teste, che sono cicli completi |

---

## 2. Correzione 1 — la misura è rinominata, e la sua portata ridotta

`ambiguity_analysis` non esiste più. La funzione è `support_sparsity_diagnostic`, la chiave di receipt è `q3_support_sparsity_and_local_smoothness`, e la sua interpretazione è una costante esplicita `NOT_AN_INFORMATION_FLOOR` che dice, testualmente, che **non** è un limite informativo e che un vicino a distanza normalizzata 0,4 è un punto lontano, non una collisione.

Ogni stratto ora riporta la **distanza** del vicino accanto al disaccordo, perché era proprio la loro separazione a rendere possibile la lettura sbagliata: un disaccordo di 0,43 a distanza 0,43 non dice nulla su cosa una funzione del supporto possa fare.

Il test verifica meccanicamente che ogni occorrenza della locuzione nel modulo sia una **negazione** (normalizzando gli spazi, così una menzione spezzata su più righe non sfugge alla scansione) e che nessuna chiave o dicitura `informational limit/floor` sopravviva.

## 3. Correzione 2 — audit collisioni: **non esiste alcuna collisione**

Esaustivo, nessun campionamento: tutte le **1 124 250** coppie non ordinate, nei 23D effettivi, sia in unità grezze sia standardizzate, stratificato per le 5 firme discrete esatte (colonne 17,18,19,20,21,24).

| soglia | coppie entro la soglia (grezzo) | coppie entro la soglia (standardizzato) |
|---|---|---|
| 1e-8 | **0** | **0** |
| 1e-6 | **0** | **0** |
| 1e-4 | **0** | **0** |
| 1e-3 | **0** | **0** |

E non di poco: la distanza minima fra due righe qualsiasi del dataset è **0,030914** (grezzo) e **0,075620** (standardizzato), cioè **≥ 31×** la soglia più larga. Vale in ogni singolo stratto di firma.

Distanza minima fra coppie con disaccordo target rilevante:

| query | grezzo | standardizzato |
|---|---|---|
| \|Δtarget\| > 0,02 | 0,030914 | 0,075620 |
| \|Δtarget\| > 0,05 | 0,030914 | 0,152916 |

**Conclusione**: sul dataset la mappa 23D→teacher è iniettiva con margine ampio. Non c'è nessuna evidenza di non-identificabilità, e quindi nessun limite informativo da invocare. La tesi Q3 del report precedente è **ritirata**, non attenuata.

## 4. Correzione 3 — residuo: R² out-of-fold, e il fit in-sample è rimosso

Ridge sui 23D standardizzati, valutata **solo su righe held-out**, 12 fold leave-one-segment-out con embargo 20 e scaler fit solo sul train: nessun fold mescola righe contigue dello stesso segmento. Il fit in-sample sul peak cluster **non è più calcolato** — è marcato in codice `DELIBERATELY NOT COMPUTED`, non semplicemente omesso dal report.

| gruppo | n | R² oof ginocchio | R² oof caviglia |
|---|---|---|---|
| tutte | 1500 | **−0,038** | **−0,160** |
| B1R1-uncovered startup/tails | 403 | **+0,160** | −0,364 |
| cicli coperti da B1R1 | 910 | −0,421 | +0,000 |
| cicli strutturalmente completi | 1218 | −0,046 | −0,156 |
| code | 95 | −0,368 | −1,128 |
| WAIT | 187 | −0,150 | −0,358 |

Lo sweep su α (0,01→100) non cambia il quadro: il massimo su tutti i gruppi e tutte le α resta sotto 0,20.

**Conclusione**: fuori campione il residuo **non è linearmente prevedibile** dal supporto. I valori 0,9021 e 0,9996 del report precedente erano artefatti di 23 regressori su poche decine di righe contigue, esattamente come avevi indicato, e non supportano alcuna conclusione.

## 5. Correzione 4 — riclassificazione, e il risultato che ne è uscito

La classificazione è **derivata dalle colonne FSM**, non scritta a mano: un segmento è un ciclo completo se attraversa stance piena e poi swing pieno **e si chiude sul tallone successivo** (la riga seguente esiste ed è in stance); è una coda se l'episodio finisce dentro di esso.

| segmento | righe | sequenza | chiuso da HS | classificazione |
|---|---|---|---|---|
| minus020 coda 489:499 | 11 | stance | no | coda |
| **nominal testa 0:163** | 164 | stance→swing | **sì** | **ciclo completo** |
| nominal coda 468:499 | 32 | stance→swing | no | coda |
| **plus020 testa 0:143** | 144 | stance→swing | **sì** | **ciclo completo** |
| plus020 coda 448:499 | 52 | stance→swing | no | coda |

Il disegno a 11 fold è quindi **8 cicli completi + 3 code** (6 da B1R1 + 2 teste, più 3 code). Copertura e gate invariati: 1313 righe non-WAIT held-out esattamente una volta, embargo 20, distanza temporale minima 21, nessuna firma mancante, 23D ovunque, scaler train-only, train minimo 1307.

### 5.1 Il gruppo da 403 nascondeva due popolazioni opposte

Ricostruzione (fit da B0 su tutte le 1500 righe, migliore epoca per gruppo; gate 0,02 / 0,03 / 0,15):

| gruppo | n | best RMSE agg | epoca | knee rmse/max | ankle rmse/max | gate |
|---|---|---|---|---|---|---|
| tutte | 1500 | 0,02979 | 396 | 0,03493 / 0,15591 | 0,02356 / 0,11063 | FAIL |
| coperte da B1R1 | 910 | 0,02386 | 394 | 0,02634 / 0,08757 | 0,02108 / 0,08808 | FAIL |
| B1R1-uncovered startup/tails | 403 | 0,03344 | 388 | 0,03612 / 0,11177 | 0,03052 / 0,10925 | FAIL |
| cicli strutturalmente completi | 1218 | 0,03203 | 396 | 0,03771 / 0,15591 | 0,02510 / 0,11063 | FAIL |
| **code** | **95** | **0,01470** | 359 | 0,01580 / 0,06200 | 0,01352 / 0,03125 | **PASS** |
| WAIT | 187 | 0,01763 | 396 | 0,01959 / 0,13874 | 0,01544 / 0,08201 | PASS |
| *(derivato)* teste 308 | 308 | 0,03568 | 259 | — | — | FAIL |

**Le code superano tutti e tre i gate di ricostruzione.** Sono il gruppo meglio ricostruito del dataset dopo WAIT. Chiamarle «cicli parziali» e mediarle con le teste produceva 0,04878 e attribuiva l'errore all'incompletezza — cioè esattamente alla parte che funziona.

Sull'attore B1R1 la separazione è ancora più netta:

| gruppo strutturale | n | RMSE agg | max abs |
|---|---|---|---|
| **teste (cicli completi)** | 308 | **0,05505** | **0,17049** |
| cicli coperti da B1R1 | 910 | 0,02723 | 0,09588 |
| WAIT | 187 | 0,02679 | 0,12686 |
| **code** | 95 | **0,01641** | 0,06433 |

Delle 25 righe con errore peggiore, **24 sono nelle due teste e nessuna in una coda** (la 25ª è WAIT). Tutte con label verificata corretta.

### 5.2 E la sparsità non spiega neanche questo

A parità di firma FSM (SWING / credit 0,5, n=738), separando per posizione strutturale:

| posizione | n | RMSE agg | distanza nn mediana |
|---|---|---|---|
| teste | 188 | **0,06575** | **0,3394** |
| coperte da B1R1 | 539 | 0,03115 | 0,3868 |
| code | 11 | **0,01152** | **2,1483** |

La relazione è **invertita**: le code sono 6× più isolate nel supporto e ricostruiscono 5,7× meglio. Stesso esito per l'extrapolazione: il **100%** delle righe di coda cade fuori dal p99 per-colonna dei cicli validati, contro il **49,4%** delle teste — e sono le code ad andare bene.

Per completezza la firma STANCE/credit-0 (n=120) vive **esclusivamente** nelle due teste (120 su 120): è la stance precedente al primo tallone, che nessun altro punto del dataset visita. È un buco di copertura reale, ma da solo non spiega l'errore, che si concentra nella porzione di **swing** delle teste.

## 6. Cosa resta valido dal report precedente

Invariati e riconfermati bit-per-bit dalla riesecuzione: la curva di ricostruzione sui gruppi già misurati (0,02979 @396 · 0,02386 @394 · 0,03344 @388 · 0,01763 @396), l'audit label/action (label ri-derivate **bit-identiche** alle cache pinnate, round-trip 2,22e-16, tutto in bounds, **nessun difetto**), l'esclusione dell'anchor (rapporto sui valori 2,12e-06, sui gradienti 1,04e-07) e della saturazione (0,00% layer 1, 0,11% layer 2 oltre |0,99|).

## 7. Conclusione — prudente

Il report precedente concludeva che l'errore era spiegato da un limite informativo sulle righe «parziali». **Quella conclusione è ritirata.** Sottoposta alle misure che avevi richiesto, non sopravvive: non ci sono collisioni, il residuo non è prevedibile fuori campione, e le righe «parziali» che dovevano essere il problema superano i gate.

Ciò che le misure dicono, e nulla di più: l'errore in eccesso è **localizzato nelle due teste di episodio**, che sono cicli strutturalmente completi mai messi in validation da B1R1. Ogni ipotesi informativa testata è **respinta o anti-correlata** con quella localizzazione:

- collisioni → nessuna, a nessuna soglia;
- difetti di label o semantica dell'azione → nessuno;
- sparsità locale del supporto → anti-correlata (le teste hanno i vicini più vicini e l'errore peggiore);
- extrapolazione fuori distribuzione → anti-correlata (le code sono al 100% fuori p99 e vanno meglio);
- struttura lineare residua → assente fuori campione;
- anchor e saturazione → già esclusi.

**Lo studio quindi non identifica la causa.** Resta come ipotesi non verificata — e la segnalo come tale, non come risultato — che le teste siano il transitorio iniziale di episodio, in cui lo stato SEA/motore parte da fermo e la FSM non ha ancora osservato alcun tallone, così che la relazione ingresso→uscita in quella regione differisca pur a parità di firma discreta. Verificarlo richiederebbe una misura che non ho eseguito e che non propongo di eseguire senza tua autorizzazione.

Due cautele esplicite: le code sono 95 righe in tre segmenti (11 / 32 / 52), quindi i loro numeri favorevoli vanno presi con la prudenza del campione piccolo; e la riclassificazione **non** allenta nulla — le 403 righe restano vincolanti nei gate finali esattamente come prima, come da tua decisione.

**Non avvio candidato, rollout, collection, DAgger o fit correttivo.** Non propongo di rilassare soglie.

## 8. Immutabilità verificata

| artefatto | SHA-256 | esito |
|---|---|---|
| B0 `module_state.pkl` | `aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f` | intatto |
| B1 receipt | `e7328ac2eb49787e905ef0919f06c540bdd2ce4ec30d290e711f8b0af42f53b1` | intatto |
| B1 `module_state.pkl` | `778ff748bd33ca658b62aae1b091be7b702242e9faabb8e0192daf7453e80a5b` | intatto |
| B1R1 receipt | `6a604ada500f6f51594500f596ba7a7797cd1350a110b9d0fe1807ee4d2cadbb` | intatto |
| B1R1 `module_state.pkl` | `9ffcdceb7f90f12cbcca0c151f8b667e3003ba4f48a124e630582d76cf0bd980` | intatto |
| receipt B1R2 pre-revisione | `ebf0c8e88762c3bb826754753def2d1980ff0c43cfdbd2108062221cf1cb114f` | intatto |

## 9. File

| file | SHA-256 |
|---|---|
| `v26b_b1r2_diagnostics.py` | `2f0093e808dfd75bd983cba4cd9b35e383cb0ff4212d4bdea256746fa6507cd6` |
| `test_v26b_b1r2_diagnostics.py` | `952291c8e2f88656baa5a73f086c874dd62b4287e20813619e6e8b430990c127` |
| `diagnostics/b1r2/v26b_b1r2_diagnostics_rev1.json` | `d0f1e5155d8273651f6055ca9a47601ad5f265720db2d8aca295dabefc84ff4c` |

**Comandi eseguiti**
```
python test_v26b_b1r2_diagnostics.py   -> PASS, 225 check (erano 42)
python v26b_b1r2_diagnostics.py --run  -> receipt rev1, additivo, no-clobber
```

Tutte le modifiche ai file esistenti sono state fatte in patch-style. `fit_masked` resta riusata invariata da `v26b_b_exec`; il test continua a verificare che il twin strumentato sia **bit-identico** alle epoche 1, 4, 9 e che le colonne mascherate e di clock restino esattamente zero a ogni epoca.

## 10. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — 590 righe mai validate sotto LOCO. *(riformulato: l'errore non è nelle 590 né nelle «parziali», ma nelle 308 righe delle due teste; le 95 righe di coda superano i gate)*
- **TODO-15** — Fold 0 perde 20 righe WAIT per l'embargo. *(chiuso per decisione)*
- **TODO-16** — Patience della curva pooled ferma a 42 stale su 60 per esaurimento budget. *(aperto)*
- **TODO-17** — Nuovo: conclusione Q3 «limite informativo» **ritirata**; causa dell'errore sulle teste **non identificata**, ipotesi transitorio di episodio non verificata. *(nuovo)*
